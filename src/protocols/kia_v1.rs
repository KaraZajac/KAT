//! Kia V1 decoder + encoder.
//!
//! Port of ProtoPirate `protocols/kia_v1.c` (HEAD 2026-10; since KAT's v1.3.0 port only the
//! displayed CRC changed — `instance->crc = crc` instead of `cnt_high << 4 | crc` — which KAT does
//! not display). Flipper-ARF's `kia_v1.c` builds the same upload and differs only in resetting the
//! decoder on an out-of-tolerance pulse; on every IMPORTS capture both C decoders agree, and on
//! random C-encoded frames ProtoPirate's decodes are a superset, so KAT follows ProtoPirate, with
//! doctor-firmware's two changes to the same decoder: te_delta 300 and only CRC-valid frames
//! reported (see "Timing tolerance").
//!
//! Air format (315 / 433.92 MHz, AM): Manchester, te_short 800µs / te_long 1600µs (te_delta 300,
//! strict `<`; ProtoPirate 200). Preamble: LOW/HIGH long pairs; >70 of them followed by a short LOW is the sync,
//! which also stands for the first data bit (`1`, the serial's MSB). 57-bit frame:
//! `serial32 << 24 | button8 << 16 | cnt_lo8 << 8 | cnt_hi4 << 4 | crc4`, counter 12 bits.
//! CRC-4: XOR of all nibbles of serial, button, cnt_lo and cnt_hi, plus 1 (`kia_v1_crc4(.., 7, 1)`);
//! ProtoPirate reports it (`crc_check`) but emits regardless; doctor-firmware, and KAT, emit only
//! frames whose CRC holds.
//!
//! Decoder (`kia_protocol_decoder_v1_feed`): Flipper's Manchester machine (common.rs) with
//! ProtoPirate's `pp_manchester_event` mapping (`level ? ShortLow : ShortHigh`), state reset to
//! Mid1 on entering the preamble. In the data phase an out-of-tolerance pulse is ignored (the
//! state machine keeps going), and the frame is reported on the pulse completing bit 57.
//! (KAT's earlier port used a hand-written Manchester table that mis-decoded the real captures:
//! wrong serial, CRC failing. The C oracle and this port agree on every IMPORTS capture.)
//!
//! **Timing tolerance** — doctor-firmware's copy of this decoder widens te_delta from 200 to 300 µs
//! and drops frames whose CRC fails; KAT does both. On KAT's simulated radio (synthetic IQ through
//! the receive pipeline; the 35 real frames of the IMPORTS captures and the encoder's frame) 300
//! decodes 87 % of frames with SNR 4 dB, 60 µs pulse jitter and a ±10 % fob clock, where 200
//! decodes 29 %, 97 % instead of 25 % with 75 µs jitter, and frames with a fob clock 10–15 % off
//! that 200 never reads; it loses none. Under heavy jitter both windows also read corrupted frames
//! (wrong data); 300 reads fewer of them (67 against 72 per run), and fewer pass the 4-bit CRC (1
//! against 8), which the CRC check then drops with the rest. Every IMPORTS frame passes its CRC and
//! decodes as before; ProtoPirate's and doctor-firmware's C agree with the two settings on 240
//! edge-case recordings each.
//!
//! Encoder (`kia_protocol_encoder_v1_get_upload`): 3 bursts separated by LOW 25 000µs, each
//! 90 × (LOW 1600, HIGH 1600), LOW 800, then frame bits 55..0 as Manchester (1 → HIGH 800 +
//! LOW 800, 0 → LOW 800 + HIGH 800); adjacent equal levels merged (identical on air). One upload
//! per call; KAT sends it 3 times (`tx_copies.rs`: ProtoPirate's TX loop repeats it 10 times,
//! doctor-firmware's 3; a key file's `Repeat` line sets the count). Like ProtoPirate's emulate scene, each press sends the
//! counter plus one; KAT's generic button maps to the C's names: Lock → 1 ("Close"), Unlock → 2
//! ("Open"), Trunk → 3 ("Boot"), Panic → 4 (emulate D-pad Left); anything else keeps the captured
//! button.
//!
//! Key files (`kia_protocol_decoder_v1_serialize` / `_deserialize`): Protocol "Kia V1", Bit 57,
//! Key = the frame, then Serial, Btn, Cnt (decimal; `pp_serialize_fields`). Flipper-ARF's
//! "KIA/HYU V1" files carry the same lines. Loading checks Bit = 57 and decodes the Key as the
//! decoder does. ProtoPirate's encoder transmits from Serial/Btn/Cnt (CRC recomputed): when a
//! file's fields differ from its Key's, KAT rebuilds the frame from them the same way.

use super::common::{ManchesterEvent, ManchesterState};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Kia V1",
    decoder: "Kia V1",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "Kia/Hyundai",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// ProtoPirate kia_v1.c names (Close/Open/Boot) and the emulate D-pad's Panic; Flipper-ARF uses the same codes for Kia V2 and V6.
pub(super) fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x1 => "Lock",
        0x2 => "Unlock",
        0x3 => "Trunk",
        0x4 => "Panic",
        _ => "Other",
    })
}

const TE_SHORT: u32 = 800;
const TE_LONG: u32 = 1600;
/// doctor-firmware's (ProtoPirate: 200); see "Timing tolerance".
const TE_DELTA: u32 = 300;
const MIN_COUNT_BIT: usize = 57;
const TOTAL_BURSTS: usize = 3;
const INTER_BURST_GAP_US: u32 = 25_000;
const HEADER_PULSES: usize = 90;

/// Decoder states (`KiaV1DecoderStep`).
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    CheckPreamble,
    DecodeData,
}

/// Kia V1 decoder.
pub struct KiaV1Decoder {
    step: DecoderStep,
    te_last: u32,
    header_count: u16,
    decode_data: u64,
    /// `SubGhzBlockDecoder.decode_count_bit` is a `uint8_t`.
    decode_count_bit: u8,
    manchester: ManchesterState,
}

#[inline]
fn is_short(d: u32) -> bool {
    duration_diff!(d, TE_SHORT) < TE_DELTA
}

#[inline]
fn is_long(d: u32) -> bool {
    duration_diff!(d, TE_LONG) < TE_DELTA
}

/// `kia_v1_crc4`: XOR of both nibbles of every byte, plus `offset`, low nibble.
fn crc4(bytes: &[u8], offset: u8) -> u8 {
    let crc = bytes.iter().fold(0u8, |acc, &b| acc ^ (b & 0x0F) ^ (b >> 4));
    crc.wrapping_add(offset) & 0x0F
}

/// The CRC of `kia_v1_check_remote_controller` / the encoder.
fn frame_crc(serial: u32, button: u8, counter: u16) -> u8 {
    let s = serial.to_be_bytes();
    crc4(&[s[0], s[1], s[2], s[3], button, counter as u8, ((counter >> 8) & 0x0F) as u8], 1)
}

/// The 57-bit frame for the fields (`generic.data` in the encoder).
fn build_frame(serial: u32, button: u8, counter: u16) -> u64 {
    ((serial as u64) << 24)
        | ((button as u64) << 16)
        | (((counter & 0xFF) as u64) << 8)
        | ((((counter >> 8) & 0x0F) as u64) << 4)
        | frame_crc(serial, button, counter) as u64
}

/// `kia_v1_check_remote_controller`: serial, button, counter, CRC check of a frame.
fn parse_frame(data: u64) -> DecodedSignal {
    let serial = (data >> 24) as u32;
    let button = (data >> 16) as u8;
    let counter = ((((data >> 4) & 0x0F) << 8) | ((data >> 8) & 0xFF)) as u16;
    DecodedSignal {
        serial: Some(serial),
        button: Some(button),
        counter: Some(counter),
        crc_valid: frame_crc(serial, button, counter) == (data & 0x0F) as u8,
        data,
        data_count_bit: MIN_COUNT_BIT,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

impl KiaV1Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            te_last: 0,
            header_count: 0,
            decode_data: 0,
            decode_count_bit: 0,
            manchester: ManchesterState::Mid1,
        }
    }
}

impl ProtocolDecoder for KiaV1Decoder {
    fn name(&self) -> &'static str {
        "Kia V1"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: MIN_COUNT_BIT,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        // kia_protocol_decoder_v1_reset only rewinds the step; the rest is re-initialised on
        // entering the preamble.
        self.step = DecoderStep::Reset;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::Reset => {
                if level && is_long(duration) {
                    self.step = DecoderStep::CheckPreamble;
                    self.te_last = duration;
                    self.header_count = 0;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.manchester = ManchesterState::Mid1;
                }
            }
            DecoderStep::CheckPreamble => {
                if !level {
                    if is_long(duration) && is_long(self.te_last) {
                        self.header_count = self.header_count.wrapping_add(1);
                        self.te_last = duration;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                }
                // Not an else: a short LOW after >70 pairs overrides the Reset above.
                if self.header_count > 70 && !level && is_short(duration) && is_long(self.te_last) {
                    self.decode_count_bit = 1;
                    self.decode_data = (self.decode_data << 1) | 1;
                    self.decode_count_bit += 1;
                    self.header_count = 0;
                    self.step = DecoderStep::DecodeData;
                }
            }
            DecoderStep::DecodeData => {
                // pp_manchester_event: level ? Short/LongLow : Short/LongHigh; others ignored.
                let event = if is_short(duration) {
                    Some(ManchesterEvent::from_half_bit(!level, false))
                } else if is_long(duration) {
                    Some(ManchesterEvent::from_half_bit(!level, true))
                } else {
                    None
                };
                if let Some(bit) = event.and_then(|e| self.manchester.advance(e)) {
                    self.decode_data = (self.decode_data << 1) | bit as u64;
                    self.decode_count_bit = self.decode_count_bit.wrapping_add(1);
                }
                if self.decode_count_bit as usize == MIN_COUNT_BIT {
                    let frame = parse_frame(self.decode_data);
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.step = DecoderStep::Reset;
                    // doctor-firmware: `if(instance->crc_check && instance->base.callback)`
                    return frame.crc_valid.then_some(frame);
                }
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        let parsed = parse_frame(decoded.data);
        let serial = decoded.serial.or(parsed.serial)?;
        let captured = decoded.button.or(parsed.button).unwrap_or(0);
        let counter = decoded.counter.or(parsed.counter).unwrap_or(0).wrapping_add(1) & 0x0FFF;
        let button = match button {
            0x01 => 0x1,
            0x02 => 0x2,
            0x04 => 0x3,
            0x08 => 0x4,
            _ => captured,
        };
        Some(build_upload(build_frame(serial, button, counter)))
    }

    /// `kia_protocol_decoder_v1_deserialize` (Bit must be 57) + the encoder's use of
    /// Serial/Btn/Cnt: a file whose fields differ from its Key's is rebuilt from them (see the
    /// module docs).
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != MIN_COUNT_BIT {
            return None;
        }
        let data = key.key_u64()?;
        let d = parse_frame(data);
        let (serial, button, counter) = (key.u32_field("Serial"), key.u32_field("Btn"), key.u32_field("Cnt"));
        if serial.is_none() && button.is_none() && counter.is_none() {
            return Some(d);
        }
        let fields = (
            serial.or(d.serial)?,
            button.map_or(d.button, |b| Some(b as u8))?,
            counter.map_or(d.counter, |c| Some((c & 0x0FFF) as u16))?,
        );
        if fields == (d.serial?, d.button?, d.counter?) {
            return Some(d);
        }
        Some(parse_frame(build_frame(fields.0, fields.1, fields.2)))
    }

    /// `kia_protocol_decoder_v1_serialize`: "Kia V1", 57 bits, Key, then Serial, Btn, Cnt.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        let f = parse_frame(decoded.data);
        let mut key = FlipperKey::for_protocol("Kia V1", MIN_COUNT_BIT, decoded.data);
        key.push_u32("Serial", f.serial?);
        key.push_u32("Btn", f.button?.into());
        key.push_u32("Cnt", f.counter?.into());
        Some(key)
    }
}

impl Default for KiaV1Decoder {
    fn default() -> Self {
        Self::new()
    }
}

/// Append one level, merging with the previous entry when the level repeats (identical on air).
fn push(out: &mut Vec<LevelDuration>, level: bool, duration: u32) {
    match out.last_mut() {
        Some(last) if last.level == level => last.duration_us += duration,
        _ => out.push(LevelDuration::new(level, duration)),
    }
}

/// `kia_protocol_encoder_v1_get_upload` for a frame.
fn build_upload(frame: u64) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(TOTAL_BURSTS * (HEADER_PULSES * 2 + 1 + 2 * (MIN_COUNT_BIT - 1)));
    for burst in 0..TOTAL_BURSTS {
        if burst > 0 {
            push(&mut out, false, INTER_BURST_GAP_US);
        }
        for _ in 0..HEADER_PULSES {
            push(&mut out, false, TE_LONG);
            push(&mut out, true, TE_LONG);
        }
        push(&mut out, false, TE_SHORT);
        for bit in (0..MIN_COUNT_BIT - 1).rev() {
            let one = (frame >> bit) & 1 != 0;
            push(&mut out, one, TE_SHORT);
            push(&mut out, !one, TE_SHORT);
        }
    }
    out
}

#[cfg(test)]
pub(super) mod tests {
    use super::*;

    fn feed_all(wire: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = KiaV1Decoder::new();
        wire.iter().filter_map(|p| dec.feed(p.level, p.duration_us)).collect()
    }

    /// The real `IMPORTS/KIA/KIA V1 N1 RAW.sub` frames as the C (`kia_v1.c` on the host) decodes
    /// them: key, serial, button, counter (CRC OK).
    const C_CAPTURE: &[(u64, u32, u8, u16)] = &[
        (0x00B4_4470_6E01_910A, 0xB444_706E, 0x01, 0x091),
        (0x00B4_4470_6E01_950E, 0xB444_706E, 0x01, 0x095),
        (0x00B4_4470_6E02_9A02, 0xB444_706E, 0x02, 0x09A),
    ];

    #[test]
    fn capture_frames_round_trip() {
        for &(key, serial, button, counter) in C_CAPTURE {
            let d = parse_frame(key);
            assert_eq!((d.serial, d.button, d.counter, d.crc_valid), (Some(serial), Some(button), Some(counter), true));
            assert_eq!(build_frame(serial, button, counter), key);
            let got = feed_all(&build_upload(key));
            assert_eq!(got.len(), TOTAL_BURSTS);
            assert!(got.iter().all(|g| g.data == key && g.crc_valid));
        }
    }

    /// encode(): counter + 1 (12 bits) and KAT's generic button mapped to the C's codes.
    #[test]
    fn encode_bumps_counter_and_maps_buttons() {
        for (generic, want) in [(0x01u8, 0x1u8), (0x02, 0x2), (0x04, 0x3), (0x08, 0x4), (0x40, 0x55)] {
            let src = parse_frame(build_frame(0xB444_706E, 0x55, 0xFFF));
            let got = feed_all(&KiaV1Decoder::new().encode(&src, generic).unwrap());
            let d = &got[0];
            assert_eq!((d.serial, d.button, d.counter), (Some(0xB444_706E), Some(want), Some(0)));
            assert!(d.crc_valid);
        }
    }

    /// Key files (ProtoPirate `kia_protocol_decoder_v1_serialize` / `_deserialize`): the capture's
    /// decodes round-trip with the C's lines, a file with edited fields is rebuilt from them like
    /// the C encoder does, and TX from a loaded key decodes back.
    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::{assert_round_trip, key};
        let dec = KiaV1Decoder::new();
        for &(frame, ..) in C_CAPTURE {
            for d in feed_all(&build_upload(frame)) {
                assert_round_trip(&dec, &d);
            }
        }
        let d = feed_all(&build_upload(C_CAPTURE[0].0)).remove(0);
        let text = assert_round_trip(&dec, &d);
        assert!(
            text.ends_with("Protocol: Kia V1\nBit: 57\nKey: 00 B4 44 70 6E 01 91 0A\nSerial: 3024384110\nBtn: 1\nCnt: 145\n"),
            "{text}"
        );
        // Flipper-ARF ("KIA/HYU V1", same lines) and a bare key both load; a wrong Bit does not.
        let arf = key("Protocol: KIA/HYU V1\nBit: 57\nKey: 00 B4 44 70 6E 01 95 0E\n");
        let l = dec.from_flipper_key(&arf).unwrap();
        assert_eq!((l.serial, l.button, l.counter, l.crc_valid), (Some(0xB444_706E), Some(0x01), Some(0x095), true));
        assert!(dec.from_flipper_key(&key("Protocol: Kia V1\nBit: 56\nKey: 00 B4 44 70 6E 01 95 0E\n")).is_none());
        // Edited Cnt: rebuilt (CRC recomputed), as ProtoPirate's encoder transmits it.
        let edited = key(&text.replace("Cnt: 145", "Cnt: 2000"));
        let e = dec.from_flipper_key(&edited).unwrap();
        assert_eq!((e.serial, e.button, e.counter, e.crc_valid), (Some(0xB444_706E), Some(0x01), Some(2000), true));
        // TX from the loaded key: counter + 1.
        let tx = feed_all(&dec.encode(&l, 0x02).unwrap());
        assert_eq!((tx[0].serial, tx[0].button, tx[0].counter), (Some(0xB444_706E), Some(0x2), Some(0x096)));
    }

    /// `kia_v1`'s and the other modules' oracle vectors: write `<name>.sub` (a Flipper RAW file of
    /// `upload` sent twice, 20 ms apart) and `<name>.kat.txt` (`key` with that Frequency/Preset)
    /// into `dir`.
    pub(in crate::protocols) fn write_key_vector(
        dir: &std::path::Path,
        name: &str,
        frequency: u32,
        preset: &str,
        upload: &[LevelDuration],
        key: &FlipperKey,
    ) {
        crate::protocols::flipper_key::testing::write_vector(dir, name, frequency, preset, upload, key);
    }

    /// Oracle vectors (`#[ignore]`d): `KAT_KEY_VECTORS=<dir> cargo test kia_v1::tests::key_vectors
    /// -- --ignored` writes `kia_v1_<n>.sub` / `.kat.txt` (see [write_key_vector]).
    #[test]
    #[ignore]
    fn key_vectors() {
        let Some(dir) = std::env::var_os("KAT_KEY_VECTORS") else { return };
        let dir = std::path::PathBuf::from(dir);
        for (n, &(frame, ..)) in C_CAPTURE.iter().take(2).enumerate() {
            let upload = build_upload(frame);
            let d = feed_all(&upload).remove(0);
            let key = KiaV1Decoder::new().to_flipper_key(&d).unwrap();
            write_key_vector(&dir, &format!("kia_v1_{n}"), 433_920_000, "FuriHalSubGhzPresetOok650Async", &upload, &key);
        }
    }
}
