//! Kia V7 protocol decoder/encoder
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/kia_v7.c` and `kia_v7.h`.
//! Manchester 250/500µs, 64 bits, FM. The decoded 64-bit word is bit-inverted (`~data`); a valid
//! frame has a fixed high byte 0x4C and a CRC8 (poly 0x7F, init 0x4C) over bytes 0..7. Emission is
//! gated on header + CRC, so Kia V7 is strongly validated and will not false-match.
//!
//! Decoder steps: Reset → Preamble (short pairs, ≥16) → SyncLow → Data. The preamble→sync transition
//! preloads four seed bits (1,0,1,1 = the inverted header's top nibble 0xB) before collecting the
//! remaining 60 Manchester bits. Encoder supported.
//!
//! **Manchester**: the shared Flipper `manchester_advance` port in `common.rs`. Event mapping is
//! `level ? ShortHigh : ShortLow` (kia_v7.c:463 `(level & 1) << 1`) and `level ? LongHigh :
//! LongLow` (kia_v7.c:465); out-of-range pulses are ignored (no Manchester call). The state is reset
//! to Mid1 when the preamble starts (kia_v7.c:411) and after every 64-bit candidate (kia_v7.c:499,
//! 508). With the on-air 1 = HIGH,LOW encoding the decoded bits are the inverse of the key, hence
//! the `~data` and the inverted seed nibble.
//!
//! **Encoder** (kia_v7_encoder_get_upload): two copies of the frame, the first behind a 319-pair
//! (0x13F) short preamble and the second behind a 15-pair (0x0F) one; each frame is preamble, a
//! short HIGH, 64 Manchester bits (1 = HIGH,LOW) MSB first, a short HIGH and a 2000µs LOW tail.
//! Like ProtoPirate's emulate scene each press sends counter + 1 (16 bits); KAT's generic button
//! maps to the emulate D-pad codes (Lock 1, Unlock 2, Trunk 3, Panic 8), anything else keeps the
//! captured button.
//!
//! **Flipper-ARF** `kia_v7.c`: same decoder (C oracle: identical on 600 random frames × 2 copies
//! and on IMPORTS); its encoder is ProtoPirate's pre-2026-09-14 one (a single 319-pair frame, the
//! second pass overwriting the first), which ProtoPirate replaced with the two-frame upload above.
//!
//! **Key files** (`kia_protocol_decoder_v7_serialize` / `_deserialize`): "Kia V7", Bit 64, Key, then
//! Serial, Btn, Cnt, `Repeat: 3` and `Disp: Kia V7 - <button>`. Loading rebuilds the key from
//! Serial/Btn/Cnt when present (CRC recomputed), as the C does; a key without the 0x4C header or
//! with a bad CRC and no fields does not load (KAT's decoder would not report it).

use super::{ProtocolDecoder, ProtocolTiming, DecodedSignal, FlipperKey};
use super::common::{crc8, ManchesterEvent, ManchesterState};
use crate::radio::demodulator::LevelDuration;
use crate::duration_diff;

use super::info::prelude::*;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Kia V7",
    decoder: "Kia V7",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::FM,
    encryption: "Rolling Code",
    make: "Kia/Hyundai",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&FM476],
};

/// The references' names (`kia_v7.c`: 1 Lock, 2 Unlock, 3 and 8 Trunk, else "??"); KAT's
/// generic 1/2 coincide, and the encoder's Panic → 0x08 is the ProtoPirate D-pad's.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x1 => "Lock",
        0x2 => "Unlock",
        0x3 | 0x8 => "Trunk",
        _ => "Other",
    })
}

const TE_SHORT: u32 = 250;
const TE_LONG: u32 = 500;
const TE_DELTA: u32 = 100;
const KEY_BITS: usize = 64;
const HEADER: u8 = 0x4C;
const PREAMBLE_MIN_PAIRS: u16 = 16;
const TAIL_GAP_US: u32 = 2000;
/// Preamble pairs of the two frames in the C upload (KIA_V7_PREAMBLE_PAIRS, KIA_V7_TAIL_PREAMBLE_PAIRS).
const TX_PREAMBLE_PAIRS: [usize; 2] = [0x13F, 0x0F];

#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Preamble,
    SyncLow,
    Data,
}

pub struct KiaV7Decoder {
    step: DecoderStep,
    manchester_state: ManchesterState,
    te_last: u32,
    preamble_count: u16,
    decode_data: u64,
    decode_count_bit: usize,
}

impl KiaV7Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            manchester_state: ManchesterState::Mid1,
            te_last: 0,
            preamble_count: 0,
            decode_data: 0,
            decode_count_bit: 0,
        }
    }

    fn is_short(d: u32) -> bool {
        duration_diff!(d, TE_SHORT) < TE_DELTA
    }
    fn is_long(d: u32) -> bool {
        duration_diff!(d, TE_LONG) < TE_DELTA
    }

    fn add_bit(&mut self, bit: bool) {
        self.decode_data = (self.decode_data << 1) | (bit as u64);
        self.decode_count_bit += 1;
    }

    /// Decode the 8 plaintext bytes of the (already inverted) key into fields.
    /// Returns (serial, button, counter, crc_valid).
    fn decode_key(data: u64) -> (u32, u8, u16, bool) {
        let bytes = data.to_be_bytes();
        let serial = (((bytes[3] as u32) << 20)
            | ((bytes[4] as u32) << 12)
            | ((bytes[5] as u32) << 4)
            | ((bytes[6] as u32) >> 4))
            & 0x0FFF_FFFF;
        let counter = ((bytes[1] as u16) << 8) | bytes[2] as u16;
        let button = bytes[6] & 0x0F;
        let crc_calc = crc8(&bytes[0..7], 0x7F, 0x4C);
        let crc_valid = crc_calc == bytes[7];
        (serial, button, counter, crc_valid)
    }

    /// The decode of a 64-bit key with the 0x4C header: reported only when its CRC checks (the C's
    /// emission gate).
    fn frame_signal(key: u64) -> Option<DecodedSignal> {
        let (serial, button, counter, crc_valid) = Self::decode_key(key);
        crc_valid.then_some(DecodedSignal {
            serial: Some(serial),
            button: Some(button),
            counter: Some(counter),
            crc_valid: true,
            data: key,
            data_count_bit: KEY_BITS,
            encoder_capable: true,
            extra: None,
            protocol_display_name: None,
        })
    }

    /// Rebuild the 64-bit key from fields (matches kia_v7_encode_key).
    fn encode_key(serial: u32, button: u8, counter: u16) -> u64 {
        let serial = serial & 0x0FFF_FFFF;
        let button = button & 0x0F;
        let mut bytes = [0u8; 8];
        bytes[0] = HEADER;
        bytes[1] = (counter >> 8) as u8;
        bytes[2] = counter as u8;
        bytes[3] = (serial >> 20) as u8;
        bytes[4] = (serial >> 12) as u8;
        bytes[5] = (serial >> 4) as u8;
        bytes[6] = (((serial & 0x0F) as u8) << 4) | button;
        bytes[7] = crc8(&bytes[0..7], 0x7F, 0x4C);
        u64::from_be_bytes(bytes)
    }

    /// Map KAT's generic button command to the Kia V7 code (ProtoPirate's emulate D-pad);
    /// anything else keeps the captured button.
    fn map_button(button: u8, captured: u8) -> u8 {
        match button {
            0x01 => 0x01, // Lock
            0x02 => 0x02, // Unlock
            0x04 => 0x03, // Trunk
            0x08 => 0x08, // Panic
            _ => captured,
        }
    }

    fn enc_add_level(signal: &mut Vec<LevelDuration>, level: bool, duration: u32) {
        if let Some(last) = signal.last_mut() {
            if last.level == level {
                *last = LevelDuration::new(level, last.duration_us + duration);
                return;
            }
        }
        signal.push(LevelDuration::new(level, duration));
    }
}

impl ProtocolDecoder for KiaV7Decoder {
    fn name(&self) -> &'static str {
        "Kia V7"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: KEY_BITS,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.manchester_state.advance(ManchesterEvent::Reset);
        self.te_last = 0;
        self.preamble_count = 0;
        self.decode_data = 0;
        self.decode_count_bit = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::Reset => {
                if level && Self::is_short(duration) {
                    self.step = DecoderStep::Preamble;
                    self.te_last = duration;
                    self.preamble_count = 0;
                    self.manchester_state.advance(ManchesterEvent::Reset);
                }
            }

            DecoderStep::Preamble => {
                if level {
                    if Self::is_long(duration) && Self::is_short(self.te_last) {
                        if self.preamble_count > (PREAMBLE_MIN_PAIRS - 1) {
                            self.decode_data = 0;
                            self.decode_count_bit = 0;
                            self.preamble_count = 0;
                            // Seed the inverted-header top nibble (1,0,1,1).
                            self.add_bit(true);
                            self.add_bit(false);
                            self.add_bit(true);
                            self.add_bit(true);
                            self.te_last = duration;
                            self.step = DecoderStep::SyncLow;
                        } else {
                            self.step = DecoderStep::Reset;
                        }
                    } else if Self::is_short(duration) {
                        self.te_last = duration;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else if Self::is_short(duration) && Self::is_short(self.te_last) {
                    self.preamble_count += 1;
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::SyncLow => {
                if !level && Self::is_short(duration) && Self::is_long(self.te_last) {
                    self.te_last = duration;
                    self.step = DecoderStep::Data;
                }
            }

            DecoderStep::Data => {
                // kia_v7.c:462-476: level ? ShortHigh : ShortLow (same for long); other durations
                // are skipped without touching the Manchester state.
                let event = if Self::is_short(duration) {
                    Some(ManchesterEvent::from_half_bit(level, false))
                } else if Self::is_long(duration) {
                    Some(ManchesterEvent::from_half_bit(level, true))
                } else {
                    None
                };

                if let Some(ev) = event {
                    if let Some(bit) = self.manchester_state.advance(ev) {
                        self.add_bit(bit);
                    }
                }

                if self.decode_count_bit == KEY_BITS {
                    let candidate = !self.decode_data;
                    let hdr = (candidate >> 56) as u8;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.step = DecoderStep::Reset;
                    self.manchester_state.advance(ManchesterEvent::Reset);

                    if hdr == HEADER {
                        return Self::frame_signal(candidate);
                    }
                }
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        let serial = decoded.serial?;
        let counter = decoded.counter.unwrap_or(0).wrapping_add(1);
        let button = Self::map_button(button, decoded.button.unwrap_or(0));
        Some(Self::build_upload(Self::encode_key(serial, button, counter)))
    }

    /// Key files: `kia_protocol_decoder_v7_deserialize` — Bit 64; when the file has Serial/Btn/Cnt
    /// the key is rebuilt from them (header byte kept, CRC recomputed), as the C does. KAT reports
    /// only what its decoder reports: a 0x4C header and a valid CRC (else None).
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != KEY_BITS {
            return None;
        }
        let mut data = key.key_u64()?;
        if (data >> 56) as u8 != HEADER {
            return None;
        }
        let (serial, button, counter) = (key.u32_field("Serial"), key.u32_field("Btn"), key.u32_field("Cnt"));
        if serial.is_some() || button.is_some() || counter.is_some() {
            let (s, b, c, _) = Self::decode_key(data);
            data = Self::encode_key(
                serial.unwrap_or(s),
                button.map_or(b, |b| b as u8),
                counter.map_or(c, |c| c as u16),
            );
        }
        Self::frame_signal(data)
    }

    /// Key files: `kia_protocol_decoder_v7_serialize` — "Kia V7", 64 bits, Key, then Serial, Btn,
    /// Cnt (`pp_serialize_fields`), `Repeat: 3` and ProtoPirate's display line
    /// (`Disp: Kia V7 - <button>`, `pp_write_display`).
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != KEY_BITS {
            return None;
        }
        let (serial, button, counter, _) = Self::decode_key(decoded.data);
        let mut key = FlipperKey::for_protocol("Kia V7", KEY_BITS, decoded.data);
        key.push_u32("Serial", serial);
        key.push_u32("Btn", button.into());
        key.push_u32("Cnt", counter.into());
        key.push_u32("Repeat", 3);
        // kia_v7_get_button_name
        let label = match button {
            0x01 => "Lock",
            0x02 => "Unlock",
            0x03 | 0x08 => "Trunk",
            _ => "??",
        };
        key.push("Disp", format!("Kia V7 - {label}"));
        Some(key)
    }
}

impl KiaV7Decoder {
    /// `kia_v7_encoder_get_upload` for a key.
    fn build_upload(key: u64) -> Vec<LevelDuration> {
        let pairs: usize = TX_PREAMBLE_PAIRS.iter().sum();
        let mut signal = Vec::with_capacity(pairs * 2 + TX_PREAMBLE_PAIRS.len() * (KEY_BITS * 2 + 3));
        // kia_v7_encoder_append_frame, once per preamble length.
        for preamble_pairs in TX_PREAMBLE_PAIRS {
            // Preamble: alternating short pulses.
            for _ in 0..preamble_pairs {
                Self::enc_add_level(&mut signal, true, TE_SHORT);
                Self::enc_add_level(&mut signal, false, TE_SHORT);
            }
            // Standalone high short. With the 0x4C header (first bit 0) the long sync HIGH the
            // decoder waits for is the second half of bit 63 merged with the first half of bit 62.
            Self::enc_add_level(&mut signal, true, TE_SHORT);
            // Manchester data, MSB first: bit 1 → (H,L), bit 0 → (L,H).
            for bit in (0..KEY_BITS).rev() {
                let value = (key >> bit) & 1 != 0;
                if value {
                    Self::enc_add_level(&mut signal, true, TE_SHORT);
                    Self::enc_add_level(&mut signal, false, TE_SHORT);
                } else {
                    Self::enc_add_level(&mut signal, false, TE_SHORT);
                    Self::enc_add_level(&mut signal, true, TE_SHORT);
                }
            }
            // Trailing high short + tail gap.
            Self::enc_add_level(&mut signal, true, TE_SHORT);
            Self::enc_add_level(&mut signal, false, TAIL_GAP_US);
        }
        signal
    }
}

impl Default for KiaV7Decoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn feed_all(signal: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = KiaV7Decoder::new();
        signal.iter().filter_map(|p| dec.feed(p.level, p.duration_us)).collect()
    }

    fn sample() -> DecodedSignal {
        let key = KiaV7Decoder::encode_key(0x0ABC_DEF1, 0x01, 0x1234);
        let (serial, button, counter, crc_valid) = KiaV7Decoder::decode_key(key);
        assert!(crc_valid);
        DecodedSignal {
            serial: Some(serial),
            button: Some(button),
            counter: Some(counter),
            crc_valid,
            data: key,
            data_count_bit: KEY_BITS,
            encoder_capable: true,
            extra: None,
            protocol_display_name: None,
        }
    }

    #[test]
    fn key_fields_round_trip() {
        let key = KiaV7Decoder::encode_key(0x0ABC_DEF1, 0x02, 0xBEEF);
        assert_eq!((key >> 56) as u8, HEADER);
        assert_eq!(KiaV7Decoder::decode_key(key), (0x0ABC_DEF1, 0x02, 0xBEEF, true));
    }

    /// Encoder → decoder through the full waveform: both frames of the C upload decode (the
    /// second one behind its 15-pair preamble) with the requested button.
    #[test]
    fn encode_decode_roundtrip() {
        for (button, code) in [(0x01u8, 0x01u8), (0x02, 0x02), (0x04, 0x03), (0x08, 0x08), (0x40, 0x01)] {
            let upload = KiaV7Decoder::new().encode(&sample(), button).unwrap();
            let out = feed_all(&upload);
            assert_eq!(out.len(), TX_PREAMBLE_PAIRS.len(), "button {button:#04x}");
            for d in &out {
                assert!(d.crc_valid);
                assert_eq!(d.serial, Some(0x0ABC_DEF1));
                assert_eq!(d.counter, Some(0x1235), "fresh press: counter + 1");
                assert_eq!(d.button, Some(code));
                assert_eq!(d.data, KiaV7Decoder::encode_key(0x0ABC_DEF1, code, 0x1235));
            }
        }
    }

    /// The upload has the C's two preambles (319 and 15 short pairs).
    #[test]
    fn upload_has_reference_preambles() {
        let upload = KiaV7Decoder::new().encode(&sample(), 0x01).unwrap();
        let mut runs = Vec::new();
        let mut run = 0;
        for p in &upload {
            if p.duration_us == TE_SHORT {
                run += 1;
            } else {
                if run > 20 {
                    runs.push(run);
                }
                run = 0;
            }
        }
        // 2 * pairs + the standalone HIGH + the first data half (bit 63 = 0 → LOW short).
        assert_eq!(runs, vec![2 * 0x13F + 2, 2 * 0x0F + 2]);
    }

    /// One frame for an arbitrary 64-bit key (no CRC recomputation): 40 short pairs, the
    /// standalone HIGH, the key MSB first (1 = HIGH,LOW), trailing HIGH and tail.
    fn frame_wave(key: u64) -> Vec<LevelDuration> {
        let mut wave = Vec::new();
        for _ in 0..40 {
            KiaV7Decoder::enc_add_level(&mut wave, true, TE_SHORT);
            KiaV7Decoder::enc_add_level(&mut wave, false, TE_SHORT);
        }
        KiaV7Decoder::enc_add_level(&mut wave, true, TE_SHORT);
        for bit in (0..KEY_BITS).rev() {
            let v = (key >> bit) & 1 != 0;
            KiaV7Decoder::enc_add_level(&mut wave, v, TE_SHORT);
            KiaV7Decoder::enc_add_level(&mut wave, !v, TE_SHORT);
        }
        KiaV7Decoder::enc_add_level(&mut wave, true, TE_SHORT);
        KiaV7Decoder::enc_add_level(&mut wave, false, TAIL_GAP_US);
        wave
    }

    /// A key whose CRC byte does not match (one counter bit flipped) must not be emitted, nor a
    /// valid-CRC key with another header byte.
    #[test]
    fn rejects_bad_crc_and_header() {
        let key = sample().data;
        assert_eq!(feed_all(&frame_wave(key)).len(), 1, "control frame decodes");
        assert!(feed_all(&frame_wave(key ^ (1 << 40))).is_empty(), "CRC mismatch");
        let mut bytes = key.to_be_bytes();
        bytes[0] = 0x4D;
        bytes[7] = crc8(&bytes[0..7], 0x7F, 0x4C);
        assert!(feed_all(&frame_wave(u64::from_be_bytes(bytes))).is_empty(), "header 0x4D");
    }

    /// Key files: decodes round-trip with ProtoPirate's lines (Repeat + Disp), Serial/Btn/Cnt
    /// rebuild the key as the C does (fixing a bad CRC too), and TX from a loaded key decodes.
    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::{assert_round_trip, key};
        let dec = KiaV7Decoder::new();
        let d = feed_all(&KiaV7Decoder::new().encode(&sample(), 0x02).unwrap()).remove(0);
        let text = assert_round_trip(&dec, &d);
        let k = d.data.to_be_bytes().iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(" ");
        assert!(
            text.ends_with(&format!(
                "Protocol: Kia V7\nBit: 64\nKey: {k}\nSerial: 180150001\nBtn: 2\nCnt: 4661\nRepeat: 3\nDisp: Kia V7 - Unlock\n"
            )),
            "{text}"
        );
        for button in [0x01u8, 0x04, 0x08] {
            let d = feed_all(&KiaV7Decoder::new().encode(&sample(), button).unwrap()).remove(0);
            assert_round_trip(&dec, &d);
        }
        // A bare key loads; a broken CRC only with fields (rebuilt), and never another header.
        let bare = format!("Protocol: Kia V7\nBit: 64\nKey: {k}\n");
        assert_eq!(dec.from_flipper_key(&key(&bare)).as_ref(), Some(&d));
        let bad = d.data ^ 1;
        let bad_k = bad.to_be_bytes().iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(" ");
        assert!(dec.from_flipper_key(&key(&format!("Protocol: Kia V7\nBit: 64\nKey: {bad_k}\n"))).is_none());
        let fixed = dec.from_flipper_key(&key(&format!("Protocol: Kia V7\nBit: 64\nKey: {bad_k}\nCnt: 4661\n")));
        assert_eq!(fixed.as_ref(), Some(&d));
        let edited = dec.from_flipper_key(&key(&text.replace("Cnt: 4661", "Cnt: 5000"))).unwrap();
        assert_eq!((edited.serial, edited.button, edited.counter), (d.serial, d.button, Some(5000)));
        // TX from the loaded key: counter + 1.
        let tx = feed_all(&dec.encode(&edited, 0x01).unwrap());
        assert_eq!((tx[0].serial, tx[0].button, tx[0].counter), (d.serial, Some(0x01), Some(5001)));
    }

    /// Oracle vectors (`#[ignore]`d): `KAT_KEY_VECTORS=<dir> cargo test kia_v7::tests::key_vectors
    /// -- --ignored` writes `kia_v7_<n>.sub` / `.kat.txt`.
    #[test]
    #[ignore]
    fn key_vectors() {
        let Some(dir) = std::env::var_os("KAT_KEY_VECTORS") else { return };
        let dir = std::path::PathBuf::from(dir);
        for (n, button) in [0x01u8, 0x04].into_iter().enumerate() {
            let upload = KiaV7Decoder::new().encode(&sample(), button).unwrap();
            let d = feed_all(&upload).remove(0);
            let key = KiaV7Decoder::new().to_flipper_key(&d).unwrap();
            crate::protocols::kia_v1::tests::write_key_vector(
                &dir,
                &format!("kia_v7_{n}"),
                433_920_000,
                "FuriHalSubGhzPreset2FSKDev476Async",
                &upload,
                &key,
            );
        }
    }
}
