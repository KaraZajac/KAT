//! Kia V5 decoder + encoder.
//!
//! Port of ProtoPirate `protocols/kia_v5.c` (HEAD 2026-10; the encoder dates from 2026-05-19 "Fix
//! KIA V5 encoder", which KAT had not ported). Flipper-ARF's `kia_v5.c` builds the same upload and
//! names the buttons (used below); its decoder only reports exactly-67-bit frames, a subset of
//! ProtoPirate's ≥64 (C oracle on IMPORTS: 117 vs 119 frames), so KAT follows ProtoPirate.
//!
//! Air format (315 / 433.92 MHz, FM): Manchester, te_short 400µs / te_long 800µs (te_delta 150).
//! Preamble of short pairs (>40 counted), a long HIGH sync, then the frame: 64 data bits and a 3-bit
//! CRC field (`0`, then the 2-bit CRC). The key is sent bit-reversed: `yek` = the 64-bit reverse of
//! the received key = `button4 << 60 | serial28 << 32 | hop32`, and the counter is the hop run
//! through the "mixer" cipher keyed by the KIA V5 key (keystore type 13).
//!
//! Decoder (`kia_protocol_decoder_v5_feed`): Flipper's Manchester machine (common.rs) with the
//! C's own event sense `level ? ShortHigh : ShortLow`, reset to Mid1 when the preamble starts; bits
//! after the 64th go to the CRC field (at most 67 bits). The first out-of-tolerance pulse ends the
//! frame, which is reported if ≥64 bits arrived: `data` = the 64 key bits, `data_count_bit` =
//! min(bits, 67), `extra` = the CRC field bits (right-aligned). The C only displays the CRC; KAT
//! sets `crc_valid` when the received CRC bits match `kia_v5_calculate_crc` of the key (all 117
//! complete frames in IMPORTS do). (KAT's earlier port used a hand-written Manchester table and
//! decoded garbage: serials changing between presses of the same fob.)
//!
//! Encoder (`kia_protocol_encoder_v5_deserialize` + `get_upload`): hop = `mixer_encode(serial,
//! counter, button)`, key = reverse(yek), CRC recomputed; 200 × (HIGH 400, LOW 400), LOW 400,
//! HIGH 800, LOW 400, HIGH 400, the 64 key bits + `0` + 2 CRC bits as Manchester (1 → LOW, HIGH;
//! 0 → HIGH, LOW, 400µs halves), closing LOW 400 + HIGH 400; adjacent equal levels merged. One
//! upload per call, which KAT sends 6 times as the C's TX loop does (`tx_copies.rs`). Each press sends counter + 1, as both
//! references do; KAT's generic button maps to ARF's Kia V5 codes: Lock → 0x2, Unlock → 0x1,
//! Trunk → 0x4, Panic → 0x8 ("Horn"); anything else keeps the captured button.
//!
//! Key files: ProtoPirate's `kia_protocol_decoder_v5_serialize` — "Kia V5", Bit 64..67, Key, Serial,
//! Btn, Cnt, CRC (the received CRC-field bits = `extra`), DataHi, DataLo, YekHi, YekLo. Loading
//! takes Bit 64..=67 + Key + CRC (ProtoPirate's decoder deserialize, which wants exactly 64, cannot
//! read its own 67-bit files); Flipper-ARF's "KIA/HYU V5" files (`IMPORTS/KIA/KIA V5 N3.sub`,
//! `HYUNDAI/test_new_kia_hyu.sub`: Bit 67, Key, CRC 3) load the same way, with `crc_valid` false as
//! the CRC field matches neither key (ARF's own display says the same). Edited Serial/Btn/Cnt
//! rebuild the frame as ProtoPirate's encoder sends it.

use super::common::{ManchesterEvent, ManchesterState};
use super::keys;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Kia V5",
    decoder: "Kia V5",
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

/// Flipper-ARF kia_v5.c names.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x1 => "Unlock",
        0x2 => "Lock",
        0x4 => "Trunk",
        0x8 => "Horn",
        _ => "Other",
    })
}

const TE_SHORT: u32 = 400;
const TE_LONG: u32 = 800;
const TE_DELTA: u32 = 150;
const MIN_COUNT_BIT: usize = 64;
const MAX_BITS: u8 = 67;
const PREAMBLE_PAIRS: usize = 200;

/// Decoder states (`KiaV5DecoderStep`).
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    CheckPreamble,
    Data,
}

/// Kia V5 decoder.
pub struct KiaV5Decoder {
    step: DecoderStep,
    te_last: u32,
    header_count: u16,
    bit_count: u8,
    decoded_data: u64,
    saved_key: u64,
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

/// `build_keystore_from_mfkey`: the KIA V5 key (keystore type 13), big-endian bytes.
fn keystore_bytes() -> [u8; 8] {
    keys::get_keystore().get_kia_v5_key().to_be_bytes()
}

/// `mixer_decode`: the 16-bit counter from the 32-bit hop.
fn mixer_decode(encrypted: u32) -> u16 {
    let [mut s0, mut s1, mut s2, mut s3] = encrypted.to_le_bytes();
    let ks = keystore_bytes();
    let mut round_index = 1usize;
    for _ in 0..18 {
        let mut r = ks[round_index];
        for _ in 0..8 {
            let mut base: u8 = match (s3 & 0x40 != 0, s3 & 0x02 != 0) {
                (false, false) => 0x74,
                (false, true) => 0x2E,
                (true, false) => 0x3A,
                (true, true) => 0x5C,
            };
            if s2 & 0x08 != 0 {
                base = base.rotate_left(4);
            }
            if s1 & 0x01 != 0 {
                base = (base & 0x3F) << 2;
            }
            if s0 & 0x01 != 0 {
                base <<= 1;
            }
            let temp = s3 ^ s1;
            s3 = (s3 << 1) | (s2 >> 7);
            s2 = (s2 << 1) | (s1 >> 7);
            s1 = (s1 << 1) | (s0 >> 7);
            s0 <<= 1;
            if (base ^ r ^ temp) & 0x80 != 0 {
                s0 |= 0x01;
            }
            r <<= 1;
        }
        round_index = round_index.wrapping_sub(1) & 0x7;
    }
    u16::from_le_bytes([s0, s1])
}

/// `mixer_encode`: the 32-bit hop for serial / counter / button.
fn mixer_encode(serial: u32, counter: u16, button: u8) -> u32 {
    let ks = keystore_bytes();
    let mut a = (((serial >> 8) & 0x0F) as u8) | ((button & 0x0F) << 4);
    let mut b = (counter >> 8) as u8;
    let mut c = serial as u8;
    let mut d = counter as u8;
    for round in 0..18 {
        let mut r = ks[round & 0x07];
        let mut running_d = d;
        for _ in 0..8 {
            let mut base: u8 = match (a & 0x80 != 0, a & 0x04 != 0) {
                (false, false) => 0x74,
                (false, true) => 0x2E,
                (true, false) => 0x3A,
                (true, true) => 0x5C,
            };
            if c & 0x10 != 0 {
                base = base.rotate_left(4);
            }
            if b & 0x02 != 0 {
                base = (base & 0x3F) << 2;
            }
            let base_final = if running_d & 0x02 != 0 { (base & 0x7F) << 1 } else { base };
            let (carry_b, carry_c, carry_a) = (b & 1 != 0, c & 1 != 0, a & 1 != 0);
            let mut new_d = running_d >> 1;
            if carry_b {
                new_d |= 0x80;
            }
            running_d ^= c;
            b = (b >> 1) | if carry_c { 0x80 } else { 0 };
            c = (c >> 1) | if carry_a { 0x80 } else { 0 };
            let feedback = ((running_d ^ r) << 7) ^ base_final;
            a = (a >> 1) | (feedback & 0x80);
            r >>= 1;
            running_d = new_d;
        }
        d = running_d;
    }
    u32::from_be_bytes([a, c, b, d])
}

/// `kia_v5_calculate_crc`: 2-bit CRC of the 64-bit key.
fn calculate_crc(data: u64) -> u8 {
    let mut crc = 0u8;
    for i in (0..64).rev() {
        let bit = ((data >> i) & 1) as u8;
        let shifted_out = (crc >> 1) & 1;
        crc = ((crc & 1) << 1) | bit;
        if shifted_out != 0 {
            crc ^= 3;
        }
    }
    crc & 3
}

/// Serial, button and counter of a received key (`yek` = the key bit-reversed).
fn parse_key(key: u64) -> (u32, u8, u16) {
    let yek = key.reverse_bits();
    (((yek >> 32) & 0x0FFF_FFFF) as u32, (yek >> 60) as u8, mixer_decode(yek as u32))
}

impl KiaV5Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            te_last: 0,
            header_count: 0,
            bit_count: 0,
            decoded_data: 0,
            saved_key: 0,
            manchester: ManchesterState::Mid1,
        }
    }

    /// The C's emission (with KAT's CRC flag, see the module docs).
    fn report(&self) -> DecodedSignal {
        frame_signal(self.saved_key, self.bit_count.min(MAX_BITS), self.decoded_data & 0x07)
    }
}

/// The decode of a `bits`-bit frame (64..=67): the 64 key bits and the CRC-field bits that
/// followed them (`received`, right-aligned).
fn frame_signal(key: u64, bits: u8, received: u64) -> DecodedSignal {
    let tail_bits = bits - MIN_COUNT_BIT as u8;
    // Sent as `0, crc1, crc0`: compare the bits that arrived with that prefix.
    let expected = (calculate_crc(key) as u64) >> (3 - tail_bits);
    let (serial, button, counter) = parse_key(key);
    DecodedSignal {
        serial: Some(serial),
        button: Some(button),
        counter: Some(counter),
        crc_valid: received == expected,
        data: key,
        data_count_bit: bits as usize,
        encoder_capable: true,
        extra: (tail_bits > 0).then_some(received),
        protocol_display_name: None,
    }
}

impl ProtocolDecoder for KiaV5Decoder {
    fn name(&self) -> &'static str {
        "Kia V5"
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
        self.step = DecoderStep::Reset;
        self.header_count = 0;
        self.bit_count = 0;
        self.decoded_data = 0;
        self.saved_key = 0;
        self.manchester = ManchesterState::Mid1;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::Reset => {
                if level && is_short(duration) {
                    self.step = DecoderStep::CheckPreamble;
                    self.te_last = duration;
                    self.header_count = 1;
                    self.bit_count = 0;
                    self.decoded_data = 0;
                    self.manchester = ManchesterState::Mid1;
                }
            }
            DecoderStep::CheckPreamble => {
                if level {
                    if is_long(duration) {
                        if self.header_count > 40 {
                            self.step = DecoderStep::Data;
                            self.bit_count = 0;
                            self.decoded_data = 0;
                            self.saved_key = 0;
                            self.header_count = 0;
                        } else {
                            self.te_last = duration;
                        }
                    } else if is_short(duration) {
                        self.te_last = duration;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else {
                    if (is_short(duration) && is_short(self.te_last))
                        || (is_long(duration) && is_short(self.te_last))
                        || is_long(self.te_last)
                    {
                        self.header_count = self.header_count.wrapping_add(1);
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                    self.te_last = duration;
                }
            }
            DecoderStep::Data => {
                // The C's event sense: level ? Short/LongHigh : Short/LongLow.
                let event = if is_short(duration) {
                    ManchesterEvent::from_half_bit(level, false)
                } else if is_long(duration) {
                    ManchesterEvent::from_half_bit(level, true)
                } else {
                    self.step = DecoderStep::Reset;
                    return (self.bit_count as usize >= MIN_COUNT_BIT).then(|| self.report());
                };
                if self.bit_count <= 66 {
                    if let Some(bit) = self.manchester.advance(event) {
                        self.decoded_data = (self.decoded_data << 1) | bit as u64;
                        self.bit_count += 1;
                        if self.bit_count as usize == MIN_COUNT_BIT {
                            self.saved_key = self.decoded_data;
                            self.decoded_data = 0;
                        }
                    }
                }
                self.te_last = duration;
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        let (p_serial, p_button, p_counter) = parse_key(decoded.data);
        let serial = decoded.serial.unwrap_or(p_serial) & 0x0FFF_FFFF;
        let captured = decoded.button.unwrap_or(p_button) & 0x0F;
        let counter = decoded.counter.unwrap_or(p_counter).wrapping_add(1);
        let button = match button {
            0x01 => 0x2,
            0x02 => 0x1,
            0x04 => 0x4,
            0x08 => 0x8,
            _ => captured,
        };
        Some(build_upload(build_key(serial, button, counter)))
    }

    /// Key files. ProtoPirate's decoder deserialize insists on Bit 64 and so cannot read its own
    /// 67-bit files; KAT loads what the decoder reports (Bit 64..=67, the C's encoder accepts any
    /// ≥ 64; Flipper-ARF 64 or 67): Key = the 64 key bits, `CRC` = the CRC-field bits that followed
    /// (as written; ARF's "KIA/HYU V5" files carry the same two lines). The derived DataHi/DataLo/
    /// YekHi/YekLo lines are not read (ProtoPirate's decoder ignores them; ARF prefers YekHi/YekLo
    /// over the Key). ProtoPirate's encoder transmits from Serial/Btn/Cnt: when they differ from
    /// the Key's, KAT rebuilds the frame from them (mixer re-encrypted, a full 67-bit frame with
    /// a fresh CRC), as that encoder sends it.
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if !(MIN_COUNT_BIT..=MAX_BITS as usize).contains(&key.bits) {
            return None;
        }
        let data = key.key_u64()?;
        let tail_bits = key.bits - MIN_COUNT_BIT;
        let received = key.u32_field("CRC").map_or(0, |c| c as u64) & ((1 << tail_bits) - 1);
        let d = frame_signal(data, key.bits as u8, received);
        let (serial, button, counter) = (key.u32_field("Serial"), key.u32_field("Btn"), key.u32_field("Cnt"));
        if serial.is_none() && button.is_none() && counter.is_none() {
            return Some(d);
        }
        let fields = (
            serial.map_or(d.serial?, |s| s & 0x0FFF_FFFF),
            button.map_or(d.button?, |b| (b & 0x0F) as u8),
            counter.map_or(d.counter?, |c| c as u16),
        );
        if fields == (d.serial?, d.button?, d.counter?) {
            return Some(d);
        }
        let rebuilt = build_key(fields.0, fields.1, fields.2);
        Some(frame_signal(rebuilt, MAX_BITS, calculate_crc(rebuilt) as u64))
    }

    /// Key files: ProtoPirate `kia_protocol_decoder_v5_serialize` — "Kia V5", Bit 64..67, Key =
    /// the 64 key bits, Serial, Btn, Cnt (`pp_serialize_fields`), CRC (the received CRC-field bits;
    /// 0 for a 64-bit frame), DataHi, DataLo (the Key's halves), YekHi, YekLo (the bit-reversed
    /// key's halves), all decimal.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if !(MIN_COUNT_BIT..=MAX_BITS as usize).contains(&decoded.data_count_bit) {
            return None;
        }
        let data = decoded.data;
        let (serial, button, counter) = parse_key(data);
        let yek = data.reverse_bits();
        let mut key = FlipperKey::for_protocol("Kia V5", decoded.data_count_bit, data);
        key.push_u32("Serial", serial);
        key.push_u32("Btn", button.into());
        key.push_u32("Cnt", decoded.counter.unwrap_or(counter).into());
        key.push_u32("CRC", decoded.extra.unwrap_or(0) as u32 & 0x07);
        key.push_u32("DataHi", (data >> 32) as u32);
        key.push_u32("DataLo", data as u32);
        key.push_u32("YekHi", (yek >> 32) as u32);
        key.push_u32("YekLo", yek as u32);
        Some(key)
    }
}

impl Default for KiaV5Decoder {
    fn default() -> Self {
        Self::new()
    }
}

/// The key the encoder sends: reverse(`button << 60 | serial << 32 | mixer_encode(..)`).
fn build_key(serial: u32, button: u8, counter: u16) -> u64 {
    let serial = serial & 0x0FFF_FFFF;
    let button = button & 0x0F;
    let yek = ((button as u64) << 60) | ((serial as u64) << 32) | mixer_encode(serial, counter, button) as u64;
    yek.reverse_bits()
}

/// Append one level, merging with the previous entry when the level repeats (identical on air).
fn push(out: &mut Vec<LevelDuration>, level: bool, duration: u32) {
    match out.last_mut() {
        Some(last) if last.level == level => last.duration_us += duration,
        _ => out.push(LevelDuration::new(level, duration)),
    }
}

/// `kia_v5_emit_manchester_bit`: 1 → LOW, HIGH; 0 → HIGH, LOW.
fn push_bit(out: &mut Vec<LevelDuration>, bit: bool) {
    push(out, !bit, TE_SHORT);
    push(out, bit, TE_SHORT);
}

/// `kia_protocol_encoder_v5_get_upload` for a key.
fn build_upload(key: u64) -> Vec<LevelDuration> {
    let crc = calculate_crc(key);
    let mut out = Vec::with_capacity(PREAMBLE_PAIRS * 2 + 4 + 2 * 67 + 2);
    for _ in 0..PREAMBLE_PAIRS {
        push(&mut out, true, TE_SHORT);
        push(&mut out, false, TE_SHORT);
    }
    push(&mut out, false, TE_SHORT);
    push(&mut out, true, TE_LONG);
    push(&mut out, false, TE_SHORT);
    push(&mut out, true, TE_SHORT);
    for b in (0..64).rev() {
        push_bit(&mut out, (key >> b) & 1 != 0);
    }
    push_bit(&mut out, false);
    push_bit(&mut out, crc & 2 != 0);
    push_bit(&mut out, crc & 1 != 0);
    push(&mut out, false, TE_SHORT);
    push(&mut out, true, TE_SHORT);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn feed_all(wire: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = KiaV5Decoder::new();
        wire.iter()
            .chain(std::iter::once(&LevelDuration::new(false, 20_000)))
            .filter_map(|p| dec.feed(p.level, p.duration_us))
            .collect()
    }

    /// Real `IMPORTS/KIA/KIA V5 N1 RAW.sub` / `KIA V5 N2 RAW.sub` frames as the C (`kia_v5.c` on the
    /// host, KIA V5 key) decodes them: key, serial, button, counter, CRC field.
    const C_CAPTURE: &[(u64, u32, u8, u16, u64)] = &[
        (0x9CAF_491D_6803_0228, 0x0440_C016, 0x1, 0x5360, 1),
        (0x561D_6DAA_6803_0228, 0x0440_C016, 0x1, 0x5361, 2),
        (0xB8BE_DC92_70A4_EE88, 0x0177_250E, 0x1, 0x2DE4, 2),
        (0xA07F_45FC_70A4_EE88, 0x0177_250E, 0x1, 0x2DE5, 3),
    ];

    #[test]
    fn capture_frames_and_cipher() {
        keys::load_keystore_from_embedded();
        for &(key, serial, button, counter, crc) in C_CAPTURE {
            assert_eq!(parse_key(key), (serial, button, counter));
            assert_eq!(calculate_crc(key) as u64, crc);
            // The encoder's mixer is the inverse of the decoder's: same key for the same fields.
            assert_eq!(build_key(serial, button, counter), key);
            let got = feed_all(&build_upload(key));
            assert_eq!(got.len(), 1);
            let d = &got[0];
            assert_eq!((d.data, d.data_count_bit, d.extra, d.crc_valid), (key, 67, Some(crc), true));
        }
    }

    /// encode(): counter + 1 and KAT's generic button mapped to ARF's Kia V5 codes.
    #[test]
    fn encode_bumps_counter_and_maps_buttons() {
        keys::load_keystore_from_embedded();
        let (key, serial, _, counter, _) = C_CAPTURE[0];
        for (generic, want) in [(0x01u8, 0x2u8), (0x02, 0x1), (0x04, 0x4), (0x08, 0x8), (0x40, 0x1)] {
            let src = DecodedSignal {
                serial: Some(serial),
                button: Some(0x1),
                counter: Some(counter),
                crc_valid: true,
                data: key,
                data_count_bit: 67,
                encoder_capable: true,
                extra: Some(1),
                protocol_display_name: None,
            };
            let got = feed_all(&KiaV5Decoder::new().encode(&src, generic).unwrap());
            let d = &got[0];
            assert_eq!((d.serial, d.button, d.counter), (Some(serial), Some(want), Some(counter + 1)));
            assert!(d.crc_valid);
        }
    }

    /// Real fob data: the IMPORTS key files `KIA/KIA V5 N3.sub` and `HYUNDAI/test_new_kia_hyu.sub`
    /// (Flipper-ARF "KIA/HYU V5"): the encoder rebuilds each key exactly from its fields. (Their
    /// saved "CRC: 3" matches neither key's CRC; the RAW captures' CRC fields all match.)
    #[test]
    fn real_key_files_rebuild() {
        keys::load_keystore_from_embedded();
        for key in [0x4CD2_BB72_AF95_1B84u64, 0x7AA3_1142_AF95_1B84] {
            let (serial, button, counter) = parse_key(key);
            assert_eq!(build_key(serial, button, counter), key);
        }
    }

    /// Key files: the captures' decodes (67 bits) and a 64-bit decode round-trip with ProtoPirate's
    /// lines; the IMPORTS key files (Flipper-ARF "KIA/HYU V5") load; edited fields rebuild the
    /// frame; TX from a loaded key decodes back.
    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::{assert_round_trip, key};
        keys::load_keystore_from_embedded();
        let dec = KiaV5Decoder::new();
        for &(k, ..) in C_CAPTURE {
            for d in feed_all(&build_upload(k)) {
                assert_round_trip(&dec, &d);
            }
        }
        let d = feed_all(&build_upload(C_CAPTURE[0].0)).remove(0);
        let text = assert_round_trip(&dec, &d);
        assert!(
            text.ends_with(
                "Protocol: Kia V5\nBit: 67\nKey: 9C AF 49 1D 68 03 02 28\nSerial: 71352342\nBtn: 1\n\
                 Cnt: 21344\nCRC: 1\nDataHi: 2628733213\nDataLo: 1745027624\nYekHi: 339787798\n\
                 YekLo: 3096638777\n"
            ),
            "{text}"
        );
        // A frame cut after the 64 key bits (no CRC field): Bit 64, CRC 0, no `extra`.
        let short = frame_signal(C_CAPTURE[1].0, 64, 0);
        assert!(assert_round_trip(&dec, &short).contains("\nBit: 64\n"));
        // IMPORTS/KIA/KIA V5 N3.sub and HYUNDAI/test_new_kia_hyu.sub.
        for k in ["4C D2 BB 72 AF 95 1B 84", "7A A3 11 42 AF 95 1B 84"] {
            let file = format!(
                "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                 Preset: FuriHalSubGhzPreset2FSKDev476Async\nProtocol: KIA/HYU V5\nBit: 67\nKey: {k}\nCRC: 3\n"
            );
            let l = dec.from_flipper_key(&key(&file)).unwrap();
            let (serial, button, counter) = parse_key(l.data);
            assert_eq!((l.serial, l.button, l.counter), (Some(serial), Some(button), Some(counter)));
            assert_eq!((l.data_count_bit, l.extra, l.crc_valid), (67, Some(3), false));
            // TX from the loaded key: same serial, counter + 1, fresh CRC.
            let tx = feed_all(&dec.encode(&l, 0x01).unwrap());
            assert_eq!((tx[0].serial, tx[0].button, tx[0].counter, tx[0].crc_valid), (Some(serial), Some(0x2), Some(counter.wrapping_add(1)), true));
        }
        // An edited counter rebuilds the frame (mixer re-encrypted, fresh CRC); other Bits fail.
        let e = dec.from_flipper_key(&key(&text.replace("Cnt: 21344", "Cnt: 21400"))).unwrap();
        assert_eq!((e.serial, e.button, e.counter, e.crc_valid, e.data_count_bit), (d.serial, d.button, Some(21400), true, 67));
        assert!(dec.from_flipper_key(&key(&text.replace("Bit: 67", "Bit: 68"))).is_none());
    }

    /// Oracle vectors (`#[ignore]`d): `KAT_KEY_VECTORS=<dir> cargo test kia_v5::tests::key_vectors
    /// -- --ignored` writes `kia_v5_<n>.sub` / `.kat.txt`.
    #[test]
    #[ignore]
    fn key_vectors() {
        let Some(dir) = std::env::var_os("KAT_KEY_VECTORS") else { return };
        let dir = std::path::PathBuf::from(dir);
        keys::load_keystore_from_embedded();
        for (n, &(k, ..)) in C_CAPTURE.iter().take(2).enumerate() {
            let upload = build_upload(k);
            let d = feed_all(&upload).remove(0);
            let key = KiaV5Decoder::new().to_flipper_key(&d).unwrap();
            crate::protocols::kia_v1::tests::write_key_vector(
                &dir,
                &format!("kia_v5_{n}"),
                433_920_000,
                "FuriHalSubGhzPreset2FSKDev476Async",
                &upload,
                &key,
            );
        }
    }
}
