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
//! upload per call (the C TX loop repeats it 6 times). Each press sends counter + 1, as both
//! references do; KAT's generic button maps to ARF's Kia V5 codes: Lock → 0x2, Unlock → 0x1,
//! Trunk → 0x4, Panic → 0x8 ("Horn"); anything else keeps the captured button.

use super::common::{ManchesterEvent, ManchesterState};
use super::keys;
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

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
        let key = self.saved_key;
        let bits = self.bit_count.min(MAX_BITS);
        let tail_bits = bits - MIN_COUNT_BIT as u8;
        let received = self.decoded_data & 0x07;
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
}
