//! HondaV2&V3 protocol decoder (**decode only**; FoxFW's encoder lives in an SD plugin that KAT
//! does not port)
//!
//! Ported from FoxFW `lib/subghz/protocols/honda_v2_v3.c` and `honda_v2_v3.h`; no other reference
//! carries it (KAT's own "Honda V2" is ProtoPirate's differential-Manchester protocol, unrelated:
//! this one is registered by FoxFW as "HondaV2&V3"). One protocol, two variants told apart at
//! run time by the first pulse that matches either timing; the FM (2-FSK, 433.92 MHz) flag is the
//! C's.
//!
//! **Air format**: Manchester through the Honda state machine (`{0x01, 0x91, 0x9B, 0xFB}` indexed
//! by the previous state and shifted by the pulse type: high short 0, low short 2, high long 4,
//! low long 6; `ns == state` yields nothing, state 2 yields a 0 and state 1 a 1), at most 200 bits:
//!
//! | | V2 | V3 |
//! |---|---|---|
//! | te short / long / delta | 96 / 192 / 50 µs | 250 / 500 / 80 µs |
//! | frame ends on a gap above | 500 µs | 1500 µs |
//! | preamble (not checked) | ~116 x 1460 µs | ~230 x 500 µs |
//! | minimum bits to process | 100 | 100 |
//!
//! The first pulse in Reset within either variant's short window (else long window) picks the
//! variant (a tie goes to V2) and starts collecting bits; a pulse that is not valid for the
//! variant, a gap, or 200 bits ends the burst and, with at least 100 bits, processes the frame.
//!
//! **V2 frame** (13 data bytes after the sync `0E 1E 1E 1E 0D 6C` found at any byte offset of the
//! decoded bits; without the sync, the first offset whose bytes 3 and 4 are a known button pair):
//! `serial = d[0..3]` (24 bits), `btn1 = d[3]`, `btn2 = d[4]`, padding `d[5..7]`, `counter = d[7]`,
//! `rolling = d[8..12]`, `tail = d[12]`. The button pair must be one of (0x40, 0x20) lock,
//! (0x41, 0x40) unlock, (0x41, 0x20) unlock (retransmit) or (0x40, 0x60) trunk. **There is no
//! CRC**, so `crc_valid` is false. KAT fields: `data` = `serial << 40 | counter << 32 | rolling`
//! (the C `generic.data`), `data_count_bit` 148, `serial`, `button` = `btn2` (as the C), `counter`
//! = the 8-bit counter, `extra` = `tail | btn1 << 8 | padding << 16 | has_sync << 32`.
//!
//! **V3 frame** (15 bytes starting at the first `0x6A` of the decoded bytes, after stripping
//! trailing `0x55` idle bytes down to 15): `d[0]` = 0x6A, `d[1]` = counter byte (`counter << 2 |
//! tx_flag << 1 | bit0`), `d[2..14]` payload, `d[14]` = CRC. The CRC is a CRC-16 (MSB first, poly
//! 0x2627, init 0) over `d[1..14]`, folded to 8 bits by eight parity masks (`0x8181, 0x4141,
//! 0x2121, 0x80F2, 0xAF15, 0x0606, 0x0202, 0x0000`) xor the constant 0x1B; emission is gated on
//! it. A *fixed* (type A) frame has `d[4] & 0xF0 == 0x80` and `d[5]` of 0x82 / 0x83: then `serial
//! = d[2..4]` and `button = d[4]`; a hopping (type B) frame is encrypted throughout and has
//! serial 0 and button 0. KAT fields: `data` = `serial << 48 | button << 40 | counter_byte << 32 |
//! crc << 24` (the C `generic.data`), `data_count_bit` 120, `serial`, `button`, `counter` = the
//! counter byte >> 2, `crc_valid` true, `extra` = `d[6..14]` (the last eight payload bytes; the C
//! also keeps `d[2..6]`, which for a fixed frame are the serial, button and flags, and which KAT
//! does not keep for a hopping frame).
//!
//! Key files are not supported: the C's V3 key file carries the 12-byte payload, which does not
//! fit a KAT decode.
//!
//! No capture of this protocol exists in the test corpus: the port is validated against the C
//! with synthetic pulse sequences only.

use super::info::prelude::*;
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (see `info.rs`): the C's `SubGhzProtocolFlag_433 | FM`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::FM,
    encryption: "Rolling Code",
    make: "Honda/Acura",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&FM476],
};

const NAME: &str = "HondaV2&V3";

const V2_TE_SHORT: u32 = 96;
const V2_TE_LONG: u32 = 192;
const V2_TE_DELTA: u32 = 50;
const V2_MIN_GAP: u32 = 500;
const V2_FRAME_BITS: usize = 148;
const V2_MIN_FRAME_BITS: usize = 100;
const V2_MAX_BITS: usize = 200;
const V2_SYNC_BYTES: usize = 6;
const V2_DATA_BYTES: usize = 13;
const V2_SYNC_PATTERN: [u8; V2_SYNC_BYTES] = [0x0E, 0x1E, 0x1E, 0x1E, 0x0D, 0x6C];

const V3_TE_SHORT: u32 = 250;
const V3_TE_LONG: u32 = 500;
const V3_TE_DELTA: u32 = 80;
const V3_MIN_GAP: u32 = 1500;
const V3_FRAME_BITS: usize = 120;
const V3_MIN_FRAME_BITS: usize = 100;
const V3_MAX_BITS: usize = 200;
const V3_SYNC_BYTE: u8 = 0x6A;
const V3_TOTAL_BYTES: usize = 15;

const MAN_TABLE: [u8; 4] = [0x01, 0x91, 0x9B, 0xFB];

#[derive(Clone, Copy, PartialEq, Eq)]
enum Variant {
    V2,
    V3,
}

impl Variant {
    /// (te_short, te_long, te_delta, min_gap, min_count_bit_for_found, max_bits)
    fn params(self) -> (u32, u32, u32, u32, usize, usize) {
        match self {
            Variant::V2 => (V2_TE_SHORT, V2_TE_LONG, V2_TE_DELTA, V2_MIN_GAP, V2_MIN_FRAME_BITS, V2_MAX_BITS),
            Variant::V3 => (V3_TE_SHORT, V3_TE_LONG, V3_TE_DELTA, V3_MIN_GAP, V3_MIN_FRAME_BITS, V3_MAX_BITS),
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    Data,
}

/// HondaV2&V3 decoder (matches SubGhzProtocolDecoderHondaV2V3).
pub struct HondaV2V3Decoder {
    step: Step,
    variant: Variant,
    manchester_state: u8,
    bits: Vec<u8>,
}

/// `honda_manchester_decode`: Some(bit) when the machine yields one.
fn manchester_decode(state: &mut u8, timing_type: u8) -> Option<u8> {
    if timing_type == 8 || *state >= 4 {
        *state = 1;
        return None;
    }
    let new_state = (MAN_TABLE[usize::from(*state)] >> timing_type) & 0x3;
    if *state == new_state {
        return None;
    }
    *state = new_state;
    match new_state {
        2 => Some(0),
        1 => Some(1),
        _ => None,
    }
}

/// `honda_v2_validate_buttons`.
fn v2_validate_buttons(b1: u8, b2: u8) -> bool {
    matches!((b1, b2), (0x40, 0x20) | (0x41, 0x40) | (0x41, 0x20) | (0x40, 0x60))
}

/// `honda_v3_compute_crc`: CRC-16 (MSB first, poly 0x2627) over `frame[1..14]` folded to 8 bits.
fn v3_compute_crc(frame: &[u8]) -> u8 {
    let mut crc: u16 = 0;
    for &b in &frame[1..14] {
        crc ^= u16::from(b) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 { (crc << 1) ^ 0x2627 } else { crc << 1 };
        }
    }
    const MASKS: [u16; 8] = [0x8181, 0x4141, 0x2121, 0x80F2, 0xAF15, 0x0606, 0x0202, 0x0000];
    const CONSTANTS: u8 = 0x1B;
    let mut result = 0u8;
    for (b, &mask) in MASKS.iter().enumerate() {
        let parity = ((crc & mask).count_ones() & 1) as u8;
        result |= (parity ^ ((CONSTANTS >> (7 - b)) & 1)) << (7 - b);
    }
    result
}

impl HondaV2V3Decoder {
    pub fn new() -> Self {
        Self { step: Step::Reset, variant: Variant::V2, manchester_state: 1, bits: Vec::with_capacity(200) }
    }

    fn restart(&mut self) {
        self.step = Step::Reset;
        self.bits.clear();
        self.manchester_state = 1;
    }

    /// `honda_process_frame`.
    fn process_frame(&self) -> Option<DecodedSignal> {
        let bit_count = self.bits.len();
        let byte_count = bit_count.div_ceil(8);
        let mut raw = [0u8; 32];
        for (i, &b) in self.bits.iter().enumerate() {
            if b != 0 && i / 8 < raw.len() {
                raw[i / 8] |= 1 << (7 - (i % 8));
            }
        }
        match self.variant {
            Variant::V2 => {
                let mut sync_offset = None;
                if byte_count >= V2_SYNC_BYTES + V2_DATA_BYTES {
                    sync_offset = (0..=byte_count - V2_SYNC_BYTES - V2_DATA_BYTES)
                        .find(|&o| raw[o..o + V2_SYNC_BYTES] == V2_SYNC_PATTERN);
                }
                let has_sync = sync_offset.is_some();
                let start = match sync_offset {
                    Some(o) => o + V2_SYNC_BYTES,
                    None => {
                        // Fallback: the first offset whose bytes 3 / 4 are a known button pair.
                        if byte_count < V2_DATA_BYTES {
                            return None;
                        }
                        (0..=byte_count - V2_DATA_BYTES).find(|&o| v2_validate_buttons(raw[o + 3], raw[o + 4]))?
                    }
                };
                let d = &raw[start..start + V2_DATA_BYTES];
                let serial = (u32::from(d[0]) << 16) | (u32::from(d[1]) << 8) | u32::from(d[2]);
                let (btn1, btn2) = (d[3], d[4]);
                let padding = (u64::from(d[5]) << 8) | u64::from(d[6]);
                let counter = d[7];
                let rolling = u32::from_be_bytes([d[8], d[9], d[10], d[11]]);
                let tail = d[12];
                if !v2_validate_buttons(btn1, btn2) {
                    return None;
                }
                Some(DecodedSignal {
                    serial: Some(serial),
                    button: Some(btn2),
                    counter: Some(u16::from(counter)),
                    crc_valid: false,
                    data: (u64::from(serial) << 40) | (u64::from(counter) << 32) | u64::from(rolling),
                    data_count_bit: V2_FRAME_BITS,
                    encoder_capable: false,
                    extra: Some(
                        u64::from(tail)
                            | (u64::from(btn1) << 8)
                            | (padding << 16)
                            | (u64::from(has_sync) << 32),
                    ),
                    protocol_display_name: None,
                })
            }
            Variant::V3 => {
                // Strip trailing 0x55 idle bytes, never below the frame size.
                let mut byte_count = byte_count;
                while byte_count > V3_TOTAL_BYTES && raw[byte_count - 1] == 0x55 {
                    byte_count -= 1;
                }
                if byte_count < V3_TOTAL_BYTES {
                    return None;
                }
                let sync_idx = (0..=byte_count - V3_TOTAL_BYTES).find(|&i| raw[i] == V3_SYNC_BYTE)?;
                let d = &raw[sync_idx..sync_idx + V3_TOTAL_BYTES];
                let counter = d[1];
                let crc = d[14];
                if v3_compute_crc(d) != crc {
                    return None;
                }
                let raw_btn = d[4];
                let flags = d[5];
                let is_fixed = (raw_btn & 0xF0) == 0x80 && (flags == 0x82 || flags == 0x83);
                let (serial, btn_byte) =
                    if is_fixed { ((u16::from(d[2]) << 8) | u16::from(d[3]), raw_btn) } else { (0, 0) };
                Some(DecodedSignal {
                    serial: Some(u32::from(serial)),
                    button: Some(btn_byte),
                    counter: Some(u16::from(counter >> 2)),
                    crc_valid: true,
                    data: (u64::from(serial) << 48)
                        | (u64::from(btn_byte) << 40)
                        | (u64::from(counter) << 32)
                        | (u64::from(crc) << 24),
                    data_count_bit: V3_FRAME_BITS,
                    encoder_capable: false,
                    extra: Some(d[6..14].iter().fold(0u64, |a, &b| (a << 8) | u64::from(b))),
                    protocol_display_name: None,
                })
            }
        }
    }

    /// `honda_decode_timing`.
    fn decode_timing(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let (te_short, te_long, te_delta, _, min_bits, max_bits) = self.variant.params();
        let timing_type = if duration.abs_diff(te_short) <= te_delta {
            Some(if level { 0 } else { 2 })
        } else if duration.abs_diff(te_long) <= te_delta {
            Some(if level { 4 } else { 6 })
        } else {
            None
        };
        match timing_type {
            Some(tt) => {
                if let Some(bit) = manchester_decode(&mut self.manchester_state, tt) {
                    if self.bits.len() < max_bits {
                        self.bits.push(bit);
                    }
                    if self.bits.len() >= max_bits {
                        let out = self.process_frame();
                        self.restart();
                        return out;
                    }
                }
                None
            }
            None => {
                let out = if self.bits.len() >= min_bits { self.process_frame() } else { None };
                self.restart();
                out
            }
        }
    }
}

impl Default for HondaV2V3Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for HondaV2V3Decoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: V2_TE_SHORT,
            te_long: V2_TE_LONG,
            te_delta: V2_TE_DELTA,
            min_count_bit: V2_MIN_FRAME_BITS,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[433_920_000]
    }

    fn reset(&mut self) {
        self.restart();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                // Variant by the closest short window, else the closest long window (V2 on a tie).
                let mut best: Option<(Variant, u32)> = None;
                for v in [Variant::V2, Variant::V3] {
                    let (ts, _, td, ..) = v.params();
                    let diff = duration.abs_diff(ts);
                    if diff <= td && best.is_none_or(|(_, bd)| diff < bd) {
                        best = Some((v, diff));
                    }
                }
                if best.is_none() {
                    for v in [Variant::V2, Variant::V3] {
                        let (_, tl, td, ..) = v.params();
                        let diff = duration.abs_diff(tl);
                        if diff <= td && best.is_none_or(|(_, bd)| diff < bd) {
                            best = Some((v, diff));
                        }
                    }
                }
                let (v, _) = best?;
                self.variant = v;
                self.step = Step::Data;
                self.manchester_state = 1;
                self.bits.clear();
                self.decode_timing(level, duration)
            }
            Step::Data => {
                let (_, _, _, min_gap, min_bits, _) = self.variant.params();
                if duration > min_gap {
                    let out = if self.bits.len() >= min_bits { self.process_frame() } else { None };
                    self.restart();
                    return out;
                }
                self.decode_timing(level, duration)
            }
        }
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::super::honda_synth::{bits_of, decode, dump, halves_to_pulses, Pulse};
    use super::*;

    const PRESET: &str = "FuriHalSubGhzPreset2FSKDev476Async";

    /// Pulses of `payload` bytes after `lead_ones` preamble ones, Manchester at `half` µs per half
    /// bit (1 = high, low; 0 = low, high), then a closing gap.
    fn manchester_burst(payload: &[u8], lead_ones: usize, half: u32, gap: u32) -> Vec<Pulse> {
        let mut bits = vec![true; lead_ones];
        bits.extend(bits_of(payload));
        let halves: Vec<bool> = bits.iter().flat_map(|&b| if b { [true, false] } else { [false, true] }).collect();
        let mut p = halves_to_pulses(&halves, half);
        p.push((false, gap));
        p
    }

    /// Made-up V2 payload: sync, then serial, buttons, padding, counter, rolling, tail.
    fn v2_payload(btn: (u8, u8)) -> Vec<u8> {
        let mut p = V2_SYNC_PATTERN.to_vec();
        p.extend([0x12, 0x34, 0x56, btn.0, btn.1, 0x00, 0x00, 0x2A, 0xDE, 0xAD, 0xBE, 0xEF, 0x7E]);
        p
    }

    /// Made-up V3 frame: 0x6A, counter byte, 12 payload bytes, folded CRC.
    fn v3_frame(counter_byte: u8, payload: [u8; 12]) -> Vec<u8> {
        let mut f = vec![V3_SYNC_BYTE, counter_byte];
        f.extend(payload);
        f.push(0);
        let crc = v3_compute_crc(&f);
        f[14] = crc;
        f
    }

    #[test]
    fn honda_v2_v3_decodes_a_v2_frame() {
        // 17 preamble ones: the state machine drops the first bit, leaving 16, so that the sync byte
        // is aligned to a byte boundary of the decoded bits.
        let pulses = manchester_burst(&v2_payload((0x41, 0x40)), 17, 96, 3000);
        dump("hondav2v3_v2_unlock", &pulses, PRESET);
        let got = decode(&mut HondaV2V3Decoder::new(), &pulses);
        assert_eq!(got.len(), 1);
        let d = &got[0];
        assert_eq!((d.serial, d.button, d.counter), (Some(0x12_3456), Some(0x40), Some(0x2A)));
        assert_eq!((d.data_count_bit, d.crc_valid), (148, false));
        assert_eq!(d.data, (0x12_3456u64 << 40) | (0x2Au64 << 32) | 0xDEAD_BEEF);
        assert_eq!(d.extra, Some(0x7E | (0x41 << 8) | (1u64 << 32)));
    }

    #[test]
    fn honda_v2_v3_decodes_v3_frames_fixed_and_hopping() {
        // Fixed (type A): d[2..4] serial, d[4] 0x8C button, d[5] 0x82.
        let fixed = v3_frame(0x3D, [0xAB, 0xCD, 0x8C, 0x82, 1, 2, 3, 4, 5, 6, 7, 8]);
        let pulses = manchester_burst(&fixed, 17, 250, 4000);
        dump("hondav2v3_v3_fixed", &pulses, PRESET);
        let got = decode(&mut HondaV2V3Decoder::new(), &pulses);
        assert_eq!(got.len(), 1);
        let d = &got[0];
        assert_eq!((d.serial, d.button, d.counter), (Some(0xABCD), Some(0x8C), Some(0x3D >> 2)));
        assert_eq!((d.data_count_bit, d.crc_valid), (120, true));
        assert_eq!(d.data, (0xABCDu64 << 48) | (0x8Cu64 << 40) | (0x3Du64 << 32) | (u64::from(fixed[14]) << 24));
        assert_eq!(d.extra, Some(u64::from_be_bytes([1, 2, 3, 4, 5, 6, 7, 8])));
        // Hopping (type B): no readable serial / button.
        let hop = v3_frame(0x41, [0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x01, 0x02, 0x03, 0x04, 0x05]);
        let pulses = manchester_burst(&hop, 17, 250, 4000);
        dump("hondav2v3_v3_hopping", &pulses, PRESET);
        let got = decode(&mut HondaV2V3Decoder::new(), &pulses);
        assert_eq!(got.len(), 1);
        assert_eq!((got[0].serial, got[0].button, got[0].counter), (Some(0), Some(0), Some(0x41 >> 2)));
    }

    #[test]
    fn honda_v2_v3_gating() {
        // V3: a corrupted CRC, a wrong sync byte and a short frame are rejected.
        let ok = v3_frame(0x3D, [0xAB, 0xCD, 0x8C, 0x82, 1, 2, 3, 4, 5, 6, 7, 8]);
        let mut bad_crc = ok.clone();
        bad_crc[14] ^= 0x01;
        let p = manchester_burst(&bad_crc, 17, 250, 4000);
        dump("hondav2v3_v3_bad_crc", &p, PRESET);
        assert!(decode(&mut HondaV2V3Decoder::new(), &p).is_empty());
        let mut bad_sync = ok.clone();
        bad_sync[0] = 0x6B;
        let p = manchester_burst(&bad_sync, 17, 250, 4000);
        dump("hondav2v3_v3_wrong_sync", &p, PRESET);
        assert!(decode(&mut HondaV2V3Decoder::new(), &p).is_empty());
        let p = manchester_burst(&ok[..10], 17, 250, 4000);
        dump("hondav2v3_v3_short_frame", &p, PRESET);
        assert!(decode(&mut HondaV2V3Decoder::new(), &p).is_empty());
        // V2: an unknown button pair, a damaged sync (and no valid pair) and a short frame.
        let p = manchester_burst(&v2_payload((0x42, 0x40)), 17, 96, 3000);
        dump("hondav2v3_v2_bad_buttons", &p, PRESET);
        assert!(decode(&mut HondaV2V3Decoder::new(), &p).is_empty());
        let mut bad = v2_payload((0x41, 0x40));
        bad[2] ^= 0x10;
        bad[9] = 0x00; // no fallback pair either
        let p = manchester_burst(&bad, 17, 96, 3000);
        dump("hondav2v3_v2_wrong_sync", &p, PRESET);
        assert!(decode(&mut HondaV2V3Decoder::new(), &p).is_empty());
        let p = manchester_burst(&v2_payload((0x41, 0x40))[..9], 17, 96, 3000);
        dump("hondav2v3_v2_short_frame", &p, PRESET);
        assert!(decode(&mut HondaV2V3Decoder::new(), &p).is_empty());
    }

    #[test]
    fn v3_crc_fold_has_the_constant_term() {
        // All-zero frame: the CRC-16 is 0, every parity is 0, so the result is the constant.
        assert_eq!(v3_compute_crc(&[0u8; 15]), 0x1B);
        assert!(v2_validate_buttons(0x40, 0x60));
        assert!(!v2_validate_buttons(0x60, 0x40));
    }
}
