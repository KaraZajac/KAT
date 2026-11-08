//! "Fiat Multi" (Fiat V3 / V5) protocol decoder (decode-only)
//!
//! Ported from FoxFW: `REFERENCES/FoxFW/lib/subghz/protocols/fiat_v2_v3_v5.c`
//! (`SUBGHZ_PROTOCOL_FIAT_NAME = "Fiat Multi"`, `433 | AM`, Dynamic; the C's encoder lives in an SD
//! plugin and is not ported). One registration with two variants chosen from the first pulse. Not
//! KAT's "Fiat V0" (64-bit, 100/200 µs), "Fiat V1" (Hitag2) or "Fiat V2" (Hitag2 112-bit); the
//! FoxFW file is called `fiat_v2_v3_v5` but its code only has the V3 and V5 frames.
//!
//! # Variant detection
//! In `Reset` the first pulse picks the variant whose `te_short` it is closest to (within the
//! variant's own tolerance, V3 wins an exact tie): **V3** `te` 460/920 ± 200 µs, **V5** `te`
//! 200/400 ± 100 µs. A pulse in both ranges (260..300 µs) goes to V5. Manchester bits then come
//! from the PCF-style table `[0x01 0x91 0x9B 0xFB]` with `level ? ShortLow : ShortHigh` (long
//! likewise), "same state gives no bit". A gap longer than the variant's `min_gap` (V3 1500 µs,
//! V5 2000 µs), or any pulse that is neither short nor long, ends the frame; the frame is checked
//! only if it has at least 60 (V3) / 90 (V5) bits. Bits after a run of leading ones (the preamble)
//! form the frame.
//!
//! # V3 frame (70 bits): `fix:22 | btn:2 | status:2 | hop:44`
//! Gates (all from the C): 4..=20 preamble ones; fewer than 100 bits collected; at most 170 valid
//! timings; the average LOW pulse before the first 0 bit at least 400 µs; for a complete 70-bit
//! frame `hop[43:42] == 00` and `hop[0] == 1`; the hop's linear checks: the XOR of hop bits
//! `{4,5,12,13,20,21,28,29,36,37}` must equal the frame's button, and four invariant masks must
//! XOR to 0 (a self-check that holds for real hops). `crc_valid` = true for every V3 frame the
//! decoder reports (the C only emits frames that pass).
//! Buttons: 0 Lock, 1 Unlock, (2 Trunk is named by the C but the hop check gives only 0 or 1).
//!
//! # V5 frame (97 bits): `fix:32 | type:8 | btn:8 | seed:8 | cnt:8 | hop:32 | trail:1`
//! Gates: at least 10 preamble ones, at least 90 data bits (a short frame is decoded from what
//! there is), button one of `0x80` Open, `0x40` Trunk, `0x20` Close; for a complete 97-bit frame the
//! hop's lowest bit (the C tests `hop & 1`, i.e. frame bit 95, not the trailing bit) must be 1.
//! There is no checksum: `crc_valid` is false.
//!
//! # DecodedSignal packing
//! - V3: `data` = `fix<<42 | btn<<40 | hop[43:4]` (the C's packing), `data_count_bit` = 70,
//!   `extra` = `status<<4 | hop[3:0]`, `serial` = fix, `button` = btn.
//! - V5: `data` = `fix<<32 | hop`, `data_count_bit` = 97, `extra` = `type<<8 | seed`, `serial` =
//!   fix, `button` = btn, `counter` = cnt.
//!   The variant is told by `data_count_bit` (70 / 97).
//!
//! # Encoder
//! None here (`supports_encoding()` false).
//!
//! # False matches
//! FoxFW's registry order puts "Fiat V2" and the VAG decoders before this one, as KAT's does.

use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Fiat Multi",
    decoder: "Fiat Multi",
    aliases: &["Fiat V3", "Fiat V5"],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Rolling",
    make: "Fiat",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// V3: 0 Lock, 1 Unlock, 2 Trunk. V5: 0x80 Open, 0x40 Trunk, 0x20 Close.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x00 => "Lock",
        0x01 => "Unlock",
        0x02 => "Trunk",
        0x80 => "Open",
        0x40 => "Trunk",
        0x20 => "Close",
        _ => "Other",
    })
}

// V3
const V3_TE_SHORT: u32 = 460;
const V3_TE_LONG: u32 = 920;
const V3_TE_DELTA: u32 = 200;
const V3_MIN_GAP: u32 = 1500;
const V3_FRAME_BITS: usize = 70;
const V3_MIN_FRAME_BITS: usize = 60;
const V3_MAX_BITS: usize = 100;
const V3_PREAMBLE_MIN: usize = 4;
const V3_PREAMBLE_MAX: usize = 20;
const V3_MAX_TIMINGS: u32 = 170;
const V3_MIN_PREAMBLE_GAP: u32 = 400;
// V5
const V5_TE_SHORT: u32 = 200;
const V5_TE_LONG: u32 = 400;
const V5_TE_DELTA: u32 = 100;
const V5_MIN_GAP: u32 = 2000;
const V5_FRAME_BITS: usize = 97;
const V5_MIN_FRAME_BITS: usize = 90;
const V5_MAX_BITS: usize = 130;
const V5_PREAMBLE_MIN: usize = 10;

const MANCHESTER_TABLE: [u8; 4] = [0x01, 0x91, 0x9B, 0xFB];

#[derive(Debug, Clone, Copy, PartialEq)]
enum Variant {
    V3,
    V5,
}

impl Variant {
    fn te_short(self) -> u32 {
        match self {
            Variant::V3 => V3_TE_SHORT,
            Variant::V5 => V5_TE_SHORT,
        }
    }
    fn te_long(self) -> u32 {
        match self {
            Variant::V3 => V3_TE_LONG,
            Variant::V5 => V5_TE_LONG,
        }
    }
    fn te_delta(self) -> u32 {
        match self {
            Variant::V3 => V3_TE_DELTA,
            Variant::V5 => V5_TE_DELTA,
        }
    }
    fn min_gap(self) -> u32 {
        match self {
            Variant::V3 => V3_MIN_GAP,
            Variant::V5 => V5_MIN_GAP,
        }
    }
    fn max_bits(self) -> usize {
        match self {
            Variant::V3 => V3_MAX_BITS,
            Variant::V5 => V5_MAX_BITS,
        }
    }
    fn min_frame_bits(self) -> usize {
        match self {
            Variant::V3 => V3_MIN_FRAME_BITS,
            Variant::V5 => V5_MIN_FRAME_BITS,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    Data,
}

/// XOR (parity) of the bits of `v` selected by `mask`.
fn parity(v: u64, mask: u64) -> u8 {
    ((v & mask).count_ones() & 1) as u8
}

/// `fiat_v3_hop_btn_extract`: XOR of hop bits {4,5,12,13,20,21,28,29,36,37}.
fn v3_hop_btn_extract(hop: u64) -> u8 {
    let mask = (3u64 << 4) | (3 << 12) | (3 << 20) | (3 << 28) | (3 << 36);
    parity(hop, mask)
}

/// `fiat_v3_hop_invariants_check`: the four cipher invariants all XOR to 0.
fn v3_hop_invariants_check(hop: u64) -> bool {
    let r1 = (3u64 << 5) | (3 << 13) | (3 << 21) | (3 << 29) | (3 << 37);
    let r2 = (5u64 << 5) | (5 << 13) | (5 << 21) | (5 << 29) | (5 << 37);
    let r3 = (1u64 << 5)
        | (1 << 8)
        | (1 << 13)
        | (1 << 16)
        | (1 << 21)
        | (1 << 24)
        | (1 << 29)
        | (1 << 32)
        | (1 << 37)
        | (1 << 40);
    let r4 = (1u64 << 1) | (1 << 9) | (1 << 17) | (1 << 25) | (1 << 33) | (1 << 41);
    [r1, r2, r3, r4].iter().all(|&m| parity(hop, m) == 0)
}

/// Fiat Multi decoder (matches `SubGhzProtocolDecoderFiat`).
pub struct FiatMultiDecoder {
    step: Step,
    variant: Variant,
    manchester_state: u8,
    bits: [u8; 150],
    bit_count: usize,
    // V3 timing-quality tracking (`extra.v3`)
    timing_count: u32,
    preamble_gap_sum: u32,
    preamble_gap_count: u32,
    preamble_ended: bool,
}

impl FiatMultiDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            variant: Variant::V3,
            manchester_state: 1,
            bits: [0; 150],
            bit_count: 0,
            timing_count: 0,
            preamble_gap_sum: 0,
            preamble_gap_count: 0,
            preamble_ended: false,
        }
    }

    /// `fiat_manchester_decode` (timing types 0..6 only; the C's type 8 never occurs here).
    fn manchester_decode(&mut self, timing_type: u8) -> Option<u8> {
        if self.manchester_state >= 4 {
            self.manchester_state = 1;
            return None;
        }
        let entry = MANCHESTER_TABLE[self.manchester_state as usize];
        let new_state = (entry >> timing_type) & 0x3;
        if self.manchester_state == new_state {
            return None;
        }
        self.manchester_state = new_state;
        match new_state {
            2 => Some(0),
            1 => Some(1),
            _ => None,
        }
    }

    fn clear_v3(&mut self) {
        self.timing_count = 0;
        self.preamble_gap_sum = 0;
        self.preamble_gap_count = 0;
        self.preamble_ended = false;
    }

    /// Back to `Reset` with the frame state cleared (the C's `memset(&extra, 0, ..)` paths).
    fn restart(&mut self) {
        self.step = Step::Reset;
        self.bit_count = 0;
        self.manchester_state = 1;
        self.clear_v3();
    }

    /// `fiat_process_frame`.
    fn process_frame(&self) -> Option<DecodedSignal> {
        let mut start = 0;
        while start < self.bit_count && self.bits[start] == 1 {
            start += 1;
        }
        let preamble_len = start;
        let mut data_len = self.bit_count - start;
        let field = |from: usize, to: usize, data_len: usize| -> u64 {
            let mut v = 0u64;
            for i in from..to.min(data_len) {
                v = (v << 1) | self.bits[start + i] as u64;
            }
            v
        };
        match self.variant {
            Variant::V3 => {
                if preamble_len < V3_PREAMBLE_MIN || data_len < V3_MIN_FRAME_BITS {
                    return None;
                }
                if preamble_len > V3_PREAMBLE_MAX {
                    return None;
                }
                if self.bit_count >= V3_MAX_BITS {
                    return None;
                }
                if self.timing_count > V3_MAX_TIMINGS {
                    return None;
                }
                if let Some(avg_gap) = self.preamble_gap_sum.checked_div(self.preamble_gap_count) {
                    if avg_gap < V3_MIN_PREAMBLE_GAP {
                        return None;
                    }
                }
                if data_len > V3_FRAME_BITS {
                    data_len = V3_FRAME_BITS;
                }
                let fix = field(0, 22, data_len) as u32;
                let btn = field(22, 24, data_len) as u8;
                let status = field(24, 26, data_len) as u8;
                let hop = field(26, 70, data_len);

                // Complete frames: hop[43:42] must be 00 and the stop bit hop[0] 1.
                if data_len >= V3_FRAME_BITS && ((hop >> 42) != 0 || (hop & 1) != 1) {
                    return None;
                }
                if v3_hop_btn_extract(hop) != btn {
                    return None;
                }
                if !v3_hop_invariants_check(hop) {
                    return None;
                }
                let data = ((fix as u64 & 0x3F_FFFF) << 42)
                    | ((btn as u64 & 0x3) << 40)
                    | ((hop >> 4) & 0xFF_FFFF_FFFF);
                Some(DecodedSignal {
                    serial: Some(fix),
                    button: Some(btn),
                    counter: None,
                    crc_valid: true,
                    data,
                    data_count_bit: V3_FRAME_BITS,
                    encoder_capable: false,
                    extra: Some(((status as u64) << 4) | (hop & 0xF)),
                    protocol_display_name: None,
                })
            }
            Variant::V5 => {
                if preamble_len < V5_PREAMBLE_MIN || data_len < V5_MIN_FRAME_BITS {
                    return None;
                }
                if data_len > V5_FRAME_BITS {
                    data_len = V5_FRAME_BITS;
                }
                let fix = field(0, 32, data_len) as u32;
                let type_byte = field(32, 40, data_len) as u8;
                let btn = field(40, 48, data_len) as u8;
                let seed = field(48, 56, data_len) as u8;
                let cnt = field(56, 64, data_len) as u8;
                let hop = field(64, 96, data_len) as u32;

                if !matches!(btn, 0x80 | 0x40 | 0x20) {
                    return None;
                }
                if data_len >= V5_FRAME_BITS && (hop & 1) != 1 {
                    return None;
                }
                Some(DecodedSignal {
                    serial: Some(fix),
                    button: Some(btn),
                    counter: Some(cnt as u16),
                    crc_valid: false,
                    data: ((fix as u64) << 32) | hop as u64,
                    data_count_bit: V5_FRAME_BITS,
                    encoder_capable: false,
                    extra: Some(((type_byte as u64) << 8) | seed as u64),
                    protocol_display_name: None,
                })
            }
        }
    }

    /// `fiat_decode_timing`.
    fn decode_timing(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let v = self.variant;
        let timing_type = if duration_diff!(duration, v.te_short()) <= v.te_delta() {
            Some(if level { 0 } else { 2 })
        } else if duration_diff!(duration, v.te_long()) <= v.te_delta() {
            Some(if level { 4 } else { 6 })
        } else {
            None
        };

        let Some(timing_type) = timing_type else {
            let out = if self.bit_count >= v.min_frame_bits() { self.process_frame() } else { None };
            self.restart();
            return out;
        };

        if v == Variant::V3 {
            self.timing_count += 1;
            if !level && !self.preamble_ended {
                self.preamble_gap_sum += duration;
                self.preamble_gap_count += 1;
            }
            if self.timing_count > V3_MAX_TIMINGS {
                self.step = Step::Reset;
                self.bit_count = 0;
                self.manchester_state = 1;
                self.clear_v3();
                return None;
            }
        }

        if let Some(bit) = self.manchester_decode(timing_type) {
            if self.bit_count < v.max_bits() {
                self.bits[self.bit_count] = bit;
                self.bit_count += 1;
            }
            if v == Variant::V3 && bit == 0 && !self.preamble_ended {
                self.preamble_ended = true;
            }
            if self.bit_count >= v.max_bits() {
                let out = self.process_frame();
                self.restart();
                return out;
            }
        }
        None
    }
}

impl Default for FiatMultiDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for FiatMultiDecoder {
    fn name(&self) -> &'static str {
        "Fiat Multi"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: V5_TE_SHORT,
            te_long: V5_TE_LONG,
            te_delta: V5_TE_DELTA,
            min_count_bit: V5_MIN_FRAME_BITS,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        // C flags: SubGhzProtocolFlag_433 | SubGhzProtocolFlag_AM.
        &[433_920_000]
    }

    fn reset(&mut self) {
        self.step = Step::Reset;
        self.manchester_state = 1;
        self.bit_count = 0;
        self.clear_v3();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                // Best-fit te_short among the variants (V3 first, strictly closer wins).
                let mut best: Option<(Variant, u32)> = None;
                for v in [Variant::V3, Variant::V5] {
                    let diff = duration_diff!(duration, v.te_short());
                    if diff <= v.te_delta() && best.is_none_or(|(_, b)| diff < b) {
                        best = Some((v, diff));
                    }
                }
                if let Some((v, _)) = best {
                    self.variant = v;
                    self.step = Step::Data;
                    self.manchester_state = 1;
                    self.bit_count = 0;
                    self.clear_v3();
                    return self.decode_timing(level, duration);
                }
                None
            }
            Step::Data => {
                if duration > self.variant.min_gap() {
                    let out = if self.bit_count >= self.variant.min_frame_bits() {
                        self.process_frame()
                    } else {
                        None
                    };
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
    use super::*;

    // Test vectors: made-up field values, built as pulse sequences the way the C expects them, written
    // as Flipper RAW files and decoded by the FoxFW C decoder (host oracle, `--registry all
    // --protocol "Fiat Multi"`); the expectations below are the fields the oracle printed. Levels
    // alternate from `.0`.

    /// `f3_ok`: first pulse HIGH, 130 pulses.
    const F3_OK: (bool, &[u32]) = (
        true,
        &[
        460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460,
        460, 460, 460, 460, 460, 460, 460, 460, 460, 920, 920, 460, 460, 920, 920, 920,
        460, 460, 460, 460, 920, 920, 920, 920, 920, 460, 460, 920, 460, 460, 920, 460,
        460, 460, 460, 460, 460, 920, 460, 460, 460, 460, 920, 460, 460, 920, 460, 460,
        460, 460, 920, 920, 920, 460, 460, 460, 460, 460, 460, 460, 460, 920, 920, 920,
        920, 460, 460, 920, 460, 460, 920, 920, 920, 460, 460, 920, 460, 460, 460, 460,
        460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 920, 920, 460, 460, 460, 460,
        920, 920, 460, 460, 460, 460, 460, 460, 920, 920, 920, 920, 460, 460, 920, 460,
        460, 3000,
        ],
    );

    /// `f3_ok0`: first pulse HIGH, 110 pulses.
    const F3_OK0: (bool, &[u32]) = (
        true,
        &[
        460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 920, 460, 460,
        920, 460, 460, 920, 920, 920, 920, 920, 920, 460, 460, 460, 460, 460, 460, 920,
        460, 460, 920, 460, 460, 460, 460, 920, 460, 460, 460, 460, 920, 920, 460, 460,
        460, 460, 920, 920, 460, 460, 920, 920, 920, 460, 460, 920, 920, 920, 920, 920,
        920, 920, 460, 460, 920, 460, 460, 920, 920, 920, 920, 460, 460, 920, 920, 460,
        460, 920, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460,
        920, 920, 920, 920, 460, 460, 920, 920, 920, 920, 920, 920, 920, 3000,
        ],
    );

    /// `f5_ok`: first pulse HIGH, 180 pulses.
    const F5_OK: (bool, &[u32]) = (
        true,
        &[
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 200,
        400, 400, 400, 400, 400, 200, 200, 200, 200, 400, 200, 200, 400, 400, 400, 400,
        200, 200, 400, 400, 400, 400, 200, 200, 200, 200, 400, 200, 200, 200, 200, 200,
        200, 400, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 400, 400, 200, 200, 400, 400, 400, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 200, 200,
        200, 400, 400, 400, 200, 200, 400, 200, 200, 400, 400, 200, 200, 200, 200, 200,
        200, 400, 400, 400, 200, 200, 400, 400, 400, 200, 200, 200, 200, 400, 200, 200,
        200, 200, 200, 200, 400, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 200,
        400, 200, 200, 400, 400, 400, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        400, 200, 200, 3000,
        ],
    );

    /// `f5_open`: first pulse HIGH, 168 pulses.
    const F5_OPEN: (bool, &[u32]) = (
        true,
        &[
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 400, 200, 200, 200, 200, 200, 200, 400, 200, 200, 200,
        200, 400, 200, 200, 200, 200, 200, 200, 400, 200, 200, 200, 200, 200, 200, 200,
        200, 400, 200, 200, 400, 400, 400, 400, 400, 400, 200, 200, 200, 200, 400, 200,
        200, 200, 200, 400, 400, 200, 200, 400, 200, 200, 200, 200, 400, 400, 200, 200,
        400, 400, 400, 400, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 400, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 200, 400, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 200, 400, 400,
        400, 200, 200, 400, 400, 400, 200, 200, 400, 400, 400, 200, 200, 400, 400, 400,
        200, 200, 400, 400, 400, 200, 200, 400, 400, 400, 200, 200, 400, 400, 400, 200,
        200, 400, 400, 200, 200, 200, 200, 3000,
        ],
    );

    /// `f3_badinv`: first pulse HIGH, 128 pulses.
    const F3_BADINV: (bool, &[u32]) = (
        true,
        &[
        460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460,
        460, 460, 460, 460, 460, 460, 460, 460, 460, 920, 920, 460, 460, 920, 920, 920,
        460, 460, 460, 460, 920, 920, 920, 920, 920, 460, 460, 920, 460, 460, 920, 460,
        460, 460, 460, 460, 460, 920, 460, 460, 460, 460, 920, 460, 460, 920, 460, 460,
        460, 460, 920, 920, 920, 460, 460, 460, 460, 460, 460, 460, 460, 920, 920, 920,
        920, 460, 460, 920, 460, 460, 920, 920, 920, 460, 460, 920, 460, 460, 460, 460,
        460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 920, 920, 460, 460, 460, 460,
        920, 920, 920, 920, 460, 460, 920, 920, 920, 920, 460, 460, 920, 460, 460, 3000,
        ],
    );

    /// `f3_badbtn`: first pulse HIGH, 130 pulses.
    const F3_BADBTN: (bool, &[u32]) = (
        true,
        &[
        460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460,
        460, 460, 460, 460, 460, 460, 460, 460, 460, 920, 920, 460, 460, 920, 920, 920,
        460, 460, 460, 460, 920, 920, 920, 920, 920, 460, 460, 920, 460, 460, 920, 460,
        460, 460, 460, 460, 460, 920, 460, 460, 460, 460, 460, 460, 920, 920, 460, 460,
        460, 460, 920, 920, 920, 460, 460, 460, 460, 460, 460, 460, 460, 920, 920, 920,
        920, 460, 460, 920, 460, 460, 920, 920, 920, 460, 460, 920, 460, 460, 460, 460,
        460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 920, 920, 460, 460, 460, 460,
        920, 920, 460, 460, 460, 460, 460, 460, 920, 920, 920, 920, 460, 460, 920, 460,
        460, 3000,
        ],
    );

    /// `f3_hop_hi`: first pulse HIGH, 128 pulses.
    const F3_HOP_HI: (bool, &[u32]) = (
        true,
        &[
        460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460,
        460, 460, 460, 460, 460, 460, 460, 460, 460, 920, 920, 460, 460, 920, 920, 920,
        460, 460, 460, 460, 920, 920, 920, 920, 920, 460, 460, 920, 460, 460, 920, 460,
        460, 460, 460, 460, 460, 920, 460, 460, 460, 460, 920, 460, 460, 920, 920, 920,
        920, 920, 920, 460, 460, 460, 460, 460, 460, 460, 460, 920, 920, 920, 920, 460,
        460, 920, 460, 460, 920, 920, 920, 460, 460, 920, 460, 460, 460, 460, 460, 460,
        460, 460, 460, 460, 460, 460, 460, 460, 920, 920, 460, 460, 460, 460, 920, 920,
        460, 460, 460, 460, 460, 460, 920, 920, 920, 920, 460, 460, 920, 460, 460, 3000,
        ],
    );

    /// `f3_nostop`: first pulse HIGH, 130 pulses.
    const F3_NOSTOP: (bool, &[u32]) = (
        true,
        &[
        460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 460,
        460, 460, 460, 460, 460, 460, 460, 460, 460, 920, 920, 460, 460, 920, 920, 920,
        460, 460, 460, 460, 920, 920, 920, 920, 920, 460, 460, 920, 460, 460, 920, 460,
        460, 460, 460, 460, 460, 920, 460, 460, 460, 460, 920, 460, 460, 920, 460, 460,
        460, 460, 920, 920, 920, 460, 460, 460, 460, 460, 460, 460, 460, 920, 920, 920,
        920, 460, 460, 920, 460, 460, 920, 920, 920, 460, 460, 920, 460, 460, 460, 460,
        460, 460, 460, 460, 460, 460, 460, 460, 460, 460, 920, 920, 460, 460, 460, 460,
        920, 920, 460, 460, 460, 460, 460, 460, 920, 920, 920, 920, 460, 460, 920, 920,
        460, 3000,
        ],
    );

    /// `f3_shortpre`: first pulse HIGH, 110 pulses.
    const F3_SHORTPRE: (bool, &[u32]) = (
        true,
        &[
        460, 460, 460, 460, 460, 920, 920, 460, 460, 920, 920, 920, 460, 460, 460, 460,
        920, 920, 920, 920, 920, 460, 460, 920, 460, 460, 920, 460, 460, 460, 460, 460,
        460, 920, 460, 460, 460, 460, 920, 460, 460, 920, 460, 460, 460, 460, 920, 920,
        920, 460, 460, 460, 460, 460, 460, 460, 460, 920, 920, 920, 920, 460, 460, 920,
        460, 460, 920, 920, 920, 460, 460, 920, 460, 460, 460, 460, 460, 460, 460, 460,
        460, 460, 460, 460, 460, 460, 920, 920, 460, 460, 460, 460, 920, 920, 460, 460,
        460, 460, 460, 460, 920, 920, 920, 920, 460, 460, 920, 460, 460, 3000,
        ],
    );

    /// `f5_badbtn`: first pulse HIGH, 180 pulses.
    const F5_BADBTN: (bool, &[u32]) = (
        true,
        &[
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 200,
        400, 400, 400, 400, 400, 200, 200, 200, 200, 400, 200, 200, 400, 400, 400, 400,
        200, 200, 400, 400, 400, 400, 200, 200, 200, 200, 400, 200, 200, 200, 200, 200,
        200, 400, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 400, 400, 200, 200, 400, 200, 200, 200, 200, 400, 400,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 200, 200,
        200, 400, 400, 400, 200, 200, 400, 200, 200, 400, 400, 200, 200, 200, 200, 200,
        200, 400, 400, 400, 200, 200, 400, 400, 400, 200, 200, 200, 200, 400, 200, 200,
        200, 200, 200, 200, 400, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 200,
        400, 200, 200, 400, 400, 400, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        400, 200, 200, 3000,
        ],
    );

    /// `f5_nostop`: first pulse HIGH, 180 pulses.
    const F5_NOSTOP: (bool, &[u32]) = (
        true,
        &[
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 200,
        400, 400, 400, 400, 400, 200, 200, 200, 200, 400, 200, 200, 400, 400, 400, 400,
        200, 200, 400, 400, 400, 400, 200, 200, 200, 200, 400, 200, 200, 200, 200, 200,
        200, 400, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 400, 400, 200, 200, 400, 400, 400, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 200, 200,
        200, 400, 400, 400, 200, 200, 400, 200, 200, 400, 400, 200, 200, 200, 200, 200,
        200, 400, 400, 400, 200, 200, 400, 400, 400, 200, 200, 200, 200, 400, 200, 200,
        200, 200, 200, 200, 400, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 200,
        400, 200, 200, 400, 400, 400, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 400, 3000,
        ],
    );

    /// `f5_shortpre`: first pulse HIGH, 168 pulses.
    const F5_SHORTPRE: (bool, &[u32]) = (
        true,
        &[
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 400, 200, 200, 400, 400, 400, 400, 400, 200, 200, 200, 200, 400, 200, 200,
        400, 400, 400, 400, 200, 200, 400, 400, 400, 400, 200, 200, 200, 200, 400, 200,
        200, 200, 200, 200, 200, 400, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 400, 400, 200, 200, 400, 400, 400,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        400, 200, 200, 200, 200, 400, 400, 400, 200, 200, 400, 200, 200, 400, 400, 200,
        200, 200, 200, 200, 200, 400, 400, 400, 200, 200, 400, 400, 400, 200, 200, 200,
        200, 400, 200, 200, 200, 200, 200, 200, 400, 200, 200, 200, 200, 200, 200, 200,
        200, 400, 200, 200, 400, 200, 200, 400, 400, 400, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 400, 200, 200, 3000,
        ],
    );

    /// `f5_short`: first pulse HIGH, 166 pulses.
    const F5_SHORT: (bool, &[u32]) = (
        true,
        &[
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 200,
        400, 400, 400, 400, 400, 200, 200, 200, 200, 400, 200, 200, 400, 400, 400, 400,
        200, 200, 400, 400, 400, 400, 200, 200, 200, 200, 400, 200, 200, 200, 200, 200,
        200, 400, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 400, 400, 200, 200, 400, 400, 400, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 200, 200,
        200, 400, 400, 400, 200, 200, 400, 200, 200, 400, 400, 200, 200, 200, 200, 200,
        200, 400, 400, 400, 200, 200, 400, 400, 400, 200, 200, 200, 200, 400, 200, 200,
        200, 200, 200, 200, 400, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 200,
        400, 200, 200, 400, 400, 3000,
        ],
    );

    fn run(v: (bool, &[u32])) -> Vec<DecodedSignal> {
        let mut dec = FiatMultiDecoder::new();
        let mut level = v.0;
        let mut out = Vec::new();
        for &d in v.1 {
            if let Some(s) = dec.feed(level, d) {
                out.push(s);
            }
            level = !level;
        }
        out
    }

    #[test]
    fn decodes_the_oracle_frames() {
        assert_eq!(run(F3_OK), vec![DecodedSignal { serial: Some(0x1a2b3c), button: Some(0x1), counter: None, crc_valid: true, data: 0x68acf12facb00885, data_count_bit: 70, encoder_capable: false, extra: Some(0x23), protocol_display_name: None }], "f3_ok");
        assert_eq!(run(F3_OK0), vec![DecodedSignal { serial: Some(0xd5e71), button: Some(0x0), counter: None, crc_valid: true, data: 0x3579c42d535b00a5, data_count_bit: 70, encoder_capable: false, extra: Some(0x15), protocol_display_name: None }], "f3_ok0");
        assert_eq!(run(F5_OK), vec![DecodedSignal { serial: Some(0x2b94a3c1), button: Some(0x40), counter: Some(0x37), crc_valid: false, data: 0x2b94a3c1a5c3e681, data_count_bit: 97, encoder_capable: false, extra: Some(0xfb1d), protocol_display_name: None }], "f5_ok");
        assert_eq!(run(F5_OPEN), vec![DecodedSignal { serial: Some(0xe1f2a3b), button: Some(0x80), counter: Some(0xfe), crc_valid: false, data: 0x0e1f2a3b5a5a5a5b, data_count_bit: 97, encoder_capable: false, extra: Some(0x127c), protocol_display_name: None }], "f5_open");
    }

    #[test]
    fn rejects_what_the_c_rejects() {
        assert!(run(F3_BADINV).is_empty(), "f3_badinv: a flipped hop bit breaks the invariants");
        assert!(run(F3_BADBTN).is_empty(), "f3_badbtn: hop button parity != frame button");
        assert!(run(F3_HOP_HI).is_empty(), "f3_hop_hi: hop[43:42] != 00");
        assert!(run(F3_NOSTOP).is_empty(), "f3_nostop: hop[0] != 1");
        assert!(run(F3_SHORTPRE).is_empty(), "f3_shortpre: 3 preamble ones");
        assert!(run(F5_BADBTN).is_empty(), "f5_badbtn: button 0x10");
        assert!(run(F5_NOSTOP).is_empty(), "f5_nostop: hop[0] == 0");
        assert!(run(F5_SHORTPRE).is_empty(), "f5_shortpre: 9 preamble ones");
        assert!(run(F5_SHORT).is_empty(), "f5_short: 89 data bits");
    }

    #[test]
    fn decode_only() {
        let d = FiatMultiDecoder::new();
        assert!(!d.supports_encoding());
        assert!(d.encode(&DecodedSignal::new(0, 0), 1).is_none());
    }
}
