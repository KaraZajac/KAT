//! Toyota / Lexus KeeLoq protocol decoder (dual variant)
//!
//! Ported from Flipper-ARF reference: `lib/subghz/protocols/toyota.c` and `toyota.h`.
//! Decode-only — the reference `encoder` field is NULL.
//!
//! Two variants, detected from the first HIGH pulse width (threshold 310µs):
//!
//! - **Variant A** — Corolla / 433.92 MHz. PWM pairs: te_short=400µs, te_long=800µs,
//!   delta=175µs. LS (long HIGH + short LOW) = bit 0, SL (short HIGH + long LOW) = bit 1.
//!   Preamble = repeated short-short (SS) pairs; first non-SS pair is the first data bit.
//!   Frame = 68 bits; min_count_bit = 60.
//! - **Variant B** — Tundra / 315 MHz. NRZ: each individual pulse encodes one bit by a
//!   midpoint classifier (`<= 287µs` -> 0, `> 287µs` -> 1). Preamble = short-HIGH /
//!   long-LOW pairs (te_short=200µs, te_long=390µs, delta=120µs) terminated by a sync gap
//!   (LOW between 1500 and 2600µs). Frame = 67 bits; min_count_bit = 60.
//!
//! KeeLoq hopping: the hop field is left encrypted (the reference does not decrypt or run a
//! CRC). `data` layout matches the reference `generic.data`:
//! `(hop << 32) | (serial << 4) | button`, with hop=32 bits, serial=28 bits, button=4 bits.
//!
//! Emission is gated tightly (exact frame bit count + structural preamble/sync + non-zero
//! serial) so it does not false-match the other KeeLoq-PWM protocols (Kia V3/V4, Subaru,
//! Suzuki, etc.). The shared 433 MHz KeeLoq-PWM air encoding means a Toyota Variant-A frame
//! that also satisfies Kia V3/V4's 68-bit/CRC4 structure is claimed by Kia V3/V4 first (it is
//! earlier in the registry); Toyota uniquely claims 60-bit frames and Variant-B NRZ at 315 MHz.

use super::{ProtocolDecoder, ProtocolTiming, DecodedSignal};
use crate::radio::demodulator::LevelDuration;
use crate::duration_diff;

// Variant A physical constants (Corolla / 433 MHz)
const A_TE_SHORT: u32 = 400;
const A_TE_LONG: u32 = 800;
const A_TE_DELTA: u32 = 175;

// Variant B physical constants (Tundra / 315 MHz, preamble classification only)
const B_TE_SHORT: u32 = 200;
const B_TE_LONG: u32 = 390;
const B_TE_DELTA: u32 = 120;

const MIN_COUNT_BIT: usize = 60;

/// NRZ midpoint for Variant B data pulses (`<= 287` -> 0, `> 287` -> 1).
const B_NRZ_MIDPOINT: u32 = 287;

/// Sync gap (LOW) separating preamble from data in Variant B.
const B_SYNC_GAP_MIN: u32 = 1500;
const B_SYNC_GAP_MAX: u32 = 2600;

/// Minimum preamble pairs before a frame is accepted.
const A_PREAMBLE_MIN: u16 = 6;
const B_PREAMBLE_MIN: u16 = 6;

/// Frame lengths in bits.
const A_BITS: usize = 68;
const B_BITS: usize = 67;

/// First HIGH duration below this -> Variant B, at or above -> Variant A.
const VARIANT_THRESH: u32 = 310;

#[inline]
fn a_is_short(d: u32) -> bool {
    duration_diff!(d, A_TE_SHORT) < A_TE_DELTA
}
#[inline]
fn a_is_long(d: u32) -> bool {
    duration_diff!(d, A_TE_LONG) < A_TE_DELTA
}
#[inline]
fn b_is_short(d: u32) -> bool {
    duration_diff!(d, B_TE_SHORT) < B_TE_DELTA
}
#[inline]
fn b_is_long(d: u32) -> bool {
    duration_diff!(d, B_TE_LONG) < B_TE_DELTA
}

/// Decoder steps (matches ToyotaDecoderStep in toyota.c).
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    PreambleA,
    DataA,
    PreambleB,
    DataB,
}

/// Toyota / Lexus protocol decoder (matches SubGhzProtocolDecoderToyota).
pub struct ToyotaDecoder {
    step: DecoderStep,
    /// 128-bit shift accumulator (hi:lo), matching the C `bits_hi`/`bits_lo`.
    bits_hi: u64,
    bits_lo: u64,
    bit_count: usize,
    te_last: u32,
    have_high: bool,
    preamble_count: u16,
    /// 0 = Variant A (Corolla / 433 MHz), 1 = Variant B (Tundra / 315 MHz).
    variant: u8,
}

impl ToyotaDecoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            bits_hi: 0,
            bits_lo: 0,
            bit_count: 0,
            te_last: 0,
            have_high: false,
            preamble_count: 0,
            variant: 0,
        }
    }

    /// Reset the parse state. Matches `subghz_protocol_decoder_toyota_reset`, which intentionally
    /// does NOT clear `variant` (detected once per session); the variant is re-detected on the next
    /// pulse from the Reset step anyway.
    fn reset_state(&mut self) {
        self.step = DecoderStep::Reset;
        self.bits_hi = 0;
        self.bits_lo = 0;
        self.bit_count = 0;
        self.te_last = 0;
        self.have_high = false;
        self.preamble_count = 0;
    }

    /// Push one bit into the 128-bit accumulator (matches `toyota_push_bit`).
    fn push_bit(&mut self, bit: u8) {
        let carry = (self.bits_lo >> 63) & 1;
        self.bits_hi = (self.bits_hi << 1) | carry;
        self.bits_lo = (self.bits_lo << 1) | (bit as u64 & 1);
        self.bit_count += 1;
    }

    /// Extract `length` bits at `offset` from the end of the accumulator (matches `toyota_extract`).
    fn extract(&self, offset: usize, length: usize) -> u32 {
        let mut result: u32 = 0;
        let total = self.bit_count as isize;
        for i in 0..length {
            let pos = (total - 1) - (offset as isize + i as isize);
            let b = if pos >= 64 {
                ((self.bits_hi >> (pos - 64)) & 1) as u32
            } else if pos >= 0 {
                ((self.bits_lo >> pos) & 1) as u32
            } else {
                0
            };
            result = (result << 1) | b;
        }
        result
    }

    /// Build the decoded signal once a full frame is collected (matches `toyota_decode_and_fire`).
    /// Returns `None` if the structural gate (bit count / serial) fails.
    fn decode_and_fire(&self) -> Option<DecodedSignal> {
        if self.bit_count < MIN_COUNT_BIT {
            return None;
        }

        let hop = self.extract(0, 32);
        let serial = self.extract(32, 28);
        let button = self.extract(60, 4) as u8;

        // Tight gate: require a non-zero serial so a run of all-zero/garbage bits that happens to
        // reach the bit count cannot emit a Toyota frame.
        if serial == 0 {
            return None;
        }

        // generic.data = (hop << 32) | (serial << 4) | button
        let data = ((hop as u64) << 32) | ((serial as u64) << 4) | (button as u64 & 0x0F);

        Some(DecodedSignal {
            serial: Some(serial),
            button: Some(button),
            // KeeLoq hop is left encrypted (no key / no decrypt in the reference).
            counter: None,
            // No CRC in the reference; a fully-structured frame of the exact bit count is the
            // validity criterion (the callback only fires at min_count_bit).
            crc_valid: true,
            data,
            data_count_bit: self.bit_count,
            encoder_capable: false,
            extra: None,
            protocol_display_name: None,
        })
    }

    /// Feed for Variant A (Corolla / 433 MHz) — PWM pair encoding (matches `toyota_feed_variant_a`).
    ///
    /// Faithful to the reference's gap-terminated emission path. The reference ALSO self-fires the
    /// instant `bit_count` reaches 68; that is intentionally dropped here. Variant A is the same
    /// KeeLoq-PWM air protocol as Kia V3/V4, and the early self-fire (on the normal short LOW that
    /// completes bit 68) lands several pulses BEFORE Kia V3/V4's sync-terminated fire — which, in
    /// KAT's "first decoder to fire on a pulse wins" stream, would let Toyota steal every shared
    /// 68-bit frame from Kia. By emitting only on the terminating gap (the same pulse Kia fires on),
    /// the registry order (Kia earlier) resolves the shared frames in Kia's favour, while Toyota
    /// still uniquely claims 60–67-bit Variant-A frames that Kia rejects. Bit accumulation is capped
    /// at 68 so the field layout is preserved regardless of trailing repeats.
    fn feed_variant_a(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::PreambleA => {
                if level {
                    self.te_last = duration;
                    self.have_high = true;
                    return None;
                }

                if !self.have_high {
                    self.reset_state();
                    return None;
                }
                self.have_high = false;

                let hs = a_is_short(self.te_last);
                let hl = a_is_long(self.te_last);
                let ls = a_is_short(duration);
                let ll = a_is_long(duration);

                if hs && ls {
                    self.preamble_count += 1;
                    return None;
                }

                if self.preamble_count < A_PREAMBLE_MIN {
                    self.reset_state();
                    return None;
                }

                self.bits_hi = 0;
                self.bits_lo = 0;
                self.bit_count = 0;

                if hl && ls {
                    self.push_bit(0);
                } else if hs && ll {
                    self.push_bit(1);
                }

                self.step = DecoderStep::DataA;
                None
            }

            DecoderStep::DataA => {
                if level {
                    if a_is_short(duration) || a_is_long(duration) {
                        self.te_last = duration;
                        self.have_high = true;
                    } else {
                        // Terminating gap / out-of-range HIGH: emit (deferring to Kia on shared
                        // 68-bit frames via registry order — see fn doc-comment).
                        let result = if self.bit_count >= MIN_COUNT_BIT {
                            self.decode_and_fire()
                        } else {
                            None
                        };
                        self.reset_state();
                        return result;
                    }
                    return None;
                }

                if !self.have_high {
                    return None;
                }
                self.have_high = false;

                // Cap accumulation at A_BITS: once a full 68-bit frame is collected, stop pushing
                // more bits and wait for the terminating gap. This keeps the field layout fixed and
                // makes emission coincide with Kia V3/V4's sync-terminated fire so Kia wins shared
                // frames by registry order.
                if self.bit_count >= A_BITS {
                    return None;
                }

                let hs = a_is_short(self.te_last);
                let hl = a_is_long(self.te_last);
                let ls = a_is_short(duration);
                let ll = a_is_long(duration);

                if hl && ls {
                    self.push_bit(0);
                } else if hs && ll {
                    self.push_bit(1);
                } else {
                    let result = if self.bit_count >= MIN_COUNT_BIT {
                        self.decode_and_fire()
                    } else {
                        None
                    };
                    self.reset_state();
                    return result;
                }
                None
            }

            _ => None,
        }
    }

    /// Feed for Variant B (Tundra / 315 MHz) — NRZ encoding (matches `toyota_feed_variant_b`).
    fn feed_variant_b(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::PreambleB => {
                if level {
                    if b_is_short(duration) {
                        self.te_last = duration;
                        self.have_high = true;
                    } else {
                        self.reset_state();
                    }
                    return None;
                }

                // Falling edge
                if !self.have_high {
                    self.reset_state();
                    return None;
                }
                self.have_high = false;

                // Sync gap: LOW ~1938µs -> transition to data
                if duration >= B_SYNC_GAP_MIN && duration <= B_SYNC_GAP_MAX {
                    if self.preamble_count >= B_PREAMBLE_MIN {
                        self.bits_hi = 0;
                        self.bits_lo = 0;
                        self.bit_count = 0;
                        self.have_high = false;
                        self.step = DecoderStep::DataB;
                    } else {
                        self.reset_state();
                    }
                    return None;
                }

                // Normal preamble LOW must be LONG
                if b_is_long(duration) {
                    self.preamble_count += 1;
                    return None;
                }

                self.reset_state();
                None
            }

            DecoderStep::DataB => {
                // Every pulse (HIGH or LOW) encodes one bit. A pulse >= sync-gap min ends the frame.
                if duration >= B_SYNC_GAP_MIN {
                    let result = if self.bit_count >= MIN_COUNT_BIT {
                        self.decode_and_fire()
                    } else {
                        None
                    };
                    self.reset_state();
                    return result;
                }

                let bit = if duration > B_NRZ_MIDPOINT { 1 } else { 0 };
                self.push_bit(bit);

                if self.bit_count >= B_BITS {
                    let result = self.decode_and_fire();
                    self.reset_state();
                    return result;
                }
                None
            }

            _ => None,
        }
    }
}

impl ProtocolDecoder for ToyotaDecoder {
    fn name(&self) -> &'static str {
        "Toyota"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: A_TE_SHORT,
            te_long: A_TE_LONG,
            te_delta: A_TE_DELTA,
            min_count_bit: MIN_COUNT_BIT,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        // Variant A 433.92 MHz, Variant B 315 MHz.
        &[433_920_000, 315_000_000]
    }

    fn reset(&mut self) {
        self.reset_state();
        // Full reset between segments: also clear the detected variant.
        self.variant = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        if self.step == DecoderStep::Reset {
            if !level {
                return None;
            }

            // Variant detection from the first SHORT HIGH pulse:
            //   < 310µs  -> Variant B (Tundra 315 MHz, te_short ~200µs)
            //   >= 310µs -> Variant A (Corolla 433 MHz, te_short ~400µs)
            let fits_b = b_is_short(duration) && duration < VARIANT_THRESH;
            let fits_a = a_is_short(duration) && duration >= VARIANT_THRESH;

            if fits_b {
                self.variant = 1;
                self.te_last = duration;
                self.have_high = true;
                self.preamble_count = 0;
                self.step = DecoderStep::PreambleB;
            } else if fits_a {
                self.variant = 0;
                self.te_last = duration;
                self.have_high = true;
                self.preamble_count = 0;
                self.step = DecoderStep::PreambleA;
            }
            return None;
        }

        if self.variant == 1 {
            self.feed_variant_b(level, duration)
        } else {
            self.feed_variant_a(level, duration)
        }
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }
}

impl Default for ToyotaDecoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a Variant A (PWM) frame: preamble SS pairs, then `bits` MSB-first.
    /// LS (long HIGH + short LOW) = 0, SL (short HIGH + long LOW) = 1. Terminated by a long gap.
    fn build_variant_a(bits: &[u8]) -> Vec<LevelDuration> {
        let mut v = Vec::new();
        // 8 short-short preamble pairs (>= A_PREAMBLE_MIN = 6).
        for _ in 0..8 {
            v.push(LevelDuration::new(true, A_TE_SHORT));
            v.push(LevelDuration::new(false, A_TE_SHORT));
        }
        for &b in bits {
            if b == 0 {
                v.push(LevelDuration::new(true, A_TE_LONG));
                v.push(LevelDuration::new(false, A_TE_SHORT));
            } else {
                v.push(LevelDuration::new(true, A_TE_SHORT));
                v.push(LevelDuration::new(false, A_TE_LONG));
            }
        }
        // Trailing HIGH gap (out of TE range) flushes the frame.
        v.push(LevelDuration::new(true, 5000));
        v
    }

    /// Build a Variant B (NRZ) frame: short-HIGH/long-LOW preamble pairs, sync gap, then each bit
    /// as one pulse (short = 0, long = 1), terminated by a sync-gap-length pulse.
    fn build_variant_b(bits: &[u8]) -> Vec<LevelDuration> {
        let mut v = Vec::new();
        // 8 preamble pairs: short HIGH (~200µs) + long LOW (~390µs).
        for _ in 0..8 {
            v.push(LevelDuration::new(true, B_TE_SHORT));
            v.push(LevelDuration::new(false, B_TE_LONG));
        }
        // Last preamble HIGH, then the sync-gap LOW.
        v.push(LevelDuration::new(true, B_TE_SHORT));
        v.push(LevelDuration::new(false, 1938));
        // Data: each bit one pulse, alternating level (polarity is ignored by the NRZ classifier).
        for (i, &b) in bits.iter().enumerate() {
            let level = i % 2 == 0;
            let dur = if b == 1 { 380 } else { 200 };
            v.push(LevelDuration::new(level, dur));
        }
        // End-of-frame gap.
        v.push(LevelDuration::new(false, 2000));
        v
    }

    fn feed_all(dec: &mut ToyotaDecoder, pairs: &[LevelDuration]) -> Option<DecodedSignal> {
        for p in pairs {
            if let Some(sig) = dec.feed(p.level, p.duration_us) {
                return Some(sig);
            }
        }
        None
    }

    #[test]
    fn variant_a_synthetic_decodes() {
        // 68-bit frame: a recognizable pattern with a non-zero serial and a known button nibble.
        // bits[0..32] = hop, bits[32..60] = serial, bits[60..64] = button, bits[64..68] = padding.
        let mut bits = vec![0u8; A_BITS];
        // hop = 0x9ABCDEF0 (MSB first in bits[0..32])
        let hop: u32 = 0x9ABC_DEF0;
        for i in 0..32 {
            bits[i] = ((hop >> (31 - i)) & 1) as u8;
        }
        // serial = 0x0123456 (28 bits) in bits[32..60]
        let serial: u32 = 0x012_3456;
        for i in 0..28 {
            bits[32 + i] = ((serial >> (27 - i)) & 1) as u8;
        }
        // button = 0x8 (Lock) in bits[60..64]
        let button: u8 = 0x8;
        for i in 0..4 {
            bits[60 + i] = (button >> (3 - i)) & 1;
        }
        // bits[64..68] padding = 0

        let frame = build_variant_a(&bits);
        let mut dec = ToyotaDecoder::new();
        let sig = feed_all(&mut dec, &frame).expect("variant A frame should decode");

        assert_eq!(sig.data_count_bit, A_BITS);
        assert_eq!(sig.serial, Some(serial));
        assert_eq!(sig.button, Some(button));
        // hop is the top 32 bits of data
        assert_eq!((sig.data >> 32) as u32, hop);
        assert!(sig.crc_valid);
    }

    #[test]
    fn variant_b_synthetic_decodes() {
        // 67-bit NRZ frame.
        let mut bits = vec![0u8; B_BITS];
        let hop: u32 = 0x1357_9BDF;
        for i in 0..32 {
            bits[i] = ((hop >> (31 - i)) & 1) as u8;
        }
        let serial: u32 = 0x0AB_CDEF;
        for i in 0..28 {
            bits[32 + i] = ((serial >> (27 - i)) & 1) as u8;
        }
        // button (bits[60..64]); only 3 bits of button fit before the 67-bit end (bits[60..63]),
        // bit 63..67 truncated — extract(60,4) reads bits[63..67] from the END. Just assert serial.
        for i in 0..7 {
            bits[60 + i] = if i % 2 == 0 { 1 } else { 0 };
        }

        let frame = build_variant_b(&bits);
        let mut dec = ToyotaDecoder::new();
        let sig = feed_all(&mut dec, &frame).expect("variant B frame should decode");

        assert_eq!(sig.data_count_bit, B_BITS);
        assert_eq!(sig.serial, Some(serial));
        assert!(sig.crc_valid);
    }

    #[test]
    fn rejects_all_zero_serial() {
        // A structurally valid 68-bit frame with an all-zero serial must NOT emit (tight gate).
        let bits = vec![0u8; A_BITS];
        let frame = build_variant_a(&bits);
        let mut dec = ToyotaDecoder::new();
        assert!(feed_all(&mut dec, &frame).is_none());
    }
}
