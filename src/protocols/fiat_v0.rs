//! Fiat V0 protocol decoder/encoder
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/fiat_v0.c` and `fiat_v0.h`
//! (Flipper-ARF and doctor-firmware carry the same decoder and upload).
//!
//! Preamble: count short pulses (HIGH or LOW, 200±100µs); when preamble_count >= 150 (0x96), an
//! 800µs LOW gap (±100) enters Data. Data: 64 bits (fix/serial = data_low, hop/counter =
//! data_high), then 7 more bits; the frame completes at bit 71 (0x47).
//!
//! **Manchester**: the shared Flipper `manchester_advance` port in `common.rs`, event mapping
//! `level ? ShortLow : ShortHigh` (same for long; fiat_v0.c:426-439), state reset to Mid1
//! (`ManchesterEventReset`) when the preamble starts and at the gap (fiat_v0.c:343, 387, 411).
//! The first HIGH after the gap belongs to an implied leading 1 bit; Mid1 drops it and locks.
//!
//! **Frame on air**: 64 data bits + 6 tail bits, Manchester with 1 = HIGH,LOW. The decoder's
//! 71st bit is not part of the burst: after bit 70 the inter-burst gap is a Manchester reset
//! event that the C ignores once 64 bits are in, and the 71st bit comes from the NEXT burst's
//! preamble (always 1). So the last burst of a press never completes (as in the C). On every
//! real capture the 6 tail bits are `0xxxx1`.
//!
//! **DecodedSignal**: `data` = hop << 32 | fix, `data_count_bit` = 71, `serial` = fix,
//! `counter` = hop[15:0] (hop is the 32-bit rolling/encrypted word), `button` = the 7 decoded
//! tail bits (the C keeps `& 0x3F`; identical on every real capture since the first tail bit is
//! 0 — KAT keeps all 7 so the encoder can resend the 6 on-air tail bits losslessly).
//!
//! **Gating**: there is no checksum; emission is gated on structure (≥150-pulse preamble,
//! 800µs gap, 71 Manchester bits), like the C.
//!
//! **Encoder** (subghz_protocol_encoder_fiat_v0_get_upload): byte-exact replay of the captured
//! frame — hop is a rolling code and the C replays the captured EndByte regardless of the
//! selected button. 3 bursts (25ms apart) of 150 short pairs whose last LOW is the 800µs gap,
//! then the 64 data bits and the 6 tail bits (`endbyte >> 1`) transition-coded as in the C, a
//! final LOW short if the last bit is 1, and a 1600µs LOW end marker.
//! **Deviation from C**: the C sends a first data bit of 1 as one long HIGH, which its own
//! decoder cannot lock onto (the frame decodes shifted by one bit); real fobs send `H s, L s, H s`
//! (IMPORTS/FIAT/Fiat Bravo...: frames with hop bit 31 = 1 start `-754 +201 -218 +145`), i.e. a
//! short HIGH followed by the normal transition from an implied 1. KAT does that; for a first bit
//! of 0 the output is identical to the C.

use super::common::{ManchesterEvent, ManchesterState};
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

const TE_SHORT: u32 = 200;
const TE_LONG: u32 = 400;
const TE_DELTA: u32 = 100;
#[allow(dead_code)]
const MIN_COUNT_BIT: usize = 64;
const FRAME_BITS: u8 = 0x47; // 71
const PREAMBLE_PAIRS: u16 = 150; // 0x96 in reference
const GAP_US: u32 = 800;
const GAP_THRESHOLD: u32 = 800;
const TOTAL_BURSTS: u8 = 3;
const INTER_BURST_GAP: u32 = 25000;
/// Tail bits sent after the 64 data bits (`endbyte >> 1`, bits 5..0).
const TAIL_BITS: u32 = 6;

#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Preamble,
    Data,
}

pub struct FiatV0Decoder {
    step: DecoderStep,
    preamble_count: u16,
    manchester_state: ManchesterState,
    data_low: u32,
    data_high: u32,
    bit_count: u8,
    cnt: u32,
    serial: u32,
    btn: u8,
    te_last: u32,
}

impl FiatV0Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            preamble_count: 0,
            manchester_state: ManchesterState::Mid1,
            data_low: 0,
            data_high: 0,
            bit_count: 0,
            cnt: 0,
            serial: 0,
            btn: 0,
            te_last: 0,
        }
    }

    fn manchester_reset(&mut self) {
        self.manchester_state.advance(ManchesterEvent::Reset);
    }

    fn parse_data(&self) -> DecodedSignal {
        let data = ((self.cnt as u64) << 32) | (self.serial as u64);

        DecodedSignal {
            serial: Some(self.serial),
            button: Some(self.btn),
            counter: Some(self.cnt as u16),
            crc_valid: true,
            data,
            data_count_bit: FRAME_BITS as usize,
            encoder_capable: true,
            extra: None,
            protocol_display_name: None,
        }
    }

    /// fiat_v0_finish_packet: capture the tail bits, build the signal and return to Reset.
    fn finish_packet(&mut self) -> DecodedSignal {
        self.btn = (self.data_low & 0x7F) as u8;
        let result = self.parse_data();
        self.data_low = 0;
        self.data_high = 0;
        self.bit_count = 0;
        self.step = DecoderStep::Reset;
        result
    }

    /// pp_emit with ADD_LEVEL-style merging of equal adjacent levels.
    fn add_level(signal: &mut Vec<LevelDuration>, level: bool, duration: u32) {
        if let Some(last) = signal.last_mut() {
            if last.level == level {
                *last = LevelDuration::new(level, last.duration_us + duration);
                return;
            }
        }
        signal.push(LevelDuration::new(level, duration));
    }

    /// The C's (prev, curr) cases: the second half of `prev` then the first half of `curr`
    /// (1 = HIGH,LOW; 0 = LOW,HIGH on air).
    fn add_transition(signal: &mut Vec<LevelDuration>, prev: bool, curr: bool) {
        match (prev, curr) {
            (false, false) => {
                Self::add_level(signal, true, TE_SHORT);
                Self::add_level(signal, false, TE_SHORT);
            }
            (false, true) => Self::add_level(signal, true, TE_LONG),
            (true, false) => Self::add_level(signal, false, TE_LONG),
            (true, true) => {
                Self::add_level(signal, false, TE_SHORT);
                Self::add_level(signal, true, TE_SHORT);
            }
        }
    }
}

impl ProtocolDecoder for FiatV0Decoder {
    fn name(&self) -> &'static str {
        "Fiat V0"
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
        &[433_920_000, 433_880_000]
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.preamble_count = 0;
        self.data_low = 0;
        self.data_high = 0;
        self.bit_count = 0;
        self.cnt = 0;
        self.serial = 0;
        self.btn = 0;
        self.te_last = 0;
        self.manchester_reset();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let diff_short = if duration < TE_SHORT {
            TE_SHORT - duration
        } else {
            duration - TE_SHORT
        };

        match self.step {
            DecoderStep::Reset => {
                if !level {
                    return None;
                }
                if diff_short < TE_DELTA {
                    self.data_low = 0;
                    self.data_high = 0;
                    self.step = DecoderStep::Preamble;
                    self.te_last = duration;
                    self.preamble_count = 0;
                    self.bit_count = 0;
                    self.manchester_reset();
                }
            }

            DecoderStep::Preamble => {
                // fiat_v0.c (FiatV0DecoderStepPreamble): a HIGH must be short; a short LOW counts;
                // any other LOW after >= 150 shorts that is the 800µs gap enters Data, otherwise
                // Reset. (The C's second gap check — still in Preamble after a short LOW — can
                // never match: a short LOW is never within te_delta of 800µs.)
                if level {
                    if diff_short < TE_DELTA {
                        self.preamble_count = self.preamble_count.wrapping_add(1);
                        self.te_last = duration;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                    return None;
                }
                if diff_short < TE_DELTA {
                    self.preamble_count = self.preamble_count.wrapping_add(1);
                    self.te_last = duration;
                } else {
                    if self.preamble_count >= PREAMBLE_PAIRS
                        && duration_diff!(duration, GAP_THRESHOLD) < TE_DELTA
                    {
                        self.step = DecoderStep::Data;
                        self.preamble_count = 0;
                        self.data_low = 0;
                        self.data_high = 0;
                        self.bit_count = 0;
                        self.te_last = duration;
                        self.manchester_reset();
                        return None;
                    }
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::Data => {
                // fiat_v0.c:422-442: level ? ShortLow : ShortHigh (same for long); else Reset event.
                let event = if diff_short < TE_DELTA {
                    Some(ManchesterEvent::from_half_bit(!level, false))
                } else if duration_diff!(duration, TE_LONG) < TE_DELTA {
                    Some(ManchesterEvent::from_half_bit(!level, true))
                } else {
                    None
                };

                if let Some(event) = event {
                    if let Some(data_bit_bool) = self.manchester_state.advance(event) {
                        let new_bit = if data_bit_bool { 1u32 } else { 0u32 };
                        let carry = (self.data_low >> 31) & 1;
                        self.data_low = (self.data_low << 1) | new_bit;
                        self.data_high = (self.data_high << 1) | carry;
                        self.bit_count += 1;

                        if self.bit_count == 0x40 {
                            self.serial = self.data_low;
                            self.cnt = self.data_high;
                            self.data_low = 0;
                            self.data_high = 0;
                        }

                        if self.bit_count == FRAME_BITS {
                            return Some(self.finish_packet());
                        }
                    }
                } else {
                    // Manchester reset event (gap path): complete at exactly 71 bits; below 64 bits
                    // give up; 64..70 bits keep waiting (the 71st bit comes from the next burst).
                    if self.bit_count == FRAME_BITS {
                        return Some(self.finish_packet());
                    } else if self.bit_count < 0x40 {
                        self.step = DecoderStep::Reset;
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

    /// Byte-exact replay of the captured frame (the requested button is ignored, as in the C).
    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        let serial = decoded.serial?;
        let hop = (decoded.data >> 32) as u32;
        // C: data = (hop << 32) | fix; endbyte_to_send = endbyte >> 1 (6 tail bits).
        let data = ((hop as u64) << 32) | (serial as u64);
        let tail = (decoded.button.unwrap_or(0) >> 1) & 0x3F;
        let bits = (0..64)
            .rev()
            .map(|bit| (data >> bit) & 1 == 1)
            .chain((0..TAIL_BITS).rev().map(|bit| (tail >> bit) & 1 == 1));

        let mut signal = Vec::with_capacity(1024);
        for burst in 0..TOTAL_BURSTS {
            if burst > 0 {
                Self::add_level(&mut signal, false, INTER_BURST_GAP);
            }

            // Preamble: alternating short pulses; the last LOW becomes the 800µs gap.
            for _ in 0..PREAMBLE_PAIRS - 1 {
                Self::add_level(&mut signal, true, TE_SHORT);
                Self::add_level(&mut signal, false, TE_SHORT);
            }
            Self::add_level(&mut signal, true, TE_SHORT);
            Self::add_level(&mut signal, false, GAP_US);

            // Short HIGH, then every bit as a transition from an implied leading 1 (see the module
            // doc: for a first bit of 0 this is exactly the C's "H short, L long").
            Self::add_level(&mut signal, true, TE_SHORT);
            let mut prev_bit = true;
            for curr_bit in bits.clone() {
                Self::add_transition(&mut signal, prev_bit, curr_bit);
                prev_bit = curr_bit;
            }

            if prev_bit {
                Self::add_level(&mut signal, false, TE_SHORT);
            }
            // End marker: te_short * 8 LOW (matches C)
            Self::add_level(&mut signal, false, TE_SHORT * 8);
        }

        Some(signal)
    }
}

impl Default for FiatV0Decoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::export::flipper::import_sub_raw;
    use crate::protocols::ProtocolRegistry;
    use std::path::Path;

    const FIX: u32 = 0x5453_6200;

    fn frame(hop: u32, fix: u32, tail7: u8) -> DecodedSignal {
        DecodedSignal {
            serial: Some(fix),
            button: Some(tail7),
            counter: Some(hop as u16),
            crc_valid: true,
            data: ((hop as u64) << 32) | fix as u64,
            data_count_bit: FRAME_BITS as usize,
            encoder_capable: true,
            extra: None,
            protocol_display_name: None,
        }
    }

    fn feed_all(signal: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = FiatV0Decoder::new();
        signal.iter().filter_map(|p| dec.feed(p.level, p.duration_us)).collect()
    }

    /// Encode → decode for real frames (IMPORTS/FIAT/Fiat Bravo...), one with hop bit 31 = 0 and one
    /// with hop bit 31 = 1. Bursts 1 and 2 decode; burst 3 has no following preamble to supply the
    /// 71st bit (C behaviour).
    #[test]
    fn encode_decode_roundtrip() {
        for (hop, tail7) in [(0x0DE2_1BFEu32, 0x13u8), (0xFEC0_E84D, 0x1B), (0xA03B_F868, 0x3B)] {
            let upload = FiatV0Decoder::new().encode(&frame(hop, FIX, tail7), 0x02).unwrap();
            let out = feed_all(&upload);
            assert_eq!(out.len(), (TOTAL_BURSTS - 1) as usize, "hop {hop:08X}");
            for d in &out {
                assert_eq!(d.data, ((hop as u64) << 32) | FIX as u64, "hop {hop:08X}");
                assert_eq!(d.serial, Some(FIX));
                assert_eq!(d.button, Some(tail7), "tail bits replayed");
                assert_eq!(d.data_count_bit, 71);
            }
        }
    }

    /// The upload starts like the C's after the 800µs gap: `H s, L long` for a first bit of 0 and
    /// (corrected, as on real captures) `H s, L s, H s` for a first bit of 1.
    #[test]
    fn upload_frame_start_matches_reference() {
        let gap_at = |up: &[LevelDuration]| up.iter().position(|p| !p.level && p.duration_us == GAP_US).unwrap();
        let up0 = FiatV0Decoder::new().encode(&frame(0x0DE2_1BFE, FIX, 0x13), 0).unwrap();
        let i = gap_at(&up0);
        assert_eq!(i, 2 * PREAMBLE_PAIRS as usize - 1, "150 HIGH shorts, the last LOW is the gap");
        let start: Vec<(bool, u32)> = up0[i + 1..i + 3].iter().map(|p| (p.level, p.duration_us)).collect();
        assert_eq!(start, [(true, TE_SHORT), (false, TE_LONG)]);
        let up1 = FiatV0Decoder::new().encode(&frame(0xFEC0_E84D, FIX, 0x1B), 0).unwrap();
        let i = gap_at(&up1);
        let start: Vec<(bool, u32)> = up1[i + 1..i + 4].iter().map(|p| (p.level, p.duration_us)).collect();
        assert_eq!(start, [(true, TE_SHORT), (false, TE_SHORT), (true, TE_SHORT)]);
        // Ends with the 1600µs end marker (merged with a final LOW short after a final 1).
        assert_eq!(up1.last().map(|p| (p.level, p.duration_us)), Some((false, TE_SHORT * 9)));
    }

    /// Structure gate: a preamble shorter than 150 pulses or a wrong gap must not decode.
    #[test]
    fn gap_window_matches_reference() {
        // fiat_v0.c enters Data on a LOW within te_delta of 800µs (701..=899); 799µs used to
        // underflow here (debug panic / missed frame in release).
        let good = FiatV0Decoder::new().encode(&frame(0x0DE2_1BFE, FIX, 0x13), 0).unwrap();
        for (gap, decodes) in [(701u32, true), (799, true), (800, true), (899, true), (700, false), (900, false)] {
            let mut wave = good.clone();
            for p in wave.iter_mut().filter(|p| !p.level && p.duration_us == GAP_US) {
                *p = LevelDuration::new(false, gap);
            }
            assert_eq!(!feed_all(&wave).is_empty(), decodes, "gap {gap}µs");
        }
    }

    #[test]
    fn rejects_bad_structure() {
        let good = FiatV0Decoder::new().encode(&frame(0x0DE2_1BFE, FIX, 0x13), 0).unwrap();
        assert!(!feed_all(&good).is_empty());

        let mut bad_gap = good.clone();
        for p in bad_gap.iter_mut().filter(|p| !p.level && p.duration_us == GAP_US) {
            *p = LevelDuration::new(false, 1000);
        }
        assert!(feed_all(&bad_gap).is_empty(), "1000µs gap");

        // Keep only the last 70 preamble pairs of every burst: 138 counted pulses < 150.
        let mut short_pre = Vec::new();
        let mut skip = 160;
        for p in &good {
            if skip > 0 {
                skip -= 1;
                continue;
            }
            short_pre.push(*p);
            if !p.level && p.duration_us >= INTER_BURST_GAP {
                skip = 160;
            }
        }
        assert!(feed_all(&short_pre).is_empty(), "short preamble");
    }

    /// Real capture: IMPORTS/FIAT/Fiat Bravo 2010 S.Croce 433.88 am650.sub decodes as Fiat V0 with
    /// one fix (serial) and a rolling hop, and one frame equals the saved key file
    /// IMPORTS/FIAT/Fiat type 0 N1.sub (Key: A0 3B F8 68 54 53 62 00).
    #[test]
    fn decodes_real_capture() {
        let path = Path::new("IMPORTS/FIAT/Fiat Bravo 2010 S.Croce 433.88 am650.sub");
        if !path.exists() {
            eprintln!("Skip: {:?} not found (run from crate root)", path);
            return;
        }
        let (freq, raw) = import_sub_raw(path).unwrap();
        let pairs: Vec<LevelDuration> =
            raw.iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect();
        let results = ProtocolRegistry::new().process_signal_stream(&pairs, freq);
        let fiat: Vec<&DecodedSignal> =
            results.iter().filter(|(n, _, _)| n == "Fiat V0").map(|(_, d, _)| d).collect();
        assert!(fiat.len() >= 5, "got {:?}", results.iter().map(|(n, _, _)| n).collect::<Vec<_>>());
        assert!(fiat.iter().all(|d| d.serial == Some(FIX)), "one fob");
        assert!(fiat.iter().any(|d| d.data == 0xA03B_F868_5453_6200), "matches Fiat type 0 N1.sub");
        // Every real tail is 0xxxx1 (+ the 71st bit, 1, from the next preamble).
        assert!(fiat.iter().all(|d| d.button.map_or(false, |b| b & 0x43 == 0x03)));
    }
}
