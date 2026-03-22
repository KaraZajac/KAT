//! Fiat V0 protocol decoder/encoder
//!
//! Aligned with older ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/fiat_v0.c` and
//! `fiat_v0.h`. Preamble: count short pulses (HIGH or LOW, 200±100µs); when preamble_count >= 150
//! (0x96), accept 800µs LOW gap (gap_threshold 800, te_delta 100) and enter Data. Data: 64 bits
//! (serial=data_low, cnt=data_high) then 7 more bits; complete when bit_count > 0x46 with
//! btn = (uint8_t)data_low, 71 bits total. Encoder: standard Manchester, 150 preamble pairs,
//! last LOW replaced by 800µs gap; 64 data bits then 7 endbyte bits (endbyte & 0x7F); end
//! marker te_short*4 LOW.

use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

const TE_SHORT: u32 = 200;
const TE_LONG: u32 = 400;
const TE_DELTA: u32 = 100;
#[allow(dead_code)]
const MIN_COUNT_BIT: usize = 64;
const PREAMBLE_PAIRS: u16 = 150; // 0x96 in reference
const GAP_US: u32 = 800;
const GAP_THRESHOLD: u32 = 800;
const TOTAL_BURSTS: u8 = 3;
const INTER_BURST_GAP: u32 = 25000;

/// Fiat V0 Manchester state machine. Matches Flipper manchester_decoder.h (ref uses it).
#[derive(Debug, Clone, Copy, PartialEq)]
enum FiatV0ManchesterState {
    Mid0 = 0,
    Mid1 = 1,
    Start0 = 2,
    Start1 = 3,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Preamble,
    Data,
}

pub struct FiatV0Decoder {
    step: DecoderStep,
    preamble_count: u16,
    manchester_state: FiatV0ManchesterState,
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
            manchester_state: FiatV0ManchesterState::Mid1,
            data_low: 0,
            data_high: 0,
            bit_count: 0,
            cnt: 0,
            serial: 0,
            btn: 0,
            te_last: 0,
        }
    }

    fn manchester_advance(&mut self, event: u8) -> Option<bool> {
        let (new_state, emit) = match (self.manchester_state, event) {
            (FiatV0ManchesterState::Mid0, 0) => (FiatV0ManchesterState::Mid0, false),
            (FiatV0ManchesterState::Mid0, 1) => (FiatV0ManchesterState::Start1, true),
            (FiatV0ManchesterState::Mid0, 2) => (FiatV0ManchesterState::Mid0, false),
            (FiatV0ManchesterState::Mid0, 3) => (FiatV0ManchesterState::Mid1, true),

            (FiatV0ManchesterState::Mid1, 0) => (FiatV0ManchesterState::Start0, true),
            (FiatV0ManchesterState::Mid1, 1) => (FiatV0ManchesterState::Mid1, false),
            (FiatV0ManchesterState::Mid1, 2) => (FiatV0ManchesterState::Mid0, true),
            (FiatV0ManchesterState::Mid1, 3) => (FiatV0ManchesterState::Mid1, false),

            (FiatV0ManchesterState::Start0, 0) => (FiatV0ManchesterState::Mid0, false),
            (FiatV0ManchesterState::Start0, 1) => (FiatV0ManchesterState::Mid0, false),
            (FiatV0ManchesterState::Start0, 2) => (FiatV0ManchesterState::Mid0, false),
            (FiatV0ManchesterState::Start0, 3) => (FiatV0ManchesterState::Mid1, false),

            (FiatV0ManchesterState::Start1, 0) => (FiatV0ManchesterState::Mid0, false),
            (FiatV0ManchesterState::Start1, 1) => (FiatV0ManchesterState::Mid1, false),
            (FiatV0ManchesterState::Start1, 2) => (FiatV0ManchesterState::Mid0, false),
            (FiatV0ManchesterState::Start1, 3) => (FiatV0ManchesterState::Mid1, false),

            _ => (FiatV0ManchesterState::Mid1, false),
        };

        self.manchester_state = new_state;
        if emit {
            Some((event & 1) == 1)
        } else {
            None
        }
    }

    fn manchester_reset(&mut self) {
        self.manchester_state = FiatV0ManchesterState::Mid1;
    }

    fn parse_data(&self) -> DecodedSignal {
        let data = ((self.cnt as u64) << 32) | (self.serial as u64);

        DecodedSignal {
            serial: Some(self.serial),
            button: Some(self.btn),
            counter: Some(self.cnt as u16),
            crc_valid: true,
            data,
            data_count_bit: 71,
            encoder_capable: true,
            extra: None,
            protocol_display_name: None,
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
                // Ref: only look at LOW for gap; both HIGH and LOW can count as short
                if level {
                    // HIGH pulse
                    if diff_short < TE_DELTA {
                        self.preamble_count += 1;
                        self.te_last = duration;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                    return None;
                }

                // LOW pulse
                if duration < TE_SHORT {
                    let diff = TE_SHORT - duration;
                    if diff < TE_DELTA {
                        self.preamble_count += 1;
                        self.te_last = duration;
                        if self.preamble_count >= PREAMBLE_PAIRS {
                            let gap_diff = if duration < GAP_THRESHOLD {
                                GAP_THRESHOLD - duration
                            } else {
                                duration - GAP_THRESHOLD
                            };
                            if gap_diff < TE_DELTA {
                                self.step = DecoderStep::Data;
                                self.preamble_count = 0;
                                self.data_low = 0;
                                self.data_high = 0;
                                self.bit_count = 0;
                                self.te_last = duration;
                                self.manchester_reset();
                                return None;
                            }
                        }
                    } else {
                        self.step = DecoderStep::Reset;
                        if self.preamble_count >= PREAMBLE_PAIRS {
                            let gap_diff = if duration < GAP_THRESHOLD {
                                GAP_THRESHOLD - duration
                            } else {
                                duration - GAP_THRESHOLD
                            };
                            if gap_diff < TE_DELTA {
                                self.step = DecoderStep::Data;
                                self.preamble_count = 0;
                                self.data_low = 0;
                                self.data_high = 0;
                                self.bit_count = 0;
                                self.te_last = duration;
                                self.manchester_reset();
                                return None;
                            }
                        }
                    }
                } else {
                    let diff = duration - TE_SHORT;
                    if diff < TE_DELTA {
                        self.preamble_count += 1;
                        self.te_last = duration;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                    if self.preamble_count >= PREAMBLE_PAIRS {
                        let gap_diff = if duration >= 799 {
                            duration - GAP_THRESHOLD
                        } else {
                            GAP_THRESHOLD - duration
                        };
                        if gap_diff < TE_DELTA {
                            self.step = DecoderStep::Data;
                            self.preamble_count = 0;
                            self.data_low = 0;
                            self.data_high = 0;
                            self.bit_count = 0;
                            self.te_last = duration;
                            self.manchester_reset();
                            return None;
                        }
                    }
                }
            }

            DecoderStep::Data => {
                let mut event = 4u8; // ManchesterEventReset
                if duration < TE_SHORT {
                    let diff = TE_SHORT - duration;
                    if diff < TE_DELTA {
                        event = if level { 0 } else { 1 }; // ShortLow : ShortHigh
                    }
                } else {
                    let diff = duration - TE_SHORT;
                    if diff < TE_DELTA {
                        event = if level { 0 } else { 1 };
                    } else {
                        let long_diff = duration_diff!(duration, TE_LONG);
                        if long_diff < TE_DELTA {
                            event = if level { 2 } else { 3 }; // LongLow : LongHigh
                        }
                    }
                }

                if event != 4 {
                    if let Some(data_bit_bool) = self.manchester_advance(event) {
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

                        if self.bit_count == 0x47 {
                            // C: endbyte = (uint8_t)data_low (no transform)
                            self.btn = self.data_low as u8;
                            let result = self.parse_data();
                            self.data_low = 0;
                            self.data_high = 0;
                            self.bit_count = 0;
                            self.step = DecoderStep::Reset;
                            return Some(result);
                        }
                    }
                } else {
                    // Manchester reset event (gap path) — C extracts at exactly 71 bits
                    if self.bit_count == 0x47 {
                        self.btn = self.data_low as u8;
                        let result = self.parse_data();
                        self.data_low = 0;
                        self.data_high = 0;
                        self.bit_count = 0;
                        self.step = DecoderStep::Reset;
                        return Some(result);
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

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        let serial = decoded.serial?;
        let cnt = decoded.counter.unwrap_or(0) as u32;
        let endbyte = if button != 0 { button } else { decoded.button.unwrap_or(0) };

        // C: data = (hop << 32) | fix; endbyte sent as-is (7 bits, & 0x7F)
        let data = ((cnt as u64) << 32) | (serial as u64);
        let te_short = TE_SHORT;

        let mut signal = Vec::with_capacity(1024);

        for burst in 0..TOTAL_BURSTS {
            if burst > 0 {
                signal.push(LevelDuration::new(false, INTER_BURST_GAP));
            }

            // Preamble: alternating short pulses; last LOW extended to gap
            for _ in 0..PREAMBLE_PAIRS {
                signal.push(LevelDuration::new(true, te_short));
                signal.push(LevelDuration::new(false, te_short));
            }
            // Extend last LOW to create gap (matches C: upload[index-1] = gap)
            if let Some(last) = signal.last_mut() {
                *last = LevelDuration::new(false, GAP_US);
            }

            // Standard Manchester encode 64 bits of data (matches C)
            for bit in (0..64).rev() {
                let curr_bit = (data >> bit) & 1 == 1;
                if curr_bit {
                    signal.push(LevelDuration::new(true, te_short));
                    signal.push(LevelDuration::new(false, te_short));
                } else {
                    signal.push(LevelDuration::new(false, te_short));
                    signal.push(LevelDuration::new(true, te_short));
                }
            }

            // Standard Manchester encode 7 bits of endbyte (matches C: endbyte & 0x7F, bits 6..0)
            let endbyte_masked = endbyte & 0x7F;
            for bit in (0..7).rev() {
                let curr_bit = (endbyte_masked >> bit) & 1 == 1;
                if curr_bit {
                    signal.push(LevelDuration::new(true, te_short));
                    signal.push(LevelDuration::new(false, te_short));
                } else {
                    signal.push(LevelDuration::new(false, te_short));
                    signal.push(LevelDuration::new(true, te_short));
                }
            }

            // End marker: te_short * 4 LOW (matches C)
            signal.push(LevelDuration::new(false, te_short * 4));
        }

        Some(signal)
    }
}

impl Default for FiatV0Decoder {
    fn default() -> Self {
        Self::new()
    }
}
