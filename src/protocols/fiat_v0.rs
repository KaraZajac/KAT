//! Fiat V0 protocol decoder/encoder
//!
//! Ported from protopirate's fiat_v0.c
//!
//! Protocol characteristics:
//! - Differential Manchester encoding: 200/400µs timing
//! - 64-bit data (cnt:32 | serial:32) + 6-bit button
//! - 150 preamble pairs, 800µs gap, 3 bursts

use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

const TE_SHORT: u32 = 200;
const TE_LONG: u32 = 400;
const TE_DELTA: u32 = 100;
#[allow(dead_code)]
const MIN_COUNT_BIT: usize = 64;
const PREAMBLE_PAIRS: u16 = 150;
const GAP_US: u32 = 800;
const TOTAL_BURSTS: u8 = 3;
const INTER_BURST_GAP: u32 = 25000;

/// Manchester decoder states
#[derive(Debug, Clone, Copy, PartialEq)]
enum ManchesterState {
    Mid0,
    Mid1,
    Start0,
    Start1,
}

/// Decoder states
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Preamble,
    Data,
}

/// Fiat V0 protocol decoder
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

    /// Manchester advance - returns decoded bit or None
    fn manchester_advance(&mut self, is_short: bool, is_high: bool) -> Option<bool> {
        let event = match (is_short, is_high) {
            (true, true) => 0,
            (true, false) => 1,
            (false, true) => 2,
            (false, false) => 3,
        };

        let (new_state, output) = match (self.manchester_state, event) {
            (ManchesterState::Mid0, 0) | (ManchesterState::Mid1, 0) => {
                (ManchesterState::Start1, None)
            }
            (ManchesterState::Mid0, 1) | (ManchesterState::Mid1, 1) => {
                (ManchesterState::Start0, None)
            }
            (ManchesterState::Start1, 1) => (ManchesterState::Mid1, Some(true)),
            (ManchesterState::Start1, 3) => (ManchesterState::Start0, Some(true)),
            (ManchesterState::Start0, 0) => (ManchesterState::Mid0, Some(false)),
            (ManchesterState::Start0, 2) => (ManchesterState::Start1, Some(false)),
            _ => (ManchesterState::Mid1, None),
        };

        self.manchester_state = new_state;
        output
    }

    fn manchester_reset(&mut self) {
        self.manchester_state = ManchesterState::Mid1;
    }

    fn add_manchester_bit(&mut self, bit: bool) {
        let new_bit = if bit { 1u32 } else { 0u32 };
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
    }

    fn parse_data(&self) -> DecodedSignal {
        let data = ((self.cnt as u64) << 32) | (self.serial as u64);

        DecodedSignal {
            serial: Some(self.serial),
            button: Some(self.btn),
            counter: Some(self.cnt as u16),
            crc_valid: true, // No CRC in Fiat V0
            data,
            data_count_bit: 71,
            encoder_capable: true,
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
        &[433_920_000]
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
        match self.step {
            DecoderStep::Reset => {
                if !level {
                    return None;
                }
                if duration_diff!(duration, TE_SHORT) < TE_DELTA {
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
                // Count short pulses in preamble, look for gap
                if duration_diff!(duration, TE_SHORT) < TE_DELTA {
                    self.preamble_count += 1;
                    self.te_last = duration;
                } else if self.preamble_count >= PREAMBLE_PAIRS {
                    // Check for gap
                    if duration_diff!(duration, GAP_US) < TE_DELTA {
                        self.step = DecoderStep::Data;
                        self.preamble_count = 0;
                        self.data_low = 0;
                        self.data_high = 0;
                        self.bit_count = 0;
                        self.te_last = duration;
                        return None;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::Data => {
                let is_short = duration_diff!(duration, TE_SHORT) < TE_DELTA;
                let is_long = duration_diff!(duration, TE_LONG) < TE_DELTA;

                if is_short || is_long {
                    if let Some(bit) = self.manchester_advance(is_short, level) {
                        self.add_manchester_bit(bit);

                        if self.bit_count > 0x46 {
                            self.btn = ((self.data_low << 1) | 1) as u8;
                            let result = self.parse_data();
                            self.data_low = 0;
                            self.data_high = 0;
                            self.bit_count = 0;
                            self.step = DecoderStep::Reset;
                            return Some(result);
                        }
                    }
                } else if duration > TE_LONG * 3 {
                    // End of signal
                    self.step = DecoderStep::Reset;
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

        let data = ((cnt as u64) << 32) | (serial as u64);
        // Reverse the decoder's btn fix: decoder does (x << 1) | 1
        let btn_to_send = button >> 1;

        let mut signal = Vec::with_capacity(1024);

        for burst in 0..TOTAL_BURSTS {
            if burst > 0 {
                signal.push(LevelDuration::new(false, INTER_BURST_GAP));
            }

            // Preamble
            for i in 0..PREAMBLE_PAIRS {
                signal.push(LevelDuration::new(true, TE_SHORT));
                if i < PREAMBLE_PAIRS - 1 {
                    signal.push(LevelDuration::new(false, TE_SHORT));
                } else {
                    signal.push(LevelDuration::new(false, GAP_US));
                }
            }

            // First bit (bit 63)
            let first_bit = (data >> 63) & 1 == 1;
            if first_bit {
                signal.push(LevelDuration::new(true, TE_LONG));
            } else {
                signal.push(LevelDuration::new(true, TE_SHORT));
                signal.push(LevelDuration::new(false, TE_LONG));
            }

            let mut prev_bit = first_bit;

            // Remaining 63 data bits using differential Manchester
            for bit in (0..63).rev() {
                let curr_bit = (data >> bit) & 1 == 1;
                match (prev_bit, curr_bit) {
                    (false, false) => {
                        signal.push(LevelDuration::new(true, TE_SHORT));
                        signal.push(LevelDuration::new(false, TE_SHORT));
                    }
                    (false, true) => {
                        signal.push(LevelDuration::new(true, TE_LONG));
                    }
                    (true, false) => {
                        signal.push(LevelDuration::new(false, TE_LONG));
                    }
                    (true, true) => {
                        signal.push(LevelDuration::new(false, TE_SHORT));
                        signal.push(LevelDuration::new(true, TE_SHORT));
                    }
                }
                prev_bit = curr_bit;
            }

            // 6 button bits
            for bit in (0..6).rev() {
                let curr_bit = (btn_to_send >> bit) & 1 == 1;
                match (prev_bit, curr_bit) {
                    (false, false) => {
                        signal.push(LevelDuration::new(true, TE_SHORT));
                        signal.push(LevelDuration::new(false, TE_SHORT));
                    }
                    (false, true) => {
                        signal.push(LevelDuration::new(true, TE_LONG));
                    }
                    (true, false) => {
                        signal.push(LevelDuration::new(false, TE_LONG));
                    }
                    (true, true) => {
                        signal.push(LevelDuration::new(false, TE_SHORT));
                        signal.push(LevelDuration::new(true, TE_SHORT));
                    }
                }
                prev_bit = curr_bit;
            }

            // End marker
            if prev_bit {
                signal.push(LevelDuration::new(false, TE_SHORT));
            }
            signal.push(LevelDuration::new(false, TE_SHORT * 8));
        }

        Some(signal)
    }
}
