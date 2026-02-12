//! Fiat V0 protocol decoder/encoder
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/fiat_v0.c` (Flipper).
//! Decode/encode logic (preamble, gap, Manchester, data/btn extraction, upload waveform) matches reference.
//!
//! Protocol characteristics:
//! - Differential Manchester encoding: 200/400µs timing
//! - 64-bit data (cnt:32 | serial:32) + 6-bit button
//! - 150 preamble pairs (count LOW pulses), 800µs gap, 3 bursts

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
const TOTAL_BURSTS: u8 = 3;
const INTER_BURST_GAP: u32 = 25000;

/// Manchester state machine states (matches Flipper's manchester_decoder.h, same as Ford V0)
#[derive(Debug, Clone, Copy, PartialEq)]
enum ManchesterState {
    Mid0 = 0,
    Mid1 = 1,
    Start0 = 2,
    Start1 = 3,
}

/// Decoder states (matches protopirate's FiatV0DecoderStep)
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

    /// Manchester state machine (same as Ford V0 / Flipper manchester_decoder).
    /// Event: 0=ShortLow, 1=ShortHigh, 2=LongLow, 3=LongHigh.
    fn manchester_advance(&mut self, event: u8) -> Option<bool> {
        let (new_state, emit) = match (self.manchester_state, event) {
            (ManchesterState::Mid0, 0) => (ManchesterState::Mid0, false),
            (ManchesterState::Mid0, 1) => (ManchesterState::Start1, true),
            (ManchesterState::Mid0, 2) => (ManchesterState::Mid0, false),
            (ManchesterState::Mid0, 3) => (ManchesterState::Mid1, true),

            (ManchesterState::Mid1, 0) => (ManchesterState::Start0, true),
            (ManchesterState::Mid1, 1) => (ManchesterState::Mid1, false),
            (ManchesterState::Mid1, 2) => (ManchesterState::Mid0, true),
            (ManchesterState::Mid1, 3) => (ManchesterState::Mid1, false),

            (ManchesterState::Start0, 0) => (ManchesterState::Mid0, false),
            (ManchesterState::Start0, 1) => (ManchesterState::Mid0, false),
            (ManchesterState::Start0, 2) => (ManchesterState::Mid0, false),
            (ManchesterState::Start0, 3) => (ManchesterState::Mid1, false),

            (ManchesterState::Start1, 0) => (ManchesterState::Mid0, false),
            (ManchesterState::Start1, 1) => (ManchesterState::Mid1, false),
            (ManchesterState::Start1, 2) => (ManchesterState::Mid0, false),
            (ManchesterState::Start1, 3) => (ManchesterState::Mid1, false),

            _ => (ManchesterState::Mid1, false),
        };

        self.manchester_state = new_state;
        if emit {
            Some((event & 1) == 1)
        } else {
            None
        }
    }

    fn manchester_reset(&mut self) {
        self.manchester_state = ManchesterState::Mid1;
    }

    /// Add bit to accumulator; at 64 bits extract serial/cnt and clear data (bit_count unchanged in reference).
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
            // Reset: wait for short HIGH (matches reference)
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

            // Preamble: only process LOW pulses (reference: if(level) return). Count short LOWs; gap = 800µs LOW.
            DecoderStep::Preamble => {
                if level {
                    return None;
                }
                let short_ok = duration_diff!(duration, TE_SHORT) < TE_DELTA;
                let gap_ok = duration_diff!(duration, GAP_US) < TE_DELTA;

                if short_ok {
                    self.preamble_count += 1;
                    self.te_last = duration;
                } else {
                    if self.preamble_count >= PREAMBLE_PAIRS && gap_ok {
                        self.step = DecoderStep::Data;
                        self.preamble_count = 0;
                        self.data_low = 0;
                        self.data_high = 0;
                        self.bit_count = 0;
                        self.te_last = duration;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                }
            }

            // Data: Manchester events — short first, then long (matches reference)
            DecoderStep::Data => {
                let short_diff = duration_diff!(duration, TE_SHORT);
                let long_diff = duration_diff!(duration, TE_LONG);

                let event = if short_diff < TE_DELTA {
                    if level { 0 } else { 1 }
                } else if long_diff < TE_DELTA {
                    if level { 2 } else { 3 }
                } else {
                    self.te_last = duration;
                    if duration > TE_LONG * 3 {
                        self.step = DecoderStep::Reset;
                    }
                    return None;
                };

                if let Some(bit) = self.manchester_advance(event) {
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

            // Preamble: 150 HIGH-LOW pairs; last LOW is gap (matches reference get_upload)
            for i in 0..PREAMBLE_PAIRS {
                signal.push(LevelDuration::new(true, TE_SHORT));
                signal.push(LevelDuration::new(
                    false,
                    if i == PREAMBLE_PAIRS - 1 { GAP_US } else { TE_SHORT },
                ));
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
