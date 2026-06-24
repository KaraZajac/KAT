//! Ford V3 protocol decoder
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/ford_v3.c` and `ford_v3.h`.
//! Manchester 240/480µs, 104 bits (13 bytes), FM, plaintext (no CRC/encryption). Decode-only —
//! the reference encoder is NULL.
//!
//! Manchester uses Flipper's manchester_decoder.h transition table (same as Ford V0), but Ford V3
//! maps level the OPPOSITE way from Ford V0: `level ? ShortHigh/LongHigh : ShortLow/LongLow`
//! (Ford V0 uses `level ? Low : High`). Decoder steps: Reset → Preamble (≥30 shorts) → Data.

use super::{ProtocolDecoder, ProtocolTiming, DecodedSignal};
use crate::radio::demodulator::LevelDuration;
use crate::duration_diff;

const TE_SHORT: u32 = 240;
const TE_LONG: u32 = 480;
const TE_DELTA: u32 = 60;
const DATA_BITS: usize = 104;
const DATA_BYTES: usize = 13;
const PREAMBLE_MIN: u16 = 30;

const BTN_LOCK: u8 = 0x01;
const BTN_UNLOCK: u8 = 0x02;

/// Manchester state machine (Flipper manchester_decoder.h transition table; same as Ford V0).
#[derive(Debug, Clone, Copy, PartialEq)]
enum ManchesterState {
    Mid0 = 0,
    Mid1 = 1,
    Start0 = 2,
    Start1 = 3,
}

/// Decoder step states (matches FordV3DecoderStep in ford_v3.c)
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Preamble,
    Data,
}

/// Ford V3 protocol decoder (matches SubGhzProtocolDecoderFordV3)
pub struct FordV3Decoder {
    step: DecoderStep,
    manchester_state: ManchesterState,
    raw_bytes: [u8; DATA_BYTES],
    bit_count: usize,
    preamble_count: u16,
}

impl FordV3Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            manchester_state: ManchesterState::Mid1,
            raw_bytes: [0; DATA_BYTES],
            bit_count: 0,
            preamble_count: 0,
        }
    }

    /// Reset accumulators (matches ford_v3_reset_data)
    fn reset_data(&mut self) {
        self.raw_bytes = [0; DATA_BYTES];
        self.bit_count = 0;
        self.preamble_count = 0;
        self.manchester_state = ManchesterState::Mid1;
    }

    /// Add a decoded bit MSB-first into the byte buffer (matches ford_v3_add_bit)
    fn add_bit(&mut self, bit: bool) {
        if self.bit_count >= DATA_BITS {
            return;
        }
        let byte_index = self.bit_count / 8;
        let bit_in_byte = 7 - (self.bit_count % 8);
        if bit {
            self.raw_bytes[byte_index] |= 1 << bit_in_byte;
        }
        self.bit_count += 1;
    }

    /// Manchester state machine (Flipper manchester_advance).
    /// Event: 0=ShortLow, 1=ShortHigh, 2=LongLow, 3=LongHigh. Returns Some(bit) when a bit emits.
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

    fn is_short(duration: u32) -> bool {
        duration_diff!(duration, TE_SHORT) < TE_DELTA
    }

    fn is_long(duration: u32) -> bool {
        duration_diff!(duration, TE_LONG) < TE_DELTA
    }

    /// Build the decoded signal when 104 bits are collected (matches ford_v3_parse_fields).
    fn build_signal(&self) -> DecodedSignal {
        let b = &self.raw_bytes;
        let serial = ((b[1] as u32) << 24)
            | ((b[2] as u32) << 16)
            | ((b[3] as u32) << 8)
            | (b[4] as u32);
        // Counter is the bitwise-inverted bytes 7 and 8 (ref: ~b[7], ~b[8])
        let counter = (((!b[7]) as u16) << 8) | ((!b[8]) as u16);
        let button = if b[6] & 0x01 != 0 { BTN_UNLOCK } else { BTN_LOCK };

        // Pack the first 8 bytes (big-endian) into the 64-bit data field for display/export
        // (matches ford_v3.c serialize, which stores bytes[0..8] in generic.data).
        let mut data = 0u64;
        for &byte in b.iter().take(8) {
            data = (data << 8) | byte as u64;
        }

        DecodedSignal {
            serial: Some(serial),
            button: Some(button),
            counter: Some(counter),
            crc_valid: true, // Ford V3 is plaintext with no CRC
            data,
            data_count_bit: DATA_BITS,
            encoder_capable: false,
            extra: None,
            protocol_display_name: None,
        }
    }
}

impl ProtocolDecoder for FordV3Decoder {
    fn name(&self) -> &'static str {
        "Ford V3"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: DATA_BITS,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.reset_data();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            // Ref: any short pulse begins the preamble (no level check).
            DecoderStep::Reset => {
                if Self::is_short(duration) {
                    self.reset_data();
                    self.preamble_count = 1;
                    self.step = DecoderStep::Preamble;
                }
            }

            DecoderStep::Preamble => {
                if Self::is_short(duration) {
                    self.preamble_count += 1;
                } else if self.preamble_count >= PREAMBLE_MIN && Self::is_long(duration) {
                    // First data bit: long pulse, mapped level ? LongHigh : LongLow.
                    self.manchester_state = ManchesterState::Mid1;
                    let event = if level { 3 } else { 2 };
                    if let Some(bit) = self.manchester_advance(event) {
                        self.add_bit(bit);
                    }
                    self.step = DecoderStep::Data;
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::Data => {
                let short = Self::is_short(duration);
                let long = Self::is_long(duration);

                if !short && !long {
                    // Gap / out-of-range pulse ends the frame.
                    let ready = self.bit_count >= DATA_BITS;
                    let result = if ready { Some(self.build_signal()) } else { None };
                    self.step = DecoderStep::Reset;
                    self.reset_data();
                    return result;
                }

                // Ford V3 polarity: level ? High : Low (opposite of Ford V0).
                let event = if level {
                    if short { 1 } else { 3 } // ShortHigh / LongHigh
                } else if short {
                    0 // ShortLow
                } else {
                    2 // LongLow
                };

                if let Some(bit) = self.manchester_advance(event) {
                    self.add_bit(bit);
                    if self.bit_count >= DATA_BITS {
                        let result = self.build_signal();
                        self.step = DecoderStep::Reset;
                        self.reset_data();
                        return Some(result);
                    }
                }
            }
        }

        None
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }
}

impl Default for FordV3Decoder {
    fn default() -> Self {
        Self::new()
    }
}
