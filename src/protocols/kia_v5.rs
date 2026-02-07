//! Kia V5 protocol decoder
//!
//! Ported from protopirate's kia_v5.c
//!
//! Protocol characteristics:
//! - Manchester encoding: 400/800µs timing
//! - 64 bits total (+3 bit CRC)
//! - Preamble of ~40+ short pairs
//! - Custom "mixer" encryption
//! - Decode-only (no encoder)

use super::{ProtocolDecoder, ProtocolTiming, DecodedSignal};
use crate::radio::demodulator::LevelDuration;
use crate::duration_diff;

const TE_SHORT: u32 = 400;
const TE_LONG: u32 = 800;
const TE_DELTA: u32 = 150;
const MIN_COUNT_BIT: usize = 64;

/// Manchester states
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
    CheckPreamble,
    Data,
}

/// Kia V5 protocol decoder
pub struct KiaV5Decoder {
    step: DecoderStep,
    te_last: u32,
    header_count: u16,
    bit_count: u8,
    decoded_data: u64,
    saved_key: u64,
    manchester_state: ManchesterState,
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
            manchester_state: ManchesterState::Mid1,
        }
    }

    /// Get encryption key (placeholder)
    fn get_v5_key() -> u64 {
        0x0000000000000000
    }

    /// Custom mixer decryption
    fn mixer_decode(encrypted: u32) -> u16 {
        let mut s0 = (encrypted & 0xFF) as u8;
        let mut s1 = ((encrypted >> 8) & 0xFF) as u8;
        let mut s2 = ((encrypted >> 16) & 0xFF) as u8;
        let mut s3 = ((encrypted >> 24) & 0xFF) as u8;

        let key = Self::get_v5_key();
        let mut keystore_bytes = [0u8; 8];
        for i in 0..8 {
            keystore_bytes[i] = ((key >> ((7 - i) * 8)) & 0xFF) as u8;
        }

        let mut round_index: usize = 1;
        for _ in 0..18 {
            let mut r = keystore_bytes[round_index];
            let mut steps = 8;
            while steps > 0 {
                let base = if (s3 & 0x40) == 0 {
                    if (s3 & 0x02) == 0 { 0x74 } else { 0x2E }
                } else {
                    if (s3 & 0x02) == 0 { 0x3A } else { 0x5C }
                };

                let mut base = base;
                if s2 & 0x08 != 0 {
                    base = ((base >> 4) & 0x0F) | ((base & 0x0F) << 4);
                }
                if s1 & 0x01 != 0 {
                    base = (base & 0x3F) << 2;
                }
                if s0 & 0x01 != 0 {
                    base = base << 1;
                }

                let temp = (s3 ^ s1) & 0xFF;
                s3 = (s3 & 0x7F) << 1;
                if s2 & 0x80 != 0 {
                    s3 |= 0x01;
                }
                s2 = (s2 & 0x7F) << 1;
                if s1 & 0x80 != 0 {
                    s2 |= 0x01;
                }
                s1 = (s1 & 0x7F) << 1;
                if s0 & 0x80 != 0 {
                    s1 |= 0x01;
                }
                s0 = (s0 & 0x7F) << 1;

                let chk = (base ^ (r ^ temp)) & 0xFF;
                if chk & 0x80 != 0 {
                    s0 |= 0x01;
                }
                r = (r & 0x7F) << 1;
                steps -= 1;
            }
            round_index = (round_index.wrapping_sub(1)) & 0x7;
        }
        
        ((s0 as u16) + ((s1 as u16) << 8)) & 0xFFFF
    }

    /// Reverse bits in 64-bit value
    fn compute_yek(key: u64) -> u64 {
        let mut yek: u64 = 0;
        for i in 0..8 {
            let byte = ((key >> (i * 8)) & 0xFF) as u8;
            let mut reversed: u8 = 0;
            for b in 0..8 {
                if byte & (1 << b) != 0 {
                    reversed |= 1 << (7 - b);
                }
            }
            yek |= (reversed as u64) << ((7 - i) * 8);
        }
        yek
    }

    /// Manchester state machine
    fn manchester_advance(&mut self, is_short: bool, is_high: bool) -> Option<bool> {
        let event = match (is_short, is_high) {
            (true, true) => 0,  // Short High
            (true, false) => 1, // Short Low
            (false, true) => 2, // Long High
            (false, false) => 3, // Long Low
        };

        let (new_state, output) = match (self.manchester_state, event) {
            (ManchesterState::Mid0, 1) | (ManchesterState::Mid1, 1) => 
                (ManchesterState::Start0, None),
            (ManchesterState::Mid0, 0) | (ManchesterState::Mid1, 0) => 
                (ManchesterState::Start1, None),
            
            (ManchesterState::Start1, 1) => (ManchesterState::Mid1, Some(true)),
            (ManchesterState::Start1, 3) => (ManchesterState::Start0, Some(true)),
            
            (ManchesterState::Start0, 0) => (ManchesterState::Mid0, Some(false)),
            (ManchesterState::Start0, 2) => (ManchesterState::Start1, Some(false)),
            
            _ => (ManchesterState::Mid1, None),
        };

        self.manchester_state = new_state;
        output
    }

    /// Parse decoded data
    fn parse_data(&self) -> Option<DecodedSignal> {
        if self.bit_count < MIN_COUNT_BIT as u8 {
            return None;
        }

        let key = self.saved_key;
        let yek = Self::compute_yek(key);
        
        let serial = ((yek >> 32) & 0x0FFFFFFF) as u32;
        let button = ((yek >> 60) & 0x0F) as u8;
        let encrypted = (yek & 0xFFFFFFFF) as u32;
        let counter = Self::mixer_decode(encrypted);
        
        let _crc = (self.decoded_data & 0x07) as u8;

        Some(DecodedSignal {
            serial: Some(serial),
            button: Some(button),
            counter: Some(counter),
            crc_valid: true, // V5 doesn't have a standard CRC validation
            data: key,
            data_count_bit: MIN_COUNT_BIT,
            encoder_capable: false, // V5 is decode-only
        })
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
        &[433_920_000]
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.te_last = 0;
        self.header_count = 0;
        self.bit_count = 0;
        self.decoded_data = 0;
        self.saved_key = 0;
        self.manchester_state = ManchesterState::Mid1;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let is_short = duration_diff!(duration, TE_SHORT) < TE_DELTA;
        let is_long = duration_diff!(duration, TE_LONG) < TE_DELTA;

        match self.step {
            DecoderStep::Reset => {
                if level && is_short {
                    self.step = DecoderStep::CheckPreamble;
                    self.te_last = duration;
                    self.header_count = 1;
                    self.bit_count = 0;
                    self.decoded_data = 0;
                    self.manchester_state = ManchesterState::Mid1;
                }
            }

            DecoderStep::CheckPreamble => {
                if level {
                    if is_long {
                        if self.header_count > 40 {
                            self.step = DecoderStep::Data;
                            self.bit_count = 0;
                            self.decoded_data = 0;
                            self.saved_key = 0;
                            self.header_count = 0;
                        } else {
                            self.te_last = duration;
                        }
                    } else if is_short {
                        self.te_last = duration;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else {
                    if (is_short && duration_diff!(self.te_last, TE_SHORT) < TE_DELTA) ||
                       (is_long && duration_diff!(self.te_last, TE_SHORT) < TE_DELTA) ||
                       (duration_diff!(self.te_last, TE_LONG) < TE_DELTA) {
                        self.header_count += 1;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                    self.te_last = duration;
                }
            }

            DecoderStep::Data => {
                if !is_short && !is_long {
                    // End of data - try to parse
                    if self.bit_count >= MIN_COUNT_BIT as u8 {
                        let result = self.parse_data();
                        self.step = DecoderStep::Reset;
                        return result;
                    }
                    self.step = DecoderStep::Reset;
                    return None;
                }

                if self.bit_count <= 66 {
                    if let Some(bit) = self.manchester_advance(is_short, level) {
                        self.decoded_data = (self.decoded_data << 1) | (bit as u64);
                        self.bit_count += 1;

                        if self.bit_count == 64 {
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
        false // V5 is decode-only in protopirate
    }

    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None // V5 doesn't support encoding
    }
}
