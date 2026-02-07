//! Ford V0 protocol decoder
//!
//! Ported from protopirate's ford_v0.c
//!
//! Protocol characteristics:
//! - Manchester encoding: 250/500µs timing
//! - 64 bits total
//! - Matrix-based CRC

use super::{ProtocolDecoder, ProtocolTiming, DecodedSignal};
use crate::radio::demodulator::LevelDuration;
use crate::duration_diff;

const TE_SHORT: u32 = 250;
const TE_LONG: u32 = 500;
const TE_DELTA: u32 = 100;
const MIN_COUNT_BIT: usize = 64;

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
    CheckPreamble,
    SaveDuration,
    CheckDuration,
}

/// Ford V0 protocol decoder
pub struct FordV0Decoder {
    step: DecoderStep,
    te_last: u32,
    header_count: u16,
    decode_data: u64,
    decode_count_bit: usize,
    manchester_state: ManchesterState,
}

impl FordV0Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            te_last: 0,
            header_count: 0,
            decode_data: 0,
            decode_count_bit: 0,
            manchester_state: ManchesterState::Mid1,
        }
    }

    /// Manchester decode: advance state machine
    fn manchester_advance(&mut self, is_short: bool, is_high: bool) -> Option<bool> {
        let event = match (is_short, is_high) {
            (true, true) => 0,   // Short High
            (true, false) => 1,  // Short Low
            (false, true) => 2,  // Long High
            (false, false) => 3, // Long Low
        };

        let (new_state, output) = match (self.manchester_state, event) {
            // From Mid0 or Mid1
            (ManchesterState::Mid0, 0) | (ManchesterState::Mid1, 0) => 
                (ManchesterState::Start1, None),
            (ManchesterState::Mid0, 1) | (ManchesterState::Mid1, 1) => 
                (ManchesterState::Start0, None),
            
            // From Start1
            (ManchesterState::Start1, 1) => (ManchesterState::Mid1, Some(true)),
            (ManchesterState::Start1, 3) => (ManchesterState::Start0, Some(true)),
            
            // From Start0
            (ManchesterState::Start0, 0) => (ManchesterState::Mid0, Some(false)),
            (ManchesterState::Start0, 2) => (ManchesterState::Start1, Some(false)),
            
            // Reset on invalid transitions
            _ => (ManchesterState::Mid1, None),
        };

        self.manchester_state = new_state;
        output
    }

    /// CRC matrix for Ford
    const CRC_MATRIX: [[u8; 4]; 8] = [
        [0x0C, 0xBB, 0x51, 0x25],
        [0x18, 0xB1, 0x62, 0xCB],
        [0x30, 0xA7, 0x44, 0x57],
        [0x60, 0x89, 0x88, 0xAE],
        [0xC0, 0xD6, 0xD4, 0x97],
        [0x25, 0x49, 0x6D, 0xE1],
        [0x4A, 0x92, 0xDA, 0x03],
        [0x94, 0xE1, 0x71, 0x06],
    ];

    /// Calculate Ford CRC
    fn calculate_crc(data: u64) -> u8 {
        let mut crc = 0u8;
        
        for byte_idx in 0..7 {
            let byte = ((data >> (56 - byte_idx * 8)) & 0xFF) as u8;
            for bit in 0..8 {
                if (byte >> (7 - bit)) & 1 == 1 {
                    crc ^= Self::CRC_MATRIX[bit][byte_idx % 4];
                }
            }
        }
        
        crc
    }

    /// Parse decoded data
    fn parse_data(data: u64) -> DecodedSignal {
        // Ford V0 format:
        // Bits 60-63: Prefix (0x5)
        // Bits 32-59: Serial (28 bits)
        // Bits 28-31: Button (4 bits)
        // Bits 16-27: Counter (12 bits)
        // Bits 8-15: Encrypted data
        // Bits 0-7: CRC
        
        let serial = ((data >> 32) & 0x0FFFFFFF) as u32;
        let button = ((data >> 28) & 0x0F) as u8;
        let counter = ((data >> 16) & 0x0FFF) as u16;
        let received_crc = (data & 0xFF) as u8;
        let calculated_crc = Self::calculate_crc(data);

        DecodedSignal {
            serial: Some(serial),
            button: Some(button),
            counter: Some(counter),
            crc_valid: received_crc == calculated_crc,
            data,
            data_count_bit: MIN_COUNT_BIT,
            encoder_capable: true,
        }
    }
}

impl ProtocolDecoder for FordV0Decoder {
    fn name(&self) -> &'static str {
        "Ford V0"
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
        &[315_000_000, 433_920_000] // 315 MHz (US) and 433.92 MHz (EU)
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.te_last = 0;
        self.header_count = 0;
        self.decode_data = 0;
        self.decode_count_bit = 0;
        self.manchester_state = ManchesterState::Mid1;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let is_short = duration_diff!(duration, TE_SHORT) < TE_DELTA;
        let is_long = duration_diff!(duration, TE_LONG) < TE_DELTA;

        match self.step {
            DecoderStep::Reset => {
                if level && is_short {
                    self.step = DecoderStep::CheckPreamble;
                    self.header_count = 1;
                    self.manchester_state = ManchesterState::Mid1;
                }
            }

            DecoderStep::CheckPreamble => {
                if is_short {
                    self.header_count += 1;
                    if self.header_count > 20 && !level {
                        // Enough preamble, start looking for data
                        self.step = DecoderStep::SaveDuration;
                        self.decode_data = 0;
                        self.decode_count_bit = 0;
                        self.manchester_state = ManchesterState::Mid1;
                    }
                } else if is_long {
                    if self.header_count > 10 {
                        self.step = DecoderStep::SaveDuration;
                        self.decode_data = 0;
                        self.decode_count_bit = 0;
                        self.manchester_state = ManchesterState::Mid1;
                        
                        // Process this long pulse
                        if let Some(bit) = self.manchester_advance(false, level) {
                            self.decode_data = (self.decode_data << 1) | (bit as u64);
                            self.decode_count_bit += 1;
                        }
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::SaveDuration => {
                self.te_last = duration;
                self.step = DecoderStep::CheckDuration;
            }

            DecoderStep::CheckDuration => {
                let last_short = duration_diff!(self.te_last, TE_SHORT) < TE_DELTA;
                let last_long = duration_diff!(self.te_last, TE_LONG) < TE_DELTA;
                
                // Check for end of transmission
                if duration > TE_LONG * 3 {
                    if self.decode_count_bit >= MIN_COUNT_BIT {
                        let result = Self::parse_data(self.decode_data);
                        self.step = DecoderStep::Reset;
                        return Some(result);
                    }
                    self.step = DecoderStep::Reset;
                    return None;
                }

                // Manchester decode
                if last_short {
                    if let Some(bit) = self.manchester_advance(true, !level) {
                        self.decode_data = (self.decode_data << 1) | (bit as u64);
                        self.decode_count_bit += 1;
                    }
                } else if last_long {
                    if let Some(bit) = self.manchester_advance(false, !level) {
                        self.decode_data = (self.decode_data << 1) | (bit as u64);
                        self.decode_count_bit += 1;
                    }
                }

                if is_short || is_long {
                    if let Some(bit) = self.manchester_advance(is_short, level) {
                        self.decode_data = (self.decode_data << 1) | (bit as u64);
                        self.decode_count_bit += 1;
                    }
                    self.step = DecoderStep::SaveDuration;
                } else {
                    self.step = DecoderStep::Reset;
                }

                // Check if we have enough bits
                if self.decode_count_bit >= MIN_COUNT_BIT {
                    let result = Self::parse_data(self.decode_data);
                    self.step = DecoderStep::Reset;
                    return Some(result);
                }
            }
        }

        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        let serial = decoded.serial?;
        let counter = decoded.counter.unwrap_or(0);

        // Build data packet
        let mut data: u64 = 0;
        data |= 0x5 << 60; // Prefix
        data |= ((serial as u64) & 0x0FFFFFFF) << 32;
        data |= ((button as u64) & 0x0F) << 28;
        data |= ((counter as u64) & 0x0FFF) << 16;
        data |= ((decoded.data >> 8) & 0xFF) << 8; // Keep encrypted byte
        data |= Self::calculate_crc(data) as u64;

        let mut signal = Vec::with_capacity(256);

        // Preamble
        for _ in 0..30 {
            signal.push(LevelDuration::new(true, TE_SHORT));
            signal.push(LevelDuration::new(false, TE_SHORT));
        }

        // Sync
        signal.push(LevelDuration::new(true, TE_LONG));
        signal.push(LevelDuration::new(false, TE_LONG));

        // Data: Manchester encoded, 64 bits MSB first
        for bit_num in (0..64).rev() {
            let bit = (data >> bit_num) & 1 == 1;
            if bit {
                // Manchester 1: low-high
                signal.push(LevelDuration::new(false, TE_SHORT));
                signal.push(LevelDuration::new(true, TE_SHORT));
            } else {
                // Manchester 0: high-low
                signal.push(LevelDuration::new(true, TE_SHORT));
                signal.push(LevelDuration::new(false, TE_SHORT));
            }
        }

        // End
        signal.push(LevelDuration::new(false, TE_LONG * 4));

        Some(signal)
    }
}
