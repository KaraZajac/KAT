//! Kia V1 protocol decoder
//!
//! Ported from protopirate's kia_v1.c
//!
//! Protocol characteristics:
//! - Manchester encoding: 800/1600µs timing
//! - 57 bits total
//! - Long preamble of ~90 pulses
//! - CRC4 checksum

use super::{ProtocolDecoder, ProtocolTiming, DecodedSignal};
use crate::radio::demodulator::LevelDuration;
use crate::duration_diff;

const TE_SHORT: u32 = 800;
const TE_LONG: u32 = 1600;
const TE_DELTA: u32 = 200;
const MIN_COUNT_BIT: usize = 57;

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
    DecodeData,
}

/// Kia V1 protocol decoder
pub struct KiaV1Decoder {
    step: DecoderStep,
    te_last: u32,
    header_count: u16,
    decode_data: u64,
    decode_count_bit: usize,
    manchester_state: ManchesterState,
}

impl KiaV1Decoder {
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

    /// CRC4 calculation for Kia V1
    fn crc4(bytes: &[u8], offset: u8) -> u8 {
        let mut crc: u8 = 0;
        for &byte in bytes {
            crc ^= (byte & 0x0F) ^ (byte >> 4);
        }
        (crc.wrapping_add(offset)) & 0x0F
    }

    /// Manchester state machine
    fn manchester_advance(&mut self, is_short: bool, is_high: bool) -> Option<bool> {
        let event = match (is_short, is_high) {
            (true, false) => 0,  // Short Low
            (true, true) => 1,   // Short High
            (false, false) => 2, // Long Low
            (false, true) => 3,  // Long High
        };

        let (new_state, output) = match (self.manchester_state, event) {
            (ManchesterState::Mid0, 0) | (ManchesterState::Mid1, 0) => 
                (ManchesterState::Start0, None),
            (ManchesterState::Mid0, 1) | (ManchesterState::Mid1, 1) => 
                (ManchesterState::Start1, None),
            
            (ManchesterState::Start1, 0) => (ManchesterState::Mid1, Some(true)),
            (ManchesterState::Start1, 2) => (ManchesterState::Start0, Some(true)),
            
            (ManchesterState::Start0, 1) => (ManchesterState::Mid0, Some(false)),
            (ManchesterState::Start0, 3) => (ManchesterState::Start1, Some(false)),
            
            _ => (ManchesterState::Mid1, None),
        };

        self.manchester_state = new_state;
        output
    }

    /// Parse decoded data
    fn parse_data(&self) -> DecodedSignal {
        let data = self.decode_data;
        
        // Extract fields per kia_v1.c
        let serial = (data >> 24) as u32;
        let button = ((data >> 16) & 0xFF) as u8;
        let cnt_low = ((data >> 8) & 0xFF) as u16;
        let cnt_high = ((data >> 4) & 0x0F) as u16;
        let counter = (cnt_high << 8) | cnt_low;
        let received_crc = (data & 0x0F) as u8;

        // Calculate CRC
        let mut char_data = [0u8; 7];
        char_data[0] = ((serial >> 24) & 0xFF) as u8;
        char_data[1] = ((serial >> 16) & 0xFF) as u8;
        char_data[2] = ((serial >> 8) & 0xFF) as u8;
        char_data[3] = (serial & 0xFF) as u8;
        char_data[4] = button;
        char_data[5] = (counter & 0xFF) as u8;

        let crc = if cnt_high == 0 {
            let offset = if counter >= 0x098 { button } else { 1 };
            Self::crc4(&char_data[..6], offset)
        } else if cnt_high >= 0x6 {
            char_data[6] = cnt_high as u8;
            Self::crc4(&char_data, 1)
        } else {
            Self::crc4(&char_data[..6], 1)
        };

        DecodedSignal {
            serial: Some(serial),
            button: Some(button),
            counter: Some(counter),
            crc_valid: received_crc == crc,
            data,
            data_count_bit: MIN_COUNT_BIT,
            encoder_capable: true,
        }
    }
}

impl ProtocolDecoder for KiaV1Decoder {
    fn name(&self) -> &'static str {
        "Kia V1"
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
        &[315_000_000, 433_920_000]
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
                if level && is_long {
                    self.step = DecoderStep::CheckPreamble;
                    self.te_last = duration;
                    self.header_count = 0;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.manchester_state = ManchesterState::Mid1;
                }
            }

            DecoderStep::CheckPreamble => {
                if !level {
                    if is_long && duration_diff!(self.te_last, TE_LONG) < TE_DELTA {
                        self.header_count += 1;
                        self.te_last = duration;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                }
                
                if self.header_count > 70 {
                    if !level && is_short && duration_diff!(self.te_last, TE_LONG) < TE_DELTA {
                        self.decode_count_bit = 1;
                        self.decode_data = 1; // Add first bit
                        self.header_count = 0;
                        self.step = DecoderStep::DecodeData;
                    }
                }
            }

            DecoderStep::DecodeData => {
                if is_short {
                    if let Some(bit) = self.manchester_advance(true, level) {
                        self.decode_data = (self.decode_data << 1) | (bit as u64);
                        self.decode_count_bit += 1;
                    }
                } else if is_long {
                    if let Some(bit) = self.manchester_advance(false, level) {
                        self.decode_data = (self.decode_data << 1) | (bit as u64);
                        self.decode_count_bit += 1;
                    }
                } else {
                    self.step = DecoderStep::Reset;
                    return None;
                }

                if self.decode_count_bit >= MIN_COUNT_BIT {
                    let result = self.parse_data();
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

        // Calculate CRC
        let cnt_high = ((counter >> 8) & 0x0F) as u8;
        let mut char_data = [0u8; 7];
        char_data[0] = ((serial >> 24) & 0xFF) as u8;
        char_data[1] = ((serial >> 16) & 0xFF) as u8;
        char_data[2] = ((serial >> 8) & 0xFF) as u8;
        char_data[3] = (serial & 0xFF) as u8;
        char_data[4] = button;
        char_data[5] = (counter & 0xFF) as u8;

        let crc = if cnt_high == 0 {
            let offset = if counter >= 0x098 { button } else { 1 };
            Self::crc4(&char_data[..6], offset)
        } else if cnt_high >= 0x6 {
            char_data[6] = cnt_high;
            Self::crc4(&char_data, 1)
        } else {
            Self::crc4(&char_data[..6], 1)
        };

        // Build data
        let data: u64 = ((serial as u64) << 24) |
                        ((button as u64) << 16) |
                        (((counter & 0xFF) as u64) << 8) |
                        ((cnt_high as u64) << 4) |
                        (crc as u64);

        let mut signal = Vec::with_capacity(600);

        // Generate 3 bursts
        for burst in 0..3 {
            if burst > 0 {
                signal.push(LevelDuration::new(false, 25000));
            }

            // Preamble: 90 long pairs
            for _ in 0..90 {
                signal.push(LevelDuration::new(false, TE_LONG));
                signal.push(LevelDuration::new(true, TE_LONG));
            }

            // Short gap
            signal.push(LevelDuration::new(false, TE_SHORT));

            // Data: Manchester encoded, MSB first
            for bit_num in (1..MIN_COUNT_BIT).rev() {
                let bit = ((data >> (bit_num - 1)) & 1) == 1;
                if bit {
                    signal.push(LevelDuration::new(true, TE_SHORT));
                    signal.push(LevelDuration::new(false, TE_SHORT));
                } else {
                    signal.push(LevelDuration::new(false, TE_SHORT));
                    signal.push(LevelDuration::new(true, TE_SHORT));
                }
            }
        }

        Some(signal)
    }
}
