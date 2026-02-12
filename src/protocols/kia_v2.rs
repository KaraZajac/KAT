//! Kia V2 protocol decoder/encoder
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/kia_v2.c`.
//! Decode/encode logic (Manchester, CRC4, preamble, byte-swapped counter) matches reference.
//!
//! Protocol characteristics:
//! - Manchester encoding: 500/1000µs timing
//! - 53 bits total (32 serial + 4 button + 12 counter + 4 CRC, plus start bit)
//! - Long preamble of 252 long pairs
//! - CRC4 checksum (XOR nibbles + offset 1)

use super::{ProtocolDecoder, ProtocolTiming, DecodedSignal};
use crate::radio::demodulator::LevelDuration;
use crate::duration_diff;

const TE_SHORT: u32 = 500;
const TE_LONG: u32 = 1000;
const TE_DELTA: u32 = 150;
const MIN_COUNT_BIT: usize = 53;

/// Manchester decoder states (matches protopirate kia_v2 Manchester state machine)
#[derive(Debug, Clone, Copy, PartialEq)]
enum ManchesterState {
    Mid0,
    Mid1,
    Start0,
    Start1,
}

/// Decoder states (matches protopirate's KiaV2DecoderStep)
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    CheckPreamble,
    CollectRawBits,
}

/// Kia V2 protocol decoder
pub struct KiaV2Decoder {
    step: DecoderStep,
    te_last: u32,
    header_count: u16,
    decode_data: u64,
    decode_count_bit: usize,
    manchester_state: ManchesterState,
}

impl KiaV2Decoder {
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

    /// CRC4 for Kia V2 (matches kia_v2.c: 6-byte permuted input, XOR nibbles, offset 1)
    fn calculate_crc(data: u64) -> u8 {
        let serial = ((data >> 20) & 0xFFFFFFFF) as u32;
        let u_var4 = (data & 0xFFFFFFFF) as u32;

        let mut bytes = [0u8; 6];
        bytes[0] = (u_var4 >> 20) as u8;
        bytes[1] = ((u_var4 >> 28) | ((serial & 0x0F) << 4)) as u8;
        bytes[2] = (serial >> 4) as u8;
        bytes[3] = (serial >> 12) as u8;
        bytes[4] = (u_var4 >> 4) as u8;
        bytes[5] = (u_var4 >> 12) as u8;

        let mut crc: u8 = 0;
        for &byte in &bytes {
            crc ^= (byte & 0x0F) ^ (byte >> 4);
        }

        (crc.wrapping_add(1)) & 0x0F
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

    /// Parse decoded data (field layout matches kia_v2.c)
    fn parse_data(&self) -> DecodedSignal {
        let data = self.decode_data;
        // serial(32) | button(4) | counter_swapped(12) | crc(4); counter byte-swapped in stream
        let serial = ((data >> 20) & 0xFFFFFFFF) as u32;
        let button = ((data >> 16) & 0x0F) as u8;
        
        // Counter has byte-swapped format
        let raw_count = ((data >> 4) & 0xFFF) as u16;
        let counter = ((raw_count >> 4) | (raw_count << 8)) & 0xFFF;
        
        let received_crc = (data & 0x0F) as u8;
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

impl ProtocolDecoder for KiaV2Decoder {
    fn name(&self) -> &'static str {
        "Kia V2"
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
                    self.manchester_state = ManchesterState::Mid1;
                }
            }

            DecoderStep::CheckPreamble => {
                if level {
                    if is_long {
                        self.te_last = duration;
                        self.header_count += 1;
                    } else if is_short && self.header_count >= 100 {
                        self.header_count = 0;
                        self.decode_data = 0;
                        self.decode_count_bit = 1;
                        self.step = DecoderStep::CollectRawBits;
                        self.decode_data = 1; // First bit
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else {
                    if is_long {
                        self.header_count += 1;
                        self.te_last = duration;
                    } else if !is_short {
                        self.step = DecoderStep::Reset;
                    }
                }
            }

            DecoderStep::CollectRawBits => {
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

        // Reconstruct data in V2 format
        let u_var6 = ((counter & 0xFF) as u32) << 8 |
                     ((button & 0x0F) as u32) << 16 |
                     (((counter >> 4) & 0xF0) as u32);

        let mut new_data: u64 = 1u64 << 52; // Start bit
        new_data |= ((serial as u64) << 20) & 0xFFFFFFFFF00000;
        new_data |= u_var6 as u64;

        // Calculate and apply CRC
        let crc = Self::calculate_crc(new_data);
        new_data = (new_data & !0x0F) | (crc as u64);

        let mut signal = Vec::with_capacity(700);

        // Generate 2 bursts (matches protopirate kia_v2 encode)
        for _burst in 0..2 {
            // Preamble: 252 long pairs
            for _ in 0..252 {
                signal.push(LevelDuration::new(false, TE_LONG));
                signal.push(LevelDuration::new(true, TE_LONG));
            }

            // Short gap before data
            signal.push(LevelDuration::new(false, TE_SHORT));

            // Data: 53 bits Manchester encoded, MSB first
            for bit_num in (1..MIN_COUNT_BIT).rev() {
                let bit = ((new_data >> (bit_num - 1)) & 1) == 1;
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

impl Default for KiaV2Decoder {
    fn default() -> Self {
        Self::new()
    }
}
