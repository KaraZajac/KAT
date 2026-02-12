//! Kia V0 protocol decoder/encoder
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/kia_v0.c`.
//! Decode/encode logic (CRC8, preamble/sync, field layout) matches reference.
//!
//! Protocol characteristics:
//! - PWM encoding: short pulse (250µs) = 0, long pulse (500µs) = 1
//! - 61 bits total (1 sync bit + 60 data bits)
//! - Preamble: alternating short pulses; sync: long-long pattern
//! - Data: 60 bits (4-bit prefix + 16-bit counter + 28-bit serial + 4-bit button + 8-bit CRC)

use super::{ProtocolDecoder, ProtocolTiming, DecodedSignal};
use super::common::{crc8_kia, add_bit};
use crate::radio::demodulator::LevelDuration;
use crate::duration_diff;

const TE_SHORT: u32 = 250;
const TE_LONG: u32 = 500;
const TE_DELTA: u32 = 100;
const MIN_COUNT_BIT: usize = 61;

/// Decoder states (matches protopirate's KiaV0DecoderStep)
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    CheckPreamble,
    SaveDuration,
    CheckDuration,
}

/// Kia V0 protocol decoder
pub struct KiaV0Decoder {
    step: DecoderStep,
    te_last: u32,
    header_count: u16,
    decode_data: u64,
    decode_count_bit: usize,
}

impl KiaV0Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            te_last: 0,
            header_count: 0,
            decode_data: 0,
            decode_count_bit: 0,
        }
    }

    /// CRC8 for Kia data packet (matches kia_v0.c kia_crc8: polynomial 0x7F, init 0x00)
    fn calculate_crc(data: u64) -> u8 {
        let crc_data = [
            ((data >> 48) & 0xFF) as u8,
            ((data >> 40) & 0xFF) as u8,
            ((data >> 32) & 0xFF) as u8,
            ((data >> 24) & 0xFF) as u8,
            ((data >> 16) & 0xFF) as u8,
            ((data >> 8) & 0xFF) as u8,
        ];
        crc8_kia(&crc_data)
    }

    /// Verify CRC of received data
    fn verify_crc(data: u64) -> bool {
        let received_crc = (data & 0xFF) as u8;
        let calculated_crc = Self::calculate_crc(data);
        received_crc == calculated_crc
    }

    /// Extract fields from decoded data
    fn parse_data(data: u64) -> DecodedSignal {
        let serial = ((data >> 12) & 0x0FFFFFFF) as u32;
        let button = ((data >> 8) & 0x0F) as u8;
        let counter = ((data >> 40) & 0xFFFF) as u16;
        let crc_valid = Self::verify_crc(data);

        DecodedSignal {
            serial: Some(serial),
            button: Some(button),
            counter: Some(counter),
            crc_valid,
            data,
            data_count_bit: MIN_COUNT_BIT,
            encoder_capable: true,
        }
    }
}

impl ProtocolDecoder for KiaV0Decoder {
    fn name(&self) -> &'static str {
        "Kia V0"
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
        &[433_920_000] // 433.92 MHz
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.te_last = 0;
        self.header_count = 0;
        self.decode_data = 0;
        self.decode_count_bit = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::Reset => {
                if level && duration_diff!(duration, TE_SHORT) < TE_DELTA {
                    self.step = DecoderStep::CheckPreamble;
                    self.te_last = duration;
                    self.header_count = 0;
                }
            }

            DecoderStep::CheckPreamble => {
                if level {
                    if duration_diff!(duration, TE_SHORT) < TE_DELTA ||
                       duration_diff!(duration, TE_LONG) < TE_DELTA {
                        self.te_last = duration;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else if duration_diff!(duration, TE_SHORT) < TE_DELTA &&
                          duration_diff!(self.te_last, TE_SHORT) < TE_DELTA {
                    // Short-short pair in preamble
                    self.header_count += 1;
                } else if duration_diff!(duration, TE_LONG) < TE_DELTA &&
                          duration_diff!(self.te_last, TE_LONG) < TE_DELTA {
                    // Long-long sync pattern
                    if self.header_count > 15 {
                        self.step = DecoderStep::SaveDuration;
                        self.decode_data = 0;
                        self.decode_count_bit = 1;
                        // Add first bit (the sync is also a '1' bit)
                        add_bit(&mut self.decode_data, &mut self.decode_count_bit, true);
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::SaveDuration => {
                if level {
                    if duration >= TE_LONG + TE_DELTA * 2 {
                        // End of transmission
                        self.step = DecoderStep::Reset;

                        if self.decode_count_bit == MIN_COUNT_BIT {
                            return Some(Self::parse_data(self.decode_data));
                        }
                    } else {
                        self.te_last = duration;
                        self.step = DecoderStep::CheckDuration;
                    }
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::CheckDuration => {
                if !level {
                    if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA &&
                       duration_diff!(duration, TE_SHORT) < TE_DELTA {
                        // Short-short = bit 0
                        add_bit(&mut self.decode_data, &mut self.decode_count_bit, false);
                        self.step = DecoderStep::SaveDuration;
                    } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA &&
                              duration_diff!(duration, TE_LONG) < TE_DELTA {
                        // Long-long = bit 1
                        add_bit(&mut self.decode_data, &mut self.decode_count_bit, true);
                        self.step = DecoderStep::SaveDuration;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else {
                    self.step = DecoderStep::Reset;
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
        
        // Bits 56-59: Preserve from original (usually 0xF)
        data |= decoded.data & 0x0F00000000000000;
        
        // Bits 40-55: Counter (16 bits)
        data |= ((counter as u64) & 0xFFFF) << 40;
        
        // Bits 12-39: Serial (28 bits)
        data |= ((serial as u64) & 0x0FFFFFFF) << 12;
        
        // Bits 8-11: Button (4 bits)
        data |= ((button as u64) & 0x0F) << 8;
        
        // Bits 0-7: CRC
        let crc = Self::calculate_crc(data);
        data |= crc as u64;

        let mut signal = Vec::with_capacity(256);

        // Generate 2 bursts
        for burst in 0..2 {
            if burst > 0 {
                // Inter-burst gap
                signal.push(LevelDuration::new(false, 25000));
            }

            // Preamble: 32 alternating short pulses
            for i in 0..32 {
                let is_high = (i % 2) == 0;
                signal.push(LevelDuration::new(is_high, TE_SHORT));
            }

            // Sync: long-long (matches protopirate kia_v0 encode)
            signal.push(LevelDuration::new(true, TE_LONG));
            signal.push(LevelDuration::new(false, TE_LONG));

            // Data: 60 bits MSB first (bits 59..0)
            for bit_num in 0..60 {
                let bit_mask = 1u64 << (59 - bit_num);
                let bit = (data & bit_mask) != 0;
                let duration = if bit { TE_LONG } else { TE_SHORT };

                signal.push(LevelDuration::new(true, duration));
                signal.push(LevelDuration::new(false, duration));
            }

            // End marker
            signal.push(LevelDuration::new(true, TE_LONG * 2));
        }

        Some(signal)
    }
}

impl Default for KiaV0Decoder {
    fn default() -> Self {
        Self::new()
    }
}
