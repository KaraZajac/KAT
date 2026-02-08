//! Subaru protocol decoder
//!
//! Ported from protopirate's subaru.c
//!
//! Protocol characteristics:
//! - PWM encoding: short HIGH (800µs) = 1, long HIGH (1600µs) = 0
//! - 64 bits total
//! - Long preamble of 1600µs pulses
//! - Gap and sync pattern
//! - Complex counter encoding

use super::{ProtocolDecoder, ProtocolTiming, DecodedSignal};
use crate::radio::demodulator::LevelDuration;
use crate::duration_diff;

const TE_SHORT: u32 = 800;
const TE_LONG: u32 = 1600;
const TE_DELTA: u32 = 300;
#[allow(dead_code)]
const MIN_COUNT_BIT: usize = 64;

const GAP_US: u32 = 2800;
const SYNC_US: u32 = 2800;

/// Decoder states
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    CheckPreamble,
    FoundGap,
    FoundSync,
    SaveDuration,
    CheckDuration,
}

/// Subaru protocol decoder
pub struct SubaruDecoder {
    step: DecoderStep,
    te_last: u32,
    header_count: u16,
    data: [u8; 8],
    bit_count: usize,
}

impl SubaruDecoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            te_last: 0,
            header_count: 0,
            data: [0u8; 8],
            bit_count: 0,
        }
    }

    /// Add a bit to the data buffer
    fn add_bit(&mut self, bit: bool) {
        if self.bit_count < 64 {
            let byte_idx = self.bit_count / 8;
            let bit_idx = 7 - (self.bit_count % 8);
            if bit {
                self.data[byte_idx] |= 1 << bit_idx;
            } else {
                self.data[byte_idx] &= !(1 << bit_idx);
            }
            self.bit_count += 1;
        }
    }

    /// Decode the counter from the complex Subaru encoding
    fn decode_counter(kb: &[u8; 8]) -> u16 {
        let mut lo: u8 = 0;
        if (kb[4] & 0x40) == 0 { lo |= 0x01; }
        if (kb[4] & 0x80) == 0 { lo |= 0x02; }
        if (kb[5] & 0x01) == 0 { lo |= 0x04; }
        if (kb[5] & 0x02) == 0 { lo |= 0x08; }
        if (kb[6] & 0x01) == 0 { lo |= 0x10; }
        if (kb[6] & 0x02) == 0 { lo |= 0x20; }
        if (kb[5] & 0x40) == 0 { lo |= 0x40; }
        if (kb[5] & 0x80) == 0 { lo |= 0x80; }

        let mut reg_sh1 = (kb[7] << 4) & 0xF0;
        if kb[5] & 0x04 != 0 { reg_sh1 |= 0x04; }
        if kb[5] & 0x08 != 0 { reg_sh1 |= 0x08; }
        if kb[6] & 0x80 != 0 { reg_sh1 |= 0x02; }
        if kb[6] & 0x40 != 0 { reg_sh1 |= 0x01; }

        let reg_sh2 = ((kb[6] << 2) & 0xF0) | ((kb[7] >> 4) & 0x0F);

        let mut ser0 = kb[3];
        let mut ser1 = kb[1];
        let mut ser2 = kb[2];

        let total_rot = 4 + lo;
        for _ in 0..total_rot {
            let t_bit = (ser0 >> 7) & 1;
            ser0 = ((ser0 << 1) & 0xFE) | ((ser1 >> 7) & 1);
            ser1 = ((ser1 << 1) & 0xFE) | ((ser2 >> 7) & 1);
            ser2 = ((ser2 << 1) & 0xFE) | t_bit;
        }

        let t1 = ser1 ^ reg_sh1;
        let t2 = ser2 ^ reg_sh2;

        let mut hi: u8 = 0;
        if (t1 & 0x10) == 0 { hi |= 0x04; }
        if (t1 & 0x20) == 0 { hi |= 0x08; }
        if (t2 & 0x80) == 0 { hi |= 0x02; }
        if (t2 & 0x40) == 0 { hi |= 0x01; }
        if (t1 & 0x01) == 0 { hi |= 0x40; }
        if (t1 & 0x02) == 0 { hi |= 0x80; }
        if (t2 & 0x08) == 0 { hi |= 0x20; }
        if (t2 & 0x04) == 0 { hi |= 0x10; }

        ((hi as u16) << 8) | (lo as u16)
    }

    /// Add a level+duration to the signal, merging with the previous entry
    /// if it has the same level. This prevents consecutive same-level pulses
    /// which would silently merge during HackRF transmission.
    fn add_level(signal: &mut Vec<LevelDuration>, level: bool, duration: u32) {
        if let Some(last) = signal.last_mut() {
            if last.level == level {
                *last = LevelDuration::new(level, last.duration_us + duration);
                return;
            }
        }
        signal.push(LevelDuration::new(level, duration));
    }

    /// Process the decoded data
    fn process_data(&self) -> Option<DecodedSignal> {
        if self.bit_count < 64 {
            return None;
        }

        let b = &self.data;
        
        // Build 64-bit key
        let key = ((b[0] as u64) << 56) | ((b[1] as u64) << 48) |
                  ((b[2] as u64) << 40) | ((b[3] as u64) << 32) |
                  ((b[4] as u64) << 24) | ((b[5] as u64) << 16) |
                  ((b[6] as u64) << 8)  | (b[7] as u64);

        let serial = ((b[1] as u32) << 16) | ((b[2] as u32) << 8) | (b[3] as u32);
        let button = b[0] & 0x0F;
        let counter = Self::decode_counter(&self.data);

        Some(DecodedSignal {
            serial: Some(serial),
            button: Some(button),
            counter: Some(counter),
            crc_valid: true, // Subaru doesn't use CRC
            data: key,
            data_count_bit: 64,
            encoder_capable: true,
        })
    }
}

impl ProtocolDecoder for SubaruDecoder {
    fn name(&self) -> &'static str {
        "Subaru"
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
        &[433_920_000, 315_000_000] // 433.92 MHz (EU/AU) and 315 MHz (US/JP)
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.te_last = 0;
        self.header_count = 0;
        self.data = [0u8; 8];
        self.bit_count = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::Reset => {
                if level && duration_diff!(duration, TE_LONG) < TE_DELTA {
                    self.step = DecoderStep::CheckPreamble;
                    self.te_last = duration;
                    self.header_count = 1;
                }
            }

            DecoderStep::CheckPreamble => {
                if !level {
                    if duration_diff!(duration, TE_LONG) < TE_DELTA {
                        self.header_count += 1;
                    } else if duration > 2000 && duration < 3500 {
                        // Gap detected
                        if self.header_count > 20 {
                            self.step = DecoderStep::FoundGap;
                        } else {
                            self.step = DecoderStep::Reset;
                        }
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else {
                    if duration_diff!(duration, TE_LONG) < TE_DELTA {
                        self.te_last = duration;
                        self.header_count += 1;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                }
            }

            DecoderStep::FoundGap => {
                if level && duration > 2000 && duration < 3500 {
                    self.step = DecoderStep::FoundSync;
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::FoundSync => {
                if !level && duration_diff!(duration, TE_LONG) < TE_DELTA {
                    self.step = DecoderStep::SaveDuration;
                    self.bit_count = 0;
                    self.data = [0u8; 8];
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::SaveDuration => {
                if level {
                    if duration_diff!(duration, TE_SHORT) < TE_DELTA {
                        // Short HIGH = bit 1
                        self.add_bit(true);
                        self.te_last = duration;
                        self.step = DecoderStep::CheckDuration;
                    } else if duration_diff!(duration, TE_LONG) < TE_DELTA {
                        // Long HIGH = bit 0
                        self.add_bit(false);
                        self.te_last = duration;
                        self.step = DecoderStep::CheckDuration;
                    } else if duration > 3000 {
                        // End of transmission
                        if self.bit_count >= 64 {
                            let result = self.process_data();
                            self.step = DecoderStep::Reset;
                            return result;
                        }
                        self.step = DecoderStep::Reset;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::CheckDuration => {
                if !level {
                    if duration_diff!(duration, TE_SHORT) < TE_DELTA ||
                       duration_diff!(duration, TE_LONG) < TE_DELTA {
                        self.step = DecoderStep::SaveDuration;
                    } else if duration > 3000 {
                        // Gap - end of packet
                        if self.bit_count >= 64 {
                            let result = self.process_data();
                            self.step = DecoderStep::Reset;
                            return result;
                        }
                        self.step = DecoderStep::Reset;
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

    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        let key = decoded.data;
        let mut signal = Vec::with_capacity(512);

        // Generate 3 bursts.
        //
        // IMPORTANT: Uses add_level() to merge adjacent same-level pulses.
        // The HackRF transmitter generates IQ samples sequentially from the
        // LevelDuration list, so consecutive same-level pairs silently merge
        // and corrupt timing. For example, the last preamble LOW (1600µs) +
        // gap LOW (2800µs) would become a single 4400µs LOW without merging.
        for burst in 0..3 {
            if burst > 0 {
                // Inter-burst silence
                Self::add_level(&mut signal, false, 25000);
            }

            // Preamble: 79 full pairs + 80th HIGH only.
            // The gap LOW replaces the 80th preamble LOW.
            for i in 0..80 {
                Self::add_level(&mut signal, true, TE_LONG);
                if i < 79 {
                    Self::add_level(&mut signal, false, TE_LONG);
                }
            }

            // Gap (replaces the 80th preamble LOW)
            Self::add_level(&mut signal, false, GAP_US);

            // Sync
            Self::add_level(&mut signal, true, SYNC_US);
            Self::add_level(&mut signal, false, TE_LONG);

            // Data: 64 bits (MSB first)
            // Short HIGH = 1, Long HIGH = 0
            for bit in (0..64).rev() {
                if (key >> bit) & 1 == 1 {
                    Self::add_level(&mut signal, true, TE_SHORT);
                } else {
                    Self::add_level(&mut signal, true, TE_LONG);
                }
                Self::add_level(&mut signal, false, TE_SHORT);
            }

            // End-of-burst gap (extends the last data LOW)
            Self::add_level(&mut signal, false, TE_LONG * 2);
        }

        Some(signal)
    }
}
