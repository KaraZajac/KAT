//! Kia V3/V4 protocol decoder/encoder
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/kia_v3_v4.c`.
//! Decode/encode logic (PWM, preamble, sync polarity, KeeLoq, CRC4, byte order) matches reference.
//!
//! Protocol characteristics:
//! - PWM encoding: 400/800µs (short=0, long=1)
//! - 68 bits total (8 bytes encrypted + 4 bits CRC)
//! - Short preamble of 16 pairs; sync 1200µs (V4: long HIGH, V3: long LOW)
//! - KeeLoq encryption (KIA manufacturer key); V3/V4 differ only in sync polarity

use super::{ProtocolDecoder, ProtocolTiming, DecodedSignal};
use super::keeloq_common::{keeloq_decrypt, keeloq_encrypt};
use super::keys;
use crate::radio::demodulator::LevelDuration;
use crate::duration_diff;

const TE_SHORT: u32 = 400;
const TE_LONG: u32 = 800;
const TE_DELTA: u32 = 150;
const MIN_COUNT_BIT: usize = 68;
const SYNC_DURATION: u32 = 1200;
const INTER_BURST_GAP_US: u32 = 10000;
const PREAMBLE_PAIRS: usize = 16;
const TOTAL_BURSTS: usize = 3;

/// Decoder states (matches protopirate's KiaV3V4DecoderStep)
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    CheckPreamble,
    CollectRawBits,
}

/// Kia V3/V4 protocol decoder
pub struct KiaV3V4Decoder {
    step: DecoderStep,
    te_last: u32,
    header_count: u16,
    raw_bits: [u8; 32],
    raw_bit_count: u16,
    is_v3_sync: bool,
}

impl KiaV3V4Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            te_last: 0,
            header_count: 0,
            raw_bits: [0; 32],
            raw_bit_count: 0,
            is_v3_sync: false,
        }
    }

    /// Reverse bits in a byte (matches kia_v3_v4.c byte order for KeeLoq payload)
    fn reverse8(byte: u8) -> u8 {
        let mut byte = byte;
        byte = (byte & 0xF0) >> 4 | (byte & 0x0F) << 4;
        byte = (byte & 0xCC) >> 2 | (byte & 0x33) << 2;
        byte = (byte & 0xAA) >> 1 | (byte & 0x55) << 1;
        byte
    }

    /// Add a raw bit to the buffer
    fn add_raw_bit(&mut self, bit: bool) {
        if self.raw_bit_count < 256 {
            let byte_idx = (self.raw_bit_count / 8) as usize;
            let bit_idx = 7 - (self.raw_bit_count % 8);
            if bit {
                self.raw_bits[byte_idx] |= 1 << bit_idx;
            } else {
                self.raw_bits[byte_idx] &= !(1 << bit_idx);
            }
            self.raw_bit_count += 1;
        }
    }

    /// CRC4 for Kia V3/V4 (matches kia_v3_v4.c: XOR nibbles over first 8 bytes)
    fn calculate_crc(bytes: &[u8]) -> u8 {
        let mut crc: u8 = 0;
        for &byte in bytes.iter().take(8) {
            crc ^= (byte & 0x0F) ^ (byte >> 4);
        }
        crc & 0x0F
    }

    /// KIA manufacturer key from keystore (type 10)
    fn get_mf_key() -> u64 {
        keys::get_keystore().get_kia_mf_key()
    }

    /// Process collected 68 bits: decrypt KeeLoq block, validate button/serial (matches kia_v3_v4.c)
    fn process_buffer(&self) -> Option<DecodedSignal> {
        if self.raw_bit_count < 68 {
            return None;
        }

        let mut b = self.raw_bits;
        // V3 sync means data is inverted
        if self.is_v3_sync {
            let num_bytes = ((self.raw_bit_count + 7) / 8) as usize;
            for i in 0..num_bytes {
                b[i] = !b[i];
            }
        }

        let _crc = (b[8] >> 4) & 0x0F;

        let encrypted = ((Self::reverse8(b[3]) as u32) << 24) |
                       ((Self::reverse8(b[2]) as u32) << 16) |
                       ((Self::reverse8(b[1]) as u32) << 8) |
                       (Self::reverse8(b[0]) as u32);

        let serial = ((Self::reverse8(b[7] & 0xF0) as u32) << 24) |
                    ((Self::reverse8(b[6]) as u32) << 16) |
                    ((Self::reverse8(b[5]) as u32) << 8) |
                    (Self::reverse8(b[4]) as u32);

        let button = (Self::reverse8(b[7]) & 0xF0) >> 4;
        let our_serial_lsb = (serial & 0xFF) as u8;

        let mf_key = Self::get_mf_key();
        let decrypted = keeloq_decrypt(encrypted, mf_key);
        let dec_btn = ((decrypted >> 28) & 0x0F) as u8;
        let dec_serial_lsb = ((decrypted >> 16) & 0xFF) as u8;

        // Validate decryption (may fail if key is wrong)
        let crc_valid = if mf_key != 0 {
            dec_btn == button && dec_serial_lsb == our_serial_lsb
        } else {
            // Can't validate without key
            true
        };

        let counter = (decrypted & 0xFFFF) as u16;

        // Build key data
        let key_data = ((b[0] as u64) << 56) |
                      ((b[1] as u64) << 48) |
                      ((b[2] as u64) << 40) |
                      ((b[3] as u64) << 32) |
                      ((b[4] as u64) << 24) |
                      ((b[5] as u64) << 16) |
                      ((b[6] as u64) << 8) |
                      (b[7] as u64);

        Some(DecodedSignal {
            serial: Some(serial),
            button: Some(button),
            counter: Some(counter),
            crc_valid,
            data: key_data,
            data_count_bit: MIN_COUNT_BIT,
            encoder_capable: true,
        })
    }
}

impl ProtocolDecoder for KiaV3V4Decoder {
    fn name(&self) -> &'static str {
        "Kia V3/V4"
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
        self.raw_bits = [0; 32];
        self.raw_bit_count = 0;
        self.is_v3_sync = false;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let is_short = duration_diff!(duration, TE_SHORT) < TE_DELTA;
        let is_long = duration_diff!(duration, TE_LONG) < TE_DELTA;
        let is_sync = duration > 1000 && duration < 1500;
        let is_very_long = duration > 1500;

        match self.step {
            DecoderStep::Reset => {
                if level && is_short {
                    self.step = DecoderStep::CheckPreamble;
                    self.te_last = duration;
                    self.header_count = 1;
                }
            }

            DecoderStep::CheckPreamble => {
                if level {
                    if is_short {
                        self.te_last = duration;
                    } else if is_sync && self.header_count >= 8 {
                        // V4 sync: long HIGH
                        self.step = DecoderStep::CollectRawBits;
                        self.raw_bit_count = 0;
                        self.is_v3_sync = false;
                        self.raw_bits = [0; 32];
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else {
                    if is_sync && self.header_count >= 8 {
                        // V3 sync: long LOW
                        self.step = DecoderStep::CollectRawBits;
                        self.raw_bit_count = 0;
                        self.is_v3_sync = true;
                        self.raw_bits = [0; 32];
                    } else if is_short && duration_diff!(self.te_last, TE_SHORT) < TE_DELTA {
                        self.header_count += 1;
                    } else if is_very_long {
                        self.step = DecoderStep::Reset;
                    }
                }
            }

            DecoderStep::CollectRawBits => {
                if level {
                    if is_sync || is_very_long {
                        // End of data
                        let result = self.process_buffer();
                        self.step = DecoderStep::Reset;
                        return result;
                    } else if is_short {
                        self.add_raw_bit(false);
                    } else if is_long {
                        self.add_raw_bit(true);
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else {
                    if is_sync || is_very_long {
                        let result = self.process_buffer();
                        self.step = DecoderStep::Reset;
                        return result;
                    }
                    // LOW durations don't carry data in PWM
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

        // Build plaintext for encryption
        let plaintext = (counter as u32) |
                       ((serial & 0xFF) << 16) |
                       (0x1 << 24) |
                       (((button & 0x0F) as u32) << 28);

        let mf_key = Self::get_mf_key();
        let encrypted = keeloq_encrypt(plaintext, mf_key);

        // Build raw bytes
        let mut raw_bytes = [0u8; 9];
        raw_bytes[0] = Self::reverse8((encrypted >> 0) as u8);
        raw_bytes[1] = Self::reverse8((encrypted >> 8) as u8);
        raw_bytes[2] = Self::reverse8((encrypted >> 16) as u8);
        raw_bytes[3] = Self::reverse8((encrypted >> 24) as u8);

        let serial_btn = (serial & 0x0FFFFFFF) | (((button & 0x0F) as u32) << 28);
        raw_bytes[4] = Self::reverse8((serial_btn >> 0) as u8);
        raw_bytes[5] = Self::reverse8((serial_btn >> 8) as u8);
        raw_bytes[6] = Self::reverse8((serial_btn >> 16) as u8);
        raw_bytes[7] = Self::reverse8((serial_btn >> 24) as u8);

        let crc = Self::calculate_crc(&raw_bytes);
        raw_bytes[8] = crc << 4;

        // Use V4 encoding by default
        let version = 0;

        if version == 1 {
            // V3: invert data
            for byte in raw_bytes.iter_mut() {
                *byte = !*byte;
            }
        }

        let mut signal = Vec::with_capacity(600);

        // 3 bursts with 10ms gap (matches protopirate kia_v3_v4 encode)
        for burst in 0..TOTAL_BURSTS {
            if burst > 0 {
                signal.push(LevelDuration::new(false, INTER_BURST_GAP_US));
            }

            // Preamble: 16 short pairs
            for _ in 0..PREAMBLE_PAIRS {
                signal.push(LevelDuration::new(true, TE_SHORT));
                signal.push(LevelDuration::new(false, TE_SHORT));
            }

            // Sync: V4 = long HIGH, V3 = long LOW
            if version == 0 {
                // V4: long HIGH, short LOW
                signal.push(LevelDuration::new(true, SYNC_DURATION));
                signal.push(LevelDuration::new(false, TE_SHORT));
            } else {
                // V3: short HIGH, long LOW
                signal.push(LevelDuration::new(true, TE_SHORT));
                signal.push(LevelDuration::new(false, SYNC_DURATION));
            }

            // Data: 68 bits PWM (8 bytes + 4-bit CRC), MSB first per byte
            for byte_idx in 0..9 {
                let bits_in_byte = if byte_idx == 8 { 4 } else { 8 };
                for bit_idx in (8 - bits_in_byte..8).rev() {
                    let bit = (raw_bytes[byte_idx] >> bit_idx) & 1 != 0;
                    if bit {
                        signal.push(LevelDuration::new(true, TE_LONG));
                        signal.push(LevelDuration::new(false, TE_SHORT));
                    } else {
                        signal.push(LevelDuration::new(true, TE_SHORT));
                        signal.push(LevelDuration::new(false, TE_LONG));
                    }
                }
            }
        }

        Some(signal)
    }
}

impl Default for KiaV3V4Decoder {
    fn default() -> Self {
        Self::new()
    }
}
