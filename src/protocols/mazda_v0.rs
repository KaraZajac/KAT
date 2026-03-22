//! Mazda V0 protocol decoder
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/mazda_v0.c`.
//! Original implementation by @lupettohf.
//!
//! Protocol characteristics:
//! - 433.92 MHz FM, decode-only (no encoder)
//! - Pair-based decoding: feed() ignores level, processes raw durations in pairs
//! - Preamble: minimum 13 short/short pairs before data starts
//! - Data uses a 14-byte buffer with inverted bit polarity
//! - XOR deobfuscation with parity-based mask selection + bit interleave swap
//! - Additive checksum over first 7 bytes must equal byte 8
//! - Field layout: serial (32-bit), button (8-bit), counter (16-bit)
//! - Button codes: 0x10=Lock, 0x20=Unlock, 0x40=Trunk

use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

const TE_SHORT: u32 = 250;
const TE_LONG: u32 = 500;
const TE_DELTA: u32 = 100;
const MIN_COUNT_BIT: usize = 64;

const PREAMBLE_MIN: u16 = 13;
const COMPLETION_MIN: u16 = 80;
const COMPLETION_MAX: u16 = 105;
const DATA_BUFFER_SIZE: usize = 14;

/// Decoder states (matches mazda_v0.c MazdaDecoderStep)
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    PreambleSave,
    PreambleCheck,
    DataSave,
    DataCheck,
}

/// Mazda V0 protocol decoder
pub struct MazdaV0Decoder {
    step: DecoderStep,
    te_last: u32,
    preamble_count: u16,
    bit_counter: u16,
    prev_state: u8,
    data_buffer: [u8; DATA_BUFFER_SIZE],
}

impl MazdaV0Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            te_last: 0,
            preamble_count: 0,
            bit_counter: 0,
            prev_state: 0,
            data_buffer: [0u8; DATA_BUFFER_SIZE],
        }
    }

    #[inline]
    fn is_short(duration: u32) -> bool {
        duration_diff!(duration, TE_SHORT) < TE_DELTA
    }

    #[inline]
    fn is_long(duration: u32) -> bool {
        duration_diff!(duration, TE_LONG) < TE_DELTA
    }

    /// Collect a single bit into the data buffer.
    /// Inverted polarity: state_bit == 0 means stored bit is 1.
    fn collect_bit(&mut self, state_bit: u8) {
        let byte_idx = (self.bit_counter >> 3) as usize;
        if byte_idx < DATA_BUFFER_SIZE {
            self.data_buffer[byte_idx] <<= 1;
            if state_bit == 0 {
                self.data_buffer[byte_idx] |= 1;
            }
        }
        self.bit_counter += 1;
    }

    /// Process a duration pair and collect bits.
    /// Returns true if the pair was valid, false otherwise.
    fn process_pair(&mut self, dur_first: u32, dur_second: u32) -> bool {
        let first_short = Self::is_short(dur_first);
        let first_long = Self::is_long(dur_first);
        let second_short = Self::is_short(dur_second);
        let second_long = Self::is_long(dur_second);

        if first_long && second_short {
            self.collect_bit(0);
            self.collect_bit(1);
            self.prev_state = 1;
            return true;
        }

        if first_short && second_long {
            self.collect_bit(1);
            self.prev_state = 0;
            return true;
        }

        if first_short && second_short {
            let ps = self.prev_state;
            self.collect_bit(ps);
            return true;
        }

        if first_long && second_long {
            self.collect_bit(0);
            self.collect_bit(1);
            self.prev_state = 0;
            return true;
        }

        false
    }

    /// Check whether enough bits have been collected and validate the frame.
    /// On success, returns a DecodedSignal.
    fn check_completion(&self) -> Option<DecodedSignal> {
        if self.bit_counter < COMPLETION_MIN || self.bit_counter > COMPLETION_MAX {
            return None;
        }

        // Shift buffer by 1 byte (discard sync/header byte)
        let mut data = [0u8; 8];
        for i in 0..8 {
            data[i] = self.data_buffer[i + 1];
        }

        // XOR deobfuscation
        Self::xor_deobfuscate(&mut data);

        // Checksum: sum of data[0..7] must equal data[7]
        let mut checksum: u8 = 0;
        for i in 0..7 {
            checksum = checksum.wrapping_add(data[i]);
        }
        if checksum != data[7] {
            return None;
        }

        // Pack into u64
        let mut packed: u64 = 0;
        for i in 0..8 {
            packed = (packed << 8) | (data[i] as u64);
        }

        // Parse fields
        let serial = (packed >> 32) as u32;
        let btn = ((packed >> 24) & 0xFF) as u8;
        let cnt = ((packed >> 8) & 0xFFFF) as u16;

        Some(DecodedSignal {
            serial: Some(serial),
            button: Some(btn),
            counter: Some(cnt),
            crc_valid: true,
            data: packed,
            data_count_bit: MIN_COUNT_BIT,
            encoder_capable: false,
            extra: None,
            protocol_display_name: None,
        })
    }

    /// Byte parity: XOR-fold to single bit (matches mazda_byte_parity in C)
    fn byte_parity(mut value: u8) -> u8 {
        value ^= value >> 4;
        value ^= value >> 2;
        value ^= value >> 1;
        value & 1
    }

    /// XOR deobfuscation with parity-based mask selection + bit interleave swap
    /// (matches mazda_xor_deobfuscate in C)
    fn xor_deobfuscate(data: &mut [u8; 8]) {
        let parity = Self::byte_parity(data[7]);

        if parity != 0 {
            let mask = data[6];
            for i in 0..6 {
                data[i] ^= mask;
            }
        } else {
            let mask = data[5];
            for i in 0..5 {
                data[i] ^= mask;
            }
            data[6] ^= mask;
        }

        // Bit interleave swap between bytes 5 and 6
        let old5 = data[5];
        let old6 = data[6];
        data[5] = (old5 & 0xAA) | (old6 & 0x55);
        data[6] = (old5 & 0x55) | (old6 & 0xAA);
    }

    /// Get button name for display
    #[allow(dead_code)]
    fn get_button_name(btn: u8) -> &'static str {
        match btn {
            0x10 => "Lock",
            0x20 => "Unlock",
            0x40 => "Trunk",
            _ => "Unknown",
        }
    }
}

impl ProtocolDecoder for MazdaV0Decoder {
    fn name(&self) -> &'static str {
        "Mazda V0"
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
        self.bit_counter = 0;
        self.prev_state = 0;
        self.te_last = 0;
        self.data_buffer = [0u8; DATA_BUFFER_SIZE];
    }

    fn feed(&mut self, _level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::Reset => {
                if Self::is_short(duration) {
                    self.te_last = duration;
                    self.preamble_count = 0;
                    self.step = DecoderStep::PreambleCheck;
                }
            }

            DecoderStep::PreambleSave => {
                self.te_last = duration;
                self.step = DecoderStep::PreambleCheck;
            }

            DecoderStep::PreambleCheck => {
                if Self::is_short(self.te_last) && Self::is_short(duration) {
                    self.preamble_count += 1;
                    self.step = DecoderStep::PreambleSave;
                } else if Self::is_short(self.te_last)
                    && Self::is_long(duration)
                    && self.preamble_count >= PREAMBLE_MIN
                {
                    // Transition from preamble to data
                    self.bit_counter = 1;
                    self.data_buffer = [0u8; DATA_BUFFER_SIZE];
                    self.collect_bit(1);
                    self.prev_state = 0;
                    self.step = DecoderStep::DataSave;
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::DataSave => {
                self.te_last = duration;
                self.step = DecoderStep::DataCheck;
            }

            DecoderStep::DataCheck => {
                if self.process_pair(self.te_last, duration) {
                    self.step = DecoderStep::DataSave;
                } else {
                    // Pair was invalid - check if we have a complete frame
                    let result = self.check_completion();
                    self.step = DecoderStep::Reset;
                    if result.is_some() {
                        return result;
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

impl Default for MazdaV0Decoder {
    fn default() -> Self {
        Self::new()
    }
}
