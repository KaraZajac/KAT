//! Mitsubishi V0 protocol decoder
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/mitsubishi_v0.c`.
//! Original implementation by @lupettohf.
//!
//! Protocol characteristics:
//! - PWM encoding: Short HIGH + Long LOW = bit 1, Long HIGH + Short LOW = bit 0
//! - 96-bit frame (12 bytes), collected MSB-first into byte buffer
//! - Level-aware state machine: HIGH pulses are saved, LOW pulses complete the pair
//! - Unscramble: NOT first 8 bytes, extract counter from bytes[4..5], compute XOR mask, apply to bytes[0..5]
//! - Field layout: serial = bytes[0..3] (32-bit), counter = bytes[4..5] (16-bit), button = byte[6]
//! - 868 MHz, FM modulation, decode-only (no encoder)

use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

const TE_SHORT: u32 = 250;
const TE_LONG: u32 = 500;
const TE_DELTA: u32 = 100;
#[allow(dead_code)]
const MIN_COUNT_BIT: usize = 80;
const BIT_COUNT: usize = 96;
const DATA_BYTES: usize = 12;

/// Decoder states (matches mitsubishi_v0.c MitsubishiDecoderStep)
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    DataSave,
    DataCheck,
}

/// Mitsubishi V0 protocol decoder
pub struct MitsubishiV0Decoder {
    step: DecoderStep,
    te_last: u32,
    bit_count: usize,
    decode_data: [u8; DATA_BYTES],
}

impl MitsubishiV0Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            te_last: 0,
            bit_count: 0,
            decode_data: [0u8; DATA_BYTES],
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

    fn reset_payload(&mut self) {
        self.bit_count = 0;
        self.decode_data = [0u8; DATA_BYTES];
    }

    /// Collect a HIGH+LOW pair and decode one bit.
    /// Short HIGH + Long LOW = bit 1; Long HIGH + Short LOW = bit 0.
    /// Bits are stored MSB-first into the byte buffer.
    fn collect_pair(&mut self, high: u32, low: u32) -> bool {
        let bit_value;

        if Self::is_short(high) && Self::is_long(low) {
            bit_value = true;
        } else if Self::is_long(high) && Self::is_short(low) {
            bit_value = false;
        } else {
            return false;
        }

        let bit_index = self.bit_count;
        if bit_index < BIT_COUNT {
            if bit_value {
                let byte_index = bit_index >> 3;
                let bit_position = 7 - (bit_index & 0x07);
                self.decode_data[byte_index] |= 1u8 << bit_position;
            }
            self.bit_count += 1;
        }

        true
    }

    /// Unscramble the payload (matches mitsubishi_unscramble_payload in C reference).
    /// 1. Bitwise NOT first 8 bytes
    /// 2. Extract counter from bytes[4..5]
    /// 3. Compute masks from counter and XOR bytes[0..5]
    fn unscramble_payload(payload: &mut [u8; DATA_BYTES]) {
        // Step 1: NOT first 8 bytes
        for i in 0..8 {
            payload[i] = !payload[i];
        }

        // Step 2: Extract counter
        let counter = ((payload[4] as u16) << 8) | (payload[5] as u16);
        let hi = ((counter >> 8) & 0xFF) as u8;
        let lo = (counter & 0xFF) as u8;

        // Step 3: Compute masks
        let mask1 = (hi & 0xAA) | (lo & 0x55);
        let mask2 = (lo & 0xAA) | (hi & 0x55);
        let mask3 = mask1 ^ mask2;

        // Step 4: XOR bytes[0..5] with mask3
        for i in 0..5 {
            payload[i] ^= mask3;
        }
    }

    /// Parse the unscrambled payload into a DecodedSignal.
    fn publish_frame(&self) -> DecodedSignal {
        let mut payload = self.decode_data;
        Self::unscramble_payload(&mut payload);

        let serial = ((payload[0] as u32) << 24)
            | ((payload[1] as u32) << 16)
            | ((payload[2] as u32) << 8)
            | (payload[3] as u32);

        let counter = ((payload[4] as u16) << 8) | (payload[5] as u16);
        let button = payload[6];

        // Store first 8 bytes (post-unscramble) as u64 data field
        let data = ((payload[0] as u64) << 56)
            | ((payload[1] as u64) << 48)
            | ((payload[2] as u64) << 40)
            | ((payload[3] as u64) << 32)
            | ((payload[4] as u64) << 24)
            | ((payload[5] as u64) << 16)
            | ((payload[6] as u64) << 8)
            | (payload[7] as u64);

        DecodedSignal {
            serial: Some(serial),
            button: Some(button),
            counter: Some(counter),
            crc_valid: true,
            data,
            data_count_bit: BIT_COUNT,
            encoder_capable: false,
            extra: None,
            protocol_display_name: None,
        }
    }
}

impl ProtocolDecoder for MitsubishiV0Decoder {
    fn name(&self) -> &'static str {
        "Mitsubishi V0"
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
        &[868_350_000]
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.te_last = 0;
        self.reset_payload();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::Reset => {
                // Wait for a HIGH pulse to start
                if level {
                    self.te_last = duration;
                    self.step = DecoderStep::DataCheck;
                }
            }

            DecoderStep::DataSave => {
                if level {
                    // HIGH pulse: save duration and move to DataCheck
                    self.te_last = duration;
                    self.step = DecoderStep::DataCheck;
                } else {
                    // LOW pulse without preceding HIGH data check: reset
                    self.step = DecoderStep::Reset;
                    self.reset_payload();
                }
            }

            DecoderStep::DataCheck => {
                if !level {
                    // LOW pulse: complete the HIGH+LOW pair
                    if self.collect_pair(self.te_last, duration) {
                        if self.bit_count >= BIT_COUNT {
                            // Full frame received
                            let result = self.publish_frame();
                            self.reset_payload();
                            self.step = DecoderStep::Reset;
                            return Some(result);
                        } else {
                            self.step = DecoderStep::DataSave;
                        }
                    } else {
                        // Invalid pair: reset
                        self.reset_payload();
                        self.step = DecoderStep::Reset;
                    }
                } else {
                    // Another HIGH pulse while expecting LOW: update te_last
                    self.te_last = duration;
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

impl Default for MitsubishiV0Decoder {
    fn default() -> Self {
        Self::new()
    }
}
