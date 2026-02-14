//! PSA (Peugeot/Citroën) protocol decoder/encoder
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/psa.c`.
//! Decode/encode logic (Manchester, preamble, TEA, XOR, mode 0x23/0x36) matches reference.
//!
//! Protocol characteristics:
//! - Manchester encoding: 250/500µs symbol (125/250µs sub-symbol for preamble)
//! - 128 bits total: key1 (64) + validation (16) + key2/rest (48); decode uses key1 + 16-bit validation
//! - TEA decrypt/encrypt with fixed key schedules; mode 0x23 adds XOR layer
//! - Two modes: seed 0x23 (TEA + XOR), seed 0xF3/0x36 (TEA, BF2 key schedule)

use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

const TE_SHORT: u32 = 250;
const TE_LONG: u32 = 500;
const TE_DELTA: u32 = 100;
const MIN_COUNT_BIT: usize = 128;

// Internal timing for Manchester sub-symbol detection
const TE_SHORT_125: u32 = 125;
const TE_LONG_250: u32 = 250;
const TE_TOLERANCE_49: u32 = 49;
const TE_TOLERANCE_50: u32 = 50;
const TE_TOLERANCE_99: u32 = 99;
const TE_END_1000: u32 = 1000;

// TEA constants
const TEA_DELTA: u32 = 0x9E3779B9;
const TEA_ROUNDS: u32 = 32;

// Brute-force constants for mode 0x23
const BF1_KEY_SCHEDULE: [u32; 4] = [0x4A434915, 0xD6743C2B, 0x1F29D308, 0xE6B79A64];

// Brute-force constants for mode 0x36
const BF2_KEY_SCHEDULE: [u32; 4] = [0x4039C240, 0xEDA92CAB, 0x4306C02A, 0x02192A04];

/// Manchester decoder states (matches protopirate psa.c Manchester state machine)
#[derive(Debug, Clone, Copy, PartialEq)]
enum ManchesterState {
    Mid0,
    Mid1,
    Start0,
    Start1,
}

/// Decoder states (matches protopirate's PsaDecoderState)
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderState {
    WaitEdge,
    CountPattern,
    DecodeManchester,
    End,
}

/// PSA protocol decoder
pub struct PsaDecoder {
    state: DecoderState,
    prev_duration: u32,
    manchester_state: ManchesterState,
    pattern_counter: u16,
    data_low: u32,
    data_high: u32,
    bit_count: u8,
    // Decoded fields
    key1_low: u32,
    key1_high: u32,
    validation_field: u16,
    key2_low: u32,
    key2_high: u32,
    seed: u32,
}

impl PsaDecoder {
    pub fn new() -> Self {
        Self {
            state: DecoderState::WaitEdge,
            prev_duration: 0,
            manchester_state: ManchesterState::Mid1,
            pattern_counter: 0,
            data_low: 0,
            data_high: 0,
            bit_count: 0,
            key1_low: 0,
            key1_high: 0,
            validation_field: 0,
            key2_low: 0,
            key2_high: 0,
            seed: 0,
        }
    }

    /// Manchester state machine (matches psa.c event mapping)
    fn manchester_advance(&mut self, is_short: bool, is_high: bool) -> Option<bool> {
        let event = match (is_short, is_high) {
            (true, true) => 0,
            (true, false) => 1,
            (false, true) => 2,
            (false, false) => 3,
        };

        let (new_state, output) = match (self.manchester_state, event) {
            (ManchesterState::Mid0, 0) | (ManchesterState::Mid1, 0) => {
                (ManchesterState::Start1, None)
            }
            (ManchesterState::Mid0, 1) | (ManchesterState::Mid1, 1) => {
                (ManchesterState::Start0, None)
            }
            (ManchesterState::Start1, 1) => (ManchesterState::Mid1, Some(true)),
            (ManchesterState::Start1, 3) => (ManchesterState::Start0, Some(true)),
            (ManchesterState::Start0, 0) => (ManchesterState::Mid0, Some(false)),
            (ManchesterState::Start0, 2) => (ManchesterState::Start1, Some(false)),
            _ => (ManchesterState::Mid1, None),
        };

        self.manchester_state = new_state;
        output
    }

    fn add_bit(&mut self, bit: bool) {
        let new_bit = if bit { 1u32 } else { 0u32 };
        let carry = (self.data_low >> 31) & 1;
        self.data_low = (self.data_low << 1) | new_bit;
        self.data_high = (self.data_high << 1) | carry;
        self.bit_count += 1;

        // Extract key1 at 64 bits
        if self.bit_count == 64 {
            self.key1_low = self.data_low;
            self.key1_high = self.data_high;
            self.data_low = 0;
            self.data_high = 0;
        }
        // Extract validation at 80 bits (16 more)
        else if self.bit_count == 80 {
            self.validation_field = self.data_low as u16;
            self.data_low = 0;
            self.data_high = 0;
        }
    }

    /// TEA decrypt (matches psa.c / standard TEA)
    fn tea_decrypt(v0: &mut u32, v1: &mut u32, key: &[u32; 4]) {
        let mut sum = TEA_DELTA.wrapping_mul(TEA_ROUNDS);
        for _ in 0..TEA_ROUNDS {
            *v1 = v1.wrapping_sub(
                (v0.wrapping_shl(4).wrapping_add(key[2]))
                    ^ (v0.wrapping_add(sum))
                    ^ (v0.wrapping_shr(5).wrapping_add(key[3])),
            );
            *v0 = v0.wrapping_sub(
                (v1.wrapping_shl(4).wrapping_add(key[0]))
                    ^ (v1.wrapping_add(sum))
                    ^ (v1.wrapping_shr(5).wrapping_add(key[1])),
            );
            sum = sum.wrapping_sub(TEA_DELTA);
        }
    }

    /// TEA encrypt (matches psa.c / standard TEA)
    fn tea_encrypt(v0: &mut u32, v1: &mut u32, key: &[u32; 4]) {
        let mut sum: u32 = 0;
        for _ in 0..TEA_ROUNDS {
            sum = sum.wrapping_add(TEA_DELTA);
            *v0 = v0.wrapping_add(
                (v1.wrapping_shl(4).wrapping_add(key[0]))
                    ^ (v1.wrapping_add(sum))
                    ^ (v1.wrapping_shr(5).wrapping_add(key[1])),
            );
            *v1 = v1.wrapping_add(
                (v0.wrapping_shl(4).wrapping_add(key[2]))
                    ^ (v0.wrapping_add(sum))
                    ^ (v0.wrapping_shr(5).wrapping_add(key[3])),
            );
        }
    }

    /// XOR decrypt for mode 0x23 (matches psa.c)
    fn xor_decrypt(buffer: &mut [u8]) {
        let e6 = buffer[8];
        let e7 = buffer[9];
        let e5 = buffer[7];
        let e0 = buffer[2];
        let e1 = buffer[3];
        let e2 = buffer[4];
        let e3 = buffer[5];
        let e4 = buffer[6];

        buffer[2] = e0 ^ e5;
        buffer[3] = e1 ^ (e0 ^ e5 ^ e6 ^ e7);
        buffer[4] = e2 ^ e0;
        buffer[5] = e3 ^ (e0 ^ e5 ^ e6 ^ e7);
        buffer[6] = e4 ^ e2;
        buffer[7] = e5 ^ e6 ^ e7;
    }

    /// Decrypt key1 + validation: mode 0x23 (TEA+XOR) or 0x36 (TEA, BF2) — matches psa.c
    fn try_decrypt(&self) -> Option<(u32, u8, u32, u16, u8)> {
        // Try mode 0x23 first (seed byte 0x23)
        let seed_byte = (self.key1_high >> 24) as u8;

        if seed_byte >= 0x23 && seed_byte < 0x24 {
            // Mode 0x23 - TEA + XOR
            let mut v0 = self.key1_high;
            let mut v1 = self.key1_low;
            Self::tea_decrypt(&mut v0, &mut v1, &BF1_KEY_SCHEDULE);

            let mut buffer = [0u8; 10];
            buffer[0] = (v0 >> 24) as u8;
            buffer[1] = (v0 >> 16) as u8;
            buffer[2] = (v0 >> 8) as u8;
            buffer[3] = (v0 >> 0) as u8;
            buffer[4] = (v1 >> 24) as u8;
            buffer[5] = (v1 >> 16) as u8;
            buffer[6] = (v1 >> 8) as u8;
            buffer[7] = (v1 >> 0) as u8;
            buffer[8] = (self.validation_field >> 8) as u8;
            buffer[9] = (self.validation_field & 0xFF) as u8;

            Self::xor_decrypt(&mut buffer);

            let serial = ((buffer[2] as u32) << 16)
                | ((buffer[3] as u32) << 8)
                | (buffer[4] as u32);
            let counter = ((buffer[5] as u32) << 8) | (buffer[6] as u32);
            let crc = buffer[7] as u16;
            let btn = buffer[8] & 0x0F;

            return Some((serial, btn, counter, crc, 0x23));
        }

        if seed_byte >= 0xF3 && seed_byte < 0xF4 {
            // Mode 0x36 - TEA + different key schedule
            let mut v0 = self.key1_high;
            let mut v1 = self.key1_low;
            Self::tea_decrypt(&mut v0, &mut v1, &BF2_KEY_SCHEDULE);

            let serial = ((v0 >> 8) & 0xFFFF00) | ((v0 & 0xFF) as u32);
            let counter = v1 >> 16;
            let btn = ((v1 >> 8) & 0xF) as u8;
            let crc = (v1 & 0xFF) as u16;

            return Some((serial, btn, counter, crc, 0x36));
        }

        // Cannot decrypt - return raw data
        None
    }

    /// Build DecodedSignal from key1 + validation; decrypt yields serial/button/counter (matches psa.c)
    fn parse_data(&self) -> DecodedSignal {
        // Store key1 as 64-bit data for display/replay
        let data = ((self.key1_high as u64) << 32) | (self.key1_low as u64);

        if let Some((serial, btn, counter, _crc, _mode)) = self.try_decrypt() {
            DecodedSignal {
                serial: Some(serial),
                button: Some(btn),
                counter: Some(counter as u16),
                crc_valid: true,
                data,
                data_count_bit: MIN_COUNT_BIT,
                encoder_capable: true,
                extra: None,
            }
        } else {
            DecodedSignal {
                serial: None,
                button: None,
                counter: None,
                crc_valid: false,
                data,
                data_count_bit: MIN_COUNT_BIT,
                encoder_capable: false,
                extra: None,
            }
        }
    }
}

impl ProtocolDecoder for PsaDecoder {
    fn name(&self) -> &'static str {
        "PSA"
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
        self.state = DecoderState::WaitEdge;
        self.prev_duration = 0;
        self.manchester_state = ManchesterState::Mid1;
        self.pattern_counter = 0;
        self.data_low = 0;
        self.data_high = 0;
        self.bit_count = 0;
        self.key1_low = 0;
        self.key1_high = 0;
        self.validation_field = 0;
        self.key2_low = 0;
        self.key2_high = 0;
        self.seed = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.state {
            DecoderState::WaitEdge => {
                if level && duration_diff!(duration, TE_SHORT_125) < TE_TOLERANCE_49 {
                    self.state = DecoderState::CountPattern;
                    self.prev_duration = duration;
                    self.pattern_counter = 0;
                }
            }

            DecoderState::CountPattern => {
                let diff_125 = duration_diff!(duration, TE_SHORT_125);
                let diff_250 = duration_diff!(duration, TE_LONG_250);

                if diff_125 < TE_TOLERANCE_50 {
                    self.pattern_counter += 1;
                    self.prev_duration = duration;
                } else if diff_250 < TE_TOLERANCE_99 && self.pattern_counter >= 0x46 {
                    // Found end of preamble, start Manchester decoding
                    self.state = DecoderState::DecodeManchester;
                    self.data_low = 0;
                    self.data_high = 0;
                    self.bit_count = 0;
                    self.manchester_state = ManchesterState::Mid1;
                    self.prev_duration = duration;
                } else if self.pattern_counter < 2 {
                    self.state = DecoderState::WaitEdge;
                } else {
                    self.prev_duration = duration;
                }
            }

            DecoderState::DecodeManchester => {
                let is_short = duration_diff!(duration, TE_SHORT) < TE_DELTA;
                let is_long = duration_diff!(duration, TE_LONG) < TE_DELTA;
                let is_end = duration > TE_END_1000;

                if is_end || self.bit_count >= 121 {
                    // End of data
                    self.state = DecoderState::End;

                    if self.bit_count >= 96 {
                        // Got enough data
                        let result = self.parse_data();
                        self.state = DecoderState::WaitEdge;
                        return Some(result);
                    }
                    self.state = DecoderState::WaitEdge;
                    return None;
                }

                if is_short || is_long {
                    if let Some(bit) = self.manchester_advance(is_short, level) {
                        self.add_bit(bit);
                    }
                } else {
                    self.state = DecoderState::WaitEdge;
                }

                self.prev_duration = duration;
            }

            DecoderState::End => {
                self.state = DecoderState::WaitEdge;
            }
        }

        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        let serial = decoded.serial?;
        let counter = decoded.counter.unwrap_or(0).wrapping_add(1) as u32;

        // Build plaintext buffer for mode 0x23
        let mut buffer = [0u8; 10];
        buffer[0] = 0x23;
        buffer[1] = 0x00;
        buffer[2] = (serial >> 16) as u8;
        buffer[3] = (serial >> 8) as u8;
        buffer[4] = serial as u8;
        buffer[5] = (counter >> 8) as u8;
        buffer[6] = counter as u8;
        buffer[7] = 0; // CRC placeholder
        buffer[8] = button & 0x0F;
        buffer[9] = 0;

        // XOR encrypt
        {
            let e6 = buffer[8];
            let e7 = buffer[9];
            let p0 = buffer[2];
            let p1 = buffer[3];
            let p2 = buffer[4];
            let p3 = buffer[5];
            let p4 = buffer[6];
            let p5 = buffer[7];

            let ne5 = p5 ^ e7 ^ e6;
            let ne0 = p2 ^ ne5;
            let ne2 = p4 ^ ne0;
            let ne4 = p3 ^ ne2;
            let ne3 = p0 ^ ne5;
            let ne1 = p1 ^ ne3;

            buffer[2] = ne0;
            buffer[3] = ne1;
            buffer[4] = ne2;
            buffer[5] = ne3;
            buffer[6] = ne4;
            buffer[7] = ne5;
        }

        // TEA encrypt
        let mut v0 = ((buffer[0] as u32) << 24)
            | ((buffer[1] as u32) << 16)
            | ((buffer[2] as u32) << 8)
            | (buffer[3] as u32);
        let mut v1 = ((buffer[4] as u32) << 24)
            | ((buffer[5] as u32) << 16)
            | ((buffer[6] as u32) << 8)
            | (buffer[7] as u32);

        Self::tea_encrypt(&mut v0, &mut v1, &BF1_KEY_SCHEDULE);

        let key1_high = v0;
        let key1_low = v1;
        let validation = ((buffer[8] as u16) << 8) | (buffer[9] as u16);

        let mut signal = Vec::with_capacity(512);

        // Preamble + sync (matches protopirate psa encode)
        for _ in 0..70 {
            signal.push(LevelDuration::new(true, TE_SHORT_125));
            signal.push(LevelDuration::new(false, TE_SHORT_125));
        }
        signal.push(LevelDuration::new(true, TE_LONG_250));
        signal.push(LevelDuration::new(false, TE_LONG_250));

        // Key1: 64 bits Manchester, then validation 16 bits
        let key1 = ((key1_high as u64) << 32) | (key1_low as u64);
        for bit in (0..64).rev() {
            if (key1 >> bit) & 1 == 1 {
                signal.push(LevelDuration::new(false, TE_SHORT));
                signal.push(LevelDuration::new(true, TE_SHORT));
            } else {
                signal.push(LevelDuration::new(true, TE_SHORT));
                signal.push(LevelDuration::new(false, TE_SHORT));
            }
        }

        // Validation: 16 bits Manchester encoded
        for bit in (0..16).rev() {
            if (validation >> bit) & 1 == 1 {
                signal.push(LevelDuration::new(false, TE_SHORT));
                signal.push(LevelDuration::new(true, TE_SHORT));
            } else {
                signal.push(LevelDuration::new(true, TE_SHORT));
                signal.push(LevelDuration::new(false, TE_SHORT));
            }
        }

        // End marker
        signal.push(LevelDuration::new(false, TE_END_1000));

        Some(signal)
    }
}

impl Default for PsaDecoder {
    fn default() -> Self {
        Self::new()
    }
}
