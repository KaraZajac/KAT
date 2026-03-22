//! PSA (Peugeot/Citroën) protocol decoder/encoder
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/psa.c`.
//!
//! Protocol characteristics:
//! - Manchester encoding: 250/500µs symbol (125/250µs sub-symbol for preamble)
//! - 128 bits total: key1 (64) + validation (16) + key2/rest (48)
//! - Modified TEA (XTEA-like) with dynamic key selection (sum&3, (sum>>11)&3)
//! - Mode 0x23: direct XOR decrypt with checksum validation
//! - Mode 0x36: TEA brute-force with BF1/BF2 key schedules (deferred)
//! - Dual preamble: Pattern 1 (250µs) and Pattern 2 (125µs)

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

/// Decoder states (matches protopirate's PsaDecoderState 0-4)
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderState {
    /// State0: Wait for first edge (pattern detection)
    WaitEdge,
    /// State1: Count 250µs preamble patterns (Pattern 1)
    CountPattern250,
    /// State2: Manchester decode at 250/500µs (Pattern 1)
    DecodeManchester250,
    /// State3: Count 125µs preamble patterns (Pattern 2)
    CountPattern125,
    /// State4: Manchester decode at 125/250µs (Pattern 2)
    DecodeManchester125,
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

    /// Modified TEA decrypt with dynamic key selection (matches psa.c psa_tea_decrypt)
    /// Uses XTEA-like key scheduling: key[sum&3] and key[(sum>>11)&3]
    fn tea_decrypt(v0: &mut u32, v1: &mut u32, key: &[u32; 4]) {
        let mut sum = TEA_DELTA.wrapping_mul(TEA_ROUNDS);
        for _ in 0..TEA_ROUNDS {
            let k_idx2 = ((sum >> 11) & 3) as usize;
            let temp = key[k_idx2].wrapping_add(sum);
            sum = sum.wrapping_sub(TEA_DELTA);
            *v1 = v1.wrapping_sub(
                temp ^ (v0.wrapping_shr(5) ^ v0.wrapping_shl(4)).wrapping_add(*v0),
            );
            let k_idx1 = (sum & 3) as usize;
            let temp = key[k_idx1].wrapping_add(sum);
            *v0 = v0.wrapping_sub(
                temp ^ (v1.wrapping_shr(5) ^ v1.wrapping_shl(4)).wrapping_add(*v1),
            );
        }
    }

    /// Modified TEA encrypt with dynamic key selection (matches psa.c psa_tea_encrypt)
    fn tea_encrypt(v0: &mut u32, v1: &mut u32, key: &[u32; 4]) {
        let mut sum: u32 = 0;
        for _ in 0..TEA_ROUNDS {
            let k_idx1 = (sum & 3) as usize;
            let temp = key[k_idx1].wrapping_add(sum);
            sum = sum.wrapping_add(TEA_DELTA);
            *v0 = v0.wrapping_add(
                temp ^ (v1.wrapping_shr(5) ^ v1.wrapping_shl(4)).wrapping_add(*v1),
            );
            let k_idx2 = ((sum >> 11) & 3) as usize;
            let temp = key[k_idx2].wrapping_add(sum);
            *v1 = v1.wrapping_add(
                temp ^ (v0.wrapping_shr(5) ^ v0.wrapping_shl(4)).wrapping_add(*v0),
            );
        }
    }

    /// XOR decrypt for mode 0x23 (matches psa.c psa_second_stage_xor_decrypt)
    /// Uses psa_copy_reverse byte reordering then XOR operations
    fn xor_decrypt(buffer: &mut [u8]) {
        // psa_copy_reverse: reorder source bytes
        let temp = [
            buffer[5], // temp[0] = source[5]
            buffer[4], // temp[1] = source[4]
            buffer[3], // temp[2] = source[3]
            buffer[2], // temp[3] = source[2]
            buffer[9], // temp[4] = source[9]
            buffer[8], // temp[5] = source[8]
            buffer[7], // temp[6] = source[7]
            buffer[6], // temp[7] = source[6]
        ];
        buffer[2] = temp[0] ^ temp[6];
        buffer[3] = temp[2] ^ temp[0];
        buffer[4] = temp[6] ^ temp[3];
        buffer[5] = temp[7] ^ temp[1];
        buffer[6] = temp[3] ^ temp[1];
        buffer[7] = temp[6] ^ temp[4] ^ temp[5];
    }

    /// XOR encrypt for mode 0x23 encoding (matches psa.c psa_second_stage_xor_encrypt)
    fn xor_encrypt(buffer: &mut [u8]) {
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

    /// Calculate checksum over buffer[2..8] (matches psa.c psa_calculate_checksum)
    fn calculate_checksum(buffer: &[u8]) -> u8 {
        let mut checksum: u32 = 0;
        for i in 2..8 {
            checksum += (buffer[i] & 0xF) as u32 + ((buffer[i] >> 4) & 0xF) as u32;
        }
        ((checksum.wrapping_mul(0x10)) & 0xFF) as u8
    }

    /// Check if direct XOR is allowed by key2 high byte (matches psa.c)
    fn direct_xor_allowed_by_key2(key2_high_byte: u8) -> bool {
        let lo = key2_high_byte & 0xF;
        if lo < 3 { return true; }
        if lo < 7 && (key2_high_byte & 0xC) != 0 { return true; }
        false
    }

    /// Setup byte buffer from key1/key2 (matches psa.c psa_setup_byte_buffer)
    fn setup_byte_buffer(buffer: &mut [u8], key1_low: u32, key1_high: u32, key2_low: u32) {
        for i in 0..8usize {
            let shift = i * 8;
            let byte_val = if shift < 32 {
                ((key1_low >> shift) & 0xFF) as u8
            } else {
                ((key1_high >> (shift - 32)) & 0xFF) as u8
            };
            buffer[7 - i] = byte_val;
        }
        buffer[9] = (key2_low & 0xFF) as u8;
        buffer[8] = ((key2_low >> 8) & 0xFF) as u8;
    }

    /// Decrypt using the C code's approach: setup_byte_buffer then attempt direct XOR
    /// with checksum validation; mode 0x36 is marked for brute-force (matches psa.c)
    fn try_decrypt(&self) -> Option<(u32, u8, u32, u16, u8)> {
        // C: key2_low = decode_data_low (the 16-bit validation sits in the low word)
        let key2_low = self.validation_field as u32;

        let mut buffer = [0u8; 48];
        Self::setup_byte_buffer(&mut buffer, self.key1_low, self.key1_high, key2_low);

        let key2_high_byte = buffer[8];

        // Try direct XOR decrypt (mode 0x23) if allowed by key2 filter
        if Self::direct_xor_allowed_by_key2(key2_high_byte) {
            let checksum = Self::calculate_checksum(&buffer);
            let validation_result = (checksum ^ key2_high_byte) & 0xF0;

            if validation_result == 0 {
                // Direct XOR decrypt succeeded validation
                Self::xor_decrypt(&mut buffer);

                let serial = ((buffer[3] as u32) << 8)
                    | ((buffer[2] as u32) << 16)
                    | (buffer[4] as u32);
                let counter = (buffer[6] as u32) | ((buffer[5] as u32) << 8);
                let crc = buffer[7] as u16;
                let btn = buffer[8] & 0x0F;

                return Some((serial, btn, counter, crc, 0x23));
            }
        }

        // Mode 0x36 - TEA brute-force path
        // Try direct TEA decrypt with BF1 key schedule for a quick decode attempt
        {
            let mut w0 = ((buffer[3] as u32) << 16)
                | ((buffer[2] as u32) << 24)
                | ((buffer[4] as u32) << 8)
                | (buffer[5] as u32);
            let mut w1 = ((buffer[7] as u32) << 16)
                | ((buffer[6] as u32) << 24)
                | ((buffer[8] as u32) << 8)
                | (buffer[9] as u32);

            Self::tea_decrypt(&mut w0, &mut w1, &BF1_KEY_SCHEDULE);

            // Check if the TEA CRC validates (sum of bytes)
            let crc_calc = Self::calculate_tea_crc(w0, w1);
            if crc_calc == (w1 & 0xFF) as u8 {
                let mut dec_buffer = [0u8; 48];
                dec_buffer[2] = ((w0 >> 24) & 0xFF) as u8;
                dec_buffer[3] = ((w0 >> 16) & 0xFF) as u8;
                dec_buffer[4] = ((w0 >> 8) & 0xFF) as u8;
                dec_buffer[5] = (w0 & 0xFF) as u8;
                dec_buffer[6] = ((w1 >> 24) & 0xFF) as u8;
                dec_buffer[7] = ((w1 >> 16) & 0xFF) as u8;
                dec_buffer[8] = ((w1 >> 8) & 0xFF) as u8;
                dec_buffer[9] = (w1 & 0xFF) as u8;

                let btn = (dec_buffer[5] >> 4) & 0xF;
                let serial = ((dec_buffer[3] as u32) << 8)
                    | ((dec_buffer[2] as u32) << 16)
                    | (dec_buffer[4] as u32);
                let counter = ((dec_buffer[7] as u32) << 8)
                    | ((dec_buffer[6] as u32) << 16)
                    | (dec_buffer[8] as u32)
                    | (((dec_buffer[5] as u32) & 0xF) << 24);
                let crc = dec_buffer[9] as u16;

                return Some((serial, btn, counter, crc, 0x36));
            }
        }

        // Also try BF2 key schedule directly (XOR-derived keys)
        {
            let mut w0 = ((buffer[3] as u32) << 16)
                | ((buffer[2] as u32) << 24)
                | ((buffer[4] as u32) << 8)
                | (buffer[5] as u32);
            let mut w1 = ((buffer[7] as u32) << 16)
                | ((buffer[6] as u32) << 24)
                | ((buffer[8] as u32) << 8)
                | (buffer[9] as u32);

            Self::tea_decrypt(&mut w0, &mut w1, &BF2_KEY_SCHEDULE);

            let crc_calc = Self::calculate_tea_crc(w0, w1);
            if crc_calc == (w1 & 0xFF) as u8 {
                let mut dec_buffer = [0u8; 48];
                dec_buffer[2] = ((w0 >> 24) & 0xFF) as u8;
                dec_buffer[3] = ((w0 >> 16) & 0xFF) as u8;
                dec_buffer[4] = ((w0 >> 8) & 0xFF) as u8;
                dec_buffer[5] = (w0 & 0xFF) as u8;
                dec_buffer[6] = ((w1 >> 24) & 0xFF) as u8;
                dec_buffer[7] = ((w1 >> 16) & 0xFF) as u8;
                dec_buffer[8] = ((w1 >> 8) & 0xFF) as u8;
                dec_buffer[9] = (w1 & 0xFF) as u8;

                let btn = (dec_buffer[5] >> 4) & 0xF;
                let serial = ((dec_buffer[3] as u32) << 8)
                    | ((dec_buffer[2] as u32) << 16)
                    | (dec_buffer[4] as u32);
                let counter = ((dec_buffer[7] as u32) << 8)
                    | ((dec_buffer[6] as u32) << 16)
                    | (dec_buffer[8] as u32)
                    | (((dec_buffer[5] as u32) & 0xF) << 24);
                let crc = dec_buffer[9] as u16;

                return Some((serial, btn, counter, crc, 0x36));
            }
        }

        None
    }

    /// Calculate TEA CRC (matches psa.c psa_calculate_tea_crc)
    fn calculate_tea_crc(v0: u32, v1: u32) -> u8 {
        let mut crc: u32 = 0;
        crc += (v0 >> 24) & 0xFF;
        crc += (v0 >> 16) & 0xFF;
        crc += (v0 >> 8) & 0xFF;
        crc += v0 & 0xFF;
        crc += (v1 >> 24) & 0xFF;
        crc += (v1 >> 16) & 0xFF;
        crc += (v1 >> 8) & 0xFF;
        (crc & 0xFF) as u8
    }

    fn init_preamble_state(&mut self) {
        self.data_low = 0;
        self.data_high = 0;
        self.pattern_counter = 0;
        self.bit_count = 0;
        self.manchester_state = ManchesterState::Mid1;
    }

    fn finalize_frame(&mut self) -> Option<DecodedSignal> {
        self.state = DecoderState::WaitEdge;
        if self.bit_count >= 80 {
            // C validation: ((key1_high >> 16) & 0xF) == 0xA
            if ((self.key1_high >> 16) & 0xF) != 0xA {
                return None;
            }
            let result = self.parse_data();
            return Some(result);
        }
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
                protocol_display_name: None,
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
                protocol_display_name: None,
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
            // State0: detect preamble pattern type
            DecoderState::WaitEdge => {
                if !level {
                    return None;
                }
                let diff_250 = duration_diff!(duration, TE_SHORT);
                let diff_125 = duration_diff!(duration, TE_SHORT_125);

                if diff_250 < TE_TOLERANCE_99 {
                    // Pattern 1: 250µs preamble
                    self.init_preamble_state();
                    self.state = DecoderState::CountPattern250;
                } else if diff_125 < 40 && duration <= 180 {
                    // Pattern 2: 125µs preamble
                    self.init_preamble_state();
                    self.state = DecoderState::CountPattern125;
                }
                self.prev_duration = duration;
            }

            // State1: count 250µs preamble (Pattern 1)
            DecoderState::CountPattern250 => {
                if level {
                    return None;
                }
                let diff_short = duration_diff!(duration, TE_SHORT);
                if diff_short < TE_TOLERANCE_99 + 1 {
                    let prev_diff = duration_diff!(self.prev_duration, TE_SHORT);
                    if prev_diff <= TE_TOLERANCE_99 {
                        self.pattern_counter += 1;
                    }
                    self.prev_duration = duration;
                } else {
                    let diff_long = duration_diff!(duration, TE_LONG);
                    if diff_long < 100 && self.pattern_counter > 0x46 {
                        // Transition to Manchester decode at 250/500µs
                        self.state = DecoderState::DecodeManchester250;
                        self.data_low = 0;
                        self.data_high = 0;
                        self.bit_count = 0;
                        self.manchester_state = ManchesterState::Mid1;
                        self.pattern_counter = 0;
                        self.prev_duration = duration;
                    } else {
                        self.state = DecoderState::WaitEdge;
                        self.pattern_counter = 0;
                    }
                }
            }

            // State2: Manchester decode at 250/500µs (Pattern 1)
            DecoderState::DecodeManchester250 => {
                if self.bit_count >= 121 {
                    return self.finalize_frame();
                }
                // Check for end-of-frame marker
                if level && self.bit_count == 80 && duration >= 800 {
                    let end_diff = duration_diff!(duration, TE_END_1000);
                    if end_diff <= 199 {
                        return self.finalize_frame();
                    }
                }
                let is_short = duration_diff!(duration, TE_SHORT) < TE_DELTA;
                let is_long = duration_diff!(duration, TE_LONG) < TE_DELTA;

                if duration > 10000 {
                    self.state = DecoderState::WaitEdge;
                    self.pattern_counter = 0;
                    return None;
                }

                if is_short || is_long {
                    if let Some(bit) = self.manchester_advance(is_short, level) {
                        self.add_bit(bit);
                    }
                }
                self.prev_duration = duration;
            }

            // State3: count 125µs preamble (Pattern 2)
            DecoderState::CountPattern125 => {
                let diff_125 = duration_diff!(duration, TE_SHORT_125);
                let diff_250 = duration_diff!(duration, TE_LONG_250);

                if diff_125 < TE_TOLERANCE_50 {
                    self.pattern_counter += 1;
                    self.prev_duration = duration;
                } else if diff_250 < TE_TOLERANCE_99 && self.pattern_counter >= 0x45 {
                    // Transition to Manchester decode at 125/250µs
                    self.state = DecoderState::DecodeManchester125;
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

            // State4: Manchester decode at 125/250µs (Pattern 2)
            DecoderState::DecodeManchester125 => {
                if self.bit_count >= 121 {
                    return self.finalize_frame();
                }
                let is_short = duration_diff!(duration, TE_SHORT_125) < TE_TOLERANCE_50;
                let is_long = duration_diff!(duration, TE_LONG_250) < TE_TOLERANCE_99;
                let is_end = duration > 500;

                if is_end {
                    return self.finalize_frame();
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

        // XOR encrypt (matches psa.c psa_second_stage_xor_encrypt)
        Self::xor_encrypt(&mut buffer);

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

        // Preamble: 80 iterations at 250us HIGH+LOW (matches C: te = PSA_TE_LONG_250)
        for _ in 0..80 {
            signal.push(LevelDuration::new(true, TE_LONG_250));
            signal.push(LevelDuration::new(false, TE_LONG_250));
        }

        // Sync transition: LOW 250us, HIGH 500us, LOW 250us (matches C)
        signal.push(LevelDuration::new(false, TE_LONG_250));
        signal.push(LevelDuration::new(true, TE_LONG));
        signal.push(LevelDuration::new(false, TE_LONG_250));

        // Key1: 64 bits Manchester at 250us (C: bit 1 = true,false; bit 0 = false,true)
        let key1 = ((key1_high as u64) << 32) | (key1_low as u64);
        for bit in (0..64).rev() {
            if (key1 >> bit) & 1 == 1 {
                signal.push(LevelDuration::new(true, TE_LONG_250));
                signal.push(LevelDuration::new(false, TE_LONG_250));
            } else {
                signal.push(LevelDuration::new(false, TE_LONG_250));
                signal.push(LevelDuration::new(true, TE_LONG_250));
            }
        }

        // Validation: 16 bits Manchester at 250us
        for bit in (0..16).rev() {
            if (validation >> bit) & 1 == 1 {
                signal.push(LevelDuration::new(true, TE_LONG_250));
                signal.push(LevelDuration::new(false, TE_LONG_250));
            } else {
                signal.push(LevelDuration::new(false, TE_LONG_250));
                signal.push(LevelDuration::new(true, TE_LONG_250));
            }
        }

        // End marker: HIGH 1000us + LOW 1000us (matches C)
        signal.push(LevelDuration::new(true, TE_END_1000));
        signal.push(LevelDuration::new(false, TE_END_1000));

        Some(signal)
    }
}

impl Default for PsaDecoder {
    fn default() -> Self {
        Self::new()
    }
}
