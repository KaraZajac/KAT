//! VAG (VW/Audi/Seat/Skoda) protocol decoder/encoder
//!
//! Ported from protopirate's vag.c
//!
//! Protocol characteristics:
//! - Manchester encoding
//! - Type 1/2: 300/600µs timing (te_short=300), 80 bits, AUT64 or TEA encryption
//! - Type 3/4: 500/1000µs timing (te_short=500), 80 bits, AUT64 encryption
//! - 433.92 MHz and 434.42 MHz
//! - Four sub-types:
//!   Type 1: AUT64 encrypted (300µs Manchester, dispatch 0x2A/0x1C/0x46)
//!   Type 2: TEA encrypted (300µs Manchester, dispatch 0x2A/0x1C/0x46)
//!   Type 3: AUT64 encrypted (500µs Manchester, auto-detect key)
//!   Type 4: AUT64 encrypted (500µs Manchester, key 2)

use super::{ProtocolDecoder, ProtocolTiming, DecodedSignal};
use super::aut64;
use super::keys;
use crate::radio::demodulator::LevelDuration;

// Type 3/4 timing (used as default for ProtocolTiming)
const TE_SHORT: u32 = 500;
const TE_LONG: u32 = 1000;
const TE_DELTA: u32 = 80;
#[allow(dead_code)]
const MIN_COUNT_BIT: usize = 80;

// Type 1/2 timing
const TE_SHORT_12: u32 = 300;
const TE_LONG_12: u32 = 600;
const TE_DELTA_12: u32 = 79;

// TEA constants
const TEA_DELTA: u32 = 0x9E3779B9;
const TEA_ROUNDS: usize = 32;

/// TEA key schedule for VAG
static TEA_KEY_SCHEDULE: [u32; 4] = [0x0B46502D, 0x5E253718, 0x2BF93A19, 0x622C1206];

/// Manchester states
#[derive(Debug, Clone, Copy, PartialEq)]
enum ManchesterState {
    Mid0,
    Mid1,
    Start0,
    Start1,
}

/// Manchester event types
#[derive(Debug, Clone, Copy)]
enum ManchesterEvent {
    ShortHigh,
    ShortLow,
    LongHigh,
    LongLow,
    Reset,
}

/// Decoder states
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Preamble1,    // Type 1/2: ~300µs preamble
    Data1,        // Type 1/2: Manchester data
    Preamble2,    // Type 3/4: ~500µs preamble
    Sync2A,       // Type 3/4: sync pattern step A
    Sync2B,       // Type 3/4: sync pattern step B (750µs)
    Sync2C,       // Type 3/4: sync pattern step C (750µs)
    Data2,        // Type 3/4: Manchester data
}

/// VAG sub-type
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VagType {
    Unknown = 0,
    Type1 = 1,   // AUT64, 300µs
    Type2 = 2,   // TEA, 300µs
    Type3 = 3,   // AUT64, 500µs, auto-detect key
    Type4 = 4,   // AUT64, 500µs, key 2
}

/// VAG protocol decoder
pub struct VagDecoder {
    step: DecoderStep,
    manchester_state: ManchesterState,
    data_low: u32,
    data_high: u32,
    bit_count: usize,
    key1_low: u32,
    key1_high: u32,
    key2_low: u32,
    key2_high: u32,
    te_last: u32,
    header_count: u16,
    mid_count: u8,
    vag_type: VagType,
    // Decoded fields
    serial: u32,
    cnt: u32,
    btn: u8,
    check_byte: u8,
    key_idx: u8,
    decrypted: bool,
    data_count_bit: usize,
}

impl VagDecoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            manchester_state: ManchesterState::Mid1,
            data_low: 0,
            data_high: 0,
            bit_count: 0,
            key1_low: 0,
            key1_high: 0,
            key2_low: 0,
            key2_high: 0,
            te_last: 0,
            header_count: 0,
            mid_count: 0,
            vag_type: VagType::Unknown,
            serial: 0,
            cnt: 0,
            btn: 0,
            check_byte: 0,
            key_idx: 0xFF,
            decrypted: false,
            data_count_bit: 0,
        }
    }

    /// Manchester state machine advance
    fn manchester_advance(&mut self, event: ManchesterEvent) -> Option<bool> {
        match event {
            ManchesterEvent::Reset => {
                self.manchester_state = ManchesterState::Mid1;
                None
            }
            ManchesterEvent::ShortHigh => {
                let (new_state, output) = match self.manchester_state {
                    ManchesterState::Mid0 | ManchesterState::Mid1 => (ManchesterState::Start1, None),
                    ManchesterState::Start0 => (ManchesterState::Mid0, Some(false)),
                    _ => (ManchesterState::Mid1, None),
                };
                self.manchester_state = new_state;
                output
            }
            ManchesterEvent::ShortLow => {
                let (new_state, output) = match self.manchester_state {
                    ManchesterState::Mid0 | ManchesterState::Mid1 => (ManchesterState::Start0, None),
                    ManchesterState::Start1 => (ManchesterState::Mid1, Some(true)),
                    _ => (ManchesterState::Mid1, None),
                };
                self.manchester_state = new_state;
                output
            }
            ManchesterEvent::LongHigh => {
                let (new_state, output) = match self.manchester_state {
                    ManchesterState::Start0 => (ManchesterState::Start1, Some(false)),
                    _ => (ManchesterState::Mid1, None),
                };
                self.manchester_state = new_state;
                output
            }
            ManchesterEvent::LongLow => {
                let (new_state, output) = match self.manchester_state {
                    ManchesterState::Start1 => (ManchesterState::Start0, Some(true)),
                    _ => (ManchesterState::Mid1, None),
                };
                self.manchester_state = new_state;
                output
            }
        }
    }

    /// Push a bit into the shift register
    fn push_bit(&mut self, bit: bool) {
        let carry = (self.data_low >> 31) & 1;
        self.data_low = (self.data_low << 1) | (if bit { 1 } else { 0 });
        self.data_high = (self.data_high << 1) | carry;
        self.bit_count += 1;
    }

    /// TEA decrypt
    fn tea_decrypt(v0: &mut u32, v1: &mut u32, key_schedule: &[u32; 4]) {
        let mut sum = TEA_DELTA.wrapping_mul(TEA_ROUNDS as u32);
        for _ in 0..TEA_ROUNDS {
            *v1 = v1.wrapping_sub(
                ((*v0 << 4) ^ (*v0 >> 5)).wrapping_add(*v0)
                    ^ sum.wrapping_add(key_schedule[((sum >> 11) & 3) as usize]),
            );
            sum = sum.wrapping_sub(TEA_DELTA);
            *v0 = v0.wrapping_sub(
                ((*v1 << 4) ^ (*v1 >> 5)).wrapping_add(*v1)
                    ^ sum.wrapping_add(key_schedule[(sum & 3) as usize]),
            );
        }
    }

    /// TEA encrypt
    fn tea_encrypt(v0: &mut u32, v1: &mut u32, key_schedule: &[u32; 4]) {
        let mut sum: u32 = 0;
        for _ in 0..TEA_ROUNDS {
            *v0 = v0.wrapping_add(
                ((*v1 << 4) ^ (*v1 >> 5)).wrapping_add(*v1)
                    ^ sum.wrapping_add(key_schedule[(sum & 3) as usize]),
            );
            sum = sum.wrapping_add(TEA_DELTA);
            *v1 = v1.wrapping_add(
                ((*v0 << 4) ^ (*v0 >> 5)).wrapping_add(*v0)
                    ^ sum.wrapping_add(key_schedule[((sum >> 11) & 3) as usize]),
            );
        }
    }

    /// Check if dispatch byte matches Type 1/2
    fn dispatch_type_1_2(dispatch: u8) -> bool {
        dispatch == 0x2A || dispatch == 0x1C || dispatch == 0x46
    }

    /// Check if dispatch byte matches Type 3/4
    fn dispatch_type_3_4(dispatch: u8) -> bool {
        dispatch == 0x2B || dispatch == 0x1D || dispatch == 0x47
    }

    /// Validate decrypted button byte
    fn button_valid(dec: &[u8]) -> bool {
        let dec_byte = dec[7];
        let dec_btn = (dec_byte >> 4) & 0xF;
        if dec_btn == 1 || dec_btn == 2 || dec_btn == 4 {
            return true;
        }
        if dec_byte == 0 {
            return true;
        }
        false
    }

    /// Check if decrypted button matches dispatch byte
    fn button_matches(dec: &[u8], dispatch_byte: u8) -> bool {
        let expected_btn = (dispatch_byte >> 4) & 0xF;
        let dec_btn = (dec[7] >> 4) & 0xF;
        if dec_btn == expected_btn {
            return true;
        }
        if dec[7] == 0 && expected_btn == 2 {
            return true;
        }
        false
    }

    /// Fill decoded fields from decrypted block
    fn fill_from_decrypted(&mut self, dec: &[u8], dispatch_byte: u8) {
        let serial_raw = (dec[0] as u32)
            | ((dec[1] as u32) << 8)
            | ((dec[2] as u32) << 16)
            | ((dec[3] as u32) << 24);
        self.serial = (serial_raw << 24)
            | ((serial_raw & 0xFF00) << 8)
            | ((serial_raw >> 8) & 0xFF00)
            | (serial_raw >> 24);

        self.cnt = (dec[4] as u32) | ((dec[5] as u32) << 8) | ((dec[6] as u32) << 16);
        self.btn = (dec[7] >> 4) & 0xF;
        self.check_byte = dispatch_byte;
        self.decrypted = true;
    }

    /// Try AUT64 decryption with a specific key index
    fn try_aut64_decrypt(block: &mut [u8], key_index: usize) -> bool {
        let store = keys::get_keystore();
        if let Some(key) = store.get_vag_key((key_index + 1) as u8) {
            aut64::aut64_decrypt(key, block);
            true
        } else {
            false
        }
    }

    /// Parse the collected data and attempt decryption
    fn parse_data(&mut self) {
        self.decrypted = false;
        self.serial = 0;
        self.cnt = 0;
        self.btn = 0;

        let dispatch_byte = (self.key2_low & 0xFF) as u8;
        let key2_high_byte = ((self.key2_low >> 8) & 0xFF) as u8;

        // Build key1 bytes from key1_high/key1_low
        let mut key1_bytes = [0u8; 8];
        key1_bytes[0] = (self.key1_high >> 24) as u8;
        key1_bytes[1] = (self.key1_high >> 16) as u8;
        key1_bytes[2] = (self.key1_high >> 8) as u8;
        key1_bytes[3] = self.key1_high as u8;
        key1_bytes[4] = (self.key1_low >> 24) as u8;
        key1_bytes[5] = (self.key1_low >> 16) as u8;
        key1_bytes[6] = (self.key1_low >> 8) as u8;
        key1_bytes[7] = self.key1_low as u8;

        let _type_byte = key1_bytes[0];

        // Build encrypted block (bytes 1-7 of key1 + key2 high byte)
        let mut block = [0u8; 8];
        block[0] = key1_bytes[1];
        block[1] = key1_bytes[2];
        block[2] = key1_bytes[3];
        block[3] = key1_bytes[4];
        block[4] = key1_bytes[5];
        block[5] = key1_bytes[6];
        block[6] = key1_bytes[7];
        block[7] = key2_high_byte;

        match self.vag_type {
            VagType::Type1 => {
                if !Self::dispatch_type_1_2(dispatch_byte) {
                    return;
                }
                // Try all 3 AUT64 keys
                for key_idx in 0..3 {
                    let mut block_copy = block;
                    if !Self::try_aut64_decrypt(&mut block_copy, key_idx) {
                        continue;
                    }
                    if Self::button_valid(&block_copy) {
                        self.serial = ((block_copy[0] as u32) << 24)
                            | ((block_copy[1] as u32) << 16)
                            | ((block_copy[2] as u32) << 8)
                            | (block_copy[3] as u32);
                        self.cnt = (block_copy[4] as u32)
                            | ((block_copy[5] as u32) << 8)
                            | ((block_copy[6] as u32) << 16);
                        self.btn = block_copy[7];
                        self.check_byte = dispatch_byte;
                        self.key_idx = key_idx as u8;
                        self.decrypted = true;
                        return;
                    }
                }
            }

            VagType::Type2 => {
                if !Self::dispatch_type_1_2(dispatch_byte) {
                    return;
                }
                let mut v0 = ((block[0] as u32) << 24)
                    | ((block[1] as u32) << 16)
                    | ((block[2] as u32) << 8)
                    | (block[3] as u32);
                let mut v1 = ((block[4] as u32) << 24)
                    | ((block[5] as u32) << 16)
                    | ((block[6] as u32) << 8)
                    | (block[7] as u32);

                Self::tea_decrypt(&mut v0, &mut v1, &TEA_KEY_SCHEDULE);

                let tea_dec = [
                    (v0 >> 24) as u8,
                    (v0 >> 16) as u8,
                    (v0 >> 8) as u8,
                    v0 as u8,
                    (v1 >> 24) as u8,
                    (v1 >> 16) as u8,
                    (v1 >> 8) as u8,
                    v1 as u8,
                ];

                if !Self::button_matches(&tea_dec, dispatch_byte) {
                    return;
                }

                self.fill_from_decrypted(&tea_dec, dispatch_byte);
                self.key_idx = 0xFF;
            }

            VagType::Type3 => {
                // Try key 2 first, then key 1, then key 0
                let mut block_copy = block;
                if Self::try_aut64_decrypt(&mut block_copy, 2) && Self::button_valid(&block_copy) {
                    self.vag_type = VagType::Type4;
                    self.key_idx = 2;
                    self.fill_from_decrypted(&block_copy, dispatch_byte);
                    return;
                }

                block_copy = block;
                if Self::try_aut64_decrypt(&mut block_copy, 1) && Self::button_valid(&block_copy) {
                    self.key_idx = 1;
                    self.fill_from_decrypted(&block_copy, dispatch_byte);
                    return;
                }

                block_copy = block;
                if Self::try_aut64_decrypt(&mut block_copy, 0) && Self::button_valid(&block_copy) {
                    self.key_idx = 0;
                    self.fill_from_decrypted(&block_copy, dispatch_byte);
                    return;
                }
            }

            VagType::Type4 => {
                if !Self::dispatch_type_3_4(dispatch_byte) {
                    return;
                }
                let mut block_copy = block;
                if !Self::try_aut64_decrypt(&mut block_copy, 2) {
                    return;
                }
                if !Self::button_matches(&block_copy, dispatch_byte) {
                    return;
                }
                self.key_idx = 2;
                self.fill_from_decrypted(&block_copy, dispatch_byte);
            }

            VagType::Unknown => {}
        }
    }

    /// Get vehicle name from type byte
    #[allow(dead_code)]
    fn get_vehicle_name(type_byte: u8) -> &'static str {
        match type_byte {
            0x00 => "VW Passat",
            0xC0 => "VW",
            0xC1 => "Audi",
            0xC2 => "Seat",
            0xC3 => "Skoda",
            _ => "VAG",
        }
    }

    /// Get button name
    #[allow(dead_code)]
    fn get_button_name(btn: u8) -> &'static str {
        match btn {
            1 => "Unlock",
            2 => "Lock",
            4 => "Trunk",
            _ => "Unknown",
        }
    }

    /// Build encoder output for the given signal
    fn encode_signal(&self, decoded: &DecodedSignal) -> Option<Vec<LevelDuration>> {
        if !self.decrypted {
            return None;
        }

        match self.vag_type {
            VagType::Type1 => self.encode_type1(decoded),
            VagType::Type2 => self.encode_type2(decoded),
            VagType::Type3 | VagType::Type4 => self.encode_type3_4(decoded),
            _ => None,
        }
    }

    /// Encode Type 1 (300µs, AUT64)
    fn encode_type1(&self, _decoded: &DecodedSignal) -> Option<Vec<LevelDuration>> {
        let mut upload = Vec::with_capacity(700);

        let btn_byte = self.btn;
        let dispatch = Self::get_dispatch_byte(btn_byte, 1);
        let type_byte = (self.key1_high >> 24) as u8;

        // Build plaintext block
        let mut block = [0u8; 8];
        block[0] = (self.serial >> 24) as u8;
        block[1] = (self.serial >> 16) as u8;
        block[2] = (self.serial >> 8) as u8;
        block[3] = self.serial as u8;
        block[4] = self.cnt as u8;
        block[5] = (self.cnt >> 8) as u8;
        block[6] = (self.cnt >> 16) as u8;
        block[7] = btn_byte;

        // Encrypt with AUT64
        let key_idx = if self.key_idx != 0xFF { self.key_idx as usize } else { 0 };
        let store = keys::get_keystore();
        if let Some(key) = store.get_vag_key((key_idx + 1) as u8) {
            aut64::aut64_encrypt(key, &mut block);
        } else {
            return None;
        }
        drop(store);

        // Build key values
        let key1_high = ((type_byte as u32) << 24)
            | ((block[0] as u32) << 16)
            | ((block[1] as u32) << 8)
            | (block[2] as u32);
        let key1_low = ((block[3] as u32) << 24)
            | ((block[4] as u32) << 16)
            | ((block[5] as u32) << 8)
            | (block[6] as u32);
        let key2 = (((block[7] as u16) << 8) | (dispatch as u16)) & 0xFFFF;

        // Preamble: 220 cycles of 300µs HIGH/LOW
        for _ in 0..220 {
            upload.push(LevelDuration::new(true, 300));
            upload.push(LevelDuration::new(false, 300));
        }
        upload.push(LevelDuration::new(false, 300));
        upload.push(LevelDuration::new(true, 300));

        // Prefix: 0xAF3F (16 bits, Manchester)
        let prefix: u16 = 0xAF3F;
        Self::encode_manchester_16(&mut upload, prefix, 300);

        // Key1: 64 bits inverted, Manchester encoded
        let key1 = ((key1_high as u64) << 32) | (key1_low as u64);
        let key1_inv = !key1;
        Self::encode_manchester_64(&mut upload, key1_inv, 300);

        // Key2: 16 bits inverted, Manchester encoded
        let key2_inv = !key2;
        Self::encode_manchester_16(&mut upload, key2_inv, 300);

        // Gap
        upload.push(LevelDuration::new(false, 6000));

        Some(upload)
    }

    /// Encode Type 2 (300µs, TEA)
    fn encode_type2(&self, _decoded: &DecodedSignal) -> Option<Vec<LevelDuration>> {
        let mut upload = Vec::with_capacity(700);

        let btn_byte = Self::btn_to_byte(self.btn, 2);
        let dispatch = Self::get_dispatch_byte(btn_byte, 2);
        let type_byte = (self.key1_high >> 24) as u8;

        // Build plaintext block
        let mut block = [0u8; 8];
        block[0] = (self.serial >> 24) as u8;
        block[1] = (self.serial >> 16) as u8;
        block[2] = (self.serial >> 8) as u8;
        block[3] = self.serial as u8;
        block[4] = self.cnt as u8;
        block[5] = (self.cnt >> 8) as u8;
        block[6] = (self.cnt >> 16) as u8;
        block[7] = btn_byte;

        // Encrypt with TEA
        let mut v0 = ((block[0] as u32) << 24)
            | ((block[1] as u32) << 16)
            | ((block[2] as u32) << 8)
            | (block[3] as u32);
        let mut v1 = ((block[4] as u32) << 24)
            | ((block[5] as u32) << 16)
            | ((block[6] as u32) << 8)
            | (block[7] as u32);

        Self::tea_encrypt(&mut v0, &mut v1, &TEA_KEY_SCHEDULE);

        let enc_block = [
            (v0 >> 24) as u8, (v0 >> 16) as u8, (v0 >> 8) as u8, v0 as u8,
            (v1 >> 24) as u8, (v1 >> 16) as u8, (v1 >> 8) as u8, v1 as u8,
        ];

        let key1_high = ((type_byte as u32) << 24)
            | ((enc_block[0] as u32) << 16)
            | ((enc_block[1] as u32) << 8)
            | (enc_block[2] as u32);
        let key1_low = ((enc_block[3] as u32) << 24)
            | ((enc_block[4] as u32) << 16)
            | ((enc_block[5] as u32) << 8)
            | (enc_block[6] as u32);
        let key2 = (((enc_block[7] as u16) << 8) | (dispatch as u16)) & 0xFFFF;

        // Preamble
        for _ in 0..220 {
            upload.push(LevelDuration::new(true, 300));
            upload.push(LevelDuration::new(false, 300));
        }
        upload.push(LevelDuration::new(false, 300));
        upload.push(LevelDuration::new(true, 300));

        // Prefix: 0xAF1C (16 bits, Manchester)
        let prefix: u16 = 0xAF1C;
        Self::encode_manchester_16(&mut upload, prefix, 300);

        // Key1: 64 bits inverted
        let key1 = ((key1_high as u64) << 32) | (key1_low as u64);
        let key1_inv = !key1;
        Self::encode_manchester_64(&mut upload, key1_inv, 300);

        // Key2: 16 bits inverted
        let key2_inv = !key2;
        Self::encode_manchester_16(&mut upload, key2_inv, 300);

        // Gap
        upload.push(LevelDuration::new(false, 6000));

        Some(upload)
    }

    /// Encode Type 3/4 (500µs, AUT64)
    fn encode_type3_4(&self, _decoded: &DecodedSignal) -> Option<Vec<LevelDuration>> {
        let mut upload = Vec::with_capacity(600);
        let vag_type_num = self.vag_type as u8;

        let btn_byte = Self::btn_to_byte(self.btn, vag_type_num);
        let dispatch = Self::get_dispatch_byte(btn_byte, vag_type_num);
        let type_byte = (self.key1_high >> 24) as u8;

        let mut block = [0u8; 8];
        block[0] = (self.serial >> 24) as u8;
        block[1] = (self.serial >> 16) as u8;
        block[2] = (self.serial >> 8) as u8;
        block[3] = self.serial as u8;
        block[4] = self.cnt as u8;
        block[5] = (self.cnt >> 8) as u8;
        block[6] = (self.cnt >> 16) as u8;
        block[7] = btn_byte;

        let key_idx = if self.key_idx != 0xFF {
            self.key_idx as usize
        } else if self.vag_type == VagType::Type4 { 2 } else { 1 };

        let store = keys::get_keystore();
        if let Some(key) = store.get_vag_key((key_idx + 1) as u8) {
            aut64::aut64_encrypt(key, &mut block);
        } else {
            return None;
        }
        drop(store);

        let key1_high = ((type_byte as u32) << 24)
            | ((block[0] as u32) << 16)
            | ((block[1] as u32) << 8)
            | (block[2] as u32);
        let key1_low = ((block[3] as u32) << 24)
            | ((block[4] as u32) << 16)
            | ((block[5] as u32) << 8)
            | (block[6] as u32);
        let key2 = (((block[7] as u16) << 8) | (dispatch as u16)) & 0xFFFF;

        let key1 = ((key1_high as u64) << 32) | (key1_low as u64);

        // Two repeats
        for _ in 0..2 {
            // Preamble: 45 cycles of 500µs HIGH/LOW
            for _ in 0..45 {
                upload.push(LevelDuration::new(true, 500));
                upload.push(LevelDuration::new(false, 500));
            }

            // Sync: 1000µs HIGH, 500µs LOW
            upload.push(LevelDuration::new(true, 1000));
            upload.push(LevelDuration::new(false, 500));

            // Mid sync: 3 cycles of 750µs HIGH/LOW
            for _ in 0..3 {
                upload.push(LevelDuration::new(true, 750));
                upload.push(LevelDuration::new(false, 750));
            }

            // Key1: 64 bits (NOT inverted for Type 3/4)
            for i in (0..64).rev() {
                let bit = (key1 >> i) & 1 == 1;
                if bit {
                    upload.push(LevelDuration::new(true, 500));
                    upload.push(LevelDuration::new(false, 500));
                } else {
                    upload.push(LevelDuration::new(false, 500));
                    upload.push(LevelDuration::new(true, 500));
                }
            }

            // Key2: 16 bits
            for i in (0..16).rev() {
                let bit = (key2 >> i) & 1 == 1;
                if bit {
                    upload.push(LevelDuration::new(true, 500));
                    upload.push(LevelDuration::new(false, 500));
                } else {
                    upload.push(LevelDuration::new(false, 500));
                    upload.push(LevelDuration::new(true, 500));
                }
            }

            // Gap
            upload.push(LevelDuration::new(false, 10000));
        }

        Some(upload)
    }

    /// Get dispatch byte from button and type
    fn get_dispatch_byte(btn: u8, vag_type: u8) -> u8 {
        if vag_type == 1 || vag_type == 2 {
            match btn {
                0x20 | 2 => 0x2A,
                0x40 | 4 => 0x46,
                0x10 | 1 => 0x1C,
                _ => 0x2A,
            }
        } else {
            match btn {
                0x20 | 2 => 0x2B,
                0x40 | 4 => 0x47,
                0x10 | 1 => 0x1D,
                _ => 0x2B,
            }
        }
    }

    /// Convert button code to byte for encoding
    fn btn_to_byte(btn: u8, vag_type: u8) -> u8 {
        if vag_type == 1 {
            btn
        } else {
            match btn {
                1 => 0x10,
                2 => 0x20,
                4 => 0x40,
                _ => 0x20,
            }
        }
    }

    /// Encode 16 bits in Manchester at given half-period
    fn encode_manchester_16(upload: &mut Vec<LevelDuration>, data: u16, te: u32) {
        for i in (0..16).rev() {
            let bit = (data >> i) & 1 == 1;
            if bit {
                upload.push(LevelDuration::new(true, te));
                upload.push(LevelDuration::new(false, te));
            } else {
                upload.push(LevelDuration::new(false, te));
                upload.push(LevelDuration::new(true, te));
            }
        }
    }

    /// Encode 64 bits in Manchester at given half-period
    fn encode_manchester_64(upload: &mut Vec<LevelDuration>, data: u64, te: u32) {
        for i in (0..64).rev() {
            let bit = (data >> i) & 1 == 1;
            if bit {
                upload.push(LevelDuration::new(true, te));
                upload.push(LevelDuration::new(false, te));
            } else {
                upload.push(LevelDuration::new(false, te));
                upload.push(LevelDuration::new(true, te));
            }
        }
    }

    /// Build DecodedSignal from internal state
    fn build_decoded_signal(&self) -> DecodedSignal {
        let key1 = ((self.key1_high as u64) << 32) | (self.key1_low as u64);

        DecodedSignal {
            serial: if self.decrypted { Some(self.serial) } else { None },
            button: if self.decrypted { Some(self.btn) } else { None },
            counter: if self.decrypted { Some((self.cnt & 0xFFFF) as u16) } else { None },
            crc_valid: self.decrypted,
            data: key1,
            data_count_bit: self.data_count_bit,
            encoder_capable: self.decrypted,
        }
    }
}

impl ProtocolDecoder for VagDecoder {
    fn name(&self) -> &'static str {
        "VAG"
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
        &[433_920_000, 434_420_000]
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.manchester_state = ManchesterState::Mid1;
        self.data_low = 0;
        self.data_high = 0;
        self.bit_count = 0;
        self.header_count = 0;
        self.mid_count = 0;
        self.vag_type = VagType::Unknown;
        self.te_last = 0;
        self.decrypted = false;
        self.serial = 0;
        self.cnt = 0;
        self.btn = 0;
        self.check_byte = 0;
        self.key_idx = 0xFF;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::Reset => {
                if !level {
                    return None;
                }

                if duration < TE_SHORT_12 {
                    if (TE_SHORT_12 - duration) > TE_DELTA_12 {
                        return None;
                    }
                    // Init pattern 1 (Type 1/2)
                    self.step = DecoderStep::Preamble1;
                    self.data_low = 0;
                    self.data_high = 0;
                    self.header_count = 0;
                    self.mid_count = 0;
                    self.bit_count = 0;
                    self.vag_type = VagType::Unknown;
                    self.te_last = duration;
                    self.manchester_advance(ManchesterEvent::Reset);
                } else if (duration.wrapping_sub(TE_SHORT_12)) <= TE_DELTA_12 {
                    // Still pattern 1
                    self.step = DecoderStep::Preamble1;
                    self.data_low = 0;
                    self.data_high = 0;
                    self.header_count = 0;
                    self.mid_count = 0;
                    self.bit_count = 0;
                    self.vag_type = VagType::Unknown;
                    self.te_last = duration;
                    self.manchester_advance(ManchesterEvent::Reset);
                } else {
                    // Check for pattern 2 (Type 3/4, 500µs)
                    let diff = if duration > TE_SHORT { duration - TE_SHORT } else { TE_SHORT - duration };
                    if diff <= TE_DELTA_12 {
                        self.step = DecoderStep::Preamble2;
                        self.data_low = 0;
                        self.data_high = 0;
                        self.header_count = 0;
                        self.mid_count = 0;
                        self.bit_count = 0;
                        self.vag_type = VagType::Unknown;
                        self.te_last = duration;
                        self.manchester_advance(ManchesterEvent::Reset);
                    }
                }
            }

            DecoderStep::Preamble1 => {
                if level {
                    return None;
                }

                let te_diff = if duration > TE_SHORT_12 {
                    duration - TE_SHORT_12
                } else {
                    TE_SHORT_12 - duration
                };

                if te_diff < TE_DELTA_12 {
                    // Check previous pulse
                    let prev_diff = if self.te_last > TE_SHORT_12 {
                        self.te_last - TE_SHORT_12
                    } else {
                        TE_SHORT_12 - self.te_last
                    };
                    if prev_diff <= TE_DELTA_12 {
                        self.te_last = duration;
                        self.header_count += 1;
                        return None;
                    }
                    self.step = DecoderStep::Reset;
                    return None;
                }

                // Check for gap (end of preamble)
                if self.header_count >= 201 {
                    let gap_diff = if duration > TE_LONG_12 {
                        duration - TE_LONG_12
                    } else {
                        TE_LONG_12 - duration
                    };
                    if gap_diff <= TE_DELTA_12 {
                        let prev_diff = if self.te_last > TE_SHORT_12 {
                            self.te_last - TE_SHORT_12
                        } else {
                            TE_SHORT_12 - self.te_last
                        };
                        if prev_diff <= TE_DELTA_12 {
                            self.step = DecoderStep::Data1;
                            return None;
                        }
                    }
                }

                self.step = DecoderStep::Reset;
            }

            DecoderStep::Data1 => {
                if self.bit_count < 96 {
                    // Determine Manchester event
                    let short_diff = if duration > TE_SHORT_12 {
                        duration - TE_SHORT_12
                    } else {
                        TE_SHORT_12 - duration
                    };
                    let long_diff = if duration > TE_LONG_12 {
                        duration - TE_LONG_12
                    } else {
                        TE_LONG_12 - duration
                    };

                    let event = if short_diff <= TE_DELTA_12 {
                        Some(if level { ManchesterEvent::ShortLow } else { ManchesterEvent::ShortHigh })
                    } else if long_diff <= TE_DELTA_12 {
                        Some(if level { ManchesterEvent::LongLow } else { ManchesterEvent::LongHigh })
                    } else {
                        None
                    };

                    if let Some(evt) = event {
                        if let Some(bit_value) = self.manchester_advance(evt) {
                            self.push_bit(bit_value);

                            // Check for type identifier at bit 15
                            if self.bit_count == 15 {
                                if self.data_low == 0x2F3F && self.data_high == 0 {
                                    self.data_low = 0;
                                    self.data_high = 0;
                                    self.bit_count = 0;
                                    self.vag_type = VagType::Type1;
                                } else if self.data_low == 0x2F1C && self.data_high == 0 {
                                    self.data_low = 0;
                                    self.data_high = 0;
                                    self.bit_count = 0;
                                    self.vag_type = VagType::Type2;
                                }
                            } else if self.bit_count == 64 {
                                self.key1_low = !self.data_low;
                                self.key1_high = !self.data_high;
                                self.data_low = 0;
                                self.data_high = 0;
                            }
                        }
                        return None;
                    }
                }

                // Check for gap (end of data)
                if !level {
                    let gap_diff = if duration > 6000 {
                        duration - 6000
                    } else {
                        6000 - duration
                    };

                    if gap_diff < 4000 && self.bit_count == 80 {
                        self.key2_low = (!self.data_low) & 0xFFFF;
                        self.key2_high = 0;
                        self.data_count_bit = 80;

                        self.parse_data();

                        let result = self.build_decoded_signal();
                        self.data_low = 0;
                        self.data_high = 0;
                        self.bit_count = 0;
                        self.step = DecoderStep::Reset;
                        return Some(result);
                    }
                }

                self.data_low = 0;
                self.data_high = 0;
                self.bit_count = 0;
                self.step = DecoderStep::Reset;
            }

            DecoderStep::Preamble2 => {
                if !level {
                    // Low pulse - check if matches 500µs
                    let diff = if duration > TE_SHORT { duration - TE_SHORT } else { TE_SHORT - duration };
                    if diff < TE_DELTA {
                        let prev_diff = if self.te_last > TE_SHORT {
                            self.te_last - TE_SHORT
                        } else {
                            TE_SHORT - self.te_last
                        };
                        if prev_diff < TE_DELTA {
                            self.te_last = duration;
                            self.header_count += 1;
                            return None;
                        }
                    }
                    self.step = DecoderStep::Reset;
                    return None;
                }

                // High pulse after sufficient preamble
                if self.header_count < 41 {
                    return None;
                }

                // Check for 1000µs sync HIGH
                let diff = if duration > TE_LONG { duration - TE_LONG } else { TE_LONG - duration };
                if diff > TE_DELTA_12 {
                    return None;
                }

                let prev_diff = if self.te_last > TE_SHORT {
                    self.te_last - TE_SHORT
                } else {
                    TE_SHORT - self.te_last
                };
                if prev_diff > TE_DELTA_12 {
                    return None;
                }

                self.te_last = duration;
                self.step = DecoderStep::Sync2A;
            }

            DecoderStep::Sync2A => {
                if !level {
                    let diff = if duration > TE_SHORT { duration - TE_SHORT } else { TE_SHORT - duration };
                    if diff < TE_DELTA {
                        let prev_diff = if self.te_last > TE_LONG {
                            self.te_last - TE_LONG
                        } else {
                            TE_LONG - self.te_last
                        };
                        if prev_diff < TE_DELTA {
                            self.te_last = duration;
                            self.step = DecoderStep::Sync2B;
                            return None;
                        }
                    }
                }
                self.step = DecoderStep::Reset;
            }

            DecoderStep::Sync2B => {
                if level {
                    let diff = if duration > 750 { duration - 750 } else { 750 - duration };
                    if diff < TE_DELTA {
                        self.te_last = duration;
                        self.step = DecoderStep::Sync2C;
                        return None;
                    }
                }
                self.step = DecoderStep::Reset;
            }

            DecoderStep::Sync2C => {
                if !level {
                    let diff = if duration > 750 { duration - 750 } else { 750 - duration };
                    if diff <= TE_DELTA_12 {
                        let prev_diff = if self.te_last > 750 {
                            self.te_last - 750
                        } else {
                            750 - self.te_last
                        };
                        if prev_diff <= TE_DELTA_12 {
                            self.mid_count += 1;
                            self.step = DecoderStep::Sync2B;

                            if self.mid_count == 3 {
                                self.data_low = 1;
                                self.data_high = 0;
                                self.bit_count = 1;
                                self.manchester_advance(ManchesterEvent::Reset);
                                self.step = DecoderStep::Data2;
                            }
                            return None;
                        }
                    }
                }
                self.step = DecoderStep::Reset;
            }

            DecoderStep::Data2 => {
                // Determine Manchester event for Type 3/4 (500/1000µs)
                let event = if duration >= 380 && duration <= 620 {
                    Some(if level { ManchesterEvent::ShortLow } else { ManchesterEvent::ShortHigh })
                } else if duration >= 880 && duration <= 1120 {
                    Some(if level { ManchesterEvent::LongLow } else { ManchesterEvent::LongHigh })
                } else {
                    None
                };

                if let Some(evt) = event {
                    if let Some(bit_value) = self.manchester_advance(evt) {
                        self.push_bit(bit_value);

                        if self.bit_count == 64 {
                            self.key1_low = self.data_low;
                            self.key1_high = self.data_high;
                            self.data_low = 0;
                            self.data_high = 0;
                        }
                    }
                }

                // Check for completion at 80 bits
                if self.bit_count == 80 {
                    self.key2_low = self.data_low & 0xFFFF;
                    self.key2_high = 0;
                    self.data_count_bit = 80;
                    self.vag_type = VagType::Type3;

                    self.parse_data();

                    let result = self.build_decoded_signal();
                    self.data_low = 0;
                    self.data_high = 0;
                    self.bit_count = 0;
                    self.step = DecoderStep::Reset;
                    return Some(result);
                }
            }
        }

        None
    }

    fn supports_encoding(&self) -> bool {
        self.decrypted
    }

    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        self.encode_signal(decoded)
    }
}
