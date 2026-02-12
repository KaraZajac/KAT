//! Kia V6 protocol decoder (decode-only)
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/kia_v6.c`.
//! Decode logic (Manchester level mapping, 3-part 144-bit frame, AES-128, CRC8, keystore XOR) matches reference.
//! No encoder in protopirate.
//!
//! Protocol characteristics:
//! - Manchester encoding: 200/400µs (level convention inverted vs Flipper; see manchester_advance)
//! - 144 bits total: part1 (64) + part2 (64) + part3 (16), each part inverted on store
//! - Long preamble of 601 pairs; sync bits 1,1,0,1 then data
//! - AES-128 decryption with key derived from KIA V6 A/B keystores (types 11/12) and XOR masks

use super::{ProtocolDecoder, ProtocolTiming, DecodedSignal};
use super::keys;
use crate::radio::demodulator::LevelDuration;
use crate::duration_diff;

const TE_SHORT: u32 = 200;
const TE_LONG: u32 = 400;
const TE_DELTA: u32 = 100;
const MIN_COUNT_BIT: usize = 144;
const PREAMBLE_COUNT: u16 = 601;

const XOR_MASK_LOW: u32 = 0x84AF25FB;
const XOR_MASK_HIGH: u32 = 0x638766AB;

/// AES S-box
const AES_SBOX: [u8; 256] = [
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
    0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
    0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
    0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
    0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
    0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
    0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
    0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
    0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
    0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
    0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16,
];

/// AES inverse S-box
const AES_SBOX_INV: [u8; 256] = [
    0x52, 0x09, 0x6a, 0xd5, 0x30, 0x36, 0xa5, 0x38, 0xbf, 0x40, 0xa3, 0x9e, 0x81, 0xf3, 0xd7, 0xfb,
    0x7c, 0xe3, 0x39, 0x82, 0x9b, 0x2f, 0xff, 0x87, 0x34, 0x8e, 0x43, 0x44, 0xc4, 0xde, 0xe9, 0xcb,
    0x54, 0x7b, 0x94, 0x32, 0xa6, 0xc2, 0x23, 0x3d, 0xee, 0x4c, 0x95, 0x0b, 0x42, 0xfa, 0xc3, 0x4e,
    0x08, 0x2e, 0xa1, 0x66, 0x28, 0xd9, 0x24, 0xb2, 0x76, 0x5b, 0xa2, 0x49, 0x6d, 0x8b, 0xd1, 0x25,
    0x72, 0xf8, 0xf6, 0x64, 0x86, 0x68, 0x98, 0x16, 0xd4, 0xa4, 0x5c, 0xcc, 0x5d, 0x65, 0xb6, 0x92,
    0x6c, 0x70, 0x48, 0x50, 0xfd, 0xed, 0xb9, 0xda, 0x5e, 0x15, 0x46, 0x57, 0xa7, 0x8d, 0x9d, 0x84,
    0x90, 0xd8, 0xab, 0x00, 0x8c, 0xbc, 0xd3, 0x0a, 0xf7, 0xe4, 0x58, 0x05, 0xb8, 0xb3, 0x45, 0x06,
    0xd0, 0x2c, 0x1e, 0x8f, 0xca, 0x3f, 0x0f, 0x02, 0xc1, 0xaf, 0xbd, 0x03, 0x01, 0x13, 0x8a, 0x6b,
    0x3a, 0x91, 0x11, 0x41, 0x4f, 0x67, 0xdc, 0xea, 0x97, 0xf2, 0xcf, 0xce, 0xf0, 0xb4, 0xe6, 0x73,
    0x96, 0xac, 0x74, 0x22, 0xe7, 0xad, 0x35, 0x85, 0xe2, 0xf9, 0x37, 0xe8, 0x1c, 0x75, 0xdf, 0x6e,
    0x47, 0xf1, 0x1a, 0x71, 0x1d, 0x29, 0xc5, 0x89, 0x6f, 0xb7, 0x62, 0x0e, 0xaa, 0x18, 0xbe, 0x1b,
    0xfc, 0x56, 0x3e, 0x4b, 0xc6, 0xd2, 0x79, 0x20, 0x9a, 0xdb, 0xc0, 0xfe, 0x78, 0xcd, 0x5a, 0xf4,
    0x1f, 0xdd, 0xa8, 0x33, 0x88, 0x07, 0xc7, 0x31, 0xb1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xec, 0x5f,
    0x60, 0x51, 0x7f, 0xa9, 0x19, 0xb5, 0x4a, 0x0d, 0x2d, 0xe5, 0x7a, 0x9f, 0x93, 0xc9, 0x9c, 0xef,
    0xa0, 0xe0, 0x3b, 0x4d, 0xae, 0x2a, 0xf5, 0xb0, 0xc8, 0xeb, 0xbb, 0x3c, 0x83, 0x53, 0x99, 0x61,
    0x17, 0x2b, 0x04, 0x7e, 0xba, 0x77, 0xd6, 0x26, 0xe1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0c, 0x7d,
];

const AES_RCON: [u8; 10] = [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36];

/// Manchester decoder states (event mapping 0/2/4/6 matches protopirate kia_v6 level convention)
#[derive(Debug, Clone, Copy, PartialEq)]
enum ManchesterState {
    Mid0,
    Mid1,
    Start0,
    Start1,
}

/// Decoder states (matches protopirate's KiaV6DecoderStep)
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    WaitFirstHigh,
    WaitLongHigh,
    Data,
}

/// Kia V6 protocol decoder
pub struct KiaV6Decoder {
    step: DecoderStep,
    te_last: u32,
    header_count: u16,
    manchester_state: ManchesterState,
    
    data_part1_low: u32,
    data_part1_high: u32,
    stored_part1_low: u32,
    stored_part1_high: u32,
    stored_part2_low: u32,
    stored_part2_high: u32,
    data_part3: u16,
    
    bit_count: u8,
}

impl KiaV6Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            te_last: 0,
            header_count: 0,
            manchester_state: ManchesterState::Mid1,
            data_part1_low: 0,
            data_part1_high: 0,
            stored_part1_low: 0,
            stored_part1_high: 0,
            stored_part2_low: 0,
            stored_part2_high: 0,
            data_part3: 0,
            bit_count: 0,
        }
    }

    /// KIA V6 keystore A from keystore (type 11)
    fn get_keystore_a() -> u64 {
        keys::get_keystore().get_kia_v6_keystore_a()
    }

    /// KIA V6 keystore B from keystore (type 12)
    fn get_keystore_b() -> u64 {
        keys::get_keystore().get_kia_v6_keystore_b()
    }

    /// CRC8 for V6 (matches kia_v6.c: init 0xFF, polynomial 0x07, over first 15 bytes)
    fn crc8(data: &[u8], init: u8, polynomial: u8) -> u8 {
        let mut crc = init;
        for &byte in data {
            crc ^= byte;
            for _ in 0..8 {
                let b = crc << 1;
                if (crc & 0x80) != 0 {
                    crc = b ^ polynomial;
                } else {
                    crc = b;
                }
            }
        }
        crc
    }

    /// GF(2^8) multiply by 2
    fn gf_mul2(x: u8) -> u8 {
        ((x >> 7).wrapping_mul(0x1b)) ^ (x << 1)
    }

    /// AES inverse SubBytes
    fn aes_subbytes_inv(state: &mut [u8; 16]) {
        for i in 0..16 {
            state[i] = AES_SBOX_INV[state[i] as usize];
        }
    }

    /// AES inverse ShiftRows
    fn aes_shiftrows_inv(state: &mut [u8; 16]) {
        let temp = state[13];
        state[13] = state[9];
        state[9] = state[5];
        state[5] = state[1];
        state[1] = temp;

        let temp = state[2];
        state[2] = state[10];
        state[10] = temp;
        let temp = state[6];
        state[6] = state[14];
        state[14] = temp;

        let temp = state[3];
        state[3] = state[7];
        state[7] = state[11];
        state[11] = state[15];
        state[15] = temp;
    }

    /// AES inverse MixColumns
    fn aes_mixcolumns_inv(state: &mut [u8; 16]) {
        for i in 0..4 {
            let a = state[i * 4];
            let b = state[i * 4 + 1];
            let c = state[i * 4 + 2];
            let d = state[i * 4 + 3];

            let a2 = Self::gf_mul2(a);
            let a4 = Self::gf_mul2(a2);
            let a8 = Self::gf_mul2(a4);
            let b2 = Self::gf_mul2(b);
            let b4 = Self::gf_mul2(b2);
            let b8 = Self::gf_mul2(b4);
            let c2 = Self::gf_mul2(c);
            let c4 = Self::gf_mul2(c2);
            let c8 = Self::gf_mul2(c4);
            let d2 = Self::gf_mul2(d);
            let d4 = Self::gf_mul2(d2);
            let d8 = Self::gf_mul2(d4);

            state[i * 4] = (a8 ^ a4 ^ a2) ^ (b8 ^ b2 ^ b) ^ (c8 ^ c4 ^ c) ^ (d8 ^ d);
            state[i * 4 + 1] = (a8 ^ a) ^ (b8 ^ b4 ^ b2) ^ (c8 ^ c2 ^ c) ^ (d8 ^ d4 ^ d);
            state[i * 4 + 2] = (a8 ^ a4 ^ a) ^ (b8 ^ b) ^ (c8 ^ c4 ^ c2) ^ (d8 ^ d2 ^ d);
            state[i * 4 + 3] = (a8 ^ a2 ^ a) ^ (b8 ^ b4 ^ b) ^ (c8 ^ c) ^ (d8 ^ d4 ^ d2);
        }
    }

    /// AES AddRoundKey
    fn aes_addroundkey(state: &mut [u8; 16], round_key: &[u8]) {
        for i in 0..16 {
            state[i] ^= round_key[i];
        }
    }

    /// AES key expansion
    fn aes_key_expansion(key: &[u8; 16]) -> [u8; 176] {
        let mut round_keys = [0u8; 176];
        round_keys[..16].copy_from_slice(key);

        for i in 4..44 {
            let prev_word_idx = (i - 1) * 4;
            let mut b0 = round_keys[prev_word_idx];
            let mut b1 = round_keys[prev_word_idx + 1];
            let mut b2 = round_keys[prev_word_idx + 2];
            let mut b3 = round_keys[prev_word_idx + 3];

            if (i % 4) == 0 {
                let new_b0 = AES_SBOX[b1 as usize] ^ AES_RCON[(i / 4) - 1];
                let new_b1 = AES_SBOX[b2 as usize];
                let new_b2 = AES_SBOX[b3 as usize];
                let new_b3 = AES_SBOX[b0 as usize];
                b0 = new_b0;
                b1 = new_b1;
                b2 = new_b2;
                b3 = new_b3;
            }

            let back_word_idx = (i - 4) * 4;
            b0 ^= round_keys[back_word_idx];
            b1 ^= round_keys[back_word_idx + 1];
            b2 ^= round_keys[back_word_idx + 2];
            b3 ^= round_keys[back_word_idx + 3];

            let curr_word_idx = i * 4;
            round_keys[curr_word_idx] = b0;
            round_keys[curr_word_idx + 1] = b1;
            round_keys[curr_word_idx + 2] = b2;
            round_keys[curr_word_idx + 3] = b3;
        }

        round_keys
    }

    /// AES-128 decrypt
    fn aes128_decrypt(expanded_key: &[u8; 176], data: &mut [u8; 16]) {
        let mut state = *data;

        Self::aes_addroundkey(&mut state, &expanded_key[160..176]);

        for round in (1..10).rev() {
            Self::aes_shiftrows_inv(&mut state);
            Self::aes_subbytes_inv(&mut state);
            Self::aes_addroundkey(&mut state, &expanded_key[round * 16..(round + 1) * 16]);
            Self::aes_mixcolumns_inv(&mut state);
        }

        Self::aes_shiftrows_inv(&mut state);
        Self::aes_subbytes_inv(&mut state);
        Self::aes_addroundkey(&mut state, &expanded_key[0..16]);

        *data = state;
    }

    /// AES-128 key from V6 keystores A+B with XOR_MASK_LOW/HIGH (matches kia_v6.c)
    fn get_aes_key() -> [u8; 16] {
        let keystore_a = Self::get_keystore_a();
        let keystore_a_hi = ((keystore_a >> 32) & 0xFFFFFFFF) as u32;
        let keystore_a_lo = (keystore_a & 0xFFFFFFFF) as u32;

        let u_var15_a = keystore_a_lo ^ XOR_MASK_LOW;
        let u_var5_a = XOR_MASK_HIGH ^ keystore_a_hi;

        let val64_a = ((u_var5_a as u64) << 32) | (u_var15_a as u64);
        
        let keystore_b = Self::get_keystore_b();
        let keystore_b_hi = ((keystore_b >> 32) & 0xFFFFFFFF) as u32;
        let keystore_b_lo = (keystore_b & 0xFFFFFFFF) as u32;

        let u_var15_b = keystore_b_lo ^ XOR_MASK_LOW;
        let u_var5_b = XOR_MASK_HIGH ^ keystore_b_hi;

        let val64_b = ((u_var5_b as u64) << 32) | (u_var15_b as u64);

        let mut aes_key = [0u8; 16];
        for i in 0..8 {
            aes_key[i] = ((val64_a >> (56 - i * 8)) & 0xFF) as u8;
        }
        for i in 0..8 {
            aes_key[i + 8] = ((val64_b >> (56 - i * 8)) & 0xFF) as u8;
        }

        aes_key
    }

    /// Decrypt 16-byte block: byte layout matches kia_v6.c; AES-128 then CRC8 check
    fn decrypt(&self) -> Option<(u32, u8, u32, bool)> {
        let mut encrypted_data = [0u8; 16];

        encrypted_data[0] = ((self.stored_part1_high >> 8) & 0xFF) as u8;
        encrypted_data[1] = (self.stored_part1_high & 0xFF) as u8;
        encrypted_data[2] = ((self.stored_part1_low >> 24) & 0xFF) as u8;
        encrypted_data[3] = ((self.stored_part1_low >> 16) & 0xFF) as u8;
        encrypted_data[4] = ((self.stored_part1_low >> 8) & 0xFF) as u8;
        encrypted_data[5] = (self.stored_part1_low & 0xFF) as u8;
        encrypted_data[6] = ((self.stored_part2_high >> 24) & 0xFF) as u8;
        encrypted_data[7] = ((self.stored_part2_high >> 16) & 0xFF) as u8;
        encrypted_data[8] = ((self.stored_part2_high >> 8) & 0xFF) as u8;
        encrypted_data[9] = (self.stored_part2_high & 0xFF) as u8;
        encrypted_data[10] = ((self.stored_part2_low >> 24) & 0xFF) as u8;
        encrypted_data[11] = ((self.stored_part2_low >> 16) & 0xFF) as u8;
        encrypted_data[12] = ((self.stored_part2_low >> 8) & 0xFF) as u8;
        encrypted_data[13] = (self.stored_part2_low & 0xFF) as u8;
        encrypted_data[14] = ((self.data_part3 >> 8) & 0xFF) as u8;
        encrypted_data[15] = (self.data_part3 & 0xFF) as u8;

        let aes_key = Self::get_aes_key();
        let expanded_key = Self::aes_key_expansion(&aes_key);

        Self::aes128_decrypt(&expanded_key, &mut encrypted_data);

        let decrypted = &encrypted_data;
        let calculated_crc = Self::crc8(&decrypted[..15], 0xFF, 0x07);
        let stored_crc = decrypted[15];
        let crc_valid = (calculated_crc ^ stored_crc) < 2;

        // Serial: bytes 4-6 as 24-bit big-endian
        let serial = ((decrypted[4] as u32) << 16) | ((decrypted[5] as u32) << 8) | (decrypted[6] as u32);
        let button = decrypted[7];
        let counter = ((decrypted[8] as u32) << 24) |
                     ((decrypted[9] as u32) << 16) |
                     ((decrypted[10] as u32) << 8) |
                     (decrypted[11] as u32);

        Some((serial, button, counter, crc_valid))
    }

    /// Manchester state machine
    /// NOTE: Due to opposite level conventions between Flipper and KAT,
    /// KAT level=true corresponds to Flipper level=false (and vice versa).
    /// For short pulses: protopirate uses (level & 0x7F) << 1, which gives 0/2.
    /// For long pulses: protopirate uses level ? 6 : 4.
    /// With the inverted convention, KAT's is_high=true maps to Flipper level=false.
    fn manchester_advance(&mut self, is_short: bool, is_high: bool) -> Option<bool> {
        let event = match (is_short, is_high) {
            (true, true) => 0,  // Short High (KAT) → Flipper level=false → 0
            (true, false) => 2, // Short Low (KAT) → Flipper level=true → 2
            (false, true) => 4, // Long High (KAT) → Flipper level=false → 4
            (false, false) => 6, // Long Low (KAT) → Flipper level=true → 6
        };

        let (new_state, output) = match (self.manchester_state, event) {
            (ManchesterState::Mid0, 2) | (ManchesterState::Mid1, 2) => 
                (ManchesterState::Start0, None),
            (ManchesterState::Mid0, 0) | (ManchesterState::Mid1, 0) => 
                (ManchesterState::Start1, None),
            
            (ManchesterState::Start1, 2) => (ManchesterState::Mid1, Some(true)),
            (ManchesterState::Start1, 4) => (ManchesterState::Start0, Some(true)),
            
            (ManchesterState::Start0, 0) => (ManchesterState::Mid0, Some(false)),
            (ManchesterState::Start0, 6) => (ManchesterState::Start1, Some(false)),
            
            _ => (ManchesterState::Mid1, None),
        };

        self.manchester_state = new_state;
        output
    }

    /// Add initial sync bits (1,1,0,1 — matches kia_v6.c)
    fn add_sync_bits(&mut self) {
        for bit in [true, true, false, true] {
            let carry = self.data_part1_low >> 31;
            self.data_part1_low = (self.data_part1_low << 1) | (bit as u32);
            self.data_part1_high = (self.data_part1_high << 1) | carry;
            self.bit_count += 1;
        }
    }
}

impl ProtocolDecoder for KiaV6Decoder {
    fn name(&self) -> &'static str {
        "Kia V6"
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
        self.te_last = 0;
        self.header_count = 0;
        self.manchester_state = ManchesterState::Mid1;
        self.data_part1_low = 0;
        self.data_part1_high = 0;
        self.stored_part1_low = 0;
        self.stored_part1_high = 0;
        self.stored_part2_low = 0;
        self.stored_part2_high = 0;
        self.data_part3 = 0;
        self.bit_count = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let is_short = duration_diff!(duration, TE_SHORT) < TE_DELTA;
        let is_long = duration_diff!(duration, TE_LONG) < TE_DELTA;

        match self.step {
            DecoderStep::Reset => {
                if level && is_short {
                    self.step = DecoderStep::WaitFirstHigh;
                    self.te_last = duration;
                    self.header_count = 0;
                    self.manchester_state = ManchesterState::Mid1;
                }
            }

            DecoderStep::WaitFirstHigh => {
                if level {
                    return None;
                }

                let diff_short = duration_diff!(duration, TE_SHORT);
                let diff_long = duration_diff!(duration, TE_LONG);

                if diff_long < TE_DELTA && diff_long < diff_short {
                    if self.header_count >= PREAMBLE_COUNT {
                        self.header_count = 0;
                        self.te_last = duration;
                        self.step = DecoderStep::WaitLongHigh;
                        return None;
                    }
                }

                if diff_short >= TE_DELTA && diff_long >= TE_DELTA {
                    self.step = DecoderStep::Reset;
                    return None;
                }

                if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA {
                    self.te_last = duration;
                    self.header_count += 1;
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::WaitLongHigh => {
                if !level {
                    self.step = DecoderStep::Reset;
                    return None;
                }

                let diff_long = duration_diff!(duration, TE_LONG);
                let diff_short = duration_diff!(duration, TE_SHORT);

                if diff_long >= TE_DELTA && diff_short >= TE_DELTA {
                    self.step = DecoderStep::Reset;
                    return None;
                }

                if duration_diff!(self.te_last, TE_LONG) >= TE_DELTA {
                    self.step = DecoderStep::Reset;
                    return None;
                }

                self.data_part1_low = 0;
                self.data_part1_high = 0;
                self.bit_count = 0;
                self.add_sync_bits();
                self.step = DecoderStep::Data;
            }

            DecoderStep::Data => {
                if !is_short && !is_long {
                    self.step = DecoderStep::Reset;
                    return None;
                }

                if let Some(bit) = self.manchester_advance(is_short, level) {
                    let carry = self.data_part1_low >> 31;
                    self.data_part1_low = (self.data_part1_low << 1) | (bit as u32);
                    self.data_part1_high = (self.data_part1_high << 1) | carry;
                    self.bit_count += 1;

                    if self.bit_count == 64 {
                        self.stored_part1_low = !self.data_part1_low;
                        self.stored_part1_high = !self.data_part1_high;
                        self.data_part1_low = 0;
                        self.data_part1_high = 0;
                    } else if self.bit_count == 128 {
                        self.stored_part2_low = !self.data_part1_low;
                        self.stored_part2_high = !self.data_part1_high;
                        self.data_part1_low = 0;
                        self.data_part1_high = 0;
                    }
                }

                self.te_last = duration;

                if self.bit_count as usize == MIN_COUNT_BIT {
                    self.data_part3 = !(self.data_part1_low as u16);

                    if let Some((serial, button, counter, crc_valid)) = self.decrypt() {
                        let key_data = ((self.stored_part1_high as u64) << 32) |
                                      (self.stored_part1_low as u64);

                        self.step = DecoderStep::Reset;
                        return Some(DecodedSignal {
                            serial: Some(serial),
                            button: Some(button),
                            counter: Some((counter & 0xFFFF) as u16), // V6 has 32-bit counter but we only store 16
                            crc_valid,
                            data: key_data,
                            data_count_bit: MIN_COUNT_BIT,
                            encoder_capable: false,
                        });
                    }

                    self.step = DecoderStep::Reset;
                }
            }
        }

        None
    }

    fn supports_encoding(&self) -> bool {
        false // V6 is decode-only
    }

    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None // V6 decode-only in protopirate
    }
}

impl Default for KiaV6Decoder {
    fn default() -> Self {
        Self::new()
    }
}
