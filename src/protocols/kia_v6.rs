//! Kia V6 decoder + encoder (AES-128).
//!
//! Port of ProtoPirate `protocols/kia_v6.c` (HEAD 2026-10; unchanged in behaviour since KAT's
//! v1.3.0 port). Flipper-ARF's `kia_v6.c` and doctor-firmware's carry the same frame; ARF names
//! the buttons (Lock 1, Unlock 2, Trunk 3, Panic 4), which the encoder's button map uses.
//!
//! Air format (315 / 433.92 MHz, FM): Manchester 200/400µs (te_delta 100). ≥601 short LOWs of
//! preamble, a long LOW + long HIGH sync (= bits 1101), then 144 bits stored inverted: part 1 (64,
//! whose top byte pair carries the Fx field), part 2 (64), part 3 (16). Parts 1..3 minus the Fx
//! nibbles are an AES-128 block (key = KIA V6 keystores A/B, types 11/12, XORed with fixed masks):
//! byte 0 Fx, 4..6 serial, 7 button, 8..11 counter (32 bits), 12 = S-box[counter LSB], 15 CRC-8
//! (poly 0x07, init 0xFF; the C accepts a 1-bit mismatch). The C reports every 144-bit frame and
//! only displays the CRC; `crc_valid` carries its check.
//!
//! Decoder (`kia_protocol_decoder_v6_feed`): Flipper's Manchester machine (common.rs) with the C's
//! event sense `level ? ShortHigh : ShortLow` (`(level & 0x7F) << 1`, `level ? 6 : 4`), reset to
//! Mid1 on entering the preamble; an out-of-tolerance pulse in the data restarts. (KAT's earlier
//! port used a hand-written Manchester table and kept 16 counter bits; on 500 C-encoded frames it
//! reproduced none of the C's frames.)
//! `data` = part 1 (the C's saved Key); `extra` = encoder state: Fx (bits 0..7) and the counter's
//! high 16 bits (bits 8..23) — `counter` holds the low 16 bits of the 32-bit counter.
//!
//! Encoder (`kia_protocol_encoder_v6_build_upload`): AES-encrypt the fields, then the message twice
//! (640- and 38-pair preambles, each: LOW 200, HIGH 400, LOW 200, the 141 inverted bits as
//! Manchester 1 → LOW, HIGH / 0 → HIGH, LOW, then LOW 400); adjacent equal levels merged. One
//! upload per call. Each press sends counter + 1 (32 bits); KAT's generic button maps to Lock 1,
//! Unlock 2, Trunk 3, Panic 4; anything else keeps the captured button.
//!
//! `extra` bits 24..39 (KAT): plaintext byte 0 XOR Fx and stored CRC XOR computed CRC — 0 for the
//! encoder's frames; a real fob may differ (`IMPORTS/KIA/KIA V6 N2.sub`: byte 0 = Fx ^ 1). The
//! encoder ignores them (it builds what the C builds); key files need them to rebuild parts 2/3.
//!
//! Key files (ProtoPirate `kia_protocol_decoder_v6_serialize` / `_deserialize`): "Kia V6", Bit 144,
//! Key = part 1 (one 16-digit word, as the C writes it), then Serial, Btn, Cnt (32 bit), Key_2/Key_3
//! = part 2 low/high, Key_4 = part 3, Fx, decimal. Parts 2/3 are re-encrypted from the decode's fields (KAT does not keep them), so a
//! frame whose remaining plaintext bytes are not the encoder's (bytes 1..3, 13, 14 zero, byte 12 =
//! S-box[counter LSB]) cannot be saved. Loading also reads the older Flipper-ARF layout of the
//! IMPORTS files ("KIA/HYU V6": Key_2/Key_3 = parts 2/3 as hex bytes).

use super::common::{ManchesterEvent, ManchesterState};
use super::{ProtocolDecoder, ProtocolTiming, DecodedSignal, FlipperKey};
use super::keys;
use crate::radio::demodulator::LevelDuration;
use crate::duration_diff;

use super::info::prelude::*;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Kia V6",
    decoder: "Kia V6",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::FM,
    encryption: "AES-128",
    make: "Kia/Hyundai",
    category: Category::Vehicle,
    buttons: Some(super::kia_v1::native_buttons),
    presets: &[&FM476],
};

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

    // =========================================================================
    // Forward AES functions for encoder (matches kia_v6.c ENABLE_EMULATE_FEATURE)
    // =========================================================================

    /// AES forward SubBytes
    fn aes_subbytes(state: &mut [u8; 16]) {
        for i in 0..16 {
            state[i] = AES_SBOX[state[i] as usize];
        }
    }

    /// AES forward ShiftRows
    fn aes_shiftrows(state: &mut [u8; 16]) {
        let temp = state[1];
        state[1] = state[5];
        state[5] = state[9];
        state[9] = state[13];
        state[13] = temp;

        let temp = state[2];
        state[2] = state[10];
        state[10] = temp;
        let temp = state[6];
        state[6] = state[14];
        state[14] = temp;

        let temp = state[3];
        state[3] = state[15];
        state[15] = state[11];
        state[11] = state[7];
        state[7] = temp;
    }

    /// AES forward MixColumns
    fn aes_mixcolumns(state: &mut [u8; 16]) {
        for i in 0..4 {
            let a = state[i * 4];
            let b = state[i * 4 + 1];
            let c = state[i * 4 + 2];
            let d = state[i * 4 + 3];
            state[i * 4]     = Self::gf_mul2(a) ^ Self::gf_mul2(b) ^ b ^ c ^ d;
            state[i * 4 + 1] = a ^ Self::gf_mul2(b) ^ Self::gf_mul2(c) ^ c ^ d;
            state[i * 4 + 2] = a ^ b ^ Self::gf_mul2(c) ^ Self::gf_mul2(d) ^ d;
            state[i * 4 + 3] = Self::gf_mul2(a) ^ a ^ b ^ c ^ Self::gf_mul2(d);
        }
    }

    /// AES-128 encrypt
    fn aes128_encrypt(expanded_key: &[u8; 176], data: &mut [u8; 16]) {
        let mut state = *data;

        Self::aes_addroundkey(&mut state, &expanded_key[0..16]);

        for round in 1..10 {
            Self::aes_subbytes(&mut state);
            Self::aes_shiftrows(&mut state);
            Self::aes_mixcolumns(&mut state);
            Self::aes_addroundkey(&mut state, &expanded_key[round * 16..(round + 1) * 16]);
        }

        Self::aes_subbytes(&mut state);
        Self::aes_shiftrows(&mut state);
        Self::aes_addroundkey(&mut state, &expanded_key[160..176]);

        *data = state;
    }

    /// Encrypt payload for transmission (matches kia_v6.c kia_v6_encrypt_payload)
    fn encrypt_payload(
        fx_field: u8,
        serial: u32,
        button: u8,
        counter: u32,
    ) -> (u32, u32, u32, u32, u16) {
        let mut plain = [0u8; 16];
        plain[0] = fx_field;
        plain[4] = ((serial >> 16) & 0xFF) as u8;
        plain[5] = ((serial >> 8) & 0xFF) as u8;
        plain[6] = (serial & 0xFF) as u8;
        plain[7] = button & 0x0F;
        plain[8] = ((counter >> 24) & 0xFF) as u8;
        plain[9] = ((counter >> 16) & 0xFF) as u8;
        plain[10] = ((counter >> 8) & 0xFF) as u8;
        plain[11] = (counter & 0xFF) as u8;
        plain[12] = AES_SBOX[(counter & 0xFF) as usize];
        plain[15] = Self::crc8(&plain[..15], 0xFF, 0x07);

        let aes_key = Self::get_aes_key();
        let expanded_key = Self::aes_key_expansion(&aes_key);
        Self::aes128_encrypt(&expanded_key, &mut plain);

        let fx_hi = 0x20 | (fx_field >> 4);
        let fx_lo = fx_field & 0x0F;
        let part1_high = ((fx_hi as u32) << 24)
            | ((fx_lo as u32) << 16)
            | ((plain[0] as u32) << 8)
            | (plain[1] as u32);
        let part1_low = ((plain[2] as u32) << 24)
            | ((plain[3] as u32) << 16)
            | ((plain[4] as u32) << 8)
            | (plain[5] as u32);
        let part2_high = ((plain[6] as u32) << 24)
            | ((plain[7] as u32) << 16)
            | ((plain[8] as u32) << 8)
            | (plain[9] as u32);
        let part2_low = ((plain[10] as u32) << 24)
            | ((plain[11] as u32) << 16)
            | ((plain[12] as u32) << 8)
            | (plain[13] as u32);
        let part3 = ((plain[14] as u16) << 8) | (plain[15] as u16);

        (part1_low, part1_high, part2_low, part2_high, part3)
    }

    /// Build encoder signal: two-pass Manchester with preambles (matches kia_v6.c)
    fn build_upload(
        p1_lo: u32, p1_hi: u32,
        p2_lo: u32, p2_hi: u32,
        p3: u16,
    ) -> Vec<LevelDuration> {
        let mut signal = Vec::with_capacity(2000);

        // Two passes: 640 preamble pairs, then 38 preamble pairs
        for &preamble_pairs in &[640u32, 38u32] {
            // Preamble: short/short pairs
            for _ in 0..preamble_pairs {
                signal.push(LevelDuration::new(true, TE_SHORT));
                signal.push(LevelDuration::new(false, TE_SHORT));
            }

            // Sync: short LOW, long HIGH, short LOW
            signal.push(LevelDuration::new(false, TE_SHORT));
            signal.push(LevelDuration::new(true, TE_LONG));
            signal.push(LevelDuration::new(false, TE_SHORT));

            // Part1: bits 60 down to 0 (61 bits), inverted
            for b in (0..=60).rev() {
                let word = if b >= 32 { p1_hi } else { p1_lo };
                let shift = if b >= 32 { b - 32 } else { b };
                let bit = ((!word) >> shift) & 1 == 1;
                Self::encode_manchester_bit(&mut signal, bit);
            }

            // Part2: bits 63 down to 0 (64 bits), inverted
            for b in (0..=63).rev() {
                let word = if b >= 32 { p2_hi } else { p2_lo };
                let shift = if b >= 32 { b - 32 } else { b };
                let bit = ((!word) >> shift) & 1 == 1;
                Self::encode_manchester_bit(&mut signal, bit);
            }

            // Part3: bits 15 down to 0 (16 bits), inverted
            for b in (0..=15).rev() {
                let bit = ((!p3) >> b) & 1 == 1;
                Self::encode_manchester_bit(&mut signal, bit);
            }

            // Gap between passes
            signal.push(LevelDuration::new(false, TE_LONG));
        }

        // Merge adjacent equal levels (identical on air), e.g. the preamble's last LOW and the sync LOW.
        let mut merged: Vec<LevelDuration> = Vec::with_capacity(signal.len());
        for p in signal {
            match merged.last_mut() {
                Some(last) if last.level == p.level => last.duration_us += p.duration_us,
                _ => merged.push(p),
            }
        }
        merged
    }

    /// Encode one Manchester bit (matches kia_v6.c kia_v6_encode_manchester_bit)
    fn encode_manchester_bit(signal: &mut Vec<LevelDuration>, bit: bool) {
        if bit {
            signal.push(LevelDuration::new(false, TE_SHORT));
            signal.push(LevelDuration::new(true, TE_SHORT));
        } else {
            signal.push(LevelDuration::new(true, TE_SHORT));
            signal.push(LevelDuration::new(false, TE_SHORT));
        }
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

    /// Extract fx_field from stored_part1_high (matches kia_v6.c fx_field extraction)
    #[cfg(test)]
    fn extract_fx_field(&self) -> u8 {
        header_fx(((self.stored_part1_high as u64) << 32) | self.stored_part1_low as u64)
    }

    /// Decrypt 16-byte block: byte layout matches kia_v6.c; AES-128 then CRC8 check
    #[cfg(test)]
    fn decrypt(&self) -> Option<(u32, u8, u32, bool)> {
        let p1 = ((self.stored_part1_high as u64) << 32) | self.stored_part1_low as u64;
        let p2 = ((self.stored_part2_high as u64) << 32) | self.stored_part2_low as u64;
        let d = frame_signal(p1, p2, self.data_part3);
        let counter = ((d.extra? >> 8) & 0xFFFF) << 16 | d.counter? as u64;
        Some((d.serial?, d.button?, counter as u32, d.crc_valid))
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
        // SubGhzProtocolFlag_315 | SubGhzProtocolFlag_433
        &[315_000_000, 433_920_000]
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

                // The C's event sense: level ? Short/LongHigh : Short/LongLow.
                if let Some(bit) = self.manchester_state.advance(ManchesterEvent::from_half_bit(level, !is_short)) {
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
                    self.step = DecoderStep::Reset;
                    let p1 = ((self.stored_part1_high as u64) << 32) | self.stored_part1_low as u64;
                    let p2 = ((self.stored_part2_high as u64) << 32) | self.stored_part2_low as u64;
                    return Some(frame_signal(p1, p2, self.data_part3));
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
        let extra = decoded.extra.unwrap_or(0);
        let fx_field = extra as u8;
        let counter = ((((extra >> 8) & 0xFFFF) as u32) << 16 | decoded.counter.unwrap_or(0) as u32)
            .wrapping_add(1);
        let button = match button {
            0x01 => 0x1,
            0x02 => 0x2,
            0x04 => 0x3,
            0x08 => 0x4,
            _ => decoded.button.unwrap_or(0),
        };

        let (p1_lo, p1_hi, p2_lo, p2_hi, p3) =
            Self::encrypt_payload(fx_field, serial, button, counter);

        Some(Self::build_upload(p1_lo, p1_hi, p2_lo, p2_hi, p3))
    }

    /// Key files: `kia_protocol_decoder_v6_deserialize` (Bit 144; Key = part 1, Key_2 = part 2's
    /// low half, Key_3 its high half, Key_4 = part 3, decimal) and the older Flipper-ARF layout of
    /// the IMPORTS files (Key_2 = part 2 and Key_3 = part 3 as hex bytes, which today's references
    /// cannot read); the parts are decrypted as the decoder does. ProtoPirate's encoder transmits
    /// from Serial/Btn/Cnt/Fx (re-encrypting them): when those differ from the decrypted fields,
    /// KAT rebuilds the frame from them the same way.
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != MIN_COUNT_BIT {
            return None;
        }
        let p1 = key.key_u64()?;
        let (p2, p3) = match key.field("Key_2") {
            Some(v) if v.split_whitespace().count() > 1 => {
                (key.hex_field("Key_2")?, key.hex_field("Key_3").unwrap_or(0) as u16)
            }
            _ => {
                let lo = key.u32_field("Key_2").unwrap_or(0) as u64;
                let hi = key.u32_field("Key_3").unwrap_or(0) as u64;
                (hi << 32 | lo, key.u32_field("Key_4").unwrap_or(0) as u16)
            }
        };
        let d = frame_signal(p1, p2, p3);
        let (serial, button, counter, fx) =
            (key.u32_field("Serial"), key.u32_field("Btn"), key.u32_field("Cnt"), key.u32_field("Fx"));
        if serial.is_none() && button.is_none() && counter.is_none() && fx.is_none() {
            return Some(d);
        }
        let extra = d.extra?;
        let now = (d.serial?, d.button?, (((extra >> 8) & 0xFFFF) as u32) << 16 | d.counter? as u32, extra as u8);
        let want = (
            serial.map_or(now.0, |s| s & 0x00FF_FFFF),
            button.map_or(now.1, |b| b as u8),
            counter.unwrap_or(now.2),
            fx.map_or(now.3, |f| f as u8),
        );
        if want == now {
            return Some(d);
        }
        let (lo1, hi1, lo2, hi2, part3) = Self::encrypt_payload(want.3, want.0, want.1 & 0x0F, want.2);
        Some(frame_signal((hi1 as u64) << 32 | lo1 as u64, (hi2 as u64) << 32 | lo2 as u64, part3))
    }

    /// Key files: `kia_protocol_decoder_v6_serialize` — "Kia V6", Bit 144, Key = part 1, then
    /// Serial (24 bit), Btn, Cnt (32 bit), Key_2 / Key_3 = part 2's low / high half, Key_4 = part 3,
    /// Fx (all decimal). KAT's decode keeps part 1 and the decrypted fields, not parts 2 and 3: they
    /// are re-encrypted from the fields (plus the plaintext deviations kept in `extra`, see the
    /// module docs). None when that does not reproduce part 1's cipher bytes (a frame whose other
    /// plaintext bytes are not what the encoder builds). The Key is one 16-digit word, as the C
    /// writes it (ProtoPirate's and doctor-firmware's loaders also read spaced bytes).
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        let extra = decoded.extra?;
        let counter = (((extra >> 8) & 0xFFFF) as u32) << 16 | decoded.counter? as u32;
        let (serial, button) = (decoded.serial?, decoded.button?);
        let mut plain = [0u8; 16];
        plain[0] = extra as u8 ^ (extra >> 24) as u8;
        plain[4..7].copy_from_slice(&serial.to_be_bytes()[1..]);
        plain[7] = button;
        plain[8..12].copy_from_slice(&counter.to_be_bytes());
        plain[12] = AES_SBOX[(counter & 0xFF) as usize];
        plain[15] = Self::crc8(&plain[..15], 0xFF, 0x07) ^ (extra >> 32) as u8;
        let expanded_key = Self::aes_key_expansion(&Self::get_aes_key());
        Self::aes128_encrypt(&expanded_key, &mut plain);
        if plain[..6] != decoded.data.to_be_bytes()[2..] {
            return None;
        }
        let part2 = u64::from_be_bytes(plain[6..14].try_into().unwrap());
        let part3 = u16::from_be_bytes([plain[14], plain[15]]);
        let mut key = FlipperKey::for_protocol("Kia V6", MIN_COUNT_BIT, decoded.data);
        key.key_word = true;
        key.push_u32("Serial", serial);
        key.push_u32("Btn", button.into());
        key.push_u32("Cnt", counter);
        key.push_u32("Key_2", part2 as u32);
        key.push_u32("Key_3", (part2 >> 32) as u32);
        key.push_u32("Key_4", part3.into());
        key.push_u32("Fx", extra as u8 as u32);
        Some(key)
    }
}

/// The Fx field carried in part 1's header nibbles (`kia_v6_decrypt`).
fn header_fx(part1: u64) -> u8 {
    let hi = (part1 >> 32) as u32;
    ((((hi >> 24) & 0xF) as u8) << 4) | ((hi >> 16) & 0xF) as u8
}

/// The decode of a 144-bit frame (parts 1..3 as stored, i.e. un-inverted): AES-decrypt part 1's
/// low 6 bytes + part 2 + part 3 (`kia_v6_decrypt`) and report serial (bytes 4..6), button (byte
/// 7), counter (bytes 8..11; low 16 bits in `counter`, high 16 in `extra` bits 8..23), Fx (part 1's
/// header nibbles, `extra` bits 0..7) and the CRC check (1-bit tolerance, as the C). `extra` bits
/// 24..31 / 32..39 keep how plaintext byte 0 / the stored CRC differ from Fx / the computed CRC
/// (0 for the encoder's frames), so a key file can rebuild parts 2 and 3 exactly.
fn frame_signal(p1: u64, p2: u64, p3: u16) -> DecodedSignal {
    let mut block = [0u8; 16];
    block[..6].copy_from_slice(&p1.to_be_bytes()[2..]);
    block[6..14].copy_from_slice(&p2.to_be_bytes());
    block[14..].copy_from_slice(&p3.to_be_bytes());
    let expanded_key = KiaV6Decoder::aes_key_expansion(&KiaV6Decoder::get_aes_key());
    KiaV6Decoder::aes128_decrypt(&expanded_key, &mut block);
    let calculated_crc = KiaV6Decoder::crc8(&block[..15], 0xFF, 0x07);
    let fx = header_fx(p1);
    let serial = u32::from_be_bytes([0, block[4], block[5], block[6]]);
    let counter = u32::from_be_bytes([block[8], block[9], block[10], block[11]]);
    let extra = fx as u64
        | ((counter >> 16) as u64) << 8
        | ((block[0] ^ fx) as u64) << 24
        | ((block[15] ^ calculated_crc) as u64) << 32;
    DecodedSignal {
        serial: Some(serial),
        button: Some(block[7]),
        counter: Some(counter as u16),
        crc_valid: (calculated_crc ^ block[15]) < 2,
        data: p1,
        data_count_bit: MIN_COUNT_BIT,
        encoder_capable: true,
        extra: Some(extra),
        protocol_display_name: None,
    }
}

impl Default for KiaV6Decoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn feed_all(wire: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = KiaV6Decoder::new();
        wire.iter().filter_map(|p| dec.feed(p.level, p.duration_us)).collect()
    }

    /// Golden frames from the C (`kia_v6.c` on the host, KAT's keystore A/B): Serial/Btn/Cnt/Fx
    /// in; its decoder reports part 1 of the first message and the CRC fields out.
    #[rustfmt::skip]
    const C_VECTORS: &[(u32, u8, u32, u8, u64, u8, u8)] = &[];

    #[test]
    fn c_vectors() {
        keys::load_keystore_from_embedded();
        for &(serial, button, counter, fx, part1, crc1, crc2) in C_VECTORS {
            let (p1_lo, p1_hi, p2_lo, p2_hi, p3) = KiaV6Decoder::encrypt_payload(fx, serial, button, counter);
            assert_eq!(((p1_hi as u64) << 32) | p1_lo as u64, part1);
            let got = feed_all(&KiaV6Decoder::build_upload(p1_lo, p1_hi, p2_lo, p2_hi, p3));
            assert_eq!(got.len(), 1, "the 38-pair second message is below the 601-pair preamble gate");
            let d = &got[0];
            assert_eq!((d.serial, d.button, d.counter), (Some(serial), Some(button), Some(counter as u16)));
            assert_eq!(d.extra, Some(fx as u64 | ((counter >> 16) as u64) << 8));
            assert_eq!(d.data, part1);
            assert!(d.crc_valid);
            let _ = (crc1, crc2);
        }
    }

    /// encode(): the full 32-bit counter + 1 (high half from `extra`), Fx kept, button mapped.
    #[test]
    fn encode_bumps_32_bit_counter_and_maps_buttons() {
        keys::load_keystore_from_embedded();
        for (generic, want) in [(0x01u8, 0x1u8), (0x02, 0x2), (0x04, 0x3), (0x08, 0x4), (0x40, 0x7)] {
            let src = DecodedSignal {
                serial: Some(0x00D2_6B94),
                button: Some(0x7),
                counter: Some(0xFFFF),
                crc_valid: true,
                data: 0,
                data_count_bit: MIN_COUNT_BIT,
                encoder_capable: true,
                extra: Some(0x25 | (0x42F9 << 8)),
                protocol_display_name: None,
            };
            let got = feed_all(&KiaV6Decoder::new().encode(&src, generic).unwrap());
            let d = &got[0];
            assert_eq!((d.serial, d.button, d.counter), (Some(0x00D2_6B94), Some(want), Some(0x0000)));
            assert_eq!(d.extra, Some(0x25 | (0x42FA << 8)), "0x42F9FFFF + 1 carries into the high half");
            assert!(d.crc_valid);
        }
    }

    /// Real fob data: `IMPORTS/HYUNDAI/HYU V6_test.sub` (Flipper-ARF "KIA/HYU V6": Key = part 1,
    /// Key_2 = part 2, Key_3 = part 3) decrypts with a valid CRC, and re-encrypting its fields gives
    /// the same parts 2 and 3 and the same transmitted part-1 bits (60..0; the top three are not
    /// sent). (`KIA V6 N2.sub` decrypts too, but its plaintext byte 0 differs from the header Fx in
    /// the LSB — the C's 1-bit CRC tolerance — so the C encoder cannot rebuild it bit for bit.)
    #[test]
    fn real_key_file_rebuilds() {
        keys::load_keystore_from_embedded();
        let (p1, p2, p3) = (0xED02_8AE7_51F0_ECAEu64, 0xDA18_1151_A0E5_669Du64, 0x0E02u16);
        let mut d = KiaV6Decoder::new();
        d.stored_part1_high = (p1 >> 32) as u32;
        d.stored_part1_low = p1 as u32;
        d.stored_part2_high = (p2 >> 32) as u32;
        d.stored_part2_low = p2 as u32;
        d.data_part3 = p3;
        let (serial, button, counter, crc_ok) = d.decrypt().unwrap();
        assert_eq!((serial, button, counter, crc_ok), (0x1A_D9C3, 0x01, 0x2012, true));
        let fx = d.extract_fx_field();
        assert_eq!(fx, 0xD2);
        let (lo1, hi1, lo2, hi2, part3) = KiaV6Decoder::encrypt_payload(fx, serial, button, counter);
        assert_eq!((((hi1 as u64) << 32) | lo1 as u64) & ((1 << 61) - 1), p1 & ((1 << 61) - 1));
        assert_eq!((((hi2 as u64) << 32) | lo2 as u64, part3), (p2, p3));
    }

    /// Key files: encoder frames round-trip with ProtoPirate's lines; both IMPORTS key files (older
    /// Flipper-ARF layout) load, and — with the plaintext deviations in `extra` — re-save to the same
    /// parts and reload identically; edited fields rebuild the frame; TX from a loaded key decodes.
    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::{assert_round_trip, key};
        keys::load_keystore_from_embedded();
        let dec = KiaV6Decoder::new();
        let mut first = None;
        for (serial, button, counter, fx) in [(0x00D2_6B94u32, 0x1u8, 0x42F9_0001u32, 0x25u8), (0x001A_D9C3, 0x2, 0x2012, 0xD2)] {
            let (p1_lo, p1_hi, p2_lo, p2_hi, p3) = KiaV6Decoder::encrypt_payload(fx, serial, button, counter);
            let d = feed_all(&KiaV6Decoder::build_upload(p1_lo, p1_hi, p2_lo, p2_hi, p3)).remove(0);
            assert_eq!(d.extra.unwrap() >> 24, 0, "encoder frames have no plaintext deviation");
            let text = assert_round_trip(&dec, &d);
            assert!(text.contains(&format!("\nKey_2: {p2_lo}\nKey_3: {p2_hi}\nKey_4: {p3}\nFx: {fx}\n")), "{text}");
            first.get_or_insert((d, text));
        }
        let (d, text) = first.unwrap();
        assert!(text.contains("\nProtocol: Kia V6\nBit: 144\n"), "{text}");
        assert!(text.contains("\nSerial: 13790100\nBtn: 1\nCnt: 1123614721\nKey_2: "), "{text}");
        // IMPORTS/HYUNDAI/HYU V6_test.sub and KIA/KIA V6 N2.sub.
        for (k, k2, k3, serial, button, counter) in [
            ("ED 02 8A E7 51 F0 EC AE", "DA 18 11 51 A0 E5 66 9D", "00 00 0E 02", 0x1A_D9C3, 0x01, 0x2012u32),
            ("DA 07 82 7E FC A4 EA 82", "A7 25 6C 30 73 33 1F 84", "00 00 2B F7", 0x10_9DA7, 0x0B, 0xC106_C04B),
        ] {
            let file = format!(
                "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                 Preset: FuriHalSubGhzPreset2FSKDev476Async\nProtocol: KIA/HYU V6\nBit: 144\nKey: {k}\n\
                 Key_2: {k2}\nKey_3: {k3}\n"
            );
            let l = dec.from_flipper_key(&key(&file)).unwrap();
            let full = ((l.extra.unwrap() >> 8) & 0xFFFF) << 16 | l.counter.unwrap() as u64;
            assert_eq!((l.serial, l.button, full as u32, l.crc_valid), (Some(serial), Some(button), counter, true));
            assert_eq!(l.data, key(&file).key);
            // Saved by KAT (ProtoPirate's decimal layout) and reloaded: identical.
            let resaved = assert_round_trip(&dec, &l);
            let (p2, p3) = (key(&file).hex_field("Key_2").unwrap(), key(&file).hex_field("Key_3").unwrap());
            assert!(resaved.contains(&format!("\nKey_2: {}\nKey_3: {}\nKey_4: {}\n", p2 as u32, p2 >> 32, p3)), "{resaved}");
            // TX from the loaded key decodes back with counter + 1.
            let tx = feed_all(&dec.encode(&l, 0x02).unwrap());
            assert_eq!((tx[0].serial, tx[0].button, tx[0].counter), (Some(serial), Some(0x2), Some((counter + 1) as u16)));
        }
        // KIA V6 N2's plaintext byte 0 is Fx ^ 1.
        let n2 = dec.from_flipper_key(&key(
            "Protocol: KIA/HYU V6\nBit: 144\nKey: DA 07 82 7E FC A4 EA 82\nKey_2: A7 25 6C 30 73 33 1F 84\nKey_3: 00 00 2B F7\n",
        ));
        assert_eq!(n2.unwrap().extra.unwrap() >> 24, 0x01);
        // Edited Cnt: re-encrypted like ProtoPirate's encoder; other Bits fail.
        let e = dec.from_flipper_key(&key(&text.replace("Cnt: 1123614721", "Cnt: 1123614800"))).unwrap();
        assert_eq!((e.serial, e.button, e.counter, e.crc_valid), (d.serial, d.button, Some(0x0050), true));
        assert!(dec.from_flipper_key(&key(&text.replace("Bit: 144", "Bit: 143"))).is_none());
    }

    /// Oracle vectors (`#[ignore]`d): `KAT_KEY_VECTORS=<dir> cargo test kia_v6::tests::key_vectors
    /// -- --ignored` writes `kia_v6_<n>.sub` / `.kat.txt`.
    #[test]
    #[ignore]
    fn key_vectors() {
        let Some(dir) = std::env::var_os("KAT_KEY_VECTORS") else { return };
        let dir = std::path::PathBuf::from(dir);
        keys::load_keystore_from_embedded();
        for (n, (serial, button, counter, fx)) in [(0x00D2_6B94u32, 0x1u8, 0x42F9_0001u32, 0x25u8), (0x001A_D9C3, 0x2, 0x2012, 0xD2)].into_iter().enumerate() {
            let (p1_lo, p1_hi, p2_lo, p2_hi, p3) = KiaV6Decoder::encrypt_payload(fx, serial, button, counter);
            let upload = KiaV6Decoder::build_upload(p1_lo, p1_hi, p2_lo, p2_hi, p3);
            let d = feed_all(&upload).remove(0);
            let key = KiaV6Decoder::new().to_flipper_key(&d).unwrap();
            crate::protocols::kia_v1::tests::write_key_vector(
                &dir,
                &format!("kia_v6_{n}"),
                433_920_000,
                "FuriHalSubGhzPreset2FSKDev476Async",
                &upload,
                &key,
            );
        }
    }
}
