//! AUT64 block cipher implementation
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/aut64.c`.
//! Encrypt/decrypt, pack/unpack, and all tables match the reference.
//!
//! AUT64 algorithm: 12 rounds, 8-byte block/key size.
//! See: https://www.usenix.org/system/files/conference/usenixsecurity16/sec16_paper_garcia.pdf

pub const AUT64_NUM_ROUNDS: usize = 12;
pub const AUT64_BLOCK_SIZE: usize = 8;
pub const AUT64_KEY_SIZE: usize = 8;
pub const AUT64_PBOX_SIZE: usize = 8;
pub const AUT64_SBOX_SIZE: usize = 16;
#[allow(dead_code)]
pub const AUT64_KEY_STRUCT_PACKED_SIZE: usize = 16;

/// AUT64 key structure
#[derive(Debug, Clone)]
pub struct Aut64Key {
    pub index: u8,
    pub key: [u8; AUT64_KEY_SIZE],
    pub pbox: [u8; AUT64_PBOX_SIZE],
    pub sbox: [u8; AUT64_SBOX_SIZE],
}

impl Default for Aut64Key {
    fn default() -> Self {
        Self {
            index: 0,
            key: [0u8; AUT64_KEY_SIZE],
            pbox: [0u8; AUT64_PBOX_SIZE],
            sbox: [0u8; AUT64_SBOX_SIZE],
        }
    }
}

/// Round-dependent upper-nibble lookup table
static TABLE_LN: [[u8; 8]; AUT64_NUM_ROUNDS] = [
    [0x4, 0x5, 0x6, 0x7, 0x0, 0x1, 0x2, 0x3], // Round 0
    [0x5, 0x4, 0x7, 0x6, 0x1, 0x0, 0x3, 0x2], // Round 1
    [0x6, 0x7, 0x4, 0x5, 0x2, 0x3, 0x0, 0x1], // Round 2
    [0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x1, 0x0], // Round 3
    [0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7], // Round 4
    [0x1, 0x0, 0x3, 0x2, 0x5, 0x4, 0x7, 0x6], // Round 5
    [0x2, 0x3, 0x0, 0x1, 0x6, 0x7, 0x4, 0x5], // Round 6
    [0x3, 0x2, 0x1, 0x0, 0x7, 0x6, 0x5, 0x4], // Round 7
    [0x5, 0x4, 0x7, 0x6, 0x1, 0x0, 0x3, 0x2], // Round 8
    [0x4, 0x5, 0x6, 0x7, 0x0, 0x1, 0x2, 0x3], // Round 9
    [0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x1, 0x0], // Round 10
    [0x6, 0x7, 0x4, 0x5, 0x2, 0x3, 0x0, 0x1], // Round 11
];

/// Round-dependent lower-nibble lookup table
static TABLE_UN: [[u8; 8]; AUT64_NUM_ROUNDS] = [
    [0x1, 0x0, 0x3, 0x2, 0x5, 0x4, 0x7, 0x6], // Round 0
    [0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7], // Round 1
    [0x3, 0x2, 0x1, 0x0, 0x7, 0x6, 0x5, 0x4], // Round 2
    [0x2, 0x3, 0x0, 0x1, 0x6, 0x7, 0x4, 0x5], // Round 3
    [0x5, 0x4, 0x7, 0x6, 0x1, 0x0, 0x3, 0x2], // Round 4
    [0x4, 0x5, 0x6, 0x7, 0x0, 0x1, 0x2, 0x3], // Round 5
    [0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x1, 0x0], // Round 6
    [0x6, 0x7, 0x4, 0x5, 0x2, 0x3, 0x0, 0x1], // Round 7
    [0x3, 0x2, 0x1, 0x0, 0x7, 0x6, 0x5, 0x4], // Round 8
    [0x2, 0x3, 0x0, 0x1, 0x6, 0x7, 0x4, 0x5], // Round 9
    [0x1, 0x0, 0x3, 0x2, 0x5, 0x4, 0x7, 0x6], // Round 10
    [0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7], // Round 11
];

/// GF(2^4) multiplication table (nibble offset table)
#[rustfmt::skip]
static TABLE_OFFSET: [u8; 256] = [
    // 0    1    2    3    4    5    6    7    8    9    A    B    C    D    E    F
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, // 0
    0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xA, 0xB, 0xC, 0xD, 0xE, 0xF, // 1
    0x0, 0x2, 0x4, 0x6, 0x8, 0xA, 0xC, 0xE, 0x3, 0x1, 0x7, 0x5, 0xB, 0x9, 0xF, 0xD, // 2
    0x0, 0x3, 0x6, 0x5, 0xC, 0xF, 0xA, 0x9, 0xB, 0x8, 0xD, 0xE, 0x7, 0x4, 0x1, 0x2, // 3
    0x0, 0x4, 0x8, 0xC, 0x3, 0x7, 0xB, 0xF, 0x6, 0x2, 0xE, 0xA, 0x5, 0x1, 0xD, 0x9, // 4
    0x0, 0x5, 0xA, 0xF, 0x7, 0x2, 0xD, 0x8, 0xE, 0xB, 0x4, 0x1, 0x9, 0xC, 0x3, 0x6, // 5
    0x0, 0x6, 0xC, 0xA, 0xB, 0xD, 0x7, 0x1, 0x5, 0x3, 0x9, 0xF, 0xE, 0x8, 0x2, 0x4, // 6
    0x0, 0x7, 0xE, 0x9, 0xF, 0x8, 0x1, 0x6, 0xD, 0xA, 0x3, 0x4, 0x2, 0x5, 0xC, 0xB, // 7
    0x0, 0x8, 0x3, 0xB, 0x6, 0xE, 0x5, 0xD, 0xC, 0x4, 0xF, 0x7, 0xA, 0x2, 0x9, 0x1, // 8
    0x0, 0x9, 0x1, 0x8, 0x2, 0xB, 0x3, 0xA, 0x4, 0xD, 0x5, 0xC, 0x6, 0xF, 0x7, 0xE, // 9
    0x0, 0xA, 0x7, 0xD, 0xE, 0x4, 0x9, 0x3, 0xF, 0x5, 0x8, 0x2, 0x1, 0xB, 0x6, 0xC, // A
    0x0, 0xB, 0x5, 0xE, 0xA, 0x1, 0xF, 0x4, 0x7, 0xC, 0x2, 0x9, 0xD, 0x6, 0x8, 0x3, // B
    0x0, 0xC, 0xB, 0x7, 0x5, 0x9, 0xE, 0x2, 0xA, 0x6, 0x1, 0xD, 0xF, 0x3, 0x4, 0x8, // C
    0x0, 0xD, 0x9, 0x4, 0x1, 0xC, 0x8, 0x5, 0x2, 0xF, 0xB, 0x6, 0x3, 0xE, 0xA, 0x7, // D
    0x0, 0xE, 0xF, 0x1, 0xD, 0x3, 0x2, 0xC, 0x9, 0x7, 0x6, 0x8, 0x4, 0xA, 0xB, 0x5, // E
    0x0, 0xF, 0xD, 0x2, 0x9, 0x6, 0x4, 0xB, 0x1, 0xE, 0xC, 0x3, 0x8, 0x7, 0x5, 0xA, // F
];

/// S-box substitution table
static TABLE_SUB: [u8; 16] = [
    0x0, 0x1, 0x9, 0xE, 0xD, 0xB, 0x7, 0x6,
    0xF, 0x2, 0xC, 0x5, 0xA, 0x4, 0x3, 0x8,
];

/// Key nibble operation: apply key-dependent GF offset
fn key_nibble(key: &Aut64Key, nibble: u8, table: &[u8; 8], iteration: usize) -> u8 {
    let key_value = key.key[table[iteration] as usize];
    let offset = ((key_value as usize) << 4) | (nibble as usize);
    TABLE_OFFSET[offset]
}

/// Compute round key from state
fn round_key(key: &Aut64Key, state: &[u8], round_n: usize) -> u8 {
    let mut result_hi: u8 = 0;
    let mut result_lo: u8 = 0;

    for i in 0..(AUT64_BLOCK_SIZE - 1) {
        result_hi ^= key_nibble(key, state[i] >> 4, &TABLE_UN[round_n], i);
        result_lo ^= key_nibble(key, state[i] & 0x0F, &TABLE_LN[round_n], i);
    }

    (result_hi << 4) | result_lo
}

/// Final byte nibble for key schedule
fn final_byte_nibble(key: &Aut64Key, table: &[u8; 8]) -> u8 {
    let key_value = key.key[table[AUT64_BLOCK_SIZE - 1] as usize];
    TABLE_SUB[key_value as usize] << 4
}

/// Encrypt final byte nibble (inverse S-box lookup through offset table)
fn encrypt_final_byte_nibble(key: &Aut64Key, nibble: u8, table: &[u8; 8]) -> u8 {
    let offset = final_byte_nibble(key, table) as usize;

    for i in 0u8..16 {
        if TABLE_OFFSET[offset + i as usize] == nibble {
            return i;
        }
    }
    0 // Should not reach here for valid inputs
}

/// Encrypt compress: compute encrypted output byte for a round
fn encrypt_compress(key: &Aut64Key, state: &[u8], round_n: usize) -> u8 {
    let round_k = round_key(key, state, round_n);
    let mut result_hi = round_k >> 4;
    let mut result_lo = round_k & 0x0F;

    result_hi ^= encrypt_final_byte_nibble(key, state[AUT64_BLOCK_SIZE - 1] >> 4, &TABLE_UN[round_n]);
    result_lo ^= encrypt_final_byte_nibble(key, state[AUT64_BLOCK_SIZE - 1] & 0x0F, &TABLE_LN[round_n]);

    (result_hi << 4) | result_lo
}

/// Decrypt final byte nibble (forward S-box through offset table)
fn decrypt_final_byte_nibble(key: &Aut64Key, nibble: u8, table: &[u8; 8], result: u8) -> u8 {
    let offset = final_byte_nibble(key, table) as usize;
    TABLE_OFFSET[(result ^ nibble) as usize + offset]
}

/// Decrypt compress: compute decrypted output byte for a round
fn decrypt_compress(key: &Aut64Key, state: &[u8], round_n: usize) -> u8 {
    let round_k = round_key(key, state, round_n);
    let result_hi = round_k >> 4;
    let result_lo = round_k & 0x0F;

    let hi = decrypt_final_byte_nibble(
        key,
        state[AUT64_BLOCK_SIZE - 1] >> 4,
        &TABLE_UN[round_n],
        result_hi,
    );
    let lo = decrypt_final_byte_nibble(
        key,
        state[AUT64_BLOCK_SIZE - 1] & 0x0F,
        &TABLE_LN[round_n],
        result_lo,
    );

    (hi << 4) | lo
}

/// S-box substitution on a full byte (applies S-box to each nibble independently)
fn substitute(key: &Aut64Key, byte: u8) -> u8 {
    (key.sbox[(byte >> 4) as usize] << 4) | key.sbox[(byte & 0x0F) as usize]
}

/// Byte-level permutation using P-box
fn permute_bytes(key: &Aut64Key, state: &mut [u8]) {
    let mut result = [0u8; AUT64_PBOX_SIZE];
    for i in 0..AUT64_PBOX_SIZE {
        result[key.pbox[i] as usize] = state[i];
    }
    state[..AUT64_PBOX_SIZE].copy_from_slice(&result);
}

/// Bit-level permutation using P-box
fn permute_bits(key: &Aut64Key, byte: u8) -> u8 {
    let mut result: u8 = 0;
    for i in 0..8 {
        if byte & (1 << i) != 0 {
            result |= 1 << key.pbox[i];
        }
    }
    result
}

/// Compute inverse permutation box
fn reverse_box(box_in: &[u8], len: usize) -> Vec<u8> {
    let mut reversed = vec![0u8; len];
    for i in 0..len {
        for j in 0..len {
            if box_in[j] == i as u8 {
                reversed[i] = j as u8;
                break;
            }
        }
    }
    reversed
}

/// AUT64 encrypt: 12 rounds of the cipher
pub fn aut64_encrypt(key: &Aut64Key, message: &mut [u8]) {
    // Create reverse key for encryption
    let mut reverse_key = key.clone();
    let rev_pbox = reverse_box(&key.pbox, AUT64_PBOX_SIZE);
    let rev_sbox = reverse_box(&key.sbox, AUT64_SBOX_SIZE);
    reverse_key.pbox.copy_from_slice(&rev_pbox);
    reverse_key.sbox.copy_from_slice(&rev_sbox);

    for i in 0..AUT64_NUM_ROUNDS {
        permute_bytes(&reverse_key, message);
        message[7] = encrypt_compress(&reverse_key, message, i);
        message[7] = substitute(&reverse_key, message[7]);
        message[7] = permute_bits(&reverse_key, message[7]);
        message[7] = substitute(&reverse_key, message[7]);
    }
}

/// AUT64 decrypt: 12 rounds of the cipher (reverse order)
pub fn aut64_decrypt(key: &Aut64Key, message: &mut [u8]) {
    for i in (0..AUT64_NUM_ROUNDS).rev() {
        message[7] = substitute(key, message[7]);
        message[7] = permute_bits(key, message[7]);
        message[7] = substitute(key, message[7]);
        message[7] = decrypt_compress(key, message, i);
        permute_bytes(key, message);
    }
}

/// Pack an AUT64 key structure into a 16-byte array
#[allow(dead_code)]
pub fn aut64_pack(src: &Aut64Key) -> [u8; AUT64_KEY_STRUCT_PACKED_SIZE] {
    let mut dest = [0u8; AUT64_KEY_STRUCT_PACKED_SIZE];
    dest[0] = src.index;

    for i in 0..(src.key.len() / 2) {
        dest[i + 1] = (src.key[i * 2] << 4) | src.key[i * 2 + 1];
    }

    let mut pbox: u32 = 0;
    for i in 0..src.pbox.len() {
        pbox = (pbox << 3) | src.pbox[i] as u32;
    }
    dest[5] = (pbox >> 16) as u8;
    dest[6] = ((pbox >> 8) & 0xFF) as u8;
    dest[7] = (pbox & 0xFF) as u8;

    for i in 0..(src.sbox.len() / 2) {
        dest[i + 8] = (src.sbox[i * 2] << 4) | src.sbox[i * 2 + 1];
    }

    dest
}

/// Unpack a 16-byte array into an AUT64 key structure (matches aut64_unpack in reference)
#[allow(dead_code)]
pub fn aut64_unpack(src: &[u8]) -> Aut64Key {
    let mut dest = Aut64Key::default();
    dest.index = src[0];

    for i in 0..(dest.key.len() / 2) {
        dest.key[i * 2] = src[i + 1] >> 4;
        dest.key[i * 2 + 1] = src[i + 1] & 0xF;
    }

    let mut pbox: u32 = (u32::from(src[5]) << 16) | (u32::from(src[6]) << 8) | u32::from(src[7]);
    for i in (0..dest.pbox.len()).rev() {
        dest.pbox[i] = (pbox & 0x7) as u8;
        pbox >>= 3;
    }

    for i in 0..(dest.sbox.len() / 2) {
        dest.sbox[i * 2] = src[i + 8] >> 4;
        dest.sbox[i * 2 + 1] = src[i + 8] & 0xF;
    }

    dest
}
