//! KeeLoq common encryption/decryption routines
//!
//! Shared by Kia V3/V4, Star Line, and other KeeLoq-based protocols.
//! Based on the NLF (Non-Linear Feedback) function with constant 0x3A5C742E.

/// The KeeLoq NLF constant
const KEELOQ_NLF: u32 = 0x3A5C742E;

/// KeeLoq decrypt: 528 rounds of the KeeLoq cipher (decrypt direction)
pub fn keeloq_decrypt(data: u32, key: u64) -> u32 {
    let mut block = data;
    let mut tkey = key;

    for _ in 0..528 {
        let lutkey = ((block >> 0) & 1)
            | ((block >> 7) & 2)
            | ((block >> 17) & 4)
            | ((block >> 22) & 8)
            | ((block >> 26) & 16);
        let lsb = ((block >> 31)
            ^ ((block >> 15) & 1)
            ^ ((KEELOQ_NLF >> lutkey) & 1)
            ^ (((tkey >> 15) & 1) as u32)) as u32;
        block = ((block & 0x7FFFFFFF) << 1) | lsb;
        tkey = ((tkey & 0x7FFFFFFFFFFFFFFF) << 1) | (tkey >> 63);
    }
    block
}

/// KeeLoq encrypt: 528 rounds of the KeeLoq cipher (encrypt direction)
pub fn keeloq_encrypt(data: u32, key: u64) -> u32 {
    let mut block = data;
    let mut tkey = key;

    for _ in 0..528 {
        let lutkey = ((block >> 1) & 1)
            | ((block >> 8) & 2)
            | ((block >> 18) & 4)
            | ((block >> 23) & 8)
            | ((block >> 27) & 16);
        let msb = ((block >> 0)
            ^ ((block >> 16) & 1)
            ^ ((KEELOQ_NLF >> lutkey) & 1)
            ^ (((tkey >> 0) & 1) as u32)) as u32;
        block = ((block >> 1) & 0x7FFFFFFF) | (msb << 31);
        tkey = ((tkey >> 1) & 0x7FFFFFFFFFFFFFFF) | ((tkey & 1) << 63);
    }
    block
}

/// Normal learning key derivation
/// Derives a 64-bit key from a 32-bit fix code and a 64-bit manufacturer key
pub fn keeloq_normal_learning(fix: u32, manufacturer_key: u64) -> u64 {
    let serial_low = fix & 0xFFFF;
    let serial_high = (fix >> 16) & 0xFFFF;

    let key_low = keeloq_decrypt(serial_low as u32 | 0x20000000, manufacturer_key);
    let key_high = keeloq_decrypt(serial_high as u32 | 0x60000000, manufacturer_key);

    ((key_high as u64) << 32) | (key_low as u64)
}

/// Reverse the bits in a 64-bit key (for protocols that store data MSB-first)
pub fn reverse_key(key: u64, bit_count: usize) -> u64 {
    let mut result: u64 = 0;
    for i in 0..bit_count {
        if (key >> i) & 1 == 1 {
            result |= 1 << (bit_count - 1 - i);
        }
    }
    result
}

/// Reverse bits in a byte
#[allow(dead_code)]
pub fn reverse8(byte: u8) -> u8 {
    let mut b = byte;
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    b
}

/// Secure learning key derivation
/// Derives a 64-bit key from a serial, seed, and manufacturer key
#[allow(dead_code)]
pub fn keeloq_secure_learning(data: u32, seed: u32, key: u64) -> u64 {
    let serial = data & 0x0FFFFFFF;
    let k1 = keeloq_decrypt(serial, key);
    let k2 = keeloq_decrypt(seed, key);
    ((k1 as u64) << 32) | (k2 as u64)
}

/// FAAC SLH (Spa) learning key derivation
/// Derives a 64-bit key from a seed and manufacturer key
#[allow(dead_code)]
pub fn keeloq_faac_learning(seed: u32, key: u64) -> u64 {
    let hs = (seed >> 16) as u16;
    let ending: u16 = 0x544D;
    let lsb = ((hs as u32) << 16) | (ending as u32);
    ((keeloq_encrypt(seed, key) as u64) << 32) | (keeloq_encrypt(lsb, key) as u64)
}

/// Magic XOR Type 1 learning key derivation
#[allow(dead_code)]
pub fn keeloq_magic_xor_type1_learning(data: u32, xor: u64) -> u64 {
    let serial = data & 0x0FFFFFFF;
    (((serial as u64) << 32) | (serial as u64)) ^ xor
}

/// Magic Serial Type 1 learning key derivation
#[allow(dead_code)]
pub fn keeloq_magic_serial_type1_learning(data: u32, man: u64) -> u64 {
    (man & 0xFFFFFFFF)
        | ((data as u64) << 40)
        | (((((data & 0xFF).wrapping_add((data >> 8) & 0xFF)) & 0xFF) as u64) << 32)
}

/// Magic Serial Type 2 learning key derivation
#[allow(dead_code)]
pub fn keeloq_magic_serial_type2_learning(data: u32, man: u64) -> u64 {
    let p = data.to_le_bytes();
    let mut m = man.to_le_bytes();
    m[7] = p[0];
    m[6] = p[1];
    m[5] = p[2];
    m[4] = p[3];
    u64::from_le_bytes(m)
}

/// Magic Serial Type 3 learning key derivation
#[allow(dead_code)]
pub fn keeloq_magic_serial_type3_learning(data: u32, man: u64) -> u64 {
    (man & 0xFFFFFFFFFF000000) | ((data & 0xFFFFFF) as u64)
}

/// KeeLoq learning type constants
#[allow(dead_code)]
pub mod learning_types {
    pub const UNKNOWN: u32 = 0;
    pub const SIMPLE: u32 = 1;
    pub const NORMAL: u32 = 2;
    // pub const SECURE: u32 = 3;
    pub const MAGIC_XOR_TYPE_1: u32 = 4;
    // pub const FAAC: u32 = 5;
    pub const MAGIC_SERIAL_TYPE_1: u32 = 6;
    pub const MAGIC_SERIAL_TYPE_2: u32 = 7;
    pub const MAGIC_SERIAL_TYPE_3: u32 = 8;
}
