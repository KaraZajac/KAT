//! KeeLoq common encryption/decryption and learning routines
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/keeloq_common.c` and
//! `keeloq_common.h`. Shared by Kia V3/V4, Star Line, and other KeeLoq-based protocols.
//! NLF (Non-Linear Feedback) constant 0x3A5C742E per reference.
//!
//! Learning types and encrypt/decrypt/normal/magic_* match the reference; secure_learning and
//! faac_learning are from other SubGHz docs and are not present in keeloq_common.c.

/// The KeeLoq NLF constant (KEELOQ_NLF in reference)
const KEELOQ_NLF: u32 = 0x3A5C742E;

#[inline]
fn bit(x: u32, n: u32) -> u32 {
    (x >> n) & 1
}

/// g5(x,a,b,c,d,e) = bit(x,a) + bit(x,b)*2 + ... + bit(x,e)*16 (0..31). Matches reference macro.
#[inline]
fn g5(x: u32, a: u32, b: u32, c: u32, d: u32, e: u32) -> u32 {
    bit(x, a) | (bit(x, b) << 1) | (bit(x, c) << 2) | (bit(x, d) << 3) | (bit(x, e) << 4)
}

/// Simple Learning Decrypt. 528 rounds; key bit for round r is key[(15 - r) & 63]; NLF g5(x, 0, 8, 19, 25, 30).
/// Matches `subghz_protocol_keeloq_common_decrypt`.
///
/// * `data` - keeloq-encrypted 32-bit value  
/// * `key` - manufacture key (64-bit)  
/// * returns 0xBSSSCCCC: B(4bit) key, S(10bit) serial&0x3FF, C(16bit) counter
pub fn keeloq_decrypt(data: u32, key: u64) -> u32 {
    let mut x = data;
    for r in 0..528u32 {
        let key_bit = ((key >> ((15 - r) & 63)) & 1) as u32;
        let new_lsb = bit(x, 31) ^ bit(x, 15) ^ key_bit
            ^ bit(KEELOQ_NLF, g5(x, 0, 8, 19, 25, 30));
        x = (x << 1) ^ new_lsb;
    }
    x
}

/// Simple Learning Encrypt. 528 rounds; key bit for round r is key[r & 63]; NLF g5(x, 1, 9, 20, 26, 31).
/// Matches `subghz_protocol_keeloq_common_encrypt`.
///
/// * `data` - 0xBSSSCCCC (B 4bit key, S 10bit serial&0x3FF, C 16bit counter)  
/// * `key` - manufacture key (64-bit)  
/// * returns keeloq-encrypted 32-bit value
pub fn keeloq_encrypt(data: u32, key: u64) -> u32 {
    let mut x = data;
    for r in 0..528u32 {
        let key_bit = ((key >> (r & 63)) & 1) as u32;
        let new_msb = bit(x, 0) ^ bit(x, 16) ^ key_bit
            ^ bit(KEELOQ_NLF, g5(x, 1, 9, 20, 26, 31));
        x = (x >> 1) ^ (new_msb << 31);
    }
    x
}

/// Normal Learning. Matches `subghz_protocol_keeloq_common_normal_learning`.
///
/// * `data` - serial number (28-bit, upper bits ignored)  
/// * `key` - manufacture key (64-bit)  
/// * returns derived key for this serial (64-bit)
pub fn keeloq_normal_learning(data: u32, key: u64) -> u64 {
    let data = data & 0x0FFFFFFF;
    let k1 = keeloq_decrypt(data | 0x20000000, key);
    let k2 = keeloq_decrypt(data | 0x60000000, key);
    ((k2 as u64) << 32) | (k1 as u64)
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

/// Secure learning key derivation (not in keeloq_common.c; from other SubGHz docs).
#[allow(dead_code)]
pub fn keeloq_secure_learning(data: u32, seed: u32, key: u64) -> u64 {
    let serial = data & 0x0FFFFFFF;
    let k1 = keeloq_decrypt(serial, key);
    let k2 = keeloq_decrypt(seed, key);
    ((k1 as u64) << 32) | (k2 as u64)
}

/// FAAC SLH (Spa) learning (not in keeloq_common.c; from other SubGHz docs).
#[allow(dead_code)]
pub fn keeloq_faac_learning(seed: u32, key: u64) -> u64 {
    let hs = (seed >> 16) as u16;
    let ending: u16 = 0x544D;
    let lsb = ((hs as u32) << 16) | (ending as u32);
    ((keeloq_encrypt(seed, key) as u64) << 32) | (keeloq_encrypt(lsb, key) as u64)
}

/// Magic_xor_type1 Learning. Matches `subghz_protocol_keeloq_common_magic_xor_type1_learning`.
/// * `data` - serial (28-bit) * `xor` - magic xor (64-bit) * returns derived key (64-bit)
#[allow(dead_code)]
pub fn keeloq_magic_xor_type1_learning(data: u32, xor: u64) -> u64 {
    let serial = data & 0x0FFFFFFF;
    (((serial as u64) << 32) | (serial as u64)) ^ xor
}

/// Magic_serial_type1 Learning. Matches `subghz_protocol_keeloq_common_magic_serial_type1_learning`.
/// * `data` - serial (28-bit) * `man` - magic man (64-bit) * returns derived key (64-bit)
#[allow(dead_code)]
pub fn keeloq_magic_serial_type1_learning(data: u32, man: u64) -> u64 {
    (man & 0xFFFFFFFF)
        | ((data as u64) << 40)
        | (((((data & 0xFF).wrapping_add((data >> 8) & 0xFF)) & 0xFF) as u64) << 32)
}

/// Magic_serial_type2 Learning. Matches `subghz_protocol_keeloq_common_magic_serial_type2_learning`.
/// Byte-copy of `data` into high 32 bits of result (LE layout as in reference).
/// * `data` - btn+serial (32-bit) * `man` - magic man (64-bit) * returns derived key (64-bit)
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

/// Magic_serial_type3 Learning. Matches `subghz_protocol_keeloq_common_magic_serial_type3_learning`.
/// * `data` - serial (24-bit) * `man` - magic man (64-bit) * returns derived key (64-bit)
#[allow(dead_code)]
pub fn keeloq_magic_serial_type3_learning(data: u32, man: u64) -> u64 {
    (man & 0xFFFFFFFFFF000000) | ((data & 0xFFFFFF) as u64)
}

/// KeeLoq learning type constants (keeloq_common.h: KEELOQ_LEARNING_*).
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
