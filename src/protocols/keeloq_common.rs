//! KeeLoq common encryption/decryption and learning routines
//!
//! Aligned with ProtoPirate `REFERENCES/ProtoPirate/protocols/keeloq_common.h` and Flipper-ARF
//! `lib/subghz/protocols/keeloq_common.c` / `keeloq_common.h` (the superset: it adds secure, FAAC,
//! AERF, Erreka and Pujol learning and the `decrypt_derived` cipher variant). Shared by KeeLoq,
//! Star Line, Kia V3/V4, Sheriff CFM and the KeeLoq generic fallback.
//! NLF (Non-Linear Feedback) constant 0x3A5C742E per reference.
//!
//! [`keeloq_keystore`] is the keystore as the C KeeLoq family walks it (`M_EACH` over the
//! Flipper keystore): every embedded entry in blob order with its learning type, minus
//! ProtoPirate's KIA_KEY1..4 entries, which share type numbers 10..13 with Flipper's
//! Kinggates/Jarolift/Erreka/Pujol learning types but are not KeeLoq manufacturer keys.

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
        // (15 - r) relies on unsigned wraparound (matches the C reference `(15 - r) & 63`);
        // use wrapping_sub so debug builds don't panic on the overflow that release silently wraps.
        let key_bit = ((key >> ((15u32.wrapping_sub(r)) & 63)) & 1) as u32;
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

/// Secure Learning. Matches ARF `subghz_protocol_keeloq_common_secure_learning`.
/// * `data` - serial (28-bit) * `seed` - seed (32-bit) * returns derived key (64-bit)
pub fn keeloq_secure_learning(data: u32, seed: u32, key: u64) -> u64 {
    let serial = data & 0x0FFFFFFF;
    let k1 = keeloq_decrypt(serial, key);
    let k2 = keeloq_decrypt(seed, key);
    ((k1 as u64) << 32) | (k2 as u64)
}

/// FAAC SLH (Spa) learning. Matches ARF `subghz_protocol_keeloq_common_faac_learning`.
#[allow(dead_code)]
pub fn keeloq_faac_learning(seed: u32, key: u64) -> u64 {
    let hs = (seed >> 16) as u16;
    let ending: u16 = 0x544D;
    let lsb = ((hs as u32) << 16) | (ending as u32);
    ((keeloq_encrypt(seed, key) as u64) << 32) | (keeloq_encrypt(lsb, key) as u64)
}

/// Magic_xor_type1 Learning. Matches `subghz_protocol_keeloq_common_magic_xor_type1_learning`.
/// * `data` - serial (28-bit) * `xor` - magic xor (64-bit) * returns derived key (64-bit)
pub fn keeloq_magic_xor_type1_learning(data: u32, xor: u64) -> u64 {
    let serial = data & 0x0FFFFFFF;
    (((serial as u64) << 32) | (serial as u64)) ^ xor
}

/// Magic_serial_type1 Learning. Matches `subghz_protocol_keeloq_common_magic_serial_type1_learning`.
/// * `data` - serial (28-bit) * `man` - magic man (64-bit) * returns derived key (64-bit)
pub fn keeloq_magic_serial_type1_learning(data: u32, man: u64) -> u64 {
    (man & 0xFFFFFFFF)
        | ((data as u64) << 40)
        | (((((data & 0xFF).wrapping_add((data >> 8) & 0xFF)) & 0xFF) as u64) << 32)
}

/// Magic_serial_type2 Learning. Matches `subghz_protocol_keeloq_common_magic_serial_type2_learning`.
/// Byte-copy of `data` into high 32 bits of result (LE layout as in reference).
/// * `data` - btn+serial (32-bit) * `man` - magic man (64-bit) * returns derived key (64-bit)
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
pub fn keeloq_magic_serial_type3_learning(data: u32, man: u64) -> u64 {
    (man & 0xFFFFFFFFFF000000) | ((data & 0xFFFFFF) as u64)
}

/// ARF `subghz_protocol_keeloq_common_manufacturer_nl_extend` (decompiled ARM routine behind
/// AERF learning and `decrypt_derived`). Each of the first `min(outer_limit, 0x210)` rounds:
/// `x = rotr(x, 1) ^ x[15] ^ key[(15 - r) & 63] ^ NLF[g5(x, 0, 8, 19, 25, 30)]` (the three
/// single-bit terms land in bit 0). The C's key-bit select shifts by 32..255 for some rounds; on
/// the Cortex-M4 the firmware runs on, register shifts of 32 or more yield 0, so the select is
/// exactly key bit `(15 - r) & 63` — that is what is implemented here.
fn manufacturer_nl_extend(mut x: u32, key: u64, outer_limit: u32) -> u32 {
    let rounds = outer_limit.min(0x210);
    for r5 in 0..rounds {
        let n = 15u32.wrapping_sub(r5) & 0x3F;
        let key_bit = ((key >> n) & 1) as u32;
        let r7 = ((x >> 15) & 1) ^ x.rotate_right(1);
        let r1 = key_bit ^ r7;
        x = bit(KEELOQ_NLF, g5(x, 0, 8, 19, 25, 30)) ^ r1;
    }
    x
}

/// ARF `subghz_protocol_keeloq_common_decrypt_derived` (AERF hop decrypt with a derived key).
pub fn keeloq_decrypt_derived(hop_encrypted: u32, derived_key: u64, outer_limit: u32) -> u32 {
    manufacturer_nl_extend(hop_encrypted, derived_key, outer_limit)
}

/// ARF `subghz_protocol_keeloq_common_learning_aerf`.
pub fn keeloq_learning_aerf(data: u32, key: u64) -> u64 {
    let d = data & 0x0FFF_FFFF;
    let k1 = manufacturer_nl_extend(d | 0x2000_0000, key, 0x40);
    let k2 = manufacturer_nl_extend(d | 0x6000_0000, key, 0x40);
    ((k2 as u64) << 32) | (k1 as u64)
}

/// ARF `subghz_protocol_keeloq_common_learning_erreka` (`mix` = the seed).
pub fn keeloq_learning_erreka(data: u32, mix: u32, key: u64) -> u64 {
    let d = data & 0x0FFF_FFFF;
    let k1 = keeloq_decrypt(d | 0x2000_0000, key);
    let r4 = ((mix >> 4) & 0x0F00_0F00) | ((mix << 4) & 0xF000_F000);
    let x = r4 | (mix & 0x00FF_00FF) | 0x6000_0000;
    let k2 = keeloq_decrypt(x, key);
    ((k2 as u64) << 32) | (k1 as u64)
}

/// ARF `subghz_protocol_keeloq_common_learning_pujol`.
pub fn keeloq_learning_pujol(data: u32, key: u64) -> u64 {
    let d = data & 0x0FFF_FFFF;
    let k1 = keeloq_decrypt(d | 0x2000_0000, key).rotate_left(16);
    let k2 = keeloq_decrypt(d | 0x6000_0000, key).rotate_left(16);
    ((k2 as u64) << 32) | (k1 as u64)
}

/// KeeLoq learning type constants (Flipper-ARF keeloq_common.h: KEELOQ_LEARNING_*).
#[allow(dead_code)]
pub mod learning_types {
    pub const UNKNOWN: u32 = 0;
    pub const SIMPLE: u32 = 1;
    pub const NORMAL: u32 = 2;
    pub const SECURE: u32 = 3;
    pub const MAGIC_XOR_TYPE_1: u32 = 4;
    pub const FAAC: u32 = 5;
    pub const MAGIC_SERIAL_TYPE_1: u32 = 6;
    pub const MAGIC_SERIAL_TYPE_2: u32 = 7;
    pub const MAGIC_SERIAL_TYPE_3: u32 = 8;
    // 9 is reserved (Beninca ARC key type).
    pub const SIMPLE_KINGGATES: u32 = 10;
    pub const NORMAL_JAROLIFT: u32 = 11;
    pub const ERREKA: u32 = 12;
    pub const PUJOL: u32 = 13;
    pub const AERF: u32 = 14;
    pub const SIMPLE_JCM: u32 = 15;
}

/// One keystore manufacturer entry (C `SubGhzKey`: name, key, learning type).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MfKey {
    pub name: String,
    pub learning: u32,
    pub key: u64,
}

/// Learning-type corrections applied to KAT's embedded keystore, each backed by real captures.
/// - DoorHan: the blob says NORMAL (2), but the three real DoorHan remotes in Flipper-ARF's
///   unit-test captures (`unit_tests/subghz/`: serial D36C681 in `doorhan_raw.sub` +
///   `test_random_raw.sub`, counters 04BD..04C2; serial 47D94C1 in `test_random_raw.sub`, counters
///   0490/0491; serial 8A91ECC in `doorhan.sub`) all decrypt with the DoorHan key under SIMPLE
///   learning (1) and nothing else (see docs/keeloq.md).
/// - Kingates_Stylo4k: the blob says SIMPLE (1); the references' KingGates Stylo4k decoder looks for
///   learning type 10 (`KEELOQ_LEARNING_SIMPLE_KINGGATES`), which the KeeLoq selector skips. The real
///   remote in `kinggates_stylo4k_raw.sub` (serial 0x130F8127, button 0xE, counter 0x0B9D) passes
///   the KingGates check with this key, and the reference C reproduces those fields only with the
///   key typed 10 (see `kinggates_stylo_4k.rs`).
const LEARNING_TYPE_FIXES: &[(&str, u32, u32)] = &[
    ("DoorHan", learning_types::NORMAL, learning_types::SIMPLE),
    ("Kingates_Stylo4k", learning_types::SIMPLE, learning_types::SIMPLE_KINGGATES),
];

/// The keystore as the C KeeLoq-family decoders iterate it (see the module doc): all embedded
/// entries in blob order except ProtoPirate's KIA_KEY1..4 entries, with [`LEARNING_TYPE_FIXES`]
/// applied, then the user keystore, then keys added during this session ([`add_keeloq_key`]).
/// Parsed once; the slice handed out stays valid when a key is added (the list is replaced, not
/// changed).
pub fn keeloq_keystore() -> &'static [MfKey] {
    *keystore_cell().read().unwrap_or_else(|e| e.into_inner())
}

fn keystore_cell() -> &'static std::sync::RwLock<&'static [MfKey]> {
    static KEYS: std::sync::OnceLock<std::sync::RwLock<&'static [MfKey]>> = std::sync::OnceLock::new();
    KEYS.get_or_init(|| {
        let keys: Vec<MfKey> = crate::keystore::keystore_entries_with_names()
            .into_iter()
            .filter(|(name, ty, _)| !((10..=13).contains(ty) && name.starts_with("KIA")))
            .map(|(name, ty, key)| {
                let learning = LEARNING_TYPE_FIXES
                    .iter()
                    .find(|(n, from, _)| *n == name && *from == ty)
                    .map_or(ty, |&(_, _, to)| to);
                MfKey { name, learning, key }
            })
            .collect();
        std::sync::RwLock::new(Vec::leak(keys))
    })
}

/// Add a manufacturer key for the rest of the session, after all others (where the user keystore
/// file puts it at the next start), e.g. one recovered by [`super::keeloq_bf`]. Returns false when
/// the same entry is already there. The previous list is leaked: decoders may still hold it, and
/// keys are added rarely.
pub fn add_keeloq_key(entry: MfKey) -> bool {
    let mut keys = keystore_cell().write().unwrap_or_else(|e| e.into_inner());
    if keys.contains(&entry) {
        return false;
    }
    let mut next = keys.to_vec();
    next.push(entry);
    *keys = Vec::leak(next);
    true
}

/// C "mirrored man": the manufacturer key with its byte order reversed (tried for
/// `KEELOQ_LEARNING_UNKNOWN` entries).
pub fn mirrored_key(key: u64) -> u64 {
    key.swap_bytes()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Values computed by the Flipper-ARF C (`keeloq_common.c`, the decompiled routine's register
    /// shifts given Cortex-M4 semantics) for fixed inputs.
    const ERREKA_VECTOR: u64 = 0x29F6_7909_3470_B435;
    const PUJOL_VECTOR: u64 = 0xD070_1E24_B435_3470;
    const AERF_VECTOR: u64 = 0x8095_6967_8826_98C7;
    const DERIVED_VECTOR: u32 = 0x91D3_302A;
    const DERIVED40_VECTOR: u32 = 0x6C30_0569;

    #[test]
    fn arf_learning_functions_match_reference_vectors() {
        let data = 0x1234_5678u32;
        let key = 0x0123_4567_89AB_CDEFu64;
        assert_eq!(keeloq_learning_erreka(data, 0xCAFE_BABE, key), ERREKA_VECTOR);
        assert_eq!(keeloq_learning_pujol(data, key), PUJOL_VECTOR);
        assert_eq!(keeloq_learning_aerf(data, key), AERF_VECTOR);
        assert_eq!(keeloq_decrypt_derived(data, key, 0x240), DERIVED_VECTOR);
        assert_eq!(keeloq_decrypt_derived(data, key, 0x40), DERIVED40_VECTOR);
    }

    #[test]
    fn keystore_view_excludes_kia_and_fixes_doorhan() {
        let ks = keeloq_keystore();
        assert!(ks.iter().all(|e| !e.name.starts_with("KIA")));
        let doorhan = ks.iter().find(|e| e.name == "DoorHan").expect("DoorHan");
        assert_eq!(doorhan.learning, learning_types::SIMPLE);
        assert!(ks.iter().any(|e| e.name == "Jarolift"), "Flipper type-11 entry kept");
    }

    /// C-oracle driver for the learning functions (ignored): `KAT_LRN_IN` lines
    /// `<fn> <data hex> <key hex> <extra hex>` → `KAT_LRN_OUT` lines `<fn> <result>`.
    #[test]
    #[ignore]
    fn learning_oracle() {
        let input = std::fs::read_to_string(std::env::var("KAT_LRN_IN").unwrap()).unwrap();
        let mut out = String::new();
        for line in input.lines() {
            let t: Vec<&str> = line.split_whitespace().collect();
            let a = u32::from_str_radix(t[1], 16).unwrap();
            let b = u64::from_str_radix(t[2], 16).unwrap();
            let c = u32::from_str_radix(t[3], 16).unwrap();
            let r = match t[0] {
                "aerf" => keeloq_learning_aerf(a, b),
                "erreka" => keeloq_learning_erreka(a, c, b),
                "pujol" => keeloq_learning_pujol(a, b),
                "derived" => u64::from(keeloq_decrypt_derived(a, b, c)),
                "faac" => keeloq_faac_learning(a, b),
                "secure" => keeloq_secure_learning(a, c, b),
                "ms1" => keeloq_magic_serial_type1_learning(a, b),
                "ms2" => keeloq_magic_serial_type2_learning(a, b),
                _ => keeloq_magic_serial_type3_learning(a, b),
            };
            out.push_str(&format!("{} {:016X}\n", t[0], r));
        }
        std::fs::write(std::env::var("KAT_LRN_OUT").unwrap(), out).unwrap();
    }
}
