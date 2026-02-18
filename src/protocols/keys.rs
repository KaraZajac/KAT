//! Key management module for protocol encryption/decryption
//!
//! All keys are loaded from the **embedded keystore only** (no keystore.ini or file-based keys).
//! The blob is built from the master list `REFERENCES/mf_keys.txt` by
//! `scripts/build_keystore_from_mf_keys.py` and embedded in `crate::keystore::embedded`.
//!
//! Aligned with ProtoPirate's keys.c (KIA_KEY1..4, get_kia_mf_key, etc.).
//! At startup `load_keystore_from_embedded()` parses `keystore::embedded::KEYSTORE_BLOB` and populates:
//!
//! - **KIA**: type 10 → kia_mf_key, 11 → kia_v6_a_key, 12 → kia_v6_b_key, 13 → kia_v5_key
//! - **Star Line**: type 20 → star_line_mf_key
//! - **VAG**: raw 64 bytes after "VAG " tag = 4 × 16-byte AUT64 packed keys (index in byte 0 of each)
//!
//! VAG protocol looks up keys by `get_vag_key((key_index + 1) as u8)` (index 1, 2, 3).
//! KIA V5 uses `get_kia_v5_key()`, KIA V6 uses `get_kia_v6_keystore_a()` / `get_kia_v6_keystore_b()`.

use super::aut64::{self, Aut64Key, AUT64_KEY_STRUCT_PACKED_SIZE};
use std::sync::{OnceLock, RwLock};
use tracing::{error, info};

/// Key type identifiers; must match type IDs in `keystore::embedded::KEYSTORE_BLOB`.
const KIA_KEY1: u32 = 10; // kia_mf_key (KIA V3/V4)
const KIA_KEY2: u32 = 11; // kia_v6_a_key (KIA V6A)
const KIA_KEY3: u32 = 12; // kia_v6_b_key (KIA V6B)
const KIA_KEY4: u32 = 13; // kia_v5_key (KIA V5)
const STAR_LINE_KEY: u32 = 20; // star_line_mf_key

/// Maximum number of VAG AUT64 keys (embedded blob has 64 bytes = 4 keys)
const MAX_VAG_KEYS: usize = 4;

/// Global key store - thread-safe access to loaded keys
pub struct KeyStore {
    /// KIA manufacturer key (for KeeLoq-based V3/V4)
    pub kia_mf_key: u64,
    /// KIA V6 AES key A
    pub kia_v6_a_key: u64,
    /// KIA V6 AES key B
    pub kia_v6_b_key: u64,
    /// KIA V5 mixer key
    pub kia_v5_key: u64,
    /// Star Line manufacturer key (for KeeLoq)
    pub star_line_mf_key: u64,
    /// VAG AUT64 keys
    pub vag_keys: Vec<Aut64Key>,
    /// Whether VAG keys have been loaded
    pub vag_keys_loaded: bool,
}

impl Default for KeyStore {
    fn default() -> Self {
        Self {
            kia_mf_key: 0,
            kia_v6_a_key: 0,
            kia_v6_b_key: 0,
            kia_v5_key: 0,
            star_line_mf_key: 0,
            vag_keys: Vec::new(),
            vag_keys_loaded: false,
        }
    }
}

impl KeyStore {
    /// Create a new empty key store
    pub fn new() -> Self {
        Self::default()
    }

    /// Load KIA keys from a key entries list
    /// Each entry is (type_id, key_value)
    pub fn load_kia_keys(&mut self, entries: &[(u32, u64)]) {
        for &(key_type, key_value) in entries {
            match key_type {
                KIA_KEY1 => self.kia_mf_key = key_value,
                KIA_KEY2 => self.kia_v6_a_key = key_value,
                KIA_KEY3 => self.kia_v6_b_key = key_value,
                KIA_KEY4 => self.kia_v5_key = key_value,
                STAR_LINE_KEY => self.star_line_mf_key = key_value,
                _ => {}
            }
        }
    }

    /// Load VAG AUT64 keys from raw binary data (16 bytes per key; up to MAX_VAG_KEYS).
    /// Used only by the embedded keystore parser.
    pub fn load_vag_keys_from_data(&mut self, data: &[u8]) {
        if self.vag_keys_loaded {
            return;
        }

        self.vag_keys.clear();
        let n = (data.len() / AUT64_KEY_STRUCT_PACKED_SIZE).min(MAX_VAG_KEYS);

        for i in 0..n {
            let offset = i * AUT64_KEY_STRUCT_PACKED_SIZE;
            if offset + AUT64_KEY_STRUCT_PACKED_SIZE > data.len() {
                break;
            }
            let key = aut64::aut64_unpack(&data[offset..offset + AUT64_KEY_STRUCT_PACKED_SIZE]);
            self.vag_keys.push(key);
        }

        self.vag_keys_loaded = true;
        info!("Loaded {} VAG keys", self.vag_keys.len());
    }

    /// Get a VAG AUT64 key by its internal index field
    pub fn get_vag_key(&self, index: u8) -> Option<&Aut64Key> {
        self.vag_keys.iter().find(|k| k.index == index)
    }

    /// Get a VAG AUT64 key by array position (0-based)
    pub fn get_vag_key_by_position(&self, position: usize) -> Option<&Aut64Key> {
        self.vag_keys.get(position)
    }

    /// Get the KIA manufacturer key
    pub fn get_kia_mf_key(&self) -> u64 {
        self.kia_mf_key
    }

    /// Get the KIA V6 AES key A
    pub fn get_kia_v6_keystore_a(&self) -> u64 {
        self.kia_v6_a_key
    }

    /// Get the KIA V6 AES key B
    pub fn get_kia_v6_keystore_b(&self) -> u64 {
        self.kia_v6_b_key
    }

    /// Get the KIA V5 mixer key
    pub fn get_kia_v5_key(&self) -> u64 {
        self.kia_v5_key
    }

    /// Get the Star Line manufacturer key
    pub fn get_star_line_mf_key(&self) -> u64 {
        self.star_line_mf_key
    }
}

/// Global singleton keystore
fn global_keystore() -> &'static RwLock<KeyStore> {
    static GLOBAL_KEYSTORE: OnceLock<RwLock<KeyStore>> = OnceLock::new();
    GLOBAL_KEYSTORE.get_or_init(|| RwLock::new(KeyStore::new()))
}

/// Get a read reference to the global keystore
pub fn get_keystore() -> std::sync::RwLockReadGuard<'static, KeyStore> {
    global_keystore().read().unwrap()
}

/// Get a write reference to the global keystore
pub fn get_keystore_mut() -> std::sync::RwLockWriteGuard<'static, KeyStore> {
    global_keystore().write().unwrap()
}

/// Initialize the global keystore with KIA keys (matches protopirate_keys_load pattern)
pub fn load_keys(kia_entries: &[(u32, u64)]) {
    let mut store = get_keystore_mut();
    store.load_kia_keys(kia_entries);
}

/// Load the global keystore from the embedded blob (src/keystore/embedded.rs).
/// Populates KIA (V3/V4, V5, V6A, V6B), Star Line, and VAG AUT64 keys from the blob.
/// VAG raw bytes are 64 bytes = 4 × 16-byte packed keys; each key's `index` is byte 0 (used by VAG lookup).
pub fn load_keystore_from_embedded() {
    let blob = crate::keystore::embedded_blob();
    let Some(parsed) = crate::keystore::parse_blob(blob) else {
        error!("Failed to parse embedded keystore blob");
        return;
    };
    let mut store = get_keystore_mut();
    store.load_kia_keys(&parsed.entries);
    if !parsed.vag_bytes.is_empty() {
        store.load_vag_keys_from_data(&parsed.vag_bytes);
    }
    info!(
        "Keystore loaded from embedded blob ({} entries, {} VAG keys)",
        parsed.entries.len(),
        store.vag_keys.len()
    );
}
