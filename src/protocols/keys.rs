//! Key management module for protocol encryption/decryption
//!
//! Ported from protopirate's keys.c
//!
//! Manages manufacturer keys used by various protocols:
//! - KIA V3/V4: kia_mf_key (manufacturer key for KeeLoq)
//! - KIA V5: kia_v5_key (custom mixer cipher key)
//! - KIA V6: kia_v6_a_key, kia_v6_b_key (AES-128 XOR mask keys)
//! - VAG: AUT64 keys loaded from keystore files

use super::aut64::{self, Aut64Key, AUT64_KEY_STRUCT_PACKED_SIZE};
use std::path::Path;
use std::sync::{OnceLock, RwLock};
use tracing::{info, warn, error};

/// Key type identifiers (matches protopirate's keystore types)
const KIA_KEY1: u32 = 10; // kia_mf_key
const KIA_KEY2: u32 = 11; // kia_v6_a_key
const KIA_KEY3: u32 = 12; // kia_v6_b_key
const KIA_KEY4: u32 = 13; // kia_v5_key

/// Maximum number of VAG AUT64 keys
const VAG_KEYS_COUNT: usize = 3;

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
                _ => {}
            }
        }
    }

    /// Load VAG AUT64 keys from raw binary data
    /// The data should contain packed AUT64 key structures (16 bytes each)
    pub fn load_vag_keys_from_data(&mut self, data: &[u8]) {
        if self.vag_keys_loaded {
            return;
        }

        self.vag_keys.clear();

        for i in 0..VAG_KEYS_COUNT {
            let offset = i * AUT64_KEY_STRUCT_PACKED_SIZE;
            if offset + AUT64_KEY_STRUCT_PACKED_SIZE > data.len() {
                error!("VAG key data too short for key {}", i);
                break;
            }

            let key = aut64::aut64_unpack(&data[offset..offset + AUT64_KEY_STRUCT_PACKED_SIZE]);
            self.vag_keys.push(key);
        }

        self.vag_keys_loaded = true;
        info!("Loaded {} VAG keys", self.vag_keys.len());
    }

    /// Load VAG AUT64 keys from a file path
    pub fn load_vag_keys_from_file(&mut self, path: &str) {
        if self.vag_keys_loaded {
            return;
        }

        let file_path = Path::new(path);
        if !file_path.exists() {
            warn!("VAG key file not found: {}", path);
            return;
        }

        match std::fs::read(file_path) {
            Ok(data) => {
                self.load_vag_keys_from_data(&data);
            }
            Err(e) => {
                error!("Failed to read VAG key file {}: {}", path, e);
            }
        }
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

/// Initialize the global keystore with KIA keys
pub fn load_keys(kia_entries: &[(u32, u64)]) {
    let mut store = get_keystore_mut();
    store.load_kia_keys(kia_entries);
}

/// Initialize VAG keys from file
pub fn load_vag_keys(path: &str) {
    let mut store = get_keystore_mut();
    store.load_vag_keys_from_file(path);
}
