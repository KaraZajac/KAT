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
#[derive(Default)]
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
    /// JLR car keys the user supplied (name, 16-byte key) — for the Ford V1 (JLR) rolling code. KAT
    /// ships none; each is a car's own key, read from the owner's keyless module (see [`crate::jlr`]).
    pub jlr_keys: Vec<(String, [u8; 16])>,
    /// Fob ID → (car key, last counter seen, last button) learned as frames decipher; lets the
    /// encoder make a fob's next code after one of its captures has been decoded.
    pub jlr_fobs: std::collections::HashMap<u32, ([u8; 16], u32, u8)>,
    /// Audi BCM2 car keys the user supplied — for the Audi V3 (BCM2) rolling code's MAC. KAT ships
    /// none; each is a car's own secrets, read from the owner's BCM2 module (see [`crate::audi_v3`]).
    pub audi_bcm2_keys: Vec<AudiBcm2Car>,
    /// The PSA older-code key table (16 bytes): the one global table that deciphers the legacy
    /// two-button Peugeot/Citroën remotes (GitHub issue #6), recovered from a BSI module's firmware.
    /// KAT ships it ([`crate::keystore::PSA_BSI_TABLE`], loaded at startup); a `psa_bsi_key` file
    /// overrides it. None only in a test that has cleared it.
    pub psa_bsi_table: Option<[u8; 16]>,
    /// The PSA 2-Button cipher key table (16 bytes): the table that deciphers a Valeo two-button
    /// Citroën/Peugeot flip key's 64-bit code (see [`crate::protocols::psa_two_button`]). KAT ships a
    /// built-in ([`crate::keystore::PSA_2BUTTON_TABLE`], loaded at startup); a `psa_2button_key` file
    /// overrides it. Recovered from one key's captures, so whether one table serves the whole family
    /// is not established yet. None only before startup, or in a test that has not set it.
    pub psa_2button_table: Option<[u8; 16]>,
    /// BMW CAS4 car keys the user supplied — for the F-series CAS4 remote's AES-128 MAC
    /// ([`crate::protocols::bmw_rke`]; not the CAS3 `0xCC` frame, which is HITAG2). KAT ships none;
    /// each is a car's own secret, read from the owner's CAS4 module. Empty unless a `bmw_cas4_keys`
    /// file was loaded.
    pub bmw_cas4_keys: Vec<BmwCas4Car>,
    /// BMW/Mini **CAS3** fob secrets the user supplied — for the HITAG2 rolling code the CAS3
    /// checks ([`crate::protocols::bmw_v3`]). Keyed by the fob's 16-bit ID (`b2 b3`). Each holds the
    /// HITAG2 `uid`, 48-bit key and last-seen 32-bit counter, read from the car's own CAS3 module;
    /// KAT ships none and cannot derive them from the air. The counter is updated (resynced) as the
    /// fob's frames decode. Empty unless a `bmw_cas3_keys` file was loaded.
    pub bmw_cas3_fobs: std::collections::HashMap<u16, BmwCas3Fob>,
}

/// One learned BMW/Mini **CAS3** fob's secrets, needed to verify and forge its HITAG2 rolling code
/// ([`crate::protocols::bmw_v3`]). Read from the car's own CAS3 module (the per-key records in the
/// module's EEPROM); KAT ships none and cannot derive them from the air.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BmwCas3Fob {
    /// A label for the fob/car (from the key file).
    pub name: String,
    /// The fob's 16-bit ID (`b2 b3` on the air; the CAS3's lookup key).
    pub id: u16,
    /// The HITAG2 32-bit UID (per-key, module record `uid32`).
    pub uid: u32,
    /// The HITAG2 48-bit key (module record `key16 ‖ key32`, big-endian: `key[0..2]` the 16-bit
    /// half, `key[2..6]` the 32-bit half).
    pub key: [u8; 6],
    /// The last-seen 32-bit counter (the HITAG2 IV). Updated as the fob's frames resync.
    pub counter: u32,
}

/// The secrets of one BMW CAS4 car, needed to recompute the remote frame's AES-128 MAC
/// ([`crate::protocols::bmw_rke`]). Read from the car's own module; KAT ships none and cannot
/// derive them from the air.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BmwCas4Car {
    /// A label for the car (from the key file).
    pub name: String,
    /// The car's 128-bit master secret, when supplied — KAT derives each remote's key from it
    /// ([`crate::protocols::bmw_rke::derive_key`]). None when the per-remote keys are given directly.
    pub master: Option<[u8; 16]>,
    /// Each learned remote's 128-bit key, by key number. Given directly, or derived from `master`.
    pub keys: Vec<BmwKey>,
}

/// One learned BMW remote's key: its key number (`t[11]`, 0..=7) and 128-bit AES key.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BmwKey {
    /// The remote's key number in the car.
    pub num: u8,
    /// The remote's 128-bit MAC key.
    pub key: [u8; 16],
}

/// The secrets of one Audi BCM2 car, needed to recompute and forge the RF frame's AES-128 MAC
/// (see [`crate::protocols::audi_v3`]). All of it is read from the car's own module (the immobiliser
/// block and key slots); KAT ships none and cannot derive them from the air.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AudiBcm2Car {
    /// A label for the car (from the key file).
    pub name: String,
    /// The 128-bit MAC key `K` (immobiliser block, rec07[0x10..0x20]).
    pub key: [u8; 16],
    /// The 4-byte per-car value folded into every MAC (rec13[12..16]).
    pub car_value: [u8; 4],
    /// The 4-byte ID of each learned key, in key-number order (key 1 first); up to 8.
    pub key_ids: Vec<[u8; 4]>,
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

    /// Replace the user's JLR car keys (and forget any learned fob associations).
    pub fn load_jlr_keys(&mut self, keys: Vec<(String, [u8; 16])>) {
        self.jlr_keys = keys;
        self.jlr_fobs.clear();
    }

    /// The user's JLR car keys (name, key). Empty unless the user loaded a key file.
    pub fn jlr_keys(&self) -> &[(String, [u8; 16])] {
        &self.jlr_keys
    }

    /// Replace the user's Audi BCM2 car keys.
    pub fn load_audi_bcm2_keys(&mut self, cars: Vec<AudiBcm2Car>) {
        self.audi_bcm2_keys = cars;
    }

    /// The user's Audi BCM2 car keys. Empty unless the user loaded a key file.
    pub fn audi_bcm2_keys(&self) -> &[AudiBcm2Car] {
        &self.audi_bcm2_keys
    }

    /// Replace the user's BMW CAS4 car keys.
    pub fn load_bmw_cas4_keys(&mut self, cars: Vec<BmwCas4Car>) {
        self.bmw_cas4_keys = cars;
    }

    /// The user's BMW CAS4 car keys. Empty unless the user loaded a key file.
    pub fn bmw_cas4_keys(&self) -> &[BmwCas4Car] {
        &self.bmw_cas4_keys
    }

    /// Replace the user's BMW/Mini CAS3 fob secrets (keyed by 16-bit fob ID).
    pub fn load_bmw_cas3_fobs(&mut self, fobs: Vec<BmwCas3Fob>) {
        self.bmw_cas3_fobs = fobs.into_iter().map(|f| (f.id, f)).collect();
    }

    /// The CAS3 fob secrets for a 16-bit fob ID, if the user loaded them. None by default.
    pub fn bmw_cas3_fob(&self, id: u16) -> Option<BmwCas3Fob> {
        self.bmw_cas3_fobs.get(&id).cloned()
    }

    /// Any CAS3 fobs loaded (for the status line / tests).
    pub fn bmw_cas3_fob_count(&self) -> usize {
        self.bmw_cas3_fobs.len()
    }

    /// Advance a CAS3 fob's stored counter after a frame verified against it (resync). Only moves
    /// forward; a lower value is ignored. No-op when the fob is unknown.
    pub fn remember_bmw_cas3_counter(&mut self, id: u16, counter: u32) {
        if let Some(f) = self.bmw_cas3_fobs.get_mut(&id) {
            if counter > f.counter {
                f.counter = counter;
            }
        }
    }

    /// Set (or clear) the PSA older-code BSI key table.
    pub fn load_psa_bsi_table(&mut self, table: Option<[u8; 16]>) {
        self.psa_bsi_table = table;
    }

    /// The PSA older-code BSI key table, if the user loaded one. None by default.
    pub fn psa_bsi_table(&self) -> Option<[u8; 16]> {
        self.psa_bsi_table
    }

    pub fn load_psa_2button_table(&mut self, table: Option<[u8; 16]>) {
        self.psa_2button_table = table;
    }

    pub fn psa_2button_table(&self) -> Option<[u8; 16]> {
        self.psa_2button_table
    }

    /// Add one JLR car key (e.g. recovered by "Recover key (JLR)"), unless the same key is already
    /// present. Keeps any fob associations already learned.
    pub fn add_jlr_key(&mut self, name: String, key: [u8; 16]) {
        if !self.jlr_keys.iter().any(|(_, k)| *k == key) {
            self.jlr_keys.push((name, key));
        }
    }

    /// Record the car key, counter and button a fob's frame deciphered to, so its next code can be
    /// made. The newest counter for an ID wins.
    pub fn remember_jlr_fob(&mut self, id: u32, key: [u8; 16], counter: u32, button: u8) {
        self.jlr_fobs.insert(id, (key, counter, button));
    }

    /// The car key, last counter and last button learned for a fob ID, if any.
    pub fn jlr_fob(&self, id: u32) -> Option<([u8; 16], u32, u8)> {
        self.jlr_fobs.get(&id).copied()
    }
}

/// Global singleton keystore
fn global_keystore() -> &'static RwLock<KeyStore> {
    static GLOBAL_KEYSTORE: OnceLock<RwLock<KeyStore>> = OnceLock::new();
    GLOBAL_KEYSTORE.get_or_init(|| RwLock::new(KeyStore::new()))
}

/// Get a read reference to the global keystore
pub fn get_keystore() -> std::sync::RwLockReadGuard<'static, KeyStore> {
    global_keystore().read().unwrap_or_else(|e| e.into_inner())
}

/// Get a write reference to the global keystore
pub fn get_keystore_mut() -> std::sync::RwLockWriteGuard<'static, KeyStore> {
    global_keystore().write().unwrap_or_else(|e| e.into_inner())
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
