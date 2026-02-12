//! Key management module for protocol encryption/decryption
//!
//! Aligned with ProtoPirate's keys.c (KIA_KEY1..4, get_kia_mf_key, etc.).
//! Keys are loaded from the embedded keystore blob in `crate::keystore` at startup,
//! matching the standard encrypted + VAG raw keystore data.

use super::aut64::{self, Aut64Key, AUT64_KEY_STRUCT_PACKED_SIZE};
use configparser::ini::Ini;
use std::path::Path;
use std::sync::{OnceLock, RwLock};
use tracing::{info, warn, error};

/// Key type identifiers (matches protopirate's keystore types)
const KIA_KEY1: u32 = 10; // kia_mf_key
const KIA_KEY2: u32 = 11; // kia_v6_a_key
const KIA_KEY3: u32 = 12; // kia_v6_b_key
const KIA_KEY4: u32 = 13; // kia_v5_key
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

    /// Load VAG AUT64 keys from raw binary data (16 bytes per key; up to MAX_VAG_KEYS)
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

/// Initialize VAG keys from file
pub fn load_vag_keys(path: &str) {
    let mut store = get_keystore_mut();
    store.load_vag_keys_from_file(path);
}

/// Load the global keystore from the embedded blob (src/keystore/embedded.rs).
/// Matches ProtoPirate loading from encrypted + VAG keystore; keys are compiled in.
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

// =============================================================================
// Keystore file loading from ~/.config/KAT/keystore/
// =============================================================================

/// Parse a hex string (with or without "0x" prefix) into a u64.
fn parse_hex_u64(s: &str) -> Option<u64> {
    let s = s.trim();
    let s = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")).unwrap_or(s);
    u64::from_str_radix(s, 16).ok()
}

/// Default keystore.ini template content.
/// Written to disk on first run so users know which keys to configure.
const DEFAULT_KEYSTORE_INI: &str = r#"; KAT Keystore — Protocol Encryption Keys
;
; Place your protocol keys here. Key values should be in hexadecimal
; with a 0x prefix (e.g. 0x0123456789ABCDEF).
;
; Keys left at 0x0000000000000000 or omitted are treated as "not loaded"
; and the corresponding protocol will decode without decryption
; (serial/button still visible, but counter may not validate).
;
; This file corresponds to protopirate's keystore/encrypted and keystore/vag
; assets. Since KAT runs on a PC (not a Flipper), keys must be provided
; in plaintext here rather than in the Flipper's encrypted keystore format.

[kia]
; KIA V3/V4: KeeLoq manufacturer key (used for hop-code decryption)
mf_key = 0x0000000000000000

; KIA V5: Custom mixer cipher key
v5_key = 0x0000000000000000

; KIA V6: AES-128 key components (XOR-masked before use)
v6_a_key = 0x0000000000000000
v6_b_key = 0x0000000000000000

[star_line]
; Star Line: KeeLoq manufacturer key (used for hop-code decryption)
mf_key = 0x0000000000000000

[vag]
; VAG: Path to AUT64 binary key file (raw packed keys, 16 bytes each).
; Can be absolute or relative to the keystore directory.
; Leave empty or commented out if you don't have VAG keys.
; keys_file = vag.bin
"#;

/// Write the default keystore.ini template if it doesn't exist yet.
pub fn create_default_keystore(keystore_dir: &Path) {
    let ini_path = keystore_dir.join("keystore.ini");
    if ini_path.exists() {
        return;
    }

    match std::fs::write(&ini_path, DEFAULT_KEYSTORE_INI) {
        Ok(_) => info!("Created default keystore template at {:?}", ini_path),
        Err(e) => warn!("Could not create default keystore.ini: {}", e),
    }
}

/// Load all keys from the keystore directory.
///
/// Reads `keystore.ini` from the given directory and populates the global
/// [`KeyStore`] with:
///
/// - KIA V3/V4 manufacturer key (`[kia] mf_key`)
/// - KIA V5 mixer key (`[kia] v5_key`)
/// - KIA V6 AES keys (`[kia] v6_a_key`, `[kia] v6_b_key`)
/// - Star Line manufacturer key (`[star_line] mf_key`)
/// - VAG AUT64 keys from binary file (`[vag] keys_file`)
///
/// Keys that are missing, zeroed, or unparseable are silently skipped.
pub fn load_keystore_from_dir(keystore_dir: &Path) {
    let ini_path = keystore_dir.join("keystore.ini");

    if !ini_path.exists() {
        info!("No keystore.ini found at {:?} — keys not loaded", ini_path);
        return;
    }

    let mut ini = Ini::new();
    if let Err(e) = ini.load(ini_path.to_string_lossy().as_ref()) {
        error!("Failed to parse keystore.ini: {}", e);
        return;
    }

    let mut store = get_keystore_mut();
    let mut loaded_count = 0u32;

    // ── KIA keys ──────────────────────────────────────────────────────────
    if let Some(val) = ini.get("kia", "mf_key").and_then(|s| parse_hex_u64(&s)) {
        if val != 0 {
            store.kia_mf_key = val;
            loaded_count += 1;
            info!("Loaded KIA V3/V4 manufacturer key");
        }
    }

    if let Some(val) = ini.get("kia", "v5_key").and_then(|s| parse_hex_u64(&s)) {
        if val != 0 {
            store.kia_v5_key = val;
            loaded_count += 1;
            info!("Loaded KIA V5 mixer key");
        }
    }

    if let Some(val) = ini.get("kia", "v6_a_key").and_then(|s| parse_hex_u64(&s)) {
        if val != 0 {
            store.kia_v6_a_key = val;
            loaded_count += 1;
            info!("Loaded KIA V6 AES key A");
        }
    }

    if let Some(val) = ini.get("kia", "v6_b_key").and_then(|s| parse_hex_u64(&s)) {
        if val != 0 {
            store.kia_v6_b_key = val;
            loaded_count += 1;
            info!("Loaded KIA V6 AES key B");
        }
    }

    // ── Star Line keys ────────────────────────────────────────────────────
    if let Some(val) = ini.get("star_line", "mf_key").and_then(|s| parse_hex_u64(&s)) {
        if val != 0 {
            store.star_line_mf_key = val;
            loaded_count += 1;
            info!("Loaded Star Line manufacturer key");
        }
    }

    // ── VAG AUT64 keys (binary file) ──────────────────────────────────────
    if let Some(vag_file) = ini.get("vag", "keys_file") {
        let vag_file = vag_file.trim().to_string();
        if !vag_file.is_empty() {
            let vag_path = if Path::new(&vag_file).is_absolute() {
                std::path::PathBuf::from(&vag_file)
            } else {
                keystore_dir.join(&vag_file)
            };

            if vag_path.exists() {
                match std::fs::read(&vag_path) {
                    Ok(data) => {
                        store.load_vag_keys_from_data(&data);
                        if store.vag_keys_loaded {
                            loaded_count += store.vag_keys.len() as u32;
                            info!("Loaded {} VAG AUT64 keys from {:?}", store.vag_keys.len(), vag_path);
                        }
                    }
                    Err(e) => {
                        error!("Failed to read VAG key file {:?}: {}", vag_path, e);
                    }
                }
            } else {
                warn!("VAG key file not found: {:?}", vag_path);
            }
        }
    }

    if loaded_count > 0 {
        info!("Keystore loaded: {} key(s) from {:?}", loaded_count, keystore_dir);
    } else {
        info!("Keystore loaded but no non-zero keys found. Edit keystore.ini to add your keys.");
    }
}
