//! Per-remote HITAG2 key database — Flipper-ARF / doctor-firmware `hitag2_key_db.c` (ARF ported it
//! from doctor-firmware so that "a cracked Hitag2 key/seed [is] reused across different captures of
//! the SAME car (keyed by UID), instead of re-cracking every time").
//!
//! The firmwares keep it in `subghz/assets/hitag2_keys.txt`; KAT keeps the same file in its config
//! folder ([`FILE_NAME`]), so it can be copied either way. Header `# Flipper Hitag2 UID keys` /
//! `# UID KEY EPOCH SEED`, then one line per remote, `hitag2_key_db_format_line`: the UID (8 hex
//! digits), the key (12 hex digits or `-`), the epoch (decimal, 18 bits, `-` without a key) and the
//! seed (8 hex digits or `-`). The first line of a UID counts (`hitag2_key_db_find`); saving a key
//! replaces that line, drops later ones and keeps the line's seed (`hitag2_key_db_save_key` +
//! `_upsert`).
//!
//! The key bytes are as each firmware's protocol stores them: Fiat V1 (and so KAT's Fiat V2) in the
//! order [`super::hitag2::fiat_bcm_authenticator`] takes (doctor-firmware's "cipher" order),
//! Renault V1 in the standard HITAG2 order of [`super::hitag2::renault_authenticator`]
//! (doctor-firmware's "Hitag2 Key" line). A protocol uses a record only once it reproduces the
//! captured hop, as in the C (a record that does not verify is ignored); KAT also tries the bytes
//! in the other order ([`keys_for`]), so a key saved from Fiat V1 captures serves the same remote
//! decoded as Renault V1 (`IMPORTS/OPEL/Opel Vectra 2008.sub` decodes under both names).
//!
//! KAT reads the file once at startup ([`load`]) and writes it when "Recover key (HiTag2)" finds a
//! key ([`save_key`]). The seed column (doctor-firmware's Renault seed) is kept but not read: KAT
//! recovers Renault seeds from the frame itself.

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::RwLock;

/// The database's file name, in KAT's config folder as in the firmwares' `subghz/assets`.
pub const FILE_NAME: &str = "hitag2_keys.txt";

/// `hitag2_key_db_header`, written when the file is created.
const HEADER: &str = "# Flipper Hitag2 UID keys\n# UID KEY EPOCH SEED\n";

/// The epoch's 18 bits (`0x3FFFF`).
const EPOCH_MASK: u32 = 0x3FFFF;

/// One remote's record (`Hitag2KeyDbRecord`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Record {
    pub key: Option<[u8; 6]>,
    /// 0 without a key (the C parses it so).
    pub epoch: u32,
    pub seed: Option<u32>,
}

/// The byte order a protocol's cipher takes the key in.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KeyOrder {
    /// The standard HITAG2 order ([`super::hitag2::renault_authenticator`], Renault V1).
    Standard,
    /// [`super::hitag2::fiat_bcm_authenticator`] (Fiat V1, Fiat V2): the standard order rotated
    /// left by two bytes.
    FiatBcm,
}

impl KeyOrder {
    /// A key in the standard order, as this order takes it.
    pub fn arrange(self, key: [u8; 6]) -> [u8; 6] {
        match self {
            KeyOrder::Standard => key,
            KeyOrder::FiatBcm => [key[2], key[3], key[4], key[5], key[0], key[1]],
        }
    }

    /// A key in this order, in the standard order.
    pub fn standard(self, key: [u8; 6]) -> [u8; 6] {
        match self {
            KeyOrder::Standard => key,
            KeyOrder::FiatBcm => [key[4], key[5], key[0], key[1], key[2], key[3]],
        }
    }
}

/// The records of this session: the file's at startup, then every key saved.
static DB: RwLock<Vec<(u32, Record)>> = RwLock::new(Vec::new());

/// Bumped on every change of [`DB`], so callers can memoise what they derive from it.
static GENERATION: AtomicU64 = AtomicU64::new(0);

/// The database's version: it changes whenever a record is loaded or saved.
pub fn generation() -> u64 {
    GENERATION.load(Ordering::Relaxed)
}

/// `hitag2_key_db_uid_ok`.
fn uid_ok(uid: u32) -> bool {
    uid != 0 && uid != u32::MAX
}

/// `N` bytes as `2N` hex digits, or `-` (absent). `None` = a bad field.
fn hex_or_absent<const N: usize>(field: &str) -> Option<Option<[u8; N]>> {
    if field == "-" {
        return Some(None);
    }
    if field.len() != 2 * N || !field.bytes().all(|b| b.is_ascii_hexdigit()) {
        return None;
    }
    let mut out = [0u8; N];
    for (i, byte) in out.iter_mut().enumerate() {
        *byte = u8::from_str_radix(&field[2 * i..2 * i + 2], 16).ok()?;
    }
    Some(Some(out))
}

/// `hitag2_key_db_parse_line`: a record line as (UID, record). `None` for comments, blank lines,
/// bad lines and the UIDs 00000000 / FFFFFFFF.
pub fn parse_line(line: &str) -> Option<(u32, Record)> {
    let mut fields = line.split_whitespace();
    let uid = u32::from_be_bytes(hex_or_absent::<4>(fields.next()?)??);
    let key = hex_or_absent::<6>(fields.next()?)?;
    let epoch = match fields.next()? {
        "-" => 0,
        digits if digits.bytes().all(|b| b.is_ascii_digit()) => {
            // The C accumulates in a uint32_t and keeps 18 bits with a key, none without.
            let value = digits.bytes().fold(0u32, |v, b| v.wrapping_mul(10).wrapping_add(u32::from(b - b'0')));
            if key.is_some() {
                value & EPOCH_MASK
            } else {
                0
            }
        }
        _ => return None,
    };
    let seed = hex_or_absent::<4>(fields.next()?)?.map(u32::from_be_bytes);
    if fields.next().is_some() || !uid_ok(uid) {
        return None;
    }
    Some((uid, Record { key, epoch, seed }))
}

/// `hitag2_key_db_format_line` (with its newline).
pub fn format_line(uid: u32, record: &Record) -> String {
    let (key, epoch) = match record.key {
        Some(k) => (k.iter().map(|b| format!("{b:02X}")).collect(), (record.epoch & EPOCH_MASK).to_string()),
        None => ("-".to_string(), "-".to_string()),
    };
    let seed = record.seed.map_or_else(|| "-".to_string(), |s| format!("{s:08X}"));
    format!("{uid:08X} {key} {epoch} {seed}\n")
}

/// The record of remote `uid` in this session (`hitag2_key_db_find`): its first line, if that has
/// a key or a seed.
pub fn find(uid: u32) -> Option<Record> {
    if !uid_ok(uid) {
        return None;
    }
    let db = DB.read().unwrap_or_else(|e| e.into_inner());
    db.iter()
        .find(|(u, _)| *u == uid)
        .map(|(_, r)| *r)
        .filter(|r| r.key.is_some() || r.seed.is_some())
}

/// The keys a protocol whose cipher takes `order` tries for remote `uid`, with the record's
/// epoch: the stored bytes, then the same key read as the other order (module doc). Empty when
/// the remote has no key.
pub fn keys_for(uid: u32, order: KeyOrder) -> Vec<([u8; 6], u32)> {
    let Some(Record { key: Some(key), epoch, .. }) = find(uid) else {
        return Vec::new();
    };
    let other = match order {
        // Stored in the standard order (a Renault V1 record): rotate into the Fiat order.
        KeyOrder::FiatBcm => KeyOrder::FiatBcm.arrange(key),
        // Stored in the Fiat order: back to the standard one.
        KeyOrder::Standard => KeyOrder::FiatBcm.standard(key),
    };
    vec![(key, epoch), (other, epoch)]
}

/// Use `record` for remote `uid` for the rest of the session, in place of any earlier one.
fn remember(uid: u32, record: Record) {
    let mut db = DB.write().unwrap_or_else(|e| e.into_inner());
    db.retain(|(u, _)| *u != uid);
    db.push((uid, record));
    GENERATION.fetch_add(1, Ordering::Relaxed);
}

/// Read `<config dir>/hitag2_keys.txt` if it exists (once, at startup, before any transmit).
/// Returns a status line for the user when there was a file.
pub fn load(config_dir: &Path) -> Option<String> {
    let path = config_dir.join(FILE_NAME);
    let text = std::fs::read_to_string(&path).ok()?;
    let mut keys = 0;
    {
        let mut db = DB.write().unwrap_or_else(|e| e.into_inner());
        for (uid, record) in text.lines().filter_map(parse_line) {
            // The first line of a UID counts, as `hitag2_key_db_find` stops there.
            if !db.iter().any(|(u, _)| *u == uid) {
                keys += usize::from(record.key.is_some());
                db.push((uid, record));
            }
        }
        GENERATION.fetch_add(1, Ordering::Relaxed);
    }
    Some(format!("Loaded {keys} HiTag2 key{} from {}", crate::plural(keys), path.display()))
}

/// `hitag2_key_db_save_key`: use `key` (in the order of the remote's protocol, module doc) with
/// `epoch` for remote `uid` from now on, and write it to `<config dir>/hitag2_keys.txt`: the
/// remote's line is replaced (its seed kept) or appended, a new file starts with the firmwares'
/// header. The session keeps the key even when the file cannot be written. Returns the file.
pub fn save_key(config_dir: &Path, uid: u32, key: [u8; 6], epoch: u32) -> std::io::Result<PathBuf> {
    if !uid_ok(uid) {
        return Err(std::io::Error::new(std::io::ErrorKind::InvalidInput, format!("UID {uid:08X} cannot be saved")));
    }
    let path = config_dir.join(FILE_NAME);
    let existing = match std::fs::read_to_string(&path) {
        Ok(text) => Some(text),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => None,
        Err(e) => {
            remember(uid, Record { key: Some(key), epoch: epoch & EPOCH_MASK, seed: find(uid).and_then(|r| r.seed) });
            return Err(e);
        }
    };
    let seed = existing
        .as_deref()
        .and_then(|text| text.lines().filter_map(parse_line).find(|(u, _)| *u == uid))
        .and_then(|(_, r)| r.seed);
    let record = Record { key: Some(key), epoch: epoch & EPOCH_MASK, seed };
    remember(uid, record);

    let mut out = String::new();
    let mut written = false;
    match &existing {
        None => out.push_str(HEADER),
        Some(text) => {
            for line in text.lines() {
                if parse_line(line).is_some_and(|(u, _)| u == uid) {
                    if !written {
                        out.push_str(&format_line(uid, &record));
                        written = true;
                    }
                    continue;
                }
                out.push_str(line);
                out.push('\n');
            }
        }
    }
    if !written {
        out.push_str(&format_line(uid, &record));
    }
    std::fs::create_dir_all(config_dir)?;
    // Written next to it and renamed over it, as the C does (`hitag2_keys.txt.tmp`).
    let tmp = config_dir.join(format!("{FILE_NAME}.tmp"));
    std::fs::write(&tmp, out)?;
    std::fs::rename(&tmp, &path)?;
    Ok(path)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// Give remote `uid` a key for this test process (other tests must use other UIDs: the
    /// database is global, as the firmwares' file is).
    pub(crate) fn remember_for_test(uid: u32, key: [u8; 6]) {
        remember(uid, Record { key: Some(key), epoch: 0, seed: None });
    }

    /// Lines as `hitag2_key_db_format_line` writes them and `_parse_line` reads them.
    #[test]
    fn lines_round_trip_like_the_c() {
        let rec = Record { key: Some([0xE9, 0x9B, 0xDE, 0x9C, 0x28, 0x13]), epoch: 3, seed: None };
        assert_eq!(format_line(0x13E9_EC28, &rec), "13E9EC28 E99BDE9C2813 3 -\n");
        assert_eq!(parse_line("13E9EC28 E99BDE9C2813 3 -"), Some((0x13E9_EC28, rec)));
        let seed_only = Record { key: None, epoch: 0, seed: Some(0x0102_0304) };
        assert_eq!(format_line(0xA1B2_C3D4, &seed_only), "A1B2C3D4 - - 01020304\n");
        assert_eq!(parse_line("  a1b2c3d4\t-  77 01020304 "), Some((0xA1B2_C3D4, seed_only)), "epoch ignored without a key");
        // The epoch keeps 18 bits.
        assert_eq!(parse_line("13E9EC28 E99BDE9C2813 262145 -").map(|(_, r)| r.epoch), Some(1));
        for bad in [
            "# Flipper Hitag2 UID keys",
            "",
            "13E9EC28 E99BDE9C2813 3",
            "13E9EC28 E99BDE9C28 3 -",
            "13E9EC28 E99BDE9C2813 x -",
            "13E9EC28 E99BDE9C2813 3 - extra",
            "00000000 E99BDE9C2813 0 -",
            "FFFFFFFF E99BDE9C2813 0 -",
        ] {
            assert_eq!(parse_line(bad), None, "{bad:?}");
        }
    }

    /// The two byte orders: Fiat's is the standard one rotated by two bytes (as
    /// `hitag2_hell::fiat_bcm_key` and doctor-firmware's `fiat_v1_hitag2_key_disk_to_cipher`).
    #[test]
    fn key_orders() {
        let std_key = [1, 2, 3, 4, 5, 6];
        assert_eq!(KeyOrder::FiatBcm.arrange(std_key), [3, 4, 5, 6, 1, 2]);
        assert_eq!(KeyOrder::FiatBcm.standard([3, 4, 5, 6, 1, 2]), std_key);
        assert_eq!(KeyOrder::Standard.arrange(std_key), std_key);
    }

    /// Saving: a new file gets the header; the remote's line is replaced in place (seed kept,
    /// later duplicates dropped), other lines and comments stay; the session uses the key at once.
    #[test]
    fn save_replaces_the_remote_line() {
        let dir = std::env::temp_dir().join(format!("kat-hitag2-keys-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let uid = 0x5EED_0001;
        let path = save_key(&dir, uid, [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF], 0).unwrap();
        assert_eq!(std::fs::read_to_string(&path).unwrap(), format!("{HEADER}5EED0001 AABBCCDDEEFF 0 -\n"));
        assert_eq!(keys_for(uid, KeyOrder::Standard)[0], ([0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF], 0));
        assert_eq!(keys_for(uid, KeyOrder::FiatBcm)[1], ([0xCC, 0xDD, 0xEE, 0xFF, 0xAA, 0xBB], 0));

        std::fs::write(
            &path,
            "# Flipper Hitag2 UID keys\n# my notes\n12345678 010203040506 0 -\n5EED0001 - - 0A0B0C0D\n\n5EED0001 111111111111 0 -\n",
        )
        .unwrap();
        save_key(&dir, uid, [0x21, 0x22, 0x23, 0x24, 0x25, 0x26], 2).unwrap();
        assert_eq!(
            std::fs::read_to_string(&path).unwrap(),
            "# Flipper Hitag2 UID keys\n# my notes\n12345678 010203040506 0 -\n5EED0001 212223242526 2 0A0B0C0D\n\n"
        );
        assert_eq!(find(uid), Some(Record { key: Some([0x21, 0x22, 0x23, 0x24, 0x25, 0x26]), epoch: 2, seed: Some(0x0A0B_0C0D) }));
        assert!(save_key(&dir, 0, [0; 6], 0).is_err());
        let _ = std::fs::remove_dir_all(&dir);
    }
}
