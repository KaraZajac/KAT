//! Embedded keystore: binary blob of protocol keys (KIA, Star Line, VAG).
//! Matches ProtoPirate key types (keys.c: KIA_KEY1..4). Loaded at startup instead of keystore_dir.

mod embedded;

use std::convert::TryInto;

const MAGIC: &[u8; 4] = b"KATK";
const VAG_TAG: &[u8; 4] = b"VAG ";
const VAG_SIZE: usize = 64;
const ENTRY_SIZE: usize = 4 + 8; // u32 type + u64 key

/// Parsed result from the embedded blob: (type_id, key) pairs and raw VAG bytes.
pub struct ParsedKeystore {
    pub entries: Vec<(u32, u64)>,
    pub vag_bytes: Vec<u8>,
}

/// Parse the embedded keystore blob. Returns KIA/Star Line entries and VAG raw bytes.
pub fn parse_blob(blob: &[u8]) -> Option<ParsedKeystore> {
    if blob.len() < 4 || &blob[0..4] != MAGIC {
        return None;
    }
    let n = u16::from_le_bytes(blob[4..6].try_into().ok()?) as usize;
    let mut off = 6;
    let mut entries = Vec::with_capacity(n);
    for _ in 0..n {
        if off + ENTRY_SIZE > blob.len() {
            return None;
        }
        let ty = u32::from_le_bytes(blob[off..off + 4].try_into().ok()?);
        let key = u64::from_le_bytes(blob[off + 4..off + 12].try_into().ok()?);
        entries.push((ty, key));
        off += ENTRY_SIZE;
    }
    if off + 4 + VAG_SIZE > blob.len() || &blob[off..off + 4] != VAG_TAG {
        return Some(ParsedKeystore {
            entries,
            vag_bytes: Vec::new(),
        });
    }
    off += 4;
    let vag_bytes = blob[off..off + VAG_SIZE].to_vec();
    Some(ParsedKeystore {
        entries,
        vag_bytes,
    })
}

/// Return the embedded keystore blob for loading.
pub fn embedded_blob() -> &'static [u8] {
    embedded::KEYSTORE_BLOB
}
