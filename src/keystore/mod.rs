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

/// Return all (type_id, key_u64) from the embedded blob for comparison with external key lists (e.g. Pandora).
/// Key is stored LE in blob; returned as u64. Format as 16-char hex with format!("{:016X}", key) to match Pandora.
#[cfg(test)]
pub fn embedded_entries_for_compare() -> Vec<(u32, u64)> {
    parse_blob(embedded_blob()).map(|p| p.entries).unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dump_keystore_keys_for_pandora_compare() {
        let entries = embedded_entries_for_compare();
        const NAMES: &[&str] = &[
            "KIA", "KIAV6A", "KIAV6B", "KIAV5", "Alligator", "Mongoose",
            "SL_A6-A9/Tomahawk_9010", "Pantera", "SL_A2-A4", "Cenmax_St-5", "SL_B6,B9_dop",
            "Harpoon", "Tomahawk_TZ-9030", "Tomahawk_Z,X_3-5", "Cenmax_St-7", "Sheriff",
            "Pantera_CLK", "Cenmax", "Alligator_S-275", "Guard_RF-311A", "Partisan_RX",
            "APS-1100_APS-2550", "Pantera_XS/Jaguar", "Teco", "Leopard", "Faraon", "Reff",
            "ZX-730-750-1055", "Star Line",
            "Pandora_M101", "Pandora_PRO", "Pandora_PRO2", "Pandora_SUBARU", "Pandora_SUZUKI",
            "Pandora_DEA", "Pandora_GIBIDI", "Pandora_MCODE", "Pandora_Unknown_1", "Pandora_Unknown_2",
            "Pandora_Test_Debug_2",
        ];
        eprintln!("KAT embedded keystore keys (type, hex, name):");
        for (i, (ty, key)) in entries.iter().enumerate() {
            let name = NAMES.get(i).copied().unwrap_or("?");
            eprintln!("  type {}  {}  {}", ty, format!("{:016X}", key), name);
        }
    }
}
