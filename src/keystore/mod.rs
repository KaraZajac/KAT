//! Embedded keystore: binary blob of protocol keys (KIA, Star Line, VAG).
//! Matches ProtoPirate key types (keys.c: KIA_KEY1..4). Loaded at startup instead of keystore_dir.

mod embedded;

use std::convert::TryInto;

/// Type IDs that are KeeLoq manufacturer keys (64-bit keys used for keeloq_decrypt).
const KEELOQ_MF_TYPES: &[u32] = &[0, 1, 2, 10, 20];

/// Names for each key entry in the blob (same order as entries before VAG). Used for fallback decode display.
const KEY_ENTRY_NAMES: &[&str] = &[
    "KGB/Subaru",
    "Magic_2",
    "IronLogic",
    "Stilmatic",
    "Pandora_Test_Debug_2",
    "Rosh",
    "Dea_Mio",
    "Leopard",
    "Sheriff",
    "Cenmax",
    "Centurion",
    "Guard_RF-311A",
    "Pandora_PRO",
    "Cardin_S449",
    "Pandora_DEA",
    "Pandora_GIBIDI",
    "Pandora_MCODE",
    "Pandora_Unknown_1",
    "Pandora_SUZUKI",
    "Harpoon",
    "Gibidi",
    "Pandora_Unknown_2",
    "Jarolift",
    "KEY",
    "Novoferm",
    "Pandora_SUBARU",
    "Pecinin",
    "Merlin",
    "Pandora_M101",
    "IL-100(Smart)",
    "Pantera_CLK",
    "Kingates_Stylo4k",
    "Monarch",
    "Aprimatic",
    "NICE_MHOUSE",
    "BFT",
    "Pantera",
    "KIAV5",
    "FAAC_SLH",
    "Alligator_S-275",
    "NICE_Smilo",
    "SL_A6-A9/Tomahawk_9010",
    "Motorline",
    "KIAV6A",
    "Tomahawk_TZ-9030",
    "Cenmax_St-7",
    "Pantera_XS/Jaguar",
    "APS-1100_APS-2550",
    "Reff",
    "KIAV6B",
    "Beninca_ARC",
    "Tomahawk_Z,X_3-5",
    "Sommer",
    "Pandora_PRO2",
    "DoorHan",
    "EcoStar",
    "JCM_Tech",
    "Faraon",
    "Came_Space",
    "Magic_4",
    "SL_A2-A4",
    "Elmes_Poland",
    "Normstahl",
    "KIA",
    "GSN",
    "Mutanco_Mutancode",
    "Cenmax_St-5",
    "Magic_3",
    "Partisan_RX",
    "Steelmate",
    "Teco",
    "ZX-730-750-1055",
    "Jolly_Motors",
    "Beninca",
    "Mongoose",
    "Alligator",
    "Comunello",
    "SL_B6,B9_dop",
    "FAAC_RC,XT",
    "Genius_Bravo",
    "Rossi",
    "DTM_Neo",
    "Star Line",
];

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
///
/// Keys are stored as 8 bytes **little-endian** per entry; we load with `u64::from_le_bytes`.
/// The resulting u64 value matches the reference/Pandora hex (e.g. KIA = 0xA8F5DFFC8DAA5CDB),
/// which is typically written MSB-first in docs; KeeLoq uses this value as a 64-bit key.
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

/// Return all KeeLoq manufacturer keys from the embedded blob with display names.
/// Used when an unknown signal is tried as KeeLoq with every key; on success we show "Keeloq (name)".
/// Only includes entry types that are KeeLoq MF keys (0, 1, 2, 10, 20).
pub fn keeloq_mf_keys_with_names() -> Vec<(String, u64)> {
    let blob = embedded_blob();
    let Some(parsed) = parse_blob(blob) else {
        return Vec::new();
    };
    parsed
        .entries
        .iter()
        .enumerate()
        .filter(|(_, (ty, _))| KEELOQ_MF_TYPES.contains(ty))
        .filter_map(|(i, (_, key))| {
            let name = KEY_ENTRY_NAMES.get(i).copied().unwrap_or("?").to_string();
            if *key == 0 {
                None
            } else {
                Some((name, *key))
            }
        })
        .collect()
}

/// Every keystore entry as `(name, type, key)` in blob order, unfiltered. The type is the entry's
/// learning type as stored (Flipper `KEELOQ_LEARNING_*` numbering, except ProtoPirate's KIA_KEY1..4
/// entries "KIA"/"KIAV6A"/"KIAV6B"/"KIAV5" = 10..13 and KAT's "Star Line" alias = 20). The KeeLoq
/// family decoders (keeloq, star_line) walk this list the way the C walks the Flipper keystore.
pub fn keystore_entries_with_names() -> Vec<(String, u32, u64)> {
    let Some(parsed) = parse_blob(embedded_blob()) else {
        return Vec::new();
    };
    parsed
        .entries
        .iter()
        .enumerate()
        .map(|(i, &(ty, key))| {
            let name = KEY_ENTRY_NAMES.get(i).copied().unwrap_or("?").to_string();
            (name, ty, key)
        })
        .collect()
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

    /// Reference dump of the decrypted Flipper SubGhz keystore (standard `KEY:TYPE:NAME` entries and
    /// the raw 64-byte VAG/AUT64 file). Every entry must be present in the embedded blob with the same
    /// learning type and key: Kia V3/V4 (KIA), Kia V5 (KIAV5), Kia V6 (KIAV6A/B), Star Line and the
    /// other alarm KeeLoq keys, Sheriff CFM's ZX-750 key (ZX-730-750-1055) and VAG's AUT64 keys.
    #[test]
    fn embedded_keystore_matches_reference_dump() {
        const DUMP: &[(&str, u32, &str)] = &[
        ("A8F5DFFC8DAA5CDB", 10, "KIA"),
        ("638664A880AA23FC", 11, "KIAV6A"),
        ("6B8E6CA088A22BF4", 12, "KIAV6B"),
        ("535446524B453030", 13, "KIAV5"),
        ("EEF3D4B2626C5578", 1, "Alligator"),
        ("D5A5E7B2A7C1A0BA", 2, "Mongoose"),
        ("5504045708301203", 1, "SL_A6-A9/Tomahawk_9010"),
        ("4D41474E64796E65", 1, "Pantera"),
        ("9BF7F89BF8FE78DA", 1, "SL_A2-A4"),
        ("AAFBFBA8F7CFEDFC", 1, "Cenmax_St-5"),
        ("F6E5D4B32ABADCFE", 1, "SL_B6,B9_dop"),
        ("2587010764070864", 1, "Harpoon"),
        ("6408076407018725", 1, "Tomahawk_TZ-9030"),
        ("6F5E4D3B2AABCDEF", 1, "Tomahawk_Z,X_3-5"),
        ("66B446B980AC752B", 1, "Cenmax_St-7"),
        ("12332594A9189478", 1, "Sheriff"),
        ("4130850A82610A14", 1, "Pantera_CLK"),
        ("1234567890123456", 1, "Cenmax"),
        ("54365CB7676284F9", 1, "Alligator_S-275"),
        ("1961DAC2EB198847", 2, "Guard_RF-311A"),
        ("AD3C12A17028F11E", 1, "Partisan_RX"),
        ("68732C5056334728", 1, "APS-1100_APS-2550"),
        ("67732C5056334627", 1, "Pantera_XS/Jaguar"),
        ("B51526E8F126D1D7", 0, "Teco"),
        ("1067DC33E7D88A46", 0, "Leopard"),
        ("8A326438B62287F5", 0, "Faraon"),
        ("6912FA557DC2418A", 0, "Reff"),
        ("B51A7AAB27351596", 0, "ZX-730-750-1055"),
        ];
        let entries = embedded_entries_for_compare();
        for &(hex, ty, name) in DUMP {
            let key = u64::from_str_radix(hex, 16).unwrap();
            let idx = KEY_ENTRY_NAMES
                .iter()
                .position(|n| *n == name)
                .unwrap_or_else(|| panic!("keystore has no entry named {name}"));
            assert_eq!(
                entries.get(idx).copied(),
                Some((ty, key)),
                "keystore entry {name} differs from the reference dump"
            );
        }
        const VAG_DUMP: &str = concat!(
            "01376C86ADABCC43074DE859C12F36AB",
            "02377C65CEDC42EAA453E861D9B720FC",
            "038AA37B1E561F8384B619C52E0A3FD7",
            "00000000000000000000000000000000",
        );
        let vag: Vec<u8> = (0..VAG_DUMP.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&VAG_DUMP[i..i + 2], 16).unwrap())
            .collect();
        let parsed = parse_blob(embedded_blob()).expect("embedded blob parses");
        assert_eq!(parsed.vag_bytes, vag, "VAG AUT64 keys differ from the reference dump");
    }

    #[test]
    fn dump_keystore_keys_for_pandora_compare() {
        let entries = embedded_entries_for_compare();
        eprintln!("KAT embedded keystore keys (type, hex, name):");
        for (i, (ty, key)) in entries.iter().enumerate() {
            let name = KEY_ENTRY_NAMES.get(i).copied().unwrap_or("?");
            eprintln!("  type {}  {}  {}", ty, format!("{:016X}", key), name);
        }
    }
}
