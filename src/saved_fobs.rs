//! Saved-fob matching: is a decoded signal one of the fobs already saved as a `.fob` file?
//!
//! Mirrors ProtoPirate's `helpers/protopirate_saved_match.c` (`protopirate_saved_match_signal`,
//! run on every new history item when "Check Saved" is on). A received signal matches a saved file
//! when:
//! 1. the protocol is the same. KAT compares canonical names as `ProtocolRegistry::get()` resolves
//!    them ([`canonical_name`]), so retired names and display variants match: "PSA2" = "PSA",
//!    "Kia V3" = "Kia V3/V4", "Land Rover V0" = "Honda V2", "Star Line [SL_A2-A4]" = "Star Line";
//! 2. the identity is the same: the serial when the received signal has one (the saved file must
//!    then carry the same serial), otherwise the key (ProtoPirate compares the `Key` string; KAT the
//!    data hex that `.fob` files store as `key`);
//! 3. when both carry a counter, the counters differ by at most [`CNT_MATCH_MARGIN`] (50).
//!
//! A signal with neither serial nor key (an unknown capture) never matches. Two KAT differences,
//! documented in docs/session.md: KAT keeps 16-bit counters, so the difference is taken modulo 2^16
//! (a counter that wrapped in its stored 16 bits still matches); and where ProtoPirate returns the
//! first match in directory order, KAT returns the one with the smallest counter difference (then
//! path order), so the nearest saved press is named.
//!
//! The index ([`scan_dirs`] / [`spawn_scan`]) holds the decoded fields and vehicle metadata of every
//! `.fob` (v1 and v2) in KAT's export and import directories, recursively; it is built off the UI
//! thread at startup and after each export.

use std::path::{Path, PathBuf};
use std::sync::mpsc::{self, Receiver};

use serde::Deserialize;

use crate::capture::Capture;
use crate::protocols::ProtocolRegistry;

/// ProtoPirate's `CNT_MATCH_MARGIN`: largest counter difference that still matches.
pub const CNT_MATCH_MARGIN: i32 = 50;
/// Directory depth limit for the scan (guards against symlink loops).
const MAX_SCAN_DEPTH: usize = 16;

/// Decoded fields of one saved `.fob` file.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SavedFob {
    pub path: PathBuf,
    /// File name without extension (ProtoPirate's matched name).
    pub name: String,
    /// Protocol as stored in the file.
    pub protocol: String,
    /// Canonical protocol name ([`canonical_name`]); set by [`canonicalize`].
    pub canonical: String,
    pub serial: Option<u32>,
    /// Key (data hex, upper case, no `0x`), compared when the received signal has no serial.
    pub key: Option<String>,
    pub counter: Option<u16>,
    pub year: Option<String>,
    pub make: Option<String>,
    pub model: Option<String>,
    pub region: Option<String>,
    pub command: Option<String>,
    pub notes: Option<String>,
}

/// A decoded capture matched to a saved file.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SavedFobMatch {
    pub path: PathBuf,
    /// Saved file name without extension.
    pub name: String,
    pub year: Option<String>,
    pub make: Option<String>,
    pub model: Option<String>,
    pub region: Option<String>,
    pub command: Option<String>,
    pub notes: Option<String>,
    /// Received counter − saved counter (signed, modulo 2^16) when both have a counter.
    pub counter_diff: Option<i32>,
}

impl SavedFobMatch {
    /// "cnt +3", "cnt -2", "cnt +0", or "cnt n/a" without counters.
    pub fn counter_label(&self) -> String {
        match self.counter_diff {
            Some(d) => format!("cnt {:+}", d),
            None => "cnt n/a".to_string(),
        }
    }

    /// "2019 VW Golf (EU), Lock" from the saved metadata (empty parts left out).
    pub fn vehicle_label(&self) -> String {
        let car: Vec<&str> = [&self.year, &self.make, &self.model]
            .into_iter()
            .filter_map(|s| s.as_deref())
            .map(str::trim)
            .filter(|s| !s.is_empty())
            .collect();
        let mut label = car.join(" ");
        if let Some(region) = self.region.as_deref().map(str::trim).filter(|s| !s.is_empty()) {
            label.push_str(&format!(" ({})", region));
        }
        if let Some(cmd) = self.command.as_deref().map(str::trim).filter(|s| !s.is_empty()) {
            if !label.is_empty() {
                label.push_str(", ");
            }
            label.push_str(cmd);
        }
        label.trim().to_string()
    }
}

/// Canonical protocol name: the registry decoder `get()` resolves the name to (legacy names and
/// " [variant]" display suffixes included), else the name without its suffix. The generic KeeLoq
/// fallback's "Keeloq (<keystore name>)" spelling resolves to the registry's KeeLoq as well.
pub fn canonical_name(registry: &ProtocolRegistry, name: &str) -> String {
    let base = name.split(" [").next().unwrap_or(name).trim();
    if let Some(d) = registry.get(base) {
        return d.name().to_string();
    }
    if base.to_ascii_lowercase().starts_with("keeloq") {
        if let Some(d) = registry.get("KeeLoq") {
            return d.name().to_string();
        }
    }
    base.to_string()
}

/// Fill every entry's [`SavedFob::canonical`].
pub fn canonicalize(index: &mut [SavedFob], registry: &ProtocolRegistry) {
    for fob in index {
        fob.canonical = canonical_name(registry, &fob.protocol);
    }
}

/// What the matcher compares from a received capture.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MatchKey {
    pub canonical: String,
    pub serial: Option<u32>,
    pub key: Option<String>,
    pub counter: Option<u16>,
}

impl MatchKey {
    /// `None` for captures that cannot match: no protocol, or neither serial nor key.
    pub fn for_capture(capture: &Capture, registry: &ProtocolRegistry) -> Option<Self> {
        let protocol = capture.protocol.as_deref()?;
        let key = (capture.data_count_bit > 0).then(|| capture.data_hex());
        if capture.serial.is_none() && key.is_none() {
            return None;
        }
        Some(Self {
            canonical: canonical_name(registry, protocol),
            serial: capture.serial,
            key,
            counter: capture.counter,
        })
    }
}

/// Received − saved counter, signed and modulo 2^16 (KAT stores 16-bit counters).
pub fn counter_diff(received: u16, saved: u16) -> i32 {
    i32::from(received.wrapping_sub(saved) as i16)
}

/// Hex keys compare by value: no `0x`, case, or leading zeros (files written before v1.8.7 padded
/// frames longer than 64 bits with zeros in front of their first 64 bits).
fn normalize_hex(s: &str) -> String {
    let t = s.trim();
    let t = t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")).unwrap_or(t);
    let t = t.trim_start_matches('0');
    if t.is_empty() { "0".to_string() } else { t.to_ascii_uppercase() }
}

/// ProtoPirate's rules (see the module doc) over the whole index; the match with the smallest
/// counter difference wins (a match without counters ranks after every counter match), then path.
pub fn find_match(index: &[SavedFob], rx: &MatchKey) -> Option<SavedFobMatch> {
    let mut best: Option<(i32, &SavedFob, Option<i32>)> = None;
    for fob in index {
        if !fob.canonical.eq_ignore_ascii_case(&rx.canonical) {
            continue;
        }
        let identity = match rx.serial {
            Some(serial) => fob.serial == Some(serial),
            None => match (&rx.key, &fob.key) {
                (Some(a), Some(b)) => normalize_hex(a) == normalize_hex(b),
                _ => false,
            },
        };
        if !identity {
            continue;
        }
        let diff = match (rx.counter, fob.counter) {
            (Some(r), Some(s)) => {
                let d = counter_diff(r, s);
                if d.abs() > CNT_MATCH_MARGIN {
                    continue;
                }
                Some(d)
            }
            _ => None,
        };
        let rank = diff.map_or(CNT_MATCH_MARGIN + 1, i32::abs);
        let better = match &best {
            None => true,
            Some((r, b, _)) => rank < *r || (rank == *r && fob.path < b.path),
        };
        if better {
            best = Some((rank, fob, diff));
        }
    }
    best.map(|(_, fob, diff)| SavedFobMatch {
        path: fob.path.clone(),
        name: fob.name.clone(),
        year: fob.year.clone(),
        make: fob.make.clone(),
        model: fob.model.clone(),
        region: fob.region.clone(),
        command: fob.command.clone(),
        notes: fob.notes.clone(),
        counter_diff: diff,
    })
}

// ─── .fob parsing ────────────────────────────────────────────────────────────

/// The parts of a .fob (v2: `signal` + `vehicle`; v1: everything under `capture`) the index needs.
/// Every field is optional so either layout (and hand-edited files) parse; raw pairs are skipped.
#[derive(Deserialize, Default)]
struct FobHead {
    #[serde(default)]
    signal: Option<FobFields>,
    #[serde(default)]
    vehicle: Option<FobFields>,
    #[serde(default)]
    capture: Option<FobFields>,
}

#[derive(Deserialize, Default)]
struct FobFields {
    #[serde(default)]
    protocol: Option<String>,
    #[serde(default)]
    serial: Option<String>,
    #[serde(default)]
    key: Option<String>,
    #[serde(default)]
    data_hex: Option<String>,
    #[serde(default)]
    counter: Option<u64>,
    #[serde(default)]
    year: Option<serde_json::Value>,
    #[serde(default)]
    make: Option<String>,
    #[serde(default)]
    model: Option<String>,
    #[serde(default)]
    region: Option<String>,
    #[serde(default)]
    command: Option<String>,
    #[serde(default)]
    notes: Option<String>,
}

fn non_empty(s: Option<String>) -> Option<String> {
    s.map(|s| s.trim().to_string()).filter(|s| !s.is_empty())
}

fn parse_hex_u32(s: Option<&str>) -> Option<u32> {
    let s = normalize_hex(s?);
    u32::from_str_radix(&s, 16).ok()
}

/// Read one `.fob` file (v1 or v2). `None` if it cannot be read, is not a .fob, or holds an
/// unknown-protocol capture (which can never match).
pub fn parse_fob(path: &Path) -> Option<SavedFob> {
    let text = std::fs::read_to_string(path).ok()?;
    parse_fob_str(path, &text)
}

fn parse_fob_str(path: &Path, text: &str) -> Option<SavedFob> {
    let head: FobHead = serde_json::from_str(text).ok()?;
    // v2 keeps the decoded fields in `signal`; v1 kept everything in `capture`.
    let v2 = head.signal.as_ref().and_then(|s| s.protocol.as_ref()).is_some();
    let (sig, veh) = if v2 {
        (head.signal?, head.vehicle.unwrap_or_default())
    } else {
        let cap = head.capture?;
        let veh = FobFields {
            year: cap.year.clone(),
            make: cap.make.clone(),
            model: cap.model.clone(),
            ..FobFields::default()
        };
        (cap, veh)
    };
    let protocol = non_empty(sig.protocol)?;
    if protocol.eq_ignore_ascii_case("unknown") {
        return None;
    }
    let key = non_empty(sig.key.or(sig.data_hex))
        .filter(|k| k != "-")
        .map(|k| normalize_hex(&k));
    let year = match veh.year {
        Some(serde_json::Value::Number(n)) => Some(n.to_string()),
        Some(serde_json::Value::String(s)) => non_empty(Some(s)),
        _ => None,
    };
    Some(SavedFob {
        path: path.to_path_buf(),
        name: path.file_stem().map(|s| s.to_string_lossy().to_string()).unwrap_or_default(),
        canonical: protocol.clone(),
        protocol,
        serial: parse_hex_u32(sig.serial.as_deref()),
        key,
        counter: sig.counter.and_then(|c| u16::try_from(c).ok()),
        year,
        make: non_empty(veh.make),
        model: non_empty(veh.model),
        region: non_empty(veh.region),
        command: non_empty(veh.command),
        notes: non_empty(veh.notes),
    })
}

// ─── Scanning ────────────────────────────────────────────────────────────────

fn walk_fobs(dir: &Path, depth: usize, out: &mut Vec<PathBuf>) {
    if depth > MAX_SCAN_DEPTH {
        return;
    }
    let Ok(entries) = std::fs::read_dir(dir) else {
        return;
    };
    for entry in entries.flatten() {
        let path = entry.path();
        if path.is_dir() {
            walk_fobs(&path, depth + 1, out);
        } else if path.extension().is_some_and(|e| e.eq_ignore_ascii_case("fob")) {
            out.push(path);
        }
    }
}

/// Index every `.fob` under `dirs` (recursively; a file reachable twice is indexed once), sorted
/// by path. Unreadable files and unknown-protocol captures are left out.
pub fn scan_dirs(dirs: &[PathBuf]) -> Vec<SavedFob> {
    let mut paths = Vec::new();
    for dir in dirs {
        walk_fobs(dir, 0, &mut paths);
    }
    let mut seen = std::collections::HashSet::new();
    paths.retain(|p| seen.insert(std::fs::canonicalize(p).unwrap_or_else(|_| p.clone())));
    paths.sort();
    paths.iter().filter_map(|p| parse_fob(p)).collect()
}

/// Run [`scan_dirs`] on a background thread; the receiver gets the index once.
pub fn spawn_scan(dirs: Vec<PathBuf>) -> Receiver<Vec<SavedFob>> {
    let (tx, rx) = mpsc::channel();
    let spawned = std::thread::Builder::new()
        .name("kat-fob-index".to_string())
        .spawn(move || {
            let _ = tx.send(scan_dirs(&dirs));
        });
    if let Err(e) = spawned {
        tracing::warn!("Could not start the saved-fob index thread: {}", e);
    }
    rx
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture::{CaptureStatus, StoredLevelDuration};
    use crate::export::fob::{export_fob, FobMetadata};

    fn temp_dir(tag: &str) -> PathBuf {
        let n = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let dir = std::env::temp_dir().join(format!("kat_fobs_{}_{}_{}", tag, std::process::id(), n));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn saved(protocol: &str, serial: Option<u32>, key: Option<&str>, counter: Option<u16>, path: &str) -> SavedFob {
        let reg = ProtocolRegistry::new();
        SavedFob {
            path: PathBuf::from(path),
            name: Path::new(path).file_stem().unwrap().to_string_lossy().to_string(),
            protocol: protocol.to_string(),
            canonical: canonical_name(&reg, protocol),
            serial,
            key: key.map(str::to_string),
            counter,
            year: None,
            make: None,
            model: None,
            region: None,
            command: None,
            notes: None,
        }
    }

    fn rx(protocol: &str, serial: Option<u32>, key: Option<&str>, counter: Option<u16>) -> MatchKey {
        MatchKey {
            canonical: canonical_name(&ProtocolRegistry::new(), protocol),
            serial,
            key: key.map(str::to_string),
            counter,
        }
    }

    fn capture(protocol: Option<&str>, serial: Option<u32>, counter: Option<u16>) -> Capture {
        let mut c = Capture::from_pairs(1, 433_920_000, vec![StoredLevelDuration { level: true, duration_us: 400 }]);
        c.protocol = protocol.map(str::to_string);
        c.serial = serial;
        c.counter = counter;
        if protocol.is_some() {
            c.data = 0x00AB_CDEF_0102_0304;
            c.data_count_bit = 64;
            c.status = CaptureStatus::Decoded;
        }
        c
    }

    #[test]
    fn serial_and_counter_margin() {
        let index = vec![saved("Kia V1", Some(0xB444706E), Some("B444706E016602"), Some(0x66), "a/kia.fob")];
        let m = find_match(&index, &rx("Kia V1", Some(0xB444706E), None, Some(0x69))).unwrap();
        assert_eq!(m.name, "kia");
        assert_eq!(m.counter_diff, Some(3));
        assert_eq!(m.counter_label(), "cnt +3");
        // ±50 matches, 51 does not (both directions).
        assert!(find_match(&index, &rx("Kia V1", Some(0xB444706E), None, Some(0x66 + 50))).is_some());
        assert!(find_match(&index, &rx("Kia V1", Some(0xB444706E), None, Some(0x66 + 51))).is_none());
        assert_eq!(find_match(&index, &rx("Kia V1", Some(0xB444706E), None, Some(0x66 - 50))).unwrap().counter_label(), "cnt -50");
        assert!(find_match(&index, &rx("Kia V1", Some(0xB444706E), None, Some(0x66 - 51))).is_none());
        // Other serial, other protocol: no match.
        assert!(find_match(&index, &rx("Kia V1", Some(0xB444706F), None, Some(0x66))).is_none());
        assert!(find_match(&index, &rx("Kia V2", Some(0xB444706E), None, Some(0x66))).is_none());
        // Counter on one side only: ProtoPirate skips the counter check.
        assert_eq!(find_match(&index, &rx("Kia V1", Some(0xB444706E), None, None)).unwrap().counter_label(), "cnt n/a");
        let no_cnt = vec![saved("Kia V1", Some(0xB444706E), None, None, "b.fob")];
        assert!(find_match(&no_cnt, &rx("Kia V1", Some(0xB444706E), None, Some(9999))).is_some());
        // The received signal has a serial but the saved file has none: no match (ProtoPirate).
        let no_serial = vec![saved("Kia V1", None, Some("B444706E016602"), Some(0x66), "c.fob")];
        assert!(find_match(&no_serial, &rx("Kia V1", Some(0xB444706E), Some("B444706E016602"), Some(0x66))).is_none());
    }

    #[test]
    fn key_identity_without_serial() {
        let index = vec![saved("PSA", None, Some("0xa1b2c3d4e5f60718"), Some(10), "psa.fob")];
        assert!(find_match(&index, &rx("PSA", None, Some("A1B2C3D4E5F60718"), Some(12))).is_some());
        assert!(find_match(&index, &rx("PSA", None, Some("A1B2C3D4E5F60719"), Some(12))).is_none());
        assert!(find_match(&index, &rx("PSA", None, None, Some(12))).is_none());
        // A file written before v1.8.7 padded an 80-bit frame's key to 20 digits.
        let old = vec![saved("PSA", None, Some("0000A1B2C3D4E5F60718"), Some(10), "psa.fob")];
        assert!(find_match(&old, &rx("PSA", None, Some("A1B2C3D4E5F60718"), Some(12))).is_some());
    }

    #[test]
    fn aliases_and_variants_match_their_canonical_protocol() {
        for (saved_name, received) in [
            ("PSA2", "PSA"),
            ("PSA", "PSA2"),
            ("Kia V3", "Kia V3/V4"),
            ("Kia V4", "Kia V3"),
            ("Land Rover V0", "Honda V2"),
            ("Porsche Touareg [First]", "Porsche Cayenne"),
            ("Star Line [SL_A2-A4]", "Star Line"),
            ("Suzuki", "Suzuki V0"),
            ("KeeLoq (DoorHan)", "Keeloq (DoorHan)"),
            ("kia v1", "Kia V1"),
        ] {
            let index = vec![saved(saved_name, Some(7), None, Some(1), "x.fob")];
            assert!(
                find_match(&index, &rx(received, Some(7), None, Some(2))).is_some(),
                "{saved_name} vs {received}"
            );
        }
        let reg = ProtocolRegistry::new();
        assert_eq!(canonical_name(&reg, "Renault V1 (ARF) [0x0C]"), "Renault V1 (ARF)");
        assert_eq!(canonical_name(&reg, "Something New [x]"), "Something New");
        assert_ne!(canonical_name(&reg, "Kia V5"), canonical_name(&reg, "Kia V6"));
    }

    #[test]
    fn counter_wrap_and_nearest_match() {
        let wrapped = vec![saved("Ford V0", Some(1), None, Some(0xFFFE), "w.fob")];
        let m = find_match(&wrapped, &rx("Ford V0", Some(1), None, Some(0x0003))).unwrap();
        assert_eq!(m.counter_diff, Some(5));
        assert_eq!(counter_diff(0xFFFE, 0x0003), -5);

        let index = vec![
            saved("Ford V0", Some(1), None, None, "a_no_counter.fob"),
            saved("Ford V0", Some(1), None, Some(100), "b_far.fob"),
            saved("Ford V0", Some(1), None, Some(130), "c_near.fob"),
            saved("Ford V0", Some(1), None, Some(130), "d_same_but_later_path.fob"),
        ];
        let m = find_match(&index, &rx("Ford V0", Some(1), None, Some(132))).unwrap();
        assert_eq!((m.name.as_str(), m.counter_diff), ("c_near", Some(2)));
        // Out of every counter's margin: only the counter-less file still matches.
        let m = find_match(&index, &rx("Ford V0", Some(1), None, Some(1000))).unwrap();
        assert_eq!(m.name, "a_no_counter");
    }

    #[test]
    fn unknown_or_identity_less_captures_never_match() {
        let reg = ProtocolRegistry::new();
        assert!(MatchKey::for_capture(&capture(None, Some(1), Some(1)), &reg).is_none());
        let mut c = capture(Some("PSA"), None, None);
        c.data_count_bit = 0;
        assert!(MatchKey::for_capture(&c, &reg).is_none());
        let k = MatchKey::for_capture(&capture(Some("PSA2"), None, Some(4)), &reg).unwrap();
        assert_eq!(k.canonical, "PSA");
        assert_eq!(k.key.as_deref(), Some("00ABCDEF01020304"));
    }

    #[test]
    fn index_reads_exported_v2_and_legacy_v1_files() {
        let export = temp_dir("export");
        let import = temp_dir("import");
        std::fs::create_dir_all(import.join("KIA")).unwrap();

        // v2, written by KAT's own exporter with metadata, in a sub-folder of the import dir.
        let mut c = capture(Some("Kia V1"), Some(0xB444706E), Some(0x66));
        c.status = CaptureStatus::EncoderCapable;
        let meta = FobMetadata {
            year: Some(2019),
            make: "Hyundai".to_string(),
            model: "Elantra".to_string(),
            region: "NA".to_string(),
            command: "Lock".to_string(),
            notes: "spare key".to_string(),
        };
        export_fob(&c, &import.join("KIA").join("2019_Hyundai_Elantra.fob"), true, Some(&meta)).unwrap();
        // v2 without serial (key identity) in the export dir.
        let mut psa = capture(Some("PSA"), None, Some(3));
        psa.data = 0x1122_3344_5566_7788;
        export_fob(&psa, &export.join("psa.fob"), false, None).unwrap();
        // v1 legacy layout.
        std::fs::write(
            export.join("old.fob"),
            r#"{"version": "1.0", "format": "kat-fob", "capture": {"timestamp": "2025-01-01T00:00:00Z",
                "frequency": 433920000, "protocol": "Land Rover V0", "year": 2015, "make": "Honda",
                "model": "Civic", "serial": "0ABCDEF", "key": "00112233", "button": 2,
                "button_name": "Unlock", "counter": 77, "encryption": "x", "crc_valid": true,
                "data_bits": 81}}"#,
        )
        .unwrap();
        // Ignored: unknown protocol, not JSON, a .sub file.
        export_fob(&capture(None, None, None), &export.join("unknown.fob"), false, None).unwrap();
        std::fs::write(export.join("broken.fob"), "{nope").unwrap();
        std::fs::write(export.join("raw.sub"), "Filetype: Flipper SubGhz RAW File\n").unwrap();

        // The export dir listed twice (and nested in nothing): every file indexed once.
        let mut index = scan_dirs(&[export.clone(), import.clone(), export.clone()]);
        canonicalize(&mut index, &ProtocolRegistry::new());
        let names: Vec<&str> = index.iter().map(|f| f.name.as_str()).collect();
        assert_eq!(names.len(), 3, "{names:?}");
        let kia = index.iter().find(|f| f.name == "2019_Hyundai_Elantra").unwrap();
        assert_eq!((kia.serial, kia.counter), (Some(0xB444706E), Some(0x66)));
        assert_eq!(kia.year.as_deref(), Some("2019"));
        assert_eq!(kia.notes.as_deref(), Some("spare key"));
        let old = index.iter().find(|f| f.name == "old").unwrap();
        assert_eq!((old.canonical.as_str(), old.serial, old.counter), ("Honda V2", Some(0x0ABCDEF), Some(77)));
        assert_eq!((old.year.as_deref(), old.model.as_deref()), (Some("2015"), Some("Civic")));
        let p = index.iter().find(|f| f.name == "psa").unwrap();
        assert_eq!((p.serial, p.key.as_deref()), (None, Some("1122334455667788")));

        // End to end: the same fob three presses later matches the exported file.
        let reg = ProtocolRegistry::new();
        let later = capture(Some("Kia V1"), Some(0xB444706E), Some(0x69));
        let m = find_match(&index, &MatchKey::for_capture(&later, &reg).unwrap()).unwrap();
        assert_eq!(m.name, "2019_Hyundai_Elantra");
        assert_eq!(m.counter_label(), "cnt +3");
        assert_eq!(m.vehicle_label(), "2019 Hyundai Elantra (NA), Lock");
        let same_psa = MatchKey::for_capture(&psa, &reg).unwrap();
        assert_eq!(find_match(&index, &same_psa).unwrap().counter_label(), "cnt +0");

        // Background scan returns the same index.
        let rx = spawn_scan(vec![export.clone(), import.clone()]);
        let bg = rx.recv_timeout(std::time::Duration::from_secs(10)).unwrap();
        assert_eq!(bg.len(), 3);
        let _ = std::fs::remove_dir_all(&export);
        let _ = std::fs::remove_dir_all(&import);
    }
}
