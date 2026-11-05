//! Files the app writes and reads.
//!
//! - **Exports** (`.fob`, RAW `.sub`, Flipper key `.sub`): [App::export_path] puts the name in the
//!   export folder and never replaces a file (`_2`, `_3`, … before the extension,
//!   [crate::export::unique_path]); [App::report_export] ends with "Exported to <full path>", or
//!   reports why the write failed. A failed export (an unplugged Flipper SD card as the export
//!   folder, a read-only or full disk, a name the file system refuses) used to end KAT and lose
//!   the session's unsaved work; now it is a line in the status bar.
//! - **Imports** (the startup import, `:load`, the load browser): an [Importer] reads each `.sub`
//!   (key file or RAW, any case of the extension) or `.fob` file, and [import_summary] says what
//!   came of it: "Imported 15 captures from Kia3.sub", "Nothing imported from notes.fob: <why>",
//!   "Imported 12 captures from 3 files (1 failed: x.sub: <why>)". Read errors used to go to the
//!   log only, under an "Imported" message.

use std::collections::HashSet;
use std::path::{Path, PathBuf};

use super::App;
use crate::capture::{Capture, CaptureStatus, StoredLevelDuration};
use crate::export::flipper::{self, SubContent, SubFile};
use crate::protocols::{DecodedSignal, FlipperKey, ProtocolRegistry};
use crate::radio::LevelDuration;

/// Longest file name stem KAT suggests for an export: file systems take 255 bytes, and the stem
/// leaves room for a `_NN` suffix and the extension.
const MAX_STEM_BYTES: usize = 120;

/// A suggested export name every export folder accepts, a Flipper's FAT SD card included: the
/// characters FAT and Windows refuse (`< > : " / \ | ? *`, control characters) become `-`, and a
/// long name is cut (at a character boundary).
pub(super) fn safe_stem(name: &str) -> String {
    let mut out: String = name
        .trim()
        .chars()
        .map(|c| if c.is_control() || "<>:\"/\\|?*".contains(c) { '-' } else { c })
        .collect();
    if out.len() > MAX_STEM_BYTES {
        let cut = (0..=MAX_STEM_BYTES).rev().find(|&i| out.is_char_boundary(i)).unwrap_or(0);
        out.truncate(cut);
    }
    out
}

/// The file an export of the typed name writes in `dir`: `.ext` added unless already typed (in
/// any case), and a name no file has yet.
pub(super) fn export_file(dir: &Path, typed: &str, ext: &str) -> PathBuf {
    let typed = typed.trim();
    let suffix = format!(".{ext}");
    let has_ext = typed.len() > suffix.len()
        && typed.is_char_boundary(typed.len() - suffix.len())
        && typed[typed.len() - suffix.len()..].eq_ignore_ascii_case(&suffix);
    let name = if has_ext { typed.to_string() } else { format!("{typed}{suffix}") };
    crate::export::unique_path(&dir.join(name))
}

/// Whether the list already has this signal (same protocol, data, serial and button). Unknown
/// signals (no protocol) are never duplicates, so they can be kept for research.
pub(super) fn is_duplicate(captures: &[Capture], capture: &Capture) -> bool {
    capture.protocol.is_some()
        && captures.iter().any(|c| {
            c.protocol == capture.protocol
                && c.data == capture.data
                && c.serial == capture.serial
                && c.button == capture.button
        })
}

impl App {
    /// Where an export of the typed name goes ([export_file]) in the export folder, which is
    /// created if needed. Err: the message for the status line (the folder cannot be created).
    pub(super) fn export_path(&self, typed: &str, ext: &str) -> Result<PathBuf, String> {
        let dir = self.storage.export_dir();
        if typed.trim().is_empty() {
            return Err("File name cannot be empty".to_string());
        }
        // The reason first: a long path is cut off at the end of the status line.
        std::fs::create_dir_all(dir).map_err(|e| {
            format!(
                "Export failed: {e} (the export folder cannot be created; check export_directory in config.ini): {}",
                dir.display()
            )
        })?;
        Ok(export_file(dir, typed, ext))
    }

    /// Report an export of `path` in the status line: "Exported to <full path>", or why it failed.
    /// Returns whether it was written.
    pub(super) fn report_export(&mut self, path: &Path, result: anyhow::Result<()>) -> bool {
        match result {
            Ok(()) => {
                self.status_message = Some(format!("Exported to {}", path.display()));
                true
            }
            Err(e) => {
                tracing::warn!("Export to {:?} failed: {e:#}", path);
                self.last_error = Some(format!("Export failed: {e:#}: {}", path.display()));
                false
            }
        }
    }
}

/// What importing one file did.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) enum FileImport {
    /// Captures added (one or more).
    Added(usize),
    /// Read, but nothing to add: why.
    Nothing(String),
    /// Not readable, or not a file KAT imports: why.
    Failed(String),
}

/// Imports files into the capture list (the parts of [App] it needs, so tests run it without a
/// radio or a config folder).
pub(super) struct Importer<'a> {
    pub protocols: &'a mut ProtocolRegistry,
    pub captures: &'a mut Vec<Capture>,
    pub next_id: &'a mut u32,
    /// Unknown signals are listed too (config `show_unknown_signals`).
    pub show_unknown: bool,
    /// A signal already in the list is not added again (config `ignore_duplicates`).
    pub ignore_duplicates: bool,
    /// `PSA [unchecked]` frames the files held, for `autobf` to list only if their key is found.
    pub unchecked: Vec<Capture>,
    /// For each capture's `source_file` (the path relative to it).
    pub import_dir: &'a Path,
}

impl Importer<'_> {
    /// Import one file: its name (for the summary) and what came of it.
    pub fn import(&mut self, path: &Path) -> (String, FileImport) {
        let name = path.file_name().unwrap_or(path.as_os_str()).to_string_lossy().into_owned();
        let result = match std::fs::read_to_string(path) {
            Err(e) => FileImport::Failed(format!("cannot read it ({e})")),
            Ok(text) if crate::export::has_extension(path, "sub") => match flipper::parse_sub_content(&text) {
                Ok(SubContent::Key(key)) => self.import_key(path, &key),
                Ok(SubContent::Raw(sub)) => self.import_raw(path, sub),
                Err(e) => FileImport::Failed(format!("{e:#}")),
            },
            Ok(text) => self.import_fob(path, &text),
        };
        if let FileImport::Failed(why) = &result {
            tracing::warn!("Import {:?}: {why}", path);
        }
        (name, result)
    }

    /// Add `capture` unless the list has it (and duplicates are ignored); it gets the next id and
    /// its source file. Repeats within one file are merged before this, either way.
    fn add(&mut self, mut capture: Capture, path: &Path) -> bool {
        if self.ignore_duplicates && is_duplicate(self.captures, &capture) {
            return false;
        }
        capture.id = *self.next_id;
        *self.next_id += 1;
        capture.source_file = Some(App::path_relative_to_import(path, self.import_dir));
        self.captures.push(capture);
        true
    }

    /// A Flipper key file, rebuilt by its protocol (`docs/flipper_key_files.md`).
    fn import_key(&mut self, path: &Path, key: &FlipperKey) -> FileImport {
        match flipper::capture_from_key(key, self.protocols, *self.next_id) {
            Ok(capture) => match self.add(capture, path) {
                true => FileImport::Added(1),
                false => FileImport::Nothing("its key is already in the list".to_string()),
            },
            Err(e) => FileImport::Failed(format!("{e:#}")),
        }
    }

    /// A RAW (or BinRAW) `.sub`: decoded as a stream (ProtoPirate's sub_decode), gated by the
    /// file's preset modulation (AM/FM) as Flipper-ARF does; one capture per distinct decode.
    fn import_raw(&mut self, path: &Path, sub: SubFile) -> FileImport {
        let pairs: Vec<LevelDuration> = sub.pairs.iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect();
        let rf = sub.preset.as_ref().and_then(|p| p.rf());
        let decodes = self.protocols.process_signal_stream_rf(&pairs, sub.frequency, rf);
        let decoded_any = !decodes.is_empty();
        // The same signal can decode at several stream positions (e.g. Ford V0 across bursts).
        let mut seen: HashSet<(String, u64, Option<u32>, Option<u8>)> = HashSet::new();
        let mut added = 0;
        for (protocol_name, decoded, segment) in decodes {
            if !seen.insert((protocol_name.clone(), decoded.data, decoded.serial, decoded.button)) {
                continue;
            }
            let raw: Vec<StoredLevelDuration> =
                segment.iter().map(|p| StoredLevelDuration { level: p.level, duration_us: p.duration_us }).collect();
            let mut capture = Capture::from_pairs_with_rf(*self.next_id, sub.frequency, raw, None);
            capture.rf_preset = sub.preset.clone();
            capture.protocol = Some(protocol_name);
            set_decode(&mut capture, &decoded);
            if capture.protocol.as_deref() == Some(crate::protocols::PSA_UNCHECKED_NAME) {
                capture.source_file = Some(App::path_relative_to_import(path, self.import_dir));
                self.unchecked.push(capture);
                continue;
            }
            added += usize::from(self.add(capture, path));
        }
        // Nothing decoded: the whole file as one unknown capture in research mode, as for a live
        // signal. (Decodes that were all in the list already add nothing: re-importing a file used
        // to add a spurious "Unknown" capture of all of it.)
        if !decoded_any {
            if !self.show_unknown {
                return FileImport::Nothing(
                    "no protocol decodes it, and unknown signals are not listed (show_unknown_signals = false)".to_string(),
                );
            }
            let mut capture = Capture::from_pairs_with_rf(*self.next_id, sub.frequency, sub.pairs, None);
            capture.rf_preset = sub.preset;
            added += usize::from(self.add(capture, path));
        }
        match added {
            0 => FileImport::Nothing("its signals are already in the list".to_string()),
            n => FileImport::Added(n),
        }
    }

    /// A `.fob` (or any other file: it is tried as one).
    fn import_fob(&mut self, path: &Path, text: &str) -> FileImport {
        let mut capture = match crate::export::fob::parse_fob(text, *self.next_id) {
            Ok(c) => c,
            Err(e) if crate::export::has_extension(path, "fob") => {
                return FileImport::Failed(format!("not a valid .fob file ({e})"));
            }
            Err(_) => return FileImport::Failed("not a .fob or .sub file".to_string()),
        };
        // Re-run the current decoders whenever raw pairs are present (same as .sub), so captures
        // exported by older KAT versions pick up today's decoders: renamed/retired protocols (Land
        // Rover V0 → Honda V2, Porsche Touareg → Porsche Cayenne), new encoders (Renault V1, Fiat
        // V1, VAG …) and their `data_extra`. If nothing decodes, the stored fields are kept as-is.
        if !capture.raw_pairs.is_empty() {
            let pairs: Vec<LevelDuration> =
                capture.raw_pairs.iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect();
            // Received by KAT (path: preferred, then all) or imported (preset: strict, as Flipper-ARF).
            let decoded = match capture.received_rf {
                Some(path) => self.protocols.process_signal_path(&pairs, capture.frequency, Some(path)),
                None => {
                    let rf = capture.rf_preset.as_ref().and_then(|p| p.rf());
                    self.protocols.process_signal_rf(&pairs, capture.frequency, rf)
                }
            };
            if let Some((protocol_name, decoded)) = decoded {
                capture.protocol = Some(protocol_name);
                set_decode(&mut capture, &decoded);
            }
        }
        super::key_files::fit_encoder_status(self.protocols, &mut capture);
        if !self.show_unknown && capture.protocol.is_none() {
            return FileImport::Nothing(
                "its signal is not decoded, and unknown signals are not listed (show_unknown_signals = false)".to_string(),
            );
        }
        // A .fob is added as saved (also when the list has the same signal), as before.
        *self.next_id += 1;
        capture.source_file = Some(App::path_relative_to_import(path, self.import_dir));
        self.captures.push(capture);
        FileImport::Added(1)
    }
}

/// A decode's fields on a capture.
pub(super) fn set_decode(capture: &mut Capture, decoded: &DecodedSignal) {
    capture.serial = decoded.serial;
    capture.button = decoded.button;
    capture.counter = decoded.counter;
    capture.crc_valid = decoded.crc_valid;
    capture.data = decoded.data;
    capture.data_count_bit = decoded.data_count_bit;
    capture.data_extra = decoded.extra;
    capture.status = if decoded.encoder_capable { CaptureStatus::EncoderCapable } else { CaptureStatus::Decoded };
}

/// The status line after importing files: (status message, error). One file: "Imported 15
/// captures from Kia3.sub", "Nothing imported from Kia3.sub: its signals are already in the list"
/// or the error "Nothing imported from notes.fob: <why>"; several: "Imported 12 captures from 3
/// files (1 failed: x.sub: <why>)" (an error when nothing was imported).
pub(super) fn import_summary(results: &[(String, FileImport)]) -> (Option<String>, Option<String>) {
    let captures = |n: usize| if n == 1 { "1 capture".to_string() } else { format!("{n} captures") };
    match results {
        [] => (None, None),
        [(name, FileImport::Added(n))] => (Some(format!("Imported {} from {name}", captures(*n))), None),
        [(name, FileImport::Nothing(why))] => (Some(format!("Nothing imported from {name}: {why}")), None),
        [(name, FileImport::Failed(why))] => (None, Some(format!("Nothing imported from {name}: {why}"))),
        _ => {
            let added: usize = results.iter().map(|(_, r)| if let FileImport::Added(n) = r { *n } else { 0 }).sum();
            let failed: Vec<String> = results
                .iter()
                .filter_map(|(name, r)| match r {
                    FileImport::Failed(why) => Some(format!("{name}: {why}")),
                    _ => None,
                })
                .collect();
            let files = results.len();
            let mut msg = if added > 0 {
                format!("Imported {} from {files} files", captures(added))
            } else {
                format!("Nothing imported from {files} files")
            };
            match failed.as_slice() {
                [] if added == 0 => msg.push_str(": their signals are already in the list, or not decoded"),
                [] => {}
                [one] => msg.push_str(&format!(" (1 failed: {one})")),
                [first, rest @ ..] => msg.push_str(&format!(" ({} failed: {first}; {} more in kat.log)", failed.len(), rest.len())),
            }
            if added == 0 && !failed.is_empty() {
                (None, Some(msg))
            } else {
                (Some(msg), None)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("kat-files-{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// Suggested names stay plain file names a FAT card accepts, and are cut when long.
    #[test]
    fn suggested_names_are_safe_everywhere() {
        assert_eq!(safe_stem("2019_Kia: EU?_\"Rio\"*_Unlock"), "2019_Kia- EU-_-Rio--_Unlock");
        assert_eq!(safe_stem(" Honda/Acura\\x|y<z>\t"), "Honda-Acura-x-y-z-");
        let long = "é".repeat(100);
        let cut = safe_stem(&long);
        assert!(cut.len() <= MAX_STEM_BYTES && cut.chars().all(|c| c == 'é'), "{}", cut.len());
    }

    /// The typed name gets the extension once, in the folder, and never an existing file's name.
    #[test]
    fn export_files_never_replace_a_file() {
        let dir = scratch("export");
        let first = export_file(&dir, "Unknown_Unknown_Unknown_Unknown_Unknown", "sub");
        assert_eq!(first, dir.join("Unknown_Unknown_Unknown_Unknown_Unknown.sub"));
        std::fs::write(&first, "x").unwrap();
        assert_eq!(export_file(&dir, "Unknown_Unknown_Unknown_Unknown_Unknown", "sub"), dir.join("Unknown_Unknown_Unknown_Unknown_Unknown_2.sub"));
        assert_eq!(export_file(&dir, " mine.SUB ", "sub"), dir.join("mine.SUB"));
        assert_eq!(export_file(&dir, "fob_433.92", "fob"), dir.join("fob_433.92.fob"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    fn importer<'a>(reg: &'a mut ProtocolRegistry, caps: &'a mut Vec<Capture>, next: &'a mut u32, research: bool) -> Importer<'a> {
        Importer {
            protocols: reg,
            captures: caps,
            next_id: next,
            show_unknown: research,
            ignore_duplicates: true,
            import_dir: Path::new("IMPORTS"),
            unchecked: Vec::new(),
        }
    }

    /// A RAW `.sub` adds one capture per distinct decode; importing it again adds nothing (it used
    /// to add an "Unknown" capture of the whole file), also when its name says `.SUB`.
    #[test]
    fn reimporting_a_sub_adds_nothing() {
        crate::protocols::keys::load_keystore_from_embedded();
        let src = Path::new("IMPORTS/KIA/Kia3_5cl_5op_5tr.sub");
        if !src.exists() {
            eprintln!("Skip: {src:?} not found (run from crate root)");
            return;
        }
        let (mut reg, mut caps, mut next) = (ProtocolRegistry::new(), Vec::new(), 1u32);
        let mut imp = importer(&mut reg, &mut caps, &mut next, true);
        let (name, first) = imp.import(src);
        assert_eq!((name.as_str(), &first), ("Kia3_5cl_5op_5tr.sub", &FileImport::Added(15)));
        let dir = scratch("upper");
        let upper = dir.join("Garage.SUB");
        std::fs::copy(src, &upper).unwrap();
        let (_, again) = imp.import(&upper);
        assert_eq!(again, FileImport::Nothing("its signals are already in the list".to_string()));
        assert_eq!(caps.len(), 15);
        assert!(caps.iter().all(|c| c.protocol.is_some()), "no Unknown capture of the whole file");
        assert_eq!(caps.iter().map(|c| c.id).collect::<Vec<_>>(), (1..=15).collect::<Vec<_>>());
        assert_eq!(caps[0].source_file.as_deref(), Some("KIA/Kia3_5cl_5op_5tr.sub"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// With duplicates kept (`ignore_duplicates = false`), importing a file again adds its
    /// captures again; repeats within the file are still one capture each.
    #[test]
    fn kept_duplicates_import_again() {
        crate::protocols::keys::load_keystore_from_embedded();
        let src = Path::new("IMPORTS/KIA/Kia3_5cl_5op_5tr.sub");
        if !src.exists() {
            eprintln!("Skip: {src:?} not found (run from crate root)");
            return;
        }
        let (mut reg, mut caps, mut next) = (ProtocolRegistry::new(), Vec::new(), 1u32);
        let mut imp = importer(&mut reg, &mut caps, &mut next, true);
        imp.ignore_duplicates = false;
        assert_eq!(imp.import(src).1, FileImport::Added(15));
        assert_eq!(imp.import(src).1, FileImport::Added(15));
        assert_eq!(caps.len(), 30);
        assert_eq!(caps[..15].iter().map(|c| c.data).collect::<Vec<_>>(), caps[15..].iter().map(|c| c.data).collect::<Vec<_>>());
    }

    /// Unreadable and unknown files, and undecodable signals without research mode, say why.
    #[test]
    fn failed_imports_say_why() {
        let dir = scratch("bad");
        let (mut reg, mut caps, mut next) = (ProtocolRegistry::new(), Vec::new(), 1u32);
        let notes = dir.join("notes.fob");
        std::fs::write(&notes, "hello").unwrap();
        let empty_raw = dir.join("empty.sub");
        std::fs::write(&empty_raw, "Filetype: Flipper SubGhz RAW File\nVersion: 1\nProtocol: RAW\n").unwrap();
        let noise = dir.join("noise.sub");
        std::fs::write(&noise, "Filetype: Flipper SubGhz RAW File\nFrequency: 433920000\nProtocol: RAW\nRAW_Data: 500 -500 500 -500\n").unwrap();
        let text = dir.join("x.txt");
        std::fs::write(&text, "x").unwrap();
        let mut imp = importer(&mut reg, &mut caps, &mut next, false);
        assert!(matches!(imp.import(&notes).1, FileImport::Failed(why) if why.starts_with("not a valid .fob file (")));
        assert!(matches!(imp.import(&empty_raw).1, FileImport::Failed(why) if why.contains("No RAW_Data")));
        assert!(matches!(imp.import(&dir.join("gone.sub")).1, FileImport::Failed(why) if why.starts_with("cannot read it (")));
        assert_eq!(imp.import(&text).1, FileImport::Failed("not a .fob or .sub file".to_string()));
        assert!(matches!(imp.import(&noise).1, FileImport::Nothing(why) if why.contains("show_unknown_signals = false")));
        // In research mode the undecoded file is one unknown capture.
        let mut imp = importer(&mut reg, &mut caps, &mut next, true);
        assert_eq!(imp.import(&noise).1, FileImport::Added(1));
        assert_eq!((caps.len(), caps[0].protocol.as_deref(), caps[0].raw_pairs.len()), (1, None, 4));
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The status line names the count of captures and of files, and the failures.
    #[test]
    fn import_summary_counts_captures_and_files() {
        let r = |name: &str, f: FileImport| (name.to_string(), f);
        assert_eq!(import_summary(&[r("Kia3.sub", FileImport::Added(15))]), (Some("Imported 15 captures from Kia3.sub".into()), None));
        assert_eq!(import_summary(&[r("a.fob", FileImport::Added(1))]).0.unwrap(), "Imported 1 capture from a.fob");
        assert_eq!(
            import_summary(&[r("notes.fob", FileImport::Failed("not a valid .fob file (x)".into()))]),
            (None, Some("Nothing imported from notes.fob: not a valid .fob file (x)".into()))
        );
        assert_eq!(
            import_summary(&[r("Kia3.sub", FileImport::Nothing("its signals are already in the list".into()))]).0.unwrap(),
            "Nothing imported from Kia3.sub: its signals are already in the list"
        );
        let three = [
            r("a.sub", FileImport::Added(10)),
            r("b.fob", FileImport::Added(2)),
            r("x.sub", FileImport::Failed("No RAW_Data in .sub file".into())),
        ];
        assert_eq!(import_summary(&three), (Some("Imported 12 captures from 3 files (1 failed: x.sub: No RAW_Data in .sub file)".into()), None));
        let bad = [r("x.sub", FileImport::Failed("e1".into())), r("y.fob", FileImport::Failed("e2".into())), r("z.sub", FileImport::Failed("e3".into()))];
        assert_eq!(import_summary(&bad), (None, Some("Nothing imported from 3 files (3 failed: x.sub: e1; 2 more in kat.log)".into())));
        let dups = [r("a.sub", FileImport::Nothing("dup".into())), r("b.sub", FileImport::Nothing("dup".into()))];
        assert_eq!(import_summary(&dups).0.unwrap(), "Nothing imported from 2 files: their signals are already in the list, or not decoded");
        assert_eq!(import_summary(&[]), (None, None));
    }
}
