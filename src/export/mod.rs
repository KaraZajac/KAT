//! Export formats for captured signals.
//!
//! Import: use [scan_import_files_recursive] to find all .fob and .sub files under a
//! directory (e.g. EXPORTS/FIAT, EXPORTS/KIA) so that manufacturer subfolders are included.
//! Writing: [unique_path] picks a name that does not exist yet, so an export or a recording never
//! replaces a file.

pub mod binraw;
pub mod fob;
pub mod flipper;

use std::path::{Path, PathBuf};

/// Recursively scan a directory for importable files (.fob and .sub), including all subdirectories.
/// Use this so that EXPORTS/FIAT, EXPORTS/KIA, EXPORTS/FORD, etc. are all discovered.
pub fn scan_import_files_recursive(dir: &Path) -> Vec<PathBuf> {
    let mut files = Vec::new();
    if !dir.exists() || !dir.is_dir() {
        return files;
    }
    walk_for_imports(dir, &mut files);
    files.sort();
    files
}

fn walk_for_imports(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return;
    };
    for entry in entries.flatten() {
        let path = entry.path();
        if path.is_dir() {
            walk_for_imports(&path, out);
        } else if path.is_file() && (has_extension(&path, "fob") || has_extension(&path, "sub")) {
            out.push(path);
        }
    }
}

/// Whether `path` ends in `.ext`, ignoring case (`Garage.SUB` is a `.sub` file: Flipper and
/// Windows tools write upper-case names).
pub fn has_extension(path: &Path, ext: &str) -> bool {
    path.extension().and_then(|e| e.to_str()).is_some_and(|e| e.eq_ignore_ascii_case(ext))
}

/// `path` when no file or folder has that name, else the first free `<stem>_2.<ext>`,
/// `<stem>_3.<ext>`, … next to it, so a write never replaces an earlier export or recording.
pub fn unique_path(path: &Path) -> PathBuf {
    if !path.exists() {
        return path.to_path_buf();
    }
    let stem = path.file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();
    let ext = path.extension().map(|e| e.to_string_lossy().into_owned());
    (2u32..)
        .map(|n| {
            let name = match &ext {
                Some(ext) => format!("{stem}_{n}.{ext}"),
                None => format!("{stem}_{n}"),
            };
            path.with_file_name(name)
        })
        .find(|p| !p.exists())
        .expect("a free name")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("kat-export-mod-{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// Taken names get `_2`, `_3`, … before the extension; free names are kept.
    #[test]
    fn unique_path_never_reuses_a_name() {
        let dir = scratch("unique");
        let first = dir.join("Unknown_Kia.sub");
        assert_eq!(unique_path(&first), first);
        std::fs::write(&first, "x").unwrap();
        assert_eq!(unique_path(&first), dir.join("Unknown_Kia_2.sub"));
        std::fs::write(dir.join("Unknown_Kia_2.sub"), "x").unwrap();
        assert_eq!(unique_path(&first), dir.join("Unknown_Kia_3.sub"));
        // Two dots: the suffix goes before the last extension; no extension: at the end.
        std::fs::write(dir.join("fob_433.92.cs8"), "x").unwrap();
        assert_eq!(unique_path(&dir.join("fob_433.92.cs8")), dir.join("fob_433.92_2.cs8"));
        std::fs::write(dir.join("notes"), "x").unwrap();
        assert_eq!(unique_path(&dir.join("notes")), dir.join("notes_2"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The startup scan and the import path treat `.SUB` / `.Fob` like `.sub` / `.fob`.
    #[test]
    fn import_scan_ignores_extension_case() {
        let dir = scratch("scan");
        std::fs::create_dir_all(dir.join("GARAGE")).unwrap();
        for f in ["a.sub", "GARAGE/Gate.SUB", "b.Fob", "c.txt", "d.cs8"] {
            std::fs::write(dir.join(f), "").unwrap();
        }
        let found: Vec<String> = scan_import_files_recursive(&dir)
            .iter()
            .map(|p| p.strip_prefix(&dir).unwrap().to_string_lossy().replace('\\', "/"))
            .collect();
        assert_eq!(found, ["GARAGE/Gate.SUB", "a.sub", "b.Fob"]);
        assert!(has_extension(Path::new("x/Garage.SUB"), "sub"));
        assert!(!has_extension(Path::new("x/Garage.sub.txt"), "sub"));
        let _ = std::fs::remove_dir_all(&dir);
    }
}
