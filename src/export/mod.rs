//! Export formats for captured signals.
//!
//! Import: use [scan_import_files_recursive] to find all .fob and .sub files under a
//! directory (e.g. EXPORTS/FIAT, EXPORTS/KIA) so that manufacturer subfolders are included.

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
        } else if path.is_file() {
            if path.extension().map_or(false, |e| e == "fob" || e == "sub") {
                out.push(path);
            }
        }
    }
}
