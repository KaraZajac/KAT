//! Flipper Zero .sub export format.
//!
//! Aligned with ProtoPirate raw_file_reader and sub_decode: same file format
//! (Flipper SubGhz RAW File, Protocol RAW, RAW_Data as int32: positive = HIGH,
//! negative = LOW, duration in µs). Import uses streaming decode: feed the whole
//! stream and reset decoders on each decode (like ProtoPirate sub_decode). Ford

use anyhow::{Context, Result};
use std::path::Path;

use crate::capture::{Capture, StoredLevelDuration};

/// Export a capture to Flipper Zero .sub RAW format
pub fn export_flipper_sub(capture: &Capture, path: &Path) -> Result<()> {
    if capture.raw_pairs.is_empty() {
        return Err(anyhow::anyhow!("No raw signal data to export"));
    }

    let mut lines = Vec::new();

    // Header
    lines.push("Filetype: Flipper SubGhz RAW File".to_string());
    lines.push("Version: 1".to_string());
    lines.push(format!("Frequency: {}", capture.frequency));
    lines.push("Preset: FuriHalSubGhzPresetOok270Async".to_string());
    lines.push("Protocol: RAW".to_string());

    // Convert raw_pairs to alternating +/- durations
    // Flipper format: positive values = HIGH, negative values = LOW
    let mut raw_data = Vec::new();
    for pair in &capture.raw_pairs {
        let duration = pair.duration_us as i64;
        if pair.level {
            raw_data.push(duration);
        } else {
            raw_data.push(-duration);
        }
    }

    // Write RAW_Data lines (max ~512 values per line for readability)
    const MAX_PER_LINE: usize = 512;
    for chunk in raw_data.chunks(MAX_PER_LINE) {
        let values: Vec<String> = chunk.iter().map(|v| v.to_string()).collect();
        lines.push(format!("RAW_Data: {}", values.join(" ")));
    }

    let content = lines.join("\n") + "\n";
    std::fs::write(path, content)?;
    tracing::info!("Exported Flipper .sub to {:?}", path);
    Ok(())
}

/// Scan a directory for Flipper .sub files (top-level only). Prefer [crate::export::scan_import_files_recursive] for import.
#[allow(dead_code)]
pub fn scan_sub_files(dir: &Path) -> Vec<std::path::PathBuf> {
    let mut out = Vec::new();
    let Ok(entries) = std::fs::read_dir(dir) else {
        return out;
    };
    for e in entries.flatten() {
        let p = e.path();
        if p.extension().map_or(false, |e| e == "sub") {
            out.push(p);
        }
    }
    out.sort();
    out
}

/// Parse a Flipper SubGhz RAW .sub file into frequency and raw pairs (no splitting).
/// Caller runs streaming decode (e.g. [crate::protocols::ProtocolRegistry::process_signal_stream]) on the pairs.
/// Positive values = HIGH, negative = LOW; duration in microseconds.
pub fn import_sub_raw(path: &Path) -> Result<(u32, Vec<StoredLevelDuration>)> {
    let s = std::fs::read_to_string(path)
        .with_context(|| format!("Read .sub file: {:?}", path))?;

    let mut frequency_hz: Option<u32> = None;
    let mut raw_data = Vec::new();

    for line in s.lines() {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        if let Some(rest) = line.strip_prefix("Frequency:") {
            let n: u32 = rest.trim().parse().context("Parse Frequency in .sub")?;
            frequency_hz = Some(n);
            continue;
        }
        if let Some(rest) = line.strip_prefix("RAW_Data:") {
            for word in rest.split_whitespace() {
                let value: i64 = word.parse().with_context(|| format!("Parse RAW_Data value: {:?}", word))?;
                raw_data.push(value);
            }
        }
    }

    let frequency = frequency_hz.unwrap_or(433_920_000);

    // Same convention as ProtoPirate raw_file_reader_get_next: positive => HIGH (true), negative => LOW (false)
    let raw_pairs: Vec<StoredLevelDuration> = raw_data
        .into_iter()
        .map(|v| {
            let duration_us = v.unsigned_abs() as u32;
            let level = v >= 0;
            StoredLevelDuration { level, duration_us }
        })
        .collect();

    if raw_pairs.is_empty() {
        anyhow::bail!("No RAW_Data in .sub file");
    }

    Ok((frequency, raw_pairs))
}
