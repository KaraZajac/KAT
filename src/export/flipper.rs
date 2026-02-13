//! Flipper Zero .sub export format.
//!
//! Read/write files in the Flipper SubGhz RAW format with alternating
//! positive (high) and negative (low) durations in microseconds.

use anyhow::{Context, Result};
use std::path::Path;

use crate::capture::{Capture, CaptureStatus, StoredLevelDuration};

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

/// Scan a directory for Flipper .sub files (same format we export).
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

/// Gap duration (µs) used to split a .sub stream into separate transmissions.
/// Keyfobs typically use 10–25 ms between button pushes; in-frame gaps are &lt; 1 ms.
pub const SUB_INTER_BURST_GAP_US: u32 = 10_000;

/// Split raw level/duration pairs into segments at long gaps (e.g. between keyfob button pushes).
/// Any pulse (HIGH or LOW) with duration >= `gap_threshold_us` starts a new segment; the long pulse is not included in any segment.
pub fn split_raw_pairs_by_gap(
    pairs: &[StoredLevelDuration],
    gap_threshold_us: u32,
) -> Vec<Vec<StoredLevelDuration>> {
    let mut segments = Vec::new();
    let mut current = Vec::new();

    for p in pairs {
        if p.duration_us >= gap_threshold_us {
            if !current.is_empty() {
                segments.push(std::mem::take(&mut current));
            }
        } else {
            current.push(*p);
        }
    }
    if !current.is_empty() {
        segments.push(current);
    }
    segments
}

/// Parse a Flipper SubGhz RAW .sub file and return one Capture per transmission.
/// The stream is split at long gaps (see `split_raw_pairs_by_gap`) so multiple button pushes become separate captures.
/// Positive values = HIGH, negative = LOW; duration in microseconds.
pub fn import_sub(path: &Path, next_id: u32) -> Result<Vec<Capture>> {
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

    let segments = split_raw_pairs_by_gap(&raw_pairs, SUB_INTER_BURST_GAP_US);

    let captures: Vec<Capture> = segments
        .into_iter()
        .enumerate()
        .map(|(i, pairs)| {
            let cap = Capture::from_pairs_with_rf(next_id + i as u32, frequency, pairs, None);
            Capture {
                status: CaptureStatus::Unknown,
                ..cap
            }
        })
        .collect();

    Ok(captures)
}
