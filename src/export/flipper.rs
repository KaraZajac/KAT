//! Flipper Zero .sub export format.
//!
//! Outputs files in the Flipper SubGhz RAW format with alternating
//! positive (high) and negative (low) durations in microseconds.

use anyhow::Result;
use std::path::Path;

use crate::capture::Capture;

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
