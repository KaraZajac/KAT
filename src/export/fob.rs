//! .fob export/import format - rich JSON metadata for captured keyfob signals.

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use std::path::Path;

use crate::app::App;
use crate::capture::{Capture, CaptureStatus, StoredLevelDuration};

/// User-provided metadata for .fob export
#[derive(Debug, Clone, Default)]
pub struct FobMetadata {
    pub year: Option<u32>,
    pub make: String,
    pub model: String,
    pub region: String,
    pub notes: String,
}

/// Top-level .fob file structure
#[derive(Serialize, Deserialize)]
pub struct FobFile {
    pub version: String,
    pub format: String,
    pub signal: FobSignalInfo,
    pub vehicle: FobVehicleInfo,
    pub capture: FobCapture,
}

/// Signal-level metadata (derived from protocol)
#[derive(Serialize, Deserialize)]
pub struct FobSignalInfo {
    pub protocol: String,
    pub frequency: u32,
    pub frequency_mhz: String,
    pub modulation: String,
    /// RF carrier modulation: AM, FM, or AM/FM (from ProtoPirate). KAT receives AM only.
    #[serde(skip_serializing_if = "Option::is_none", default)]
    pub rf_modulation: Option<String>,
    pub encryption: String,
    pub data_bits: usize,
    pub data_hex: String,
    pub serial: String,
    pub key: String,
    #[serde(default)]
    pub button: Option<u8>,
    pub button_name: String,
    #[serde(default)]
    pub counter: Option<u16>,
    pub crc_valid: bool,
    pub encoder_capable: bool,
}

/// Vehicle metadata (user-provided + auto-detected)
#[derive(Serialize, Deserialize)]
pub struct FobVehicleInfo {
    #[serde(default)]
    pub year: Option<u32>,
    pub make: String,
    #[serde(default)]
    pub model: Option<String>,
    #[serde(default)]
    pub region: Option<String>,
    #[serde(default)]
    pub notes: Option<String>,
}

/// Capture data within a .fob file (timing + raw data)
#[derive(Serialize, Deserialize)]
pub struct FobCapture {
    pub timestamp: String,
    /// Raw data value (hex string) for signal reconstruction
    #[serde(default)]
    pub raw_data_hex: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none", default)]
    pub raw_pairs: Option<Vec<FobPair>>,
    #[serde(default)]
    pub raw_pair_count: usize,
}

/// A single level+duration pair in the .fob file
#[derive(Serialize, Deserialize)]
pub struct FobPair {
    pub level: bool,
    pub duration_us: u32,
}

/// Export a capture to .fob format with optional user metadata
pub fn export_fob(
    capture: &Capture,
    path: &Path,
    include_raw: bool,
    metadata: Option<&FobMetadata>,
) -> Result<()> {
    let protocol_name = capture.protocol_name().to_string();
    let make = metadata
        .map(|m| m.make.clone())
        .filter(|m| !m.is_empty())
        .unwrap_or_else(|| App::get_make_for_protocol(&protocol_name).to_string());
    let model = metadata.and_then(|m| {
        if m.model.is_empty() {
            None
        } else {
            Some(m.model.clone())
        }
    });
    let year = metadata.and_then(|m| m.year);
    let region = metadata.and_then(|m| {
        if m.region.is_empty() {
            None
        } else {
            Some(m.region.clone())
        }
    });
    let notes = metadata.and_then(|m| {
        if m.notes.is_empty() {
            None
        } else {
            Some(m.notes.clone())
        }
    });

    let raw_pairs = if include_raw && !capture.raw_pairs.is_empty() {
        Some(
            capture
                .raw_pairs
                .iter()
                .map(|p| FobPair {
                    level: p.level,
                    duration_us: p.duration_us,
                })
                .collect(),
        )
    } else {
        None
    };

    let fob = FobFile {
        version: "2.0".to_string(),
        format: "kat-fob".to_string(),
        signal: FobSignalInfo {
            protocol: protocol_name.clone(),
            frequency: capture.frequency,
            frequency_mhz: capture.frequency_mhz(),
            modulation: capture.modulation().to_string(),
            rf_modulation: match capture.rf_modulation() {
                crate::capture::RfModulation::Unknown => None,
                r => Some(r.to_string()),
            },
            encryption: capture.encryption_type().to_string(),
            data_bits: capture.data_count_bit,
            data_hex: capture.data_hex(),
            serial: capture.serial_hex(),
            key: capture.data_hex(),
            button: capture.button,
            button_name: capture.button_name().to_string(),
            counter: capture.counter,
            crc_valid: capture.crc_valid,
            encoder_capable: capture.status == CaptureStatus::EncoderCapable,
        },
        vehicle: FobVehicleInfo {
            year,
            make,
            model,
            region,
            notes,
        },
        capture: FobCapture {
            timestamp: capture.timestamp.to_rfc3339(),
            raw_data_hex: Some(capture.data_hex()),
            raw_pair_count: capture.raw_pairs.len(),
            raw_pairs,
        },
    };

    let json = serde_json::to_string_pretty(&fob)?;
    std::fs::write(path, json)?;
    tracing::info!("Exported .fob v2 to {:?}", path);
    Ok(())
}

/// Import a .fob file and return a Capture (supports v1 and v2 formats)
pub fn import_fob(path: &Path, next_id: u32) -> Result<Capture> {
    let content =
        std::fs::read_to_string(path).with_context(|| format!("Failed to read {:?}", path))?;

    // Try v2 format first
    if let Ok(fob) = serde_json::from_str::<FobFile>(&content) {
        return import_fob_v2(&fob, next_id);
    }

    // Fall back to v1 format
    let fob: FobFileV1 =
        serde_json::from_str(&content).with_context(|| format!("Failed to parse {:?}", path))?;
    import_fob_v1(&fob, next_id)
}

// --- V1 compatibility types ---

/// Legacy v1 .fob file structure
#[derive(Serialize, Deserialize)]
struct FobFileV1 {
    #[allow(dead_code)]
    pub version: String,
    #[allow(dead_code)]
    pub format: String,
    pub capture: FobCaptureV1,
}

/// Legacy v1 capture data
#[derive(Serialize, Deserialize)]
struct FobCaptureV1 {
    pub timestamp: String,
    pub frequency: u32,
    pub protocol: String,
    #[serde(default)]
    pub year: Option<u32>,
    #[allow(dead_code)]
    pub make: String,
    #[serde(default)]
    #[allow(dead_code)]
    pub model: Option<String>,
    pub serial: String,
    pub key: String,
    #[serde(default)]
    pub button: Option<u8>,
    #[allow(dead_code)]
    pub button_name: String,
    #[serde(default)]
    pub counter: Option<u16>,
    #[allow(dead_code)]
    pub encryption: String,
    pub crc_valid: bool,
    pub data_bits: usize,
    #[serde(default)]
    pub data_hex: Option<String>,
    #[serde(default)]
    pub raw_pairs: Option<Vec<FobPair>>,
}

fn import_fob_v2(fob: &FobFile, next_id: u32) -> Result<Capture> {
    let sig = &fob.signal;
    let cap = &fob.capture;

    // Parse serial from hex string
    let serial = u32::from_str_radix(sig.serial.trim_start_matches("0x"), 16).ok();

    // Parse data from hex string
    let data = cap
        .raw_data_hex
        .as_deref()
        .or(Some(sig.data_hex.as_str()))
        .and_then(|s| u64::from_str_radix(s.trim_start_matches("0x"), 16).ok())
        .unwrap_or(0);

    // Parse timestamp
    let timestamp = chrono::DateTime::parse_from_rfc3339(&cap.timestamp)
        .map(|dt| dt.with_timezone(&chrono::Utc))
        .unwrap_or_else(|_| chrono::Utc::now());

    // Reconstruct raw pairs if present
    let raw_pairs: Vec<StoredLevelDuration> = cap
        .raw_pairs
        .as_ref()
        .map(|pairs| {
            pairs
                .iter()
                .map(|p| StoredLevelDuration {
                    level: p.level,
                    duration_us: p.duration_us,
                })
                .collect()
        })
        .unwrap_or_default();

    let protocol = if sig.protocol == "Unknown" {
        None
    } else {
        Some(sig.protocol.clone())
    };

    let status = if sig.encoder_capable && !raw_pairs.is_empty() {
        CaptureStatus::EncoderCapable
    } else if protocol.is_some() {
        CaptureStatus::Decoded
    } else {
        CaptureStatus::Unknown
    };

    Ok(Capture {
        id: next_id,
        timestamp,
        frequency: sig.frequency,
        protocol,
        serial,
        button: sig.button,
        counter: sig.counter,
        crc_valid: sig.crc_valid,
        data,
        data_count_bit: sig.data_bits,
        data_extra: None,
        raw_pairs,
        status,
        received_rf: None,
        year: None,
        make: None,
        model: None,
        region: None,
    })
}

fn import_fob_v1(fob: &FobFileV1, next_id: u32) -> Result<Capture> {
    let cap = &fob.capture;

    // Parse serial from hex string
    let serial = u32::from_str_radix(cap.serial.trim_start_matches("0x"), 16).ok();

    // Parse data from hex string
    let data = cap
        .data_hex
        .as_deref()
        .or(Some(cap.key.as_str()))
        .and_then(|s| u64::from_str_radix(s.trim_start_matches("0x"), 16).ok())
        .unwrap_or(0);

    // Parse timestamp
    let timestamp = chrono::DateTime::parse_from_rfc3339(&cap.timestamp)
        .map(|dt| dt.with_timezone(&chrono::Utc))
        .unwrap_or_else(|_| chrono::Utc::now());

    // Reconstruct raw pairs if present
    let raw_pairs: Vec<StoredLevelDuration> = cap
        .raw_pairs
        .as_ref()
        .map(|pairs| {
            pairs
                .iter()
                .map(|p| StoredLevelDuration {
                    level: p.level,
                    duration_us: p.duration_us,
                })
                .collect()
        })
        .unwrap_or_default();

    let protocol = if cap.protocol == "Unknown" {
        None
    } else {
        Some(cap.protocol.clone())
    };

    let status = if protocol.is_some() && !raw_pairs.is_empty() {
        CaptureStatus::EncoderCapable
    } else if protocol.is_some() {
        CaptureStatus::Decoded
    } else {
        CaptureStatus::Unknown
    };

    Ok(Capture {
        id: next_id,
        timestamp,
        frequency: cap.frequency,
        protocol,
        serial,
        button: cap.button,
        counter: cap.counter,
        crc_valid: cap.crc_valid,
        data,
        data_count_bit: cap.data_bits,
        data_extra: None,
        raw_pairs,
        status,
        received_rf: None,
        year: None,
        make: None,
        model: None,
        region: None,
    })
}

/// Scan a directory for .fob files (top-level only). Prefer [crate::export::scan_import_files_recursive] for import.
#[allow(dead_code)]
pub fn scan_fob_files(dir: &Path) -> Vec<std::path::PathBuf> {
    if !dir.exists() || !dir.is_dir() {
        return Vec::new();
    }

    let mut files = Vec::new();
    if let Ok(entries) = std::fs::read_dir(dir) {
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_file() && path.extension().map_or(false, |e| e == "fob") {
                files.push(path);
            }
        }
    }
    files.sort();
    files
}
