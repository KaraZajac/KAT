//! .fob export/import format - rich JSON metadata for captured keyfob signals.

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use std::path::Path;

use crate::capture::{Capture, CaptureStatus, RfModulation, StoredLevelDuration};
use crate::radio::preset::{format_hex_bytes, parse_hex_bytes, RfPreset};

/// User-provided metadata for .fob export
#[derive(Debug, Clone, Default)]
pub struct FobMetadata {
    pub year: Option<u32>,
    pub make: String,
    pub model: String,
    pub region: String,
    /// Command label (e.g. Unlock, Lock) for export filename and .fob vehicle info.
    pub command: String,
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
    /// RF carrier modulation the references flag the protocol with: AM, FM, or AM/FM.
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
    /// Protocol-specific encoder state (`Capture::data_extra`) as hex, e.g. VAG type/key index or the
    /// bits beyond 64 of a long frame. Optional: without it, decoded imports could not re-encode
    /// protocols whose encoder needs it.
    #[serde(skip_serializing_if = "Option::is_none", default)]
    pub data_extra: Option<String>,
    /// Demodulator path that received a live capture ("AM" / "FM"); absent for imports.
    #[serde(skip_serializing_if = "Option::is_none", default)]
    pub received_rf: Option<String>,
    /// RF preset KAT transmits the capture with, as a Flipper `.sub` names it (`Preset:`): the
    /// capture's own (imported `.sub`) or the one KAT chose from its protocol and receive path.
    /// Absent in files written before KAT 1.5 (the protocol then decides).
    #[serde(skip_serializing_if = "Option::is_none", default)]
    pub preset: Option<String>,
    /// `Custom_preset_module` of a custom preset (e.g. "CC1101").
    #[serde(skip_serializing_if = "Option::is_none", default)]
    pub custom_preset_module: Option<String>,
    /// `Custom_preset_data` of a custom preset: CC1101 registers as hex ("02 0D 0B 06 …").
    #[serde(skip_serializing_if = "Option::is_none", default)]
    pub custom_preset_data: Option<String>,
    /// What that preset puts on air ("OOK", "2-FSK ±14.28 kHz"), for readers; ignored on import.
    #[serde(skip_serializing_if = "Option::is_none", default)]
    pub tx_modulation: Option<String>,
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
    /// Command label (e.g. Unlock, Lock); optional for backwards compatibility.
    #[serde(default)]
    pub command: Option<String>,
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
        .unwrap_or_else(|| capture.protocol_make().to_string());
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
    let command = metadata.and_then(|m| {
        let s = m.command.trim();
        if s.is_empty() {
            None
        } else {
            Some(s.to_string())
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

    let preset = capture.export_preset();
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
            data_extra: capture.data_extra.map(|e| format!("{:016X}", e)),
            received_rf: capture.received_rf.map(|r| r.to_string()),
            custom_preset_module: preset.module.clone(),
            custom_preset_data: preset.data.as_deref().map(format_hex_bytes),
            tx_modulation: Some(capture.tx_modulation().to_string()),
            preset: Some(preset.name),
        },
        vehicle: FobVehicleInfo {
            year,
            make,
            model,
            region,
            command,
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

    // Encoder state (optional; absent in files written before it was exported)
    let data_extra = sig
        .data_extra
        .as_deref()
        .and_then(|s| u64::from_str_radix(s.trim_start_matches("0x"), 16).ok());

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

    // Without raw pairs (a capture loaded from a Flipper key file, or exported without pairs) the
    // stored fields are all TX needs; the app keeps EncoderCapable only when its encoder produces a
    // frame from them (`App::check_encoder_capable`).
    let status = if sig.encoder_capable {
        CaptureStatus::EncoderCapable
    } else if protocol.is_some() {
        CaptureStatus::Decoded
    } else {
        CaptureStatus::Unknown
    };

    // RF preset / receive path (optional; absent before KAT 1.5).
    let rf_preset = sig.preset.as_ref().filter(|n| !n.trim().is_empty()).map(|name| RfPreset {
        name: name.trim().to_string(),
        module: sig.custom_preset_module.clone(),
        data: sig.custom_preset_data.as_deref().and_then(parse_hex_bytes),
    });
    let received_rf = match sig.received_rf.as_deref().map(str::trim) {
        Some(r) if r.eq_ignore_ascii_case("AM") => Some(RfModulation::AM),
        Some(r) if r.eq_ignore_ascii_case("FM") => Some(RfModulation::FM),
        _ => None,
    };

    let vehicle = &fob.vehicle;
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
        data_extra,
        raw_pairs,
        status,
        received_rf,
        rf_preset,
        year: vehicle.year.map(|y| y.to_string()),
        make: Some(vehicle.make.clone()).filter(|s| !s.is_empty()),
        model: vehicle.model.clone().filter(|s| !s.is_empty()),
        region: vehicle.region.clone().filter(|s| !s.is_empty()),
        command: vehicle.command.clone().filter(|s| !s.is_empty()),
        source_file: None,
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
        rf_preset: None,
        year: None,
        make: None,
        model: None,
        region: None,
        command: None,
        source_file: None,
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

#[cfg(test)]
mod tests {
    use super::*;

    /// `data_extra` (encoder state, e.g. VAG type/key index or bits beyond 64 of a long frame) must
    /// survive a .fob export → import round trip, so decoded imports can still re-encode.
    #[test]
    fn fob_roundtrip_preserves_data_extra() {
        let pairs = vec![
            StoredLevelDuration { level: true, duration_us: 500 },
            StoredLevelDuration { level: false, duration_us: 1000 },
        ];
        let mut capture = Capture::from_pairs_with_rf(7, 433_920_000, pairs, None);
        capture.protocol = Some("VAG".to_string());
        capture.serial = Some(0x00AB_CDEF);
        capture.button = Some(0x02);
        capture.counter = Some(0x1234);
        capture.crc_valid = true;
        capture.data = 0x0123_4567_89AB_CDEF;
        capture.data_count_bit = 80;
        capture.data_extra = Some(0xFEDC_BA98_7654_3210);
        capture.status = CaptureStatus::EncoderCapable;

        let path = std::env::temp_dir().join(format!(
            "kat_fob_extra_roundtrip_{}.fob",
            std::process::id()
        ));
        export_fob(&capture, &path, true, None).expect("export");
        let imported = import_fob(&path, 99);
        let _ = std::fs::remove_file(&path);
        let imported = imported.expect("import");

        assert_eq!(imported.protocol.as_deref(), Some("VAG"));
        assert_eq!(imported.data, capture.data);
        assert_eq!(imported.data_count_bit, 80);
        assert_eq!(imported.data_extra, Some(0xFEDC_BA98_7654_3210));
        assert_eq!(imported.status, CaptureStatus::EncoderCapable);
    }

    /// Files written before `data_extra` existed must still import (field absent → None).
    #[test]
    fn fob_import_without_data_extra_field() {
        let mut capture = Capture::from_pairs_with_rf(1, 433_920_000, Vec::new(), None);
        capture.protocol = Some("Kia V7".to_string());
        capture.data = 0x4C00_0000_0000_0000;
        capture.data_count_bit = 64;
        let path = std::env::temp_dir().join(format!(
            "kat_fob_no_extra_{}.fob",
            std::process::id()
        ));
        export_fob(&capture, &path, false, None).expect("export");
        let json = std::fs::read_to_string(&path).expect("read");
        let _ = std::fs::remove_file(&path);
        assert!(!json.contains("data_extra"), "None must not be serialized");
        let fob: FobFile = serde_json::from_str(&json).expect("parse");
        assert!(fob.signal.data_extra.is_none());
    }

    fn temp_path(tag: &str) -> std::path::PathBuf {
        std::env::temp_dir().join(format!("kat_fob_{tag}_{}.fob", std::process::id()))
    }

    /// KAT 1.5 persists the receive path and the transmit preset (custom registers included);
    /// re-importing gives the same capture preset and therefore the same TX modulation.
    #[test]
    fn fob_roundtrip_preserves_preset_and_received_rf() {
        use crate::radio::preset::{TxModulation, FM476, HONDA1};
        let pairs = vec![StoredLevelDuration { level: true, duration_us: 63 }];
        // Imported Honda Static capture with its custom (Honda1) preset.
        let mut honda = Capture::from_pairs_with_rf(1, 433_657_070, pairs.clone(), None);
        honda.protocol = Some("Honda Static".to_string());
        honda.rf_preset = Some(RfPreset::from_def(&HONDA1));
        // Live FM capture of Ford V1 (no preset): KAT persists its choice, the F4 custom preset.
        let mut ford = Capture::from_pairs_with_rf(2, 433_920_000, pairs.clone(), Some(RfModulation::FM));
        ford.protocol = Some("Ford V1".to_string());
        // Stock preset by name.
        let mut kia = Capture::from_pairs_with_rf(3, 433_920_000, pairs, Some(RfModulation::FM));
        kia.protocol = Some("Kia V5".to_string());
        kia.rf_preset = Some(RfPreset::from_def(&FM476));

        for (tag, c, label) in [("honda", &honda, "Honda1"), ("ford", &ford, "F4"), ("kia", &kia, "FM476")] {
            let path = temp_path(tag);
            export_fob(c, &path, true, None).expect("export");
            let json = std::fs::read_to_string(&path).expect("read");
            let imported = import_fob(&path, 9);
            let _ = std::fs::remove_file(&path);
            let imported = imported.expect("import");
            let p = imported.rf_preset.clone().expect("preset persisted");
            assert_eq!(p, c.export_preset(), "{tag}");
            assert_eq!(p.label(), label, "{tag}");
            assert_eq!(imported.received_rf, c.received_rf, "{tag}");
            assert_eq!(imported.tx_modulation(), c.tx_modulation(), "{tag}");
            assert!(imported.tx_modulation().is_fsk(), "{tag}");
            assert!(json.contains("\"tx_modulation\": \"2-FSK ±"), "{tag}: {json}");
            if p.is_custom() {
                assert!(json.contains("\"custom_preset_module\": \"CC1101\""), "{tag}");
                assert!(json.contains("\"custom_preset_data\": \"02 0D 0B 06"), "{tag}");
            } else {
                assert!(!json.contains("custom_preset"), "{tag}");
            }
        }
        // An OOK capture says so.
        let mut ook = Capture::from_pairs_with_rf(4, 433_920_000, Vec::new(), Some(RfModulation::AM));
        ook.protocol = Some("Kia V1".to_string());
        let path = temp_path("ook");
        export_fob(&ook, &path, false, None).expect("export");
        let imported = import_fob(&path, 9).expect("import");
        let _ = std::fs::remove_file(&path);
        assert_eq!(imported.tx_modulation(), TxModulation::Ook);
        assert_eq!(imported.rf_preset.unwrap().name, "FuriHalSubGhzPresetOok650Async");
    }

    /// A `.fob` written by KAT 1.4 (no preset / received_rf fields) still imports; its TX
    /// modulation then comes from the protocol (Ford V1 → F4, 2-FSK ±14.28 kHz).
    #[test]
    fn fob_from_kat_1_4_still_imports() {
        let json = r#"{
  "version": "2.0",
  "format": "kat-fob",
  "signal": {
    "protocol": "Ford V1",
    "frequency": 433920000,
    "frequency_mhz": "433.92MHz",
    "modulation": "Manchester",
    "rf_modulation": "FM",
    "encryption": "Rolling Code",
    "data_bits": 136,
    "data_hex": "C0FFEE1A2B3C4D5E",
    "serial": "1A2B3C4D",
    "key": "C0FFEE1A2B3C4D5E",
    "button": 2,
    "button_name": "Unlock",
    "counter": 4660,
    "crc_valid": true,
    "encoder_capable": true,
    "data_extra": "0011223344556677"
  },
  "vehicle": { "year": null, "make": "Ford", "model": null, "region": null, "command": null, "notes": null },
  "capture": {
    "timestamp": "2026-11-03T12:00:00+00:00",
    "raw_data_hex": "C0FFEE1A2B3C4D5E",
    "raw_pairs": [ { "level": true, "duration_us": 65 }, { "level": false, "duration_us": 130 } ],
    "raw_pair_count": 2
  }
}"#;
        let path = temp_path("v14");
        std::fs::write(&path, json).unwrap();
        let imported = import_fob(&path, 5);
        let _ = std::fs::remove_file(&path);
        let c = imported.expect("1.4 .fob imports");
        assert_eq!(c.protocol.as_deref(), Some("Ford V1"));
        assert_eq!(c.data_extra, Some(0x0011_2233_4455_6677));
        assert_eq!((c.rf_preset.clone(), c.received_rf), (None, None));
        assert_eq!(c.export_preset().label(), "F4");
        assert!(matches!(c.tx_modulation(), crate::radio::preset::TxModulation::Fsk { deviation_hz } if (deviation_hz - 14_282.23).abs() < 0.01));
    }
}
