//! Flipper Zero .sub export format.
//!
//! Aligned with ProtoPirate raw_file_reader and sub_decode: same file format
//! (Flipper SubGhz RAW File, Protocol RAW, RAW_Data as int32: positive = HIGH,
//! negative = LOW, duration in µs). Import uses streaming decode: feed the whole
//! stream and reset decoders on each decode (like ProtoPirate sub_decode).
//!
//! The header's `Preset:` (with `Custom_preset_module` / `Custom_preset_data` for a custom
//! CC1101 preset) is read into [RfPreset] and kept on the capture: it decides OOK vs 2-FSK and the
//! deviation for transmit. Export writes the capture's own preset back, or the one KAT transmits
//! it with ([Capture::export_preset]): built-in presets by name, others as custom registers.
//!
//! **Key files** (`docs/flipper_key_files.md`): a `.sub` without `RAW_Data` is a decoded key
//! (`Protocol` / `Bit` / `Key` + the protocol's own lines, as ProtoPirate / Flipper-ARF save every
//! decode). [read_sub] tells the two apart; [capture_from_key] rebuilds the decode through the
//! protocol's `from_flipper_key` (a capture without raw pairs), and [export_flipper_key] writes a
//! capture as the key file its reference's `serialize` would write.

use anyhow::{Context, Result};
use std::path::Path;

use crate::capture::{Capture, CaptureStatus, StoredLevelDuration};
use crate::protocols::{FlipperKey, ProtocolRegistry};
use crate::radio::demodulator::LevelDuration;
use crate::radio::preset::{format_hex_bytes, parse_hex_bytes, RfPreset};

/// What a `.sub` file holds: RAW pairs or a decoded key.
#[derive(Debug, Clone)]
pub enum SubContent {
    Raw(SubFile),
    Key(FlipperKey),
}

/// Read a `.sub` file: RAW (`RAW_Data` lines, or `Protocol: RAW`), BinRAW (`Data_RAW` bit blocks,
/// turned into pulses) or a key file.
#[allow(dead_code)] // tests and tools; the app reads the text itself and calls [parse_sub_content]
pub fn read_sub(path: &Path) -> Result<SubContent> {
    let s = std::fs::read_to_string(path).with_context(|| format!("Read .sub file: {:?}", path))?;
    parse_sub_content(&s)
}

/// Parse the text of a `.sub` file (see [read_sub]).
pub fn parse_sub_content(s: &str) -> Result<SubContent> {
    let is_raw = s.lines().any(|l| {
        let l = l.trim_start();
        l.starts_with("RAW_Data:")
            || l.starts_with("Data_RAW:")
            || l.strip_prefix("Protocol:").is_some_and(|p| matches!(p.trim(), "RAW" | "BinRAW"))
    });
    if is_raw {
        return parse_sub(s).map(SubContent::Raw);
    }
    FlipperKey::parse(s).map(SubContent::Key).map_err(|e| anyhow::anyhow!("Not a RAW or key .sub file: {e}"))
}

/// The capture a key file stands for: the decode its protocol rebuilds from it
/// (`ProtocolDecoder::from_flipper_key`), at the file's frequency and preset, without raw pairs —
/// except for a protocol whose frame does not fit the decode (Urmet), whose transmission becomes
/// the capture's recording (`ProtocolDecoder::key_file_pulses`), so Replay sends it.
/// Err when the protocol is unknown to KAT or the key does not load (wrong Bit, failed check …).
pub fn capture_from_key(key: &FlipperKey, registry: &ProtocolRegistry, id: u32) -> Result<Capture> {
    let (name, decoded) = registry.decode_key(key).ok_or_else(|| {
        anyhow::anyhow!("{} key file ({} bits) is not supported or does not validate", key.protocol, key.bits)
    })?;
    let pairs: Vec<StoredLevelDuration> = registry
        .get(&name)
        .and_then(|p| p.key_file_pulses(key))
        .unwrap_or_default()
        .into_iter()
        .map(|p| StoredLevelDuration { level: p.level, duration_us: p.duration_us })
        .collect();
    let mut capture = Capture::from_pairs_with_rf(id, key.frequency, pairs, None);
    capture.rf_preset = key.rf_preset();
    capture.protocol = Some(name);
    capture.serial = decoded.serial;
    capture.button = decoded.button;
    capture.counter = decoded.counter;
    capture.crc_valid = decoded.crc_valid;
    capture.data = decoded.data;
    capture.data_count_bit = decoded.data_count_bit;
    capture.data_extra = decoded.extra;
    capture.key_repeat = key.u32_field("Repeat");
    capture.status = if decoded.encoder_capable { CaptureStatus::EncoderCapable } else { CaptureStatus::Decoded };
    Ok(capture)
}

/// The key file the capture's protocol writes for it (its reference's `serialize`), with the
/// capture's frequency and preset ([Capture::export_preset]). None when the protocol has no key
/// file support (or the capture is not a decode of it).
pub fn capture_flipper_key(capture: &Capture, registry: &ProtocolRegistry) -> Option<FlipperKey> {
    let decoder = registry.get(capture.protocol.as_deref()?)?;
    let mut key = decoder.to_flipper_key(&capture.decoded_signal()).or_else(|| {
        // A frame the decode cannot hold (Urmet): rebuilt from the capture's recording.
        let pulses: Vec<LevelDuration> =
            capture.raw_pairs.iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect();
        decoder.key_file_from_pulses(&pulses)
    })?;
    key.frequency = capture.frequency;
    key.set_preset(&capture.export_preset());
    // A loaded file's `Repeat` line stays with it (a protocol that writes its own, Kia V7, keeps that).
    if let (Some(n), None) = (capture.key_repeat, key.field("Repeat")) {
        key.push_u32("Repeat", n);
    }
    Some(key)
}

/// Export a capture as a Flipper key file (`.sub` with Protocol/Bit/Key, no RAW_Data).
pub fn export_flipper_key(capture: &Capture, registry: &ProtocolRegistry, path: &Path) -> Result<()> {
    let key = capture_flipper_key(capture, registry).ok_or_else(|| {
        anyhow::anyhow!("{} captures cannot be saved as a Flipper key file", capture.protocol_name())
    })?;
    std::fs::write(path, key.to_text())?;
    tracing::info!("Exported Flipper key file to {:?}", path);
    Ok(())
}

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
    let preset = capture.export_preset();
    lines.push(format!("Preset: {}", preset.name));
    if preset.is_custom() {
        if let Some(data) = &preset.data {
            let module = preset.module.as_deref().unwrap_or(crate::radio::preset::CUSTOM_PRESET_MODULE);
            lines.push(format!("Custom_preset_module: {module}"));
            lines.push(format!("Custom_preset_data: {}", format_hex_bytes(data)));
        }
    }
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
        if super::has_extension(&p, "sub") {
            out.push(p);
        }
    }
    out.sort();
    out
}

/// A Flipper SubGhz RAW file: frequency, RF preset (when the header has one) and raw pairs.
#[derive(Debug, Clone)]
pub struct SubFile {
    pub frequency: u32,
    pub preset: Option<RfPreset>,
    pub pairs: Vec<StoredLevelDuration>,
}

/// Parse a Flipper SubGhz RAW .sub file into frequency and raw pairs (no splitting).
/// Caller runs streaming decode (e.g. [crate::protocols::ProtocolRegistry::process_signal_stream]) on the pairs.
/// Positive values = HIGH, negative = LOW; duration in microseconds.
#[allow(dead_code)] // tests and callers that only need the pairs; the app uses [import_sub]
pub fn import_sub_raw(path: &Path) -> Result<(u32, Vec<StoredLevelDuration>)> {
    let sub = import_sub(path)?;
    Ok((sub.frequency, sub.pairs))
}

/// Parse a Flipper SubGhz RAW .sub file, header preset included (see [SubFile]).
pub fn import_sub(path: &Path) -> Result<SubFile> {
    let s = std::fs::read_to_string(path)
        .with_context(|| format!("Read .sub file: {:?}", path))?;
    parse_sub(&s)
}

/// Parse the text of a Flipper SubGhz RAW .sub file (see [import_sub]).
pub fn parse_sub(s: &str) -> Result<SubFile> {
    let mut frequency_hz: Option<u32> = None;
    let mut preset_name: Option<String> = None;
    let mut preset_module: Option<String> = None;
    let mut preset_data: Option<Vec<u8>> = None;
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
        if let Some(rest) = line.strip_prefix("Preset:") {
            preset_name = Some(rest.trim().to_string()).filter(|n| !n.is_empty());
            continue;
        }
        if let Some(rest) = line.strip_prefix("Custom_preset_module:") {
            preset_module = Some(rest.trim().to_string()).filter(|n| !n.is_empty());
            continue;
        }
        if let Some(rest) = line.strip_prefix("Custom_preset_data:") {
            // An unparsable register image is kept out (the preset then counts as unknown).
            preset_data = parse_hex_bytes(rest);
            continue;
        }
        if let Some(rest) = line.strip_prefix("RAW_Data:") {
            // Flipper reads each value with sscanf, so "1718, -32700," (some unit-test files) parses.
            for word in rest.split(|c: char| c.is_whitespace() || c == ',').filter(|w| !w.is_empty()) {
                let value: i64 = word.parse().with_context(|| format!("Parse RAW_Data value: {:?}", word))?;
                raw_data.push(value);
            }
        }
    }

    let frequency = frequency_hz.unwrap_or(433_920_000);
    let preset = preset_name.map(|name| RfPreset { name, module: preset_module, data: preset_data });

    // Same convention as ProtoPirate raw_file_reader_get_next: positive => HIGH (true), negative => LOW (false)
    let pairs: Vec<StoredLevelDuration> = raw_data
        .into_iter()
        .map(|v| {
            let duration_us = v.unsigned_abs() as u32;
            let level = v >= 0;
            StoredLevelDuration { level, duration_us }
        })
        .collect();

    if pairs.is_empty() {
        // A Flipper BinRAW file carries bit blocks instead of RAW_Data.
        if let Some(pairs) = super::binraw::binraw_pairs(s) {
            return Ok(SubFile { frequency, preset, pairs });
        }
        anyhow::bail!("No RAW_Data in .sub file");
    }

    Ok(SubFile { frequency, preset, pairs })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture::RfModulation;
    use crate::radio::preset::{TxModulation, F4, FM476, HONDA1};

    fn fsk_dev(m: Option<TxModulation>) -> f64 {
        match m {
            Some(TxModulation::Fsk { deviation_hz }) => deviation_hz,
            other => panic!("expected FSK, got {other:?}"),
        }
    }

    fn capture(protocol: Option<&str>, received: Option<RfModulation>) -> Capture {
        let pairs = vec![
            StoredLevelDuration { level: true, duration_us: 500 },
            StoredLevelDuration { level: false, duration_us: 1000 },
            StoredLevelDuration { level: true, duration_us: 250 },
        ];
        let mut c = Capture::from_pairs_with_rf(1, 433_920_000, pairs, received);
        c.protocol = protocol.map(str::to_string);
        c
    }

    fn roundtrip(c: &Capture) -> (String, SubFile) {
        static N: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let n = N.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let path = std::env::temp_dir().join(format!("kat_sub_preset_{}_{n}.sub", std::process::id()));
        export_flipper_sub(c, &path).expect("export");
        let text = std::fs::read_to_string(&path).expect("read");
        let sub = import_sub(&path);
        let _ = std::fs::remove_file(&path);
        (text, sub.expect("import"))
    }

    /// The IMPORTS headers: stock OOK/FSK presets and the Honda captures' custom (Honda1) preset,
    /// CRLF line endings included.
    #[test]
    fn parses_stock_and_custom_presets() {
        let stock = "Filetype: Flipper SubGhz RAW File\r\nVersion: 1\r\nFrequency: 315000000\r\n\
                     Preset: FuriHalSubGhzPreset2FSKDev476Async\r\nProtocol: RAW\r\nRAW_Data: 55 -954 51\r\n";
        let sub = parse_sub(stock).unwrap();
        assert_eq!(sub.frequency, 315_000_000);
        assert_eq!(sub.pairs.len(), 3);
        let p = sub.preset.unwrap();
        assert_eq!(p.def().map(|d| d.name), Some("FM476"));
        assert!((fsk_dev(p.modulation()) - 47_607.42).abs() < 0.01);

        let ook = parse_sub("Frequency: 433920000\nPreset: FuriHalSubGhzPresetOok270Async\nRAW_Data: 1 -2\n").unwrap();
        let commas = parse_sub("RAW_Data: 1718, -32700, 32700,\n").unwrap();
        assert_eq!(commas.pairs.iter().map(|p| (p.level, p.duration_us)).collect::<Vec<_>>(), [(true, 1718), (false, 32700), (true, 32700)]);
        assert_eq!(ook.preset.unwrap().modulation(), Some(TxModulation::Ook));
        let fm238 = parse_sub("Preset: FuriHalSubGhzPreset2FSKDev238Async\nRAW_Data: 1 -2\n").unwrap();
        assert!((fsk_dev(fm238.preset.unwrap().modulation()) - 2_380.37).abs() < 0.01);

        let honda = "Filetype: Flipper SubGhz RAW File\r\nVersion: 1\r\nFrequency: 433657070\r\n\
            Preset: FuriHalSubGhzPresetCustom\r\nCustom_preset_module: CC1101\r\n\
            Custom_preset_data: 02 0D 0B 06 08 32 07 04 14 00 13 02 12 04 11 36 10 69 15 32 18 18 19 16 1D 91 1C 00 1B 07 20 FB 22 10 21 56 00 00 C0 00 00 00 00 00 00 00\r\n\
            Protocol: RAW\r\nRAW_Data: 190 -224 73\r\n";
        let p = parse_sub(honda).unwrap().preset.unwrap();
        assert!(p.is_custom());
        assert_eq!(p.module.as_deref(), Some("CC1101"));
        assert_eq!(p.data.as_deref(), Some(HONDA1.registers));
        assert_eq!(p.label(), "Honda1");
        assert!((fsk_dev(p.modulation()) - 15_869.14).abs() < 0.01);

        // No Preset line (hand-made files): no preset, the protocol decides at TX time.
        assert!(parse_sub("RAW_Data: 1 -2\n").unwrap().preset.is_none());
    }

    /// Every IMPORTS capture's preset parses; the FSK and custom ones are what the files say.
    #[test]
    fn imports_presets_parse() {
        let root = Path::new("IMPORTS");
        if !root.exists() {
            eprintln!("Skip: IMPORTS not found (run from crate root)");
            return;
        }
        let mut fsk = 0;
        for make in std::fs::read_dir(root).unwrap().flatten() {
            for f in std::fs::read_dir(make.path()).into_iter().flatten().flatten() {
                let path = f.path();
                if path.extension().is_none_or(|e| e != "sub") {
                    continue;
                }
                let Ok(sub) = import_sub(&path) else { continue }; // key files have no RAW_Data
                // `SUBARU/Subaru Forester 2012 AU.sub` lost its header (it starts mid-RAW_Data); it
                // still decodes, at the default 433.92 MHz, without a preset.
                if sub.preset.is_none() && !std::fs::read_to_string(&path).unwrap().contains("Filetype:") {
                    continue;
                }
                let p = sub.preset.unwrap_or_else(|| panic!("{path:?}: no preset"));
                // `BYD/*.sub` say "Custom" without the register lines: modulation unknown (the
                // decoders are then not gated by AM/FM).
                if p.is_custom() && p.data.is_none() {
                    continue;
                }
                let m = p.modulation().unwrap_or_else(|| panic!("{path:?}: {p:?}"));
                fsk += usize::from(m.is_fsk());
            }
        }
        // 10 FM476 RAW captures (Kia V3/V4/V5, Suzuki V0, the undecoded Lexus UX 250 pair) + 3
        // with the Honda1 custom preset (two Honda Static, one Ford Escape); the 9 FM476 key files
        // have no RAW_Data.
        assert_eq!(fsk, 10 + 3);
    }

    /// `.sub` export writes the capture's own preset back verbatim (stock name, or custom module +
    /// registers) so an imported capture round-trips.
    #[test]
    fn sub_roundtrip_keeps_the_capture_preset() {
        for preset in [
            RfPreset::from_def(&FM476),
            RfPreset::from_def(&HONDA1),
            RfPreset { name: "FuriHalSubGhzPresetOok270Async".into(), module: None, data: None },
            // A preset KAT does not know is still written back as it came.
            RfPreset { name: "FM20KR".into(), module: None, data: None },
        ] {
            let mut c = capture(Some("Kia V5"), None);
            c.rf_preset = Some(preset.clone());
            let (_, sub) = roundtrip(&c);
            assert_eq!(sub.preset.as_ref(), Some(&preset));
            assert_eq!(sub.pairs.len(), 3);
        }
    }

    /// `.sub` files are RAW (pairs) or key files (Protocol/Bit/Key); a RAW header without pairs is
    /// still RAW (and fails as before).
    #[test]
    fn key_files_are_told_apart_from_raw() {
        let raw = "Filetype: Flipper SubGhz RAW File\nVersion: 1\nFrequency: 433920000\n\
                   Preset: FuriHalSubGhzPresetOok650Async\nProtocol: RAW\nRAW_Data: 500 -500\n";
        assert!(matches!(parse_sub_content(raw), Ok(SubContent::Raw(s)) if s.pairs.len() == 2));
        let empty_raw = "Filetype: Flipper SubGhz RAW File\nVersion: 1\nProtocol: RAW\n";
        assert!(parse_sub_content(empty_raw).unwrap_err().to_string().contains("No RAW_Data"));
        let key = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                   Preset: FuriHalSubGhzPreset2FSKDev476Async\nProtocol: KIA/HYU V0\nBit: 61\n\
                   Key: 0F 73 91 67 BD 75 B2 BD\n";
        let SubContent::Key(k) = parse_sub_content(key).unwrap() else { panic!("key file") };
        assert_eq!((k.protocol.as_str(), k.bits, k.key), ("KIA/HYU V0", 61, 0x0F73_9167_BD75_B2BD));
        assert!(parse_sub_content("Filetype: Flipper SubGhz Key File\nVersion: 1\n").is_err());
    }

    /// A key file becomes a capture with the decode, the file's frequency and preset and no raw
    /// pairs; KAT writes it back as the same key file, which reloads as the same capture.
    #[test]
    fn key_file_capture_round_trip() {
        let reg = ProtocolRegistry::new();
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPreset2FSKDev476Async\nProtocol: KIA/HYU V0\nBit: 61\n\
                    Key: 0F 73 91 67 BD 75 B2 BD\n";
        let SubContent::Key(key) = parse_sub_content(text).unwrap() else { panic!("key file") };
        let c = capture_from_key(&key, &reg, 7).unwrap();
        assert_eq!((c.id, c.frequency, c.protocol.as_deref()), (7, 433_920_000, Some("Kia V0")));
        assert!(c.raw_pairs.is_empty() && !c.has_raw_data());
        assert_eq!((c.serial, c.button, c.counter, c.crc_valid), (Some(0x67B_D75B), Some(2), Some(0x7391), true));
        assert_eq!(c.status, CaptureStatus::EncoderCapable);
        assert_eq!(c.rf_preset.as_ref().and_then(|p| p.def()).map(|d| d.name), Some("FM476"));
        assert!(c.tx_modulation().is_fsk());

        // Export as KAT's (ProtoPirate's) key file and load it again.
        let path = std::env::temp_dir().join(format!("kat_key_rt_{}.sub", std::process::id()));
        export_flipper_key(&c, &reg, &path).unwrap();
        let written = std::fs::read_to_string(&path).unwrap();
        let back = read_sub(&path);
        let _ = std::fs::remove_file(&path);
        assert_eq!(
            written,
            "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
             Preset: FuriHalSubGhzPreset2FSKDev476Async\nProtocol: Kia V0\nBit: 61\n\
             Key: 0F 73 91 67 BD 75 B2 BD\nSerial: 108779355\nBtn: 2\nCnt: 29585\nType: 1\n"
        );
        let Ok(SubContent::Key(k2)) = back else { panic!("written key file reads back") };
        let c2 = capture_from_key(&k2, &reg, 8).unwrap();
        assert_eq!(
            (c2.protocol, c2.serial, c2.button, c2.counter, c2.data, c2.data_count_bit, c2.data_extra, c2.status, c2.rf_preset),
            (c.protocol.clone(), c.serial, c.button, c.counter, c.data, c.data_count_bit, c.data_extra, c.status, c.rf_preset.clone())
        );
        // A RAW export of it is refused (no pairs); an unknown protocol has no key file.
        assert!(export_flipper_sub(&c, &path).is_err());
        let mut unknown = c.clone();
        unknown.protocol = Some("No Such Protocol".into());
        assert!(capture_flipper_key(&unknown, &reg).is_none());
        let mut k3 = k2.clone();
        k3.protocol = "No Such Protocol".into();
        assert!(capture_from_key(&k3, &reg, 9).is_err());
    }

    /// A RAW capture's decode (with its custom CC1101 preset) exported as a key file keeps the
    /// preset lines where the Flipper expects them and reloads to the same decode.
    /// Urmet's frame does not fit the decode: a received frame still saves as an Urmet key file
    /// (`Raw` + `Gaps`, rebuilt from the recording), and that file loads with doctor-firmware's
    /// transmission as the capture's recording, which Replay sends.
    #[test]
    fn urmet_captures_keep_their_frame_through_key_files() {
        // A frame as the remote sends it: wake-up burst, 34 ms gap, preamble H1 L1 H1 L11, 18
        // inverted-USART bytes in the 5/7/D/F alphabet, a closing HIGH, 34 ms.
        let bytes = [0x55u8, 0x57, 0x5D, 0x5F, 0x75, 0x77, 0x7D, 0x7F, 0xD5, 0xD7, 0xDD, 0xDF, 0xF5, 0xF7, 0xFD, 0xFF, 0x55, 0x7D];
        let mut bits: Vec<bool> = [1u8, 0, 1].iter().chain([0u8; 11].iter()).map(|&b| b == 1).collect();
        for v in bytes {
            bits.push(true);
            bits.extend((0..8).map(|k| (v >> k) & 1 == 0));
            bits.push(false);
        }
        bits.push(true);
        let mut pairs: Vec<crate::radio::LevelDuration> = Vec::new();
        let mut push = |level: bool, d: u32| match pairs.last_mut() {
            Some(last) if last.level == level => last.duration_us += d,
            _ => pairs.push(crate::radio::LevelDuration::new(level, d)),
        };
        push(false, 20_000);
        for _ in 0..12 {
            push(true, 150);
            push(false, 150);
        }
        push(false, 34_000);
        for b in &bits {
            push(*b, 500);
        }
        push(false, 34_000);
        let mut reg = ProtocolRegistry::new();
        let (name, d, seg) = reg.process_signal_stream(&pairs, 433_920_000).into_iter().next().unwrap();
        assert_eq!(name, "Urmet");
        let mut c = Capture::from_pairs_with_rf(1, 433_920_000, seg.iter().map(|p| StoredLevelDuration { level: p.level, duration_us: p.duration_us }).collect(), None);
        c.protocol = Some(name.clone());
        c.data = d.data;
        c.data_count_bit = d.data_count_bit;
        let text = capture_flipper_key(&c, &reg).expect("an Urmet key file").to_text();
        assert!(text.contains("\nProtocol: Urmet\nBit: 152\nKey: FD 55 57 5D 5F 75 77 7D\nRaw: FD 55 57 5D"), "{text}");
        assert!(text.contains("\nGaps: "), "{text}");
        let SubContent::Key(k) = parse_sub_content(&text).unwrap() else { panic!() };
        let back = capture_from_key(&k, &reg, 2).unwrap();
        assert_eq!((back.protocol.as_deref(), back.data), (Some("Urmet"), d.data));
        // 5 copies of: per gap HIGH 500 + LOW n·500, a closing HIGH 500, LOW 34 ms.
        assert!(!back.raw_pairs.is_empty() && back.raw_pairs.len().is_multiple_of(5));
        assert_eq!(back.raw_pairs.iter().filter(|p| !p.level && p.duration_us == 34_000).count(), 5);
        assert!(back.raw_pairs.iter().filter(|p| p.level).all(|p| p.duration_us == 500));
        // A key file of any other protocol still loads without a recording.
        assert!(capture_from_key(&FlipperKey::parse("Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\nPreset: FuriHalSubGhzPresetOok650Async\nProtocol: CAME\nBit: 12\nKey: 00 00 00 00 00 00 0A BC\n").unwrap(), &reg, 3).unwrap().raw_pairs.is_empty());
    }

    #[test]
    fn raw_capture_exports_as_key_file() {
        crate::protocols::keys::load_keystore_from_embedded();
        let path = Path::new("IMPORTS/SUZUKI/Suzuki_v0_kd.sub");
        if !path.exists() {
            eprintln!("Skip: {path:?} not found (run from crate root)");
            return;
        }
        let sub = import_sub(path).unwrap();
        let pairs: Vec<crate::radio::LevelDuration> =
            sub.pairs.iter().map(|p| crate::radio::LevelDuration::new(p.level, p.duration_us)).collect();
        let mut reg = ProtocolRegistry::new();
        let (name, d, seg) = reg.process_signal_stream(&pairs, sub.frequency).into_iter().next().unwrap();
        let mut c = Capture::from_pairs_with_rf(1, sub.frequency, seg.iter().map(|p| StoredLevelDuration { level: p.level, duration_us: p.duration_us }).collect(), None);
        c.rf_preset = Some(RfPreset::from_def(&crate::radio::preset::HONDA1)); // a custom preset
        c.protocol = Some(name.clone());
        c.serial = d.serial;
        c.button = d.button;
        c.counter = d.counter;
        c.crc_valid = d.crc_valid;
        c.data = d.data;
        c.data_count_bit = d.data_count_bit;
        c.data_extra = d.extra;
        c.status = CaptureStatus::EncoderCapable;
        let key = capture_flipper_key(&c, &reg).unwrap();
        let text = key.to_text();
        assert!(text.contains("Preset: FuriHalSubGhzPresetCustom\nCustom_preset_module: CC1101\nCustom_preset_data: 02 0D"), "{text}");
        assert!(text.contains("\nProtocol: Kia V0\nBit: 64\nKey: F0 01 91 53 02 42 3C 10\nSerial: 22217282\nBtn: 3\nCnt: 25\nType: 2\n"), "{text}");
        let SubContent::Key(k) = parse_sub_content(&text).unwrap() else { panic!() };
        let back = capture_from_key(&k, &reg, 2).unwrap();
        assert_eq!(back.protocol.as_deref(), Some("Suzuki V0"));
        assert_eq!(back.decoded_signal(), c.decoded_signal());
        assert_eq!(back.rf_preset, c.rf_preset);
    }

    /// Without a preset of its own, export writes the one KAT transmits with: the references' AM
    /// preset for OOK, the protocol's FSK preset for FM (custom registers where no built-in fits).
    #[test]
    fn sub_export_writes_a_matching_preset() {
        // Ford V1 received on the FM path → custom F4 (2-FSK ±14.28 kHz).
        let (text, sub) = roundtrip(&capture(Some("Ford V1"), Some(RfModulation::FM)));
        assert!(text.contains("Preset: FuriHalSubGhzPresetCustom\nCustom_preset_module: CC1101\nCustom_preset_data: 02 0D 0B 06"), "{text}");
        assert_eq!(sub.preset.unwrap().def().map(|d| d.name), Some(F4.name));
        // Kia V5 (FM476, a built-in preset) is written by name.
        let (text, _) = roundtrip(&capture(Some("Kia V5"), None));
        assert!(text.contains("Preset: FuriHalSubGhzPreset2FSKDev476Async\nProtocol: RAW"), "{text}");
        // AM protocols and unknown OOK signals: AM650 (KAT ≤ 1.4 wrote Ook270 for everything).
        let (text, _) = roundtrip(&capture(Some("Kia V1"), Some(RfModulation::AM)));
        assert!(text.contains("Preset: FuriHalSubGhzPresetOok650Async\n"), "{text}");
        let (text, _) = roundtrip(&capture(None, None));
        assert!(text.contains("Preset: FuriHalSubGhzPresetOok650Async\n"), "{text}");
        // An unknown signal received as FM: FM476.
        let (text, _) = roundtrip(&capture(None, Some(RfModulation::FM)));
        assert!(text.contains("Preset: FuriHalSubGhzPreset2FSKDev476Async\n"), "{text}");
    }
}
