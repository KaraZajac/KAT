//! Flipper key files in the app (`docs/flipper_key_files.md`).
//!
//! - **Load**: a `.sub` without `RAW_Data` (`:load`, the load browser, the startup import) is a
//!   decoded key. The importer (`app/files.rs`) rebuilds its decode with the protocol's
//!   `from_flipper_key` (`crate::export::flipper::capture_from_key`): a capture with the protocol
//!   and fields, the file's frequency and preset, no raw pairs, and EncoderCapable when the encoder
//!   can transmit it.
//! - **Signal menu**: captures without raw pairs (key files, `.fob` files saved without pairs)
//!   cannot be replayed, exported as RAW `.sub` or timing-analysed, so those actions are hidden
//!   ([`App::adjust_signal_actions`]); TX Lock/Unlock/Trunk/Panic and "Send next code" re-encode
//!   from the fields and work as for any capture. "Export Flipper key" appears for every capture
//!   whose protocol writes key files.
//! - **Export**: [`App::export_flipper_key`] / [`App::complete_flipper_key_export`] write
//!   `<name>_key.sub` (so it does not take the RAW export's name) with the protocol's
//!   `to_flipper_key` (its reference's `serialize`), never over an existing file.

use anyhow::Result;

use super::{App, ExportFormat, InputMode, SignalAction};
use crate::capture::{Capture, CaptureStatus};
use crate::export::flipper;
use crate::protocols::ProtocolRegistry;

/// The signal menu for a capture: without raw pairs no Replay, no RAW `.sub` export and no timing
/// analysis (they need the pairs); "Export Flipper key" right after the RAW export's place (or
/// the `.fob` export's) when `key_file` (the protocol writes the capture as a key file).
pub(super) fn fit_signal_actions(actions: &mut Vec<SignalAction>, has_raw: bool, key_file: bool) {
    if !has_raw {
        actions.retain(|a| !matches!(a, SignalAction::Replay | SignalAction::ExportFlipper | SignalAction::Analyze));
    }
    if key_file {
        let at = actions
            .iter()
            .position(|a| matches!(a, SignalAction::ExportFlipper))
            .or_else(|| actions.iter().position(|a| matches!(a, SignalAction::ExportFob)))
            .map_or(actions.len(), |i| i + 1);
        actions.insert(at, SignalAction::ExportFlipperKey);
    }
}

/// Whether the capture's encoder produces a frame from its stored fields (what TX uses).
pub(super) fn encoder_works(registry: &ProtocolRegistry, capture: &Capture) -> bool {
    let d = capture.decoded_signal();
    capture
        .protocol
        .as_deref()
        .and_then(|p| registry.get(p))
        .is_some_and(|p| p.supports_encoding() && p.encode(&d, d.button.unwrap_or(0)).is_some())
}

/// A capture from a `.fob` saved without raw pairs keeps "EncoderCapable" only when its encoder
/// produces a frame from the stored fields (TX needs nothing else); otherwise it is Decoded.
pub(super) fn fit_encoder_status(registry: &ProtocolRegistry, capture: &mut Capture) {
    if !capture.has_raw_data() && capture.status == CaptureStatus::EncoderCapable && !encoder_works(registry, capture) {
        capture.status = CaptureStatus::Decoded;
    }
}

impl App {
    /// Whether the capture's protocol writes it as a Flipper key file.
    pub fn can_export_flipper_key(&self, capture: &Capture) -> bool {
        flipper::capture_flipper_key(capture, &self.protocols).is_some()
    }

    /// Fit the signal menu to the selected capture: without raw pairs, no Replay, no RAW `.sub`
    /// export and no Analyze; "Export Flipper key" (after the RAW export's place) when
    /// the protocol writes key files.
    pub(super) fn adjust_signal_actions(&self, actions: &mut Vec<SignalAction>) {
        if let Some(c) = self.selected_capture.and_then(|idx| self.captures.get(idx)) {
            fit_signal_actions(actions, c.has_raw_data(), self.can_export_flipper_key(c));
        }
    }

    /// Start a Flipper key file export: ask for the file name (default: the RAW export's with
    /// `_key`, so a capture's two exports do not share a name).
    pub fn export_flipper_key(&mut self, id: u32) -> Result<()> {
        let Some(capture) = self.captures.iter().find(|c| c.id == id) else {
            self.last_error = Some(format!("Capture #{:02} not found", id));
            return Ok(());
        };
        if !self.can_export_flipper_key(capture) {
            self.last_error = Some(format!("{} captures cannot be saved as a Flipper key file", capture.protocol_name()));
            return Ok(());
        }
        self.export_filename = format!("{}_key", Self::default_export_filename(capture));
        self.export_capture_id = Some(id);
        self.export_format = Some(ExportFormat::FlipperKey);
        self.input_mode = InputMode::ExportFilename;
        Ok(())
    }

    /// Write the Flipper key file (called after the file name is confirmed). Never over an
    /// existing file; a failure is reported in the status line (app/files.rs).
    pub fn complete_flipper_key_export(&mut self) -> Result<()> {
        let Some(id) = self.export_capture_id else {
            self.last_error = Some("No capture selected for export".to_string());
            return Ok(());
        };
        let Some(capture) = self.captures.iter().find(|c| c.id == id).cloned() else {
            self.last_error = Some(format!("Capture #{:02} not found", id));
            return Ok(());
        };
        self.export_capture_id = None;
        self.export_format = None;
        match self.export_path(&self.export_filename, "sub") {
            Ok(path) => {
                let result = flipper::export_flipper_key(&capture, &self.protocols, &path);
                self.report_export(&path, result);
            }
            Err(e) => self.last_error = Some(e),
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture::StoredLevelDuration;

    /// Captures without raw pairs lose Replay, the RAW export and Analyze; the key-file
    /// export sits after the RAW export (or the .fob export when the RAW one is gone).
    #[test]
    fn signal_menu_fits_the_capture() {
        use SignalAction::*;
        let car = SignalAction::ALL.to_vec();
        let mut a = car.clone();
        fit_signal_actions(&mut a, true, false);
        assert_eq!(a, car);
        let mut a = car.clone();
        fit_signal_actions(&mut a, true, true);
        assert_eq!(a, [Replay, Lock, Unlock, Trunk, Panic, ExportFob, ExportFlipper, ExportFlipperKey, Analyze, Delete]);
        let mut a = car.clone();
        fit_signal_actions(&mut a, false, true);
        assert_eq!(a, [Lock, Unlock, Trunk, Panic, ExportFob, ExportFlipperKey, Delete]);
        let gate = vec![Replay, SendNextCode, ExportFob, ExportFlipper, Analyze, Delete];
        let mut a = gate.clone();
        fit_signal_actions(&mut a, false, true);
        assert_eq!(a, [SendNextCode, ExportFob, ExportFlipperKey, Delete]);
        let mut a = vec![ExportFob, ExportFlipper, Analyze, Delete];
        fit_signal_actions(&mut a, false, false);
        assert_eq!(a, [ExportFob, Delete]);
    }

    /// The default export name never contains a path separator (makes such as "Kia/Hyundai").
    #[test]
    fn default_export_name_is_a_plain_file_name() {
        let mut c = Capture::from_pairs(1, 433_920_000, Vec::new());
        c.protocol = Some("Kia V0".into());
        c.button = Some(2);
        let name = App::default_export_filename(&c);
        assert_eq!(name, "Unknown_Kia-Hyundai_Unknown_Unknown_Unlock");
        c.make = Some("Honda/Acura".into());
        assert!(!App::default_export_filename(&c).contains('/'));
        // What the export form builds from its fields is the same name: an empty field is Unknown.
        assert_eq!(App::export_stem("", "Kia/Hyundai", " ", "", "Unlock"), name);
        assert_eq!(App::export_stem("2015", "Kia", "Sportage SX", "NA", "-"), "2015_Kia_Sportage_SX_NA_Unknown");
        // An unknown capture has nothing to name it by.
        let unknown = Capture::from_pairs(2, 433_920_000, Vec::new());
        assert_eq!(App::default_export_filename(&unknown), "Unknown_Unknown_Unknown_Unknown_Unknown");
    }

    /// A key-file capture saved as `.fob` (no raw pairs) imports EncoderCapable again: its encoder
    /// works from the stored fields. A capture whose encoder cannot transmit does not.
    #[test]
    fn fob_without_pairs_keeps_tx_when_the_encoder_works() {
        let reg = ProtocolRegistry::new();
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPreset2FSKDev476Async\nProtocol: Kia V0\nBit: 61\n\
                    Key: 0F 73 91 67 BD 75 B2 BD\n";
        let key = crate::protocols::FlipperKey::parse(text).unwrap();
        let c = flipper::capture_from_key(&key, &reg, 1).unwrap();
        let path = std::env::temp_dir().join(format!("kat_key_fob_{}.fob", std::process::id()));
        crate::export::fob::export_fob(&c, &path, true, None).unwrap();
        let imported = crate::export::fob::import_fob(&path, 2);
        let _ = std::fs::remove_file(&path);
        let imported = imported.unwrap();
        assert!(imported.raw_pairs.is_empty());
        assert_eq!(imported.status, CaptureStatus::EncoderCapable);
        assert!(encoder_works(&reg, &imported));
        assert_eq!(imported.decoded_signal(), c.decoded_signal());
        assert_eq!(imported.rf_preset, c.rf_preset);

        let mut unknown = Capture::from_pairs(3, 433_920_000, vec![StoredLevelDuration { level: true, duration_us: 1 }]);
        unknown.protocol = Some("No Such Protocol".into());
        assert!(!encoder_works(&reg, &unknown));
    }
}
