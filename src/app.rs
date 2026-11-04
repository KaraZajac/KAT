//! Application state management.

use anyhow::Result;
use std::path::PathBuf;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::Arc;

use crate::capture::{ButtonCommand, Capture};
use crate::protocols::{
    is_keeloq_non_car, psa_recover_key, ProtocolRegistry, PsaBruteForceControl, PsaBruteForceOutcome,
    PSA_BRUTE_FORCE_TOTAL,
};
use crate::radio::{HackRfController, LevelDuration, RtlSdrController};
use crate::storage::Storage;

mod rx;

/// Active radio device: HackRF (full TX/RX) or RTL-SDR (receive-only).
pub enum RadioDevice {
    HackRf(HackRfController),
    RtlSdr(RtlSdrController),
}

impl RadioDevice {
    pub fn is_available(&self) -> bool {
        match self {
            RadioDevice::HackRf(h) => h.is_available(),
            RadioDevice::RtlSdr(r) => r.is_available(),
        }
    }

    pub fn supports_tx(&self) -> bool {
        match self {
            RadioDevice::HackRf(h) => h.supports_tx(),
            RadioDevice::RtlSdr(r) => r.supports_tx(),
        }
    }

    pub fn start_receiving(&mut self, frequency: u32) -> anyhow::Result<()> {
        match self {
            RadioDevice::HackRf(h) => h.start_receiving(frequency),
            RadioDevice::RtlSdr(r) => r.start_receiving(frequency),
        }
    }

    pub fn stop_receiving(&mut self) -> anyhow::Result<()> {
        match self {
            RadioDevice::HackRf(h) => h.stop_receiving(),
            RadioDevice::RtlSdr(r) => r.stop_receiving(),
        }
    }

    pub fn set_frequency(&mut self, frequency: u32) -> anyhow::Result<()> {
        match self {
            RadioDevice::HackRf(h) => h.set_frequency(frequency),
            RadioDevice::RtlSdr(r) => r.set_frequency(frequency),
        }
    }

    pub fn set_lna_gain(&mut self, gain: u32) -> anyhow::Result<()> {
        match self {
            RadioDevice::HackRf(h) => h.set_lna_gain(gain),
            RadioDevice::RtlSdr(r) => r.set_lna_gain(gain),
        }
    }

    pub fn set_vga_gain(&mut self, gain: u32) -> anyhow::Result<()> {
        match self {
            RadioDevice::HackRf(h) => h.set_vga_gain(gain),
            RadioDevice::RtlSdr(r) => r.set_vga_gain(gain),
        }
    }

    pub fn set_amp_enable(&mut self, enabled: bool) -> anyhow::Result<()> {
        match self {
            RadioDevice::HackRf(h) => h.set_amp_enable(enabled),
            RadioDevice::RtlSdr(r) => r.set_amp_enable(enabled),
        }
    }

    pub fn transmit(
        &mut self,
        signal: &[LevelDuration],
        frequency: u32,
        settings: &crate::radio::tx::TxSettings,
    ) -> anyhow::Result<()> {
        match self {
            RadioDevice::HackRf(h) => h.transmit(signal, frequency, settings),
            RadioDevice::RtlSdr(r) => r.transmit(signal, frequency),
        }
    }

    /// Display name for UI (e.g. "HackRF", "RTL-SDR (RX only)").
    pub fn display_name(&self) -> &'static str {
        match self {
            RadioDevice::HackRf(_) => "HackRF",
            RadioDevice::RtlSdr(_) => "RTL-SDR (RX only)",
        }
    }

    /// Shared atomic for RSSI (UI reads so RX never blocks on channel). None if no radio.
    pub fn rssi_source(&self) -> Option<Arc<AtomicU32>> {
        match self {
            RadioDevice::HackRf(h) => Some(h.rssi_source()),
            RadioDevice::RtlSdr(r) => Some(r.rssi_source()),
        }
    }
}

// v1.5 app features: saved session, saved-fob matching, timing analysis (impl App blocks).
mod fob_matching;
mod persistence;
mod timing_view;

/// Input mode for the application
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InputMode {
    /// Normal navigation mode
    Normal,
    /// Command input mode (after pressing :)
    Command,
    /// Signal action popup menu
    SignalMenu,
    /// Tab bar - selecting which radio setting
    SettingsSelect,
    /// Editing a radio setting value
    SettingsEdit,
    /// Startup warning: HackRF not detected (dismiss to continue)
    HackRfNotDetected,
    /// Startup prompt: found .fob files, import? (y/n)
    StartupImport,
    /// Startup prompt: restore the captures of the saved session? (y/n)
    SessionRestore,
    /// `:clear` confirmation: delete every capture and the saved session? (y/n)
    ConfirmClear,
    /// Timing analysis overlay (scrollable text)
    TimingAnalysis,
    /// Export: editing filename (before format-specific steps)
    ExportFilename,
    /// Fob export metadata: editing year field
    FobMetaYear,
    /// Fob export metadata: editing make field
    FobMetaMake,
    /// Fob export metadata: editing model field
    FobMetaModel,
    /// Fob export metadata: editing region field
    FobMetaRegion,
    /// Fob export metadata: editing command field
    FobMetaCommand,
    /// Fob export metadata: editing notes field
    FobMetaNotes,
    /// Capture metadata (Year/Make/Model/Region/Command) for vuln lookup — press i on a capture
    CaptureMetaYear,
    CaptureMetaMake,
    CaptureMetaModel,
    CaptureMetaRegion,
    CaptureMetaCommand,
    /// License overlay (centered box)
    License,
    /// Credits overlay (centered box)
    Credits,
    /// :load file browser (import .fob/.sub from import dir)
    LoadFileBrowser,
    /// `:new`: the "Add Manually" type picker (app/add_manually.rs)
    NewRemote,
    /// `:new`: editing a type's serial / button / counter / seed / key before creating it
    NewRemoteEdit,
    /// Signal menu "Send button…": a gate remote's native buttons
    SendButton,
}

/// Export format being used
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExportFormat {
    Fob,
    Flipper,
    /// Flipper key file (`.sub` with Protocol/Bit/Key, no RAW_Data; app/key_files.rs).
    FlipperKey,
}

mod tx;
pub use tx::{tx_power_label, TX_POWER_STEPS};
mod key_files;
// `:new` ("Add Manually" remotes) and the signal menu's "Send button…".
mod add_manually;
mod keeloq_bf;
pub use add_manually::{PickerRow, PICKER_VISIBLE_ROWS};

/// Items available in the signal action menu
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SignalAction {
    Replay,
    SendNextCode,
    /// Gate / garage remotes: pick one of the protocol's native buttons to send (app/add_manually.rs).
    SendButton,
    Lock,
    Unlock,
    Trunk,
    Panic,
    ExportFob,
    ExportFlipper,
    /// Flipper key file via the protocol's `to_flipper_key` (only when it writes one).
    ExportFlipperKey,
    /// Pulse timing vs the protocol definition (ProtoPirate's Timing Tuner), clusters for unknowns.
    TimingAnalysis,
    Delete,
    /// PSA TEA frame: run ProtoPirate's key brute force on a background thread.
    RecoverKey,
    /// KeeLoq frame no key decrypts: Flipper-ARF's magic-serial key search (app/keeloq_bf.rs).
    RecoverKeeloqKey,
    /// Stop a running "Recover key".
    CancelKeyRecovery,
}

impl SignalAction {
    /// All actions (car keyfob menu). Barrier/alarm menu is built separately to include SendNextCode only there.
    pub const ALL: [SignalAction; 9] = [
        SignalAction::Replay,
        SignalAction::Lock,
        SignalAction::Unlock,
        SignalAction::Trunk,
        SignalAction::Panic,
        SignalAction::ExportFob,
        SignalAction::ExportFlipper,
        SignalAction::TimingAnalysis,
        SignalAction::Delete,
    ];

    pub fn label(&self) -> &'static str {
        match self {
            SignalAction::Replay => "Replay",
            SignalAction::SendNextCode => "Send next code",
            SignalAction::SendButton => "Send button…",
            SignalAction::Lock => "TX Lock",
            SignalAction::Unlock => "TX Unlock",
            SignalAction::Trunk => "TX Trunk",
            SignalAction::Panic => "TX Panic",
            SignalAction::ExportFob => "Export .fob",
            SignalAction::ExportFlipper => "Export .sub (Flipper)",
            SignalAction::ExportFlipperKey => "Export Flipper key (.sub)",
            SignalAction::TimingAnalysis => "Timing analysis",
            SignalAction::Delete => "Delete Signal",
            SignalAction::RecoverKey => "Recover key (PSA brute force)",
            SignalAction::RecoverKeeloqKey => "Recover key (KeeLoq magic serial)",
            SignalAction::CancelKeyRecovery => "Cancel key recovery",
        }
    }
}

/// Radio settings selectable via Tab
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SettingsField {
    Freq,
    /// Frequency hopping on/off
    Hop,
    Lna,
    Vga,
    Amp,
    /// TX power (TX VGA gain + RF amp) used for every transmission.
    TxPower,
}

impl SettingsField {
    pub const ALL: &'static [SettingsField] = &[
        SettingsField::Freq,
        SettingsField::Hop,
        SettingsField::Lna,
        SettingsField::Vga,
        SettingsField::Amp,
        SettingsField::TxPower,
    ];

    pub fn label(&self) -> &'static str {
        match self {
            SettingsField::Freq => "Freq",
            SettingsField::Hop => "Hop",
            SettingsField::Lna => "LNA",
            SettingsField::Vga => "VGA",
            SettingsField::Amp => "AMP",
            SettingsField::TxPower => "TX",
        }
    }
}

/// Common keyfob frequencies (Hz)
pub const PRESET_FREQUENCIES: [(u32, &str); 10] = [
    (300_000_000, "300.00 MHz"),
    (303_875_000, "303.875 MHz"),
    (310_000_000, "310.00 MHz"),
    (315_000_000, "315.00 MHz"),
    (318_000_000, "318.00 MHz"),
    (390_000_000, "390.00 MHz"),
    (433_920_000, "433.92 MHz"),
    (434_420_000, "434.42 MHz"),
    (868_350_000, "868.35 MHz"),
    (915_000_000, "915.00 MHz"),
];

/// LNA gain steps (dB)
pub const LNA_STEPS: [u32; 6] = [0, 8, 16, 24, 32, 40];

/// VGA gain steps (dB, subset for menu)
pub const VGA_STEPS: [u32; 8] = [0, 8, 16, 20, 24, 32, 40, 62];

/// Radio state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RadioState {
    /// Not connected
    Disconnected,
    /// Connected but idle
    Idle,
    /// Receiving signals
    Receiving,
    /// Transmitting
    #[allow(dead_code)]
    Transmitting,
}

impl std::fmt::Display for RadioState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RadioState::Disconnected => write!(f, "DISCONNECTED"),
            RadioState::Idle => write!(f, "IDLE"),
            RadioState::Receiving => write!(f, "RX"),
            RadioState::Transmitting => write!(f, "TX"),
        }
    }
}

/// Events from the radio subsystem
pub enum RadioEvent {
    /// New signal captured
    SignalCaptured(Capture),
    /// Error occurred
    Error(String),
    /// State changed
    #[allow(dead_code)]
    StateChanged(RadioState),
}

/// License text (embedded at compile time)
pub const LICENSE_TEXT: &str = include_str!("../LICENSE");

/// A running PSA "Recover key" brute force (ProtoPirate's `psa_brute_force_run`, on a thread so the
/// UI stays responsive; ≈0.5 s worst case on a 16-core PC, longer on small machines).
struct PsaRecoverJob {
    capture_id: u32,
    ctl: Arc<PsaBruteForceControl>,
    handle: std::thread::JoinHandle<PsaBruteForceOutcome>,
}

/// Main application state
pub struct App {
    /// Current input mode
    pub input_mode: InputMode,
    /// Command input buffer
    pub command_input: String,
    /// List of captures
    pub captures: Vec<Capture>,
    /// Currently selected capture index
    pub selected_capture: Option<usize>,
    /// Scroll offset for captures list
    pub scroll_offset: usize,
    /// Current frequency in Hz
    pub frequency: u32,
    /// LNA gain (0-40 dB, 8 dB steps)
    pub lna_gain: u32,
    /// VGA gain (0-62 dB, 2 dB steps)
    pub vga_gain: u32,
    /// Amplifier enabled
    pub amp_enabled: bool,
    /// Radio state
    pub radio_state: RadioState,
    /// Last error message
    pub last_error: Option<String>,
    /// Last status message
    pub status_message: Option<String>,
    /// Latest RSSI (average magnitude, 0..~1) from receiver
    pub rssi: f32,

    // -- Signal action menu state --
    /// Currently selected signal menu item index
    pub signal_menu_index: usize,

    // -- License/Credits overlay --
    /// Scroll offset for license/credits overlay (lines)
    pub overlay_scroll: usize,

    // -- Settings menu state --
    /// Currently selected settings field
    pub settings_field_index: usize,
    /// Currently selected value index within the settings field editor
    pub settings_value_index: usize,

    /// Next capture ID
    next_capture_id: u32,
    /// Storage manager
    pub storage: Storage,
    /// Protocol registry
    protocols: ProtocolRegistry,
    /// Active radio device (HackRF or RTL-SDR), if any
    radio: Option<RadioDevice>,
    /// Channel for radio events
    radio_event_rx: Receiver<RadioEvent>,
    /// Sender for radio events (cloned to radio thread)
    #[allow(dead_code)]
    radio_event_tx: Sender<RadioEvent>,
    /// RSSI read from radio atomic (no channel traffic, avoids RX blocking)
    pub rssi_source: Option<Arc<AtomicU32>>,
    /// IQ files, IQ recording and frequency hopping (src/app/rx.rs)
    pub rx: rx::RxExtras,
    /// Set by :q / :quit so the main loop can exit cleanly (terminal cleanup)
    pub quit_requested: bool,

    // -- Startup import state --
    /// .fob files found on startup in export_dir
    pub pending_fob_files: Vec<std::path::PathBuf>,

    // -- Export state --
    /// Capture ID being exported
    pub export_capture_id: Option<u32>,
    /// Export filename input buffer (without extension)
    pub export_filename: String,
    /// Which export format is in progress
    pub export_format: Option<ExportFormat>,

    // -- .fob export metadata state --
    /// Year input buffer
    pub fob_meta_year: String,
    /// Make input buffer
    pub fob_meta_make: String,
    /// Model input buffer
    pub fob_meta_model: String,
    /// Region input buffer (e.g. NA, EU, APAC, etc.)
    pub fob_meta_region: String,
    /// Command input buffer (e.g. Unlock, Lock)
    pub fob_meta_command: String,
    /// Notes input buffer
    pub fob_meta_notes: String,

    // -- Capture metadata (Year/Make/Model/Region/Command for vuln lookup, set via 'i') --
    pub capture_meta_year: String,
    pub capture_meta_make: String,
    pub capture_meta_model: String,
    pub capture_meta_region: String,
    pub capture_meta_command: String,
    /// Which capture is being edited (when in CaptureMeta* modes)
    pub capture_meta_capture_id: Option<u32>,

    // -- Pending transmit (so UI can draw TX state before blocking) --
    /// Queued transmissions (signal, frequency, modulation); the main loop draws, then runs them
    /// one at a time, first in first out.
    pending_transmit_queue: tx::TransmitQueue,
    /// TX VGA gain in dB (0-47) for every transmission (config `tx_vga_gain`, `:txpower`).
    pub tx_vga_gain: u32,
    /// TX RF amplifier on/off (config `tx_amp`, `:txpower ... amp on|off`).
    pub tx_amp: bool,
    /// State to restore when queue becomes empty (set when first item is queued).
    pending_transmit_restore: Option<RadioState>,

    /// PSA key recovery running in the background (at most one key recovery at a time).
    psa_recover_job: Option<PsaRecoverJob>,
    /// KeeLoq magic-serial key search running in the background.
    keeloq_bf_job: Option<keeloq_bf::KeeloqBfJob>,

    // -- :load file browser --
    /// Current directory in the load file browser
    pub load_browser_cwd: PathBuf,
    /// Selected index in the file list
    pub load_browser_selected: usize,
    /// Scroll offset for the file list (so selection stays in view)
    pub load_browser_scroll: usize,
    /// Entries: (display name, full path, is_dir)
    pub load_browser_entries: Vec<(String, PathBuf, bool)>,

    // -- v1.5: saved session, saved-fob matching, timing analysis --
    /// Session autosave/restore (app/persistence.rs)
    session: persistence::SessionState,
    /// Saved .fob index and per-capture matches (app/fob_matching.rs)
    saved_fobs: fob_matching::SavedFobState,
    /// Open timing-analysis overlay (app/timing_view.rs)
    timing_view: Option<timing_view::TimingView>,
    /// `:new` picker / field form and the "Send button…" list (app/add_manually.rs)
    add_manually: add_manually::AddManuallyState,
}

impl App {
    /// Create a new application instance
    pub fn new() -> Result<Self> {
        let storage = Storage::new()?;

        // ── Load protocol encryption keys from embedded keystore ─────────
        crate::protocols::keys::load_keystore_from_embedded();
        // ── … plus the user's KeeLoq keys (Flipper `keeloq_mfcodes_user` format), if present ──
        let user_keystore_msg = crate::keystore::load_user_keystore(storage.config_dir());

        let protocols = ProtocolRegistry::new();
        let (radio_event_tx, radio_event_rx) = mpsc::channel();

        // Try HackRF first, then RTL-SDR
        let radio: Option<RadioDevice> = match HackRfController::new(radio_event_tx.clone()) {
            Ok(mut h) if h.is_available() => {
                tracing::info!("HackRF initialized successfully");
                let _ = h.set_lna_gain(storage.config.default_lna_gain);
                let _ = h.set_vga_gain(storage.config.default_vga_gain);
                let _ = h.set_amp_enable(storage.config.default_amp);
                Some(RadioDevice::HackRf(h))
            }
            _ => match RtlSdrController::new(radio_event_tx.clone()) {
                Ok(mut r) if r.is_available() => {
                    tracing::info!("RTL-SDR initialized (receive-only)");
                    let _ = r.set_lna_gain(storage.config.default_lna_gain);
                    let _ = r.set_vga_gain(storage.config.default_vga_gain);
                    Some(RadioDevice::RtlSdr(r))
                }
                _ => None
            }
        };

        let device_detected = radio.as_ref().map_or(false, |r| r.is_available());

        let radio_state = if device_detected {
            RadioState::Idle
        } else {
            RadioState::Disconnected
        };

        // Captures start empty; the saved session (if any) is offered for restore, then the
        // .fob/.sub files of the import folder (see app/persistence.rs).
        let captures: Vec<Capture> = Vec::new();
        let next_capture_id = 1u32;

        // Use config defaults for radio settings
        let frequency = storage.config.default_frequency;
        let lna_gain = storage.config.default_lna_gain;
        let vga_gain = storage.config.default_vga_gain;
        let amp_enabled = storage.config.default_amp;
        let (tx_vga_gain, tx_amp) = (storage.config.tx_vga_gain.min(tx::TX_VGA_MAX), storage.config.tx_amp);

        // Recursively scan import directory for .fob and .sub at startup (separate from export dir)
        let pending_fob_files =
            crate::export::scan_import_files_recursive(storage.import_dir());
        let initial_mode = if !device_detected {
            InputMode::HackRfNotDetected
        } else if !pending_fob_files.is_empty() {
            tracing::info!(
                "Found {} importable file(s) in import dir (recursive)",
                pending_fob_files.len()
            );
            InputMode::StartupImport
        } else {
            InputMode::Normal
        };

        let rssi_source = radio.as_ref().and_then(|r| r.rssi_source());

        let mut app = Self {
            input_mode: initial_mode,
            command_input: String::new(),
            captures,
            selected_capture: None,
            scroll_offset: 0,
            frequency,
            lna_gain,
            vga_gain,
            amp_enabled,
            radio_state,
            last_error: None,
            status_message: None,
            rssi: 0.0,
            signal_menu_index: 0,
            overlay_scroll: 0,
            settings_field_index: 0,
            settings_value_index: 0,
            next_capture_id,
            storage,
            protocols,
            radio,
            radio_event_rx,
            radio_event_tx,
            rssi_source,
            rx: rx::RxExtras::new(),
            quit_requested: false,
            pending_fob_files,
            export_capture_id: None,
            export_filename: String::new(),
            export_format: None,
            fob_meta_year: String::new(),
            fob_meta_make: String::new(),
            fob_meta_model: String::new(),
            fob_meta_region: String::new(),
            fob_meta_command: String::new(),
            fob_meta_notes: String::new(),
            capture_meta_year: String::new(),
            capture_meta_make: String::new(),
            capture_meta_model: String::new(),
            capture_meta_region: String::new(),
            capture_meta_command: String::new(),
            capture_meta_capture_id: None,
            pending_transmit_queue: tx::TransmitQueue::new(),
            tx_vga_gain,
            tx_amp,
            pending_transmit_restore: None,
            psa_recover_job: None,
            keeloq_bf_job: None,
            load_browser_cwd: PathBuf::new(),
            load_browser_selected: 0,
            load_browser_scroll: 0,
            load_browser_entries: Vec::new(),
            session: persistence::SessionState::default(),
            saved_fobs: fob_matching::SavedFobState::default(),
            timing_view: None,
            add_manually: add_manually::AddManuallyState::default(),
        };
        // Saved-session restore prompt, session writer, saved-fob index (app/persistence.rs).
        app.init_app_extensions();
        if let Some(msg) = user_keystore_msg {
            tracing::info!("{msg}");
            app.status_message.get_or_insert(msg);
        }
        Ok(app)
    }

    /// Get the frequency in MHz
    pub fn frequency_mhz(&self) -> f64 {
        self.frequency as f64 / 1_000_000.0
    }

    /// Display name of the active radio device, if any (e.g. "HackRF", "RTL-SDR (RX only)").
    pub fn radio_device_name(&self) -> Option<&'static str> {
        self.radio.as_ref().map(|r| r.display_name())
    }

    /// True if the active device supports transmit (HackRF only; RTL-SDR is receive-only).
    #[allow(dead_code)]
    pub fn can_transmit(&self) -> bool {
        self.radio.as_ref().map_or(false, |r| r.supports_tx())
    }

    /// Select the next capture in the list
    pub fn next_capture(&mut self) {
        if self.captures.is_empty() {
            return;
        }
        self.selected_capture = Some(match self.selected_capture {
            Some(i) => (i + 1).min(self.captures.len() - 1),
            None => 0,
        });
        // Update scroll to keep selection visible
        self.ensure_selection_visible();
    }

    /// Select the previous capture in the list
    pub fn previous_capture(&mut self) {
        if self.captures.is_empty() {
            return;
        }
        self.selected_capture = Some(match self.selected_capture {
            Some(i) => i.saturating_sub(1),
            None => 0,
        });
        // Update scroll to keep selection visible
        self.ensure_selection_visible();
    }

    /// Ensure the selected capture is visible in the scroll view
    fn ensure_selection_visible(&mut self) {
        if let Some(selected) = self.selected_capture {
            // Assume visible area is about 15 items (will be adjusted by UI)
            let visible_rows = 15;
            
            if selected < self.scroll_offset {
                self.scroll_offset = selected;
            } else if selected >= self.scroll_offset + visible_rows {
                self.scroll_offset = selected.saturating_sub(visible_rows - 1);
            }
        }
    }

    /// Toggle receiving state
    pub fn toggle_receiving(&mut self) -> Result<()> {
        // Clear any previous error when user takes action
        self.last_error = None;
        
        match self.radio_state {
            RadioState::Disconnected => {
                self.last_error = Some("No radio device connected".to_string());
            }
            RadioState::Idle => {
                if let Some(ref mut radio) = self.radio {
                    radio.start_receiving(self.frequency)?;
                    self.radio_state = RadioState::Receiving;
                    self.status_message = Some(format!("Receiving on {:.2} MHz", self.frequency_mhz()));
                }
            }
            RadioState::Receiving => {
                if let Some(ref mut radio) = self.radio {
                    radio.stop_receiving()?;
                    self.radio_state = RadioState::Idle;
                    self.status_message = Some("Stopped receiving".to_string());
                }
            }
            RadioState::Transmitting => {
                self.last_error = Some("Cannot change state while transmitting".to_string());
            }
        }
        Ok(())
    }

    /// Parse an ID spec into a list of capture IDs in order.
    /// Supports: single "1", comma-separated "1, 3, 5", range "1-5", and mixed "1, 3-5, 7".
    fn parse_id_spec(s: &str) -> Result<Vec<u32>, String> {
        let mut ids = Vec::new();
        for part in s.split(',') {
            let part = part.trim();
            if part.is_empty() {
                continue;
            }
            if let Some((low, high)) = part.split_once('-') {
                let low = low.trim().parse::<u32>().map_err(|_| "Invalid ID in range".to_string())?;
                let high = high.trim().parse::<u32>().map_err(|_| "Invalid ID in range".to_string())?;
                if low <= high {
                    ids.extend(low..=high);
                } else {
                    ids.extend((high..=low).rev());
                }
            } else {
                let id = part.parse::<u32>().map_err(|_| "Invalid capture ID".to_string())?;
                ids.push(id);
            }
        }
        if ids.is_empty() {
            return Err("No valid IDs".to_string());
        }
        Ok(ids)
    }

    /// Execute a command
    pub fn execute_command(&mut self, command: &str) -> Result<()> {
        let parts: Vec<&str> = command.trim().split_whitespace().collect();
        if parts.is_empty() {
            return Ok(());
        }

        self.last_error = None;
        self.status_message = None;

        match parts[0] {
            "q" | "quit" => {
                self.quit_requested = true;
            }
            "freq" => {
                if parts.len() < 2 {
                    self.last_error = Some("Usage: :freq <MHz>".to_string());
                    return Ok(());
                }
                match parts[1].parse::<f64>() {
                    Ok(mhz) => {
                        let hz = (mhz * 1_000_000.0) as u32;
                        self.set_frequency(hz)?;
                    }
                    Err(_) => {
                        self.last_error = Some("Invalid frequency".to_string());
                    }
                }
            }
            "iq" => self.cmd_iq(&parts[1..])?,
            "record" | "rec" => self.cmd_record(&parts[1..])?,
            "hop" => self.cmd_hop(&parts[1..])?,
            "unlock" => self.transmit_command(parts.get(1).map(|_| parts[1..].join(" ")), ButtonCommand::Unlock)?,
            "lock" => self.transmit_command(parts.get(1).map(|_| parts[1..].join(" ")), ButtonCommand::Lock)?,
            "trunk" => self.transmit_command(parts.get(1).map(|_| parts[1..].join(" ")), ButtonCommand::Trunk)?,
            "panic" => self.transmit_command(parts.get(1).map(|_| parts[1..].join(" ")), ButtonCommand::Panic)?,
            "license" | "licence" => {
                self.input_mode = InputMode::License;
                self.overlay_scroll = 0;
            }
            "credits" => {
                self.input_mode = InputMode::Credits;
                self.overlay_scroll = 0;
            }
            "load" => {
                // The rest of the line, so file names with spaces work.
                let arg = command.trim()[parts[0].len()..].trim();
                if arg.is_empty() {
                    self.open_load_browser()?;
                } else {
                    self.load_path(arg)?;
                }
            }
            "new" => self.new_remote_command(&parts[1..]),
            "delete" => {
                if parts.len() < 2 {
                    self.last_error = Some("Usage: :delete <ID> or :delete all".to_string());
                    return Ok(());
                }
                if parts[1].eq_ignore_ascii_case("all") {
                    self.delete_all_captures()?;
                } else {
                    self.delete_capture(parts[1])?;
                }
            }
            "timing" => {
                self.timing_command(parts.get(1).copied());
            }
            "klbf" => {
                self.klbf_command(&parts[1..]);
            }
            "clear" => {
                self.request_clear();
            }
            "replay" => {
                let id_spec = parts.get(1).map(|_| parts[1..].join(" "));
                let id_spec = match id_spec.as_deref() {
                    Some(s) if !s.is_empty() => s,
                    _ => {
                        self.last_error = Some("Usage: :replay <ID> (e.g. 1, 1,3,5, 1-5)".to_string());
                        return Ok(());
                    }
                };
                match Self::parse_id_spec(id_spec) {
                    Ok(ids) => {
                        for id in ids {
                            self.replay_capture(id)?;
                        }
                    }
                    Err(e) => self.last_error = Some(e),
                }
            }
            "lna" => {
                if parts.len() < 2 {
                    self.last_error = Some("Usage: :lna <0-40>".to_string());
                    return Ok(());
                }
                match parts[1].parse::<u32>() {
                    Ok(gain) => self.set_lna_gain(gain)?,
                    Err(_) => {
                        self.last_error = Some("Invalid LNA gain value".to_string());
                    }
                }
            }
            "vga" => {
                if parts.len() < 2 {
                    self.last_error = Some("Usage: :vga <0-62>".to_string());
                    return Ok(());
                }
                match parts[1].parse::<u32>() {
                    Ok(gain) => self.set_vga_gain(gain)?,
                    Err(_) => {
                        self.last_error = Some("Invalid VGA gain value".to_string());
                    }
                }
            }
            "amp" => {
                if parts.len() < 2 {
                    // Toggle if no argument
                    self.toggle_amp()?;
                } else {
                    match parts[1].to_lowercase().as_str() {
                        "on" | "1" | "true" => self.set_amp(true)?,
                        "off" | "0" | "false" => self.set_amp(false)?,
                        _ => {
                            self.last_error = Some("Usage: :amp [on|off]".to_string());
                        }
                    }
                }
            }
            "txpower" => self.txpower_command(&parts[1..]),
            _ => {
                self.last_error = Some(format!("Unknown command: {}", parts[0]));
            }
        }

        Ok(())
    }

    /// Set the receive frequency
    fn set_frequency(&mut self, hz: u32) -> Result<()> {
        // Validate frequency range (common keyfob frequencies)
        if hz < 300_000_000 || hz > 928_000_000 {
            self.last_error = Some("Frequency must be between 300-928 MHz".to_string());
            return Ok(());
        }

        self.frequency = hz;

        // If receiving, restart receiver so the new frequency takes effect
        if let Some(ref mut radio) = self.radio {
            if self.radio_state == RadioState::Receiving {
                radio.stop_receiving()?;
                radio.start_receiving(hz)?;
            } else {
                radio.set_frequency(hz)?;
            }
        }

        self.status_message = Some(format!("Frequency set to {:.2} MHz", hz as f64 / 1_000_000.0));
        Ok(())
    }

    /// Set the LNA gain
    fn set_lna_gain(&mut self, gain: u32) -> Result<()> {
        // LNA gain is 0-40 dB in 8 dB steps
        if gain > 40 {
            self.last_error = Some("LNA gain must be 0-40 dB".to_string());
            return Ok(());
        }
        
        // Round to nearest 8 dB step
        let gain = (gain / 8) * 8;
        self.lna_gain = gain;

        if let Some(ref mut radio) = self.radio {
            radio.set_lna_gain(gain)?;
        }

        self.status_message = Some(format!("LNA gain set to {} dB", gain));
        Ok(())
    }

    /// Set the VGA gain
    fn set_vga_gain(&mut self, gain: u32) -> Result<()> {
        // VGA gain is 0-62 dB in 2 dB steps
        if gain > 62 {
            self.last_error = Some("VGA gain must be 0-62 dB".to_string());
            return Ok(());
        }
        
        // Round to nearest 2 dB step
        let gain = (gain / 2) * 2;
        self.vga_gain = gain;

        if let Some(ref mut radio) = self.radio {
            radio.set_vga_gain(gain)?;
        }

        self.status_message = Some(format!("VGA gain set to {} dB", gain));
        Ok(())
    }

    /// Toggle amplifier
    fn toggle_amp(&mut self) -> Result<()> {
        self.set_amp(!self.amp_enabled)
    }

    /// Set amplifier state
    fn set_amp(&mut self, enabled: bool) -> Result<()> {
        self.amp_enabled = enabled;

        if let Some(ref mut radio) = self.radio {
            radio.set_amp_enable(enabled)?;
        }

        self.status_message = Some(format!("Amp {}", if enabled { "enabled" } else { "disabled" }));
        Ok(())
    }

    /// Transmit a command for one or more captures. ID spec: "1", "1, 3, 5", "1-5", or mixed.
    fn transmit_command(&mut self, id_spec: Option<String>, command: ButtonCommand) -> Result<()> {
        let id_spec = match id_spec.as_deref() {
            Some(s) if !s.is_empty() => s,
            _ => {
                self.last_error = Some(format!("Usage: :{:?} <ID> (e.g. 1, 1,3,5, 1-5)", command).to_lowercase());
                return Ok(());
            }
        };
        let ids = match Self::parse_id_spec(id_spec) {
            Ok(ids) => ids,
            Err(e) => {
                self.last_error = Some(e);
                return Ok(());
            }
        };
        for id in ids {
            self.transmit_one_command(id, command)?;
        }
        Ok(())
    }

    /// Transmit a command for a single capture by ID.
    fn transmit_one_command(&mut self, id: u32, command: ButtonCommand) -> Result<()> {
        use crate::protocols::DecodedSignal;

        if let Some(ref radio) = self.radio {
            if !radio.supports_tx() {
                self.last_error = Some("Transmit not available – RTL-SDR is receive-only".to_string());
                return Ok(());
            }
        } else {
            self.last_error = Some("No radio device connected".to_string());
            return Ok(());
        }

        let capture = match self.captures.iter().find(|c| c.id == id) {
            Some(c) => c.clone(),
            None => {
                self.last_error = Some(format!("Capture {} not found", id));
                return Ok(());
            }
        };

        if capture.protocol.is_none() {
            self.last_error = Some("Cannot transmit: unknown protocol".to_string());
            return Ok(());
        }

        let protocol_name = capture.protocol.as_ref().unwrap();
        let protocol = match self.protocols.get(protocol_name) {
            Some(p) => p,
            None => {
                self.last_error = Some(format!("Protocol {} not supported for encoding", protocol_name));
                return Ok(());
            }
        };

        if !protocol.supports_encoding() {
            self.last_error = Some(format!("Protocol {} does not support encoding", protocol_name));
            return Ok(());
        }

        // Create a DecodedSignal from the capture (with its display name: KeeLoq encodes with the
        // manufacturer it names, as Flipper-ARF does with a key file's `Manufacture`)
        let decoded = DecodedSignal {
            serial: capture.serial,
            button: capture.button,
            counter: capture.counter,
            crc_valid: capture.crc_valid,
            data: capture.data,
            data_count_bit: capture.data_count_bit,
            encoder_capable: true,
            extra: capture.data_extra,
            protocol_display_name: capture.protocol.clone(),
        };

        // Generate the signal with the new button
        let button_code = command.code();
        let signal = match protocol.encode(&decoded, button_code) {
            Some(s) => s,
            None => {
                self.last_error = Some("Failed to encode signal".to_string());
                return Ok(());
            }
        };

        let (preset, modulation) = capture.tx_preset();
        self.queue_transmit(signal, capture.frequency, modulation);
        self.status_message = Some(format!(
            "Transmitted {:?} for capture {} ({}, {})",
            command,
            id,
            modulation,
            preset.label()
        ));
        Ok(())
    }

    /// Replay a capture by re-transmitting its raw level/duration pairs (no re-encoding).
    pub fn replay_capture(&mut self, id: u32) -> Result<()> {
        if let Some(ref radio) = self.radio {
            if !radio.supports_tx() {
                self.last_error = Some("Transmit not available – RTL-SDR is receive-only".to_string());
                return Ok(());
            }
        } else {
            self.last_error = Some("No radio device connected".to_string());
            return Ok(());
        }

        let capture = match self.captures.iter().find(|c| c.id == id) {
            Some(c) => c,
            None => {
                self.last_error = Some(format!("Capture {} not found", id));
                return Ok(());
            }
        };

        if capture.raw_pairs.is_empty() {
            self.last_error = Some("No raw signal to replay (capture has no level/duration data)".to_string());
            return Ok(());
        }

        let signal: Vec<LevelDuration> = capture
            .raw_pairs
            .iter()
            .map(|p| LevelDuration::new(p.level, p.duration_us))
            .collect();
        let pair_count = signal.len();
        let frequency = capture.frequency;
        let (preset, modulation) = capture.tx_preset();

        self.queue_transmit(signal, frequency, modulation);
        self.status_message = Some(format!(
            "Replayed capture {} ({} pairs, {}, {})",
            id,
            pair_count,
            modulation,
            preset.label()
        ));
        Ok(())
    }

    /// Transmit the next KeeLoq rolling code for a barrier/alarm capture (same button, counter+1).
    pub fn transmit_next_code(&mut self, id: u32) -> Result<()> {
        self.transmit_button(id, None)
    }

    /// Transmit the next code of a capture with one of its protocol's native buttons (`None`: the
    /// captured button): the protocol's encoder with that button (counter + 1 for rolling codes).
    pub fn transmit_button(&mut self, id: u32, native_button: Option<u8>) -> Result<()> {
        use crate::protocols::DecodedSignal;

        if let Some(ref radio) = self.radio {
            if !radio.supports_tx() {
                self.last_error = Some("Transmit not available – RTL-SDR is receive-only".to_string());
                return Ok(());
            }
        } else {
            self.last_error = Some("No radio device connected".to_string());
            return Ok(());
        }

        let capture = match self.captures.iter().find(|c| c.id == id) {
            Some(c) => c.clone(),
            None => {
                self.last_error = Some(format!("Capture {} not found", id));
                return Ok(());
            }
        };

        if capture.protocol.is_none() {
            self.last_error = Some("Cannot transmit: unknown protocol".to_string());
            return Ok(());
        }

        let protocol_name = capture.protocol.as_ref().unwrap();
        let protocol = match self.protocols.get(protocol_name) {
            Some(p) => p,
            None => {
                self.last_error = Some(format!("Protocol {} not supported for encoding", protocol_name));
                return Ok(());
            }
        };

        if !protocol.supports_encoding() {
            self.last_error = Some("Protocol does not support encoding".to_string());
            return Ok(());
        }

        let button = native_button.or(capture.button).unwrap_or(0);
        let decoded = DecodedSignal {
            serial: capture.serial,
            button: capture.button,
            counter: capture.counter,
            crc_valid: capture.crc_valid,
            data: capture.data,
            data_count_bit: capture.data_count_bit,
            encoder_capable: true,
            extra: capture.data_extra,
            protocol_display_name: capture.protocol.clone(),
        };

        let signal = match protocol.encode(&decoded, button) {
            Some(s) => s,
            None => {
                self.last_error = Some("Failed to encode next code".to_string());
                return Ok(());
            }
        };

        let (preset, modulation) = capture.tx_preset();
        self.queue_transmit(signal, capture.frequency, modulation);
        let what = if native_button.is_some() { "Sent" } else { "Sent next code," };
        self.status_message = Some(format!(
            "{} {} (0x{:02X}) for capture {} ({}, {})",
            what,
            crate::protocols::gate_buttons::button_label(protocol_name, button),
            button,
            id,
            modulation,
            preset.label()
        ));
        Ok(())
    }

    /// Delete the currently selected capture (if any). No-op if none selected or list empty.
    pub fn delete_selected_capture(&mut self) -> Result<()> {
        let id = match self.selected_capture {
            Some(idx) if idx < self.captures.len() => self.captures[idx].id,
            _ => return Ok(()),
        };
        self.delete_capture(&id.to_string())
    }

    /// Delete a capture by ID (the saved session follows on the next autosave)
    fn delete_capture(&mut self, id_str: &str) -> Result<()> {
        let id: u32 = match id_str.parse() {
            Ok(i) => i,
            Err(_) => {
                self.last_error = Some("Invalid capture ID".to_string());
                return Ok(());
            }
        };

        let idx = match self.captures.iter().position(|c| c.id == id) {
            Some(i) => i,
            None => {
                self.last_error = Some(format!("Capture {} not found", id));
                return Ok(());
            }
        };

        self.captures.remove(idx);

        // Adjust selection
        if let Some(sel) = self.selected_capture {
            if sel >= self.captures.len() && !self.captures.is_empty() {
                self.selected_capture = Some(self.captures.len() - 1);
            } else if self.captures.is_empty() {
                self.selected_capture = None;
            }
        }

        self.status_message = Some(format!("Deleted capture {}", id));
        Ok(())
    }

    /// Delete all captures (the saved session follows on the next autosave; `:clear` also deletes
    /// the session file and its backup)
    fn delete_all_captures(&mut self) -> Result<()> {
        let count = self.captures.len();
        
        if count == 0 {
            self.status_message = Some("No captures to delete".to_string());
            return Ok(());
        }

        // Clear the list
        self.captures.clear();
        self.selected_capture = None;
        self.scroll_offset = 0;

        self.status_message = Some(format!("Deleted all {} captures", count));
        Ok(())
    }

    /// True if a capture with the same protocol, data, serial, and button already exists.
    /// Unknown signals (no protocol) are never treated as duplicates so they can be kept for research.
    fn capture_duplicate_of_existing(&self, capture: &Capture) -> bool {
        if capture.protocol.is_none() {
            return false;
        }
        self.captures.iter().any(|c| {
            c.protocol == capture.protocol
                && c.data == capture.data
                && c.serial == capture.serial
                && c.button == capture.button
        })
    }

    /// Process pending radio events
    pub fn process_radio_events(&mut self) -> Result<()> {
        self.poll_psa_key_recovery();
        self.poll_keeloq_key_recovery();
        if let Some(ref rssi_arc) = self.rssi_source {
            self.rssi = f32::from_bits(rssi_arc.load(Ordering::Relaxed));
        }
        while let Ok(event) = self.radio_event_rx.try_recv() {
            match event {
                RadioEvent::SignalCaptured(mut capture) => {
                    // Convert stored pairs to the format protocols expect
                    let pairs: Vec<crate::radio::LevelDuration> = capture.raw_pairs
                        .iter()
                        .map(|p| crate::radio::LevelDuration::new(p.level, p.duration_us))
                        .collect();

                    // Try to decode with registered protocols (those flagged for the receive path's
                    // AM/FM first, then all)
                    if let Some((protocol_name, decoded)) =
                        self.protocols.process_signal_path(&pairs, capture.frequency, capture.received_rf)
                    {
                        capture.protocol = Some(protocol_name);
                        capture.serial = decoded.serial;
                        capture.button = decoded.button;
                        capture.counter = decoded.counter;
                        capture.crc_valid = decoded.crc_valid;
                        capture.data = decoded.data;
                        capture.data_count_bit = decoded.data_count_bit;
                        capture.data_extra = decoded.extra;
                        capture.status = if decoded.encoder_capable {
                            crate::capture::CaptureStatus::EncoderCapable
                        } else {
                            crate::capture::CaptureStatus::Decoded
                        };
                    }

                    // When research_mode is off, only add successfully decoded signals.
                    let show = self.storage.config.research_mode || capture.protocol.is_some();
                    if show {
                        // Same IQ is fed to AM and FM demodulators; both can emit for one keypress.
                        // Skip if we already have this exact signal (protocol + data + serial + button).
                        if self.capture_duplicate_of_existing(&capture) {
                            self.status_message = Some("Duplicate signal ignored".to_string());
                        } else {
                            capture.id = self.next_capture_id;
                            self.next_capture_id += 1;
                            // Autosaved to the session file on the next tick (app/persistence.rs).
                            self.captures.push(capture);

                            // Auto-select and scroll to new capture
                            let new_idx = self.captures.len() - 1;
                            self.selected_capture = Some(new_idx);
                            self.ensure_selection_visible();

                            self.status_message = Some("New signal captured".to_string());
                        }
                    }
                    // When research_mode is off and decode failed, the signal is dropped (not shown).
                }
                RadioEvent::Error(e) => {
                    self.last_error = Some(e);
                }
                RadioEvent::StateChanged(state) => {
                    self.radio_state = state;
                }
            }
        }
        // Saved-fob matching of new captures + session autosave (app/persistence.rs).
        self.poll_app_extensions();
        Ok(())
    }

    // -- Signal Action Menu helpers --

    /// Signal actions shown in the menu. With HackRF: Replay always; Lock/Unlock/Trunk/Panic only when
    /// the selected capture is encoder-capable and not a barrier/gate/garage or alarm (KeeLoq barrier
    /// and alarm protocols get Replay + export + delete only). Without TX (e.g. RTL-SDR): only export and delete.
    pub fn available_signal_actions(&self) -> Vec<SignalAction> {
        let mut actions = self.signal_actions_base();
        self.adjust_signal_actions(&mut actions);
        self.add_send_button_action(&mut actions);
        if let Some(action) = self.key_recovery_action() {
            actions.insert(0, action);
        }
        actions
    }

    /// "Recover key" for an undecrypted PSA (TEA) capture or a KeeLoq frame no key decrypts, or
    /// "Cancel key recovery" while one runs.
    fn key_recovery_action(&self) -> Option<SignalAction> {
        let c = self.selected_capture.and_then(|idx| self.captures.get(idx))?;
        if let Some(job) = &self.psa_recover_job {
            return (job.capture_id == c.id).then_some(SignalAction::CancelKeyRecovery);
        }
        if let Some(job) = &self.keeloq_bf_job {
            return (job.capture_id == c.id).then_some(SignalAction::CancelKeyRecovery);
        }
        let undecrypted_psa =
            c.protocol_name() == "PSA" && c.serial.is_none() && c.data_extra.is_some();
        if undecrypted_psa {
            return Some(SignalAction::RecoverKey);
        }
        Self::keeloq_bf_offered(c).then_some(SignalAction::RecoverKeeloqKey)
    }

    fn signal_actions_base(&self) -> Vec<SignalAction> {
        let has_tx = self.radio.as_ref().map_or(false, |r| r.supports_tx());
        let selected = self
            .selected_capture
            .and_then(|idx| self.captures.get(idx));
        let encoder_capable = selected
            .map_or(false, |c| c.status == crate::capture::CaptureStatus::EncoderCapable);
        // Gate/garage/barrier remotes (and KeeLoq barrier/alarm manufacturers) get the replay /
        // next-code menu instead of the car commands.
        let is_non_car_keeloq = selected.map_or(false, |c| {
            is_keeloq_non_car(c.protocol_name())
                || crate::protocols::protocol_info(c.protocol_name())
                    .is_some_and(|i| i.category == crate::protocols::info::Category::Gate)
        });

        if !has_tx {
            return SignalAction::ALL
                .iter()
                .filter(|a| {
                    matches!(
                        a,
                        SignalAction::ExportFob
                            | SignalAction::ExportFlipper
                            | SignalAction::TimingAnalysis
                            | SignalAction::Delete
                    )
                })
                .copied()
                .collect();
        }

        // Barrier/gate/garage or alarm: Replay, Send next code (encoder next rolling code), export + delete
        if encoder_capable && is_non_car_keeloq {
            return vec![
                SignalAction::Replay,
                SignalAction::SendNextCode,
                SignalAction::ExportFob,
                SignalAction::ExportFlipper,
                SignalAction::TimingAnalysis,
                SignalAction::Delete,
            ];
        }

        if encoder_capable {
            SignalAction::ALL.to_vec()
        } else {
            // Unknown or decoded-only: only Replay + export + delete (no TX Lock/Unlock/Trunk/Panic)
            SignalAction::ALL
                .iter()
                .filter(|a| {
                    !matches!(
                        a,
                        SignalAction::Lock
                            | SignalAction::Unlock
                            | SignalAction::Trunk
                            | SignalAction::Panic
                    )
                })
                .copied()
                .collect()
        }
    }

    /// Execute the currently selected signal action
    pub fn execute_signal_action(&mut self) -> Result<()> {
        let actions = self.available_signal_actions();
        let idx = self
            .signal_menu_index
            .min(actions.len().saturating_sub(1));
        let action = actions[idx];
        let capture_id = match self.selected_capture {
            Some(idx) if idx < self.captures.len() => self.captures[idx].id,
            _ => {
                self.last_error = Some("No capture selected".to_string());
                return Ok(());
            }
        };

        match action {
            SignalAction::Replay => {
                self.replay_capture(capture_id)?;
            }
            SignalAction::SendNextCode => {
                self.transmit_next_code(capture_id)?;
            }
            SignalAction::SendButton => {
                self.open_send_button_menu(capture_id);
            }
            SignalAction::Lock => {
                self.transmit_command(Some(capture_id.to_string()), ButtonCommand::Lock)?;
            }
            SignalAction::Unlock => {
                self.transmit_command(Some(capture_id.to_string()), ButtonCommand::Unlock)?;
            }
            SignalAction::Trunk => {
                self.transmit_command(Some(capture_id.to_string()), ButtonCommand::Trunk)?;
            }
            SignalAction::Panic => {
                self.transmit_command(Some(capture_id.to_string()), ButtonCommand::Panic)?;
            }
            SignalAction::ExportFob => {
                self.export_fob(capture_id)?;
            }
            SignalAction::ExportFlipper => {
                self.export_flipper(capture_id)?;
            }
            SignalAction::ExportFlipperKey => {
                self.export_flipper_key(capture_id)?;
            }
            SignalAction::TimingAnalysis => {
                self.open_timing_analysis(capture_id);
            }
            SignalAction::Delete => {
                let id_str = capture_id.to_string();
                self.delete_capture(&id_str)?;
            }
            SignalAction::RecoverKey => {
                self.start_psa_key_recovery(capture_id);
            }
            SignalAction::RecoverKeeloqKey => {
                self.start_keeloq_key_recovery(capture_id, None);
            }
            SignalAction::CancelKeyRecovery => {
                if let Some(job) = &self.psa_recover_job {
                    job.ctl.cancel.store(true, Ordering::Relaxed);
                    self.status_message = Some("Cancelling key recovery…".to_string());
                } else {
                    self.cancel_keeloq_key_recovery();
                }
            }
        }
        Ok(())
    }

    /// Start ProtoPirate's PSA TEA key brute force for a capture on a background thread.
    fn start_psa_key_recovery(&mut self, id: u32) {
        if self.psa_recover_job.is_some() {
            self.last_error = Some("A key recovery is already running".to_string());
            return;
        }
        let Some(capture) = self.captures.iter().find(|c| c.id == id) else {
            self.last_error = Some(format!("Capture {} not found", id));
            return;
        };
        let Some(extra) = capture.data_extra else {
            self.last_error =
                Some("Capture has no key2; re-import it from its raw pairs".to_string());
            return;
        };
        let data = capture.data;
        let ctl = Arc::new(PsaBruteForceControl::default());
        let worker = Arc::clone(&ctl);
        let handle = std::thread::spawn(move || psa_recover_key(data, extra, &worker));
        self.psa_recover_job = Some(PsaRecoverJob { capture_id: id, ctl, handle });
        self.status_message = Some(format!("Recovering PSA key for capture {}… 0%", id));
    }

    /// Report progress of / collect the result of a running PSA key recovery.
    fn poll_psa_key_recovery(&mut self) {
        let Some(job) = &self.psa_recover_job else {
            return;
        };
        if !job.handle.is_finished() {
            let done = u64::from(job.ctl.progress.load(Ordering::Relaxed));
            let pct = (done * 100 / u64::from(PSA_BRUTE_FORCE_TOTAL)).min(99);
            self.status_message =
                Some(format!("Recovering PSA key for capture {}… {}%", job.capture_id, pct));
            return;
        }
        let Some(job) = self.psa_recover_job.take() else {
            return;
        };
        let id = job.capture_id;
        match job.handle.join() {
            Ok(PsaBruteForceOutcome::Found(hit)) => {
                if let Some(c) = self.captures.iter_mut().find(|c| c.id == id) {
                    c.serial = Some(hit.fields.serial);
                    c.button = Some(hit.fields.button);
                    // PSA counters are up to 28 bits; the capture keeps the low 16 like the decoder.
                    c.counter = Some(hit.fields.counter as u16);
                    c.crc_valid = true;
                    c.status = crate::capture::CaptureStatus::EncoderCapable;
                    self.status_message = Some(format!(
                        "PSA key recovered for capture {}: serial {:06X}, counter {:X}",
                        id, hit.fields.serial, hit.fields.counter
                    ));
                }
            }
            Ok(PsaBruteForceOutcome::NotFound) => {
                self.last_error = Some(format!("No PSA key found for capture {}", id));
            }
            Ok(PsaBruteForceOutcome::Cancelled) => {
                self.status_message = Some(format!("Key recovery for capture {} cancelled", id));
            }
            Err(_) => {
                self.last_error = Some("Key recovery thread failed".to_string());
            }
        }
    }

    /// Generate a default export filename (without extension) for a capture
    /// Path relative to the import directory for display; falls back to full path if not under import_dir.
    fn path_relative_to_import(path: &std::path::Path, import_dir: &std::path::Path) -> String {
        path.strip_prefix(import_dir)
            .map(|p| p.to_string_lossy().to_string())
            .unwrap_or_else(|_| path.to_string_lossy().to_string())
    }

    /// Default export filename for .fob: Year_Make_Model_Region_Command (same format for all captures).
    /// Uses capture metadata when set; fallbacks: make from protocol for known, command from button_name(), else "Unknown".
    fn default_export_filename(capture: &Capture) -> String {
        let year = capture
            .year
            .as_deref()
            .unwrap_or("Unknown")
            .trim()
            .replace(' ', "_");
        let make = capture
            .make
            .as_deref()
            .filter(|s| !s.trim().is_empty())
            .map(|s| s.trim().replace(' ', "_"))
            .unwrap_or_else(|| {
                if capture.protocol_name().eq_ignore_ascii_case("unknown") {
                    "Unknown".to_string()
                } else {
                    capture.protocol_make().trim().replace(' ', "_")
                }
            });
        let model = capture
            .model
            .as_deref()
            .unwrap_or("Unknown")
            .trim()
            .replace(' ', "_");
        let region = capture
            .region
            .as_deref()
            .unwrap_or("Unknown")
            .trim()
            .replace(' ', "_");
        let cmd_str = capture
            .command
            .as_deref()
            .unwrap_or_else(|| capture.button_name())
            .trim();
        let command = if cmd_str.is_empty() || cmd_str == "-" {
            "Unknown".to_string()
        } else {
            cmd_str.replace(' ', "_")
        };
        // Makes such as "Kia/Hyundai" must not turn the name into a path (the export would fail).
        format!("{}_{}_{}_{}_{}", year, make, model, region, command).replace(['/', '\\'], "-")
    }

    /// Start .fob export by entering filename input mode
    pub fn export_fob(&mut self, id: u32) -> Result<()> {
        if !self.captures.iter().any(|c| c.id == id) {
            self.last_error = Some(format!("Capture {} not found", id));
            return Ok(());
        }

        // Pre-fill filename: Year_Make_Model_Region_Command_8HEX for all .fob exports
        let capture = self.captures.iter().find(|c| c.id == id);
        let default_name = capture
            .map(|c| Self::default_export_filename(c))
            .unwrap_or_else(|| format!("capture_{}", id));
        let suffix_nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;
        let suffix = (suffix_nanos.wrapping_add(id as u64 * 2654435761) % 0x100_000_000) as u32;
        self.export_filename = format!("{}_{:08X}", default_name, suffix);

        // Pre-fill metadata from capture if set, otherwise make from protocol
        let make = capture
            .and_then(|c| c.make.as_ref().map(String::clone))
            .filter(|s| !s.is_empty())
            .unwrap_or_else(|| {
                capture
                    .map(|c| c.protocol_make().to_string())
                    .unwrap_or_default()
            });
        self.export_capture_id = Some(id);
        self.export_format = Some(ExportFormat::Fob);
        self.fob_meta_year = capture
            .and_then(|c| c.year.as_ref())
            .map(String::clone)
            .unwrap_or_default();
        self.fob_meta_make = make;
        self.fob_meta_model = capture
            .and_then(|c| c.model.as_ref())
            .map(String::clone)
            .unwrap_or_default();
        self.fob_meta_region = capture
            .and_then(|c| c.region.as_ref())
            .map(String::clone)
            .unwrap_or_default();
        self.fob_meta_command = capture
            .and_then(|c| c.command.clone())
            .unwrap_or_else(|| {
                let b = capture.map(|c| c.button_name().to_string()).unwrap_or_default();
                if b.is_empty() || b == "-" {
                    String::new()
                } else {
                    b
                }
            });
        self.fob_meta_notes = String::new();
        self.input_mode = InputMode::ExportFilename;
        Ok(())
    }

    /// Complete the .fob export with collected metadata
    pub fn complete_fob_export(&mut self) -> Result<()> {
        let id = match self.export_capture_id {
            Some(id) => id,
            None => {
                self.last_error = Some("No capture selected for export".to_string());
                return Ok(());
            }
        };

        let capture = match self.captures.iter().find(|c| c.id == id) {
            Some(c) => c.clone(),
            None => {
                self.last_error = Some(format!("Capture {} not found", id));
                return Ok(());
            }
        };

        let export_dir = self.storage.export_dir().clone();
        if !export_dir.exists() {
            std::fs::create_dir_all(&export_dir)?;
        }

        let metadata = crate::export::fob::FobMetadata {
            year: self.fob_meta_year.parse::<u32>().ok(),
            make: self.fob_meta_make.clone(),
            model: self.fob_meta_model.clone(),
            region: self.fob_meta_region.clone(),
            command: self.fob_meta_command.clone(),
            notes: self.fob_meta_notes.clone(),
        };

        // All .fob exports use Year_Make_Model_Region_Command_8HEX; append 8-hex if user removed it
        let already_has_8hex = self.export_filename.len() >= 9
            && self.export_filename.as_bytes()[self.export_filename.len() - 9] == b'_'
            && self.export_filename[self.export_filename.len() - 8..]
                .chars()
                .all(|c| c.is_ascii_hexdigit());
        let filename = if already_has_8hex {
            format!("{}.fob", self.export_filename.trim())
        } else {
            let base = self.export_filename.trim();
            let suffix_nanos = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64;
            let suffix = (suffix_nanos.wrapping_add(id as u64 * 2654435761) % 0x100_000_000) as u32;
            let hex_suffix = format!("{:08X}", suffix);
            if base.is_empty() {
                format!("unknown_{}_{}.fob", id, hex_suffix)
            } else {
                format!("{}_{}.fob", base, hex_suffix)
            }
        };
        let path = export_dir.join(&filename);

        crate::export::fob::export_fob(
            &capture,
            &path,
            self.storage.config.include_raw_pairs,
            Some(&metadata),
        )?;
        // Saved-fob matching picks the new file up (app/fob_matching.rs).
        self.saved_fobs_rescan();

        self.export_capture_id = None;
        self.export_format = None;
        self.status_message = Some(format!("Exported to {}", filename));
        Ok(())
    }

    /// Open the capture metadata form for the given capture (Year/Make/Model/Region). Called when user presses 'i'.
    pub fn open_capture_meta_form(&mut self, capture_id: u32) {
        let capture = self.captures.iter().find(|c| c.id == capture_id);
        self.capture_meta_year = capture
            .and_then(|c| c.year.as_ref())
            .map(|s| s.to_string())
            .unwrap_or_default();
        self.capture_meta_make = capture
            .and_then(|c| c.make.as_ref())
            .map(|s| s.to_string())
            .unwrap_or_default();
        self.capture_meta_model = capture
            .and_then(|c| c.model.as_ref())
            .map(|s| s.to_string())
            .unwrap_or_default();
        self.capture_meta_region = capture
            .and_then(|c| c.region.as_ref())
            .map(|s| s.to_string())
            .unwrap_or_default();
        self.capture_meta_command = capture
            .and_then(|c| c.command.clone())
            .unwrap_or_else(|| {
                let b = capture.map(|c| c.button_name().to_string()).unwrap_or_default();
                if b.is_empty() || b == "-" {
                    String::new()
                } else {
                    b
                }
            });
        self.capture_meta_capture_id = Some(capture_id);
        self.input_mode = InputMode::CaptureMetaYear;
    }

    /// Save capture metadata from the form into the selected capture and return to Normal.
    pub fn save_capture_meta(&mut self) {
        let id = match self.capture_meta_capture_id {
            Some(id) => id,
            None => {
                self.input_mode = InputMode::Normal;
                self.capture_meta_capture_id = None;
                return;
            }
        };
        if let Some(capture) = self.captures.iter_mut().find(|c| c.id == id) {
            capture.year = Some(self.capture_meta_year.clone()).filter(|s| !s.is_empty());
            capture.make = Some(self.capture_meta_make.clone()).filter(|s| !s.is_empty());
            capture.model = Some(self.capture_meta_model.clone()).filter(|s| !s.is_empty());
            capture.region = Some(self.capture_meta_region.clone()).filter(|s| !s.is_empty());
            capture.command = Some(self.capture_meta_command.clone()).filter(|s| !s.is_empty());
        }
        self.input_mode = InputMode::Normal;
        self.capture_meta_capture_id = None;
    }

    /// Cancel capture metadata form without saving.
    pub fn cancel_capture_meta(&mut self) {
        self.input_mode = InputMode::Normal;
        self.capture_meta_capture_id = None;
    }

    /// Open the :load file browser starting at the config import directory.
    pub fn open_load_browser(&mut self) -> Result<()> {
        self.load_browser_cwd = self.storage.import_dir().clone();
        self.load_browser_selected = 0;
        self.refresh_load_browser_entries()?;
        self.input_mode = InputMode::LoadFileBrowser;
        Ok(())
    }

    /// Refresh the file list for the current load-browser directory.
    pub fn refresh_load_browser_entries(&mut self) -> Result<()> {
        let import_dir = self.storage.import_dir().clone();
        let mut entries: Vec<(String, PathBuf, bool)> = Vec::new();

        if self.load_browser_cwd != import_dir {
            if let Some(parent) = self.load_browser_cwd.parent() {
                entries.push(("..".to_string(), parent.to_path_buf(), true));
            }
        }

        let dir_entries = match std::fs::read_dir(&self.load_browser_cwd) {
            Ok(d) => d,
            Err(e) => {
                self.last_error = Some(format!("Cannot read directory: {}", e));
                self.load_browser_entries = entries;
                return Ok(());
            }
        };

        let mut dirs: Vec<(String, PathBuf)> = Vec::new();
        let mut files: Vec<(String, PathBuf)> = Vec::new();
        for e in dir_entries.flatten() {
            let path = e.path();
            let name = e
                .file_name()
                .to_string_lossy()
                .to_string();
            if path.is_dir() {
                dirs.push((name, path));
            } else if path.is_file() {
                let ext = path.extension().map(|e| e.to_string_lossy().to_lowercase());
                if ext.as_deref() == Some("fob") || ext.as_deref() == Some("sub") || crate::radio::iq_file::is_iq_path(&path) {
                    files.push((name, path));
                }
            }
        }

        dirs.sort_by(|a, b| a.0.to_lowercase().cmp(&b.0.to_lowercase()));
        files.sort_by(|a, b| a.0.to_lowercase().cmp(&b.0.to_lowercase()));

        for (name, path) in dirs {
            entries.push((name, path, true));
        }
        for (name, path) in files {
            entries.push((name, path, false));
        }

        let len = entries.len();
        self.load_browser_entries = entries;
        self.load_browser_selected = self.load_browser_selected.min(len.saturating_sub(1));
        const VISIBLE: usize = 16;
        if self.load_browser_selected < self.load_browser_scroll {
            self.load_browser_scroll = self.load_browser_selected;
        }
        if self.load_browser_selected >= self.load_browser_scroll + VISIBLE {
            self.load_browser_scroll = self.load_browser_selected.saturating_sub(VISIBLE - 1);
        }
        self.load_browser_scroll = self.load_browser_scroll.min(len.saturating_sub(1));
        Ok(())
    }

    /// Handle Enter in the load file browser: open dir or import file.
    pub fn load_browser_enter(&mut self) -> Result<()> {
        let Some((_name, path, is_dir)) = self.load_browser_entries.get(self.load_browser_selected)
        else {
            return Ok(());
        };
        let path = path.clone();
        let is_dir = *is_dir;
        if is_dir {
            self.load_browser_cwd = path;
            self.load_browser_selected = 0;
            self.refresh_load_browser_entries()?;
        } else if crate::radio::iq_file::is_iq_path(&path) {
            // Raw IQ recording: run it through the receive pipeline in the background.
            self.input_mode = InputMode::Normal;
            self.start_iq_file(path, None, None);
        } else {
            let name = path.file_name().unwrap_or_default().to_string_lossy().to_string();
            self.pending_fob_files = vec![path];
            self.import_fob_files()?;
            self.input_mode = InputMode::Normal;
            self.status_message = Some(format!("Imported {}", name));
        }
        Ok(())
    }

    /// `:load <file|dir>`: import a `.sub` / `.fob` file, decode an IQ recording, or open the file
    /// browser in a directory (`~/` = home).
    pub fn load_path(&mut self, arg: &str) -> Result<()> {
        let path = match arg.strip_prefix("~/") {
            Some(rest) => dirs::home_dir().map_or_else(|| PathBuf::from(arg), |home| home.join(rest)),
            None => PathBuf::from(arg),
        };
        if path.is_dir() {
            self.load_browser_cwd = path;
            self.load_browser_selected = 0;
            self.refresh_load_browser_entries()?;
            self.input_mode = InputMode::LoadFileBrowser;
        } else if !path.exists() {
            self.last_error = Some(format!("No such file: {}", path.display()));
        } else if crate::radio::iq_file::is_iq_path(&path) {
            self.start_iq_file(path, None, None);
        } else {
            let name = path.file_name().unwrap_or_default().to_string_lossy().to_string();
            self.pending_fob_files = vec![path];
            self.import_fob_files()?;
            if self.last_error.is_none() {
                self.status_message = Some(format!("Imported {}", name));
            }
        }
        Ok(())
    }

    /// Close the load file browser without importing.
    pub fn close_load_browser(&mut self) {
        self.input_mode = InputMode::Normal;
    }

    /// Import pending .fob and .sub files into captures list.
    /// .sub files are decoded with registered protocols after load (no metadata in file).
    /// When research_mode is off, only decoded captures are added (same as live capture).
    pub fn import_fob_files(&mut self) -> Result<()> {
        let files = std::mem::take(&mut self.pending_fob_files);
        let mut imported = 0;
        let research_mode = self.storage.config.research_mode;

        for path in &files {
            let is_sub = path.extension().map_or(false, |e| e == "sub");

            if is_sub {
                // A key file (Protocol/Key, no RAW_Data): rebuilt by its protocol (app/key_files.rs).
                if let Some(added) = self.import_key_file(path) {
                    imported += added;
                    continue;
                }
                match crate::export::flipper::import_sub(path) {
                    Ok(crate::export::flipper::SubFile { frequency, preset: sub_preset, pairs: raw_pairs }) => {
                        let pairs: Vec<crate::radio::LevelDuration> = raw_pairs
                            .iter()
                            .map(|p| crate::radio::LevelDuration::new(p.level, p.duration_us))
                            .collect();
                        // Gated by the file's preset modulation (AM/FM), as Flipper-ARF does.
                        let sub_rf = sub_preset.as_ref().and_then(|p| p.rf());
                        let decoded_list =
                            self.protocols.process_signal_stream_rf(&pairs, frequency, sub_rf);
                        // Deduplicate: same signal can decode at multiple stream positions (e.g. Ford V0 across bursts)
                        let mut seen: std::collections::HashSet<(String, u64, Option<u32>, Option<u8>)> =
                            std::collections::HashSet::new();
                        let mut any_decoded_added = false;
                        for (protocol_name, decoded, segment_pairs) in decoded_list {
                            let key = (
                                protocol_name.clone(),
                                decoded.data,
                                decoded.serial,
                                decoded.button,
                            );
                            if seen.contains(&key) {
                                continue;
                            }
                            seen.insert(key);
                            let raw: Vec<crate::capture::StoredLevelDuration> = segment_pairs
                                .iter()
                                .map(|p| crate::capture::StoredLevelDuration {
                                    level: p.level,
                                    duration_us: p.duration_us,
                                })
                                .collect();
                            let mut capture = crate::capture::Capture::from_pairs_with_rf(
                                self.next_capture_id,
                                frequency,
                                raw,
                                None,
                            );
                            capture.rf_preset = sub_preset.clone();
                            capture.protocol = Some(protocol_name);
                            capture.serial = decoded.serial;
                            capture.button = decoded.button;
                            capture.counter = decoded.counter;
                            capture.crc_valid = decoded.crc_valid;
                            capture.data = decoded.data;
                            capture.data_count_bit = decoded.data_count_bit;
                            capture.data_extra = decoded.extra;
                            capture.status = if decoded.encoder_capable {
                                crate::capture::CaptureStatus::EncoderCapable
                            } else {
                                crate::capture::CaptureStatus::Decoded
                            };
                            if research_mode || capture.protocol.is_some() {
                                if !self.capture_duplicate_of_existing(&capture) {
                                    self.next_capture_id += 1;
                                    capture.source_file = Some(Self::path_relative_to_import(path, self.storage.import_dir()));
                                    self.captures.push(capture);
                                    imported += 1;
                                    any_decoded_added = true;
                                }
                            }
                        }
                        // When no protocol decoded the stream, add a single Unknown capture if research_mode (same as live capture).
                        if !any_decoded_added && research_mode && !raw_pairs.is_empty() {
                            let mut capture = crate::capture::Capture::from_pairs_with_rf(
                                self.next_capture_id,
                                frequency,
                                raw_pairs.clone(),
                                None,
                            );
                            capture.rf_preset = sub_preset.clone();
                            self.next_capture_id += 1;
                            capture.source_file = Some(Self::path_relative_to_import(path, self.storage.import_dir()));
                            self.captures.push(capture);
                            imported += 1;
                        }
                    }
                    Err(e) => tracing::warn!("Failed to import {:?}: {}", path, e),
                }
            } else {
                match crate::export::fob::import_fob(path, self.next_capture_id) {
                    Ok(mut capture) => {
                        self.next_capture_id += 1;
                        capture.source_file = Some(Self::path_relative_to_import(path, self.storage.import_dir()));
                        // Re-run the current decoders whenever raw pairs are present (same as .sub),
                        // so captures exported by older KAT versions pick up today's decoders:
                        // renamed/retired protocols (Land Rover V0 → Honda V2, Porsche Touareg →
                        // Porsche Cayenne), new encoders (Renault V1, Fiat V1, VAG …) and their
                        // `data_extra`. If nothing decodes, the stored fields are kept as-is.
                        if !capture.raw_pairs.is_empty() {
                            let pairs: Vec<crate::radio::LevelDuration> = capture
                                .raw_pairs
                                .iter()
                                .map(|p| crate::radio::LevelDuration::new(p.level, p.duration_us))
                                .collect();
                            // Received by KAT (path: preferred, then all) or imported (preset:
                            // strict, as Flipper-ARF).
                            let decoded = match capture.received_rf {
                                Some(path) => self.protocols.process_signal_path(&pairs, capture.frequency, Some(path)),
                                None => {
                                    let rf = capture.rf_preset.as_ref().and_then(|p| p.rf());
                                    self.protocols.process_signal_rf(&pairs, capture.frequency, rf)
                                }
                            };
                            if let Some((protocol_name, decoded)) = decoded {
                                capture.protocol = Some(protocol_name);
                                capture.serial = decoded.serial;
                                capture.button = decoded.button;
                                capture.counter = decoded.counter;
                                capture.crc_valid = decoded.crc_valid;
                                capture.data = decoded.data;
                                capture.data_count_bit = decoded.data_count_bit;
                                capture.data_extra = decoded.extra;
                                capture.status = if decoded.encoder_capable {
                                    crate::capture::CaptureStatus::EncoderCapable
                                } else {
                                    crate::capture::CaptureStatus::Decoded
                                };
                            }
                        }
                        self.check_encoder_capable(&mut capture);
                        if research_mode || capture.protocol.is_some() {
                            self.captures.push(capture);
                            imported += 1;
                        }
                    }
                    Err(e) => tracing::warn!("Failed to import {:?}: {}", path, e),
                }
            }
        }

        if imported > 0 {
            self.selected_capture = Some(0);
            self.status_message = Some(format!("Imported {} file(s)", imported));
        }

        Ok(())
    }

    /// Skip .fob import and start blank
    pub fn skip_fob_import(&mut self) {
        self.pending_fob_files.clear();
        self.status_message = Some("Starting with no imported signals".to_string());
    }

    /// Start .sub (Flipper) export by entering filename input mode
    pub fn export_flipper(&mut self, id: u32) -> Result<()> {
        if !self.captures.iter().any(|c| c.id == id) {
            self.last_error = Some(format!("Capture {} not found", id));
            return Ok(());
        }

        let default_name = self.captures.iter().find(|c| c.id == id)
            .map(|c| Self::default_export_filename(c))
            .unwrap_or_else(|| format!("capture_{}", id));

        self.export_capture_id = Some(id);
        self.export_filename = default_name;
        self.export_format = Some(ExportFormat::Flipper);
        self.input_mode = InputMode::ExportFilename;
        Ok(())
    }

    /// Complete Flipper .sub export (called after filename is confirmed)
    pub fn complete_flipper_export(&mut self) -> Result<()> {
        let id = match self.export_capture_id {
            Some(id) => id,
            None => {
                self.last_error = Some("No capture selected for export".to_string());
                return Ok(());
            }
        };

        let capture = match self.captures.iter().find(|c| c.id == id) {
            Some(c) => c.clone(),
            None => {
                self.last_error = Some(format!("Capture {} not found", id));
                return Ok(());
            }
        };

        let export_dir = self.storage.export_dir().clone();
        if !export_dir.exists() {
            std::fs::create_dir_all(&export_dir)?;
        }

        let filename = format!("{}.sub", self.export_filename);
        let path = export_dir.join(&filename);

        crate::export::flipper::export_flipper_sub(&capture, &path)?;
        self.export_capture_id = None;
        self.export_format = None;
        self.status_message = Some(format!("Exported to {}", filename));
        Ok(())
    }

    // -- Settings Menu helpers --

    /// Get the current value index for the active settings field
    pub fn current_settings_value_index(&self) -> usize {
        let field = SettingsField::ALL[self.settings_field_index];
        match field {
            SettingsField::Freq => {
                PRESET_FREQUENCIES.iter().position(|(f, _)| *f == self.frequency).unwrap_or(0)
            }
            SettingsField::Hop => {
                if self.rx.hop_enabled { 0 } else { 1 }
            }
            SettingsField::Lna => {
                LNA_STEPS.iter().position(|&g| g == self.lna_gain).unwrap_or(0)
            }
            SettingsField::Vga => {
                VGA_STEPS.iter().position(|&g| g == self.vga_gain).unwrap_or(0)
            }
            SettingsField::Amp => {
                if self.amp_enabled { 0 } else { 1 }
            }
            SettingsField::TxPower => self.tx_power_value_index(),
        }
    }

    /// Get the number of values for the active settings field
    pub fn settings_value_count(&self) -> usize {
        let field = SettingsField::ALL[self.settings_field_index];
        match field {
            SettingsField::Freq => PRESET_FREQUENCIES.len(),
            SettingsField::Hop => 2, // ON / OFF
            SettingsField::Lna => LNA_STEPS.len(),
            SettingsField::Vga => VGA_STEPS.len(),
            SettingsField::Amp => 2, // ON / OFF
            SettingsField::TxPower => TX_POWER_STEPS.len(),
        }
    }

    /// Apply the selected settings value
    pub fn apply_settings_value(&mut self) -> Result<()> {
        let field = SettingsField::ALL[self.settings_field_index];
        match field {
            SettingsField::Freq => {
                if self.settings_value_index < PRESET_FREQUENCIES.len() {
                    let (hz, _) = PRESET_FREQUENCIES[self.settings_value_index];
                    self.set_frequency(hz)?;
                }
            }
            SettingsField::Hop => {
                self.set_hopping(self.settings_value_index == 0);
            }
            SettingsField::Lna => {
                if self.settings_value_index < LNA_STEPS.len() {
                    self.set_lna_gain(LNA_STEPS[self.settings_value_index])?;
                }
            }
            SettingsField::Vga => {
                if self.settings_value_index < VGA_STEPS.len() {
                    self.set_vga_gain(VGA_STEPS[self.settings_value_index])?;
                }
            }
            SettingsField::Amp => {
                self.set_amp(self.settings_value_index == 0)?;
            }
            SettingsField::TxPower => self.apply_tx_power_value(self.settings_value_index),
        }
        Ok(())
    }

    /// Add a demo capture (for testing without HackRF)
    #[allow(dead_code)]
    pub fn add_demo_capture(&mut self) {
        let capture = Capture {
            id: self.next_capture_id,
            timestamp: chrono::Utc::now(),
            frequency: 433_920_000,
            protocol: Some("Ford V0".to_string()),
            serial: Some(0x1A2B3C4D),
            button: Some(0x01),
            counter: Some(1234),
            crc_valid: true,
            data: 0x5A2B3C4D00001234,
            data_count_bit: 64,
            data_extra: None,
            raw_pairs: vec![],
            status: crate::capture::CaptureStatus::EncoderCapable,
            received_rf: None,
            rf_preset: None,
            year: None,
            make: None,
            model: None,
            region: None,
            command: None,
            source_file: None,
        };
        self.next_capture_id += 1;
        self.captures.push(capture);
    }
}
