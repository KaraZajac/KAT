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
mod files;

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
    /// Help overlay (`?`, F1, `:help`), scrolled like the License overlay
    Help,
    /// "Quit? … is still running" (y/n): `q` while a key search, IQ file, recording or transmit
    /// queue runs (app/input.rs)
    ConfirmQuit,
    /// `:delete all` confirmation (y/n); `u` undoes it afterwards
    ConfirmDeleteAll,
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
mod hitag2_bf;
mod auto_bf;
mod input;
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
    /// Fiat V1 / Renault V1 / Fiat V2 frame without a key: qDoctor's HiTag2 Hell (app/hitag2_bf.rs).
    RecoverHitag2Key,
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
            SignalAction::RecoverHitag2Key => "Recover key (HiTag2)",
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
#[allow(clippy::large_enum_variant)] // one event per capture; boxing would touch every sender in src/radio
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

/// A running PSA key brute force (ProtoPirate's `psa_brute_force_run`, on a thread so the UI stays
/// responsive; ≈0.5 s worst case on a 16-core PC, longer on small machines): the "Recover key"
/// action, or `autobf` (app/auto_bf.rs).
struct PsaRecoverJob {
    /// The listed capture whose key is searched; None for an unchecked frame.
    capture_id: Option<u32>,
    /// A `PSA [unchecked]` frame, listed only if its key is found.
    unchecked: Option<Box<Capture>>,
    /// Started by `autobf`: reports quietly, and quitting does not wait for it.
    auto: bool,
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
    /// First row the captures table shows: kept by the renderer between frames (ratatui scrolls it
    /// just enough to keep the selection in view for the table's real height).
    pub captures_table_offset: std::cell::Cell<usize>,
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
    /// Largest useful `overlay_scroll` of the License / Credits / help overlay as the renderer
    /// last drew it (set every frame), so scrolling stops at the last line.
    pub overlay_scroll_max: std::cell::Cell<usize>,
    /// Rows of the captures list the last frame showed (set by the renderer): PgUp / PgDn step.
    pub list_page_rows: std::cell::Cell<usize>,
    /// Signal-menu target, delete undo, transmit progress (app/input.rs)
    input: input::InputState,

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
    /// Why no radio could be opened at startup, per device ("HackRF: not found · RTL-SDR: found
    /// but busy …"), for the no-device popup; None when a radio is in use.
    pub radio_open_error: Option<String>,

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
    /// Key searches waiting for the background job (`autobf`, app/auto_bf.rs).
    auto_bf: auto_bf::AutoBf,
    /// KeeLoq magic-serial key search running in the background.
    keeloq_bf_job: Option<keeloq_bf::KeeloqBfJob>,
    /// HiTag2 key search running in the background.
    hitag2_bf_job: Option<hitag2_bf::Hitag2BfJob>,
    /// Whether the signal menu offers "Recover key (HiTag2)" (app/hitag2_bf.rs): the menu asks on
    /// every frame it draws.
    hitag2_offer_memo: std::cell::Cell<Option<hitag2_bf::OfferMemo>>,

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
        // ── … and the HiTag2 keys saved per remote (Flipper-ARF's `hitag2_keys.txt`), if any ──
        let hitag2_keys_msg = crate::protocols::hitag2_keys::load(storage.config_dir());

        let mut protocols = ProtocolRegistry::new();
        protocols.set_auto_key_search(storage.config.autobf);
        let (radio_event_tx, radio_event_rx) = mpsc::channel();

        // Try HackRF first, then RTL-SDR; keep why each could not be opened (no-device popup).
        let mut open_errors: Vec<String> = Vec::new();
        let radio: Option<RadioDevice> = match HackRfController::new(radio_event_tx.clone()) {
            Ok(mut h) if h.is_available() => {
                tracing::info!("HackRF initialized successfully");
                let _ = h.set_lna_gain(storage.config.default_lna_gain);
                let _ = h.set_vga_gain(storage.config.default_vga_gain);
                let _ = h.set_amp_enable(storage.config.default_amp);
                Some(RadioDevice::HackRf(h))
            }
            hackrf => {
                open_errors.extend(hackrf.ok().and_then(|h| h.open_error().map(|e| format!("HackRF: {e}"))));
                match RtlSdrController::new(radio_event_tx.clone()) {
                    Ok(mut r) if r.is_available() => {
                        tracing::info!("RTL-SDR initialized (receive-only)");
                        let _ = r.set_lna_gain(storage.config.default_lna_gain);
                        let _ = r.set_vga_gain(storage.config.default_vga_gain);
                        Some(RadioDevice::RtlSdr(r))
                    }
                    rtl => {
                        open_errors.extend(rtl.ok().and_then(|r| r.open_error().map(|e| format!("RTL-SDR: {e}"))));
                        None
                    }
                }
            }
        };
        let radio_open_error = (radio.is_none() && !open_errors.is_empty()).then(|| open_errors.join(" · "));

        let device_detected = radio.as_ref().is_some_and(|r| r.is_available());

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
            captures_table_offset: std::cell::Cell::new(0),
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
            overlay_scroll_max: std::cell::Cell::new(0),
            list_page_rows: std::cell::Cell::new(10),
            input: input::InputState::default(),
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
            auto_bf: auto_bf::AutoBf::default(),
            keeloq_bf_job: None,
            hitag2_bf_job: None,
            hitag2_offer_memo: std::cell::Cell::new(None),
            load_browser_cwd: PathBuf::new(),
            load_browser_selected: 0,
            load_browser_scroll: 0,
            load_browser_entries: Vec::new(),
            session: persistence::SessionState::default(),
            saved_fobs: fob_matching::SavedFobState::default(),
            timing_view: None,
            radio_open_error,
            add_manually: add_manually::AddManuallyState::default(),
        };
        if let Some(why) = &app.radio_open_error {
            tracing::warn!("No radio: {why}");
        }
        // Saved-session restore prompt, session writer, saved-fob index (app/persistence.rs).
        app.init_app_extensions();
        for msg in [user_keystore_msg, hitag2_keys_msg].into_iter().flatten() {
            tracing::info!("{msg}");
            app.status_message.get_or_insert(msg);
        }
        // config.ini problems (logged by Storage, all kept in `storage.warnings`): the first one
        // in the status line, so a typo does not go unnoticed.
        if let Some(first) = app.storage.warnings.first() {
            let more = app.storage.warnings.len() - 1;
            let msg = if more == 0 { first.clone() } else { format!("{first} (+{more} more in kat.log)") };
            app.last_error.get_or_insert(msg);
        }
        Ok(app)
    }

    /// Display name of the active radio device, if any (e.g. "HackRF", "RTL-SDR (RX only)").
    pub fn radio_device_name(&self) -> Option<&'static str> {
        self.radio.as_ref().map(|r| r.display_name())
    }

    /// True if the active device supports transmit (HackRF only; RTL-SDR is receive-only).
    #[allow(dead_code)]
    pub fn can_transmit(&self) -> bool {
        self.radio.as_ref().is_some_and(|r| r.supports_tx())
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
                    self.status_message = Some(format!("Receiving on {} MHz", crate::radio::hopper::format_mhz(self.frequency)));
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

    /// Execute a command
    pub fn execute_command(&mut self, command: &str) -> Result<()> {
        let parts: Vec<&str> = command.split_whitespace().collect();
        if parts.is_empty() {
            return Ok(());
        }

        self.last_error = None;
        self.status_message = None;

        match parts[0] {
            "q" | "quit" => {
                // Asks first while a key search, IQ file, recording or transmit queue runs.
                self.quit_requested = self.request_quit();
            }
            "freq" => {
                // "433.92", "433.92M", "433.92 MHz", "433920000", "433920000Hz" (any case)
                match crate::radio::iq_file::parse_frequency_arg(&parts[1..].join(" ")) {
                    Some(hz) => self.set_frequency(hz)?,
                    None => {
                        self.last_error = Some("Usage: :freq <MHz> (e.g. 433.92; 300–928 MHz)".to_string());
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
            "help" | "h" => self.open_help(),
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
                // An ID list like :replay's ("2", "1-3, 7", "4 5"); `all` asks first (app/input.rs).
                let spec = parts[1..].join(" ");
                if spec.is_empty() {
                    self.last_error = Some("Usage: :delete <ID> (e.g. 4, 1-3, 1,5) or :delete all".to_string());
                } else if spec.eq_ignore_ascii_case("all") {
                    self.request_delete_all();
                } else {
                    self.delete_capture(&spec)?;
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
                self.transmit_id_list(id_spec, None)?;
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
                self.last_error = Some(format!("Unknown command: {} (:help lists commands)", parts[0]));
            }
        }

        Ok(())
    }

    /// Set the receive frequency
    fn set_frequency(&mut self, hz: u32) -> Result<()> {
        // Validate frequency range (common keyfob frequencies)
        if !(300_000_000..=928_000_000).contains(&hz) {
            self.last_error = Some(format!(
                "{} MHz is out of range: KAT receives 300–928 MHz (e.g. :freq 433.92)",
                crate::radio::hopper::format_mhz(hz)
            ));
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

        self.status_message = Some(format!("Frequency set to {} MHz", crate::radio::hopper::format_mhz(hz)));
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

    /// Transmit a command for one or more captures. ID spec: "1", "1, 3, 5", "1-5", or mixed
    /// (app/input.rs).
    fn transmit_command(&mut self, id_spec: Option<String>, command: ButtonCommand) -> Result<()> {
        let id_spec = match id_spec.as_deref() {
            Some(s) if !s.is_empty() => s,
            _ => {
                let name = format!("{:?}", command).to_lowercase();
                self.last_error = Some(format!("Usage: :{} <ID> (e.g. 1, 1,3,5, 1-5)", name));
                return Ok(());
            }
        };
        self.transmit_id_list(id_spec, Some(command))
    }

    /// Transmit a command for a single capture by ID.
    fn transmit_one_command(&mut self, id: u32, command: ButtonCommand) -> Result<()> {
        use crate::protocols::DecodedSignal;

        if !self.transmit_allowed() {
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
        // A rolling code's capture advances to the frame sent once it went out (app/tx.rs).
        let advance = self.tx_advance(id, protocol_name, &decoded, button_code, &signal);
        self.queue_transmit_advancing(signal, capture.frequency, modulation, advance);
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
        if !self.transmit_allowed() {
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
    /// captured button): the protocol's encoder with that button (counter + 1 for rolling codes,
    /// kept on the capture once sent, so the next press sends the code after).
    pub fn transmit_button(&mut self, id: u32, native_button: Option<u8>) -> Result<()> {
        use crate::protocols::DecodedSignal;

        if !self.transmit_allowed() {
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
        let advance = self.tx_advance(id, protocol_name, &decoded, button, &signal);
        self.queue_transmit_advancing(signal, capture.frequency, modulation, advance);
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

    /// True if a capture with the same protocol, data, serial, and button already exists.
    /// Unknown signals (no protocol) are never treated as duplicates so they can be kept for research.
    fn capture_duplicate_of_existing(&self, capture: &Capture) -> bool {
        files::is_duplicate(&self.captures, capture)
    }

    /// Process pending radio events
    pub fn process_radio_events(&mut self) -> Result<()> {
        self.poll_psa_key_recovery();
        self.poll_keeloq_key_recovery();
        self.poll_hitag2_key_recovery();
        self.auto_bf_tick();
        if let Some(ref rssi_arc) = self.rssi_source {
            self.rssi = f32::from_bits(rssi_arc.load(Ordering::Relaxed));
        }
        while let Ok(event) = self.radio_event_rx.try_recv() {
            match event {
                RadioEvent::SignalCaptured(capture) => self.add_received_signal(capture),
                RadioEvent::Error(e) => {
                    // Only a receive thread sends this, when RX ended on its own (open/configure
                    // failure, unplug); it already stopped streaming, ended any recording and zeroed
                    // the level. Back to idle, so one `r` restarts it (the header kept "RX" before).
                    match self.radio_state {
                        RadioState::Receiving => self.radio_state = RadioState::Idle,
                        RadioState::Transmitting if self.pending_transmit_restore == Some(RadioState::Receiving) => {
                            self.pending_transmit_restore = Some(RadioState::Idle);
                        }
                        _ => {}
                    }
                    self.rssi = 0.0;
                    self.last_error = Some(format!("Receiver stopped: {e} (r restarts)"));
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
        // No capture to act on (the menu's capture was deleted): nothing to offer.
        if self.signal_menu_capture().is_none() {
            return Vec::new();
        }
        let mut actions = self.signal_actions_base();
        self.adjust_signal_actions(&mut actions);
        self.add_send_button_action(&mut actions);
        if let Some(action) = self.key_recovery_action() {
            actions.insert(0, action);
        }
        actions
    }

    /// "Recover key" for an undecrypted PSA (TEA) capture, a KeeLoq frame no key decrypts or a
    /// HiTag2 frame without a key, or "Cancel key recovery" while one runs for it.
    fn key_recovery_action(&self) -> Option<SignalAction> {
        let c = self.signal_menu_capture()?;
        if self.key_search_runs_for(c.id) {
            return Some(SignalAction::CancelKeyRecovery);
        }
        // A PSA search (well under a second) runs beside a long one, or waits for the PSA search
        // that runs (app/auto_bf.rs).
        if crate::protocols::psa_needs_key_search(c.protocol_name(), c.serial, c.data_extra) {
            return Some(SignalAction::RecoverKey);
        }
        if self.long_key_search_runs() {
            return None;
        }
        if Self::keeloq_bf_offered(c) {
            return Some(SignalAction::RecoverKeeloqKey);
        }
        self.hitag2_bf_offered(c).then_some(SignalAction::RecoverHitag2Key)
    }

    /// Whether a key search runs for capture `id` (the signal menu offers to cancel it).
    fn key_search_runs_for(&self, id: u32) -> bool {
        self.psa_recover_job.as_ref().is_some_and(|j| j.capture_id == Some(id))
            || self.keeloq_bf_job.as_ref().is_some_and(|j| j.capture_id == id)
            || self.hitag2_bf_job.as_ref().is_some_and(|j| j.capture_ids.contains(&id))
    }

    fn signal_actions_base(&self) -> Vec<SignalAction> {
        let has_tx = self.radio.as_ref().is_some_and(|r| r.supports_tx());
        let selected = self.signal_menu_capture();
        let encoder_capable = selected
            .is_some_and(|c| c.status == crate::capture::CaptureStatus::EncoderCapable);
        // Gate/garage/barrier remotes (and KeeLoq barrier/alarm manufacturers) get the replay /
        // next-code menu instead of the car commands.
        let is_non_car_keeloq = selected.is_some_and(|c| {
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

    /// Execute the highlighted signal action on the menu's capture
    pub fn execute_signal_action(&mut self) -> Result<()> {
        // A new action: the previous message goes.
        self.clear_messages();
        // The row on screen, which follows its action when the list changes (sync_signal_menu);
        // never clamped onto another row (that landed on "Delete Signal").
        let Some(action) = self.available_signal_actions().get(self.signal_menu_index).copied() else {
            return Ok(());
        };
        let capture_id = match self.signal_menu_capture() {
            Some(c) => c.id,
            None => {
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
            SignalAction::RecoverHitag2Key => {
                self.start_hitag2_key_recovery(capture_id);
            }
            SignalAction::CancelKeyRecovery => {
                // The search that runs for this capture (an autobf PSA search may run beside a
                // long one).
                if self.hitag2_bf_job.as_ref().is_some_and(|j| j.capture_ids.contains(&capture_id)) {
                    self.cancel_hitag2_key_recovery();
                } else if self.keeloq_bf_job.as_ref().is_some_and(|j| j.capture_id == capture_id) {
                    self.cancel_keeloq_key_recovery();
                } else if let Some(job) = &self.psa_recover_job {
                    job.ctl.cancel.store(true, Ordering::Relaxed);
                    self.status_message = Some("Cancelling key recovery…".to_string());
                }
            }
        }
        Ok(())
    }

    /// Start ProtoPirate's PSA TEA key brute force for a capture on a background thread (the
    /// "Recover key" action); while another PSA search runs it waits, first in line. It runs
    /// beside a long KeeLoq or HiTag2 search.
    fn start_psa_key_recovery(&mut self, id: u32) {
        let Some(capture) = self.captures.iter().find(|c| c.id == id) else {
            self.last_error = Some(format!("Capture {} not found", id));
            return;
        };
        let Some(extra) = capture.data_extra else {
            self.last_error =
                Some("Capture has no key2; re-import it from its raw pairs".to_string());
            return;
        };
        if self.psa_recover_job.is_some() {
            self.queue_asked_key_search(id);
            self.status_message = Some(format!("Key recovery for capture {id} queued"));
            return;
        }
        let data = capture.data;
        self.spawn_psa_job(data, extra, Some(id), None, false);
        // Its progress is job_status() (status bar title), not this line: rewriting the message
        // every tick wiped every other message.
        self.status_message = Some(format!("Recovering PSA key for capture {}…", id));
    }

    /// Run the PSA key search on (`data`, `extra`) on a background thread, for listed capture
    /// `capture_id` or for an `unchecked` frame.
    fn spawn_psa_job(&mut self, data: u64, extra: u64, capture_id: Option<u32>, unchecked: Option<Box<Capture>>, auto: bool) {
        let ctl = Arc::new(PsaBruteForceControl::default());
        let worker = Arc::clone(&ctl);
        let handle = std::thread::spawn(move || psa_recover_key(data, extra, &worker));
        self.psa_recover_job = Some(PsaRecoverJob { capture_id, unchecked, auto, ctl, handle });
    }

    /// Progress of a running PSA key recovery, for [App::job_status].
    fn psa_key_recovery_progress(&self) -> Option<String> {
        let job = self.psa_recover_job.as_ref().filter(|j| !j.handle.is_finished())?;
        let done = u64::from(job.ctl.progress.load(Ordering::Relaxed));
        let pct = (done * 100 / u64::from(PSA_BRUTE_FORCE_TOTAL)).min(99);
        Some(match (job.capture_id, job.auto) {
            (Some(id), false) => format!("Recovering PSA key for capture {id}… {pct}%"),
            (Some(id), true) => format!("Checking the PSA key of capture {id}… {pct}%"),
            (None, _) => format!("Checking a PSA frame… {pct}%"),
        })
    }

    /// Collect the result of a running PSA key recovery (progress: [App::job_status]).
    fn poll_psa_key_recovery(&mut self) {
        let Some(job) = &self.psa_recover_job else {
            return;
        };
        if !job.handle.is_finished() {
            return;
        }
        let Some(job) = self.psa_recover_job.take() else {
            return;
        };
        let outcome = job.handle.join();
        match (job.capture_id, job.unchecked) {
            (Some(id), _) => self.psa_search_done(id, job.auto, outcome),
            (None, Some(frame)) => self.unchecked_search_done(*frame, outcome),
            (None, None) => {}
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
        // Makes such as "Kia/Hyundai" must not turn the name into a path, and metadata typed with
        // `i` may hold characters a Flipper's FAT card refuses (the export would fail).
        files::safe_stem(&format!("{}_{}_{}_{}_{}", year, make, model, region, command))
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
            .map(Self::default_export_filename)
            .unwrap_or_else(|| format!("capture_{}", id));
        let suffix_nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;
        let suffix = (suffix_nanos.wrapping_add(id as u64 * 2654435761) % 0x100_000_000) as u32;
        self.export_filename = format!("{}_{:08X}", default_name, suffix);

        // Pre-fill metadata from capture if set, otherwise make from protocol
        let make = capture
            .and_then(|c| c.make.clone())
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
            .cloned()
            .unwrap_or_default();
        self.fob_meta_make = make;
        self.fob_meta_model = capture
            .and_then(|c| c.model.as_ref())
            .cloned()
            .unwrap_or_default();
        self.fob_meta_region = capture
            .and_then(|c| c.region.as_ref())
            .cloned()
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
        let stem = if already_has_8hex {
            self.export_filename.trim().to_string()
        } else {
            let base = self.export_filename.trim();
            let suffix_nanos = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64;
            let suffix = (suffix_nanos.wrapping_add(id as u64 * 2654435761) % 0x100_000_000) as u32;
            let hex_suffix = format!("{:08X}", suffix);
            if base.is_empty() {
                format!("unknown_{}_{}", id, hex_suffix)
            } else {
                format!("{}_{}", base, hex_suffix)
            }
        };

        self.export_capture_id = None;
        self.export_format = None;
        // Never over an existing file; a failure is reported and KAT keeps running (app/files.rs).
        let path = match self.export_path(&stem, "fob") {
            Ok(path) => path,
            Err(e) => {
                self.last_error = Some(e);
                return Ok(());
            }
        };
        let result = crate::export::fob::export_fob(
            &capture,
            &path,
            self.storage.config.include_raw_pairs,
            Some(&metadata),
        );
        if self.report_export(&path, result) {
            // Saved-fob matching picks the new file up (app/fob_matching.rs).
            self.saved_fobs_rescan();
        }
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
            // A form submit is a new action: its own message replaces the last one.
            self.last_error = None;
            self.status_message = Some(format!("Saved metadata for #{:02}", id));
        } else {
            self.last_error = Some(format!("Capture {} is no longer in the list; metadata not saved", id));
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
        // Absolute, so ".." always leads somewhere (`:load .` then ".." used to reach "", which
        // cannot be read).
        if let Ok(dir) = std::fs::canonicalize(&self.load_browser_cwd) {
            self.load_browser_cwd = dir;
        }
        let mut entries: Vec<(String, PathBuf, bool)> = Vec::new();

        // ".." in every folder that has a parent, the import folder too: an empty import folder
        // (the default on a fresh install) was a dead end.
        if let Some(parent) = self.load_browser_cwd.parent() {
            entries.push(("..".to_string(), parent.to_path_buf(), true));
        }

        let dir_entries = match std::fs::read_dir(&self.load_browser_cwd) {
            Ok(d) => d,
            Err(e) => {
                // The folder's name, not its path: the reason must fit in the status line (the
                // browser shows the path).
                let name = self.load_browser_cwd.file_name().map(|n| n.to_string_lossy().to_string());
                let name = name.unwrap_or_else(|| self.load_browser_cwd.display().to_string());
                self.last_error = Some(format!("Cannot open folder {}: {} (Backspace: back)", name, e));
                self.load_browser_entries = entries;
                // Reset the selection and scroll too: kept from the old listing, they made the
                // renderer slice past the end of this one (a panic).
                self.load_browser_selected = 0;
                self.load_browser_scroll = 0;
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

        dirs.sort_by_key(|a| a.0.to_lowercase());
        files.sort_by_key(|a| a.0.to_lowercase());

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
        let Some((name, path, is_dir)) = self.load_browser_entries.get(self.load_browser_selected)
        else {
            return Ok(());
        };
        if name == ".." {
            // Up, with the folder just left selected (app/input.rs).
            return self.load_browser_up();
        }
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
            // import_fob_files reports what it imported.
            self.pending_fob_files = vec![path];
            self.import_fob_files()?;
            self.input_mode = InputMode::Normal;
        }
        Ok(())
    }

    /// `:load <file|dir>`: import a `.sub` / `.fob` file, decode an IQ recording, or open the file
    /// browser in a directory. Found like `:iq` finds its file (as given, else in the import and
    /// export folders), with quotes removed and `~` expanded (app/input.rs).
    pub fn load_path(&mut self, arg: &str) -> Result<()> {
        let path = self.resolve_load_arg(arg);
        if path.is_dir() {
            self.load_browser_cwd = path;
            self.load_browser_selected = 0;
            self.refresh_load_browser_entries()?;
            self.input_mode = InputMode::LoadFileBrowser;
        } else if !path.exists() {
            self.last_error = Some(format!(
                "No such file or folder: {} (looked here, in the import folder and in the export folder)",
                path.display()
            ));
        } else if crate::radio::iq_file::is_iq_path(&path) {
            self.start_iq_file(path, None, None);
        } else {
            // import_fob_files reports what it imported.
            self.pending_fob_files = vec![path];
            self.import_fob_files()?;
        }
        Ok(())
    }

    /// Close the load file browser without importing.
    pub fn close_load_browser(&mut self) {
        self.input_mode = InputMode::Normal;
    }

    /// Import pending .fob and .sub files into captures list.
    /// .sub files are decoded with registered protocols after load (no metadata in file).
    /// With show_unknown_signals off, only decoded captures are added (as in live receive); with
    /// ignore_duplicates off, a signal already in the list is added again.
    /// The status line says what each file gave (app/files.rs); the first new capture is selected.
    pub fn import_fob_files(&mut self) -> Result<()> {
        let paths = std::mem::take(&mut self.pending_fob_files);
        let first_new = self.captures.len();
        let import_dir = self.storage.import_dir().clone();
        let mut importer = files::Importer {
            protocols: &mut self.protocols,
            captures: &mut self.captures,
            next_id: &mut self.next_capture_id,
            show_unknown: self.storage.config.show_unknown_signals,
            ignore_duplicates: self.storage.config.ignore_duplicates,
            import_dir: &import_dir,
            unchecked: Vec::new(),
        };
        let results: Vec<(String, files::FileImport)> = paths.iter().map(|path| importer.import(path)).collect();
        let unchecked = std::mem::take(&mut importer.unchecked);
        // autobf: the new undecrypted PSA captures, and the frames listed only if their key is found.
        let new_ids: Vec<u32> = self.captures[first_new..].iter().map(|c| c.id).collect();
        self.auto_bf_listed(new_ids);
        for frame in unchecked {
            self.auto_bf_unchecked(frame);
        }

        if self.captures.len() > first_new {
            self.selected_capture = Some(first_new);
            self.ensure_selection_visible();
        }
        let (status, error) = files::import_summary(&results);
        if status.is_some() {
            self.status_message = status;
        }
        if error.is_some() {
            self.last_error = error;
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
            .map(Self::default_export_filename)
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

        self.export_capture_id = None;
        self.export_format = None;
        // Never over an existing file; a failure is reported and KAT keeps running (app/files.rs).
        match self.export_path(&self.export_filename, "sub") {
            Ok(path) => {
                let result = crate::export::flipper::export_flipper_sub(&capture, &path);
                self.report_export(&path, result);
            }
            Err(e) => self.last_error = Some(e),
        }
        Ok(())
    }

    // -- Settings Menu helpers --

    /// Get the current value index for the active settings field: its row in
    /// [App::settings_value_rows] (a value that is no preset has a row of its own, so Enter on
    /// the highlighted row keeps it instead of applying the first preset)
    pub fn current_settings_value_index(&self) -> usize {
        self.settings_choices().1
    }

    /// Get the number of values for the active settings field
    pub fn settings_value_count(&self) -> usize {
        self.settings_value_rows().len()
    }

    /// Apply the selected settings value
    pub fn apply_settings_value(&mut self) -> Result<()> {
        use input::SettingChoice;
        let Some(&(_, choice)) = self.settings_choices().0.get(self.settings_value_index) else {
            return Ok(());
        };
        match choice {
            SettingChoice::Freq(hz) => self.set_frequency(hz)?,
            SettingChoice::Hop(on) => self.set_hopping(on),
            SettingChoice::Lna(gain) => self.set_lna_gain(gain)?,
            SettingChoice::Vga(gain) => self.set_vga_gain(gain)?,
            SettingChoice::Amp(on) => self.set_amp(on)?,
            SettingChoice::TxStep(index) => self.apply_tx_power_value(index),
            SettingChoice::TxPower(gain, amp) => self.set_tx_power(gain, amp),
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
            on_air: None,
            key_recovery: None,
        };
        self.next_capture_id += 1;
        self.captures.push(capture);
    }
}
