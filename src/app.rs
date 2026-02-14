//! Application state management.

use anyhow::Result;
use std::sync::mpsc::{self, Receiver, Sender};

use crate::capture::{ButtonCommand, Capture};
use crate::protocols::ProtocolRegistry;
use crate::radio::{HackRfController, LevelDuration};
use crate::storage::Storage;

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
    /// Startup prompt: found .fob files, import? (y/n)
    StartupImport,
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
    /// Fob export metadata: editing notes field
    FobMetaNotes,
    /// License overlay (centered box)
    License,
    /// Credits overlay (centered box)
    Credits,
}

/// Export format being used
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExportFormat {
    Fob,
    Flipper,
}

/// Items available in the signal action menu
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SignalAction {
    Replay,
    Lock,
    Unlock,
    Trunk,
    Panic,
    ExportFob,
    ExportFlipper,
    Delete,
}

impl SignalAction {
    pub const ALL: [SignalAction; 8] = [
        SignalAction::Replay,
        SignalAction::Lock,
        SignalAction::Unlock,
        SignalAction::Trunk,
        SignalAction::Panic,
        SignalAction::ExportFob,
        SignalAction::ExportFlipper,
        SignalAction::Delete,
    ];

    pub fn label(&self) -> &'static str {
        match self {
            SignalAction::Replay => "Replay",
            SignalAction::Lock => "TX Lock",
            SignalAction::Unlock => "TX Unlock",
            SignalAction::Trunk => "TX Trunk",
            SignalAction::Panic => "TX Panic",
            SignalAction::ExportFob => "Export .fob",
            SignalAction::ExportFlipper => "Export .sub (Flipper)",
            SignalAction::Delete => "Delete Signal",
        }
    }
}

/// Radio settings selectable via Tab
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SettingsField {
    Freq,
    Lna,
    Vga,
    Amp,
}

impl SettingsField {
    pub const ALL: [SettingsField; 4] = [
        SettingsField::Freq,
        SettingsField::Lna,
        SettingsField::Vga,
        SettingsField::Amp,
    ];

    pub fn label(&self) -> &'static str {
        match self {
            SettingsField::Freq => "Freq",
            SettingsField::Lna => "LNA",
            SettingsField::Vga => "VGA",
            SettingsField::Amp => "AMP",
        }
    }
}

/// Common keyfob frequencies (Hz)
pub const PRESET_FREQUENCIES: [(u32, &str); 9] = [
    (300_000_000, "300.00 MHz"),
    (303_875_000, "303.875 MHz"),
    (310_000_000, "310.00 MHz"),
    (315_000_000, "315.00 MHz"),
    (318_000_000, "318.00 MHz"),
    (390_000_000, "390.00 MHz"),
    (433_920_000, "433.92 MHz"),
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
    /// HackRF controller (optional - may not be connected)
    hackrf: Option<HackRfController>,
    /// Channel for radio events
    radio_event_rx: Receiver<RadioEvent>,
    /// Sender for radio events (cloned to radio thread)
    #[allow(dead_code)]
    radio_event_tx: Sender<RadioEvent>,

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
    /// Notes input buffer
    pub fob_meta_notes: String,
}

impl App {
    /// Create a new application instance
    pub fn new() -> Result<Self> {
        let storage = Storage::new()?;

        // ── Load protocol encryption keys from embedded keystore ─────────
        crate::protocols::keys::load_keystore_from_embedded();

        let protocols = ProtocolRegistry::new();
        let (radio_event_tx, radio_event_rx) = mpsc::channel();

        // Try to initialize HackRF
        let hackrf = match HackRfController::new(radio_event_tx.clone()) {
            Ok(mut h) => {
                tracing::info!("HackRF initialized successfully");
                // Push config defaults to the controller so they're used on first start_receiving
                let _ = h.set_lna_gain(storage.config.default_lna_gain);
                let _ = h.set_vga_gain(storage.config.default_vga_gain);
                let _ = h.set_amp_enable(storage.config.default_amp);
                Some(h)
            }
            Err(e) => {
                tracing::warn!("Failed to initialize HackRF: {}", e);
                None
            }
        };

        let radio_state = if hackrf.is_some() {
            RadioState::Idle
        } else {
            RadioState::Disconnected
        };

        // Captures start empty — they are in-memory only and discarded on exit.
        // The user is offered the chance to import .fob files from their exports folder.
        let captures: Vec<Capture> = Vec::new();
        let next_capture_id = 1u32;

        // Use config defaults for radio settings
        let frequency = storage.config.default_frequency;
        let lna_gain = storage.config.default_lna_gain;
        let vga_gain = storage.config.default_vga_gain;
        let amp_enabled = storage.config.default_amp;

        // Recursively scan import directory for .fob and .sub at startup (separate from export dir)
        let pending_fob_files =
            crate::export::scan_import_files_recursive(storage.import_dir());
        let initial_mode = if !pending_fob_files.is_empty() {
            tracing::info!(
                "Found {} importable file(s) in import dir (recursive)",
                pending_fob_files.len()
            );
            InputMode::StartupImport
        } else {
            InputMode::Normal
        };

        Ok(Self {
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
            signal_menu_index: 0,
            overlay_scroll: 0,
            settings_field_index: 0,
            settings_value_index: 0,
            next_capture_id,
            storage,
            protocols,
            hackrf,
            radio_event_rx,
            radio_event_tx,
            pending_fob_files,
            export_capture_id: None,
            export_filename: String::new(),
            export_format: None,
            fob_meta_year: String::new(),
            fob_meta_make: String::new(),
            fob_meta_model: String::new(),
            fob_meta_region: String::new(),
            fob_meta_notes: String::new(),
        })
    }

    /// Get the frequency in MHz
    pub fn frequency_mhz(&self) -> f64 {
        self.frequency as f64 / 1_000_000.0
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
                self.last_error = Some("HackRF not connected".to_string());
            }
            RadioState::Idle => {
                if let Some(ref mut hackrf) = self.hackrf {
                    hackrf.start_receiving(self.frequency)?;
                    self.radio_state = RadioState::Receiving;
                    self.status_message = Some(format!("Receiving on {:.2} MHz", self.frequency_mhz()));
                }
            }
            RadioState::Receiving => {
                if let Some(ref mut hackrf) = self.hackrf {
                    hackrf.stop_receiving()?;
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
        let parts: Vec<&str> = command.trim().split_whitespace().collect();
        if parts.is_empty() {
            return Ok(());
        }

        self.last_error = None;
        self.status_message = None;

        match parts[0] {
            "q" | "quit" => {
                // Will be handled by main loop
                std::process::exit(0);
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
            "unlock" => self.transmit_command(parts.get(1), ButtonCommand::Unlock)?,
            "lock" => self.transmit_command(parts.get(1), ButtonCommand::Lock)?,
            "trunk" => self.transmit_command(parts.get(1), ButtonCommand::Trunk)?,
            "panic" => self.transmit_command(parts.get(1), ButtonCommand::Panic)?,
            "license" | "licence" => {
                self.input_mode = InputMode::License;
                self.overlay_scroll = 0;
            }
            "credits" => {
                self.input_mode = InputMode::Credits;
                self.overlay_scroll = 0;
            }
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

        // If receiving, restart receiver so the new frequency takes effect (HackRF thread reads freq only at start)
        if let Some(ref mut hackrf) = self.hackrf {
            if self.radio_state == RadioState::Receiving {
                hackrf.stop_receiving()?;
                hackrf.start_receiving(hz)?;
            } else {
                hackrf.set_frequency(hz)?;
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

        if let Some(ref mut hackrf) = self.hackrf {
            hackrf.set_lna_gain(gain)?;
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

        if let Some(ref mut hackrf) = self.hackrf {
            hackrf.set_vga_gain(gain)?;
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

        if let Some(ref mut hackrf) = self.hackrf {
            hackrf.set_amp_enable(enabled)?;
        }

        self.status_message = Some(format!("Amp {}", if enabled { "enabled" } else { "disabled" }));
        Ok(())
    }

    /// Transmit a command for a capture
    fn transmit_command(&mut self, id_str: Option<&&str>, command: ButtonCommand) -> Result<()> {
        use crate::protocols::DecodedSignal;
        
        let id_str = match id_str {
            Some(s) => s,
            None => {
                self.last_error = Some(format!("Usage: :{:?} <ID>", command).to_lowercase());
                return Ok(());
            }
        };

        let id: u32 = match id_str.parse() {
            Ok(i) => i,
            Err(_) => {
                self.last_error = Some("Invalid capture ID".to_string());
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

        // Create a DecodedSignal from the capture
        let decoded = DecodedSignal {
            serial: capture.serial,
            button: capture.button,
            counter: capture.counter,
            crc_valid: capture.crc_valid,
            data: capture.data,
            data_count_bit: capture.data_count_bit,
            encoder_capable: true,
            extra: capture.data_extra,
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

        // Transmit
        if let Some(ref mut hackrf) = self.hackrf {
            hackrf.transmit(&signal, capture.frequency)?;
            self.status_message = Some(format!("Transmitted {:?} for capture {}", command, id));
        } else {
            self.last_error = Some("HackRF not connected".to_string());
        }

        Ok(())
    }

    /// Replay a capture by re-transmitting its raw level/duration pairs (no re-encoding).
    pub fn replay_capture(&mut self, id: u32) -> Result<()> {
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

        if let Some(ref mut hackrf) = self.hackrf {
            hackrf.transmit(&signal, capture.frequency)?;
            self.status_message = Some(format!("Replayed capture {} ({} pairs)", id, signal.len()));
        } else {
            self.last_error = Some("HackRF not connected".to_string());
        }

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

    /// Delete a capture by ID (in-memory only — captures are not persisted)
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

    /// Delete all captures (in-memory only)
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

    /// Process pending radio events
    pub fn process_radio_events(&mut self) -> Result<()> {
        while let Ok(event) = self.radio_event_rx.try_recv() {
            match event {
                RadioEvent::SignalCaptured(mut capture) => {
                    // Convert stored pairs to the format protocols expect
                    let pairs: Vec<crate::radio::LevelDuration> = capture.raw_pairs
                        .iter()
                        .map(|p| crate::radio::LevelDuration::new(p.level, p.duration_us))
                        .collect();

                    // Try to decode with registered protocols
                    if let Some((protocol_name, decoded)) = self.protocols.process_signal(&pairs, capture.frequency) {
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
                        capture.id = self.next_capture_id;
                        self.next_capture_id += 1;
                        // Captures are in-memory only — no auto-save to disk.
                        // Use Export (.fob / .sub) to persist a signal.
                        self.captures.push(capture);

                        // Auto-select and scroll to new capture
                        let new_idx = self.captures.len() - 1;
                        self.selected_capture = Some(new_idx);
                        self.ensure_selection_visible();

                        self.status_message = Some("New signal captured".to_string());
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
        Ok(())
    }

    // -- Signal Action Menu helpers --

    /// Execute the currently selected signal action
    pub fn execute_signal_action(&mut self) -> Result<()> {
        let action = SignalAction::ALL[self.signal_menu_index];
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
            SignalAction::Lock => {
                let id_str = capture_id.to_string();
                self.transmit_command(Some(&&*id_str.as_str()), ButtonCommand::Lock)?;
            }
            SignalAction::Unlock => {
                let id_str = capture_id.to_string();
                self.transmit_command(Some(&&*id_str.as_str()), ButtonCommand::Unlock)?;
            }
            SignalAction::Trunk => {
                let id_str = capture_id.to_string();
                self.transmit_command(Some(&&*id_str.as_str()), ButtonCommand::Trunk)?;
            }
            SignalAction::Panic => {
                let id_str = capture_id.to_string();
                self.transmit_command(Some(&&*id_str.as_str()), ButtonCommand::Panic)?;
            }
            SignalAction::ExportFob => {
                self.export_fob(capture_id)?;
            }
            SignalAction::ExportFlipper => {
                self.export_flipper(capture_id)?;
            }
            SignalAction::Delete => {
                let id_str = capture_id.to_string();
                self.delete_capture(&id_str)?;
            }
        }
        Ok(())
    }

    /// Generate a default export filename (without extension) for a capture
    fn default_export_filename(capture: &Capture) -> String {
        format!(
            "{}_{}",
            capture.protocol_name().replace(' ', "_").to_lowercase(),
            capture.serial_hex()
        )
    }

    /// Start .fob export by entering filename input mode
    pub fn export_fob(&mut self, id: u32) -> Result<()> {
        if !self.captures.iter().any(|c| c.id == id) {
            self.last_error = Some(format!("Capture {} not found", id));
            return Ok(());
        }

        // Pre-fill filename from protocol + serial
        let default_name = self.captures.iter().find(|c| c.id == id)
            .map(|c| Self::default_export_filename(c))
            .unwrap_or_else(|| format!("capture_{}", id));

        // Pre-fill make from protocol
        let make = self.captures.iter().find(|c| c.id == id).map(|c| {
            Self::get_make_for_protocol(c.protocol_name()).to_string()
        }).unwrap_or_default();

        self.export_capture_id = Some(id);
        self.export_filename = default_name;
        self.export_format = Some(ExportFormat::Fob);
        self.fob_meta_year = String::new();
        self.fob_meta_make = make;
        self.fob_meta_model = String::new();
        self.fob_meta_region = String::new();
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
            notes: self.fob_meta_notes.clone(),
        };

        let filename = format!("{}.fob", self.export_filename);
        let path = export_dir.join(&filename);

        crate::export::fob::export_fob(
            &capture,
            &path,
            self.storage.config.include_raw_pairs,
            Some(&metadata),
        )?;

        self.export_capture_id = None;
        self.export_format = None;
        self.status_message = Some(format!("Exported to {}", filename));
        Ok(())
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
                match crate::export::flipper::import_sub_raw(path) {
                    Ok((frequency, raw_pairs)) => {
                        let pairs: Vec<crate::radio::LevelDuration> = raw_pairs
                            .iter()
                            .map(|p| crate::radio::LevelDuration::new(p.level, p.duration_us))
                            .collect();
                        let decoded_list =
                            self.protocols.process_signal_stream(&pairs, frequency);
                        for (protocol_name, decoded, segment_pairs) in decoded_list {
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
                            self.next_capture_id += 1;
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
                                self.captures.push(capture);
                                imported += 1;
                            }
                        }
                    }
                    Err(e) => tracing::warn!("Failed to import {:?}: {}", path, e),
                }
            } else {
                match crate::export::fob::import_fob(path, self.next_capture_id) {
                    Ok(mut capture) => {
                        self.next_capture_id += 1;
                        // Re-run decoder when Unknown and raw_pairs present (same as .sub)
                        if capture.status == crate::capture::CaptureStatus::Unknown
                            && !capture.raw_pairs.is_empty()
                        {
                            let pairs: Vec<crate::radio::LevelDuration> = capture
                                .raw_pairs
                                .iter()
                                .map(|p| crate::radio::LevelDuration::new(p.level, p.duration_us))
                                .collect();
                            if let Some((protocol_name, decoded)) =
                                self.protocols.process_signal(&pairs, capture.frequency)
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
                        }
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
            SettingsField::Lna => {
                LNA_STEPS.iter().position(|&g| g == self.lna_gain).unwrap_or(0)
            }
            SettingsField::Vga => {
                VGA_STEPS.iter().position(|&g| g == self.vga_gain).unwrap_or(0)
            }
            SettingsField::Amp => {
                if self.amp_enabled { 0 } else { 1 }
            }
        }
    }

    /// Get the number of values for the active settings field
    pub fn settings_value_count(&self) -> usize {
        let field = SettingsField::ALL[self.settings_field_index];
        match field {
            SettingsField::Freq => PRESET_FREQUENCIES.len(),
            SettingsField::Lna => LNA_STEPS.len(),
            SettingsField::Vga => VGA_STEPS.len(),
            SettingsField::Amp => 2, // ON / OFF
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
        }
        Ok(())
    }

    /// Get the make for a protocol name
    pub fn get_make_for_protocol(protocol: &str) -> &'static str {
        match protocol {
            p if p.starts_with("Kia") => "Kia/Hyundai",
            p if p.starts_with("Ford") => "Ford",
            p if p.starts_with("Fiat") => "Fiat",
            "Subaru" => "Subaru",
            "Suzuki" => "Suzuki",
            "VAG" | "VW" => "VW/Audi/Seat/Skoda",
            "PSA" => "Peugeot/Citroen",
            "Star Line" => "Star Line",
            "Scher-Khan" => "Scher-Khan",
            _ => "Unknown",
        }
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
        };
        self.next_capture_id += 1;
        self.captures.push(capture);
    }
}
