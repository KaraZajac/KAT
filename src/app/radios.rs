//! KAT's radios: every HackRF, RTL-SDR and Flipper Zero it finds (config `device`), each tuned on
//! its own (frequency, gains, hopping, recording, receiving on or off). The commands and keys that
//! tune a radio act on the selected one (`1`–`9`, `:radio`, the Radio settings tab); one radio
//! transmits (`:txradio`, config `tx_radio`), and only it pauses receiving to do so. Captures say
//! which radio heard them; one heard while another of KAT's radios was transmitting near its
//! frequency is KAT's own transmission, marked as such (`own_tx`).
//!
//! With one radio KAT looks and works as it did before it used several.

use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::mpsc::{self, Receiver};
use std::sync::Arc;

use anyhow::Result;
use chrono::{DateTime, Duration, Utc};

use super::{App, InputMode, RadioDevice, RadioEvent, RadioState, SettingsField};
use crate::capture::Capture;
use crate::radio::devices::{discover, FoundRadio, RadioKind};
use crate::radio::hopper::{format_mhz, HopPlan};
use crate::radio::{FlipperController, HackRfController, RtlDevice, RtlSdrController};
use crate::storage::{Config, RadioPick};

/// How a radio is tuned.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Tuning {
    pub frequency: u32,
    /// LNA gain (0-40 dB, 8 dB steps): a HackRF's
    pub lna_gain: u32,
    /// VGA gain (0-62 dB, 2 dB steps): a HackRF's, and an RTL-SDR's one tuner gain
    pub vga_gain: u32,
    /// RF amplifier: a HackRF's
    pub amp: bool,
}

/// The settings a radio of `kind` has besides its frequency, in the settings bar's order: a
/// HackRF's two gains and amplifier (also without a radio: they are kept), an RTL-SDR's one tuner
/// gain. A Flipper has none: its receive preset sets its receiver, and it does not hop.
pub fn kind_settings(kind: Option<RadioKind>) -> &'static [SettingsField] {
    use SettingsField::{Amp, Gain, Hop, Lna, Vga};
    match kind {
        None | Some(RadioKind::HackRf) => &[Hop, Lna, Vga, Amp],
        Some(RadioKind::RtlSdr) => &[Hop, Gain],
        Some(RadioKind::Flipper) => &[],
    }
}

/// Why `:lna`, `:vga` or `:amp` has nothing to set on the radio `name` of `kind`; None when it
/// has. They used to report success on every radio: on a Flipper, which has no gain settings, and
/// on an RTL-SDR, where `:lna` and `:vga` both set its one gain (whichever came last won, the VGA
/// default at startup) and `:amp` nothing.
pub fn missing_setting(kind: RadioKind, name: &str, field: SettingsField) -> Option<String> {
    use SettingsField::{Amp, Gain, Lna, Vga};
    match (kind, field) {
        (RadioKind::Flipper, Lna | Vga | Gain | Amp) => {
            Some(format!("{name} has no gain settings (its receive preset sets them)"))
        }
        (RadioKind::RtlSdr, Lna) => Some(format!("{name} has one gain: :vga <0-62> sets it")),
        (RadioKind::RtlSdr, Amp) => Some(format!("{name} has no amplifier (its one gain: :vga <0-62>)")),
        _ => None,
    }
}

impl Tuning {
    /// The general defaults (`default_frequency`, `default_lna_gain`, `default_vga_gain`,
    /// `default_amp`).
    pub fn defaults(config: &Config) -> Self {
        Self {
            frequency: config.default_frequency,
            lna_gain: config.default_lna_gain,
            vga_gain: config.default_vga_gain,
            amp: config.default_amp,
        }
    }

    /// A radio's start settings: the defaults, then its kind's `[radio.<kind>]` sections, then its
    /// own `[radio.<kind>.<serial>]` ones (the more specific wins).
    pub fn for_radio(found: &FoundRadio, config: &Config) -> Self {
        let mut t = Self::defaults(config);
        let sections = &config.radio_sections;
        let kind_wide = sections.iter().filter(|s| s.serial.is_none());
        let own = sections.iter().filter(|s| s.serial.is_some());
        for s in kind_wide.chain(own).filter(|s| s.applies_to(found)) {
            t.frequency = s.frequency.unwrap_or(t.frequency);
            t.lna_gain = s.lna_gain.unwrap_or(t.lna_gain);
            t.vga_gain = s.vga_gain.unwrap_or(t.vga_gain);
            t.amp = s.amp.unwrap_or(t.amp);
        }
        t
    }
}

/// Frequency hopping of one radio.
#[derive(Debug, Clone, Default)]
pub struct HopState {
    /// Switched on (`:hop on`, the Hop setting); kept by the radio across RX restarts.
    pub enabled: bool,
    /// Set with `:hop <MHz> …` / `:hop dwell` (None: the config's list).
    pub plan: Option<HopPlan>,
    /// The radio's frequency when hopping was switched on: a `:freq` change switches it off.
    pub home: u32,
}

/// One of KAT's radios.
pub struct Radio {
    pub device: RadioDevice,
    pub found: FoundRadio,
    /// Its name on screen and in the captures it receives: "HackRF", "RTL-SDR 2".
    pub name: String,
    pub state: RadioState,
    pub tuning: Tuning,
    /// Latest signal level (average magnitude, 0..~1) while receiving.
    pub rssi: f32,
    /// Read every tick (no channel traffic, RX never blocks); None for a Flipper (no level).
    rssi_source: Option<Arc<AtomicU32>>,
    /// Its controller's events: captures, a receiver that stopped on its own.
    events: Receiver<RadioEvent>,
    pub hop: HopState,
    /// Why it stopped receiving on its own, or why it could not start (`:radios`).
    pub last_error: Option<String>,
}

impl Radio {
    /// A controller for `found`, tuned to `tuning`. Nothing is opened yet: an SDR is opened for
    /// each receive run and transmission, a Flipper's console on first use.
    pub fn new(found: FoundRadio, name: String, tuning: Tuning, config: &Config) -> Self {
        let (tx, events) = mpsc::channel();
        let mut device = match found.kind {
            RadioKind::HackRf => RadioDevice::HackRf(HackRfController::new(tx, found.serial.clone())),
            RadioKind::RtlSdr => RadioDevice::RtlSdr(RtlSdrController::new(tx, RtlDevice::of(&found))),
            RadioKind::Flipper => RadioDevice::Flipper(FlipperController::on_port(
                tx,
                found.port.clone().unwrap_or_default(),
                config.flipper_preset.as_deref(),
                config.autobf,
            )),
        };
        let _ = device.set_lna_gain(tuning.lna_gain);
        let _ = device.set_vga_gain(tuning.vga_gain);
        let _ = device.set_amp_enable(tuning.amp);
        let _ = device.set_frequency(tuning.frequency);
        let rssi_source = device.rssi_source();
        Self {
            device,
            found,
            name,
            state: RadioState::Idle,
            tuning,
            rssi: 0.0,
            rssi_source,
            events,
            hop: HopState::default(),
            last_error: None,
        }
    }

    pub fn kind(&self) -> RadioKind {
        self.found.kind
    }

    /// Its settings as the header shows them: a HackRF's gains, an RTL-SDR's one gain, or a
    /// Flipper's receive preset.
    pub fn settings_label(&self, flipper_preset: Option<&str>) -> String {
        match self.kind() {
            RadioKind::Flipper => format!("Preset: {}", flipper_preset.unwrap_or("firmware's AM")),
            RadioKind::RtlSdr => format!("Gain:{}", self.tuning.vga_gain),
            RadioKind::HackRf => format!(
                "LNA:{} VGA:{} AMP:{}",
                self.tuning.lna_gain,
                self.tuning.vga_gain,
                if self.tuning.amp { "ON" } else { "OFF" }
            ),
        }
    }
}

/// Names for newly found radios, in order: the kind's name for the first of a kind, then "HackRF
/// 2", "HackRF 3" … counting the radios of the kind KAT already has (`existing`), so a radio
/// keeps its name when more are found later (captures carry it).
pub fn radio_names(found: &[FoundRadio], existing: &[RadioKind]) -> Vec<String> {
    let mut seen: Vec<RadioKind> = existing.to_vec();
    found
        .iter()
        .map(|f| {
            seen.push(f.kind);
            match seen.iter().filter(|&&k| k == f.kind).count() {
                1 => f.kind.label().to_string(),
                n => format!("{} {n}", f.kind.label()),
            }
        })
        .collect()
}

/// The radio that transmits: the one `pick` names (config `tx_radio`), else the first HackRF, else
/// the first radio that can (a Flipper).
pub fn choose_tx(found: &[&FoundRadio], pick: Option<&RadioPick>) -> Option<usize> {
    pick.and_then(|p| found.iter().position(|f| p.matches(f)))
        .or_else(|| found.iter().position(|f| f.kind == RadioKind::HackRf))
        .or_else(|| found.iter().position(|f| f.kind.can_transmit()))
}

/// What a radio command or key says when KAT has no radio at all: `:radios` looks for the ones
/// plugged in since it started.
pub(super) const NO_RADIO: &str = "No radio connected (:radios looks for one)";

/// The radios of `config` that are plugged in, named and tuned, and why each kind without one is
/// not in use. Nothing is opened: starting KAT never claims a radio, so a Flipper another
/// program is using (qFlipper, a terminal on its console) is left alone until it is received or
/// transmitted with here; KAT 1.10 connected at startup when it was the only radio, and its
/// Ctrl+C would have ended what the other program ran on it.
pub(super) fn open_radios(config: &Config) -> (Vec<Radio>, Vec<String>) {
    let d = discover(&config.radio_devices, config.flipper_port.as_deref());
    let names = radio_names(&d.radios, &[]);
    let radios = d
        .radios
        .into_iter()
        .zip(names)
        .map(|(found, name)| {
            let tuning = Tuning::for_radio(&found, config);
            Radio::new(found, name, tuning, config)
        })
        .collect();
    (radios, d.problems)
}

/// One transmission of one of KAT's radios, for telling its own transmissions from other signals.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TxWindow {
    pub radio: usize,
    pub frequency: u32,
    pub start: DateTime<Utc>,
    pub end: DateTime<Utc>,
}

/// How long after a transmission ended a capture of it may still arrive: a receiver reports a
/// burst once it is over and processed (an SDR's processing can be a second or two behind).
const OWN_TX_SLACK_MS: i64 = 2_500;
/// How far from a receiver's frequency a transmission is still heard (an SDR hears from 1.15 MHz
/// below to 0.65 MHz above it).
const OWN_TX_BAND_HZ: u32 = 1_200_000;
/// Transmissions kept for the check.
const OWN_TX_KEEP_S: i64 = 30;

/// The transmission a capture radio `receiver` made at `at` on `frequency` is, if any: received by
/// another of KAT's radios while one of them transmitted near that frequency.
pub fn own_transmission(windows: &[TxWindow], receiver: usize, at: DateTime<Utc>, frequency: u32) -> Option<&TxWindow> {
    windows.iter().rev().find(|w| {
        w.radio != receiver
            && at >= w.start
            && at <= w.end + Duration::milliseconds(OWN_TX_SLACK_MS)
            && w.frequency.abs_diff(frequency) <= OWN_TX_BAND_HZ
    })
}

/// "RX", "TX", "idle" (`:radios`, the status line).
fn state_word(state: RadioState) -> &'static str {
    match state {
        RadioState::Receiving => "RX",
        RadioState::Transmitting => "TX",
        RadioState::Idle => "idle",
        RadioState::Disconnected => "not connected",
    }
}

/// The `:radios` overlay's text (≤ 62 columns a line, like the help): each radio with what it
/// does now, its serial or port and settings, the frequencies each kind tunes, and why the kinds
/// KAT has none of are not in use.
pub fn overview_text(
    radios: &[Radio],
    selected: Option<usize>,
    tx: Option<usize>,
    problems: &[String],
    flipper_preset: Option<&str>,
) -> String {
    let mut out = String::from("RADIOS   1-9 or :radio <n> selects, :txradio <n> transmits\n\n");
    let name_w = radios.iter().map(|r| r.name.chars().count()).max().unwrap_or(0).max(5);
    for (i, r) in radios.iter().enumerate() {
        let mark = if selected == Some(i) { '▸' } else { ' ' };
        let now = match (r.state, r.hop.enabled) {
            (RadioState::Receiving, true) => format!("RX hopping ({} MHz now)", format_mhz(r.device.rx_frequency())),
            (RadioState::Receiving | RadioState::Transmitting, _) => {
                format!("{} {} MHz", state_word(r.state), format_mhz(r.tuning.frequency))
            }
            (s, _) => format!("{}, {} MHz", state_word(s), format_mhz(r.tuning.frequency)),
        };
        let can = match (r.device.supports_tx(), tx == Some(i)) {
            (true, true) => "RX+TX, transmits",
            (true, false) => "RX+TX",
            (false, _) => "RX only",
        };
        out.push_str(&format!("{mark}{:<2} {:<name_w$}  {now:<24}  {can}\n", i + 1, r.name));
        let mut more = vec![r.settings_label(flipper_preset)];
        if let Some(st) = r.device.recording_status() {
            more.push(format!("● REC {:.1} MB", st.bytes as f64 / (1u64 << 20) as f64));
        }
        if let RadioDevice::Flipper(f) = &r.device {
            match f.open_error() {
                Some(e) if !f.is_connected() => more.push(format!("could not connect: {e}")),
                _ if !f.is_connected() => more.push("not connected yet".to_string()),
                _ => {}
            }
        }
        out.push_str(&format!("    {:<name_w$}  {}\n", r.found.identity(), more.join(" · ")));
        if let Some(e) = &r.last_error {
            out.push_str(&format!("    {:<name_w$}  last stopped: {e}\n", ""));
        }
    }
    if radios.is_empty() {
        out.push_str("  None found.\n");
    }
    if radios.iter().any(|r| r.kind() == RadioKind::RtlSdr && r.found.serial.is_some() && !r.found.serial_unique) {
        out.push_str("\nRTL-SDRs with the same serial are told apart by USB order;\ngive each its own with rtl_eeprom -s <serial>.\n");
    }
    let kinds: Vec<RadioKind> =
        RadioKind::ALL.into_iter().filter(|k| radios.iter().any(|r| r.kind() == *k)).collect();
    if !kinds.is_empty() {
        out.push_str("\nFREQUENCIES\n");
        for k in kinds {
            out.push_str(&format!("  {:<9}{}\n", k.label(), k.coverage()));
        }
    }
    if !problems.is_empty() {
        out.push_str("\nNOT IN USE\n");
        for p in problems {
            out.push_str(&format!("  {p}\n"));
        }
    }
    out.push_str("\n:radios looks for radios plugged in since. config.ini [radio]\nsets which radios KAT uses and how each starts.");
    out
}

impl App {
    /// The radios KAT has, in the order the header and `:radios` list them.
    pub fn radios(&self) -> &[Radio] {
        &self.radios
    }

    /// The selected radio (the one `:freq`, `:lna`, `r` … act on); None without radios.
    pub fn selected_radio(&self) -> Option<usize> {
        (self.selected_radio < self.radios.len()).then_some(self.selected_radio)
    }

    /// The radio that transmits; None when none can.
    pub fn tx_radio(&self) -> Option<usize> {
        self.tx_radio.filter(|&i| i < self.radios.len())
    }

    pub(super) fn selected(&self) -> Option<&Radio> {
        self.selected_radio().map(|i| &self.radios[i])
    }

    pub(super) fn selected_mut(&mut self) -> Option<&mut Radio> {
        let i = self.selected_radio()?;
        Some(&mut self.radios[i])
    }

    /// The selected radio's tuning; without a radio, the settings `:freq`, `:lna` … still keep
    /// (`:iq` takes the frequency of a file that does not name one from it).
    pub fn tuning(&self) -> Tuning {
        self.selected().map_or(self.offline_tuning, |r| r.tuning)
    }

    pub(super) fn tuning_mut(&mut self) -> &mut Tuning {
        match self.selected_radio() {
            Some(i) => &mut self.radios[i].tuning,
            None => &mut self.offline_tuning,
        }
    }

    /// The selected radio's frequency.
    pub fn frequency(&self) -> u32 {
        self.tuning().frequency
    }

    /// The selected radio's state; Disconnected without radios.
    pub fn radio_state(&self) -> RadioState {
        self.selected().map_or(RadioState::Disconnected, |r| r.state)
    }

    /// Whether the selected radio hops.
    pub fn hop_enabled(&self) -> bool {
        self.selected().is_some_and(|r| r.hop.enabled)
    }

    /// More than one radio: the header lists each, the settings bar gets a Radio tab, captures
    /// show which radio heard them.
    pub fn several_radios(&self) -> bool {
        self.radios.len() > 1
    }

    /// Whether the captures list shows the Radio column: several radios, or captures from several.
    pub fn show_radio_column(&self) -> bool {
        if self.several_radios() {
            return true;
        }
        let mut names = self.captures.iter().filter_map(|c| c.radio.as_deref());
        names.next().is_some_and(|first| names.any(|n| n != first))
    }

    /// Radio `i`'s settings as the header shows them.
    pub fn radio_settings_label(&self, i: usize) -> String {
        self.radios.get(i).map_or_else(String::new, |r| r.settings_label(self.storage.config.flipper_preset.as_deref()))
    }

    /// `1`–`9`, `:radio <n>`, the Radio settings tab: the radio the tuning commands act on.
    pub fn select_radio(&mut self, i: usize) {
        let Some(r) = self.radios.get(i) else {
            self.last_error = Some(match self.radios.len() {
                0 => NO_RADIO.to_string(),
                1 => "There is only radio 1".to_string(),
                n => format!("There are radios 1-{n} (:radios lists them)"),
            });
            return;
        };
        self.selected_radio = i;
        self.status_message =
            Some(format!("Radio {}: {}, {} {} MHz", i + 1, r.name, state_word(r.state), format_mhz(r.tuning.frequency)));
    }

    /// A radio by its number (1-based) or its name ("rtl-sdr 2", "flipper"), as `:radio` and
    /// `:txradio` take it.
    fn find_radio(&self, arg: &str) -> Option<usize> {
        if let Ok(n) = arg.trim().trim_start_matches('#').parse::<usize>() {
            return n.checked_sub(1).filter(|&i| i < self.radios.len());
        }
        let squash = |s: &str| s.to_ascii_lowercase().replace(['-', '_', ' '], "");
        let wanted = squash(arg);
        self.radios
            .iter()
            .position(|r| squash(&r.name) == wanted)
            .or_else(|| RadioKind::parse(arg).and_then(|k| self.radios.iter().position(|r| r.kind() == k)))
    }

    /// `:radio [<n>|<name>]`: select a radio, or say which one is selected.
    pub(super) fn cmd_radio(&mut self, args: &[&str]) {
        if self.radios.is_empty() {
            self.last_error = Some(NO_RADIO.to_string());
            return;
        }
        if args.is_empty() {
            let i = self.selected_radio().unwrap_or(0);
            self.select_radio(i);
            return;
        }
        match self.find_radio(&args.join(" ")) {
            Some(i) => self.select_radio(i),
            None => self.last_error = Some(format!("No radio {} (:radios lists them)", args.join(" "))),
        }
    }

    /// `:txradio [<n>|<name>]`: choose the radio that transmits, or say which one does.
    pub(super) fn cmd_txradio(&mut self, args: &[&str]) {
        if args.is_empty() {
            match self.tx_radio() {
                Some(i) => self.status_message = Some(format!("Radio {} ({}) transmits", i + 1, self.radios[i].name)),
                None => self.last_error = Some("No radio here can transmit".to_string()),
            }
            return;
        }
        if self.has_pending_transmit() {
            self.last_error = Some("Transmissions are queued: wait for them, or Esc cancels them".to_string());
            return;
        }
        let Some(i) = self.find_radio(&args.join(" ")) else {
            self.last_error = Some(format!("No radio {} (:radios lists them)", args.join(" ")));
            return;
        };
        let r = &self.radios[i];
        if !r.device.supports_tx() {
            self.last_error = Some(format!("{} cannot transmit (receive only)", r.name));
            return;
        }
        self.tx_radio = Some(i);
        self.status_message = Some(format!("Radio {} ({}) transmits", i + 1, r.name));
    }

    /// `:radios`: look for radios plugged in since KAT started (they are added), then list them.
    pub(super) fn cmd_radios(&mut self) {
        let added = self.rescan_radios();
        if !added.is_empty() {
            self.status_message = Some(format!("Found {}", added.join(", ")));
        }
        self.input_mode = InputMode::Radios;
        self.overlay_scroll = 0;
    }

    /// Add the radios found that KAT does not have yet (a radio unplugged stays in the list: its
    /// receiver says it stopped). Returns their names.
    pub(super) fn rescan_radios(&mut self) -> Vec<String> {
        let config = &self.storage.config;
        let d = discover(&config.radio_devices, config.flipper_port.as_deref());
        let new: Vec<FoundRadio> =
            d.radios.into_iter().filter(|f| !self.radios.iter().any(|r| r.found.same_device(f))).collect();
        let existing: Vec<RadioKind> = self.radios.iter().map(Radio::kind).collect();
        let names = radio_names(&new, &existing);
        for (found, name) in new.into_iter().zip(names.iter().cloned()) {
            let tuning = Tuning::for_radio(&found, &self.storage.config);
            self.radios.push(Radio::new(found, name, tuning, &self.storage.config));
        }
        // Why kinds KAT has none of are not in use (a radio KAT itself has open can look busy to
        // a second look, on Windows).
        let have: Vec<&'static str> =
            self.radios.iter().map(|r| crate::radio::devices::kind_name(r.kind())).collect();
        self.radio_problems = d.problems.into_iter().filter(|p| !have.iter().any(|k| p.starts_with(&format!("{k}:")))).collect();
        if !self.radios.is_empty() {
            self.radio_open_error = None;
        }
        if self.tx_radio().is_none() {
            let found: Vec<&FoundRadio> = self.radios.iter().map(|r| &r.found).collect();
            self.tx_radio = choose_tx(&found, self.storage.config.tx_radio.as_ref());
        }
        names
    }

    /// The `:radios` overlay's text.
    pub fn radios_overview(&self) -> String {
        overview_text(
            &self.radios,
            self.selected_radio(),
            self.tx_radio(),
            &self.radio_problems,
            self.storage.config.flipper_preset.as_deref(),
        )
    }

    /// Start or stop receiving on radio `i`.
    pub(super) fn toggle_radio(&mut self, i: usize) -> Result<()> {
        let several = self.several_radios();
        let r = &mut self.radios[i];
        let on = if several { format!(" with {}", r.name) } else { String::new() };
        match r.state {
            RadioState::Disconnected => self.last_error = Some(format!("{} is not connected", r.name)),
            RadioState::Idle => {
                if let Err(e) = r.device.start_receiving(r.tuning.frequency) {
                    r.last_error = Some(format!("{e:#}"));
                    let name = r.name.clone();
                    return Err(e.context(name));
                }
                r.state = RadioState::Receiving;
                r.last_error = None;
                self.status_message = Some(format!("Receiving on {} MHz{on}", format_mhz(r.tuning.frequency)));
            }
            RadioState::Receiving => {
                r.device.stop_receiving()?;
                r.state = RadioState::Idle;
                r.rssi = 0.0;
                self.status_message = Some(format!("Stopped receiving{on}"));
            }
            RadioState::Transmitting => self.last_error = Some("Cannot change state while transmitting".to_string()),
        }
        Ok(())
    }

    /// `R`: stop every radio that receives, or, when none does, start them all.
    pub fn toggle_all_receiving(&mut self) -> Result<()> {
        self.last_error = None;
        if self.radios.is_empty() {
            self.last_error = Some(NO_RADIO.to_string());
            return Ok(());
        }
        let stop = self.radios.iter().any(|r| r.state == RadioState::Receiving);
        let mut started = Vec::new();
        let mut failed = Vec::new();
        for r in &mut self.radios {
            match (stop, r.state) {
                (true, RadioState::Receiving) => {
                    r.device.stop_receiving()?;
                    r.state = RadioState::Idle;
                    r.rssi = 0.0;
                }
                (false, RadioState::Idle) => match r.device.start_receiving(r.tuning.frequency) {
                    Ok(()) => {
                        r.state = RadioState::Receiving;
                        r.last_error = None;
                        started.push(format!("{} {}", r.name, format_mhz(r.tuning.frequency)));
                    }
                    Err(e) => {
                        r.last_error = Some(format!("{e:#}"));
                        failed.push(format!("{}: {e:#}", r.name));
                    }
                },
                _ => {}
            }
        }
        if stop {
            self.status_message = Some("Stopped receiving on every radio".to_string());
        } else if !started.is_empty() {
            self.status_message = Some(format!("Receiving: {}", started.join(" · ")));
        }
        if !failed.is_empty() {
            self.last_error = Some(failed.join(" · "));
        }
        Ok(())
    }

    /// Every radio's level, captures and stopped receivers (from [App::process_radio_events]).
    pub(super) fn poll_radios(&mut self) {
        for i in 0..self.radios.len() {
            let r = &mut self.radios[i];
            if let Some(src) = &r.rssi_source {
                r.rssi = f32::from_bits(src.load(Ordering::Relaxed));
            }
            let events: Vec<RadioEvent> = r.events.try_iter().collect();
            for event in events {
                match event {
                    RadioEvent::SignalCaptured(mut capture) => {
                        capture.radio = Some(self.radios[i].name.clone());
                        self.add_received_signal(capture, Some(i));
                    }
                    RadioEvent::Error(e) => self.receiver_stopped(i, e),
                    RadioEvent::StateChanged(state) => self.radios[i].state = state,
                }
            }
        }
    }

    /// Radio `i`'s receiver ended on its own (open or configure failure, unplug): it already
    /// stopped streaming, ended any recording and zeroed its level. Back to idle, so `r`
    /// restarts it (the header kept "RX" before).
    fn receiver_stopped(&mut self, i: usize, e: String) {
        let several = self.several_radios();
        let r = &mut self.radios[i];
        match r.state {
            RadioState::Receiving => r.state = RadioState::Idle,
            RadioState::Transmitting if self.pending_transmit_restore == Some((i, RadioState::Receiving)) => {
                self.pending_transmit_restore = Some((i, RadioState::Idle));
            }
            _ => {}
        }
        r.rssi = 0.0;
        r.last_error = Some(e.clone());
        self.last_error = Some(if several {
            format!("{} stopped receiving: {e} (select it with {}, then r restarts it)", r.name, i + 1)
        } else {
            format!("Receiver stopped: {e} (r restarts)")
        });
    }

    /// Note a transmission of radio `radio` (from `start` until now), for [Self::own_tx_of].
    pub(super) fn note_transmission(&mut self, radio: usize, frequency: u32, start: DateTime<Utc>) {
        let end = Utc::now();
        self.recent_tx.retain(|w| end - w.end <= Duration::seconds(OWN_TX_KEEP_S));
        self.recent_tx.push(TxWindow { radio, frequency, start, end });
    }

    /// The radio whose transmission a capture of radio `receiver` is, by its name, if it is KAT's
    /// own (see [own_transmission]).
    pub(super) fn own_tx_of(&self, capture: &Capture, receiver: usize) -> Option<String> {
        let w = own_transmission(&self.recent_tx, receiver, capture.timestamp, capture.frequency)?;
        self.radios.get(w.radio).map(|r| r.name.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::storage::RadioSection;

    fn rtl(index: usize) -> FoundRadio {
        FoundRadio::rtlsdr(index, Some(format!("0000000{}", index + 1)), true)
    }

    #[test]
    fn radios_are_named_by_kind_and_keep_their_names() {
        let hackrf = FoundRadio::hackrf(Some("0000000000000000457863c82e5a1f3d".to_string()));
        let flipper = FoundRadio::flipper("/dev/ttyACM0".to_string(), None);
        assert_eq!(radio_names(&[hackrf.clone(), rtl(0), flipper.clone()], &[]), ["HackRF", "RTL-SDR", "Flipper"]);
        assert_eq!(radio_names(&[rtl(0), rtl(1), hackrf], &[]), ["RTL-SDR", "RTL-SDR 2", "HackRF"]);
        // Found later: numbered after the ones KAT has.
        assert_eq!(radio_names(&[rtl(2), flipper], &[RadioKind::RtlSdr, RadioKind::RtlSdr]), ["RTL-SDR 3", "Flipper"]);
    }

    #[test]
    fn the_transmitter_is_the_configured_one_else_a_hackrf_else_a_flipper() {
        let h1 = FoundRadio::hackrf(Some("0000000000000000457863c800000001".to_string()));
        let h2 = FoundRadio::hackrf(Some("0000000000000000457863c800000002".to_string()));
        let f = FoundRadio::flipper("/dev/ttyACM0".to_string(), Some("flip_Ab1cD2".to_string()));
        let r = rtl(0);
        assert_eq!(choose_tx(&[&r, &f, &h1, &h2], None), Some(2), "a HackRF first");
        assert_eq!(choose_tx(&[&r, &f], None), Some(1));
        assert_eq!(choose_tx(&[&r], None), None, "an RTL-SDR cannot");
        let pick = |kind, serial: Option<&str>| RadioPick { kind, serial: serial.map(str::to_string) };
        assert_eq!(choose_tx(&[&h1, &f], Some(&pick(RadioKind::Flipper, None))), Some(1));
        assert_eq!(choose_tx(&[&h1, &h2], Some(&pick(RadioKind::HackRf, Some("00000002")))), Some(1));
        assert_eq!(choose_tx(&[&h1, &r], Some(&pick(RadioKind::Flipper, None))), Some(0), "not plugged in: auto");
    }

    #[test]
    fn sections_tune_kinds_then_single_radios() {
        let mut config = Config { default_frequency: 433_920_000, ..Config::default() };
        let section = |kind, serial: Option<&str>, frequency, lna| RadioSection {
            kind,
            serial: serial.map(str::to_string),
            frequency,
            lna_gain: lna,
            vga_gain: None,
            amp: None,
        };
        // The serial section is written first: the more specific one still wins.
        config.radio_sections = vec![
            section(RadioKind::RtlSdr, Some("00000002"), Some(868_350_000), None),
            section(RadioKind::RtlSdr, None, Some(315_000_000), Some(40)),
        ];
        let first = Tuning::for_radio(&rtl(0), &config);
        let second = Tuning::for_radio(&rtl(1), &config);
        assert_eq!((first.frequency, first.lna_gain), (315_000_000, 40));
        assert_eq!((second.frequency, second.lna_gain), (868_350_000, 40));
        let hackrf = Tuning::for_radio(&FoundRadio::hackrf(None), &config);
        assert_eq!(hackrf, Tuning::defaults(&config));
    }

    fn three_radios() -> Vec<Radio> {
        let config = Config::default();
        let t = |frequency| Tuning { frequency, lna_gain: 24, vga_gain: 20, amp: false };
        let mut hackrf = Radio::new(
            FoundRadio::hackrf(Some("0000000000000000457863c82e5a1f3d".to_string())),
            "HackRF".to_string(),
            t(433_920_000),
            &config,
        );
        hackrf.state = RadioState::Receiving;
        let rtl = Radio::new(FoundRadio::rtlsdr(0, Some("00000001".to_string()), false), "RTL-SDR".to_string(), t(315_000_000), &config);
        let flipper = Radio::new(FoundRadio::flipper("/dev/ttyACM0".to_string(), None), "Flipper".to_string(), t(868_350_000), &config);
        vec![hackrf, rtl, flipper]
    }

    /// `:radios` lists each radio (what it does, its serial or port, its settings), the
    /// frequencies of each kind and why other kinds are not in use, within the help's 62 columns.
    #[test]
    fn the_radios_overlay_says_what_each_radio_does() {
        let radios = three_radios();
        let text = overview_text(&radios, Some(1), Some(0), &["Flipper Zero: not found".to_string()], None);
        let lines: Vec<&str> = text.lines().collect();
        assert!(lines.iter().all(|l| l.chars().count() <= 62), "{text}");
        let line = |start: &str| lines.iter().find(|l| l.starts_with(start)).copied().unwrap_or_else(|| panic!("{start:?} in\n{text}"));
        assert!(line(" 1  HackRF").contains("RX 433.92 MHz") && line(" 1  HackRF").ends_with("RX+TX, transmits"), "{text}");
        assert!(line("▸2  RTL-SDR").contains("idle, 315.00 MHz") && line("▸2  RTL-SDR").ends_with("RX only"), "{text}");
        assert!(line(" 3  Flipper").ends_with("RX+TX"), "{text}");
        assert!(line("    …2e5a1f3d").contains("LNA:24 VGA:20 AMP:OFF"), "{text}");
        let rtl_settings = lines[lines.iter().position(|l| l.starts_with("▸2  RTL-SDR")).expect("the RTL-SDR") + 1];
        assert!(rtl_settings.ends_with("Gain:20") && !rtl_settings.contains("LNA"), "its one gain: {text}");
        assert!(line("    ttyACM0").contains("Preset: firmware's AM · not connected yet"), "{text}");
        assert!(text.contains("rtl_eeprom -s"), "a serial two dongles share: {text}");
        assert!(text.contains("  Flipper  300–348, 387–464, 779–928 MHz"), "{text}");
        assert!(text.contains("NOT IN USE\n  Flipper Zero: not found"), "{text}");
    }

    /// The settings bar offers a radio what it has, and `:lna` / `:vga` / `:amp` say why they set
    /// nothing on a radio without that setting (they reported success on every radio).
    #[test]
    fn each_kind_of_radio_has_its_own_settings() {
        use SettingsField::{Amp, Gain, Hop, Lna, Vga};
        assert_eq!(kind_settings(Some(RadioKind::HackRf)), [Hop, Lna, Vga, Amp]);
        assert_eq!(kind_settings(None), kind_settings(Some(RadioKind::HackRf)), "kept for a radio plugged in later");
        assert_eq!(kind_settings(Some(RadioKind::RtlSdr)), [Hop, Gain]);
        assert!(kind_settings(Some(RadioKind::Flipper)).is_empty(), "a Flipper has its frequency, and no hopping");

        for field in [Lna, Vga, Amp] {
            assert_eq!(missing_setting(RadioKind::HackRf, "HackRF", field), None);
            assert_eq!(
                missing_setting(RadioKind::Flipper, "Flipper", field).as_deref(),
                Some("Flipper has no gain settings (its receive preset sets them)")
            );
        }
        assert_eq!(missing_setting(RadioKind::RtlSdr, "RTL-SDR 2", Vga), None, "its one gain");
        assert_eq!(missing_setting(RadioKind::RtlSdr, "RTL-SDR 2", Gain), None);
        assert_eq!(
            missing_setting(RadioKind::RtlSdr, "RTL-SDR 2", Lna).as_deref(),
            Some("RTL-SDR 2 has one gain: :vga <0-62> sets it")
        );
        assert_eq!(
            missing_setting(RadioKind::RtlSdr, "RTL-SDR 2", Amp).as_deref(),
            Some("RTL-SDR 2 has no amplifier (its one gain: :vga <0-62>)")
        );
        // Every setting a kind is offered is one it has.
        for kind in RadioKind::ALL {
            for &field in kind_settings(Some(kind)) {
                assert_eq!(missing_setting(kind, "radio", field), None, "{kind:?} {field:?}");
            }
        }
    }

    /// A capture heard by another radio while one transmitted near its frequency (until shortly
    /// after) is KAT's own; the transmitter's own captures, other bands and later ones are not.
    #[test]
    fn own_transmissions_are_told_from_other_signals() {
        let t0 = Utc::now();
        let ms = Duration::milliseconds;
        let windows = [TxWindow { radio: 0, frequency: 433_920_000, start: t0, end: t0 + ms(800) }];
        assert!(own_transmission(&windows, 1, t0 + ms(500), 433_920_000).is_some(), "during it");
        assert!(own_transmission(&windows, 1, t0 + ms(2_800), 434_420_000).is_some(), "reported 2 s after it ended, 500 kHz off");
        assert!(own_transmission(&windows, 0, t0 + ms(500), 433_920_000).is_none(), "the transmitter's own receiver");
        assert!(own_transmission(&windows, 1, t0 + ms(500), 315_000_000).is_none(), "another band");
        assert!(own_transmission(&windows, 1, t0 - ms(100), 433_920_000).is_none(), "before it");
        assert!(own_transmission(&windows, 1, t0 + ms(4_000), 433_920_000).is_none(), "long after it");
    }
}
