//! Storage management for configuration and exports.
//!
//! All application data lives under `~/.config/KAT/`. The protocol keys are embedded in the
//! binary (see [crate::protocols::keys]); the user's own key files sit next to `config.ini`.
//!
//! ```text
//! ~/.config/KAT/
//!   config.ini          — User configuration
//!   kat.log             — Log of the last run
//!   session.json        — Autosaved captures of the last session (0600; see crate::session)
//!   keeloq_mfcodes_user — The user's KeeLoq manufacturer keys (see crate::keystore)
//!   nice_flor_s.bin, alutech_at_4n.bin — Optional tables for those protocols
//!   exports/            — Exported .fob / .sub files (save location)
//!   import/             — Scanned at startup for .fob / .sub to import
//! ```
//!
//! Captures are autosaved to `session.json` (config `session_autosave`, default on) and offered
//! for restore at the next start; exported signals (.fob / .sub) persist independently.
//!
//! `config.ini` is read leniently: a value KAT cannot use (or has to fit to the radio's range),
//! an unknown key, or an export/import folder that cannot be created becomes a line in
//! [Storage::warnings] and the default is used; the app shows them at startup.

use anyhow::{Context, Result};
use configparser::ini::Ini;
use std::fs;
use std::path::PathBuf;

// ─── Config ──────────────────────────────────────────────────────────────────

/// Application configuration loaded from `~/.config/KAT/config.ini`
#[derive(Debug, Clone)]
pub struct Config {
    // [general]
    /// Directory for exporting signals (.fob / .sub files)
    pub export_directory: PathBuf,
    /// Directory scanned at startup for .fob and .sub files to import (separate from export)
    pub import_directory: PathBuf,
    /// Maximum captures kept in the saved session (the newest; 0 = no limit)
    pub max_captures: usize,
    /// Unknown (undecoded) signals are listed too; off: only decoded ones (key `show_unknown_signals`,
    /// formerly `research_mode`).
    pub show_unknown_signals: bool,
    /// A decoded signal already in the list is not added again. Off: every press is listed, and only
    /// a repeat within 500 ms of the same signal (one press still on the air) is merged.
    pub ignore_duplicates: bool,
    /// Brute-force the key of every capture a search can recover (undecrypted PSA frames) without
    /// being asked, in the background (`autobf`).
    pub autobf: bool,

    // [radio]
    /// Default frequency in Hz
    pub default_frequency: u32,
    /// Frequency hopping list in Hz (`:hop on` / the Hop setting); default: Flipper-ARF's list
    pub hop_frequencies: Vec<u32>,
    /// Time (ms) spent on a quiet frequency before hopping to the next one
    pub hop_dwell_ms: u32,
    /// Default LNA gain (0-40 dB, 8 dB steps)
    pub default_lna_gain: u32,
    /// Default VGA gain (0-62 dB, 2 dB steps)
    pub default_vga_gain: u32,
    /// Default amplifier state
    pub default_amp: bool,
    /// TX VGA gain in dB (0-47) for every transmission
    pub tx_vga_gain: u32,
    /// TX RF amplifier (about +11 dB) for every transmission
    pub tx_amp: bool,
    /// Which radio KAT opens (`device`): auto (a HackRF, else an RTL-SDR, else a Flipper Zero), or one.
    pub radio_device: RadioChoice,
    /// A Flipper Zero's console port (`flipper_port`); None: found by its USB name.
    pub flipper_port: Option<String>,
    /// The receive preset of doctor-firmware's `rx_raw` (`flipper_preset`, e.g. FM476); None: the
    /// firmware's own (OOK).
    pub flipper_preset: Option<String>,

    // [export]
    /// Default export format (fob or sub)
    pub default_export_format: String,
    /// Include raw level/duration pairs in exports
    pub include_raw_pairs: bool,
    // [session]
    /// Autosave all captures to `session.json` and offer to restore them at the next start.
    pub session_autosave: bool,
}

impl Config {
    /// Build the default config, using the given config_dir as the base.
    /// This keeps everything under `~/.config/KAT/` by default.
    fn default_for(config_dir: &std::path::Path) -> Self {
        Self {
            export_directory: config_dir.join("exports"),
            import_directory: config_dir.join("import"),
            max_captures: 100,
            show_unknown_signals: true, // researchers need to see them
            ignore_duplicates: true,
            autobf: true,
            default_frequency: 433_920_000,
            hop_frequencies: crate::radio::hopper::DEFAULT_HOP_FREQUENCIES.to_vec(),
            hop_dwell_ms: crate::radio::hopper::DEFAULT_DWELL_MS,
            default_lna_gain: 24,
            default_vga_gain: 20,
            default_amp: false,
            // KAT ≤ 1.4 always transmitted at maximum power.
            tx_vga_gain: 47,
            tx_amp: true,
            radio_device: RadioChoice::Auto,
            flipper_port: None,
            flipper_preset: None,
            default_export_format: "fob".to_string(),
            include_raw_pairs: true,
            session_autosave: true,
        }
    }

    /// Load config from an INI file, falling back to defaults for missing or unusable keys.
    /// Returns the config and one warning per key that was ignored or adjusted (shown at startup
    /// and logged), or for a file that cannot be read at all.
    fn load_from_ini(path: &std::path::Path, config_dir: &std::path::Path) -> (Self, Vec<String>) {
        match fs::read_to_string(path) {
            Ok(text) => Self::parse_ini(&text, config_dir),
            Err(e) => (
                Config::default_for(config_dir),
                vec![format!("config.ini could not be read ({e}); using the default settings")],
            ),
        }
    }

    /// [Self::load_from_ini] for the text of the file.
    fn parse_ini(text: &str, config_dir: &std::path::Path) -> (Self, Vec<String>) {
        let defaults = Config::default_for(config_dir);
        let mut ini = Ini::new();
        if let Err(e) = ini.read(text.to_string()) {
            return (defaults, vec![format!("config.ini is not valid ({e}); using the default settings")]);
        }
        let mut r = IniReader { ini: &ini, warnings: Vec::new() };
        r.check_known_keys();

        let export_directory = r
            .text("general", "export_directory")
            .map(|s| expand_tilde(&s))
            .unwrap_or(defaults.export_directory);
        let import_directory = r
            .text("general", "import_directory")
            .map(|s| expand_tilde(&s))
            .unwrap_or(defaults.import_directory);
        let max_captures = r.uint("general", "max_captures", defaults.max_captures as u64) as usize;
        // `research_mode` is the old name of `show_unknown_signals`, read when the new key is absent.
        let legacy_unknown = r.bool("general", "research_mode", defaults.show_unknown_signals);
        let show_unknown_signals = r.bool("general", "show_unknown_signals", legacy_unknown);
        let ignore_duplicates = r.bool("general", "ignore_duplicates", defaults.ignore_duplicates);
        let autobf = r.bool("general", "autobf", defaults.autobf);

        let default_frequency = r.frequency("radio", "default_frequency", defaults.default_frequency);
        let hop_frequencies = r.hop_list("radio", "hop_frequencies", defaults.hop_frequencies);
        let hop_dwell_ms = r.dwell("radio", "hop_dwell_ms", defaults.hop_dwell_ms);
        let default_lna_gain = r.gain("radio", "default_lna_gain", defaults.default_lna_gain, 40, 8);
        let default_vga_gain = r.gain("radio", "default_vga_gain", defaults.default_vga_gain, 62, 2);
        let default_amp = r.bool("radio", "default_amp", defaults.default_amp);
        let tx_vga_gain = r.gain("radio", "tx_vga_gain", defaults.tx_vga_gain, 47, 1);
        let tx_amp = r.bool("radio", "tx_amp", defaults.tx_amp);
        let radio_device = r.get("radio", "device", defaults.radio_device, |d| d.name().to_string(), RadioChoice::parse);
        let flipper_port = r.text("radio", "flipper_port");
        let flipper_preset = r.text("radio", "flipper_preset");

        let default_export_format = match r.text("export", "default_format") {
            Some(f) if f.eq_ignore_ascii_case("fob") || f.eq_ignore_ascii_case("sub") => f.to_ascii_lowercase(),
            Some(f) => {
                r.warn(format!(
                    "default_format = {f} is not fob or sub; using {}",
                    defaults.default_export_format
                ));
                defaults.default_export_format
            }
            None => defaults.default_export_format,
        };
        let include_raw_pairs = r.bool("export", "include_raw_pairs", defaults.include_raw_pairs);

        // [session] (also accepted under [general], where users may look for it first)
        let session_autosave = if r.has("session", "session_autosave") {
            r.bool("session", "session_autosave", defaults.session_autosave)
        } else {
            r.bool("general", "session_autosave", defaults.session_autosave)
        };

        let config = Self {
            export_directory,
            import_directory,
            max_captures,
            show_unknown_signals,
            ignore_duplicates,
            autobf,
            default_frequency,
            hop_frequencies,
            hop_dwell_ms,
            default_lna_gain,
            default_vga_gain,
            default_amp,
            tx_vga_gain,
            tx_amp,
            radio_device,
            flipper_port,
            flipper_preset,
            default_export_format,
            include_raw_pairs,
            session_autosave,
        };
        (config, r.warnings)
    }

    /// Save config to an INI-style file with comments explaining each field.
    fn save_to_ini(&self, path: &std::path::Path) -> Result<()> {
        let export_str = self.export_directory.to_string_lossy();
        let import_str = self.import_directory.to_string_lossy();
        let freq_mhz = self.default_frequency as f64 / 1_000_000.0;

        let content = format!(
            r#"; KAT — Keyfob Analysis Toolkit configuration
; Location: {path}
;
; Edit this file to change default settings. Switches take true/false (or yes/no, on/off, 1/0).
; A value KAT cannot use is reported at startup (status line and kat.log) and replaced by the
; default. Lines starting with ; or # are comments.
;
; The protocol keys are built into KAT. This folder also holds your own KeeLoq manufacturer
; keys (keeloq_mfcodes_user, Flipper format; recovered keys are added there) and the optional
; Nice FloR-S / Alutech AT-4N tables (nice_flor_s.bin, alutech_at_4n.bin).

[general]
; Directory where .fob and .sub exports are saved.
; Supports ~ for home directory.
export_directory = {export_dir}

; Directory scanned at startup for .fob and .sub files to import (not used for saving).
; Supports ~ for home directory.
import_directory = {import_dir}

; Maximum number of captures kept in the saved session (the newest ones;
; 0 = no limit). See [session] below.
max_captures = {max_captures}

; List unknown (undecoded) signals too, for timing analysis and research (true), or only the
; signals a protocol decodes (false). Older versions called this research_mode.
show_unknown_signals = {show_unknown_signals}

; A decoded signal that is already in the list is not added again (true). With false, every
; press is listed, as a Flipper's receive history does: only a repeat of the same signal within
; 500 ms (one press still on the air) is merged.
ignore_duplicates = {ignore_duplicates}

; Recover the key of every capture a brute force can crack, without being asked: PSA frames
; encrypted with TEA get ProtoPirate's BF1/BF2 search in the background (well under a second
; each) and, when it finds the key, their serial, button and counter, so they can be exported
; and transmitted. false: only the signal menu's "Recover key" runs it.
autobf = {autobf}

[radio]
; Default receive frequency, in Hz or MHz ({freq_mhz:.2} MHz), 300-928 MHz.
; Common keyfob frequencies: 315000000, 433920000, 868350000 (or 315, 433.92, 868.35)
default_frequency = {frequency}

; Frequency hopping (:hop on, or the Hop setting): frequencies in MHz, and the time in ms spent
; on a quiet frequency before hopping on. KAT stays on a frequency while a signal is received.
hop_frequencies = {hop_list}
hop_dwell_ms = {hop_dwell}

; Default LNA gain in dB (0, 8, 16, 24, 32, 40)
default_lna_gain = {lna}

; Default VGA gain in dB (0-62, even numbers)
default_vga_gain = {vga}

; Enable RF amplifier by default (true/false)
default_amp = {amp}

; Transmit power: TX VGA gain in dB (0-47) and the RF amplifier (true/false, about +11 dB).
; 47 + true is the HackRF's maximum; lower it for nearby targets. Also :txpower <0-47> [amp on|off].
tx_vga_gain = {tx_vga}
tx_amp = {tx_amp}

; Which radio KAT opens: auto (a HackRF, else an RTL-SDR, else a Flipper Zero on USB), hackrf,
; rtlsdr or flipper.
device = {device}

; A Flipper Zero is used through its USB console (transmit and receive, docs/flipper_radio.md).
; flipper_port: its serial port, when KAT does not find it (/dev/ttyACM0, /dev/cu.usbmodemflip_Name1,
; COM5). flipper_preset: the receive preset of doctor-firmware's rx_raw (AM270, AM650, FM238,
; FM476, ...); the other firmwares receive with their own AM preset only.
{flipper_port}
{flipper_preset}

[export]
; Default export format: fob (JSON metadata) or sub (Flipper Zero)
default_format = {export_fmt}

; Include raw signal level/duration pairs in .fob exports.
; Enables signal replay but increases file size.
include_raw_pairs = {raw_pairs}

[session]
; Autosave every capture (raw pairs and metadata included) to session.json in this
; directory and offer to restore them at the next start. The file holds rolling codes:
; it is written with owner-only permissions (0600). :clear deletes it.
session_autosave = {session_autosave}
"#,
            path = path.display(),
            export_dir = export_str,
            import_dir = import_str,
            max_captures = self.max_captures,
            show_unknown_signals = self.show_unknown_signals,
            ignore_duplicates = self.ignore_duplicates,
            autobf = self.autobf,
            freq_mhz = freq_mhz,
            frequency = self.default_frequency,
            hop_list = self
                .hop_frequencies
                .iter()
                .map(|&f| crate::radio::hopper::format_mhz(f))
                .collect::<Vec<_>>()
                .join(", "),
            hop_dwell = self.hop_dwell_ms,
            lna = self.default_lna_gain,
            vga = self.default_vga_gain,
            amp = self.default_amp,
            tx_vga = self.tx_vga_gain,
            tx_amp = self.tx_amp,
            device = self.radio_device.name(),
            flipper_port = match &self.flipper_port {
                Some(p) => format!("flipper_port = {p}"),
                None => "; flipper_port = /dev/ttyACM0".to_string(),
            },
            flipper_preset = match &self.flipper_preset {
                Some(p) => format!("flipper_preset = {p}"),
                None => "; flipper_preset = FM476".to_string(),
            },
            export_fmt = self.default_export_format,
            raw_pairs = self.include_raw_pairs,
            session_autosave = self.session_autosave,
        );

        fs::write(path, content)
            .with_context(|| format!("Failed to write config to {:?}", path))?;

        Ok(())
    }
}

/// Which radio KAT opens (`[radio] device`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RadioChoice {
    /// A HackRF, else an RTL-SDR, else a Flipper Zero.
    Auto,
    HackRf,
    RtlSdr,
    Flipper,
}

impl RadioChoice {
    fn parse(value: &str) -> Result<Self, String> {
        match value.to_ascii_lowercase().replace(['-', '_', ' '], "").as_str() {
            "auto" => Ok(Self::Auto),
            "hackrf" => Ok(Self::HackRf),
            "rtlsdr" => Ok(Self::RtlSdr),
            "flipper" | "flipperzero" => Ok(Self::Flipper),
            _ => Err("is not auto, hackrf, rtlsdr or flipper".to_string()),
        }
    }

    /// As `config.ini` writes it.
    pub fn name(self) -> &'static str {
        match self {
            Self::Auto => "auto",
            Self::HackRf => "hackrf",
            Self::RtlSdr => "rtlsdr",
            Self::Flipper => "flipper",
        }
    }
}

/// The settings KAT reads, per section (`session_autosave` is also read under `[general]`).
const KNOWN_KEYS: &[(&str, &[&str])] = &[
    (
        "general",
        &[
            "export_directory",
            "import_directory",
            "max_captures",
            "show_unknown_signals",
            "research_mode",
            "ignore_duplicates",
            "autobf",
            "session_autosave",
        ],
    ),
    (
        "radio",
        &[
            "default_frequency",
            "hop_frequencies",
            "hop_dwell_ms",
            "default_lna_gain",
            "default_vga_gain",
            "default_amp",
            "tx_vga_gain",
            "tx_amp",
            "device",
            "flipper_port",
            "flipper_preset",
        ],
    ),
    ("export", &["default_format", "include_raw_pairs"]),
    ("session", &["session_autosave"]),
];

/// Receive frequencies `default_frequency` may name: the range `:freq` accepts.
const FREQUENCY_RANGE: std::ops::RangeInclusive<u32> = 300_000_000..=928_000_000;

/// `config.ini` switches: true/false, yes/no, on/off or 1/0, in any case.
fn parse_switch(value: &str) -> Result<bool, String> {
    match value.to_ascii_lowercase().as_str() {
        "true" | "yes" | "on" | "1" => Ok(true),
        "false" | "no" | "off" | "0" => Ok(false),
        _ => Err("is not true/false (or yes/no, on/off, 1/0)".to_string()),
    }
}

/// Reads `config.ini` values for [Config::parse_ini]. A value KAT cannot use, or has to fit to
/// the radio's range, adds a warning that names the key, the value and what KAT uses instead:
/// an unusable value used to fall back silently (`tx_amp = off` kept the amplifier on).
struct IniReader<'a> {
    ini: &'a Ini,
    warnings: Vec<String>,
}

impl IniReader<'_> {
    fn warn(&mut self, what: String) {
        self.warnings.push(format!("config.ini: {what}"));
    }

    fn has(&self, section: &str, key: &str) -> bool {
        self.ini.get_map_ref().get(section).is_some_and(|s| s.contains_key(key))
    }

    /// Report sections and keys KAT does not read (a typo such as `default_frequncy` would
    /// otherwise leave the setting at its default without a word).
    fn check_known_keys(&mut self) {
        let mut unknown = Vec::new();
        for (section, keys) in self.ini.get_map_ref() {
            match KNOWN_KEYS.iter().find(|(name, _)| name == section) {
                Some((_, known)) => {
                    for key in keys.keys().filter(|k| !known.contains(&k.as_str())) {
                        unknown.push(format!("[{section}] {key} is not a KAT setting; ignored"));
                    }
                }
                // configparser files keys above the first [section] under "default".
                None if section == "default" => {
                    for key in keys.keys() {
                        unknown.push(format!("{key} is not under a [section]; ignored"));
                    }
                }
                None => unknown.push(format!("[{section}] is not a KAT section; its settings are ignored")),
            }
        }
        unknown.sort();
        for w in unknown {
            self.warn(w);
        }
    }

    /// `[section] key` through `parse` (Err: why the value is unusable). Missing → `default`;
    /// empty or unusable → a warning and `default` (`shown` writes it as the user would).
    fn get<T>(
        &mut self,
        section: &str,
        key: &str,
        default: T,
        shown: impl Fn(&T) -> String,
        parse: impl Fn(&str) -> Result<T, String>,
    ) -> T {
        let Some(value) = self.ini.get_map_ref().get(section).and_then(|s| s.get(key)) else {
            return default;
        };
        let value = value.as_deref().map(str::trim).unwrap_or("");
        if value.is_empty() {
            self.warn(format!("{key} has no value; using {}", shown(&default)));
            return default;
        }
        match parse(value) {
            Ok(v) => v,
            Err(why) => {
                self.warn(format!("{key} = {value} {why}; using {}", shown(&default)));
                default
            }
        }
    }

    fn text(&mut self, section: &str, key: &str) -> Option<String> {
        self.get(section, key, None, |_| "the default".to_string(), |v| Ok(Some(v.to_string())))
    }

    fn bool(&mut self, section: &str, key: &str, default: bool) -> bool {
        self.get(section, key, default, |d| d.to_string(), parse_switch)
    }

    fn uint(&mut self, section: &str, key: &str, default: u64) -> u64 {
        self.get(section, key, default, |d| d.to_string(), |v| {
            v.parse().map_err(|_| "is not a whole number".to_string())
        })
    }

    /// MHz (433.92) or Hz (433920000), as `--freq` and `:iq` take it; 300–928 MHz like `:freq`.
    fn frequency(&mut self, section: &str, key: &str, default: u32) -> u32 {
        let mhz = |hz: u32| crate::radio::hopper::format_mhz(hz);
        self.get(section, key, default, |&d| format!("{} MHz", mhz(d)), |v| {
            let hz = crate::radio::iq_file::parse_frequency_arg(v)
                .ok_or_else(|| "is not a frequency (MHz like 433.92, or Hz like 433920000)".to_string())?;
            if FREQUENCY_RANGE.contains(&hz) {
                Ok(hz)
            } else {
                Err(format!(
                    "is outside {}-{} MHz",
                    FREQUENCY_RANGE.start() / 1_000_000,
                    FREQUENCY_RANGE.end() / 1_000_000
                ))
            }
        })
    }

    /// A gain in dB, fitted to 0..=`max` in `step` dB steps (rounded down, as `:lna` / `:vga`
    /// do): a gain the radio rejects made every RX start fail ("Failed to set RXVGA gain").
    fn gain(&mut self, section: &str, key: &str, default: u32, max: u32, step: u32) -> u32 {
        let raw = self.get(section, key, default, |d| d.to_string(), |v| {
            v.parse::<u32>().map_err(|_| "is not a number of dB".to_string())
        });
        let fitted = raw.min(max) / step * step;
        if fitted != raw {
            let why = match step {
                _ if raw > max => format!("is above {max} dB"),
                2 => "is not an even number of dB".to_string(),
                _ => format!("is not a multiple of {step} dB"),
            };
            self.warn(format!("{key} = {raw} {why}; using {fitted}"));
        }
        fitted
    }

    /// The `:hop` list: two or more frequencies.
    fn hop_list(&mut self, section: &str, key: &str, default: Vec<u32>) -> Vec<u32> {
        self.get(section, key, default, |_| "the default list".to_string(), |v| {
            match crate::radio::hopper::parse_frequency_list(v) {
                Ok(f) if f.len() >= 2 => Ok(f),
                Ok(_) => Err("needs at least two frequencies".to_string()),
                Err(e) => Err(format!("cannot be used ({e})")),
            }
        })
    }

    /// Hop dwell: 20 ms to 10 s, the range the hopper keeps to.
    fn dwell(&mut self, section: &str, key: &str, default: u32) -> u32 {
        let raw = self.uint(section, key, u64::from(default));
        let fitted = raw.clamp(20, 10_000) as u32;
        if u64::from(fitted) != raw {
            self.warn(format!("{key} = {raw} is outside 20-10000 ms; using {fitted}"));
        }
        fitted
    }
}

/// Fallback Default (without knowing config_dir). Only used if something goes
/// very wrong and we need a Config without a Storage instance.
impl Default for Config {
    fn default() -> Self {
        let fallback = resolve_config_dir()
            .unwrap_or_else(|| PathBuf::from(".").join("KAT"));
        Config::default_for(&fallback)
    }
}

/// Expand `~` at the start of a path to the user's home directory.
fn expand_tilde(s: &str) -> PathBuf {
    if let Some(rest) = s.strip_prefix("~/") {
        if let Some(home) = dirs::home_dir() {
            return home.join(rest);
        }
    }
    PathBuf::from(s)
}

/// Resolve the KAT config directory to `~/.config/KAT/` regardless of OS.
/// The `autobf` setting of `config.ini`, read without creating anything (`kat --decode`); the
/// default when there is no config file.
pub fn autobf_setting() -> bool {
    let Some(dir) = resolve_config_dir() else {
        return Config::default_for(std::path::Path::new(".")).autobf;
    };
    match fs::read_to_string(dir.join("config.ini")) {
        Ok(text) => Config::parse_ini(&text, &dir).0.autobf,
        Err(_) => Config::default_for(&dir).autobf,
    }
}

pub fn resolve_config_dir() -> Option<PathBuf> {
    dirs::home_dir().map(|home| home.join(".config").join("KAT"))
}

// ─── Storage ─────────────────────────────────────────────────────────────────

/// Storage manager for configuration and exports.
///
/// On construction it ensures the directory tree exists:
///
/// ```text
/// ~/.config/KAT/
///   config.ini
///   exports/
/// ```
///
/// Captures live in memory and are autosaved to `session.json` (see [crate::session]).
pub struct Storage {
    /// Base config directory (~/.config/KAT)
    config_dir: PathBuf,
    /// Configuration
    pub config: Config,
    /// What was wrong with `config.ini` or its folders: one line per ignored or adjusted key, or
    /// folder KAT could not use (already logged; the app shows them at startup).
    pub warnings: Vec<String>,
}

/// Make sure `dir` is a usable folder (created if missing).
fn ensure_dir(dir: &std::path::Path) -> std::io::Result<()> {
    if dir.as_os_str().is_empty() {
        return Err(std::io::Error::new(std::io::ErrorKind::InvalidInput, "empty path"));
    }
    if dir.exists() && !dir.is_dir() {
        return Err(std::io::Error::new(std::io::ErrorKind::AlreadyExists, "a file has that name"));
    }
    if !dir.exists() {
        fs::create_dir_all(dir)?;
        tracing::info!("Created directory: {:?}", dir);
    }
    Ok(())
}

/// The configured export or import folder (`key`), or the default one when it cannot be created
/// (an unplugged SD card, a read-only path): a warning, not a reason to refuse to start.
fn usable_dir(key: &str, configured: PathBuf, default: PathBuf, warnings: &mut Vec<String>) -> PathBuf {
    let Err(e) = ensure_dir(&configured) else {
        return configured;
    };
    if configured == default {
        warnings.push(format!("Cannot create {} ({e}); exports and imports there will fail", configured.display()));
        return configured;
    }
    warnings.push(format!(
        "config.ini: {key} {} cannot be used ({e}); using {} instead",
        configured.display(),
        default.display()
    ));
    if let Err(e) = ensure_dir(&default) {
        warnings.push(format!("Cannot create {} ({e}); exports and imports there will fail", default.display()));
    }
    default
}

impl Storage {
    /// Create a new storage manager.
    ///
    /// 1. Resolves the config directory (`~/.config/KAT`).
    /// 2. Creates it if missing.
    /// 3. Loads `config.ini` if it exists, otherwise writes a default one.
    /// 4. Creates the export and import directories if missing (falling back to the default ones).
    ///
    /// Only a missing home directory or an uncreatable config directory is an error; everything
    /// else ends up in [Self::warnings].
    pub fn new() -> Result<Self> {
        // ── 1. Resolve base path ─────────────────────────────────────────
        let config_dir = resolve_config_dir()
            .context("Could not determine home directory (is $HOME set?)")?;

        let config_path = config_dir.join("config.ini");

        // ── 2. Ensure directory tree exists ──────────────────────────────
        if !config_dir.exists() {
            fs::create_dir_all(&config_dir)
                .with_context(|| format!("Failed to create config dir: {:?}", config_dir))?;
            tracing::info!("Created config directory: {:?}", config_dir);
        }

        // ── 3. Load or create config.ini ─────────────────────────────────
        let (mut config, mut warnings) = if config_path.exists() {
            tracing::info!("Loading config from {:?}", config_path);
            Config::load_from_ini(&config_path, &config_dir)
        } else {
            tracing::info!(
                "No config.ini found — creating default at {:?}",
                config_path
            );
            let config = Config::default_for(&config_dir);
            if let Err(e) = config.save_to_ini(&config_path) {
                tracing::warn!("Could not write default config.ini: {}", e);
            }
            (config, Vec::new())
        };

        // ── 4. Ensure the export and import directories exist ────────────
        let defaults = Config::default_for(&config_dir);
        config.export_directory =
            usable_dir("export_directory", config.export_directory, defaults.export_directory, &mut warnings);
        config.import_directory =
            usable_dir("import_directory", config.import_directory, defaults.import_directory, &mut warnings);
        for w in &warnings {
            tracing::warn!("{w}");
        }

        // ── 5. Log resolved paths ───────────────────────────────────────
        tracing::info!("Config dir: {:?}", config_dir);
        tracing::info!("Export dir: {:?}", config.export_directory);
        tracing::info!("Import dir: {:?}", config.import_directory);

        Ok(Self {
            config_dir,
            config,
            warnings,
        })
    }

    /// Save the current configuration back to `config.ini`.
    #[allow(dead_code)]
    pub fn save_config(&self) -> Result<()> {
        let config_path = self.config_dir.join("config.ini");
        self.config.save_to_ini(&config_path)?;
        tracing::info!("Saved config to {:?}", config_path);
        Ok(())
    }

    // ─── Path accessors ──────────────────────────────────────────────────

    /// Get the config directory path (`~/.config/KAT`)
    #[allow(dead_code)]
    pub fn config_dir(&self) -> &PathBuf {
        &self.config_dir
    }

    /// Get the export directory path (from config, default `~/.config/KAT/exports`)
    pub fn export_dir(&self) -> &PathBuf {
        &self.config.export_directory
    }

    /// Get the import directory path (from config, default `~/.config/KAT/import`)
    pub fn import_dir(&self) -> &PathBuf {
        &self.config.import_directory
    }

}

#[cfg(test)]
mod tx_power_config_tests {
    use super::*;

    /// TX power keys: defaults keep KAT ≤ 1.4's full power, a 1.4 config.ini (without the keys)
    /// loads with them, values are clamped, and the saved template round-trips them.
    #[test]
    fn tx_power_config_loads_saves_and_defaults() {
        let dir = std::env::temp_dir().join(format!("kat_cfg_tx_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("config.ini");

        let defaults = Config::default_for(&dir);
        assert_eq!((defaults.tx_vga_gain, defaults.tx_amp), (47, true));

        std::fs::write(&path, "[radio]\ndefault_frequency = 315000000\ndefault_amp = false\n").unwrap();
        let (old, warnings) = Config::load_from_ini(&path, &dir);
        assert_eq!((old.default_frequency, old.tx_vga_gain, old.tx_amp), (315_000_000, 47, true));
        assert!(warnings.is_empty(), "{warnings:?}");

        std::fs::write(&path, "[radio]\ntx_vga_gain = 99\ntx_amp = false\n").unwrap();
        let (clamped, warnings) = Config::load_from_ini(&path, &dir);
        assert_eq!((clamped.tx_vga_gain, clamped.tx_amp), (47, false));
        assert_eq!(warnings, ["config.ini: tx_vga_gain = 99 is above 47 dB; using 47"]);

        let mut cfg = Config::default_for(&dir);
        cfg.tx_vga_gain = 20;
        cfg.tx_amp = false;
        cfg.save_to_ini(&path).unwrap();
        let (back, warnings) = Config::load_from_ini(&path, &dir);
        let _ = std::fs::remove_dir_all(&dir);
        assert_eq!((back.tx_vga_gain, back.tx_amp), (20, false));
        assert!(warnings.is_empty(), "{warnings:?}");
    }
}

#[cfg(test)]
mod config_warning_tests {
    use super::*;

    fn parse(text: &str) -> (Config, Vec<String>) {
        Config::parse_ini(text, &PathBuf::from("/home/u/.config/KAT"))
    }

    /// The template KAT writes loads back without a single warning.
    #[test]
    fn written_template_loads_cleanly() {
        let dir = std::env::temp_dir().join(format!("kat_cfg_tpl_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("config.ini");
        Config::default_for(&dir).save_to_ini(&path).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_dir_all(&dir);
        let (cfg, warnings) = Config::parse_ini(&text, &dir);
        assert!(warnings.is_empty(), "{warnings:?}");
        assert_eq!(cfg.default_frequency, 433_920_000);
        assert!(!text.contains("no keystore directory"), "stale comment");
        assert!(text.contains("keeloq_mfcodes_user") && text.contains("nice_flor_s.bin"));
    }

    /// `show_unknown_signals` and `ignore_duplicates` default to on; `research_mode`, the old name
    /// of `show_unknown_signals`, still works and gives way to the new key.
    #[test]
    fn unknown_signals_and_duplicates_switches() {
        let (cfg, warnings) = parse("[general]\nmax_captures = 100\n");
        assert_eq!((cfg.show_unknown_signals, cfg.ignore_duplicates), (true, true));
        assert!(warnings.is_empty(), "{warnings:?}");
        let (cfg, warnings) = parse("[general]\nshow_unknown_signals = false\nignore_duplicates = no\n");
        assert_eq!((cfg.show_unknown_signals, cfg.ignore_duplicates), (false, false));
        assert!(warnings.is_empty(), "{warnings:?}");
        let (cfg, warnings) = parse("[general]\nresearch_mode = false\n");
        assert!(!cfg.show_unknown_signals && warnings.is_empty(), "{warnings:?}");
        let (cfg, _) = parse("[general]\nresearch_mode = false\nshow_unknown_signals = true\n");
        assert!(cfg.show_unknown_signals, "the new key wins");
    }

    /// `device` picks the radio (auto by default); the Flipper's port and receive preset are read
    /// as written, and a written config with them set loads back as it was.
    #[test]
    fn radio_device_and_flipper_settings() {
        let (cfg, warnings) = parse("[radio]\ntx_amp = true\n");
        assert_eq!((cfg.radio_device, cfg.flipper_port.as_deref(), cfg.flipper_preset.as_deref()), (RadioChoice::Auto, None, None));
        assert!(warnings.is_empty(), "{warnings:?}");
        for (word, want) in [("flipper", RadioChoice::Flipper), ("Flipper Zero", RadioChoice::Flipper), ("RTL-SDR", RadioChoice::RtlSdr), ("hackrf", RadioChoice::HackRf)] {
            let (cfg, warnings) = parse(&format!("[radio]\ndevice = {word}\n"));
            assert_eq!(cfg.radio_device, want, "{word}");
            assert!(warnings.is_empty(), "{warnings:?}");
        }
        let (cfg, warnings) = parse("[radio]\ndevice = bladerf\n");
        assert_eq!(cfg.radio_device, RadioChoice::Auto);
        assert!(warnings.iter().any(|w| w.contains("device = bladerf")), "{warnings:?}");

        let dir = std::env::temp_dir().join(format!("kat_cfg_flipper_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("config.ini");
        let mut want = Config::default_for(&dir);
        want.radio_device = RadioChoice::Flipper;
        want.flipper_port = Some("/dev/ttyACM1".to_string());
        want.flipper_preset = Some("FM476".to_string());
        want.save_to_ini(&path).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_dir_all(&dir);
        let (cfg, warnings) = Config::parse_ini(&text, &dir);
        assert!(warnings.is_empty(), "{warnings:?}");
        assert_eq!(
            (cfg.radio_device, cfg.flipper_port.as_deref(), cfg.flipper_preset.as_deref()),
            (RadioChoice::Flipper, Some("/dev/ttyACM1"), Some("FM476"))
        );
    }

    /// `autobf` is on unless the config says otherwise.
    #[test]
    fn autobf_switch() {
        let (cfg, warnings) = parse("[general]\nmax_captures = 100\n");
        assert!(cfg.autobf && warnings.is_empty(), "{warnings:?}");
        let (cfg, warnings) = parse("[general]\nautobf = false\n");
        assert!(!cfg.autobf && warnings.is_empty(), "{warnings:?}");
    }

    /// Switches take true/false, yes/no, on/off and 1/0 in any case; anything else keeps the
    /// default with a warning (`tx_amp = off` used to leave the amplifier on silently).
    #[test]
    fn switches_accept_the_usual_words() {
        for (word, want) in [("off", false), ("No", false), ("0", false), ("FALSE", false), ("on", true), ("yes", true), ("1", true)] {
            let (cfg, warnings) = parse(&format!("[radio]\ntx_amp = {word}\n[session]\nsession_autosave = {word}\n"));
            assert_eq!((cfg.tx_amp, cfg.session_autosave), (want, want), "{word}");
            assert!(warnings.is_empty(), "{word}: {warnings:?}");
        }
        let (cfg, warnings) = parse("[radio]\ntx_amp = maybe\n[general]\nsession_autosave = no\n");
        assert!(cfg.tx_amp && !cfg.session_autosave, "[general] session_autosave still counts");
        assert_eq!(warnings, ["config.ini: tx_amp = maybe is not true/false (or yes/no, on/off, 1/0); using true"]);
    }

    /// `default_frequency` in MHz or Hz, within 300-928 MHz.
    #[test]
    fn default_frequency_takes_mhz_or_hz_in_range() {
        for (value, hz) in [("315", 315_000_000), ("315.0", 315_000_000), ("433920000", 433_920_000), ("868.35", 868_350_000)] {
            let (cfg, warnings) = parse(&format!("[radio]\ndefault_frequency = {value}\n"));
            assert_eq!(cfg.default_frequency, hz, "{value}");
            assert!(warnings.is_empty(), "{value}: {warnings:?}");
        }
        let (cfg, warnings) = parse("[radio]\ndefault_frequency = 1000\n");
        assert_eq!(cfg.default_frequency, 433_920_000);
        assert_eq!(warnings, ["config.ini: default_frequency = 1000 is outside 300-928 MHz; using 433.92 MHz"]);
        let (_, warnings) = parse("[radio]\ndefault_frequency = fast\n");
        assert!(warnings[0].contains("is not a frequency"), "{warnings:?}");
    }

    /// Gains are fitted to what the radio accepts (a VGA gain of 64 made every RX start fail).
    #[test]
    fn gains_are_fitted_to_the_radio() {
        let (cfg, warnings) = parse(
            "[radio]\ndefault_lna_gain = 30\ndefault_vga_gain = 64\ntx_vga_gain = 12\n",
        );
        assert_eq!((cfg.default_lna_gain, cfg.default_vga_gain, cfg.tx_vga_gain), (24, 62, 12));
        assert_eq!(
            warnings,
            [
                "config.ini: default_lna_gain = 30 is not a multiple of 8 dB; using 24",
                "config.ini: default_vga_gain = 64 is above 62 dB; using 62",
            ]
        );
        let (cfg, warnings) = parse("[radio]\ndefault_vga_gain = 21\ndefault_lna_gain = high\n");
        assert_eq!((cfg.default_vga_gain, cfg.default_lna_gain), (20, 24));
        assert_eq!(
            warnings,
            [
                "config.ini: default_lna_gain = high is not a number of dB; using 24",
                "config.ini: default_vga_gain = 21 is not an even number of dB; using 20",
            ]
        );
    }

    /// Unknown keys and sections, keys without a value and an unreadable file are all reported.
    #[test]
    fn ignored_keys_and_files_are_reported() {
        let (cfg, warnings) = parse(
            "stray = 1\n[radio]\ndefault_frequncy = 315\ntx_amp\nhop_frequencies = 433.92\nhop_dwell_ms = 5\n[rdio]\nx = 1\n[export]\ndefault_format = json\n",
        );
        assert_eq!(cfg.default_frequency, 433_920_000, "the typo is not read");
        assert_eq!(cfg.hop_dwell_ms, 20);
        assert_eq!(cfg.hop_frequencies, crate::radio::hopper::DEFAULT_HOP_FREQUENCIES.to_vec());
        assert_eq!(
            warnings,
            [
                "config.ini: [radio] default_frequncy is not a KAT setting; ignored",
                "config.ini: [rdio] is not a KAT section; its settings are ignored",
                "config.ini: stray is not under a [section]; ignored",
                "config.ini: hop_frequencies = 433.92 needs at least two frequencies; using the default list",
                "config.ini: hop_dwell_ms = 5 is outside 20-10000 ms; using 20",
                "config.ini: tx_amp has no value; using true",
                "config.ini: default_format = json is not fob or sub; using fob",
            ]
        );
        let (cfg, warnings) = parse("[radio\ntx_amp = false\n");
        assert!(cfg.tx_amp, "defaults");
        assert_eq!(warnings.len(), 1);
        assert!(warnings[0].starts_with("config.ini is not valid ("), "{warnings:?}");
    }

    /// An export or import folder that cannot be created falls back to the default one.
    #[test]
    fn unusable_folders_fall_back_to_the_default() {
        let base = std::env::temp_dir().join(format!("kat_cfg_dirs_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&base);
        std::fs::create_dir_all(&base).unwrap();
        let blocker = base.join("not_a_folder");
        std::fs::write(&blocker, "x").unwrap();
        let mut warnings = Vec::new();
        let got = usable_dir("export_directory", blocker.join("exports"), base.join("exports"), &mut warnings);
        assert_eq!(got, base.join("exports"));
        assert!(got.is_dir());
        assert_eq!(warnings.len(), 1);
        assert!(warnings[0].starts_with("config.ini: export_directory ") && warnings[0].ends_with(&format!("using {} instead", got.display())), "{warnings:?}");
        let mut warnings = Vec::new();
        assert_eq!(usable_dir("import_directory", blocker.clone(), base.join("import"), &mut warnings), base.join("import"));
        assert!(warnings[0].contains("a file has that name"), "{warnings:?}");
        let mut warnings = Vec::new();
        assert_eq!(usable_dir("import_directory", base.join("import"), base.join("import"), &mut warnings), base.join("import"));
        assert!(warnings.is_empty());
        let _ = std::fs::remove_dir_all(&base);
    }
}
