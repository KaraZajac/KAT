//! Storage management for configuration and exports.
//!
//! All application data lives under `~/.config/KAT/`. **No keystore directory is created**
//! — keys are embedded in the binary (see [crate::protocols::keys] and [crate::keystore]).
//!
//! ```text
//! ~/.config/KAT/
//!   config.ini          — User configuration
//!   session.json        — Autosaved captures of the last session (0600; see crate::session)
//!   exports/            — Exported .fob / .sub files (save location)
//!   import/             — Scanned at startup for .fob / .sub to import
//! ```
//!
//! Captures are autosaved to `session.json` (config `session_autosave`, default on) and offered
//! for restore at the next start; exported signals (.fob / .sub) persist independently.

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
    /// If off, only successfully decoded signals are added to the list. If on, unknown signals are also shown.
    pub research_mode: bool,

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
    fn default_for(config_dir: &PathBuf) -> Self {
        Self {
            export_directory: config_dir.join("exports"),
            import_directory: config_dir.join("import"),
            max_captures: 100,
            research_mode: true, // show unknown signals by default (researchers need to see them)
            default_frequency: 433_920_000,
            hop_frequencies: crate::radio::hopper::DEFAULT_HOP_FREQUENCIES.to_vec(),
            hop_dwell_ms: crate::radio::hopper::DEFAULT_DWELL_MS,
            default_lna_gain: 24,
            default_vga_gain: 20,
            default_amp: false,
            // KAT ≤ 1.4 always transmitted at maximum power.
            tx_vga_gain: 47,
            tx_amp: true,
            default_export_format: "fob".to_string(),
            include_raw_pairs: true,
            session_autosave: true,
        }
    }

    /// Load config from an INI file, falling back to defaults for missing keys.
    fn load_from_ini(path: &std::path::Path, config_dir: &PathBuf) -> Result<Self> {
        let mut ini = Ini::new();
        ini.load(path)
            .map_err(|e| anyhow::anyhow!("Failed to load config: {}", e))?;

        let defaults = Config::default_for(config_dir);

        let export_directory = ini
            .get("general", "export_directory")
            .map(|s| expand_tilde(&s))
            .unwrap_or(defaults.export_directory);

        let import_directory = ini
            .get("general", "import_directory")
            .map(|s| expand_tilde(&s))
            .unwrap_or(defaults.import_directory);

        let max_captures = ini
            .getuint("general", "max_captures")
            .ok()
            .flatten()
            .map(|v| v as usize)
            .unwrap_or(defaults.max_captures);

        let research_mode = ini
            .getbool("general", "research_mode")
            .ok()
            .flatten()
            .unwrap_or(defaults.research_mode);

        let default_frequency = ini
            .getuint("radio", "default_frequency")
            .ok()
            .flatten()
            .map(|v| v as u32)
            .unwrap_or(defaults.default_frequency);

        let hop_frequencies = ini
            .get("radio", "hop_frequencies")
            .and_then(|s| crate::radio::hopper::parse_frequency_list(&s).ok())
            .unwrap_or(defaults.hop_frequencies);

        let hop_dwell_ms = ini
            .getuint("radio", "hop_dwell_ms")
            .ok()
            .flatten()
            .map(|v| v as u32)
            .unwrap_or(defaults.hop_dwell_ms);

        let default_lna_gain = ini
            .getuint("radio", "default_lna_gain")
            .ok()
            .flatten()
            .map(|v| v as u32)
            .unwrap_or(defaults.default_lna_gain);

        let default_vga_gain = ini
            .getuint("radio", "default_vga_gain")
            .ok()
            .flatten()
            .map(|v| v as u32)
            .unwrap_or(defaults.default_vga_gain);

        let default_amp = ini
            .getbool("radio", "default_amp")
            .ok()
            .flatten()
            .unwrap_or(defaults.default_amp);

        let tx_vga_gain = ini
            .getuint("radio", "tx_vga_gain")
            .ok()
            .flatten()
            .map(|v| v.min(47) as u32)
            .unwrap_or(defaults.tx_vga_gain);

        let tx_amp = ini
            .getbool("radio", "tx_amp")
            .ok()
            .flatten()
            .unwrap_or(defaults.tx_amp);

        let default_export_format = ini
            .get("export", "default_format")
            .unwrap_or(defaults.default_export_format);

        let include_raw_pairs = ini
            .getbool("export", "include_raw_pairs")
            .ok()
            .flatten()
            .unwrap_or(defaults.include_raw_pairs);

        // [session] (also accepted under [general], where users may look for it first)
        let session_autosave = ini
            .getbool("session", "session_autosave")
            .ok()
            .flatten()
            .or_else(|| ini.getbool("general", "session_autosave").ok().flatten())
            .unwrap_or(defaults.session_autosave);

        Ok(Self {
            export_directory,
            import_directory,
            max_captures,
            research_mode,
            default_frequency,
            hop_frequencies,
            hop_dwell_ms,
            default_lna_gain,
            default_vga_gain,
            default_amp,
            tx_vga_gain,
            tx_amp,
            default_export_format,
            include_raw_pairs,
            session_autosave,
        })
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
; Edit this file to change default settings.
; Keys are embedded in the program — no keystore directory is used or created.
; Lines starting with ; or # are comments.

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

; When off, only successfully decoded signals appear in the list.
; When on, unknown (unidentified) signals are also shown (research mode).
research_mode = {research_mode}

[radio]
; Default receive frequency in Hz ({freq_mhz:.2} MHz)
; Common keyfob frequencies: 315000000, 433920000, 868350000
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
            research_mode = self.research_mode,
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
            export_fmt = self.default_export_format,
            raw_pairs = self.include_raw_pairs,
            session_autosave = self.session_autosave,
        );

        fs::write(path, content)
            .with_context(|| format!("Failed to write config to {:?}", path))?;

        Ok(())
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
    if s.starts_with("~/") {
        if let Some(home) = dirs::home_dir() {
            return home.join(&s[2..]);
        }
    }
    PathBuf::from(s)
}

/// Resolve the KAT config directory to `~/.config/KAT/` regardless of OS.
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
}

impl Storage {
    /// Create a new storage manager.
    ///
    /// 1. Resolves the config directory (`~/.config/KAT`).
    /// 2. Creates it if missing.
    /// 3. Loads `config.ini` if it exists, otherwise writes a default one.
    /// 4. Creates the export directory if missing.
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
        let config = if config_path.exists() {
            tracing::info!("Loading config from {:?}", config_path);
            match Config::load_from_ini(&config_path, &config_dir) {
                Ok(cfg) => cfg,
                Err(e) => {
                    tracing::warn!(
                        "Failed to parse config.ini, using defaults: {}",
                        e
                    );
                    Config::default_for(&config_dir)
                }
            }
        } else {
            tracing::info!(
                "No config.ini found — creating default at {:?}",
                config_path
            );
            let config = Config::default_for(&config_dir);
            if let Err(e) = config.save_to_ini(&config_path) {
                tracing::warn!("Could not write default config.ini: {}", e);
            }
            config
        };

        // ── 4. Ensure export directory exists ────────────────────────────
        if !config.export_directory.exists() {
            fs::create_dir_all(&config.export_directory).with_context(|| {
                format!(
                    "Failed to create export dir: {:?}",
                    config.export_directory
                )
            })?;
            tracing::info!(
                "Created export directory: {:?}",
                config.export_directory
            );
        }

        // ── 5. Ensure import directory exists ────────────────────────────
        if !config.import_directory.exists() {
            fs::create_dir_all(&config.import_directory).with_context(|| {
                format!(
                    "Failed to create import dir: {:?}",
                    config.import_directory
                )
            })?;
            tracing::info!(
                "Created import directory: {:?}",
                config.import_directory
            );
        }

        // ── 6. Log resolved paths ───────────────────────────────────────
        tracing::info!("Config dir: {:?}", config_dir);
        tracing::info!("Export dir: {:?}", config.export_directory);
        tracing::info!("Import dir: {:?}", config.import_directory);

        Ok(Self {
            config_dir,
            config,
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
        let old = Config::load_from_ini(&path, &dir).unwrap();
        assert_eq!((old.default_frequency, old.tx_vga_gain, old.tx_amp), (315_000_000, 47, true));

        std::fs::write(&path, "[radio]\ntx_vga_gain = 99\ntx_amp = false\n").unwrap();
        let clamped = Config::load_from_ini(&path, &dir).unwrap();
        assert_eq!((clamped.tx_vga_gain, clamped.tx_amp), (47, false));

        let mut cfg = Config::default_for(&dir);
        cfg.tx_vga_gain = 20;
        cfg.tx_amp = false;
        cfg.save_to_ini(&path).unwrap();
        let back = Config::load_from_ini(&path, &dir).unwrap();
        let _ = std::fs::remove_dir_all(&dir);
        assert_eq!((back.tx_vga_gain, back.tx_amp), (20, false));
    }
}
