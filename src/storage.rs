//! Storage management for configuration, exports, and keystores.
//!
//! All application data lives under `~/.config/KAT/`:
//!
//! ```text
//! ~/.config/KAT/
//!   config.ini          — User configuration
//!   exports/            — Exported .fob / .sub files (save location)
//!   import/             — Scanned at startup for .fob / .sub to import
//! ```
//!
//! Captures are **in-memory only** and are discarded when KAT exits.
//! Only explicitly exported signals (.fob / .sub) persist between runs.

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
    /// Maximum captures to keep in memory during a session
    pub max_captures: usize,
    /// If off, only successfully decoded signals are added to the list. If on, unknown signals are also shown.
    pub research_mode: bool,

    // [radio]
    /// Default frequency in Hz
    pub default_frequency: u32,
    /// Default LNA gain (0-40 dB, 8 dB steps)
    pub default_lna_gain: u32,
    /// Default VGA gain (0-62 dB, 2 dB steps)
    pub default_vga_gain: u32,
    /// Default amplifier state
    pub default_amp: bool,

    // [export]
    /// Default export format (fob or sub)
    pub default_export_format: String,
    /// Include raw level/duration pairs in exports
    pub include_raw_pairs: bool,
}

impl Config {
    /// Build the default config, using the given config_dir as the base.
    /// This keeps everything under `~/.config/KAT/` by default.
    fn default_for(config_dir: &PathBuf) -> Self {
        Self {
            export_directory: config_dir.join("exports"),
            import_directory: config_dir.join("import"),
            max_captures: 100,
            research_mode: false,
            default_frequency: 433_920_000,
            default_lna_gain: 24,
            default_vga_gain: 20,
            default_amp: false,
            default_export_format: "fob".to_string(),
            include_raw_pairs: true,
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

        let default_export_format = ini
            .get("export", "default_format")
            .unwrap_or(defaults.default_export_format);

        let include_raw_pairs = ini
            .getbool("export", "include_raw_pairs")
            .ok()
            .flatten()
            .unwrap_or(defaults.include_raw_pairs);

        Ok(Self {
            export_directory,
            import_directory,
            max_captures,
            research_mode,
            default_frequency,
            default_lna_gain,
            default_vga_gain,
            default_amp,
            default_export_format,
            include_raw_pairs,
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
; Lines starting with ; or # are comments.

[general]
; Directory where .fob and .sub exports are saved.
; Supports ~ for home directory.
export_directory = {export_dir}

; Directory scanned at startup for .fob and .sub files to import (not used for saving).
; Supports ~ for home directory.
import_directory = {import_dir}

; Maximum number of captures to keep in memory per session.
; Captures are NOT persisted between runs — only exported
; signals (.fob / .sub) survive in the exports folder.
max_captures = {max_captures}

; When off, only successfully decoded signals appear in the list.
; When on, unknown (unidentified) signals are also shown (research mode).
research_mode = {research_mode}

[radio]
; Default receive frequency in Hz ({freq_mhz:.2} MHz)
; Common keyfob frequencies: 315000000, 433920000, 868350000
default_frequency = {frequency}

; Default LNA gain in dB (0, 8, 16, 24, 32, 40)
default_lna_gain = {lna}

; Default VGA gain in dB (0-62, even numbers)
default_vga_gain = {vga}

; Enable RF amplifier by default (true/false)
default_amp = {amp}

[export]
; Default export format: fob (JSON metadata) or sub (Flipper Zero)
default_format = {export_fmt}

; Include raw signal level/duration pairs in .fob exports.
; Enables signal replay but increases file size.
include_raw_pairs = {raw_pairs}
"#,
            path = path.display(),
            export_dir = export_str,
            import_dir = import_str,
            max_captures = self.max_captures,
            research_mode = self.research_mode,
            freq_mhz = freq_mhz,
            frequency = self.default_frequency,
            lna = self.default_lna_gain,
            vga = self.default_vga_gain,
            amp = self.default_amp,
            export_fmt = self.default_export_format,
            raw_pairs = self.include_raw_pairs,
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
/// Captures are in-memory only — they are discarded on exit.
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
