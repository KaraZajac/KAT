//! CC1101 RF presets (the Flipper Zero `.sub` `Preset:` / `Custom_preset_data:` model) and the
//! transmit modulation they imply.
//!
//! The references (ProtoPirate, Flipper-ARF, doctor-firmware) run on a CC1101 whose "preset" is a
//! register image: address/value pairs, a `00 00` terminator and the 8-byte PA table. Two registers
//! decide what goes on air:
//! - **MDMCFG2** (0x12), MOD_FORMAT in bits 6:4: 0 2-FSK, 1 GFSK, 3 ASK/OOK, 4 4-FSK, 7 MSK;
//! - **DEVIATN** (0x15): f_dev = f_xosc / 2^17 × (8 + m) × 2^e, m = bits 2:0, e = bits 6:4,
//!   f_xosc = 26 MHz (0x31 ≈ 14.28 kHz, 0x32 ≈ 15.87 kHz, 0x42 ≈ 31.74 kHz, 0x47 ≈ 47.61 kHz).
//!
//! KAT stores the preset of an imported `.sub` on the capture ([RfPreset]), persists it in `.fob`,
//! writes a matching `Preset:` on `.sub` export and transmits OOK or continuous-phase 2-FSK with the
//! preset's deviation ([TxModulation]). Captures without a preset get one from their protocol
//! ([choose_tx_preset]); the per-protocol table and its sources are in `docs/transmit.md`.

use serde::{Deserialize, Serialize};

use crate::capture::RfModulation;

/// CC1101 crystal frequency (Flipper Zero, every reference firmware).
pub const CC1101_XOSC_HZ: f64 = 26_000_000.0;
/// MDMCFG2: MOD_FORMAT in bits 6:4.
pub const REG_MDMCFG2: u8 = 0x12;
/// DEVIATN: FSK deviation exponent (bits 6:4) and mantissa (bits 2:0).
pub const REG_DEVIATN: u8 = 0x15;
/// CC1101 power-on value of DEVIATN (47.6 kHz): firmwares reset the chip before loading a preset,
/// so an FSK preset that does not set DEVIATN transmits with it.
const DEVIATN_RESET: u8 = 0x47;
/// `Preset:` value of a register-image preset (followed by `Custom_preset_module` / `_data`).
pub const CUSTOM_PRESET_NAME: &str = "FuriHalSubGhzPresetCustom";
/// `Custom_preset_module:` value for CC1101 register images.
pub const CUSTOM_PRESET_MODULE: &str = "CC1101";

/// How KAT keys the carrier for one transmission.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TxModulation {
    /// On-off keying: carrier on while the level is high, off while it is low.
    Ook,
    /// Continuous-phase 2-FSK: high = carrier + deviation, low = carrier − deviation (the CC1101's
    /// asynchronous 2-FSK mode and KAT's FM receive convention).
    Fsk { deviation_hz: f64 },
}

impl TxModulation {
    pub fn is_fsk(&self) -> bool {
        matches!(self, TxModulation::Fsk { .. })
    }
}

impl std::fmt::Display for TxModulation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TxModulation::Ook => write!(f, "OOK"),
            TxModulation::Fsk { deviation_hz } => {
                write!(f, "2-FSK ±{:.2} kHz", deviation_hz / 1000.0)
            }
        }
    }
}

/// A preset KAT knows by name: the stock Flipper presets and the references' custom presets.
#[derive(Debug)]
pub struct PresetDef {
    /// Name in the firmwares' preset lists and ProtoPirate's help ("AM650", "FM476", "F4", …).
    pub name: &'static str,
    /// `Preset:` value firmwares write for it in a `.sub` (built-in presets); None for the
    /// `setting_user` customs, which a `.sub` stores as [CUSTOM_PRESET_NAME] + registers.
    pub sub_name: Option<&'static str>,
    /// Built into every Flipper firmware (the official four). KAT exports these by name and every
    /// other preset as a custom register image, so the `.sub` loads on any firmware.
    pub universal: bool,
    /// Register image as in `Custom_preset_data` (pairs, `00 00`, 8-byte PA table).
    pub registers: &'static [u8],
    /// Where the registers come from (documentation; listed in `docs/transmit.md`).
    #[allow(dead_code)]
    pub source: &'static str,
}

impl PresetDef {
    /// Modulation the preset puts on air.
    pub fn modulation(&self) -> TxModulation {
        // Every table entry decodes (tested); OOK is the safe fallback.
        modulation_from_registers(self.registers).unwrap_or(TxModulation::Ook)
    }
}

/// The FSK presets' common modem image (stock `2fsk_dev*_async_regs`), DEVIATN last-but-one.
macro_rules! stock_2fsk {
    ($deviatn:expr) => {
        &[
            0x02, 0x0D, 0x0B, 0x06, 0x08, 0x32, 0x07, 0x04, 0x14, 0x00, 0x13, 0x02, 0x12, 0x04,
            0x11, 0x83, 0x10, 0x67, 0x15, $deviatn, 0x18, 0x18, 0x19, 0x16, 0x1D, 0x91, 0x1C, 0x00,
            0x1B, 0x07, 0x20, 0xFB, 0x22, 0x10, 0x21, 0x56, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
        ]
    };
}

/// Stock OOK, 270 kHz RX bandwidth.
pub const AM270: PresetDef = PresetDef {
    name: "AM270",
    sub_name: Some("FuriHalSubGhzPresetOok270Async"),
    universal: true,
    registers: &[
        0x02, 0x0D, 0x03, 0x47, 0x08, 0x32, 0x0B, 0x06, 0x14, 0x00, 0x13, 0x00, 0x12, 0x30, 0x11,
        0x32, 0x10, 0x67, 0x18, 0x18, 0x19, 0x18, 0x1D, 0x40, 0x1C, 0x00, 0x1B, 0x03, 0x20, 0xFB,
        0x22, 0x11, 0x21, 0xB6, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    ],
    source: "firmware cc1101_configs.c subghz_device_cc1101_preset_ook_270khz_async_regs",
};

/// Stock OOK, 650 kHz RX bandwidth: the references' AM preset for every AM car protocol.
pub const AM650: PresetDef = PresetDef {
    name: "AM650",
    sub_name: Some("FuriHalSubGhzPresetOok650Async"),
    universal: true,
    registers: &[
        0x02, 0x0D, 0x03, 0x07, 0x08, 0x32, 0x0B, 0x06, 0x14, 0x00, 0x13, 0x00, 0x12, 0x30, 0x11,
        0x32, 0x10, 0x17, 0x18, 0x18, 0x19, 0x18, 0x1D, 0x91, 0x1C, 0x00, 0x1B, 0x07, 0x20, 0xFB,
        0x22, 0x11, 0x21, 0xB6, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    ],
    source: "firmware cc1101_configs.c subghz_device_cc1101_preset_ook_650khz_async_regs",
};

/// doctor-firmware built-in OOK preset (101 kHz RX bandwidth, 16 kBaud).
pub const AM101: PresetDef = PresetDef {
    name: "AM101",
    sub_name: Some("AM101"),
    universal: false,
    registers: &[
        0x02, 0x0D, 0x08, 0x32, 0x03, 0x47, 0x0B, 0x06, 0x0C, 0x00, 0x10, 0xC9, 0x11, 0x40, 0x12,
        0x30, 0x13, 0x00, 0x14, 0x00, 0x15, 0x40, 0x18, 0x18, 0x19, 0x18, 0x1B, 0x1F, 0x1C, 0x00,
        0x1D, 0x91, 0x20, 0xFB, 0x22, 0x11, 0x21, 0xB6, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00,
    ],
    source: "doctor-firmware cc1101_configs.c subghz_device_cc1101_preset_ook_101khz_16kbaud_async_regs",
};

/// Stock 2-FSK, ±2.38 kHz (DEVIATN 0x04).
pub const FM238: PresetDef = PresetDef {
    name: "FM238",
    sub_name: Some("FuriHalSubGhzPreset2FSKDev238Async"),
    universal: true,
    registers: stock_2fsk!(0x04),
    source: "firmware cc1101_configs.c subghz_device_cc1101_preset_2fsk_dev2_38khz_async_regs",
};

/// Flipper-ARF / doctor-firmware (Unleashed) built-in 2-FSK, ±12.70 kHz (DEVIATN 0x30).
pub const FM12K: PresetDef = PresetDef {
    name: "FM12K",
    sub_name: Some("FuriHalSubGhzPreset2FSKDev12KAsync"),
    universal: false,
    registers: stock_2fsk!(0x30),
    source: "Flipper-ARF / doctor-firmware cc1101_configs.c subghz_device_cc1101_preset_2fsk_dev12khz_async_regs",
};

/// doctor-firmware built-in 2-FSK ±14.28 kHz, 38 kBaud (DEVIATN 0x31; the same modem image as F4).
pub const FM14K: PresetDef = PresetDef {
    name: "FM14K",
    sub_name: Some("FM14K"),
    universal: false,
    registers: &[
        0x02, 0x0D, 0x0B, 0x06, 0x08, 0x32, 0x07, 0x04, 0x14, 0x00, 0x13, 0x02, 0x12, 0x04, 0x11,
        0x83, 0x10, 0x6A, 0x15, 0x31, 0x18, 0x18, 0x19, 0x16, 0x1D, 0x91, 0x1C, 0x00, 0x1B, 0x07,
        0x20, 0xFB, 0x22, 0x10, 0x21, 0x56, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ],
    source: "doctor-firmware cc1101_configs.c subghz_device_cc1101_preset_2fsk_dev14khz_38kbaud_async_regs",
};

/// doctor-firmware built-in "mercedes" preset (registered as FM20K): 2-FSK ±12.70 kHz, 20 kBaud.
pub const FM20K: PresetDef = PresetDef {
    name: "FM20K",
    sub_name: Some("FM20K"),
    universal: false,
    registers: &[
        0x02, 0x0D, 0x03, 0x47, 0x07, 0x04, 0x08, 0x32, 0x0B, 0x06, 0x0C, 0x00, 0x10, 0x89, 0x11,
        0x93, 0x12, 0x00, 0x13, 0x02, 0x14, 0x00, 0x15, 0x30, 0x18, 0x18, 0x19, 0x17, 0x1B, 0x07,
        0x1C, 0x00, 0x1D, 0x91, 0x20, 0xFB, 0x22, 0x10, 0x21, 0x56, 0x00, 0x00, 0xC0, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00,
    ],
    source: "doctor-firmware cc1101_configs.c subghz_device_cc1101_preset_mercedes_async_regs",
};

/// Stock 2-FSK, ±47.61 kHz (DEVIATN 0x47): ProtoPirate's "FM476", the references' default FM preset.
pub const FM476: PresetDef = PresetDef {
    name: "FM476",
    sub_name: Some("FuriHalSubGhzPreset2FSKDev476Async"),
    universal: true,
    registers: stock_2fsk!(0x47),
    source: "firmware cc1101_configs.c subghz_device_cc1101_preset_2fsk_dev47_6khz_async_regs",
};

/// Custom "F2": 2-FSK ±19.04 kHz (DEVIATN 0x34). ProtoPirate: Mazda V0's FM preset.
pub const F2: PresetDef = PresetDef {
    name: "F2",
    sub_name: None,
    universal: false,
    registers: &[
        0x02, 0x0D, 0x03, 0x47, 0x08, 0x32, 0x0B, 0x0C, 0x15, 0x34, 0x14, 0xF8, 0x13, 0x22, 0x12,
        0x00, 0x11, 0x93, 0x10, 0x69, 0x18, 0x18, 0x19, 0x1D, 0x1D, 0x92, 0x1C, 0x00, 0x1B, 0x04,
        0x20, 0xFB, 0x22, 0x17, 0x21, 0xB6, 0x00, 0x00, 0x12, 0x0E, 0x1D, 0x34, 0x60, 0x84, 0xC8,
        0xC0,
    ],
    source: "doctor-firmware setting_user.example Custom_preset_name: F2",
};

/// Custom "F3": 2-FSK ±31.74 kHz (DEVIATN 0x42). ProtoPirate: PSA's FM preset.
pub const F3: PresetDef = PresetDef {
    name: "F3",
    sub_name: None,
    universal: false,
    registers: &[
        0x02, 0x0D, 0x03, 0x47, 0x08, 0x32, 0x0B, 0x06, 0x0C, 0x00, 0x0D, 0x10, 0x0E, 0xB0, 0x0F,
        0x71, 0x10, 0xCA, 0x11, 0x83, 0x12, 0x80, 0x13, 0x22, 0x14, 0xF8, 0x15, 0x42, 0x16, 0x07,
        0x17, 0x30, 0x18, 0x18, 0x19, 0x1D, 0x1A, 0x1C, 0x1B, 0x43, 0x1C, 0x40, 0x1D, 0x91, 0x20,
        0xFB, 0x21, 0xB6, 0x22, 0x00, 0x23, 0xE9, 0x24, 0x2A, 0x25, 0x00, 0x26, 0x1F, 0x2C, 0x81,
        0x2D, 0x35, 0x2E, 0x09, 0x00, 0x00, 0x12, 0x0E, 0x1D, 0x34, 0x60, 0x84, 0xC8, 0xC0,
    ],
    source: "doctor-firmware setting_user.example Custom_preset_name: F3 (Flipper-ARF setting_user: same DEVIATN)",
};

/// Custom "F4": 2-FSK ±14.28 kHz (DEVIATN 0x31). ProtoPirate: Ford V1/V2/V3 and Honda V2.
pub const F4: PresetDef = PresetDef {
    name: "F4",
    sub_name: None,
    universal: false,
    registers: &[
        0x02, 0x0D, 0x0B, 0x06, 0x08, 0x32, 0x07, 0x04, 0x14, 0x00, 0x13, 0x02, 0x12, 0x04, 0x11,
        0x83, 0x10, 0x6A, 0x15, 0x31, 0x18, 0x18, 0x19, 0x16, 0x1D, 0x91, 0x1C, 0x00, 0x1B, 0x07,
        0x20, 0xFB, 0x22, 0x10, 0x21, 0x56, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ],
    source: "doctor-firmware setting_user.example Custom_preset_name: F4",
};

/// Custom "Honda1": 2-FSK ±15.87 kHz (DEVIATN 0x32). ProtoPirate: Honda Static.
pub const HONDA1: PresetDef = PresetDef {
    name: "Honda1",
    sub_name: None,
    universal: false,
    registers: &[
        0x02, 0x0D, 0x0B, 0x06, 0x08, 0x32, 0x07, 0x04, 0x14, 0x00, 0x13, 0x02, 0x12, 0x04, 0x11,
        0x36, 0x10, 0x69, 0x15, 0x32, 0x18, 0x18, 0x19, 0x16, 0x1D, 0x91, 0x1C, 0x00, 0x1B, 0x07,
        0x20, 0xFB, 0x22, 0x10, 0x21, 0x56, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ],
    source: "doctor-firmware setting_user.example / Flipper-ARF setting_user Custom_preset_name: Honda1",
};

/// Every preset KAT knows by name, in lookup order. F4 precedes doctor-firmware's FM14K: their
/// register images are identical and ProtoPirate calls it F4.
pub const ALL_PRESETS: &[&PresetDef] =
    &[&AM650, &AM270, &FM476, &FM238, &F4, &HONDA1, &F3, &F2, &FM12K, &FM14K, &FM20K, &AM101];

/// FSK deviation in Hz for a DEVIATN register value.
pub fn deviation_hz(deviatn: u8) -> f64 {
    let m = f64::from(deviatn & 0x07);
    let e = (deviatn >> 4) & 0x07;
    CC1101_XOSC_HZ / f64::from(1u32 << 17) * (8.0 + m) * f64::from(1u32 << e)
}

/// Value of register `addr` in a preset image, walking address/value pairs up to the `00 00`
/// terminator (ProtoPirate `protopirate_preset_try_get_register`).
pub fn register_value(regs: &[u8], addr: u8) -> Option<u8> {
    regs.chunks_exact(2)
        .take_while(|p| !(p[0] == 0x00 && p[1] == 0x00))
        .find(|p| p[0] == addr)
        .map(|p| p[1])
}

/// Modulation of a preset image: MDMCFG2's MOD_FORMAT (ASK/OOK → OOK; 2-FSK, GFSK, 4-FSK and MSK →
/// 2-FSK with DEVIATN's deviation, KAT's transmitter being 2-FSK only). None without MDMCFG2 (like
/// ProtoPirate, KAT does not guess) or with a reserved format.
pub fn modulation_from_registers(regs: &[u8]) -> Option<TxModulation> {
    let mdmcfg2 = register_value(regs, REG_MDMCFG2)?;
    match (mdmcfg2 >> 4) & 0x07 {
        3 => Some(TxModulation::Ook),
        0 | 1 | 4 | 7 => {
            let dev = register_value(regs, REG_DEVIATN).unwrap_or(DEVIATN_RESET);
            Some(TxModulation::Fsk { deviation_hz: deviation_hz(dev) })
        }
        _ => None,
    }
}

/// Parse `Custom_preset_data`-style hex bytes ("02 0D 0B 06 …").
pub fn parse_hex_bytes(s: &str) -> Option<Vec<u8>> {
    let bytes: Option<Vec<u8>> =
        s.split_whitespace().map(|w| u8::from_str_radix(w, 16).ok()).collect();
    bytes.filter(|b| !b.is_empty())
}

/// Format bytes the way Flipper writes `Custom_preset_data` ("02 0D 0B 06 …").
pub fn format_hex_bytes(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(" ")
}

/// The RF preset a capture was recorded with, as a `.sub` stores it.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RfPreset {
    /// `Preset:` value, e.g. "FuriHalSubGhzPresetOok650Async" or [CUSTOM_PRESET_NAME].
    pub name: String,
    /// `Custom_preset_module:` (custom presets only).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub module: Option<String>,
    /// `Custom_preset_data:` register image (custom presets only).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub data: Option<Vec<u8>>,
}

impl RfPreset {
    /// The preset as KAT writes it: built-in presets by name, everything else as a custom image.
    pub fn from_def(def: &PresetDef) -> Self {
        match def.sub_name {
            Some(name) if def.universal => Self { name: name.to_string(), module: None, data: None },
            _ => Self {
                name: CUSTOM_PRESET_NAME.to_string(),
                module: Some(CUSTOM_PRESET_MODULE.to_string()),
                data: Some(def.registers.to_vec()),
            },
        }
    }

    /// A register-image preset (`FuriHalSubGhzPresetCustom`, or ProtoPirate's "Custom"/"CUSTOM").
    pub fn is_custom(&self) -> bool {
        let n = self.name.trim();
        n.eq_ignore_ascii_case("custom") || n.contains("PresetCustom")
    }

    /// The known preset this is: by `Preset:` / short name, or a custom image identical to one.
    pub fn def(&self) -> Option<&'static PresetDef> {
        if self.is_custom() {
            let data = self.data.as_deref()?;
            return ALL_PRESETS.iter().copied().find(|d| d.registers == data);
        }
        let n = self.name.trim();
        ALL_PRESETS.iter().copied().find(|d| {
            d.sub_name.is_some_and(|s| s.eq_ignore_ascii_case(n)) || d.name.eq_ignore_ascii_case(n)
        })
    }

    /// Modulation on air; None for an unknown preset name or an undecodable custom image.
    pub fn modulation(&self) -> Option<TxModulation> {
        if self.is_custom() {
            return modulation_from_registers(self.data.as_deref()?);
        }
        self.def().map(PresetDef::modulation)
    }

    /// AM (OOK/ASK) or FM (2-FSK) as a receive modulation; None when the modulation is unknown.
    pub fn rf(&self) -> Option<crate::capture::RfModulation> {
        self.modulation().map(|m| if m.is_fsk() { crate::capture::RfModulation::FM } else { crate::capture::RfModulation::AM })
    }

    /// Short label: "AM650", "F4", "Custom 2-FSK ±14.28 kHz", or the raw name when unknown.
    pub fn label(&self) -> String {
        match (self.def(), self.modulation()) {
            (Some(d), _) => d.name.to_string(),
            (None, Some(m)) if self.is_custom() => format!("Custom {m}"),
            _ => self.name.clone(),
        }
    }
}

/// Choose the preset (and so the modulation) KAT transmits a capture with and writes on export:
/// 1. the capture's own preset (imported `.sub` / `.fob`) when KAT can decode it;
/// 2. a live capture's demodulator path: AM → the protocol's OOK preset (AM650 if it has none);
///    FM → the protocol's FSK preset; a protocol the references only know as AM keeps its OOK preset
///    (an FM-path decode of an OOK burst is possible, the reverse is not); an unknown protocol → FM476;
/// 3. otherwise the protocol's primary preset, or AM650 for an unknown protocol (KAT ≤ 1.4's OOK).
///
/// `protocol_presets` is the protocol's preset list (primary first), None for an unknown protocol.
pub fn choose_tx_preset(
    own: Option<&RfPreset>,
    received: Option<RfModulation>,
    protocol_presets: Option<&[&'static PresetDef]>,
) -> (RfPreset, TxModulation) {
    if let Some(own) = own {
        if let Some(m) = own.modulation() {
            return (own.clone(), m);
        }
    }
    let presets = protocol_presets.unwrap_or(&[]);
    let find = |fsk: bool| presets.iter().copied().find(|d| d.modulation().is_fsk() == fsk);
    let def: &PresetDef = match received {
        Some(RfModulation::AM) => find(false).unwrap_or(&AM650),
        Some(RfModulation::FM) => find(true).or_else(|| presets.first().copied()).unwrap_or(&FM476),
        _ => presets.first().copied().unwrap_or(&AM650),
    };
    (RfPreset::from_def(def), def.modulation())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dev(m: TxModulation) -> f64 {
        match m {
            TxModulation::Fsk { deviation_hz } => deviation_hz,
            TxModulation::Ook => panic!("expected FSK"),
        }
    }

    #[test]
    fn deviatn_formula_matches_the_cc1101_datasheet_and_firmware_comments() {
        // cc1101_configs.c: 0x04 "2.380371 kHz", 0x30 "12.695312 kHz", 0x47 "47.60742 kHz".
        assert!((deviation_hz(0x04) - 2380.371).abs() < 0.001);
        assert!((deviation_hz(0x30) - 12695.312).abs() < 0.001);
        assert!((deviation_hz(0x47) - 47607.421).abs() < 0.001);
        assert!((deviation_hz(0x31) - 14282.226).abs() < 0.001); // F4
        assert!((deviation_hz(0x32) - 15869.140).abs() < 0.001); // Honda1
        assert!((deviation_hz(0x42) - 31738.281).abs() < 0.001); // F3
        assert!((deviation_hz(0x34) - 19042.968).abs() < 0.001); // F2
    }

    #[test]
    fn every_known_preset_decodes_to_its_documented_modulation() {
        let expect: &[(&PresetDef, Option<f64>)] = &[
            (&AM650, None),
            (&AM270, None),
            (&AM101, None),
            (&FM238, Some(2380.371)),
            (&FM12K, Some(12695.312)),
            (&FM14K, Some(14282.226)),
            (&FM20K, Some(12695.312)),
            (&FM476, Some(47607.421)),
            (&F2, Some(19042.968)),
            (&F3, Some(31738.281)),
            (&F4, Some(14282.226)),
            (&HONDA1, Some(15869.140)),
        ];
        assert_eq!(expect.len(), ALL_PRESETS.len());
        for (def, want) in expect {
            let got = modulation_from_registers(def.registers).expect(def.name);
            match want {
                None => assert_eq!(got, TxModulation::Ook, "{}", def.name),
                Some(hz) => assert!((dev(got) - hz).abs() < 0.001, "{}: {got}", def.name),
            }
            // Image shape: pairs, a 00 00 terminator, then the 8-byte PA table.
            let end = def.registers.chunks_exact(2).position(|p| p == [0, 0]).expect(def.name);
            assert_eq!(def.registers.len(), end * 2 + 2 + 8, "{}", def.name);
        }
    }

    /// The custom presets' bytes are the references' `Custom_preset_data` lines verbatim.
    #[test]
    fn custom_preset_images_match_the_reference_setting_user_files() {
        let refs = [
            (&F2, "02 0D 03 47 08 32 0B 0C 15 34 14 F8 13 22 12 00 11 93 10 69 18 18 19 1D 1D 92 1C 00 1B 04 20 FB 22 17 21 B6 00 00 12 0E 1D 34 60 84 C8 C0"),
            (&F3, "02 0D 03 47 08 32 0B 06 0C 00 0D 10 0E B0 0F 71 10 CA 11 83 12 80 13 22 14 F8 15 42 16 07 17 30 18 18 19 1D 1A 1C 1B 43 1C 40 1D 91 20 FB 21 B6 22 00 23 E9 24 2A 25 00 26 1F 2C 81 2D 35 2E 09 00 00 12 0E 1D 34 60 84 C8 C0"),
            (&F4, "02 0D 0B 06 08 32 07 04 14 00 13 02 12 04 11 83 10 6A 15 31 18 18 19 16 1D 91 1C 00 1B 07 20 FB 22 10 21 56 00 00 C0 00 00 00 00 00 00 00"),
            (&HONDA1, "02 0D 0B 06 08 32 07 04 14 00 13 02 12 04 11 36 10 69 15 32 18 18 19 16 1D 91 1C 00 1B 07 20 FB 22 10 21 56 00 00 C0 00 00 00 00 00 00 00"),
        ];
        for (def, text) in refs {
            assert_eq!(parse_hex_bytes(text).as_deref(), Some(def.registers), "{}", def.name);
            assert_eq!(format_hex_bytes(def.registers), text, "{}", def.name);
        }
    }

    #[test]
    fn stock_and_firmware_preset_names_parse() {
        let named = |n: &str| RfPreset { name: n.to_string(), module: None, data: None };
        assert_eq!(named("FuriHalSubGhzPresetOok270Async").modulation(), Some(TxModulation::Ook));
        assert_eq!(named("FuriHalSubGhzPresetOok650Async").modulation(), Some(TxModulation::Ook));
        assert!((dev(named("FuriHalSubGhzPreset2FSKDev238Async").modulation().unwrap()) - 2380.371).abs() < 0.01);
        assert!((dev(named("FuriHalSubGhzPreset2FSKDev476Async").modulation().unwrap()) - 47607.42).abs() < 0.01);
        assert!((dev(named("FuriHalSubGhzPreset2FSKDev12KAsync").modulation().unwrap()) - 12695.31).abs() < 0.01);
        // doctor-firmware writes its extra built-ins by short name.
        assert!((dev(named("FM20K").modulation().unwrap()) - 12695.31).abs() < 0.01);
        assert!((dev(named("FM14K").modulation().unwrap()) - 14282.23).abs() < 0.01);
        assert_eq!(named("AM101").modulation(), Some(TxModulation::Ook));
        // Short names work too; unknown names are unknown (the caller falls back to the protocol).
        assert_eq!(named("am650").def().map(|d| d.name), Some("AM650"));
        assert_eq!(named("FuriHalSubGhzPresetIDLE").modulation(), None);
        assert_eq!(named("FuriHalSubGhzPresetCustom").modulation(), None, "custom without data");
    }

    #[test]
    fn custom_presets_parse_from_their_registers() {
        let custom = |text: &str| RfPreset {
            name: CUSTOM_PRESET_NAME.to_string(),
            module: Some("CC1101".to_string()),
            data: parse_hex_bytes(text),
        };
        // The IMPORTS Honda captures' preset (= Honda1).
        let honda = custom("02 0D 0B 06 08 32 07 04 14 00 13 02 12 04 11 36 10 69 15 32 18 18 19 16 1D 91 1C 00 1B 07 20 FB 22 10 21 56 00 00 C0 00 00 00 00 00 00 00");
        assert_eq!(honda.def().map(|d| d.name), Some("Honda1"));
        assert_eq!(honda.label(), "Honda1");
        assert!((dev(honda.modulation().unwrap()) - 15869.14).abs() < 0.01);
        // Flipper-ARF's F3 differs from doctor's in MDMCFG4/FREQ2 but has the same DEVIATN.
        let arf_f3 = custom("02 0D 03 47 08 32 0B 06 0C 00 0D 10 0D 0D 10 C5 11 83 12 80 13 22 14 F8 15 42 16 07 17 30 18 18 19 1D 1A 1C 1B 43 1C 40 1D 91 20 FB 21 B6 22 00 23 E9 24 2A 25 00 26 1F 2C 81 2D 35 2E 09 00 00 12 0E 1D 34 60 84 C8 C0");
        assert!(arf_f3.def().is_none());
        assert!((dev(arf_f3.modulation().unwrap()) - 31738.28).abs() < 0.01);
        assert_eq!(arf_f3.label(), "Custom 2-FSK ±31.74 kHz");
        // ARF's FM95 (DEVIATN 0x24 ≈ 9.52 kHz) and an OOK custom (doctor A1).
        let fm95 = custom("02 0D 07 04 08 32 0B 06 10 67 11 83 12 04 13 02 15 24 18 18 19 16 1B 07 1C 00 1D 91 20 FB 21 56 22 10 00 00 C0 00 00 00 00 00 00 00");
        assert!((dev(fm95.modulation().unwrap()) - 9521.48).abs() < 0.01);
        let a1 = custom("02 0D 0B 06 08 32 07 04 14 00 13 00 12 30 11 32 10 17 18 18 19 18 1D 91 1C 00 1B 07 20 FB 22 11 21 B6 00 00 00 C0 00 00 00 00 00 00");
        assert_eq!(a1.modulation(), Some(TxModulation::Ook));
        // MSK / GFSK / 4-FSK formats go out as 2-FSK; a missing DEVIATN means the reset value.
        let msk = custom("12 70 00 00 00 00 00 00 00 00 00 00");
        assert!((dev(msk.modulation().unwrap()) - 47607.42).abs() < 0.01);
        // No MDMCFG2 → unknown (ProtoPirate does not guess either); registers after 00 00 ignored.
        assert_eq!(custom("15 31 00 00 12 30").modulation(), None);
        assert_eq!(custom("12 20 00 00").modulation(), None, "reserved MOD_FORMAT");
    }

    #[test]
    fn from_def_writes_builtins_by_name_and_everything_else_as_custom() {
        assert_eq!(RfPreset::from_def(&FM476).name, "FuriHalSubGhzPreset2FSKDev476Async");
        assert_eq!(RfPreset::from_def(&FM476).data, None);
        for def in [&F4, &HONDA1, &F3, &F2, &FM20K, &FM12K, &FM14K, &AM101] {
            let p = RfPreset::from_def(def);
            assert!(p.is_custom(), "{}", def.name);
            assert_eq!(p.module.as_deref(), Some("CC1101"));
            // A custom image maps back to its preset (FM14K's image is F4's, byte for byte).
            let back = if def.name == "FM14K" { "F4" } else { def.name };
            assert_eq!(p.def().map(|d| d.name), Some(back), "custom image maps back");
            assert_eq!(p.modulation(), Some(def.modulation()));
        }
    }

    #[test]
    fn tx_preset_choice() {
        let fm_only: &[&'static PresetDef] = &[&F4];
        let both: &[&'static PresetDef] = &[&AM650, &F3];
        let fsk_first: &[&'static PresetDef] = &[&FM476, &AM650];
        let am_only: &[&'static PresetDef] = &[&AM650];
        let name = |p: (RfPreset, TxModulation)| p.0.def().unwrap().name;
        // The capture's own preset wins.
        let own = RfPreset::from_def(&HONDA1);
        assert_eq!(name(choose_tx_preset(Some(&own), Some(RfModulation::AM), Some(am_only))), "Honda1");
        // ... unless KAT cannot decode it.
        let odd = RfPreset { name: "FuriHalSubGhzPresetIDLE".into(), module: None, data: None };
        assert_eq!(name(choose_tx_preset(Some(&odd), None, Some(fm_only))), "F4");
        // Live captures: the demodulator path picks the protocol's preset of that kind.
        assert_eq!(name(choose_tx_preset(None, Some(RfModulation::FM), Some(both))), "F3");
        assert_eq!(name(choose_tx_preset(None, Some(RfModulation::AM), Some(both))), "AM650");
        assert_eq!(name(choose_tx_preset(None, Some(RfModulation::AM), Some(fm_only))), "AM650");
        assert_eq!(name(choose_tx_preset(None, Some(RfModulation::FM), Some(am_only))), "AM650");
        assert_eq!(name(choose_tx_preset(None, Some(RfModulation::FM), None)), "FM476");
        // No evidence: the protocol's primary preset; unknown protocol → OOK (KAT ≤ 1.4).
        assert_eq!(name(choose_tx_preset(None, None, Some(fsk_first))), "FM476");
        assert_eq!(name(choose_tx_preset(None, None, Some(both))), "AM650");
        assert_eq!(choose_tx_preset(None, None, None).1, TxModulation::Ook);
    }
}
