//! Capture data structures for storing decoded signals.

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

/// Status of a captured signal
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CaptureStatus {
    /// Signal decoded but protocol unknown
    Unknown,
    /// Signal decoded with known protocol
    Decoded,
    /// Signal can be re-encoded for transmission
    EncoderCapable,
}

impl std::fmt::Display for CaptureStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CaptureStatus::Unknown => write!(f, "Unknown"),
            CaptureStatus::Decoded => write!(f, "Decoded"),
            CaptureStatus::EncoderCapable => write!(f, "Encode"),
        }
    }
}

/// Level+duration pair for storage (serializable version)
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct StoredLevelDuration {
    pub level: bool,
    pub duration_us: u32,
}

/// A captured keyfob signal
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Capture {
    /// Unique identifier
    pub id: u32,
    /// When the signal was captured
    pub timestamp: DateTime<Utc>,
    /// Frequency in Hz
    pub frequency: u32,
    /// Protocol name if identified
    pub protocol: Option<String>,
    /// Serial number / key identifier (32-bit)
    pub serial: Option<u32>,
    /// Button code
    pub button: Option<u8>,
    /// Rolling counter value
    pub counter: Option<u16>,
    /// Whether CRC validation passed
    pub crc_valid: bool,
    /// Raw 64-bit data value
    pub data: u64,
    /// Number of valid bits in data
    pub data_count_bit: usize,
    /// Protocol-specific extra for encoding (e.g. VAG vag_type + key_idx)
    #[serde(default)]
    pub data_extra: Option<u64>,
    /// Raw level+duration pairs
    pub raw_pairs: Vec<StoredLevelDuration>,
    /// Current status
    pub status: CaptureStatus,
    /// Which demodulator path produced this capture (AM or FM). None if unknown/imported.
    #[serde(default)]
    pub received_rf: Option<RfModulation>,
    /// RF preset the capture was recorded with: an imported `.sub`'s `Preset:` (+ custom CC1101
    /// registers) or the one a `.fob` persisted. Decides OOK vs 2-FSK (and the deviation) for
    /// transmit and the `.sub` export's `Preset:`; None for live captures (see [Capture::tx_preset]).
    #[serde(default)]
    pub rf_preset: Option<crate::radio::preset::RfPreset>,
    /// Vehicle/year for vulnerability lookup and .fob export (set via 'i' in UI).
    #[serde(default)]
    pub year: Option<String>,
    /// Make for vulnerability lookup and .fob export.
    #[serde(default)]
    pub make: Option<String>,
    /// Model for vulnerability lookup and .fob export.
    #[serde(default)]
    pub model: Option<String>,
    /// Region (e.g. NA, EU) for vulnerability lookup and .fob export.
    #[serde(default)]
    pub region: Option<String>,
    /// User-editable command label (e.g. Unlock, Lock) for .fob export and filename; set via 'i' or export form.
    #[serde(default)]
    pub command: Option<String>,
    /// Source file path when imported from .sub or .fob; None for live captures.
    #[serde(default)]
    pub source_file: Option<String>,
    /// When the signal was on the air, for a capture received live or from an IQ file (not saved).
    #[serde(skip)]
    pub on_air: Option<OnAir>,
    /// What a key search (a "Recover key" action, or `autobf`) found for this capture.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub key_recovery: Option<KeyRecovery>,
}

/// The result of a key search for a capture.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum KeyRecovery {
    /// The key was recovered, by the search named ("PSA BF1", "PSA BF2", "KeeLoq type 6" …).
    Recovered(String),
    /// The search named ran to its end and no key fits.
    NotFound(String),
}

/// When a received capture's pulses were on the air: microseconds from the start of its receive
/// session (one RX run or one IQ file), which tells a press still going from a new one. Counted in
/// the receiver's samples, so it is exact between bursts of one session but is no clock time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OnAir {
    /// The receive session (`radio::rx::new_session`).
    pub session: u64,
    /// First pulse.
    pub start_us: u64,
    /// End of the last pulse.
    pub end_us: u64,
}

/// Modulation type used by protocol (encoding: PWM vs Manchester)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModulationType {
    Unknown,
    Pwm,
    Manchester,
    DifferentialManchester,
    /// Non-return-to-zero: one level per bit period (rtl_433's PCM slicer).
    Nrz,
}

impl std::fmt::Display for ModulationType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ModulationType::Unknown => write!(f, "Unknown"),
            ModulationType::Pwm => write!(f, "PWM"),
            ModulationType::Manchester => write!(f, "Manchester"),
            ModulationType::DifferentialManchester => write!(f, "Diff. Manchester"),
            ModulationType::Nrz => write!(f, "NRZ"),
        }
    }
}

/// RF modulation (carrier): AM/OOK vs FM/2FSK. From ProtoPirate SubGhzProtocolFlag_AM / _FM.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum RfModulation {
    AM,
    FM,
    /// Protocol used with both AM and FM (e.g. Kia V3/V4)
    Both,
    Unknown,
}

impl std::fmt::Display for RfModulation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RfModulation::AM => write!(f, "AM"),
            RfModulation::FM => write!(f, "FM"),
            RfModulation::Both => write!(f, "AM/FM"),
            RfModulation::Unknown => write!(f, "—"),
        }
    }
}

impl Capture {
    /// Create a new capture from level+duration pairs (received_rf = None).
    #[allow(dead_code)] // public API for tests / code that doesn't have receive path
    pub fn from_pairs(id: u32, frequency: u32, pairs: Vec<StoredLevelDuration>) -> Self {
        Self::from_pairs_with_rf(id, frequency, pairs, None)
    }

    /// Create a new capture from level+duration pairs and optional receive path (AM/FM).
    pub fn from_pairs_with_rf(
        id: u32,
        frequency: u32,
        pairs: Vec<StoredLevelDuration>,
        received_rf: Option<RfModulation>,
    ) -> Self {
        Self {
            id,
            timestamp: Utc::now(),
            frequency,
            protocol: None,
            serial: None,
            button: None,
            counter: None,
            crc_valid: false,
            data: 0,
            data_count_bit: 0,
            data_extra: None,
            raw_pairs: pairs,
            status: CaptureStatus::Unknown,
            received_rf,
            rf_preset: None,
            year: None,
            make: None,
            model: None,
            region: None,
            command: None,
            source_file: None,
            on_air: None,
            key_recovery: None,
        }
    }

    /// Get the serial as a hex string
    pub fn serial_hex(&self) -> String {
        match self.serial {
            Some(s) => format!("{:07X}", s),
            None => "-".to_string(),
        }
    }

    /// Get the frequency in MHz as a string
    pub fn frequency_mhz(&self) -> String {
        format!("{:.2}MHz", self.frequency as f64 / 1_000_000.0)
    }

    /// Get the protocol name or "Unknown"
    pub fn protocol_name(&self) -> &str {
        self.protocol.as_deref().unwrap_or("Unknown")
    }

    /// Whether this capture's frame had a checksum (or another check) KAT verifies: unknown
    /// signals and protocols without one have none, and some decoders check only some of their
    /// frames (`protocols::info::has_checksum_for`).
    pub fn crc_checked(&self) -> bool {
        self.protocol
            .as_deref()
            .is_some_and(|p| crate::protocols::has_checksum_for(p, self.data_count_bit, self.data_extra))
    }

    /// Get CRC status as a string: "-" when there is nothing to check (an unknown signal, or a
    /// protocol without a checksum), else "OK" or "FAIL".
    pub fn crc_status(&self) -> &str {
        if !self.crc_checked() {
            "-"
        } else if self.crc_valid {
            "OK"
        } else {
            "FAIL"
        }
    }

    /// Get button name: the protocol's own name for its button code; for a gate / garage remote
    /// whose protocol names none, "Btn 0xN"; else KAT's car commands.
    pub fn button_name(&self) -> &str {
        if let Some(code) = self.button {
            if let Some(name) = crate::protocols::native_button_name(self.protocol_name(), code) {
                return name;
            }
            if crate::protocols::gate_buttons::is_gate(self.protocol_name()) {
                return crate::protocols::gate_buttons::hex_label(code);
            }
        }
        match self.button {
            Some(0x01) => "Lock",
            Some(0x02) => "Unlock",
            Some(0x03) => "Lk+Un",
            Some(0x04) => "Trunk",
            Some(0x08) => "Panic",
            Some(_) => "Other",
            None => "-",
        }
    }

    /// The frame's first 64 bits (`data`) as hex, padded to the frame's bytes up to 8: a 64-bit frame
    /// 0x1234 is "0000000000001234", a 24-bit one "001234". A frame longer than 64 bits (its rest is
    /// in `data_extra`) shows its first 16 digits, like the references' `Key:` line; padding to the
    /// frame's full length put zeros in front of them instead (`000000F339…` for an 88-bit frame).
    pub fn data_hex(&self) -> String {
        if self.data_count_bit > 0 {
            let bytes = self.data_count_bit.div_ceil(8).min(8);
            format!("{:0width$X}", self.data, width = bytes * 2)
        } else {
            "-".to_string()
        }
    }

    /// The protocol's metadata (line coding, RF, encryption, make, buttons, TX presets), resolved
    /// from the protocol / display name ([crate::protocols::info]). None for unknown protocols.
    pub fn protocol_info(&self) -> Option<&'static crate::protocols::ProtocolInfo> {
        self.protocol.as_deref().and_then(crate::protocols::protocol_info)
    }

    /// Get the modulation type (line coding) based on the protocol
    pub fn modulation(&self) -> ModulationType {
        self.protocol_info().map_or(ModulationType::Unknown, |i| i.encoding)
    }

    /// Get the RF modulation (AM/FM) the references flag this protocol with (ProtoPirate
    /// SubGhzProtocolFlag_AM / _FM, Flipper-ARF / doctor-firmware for their protocols). Which
    /// demodulator actually received the capture is [Capture::received_rf].
    pub fn rf_modulation(&self) -> RfModulation {
        self.protocol_info().map_or(RfModulation::Unknown, |i| i.rf)
    }

    /// Get the encryption/encoding type based on the protocol
    pub fn encryption_type(&self) -> &'static str {
        self.protocol_info().map_or("Unknown", |i| i.encryption)
    }

    /// Make suggested by the protocol (e.g. "Kia/Hyundai"), "Unknown" for unknown protocols.
    pub fn protocol_make(&self) -> &'static str {
        self.protocol_info().map_or("Unknown", |i| i.make)
    }

    /// The preset KAT transmits this capture with, and the modulation it implies: the capture's
    /// own preset, else the protocol's preset for the demodulator path that received it, else the
    /// protocol's primary preset (see [crate::radio::preset::choose_tx_preset]).
    pub fn tx_preset(
        &self,
    ) -> (crate::radio::preset::RfPreset, crate::radio::preset::TxModulation) {
        crate::radio::preset::choose_tx_preset(
            self.rf_preset.as_ref(),
            self.received_rf,
            self.protocol_info().map(|i| i.presets),
        )
    }

    /// Modulation KAT transmits this capture with (replay and re-encode alike).
    pub fn tx_modulation(&self) -> crate::radio::preset::TxModulation {
        self.tx_preset().1
    }

    /// Preset written on export (`.sub` `Preset:`, `.fob` `preset`): the capture's own preset
    /// verbatim when it has one, else the one KAT transmits with.
    pub fn export_preset(&self) -> crate::radio::preset::RfPreset {
        self.rf_preset.clone().unwrap_or_else(|| self.tx_preset().0)
    }

    /// Get the counter as a formatted string
    pub fn counter_str(&self) -> String {
        match self.counter {
            Some(c) => format!("{:04X}", c),
            None => "-".to_string(),
        }
    }

    /// Get the timestamp formatted for display
    pub fn timestamp_short(&self) -> String {
        self.timestamp.format("%H:%M:%S").to_string()
    }

    /// Get full timestamp for detail display
    pub fn timestamp_full(&self) -> String {
        self.timestamp.format("%Y-%m-%d %H:%M:%S UTC").to_string()
    }

    /// Get button code as hex
    pub fn button_hex(&self) -> String {
        match self.button {
            Some(b) => format!("0x{:02X}", b),
            None => "-".to_string(),
        }
    }

    /// Get data bits description
    pub fn data_bits_str(&self) -> String {
        if self.data_count_bit > 0 {
            format!("{} bits", self.data_count_bit)
        } else {
            "-".to_string()
        }
    }

    /// Whether this capture has raw signal data for replay
    pub fn has_raw_data(&self) -> bool {
        !self.raw_pairs.is_empty()
    }

    /// The decode this capture holds, as the protocols take it (`encode`, `to_flipper_key`): the
    /// capture's protocol name is the decode's display name (e.g. "KeeLoq (DoorHan)").
    pub fn decoded_signal(&self) -> crate::protocols::DecodedSignal {
        crate::protocols::DecodedSignal {
            serial: self.serial,
            button: self.button,
            counter: self.counter,
            crc_valid: self.crc_valid,
            data: self.data,
            data_count_bit: self.data_count_bit,
            encoder_capable: self.status == CaptureStatus::EncoderCapable,
            extra: self.data_extra,
            protocol_display_name: self.protocol.clone(),
        }
    }

    /// Number of raw signal transitions
    pub fn raw_pair_count(&self) -> usize {
        self.raw_pairs.len()
    }
}

/// Button types for keyfob commands
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ButtonCommand {
    Unlock,
    Lock,
    Trunk,
    Panic,
}

impl ButtonCommand {
    /// Get the button code for this command
    pub fn code(&self) -> u8 {
        match self {
            ButtonCommand::Unlock => 0x02,
            ButtonCommand::Lock => 0x01,
            ButtonCommand::Trunk => 0x04,
            ButtonCommand::Panic => 0x08,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `data_hex` pads to the frame's bytes up to 8: never zeros in front of a long frame's first
    /// 64 bits (the rest of such a frame is in `data_extra`).
    #[test]
    fn data_hex_pads_to_at_most_16_digits() {
        let mut c = Capture::from_pairs(0, 433_920_000, Vec::new());
        assert_eq!(c.data_hex(), "-");
        c.data = 0x1234;
        c.data_count_bit = 24;
        assert_eq!(c.data_hex(), "001234");
        c.data_count_bit = 64;
        assert_eq!(c.data_hex(), "0000000000001234");
        c.data = 0xF339_7867_4058_45F1;
        c.data_count_bit = 88;
        c.data_extra = Some(0x9AAA49);
        assert_eq!(c.data_hex(), "F3397867405845F1");
        c.data_count_bit = 176;
        assert_eq!(c.data_hex(), "F3397867405845F1");
    }
}
