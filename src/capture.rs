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
}

/// Modulation type used by protocol (encoding: PWM vs Manchester)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModulationType {
    Unknown,
    Pwm,
    Manchester,
    #[allow(dead_code)]
    DifferentialManchester,
}

impl std::fmt::Display for ModulationType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ModulationType::Unknown => write!(f, "Unknown"),
            ModulationType::Pwm => write!(f, "PWM"),
            ModulationType::Manchester => write!(f, "Manchester"),
            ModulationType::DifferentialManchester => write!(f, "Diff. Manchester"),
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
            year: None,
            make: None,
            model: None,
            region: None,
            command: None,
            source_file: None,
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

    /// Get CRC status as a string
    pub fn crc_status(&self) -> &str {
        if self.protocol.is_none() {
            "-"
        } else if self.crc_valid {
            "OK"
        } else {
            "FAIL"
        }
    }

    /// Get button name
    pub fn button_name(&self) -> &str {
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

    /// Get data as hex string
    pub fn data_hex(&self) -> String {
        if self.data_count_bit > 0 {
            let bytes = (self.data_count_bit + 7) / 8;
            format!("{:0width$X}", self.data, width = bytes * 2)
        } else {
            "-".to_string()
        }
    }

    /// Get the modulation type based on the protocol
    pub fn modulation(&self) -> ModulationType {
        match self.protocol_name() {
            // Manchester-encoded protocols
            p if p.starts_with("Kia V1") => ModulationType::Manchester,
            p if p.starts_with("Kia V2") => ModulationType::Manchester,
            p if p.starts_with("Kia V5") => ModulationType::Manchester,
            p if p.starts_with("Kia V6") => ModulationType::Manchester,
            "Ford V0" => ModulationType::Manchester,
            "Fiat V0" => ModulationType::Manchester,
            "PSA" => ModulationType::Manchester,
            "VAG" => ModulationType::Manchester,
            // PWM-encoded protocols
            p if p.starts_with("Kia V0") => ModulationType::Pwm,
            p if p.starts_with("Kia V3") => ModulationType::Pwm,
            p if p.starts_with("Kia V4") => ModulationType::Pwm,
            "Subaru" => ModulationType::Pwm,
            "Suzuki" => ModulationType::Pwm,
            "Star Line" => ModulationType::Pwm,
            p if p.starts_with("Keeloq (") => ModulationType::Pwm,
            "Scher-Khan" => ModulationType::Pwm,
            // Unknown
            _ => ModulationType::Unknown,
        }
    }

    /// Get the RF modulation (AM/FM) for this protocol. From ProtoPirate SubGhzProtocolFlag_AM / _FM.
    /// KAT's demodulator is AM/OOK only; FM protocols may still decode if the signal is strong.
    pub fn rf_modulation(&self) -> RfModulation {
        match self.protocol_name() {
            // FM only (ProtoPirate SubGhzProtocolFlag_FM)
            p if p.starts_with("Kia V0") => RfModulation::FM,
            p if p.starts_with("Kia V2") => RfModulation::FM,
            p if p.starts_with("Kia V5") => RfModulation::FM,
            p if p.starts_with("Kia V6") => RfModulation::FM,
            "Scher-Khan" => RfModulation::FM,
            "PSA" => RfModulation::FM,
            "Fiat V0" => RfModulation::FM,
            "Ford V0" => RfModulation::FM,
            // AM only (SubGhzProtocolFlag_AM)
            p if p.starts_with("Kia V1") => RfModulation::AM,
            "VAG" => RfModulation::AM,
            "Subaru" => RfModulation::AM,
            "Suzuki" => RfModulation::AM,
            "Star Line" => RfModulation::AM,
            p if p.starts_with("Keeloq (") => RfModulation::AM,
            // Both AM and FM (Kia V3/V4)
            p if p.starts_with("Kia V3") || p.starts_with("Kia V4") => RfModulation::Both,
            _ => RfModulation::Unknown,
        }
    }

    /// Get the encryption/encoding type based on the protocol
    pub fn encryption_type(&self) -> &'static str {
        match self.protocol_name() {
            p if p.starts_with("Kia V3") || p.starts_with("Kia V4") => "KeeLoq",
            "Star Line" => "KeeLoq",
            p if p.starts_with("Keeloq (") => "KeeLoq",
            "PSA" => "XTEA/XOR",
            "VAG" => "AUT64/XTEA",
            "Scher-Khan" => "Magic Code",
            "Subaru" | "Suzuki" => "Rolling Code",
            p if p.starts_with("Ford") => "Fixed Code",
            p if p.starts_with("Fiat") => "Fixed Code",
            p if p.starts_with("Kia V0") => "Fixed Code",
            p if p.starts_with("Kia V1") || p.starts_with("Kia V2") => "Fixed Code",
            p if p.starts_with("Kia V5") || p.starts_with("Kia V6") => "Fixed Code",
            _ => "Unknown",
        }
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
