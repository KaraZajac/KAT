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
        if let Some(code) = self.button {
            if let Some(name) = crate::protocols::native_button_name(self.protocol_name(), code) {
                return name;
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
            p if p.starts_with("Kia V7") => ModulationType::Manchester,
            p if p.starts_with("Ford") => ModulationType::Manchester,
            "Honda Static" => ModulationType::Manchester,
            "Mazda Siemens" => ModulationType::Manchester,
            "Mazda V0" => ModulationType::Manchester,
            "BMW CAS4" => ModulationType::Manchester,
            "Fiat V0" => ModulationType::Manchester,
            // "PSA2" is the retired name (PSA now decodes those frames); kept for old captures.
            "PSA" | "PSA2" => ModulationType::Manchester,
            "VAG" => ModulationType::Manchester,
            // Differential Manchester
            // Honda V2 (legacy name "Land Rover V0" kept for older .fob imports).
            "Honda V2" | "Land Rover V0" => ModulationType::DifferentialManchester,
            // PWM-encoded protocols
            p if p.starts_with("Kia V0") => ModulationType::Pwm,
            // Kia V0 frame types (ProtoPirate display names) and ARF's 96-bit Mitsubishi V0.
            "Suzuki V0" | "Honda V0" | "Mitsubishi V0" => ModulationType::Pwm,
            p if p.starts_with("Kia V3") => ModulationType::Pwm,
            p if p.starts_with("Kia V4") => ModulationType::Pwm,
            "Honda V1" => ModulationType::Pwm,
            "Subaru" => ModulationType::Pwm,
            "Suzuki" => ModulationType::Pwm,
            p if p.starts_with("Star Line") => ModulationType::Pwm,
            // "Keeloq (<keystore name>)" (generic fallback) and "KeeLoq (<manufacturer>)" (registry).
            p if p.to_ascii_lowercase().starts_with("keeloq (") => ModulationType::Pwm,
            p if p.starts_with("Scher-Khan") => ModulationType::Pwm,
            "Chrysler V0" => ModulationType::Pwm,
            "Land Rover RKE" => ModulationType::Pwm,
            // Toyota (ARF/doctor "Toyota Lexus"): bi-phase, two shorts = 0, one long = 1.
            "Toyota" => ModulationType::DifferentialManchester,
            // "Porsche Cayenne", plus legacy "Porsche Touareg [...]" captures (same protocol).
            p if p.starts_with("Porsche") => ModulationType::Pwm,
            // v1.4.0 additions (ProtoPirate / Flipper-ARF / doctor-firmware ports)
            "Fiat V1" | "Fiat V2" => ModulationType::Manchester,
            "Renault V0" | "Renault V1" => ModulationType::Manchester,
            // Display names carry the frame type, e.g. "Renault V1 (ARF) [0x0C]".
            p if p.starts_with("Renault V1 (ARF)") => ModulationType::Manchester,
            "Mercedes V0" | "Mercedes V1" => ModulationType::Manchester,
            "Opel V1" | "BMW" | "Datsun Lada" => ModulationType::Manchester,
            "Mazda Infinity" | "Hyundai" => ModulationType::Manchester,
            // Bi-phase with a transition at every bit boundary (the bit is the second half's level).
            "Mazda V2" => ModulationType::DifferentialManchester,
            // GM is pulse-position (gap width) — closest label is PWM.
            "GM" | "Sheriff CFM" | "Audi" => ModulationType::Pwm,
            // Unknown
            _ => ModulationType::Unknown,
        }
    }

    /// Get the RF modulation (AM/FM) for this protocol. From ProtoPirate SubGhzProtocolFlag_AM / _FM.
    /// KAT's demodulator is AM/OOK only; FM protocols may still decode if the signal is strong.
    pub fn rf_modulation(&self) -> RfModulation {
        match self.protocol_name() {
            // FM only (ProtoPirate SubGhzProtocolFlag_FM)
            // Kia V0 family: ProtoPirate flags FM, Flipper-ARF AM + FM.
            p if p.starts_with("Kia V0") => RfModulation::Both,
            "Suzuki" | "Suzuki V0" | "Honda V0" => RfModulation::Both,
            "Mitsubishi V0" => RfModulation::FM,
            // Kia V2: ProtoPirate flags AM.
            p if p.starts_with("Kia V2") => RfModulation::AM,
            p if p.starts_with("Kia V5") => RfModulation::FM,
            p if p.starts_with("Kia V6") => RfModulation::FM,
            p if p.starts_with("Kia V7") => RfModulation::FM,
            // ProtoPirate flags Scher-Khan FM; Flipper-ARF (Magic Code PRO1/PRO2) FM + AM.
            p if p.starts_with("Scher-Khan") => RfModulation::Both,
            // PSA: ProtoPirate routes by preset (AM and FM); the IMPORTS captures are OOK.
            "PSA" | "PSA2" => RfModulation::Both,
            // Fiat V0 / Ford V0: ProtoPirate flags AM (the IMPORTS captures are AM650).
            "Fiat V0" => RfModulation::AM,
            "Ford V0" => RfModulation::AM,
            // Ford V3 is flagged AM | FM in ProtoPirate (EU + US variants).
            "Ford V3" => RfModulation::Both,
            p if p.starts_with("Ford") => RfModulation::FM,
            "Honda Static" => RfModulation::FM,
            "Mazda Siemens" => RfModulation::FM,
            // Mazda V0 (ProtoPirate) is flagged AM | FM.
            "Mazda V0" => RfModulation::Both,
            "Honda V2" | "Land Rover V0" => RfModulation::FM,
            // AM only (SubGhzProtocolFlag_AM)
            p if p.starts_with("Kia V1") => RfModulation::AM,
            "Honda V1" => RfModulation::AM,
            "VAG" => RfModulation::AM,
            "Subaru" => RfModulation::AM,
            p if p.starts_with("Star Line") => RfModulation::AM,
            "Chrysler V0" => RfModulation::AM,
            "Land Rover RKE" => RfModulation::AM,
            "Toyota" => RfModulation::AM,
            "BMW CAS4" => RfModulation::AM,
            p if p.starts_with("Porsche") => RfModulation::AM,
            p if p.to_ascii_lowercase().starts_with("keeloq (") => RfModulation::AM,
            // Kia V3/V4: ProtoPirate routes it to the FM registry.
            p if p.starts_with("Kia V3") || p.starts_with("Kia V4") => RfModulation::FM,
            // v1.4.0 additions
            "Mercedes V0" | "Mercedes V1" | "Datsun Lada" => RfModulation::FM,
            "Fiat V1" | "Fiat V2" | "Renault V0" | "Opel V1" | "BMW" | "GM" => RfModulation::AM,
            "Sheriff CFM" | "Audi" | "Mazda Infinity" | "Hyundai" => RfModulation::AM,
            "Renault V1" | "Mazda V2" => RfModulation::Both,
            p if p.starts_with("Renault V1 (ARF)") => RfModulation::Both,
            _ => RfModulation::Unknown,
        }
    }

    /// Get the encryption/encoding type based on the protocol
    pub fn encryption_type(&self) -> &'static str {
        match self.protocol_name() {
            p if p.starts_with("Kia V3") || p.starts_with("Kia V4") => "KeeLoq",
            p if p.starts_with("Star Line") => "KeeLoq",
            p if p.to_ascii_lowercase().starts_with("keeloq (") => "KeeLoq",
            "PSA" | "PSA2" => "TEA/XOR",
            "VAG" => "AUT64/XTEA",
            p if p.starts_with("Scher-Khan") => "Magic Code",
            "Subaru" | "Suzuki" | "Suzuki V0" | "Honda V0" | "Mitsubishi V0" => "Rolling Code",
            "Chrysler V0" => "Rolling Code",
            "Honda V2" | "Land Rover V0" => "Rolling Code",
            "Land Rover RKE" => "KeeLoq",
            "Toyota" => "Rolling Code",
            "Mazda Siemens" => "Siemens XOR",
            "Mazda V0" => "XOR whitening",
            "BMW CAS4" => "CAS4 (rolling)",
            p if p.starts_with("Porsche") => "VAG rolling",
            // Ford V0/V1/V2 are rolling codes; must precede the generic Ford "Fixed Code" arm below.
            "Ford V0" | "Ford V1" | "Ford V2" => "Rolling Code",
            // Ford V3 carries a counter + hop in both variants.
            "Ford V3" => "Rolling Code",
            p if p.starts_with("Ford") => "Fixed Code",
            "Honda Static" => "Fixed Code",
            "Honda V1" => "Fixed Code",
            // Fiat V1 (ProtoPirate re-port) verifies the hop against embedded HITAG2 keys.
            "Fiat V1" => "HITAG2",
            "Fiat V0" | "Fiat V2" => "Rolling Code",
            p if p.starts_with("Fiat") => "Fixed Code",
            p if p.starts_with("Kia V6") => "AES-128",
            p if p.starts_with("Kia V0") || p.starts_with("Kia V1") || p.starts_with("Kia V2") => "Rolling Code",
            p if p.starts_with("Kia V5") || p.starts_with("Kia V7") => "Rolling Code",
            // v1.4.0 additions
            "Renault V1" | "Datsun Lada" => "HITAG2",
            // ARF's type-table frames: rolling, replay-only (no key path is validated upstream).
            p if p.starts_with("Renault V1 (ARF)") => "Rolling Code",
            "Opel V1" => "XTEA",
            "Sheriff CFM" => "KeeLoq",
            "Renault V0" | "Mercedes V0" | "Mercedes V1" | "BMW" | "GM" => "Rolling Code",
            "Mazda V2" | "Mazda Infinity" | "Audi" | "Hyundai" => "Rolling Code",
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
