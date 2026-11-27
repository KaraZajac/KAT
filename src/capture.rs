//! Capture data structures for storing decoded signals.

use chrono::{DateTime, Local, Utc};
use serde::{Deserialize, Serialize};

/// A stored time on the user's clock, for the screen (files and sessions keep UTC).
pub fn local_time(t: DateTime<Utc>) -> DateTime<Local> {
    t.with_timezone(&Local)
}

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
    /// A loaded key file's `Repeat` line: the copies a reference that reads it sends
    /// ([crate::protocols::transmission]); written back when the capture is saved as a key file.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub key_repeat: Option<u32>,
    /// The radio that received it, by its name on screen ("HackRF", "RTL-SDR 2"); None for a file.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub radio: Option<String>,
    /// KAT's own transmission heard back: received while this radio (another of KAT's) was
    /// transmitting near its frequency.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub own_tx: Option<String>,
    /// What KAT's receiver measured of the burst this capture came from (a HackRF, an RTL-SDR or an
    /// IQ file); None for a file of pulses and for a Flipper, which reports pulses only.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub measured: Option<RfMeasurement>,
}

/// The RF side of a received burst, as KAT's receive pipeline measured it (`radio::pipeline`). The
/// "Analyze" report shows it (`analysis::signal`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct RfMeasurement {
    /// Where the signal sat, in Hz from the capture's frequency: the carrier (OOK), or the centre
    /// of the two tones (FSK).
    pub offset_hz: f32,
    /// Carrier level over the noise floor in dB.
    pub snr_db: f32,
    /// Half the distance between the two tones, when the burst carries two (2-FSK).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub deviation_hz: Option<f32>,
    /// From the burst's first carrier to its last, in µs.
    pub burst_us: u64,
    /// The burst gave pulses on both paths (AM and FM): the list holds a capture of each unless a
    /// decoder claimed one.
    #[serde(default)]
    pub both_paths: bool,
}

impl RfMeasurement {
    /// A measurement with every number finite (JSON has no NaN or infinity, and the session file
    /// would not load again with one): None when the offset or the SNR is not a number.
    pub fn checked(self) -> Option<Self> {
        (self.offset_hz.is_finite() && self.snr_db.is_finite()).then_some(Self {
            deviation_hz: self.deviation_hz.filter(|d| d.is_finite() && *d > 0.0),
            ..self
        })
    }
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
    /// Miller (delay modulation): a transition in the middle of a 1 and between two 0s, so pulses
    /// of one, one and a half and two bit periods (Honda V6).
    Miller,
}

impl std::fmt::Display for ModulationType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ModulationType::Unknown => write!(f, "Unknown"),
            ModulationType::Pwm => write!(f, "PWM"),
            ModulationType::Manchester => write!(f, "Manchester"),
            ModulationType::DifferentialManchester => write!(f, "Diff. Manchester"),
            ModulationType::Nrz => write!(f, "NRZ"),
            ModulationType::Miller => write!(f, "Miller"),
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
            key_repeat: None,
            radio: None,
            own_tx: None,
            measured: None,
        }
    }

    /// Get the serial as a hex string
    pub fn serial_hex(&self) -> String {
        match self.serial {
            Some(s) => format!("{:07X}", s),
            None => "-".to_string(),
        }
    }

    /// Where the remote sent, as heard: the frequency the receiver was tuned to plus the offset KAT's
    /// receive pipeline measured of the burst ([RfMeasurement::offset_hz]: the carrier, or the centre
    /// of the two tones), to the nearest kHz; the capture's frequency when nothing was measured (a
    /// file of pulses, a Flipper). Transmissions and exports use it: a wideband receiver hears a
    /// remote far from its tuned frequency (a HackRF ±1 MHz), and a remote whose channels are not the
    /// tuned frequency (a Ford V1 remote 330 kHz below and 480 kHz above 433.92 MHz) is not heard by
    /// its car's narrowband receiver there.
    pub fn heard_frequency(&self) -> u32 {
        match self.measured {
            Some(m) if m.offset_hz.is_finite() => {
                let hz = f64::from(self.frequency) + f64::from(m.offset_hz);
                ((hz / 1e3).round() * 1e3).clamp(0.0, f64::from(u32::MAX)) as u32
            }
            _ => self.frequency,
        }
    }

    /// The frequency it was heard on ([Self::heard_frequency]) in MHz, as a string ("433.92MHz").
    pub fn heard_frequency_mhz(&self) -> String {
        format!("{:.2}MHz", f64::from(self.heard_frequency()) / 1e6)
    }

    /// Get the protocol name or "Unknown"
    pub fn protocol_name(&self) -> &str {
        self.protocol.as_deref().unwrap_or("Unknown")
    }

    /// The remote's user tag, if any (`protocols/remote_tags.rs`): a Fiat V1 remote the user has
    /// confirmed is a Renault-Nissan Alliance one, for example.
    pub fn remote_tag(&self) -> Option<crate::protocols::remote_tags::RemoteTag> {
        self.serial.and_then(crate::protocols::remote_tags::tag_of)
    }

    /// The protocol name to show the user: the tag's (e.g. "Nissan V1") when this capture is
    /// tagged, else the engine's ([`Self::protocol_name`]). The engine name never changes, so
    /// decode, key resolution and TX are unaffected.
    pub fn display_protocol_name(&self) -> &str {
        match self.remote_tag() {
            Some(tag) => tag.display_protocol(),
            None => self.protocol_name(),
        }
    }

    /// Whether this capture's frame had a checksum (or another check) KAT verifies: unknown
    /// signals and protocols without one have none, and some decoders check only some of their
    /// frames (`protocols::info::has_checksum_for`).
    pub fn crc_checked(&self) -> bool {
        self.protocol
            .as_deref()
            .is_some_and(|p| crate::protocols::has_checksum_for(p, self.data_count_bit, self.data_extra))
            // A frame of the older PSA code: its check is inside the enciphered block (applied while
            // deciphering, when a key table is loaded), so there is no CRC to show.
            && !self.psa_older_code().is_some_and(|(_, remote)| remote.is_some())
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

    /// What KAT made of the capture, the list's Status column: "Own TX" for one of KAT's own
    /// transmissions heard by another of its radios, else whether it was decoded and whether KAT
    /// can encode its protocol.
    pub fn status_label(&self) -> &'static str {
        match self.status {
            _ if self.own_tx.is_some() => "Own TX",
            CaptureStatus::EncoderCapable => "Encode",
            CaptureStatus::Decoded => "Decoded",
            CaptureStatus::Unknown => "Unknown",
        }
    }

    /// Whether the capture has a known vulnerability: it can be re-encoded, or its vehicle (set
    /// with `i`) matches a CVE.
    pub fn vuln_found(&self) -> bool {
        self.status == CaptureStatus::EncoderCapable
            || !crate::vuln_db::match_vulns(
                self.year.as_deref(),
                self.make.as_deref(),
                self.model.as_deref(),
                self.region.as_deref(),
            )
            .is_empty()
    }

    /// Get button name: the protocol's own name for its button code; for a gate / garage remote
    /// whose protocol names none, "Btn 0xN"; else KAT's car commands.
    pub fn button_name(&self) -> &str {
        if let Some(code) = self.button {
            // A tagged remote (e.g. Nissan) relabels the buttons it maps; others fall through.
            if let Some(name) = self.remote_tag().and_then(|tag| tag.button_name(code)) {
                return name;
            }
            if let Some(name) = crate::protocols::capture_button_name(self.protocol_name(), code, self.data, self.data_extra) {
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

    /// The frame as sent, in hex, and its bits on air: the whole of a long frame whose protocol
    /// keeps the rest in `data_extra` as plain bits ([crate::protocols::frame_hex]: PSA's 80 bits,
    /// not 64), else [Capture::data_hex] and `data_count_bit`.
    pub fn frame_hex(&self) -> (String, usize) {
        crate::protocols::frame_hex(self.protocol_name(), self.data, self.data_extra)
            .unwrap_or_else(|| (self.data_hex(), self.data_count_bit))
    }

    /// The older PSA code's remote type in the frame's header, and the remote it is for and its
    /// part numbers when known ([crate::protocols::psa_older_code]).
    pub fn psa_older_code(&self) -> Option<(u16, Option<(&'static str, &'static str)>)> {
        crate::protocols::psa_older_code(self.protocol_name(), self.serial, self.data)
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

    /// The time of day it was captured, on the user's clock (the list's Time column).
    pub fn timestamp_short(&self) -> String {
        local_time(self.timestamp).format("%H:%M:%S").to_string()
    }

    /// Date and time it was captured on the user's clock, with the clock's offset from UTC:
    /// "2026-11-04 10:31:07 UTC+01:00".
    pub fn timestamp_full(&self) -> String {
        local_time(self.timestamp).format("%Y-%m-%d %H:%M:%S UTC%:z").to_string()
    }

    /// Get button code as hex
    pub fn button_hex(&self) -> String {
        match self.button {
            Some(b) => format!("0x{:02X}", b),
            None => "-".to_string(),
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

    /// The screen shows the user's clock: the full form carries the offset, so it reads back as
    /// the stored instant whatever the zone of the machine running this.
    #[test]
    fn shown_times_are_the_stored_instant_on_the_local_clock() {
        let mut c = Capture::from_pairs(0, 433_920_000, Vec::new());
        c.timestamp = DateTime::parse_from_rfc3339("2026-11-04T10:31:07.123456789Z").unwrap().with_timezone(&Utc);
        let full = c.timestamp_full();
        let back = DateTime::parse_from_str(&full, "%Y-%m-%d %H:%M:%S UTC%:z").expect(&full);
        assert_eq!(back.timestamp(), c.timestamp.timestamp(), "{full}");
        assert!(full.contains(&format!(" {} UTC", c.timestamp_short())), "{full}");
    }

    /// A measurement that is not a number is dropped, and so is a deviation that is none.
    #[test]
    fn a_measurement_holds_numbers_only() {
        let m = RfMeasurement { offset_hz: 1_500.0, snr_db: 22.0, deviation_hz: Some(47_600.0), burst_us: 9_000, both_paths: true };
        assert_eq!(m.checked(), Some(m));
        assert_eq!(RfMeasurement { offset_hz: f32::NAN, ..m }.checked(), None);
        assert_eq!(RfMeasurement { snr_db: f32::INFINITY, ..m }.checked(), None);
        for bad in [f32::NAN, 0.0, -3.0] {
            assert_eq!(RfMeasurement { deviation_hz: Some(bad), ..m }.checked().map(|m| m.deviation_hz), Some(None));
        }
    }

    /// Where a capture was heard: the tuned frequency plus the measured offset, to the kHz; the
    /// tuned frequency itself for a capture nothing measured (a file, a Flipper).
    #[test]
    fn heard_frequency_adds_the_measured_offset() {
        let mut c = Capture::from_pairs(0, 433_920_000, Vec::new());
        assert_eq!(c.heard_frequency(), 433_920_000);
        let m = RfMeasurement { offset_hz: -329_500.0, snr_db: 11.2, deviation_hz: None, burst_us: 120_000, both_paths: false };
        for (offset, heard) in [(-329_500.0, 433_591_000), (477_200.0, 434_397_000), (-14_449.0, 433_906_000), (0.0, 433_920_000)] {
            c.measured = Some(RfMeasurement { offset_hz: offset, ..m });
            assert_eq!(c.heard_frequency(), heard, "{offset}");
        }
        c.measured = Some(RfMeasurement { offset_hz: f32::NAN, ..m });
        assert_eq!(c.heard_frequency(), 433_920_000);
    }

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
        // No protocol says its extra is plain frame bits: the frame is what data_hex shows.
        assert_eq!(c.frame_hex(), ("F3397867405845F1".to_string(), 176));
    }

    /// A PSA frame is 80 bits on air: key1 and key2's 16-bit validation word (GitHub issue #6:
    /// "only 8 of the 10 bytes"). An older-code frame's header names its remote, and its checksum
    /// counts as unread, not failed.
    #[test]
    fn a_psa_frame_shows_its_80_bits_and_its_remote_type() {
        let mut c = Capture::from_pairs(0, 433_920_000, Vec::new());
        c.protocol = Some("PSA".to_string());
        (c.data, c.data_count_bit, c.data_extra) = (0xC1AA_18C8_F715_6A8A, 128, Some(0x0D09));
        assert_eq!(c.frame_hex(), ("C1AA18C8F7156A8A0D09".to_string(), 80));
        assert_eq!(c.data_hex(), "C1AA18C8F7156A8A", "key files keep key1 alone");
        let (ty, remote) = c.psa_older_code().unwrap();
        assert_eq!(ty, 0x0C1A);
        assert!(remote.is_some_and(|(r, p)| r.contains("C3 XTR") && p == "6554RE"));
        assert!(!c.crc_checked());
        // A TEA frame (header F47A): no type, its checksum is checked.
        c.data = 0xF47A_2EA1_1A4C_7CAF;
        assert_eq!(c.psa_older_code(), None);
        assert!(c.crc_checked());
        // A decrypted frame has no older-code reading.
        (c.data, c.serial) = (0xC1AA_18C8_F715_6A8A, Some(0x123456));
        assert_eq!(c.psa_older_code(), None);
    }
}
