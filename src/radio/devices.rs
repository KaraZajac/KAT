//! Which radios are plugged in: HackRFs, RTL-SDRs and Flipper Zeros, each with what tells it from
//! another of its kind (a serial number, a serial port), so KAT opens that one every time it
//! receives or transmits with it.
//!
//! Finding a radio does not claim it: the SDRs are opened for each receive run and each
//! transmission, a Flipper's console when KAT first uses it ([super::FlipperController]).

/// A kind of radio KAT can use.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum RadioKind {
    HackRf,
    RtlSdr,
    Flipper,
}

impl RadioKind {
    /// In the order KAT lists them (and used to pick its one radio).
    pub const ALL: [RadioKind; 3] = [RadioKind::HackRf, RadioKind::RtlSdr, RadioKind::Flipper];

    /// Name on screen.
    pub fn label(self) -> &'static str {
        match self {
            RadioKind::HackRf => "HackRF",
            RadioKind::RtlSdr => "RTL-SDR",
            RadioKind::Flipper => "Flipper",
        }
    }

    /// As `config.ini` writes it (`device`, `tx_radio`, `[radio.<kind>]`).
    pub fn config_name(self) -> &'static str {
        match self {
            RadioKind::HackRf => "hackrf",
            RadioKind::RtlSdr => "rtlsdr",
            RadioKind::Flipper => "flipper",
        }
    }

    /// `hackrf`, `rtlsdr` (`RTL-SDR`, `rtl`), `flipper` (`Flipper Zero`), in any case.
    pub fn parse(word: &str) -> Option<Self> {
        match word.trim().to_ascii_lowercase().replace(['-', '_', ' '], "").as_str() {
            "hackrf" | "hackrfone" => Some(RadioKind::HackRf),
            "rtlsdr" | "rtl" => Some(RadioKind::RtlSdr),
            "flipper" | "flipperzero" => Some(RadioKind::Flipper),
            _ => None,
        }
    }

    pub fn can_transmit(self) -> bool {
        matches!(self, RadioKind::HackRf | RadioKind::Flipper)
    }

    /// What it tunes of KAT's 300–928 MHz (a Flipper's CC1101 has gaps).
    pub fn coverage(self) -> &'static str {
        match self {
            RadioKind::HackRf | RadioKind::RtlSdr => "300–928 MHz",
            RadioKind::Flipper => "300–348, 387–464, 779–928 MHz",
        }
    }
}

/// A radio found on USB.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FoundRadio {
    pub kind: RadioKind,
    /// HackRF: its serial (32 hex digits; None: open the first HackRF, for a firmware that gives
    /// none). RTL-SDR: the serial in its EEPROM (cheap dongles all say `00000001`). Flipper: its USB
    /// serial (`flip_<name>`), when the system reports one.
    pub serial: Option<String>,
    /// Flipper: its console's serial port.
    pub port: Option<String>,
    /// RTL-SDR: its place among the RTL-SDRs on USB, used to open it when its serial is shared.
    pub index: usize,
    /// RTL-SDR: no other dongle has its serial, so it is opened by it (stable when USB order changes).
    pub serial_unique: bool,
}

impl FoundRadio {
    pub fn hackrf(serial: Option<String>) -> Self {
        Self { kind: RadioKind::HackRf, serial, port: None, index: 0, serial_unique: true }
    }

    pub fn rtlsdr(index: usize, serial: Option<String>, serial_unique: bool) -> Self {
        Self { kind: RadioKind::RtlSdr, serial, port: None, index, serial_unique }
    }

    pub fn flipper(port: String, serial: Option<String>) -> Self {
        Self { kind: RadioKind::Flipper, serial, port: Some(port), index: 0, serial_unique: true }
    }

    /// The same device as `other` (a rescan adds only the new ones).
    pub fn same_device(&self, other: &FoundRadio) -> bool {
        if self.kind != other.kind {
            return false;
        }
        match self.kind {
            RadioKind::Flipper => self.port == other.port,
            RadioKind::RtlSdr if !(self.serial_unique && other.serial_unique) => {
                self.serial == other.serial && self.index == other.index
            }
            _ => self.serial == other.serial,
        }
    }

    /// The identity `:radios` shows: the serial (a HackRF's last 8 digits), else the port.
    pub fn identity(&self) -> String {
        let serial = self.serial.as_deref().map(str::trim).filter(|s| !s.is_empty());
        match (self.kind, serial) {
            (RadioKind::HackRf, Some(s)) if s.chars().count() > 8 => {
                format!("…{}", s.chars().skip(s.chars().count() - 8).collect::<String>())
            }
            (RadioKind::RtlSdr, Some(s)) if !self.serial_unique => format!("{s} (#{})", self.index),
            (_, Some(s)) => s.to_string(),
            (_, None) => match (&self.port, self.kind) {
                (Some(p), _) => p.rsplit(['/', '\\']).next().unwrap_or(p).to_string(),
                (None, RadioKind::RtlSdr) => format!("#{}", self.index),
                (None, _) => "-".to_string(),
            },
        }
    }

    /// Whether a `[radio.<kind>.<serial>]` section's `serial` names this radio: its serial, or its
    /// last digits (as `hackrf_transfer -d` takes them), in any case; a Flipper also by its port.
    pub fn matches_serial(&self, wanted: &str) -> bool {
        let wanted = wanted.trim().to_ascii_lowercase();
        if wanted.is_empty() {
            return false;
        }
        let ends = |s: &str| s.trim().to_ascii_lowercase().ends_with(&wanted);
        self.serial.as_deref().is_some_and(ends) || self.port.as_deref().is_some_and(ends)
    }
}

/// What a search for radios found: the radios, and for each kind with none usable, why.
#[derive(Debug, Default)]
pub struct Discovery {
    pub radios: Vec<FoundRadio>,
    /// "HackRF: not found", "RTL-SDR: found but busy …" (the no-device popup, `:radios`).
    pub problems: Vec<String>,
}

/// Look for the radios of `kinds` (in [RadioKind::ALL] order). `flipper_port`: the configured
/// console port (config `flipper_port`); None: Flippers are found by their USB name.
pub fn discover(kinds: &[RadioKind], flipper_port: Option<&str>) -> Discovery {
    let mut d = Discovery::default();
    for kind in RadioKind::ALL.into_iter().filter(|k| kinds.contains(k)) {
        let (found, problem) = match kind {
            RadioKind::HackRf => super::hackrf::find_devices(),
            RadioKind::RtlSdr => super::rtlsdr::find_devices(),
            RadioKind::Flipper => super::flipper::find_devices(flipper_port),
        };
        d.radios.extend(found);
        d.problems.extend(problem.map(|why| format!("{}: {why}", kind_name(kind))));
    }
    d
}

/// The kind's full name in messages ("Flipper Zero: not found").
pub fn kind_name(kind: RadioKind) -> &'static str {
    match kind {
        RadioKind::Flipper => "Flipper Zero",
        k => k.label(),
    }
}

/// RTL-SDR serials that name one dongle: `serials[i]` is unique when no other dongle has it (and
/// it is not empty).
pub fn unique_serials(serials: &[Option<String>]) -> Vec<bool> {
    serials
        .iter()
        .map(|s| match s.as_deref().map(str::trim) {
            Some(s) if !s.is_empty() => serials.iter().filter(|o| o.as_deref().map(str::trim) == Some(s)).count() == 1,
            _ => false,
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kinds_parse_as_config_ini_and_users_write_them() {
        for (word, kind) in [
            ("hackrf", RadioKind::HackRf),
            ("HackRF One", RadioKind::HackRf),
            ("RTL-SDR", RadioKind::RtlSdr),
            ("rtl", RadioKind::RtlSdr),
            ("rtl_sdr", RadioKind::RtlSdr),
            ("Flipper Zero", RadioKind::Flipper),
            ("flipper", RadioKind::Flipper),
        ] {
            assert_eq!(RadioKind::parse(word), Some(kind), "{word}");
        }
        assert_eq!(RadioKind::parse("bladerf"), None);
        for k in RadioKind::ALL {
            assert_eq!(RadioKind::parse(k.config_name()), Some(k));
        }
        assert!(RadioKind::HackRf.can_transmit() && RadioKind::Flipper.can_transmit() && !RadioKind::RtlSdr.can_transmit());
    }

    /// Two dongles with the factory serial are told apart by their USB order; a unique serial is
    /// enough on its own.
    #[test]
    fn rtl_sdrs_with_the_same_serial() {
        let serials = [Some("00000001".to_string()), Some("00000001".to_string()), Some("00000002".to_string()), None];
        assert_eq!(unique_serials(&serials), vec![false, false, true, false]);
        let a = FoundRadio::rtlsdr(0, serials[0].clone(), false);
        let b = FoundRadio::rtlsdr(1, serials[1].clone(), false);
        let c = FoundRadio::rtlsdr(2, serials[2].clone(), true);
        assert!(!a.same_device(&b) && a.same_device(&a.clone()));
        let mut moved = c.clone();
        moved.index = 0;
        assert!(c.same_device(&moved), "a unique serial is the same dongle at any index");
        assert_eq!((a.identity(), c.identity()), ("00000001 (#0)".to_string(), "00000002".to_string()));
    }

    /// Config sections name a radio by its serial or its last digits; a Flipper also by its port.
    #[test]
    fn serials_match_by_their_end() {
        let h = FoundRadio::hackrf(Some("0000000000000000457863c82e5a1f3d".to_string()));
        assert!(h.matches_serial("2e5a1f3d") && h.matches_serial("2E5A1F3D") && h.matches_serial("0000000000000000457863c82e5a1f3d"));
        assert!(!h.matches_serial("") && !h.matches_serial("1f3e"));
        assert_eq!(h.identity(), "…2e5a1f3d");
        let f = FoundRadio::flipper("/dev/ttyACM0".to_string(), Some("flip_Ab1cD2".to_string()));
        assert!(f.matches_serial("ab1cd2") && f.matches_serial("ttyACM0"));
        assert_eq!(f.identity(), "flip_Ab1cD2");
        let unnamed = FoundRadio::flipper("/dev/cu.usbmodemflip_Ab1cD21".to_string(), None);
        assert_eq!(unnamed.identity(), "cu.usbmodemflip_Ab1cD21");
        assert!(!f.same_device(&unnamed) && !h.same_device(&f));
    }
}
