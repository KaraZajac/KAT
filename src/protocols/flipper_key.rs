//! Flipper SubGhz key files: a capture the Flipper (or ProtoPirate / Flipper-ARF) decoded and saved
//! as `Protocol` + `Bit` + `Key` plus protocol-specific lines (`Key_2`, `Manufacture`, `Seed`, `Cnt`,
//! `Btn`, `Serial`, `Data`, `CRC`, `Type`, ...), instead of RAW pairs.
//!
//! [`FlipperKey`] holds the parsed file. A decoder rebuilds its decode from it with
//! [`super::ProtocolDecoder::from_flipper_key`] (the references' `deserialize`), which lets KAT
//! list, analyse and re-transmit saved keys, and writes one with
//! [`super::ProtocolDecoder::to_flipper_key`] (the references' `serialize`) so a Flipper /
//! ProtoPirate can load what KAT captured.

/// A parsed key file.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct FlipperKey {
    pub frequency: u32,
    /// `Preset:` line as written (stock name or "Custom").
    pub preset: Option<String>,
    /// `Protocol:` (the reference's display name, e.g. "CAME", "KeeLoq", "KIA/HYU V5").
    pub protocol: String,
    /// `Bit:`.
    pub bits: usize,
    /// `Key:` as a big-endian integer (the reference's `generic.data`); the first 8 bytes when the
    /// key is longer.
    pub key: u64,
    /// `Key:` bytes as written (longer than 8 for a few protocols, e.g. Hörmann BiSecur).
    pub key_bytes: Vec<u8>,
    /// `Key:` is one 16-digit word (`0000E8C1619E830D`, the `%016llX` string some ProtoPirate /
    /// Flipper-ARF serializers write) rather than the Flipper's spaced bytes (`00 00 E8 C1 …`).
    /// Both read the same; KAT writes what the protocol's reference writes, because ProtoPirate's
    /// saved-signal matching compares the `Key` text.
    pub key_word: bool,
    /// Every other `Name: value` line, in file order (the reference's `serialize` order matters
    /// for files KAT writes: the Flipper reads them sequentially).
    pub fields: Vec<(String, String)>,
}

/// `Protocol:` names the references write in key files that are not a KAT registry name, display
/// name or `info.rs` alias, mapped to the KAT name that loads them (matched ASCII
/// case-insensitively, like [super::ProtocolRegistry::get]):
/// - Flipper-ARF (`lib/subghz/protocols/*.h`) names its Kia family "KIA/HYU Vn" and calls VAG
///   "VAG GROUP", Porsche Cayenne "Porsche AG", Mazda Siemens "MazdaSiemens", Chrysler V0 "Chrysler";
/// - doctor-firmware calls Toyota "Toyota Lexus", and its `lib/subghz/registry.c` keeps loading
///   legacy names ("Korea V0" … "Korea V7", "Hitag-2", "Renault", "ScherKhan", "HondaCVE",
///   "Passat", "VAG v1"/"VAG v2", "Audi" → VAG …);
/// - "Fiat type 0": the name in `IMPORTS/FIAT/Fiat type 0 N1.sub` (an older third-party build; its
///   key is a Fiat V0 frame of the `Fiat Bravo` RAW capture in the same folder).
///
/// ProtoPirate's own catalog aliases (`protocol_items.c`: "StarLine", "Kia V3", "KIA/HYU V3",
/// "Suzuki", "Land Rover V0", "VW" …) are already `info.rs` aliases. ProtoPirate also loads a
/// "Mitsubishi V0" key as Kia V0; KAT's "Mitsubishi V0" entry hands its 61-bit keys to Kia V0.
const KEY_FILE_NAMES: &[(&str, &str)] = &[
    // Flipper-ARF
    ("KIA/HYU V0", "Kia V0"),
    ("KIA/HYU V1", "Kia V1"),
    ("KIA/HYU V2", "Kia V2"),
    ("KIA/HYU V3/V4", "Kia V3/V4"),
    ("KIA/HYU V5", "Kia V5"),
    ("KIA/HYU V6", "Kia V6"),
    ("VAG GROUP", "VAG"),
    ("Porsche AG", "Porsche Cayenne"),
    ("MazdaSiemens", "Mazda Siemens"),
    ("Chrysler", "Chrysler V0"),
    // doctor-firmware (protocol name + registry.c legacy aliases)
    ("Toyota Lexus", "Toyota"),
    ("Korea V0", "Kia V0"),
    ("Korea V1", "Kia V1"),
    ("Korea V2", "Kia V2"),
    ("Korea v3v4", "Kia V3/V4"),
    ("Korea V3", "Kia V3/V4"),
    ("Korea V4", "Kia V3/V4"),
    ("Korea V5", "Kia V5"),
    ("Korea V6", "Kia V6"),
    ("Korea V7", "Kia V7"),
    ("Hitag-2", "Renault V1"),
    ("Renault", "Renault V0"),
    ("ScherKhan", "Scher-Khan"),
    ("HondaCVE", "Honda Static"),
    ("Passat", "VAG"),
    ("VAG v1", "VAG"),
    ("VAG v2", "VAG"),
    ("Audi", "VAG"),
    // Older third-party build (IMPORTS/FIAT/Fiat type 0 N1.sub)
    ("Fiat type 0", "Fiat V0"),
];

/// The KAT name to load a key file's `Protocol:` with (see [KEY_FILE_NAMES]); other names are
/// returned as they are.
pub fn registry_name(protocol: &str) -> &str {
    let p = protocol.trim();
    KEY_FILE_NAMES
        .iter()
        .find(|(alias, _)| alias.eq_ignore_ascii_case(p))
        .map_or(p, |(_, kat)| kat)
}

/// Parse "00 00 0E 02", "0E02" or "0x0E02" (hex, big-endian) to an integer (≤ 64 bits).
#[allow(dead_code)] // used by protocols' decode_key
pub fn parse_hex_bytes(s: &str) -> Option<u64> {
    let s = s.trim();
    let s = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")).unwrap_or(s);
    let digits: String = s.chars().filter(|c| !c.is_whitespace()).collect();
    if digits.is_empty() || digits.len() > 16 || !digits.chars().all(|c| c.is_ascii_hexdigit()) {
        return None;
    }
    u64::from_str_radix(&digits, 16).ok()
}

/// Parse space-separated hex bytes ("70 08 0E D9 ...", any length) or one run of hex digits.
pub fn parse_hex_vec(s: &str) -> Option<Vec<u8>> {
    let digits: String = s.chars().filter(|c| !c.is_whitespace()).collect();
    if digits.is_empty() || !digits.len().is_multiple_of(2) || !digits.chars().all(|c| c.is_ascii_hexdigit()) {
        return None;
    }
    (0..digits.len()).step_by(2).map(|i| u8::from_str_radix(&digits[i..i + 2], 16).ok()).collect()
}

#[allow(dead_code)] // field accessors for protocols' decode_key
impl FlipperKey {
    /// Raw text of a field (`Manufacture: DoorHan` → "DoorHan").
    pub fn field(&self, name: &str) -> Option<&str> {
        self.fields.iter().find(|(n, _)| n == name).map(|(_, v)| v.as_str())
    }

    /// A key for `protocol` with `bits` and `key` (no extra lines yet), at `frequency` with the
    /// stock preset name `preset` (e.g. "FuriHalSubGhzPresetOok650Async").
    pub fn new(protocol: &str, bits: usize, key: u64, frequency: u32, preset: &str) -> Self {
        FlipperKey {
            frequency,
            preset: Some(preset.to_string()),
            protocol: protocol.to_string(),
            bits,
            key,
            key_bytes: key.to_be_bytes().to_vec(),
            key_word: false,
            fields: Vec::new(),
        }
    }

    /// A key for `protocol` with `bits` and `key` and no frequency/preset yet: what a decoder's
    /// `to_flipper_key` starts from (the caller fills in `frequency` and the preset).
    pub fn for_protocol(protocol: &str, bits: usize, key: u64) -> Self {
        FlipperKey {
            frequency: 0,
            preset: None,
            protocol: protocol.to_string(),
            bits,
            key,
            key_bytes: key.to_be_bytes().to_vec(),
            key_word: false,
            fields: Vec::new(),
        }
    }

    /// Append a `Name: value` line (written after `Key`, in call order).
    pub fn push(&mut self, name: &str, value: impl Into<String>) {
        self.fields.push((name.to_string(), value.into()));
    }

    /// Append a decimal line the way `flipper_format_write_uint32` writes it (`Cnt: 1234`).
    pub fn push_u32(&mut self, name: &str, value: u32) {
        self.push(name, value.to_string());
    }

    /// Append a hex-bytes line of any length the way `flipper_format_write_hex` writes it
    /// (`Data: 00 01 EF B8`): upper case, space separated.
    pub fn push_hex_bytes(&mut self, name: &str, bytes: &[u8]) {
        let text: Vec<String> = bytes.iter().map(|b| format!("{b:02X}")).collect();
        self.push(name, text.join(" "));
    }

    /// A hex-bytes field of any length (`Hitag2 Key: 4D 49 4B 52 ...`).
    pub fn hex_bytes_field(&self, name: &str) -> Option<Vec<u8>> {
        self.field(name).and_then(parse_hex_vec)
    }

    /// The `Key:` line as the Flipper reads it into `generic.data` (`flipper_format_read_hex(…, 8)`):
    /// exactly 8 bytes, big-endian. None for a shorter or longer key.
    pub fn key_u64(&self) -> Option<u64> {
        (self.key_bytes.len() == 8).then_some(self.key)
    }

    /// Set the `Preset:` line (and, for a custom CC1101 preset, the `Custom_preset_module` /
    /// `Custom_preset_data` lines the Flipper writes right after it).
    pub fn set_preset(&mut self, preset: &crate::radio::preset::RfPreset) {
        self.fields.retain(|(n, _)| n != "Custom_preset_module" && n != "Custom_preset_data");
        self.preset = Some(preset.name.clone());
        if preset.is_custom() {
            if let Some(data) = &preset.data {
                let module = preset.module.clone().unwrap_or_else(|| crate::radio::preset::CUSTOM_PRESET_MODULE.to_string());
                let hex: Vec<String> = data.iter().map(|b| format!("{b:02X}")).collect();
                self.fields.insert(0, ("Custom_preset_data".to_string(), hex.join(" ")));
                self.fields.insert(0, ("Custom_preset_module".to_string(), module));
            }
        }
    }

    /// The file's RF preset (`Preset:` plus custom registers), as KAT keeps it on a capture.
    pub fn rf_preset(&self) -> Option<crate::radio::preset::RfPreset> {
        let name = self.preset.as_deref().map(str::trim).filter(|n| !n.is_empty())?;
        Some(crate::radio::preset::RfPreset {
            name: name.to_string(),
            module: self.field("Custom_preset_module").map(str::to_string).filter(|m| !m.is_empty()),
            data: self.field("Custom_preset_data").and_then(crate::radio::preset::parse_hex_bytes),
        })
    }

    /// Append a hex-bytes line the way the Flipper writes `Key`-style values: `width` bytes,
    /// big-endian, upper case, space separated (`Key_2: 00 00 00 00 00 00 A7 2B`).
    pub fn push_hex(&mut self, name: &str, value: u64, width: usize) {
        let bytes = value.to_be_bytes();
        let text: Vec<String> = bytes[8 - width.min(8)..].iter().map(|b| format!("{b:02X}")).collect();
        self.push(name, text.join(" "));
    }

    /// The file text, as the Flipper writes a key file: header, Frequency, Preset, Protocol, Bit,
    /// Key, then the protocol's own lines in order.
    pub fn to_text(&self) -> String {
        let key: Vec<String> = if self.key_bytes.is_empty() {
            self.key.to_be_bytes().iter().map(|b| format!("{b:02X}")).collect()
        } else {
            self.key_bytes.iter().map(|b| format!("{b:02X}")).collect()
        };
        let key = if self.key_word { key.concat() } else { key.join(" ") };
        let mut out = String::from("Filetype: Flipper SubGhz Key File\nVersion: 1\n");
        out += &format!("Frequency: {}\n", self.frequency);
        if let Some(p) = &self.preset {
            out += &format!("Preset: {p}\n");
        }
        // A custom preset's register lines follow `Preset:` (subghz_block_generic_serialize).
        let custom = |n: &str| n == "Custom_preset_module" || n == "Custom_preset_data";
        for (n, v) in self.fields.iter().filter(|(n, _)| custom(n)) {
            out += &format!("{n}: {v}\n");
        }
        out += &format!("Protocol: {}\nBit: {}\nKey: {}\n", self.protocol, self.bits, key);
        for (n, v) in self.fields.iter().filter(|(n, _)| !custom(n)) {
            out += &format!("{n}: {v}\n");
        }
        out
    }

    /// A hex-bytes field (`Key_2: 00 00 00 00 00 00 A7 2B`, `Seed: 1A 2B 3C 4D`) as an integer.
    pub fn hex_field(&self, name: &str) -> Option<u64> {
        self.field(name).and_then(parse_hex_bytes)
    }

    /// A decimal field (`Cnt: 1234`), or hex with a 0x prefix.
    pub fn u32_field(&self, name: &str) -> Option<u32> {
        let v = self.field(name)?.trim();
        match v.strip_prefix("0x").or_else(|| v.strip_prefix("0X")) {
            Some(h) => u32::from_str_radix(h, 16).ok(),
            None => v.parse().ok(),
        }
    }

    /// Parse a key file's text. Errors when it has RAW_Data (a RAW file) or lacks Protocol/Key.
    pub fn parse(text: &str) -> Result<Self, String> {
        let mut k = FlipperKey::default();
        let mut have_key = false;
        let mut have_bits = false;
        for line in text.lines() {
            let Some((name, value)) = line.split_once(':') else { continue };
            let (name, value) = (name.trim(), value.trim());
            match name {
                "Filetype" | "Version" => {}
                "RAW_Data" => return Err("RAW file, not a key file".to_string()),
                "Frequency" => k.frequency = value.parse().map_err(|_| format!("bad Frequency: {value}"))?,
                "Preset" => k.preset = Some(value.to_string()),
                "Protocol" => k.protocol = value.to_string(),
                "Bit" => {
                    k.bits = value.parse().map_err(|_| format!("bad Bit: {value}"))?;
                    have_bits = true;
                }
                "Key" => {
                    k.key_bytes = parse_hex_vec(value).ok_or_else(|| format!("bad Key: {value}"))?;
                    k.key = k.key_bytes.iter().take(8).fold(0u64, |acc, &b| acc << 8 | u64::from(b));
                    k.key_word = k.key_bytes.len() == 8 && !value.contains(char::is_whitespace);
                    have_key = true;
                }
                _ => k.fields.push((name.to_string(), value.to_string())),
            }
        }
        if k.protocol.is_empty() || k.protocol == "RAW" {
            return Err("no Protocol in key file".to_string());
        }
        if !have_key || !have_bits {
            return Err(format!("{} key file without Key/Bit", k.protocol));
        }
        Ok(k)
    }
}

/// Test helpers for the protocols' key-file support.
#[cfg(test)]
pub(crate) mod testing {
    use super::FlipperKey;
    use crate::protocols::{DecodedSignal, ProtocolDecoder};

    /// Save `d` as KAT does (`to_flipper_key` → frequency/preset → file text) and load the text back
    /// (`FlipperKey::parse` → `from_flipper_key`). Returns the file text and the reloaded decode;
    /// None when the decoder does not write key files. Panics when the text does not parse back to
    /// the same key, or when a written key does not load.
    pub fn round_trip(dec: &dyn ProtocolDecoder, d: &DecodedSignal) -> Option<(String, DecodedSignal)> {
        let mut key = dec.to_flipper_key(d)?;
        key.frequency = 433_920_000;
        key.preset = Some("FuriHalSubGhzPresetOok650Async".to_string());
        let text = key.to_text();
        let parsed = FlipperKey::parse(&text).unwrap_or_else(|e| panic!("{}: {e}\n{text}", dec.name()));
        assert_eq!(parsed, key, "{}: key text does not parse back\n{text}", dec.name());
        let back = dec
            .from_flipper_key(&parsed)
            .unwrap_or_else(|| panic!("{}: key file KAT wrote does not load:\n{text}", dec.name()));
        Some((text, back))
    }

    /// [round_trip] that must reproduce `d` exactly; returns the file text.
    pub fn assert_round_trip(dec: &dyn ProtocolDecoder, d: &DecodedSignal) -> String {
        let (text, back) = round_trip(dec, d).unwrap_or_else(|| panic!("{}: no to_flipper_key", dec.name()));
        assert_eq!(&back, d, "{}: key file round trip changed the decode\n{text}", dec.name());
        text
    }

    /// Parse key-file text, panicking with the text on error.
    pub fn key(text: &str) -> FlipperKey {
        FlipperKey::parse(text).unwrap_or_else(|e| panic!("{e}\n{text}"))
    }

    /// Where the modules' `#[ignore]`d vector tests write (`KAT_KEY_VECTORS=<dir> cargo test
    /// vectors -- --ignored`); None when unset.
    pub fn vector_dir() -> Option<std::path::PathBuf> {
        std::env::var_os("KAT_KEY_VECTORS").map(Into::into)
    }

    /// A reference-oracle test vector: `<dir>/<name>.sub`, a Flipper RAW file of `upload` sent
    /// twice (a 20 ms LOW gap before, between and after; adjacent equal levels merged; HIGH
    /// positive, LOW negative, ≤ 512 values a line), and `<dir>/<name>.kat.txt`, KAT's key file
    /// `key` with the same Frequency / Preset (custom preset lines included). An oracle's
    /// `decode <name>.sub` prints the reference's lines for the same frame.
    pub fn write_vector(
        dir: &std::path::Path,
        name: &str,
        frequency: u32,
        preset: &str,
        upload: &[crate::radio::LevelDuration],
        key: &FlipperKey,
    ) {
        let mut values: Vec<i64> = Vec::new();
        let mut push = |level: bool, us: u32| {
            let v = if level { i64::from(us) } else { -i64::from(us) };
            match values.last_mut() {
                Some(last) if (*last > 0) == level => *last += v,
                _ => values.push(v),
            }
        };
        push(false, 20_000);
        for _ in 0..2 {
            for p in upload {
                push(p.level, p.duration_us);
            }
            push(false, 20_000);
        }
        let mut key = key.clone();
        key.frequency = frequency;
        key.preset = Some(preset.to_string());
        let mut sub = format!("Filetype: Flipper SubGhz RAW File\nVersion: 1\nFrequency: {frequency}\nPreset: {preset}\n");
        for line in ["Custom_preset_module", "Custom_preset_data"] {
            if let Some(v) = key.field(line) {
                sub += &format!("{line}: {v}\n");
            }
        }
        sub += "Protocol: RAW\n";
        for chunk in values.chunks(512) {
            let line: Vec<String> = chunk.iter().map(i64::to_string).collect();
            sub += &format!("RAW_Data: {}\n", line.join(" "));
        }
        std::fs::create_dir_all(dir).unwrap();
        std::fs::write(dir.join(format!("{name}.sub")), sub).unwrap();
        std::fs::write(dir.join(format!("{name}.kat.txt")), key.to_text()).unwrap();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_a_key_file() {
        let k = FlipperKey::parse(
            "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
             Preset: FuriHalSubGhzPresetOok650Async\nProtocol: KeeLoq\nBit: 64\n\
             Key: C6 03 BC 12 8D 2B 3A 5F\nManufacture: DoorHan\nSeed: 00 00 00 00\nCnt: 0x10\n",
        )
        .unwrap();
        assert_eq!(k.frequency, 433_920_000);
        assert_eq!(k.protocol, "KeeLoq");
        assert_eq!((k.bits, k.key), (64, 0xC603_BC12_8D2B_3A5F));
        assert_eq!(k.field("Manufacture"), Some("DoorHan"));
        assert_eq!(k.hex_field("Seed"), Some(0));
        assert_eq!(k.u32_field("Cnt"), Some(16));
        assert!(FlipperKey::parse("Protocol: RAW\nRAW_Data: 1 -2\n").is_err());
        assert!(FlipperKey::parse("Protocol: CAME\nBit: 12\n").is_err());
        assert_eq!(parse_hex_bytes("00 00 0E 02"), Some(0xE02));
        assert_eq!(parse_hex_bytes("zz"), None);
        let long = FlipperKey::parse("Protocol: Hormann BiSecur\nBit: 176\nKey: 70 08 0E D9 69 CA EE 02 6C F1\n").unwrap();
        assert_eq!((long.key_bytes.len(), long.key), (10, 0x7008_0ED9_69CA_EE02));
    }

    #[test]
    fn writes_what_it_reads() {
        let mut k = FlipperKey::new("VAG", 80, 0xC09E_BD07_92CA_7FC3, 434_390_000, "FuriHalSubGhzPresetOok650Async");
        k.push_hex("Key_2", 0xA72B, 8);
        k.push("Type", "11011");
        let text = k.to_text();
        assert!(text.starts_with("Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 434390000\n"), "{text}");
        assert!(text.contains("Protocol: VAG\nBit: 80\nKey: C0 9E BD 07 92 CA 7F C3\nKey_2: 00 00 00 00 00 00 A7 2B\nType: 11011\n"), "{text}");
        assert_eq!(FlipperKey::parse(&text).unwrap(), k);
    }

    #[test]
    fn field_writers_and_custom_presets() {
        use crate::radio::preset::{RfPreset, HONDA1};
        let mut k = FlipperKey::for_protocol("Honda Static", 64, 0x0123_4567_89AB_CDEF);
        k.push_u32("Cnt", 1234);
        k.push_hex_bytes("Data", &[0x00, 0x01, 0xEF]);
        k.frequency = 433_657_070;
        k.set_preset(&RfPreset::from_def(&HONDA1));
        let text = k.to_text();
        // Custom preset lines right after Preset (as subghz_block_generic_serialize writes them),
        // the protocol's own lines after Key.
        assert!(
            text.starts_with(
                "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433657070\n\
                 Preset: FuriHalSubGhzPresetCustom\nCustom_preset_module: CC1101\n\
                 Custom_preset_data: 02 0D 0B 06"
            ),
            "{text}"
        );
        assert!(text.ends_with("Protocol: Honda Static\nBit: 64\nKey: 01 23 45 67 89 AB CD EF\nCnt: 1234\nData: 00 01 EF\n"), "{text}");
        let back = FlipperKey::parse(&text).unwrap();
        assert_eq!(back, k);
        assert_eq!(back.rf_preset(), Some(RfPreset::from_def(&HONDA1)));
        assert_eq!(back.u32_field("Cnt"), Some(1234));
        assert_eq!(back.hex_bytes_field("Data"), Some(vec![0x00, 0x01, 0xEF]));
        assert_eq!(back.key_u64(), Some(0x0123_4567_89AB_CDEF));
        // A stock preset has no register lines; setting another preset replaces them.
        k.set_preset(&RfPreset { name: "FuriHalSubGhzPresetOok650Async".into(), module: None, data: None });
        assert!(!k.to_text().contains("Custom_preset"));
        assert_eq!(k.fields.len(), 2);
    }

    /// A one-word `Key:` (`%016llX`, as some ProtoPirate / Flipper-ARF serializers write it) reads
    /// like spaced bytes and is written back the same way.
    #[test]
    fn one_word_keys() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Scher-Khan\nBit: 51\n\
                    Key: 0000E8C1619E830C\nSerial: 0\n";
        let k = FlipperKey::parse(text).unwrap();
        assert!(k.key_word);
        assert_eq!((k.key, k.key_u64()), (0x0000_E8C1_619E_830C, Some(0x0000_E8C1_619E_830C)));
        assert_eq!(k.to_text(), text);
        let mut spaced = k.clone();
        spaced.key_word = false;
        assert!(spaced.to_text().contains("\nKey: 00 00 E8 C1 61 9E 83 0C\n"));
        assert!(!FlipperKey::parse(&spaced.to_text()).unwrap().key_word);
    }

    #[test]
    fn reference_names_resolve() {
        assert_eq!(registry_name("KIA/HYU V5"), "Kia V5");
        assert_eq!(registry_name("kia/hyu v0"), "Kia V0");
        assert_eq!(registry_name("VAG GROUP"), "VAG");
        assert_eq!(registry_name("Porsche AG"), "Porsche Cayenne");
        assert_eq!(registry_name("Toyota Lexus"), "Toyota");
        assert_eq!(registry_name("Fiat type 0"), "Fiat V0");
        assert_eq!(registry_name("Korea v3v4"), "Kia V3/V4");
        assert_eq!(registry_name("Kia V5"), "Kia V5");
        assert_eq!(registry_name("Something Else"), "Something Else");
    }
}
