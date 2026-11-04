//! Helpers shared by the G6 gate protocols: [super::nice_flor_s], [super::came_atomo],
//! [super::alutech_at_4n] and [super::ditec_gol4].
//!
//! **Rainbow tables.** Nice FloR-S and Alutech AT-4N encrypt with a 32-byte secret table that the
//! Flipper firmwares read from an asset file (`subghz/assets/nice_flor_s`,
//! `subghz/assets/alutech_at_4n`) with `subghz_keystore_raw_get_data(file, 0, buffer, 32)`: the
//! first 32 bytes of the file's decrypted payload. Those asset files ship AES-encrypted with the
//! Flipper's device key, so neither the references nor KAT have the plaintext. KAT reads an
//! optional plaintext copy the user provides, `~/.config/KAT/nice_flor_s.bin` and
//! `~/.config/KAT/alutech_at_4n.bin`: exactly [TABLE_LEN] raw bytes, the C's `buffer[0..32]`
//! (see each module for how its bytes are used). A file of any other size is ignored with a
//! warning. Without a table KAT decodes exactly what a Flipper decodes when the asset is missing
//! (every frame is reported, the encrypted fields are not recovered) and does not transmit: the C
//! would send a frame whose encrypted part is a constant (Nice FloR-S: 0, Alutech: all ones),
//! which no receiver accepts.
//!
//! **Counters.** The references bump the 16-bit counter by the firmware's "rolling counter
//! multiplier" (`furi_hal_subghz_get_rolling_counter_mult()`, 1 by default in doctor-firmware and
//! Flipper-ARF), wrapping to 0 past 0xFFFF; a `CounterMode` key-file line selects the
//! protocol's alternative sequences. KAT has no multiplier setting and uses the default 1.

use crate::radio::demodulator::LevelDuration;

/// Size of a rainbow table (`SUBGHZ_NICE_FLOR_S_RAINBOW_TABLE_SIZE_BYTES`,
/// `SUBGHZ_ALUTECH_AT_4N_RAINBOW_TABLE_SIZE_BYTES`).
pub(super) const TABLE_LEN: usize = 32;

/// A plaintext rainbow table.
pub(super) type Table = [u8; TABLE_LEN];

/// Read the plaintext table `~/.config/KAT/<file>`; None when it is absent or not exactly
/// [TABLE_LEN] bytes long.
pub(super) fn read_config_table(file: &str) -> Option<Table> {
    let path = crate::storage::resolve_config_dir()?.join(file);
    let bytes = std::fs::read(&path).ok()?;
    match Table::try_from(bytes.as_slice()) {
        Ok(table) => {
            tracing::info!("Loaded rainbow table {}", path.display());
            Some(table)
        }
        Err(_) => {
            tracing::warn!(
                "{}: {} bytes, expected {TABLE_LEN} raw bytes; ignored",
                path.display(),
                bytes.len()
            );
            None
        }
    }
}

/// Append one level to an upload, merging it into the previous entry when the level repeats (the
/// radio sends adjacent equal levels as one pulse).
pub(super) fn push(out: &mut Vec<LevelDuration>, level: bool, duration: u32) {
    match out.last_mut() {
        Some(last) if last.level == level => last.duration_us += duration,
        _ => out.push(LevelDuration::new(level, duration)),
    }
}

/// The references' default counter step (`CounterMode` 0, multiplier 1): + 1, and 0 instead of
/// a value above 0xFFFF.
pub(super) fn counter_increment(cnt: u32) -> u32 {
    if cnt + 1 > 0xFFFF {
        0
    } else {
        cnt + 1
    }
}

/// The references' "OFEX" sequence (Unleashed's overflow-experimental multiplier, also CAME
/// Atomo's and Alutech's `CounterMode` 1): 0 → 1 → 0xFFFE → 0xFFFF → 0.
pub(super) fn counter_ofex(cnt: u32) -> u32 {
    if cnt + 1 > 0xFFFF {
        0
    } else if cnt >= 1 && cnt != 0xFFFE {
        0xFFFE
    } else {
        cnt + 1
    }
}

/// The `CounterMode` line of a key file (`(uint8_t)` of the value), 0 when absent.
pub(super) fn counter_mode(key: &super::FlipperKey) -> u8 {
    key.u32_field("CounterMode").map_or(0, |v| v as u8)
}

/// An upload as (level, µs) tuples (tests compare uploads; `LevelDuration` has no `PartialEq`).
#[cfg(test)]
pub(super) fn tuples(pulses: &[LevelDuration]) -> Vec<(bool, u32)> {
    pulses.iter().map(|p| (p.level, p.duration_us)).collect()
}

/// Every complete decode a fresh decoder reports for `pulses` (the registry resets all decoders
/// after a decode, like the Flipper receiver: `reset()` is called after each one).
#[cfg(test)]
pub(super) fn decode_all(
    decoder: &mut dyn super::ProtocolDecoder,
    pulses: &[LevelDuration],
) -> Vec<super::DecodedSignal> {
    let mut out = Vec::new();
    for p in pulses {
        if let Some(d) = decoder.feed(p.level, p.duration_us) {
            out.push(d);
            decoder.reset();
        }
    }
    out
}

/// A Flipper unit-test RAW capture from `REFERENCES/` (crate root; gitignored), as pulses; None
/// (test skipped) when REFERENCES is absent, as in CI.
#[cfg(test)]
pub(super) fn unit_test_capture(name: &str) -> Option<(u32, Vec<LevelDuration>)> {
    let path = std::path::Path::new(
        "REFERENCES/Flipper-ARF/applications/debug/unit_tests/resources/unit_tests/subghz",
    )
    .join(name);
    if !path.exists() {
        eprintln!("Skip: {} not found", path.display());
        return None;
    }
    let (freq, raw) = crate::export::flipper::import_sub_raw(&path).ok()?;
    Some((freq, raw.iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect()))
}

/// Oracle parity diagnostics (`#[ignore]`d; driven by environment variables, see each test).
/// Output lines are tab-separated so a script can compare them with the C oracle's.
#[cfg(test)]
mod parity {
    use super::super::{alutech_at_4n, came_atomo, ditec_gol4, nice_flor_s};
    use super::super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolRegistry};
    use super::*;

    /// The four decoders without tables (as the oracle runs: asset files missing).
    fn decoders() -> Vec<Box<dyn ProtocolDecoder>> {
        vec![
            Box::new(alutech_at_4n::AlutechAt4nDecoder::with_table(None)),
            Box::new(nice_flor_s::NiceFlorSDecoder::with_table(None)),
            Box::new(ditec_gol4::DitecGol4Decoder::new()),
            Box::new(came_atomo::CameAtomoDecoder::new()),
        ]
    }

    fn list(var: &str) -> Vec<String> {
        let Ok(path) = std::env::var(var) else { return Vec::new() };
        std::fs::read_to_string(path).unwrap().lines().map(str::to_string).filter(|l| !l.is_empty()).collect()
    }

    /// The C's `generic` view of a decode: (key = `generic.data`, aux = Nice One `Data` or
    /// Alutech `CRC`).
    fn c_view(name: &str, d: &DecodedSignal) -> (u64, String) {
        match (name, d.data_count_bit) {
            ("Nice FloR-S", 72) => (d.data >> 12, (((d.data & 0xFFF) << 8) | (d.extra.unwrap_or(0) & 0xFF)).to_string()),
            ("Alutech AT-4N", _) => (d.data, (d.extra.unwrap_or(0) & 0xFF).to_string()),
            _ => (d.data, "-".to_string()),
        }
    }

    fn hex(v: Option<u64>) -> String {
        v.map_or("-".to_string(), |v| format!("{v:X}"))
    }

    /// `KAT_G6_LIST=<file listing .sub paths>`: every decode of each G6 decoder fed alone and
    /// reset after each decode (the oracle's `decode --protocol NAME`), normal polarity only.
    #[test]
    #[ignore]
    fn g6_dump_decodes() {
        for path in list("KAT_G6_LIST") {
            let Ok((_, raw)) = crate::export::flipper::import_sub_raw(std::path::Path::new(&path)) else {
                println!("ERR\t{path}");
                continue;
            };
            let pulses: Vec<LevelDuration> = raw.iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect();
            for mut dec in decoders() {
                let name = dec.name();
                for d in decode_all(dec.as_mut(), &pulses) {
                    let (key, aux) = c_view(name, &d);
                    println!(
                        "DEC\t{path}\t{name}\t{}\t{key:X}\t{aux}\t{}\t{}\t{}",
                        d.data_count_bit,
                        hex(d.serial.map(u64::from)),
                        hex(d.button.map(u64::from)),
                        hex(d.counter.map(u64::from))
                    );
                }
            }
        }
    }

    /// `KAT_G6_KEYS=<file listing key files>`: the upload KAT sends for each (the captured
    /// button; Nice FloR-S / Alutech: the C's upload without a table), as signed µs.
    #[test]
    #[ignore]
    fn g6_dump_encodes() {
        let reg = ProtocolRegistry::new();
        for path in list("KAT_G6_KEYS") {
            let key = FlipperKey::parse(&std::fs::read_to_string(&path).unwrap()).unwrap();
            let decoder = reg.get(&key.protocol).unwrap();
            let Some(d) = decoder.from_flipper_key(&key) else {
                println!("ENC\t{path}\tLOAD-ERR");
                continue;
            };
            let wire = match decoder.name() {
                "Nice FloR-S" => nice_flor_s::c_upload_without_table(&d),
                "Alutech AT-4N" => Some(alutech_at_4n::c_upload_without_table()),
                _ => decoder.encode(&d, d.button.unwrap_or(0)),
            };
            let text: Vec<String> = wire
                .unwrap_or_default()
                .iter()
                .map(|p| if p.level { p.duration_us.to_string() } else { format!("-{}", p.duration_us) })
                .collect();
            println!("ENC\t{path}\t{}", text.join(" "));
        }
    }

    /// xorshift64* (deterministic, no crates).
    struct Rng(u64);

    impl Rng {
        fn next(&mut self) -> u64 {
            self.0 ^= self.0 >> 12;
            self.0 ^= self.0 << 25;
            self.0 ^= self.0 >> 27;
            self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
        }
    }

    fn write_raw(path: &std::path::Path, pulses: &[LevelDuration]) {
        let mut text = String::from(
            "Filetype: Flipper SubGhz RAW File\nVersion: 1\nFrequency: 433920000\n\
             Preset: FuriHalSubGhzPresetOok650Async\nProtocol: RAW\n",
        );
        for chunk in pulses.chunks(512) {
            let v: Vec<String> = chunk
                .iter()
                .map(|p| if p.level { p.duration_us.to_string() } else { format!("-{}", p.duration_us) })
                .collect();
            text += &format!("RAW_Data: {}\n", v.join(" "));
        }
        std::fs::write(path, text).unwrap();
    }

    /// `KAT_G6_OUT=<dir>` (`KAT_G6_N` frames per protocol, default 40): random key files (the
    /// references' format, from `to_flipper_key`) and the RAW uploads KAT's encoders make for
    /// them (Nice FloR-S / Alutech with synthetic tables), as fuzzing seeds.
    #[test]
    #[ignore]
    fn g6_write_corpus() {
        let out = std::path::PathBuf::from(std::env::var("KAT_G6_OUT").expect("KAT_G6_OUT"));
        let n: usize = std::env::var("KAT_G6_N").ok().and_then(|v| v.parse().ok()).unwrap_or(40);
        std::fs::create_dir_all(out.join("keys")).unwrap();
        std::fs::create_dir_all(out.join("raw")).unwrap();
        let mut rng = Rng(0x6A09_E667_F3BC_C908);
        let nice_table: Table = std::array::from_fn(|i| (i as u8).wrapping_mul(37).wrapping_add(11));
        let alu_table: Table = {
            let delta = 0x9E37_79B9u32;
            let w = [delta.wrapping_mul(32), 0xA56B_3C91, 0x1F2E_3D4C, delta.wrapping_neg(), 0x7788_99AA, 0x0BAD_F00D, delta, 0];
            std::array::from_fn(|i| w[i / 4].to_be_bytes()[i % 4])
        };
        let nice = nice_flor_s::NiceFlorSDecoder::with_table(Some(nice_table));
        let alu = alutech_at_4n::AlutechAt4nDecoder::with_table(Some(alu_table));
        let came = came_atomo::CameAtomoDecoder::new();
        let ditec = ditec_gol4::DitecGol4Decoder::new();
        for i in 0..n {
            let r = rng.next();
            // Nice FloR-S: a parcel-0 frame of random fields (52 or 72 bits, sometimes O-Code /
            // CounterMode), decoded from the upload the table encoder makes for it.
            let btn = [1u8, 2, 4, 8, 3][(r % 5) as usize];
            let mut text = format!(
                "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                 Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Nice FloR-S\nBit: {}\nKey: {}\n",
                if i % 2 == 0 { 52 } else { 72 },
                hexbytes(((u64::from(btn) << 4 | u64::from(0xF ^ btn)) << 44) | (rng.next() & ((1 << 44) - 1)))
            );
            if i % 2 == 1 {
                text += &format!("Data: {}\n", rng.next() & 0xF_FFFF);
            }
            if i % 7 == 3 {
                text += &format!("IC: {:02X} {:02X}\n", rng.next() as u8, rng.next() as u8);
            }
            if i % 5 == 4 {
                text += &format!("CounterMode: {}\n", rng.next() % 3);
            }
            write_key(&out, "nice_flor_s", i, &nice, &text);
            // Alutech AT-4N: valid frames (synthetic table) and raw random keys.
            let key_text = if i % 3 == 0 {
                format!(
                    "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                     Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Alutech AT-4N\nBit: 72\nKey: {}\nCRC: {}\n",
                    hexbytes(rng.next()),
                    rng.next() & 0xFF
                )
            } else {
                let d = alu_frame(&alu, &mut rng);
                alu.to_flipper_key(&d).unwrap().to_text()
            };
            write_key(&out, "alutech_at_4n", i, &alu, &key_text);
            // CAME Atomo: random received keys (61 bits, leading 1) and CounterModes.
            let mut text = format!(
                "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                 Preset: FuriHalSubGhzPresetOok650Async\nProtocol: CAME Atomo\nBit: 62\nKey: {}\n",
                hexbytes((1 << 60) | (rng.next() & ((1 << 60) - 1)))
            );
            if i % 4 == 3 {
                text += &format!("CounterMode: {}\n", rng.next() % 4);
            }
            write_key(&out, "came_atomo", i, &came, &text);
            // Ditec GOL4: random 54-bit keys.
            let text = format!(
                "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                 Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Ditec GOL4\nBit: 54\nKey: {}\n",
                hexbytes(rng.next() & ((1 << 54) - 1))
            );
            write_key(&out, "ditec_gol4", i, &ditec, &text);
        }

        fn hexbytes(v: u64) -> String {
            v.to_be_bytes().iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(" ")
        }

        fn alu_frame(alu: &alutech_at_4n::AlutechAt4nDecoder, rng: &mut Rng) -> DecodedSignal {
            let btn = [0xFFu8, 0x11, 0x22, 0x33, 0x44][(rng.next() % 5) as usize];
            alu.test_frame(rng.next() as u32, rng.next() as u16, btn)
        }

        /// The key file, plus the encoder's RAW upload when it can transmit it.
        fn write_key(out: &std::path::Path, proto: &str, i: usize, dec: &dyn ProtocolDecoder, text: &str) {
            std::fs::write(out.join("keys").join(format!("{proto}_{i:03}.sub")), text).unwrap();
            let key = FlipperKey::parse(text).unwrap();
            if let Some(d) = dec.from_flipper_key(&key) {
                if let Some(wire) = dec.encode(&d, d.button.unwrap_or(0)) {
                    write_raw(&out.join("raw").join(format!("{proto}_{i:03}.sub")), &wire);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::{alutech_at_4n, came_atomo, ditec_gol4, nice_flor_s};
    use crate::protocols::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolRegistry};

    /// Every G6 encoder's upload, run through the whole registry (car decoders first), decodes
    /// only as its own protocol: nothing earlier in the registry steals or splits these frames.
    #[test]
    fn registry_attributes_g6_uploads_to_their_own_protocol() {
        let nice_table: Table = std::array::from_fn(|i| (i as u8).wrapping_mul(29).wrapping_add(3));
        let alu_table: Table = {
            let delta = 0x9E37_79B9u32;
            let w = [delta.wrapping_mul(32), 1, 2, delta.wrapping_neg(), 3, 4, delta, 0];
            std::array::from_fn(|i| w[i / 4].to_be_bytes()[i % 4])
        };
        let key = |proto: &str, bits: usize, key: u64, extra: &str| {
            FlipperKey::parse(&format!(
                "Protocol: {proto}\nBit: {bits}\nKey: {}\n{extra}",
                key.to_be_bytes().iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(" ")
            ))
            .unwrap()
        };
        let nice = nice_flor_s::NiceFlorSDecoder::with_table(Some(nice_table));
        let alu = alutech_at_4n::AlutechAt4nDecoder::with_table(Some(alu_table));
        let came = came_atomo::CameAtomoDecoder::new();
        let ditec = ditec_gol4::DitecGol4Decoder::new();
        let cases: Vec<(&str, &dyn ProtocolDecoder, DecodedSignal)> = vec![
            ("Nice FloR-S", &nice, nice.from_flipper_key(&key("Nice FloR-S", 52, 0x2D_0123_4567_89AB, "")).unwrap()),
            ("Nice One", &nice, nice.from_flipper_key(&key("Nice FloR-S", 72, 0x2D_0123_4567_89AB, "Data: 123456\n")).unwrap()),
            ("Alutech AT-4N", &alu, alu.test_frame(0x1234_5678, 0x0100, 0x22)),
            ("CAME Atomo", &came, came.from_flipper_key(&key("CAME Atomo", 62, 0x1FF0_8D99_2498_4115, "")).unwrap()),
            ("Ditec GOL4", &ditec, ditec.from_flipper_key(&key("Ditec GOL4", 54, 0x000C_CB2F_8320_ED66, "")).unwrap()),
        ];
        for (name, dec, d) in cases {
            let wire = dec.encode(&d, d.button.unwrap_or(0)).unwrap();
            for freq in crate::protocols::test_frequencies(crate::protocols::ANY_FREQUENCY) {
                let names: Vec<String> =
                    ProtocolRegistry::new().process_signal_stream(&wire, freq).into_iter().map(|(n, _, _)| n).collect();
                assert!(!names.is_empty() && names.iter().all(|n| n == name), "{name} @ {freq}: {names:?}");
            }
        }
    }

    #[test]
    fn push_merges_equal_levels() {
        let mut out = Vec::new();
        push(&mut out, true, 100);
        push(&mut out, true, 50);
        push(&mut out, false, 20);
        assert_eq!(tuples(&out), vec![(true, 150), (false, 20)]);
    }

    #[test]
    fn counter_sequences() {
        assert_eq!(counter_increment(0x1233), 0x1234);
        assert_eq!(counter_increment(0xFFFF), 0);
        let mut seq = vec![0u32];
        for _ in 0..5 {
            seq.push(counter_ofex(*seq.last().unwrap()));
        }
        assert_eq!(seq, [0, 1, 0xFFFE, 0xFFFF, 0, 1]);
        assert_eq!(counter_ofex(0x1234), 0xFFFE);
    }
}
