//! Helpers shared by the gate and alarm remotes ported from doctor-firmware, Flipper-ARF, Momentum
//! and RogueMaster (GangQi, Hollarm, Hörmann BiSecur, Cardin S508, Freezer, Faac TM): upload building
//! as the C encoders' `yield` sends it, and the test / oracle-parity tools.

use crate::radio::demodulator::LevelDuration;

/// Append one level to an upload, merging it into the previous entry when the level repeats (two
/// adjacent same-level durations are one pulse on air).
pub(super) fn push(out: &mut Vec<LevelDuration>, level: bool, duration: u32) {
    match out.last_mut() {
        Some(last) if last.level == level => last.duration_us += duration,
        _ => out.push(LevelDuration::new(level, duration)),
    }
}

/// What the encoder's `yield` sends: `one` upload `repeat` times back to back (equal levels at the
/// seams merged).
pub(super) fn with_repeats(one: &[LevelDuration], repeat: usize) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(one.len() * repeat);
    for _ in 0..repeat {
        for p in one {
            push(&mut out, p.level, p.duration_us);
        }
    }
    out
}

/// `subghz_protocol_blocks_add_bit`: shift a bit into the decoder's 64-bit register; the bit count
/// is the C's `uint8_t decode_count_bit` (wraps after 255).
pub(super) fn add_bit(data: &mut u64, count: &mut u8, bit: bool) {
    *data = (*data << 1) | u64::from(bit);
    *count = count.wrapping_add(1);
}

#[cfg(test)]
pub(super) mod test_support {
    //! Test helpers for the F modules.
    use crate::protocols::{DecodedSignal, ProtocolDecoder, ProtocolRegistry};
    use crate::radio::demodulator::LevelDuration;
    use std::path::{Path, PathBuf};

    /// Flipper-ARF's SubGHz unit-test resources (REFERENCES is a gitignored clone).
    pub const ARF_UNIT_TESTS: &str =
        "REFERENCES/Flipper-ARF/applications/debug/unit_tests/resources/unit_tests/subghz";
    /// Momentum's (the only unit-test files with Hörmann BiSecur captures, also in RogueMaster's).
    pub const MOMENTUM_UNIT_TESTS: &str =
        "REFERENCES/Momentum-Firmware/applications/debug/unit_tests/resources/unit_tests/subghz";

    /// `dir/name` when it exists (None and a note when REFERENCES is absent, e.g. in CI).
    pub fn reference_file(dir: &str, name: &str) -> Option<PathBuf> {
        let path = Path::new(dir).join(name);
        if path.exists() {
            Some(path)
        } else {
            eprintln!("Skip: {path:?} not found (REFERENCES absent)");
            None
        }
    }

    /// Feed `wire` to `dec` and collect what it reports.
    pub fn feed_all(dec: &mut dyn ProtocolDecoder, wire: &[LevelDuration]) -> Vec<DecodedSignal> {
        wire.iter()
            .filter_map(|p| dec.feed(p.level, p.duration_us))
            .collect()
    }

    /// Feed `wire` to `dec`, resetting it after each decode (what the C oracle's
    /// `decode --protocol NAME` does: the unit test's callback resets the receiver).
    pub fn feed_resetting(
        dec: &mut dyn ProtocolDecoder,
        wire: &[LevelDuration],
    ) -> Vec<DecodedSignal> {
        dec.reset();
        let mut out = Vec::new();
        for p in wire {
            if let Some(d) = dec.feed(p.level, p.duration_us) {
                out.push(d);
                dec.reset();
            }
        }
        out
    }

    /// A RAW `.sub` file's pulses as KAT imports them.
    pub fn sub_pulses(path: &Path) -> (u32, Vec<LevelDuration>) {
        let sub =
            crate::export::flipper::import_sub(path).unwrap_or_else(|e| panic!("{path:?}: {e}"));
        let pulses = sub
            .pairs
            .iter()
            .map(|p| LevelDuration::new(p.level, p.duration_us))
            .collect();
        (sub.frequency, pulses)
    }

    /// A RAW `.sub` file decoded through the whole registry the way the app imports it (gated by
    /// the file's preset modulation): (display name, decode) of every frame.
    pub fn registry_decodes(path: &Path) -> Vec<(String, DecodedSignal)> {
        crate::protocols::keys::load_keystore_from_embedded();
        let sub =
            crate::export::flipper::import_sub(path).unwrap_or_else(|e| panic!("{path:?}: {e}"));
        let pulses: Vec<LevelDuration> = sub
            .pairs
            .iter()
            .map(|p| LevelDuration::new(p.level, p.duration_us))
            .collect();
        let rf = sub.preset.as_ref().and_then(|p| p.rf());
        let mut reg = ProtocolRegistry::new();
        reg.process_signal_stream_rf(&pulses, sub.frequency, rf)
            .into_iter()
            .map(|(n, d, _)| (n, d))
            .collect()
    }

    /// Deterministic xorshift64* (no extra crates).
    pub struct Rng(pub u64);

    impl Rng {
        pub fn next(&mut self) -> u64 {
            self.0 ^= self.0 >> 12;
            self.0 ^= self.0 << 25;
            self.0 ^= self.0 >> 27;
            self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
        }

        pub fn below(&mut self, n: u64) -> u64 {
            self.next() % n.max(1)
        }
    }

    /// Jitter every duration by up to ±`amount` µs (deterministic).
    pub fn jitter(wire: &[LevelDuration], rng: &mut Rng, amount: u32) -> Vec<LevelDuration> {
        wire.iter()
            .map(|p| {
                let delta = rng.below(u64::from(2 * amount) + 1) as i64 - i64::from(amount);
                LevelDuration::new(p.level, (i64::from(p.duration_us) + delta).max(1) as u32)
            })
            .collect()
    }
}

#[cfg(test)]
mod parity {
    //! Oracle-parity tools (`#[ignore]`d; see the protocols' docs, "Validation"):
    //! - `F_FILES=<list>` (one RAW `.sub` path per line): feed every F decoder alone over each file,
    //!   resetting it after each decode like the C oracle's `decode --protocol NAME`, and print
    //!   `<file> | <protocol> | <key-file lines>` (the oracle's layout, Frequency/Preset left out);
    //!   `F_PROTO=<name>` keeps one protocol.
    //! - `F_KEYS=<list>` (key files): load each, encode (captured button) and print the upload as
    //!   `<file> | <protocol> | <signed µs, equal levels merged>`.
    //! - `F_FUZZ=<dir>`: write fuzz `.sub` files (each encoder's uploads with jitter, dropped and
    //!   split pulses, spliced into each other) for both sides to decode.
    use super::super::{flipper_key::FlipperKey, DecodedSignal, ProtocolDecoder, ProtocolRegistry};
    use super::test_support::*;
    use crate::radio::demodulator::LevelDuration;

    const NAMES: [&str; 14] = [
        "Hormann BiSecur", "Cardin S508", "GangQi", "Hollarm", "Freezer", "Faac TM", "Magellan", "X10", "Legrand",
        "Doorbell32", "Intertechno_V3", "KeyFinder", "Feron", "KeyFinder2",
    ];

    fn decoder(name: &str) -> Box<dyn ProtocolDecoder> {
        match name {
            "GangQi" => Box::new(super::super::gangqi::GangQiDecoder::new()),
            "Hollarm" => Box::new(super::super::hollarm::HollarmDecoder::new()),
            "Hormann BiSecur" => {
                Box::new(super::super::hormann_bisecur::HormannBiSecurDecoder::new())
            }
            "Cardin S508" => Box::new(super::super::cardin_s508::CardinS508Decoder::new()),
            "Freezer" => Box::new(super::super::freezer::FreezerDecoder::new()),
            "Faac TM" => Box::new(super::super::faac_tm::FaacTmDecoder::new()),
            "Magellan" => Box::new(super::super::magellan::MagellanDecoder::new()),
            "X10" => Box::new(super::super::x10::X10Decoder::new()),
            "Legrand" => Box::new(super::super::legrand::LegrandDecoder::new()),
            "Doorbell32" => Box::new(super::super::doorbell32::Doorbell32Decoder::new()),
            "Intertechno_V3" => Box::new(super::super::intertechno_v3::IntertechnoV3Decoder::new()),
            "KeyFinder" => Box::new(super::super::keyfinder::KeyFinderDecoder::new()),
            "Feron" => Box::new(super::super::feron::FeronDecoder::new()),
            "KeyFinder2" => Box::new(super::super::keyfinder2::KeyFinder2Decoder::new()),
            _ => unreachable!("{name}"),
        }
    }

    fn list(var: &str) -> Vec<String> {
        let Ok(path) = std::env::var(var) else {
            return Vec::new();
        };
        std::fs::read_to_string(path)
            .unwrap()
            .lines()
            .filter(|l| !l.trim().is_empty())
            .map(str::to_string)
            .collect()
    }

    fn names() -> Vec<&'static str> {
        let only = std::env::var("F_PROTO").ok();
        NAMES
            .iter()
            .copied()
            .filter(|n| only.as_deref().is_none_or(|o| o == *n))
            .collect()
    }

    /// The key-file lines after `Protocol` (Bit, Key, the protocol's own), joined like the oracle's.
    fn key_lines(dec: &dyn ProtocolDecoder, d: &DecodedSignal) -> String {
        let Some(key) = dec.to_flipper_key(d) else {
            return "(no key file)".to_string();
        };
        key.to_text()
            .lines()
            .skip_while(|l| !l.starts_with("Protocol:"))
            .collect::<Vec<_>>()
            .join("; ")
    }

    #[test]
    #[ignore]
    fn f_parity_decode_dump() {
        for file in list("F_FILES") {
            let (_, wire) = sub_pulses(std::path::Path::new(&file));
            let mut any = false;
            for name in names() {
                let mut dec = decoder(name);
                for d in feed_resetting(dec.as_mut(), &wire) {
                    any = true;
                    println!("{file} | {name} | {}", key_lines(dec.as_ref(), &d));
                }
            }
            if !any {
                println!("{file} | (none)");
            }
        }
    }

    #[test]
    #[ignore]
    fn f_parity_encode_dump() {
        let reg = ProtocolRegistry::new();
        for file in list("F_KEYS") {
            let text = std::fs::read_to_string(&file).unwrap();
            let Ok(key) = FlipperKey::parse(&text) else {
                println!("{file} | (not a key file)");
                continue;
            };
            let Some((name, d)) = reg.decode_key(&key) else {
                println!("{file} | {} | (does not load)", key.protocol);
                continue;
            };
            let Some(upload) = reg
                .get(&name)
                .and_then(|p| p.encode(&d, d.button.unwrap_or(0)))
            else {
                println!("{file} | {name} | (no encoder)");
                continue;
            };
            let values: Vec<String> = upload
                .iter()
                .map(|p| {
                    if p.level {
                        p.duration_us.to_string()
                    } else {
                        format!("-{}", p.duration_us)
                    }
                })
                .collect();
            println!("{file} | {name} | {}", values.join(" "));
        }
    }

    /// A Flipper RAW `.sub` file of `wire` (HIGH positive, LOW negative, ≤ 512 values a line).
    fn write_sub(path: &std::path::Path, frequency: u32, preset: &str, wire: &[LevelDuration]) {
        let mut text = format!(
            "Filetype: Flipper SubGhz RAW File\nVersion: 1\nFrequency: {frequency}\nPreset: {preset}\nProtocol: RAW\n"
        );
        let values: Vec<String> = wire
            .iter()
            .map(|p| {
                if p.level {
                    p.duration_us.to_string()
                } else {
                    format!("-{}", p.duration_us)
                }
            })
            .collect();
        for chunk in values.chunks(512) {
            text += &format!("RAW_Data: {}\n", chunk.join(" "));
        }
        std::fs::write(path, text).unwrap();
    }

    /// Random frames of each F protocol (as `DecodedSignal`s their encoders take).
    fn random_frame(name: &str, rng: &mut Rng) -> DecodedSignal {
        match name {
            "GangQi" => {
                super::super::gangqi::testing::frame(rng.next() as u16, (rng.next() & 0xF) as u8)
            }
            "Hollarm" => super::super::hollarm::testing::frame(
                rng.next() as u32 & 0x0FFF_FFFF,
                (rng.next() & 0xF) as u8,
            ),
            "Hormann BiSecur" => {
                let mut bytes = [0u8; 22];
                for b in &mut bytes {
                    *b = rng.next() as u8;
                }
                bytes[0] = if rng.next() & 1 == 0 { 0x50 } else { 0x70 };
                super::super::hormann_bisecur::testing::with_valid_crc(bytes)
            }
            _ => unreachable!(),
        }
    }

    /// `wire` with one of: jitter, a dropped pulse, a pulse split in two, two pulses merged.
    fn perturb(wire: &[LevelDuration], rng: &mut Rng) -> Vec<LevelDuration> {
        let mut w = wire.to_vec();
        match rng.below(5) {
            0 => w = jitter(&w, rng, 60),
            1 => w = jitter(&w, rng, 150),
            2 if w.len() > 4 => {
                w.remove(rng.below(w.len() as u64) as usize);
            }
            3 if !w.is_empty() => {
                let i = rng.below(w.len() as u64) as usize;
                let d = w[i].duration_us;
                let cut = 1 + rng.below(u64::from(d.max(2) - 1)) as u32;
                w[i].duration_us = cut;
                w.insert(
                    i + 1,
                    LevelDuration::new(!w[i].level, 40 + rng.below(80) as u32),
                );
                w.insert(
                    i + 2,
                    LevelDuration::new(w[i].level, d.saturating_sub(cut).max(1)),
                );
            }
            _ => {
                if let Some(p) = w.get_mut(rng.below(wire.len().max(1) as u64) as usize) {
                    p.duration_us = (p.duration_us * (50 + rng.below(100) as u32)) / 100 + 1;
                }
            }
        }
        // Keep levels alternating: merge equal neighbours.
        let mut out: Vec<LevelDuration> = Vec::with_capacity(w.len());
        for p in w {
            super::push(&mut out, p.level, p.duration_us);
        }
        out
    }

    /// `F_KEYGEN=<dir>`: random key files of the three encodable F protocols, written as KAT
    /// writes them (`to_flipper_key`), for `encode` parity. GangQi / Hollarm keys also get
    /// arbitrary low bits / byte sums (the encoders rebuild them), BiSecur both frame types.
    #[test]
    #[ignore]
    fn f_keygen() {
        let Some(dir) = std::env::var_os("F_KEYGEN") else {
            return;
        };
        let dir = std::path::PathBuf::from(dir);
        std::fs::create_dir_all(&dir).unwrap();
        let mut rng = Rng(0x0DDB_A11C_AFE5_1234);
        let reg = ProtocolRegistry::new();
        for name in ["GangQi", "Hollarm", "Hormann BiSecur"] {
            let dec = reg.get(name).unwrap();
            for i in 0..100 {
                let mut d = random_frame(name, &mut rng);
                if i % 3 == 1 && name != "Hormann BiSecur" {
                    // arbitrary bits below the button (the encoder rewrites them)
                    let mask = if name == "GangQi" { 0x3FF } else { 0xFF };
                    d.data ^= rng.next() & mask;
                }
                let mut key = dec.to_flipper_key(&d).unwrap();
                key.frequency = if name == "Hormann BiSecur" {
                    868_276_000
                } else {
                    433_920_000
                };
                key.preset = Some(
                    if name == "Hormann BiSecur" {
                        "FuriHalSubGhzPreset2FSKDev476Async"
                    } else {
                        "FuriHalSubGhzPresetOok650Async"
                    }
                    .to_string(),
                );
                std::fs::write(
                    dir.join(format!("{}_{i:03}.sub", name.replace(' ', "_"))),
                    key.to_text(),
                )
                .unwrap();
            }
        }
    }

    #[test]
    #[ignore]
    fn f_fuzz_gen() {
        let Some(dir) = std::env::var_os("F_FUZZ") else {
            return;
        };
        let dir = std::path::PathBuf::from(dir);
        std::fs::create_dir_all(&dir).unwrap();
        let count: usize = std::env::var("F_FUZZ_COUNT")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(200);
        let mut rng = Rng(0x5EED_F00D_1234_5678);
        let reg = ProtocolRegistry::new();
        for name in ["GangQi", "Hollarm", "Hormann BiSecur"] {
            let enc = reg.get(name).unwrap();
            for i in 0..count {
                let mut wire = vec![LevelDuration::new(false, 20_000)];
                // 1..3 frames of this protocol (some with another button), perturbed.
                for _ in 0..1 + rng.below(3) {
                    let d = random_frame(name, &mut rng);
                    // One frame in four with a wrong byte sum / CRC (C: reported; KAT: dropped).
                    let up = match name {
                        "GangQi" if rng.below(4) == 0 => super::with_repeats(
                            &super::super::gangqi::testing::raw_upload(
                                d.data ^ (1 + rng.below(0xFF)) << 2,
                            ),
                            3,
                        ),
                        "Hormann BiSecur" if rng.below(4) == 0 => {
                            let mut bytes = [0u8; 22];
                            for b in &mut bytes {
                                *b = rng.next() as u8;
                            }
                            bytes[0] = [0x50, 0x70, rng.next() as u8][rng.below(3) as usize] & 0x7F;
                            super::super::hormann_bisecur::testing::raw_upload(&bytes)
                        }
                        _ => enc.encode(&d, d.button.unwrap_or(0)).unwrap(),
                    };
                    let up = if rng.below(3) == 0 {
                        up
                    } else {
                        perturb(&up, &mut rng)
                    };
                    for p in up
                        .into_iter()
                        .chain([LevelDuration::new(false, 5_000 + rng.below(20_000) as u32)])
                    {
                        super::push(&mut wire, p.level, p.duration_us);
                    }
                }
                let (freq, preset) = if name == "Hormann BiSecur" {
                    (868_276_000, "FuriHalSubGhzPreset2FSKDev476Async")
                } else {
                    (433_920_000, "FuriHalSubGhzPresetOok650Async")
                };
                let stem = name.replace(' ', "_");
                write_sub(&dir.join(format!("{stem}_{i:04}.sub")), freq, preset, &wire);
            }
        }
        // Cardin S508 has no encoder: synthetic presses (both bit conventions, symmetric and
        // duty-cycle-biased timing, 1..3 repeats), perturbed.
        for i in 0..count {
            let timing = if rng.below(2) == 0 {
                [100, 200, 100, 200]
            } else {
                [140, 225, 95, 175]
            };
            let press = super::super::cardin_s508::testing::press(
                rng.next(),
                rng.next(),
                1 + rng.below(3) as usize,
                rng.below(2) == 0,
                timing,
            );
            let press = if rng.below(3) == 0 {
                press
            } else {
                perturb(&press, &mut rng)
            };
            let mut wire = vec![LevelDuration::new(false, 20_000)];
            for p in press {
                super::push(&mut wire, p.level, p.duration_us);
            }
            write_sub(
                &dir.join(format!("Cardin_S508_{i:04}.sub")),
                868_350_000,
                "FuriHalSubGhzPreset2FSKDev12KAsync",
                &wire,
            );
        }
    }
}
