//! Shared pieces of the G1 static gate/garage protocols (Princeton, CAME, CAME TWEE, Nice FLO,
//! GateTX, Linear, LinearDelta3, MegaCode, Holtek, Holtek_HT12X, Cham_Code, SMC5326), all ported
//! from Flipper-ARF's garage app (`applications/system/subghz_garage/protocols/`):
//!
//! - [reverse_key]: Flipper's `subghz_protocol_blocks_reverse_key`;
//! - [push] / [with_repeats]: the encoder upload with adjacent equal levels merged (identical on
//!   air), repeated `encoder.repeat` times as Flipper's TX loop (`..._yield`) sends it;
//! - [fixed_code] / [generic_key] / [generic_fields]: the stock key-file layout every one of these
//!   protocols uses — Flipper-ARF's `subghz_block_generic_serialize` writes `Protocol`, `Bit` and
//!   `Key` (8 bytes, big-endian) and `subghz_block_generic_deserialize` reads `Bit` and `Key` back;
//!   the TE-carrying protocols (Princeton, Holtek_HT12X, SMC5326) append their own lines.
//!
//! **Decode-callback semantics.** The Flipper SubGHz app (and the unit-test harness) resets the
//! receiver from inside the decoder's callback, so the C code that runs after `callback()` (e.g.
//! `last_data = decode_data`, `parser_step = FoundStartBit`) is applied on top of the reset state.
//! The G1 decoders apply that reset at their emission point themselves and keep the resulting
//! state through the registry's reset that follows an emission (`just_emitted`), so a capture
//! decodes the same frames as on the Flipper.

use super::{DecodedSignal, FlipperKey};
use crate::radio::demodulator::LevelDuration;

/// Preset written into key files KAT builds (the caller replaces frequency and preset).
#[allow(dead_code)] // `to_flipper_key` has no caller in the app yet
pub(super) const KEY_PRESET: &str = "FuriHalSubGhzPresetOok650Async";

/// Flipper `encoder.repeat` of every G1 encoder (Flipper-ARF garage app: 3 for all twelve).
pub(super) const TX_REPEAT: usize = 3;

/// `subghz_protocol_blocks_reverse_key`: the low `bit_count` bits of `key`, mirrored (bits past 63
/// read as 0, where the C shift would be undefined).
pub(super) fn reverse_key(key: u64, bit_count: u8) -> u64 {
    (0..bit_count).fold(0u64, |acc, i| {
        acc << 1 | if i < 64 { (key >> i) & 1 } else { 0 }
    })
}

/// Append one level to an upload, merging it into the previous entry when the level repeats.
pub(super) fn push(out: &mut Vec<LevelDuration>, level: bool, duration: u32) {
    match out.last_mut() {
        Some(last) if last.level == level => last.duration_us += duration,
        _ => out.push(LevelDuration::new(level, duration)),
    }
}

/// `repeat` back-to-back copies of one upload (equal levels at the seams merged).
pub(super) fn with_repeats(one: &[LevelDuration], repeat: usize) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(one.len() * repeat);
    for _ in 0..repeat {
        for ld in one {
            push(&mut out, ld.level, ld.duration_us);
        }
    }
    out
}

/// A fixed-code decode without fields (the frame is the code).
pub(super) fn fixed_code(data: u64, bits: usize) -> DecodedSignal {
    DecodedSignal {
        serial: None,
        button: None,
        counter: None,
        crc_valid: false,
        data,
        data_count_bit: bits,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

/// `subghz_block_generic_serialize`: `Protocol`, `Bit`, `Key` (8 bytes).
#[allow(dead_code)] // `to_flipper_key` has no caller in the app yet
pub(super) fn generic_key(protocol: &str, d: &DecodedSignal) -> FlipperKey {
    FlipperKey::new(protocol, d.data_count_bit, d.data, 433_920_000, KEY_PRESET)
}

/// `subghz_block_generic_deserialize`: (`Bit`, `Key`).
pub(super) fn generic_fields(key: &FlipperKey) -> (usize, u64) {
    (key.bits, key.key)
}

/// Label of a numbered button code as the C displays it (`Btn:%X`): "Btn 0".."Btn F", "Other"
/// beyond (so KAT's generic car names never apply to a gate remote).
pub(super) fn numbered_button(code: u8) -> &'static str {
    const NAMES: [&str; 16] = [
        "Btn 0", "Btn 1", "Btn 2", "Btn 3", "Btn 4", "Btn 5", "Btn 6", "Btn 7", "Btn 8", "Btn 9",
        "Btn A", "Btn B", "Btn C", "Btn D", "Btn E", "Btn F",
    ];
    NAMES.get(code as usize).copied().unwrap_or("Other")
}

/// [numbered_button] as an `INFO.buttons` function.
pub(super) fn numbered_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(numbered_button(code))
}

/// A `uint32` key-file line (`TE: 400`), as `flipper_format_read_uint32` reads it.
pub(super) fn u32_line(key: &FlipperKey, name: &str) -> Option<u32> {
    key.u32_field(name)
}

#[cfg(test)]
pub(super) mod test_support {
    //! Helpers for the G1 module tests.
    use super::super::{DecodedSignal, ProtocolDecoder, ProtocolRegistry};
    use crate::radio::demodulator::LevelDuration;
    use std::path::Path;

    /// Feed `wire` to `dec` and collect what it emits.
    pub fn feed_all(dec: &mut dyn ProtocolDecoder, wire: &[LevelDuration]) -> Vec<DecodedSignal> {
        wire.iter()
            .filter_map(|p| dec.feed(p.level, p.duration_us))
            .collect()
    }

    /// Flipper-ARF's SubGHz unit-test resources (REFERENCES is a gitignored clone).
    pub const UNIT_TEST_DIR: &str =
        "REFERENCES/Flipper-ARF/applications/debug/unit_tests/resources/unit_tests/subghz";

    /// Unit-test capture `name` from Flipper-ARF's test resources, decoded through the full
    /// registry as the app imports it (None when REFERENCES is absent, e.g. in CI).
    pub fn decode_reference_capture(name: &str) -> Option<Vec<(String, DecodedSignal)>> {
        let path = Path::new(UNIT_TEST_DIR).join(name);
        if !path.exists() {
            eprintln!("Skip: {path:?} not found (REFERENCES absent)");
            return None;
        }
        let (freq, raw) = crate::export::flipper::import_sub_raw(&path).ok()?;
        let pairs: Vec<LevelDuration> = raw
            .iter()
            .map(|p| LevelDuration::new(p.level, p.duration_us))
            .collect();
        let mut reg = ProtocolRegistry::new();
        Some(
            reg.process_signal_stream(&pairs, freq)
                .into_iter()
                .map(|(n, d, _)| (n, d))
                .collect(),
        )
    }

    /// Jitter every duration by a deterministic ±`amount` µs pattern.
    pub fn jitter(wire: &[LevelDuration], amount: u32) -> Vec<LevelDuration> {
        wire.iter()
            .enumerate()
            .map(|(i, p)| {
                let d = match i % 3 {
                    0 => p.duration_us + amount,
                    1 => p.duration_us.saturating_sub(amount),
                    _ => p.duration_us,
                };
                LevelDuration::new(p.level, d)
            })
            .collect()
    }
}

#[cfg(test)]
mod parity {
    //! Oracle-parity dumps (`#[ignore]`d tools, see the protocols' docs, "Validation"):
    //! `G1_FILES=<list>` (one `.sub` path per line) feeds every G1 decoder alone, resetting it after
    //! each decode like the C oracle's `decode --protocol NAME`; `G1_KEYS=<list>` loads each key file
    //! and prints the encoder's upload (captured button). Optional `G1_PROTO=<name>` filters.
    use super::super::*;
    use crate::radio::demodulator::LevelDuration;

    const NAMES: [&str; 12] = [
        "Holtek",
        "CAME TWEE",
        "MegaCode",
        "Cham_Code",
        "GateTX",
        "Holtek_HT12X",
        "CAME",
        "Nice FLO",
        "Linear",
        "LinearDelta3",
        "SMC5326",
        "Princeton",
    ];

    fn fresh(name: &str) -> Box<dyn ProtocolDecoder> {
        match name {
            "Holtek" => Box::new(holtek::HoltekDecoder::new()),
            "CAME TWEE" => Box::new(came_twee::CameTweeDecoder::new()),
            "MegaCode" => Box::new(megacode::MegaCodeDecoder::new()),
            "Cham_Code" => Box::new(chamberlain_code::ChamberlainCodeDecoder::new()),
            "GateTX" => Box::new(gate_tx::GateTxDecoder::new()),
            "Holtek_HT12X" => Box::new(holtek_ht12x::HoltekHt12xDecoder::new()),
            "CAME" => Box::new(came::CameDecoder::new()),
            "Nice FLO" => Box::new(nice_flo::NiceFloDecoder::new()),
            "Linear" => Box::new(linear::LinearDecoder::new()),
            "LinearDelta3" => Box::new(linear_delta3::LinearDelta3Decoder::new()),
            "SMC5326" => Box::new(smc5326::Smc5326Decoder::new()),
            _ => Box::new(princeton::PrincetonDecoder::new()),
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
        let only = std::env::var("G1_PROTO").ok();
        NAMES
            .iter()
            .copied()
            .filter(|n| only.is_none() || only.as_deref() == Some(*n))
            .collect()
    }

    /// The key-file view of a decode: `Bit`, `Key` and the protocol's own lines.
    fn key_fields(dec: &dyn ProtocolDecoder, d: &DecodedSignal) -> String {
        let key = dec.to_flipper_key(d).unwrap();
        let mut s = format!("Bit: {}; Key: {:016X}", key.bits, key.key);
        for (n, v) in &key.fields {
            s += &format!("; {n}: {v}");
        }
        s
    }

    #[test]
    #[ignore]
    fn g1_parity_dump() {
        for path in list("G1_FILES") {
            let Ok((_, raw)) = crate::export::flipper::import_sub_raw(std::path::Path::new(&path))
            else {
                println!("{path} | ERR");
                continue;
            };
            for name in names() {
                let mut dec = fresh(name);
                for p in &raw {
                    if let Some(d) = dec.feed(p.level, p.duration_us) {
                        let hex = |v: Option<u64>| v.map_or("-".to_string(), |v| format!("{v:X}"));
                        println!(
                            "{path} | {name} | {} | sn={} btn={} cnt={}",
                            key_fields(dec.as_ref(), &d),
                            hex(d.serial.map(u64::from)),
                            hex(d.button.map(u64::from)),
                            hex(d.counter.map(u64::from))
                        );
                        dec.reset();
                    }
                }
            }
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
        fn below(&mut self, n: u64) -> u64 {
            self.next() % n.max(1)
        }
    }

    /// A random decode the encoder of `name` accepts.
    fn random_frame(name: &str, rng: &mut Rng) -> DecodedSignal {
        let bits = match name {
            "Holtek" => 40,
            "CAME TWEE" => 54,
            "Cham_Code" => 7 + rng.below(3) as usize,
            "Holtek_HT12X" => 12,
            "CAME" => [12, 18, 24, 25, 42][rng.below(5) as usize],
            "Nice FLO" => [12, 24][rng.below(2) as usize],
            "Linear" => 10,
            "LinearDelta3" => 8,
            "SMC5326" => 25,
            _ => 24,
        };
        let mut data = rng.next() & ((1u64 << bits) - 1);
        match name {
            "Holtek" => data = (data & 0x0F_FFFF_FFFF) | 0x50_0000_0000,
            "MegaCode" => data |= 1 << 23,
            "CAME TWEE" => data = 0x003F_FF72_0000_0000 | (data & 0xFFFF_FFF0) | rng.below(15),
            _ => {}
        }
        let te = match name {
            "Holtek_HT12X" => 200 + rng.below(240) as u32,
            "SMC5326" => 180 + rng.below(240) as u32,
            "Princeton" => 250 + rng.below(300) as u32,
            _ => 0,
        };
        let mut key = FlipperKey::new(name, bits, data, 433_920_000, super::KEY_PRESET);
        if te != 0 {
            key.push("TE", te.to_string());
        }
        if name == "Princeton" {
            key.push("Guard_time", (15 + rng.below(58)).to_string());
        }
        fresh(name).from_flipper_key(&key).unwrap()
    }

    /// Perturb a wire: jitter, dropped / merged / split pulses, glitches.
    fn perturb(wire: &[LevelDuration], rng: &mut Rng, strength: u64) -> Vec<LevelDuration> {
        let mut out: Vec<LevelDuration> = Vec::new();
        for p in wire {
            let pct = rng.below(2 * strength + 1) as i64 - strength as i64;
            let d = (p.duration_us as i64 + p.duration_us as i64 * pct / 100).max(1) as u32;
            match rng.below(200) {
                0 => continue, // drop
                1 => {
                    super::push(&mut out, p.level, d / 2); // split by a glitch
                    out.push(LevelDuration::new(!p.level, 20 + rng.below(80) as u32));
                    out.push(LevelDuration::new(p.level, d - d / 2));
                }
                _ => out.push(LevelDuration::new(p.level, d)),
            }
        }
        out
    }

    fn write_sub(path: &std::path::Path, wire: &[LevelDuration]) {
        let mut text = String::from(
            "Filetype: Flipper SubGhz RAW File\nVersion: 1\nFrequency: 433920000\n\
             Preset: FuriHalSubGhzPresetOok650Async\nProtocol: RAW\n",
        );
        for chunk in wire.chunks(512) {
            let v: Vec<String> = chunk
                .iter()
                .map(|p| {
                    if p.level {
                        p.duration_us.to_string()
                    } else {
                        format!("-{}", p.duration_us)
                    }
                })
                .collect();
            text += &format!("RAW_Data: {}\n", v.join(" "));
        }
        std::fs::write(path, text).unwrap();
    }

    /// `G1_FUZZ_DIR=<dir>` (and `G1_FUZZ_N`, default 40 per protocol): perturbed encoder output of
    /// random keys for every G1 protocol, plus random pulse soups, as RAW `.sub` files.
    #[test]
    #[ignore]
    fn g1_fuzz_gen() {
        let dir = std::path::PathBuf::from(std::env::var("G1_FUZZ_DIR").unwrap());
        std::fs::create_dir_all(&dir).unwrap();
        let n: usize = std::env::var("G1_FUZZ_N")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(40);
        let mut rng = Rng(0x0123_4567_89AB_CDEF);
        for name in NAMES {
            for i in 0..n {
                let d = random_frame(name, &mut rng);
                let button = if rng.below(2) == 0 {
                    d.button.unwrap_or(0)
                } else {
                    rng.below(256) as u8
                };
                let one = fresh(name).encode(&d, button).unwrap();
                let mut wire = vec![LevelDuration::new(false, 20_000 + rng.below(40_000) as u32)];
                for _ in 0..(1 + rng.below(3)) {
                    wire.extend_from_slice(&one);
                }
                wire.push(LevelDuration::new(false, 30_000 + rng.below(30_000) as u32));
                let strength = [0, 5, 15, 30][i % 4];
                let wire = perturb(&wire, &mut rng, strength);
                let file = dir.join(format!("{}_{i:03}.sub", name.replace(' ', "_")));
                write_sub(&file, &wire);
            }
        }
        // Pulse soups: durations near the G1 time bases, and uniform noise.
        let bases = [
            300u32, 320, 350, 390, 430, 500, 640, 700, 870, 900, 1000, 1170, 1400, 1500, 2000, 3000,
        ];
        for i in 0..(n * 6) {
            let mut level = rng.below(2) == 0;
            let mut wire = Vec::new();
            for _ in 0..(200 + rng.below(800)) {
                let d = if i % 3 == 0 {
                    50 + rng.below(40_000) as u32
                } else if rng.below(40) == 0 {
                    3_000 + rng.below(50_000) as u32
                } else {
                    let b = bases[rng.below(bases.len() as u64) as usize] as i64;
                    (b * (1 + rng.below(3) as i64) + rng.below(b as u64 / 2) as i64 - b / 4).max(1)
                        as u32
                };
                wire.push(LevelDuration::new(level, d.max(1)));
                level = !level;
            }
            write_sub(&dir.join(format!("soup_{i:03}.sub")), &wire);
        }
    }

    /// `G1_KEYS_DIR=<dir>`: `to_flipper_key` files of random frames for every G1 protocol (the
    /// encoder-parity corpus: load in the C oracle and in KAT, compare the uploads).
    #[test]
    #[ignore]
    fn g1_keys_gen() {
        let dir = std::path::PathBuf::from(std::env::var("G1_KEYS_DIR").unwrap());
        std::fs::create_dir_all(&dir).unwrap();
        let mut rng = Rng(0xFEED_FACE_CAFE_BEEF);
        for name in NAMES {
            for i in 0..25 {
                let d = random_frame(name, &mut rng);
                let key = fresh(name).to_flipper_key(&d).unwrap();
                std::fs::write(
                    dir.join(format!("{}_{i:02}.sub", name.replace(' ', "_"))),
                    key.to_text(),
                )
                .unwrap();
            }
        }
    }

    #[test]
    #[ignore]
    fn g1_encode_dump() {
        let reg = ProtocolRegistry::new();
        for path in list("G1_KEYS") {
            let text = std::fs::read_to_string(&path).unwrap();
            let Ok(key) = FlipperKey::parse(&text) else {
                println!("{path} | ERR parse");
                continue;
            };
            let Some(dec) = reg.get(&key.protocol) else {
                println!("{path} | ERR protocol {}", key.protocol);
                continue;
            };
            let Some(d) = dec.from_flipper_key(&key) else {
                println!("{path} | {} | ERR load", dec.name());
                continue;
            };
            let button = std::env::var("G1_BUTTON")
                .ok()
                .and_then(|b| u8::from_str_radix(&b, 16).ok());
            let Some(wire) = dec.encode(&d, button.or(d.button).unwrap_or(0)) else {
                println!("{path} | {} | ERR encode", dec.name());
                continue;
            };
            let mut merged: Vec<LevelDuration> = Vec::new();
            for p in wire {
                super::push(&mut merged, p.level, p.duration_us);
            }
            let raw: Vec<String> = merged
                .iter()
                .map(|p| {
                    if p.level {
                        format!("{}", p.duration_us)
                    } else {
                        format!("-{}", p.duration_us)
                    }
                })
                .collect();
            println!("{path} | {} | {}", dec.name(), raw.join(" "));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reverse_key_matches_flipper() {
        assert_eq!(reverse_key(0b1101, 4), 0b1011);
        assert_eq!(reverse_key(0x95D5D4, 24), 0x2BABA9);
        assert_eq!(reverse_key(1, 64), 1 << 63);
        assert_eq!(reverse_key(0xFFFF, 0), 0);
    }

    #[test]
    fn uploads_merge_equal_levels() {
        let mut out = Vec::new();
        push(&mut out, false, 100);
        push(&mut out, false, 50);
        push(&mut out, true, 10);
        assert_eq!(out.len(), 2);
        assert_eq!(out[0].duration_us, 150);
        let rep = with_repeats(
            &[
                LevelDuration::new(false, 5),
                LevelDuration::new(true, 1),
                LevelDuration::new(false, 2),
            ],
            2,
        );
        let got: Vec<(bool, u32)> = rep.iter().map(|l| (l.level, l.duration_us)).collect();
        assert_eq!(
            got,
            [(false, 5), (true, 1), (false, 7), (true, 1), (false, 2)]
        );
    }
}
