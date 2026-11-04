//! Shared pieces of the G2 static gate/garage protocols (Marantec, Marantec24, BETT, Doitrand,
//! Clemsa, Ansonic, Mastercode, Dickert MAHS, Nero Sketch, Nero Radio, iDo 117/111, Hörmann HSM,
//! Elplast): the Flipper `lib/subghz/blocks` helpers their C files call, and the key-file layout of
//! Flipper-ARF's `subghz_block_generic_serialize` / `subghz_block_generic_deserialize`.
//!
//! **Key files**: every one of these protocols serializes through the generic block, which in
//! Flipper-ARF (and its `subghz_garage` app) writes only `Protocol`, `Bit` and `Key` (the 64-bit
//! `generic.data`, 8 bytes big-endian). doctor-firmware's generic block also appends
//! `Serial`/`Btn`/`Cnt` (and `Key_2`) when any of them is non-zero, but its SubGHz app serializes a
//! decode *before* calling `get_string`, which is what fills those fields here, so they are absent
//! for a session's first decode and stale (the previous frame's) afterwards; KAT writes the ARF
//! layout. Loading reads `Bit` + `Key` (the protocol checks the bit count, as its C `deserialize`
//! does) and ignores any other line.

use super::{DecodedSignal, FlipperKey};
use crate::radio::demodulator::LevelDuration;

/// Flipper's `DURATION_DIFF(duration, target) < delta`.
#[inline]
pub(super) fn near(duration: u32, target: u32, delta: u32) -> bool {
    duration.abs_diff(target) < delta
}

/// The `decode_data` / `decode_count_bit` pair of Flipper's `SubGhzBlockDecoder`:
/// `subghz_protocol_blocks_add_bit` shifts a bit into a 64-bit register (older bits fall off the
/// top) and counts it in a `uint8_t` (wrapping like the C).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub(super) struct BitReg {
    pub data: u64,
    pub count: u8,
}

impl BitReg {
    /// `decode_data = 0; decode_count_bit = 0`.
    pub fn clear(&mut self) {
        *self = Self::default();
    }

    /// `subghz_protocol_blocks_add_bit`.
    pub fn add(&mut self, bit: bool) {
        self.data = (self.data << 1) | u64::from(bit);
        self.count = self.count.wrapping_add(1);
    }
}

/// `bit_read(key, bit)` for a 64-bit key; a bit index ≥ 64 reads 0 (the ARM shift the Flipper
/// executes; C leaves it undefined).
#[inline]
pub(super) fn bit(key: u64, index: usize) -> bool {
    key.checked_shr(index as u32).is_some_and(|v| v & 1 != 0)
}

/// `subghz_protocol_blocks_reverse_key(key, bit_count)`: the low `bits` bits, mirrored.
pub(super) fn reverse_key(key: u64, bits: usize) -> u64 {
    (0..bits).fold(0u64, |acc, i| (acc << 1) | u64::from(bit(key, i)))
}

/// Append one level, merging with the previous entry when the level repeats (identical on air).
pub(super) fn push(out: &mut Vec<LevelDuration>, level: bool, duration: u32) {
    match out.last_mut() {
        Some(last) if last.level == level => last.duration_us += duration,
        _ => out.push(LevelDuration::new(level, duration)),
    }
}

/// The transmitter's output: the upload sent `repeat` times (`encoder.repeat`), back to back,
/// adjacent equal levels merged.
pub(super) fn repeated(upload: &[LevelDuration], repeat: usize) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(upload.len() * repeat);
    for _ in 0..repeat {
        for ld in upload {
            push(&mut out, ld.level, ld.duration_us);
        }
    }
    out
}

/// A decode of a static gate remote (fields filled in by the caller).
pub(super) fn decoded(data: u64, bits: usize) -> DecodedSignal {
    DecodedSignal {
        serial: None,
        button: None,
        counter: None,
        crc_valid: false,
        data,
        data_count_bit: bits,
        encoder_capable: false,
        extra: None,
        protocol_display_name: None,
    }
}

/// The key file Flipper-ARF's generic `serialize` writes: Protocol, Bit, Key (frequency and preset
/// are filled in by the caller).
#[allow(dead_code)] // through `ProtocolDecoder::to_flipper_key`, which the app does not call yet
pub(super) fn key_file(protocol: &str, decoded: &DecodedSignal) -> FlipperKey {
    FlipperKey::new(
        protocol,
        decoded.data_count_bit,
        decoded.data,
        433_920_000,
        "FuriHalSubGhzPresetOok650Async",
    )
}

/// `subghz_block_generic_deserialize`: the key file's `Bit` and `Key` when its `Bit` passes
/// `bits_ok` (the protocol's `deserialize_check_count_bit` or its own rule).
pub(super) fn key_data(key: &FlipperKey, bits_ok: impl Fn(usize) -> bool) -> Option<(u64, usize)> {
    bits_ok(key.bits).then_some((key.key, key.bits))
}

/// Test helpers shared by the G2 modules.
#[cfg(test)]
pub(super) mod test_util {
    use super::super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolRegistry};
    use crate::radio::demodulator::LevelDuration;
    use std::path::Path;

    /// Directory of the Flipper unit-test captures (REFERENCES is a gitignored clone).
    pub const UNIT_TESTS: &str =
        "REFERENCES/Flipper-ARF/applications/debug/unit_tests/resources/unit_tests/subghz";

    /// Every frame `decoder` reports for `wire`, fed from a fresh state.
    pub fn feed_all(
        decoder: &mut dyn ProtocolDecoder,
        wire: &[LevelDuration],
    ) -> Vec<DecodedSignal> {
        decoder.reset();
        wire.iter()
            .filter_map(|p| decoder.feed(p.level, p.duration_us))
            .collect()
    }

    /// Like [feed_all], resetting the decoder after each report as the registry (and the Flipper
    /// receiver) does.
    pub fn feed_with_resets(
        decoder: &mut dyn ProtocolDecoder,
        wire: &[LevelDuration],
    ) -> Vec<DecodedSignal> {
        decoder.reset();
        let mut out = Vec::new();
        for p in wire {
            if let Some(d) = decoder.feed(p.level, p.duration_us) {
                out.push(d);
                decoder.reset();
            }
        }
        out
    }

    /// Signed durations (Flipper RAW_Data style: HIGH positive) to pulses.
    pub fn wire(raw: &[i32]) -> Vec<LevelDuration> {
        raw.iter()
            .map(|&v| LevelDuration::new(v > 0, v.unsigned_abs()))
            .collect()
    }

    /// The registry's decodes (name, signal) of a RAW `.sub` file; None when the file is absent.
    pub fn registry_decodes(path: &str) -> Option<Vec<(String, DecodedSignal)>> {
        let path = Path::new(path);
        if !path.exists() {
            eprintln!("Skip: {path:?} not found (REFERENCES absent)");
            return None;
        }
        let (freq, raw) = crate::export::flipper::import_sub_raw(path).unwrap();
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

    /// `from_flipper_key(parse(to_flipper_key(d)))` gives back `d` (all fields).
    pub fn assert_key_roundtrip(decoder: &dyn ProtocolDecoder, d: &DecodedSignal) {
        let key = decoder.to_flipper_key(d).expect("to_flipper_key");
        let text = key.to_text();
        let back = decoder
            .from_flipper_key(&FlipperKey::parse(&text).unwrap())
            .expect("from_flipper_key");
        assert_eq!(format!("{back:?}"), format!("{d:?}"), "{text}");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::ProtocolDecoder;

    /// A fresh decoder of one of the G2 protocols, by registry name.
    fn g2_decoder(name: &str) -> Option<Box<dyn ProtocolDecoder>> {
        use super::super::*;
        Some(match name {
            "Marantec" => Box::new(marantec::MarantecDecoder::new()),
            "Marantec24" => Box::new(marantec24::Marantec24Decoder::new()),
            "BETT" => Box::new(bett::BettDecoder::new()),
            "Doitrand" => Box::new(doitrand::DoitrandDecoder::new()),
            "Clemsa" => Box::new(clemsa::ClemsaDecoder::new()),
            "Ansonic" => Box::new(ansonic::AnsonicDecoder::new()),
            "Mastercode" => Box::new(mastercode::MastercodeDecoder::new()),
            "Dickert_MAHS" => Box::new(dickert_mahs::DickertMahsDecoder::new()),
            "Nero Sketch" => Box::new(nero_sketch::NeroSketchDecoder::new()),
            "Nero Radio" => Box::new(nero_radio::NeroRadioDecoder::new()),
            "iDo 117/111" => Box::new(ido::IdoDecoder::new()),
            "Hormann HSM" => Box::new(hormann::HormannDecoder::new()),
            "Elplast" => Box::new(elplast::ElplastDecoder::new()),
            _ => return None,
        })
    }

    /// Cross-protocol guard: every G2 encoder's upload (iDo: a synthesised frame) goes through the
    /// whole registry; its own protocol must be reported, and no other G2 protocol may claim it.
    #[test]
    fn g2_uploads_are_claimed_by_their_own_decoder_only() {
        use crate::protocols::{DecodedSignal, ProtocolRegistry};
        let g2 = [
            "Marantec",
            "Marantec24",
            "BETT",
            "Doitrand",
            "Clemsa",
            "Ansonic",
            "Mastercode",
            "Dickert_MAHS",
            "Nero Sketch",
            "Nero Radio",
            "iDo 117/111",
            "Hormann HSM",
            "Elplast",
        ];
        // (protocol, bits, keys) — unit-test keys where there are some.
        let cases: [(&str, usize, &[u64]); 12] = [
            ("Marantec", 49, &[0x1_3007_10DF_869F, 0x1_307E_DF64_86C5]),
            ("Marantec24", 24, &[0xAC05C4, 0x5A5A5A]),
            ("BETT", 18, &[0xB8B8, 0x2_5A5A]),
            ("Doitrand", 37, &[0x1E_6008_2F5F, 0x0A_5A5A_5A5A]),
            ("Clemsa", 18, &[0x2_FCAA, 0x1_2345]),
            ("Ansonic", 12, &[0x55A, 0xAA3]),
            ("Mastercode", 36, &[0xB_7E00_3C08, 0x5_A5A5_A5A5]),
            ("Dickert_MAHS", 36, &[0x1_5557_5515, 0xF_4F07_5515]),
            ("Nero Sketch", 40, &[0x85_1113_8511, 0x12_3456_789A]),
            (
                "Nero Radio",
                56,
                &[0x0057_2505_0104_9DD3, 0x0036_E4E8_0104_A644],
            ),
            ("Hormann HSM", 44, &[0xFF2_947C_AF13, 0xFF1_2345_6203]),
            ("Elplast", 18, &[0x3_2C82, 0x3_2C0A]),
        ];
        let reg = ProtocolRegistry::new();
        let mut uploads: Vec<(&str, Vec<LevelDuration>)> = Vec::new();
        for (name, bits, keys) in cases {
            let dec = reg.get(name).unwrap();
            for &key in keys {
                let d = DecodedSignal {
                    data: key,
                    data_count_bit: bits,
                    ..decoded(key, bits)
                };
                let mut up = vec![LevelDuration::new(false, 50_000)];
                let mut tx = dec
                    .encode(&d, d.button.unwrap_or(0))
                    .unwrap_or_else(|| panic!("{name}"));
                if name == "Marantec24" {
                    // The C's own 15 200 µs gap sits on its decoder's strict bound: 1 µs of jitter.
                    tx.iter_mut()
                        .filter(|p| p.duration_us == 15_200)
                        .for_each(|p| p.duration_us -= 1);
                }
                up.extend(tx);
                push(&mut up, false, 50_000);
                uploads.push((name, up));
            }
        }
        let mut ido = vec![LevelDuration::new(false, 50_000)];
        for _ in 0..3 {
            push(&mut ido, true, 4500);
            push(&mut ido, false, 4500);
            for i in (0..48).rev() {
                push(&mut ido, true, 450);
                push(
                    &mut ido,
                    false,
                    if (0x574A_0CBB_BF40u64 >> i) & 1 != 0 {
                        450
                    } else {
                        1450
                    },
                );
            }
        }
        push(&mut ido, true, 4500);
        push(&mut ido, false, 4500);
        uploads.push(("iDo 117/111", ido));
        for (name, up) in uploads {
            for f in crate::protocols::test_frequencies(super::super::ANY_FREQUENCY) {
                let mut reg = ProtocolRegistry::new();
                let got: Vec<String> = reg
                    .process_signal_stream(&up, f)
                    .into_iter()
                    .map(|(n, _, _)| n)
                    .collect();
                assert!(got.iter().any(|n| n == name), "{name} @ {f}: {got:?}");
                assert!(
                    got.iter().all(|n| n == name || !g2.contains(&n.as_str())),
                    "{name} @ {f}: {got:?}"
                );
            }
        }
    }

    /// Oracle-parity harness (`#[ignore]`d): `KAT_G2_DUMP=<manifest>` lists `<protocol>|<file.sub>`
    /// lines; each file's RAW_Data is fed to that one decoder the way the C oracle's
    /// `decode --protocol` feeds it (Flipper file worker: values beyond ±1 s clamped, a 0 ends the
    /// stream; receiver reset after each report), printing one line per report:
    /// `<file> | <protocol> | <bits> | <key hex> | sn | btn | cnt | crc | tx`, or `(none)`.
    /// `KAT_G2_ENCODE=<manifest>` lists `<file.sub>` key files: prints the merged upload of
    /// `encode(from_flipper_key(file), captured button)` as signed durations.
    #[test]
    #[ignore]
    fn g2_parity_dump() {
        let opt = |v: Option<u64>| v.map_or("-".to_string(), |v| format!("{v:X}"));
        if let Ok(manifest) = std::env::var("KAT_G2_DUMP") {
            for line in std::fs::read_to_string(manifest)
                .unwrap()
                .lines()
                .filter(|l| !l.is_empty())
            {
                let (name, file) = line.split_once('|').unwrap();
                let mut dec = g2_decoder(name).unwrap_or_else(|| panic!("{name}"));
                let text = std::fs::read_to_string(file).unwrap();
                let mut values = Vec::new();
                for l in text.lines() {
                    let Some(rest) = l.trim().strip_prefix("RAW_Data:") else {
                        continue;
                    };
                    for w in rest
                        .split(|c: char| c.is_whitespace() || c == ',')
                        .filter(|w| !w.is_empty())
                    {
                        values.push(w.parse::<i64>().unwrap());
                    }
                }
                dec.reset();
                let mut n = 0;
                for v in values {
                    if v == 0 {
                        break;
                    }
                    let d = v.unsigned_abs().min(1_000_000) as u32;
                    if let Some(r) = dec.feed(v > 0, d) {
                        n += 1;
                        println!(
                            "KATDUMP {file} | {name} | {} | {:016X} | {} | {} | {} | {} | {}",
                            r.data_count_bit,
                            r.data,
                            opt(r.serial.map(u64::from)),
                            opt(r.button.map(u64::from)),
                            opt(r.counter.map(u64::from)),
                            u8::from(r.crc_valid),
                            u8::from(r.encoder_capable)
                        );
                        dec.reset();
                    }
                }
                if n == 0 {
                    println!("KATDUMP {file} | {name} | (none)");
                }
            }
        }
        if let Ok(manifest) = std::env::var("KAT_G2_ENCODE") {
            let files = std::fs::read_to_string(manifest).unwrap();
            for (n, file) in files.lines().filter(|l| !l.is_empty()).enumerate() {
                let key = FlipperKey::parse(&std::fs::read_to_string(file).unwrap()).unwrap();
                let dec = g2_decoder(&key.protocol).unwrap_or_else(|| panic!("{}", key.protocol));
                let Some(d) = dec.from_flipper_key(&key) else {
                    println!("KATENC {file} | load failed");
                    continue;
                };
                // KAT's own key file for the same decode, for the oracle to load:
                // `$KAT_G2_KEYDIR/<n>_<file name>.kat.sub`.
                if let (Ok(dir), Some(mut k)) =
                    (std::env::var("KAT_G2_KEYDIR"), dec.to_flipper_key(&d))
                {
                    (k.frequency, k.preset) = (key.frequency, key.preset.clone());
                    let base = std::path::Path::new(file)
                        .file_name()
                        .unwrap()
                        .to_string_lossy()
                        .to_string();
                    std::fs::write(format!("{dir}/{n}_{base}.kat.sub"), k.to_text()).unwrap();
                }
                let button = d.button.unwrap_or(0);
                match dec.encode(&d, button) {
                    Some(up) => {
                        let v: Vec<String> = up
                            .iter()
                            .map(|p| {
                                if p.level {
                                    p.duration_us.to_string()
                                } else {
                                    format!("-{}", p.duration_us)
                                }
                            })
                            .collect();
                        println!("KATENC {file} | {}", v.join(" "));
                    }
                    None => println!("KATENC {file} | no encoder"),
                }
            }
        }
    }

    #[test]
    fn reverse_key_matches_the_block_helper() {
        assert_eq!(reverse_key(0b1101, 4), 0b1011);
        assert_eq!(reverse_key(0x1, 48), 1 << 47);
        assert_eq!(reverse_key(u64::MAX, 70), u64::MAX << 6, "bits ≥ 64 read 0");
        let mut r = BitReg::default();
        for b in [true, false, true] {
            r.add(b);
        }
        assert_eq!((r.data, r.count), (0b101, 3));
        r.count = 255;
        r.add(true);
        assert_eq!(r.count, 0, "uint8_t decode_count_bit wraps");
    }

    #[test]
    fn repeated_merges_the_seams() {
        let up = [
            LevelDuration::new(false, 100),
            LevelDuration::new(true, 50),
            LevelDuration::new(false, 7),
        ];
        let out = repeated(&up, 2);
        let got: Vec<(bool, u32)> = out.iter().map(|p| (p.level, p.duration_us)).collect();
        assert_eq!(
            got,
            [
                (false, 100),
                (true, 50),
                (false, 107),
                (true, 50),
                (false, 7)
            ]
        );
    }
}
