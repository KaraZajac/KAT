//! Helpers shared by the G5 gate/garage protocols (Faac SLH, KingGates Stylo4k, Jarolift, Beninca
//! ARC): keystore lookups the way their C walks the Flipper keystore, the key-file "Data" line and
//! upload building with adjacent same-level durations merged.

use super::flipper_key::FlipperKey;
use super::keeloq_common::{keeloq_keystore, MfKey};
use crate::radio::demodulator::LevelDuration;

/// First keystore entry (the C's `M_EACH` order over the keystore) that satisfies `pred`.
pub(super) fn keystore_entry(pred: impl Fn(&MfKey) -> bool) -> Option<&'static MfKey> {
    keeloq_keystore().iter().find(|e| pred(e))
}

/// Append one level/duration to an upload, merging it into the previous element when the level
/// is the same (the radio sees one continuous level either way).
pub(super) fn push(out: &mut Vec<LevelDuration>, level: bool, duration_us: u32) {
    match out.last_mut() {
        Some(last) if last.level == level => last.duration_us += duration_us,
        _ => out.push(LevelDuration::new(level, duration_us)),
    }
}

/// `flipper_format_read_hex(.., name, data, n)`: the first `n` space-separated values of the
/// line, each read from its first two hex digits; fewer than `n` values is an error.
pub(super) fn read_hex(key: &FlipperKey, name: &str, n: usize) -> Option<Vec<u8>> {
    let tokens: Vec<&str> = key.field(name)?.split_whitespace().collect();
    if tokens.len() < n {
        return None;
    }
    tokens[..n]
        .iter()
        .map(|t| {
            let d = t.as_bytes().get(..2)?;
            if !d.iter().all(u8::is_ascii_hexdigit) {
                return None;
            }
            u8::from_str_radix(std::str::from_utf8(d).ok()?, 16).ok()
        })
        .collect()
}

/// A key-file hex line of 8 bytes (`flipper_format_read_hex(.., 8)`), as a big-endian u64.
pub(super) fn hex8_field(key: &FlipperKey, name: &str) -> Option<u64> {
    Some(
        read_hex(key, name, 8)?
            .iter()
            .fold(0u64, |acc, &b| acc << 8 | u64::from(b)),
    )
}

/// The key file's `Key` as the C reads it (`flipper_format_read_hex(.., "Key", .., 8)`): exactly
/// 8 bytes.
pub(super) fn key8(key: &FlipperKey) -> Option<u64> {
    (key.key_bytes.len() == 8).then_some(key.key)
}

/// The C's counter step in standard counter mode (rolling-counter multiplier 1, no counter
/// override): `cnt + 1`, or 0 when that exceeds `max`.
pub(super) fn next_counter(cnt: u32, max: u32) -> u32 {
    if u64::from(cnt) + 1 > u64::from(max) {
        0
    } else {
        cnt + 1
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn push_merges_same_level() {
        let mut v = Vec::new();
        push(&mut v, false, 100);
        push(&mut v, false, 50);
        push(&mut v, true, 10);
        let got: Vec<(bool, u32)> = v.iter().map(|l| (l.level, l.duration_us)).collect();
        assert_eq!(got, [(false, 150), (true, 10)]);
    }

    #[test]
    fn hex8_requires_eight_bytes() {
        let k = FlipperKey::parse(
            "Protocol: Jarolift\nBit: 72\nKey: 01 02 03 04 05 06 07 08\n\
             Data: 00 00 00 00 00 00 00 80\nShort: 00 80\nLong: 00 00 00 00 00 00 00 81 FF\n\
             Run: 0000000000000082\nSeed: 1A2B 3C 4D 5E\n",
        )
        .unwrap();
        assert_eq!(hex8_field(&k, "Data"), Some(0x80));
        assert_eq!(hex8_field(&k, "Short"), None);
        // Like the Flipper: extra values are ignored, one run of digits is one value.
        assert_eq!(hex8_field(&k, "Long"), Some(0x81));
        assert_eq!(hex8_field(&k, "Run"), None);
        assert_eq!(read_hex(&k, "Seed", 4), Some(vec![0x1A, 0x3C, 0x4D, 0x5E]));
        assert_eq!(key8(&k), Some(0x0102_0304_0506_0708));
        assert_eq!(next_counter(0xFFFF, 0xFFFF), 0);
        assert_eq!(next_counter(0xFFFE, 0xFFFF), 0xFFFF);
        assert_eq!(next_counter(u32::MAX, u32::MAX), 0);
    }
}

/// C-oracle driver for the G5 protocols (ignored; compares KAT with `subghz_oracle_*`).
/// `KAT_G5_PROTO=<protocol name>`, `KAT_G5_FILES=<file listing one path per line>`,
/// `KAT_G5_OUT=<output file>`, `KAT_G5_MODE`:
/// - `dec`: feed each RAW file to that one decoder (reset after each decode, like the oracle's
///   `decode --protocol`), one line per decode: `<file> | <fields>`;
/// - `key`: `from_flipper_key` of each key file: `<file> | <fields> | <to_flipper_key lines>`;
/// - `enc`: `encode(from_flipper_key, captured button)` of each key file: `<file> | <pulses>`,
///   signed µs, adjacent same-level durations merged.
#[cfg(test)]
mod oracle_driver {
    use super::super::common::DecodedSignal;
    use super::super::ProtocolDecoder;
    use crate::protocols::FlipperKey;
    use std::fmt::Write as _;

    fn decoder(name: &str) -> Box<dyn ProtocolDecoder> {
        match name {
            "Faac SLH" => Box::new(super::super::faac_slh::FaacSlhDecoder::new()),
            "KingGates Stylo4k" => {
                Box::new(super::super::kinggates_stylo_4k::KingGatesStylo4kDecoder::new())
            }
            "Jarolift" => Box::new(super::super::jarolift::JaroliftDecoder::new()),
            "Beninca ARC" => Box::new(super::super::beninca_arc::BenincaArcDecoder::new()),
            other => panic!("not a G5 protocol: {other}"),
        }
    }

    fn fields(d: &DecodedSignal) -> String {
        let hex = |v: Option<u64>| v.map_or("-".to_string(), |v| format!("{v:X}"));
        format!(
            "data={:016X} extra={} bits={} sn={} btn={} cnt={} crc={} tx={}",
            d.data,
            hex(d.extra),
            d.data_count_bit,
            hex(d.serial.map(u64::from)),
            hex(d.button.map(u64::from)),
            hex(d.counter.map(u64::from)),
            u8::from(d.crc_valid),
            u8::from(d.encoder_capable)
        )
    }

    #[test]
    #[ignore]
    fn g5_oracle_driver() {
        let (Ok(proto), Ok(list), Ok(out)) = (
            std::env::var("KAT_G5_PROTO"),
            std::env::var("KAT_G5_FILES"),
            std::env::var("KAT_G5_OUT"),
        ) else {
            eprintln!("KAT_G5_PROTO / KAT_G5_FILES / KAT_G5_OUT not set");
            return;
        };
        let mode = std::env::var("KAT_G5_MODE").unwrap_or_else(|_| "dec".to_string());
        if mode == "gen" {
            // KAT_G5_FILES = output directory, KAT_G5_OUT = count: write random valid key files.
            let n: u64 = out.parse().unwrap();
            let mut r: u64 = 0x9E37_79B9_7F4A_7C15;
            for i in 0..n {
                r ^= r << 13;
                r ^= r >> 7;
                r ^= r << 17;
                let key = match proto.as_str() {
                    "Faac SLH" => super::super::faac_slh::fixture_key(r),
                    "KingGates Stylo4k" => super::super::kinggates_stylo_4k::fixture_key(r),
                    "Jarolift" => super::super::jarolift::fixture_key(r),
                    _ => super::super::beninca_arc::fixture_key(r),
                };
                let key = FlipperKey {
                    frequency: 433_920_000,
                    preset: Some("FuriHalSubGhzPresetOok650Async".into()),
                    ..key
                };
                std::fs::write(format!("{list}/k{i:04}.sub"), key.to_text()).unwrap();
            }
            return;
        }
        let mut text = String::new();
        for path in std::fs::read_to_string(list)
            .unwrap()
            .lines()
            .filter(|l| !l.is_empty())
        {
            let mut dec = decoder(&proto);
            match mode.as_str() {
                "dec" => {
                    let Ok((_, pairs)) =
                        crate::export::flipper::import_sub_raw(std::path::Path::new(path))
                    else {
                        writeln!(text, "{path} | !ERROR").unwrap();
                        continue;
                    };
                    let mut n = 0;
                    for p in &pairs {
                        if let Some(d) = dec.feed(p.level, p.duration_us) {
                            dec.reset();
                            writeln!(text, "{path} | {}", fields(&d)).unwrap();
                            n += 1;
                        }
                    }
                    if n == 0 {
                        writeln!(text, "{path} | (none)").unwrap();
                    }
                }
                "key" | "enc" => {
                    let key = FlipperKey::parse(&std::fs::read_to_string(path).unwrap()).unwrap();
                    let Some(d) = dec.from_flipper_key(&key) else {
                        writeln!(text, "{path} | (rejected)").unwrap();
                        continue;
                    };
                    if mode == "key" {
                        let back = dec.to_flipper_key(&d).map_or(String::new(), |k| {
                            k.to_text().lines().skip(4).collect::<Vec<_>>().join("; ")
                        });
                        writeln!(text, "{path} | {} | {back}", fields(&d)).unwrap();
                    } else {
                        match dec.encode(&d, d.button.unwrap_or(0)) {
                            Some(wire) => {
                                let v: Vec<String> = wire
                                    .iter()
                                    .map(|l| {
                                        if l.level {
                                            format!("{}", l.duration_us)
                                        } else {
                                            format!("-{}", l.duration_us)
                                        }
                                    })
                                    .collect();
                                writeln!(text, "{path} | {}", v.join(" ")).unwrap();
                            }
                            None => writeln!(text, "{path} | (no upload)").unwrap(),
                        }
                    }
                }
                other => panic!("KAT_G5_MODE {other}"),
            }
        }
        std::fs::write(out, text).unwrap();
    }
}
