//! How many copies of an encoder's upload KAT transmits, and a key file's `Repeat` line.
//!
//! A Flipper encoder builds one upload, and its TX loop (`subghz_protocol_encoder_*_yield`) sends
//! it `encoder.repeat` times. Most encoders also read a key file's optional `Repeat` line into that
//! count (`flipper_format_read_uint32(…, "Repeat", …)`; ProtoPirate's `pp_encoder_read_repeat`).
//! KAT's `encode` returns either every copy the reference sends (the gate and home encoders and most
//! others: `per_call` = the reference's count) or one upload (most car encoders: `per_call` = 1).
//! [transmission] sends [TxCopies::default] copies, or the file's `Repeat` where the reference reads
//! it.
//!
//! **Car encoders** — ProtoPirate's send 10 copies for Fiat V0, Ford V0 / V1 / V2, Honda V2,
//! Kia V1 / V2 / V6, Mazda V0, Subaru and VAG, up to 12 s for one press (a Kia V1 upload lasts
//! 1.2 s), but its emulate screen stops at the button's release once 666 ms have passed, and
//! doctor-firmware's copies of the same encoders send 3. KAT sends 3; a key file's `Repeat: 10` gives
//! ProtoPirate's full count. Every other count is the module's reference's: Chrysler V0 2, Kia V0 /
//! Honda Static / Kia V7 3, Kia V5 6 (ProtoPirate and doctor-firmware), Mazda Siemens 4 and GM 5
//! (Flipper-ARF).
//!
//! **Repeat line** — KAT reads it wherever it is in the file. A Flipper reads a file's lines in
//! order, so it finds `Repeat` only when every line its loader looks for before it is there
//! (Princeton: an optional `Guard_time`); on such files it ignores the line. `Repeat: 0` sends
//! nothing, as on a Flipper.
//!
//! The counts and the `Repeat` handling were measured by running each reference's encoder (host
//! builds of the C, as `encode` with its default count and with `--repeat 1` / `--repeat 5`) on
//! KAT's key files of every encoder-capable decode in the test captures and of generated frames
//! (`docs/transmit.md`, "Copies per press").

use super::{DecodedSignal, ProtocolDecoder};
use crate::radio::demodulator::LevelDuration;

/// How KAT sends one encoder's output.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TxCopies {
    /// Identical copies of the upload in one `encode` call.
    pub per_call: u32,
    /// Copies KAT transmits without a `Repeat` line.
    pub default: u32,
    /// Whether the reference reads a key file's `Repeat` line into its count.
    pub repeat_line: bool,
}

const fn copies(per_call: u32, default: u32, repeat_line: bool) -> TxCopies {
    TxCopies { per_call, default, repeat_line }
}

/// Encoders not listed here: `encode` is the whole transmission and a `Repeat` line changes nothing.
const WHOLE: TxCopies = copies(1, 1, false);

/// The table: decoder names (`ProtocolDecoder::name`) and their copies. Not listed: the reference
/// ignores a `Repeat` line (KeeLoq, PSA, Porsche Cayenne, Telcoma/Cardin EDGE, Doorbell32, MARELLI,
/// Renault V1 (ARF)); its copies differ (Kia V3/V4: ProtoPirate's 16 bursts each carry another CRC
/// nibble, a brute force of the unknown CRC, so `Repeat: n` would be its first n bursts); or KAT
/// has not compared the encoder with one (Beninca ARC, Cham_Code, Honeywell Sec, Jarolift, X10,
/// Mitsubishi V0, Toyota).
const TABLE: &[(&[&str], TxCopies)] = &[
    // Car encoders returning one upload: the reference's count (ProtoPirate's 10 → 3, see above).
    (&["Chrysler V0"], copies(1, 2, true)),
    (&["Kia V0", "Honda Static", "Kia V7"], copies(1, 3, true)),
    (
        &["Fiat V0", "Ford V0", "Ford V1", "Ford V2", "Honda V2", "Kia V1", "Kia V2", "Kia V6", "Mazda V0", "Subaru"],
        copies(1, 3, true),
    ),
    (&["VAG"], copies(1, 3, false)),
    (&["Mazda Siemens"], copies(1, 4, true)),
    (&["GM"], copies(1, 5, true)),
    (&["Kia V5"], copies(1, 6, true)),
    // Encoders whose `encode` holds the reference's copies.
    (
        &["CAME Atomo", "Hormann HSM", "Hormann BiSecur", "Honda V1", "Mercedes V0", "Renault V0", "Renault V1", "Datsun Lada"],
        copies(1, 1, true),
    ),
    (
        &[
            "Ansonic", "BETT", "CAME", "CAME TWEE", "Clemsa", "Dickert_MAHS", "Doitrand", "Dooya", "Elplast", "Feron",
            "GangQi", "GateTX", "Hay21", "Hollarm", "Holtek", "Holtek_HT12X", "Honeywell", "Intertechno_V3",
            "KingGates Stylo4k", "Legrand", "Linear", "LinearDelta3", "Magellan", "Marantec", "Marantec24",
            "Mastercode", "MegaCode", "Nero Radio", "Nero Sketch", "Nice FLO", "Phoenix_V2", "Power Smart",
            "Princeton", "Revers_RB2", "Roger", "SMC5326", "Security+ 1.0", "Security+ 2.0", "Somfy Keytis",
            "Somfy Telis", "Treadmill37", "Prastel", "Nord ICE", "Freezer", "Faac TM", "Faac SLH", "BMW", "BMW CAS3",
            "Hyundai/Kia RIO", "SantaFe 13-16", "Sheriff CFM", "Mazda V2",
        ],
        copies(3, 3, true),
    ),
    (&["Ditec GOL4", "Mercedes V1"], copies(4, 4, true)),
    (&["Allstar Firefly", "KeyFinder", "KeyFinder2", "Tesla Charge"], copies(5, 5, true)),
    (&["Fiat V1", "Fiat V2"], copies(6, 6, true)),
    (&["Scher-Khan"], copies(7, 7, true)),
    (&["Proteco"], copies(10, 10, true)),
    (&["Star Line"], copies(40, 40, true)),
];

/// The copies of `decoder` (a decoder's `ProtocolDecoder::name`).
pub fn tx_copies(decoder: &str) -> TxCopies {
    TABLE.iter().find(|(names, _)| names.contains(&decoder)).map_or(WHOLE, |&(_, c)| c)
}

/// What KAT transmits for `decoded` and `button`: `encode`'s upload sent [TxCopies::default] times,
/// or `repeat` times (a key file's `Repeat` line) where the reference reads that line. None when the
/// encoder has nothing to send, or for `Repeat: 0`, with which a Flipper sends nothing.
pub fn transmission(
    p: &dyn ProtocolDecoder,
    decoded: &DecodedSignal,
    button: u8,
    repeat: Option<u32>,
) -> Option<Vec<LevelDuration>> {
    let out = p.encode(decoded, button)?;
    let c = tx_copies(p.name());
    let n = match repeat {
        Some(n) if c.repeat_line => n,
        _ => c.default,
    };
    if n == 0 {
        return None;
    }
    if n == c.per_call {
        return Some(out);
    }
    // `encode`'s output is `per_call` identical copies for every listed encoder (tested below);
    // anything else is sent as it is.
    let Some((one, seams)) = one_copy(&out, c.per_call as usize) else { return Some(out) };
    // A file's `Repeat: 4000000000` would not fit in memory: past the transmit limit's airtime one more
    // copy is as good as any number (the transmit step refuses it with its message).
    let upload_us: u64 = one.iter().map(|l| u64::from(l.duration_us)).sum();
    let n = u64::from(n).min(crate::radio::tx::MAX_TX_AIR_US / upload_us.max(1) + 1) as usize;
    Some(match seams {
        Seams::Merged => super::f_common::with_repeats(&one, n),
        Seams::Apart => one.repeat(n),
    })
}

/// How an encoder joins its copies: side by side (`Vec::repeat`), or merging the last pulse of one
/// with the first of the next when they share a level (`with_repeats`; one pulse on air either way).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Seams {
    Apart,
    Merged,
}

/// The upload `out` is `k` copies of, and how they are joined. None when `out` is not that.
fn one_copy(out: &[LevelDuration], k: usize) -> Option<(Vec<LevelDuration>, Seams)> {
    if k <= 1 {
        return Some((out.to_vec(), Seams::Apart));
    }
    let same = |a: &[LevelDuration], b: &[LevelDuration]| {
        a.len() == b.len() && a.iter().zip(b).all(|(x, y)| x.level == y.level && x.duration_us == y.duration_us)
    };
    if !out.is_empty() && out.len().is_multiple_of(k) {
        let len = out.len() / k;
        if out.chunks(len).all(|c| same(c, &out[..len])) {
            return Some((out[..len].to_vec(), Seams::Apart));
        }
    }
    // merged: k copies of `len` pulses minus one at each of the k − 1 seams
    let total = out.len() + k - 1;
    if total.is_multiple_of(k) && total / k >= 2 {
        let len = total / k;
        let one: Vec<LevelDuration> = out[..len - 1].iter().chain(out.last()).copied().collect();
        if same(&super::f_common::with_repeats(&one, k), out) {
            return Some((one, Seams::Merged));
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::ProtocolRegistry;

    /// Every name in the table is an encoder of the registry, listed once (a renamed protocol would
    /// silently fall back to "the whole transmission").
    #[test]
    fn the_table_names_registered_encoders() {
        let reg = ProtocolRegistry::new();
        let names: Vec<&str> = TABLE.iter().flat_map(|(n, _)| n.iter().copied()).collect();
        for n in &names {
            assert!(reg.get(n).is_some_and(|p| p.supports_encoding() && p.name() == *n), "{n}");
            assert_eq!(names.iter().filter(|m| *m == n).count(), 1, "{n} listed twice");
        }
    }

    /// `encode`'s output is `per_call` identical copies for every listed encoder that holds more than
    /// one, so a `Repeat` line can set the count: checked on random code words (what each encoder sends
    /// for them), on the firmwares' unit-test captures and key files where present, on the IMPORTS
    /// Fiat and Chrysler captures, and on made-up key files.
    #[test]
    fn listed_encoders_hold_identical_copies() {
        let reg = ProtocolRegistry::new();
        let mut seen: std::collections::HashMap<&str, usize> = Default::default();
        let mut check = |name: &'static str, out: Vec<LevelDuration>| {
            let k = tx_copies(name).per_call as usize;
            assert!(one_copy(&out, k).is_some(), "{name}: {} pulses are not {k} identical copies", out.len());
            *seen.entry(name).or_default() += 1;
        };
        let mut seed = 0x2545_F491_4F6C_DD1Du64;
        let mut next = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            seed
        };
        let multi: Vec<&'static str> =
            TABLE.iter().filter(|(_, c)| c.per_call > 1).flat_map(|(n, _)| n.iter().copied()).collect();
        for &name in &multi {
            let p = reg.get(name).unwrap();
            // the frame's own bit count (encoders check it); the code word holds at most 64 of them
            let bits = p.timing().min_count_bit.max(1);
            for _ in 0..200 {
                let mask = if bits >= 64 { u64::MAX } else { (1u64 << bits) - 1 };
                let mut d = DecodedSignal::new(next() & mask, bits);
                (d.serial, d.button, d.counter) = (Some(next() as u32 & 0x0FFF_FFFF), Some((next() & 0xF) as u8), Some(next() as u16));
                // encoder state some protocols keep in `extra` (a TE, a guard time, a second word)
                d.extra = match next() % 3 {
                    0 => None,
                    1 => Some(next() & 0xFFFF),
                    _ => Some(next() & 0x00FF_0000_03FF),
                };
                d.encoder_capable = true;
                if let Some(out) = p.encode(&d, d.button.unwrap_or(0)) {
                    check(name, out);
                }
            }
        }
        // The captures and key files of the firmwares' unit tests, and the IMPORTS captures with Fiat
        // V1 / V2 frames, where present.
        let roots = [
            "IMPORTS/FIAT",
            "IMPORTS/chrysler",
            "REFERENCES/Flipper-ARF/applications/debug/unit_tests/resources/unit_tests/subghz",
            "REFERENCES/Momentum-Firmware/applications/debug/unit_tests/resources/unit_tests/subghz",
            "REFERENCES/RogueMaster/applications/debug/unit_tests/resources/unit_tests/subghz",
        ];
        for root in roots.iter().map(std::path::Path::new).filter(|r| r.exists()) {
            let mut files = Vec::new();
            let mut dirs = vec![root.to_path_buf()];
            while let Some(dir) = dirs.pop() {
                for e in std::fs::read_dir(dir).unwrap().flatten() {
                    let path = e.path();
                    if path.is_dir() {
                        dirs.push(path);
                    } else if path.extension().is_some_and(|x| x == "sub") {
                        files.push(path);
                    }
                }
            }
            for path in files {
                let text = std::fs::read_to_string(&path).unwrap_or_default();
                if let Ok(key) = crate::protocols::FlipperKey::parse(&text) {
                    let r = ProtocolRegistry::new();
                    if let Some((n, d)) = r.decode_key(&key) {
                        let p = r.get(&n).unwrap();
                        if let Some(&name) = multi.iter().find(|&&m| m == p.name()) {
                            if let Some(out) = p.encode(&d, d.button.unwrap_or(0)) {
                                check(name, out);
                            }
                        }
                    }
                    continue;
                }
                let Ok(sub) = crate::export::flipper::import_sub(&path) else { continue };
                let pairs: Vec<LevelDuration> = sub.pairs.iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect();
                let mut r = ProtocolRegistry::new();
                for (n, d, _) in r.process_signal_stream(&pairs, sub.frequency) {
                    let Some(p) = r.get(&n) else { continue };
                    let Some(&name) = multi.iter().find(|&&m| m == p.name()) else { continue };
                    if d.encoder_capable {
                        if let Some(out) = p.encode(&d, d.button.unwrap_or(0)) {
                            check(name, out);
                        }
                    }
                }
            }
        }
        // Made-up key files (KAT's vector generators) of encoders whose input must decrypt or pass a
        // check, or whose key files hold another bit count than a decoded packet (Security+ 1.0: 42
        // bits, two 21-symbol packets), so random code words never encode; and KingGates Stylo4k
        // frames from its fixture generator (the hop must decrypt). With these every listed encoder
        // is checked without REFERENCES, as on CI.
        let mut keys: Vec<crate::protocols::FlipperKey> = [
            "Protocol: BMW\nBit: 128\nKey: 59 9E 12 34 56 78 11 89\nSerial: 305419896\nBtn: 2\nCnt: 107971\n\
             ValidationField: 137\nTE: 62\nKey_2: D2 E1 DA 5A 12 34 AB CD\nKey_3: 00 00 5A 5A 12 34 AB CD\n",
            "Protocol: Mercedes V1\nBit: 122\nKey: FB 29 1A 2B 3C 4D A6 F5\nSerial: 439041101\nBtn: 1\nCnt: 727\nTE: 50\n\
             Extra: 1\nKey_2: B5 AF 4E 0A 79 D3 68 C0\nKey_3: 00 2B 5E 9C 14 F3 A6 D1\n",
            "Protocol: Sheriff CFM\nBit: 64\nKey: 5A 3C 10 C5 4D 23 BD A9\nModel: 0\nSerial: 1513885893\nBtn: 32\nCnt: 257\n",
            "Protocol: SantaFe 13-16\nBit: 80\nKey: 03 54 50 C9 37 6C 54 F8\nSerial: 3632212\nBtn: 1\nCnt: 248\nRolling: 55857353\n",
            "Protocol: Security+ 1.0\nBit: 42\nKey: 0A 1B 2C 3D E6 01 23 45\n",
        ]
        .iter()
        .map(|text| crate::protocols::FlipperKey::parse(text).unwrap())
        .collect();
        keys.extend([0x1234_5678_0000_0013, 0x0BAD_C0DE_7F01_0025].map(super::super::kinggates_stylo_4k::fixture_key));
        for key in &keys {
            let (n, d) = reg.decode_key(key).unwrap_or_else(|| panic!("{}", key.to_text()));
            let p = reg.get(&n).unwrap();
            let name = multi.iter().copied().find(|&m| m == p.name()).unwrap();
            check(name, p.encode(&d, d.button.unwrap_or(0)).unwrap());
        }
        let untested: Vec<&&str> = multi.iter().filter(|n| !seen.contains_key(*n)).collect();
        assert!(untested.is_empty(), "no transmission checked for {untested:?}");
    }

    /// One upload sent n times; copies already in `encode` split back into one; `Repeat: 0` and an
    /// output that is not `per_call` copies.
    #[test]
    fn copies_of_one_upload() {
        let up = |d: u32| LevelDuration::new(true, d);
        let one = vec![up(500), LevelDuration::new(false, 900)];
        let three = super::super::f_common::with_repeats(&one, 3);
        assert_eq!(one_copy(&three, 3).map(|(o, seams)| (o.len(), seams)), Some((2, Seams::Apart)));
        assert!(one_copy(&three, 2).is_none());
        assert!(one_copy(&three[..5], 3).is_none());
        let mut odd = three.clone();
        odd[4].duration_us = 501;
        assert!(one_copy(&odd, 3).is_none());
        // copies merged at the seams (an upload starting and ending HIGH) split back too
        let seam = super::super::f_common::with_repeats(&[up(500), LevelDuration::new(false, 900), up(300)], 3);
        assert_eq!(seam.len(), 7);
        let (back, seams) = one_copy(&seam, 3).unwrap();
        assert_eq!((back.iter().map(|l| l.duration_us).collect::<Vec<_>>(), seams), (vec![500, 900, 300], Seams::Merged));
        // and copies left apart although they share a level (`Vec::repeat`, as the RIO encoder)
        let apart = [up(500), LevelDuration::new(false, 900), up(300)].repeat(3);
        assert_eq!(one_copy(&apart, 3).map(|(o, seams)| (o.len(), seams)), Some((3, Seams::Apart)));
    }

    /// Through the real encoders: a car encoder's one upload goes out as the reference's copies (Kia
    /// V1: 3; GM: 5), a `Repeat` line sets the count where the reference reads it (VAG's does not),
    /// a gate encoder's 3 copies stay as they are and become `Repeat`'s count, and `Repeat: 0` sends
    /// nothing.
    #[test]
    fn transmissions_follow_the_references_counts() {
        let reg = ProtocolRegistry::new();
        let len = |name: &str, d: &DecodedSignal, repeat: Option<u32>| {
            let p = reg.get(name).unwrap();
            transmission(p, d, d.button.unwrap_or(0), repeat).map(|t| t.iter().map(|l| u64::from(l.duration_us)).sum::<u64>())
        };
        let one = |name: &str, d: &DecodedSignal| {
            reg.get(name).unwrap().encode(d, d.button.unwrap_or(0)).unwrap().iter().map(|l| u64::from(l.duration_us)).sum::<u64>()
        };
        // Kia V1, made-up fields
        let mut kia = DecodedSignal::new(0, 57);
        (kia.serial, kia.button, kia.counter, kia.encoder_capable) = (Some(0x1234_5678), Some(1), Some(0x123), true);
        let up = one("Kia V1", &kia);
        assert_eq!(len("Kia V1", &kia, None), Some(3 * up));
        assert_eq!(len("Kia V1", &kia, Some(10)), Some(10 * up));
        assert_eq!(len("Kia V1", &kia, Some(1)), Some(up));
        assert_eq!(len("Kia V1", &kia, Some(0)), None);
        // an absurd count stops just past the transmit limit, which then refuses it
        let huge = len("Kia V1", &kia, Some(u32::MAX)).unwrap();
        assert!(huge > crate::radio::tx::MAX_TX_AIR_US && huge <= crate::radio::tx::MAX_TX_AIR_US + up, "{huge}");
        // Princeton: `encode` holds 3 copies; `Repeat: 5` makes it 5
        let mut pr = DecodedSignal::new(0x95_D5D4, 24);
        pr.encoder_capable = true;
        pr.extra = Some(0x1E_0000_0190);
        let three = one("Princeton", &pr);
        assert_eq!(len("Princeton", &pr, None), Some(three));
        assert_eq!(len("Princeton", &pr, Some(5)), Some(three / 3 * 5));
        // Proteco: 10 copies (RogueMaster's count), `Repeat: 1` and `Repeat: 3` as its encoder reads them
        let mut pt = DecodedSignal::new(0xDF_2396, 24);
        pt.encoder_capable = true;
        let ten = one("Proteco", &pt);
        assert_eq!(len("Proteco", &pt, None), Some(ten));
        assert_eq!(len("Proteco", &pt, Some(1)), Some(ten / 10));
        assert_eq!(len("Proteco", &pt, Some(3)), Some(ten / 10 * 3));
    }
}
