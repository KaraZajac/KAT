//! Test helpers shared by the rtl_433 ports: bit and wire builders, decoder runners, the
//! rtl_433_tests recordings through KAT's IQ pipeline, rtl_433 pulse files, and the fuzz trains
//! for the C harness.

use crate::protocols::{DecodedSignal, ProtocolDecoder, ProtocolRegistry};
use crate::radio::demodulator::LevelDuration;
use std::path::{Path, PathBuf};

/// Bits of `value`, MSB first, `n` long.
pub fn bits_of(value: u64, n: usize) -> Vec<bool> {
    (0..n).rev().map(|i| value >> i & 1 == 1).collect()
}

/// Bits of a byte string, MSB first, `n` long.
pub fn bits_of_bytes(bytes: &[u8], n: usize) -> Vec<bool> {
    (0..n).map(|i| bytes[i / 8] >> (7 - i % 8) & 1 == 1).collect()
}

/// `(HIGH, LOW)` µs pairs as a level/duration stream (a 0 µs LOW is left out).
pub fn wire(pairs: &[(u32, u32)]) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(2 * pairs.len());
    for &(p, g) in pairs {
        out.push(LevelDuration::new(true, p));
        if g > 0 {
            out.push(LevelDuration::new(false, g));
        }
    }
    out
}

/// Manchester for rtl_433's zero-bit slicer (1 = HIGH→LOW at mid-bit, 0 = LOW→HIGH): the
/// (pulse, gap) pairs that slice to `row` at half-bit `half` µs, the last gap extended by `tail`
/// µs (a row ending HIGH has a 0 µs last gap before the tail). `row[0]` must be 0: it is the
/// slicer's hard-coded leading 0, whose rising edge starts the first pulse.
pub fn manchester_zerobit(row: &[bool], half: u32, tail: u32) -> Vec<(u32, u32)> {
    assert!(!row.is_empty() && !row[0], "a zero-bit row starts with the hard-coded 0");
    // Half-bit levels; the leading 0 bit's LOW→HIGH edge starts the first pulse.
    let mut levels = vec![true];
    for &b in &row[1..] {
        levels.extend([b, !b]);
    }
    let mut runs: Vec<(bool, u32)> = Vec::new();
    for l in levels {
        match runs.last_mut() {
            Some((lv, d)) if *lv == l => *d += half,
            _ => runs.push((l, half)),
        }
    }
    let mut pairs: Vec<(u32, u32)> = runs.chunks(2).map(|c| (c[0].1, c.get(1).map_or(0, |r| r.1))).collect();
    if let Some(last) = pairs.last_mut() {
        last.1 += tail;
    }
    pairs
}

/// NRZ (PCM) levels of `bits` at `bit_us` per bit, same-level bits merged.
pub fn nrz(bits: &[bool], bit_us: u32) -> Vec<LevelDuration> {
    let mut out: Vec<LevelDuration> = Vec::new();
    for &b in bits {
        match out.last_mut() {
            Some(l) if l.level == b => l.duration_us += bit_us,
            _ => out.push(LevelDuration::new(b, bit_us)),
        }
    }
    out
}

/// `pulses` between two idle LOWs of `idle_us` (merged into a LOW they touch).
pub fn with_idle(pulses: &[LevelDuration], idle_us: u32) -> Vec<LevelDuration> {
    let mut out = vec![LevelDuration::new(false, idle_us)];
    for p in pulses {
        match out.last_mut() {
            Some(last) if last.level == p.level => last.duration_us += p.duration_us,
            _ => out.push(*p),
        }
    }
    match out.last_mut() {
        Some(last) if !last.level => last.duration_us += idle_us,
        _ => out.push(LevelDuration::new(false, idle_us)),
    }
    out
}

/// Feed `(HIGH, LOW)` µs pairs to a decoder from a reset (no leading LOW), collecting its reports
/// (no reset after them).
pub fn feed_pairs(dec: &mut dyn ProtocolDecoder, pairs: &[(u32, u32)]) -> Vec<DecodedSignal> {
    dec.reset();
    let mut out = Vec::new();
    for &(p, g) in pairs {
        out.extend(dec.feed(true, p));
        out.extend(dec.feed(false, g));
    }
    out
}

/// Every frame `dec` reports for `wire`, from a reset and reset after each report (as the registry
/// does).
pub fn decode_all(dec: &mut dyn ProtocolDecoder, wire: &[LevelDuration]) -> Vec<DecodedSignal> {
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

/// Oracle harness for the ports FoxFW also decodes (its key files): for every RAW `.sub` listed in
/// the file `$env` (one path a line), the key-file lines `dec.to_flipper_key` writes for each frame
/// `dec` reports, fed the file's values from a reset and reset after each report like
/// `subghz_oracle_foxfwenc decode --protocol`, with the file's Frequency / Preset:
/// `KATKEY <file> | <lines after Version, joined by "; ">` (the oracle's serialized-fields layout),
/// `KATKEY <file> | (none)` when nothing decodes. Nothing when `$env` is unset.
pub fn print_raw_key_lines(env: &str, dec: &mut dyn ProtocolDecoder) {
    let Ok(list) = std::env::var(env) else { return };
    for file in std::fs::read_to_string(list).unwrap().lines().filter(|l| !l.is_empty()) {
        let sub = match crate::export::flipper::read_sub(Path::new(file)) {
            Ok(crate::export::flipper::SubContent::Raw(sub)) => sub,
            _ => panic!("{file}: not a RAW .sub"),
        };
        let wire: Vec<LevelDuration> = sub.pairs.iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect();
        let decodes = decode_all(dec, &wire);
        if decodes.is_empty() {
            println!("KATKEY {file} | (none)");
        }
        for d in decodes {
            let text = dec.to_flipper_key(&d).map_or("(no key file)".to_string(), |mut key| {
                key.frequency = sub.frequency;
                if let Some(p) = &sub.preset {
                    key.set_preset(p);
                }
                key.to_text().lines().filter(|l| !l.starts_with("Filetype:") && !l.starts_with("Version:")).collect::<Vec<_>>().join("; ")
            });
            println!("KATKEY {file} | {text}");
        }
    }
}

/// Oracle harness for the encoder: for every key file listed in the file `$env`, KAT's
/// transmission of it (`from_flipper_key`, then `encode` with the key's own button, as Replay /
/// "Send next code" and the round-trip tests do): `KATENC <file> | <signed µs …>`, or
/// `KATENC <file> | (none)` when the key does not load or nothing is sent — to compare with
/// `subghz_oracle_foxfwenc encode <file>`. Nothing when `$env` is unset.
pub fn print_key_file_encodes(env: &str, dec: &dyn ProtocolDecoder) {
    let Ok(list) = std::env::var(env) else { return };
    for file in std::fs::read_to_string(list).unwrap().lines().filter(|l| !l.is_empty()) {
        let key = crate::protocols::FlipperKey::parse(&std::fs::read_to_string(file).unwrap()).ok();
        let tx = key.and_then(|k| dec.from_flipper_key(&k)).and_then(|d| dec.encode(&d, d.button.unwrap_or(0)));
        let text = tx.map_or("(none)".to_string(), |tx| {
            tx.iter().map(|p| if p.level { p.duration_us.to_string() } else { format!("-{}", p.duration_us) }).collect::<Vec<_>>().join(" ")
        });
        println!("KATENC {file} | {text}");
    }
}

/// The registry's decodes (name, decode) of `wire` at `freq`, received as `rf`.
pub fn registry(wire: &[LevelDuration], freq: u32, rf: Option<crate::capture::RfModulation>) -> Vec<(String, DecodedSignal)> {
    ProtocolRegistry::new().process_signal_stream_rf(wire, freq, rf).into_iter().map(|(n, d, _)| (n, d)).collect()
}

/// `REFERENCES/rtl_433_tests/tests` (or `$KAT_RTL433_TESTS`), None when absent (CI).
pub fn rtl433_tests() -> Option<PathBuf> {
    let root = PathBuf::from(std::env::var("KAT_RTL433_TESTS").unwrap_or_else(|_| "REFERENCES/rtl_433_tests/tests".into()));
    root.is_dir().then_some(root)
}

/// One receive-pipeline burst: (AM pairs, FM pairs, frequency).
pub type Burst = (Option<Vec<LevelDuration>>, Option<Vec<LevelDuration>>, u32);

/// Every burst of IQ samples through KAT's receive pipeline.
pub fn iq_bursts(iq: &[u8], format: crate::radio::dsp::IqFormat, rate: u32, offset_hz: f64, freq: u32) -> Vec<Burst> {
    let mut p = crate::radio::pipeline::RxPipeline::new(rate, offset_hz, freq);
    let mut bursts = p.process(iq, format);
    bursts.extend(p.flush());
    bursts.into_iter().map(|b| (b.am, b.fm, b.frequency)).collect()
}

/// Every burst of an IQ recording through KAT's receive pipeline (rate and frequency from the file
/// name, rtl_433's 250 kS/s when it names no rate).
pub fn recording_bursts(path: &Path) -> Vec<Burst> {
    let name = path.file_name().unwrap().to_string_lossy().to_string();
    let rate = crate::radio::iq_file::parse_name_hints(&name).rate.is_none().then_some(250_000);
    let spec = crate::radio::iq_file::IqFileSpec::resolve(path.to_path_buf(), None, rate, 433_920_000).unwrap();
    let offset = spec.target_hz as f64 - spec.centre_hz as f64;
    iq_bursts(&std::fs::read(path).unwrap(), spec.format, spec.sample_rate, offset, spec.target_hz)
}

/// The registry's decodes (name, decode) of bursts, as `kat --decode` makes them: each burst's AM
/// and FM pulses on their receive path.
pub fn burst_decodes(bursts: Vec<Burst>) -> Vec<(String, DecodedSignal)> {
    crate::protocols::keys::load_keystore_from_embedded();
    let mut reg = ProtocolRegistry::new();
    let mut out = Vec::new();
    for (am, fm, freq) in bursts {
        for (pairs, rf) in [(am, crate::capture::RfModulation::AM), (fm, crate::capture::RfModulation::FM)] {
            if let Some(pairs) = pairs {
                out.extend(reg.process_signal_stream_path(&pairs, freq, Some(rf)).into_iter().map(|(n, d, _)| (n, d)));
            }
        }
    }
    out
}

/// The registry's decodes of a recording ([burst_decodes] of its [recording_bursts]).
pub fn registry_decodes(path: &Path) -> Vec<(String, DecodedSignal)> {
    burst_decodes(recording_bursts(path))
}

/// The reports of one decoder alone over a recording's bursts (`am`: the AM or the FM pairs).
pub fn decoder_decodes(dec: &mut dyn ProtocolDecoder, path: &Path, am: bool) -> Vec<DecodedSignal> {
    let mut out = Vec::new();
    for (a, f, _) in recording_bursts(path) {
        if let Some(pairs) = if am { a } else { f } {
            dec.reset();
            out.extend(pairs.iter().filter_map(|p| dec.feed(p.level, p.duration_us)));
        }
    }
    out
}

/// rtl_433's expected events of a recording (its `.json`, `time` dropped), in order.
pub fn expected_json(path: &Path) -> Vec<serde_json::Value> {
    let text = std::fs::read_to_string(path.with_extension("json")).unwrap_or_default();
    text.lines()
        .filter(|l| !l.trim().is_empty())
        .map(|l| {
            let mut v: serde_json::Value = serde_json::from_str(l).unwrap();
            v.as_object_mut().unwrap().remove("time");
            v
        })
        .collect()
}

/// Distinct values, in order of first appearance.
pub fn distinct(v: &[serde_json::Value]) -> Vec<serde_json::Value> {
    let mut out: Vec<serde_json::Value> = Vec::new();
    for x in v {
        if !out.contains(x) {
            out.push(x.clone());
        }
    }
    out
}

/// The rtl_433_tests recordings under `dirs` (each `.cu8` with rtl_433's expected output in the
/// `.json` next to it): the distinct frames KAT reports as `name` (rendered by `fields`) must be
/// exactly the distinct `model` lines of the JSON (rendered by `json`); a recording without a JSON
/// expects none. Returns how many recordings carry `model` lines, None (skip) when no folder exists.
pub fn check_recordings(dirs: &[&str], name: &str, model: &str, fields: fn(&DecodedSignal) -> String, json: fn(&serde_json::Value) -> String) -> Option<usize> {
    use std::collections::BTreeSet;
    let Some(root) = rtl433_tests() else {
        eprintln!("Skip: rtl_433_tests not found");
        return None;
    };
    let mut stack: Vec<_> = dirs.iter().map(|d| root.join(d)).filter(|d| d.is_dir()).collect();
    if stack.is_empty() {
        eprintln!("Skip: {dirs:?} not found under {}", root.display());
        return None;
    }
    let mut files = Vec::new();
    while let Some(d) = stack.pop() {
        for e in std::fs::read_dir(&d).into_iter().flatten().flatten() {
            let p = e.path();
            if p.is_dir() {
                stack.push(p);
            } else if p.extension().is_some_and(|x| x == "cu8") {
                files.push(p);
            }
        }
    }
    files.sort();
    let (mut with_lines, mut bad) = (0, Vec::new());
    for f in &files {
        let want: BTreeSet<String> = expected_json(f).iter().filter(|v| v["model"] == model).map(json).collect();
        let got: BTreeSet<String> = registry_decodes(f).iter().filter(|(n, _)| n == name).map(|(_, d)| fields(d)).collect();
        with_lines += usize::from(!want.is_empty());
        if got != want {
            bad.push(format!("{}: KAT {got:?}, rtl_433 {want:?}", f.display()));
        }
    }
    assert!(bad.is_empty(), "{name} vs rtl_433 {model}:\n{}", bad.join("\n"));
    Some(with_lines)
}

/// The packages of an rtl_433 pulse file (`-w x.ook`): (pulse, gap) µs pairs (at `-s 1M`); a `;`
/// line or an empty line ends a package.
pub fn read_ook(text: &str) -> Vec<Vec<(u32, u32)>> {
    let mut out: Vec<Vec<(u32, u32)>> = Vec::new();
    let mut cur: Vec<(u32, u32)> = Vec::new();
    for line in text.lines() {
        let line = line.trim();
        if line.starts_with(';') || line.is_empty() {
            if !cur.is_empty() {
                out.push(std::mem::take(&mut cur));
            }
            continue;
        }
        let mut it = line.split_whitespace().filter_map(|v| v.parse::<u32>().ok());
        if let (Some(p), Some(g)) = (it.next(), it.next()) {
            cur.push((p, g));
        }
    }
    if !cur.is_empty() {
        out.push(cur);
    }
    out
}

/// One package as KAT's stream: 100 ms idle, its pulses, its last gap merged with 100 ms idle.
pub fn package_wire(pkg: &[(u32, u32)]) -> Vec<LevelDuration> {
    let mut w = vec![LevelDuration::new(false, 100_000)];
    for &(p, g) in pkg {
        w.push(LevelDuration::new(true, p));
        w.push(LevelDuration::new(false, g));
    }
    w.last_mut().unwrap().duration_us += 100_000;
    w
}

/// xorshift64* for the fuzz trains.
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
    pub fn chance(&mut self, percent: u64) -> bool {
        self.below(100) < percent
    }
    /// `v` ± `spread` µs.
    pub fn jitter(&mut self, v: u32, spread: u32) -> u32 {
        (i64::from(v) + self.below(2 * u64::from(spread) + 1) as i64 - i64::from(spread)).max(1) as u32
    }
}

/// Random edits of a train: jitter, retimed / dropped / duplicated / split pairs, cut ends.
pub fn mutate(rng: &mut Rng, pairs: &mut Vec<(u32, u32)>, interesting: &[u32]) {
    let edits = rng.below(4);
    for _ in 0..edits {
        if pairs.is_empty() {
            return;
        }
        let i = rng.below(pairs.len() as u64) as usize;
        match rng.below(9) {
            0 => pairs[i].0 = rng.jitter(pairs[i].0, pairs[i].0 / 3 + 1),
            1 => pairs[i].1 = rng.jitter(pairs[i].1, pairs[i].1 / 3 + 1),
            2 => {
                pairs.remove(i);
            }
            3 => {
                let p = pairs[i];
                pairs.insert(i, p);
            }
            4 => pairs.truncate(i + 1),
            5 => pairs[i].0 = interesting[rng.below(interesting.len() as u64) as usize],
            6 => pairs[i].1 = interesting[rng.below(interesting.len() as u64) as usize],
            7 => {
                pairs.drain(..i);
            }
            _ => {
                let p = pairs[i];
                pairs[i] = (p.0 / 2 + 1, 1 + rng.below(40) as u32);
                pairs.insert(i + 1, (p.0 / 2 + 1, p.1));
            }
        }
    }
}

/// The fuzz dump: for `n` trains from `gen`, write `<dir>/<name>.trains` (one train per line,
/// `pulse gap …` µs — the C harness's input) and `<dir>/<name>.kat` (`<line>\t<json>` per report,
/// `<line>\tEND` per train — the harness's output format), with `KAT_RTL433_FUZZ_DIR=<dir>`
/// (`KAT_RTL433_FUZZ_SEED` changes the seed). Each train goes to a fresh decoder.
pub fn dump_fuzz_trains(
    name: &str,
    n: usize,
    mut gen: impl FnMut(&mut Rng, usize) -> Vec<(u32, u32)>,
    mut dec: impl FnMut() -> Box<dyn ProtocolDecoder>,
    json: fn(&DecodedSignal) -> String,
) {
    let Ok(dir) = std::env::var("KAT_RTL433_FUZZ_DIR") else {
        eprintln!("Skip: set KAT_RTL433_FUZZ_DIR");
        return;
    };
    let seed = std::env::var("KAT_RTL433_FUZZ_SEED").ok().and_then(|s| s.parse().ok()).unwrap_or(0x5EED_0001_u64);
    let mut rng = Rng(seed ^ name.bytes().fold(0u64, |a, b| a.wrapping_mul(131).wrapping_add(u64::from(b))) | 1);
    let mut trains = String::new();
    let mut kat = String::new();
    let mut reports = 0;
    for i in 0..n {
        let mut pairs = gen(&mut rng, i);
        if pairs.is_empty() {
            pairs.push((100, 100));
        }
        // every train ends in the pulse detector's end of package
        pairs.last_mut().unwrap().1 = 150_000;
        let line: Vec<String> = pairs.iter().flat_map(|&(p, g)| [p.to_string(), g.to_string()]).collect();
        trains += &(line.join(" ") + "\n");
        let mut d = dec();
        for got in feed_pairs(d.as_mut(), &pairs) {
            kat += &format!("{i}\t{}\n", json(&got));
            reports += 1;
        }
        kat += &format!("{i}\tEND\n");
    }
    std::fs::write(Path::new(&dir).join(format!("{name}.trains")), trains).unwrap();
    std::fs::write(Path::new(&dir).join(format!("{name}.kat")), kat).unwrap();
    eprintln!("{name}: {n} trains, {reports} reports");
}
