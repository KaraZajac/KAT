//! Headless decoding (`kat --decode <file|dir>...`): run Flipper `.sub` captures and IQ recordings
//! through the protocol registry without the TUI and print what decodes, one line per distinct
//! decode. Meant for scripting, for checking a capture collection, and as the input/output harness
//! for decoder work (the same paths the app uses: `.sub` RAW pairs → `process_signal_stream`, IQ →
//! [`RxPipeline`] → AM and FM pairs of every burst → `process_signal_stream`).
//!
//! Output (stable, `|`-separated, easy to grep or diff):
//!
//! ```text
//! <file> | <freq Hz> | <source> | <protocol> | x<count> | data=<hex>/<bits> sn= btn= cnt= crc= tx=
//! <file> | <freq Hz> | <source> | (none)
//! <file> | error: <message>
//! ```
//!
//! `<source>` is `raw` for a RAW (or BinRAW) `.sub`, `key` for a Flipper key file (Protocol/Key, no
//! RAW_Data), and `am`/`fm` for IQ recordings (which demodulated path decoded).

use std::collections::BTreeMap;
use std::io::Read;
use std::path::{Path, PathBuf};

use crate::capture::RfModulation;
use crate::protocols::{DecodedSignal, FlipperKey, ProtocolRegistry};
use crate::radio::dsp::IqFormat;
use crate::radio::iq_file::IqFileSpec;
use crate::radio::pipeline::RxPipeline;
use crate::radio::LevelDuration;

/// Options for `--decode`.
#[derive(Debug, Default, PartialEq)]
pub struct DecodeOptions {
    pub paths: Vec<PathBuf>,
    /// Centre frequency for IQ files (overrides the file name).
    pub freq: Option<u32>,
    /// Sample rate for IQ files (overrides the file name).
    pub rate: Option<u32>,
    /// Print every decode in stream order instead of one line per distinct decode.
    pub all: bool,
    /// Also print each IQ burst (time, length, SNR, carrier offset and frequency, deviation) and its
    /// pulses (stderr).
    pub verbose: bool,
}

/// One decode as printed.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct DecodeLine {
    pub source: &'static str,
    pub protocol: String,
    pub fields: String,
    /// The decode's frame (`DecodedSignal::data`).
    pub data: u64,
}

/// Result of one file.
#[derive(Debug)]
pub struct FileResult {
    pub frequency: u32,
    /// Decodes in stream order.
    pub decodes: Vec<DecodeLine>,
}

/// The fields every decode line carries (same layout as the IMPORTS golden test).
pub fn fields(d: &DecodedSignal) -> String {
    let hex = |v: Option<u64>| v.map(|v| format!("{v:X}")).unwrap_or_else(|| "-".to_string());
    format!(
        "data={:X}/{}{} sn={} btn={} cnt={} crc={} tx={}",
        d.data,
        d.data_count_bit,
        d.extra.map(|e| format!(" extra={e:X}")).unwrap_or_default(),
        hex(d.serial.map(u64::from)),
        hex(d.button.map(u64::from)),
        hex(d.counter.map(u64::from)),
        u8::from(d.crc_valid),
        u8::from(d.encoder_capable),
    )
}

/// Files under `path` that `--decode` understands (.sub and IQ recordings), sorted; `path` itself
/// when it is a file.
pub fn collect_files(path: &Path) -> Vec<PathBuf> {
    if path.is_file() {
        return vec![path.to_path_buf()];
    }
    let mut out = Vec::new();
    let mut stack = vec![path.to_path_buf()];
    while let Some(dir) = stack.pop() {
        let Ok(rd) = std::fs::read_dir(&dir) else { continue };
        for entry in rd.flatten() {
            let p = entry.path();
            if p.is_dir() {
                stack.push(p);
            } else if is_decodable(&p) {
                out.push(p);
            }
        }
    }
    out.sort();
    out
}

fn is_decodable(p: &Path) -> bool {
    let ext = p.extension().and_then(|e| e.to_str()).map(|e| e.to_ascii_lowercase());
    ext.as_deref() == Some("sub") || IqFormat::from_path(p).is_some()
}

/// IQ bursts: the receive path's protocols first, then all (`process_signal_stream_path`).
fn push_stream(reg: &mut ProtocolRegistry, pairs: &[LevelDuration], freq: u32, source: &'static str, out: &mut Vec<DecodeLine>) {
    let path = match source {
        "am" => Some(RfModulation::AM),
        "fm" => Some(RfModulation::FM),
        _ => None,
    };
    for (name, d, _) in reg.process_signal_stream_path(pairs, freq, path) {
        out.push(DecodeLine { source, protocol: name, fields: fields(&d), data: d.data });
    }
}

/// `.sub` files: Flipper-ARF's strict gate by the file's preset modulation.
fn push_stream_rf(
    reg: &mut ProtocolRegistry,
    pairs: &[LevelDuration],
    freq: u32,
    source: &'static str,
    rf: Option<RfModulation>,
    out: &mut Vec<DecodeLine>,
) {
    for (name, d, _) in reg.process_signal_stream_rf(pairs, freq, rf) {
        out.push(DecodeLine { source, protocol: name, fields: fields(&d), data: d.data });
    }
}

/// Decode one `.sub` file (RAW pairs, or a key file's Protocol/Key through the protocol's own
/// waveform when KAT can rebuild it).
pub fn decode_sub(reg: &mut ProtocolRegistry, path: &Path) -> Result<FileResult, String> {
    let text = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
    let key: FlipperKey = match crate::export::flipper::parse_sub_content(&text).map_err(|e| format!("{e:#}"))? {
        crate::export::flipper::SubContent::Raw(sub) => {
            let pairs: Vec<LevelDuration> = sub.pairs.iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect();
            let mut decodes = Vec::new();
            let rf = sub.preset.as_ref().and_then(|p| p.rf());
            push_stream_rf(reg, &pairs, sub.frequency, "raw", rf, &mut decodes);
            return Ok(FileResult { frequency: sub.frequency, decodes });
        }
        crate::export::flipper::SubContent::Key(key) => key,
    };
    let mut decodes = Vec::new();
    match reg.decode_key(&key) {
        Some((name, d)) => decodes.push(DecodeLine { source: "key", protocol: name, fields: fields(&d), data: d.data }),
        None => {
            return Err(format!(
                "key file for protocol {:?} ({} bits) is not supported",
                key.protocol, key.bits
            ))
        }
    }
    Ok(FileResult { frequency: key.frequency, decodes })
}

/// `--verbose` pulse dump: signed durations (µs), HIGH positive, like a Flipper RAW_Data line.
fn pulses(p: &[LevelDuration]) -> String {
    let v: Vec<String> = p
        .iter()
        .map(|x| if x.level { format!("{}", x.duration_us) } else { format!("-{}", x.duration_us) })
        .collect();
    v.join(" ")
}

/// Decode one IQ recording: every burst's AM and FM pairs (`verbose`: dump bursts to stderr).
pub fn decode_iq(
    reg: &mut ProtocolRegistry,
    path: &Path,
    freq: Option<u32>,
    rate: Option<u32>,
    verbose: bool,
) -> Result<FileResult, String> {
    let spec = IqFileSpec::resolve(path.to_path_buf(), freq, rate, 433_920_000)?;
    let offset = spec.target_hz as f64 - spec.centre_hz as f64;
    let mut pipeline = RxPipeline::new(spec.sample_rate, offset, spec.target_hz);
    let mut file = std::fs::File::open(path).map_err(|e| e.to_string())?;
    let mut buf = vec![0u8; 1 << 20];
    let mut decodes = Vec::new();
    let fs = spec.sample_rate as f64;
    let handle = |reg: &mut ProtocolRegistry, bursts: Vec<crate::radio::pipeline::Burst>, decodes: &mut Vec<DecodeLine>| {
        for b in bursts {
            if verbose {
                eprintln!(
                    "burst @{:.3}s {:.1} ms SNR {:.1} dB offset {:.1} kHz ({:.4} MHz) dev {} am={} fm={}",
                    b.start_sample as f64 / fs,
                    b.duration_us as f64 / 1e3,
                    b.snr_db,
                    b.offset_hz / 1e3,
                    (b.frequency as f64 + b.offset_hz as f64) / 1e6,
                    b.deviation_hz.map_or("-".to_string(), |d| format!("{:.1} kHz", d / 1e3)),
                    b.am.as_ref().map_or(0, Vec::len),
                    b.fm.as_ref().map_or(0, Vec::len),
                );
                for (tag, p) in [("am", &b.am), ("fm", &b.fm)] {
                    if let Some(p) = p {
                        eprintln!("  {tag}: {}", pulses(p));
                    }
                }
            }
            if let Some(p) = &b.am {
                push_stream(reg, p, b.frequency, "am", decodes);
            }
            if let Some(p) = &b.fm {
                push_stream(reg, p, b.frequency, "fm", decodes);
            }
        }
    };
    loop {
        let n = file.read(&mut buf).map_err(|e| e.to_string())?;
        if n == 0 {
            break;
        }
        let bursts = pipeline.process(&buf[..n], spec.format);
        handle(reg, bursts, &mut decodes);
    }
    let bursts = pipeline.flush();
    handle(reg, bursts, &mut decodes);
    Ok(FileResult { frequency: spec.target_hz, decodes })
}

/// Decode any supported file.
pub fn decode_file(
    reg: &mut ProtocolRegistry,
    path: &Path,
    freq: Option<u32>,
    rate: Option<u32>,
    verbose: bool,
) -> Result<FileResult, String> {
    if IqFormat::from_path(path).is_some() {
        decode_iq(reg, path, freq, rate, verbose)
    } else {
        decode_sub(reg, path)
    }
}

/// Lines `--decode` prints for one file.
pub fn format_result(r: &Result<FileResult, String>, path: &Path, all: bool) -> Vec<String> {
    let shown = path.display();
    match r {
        Err(e) => vec![format!("{shown} | error: {e}")],
        Ok(r) if r.decodes.is_empty() => vec![format!("{shown} | {} | - | (none)", r.frequency)],
        Ok(r) if all => r
            .decodes
            .iter()
            .map(|d| format!("{shown} | {} | {} | {} | {}", r.frequency, d.source, d.protocol, d.fields))
            .collect(),
        Ok(r) => {
            // One line per distinct decode, in order of first appearance.
            let mut order: Vec<&DecodeLine> = Vec::new();
            let mut counts: BTreeMap<&DecodeLine, usize> = BTreeMap::new();
            for d in &r.decodes {
                let c = counts.entry(d).or_insert(0);
                if *c == 0 {
                    order.push(d);
                }
                *c += 1;
            }
            order
                .into_iter()
                .map(|d| format!("{shown} | {} | {} | {} | x{} | {}", r.frequency, d.source, d.protocol, counts[d], d.fields))
                .collect()
        }
    }
}

/// `kat --decode`: print the decodes of every file; returns the process exit code (0 when every
/// file was read, 1 when any failed).
pub fn run(opts: &DecodeOptions) -> i32 {
    load_keystores();
    let mut reg = ProtocolRegistry::new();
    let mut failed = false;
    for root in &opts.paths {
        if !root.exists() {
            println!("{} | error: no such file or directory", root.display());
            failed = true;
            continue;
        }
        for path in collect_files(root) {
            let r = decode_file(&mut reg, &path, opts.freq, opts.rate, opts.verbose);
            failed |= r.is_err();
            for line in format_result(&r, &path, opts.all) {
                println!("{line}");
            }
        }
    }
    i32::from(failed)
}

/// The embedded keys and the user's `keeloq_mfcodes_user`, before any decode.
fn load_keystores() {
    crate::protocols::keys::load_keystore_from_embedded();
    if let Some(msg) = crate::storage::resolve_config_dir().and_then(|d| crate::keystore::load_user_keystore(&d)) {
        eprintln!("{msg}");
    }
}

/// `kat --keeloq-bf`: recover a magic-serial KeeLoq manufacturer key from captures of one remote.
#[derive(Debug, Default, PartialEq)]
pub struct KeeloqBfOptions {
    pub paths: Vec<PathBuf>,
    /// Centre frequency / sample rate for IQ files (override the file name).
    pub freq: Option<u32>,
    pub rate: Option<u32>,
    /// Learning types to search, in order (Auto: 6, 7).
    pub types: Vec<u32>,
    /// Append the best key to the user keystore (`keeloq_mfcodes_user`).
    pub save: bool,
}

/// Parse `--type`: `auto`, or learning types 6, 7, 8 (comma separated).
pub fn parse_bf_types(v: &str) -> Result<Vec<u32>, String> {
    if v.eq_ignore_ascii_case("auto") {
        return Ok(crate::protocols::keeloq_bf::AUTO_TYPES.to_vec());
    }
    v.split(',')
        .map(|t| match t.trim() {
            "6" | "7" | "8" => Ok(t.trim().parse().unwrap()),
            _ => Err(format!("invalid --type {v}: auto, 6, 7 or 8 (magic serial 1, 2, 3)")),
        })
        .collect()
}

/// The KeeLoq frames of the files (one per hop code, in file order), with where each came from.
fn keeloq_frames(opts: &KeeloqBfOptions) -> Vec<(crate::protocols::keeloq_bf::Frame, String)> {
    let mut reg = ProtocolRegistry::new();
    let mut frames: Vec<(crate::protocols::keeloq_bf::Frame, String)> = Vec::new();
    for root in &opts.paths {
        for path in collect_files(root) {
            match decode_file(&mut reg, &path, opts.freq, opts.rate, false) {
                Ok(r) => {
                    for d in r.decodes.iter().filter(|d| d.protocol.starts_with("KeeLoq")) {
                        let f = crate::protocols::keeloq_bf::Frame::from_data(d.data);
                        if !frames.iter().any(|(g, _)| g.hop == f.hop) {
                            frames.push((f, format!("{} ({})", path.display(), d.protocol)));
                        }
                    }
                }
                Err(e) => eprintln!("{}: {e}", path.display()),
            }
        }
    }
    frames
}

/// Run `kat --keeloq-bf`; the exit status is 0 when a key was found.
pub fn keeloq_bf(opts: &KeeloqBfOptions) -> i32 {
    use crate::protocols::keeloq_bf::{self as bf, Control, Outcome};
    use std::sync::atomic::Ordering;
    load_keystores();
    let frames = keeloq_frames(opts);
    let Some(serial) = frames.iter().map(|(f, _)| f.serial()).max_by_key(|&s| frames.iter().filter(|(f, _)| f.serial() == s).count()) else {
        eprintln!("No KeeLoq frames in the given files");
        return 1;
    };
    let group: Vec<bf::Frame> = frames.iter().filter(|(f, _)| f.serial() == serial).map(|(f, _)| *f).collect();
    println!("Remote {serial:07X}: {} capture(s)", group.len());
    for (f, from) in frames.iter().filter(|(f, _)| f.serial() == serial) {
        println!("  btn {:X} hop {:08X}  {from}", f.fix >> 28, f.hop);
    }
    let others = frames.len() - group.len();
    if others > 0 {
        println!("  ({others} frame(s) of other remotes left out)");
    }
    if let Err(e) = bf::check_input(&group, &opts.types) {
        eprintln!("Cannot search: {e}");
        return 1;
    }
    let ctl = std::sync::Arc::new(Control::default());
    let worker = std::sync::Arc::clone(&ctl);
    let types = opts.types.clone();
    let job = std::thread::spawn(move || bf::search(&group, &types, &worker));
    let start = std::time::Instant::now();
    eprintln!("Searching type(s) {:?} on {} threads…", opts.types, std::thread::available_parallelism().map_or(1, |n| n.get()));
    while !job.is_finished() {
        std::thread::sleep(std::time::Duration::from_millis(500));
        let (tested, total) = (ctl.tested.load(Ordering::Relaxed), ctl.total.load(Ordering::Relaxed).max(1));
        let secs = start.elapsed().as_secs_f64().max(0.001);
        let rate = tested as f64 / secs;
        let eta = if rate > 0.0 { (total - tested.min(total)) as f64 / rate } else { 0.0 };
        eprint!("\r  {:5.1}%  {:6.0} M keys/s  ETA {:>5.0} s ", tested as f64 * 100.0 / total as f64, rate / 1e6, eta);
    }
    eprintln!();
    match job.join() {
        Ok(Outcome::Found(found)) => {
            for (i, c) in found.iter().enumerate() {
                let counters: Vec<String> = c.counters.iter().map(|n| format!("{n:04X}")).collect();
                println!("{} type {} key {:016X}  counters {}", if i == 0 { "Found" } else { "  also" }, c.learning, c.man_key, counters.join(" "));
            }
            if found.len() > 1 {
                println!("{} keys decrypt every capture; the first has the closest counters. A further capture would tell them apart.", found.len());
            }
            if opts.save {
                let best = &found[0];
                let name = format!("BF_{serial:07X}");
                match crate::storage::resolve_config_dir()
                    .ok_or_else(|| "no config directory".to_string())
                    .and_then(|d| crate::keystore::append_user_key(&d, &name, best.learning, best.man_key).map_err(|e| e.to_string()))
                {
                    Ok(path) => println!("Saved as {name} in {}", path.display()),
                    Err(e) => {
                        eprintln!("Could not save the key: {e}");
                        return 1;
                    }
                }
            }
            0
        }
        Ok(Outcome::NotFound) => {
            println!("No key found ({} keys tried in {:.0} s)", ctl.tested.load(Ordering::Relaxed), start.elapsed().as_secs_f64());
            1
        }
        Ok(Outcome::Cancelled) | Err(_) => 1,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_a_raw_sub_file() {
        crate::protocols::keys::load_keystore_from_embedded();
        let path = Path::new("IMPORTS/KIA/Kia_v3_lock.sub");
        if !path.exists() {
            return;
        }
        let mut reg = ProtocolRegistry::new();
        let r = decode_file(&mut reg, path, None, None, false).unwrap();
        assert!(r.decodes.iter().any(|d| d.protocol.starts_with("Kia")), "{:?}", r.decodes);
        let lines = format_result(&Ok(r), path, false);
        assert!(lines[0].contains(" | raw | Kia"), "{lines:?}");
    }

    /// A key file decodes through its protocol's `from_flipper_key` (source `key`); a key file of a
    /// protocol KAT does not have, and a RAW header without pulses, are errors.
    #[test]
    fn decodes_a_key_file() {
        let dir = std::env::temp_dir().join(format!("kat_headless_key_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let key = dir.join("hyu_v0.sub");
        std::fs::write(
            &key,
            "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
             Preset: FuriHalSubGhzPreset2FSKDev476Async\nProtocol: KIA/HYU V0\nBit: 61\n\
             Key: 0F 73 91 67 BD 75 B2 BD\n",
        )
        .unwrap();
        let unknown = dir.join("unknown.sub");
        std::fs::write(&unknown, "Filetype: Flipper SubGhz Key File\nVersion: 1\nProtocol: Nope\nBit: 8\nKey: 00 00 00 00 00 00 00 01\n").unwrap();
        let empty_raw = dir.join("empty_raw.sub");
        std::fs::write(&empty_raw, "Filetype: Flipper SubGhz RAW File\nVersion: 1\nProtocol: RAW\n").unwrap();

        let mut reg = ProtocolRegistry::new();
        let r = decode_file(&mut reg, &key, None, None, false);
        let lines = format_result(&r, &key, false);
        assert_eq!(lines.len(), 1, "{lines:?}");
        assert!(
            lines[0].ends_with("| 433920000 | key | Kia V0 | x1 | data=F739167BD75B2BD/61 sn=67BD75B btn=2 cnt=7391 crc=1 tx=1"),
            "{lines:?}"
        );
        let e = decode_file(&mut reg, &unknown, None, None, false).unwrap_err();
        assert!(e.contains("\"Nope\"") && e.contains("not supported"), "{e}");
        let e = decode_file(&mut reg, &empty_raw, None, None, false).unwrap_err();
        assert!(e.contains("No RAW_Data"), "{e}");
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn collects_only_decodable_files() {
        let dir = std::env::temp_dir().join(format!("kat_headless_{}", std::process::id()));
        std::fs::create_dir_all(dir.join("sub")).unwrap();
        for f in ["a.sub", "b.cu8", "c.txt", "sub/d.cs8", "sub/e.json"] {
            std::fs::write(dir.join(f), b"").unwrap();
        }
        let files: Vec<String> = collect_files(&dir)
            .iter()
            .map(|p| p.strip_prefix(&dir).unwrap().display().to_string())
            .collect();
        assert_eq!(files, ["a.sub", "b.cu8", "sub/d.cs8"]);
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
