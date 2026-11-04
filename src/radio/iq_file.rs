//! IQ file input (`:iq`, `kat --iq`, `:load` on a `.cs8`/`.cu8`): run a recording through the same
//! [`RxPipeline`] as live RX on a background thread and emit its captures through the radio event
//! channel, so decoding, de-duplication and research mode behave exactly as for live signals. Works
//! without any radio attached.
//!
//! Formats: `.cs8` (hackrf_transfer, signed 8-bit), `.cu8` (rtl_sdr / rtl_433, unsigned 8-bit),
//! `.cf32` (GNU Radio / URH float32). Centre frequency and sample rate come from the command, else
//! from the file name (KAT `kat_<centre>Hz_<rate>sps[_rx<target>Hz]_…`, rtl_433 `…_433.92M_250k`,
//! gqrx-style bare numbers), else the current frequency and 2 MS/s.

use std::io::Read;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::mpsc::Sender;
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crate::app::RadioEvent;

use super::dsp::IqFormat;
use super::pipeline::RxPipeline;
use super::rx::burst_captures;

/// Default sample rate when neither the command nor the file name gives one.
pub const DEFAULT_FILE_RATE: u32 = 2_000_000;

/// Frequency / rate hints found in a file name.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct NameHints {
    /// Centre frequency of the samples (Hz).
    pub centre: Option<u32>,
    /// Sample rate (S/s).
    pub rate: Option<u32>,
    /// Target the recording was made for, if not the centre (KAT's `rx<target>Hz`).
    pub target: Option<u32>,
}

fn parse_scaled(num: &str, scale: f64) -> Option<f64> {
    let v: f64 = num.parse().ok()?;
    (v.is_finite() && v > 0.0).then_some(v * scale)
}

/// Split a token like "433.92M" into ("433.92", "M").
fn split_number(tok: &str) -> (&str, &str) {
    let end = tok
        .char_indices()
        .find(|&(_, c)| !(c.is_ascii_digit() || c == '.'))
        .map(|(i, _)| i)
        .unwrap_or(tok.len());
    (&tok[..end], &tok[end..])
}

/// Bare 8-digit YYYYMMDD or 6-digit HHMMSS tokens (gqrx/SDR++ time stamps) are not rates.
fn looks_like_date_or_time(num: &str) -> bool {
    let d: Vec<u32> = num.bytes().map(|b| (b - b'0') as u32).collect();
    let two = |i: usize| d[i] * 10 + d[i + 1];
    match d.len() {
        8 => (19..=20).contains(&two(0)) && (1..=12).contains(&two(4)) && (1..=31).contains(&two(6)),
        6 => two(0) <= 23 && two(2) <= 59 && two(4) <= 59,
        _ => false,
    }
}

/// Read frequency/rate hints from a file name (see the module docs for the conventions).
pub fn parse_name_hints(name: &str) -> NameHints {
    let stem = Path::new(name).file_stem().and_then(|s| s.to_str()).unwrap_or(name);
    let mut h = NameHints::default();
    let is_freq = |v: f64| (24e6..=7.25e9).contains(&v);
    let is_rate = |v: f64| (100e3..=40e6).contains(&v);
    // '-' too: rtl_433 test files are named like "hcs362-pwm-button2-868.3M_1000k".
    for tok in stem.split(['_', ' ', ',', '-']) {
        if tok.is_empty() {
            continue;
        }
        if let Some(rest) = tok.strip_prefix("rx") {
            let (num, unit) = split_number(rest);
            if unit.eq_ignore_ascii_case("hz") {
                if let Some(v) = parse_scaled(num, 1.0).filter(|&v| is_freq(v)) {
                    h.target.get_or_insert(v.round() as u32);
                }
            }
            continue;
        }
        let (num, unit) = split_number(tok);
        if num.is_empty() {
            continue;
        }
        let unit = unit.to_ascii_lowercase();
        let (freq, rate) = match unit.as_str() {
            "hz" => (parse_scaled(num, 1.0), None),
            "khz" => (parse_scaled(num, 1e3), None),
            "mhz" => (parse_scaled(num, 1e6), None),
            "sps" | "s" => (None, parse_scaled(num, 1.0)),
            "ksps" | "k" => (None, parse_scaled(num, 1e3)),
            "msps" => (None, parse_scaled(num, 1e6)),
            // "433.92M" is a frequency, "2M" / "2.4M" a sample rate.
            "m" => match parse_scaled(num, 1e6) {
                Some(v) if v >= 24e6 => (Some(v), None),
                v => (None, v),
            },
            // Bare numbers (gqrx): big ones are frequencies, mid-size ones rates (not time stamps).
            "" if !num.contains('.') && !looks_like_date_or_time(num) => match parse_scaled(num, 1.0) {
                Some(v) if is_freq(v) => (Some(v), None),
                Some(v) if v >= 200e3 && is_rate(v) => (None, Some(v)),
                _ => (None, None),
            },
            _ => (None, None),
        };
        if let Some(v) = freq.filter(|&v| is_freq(v)) {
            h.centre.get_or_insert(v.round() as u32);
        }
        if let Some(v) = rate.filter(|&v| is_rate(v)) {
            h.rate.get_or_insert(v.round() as u32);
        }
    }
    h
}

/// Parse a frequency argument: MHz below 10 000, else Hz ("433.92", "433920000", "433.92M").
pub fn parse_frequency_arg(s: &str) -> Option<u32> {
    let t = s.trim().trim_end_matches(['M', 'm']).trim_end_matches("Hz").trim_end_matches(['M', 'm']);
    let v: f64 = t.parse().ok()?;
    let hz = if v < 10_000.0 { v * 1e6 } else { v };
    (24e6..=7.25e9).contains(&hz).then(|| hz.round() as u32)
}

/// Parse a sample-rate argument: "2M", "2.4M", "2048k", "2000000", "2.048" (MS/s below 100).
pub fn parse_rate_arg(s: &str) -> Option<u32> {
    let s = s.trim().to_ascii_lowercase();
    let s = s.trim_end_matches("sps").trim_end_matches("s/s");
    let (num, scale) = if let Some(n) = s.strip_suffix('m') {
        (n, 1e6)
    } else if let Some(n) = s.strip_suffix('k') {
        (n, 1e3)
    } else {
        (s, 1.0)
    };
    let mut v: f64 = num.parse().ok()?;
    v *= scale;
    if scale == 1.0 && v < 100.0 {
        v *= 1e6;
    }
    (100e3..=40e6).contains(&v).then(|| v.round() as u32)
}

/// What to process.
#[derive(Debug, Clone)]
pub struct IqFileSpec {
    pub path: PathBuf,
    pub format: IqFormat,
    /// Centre frequency of the samples.
    pub centre_hz: u32,
    /// Channel to decode (tag for the captures); equals the centre unless the name says otherwise.
    pub target_hz: u32,
    pub sample_rate: u32,
}

impl IqFileSpec {
    /// Build a spec: explicit arguments win, then file-name hints, then the defaults.
    pub fn resolve(path: PathBuf, freq: Option<u32>, rate: Option<u32>, default_freq: u32) -> Result<Self, String> {
        let format = IqFormat::from_path(&path)
            .ok_or_else(|| format!("{}: not an IQ file (.cs8, .cu8, .cf32)", path.display()))?;
        let name = path.file_name().and_then(|n| n.to_str()).unwrap_or("");
        let hints = parse_name_hints(name);
        let centre = freq.or(hints.centre).unwrap_or(default_freq);
        // A KAT recording names its target relative to its centre; keep that offset.
        let target = match (hints.centre, hints.target) {
            (Some(c), Some(t)) => (centre as i64 + t as i64 - c as i64).clamp(1, u32::MAX as i64) as u32,
            _ => centre,
        };
        let sample_rate = rate.or(hints.rate).unwrap_or(DEFAULT_FILE_RATE);
        if sample_rate < 200_000 {
            return Err(format!("Sample rate {sample_rate} S/s is too low (need ≥ 200 kS/s)"));
        }
        Ok(Self { path, format, centre_hz: centre, target_hz: target, sample_rate })
    }

    /// Seconds of IQ in `bytes` bytes.
    pub fn seconds(&self, bytes: u64) -> f64 {
        bytes as f64 / self.format.bytes_per_sample() as f64 / self.sample_rate as f64
    }
}

/// Result of a finished file.
#[derive(Debug, Clone)]
pub struct IqFileSummary {
    pub bursts: u32,
    pub captures: u32,
    pub seconds: f64,
    pub elapsed: Duration,
    pub cancelled: bool,
}

/// A file being processed in the background.
pub struct IqFileJob {
    pub spec: IqFileSpec,
    progress: Arc<AtomicU32>,
    bursts: Arc<AtomicU32>,
    cancel: Arc<AtomicBool>,
    handle: Option<JoinHandle<Result<IqFileSummary, String>>>,
}

impl IqFileJob {
    /// Start processing `spec`; captures go to `events` as `RadioEvent::SignalCaptured`.
    pub fn spawn(spec: IqFileSpec, events: Sender<RadioEvent>) -> Result<Self, String> {
        let file = std::fs::File::open(&spec.path).map_err(|e| format!("{}: {e}", spec.path.display()))?;
        let total = file.metadata().map(|m| m.len()).unwrap_or(0);
        let progress = Arc::new(AtomicU32::new(0));
        let bursts = Arc::new(AtomicU32::new(0));
        let cancel = Arc::new(AtomicBool::new(false));
        let (p, b, c, s) = (progress.clone(), bursts.clone(), cancel.clone(), spec.clone());
        let handle = std::thread::Builder::new()
            .name("kat-iq-file".to_string())
            .spawn(move || run(file, total, &s, &p, &b, &c, &events))
            .map_err(|e| e.to_string())?;
        Ok(Self { spec, progress, bursts, cancel, handle: Some(handle) })
    }

    /// Fraction done (0..=1).
    pub fn progress(&self) -> f32 {
        self.progress.load(Ordering::Relaxed) as f32 / 1000.0
    }

    /// Bursts found so far.
    pub fn bursts(&self) -> u32 {
        self.bursts.load(Ordering::Relaxed)
    }

    pub fn is_finished(&self) -> bool {
        match &self.handle {
            Some(h) => h.is_finished(),
            None => true,
        }
    }

    pub fn cancel(&self) {
        self.cancel.store(true, Ordering::Relaxed);
    }

    /// Wait for the thread and return its result.
    pub fn join(mut self) -> Result<IqFileSummary, String> {
        match self.handle.take().map(|h| h.join()) {
            Some(Ok(r)) => r,
            Some(Err(_)) => Err("IQ file thread panicked".to_string()),
            None => Err("IQ file job already joined".to_string()),
        }
    }
}

impl Drop for IqFileJob {
    fn drop(&mut self) {
        self.cancel.store(true, Ordering::Relaxed);
    }
}

fn run(
    mut file: std::fs::File,
    total: u64,
    spec: &IqFileSpec,
    progress: &AtomicU32,
    bursts_found: &AtomicU32,
    cancel: &AtomicBool,
    events: &Sender<RadioEvent>,
) -> Result<IqFileSummary, String> {
    let t0 = Instant::now();
    let offset = spec.target_hz as f64 - spec.centre_hz as f64;
    if offset.abs() > 0.45 * spec.sample_rate as f64 {
        return Err(format!(
            "target {} Hz is outside the recording ({} Hz ± {} S/s / 2)",
            spec.target_hz, spec.centre_hz, spec.sample_rate
        ));
    }
    let mut pipeline = RxPipeline::new(spec.sample_rate, offset, spec.target_hz);
    let mut buf = vec![0u8; 1 << 20];
    let mut done = 0u64;
    let (mut n_bursts, mut n_caps) = (0u32, 0u32);
    let fs = spec.sample_rate as f64;
    let mut emit = |bs: Vec<super::pipeline::Burst>| -> bool {
        for b in bs {
            n_bursts += 1;
            tracing::debug!(
                "IQ file burst at {:.3} s: {:.1} ms, SNR {:.1} dB, offset {:.1} kHz, dev {:?}",
                b.start_sample as f64 / fs,
                b.duration_us as f64 / 1e3,
                b.snr_db,
                b.offset_hz / 1e3,
                b.deviation_hz
            );
            for cap in burst_captures(&b) {
                n_caps += 1;
                if events.send(RadioEvent::SignalCaptured(cap)).is_err() {
                    return false;
                }
            }
        }
        bursts_found.store(n_bursts, Ordering::Relaxed);
        true
    };
    let mut cancelled = false;
    loop {
        if cancel.load(Ordering::Relaxed) {
            cancelled = true;
            break;
        }
        let n = file.read(&mut buf).map_err(|e| format!("{}: {e}", spec.path.display()))?;
        if n == 0 {
            break;
        }
        done += n as u64;
        let bs = pipeline.process(&buf[..n], spec.format);
        if !emit(bs) {
            cancelled = true;
            break;
        }
        if total > 0 {
            progress.store(((done as f64 / total as f64) * 1000.0).min(1000.0) as u32, Ordering::Relaxed);
        }
    }
    if !cancelled {
        let bs = pipeline.flush();
        emit(bs);
        progress.store(1000, Ordering::Relaxed);
    }
    Ok(IqFileSummary {
        bursts: n_bursts,
        captures: n_caps,
        seconds: spec.seconds(done),
        elapsed: t0.elapsed(),
        cancelled,
    })
}

/// Resolve a user path: as given (with `~/`), else relative to each of `dirs` in turn.
pub fn find_file(user: &str, dirs: &[&Path]) -> Option<PathBuf> {
    let expanded = if let Some(rest) = user.strip_prefix("~/") {
        dirs::home_dir().map(|h| h.join(rest)).unwrap_or_else(|| PathBuf::from(user))
    } else {
        PathBuf::from(user)
    };
    if expanded.is_file() {
        return Some(expanded);
    }
    if expanded.is_relative() {
        for d in dirs {
            let p = d.join(&expanded);
            if p.is_file() {
                return Some(p);
            }
        }
    }
    None
}

/// True for file names the IQ input understands.
pub fn is_iq_path(path: &Path) -> bool {
    IqFormat::from_path(path).is_some()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn name_hints() {
        let h = parse_name_hints("kat_433670000Hz_2000000sps_rx433920000Hz_20261104-153000.cs8");
        assert_eq!(h, NameHints { centre: Some(433_670_000), rate: Some(2_000_000), target: Some(433_920_000) });
        let h = parse_name_hints("g001_433.92M_250k.cu8");
        assert_eq!(h, NameHints { centre: Some(433_920_000), rate: Some(250_000), target: None });
        let h = parse_name_hints("gqrx_20231104_120000_433920000_2000000_fc.raw");
        assert_eq!((h.centre, h.rate), (Some(433_920_000), Some(2_000_000)));
        let h = parse_name_hints("fob 315MHz 2.4M.cs8");
        assert_eq!((h.centre, h.rate), (Some(315_000_000), Some(2_400_000)));
        assert_eq!(parse_name_hints("capture.cs8"), NameHints::default());
        let h = parse_name_hints("hcs362-pwm-button2-868.3M_1000k.cu8");
        assert_eq!((h.centre, h.rate), (Some(868_300_000), Some(1_000_000)));
    }

    #[test]
    fn arguments() {
        assert_eq!(parse_frequency_arg("433.92"), Some(433_920_000));
        assert_eq!(parse_frequency_arg("433920000"), Some(433_920_000));
        assert_eq!(parse_frequency_arg("433.92M"), Some(433_920_000));
        assert_eq!(parse_frequency_arg("x"), None);
        assert_eq!(parse_rate_arg("2M"), Some(2_000_000));
        assert_eq!(parse_rate_arg("2.4"), Some(2_400_000));
        assert_eq!(parse_rate_arg("2048k"), Some(2_048_000));
        assert_eq!(parse_rate_arg("250000"), Some(250_000));
        assert_eq!(parse_rate_arg("2e6"), Some(2_000_000));
        assert_eq!(parse_rate_arg("fast"), None);
    }

    #[test]
    fn spec_resolution() {
        let s = IqFileSpec::resolve(PathBuf::from("kat_433670000Hz_2000000sps_rx433920000Hz_x.cs8"), None, None, 315_000_000).unwrap();
        assert_eq!((s.centre_hz, s.target_hz, s.sample_rate), (433_670_000, 433_920_000, 2_000_000));
        // An explicit centre keeps the recorded target offset.
        let s = IqFileSpec::resolve(PathBuf::from("kat_433670000Hz_2000000sps_rx433920000Hz_x.cs8"), Some(433_680_000), None, 0).unwrap();
        assert_eq!(s.target_hz, 433_930_000);
        let s = IqFileSpec::resolve(PathBuf::from("fob.cu8"), None, Some(2_048_000), 868_350_000).unwrap();
        assert_eq!((s.format, s.centre_hz, s.target_hz, s.sample_rate), (IqFormat::Cu8, 868_350_000, 868_350_000, 2_048_000));
        assert!(IqFileSpec::resolve(PathBuf::from("fob.sub"), None, None, 0).is_err());
    }
}
