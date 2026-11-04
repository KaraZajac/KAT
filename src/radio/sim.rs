//! Synthetic-IQ tests for the receive pipeline (the v1.5 "radio lab", moved into the repo).
//!
//! Frames come from KAT's own encoders (or from real `IMPORTS/*.sub` RAW timings), are modulated as
//! OOK or 2FSK at a fob frequency offset, with random carrier phase, continuous Gaussian noise
//! before and after the burst, optional static DC offset, and quantised to int8 like a HackRF. The
//! IQ goes through [`RxPipeline`] exactly as the HackRF callback feeds it (target at +fs/8 from the
//! LO), and the bursts through the protocol registry.
//!
//! SNR is quoted like the lab: over the full 2 MHz, `10·log10(A² / 2σ²)` with A = 60 LSB.
//!
//! Fast tests (normal suite) use few trials; the `#[ignore]`d `sweep_*` tests print the tables in
//! `docs/radio.md`. Run them in release:
//! `cargo test --release --offline sweep_ -- --ignored --nocapture --test-threads 1`

use super::demodulator::{Demodulator, FmDemodulator, LevelDuration};
use super::dsp::IqFormat;
use super::pipeline::{Burst, RxPipeline};
use crate::capture::{Capture, RfModulation, StoredLevelDuration};
use crate::export::flipper::import_sub_raw;
use crate::protocols::{DecodedSignal, ProtocolRegistry};
use std::path::Path;

/// xorshift64* (same generator as the lab, so trial seeds are reproducible).
pub(crate) struct Rng(u64);

impl Rng {
    fn next_u64(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    /// Two independent standard normal values (Box-Muller).
    fn gauss2(&mut self) -> (f64, f64) {
        let u1 = self.uniform().max(1e-300);
        let u2 = self.uniform();
        let r = (-2.0 * u1.ln()).sqrt();
        let a = std::f64::consts::TAU * u2;
        (r * a.cos(), r * a.sin())
    }
}

const AMP: f64 = 60.0;
const FS: f64 = 2_000_000.0;
/// Live-RX tuning offset at 2 MS/s (the target sits fs/8 above the LO).
const TUNE: f64 = FS / 8.0;

#[derive(Clone, Copy, Debug)]
enum Modu {
    Ook,
    Fsk(f64),
}

/// How a test signal is put on the air.
#[derive(Clone, Copy, Debug)]
struct Spec {
    fs: f64,
    /// Target position in the samples (LO offset): fs/8 for live RX, 0 for a file at the fob frequency.
    tune: f64,
    /// Fob frequency error.
    offset: f64,
    modu: Modu,
    /// Noise standard deviation per component (LSB).
    sigma: f64,
    /// Static DC offset (LSB) on I and Q.
    dc: f64,
    lead_ms: f64,
    tail_ms: f64,
    /// Carrier amplitude (LSB); `sigma` is set relative to it.
    amp: f64,
}

impl Spec {
    fn live(modu: Modu, offset: f64, snr_db: f64) -> Self {
        Spec { fs: FS, tune: TUNE, offset, modu, sigma: sigma_for(snr_db), dc: 0.0, lead_ms: 20.0, tail_ms: 5.0, amp: AMP }
    }

    /// Same SNR with carrier amplitude `amp` (a low amplitude keeps int8 clipping out of the
    /// sensitivity sweeps).
    fn with_amp(mut self, amp: f64) -> Self {
        self.sigma *= amp / self.amp;
        self.amp = amp;
        self
    }
}

fn sigma_for(snr_db: f64) -> f64 {
    (AMP * AMP / (2.0 * 10f64.powf(snr_db / 10.0))).sqrt()
}

fn snr_db(sigma: f64) -> f64 {
    10.0 * (AMP * AMP / (2.0 * sigma * sigma)).log10()
}

/// Several bursts (each a pulse train) separated by `gap_ms` of idle noise, as int8 IQ bytes.
/// OOK: level = carrier on/off. FSK: carrier on for the whole pulse train, level = upper/lower tone.
fn synth_bursts(bursts: &[&[LevelDuration]], spec: &Spec, gap_ms: f64, rng: &mut Rng) -> Vec<u8> {
    let total_us: f64 = bursts.iter().flat_map(|b| b.iter()).map(|l| l.duration_us as f64).sum::<f64>()
        + 1e3 * (spec.lead_ms + spec.tail_ms + gap_ms * bursts.len() as f64);
    let mut out = Vec::with_capacity((total_us * spec.fs / 1e6) as usize * 2 + 16);
    let mut phase = rng.uniform() * std::f64::consts::TAU;
    let mut t_acc = 0f64; // samples owed
    let mut emit = |us: f64, carrier: bool, level: bool, out: &mut Vec<u8>, rng: &mut Rng| {
        t_acc += us * spec.fs / 1e6;
        let n = t_acc.floor() as usize;
        t_acc -= n as f64;
        let f = spec.tune
            + spec.offset
            + match spec.modu {
                Modu::Ook => 0.0,
                Modu::Fsk(dev) => {
                    if level {
                        dev
                    } else {
                        -dev
                    }
                }
            };
        let dphi = std::f64::consts::TAU * f / spec.fs;
        let a = match spec.modu {
            Modu::Ook if carrier && level => spec.amp,
            Modu::Fsk(_) if carrier => spec.amp,
            _ => 0.0,
        };
        for _ in 0..n {
            phase += dphi;
            if phase > std::f64::consts::TAU {
                phase -= std::f64::consts::TAU;
            } else if phase < 0.0 {
                phase += std::f64::consts::TAU;
            }
            let (gi, gq) = rng.gauss2();
            let i = a * phase.cos() + spec.sigma * gi + spec.dc;
            let q = a * phase.sin() + spec.sigma * gq + spec.dc;
            out.push(i.round().clamp(-127.0, 127.0) as i8 as u8);
            out.push(q.round().clamp(-127.0, 127.0) as i8 as u8);
        }
    };
    emit(spec.lead_ms * 1e3, false, false, &mut out, rng);
    for (k, b) in bursts.iter().enumerate() {
        if k > 0 {
            emit(gap_ms * 1e3, false, false, &mut out, rng);
        }
        for ld in b.iter() {
            emit(ld.duration_us as f64, true, ld.level, &mut out, rng);
        }
    }
    emit(spec.tail_ms * 1e3, false, false, &mut out, rng);
    out
}

fn synth(truth: &[LevelDuration], spec: &Spec, rng: &mut Rng) -> Vec<u8> {
    synth_bursts(&[truth], spec, 0.0, rng)
}

/// Feed IQ in HackRF-sized chunks (256 KiB), then flush.
fn run_pipeline(iq: &[u8], spec: &Spec, freq: u32) -> Vec<Burst> {
    let mut p = RxPipeline::new(spec.fs as u32, spec.tune, freq);
    let mut bursts = Vec::new();
    for chunk in iq.chunks(262_144) {
        bursts.extend(p.process(chunk, IqFormat::Cs8));
    }
    bursts.extend(p.flush());
    bursts
}

/// KAT 1.4's demodulators (the "as-is" baseline rows), on IQ with the target at 0 Hz.
fn run_legacy(iq: &[u8], fm: bool) -> Vec<Vec<LevelDuration>> {
    let samples: Vec<i8> = iq.iter().map(|&b| b as i8).collect();
    let mut am = Demodulator::new(2_000_000);
    let mut fmd = FmDemodulator::new(2_000_000);
    let mut caps = Vec::new();
    for chunk in samples.chunks(262_144) {
        let r = if fm { fmd.process_samples(chunk) } else { am.process_samples(chunk) };
        caps.extend(r);
    }
    // Close a capture still open at the end (the 1.4 demods need an 80 ms gap).
    let idle = vec![0i8; 400_000];
    let r = if fm { fmd.process_samples(&idle) } else { am.process_samples(&idle) };
    caps.extend(r);
    caps
}

struct Case {
    proto: &'static str,
    freq: u32,
    truth: Vec<LevelDuration>,
    serial: u32,
}

fn case(path: &str, proto: &'static str) -> Case {
    crate::protocols::keys::load_keystore_from_embedded();
    let (freq, raw) = import_sub_raw(Path::new(path)).unwrap();
    let pairs: Vec<LevelDuration> = raw.iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect();
    let mut reg = ProtocolRegistry::new();
    let d = reg
        .process_signal_stream(&pairs, freq)
        .into_iter()
        .find(|(n, _, _)| n.starts_with(proto))
        .unwrap_or_else(|| panic!("{path}: no {proto} decode"))
        .1;
    let truth = reg.get(proto).unwrap().encode(&d, 0x01).expect("encode");
    Case { proto, freq, truth, serial: d.serial.unwrap() }
}

fn kia_v5() -> Case {
    case("IMPORTS/KIA/KIA V5 N1 RAW.sub", "Kia V5")
}
fn kia_v3() -> Case {
    case("IMPORTS/KIA/KIA V3 N1 RAW.sub", "Kia V3")
}
fn ford_v0() -> Case {
    case("IMPORTS/FORD/2008_Focus_Lock.sub", "Ford V0")
}
fn kia_v1() -> Case {
    case("IMPORTS/KIA/KIA V1 N1 RAW.sub", "Kia V1")
}

fn decodes_as(pairs: &[LevelDuration], c: &Case) -> bool {
    let mut reg = ProtocolRegistry::new();
    matches!(reg.process_signal(pairs, c.freq), Some((n, d)) if n.starts_with(c.proto) && d.serial == Some(c.serial))
}

/// Largest `x` in `lo..=hi` with `ok(x)` true, for `ok` true at `lo` and monotone (true … false).
fn last_true(mut lo: usize, mut hi: usize, ok: impl Fn(usize) -> bool) -> usize {
    while lo < hi {
        let mid = (lo + hi).div_ceil(2);
        if ok(mid) {
            lo = mid;
        } else {
            hi = mid - 1;
        }
    }
    lo
}

/// Smallest slice `[start, end)` of `pairs` that still decodes per `ok` (checked on clean pulses
/// with a leading and trailing gap), widened by a few pulses. Keeps the fast tests short.
fn minimal_slice(pairs: &[LevelDuration], ok: impl Fn(&[LevelDuration]) -> bool) -> (usize, usize) {
    let framed = |a: usize, b: usize| {
        let mut p = vec![LevelDuration::new(false, 20_000)];
        p.extend_from_slice(&pairs[a..b]);
        p.push(LevelDuration::new(false, 100_000));
        ok(&p)
    };
    let n = pairs.len();
    if !framed(0, n) {
        return (0, n);
    }
    // Shortest end (monotone: a longer prefix still decodes), then the latest start.
    let end = n - last_true(0, n, |cut| framed(0, n - cut));
    let start = last_true(0, end.saturating_sub(1), |s| framed(s, end));
    (start.saturating_sub(4), (end + 4).min(n))
}

/// The case cut down to (about) one decodable frame.
fn short(c: &Case) -> Case {
    let (a, b) = minimal_slice(&c.truth, |p| decodes_as(p, c));
    Case { proto: c.proto, freq: c.freq, truth: c.truth[a..b].to_vec(), serial: c.serial }
}

fn burst_decodes(bursts: &[Burst], c: &Case) -> bool {
    bursts
        .iter()
        .any(|b| b.am.iter().chain(b.fm.iter()).any(|p| decodes_as(p, c)))
}

fn seed(t: u64) -> Rng {
    Rng(0x9E37_79B9_7F4A_7C15 ^ (t + 1).wrapping_mul(0x1234_5678_9ABC_DEF1))
}

/// Decode rate over `trials` for one configuration: (successes, bursts emitted).
fn rate(c: &Case, spec: &Spec, trials: u64) -> (u64, usize) {
    let mut ok = 0;
    let mut emitted = 0;
    for t in 0..trials {
        let mut rng = seed(t);
        let iq = synth(&c.truth, spec, &mut rng);
        let bursts = run_pipeline(&iq, spec, c.freq);
        emitted += bursts.len();
        if burst_decodes(&bursts, c) {
            ok += 1;
        }
    }
    (ok, emitted)
}

fn legacy_rate(c: &Case, spec: &Spec, trials: u64) -> u64 {
    let fm = matches!(spec.modu, Modu::Fsk(_));
    let mut ok = 0;
    for t in 0..trials {
        let mut rng = seed(t);
        let mut s = *spec;
        s.tune = 0.0; // 1.4 had no offset tuning: target at 0 Hz
        s.lead_ms = 200.0;
        let iq = synth(&c.truth, &s, &mut rng);
        if run_legacy(&iq, fm).iter().any(|p| decodes_as(p, c)) {
            ok += 1;
        }
    }
    ok
}

// ───────────────────────────── fast acceptance tests ─────────────────────────────

/// FSK at ±47.6 kHz and ±20 kHz deviation decodes at fob offsets up to ±40 kHz, SNR 6 dB.
#[test]
fn fsk_decodes_across_fob_offsets() {
    for c in [short(&kia_v5()), short(&kia_v3()), short(&ford_v0())] {
        for (dev, offsets) in [(47_600.0, &[-40e3, 40e3][..]), (20_000.0, &[-40e3][..])] {
            for &off in offsets {
                let spec = Spec::live(Modu::Fsk(dev), off, 6.0);
                let (ok, _) = rate(&c, &spec, 1);
                assert_eq!(ok, 1, "{} FSK ±{dev} Hz at {off} Hz offset, 6 dB", c.proto);
            }
        }
    }
}

/// OOK at −3 dB (2 MHz) decodes with 10 kHz and 50 kHz fob offsets (the 1.4 demod needed ~18 dB).
#[test]
fn ook_decodes_at_minus_3_db() {
    let c = short(&kia_v1());
    for off in [10e3, 50e3] {
        let spec = Spec::live(Modu::Ook, off, -3.0);
        let (ok, _) = rate(&c, &spec, 1);
        assert_eq!(ok, 1, "Kia V1 OOK at {off} Hz offset, -3 dB");
    }
}

/// A static DC offset (10 LSB on I and Q) breaks neither AM nor FM, with or without offset tuning.
#[test]
fn dc_offset_is_harmless() {
    let ook = short(&kia_v1());
    let fsk = short(&kia_v5());
    for tune in [0.0, TUNE] {
        for (c, modu, off) in [(&ook, Modu::Ook, 10e3), (&fsk, Modu::Fsk(47_600.0), -20e3)] {
            let mut spec = Spec::live(modu, off, 10.0);
            spec.tune = tune;
            spec.dc = 10.0;
            let (ok, _) = rate(c, &spec, 1);
            assert_eq!(ok, 1, "{} with DC 10 LSB, tune {tune}", c.proto);
        }
    }
}

/// One capture per burst: an OOK burst doesn't also come out as FM noise, an FSK burst not as an
/// AM "carrier on" capture (so research mode doesn't show undecodable twins).
#[test]
fn no_am_fm_twins() {
    for (c, modu) in [(short(&kia_v1()), Modu::Ook), (short(&kia_v5()), Modu::Fsk(47_600.0)), (short(&ford_v0()), Modu::Fsk(20_000.0))] {
        let spec = Spec::live(modu, 25e3, 10.0);
        let bursts = run_pipeline(&synth(&c.truth, &spec, &mut seed(4)), &spec, c.freq);
        assert_eq!(bursts.len(), 1, "{}", c.proto);
        let b = &bursts[0];
        match modu {
            Modu::Ook => assert!(b.am.is_some() && b.fm.is_none(), "{} OOK: {b:?}", c.proto),
            Modu::Fsk(_) => assert!(b.fm.is_some() && b.am.is_none(), "{} FSK: {b:?}", c.proto),
        }
    }
}

/// Five seconds of pure noise open no burst (and so emit no capture). The squelch works on the
/// 250 kS/s channel samples, so the test feeds 250 kS/s noise (same channel statistics as 2 MS/s,
/// 8× cheaper in a debug build); `sweep_noise` runs 2 MS/s for longer.
#[test]
fn pure_noise_emits_nothing() {
    let spec = Spec { fs: 250_000.0, tune: 0.0, offset: 0.0, modu: Modu::Ook, sigma: 20.0, dc: 3.0, lead_ms: 5000.0, tail_ms: 0.0, amp: AMP };
    let mut rng = Rng(7);
    let iq = synth(&[], &spec, &mut rng);
    let bursts = run_pipeline(&iq, &spec, 433_920_000);
    assert!(bursts.is_empty(), "{} bursts from 5 s of noise", bursts.len());
}

/// Noise between two bursts doesn't keep a capture open: two presses 150 ms apart are two bursts.
#[test]
fn bursts_close_on_carrier_absence() {
    let c = short(&kia_v5());
    let spec = Spec::live(Modu::Fsk(47_600.0), 15e3, 3.0);
    let mut rng = seed(3);
    let iq = synth_bursts(&[&c.truth, &c.truth], &spec, 150.0, &mut rng);
    let bursts = run_pipeline(&iq, &spec, c.freq);
    assert_eq!(bursts.len(), 2, "{bursts:?}");
    for b in &bursts {
        assert!(b.fm.as_deref().is_some_and(|p| decodes_as(p, &c)));
        assert!(b.am.is_none(), "FSK burst also emitted as AM");
        let dev = b.deviation_hz.unwrap();
        assert!((dev - 47_600.0).abs() < 3_000.0, "deviation {dev}");
        assert!((b.offset_hz - 15e3).abs() < 3_000.0, "offset {}", b.offset_hz);
    }
}

/// One line per burst for assertion messages: offset, SNR, deviation, AM / FM pair counts.
fn summary(bursts: &[Burst]) -> Vec<String> {
    bursts
        .iter()
        .map(|b| {
            format!(
                "{:+.1} kHz {:.1} dB dev {:?} am {} fm {}",
                b.offset_hz / 1e3,
                b.snr_db,
                b.deviation_hz,
                b.am.as_ref().map_or(0, Vec::len),
                b.fm.as_ref().map_or(0, Vec::len)
            )
        })
        .collect()
}

/// Bursts that decode as `c`, with their measured offsets.
fn decoding_offsets(bursts: &[Burst], c: &Case) -> Vec<f32> {
    bursts
        .iter()
        .filter(|b| b.am.iter().chain(b.fm.iter()).any(|p| decodes_as(p, c)))
        .map(|b| b.offset_hz)
        .collect()
}

/// Fobs far off the target frequency (cheap SAW remotes, a Flipper's ±325 kHz OOK650 window)
/// decode, OOK and ±47.6 kHz FSK, 150 and 300 kHz either side, live tuning (the LO is 250 kHz
/// below the target), and the burst reports the true offset (and deviation).
#[test]
fn far_off_frequency_fobs_decode() {
    let ook = short(&kia_v1());
    let fsk = short(&kia_v5());
    for off in [-300e3, -150e3, 150e3, 300e3] {
        for (c, modu) in [(&ook, Modu::Ook), (&fsk, Modu::Fsk(47_600.0))] {
            let spec = Spec::live(modu, off, 6.0);
            let bursts = run_pipeline(&synth(&c.truth, &spec, &mut seed(1)), &spec, c.freq);
            let found = decoding_offsets(&bursts, c);
            assert!(!found.is_empty(), "{} {modu:?} at {off} Hz: no decode {:?}", c.proto, summary(&bursts));
            assert!(found.iter().all(|&o| (o as f64 - off).abs() < 5e3), "{} at {off} Hz: offsets {found:?}", c.proto);
            if let Modu::Fsk(dev) = modu {
                let d = bursts.iter().filter_map(|b| b.deviation_hz).collect::<Vec<_>>();
                assert!(d.iter().any(|&x| (x as f64 - dev).abs() < 3e3), "{} at {off} Hz: deviation {d:?}", c.proto);
            }
        }
    }
}

/// A file recorded at the fob frequency uses its whole band: a fob at the Nyquist edge of a
/// 250 kS/s rtl_433 recording (like the HCS200 remotes 117–125 kHz off) decodes, on either side.
#[test]
fn fob_at_the_edge_of_a_narrow_recording_decodes() {
    let c = short(&kia_v1());
    for off in [-121e3, 117e3, 124e3] {
        let spec = Spec { fs: 250_000.0, tune: 0.0, ..Spec::live(Modu::Ook, off, 3.0) };
        let bursts = run_pipeline(&synth(&c.truth, &spec, &mut seed(2)), &spec, c.freq);
        let found = decoding_offsets(&bursts, &c);
        assert!(!found.is_empty(), "OOK at {off} Hz in a 250 kS/s file: no decode {:?}", summary(&bursts));
        // Offsets are measured on the 250 kHz circle: +124 kHz and −126 kHz are the same carrier.
        let err = |o: f32| ((o as f64 - off + 125e3).rem_euclid(250e3) - 125e3).abs();
        assert!(found.iter().all(|&o| err(o) < 5e3), "{off} Hz: offsets {found:?}");
    }
}

/// Several transmitters at once (each: pulses, modulation, fob offset, amplitude, start in ms) at
/// live tuning, an optional I/Q-imbalance image (dB relative to the signal, mirrored around the
/// LO), Gaussian noise and int8 quantisation.
fn synth_mix(signals: &[(&[LevelDuration], Modu, f64, f64, f64)], sigma: f64, image_db: Option<f64>, rng: &mut Rng) -> Vec<u8> {
    let end_us = signals
        .iter()
        .map(|(t, _, _, _, start)| start * 1e3 + t.iter().map(|l| l.duration_us as f64).sum::<f64>())
        .fold(0f64, f64::max);
    let n = ((end_us + 30e3) * FS / 1e6) as usize;
    let (mut re, mut im) = (vec![0f64; n], vec![0f64; n]);
    for &(truth, modu, offset, amp, start_ms) in signals {
        let mut phase = rng.uniform() * std::f64::consts::TAU;
        let mut k = (start_ms * 1e-3 * FS) as usize;
        let mut t_acc = 0f64;
        for ld in truth {
            t_acc += ld.duration_us as f64 * FS / 1e6;
            let cnt = t_acc.floor() as usize;
            t_acc -= cnt as f64;
            let dev = match modu {
                Modu::Fsk(d) if ld.level => d,
                Modu::Fsk(d) => -d,
                Modu::Ook => 0.0,
            };
            let a = if matches!(modu, Modu::Fsk(_)) || ld.level { amp } else { 0.0 };
            let dphi = std::f64::consts::TAU * (TUNE + offset + dev) / FS;
            for _ in 0..cnt {
                phase = (phase + dphi).rem_euclid(std::f64::consts::TAU);
                if k < n {
                    re[k] += a * phase.cos();
                    im[k] += a * phase.sin();
                }
                k += 1;
            }
        }
    }
    let eps = image_db.map_or(0.0, |db| 10f64.powf(db / 20.0));
    let mut out = Vec::with_capacity(2 * n);
    for k in 0..n {
        // z + ε·conj(z): a tone at +f gets an image at −f.
        let (i, q) = (re[k] + eps * re[k], im[k] - eps * im[k]);
        let (gi, gq) = rng.gauss2();
        out.push((i + sigma * gi).round().clamp(-127.0, 127.0) as i8 as u8);
        out.push((q + sigma * gq).round().clamp(-127.0, 127.0) as i8 as u8);
    }
    out
}

/// Two fobs on different frequencies at the same time (OOK 150 kHz below the target, FSK 150 kHz
/// above, overlapping) are two bursts, and both decode.
#[test]
fn two_fobs_at_once_both_decode() {
    let ook = short(&kia_v1());
    let fsk = short(&kia_v5());
    let sigma = sigma_for(10.0);
    let iq = synth_mix(
        &[(&ook.truth, Modu::Ook, -150e3, AMP, 20.0), (&fsk.truth, Modu::Fsk(47_600.0), 150e3, AMP, 25.0)],
        sigma,
        None,
        &mut seed(5),
    );
    let spec = Spec::live(Modu::Ook, 0.0, 10.0);
    let bursts = run_pipeline(&iq, &spec, ook.freq);
    let (a, b) = (decoding_offsets(&bursts, &ook), decoding_offsets(&bursts, &fsk));
    assert!(a.iter().any(|&o| (o + 150e3).abs() < 5e3), "OOK fob: {a:?} in {:?}", summary(&bursts));
    assert!(b.iter().any(|&o| (o - 150e3).abs() < 5e3), "FSK fob: {b:?} in {:?}", summary(&bursts));
}

/// A carrier that comes on elsewhere in the band (an interferer, 300 kHz from the fob and 15 dB
/// stronger) and stays on doesn't hide or distort a fob keyed while it is on.
#[test]
fn interferer_elsewhere_does_not_hide_a_fob() {
    let c = short(&kia_v1());
    let fob_us: u32 = c.truth.iter().map(|l| l.duration_us).sum();
    let cw = [LevelDuration::new(true, fob_us + 60_000)];
    let iq = synth_mix(
        &[(&cw[..], Modu::Ook, 200e3, 5.6 * AMP / 3.0, 10.0), (&c.truth, Modu::Ook, -100e3, AMP / 3.0, 40.0)],
        sigma_for(10.0) / 3.0,
        None,
        &mut seed(7),
    );
    let bursts = run_pipeline(&iq, &Spec::live(Modu::Ook, 0.0, 10.0), c.freq);
    let found = decoding_offsets(&bursts, &c);
    assert!(found.iter().any(|&o| (o + 100e3).abs() < 5e3), "{found:?} in {:?}", summary(&bursts));
}

/// A strong fob's I/Q-imbalance image (−30 dB, mirrored around the LO, so 540 kHz below the target
/// here) is strong enough for the detector, but it is recognised as the fob's artifact (present
/// only together with it, much weaker): one capture, not two.
#[test]
fn image_of_a_strong_fob_is_not_a_second_capture() {
    let c = short(&kia_v1());
    let iq = synth_mix(&[(&c.truth, Modu::Ook, 40e3, AMP, 20.0)], sigma_for(30.0), Some(-30.0), &mut seed(6));
    let spec = Spec::live(Modu::Ook, 40e3, 30.0);
    let bursts = run_pipeline(&iq, &spec, c.freq);
    assert_eq!(bursts.len(), 1, "{:?}", summary(&bursts));
    assert!(burst_decodes(&bursts, &c) && (bursts[0].offset_hz - 40e3).abs() < 3e3, "{:?}", summary(&bursts));
}

/// The pipeline is rate-aware: rtl_sdr's 2.048 and 2.4 MS/s decode like 2 MS/s.
#[test]
fn other_sample_rates_decode() {
    let fsk = short(&kia_v5());
    let ook = short(&kia_v1());
    for (fs, cases) in [(2_048_000.0, &[0usize, 1][..]), (2_400_000.0, &[0, 1][..]), (1_000_000.0, &[0][..])] {
        for (c, modu) in cases.iter().map(|&k| [(&fsk, Modu::Fsk(47_600.0)), (&ook, Modu::Ook)][k]) {
            let mut spec = Spec::live(modu, 20e3, 6.0);
            spec.fs = fs;
            spec.tune = fs / 8.0;
            let (ok, _) = rate(c, &spec, 1);
            assert_eq!(ok, 1, "{} at {fs} S/s", c.proto);
        }
    }
}

/// Decodes from captures that came through the radio event channel.
fn decodes_from_events(rx: &std::sync::mpsc::Receiver<crate::app::RadioEvent>, c: &Case) -> usize {
    let mut n = 0;
    while let Ok(ev) = rx.try_recv() {
        if let crate::app::RadioEvent::SignalCaptured(cap) = ev {
            let pairs: Vec<LevelDuration> = cap.raw_pairs.iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect();
            assert_eq!(cap.frequency, c.freq, "capture tagged with the target frequency");
            if decodes_as(&pairs, c) {
                n += 1;
            }
        }
    }
    n
}

fn scratch_dir(name: &str) -> std::path::PathBuf {
    let d = std::env::temp_dir().join(format!("kat-sim-{}-{name}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).unwrap();
    d
}

/// `.cs8` and `.cu8` files (recorded at the fob frequency) decode through `IqFileJob` and the
/// radio event channel, with the frequency and rate taken from the file name.
#[test]
fn iq_files_decode_through_the_event_path() {
    use super::iq_file::{IqFileJob, IqFileSpec};
    let fsk = short(&kia_v5());
    let ook = short(&kia_v1());
    let dir = scratch_dir("files");
    for (c, modu) in [(&fsk, Modu::Fsk(47_600.0)), (&ook, Modu::Ook)] {
        let mut spec = Spec::live(modu, 15e3, 6.0);
        spec.tune = 0.0; // recorded centred on the fob frequency, like hackrf_transfer -f <fob>
        let iq = synth(&c.truth, &spec, &mut seed(1));
        for ext in ["cs8", "cu8"] {
            let bytes: Vec<u8> = if ext == "cs8" { iq.clone() } else { iq.iter().map(|&b| (b as i8 as i16 + 128) as u8).collect() };
            let path = dir.join(format!("fob_{}Hz_2000000sps.{ext}", c.freq));
            std::fs::write(&path, &bytes).unwrap();
            let (tx, rx) = std::sync::mpsc::channel();
            let job = IqFileJob::spawn(IqFileSpec::resolve(path, None, None, 1).unwrap(), tx).unwrap();
            let sum = job.join().unwrap();
            assert!(sum.bursts >= 1 && !sum.cancelled, "{} {ext}: {sum:?}", c.proto);
            assert!(decodes_from_events(&rx, c) >= 1, "{} from .{ext}", c.proto);
        }
    }
    let _ = std::fs::remove_dir_all(&dir);
}

/// `:record` writes the raw buffers as received (LO fs/8 below the target); the KAT file name
/// lets `:iq` find the target channel again.
#[test]
fn recording_round_trips_through_iq_input() {
    use super::iq_file::{IqFileJob, IqFileSpec};
    use super::recorder::recording_file_name;
    use super::rx::{lo_for, LiveRx, LiveShared};
    use std::sync::atomic::{AtomicBool, AtomicU32};
    use std::sync::Arc;
    let c = short(&kia_v5());
    let dir = scratch_dir("record");
    let spec = Spec::live(Modu::Fsk(47_600.0), -10e3, 6.0);
    let iq = synth(&c.truth, &spec, &mut seed(2));
    let shared = LiveShared::new(Arc::new(AtomicBool::new(true)), Arc::new(AtomicU32::new(0)), c.freq);
    let path = dir.join(recording_file_name(lo_for(c.freq, 2_000_000), 2_000_000, c.freq, IqFormat::Cs8));
    shared.start_recording(path.clone()).unwrap();
    let (tx, live_rx) = std::sync::mpsc::channel();
    let mut live = LiveRx::new(2_000_000, IqFormat::Cs8, c.freq, tx);
    for buf in iq.chunks(262_144) {
        shared.record(buf); // what the device thread does with every buffer
        live.process(buf, &shared);
    }
    live.finish();
    let sum = shared.stop_recording().unwrap();
    assert_eq!(sum.bytes as usize, iq.len());
    assert_eq!(std::fs::read(&path).unwrap(), iq, "recording is the raw IQ");
    assert!(decodes_from_events(&live_rx, &c) >= 1, "live decode");
    let (tx, rx) = std::sync::mpsc::channel();
    let file_spec = IqFileSpec::resolve(path, None, None, 1).unwrap();
    assert_eq!((file_spec.centre_hz, file_spec.target_hz), (lo_for(c.freq, 2_000_000), c.freq));
    IqFileJob::spawn(file_spec, tx).unwrap().join().unwrap();
    assert!(decodes_from_events(&rx, &c) >= 1, "decode from the recording");
    let _ = std::fs::remove_dir_all(&dir);
}

// ───────────────────────────── real timings (IMPORTS) ─────────────────────────────

/// Golden-file field string (same format as `protocols::tests::golden_fields`).
fn golden_fields(d: &DecodedSignal) -> String {
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

/// Golden decode lines ("Protocol | fields") of one IMPORTS file, in order.
fn golden_for(rel: &str) -> Vec<String> {
    let golden = std::fs::read_to_string("testdata/imports_decodes.txt").expect("golden file");
    golden
        .lines()
        .filter_map(|l| {
            let parts: Vec<&str> = l.split(" | ").collect();
            (parts.len() == 4 && parts[0] == rel).then(|| format!("{} | {}", parts[2], parts[3]))
        })
        .collect()
}

/// Air modulation for a protocol: `Capture::rf_modulation`, with "Both" resolved by the preset the
/// file was recorded with.
fn air_modulation(protocol: &str, preset_is_fsk: bool) -> RfModulation {
    let mut cap = Capture::from_pairs(0, 433_920_000, Vec::<StoredLevelDuration>::new());
    cap.protocol = Some(protocol.to_string());
    match cap.rf_modulation() {
        RfModulation::Both | RfModulation::Unknown => {
            if preset_is_fsk {
                RfModulation::FM
            } else {
                RfModulation::AM
            }
        }
        m => m,
    }
}

/// Re-transmit an IMPORTS capture's RAW timings through the air model and the pipeline, and compare
/// the decodes with the golden file. Each golden decode's stream segment (the pulses after the
/// previous decode up to this one, plus two pulses of context) becomes one burst, so FSK files —
/// whose RAW is continuous demodulator output — get realistic carrier-off gaps between bursts.
/// `max_decodes` limits the work to the first decodes (fast suite). Returns (expected, got).
fn real_timings(rel: &str, snr: f64, fs: f64, max_decodes: usize, minimal: bool) -> (Vec<String>, Vec<String>) {
    crate::protocols::keys::load_keystore_from_embedded();
    let path = Path::new("IMPORTS").join(rel);
    let text = std::fs::read_to_string(&path).unwrap();
    let preset_fsk = text.lines().any(|l| l.starts_with("Preset:") && (l.contains("FSK") || l.contains("Custom")));
    let (freq, raw) = import_sub_raw(&path).unwrap();
    let pairs: Vec<LevelDuration> = raw.iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect();
    let mut reg = ProtocolRegistry::new();
    let stream = reg.process_signal_stream(&pairs, freq);
    let golden = golden_for(rel);
    let stream_lines: Vec<String> = stream.iter().map(|(n, d, _)| format!("{n} | {}", golden_fields(d))).collect();
    assert_eq!(stream_lines, golden, "{rel}: stream decode differs from the golden file");
    let take = stream.len().min(max_decodes);
    let expected: Vec<String> = stream_lines[..take].to_vec();

    let mut got = Vec::new();
    let mut start = 0usize;
    let mut rng = Rng(0xC0FFEE ^ rel.len() as u64);
    for (k, (name, _, seg)) in stream.iter().take(take).enumerate() {
        let end = (start + seg.len() + 2).min(pairs.len());
        let mut first = start;
        if minimal {
            // Fast suite: only the pulses this decode needs (checked on the clean RAW pulses).
            let want = &stream_lines[k];
            let (a, _) = minimal_slice(&pairs[start..end], |p| {
                let mut reg = ProtocolRegistry::new();
                reg.process_signal_stream(p, freq).iter().any(|(n, d, _)| &format!("{n} | {}", golden_fields(d)) == want)
            });
            first = start + a;
        }
        let modu = match air_modulation(name, preset_fsk) {
            RfModulation::FM => Modu::Fsk(if text.contains("FSKDev476") { 47_600.0 } else { 20_000.0 }),
            _ => Modu::Ook,
        };
        let spec = Spec { fs, tune: fs / 8.0, offset: 12e3, modu, sigma: sigma_for(snr), dc: 0.0, lead_ms: 20.0, tail_ms: 5.0, amp: AMP };
        let iq = synth(&pairs[first..end], &spec, &mut rng);
        for b in run_pipeline(&iq, &spec, freq) {
            for p in b.am.iter().chain(b.fm.iter()) {
                let mut reg = ProtocolRegistry::new();
                for (n, d, _) in reg.process_signal_stream(p, freq) {
                    got.push(format!("{n} | {}", golden_fields(&d)));
                }
            }
        }
        start += seg.len();
    }
    (expected, got)
}

/// Multiset comparison: every expected decode must appear (as often), nothing else may.
fn same_decodes(expected: &[String], got: &[String]) -> bool {
    let mut e = expected.to_vec();
    let mut g = got.to_vec();
    e.sort();
    g.sort();
    e == g
}

/// Real pulse timings (AM and FM protocols) survive the air + pipeline at moderate SNR.
#[test]
fn imports_real_timings_decode() {
    if !Path::new("IMPORTS").exists() {
        eprintln!("Skip: IMPORTS not found");
        return;
    }
    for (rel, n) in [
        ("KIA/KIA V1 N1 RAW.sub", 1),
        ("FORD/2008_Focus_Lock.sub", 1),
        ("GROUPE PSA/PSA_523_536.sub", 1),
        ("VAG/VAG_2.sub", 1),
        ("KIA/KIA V5 N1 RAW.sub", 1),
        ("KIA/KIA V3 N1 RAW.sub", 1),
        ("honda/Lock_honda.sub", 1),
        ("SUZUKI/Suzuki_v0_kd.sub", 1),
    ] {
        let (expected, got) = real_timings(rel, 10.0, 1_000_000.0, n, true);
        assert!(same_decodes(&expected, &got), "{rel}:\nexpected {expected:#?}\ngot {got:#?}");
    }
}

// ───────────────────────────── sweeps (tables for docs/radio.md) ─────────────────────────────

fn cell((ok, _): (u64, usize), trials: u64) -> String {
    format!("{ok}/{trials}")
}

/// FSK decode rate vs. fob offset and vs. SNR (6 trials).
#[test]
#[ignore]
fn sweep_fsk() {
    let cases = [kia_v5(), kia_v3(), ford_v0()];
    let offsets = [-40e3, -20e3, -10e3, 0.0, 10e3, 20e3, 30e3, 40e3];
    for dev in [20_000.0, 47_600.0] {
        println!("\nFSK ±{:.1} kHz, SNR 6 dB (2 MHz): decode rate vs fob offset (kHz)", dev / 1e3);
        print!("| {:10} |", "protocol");
        for o in offsets {
            print!(" {:>4} |", o / 1e3);
        }
        println!();
        for c in &cases {
            print!("| {:10} |", c.proto);
            for o in offsets {
                print!(" {:>4} |", cell(rate(c, &Spec::live(Modu::Fsk(dev), o, 6.0), 6), 6));
            }
            println!();
        }
    }
    let snrs = [12.0, 6.0, 3.0, 0.0, -3.0, -6.0, -9.0];
    for dev in [20_000.0, 47_600.0] {
        println!("\nFSK ±{:.1} kHz, 20 kHz fob offset: decode rate vs SNR (dB, 2 MHz)", dev / 1e3);
        print!("| {:16} |", "protocol");
        for s in snrs {
            print!(" {:>5} |", s);
        }
        println!();
        for c in &cases {
            print!("| {:16} |", c.proto);
            for s in snrs {
                print!(" {:>5} |", cell(rate(c, &Spec::live(Modu::Fsk(dev), 20e3, s), 6), 6));
            }
            println!();
            if c.proto == "Kia V5" {
                print!("| {:16} |", "Kia V5 (1.4, 0 Hz)");
                for s in snrs {
                    let mut spec = Spec::live(Modu::Fsk(dev), 0.0, s);
                    spec.offset = 0.0;
                    print!(" {:>5} |", format!("{}/6", legacy_rate(c, &spec, 6)));
                }
                println!();
            }
        }
    }
}

/// OOK decode rate vs SNR at 10 and 50 kHz offsets, plus the 1.4 baseline (6 trials).
#[test]
#[ignore]
fn sweep_ook() {
    let c = kia_v1();
    let ford = ford_v0();
    let snrs = [18.6, 12.6, 9.0, 6.5, 0.5, -3.0, -5.5, -8.0, -10.0];
    println!("\nOOK decode rate vs SNR (dB, 2 MHz), 6 trials");
    print!("| {:28} |", "case");
    for s in snrs {
        print!(" {:>5} |", s);
    }
    println!();
    for (label, c, off) in [
        ("Kia V1, 10 kHz offset", &c, 10e3),
        ("Kia V1, 50 kHz offset", &c, 50e3),
        ("Kia V1, 75 kHz offset", &c, 75e3),
        ("Ford V0 (OOK), 10 kHz", &ford, 10e3),
    ] {
        print!("| {:28} |", label);
        for s in snrs {
            print!(" {:>5} |", cell(rate(c, &Spec::live(Modu::Ook, off, s), 6), 6));
        }
        println!();
    }
    print!("| {:28} |", "Kia V1, 10 kHz, KAT 1.4 demod");
    for s in snrs {
        print!(" {:>5} |", format!("{}/6", legacy_rate(&c, &Spec::live(Modu::Ook, 10e3, s), 6)));
    }
    println!();
}

/// Decode count over `trials` (same seeds as [`rate`]), spread over `KAT_THREADS` threads (6).
fn rate_par(c: &Case, spec: &Spec, trials: u64) -> u64 {
    let threads = std::env::var("KAT_THREADS").ok().and_then(|s| s.parse().ok()).unwrap_or(6u64).max(1);
    std::thread::scope(|s| {
        let handles: Vec<_> = (0..threads)
            .map(|k| {
                s.spawn(move || {
                    (k..trials)
                        .step_by(threads as usize)
                        .filter(|&t| {
                            let iq = synth(&c.truth, spec, &mut seed(t));
                            burst_decodes(&run_pipeline(&iq, spec, c.freq), c)
                        })
                        .count() as u64
                })
            })
            .collect();
        handles.into_iter().map(|h| h.join().unwrap()).sum()
    })
}

/// Comma-separated numbers from the environment, else `default`.
fn env_list(name: &str, default: &[f64]) -> Vec<f64> {
    std::env::var(name)
        .ok()
        .map(|v| v.split(',').filter_map(|x| x.trim().parse().ok()).collect::<Vec<f64>>())
        .filter(|v| !v.is_empty())
        .unwrap_or_else(|| default.to_vec())
}

/// SNR where the decode rate crosses 50 % (linear between the sweep points; SNRs descending).
fn threshold_50(snrs: &[f64], rates: &[f64]) -> String {
    for i in 0..rates.len() {
        if rates[i] < 0.5 {
            if i == 0 {
                return format!("> {}", snrs[0]);
            }
            let (s0, s1, r0, r1) = (snrs[i - 1], snrs[i], rates[i - 1], rates[i]);
            return format!("{:.1}", s0 + (0.5 - r0) / (r1 - r0) * (s1 - s0));
        }
    }
    format!("< {}", snrs[snrs.len() - 1])
}

/// Sensitivity across the receive band: decode rate vs SNR (dB over 2 MHz) at fob offsets from
/// −325 to +325 kHz, live tuning (target at +fs/8, so −250 kHz is the LO / DC spike), with the
/// 50 % point of each row. Env: `KAT_TRIALS` (8), `KAT_AMP` (20 LSB: no int8 clipping),
/// `KAT_OFFSETS` (kHz), `KAT_SNRS` (dB), `KAT_CASES` (`ook,fsk47,fsk20`), `KAT_THREADS` (6).
#[test]
#[ignore]
fn sweep_offset_sensitivity() {
    let trials = std::env::var("KAT_TRIALS").ok().and_then(|s| s.parse().ok()).unwrap_or(8u64);
    let amp = env_list("KAT_AMP", &[20.0])[0];
    let offsets = env_list("KAT_OFFSETS", &[-325.0, -300.0, -200.0, -100.0, 0.0, 100.0, 200.0, 300.0, 325.0]);
    let snrs = env_list("KAT_SNRS", &[4.0, 0.0, -2.0, -4.0, -5.0, -6.0, -7.0, -8.0, -9.0, -10.0, -11.0, -12.0]);
    let wanted = std::env::var("KAT_CASES").unwrap_or_else(|_| "ook,fsk47,fsk20".to_string());
    let mut cases = Vec::new();
    for w in wanted.split(',') {
        match w.trim() {
            "ook" => cases.push(("Kia V1, OOK", kia_v1(), Modu::Ook)),
            "fsk47" => cases.push(("Kia V5, FSK ±47.6 kHz", kia_v5(), Modu::Fsk(47_600.0))),
            "fsk20" => cases.push(("Ford V0, FSK ±20 kHz", ford_v0(), Modu::Fsk(20_000.0))),
            "ook_ford" => cases.push(("Ford V0, OOK", ford_v0(), Modu::Ook)),
            _ => {}
        }
    }
    for (label, c, modu) in &cases {
        println!("\n{label}: decodes/{trials} vs SNR (dB over 2 MHz), A = {amp} LSB, live tuning");
        print!("| offset kHz |");
        for s in &snrs {
            print!(" {s:>5} |");
        }
        println!(" 50 % at |");
        for &off in &offsets {
            print!("| {off:>10} |");
            let mut rates = Vec::new();
            for &s in &snrs {
                let spec = Spec::live(*modu, off * 1e3, s).with_amp(amp);
                let ok = rate_par(c, &spec, trials);
                rates.push(ok as f64 / trials as f64);
                print!(" {:>5} |", format!("{ok}/{trials}"));
            }
            println!(" {:>7} |", threshold_50(&snrs, &rates));
        }
    }
}

/// DC offset tolerance table (6 trials, 6 dB).
#[test]
#[ignore]
fn sweep_dc() {
    let ook = kia_v1();
    let fsk = kia_v5();
    println!("\nStatic DC offset on I and Q, SNR 6 dB, 6 trials");
    println!("| DC (LSB) | tuning | Kia V1 OOK 10k | Kia V5 FSK ±47.6k -20k |");
    for dc in [0.0, 10.0, 30.0] {
        for (tune, label) in [(0.0, "0 Hz (file)"), (TUNE, "fs/8 (live)")] {
            let mut cells = Vec::new();
            for (c, modu, off) in [(&ook, Modu::Ook, 10e3), (&fsk, Modu::Fsk(47_600.0), -20e3)] {
                let mut spec = Spec::live(modu, off, 6.0);
                spec.tune = tune;
                spec.dc = dc;
                cells.push(cell(rate(c, &spec, 6), 6));
            }
            println!("| {dc} | {label} | {} | {} |", cells[0], cells[1]);
        }
    }
}

/// False bursts on long pure noise (several levels).
#[test]
#[ignore]
fn sweep_noise() {
    println!("\nPure noise: bursts emitted");
    for (sigma, secs) in [(3.0, 30.0), (20.0, 30.0), (60.0, 30.0)] {
        let spec = Spec { fs: FS, tune: TUNE, offset: 0.0, modu: Modu::Ook, sigma, dc: 3.0, lead_ms: 1000.0 * secs, tail_ms: 0.0, amp: AMP };
        let mut rng = Rng(11);
        let iq = synth(&[], &spec, &mut rng);
        let bursts = run_pipeline(&iq, &spec, 433_920_000);
        println!("| σ {sigma} LSB | {secs} s | {} bursts |", bursts.len());
    }
}

/// Real timings over every IMPORTS file that has decodes.
#[test]
#[ignore]
fn sweep_imports_real_timings() {
    let golden = std::fs::read_to_string("testdata/imports_decodes.txt").expect("golden file");
    let mut files: Vec<String> = golden
        .lines()
        .filter_map(|l| {
            let p: Vec<&str> = l.split(" | ").collect();
            (p.len() == 4).then(|| p[0].to_string())
        })
        .collect();
    files.dedup();
    let (mut n_ok, mut n_exp, mut n_got) = (0, 0, 0);
    for snr in [10.0] {
        for rel in &files {
            let (expected, got) = real_timings(rel, snr, FS, usize::MAX, false);
            let same = same_decodes(&expected, &got);
            let matched = expected.iter().filter(|e| got.contains(e)).count();
            n_ok += usize::from(same);
            n_exp += expected.len();
            n_got += matched;
            println!("| {rel} | {} | {}/{} | {} |", if same { "same" } else { "DIFF" }, matched, expected.len(), got.len());
        }
    }
    println!("files identical: {n_ok}/{}; decodes found {n_got}/{n_exp}", files.len());
}

/// Real recordings: rtl_433's public test files (`git clone https://github.com/merbanan/rtl_433_tests`;
/// point `KAT_RTL433_TESTS` at its `tests` directory, default `rtl_433_tests/tests`). Skipped when
/// absent. Checks the decodes KAT gets from them (HCS200 remotes 95–125 kHz off at the edge of a
/// 250 kS/s recording among them) and, for remotes without a KAT decoder, that the burst is found
/// at the carrier's real offset (Chrysler up to 384 kHz off) with clean pulses.
#[test]
#[ignore]
fn rtl433_recordings_decode() {
    let root = std::path::PathBuf::from(std::env::var("KAT_RTL433_TESTS").unwrap_or_else(|_| "rtl_433_tests/tests".into()));
    if !root.join("Microchip/HCS200/01").is_dir() {
        eprintln!("Skip: rtl_433 test recordings not found at {}", root.display());
        return;
    }
    crate::protocols::keys::load_keystore_from_embedded();
    // rtl_433 recordings without a rate in the name are 250 kS/s.
    let rate_of = |f: &str| super::iq_file::parse_name_hints(f).rate.is_none().then_some(250_000);
    // (file, protocol, data): one line per expected distinct decode.
    let expected: &[(&str, &str, &str)] = &[
        ("Microchip/HCS200/01/g001-BTN1_433.92M_250k.cu8", "KeeLoq (Magic_3)", "1DB4F14A8490B004"),
        ("Microchip/HCS200/01/g002-BTN1_433.92M_250k.cu8", "KeeLoq (Magic_3)", "2F9DE53C8490B004"),
        ("Microchip/HCS200/01/g003-BTN1_433.92M_250k.cu8", "KeeLoq (Magic_3)", "CE93E10B8490B004"),
        ("Microchip/HCS200/01/g004-BTN1_433.92M_250k.cu8", "KeeLoq (Magic_3)", "2419211B8490B004"),
        ("Microchip/HCS200/01/g006-BTN2_433.92M_250k.cu8", "KeeLoq (Magic_3)", "28C983D18490B002"),
        ("Microchip/HCS200/01/g007-BTN2_433.92M_250k.cu8", "KeeLoq (Magic_3)", "26A38E018490B002"),
        ("Microchip/HCS200/01/g008-BTN2_433.92M_250k.cu8", "KeeLoq (Magic_3)", "C6779F488490B002"),
        ("Microchip/HCS200/01/g009-BTN2_433.92M_250k.cu8", "KeeLoq (Magic_3)", "E6BECB2C8490B002"),
        ("Microchip/HCS200/01/g012-BTN3_433.92M_250k.cu8", "KeeLoq (Magic_3)", "EF48E8168490B009"),
        ("Microchip/HCS200/01/g014-BTN3_433.92M_250k.cu8", "KeeLoq (Magic_3)", "2F5D55588490B009"),
        ("Microchip/HCS200/01/g019-BTN1-BOK_433.92M_250k.cu8", "KeeLoq (Magic_3)", "DA45C57F8490B004"),
        ("Microchip/HCS200/01/g020-BTN1-BOK_433.92M_250k.cu8", "KeeLoq (Magic_3)", "C37C30438490B004"),
        ("fordremote/01/ford-unlock001.cu8", "Ford V0", "55FFAFFE7FA7B763"),
        ("fordremote/01/ford-unlock002.cu8", "Ford V0", "559ECE9F1EC660D6"),
        ("fordremote/01/ford-lock003.cu8", "Ford V0", "559ECE9F1E9661D6"),
        ("fordremote/01/ford-lock004.cu8", "Ford V0", "55FAAAFB7AF2B26E"),
        ("fordremote/01/ford-boot005.cu8", "Ford V0", "5594C49514FC6FDC"),
        ("fordremote/01/ford-boot006.cu8", "Ford V0", "55FAAAFB7A92B26C"),
        ("honda_remote/01/hondacar-bootunlock.cu8", "Honda V0", "F09AF01924A5AA0D"),
        ("honda_remote/01/hondacar-lock2.cu8", "Honda V0", "F09A101924A5CA97"),
        ("honda_remote/01/hondacar-unlock.cu8", "Honda V0", "F09A701924A52AC3"),
        ("general_motors/ABO1502T/01/g001_314.9M_250k.cu8", "GM", "FF4BE2DB3B73EA00"),
        ("general_motors/ABO1502T/01/g002_314.9M_250k.cu8", "GM", "FF4BF1DB3B73EA00"),
        ("general_motors/ABO1502T/01/g006_314.9M_250k.cu8", "GM", "FF4BD3DB3B73EA00"),
        ("general_motors/ABO1502T/01/g007_314.9M_250k.cu8", "GM", "FF4BC4DB3B73EA00"),
        ("general_motors/ABO1502T/01/lock_unlock_314.9M_250k.cu8", "GM", "FF4B1FDB3B73EA6E"),
        ("hyundai_remote/01/gfile002.cu8", "Kia V3", "BCDE8BF6B1EE2006"),
        ("hyundai_remote/01/gfile004.cu8", "Kia V3", "F06A6058B1EE2006"),
        ("hyundai_remote/01/gfile007.cu8", "Kia V3", "1D6C448CB1EE2006"),
        ("hyundai_remote/01/gfile008.cu8", "Kia V3", "F382F660B1EE2006"),
        ("compustar/1wg3r/01/lock_433.92M_250k.cu8", "Scher-Khan [MAGIC CODE, Static]", "30A580FF0"),
        ("compustar/1wg3r/01/lock_start_long_press_433.92M_250k.cu8", "Scher-Khan [MAGIC CODE, Static]", "30A5819E6"),
        ("compustar/1wg3r/01/secondary_mode_433.92M_250k.cu8", "Scher-Khan [MAGIC CODE, Static]", "30A588778"),
        ("compustar/1wg3r/01/secondary_mode_433.92M_250k.cu8", "Scher-Khan [MAGIC CODE, Static]", "30A58827D"),
        ("compustar/1wg3r/01/secondary_mode_433.92M_250k.cu8", "Scher-Khan [MAGIC CODE, Static]", "30A58847B"),
        ("compustar/1wg3r/01/trunk_433.92M_250k.cu8", "Scher-Khan [MAGIC CODE, Static]", "30A5802FD"),
        ("compustar/1wg3r/01/unlock_433.92M_250k.cu8", "Scher-Khan [MAGIC CODE, Static]", "30A5807F8"),
    ];
    let mut reg = ProtocolRegistry::new();
    let mut missing = Vec::new();
    for &(file, proto, data) in expected {
        let r = crate::headless::decode_file(&mut reg, &root.join(file), None, rate_of(file), false).expect(file);
        if !r.decodes.iter().any(|d| d.protocol == proto && d.fields.starts_with(&format!("data={data}/"))) {
            missing.push(format!("{file}: {proto} {data} (got {:?})", r.decodes.iter().map(|d| &d.protocol).collect::<Vec<_>>()));
        }
    }
    // No KAT decoder: the burst is found at the carrier's offset, with clean PWM pulses.
    // (file, rate, offset kHz, high pulse lengths µs)
    let bursts: &[(&str, u32, f32, &[f32])] = &[
        ("chrysler_car_remote/01/lock_315.1M_920k.cu8", 920_000, -318.0, &[350.0, 680.0]),
        ("chrysler_car_remote/02/panic_315.1M_920k.cu8", 920_000, -126.0, &[315.0, 635.0]),
        ("chrysler_car_remote/03/panic_315.1M_920k.cu8", 920_000, -93.0, &[312.0, 625.0]),
        ("chrysler_car_remote/04/panic_315.1M_920k.cu8", 920_000, -384.0, &[335.0, 640.0]),
        ("chrysler_car_remote/05/panic_315.1M_920k.cu8", 920_000, -74.0, &[320.0, 643.0]),
        ("Microchip/HCS361/01/lock_315.1M_250k.cu8", 250_000, -87.0, &[208.0, 412.0]),
        ("Microchip/HCS361/04/panic_315.1M_250k.cu8", 250_000, -90.0, &[208.0, 412.0]),
    ];
    for &(file, rate, off_khz, highs) in bursts {
        let bytes = std::fs::read(root.join(file)).expect(file);
        let mut p = RxPipeline::new(rate, 0.0, 315_000_000);
        let mut got = p.process(&bytes, IqFormat::Cu8);
        got.extend(p.flush());
        let best = got.iter().filter(|b| b.am.is_some()).max_by_key(|b| b.am.as_ref().map_or(0, Vec::len));
        let ok = best.is_some_and(|b| {
            let pulses: Vec<f32> = b.am.as_ref().unwrap().iter().filter(|l| l.level).map(|l| l.duration_us as f32).collect();
            let clean = pulses.iter().filter(|&&d| highs.iter().any(|&h| (d - h).abs() < 0.12 * h)).count();
            (b.offset_hz / 1e3 - off_khz).abs() < 15.0 && b.snr_db > 10.0 && pulses.len() >= 30 && clean as f32 >= 0.9 * pulses.len() as f32
        });
        if !ok {
            missing.push(format!(
                "{file}: no clean burst at {off_khz} kHz: {:?}",
                got.iter().map(|b| (b.offset_hz, b.snr_db, b.am.as_ref().map(|p| p.len()))).collect::<Vec<_>>()
            ));
        }
    }
    assert!(missing.is_empty(), "{}", missing.join("\n"));
}

/// CPU time of the calling thread (s), from `/proc/thread-self/schedstat` (Linux); throughput on a
/// busy machine is measured against this rather than the wall clock.
fn thread_cpu_s() -> Option<f64> {
    let s = std::fs::read_to_string("/proc/thread-self/schedstat").ok()?;
    Some(s.split_whitespace().next()?.parse::<f64>().ok()? / 1e9)
}

/// Pipeline throughput on one thread (run in release): a burst-heavy signal (2.5 s of FSK bursts
/// back to back, the slicer busy all the time) at several input rates, and idle noise at 2 MS/s
/// (what live RX does most of the time). MS/s per CPU second, and per wall second.
#[test]
#[ignore]
fn sweep_throughput() {
    let c = kia_v5();
    let spec = Spec::live(Modu::Fsk(47_600.0), 10e3, 6.0);
    let mut rng = Rng(5);
    let mut iq = Vec::new();
    for _ in 0..10 {
        iq.extend(synth(&c.truth, &spec, &mut rng));
    }
    let noise_spec = Spec { lead_ms: 4000.0, tail_ms: 0.0, ..Spec::live(Modu::Ook, 0.0, 6.0) };
    let noise = synth(&[], &noise_spec, &mut Rng(6));
    for (label, data, fs) in [
        ("bursts", &iq, 2_000_000u32),
        ("bursts", &iq, 2_400_000),
        ("bursts", &iq, 10_000_000),
        ("noise", &noise, 2_000_000),
        ("noise", &noise, 250_000),
    ] {
        let samples = (data.len() / 2) as f64;
        let (t0, c0) = (std::time::Instant::now(), thread_cpu_s());
        let mut p = RxPipeline::new(fs, fs as f64 / 8.0, c.freq);
        let mut n = 0;
        for chunk in data.chunks(262_144) {
            n += p.process(chunk, IqFormat::Cs8).len();
        }
        n += p.flush().len();
        let wall = t0.elapsed().as_secs_f64();
        let cpu = thread_cpu_s().zip(c0).map_or(wall, |(a, b)| a - b);
        println!(
            "| {label} | {:.3} MS/s input | {:.1} MS/s per CPU s ({:.0}x real time) | wall {:.1} MS/s | {n} bursts |",
            fs as f64 / 1e6,
            samples / cpu / 1e6,
            samples / cpu / fs as f64,
            samples / wall / 1e6
        );
    }
}

/// Debug helper: show what the pipeline emits for one configuration.
#[test]
#[ignore]
fn sweep_debug_one() {
    let snr: f64 = std::env::var("KAT_SNR").ok().and_then(|s| s.parse().ok()).unwrap_or(6.0);
    let off: f64 = std::env::var("KAT_OFF").ok().and_then(|s| s.parse().ok()).unwrap_or(0.0);
    let amp = env_list("KAT_AMP", &[AMP])[0];
    let (c, modu) = match std::env::var("KAT_CASE").as_deref() {
        Ok("ook") => (kia_v1(), Modu::Ook),
        Ok("fsk20") => (ford_v0(), Modu::Fsk(20_000.0)),
        _ => (kia_v5(), Modu::Fsk(47_600.0)),
    };
    let spec = Spec::live(modu, off, snr).with_amp(amp);
    let mut rng = seed(std::env::var("KAT_SEED").ok().and_then(|s| s.parse().ok()).unwrap_or(0));
    let iq = synth(&c.truth, &spec, &mut rng);
    let show = |p: &[LevelDuration]| p.iter().take(60).map(|l| format!("{}{}", if l.level { "+" } else { "-" }, l.duration_us)).collect::<Vec<_>>().join(" ");
    println!("truth ({}): {}", c.truth.len(), show(&c.truth));
    for b in run_pipeline(&iq, &spec, c.freq) {
        println!("burst snr {:.1} dB offset {:.0} dev {:?}", b.snr_db, b.offset_hz, b.deviation_hz);
        if let Some(p) = &b.am {
            println!("  AM ({}) {}: {}", p.len(), decodes_as(p, &c), show(p));
        }
        if let Some(p) = &b.fm {
            println!("  FM ({}) {}: {}", p.len(), decodes_as(p, &c), show(p));
        }
    }
    let _ = snr_db(1.0);
}

/// Per-stage cost of the receive path (run in release), per 2 MS/s input sample: conversion, DC
/// removal, the detector's FFT (16 Hann frames of 64 per 512-sample block), and the per-burst
/// channel extraction (mixer + both FIR stages; only paid while bursts are present). CPU time.
#[test]
#[ignore]
fn sweep_stage_costs() {
    use super::dsp::{lowpass_taps, BatchFft, DcTracker, FirDecimator, Nco, C32, FFT_LANES};
    let n = 8_000_000usize;
    let mut rng = Rng(9);
    let bytes: Vec<u8> = (0..2 * n).map(|_| (rng.next_u64() >> 59) as u8).collect();
    let clock = || thread_cpu_s().unwrap_or(0.0);
    let per = |t: f64| (clock() - t) * 1e9 / n as f64;
    let t = clock();
    let mut v = Vec::with_capacity(n);
    for c in bytes.chunks(8192) {
        IqFormat::Cs8.convert(c, &mut v);
    }
    println!("convert {:.1} ns/sample", per(t));
    let t = clock();
    let mut dc = DcTracker::new();
    for c in v.chunks_mut(4096) {
        dc.process(c, true);
    }
    println!("dc {:.1} ns/sample", per(t));
    let t = clock();
    let mut fft = BatchFft::new(64);
    let mut acc = vec![0f32; 64];
    let mut blocks = 0;
    for blk in v.chunks_exact(512 + 32).step_by(1) {
        for batch in 0..2 {
            fft.accumulate_power(blk, std::array::from_fn(|l| (batch * FFT_LANES + l) * 32), &mut acc);
        }
        blocks += 1;
    }
    println!("detector FFT {:.1} ns/sample ({blocks} blocks, {:.0})", per(t) * (544.0 / 512.0), acc[3]);
    let t = clock();
    let mut nco = Nco::new(250e3, 2e6);
    for c in v.chunks_mut(4096) {
        nco.mix(c);
    }
    println!("burst: mixer {:.1} ns/sample", per(t));
    let t = clock();
    let mut s1 = FirDecimator::new(lowpass_taps(2e6, 120e3, 380e3, 60.0), 4);
    let mut mid: Vec<C32> = Vec::with_capacity(n / 4 + 10);
    for c in v.chunks(4096) {
        s1.process(c, &mut mid);
    }
    println!("burst: channel stage 1 ({} taps) {:.1} ns/input sample", s1.len(), per(t));
    let t = clock();
    let mut s2 = FirDecimator::new(lowpass_taps(5e5, 100e3, 150e3, 60.0), 2);
    let mut out: Vec<C32> = Vec::with_capacity(n / 8 + 10);
    for c in mid.chunks(4096) {
        s2.process(c, &mut out);
    }
    println!("burst: channel stage 2 ({} taps) {:.1} ns/input sample", s2.len(), per(t));
    let t = clock();
    let mut p = RxPipeline::new(2_000_000, 250e3, 433_920_000);
    let mut nb = 0;
    for c in bytes.chunks(262_144) {
        nb += p.process(c, IqFormat::Cs8).len();
    }
    println!("pipeline (noise) {:.1} ns/sample, {nb} bursts", per(t));
}

/// Debug helper for real-timing misses: `KAT_FILE="KIA/KIA V1 N1 RAW.sub" cargo test --release
/// sweep_debug_imports -- --ignored --nocapture` shows, per golden decode, what the pipeline made of it.
#[test]
#[ignore]
fn sweep_debug_imports() {
    let rel = std::env::var("KAT_FILE").unwrap_or_else(|_| "KIA/KIA V1 N1 RAW.sub".to_string());
    let snr: f64 = std::env::var("KAT_SNR").ok().and_then(|s| s.parse().ok()).unwrap_or(10.0);
    crate::protocols::keys::load_keystore_from_embedded();
    let path = Path::new("IMPORTS").join(&rel);
    let text = std::fs::read_to_string(&path).unwrap();
    let preset_fsk = text.lines().any(|l| l.starts_with("Preset:") && (l.contains("FSK") || l.contains("Custom")));
    let (freq, raw) = import_sub_raw(&path).unwrap();
    let pairs: Vec<LevelDuration> = raw.iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect();
    let mut reg = ProtocolRegistry::new();
    let stream = reg.process_signal_stream(&pairs, freq);
    let show = |p: &[LevelDuration]| p.iter().map(|l| format!("{}{}", if l.level { "+" } else { "-" }, l.duration_us)).collect::<Vec<_>>().join(" ");
    let mut start = 0usize;
    let mut rng = Rng(0xC0FFEE ^ rel.len() as u64);
    for (k, (name, d, seg)) in stream.iter().enumerate() {
        let end = (start + seg.len() + 2).min(pairs.len());
        let want = format!("{name} | {}", golden_fields(d));
        let modu = match air_modulation(name, preset_fsk) {
            RfModulation::FM => Modu::Fsk(if text.contains("FSKDev476") { 47_600.0 } else { 20_000.0 }),
            _ => Modu::Ook,
        };
        let spec = Spec { fs: FS, tune: TUNE, offset: 12e3, modu, sigma: sigma_for(snr), dc: 0.0, lead_ms: 20.0, tail_ms: 5.0, amp: AMP };
        let iq = synth(&pairs[start..end], &spec, &mut rng);
        let mut got = Vec::new();
        let mut outs = Vec::new();
        for b in run_pipeline(&iq, &spec, freq) {
            for p in b.am.iter().chain(b.fm.iter()) {
                let mut reg = ProtocolRegistry::new();
                for (n, d, _) in reg.process_signal_stream(p, freq) {
                    got.push(format!("{n} | {}", golden_fields(&d)));
                }
                outs.push(p.clone());
            }
        }
        let extra: Vec<&String> = got.iter().filter(|g| **g != want).collect();
        if !extra.is_empty() {
            println!("decode {k} ({want}) EXTRA: {extra:?}");
        }
        if !got.contains(&want) {
            println!("decode {k} MISSING: {want}\n  got: {got:?}");
            // Which framing difference matters? Clean pulses with the pipeline's lead / tail.
            let clean = &pairs[start..end];
            let try_decode = |p: &[LevelDuration]| {
                let mut reg = ProtocolRegistry::new();
                reg.process_signal_stream(p, freq).iter().any(|(n, d, _)| format!("{n} | {}", golden_fields(d)) == want)
            };
            let mut a = clean.to_vec();
            a[0] = LevelDuration::new(false, 20_000);
            let mut b = clean.to_vec();
            b.push(LevelDuration::new(false, 80_000));
            let mut c = a.clone();
            c.push(LevelDuration::new(false, 80_000));
            println!("  clean decodes: as-is {} | long lead {} | + trailing gap {} | both {}", try_decode(clean), try_decode(&a), try_decode(&b), try_decode(&c));
            for o in &outs {
                let mut x = o.clone();
                x[0] = clean[0];
                let mut y = o.clone();
                y.pop();
                y.push(clean[clean.len() - 1]);
                let z: Vec<LevelDuration> = o.iter().zip(clean).map(|(p, c)| LevelDuration::new(p.level, if p.level { c.duration_us } else { p.duration_us })).collect();
                let w: Vec<LevelDuration> = o.iter().zip(clean).map(|(p, c)| LevelDuration::new(p.level, if p.level { p.duration_us } else { c.duration_us })).collect();
                println!(
                    "  pipeline decodes: as-is {} | clean lead {} | clean tail {} | clean highs {} | clean lows {}",
                    try_decode(o),
                    try_decode(&x),
                    try_decode(&y),
                    try_decode(&z),
                    try_decode(&w)
                );
            }
            println!("  clean ({} pairs): {}", end - start, show(&pairs[start..end]));
            for o in &outs {
                println!("  pipeline ({} pairs): {}", o.len(), show(o));
            }
        }
        start += seg.len();
    }
}

// ─── TX → RX loopback (v1.5 integration) ──────────────────────────────────────

/// The transmitter's own samples for `c` (`radio::tx::generate_tx_iq` with the protocol's primary
/// TX preset) as the live receiver sees them. TX puts the carrier `TX_LO_OFFSET_HZ` above its LO and
/// RX tunes its LO `TUNE` below the carrier, so the waveform lands at `TUNE` (+ `carrier_error_hz`,
/// e.g. crystal error). Scaled to the lab amplitude, with noise for `snr_db` and idle noise around
/// it, as int8 IQ bytes.
fn tx_as_received(
    c: &Case,
    carrier_error_hz: f64,
    snr_db: f64,
    rng: &mut Rng,
) -> (Vec<u8>, crate::radio::preset::TxModulation) {
    use crate::radio::preset::choose_tx_preset;
    use crate::radio::tx::{generate_tx_iq, TX_LO_OFFSET_HZ, TX_SAMPLE_RATE};
    let info = crate::protocols::info::lookup(c.proto).expect("protocol metadata");
    let (_, modulation) = choose_tx_preset(None, None, Some(info.presets));
    let fs = f64::from(TX_SAMPLE_RATE);
    assert_eq!(fs, FS);
    let tx = generate_tx_iq(&c.truth, modulation, TX_SAMPLE_RATE, TX_LO_OFFSET_HZ);
    let shift = TUNE - TX_LO_OFFSET_HZ + carrier_error_hz;
    let sigma = sigma_for(snr_db);
    let scale = AMP / 127.0;
    let idle = (0.02 * fs) as usize; // 20 ms of noise before and after
    let mut out = Vec::with_capacity(2 * (2 * idle + tx.len()));
    let push = |re: f64, im: f64, out: &mut Vec<u8>, rng: &mut Rng| {
        let (gi, gq) = rng.gauss2();
        out.push((re + sigma * gi).round().clamp(-127.0, 127.0) as i8 as u8);
        out.push((im + sigma * gq).round().clamp(-127.0, 127.0) as i8 as u8);
    };
    for _ in 0..idle {
        push(0.0, 0.0, &mut out, rng);
    }
    for (n, s) in tx.iter().enumerate() {
        let (sin, cos) = (std::f64::consts::TAU * shift * n as f64 / fs).sin_cos();
        let (re, im) = (f64::from(s.re) * scale, f64::from(s.im) * scale);
        push(re * cos - im * sin, re * sin + im * cos, &mut out, rng);
    }
    for _ in 0..idle {
        push(0.0, 0.0, &mut out, rng);
    }
    (out, modulation)
}

/// KAT's transmitter feeding KAT's receiver: OOK (AM650) and 2-FSK (FM476 ±47.6 kHz, Honda1
/// ±15.9 kHz) decode to the transmitted serial at 6 dB SNR, also with a 20 kHz carrier error, and
/// the receiver measures the deviation that was sent.
#[test]
fn tx_loopback_decodes_through_the_rx_pipeline() {
    let honda = case("IMPORTS/honda/Lock_honda.sub", "Honda Static");
    let cases = [short(&kia_v5()), short(&kia_v3()), short(&kia_v1()), short(&ford_v0()), short(&honda)];
    let spec = Spec::live(Modu::Ook, 0.0, 6.0); // run_pipeline uses only fs and tune
    for c in &cases {
        for (t, err) in [0.0, 20e3].into_iter().enumerate() {
            let mut rng = seed(t as u64);
            let (iq, m) = tx_as_received(c, err, 6.0, &mut rng);
            let bursts = run_pipeline(&iq, &spec, c.freq);
            assert!(
                burst_decodes(&bursts, c),
                "{} sent as {m:?} with {err} Hz carrier error: not decoded",
                c.proto
            );
            // The receiver measures the deviation the transmitter used. Its estimate runs low on
            // very short symbols (Honda Static's 63 µs elements: −12 %), so allow 20 %: enough to
            // catch a wrong preset (FM476's ±47.6 kHz vs Honda1's ±15.9 kHz).
            if let crate::radio::preset::TxModulation::Fsk { deviation_hz } = m {
                let measured: Vec<f32> = bursts.iter().filter_map(|b| b.deviation_hz).collect();
                assert!(
                    measured.iter().any(|&d| (f64::from(d) - deviation_hz).abs() < 0.2 * deviation_hz),
                    "{}: sent ±{deviation_hz:.0} Hz, measured {measured:?}",
                    c.proto
                );
            }
        }
    }
}
