//! Transmit waveform generation and the TX sample stream (the HackRF-independent half of TX).
//!
//! A transmission is a protocol's level/duration pairs turned into 8-bit IQ at 2 MS/s, exactly
//! 2 samples per µs (sample boundaries are rounded from the cumulative time, so no drift):
//! - **OOK**: carrier on while the level is high, exact zeros while it is low.
//! - **2-FSK**: continuous phase; high = carrier + deviation, low = carrier − deviation (the
//!   CC1101's asynchronous 2-FSK mode, and KAT's FM receive convention). The deviation comes from
//!   the capture's or the protocol's preset ([crate::radio::preset]).
//! - **Offset TX**: the HackRF LO is tuned [TX_LO_OFFSET_HZ] below the carrier and the waveform is
//!   shifted up digitally, so the LO leakage (a CW tone at the LO frequency) sits 500 kHz away from
//!   the carrier, outside a key-fob receiver's channel, instead of on it during OOK "off" time.
//!   500 kHz = fs/4 keeps the shifted signal (±47.6 kHz at most) inside the HackRF's 1.75 MHz
//!   baseband filter and puts the IQ-imbalance image 1 MHz below the carrier.
//!
//! [TxStream] owns the samples while libhackrf's TX callback drains them and tells the transmitter
//! when the last one has left the host (see [TxStream::is_complete]).

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;

use num_complex::Complex;

use super::demodulator::LevelDuration;
use super::preset::TxModulation;

/// TX sample rate (the HackRF's, as for receive).
pub const TX_SAMPLE_RATE: u32 = 2_000_000;
/// The LO sits this far below the carrier; the waveform is shifted up by the same amount.
pub const TX_LO_OFFSET_HZ: f64 = 500_000.0;
/// Full-scale amplitude of the 8-bit IQ samples.
const TX_AMPLITUDE: f64 = 127.0;
/// libhackrf's `TRANSFER_COUNT`: USB transfers in flight while streaming (4 in every release).
pub const LIBHACKRF_TRANSFER_COUNT: usize = 4;

/// Everything a transmission needs besides the pairs and the carrier frequency.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TxSettings {
    pub modulation: TxModulation,
    /// HackRF TX VGA (IF) gain, 0–47 dB.
    pub vga_gain: u32,
    /// HackRF RF amplifier (about +11 dB).
    pub amp: bool,
}

impl Default for TxSettings {
    /// KAT ≤ 1.4: OOK at full power (TXVGA 47 dB + amp).
    fn default() -> Self {
        Self { modulation: TxModulation::Ook, vga_gain: 47, amp: true }
    }
}

/// LO frequency for a carrier (the carrier minus [TX_LO_OFFSET_HZ]).
pub fn tx_lo_frequency(carrier_hz: u32) -> u64 {
    (f64::from(carrier_hz) - TX_LO_OFFSET_HZ).round() as u64
}

/// Number of samples a signal occupies at `sample_rate` (sum of the rounded pair boundaries).
pub fn tx_sample_count(signal: &[LevelDuration], sample_rate: u32) -> usize {
    let total_us: u64 = signal.iter().map(|p| u64::from(p.duration_us)).sum();
    us_to_samples(total_us, sample_rate) as usize
}

fn us_to_samples(us: u64, sample_rate: u32) -> u64 {
    (us * u64::from(sample_rate) + 500_000) / 1_000_000
}

/// IQ samples for one transmission: `signal` keyed with `modulation` on a tone `offset_hz` above
/// the LO (0 = at the LO). Sample boundaries follow the cumulative duration, so a pair of `d` µs
/// is exactly `2·d` samples at 2 MS/s. The phase runs continuously through every pair (and through
/// OOK "off" time, which is exact zeros).
pub fn generate_tx_iq(
    signal: &[LevelDuration],
    modulation: TxModulation,
    sample_rate: u32,
    offset_hz: f64,
) -> Vec<Complex<i8>> {
    let fs = f64::from(sample_rate);
    let mut out = Vec::with_capacity(tx_sample_count(signal, sample_rate));
    let mut phase = 0.0f64; // cycles, in [0, 1)
    let mut elapsed_us = 0u64;
    let mut done = 0u64;
    for pair in signal {
        elapsed_us += u64::from(pair.duration_us);
        let end = us_to_samples(elapsed_us, sample_rate);
        let (freq, on) = match modulation {
            TxModulation::Ook => (offset_hz, pair.level),
            TxModulation::Fsk { deviation_hz } => {
                (offset_hz + if pair.level { deviation_hz } else { -deviation_hz }, true)
            }
        };
        let step = freq / fs;
        for _ in done..end {
            out.push(if on {
                let (s, c) = (std::f64::consts::TAU * phase).sin_cos();
                Complex::new((TX_AMPLITUDE * c).round() as i8, (TX_AMPLITUDE * s).round() as i8)
            } else {
                Complex::new(0, 0)
            });
            phase += step;
            phase -= phase.floor();
        }
        done = end;
    }
    out
}

/// Samples queued for libhackrf's TX callback, plus the bookkeeping that tells when they have all
/// been handed to the device.
///
/// libhackrf keeps [LIBHACKRF_TRANSFER_COUNT] USB transfers in flight and calls the TX callback
/// to refill one each time one completes (and, in newer releases, once per transfer to prime the
/// queue). If the callback that wrote the last sample was call `L`, its buffer has left the host
/// when call `L + TRANSFER_COUNT` arrives: the transfers ahead of it complete first, then it does.
/// What remains is the device's own USB buffer (32 KB ≈ 8 ms at 2 MS/s), which the transmitter
/// waits out before stopping.
pub struct TxStream {
    samples: Mutex<Vec<Complex<i8>>>,
    total: usize,
    cursor: AtomicUsize,
    callbacks: AtomicUsize,
    /// 1-based callback index that wrote the last sample (`usize::MAX` until then).
    last_sample_callback: AtomicUsize,
}

impl TxStream {
    pub fn new(samples: Vec<Complex<i8>>) -> Self {
        let total = samples.len();
        Self {
            samples: Mutex::new(samples),
            total,
            cursor: AtomicUsize::new(0),
            callbacks: AtomicUsize::new(0),
            last_sample_callback: AtomicUsize::new(if total == 0 { 0 } else { usize::MAX }),
        }
    }

    /// Fill one libhackrf buffer: the next samples, then zeros (carrier off). Never panics (it runs
    /// on libhackrf's thread, behind an `extern "C"` callback).
    pub fn fill(&self, buf: &mut [Complex<i8>]) {
        let call = self.callbacks.fetch_add(1, Ordering::SeqCst) + 1;
        let start = self.cursor.load(Ordering::SeqCst);
        let n = buf.len().min(self.total.saturating_sub(start));
        let samples = self.samples.lock().unwrap_or_else(|e| e.into_inner());
        let copied = match samples.get(start..start + n) {
            Some(src) => {
                buf[..n].copy_from_slice(src);
                n
            }
            None => 0, // defensive: send silence rather than panic on libhackrf's thread
        };
        drop(samples);
        for s in &mut buf[copied..] {
            *s = Complex::new(0, 0);
        }
        if n > 0 {
            self.cursor.store(start + n, Ordering::SeqCst);
            if start + n == self.total {
                self.last_sample_callback.store(call, Ordering::SeqCst);
            }
        }
    }

    /// All samples have left the host (see the type docs).
    pub fn is_complete(&self) -> bool {
        let last = self.last_sample_callback.load(Ordering::SeqCst);
        last != usize::MAX
            && self.callbacks.load(Ordering::SeqCst) >= last.saturating_add(LIBHACKRF_TRANSFER_COUNT)
    }

    /// Samples handed to libhackrf so far.
    pub fn sent(&self) -> usize {
        self.cursor.load(Ordering::SeqCst)
    }
}

/// Test helper: transmit `signal` with `m` on the 500 kHz offset, remove the offset digitally
/// (what a receiver tuned to the carrier sees), add idle time without carrier (20 ms before,
/// 120 ms after, so the 80 ms end-of-signal gap closes the capture) and run KAT 1.4's
/// `FmDemodulator` (at zero noise an ideal discriminator; it checks the waveform's levels and
/// timing). The loopback through the live receive pipeline is
/// `radio::sim::tx_loopback_decodes_through_the_rx_pipeline`. Returns the captures it emits.
#[cfg(test)]
pub(crate) fn fm_loopback(signal: &[LevelDuration], m: TxModulation) -> Vec<Vec<LevelDuration>> {
    use crate::radio::demodulator::FmDemodulator;
    let fs = TX_SAMPLE_RATE;
    let tx = generate_tx_iq(signal, m, fs, TX_LO_OFFSET_HZ);
    let mut iq = vec![0i8; 2 * 40_000];
    iq.reserve(tx.len() * 2 + 2 * 240_000);
    for (n, s) in tx.iter().enumerate() {
        let (sin, cos) = (-std::f64::consts::TAU * TX_LO_OFFSET_HZ * n as f64 / f64::from(fs)).sin_cos();
        let (re, im) = (f64::from(s.re), f64::from(s.im));
        iq.push((re * cos - im * sin).round().clamp(-127.0, 127.0) as i8);
        iq.push((re * sin + im * cos).round().clamp(-127.0, 127.0) as i8);
    }
    iq.extend(std::iter::repeat_n(0i8, 2 * 240_000));
    let mut demod = FmDemodulator::new(fs);
    iq.chunks(262_144).filter_map(|c| demod.process_samples(c)).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::radio::preset::{TxModulation, F4, FM476, HONDA1};

    const FS: u32 = TX_SAMPLE_RATE;

    fn pairs(v: &[(bool, u32)]) -> Vec<LevelDuration> {
        v.iter().map(|&(l, d)| LevelDuration::new(l, d)).collect()
    }

    /// Phase step (cycles, in (-0.5, 0.5]) between two consecutive non-zero samples.
    fn step(a: Complex<i8>, b: Complex<i8>) -> f64 {
        let (ar, ai) = (f64::from(a.re), f64::from(a.im));
        let (br, bi) = (f64::from(b.re), f64::from(b.im));
        // arg(b · conj(a))
        (bi * ar - br * ai).atan2(br * ar + bi * ai) / std::f64::consts::TAU
    }

    fn fsk(dev: f64) -> TxModulation {
        TxModulation::Fsk { deviation_hz: dev }
    }

    #[test]
    fn sample_counts_are_exactly_two_per_microsecond() {
        let sig = pairs(&[(false, 1), (true, 250), (false, 501), (true, 7), (false, 10_000)]);
        for m in [TxModulation::Ook, fsk(47_607.42)] {
            for off in [0.0, TX_LO_OFFSET_HZ] {
                let iq = generate_tx_iq(&sig, m, FS, off);
                assert_eq!(iq.len(), 2 * (1 + 250 + 501 + 7 + 10_000), "{m} offset {off}");
            }
        }
        assert_eq!(tx_sample_count(&sig, FS), 2 * 10_759);
        // A non-integer rate still lands every boundary on the rounded cumulative time.
        assert_eq!(tx_sample_count(&pairs(&[(true, 3)]), 2_500_000), 8);
        assert_eq!(generate_tx_iq(&pairs(&[(true, 1), (false, 1), (true, 1)]), TxModulation::Ook, 2_500_000, 0.0).len(), 8);
    }

    #[test]
    fn ook_is_zero_while_low_and_full_scale_on_the_offset_tone_while_high() {
        let sig = pairs(&[(false, 100), (true, 400), (false, 300), (true, 50)]);
        let iq = generate_tx_iq(&sig, TxModulation::Ook, FS, TX_LO_OFFSET_HZ);
        let (lo1, hi1, lo2, hi2) = (0..200, 200..1000, 1000..1600, 1600..1700);
        for i in lo1.chain(lo2) {
            assert_eq!(iq[i], Complex::new(0, 0), "sample {i} must be silent");
        }
        for r in [hi1, hi2] {
            for i in r.clone() {
                let mag = (f64::from(iq[i].re).powi(2) + f64::from(iq[i].im).powi(2)).sqrt();
                assert!((mag - 127.0).abs() < 1.0, "sample {i}: |s| = {mag}");
            }
            // fs/4: a quarter turn per sample, i.e. +500 kHz.
            for i in r.start..r.end - 1 {
                assert!((step(iq[i], iq[i + 1]) - 0.25).abs() < 1e-3, "sample {i}");
            }
        }
        assert_eq!(tx_lo_frequency(433_920_000), 433_420_000);
    }

    #[test]
    fn fsk_instantaneous_frequency_is_offset_plus_minus_deviation_with_continuous_phase() {
        for (dev, off) in [(47_607.42, TX_LO_OFFSET_HZ), (14_282.23, TX_LO_OFFSET_HZ), (15_869.14, 0.0)] {
            let sig = pairs(&[(false, 1000), (true, 250), (false, 250), (true, 500), (false, 65), (true, 130)]);
            let iq = generate_tx_iq(&sig, fsk(dev), FS, off);
            // Expected frequency of every sample interval, from the pairs.
            let mut want = Vec::new();
            for p in &sig {
                let f = off + if p.level { dev } else { -dev };
                want.extend(std::iter::repeat_n(f, 2 * p.duration_us as usize));
            }
            assert_eq!(iq.len(), want.len());
            // Phase continuity: every sample's angle follows the ideal continuous-phase trajectory
            // (the running integral of the pair frequencies) within 8-bit quantisation, which at
            // radius 127 is asin(0.5·√2 / 127) ≈ 0.00089 cycles. A phase jump at a bit boundary,
            // or a frequency error accumulating over the pairs, would break this.
            let mut ideal = 0.0f64;
            let mut max_phase_err = 0.0f64;
            let mut max_step_err = 0.0f64;
            for i in 0..iq.len() {
                let mag = (f64::from(iq[i].re).powi(2) + f64::from(iq[i].im).powi(2)).sqrt();
                assert!((mag - 127.0).abs() < 1.0, "constant envelope (sample {i})");
                let angle = f64::from(iq[i].im).atan2(f64::from(iq[i].re)) / std::f64::consts::TAU;
                let mut e = angle - ideal;
                e -= e.round();
                max_phase_err = max_phase_err.max(e.abs());
                if i + 1 < iq.len() {
                    // Instantaneous frequency of sample i's interval vs the pair it belongs to.
                    let f = step(iq[i], iq[i + 1]) * f64::from(FS);
                    let w = if want[i] > f64::from(FS) / 2.0 { want[i] - f64::from(FS) } else { want[i] };
                    max_step_err = max_step_err.max((f - w).abs());
                }
                ideal += want[i] / f64::from(FS);
                ideal -= ideal.floor();
            }
            assert!(max_phase_err < 0.001, "dev {dev} off {off}: phase error {max_phase_err} cycles");
            // Two samples' quantisation errors: ≤ 2 × 0.00089 cycles × 2 MS/s ≈ 3.5 kHz.
            assert!(max_step_err < 3_600.0, "dev {dev} off {off}: step error {max_step_err} Hz");
            // Averaged over a pair, the frequency is exact: high = upper tone, low = lower tone.
            let avg = |r: std::ops::Range<usize>| {
                r.clone().take(r.len() - 1).map(|i| step(iq[i], iq[i + 1])).sum::<f64>() / (r.len() - 1) as f64 * f64::from(FS)
            };
            let wrap = |f: f64| if f > f64::from(FS) / 2.0 { f - f64::from(FS) } else { f };
            assert!((avg(0..2000) - wrap(off - dev)).abs() < 20.0, "low tone");
            assert!((avg(2000..2500) - wrap(off + dev)).abs() < 20.0, "high tone");
        }
    }

    #[test]
    fn fsk_never_switches_the_carrier_off() {
        let iq = generate_tx_iq(&pairs(&[(false, 500), (true, 500)]), fsk(47_607.42), FS, 0.0);
        assert!(iq.iter().all(|s| s.re != 0 || s.im != 0));
    }

    #[test]
    fn stream_fills_buffers_then_zeros_and_completes_after_the_in_flight_transfers() {
        let samples: Vec<Complex<i8>> = (0..10).map(|i| Complex::new(i as i8 + 1, -(i as i8) - 1)).collect();
        let stream = TxStream::new(samples.clone());
        let mut buf = vec![Complex::new(9, 9); 4];
        stream.fill(&mut buf); // call 1
        assert_eq!(buf, samples[0..4]);
        stream.fill(&mut buf); // call 2
        assert_eq!(buf, samples[4..8]);
        stream.fill(&mut buf); // call 3: last two samples, then zeros
        assert_eq!(&buf[..2], &samples[8..10]);
        assert_eq!(&buf[2..], &[Complex::new(0, 0); 2]);
        assert_eq!(stream.sent(), 10);
        for call in 4..3 + LIBHACKRF_TRANSFER_COUNT {
            assert!(!stream.is_complete(), "call {call}: buffer 3 is still in flight");
            stream.fill(&mut buf);
            assert_eq!(buf, vec![Complex::new(0, 0); 4]);
        }
        assert!(!stream.is_complete());
        stream.fill(&mut buf); // call 3 + TRANSFER_COUNT: buffer 3 has completed
        assert!(stream.is_complete());
        // Nothing to send: complete once the queue has cycled.
        let empty = TxStream::new(Vec::new());
        for _ in 0..LIBHACKRF_TRANSFER_COUNT {
            assert!(!empty.is_complete());
            empty.fill(&mut buf);
        }
        assert!(empty.is_complete());
    }

    /// End-to-end, no noise: a real capture's fields → the protocol encoder → this module's 2-FSK
    /// at the protocol's deviation on the 500 kHz offset → offset removed → KAT's current
    /// `FmDemodulator` → registry → the same serial. (Ford V1 / F4, which has no IMPORTS capture,
    /// runs the same loopback in `ford_v1.rs`.)
    #[test]
    fn fsk_loopback_through_the_fm_demodulator_and_registry() {
        use crate::export::flipper::import_sub_raw;
        use crate::protocols::ProtocolRegistry;
        use std::path::Path;

        crate::protocols::keys::load_keystore_from_embedded();
        let cases = [
            ("KIA/KIA V5 N1 RAW.sub", "Kia V5", &FM476),
            ("KIA/Kia4_5cl_5op_5tr.sub", "Kia V4", &FM476),
            ("SUZUKI/Suzuki_v0_kd.sub", "Suzuki V0", &FM476),
            ("honda/Unlock_honda.sub", "Honda Static", &HONDA1),
        ];
        for (file, proto, preset) in cases {
            let path = Path::new("IMPORTS").join(file);
            if !path.exists() {
                eprintln!("Skip: {path:?} not found (run from crate root)");
                continue;
            }
            let (freq, raw) = import_sub_raw(&path).unwrap();
            let pairs: Vec<LevelDuration> =
                raw.iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect();
            let mut reg = ProtocolRegistry::new();
            let (_, decoded, _) = reg
                .process_signal_stream(&pairs, freq)
                .into_iter()
                .find(|(n, _, _)| n == proto)
                .unwrap_or_else(|| panic!("{file}: no {proto} decode"));
            let truth = reg.get(proto).unwrap().encode(&decoded, decoded.button.unwrap_or(1)).expect("encode");
            let m = preset.modulation();
            assert!(m.is_fsk(), "{proto}: {m}");
            let caps = fm_loopback(&truth, m);
            assert!(!caps.is_empty(), "{proto}: FM demodulator emitted nothing");
            let hit = caps.iter().any(|p| {
                matches!(ProtocolRegistry::new().process_signal(p, freq),
                    Some((n, d)) if n == proto && d.serial == decoded.serial)
            });
            assert!(hit, "{proto} ({m}): loopback did not decode serial {:X?}", decoded.serial);
        }
    }

    /// High goes out on the upper tone: KAT's FM receiver (frequency above the carrier = high)
    /// reads back the transmitted levels and durations, not their inverse.
    #[test]
    fn fsk_loopback_keeps_levels_and_durations() {
        let sig = pairs(&[(false, 2000), (true, 500), (false, 250), (true, 250), (false, 1000)]);
        let caps = fm_loopback(&sig, F4.modulation());
        let got: Vec<(bool, u32)> = caps[0].iter().map(|p| (p.level, p.duration_us)).collect();
        // The demodulator reports the same levels (durations within its 40 µs debounce/EMA lag).
        let want: [(bool, u32); 3] = [(true, 500), (false, 250), (true, 250)];
        let mid: Vec<(bool, u32)> = got.iter().copied().skip_while(|p| !p.0).take(3).collect();
        assert_eq!(mid.len(), 3, "{got:?}");
        for ((wl, wd), (gl, gd)) in want.iter().zip(&mid) {
            assert_eq!(wl, gl, "{got:?}");
            assert!(wd.abs_diff(*gd) <= 4, "{got:?}");
        }
    }
}
