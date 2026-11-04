//! DSP building blocks for the receive pipeline ([`super::pipeline`]).
//!
//! Everything here works on complex `f32` samples in "LSB units" (the int8 scale of a HackRF, so a
//! full-scale carrier has magnitude ~127). The pieces are deliberately small and allocation-free
//! in the per-sample paths, because the pipeline runs inside the HackRF USB callback:
//!
//! - [`IqFormat`]: raw IQ sample formats (`.cs8` HackRF / hackrf_transfer, `.cu8` RTL-SDR / rtl_sdr,
//!   `.cf32` GNU Radio / URH) and their conversion to complex samples.
//! - [`DcTracker`]: removes the static DC offset (LO leakage), learning it only while no burst is open
//!   so a carrier sitting at 0 Hz (a file recorded at the fob frequency) is not eaten.
//! - [`Nco`]: digital mixer that moves the target frequency to 0 Hz (offset tuning).
//! - [`FirDecimator`] + [`lowpass_taps`]: Kaiser-windowed sinc low-pass filters with polyphase
//!   decimation; the pipeline chains two of them as the channel filter.
//! - small helpers used when a burst is sliced: centred moving average, two-means clustering.

use num_complex::{Complex32, Complex64};

/// Complex sample type used throughout the receive pipeline.
pub type C32 = Complex32;

/// Raw interleaved IQ sample formats.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IqFormat {
    /// Signed 8-bit I, Q (HackRF, `hackrf_transfer -r`): `.cs8`
    Cs8,
    /// Unsigned 8-bit I, Q centred on 127.5 (RTL-SDR, `rtl_sdr`): `.cu8`
    Cu8,
    /// Little-endian 32-bit float I, Q in [-1, 1] (GNU Radio, URH, gqrx `.raw`): `.cf32`
    Cf32,
}

impl IqFormat {
    /// Bytes per complex sample.
    pub fn bytes_per_sample(self) -> usize {
        match self {
            IqFormat::Cs8 | IqFormat::Cu8 => 2,
            IqFormat::Cf32 => 8,
        }
    }

    /// Canonical file extension (without the dot).
    pub fn extension(self) -> &'static str {
        match self {
            IqFormat::Cs8 => "cs8",
            IqFormat::Cu8 => "cu8",
            IqFormat::Cf32 => "cf32",
        }
    }

    /// Format from a file extension (case-insensitive). `.cfile`/`.complex`/`.fc32` are cf32 too.
    pub fn from_extension(ext: &str) -> Option<Self> {
        match ext.to_ascii_lowercase().as_str() {
            "cs8" => Some(IqFormat::Cs8),
            "cu8" => Some(IqFormat::Cu8),
            "cf32" | "fc32" | "cfile" | "complex" => Some(IqFormat::Cf32),
            _ => None,
        }
    }

    /// Format of a path from its extension.
    pub fn from_path(path: &std::path::Path) -> Option<Self> {
        path.extension().and_then(|e| e.to_str()).and_then(Self::from_extension)
    }

    /// Convert whole samples from `bytes` (any trailing partial sample is ignored) and append them
    /// to `out` in LSB units (cf32 is scaled by 128 so every format shares one level scale).
    pub fn convert(self, bytes: &[u8], out: &mut Vec<C32>) {
        let n = bytes.len() / self.bytes_per_sample();
        let start = out.len();
        out.resize(start + n, C32::new(0.0, 0.0));
        let dst = &mut out[start..];
        match self {
            IqFormat::Cs8 => {
                for (o, b) in dst.iter_mut().zip(bytes.chunks_exact(2)) {
                    *o = C32::new(b[0] as i8 as f32, b[1] as i8 as f32);
                }
            }
            IqFormat::Cu8 => {
                for (o, b) in dst.iter_mut().zip(bytes.chunks_exact(2)) {
                    *o = C32::new(b[0] as f32 - 127.5, b[1] as f32 - 127.5);
                }
            }
            IqFormat::Cf32 => {
                for (o, b) in dst.iter_mut().zip(bytes.chunks_exact(8)) {
                    let i = f32::from_le_bytes([b[0], b[1], b[2], b[3]]);
                    let q = f32::from_le_bytes([b[4], b[5], b[6], b[7]]);
                    *o = C32::new(i * 128.0, q * 128.0);
                }
            }
        }
    }
}

/// Static DC-offset remover. The estimate is the mean of recent input blocks, updated only when the
/// caller says the channel is idle, and subtracted from every sample.
#[derive(Debug, Clone)]
pub struct DcTracker {
    dc: C32,
    initialized: bool,
}

impl Default for DcTracker {
    fn default() -> Self {
        Self::new()
    }
}

impl DcTracker {
    /// Weight of one block's mean in the running estimate (blocks are ~2 ms → τ ≈ 40 ms).
    const ALPHA: f32 = 0.05;

    pub fn new() -> Self {
        Self { dc: C32::new(0.0, 0.0), initialized: false }
    }

    /// Learn from `block` (if `learn`) and subtract the estimate in place.
    pub fn process(&mut self, block: &mut [C32], learn: bool) {
        if block.is_empty() {
            return;
        }
        if learn || !self.initialized {
            // Blocks are ≤ a few thousand samples of |x| ≤ 128: f32 sums are exact enough.
            let (mut sr, mut si) = ([0f32; 4], [0f32; 4]);
            let chunks = block.chunks_exact(4);
            let rest = chunks.remainder();
            for c in chunks {
                for k in 0..4 {
                    sr[k] += c[k].re;
                    si[k] += c[k].im;
                }
            }
            let mut acc = C32::new(sr.iter().sum(), si.iter().sum());
            for v in rest {
                acc += v;
            }
            let mean = acc / block.len() as f32;
            if self.initialized {
                self.dc += (mean - self.dc) * Self::ALPHA;
            } else {
                self.dc = mean;
                self.initialized = true;
            }
        }
        let dc = self.dc;
        for s in block.iter_mut() {
            *s -= dc;
        }
    }

    /// Current estimate (LSB units).
    #[allow(dead_code)]
    pub fn estimate(&self) -> C32 {
        self.dc
    }

    /// Forget the estimate (e.g. after a retune).
    pub fn reset(&mut self) {
        self.initialized = false;
        self.dc = C32::new(0.0, 0.0);
    }
}

/// Numerically controlled oscillator: multiplies samples by `exp(-j·2π·f·n/fs)`, moving a signal at
/// `+f` to 0 Hz. A table holds one block of the rotation; each block is anchored at the exact
/// phase (kept in f64 cycles), so it neither drifts nor costs a serial complex multiply per sample.
#[derive(Debug, Clone)]
pub struct Nco {
    table: Vec<C32>,
    /// Phase (cycles, 0..1) at the next sample.
    phase: f64,
    /// Phase step per sample (cycles).
    step: f64,
}

impl Nco {
    const TABLE: usize = 4096;

    /// Mixer that shifts a component at `+shift_hz` down to 0 Hz at sample rate `fs`.
    pub fn new(shift_hz: f64, fs: f64) -> Self {
        let step = -shift_hz / fs;
        let table = (0..Self::TABLE)
            .map(|k| {
                let a = std::f64::consts::TAU * step * k as f64;
                C32::new(a.cos() as f32, a.sin() as f32)
            })
            .collect();
        Self { table, phase: 0.0, step }
    }

    pub fn mix(&mut self, buf: &mut [C32]) {
        for block in buf.chunks_mut(Self::TABLE) {
            let a = std::f64::consts::TAU * self.phase;
            let anchor = C32::new(a.cos() as f32, a.sin() as f32);
            for (s, t) in block.iter_mut().zip(&self.table) {
                *s *= anchor * t;
            }
            self.phase = (self.phase + self.step * block.len() as f64).rem_euclid(1.0);
        }
    }
}

/// Modified Bessel function of the first kind, order 0 (for the Kaiser window).
fn bessel_i0(x: f64) -> f64 {
    let y = x * x / 4.0;
    let mut sum = 1.0;
    let mut term = 1.0;
    for k in 1..64 {
        term *= y / (k as f64 * k as f64);
        sum += term;
        if term < 1e-12 * sum {
            break;
        }
    }
    sum
}

/// Kaiser-windowed sinc low-pass FIR (odd length, unity DC gain).
///
/// `pass_hz`/`stop_hz` are the edges of the transition band and `atten_db` the stop-band
/// attenuation; the length follows Kaiser's formula `N ≈ (A − 7.95) / (14.36 · Δf/fs) + 1`.
pub fn lowpass_taps(fs: f64, pass_hz: f64, stop_hz: f64, atten_db: f64) -> Vec<f32> {
    let tw = ((stop_hz - pass_hz) / fs).max(1e-4);
    let mut n = ((atten_db - 7.95) / (14.36 * tw)).ceil() as usize + 1;
    n = n.max(3) | 1; // odd length: symmetric, integer delay
    let beta = if atten_db > 50.0 {
        0.1102 * (atten_db - 8.7)
    } else if atten_db >= 21.0 {
        0.5842 * (atten_db - 21.0).powf(0.4) + 0.07886 * (atten_db - 21.0)
    } else {
        0.0
    };
    let fc = (pass_hz + stop_hz) / 2.0 / fs; // cycles/sample at the -6 dB point
    let m = (n - 1) as f64 / 2.0;
    let i0_beta = bessel_i0(beta);
    let mut taps: Vec<f64> = (0..n)
        .map(|k| {
            let x = k as f64 - m;
            let sinc = if x == 0.0 {
                2.0 * fc
            } else {
                (std::f64::consts::TAU * fc * x).sin() / (std::f64::consts::PI * x)
            };
            let r = x / m;
            let w = bessel_i0(beta * (1.0 - r * r).max(0.0).sqrt()) / i0_beta;
            sinc * w
        })
        .collect();
    let sum: f64 = taps.iter().sum();
    for t in taps.iter_mut() {
        *t /= sum;
    }
    taps.into_iter().map(|t| t as f32).collect()
}

/// Dot product with eight independent accumulators so the compiler can vectorise it.
#[inline]
fn dot(a: &[f32], b: &[f32]) -> f32 {
    let mut acc = [0f32; 8];
    let ca = a.chunks_exact(8);
    let cb = b.chunks_exact(8);
    let (ra, rb) = (ca.remainder(), cb.remainder());
    for (x, y) in ca.zip(cb) {
        let x: &[f32; 8] = x.try_into().expect("chunk of 8");
        let y: &[f32; 8] = y.try_into().expect("chunk of 8");
        for k in 0..8 {
            acc[k] += x[k] * y[k];
        }
    }
    let mut s = acc.iter().sum::<f32>();
    for (x, y) in ra.iter().zip(rb) {
        s += x * y;
    }
    s
}

/// Complex-input FIR low-pass with integer decimation (only every `decim`-th output is computed).
/// Each call appends the input to the last `taps − 1` samples (split into re/im), so every output
/// is two contiguous dot products. The taps are padded with leading zeros to a multiple of eight
/// (same output, no scalar remainder in the vectorised dot products).
#[derive(Debug, Clone)]
pub struct FirDecimator {
    taps: Vec<f32>,
    /// Taps before padding.
    len: usize,
    decim: usize,
    re: Vec<f32>,
    im: Vec<f32>,
    /// Index (in the next input block) of the sample that completes the next output.
    next_out: usize,
}

impl FirDecimator {
    pub fn new(taps: Vec<f32>, decim: usize) -> Self {
        let len = taps.len();
        let mut padded = vec![0.0; (8 - len % 8) % 8];
        padded.extend(taps);
        let hist = padded.len() - 1;
        let decim = decim.max(1);
        Self { taps: padded, len, decim, re: vec![0.0; hist], im: vec![0.0; hist], next_out: decim - 1 }
    }

    /// Number of taps.
    pub fn len(&self) -> usize {
        self.len
    }

    /// Filter `input`, appending one output per `decim` inputs to `out`.
    pub fn process(&mut self, input: &[C32], out: &mut Vec<C32>) {
        let n = self.taps.len();
        let hist = n - 1;
        self.re.extend(input.iter().map(|x| x.re));
        self.im.extend(input.iter().map(|x| x.im));
        let total = self.re.len();
        let mut p = hist + self.next_out; // newest sample of the next output's window
        while p < total {
            let w = p + 1 - n;
            out.push(C32::new(dot(&self.taps, &self.re[w..=p]), dot(&self.taps, &self.im[w..=p])));
            p += self.decim;
        }
        self.next_out = p - total;
        self.re.drain(..total - hist);
        self.im.drain(..total - hist);
    }

    /// Clear the history (after a retune, so old-frequency samples don't leak into new outputs).
    pub fn reset(&mut self) {
        let hist = self.taps.len() - 1;
        self.re.clear();
        self.re.resize(hist, 0.0);
        self.im.clear();
        self.im.resize(hist, 0.0);
        self.next_out = self.decim - 1;
    }
}

/// Frames transformed together by [`BatchFft`] (one per SIMD lane).
pub const FFT_LANES: usize = 8;

/// Radix-2 FFT of [`FFT_LANES`] frames at once, Hann-windowed, giving their summed power
/// spectrum. The frames are independent, so every butterfly works on eight lanes, which the
/// compiler vectorises; this is the receive pipeline's wide-band detector (see
/// [`super::pipeline`]), which runs on every input sample and must be cheap.
#[derive(Debug, Clone)]
pub struct BatchFft {
    n: usize,
    window: Vec<f32>,
    bitrev: Vec<usize>,
    tw_re: Vec<f32>,
    tw_im: Vec<f32>,
    re: Vec<[f32; FFT_LANES]>,
    im: Vec<[f32; FFT_LANES]>,
}

impl BatchFft {
    /// FFT of length `n` (a power of two, ≥ 4) with a periodic Hann window (which sums to a
    /// constant at 50 % overlap).
    pub fn new(n: usize) -> Self {
        assert!(n >= 4 && n.is_power_of_two(), "FFT length {n}");
        let bits = n.trailing_zeros();
        let window = (0..n)
            .map(|i| (0.5 - 0.5 * (std::f64::consts::TAU * i as f64 / n as f64).cos()) as f32)
            .collect();
        let bitrev = (0..n).map(|i| i.reverse_bits() >> (usize::BITS - bits)).collect();
        let (tw_re, tw_im) = (0..n / 2)
            .map(|k| {
                let a = -std::f64::consts::TAU * k as f64 / n as f64;
                (a.cos() as f32, a.sin() as f32)
            })
            .unzip();
        Self { n, window, bitrev, tw_re, tw_im, re: vec![[0.0; FFT_LANES]; n], im: vec![[0.0; FFT_LANES]; n] }
    }

    /// Σ w[i] (a tone of amplitude A at a bin centre has power (A·Σw)² in that bin).
    pub fn coherent_gain(&self) -> f32 {
        self.window.iter().sum()
    }

    /// Σ w[i]² (white noise of variance σ² per sample has power σ²·Σw² in every bin).
    pub fn noise_gain(&self) -> f32 {
        self.window.iter().map(|w| w * w).sum()
    }

    /// Add `Σ_lanes |X_l[k]|²` to `acc[k]` (natural order: bin k is k·fs/n, the upper half
    /// negative frequencies), where frame `l` is `x[starts[l] .. starts[l] + n]`.
    ///
    /// Decimation in time: the windowed load in bit-reversed order is fused with the first two
    /// radix-2 stages (twiddles 1 and −j, so additions only), and the last stage with the power sum.
    #[allow(clippy::needless_range_loop)] // one index into several arrays per lane
    pub fn accumulate_power(&mut self, x: &[C32], starts: [usize; FFT_LANES], acc: &mut [f32]) {
        let n = self.n;
        let frames: [&[C32]; FFT_LANES] = starts.map(|s| &x[s..s + n]);
        for i in 0..n {
            let j = self.bitrev[i];
            let w = self.window[i];
            let (mut r, mut m) = ([0f32; FFT_LANES], [0f32; FFT_LANES]);
            for l in 0..FFT_LANES {
                let v = frames[l][i];
                r[l] = v.re * w;
                m[l] = v.im * w;
            }
            self.re[j] = r;
            self.im[j] = m;
        }
        // First two stages as one radix-4 pass: twiddles 1 and −j, additions only.
        for p in (0..n).step_by(4) {
            let (ar, ai, br, bi) = (self.re[p], self.im[p], self.re[p + 1], self.im[p + 1]);
            let (cr, ci, dr, di) = (self.re[p + 2], self.im[p + 2], self.re[p + 3], self.im[p + 3]);
            let mut o = [[0f32; FFT_LANES]; 8];
            for l in 0..FFT_LANES {
                let (s0r, s0i, s1r, s1i) = (ar[l] + br[l], ai[l] + bi[l], ar[l] - br[l], ai[l] - bi[l]);
                let (s2r, s2i, s3r, s3i) = (cr[l] + dr[l], ci[l] + di[l], cr[l] - dr[l], ci[l] - di[l]);
                o[0][l] = s0r + s2r;
                o[1][l] = s0i + s2i;
                o[2][l] = s1r + s3i;
                o[3][l] = s1i - s3r;
                o[4][l] = s0r - s2r;
                o[5][l] = s0i - s2i;
                o[6][l] = s1r - s3i;
                o[7][l] = s1i + s3r;
            }
            for k in 0..4 {
                self.re[p + k] = o[2 * k];
                self.im[p + k] = o[2 * k + 1];
            }
        }
        let power = |r: &[f32; FFT_LANES], m: &[f32; FFT_LANES]| {
            let mut s = 0f32;
            for l in 0..FFT_LANES {
                s += r[l] * r[l] + m[l] * m[l];
            }
            s
        };
        if n == 4 {
            for (k, a) in acc.iter_mut().enumerate().take(n) {
                *a += power(&self.re[k], &self.im[k]);
            }
            return;
        }
        let mut half = 4;
        while half < n {
            let last = 2 * half == n;
            let step = n / (2 * half);
            for base in (0..n).step_by(2 * half) {
                for k in 0..half {
                    let (wr, wi) = (self.tw_re[k * step], self.tw_im[k * step]);
                    let (a, b) = (base + k, base + k + half);
                    let (ar, ai, br, bi) = (self.re[a], self.im[a], self.re[b], self.im[b]);
                    let (mut xr, mut xi, mut yr, mut yi) = ([0f32; FFT_LANES], [0f32; FFT_LANES], [0f32; FFT_LANES], [0f32; FFT_LANES]);
                    for l in 0..FFT_LANES {
                        let tr = br[l] * wr - bi[l] * wi;
                        let ti = br[l] * wi + bi[l] * wr;
                        xr[l] = ar[l] + tr;
                        xi[l] = ai[l] + ti;
                        yr[l] = ar[l] - tr;
                        yi[l] = ai[l] - ti;
                    }
                    if last {
                        acc[a] += power(&xr, &xi);
                        acc[b] += power(&yr, &yi);
                    } else {
                        self.re[a] = xr;
                        self.im[a] = xi;
                        self.re[b] = yr;
                        self.im[b] = yi;
                    }
                }
            }
            half *= 2;
        }
    }
}

/// Centred moving average of odd width `w` (edges use the samples available).
pub fn moving_average(input: &[f32], w: usize) -> Vec<f32> {
    let n = input.len();
    let half = w / 2;
    let mut out = Vec::with_capacity(n);
    if n == 0 {
        return out;
    }
    let mut prefix = Vec::with_capacity(n + 1);
    prefix.push(0f64);
    let mut acc = 0f64;
    for &v in input {
        acc += v as f64;
        prefix.push(acc);
    }
    for i in 0..n {
        let lo = i.saturating_sub(half);
        let hi = (i + half + 1).min(n);
        out.push(((prefix[hi] - prefix[lo]) / (hi - lo) as f64) as f32);
    }
    out
}

/// Centred moving sum of odd width `w` over complex samples (edges use the samples available).
pub fn moving_sum_complex(input: &[C32], w: usize) -> Vec<C32> {
    let n = input.len();
    let half = w / 2;
    let mut prefix = Vec::with_capacity(n + 1);
    prefix.push(Complex64::new(0.0, 0.0));
    let mut acc = Complex64::new(0.0, 0.0);
    for v in input {
        acc += Complex64::new(v.re as f64, v.im as f64);
        prefix.push(acc);
    }
    (0..n)
        .map(|i| {
            let lo = i.saturating_sub(half);
            let hi = (i + half + 1).min(n);
            let s = prefix[hi] - prefix[lo];
            C32::new(s.re as f32, s.im as f32)
        })
        .collect()
}

/// Result of [`two_means`]: the two cluster centres, their sizes and the pooled within-cluster
/// standard deviation.
#[derive(Debug, Clone, Copy)]
pub struct TwoMeans {
    pub lo: f32,
    pub hi: f32,
    pub n_lo: usize,
    pub n_hi: usize,
    pub spread: f32,
}

/// One-dimensional two-means (Lloyd) clustering starting from `lo`/`hi`.
pub fn two_means(values: &[f32], mut lo: f32, mut hi: f32) -> TwoMeans {
    let mut n_lo = 0;
    let mut n_hi = 0;
    for _ in 0..12 {
        let thr = (lo + hi) / 2.0;
        let (mut s_lo, mut s_hi) = (0f64, 0f64);
        n_lo = 0;
        n_hi = 0;
        for &v in values {
            if v < thr {
                s_lo += v as f64;
                n_lo += 1;
            } else {
                s_hi += v as f64;
                n_hi += 1;
            }
        }
        let new_lo = if n_lo > 0 { (s_lo / n_lo as f64) as f32 } else { lo };
        let new_hi = if n_hi > 0 { (s_hi / n_hi as f64) as f32 } else { hi };
        let done = (new_lo - lo).abs() < 1e-4 * (1.0 + lo.abs()) && (new_hi - hi).abs() < 1e-4 * (1.0 + hi.abs());
        lo = new_lo;
        hi = new_hi;
        if done {
            break;
        }
    }
    let thr = (lo + hi) / 2.0;
    let mut ss = 0f64;
    for &v in values {
        let c = if v < thr { lo } else { hi };
        ss += ((v - c) as f64).powi(2);
    }
    let spread = if values.is_empty() { 0.0 } else { (ss / values.len() as f64).sqrt() as f32 };
    TwoMeans { lo, hi, n_lo, n_hi, spread }
}

/// The `q`-quantile (0..=1) of `values` (copies; O(n)).
pub fn quantile(values: &[f32], q: f32) -> f32 {
    if values.is_empty() {
        return 0.0;
    }
    let mut v: Vec<f32> = values.to_vec();
    let k = ((v.len() - 1) as f32 * q.clamp(0.0, 1.0)).round() as usize;
    let (_, x, _) = v.select_nth_unstable_by(k, |a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    *x
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tone_gain(taps: &[f32], f_norm: f64) -> f64 {
        let mut acc = Complex64::new(0.0, 0.0);
        for (k, &t) in taps.iter().enumerate() {
            let w = -std::f64::consts::TAU * f_norm * k as f64;
            acc += Complex64::new(w.cos(), w.sin()) * t as f64;
        }
        acc.norm()
    }

    #[test]
    fn lowpass_meets_its_spec() {
        let fs = 500_000.0;
        let taps = lowpass_taps(fs, 100e3, 150e3, 60.0);
        assert_eq!(taps.len() % 2, 1);
        for f in [0.0, 50e3, 100e3] {
            let g = tone_gain(&taps, f / fs);
            assert!((g - 1.0).abs() < 0.01, "passband {f}: {g}");
        }
        for f in [150e3, 200e3, 250e3] {
            let g = 20.0 * tone_gain(&taps, f / fs).log10();
            assert!(g < -55.0, "stopband {f}: {g} dB");
        }
    }

    #[test]
    fn decimator_passes_a_tone_and_rejects_another() {
        let fs = 2_000_000.0;
        let mut dec = FirDecimator::new(lowpass_taps(fs, 120e3, 380e3, 60.0), 4);
        let tone = |f: f64, n: usize| -> Vec<C32> {
            (0..n)
                .map(|k| {
                    let p = std::f64::consts::TAU * f * k as f64 / fs;
                    C32::new(p.cos() as f32 * 100.0, p.sin() as f32 * 100.0)
                })
                .collect()
        };
        let mut out = Vec::new();
        dec.process(&tone(50e3, 4000), &mut out);
        assert_eq!(out.len(), 1000);
        let mag = out[500..].iter().map(|c| c.norm()).sum::<f32>() / 500.0;
        assert!((mag - 100.0).abs() < 1.5, "in-band magnitude {mag}");
        dec.reset();
        out.clear();
        dec.process(&tone(600e3, 4000), &mut out);
        let mag = out[500..].iter().map(|c| c.norm()).sum::<f32>() / 500.0;
        assert!(mag < 0.2, "out-of-band magnitude {mag}");
    }

    #[test]
    fn nco_moves_a_tone_to_dc() {
        let fs = 2_000_000.0;
        let mut buf: Vec<C32> = (0..10_000)
            .map(|k| {
                let p = std::f64::consts::TAU * 250e3 * k as f64 / fs + 0.3;
                C32::new(p.cos() as f32, p.sin() as f32)
            })
            .collect();
        Nco::new(250e3, fs).mix(&mut buf);
        let first = buf[10];
        for s in &buf[5000..5010] {
            assert!((s - first).norm() < 1e-3);
        }
    }

    #[test]
    fn dc_tracker_learns_only_when_allowed() {
        let mut dc = DcTracker::new();
        let mut block = vec![C32::new(10.0, -5.0); 4096];
        dc.process(&mut block, true);
        assert!(block.iter().all(|s| s.norm() < 1e-4));
        // A "burst" (learn = false) must not move the estimate.
        let mut burst = vec![C32::new(70.0, 30.0); 4096];
        dc.process(&mut burst, false);
        assert!((dc.estimate() - C32::new(10.0, -5.0)).norm() < 1e-4);
        assert!((burst[0] - C32::new(60.0, 35.0)).norm() < 1e-4);
    }

    #[test]
    fn iq_format_conversion() {
        let mut out = Vec::new();
        IqFormat::Cs8.convert(&[0x7F, 0x80, 1], &mut out);
        assert_eq!(out, vec![C32::new(127.0, -128.0)]);
        out.clear();
        IqFormat::Cu8.convert(&[255, 0], &mut out);
        assert_eq!(out, vec![C32::new(127.5, -127.5)]);
        out.clear();
        let mut b = Vec::new();
        b.extend_from_slice(&0.5f32.to_le_bytes());
        b.extend_from_slice(&(-0.25f32).to_le_bytes());
        IqFormat::Cf32.convert(&b, &mut out);
        assert_eq!(out, vec![C32::new(64.0, -32.0)]);
        assert_eq!(IqFormat::from_extension("CU8"), Some(IqFormat::Cu8));
        assert_eq!(IqFormat::from_extension("sub"), None);
    }

    #[test]
    fn batch_fft_matches_a_dft() {
        for n in [4usize, 8, 32, 64, 128] {
            let x: Vec<C32> = (0..n * 9)
                .map(|k| C32::new(((k * 37 % 11) as f32) - 5.0, ((k * 53 % 7) as f32) - 3.0))
                .collect();
            let mut fft = BatchFft::new(n);
            let starts: [usize; FFT_LANES] = std::array::from_fn(|l| l * n / 2 + 3);
            let mut acc = vec![0f32; n];
            fft.accumulate_power(&x, starts, &mut acc);
            for (k, &got) in acc.iter().enumerate() {
                let mut p = 0f64;
                for &s0 in &starts {
                    let mut s = Complex64::new(0.0, 0.0);
                    for i in 0..n {
                        let w = 0.5 - 0.5 * (std::f64::consts::TAU * i as f64 / n as f64).cos();
                        let a = -std::f64::consts::TAU * (k * i) as f64 / n as f64;
                        let v = x[s0 + i];
                        s += Complex64::new(v.re as f64, v.im as f64) * Complex64::new(a.cos(), a.sin()) * w;
                    }
                    p += s.norm_sqr();
                }
                assert!((got as f64 - p).abs() < 1e-4 * p.max(1.0), "n {n} bin {k}: {got} vs {p}");
            }
        }
        // A tone at bin 3 of 16 with amplitude 10: power (10·Σw)² per frame, in bin 3 only.
        let n = 16;
        let x: Vec<C32> = (0..n * 8)
            .map(|k| {
                let a = std::f64::consts::TAU * 3.0 * k as f64 / n as f64;
                C32::new(10.0 * a.cos() as f32, 10.0 * a.sin() as f32)
            })
            .collect();
        let mut fft = BatchFft::new(n);
        let mut acc = vec![0f32; n];
        fft.accumulate_power(&x, std::array::from_fn(|l| l * n / 2), &mut acc);
        let expect = 8.0 * (10.0 * fft.coherent_gain()).powi(2);
        assert!((acc[3] - expect).abs() < 1e-3 * expect, "{} vs {expect}", acc[3]);
        assert!(acc[7] < 1e-6 * expect);
    }

    #[test]
    fn two_means_finds_both_clusters() {
        let mut v = vec![1.0f32; 300];
        v.extend(vec![5.0f32; 100]);
        let tm = two_means(&v, 0.0, 10.0);
        assert!((tm.lo - 1.0).abs() < 1e-5 && (tm.hi - 5.0).abs() < 1e-5);
        assert_eq!((tm.n_lo, tm.n_hi), (300, 100));
        assert!(tm.spread < 1e-5);
    }
}
