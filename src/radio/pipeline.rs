//! Shared receive pipeline: raw IQ in, keyfob bursts out.
//!
//! Used by the HackRF and RTL-SDR processing threads and by IQ-file input, so live and offline
//! decoding behave the same. One [`RxPipeline`] turns a stream of IQ samples into [`Burst`]s, each
//! holding the level/duration pairs the protocol decoders expect (µs durations, `level = true` for
//! carrier on (AM) or the upper FSK tone (FM); a burst starts with a long low for the idle period
//! and ends with the trailing gap).
//!
//! A fob is received anywhere in the band of the input, not just near the target frequency: cheap
//! SAW-resonator remotes are often 75–150 kHz off, and a Flipper receives ±325 kHz (OOK650). So
//! detection is wide-band, and the narrow channel filter runs per burst, centred on the carrier
//! that burst was found on:
//!
//! ```text
//! IQ (cs8/cu8/cf32) → DC removal (learned while idle, never from dead input: ≥ 1 ms of identical
//!   samples, an RTL-SDR's zero bytes at start)
//!   [input > 2.5 MS/s: mixer (target → 0 Hz) → FIR ÷D to ≤ 2.5 MS/s]
//!   → wide-band sample buffer (kept from just before the oldest open burst)
//!   → detector: Hann-windowed FFT frames, ~31 kHz bins, 50 % overlap; power per bin over 256 µs
//!     blocks against a noise floor tracked per bin. A bin ≥ 5.4 dB over its floor is "carrier
//!     present". Present bins ≤ ~100 kHz apart are one signal (both FSK tones) unless a separate
//!     power peak stands out between them (a fob next to a strong carrier's skirt); signals further
//!     apart are separate bursts (two fobs at once). A burst closes after 80 ms without carrier in
//!     its bins (noise between bursts can't keep it open, pure noise opens nothing). With offset
//!     tuning the LO / DC bins and the band edges never open a burst. The whole band above its
//!     floor by the same factor is a noise step (the floors are re-learned); a burst still on at
//!     3 s is cut, and goes on from its last frame gap if it carries data (a continuous FSK
//!     carrier), else its bins are re-baselined (a steady carrier, a moved floor).
//!   → per closed burst: carrier = centre of its bins, weighted by how much their power changed
//!     while it was on (the OOK carrier, or between the FSK tones). A burst ≥ 10 dB weaker than
//!     one it is keyed with is that one's artifact (I/Q image, spur) and is dropped, and so is one
//!     that ends up on the carrier of a stronger burst at the same time (a duplicate).
//!   → mixer (carrier → 0 Hz) → FIR ÷D1 → channel FIR ÷2 (±100 kHz pass, ±150 kHz stop,
//!     ~250 kS/s); noise level from the quiet channel samples around the burst (capped by the
//!     burst's own quietest stretches)
//!   → pass 1:
//!       AM: |z| → 20 µs moving average → threshold between the noise level and the carrier level
//!           (hysteresis, interpolated crossing times, 30 µs debounce)
//!       FM: arg(Σ z·z*[-1]) over 20 µs → two-means on the carrier-on samples finds both tones
//!           (per-burst AFC, so the remaining carrier error doesn't matter) → slice at their midpoint;
//!           symbols too short for that (13 µs GFSK), or merged by it (26 µs), are sliced again
//!           without smoothing
//!   → pass 2, per frame (split at lows ≥ 6 ms; TE from all its levels, or from its HIGHs or LOWs
//!     alone where that TE's debounce would erase them): averaging over ~TE/5 of that frame, coherent for
//!     OOK (carrier phase removed first: the frame's mean frequency, or a track of a carrier that
//!     drifts and is pulled at every key-on), on the discriminator for FSK; TE/4 debounce
//!   → each path is emitted only if it carries modulation (FSK bursts don't also come out as an
//!     AM "carrier on" twin, OOK bursts don't come out as FM noise)
//!   → if the slicer finds the carrier (or the middle of the tones) > 20 kHz from the channel
//!     centre, the burst is filtered and sliced again around it
//! ```
//!
//! The channel (±100 kHz around the burst's own carrier) holds ±47.6 kHz FSK deviation plus carrier
//! drift within a burst; pass 2 more than recovers the sensitivity a narrower filter would give.
//! [`Burst::offset_hz`] is the carrier's offset from the target. See `docs/radio.md` for the
//! measurements (synthetic IQ, `src/radio/sim.rs`, and real recordings).

use std::collections::VecDeque;
use std::ops::Range;

use num_complex::Complex64;

use super::demodulator::LevelDuration;
use super::dsp::{
    lowpass_taps, moving_average, moving_sum_complex, quantile, two_means, BatchFft, DcTracker, FirDecimator,
    IqFormat, Nco, C32, FFT_LANES,
};

/// Output (channel) sample rate the decimation chain aims for.
pub const TARGET_OUTPUT_RATE: f64 = 250_000.0;
/// Channel filter pass band (each side of the burst's carrier): FSK deviation + drift.
pub const CHANNEL_PASS_HZ: f64 = 100_000.0;
/// First decimation stage pass band (a bit wider than the channel).
const STAGE1_PASS_HZ: f64 = 120_000.0;
/// Stop-band attenuation of the filters.
const FILTER_ATTEN_DB: f64 = 60.0;
/// Highest rate the detector runs at; faster input is decimated around the target first.
const MAX_WIDE_RATE: f64 = 2_500_000.0;
/// Pass band of that front decimation (fraction of its output rate; the band searched: ±875 kHz
/// around the target at 2.5 MS/s).
const FRONT_PASS: f64 = 0.35;
/// Detector bin width aimed for (the FFT length is the nearest power of two).
const BIN_HZ: f64 = 31_250.0;
/// FFT frames per detector block; they hop by half a frame, so a block is 8 FFT lengths (~256 µs).
const FRAMES: usize = 2 * FFT_LANES;
/// Block power over the noise floor that makes a bin "carrier present" (5.4 dB; averaged over 16
/// frames, noise alone gets there in ~1e-8 of the bin-blocks).
const DETECT_RATIO: f32 = 3.5;
/// Present bins at most this far apart (rounded up to whole bins) belong to one signal: both tones
/// of ±47.6 kHz FSK. Signals further apart are separate bursts.
const JOIN_HZ: f64 = 96_000.0;
/// A burst's carrier is the power-weighted centre of the bins this close to its strongest bin.
const CENTROID_HZ: f64 = 125_000.0;
/// With offset tuning, the detector band ends here (fraction of the rate; filter roll-off and
/// aliases beyond).
const EDGE_FRACTION: f64 = 0.45;
/// A burst at least 10 dB weaker than another that is keyed with it (90 % of its carrier-present
/// blocks are the other's, and the other is keyed over its span, or their block levels go up and
/// down together, or they start and end together) is an artifact of that burst: an I/Q-imbalance
/// image, a spur or an intermodulation product of a strong fob. It is dropped. A weaker fob keyed
/// while a strong carrier is simply on is not.
const ARTIFACT_COINCIDENCE: f32 = 0.9;
const ARTIFACT_RATIO: f32 = 10.0;
const ARTIFACT_CORRELATION: f32 = 0.5;
/// One transmitter is one burst: a burst whose carrier ends up this close to that of a stronger
/// burst overlapping it in time is a duplicate of it (a weak spectral line of the same transmitter,
/// filtered again around the main carrier) and is dropped.
const DUPLICATE_HZ: f64 = 50_000.0;
/// A power peak in a cluster of present bins is a signal of its own if it is more than the join
/// distance from every stronger peak and this much (6 dB) over the lowest bin between them: a fob
/// next to the skirt of a strong carrier.
const PEAK_PROMINENCE: f32 = 4.0;
/// A burst whose slicer measures its carrier this far from the centre of its channel is filtered
/// again around the measured carrier.
const REFINE_HZ: f32 = 20_000.0;
/// Detector blocks of quiet samples filtered before / after a burst to measure its channel noise.
const PRE_QUIET_BLOCKS: u64 = 6;
const TAIL_QUIET_BLOCKS: u64 = 16;
/// Channel samples per noise-measurement block (and quiet blocks needed for a measurement).
const NOISE_BLOCK: usize = 64;
const MIN_QUIET_BLOCKS: usize = 6;
/// 10 % quantile of the mean power of `NOISE_BLOCK` noise samples, relative to the mean.
const NOISE_Q10: f32 = 0.845;
/// The slicer's thresholds were tuned with the old channel squelch's floor tracker (fast down, slow
/// up), which settles ~8 % (0.36 dB) below the mean noise power; the measured level is scaled the
/// same way (0.3 dB of OOK sensitivity at the decode limit, 0.2 dB of FSK).
const NOISE_CALIBRATION: f32 = 0.92;
/// Noise floor per bin: median of the first blocks, then tracked while no burst is open (fast
/// down after a large drop, e.g. a gain change).
const FLOOR_INIT_BLOCKS: usize = 8;
/// In a file recorded at the fob frequency, a bin learned more than this far above the median bin
/// (10 dB) is taken for a transmission that was already on when the recording started, not for
/// background: its floor is learned at this level (a steady carrier is re-baselined after
/// MAX_BURST_US like any stuck burst). Not with offset tuning: live, a strong receiver spur would
/// hold the hopper for MAX_BURST_US on every new frequency.
const FLOOR_INIT_CAP: f32 = 10.0;
const FLOOR_ALPHA: f32 = 0.02;
const FLOOR_ALPHA_FAST: f32 = 0.1;
/// The whole band this many blocks (2 ms) above its floor, flat within a factor 8 (9 dB, noise
/// estimates of 64 bins spread ±4 dB), is a noise step (see `end_block`).
const STEP_BLOCKS: u32 = 8;
const STEP_FLATNESS: f32 = 8.0;
/// A burst ends after this much time without carrier (keeps one multi-frame button press together).
pub const GAP_US: u64 = 80_000;
/// Longest burst before it is cut and emitted (a held button / continuous carrier).
pub const MAX_BURST_US: u64 = 3_000_000;
/// A burst cut at MAX_BURST_US that carries data (FSK tones or OOK keying, at least this SNR) is a
/// transmission: it continues as a new burst from its last frame gap, at most this far back (and
/// this far back when it has none), so the frame across the cut is whole in the next one. Only an
/// unmodulated one (a steady carrier, a moved floor) has its bins re-baselined.
const CONTINUE_SNR_DB: f32 = 10.0;
const MAX_OVERLAP_US: u64 = 500_000;
/// Input that repeats the same sample this long (1 ms, at least 64 samples) is dead: an RTL-SDR's
/// zero bytes before its first real buffer (rtl_433's recordings start with up to 260 ms of them),
/// a stalled stream. A live receiver always has noise, so it never does that; the DC estimate and
/// the detector's noise floor are not learned from it.
const DEAD_US: f64 = 1_000.0;
const DEAD_MIN_SAMPLES: u64 = 64;
/// Post-detection moving average.
const SMOOTH_US: f64 = 20.0;
/// Levels shorter than this are absorbed into their neighbours (the shortest protocol TE is 50 µs).
const DEBOUNCE_US: f64 = 30.0;
/// A burst needs this many carrier-present blocks (~1 ms) to be considered at all.
const MIN_PRESENT_BLOCKS: u32 = 4;
/// Captures with fewer pairs are dropped (same rule as KAT 1.4).
const MIN_PAIRS: usize = 5;
/// Pass 2 treats lows at least this long as frame boundaries.
const FRAME_GAP_US: f64 = 6_000.0;
/// Leading idle low emitted before the first edge is clamped to this range (µs).
const LEAD_MIN_US: u64 = 10_000;
const LEAD_MAX_US: u64 = 1_000_000;
/// Minimum FSK tone separation and bimodality (separation / within-tone spread).
const FSK_MIN_SEPARATION_HZ: f32 = 4_000.0;
const FSK_MIN_BIMODALITY: f32 = 3.5;
/// Mean |z| of complex Gaussian noise relative to sqrt(mean |z|²) (Rayleigh mean, √π/2).
const RAYLEIGH_MEAN: f32 = 0.886_226_9;

/// One received burst (one button press, typically) with the demodulated pairs for each path.
#[derive(Debug, Clone)]
pub struct Burst {
    /// Target frequency the burst was heard on (Hz).
    pub frequency: u32,
    /// AM/OOK pairs, if the burst carries amplitude modulation.
    pub am: Option<Vec<LevelDuration>>,
    /// FM/2FSK pairs, if the burst carries two distinct tones.
    pub fm: Option<Vec<LevelDuration>>,
    /// Input sample index where the burst starts (for files: position in the recording).
    pub start_sample: u64,
    /// Length from the first to the last carrier-present block (µs).
    pub duration_us: u64,
    /// Carrier level over the noise floor (dB, in the channel).
    pub snr_db: f32,
    /// Measured carrier offset from the target (Hz): FSK centre, or OOK carrier.
    pub offset_hz: f32,
    /// Measured FSK deviation (half the tone separation), when the burst is FSK.
    pub deviation_hz: Option<f32>,
}

/// One decimation stage: taps + factor.
#[derive(Debug, Clone, Copy)]
struct StageSpec {
    decim: usize,
    pass: f64,
    stop: f64,
}

/// Channel decimation plan for an input rate: two stages for even factors ≥ 4 (cheap first stage,
/// sharp channel stage at ~500 kS/s), otherwise one.
fn plan_stages(fs_in: f64) -> (Vec<StageSpec>, f64) {
    let d = ((fs_in / TARGET_OUTPUT_RATE).round() as usize).max(1);
    let fs_out = fs_in / d as f64;
    let channel_stop = |fs_out: f64, fs: f64| (fs_out - CHANNEL_PASS_HZ).min(0.5 * fs).max(CHANNEL_PASS_HZ * 1.2);
    if d >= 4 && d & 1 == 0 {
        let d1 = d / 2;
        let fs_mid = fs_in / d1 as f64;
        (
            vec![
                StageSpec { decim: d1, pass: STAGE1_PASS_HZ, stop: fs_mid - STAGE1_PASS_HZ },
                StageSpec { decim: 2, pass: CHANNEL_PASS_HZ, stop: channel_stop(fs_out, fs_mid) },
            ],
            fs_out,
        )
    } else {
        let pass = CHANNEL_PASS_HZ.min(0.4 * fs_in);
        let stop = if d == 1 { (0.5 * fs_in).max(pass * 1.2) } else { channel_stop(fs_out, fs_in) };
        (vec![StageSpec { decim: d, pass, stop }], fs_out)
    }
}

/// Detector bins as a bit set: bit `i` is bin `i` in frequency order (bin `n/2` is 0 Hz of the
/// wide band, bin 0 is −fs/2).
#[derive(Debug, Clone, Copy)]
struct BinSpace {
    n: usize,
    /// The band wraps around (a file's whole complex band, nothing cut off at the edges).
    circular: bool,
}

impl BinSpace {
    fn all(self) -> u128 {
        if self.n >= 128 {
            u128::MAX
        } else {
            (1u128 << self.n) - 1
        }
    }

    fn has(m: u128, i: usize) -> bool {
        (m >> i) & 1 == 1
    }

    /// Bins within `d` of a bin in `m`.
    fn dilate(self, m: u128, d: usize) -> u128 {
        let n = self.n;
        let mut out = m;
        for s in 1..=d.min(n - 1) {
            out |= (m << s) | (m >> s);
            if self.circular {
                out |= (m >> (n - s)) | (m << (n - s));
            }
        }
        out & self.all()
    }

    /// Present bins grouped into signals: bins at most `join` apart (around the circle if circular).
    fn clusters(self, present: u128, join: usize) -> Vec<u128> {
        let bins: Vec<usize> = (0..self.n).filter(|&i| Self::has(present, i)).collect();
        let mut groups: Vec<u128> = Vec::new();
        for (k, &i) in bins.iter().enumerate() {
            match groups.last_mut() {
                Some(g) if i - bins[k - 1] <= join => *g |= 1 << i,
                _ => groups.push(1 << i),
            }
        }
        if self.circular && groups.len() > 1 && bins[0] + self.n - bins[bins.len() - 1] <= join {
            let last = groups.pop().expect("two groups");
            groups[0] |= last;
        }
        groups
    }

    /// Split a cluster into signals at its prominent power peaks (see [`PEAK_PROMINENCE`]); every
    /// bin goes to the nearest peak. Returns (bins, peak bin) per signal, strongest first.
    fn split(self, c: u128, power: &[f32], join: usize) -> Vec<(u128, usize)> {
        let mut bins: Vec<usize> = (0..self.n).filter(|&i| Self::has(c, i)).collect();
        if bins.is_empty() {
            return Vec::new();
        }
        // Positions along the cluster (a cluster that wraps around starts after its widest gap).
        let mut pos: Vec<usize> = bins.clone();
        if self.circular && bins.len() < self.n {
            let gaps = (0..bins.len()).map(|k| (bins[(k + 1) % bins.len()] + self.n - bins[k]) % self.n);
            let (widest, _) = gaps.enumerate().max_by_key(|&(_, g)| g).expect("non-empty");
            let len = bins.len();
            bins.rotate_left((widest + 1) % len);
            pos = bins.clone();
            for k in 1..pos.len() {
                while pos[k] < pos[k - 1] {
                    pos[k] += self.n;
                }
            }
        } else if bins.len() == self.n {
            let peak = (0..self.n).max_by(|&a, &b| power[a].total_cmp(&power[b])).expect("bins");
            return vec![(c, peak)];
        }
        let m = bins.len();
        let mut maxima: Vec<usize> = (0..m)
            .filter(|&k| (k == 0 || power[bins[k]] >= power[bins[k - 1]]) && (k + 1 == m || power[bins[k]] >= power[bins[k + 1]]))
            .collect();
        maxima.sort_by(|&a, &b| power[bins[b]].total_cmp(&power[bins[a]]));
        let mut peaks: Vec<usize> = Vec::new();
        for k in maxima {
            let Some(&a) = peaks.iter().min_by_key(|&&a| pos[a].abs_diff(pos[k])) else {
                peaks.push(k);
                continue;
            };
            if pos[a].abs_diff(pos[k]) <= join {
                continue;
            }
            let valley = (a.min(k) + 1..a.max(k)).map(|j| power[bins[j]]).fold(f32::INFINITY, f32::min);
            if power[bins[k]] >= PEAK_PROMINENCE * valley {
                peaks.push(k);
            }
        }
        let mut out: Vec<(u128, usize)> = peaks.iter().map(|&k| (0u128, bins[k])).collect();
        for j in 0..m {
            let (owner, _) = peaks
                .iter()
                .enumerate()
                .min_by_key(|&(r, &k)| (pos[k].abs_diff(pos[j]), r))
                .expect("a peak");
            out[owner].0 |= 1 << bins[j];
        }
        out
    }
}

/// Wide-band detector state (see the module docs).
#[derive(Debug, Clone)]
struct Detector {
    space: BinSpace,
    bin_hz: f64,
    /// Samples per block (8 FFT lengths).
    block: usize,
    fft: BatchFft,
    /// Bins that can open or extend a burst.
    allowed: u128,
    /// Join distance and carrier-centroid half width (bins).
    join: usize,
    centroid: usize,
    /// Noise floor per bin (mean frame power, frequency order); empty until learned.
    floor: Vec<f32>,
    init: Vec<Vec<f32>>,
    /// This block's mean frame power per bin (frequency order).
    power: Vec<f32>,
    acc: Vec<f32>,
    /// (Σw)²: a carrier of amplitude A at a bin centre has power A²·cw_gain.
    cw_gain: f32,
    /// Σw²: white noise of variance σ² per sample has power σ²·noise_gain in every bin.
    noise_gain: f32,
}

impl Detector {
    fn new(fs_w: f64, wide_decim: usize, offset_hz: f64) -> Self {
        let n = (2f64.powf((fs_w / BIN_HZ).log2().round()) as usize).clamp(8, 128);
        let bin_hz = fs_w / n as f64;
        let space = BinSpace { n, circular: wide_decim == 1 && offset_hz == 0.0 };
        let fft = BatchFft::new(n);
        let cw_gain = fft.coherent_gain().powi(2);
        let noise_gain = fft.noise_gain();
        let mut det = Self {
            space,
            bin_hz,
            block: FRAMES / 2 * n,
            fft,
            allowed: 0,
            join: ((JOIN_HZ / bin_hz).ceil() as usize).max(2),
            centroid: ((CENTROID_HZ / bin_hz).round() as usize).clamp(1, (n - 1) / 2),
            floor: Vec::new(),
            init: Vec::new(),
            power: vec![0.0; n],
            acc: vec![0.0; n],
            cw_gain,
            noise_gain,
        };
        det.set_band(fs_w, wide_decim, offset_hz);
        det
    }

    /// Bins that may open bursts. A file recorded at the fob frequency (no offset tuning) uses its
    /// whole band, around the circle. With offset tuning (live RX and its recordings) the bins at
    /// the LO (DC spike, 1/f noise) and beyond ±0.45·fs (filter roll-off, aliases) are left out;
    /// after a front decimation, everything outside its pass band.
    fn set_band(&mut self, fs_w: f64, wide_decim: usize, offset_hz: f64) {
        let tuned = offset_hz != 0.0;
        self.space.circular = wide_decim == 1 && !tuned;
        let edge = match (self.space.circular, wide_decim) {
            (true, _) => f64::INFINITY,
            (false, 1) => EDGE_FRACTION * fs_w,
            _ => FRONT_PASS * fs_w,
        };
        let dc = if wide_decim > 1 { -offset_hz } else { 0.0 };
        self.allowed = 0;
        for i in 0..self.space.n {
            let f = self.freq(i as f64);
            let at_dc = tuned && (f - dc).abs() <= 1.01 * self.bin_hz;
            if f.abs() <= edge + 1.0 && !at_dc {
                self.allowed |= 1 << i;
            }
        }
    }

    /// Frequency (Hz, wide band) of a (fractional) bin position.
    fn freq(&self, pos: f64) -> f64 {
        (pos - (self.space.n / 2) as f64) * self.bin_hz
    }

    /// Fraction of each bin's noise that falls into a channel centred on `carrier_hz` (half width
    /// = the channel filter's −6 dB point), times 1/noise_gain: Σ floor·weight = channel noise power.
    fn channel_noise(&self, carrier_hz: f64, fs_w: f64, half_width: f64) -> f32 {
        let n = self.space.n;
        let mut s = 0f64;
        for (i, &fl) in self.floor.iter().enumerate() {
            let d = wrap_hz(self.freq(i as f64) - carrier_hz, fs_w).abs();
            let frac = ((half_width - d) / self.bin_hz + 0.5).clamp(0.0, 1.0);
            s += frac * fl as f64 / self.noise_gain as f64;
        }
        (s / n as f64) as f32
    }
}

/// `f` folded into [−fs/2, fs/2).
fn wrap_hz(f: f64, fs: f64) -> f64 {
    (f + fs / 2.0).rem_euclid(fs) - fs / 2.0
}

/// A burst being collected (positions are wide-band sample indices).
#[derive(Debug, Clone)]
struct OpenBurst {
    /// First sample of the first carrier-present block.
    first_present: u64,
    /// One past the last sample of the last carrier-present block.
    last_present_end: u64,
    present_blocks: u32,
    /// Carrier present in this block (bookkeeping within `end_block`).
    present_now: bool,
    /// Bins the burst's signal was present in.
    seen: u128,
    /// Its power peaks (FSK: both tones): a signal peaking near one of them belongs to this burst.
    core: u128,
    /// Σ (power − floor) per bin over the present blocks (where the carrier is, see `carrier_hz`).
    excess: Vec<f32>,
    /// Lowest block power per bin while open (a bin that never drops is a floor change).
    min_power: Vec<f32>,
    /// Per block from block number `first_block` on: level of its strongest bin over the floor
    /// (dB) when carrier present, NaN when not (artifact test).
    first_block: u64,
    history: Vec<f32>,
    /// The rest of a burst cut at MAX_BURST_US: what came before it is not quiet.
    continued: bool,
    /// Cut at MAX_BURST_US while still on: what comes after it is not quiet.
    cut: bool,
}

impl OpenBurst {
    fn new(start: u64, block: u64, n: usize) -> Self {
        Self {
            first_present: start,
            last_present_end: start,
            present_blocks: 0,
            present_now: false,
            seen: 0,
            core: 0,
            excess: vec![0.0; n],
            min_power: vec![f32::INFINITY; n],
            first_block: block,
            history: Vec::new(),
            continued: false,
            cut: false,
        }
    }

    /// Record block number `k` as carrier present at `level_db` (the strongest of the calls).
    fn mark(&mut self, k: u64, level_db: f32) {
        let i = (k - self.first_block) as usize;
        if self.history.len() <= i {
            self.history.resize(i + 1, f32::NAN);
        }
        let h = &mut self.history[i];
        *h = if h.is_nan() { level_db } else { h.max(level_db) };
    }

    /// Mean excess power per present block in the burst's strongest bin (artifact test, closing
    /// order).
    fn strength(&self, allowed: u128) -> f32 {
        let peak = (0..self.excess.len())
            .filter(|&i| BinSpace::has(self.seen & allowed, i))
            .map(|i| self.excess[i].abs())
            .fold(0f32, f32::max);
        peak / self.present_blocks.max(1) as f32
    }

    /// Merge `o` (the same signal, seen as two until now) into this one.
    fn absorb(&mut self, o: OpenBurst) {
        self.first_present = self.first_present.min(o.first_present);
        self.last_present_end = self.last_present_end.max(o.last_present_end);
        self.present_now |= o.present_now;
        self.continued |= o.continued;
        self.seen |= o.seen;
        self.core |= o.core;
        for (a, b) in self.excess.iter_mut().zip(&o.excess) {
            *a += b;
        }
        for (a, b) in self.min_power.iter_mut().zip(&o.min_power) {
            *a = a.min(*b);
        }
        let first = self.first_block.min(o.first_block);
        let mut history = vec![f32::NAN; (self.first_block - first) as usize];
        history.extend(&self.history);
        let shift = (o.first_block - first) as usize;
        if history.len() < shift + o.history.len() {
            history.resize(shift + o.history.len(), f32::NAN);
        }
        for (i, &l) in o.history.iter().enumerate() {
            let h = &mut history[shift + i];
            if !l.is_nan() {
                *h = if h.is_nan() { l } else { h.max(l) };
            }
        }
        self.first_block = first;
        self.present_blocks = history.iter().filter(|l| !l.is_nan()).count() as u32;
        self.history = history;
    }
}

impl OpenBurst {
    /// The rest of this burst from wide sample `resume` on (detector blocks of `blk` samples, the
    /// first starting at `half_n`), as a new burst: a transmission cut at MAX_BURST_US goes on.
    fn continue_from(&self, resume: u64, blk: u64, half_n: u64) -> OpenBurst {
        let block = (resume.saturating_sub(half_n) / blk).max(self.first_block);
        let skip = ((block - self.first_block) as usize).min(self.history.len());
        let history = self.history[skip..].to_vec();
        let present = history.iter().filter(|l| !l.is_nan()).count() as u32;
        let scale = present as f32 / self.present_blocks.max(1) as f32;
        OpenBurst {
            first_present: (half_n + block * blk).max(self.first_present),
            last_present_end: self.last_present_end,
            present_blocks: present,
            present_now: false,
            seen: self.seen,
            core: self.core,
            excess: self.excess.iter().map(|e| e * scale).collect(),
            min_power: self.min_power.clone(),
            first_block: block,
            history,
            continued: true,
            cut: false,
        }
    }
}

/// A burst that closed recently (for the artifact test of bursts closing after it).
#[derive(Debug, Clone)]
struct ClosedBurst {
    first_present: u64,
    last_present_end: u64,
    strength: f32,
    /// Carrier (Hz, wide band): where it was finally sliced, or the estimate.
    carrier_hz: f64,
    first_block: u64,
    history: Vec<f32>,
    /// Cut at MAX_BURST_US and continued by an open burst (not a duplicate of it).
    continued: bool,
}

/// What closing a burst produced (for a burst cut at MAX_BURST_US).
struct Closed {
    /// It carried data: a transmission, not a moved floor.
    modulated: bool,
    /// Wide-sample position of the middle of its last frame gap.
    last_gap: Option<u64>,
}

/// True if burst `b` (block levels `b_hist` from block number `b0`, NaN = absent) is keyed with
/// burst `a` (from `a0`): present (almost) only when `a` is, and `a` keyed over `b`'s span, or their
/// levels correlated, or `a` on all through it and starting and ending with it (a steady FSK fob
/// and its spur).
fn keyed_together(b0: u64, b_hist: &[f32], a0: u64, a_hist: &[f32]) -> bool {
    let a_at = |k: u64| if k >= a0 { a_hist.get((k - a0) as usize).copied().unwrap_or(f32::NAN) } else { f32::NAN };
    let b_blocks: Vec<u64> = (0..b_hist.len()).filter(|&i| !b_hist[i].is_nan()).map(|i| b0 + i as u64).collect();
    let (Some(&first), Some(&last)) = (b_blocks.first(), b_blocks.last()) else {
        return false;
    };
    let both: Vec<(f32, f32)> = b_blocks
        .iter()
        .filter(|&&k| !a_at(k).is_nan())
        .map(|&k| (b_hist[(k - b0) as usize], a_at(k)))
        .collect();
    if (both.len() as f32) < ARTIFACT_COINCIDENCE * b_blocks.len() as f32 {
        return false;
    }
    let span = last - first + 1;
    let a_in_span = (first..=last).filter(|&k| !a_at(k).is_nan()).count();
    if (a_in_span as f32) < ARTIFACT_COINCIDENCE * span as f32 {
        return true;
    }
    if both.len() >= 8 && correlation(&both) >= ARTIFACT_CORRELATION {
        return true;
    }
    let a_first = a0 + a_hist.iter().position(|l| !l.is_nan()).unwrap_or(0) as u64;
    let a_last = a0 + a_hist.iter().rposition(|l| !l.is_nan()).unwrap_or(0) as u64;
    let tolerance = (span / 50).max(8);
    a_first.abs_diff(first) <= tolerance && a_last.abs_diff(last) <= tolerance
}

/// Pearson correlation of paired values (0 if either side is constant).
fn correlation(pairs: &[(f32, f32)]) -> f32 {
    let n = pairs.len() as f64;
    let (mx, my) = pairs.iter().fold((0f64, 0f64), |(x, y), &(a, b)| (x + a as f64 / n, y + b as f64 / n));
    let (mut sxy, mut sxx, mut syy) = (0f64, 0f64, 0f64);
    for &(a, b) in pairs {
        let (dx, dy) = (a as f64 - mx, b as f64 - my);
        sxy += dx * dy;
        sxx += dx * dx;
        syy += dy * dy;
    }
    if sxx <= 0.0 || syy <= 0.0 {
        0.0
    } else {
        (sxy / (sxx * syy).sqrt()) as f32
    }
}

/// Shared receive pipeline (see the module docs).
pub struct RxPipeline {
    fs_in: f64,
    /// Detector / burst-buffer rate: the input rate up to 2.5 MS/s, else decimated to it.
    fs_w: f64,
    wide_decim: usize,
    fs_out: f64,
    offset_hz: f64,
    frequency: u32,
    /// Front decimation for fast input: mixer (target → 0 Hz) and FIR.
    front: Option<(Nco, FirDecimator)>,
    dc: DcTracker,
    carry: Vec<u8>,
    conv: Vec<C32>,
    tmp: Vec<C32>,
    blank_in: u64,
    /// The last input sample and how many times in a row it has come (dead-input test).
    dead_last: Option<C32>,
    dead_run: u64,
    dead_min: u64,
    /// Wide-band sample ranges of dead input (pruned as the detector moves on).
    dead: VecDeque<(u64, u64)>,
    /// Wide-band samples `[buf_start, buf_start + buf.len())`.
    buf: Vec<C32>,
    buf_start: u64,
    /// First sample of the next detector block.
    next_block: u64,
    det: Detector,
    bursts: Vec<OpenBurst>,
    recent: VecDeque<ClosedBurst>,
    floor_cache: Vec<(u32, Vec<f32>)>,
    /// Consecutive blocks in which the whole band stood above its floor by about the same factor
    /// (a noise step), and the first of them.
    step_blocks: u32,
    step_start: u64,
    /// Channel filter stages (taps, decimation) from the wide rate to ~250 kS/s.
    chan: Vec<(Vec<f32>, usize)>,
    /// Wide samples the channel filter needs before its first valid output.
    chan_len_w: u64,
    last_burst_end: u64,
    rssi_peak: f32,
    out: Vec<Burst>,
}

impl RxPipeline {
    /// Pipeline for input at `sample_rate` S/s where the target (`frequency`, used to tag bursts)
    /// sits `offset_hz` above the centre of the samples (0 for files recorded at the fob frequency;
    /// +fs/8 for live RX with offset tuning).
    pub fn new(sample_rate: u32, offset_hz: f64, frequency: u32) -> Self {
        let fs_in = sample_rate.max(1) as f64;
        let wide_decim = ((fs_in / MAX_WIDE_RATE).ceil() as usize).max(1);
        let fs_w = fs_in / wide_decim as f64;
        let front = (wide_decim > 1).then(|| {
            let taps = lowpass_taps(fs_in, FRONT_PASS * fs_w, (1.0 - FRONT_PASS) * fs_w, FILTER_ATTEN_DB);
            (Nco::new(offset_hz, fs_in), FirDecimator::new(taps, wide_decim))
        });
        let (specs, fs_out) = plan_stages(fs_w);
        let mut fs = fs_w;
        let mut chan = Vec::new();
        for s in &specs {
            chan.push((lowpass_taps(fs, s.pass, s.stop, FILTER_ATTEN_DB), s.decim));
            fs /= s.decim as f64;
        }
        let mut chan_len_w = 1u64;
        let mut step = 1u64;
        for (taps, d) in &chan {
            chan_len_w += (taps.len() as u64 - 1) * step;
            step *= *d as u64;
        }
        let det = Detector::new(fs_w, wide_decim, offset_hz);
        let next_block = (det.space.n / 2) as u64;
        Self {
            fs_in,
            fs_w,
            wide_decim,
            fs_out,
            offset_hz,
            frequency,
            front,
            dc: DcTracker::new(),
            carry: Vec::new(),
            conv: Vec::with_capacity(4096),
            tmp: Vec::with_capacity(4096),
            blank_in: 0,
            dead_last: None,
            dead_run: 0,
            dead_min: ((DEAD_US * fs_in / 1e6) as u64).max(DEAD_MIN_SAMPLES),
            dead: VecDeque::new(),
            buf: Vec::new(),
            buf_start: 0,
            next_block,
            det,
            bursts: Vec::new(),
            recent: VecDeque::new(),
            floor_cache: Vec::new(),
            step_blocks: 0,
            step_start: 0,
            chan,
            chan_len_w,
            last_burst_end: 0,
            rssi_peak: 0.0,
            out: Vec::new(),
        }
    }

    /// Input sample rate (S/s).
    #[allow(dead_code)]
    pub fn input_rate(&self) -> f64 {
        self.fs_in
    }

    /// Detector rate (S/s): the input rate, or ≤ 2.5 MS/s after the front decimation.
    #[allow(dead_code)]
    pub fn wide_rate(&self) -> f64 {
        self.fs_w
    }

    /// Channel (output) sample rate after decimation (S/s).
    #[allow(dead_code)]
    pub fn output_rate(&self) -> f64 {
        self.fs_out
    }

    /// Target frequency bursts are tagged with.
    pub fn frequency(&self) -> u32 {
        self.frequency
    }

    /// Offset of the target from the centre of the input samples (Hz).
    #[allow(dead_code)]
    pub fn offset_hz(&self) -> f64 {
        self.offset_hz
    }

    /// Range of carrier offsets from the target (Hz) where a burst can be found (edges of the
    /// detector band; with offset tuning there is also a blind spot of ±1 bin at the LO).
    #[allow(dead_code)]
    pub fn search_range_hz(&self) -> (f64, f64) {
        let d = &self.det;
        let bins: Vec<f64> =
            (0..d.space.n).filter(|&i| BinSpace::has(d.allowed, i)).map(|i| d.freq(i as f64)).collect();
        let lo = bins.iter().copied().fold(f64::INFINITY, f64::min) - d.bin_hz / 2.0;
        let hi = bins.iter().copied().fold(f64::NEG_INFINITY, f64::max) + d.bin_hz / 2.0;
        (lo - self.target_w(), hi - self.target_w())
    }

    /// Filter taps: channel stages (and the front decimator, last, if any). For diagnostics.
    #[allow(dead_code)]
    pub fn filter_taps(&self) -> Vec<usize> {
        let mut v: Vec<usize> = self.chan.iter().map(|(t, _)| t.len()).collect();
        if let Some((_, f)) = &self.front {
            v.push(f.len());
        }
        v
    }

    /// True while a burst is open (carrier seen within the last [`GAP_US`]): the hopper holds.
    pub fn busy(&self) -> bool {
        !self.bursts.is_empty()
    }

    /// Median noise floor of the detector bins as a per-sample power (10·log10 of σ² / 128²), or
    /// None until learned.
    #[allow(dead_code)]
    pub fn noise_floor_db(&self) -> Option<f32> {
        if self.det.floor.is_empty() {
            return None;
        }
        let m = quantile(&self.det.floor, 0.5) / self.det.noise_gain;
        Some(10.0 * (m.max(1e-12) / (128.0 * 128.0)).log10())
    }

    /// Peak carrier magnitude (strongest detector bin) since the last call, normalised to full
    /// scale (0..~1).
    pub fn take_rssi(&mut self) -> f32 {
        let r = self.rssi_peak.sqrt() / 128.0;
        self.rssi_peak = 0.0;
        r
    }

    /// Forget the noise floor (after a gain change) so it is re-learned within a few ms.
    pub fn reset_floor(&mut self) {
        self.det.floor.clear();
        self.det.init.clear();
        self.floor_cache.clear();
        self.step_blocks = 0;
    }

    /// Feed raw IQ bytes. Completed bursts are returned (usually none).
    pub fn process(&mut self, bytes: &[u8], format: IqFormat) -> Vec<Burst> {
        let bps = format.bytes_per_sample();
        let mut data = bytes;
        // Complete a sample split across calls.
        if !self.carry.is_empty() {
            let need = bps - self.carry.len();
            let take = need.min(data.len());
            self.carry.extend_from_slice(&data[..take]);
            data = &data[take..];
            if self.carry.len() == bps {
                let carry = std::mem::take(&mut self.carry);
                self.process_whole(&carry, format);
            }
        }
        let whole = data.len() / bps * bps;
        self.process_whole(&data[..whole], format);
        self.carry.extend_from_slice(&data[whole..]);
        std::mem::take(&mut self.out)
    }

    /// End of input (file): close the open bursts now.
    pub fn flush(&mut self) -> Vec<Burst> {
        let end = self.buf_end();
        // Strongest first, so a weaker artifact closing with it can be recognised.
        let allowed = self.det.allowed;
        self.bursts.sort_by(|a, b| b.strength(allowed).total_cmp(&a.strength(allowed)));
        while !self.bursts.is_empty() {
            let b = self.bursts.remove(0);
            self.close(b, end);
        }
        std::mem::take(&mut self.out)
    }

    /// Switch to another target frequency: open bursts are closed (and returned), the noise floor
    /// of the new frequency is restored if known, and the next `blank_input` samples are dropped
    /// (PLL settling / samples still from the old frequency).
    pub fn retune(&mut self, frequency: u32, offset_hz: f64, blank_input: u64) -> Vec<Burst> {
        let mut out = self.flush();
        if !self.det.floor.is_empty() {
            let floor = self.det.floor.clone();
            match self.floor_cache.iter_mut().find(|(f, _)| *f == self.frequency) {
                Some(e) => e.1 = floor,
                None => self.floor_cache.push((self.frequency, floor)),
            }
        }
        self.frequency = frequency;
        self.det.floor =
            self.floor_cache.iter().find(|(f, _)| *f == frequency).map(|e| e.1.clone()).unwrap_or_default();
        self.det.init.clear();
        self.step_blocks = 0;
        if offset_hz != self.offset_hz {
            self.offset_hz = offset_hz;
            self.det.set_band(self.fs_w, self.wide_decim, offset_hz);
            if let Some((nco, _)) = self.front.as_mut() {
                *nco = Nco::new(offset_hz, self.fs_in);
            }
        }
        self.dc.reset();
        if let Some((_, fir)) = self.front.as_mut() {
            fir.reset();
        }
        // The buffered samples are from the old frequency.
        let end = self.buf_end();
        self.buf.clear();
        self.buf_start = end;
        self.next_block = end + (self.det.space.n / 2) as u64;
        self.recent.clear();
        self.blank_in = blank_input;
        out.append(&mut self.out);
        out
    }

    fn buf_end(&self) -> u64 {
        self.buf_start + self.buf.len() as u64
    }

    /// Target position in the wide band (Hz).
    fn target_w(&self) -> f64 {
        if self.front.is_some() {
            0.0
        } else {
            self.offset_hz
        }
    }

    fn us_to_wide(&self, us: u64) -> u64 {
        (us as f64 * self.fs_w / 1e6) as u64
    }

    fn process_whole(&mut self, bytes: &[u8], format: IqFormat) {
        let bps = format.bytes_per_sample();
        const CHUNK: usize = 4096;
        for chunk in bytes.chunks(CHUNK * bps) {
            let n = (chunk.len() / bps) as u64;
            let mut skip = 0usize;
            if self.blank_in > 0 {
                let b = self.blank_in.min(n);
                self.blank_in -= b;
                skip = b as usize;
                if skip as u64 == n {
                    continue;
                }
            }
            let mut cur = std::mem::take(&mut self.conv);
            cur.clear();
            format.convert(&chunk[skip * bps..], &mut cur);
            let learn = self.bursts.is_empty();
            let dead = self.find_dead(&cur);
            if dead.is_empty() {
                self.dc.process(&mut cur, learn);
            } else {
                let mut pos = 0;
                for r in &dead {
                    if r.start > pos {
                        self.dc.process(&mut cur[pos..r.start], learn);
                    }
                    self.dc.subtract(&mut cur[r.clone()]);
                    pos = r.end;
                }
                if pos < cur.len() {
                    self.dc.process(&mut cur[pos..], learn);
                }
                let w0 = self.buf_end();
                let d = self.wide_decim as u64;
                for r in &dead {
                    let (a, b) = (w0 + r.start as u64 / d, w0 + (r.end as u64).div_ceil(d));
                    match self.dead.back_mut() {
                        Some(last) if last.1 >= a => last.1 = last.1.max(b),
                        _ => self.dead.push_back((a, b)),
                    }
                }
            }
            match self.front.as_mut() {
                Some((nco, fir)) => {
                    nco.mix(&mut cur);
                    let mut spare = std::mem::take(&mut self.tmp);
                    spare.clear();
                    fir.process(&cur, &mut spare);
                    self.buf.extend_from_slice(&spare);
                    self.tmp = spare;
                }
                None => self.buf.extend_from_slice(&cur),
            }
            self.conv = cur;
            self.run_blocks();
        }
    }

    /// Ranges of `cur` (converted input) that are dead: part of a run of at least `dead_min`
    /// identical samples (a run continues across calls).
    fn find_dead(&mut self, cur: &[C32]) -> Vec<Range<usize>> {
        let mut out: Vec<Range<usize>> = Vec::new();
        let Some(&end) = cur.last() else {
            return out;
        };
        // Fast path (live noise): a run of `dead_min` ≥ 2·STEP samples inside `cur` has two equal
        // samples STEP apart, and one continuing from the last call starts with the last sample.
        const STEP: usize = (DEAD_MIN_SAMPLES / 2) as usize;
        let continues = self.dead_last == Some(cur[0]);
        if !continues && !(STEP..cur.len()).step_by(STEP).any(|k| cur[k] == cur[k - STEP]) {
            let trailing = cur.iter().rev().take_while(|&&s| s == end).count();
            self.dead_last = Some(end);
            self.dead_run = trailing as u64;
            return out;
        }
        let mut run_start = 0usize; // where the current run starts in `cur` (0 if it began before)
        for (k, &s) in cur.iter().enumerate() {
            if self.dead_last == Some(s) {
                self.dead_run += 1;
            } else {
                self.dead_last = Some(s);
                self.dead_run = 1;
                run_start = k;
            }
            if self.dead_run >= self.dead_min {
                let from = if self.dead_run == self.dead_min { run_start } else { k };
                match out.last_mut() {
                    Some(r) if r.end >= from => r.end = k + 1,
                    _ => out.push(from..k + 1),
                }
            }
        }
        out
    }

    /// Run the detector over every complete block, then drop samples no burst can need any more.
    fn run_blocks(&mut self) {
        let block = self.det.block as u64;
        let n = self.det.space.n;
        while self.buf_end() >= self.next_block + block {
            let base = (self.next_block - (n / 2) as u64 - self.buf_start) as usize;
            self.det.acc.iter_mut().for_each(|a| *a = 0.0);
            for batch in 0..FRAMES / FFT_LANES {
                let starts = std::array::from_fn(|l| base + (batch * FFT_LANES + l) * n / 2);
                self.det.fft.accumulate_power(&self.buf, starts, &mut self.det.acc);
            }
            for k in 0..n {
                self.det.power[(k + n / 2) % n] = self.det.acc[k] / FRAMES as f32;
            }
            let start = self.next_block;
            self.next_block += block;
            self.end_block(start, start + block);
        }
        // Keep what the next block's frames and every open burst's extraction (quiet samples and
        // filter run-in before its first present block) still need.
        let need = (2 + PRE_QUIET_BLOCKS) * block + self.chan_len_w + n as u64;
        let first = self.bursts.iter().map(|b| b.first_present).min().unwrap_or(self.next_block);
        let keep_from = first.min(self.next_block).saturating_sub(need).max(self.buf_start);
        let drop = (keep_from - self.buf_start) as usize;
        if drop >= 65_536 && drop * 2 >= self.buf.len() {
            self.buf.drain(..drop);
            self.buf_start = keep_from;
        }
    }

    /// Detector decisions for one block `[start, end)` (wide samples): learn / track the floors,
    /// assign present bins to bursts, close the bursts that ended.
    fn end_block(&mut self, start: u64, end: u64) {
        let n = self.det.space.n;
        let space = self.det.space;
        let allowed = self.det.allowed;
        let peak = (0..n).filter(|&i| BinSpace::has(allowed, i)).map(|i| self.det.power[i]).fold(0f32, f32::max);
        self.rssi_peak = self.rssi_peak.max(peak / self.det.cw_gain);
        // Dead input has no noise to learn the floors from.
        let frame_start = start.saturating_sub((n / 2) as u64);
        while self.dead.front().is_some_and(|&(_, b)| b + (self.det.block as u64) < frame_start) {
            self.dead.pop_front();
        }
        let dead = self.dead.iter().any(|&(a, b)| a < end + (n / 2) as u64 && frame_start < b);
        if self.det.floor.is_empty() && dead {
            return;
        }
        if self.det.floor.is_empty() {
            // Learn the floors from the median of the first blocks after start/retune.
            self.det.init.push(self.det.power.clone());
            if self.det.init.len() >= FLOOR_INIT_BLOCKS {
                let medians: Vec<f32> = (0..n)
                    .map(|i| {
                        let v: Vec<f32> = self.det.init.iter().map(|b| b[i]).collect();
                        quantile(&v, 0.5).max(1e-6)
                    })
                    .collect();
                let cap = if space.circular { FLOOR_INIT_CAP * quantile(&medians, 0.5) } else { f32::INFINITY };
                self.det.floor = medians.iter().map(|&m| m.min(cap)).collect();
                self.det.init.clear();
            }
            return;
        }
        let power = &self.det.power;
        let floor = &self.det.floor;
        let mut present = 0u128;
        let (mut lo, mut hi) = (f32::INFINITY, 0f32);
        for i in 0..n {
            if BinSpace::has(allowed, i) {
                let r = power[i] / floor[i];
                (lo, hi) = (lo.min(r), hi.max(r));
                if r > DETECT_RATIO {
                    present |= 1 << i;
                }
            }
        }
        // A noise step: every bin of the band above its floor by about the same factor, block after
        // block. That is the floor moving (a receiver still settling after it started, a gain change,
        // a broadband interferer coming on), not a signal, which is peaked: re-learn the floors
        // instead of holding a burst open over the whole band until MAX_BURST_US.
        if lo > DETECT_RATIO && hi < STEP_FLATNESS * lo {
            if self.step_blocks == 0 {
                self.step_start = start;
            }
            self.step_blocks += 1;
        } else {
            self.step_blocks = 0;
        }
        if self.step_blocks >= STEP_BLOCKS {
            let since = self.step_start;
            self.bursts.retain(|b| b.first_present < since);
            self.det.floor.clear();
            self.det.init.clear();
            self.step_blocks = 0;
            return;
        }
        // Each signal in this block extends the burst whose peaks are near its own (merging bursts
        // it connects), or opens a new one.
        let block_no = start / self.det.block as u64;
        for c in space.clusters(present, self.det.join) {
            let parts = space.split(c, power, self.det.join);
            for (signal, peak) in parts {
                let hits: Vec<usize> = (0..self.bursts.len())
                    .filter(|&k| BinSpace::has(space.dilate(self.bursts[k].core, self.det.join), peak))
                    .collect();
                let k = match hits.first() {
                    None => {
                        self.bursts.push(OpenBurst::new(start, block_no, n));
                        self.bursts.len() - 1
                    }
                    Some(&first) => {
                        for &h in hits[1..].iter().rev() {
                            let other = self.bursts.remove(h);
                            self.bursts[first].absorb(other);
                        }
                        first
                    }
                };
                let level = 10.0 * (power[peak] / floor[peak]).log10();
                let b = &mut self.bursts[k];
                b.seen |= signal;
                b.core |= 1 << peak;
                b.present_now = true;
                b.mark(block_no, level);
            }
        }
        for b in self.bursts.iter_mut() {
            for (m, &p) in b.min_power.iter_mut().zip(power) {
                *m = m.min(p);
            }
            if b.present_now {
                b.present_now = false;
                b.last_present_end = end;
                b.present_blocks += 1;
                for ((e, &p), &f) in b.excess.iter_mut().zip(power).zip(floor) {
                    *e += p - f;
                }
            }
        }
        // Track the floors while nothing is on. Not during a burst, not even in bins away from it:
        // a strong burst can move the whole band (an RTL-SDR's AGC drops the gain while it is on).
        if self.bursts.is_empty() && !dead {
            for (f, &p) in self.det.floor.iter_mut().zip(&self.det.power) {
                let a = if p < 0.5 * *f { FLOOR_ALPHA_FAST } else { FLOOR_ALPHA };
                *f = (*f + a * (p - *f)).max(1e-6);
            }
        }
        // Close the bursts that ended (silent for GAP_US) or ran too long.
        let (gap, max_len) = (self.us_to_wide(GAP_US), self.us_to_wide(MAX_BURST_US));
        let mut closing = Vec::new();
        let mut k = 0;
        while k < self.bursts.len() {
            let b = &self.bursts[k];
            if end - b.last_present_end >= gap || end - b.first_present >= max_len {
                closing.push(self.bursts.remove(k));
            } else {
                k += 1;
            }
        }
        closing.sort_by(|a, b| b.strength(allowed).total_cmp(&a.strength(allowed)));
        let blk = self.det.block as u64;
        for mut b in closing {
            b.cut = end - b.last_present_end < gap;
            let cut = b.cut.then(|| b.clone());
            let closed = self.close(b, end);
            let Some(b) = cut else { continue };
            if closed.modulated {
                // Still on and carrying data at MAX_BURST_US (a held button, a continuous FSK
                // carrier: both levels are tones): a transmission. It goes on as a new burst from
                // its last frame gap, so the frame across the cut is whole in that one; from
                // MAX_OVERLAP_US back when there is no gap in that time (frames back to back, as a
                // held Kia V1 fob sends them), and a frame before the cut may then come out twice
                // (the app drops the repeat as a duplicate).
                let earliest = end.saturating_sub(self.us_to_wide(MAX_OVERLAP_US)).max(b.first_present);
                let resume = closed.last_gap.unwrap_or(earliest).clamp(earliest, end);
                self.bursts.push(b.continue_from(resume, blk, (n / 2) as u64));
                if let Some(r) = self.recent.back_mut() {
                    r.continued = true;
                }
            } else {
                // Cut while still present without data: bins that never dropped back to their floor
                // in MAX_BURST_US mean the floor moved (gain change, new interferer, a steady
                // carrier). Re-baseline them instead of staying open forever.
                for i in (0..n).filter(|&i| BinSpace::has(space.dilate(b.seen, self.det.join), i)) {
                    if b.min_power[i] > DETECT_RATIO * self.det.floor[i] {
                        self.det.floor[i] = b.min_power[i];
                    }
                }
            }
        }
    }

    /// Carrier of a burst (Hz, wide band): centre of the bins near the one whose power changed most
    /// while the burst was on, weighted by that change. Usually the change is new power over the
    /// floor; it is a drop when the carrier was already on while the floor was learned (a recording
    /// that starts mid-transmission) and is now keyed. A steady interferer doesn't change.
    fn carrier_hz(&self, b: &OpenBurst) -> f64 {
        let d = &self.det;
        let n = d.space.n;
        let region = d.space.dilate(b.seen, d.join) & d.allowed;
        let Some(p) = (0..n)
            .filter(|&i| BinSpace::has(region, i))
            .max_by(|&x, &y| b.excess[x].abs().total_cmp(&b.excess[y].abs()))
        else {
            return 0.0;
        };
        let (mut sw, mut swp) = (0f64, 0f64);
        let c = d.centroid as isize;
        for off in -c..=c {
            let j = p as isize + off;
            let i = if d.space.circular {
                j.rem_euclid(n as isize) as usize
            } else if (0..n as isize).contains(&j) {
                j as usize
            } else {
                continue;
            };
            if BinSpace::has(region, i) {
                let w = b.excess[i].abs() as f64;
                sw += w;
                swp += w * j as f64;
            }
        }
        let pos = if sw > 0.0 { swp / sw } else { p as f64 };
        wrap_hz(d.freq(pos), self.fs_w)
    }

    /// True if `b` is keyed with a burst at least 10 dB stronger: its artifact.
    fn is_artifact(&self, b: &OpenBurst) -> bool {
        let allowed = self.det.allowed;
        let s = ARTIFACT_RATIO * b.strength(allowed);
        self.bursts.iter().any(|o| o.strength(allowed) >= s && keyed_together(b.first_block, &b.history, o.first_block, &o.history))
            || self.recent.iter().any(|o| o.strength >= s && keyed_together(b.first_block, &b.history, o.first_block, &o.history))
    }

    /// True if a stronger burst (open or just closed) on the same carrier overlaps `[first, last)`.
    fn is_duplicate(&self, first: u64, last: u64, carrier: f64, strength: f32) -> bool {
        let same = |c: f64| wrap_hz(c - carrier, self.fs_w).abs() < DUPLICATE_HZ;
        let allowed = self.det.allowed;
        self.bursts.iter().any(|o| {
            o.first_present < last && first < o.last_present_end && o.strength(allowed) > strength && same(self.carrier_hz(o))
        }) || self
            .recent
            .iter()
            .any(|o| !o.continued && o.first_present < last && first < o.last_present_end && o.strength > strength && same(o.carrier_hz))
    }

    /// A burst ended at `close` (wide samples): slice it and queue the result.
    fn close(&mut self, b: OpenBurst, close: u64) -> Closed {
        let prev_end = self.last_burst_end;
        self.last_burst_end = prev_end.max(b.last_present_end);
        let strength = b.strength(self.det.allowed);
        let mut carrier = self.carrier_hz(&b);
        let artifact = self.is_artifact(&b);
        let horizon = b.last_present_end.saturating_sub(2 * self.us_to_wide(GAP_US));
        self.recent.retain(|o| o.last_present_end >= horizon);
        let mut burst = None;
        let mut last_gap = None;
        if !artifact && b.present_blocks >= MIN_PRESENT_BLOCKS {
            if let Some((found, gap)) = self.finalize(&b, carrier, close, prev_end) {
                carrier = self.target_w() + found.offset_hz as f64;
                if !self.is_duplicate(b.first_present, b.last_present_end, carrier, strength) {
                    (burst, last_gap) = (Some(found), gap);
                }
            }
        }
        self.recent.push_back(ClosedBurst {
            first_present: b.first_present,
            last_present_end: b.last_present_end,
            strength,
            carrier_hz: carrier,
            first_block: b.first_block,
            history: b.history,
            continued: false,
        });
        let modulated = burst.as_ref().is_some_and(|x| (x.am.is_some() || x.fm.is_some()) && x.snr_db >= CONTINUE_SNR_DB);
        self.out.extend(burst);
        Closed { modulated, last_gap }
    }

    /// Mix wide samples `[x0, x1)` by −`carrier_hz` and run the channel filter. Returns the channel
    /// samples, the wide-sample position of the first one and the wide samples per channel sample.
    fn extract(&self, x0: u64, x1: u64, carrier_hz: f64) -> (Vec<C32>, f64, f64) {
        let src = &self.buf[(x0 - self.buf_start) as usize..(x1 - self.buf_start) as usize];
        let mut nco = Nco::new(carrier_hz, self.fs_w);
        let mut firs: Vec<FirDecimator> = self.chan.iter().map(|(t, d)| FirDecimator::new(t.clone(), *d)).collect();
        let total: usize = self.chan.iter().map(|(_, d)| *d).product();
        let mut out = Vec::with_capacity(src.len() / total + 2);
        let (mut cur, mut spare) = (Vec::with_capacity(16_384), Vec::with_capacity(16_384));
        for chunk in src.chunks(16_384) {
            cur.clear();
            cur.extend_from_slice(chunk);
            nco.mix(&mut cur);
            for (k, fir) in firs.iter_mut().enumerate() {
                spare.clear();
                fir.process(&cur, &mut spare);
                if k + 1 == self.chan.len() {
                    out.extend_from_slice(&spare);
                }
                std::mem::swap(&mut cur, &mut spare);
            }
        }
        // Output j of a stage is centred on input j·D + D − 1 − (taps − 1)/2.
        let (mut t0, mut step) = (x0 as f64, 1.0f64);
        for (taps, d) in &self.chan {
            t0 += (*d as f64 - 1.0 - (taps.len() as f64 - 1.0) / 2.0) * step;
            step *= *d as f64;
        }
        (out, t0, step)
    }

    /// Channel-filter a completed burst around its carrier and slice it into AM and/or FM pairs.
    /// If the slicer finds the carrier (OOK) or the middle of the tones (FSK) far from the centre,
    /// the burst is filtered and sliced again around what it found.
    fn finalize(&self, b: &OpenBurst, carrier: f64, close: u64, prev_end: u64) -> Option<(Burst, Option<u64>)> {
        let blk = self.det.block as u64;
        let region_start = b.first_present.saturating_sub(2 * blk).max(self.buf_start + self.chan_len_w);
        let region_end = (b.last_present_end + 2 * blk).min(self.buf_end());
        if region_end <= region_start + blk {
            return None;
        }
        let mut carrier = carrier;
        let mut s = self.slice_around(b, (region_start, region_end), carrier, close, prev_end)?;
        if s.offset_hz.abs() > REFINE_HZ {
            let again = carrier + s.offset_hz as f64;
            if let Some(s2) = self.slice_around(b, (region_start, region_end), again, close, prev_end) {
                (carrier, s) = (again, s2);
            }
        }
        let burst = Burst {
            frequency: self.frequency,
            am: s.am,
            fm: s.fm,
            start_sample: region_start * self.wide_decim as u64,
            duration_us: ((b.last_present_end - b.first_present) as f64 * 1e6 / self.fs_w) as u64,
            snr_db: s.snr_db,
            offset_hz: (carrier - self.target_w() + s.offset_hz as f64) as f32,
            deviation_hz: s.deviation_hz,
        };
        Some((burst, s.last_gap.map(|g| g.max(0.0) as u64)))
    }

    /// Extract the burst region `[region_start, region_end)` (wide samples) into a channel centred
    /// on `carrier` (Hz, wide band), measure the channel noise and slice.
    fn slice_around(
        &self,
        b: &OpenBurst,
        (region_start, region_end): (u64, u64),
        carrier: f64,
        close: u64,
        prev_end: u64,
    ) -> Option<SliceOut> {
        let blk = self.det.block as u64;
        let buf_end = self.buf_end();
        let x0 = region_start.saturating_sub(PRE_QUIET_BLOCKS * blk + self.chan_len_w).max(self.buf_start);
        let x1 = (region_end + TAIL_QUIET_BLOCKS * blk).min(buf_end);
        let (zc, t0, step) = self.extract(x0, x1, carrier);
        let ch = |w: u64| (((w as f64 - t0) / step).round().max(0.0) as usize).min(zc.len());
        let (r0, r1) = (ch(region_start), ch(region_end));
        if r1 <= r0 + 8 {
            return None;
        }
        let present = {
            let hi = ch(b.last_present_end).clamp(r0, r1) - r0;
            (ch(b.first_present).clamp(r0, r1) - r0).min(hi)..hi
        };
        // Noise level in this channel: the quiet samples before and after the burst (else the
        // detector's floors), but no more than in the quietest stretches of the burst itself (OOK
        // gaps): what was around the burst may not be there while it is on (a transmitter's own
        // carrier before and after its data, an interferer that comes and goes).
        let block_powers = |z: &[C32]| -> Vec<f32> {
            z.chunks_exact(NOISE_BLOCK).map(|q| q.iter().map(|c| c.norm_sqr()).sum::<f32>() / NOISE_BLOCK as f32).collect()
        };
        // Not around a transmission longer than MAX_BURST_US: before the rest of a cut burst and
        // after one cut while still on, the carrier goes on.
        let valid = ch(x0 + self.chan_len_w).min(r0);
        let mut quiet = if b.continued { Vec::new() } else { block_powers(&zc[valid..r0]) };
        if !b.cut {
            quiet.extend(block_powers(&zc[r1..]));
        }
        let around = if quiet.len() >= MIN_QUIET_BLOCKS {
            quantile(&quiet, 0.5)
        } else {
            let half = CHANNEL_PASS_HZ + 0.5 * (self.fs_out - 2.0 * CHANNEL_PASS_HZ).max(0.2 * CHANNEL_PASS_HZ);
            self.det.channel_noise(carrier, self.fs_w, half)
        };
        let inside = block_powers(&zc[r0..r1]);
        let floor = NOISE_CALIBRATION
            * if inside.len() >= 2 * MIN_QUIET_BLOCKS {
                around.min(quantile(&inside, 0.1) / NOISE_Q10)
            } else {
                around
            };
        let to_ch = |w: u64| (w as f64 / step) as u64;
        let mut s = slice_burst(&zc[r0..r1], present, floor, self.fs_out, to_ch(region_start), to_ch(close), to_ch(prev_end))?;
        // Channel sample (from r0) → wide sample.
        s.last_gap = s.last_gap.map(|g| t0 + (r0 as f64 + g) * step);
        Some(s)
    }
}

/// What the slicer made of one burst's channel samples.
struct SliceOut {
    am: Option<Vec<LevelDuration>>,
    fm: Option<Vec<LevelDuration>>,
    snr_db: f32,
    /// Carrier (OOK) or FSK centre in the channel (Hz).
    offset_hz: f32,
    deviation_hz: Option<f32>,
    /// Middle of the last frame gap (a low of ≥ 6 ms; for FSK a level of either tone): samples.
    last_gap: Option<f64>,
}

/// Slice a burst's channel samples `z` (rate `fs`, carrier near 0 Hz, noise power `floor` per
/// sample, carrier present in `present`) into AM and/or FM pairs. `region_start`, `close_out` and
/// `prev_end` (channel samples) place the leading idle and the trailing gap.
///
/// Pass 1 slices with fixed 20 µs smoothing. The burst is then split into frames at lows of
/// 6 ms or more; for each frame the element length (TE) and, for OOK, the carrier offset are
/// estimated, and pass 2 averages over ~TE/5: coherently for OOK (the carrier's phase removed, see
/// [`ook_envelope`], so noise averages out before envelope detection, close to a matched filter)
/// and on the discriminator for FSK (tones found again per frame). Short-TE frames keep pass 1.
fn slice_burst(
    z: &[C32],
    present: Range<usize>,
    floor: f32,
    fs: f64,
    region_start: u64,
    close_out: u64,
    prev_end: u64,
) -> Option<SliceOut> {
    let to_us = 1e6 / fs;
    let w1 = odd_samples(SMOOTH_US, fs);
    let db1 = DEBOUNCE_US / to_us;
    let floor = floor.max(1e-6);

    // ── Pass 1: |z| averaged over 20 µs; threshold between the noise and the carrier level.
    let mag: Vec<f32> = z.iter().map(|c| c.norm()).collect();
    let am1 = slice_level(&moving_average(&mag, w1), RAYLEIGH_MEAN * floor.sqrt(), present, db1)?;
    let mask1 = levels_from_edges(am1.init, &am1.edges, z.len());
    let mut disc = Vec::with_capacity(z.len());
    disc.push(C32::new(0.0, 0.0));
    for k in 1..z.len() {
        disc.push(z[k] * z[k - 1].conj());
    }
    // FSK with symbols shorter than that smoothing and debounce resolve (a CC1101-style GFSK
    // packet: 13 µs bits at 75 kbps) only shows its two tones at the channel's own resolution.
    // Symbols a little longer (26 µs at 38.4 kbps) do show two tones, but come out merged: many
    // levels near the debounce limit. The unsmoothed slicing then replaces pass 1 when it finds a
    // clean element grid.
    let fine_te = 2.0 * DEBOUNCE_US / to_us;
    let fm1 = match slice_fsk(&disc, &mask1, w1, db1, fs) {
        Some(f) if short_level_share(&f.edges, fine_te) >= 0.1 => fine_fsk(&disc, &mask1, fine_te, fs).or(Some(f)),
        Some(f) => Some(f),
        None => fine_fsk(&disc, &mask1, fine_te, fs),
    };
    let carrier = carrier_offset(&disc, &mask1); // cycles/sample (OOK carrier or FSK centre)
    // Tone estimates from pass 1 (its short smoothing has the fewest samples on tone ramps).
    let tones = fm1.as_ref().map(|f| (f.lo, f.hi));

    // ── Pass 2, per frame (the burst split at long lows, so a frame is sliced with its own
    //    element length, carrier or tones even if the burst also holds something else).
    let (p_init, p_edges) = match &fm1 {
        Some(f) => (f.init, f.edges.clone()),
        None => (am1.init, am1.edges.clone()),
    };
    let mut refined_edges = Vec::with_capacity(p_edges.len());
    let mut refined_any = false;
    for (a, b) in frame_ranges(p_init, &p_edges, z.len(), FRAME_GAP_US / to_us) {
        let frame_edges: Vec<f64> = p_edges.iter().copied().filter(|&e| e >= a as f64 && e < b as f64).collect();
        let refined = frame_te(&frame_edges, 20_000.0 / to_us, 5.0 * w1 as f64)
            .filter(|&te| te >= 5.0 * w1 as f64)
            .and_then(|te| {
                let w2 = ((te / 5.0).min(1_000.0 / to_us) as usize) | 1;
                let db2 = (te / 4.0).max(db1);
                let (init, edges) = if fm1.is_some() {
                    // FSK: discriminator averaged over ~TE/5, tones found again in this frame.
                    let f = slice_fsk(&disc[a..b], &mask1[a..b], w2, db2, fs)?;
                    (f.init, f.edges)
                } else {
                    // OOK: the carrier's phase removed, then averaged coherently.
                    let env = ook_envelope(&z[a..b], &disc[a..b], &mask1[a..b], w2, fs);
                    let noise2 = RAYLEIGH_MEAN * (floor / w2 as f32).sqrt();
                    let s = slice_level(&env, noise2, 0..(b - a), db2)?;
                    (s.init, s.edges)
                };
                // Frames start and end inside a long low: anything else can't be stitched.
                (!init && edges.len() % 2 == 0).then_some(edges)
            });
        match refined {
            Some(e) => {
                refined_edges.extend(e.iter().map(|x| x + a as f64));
                refined_any = true;
            }
            None => refined_edges.extend(frame_edges),
        }
    }
    let am = match (&fm1, refined_any) {
        (None, true) => Sliced { edges: refined_edges.clone(), ..am1.clone() },
        _ => am1.clone(),
    };
    let fm = match fm1 {
        Some(f) if refined_any => Some(FskSliced { edges: refined_edges, ..f }),
        other => other,
    };

    // ── Pairs, and which paths carry this burst's modulation.
    let fm_pairs = fm
        .as_ref()
        .map(|f| edges_to_pairs(f.init, &f.edges, z.len(), region_start, close_out, prev_end, to_us))
        .filter(|p| p.len() >= MIN_PAIRS);
    let am_pairs = edges_to_pairs(am.init, &am.edges, z.len(), region_start, close_out, prev_end, to_us);
    let am_rich = count_short_on_segments(am.init, &am.edges, z.len(), 5_000.0 / to_us) >= 4;
    let am_out = (am_pairs.len() >= MIN_PAIRS && (fm_pairs.is_none() || am_rich)).then_some(am_pairs);
    if am_out.is_none() && fm_pairs.is_none() {
        return None;
    }
    let offset_hz = match tones {
        Some((lo, hi)) if fm.is_some() => (lo + hi) / 2.0,
        _ => (carrier * fs) as f32,
    };
    let last_gap = match (&fm, &fm_pairs) {
        (Some(f), Some(_)) => last_long_level(f.init, &f.edges, z.len(), FRAME_GAP_US / to_us, true),
        _ => last_long_level(am.init, &am.edges, z.len(), FRAME_GAP_US / to_us, false),
    };
    Some(SliceOut {
        am: am_out,
        deviation_hz: tones.filter(|_| fm_pairs.is_some()).map(|(lo, hi)| (hi - lo) / 2.0),
        fm: fm_pairs,
        snr_db: 20.0 * (am.hi / am.noise).log10(),
        offset_hz,
        last_gap,
    })
}

/// Middle (samples) of the last level lasting at least `gap` samples: a low, or with `either`
/// (FSK: no data while one tone is held) a level of either value. The trailing level counts.
fn last_long_level(init: bool, edges: &[f64], len: usize, gap: f64, either: bool) -> Option<f64> {
    let mut found = None;
    let mut level = init;
    let mut start = 0.0;
    for &e in edges.iter().chain(std::iter::once(&(len as f64))) {
        if (either || !level) && e - start >= gap {
            found = Some((start + e) / 2.0);
        }
        level = !level;
        start = e;
    }
    found
}

/// Odd window length (≥ 1) covering `us` microseconds at `fs`.
fn odd_samples(us: f64, fs: f64) -> usize {
    ((us * fs / 1e6).round() as usize) | 1
}

/// A sliced level signal: initial level, edge times (fractional samples), and the levels used.
#[derive(Debug, Clone)]
struct Sliced {
    init: bool,
    edges: Vec<f64>,
    /// Carrier level found (envelope units).
    hi: f32,
    /// Noise level assumed (envelope units).
    noise: f32,
}

/// Threshold an envelope halfway between the known noise level and the carrier level (the mean of
/// the samples above that midpoint, iterated), with 10 % hysteresis and `debounce` samples.
fn slice_level(env: &[f32], noise: f32, present: std::ops::Range<usize>, debounce: f64) -> Option<Sliced> {
    let pe = &env[present];
    if pe.is_empty() {
        return None;
    }
    let mut hi = quantile(pe, 0.95).max(noise * 1.5);
    for _ in 0..12 {
        let thr = (noise + hi) / 2.0;
        let (mut s, mut n) = (0f64, 0usize);
        for &v in pe {
            if v >= thr {
                s += v as f64;
                n += 1;
            }
        }
        if n == 0 {
            break;
        }
        let new_hi = (s / n as f64) as f32;
        let done = (new_hi - hi).abs() < 1e-3 * hi;
        hi = new_hi;
        if done {
            break;
        }
    }
    if hi < noise * 1.25 {
        return None;
    }
    let (init, edges) = slice(env, (noise + hi) / 2.0, 0.1 * (hi - noise));
    let (init, edges) = debounce_edges(init, edges, debounce);
    Some(Sliced { init, edges, hi, noise })
}

/// A sliced FSK burst: levels plus the two tone frequencies (Hz).
#[derive(Debug, Clone)]
struct FskSliced {
    init: bool,
    edges: Vec<f64>,
    lo: f32,
    hi: f32,
}

/// FM path: average the discriminator products over `w` samples, find the two tones among the
/// carrier-on samples (two-means = per-burst AFC), and slice halfway between them (outside the
/// carrier the level is low). `None` unless the burst really has two distinct tones.
fn slice_fsk(disc: &[C32], mask: &[bool], w: usize, debounce: f64, fs: f64) -> Option<FskSliced> {
    let hz_per_rad = (fs / std::f64::consts::TAU) as f32;
    let freq: Vec<f32> = moving_sum_complex(disc, w).iter().map(|d| d.im.atan2(d.re) * hz_per_rad).collect();
    let mut on = Vec::new();
    let mut k = 0;
    while k < mask.len() {
        if mask[k] {
            let start = k;
            while k < mask.len() && mask[k] {
                k += 1;
            }
            // Skip the on/off transitions, where the average mixes in noise.
            let (a, e) = (start + w, k.saturating_sub(w));
            if e > a {
                on.extend_from_slice(&freq[a..e]);
            }
        } else {
            k += 1;
        }
    }
    if on.len() < 32 {
        return None;
    }
    let tm = two_means(&on, quantile(&on, 0.1), quantile(&on, 0.9));
    let sep = tm.hi - tm.lo;
    let minority = tm.n_lo.min(tm.n_hi) as f32 / on.len() as f32;
    if sep < FSK_MIN_SEPARATION_HZ || sep < FSK_MIN_BIMODALITY * tm.spread || minority < 0.03 {
        return None;
    }
    let off = tm.lo - 4.0 * sep;
    let sig: Vec<f32> = freq.iter().zip(mask).map(|(&f, &m)| if m { f } else { off }).collect();
    let (init, edges) = slice(&sig, (tm.lo + tm.hi) / 2.0, 0.1 * sep);
    let (init, edges) = debounce_edges(init, edges, debounce);
    Some(FskSliced { init, edges, lo: tm.lo, hi: tm.hi })
}

/// FSK sliced without smoothing, for bursts whose symbols are shorter than `max_te` samples (pass 1
/// smooths over 20 µs and drops levels under 30 µs, which merges them): only if the carrier is on
/// in long stretches (FSK holds it through a frame), its samples hold two distinct tones at the
/// channel's resolution, and the levels come out as clean multiples of one element length (at
/// least 2 samples). Slower FSK is found by pass 1 and never gets here; OOK (one tone, even a
/// pulled one: its levels are pulses, not short symbols) and noise (not two tones) don't pass.
fn fine_fsk(disc: &[C32], mask: &[bool], max_te: f64, fs: f64) -> Option<FskSliced> {
    let mut longest = 0usize;
    let mut run = 0usize;
    for &m in mask {
        run = if m { run + 1 } else { 0 };
        longest = longest.max(run);
    }
    if (longest as f64) < FINE_MIN_RUN_US * fs / 1e6 {
        return None;
    }
    let f = slice_fsk(disc, mask, 1, 1.0, fs)?;
    let te = estimate_te(&f.edges, 32.0 * max_te)?;
    if te >= max_te || te < 2.0 {
        return None;
    }
    let durations: Vec<f64> = f.edges.windows(2).map(|w| w[1] - w[0]).filter(|&d| d <= 32.0 * te).collect();
    let clean = durations.iter().filter(|&&d| (d / te - (d / te).round()).abs() < 0.3).count();
    if durations.len() < 16 || (clean as f64) < 0.8 * durations.len() as f64 {
        return None;
    }
    // Symbols fill the carrier's time, on both tones: not a pulled OOK carrier, whose key-on
    // blips of the other "tone" are short but rare.
    let (mut short, mut level, mut prev) = ([0f64; 2], f.init, 0.0);
    for &e in &f.edges {
        if e - prev <= 32.0 * te {
            short[usize::from(level)] += e - prev;
        }
        level = !level;
        prev = e;
    }
    let on = mask.iter().filter(|&&m| m).count() as f64;
    let total = short[0] + short[1];
    if total < 0.5 * on || short[0].min(short[1]) < 0.25 * total {
        return None;
    }
    slice_fsk(disc, mask, 1, (te / 4.0).max(1.0), fs)
}

/// The carrier must be on for this long at a stretch before short-symbol FSK is looked for.
const FINE_MIN_RUN_US: f64 = 1_000.0;

/// Share of a sliced burst's levels (between edges) shorter than `limit` samples.
fn short_level_share(edges: &[f64], limit: f64) -> f64 {
    let n = edges.len().saturating_sub(1);
    if n == 0 {
        return 0.0;
    }
    edges.windows(2).filter(|w| w[1] - w[0] < limit).count() as f64 / n as f64
}

/// Mean frequency (cycles/sample) of the carrier-on samples: arg of the summed discriminator
/// products (robust to noise clicks, unlike averaging per-sample frequencies).
fn carrier_offset(disc: &[C32], mask: &[bool]) -> f64 {
    let (mut re, mut im) = (0f64, 0f64);
    for (d, &m) in disc.iter().zip(mask) {
        if m {
            re += d.re as f64;
            im += d.im as f64;
        }
    }
    if re == 0.0 && im == 0.0 {
        0.0
    } else {
        im.atan2(re) / std::f64::consts::TAU
    }
}

/// Sample ranges of the frames in a sliced burst: cut in the middle of every low lasting at least
/// `gap` samples.
fn frame_ranges(init: bool, edges: &[f64], len: usize, gap: f64) -> Vec<(usize, usize)> {
    let mut cuts = vec![0usize];
    let mut level = init;
    let mut start = 0.0;
    for &e in edges {
        if !level && e - start >= gap {
            cuts.push(((start + e) / 2.0) as usize);
        }
        level = !level;
        start = e;
    }
    cuts.push(len);
    cuts.dedup();
    cuts.windows(2).filter(|w| w[1] > w[0]).map(|w| (w[0], w[1])).collect()
}

/// Pass-2 envelope of one OOK frame (`z`, its discriminator products `disc`, pass 1's carrier mask):
/// the carrier's phase removed, then |z| averaged coherently over `w` samples.
///
/// A coherent average over `w` samples loses the carrier wherever its frequency is more than
/// ~1/(4·w) cycles/sample from the one removed, and cheap remotes move: their oscillator drifts
/// several kHz during a press and is pulled at every key-on (the Audiovox PRO-OE3B in rtl_433's
/// recordings: −63 → −101 kHz within one 33 ms frame, up to 16 kHz within one 900 µs pulse). So
/// the phase comes from the frame's mean frequency, or from a track of the carrier's frequency when
/// that follows a movement the mean misses: a slow track (drift over the press, across pulses), then
/// a fast one (the pull, within each pulse). A track is used only if its deviation from the previous
/// choice is well above its own noise, measured by tracking two interleaved halves of the samples
/// separately (at low SNR a track's noise costs more than a small drift, and a steady carrier keeps
/// the mean exactly as before).
fn ook_envelope(z: &[C32], disc: &[C32], mask: &[bool], w: usize, fs: f64) -> Vec<f32> {
    let fixed = || {
        let mut zc = z.to_vec();
        Nco::new(carrier_offset(disc, mask) * fs, fs).mix(&mut zc);
        coherent_envelope(&zc, w)
    };
    // Carrier-on samples to compare the tracks at.
    let probes: Vec<usize> = (0..z.len().min(mask.len())).filter(|&k| mask[k] && k % 4 == 0).collect();
    if probes.len() < 16 {
        return fixed();
    }
    // Only keyed carriers (the pull is a key-on effect). A carrier on all through the frame is not
    // OOK: weak FSK that pass 1 didn't recognise, whose tones the mean's coherent average separates
    // like an OOK envelope (one tone passes, the other doesn't), and a track would follow them.
    let (first, last) = (probes[0], probes[probes.len() - 1]);
    let on = mask[first..=last].iter().filter(|&&m| m).count();
    if on as f32 > TRACK_MAX_DUTY * (last - first + 1) as f32 {
        return fixed();
    }
    let angle = |s: Complex64| (s.im.atan2(s.re) / std::f64::consts::TAU) as f32;
    let wrap = |d: f32| d - d.round();
    let f0 = carrier_offset(disc, mask) as f32;
    let mut reference: Vec<f32> = vec![f0; probes.len()];
    let mut ref_noise = 0f32;
    let mut chosen: Option<Vec<Complex64>> = None;
    // A movement smaller than this (rms, cycles/sample) costs the mean's envelope < 0.2 dB.
    let min_dev = (MIN_TRACKED_DEVIATION / w as f32).powi(2);
    let max_noise = (MAX_TRACK_NOISE / w as f32).powi(2);
    for (half, within_runs) in [(TREND_WINDOWS * w, false), (w, true)] {
        let block = (half / 4).max(1);
        let full = track_sums(disc, mask, half, within_runs, None);
        let a = track_sums(disc, mask, half, within_runs, Some((block, 0)));
        let b = track_sums(disc, mask, half, within_runs, Some((block, 1)));
        // Weighted by the carrier in each window (|sum|): where there is none, the track doesn't
        // matter (and is as random from half the samples as from all of them).
        let (mut dev, mut noise, mut wsum, mut n_noise) = (0f64, 0f64, 0f64, 0usize);
        let mut angles = Vec::with_capacity(probes.len());
        for (p, &k) in probes.iter().enumerate() {
            let f = if full[k] == Complex64::new(0.0, 0.0) { reference[p] } else { angle(full[k]) };
            angles.push(f);
            if a[k] != Complex64::new(0.0, 0.0) && b[k] != Complex64::new(0.0, 0.0) {
                let wt = full[k].norm();
                dev += wt * (wrap(f - reference[p]) as f64).powi(2);
                noise += wt * (wrap(angle(a[k]) - angle(b[k])) as f64).powi(2) / 4.0;
                wsum += wt;
                n_noise += 1;
            }
        }
        if n_noise < 16 || wsum <= 0.0 {
            continue;
        }
        let (dev, noise) = ((dev / wsum) as f32, (noise / wsum) as f32);
        // Nested averages: the finer one's deviation from the coarser includes its extra noise only.
        let excess = dev - (noise - ref_noise).max(0.0);
        // A track noisier than this loses more than it can gain (and its noise isn't measurable from
        // the two halves any more: at low SNR they are as random as the whole).
        let usable = noise < max_noise;
        let used = usable && excess > (TRACK_SIGNIFICANCE * noise).max(min_dev);
        if used {
            reference = angles;
            ref_noise = noise;
            chosen = Some(full);
        }
    }
    match chosen {
        Some(sums) => coherent_envelope(&derotate(z, &sums), w),
        None => fixed(),
    }
}

/// Frames whose carrier is on for more than this fraction of their span are not tracked.
const TRACK_MAX_DUTY: f32 = 0.9;
/// A track replaces the mean (or the slower track) only if the carrier's movement it follows is this
/// many times its own noise (variance ratio).
const TRACK_SIGNIFICANCE: f32 = 4.0;
/// ... and only if that movement matters: rms deviation over 0.12 / (coherent window) cycles.
const MIN_TRACKED_DEVIATION: f32 = 0.12;
/// A track noisier than this (rms, cycles/sample, times the coherent window) is not used at all.
const MAX_TRACK_NOISE: f32 = 0.35;
/// The slow track (drift over the press) averages the discriminator over ± this many coherent
/// windows, across pulses; the fast one (the pull at every key-on) over ± one, within each pulse.
const TREND_WINDOWS: usize = 8;

/// Windowed sums of the discriminator products of the carrier-on samples (`mask`) per sample: their
/// argument is the carrier's local frequency. The window is ±`half` samples; `within_runs`: only
/// the same carrier-on run (a pulse's frequency is not mixed with the next one's: each key-on pulls
/// it anew), over a window of the same length kept inside the run. `subset` = (block, parity): only
/// the samples in every other block of `block` samples (two independent halves, for the noise).
/// Zero where the window has no samples.
fn track_sums(disc: &[C32], mask: &[bool], half: usize, within_runs: bool, subset: Option<(usize, usize)>) -> Vec<Complex64> {
    let n = disc.len().min(mask.len());
    let mut prefix = vec![Complex64::new(0.0, 0.0); n + 1];
    for k in 0..n {
        let take = mask[k] && subset.is_none_or(|(blk, parity)| (k / blk) % 2 == parity);
        let v = if take { Complex64::new(disc[k].re as f64, disc[k].im as f64) } else { Complex64::new(0.0, 0.0) };
        prefix[k + 1] = prefix[k] + v;
    }
    let mut out = vec![Complex64::new(0.0, 0.0); n];
    if !within_runs {
        for (j, o) in out.iter_mut().enumerate() {
            *o = prefix[(j + half + 1).min(n)] - prefix[j.saturating_sub(half)];
        }
        return out;
    }
    let mut k = 0;
    while k < n {
        if !mask[k] {
            k += 1;
            continue;
        }
        let s0 = k;
        while k < n && mask[k] {
            k += 1;
        }
        // The run's products, without its first and last (they straddle the edges).
        let (i0, i1) = if k - s0 > 4 { (s0 + 1, k - 1) } else { (s0, k) };
        let span = (2 * half + 1).min(i1 - i0);
        for (j, o) in out.iter_mut().enumerate().take(k).skip(s0) {
            let lo = j.saturating_sub(half).clamp(i0, i1 - span);
            *o = prefix[lo + span] - prefix[lo];
        }
    }
    out
}

/// `z` with the carrier's phase removed: the phase accumulated from the local frequency (the
/// argument of `sums`, held where it is zero, e.g. between pulses: the rotation of noise doesn't
/// matter).
fn derotate(z: &[C32], sums: &[Complex64]) -> Vec<C32> {
    let mut r = C32::new(1.0, 0.0);
    let mut u = C32::new(1.0, 0.0);
    z.iter()
        .zip(sums)
        .enumerate()
        .map(|(k, (&x, s))| {
            let m = s.norm();
            if m > 0.0 {
                u = C32::new((s.re / m) as f32, (s.im / m) as f32);
            }
            r *= u.conj();
            if k % 256 == 255 {
                r /= r.norm();
            }
            x * r
        })
        .collect()
}

/// |mean| of `z` over a centred window of `w` samples (coherent averaging).
fn coherent_envelope(z: &[C32], w: usize) -> Vec<f32> {
    let half = w / 2;
    let n = z.len();
    moving_sum_complex(z, w)
        .iter()
        .enumerate()
        .map(|(i, s)| {
            let count = (i + half + 1).min(n) - i.saturating_sub(half);
            s.norm() / count as f32
        })
        .collect()
}

/// Element length (samples) of a sliced burst: the 10 % time-weighted quantile of its level
/// durations (short noise glitches carry little time), refined to the median duration near it.
/// Durations over `max_len` (gaps between frames) are ignored.
fn estimate_te(edges: &[f64], max_len: f64) -> Option<f64> {
    te_of_durations(edges.windows(2).map(|w| w[1] - w[0]).filter(|&x| x <= max_len).collect())
}

/// Element length of one frame for pass 2: [estimate_te] of all its levels, or of one level's
/// (HIGHs or LOWs) when that is under a third of it (pass 2's TE/4 debounce would erase those
/// levels) and at least `min_te`. Short pulses between long lows (a preamble of 300 µs pulses
/// 3.7 ms apart) carry under 10 % of the frame's time, so the estimate of all levels is the long
/// low. Frames whose levels survive the estimate of all of them keep it.
fn frame_te(edges: &[f64], max_len: f64, min_te: f64) -> Option<f64> {
    let all = estimate_te(edges, max_len)?;
    let durations: Vec<f64> = edges.windows(2).map(|w| w[1] - w[0]).collect();
    let mut te = all;
    for parity in 0..2 {
        let level: Vec<f64> = durations.iter().skip(parity).step_by(2).copied().filter(|&x| x <= max_len).collect();
        if let Some(t) = te_of_durations(level).filter(|&t| t >= min_te && t < all / 3.0 && t < te) {
            te = t;
        }
    }
    Some(te)
}

/// [estimate_te] of level durations.
fn te_of_durations(mut d: Vec<f64>) -> Option<f64> {
    if d.len() < 8 {
        return None;
    }
    d.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let total: f64 = d.iter().sum();
    let mut acc = 0.0;
    let mut q = d[0];
    for &x in &d {
        acc += x;
        if acc >= 0.1 * total {
            q = x;
            break;
        }
    }
    let near: Vec<f64> = d.iter().copied().filter(|&x| x >= 0.7 * q && x <= 1.4 * q).collect();
    near.get(near.len() / 2).copied()
}

/// Slice `sig` at `thr` with hysteresis `h`. Returns the initial level and the edge times (fractional
/// sample index of the centre-threshold crossing that preceded each confirmed transition).
fn slice(sig: &[f32], thr: f32, h: f32) -> (bool, Vec<f64>) {
    let mut edges = Vec::new();
    if sig.is_empty() {
        return (false, edges);
    }
    let init = sig[0] > thr;
    let mut level = init;
    let mut last_cross = 0.0f64;
    for n in 1..sig.len() {
        let (a, b) = (sig[n - 1], sig[n]);
        if (a <= thr && b > thr) || (a > thr && b <= thr) {
            last_cross = (n - 1) as f64 + ((thr - a) / (b - a)) as f64;
        }
        if !level && b > thr + h {
            level = true;
            edges.push(last_cross);
        } else if level && b < thr - h {
            level = false;
            edges.push(last_cross);
        }
    }
    (init, edges)
}

/// Drop levels shorter than `min_len` samples by removing both of their edges (KAT 1.4's debounce).
fn debounce_edges(init: bool, edges: Vec<f64>, min_len: f64) -> (bool, Vec<f64>) {
    let mut out: Vec<f64> = Vec::with_capacity(edges.len());
    for e in edges {
        if let Some(&last) = out.last() {
            if e - last < min_len {
                out.pop();
                continue;
            }
        }
        out.push(e);
    }
    (init, out)
}

/// Per-sample level from an initial level and edge times.
fn levels_from_edges(init: bool, edges: &[f64], len: usize) -> Vec<bool> {
    let mut out = Vec::with_capacity(len);
    let mut level = init;
    let mut it = edges.iter().peekable();
    for k in 0..len {
        while let Some(&&e) = it.peek() {
            if e <= k as f64 {
                level = !level;
                it.next();
            } else {
                break;
            }
        }
        out.push(level);
    }
    out
}

/// Number of high segments shorter than `max_len` samples.
fn count_short_on_segments(init: bool, edges: &[f64], len: usize, max_len: f64) -> usize {
    let mut level = init;
    let mut start = 0.0;
    let mut n = 0;
    for &e in edges {
        if level && e - start < max_len {
            n += 1;
        }
        level = !level;
        start = e;
    }
    if level && len as f64 - start < max_len {
        n += 1;
    }
    n
}

/// Turn edges (sample times relative to `region_start`) into level/duration pairs: a leading low for
/// the idle period since `prev_end`, the levels, and a trailing segment up to `close_out`.
fn edges_to_pairs(
    init: bool,
    edges: &[f64],
    len: usize,
    region_start: u64,
    close_out: u64,
    prev_end: u64,
    to_us: f64,
) -> Vec<LevelDuration> {
    let mut pairs = Vec::with_capacity(edges.len() + 2);
    if edges.is_empty() {
        return pairs;
    }
    let mut level = init;
    let mut t_prev = 0.0f64;
    for (k, &e) in edges.iter().enumerate() {
        let dur = (e - t_prev) * to_us;
        if k == 0 {
            if level {
                pairs.push(LevelDuration::new(false, LEAD_MIN_US as u32));
                pairs.push(LevelDuration::new(true, dur.round().max(1.0) as u32));
            } else {
                let idle = (region_start.saturating_sub(prev_end)) as f64 * to_us + dur;
                let idle = (idle as u64).clamp(LEAD_MIN_US, LEAD_MAX_US);
                pairs.push(LevelDuration::new(false, idle as u32));
            }
        } else {
            pairs.push(LevelDuration::new(level, dur.round().max(1.0) as u32));
        }
        level = !level;
        t_prev = e;
    }
    // Trailing level: up to the close point (at least the gap, also when a file ends early) for a
    // low, up to the region end for a high.
    let end = if level { len as f64 } else { (close_out.max(region_start) - region_start) as f64 };
    let mut dur = ((end - t_prev) * to_us).round().max(1.0);
    if !level {
        dur = dur.max(GAP_US as f64);
    }
    pairs.push(LevelDuration::new(level, dur.min(u32::MAX as f64) as u32));
    pairs
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stage_plans_for_common_rates() {
        for (fs, out) in [
            (2_000_000.0, 250_000.0),
            (2_048_000.0, 256_000.0),
            (2_400_000.0, 240_000.0),
            (1_000_000.0, 250_000.0),
            (10_000_000.0, 250_000.0),
            (250_000.0, 250_000.0),
        ] {
            let (stages, fs_out) = plan_stages(fs);
            assert!((fs_out - out).abs() < 1.0, "{fs}: {fs_out}");
            for s in &stages {
                assert!(s.stop > s.pass, "{fs}: {s:?}");
            }
        }
    }

    #[test]
    fn bins_dilate_and_cluster() {
        let line = BinSpace { n: 16, circular: false };
        let circle = BinSpace { n: 16, circular: true };
        assert_eq!(line.dilate(1 << 1, 2), 0b1111);
        assert_eq!(circle.dilate(1 << 1, 2), 0b1111 | 1 << 15);
        assert_eq!(circle.dilate(1 << 15, 1), 1 << 14 | 1 << 15 | 1);
        // Gaps up to `join` stay together; around the circle only when the band wraps.
        let present = 1 << 0 | 1 << 3 | 1 << 9 | 1 << 14;
        assert_eq!(line.clusters(present, 3), vec![1 << 0 | 1 << 3, 1 << 9, 1 << 14]);
        assert_eq!(circle.clusters(present, 3), vec![1 << 0 | 1 << 3 | 1 << 14, 1 << 9]);
    }

    #[test]
    fn clusters_split_at_prominent_peaks() {
        let line = BinSpace { n: 16, circular: false };
        let c = (2..=10).fold(0u128, |m, i| m | 1 << i);
        let mut power = vec![1f32; 16];
        // A strong carrier at bin 4 with its skirt, and a weaker signal at bin 9 (valley 10 at bin 7).
        for (i, p) in [(2, 10.0), (3, 100.0), (4, 1000.0), (5, 100.0), (6, 20.0), (7, 10.0), (8, 15.0), (9, 60.0), (10, 15.0)] {
            power[i] = p;
        }
        let parts = line.split(c, &power, 4);
        assert_eq!(parts, vec![(0b111_1100, 4), (0b111_1000_0000, 9)]);
        // Not prominent enough over the valley: part of the strong one.
        power[9] = 30.0;
        assert_eq!(line.split(c, &power, 4), vec![(c, 4)]);
        // FSK tones 3 bins apart: one signal, however deep the dip between them.
        let mut fsk = vec![1f32; 16];
        for (i, p) in [(3, 50.0), (4, 100.0), (5, 5.0), (6, 5.0), (7, 90.0), (8, 40.0)] {
            fsk[i] = p;
        }
        let c = (3..=8).fold(0u128, |m, i| m | 1 << i);
        assert_eq!(line.split(c, &fsk, 4), vec![(c, 4)]);
    }

    #[test]
    fn artifacts_are_keyed_with_their_source() {
        let on = |v: &[u8]| v.iter().map(|&x| if x > 0 { x as f32 } else { f32::NAN }).collect::<Vec<f32>>();
        // A keyed source (OOK) and a weak copy present only in its blocks: artifact.
        let a = on(&[30, 0, 30, 30, 0, 30, 0, 30, 30, 0, 30, 30]);
        let b = on(&[5, 0, 5, 0, 0, 6, 0, 5, 5, 0, 0, 5]);
        assert!(keyed_together(0, &b, 0, &a));
        // A steady carrier, on long before and after: a fob keyed under it is not its artifact...
        let steady = on(&[20; 40]);
        let fob = on(&[9, 0, 12, 7, 0, 0, 11, 8, 0, 10, 6, 0]);
        assert!(!keyed_together(10, &fob, 0, &steady));
        // ...but a copy whose level follows the source's is, and so is one starting and ending with it.
        let src: Vec<f32> = (0..24).map(|k| 20.0 + (k % 5) as f32).collect();
        let copy: Vec<f32> = src.iter().map(|l| l - 25.0).collect();
        assert!(keyed_together(30, &copy, 30, &src));
        let flat = vec![10.0f32; 24];
        assert!(keyed_together(30, &flat, 31, &[20.0f32; 22]));
    }

    #[test]
    fn search_band_covers_the_flipper_window() {
        // Live 2 MS/s, target fs/8 above the LO: −325…+325 kHz around it except ±1 bin at the LO,
        // and on to the band edges (−1.15 / +0.65 MHz).
        let p = RxPipeline::new(2_000_000, 250e3, 433_920_000);
        let (lo, hi) = p.search_range_hz();
        assert!(lo < -1.1e6 && hi > 0.6e6, "{lo} {hi}");
        let d = &p.det;
        let open = |off: f64| {
            let i = ((off + p.target_w()) / d.bin_hz).round() as isize + (d.space.n / 2) as isize;
            BinSpace::has(d.allowed, i as usize)
        };
        for off in [-325e3, -300e3, -200e3, -100e3, 0.0, 100e3, 200e3, 300e3, 325e3] {
            assert!(open(off), "{off}");
        }
        assert!(!open(-250e3) && !open(-219e3) && !open(-281e3), "LO bins are blind");
        // A file at the fob frequency uses its whole band, around the circle.
        let f = RxPipeline::new(250_000, 0.0, 433_920_000);
        assert!(f.det.space.circular && f.det.allowed == f.det.space.all());
        // Fast input: decimated around the target, ±875 kHz.
        let fast = RxPipeline::new(10_000_000, 1.25e6, 433_920_000);
        let (lo, hi) = fast.search_range_hz();
        assert!((fast.wide_rate() - 2.5e6).abs() < 1.0 && lo < -0.85e6 && hi > 0.85e6, "{lo} {hi}");
    }

    #[test]
    fn slice_interpolates_and_debounces() {
        // Ramp crossing 0.5 between samples 2 and 3 at 2.5.
        let sig = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0];
        let (init, edges) = slice(&sig, 0.5, 0.1);
        assert!(!init);
        assert_eq!(edges, vec![2.5, 5.5]);
        // A 1-sample glitch is removed by a 2-sample debounce.
        let (_, e) = debounce_edges(false, vec![10.0, 20.0, 20.5, 40.0], 2.0);
        assert_eq!(e, vec![10.0, 40.0]);
    }

    #[test]
    fn pairs_start_low_and_end_with_the_gap() {
        // fs_out 250k → 4 µs per sample.
        let pairs = edges_to_pairs(false, &[100.0, 150.0, 200.0], 300, 1000, 1000 + 20_300, 0, 4.0);
        let v: Vec<(bool, u32)> = pairs.iter().map(|p| (p.level, p.duration_us)).collect();
        assert!(!v[0].0);
        assert!(v[0].1 >= LEAD_MIN_US as u32);
        assert_eq!(v[1], (true, 200));
        assert_eq!(v[2], (false, 200));
        assert_eq!(v[3], (true, 400));
        // A trailing low is at least the gap.
        let pairs = edges_to_pairs(false, &[100.0, 150.0], 300, 1000, 1200, 0, 4.0);
        assert_eq!(pairs.last().map(|p| (p.level, p.duration_us)), Some((false, GAP_US as u32)));
    }
}
