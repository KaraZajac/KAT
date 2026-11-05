//! Timing analysis of a capture's raw level/duration pairs: KAT's port of ProtoPirate's Timing
//! Tuner, plus pulse clustering for captures no decoder recognised.
//!
//! **Protocol comparison** ([`compare_with_protocol`]) mirrors ProtoPirate's
//! `scenes/plugins/protopirate_timing_tuner_plugin.c` (`calculate_timing_stats` + `get_result_line`):
//! every duration (both levels) inside the valid window `te_short − 2·te_delta ..= te_long +
//! 2·te_delta` is "short" below the midpoint `(te_short + te_long) / 2` and "long" from it on; the
//! averages are compared with the definition. A difference of at most 15 µs is EXCELLENT, at most
//! `te_delta` is OK, anything else is HIGH/LOW "by" the difference and the conclusion names the
//! value to set. The overall verdict is "Timing matches fob!" (both EXCELLENT), "Within tolerance."
//! (both OK) or "NEEDS ADJUSTMENT". ProtoPirate measures the last 512 pulses before a decode; KAT
//! uses all of the capture's pairs.
//!
//! One deliberate deviation: ProtoPirate floors the valid window at 100 µs, which discards every
//! short pulse of the sub-100 µs protocols (Ford V1 65 µs, Honda Static 63 µs, BMW 62 µs, Mercedes
//! V1 50 µs) and leaves their analysis empty. KAT floors it at `min(100, te_short / 2)` µs, which
//! is ProtoPirate's 100 µs for every protocol with `te_short` ≥ 200 µs.
//!
//! **Research view** (every capture, and the only view for undecoded ones): HIGH and LOW durations
//! are clustered separately the way rtl_433's pulse analyzer does it (±20 % bins, neighbours
//! fused; count, mean, min, max and jitter = max − min per cluster), the base time element (TE) is
//! estimated per level as the cluster whose integer multiples (1–4) explain most durations, refined
//! by a least-squares fit, and the line coding is guessed from how pulse and gap widths pair up
//! (PWM, PPM, Manchester / bi-phase, NRZ). See docs/timing_analysis.md.

use crate::capture::StoredLevelDuration;
use crate::protocols::ProtocolTiming;

/// Fewer valid samples than this and ProtoPirate reports "Not enough samples".
pub const MIN_SAMPLES: usize = 10;
/// |measured − defined| at or below this is EXCELLENT (ProtoPirate's `abs(diff) <= 15`).
pub const EXACT_US: i64 = 15;
/// ProtoPirate's floor for the valid-pulse window (see the module doc for KAT's lower floor).
const PP_MIN_VALID_US: u32 = 100;
/// Relative width of a duration cluster (rtl_433's pulse analyzer uses ±20 %).
const CLUSTER_TOLERANCE: f64 = 0.2;
/// Share of a level's durations a cluster needs to count as a data width ("major" cluster).
const MAJOR_SHARE: f64 = 0.08;
/// Largest multiple of the TE that still counts as a data width when estimating the TE.
const MAX_TE_MULTIPLE: f64 = 4.0;
/// How far (in TE units) a cluster mean may sit from an integer multiple of the TE.
const MULTIPLE_SLACK: f64 = 0.3;
/// Clusters listed per level in the report before the rest are summarised.
const MAX_LISTED_CLUSTERS: usize = 8;
/// ProtoPirate's ring buffer: the tuner analyses (at most) the last 512 durations before a decode.
pub const PP_BUFFER: usize = 512;
/// Step between the candidate windows searched in an undecoded capture.
const WINDOW_STEP: usize = 128;

// ─── Clustering ──────────────────────────────────────────────────────────────

/// A group of similar durations.
#[derive(Debug, Clone, PartialEq)]
pub struct Cluster {
    pub count: usize,
    /// Mean duration in µs.
    pub mean: f64,
    pub min: u32,
    pub max: u32,
}

impl Cluster {
    /// Spread of the cluster in µs (ProtoPirate's "jitter": max − min).
    pub fn jitter(&self) -> u32 {
        self.max - self.min
    }
}

#[derive(Clone, Copy)]
struct Bin {
    sum: u64,
    count: usize,
    min: u32,
    max: u32,
}

impl Bin {
    fn new(d: u32) -> Self {
        Self { sum: u64::from(d), count: 1, min: d, max: d }
    }

    fn mean(&self) -> f64 {
        self.sum as f64 / self.count as f64
    }

    fn add(&mut self, d: u32) {
        self.sum += u64::from(d);
        self.count += 1;
        self.min = self.min.min(d);
        self.max = self.max.max(d);
    }

    fn merge(&mut self, other: &Bin) {
        self.sum += other.sum;
        self.count += other.count;
        self.min = self.min.min(other.min);
        self.max = self.max.max(other.max);
    }
}

fn within_tolerance(a: f64, b: f64) -> bool {
    (a - b).abs() <= CLUSTER_TOLERANCE * a.max(b)
}

/// Group durations into clusters of similar width, sorted by mean (rtl_433's `histogram_sum` +
/// `histogram_fuse_bins` with ±20 % tolerance; a duration joins the nearest bin within tolerance).
/// Zero durations are ignored.
pub fn cluster_durations(durations: &[u32]) -> Vec<Cluster> {
    let mut bins: Vec<Bin> = Vec::new();
    for &d in durations.iter().filter(|&&d| d > 0) {
        let x = f64::from(d);
        let nearest = bins
            .iter()
            .enumerate()
            .filter(|(_, b)| within_tolerance(b.mean(), x))
            .min_by(|(_, a), (_, b)| (a.mean() - x).abs().total_cmp(&(b.mean() - x).abs()))
            .map(|(i, _)| i);
        match nearest {
            Some(i) => bins[i].add(d),
            None => bins.push(Bin::new(d)),
        }
    }
    // Bin means drift while values are added; fuse neighbours that ended up within tolerance.
    bins.sort_by(|a, b| a.mean().total_cmp(&b.mean()));
    let mut i = 0;
    while i + 1 < bins.len() {
        if within_tolerance(bins[i].mean(), bins[i + 1].mean()) {
            let next = bins.remove(i + 1);
            bins[i].merge(&next);
            i = i.saturating_sub(1);
        } else {
            i += 1;
        }
    }
    bins.into_iter()
        .map(|b| Cluster { count: b.count, mean: b.mean(), min: b.min, max: b.max })
        .collect()
}

/// Clusters holding at least [`MAJOR_SHARE`] of the level's durations (the data widths), by mean.
fn major_clusters(clusters: &[Cluster]) -> Vec<&Cluster> {
    let total: usize = clusters.iter().map(|c| c.count).sum();
    clusters
        .iter()
        .filter(|c| c.count >= 2 && c.count as f64 >= MAJOR_SHARE * total as f64)
        .collect()
}

/// `Some(k)` when `mean` is close to an integer multiple `k` (1..=4) of `te`.
fn multiple_of(mean: f64, te: f64) -> Option<f64> {
    if te <= 0.0 {
        return None;
    }
    let r = mean / te;
    let k = r.round();
    ((1.0..=MAX_TE_MULTIPLE).contains(&k) && (r - k).abs() <= MULTIPLE_SLACK).then_some(k)
}

// ─── TE estimate ─────────────────────────────────────────────────────────────

/// Estimated base time element.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TeEstimate {
    /// Best estimate in µs (mean of the HIGH and LOW estimates when they agree, which cancels the
    /// constant pulse widening/narrowing an AM receiver adds).
    pub te_us: f64,
    /// Estimate from the HIGH durations alone.
    pub high_us: Option<f64>,
    /// Estimate from the LOW durations alone.
    pub low_us: Option<f64>,
    /// Share (0..=1) of all durations whose cluster is a 1–4 multiple of the per-level estimate.
    pub explained: f64,
}

/// TE of one level: the major cluster whose 1..=4 multiples explain the most durations (on a tie the
/// larger unit), refined by a least-squares fit `Σ n·mean·k / Σ n·k²` over the clusters it explains.
/// Returns (TE, durations explained).
fn estimate_level_te(clusters: &[Cluster]) -> Option<(f64, usize)> {
    let mut best: Option<(usize, f64)> = None;
    for cand in major_clusters(clusters) {
        let explained: usize = clusters
            .iter()
            .filter(|c| multiple_of(c.mean, cand.mean).is_some())
            .map(|c| c.count)
            .sum();
        let better = match best {
            None => true,
            Some((e, te)) => explained > e || (explained == e && cand.mean > te),
        };
        if better {
            best = Some((explained, cand.mean));
        }
    }
    let (_, te0) = best?;
    let (mut num, mut den, mut explained) = (0.0, 0.0, 0usize);
    for c in clusters {
        if let Some(k) = multiple_of(c.mean, te0) {
            num += c.count as f64 * c.mean * k;
            den += c.count as f64 * k * k;
            explained += c.count;
        }
    }
    (den > 0.0).then(|| (num / den, explained))
}

/// Estimate the base TE from the per-level clusters.
pub fn estimate_te(high: &[Cluster], low: &[Cluster]) -> Option<TeEstimate> {
    let total: usize = high.iter().chain(low).map(|c| c.count).sum();
    if total == 0 {
        return None;
    }
    let h = estimate_level_te(high);
    let l = estimate_level_te(low);
    let te_us = match (h, l) {
        // Levels agree (within ×1.6): average them, cancelling a constant pulse-width offset.
        (Some((th, _)), Some((tl, _))) if th / tl < 1.6 && tl / th < 1.6 => (th + tl) / 2.0,
        (Some((th, eh)), Some((tl, el))) => {
            if eh >= el {
                th
            } else {
                tl
            }
        }
        (Some((t, _)), None) | (None, Some((t, _))) => t,
        (None, None) => return None,
    };
    let explained = h.map_or(0, |(_, e)| e) + l.map_or(0, |(_, e)| e);
    Some(TeEstimate {
        te_us,
        high_us: h.map(|(t, _)| t),
        low_us: l.map(|(t, _)| t),
        explained: explained as f64 / total as f64,
    })
}

// ─── Line coding guess ───────────────────────────────────────────────────────

/// Guessed line coding of an undecoded capture (research help, not a decode).
#[derive(Debug, Clone, PartialEq)]
pub enum LineCoding {
    /// Two pulse and two gap widths that pair short+long (a long-long pair is rare): PWM. The bit
    /// period is the mean pulse+gap of those pairs.
    Pwm { period_us: f64 },
    /// Each bit is a HIGH+LOW pair of equal length; the pair's length carries the bit (Kia V0 /
    /// Suzuki V0: 250+250 µs = 0, 500+500 µs = 1). Mixed short+long pairs are rare.
    PulseLength,
    /// Two pulse widths, one gap width.
    PwmFixedGap,
    /// One pulse width, the gap width carries the bit (pulse-position / pulse-distance).
    Ppm,
    /// HIGH and LOW both ~1 and ~2 units with frequent long-long pairs: Manchester or bi-phase
    /// (differential Manchester looks the same at the pulse level).
    Manchester,
    /// Three or more widths per level, all multiples of the TE: NRZ / PCM.
    Nrz,
    /// A single pulse width and a single gap width (preamble or carrier bursts only).
    SingleWidth,
    /// Not enough data or no recognisable pattern.
    Unknown,
}

impl LineCoding {
    /// One-line description for the report.
    pub fn describe(&self) -> String {
        match self {
            LineCoding::Pwm { period_us } => format!(
                "PWM (pulse width carries the bit; short+long pairs, bit period ~{:.0} us)",
                period_us
            ),
            LineCoding::PulseLength => {
                "PWM, equal-length HIGH+LOW per bit (the pair length carries the bit)".to_string()
            }
            LineCoding::PwmFixedGap => "PWM with fixed gap (two pulse widths, one gap width)".to_string(),
            LineCoding::Ppm => {
                "PPM / pulse-distance (one pulse width, the gap width carries the bit)".to_string()
            }
            LineCoding::Manchester => {
                "Manchester / bi-phase (HIGH and LOW both 1 TE and 2 TE, long-long pairs occur)"
                    .to_string()
            }
            LineCoding::Nrz => "NRZ / PCM (3+ widths per level, all multiples of the TE)".to_string(),
            LineCoding::SingleWidth => {
                "single width (one pulse and one gap width: preamble or carrier bursts only)".to_string()
            }
            LineCoding::Unknown => "other / unknown (no PWM, PPM or Manchester pattern)".to_string(),
        }
    }

    /// Short label ("PWM", "Manchester", …) for compact output and tests.
    pub fn label(&self) -> &'static str {
        match self {
            LineCoding::Pwm { .. } | LineCoding::PulseLength | LineCoding::PwmFixedGap => "PWM",
            LineCoding::Ppm => "PPM",
            LineCoding::Manchester => "Manchester",
            LineCoding::Nrz => "NRZ",
            LineCoding::SingleWidth => "single width",
            LineCoding::Unknown => "other",
        }
    }
}

/// Index of the data width (0 = short, 1 = long) a duration belongs to: the nearest of the two
/// clusters, if within tolerance of it.
fn width_class(widths: &[&Cluster; 2], d: u32) -> Option<usize> {
    let x = f64::from(d);
    let (i, c) = widths
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| (a.mean - x).abs().total_cmp(&(b.mean - x).abs()))?;
    within_tolerance(c.mean, x).then_some(i)
}

/// The two most populous major clusters of a level, by mean, when they hold ≥ 85 % of the level's
/// major durations (the others are then preamble/sync widths); `None` otherwise.
fn two_data_widths<'a>(majors: &[&'a Cluster]) -> Option<[&'a Cluster; 2]> {
    if majors.len() < 2 {
        return None;
    }
    let mut by_count = majors.to_vec();
    by_count.sort_by_key(|c| std::cmp::Reverse(c.count));
    let total: usize = majors.iter().map(|c| c.count).sum();
    let (a, b) = (by_count[0], by_count[1]);
    if ((a.count + b.count) as f64) < 0.85 * total as f64 {
        return None;
    }
    Some(if a.mean <= b.mean { [a, b] } else { [b, a] })
}

/// Runs of this many identical consecutive pair classes are treated as preamble and skipped.
const PREAMBLE_RUN: usize = 8;

/// Short/long class counts of the pulse pairs for one pairing order (see [`pair_stats`]).
#[derive(Debug, Clone, Copy, Default)]
struct PairStats {
    short_short: usize,
    /// One short and one long width.
    mixed: usize,
    long_long: usize,
    /// Summed pulse+gap length of the mixed pairs (the PWM bit period).
    mixed_period_us: f64,
}

impl PairStats {
    fn total(&self) -> usize {
        self.short_short + self.mixed + self.long_long
    }

    /// Share of long-long pairs among the pairs that contain a long width (≈0 for PWM).
    fn long_long_share(&self) -> Option<f64> {
        let n = self.mixed + self.long_long;
        (n > 0).then(|| self.long_long as f64 / n as f64)
    }

    /// Share of mixed pairs among all pairs (≈0 for equal-length pulse pairs).
    fn mixed_share(&self) -> Option<f64> {
        let n = self.total();
        (n > 0).then(|| self.mixed as f64 / n as f64)
    }
}

/// Pair statistics for one pairing order. `first_high` pairs every HIGH with the LOW after it,
/// otherwise every LOW with the HIGH after it (an inverted capture pairs PWM bits correctly only that
/// way round). Runs of ≥ [`PREAMBLE_RUN`] identical classes (preambles such as 1600/1600 µs or
/// 250/250 µs) are skipped so they do not count as data.
fn pair_stats(
    pairs: &[StoredLevelDuration],
    high: &[&Cluster; 2],
    low: &[&Cluster; 2],
    first_high: bool,
) -> PairStats {
    let mut classes: Vec<Option<((usize, usize), u64)>> = Vec::new();
    for w in pairs.windows(2) {
        if w[0].level != first_high || w[1].level == first_high {
            continue;
        }
        let (a, b) = if first_high { (high, low) } else { (low, high) };
        let class = width_class(a, w[0].duration_us).zip(width_class(b, w[1].duration_us));
        let period = u64::from(w[0].duration_us) + u64::from(w[1].duration_us);
        classes.push(class.map(|c| (c, period)));
    }
    let mut stats = PairStats::default();
    let mut i = 0;
    while i < classes.len() {
        let mut j = i + 1;
        while j < classes.len() && classes[j].map(|c| c.0) == classes[i].map(|c| c.0) {
            j += 1;
        }
        if j - i < PREAMBLE_RUN {
            for c in classes[i..j].iter().flatten() {
                match c.0 {
                    (0, 0) => stats.short_short += 1,
                    (1, 1) => stats.long_long += 1,
                    _ => {
                        stats.mixed += 1;
                        stats.mixed_period_us += c.1 as f64;
                    }
                }
            }
        }
        i = j;
    }
    stats
}

/// Guess the line coding from the per-level clusters and how pulses and the following gaps pair up.
pub fn guess_line_coding(
    pairs: &[StoredLevelDuration],
    high: &[Cluster],
    low: &[Cluster],
    te: Option<&TeEstimate>,
) -> LineCoding {
    let major_h = major_clusters(high);
    let major_l = major_clusters(low);
    if major_h.is_empty() || major_l.is_empty() {
        return LineCoding::Unknown;
    }
    let data_h = two_data_widths(&major_h);
    let data_l = two_data_widths(&major_l);
    match (data_h, data_l) {
        (Some(h), Some(l)) => {
            // Pair every pulse with the following gap (and, for inverted captures, every gap with
            // the following pulse) and classify each width as short/long:
            // - PWM: one short and one long width per bit, so long-long pairs are rare in the right
            //   pairing order;
            // - equal-length pairs (Kia V0): short-short or long-long, mixed pairs are rare;
            // - Manchester / bi-phase: mixed and long-long pairs in both orders.
            let orders = [pair_stats(pairs, &h, &l, true), pair_stats(pairs, &h, &l, false)];
            if orders.iter().all(|o| o.total() == 0) {
                return LineCoding::Unknown;
            }
            let pwm = orders
                .iter()
                .filter(|o| o.mixed > 0)
                .filter_map(|o| o.long_long_share().map(|share| (share, o)))
                .min_by(|a, b| a.0.total_cmp(&b.0));
            if let Some((share, o)) = pwm {
                if share <= 0.05 {
                    return LineCoding::Pwm { period_us: o.mixed_period_us / o.mixed as f64 };
                }
            }
            let equal_pairs = orders.iter().any(|o| {
                o.short_short > 0 && o.long_long > 0 && o.mixed_share().is_some_and(|m| m <= 0.05)
            });
            if equal_pairs {
                return LineCoding::PulseLength;
            }
            let one_two = |r: f64| (1.6..=2.5).contains(&r);
            let manchester = orders.iter().all(|o| {
                o.long_long_share().is_some_and(|s| s >= 0.10)
                    && o.mixed_share().is_some_and(|m| m >= 0.10)
            });
            if manchester && one_two(h[1].mean / h[0].mean) && one_two(l[1].mean / l[0].mean) {
                LineCoding::Manchester
            } else {
                LineCoding::Unknown
            }
        }
        (Some(_), None) if major_l.len() == 1 => LineCoding::PwmFixedGap,
        (None, Some(_)) if major_h.len() == 1 => LineCoding::Ppm,
        (None, None) if major_h.len() == 1 && major_l.len() == 1 => LineCoding::SingleWidth,
        _ => {
            let Some(te) = te else {
                return LineCoding::Unknown;
            };
            let fits = |c: &&Cluster, unit: f64| {
                let r = c.mean / unit;
                r.round() >= 1.0 && (r - r.round()).abs() <= MULTIPLE_SLACK
            };
            let unit_h = te.high_us.unwrap_or(te.te_us);
            let unit_l = te.low_us.unwrap_or(te.te_us);
            let all_multiples =
                major_h.iter().all(|c| fits(c, unit_h)) && major_l.iter().all(|c| fits(c, unit_l));
            if all_multiples && (major_h.len() >= 3 || major_l.len() >= 3) {
                LineCoding::Nrz
            } else {
                LineCoding::Unknown
            }
        }
    }
}

// ─── ProtoPirate comparison ──────────────────────────────────────────────────

/// ProtoPirate's statistics of one width class (short or long).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct WidthStats {
    pub count: usize,
    /// Integer average (ProtoPirate divides `int64` sums by the count).
    pub avg: i64,
    pub min: u32,
    pub max: u32,
}

impl WidthStats {
    fn add(&mut self, d: u32, sum: &mut i64) {
        if self.count == 0 {
            self.min = d;
            self.max = d;
        } else {
            self.min = self.min.min(d);
            self.max = self.max.max(d);
        }
        self.count += 1;
        *sum += i64::from(d);
    }

    /// ProtoPirate's jitter: max − min (0 without samples).
    pub fn jitter(&self) -> u32 {
        self.max - self.min
    }
}

/// Per-class conclusion (ProtoPirate's "Short: …" / "Long: …" lines).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WidthVerdict {
    /// |diff| ≤ 15 µs.
    Excellent,
    /// |diff| ≤ te_delta.
    Ok(i64),
    /// Measured longer than defined by this many µs.
    High(i64),
    /// Measured shorter than defined by this many µs.
    Low(i64),
    /// No duration fell in this class.
    NoSamples,
}

/// Overall conclusion (ProtoPirate's verdict line).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimingVerdict {
    /// Both classes EXCELLENT: "Timing matches fob!"
    Matches,
    /// Both classes within te_delta: "Within tolerance."
    WithinTolerance,
    /// "NEEDS ADJUSTMENT"
    NeedsAdjustment,
    /// Fewer than [`MIN_SAMPLES`] durations in the valid window.
    NotEnoughSamples,
    /// One class has no durations at all (e.g. Honda Static's `te_long` is its 700 µs sync, which
    /// the decoded frame does not contain), so that width cannot be checked. ProtoPirate would print
    /// "NEEDS ADJUSTMENT" and "Set te_long=0" here.
    Incomplete,
}

/// Received timing compared with a protocol definition, as ProtoPirate's Timing Tuner does it.
#[derive(Debug, Clone, PartialEq)]
pub struct ProtocolComparison {
    pub te_short: u32,
    pub te_long: u32,
    pub te_delta: u32,
    /// Short/long split: `(te_short + te_long) / 2`.
    pub threshold: u32,
    /// Valid window (durations outside it are noise or gaps and are ignored).
    pub min_valid: u32,
    pub max_valid: u32,
    pub short: WidthStats,
    pub long: WidthStats,
    /// Durations outside the valid window.
    pub ignored: usize,
    pub short_diff: i64,
    pub long_diff: i64,
    pub short_ok: bool,
    pub long_ok: bool,
    pub short_exact: bool,
    pub long_exact: bool,
}

/// Compare a capture's durations with a protocol's timing definition (ProtoPirate's
/// `calculate_timing_stats`, both levels pooled; see the module doc for the valid-window floor).
pub fn compare_with_protocol(pairs: &[StoredLevelDuration], timing: &ProtocolTiming) -> ProtocolComparison {
    let te_short = i64::from(timing.te_short);
    let te_long = i64::from(timing.te_long);
    let te_delta = i64::from(timing.te_delta);
    let threshold = (te_short + te_long) / 2;
    let floor = i64::from(PP_MIN_VALID_US.min(timing.te_short / 2));
    let min_valid = (te_short - 2 * te_delta).max(floor);
    let max_valid = te_long + 2 * te_delta;

    let (mut short, mut long) = (WidthStats::default(), WidthStats::default());
    let (mut short_sum, mut long_sum) = (0i64, 0i64);
    let mut ignored = 0usize;
    for p in pairs {
        let d = i64::from(p.duration_us);
        if d < min_valid || d > max_valid {
            ignored += 1;
        } else if d < threshold {
            short.add(p.duration_us, &mut short_sum);
        } else {
            long.add(p.duration_us, &mut long_sum);
        }
    }
    if short.count > 0 {
        short.avg = short_sum / short.count as i64;
    }
    if long.count > 0 {
        long.avg = long_sum / long.count as i64;
    }
    let short_diff = short.avg - te_short;
    let long_diff = long.avg - te_long;
    ProtocolComparison {
        te_short: timing.te_short,
        te_long: timing.te_long,
        te_delta: timing.te_delta,
        threshold: threshold as u32,
        min_valid: min_valid as u32,
        max_valid: max_valid as u32,
        short,
        long,
        ignored,
        short_diff,
        long_diff,
        short_ok: short.count > 0 && short_diff.abs() <= te_delta,
        long_ok: long.count > 0 && long_diff.abs() <= te_delta,
        short_exact: short.count > 0 && short_diff.abs() <= EXACT_US,
        long_exact: long.count > 0 && long_diff.abs() <= EXACT_US,
    }
}

impl ProtocolComparison {
    fn width_verdict(count: usize, diff: i64, exact: bool, ok: bool) -> WidthVerdict {
        if count == 0 {
            WidthVerdict::NoSamples
        } else if exact {
            WidthVerdict::Excellent
        } else if ok {
            WidthVerdict::Ok(diff)
        } else if diff > 0 {
            WidthVerdict::High(diff)
        } else {
            WidthVerdict::Low(-diff)
        }
    }

    pub fn short_verdict(&self) -> WidthVerdict {
        Self::width_verdict(self.short.count, self.short_diff, self.short_exact, self.short_ok)
    }

    pub fn long_verdict(&self) -> WidthVerdict {
        Self::width_verdict(self.long.count, self.long_diff, self.long_exact, self.long_ok)
    }

    /// Overall verdict.
    pub fn verdict(&self) -> TimingVerdict {
        if self.short.count + self.long.count < MIN_SAMPLES {
            TimingVerdict::NotEnoughSamples
        } else if self.short.count == 0 || self.long.count == 0 {
            TimingVerdict::Incomplete
        } else if self.short_exact && self.long_exact {
            TimingVerdict::Matches
        } else if self.short_ok && self.long_ok {
            TimingVerdict::WithinTolerance
        } else {
            TimingVerdict::NeedsAdjustment
        }
    }

    /// ProtoPirate's conclusion text: verdict line plus the recommendation line(s).
    pub fn conclusion_lines(&self) -> Vec<String> {
        let mut out = Vec::new();
        match self.verdict() {
            TimingVerdict::NotEnoughSamples => {
                out.push(format!(
                    "Not enough samples ({} in the valid window, need {}).",
                    self.short.count + self.long.count,
                    MIN_SAMPLES
                ));
            }
            TimingVerdict::Incomplete => {
                let (class, range) = if self.short.count == 0 {
                    ("short", format!("< {} us", self.threshold))
                } else {
                    ("long", format!(">= {} us", self.threshold))
                };
                out.push(format!(
                    "INCOMPLETE: no {} durations ({}) in the analysed pairs,",
                    class, range
                ));
                out.push(format!("so te_{} cannot be checked here.", class));
            }
            TimingVerdict::Matches => {
                out.push("Timing matches fob!".to_string());
                out.push("No changes needed.".to_string());
            }
            TimingVerdict::WithinTolerance => {
                out.push("Within tolerance.".to_string());
                out.push("Consider fine-tuning.".to_string());
            }
            TimingVerdict::NeedsAdjustment => {
                out.push("NEEDS ADJUSTMENT:".to_string());
                let set_short = !self.short_ok && self.short.count > 0;
                let set_long = !self.long_ok && self.long.count > 0;
                if set_short {
                    out.push(format!("Set te_short={}", self.short.avg));
                }
                if set_long {
                    out.push(format!("Set te_long={}", self.long.avg));
                }
            }
        }
        out
    }
}

fn verdict_text(label: &str, v: WidthVerdict) -> String {
    match v {
        WidthVerdict::Excellent => format!("{}: EXCELLENT", label),
        WidthVerdict::Ok(d) => format!("{}: OK ({:+})", label, d),
        WidthVerdict::High(d) => format!("{}: HIGH by {}", label, d),
        WidthVerdict::Low(d) => format!("{}: LOW by {}", label, d),
        WidthVerdict::NoSamples => format!("{}: NO SAMPLES", label),
    }
}

// ─── Full analysis + report ──────────────────────────────────────────────────

/// Which part of the capture was analysed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AnalysisWindow {
    /// The whole capture.
    All,
    /// Decoded capture: its last `2·min_count_bit + 32` pairs (at most ProtoPirate's 512), i.e.
    /// the decoded frame. A decode completes on the capture's last pair for imports (each decoded
    /// segment ends where its frame did); live bursts end with repeats of the frame. Noise received
    /// before the frame (a .sub file's first segment can be thousands of noise pulses) stays out.
    DecodedFrame,
    /// Undecoded capture: the most regular 512-pair window (highest share of durations that are
    /// 1–4 multiples of one TE, with HIGH and LOW agreeing on it).
    MostRegular,
}

/// Everything the timing overlay shows for one capture.
#[derive(Debug, Clone, PartialEq)]
pub struct TimingReport {
    /// Protocol name the comparison is against (the capture's decoded protocol), if any.
    pub protocol: Option<String>,
    /// Pairs in the whole capture.
    pub capture_pairs: usize,
    /// Analysed part: `window_start .. window_start + pair_count`.
    pub window: AnalysisWindow,
    pub window_start: usize,
    pub pair_count: usize,
    pub high_count: usize,
    pub low_count: usize,
    /// HIGH-level duration clusters, by mean.
    pub high: Vec<Cluster>,
    /// LOW-level duration clusters, by mean.
    pub low: Vec<Cluster>,
    pub te: Option<TeEstimate>,
    pub coding: LineCoding,
    /// ProtoPirate-style comparison when the protocol's timing definition is known.
    pub comparison: Option<ProtocolComparison>,
}

fn split_levels(pairs: &[StoredLevelDuration]) -> (Vec<u32>, Vec<u32>) {
    let highs = pairs.iter().filter(|p| p.level).map(|p| p.duration_us).collect();
    let lows = pairs.iter().filter(|p| !p.level).map(|p| p.duration_us).collect();
    (highs, lows)
}

/// Regularity score of a window (0..=1): share of durations explained by the TE estimate, scaled by
/// how well the HIGH and LOW estimates agree.
fn regularity(pairs: &[StoredLevelDuration]) -> f64 {
    let (highs, lows) = split_levels(pairs);
    let Some(te) = estimate_te(&cluster_durations(&highs), &cluster_durations(&lows)) else {
        return 0.0;
    };
    let agreement = match (te.high_us, te.low_us) {
        (Some(h), Some(l)) => h.min(l) / h.max(l),
        _ => 0.5,
    };
    te.explained * agreement
}

/// Start of the most regular [`PP_BUFFER`]-pair window (stepping [`WINDOW_STEP`], plus the final
/// window); the earliest wins a tie.
fn most_regular_window(pairs: &[StoredLevelDuration]) -> usize {
    let last = pairs.len().saturating_sub(PP_BUFFER);
    let mut starts: Vec<usize> = (0..=last).step_by(WINDOW_STEP).collect();
    if starts.last() != Some(&last) {
        starts.push(last);
    }
    let mut best = (0usize, -1.0f64);
    for start in starts {
        let score = regularity(&pairs[start..start + PP_BUFFER.min(pairs.len())]);
        if score > best.1 {
            best = (start, score);
        }
    }
    best.0
}

/// Analyse a capture's raw pairs. `protocol` is the decoded protocol name and `timing` its timing
/// definition (`ProtocolDecoder::timing()`), when known; see [`AnalysisWindow`] for which pairs
/// are used.
/// [analyze] with the decoded frame known to end at pair `frame_end` (exclusive), e.g. where the
/// protocol's decoder completed it when the capture is decoded again: pairs received after the
/// frame (the rest of a live burst) stay out of the window.
pub fn analyze_frame_ending_at(
    pairs: &[StoredLevelDuration],
    frame_end: usize,
    protocol: Option<&str>,
    timing: Option<&ProtocolTiming>,
) -> TimingReport {
    let mut report = analyze(&pairs[..frame_end.min(pairs.len())], protocol, timing);
    report.capture_pairs = pairs.len();
    report
}

pub fn analyze(
    pairs: &[StoredLevelDuration],
    protocol: Option<&str>,
    timing: Option<&ProtocolTiming>,
) -> TimingReport {
    let (window, start, len) = if protocol.is_some() {
        let frame = timing.map_or(PP_BUFFER, |t| (2 * t.min_count_bit + 32).min(PP_BUFFER));
        if pairs.len() <= frame {
            (AnalysisWindow::All, 0, pairs.len())
        } else {
            (AnalysisWindow::DecodedFrame, pairs.len() - frame, frame)
        }
    } else if pairs.len() <= PP_BUFFER {
        (AnalysisWindow::All, 0, pairs.len())
    } else {
        (AnalysisWindow::MostRegular, most_regular_window(pairs), PP_BUFFER)
    };
    let analysed = &pairs[start..start + len];
    let (highs, lows) = split_levels(analysed);
    let high = cluster_durations(&highs);
    let low = cluster_durations(&lows);
    let te = estimate_te(&high, &low);
    let coding = guess_line_coding(analysed, &high, &low, te.as_ref());
    TimingReport {
        protocol: protocol.map(str::to_string),
        capture_pairs: pairs.len(),
        window,
        window_start: start,
        pair_count: len,
        high_count: highs.len(),
        low_count: lows.len(),
        high,
        low,
        te,
        coding,
        comparison: timing.map(|t| compare_with_protocol(analysed, t)),
    }
}

impl TimingReport {
    /// Suggested `te_short` / `te_long` for a new decoder (no protocol reference): the TE and the
    /// smallest major multiple ≥ 2 of it.
    pub fn suggested_definition(&self) -> Option<(u32, Option<u32>)> {
        let te = self.te?;
        let long_k = major_clusters(&self.high)
            .into_iter()
            .chain(major_clusters(&self.low))
            .filter_map(|c| multiple_of(c.mean, te.te_us))
            .filter(|&k| k >= 2.0)
            .fold(None, |acc: Option<f64>, k| Some(acc.map_or(k, |a| a.min(k))));
        let te_short = te.te_us.round() as u32;
        Some((te_short, long_k.map(|k| (k * te.te_us).round() as u32)))
    }

    /// The report as text lines (ProtoPirate's sections first when a protocol reference exists).
    pub fn render_lines(&self) -> Vec<String> {
        let mut out = Vec::new();
        let first = self.window_start + 1;
        let last = self.window_start + self.pair_count;
        out.push(match self.window {
            AnalysisWindow::All => format!(
                "Analysed all {} pairs ({} HIGH, {} LOW)",
                self.pair_count, self.high_count, self.low_count
            ),
            AnalysisWindow::DecodedFrame => format!(
                "Analysed pairs {}..{} of {} ({} HIGH, {} LOW): the decoded frame",
                first, last, self.capture_pairs, self.high_count, self.low_count
            ),
            AnalysisWindow::MostRegular => format!(
                "Analysed pairs {}..{} of {} ({} HIGH, {} LOW): the most regular {}-pair window",
                first, last, self.capture_pairs, self.high_count, self.low_count, PP_BUFFER
            ),
        });
        out.push(String::new());

        match (&self.comparison, &self.protocol) {
            (Some(cmp), proto) => {
                out.push(format!("PROTOCOL DEFINITION ({}):", proto.as_deref().unwrap_or("?")));
                out.push(format!("  Short: {} us", cmp.te_short));
                out.push(format!("  Long: {} us", cmp.te_long));
                out.push(format!("  Tolerance: +/-{} us", cmp.te_delta));
                out.push(String::new());
                out.push("RECEIVED SIGNAL:".to_string());
                out.push(format!(
                    "  (both levels; short < {} us <= long; valid {}..{} us; {} ignored)",
                    cmp.threshold, cmp.min_valid, cmp.max_valid, cmp.ignored
                ));
                for (label, s) in [("Short", &cmp.short), ("Long", &cmp.long)] {
                    out.push(format!(
                        "  {:<5} Avg: {} us  Min: {} us  Max: {} us  Samples: {}",
                        label, s.avg, s.min, s.max, s.count
                    ));
                }
                out.push(String::new());
                out.push("ANALYSIS:".to_string());
                out.push(format!("  Short Diff: {:+} us", cmp.short_diff));
                out.push(format!("  Long Diff: {:+} us", cmp.long_diff));
                out.push(format!("  Short Jitter: {} us", cmp.short.jitter()));
                out.push(format!("  Long Jitter: {} us", cmp.long.jitter()));
                out.push(String::new());
                out.push("CONCLUSION:".to_string());
                out.push(format!("  {}", verdict_text("Short", cmp.short_verdict())));
                out.push(format!("  {}", verdict_text("Long", cmp.long_verdict())));
                for line in cmp.conclusion_lines() {
                    out.push(format!("  {}", line));
                }
            }
            (None, Some(proto)) => {
                out.push(format!("NO PROTOCOL REFERENCE (no timing definition for {})", proto));
            }
            (None, None) => {
                out.push("NO PROTOCOL REFERENCE (undecoded capture)".to_string());
            }
        }

        out.push(String::new());
        out.push("PULSE CLUSTERS (per level; jitter = max - min):".to_string());
        let te_us = self.te.map(|t| t.te_us);
        for (label, clusters, level_te) in [
            ("HIGH", &self.high, self.te.and_then(|t| t.high_us)),
            ("LOW", &self.low, self.te.and_then(|t| t.low_us)),
        ] {
            render_clusters(&mut out, label, clusters, level_te.or(te_us));
        }
        out.push(String::new());
        match self.te {
            Some(te) => {
                let mut line = format!("Base TE estimate: {:.0} us", te.te_us);
                match (te.high_us, te.low_us) {
                    (Some(h), Some(l)) => {
                        line.push_str(&format!(" (HIGH {:.0} us, LOW {:.0} us", h, l));
                        let offset = h - l;
                        if offset.abs() >= 20.0 && (h / l) < 1.6 && (l / h) < 1.6 {
                            line.push_str(&format!(
                                "; HIGH widths {} by ~{:.0} us",
                                if offset > 0.0 { "stretched" } else { "shortened" },
                                offset.abs() / 2.0
                            ));
                        }
                        line.push(')');
                    }
                    (Some(h), None) => line.push_str(&format!(" (HIGH {:.0} us)", h)),
                    (None, Some(l)) => line.push_str(&format!(" (LOW {:.0} us)", l)),
                    (None, None) => {}
                }
                out.push(line);
                out.push(format!(
                    "  {:.0}% of durations are 1-4 x TE",
                    te.explained * 100.0
                ));
            }
            None => out.push("Base TE estimate: none (no dominant width)".to_string()),
        }
        out.push(format!("Line coding guess: {}", self.coding.describe()));
        if self.comparison.is_none() {
            if let Some((short, long)) = self.suggested_definition() {
                let mut line = format!("Suggested definition: te_short ~{} us", short);
                if let Some(long) = long {
                    line.push_str(&format!(", te_long ~{} us", long));
                }
                out.push(line);
            }
        }
        out
    }
}

/// Append one level's clusters: major ones in full (with their TE multiple), then a summary of the
/// rest (noise, sync and inter-frame gaps).
fn render_clusters(out: &mut Vec<String>, label: &str, clusters: &[Cluster], te: Option<f64>) {
    if clusters.is_empty() {
        out.push(format!("  {:<4}  (none)", label));
        return;
    }
    let total: usize = clusters.iter().map(|c| c.count).sum();
    let majors = major_clusters(clusters);
    let mut listed: Vec<&Cluster> = majors.clone();
    // Fill up with the most frequent minor clusters (sync widths, gaps).
    let mut minors: Vec<&Cluster> = clusters.iter().filter(|c| !majors.contains(c)).collect();
    minors.sort_by(|a, b| b.count.cmp(&a.count).then(a.mean.total_cmp(&b.mean)));
    let room = MAX_LISTED_CLUSTERS.saturating_sub(listed.len());
    listed.extend(minors.iter().take(room).copied());
    listed.sort_by(|a, b| a.mean.total_cmp(&b.mean));
    for (i, c) in listed.iter().enumerate() {
        let multiple = te
            .and_then(|t| {
                let r = c.mean / t;
                (r >= 0.7).then(|| {
                    if (r - r.round()).abs() <= MULTIPLE_SLACK {
                        format!("{:.0} TE", r.round())
                    } else {
                        format!("{:.1} TE", r)
                    }
                })
            })
            .unwrap_or_default();
        let tag = if majors.contains(c) { "" } else { "  (minor)" };
        out.push(format!(
            "  {:<4}  {:>6.0} us  x{:<5} min {:>6}  max {:>6}  jitter {:>5}  {:>6}{}",
            if i == 0 { label } else { "" },
            c.mean,
            c.count,
            c.min,
            c.max,
            c.jitter(),
            multiple,
            tag
        ));
    }
    let rest = clusters.len() - listed.len();
    if rest > 0 {
        let rest_count: usize = total - listed.iter().map(|c| c.count).sum::<usize>();
        out.push(format!(
            "        + {} more width(s) ({} durations: noise, sync or gaps)",
            rest, rest_count
        ));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pair(level: bool, d: u32) -> StoredLevelDuration {
        StoredLevelDuration { level, duration_us: d }
    }

    /// Small deterministic PRNG (xorshift32) for reproducible jitter and bits.
    struct Rng(u32);
    impl Rng {
        fn next(&mut self) -> u32 {
            let mut x = self.0;
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            self.0 = x;
            x
        }
        fn bit(&mut self) -> bool {
            self.next() & 1 == 1
        }
        /// `d` ± up to `pct` percent.
        fn jitter(&mut self, d: u32, pct: u32) -> u32 {
            let span = d * pct / 100;
            if span == 0 {
                return d;
            }
            d - span + self.next() % (2 * span + 1)
        }
    }

    /// PWM 1:2 frame: bit 1 = long HIGH + short LOW, bit 0 = short HIGH + long LOW, behind a
    /// preamble of short/short pairs and a long header gap.
    fn pwm_train(te: u32, bits: usize, jitter_pct: u32, seed: u32) -> Vec<StoredLevelDuration> {
        let mut rng = Rng(seed);
        let mut v = vec![pair(false, 20_000)];
        for _ in 0..12 {
            v.push(pair(true, rng.jitter(te, jitter_pct)));
            v.push(pair(false, rng.jitter(te, jitter_pct)));
        }
        v.push(pair(true, rng.jitter(te, jitter_pct)));
        v.push(pair(false, 10 * te));
        for _ in 0..bits {
            let (h, l) = if rng.bit() { (2 * te, te) } else { (te, 2 * te) };
            v.push(pair(true, rng.jitter(h, jitter_pct)));
            v.push(pair(false, rng.jitter(l, jitter_pct)));
        }
        v.push(pair(false, 20_000));
        v
    }

    /// Manchester frame (half-bit cells of `te`, runs merged into 2·te pulses).
    fn manchester_train(te: u32, bits: usize, jitter_pct: u32, seed: u32) -> Vec<StoredLevelDuration> {
        let mut rng = Rng(seed);
        let mut halves: Vec<bool> = Vec::new();
        for _ in 0..16 {
            halves.extend([true, false]); // preamble of 1s
        }
        for _ in 0..bits {
            if rng.bit() {
                halves.extend([true, false]);
            } else {
                halves.extend([false, true]);
            }
        }
        let mut v = vec![pair(false, 20_000)];
        let mut i = 0;
        while i < halves.len() {
            let level = halves[i];
            let mut n = 1;
            while i + n < halves.len() && halves[i + n] == level {
                n += 1;
            }
            v.push(pair(level, rng.jitter(n as u32 * te, jitter_pct)));
            i += n;
        }
        v.push(pair(false, 20_000));
        v
    }

    fn timing(te_short: u32, te_long: u32, te_delta: u32) -> ProtocolTiming {
        ProtocolTiming { te_short, te_long, te_delta, min_count_bit: 64 }
    }

    #[test]
    fn clusters_separate_one_and_two_te() {
        let d = [250, 260, 240, 500, 510, 490, 255, 245, 505];
        let c = cluster_durations(&d);
        assert_eq!(c.len(), 2, "{c:?}");
        assert_eq!(c[0].count, 5);
        assert_eq!((c[0].min, c[0].max), (240, 260));
        assert_eq!(c[0].jitter(), 20);
        assert_eq!(c[1].count, 4);
        assert!((c[1].mean - 501.25).abs() < 1e-9);
        assert!(cluster_durations(&[]).is_empty());
        assert!(cluster_durations(&[0, 0]).is_empty());
    }

    #[test]
    fn pwm_train_is_pwm_with_te_400() {
        let pairs = pwm_train(400, 66, 10, 0x1234_5678);
        let r = analyze(&pairs, None, None);
        let te = r.te.expect("TE");
        assert!((te.te_us - 400.0).abs() < 15.0, "TE {:.1}", te.te_us);
        assert!(matches!(r.coding, LineCoding::Pwm { .. }), "{:?}", r.coding);
        if let LineCoding::Pwm { period_us } = r.coding {
            assert!((period_us - 1200.0).abs() < 40.0, "period {period_us}");
        }
        assert_eq!(r.suggested_definition().map(|(s, _)| (s as i64 - 400).abs() < 15), Some(true));
        assert_eq!(r.suggested_definition().and_then(|(_, l)| l).map(|l| (l as i64 - 800).abs() < 30), Some(true));
    }

    #[test]
    fn inverted_pwm_and_long_preambles_stay_pwm() {
        // Inverted polarity (carrier recorded as LOW): HIGH→LOW pairing would pair the halves of
        // adjacent bits; the LOW→HIGH order recovers the PWM pattern.
        let inverted: Vec<_> = pwm_train(400, 66, 8, 21).iter().map(|p| pair(!p.level, p.duration_us)).collect();
        assert!(matches!(analyze(&inverted, None, None).coding, LineCoding::Pwm { .. }));
        // Subaru-style preamble of long/long halves (800/1600 µs PWM behind 1600/1600 µs pairs).
        let mut rng = Rng(77);
        let mut v = vec![pair(false, 20_000)];
        for _ in 0..12 {
            v.push(pair(true, rng.jitter(1600, 3)));
            v.push(pair(false, rng.jitter(1600, 3)));
        }
        v.push(pair(true, 2800));
        v.push(pair(false, 2800));
        for _ in 0..64 {
            let (h, l) = if rng.bit() { (1600, 800) } else { (800, 1600) };
            v.push(pair(true, rng.jitter(h, 3)));
            v.push(pair(false, rng.jitter(l, 3)));
        }
        let r = analyze(&v, None, None);
        assert!(matches!(r.coding, LineCoding::Pwm { .. }), "{:?}", r.coding);
        assert!((r.te.unwrap().te_us - 800.0).abs() < 20.0);
    }

    #[test]
    fn equal_length_pairs_are_pulse_length_pwm() {
        // Kia V0 / Suzuki V0 air format: 250+250 µs = 0, 500+500 µs = 1 behind a short-pair preamble.
        let mut rng = Rng(5);
        let mut v = vec![pair(false, 20_000)];
        for _ in 0..40 {
            v.push(pair(true, rng.jitter(250, 6)));
            v.push(pair(false, rng.jitter(250, 6)));
        }
        for _ in 0..61 {
            let d = if rng.bit() { 500 } else { 250 };
            v.push(pair(true, rng.jitter(d, 6)));
            v.push(pair(false, rng.jitter(d, 6)));
        }
        v.push(pair(true, 1_200));
        let r = analyze(&v, None, None);
        assert_eq!(r.coding, LineCoding::PulseLength);
        assert_eq!(r.coding.label(), "PWM");
    }

    #[test]
    fn manchester_train_is_manchester_with_te_250() {
        let pairs = manchester_train(250, 80, 8, 0xC0FF_EE11);
        let r = analyze(&pairs, None, None);
        let te = r.te.expect("TE");
        assert!((te.te_us - 250.0).abs() < 10.0, "TE {:.1}", te.te_us);
        assert_eq!(r.coding, LineCoding::Manchester);
        // HIGH has exactly the 1 TE and 2 TE data widths (+ nothing else major).
        let majors = major_clusters(&r.high);
        assert_eq!(majors.len(), 2, "{:?}", r.high);
    }

    #[test]
    fn am_pulse_stretching_cancels_in_the_te_estimate() {
        // HIGH +60 µs, LOW −60 µs (an AM envelope detector widens pulses): the per-level estimates
        // are off by ±~60 µs, their mean is the true TE.
        let pairs: Vec<StoredLevelDuration> = manchester_train(500, 64, 5, 7)
            .into_iter()
            .map(|p| {
                if p.duration_us >= 5_000 {
                    p
                } else if p.level {
                    pair(true, p.duration_us + 60)
                } else {
                    pair(false, p.duration_us - 60)
                }
            })
            .collect();
        let te = analyze(&pairs, None, None).te.expect("TE");
        assert!((te.te_us - 500.0).abs() < 12.0, "TE {:.1}", te.te_us);
        assert!(te.high_us.unwrap() > te.low_us.unwrap() + 60.0);
    }

    #[test]
    fn ppm_and_fixed_gap_pwm_are_recognised() {
        let mut rng = Rng(99);
        // GM-style PPM: fixed 300 µs pulse, 300/600 µs gaps.
        let mut ppm = vec![pair(false, 15_000)];
        for _ in 0..112 {
            ppm.push(pair(true, rng.jitter(300, 5)));
            let gap = if rng.bit() { 600 } else { 300 };
            ppm.push(pair(false, rng.jitter(gap, 5)));
        }
        assert_eq!(analyze(&ppm, None, None).coding, LineCoding::Ppm);
        // Fixed gap: 400/1200 µs pulses, 400 µs gaps.
        let mut fixed_gap = vec![pair(false, 15_000)];
        for _ in 0..64 {
            let pulse = if rng.bit() { 1200 } else { 400 };
            fixed_gap.push(pair(true, rng.jitter(pulse, 5)));
            fixed_gap.push(pair(false, rng.jitter(400, 5)));
        }
        assert_eq!(analyze(&fixed_gap, None, None).coding, LineCoding::PwmFixedGap);
    }

    #[test]
    fn glitches_and_gaps_do_not_move_the_te() {
        let mut pairs = manchester_train(250, 80, 8, 3);
        // Sprinkle 40 µs glitches (≈3 % of the durations) and a few long gaps.
        let mut i = 7;
        while i < pairs.len() {
            pairs.insert(i, pair(pairs[i - 1].level, 40));
            i += 37;
        }
        pairs.extend([pair(true, 3_000), pair(false, 12_000), pair(true, 3_000)]);
        let r = analyze(&pairs, None, None);
        let te = r.te.expect("TE");
        assert!((te.te_us - 250.0).abs() < 12.0, "TE {:.1}", te.te_us);
        let text = r.render_lines().join("\n");
        assert!(text.contains("(minor)"), "{text}");
    }

    #[test]
    fn protopirate_rules_excellent_ok_high_low() {
        // Shorts at 262 µs (+12), longs at 492 µs (−8) vs 250/500/100 → both EXCELLENT.
        let mut pairs = Vec::new();
        for i in 0..40 {
            pairs.push(pair(i % 2 == 0, 262));
            pairs.push(pair(i % 2 == 1, 492));
        }
        pairs.push(pair(false, 30_000)); // gap: outside the valid window
        let c = compare_with_protocol(&pairs, &timing(250, 500, 100));
        assert_eq!(c.threshold, 375);
        assert_eq!((c.min_valid, c.max_valid), (100, 700));
        assert_eq!(c.ignored, 1);
        assert_eq!((c.short.count, c.short.avg, c.short_diff), (40, 262, 12));
        assert_eq!((c.long.count, c.long.avg, c.long_diff), (40, 492, -8));
        assert_eq!(c.short_verdict(), WidthVerdict::Excellent);
        assert_eq!(c.long_verdict(), WidthVerdict::Excellent);
        assert_eq!(c.verdict(), TimingVerdict::Matches);
        assert_eq!(c.conclusion_lines(), vec!["Timing matches fob!", "No changes needed."]);

        // Shorts +40 → OK; longs +60 → OK → "Within tolerance."
        let shifted: Vec<_> = pairs.iter().map(|p| pair(p.level, p.duration_us + if p.duration_us < 375 { 28 } else { 68 })).collect();
        let c = compare_with_protocol(&shifted, &timing(250, 500, 100));
        assert_eq!(c.short_verdict(), WidthVerdict::Ok(40));
        assert_eq!(c.long_verdict(), WidthVerdict::Ok(60));
        assert_eq!(c.verdict(), TimingVerdict::WithinTolerance);

        // A definition the shorts miss: 200/500/50 → short HIGH by 62, long still EXCELLENT (−8).
        let c = compare_with_protocol(&pairs, &timing(200, 500, 50));
        assert_eq!(c.short_verdict(), WidthVerdict::High(62));
        assert_eq!(c.long_verdict(), WidthVerdict::Excellent);
        assert_eq!(c.verdict(), TimingVerdict::NeedsAdjustment);
        assert_eq!(c.conclusion_lines(), vec!["NEEDS ADJUSTMENT:", "Set te_short=262"]);

        // Both off: 300/560/20 → LOW by 38 / LOW by 68, both values named.
        let c = compare_with_protocol(&pairs, &timing(300, 560, 20));
        assert_eq!(c.short_verdict(), WidthVerdict::Low(38));
        assert_eq!(c.long_verdict(), WidthVerdict::Low(68));
        assert_eq!(
            c.conclusion_lines(),
            vec!["NEEDS ADJUSTMENT:", "Set te_short=262", "Set te_long=492"]
        );
    }

    #[test]
    fn valid_window_floor_keeps_sub_100us_pulses() {
        // Ford V1 (65/130/39): ProtoPirate's 100 µs floor would drop every short; KAT keeps them.
        let mut pairs = Vec::new();
        for i in 0..60 {
            pairs.push(pair(i % 2 == 0, if i % 3 == 0 { 130 } else { 65 }));
        }
        let c = compare_with_protocol(&pairs, &timing(65, 130, 39));
        assert_eq!(c.min_valid, 32);
        assert_eq!(c.short.count, 40);
        assert_eq!(c.long.count, 20);
        assert_eq!(c.verdict(), TimingVerdict::Matches);
    }

    #[test]
    fn empty_and_tiny_inputs_do_not_panic() {
        let r = analyze(&[], Some("Kia V1"), Some(&timing(800, 1600, 200)));
        assert_eq!(r.comparison.as_ref().unwrap().verdict(), TimingVerdict::NotEnoughSamples);
        assert_eq!(r.coding, LineCoding::Unknown);
        assert!(r.te.is_none());
        assert!(!r.render_lines().is_empty());
        let one = [pair(true, 500)];
        let r = analyze(&one, None, None);
        assert!(r.render_lines().iter().any(|l| l.contains("NO PROTOCOL REFERENCE")));
        // No sample in one class: reported, not mistaken for a LOW verdict.
        let shorts: Vec<_> = (0..20).map(|i| pair(i % 2 == 0, 250)).collect();
        let c = compare_with_protocol(&shorts, &timing(250, 500, 100));
        assert_eq!(c.long_verdict(), WidthVerdict::NoSamples);
        assert_eq!(c.verdict(), TimingVerdict::Incomplete);
        assert!(c.conclusion_lines()[0].starts_with("INCOMPLETE: no long durations (>= 375 us)"));
    }

    /// Real captures: the decoded segments of these IMPORTS files (as `import_fob_files` stores them)
    /// match their protocol's timing definition, and the coding guess agrees with the protocol.
    #[test]
    fn imports_captures_match_their_protocol_timing() {
        use crate::protocols::ProtocolRegistry;
        use crate::radio::LevelDuration;
        let cases = [
            ("KIA/KIA V1 N1 RAW.sub", "Kia V1", "Manchester", 800.0),
            ("FORD/2008_Focus_Lock.sub", "Ford V0", "Manchester", 250.0),
            ("SUBARU/Subaro Impreza RAW .sub", "Subaru", "PWM", 800.0),
            ("KIA/KIA V3 N1 RAW.sub", "Kia V3", "PWM", 400.0),
        ];
        let mut reg = ProtocolRegistry::new();
        for (rel, proto, coding, te_us) in cases {
            let path = std::path::Path::new("IMPORTS").join(rel);
            if !path.exists() {
                eprintln!("Skip: {:?} not found (run from crate root)", path);
                continue;
            }
            crate::protocols::keys::load_keystore_from_embedded();
            let (freq, raw) = crate::export::flipper::import_sub_raw(&path).unwrap();
            let pairs: Vec<LevelDuration> =
                raw.iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect();
            let decodes: Vec<_> = reg
                .process_signal_stream(&pairs, freq)
                .into_iter()
                .filter(|(n, _, _)| n == proto)
                .take(5)
                .collect();
            assert!(!decodes.is_empty(), "{rel}: no {proto} decode");
            let timing = reg.get(proto).unwrap().timing();
            for (i, (name, _, segment)) in decodes.iter().enumerate() {
                let stored: Vec<StoredLevelDuration> = segment
                    .iter()
                    .map(|p| StoredLevelDuration { level: p.level, duration_us: p.duration_us })
                    .collect();
                let r = analyze(&stored, Some(name), Some(&timing));
                let c = r.comparison.as_ref().unwrap();
                assert!(
                    matches!(c.verdict(), TimingVerdict::Matches | TimingVerdict::WithinTolerance),
                    "{rel} #{i}: {:?}\n{}",
                    c.verdict(),
                    r.render_lines().join("\n")
                );
                assert!(c.short_ok && c.long_ok, "{rel} #{i}");
                assert_eq!(r.coding.label(), coding, "{rel} #{i}: {:?}", r.coding);
                let te = r.te.expect("TE").te_us;
                assert!((te - te_us).abs() <= f64::from(timing.te_delta), "{rel} #{i}: TE {te:.0}");
            }
        }
    }

    /// Every IMPORTS capture decoded the way `import_fob_files` does it (segment pairs).
    fn imports_decodes() -> Vec<(String, String, Vec<StoredLevelDuration>, ProtocolTiming)> {
        use crate::protocols::ProtocolRegistry;
        use crate::radio::LevelDuration;
        crate::protocols::keys::load_keystore_from_embedded();
        let root = std::path::Path::new("IMPORTS");
        let mut subs: Vec<std::path::PathBuf> = std::fs::read_dir(root)
            .into_iter()
            .flatten()
            .flatten()
            .filter(|m| m.path().is_dir())
            .filter_map(|m| std::fs::read_dir(m.path()).ok())
            .flat_map(|f| f.flatten().map(|e| e.path()))
            .filter(|p| p.extension().is_some_and(|e| e == "sub"))
            .collect();
        subs.sort();
        let mut reg = ProtocolRegistry::new();
        let mut out = Vec::new();
        for path in subs {
            let Ok((freq, raw)) = crate::export::flipper::import_sub_raw(&path) else { continue };
            let pairs: Vec<LevelDuration> =
                raw.iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect();
            for (name, _, segment) in reg.process_signal_stream(&pairs, freq) {
                let Some(timing) = reg.get(&name).map(|d| d.timing()) else { continue };
                let stored = segment
                    .iter()
                    .map(|p| StoredLevelDuration { level: p.level, duration_us: p.duration_us })
                    .collect();
                let rel = path.strip_prefix(root).unwrap_or(&path).to_string_lossy().replace('\\', "/");
                out.push((rel, name, stored, timing));
            }
        }
        out
    }

    /// Diagnostic: timing verdict, TE estimate and coding guess for every IMPORTS decode.
    /// `cargo test sweep_imports_timing -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn sweep_imports_timing() {
        let detail = std::env::var("KAT_TIMING_DETAIL").ok();
        for (rel, name, pairs, timing) in imports_decodes() {
            let r = analyze(&pairs, Some(&name), Some(&timing));
            let c = r.comparison.as_ref().unwrap();
            if detail.as_deref().is_some_and(|d| rel.contains(d)) {
                eprintln!("==== {rel} {name}\n{}", r.render_lines().join("\n"));
                let raw: Vec<String> = pairs.iter().take(400).map(|p| format!("{}{}", if p.level { "+" } else { "-" }, p.duration_us)).collect();
                eprintln!("RAW: {}", raw.join(" "));
            }
            let mut cap = crate::capture::Capture::from_pairs(0, 0, Vec::new());
            cap.protocol = Some(name.clone());
            eprintln!(
                "{:<46} {:<16} {:?} short {}/{} long {}/{} | TE {:>5.0} (H {:>5.0} L {:>5.0}) | guess {:<10} kat {}",
                rel,
                name,
                c.verdict(),
                c.short.avg,
                c.te_short,
                c.long.avg,
                c.te_long,
                r.te.map_or(0.0, |t| t.te_us),
                r.te.and_then(|t| t.high_us).unwrap_or(0.0),
                r.te.and_then(|t| t.low_us).unwrap_or(0.0),
                r.coding.label(),
                cap.modulation()
            );
        }
    }

    /// Random 60–300 µs pulses, like a Flipper OOK recording between transmissions.
    fn noise(n: usize, seed: u32) -> Vec<StoredLevelDuration> {
        let mut rng = Rng(seed);
        (0..n).map(|i| pair(i % 2 == 0, 60 + rng.next() % 241)).collect()
    }

    #[test]
    fn noise_before_the_frame_stays_out_of_the_analysis() {
        // A first decoded segment of a .sub file: thousands of noise pulses, then the frame.
        let mut pairs = noise(3000, 5);
        pairs.extend(manchester_train(250, 64, 5, 17));
        let n = pairs.len();
        let decoded = analyze(&pairs, Some("Ford V0"), Some(&timing(250, 500, 120)));
        assert_eq!(decoded.window, AnalysisWindow::DecodedFrame);
        assert_eq!(decoded.window_start + decoded.pair_count, n);
        assert_eq!(decoded.pair_count, 160);
        assert_eq!(decoded.comparison.as_ref().unwrap().verdict(), TimingVerdict::Matches);
        assert_eq!(decoded.coding, LineCoding::Manchester);

        // Undecoded: the most regular window is found inside the frame region.
        let mut long = noise(3000, 6);
        let frame_start = long.len();
        long.extend(manchester_train(250, 400, 5, 18));
        long.extend(noise(1000, 7));
        let r = analyze(&long, None, None);
        assert_eq!(r.window, AnalysisWindow::MostRegular);
        assert!(r.window_start + 64 >= frame_start, "window at {}", r.window_start);
        assert!((r.te.unwrap().te_us - 250.0).abs() < 10.0, "{:?}", r.te);
        assert_eq!(r.coding, LineCoding::Manchester);
        assert!(r.render_lines()[0].contains("most regular 512-pair window"));
    }

    #[test]
    fn extreme_durations_do_not_overflow() {
        // A crafted .sub can carry durations near u32::MAX (debug builds panic on overflow).
        let mut v = Vec::new();
        for i in 0..64 {
            v.push(pair(true, u32::MAX - (i % 3)));
            v.push(pair(false, if i % 2 == 0 { u32::MAX } else { u32::MAX / 2 }));
        }
        let r = analyze(&v, Some("Kia V1"), Some(&timing(800, 1600, 200)));
        assert!(!r.render_lines().is_empty());
        assert!(!analyze(&v, None, None).render_lines().is_empty());
    }

    #[test]
    fn report_has_protopirate_sections() {
        let pairs = manchester_train(250, 64, 5, 11);
        let r = analyze(&pairs, Some("Ford V0"), Some(&timing(250, 500, 120)));
        let text = r.render_lines().join("\n");
        for section in [
            "PROTOCOL DEFINITION (Ford V0):",
            "  Short: 250 us",
            "  Tolerance: +/-120 us",
            "RECEIVED SIGNAL:",
            "ANALYSIS:",
            "  Short Jitter:",
            "CONCLUSION:",
            "PULSE CLUSTERS",
            "Base TE estimate:",
            "Line coding guess: Manchester",
        ] {
            assert!(text.contains(section), "missing {section:?} in\n{text}");
        }
        assert!(!text.contains("Suggested definition"));
        let undecoded = analyze(&pairs, None, None).render_lines().join("\n");
        assert!(undecoded.contains("NO PROTOCOL REFERENCE (undecoded capture)"));
        assert!(undecoded.contains("Suggested definition: te_short ~"), "{undecoded}");
    }

    /// A frame followed by pulses that are not part of it (the rest of a live burst): with the
    /// frame's end known, the window stops there and the trailing pulses don't skew the result.
    #[test]
    fn frame_end_keeps_trailing_pulses_out() {
        let timing = ProtocolTiming { te_short: 400, te_long: 800, te_delta: 140, min_count_bit: 16 };
        let mut pairs = vec![pair(false, 20_000)];
        for k in 0..64 {
            pairs.push(pair(true, if k % 3 == 0 { 800 } else { 400 }));
            pairs.push(pair(false, if k % 5 == 0 { 800 } else { 400 }));
        }
        let frame_end = pairs.len();
        for k in 0..60 {
            pairs.push(pair(k % 2 == 0, 120 + (k as u32 * 37) % 300));
        }
        let framed = analyze_frame_ending_at(&pairs, frame_end, Some("Test"), Some(&timing));
        assert_eq!(framed.capture_pairs, pairs.len());
        assert_eq!(framed.window_start + framed.pair_count, frame_end);
        assert!(framed.comparison.is_some());
        let text = framed.render_lines().join("\n");
        assert!(text.contains("Short Avg: 400 us"), "{text}");
        // Without the frame end, the window ends with the capture (the trailing pulses).
        let whole = analyze(&pairs, Some("Test"), Some(&timing));
        assert_eq!(whole.window_start + whole.pair_count, pairs.len());
    }
}
