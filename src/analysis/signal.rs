//! "Analyze": a description of any capture that has raw pulses, decoded or not.
//!
//! [`timing`] measures the pulse widths, the TE and the line coding. This module adds what a person
//! needs to work a signal out: the modulation (what the receiver measured, or the recording's
//! preset), the rows of pulses between pauses (preambles, frames, repeats), a trial decode of each
//! row to bits, a waveform, the first raw durations, and the registered protocols with similar
//! timing. It decodes no protocol: the bit values follow one stated convention per line coding.
//!
//! No terminal types here: [`SignalReport::render`] gives rows of tone-tagged text for a width, and
//! the UI maps the tones to its styles.

use super::timing::{self, Cluster, LineCoding, TeEstimate, TimingReport};
use crate::capture::{RfMeasurement, RfModulation, StoredLevelDuration};
use crate::protocols::ProtocolTiming;

mod bits;
mod render;
mod rows;
mod tables;
#[cfg(test)]
mod tests;

pub use bits::Bits;
pub use render::{Line, Span, Tone, MIN_WIDTH};
pub(in crate::analysis) use render::{cut, plural, span, wrap, Out};
pub use rows::{Repeat, Row, RowKind};

/// A "TE" this long (µs) is a block of carrier, not a data width.
const MAX_TE_US: f64 = 5_000.0;
/// A HIGH this long (µs) is a carrier block.
const CARRIER_US: u32 = 5_000;
/// The pause that ends a row when the capture has no TE (µs).
const DEFAULT_GAP_US: u32 = 5_000;
/// A duration within this factor of a data width belongs to it.
const WIDTH_FACTOR: f64 = 1.35;
/// A duration this close to a whole number of TEs (in TEs), up to this many of them, fits the TE.
const FIT_SLACK: f64 = 0.2;
const MAX_FIT_MULTIPLE: f64 = 16.0;
/// The share of durations that must fit the TE for a capture without a pulse-pair pattern (NRZ, one
/// width, unknown coding) to count as a regular pulse train. Random pulses reach about 0.4.
const REGULAR_FIT: f64 = 0.75;
/// Protocols listed under "similar timing".
const MAX_SIMILAR: usize = 5;

/// What the app knows about a capture besides its pulses.
#[derive(Debug, Clone, Default)]
pub struct Source {
    /// Frequency the capture was received or recorded at, in Hz.
    pub frequency: u32,
    /// Demodulator path the pulses came from: KAT's receiver (`live`), else the file's preset.
    pub path: Option<RfModulation>,
    /// The pulses came from one of KAT's radios, not from a file.
    pub live: bool,
    /// What the receiver measured of the burst (HackRF, RTL-SDR, IQ file).
    pub measured: Option<RfMeasurement>,
    /// Preset of the recording as the UI names it ("AM650", "FM476", …).
    pub preset: Option<String>,
    /// Deviation of that preset in Hz, when it is an FSK one.
    pub preset_deviation_hz: Option<f64>,
    /// ID of the list's capture of the same burst on the other path.
    pub sibling: Option<u32>,
}

/// A registered protocol's timing, for the list of protocols with similar timing.
#[derive(Debug, Clone, PartialEq)]
pub struct KnownTiming {
    pub name: &'static str,
    pub te_short: u32,
    pub te_long: u32,
    pub te_delta: u32,
    /// Line coding and modulation as the protocol table names them ("PWM", "AM").
    pub coding: String,
    pub rf: String,
}

/// How the data is put on the carrier.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Keying {
    /// Carrier on and off (OOK / ASK).
    Ook,
    /// Two tones (2-FSK), with the deviation in Hz when it is known.
    Fsk(Option<f64>),
    /// Nothing tells.
    Unknown,
}

impl Keying {
    pub fn label(&self) -> String {
        match self {
            Keying::Ook => "OOK".to_string(),
            Keying::Fsk(Some(d)) => format!("2-FSK +/-{:.1} kHz", d / 1e3),
            Keying::Fsk(None) => "2-FSK".to_string(),
            Keying::Unknown => "modulation unknown".to_string(),
        }
    }

    fn of(source: &Source) -> Self {
        let fm = source.path == Some(RfModulation::FM);
        if let Some(m) = source.measured {
            return match m.deviation_hz {
                Some(d) => Keying::Fsk(Some(f64::from(d))),
                None if fm => Keying::Fsk(None),
                None => Keying::Ook,
            };
        }
        match (source.path, source.preset_deviation_hz) {
            (_, Some(d)) => Keying::Fsk(Some(d)),
            (Some(RfModulation::FM), None) => Keying::Fsk(None),
            (Some(RfModulation::AM), None) => Keying::Ook,
            _ => Keying::Unknown,
        }
    }
}

/// The data widths of one level in µs: the short one and, when the level has two, the long one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LevelWidths {
    pub short: f64,
    pub long: Option<f64>,
}

impl LevelWidths {
    /// From a level's clusters: its two data widths when it has two (the long one at most 4.5 times
    /// the short one: anything longer is a pause), else its most common width.
    fn of(clusters: &[Cluster], level: bool, te: &TeEstimate, coding: &LineCoding) -> Option<Self> {
        // Manchester has a 1 TE and a 2 TE width and no other: a width the recorder split in two is
        // one width again.
        let merged = match coding {
            LineCoding::Manchester => timing::merged_by_te(clusters, level, te),
            _ => Vec::new(),
        };
        let majors: Vec<&Cluster> = match coding {
            LineCoding::Manchester => merged.iter().collect(),
            _ => timing::major_clusters(clusters),
        };
        if let Some([s, l]) = timing::two_data_widths(&majors) {
            if l.mean <= 4.5 * s.mean {
                return Some(Self { short: s.mean, long: Some(l.mean) });
            }
        }
        let most = majors.iter().max_by_key(|c| c.count)?;
        Some(Self { short: most.mean, long: None })
    }

    /// 0 for the short width, 1 for the long one, None for a duration that is neither.
    pub fn class(&self, us: u32) -> Option<u8> {
        let x = f64::from(us.max(1));
        let off = |w: f64| (x / w).ln().abs();
        let (class, w) = match self.long {
            Some(l) if off(l) < off(self.short) => (1, l),
            _ => (0, self.short),
        };
        (off(w) <= WIDTH_FACTOR.ln()).then_some(class)
    }

    fn widest(&self) -> f64 {
        self.long.unwrap_or(self.short)
    }
}

/// Everything "Analyze" found out about one capture.
#[derive(Debug, Clone)]
pub struct SignalReport {
    pub source: Source,
    /// Pulse widths, TE, line coding and the comparison with the decoded protocol's definition.
    pub timing: TimingReport,
    pub keying: Keying,
    /// The capture's pulses, neighbours of one level merged and empty ones dropped.
    pub pulses: Vec<StoredLevelDuration>,
    pub high: Option<LevelWidths>,
    pub low: Option<LevelWidths>,
    /// A pause this long (µs) ends a row.
    pub gap_us: u32,
    /// The pulses make a regular train: a known pulse-pair pattern, or widths that fit the TE.
    pub regular: bool,
    /// The rows worth listing, in order: data, preambles, long pulses, other regular rows.
    pub rows: Vec<Row>,
    /// The other rows (noise, broken pieces): how many, and the pulses in them.
    pub fragments: usize,
    pub fragment_pulses: usize,
    /// The rows come round again with this period.
    pub repeat: Option<Repeat>,
    /// Most of the carrier time is long blocks without on/off data, the longest this many µs (AM
    /// path, undecoded captures only).
    pub carrier_block_us: Option<u32>,
    /// µs from the first listed row's first pulse to the end of the last one's.
    pub on_air_us: u64,
    /// Registered protocols whose definition is close to the measured widths, nearest first.
    pub similar: Vec<KnownTiming>,
}

/// The pulses with neighbours of one level merged and empty ones dropped.
fn normalise(pairs: &[StoredLevelDuration]) -> Vec<StoredLevelDuration> {
    let mut out: Vec<StoredLevelDuration> = Vec::with_capacity(pairs.len());
    for p in pairs.iter().filter(|p| p.duration_us > 0) {
        match out.last_mut() {
            Some(last) if last.level == p.level => {
                last.duration_us = last.duration_us.saturating_add(p.duration_us)
            }
            _ => out.push(*p),
        }
    }
    out
}

/// The pulse is a whole number of TEs long, within [`FIT_SLACK`].
pub(super) fn fits_te(p: &StoredLevelDuration, te: &TeEstimate) -> bool {
    let r = te.multiple(p.level, f64::from(p.duration_us));
    (1.0..=MAX_FIT_MULTIPLE).contains(&r.round()) && (r - r.round()).abs() <= FIT_SLACK
}

/// The share of the durations that fit the TE.
fn te_fit(window: &[StoredLevelDuration], te: &TeEstimate) -> f64 {
    let fits = window.iter().filter(|p| fits_te(p, te)).count();
    if window.is_empty() {
        0.0
    } else {
        fits as f64 / window.len() as f64
    }
}

/// The pause that ends a row: well over the widest data width, and over any run of equal bits the
/// coding allows.
fn gap_limit(high: Option<LevelWidths>, low: Option<LevelWidths>, te: Option<f64>, coding: &LineCoding) -> u32 {
    let Some(te) = te else { return DEFAULT_GAP_US };
    let widest = [high, low].into_iter().flatten().map(|w| w.widest()).fold(te, f64::max);
    let mut limit = (2.5 * widest).max(6.0 * te);
    if matches!(coding, LineCoding::Nrz) {
        limit = limit.max(40.0 * te);
    }
    limit.clamp(600.0, 80_000.0) as u32
}

/// The longest carrier block, when blocks of [`CARRIER_US`] or more hold most of the HIGH time.
fn carrier_block(pulses: &[StoredLevelDuration]) -> Option<u32> {
    let highs = || pulses.iter().filter(|p| p.level).map(|p| p.duration_us);
    let total: u64 = highs().map(u64::from).sum();
    let blocks: u64 = highs().filter(|d| *d >= CARRIER_US).map(u64::from).sum();
    (total > 0 && blocks * 10 > total * 6).then(|| highs().max().unwrap_or(0))
}

/// Analyse a capture's pulses. `references` are the decoded protocol's timing definitions, the
/// main one first (most protocols have one), and `frame_end` the pair its decoded frame ends at
/// (none of either for an unknown capture); `known` holds the registered protocols' timings for the
/// "similar timing" list.
pub fn analyze(
    pairs: &[StoredLevelDuration],
    source: Source,
    protocol: Option<&str>,
    references: &[ProtocolTiming],
    frame_end: Option<usize>,
    known: &[KnownTiming],
) -> SignalReport {
    let reference = references.first();
    let mut timing = match frame_end {
        Some(end) => timing::analyze_frame_ending_at(pairs, end, protocol, reference),
        None => timing::analyze(pairs, protocol, reference),
    };
    if let [_, others @ ..] = references {
        timing.compare_with_variants(pairs, others);
    }
    let pulses = normalise(pairs);
    let fm = source.path == Some(RfModulation::FM);
    let te = timing.te.filter(|t| t.te_us < MAX_TE_US);
    let (high, low) = match te {
        Some(t) => (
            LevelWidths::of(&timing.high, true, &t, &timing.coding),
            LevelWidths::of(&timing.low, false, &t, &timing.coding),
        ),
        None => (None, None),
    };
    let gap_us = gap_limit(high, low, te.map(|t| t.te_us), &timing.coding);
    let window = pairs.get(timing.window_start..timing.window_start + timing.pair_count).unwrap_or(pairs);
    let fit = te.map_or(0.0, |t| te_fit(window, &t));
    let regular = te.is_some()
        && match timing.coding {
            LineCoding::Nrz | LineCoding::SingleWidth | LineCoding::Unknown => fit >= REGULAR_FIT,
            _ => true,
        };
    let ctx = rows::Context { coding: &timing.coding, high, low, te, gap_us, either_level: fm, regular };
    let split = rows::split(&pulses, &ctx);
    let on_air_us = split.rows.last().map_or(0, |r| r.at_us + r.length_us);
    let mut report = SignalReport {
        keying: Keying::of(&source),
        // A decoded capture's signal is known: blocks elsewhere in the recording say nothing of it.
        carrier_block_us: if fm || protocol.is_some() { None } else { carrier_block(&pulses) },
        repeat: rows::find_repeat(&split.rows),
        rows: split.rows,
        fragments: split.fragments,
        fragment_pulses: split.fragment_pulses,
        similar: Vec::new(),
        regular,
        source,
        timing,
        pulses,
        high,
        low,
        gap_us,
        on_air_us,
    };
    if report.timing.comparison.is_none() && report.is_regular() {
        report.similar = report.similar_to(known);
    }
    report
}

impl SignalReport {
    /// The TE, unless it is as long as a carrier block.
    pub fn te(&self) -> Option<TeEstimate> {
        self.timing.te.filter(|t| t.te_us < MAX_TE_US)
    }

    /// The capture holds a regular pulse train (not noise, not bare carrier).
    pub fn is_regular(&self) -> bool {
        self.regular
    }

    /// The data rows, with their numbers in the row list (from 1).
    pub fn data_rows(&self) -> impl Iterator<Item = (usize, &Row, &Bits)> {
        self.rows.iter().enumerate().filter_map(|(i, r)| match &r.kind {
            RowKind::Data { bits, .. } => Some((i + 1, r, bits)),
            _ => None,
        })
    }

    /// The measured short and long widths (µs) to look protocols up by: the TE, and the mean of the
    /// levels' long widths.
    fn measured_widths(&self) -> Option<(f64, Option<f64>)> {
        let te = self.te()?.te_us;
        let longs: Vec<f64> = [self.high, self.low].iter().flatten().filter_map(|w| w.long).collect();
        let long = (!longs.is_empty()).then(|| longs.iter().sum::<f64>() / longs.len() as f64);
        Some((te, long))
    }

    /// Registered protocols whose short and long widths are both within their own tolerance of the
    /// measured ones, nearest first.
    fn similar_to(&self, known: &[KnownTiming]) -> Vec<KnownTiming> {
        let Some((short, long)) = self.measured_widths() else { return Vec::new() };
        let mut found: Vec<(f64, &KnownTiming)> = known
            .iter()
            .filter_map(|k| {
                let delta = f64::from(k.te_delta.max(1));
                let ds = (f64::from(k.te_short) - short).abs();
                let dl = long.map_or(0.0, |l| (f64::from(k.te_long) - l).abs());
                (ds <= delta && dl <= delta).then_some((ds + dl, k))
            })
            .collect();
        found.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.name.cmp(b.1.name)));
        found.into_iter().take(MAX_SIMILAR).map(|(_, k)| k.clone()).collect()
    }
}
