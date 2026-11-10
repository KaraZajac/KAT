//! Rows: the stretches of pulses between pauses, and what each one is.

use std::collections::HashMap;

use super::bits::{self, Bits};
use super::{fits_te, LevelWidths};
use crate::analysis::timing::{LineCoding, TeEstimate};
use crate::capture::StoredLevelDuration;

/// A row with fewer decoded bits than this is not data.
const MIN_DATA_BITS: usize = 8;
/// A run of this many bits is data wherever it lies in its row; a shorter one must be half the row.
const SURE_DATA_BITS: usize = 24;
/// A row of one pulse width needs this many pulses to be a preamble.
const MIN_PREAMBLE_PULSES: usize = 4;
/// A lone pulse this long (µs) is worth listing: carrier, a wake-up or a sync pulse.
const LONG_PULSE_US: f64 = 1_000.0;
/// A row the trial decode cannot read is still listed when it has this many durations and this
/// share of them (in percent) have a data width.
const MIN_PLAIN_ROW: usize = 16;
const PLAIN_ROW_FIT: usize = 85;
/// Rows compared when looking for a repeat (the search is quadratic).
const MAX_REPEAT_ROWS: usize = 2_000;
/// Long pulses within this fraction (1/n) of each other are alike, and repetitions this near the
/// usual time apart keep the pace.
const REPEAT_SLACK: u32 = 8;

/// What the row splitter needs to know about the pulse train.
pub(super) struct Context<'a> {
    pub coding: &'a LineCoding,
    pub high: Option<LevelWidths>,
    pub low: Option<LevelWidths>,
    pub te: Option<TeEstimate>,
    /// A pause this long (µs) ends a row.
    pub gap_us: u32,
    /// A long duration of either level is a pause (the FM path: a tone held), not just a LOW.
    pub either_level: bool,
    /// The capture is a regular pulse train; if not, only long pulses are worth listing.
    pub regular: bool,
}

impl Context<'_> {
    pub fn widths(&self, level: bool) -> Option<LevelWidths> {
        if level {
            self.high
        } else {
            self.low
        }
    }

    /// The duration is one of its level's data widths (within a fifth), or a whole number of TEs.
    fn fits(&self, p: &StoredLevelDuration) -> bool {
        let near = |w: f64| (f64::from(p.duration_us) - w).abs() <= 0.2 * w;
        let width = self.widths(p.level).is_some_and(|w| near(w.short) || w.long.is_some_and(near));
        width || self.te.is_some_and(|te| fits_te(p, &te))
    }
}

/// One stretch of pulses between two pauses.
#[derive(Debug, Clone, PartialEq)]
pub struct Row {
    /// Where it starts in the report's pulses, and how many of them it holds.
    pub start: usize,
    pub len: usize,
    /// µs from the first listed row's start.
    pub at_us: u64,
    /// µs from its first pulse to the end of its last.
    pub length_us: u64,
    /// µs from its end to the next listed row (None for the last).
    pub then_us: Option<u64>,
    pub kind: RowKind,
}

#[derive(Debug, Clone, PartialEq)]
pub enum RowKind {
    /// A run of bits, with the pulses before and after it in the row. `same_as` is the earlier row
    /// (index) with the same bits; `differs` the first data row of the same length and how many
    /// bits are unlike it.
    Data { bits: Bits, lead: usize, tail: usize, same_as: Option<usize>, differs: Option<(usize, usize)> },
    /// Pulses of one width per level: a preamble (or one bit value repeated).
    Preamble { high_us: u32, low_us: u32 },
    /// One long pulse: carrier, a wake-up or a sync pulse.
    Long { us: u32 },
    /// Regular pulses the trial decode cannot read as bits.
    Pulses,
}

/// Rows that come round again: `rows` of them per repetition, `times` repetitions (the last one may
/// be cut short), `every_us` from one to the next (0 when they keep no pace).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Repeat {
    pub rows: usize,
    pub times: usize,
    pub every_us: u64,
}

#[derive(Default)]
pub(super) struct Split {
    pub rows: Vec<Row>,
    pub fragments: usize,
    pub fragment_pulses: usize,
}

/// Cut the pulses into rows at the pauses and keep the ones that are data, a preamble or a long
/// pulse; the rest (noise, broken pieces) is only counted.
pub(super) fn split(pulses: &[StoredLevelDuration], ctx: &Context) -> Split {
    let is_gap = |p: &StoredLevelDuration| p.duration_us >= ctx.gap_us && (!p.level || ctx.either_level);
    let mut out = Split::default();
    let mut origin: Option<u64> = None;
    let (mut t, mut i) = (0u64, 0usize);
    while i < pulses.len() {
        if is_gap(&pulses[i]) {
            t += u64::from(pulses[i].duration_us);
            i += 1;
            continue;
        }
        let (start, start_t) = (i, t);
        while i < pulses.len() && !is_gap(&pulses[i]) {
            t += u64::from(pulses[i].duration_us);
            i += 1;
        }
        let row = &pulses[start..i];
        match classify(row, ctx) {
            Some(kind) => {
                let origin = *origin.get_or_insert(start_t);
                out.rows.push(Row {
                    start,
                    len: row.len(),
                    at_us: start_t - origin,
                    length_us: t - start_t,
                    then_us: None,
                    kind,
                });
            }
            None => {
                out.fragments += 1;
                out.fragment_pulses += row.len();
            }
        }
    }
    for k in 1..out.rows.len() {
        let end = out.rows[k - 1].at_us + out.rows[k - 1].length_us;
        out.rows[k - 1].then_us = Some(out.rows[k].at_us.saturating_sub(end));
    }
    link_equal(&mut out.rows);
    out
}

/// The median of each level's durations when every pulse of the row is within a quarter of it.
fn one_width(row: &[StoredLevelDuration]) -> Option<(u32, u32)> {
    if row.len() < MIN_PREAMBLE_PULSES {
        return None;
    }
    let median = |level: bool| -> Option<u32> {
        let mut d: Vec<u32> = row.iter().filter(|p| p.level == level).map(|p| p.duration_us).collect();
        d.sort_unstable();
        let m = *d.get(d.len() / 2)?;
        d.iter().all(|x| x.abs_diff(m) * 4 <= m).then_some(m)
    };
    Some((median(true)?, median(false)?))
}

fn classify(row: &[StoredLevelDuration], ctx: &Context) -> Option<RowKind> {
    if ctx.regular {
        if let Some((high_us, low_us)) = one_width(row) {
            return Some(RowKind::Preamble { high_us, low_us });
        }
        if let Some(run) = bits::decode(row, ctx) {
            // Noise reads as a few bits here and there: a short run must be most of its row.
            let bits = run.bits.data().len();
            let most = run.end.saturating_sub(run.start) * 2 >= row.len();
            if bits >= MIN_DATA_BITS && (most || bits >= SURE_DATA_BITS) {
                let tail = row.len().saturating_sub(run.end);
                return Some(RowKind::Data { bits: run.bits, lead: run.start, tail, same_as: None, differs: None });
            }
        }
    }
    let longest = row.iter().filter(|p| p.level).map(|p| p.duration_us).max().unwrap_or(0);
    let long = ctx.te.map_or(LONG_PULSE_US, |te| (4.0 * te.te_us).max(LONG_PULSE_US));
    if row.len() <= 3 && f64::from(longest) >= long {
        return Some(RowKind::Long { us: longest });
    }
    let fitting = row.iter().filter(|p| ctx.fits(p)).count();
    (ctx.regular && row.len() >= MIN_PLAIN_ROW && fitting * 100 >= row.len() * PLAIN_ROW_FIT)
        .then_some(RowKind::Pulses)
}

/// Mark each data row with the earlier row that has the same bits, or else with how far it is from
/// the first data row of its length.
fn link_equal(rows: &mut [Row]) {
    let mut by_bits: HashMap<Vec<bool>, usize> = HashMap::new();
    let mut by_len: HashMap<usize, usize> = HashMap::new();
    for i in 0..rows.len() {
        let RowKind::Data { bits, .. } = &rows[i].kind else { continue };
        let data = bits.data().to_vec();
        let twin = by_bits.get(&data).copied();
        let unlike = match (twin, by_len.get(&data.len())) {
            (None, Some(&first)) => match &rows[first].kind {
                RowKind::Data { bits: other, .. } => {
                    Some((first, other.data().iter().zip(&data).filter(|(a, b)| a != b).count()))
                }
                _ => None,
            },
            _ => None,
        };
        by_len.entry(data.len()).or_insert(i);
        by_bits.entry(data).or_insert(i);
        if let RowKind::Data { same_as, differs, .. } = &mut rows[i].kind {
            *same_as = twin;
            *differs = unlike;
        }
    }
}

/// Two rows are the same kind of thing: data of one length, preambles, long pulses of about one
/// length, or as many unread pulses.
fn alike(a: &Row, b: &Row) -> bool {
    match (&a.kind, &b.kind) {
        (RowKind::Data { bits: x, .. }, RowKind::Data { bits: y, .. }) => x.data().len() == y.data().len(),
        (RowKind::Preamble { .. }, RowKind::Preamble { .. }) => true,
        (RowKind::Long { us: x }, RowKind::Long { us: y }) => x.abs_diff(*y) <= x.max(y) / REPEAT_SLACK,
        (RowKind::Pulses, RowKind::Pulses) => a.len == b.len,
        _ => false,
    }
}

/// The shortest period the rows repeat with: rows [`alike`], at least twice over. The time from
/// one repetition to the next is given when most of them keep it (a button pressed again, or
/// frames sent in groups, is a repeat without a pace).
pub(super) fn find_repeat(rows: &[Row]) -> Option<Repeat> {
    let rows = &rows[..rows.len().min(MAX_REPEAT_ROWS)];
    let n = rows.len();
    let period = (1..=n / 2).find(|&p| (0..n - p).all(|i| alike(&rows[i], &rows[i + p])))?;
    let mut steps: Vec<u64> = (0..n - period).map(|i| rows[i + period].at_us - rows[i].at_us).collect();
    steps.sort_unstable();
    let median = steps[steps.len() / 2];
    let kept = steps.iter().filter(|s| s.abs_diff(median) <= median / u64::from(REPEAT_SLACK)).count();
    let every_us = if kept * 4 >= steps.len() * 3 { median } else { 0 };
    Some(Repeat { rows: period, times: n / period, every_us })
}
