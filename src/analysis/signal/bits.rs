//! A trial decode of one row of pulses to bits, by the line coding the timing analysis guessed.
//!
//! One convention per coding (the report states it): a long HIGH is 1 for PWM, a long gap is 1 for
//! PPM, HIGH then LOW is 1 for Manchester, HIGH is 1 for NRZ. A protocol may well use the opposite,
//! or a differential code on top: these are the bits as sent, to compare rows with.

use super::rows::Context;
use crate::analysis::timing::LineCoding;
use crate::capture::StoredLevelDuration;

/// A leading run of one bit value this long, with as many bits after it, is a preamble.
const PREAMBLE_BITS: usize = 8;
/// The longest run of one level NRZ is read across, in bits.
const MAX_NRZ_RUN: f64 = 64.0;

/// The bits of one row.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Bits {
    /// Every decoded bit, in the order sent.
    pub bits: Vec<bool>,
    /// How many leading bits are a preamble (one value repeated); the data follows.
    pub preamble: usize,
}

impl Bits {
    /// The bits after the preamble.
    pub fn data(&self) -> &[bool] {
        &self.bits[self.preamble.min(self.bits.len())..]
    }

    /// The data as hex digits, the first bit sent on the left, the last digit padded with zeros.
    pub fn hex(&self) -> String {
        self.data()
            .chunks(4)
            .map(|c| {
                let v = c.iter().enumerate().fold(0u32, |v, (i, b)| v | (u32::from(*b) << (3 - i)));
                char::from_digit(v, 16).unwrap_or('?').to_ascii_uppercase()
            })
            .collect()
    }

    /// The data as 0s and 1s.
    pub fn binary(&self) -> String {
        self.data().iter().map(|b| if *b { '1' } else { '0' }).collect()
    }
}

/// The longest run of bits in a row, and the pulses it spans (`start..end`).
pub(super) struct Run {
    pub bits: Bits,
    pub start: usize,
    pub end: usize,
}

type Found = Option<(Vec<bool>, usize, usize)>;

/// Keeps the longest run offered.
struct Best(Found);

impl Best {
    fn offer(&mut self, bits: &mut Vec<bool>, start: usize, end: usize) {
        if bits.len() > self.0.as_ref().map_or(0, |b| b.0.len()) {
            self.0 = Some((std::mem::take(bits), start, end));
        } else {
            bits.clear();
        }
    }
}

pub(super) fn decode(row: &[StoredLevelDuration], ctx: &Context) -> Option<Run> {
    let (bits, start, end) = match ctx.coding {
        LineCoding::Manchester => manchester(row, ctx),
        LineCoding::Nrz => nrz(row, ctx),
        LineCoding::Pwm { .. } | LineCoding::PulseLength | LineCoding::PwmFixedGap | LineCoding::Ppm => {
            pulse_pairs(row, ctx)
        }
        LineCoding::SingleWidth | LineCoding::Unknown => None,
    }?;
    let preamble = match ctx.coding {
        LineCoding::Manchester => preamble_len(&bits),
        _ => 0,
    };
    Some(Run { bits: Bits { bits, preamble }, start, end })
}

/// The length of the leading run of one bit value, when it is a preamble with data after it.
fn preamble_len(bits: &[bool]) -> usize {
    let Some(&first) = bits.first() else { return 0 };
    let run = bits.iter().take_while(|b| **b == first).count();
    if run >= PREAMBLE_BITS && bits.len() - run >= PREAMBLE_BITS {
        run
    } else {
        0
    }
}

/// A pulse's width class; `End` stands for the pause after the row's last pulse.
#[derive(Clone, Copy, PartialEq)]
enum W {
    Short,
    Long,
    Other,
    End,
}

fn width(p: &StoredLevelDuration, ctx: &Context) -> W {
    match ctx.widths(p.level).and_then(|w| w.class(p.duration_us)) {
        Some(0) => W::Short,
        Some(_) => W::Long,
        None => W::Other,
    }
}

/// The bit a pulse and the gap beside it make, for the codings that send one bit per pulse. The
/// row's last pulse counts with its gap lost in the pause, where the pulse alone tells the bit.
fn symbol(coding: &LineCoding, high: W, low: W) -> Option<bool> {
    use W::{End, Long, Short};
    match (coding, high, low) {
        (LineCoding::Pwm { .. }, Short, Long | End) => Some(false),
        (LineCoding::Pwm { .. }, Long, Short | End) => Some(true),
        (LineCoding::PulseLength, Short, Short | End) => Some(false),
        (LineCoding::PulseLength, Long, Long | End) => Some(true),
        (LineCoding::PwmFixedGap, Short, Short | End) => Some(false),
        (LineCoding::PwmFixedGap, Long, Short | End) => Some(true),
        (LineCoding::Ppm, Short, Short) => Some(false),
        (LineCoding::Ppm, Short, Long) => Some(true),
        _ => None,
    }
}

/// PWM and PPM: one bit per pulse and gap. Both pairings are tried (the pulse first, or its gap
/// first); anything that is not a bit (a preamble's equal pairs, a sync pulse) ends a run.
fn pulse_pairs(row: &[StoredLevelDuration], ctx: &Context) -> Found {
    let mut best = Best(None);
    for high_first in [true, false] {
        let Some(mut i) = row.iter().position(|p| p.level == high_first) else { continue };
        let (mut bits, mut start) = (Vec::new(), i);
        while i < row.len() {
            let a = width(&row[i], ctx);
            let b = row.get(i + 1).map_or(W::End, |p| width(p, ctx));
            let (high, low) = if high_first { (a, b) } else { (b, a) };
            match symbol(ctx.coding, high, low) {
                Some(bit) => {
                    if bits.is_empty() {
                        start = i;
                    }
                    bits.push(bit);
                }
                None => best.offer(&mut bits, start, i),
            }
            i += 2;
        }
        best.offer(&mut bits, start, row.len().min(i));
    }
    best.0
}

/// Manchester: every duration is one or two half bits of its level, and a bit is two unlike halves
/// (HIGH then LOW is 1). Both alignments are tried: the wrong one breaks at every long pulse.
fn manchester(row: &[StoredLevelDuration], ctx: &Context) -> Found {
    let (first, last) = (row.first()?, row.last()?);
    // Each half bit with the pulse it came from; None where a duration is neither one nor two. The
    // pause before and after the row has the level its first and last pulse do not, and may hold
    // the outer half of the first or last bit.
    let mut halves: Vec<Option<(bool, usize)>> = vec![Some((!first.level, 0))];
    for (i, p) in row.iter().enumerate() {
        match ctx.widths(p.level).and_then(|w| w.class(p.duration_us)) {
            Some(c) => halves.extend(std::iter::repeat_n(Some((p.level, i)), usize::from(c) + 1)),
            None => halves.push(None),
        }
    }
    halves.push(Some((!last.level, row.len() - 1)));
    let mut best = Best(None);
    for offset in [0, 1] {
        let (mut bits, mut start, mut end) = (Vec::new(), 0, 0);
        for pair in halves[offset..].chunks_exact(2) {
            match (pair[0], pair[1]) {
                (Some((a, i)), Some((b, j))) if a != b => {
                    if bits.is_empty() {
                        start = i;
                    }
                    bits.push(a);
                    end = j + 1;
                }
                _ => best.offer(&mut bits, start, end),
            }
        }
        best.offer(&mut bits, start, end);
    }
    best.0
}

/// NRZ: a duration is that many bits of its level (HIGH is 1), one bit per TE. A duration shorter
/// than half a TE, or absurdly long, ends a run.
fn nrz(row: &[StoredLevelDuration], ctx: &Context) -> Found {
    let te = ctx.te?;
    let mut best = Best(None);
    let (mut bits, mut start) = (Vec::new(), 0);
    for (i, p) in row.iter().enumerate() {
        let n = te.multiple(p.level, f64::from(p.duration_us)).round();
        if (1.0..=MAX_NRZ_RUN).contains(&n) {
            if bits.is_empty() {
                start = i;
            }
            bits.extend(std::iter::repeat_n(p.level, n as usize));
        } else {
            best.offer(&mut bits, start, i);
        }
    }
    best.offer(&mut bits, start, row.len());
    best.0
}
