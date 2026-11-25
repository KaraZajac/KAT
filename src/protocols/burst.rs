//! Whole-burst reading for KAT's own Manchester decoders (Kia Mobis, Kia Denso, Kia Forte, GM RFA,
//! Isuzu Rolling, Isuzu Static).
//!
//! These remotes were worked out from recordings, not ported from a reference, and none has a
//! checksum KAT knows. Their gate is the shape of a whole burst: every pulse one or two half bits
//! long, a fixed lead and sync, an exact number of bits, and the frame ending where the burst
//! ends. So the decoders collect the half bits of a burst and look at it once it is over.
//!
//! A burst is a run of pulses that are each one or two half bits long (within a tolerance). The
//! pulse that ends it (a gap, a glitch) and the pulse before it may each hide one half bit of
//! their own level: a frame that ends LOW runs into the LOW gap behind it. Both are added to the
//! half bits as one extra half each, so a frame may start one half into a burst and may leave one
//! half behind it; the decoders allow for that. (Levels alternate, so a reader that has seen no
//! pulse before a burst, as after a reset, takes the opposite of the burst's first level.)
//!
//! **Skew.** An on-off keyed signal faster than the receiver's filter comes out with its carrier
//! pulses stretched and its pauses shortened (or the other way round) by tens of microseconds: at
//! 118 µs per half bit a long pause and a short carrier pulse can be equally long. A reader made
//! with [BurstReader::skewed] therefore takes every pulse from 0.4 to 2.9 half bits into the
//! burst and counts the half bits when the burst is over: a Manchester signal is HIGH as long as
//! it is LOW, so the mean difference between its HIGH and LOW pulses is twice the skew; with that
//! taken out, a pulse under 1.5 half bits is one half bit and a longer one is two. (A reader may
//! allow pulses of three half bits, for a frame that ends with one: up to 3.9, three from 2.5.)
//! Counted so, a burst of another speed would pass for one of this speed, so the burst must come
//! out at the reader's half bit to within an eighth (its length by its half bits), or it is dropped.
//! So is a burst whose one-half pulses, or whose two-half pulses, are not that long on average (to
//! a quarter of a half bit, from three pulses of a length on): the preamble of a remote with a
//! 218 µs half bit is, to a 120 µs reader, a run of two-half pulses, and the few of them that
//! jitter under 180 µs are counted as one half each. Two of 30 such preambles read as a frame.
//!
//! **Differential Manchester.** [cells] reads the half bits the other way: a transition between
//! all cells, and a 1 where a cell has one in its middle too.

/// Collects the half bits of a burst.
pub struct BurstReader {
    te: u32,
    tol: u32,
    cap: usize,
    /// Count the half bits at the end of the burst, with its skew taken out.
    skewed: bool,
    /// The longest pulse of a skewed reader's burst, in half bits (2 or 3).
    max_units: u32,
    halves: Vec<bool>,
    /// The pulses of the burst (skewed readers only).
    pulses: Vec<(bool, u32)>,
    /// Level of the last pulse that was not part of a burst.
    before: Option<bool>,
    /// That level when the burst in `pulses` began.
    lead: Option<bool>,
}

impl BurstReader {
    /// `te`: one half bit in µs; `tol`: how far a pulse may be from `te` or `2 * te`; `cap`: the
    /// longest burst kept, in half bits (a longer one loses its oldest half).
    pub fn new(te: u32, tol: u32, cap: usize) -> Self {
        Self { te, tol, cap, skewed: false, max_units: 2, halves: Vec::new(), pulses: Vec::new(), before: None, lead: None }
    }

    /// A reader that corrects each burst's skew (module doc). `max_units`: the longest pulse, 2 or
    /// 3 half bits; `cap`: the longest burst kept, in pulses.
    pub fn skewed(te: u32, max_units: u32, cap: usize) -> Self {
        Self { skewed: true, max_units, ..Self::new(te, 0, cap) }
    }

    pub fn reset(&mut self) {
        self.halves.clear();
        self.pulses.clear();
        self.before = None;
        self.lead = None;
    }

    /// One pulse. The half bits of the burst this pulse ended, if it ended one.
    pub fn feed(&mut self, level: bool, duration: u32) -> Option<Vec<bool>> {
        if self.skewed {
            let (d, te) = (u64::from(duration), u64::from(self.te));
            if d * 5 < te * 2 || d * 10 > te * u64::from(self.max_units * 10 + 9) {
                let out = self.end(Some(level));
                self.before = Some(level);
                return out;
            }
            if self.pulses.is_empty() {
                self.lead = Some(self.before.unwrap_or(!level));
            }
            if self.pulses.len() >= self.cap {
                self.pulses.drain(..self.cap / 2);
                self.lead = None;
            }
            self.pulses.push((level, duration));
            return None;
        }
        let units = if duration.abs_diff(self.te) <= self.tol {
            1
        } else if duration.abs_diff(2 * self.te) <= self.tol {
            2
        } else {
            0
        };
        if units == 0 {
            let out = self.end(Some(level));
            self.before = Some(level);
            return out;
        }
        if self.halves.is_empty() {
            self.halves.push(self.before.unwrap_or(!level));
        }
        if self.halves.len() + units > self.cap {
            let keep = self.cap / 2;
            let drop = self.halves.len() - keep;
            self.halves.drain(..drop);
        }
        for _ in 0..units {
            self.halves.push(level);
        }
        None
    }

    /// The input ended: the burst collected so far.
    pub fn flush(&mut self) -> Option<Vec<bool>> {
        let out = self.end(None);
        self.before = None;
        out
    }

    fn end(&mut self, closing: Option<bool>) -> Option<Vec<bool>> {
        if self.skewed {
            self.halves = self.count();
        }
        if self.halves.is_empty() {
            return None;
        }
        let mut out = std::mem::take(&mut self.halves);
        if let Some(l) = closing {
            out.push(l);
        }
        Some(out)
    }
}

impl BurstReader {
    /// The half bits of the pulses of a skewed reader's burst.
    fn count(&mut self) -> Vec<bool> {
        let pulses = std::mem::take(&mut self.pulses);
        if pulses.is_empty() {
            return Vec::new();
        }
        let sum = |level: bool| pulses.iter().filter(|p| p.0 == level).map(|p| i64::from(p.1)).sum::<i64>();
        let skew = (sum(true) - sum(false)) / pulses.len() as i64;
        let te = i64::from(self.te);
        let mut halves: Vec<bool> = self.lead.take().into_iter().collect();
        let (mut length, mut counted) = (0, 0);
        // of the pulses counted as one half bit and as two: the sum of their widths, their number
        let mut sorted = [(0i64, 0i64); 2];
        for (level, duration) in pulses {
            let fair = i64::from(duration) + if level { -skew } else { skew };
            let units = if fair * 2 < te * 3 {
                1
            } else if fair * 2 < te * 5 || self.max_units < 3 {
                2
            } else {
                3
            };
            length += i64::from(duration);
            counted += units;
            if let Some(of_length) = sorted.get_mut(units as usize - 1) {
                *of_length = (of_length.0 + fair, of_length.1 + 1);
            }
            for _ in 0..units {
                halves.push(level);
            }
        }
        // another speed: not this reader's burst
        if (length - counted * te).abs() * 8 > counted * te {
            return Vec::new();
        }
        // pulses between the two lengths, sorted into both: not this reader's burst either
        for (units, (sum, pulses)) in (1i64..).zip(sorted) {
            if pulses >= 3 && (sum - pulses * units * te).abs() * 4 > pulses * te {
                return Vec::new();
            }
        }
        halves
    }
}

/// Manchester bits of `halves` from half `phase` on: HIGH then LOW is 1, LOW then HIGH is 0
/// (`inverted` swaps them); None where the two halves of a bit are alike.
pub fn bits(halves: &[bool], phase: usize, inverted: bool) -> Vec<Option<bool>> {
    halves
        .get(phase..)
        .unwrap_or(&[])
        .chunks_exact(2)
        .map(|p| (p[0] != p[1]).then_some(p[0] != inverted))
        .collect()
}

/// Differential Manchester bits of `halves`, two per cell from the first on: 1 where the halves of
/// a cell differ, 0 where they are alike; None where there is no transition in front of the cell.
pub fn cells(halves: &[bool]) -> Vec<Option<bool>> {
    (0..halves.len() / 2)
        .map(|k| (k == 0 || halves[2 * k - 1] != halves[2 * k]).then_some(halves[2 * k] != halves[2 * k + 1]))
        .collect()
}

/// `pattern` stands at `at`.
pub fn matches(bits: &[Option<bool>], at: usize, pattern: &[bool]) -> bool {
    bits.get(at..at + pattern.len()).is_some_and(|w| w.iter().zip(pattern).all(|(b, p)| *b == Some(*p)))
}

/// The `n` bits at `at`, if all of them are bits.
pub fn take(bits: &[Option<bool>], at: usize, n: usize) -> Option<Vec<bool>> {
    bits.get(at..at + n)?.iter().copied().collect()
}

/// Length of the run of `value` bits that ends just before `at`.
pub fn run_before(bits: &[Option<bool>], at: usize, value: bool) -> usize {
    bits[..at.min(bits.len())].iter().rev().take_while(|b| **b == Some(value)).count()
}

/// Half bits left in a burst of `len` halves behind bit `end` of the reading that starts at `phase`.
pub fn spare(len: usize, phase: usize, end: usize) -> usize {
    len.saturating_sub(phase + 2 * end)
}

/// Bits as a number, first bit highest (up to 64).
pub fn number(bits: &[bool]) -> u64 {
    bits.iter().fold(0, |acc, &b| (acc << 1) | u64::from(b))
}

/// The bits of `value`'s low `n` bits, highest first.
pub fn pattern(value: u64, n: usize) -> Vec<bool> {
    (0..n).rev().map(|i| (value >> i) & 1 == 1).collect()
}

/// Pulses of a Manchester bit string for the tests: `te` per half bit, HIGH then LOW for a 1.
#[cfg(test)]
pub fn wire(bits: &[bool], te: u32, gap: u32) -> Vec<crate::radio::demodulator::LevelDuration> {
    use crate::radio::demodulator::LevelDuration;
    let halves: Vec<bool> = bits.iter().flat_map(|&b| [b, !b]).collect();
    let mut out = Vec::new();
    let mut i = 0;
    while i < halves.len() {
        let level = halves[i];
        let run = if halves.get(i + 1) == Some(&level) { 2 } else { 1 };
        out.push(LevelDuration::new(level, te * run));
        i += run as usize;
    }
    // the gap: a LOW pulse (it swallows a closing LOW half), or a LOW one behind a closing HIGH
    if let Some(last) = out.last_mut().filter(|l| !l.level) {
        last.duration_us += gap;
    } else {
        out.push(LevelDuration::new(false, gap));
    }
    out
}

/// `wire` as a receiver gives it: neighbours of one level are one pulse. Used by the PSA 2-Button
/// replay encoder ([`super::psa_two_button`]) and the burst-reader tests.
pub fn merged(wire: Vec<crate::radio::demodulator::LevelDuration>) -> Vec<crate::radio::demodulator::LevelDuration> {
    let mut out: Vec<crate::radio::demodulator::LevelDuration> = Vec::new();
    for p in wire {
        match out.last_mut() {
            Some(q) if q.level == p.level => q.duration_us += p.duration_us,
            _ => out.push(p),
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_burst_ends_at_the_first_other_pulse_and_carries_its_edges() {
        let mut r = BurstReader::new(200, 60, 64);
        assert_eq!(r.feed(false, 5_000), None);
        for (l, d) in [(true, 200), (false, 400), (true, 210)] {
            assert_eq!(r.feed(l, d), None);
        }
        // LOW before, HIGH LOW LOW HIGH, LOW behind
        assert_eq!(r.feed(false, 3_000), Some(vec![false, true, false, false, true, false]));
        assert_eq!(r.feed(false, 3_000), None);
        assert_eq!(r.feed(true, 190), None);
        assert_eq!(r.flush(), Some(vec![false, true]));
        assert_eq!(r.flush(), None);
        // after a reset the pulse before the burst is taken to be of the other level
        r.reset();
        assert_eq!(r.feed(true, 200), None);
        assert_eq!(r.feed(false, 9_000), Some(vec![false, true, false]));
        r.reset();
        assert_eq!(r.feed(false, 200), None);
        assert_eq!(r.flush(), Some(vec![true, false]));
    }

    #[test]
    fn manchester_reading_and_search() {
        let frame = pattern(0b1011_0010, 8);
        let pulses = wire(&frame, 200, 4_000);
        let mut r = BurstReader::new(200, 60, 64);
        let mut burst = None;
        for p in &pulses {
            burst = burst.or(r.feed(p.level, p.duration_us));
        }
        // the half before the burst (taken to be LOW), then the frame
        let halves = burst.unwrap();
        let b = bits(&halves, 1, false);
        assert_eq!(take(&b, 0, 8), Some(frame.clone()));
        assert!(matches(&b, 2, &[true, true, false]));
        assert_eq!(run_before(&b, 4, true), 2);
        assert_eq!(number(&frame), 0xB2);
        assert!(spare(halves.len(), 1, 8) <= 1);
        // the other polarity reads the complement
        let inv = bits(&halves, 1, true);
        assert_eq!(take(&inv, 0, 8).map(|v| number(&v)), Some(0x4D));
        // off phase most pairs are not transitions
        assert!(bits(&halves, 0, false).iter().any(Option::is_none));
    }

    /// Carrier pulses 50 µs too long, pauses 50 µs too short: a long pause (186) is shorter than a
    /// short carrier pulse (168). A plain reader cannot count that, a skewed one can.
    #[test]
    fn a_skewed_reader_takes_the_skew_out() {
        let frame = pattern(0x0000_0D17_8B3C_5A6E, 48);
        let mut pulses = wire(&frame, 118, 60_000);
        let last = pulses.len() - 1;
        for p in &mut pulses[..last] {
            p.duration_us = if p.level { p.duration_us + 50 } else { p.duration_us - 50 };
        }
        let run = |r: &mut BurstReader| pulses.iter().find_map(|p| r.feed(p.level, p.duration_us));
        // (the frame begins with a pause, so the half taken to stand before it is HIGH)
        let halves = run(&mut BurstReader::skewed(118, 2, 512)).unwrap();
        assert_eq!(take(&bits(&halves, 1, false), 0, 48), Some(frame.clone()));
        assert!(spare(halves.len(), 1, 48) <= 1);
        let plain = run(&mut BurstReader::new(118, 40, 512));
        assert!(plain.is_none_or(|h| take(&bits(&h, 1, false), 0, 48) != Some(frame.clone())));
        // the other way round
        for p in &mut pulses[..last] {
            p.duration_us = if p.level { p.duration_us - 90 } else { p.duration_us + 90 };
        }
        let halves = pulses.iter().find_map(|p| BurstReader::skewed(118, 2, 512).feed(p.level, p.duration_us));
        assert!(halves.is_none(), "one pulse is no burst");
        let mut r = BurstReader::skewed(118, 2, 512);
        let halves = pulses.iter().find_map(|p| r.feed(p.level, p.duration_us)).unwrap();
        assert_eq!(take(&bits(&halves, 1, false), 0, 48), Some(frame));
        // any length of pulse can come
        assert_eq!(BurstReader::skewed(118, 2, 8).feed(false, u32::MAX), None);
        assert_eq!(BurstReader::new(118, 40, 8).feed(false, u32::MAX), None);
        // a pulse outside 0.4 to 2.9 half bits ends the burst; the cap keeps the newest pulses
        let mut r = BurstReader::skewed(118, 2, 8);
        for i in 0..30 {
            assert_eq!(r.feed(i % 2 == 0, 118), None);
        }
        let got = r.feed(true, 40).unwrap();
        assert!((5..=9).contains(&got.len()), "{}", got.len());
        r.reset();
        assert_eq!(r.flush(), None);
    }

    #[test]
    fn differential_cells_and_three_half_pulses() {
        // HL HL HH LL HL: 1 1 0 0 1; without the transition in front of the fourth cell it is no bit
        let h = [true, false, true, false, true, true, false, false, true, false];
        assert_eq!(cells(&h), [Some(true), Some(true), Some(false), Some(false), Some(true)]);
        let h = [true, false, true, false, true, true, true, false];
        assert_eq!(cells(&h), [Some(true), Some(true), Some(false), None]);
        // a reader for pulses up to three half bits: 400, 800 and 1200 µs
        let mut r = BurstReader::skewed(400, 3, 64);
        for (l, d) in [(true, 400), (false, 800), (true, 1_200), (false, 400), (true, 430)] {
            assert_eq!(r.feed(l, d), None);
        }
        let got = r.feed(false, 9_000).unwrap();
        assert_eq!(got, [false, true, false, false, true, true, true, false, true, false]);
        // a burst of another speed is dropped: 250 µs pulses are no 400 µs half bits
        let mut r = BurstReader::skewed(400, 3, 64);
        for i in 0..40 {
            assert_eq!(r.feed(i % 2 == 0, 250), None);
        }
        assert_eq!(r.feed(false, 9_000), None);
        for i in 0..40 {
            assert_eq!(r.feed(i % 2 == 0, 370), None);
        }
        assert_eq!(r.feed(false, 9_000).map(|h| h.len()), Some(42));
        // so is one whose short pulses are no half bit: a remote with a 218 µs half bit, to a
        // 120 µs reader, sends two-half pulses, a few of which come out under 180 µs
        let mut r = BurstReader::skewed(120, 2, 512);
        let slow = |r: &mut BurstReader, short: u32| {
            for i in 0..170 {
                assert_eq!(r.feed(i % 2 == 0, if i % 40 == 7 { short } else { 218 }), None);
            }
            r.feed(false, 436).map(|h| h.len())
        };
        assert_eq!(slow(&mut r, 176), None);
        // (with real half bits among them, the same run is a burst)
        assert_eq!(slow(&mut r, 122), Some(170 * 2 - 5 + 2));
        // the same pulses end a two-half reader's burst at the long one
        let mut r = BurstReader::skewed(400, 2, 64);
        for (l, d) in [(true, 400), (false, 400), (true, 800), (false, 800)] {
            assert_eq!(r.feed(l, d), None);
        }
        assert_eq!(r.feed(true, 1_200).map(|h| h.len()), Some(8));
    }

    #[test]
    fn a_long_burst_keeps_its_newest_half() {
        let mut r = BurstReader::new(200, 60, 8);
        for i in 0..20 {
            r.feed(i % 2 == 0, 200);
        }
        let got = r.flush().unwrap();
        assert!(got.len() <= 8 && got.len() >= 4, "{}", got.len());
        assert_eq!(*got.last().unwrap(), false);
    }
}
