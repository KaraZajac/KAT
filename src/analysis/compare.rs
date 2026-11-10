//! Compare: the frames of several presses of one remote lined up bit by bit, and what each column
//! of bits does from one press to the next: it stays fixed, it is part of a counter, it follows
//! the button, or it changes with no pattern (encrypted, a hash, a checksum).
//!
//! Analyze ([`super::signal`]) says how a signal is coded; this says what its bits mean. It works
//! on bits alone, in the order the frames are given: a decoded frame's data, or the rows the trial
//! decode reads from an unknown signal. Like the Analyze report it has no terminal types: the
//! result is lines of tone-tagged text for a width.
//!
//! What it claims, and on what evidence:
//! - **fixed**: the column is the same in every frame.
//! - **counter**: neighbouring columns that, read as a number, go up (or down) by a small step from
//!   each frame to the next, with the carries where they belong. The evidence is how many of the
//!   fields tried would do the same if their bits were random ([`SURE`], [`LIKELY`]).
//! - **button**: the column is the same in every frame of one button and differs between two
//!   buttons. Each further frame of a button that agrees halves the chance of that happening by
//!   itself ([`GROUP_SURE`], [`GROUP_LIKELY`]).
//! - **rarely / every time**: the column changes far less (or more) often than a coin toss would.
//! - **no pattern**: the rest.

use std::collections::{BTreeMap, BTreeSet};

use super::signal::{cut, plural, span, wrap, Line, Out, Span, Tone, MIN_WIDTH};

/// The widest field read as a counter.
const MAX_COUNTER_BITS: usize = 24;
/// The largest steps a counter is tried with, smallest first (each one below a power of two).
const STEPS: [u32; 4] = [1, 3, 7, 15];
/// Counters looked for in one comparison.
const MAX_COUNTERS: usize = 8;
/// Bits of evidence for "counts": random bits pass for a counter this well, somewhere in the
/// frame, once in five hundred comparisons. And for "may count": once in sixteen.
const SURE: f64 = 9.0;
const LIKELY: f64 = 4.0;
/// The same for the columns that follow the button: once in 32, and once in four.
const GROUP_SURE: f64 = 5.0;
const GROUP_LIKELY: f64 = 2.0;
/// A column changes rarely (or every time) when a coin toss does so with less than this chance,
/// over all the columns looked at.
const RATE_CHANCE: f64 = 0.05;
/// Values of a field listed in full.
const LISTED_VALUES: usize = 4;
/// The widest label of a row of bits.
const MAX_LABEL: usize = 14;

/// One frame to compare.
#[derive(Debug, Clone, PartialEq)]
pub struct Frame {
    /// What its row is called ("#07", "#07/2").
    pub name: String,
    /// The button it was sent for (or the group the user put it in), when known.
    pub group: Option<String>,
    /// Its bits, the first one sent first.
    pub bits: Vec<bool>,
}

/// What the frames' groups are, for the report's wording.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Grouping {
    /// The button each frame was sent for.
    Button,
    /// Groups the user made by hand.
    Group,
}

impl Grouping {
    fn word(self) -> &'static str {
        match self {
            Grouping::Button => "button",
            Grouping::Group => "group",
        }
    }
}

/// What a column of bits does from one frame to the next.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Class {
    Fixed,
    /// Part of the counter with this number in [`Comparison::counters`].
    Counter(usize),
    /// The same within every button (or group) and different between two of them.
    Group { sure: bool },
    /// Changes far less often than a coin toss would.
    Rare,
    /// Changes from every frame to the next.
    EveryTime,
    /// Changes with no pattern found.
    Varies,
}

impl Class {
    /// The letter under the column in the rows of bits.
    fn letter(self) -> char {
        match self {
            Class::Fixed => '.',
            Class::Counter(_) => 'C',
            Class::Group { .. } => 'B',
            Class::Rare => 'r',
            Class::EveryTime => 'e',
            Class::Varies => '?',
        }
    }

    fn tone(self) -> Tone {
        match self {
            Class::Fixed => Tone::Dim,
            Class::Counter(_) => Tone::Good,
            Class::Group { .. } => Tone::Accent,
            Class::Rare | Class::EveryTime => Tone::Warn,
            Class::Varies => Tone::Plain,
        }
    }
}

/// A field that counts.
#[derive(Debug, Clone, PartialEq)]
pub struct Counter {
    /// Its columns: `start..start + len`.
    pub start: usize,
    pub len: usize,
    /// Its low bit is sent first (the usual order is the high bit first).
    pub low_first: bool,
    pub down: bool,
    /// The smallest and the largest step from one frame to the next.
    pub steps: (u32, u32),
    /// How many times the field changed from one frame to the next, and how many of those changes
    /// are steps (the others are jumps: frames out of order, or presses in between not captured).
    pub changes: usize,
    pub fits: usize,
    /// Beyond doubt ([`SURE`]), or only likely.
    pub sure: bool,
}

impl Counter {
    /// The field's value in a frame.
    pub fn value(&self, frame: &Frame) -> u32 {
        let bits = &frame.bits[self.start..self.start + self.len];
        if self.low_first {
            bits.iter().rev().fold(0, |v, b| (v << 1) | u32::from(*b))
        } else {
            bits.iter().fold(0, |v, b| (v << 1) | u32::from(*b))
        }
    }

    /// "counts up by 1", "may count down by 1 to 3, 1 jump"; `bits` for what its bits do ("count
    /// up by 1").
    fn text(&self, bits: bool) -> String {
        let verb = match (self.sure, bits) {
            (true, false) => "counts",
            (true, true) => "count",
            (false, _) => "may count",
        };
        let verb = format!("{verb} {}", if self.down { "down" } else { "up" });
        let mut text = match self.steps {
            (a, b) if a == b => format!("{verb} by {a}"),
            (a, b) => format!("{verb} by {a} to {b}"),
        };
        let jumps = self.changes - self.fits;
        if jumps > 0 {
            text.push_str(&format!(", {jumps} jump{}", plural(jumps)));
        }
        if self.low_first {
            text.push_str(", low bit first");
        }
        text
    }
}

/// A run of neighbouring columns of one class.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Field {
    pub start: usize,
    pub len: usize,
    pub class: Class,
}

/// The frames and what was found out about their columns.
#[derive(Debug, Clone, PartialEq)]
pub struct Comparison {
    pub frames: Vec<Frame>,
    /// Bits per frame.
    pub bits: usize,
    /// What the bits are ("the decoded data", "the bits as sent").
    pub what: String,
    pub grouping: Grouping,
    /// Remarks of whoever gathered the frames (rows left out and the like).
    pub notes: Vec<String>,
    /// One class per column.
    pub classes: Vec<Class>,
    pub counters: Vec<Counter>,
    /// How many different frames there are.
    pub distinct: usize,
}

/// log2 of the chance that `k` or more of `n` tries succeed, each with chance `q`: one number for
/// every `k` from 0 to `n`.
fn log2_tails(n: usize, q: f64) -> Vec<f64> {
    let mut tails = vec![f64::NEG_INFINITY; n + 1];
    tails[0] = 0.0;
    if q >= 1.0 {
        tails.fill(0.0);
    }
    if q <= 0.0 || q >= 1.0 {
        return tails;
    }
    let (yes, no) = (q.log2(), (1.0 - q).log2());
    // log2 of "n choose k", from k = n down, and the sum of the chances so far.
    let mut choose = 0.0f64;
    let mut tail = f64::NEG_INFINITY;
    for k in (0..=n).rev() {
        if k < n {
            choose += ((k + 1) as f64 / (n - k) as f64).log2();
        }
        let exactly = choose + k as f64 * yes + (n - k) as f64 * no;
        let (more, less) = if tail > exactly { (tail, exactly) } else { (exactly, tail) };
        tail = more + (1.0 + (less - more).exp2()).log2();
        tails[k] = tail.min(0.0);
    }
    tails[0] = 0.0;
    tails
}

fn log2_tail(n: usize, k: usize, q: f64) -> f64 {
    log2_tails(n, q)[k.min(n)]
}

/// Fields tried that pass for a counter by accident with the same chance: whether they lie beside
/// bits that do not change, how many times they change, their width and the largest step.
type Tried = BTreeMap<(bool, usize, usize, u32), usize>;

/// log2 of how many of the fields tried would look as much like a counter as one whose chance of
/// doing so is 2^`log2_chance`, if their bits were random. A field with few changes or large
/// steps cannot look that much like one however its bits fall, and counts for nothing.
///
/// The fields beside fixed bits and the others are counted apart, and each count is doubled: a
/// counter's high bits do not change in a few presses, so it mostly lies beside fixed bits, and
/// there are far fewer such fields to look like one by accident.
fn log2_by_accident(tried: &Tried, beside_fixed: bool, log2_chance: f64) -> f64 {
    let mut expected = 0.0f64;
    for (&(beside, changes, width, max_step), &fields) in tried {
        if beside != beside_fixed {
            continue;
        }
        let chance = f64::from(max_step) / ((1u64 << width) - 1) as f64;
        // The likeliest outcome that is as unlikely as the one found.
        if let Some(reach) = log2_tails(changes, chance).into_iter().find(|t| *t <= log2_chance + 1e-9) {
            expected += fields as f64 * reach.exp2();
        }
    }
    1.0 + expected.log2()
}

/// The field among the `free` columns that most looks like a counter, when one does. `pairs` are
/// the frames that follow each other and differ.
///
/// Every run of free columns is read as a number both ways round, from every column as its low
/// bit and at every width, and its change from each frame to the next is taken modulo the width.
/// A counter's changes are all small steps one way. The chance that random bits do the same is
/// what counts, and a few changes that are no step are let through as jumps. Every column must
/// take part in a step: a button bit beside a counter flips without a carry into it, and is left
/// out.
fn best_counter(frames: &[Frame], pairs: &[(usize, usize)], free: &[bool]) -> Option<Counter> {
    let mut tried = Tried::new();
    // The best field that is not beside fixed bits, and the best that is.
    let mut best: [Option<(f64, Counter)>; 2] = [None, None];
    let mut values = vec![0u32; frames.len()];
    for low_first in [false, true] {
        for low in (0..free.len()).filter(|c| free[*c]) {
            values.fill(0);
            for w in 1..=MAX_COUNTER_BITS {
                // The column of the field's bit w - 1: before the low bit when the high bit is
                // sent first.
                let col = if low_first { Some(low + w - 1) } else { low.checked_sub(w - 1) };
                let Some(col) = col.filter(|c| *c < free.len() && free[*c]) else { break };
                for (v, f) in values.iter_mut().zip(frames) {
                    *v |= u32::from(f.bits[col]) << (w - 1);
                }
                if w < 2 {
                    continue;
                }
                let mask = (1u32 << w) - 1;
                let past = if low_first { Some(col + 1).filter(|c| *c < free.len()) } else { col.checked_sub(1) };
                let beside_fixed = past.is_some_and(|c| !free[c]);
                for down in [false, true] {
                    // Each change of the field: how far it went, and the bits that flipped.
                    let changes: Vec<(u32, u32)> = pairs
                        .iter()
                        .filter_map(|&(a, b)| {
                            let (from, to) = if down { (values[b], values[a]) } else { (values[a], values[b]) };
                            let step = to.wrapping_sub(from) & mask;
                            (step != 0).then_some((step, from ^ to))
                        })
                        .collect();
                    for max_step in STEPS.into_iter().take_while(|m| *m < 1 << (w - 1)) {
                        *tried.entry((beside_fixed, changes.len(), w, max_step)).or_default() += 1;
                        let steps = || changes.iter().filter(|(step, _)| *step <= max_step);
                        let fits = steps().count();
                        if fits < 2 || changes.len() - fits > changes.len() / 4 {
                            continue;
                        }
                        // A column above the step's own bits belongs when a step carried into it.
                        if steps().fold(max_step, |all, (_, flipped)| all | flipped) != mask {
                            continue;
                        }
                        let chance = f64::from(max_step) / f64::from(mask);
                        let evidence = -log2_tail(changes.len(), fits, chance);
                        let best = &mut best[usize::from(beside_fixed)];
                        if best.as_ref().is_none_or(|(e, _)| evidence > *e + 1e-9) {
                            let (smallest, largest) =
                                steps().fold((u32::MAX, 0), |(lo, hi), (step, _)| (lo.min(*step), hi.max(*step)));
                            let counter = Counter {
                                start: if low_first { low } else { col },
                                len: w,
                                low_first,
                                down,
                                steps: (smallest, largest),
                                changes: changes.len(),
                                fits,
                                sure: false,
                            };
                            *best = Some((evidence, counter));
                        }
                    }
                }
            }
        }
    }
    // Of the two, the one that is further beyond what chance explains.
    let mut found: Option<(f64, Counter)> = None;
    for (beside_fixed, best) in best.into_iter().enumerate() {
        let Some((evidence, counter)) = best else { continue };
        let beyond_chance = -log2_by_accident(&tried, beside_fixed == 1, -evidence);
        if found.as_ref().is_none_or(|(b, _)| beyond_chance > *b) {
            found = Some((beyond_chance, counter));
        }
    }
    let (beyond_chance, mut counter) = found?;
    counter.sure = beyond_chance >= SURE;
    (beyond_chance >= LIKELY).then_some(counter)
}

/// Find the counters among the columns that change, the surest first, and mark their columns.
fn find_counters(frames: &[Frame], pairs: &[(usize, usize)], classes: &mut [Class]) -> Vec<Counter> {
    let mut found = Vec::new();
    while found.len() < MAX_COUNTERS {
        let free: Vec<bool> = classes.iter().map(|c| *c == Class::Varies).collect();
        let Some(counter) = best_counter(frames, pairs, &free) else { break };
        for class in &mut classes[counter.start..counter.start + counter.len] {
            *class = Class::Counter(found.len());
        }
        found.push(counter);
    }
    found
}

/// The frames by group, each different frame once, in the order the groups first appear.
fn by_group(frames: &[Frame]) -> Vec<(&str, Vec<&Frame>)> {
    let mut groups: Vec<(&str, Vec<&Frame>)> = Vec::new();
    for frame in frames {
        let Some(name) = frame.group.as_deref() else { continue };
        let at = match groups.iter().position(|(g, _)| *g == name) {
            Some(at) => at,
            None => {
                groups.push((name, Vec::new()));
                groups.len() - 1
            }
        };
        if !groups[at].1.iter().any(|f| f.bits == frame.bits) {
            groups[at].1.push(frame);
        }
    }
    groups
}

/// How much groups with so many different frames each tell about a column that follows them, in
/// bits (-log2 of the number of the `columns` that would do so by accident). A random column is
/// the same within a group once in 2 for every further frame of it, and then differs between the
/// groups unless every group has the same value.
fn group_evidence(sizes: &[usize], columns: usize) -> f64 {
    let agreements: usize = sizes.iter().map(|n| n - 1).sum();
    let all_the_same = 0.5f64.powi(sizes.len() as i32 - 1);
    agreements as f64 - (1.0 - all_the_same).log2() - (columns as f64).log2()
}

/// The frames of each of `groups` groups from which a column that follows them shows among
/// `columns`.
fn frames_for_groups(groups: usize, columns: usize) -> usize {
    (2..).find(|n| group_evidence(&vec![*n; groups], columns) >= GROUP_LIKELY).unwrap_or(2)
}

/// Mark the columns that are the same within every group and differ between two of them, when
/// there are frames enough to tell that from chance.
fn mark_groups(frames: &[Frame], classes: &mut [Class]) {
    let groups = by_group(frames);
    let columns = classes.iter().filter(|c| **c == Class::Varies).count();
    if groups.len() < 2 || columns == 0 {
        return;
    }
    let sizes: Vec<usize> = groups.iter().map(|(_, frames)| frames.len()).collect();
    let evidence = group_evidence(&sizes, columns);
    if evidence < GROUP_LIKELY {
        return;
    }
    for (col, class) in classes.iter_mut().enumerate().filter(|(_, c)| **c == Class::Varies) {
        let within = groups.iter().all(|(_, frames)| frames.iter().all(|f| f.bits[col] == frames[0].bits[col]));
        let between = groups.iter().any(|(_, frames)| frames[0].bits[col] != groups[0].1[0].bits[col]);
        if within && between {
            *class = Class::Group { sure: evidence >= GROUP_SURE };
        }
    }
}

/// Mark the columns that change far less often than a coin toss, or every time.
fn mark_rates(frames: &[Frame], pairs: &[(usize, usize)], classes: &mut [Class]) {
    let columns = classes.iter().filter(|c| **c == Class::Varies).count() as f64;
    let unlikely = |log2_chance: f64| log2_chance + columns.log2() < RATE_CHANCE.log2();
    for (col, class) in classes.iter_mut().enumerate().filter(|(_, c)| **c == Class::Varies) {
        let flips = pairs.iter().filter(|(a, b)| frames[*a].bits[col] != frames[*b].bits[col]).count();
        if unlikely(log2_tail(pairs.len(), pairs.len() - flips, 0.5)) {
            *class = Class::Rare;
        } else if flips == pairs.len() && unlikely(log2_tail(pairs.len(), flips, 0.5)) {
            *class = Class::EveryTime;
        }
    }
}

/// How well `row` agrees with `first` when its first bit lies at bit `offset` of `first`: the bits
/// that agree less those that differ, where both have one.
fn agreement(first: &[bool], row: &[bool], offset: isize) -> isize {
    let from = offset.max(0);
    let to = (first.len() as isize).min(row.len() as isize + offset);
    (from..to).map(|i| if first[i as usize] == row[(i - offset) as usize] { 1 } else { -1 }).sum()
}

/// Line up rows that hold one kind of frame read from a few bits earlier or later (a preamble
/// caught at another point, a bit more at the end): each row is moved by up to `max_shift` bits to
/// where it agrees best with the first, and all are cut to the bits every row has. What a remote
/// sends the same every time (preamble, sync, serial) is what puts a row in its place.
pub fn align(rows: &[&[bool]], max_shift: usize) -> Vec<Vec<bool>> {
    let Some(first) = rows.first() else { return Vec::new() };
    let max = max_shift as isize;
    // Where each row's first bit lies, counted from the first row's; the smaller move on a tie.
    let offsets: Vec<isize> = rows
        .iter()
        .map(|row| (-max..=max).max_by_key(|o| (agreement(first, row, *o), -o.abs())).unwrap_or(0))
        .collect();
    let start = offsets.iter().copied().max().unwrap_or(0);
    let end = rows.iter().zip(&offsets).map(|(row, o)| o + row.len() as isize).min().unwrap_or(0);
    rows.iter()
        .zip(&offsets)
        .map(|(row, o)| if end > start { row[(start - o) as usize..(end - o) as usize].to_vec() } else { Vec::new() })
        .collect()
}

/// Compare `frames`, in their order. They are cut to the shortest one's length: the caller gives
/// frames of one length ([`align`] for rows read from the air).
pub fn compare(mut frames: Vec<Frame>, what: String, grouping: Grouping, notes: Vec<String>) -> Comparison {
    let bits = frames.iter().map(|f| f.bits.len()).min().unwrap_or(0);
    for frame in &mut frames {
        frame.bits.truncate(bits);
    }
    let pairs: Vec<(usize, usize)> =
        (1..frames.len()).filter(|i| frames[*i].bits != frames[i - 1].bits).map(|i| (i - 1, i)).collect();
    let mut classes: Vec<Class> = (0..bits)
        .map(|col| match frames.iter().all(|f| f.bits[col] == frames[0].bits[col]) {
            true => Class::Fixed,
            false => Class::Varies,
        })
        .collect();
    let counters = find_counters(&frames, &pairs, &mut classes);
    mark_groups(&frames, &mut classes);
    mark_rates(&frames, &pairs, &mut classes);
    let distinct = frames.iter().map(|f| &f.bits).collect::<BTreeSet<_>>().len();
    Comparison { frames, bits, what, grouping, notes, classes, counters, distinct }
}

/// Bits as a hex number, the first one the highest: "0x1F" (five bits: "0x1F", two: "0x3").
fn hex(bits: &[bool]) -> String {
    let lead = bits.len() % 4;
    let digit = |chunk: &[bool]| {
        let v = chunk.iter().fold(0u32, |v, b| (v << 1) | u32::from(*b));
        char::from_digit(v, 16).unwrap_or('?').to_ascii_uppercase()
    };
    let mut text = String::from("0x");
    if lead > 0 {
        text.push(digit(&bits[..lead]));
    }
    text.extend(bits[lead..].chunks(4).map(digit));
    text
}

/// "bit 7", "bits 8-39".
fn range(start: usize, len: usize) -> String {
    match len {
        1 => format!("bit {start}"),
        _ => format!("bits {start}-{}", start + len - 1),
    }
}

/// "1 bit", "8 bits".
fn count(bits: usize) -> String {
    format!("{bits} bit{}", plural(bits))
}

impl Comparison {
    /// The runs of neighbouring columns of one class, in order.
    pub fn fields(&self) -> Vec<Field> {
        let mut fields: Vec<Field> = Vec::new();
        for (col, class) in self.classes.iter().enumerate() {
            match fields.last_mut() {
                Some(last) if last.class == *class => last.len += 1,
                _ => fields.push(Field { start: col, len: 1, class: *class }),
            }
        }
        fields
    }

    /// The groups' names, in the order they first appear.
    fn groups(&self) -> Vec<&str> {
        by_group(&self.frames).into_iter().map(|(name, _)| name).collect()
    }

    /// How many columns have a class that `pick` takes.
    fn columns(&self, pick: impl Fn(Class) -> bool) -> usize {
        self.classes.iter().filter(|c| pick(**c)).count()
    }

    /// A field's value in a frame.
    fn value(&self, field: &Field, frame: &Frame) -> String {
        hex(&frame.bits[field.start..field.start + field.len])
    }

    /// The different values of a field, in the order they first appear.
    fn values(&self, field: &Field) -> Vec<String> {
        let mut seen: Vec<String> = Vec::new();
        for frame in &self.frames {
            let value = self.value(field, frame);
            if !seen.contains(&value) {
                seen.push(value);
            }
        }
        seen
    }

    /// What a field is and the values it takes, for the table of fields.
    fn describe(&self, field: &Field) -> String {
        let values = self.values(field);
        let listed = |values: &[String]| match values.len() {
            n if n <= LISTED_VALUES => values.join(", "),
            n => format!("{n} values in {} frames", self.frames.len()),
        };
        match field.class {
            Class::Fixed => format!("fixed: {}", values[0]),
            Class::Counter(n) => {
                let counter = &self.counters[n];
                let shown = |frame: &Frame| format!("0x{:X}", counter.value(frame));
                match (self.frames.first(), self.frames.last()) {
                    (Some(first), Some(last)) => format!("{}: {} to {}", counter.text(false), shown(first), shown(last)),
                    _ => counter.text(false),
                }
            }
            Class::Group { sure } => {
                let per_group: Vec<String> = by_group(&self.frames)
                    .iter()
                    .map(|(name, frames)| format!("{name} {}", self.value(field, frames[0])))
                    .collect();
                let verb = if sure { "follows" } else { "may follow" };
                format!("{verb} the {}: {}", self.grouping.word(), per_group.join(", "))
            }
            Class::Rare => format!("changes rarely: {}", listed(&values)),
            Class::EveryTime => "changes with every frame".to_string(),
            Class::Varies => format!("no pattern: {}", listed(&values)),
        }
    }

    /// The report as lines of at most `width` characters.
    pub fn render(&self, width: usize) -> Vec<Line> {
        let mut out = Out { lines: Vec::new(), width: width.max(MIN_WIDTH) };
        self.summary(&mut out);
        if self.bits > 0 && !self.frames.is_empty() {
            self.field_table(&mut out);
            self.bit_rows(&mut out);
        }
        out.lines
    }

    fn summary(&self, out: &mut Out) {
        let frames = self.frames.len();
        out.heading("SUMMARY", "");
        let compared = format!("{frames} frame{} of {}: {}", plural(frames), count(self.bits), self.what);
        out.field("Compared", &compared, Tone::Plain);
        let fixed = self.columns(|c| c == Class::Fixed);
        out.field("Fixed", &format!("{fixed} of {}", count(self.bits)), Tone::Plain);

        for counter in &self.counters {
            out.field("Counter", &format!("{} {}", range(counter.start, counter.len), counter.text(true)), Tone::Good);
        }
        let groups = self.groups();
        let word = self.grouping.word();
        let label = if self.grouping == Grouping::Button { "Button" } else { "Group" };
        let follow: Vec<Field> = self.fields().into_iter().filter(|f| matches!(f.class, Class::Group { .. })).collect();
        if !follow.is_empty() {
            let sure = follow.iter().all(|f| f.class == Class::Group { sure: true });
            let places: Vec<String> = follow.iter().map(|f| range(f.start, f.len)).collect();
            let verb = match (sure, follow.len() == 1 && follow[0].len == 1) {
                (true, true) => "follows",
                (true, false) => "follow",
                (false, _) => "may follow",
            };
            out.field(label, &format!("{} {verb} the {word} ({})", places.join(", "), groups.join(", ")), Tone::Accent);
        }
        let rare = self.columns(|c| c == Class::Rare);
        let every = self.columns(|c| c == Class::EveryTime);
        let varies = self.columns(|c| c == Class::Varies);
        let mut changing = Vec::new();
        if varies > 0 {
            changing.push(format!("{} with no pattern", count(varies)));
        }
        if rare > 0 {
            changing.push(format!("{} that change{} rarely", count(rare), if rare == 1 { "s" } else { "" }));
        }
        if every > 0 {
            changing.push(format!("{} that change{} with every frame", count(every), if every == 1 { "s" } else { "" }));
        }
        if !changing.is_empty() {
            out.field("Changing", &changing.join(", "), Tone::Plain);
        }

        for note in &self.notes {
            out.note(note, Tone::Warn);
        }
        for note in self.advice(&groups, varies) {
            out.note(&note, Tone::Dim);
        }
    }

    /// What the frames given cannot tell, and what would.
    fn advice(&self, groups: &[&str], varies: usize) -> Vec<String> {
        let word = self.grouping.word();
        let mut notes = Vec::new();
        match self.distinct {
            0 => return notes,
            1 if self.frames.len() > 1 => {
                notes.push("Every frame is the same: nothing changes from press to press (a fixed code), or these are repeats of one press.".to_string());
                return notes;
            }
            1 => {
                notes.push("One frame: nothing to compare it with.".to_string());
                return notes;
            }
            2 => notes.push(
                "Two different frames show only which bits differ. Five or more presses of one button show a counter, and then a few of another button show the button's bits.".to_string(),
            ),
            _ => {}
        }
        let unmarked = self.columns(|c| !matches!(c, Class::Fixed | Class::Counter(_)));
        let found = self.columns(|c| matches!(c, Class::Group { .. })) > 0;
        if self.distinct > 2 && unmarked > 0 && !found {
            if groups.is_empty() {
                notes.push(match self.grouping {
                    Grouping::Button => "No button is known for these frames. Name each capture's button (i, Command), or give the groups: :compare 1-5 / 6-10.".to_string(),
                    Grouping::Group => "No groups given.".to_string(),
                });
            } else if groups.len() == 1 {
                notes.push(format!("Every frame is of one {word} ({}): add presses of another to see which bits follow it.", groups[0]));
            } else {
                let sizes: Vec<usize> = by_group(&self.frames).iter().map(|(_, f)| f.len()).collect();
                notes.push(if sizes.iter().all(|n| *n == 1) {
                    format!("Each {word} gives one frame here (a fixed code, or one press of each): the bits that differ may be the {word}'s, or change by themselves.")
                } else if group_evidence(&sizes, unmarked) < GROUP_LIKELY {
                    format!(
                        "Too few frames of each {word} to tell its bits from ones that change by themselves: about {} different ones of each would do.",
                        frames_for_groups(sizes.len(), unmarked)
                    )
                } else {
                    format!("No bit follows the {word}: it may be among the bits with no pattern (encrypted with the rest).")
                });
            }
        }
        if varies > 0 && self.distinct > 2 {
            notes.push("Bits with no pattern are encrypted or hashed (a rolling code, a checksum), or follow something these frames do not show.".to_string());
        }
        notes
    }

    fn field_table(&self, out: &mut Out) {
        let fields = self.fields();
        out.heading("FIELDS", "bit 0 is sent first, values high bit first");
        let places: Vec<String> = fields
            .iter()
            .map(|f| match f.len {
                1 => f.start.to_string(),
                _ => format!("{}-{}", f.start, f.start + f.len - 1),
            })
            .collect();
        let place_width = places.iter().map(|p| p.len()).max().unwrap_or(0).max(4);
        let len_width = self.bits.to_string().len().max(4);
        let head = |place: &str, len: &str| format!(" {place:<place_width$}  {len:>len_width$}  ");
        out.row(&format!("{}What they do", head("Bits", "Size")), Tone::Label);
        let indent = head("", "").chars().count();
        for (field, place) in fields.iter().zip(&places) {
            let room = out.width.saturating_sub(indent);
            for (i, part) in wrap(&self.describe(field), room).into_iter().enumerate() {
                let lead = if i == 0 { head(place, &field.len.to_string()) } else { " ".repeat(indent) };
                out.lines.push(vec![span(lead, field.class.tone()), span(part, Tone::Plain)]);
            }
        }
        let high_bits_fixed = self.counters.iter().any(|c| {
            let beside = if c.low_first { Some(c.start + c.len).filter(|at| *at < self.bits) } else { c.start.checked_sub(1) };
            beside.is_some_and(|at| self.classes[at] == Class::Fixed)
        });
        if high_bits_fixed {
            out.note(
                "A counter's high bits do not change in a few presses: they are among the fixed bits beside it.",
                Tone::Dim,
            );
        }
    }

    /// The frames' bits in blocks, a letter under every column saying what it is.
    fn bit_rows(&self, out: &mut Out) {
        let labels: Vec<String> = self
            .frames
            .iter()
            .map(|f| match &f.group {
                Some(group) => cut(&format!("{} {group}", f.name), MAX_LABEL),
                None => cut(&f.name, MAX_LABEL),
            })
            .collect();
        let label_width = labels.iter().map(|l| l.chars().count()).max().unwrap_or(0).max(4);
        // A block of 64 bits where it fits, else of 32 (which fits the narrowest report).
        let per_block = if out.width >= label_width + 2 + 64 + 7 { 64 } else { 32 };

        let used: BTreeSet<char> = self.classes.iter().map(|c| c.letter()).collect();
        let legend: Vec<&str> = [
            ('.', ". fixed"),
            ('C', "C counter"),
            ('B', if self.grouping == Grouping::Button { "B button" } else { "B group" }),
            ('r', "r rarely"),
            ('e', "e every frame"),
            ('?', "? no pattern"),
        ]
        .into_iter()
        .filter(|(letter, _)| used.contains(letter))
        .map(|(_, text)| text)
        .collect();
        out.heading("BITS", &legend.join("  "));

        // A row of the block: its label, then a character per column in groups of eight, the
        // columns of one class in one span.
        let row = |label: &str, label_tone: Tone, from: usize, to: usize, cell: &dyn Fn(usize) -> char| -> Line {
            let mut line = vec![span(format!(" {label:<label_width$} "), label_tone)];
            for col in from..to {
                let tone = self.classes[col].tone();
                let mut text = String::new();
                if col > from && (col - from) % 8 == 0 {
                    text.push(' ');
                }
                text.push(cell(col));
                match line.last_mut() {
                    // (The block's first column starts a span of its own, after the label.)
                    Some(Span { text: last, tone: t }) if *t == tone && col > from => last.push_str(&text),
                    _ => line.push(span(text, tone)),
                }
            }
            line
        };

        for from in (0..self.bits).step_by(per_block) {
            let to = (from + per_block).min(self.bits);
            let place = range(from, to - from);
            if self.classes[from..to].iter().all(|c| *c == Class::Fixed) {
                // Nothing to see in a block that never changes: one line.
                let value = self.frames.first().map(|f| hex(&f.bits[from..to])).unwrap_or_default();
                out.row(&format!(" {place}  fixed: {value}"), Tone::Dim);
                continue;
            }
            out.row(&format!(" {place}"), Tone::Label);
            for (frame, label) in self.frames.iter().zip(&labels) {
                out.lines.push(row(label, Tone::Label, from, to, &|col| if frame.bits[col] { '1' } else { '0' }));
            }
            out.lines.push(row("", Tone::Label, from, to, &|col| self.classes[col].letter()));
            if to < self.bits {
                out.lines.push(Vec::new());
            }
        }
    }
}

#[cfg(test)]
mod tests;
