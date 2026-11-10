//! The report's tables: pulse widths, rows, bits, waveform, protocol timing and raw durations.

use super::render::{cut, plural, span, time, wrap, Line, Out, Tone};
use super::{KnownTiming, Row, RowKind, SignalReport};
use crate::analysis::timing::{self, AnalysisWindow, Cluster, LineCoding, TeEstimate, TimingVerdict, WidthVerdict};
use crate::capture::{RfModulation, StoredLevelDuration};

/// Widths listed per level, rows listed, and rows shown as bits.
const MAX_WIDTH_ROWS: usize = 6;
const MAX_ROWS: usize = 12;
const MAX_BIT_ROWS: usize = 4;
/// Rows longer than this many bits are shown as hex only.
const MAX_BINARY_BITS: usize = 192;
const WAVE_LINES: usize = 5;
const RAW_LINES: usize = 4;
/// Durations the raw section reads at most.
const RAW_DURATIONS: usize = 240;
/// Columns of a count bar, and of the longest duration a waveform draws in full.
const BAR: usize = 10;
const WAVE_MAX: usize = 8;

fn range(min: u32, max: u32) -> String {
    if max < 10_000 {
        format!("{min}..{max}")
    } else {
        format!("{:.1}..{:.1} ms", f64::from(min) / 1e3, f64::from(max) / 1e3)
    }
}

/// `text` with a space after every `n` characters.
fn grouped(text: &str, n: usize) -> String {
    let chars: Vec<char> = text.chars().collect();
    chars.chunks(n).map(|c| c.iter().collect::<String>()).collect::<Vec<_>>().join(" ")
}

/// One level's widths: the common ones in full, the rest in a line.
fn level_widths(out: &mut Out, high: bool, what: &str, clusters: &[Cluster], te: Option<TeEstimate>) {
    let total: usize = clusters.iter().map(|c| c.count).sum();
    let (label, tone) = if high { (" HIGH", Tone::High) } else { (" LOW", Tone::Low) };
    out.lines.push(vec![span(label, tone), span(format!(" ({what}): {total} duration{}", plural(total)), Tone::Dim)]);
    let mut shown = timing::major_clusters(clusters);
    if shown.is_empty() {
        shown = clusters.iter().collect();
        shown.sort_by_key(|c| std::cmp::Reverse(c.count));
        shown.truncate(3);
        shown.sort_by(|a, b| a.mean.total_cmp(&b.mean));
    }
    shown.truncate(MAX_WIDTH_ROWS);
    let most = shown.iter().map(|c| c.count).max().unwrap_or(1).max(1);
    for c in &shown {
        let multiple = te
            .map(|t| t.multiple(high, c.mean))
            .filter(|m| (0.05..99.5).contains(m))
            .map_or("-".to_string(), |m| format!("{m:.1}"));
        let text = format!(
            "  {:>9}  {:>6}  {:>13}  {:>6}  ",
            time(c.mean.round() as u64),
            format!("x{}", c.count),
            range(c.min, c.max),
            multiple
        );
        let room = out.width.saturating_sub(text.chars().count());
        let bar = "█".repeat((c.count * BAR).div_ceil(most).clamp(1, BAR).min(room));
        out.lines.push(vec![span(cut(&text, out.width), Tone::Plain), span(bar, tone)]);
    }
    let rest = clusters.len() - shown.len();
    if rest > 0 {
        let n = total - shown.iter().map(|c| c.count).sum::<usize>();
        out.row(&format!("      + {rest} other width{} ({n} duration{}): pauses, sync or noise", plural(rest), plural(n)), Tone::Dim);
    }
}

fn coding_text(coding: &LineCoding, te: Option<TeEstimate>) -> String {
    let rate = match (coding, te) {
        (LineCoding::Pwm { period_us }, _) if *period_us > 0.0 => Some(1e6 / period_us),
        (LineCoding::Manchester, Some(te)) => Some(5e5 / te.te_us),
        (LineCoding::Nrz, Some(te)) => Some(1e6 / te.te_us),
        _ => None,
    };
    match rate {
        Some(rate) => format!("{}; {rate:.0} bit/s", coding.describe()),
        None => coding.describe(),
    }
}

/// How the trial decode reads a bit, per coding.
fn convention(coding: &LineCoding) -> &'static str {
    match coding {
        LineCoding::Pwm { .. } | LineCoding::PwmFixedGap => "a long pulse is 1, a short pulse is 0",
        LineCoding::PulseLength => "a long pulse and gap is 1, a short pair is 0",
        LineCoding::Ppm => "a long gap is 1, a short gap is 0",
        LineCoding::Manchester => {
            "HIGH then LOW is 1, LOW then HIGH is 0, and a leading run of one value is left out as preamble"
        }
        _ => "HIGH is 1, LOW is 0, one bit per TE",
    }
}

impl SignalReport {
    pub(super) fn widths(&self, out: &mut Out) {
        let t = &self.timing;
        let (first, last) = (t.window_start + 1, t.window_start + t.pair_count);
        let window = match t.window {
            AnalysisWindow::All => format!("all {} durations", t.pair_count),
            AnalysisWindow::DecodedFrame => format!("durations {first}..{last} of {}: the decoded frame", t.capture_pairs),
            AnalysisWindow::MostRegular => format!("durations {first}..{last} of {}: the most regular stretch", t.capture_pairs),
        };
        out.heading("PULSE WIDTHS", &window);
        out.row(&format!("  {:>9}  {:>6}  {:>13}  {:>6}", "width", "count", "min..max", "x TE"), Tone::Dim);
        let fm = self.source.path == Some(RfModulation::FM);
        // Widths that are no multiples of anything get no "x TE" column: it would read as a finding.
        let te = self.te().filter(|_| self.regular);
        level_widths(out, true, if fm { "upper tone" } else { "carrier on" }, &t.high, te);
        level_widths(out, false, if fm { "lower tone" } else { "carrier off" }, &t.low, te);
        let Some(te) = te else {
            let none = match self.te() {
                Some(t) => format!(
                    "none: the nearest, ~{:.0} us, has only {:.0}% of the durations at 1-4 times it",
                    t.te_us,
                    t.explained * 100.0
                ),
                None => "none: no width stands out".to_string(),
            };
            out.field("TE", &none, Tone::Warn);
            out.field("Coding", &t.coding.describe(), Tone::Plain);
            return;
        };
        let levels = match (te.high_us, te.low_us) {
            (Some(h), Some(l)) => format!(" (1 TE is {h:.0} us HIGH, {l:.0} us LOW)"),
            (Some(h), None) => format!(" (from HIGH: {h:.0})"),
            (None, Some(l)) => format!(" (from LOW: {l:.0})"),
            (None, None) => String::new(),
        };
        out.field(
            "TE",
            &format!("~{:.0} us{levels}; {:.0}% of the durations are 1-4 x TE", te.te_us, te.explained * 100.0),
            Tone::Plain,
        );
        // Half the distance between the levels' 1 TE widths: what each edge is off by.
        let skew = match (te.high_us, te.low_us) {
            _ if te.skew_us != 0.0 => te.skew_us,
            (Some(h), Some(l)) if h / l < 1.6 && l / h < 1.6 => (h - l) / 2.0,
            _ => 0.0,
        };
        if skew.abs() >= 10.0 {
            let (way, other) = if skew > 0.0 { ("longer", "shorter") } else { ("shorter", "longer") };
            let whose = if fm { "the receiver's slicer sits off the middle of the two tones" } else { "the receiver's doing" };
            out.field(
                "Skew",
                &format!("every HIGH is ~{:.0} us {way} and every LOW as much {other} than sent: {whose}", skew.abs()),
                Tone::Dim,
            );
        }
        out.field("Symbol rate", &format!("{:.0} baud (one symbol per TE)", 1e6 / te.te_us), Tone::Plain);
        out.field("Coding", &coding_text(&t.coding, Some(te)), Tone::Plain);
    }

    fn row_content(&self, r: &Row) -> (String, Tone) {
        match &r.kind {
            RowKind::Preamble { high_us, low_us } => (format!("one width: {high_us}/{low_us} us"), Tone::Dim),
            RowKind::Long { us } => (format!("long pulse: {}", time(u64::from(*us))), Tone::Dim),
            RowKind::Pulses => ("not read as bits".to_string(), Tone::Dim),
            RowKind::Data { bits, lead, tail, same_as, differs } => {
                let mut text = match bits.preamble {
                    0 => String::new(),
                    n => format!("{n} preamble + "),
                };
                text.push_str(&format!("{} bits", bits.data().len()));
                if *lead >= 2 {
                    text.push_str(&format!(" after {} pulses", lead / 2));
                }
                if *tail >= 2 {
                    text.push_str(&format!(", {} more", tail / 2));
                }
                match (same_as, differs) {
                    (Some(j), _) => text.push_str(&format!(", same as #{}", j + 1)),
                    (None, Some((j, n))) => text.push_str(&format!(", {n} unlike #{}", j + 1)),
                    _ => {}
                }
                (text, Tone::Plain)
            }
        }
    }

    /// The rows the table lists (indices, in the order sent) when there are more than it has room
    /// for: the row the waveform shows, then the other rows of bits, the preambles, the unread
    /// rows and the lone long pulses (mostly noise), the earliest of each first. The sections below
    /// name rows by number, so the rows of bits must be in the table.
    fn listed_rows(&self) -> Vec<usize> {
        let rank = |i: &usize| match self.rows[*i].kind {
            RowKind::Data { .. } => 0,
            RowKind::Preamble { .. } => 1,
            RowKind::Pulses => 2,
            RowKind::Long { .. } => 3,
        };
        let mut rest: Vec<usize> = (0..self.rows.len()).collect();
        rest.sort_by_key(rank);
        let mut keep: Vec<usize> = self.shown_row().into_iter().collect();
        for i in rest {
            if keep.len() == MAX_ROWS {
                break;
            }
            if !keep.contains(&i) {
                keep.push(i);
            }
        }
        keep.sort_unstable();
        keep
    }

    pub(super) fn frames(&self, out: &mut Out) {
        out.heading("ROWS", &format!("a pause of {} or more ends a row", time(u64::from(self.gap_us))));
        if self.rows.is_empty() {
            out.note("No row of data, no preamble and no long pulse.", Tone::Dim);
        } else {
            out.row(&format!(" {:>3}  {:>8}  {:>6}  {:>8}  content", "#", "at", "pulses", "then"), Tone::Dim);
        }
        let listed = self.listed_rows();
        for (i, r) in listed.iter().map(|&i| (i, &self.rows[i])) {
            let highs = self.pulses[r.start..r.start + r.len].iter().filter(|p| p.level).count();
            let then = r.then_us.map_or("-".to_string(), time);
            let fixed = format!(" {:>3}  {:>8}  {:>6}  {:>8}  ", i + 1, time(r.at_us), highs, then);
            let (content, tone) = self.row_content(r);
            let room = out.width.saturating_sub(fixed.chars().count());
            out.lines.push(vec![span(fixed, Tone::Plain), span(cut(&content, room), tone)]);
        }
        if let Some(more) = self.rows.len().checked_sub(listed.len()).filter(|&n| n > 0) {
            // What is left out, so the gaps in the numbers are no puzzle.
            let hidden = |i: &usize| !listed.contains(i);
            let bits = self.data_rows().filter(|(n, _, _)| hidden(&(n - 1))).count();
            let what = match bits {
                0 => "without bits",
                n if n == more => "of bits",
                _ => "",
            };
            out.row(format!("      + {more} more row{} {what}", plural(more)).trim_end(), Tone::Dim);
        }
        if let Some(rep) = self.repeat {
            let what = match rep.rows {
                1 => format!("All {} rows have the same shape", self.rows.len()),
                n => format!("Rows #1 to #{n} come round {} times", rep.times),
            };
            let pace = match rep.every_us {
                0 => "at no steady pace".to_string(),
                us => format!("{} from one to the next", time(us)),
            };
            out.note(&format!("{what}, {pace}."), Tone::Plain);
        }
        if self.fragments > 0 {
            out.note(
                &format!(
                    "{} other piece{} ({} durations) are none of these: noise or broken rows.",
                    self.fragments,
                    plural(self.fragments),
                    self.fragment_pulses
                ),
                Tone::Dim,
            );
        }
    }

    pub(super) fn bit_rows(&self, out: &mut Out) {
        let distinct: Vec<_> =
            self.data_rows().filter(|(_, r, _)| matches!(r.kind, RowKind::Data { same_as: None, .. })).collect();
        if distinct.is_empty() {
            return;
        }
        out.heading("BITS", "a trial decode, the first bit sent on the left");
        let room = out.width.saturating_sub(11);
        // The row the waveform shows is one of them, however many come before it.
        let shown = self.shown_row().map(|i| i + 1);
        let first = distinct.iter().filter(|(n, _, _)| Some(*n) == shown);
        let mut listed: Vec<_> = first.chain(distinct.iter().filter(|(n, _, _)| Some(*n) != shown)).take(MAX_BIT_ROWS).collect();
        listed.sort_by_key(|(n, _, _)| *n);
        for (n, _, bits) in listed {
            let count = bits.data().len();
            out.lines.push(vec![span(format!(" #{n:<4}"), Tone::Label), span(format!("{count} bits"), Tone::Plain)]);
            for (label, text, group, tone) in
                [("hex", bits.hex(), 4, Tone::Accent), ("bin", bits.binary(), 8, Tone::Plain)]
            {
                if label == "bin" && count > MAX_BINARY_BITS {
                    continue;
                }
                for (i, part) in wrap(&grouped(&text, group), room).into_iter().enumerate() {
                    let label = if i == 0 { label } else { "" };
                    out.lines.push(vec![span(format!("      {label:<4} "), Tone::Label), span(part, tone)]);
                }
            }
        }
        if distinct.len() > MAX_BIT_ROWS {
            out.row(&format!("      + {} more rows with other bits", distinct.len() - MAX_BIT_ROWS), Tone::Dim);
        }
        out.note(
            &format!("Read as: {}. A protocol may use the opposite, or code the bits again.", convention(&self.timing.coding)),
            Tone::Dim,
        );
    }

    /// What the waveform and the raw durations show, and its name: the row with the most bits (the
    /// frame, not a piece of preamble that read as a few bits; the first of them when several are
    /// as long), else the first row of several pulses, else (long pulses only, or nothing listed)
    /// the capture's start.
    fn shown(&self) -> Option<(String, &[StoredLevelDuration])> {
        match self.shown_row().map(|i| (i + 1, &self.rows[i])) {
            Some((n, r)) => Some((format!("row #{n}"), &self.pulses[r.start..r.start + r.len])),
            None if self.pulses.is_empty() => None,
            None => Some(("the capture from its start".to_string(), &self.pulses[..])),
        }
    }

    /// The row (index) the waveform and the raw durations show, if a row is.
    fn shown_row(&self) -> Option<usize> {
        let several = || self.rows.iter().position(|r| r.len > 2);
        let frame = self.data_rows().min_by_key(|(_, _, bits)| std::cmp::Reverse(bits.data().len()));
        frame.map(|(n, _, _)| n - 1).or_else(several)
    }

    pub(super) fn waveform(&self, out: &mut Out) {
        let Some((what, pulses)) = self.shown() else { return };
        let unit = self.te().filter(|_| self.regular).map(|t| t.te_us).unwrap_or_else(|| {
            let mut d: Vec<u32> = pulses.iter().map(|p| p.duration_us).collect();
            d.sort_unstable();
            f64::from(d.get(d.len() / 2).copied().unwrap_or(100).max(1))
        });
        out.heading("WAVEFORM", &format!("{what}, one column is {unit:.0} us"));
        let room = out.width.saturating_sub(2).max(8);
        let mut cells: Vec<(char, Tone)> = Vec::new();
        for p in pulses.iter().take(room * (WAVE_LINES + 1)) {
            let cols = ((f64::from(p.duration_us) / unit).round() as usize).max(1);
            let cell = if p.level { ('▀', Tone::High) } else { ('▁', Tone::Low) };
            if cols > WAVE_MAX {
                cells.extend([cell; 3]);
                cells.push(('…', Tone::Dim));
                cells.extend([cell; 3]);
            } else {
                cells.extend(std::iter::repeat_n(cell, cols));
            }
        }
        for chunk in cells.chunks(room).take(WAVE_LINES) {
            let mut line: Line = vec![span(" ", Tone::Plain)];
            for (glyph, tone) in chunk {
                match line.last_mut() {
                    Some(last) if last.tone == *tone => last.text.push(*glyph),
                    _ => line.push(span(glyph.to_string(), *tone)),
                }
            }
            out.lines.push(line);
        }
        if cells.len() > room * WAVE_LINES {
            out.row(" … and so on", Tone::Dim);
        }
    }

    pub(super) fn reference(&self, out: &mut Out) {
        let t = &self.timing;
        let Some(cmp) = &t.comparison else {
            if let Some(proto) = &t.protocol {
                out.heading("PROTOCOL TIMING", "");
                out.note(&format!("KAT has no timing definition for {proto} to compare with."), Tone::Dim);
            } else if self.is_regular() {
                self.similar_timing(out);
            }
            return;
        };
        let name = t.protocol.as_deref().unwrap_or("?");
        out.heading("PROTOCOL TIMING", &format!("{name}: {}/{} us, +/-{} us", cmp.te_short, cmp.te_long, cmp.te_delta));
        out.row(&format!(" {:<6}{:>6}{:>6}{:>6}{:>7}{:>7}", "", "avg", "min", "max", "count", "diff"), Tone::Dim);
        for (label, s, diff, verdict) in [
            ("short", &cmp.short, cmp.short_diff, cmp.short_verdict()),
            ("long", &cmp.long, cmp.long_diff, cmp.long_verdict()),
        ] {
            let (word, tone) = match verdict {
                WidthVerdict::Excellent => ("EXCELLENT".to_string(), Tone::Good),
                WidthVerdict::Ok(_) => ("OK".to_string(), Tone::Good),
                WidthVerdict::High(d) => (format!("HIGH by {d}"), Tone::Warn),
                WidthVerdict::Low(d) => (format!("LOW by {d}"), Tone::Warn),
                WidthVerdict::NoSamples => ("NO SAMPLES".to_string(), Tone::Dim),
            };
            let numbers = match s.count {
                0 => format!(" {label:<6}{:>6}{:>6}{:>6}{:>7}{:>7}  ", "-", "-", "-", 0, "-"),
                n => format!(" {label:<6}{:>6}{:>6}{:>6}{n:>7}{diff:>+7}  ", s.avg, s.min, s.max),
            };
            out.lines.push(vec![span(numbers, Tone::Plain), span(word, tone)]);
        }
        let tone = match cmp.verdict() {
            TimingVerdict::Matches | TimingVerdict::WithinTolerance => Tone::Good,
            TimingVerdict::NeedsAdjustment => Tone::Warn,
            _ => Tone::Dim,
        };
        out.note(&cmp.conclusion_lines().join(" "), tone);
        out.note(
            &format!(
                "Both levels count: short is under {} us, long from there up; {}..{} us is read, {} left out.",
                cmp.threshold, cmp.min_valid, cmp.max_valid, cmp.ignored
            ),
            Tone::Dim,
        );
        if t.variants > 1 {
            let fits = format!("{name} is sent with {} timings: this is the one the capture fits.", t.variants);
            out.note(&fits, Tone::Dim);
        }
    }

    fn similar_timing(&self, out: &mut Out) {
        out.heading("SIMILAR TIMING", "KAT's protocols near these widths; not a decode");
        // The columns are as wide as what they hold here; the name gives way on a narrow screen.
        let widest = |text: fn(&KnownTiming) -> usize| self.similar.iter().map(text).max().unwrap_or(0);
        let (coding, rf) = (widest(|k| k.coding.chars().count()), widest(|k| k.rf.chars().count()));
        let widths = widest(|k| format!("{}/{} us", k.te_short, k.te_long).chars().count());
        let rest = 1 + 2 + widths + 2 + coding + 2 + rf;
        let name = widest(|k| k.name.chars().count()).min(out.width.saturating_sub(rest).max(8));
        for k in &self.similar {
            let text = format!(
                " {:<name$}  {:>widths$}  {:<coding$}  {}",
                cut(k.name, name),
                format!("{}/{} us", k.te_short, k.te_long),
                k.coding,
                k.rf
            );
            out.row(text.trim_end(), Tone::Plain);
        }
        if self.similar.is_empty() {
            out.note("None of KAT's protocols is defined with widths like these.", Tone::Dim);
        }
        if let Some((short, long)) = self.timing.suggested_definition() {
            let long = long.map(|l| format!(", te_long ~{l} us")).unwrap_or_default();
            out.field("As a protocol", &format!("te_short ~{short} us{long}"), Tone::Plain);
        }
    }

    pub(super) fn raw(&self, out: &mut Out) {
        let Some((what, pulses)) = self.shown() else { return };
        out.heading("RAW", &format!("{what} in us: + is HIGH, - is LOW"));
        let text: Vec<String> = pulses
            .iter()
            .take(RAW_DURATIONS)
            .map(|p| format!("{}{}", if p.level { '+' } else { '-' }, p.duration_us))
            .collect();
        let lines = wrap(&text.join(" "), out.width.saturating_sub(2));
        for line in lines.iter().take(RAW_LINES) {
            out.row(&format!(" {line}"), Tone::Plain);
        }
        if lines.len() > RAW_LINES || pulses.len() > RAW_DURATIONS {
            out.row(&format!(" … {} durations in all", pulses.len()), Tone::Dim);
        }
    }
}
