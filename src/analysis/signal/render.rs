//! The report as rows of tone-tagged text for a given width: the summary and the modulation here,
//! the tables in [`super::tables`].

use std::collections::BTreeMap;

use super::{Keying, RowKind, SignalReport};
use crate::capture::RfModulation;

/// What a piece of text is, for the UI to style.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tone {
    Plain,
    Heading,
    Label,
    Dim,
    Good,
    Warn,
    /// The HIGH level of a waveform, and the LOW one.
    High,
    Low,
    Accent,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Span {
    pub text: String,
    pub tone: Tone,
}

pub type Line = Vec<Span>;

/// Width of the label column of a field row.
const LABEL: usize = 12;
/// The narrowest the report is laid out for: its tables are this wide.
pub const MIN_WIDTH: usize = 54;

pub(super) fn span(text: impl Into<String>, tone: Tone) -> Span {
    Span { text: text.into(), tone }
}

pub(super) fn plural(n: usize) -> &'static str {
    if n == 1 {
        ""
    } else {
        "s"
    }
}

/// A duration for people: µs below 10 ms, then ms, then seconds.
pub(super) fn time(us: u64) -> String {
    if us < 10_000 {
        format!("{us} us")
    } else if us < 1_000_000 {
        format!("{:.1} ms", us as f64 / 1e3)
    } else {
        format!("{:.2} s", us as f64 / 1e6)
    }
}

/// `text` cut to `max` characters, the cut marked.
pub(super) fn cut(text: &str, max: usize) -> String {
    if text.chars().count() <= max {
        return text.to_string();
    }
    let mut out: String = text.chars().take(max.saturating_sub(1)).collect();
    out.push('…');
    out
}

/// Split text into lines of at most `width` characters at spaces (a longer word is cut).
pub(super) fn wrap(text: &str, width: usize) -> Vec<String> {
    let width = width.max(8);
    let mut lines: Vec<String> = Vec::new();
    let mut line = String::new();
    for word in text.split_whitespace() {
        let mut word: Vec<char> = word.chars().collect();
        loop {
            let used = line.chars().count();
            let space = usize::from(used > 0);
            if used + space + word.len() <= width {
                if space == 1 {
                    line.push(' ');
                }
                line.extend(word.iter());
                break;
            }
            if used == 0 {
                line.extend(word.drain(..width));
            }
            lines.push(std::mem::take(&mut line));
        }
    }
    if !line.is_empty() || lines.is_empty() {
        lines.push(line);
    }
    lines
}

/// The lines being built, and the width they must fit.
pub(super) struct Out {
    pub lines: Vec<Line>,
    pub width: usize,
}

impl Out {
    /// A section heading (after a blank line) with a dim remark beside it.
    pub fn heading(&mut self, title: &str, remark: &str) {
        if !self.lines.is_empty() {
            self.lines.push(Vec::new());
        }
        let mut line = vec![span(format!(" {title}"), Tone::Heading)];
        if !remark.is_empty() {
            let room = self.width.saturating_sub(title.chars().count() + 4);
            line.push(span(format!("   {}", cut(remark, room)), Tone::Dim));
        }
        self.lines.push(line);
    }

    /// A labelled value; a long value wraps under itself.
    pub fn field(&mut self, label: &str, value: &str, tone: Tone) {
        let room = self.width.saturating_sub(LABEL + 2);
        for (i, part) in wrap(value, room).into_iter().enumerate() {
            let label = if i == 0 { label } else { "" };
            self.lines.push(vec![span(format!(" {label:<LABEL$} "), Tone::Label), span(part, tone)]);
        }
    }

    /// A wrapped remark.
    pub fn note(&mut self, text: &str, tone: Tone) {
        for part in wrap(text, self.width.saturating_sub(2)) {
            self.lines.push(vec![span(format!(" {part}"), tone)]);
        }
    }

    /// One preformatted row, cut to the width.
    pub fn row(&mut self, text: &str, tone: Tone) {
        self.lines.push(vec![span(cut(text, self.width), tone)]);
    }
}

/// A signal level in words.
fn level(snr_db: f32) -> (String, Tone) {
    let (word, tone) = match snr_db {
        s if s >= 20.0 => ("strong", Tone::Good),
        s if s >= 10.0 => ("fair", Tone::Plain),
        _ => ("weak", Tone::Warn),
    };
    (format!("{snr_db:.0} dB over the noise ({word})"), tone)
}

impl SignalReport {
    /// The whole report for a text area `width` columns wide ([`MIN_WIDTH`] at least).
    pub fn render(&self, width: usize) -> Vec<Line> {
        let mut out = Out { lines: Vec::new(), width: width.max(MIN_WIDTH) };
        self.summary(&mut out);
        self.modulation(&mut out);
        self.widths(&mut out);
        self.frames(&mut out);
        self.bit_rows(&mut out);
        self.waveform(&mut out);
        self.reference(&mut out);
        self.raw(&mut out);
        out.lines
    }

    /// Modulation, coding and TE in a few words: the summary's first line, and the overlay's title.
    pub fn headline(&self) -> String {
        if !self.is_regular() {
            return match self.carrier_block_us {
                Some(_) => "carrier without on/off data".to_string(),
                None => "no regular pulse train".to_string(),
            };
        }
        let mut parts = Vec::new();
        if self.keying != Keying::Unknown {
            parts.push(self.keying.label());
        }
        parts.push(self.timing.coding.label().to_string());
        if let Some(te) = self.te() {
            parts.push(format!("TE ~{:.0} us", te.te_us));
        }
        parts.join(", ")
    }

    /// The rows in a sentence: how many frames, how long, whether they are alike.
    pub fn frames_line(&self) -> Option<String> {
        let data: Vec<_> = self.data_rows().collect();
        if data.is_empty() {
            let count = |want: fn(&RowKind) -> bool| self.rows.iter().filter(|r| want(&r.kind)).count();
            let plain = count(|k| matches!(k, RowKind::Pulses));
            let one = count(|k| matches!(k, RowKind::Preamble { .. }));
            let long = count(|k| matches!(k, RowKind::Long { .. }));
            return match (plain, one, long) {
                (0, 0, 0) => None,
                (0, 0, n) => Some(format!("{n} long pulse{}, no data", plural(n))),
                (0, n, _) => Some(format!("{n} row{} of one pulse width, no data", plural(n))),
                (n, _, _) => Some(format!("{n} row{} of pulses KAT cannot read as bits", plural(n))),
            };
        }
        let mut lengths: BTreeMap<usize, usize> = BTreeMap::new();
        for (_, _, bits) in &data {
            *lengths.entry(bits.data().len()).or_default() += 1;
        }
        let (len, count) = lengths.iter().max_by_key(|(len, count)| (**count, **len)).map(|(l, c)| (*l, *c))?;
        let n = data.len();
        let (shortest, longest) = (lengths.keys().next().copied()?, lengths.keys().next_back().copied()?);
        let mut line = match (count == n, count) {
            (true, _) => format!("{n} frame{} of {len} bits", plural(n)),
            // No two rows are as long: what they have in common is nothing to count.
            (false, 1) if n == 2 => format!("2 rows of bits: {shortest} and {longest} bits"),
            (false, 1) => format!("{n} rows of bits, {shortest} to {longest} bits long"),
            (false, _) => format!("{n} rows of bits, {count} of them {len} bits long"),
        };
        let distinct = data.iter().filter(|(_, r, _)| matches!(r.kind, RowKind::Data { same_as: None, .. })).count();
        if n > 1 && count > 1 {
            line.push_str(&match distinct {
                1 => ", all identical".to_string(),
                d if d == n => ", all different".to_string(),
                d => format!(", {d} distinct"),
            });
        }
        if let Some(rep) = self.repeat.filter(|r| r.every_us > 0) {
            line.push_str(&format!(", every {}", time(rep.every_us)));
        }
        Some(line)
    }

    fn summary(&self, out: &mut Out) {
        out.heading("SUMMARY", "");
        let tone = if self.is_regular() { Tone::Accent } else { Tone::Warn };
        out.field("Looks like", &self.headline(), tone);
        if let Some(frames) = self.frames_line() {
            out.field("Frames", &frames, Tone::Plain);
        }
        let highs = self.pulses.iter().filter(|p| p.level).count();
        let total: u64 = self.pulses.iter().map(|p| u64::from(p.duration_us)).sum();
        let length = if self.rows.is_empty() { total } else { self.on_air_us };
        out.field("Length", &format!("{} with {highs} pulse{}", time(length), plural(highs)), Tone::Plain);
    }

    fn modulation(&self, out: &mut Out) {
        out.heading("MODULATION", "");
        let s = &self.source;
        let fm = s.path == Some(RfModulation::FM);
        let tuned = f64::from(s.frequency);
        match (s.measured, &s.preset) {
            (Some(m), _) => {
                let (keying, what) = match self.keying {
                    Keying::Fsk(Some(d)) => (
                        format!("2-FSK: two tones +/-{:.1} kHz around the centre ({:.1} kHz apart)", d / 1e3, d / 500.0),
                        "Centre",
                    ),
                    Keying::Fsk(None) => ("2-FSK (the tones were not measured)".to_string(), "Centre"),
                    _ => ("OOK / ASK: one carrier switched on and off (no second tone found)".to_string(), "Carrier"),
                };
                out.field("Keying", &keying, Tone::Accent);
                let at = (tuned + f64::from(m.offset_hz)) / 1e6;
                let off = f64::from(m.offset_hz) / 1e3;
                out.field(what, &format!("{at:.3} MHz ({off:+.1} kHz from the tuned {:.3} MHz)", tuned / 1e6), Tone::Plain);
                let (text, tone) = level(m.snr_db);
                out.field("Level", &text, tone);
                out.field("Burst", &format!("{} from the first carrier to the last", time(m.burst_us)), Tone::Plain);
            }
            (None, Some(preset)) if !s.live => {
                out.field("Keying", &format!("{} by the recording's preset ({preset})", self.keying.label()), Tone::Accent);
                out.note(
                    "A file of pulses holds no RF measurements: the keying is what the recorder was set to, \
                     which need not be what the transmitter does.",
                    Tone::Dim,
                );
            }
            _ => {
                let keying = match self.keying {
                    Keying::Unknown => "unknown: nothing says how these pulses were received".to_string(),
                    k => format!("{} by the receive path", k.label()),
                };
                out.field("Keying", &keying, Tone::Accent);
                out.note(
                    "No RF measurements for this capture: a Flipper and a file give pulses only, and sessions \
                     saved by an older KAT did not keep them.",
                    Tone::Dim,
                );
            }
        }
        match s.path {
            Some(RfModulation::AM) => out.field("Path", "AM (envelope): HIGH is carrier on, LOW is carrier off", Tone::Plain),
            Some(RfModulation::FM) => out.field("Path", "FM (frequency): HIGH is the upper tone, LOW the lower one", Tone::Plain),
            _ => {}
        }
        let other = s.sibling.map(|id| format!(" (capture #{id:02})")).unwrap_or_default();
        if s.measured.is_some() && !fm && matches!(self.keying, Keying::Fsk(_)) {
            out.note(
                &format!(
                    "These pulses are the envelope of an FSK burst: the carrier coming and going, not the data. \
                     The data is in the FM capture of the same burst{other}."
                ),
                Tone::Warn,
            );
        } else if s.measured.is_some_and(|m| m.both_paths) && fm && s.sibling.is_some() {
            out.note(&format!("The same burst also gave pulses on the AM path{other}."), Tone::Dim);
        }
        if let Some(block) = self.carrier_block_us {
            let why = if s.measured.is_some() {
                "KAT found no second tone in it, so it is plain carrier, or FSK narrower than KAT looks for."
            } else {
                "An FSK transmitter looks like this through an AM (OOK) receiver: record it again with an FSK \
                 preset (FM476, FM238), or with a HackRF or RTL-SDR, where KAT demodulates both ways."
            };
            out.note(
                &format!("Most of the carrier time is long blocks without on/off data (the longest {}). {why}", time(u64::from(block))),
                Tone::Warn,
            );
        } else if !self.is_regular() {
            out.note(
                "KAT finds no regular pulse train here: noise, a signal too weak or too short, or a coding it does not know.",
                Tone::Warn,
            );
        }
    }
}
