//! The Compare overlay for [`App`] (comparison: [`crate::analysis::compare`]).
//!
//! Opened with `c` on a capture (the captures of its remote that the list shows, in the order they
//! came) or with `:compare IDs` (those captures, in the order given; a `/` between IDs puts them
//! in groups by hand). Decoded captures of one protocol compare their decoded data; unknown ones,
//! and decoded ones after Tab, the bits the trial decode of Analyze reads from their pulses.

use std::cell::{Ref, RefCell};

use crossterm::event::KeyCode;

use super::analyze_view::{header_lines, report_of};
use super::{App, InputMode};
use crate::analysis::compare::{self, Comparison, Frame, Grouping};
use crate::analysis::signal::{self, Span, Tone};
use crate::capture::Capture;
use crate::protocols::ProtocolRegistry;

/// The report is laid out no wider than this, like the Analyze report.
const MAX_REPORT_WIDTH: usize = 110;
/// Width the scroll limit is counted for until the first frame is drawn.
const NOMINAL_WIDTH: usize = 76;
/// The most captures compared at once, and the most frames (an unknown capture can hold several).
const MAX_CAPTURES: usize = 64;
/// Rows this many bits apart in length are one kind of frame, read from a few bits earlier or
/// later (a preamble caught at another point): they are lined up before they are compared.
const MAX_SHIFT: usize = 8;

/// A capture picked for the comparison, and the group the user put it in.
struct Pick<'a> {
    capture: &'a Capture,
    group: Option<String>,
}

/// One comparison of the captures picked: of their decoded data, or of the bits as sent.
struct Side {
    /// What it compares, for the title.
    label: &'static str,
    comparison: Comparison,
}

/// An open Compare overlay.
pub struct CompareView {
    /// What is compared ("10 captures of Kia V1").
    subject: String,
    /// The captures and what they are, above the report.
    header: Vec<String>,
    /// The decoded data's comparison when the captures have one, then that of the bits as sent.
    sides: Vec<Side>,
    /// The side on screen (Tab shows the other).
    shown: usize,
    /// The report laid out for one side and width: the renderer asks for it every frame.
    lines: RefCell<(usize, usize, Vec<signal::Line>)>,
}

impl CompareView {
    /// A view of `sides` (what each compares, for the title, and the comparison; at least one),
    /// showing the first.
    pub fn new(subject: String, header: Vec<String>, sides: Vec<(&'static str, Comparison)>) -> Self {
        let sides = sides.into_iter().map(|(label, comparison)| Side { label, comparison }).collect();
        Self { subject, header, sides, shown: 0, lines: RefCell::new((0, 0, Vec::new())) }
    }

    /// The side shown comes first: a narrow terminal cuts the title's end.
    pub fn title(&self) -> String {
        format!("Compare ({}): {}", self.sides[self.shown].label, self.subject)
    }

    /// The overlay's lines for a text area `width` columns wide.
    pub fn lines(&self, width: usize) -> Ref<'_, [signal::Line]> {
        let width = width.min(MAX_REPORT_WIDTH);
        let stale = {
            let cached = self.lines.borrow();
            (cached.0, cached.1) != (self.shown, width) || cached.2.is_empty()
        };
        if stale {
            let mut header = self.header.clone();
            if self.sides.len() > 1 {
                header.push(format!("Tab: the {}", self.sides[1 - self.shown].label));
            }
            let mut lines: Vec<signal::Line> =
                header_lines(&header, width).into_iter().map(|text| vec![Span { text, tone: Tone::Dim }]).collect();
            lines.push(Vec::new());
            lines.extend(self.sides[self.shown].comparison.render(width));
            *self.lines.borrow_mut() = (self.shown, width, lines);
        }
        Ref::map(self.lines.borrow(), |cached| cached.2.as_slice())
    }

    /// Tab: show the other side, when there are two.
    pub fn show_other_side(&mut self) -> bool {
        if self.sides.len() != 2 {
            return false;
        }
        self.shown = 1 - self.shown;
        true
    }
}

/// The name of a capture's button, when it has one: the label the user gave it (`i`), else the
/// decoded button's name (its code where KAT has no name for it).
fn button_of(c: &Capture) -> Option<String> {
    if let Some(label) = c.command.as_deref().map(str::trim).filter(|l| !l.is_empty()) {
        return Some(label.to_string());
    }
    let code = c.button?;
    Some(match c.button_name() {
        "Other" | "-" => format!("0x{code:X}"),
        name => name.to_string(),
    })
}

/// A decoded frame's bits, the first one sent first: the low `data_count_bit` bits of its data, or
/// the 64 bits a longer frame's data holds (its start).
fn decoded_bits(c: &Capture) -> Vec<bool> {
    (0..c.data_count_bit.min(64)).rev().map(|i| (c.data >> i) & 1 == 1).collect()
}

/// "#04, #05" for the captures of `picks` that `left_out` takes.
fn named(picks: &[Pick], left_out: impl Fn(&Capture) -> bool) -> Option<String> {
    let ids: Vec<u32> = picks.iter().filter(|p| left_out(p.capture)).map(|p| p.capture.id).collect();
    (!ids.is_empty()).then(|| super::input::ids_text(&ids))
}

/// "is" / "are" after a list of captures.
fn are(ids: &str) -> &'static str {
    if ids.contains(',') || ids.contains('-') {
        "are"
    } else {
        "is"
    }
}

/// The comparison of the picks' decoded data: of the protocol and frame length most of them have
/// (the earliest such on a tie). None when no pick is decoded.
fn decoded_side(picks: &[Pick], grouping: Grouping, notes: &[String]) -> Option<Side> {
    fn kind(c: &Capture) -> Option<(&str, usize)> {
        c.protocol.as_deref().filter(|_| c.data_count_bit > 0).map(|p| (p, c.data_count_bit))
    }
    let kinds: Vec<(&str, usize)> = picks.iter().filter_map(|p| kind(p.capture)).collect();
    let (protocol, bits) = *kinds.iter().max_by_key(|k| {
        let first = kinds.iter().position(|other| other == *k).unwrap_or(0);
        (kinds.iter().filter(|other| other == k).count(), std::cmp::Reverse(first))
    })?;
    let taken = |c: &Capture| kind(c) == Some((protocol, bits));
    let frames = picks
        .iter()
        .filter(|p| taken(p.capture))
        .map(|p| Frame { name: format!("#{:02}", p.capture.id), group: p.group.clone(), bits: decoded_bits(p.capture) })
        .collect();
    let mut notes = notes.to_vec();
    if bits > 64 {
        notes.push(format!("A frame is {bits} bits long: these are the first 64, which KAT keeps as its data."));
    }
    if let Some(ids) = named(picks, |c| !taken(c)) {
        notes.push(format!("{ids} {} left out: not {protocol} frames of {bits} bits.", are(&ids)));
    }
    let what = format!("the decoded data of {protocol}");
    Some(Side { label: "decoded data", comparison: compare::compare(frames, what, grouping, notes) })
}

/// The rows of bits the trial decode reads from a capture's pulses, in order.
fn rows_of(protocols: &mut ProtocolRegistry, c: &Capture) -> Vec<Vec<bool>> {
    if c.raw_pairs.is_empty() {
        return Vec::new();
    }
    let report = report_of(protocols, c);
    report.data_rows().map(|(_, _, bits)| bits.data().to_vec()).filter(|row| !row.is_empty()).collect()
}

/// A capture's rows of bits for the comparison of the bits as sent.
struct Rows {
    id: u32,
    group: Option<String>,
    /// A decoded capture: its pulses start with what was left of the press before (the repeats
    /// after that one was decoded) and end with the frame it was decoded from, its last row.
    decoded: bool,
    rows: Vec<Vec<bool>>,
}

/// The frames of the bits as sent, and what was left out. A decoded capture gives its own frame,
/// an unknown one every row it holds. Rows of one kind are compared: those of the lengths, within
/// [`MAX_SHIFT`] bits, that most captures have a row of (the longer on a tie), lined up by what
/// they hold. A capture with several different frames has them named "#07/1", "#07/2".
fn on_air_frames(captures: &[Rows]) -> (Vec<Frame>, Vec<String>) {
    let offered: Vec<Vec<&Vec<bool>>> = captures
        .iter()
        .map(|c| if c.decoded { c.rows.last().into_iter().collect() } else { c.rows.iter().collect() })
        .collect();
    let mut lengths: Vec<usize> = offered.iter().flatten().map(|r| r.len()).collect();
    lengths.sort_unstable();
    lengths.dedup();
    let kind = |low: usize, row: &[bool]| (low..=low + MAX_SHIFT).contains(&row.len());
    let captures_with = |low: usize| offered.iter().filter(|rows| rows.iter().any(|r| kind(low, r))).count();
    let rows_with = |low: usize| offered.iter().flatten().filter(|r| kind(low, r)).count();
    let Some(low) = lengths.iter().copied().max_by_key(|low| (captures_with(*low), rows_with(*low), *low)) else {
        return (Vec::new(), Vec::new());
    };

    // The rows of that kind and the capture each is of, lined up.
    let taken: Vec<(usize, &[bool])> = offered
        .iter()
        .enumerate()
        .flat_map(|(at, rows)| rows.iter().filter(|r| kind(low, r)).map(move |r| (at, r.as_slice())))
        .collect();
    let rows: Vec<&[bool]> = taken.iter().map(|(_, row)| *row).collect();
    let (shortest, longest) = (rows.iter().map(|r| r.len()).min().unwrap_or(0), rows.iter().map(|r| r.len()).max().unwrap_or(0));
    let lined_up = compare::align(&rows, MAX_SHIFT);

    let mut frames = Vec::new();
    let mut without = Vec::new();
    for (at, c) in captures.iter().enumerate() {
        // Each different frame once: a capture holds the repeats of a press.
        let mut own: Vec<&Vec<bool>> = Vec::new();
        for (_, row) in taken.iter().zip(&lined_up).filter(|((of, _), _)| *of == at) {
            if !own.contains(&row) {
                own.push(row);
            }
        }
        if own.is_empty() {
            without.push(c.id);
        }
        let several = own.len() > 1;
        for (i, row) in own.into_iter().enumerate() {
            let name = if several { format!("#{:02}/{}", c.id, i + 1) } else { format!("#{:02}", c.id) };
            frames.push(Frame { name, group: c.group.clone(), bits: row.clone() });
        }
    }

    let mut notes = Vec::new();
    if shortest != longest {
        notes.push(format!(
            "The rows are {shortest} to {longest} bits long: they are lined up by what they hold, and the bits all of them have are compared."
        ));
    }
    if !without.is_empty() {
        let ids = super::input::ids_text(&without);
        notes.push(format!("{ids} {} left out: no row of that length (a: how a capture is read).", are(&ids)));
    }
    if offered.iter().flatten().any(|r| !kind(low, r)) {
        notes.push("Rows of other lengths are left out.".to_string());
    }
    if frames.len() > MAX_CAPTURES {
        notes.push(format!("The first {MAX_CAPTURES} of {} frames are compared.", frames.len()));
        frames.truncate(MAX_CAPTURES);
    }
    (frames, notes)
}

/// The comparison of the bits as sent. None when no pick has a row of bits.
fn on_air_side(protocols: &mut ProtocolRegistry, picks: &[Pick], grouping: Grouping, notes: &[String]) -> Option<Side> {
    let rows: Vec<Rows> = picks
        .iter()
        .map(|p| Rows {
            id: p.capture.id,
            group: p.group.clone(),
            decoded: p.capture.protocol.is_some(),
            rows: rows_of(protocols, p.capture),
        })
        .collect();
    let (frames, left_out) = on_air_frames(&rows);
    if frames.is_empty() {
        return None;
    }
    let notes = notes.iter().cloned().chain(left_out).collect();
    let what = "the bits as sent, as Analyze reads them".to_string();
    Some(Side { label: "bits as sent", comparison: compare::compare(frames, what, grouping, notes) })
}

/// The comparisons of `picks`, the one to show first: the decoded data when the picks have it, then
/// the bits as sent.
fn sides_of(protocols: &mut ProtocolRegistry, picks: &[Pick], grouping: Grouping, notes: &[String]) -> Vec<Side> {
    let mut sides: Vec<Side> = Vec::new();
    sides.extend(decoded_side(picks, grouping, notes));
    sides.extend(on_air_side(protocols, picks, grouping, notes));
    // A side with one frame has nothing to compare: it goes when the other has more.
    if sides.iter().any(|s| s.comparison.frames.len() >= 2) {
        sides.retain(|s| s.comparison.frames.len() >= 2);
    }
    sides
}

/// The captures of the remote of `captures[at]` among those at `shown`, in the order they came: a
/// decoded capture's are those of its protocol and serial, an unknown one's the unknown captures
/// on its frequency and demodulator path.
fn same_remote(captures: &[Capture], shown: &[usize], at: usize) -> Vec<usize> {
    let Some(of) = captures.get(at) else { return Vec::new() };
    let same = |c: &Capture| match &of.protocol {
        Some(_) => c.protocol == of.protocol && c.serial == of.serial,
        None => c.protocol.is_none() && c.frequency == of.frequency && c.received_rf == of.received_rf,
    };
    let mut found: Vec<usize> = shown.iter().copied().filter(|i| captures.get(*i).is_some_and(same)).collect();
    found.sort_unstable();
    found
}

/// The name of group number `n` of a `:compare` with groups: "A", "B", …
fn group_name(n: usize) -> String {
    match u8::try_from(n).ok().filter(|n| *n < 26) {
        Some(n) => char::from(b'A' + n).to_string(),
        None => format!("G{}", n + 1),
    }
}

/// The captures a `:compare` names, in its order, each once: their IDs and, when the spec has
/// groups (`1-5 / 6-10`), each one's group. With them, the IDs named that no capture has.
#[allow(clippy::type_complexity)]
fn parse_spec(spec: &str, existing: &[u32]) -> Result<(Vec<(u32, Option<String>)>, Option<String>), String> {
    let parts: Vec<&str> = spec.split('/').collect();
    let grouped = parts.len() > 1;
    let mut picked: Vec<(u32, Option<String>)> = Vec::new();
    let mut missing = Vec::new();
    for (n, part) in parts.into_iter().enumerate() {
        let (ids, not_found) = super::input::ids_in_order(part, existing)?;
        missing.extend(not_found);
        for id in ids {
            if !picked.iter().any(|(other, _)| *other == id) {
                picked.push((id, grouped.then(|| group_name(n))));
            }
        }
    }
    Ok((picked, (!missing.is_empty()).then(|| missing.join(", "))))
}

impl App {
    /// The open Compare overlay.
    pub fn compare_overlay(&self) -> Option<&CompareView> {
        self.compare_view.as_ref()
    }

    /// `c`: compare the captures of the selected capture's remote.
    pub fn compare_selected(&mut self) {
        self.last_error = None;
        self.compare_command(None);
    }

    /// `:compare [IDs]`: compare the captures named, in that order, or those of the selected
    /// capture's remote that the list shows.
    pub(super) fn compare_command(&mut self, spec: Option<&str>) {
        let existing: Vec<u32> = self.captures.iter().map(|c| c.id).collect();
        let (picked, grouping, mut notes) = match spec {
            Some(spec) => match parse_spec(spec, &existing) {
                Ok((picked, missing)) => {
                    let grouping = if spec.contains('/') { Grouping::Group } else { Grouping::Button };
                    (picked, grouping, missing.map(|ids| format!("{ids} not found.")).into_iter().collect::<Vec<_>>())
                }
                Err(e) => {
                    self.last_error = Some(e);
                    return;
                }
            },
            None => {
                let Some(at) = self.selected_capture.filter(|i| *i < self.captures.len()) else {
                    self.last_error = Some("No capture selected (usage: :compare [IDs])".to_string());
                    return;
                };
                let remote = same_remote(&self.captures, &self.visible_captures(), at);
                (remote.into_iter().map(|i| (self.captures[i].id, None)).collect(), Grouping::Button, Vec::new())
            }
        };
        let mut picks: Vec<Pick> = picked
            .into_iter()
            .filter_map(|(id, group)| self.captures.iter().find(|c| c.id == id).map(|c| (c, group)))
            .map(|(capture, group)| Pick { group: group.or_else(|| button_of(capture)), capture })
            .collect();
        if picks.len() > MAX_CAPTURES {
            notes.push(format!("The first {MAX_CAPTURES} of {} captures are compared.", picks.len()));
            picks.truncate(MAX_CAPTURES);
        }

        let ids: Vec<u32> = picks.iter().map(|p| p.capture.id).collect();
        let ids_text = super::input::ids_text(&ids);
        let sides = sides_of(&mut self.protocols, &picks, grouping, &notes);
        let frames = sides.first().map_or(0, |s| s.comparison.frames.len());
        if frames < 2 {
            let noun = if picks.len() == 1 { "Capture" } else { "Captures" };
            self.last_error = Some(match (frames, picks.len(), spec) {
                (0, 1, _) => format!("{noun} {ids_text} has no bits to compare (a: analyze it)"),
                (0, _, _) => format!("{noun} {ids_text} have no bits to compare (a: analyze one)"),
                (_, 1, None) => {
                    format!("{noun} {ids_text} is the only one of its remote here (:compare IDs compares any)")
                }
                (_, 1, Some(_)) => format!("{noun} {ids_text} is one frame: name more captures (:compare 1-5)"),
                _ => format!("{noun} {ids_text} give one frame of bits: nothing to compare (a: analyze one)"),
            });
            return;
        }

        // What the captures are, when they are of one kind.
        let first = picks[0].capture;
        let one_protocol = picks.iter().all(|p| p.capture.protocol == first.protocol);
        let count = if picks.len() == 1 { "1 capture".to_string() } else { format!("{} captures", picks.len()) };
        let subject = match &first.protocol {
            Some(protocol) if one_protocol => format!("{count} of {protocol}"),
            None if one_protocol => format!("{count}, protocol unknown"),
            _ => count,
        };
        let mut header = Vec::new();
        if one_protocol {
            let serial = picks.iter().all(|p| p.capture.serial == first.serial).then_some(first.serial).flatten();
            header.push(match (&first.protocol, serial) {
                (Some(protocol), Some(_)) => format!("{protocol}  serial {}", first.serial_hex()),
                (Some(protocol), None) => protocol.clone(),
                (None, _) => "Unknown protocol".to_string(),
            });
        }
        if picks.iter().all(|p| p.capture.frequency == first.frequency) {
            header.push(format!("{} MHz", crate::radio::hopper::format_mhz(first.frequency)));
        }
        header.push(format!("{} {ids_text}", if picks.len() == 1 { "Capture" } else { "Captures" }));

        let view = CompareView::new(subject, header, sides.into_iter().map(|s| (s.label, s.comparison)).collect());
        self.overlay_scroll_max.set(view.lines(NOMINAL_WIDTH).len());
        self.compare_view = Some(view);
        self.overlay_scroll = 0;
        self.input_mode = InputMode::Compare;
    }

    pub(super) fn close_compare_view(&mut self) {
        self.compare_view = None;
        if self.input_mode == InputMode::Compare {
            self.input_mode = InputMode::Normal;
            self.overlay_scroll = 0;
        }
    }

    /// Keys of the overlay: scroll, close, and Tab between the decoded data and the bits as sent.
    pub(super) fn compare_view_key(&mut self, code: KeyCode) {
        if matches!(code, KeyCode::Esc | KeyCode::Enter | KeyCode::Char('q')) {
            self.close_compare_view();
        } else if matches!(code, KeyCode::Tab | KeyCode::BackTab) {
            if self.compare_view.as_mut().is_some_and(|view| view.show_other_side()) {
                self.overlay_scroll = 0;
            }
        } else if let Some(scroll) =
            super::input::overlay_scrolled(self.overlay_scroll, self.overlay_scroll_max.get(), code)
        {
            self.overlay_scroll = scroll;
        }
    }
}

#[cfg(test)]
mod tests;
