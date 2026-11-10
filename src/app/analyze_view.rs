//! The Analyze overlay for [`App`] (analysis: [`crate::analysis::signal`]).
//!
//! Opened from the signal menu ("Analyze") or with `:analyze [id]` (`:timing`, its name when it
//! compared pulse widths only, still works). The report is built once, a few milliseconds even for
//! a long capture, and laid out for the overlay's width when it is drawn.

use std::cell::{Ref, RefCell};

use crossterm::event::KeyCode;

use super::{App, InputMode};
use crate::analysis::signal::{self, KnownTiming, SignalReport, Source, Span, Tone};
use crate::capture::Capture;
use crate::protocols::{ProtocolRegistry, ProtocolTiming};
use crate::radio::preset::TxModulation;

/// The report is laid out no wider than this: its fields read badly as very long lines.
const MAX_REPORT_WIDTH: usize = 110;
/// Width the scroll limit is counted for until the first frame is drawn.
const NOMINAL_WIDTH: usize = 76;

/// An open Analyze overlay.
pub struct AnalyzeView {
    pub title: String,
    /// ID of the capture it is about (Left / Right move to the captures next to it).
    capture: u32,
    /// What the capture is (ID and protocol, frequency, time, radio or file), above the report.
    header: Vec<String>,
    report: SignalReport,
    /// The report laid out for one width: the renderer asks for it every frame.
    lines: RefCell<(usize, Vec<signal::Line>)>,
}

impl AnalyzeView {
    /// `header` says what the capture is, a few words each; it goes above the report.
    pub fn new(title: String, header: Vec<String>, report: SignalReport) -> Self {
        Self { title, capture: 0, header, report, lines: RefCell::new((0, Vec::new())) }
    }

    /// The overlay's lines for a text area `width` columns wide.
    pub fn lines(&self, width: usize) -> Ref<'_, [signal::Line]> {
        let stale = {
            let cached = self.lines.borrow();
            cached.0 != width || cached.1.is_empty()
        };
        if stale {
            let width = width.min(MAX_REPORT_WIDTH);
            let mut lines: Vec<signal::Line> =
                header_lines(&self.header, width).into_iter().map(|text| vec![Span { text, tone: Tone::Dim }]).collect();
            lines.push(Vec::new());
            lines.extend(self.report.render(width));
            *self.lines.borrow_mut() = (width, lines);
        }
        Ref::map(self.lines.borrow(), |cached| cached.1.as_slice())
    }
}

/// The header's parts on as few lines of `width` characters as they take, two spaces between the
/// parts (a part longer than a line is cut).
fn header_lines(parts: &[String], width: usize) -> Vec<String> {
    let room = width.saturating_sub(1).max(1);
    let mut lines: Vec<String> = Vec::new();
    for part in parts.iter().filter(|p| !p.is_empty()) {
        let mut part = part.clone();
        if part.chars().count() > room {
            part = part.chars().take(room - 1).chain(['…']).collect();
        }
        match lines.last_mut() {
            Some(line) if line.chars().count() + 2 + part.chars().count() <= width => {
                line.push_str("  ");
                line.push_str(&part);
            }
            _ => lines.push(format!(" {part}")),
        }
    }
    lines
}

/// The capture the overlay moves to from the one at `at`: the nearest with recorded pulses after it
/// (`forward`) or before it. None at the end of the list.
fn neighbour_with_pulses(captures: &[Capture], at: usize, forward: bool) -> Option<usize> {
    let has_pulses = |i: &usize| !captures[*i].raw_pairs.is_empty();
    if forward {
        (at + 1..captures.len()).find(has_pulses)
    } else {
        (0..at.min(captures.len())).rev().find(has_pulses)
    }
}

/// Where the capture's own frame ends in its pulses: decode them again and take the first frame of
/// the same protocol (and serial). Without one, the analysis takes the capture's end.
fn decoded_frame_end(protocols: &mut ProtocolRegistry, c: &Capture) -> Option<usize> {
    let proto = c.protocol.as_deref()?;
    let canonical = crate::saved_fobs::canonical_name(protocols, proto);
    let pairs: Vec<crate::radio::LevelDuration> =
        c.raw_pairs.iter().map(|p| crate::radio::LevelDuration::new(p.level, p.duration_us)).collect();
    let decodes = match c.received_rf {
        Some(path) => protocols.process_signal_stream_path(&pairs, c.frequency, Some(path)),
        None => {
            let rf = c.rf_preset.as_ref().and_then(|p| p.rf());
            protocols.process_signal_stream_rf(&pairs, c.frequency, rf)
        }
    };
    let mut end = 0;
    decodes.into_iter().find_map(|(name, d, seg)| {
        end += seg.len();
        let same = name == proto || crate::saved_fobs::canonical_name(protocols, &name) == canonical;
        (same && (c.serial.is_none() || d.serial == c.serial)).then_some(end)
    })
}

impl App {
    /// The open Analyze overlay.
    pub fn analyze_overlay(&self) -> Option<&AnalyzeView> {
        self.analyze_view.as_ref()
    }

    /// `a`: analyse the selected capture.
    pub fn analyze_selected(&mut self) {
        self.last_error = None;
        self.analyze_command(None);
    }

    /// `:analyze [ID]`: analyse the given capture, or the selected one.
    pub(super) fn analyze_command(&mut self, arg: Option<&str>) {
        let id = match arg {
            Some(s) => match s.trim().trim_start_matches('#').parse::<u32>() {
                Ok(id) => id,
                Err(_) => {
                    self.last_error = Some("Usage: :analyze [ID]".to_string());
                    return;
                }
            },
            None => match self.selected_capture.and_then(|i| self.captures.get(i)) {
                Some(c) => c.id,
                None => {
                    self.last_error = Some("No capture selected (usage: :analyze [ID])".to_string());
                    return;
                }
            },
        };
        self.open_analysis(id);
    }

    /// Timing definitions of a protocol (`ProtocolDecoder::timing()` first, then the other timings
    /// it is sent with), resolving names like the saved-fob matcher does (legacy names, display
    /// suffixes, the generic KeeLoq spelling). Empty for a name no decoder has.
    fn protocol_timings(&self, name: &str) -> Vec<ProtocolTiming> {
        self.protocols
            .get(name)
            .or_else(|| self.protocols.get(&crate::saved_fobs::canonical_name(&self.protocols, name)))
            .map(|d| std::iter::once(d.timing()).chain(d.timing_variants()).collect())
            .unwrap_or_default()
    }

    /// Every registered protocol's timing, for the report's list of protocols with similar widths.
    fn known_timings(&self) -> Vec<KnownTiming> {
        let mut known = Vec::new();
        for name in self.protocols.list_protocols() {
            let info = crate::protocols::info::lookup(name);
            for t in self.protocol_timings(name) {
                known.push(KnownTiming {
                    name,
                    te_short: t.te_short,
                    te_long: t.te_long,
                    te_delta: t.te_delta,
                    coding: info.map(|i| i.encoding.to_string()).unwrap_or_default(),
                    rf: info.map(|i| i.rf.to_string()).unwrap_or_default(),
                });
            }
        }
        known
    }

    /// Where the capture's pulses came from and what is known of the RF side.
    fn analysis_source(&self, c: &Capture) -> Source {
        let preset = c.rf_preset.as_ref();
        // The same burst on the other demodulator path, when it is still in the list.
        let sibling = c.on_air.and_then(|on| {
            self.captures
                .iter()
                .find(|o| {
                    o.id != c.id
                        && o.received_rf != c.received_rf
                        && o.on_air.is_some_and(|other| other.session == on.session && other.start_us == on.start_us)
                })
                .map(|o| o.id)
        });
        Source {
            frequency: c.frequency,
            path: c.received_rf.or_else(|| preset.and_then(|p| p.rf())),
            live: c.radio.is_some() || (c.source_file.is_none() && c.received_rf.is_some()),
            measured: c.measured,
            preset: preset.map(|p| p.label()),
            preset_deviation_hz: preset.and_then(|p| match p.modulation() {
                Some(TxModulation::Fsk { deviation_hz }) => Some(deviation_hz),
                _ => None,
            }),
            sibling,
        }
    }

    /// Analyse capture `id` and open the overlay. The list selects that capture: the detail panel
    /// behind the overlay shows it, and closing the overlay leaves the list on the capture last
    /// looked at (after `:analyze ID` and after Left / Right alike).
    pub(super) fn open_analysis(&mut self, id: u32) {
        let Some((at, c)) = self.captures.iter().enumerate().find(|(_, c)| c.id == id) else {
            self.last_error = Some(format!("Capture #{:02} not found", id));
            return;
        };
        if c.raw_pairs.is_empty() {
            self.last_error = Some(format!("Capture #{:02} has no recorded pulses to analyze", id));
            return;
        }
        let references = c.protocol.as_deref().map(|p| self.protocol_timings(p)).unwrap_or_default();
        let (source, known) = (self.analysis_source(c), self.known_timings());
        let frame_end = decoded_frame_end(&mut self.protocols, c);
        let report = signal::analyze(&c.raw_pairs, source, c.protocol.as_deref(), &references, frame_end, &known);
        let from = match (&c.radio, &c.source_file) {
            (Some(radio), _) => radio.clone(),
            (None, Some(file)) => {
                std::path::Path::new(file).file_name().map_or(file.clone(), |f| f.to_string_lossy().into_owned())
            }
            (None, None) => String::new(),
        };
        let mut view = AnalyzeView::new(
            format!("Analyze: #{:02} {} ({})", c.id, c.protocol_name(), report.headline()),
            vec![
                format!("Signal #{:02}  {}", c.id, c.protocol_name()),
                format!("{} MHz", crate::radio::hopper::format_mhz(c.frequency)),
                c.timestamp_full(),
                from,
            ],
            report,
        );
        view.capture = id;
        self.selected_capture = Some(at);
        self.overlay_scroll_max.set(view.lines(NOMINAL_WIDTH).len());
        self.analyze_view = Some(view);
        self.overlay_scroll = 0;
        self.input_mode = InputMode::Analyze;
    }

    pub(super) fn close_analyze_view(&mut self) {
        self.analyze_view = None;
        if self.input_mode == InputMode::Analyze {
            self.input_mode = InputMode::Normal;
            self.overlay_scroll = 0;
        }
    }

    /// Left / Right in the overlay: analyse the capture before / after this one in the list (the
    /// nearest that has pulses). `open_analysis` selects it in the list.
    fn analyze_neighbour(&mut self, forward: bool) {
        let Some(id) = self.analyze_view.as_ref().map(|v| v.capture) else { return };
        let Some(at) = self.captures.iter().position(|c| c.id == id) else { return };
        if let Some(i) = neighbour_with_pulses(&self.captures, at, forward) {
            self.open_analysis(self.captures[i].id);
        }
    }

    /// Keys of the overlay: scroll, close, and Left / Right to the captures next to this one.
    pub(super) fn analyze_view_key(&mut self, code: KeyCode) {
        if matches!(code, KeyCode::Esc | KeyCode::Enter | KeyCode::Char('q')) {
            self.close_analyze_view();
        } else if matches!(code, KeyCode::Left | KeyCode::Right) {
            self.analyze_neighbour(code == KeyCode::Right);
        } else if let Some(scroll) =
            super::input::overlay_scrolled(self.overlay_scroll, self.overlay_scroll_max.get(), code)
        {
            self.overlay_scroll = scroll;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture::StoredLevelDuration;

    fn capture(id: u32, pulses: usize) -> Capture {
        let pairs = (0..pulses).map(|i| StoredLevelDuration { level: i % 2 == 0, duration_us: 400 }).collect();
        Capture::from_pairs(id, 433_920_000, pairs)
    }

    /// Left / Right go to the nearest capture that can be analysed and stop at the ends of the list.
    #[test]
    fn left_and_right_skip_captures_without_pulses() {
        let list = [capture(1, 8), capture(2, 0), capture(3, 0), capture(4, 8), capture(5, 0)];
        assert_eq!(neighbour_with_pulses(&list, 0, true), Some(3));
        assert_eq!(neighbour_with_pulses(&list, 3, false), Some(0));
        assert_eq!(neighbour_with_pulses(&list, 3, true), None, "only a capture without pulses follows");
        assert_eq!(neighbour_with_pulses(&list, 0, false), None, "the first capture");
        assert_eq!(neighbour_with_pulses(&list, 4, false), Some(3));
        assert_eq!(neighbour_with_pulses(&[], 0, true), None);
        assert_eq!(neighbour_with_pulses(&[], 0, false), None);
    }

    /// The header's parts share lines while they fit, and a long file name is cut, not wrapped.
    #[test]
    fn the_header_takes_as_few_lines_as_it_needs() {
        let parts: Vec<String> =
            ["Signal #04  Unknown", "433.889 MHz", "2026-01-02 03:04:05 UTC+01:00", "", "a_recording.sub"]
                .map(String::from)
                .to_vec();
        assert_eq!(
            header_lines(&parts, 100),
            [" Signal #04  Unknown  433.889 MHz  2026-01-02 03:04:05 UTC+01:00  a_recording.sub"]
        );
        let narrow = header_lines(&parts, 54);
        assert_eq!(
            narrow,
            [" Signal #04  Unknown  433.889 MHz", " 2026-01-02 03:04:05 UTC+01:00  a_recording.sub"]
        );
        let long = vec!["x".repeat(80)];
        let cut = header_lines(&long, 54);
        assert_eq!(cut.len(), 1);
        assert!(cut[0].ends_with('…') && cut[0].chars().count() <= 54, "{cut:?}");
    }
}
