//! Timing analysis overlay for [`App`] (analysis: [`crate::analysis::timing`]).
//!
//! Opened from the signal menu ("Timing analysis") or with `:timing [id]`; the report is built once
//! (a few milliseconds even for long captures) and shown in a scrollable text overlay.

use std::cell::Cell;

use crossterm::event::KeyCode;

use super::{App, InputMode};
use crate::analysis::timing;
use crate::protocols::ProtocolTiming;

/// Lines moved by PageUp / PageDown.
const PAGE: usize = 10;

/// An open timing-analysis overlay.
pub struct TimingView {
    pub title: String,
    pub text: String,
    /// Largest useful scroll offset as the last frame drew it (wrapped lines − visible rows); the
    /// line count until the first frame. Keeps Up working right after scrolling past the end.
    scroll_max: Cell<usize>,
}

impl TimingView {
    /// Called by the renderer with the scroll limit of the frame it drew.
    pub fn set_scroll_max(&self, max: usize) {
        self.scroll_max.set(max);
    }
}

impl App {
    /// The open timing overlay.
    pub fn timing_overlay(&self) -> Option<&TimingView> {
        self.timing_view.as_ref()
    }

    /// `:timing [ID]`: analyse the given capture, or the selected one.
    pub(super) fn timing_command(&mut self, arg: Option<&str>) {
        let id = match arg {
            Some(s) => match s.trim().trim_start_matches('#').parse::<u32>() {
                Ok(id) => id,
                Err(_) => {
                    self.last_error = Some("Usage: :timing [ID]".to_string());
                    return;
                }
            },
            None => match self.selected_capture.and_then(|i| self.captures.get(i)) {
                Some(c) => c.id,
                None => {
                    self.last_error = Some("No capture selected (usage: :timing [ID])".to_string());
                    return;
                }
            },
        };
        self.open_timing_analysis(id);
    }

    /// Timing definition of a protocol (`ProtocolDecoder::timing()`), resolving names like the
    /// saved-fob matcher does (legacy names, display suffixes, the generic KeeLoq spelling).
    fn protocol_timing(&self, name: &str) -> Option<ProtocolTiming> {
        self.protocols
            .get(name)
            .or_else(|| self.protocols.get(&crate::saved_fobs::canonical_name(&self.protocols, name)))
            .map(|d| d.timing())
    }

    /// Analyse capture `id` and open the overlay.
    pub(super) fn open_timing_analysis(&mut self, id: u32) {
        let Some(c) = self.captures.iter().find(|c| c.id == id) else {
            self.last_error = Some(format!("Capture {} not found", id));
            return;
        };
        if c.raw_pairs.is_empty() {
            self.last_error = Some(format!("Capture {} has no raw pairs to analyse", id));
            return;
        }
        let reference = c.protocol.as_deref().and_then(|p| self.protocol_timing(p));
        // Where the capture's own frame ends: decode the pairs again and take the first frame of
        // the same protocol (and serial). Without one, the analysis assumes it ends the capture.
        let frame_end = c.protocol.as_deref().and_then(|proto| {
            let canonical = crate::saved_fobs::canonical_name(&self.protocols, proto);
            let pairs: Vec<crate::radio::LevelDuration> = c
                .raw_pairs
                .iter()
                .map(|p| crate::radio::LevelDuration::new(p.level, p.duration_us))
                .collect();
            let mut end = 0;
            let decodes = match c.received_rf {
                Some(path) => self.protocols.process_signal_stream_path(&pairs, c.frequency, Some(path)),
                None => {
                    let rf = c.rf_preset.as_ref().and_then(|p| p.rf());
                    self.protocols.process_signal_stream_rf(&pairs, c.frequency, rf)
                }
            };
            decodes.into_iter().find_map(|(name, d, seg)| {
                end += seg.len();
                let same = name == proto || crate::saved_fobs::canonical_name(&self.protocols, &name) == canonical;
                (same && (c.serial.is_none() || d.serial == c.serial)).then_some(end)
            })
        });
        let report = match frame_end {
            Some(end) => timing::analyze_frame_ending_at(&c.raw_pairs, end, c.protocol.as_deref(), reference.as_ref()),
            None => timing::analyze(&c.raw_pairs, c.protocol.as_deref(), reference.as_ref()),
        };
        let mut lines = vec![
            format!(
                "Signal #{:02}  {}  {}  {}",
                c.id,
                c.protocol_name(),
                c.frequency_mhz(),
                c.timestamp_full()
            ),
            String::new(),
        ];
        lines.extend(report.render_lines());
        lines.push(String::new());
        lines.push("Up/Down, PgUp/PgDn, Home/End: scroll | Esc/Enter: close".to_string());
        let text = lines.join("\n");
        let te = report.te.map(|t| format!(", TE ~{:.0} us", t.te_us)).unwrap_or_default();
        self.timing_view = Some(TimingView {
            title: format!(
                "Timing analysis: #{:02} {} ({}{})",
                c.id,
                c.protocol_name(),
                report.coding.label(),
                te
            ),
            scroll_max: Cell::new(text.lines().count()),
            text,
        });
        self.overlay_scroll = 0;
        self.input_mode = InputMode::TimingAnalysis;
    }

    pub(super) fn close_timing_view(&mut self) {
        self.timing_view = None;
        if self.input_mode == InputMode::TimingAnalysis {
            self.input_mode = InputMode::Normal;
        }
    }

    /// Scroll / close keys of the overlay.
    pub(super) fn timing_view_key(&mut self, code: KeyCode) {
        let max = self.timing_view.as_ref().map_or(0, |v| v.scroll_max.get());
        let scroll = self.overlay_scroll.min(max);
        self.overlay_scroll = match code {
            KeyCode::Esc | KeyCode::Enter | KeyCode::Char('q') => {
                self.close_timing_view();
                0
            }
            KeyCode::Up | KeyCode::Char('k') => scroll.saturating_sub(1),
            KeyCode::Down | KeyCode::Char('j') => (scroll + 1).min(max),
            KeyCode::PageUp => scroll.saturating_sub(PAGE),
            KeyCode::PageDown => (scroll + PAGE).min(max),
            KeyCode::Home | KeyCode::Char('g') => 0,
            KeyCode::End | KeyCode::Char('G') => max,
            _ => scroll,
        };
    }
}
