//! The Analyze overlay: the signal report ([`crate::analysis::signal`]) in the app's colours.
//!
//! The report comes as lines of tone-tagged text already laid out for the overlay's width, so
//! nothing is wrapped here and the scroll limit is exact.

use ratatui::{
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Clear, Paragraph},
    Frame,
};

use crate::analysis::signal::{self, Tone};
use crate::app::{AnalyzeView, App};

use super::text_overlay::{overlay_block, overlay_rect};
use super::{text, theme};

/// The style of each kind of text in the report.
fn style(tone: Tone) -> Style {
    match tone {
        Tone::Plain => theme::TEXT,
        Tone::Heading => Style::new().fg(Color::Cyan).add_modifier(Modifier::BOLD),
        Tone::Label | Tone::Dim => Style::new().fg(Color::DarkGray),
        Tone::Good => Style::new().fg(Color::Green),
        Tone::Warn => Style::new().fg(Color::Yellow),
        Tone::High => Style::new().fg(Color::Green),
        Tone::Low => Style::new().fg(Color::DarkGray),
        Tone::Accent => Style::new().fg(Color::Cyan),
    }
}

/// Draw the open Analyze overlay and tell the app where its scroll ends.
pub fn render_analyze_overlay(frame: &mut Frame, app: &App) {
    if let Some(view) = app.analyze_overlay() {
        app.overlay_scroll_max.set(render_view(frame, view, app.overlay_scroll));
    }
}

/// Draw `view` from its line `scroll` on; returns the last line it can be scrolled to.
fn render_view(frame: &mut Frame, view: &AnalyzeView, scroll: usize) -> usize {
    render_report(frame, &view.title, scroll, |width| view.lines(width))
}

/// Draw a report overlay (Analyze, Compare) from its line `scroll` on: `lines` lays the report out
/// for a text width. Returns the last line it can be scrolled to.
pub(super) fn render_report<L>(frame: &mut Frame, title: &str, scroll: usize, lines: impl FnOnce(usize) -> L) -> usize
where
    L: std::ops::Deref<Target = [signal::Line]>,
{
    let area = overlay_rect(frame.area());
    frame.render_widget(Clear, area);

    // The report's lines start with a space; one column stays free at the right border too.
    let width = area.width.saturating_sub(3) as usize;
    let rows = area.height.saturating_sub(2) as usize;
    let lines = lines(width);
    let scroll_max = lines.len().saturating_sub(rows);
    let scroll = scroll.min(scroll_max);

    let shown: Vec<Line> = lines
        .iter()
        .skip(scroll)
        .take(rows)
        .map(|line| Line::from(line.iter().map(|s| Span::styled(s.text.clone(), style(s.tone))).collect::<Vec<_>>()))
        // The report is laid out by characters: a line with wide ones in it (a file's name) is
        // cut here with "…", where the border would cut it bare.
        .map(|line| text::truncate_line(line, area.width.saturating_sub(2) as usize))
        .collect();
    let block = overlay_block(title, area.width, scroll, rows, lines.len());
    frame.render_widget(Paragraph::new(shown).block(block), area);
    scroll_max
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analysis::signal::Source;
    use crate::capture::{RfModulation, StoredLevelDuration};
    use crate::ui::text;
    use ratatui::{backend::TestBackend, Terminal};

    /// Three PWM frames of 24 bits (TE 400 µs), 12 ms apart.
    fn view() -> AnalyzeView {
        let mut pairs = Vec::new();
        for _ in 0..3 {
            for bit in 0..24 {
                let (high, low) = if bit % 3 == 0 { (800, 400) } else { (400, 800) };
                pairs.push(StoredLevelDuration { level: true, duration_us: high });
                pairs.push(StoredLevelDuration { level: false, duration_us: low });
            }
            pairs.push(StoredLevelDuration { level: true, duration_us: 400 });
            pairs.push(StoredLevelDuration { level: false, duration_us: 12_000 });
        }
        let source = Source { frequency: 433_920_000, path: Some(RfModulation::AM), ..Source::default() };
        let report = signal::analyze(&pairs, source, None, &[], None, &[]);
        let title = format!("Analyze: #07 Some Protocol V2 (Keeloq) ({})", report.headline());
        let header = ["Signal #07  Some Protocol V2 (Keeloq)", "433.92 MHz", "2026-01-02 03:04:05 UTC+01:00", "a_recording_with_a_long_name.sub"];
        AnalyzeView::new(title, header.map(String::from).to_vec(), report)
    }

    fn screen(width: u16, height: u16, view: &AnalyzeView, scroll: usize) -> (Vec<String>, usize) {
        let mut terminal = Terminal::new(TestBackend::new(width, height)).unwrap();
        let mut max = 0;
        terminal.draw(|frame| max = render_view(frame, view, scroll)).unwrap();
        let buffer = terminal.backend().buffer();
        let rows = (0..height).map(|y| (0..width).map(|x| buffer[(x, y)].symbol()).collect()).collect();
        (rows, max)
    }

    /// The report fits the overlay at every terminal size KAT runs at: no line is cut by the border,
    /// the title is, and the last line is reached at the end of the scroll.
    #[test]
    fn the_report_fits_the_overlay_at_every_size() {
        let view = view();
        for (w, h) in [(60, 16), (80, 24), (100, 30), (160, 50)] {
            let area = overlay_rect(ratatui::layout::Rect::new(0, 0, w, h));
            let inner = area.width as usize - 3;
            let lines: Vec<String> =
                view.lines(inner).iter().map(|l| l.iter().map(|s| s.text.as_str()).collect()).collect();
            assert!(inner >= 54, "{w}x{h}: {inner} columns");
            assert!(lines.iter().all(|l| text::width(l) <= inner), "{w}x{h}: {lines:#?}");

            let (top, max) = screen(w, h, &view, 0);
            assert_eq!(max, lines.len().saturating_sub(area.height as usize - 2), "{w}x{h}");
            let title = &top[area.y as usize];
            assert!(title.contains(" Analyze: #07 Some Protocol V2 (Keeloq) (OOK, PWM"), "{w}x{h}: {title}");
            assert_eq!(title.contains('…'), w == 60, "{w}x{h}: {title}");
            assert!(top[area.y as usize + 1].contains("Signal #07  Some Protocol V2 (Keeloq)  433.92 MHz"), "{w}x{h}: {top:#?}");
            assert!(top.iter().any(|row| row.contains("a_recording_with_a_long_name.sub")), "{w}x{h}: {top:#?}");
            assert!(top.iter().any(|row| row.contains("SUMMARY")), "{w}x{h}: {top:#?}");

            // Scrolled past the end is the end: the last line of the report above the border.
            let (end, _) = screen(w, h, &view, usize::MAX);
            let last = end[(area.y + area.height) as usize - 2].trim_matches(|c| c == '│' || c == ' ').to_string();
            assert_eq!(last, lines.last().unwrap().trim(), "{w}x{h}: {end:#?}");
            let hint = &end[(area.y + area.height) as usize - 1];
            assert!(hint.contains(&format!("of {}", lines.len())) || max == 0, "{w}x{h}: {hint}");
        }
    }

    /// Every kind of text has a style, and the two levels of the waveform differ.
    #[test]
    fn the_levels_of_the_waveform_differ() {
        assert_ne!(style(Tone::High), style(Tone::Low));
        assert_ne!(style(Tone::Heading), style(Tone::Plain));
    }
}
