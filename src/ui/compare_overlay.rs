//! The Compare overlay: the comparison of several captures' bits ([`crate::analysis::compare`]),
//! drawn like the Analyze report (`analyze_overlay.rs`).

use ratatui::Frame;

use crate::app::{App, CompareView};

use super::analyze_overlay::render_report;

/// Draw the open Compare overlay and tell the app where its scroll ends.
pub fn render_compare_overlay(frame: &mut Frame, app: &App) {
    if let Some(view) = app.compare_overlay() {
        app.overlay_scroll_max.set(render_view(frame, view, app.overlay_scroll));
    }
}

/// Draw `view` from its line `scroll` on; returns the last line it can be scrolled to.
fn render_view(frame: &mut Frame, view: &CompareView, scroll: usize) -> usize {
    render_report(frame, &view.title(), scroll, |width| view.lines(width))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analysis::compare::{compare, Frame as Bits, Grouping};
    use crate::ui::text;
    use ratatui::{backend::TestBackend, layout::Rect, Terminal};

    /// Twelve presses of a made-up remote: 28 fixed bits, a button in two, a counter in ten, and
    /// 24 bits that change with every press.
    fn view() -> CompareView {
        let bits = |value: u64, len: usize| (0..len).rev().map(|i| value >> i & 1 == 1).collect::<Vec<bool>>();
        let mut seed = 0x2545_F491u32;
        let frames = (0..12u64)
            .map(|i| {
                seed ^= seed << 13;
                seed ^= seed >> 17;
                seed ^= seed << 5;
                let (button, code) = if i < 6 { ("Lock", 1) } else { ("Unlock", 2) };
                let parts = [bits(0xA5C_3F01, 28), bits(code, 2), bits(0x150 + i, 10), bits(u64::from(seed), 24)];
                Bits { name: format!("#{:02}", i + 1), group: Some(button.to_string()), bits: parts.concat() }
            })
            .collect();
        let comparison = compare(frames, "the decoded data of Some Protocol V2".to_string(), Grouping::Button, Vec::new());
        let header = ["Some Protocol V2  serial 0A5C3F0", "433.92 MHz", "Captures #01-#12"].map(String::from).to_vec();
        let other = compare(Vec::new(), "the bits as sent".to_string(), Grouping::Button, Vec::new());
        CompareView::new("12 captures of Some Protocol V2".to_string(), header, vec![("decoded data", comparison), ("bits as sent", other)])
    }

    fn screen(width: u16, height: u16, view: &CompareView, scroll: usize) -> (Vec<String>, usize) {
        let mut terminal = Terminal::new(TestBackend::new(width, height)).unwrap();
        let mut max = 0;
        terminal.draw(|frame| max = render_view(frame, view, scroll)).unwrap();
        let buffer = terminal.backend().buffer();
        let rows = (0..height).map(|y| (0..width).map(|x| buffer[(x, y)].symbol()).collect()).collect();
        (rows, max)
    }

    /// The comparison fits the overlay at every terminal size KAT runs at: no line is cut by the
    /// border, and the last line is reached at the end of the scroll.
    #[test]
    fn the_comparison_fits_the_overlay_at_every_size() {
        let view = view();
        for (w, h) in [(60, 16), (80, 24), (100, 30), (160, 50)] {
            let area = crate::ui::text_overlay::overlay_rect(Rect::new(0, 0, w, h));
            let inner = area.width as usize - 3;
            let lines: Vec<String> =
                view.lines(inner).iter().map(|l| l.iter().map(|s| s.text.as_str()).collect()).collect();
            assert!(lines.iter().all(|l| text::width(l) <= inner), "{w}x{h}: {lines:#?}");
            // Every frame has its row of bits, whatever the width.
            for id in 1..=12 {
                assert!(lines.iter().any(|l| l.trim_start().starts_with(&format!("#{id:02}"))), "{w}x{h} #{id:02}: {lines:#?}");
            }

            let (top, max) = screen(w, h, &view, 0);
            assert_eq!(max, lines.len().saturating_sub(area.height as usize - 2), "{w}x{h}");
            let title = &top[area.y as usize];
            // The narrowest overlay cuts the title's end; the side shown is at its start.
            assert!(title.contains(" Compare (decoded data): 12 captures of Some Protocol"), "{w}x{h}: {title}");
            assert_eq!(title.contains('…'), w == 60, "{w}x{h}: {title}");
            assert!(top[area.y as usize + 1].contains("Some Protocol V2  serial 0A5C3F0  433.92 MHz"), "{w}x{h}: {top:#?}");
            assert!(top.iter().any(|row| row.contains("Tab: the bits as sent")), "{w}x{h}: {top:#?}");

            // Scrolled past the end is the end: the last line of the report above the border, or
            // where a report shorter than the overlay ends.
            let (end, _) = screen(w, h, &view, usize::MAX);
            let last_row = area.y as usize + lines.len().min(area.height as usize - 2);
            let last = end[last_row].trim_matches(|c| c == '│' || c == ' ').to_string();
            assert_eq!(last, lines.last().unwrap().trim(), "{w}x{h}: {end:#?}");
            let hint = &end[(area.y + area.height) as usize - 1];
            assert!(hint.contains(&format!("of {}", lines.len())) || max == 0, "{w}x{h}: {hint}");
        }
    }
}
