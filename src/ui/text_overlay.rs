//! Centered text overlays: License, Credits and the timing analysis.

use ratatui::{
    layout::{Alignment, Rect},
    style::{Color, Style},
    text::Line,
    widgets::{Block, Borders, Clear, Paragraph, Wrap},
    Frame,
};

use crate::app::App;

/// Credits text for the :credits command
pub const CREDITS_TEXT: &str = r#"=============== CREDITS ==============

KAT is developed by Kara Zajac (.kara).

KAT would not be possible without ProtoPirate. The protocol decoders, reference implementations, and community work are the foundation this tool is built on. I am truly standing on the shoulders of giants.

ProtoPirate Development Team
----
RocketGod
MMX
Leeroy
gullradriel
Skorp - Thanks, I sneaked a lot from Weather App!
Vadim's Radio Driver

Protocol Magic
----
L0rdDiakon
YougZ
RocketGod
MMX
DoobTheGoober
Skorp
Slackware
Trikk
Wootini
Li0ard
Leeroy

Reverse Engineering Support
----
DoobTheGoober
MMX
NeedNotApply
RocketGod
Slackware
Trikk
Li0ard"#;

/// Render a centered overlay with a title and scrollable text (e.g. License or Credits).
pub fn render_text_overlay(
    frame: &mut Frame,
    app: &App,
    title: &str,
    text: &str,
    alignment: Alignment,
) {
    let area = frame.area();

    // Use most of the terminal: 80% width, 85% height, centered
    let width = (area.width as f32 * 0.80) as u16;
    let height = (area.height as f32 * 0.85) as u16;
    let x = area.width.saturating_sub(width) / 2;
    let y = area.height.saturating_sub(height) / 2;
    let overlay_area = Rect::new(x, y, width.min(area.width), height.min(area.height));

    frame.render_widget(Clear, overlay_area);

    let total_lines = text.lines().count();
    let inner_height = overlay_area.height.saturating_sub(2) as usize; // minus border
    let scroll_max = total_lines.saturating_sub(inner_height).saturating_sub(1);
    let scroll = app.overlay_scroll.min(scroll_max);

    let paragraph = Paragraph::new(
        text.lines()
            .map(Line::from)
            .collect::<Vec<_>>(),
    )
    .block(
        Block::default()
            .borders(Borders::ALL)
            .title(format!(" {} ", title))
            .border_style(Style::default().fg(Color::Cyan)),
    )
    .wrap(Wrap { trim: true })
    .scroll((scroll as u16, 0))
    .alignment(alignment);

    frame.render_widget(paragraph, overlay_area);
}

/// Same centered overlay for preformatted text (timing analysis): indentation is kept, long lines
/// are word-wrapped here (so the line count is exact) and the scroll stops at the last line.
/// Returns the largest useful scroll offset for this frame size.
pub fn render_preformatted_overlay(frame: &mut Frame, app: &App, title: &str, text: &str) -> usize {
    let area = frame.area();
    let width = (area.width as f32 * 0.80) as u16;
    let height = (area.height as f32 * 0.85) as u16;
    let x = area.width.saturating_sub(width) / 2;
    let y = area.height.saturating_sub(height) / 2;
    let overlay_area = Rect::new(x, y, width.min(area.width), height.min(area.height));

    frame.render_widget(Clear, overlay_area);

    let lines = wrap_preformatted(text, overlay_area.width.saturating_sub(2) as usize);
    let inner_height = overlay_area.height.saturating_sub(2) as usize;
    let scroll_max = lines.len().saturating_sub(inner_height);
    let scroll = app.overlay_scroll.min(scroll_max);

    let paragraph = Paragraph::new(lines.into_iter().map(Line::from).collect::<Vec<_>>())
        .block(
            Block::default()
                .borders(Borders::ALL)
                .title(format!(" {} ", title))
                .border_style(Style::default().fg(Color::Cyan)),
        )
        .scroll((scroll as u16, 0));

    frame.render_widget(paragraph, overlay_area);
    scroll_max
}

/// Wrap preformatted lines to `width` columns, breaking at a space where possible; continuation
/// lines keep the line's indentation plus two spaces.
fn wrap_preformatted(text: &str, width: usize) -> Vec<String> {
    let mut out = Vec::new();
    for line in text.lines() {
        if width == 0 || line.chars().count() <= width {
            out.push(line.to_string());
            continue;
        }
        let indent = line.chars().take_while(|c| *c == ' ').count();
        let continuation = " ".repeat((indent + 2).min(width / 2));
        let mut rest: Vec<char> = line.chars().collect();
        let mut prefix = String::new();
        loop {
            let room = width - prefix.chars().count();
            if rest.len() <= room {
                out.push(format!("{}{}", prefix, rest.iter().collect::<String>()));
                break;
            }
            let cut = rest[..room].iter().rposition(|c| *c == ' ').filter(|&p| p > 0).unwrap_or(room);
            out.push(format!("{}{}", prefix, rest[..cut].iter().collect::<String>()));
            rest = rest[cut..].iter().skip_while(|c| **c == ' ').copied().collect();
            if rest.is_empty() {
                break;
            }
            prefix = continuation.clone();
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::wrap_preformatted;

    #[test]
    fn preformatted_wrap_keeps_indentation_and_words() {
        let text = "Title\n  Short Avg: 812 us  Min: 640 us  Max: 980 us  Samples: 1203\n\nend";
        let lines = wrap_preformatted(text, 30);
        assert_eq!(
            lines,
            vec![
                "Title",
                "  Short Avg: 812 us  Min: 640",
                "    us  Max: 980 us  Samples:",
                "    1203",
                "",
                "end",
            ]
        );
        assert!(lines.iter().all(|l| l.chars().count() <= 30));
        // No space to break at: hard cut; zero width leaves the text alone.
        assert_eq!(wrap_preformatted("abcdefghij", 4), vec!["abcd", "  ef", "  gh", "  ij"]);
        assert_eq!(wrap_preformatted("abc", 0), vec!["abc"]);
    }
}
