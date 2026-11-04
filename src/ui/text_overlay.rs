//! Centered text overlays: License, Credits and the timing analysis.

use ratatui::{
    layout::{Alignment, Margin, Rect},
    style::{Color, Style},
    text::Line,
    widgets::{Block, Borders, Clear, Paragraph},
    Frame,
};

use crate::app::App;

use super::text;

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

/// `?` / `F1` / `:help`: keys and commands (preformatted; README "Using the TUI" has the full
/// tables). Kept narrow (≤ 62 columns) so it fits an 80-column terminal's overlay unwrapped.
pub const HELP_TEXT: &str = r#"KEYS
  ↑ ↓  j k      Move through the captures
  PgUp PgDn     Move a page
  Home End  g G First / last capture
  Enter         Actions: transmit, export, timing,
                key recovery
  i             Vehicle info (year, make, model …)
  d  u          Delete the capture / undo
  r             Start or stop receiving
  Tab           Radio settings
  :             Command line
  ?  F1         This help
  Esc           Close; clear the message; cancel
                queued transmissions
  q  Ctrl-C     Quit (asks while work is running)

  Forms: Tab/↓ next field, Shift-Tab/↑ previous.
  File browser: ←/Backspace parent folder, → open.

COMMANDS   (ID: 4  #4  1,3,5  1 3 5  1-5)
  :freq <MHz>          Receive frequency
  :hop on|off          Frequency hopping
  :hop <MHz> …         Hop list (:hop default resets)
  :lna <0-40>          LNA gain
  :vga <0-62>          VGA gain
  :amp [on|off]        RF amplifier
  :txpower <0-47> [amp on|off]
                       Transmit power
  :lock <ID>           Send Lock (also :unlock, :trunk,
                       :panic)
  :replay <ID>         Re-send captures as recorded
  :load [file|dir]     Import .fob/.sub, decode IQ, browse
  :iq <file> [MHz] [rate]
                       Decode an IQ recording (:iq stop)
  :record [file]       Record raw IQ (:record stop)
  :new [type…]         New gate/garage remote
  :klbf [ID] [type]    Recover a KeeLoq key (:klbf stop)
  :timing [ID]         Timing analysis
  :delete <ID>|all     Delete captures (u: undo)
  :clear               Delete all captures + saved session
  :license  :credits   License, credits
  :q                   Quit"#;

/// Where the overlays go: most of the terminal (80% wide, 85% high), centered.
fn overlay_rect(area: Rect) -> Rect {
    let width = (area.width as f32 * 0.80) as u16;
    let height = (area.height as f32 * 0.85) as u16;
    let x = area.width.saturating_sub(width) / 2;
    let y = area.height.saturating_sub(height) / 2;
    Rect::new(x, y, width.min(area.width), height.min(area.height))
}

/// Bottom-border hint of an overlay showing `rows` of `total` lines from `scroll`: the lines in
/// view and the keys.
fn position_hint(scroll: usize, rows: usize, total: usize) -> String {
    if total <= rows {
        " Esc: close ".to_string()
    } else {
        format!(" {}–{} of {} · ↑↓ scroll · Esc: close ", scroll + 1, (scroll + rows).min(total), total)
    }
}

/// The overlay's frame: cyan border, title, position hint at the bottom right.
fn overlay_block(title: &str, scroll: usize, rows: usize, total: usize) -> Block<'static> {
    Block::default()
        .borders(Borders::ALL)
        .title(format!(" {} ", title))
        .title_bottom(Line::from(position_hint(scroll, rows, total)).right_aligned())
        .border_style(Style::default().fg(Color::Cyan))
}

/// Render a centered overlay with a title and scrollable text (e.g. License or Credits). The text
/// is word-wrapped here to the overlay's width, so the line count and the scroll limit are exact.
/// Returns the largest useful scroll offset for this frame size (where the keys should stop).
pub fn render_text_overlay(
    frame: &mut Frame,
    app: &App,
    title: &str,
    body: &str,
    alignment: Alignment,
) -> usize {
    let overlay_area = overlay_rect(frame.area());
    frame.render_widget(Clear, overlay_area);

    // One column of margin inside each border.
    let text_area = overlay_area.inner(Margin { horizontal: 2, vertical: 1 });
    let lines = text::wrap(body, text_area.width as usize);
    let rows = text_area.height as usize;
    let scroll_max = lines.len().saturating_sub(rows);
    let scroll = app.overlay_scroll.min(scroll_max);

    frame.render_widget(overlay_block(title, scroll, rows, lines.len()), overlay_area);
    let paragraph = Paragraph::new(lines.into_iter().map(Line::from).collect::<Vec<_>>())
        .scroll((scroll as u16, 0))
        .alignment(alignment);
    frame.render_widget(paragraph, text_area);
    scroll_max
}

/// Same centered overlay for preformatted text (timing analysis): indentation is kept, long lines
/// are word-wrapped here (so the line count is exact) and the scroll stops at the last line.
/// Returns the largest useful scroll offset for this frame size.
pub fn render_preformatted_overlay(frame: &mut Frame, app: &App, title: &str, text: &str) -> usize {
    let overlay_area = overlay_rect(frame.area());
    frame.render_widget(Clear, overlay_area);

    let lines = wrap_preformatted(text, overlay_area.width.saturating_sub(2) as usize);
    let inner_height = overlay_area.height.saturating_sub(2) as usize;
    let scroll_max = lines.len().saturating_sub(inner_height);
    let scroll = app.overlay_scroll.min(scroll_max);

    let paragraph = Paragraph::new(lines.iter().map(|l| Line::from(l.as_str())).collect::<Vec<_>>())
        .block(overlay_block(title, scroll, inner_height, lines.len()))
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
    use super::{position_hint, wrap_preformatted, CREDITS_TEXT};
    use crate::ui::text;

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

    #[test]
    fn overlay_scroll_counts_wrapped_lines() {
        // The 80×24 overlay: 64 columns, 20 rows; 60×18 of text inside the margins.
        let license = text::wrap(crate::app::LICENSE_TEXT, 60);
        assert!(license.len() > crate::app::LICENSE_TEXT.lines().count());
        assert!(license.iter().all(|l| text::width(l) <= 60));
        // The Credits' long paragraph takes several lines: counting source lines would stop the
        // scroll before the last names.
        let credits = text::wrap(CREDITS_TEXT, 60);
        assert!(credits.len() > CREDITS_TEXT.lines().count());
        assert_eq!(credits.last().map(String::as_str), Some("Li0ard"));
        assert_eq!(position_hint(0, 18, 10), " Esc: close ");
        assert_eq!(position_hint(12, 18, 41), " 13–30 of 41 · ↑↓ scroll · Esc: close ");
    }
}
