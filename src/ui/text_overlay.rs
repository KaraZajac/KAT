//! Centered text overlays: License, Credits, the help and the radios; and the frame they share
//! with the Analyze overlay (`analyze_overlay.rs`).

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

Protocols ProtoPirate does not carry are ported from Flipper-ARF (D4C1-Labs), The Doctor's firmware (thedoctorz), Momentum (Next-Flip) and RogueMaster, and the receive-only decoders from rtl_433 (Benjamin Larsson, Christian W. Zuckschwerdt and contributors), checked on the rtl_433_tests recordings.

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
/// tables). Kept narrow (56 columns at most) so no line of it wraps in the narrowest terminal's
/// overlay.
pub const HELP_TEXT: &str = r#"KEYS
  ↑ ↓  j k      Move through the captures
  PgUp PgDn     Move a page
  Home End  g G First / last capture
  Enter         Actions: transmit, export, analyze,
                key recovery
  a             Analyze: what the signal is
                (← →: the captures next to it)
  i             Vehicle info (year, make, model …)
  d  u          Delete the capture / undo
  r             Start or stop receiving
  R             Start or stop every radio
  1-9           Select a radio (several radios)
  Tab           Radio settings
  :             Command line (↑ ↓: earlier commands)
  ?  F1         This help
  Esc           Close; clear the message; cancel
                queued transmissions
  q  Ctrl-C     Quit (asks while work is running)

  Forms: Tab/↓ next field, Shift-Tab/↑ previous.
  Typing: Ctrl-U clears the line, Ctrl-W a word.
  File browser: ←/Backspace parent folder, → open.

COMMANDS   (ID: 4  #4  1,3,5  1 3 5  1-5)
  :radios              The radios, and any new ones
  :radio <n>           Select a radio (the ones below
                       tune the selected radio)
  :txradio <n>         The radio that transmits
  :freq <MHz>          Receive frequency
  :hop on|off          Frequency hopping
  :hop <MHz> …         Hop list (:hop default resets)
  :lna <0-40>          LNA gain (HackRF)
  :vga <0-62>          VGA gain (an RTL-SDR's one gain)
  :amp [on|off]        RF amplifier (HackRF)
  :txpower <0-47> [amp on|off]
                       Transmit power (HackRF)
  :lock <ID>           Send Lock (also :unlock, :trunk,
                       :panic)
  :replay <ID>         Re-send captures as recorded
  :load [file|dir]     Load .fob/.sub/IQ files, browse
  :iq <file> [MHz] [rate]
                       Decode an IQ recording (:iq stop)
  :record [file]       Record raw IQ (:record stop)
  :new [type…]         New gate/garage remote
  :klbf [ID] [type]    Recover a KeeLoq key (:klbf stop)
  :analyze [ID]        What a signal is: modulation,
                       pulse widths, coding, bits
  :delete <ID>|all     Delete captures (u: undo)
  :clear               Delete captures + saved session
  :license  :credits   License, credits
  :q  :q!              Quit (:q! without asking)"#;

/// On a small terminal an overlay takes up to this much of it: the help and the reports are laid
/// out for some 60 to 76 columns, and 80% of an 80-column terminal wrapped them.
const SMALL_TERMINAL_WIDTH: u16 = 78;
const SMALL_TERMINAL_HEIGHT: u16 = 22;

/// Where the overlays go: most of the terminal (80% wide, 85% high), centered; on a small terminal
/// all of it but a column at each side and a row above and below.
pub(super) fn overlay_rect(area: Rect) -> Rect {
    let width = ((area.width as f32 * 0.80) as u16).max(area.width.saturating_sub(2).min(SMALL_TERMINAL_WIDTH));
    let height = ((area.height as f32 * 0.85) as u16).max(area.height.saturating_sub(2).min(SMALL_TERMINAL_HEIGHT));
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

/// The frame of an overlay `width` columns wide: cyan border, the title (cut with "…" where it is
/// longer than the border), position hint at the bottom right.
pub(super) fn overlay_block(title: &str, width: u16, scroll: usize, rows: usize, total: usize) -> Block<'static> {
    Block::default()
        .borders(Borders::ALL)
        .title(format!(" {} ", text::truncate_end(title, (width as usize).saturating_sub(4))))
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

    frame.render_widget(overlay_block(title, overlay_area.width, scroll, rows, lines.len()), overlay_area);
    let paragraph = Paragraph::new(lines.into_iter().map(Line::from).collect::<Vec<_>>())
        .scroll((scroll as u16, 0))
        .alignment(alignment);
    frame.render_widget(paragraph, text_area);
    scroll_max
}

/// Same centered overlay for preformatted text (the help, the radios): indentation is kept, long
/// lines are word-wrapped here (so the line count is exact) and the scroll stops at the last line.
/// Returns the largest useful scroll offset for this frame size.
pub fn render_preformatted_overlay(frame: &mut Frame, app: &App, title: &str, text: &str) -> usize {
    let overlay_area = overlay_rect(frame.area());
    frame.render_widget(Clear, overlay_area);

    let lines = wrap_preformatted(text, overlay_area.width.saturating_sub(2) as usize);
    let inner_height = overlay_area.height.saturating_sub(2) as usize;
    let scroll_max = lines.len().saturating_sub(inner_height);
    let scroll = app.overlay_scroll.min(scroll_max);

    let paragraph = Paragraph::new(lines.iter().map(|l| Line::from(l.as_str())).collect::<Vec<_>>())
        .block(overlay_block(title, overlay_area.width, scroll, inner_height, lines.len()))
        .scroll((scroll as u16, 0));

    frame.render_widget(paragraph, overlay_area);
    scroll_max
}

/// Wrap preformatted lines to `width` columns, breaking at a space where possible; continuation
/// lines keep the line's indentation plus two spaces.
fn wrap_preformatted(body: &str, width: usize) -> Vec<String> {
    let mut out = Vec::new();
    for line in body.lines() {
        if width == 0 || text::width(line) <= width {
            out.push(line.to_string());
            continue;
        }
        let indent = line.chars().take_while(|c| *c == ' ').count();
        let continuation = " ".repeat((indent + 2).min(width / 2));
        let mut rest: Vec<char> = line.chars().collect();
        let mut prefix = String::new();
        loop {
            // The characters that fit after the prefix (spaces, a column each), by the columns
            // they take: a wide one takes two.
            let room = text::take_width(rest.iter().copied(), width - prefix.len()).len();
            if room == rest.len() {
                out.push(format!("{}{}", prefix, rest.iter().collect::<String>()));
                break;
            }
            // At least one character a line (a wide one in a single column), or this never ends.
            let room = room.max(1);
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
    use super::{overlay_rect, position_hint, wrap_preformatted, CREDITS_TEXT, HELP_TEXT};
    use crate::ui::text;
    use ratatui::layout::Rect;

    /// A small terminal gives an overlay all but its edge; a large one 80% by 85%, centered.
    #[test]
    fn overlays_take_most_of_a_small_terminal() {
        let rect = |w, h| overlay_rect(Rect::new(0, 0, w, h));
        assert_eq!(rect(60, 16), Rect::new(1, 1, 58, 14));
        assert_eq!(rect(80, 24), Rect::new(1, 1, 78, 22));
        assert_eq!(rect(100, 30), Rect::new(10, 2, 80, 25));
        assert_eq!(rect(200, 60), Rect::new(20, 4, 160, 51));
        // Smaller than KAT runs at (it shows a notice instead): nothing, and nothing overflows.
        assert_eq!(rect(1, 1), Rect::new(0, 0, 0, 0));
    }

    /// The help is laid out by hand: no line of it wraps in the narrowest terminal's overlay.
    #[test]
    fn the_help_fits_the_narrowest_overlay() {
        let inner = overlay_rect(Rect::new(0, 0, crate::ui::layout::MIN_WIDTH, crate::ui::layout::MIN_HEIGHT)).width as usize - 2;
        for line in HELP_TEXT.lines() {
            assert!(text::width(line) <= inner, "{} columns, {inner} at most: {line}", text::width(line));
        }
        assert_eq!(wrap_preformatted(HELP_TEXT, inner).len(), HELP_TEXT.lines().count());
    }

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
        // Wide characters take two columns each.
        let wide = wrap_preformatted("日本語日本語", 6);
        assert_eq!(wide, vec!["日本語", "  日本", "  語"]);
        assert!(wide.iter().all(|l| text::width(l) <= 6));
        assert_eq!(wrap_preformatted("日本", 1), vec!["日", "本"], "no room for one: one a line");
    }

    #[test]
    fn overlay_scroll_counts_wrapped_lines() {
        // Text 60 columns wide, as inside the margins of a 64-column overlay.
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
