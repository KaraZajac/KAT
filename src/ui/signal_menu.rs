//! Signal action popup menu rendered as a centered overlay.

use ratatui::{
    layout::Rect,
    style::{Color, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Clear, List, ListItem},
    Frame,
};

use crate::app::{App, SignalAction};
use crate::capture::Capture;
use crate::radio::hopper::format_mhz;

use super::{text, theme};

/// What says which capture a menu or a form is about: "#07 Kia V3/V4", "433.92 MHz | PWM" and
/// "0x1234567". An unknown capture has no line coding and no serial, and these leave them out
/// (not "Unknown" and "0x-").
pub(super) fn capture_words(c: &Capture) -> (String, String, Option<String>) {
    let coding = c.protocol_info().map(|_| c.modulation().to_string());
    (
        format!("#{:02} {}", c.id, c.protocol_name()),
        join_words([Some(format!("{} MHz", format_mhz(c.frequency))), coding]),
        c.serial.map(|_| format!("0x{}", c.serial_hex())),
    )
}

/// The words there are, with " | " between them.
pub(super) fn join_words<const N: usize>(words: [Option<String>; N]) -> String {
    words.into_iter().flatten().collect::<Vec<_>>().join(" | ")
}

/// The menu's least width, and the note on its bottom border when no radio transmits (the
/// transmit actions are left out then): it fits the narrowest menu.
const MIN_WIDTH: usize = 38;
const NO_TX_NOTE: &str = " TX: needs a HackRF or a Flipper ";

/// Render the signal action popup menu as a centered overlay
pub fn render_signal_menu(frame: &mut Frame, app: &App) {
    let area = frame.area();

    // The capture the menu was opened on (it stays put while new captures arrive).
    let (capture_info, freq_info) = match app.signal_menu_capture() {
        Some(c) => {
            let (name, tuning, serial) = capture_words(c);
            (join_words([Some(name), serial]), tuning)
        }
        None => ("No capture".to_string(), String::new()),
    };

    let actions = app.available_signal_actions();

    // Wide enough for the title and the longest action, up to the terminal.
    let longest = actions.iter().map(|a| text::width(a.label())).max().unwrap_or(0);
    let menu_width = (longest + 5)
        .max(text::width(&capture_info) + 6)
        .max(MIN_WIDTH)
        .min(area.width as usize) as u16;
    let inner_width = menu_width.saturating_sub(2) as usize;
    // A title longer than the terminal is wide is cut with "…" (the border would cut it bare).
    let title = format!(" {} ", text::truncate_end(&capture_info, inner_width.saturating_sub(2)));

    // As tall as its lines, leaving the help bar in view: the info lines go first when there is
    // not room for them and three actions; then the actions scroll to keep the highlighted one in
    // view.
    let room = area.height.saturating_sub(2) as usize;
    let info_lines = if freq_info.is_empty() || room < actions.len().min(3) + 4 { 0 } else { 2 };
    let menu_height = (actions.len() + info_lines + 2).min(room) as u16;
    let visible = (menu_height as usize).saturating_sub(2 + info_lines);
    let start = text::centered_start(app.signal_menu_index, visible, actions.len());

    // Center the menu
    let x = area.width.saturating_sub(menu_width) / 2;
    let y = area.height.saturating_sub(menu_height) / 2;
    let menu_area = Rect::new(x, y, menu_width, menu_height);

    // Clear the area behind the popup
    frame.render_widget(Clear, menu_area);

    // Build list items
    let mut items: Vec<ListItem> = Vec::new();

    // Add signal info lines at the top if we have capture data
    if info_lines > 0 {
        items.push(ListItem::new(Line::from(Span::styled(
            format!("  {}", freq_info),
            Style::default().fg(Color::DarkGray),
        ))));
        items.push(ListItem::new(Line::from(Span::raw(""))));
    }

    // Add action items (only available actions are shown)
    for (i, action) in actions.iter().enumerate().skip(start).take(visible) {
        let selected = i == app.signal_menu_index;
        let style = if selected {
            theme::HIGHLIGHT
        } else {
            match action {
                SignalAction::Delete => Style::default().fg(Color::Red),
                SignalAction::ExportFob | SignalAction::ExportFlipper | SignalAction::ExportFlipperKey => {
                    Style::default().fg(Color::Green)
                }
                _ => theme::TEXT,
            }
        };

        let prefix = if selected { " > " } else { "   " };
        let label = text::truncate_end(action.label(), inner_width.saturating_sub(3));
        // The highlight spans the whole row.
        let pad = inner_width.saturating_sub(3 + text::width(&label));
        items.push(ListItem::new(Line::from(Span::styled(
            format!("{}{}{}", prefix, label, " ".repeat(pad)),
            style,
        ))));
    }

    let mut block = Block::default()
        .title(title)
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Cyan));
    // Actions out of view (a short terminal): say how many, above and below. Else, without a
    // radio that transmits, why the menu has no Replay, Lock or Unlock.
    if let Some(hint) = text::more_hint(start, visible, actions.len()) {
        block = block.title_bottom(Line::from(hint).right_aligned());
    } else if app.tx_radio().is_none() {
        block = block.title_bottom(Line::styled(NO_TX_NOTE, Style::default().fg(Color::DarkGray)));
    }

    frame.render_widget(List::new(items).block(block), menu_area);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The note that says why the transmit actions are missing fits the narrowest menu's border.
    #[test]
    fn the_no_tx_note_fits_the_narrowest_menu() {
        assert!(text::width(NO_TX_NOTE) <= MIN_WIDTH - 2, "{}", text::width(NO_TX_NOTE));
    }
}
