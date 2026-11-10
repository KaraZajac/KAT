//! Status bar widget.

use ratatui::{
    layout::Rect,
    style::{Color, Style},
    text::Line,
    widgets::{Block, Borders, Paragraph},
    Frame,
};

use crate::app::App;

use super::text;

/// Lines of text the bar grows to for a long message (one otherwise: three rows with its border).
pub const MAX_LINES: u16 = 3;

/// The message in the bar and its colour: an error (red) before a status message (green).
fn message(app: &App) -> (String, Style) {
    if let Some(ref error) = app.last_error {
        (format!("Error: {}", error), Style::default().fg(Color::Red))
    } else if let Some(ref status) = app.status_message {
        (status.clone(), Style::default().fg(Color::Green))
    } else {
        (String::new(), Style::default())
    }
}

/// Lines the message takes in a bar `width` columns wide, 1 to [MAX_LINES]: the layout gives the
/// bar that many when the captures area can spare the rows.
pub fn lines_needed(app: &App, width: u16) -> u16 {
    message_lines(&message(app).0, width)
}

fn message_lines(message: &str, width: u16) -> u16 {
    let inner = usize::from(width.saturating_sub(2));
    text::wrap_within(message, inner, usize::from(MAX_LINES)).len() as u16
}

/// Render the status bar with messages and errors
pub fn render_status_bar(frame: &mut Frame, area: Rect, app: &App) {
    let (message, style) = message(app);

    // A message wider than the bar wraps onto the lines the layout gave it (a path and the
    // reason after it were cut in the middle before, and the point of the message with them);
    // one too long even for those keeps its start and its end.
    let inner_width = usize::from(area.width.saturating_sub(2));
    let inner_height = usize::from(area.height.saturating_sub(2));
    let lines: Vec<Line> = text::wrap_within(&message, inner_width, inner_height)
        .into_iter()
        .map(|line| Line::styled(line, style))
        .collect();

    // Background work shows its progress in the title, so it never hides a message: an IQ file,
    // a key search, the transmit queue. The position on the right needs ~14 columns.
    let mut jobs: Vec<String> = Vec::new();
    if let (Some(p), Some(n)) = (app.iq_progress(), app.iq_job_bursts()) {
        jobs.push(format!("IQ file {:.0}% · {n} burst{} (:iq stop)", p * 100.0, crate::plural(n as usize)));
    }
    jobs.extend(app.job_status());
    let title = if jobs.is_empty() {
        " Status ".to_string()
    } else {
        let room = (area.width as usize).saturating_sub(20);
        format!(" {} ", text::truncate_middle(&format!("Status · {}", jobs.join(" · ")), room))
    };

    let status = Paragraph::new(lines).block(
        Block::default()
            .borders(Borders::ALL)
            .title(title)
            .title(Line::from(capture_position(app)).right_aligned()),
    );

    frame.render_widget(status, area);
}

/// " #07 · 3/15 " (the selected capture's ID, then its row in the list as shown and the rows
/// there are), or the number of captures when none is selected: always in view, whatever the
/// message.
fn capture_position(app: &App) -> String {
    let view = app.visible_captures();
    let selected = app.selected_capture.and_then(|i| Some((view.iter().position(|&v| v == i)?, app.captures.get(i)?.id)));
    position_text(selected, view.len(), app.captures.len())
}

/// The position's text: `selected` is the selected capture's row in the list (from 0) and its
/// ID, `shown` the rows of the list, `count` the captures there are (more, under a filter).
fn position_text(selected: Option<(usize, u32)>, shown: usize, count: usize) -> String {
    let captures = |n: usize| format!("{n} capture{}", if n == 1 { "" } else { "s" });
    match selected {
        Some((row, id)) => format!(" #{:02} · {}/{} ", id, row + 1, shown),
        None if count == 0 => " No captures ".to_string(),
        None if shown < count => format!(" {shown} of {} ", captures(count)),
        None => format!(" {} ", captures(count)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_position_counts_the_rows_the_list_shows() {
        assert_eq!(position_text(Some((2, 7)), 15, 15), " #07 · 3/15 ");
        assert_eq!(position_text(None, 0, 0), " No captures ");
        assert_eq!(position_text(None, 1, 1), " 1 capture ");
        assert_eq!(position_text(None, 15, 15), " 15 captures ");
        // Under a filter: the row among the shown ones; with nothing selected, how many are shown.
        assert_eq!(position_text(Some((1, 7)), 3, 15), " #07 · 2/3 ");
        assert_eq!(position_text(None, 0, 15), " 0 of 15 captures ");
        assert_eq!(position_text(None, 1, 1), " 1 capture ");
        assert_eq!(position_text(None, 0, 1), " 0 of 1 capture ");
    }

    #[test]
    fn the_bar_asks_for_the_lines_of_its_message() {
        // Nothing to say, or a short message: one line.
        assert_eq!(message_lines("", 80), 1);
        assert_eq!(message_lines("Transmitted Unlock for #03 (2-FSK ±14.28 kHz, F4)", 80), 1);
        // 78 columns inside the border: a message of 78 fits, one of 79 takes a second line.
        assert_eq!(message_lines(&"x".repeat(78), 80), 1);
        assert_eq!(message_lines(&"x".repeat(79), 80), 2);
        // A path and the reason after it: two lines in the narrowest terminal, where the cut in
        // the middle used to take the end of the path and the start of the reason.
        let error = "Error: Could not read /home/user/captures/gate_remote.sub: Permission denied (os error 13)";
        assert_eq!(
            text::wrap_within(error, 58, usize::from(MAX_LINES)),
            ["Error: Could not read /home/user/captures/gate_remote.sub:", "Permission denied (os error 13)"]
        );
        assert_eq!(message_lines(error, 60), 2);
        assert_eq!(message_lines(error, 120), 1);
        // Never more than the bar grows to.
        assert_eq!(message_lines(&"word ".repeat(200), 60), MAX_LINES);
    }
}
