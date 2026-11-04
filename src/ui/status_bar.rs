//! Status bar widget.

use ratatui::{
    layout::Rect,
    style::{Color, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Paragraph},
    Frame,
};

use crate::app::App;

use super::text;

/// Render the status bar with messages and errors
pub fn render_status_bar(frame: &mut Frame, area: Rect, app: &App) {
    let (message, style) = if let Some(ref error) = app.last_error {
        (
            format!("Error: {}", error),
            Style::default().fg(Color::Red),
        )
    } else if let Some(ref status) = app.status_message {
        (status.clone(), Style::default().fg(Color::Green))
    } else {
        (String::new(), Style::default())
    };

    // A message wider than the bar keeps its start and its end (often the reason or the file).
    let message = text::truncate_middle(&message, area.width.saturating_sub(2) as usize);
    let status_line = Line::from(vec![Span::styled(message, style)]);

    // Background work shows its progress in the title, so it never hides a message: an IQ file,
    // a key search, the transmit queue. The position on the right needs ~14 columns.
    let mut jobs: Vec<String> = Vec::new();
    if let (Some(p), Some(n)) = (app.rx_header().iq_progress, app.iq_job_bursts()) {
        jobs.push(format!("IQ file {:.0}% · {} burst(s) (:iq stop)", p * 100.0, n));
    }
    jobs.extend(app.job_status());
    let title = if jobs.is_empty() {
        "Status".to_string()
    } else {
        let room = (area.width as usize).saturating_sub(18);
        text::truncate_middle(&format!("Status · {}", jobs.join(" · ")), room)
    };

    let status = Paragraph::new(status_line).block(
        Block::default()
            .borders(Borders::ALL)
            .title(title)
            .title(Line::from(capture_position(app)).right_aligned()),
    );

    frame.render_widget(status, area);
}

/// " #07 · 3/15 " (the selected capture's ID, then its place in the list), or the number of
/// captures when none is selected: always in view, whatever the message.
fn capture_position(app: &App) -> String {
    let count = app.captures.len();
    match app.selected_capture.and_then(|i| app.captures.get(i).map(|c| (i, c))) {
        Some((i, capture)) => format!(" #{:02} · {}/{} ", capture.id, i + 1, count),
        None if count == 0 => " No captures ".to_string(),
        None => format!(" {} capture{} ", count, if count == 1 { "" } else { "s" }),
    }
}
