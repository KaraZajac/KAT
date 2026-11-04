//! Status bar widget.

use ratatui::{
    layout::Rect,
    style::{Color, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Paragraph},
    Frame,
};

use crate::app::App;

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
        (
            format!("Captures: {}", app.captures.len()),
            Style::default().fg(Color::DarkGray),
        )
    };

    let status_line = Line::from(vec![Span::styled(message, style)]);

    // An IQ file being processed shows its progress in the title.
    let title = match (app.rx_header().iq_progress, app.iq_job_bursts()) {
        (Some(p), Some(n)) => format!("status · IQ file {:.0}% · {} burst(s) (:iq stop)", p * 100.0, n),
        _ => "status".to_string(),
    };

    let status = Paragraph::new(status_line).block(
        Block::default()
            .borders(Borders::ALL)
            .title(title),
    );

    frame.render_widget(status, area);
}
