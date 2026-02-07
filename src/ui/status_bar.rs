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

    let status = Paragraph::new(status_line).block(
        Block::default()
            .borders(Borders::ALL)
            .title("status"),
    );

    frame.render_widget(status, area);
}
