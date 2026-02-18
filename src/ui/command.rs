//! Command input widget.

use ratatui::{
    layout::Rect,
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Paragraph},
    Frame,
};

use crate::app::{App, InputMode};

/// Render the command input line
pub fn render_command_line(frame: &mut Frame, area: Rect, app: &App) {
    let (input_text, mode_text, mode_style) = match app.input_mode {
        InputMode::Normal => (
            String::new(),
            "NORMAL",
            Style::default().fg(Color::Green),
        ),
        InputMode::Command => (
            format!(":{}", app.command_input),
            "COMMAND",
            Style::default().fg(Color::Yellow),
        ),
        InputMode::SignalMenu => (
            String::new(),
            "SIGNAL",
            Style::default().fg(Color::Cyan),
        ),
        InputMode::SettingsSelect => (
            String::new(),
            "SETTINGS",
            Style::default().fg(Color::Cyan),
        ),
        InputMode::SettingsEdit => (
            String::new(),
            "EDIT",
            Style::default().fg(Color::Green),
        ),
        InputMode::HackRfNotDetected => (
            String::new(),
            "WARNING",
            Style::default().fg(Color::Red),
        ),
        InputMode::StartupImport => (
            String::new(),
            "IMPORT",
            Style::default().fg(Color::Yellow),
        ),
        InputMode::ExportFilename
        | InputMode::FobMetaYear
        | InputMode::FobMetaMake
        | InputMode::FobMetaModel
        | InputMode::FobMetaRegion
        | InputMode::FobMetaNotes => (
            String::new(),
            "EXPORT",
            Style::default().fg(Color::Green),
        ),
        InputMode::CaptureMetaYear
        | InputMode::CaptureMetaMake
        | InputMode::CaptureMetaModel
        | InputMode::CaptureMetaRegion => (
            String::new(),
            "META",
            Style::default().fg(Color::Cyan),
        ),
        InputMode::LoadFileBrowser => (
            String::new(),
            "LOAD",
            Style::default().fg(Color::Cyan),
        ),
        InputMode::License => (
            String::new(),
            "LICENSE",
            Style::default().fg(Color::Cyan),
        ),
        InputMode::Credits => (
            String::new(),
            "CREDITS",
            Style::default().fg(Color::Cyan),
        ),
    };

    let input_line = Line::from(vec![
        Span::styled(
            format!(" {} ", mode_text),
            mode_style.add_modifier(Modifier::BOLD),
        ),
        Span::raw(" "),
        Span::raw(input_text),
        Span::styled(
            if app.input_mode == InputMode::Command {
                "█"
            } else {
                ""
            },
            Style::default(),
        ),
    ]);

    let input = Paragraph::new(input_line).block(
        Block::default()
            .borders(Borders::ALL)
            .title("input"),
    );

    frame.render_widget(input, area);
}
