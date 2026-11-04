//! Command input widget.

use ratatui::{
    layout::Rect,
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Paragraph},
    Frame,
};

use crate::app::{App, InputMode};

use super::{text, theme};

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
        InputMode::SessionRestore | InputMode::ConfirmClear | InputMode::TimingAnalysis => {
            let (label, style) = super::app_prompts::mode_label(app.input_mode);
            (String::new(), label, style)
        }
        InputMode::Help | InputMode::ConfirmQuit | InputMode::ConfirmDeleteAll => {
            let (label, style) = super::app_prompts::mode_label(app.input_mode);
            (String::new(), label, style)
        }
        InputMode::NewRemote | InputMode::NewRemoteEdit | InputMode::SendButton => {
            let (label, style) = super::add_manually::mode_label(app.input_mode);
            (String::new(), label, style)
        }
        InputMode::ExportFilename
        | InputMode::FobMetaYear
        | InputMode::FobMetaMake
        | InputMode::FobMetaModel
        | InputMode::FobMetaRegion
        | InputMode::FobMetaCommand
        | InputMode::FobMetaNotes => (
            String::new(),
            "EXPORT",
            Style::default().fg(Color::Green),
        ),
        InputMode::CaptureMetaYear
        | InputMode::CaptureMetaMake
        | InputMode::CaptureMetaModel
        | InputMode::CaptureMetaRegion
        | InputMode::CaptureMetaCommand => (
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

    // A long command shows its end (where the typing happens) behind a leading "…", so the
    // cursor never leaves the line.
    let label = format!(" {} ", mode_text);
    let cursor = if app.input_mode == InputMode::Command { theme::CURSOR } else { "" };
    let room = (area.width.saturating_sub(2) as usize)
        .saturating_sub(text::width(&label) + 1 + text::width(cursor));
    let input_line = Line::from(vec![
        Span::styled(label, mode_style.add_modifier(Modifier::BOLD)),
        Span::raw(" "),
        Span::raw(text::truncate_start(&input_text, room)),
        Span::styled(cursor, theme::TEXT),
    ]);

    let input = Paragraph::new(input_line).block(
        Block::default()
            .borders(Borders::ALL)
            .title("Command"),
    );

    frame.render_widget(input, area);
}
