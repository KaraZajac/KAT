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
        // The list's filter while it is typed (app/list_view.rs)
        InputMode::Filter => (
            format!("/{}", app.list_view.filter),
            "FILTER",
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
        InputMode::SessionRestore | InputMode::ConfirmClear | InputMode::Analyze | InputMode::Compare => {
            let (label, style) = super::app_prompts::mode_label(app.input_mode);
            (String::new(), label, style)
        }
        InputMode::Help | InputMode::Radios | InputMode::ConfirmQuit | InputMode::ConfirmDeleteAll => {
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
    let typing = matches!(app.input_mode, InputMode::Command | InputMode::Filter);
    let cursor = if typing { theme::CURSOR } else { "" };
    let room = (area.width.saturating_sub(2) as usize)
        .saturating_sub(text::width(&label) + 1 + text::width(cursor));
    let input_line = Line::from(vec![
        Span::styled(label, mode_style.add_modifier(Modifier::BOLD)),
        Span::raw(" "),
        Span::raw(text::truncate_start(&input_text, room)),
        Span::styled(cursor, theme::TEXT),
    ]);

    let filtering = app.input_mode == InputMode::Filter;
    let mut block = Block::default().borders(Borders::ALL).title(if filtering { " Filter " } else { " Command " });
    if filtering {
        // What a filter can say, on the lower border.
        let hint = filter_hint(area.width.saturating_sub(2) as usize);
        block = block.title_bottom(Line::styled(hint, Style::default().fg(Color::DarkGray)));
    }
    let input = Paragraph::new(input_line).block(block);

    frame.render_widget(input, area);
}

/// What the filter line says about itself, fullest first: a filter's words, a word for one
/// column, a word to leave out, a capture's ID.
const FILTER_HINTS: [&str; 3] = [
    " every word must fit · serial:1a2b looks in one column · !word leaves out · #7 or id:3-9 by ID ",
    " every word must fit · serial:1a2b one column · !word leaves out · #7 by ID ",
    " every word must fit · serial:1a2b · !word · #7 ",
];

/// The fullest hint that fits the `room` between the box's corners (the shortest fits the
/// narrowest terminal).
fn filter_hint(room: usize) -> &'static str {
    FILTER_HINTS.iter().copied().find(|h| text::width(h) <= room).unwrap_or("")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_filter_lines_hint_fits_its_border() {
        assert!(FILTER_HINTS.windows(2).all(|w| text::width(w[0]) > text::width(w[1])));
        // The narrowest terminal is 60 columns: 58 between the corners.
        assert_eq!(filter_hint(58), FILTER_HINTS[2]);
        assert_eq!(filter_hint(78), FILTER_HINTS[1]);
        assert_eq!(filter_hint(118), FILTER_HINTS[0]);
        assert_eq!(filter_hint(10), "");
        for room in 0..130 {
            assert!(text::width(filter_hint(room)) <= room);
        }
    }
}
