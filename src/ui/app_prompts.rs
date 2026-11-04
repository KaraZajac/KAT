//! Prompts and overlays of the v1.5 app features: the startup session-restore prompt (styled like
//! the startup import prompt), the `:clear` confirmation and the timing-analysis overlay, plus the
//! mode label and help-bar text of those modes.

use ratatui::{
    layout::{Alignment, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Clear, Paragraph, Wrap},
    Frame,
};

use crate::app::{App, InputMode};

use super::text_overlay::render_preformatted_overlay;

/// Draw the overlay of the current v1.5 mode, if any (called last by `draw_ui`).
pub fn render_app_overlays(frame: &mut Frame, app: &App) {
    match app.input_mode {
        InputMode::SessionRestore => render_session_restore_prompt(frame, app),
        InputMode::ConfirmClear => render_confirm_clear_prompt(frame, app),
        InputMode::TimingAnalysis => {
            if let Some(view) = app.timing_overlay() {
                let scroll_max = render_preformatted_overlay(frame, app, &view.title, &view.text);
                view.set_scroll_max(scroll_max);
            }
        }
        _ => {}
    }
}

/// Mode label for the command line (v1.5 modes).
pub fn mode_label(mode: InputMode) -> (&'static str, Style) {
    match mode {
        InputMode::SessionRestore => ("RESTORE", Style::default().fg(Color::Yellow)),
        InputMode::ConfirmClear => ("CLEAR", Style::default().fg(Color::Red)),
        InputMode::TimingAnalysis => ("TIMING", Style::default().fg(Color::Cyan)),
        _ => ("", Style::default()),
    }
}

/// Help-bar text (v1.5 modes).
pub fn help_text(mode: InputMode) -> &'static str {
    match mode {
        InputMode::SessionRestore => "y: Restore | n: Start fresh",
        InputMode::ConfirmClear => "y: Clear everything | n: Cancel",
        InputMode::TimingAnalysis => "Up/Down, PgUp/PgDn, Home/End: Scroll | Esc/Enter: Close",
        _ => "",
    }
}

fn centered_rect(width: u16, height: u16, area: Rect) -> Rect {
    let x = area.x + area.width.saturating_sub(width) / 2;
    let y = area.y + area.height.saturating_sub(height) / 2;
    Rect::new(x, y, width.min(area.width), height.min(area.height))
}

fn render_prompt(frame: &mut Frame, title: &str, border: Color, lines: Vec<Line>) {
    let popup = centered_rect(58, lines.len() as u16 + 3, frame.area());
    frame.render_widget(Clear, popup);
    let block = Block::default()
        .title(format!(" {} ", title))
        .borders(Borders::ALL)
        .border_style(Style::default().fg(border));
    let paragraph = Paragraph::new(lines)
        .block(block)
        .alignment(Alignment::Center)
        .wrap(Wrap { trim: true });
    frame.render_widget(paragraph, popup);
}

/// "Restore N captures from your last session? (y/n)"
fn render_session_restore_prompt(frame: &mut Frame, app: &App) {
    let Some(p) = app.session_restore_prompt() else {
        return;
    };
    let question = if p.count < p.total {
        format!("Restore the newest {} of {} captures from your last session?", p.count, p.total)
    } else {
        format!(
            "Restore {} capture{} from your last session?",
            p.count,
            if p.count == 1 { "" } else { "s" }
        )
    };
    let saved = p
        .saved_at
        .map(|t| format!("saved {}", t.format("%Y-%m-%d %H:%M UTC")))
        .unwrap_or_default();
    let lines = vec![
        Line::from(""),
        Line::from(Span::styled(question, Style::default().fg(Color::Yellow))),
        Line::from(Span::styled(saved, Style::default().fg(Color::DarkGray))),
        Line::from(""),
        Line::from(Span::styled(
            "(y/n)",
            Style::default().fg(Color::White).add_modifier(Modifier::BOLD),
        )),
        Line::from(Span::styled(
            "n starts fresh; the old session is kept as session.json.bak",
            Style::default().fg(Color::DarkGray),
        )),
    ];
    render_prompt(frame, "Restore Session", Color::Cyan, lines);
}

/// `:clear` confirmation.
fn render_confirm_clear_prompt(frame: &mut Frame, app: &App) {
    let (captures, files) = app.clear_prompt();
    let what = match (captures, files) {
        (0, _) => "Delete the saved session file?".to_string(),
        (n, true) => format!(
            "Delete all {} capture{} and the saved session file?",
            n,
            if n == 1 { "" } else { "s" }
        ),
        (n, false) => format!("Delete all {} capture{}?", n, if n == 1 { "" } else { "s" }),
    };
    let lines = vec![
        Line::from(""),
        Line::from(Span::styled(what, Style::default().fg(Color::Red).add_modifier(Modifier::BOLD))),
        Line::from(Span::styled(
            "Exported .fob / .sub files are kept. This cannot be undone.",
            Style::default().fg(Color::DarkGray),
        )),
        Line::from(""),
        Line::from(Span::styled(
            "(y/n)",
            Style::default().fg(Color::White).add_modifier(Modifier::BOLD),
        )),
    ];
    render_prompt(frame, "Clear", Color::Red, lines);
}
