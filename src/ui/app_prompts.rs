//! Prompts and overlays of the v1.5 app features: the startup session-restore prompt (styled like
//! the startup import prompt), the `:clear` confirmation and the timing-analysis overlay; the quit
//! and `:delete all` confirmations (app/input.rs); plus the mode label and help-bar text of those
//! modes and of the help overlay.

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
        InputMode::ConfirmQuit => render_confirm_quit_prompt(frame, app),
        InputMode::ConfirmDeleteAll => render_confirm_delete_all_prompt(frame, app),
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
        InputMode::Help => ("HELP", Style::default().fg(Color::Cyan)),
        InputMode::ConfirmQuit => ("QUIT", Style::default().fg(Color::Yellow)),
        InputMode::ConfirmDeleteAll => ("DELETE", Style::default().fg(Color::Red)),
        _ => ("", Style::default()),
    }
}

/// Help-bar text (v1.5 modes).
pub fn help_text(mode: InputMode) -> &'static str {
    match mode {
        InputMode::SessionRestore => "y: Restore | n: Start fresh",
        InputMode::ConfirmClear => "y: Clear everything | n: Cancel",
        InputMode::TimingAnalysis => "Up/Down, PgUp/PgDn, Home/End: Scroll | Esc/Enter: Close",
        InputMode::Help => "Up/Down, PgUp/PgDn: Scroll | Esc: Close",
        InputMode::ConfirmQuit => "y: Quit | n: Back",
        InputMode::ConfirmDeleteAll => "y: Delete all | n: Cancel",
        _ => "",
    }
}

fn centered_rect(width: u16, height: u16, area: Rect) -> Rect {
    let x = area.x + area.width.saturating_sub(width) / 2;
    let y = area.y + area.height.saturating_sub(height) / 2;
    Rect::new(x, y, width.min(area.width), height.min(area.height))
}

/// Rows a line takes when word-wrapped to `width` columns (greedy, as the paragraph wraps).
fn wrapped_rows(line: &Line, width: usize) -> u16 {
    let width = width.max(1);
    let text: String = line.spans.iter().map(|s| s.content.as_ref()).collect();
    let (mut rows, mut used) = (1u16, 0usize);
    for word in text.split_whitespace().map(|w| w.chars().count()) {
        if used > 0 && used + 1 + word > width {
            rows += 1;
            used = 0;
        }
        used += if used > 0 { word + 1 } else { word };
        // A word wider than the box breaks over several rows.
        while used > width {
            rows += 1;
            used -= width;
        }
    }
    rows
}

fn render_prompt(frame: &mut Frame, title: &str, border: Color, lines: Vec<Line>) {
    // Long lines wrap: count the rows they take, or the last lines ("(y/n)") are cut off.
    let width = 58.min(frame.area().width);
    let rows: u16 = lines.iter().map(|l| wrapped_rows(l, usize::from(width.saturating_sub(2)))).sum();
    let popup = centered_rect(width, rows + 3, frame.area());
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
            Style::default().add_modifier(Modifier::BOLD),
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
        (0, _) => "Delete the saved session?".to_string(),
        (n, true) => format!(
            "Delete all {} capture{} and the saved session?",
            n,
            if n == 1 { "" } else { "s" }
        ),
        (n, false) => format!("Delete all {} capture{}?", n, if n == 1 { "" } else { "s" }),
    };
    let dim = Style::default().fg(Color::DarkGray);
    let mut lines = vec![
        Line::from(""),
        Line::from(Span::styled(what, Style::default().fg(Color::Red).add_modifier(Modifier::BOLD))),
    ];
    if files {
        // What :clear removes from the disk: the session file and its backup.
        lines.push(Line::from(Span::styled(
            "The saved session file (session.json) and its backup (session.json.bak) are deleted too.",
            dim,
        )));
    }
    lines.extend([
        Line::from(Span::styled("Exported .fob / .sub files are kept. This cannot be undone.", dim)),
        Line::from(""),
        Line::from(Span::styled(
            "(y/n)",
            Style::default().add_modifier(Modifier::BOLD),
        )),
    ]);
    render_prompt(frame, "Clear", Color::Red, lines);
}

/// "Quit? The KeeLoq key search is still running" (`q`, Ctrl-C or `:q` while work runs).
fn render_confirm_quit_prompt(frame: &mut Frame, app: &App) {
    let hint = match app.running_jobs().len() {
        0 => "y quits; n goes back",
        1 => "y stops it and quits; n goes back",
        _ => "y stops them and quits; n goes back",
    };
    let mut lines = vec![
        Line::from(""),
        Line::from(Span::styled(
            app.quit_question(),
            Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD),
        )),
    ];
    // How far it got, to help decide whether to wait.
    if let Some(progress) = app.job_status() {
        lines.push(Line::from(Span::styled(progress, Style::default().fg(Color::DarkGray))));
    }
    lines.extend([
        Line::from(""),
        Line::from(Span::styled(
            "(y/n)",
            Style::default().add_modifier(Modifier::BOLD),
        )),
        Line::from(Span::styled(hint, Style::default().fg(Color::DarkGray))),
    ]);
    render_prompt(frame, "Quit", Color::Yellow, lines);
}

/// `:delete all` confirmation.
fn render_confirm_delete_all_prompt(frame: &mut Frame, app: &App) {
    let n = app.captures.len();
    let lines = vec![
        Line::from(""),
        Line::from(Span::styled(
            format!("Delete all {} capture{}?", n, if n == 1 { "" } else { "s" }),
            Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
        )),
        Line::from(Span::styled(
            "u in the list brings them back. Exported files are kept.",
            Style::default().fg(Color::DarkGray),
        )),
        Line::from(""),
        Line::from(Span::styled(
            "(y/n)",
            Style::default().add_modifier(Modifier::BOLD),
        )),
    ];
    render_prompt(frame, "Delete all", Color::Red, lines);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prompt_rows_count_wrapped_lines() {
        assert_eq!(wrapped_rows(&Line::from(""), 56), 1);
        assert_eq!(wrapped_rows(&Line::from("(y/n)"), 56), 1);
        // 89 characters in 56 columns: two rows.
        let long = "The saved session file (session.json) and its backup (session.json.bak) are deleted too.";
        assert_eq!(wrapped_rows(&Line::from(long), 56), 2);
        assert_eq!(wrapped_rows(&Line::from("aaaa bbbb"), 4), 2);
        assert_eq!(wrapped_rows(&Line::from("abcdefghij"), 4), 3, "a word wider than the box");
    }
}
