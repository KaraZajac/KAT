//! Centered text overlay for License and Credits.

use ratatui::{
    layout::{Alignment, Rect},
    style::{Color, Style},
    text::Line,
    widgets::{Block, Borders, Clear, Paragraph, Wrap},
    Frame,
};

use crate::app::App;

/// Credits text for the :credits command
pub const CREDITS_TEXT: &str = r#"=============== CREDITS ==============

KAT is developed by Kara Zajac (.leviathan).

KAT would not be possible without ProtoPirate. The protocol decoders, reference implementations, and community work are the foundation this tool is built on. I am truly standing on the shoulders of giants.

ProtoPirate Development Team
----
RocketGod
MMX
Leeroy
gullradriel
Skorp - Thanks, I sneaked a lot from Weather App!
Vadim's Radio Driver

Protocol Magic
----
L0rdDiakon
YougZ
RocketGod
MMX
DoobTheGoober
Skorp
Slackware
Trikk
Wootini
Li0ard
Leeroy

Reverse Engineering Support
----
DoobTheGoober
MMX
NeedNotApply
RocketGod
Slackware
Trikk
Li0ard"#;

/// Render a centered overlay with a title and scrollable text (e.g. License or Credits).
pub fn render_text_overlay(
    frame: &mut Frame,
    app: &App,
    title: &str,
    text: &str,
    alignment: Alignment,
) {
    let area = frame.area();

    // Use most of the terminal: 80% width, 85% height, centered
    let width = (area.width as f32 * 0.80) as u16;
    let height = (area.height as f32 * 0.85) as u16;
    let x = area.width.saturating_sub(width) / 2;
    let y = area.height.saturating_sub(height) / 2;
    let overlay_area = Rect::new(x, y, width.min(area.width), height.min(area.height));

    frame.render_widget(Clear, overlay_area);

    let total_lines = text.lines().count();
    let inner_height = overlay_area.height.saturating_sub(2) as usize; // minus border
    let scroll_max = total_lines.saturating_sub(inner_height).saturating_sub(1);
    let scroll = app.overlay_scroll.min(scroll_max);

    let paragraph = Paragraph::new(
        text.lines()
            .map(Line::from)
            .collect::<Vec<_>>(),
    )
    .block(
        Block::default()
            .borders(Borders::ALL)
            .title(format!(" {} ", title))
            .border_style(Style::default().fg(Color::Cyan)),
    )
    .wrap(Wrap { trim: true })
    .scroll((scroll as u16, 0))
    .alignment(alignment);

    frame.render_widget(paragraph, overlay_area);
}
