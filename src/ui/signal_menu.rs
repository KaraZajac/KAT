//! Signal action popup menu rendered as a centered overlay.

use ratatui::{
    layout::Rect,
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Clear, List, ListItem},
    Frame,
};

use crate::app::{App, SignalAction};

/// Render the signal action popup menu as a centered overlay
pub fn render_signal_menu(frame: &mut Frame, app: &App) {
    let area = frame.area();

    // Get selected capture info for the header
    let (capture_info, freq_info) = if let Some(idx) = app.selected_capture {
        if idx < app.captures.len() {
            let c = &app.captures[idx];
            (
                format!("#{:02} {} | 0x{}", c.id, c.protocol_name(), c.serial_hex()),
                format!("{} | {}", c.frequency_mhz(), c.modulation()),
            )
        } else {
            ("No capture".to_string(), String::new())
        }
    } else {
        ("No capture".to_string(), String::new())
    };

    // Menu dimensions - wider to show more info
    let menu_width = 38u16;
    let extra_lines = if freq_info.is_empty() { 0u16 } else { 2u16 };
    let menu_height = (SignalAction::ALL.len() as u16) + 4 + extra_lines;

    // Center the menu
    let x = area.width.saturating_sub(menu_width) / 2;
    let y = area.height.saturating_sub(menu_height) / 2;
    let menu_area = Rect::new(x, y, menu_width.min(area.width), menu_height.min(area.height));

    // Clear the area behind the popup
    frame.render_widget(Clear, menu_area);

    // Build list items
    let mut items: Vec<ListItem> = Vec::new();

    // Add signal info lines at the top if we have capture data
    if !freq_info.is_empty() {
        items.push(ListItem::new(Line::from(Span::styled(
            format!("  {}", freq_info),
            Style::default().fg(Color::DarkGray),
        ))));
        items.push(ListItem::new(Line::from(Span::raw(""))));
    }

    // Add action items
    for (i, action) in SignalAction::ALL.iter().enumerate() {
        let style = if i == app.signal_menu_index {
            Style::default()
                .fg(Color::Black)
                .bg(Color::Cyan)
                .add_modifier(Modifier::BOLD)
        } else {
            match action {
                SignalAction::Delete => Style::default().fg(Color::Red),
                SignalAction::ExportFob | SignalAction::ExportFlipper => {
                    Style::default().fg(Color::Green)
                }
                _ => Style::default().fg(Color::White),
            }
        };

        let prefix = if i == app.signal_menu_index {
            " > "
        } else {
            "   "
        };

        let is_tx_action = matches!(
            action,
            SignalAction::Replay | SignalAction::Lock | SignalAction::Unlock
                | SignalAction::Trunk | SignalAction::Panic
        );
        let suffix = if is_tx_action && !app.can_transmit() {
            " (no TX)"
        } else {
            ""
        };

        items.push(ListItem::new(Line::from(Span::styled(
            format!("{}{}{}", prefix, action.label(), suffix),
            style,
        ))));
    }

    let list = List::new(items).block(
        Block::default()
            .title(format!(" {} ", capture_info))
            .borders(Borders::ALL)
            .border_style(Style::default().fg(Color::Cyan)),
    );

    frame.render_widget(list, menu_area);
}
