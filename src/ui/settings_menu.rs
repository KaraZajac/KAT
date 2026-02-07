//! Radio settings inline editor triggered by Tab.

use ratatui::{
    layout::Rect,
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Clear, List, ListItem, Paragraph},
    Frame,
};

use crate::app::{App, InputMode, SettingsField, PRESET_FREQUENCIES, LNA_STEPS, VGA_STEPS};

/// Render the settings selector tabs in the header area
pub fn render_settings_tabs(frame: &mut Frame, area: Rect, app: &App) {
    let mut spans = Vec::new();
    spans.push(Span::styled(" Settings: ", Style::default().fg(Color::DarkGray)));

    for (i, field) in SettingsField::ALL.iter().enumerate() {
        let is_selected = app.input_mode == InputMode::SettingsSelect
            && i == app.settings_field_index;
        let is_editing = app.input_mode == InputMode::SettingsEdit
            && i == app.settings_field_index;

        let style = if is_editing {
            Style::default().fg(Color::Black).bg(Color::Green).add_modifier(Modifier::BOLD)
        } else if is_selected {
            Style::default().fg(Color::Black).bg(Color::Cyan).add_modifier(Modifier::BOLD)
        } else {
            Style::default().fg(Color::White)
        };

        let value = match field {
            SettingsField::Freq => format!("{:.2}MHz", app.frequency as f64 / 1_000_000.0),
            SettingsField::Lna => format!("{}dB", app.lna_gain),
            SettingsField::Vga => format!("{}dB", app.vga_gain),
            SettingsField::Amp => if app.amp_enabled { "ON".to_string() } else { "OFF".to_string() },
        };

        spans.push(Span::styled(
            format!(" [{}:{}] ", field.label(), value),
            style,
        ));
    }

    let line = Line::from(spans);
    let widget = Paragraph::new(line).block(
        Block::default()
            .borders(Borders::ALL)
            .border_style(Style::default().fg(Color::Cyan))
            .title(" Radio Settings (Tab) "),
    );

    frame.render_widget(widget, area);
}

/// Render the settings value dropdown when in SettingsEdit mode
pub fn render_settings_dropdown(frame: &mut Frame, app: &App) {
    if app.input_mode != InputMode::SettingsEdit {
        return;
    }

    let area = frame.area();
    let field = SettingsField::ALL[app.settings_field_index];

    let values: Vec<String> = match field {
        SettingsField::Freq => PRESET_FREQUENCIES
            .iter()
            .map(|(_, label)| label.to_string())
            .collect(),
        SettingsField::Lna => LNA_STEPS.iter().map(|g| format!("{} dB", g)).collect(),
        SettingsField::Vga => VGA_STEPS.iter().map(|g| format!("{} dB", g)).collect(),
        SettingsField::Amp => vec!["ON".to_string(), "OFF".to_string()],
    };

    let menu_width = 22u16;
    let menu_height = (values.len() as u16) + 2; // items + borders

    // Position: below the header, near the field
    let x_offset = 12 + (app.settings_field_index as u16) * 16;
    let x = x_offset.min(area.width.saturating_sub(menu_width));
    let y = 3u16; // Below header

    let menu_area = Rect::new(
        x,
        y,
        menu_width.min(area.width.saturating_sub(x)),
        menu_height.min(area.height.saturating_sub(y)),
    );

    frame.render_widget(Clear, menu_area);

    let items: Vec<ListItem> = values
        .iter()
        .enumerate()
        .map(|(i, val)| {
            let style = if i == app.settings_value_index {
                Style::default()
                    .fg(Color::Black)
                    .bg(Color::Green)
                    .add_modifier(Modifier::BOLD)
            } else {
                Style::default().fg(Color::White)
            };

            let prefix = if i == app.settings_value_index {
                "> "
            } else {
                "  "
            };

            ListItem::new(Line::from(Span::styled(
                format!("{}{}", prefix, val),
                style,
            )))
        })
        .collect();

    let list = List::new(items).block(
        Block::default()
            .title(format!(" {} ", field.label()))
            .borders(Borders::ALL)
            .border_style(Style::default().fg(Color::Green)),
    );

    frame.render_widget(list, menu_area);
}
