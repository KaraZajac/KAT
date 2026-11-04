//! Overlays of `:new` ("Add Manually" type picker and field form) and the signal menu's
//! "Send button…" list (app/add_manually.rs).

use ratatui::{
    layout::Rect,
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Clear, Paragraph},
    Frame,
};

use crate::app::{App, InputMode, PickerRow};
use crate::protocols::add_manually::{self, Field, Source, TYPES};

/// Rows the picker list shows (as the app scrolls it).
const VISIBLE: usize = crate::app::PICKER_VISIBLE_ROWS;

/// Draw the overlay of the current mode, if it is one of these.
pub fn render(frame: &mut Frame, app: &App) {
    match app.input_mode {
        InputMode::NewRemote => render_picker(frame, app),
        InputMode::NewRemoteEdit => render_form(frame, app),
        InputMode::SendButton => render_send_button(frame, app),
        _ => {}
    }
}

/// Mode label for the command line.
pub fn mode_label(mode: InputMode) -> (&'static str, Style) {
    match mode {
        InputMode::NewRemote | InputMode::NewRemoteEdit => ("NEW", Style::default().fg(Color::Magenta)),
        InputMode::SendButton => ("BUTTON", Style::default().fg(Color::Cyan)),
        _ => ("", Style::default()),
    }
}

/// Help-bar text.
pub fn help_text(mode: InputMode) -> &'static str {
    match mode {
        InputMode::NewRemote => "Type: filter | Up/Down/PgUp/PgDn: select | Enter: create | Tab: edit fields first | Esc: close",
        InputMode::NewRemoteEdit => "0-9 A-F: edit | Tab/Up/Down: field | Enter: create | Esc: back",
        InputMode::SendButton => "Up/Down: button | 0-9 A-F: other code | Enter: send | Esc: back",
        _ => "",
    }
}

fn centered(width: u16, height: u16, area: Rect) -> Rect {
    let w = width.min(area.width);
    let h = height.min(area.height);
    Rect::new(area.x + (area.width - w) / 2, area.y + (area.height - h) / 2, w, h)
}

fn mhz(hz: u32) -> String {
    format!("{:.2} MHz", f64::from(hz) / 1e6)
}

fn field_text(field: Field, value: Option<u64>) -> String {
    match value {
        Some(v) => format!("{} 0x{v:X}", field.label()),
        None => format!("{} random", field.label()),
    }
}

fn render_picker(frame: &mut Frame, app: &App) {
    let Some(p) = app.new_remote_picker() else { return };
    let area = centered(92, VISIBLE as u16 + 10, frame.area());
    frame.render_widget(Clear, area);
    let dim = Style::default().fg(Color::DarkGray);
    let mut lines = vec![
        Line::from(vec![
            Span::styled(" Filter: ", dim),
            Span::styled(format!("{}█", p.filter), Style::default().fg(Color::Yellow)),
            Span::styled(format!("   {} of {} types", p.visible().len(), TYPES.len()), dim),
        ]),
        Line::from(""),
    ];
    let rows = p.rows();
    let selected = p.selected_type();
    for row in rows.iter().skip(p.scroll).take(VISIBLE) {
        match *row {
            PickerRow::Heading(g) => lines.push(Line::from(Span::styled(
                format!(" ── {} ", g.label()),
                Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD),
            ))),
            PickerRow::Type(i) => {
                let ty = &TYPES[i];
                let available = p.availability.get(i).is_none_or(|a| a.is_ok());
                let s = add_manually::summary(ty);
                let text = format!(
                    "{} {:<26} {:<28} {:>10} {:<6}{}",
                    if Some(i) == selected { ">" } else { " " },
                    ty.name,
                    s.protocol,
                    mhz(s.frequency),
                    s.preset,
                    match (ty.source, available) {
                        (_, false) => "  ✗ n/a",
                        (Source::Doctor, true) => "  doctor",
                        _ => "",
                    }
                );
                let style = if Some(i) == selected {
                    Style::default().fg(Color::Black).bg(Color::Cyan).add_modifier(Modifier::BOLD)
                } else if available {
                    Style::default().fg(Color::White)
                } else {
                    dim
                };
                lines.push(Line::from(Span::styled(text, style)));
            }
        }
    }
    while lines.len() < VISIBLE + 2 {
        lines.push(Line::from(""));
    }
    lines.push(Line::from(""));
    match selected {
        Some(i) => {
            let ty = &TYPES[i];
            let s = add_manually::summary(ty);
            let fields: Vec<String> = s.fields.iter().map(|&(f, v)| field_text(f, v)).collect();
            let from = match ty.source {
                Source::Both => "Flipper-ARF + doctor-firmware",
                Source::Doctor => "doctor-firmware",
            };
            lines.push(Line::from(vec![
                Span::styled(" ", dim),
                Span::styled(format!("{} · {} · {} · {}", s.protocol, mhz(s.frequency), s.preset, from), Style::default().fg(Color::Cyan)),
            ]));
            let detail = if fields.is_empty() { "Fixed code drawn at random (Security+ 1.0 rule)".to_string() } else { fields.join(" · ") };
            lines.push(Line::from(Span::styled(format!(" {detail}"), Style::default().fg(Color::White))));
            match p.availability.get(i) {
                Some(Err(e)) => lines.push(Line::from(Span::styled(format!(" Unavailable: {e}"), Style::default().fg(Color::Red)))),
                _ => lines.push(Line::from(Span::styled(" Enter: create with these defaults · Tab: edit the fields first", dim))),
            }
        }
        None => lines.push(Line::from(Span::styled(" No type matches the filter", Style::default().fg(Color::Red)))),
    }
    let block = Block::default()
        .title(" New remote (Add Manually) ")
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Magenta));
    frame.render_widget(Paragraph::new(lines).block(block), area);
}

fn render_form(frame: &mut Frame, app: &App) {
    let Some(f) = app.new_remote_form() else { return };
    let ty = &TYPES[f.type_index];
    let s = add_manually::summary(ty);
    let height = f.fields.len() as u16 + 9;
    let area = centered(64, height, frame.area());
    frame.render_widget(Clear, area);
    let dim = Style::default().fg(Color::DarkGray);
    let mut lines = vec![
        Line::from(Span::styled(format!(" {} · {} · {}", s.protocol, mhz(s.frequency), s.preset), Style::default().fg(Color::Cyan))),
        Line::from(""),
    ];
    if f.fields.is_empty() {
        lines.push(Line::from(Span::styled(" No fields: the fixed code is drawn at random.", Style::default().fg(Color::White))));
    }
    for (i, (field, text, width)) in f.fields.iter().enumerate() {
        let focused = i == f.focus;
        let value = format!("{text}{}", if focused { "█" } else { "" });
        lines.push(Line::from(vec![
            Span::styled(format!(" {} {:<8}", if focused { ">" } else { " " }, field.label()), Style::default().fg(Color::White)),
            Span::styled(
                format!(" 0x{value:<w$} ", w = width * 2 + 1),
                if focused { Style::default().fg(Color::Black).bg(Color::Yellow) } else { Style::default().fg(Color::Yellow) },
            ),
            Span::styled(format!(" {} hex digits", width * 2), dim),
        ]));
    }
    lines.push(Line::from(""));
    if let Some(e) = &f.error {
        lines.push(Line::from(Span::styled(format!(" {e}"), Style::default().fg(Color::Red))));
    }
    lines.push(Line::from(Span::styled(" Enter: create · Tab/Up/Down: field · Esc: back", dim)));
    let block = Block::default()
        .title(format!(" New remote: {} ", ty.name))
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Magenta));
    frame.render_widget(Paragraph::new(lines).block(block), area);
}

fn render_send_button(frame: &mut Frame, app: &App) {
    let Some(m) = app.send_button_menu() else { return };
    let title = app
        .captures
        .iter()
        .find(|c| c.id == m.capture_id)
        .map_or_else(String::new, |c| format!(" Send button — #{:02} {} ", c.id, c.protocol_name()));
    let height = m.buttons.len().min(20) as u16 + 6;
    let area = centered(52, height, frame.area());
    frame.render_widget(Clear, area);
    let first = m.selected.saturating_sub(19);
    let mut lines: Vec<Line> = m
        .buttons
        .iter()
        .enumerate()
        .skip(first)
        .take(20)
        .map(|(i, (_, label))| {
            if i == m.selected && m.custom.is_empty() {
                Line::from(Span::styled(
                    format!(" > {label}"),
                    Style::default().fg(Color::Black).bg(Color::Cyan).add_modifier(Modifier::BOLD),
                ))
            } else {
                Line::from(Span::styled(format!("   {label}"), Style::default().fg(Color::White)))
            }
        })
        .collect();
    lines.push(Line::from(""));
    lines.push(Line::from(vec![
        Span::styled(" Other code: 0x", Style::default().fg(Color::DarkGray)),
        Span::styled(format!("{}█", m.custom), Style::default().fg(Color::Yellow)),
    ]));
    lines.push(Line::from(Span::styled(" Enter: send the next code with it · Esc: back", Style::default().fg(Color::DarkGray))));
    let block = Block::default().title(title).borders(Borders::ALL).border_style(Style::default().fg(Color::Cyan));
    frame.render_widget(Paragraph::new(lines).block(block), area);
}
