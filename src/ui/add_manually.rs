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
use crate::radio::hopper::format_mhz;

use super::{text, theme};

/// Rows the picker list shows at most (as the app scrolls it); fewer on a short terminal.
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

/// Help-bar texts of these modes, the fullest first: the bar shows the first that fits.
pub fn help_texts(mode: InputMode) -> &'static [&'static str] {
    match mode {
        // PgUp/PgDn also move the selection.
        InputMode::NewRemote => &[
            "Type: Filter | Up/Down: Select | Enter: Create | Tab: Edit fields | Esc: Close",
            "Type: Filter | Enter: Create | Tab: Fields | Esc: Close",
        ],
        InputMode::NewRemoteEdit => &[
            "0-9 A-F: Edit | Tab/Up/Down: Field | Enter: Create | Esc: Back",
            "0-9 A-F: Edit | Tab: Field | Enter: Create | Esc: Back",
        ],
        InputMode::SendButton => &[
            "Up/Down: Button | 0-9 A-F: Other code | Enter: Send | Esc: Back",
            "Up/Down: Button | 0-9 A-F: Code | Enter: Send | Esc: Back",
        ],
        _ => &[],
    }
}

fn centered(width: u16, height: u16, area: Rect) -> Rect {
    let w = width.min(area.width);
    let h = height.min(area.height);
    Rect::new(area.x + (area.width - w) / 2, area.y + (area.height - h) / 2, w, h)
}

fn mhz(hz: u32) -> String {
    format!("{} MHz", format_mhz(hz))
}

fn field_text(field: Field, value: Option<u64>) -> String {
    match value {
        Some(v) => format!("{} 0x{v:X}", field.label()),
        None => format!("{} random", field.label()),
    }
}

fn render_picker(frame: &mut Frame, app: &App) {
    let Some(p) = app.new_remote_picker() else { return };
    // Filter, blank, the list, blank, three lines on the selected type, borders: 8 rows besides
    // the list, which gets what a short terminal leaves (the help bar stays visible).
    let height = (VISIBLE as u16 + 8).min(frame.area().height.saturating_sub(2));
    let visible = (height as usize).saturating_sub(8).max(1);
    let area = centered(92, height, frame.area());
    frame.render_widget(Clear, area);
    let inner = area.width.saturating_sub(2) as usize;
    let dim = Style::default().fg(Color::DarkGray);
    let mut lines = vec![
        Line::from(vec![
            Span::styled(" Filter: ", dim),
            Span::styled(p.filter.clone(), Style::default().fg(Color::Yellow)),
            Span::styled(theme::CURSOR, theme::TEXT),
            Span::styled(format!("   {} of {} types", p.visible().len(), TYPES.len()), dim),
        ]),
        Line::from(""),
    ];
    let rows = p.rows();
    let selected = p.selected_type();
    // The app scrolls for the full list height; a shorter list still keeps the selection in view.
    let selected_row = rows.iter().position(|r| selected.is_some_and(|i| *r == PickerRow::Type(i))).unwrap_or(0);
    let start = text::keep_in_view(p.scroll, selected_row, visible, rows.len());
    // Name and protocol columns share what the fixed ones (marker, frequency, preset, note) leave.
    let room = inner.saturating_sub(29);
    let name_width = room.saturating_sub(8).min(26);
    let protocol_width = (room - name_width).min(28);
    for row in rows.iter().skip(start).take(visible) {
        match *row {
            PickerRow::Heading(g) => lines.push(Line::from(Span::styled(
                format!(" ── {} ", g.label()),
                Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD),
            ))),
            PickerRow::Type(i) => {
                let ty = &TYPES[i];
                let available = p.availability.get(i).is_none_or(|a| a.is_ok());
                let s = add_manually::summary(ty);
                let row_text = format!(
                    "{} {} {} {:>10} {:<6}{}",
                    if Some(i) == selected { ">" } else { " " },
                    text::fit(ty.name, name_width),
                    text::fit(&s.protocol, protocol_width),
                    mhz(s.frequency),
                    s.preset,
                    match (ty.source, available) {
                        (_, false) => "  ✗ n/a",
                        (Source::Doctor, true) => "  doctor",
                        (Source::RogueMaster, true) => "  RM",
                        (Source::FoxFW, true) => "  Fox",
                        (Source::Both, true) => "",
                    }
                );
                let style = if Some(i) == selected {
                    theme::HIGHLIGHT
                } else if available {
                    theme::TEXT
                } else {
                    dim
                };
                lines.push(Line::from(Span::styled(row_text, style)));
            }
        }
    }
    while lines.len() < visible + 2 {
        lines.push(Line::from(""));
    }
    lines.push(Line::from(""));
    match selected {
        Some(i) => {
            let ty = &TYPES[i];
            let s = add_manually::summary(ty);
            let fields: Vec<String> = s.fields.iter().map(|&(f, v)| field_text(f, v)).collect();
            // The row above has the frequency and the preset: a narrow window keeps what it lacks.
            let mut about = format!(" {} · {} · {} · {}", s.protocol, mhz(s.frequency), s.preset, ty.menus());
            if text::width(&about) > inner {
                about = format!(" {} · {}", s.protocol, ty.menus());
            }
            lines.push(Line::styled(text::truncate_end(&about, inner), Style::default().fg(Color::Cyan)));
            match p.availability.get(i) {
                // Why not, in the two lines the fields and the keys have otherwise: a reason says
                // what to add and where, which one line cut away.
                Some(Err(e)) => lines.extend(
                    unavailable_lines(e, inner).into_iter().map(|l| Line::styled(l, Style::default().fg(Color::Red))),
                ),
                _ => {
                    let detail = if fields.is_empty() { "Fixed code drawn at random (Security+ 1.0 rule)".to_string() } else { fields.join(" · ") };
                    lines.push(Line::styled(text::truncate_end(&format!(" {detail}"), inner), theme::TEXT));
                    let hints = [" Enter: create with these defaults · Tab: edit the fields first", " Enter: create with these defaults · Tab: edit them"];
                    let hint = hints.iter().find(|h| text::width(h) <= inner).unwrap_or(&hints[1]);
                    lines.push(Line::styled(text::truncate_end(hint, inner), dim));
                }
            }
        }
        None => lines.push(Line::from(Span::styled(" No type matches the filter", Style::default().fg(Color::Red)))),
    }
    let mut block = Block::default()
        .title(" New remote (Add Manually) ")
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Magenta));
    // The types out of view: the headings among the rows are not counted.
    let types = |rows: &[PickerRow]| rows.iter().filter(|r| matches!(r, PickerRow::Type(_))).count();
    let (first, end) = (start.min(rows.len()), (start + visible).min(rows.len()));
    if let Some(hint) = text::more_of(types(&rows[..first]), types(&rows[end..])) {
        block = block.title_bottom(Line::from(hint).right_aligned());
    }
    frame.render_widget(Paragraph::new(lines).block(block), area);
}

/// The two lines under an unavailable type that say why KAT cannot create it, for a picker
/// `inner` columns wide inside its border (the second line is empty when one is enough).
fn unavailable_lines(reason: &str, inner: usize) -> Vec<String> {
    let mut lines = text::wrap_within(&format!("Unavailable: {reason}"), inner.saturating_sub(1), 2);
    lines.resize(2, String::new());
    lines.into_iter().map(|l| format!(" {l}")).collect()
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
        lines.push(Line::from(Span::styled(" No fields: the fixed code is drawn at random.", theme::TEXT)));
    }
    for (i, (field, text, width)) in f.fields.iter().enumerate() {
        let focused = i == f.focus;
        let value = format!("{text}{}", if focused { theme::CURSOR } else { "" });
        lines.push(Line::from(vec![
            Span::styled(format!(" {} {:<8}", if focused { ">" } else { " " }, field.label()), theme::TEXT),
            Span::styled(
                format!(" 0x{value:<w$} ", w = width * 2 + 1),
                if focused { theme::HIGHLIGHT } else { Style::default().fg(Color::Yellow) },
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
    let name = app
        .captures
        .iter()
        .find(|c| c.id == m.capture_id)
        .map(|c| format!("Send button — #{:02} {}", c.id, c.protocol_name()));
    // Up to 20 buttons, fewer on a short terminal; the list scrolls to keep the selection in view.
    let height = (m.buttons.len().min(20) as u16 + 6).min(frame.area().height.saturating_sub(2));
    // Borders, the blank line, "Other code" and the hint: 5 rows; the buttons get the rest.
    let visible = (height as usize).saturating_sub(5).max(1);
    let area = centered(52, height, frame.area());
    frame.render_widget(Clear, area);
    // A protocol name longer than the box is cut with "…" (the border would cut it bare).
    let room = usize::from(area.width).saturating_sub(4);
    let title = name.map_or_else(String::new, |n| format!(" {} ", text::truncate_end(&n, room)));
    let first = text::centered_start(m.selected, visible, m.buttons.len());
    let mut lines: Vec<Line> = m
        .buttons
        .iter()
        .enumerate()
        .skip(first)
        .take(visible)
        .map(|(i, (_, label))| {
            if i == m.selected && m.custom.is_empty() {
                Line::from(Span::styled(format!(" > {label}"), theme::HIGHLIGHT))
            } else {
                Line::from(Span::styled(format!("   {label}"), theme::TEXT))
            }
        })
        .collect();
    lines.push(Line::from(""));
    lines.push(Line::from(vec![
        Span::styled(" Other code: 0x", Style::default().fg(Color::DarkGray)),
        Span::styled(m.custom.clone(), Style::default().fg(Color::Yellow)),
        Span::styled(theme::CURSOR, theme::TEXT),
    ]));
    lines.push(Line::from(Span::styled(" Enter: send the next code with it · Esc: back", Style::default().fg(Color::DarkGray))));
    let mut block = Block::default().title(title).borders(Borders::ALL).border_style(Style::default().fg(Color::Cyan));
    if let Some(hint) = text::more_hint(first, visible, m.buttons.len()) {
        block = block.title_bottom(Line::from(hint).right_aligned());
    }
    frame.render_widget(Paragraph::new(lines).block(block), area);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Why a type cannot be created shows whole under it, in the narrowest terminal too: the
    /// reasons end with what to add and where.
    #[test]
    fn an_unavailable_types_reason_shows_whole() {
        let _keystore = crate::protocols::keeloq_common::KEYSTORE_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let inner = usize::from(crate::ui::layout::MIN_WIDTH) - 2;
        // The types KAT cannot create here (the O-Code ones always), and the longest of the other
        // reasons (they depend on the keys and tables in the config folder): a missing table, and
        // a missing key of the manufacturer with the longest name.
        let mut reasons: Vec<String> = TYPES.iter().filter_map(|t| add_manually::availability(t).err()).collect();
        assert!(reasons.iter().any(|r| r.contains("O-Code")), "{reasons:?}");
        reasons.extend([
            "needs the Alutech AT-4N rainbow table (~/.config/KAT/alutech_at_4n.bin)".to_string(),
            "no usable Clemsa_Mutancode key in the keystore (add one to ~/.config/KAT/keeloq_mfcodes_user)".to_string(),
        ]);
        for reason in reasons {
            let lines = unavailable_lines(&reason, inner);
            assert_eq!(lines.len(), 2, "{reason}");
            assert!(lines.iter().all(|l| text::width(l) <= inner), "{lines:?}");
            let shown = lines.iter().map(|l| l.trim()).filter(|l| !l.is_empty()).collect::<Vec<_>>().join(" ");
            assert_eq!(shown, format!("Unavailable: {reason}"));
        }
        // A short reason leaves the second line empty: the picker keeps its height.
        assert_eq!(unavailable_lines("KAT has no X protocol", 58), [" Unavailable: KAT has no X protocol", " "]);
    }
}
