//! Radio settings inline editor triggered by Tab.

use ratatui::{
    layout::Rect,
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Clear, List, ListItem, Paragraph},
    Frame,
};

use crate::app::{App, InputMode, SettingsField};
use crate::app::tx_power_label;
use crate::radio::hopper::format_mhz;

use super::{text, theme};

/// Tab of the setting being edited (its value list is open under it).
const EDITING: Style = Style::new().fg(Color::Black).bg(Color::Green).add_modifier(Modifier::BOLD);

fn on_off(on: bool) -> String {
    if on { "ON" } else { "OFF" }.to_string()
}

/// (label, current value) of every tab, in [App::settings_fields] order: the selected radio's
/// settings (with several radios, the Radio tab first says which).
fn tabs(app: &App) -> Vec<(&'static str, String)> {
    let tuning = app.tuning();
    app.settings_fields()
        .iter()
        .map(|field| {
            let value = match field {
                SettingsField::Radio => app
                    .selected_radio()
                    .map(|i| format!("{} {}", i + 1, app.radios()[i].name))
                    .unwrap_or_default(),
                SettingsField::Freq => format!("{} MHz", format_mhz(tuning.frequency)),
                SettingsField::Hop => on_off(app.hop_enabled()),
                SettingsField::Lna => format!("{} dB", tuning.lna_gain),
                SettingsField::Vga | SettingsField::Gain => format!("{} dB", tuning.vga_gain),
                SettingsField::Amp => on_off(tuning.amp),
                SettingsField::TxPower => tx_power_label(app.tx_vga_gain, app.tx_amp),
            };
            (field.label(), value)
        })
        .collect()
}

/// Width of a tab: " Label value ".
fn tab_width(label: &str, value: &str) -> usize {
    text::width(label) + text::width(value) + 3
}

/// The first tab drawn, and the x offset (from the bar's inner left edge) and width of every tab
/// from there on: the bar scrolls when the terminal is too narrow for all of them, so the
/// selected tab is always in view.
fn tab_positions(widths: &[usize], selected: usize, room: usize) -> (usize, Vec<(usize, usize)>) {
    let selected = selected.min(widths.len().saturating_sub(1));
    let mut first = 0;
    while first < selected && widths[first..=selected].iter().sum::<usize>() > room {
        first += 1;
    }
    let mut x = 0;
    let positions = widths[first..]
        .iter()
        .map(|&w| {
            let p = (x, w);
            x += w;
            p
        })
        .collect();
    (first, positions)
}

/// The settings bar's area inside its border.
fn bar_inner(bar: Rect) -> Rect {
    Block::default().borders(Borders::ALL).inner(bar)
}

/// Render the settings selector tabs in the header area
pub fn render_settings_tabs(frame: &mut Frame, area: Rect, app: &App) {
    let tabs = tabs(app);
    let widths: Vec<usize> = tabs.iter().map(|(l, v)| tab_width(l, v)).collect();
    let (first, _) = tab_positions(&widths, app.settings_field_index, bar_inner(area).width as usize);

    let mut spans = Vec::new();
    for (i, (label, value)) in tabs.iter().enumerate().skip(first) {
        let is_selected = app.input_mode == InputMode::SettingsSelect
            && i == app.settings_field_index;
        let is_editing = app.input_mode == InputMode::SettingsEdit
            && i == app.settings_field_index;

        if is_editing || is_selected {
            let style = if is_editing { EDITING } else { theme::HIGHLIGHT };
            spans.push(Span::styled(format!(" {} {} ", label, value), style));
        } else {
            spans.push(Span::styled(format!(" {} ", label), Style::default().fg(Color::DarkGray)));
            spans.push(Span::styled(format!("{} ", value), theme::TEXT));
        }
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

/// Render the settings value dropdown when in SettingsEdit mode: it opens under the settings bar
/// (`bar`), lined up with the tab being edited.
pub fn render_settings_dropdown(frame: &mut Frame, app: &App, bar: Rect) {
    if app.input_mode != InputMode::SettingsEdit {
        return;
    }

    let area = frame.area();
    let fields = app.settings_fields();
    let field = fields[app.settings_field_index.min(fields.len() - 1)];

    // The app's rows: the presets, plus the current value first when a command set one that isn't
    // a preset ("433.42 MHz (current)"), so Enter keeps it.
    let values: Vec<String> = app.settings_value_rows();

    // Line up with the tab: the same positions the bar was drawn with.
    let tabs = tabs(app);
    let widths: Vec<usize> = tabs.iter().map(|(l, v)| tab_width(l, v)).collect();
    let inner = bar_inner(bar);
    let (first, positions) = tab_positions(&widths, app.settings_field_index, inner.width as usize);
    let (tab_x, tab_w) = positions
        .get(app.settings_field_index.saturating_sub(first))
        .copied()
        .unwrap_or((0, 0));

    // Wide enough for the tab, its values ("> " + value) and the title, up to the terminal.
    let longest = values.iter().map(|v| text::width(v)).max().unwrap_or(0);
    let menu_width = (longest + 4)
        .max(tab_w)
        .max(text::width(field.label()) + 4)
        .min(area.width as usize) as u16;
    let x = (inner.x + tab_x as u16).min(area.width.saturating_sub(menu_width));
    // Under the bar, so the tab being edited stays visible; as tall as the list, down to the
    // help bar at most (then it scrolls to keep the highlighted value in view).
    let y = bar.y + bar.height;
    let menu_height = (values.len() as u16 + 2).min(area.height.saturating_sub(y + 1));
    let visible = menu_height.saturating_sub(2) as usize;
    let start = text::centered_start(app.settings_value_index, visible, values.len());
    let menu_area = Rect::new(x, y, menu_width, menu_height);

    frame.render_widget(Clear, menu_area);

    let inner_width = menu_width.saturating_sub(2) as usize;
    let items: Vec<ListItem> = values
        .iter()
        .enumerate()
        .skip(start)
        .take(visible)
        .map(|(i, val)| {
            let selected = i == app.settings_value_index;
            let style = if selected { theme::HIGHLIGHT } else { theme::TEXT };
            let prefix = if selected { "> " } else { "  " };
            // The highlight spans the whole row.
            let pad = inner_width.saturating_sub(2 + text::width(val));
            ListItem::new(Line::from(Span::styled(
                format!("{}{}{}", prefix, val, " ".repeat(pad)),
                style,
            )))
        })
        .collect();

    let mut block = Block::default()
        .title(format!(" {} ", field.label()))
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Green));
    if let Some(hint) = text::more_hint(start, visible, values.len()) {
        block = block.title_bottom(Line::from(hint).right_aligned());
    }

    frame.render_widget(List::new(items).block(block), menu_area);
}

#[cfg(test)]
mod tests {
    use super::tab_positions;

    #[test]
    fn tabs_scroll_to_keep_the_selection_in_view() {
        // " Freq 433.92 MHz " " Hop OFF " " LNA 24 dB " " VGA 20 dB " " AMP OFF " " TX 47 dB + amp "
        let widths = [17, 9, 11, 11, 9, 16];
        // 80-column terminal: everything fits, positions add up.
        let (first, pos) = tab_positions(&widths, 5, 78);
        assert_eq!(first, 0);
        assert_eq!(pos[5], (57, 16));
        // 60 columns: the first tabs scroll away so the TX tab is in view.
        let (first, pos) = tab_positions(&widths, 5, 58);
        assert_eq!(first, 1);
        assert_eq!(pos[4], (40, 16));
        assert!(pos[4].0 + pos[4].1 <= 58);
        let (first, _) = tab_positions(&widths, 0, 58);
        assert_eq!(first, 0);
    }
}
