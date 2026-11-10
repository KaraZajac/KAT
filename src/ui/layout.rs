//! Main UI layout.

use ratatui::{
    layout::{Alignment, Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Clear, Paragraph},
    Frame,
};

use crate::app::{App, InputMode, Radio, RadioState, LICENSE_TEXT};
use crate::capture::Capture;
use crate::radio::devices::RadioKind;
use crate::radio::hopper::format_mhz;

use super::captures_list::{render_captures_list, DETAIL_FLOOR};
use super::command::render_command_line;
use super::settings_menu::{render_settings_dropdown, render_settings_tabs};
use super::signal_menu::render_signal_menu;
use super::status_bar::{self, render_status_bar};
use super::text;
use super::text_overlay::render_text_overlay;
use super::theme;

use crate::app::InputMode as IM;

const VERSION: &str = env!("CARGO_PKG_VERSION");

/// RSSI bar width (right side)
const RSSI_BAR_WIDTH: u16 = 5;

/// Smallest terminal the screen is drawn in: header, captures table with a few rows, status bar,
/// help bar and the command line (or settings bar) all fit. Smaller, a notice says so instead.
pub(super) const MIN_WIDTH: u16 = 60;
pub(super) const MIN_HEIGHT: u16 = 16;
/// Rows the middle row (captures table and RX bar) keeps whatever else is open.
const MIDDLE_MIN_HEIGHT: u16 = 6;

/// Draw the entire UI
pub fn draw_ui(frame: &mut Frame, app: &App) {
    let main_area = frame.area();
    if main_area.width < MIN_WIDTH || main_area.height < MIN_HEIGHT {
        render_too_small(frame, main_area);
        return;
    }

    let show_settings = matches!(app.input_mode, IM::SettingsSelect | IM::SettingsEdit);
    let show_command = app.input_mode == IM::Command;
    let header = header_height(app, main_area.height);
    let other_rows = header + 1 + if show_settings { 3 } else { 0 } + if show_command { 3 } else { 0 };
    let status = status_height(status_bar::lines_needed(app, main_area.width), main_area.height, other_rows);
    let rows = main_rows(main_area, header, status, show_settings, show_command);

    let mut idx = 0;
    render_header(frame, rows[idx], app);
    idx += 1;

    let mut settings_bar = None;
    if show_settings {
        render_settings_tabs(frame, rows[idx], app);
        settings_bar = Some(rows[idx]);
        idx += 1;
    }

    // Only the middle row is split: captures (left) | RX bar (right)
    let middle_row = rows[idx];
    let h_chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Min(0), Constraint::Length(rssi_bar_width(app))])
        .split(middle_row);
    let captures_area = h_chunks[0];
    let rssi_bar_rect = h_chunks[1];
    idx += 1;

    render_captures_list(frame, captures_area, app);
    render_rssi_bar(frame, rssi_bar_rect, app);

    render_status_bar(frame, rows[idx], app);
    idx += 1;

    if show_command {
        render_command_line(frame, rows[idx], app);
        idx += 1;
    }

    render_help_bar(frame, rows[idx], app);

    // Overlay widgets (rendered on top of everything else)
    if app.input_mode == InputMode::SignalMenu {
        render_signal_menu(frame, app);
    }

    if app.input_mode == InputMode::SettingsEdit {
        if let Some(bar) = settings_bar {
            render_settings_dropdown(frame, app, bar);
        }
    }

    if app.input_mode == InputMode::HackRfNotDetected {
        render_hackrf_not_detected(frame, app);
    }

    if app.input_mode == InputMode::StartupImport {
        render_startup_import_prompt(frame, app);
    }

    if matches!(
        app.input_mode,
        InputMode::ExportFilename
            | InputMode::FobMetaYear
            | InputMode::FobMetaMake
            | InputMode::FobMetaModel
            | InputMode::FobMetaRegion
            | InputMode::FobMetaCommand
            | InputMode::FobMetaNotes
    ) {
        render_export_form(frame, app);
    }

    if matches!(
        app.input_mode,
        InputMode::CaptureMetaYear
            | InputMode::CaptureMetaMake
            | InputMode::CaptureMetaModel
            | InputMode::CaptureMetaRegion
            | InputMode::CaptureMetaCommand
    ) {
        render_capture_meta_form(frame, app);
    }

    if app.input_mode == InputMode::LoadFileBrowser {
        render_load_file_browser(frame, app);
    }

    // The overlays return their scroll limit for this frame size; the keys stop there.
    if app.input_mode == InputMode::License {
        let max = render_text_overlay(frame, app, "License", LICENSE_TEXT, Alignment::Left);
        app.overlay_scroll_max.set(max);
    }
    if app.input_mode == InputMode::Credits {
        let max = render_text_overlay(
            frame,
            app,
            "Credits",
            super::text_overlay::CREDITS_TEXT,
            Alignment::Center,
        );
        app.overlay_scroll_max.set(max);
    }
    if app.input_mode == InputMode::Help {
        let max = super::text_overlay::render_preformatted_overlay(
            frame,
            app,
            "Help",
            super::text_overlay::HELP_TEXT,
        );
        app.overlay_scroll_max.set(max);
    }
    if app.input_mode == InputMode::Radios {
        let max = super::text_overlay::render_preformatted_overlay(frame, app, "Radios", &app.radios_overview());
        app.overlay_scroll_max.set(max);
    }

    // Session restore prompt, :clear confirmation, Analyze overlay
    super::app_prompts::render_app_overlays(frame, app);
    // :new picker / field form, "Send button…" list
    super::add_manually::render(frame, app);
}

/// Full-width rows: header, settings (optional), then middle row split into [captures | RX bar],
/// status, command (optional), help. The fixed rows always get their height (a Min constraint
/// would outrank them and squeeze them away on a normal-size terminal); the middle row takes the
/// rest, 6+ rows at the minimum size ([MIDDLE_MIN_HEIGHT]).
fn main_rows(area: Rect, header: u16, status: u16, show_settings: bool, show_command: bool) -> std::rc::Rc<[Rect]> {
    let mut v_constraints = vec![
        Constraint::Length(header), // Header (full width)
        Constraint::Fill(1),    // Middle: captures table + detail panel (signal + vulnerability)
        Constraint::Length(status), // Status bar (full width), [status_height]
        Constraint::Length(1),  // Help bar (full width)
    ];
    if show_settings {
        v_constraints.insert(1, Constraint::Length(3)); // Settings tabs (full width)
    }
    if show_command {
        v_constraints.insert(v_constraints.len() - 1, Constraint::Length(3)); // Command (full width)
    }

    Layout::default()
        .direction(Direction::Vertical)
        .constraints(v_constraints)
        .split(area)
}

/// Rows of the status bar: its border and the `lines` its message takes, as far as the middle
/// row can spare them. `other_rows` are the rows of the header, the help bar and the settings
/// bar or command line when open. The middle row keeps the height its detail panel shows from
/// when it has that (the panel would go and come back with the message), and
/// [MIDDLE_MIN_HEIGHT] rows in any case; a message longer than its lines is cut in the middle.
fn status_height(lines: u16, area_height: u16, other_rows: u16) -> u16 {
    let middle = area_height.saturating_sub(other_rows + 3);
    let floor = if middle >= DETAIL_FLOOR { DETAIL_FLOOR } else { MIDDLE_MIN_HEIGHT };
    2 + lines.clamp(1, 1 + middle.saturating_sub(floor))
}

/// Shown instead of the screen when the terminal is smaller than [MIN_WIDTH]×[MIN_HEIGHT].
fn render_too_small(frame: &mut Frame, area: Rect) {
    let message = format!(
        "Terminal too small: {}×{} (need {}×{})",
        area.width, area.height, MIN_WIDTH, MIN_HEIGHT
    );
    let lines: Vec<Line> = text::wrap(&message, area.width as usize).into_iter().map(Line::from).collect();
    let height = (lines.len() as u16).min(area.height);
    let rect = Rect::new(area.x, area.y + (area.height - height) / 2, area.width, height);
    frame.render_widget(
        Paragraph::new(lines).alignment(Alignment::Center).style(Style::default().fg(Color::Yellow)),
        rect,
    );
}

/// Header rows: one line in a border, or with several radios a line for each when the terminal
/// is tall enough for them (else they share one line, [radios_compact_line]).
fn header_height(app: &App, height: u16) -> u16 {
    let n = app.radios().len() as u16;
    if n > 1 && height >= MIN_HEIGHT + n - 1 {
        n + 2
    } else {
        3
    }
}

/// The RX bar's width: one meter, or with several radios one column each with a space between
/// (2n − 1 inside the borders).
fn rssi_bar_width(app: &App) -> u16 {
    meters_width(app.radios().len())
}

fn meters_width(radios: usize) -> u16 {
    let n = radios as u16;
    if n > 1 {
        (2 * n + 1).max(RSSI_BAR_WIDTH)
    } else {
        RSSI_BAR_WIDTH
    }
}

/// A radio's meter: (fill 0..=1, style): the live level while receiving, full red while
/// transmitting, else empty and dim (the last level read is stale).
fn meter(state: RadioState, rssi: f32) -> (f32, Style) {
    match state {
        RadioState::Transmitting => (1.0, Style::default().fg(Color::Red)),
        RadioState::Receiving => ((rssi / 0.6).clamp(0.0, 1.0), Style::default().fg(Color::Green)),
        RadioState::Idle | RadioState::Disconnected => (0.0, Style::default().fg(Color::DarkGray)),
    }
}

/// Render the RSSI bar on the right (vertical bar, bottom = strong): the selected radio's meter,
/// or with several radios a column for each, numbered at the bottom.
fn render_rssi_bar(frame: &mut Frame, area: Rect, app: &App) {
    let dim = Style::default().fg(Color::DarkGray);
    let red = Style::default().fg(Color::Red);
    let state = app.radio_state();
    let any = |s: RadioState| app.radios().iter().any(|r| r.state == s);
    let (title, border_style) = if any(RadioState::Transmitting) {
        (" TX ", red)
    } else if any(RadioState::Receiving) || state == RadioState::Receiving {
        (" RX ", Style::default())
    } else {
        (" RX ", dim)
    };

    let block = Block::default()
        .borders(Borders::ALL)
        .border_style(border_style)
        .title(Span::styled(title, border_style));
    let inner = block.inner(area);
    frame.render_widget(block, area);

    if inner.width == 0 || inner.height == 0 {
        return;
    }

    if !app.several_radios() {
        let rssi = app.selected_radio().map_or(0.0, |i| app.radios()[i].rssi);
        let (fill_ratio, filled_style) = meter(state, rssi);
        let filled_rows = (inner.height as f32 * fill_ratio).round() as u16;
        let lines: Vec<Line> = (0..inner.height)
            .map(|r| {
                if r >= inner.height.saturating_sub(filled_rows) {
                    Line::styled("█".repeat(inner.width as usize), filled_style)
                } else {
                    Line::raw(" ".repeat(inner.width as usize))
                }
            })
            .collect();
        frame.render_widget(Paragraph::new(lines), inner);
        return;
    }

    // One column per radio with a space between ("█ █ █"), its number under it (the selected
    // one highlighted).
    let bars = inner.height.saturating_sub(1);
    let buf = frame.buffer_mut();
    for (i, r) in app.radios().iter().enumerate() {
        let x = inner.x + 2 * i as u16;
        if x >= inner.x + inner.width {
            break;
        }
        let (ratio, style) = meter(r.state, r.rssi);
        let filled = (bars as f32 * ratio).round() as u16;
        for row in bars.saturating_sub(filled)..bars {
            buf[(x, inner.y + row)].set_symbol("█").set_style(style);
        }
        let label = Style::default().fg(if r.state == RadioState::Idle { Color::DarkGray } else { style.fg.unwrap_or(Color::White) });
        let label = if app.selected_radio() == Some(i) { theme::HIGHLIGHT } else { label };
        let n = if i < 9 { char::from(b'1' + i as u8).to_string() } else { "+".to_string() };
        buf[(x, inner.y + bars)].set_symbol(&n).set_style(label);
    }
}

/// State symbol and its colour.
fn state_style(state: RadioState) -> (&'static str, Style) {
    match state {
        RadioState::Disconnected => ("○", Style::default().fg(Color::Red)),
        RadioState::Idle => ("○", Style::default().fg(Color::Yellow)),
        RadioState::Receiving => ("●", Style::default().fg(Color::Green)),
        RadioState::Transmitting => ("●", Style::default().fg(Color::Red)),
    }
}

/// " ● REC 1.2 MB" while `radio` records (with dropped buffers, if any).
fn rec_text(radio: Option<&Radio>) -> String {
    radio
        .and_then(|r| r.rx_header().recording)
        .map(|r| {
            let dropped = if r.dropped_buffers > 0 { format!(" ({} dropped)", r.dropped_buffers) } else { String::new() };
            format!(" ● REC {:.1} MB{dropped}", r.bytes as f64 / (1u64 << 20) as f64)
        })
        .unwrap_or_default()
}

/// `radio`'s frequency as the header shows it: "433.92 MHz", or "HOP 315.00 MHz (2/6)"; without
/// a radio, `offline`.
fn freq_text(radio: Option<&Radio>, offline: u32) -> String {
    match radio.and_then(|r| r.rx_header().hop) {
        Some((f, n, of)) => format!("HOP {} MHz ({}/{})", format_mhz(f), n + 1, of),
        None => format!("{} MHz", format_mhz(radio.map_or(offline, |r| r.tuning.frequency))),
    }
}

/// Render the header with title and radio status. When the terminal is narrow the radio state,
/// the recording indicator and the frequency stay longest: the title shortens first, then the
/// gains and the device name are left out. With several radios, [render_radios_header].
fn render_header(frame: &mut Frame, area: Rect, app: &App) {
    if app.several_radios() {
        render_radios_header(frame, area, app);
        return;
    }
    let radio_state = app.radio_state();
    let (status_symbol, status_style) = state_style(radio_state);
    let selected = app.selected_radio();
    let radio = selected.map(|i| &app.radios()[i]);
    let state = format!("{} {}", status_symbol, radio_state);
    // Recording indicator (raw IQ being written to disk), right after the state
    let rec = rec_text(radio);
    let device = app.radio_device_name().unwrap_or("No device");
    let freq = freq_text(radio, app.frequency());
    let gains = match selected {
        Some(i) => app.radio_settings_label(i),
        None => {
            let t = app.tuning();
            format!("LNA:{} VGA:{} AMP:{}", t.lna_gain, t.vga_gain, if t.amp { "ON" } else { "OFF" })
        }
    };

    let full_title = format!("Keyfob Analysis Toolkit v{}", VERSION);
    let short_title = format!("KAT v{}", VERSION);
    // (title, device name shown, gains shown), richest first: the first that fits is drawn.
    let options = [
        (full_title.as_str(), true, true),
        (short_title.as_str(), true, true),
        (short_title.as_str(), true, false),
        (short_title.as_str(), false, false),
        ("", false, false),
    ];
    let info = |device_shown: bool, gains_shown: bool| {
        let mut parts = Vec::new();
        if device_shown {
            parts.push(device);
        }
        parts.push(&freq);
        if gains_shown {
            parts.push(&gains);
        }
        format!(" | {}", parts.join(" | "))
    };
    let inner = area.width.saturating_sub(2) as usize;
    let fixed = text::width(&state) + text::width(&rec) + 2; // + a space at each end
    let (title, info) = options
        .iter()
        .map(|&(title, d, g)| (title, info(d, g)))
        .find(|(title, info)| fixed + text::width(title) + 1 + text::width(info) <= inner)
        .unwrap_or_else(|| ("", info(false, false)));

    let used = fixed + text::width(title) + text::width(&info);
    let padding = inner.saturating_sub(used);
    let header_line = Line::from(vec![
        Span::styled(format!(" {}", title), Style::default().add_modifier(Modifier::BOLD)),
        Span::raw(" ".repeat(padding)),
        Span::styled(state, status_style),
        Span::styled(rec, Style::default().fg(Color::Red).add_modifier(Modifier::BOLD)),
        Span::styled(format!("{info} "), status_style),
    ]);

    let header = Paragraph::new(header_line).block(Block::default().borders(Borders::ALL));
    frame.render_widget(header, area);
}

/// One radio's columns in the several-radios header.
struct RadioColumns {
    /// "▸1" (selected) or " 2"
    mark: String,
    state: String,
    state_style: Style,
    rec: String,
    /// Its name, " TX" after the radio that transmits.
    name: String,
    freq: String,
    settings: String,
}

/// The columns of every radio: `selected` is marked, `tx` transmits.
fn radio_columns(radios: &[Radio], selected: Option<usize>, tx: Option<usize>, flipper_preset: Option<&str>) -> Vec<RadioColumns> {
    radios
        .iter()
        .enumerate()
        .map(|(i, r)| {
            let (symbol, state_style) = state_style(r.state);
            RadioColumns {
                mark: format!("{}{}", if selected == Some(i) { '▸' } else { ' ' }, i + 1),
                state: format!("{symbol} {}", r.state),
                state_style,
                rec: rec_text(Some(r)),
                name: if tx == Some(i) { format!("{} TX", r.name) } else { r.name.clone() },
                freq: freq_text(Some(r), r.tuning.frequency),
                settings: r.settings_label(flipper_preset),
            }
        })
        .collect()
}

/// The header with several radios: the title, and a line for each radio (the selected one marked
/// ▸, the one that transmits "TX"), its columns lined up and the block right-aligned. A narrow
/// terminal leaves out the title, then the settings, then the names; a short one gets one line
/// for all radios ([radios_compact_line]).
fn render_radios_header(frame: &mut Frame, area: Rect, app: &App) {
    let inner = area.width.saturating_sub(2) as usize;
    let preset = app.storage.config.flipper_preset.as_deref();
    let cols = radio_columns(app.radios(), app.selected_radio(), app.tx_radio(), preset);
    let lines = if area.height < cols.len() as u16 + 2 {
        vec![radios_compact_line(&cols, inner)]
    } else {
        radios_header_lines(&cols, inner)
    };
    frame.render_widget(Paragraph::new(lines).block(Block::default().borders(Borders::ALL)), area);
}

/// The several-radios header's lines for `inner` columns (see [render_radios_header]).
fn radios_header_lines(cols: &[RadioColumns], inner: usize) -> Vec<Line<'static>> {
    let w = |f: fn(&RadioColumns) -> &str| cols.iter().map(|c| text::width(f(c))).max().unwrap_or(0);
    let (state_w, rec_w, name_w, freq_w, settings_w) =
        (w(|c| &c.state), w(|c| &c.rec), w(|c| &c.name), w(|c| &c.freq), w(|c| &c.settings));
    // " ▸1 ● RX  HackRF TX  433.92 MHz  LNA:24 VGA:20 AMP:OFF "
    let block_width = |names: bool, settings: bool| {
        3 + 1 + state_w + rec_w + if names { 2 + name_w } else { 0 } + 2 + freq_w + if settings { 2 + settings_w } else { 0 } + 1
    };
    let full_title = format!("Keyfob Analysis Toolkit v{}", VERSION);
    let short_title = format!("KAT v{}", VERSION);
    let options = [
        (full_title.as_str(), true, true),
        (short_title.as_str(), true, true),
        (short_title.as_str(), true, false),
        ("", true, false),
        ("", false, false),
    ];
    let (title, names, settings) = options
        .iter()
        .copied()
        .find(|&(title, n, s)| text::width(title) + 1 + block_width(n, s) <= inner)
        .unwrap_or(("", false, false));
    let block = block_width(names, settings);
    cols.iter()
        .enumerate()
        .map(|(row, c)| {
            let lead = if row == 0 { format!(" {title}") } else { String::new() };
            let pad = inner.saturating_sub(text::width(&lead) + block.min(inner));
            let mut spans = vec![
                Span::styled(lead, Style::default().add_modifier(Modifier::BOLD)),
                Span::raw(" ".repeat(pad)),
                Span::styled(format!(" {} ", c.mark), if c.mark.starts_with('▸') { theme::HIGHLIGHT } else { theme::TEXT }),
                Span::styled(format!("{:<state_w$}", c.state), c.state_style),
                Span::styled(format!("{:<rec_w$}", c.rec), Style::default().fg(Color::Red).add_modifier(Modifier::BOLD)),
            ];
            if names {
                spans.push(Span::styled(format!("  {:<name_w$}", c.name), c.state_style));
            }
            spans.push(Span::styled(format!("  {:<freq_w$}", c.freq), c.state_style));
            if settings {
                spans.push(Span::styled(format!("  {:<settings_w$}", c.settings), c.state_style));
            }
            spans.push(Span::raw(" "));
            text::truncate_line(Line::from(spans), inner)
        })
        .collect()
}

/// Every radio on one line (a short terminal): "▸1 ● 433.92  2 ● 315.00  3 ○ 868.35", cut to
/// `width`.
fn radios_compact_line(cols: &[RadioColumns], width: usize) -> Line<'static> {
    let mut spans = vec![Span::raw(" ")];
    for c in cols {
        let symbol = c.state.split(' ').next().unwrap_or("");
        let freq = c.freq.trim_end_matches(" MHz");
        spans.push(Span::styled(format!("{} {symbol} {freq}{}  ", c.mark.trim(), c.rec), c.state_style));
    }
    text::truncate_line(Line::from(spans), width)
}

/// The keys of normal mode, from the fullest list to the shortest: `?` lists every key, so the
/// shorter ones leave some out. An 80-column terminal's bar keeps Tab (nothing else on screen
/// says where the settings are) rather than `a`, which the Enter menu offers too.
const NORMAL_KEYS: [&str; 6] = [
    "?: Help | ↑↓: Select | Enter: Actions | a: Analyze | i: Info | d: Delete | r: RX | Tab: Settings | :: Command | q: Quit",
    "?: Help | Enter: Actions | a: Analyze | i: Info | d: Delete | r: RX | Tab: Settings | :: Command | q: Quit",
    "?: Help | Enter: Actions | a: Analyze | i: Info | r: RX | Tab: Settings | :: Command | q: Quit",
    "?: Help | Enter: Actions | a: Analyze | r: RX | Tab: Settings | :: Command | q: Quit",
    "?: Help | Enter: Actions | r: RX | Tab: Settings | :: Command | q: Quit",
    "?: Help | Enter: Actions | r: RX | :: Command | q: Quit",
];
/// The same with several radios: the keys that choose and start them come first. At 80 columns
/// `1-9` goes before Tab does: the settings bar's Radio tab chooses the radio as well.
const NORMAL_KEYS_RADIOS: [&str; 6] = [
    "?: Help | Enter: Actions | a: Analyze | 1-9: Radio | r: RX | R: RX all | Tab: Settings | :radios | :: Command | q: Quit",
    "?: Help | Enter: Actions | a: Analyze | 1-9: Radio | r: RX | R: RX all | Tab: Settings | :: Command | q: Quit",
    "?: Help | Enter: Actions | 1-9: Radio | r: RX | R: RX all | Tab: Settings | :: Command | q: Quit",
    "?: Help | Enter: Actions | 1-9: Radio | r/R: RX | Tab: Settings | :: Command | q: Quit",
    "?: Help | Enter: Actions | r/R: RX | Tab: Settings | :: Command | q: Quit",
    "?: Help | Enter: Actions | r/R: RX | :: Command | q: Quit",
];

/// The first of `texts` (fullest first) that fits `width` columns after the bar's leading space,
/// else the last, which is then cut.
fn fitting<'a>(texts: &[&'a str], width: usize) -> &'a str {
    texts.iter().copied().find(|t| text::width(t) < width).or(texts.last().copied()).unwrap_or("")
}

/// The help bar's text in a mode: the fullest of the mode's texts that fits `width` columns.
fn help_bar_text(
    mode: InputMode,
    export: Option<crate::app::ExportFormat>,
    several_radios: bool,
    width: usize,
) -> &'static str {
    use crate::app::ExportFormat;
    let texts: &[&str] = match mode {
        InputMode::Normal if several_radios => &NORMAL_KEYS_RADIOS,
        InputMode::Normal => &NORMAL_KEYS,
        InputMode::Command => &[
            "Enter: Execute | Up/Down: Earlier commands | Ctrl-U: Clear | Esc: Cancel",
            "Enter: Execute | Up/Down: Earlier commands | Esc: Cancel",
        ],
        InputMode::SignalMenu => &["Up/Down: Navigate | Enter: Select | Esc: Close"],
        InputMode::SettingsSelect => &["Left/Right: Select | Tab: Cycle | Enter: Edit | Esc: Back"],
        InputMode::SettingsEdit => &["Up/Down: Change value | Enter: Apply | Esc: Cancel"],
        InputMode::HackRfNotDetected => &["Press any key to continue"],
        InputMode::StartupImport => &["y: Import | n/Esc: Skip"],
        InputMode::ExportFilename => match export {
            Some(ExportFormat::Fob) => &["Tab/Enter: Next field | Esc: Cancel export"],
            Some(ExportFormat::Flipper | ExportFormat::FlipperKey) => &["Enter: Save & export | Esc: Cancel export"],
            None => &["Enter: Confirm | Esc: Cancel"],
        },
        InputMode::FobMetaYear
        | InputMode::FobMetaMake
        | InputMode::FobMetaModel
        | InputMode::FobMetaRegion
        | InputMode::FobMetaCommand => &[
            "Tab/Enter: Next field | Shift-Tab: Back | Esc: Cancel export",
            "Tab/Enter: Next | Shift-Tab: Back | Esc: Cancel export",
        ],
        InputMode::FobMetaNotes => &["Enter: Save & export | Shift-Tab: Back | Esc: Cancel export"],
        InputMode::CaptureMetaYear
        | InputMode::CaptureMetaMake
        | InputMode::CaptureMetaModel
        | InputMode::CaptureMetaRegion => &["Tab/Enter: Next field | Shift-Tab: Back | Esc: Cancel"],
        InputMode::CaptureMetaCommand => &["Enter: Save | Shift-Tab: Back | Esc: Cancel"],
        InputMode::License | InputMode::Credits => &["Up/Down, PgUp/PgDn: Scroll | Esc: Close"],
        InputMode::LoadFileBrowser => &[
            "Up/Down: Select | Enter: Open | Left/Backspace: Parent folder | Esc: Close",
            "Up/Down: Select | Enter: Open | Left: Parent | Esc: Close",
        ],
        InputMode::SessionRestore
        | InputMode::ConfirmClear
        | InputMode::Analyze
        | InputMode::Help
        | InputMode::Radios
        | InputMode::ConfirmQuit
        | InputMode::ConfirmDeleteAll => super::app_prompts::help_texts(mode),
        InputMode::NewRemote | InputMode::NewRemoteEdit | InputMode::SendButton => {
            super::add_manually::help_texts(mode)
        }
    };
    fitting(texts, width)
}

/// Render the context-sensitive help bar.
fn render_help_bar(frame: &mut Frame, area: Rect, app: &App) {
    let help_text = help_bar_text(app.input_mode, app.export_format, app.several_radios(), area.width as usize);

    // Wider than the terminal: cut with "…" so the cut shows.
    let help = Paragraph::new(Line::from(Span::styled(
        text::truncate_end(&format!(" {}", help_text), area.width as usize),
        Style::default().fg(Color::DarkGray),
    )));

    frame.render_widget(help, area);
}

/// Center a rect of given width/height in the given area
fn centered_rect(width: u16, height: u16, area: Rect) -> Rect {
    let x = area.x + area.width.saturating_sub(width) / 2;
    let y = area.y + area.height.saturating_sub(height) / 2;
    Rect::new(x, y, width.min(area.width), height.min(area.height))
}

/// Render the :load file browser overlay (centered, list of dirs and .fob/.sub/IQ files).
fn render_load_file_browser(frame: &mut Frame, app: &App) {
    let area = frame.area();
    let entries = &app.load_browser_entries;
    let dim = Style::default().fg(Color::DarkGray);
    let cyan = Style::default().fg(Color::Cyan);

    // A folder without files to load says so in the list's last row.
    let no_files = !entries.iter().any(|(_, _, is_dir)| !is_dir);
    // As wide as the longest name needs (within limits), as tall as the list (up to 16 rows and
    // the terminal): borders, path and a blank row around the entries.
    let longest = entries.iter().map(|(name, _, _)| text::width(name)).max().unwrap_or(0);
    let popup_width = (longest as u16)
        .saturating_add(8)
        .clamp(56, 96)
        .min(area.width.saturating_sub(4).max(56))
        .min(area.width);
    let wanted_rows = (entries.len() + usize::from(no_files)).clamp(1, crate::app::LOAD_BROWSER_ROWS);
    // (The help bar at the bottom stays in view.)
    let rows = wanted_rows.min(area.height.saturating_sub(6) as usize);
    let popup = centered_rect(popup_width, rows as u16 + 4, area);
    frame.render_widget(Clear, popup);
    let inner_w = popup.width.saturating_sub(2) as usize;

    // The end of a long path is the part that says where you are (cut by characters, never
    // inside one).
    let path = app.load_browser_cwd.to_string_lossy();
    let mut lines = vec![
        Line::styled(format!("  {}", text::truncate_start(&path, inner_w.saturating_sub(3))), dim),
        Line::default(),
    ];

    let entry_rows = rows.saturating_sub(usize::from(no_files));
    let selected = app.load_browser_selected.min(entries.len().saturating_sub(1));
    // The app scrolls for 16 rows; a shorter terminal shows fewer, still around the selection
    // (and a scroll position past the entries cannot slice out of range).
    let start = text::keep_in_view(app.load_browser_scroll, selected, entry_rows, entries.len());
    for (idx, (name, _path, is_dir)) in entries.iter().enumerate().skip(start).take(entry_rows) {
        let is_selected = idx == selected;
        let prefix = if is_selected { " > " } else { "   " };
        let suffix = if *is_dir { "/" } else { "" };
        // Long names keep their start and their end (extension, numbering).
        let name = text::truncate_middle(&format!("{}{}", name, suffix), inner_w.saturating_sub(4));
        let style = if is_selected {
            theme::HIGHLIGHT
        } else if *is_dir {
            cyan
        } else {
            theme::TEXT
        };
        // The highlight spans the whole row.
        let pad = inner_w.saturating_sub(prefix.len() + text::width(&name));
        lines.push(Line::styled(format!("{}{}{}", prefix, name, " ".repeat(pad)), style));
    }
    if no_files && rows > 0 {
        lines.push(Line::styled("   (no .fob, .sub or IQ files here)", dim));
    }

    let mut block = Block::default()
        .title(" Load file (.fob .sub .cs8 .cu8 .cf32) ")
        .borders(Borders::ALL)
        .border_style(cyan);
    if let Some(hint) = text::more_hint(start, entry_rows, entries.len()) {
        block = block.title_bottom(Line::from(hint).right_aligned());
    }
    frame.render_widget(Paragraph::new(lines).block(block), popup);
}

/// A centered message box sized to its text: every paragraph is word-wrapped (and centered) to
/// the box's width, an empty one is a blank line, and the box is as tall as the result (up to the
/// terminal), so nothing is cut off.
fn render_message_popup(frame: &mut Frame, title: &str, border: Color, width: u16, paragraphs: &[(String, Style)]) {
    let area = frame.area();
    let width = width.min(area.width);
    // A column of margin inside each border.
    let text_width = width.saturating_sub(4) as usize;
    let mut lines = vec![Line::default()];
    for (paragraph, style) in paragraphs {
        if paragraph.is_empty() {
            lines.push(Line::default());
        }
        for line in text::wrap(paragraph, text_width) {
            lines.push(Line::styled(line, *style));
        }
    }
    lines.push(Line::default());

    let popup = centered_rect(width, lines.len() as u16 + 2, area);
    frame.render_widget(Clear, popup);
    let block = Block::default()
        .title(Span::styled(format!(" {} ", title), Style::default().fg(border)))
        .borders(Borders::ALL)
        .border_style(Style::default().fg(border));
    frame.render_widget(Paragraph::new(lines).block(block).alignment(Alignment::Center), popup);
}

/// "a, b or c".
fn or_list(words: &[&str]) -> String {
    match words.split_last() {
        Some((last, [])) => last.to_string(),
        Some((last, rest)) => format!("{} or {last}", rest.join(", ")),
        None => String::new(),
    }
}

/// The paragraphs of the no-device warning. `kinds` are the kinds of radio KAT looks for (config
/// `device`): the warning names those, not a kind the config leaves out. `open_error` is why the
/// radios plugged in could not be used.
fn no_device_message(kinds: &[RadioKind], open_error: Option<&str>) -> Vec<(String, Style)> {
    let red = Style::default().fg(Color::Red);
    let used: Vec<RadioKind> = RadioKind::ALL.into_iter().filter(|k| kinds.is_empty() || kinds.contains(k)).collect();
    let names: Vec<&str> = used
        .iter()
        .map(|k| match k {
            RadioKind::HackRf => "HackRF",
            RadioKind::RtlSdr => "RTL-SDR",
            RadioKind::Flipper => "Flipper Zero",
        })
        .collect();
    let connect: Vec<&str> = used
        .iter()
        .map(|k| match k {
            RadioKind::HackRf => "a HackRF (TX+RX)",
            RadioKind::RtlSdr => "an RTL-SDR (RX only)",
            RadioKind::Flipper => "a Flipper Zero (TX+RX over USB)",
        })
        .collect();
    // A kind the config leaves out is not looked for: say so, or plugging one in looks broken.
    let limited = if used.len() < RadioKind::ALL.len() { " Other kinds are left out by device in config.ini." } else { "" };
    let mut lines = vec![
        (format!("No {} detected.", or_list(&names)), red.add_modifier(Modifier::BOLD)),
        (String::new(), red),
    ];
    // Why the devices couldn't be opened (permissions, busy, DVB-T driver): the popup grows to fit.
    if let Some(reason) = open_error {
        lines.push((reason.to_string(), theme::TEXT));
        lines.push((String::new(), red));
    }
    lines.extend([
        (
            format!(
                "Connect {}: :radios finds it without a restart. Until then KAT has no TX/RX, and still \
                 loads, decodes, analyzes and exports.{limited}",
                or_list(&connect)
            ),
            red,
        ),
        (String::new(), red),
        ("Press any key to continue.".to_string(), theme::TEXT.add_modifier(Modifier::BOLD)),
    ]);
    lines
}

/// Render the no-device warning (red box at startup when no radio was found)
fn render_hackrf_not_detected(frame: &mut Frame, app: &App) {
    let message = no_device_message(&app.storage.config.radio_devices, app.radio_open_error.as_deref());
    render_message_popup(frame, "Warning", Color::Red, 60, &message);
}

/// Render the startup import prompt overlay
fn render_startup_import_prompt(frame: &mut Frame, app: &App) {
    let count = app.pending_fob_files.len();
    let paragraphs = [
        (
            format!("Found {count} file{} (.fob / .sub) in import dir (incl. subfolders).", crate::plural(count)),
            Style::default().fg(Color::Yellow),
        ),
        (String::new(), theme::TEXT),
        ("Import them? (y/n)".to_string(), theme::TEXT.add_modifier(Modifier::BOLD)),
    ];
    render_message_popup(frame, "Import Saved Signals", Color::Cyan, 56, &paragraphs);
}

/// Where a form field is relative to the one being edited.
#[derive(Clone, Copy, PartialEq, Eq)]
enum FieldState {
    Done,
    Active,
    Pending,
}

impl FieldState {
    fn of(idx: usize, current: usize) -> Self {
        match idx.cmp(&current) {
            std::cmp::Ordering::Less => FieldState::Done,
            std::cmp::Ordering::Equal => FieldState::Active,
            std::cmp::Ordering::Greater => FieldState::Pending,
        }
    }
}

/// One row of a form, `width` columns wide: the label, then the value (its end, behind a leading
/// "…", when it is wider than the field: that is where typing happens), the cursor on the field
/// being edited — before a fixed `suffix` such as ".fob", where typing inserts — and ✓ on a
/// filled-in field already passed. An empty value shows the dimmed placeholder.
fn form_field_line(
    label: &str,
    value: &str,
    suffix: &str,
    placeholder: &str,
    state: FieldState,
    width: usize,
) -> Line<'static> {
    let dim = Style::default().fg(Color::DarkGray);
    let label_style = match state {
        FieldState::Active => Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD),
        FieldState::Done => Style::default().fg(Color::Green),
        FieldState::Pending => dim,
    };
    let active = state == FieldState::Active;
    let cursor = if active { theme::CURSOR } else { "" };
    let check = if state == FieldState::Done && !value.is_empty() { " ✓" } else { "" };
    // Columns left for the value once the label, cursor, suffix and ✓ have theirs.
    let room = width.saturating_sub(
        text::width(label) + text::width(cursor) + text::width(suffix) + text::width(check),
    );

    let mut spans = vec![Span::styled(label.to_string(), label_style)];
    if value.is_empty() {
        spans.push(Span::styled(cursor, theme::TEXT));
        spans.push(Span::styled(suffix.to_string(), dim));
        let gap = if cursor.is_empty() && suffix.is_empty() { "" } else { " " };
        let placeholder = text::truncate_end(placeholder, room.saturating_sub(gap.len()));
        spans.push(Span::styled(format!("{}{}", gap, placeholder), dim));
    } else {
        spans.push(Span::styled(text::truncate_start(value, room), theme::TEXT));
        spans.push(Span::styled(cursor, theme::TEXT));
        spans.push(Span::styled(suffix.to_string(), dim));
        spans.push(Span::styled(check, Style::default().fg(Color::Green)));
    }
    Line::from(spans)
}

/// "Signal:" and "Key:" rows naming the capture a form is about, and the divider under them.
fn form_capture_summary(capture: &Capture, width: usize) -> Vec<Line<'static>> {
    let dim = Style::default().fg(Color::DarkGray);
    let accent = Style::default().fg(Color::Yellow);
    let (name, tuning, serial) = super::signal_menu::capture_words(capture);
    let signal = super::signal_menu::join_words([Some(name), Some(tuning), serial]);
    // An unknown capture has no decoded bits: its files hold the recorded pulses.
    let key = if capture.data_count_bit > 0 {
        format!("0x{} ({})", capture.data_hex(), capture.encryption_type())
    } else if capture.has_raw_data() {
        "none decoded (the recorded pulses are saved)".to_string()
    } else {
        "none decoded".to_string()
    };
    vec![
        text::truncate_line(Line::from(vec![Span::styled("  Signal:  ", dim), Span::styled(signal, accent)]), width),
        text::truncate_line(Line::from(vec![Span::styled("  Key:     ", dim), Span::styled(key, accent)]), width),
    ]
}

/// The divider between a form's summary and its fields.
fn form_divider(width: usize) -> Line<'static> {
    Line::styled(
        format!("  {}", "─".repeat(width.saturating_sub(4))),
        Style::default().fg(Color::DarkGray),
    )
}

/// Width of the export and metadata forms (narrower terminals get their full width).
const FORM_WIDTH: u16 = 62;

/// Render the export form overlay (filename + optional .fob metadata)
fn render_export_form(frame: &mut Frame, app: &App) {
    use crate::app::ExportFormat;

    let is_fob = app.export_format == Some(ExportFormat::Fob);
    let ext = if is_fob { ".fob" } else { ".sub" };

    let area = frame.area();
    let popup_width = FORM_WIDTH.min(area.width);
    let width = popup_width.saturating_sub(2) as usize;
    let dim_style = Style::default().fg(Color::DarkGray);
    let accent_style = Style::default().fg(Color::Yellow);

    // Build the ordered list of field modes for this export type
    // Filename is always first; .fob adds metadata fields after
    let field_modes: Vec<InputMode> = if is_fob {
        vec![
            InputMode::ExportFilename,
            InputMode::FobMetaYear,
            InputMode::FobMetaMake,
            InputMode::FobMetaModel,
            InputMode::FobMetaRegion,
            InputMode::FobMetaCommand,
            InputMode::FobMetaNotes,
        ]
    } else {
        vec![InputMode::ExportFilename]
    };

    let current_idx = field_modes
        .iter()
        .position(|m| *m == app.input_mode)
        .unwrap_or(0);

    let mut lines = Vec::new();

    // --- Signal summary section ---
    if let Some(capture) = app
        .export_capture_id
        .and_then(|id| app.captures.iter().find(|c| c.id == id))
    {
        lines.extend(form_capture_summary(capture, width));
    }
    lines.push(form_divider(width));

    // --- Form fields: (label, value, suffix, placeholder) ---
    // The extension is drawn after the cursor: typing goes into the name before it.
    let mut fields: Vec<(&str, &str, &str, &str)> =
        vec![("  File:    ", app.export_filename.as_str(), ext, "(enter filename)")];
    if is_fob {
        fields.extend([
            ("  Year:    ", app.fob_meta_year.as_str(), "", "(e.g. 2024)"),
            ("  Make:    ", app.fob_meta_make.as_str(), "", "(auto-detected from protocol)"),
            ("  Model:   ", app.fob_meta_model.as_str(), "", "(e.g. Sportage, F-150)"),
            ("  Region:  ", app.fob_meta_region.as_str(), "", "(e.g. NA, EU, APAC, MEA)"),
            ("  Command: ", app.fob_meta_command.as_str(), "", "(e.g. Unlock, Lock, Trunk, Panic)"),
            ("  Notes:   ", app.fob_meta_notes.as_str(), "", "(optional — color, trim, VIN, etc.)"),
        ]);
    }
    for (idx, &(label, value, suffix, placeholder)) in fields.iter().enumerate() {
        let state = FieldState::of(idx, current_idx);
        lines.push(form_field_line(label, value, suffix, placeholder, state, width));
    }

    lines.push(Line::from(""));

    // Progress indicator
    let total_fields = fields.len();
    let progress = format!("  Step {}/{}", current_idx + 1, total_fields);
    let hint = if current_idx == total_fields - 1 {
        "  Enter: save & export · Esc: cancel"
    } else {
        "  Enter: next · Esc: cancel"
    };
    lines.push(text::truncate_line(
        Line::from(vec![
            Span::styled(progress, accent_style),
            Span::styled("  ", dim_style),
            Span::styled(hint, dim_style),
        ]),
        width,
    ));

    let title = if is_fob {
        " Export .fob — Filename & Vehicle Details "
    } else if app.export_format == Some(ExportFormat::FlipperKey) {
        " Export Flipper key file (.sub) "
    } else {
        " Export .sub (Flipper Zero) "
    };

    let popup = centered_rect(popup_width, lines.len() as u16 + 2, area);
    frame.render_widget(Clear, popup);
    let block = Block::default()
        .title(title)
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Green));
    frame.render_widget(Paragraph::new(lines).block(block), popup);
}

/// Render the capture metadata form (Year/Make/Model/Region/Command for vuln lookup and .fob
/// export). Shown when user presses 'i' on a capture.
fn render_capture_meta_form(frame: &mut Frame, app: &App) {
    let area = frame.area();
    let popup_width = FORM_WIDTH.min(area.width);
    let width = popup_width.saturating_sub(2) as usize;
    let dim_style = Style::default().fg(Color::DarkGray);

    let field_modes = [
        InputMode::CaptureMetaYear,
        InputMode::CaptureMetaMake,
        InputMode::CaptureMetaModel,
        InputMode::CaptureMetaRegion,
        InputMode::CaptureMetaCommand,
    ];
    let current_idx = field_modes
        .iter()
        .position(|m| *m == app.input_mode)
        .unwrap_or(0);

    let mut lines = Vec::new();

    if let Some(capture) = app
        .capture_meta_capture_id
        .and_then(|id| app.captures.iter().find(|c| c.id == id))
    {
        lines.extend(form_capture_summary(capture, width));
    }
    lines.push(form_divider(width));

    // (label, value, placeholder). The CVE lookup compares Make and Model with the database's
    // names (any case); there is no wildcard to type.
    let fields = [
        ("  Year:    ", app.capture_meta_year.as_str(), "(e.g. 2021)"),
        ("  Make:    ", app.capture_meta_make.as_str(), "(e.g. Renault, Honda)"),
        ("  Model:   ", app.capture_meta_model.as_str(), "(e.g. ZOE, Civic)"),
        ("  Region:  ", app.capture_meta_region.as_str(), "(e.g. NA, EU)"),
        ("  Command: ", app.capture_meta_command.as_str(), "(e.g. Unlock, Lock, Trunk, Panic)"),
    ];
    for (idx, &(label, value, placeholder)) in fields.iter().enumerate() {
        let state = FieldState::of(idx, current_idx);
        lines.push(form_field_line(label, value, "", placeholder, state, width));
    }

    lines.push(Line::from(""));
    let progress = format!(
        "  Step {}/{} — Used for vuln lookup and .fob export",
        current_idx + 1,
        fields.len()
    );
    lines.push(Line::styled(text::truncate_end(&progress, width), dim_style));

    let popup = centered_rect(popup_width, lines.len() as u16 + 2, area);
    frame.render_widget(Clear, popup);
    let block = Block::default()
        .title(" Capture Metadata — Year / Make / Model / Region / Command ")
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Cyan));
    frame.render_widget(Paragraph::new(lines).block(block), popup);
}

#[cfg(test)]
mod tests {
    use super::*;
    use ratatui::{backend::TestBackend, Terminal};

    fn strings(line: &Line) -> String {
        line.spans.iter().map(|s| s.content.as_ref()).collect()
    }

    /// Draw with `render` on a `width`×`height` test terminal; the screen's rows as text.
    fn screen(width: u16, height: u16, render: impl FnOnce(&mut Frame)) -> Vec<String> {
        let mut terminal = Terminal::new(TestBackend::new(width, height)).unwrap();
        terminal.draw(render).unwrap();
        let buffer = terminal.backend().buffer();
        (0..height)
            .map(|y| (0..width).map(|x| buffer[(x, y)].symbol()).collect())
            .collect()
    }

    fn three_radios() -> Vec<Radio> {
        use crate::radio::devices::FoundRadio;
        let config = crate::storage::Config::default();
        let t = |frequency| crate::app::Tuning { frequency, lna_gain: 24, vga_gain: 20, amp: false };
        let mut hackrf = Radio::new(
            FoundRadio::hackrf(Some("0000000000000000457863c82e5a1f3d".to_string())),
            "HackRF".to_string(),
            t(433_920_000),
            &config,
        );
        hackrf.state = RadioState::Receiving;
        let rtl = Radio::new(FoundRadio::rtlsdr(0, Some("00000002".to_string()), true), "RTL-SDR".to_string(), t(315_000_000), &config);
        let flipper = Radio::new(FoundRadio::flipper("/dev/ttyACM0".to_string(), None), "Flipper".to_string(), t(868_350_000), &config);
        vec![hackrf, rtl, flipper]
    }

    /// Several radios: a header line each, the selected one marked, the transmitter "TX", the
    /// columns lined up and every line as wide as the header; narrower, the title and settings go
    /// first; a short terminal gets them all on one line.
    #[test]
    fn several_radios_get_a_header_line_each() {
        let radios = three_radios();
        let cols = radio_columns(&radios, Some(1), Some(0), None);
        let lines: Vec<String> = radios_header_lines(&cols, 118).iter().map(strings).collect();
        assert_eq!(lines.len(), 3);
        assert!(lines[0].starts_with(" Keyfob Analysis Toolkit v"), "{lines:#?}");
        for (line, words) in lines.iter().zip([
            [" 1 ● RX", "HackRF TX", "433.92 MHz", "LNA:24 VGA:20 AMP:OFF"],
            ["▸2 ○ IDLE", "RTL-SDR", "315.00 MHz", "Gain:20"],
            [" 3 ○ IDLE", "Flipper", "868.35 MHz", "Preset: firmware's AM"],
        ]) {
            assert!(words.iter().all(|w| line.contains(w)), "{words:?} in {line:?}");
        }
        let freq_at = |l: &String| l[..l.find("MHz").unwrap()].chars().count();
        assert!(lines.iter().all(|l| freq_at(l) == freq_at(&lines[0]) && text::width(l) == 118), "{lines:#?}");
        let narrow: Vec<String> = radios_header_lines(&cols, 40).iter().map(strings).collect();
        assert!(narrow.iter().all(|l| text::width(l) <= 40 && l.contains("MHz") && !l.contains("LNA")), "{narrow:#?}");
        let compact = strings(&radios_compact_line(&cols, 60));
        assert!(compact.contains("1 ● 433.92") && compact.contains("▸2 ○ 315.00") && compact.contains("3 ○ 868.35"), "{compact}");
    }

    /// Each radio's meter column fits inside the bar's borders.
    #[test]
    fn every_radio_gets_a_meter_column() {
        for n in 2..=9usize {
            let inner = meters_width(n) - 2;
            let last = 2 * (n as u16 - 1);
            assert!(last < inner, "{n} radios: column {last} in {inner}");
        }
        assert_eq!(meters_width(1), RSSI_BAR_WIDTH);
    }

    #[test]
    fn no_device_warning_shows_all_of_its_text() {
        for (width, height) in [(80, 24), (60, 16), (120, 40)] {
            let rows = screen(width, height, |f| {
                render_message_popup(f, "Warning", Color::Red, 60, &no_device_message(&RadioKind::ALL, None))
            });
            let text = rows.join("\n");
            for words in ["No HackRF, RTL-SDR or Flipper Zero detected.", "Press any key to continue.", ":radios", "exports."] {
                assert!(text.contains(words), "{width}×{height}: {words:?} missing\n{text}");
            }
            assert!(!text.contains("config.ini"), "{width}×{height}: every kind is in use\n{text}");
        }
    }

    /// The warning names the kinds of radio KAT looks for: a kind `device` leaves out is not said
    /// to be missing, and not one to connect. With the reason line it still fits 60×16.
    #[test]
    fn no_device_warning_names_the_kinds_in_use() {
        let text = |kinds: &[RadioKind], why| {
            let all: Vec<String> = no_device_message(kinds, why).into_iter().map(|(t, _)| t).collect();
            all.join("\n")
        };
        let two = text(&[RadioKind::HackRf, RadioKind::RtlSdr], None);
        assert!(two.starts_with("No HackRF or RTL-SDR detected."), "{two}");
        assert!(two.contains("Connect a HackRF (TX+RX) or an RTL-SDR (RX only):"), "{two}");
        assert!(two.contains("left out by device in config.ini") && !two.contains("Flipper"), "{two}");
        let one = text(&[RadioKind::Flipper], None);
        assert!(one.starts_with("No Flipper Zero detected.") && one.contains("Connect a Flipper Zero (TX+RX over USB):"), "{one}");
        // No list (cannot be configured): every kind, as `auto`.
        assert_eq!(text(&[], None), text(&RadioKind::ALL, None));

        let why = "HackRF: not found (if one is plugged in: check its USB permissions / udev rules) · RTL-SDR: not found";
        let rows = screen(60, 16, |f| {
            let message = no_device_message(&[RadioKind::HackRf, RadioKind::RtlSdr], Some(why));
            render_message_popup(f, "Warning", Color::Red, 60, &message)
        });
        let shown = rows.join("\n");
        for words in ["No HackRF or RTL-SDR detected.", "udev", "config.ini.", "Press any key to continue."] {
            assert!(shown.contains(words), "{words:?} missing\n{shown}");
        }
    }

    /// The help bar shows the fullest list of keys that fits, and its shortest fits the narrowest
    /// terminal, so `?` and `q` are never cut away.
    #[test]
    fn the_help_bar_fits_the_terminal() {
        for keys in [NORMAL_KEYS, NORMAL_KEYS_RADIOS] {
            assert!(keys.windows(2).all(|w| text::width(w[0]) > text::width(w[1])), "{keys:?}");
            for width in [MIN_WIDTH as usize, 72, 80, 100, 120, 200] {
                let shown = fitting(&keys, width);
                assert!(text::width(shown) < width, "{width}: {shown}");
                assert!(shown.starts_with("?: Help") && shown.ends_with("q: Quit"), "{shown}");
                // The next fuller one would not have fitted.
                let fuller = keys.iter().take_while(|k| **k != shown).last();
                assert!(fuller.is_none_or(|f| text::width(f) >= width), "{width}: {fuller:?}");
            }
            assert_eq!(fitting(&keys, 200), keys[0]);
            // An 80-column terminal's bar says where the settings are.
            assert!(fitting(&keys, 80).contains("Tab: Settings"), "{}", fitting(&keys, 80));
        }
        // Nothing fits: the shortest, which the bar then cuts.
        assert_eq!(fitting(&["a long text", "short"], 3), "short");
    }

    /// In no mode is the help bar's text cut at the narrowest terminal.
    #[test]
    fn every_modes_help_fits_the_narrowest_terminal() {
        use crate::app::ExportFormat;
        let modes = [
            IM::Normal, IM::Command, IM::SignalMenu, IM::SettingsSelect, IM::SettingsEdit, IM::HackRfNotDetected,
            IM::StartupImport, IM::SessionRestore, IM::ConfirmClear, IM::Analyze, IM::ExportFilename,
            IM::FobMetaYear, IM::FobMetaMake, IM::FobMetaModel, IM::FobMetaRegion, IM::FobMetaCommand,
            IM::FobMetaNotes, IM::CaptureMetaYear, IM::CaptureMetaMake, IM::CaptureMetaModel,
            IM::CaptureMetaRegion, IM::CaptureMetaCommand, IM::License, IM::Credits, IM::LoadFileBrowser,
            IM::NewRemote, IM::NewRemoteEdit, IM::SendButton, IM::Help, IM::Radios, IM::ConfirmQuit,
            IM::ConfirmDeleteAll,
        ];
        let exports = [None, Some(ExportFormat::Fob), Some(ExportFormat::Flipper), Some(ExportFormat::FlipperKey)];
        for mode in modes {
            for export in exports {
                for radios in [false, true] {
                    let narrow = help_bar_text(mode, export, radios, MIN_WIDTH as usize);
                    assert!(!narrow.is_empty(), "{mode:?}");
                    assert!(text::width(narrow) < MIN_WIDTH as usize, "{mode:?}: {narrow}");
                    // A wide terminal shows a text at least as full.
                    let wide = help_bar_text(mode, export, radios, 200);
                    assert!(text::width(wide) >= text::width(narrow), "{mode:?}: {wide}");
                }
            }
        }
        // The overlay's keys to the captures next to this one are in every text of its mode.
        for width in [MIN_WIDTH as usize, 80, 200] {
            assert!(help_bar_text(IM::Analyze, None, false, width).contains("Left/Right: Other captures"));
        }
    }

    #[test]
    fn fixed_rows_keep_their_height() {
        let heights = |w, h, status, settings, command| -> Vec<u16> {
            main_rows(Rect::new(0, 0, w, h), 3, status, settings, command).iter().map(|r| r.height).collect()
        };
        // 80×24 with the command line open: header, middle, status, command, help.
        assert_eq!(heights(80, 24, 3, false, true), vec![3, 14, 3, 3, 1]);
        assert_eq!(heights(80, 24, 3, true, false), vec![3, 3, 14, 3, 1]);
        assert_eq!(heights(120, 40, 3, false, false), vec![3, 33, 3, 1]);
        // The smallest screen still leaves the middle row 6 rows.
        assert_eq!(heights(MIN_WIDTH, MIN_HEIGHT, 3, false, true), vec![3, 6, 3, 3, 1]);
        // A status bar grown for a three-line message takes its rows from the middle.
        assert_eq!(heights(80, 24, 5, false, false), vec![3, 15, 5, 1]);
    }

    /// The status bar grows for a long message by the rows the middle row can spare.
    #[test]
    fn the_status_bar_grows_for_a_long_message() {
        // (lines of the message, terminal rows, rows of header + help + settings / command)
        let height = status_height;
        // A short message, or none: one line in the border.
        assert_eq!(height(1, 24, 4), 3);
        assert_eq!(height(0, 24, 4), 3);
        // 80×24: the middle row has 17 rows, 3 more than its detail panel needs.
        assert_eq!(height(2, 24, 4), 4);
        assert_eq!(height(3, 24, 4), 5);
        // 22 rows: one row to spare over the detail panel; 21 rows: none, the message is cut.
        assert_eq!(height(3, 22, 4), 4);
        assert_eq!(height(3, 21, 4), 3);
        // Shorter, there is no panel to keep: the table gives rows down to its six.
        assert_eq!(height(3, 20, 4), 5);
        assert_eq!(height(3, MIN_HEIGHT, 4), 5);
        assert_eq!(height(3, 14, 4), 4);
        // The smallest screen with the command line or the settings bar open: nothing to spare.
        assert_eq!(height(3, MIN_HEIGHT, 7), 3);
        assert_eq!(height(3, 24, 7), 3);
        // Three radios with a header line each (5 rows) at the height that allows it.
        assert_eq!(height(3, MIN_HEIGHT + 2, 6), 5);
        // Whatever the message, the middle row keeps its six rows: with one radio, and with
        // three that have a header line each (from 18 rows), the command line closed and open.
        for rows in MIN_HEIGHT..60 {
            let others: &[u16] = if rows >= MIN_HEIGHT + 2 { &[4, 7, 6, 9] } else { &[4, 7] };
            for &other in others {
                let status = height(status_bar::MAX_LINES, rows, other);
                assert!((3..=5).contains(&status), "{rows} {other}: {status}");
                assert!(rows - other - status >= MIDDLE_MIN_HEIGHT, "{rows} {other}: {status}");
            }
        }
    }

    #[test]
    fn too_small_notice_names_the_sizes() {
        let rows = screen(50, 10, |f| render_too_small(f, f.area()));
        assert!(rows.iter().any(|r| r.contains("Terminal too small: 50×10 (need 60×16)")), "{rows:#?}");
    }

    #[test]
    fn form_field_keeps_the_end_cursor_and_extension_in_view() {
        let name = "Unknown_Kia-Hyundai_Unknown_Unknown_Unlock_1A2B3C4D";
        // Wide enough: name, cursor, extension.
        let line = form_field_line("  File:    ", name, ".fob", "(enter filename)", FieldState::Active, 80);
        assert_eq!(strings(&line), format!("  File:    {name}█.fob"));
        // Too narrow: the end of the name (hex suffix), the cursor and the extension stay.
        let line = form_field_line("  File:    ", name, ".fob", "(enter filename)", FieldState::Active, 60);
        let shown = strings(&line);
        assert_eq!(text::width(&shown), 60);
        assert!(shown.ends_with("_Unlock_1A2B3C4D█.fob"), "{shown}");
        assert!(shown.starts_with("  File:    …"), "{shown}");
        // Empty: cursor, extension, then the placeholder.
        let line = form_field_line("  File:    ", "", ".fob", "(enter filename)", FieldState::Active, 60);
        assert_eq!(strings(&line), "  File:    █.fob (enter filename)");
        // Passed fields get ✓; fields not reached show the placeholder alone.
        let line = form_field_line("  Year:    ", "2019", "", "(e.g. 2021)", FieldState::Done, 60);
        assert_eq!(strings(&line), "  Year:    2019 ✓");
        let line = form_field_line("  Year:    ", "", "", "(e.g. 2021)", FieldState::Pending, 60);
        assert_eq!(strings(&line), "  Year:    (e.g. 2021)");
    }
}
