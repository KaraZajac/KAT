//! Main UI layout.

use ratatui::{
    layout::{Alignment, Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Clear, Paragraph},
    Frame,
};

use crate::app::{App, InputMode, RadioState, LICENSE_TEXT};
use crate::capture::Capture;
use crate::radio::hopper::format_mhz;

use super::captures_list::render_captures_list;
use super::command::render_command_line;
use super::settings_menu::{render_settings_dropdown, render_settings_tabs};
use super::signal_menu::render_signal_menu;
use super::status_bar::render_status_bar;
use super::text;
use super::text_overlay::render_text_overlay;
use super::theme;

use crate::app::InputMode as IM;

const VERSION: &str = env!("CARGO_PKG_VERSION");

/// RSSI bar width (right side)
const RSSI_BAR_WIDTH: u16 = 5;

/// Smallest terminal the screen is drawn in: header, captures table with a few rows, status bar,
/// help bar and the command line (or settings bar) all fit. Smaller, a notice says so instead.
const MIN_WIDTH: u16 = 60;
const MIN_HEIGHT: u16 = 16;

/// Draw the entire UI
pub fn draw_ui(frame: &mut Frame, app: &App) {
    let main_area = frame.area();
    if main_area.width < MIN_WIDTH || main_area.height < MIN_HEIGHT {
        render_too_small(frame, main_area);
        return;
    }

    let show_settings = matches!(app.input_mode, IM::SettingsSelect | IM::SettingsEdit);
    let show_command = app.input_mode == IM::Command;
    let rows = main_rows(main_area, show_settings, show_command);

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
        .constraints([Constraint::Min(0), Constraint::Length(RSSI_BAR_WIDTH)])
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

    // Session restore prompt, :clear confirmation, timing analysis overlay
    super::app_prompts::render_app_overlays(frame, app);
    // :new picker / field form, "Send button…" list
    super::add_manually::render(frame, app);
}

/// Full-width rows: header, settings (optional), then middle row split into [captures | RX bar],
/// status, command (optional), help. The fixed rows always get their height (a Min constraint
/// would outrank them and squeeze them away on a normal-size terminal); the middle row takes the
/// rest, 6+ rows at the minimum size.
fn main_rows(area: Rect, show_settings: bool, show_command: bool) -> std::rc::Rc<[Rect]> {
    let mut v_constraints = vec![
        Constraint::Length(3),  // Header (full width)
        Constraint::Fill(1),    // Middle: captures table + detail panel (signal + vulnerability)
        Constraint::Length(3),  // Status bar (full width)
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

/// Render the RSSI bar on the right (vertical bar, bottom = strong): the live level while
/// receiving, full red while transmitting, else empty and dim (the last level read is stale).
fn render_rssi_bar(frame: &mut Frame, area: Rect, app: &App) {
    let dim = Style::default().fg(Color::DarkGray);
    let red = Style::default().fg(Color::Red);
    let (title, border_style, fill_ratio, filled_style) = match app.radio_state {
        RadioState::Transmitting => (" TX ", red, 1.0_f32, red),
        RadioState::Receiving => (" RX ", Style::default(), (app.rssi / 0.6).clamp(0.0, 1.0), Style::default().fg(Color::Green)),
        RadioState::Idle | RadioState::Disconnected => (" RX ", dim, 0.0, dim),
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
}

/// Render the header with title and radio status. When the terminal is narrow the radio state,
/// the recording indicator and the frequency stay longest: the title shortens first, then the
/// gains and the device name are left out.
fn render_header(frame: &mut Frame, area: Rect, app: &App) {
    let (status_symbol, status_style) = match app.radio_state {
        RadioState::Disconnected => ("○", Style::default().fg(Color::Red)),
        RadioState::Idle => ("○", Style::default().fg(Color::Yellow)),
        RadioState::Receiving => ("●", Style::default().fg(Color::Green)),
        RadioState::Transmitting => ("●", Style::default().fg(Color::Red)),
    };

    let rx = app.rx_header();
    let state = format!("{} {}", status_symbol, app.radio_state);
    // Recording indicator (raw IQ being written to disk), right after the state
    let rec = rx
        .recording
        .map(|r| {
            let dropped = if r.dropped_buffers > 0 { format!(" ({} dropped)", r.dropped_buffers) } else { String::new() };
            format!(" ● REC {:.1} MB{dropped}", r.bytes as f64 / (1u64 << 20) as f64)
        })
        .unwrap_or_default();
    let device = app.radio_device_name().unwrap_or("No device");
    let freq = match rx.hop {
        Some((f, i, n)) => format!("HOP {} MHz ({}/{})", format_mhz(f), i + 1, n),
        None => format!("{} MHz", format_mhz(app.frequency)),
    };
    let gains = app.radio_settings_label();

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

/// Render the context-sensitive help bar
fn render_help_bar(frame: &mut Frame, area: Rect, app: &App) {
    let help_text = match app.input_mode {
        InputMode::Normal => {
            // The fullest list that fits; ? lists every key, so the shortest leaves some out.
            if area.width >= 107 {
                "?: Help | ↑↓: Select | Enter: Actions | i: Info | d: Delete | r: RX | Tab: Settings | :: Command | q: Quit"
            } else if area.width >= 94 {
                "?: Help | Enter: Actions | i: Info | d: Delete | r: RX | Tab: Settings | :: Command | q: Quit"
            } else {
                "?: Help | Enter: Actions | i: Info | r: RX | Tab: Radio | :: Command | q: Quit"
            }
        }
        InputMode::Command => "Enter: Execute | Esc: Cancel",
        InputMode::SignalMenu => "Up/Down: Navigate | Enter: Select | Esc: Close",
        InputMode::SettingsSelect => "Left/Right: Select | Tab: Cycle | Enter: Edit | Esc: Back",
        InputMode::SettingsEdit => "Up/Down: Change Value | Enter: Apply | Esc: Cancel",
        InputMode::HackRfNotDetected => "Press any key to continue",
        InputMode::StartupImport => "y: Import | n: Skip",
        InputMode::SessionRestore | InputMode::ConfirmClear | InputMode::TimingAnalysis => {
            super::app_prompts::help_text(app.input_mode)
        }
        InputMode::ExportFilename => {
            match app.export_format {
                Some(crate::app::ExportFormat::Fob) => "Tab/Enter: Next field | Esc: Cancel export",
                Some(crate::app::ExportFormat::Flipper) => "Enter: Save & Export | Esc: Cancel Export",
                Some(crate::app::ExportFormat::FlipperKey) => "Enter: Save & Export | Esc: Cancel Export",
                None => "Enter: Confirm | Esc: Cancel",
            }
        }
        InputMode::FobMetaYear
        | InputMode::FobMetaMake
        | InputMode::FobMetaModel
        | InputMode::FobMetaRegion
        | InputMode::FobMetaCommand => "Tab/Enter: Next field | Shift-Tab: Back | Esc: Cancel export",
        InputMode::FobMetaNotes => "Enter: Save & export | Shift-Tab: Back | Esc: Cancel export",
        InputMode::CaptureMetaYear
        | InputMode::CaptureMetaMake
        | InputMode::CaptureMetaModel
        | InputMode::CaptureMetaRegion => "Tab/Enter: Next field | Shift-Tab: Back | Esc: Cancel",
        InputMode::CaptureMetaCommand => "Enter: Save | Shift-Tab: Back | Esc: Cancel",
        InputMode::License | InputMode::Credits => "Up/Down, PgUp/PgDn: Scroll | Esc: Close",
        InputMode::LoadFileBrowser => "Up/Down: Select | Enter: Open | Left/Backspace: Parent folder | Esc: Close",
        InputMode::Help | InputMode::ConfirmQuit | InputMode::ConfirmDeleteAll => {
            super::app_prompts::help_text(app.input_mode)
        }
        InputMode::NewRemote | InputMode::NewRemoteEdit | InputMode::SendButton => {
            super::add_manually::help_text(app.input_mode)
        }
    };

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

/// Rows of entries the :load browser shows at most (the app scrolls its selection by as many).
const LOAD_BROWSER_VISIBLE_ROWS: usize = 16;

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
    let wanted_rows = (entries.len() + usize::from(no_files)).clamp(1, LOAD_BROWSER_VISIBLE_ROWS);
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

/// The paragraphs of the no-device warning.
fn no_device_message(open_error: Option<&str>) -> Vec<(String, Style)> {
    let red = Style::default().fg(Color::Red);
    let mut lines = vec![
        ("No HackRF, RTL-SDR or Flipper Zero detected.".to_string(), red.add_modifier(Modifier::BOLD)),
        (String::new(), red),
    ];
    // Why the devices couldn't be opened (permissions, busy, DVB-T driver): the popup grows to fit.
    if let Some(reason) = open_error {
        lines.push((reason.to_string(), theme::TEXT));
        lines.push((String::new(), red));
    }
    lines.extend([
        (
            "Connect a HackRF (TX+RX), an RTL-SDR (RX only) or a Flipper Zero (TX+RX over USB) and restart, \
             or continue without TX/RX support."
                .to_string(),
            red,
        ),
        (String::new(), red),
        ("Press any key to continue.".to_string(), theme::TEXT.add_modifier(Modifier::BOLD)),
    ]);
    lines
}

/// Render the no-device warning (red box at startup when neither HackRF nor RTL-SDR found)
fn render_hackrf_not_detected(frame: &mut Frame, app: &App) {
    let message = no_device_message(app.radio_open_error.as_deref());
    render_message_popup(frame, "Warning", Color::Red, 60, &message);
}

/// Render the startup import prompt overlay
fn render_startup_import_prompt(frame: &mut Frame, app: &App) {
    let count = app.pending_fob_files.len();
    let paragraphs = [
        (
            format!("Found {} file(s) (.fob / .sub) in import dir (incl. subfolders).", count),
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
    let signal = format!(
        "#{:02} {} | {} MHz | {} | 0x{}",
        capture.id,
        capture.protocol_name(),
        format_mhz(capture.frequency),
        capture.modulation(),
        capture.serial_hex(),
    );
    let key = format!("0x{} ({})", capture.data_hex(), capture.encryption_type());
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
        "  Enter: Save & Export | Esc: Cancel"
    } else {
        "  Enter: Next | Esc: Cancel"
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

    #[test]
    fn no_device_warning_shows_all_of_its_text() {
        for (width, height) in [(80, 24), (60, 16), (120, 40)] {
            let rows = screen(width, height, |f| {
                render_message_popup(f, "Warning", Color::Red, 60, &no_device_message(None))
            });
            let text = rows.join("\n");
            for words in ["No HackRF, RTL-SDR or Flipper Zero detected.", "Press any key to continue.", "TX/RX support."] {
                assert!(text.contains(words), "{width}×{height}: {words:?} missing\n{text}");
            }
        }
    }

    #[test]
    fn fixed_rows_keep_their_height() {
        let heights = |w, h, settings, command| -> Vec<u16> {
            main_rows(Rect::new(0, 0, w, h), settings, command).iter().map(|r| r.height).collect()
        };
        // 80×24 with the command line open: header, middle, status, command, help.
        assert_eq!(heights(80, 24, false, true), vec![3, 14, 3, 3, 1]);
        assert_eq!(heights(80, 24, true, false), vec![3, 3, 14, 3, 1]);
        assert_eq!(heights(120, 40, false, false), vec![3, 33, 3, 1]);
        // The smallest screen still leaves the middle row 6 rows.
        assert_eq!(heights(MIN_WIDTH, MIN_HEIGHT, false, true), vec![3, 6, 3, 3, 1]);
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
