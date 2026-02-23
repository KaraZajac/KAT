//! Main UI layout.

use ratatui::{
    layout::{Alignment, Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span, Text},
    widgets::{Block, Borders, Clear, List, ListItem, Paragraph, Wrap},
    Frame,
};

use crate::app::{App, InputMode, RadioState, LICENSE_TEXT};

use super::captures_list::render_captures_list;
use super::command::render_command_line;
use super::settings_menu::{render_settings_dropdown, render_settings_tabs};
use super::signal_menu::render_signal_menu;
use super::status_bar::render_status_bar;
use super::text_overlay::render_text_overlay;

use crate::app::InputMode as IM;

const VERSION: &str = env!("CARGO_PKG_VERSION");

/// RSSI bar width (right side)
const RSSI_BAR_WIDTH: u16 = 5;

/// Draw the entire UI
pub fn draw_ui(frame: &mut Frame, app: &App) {
    let show_settings = matches!(app.input_mode, IM::SettingsSelect | IM::SettingsEdit);
    let show_command = app.input_mode == IM::Command;

    // Full-width rows: header, settings (optional), then middle row split into [captures | RX bar], status, command (optional), help
    let main_area = frame.area();
    let mut v_constraints = vec![
        Constraint::Length(3),  // Header (full width)
        Constraint::Min(26),    // Middle: captures table + detail panel (signal + vulnerability)
        Constraint::Length(3),  // Status bar (full width)
        Constraint::Length(1),  // Help bar (full width)
    ];
    if show_settings {
        v_constraints.insert(1, Constraint::Length(3)); // Settings tabs (full width)
    }
    if show_command {
        v_constraints.insert(v_constraints.len() - 1, Constraint::Length(3)); // Command (full width)
    }

    let rows = Layout::default()
        .direction(Direction::Vertical)
        .constraints(v_constraints)
        .split(main_area);

    let mut idx = 0;
    render_header(frame, rows[idx], app);
    idx += 1;

    if show_settings {
        render_settings_tabs(frame, rows[idx], app);
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
        render_settings_dropdown(frame, app);
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

    if app.input_mode == InputMode::License {
        render_text_overlay(frame, app, "License", LICENSE_TEXT, Alignment::Left);
    }
    if app.input_mode == InputMode::Credits {
        render_text_overlay(
            frame,
            app,
            "Credits",
            super::text_overlay::CREDITS_TEXT,
            Alignment::Center,
        );
    }
}

/// Render the RSSI bar on the right (vertical bar, bottom = strong). Shows " TX " in red when transmitting.
fn render_rssi_bar(frame: &mut Frame, area: Rect, app: &App) {
    let is_tx = app.radio_state == RadioState::Transmitting;
    let (title, filled_style, empty_style) = if is_tx {
        (" TX ", Style::default().fg(Color::Red), Style::default().fg(Color::DarkGray))
    } else {
        (" RX ", Style::default().fg(Color::Green), Style::default().fg(Color::DarkGray))
    };

    let block = Block::default()
        .borders(Borders::ALL)
        .border_style(if is_tx { Style::default().fg(Color::Red) } else { Style::default() })
        .title(title);
    let inner = block.inner(area);
    frame.render_widget(block, area);

    if inner.width == 0 || inner.height == 0 {
        return;
    }

    // When TX: show full red bar. When RX: scale RSSI to fill ratio
    let (fill_ratio, style) = if is_tx {
        (1.0_f32, filled_style)
    } else {
        let fill_ratio = (app.rssi / 0.6).min(1.0);
        (fill_ratio, filled_style)
    };
    let filled_rows = (inner.height as f32 * fill_ratio).round() as u16;

    let mut lines = Vec::with_capacity(inner.height as usize);
    for r in 0..inner.height {
        let fill = r >= inner.height.saturating_sub(filled_rows);
        let (s, line_style) = if fill {
            ("█".repeat(inner.width as usize), style)
        } else {
            (" ".repeat(inner.width as usize), empty_style)
        };
        lines.push(Line::from(Span::styled(s, line_style)));
    }
    let paragraph = Paragraph::new(Text::from(lines));
    frame.render_widget(paragraph, inner);
}

/// Render the header with title and radio status
fn render_header(frame: &mut Frame, area: Rect, app: &App) {
    let (status_symbol, status_style) = match app.radio_state {
        RadioState::Disconnected => ("○", Style::default().fg(Color::Red)),
        RadioState::Idle => ("○", Style::default().fg(Color::Yellow)),
        RadioState::Receiving => ("●", Style::default().fg(Color::Green)),
        RadioState::Transmitting => ("●", Style::default().fg(Color::Red)),
    };

    let title = format!("Keyfob Analysis Toolkit v{}", VERSION);

    // Build radio info string: device name (if any), state, freq, gains
    let amp_str = if app.amp_enabled { "ON" } else { "OFF" };
    let device_str = app
        .radio_device_name()
        .unwrap_or("No device");
    let radio_info = format!(
        "{} {} | {} | {:.2} MHz | LNA:{} VGA:{} AMP:{}",
        status_symbol,
        app.radio_state,
        device_str,
        app.frequency_mhz(),
        app.lna_gain,
        app.vga_gain,
        amp_str
    );

    // Calculate padding for right-alignment
    let padding = area
        .width
        .saturating_sub(title.len() as u16 + radio_info.len() as u16 + 4);

    let header_line = Line::from(vec![
        Span::styled(title, Style::default().add_modifier(Modifier::BOLD)),
        Span::raw(" ".repeat(padding as usize)),
        Span::styled(radio_info, status_style),
    ]);

    let header = Paragraph::new(header_line).block(
        Block::default()
            .borders(Borders::ALL)
            .border_style(Style::default()),
    );

    frame.render_widget(header, area);
}

/// Render the context-sensitive help bar
fn render_help_bar(frame: &mut Frame, area: Rect, app: &App) {
    let help_text = match app.input_mode {
        InputMode::Normal => {
            "Enter: Actions | d: Delete | Tab: Settings | r: RX Toggle | :: Command | q: Quit"
        }
        InputMode::Command => "Enter: Execute | Esc: Cancel",
        InputMode::SignalMenu => "Up/Down: Navigate | Enter: Select | Esc: Close",
        InputMode::SettingsSelect => "Left/Right: Select | Tab: Cycle | Enter: Edit | Esc: Back",
        InputMode::SettingsEdit => "Up/Down: Change Value | Enter: Apply | Esc: Cancel",
        InputMode::HackRfNotDetected => "Press any key to continue",
        InputMode::StartupImport => "y: Import | n: Skip",
        InputMode::ExportFilename => {
            match app.export_format {
                Some(crate::app::ExportFormat::Fob) => "Enter: Next Field | Esc: Cancel Export",
                Some(crate::app::ExportFormat::Flipper) => "Enter: Save & Export | Esc: Cancel Export",
                None => "Enter: Confirm | Esc: Cancel",
            }
        }
        InputMode::FobMetaYear
        | InputMode::FobMetaMake
        | InputMode::FobMetaModel
        | InputMode::FobMetaRegion
        | InputMode::FobMetaCommand => "Enter: Next Field | Esc: Cancel Export",
        InputMode::FobMetaNotes => "Enter: Save & Export | Esc: Cancel Export",
        InputMode::CaptureMetaYear
        | InputMode::CaptureMetaMake
        | InputMode::CaptureMetaModel
        | InputMode::CaptureMetaRegion => "Enter: Next Field | Esc: Cancel",
        InputMode::CaptureMetaCommand => "Enter: Save | Esc: Cancel",
        InputMode::License | InputMode::Credits => "Esc/Enter: Close | Up/Down: Scroll",
        InputMode::LoadFileBrowser => "Up/Down: Navigate | Enter: Open/Import | Esc: Close",
    };

    let help = Paragraph::new(Line::from(Span::styled(
        format!(" {}", help_text),
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

const LOAD_BROWSER_VISIBLE_ROWS: usize = 16;

/// Render the :load file browser overlay (centered, list of dirs and .fob/.sub files).
fn render_load_file_browser(frame: &mut Frame, app: &App) {
    let area = frame.area();
    let popup_height = LOAD_BROWSER_VISIBLE_ROWS as u16 + 5;
    let popup_width = 56;
    let popup = centered_rect(popup_width, popup_height, area);

    frame.render_widget(Clear, popup);

    let path_str = app.load_browser_cwd.to_string_lossy();
    let path_display = if path_str.len() > popup_width as usize - 4 {
        format!("..{}", &path_str[path_str.len().saturating_sub(popup_width as usize - 5)..])
    } else {
        path_str.to_string()
    };

    let mut items: Vec<ListItem> = Vec::new();
    items.push(ListItem::new(Line::from(Span::styled(
        format!("  {}", path_display),
        Style::default().fg(Color::DarkGray),
    ))));
    items.push(ListItem::new(Line::from(Span::raw(""))));

    let entries = &app.load_browser_entries;
    let scroll = app.load_browser_scroll;
    let selected = app.load_browser_selected.min(entries.len().saturating_sub(1));
    let end = (scroll + LOAD_BROWSER_VISIBLE_ROWS).min(entries.len());

    for (i, (name, _path, is_dir)) in entries[scroll..end].iter().enumerate() {
        let idx = scroll + i;
        let is_selected = idx == selected;
        let prefix = if is_selected { " > " } else { "   " };
        let (style, suffix) = if *is_dir {
            (
                if is_selected {
                    Style::default().fg(Color::Black).bg(Color::Cyan).add_modifier(Modifier::BOLD)
                } else {
                    Style::default().fg(Color::Cyan)
                },
                "/",
            )
        } else {
            (
                if is_selected {
                    Style::default().fg(Color::Black).bg(Color::Cyan).add_modifier(Modifier::BOLD)
                } else {
                    Style::default().fg(Color::White)
                },
                "",
            )
        };
        items.push(ListItem::new(Line::from(Span::styled(
            format!("{}{}{}", prefix, name, suffix),
            style,
        ))));
    }

    let block = Block::default()
        .title(" Load file (.fob / .sub) ")
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Cyan));

    let list = List::new(items).block(block);
    frame.render_widget(list, popup);
}

/// Render the no-device warning (red box at startup when neither HackRF nor RTL-SDR found)
fn render_hackrf_not_detected(frame: &mut Frame, _app: &App) {
    let area = frame.area();
    let popup = centered_rect(56, 8, area);

    frame.render_widget(Clear, popup);

    let red = Color::Red;
    let text = vec![
        Line::from(""),
        Line::from(Span::styled(
            "  No HackRF or RTL-SDR detected.",
            Style::default().fg(red).add_modifier(Modifier::BOLD),
        )),
        Line::from(""),
        Line::from(Span::styled(
            "  Connect a HackRF (TX+RX) or RTL-SDR (RX only) and restart,",
            Style::default().fg(red),
        )),
        Line::from(Span::styled(
            "  or continue without TX/RX support.",
            Style::default().fg(red),
        )),
        Line::from(""),
        Line::from(Span::styled(
            "  Press any key to continue.",
            Style::default().fg(Color::White),
        )),
    ];

    let block = Block::default()
        .title(" Warning ")
        .borders(Borders::ALL)
        .border_style(Style::default().fg(red))
        .style(Style::default().fg(red));

    let paragraph = Paragraph::new(text)
        .block(block)
        .alignment(Alignment::Center)
        .wrap(Wrap { trim: true });

    frame.render_widget(paragraph, popup);
}

/// Render the startup import prompt overlay
fn render_startup_import_prompt(frame: &mut Frame, app: &App) {
    let count = app.pending_fob_files.len();
    let area = frame.area();
    let popup = centered_rect(50, 7, area);

    frame.render_widget(Clear, popup);

    let text = vec![
        Line::from(""),
        Line::from(Span::styled(
            format!(
                "Found {} file(s) (.fob / .sub) in import dir (incl. subfolders).",
                count
            ),
            Style::default().fg(Color::Yellow),
        )),
        Line::from(""),
        Line::from(Span::styled(
            "Import them? (y/n)",
            Style::default()
                .fg(Color::White)
                .add_modifier(Modifier::BOLD),
        )),
    ];

    let block = Block::default()
        .title(" Import Saved Signals ")
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Cyan));

    let paragraph = Paragraph::new(text)
        .block(block)
        .alignment(Alignment::Center)
        .wrap(Wrap { trim: true });

    frame.render_widget(paragraph, popup);
}

/// Render the export form overlay (filename + optional .fob metadata)
fn render_export_form(frame: &mut Frame, app: &App) {
    use crate::app::ExportFormat;

    let is_fob = app.export_format == Some(ExportFormat::Fob);
    let ext = if is_fob { ".fob" } else { ".sub" };

    let area = frame.area();
    // Taller popup for .fob (filename + 5 metadata fields), shorter for .sub (filename only)
    let popup_height = if is_fob { 21 } else { 11 };
    let popup = centered_rect(62, popup_height, area);

    frame.render_widget(Clear, popup);

    let active_style = Style::default()
        .fg(Color::Cyan)
        .add_modifier(Modifier::BOLD);
    let inactive_style = Style::default().fg(Color::DarkGray);
    let done_style = Style::default().fg(Color::Green);
    let value_style = Style::default().fg(Color::White);
    let dim_style = Style::default().fg(Color::DarkGray);
    let accent_style = Style::default().fg(Color::Yellow);
    let cursor = Span::styled(
        "_",
        Style::default()
            .fg(Color::Cyan)
            .add_modifier(Modifier::RAPID_BLINK),
    );

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

    let style_for = |idx: usize| -> Style {
        if idx == current_idx {
            active_style
        } else if idx < current_idx {
            done_style
        } else {
            inactive_style
        }
    };

    let mut lines = Vec::new();

    // --- Signal summary section ---
    if let Some(capture) = app
        .export_capture_id
        .and_then(|id| app.captures.iter().find(|c| c.id == id))
    {
        lines.push(Line::from(vec![
            Span::styled("  Signal:  ", dim_style),
            Span::styled(
                format!(
                    "#{:02} {} | {} | {} | 0x{}",
                    capture.id,
                    capture.protocol_name(),
                    capture.frequency_mhz(),
                    capture.modulation(),
                    capture.serial_hex(),
                ),
                accent_style,
            ),
        ]));
        lines.push(Line::from(vec![
            Span::styled("  Key:     ", dim_style),
            Span::styled(
                format!("0x{} ({})", capture.data_hex(), capture.encryption_type()),
                accent_style,
            ),
        ]));
    }

    lines.push(Line::from(Span::styled(
        "  ──────────────────────────────────────────────────────",
        dim_style,
    )));

    // --- Form fields ---
    struct FormField<'a> {
        label: &'a str,
        value: &'a str,
        placeholder: &'a str,
        idx: usize,
    }

    // Filename field (always present)
    let filename_display = format!("{}{}", app.export_filename, ext);
    let mut fields: Vec<FormField> = vec![
        FormField {
            label: "  File:    ",
            value: &filename_display,
            placeholder: "(enter filename)",
            idx: 0,
        },
    ];

    // .fob metadata fields
    if is_fob {
        fields.extend([
            FormField {
                label: "  Year:    ",
                value: &app.fob_meta_year,
                placeholder: "(e.g. 2024)",
                idx: 1,
            },
            FormField {
                label: "  Make:    ",
                value: &app.fob_meta_make,
                placeholder: "(auto-detected from protocol)",
                idx: 2,
            },
            FormField {
                label: "  Model:   ",
                value: &app.fob_meta_model,
                placeholder: "(e.g. Sportage, F-150)",
                idx: 3,
            },
            FormField {
                label: "  Region:  ",
                value: &app.fob_meta_region,
                placeholder: "(e.g. NA, EU, APAC, MEA)",
                idx: 4,
            },
            FormField {
                label: "  Command: ",
                value: &app.fob_meta_command,
                placeholder: "(e.g. Unlock, Lock, Trunk, Panic)",
                idx: 5,
            },
            FormField {
                label: "  Notes:   ",
                value: &app.fob_meta_notes,
                placeholder: "(optional — color, trim, VIN, etc.)",
                idx: 6,
            },
        ]);
    }

    for field in &fields {
        let label_s = style_for(field.idx);
        let display_val = if field.value.is_empty() {
            field.placeholder
        } else {
            field.value
        };

        let val_s = if field.value.is_empty() && field.idx != current_idx {
            dim_style
        } else {
            value_style
        };

        let mut spans = vec![
            Span::styled(field.label, label_s),
            Span::styled(display_val.to_string(), val_s),
        ];

        // Show cursor on active field
        if field.idx == current_idx {
            spans.push(cursor.clone());
        }

        // Show checkmark for completed fields with values
        if field.idx < current_idx && !field.value.is_empty() {
            spans.push(Span::styled(" ✓", done_style));
        }

        lines.push(Line::from(spans));
    }

    lines.push(Line::from(""));

    // Progress indicator
    let total_fields = fields.len();
    let progress = format!(
        "  Step {}/{}",
        current_idx + 1,
        total_fields,
    );
    let hint = if current_idx == total_fields - 1 {
        "  Enter: Save & Export | Esc: Cancel"
    } else {
        "  Enter: Next | Esc: Cancel"
    };
    lines.push(Line::from(vec![
        Span::styled(progress, accent_style),
        Span::styled("  ", dim_style),
        Span::styled(hint, dim_style),
    ]));

    let title = if is_fob {
        " Export .fob — Filename & Vehicle Details "
    } else {
        " Export .sub (Flipper Zero) "
    };

    let block = Block::default()
        .title(title)
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Green));

    let paragraph = Paragraph::new(lines).block(block);

    frame.render_widget(paragraph, popup);
}

/// Render the capture metadata form (Year/Make/Model/Region for vuln lookup). Shown when user presses 'i' on a capture.
fn render_capture_meta_form(frame: &mut Frame, app: &App) {
    let area = frame.area();
    let popup = centered_rect(62, 18, area);

    frame.render_widget(Clear, popup);

    let active_style = Style::default()
        .fg(Color::Cyan)
        .add_modifier(Modifier::BOLD);
    let inactive_style = Style::default().fg(Color::DarkGray);
    let done_style = Style::default().fg(Color::Green);
    let value_style = Style::default().fg(Color::White);
    let dim_style = Style::default().fg(Color::DarkGray);
    let accent_style = Style::default().fg(Color::Yellow);
    let cursor = Span::styled(
        "_",
        Style::default()
            .fg(Color::Cyan)
            .add_modifier(Modifier::RAPID_BLINK),
    );

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

    let style_for = |idx: usize| -> Style {
        if idx == current_idx {
            active_style
        } else if idx < current_idx {
            done_style
        } else {
            inactive_style
        }
    };

    let mut lines = Vec::new();

    if let Some(capture) = app
        .capture_meta_capture_id
        .and_then(|id| app.captures.iter().find(|c| c.id == id))
    {
        lines.push(Line::from(vec![
            Span::styled("  Signal:  ", dim_style),
            Span::styled(
                format!(
                    "#{:02} {} | {} | {} | 0x{}",
                    capture.id,
                    capture.protocol_name(),
                    capture.frequency_mhz(),
                    capture.modulation(),
                    capture.serial_hex(),
                ),
                accent_style,
            ),
        ]));
        lines.push(Line::from(vec![
            Span::styled("  Key:     ", dim_style),
            Span::styled(
                format!("0x{} ({})", capture.data_hex(), capture.encryption_type()),
                accent_style,
            ),
        ]));
    }

    lines.push(Line::from(Span::styled(
        "  ──────────────────────────────────────────────────────",
        dim_style,
    )));

    struct FormField<'a> {
        label: &'a str,
        value: &'a str,
        placeholder: &'a str,
        idx: usize,
    }
    let fields: [FormField; 5] = [
        FormField {
            label: "  Year:    ",
            value: &app.capture_meta_year,
            placeholder: "(e.g. 2021)",
            idx: 0,
        },
        FormField {
            label: "  Make:    ",
            value: &app.capture_meta_make,
            placeholder: "(e.g. Renault, Honda)",
            idx: 1,
        },
        FormField {
            label: "  Model:   ",
            value: &app.capture_meta_model,
            placeholder: "(e.g. ZOE, Civic — or ALL)",
            idx: 2,
        },
        FormField {
            label: "  Region:  ",
            value: &app.capture_meta_region,
            placeholder: "(e.g. NA, EU, or ALL)",
            idx: 3,
        },
        FormField {
            label: "  Command: ",
            value: &app.capture_meta_command,
            placeholder: "(e.g. Unlock, Lock, Trunk, Panic)",
            idx: 4,
        },
    ];

    for field in &fields {
        let label_s = style_for(field.idx);
        let display_val = if field.value.is_empty() {
            field.placeholder
        } else {
            field.value
        };
        let val_s = if field.value.is_empty() && field.idx != current_idx {
            dim_style
        } else {
            value_style
        };
        let mut spans = vec![
            Span::styled(field.label, label_s),
            Span::styled(display_val.to_string(), val_s),
        ];
        if field.idx == current_idx {
            spans.push(cursor.clone());
        }
        if field.idx < current_idx && !field.value.is_empty() {
            spans.push(Span::styled(" ✓", done_style));
        }
        lines.push(Line::from(spans));
    }

    lines.push(Line::from(""));
    let progress = format!("  Step {}/4 — Used for vuln lookup and .fob export", current_idx + 1);
    lines.push(Line::from(Span::styled(progress, dim_style)));

    let block = Block::default()
        .title(" Capture Metadata — Year / Make / Model / Region ")
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::Cyan));

    let paragraph = Paragraph::new(lines).block(block);
    frame.render_widget(paragraph, popup);
}
