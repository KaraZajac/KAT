//! Main UI layout.

use ratatui::{
    layout::{Alignment, Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Clear, Paragraph, Wrap},
    Frame,
};

use crate::app::{App, InputMode, RadioState};

use super::captures_list::render_captures_list;
use super::command::render_command_line;
use super::settings_menu::{render_settings_dropdown, render_settings_tabs};
use super::signal_menu::render_signal_menu;
use super::status_bar::render_status_bar;

use crate::app::InputMode as IM;

const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Draw the entire UI
pub fn draw_ui(frame: &mut Frame, app: &App) {
    let show_settings = matches!(app.input_mode, IM::SettingsSelect | IM::SettingsEdit);
    let show_command = app.input_mode == IM::Command;

    let mut constraints = vec![Constraint::Length(3)]; // Header

    if show_settings {
        constraints.push(Constraint::Length(3)); // Settings tabs
    }

    constraints.push(Constraint::Min(8)); // Captures list
    constraints.push(Constraint::Length(3)); // Status bar

    if show_command {
        constraints.push(Constraint::Length(3)); // Command input
    }

    constraints.push(Constraint::Length(1)); // Help bar

    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints(constraints)
        .split(frame.area());

    let mut idx = 0;
    render_header(frame, chunks[idx], app);
    idx += 1;

    if show_settings {
        render_settings_tabs(frame, chunks[idx], app);
        idx += 1;
    }

    render_captures_list(frame, chunks[idx], app);
    idx += 1;

    render_status_bar(frame, chunks[idx], app);
    idx += 1;

    if show_command {
        render_command_line(frame, chunks[idx], app);
        idx += 1;
    }

    render_help_bar(frame, chunks[idx], app);

    // Overlay widgets (rendered on top of everything else)
    if app.input_mode == InputMode::SignalMenu {
        render_signal_menu(frame, app);
    }

    if app.input_mode == InputMode::SettingsEdit {
        render_settings_dropdown(frame, app);
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
            | InputMode::FobMetaNotes
    ) {
        render_export_form(frame, app);
    }
}

/// Render the header with title and radio status
fn render_header(frame: &mut Frame, area: Rect, app: &App) {
    let (status_symbol, status_style) = match app.radio_state {
        RadioState::Disconnected => ("○", Style::default().fg(Color::DarkGray)),
        RadioState::Idle => ("○", Style::default().fg(Color::Yellow)),
        RadioState::Receiving => ("●", Style::default().fg(Color::Green)),
        RadioState::Transmitting => ("●", Style::default().fg(Color::Red)),
    };

    let title = format!("Keyfob Analysis Toolkit v{}", VERSION);

    // Build radio info string with all settings
    let amp_str = if app.amp_enabled { "ON" } else { "OFF" };
    let radio_info = format!(
        "{} {} | {:.2} MHz | LNA:{} VGA:{} AMP:{}",
        status_symbol,
        app.radio_state,
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
        | InputMode::FobMetaRegion => "Enter: Next Field | Esc: Cancel Export",
        InputMode::FobMetaNotes => "Enter: Save & Export | Esc: Cancel Export",
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
                label: "  Notes:   ",
                value: &app.fob_meta_notes,
                placeholder: "(optional — color, trim, VIN, etc.)",
                idx: 5,
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
