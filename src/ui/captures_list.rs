//! Captures list widget with detail panel.

use ratatui::{
    layout::{Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Cell, Paragraph, Row, Table, TableState, Wrap},
    Frame,
};

use crate::app::App;
use crate::capture::CaptureStatus;
use crate::vuln_db;

/// Render the captures area: table + detail panel
pub fn render_captures_list(frame: &mut Frame, area: Rect, app: &App) {
    // Split vertically: table on top, detail panel on bottom
    let has_selection = app
        .selected_capture
        .map(|i| i < app.captures.len())
        .unwrap_or(false);

    let chunks = if has_selection {
        Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Min(6),     // Table (flexible, takes remaining)
                Constraint::Length(12), // Detail panel (fixed height)
            ])
            .split(area)
    } else {
        Layout::default()
            .direction(Direction::Vertical)
            .constraints([Constraint::Min(6)])
            .split(area)
    };

    render_table(frame, chunks[0], app);

    if has_selection && chunks.len() > 1 {
        render_detail_panel(frame, chunks[1], app);
    }
}

/// Render the compact signal table
fn render_table(frame: &mut Frame, area: Rect, app: &App) {
    let header_cells = [
        "ID", "Time", "Protocol", "Freq", "Serial", "Btn", "Cnt", "Modulation", "CRC", "Status",
        "Vuln Found",
    ]
    .iter()
    .map(|h| Cell::from(*h).style(Style::default().add_modifier(Modifier::BOLD)));

    let header = Row::new(header_cells).style(Style::default()).height(1);

    let rows = app.captures.iter().map(|capture| {
        let status_style = match capture.status {
            CaptureStatus::Unknown => Style::default().fg(Color::DarkGray),
            CaptureStatus::Decoded => Style::default().fg(Color::Yellow),
            CaptureStatus::EncoderCapable => Style::default().fg(Color::Green),
        };

        let crc_style = if capture.protocol.is_none() {
            Style::default().fg(Color::DarkGray)
        } else if capture.crc_valid {
            Style::default().fg(Color::Green)
        } else {
            Style::default().fg(Color::Red)
        };

        let mod_style = match capture.modulation() {
            crate::capture::ModulationType::Pwm => Style::default().fg(Color::Magenta),
            crate::capture::ModulationType::Manchester => Style::default().fg(Color::Cyan),
            crate::capture::ModulationType::DifferentialManchester => {
                Style::default().fg(Color::Blue)
            }
            crate::capture::ModulationType::Unknown => Style::default().fg(Color::DarkGray),
        };

        let status_text = match capture.status {
            CaptureStatus::EncoderCapable => "Encode",
            CaptureStatus::Decoded => "Decoded",
            CaptureStatus::Unknown => "Unknown",
        };

        let vuln_found = capture.status == CaptureStatus::EncoderCapable
            || !vuln_db::match_vulns(
                capture.year.as_deref(),
                capture.make.as_deref(),
                capture.model.as_deref(),
                capture.region.as_deref(),
            )
            .is_empty();
        let vuln_text = if vuln_found { "Yes" } else { "No" };
        let vuln_style = if vuln_found {
            Style::default().fg(Color::Green)
        } else {
            Style::default().fg(Color::DarkGray)
        };

        Row::new(vec![
            Cell::from(format!("{:02}", capture.id)),
            Cell::from(capture.timestamp_short()),
            Cell::from(capture.protocol_name().to_string()),
            Cell::from(capture.frequency_mhz()),
            Cell::from(capture.serial_hex()),
            Cell::from(capture.button_name().to_string()),
            Cell::from(capture.counter_str()),
            Cell::from(capture.modulation().to_string()).style(mod_style),
            Cell::from(capture.crc_status()).style(crc_style),
            Cell::from(status_text).style(status_style),
            Cell::from(vuln_text).style(vuln_style),
        ])
        .height(1)
    });

    let widths = [
        Constraint::Length(4),  // ID
        Constraint::Length(9),  // Time
        Constraint::Length(24), // Protocol (e.g. KeeLoq (DoorHan))
        Constraint::Length(11), // Freq
        Constraint::Length(9),  // Serial
        Constraint::Length(6),  // Btn
        Constraint::Length(6),  // Cnt
        Constraint::Length(12), // Modulation
        Constraint::Length(5),  // CRC
        Constraint::Length(10), // Status
        Constraint::Length(10), // Vuln Found
    ];

    let table = Table::new(rows, widths)
        .header(header)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .title(" Captures "),
        )
        .row_highlight_style(Style::default().add_modifier(Modifier::REVERSED));

    let mut state = TableState::default();
    state.select(app.selected_capture);

    // Apply scroll offset if needed
    if app.scroll_offset > 0 && app.selected_capture.is_some() {
        *state.offset_mut() = app.scroll_offset;
    }

    frame.render_stateful_widget(table, area, &mut state);
}

/// Render the detail panel for the selected signal (left = signal info, right = vulnerability)
fn render_detail_panel(frame: &mut Frame, area: Rect, app: &App) {
    let capture = match app.selected_capture {
        Some(idx) if idx < app.captures.len() => &app.captures[idx],
        _ => return,
    };

    let halves = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
        .split(area);

    render_signal_detail(frame, halves[0], capture);
    render_vulnerability_panel(frame, halves[1], capture);
}

/// Left half: signal information
fn render_signal_detail(frame: &mut Frame, area: Rect, capture: &crate::capture::Capture) {
    let label_style = Style::default()
        .fg(Color::DarkGray)
        .add_modifier(Modifier::BOLD);
    let value_style = Style::default().fg(Color::White);
    let accent_style = Style::default().fg(Color::Cyan);
    let good_style = Style::default().fg(Color::Green);
    let bad_style = Style::default().fg(Color::Red);

    let make = App::get_make_for_protocol(capture.protocol_name());

    let mut left_lines = Vec::new();

    // Row 1: Protocol + Make
    left_lines.push(Line::from(vec![
        Span::styled(" Protocol:  ", label_style),
        Span::styled(capture.protocol_name(), accent_style),
        Span::styled("  Make: ", label_style),
        Span::styled(make, value_style),
    ]));

    // Row 2: Freq + Mod + RF
    left_lines.push(Line::from(vec![
        Span::styled(" Freq:      ", label_style),
        Span::styled(capture.frequency_mhz(), value_style),
        Span::styled("  Mod: ", label_style),
        Span::styled(capture.modulation().to_string(), value_style),
        Span::styled("  RF: ", label_style),
        Span::styled(capture.rf_modulation().to_string(), value_style),
    ]));

    // Row 3: Enc + Rx (demodulator path when known)
    let mut row3 = vec![
        Span::styled(" Enc:       ", label_style),
        Span::styled(capture.encryption_type(), value_style),
    ];
    if let Some(rf) = capture.received_rf {
        row3.push(Span::styled("  Rx: ", label_style));
        row3.push(Span::styled(rf.to_string(), value_style));
    }
    left_lines.push(Line::from(row3));

    // Row 4: Full Serial + Button
    left_lines.push(Line::from(vec![
        Span::styled(" Serial:    ", label_style),
        Span::styled(format!("0x{}", capture.serial_hex()), accent_style),
        Span::styled("  Btn: ", label_style),
        Span::styled(
            format!("{} ({})", capture.button_name(), capture.button_hex()),
            value_style,
        ),
    ]));

    // Row 5: Counter + CRC
    let crc_span = if capture.protocol.is_none() {
        Span::styled("-", Style::default().fg(Color::DarkGray))
    } else if capture.crc_valid {
        Span::styled("OK ✓", good_style)
    } else {
        Span::styled("FAIL ✗", bad_style)
    };

    left_lines.push(Line::from(vec![
        Span::styled(" Counter:   ", label_style),
        Span::styled(capture.counter_str(), value_style),
        Span::styled("  CRC: ", label_style),
        crc_span,
        Span::styled("  Status: ", label_style),
        Span::styled(capture.status.to_string(), value_style),
    ]));

    // Row 6: Full data/key hex
    left_lines.push(Line::from(vec![
        Span::styled(" Key/Data:  ", label_style),
        Span::styled(
            format!("0x{}", capture.data_hex()),
            Style::default().fg(Color::Yellow),
        ),
        Span::styled(
            format!("  ({})", capture.data_bits_str()),
            Style::default().fg(Color::DarkGray),
        ),
    ]));

    // Row 7: Timestamp + Raw data info
    let raw_info = if capture.has_raw_data() {
        format!("✓ {} transitions", capture.raw_pair_count())
    } else {
        "None".to_string()
    };
    let raw_style = if capture.has_raw_data() {
        good_style
    } else {
        Style::default().fg(Color::DarkGray)
    };

    left_lines.push(Line::from(vec![
        Span::styled(" Captured:  ", label_style),
        Span::styled(capture.timestamp_full(), value_style),
    ]));

    left_lines.push(Line::from(vec![
        Span::styled(" Raw Data:  ", label_style),
        Span::styled(raw_info, raw_style),
    ]));

    // Row 8: File path (imported .sub/.fob only; blank for live captures)
    let file_display = capture
        .source_file
        .as_deref()
        .unwrap_or("");
    left_lines.push(Line::from(vec![
        Span::styled(" File:      ", label_style),
        Span::styled(
            file_display,
            if file_display.is_empty() {
                Style::default().fg(Color::DarkGray)
            } else {
                value_style
            },
        ),
    ]));

    // Build the title
    let title = format!(
        " Signal #{:02} — {} ",
        capture.id,
        capture.protocol_name()
    );

    let border_style = match capture.status {
        CaptureStatus::EncoderCapable => Style::default().fg(Color::Green),
        CaptureStatus::Decoded => Style::default().fg(Color::Yellow),
        CaptureStatus::Unknown => Style::default().fg(Color::DarkGray),
    };

    let detail = Paragraph::new(left_lines)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .border_style(border_style)
                .title(title),
        )
        .wrap(Wrap { trim: false });

    frame.render_widget(detail, area);
}

/// Right half: vulnerability panel (matching CVEs or prompt to set Year/Make/Model)
fn render_vulnerability_panel(
    frame: &mut Frame,
    area: Rect,
    capture: &crate::capture::Capture,
) {
    let label_style = Style::default()
        .fg(Color::DarkGray)
        .add_modifier(Modifier::BOLD);
    let value_style = Style::default().fg(Color::White);
    let accent_style = Style::default().fg(Color::Cyan);

    let vulns = vuln_db::match_vulns(
        capture.year.as_deref(),
        capture.make.as_deref(),
        capture.model.as_deref(),
        capture.region.as_deref(),
    );
    let vuln_found = capture.status == CaptureStatus::EncoderCapable || !vulns.is_empty();

    let mut lines = Vec::new();

    // When we can encode, the encryption is broken — complete emulation is available.
    if capture.status == CaptureStatus::EncoderCapable {
        let emu_style = Style::default()
            .fg(Color::Green)
            .add_modifier(Modifier::BOLD);
        lines.push(Line::from(Span::styled(
            " Encryption is broken — Complete emulation of the keyfob is available",
            emu_style,
        )));
        lines.push(Line::from(Span::raw("")));
    }

    if vulns.is_empty() {
        let has_meta = capture.year.is_some()
            || capture.make.is_some()
            || capture.model.is_some()
            || capture.region.is_some();
        if has_meta {
            lines.push(Line::from(Span::styled(
                " No matching CVE in database.",
                value_style,
            )));
        } else {
            lines.push(Line::from(Span::styled(
                " Set Year, Make, Model, Region",
                value_style,
            )));
            lines.push(Line::from(Span::styled(
                " (press i) to check vulnerabilities.",
                value_style,
            )));
        }
    } else {
        for v in vulns {
            lines.push(Line::from(vec![
                Span::styled(" CVE: ", label_style),
                Span::styled(v.cve, accent_style),
            ]));
            lines.push(Line::from(vec![
                Span::styled(" Description: ", label_style),
                Span::styled(v.description, value_style),
            ]));
            lines.push(Line::from(vec![
                Span::styled(" Source: ", label_style),
                Span::styled(v.url, value_style),
            ]));
            lines.push(Line::from(Span::raw("")));
        }
    }

    let border_color = if vuln_found { Color::Green } else { Color::Yellow };
    let block = Block::default()
        .borders(Borders::ALL)
        .border_style(Style::default().fg(border_color))
        .title(" Vulnerability ");
    let inner = block.inner(area);
    frame.render_widget(block, area);
    if inner.width > 0 && inner.height > 0 {
        let para = Paragraph::new(lines).wrap(Wrap { trim: false });
        frame.render_widget(para, inner);
    }
}
