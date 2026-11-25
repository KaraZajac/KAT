//! Captures list widget with detail panel.

use ratatui::{
    layout::{Constraint, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Cell, Paragraph, Row, Table, TableState},
    Frame,
};

use crate::app::{App, Column, RadioState, Sort};
use crate::capture::{Capture, CaptureStatus, KeyRecovery, ModulationType};
use crate::protocols::hitag2_keys::{self, ShownKey};
use crate::radio::hopper::format_mhz;
use crate::saved_fobs::SavedFobMatch;
use crate::vuln_db;

use super::text;
use super::theme;

/// Colour of the saved-fob marker (★ in the ID column, "Saved:" in the detail panel).
const SAVED_COLOR: Color = Color::Magenta;

/// Table height kept before the detail panel gets any: borders, header and five captures.
const TABLE_MIN_HEIGHT: u16 = 8;
/// Detail panel height at most (room for several CVEs) and the least worth drawing.
const DETAIL_MAX_HEIGHT: u16 = 18;
const DETAIL_MIN_HEIGHT: u16 = 6;
/// Height of the captures area from which the detail panel shows: a status bar that grows for a
/// long message leaves the area that much, so the panel does not come and go with the message.
pub(super) const DETAIL_FLOOR: u16 = TABLE_MIN_HEIGHT + DETAIL_MIN_HEIGHT;

/// Render the captures area: table + detail panel
pub fn render_captures_list(frame: &mut Frame, area: Rect, app: &App) {
    let selected = app.selected_capture.and_then(|i| app.captures.get(i));
    let wanted = selected.map(|c| {
        detail_height(area.width, c, app.saved_match(c.id), app.key_search_planned(c.id), app.hitag2_shown_key(c))
    });
    let (table_area, detail_area) = split_captures_area(area, wanted);
    render_table(frame, table_area, app);
    if let Some(detail_area) = detail_area {
        render_detail_panel(frame, detail_area, app);
    }
}

/// Signal detail on the left, vulnerabilities on the right: half each, but the signal detail gets
/// more of a narrow terminal so its rows fit.
fn detail_halves(area: Rect) -> [Rect; 2] {
    let signal_width = (area.width / 2).max(SIGNAL_PREF_WIDTH.min(area.width.saturating_sub(VULN_MIN_WIDTH)));
    Layout::horizontal([Constraint::Length(signal_width), Constraint::Fill(1)]).areas(area)
}

/// Rows the detail panel takes to show all it has on `capture` in a list `width` columns wide: the
/// taller of its halves and the borders. The captures of one remote take the same, so the table
/// above keeps its height while the selection moves through them.
fn detail_height(width: u16, capture: &Capture, saved: Option<&SavedFobMatch>, searching: bool, hitag2: Option<ShownKey>) -> u16 {
    let [left, right] = detail_halves(Rect::new(0, 0, width, DETAIL_MAX_HEIGHT));
    let signal = signal_detail_lines(capture, saved, searching, hitag2, left.width.saturating_sub(2) as usize).len();
    let vulns = vulnerability_lines(capture, right.width.saturating_sub(2) as usize, usize::MAX).len();
    (signal.max(vulns) as u16).saturating_add(2)
}

/// Table on top, detail panel below. The panel takes the `wanted` rows of the selected capture
/// (None: nothing is selected) up to its full height, less when the table would be left with
/// fewer than a few rows, and is left out when even its smallest useful height does not fit.
fn split_captures_area(area: Rect, wanted: Option<u16>) -> (Rect, Option<Rect>) {
    let detail = match wanted {
        Some(rows) => rows.min(DETAIL_MAX_HEIGHT).min(area.height.saturating_sub(TABLE_MIN_HEIGHT)),
        None => 0,
    };
    if detail < DETAIL_MIN_HEIGHT {
        return (area, None);
    }
    let [table, panel] = Layout::vertical([Constraint::Fill(1), Constraint::Length(detail)]).areas(area);
    (table, Some(panel))
}

/// Columns of the captures table, in display order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Col {
    Id,
    Time,
    Protocol,
    Freq,
    /// The radio that heard it (shown with several radios: [App::show_radio_column]).
    Radio,
    Serial,
    Btn,
    Cnt,
    Mod,
    Crc,
    Status,
    Vuln,
}

impl Col {
    const ALL: [Col; 12] = [
        Col::Id,
        Col::Time,
        Col::Protocol,
        Col::Freq,
        Col::Radio,
        Col::Serial,
        Col::Btn,
        Col::Cnt,
        Col::Mod,
        Col::Crc,
        Col::Status,
        Col::Vuln,
    ];

    /// The column as the list's filter and sort know it (app/list_view.rs): its name and its
    /// text for a capture come from there.
    fn column(self) -> Column {
        match self {
            Col::Id => Column::Id,
            Col::Time => Column::Time,
            Col::Protocol => Column::Protocol,
            Col::Freq => Column::Freq,
            Col::Radio => Column::Radio,
            Col::Serial => Column::Serial,
            Col::Btn => Column::Btn,
            Col::Cnt => Column::Cnt,
            Col::Mod => Column::Mod,
            Col::Crc => Column::Crc,
            Col::Status => Column::Status,
            Col::Vuln => Column::Vuln,
        }
    }

    fn header(self) -> &'static str {
        self.column().title()
    }

    /// The header as shown: the column the list is sorted by carries the direction ("Serial▲":
    /// upwards, "Serial▼": downwards). The order of arrival, the ID upwards, is not marked.
    fn header_shown(self, sort: Sort) -> String {
        let mark = match sort.descending {
            _ if sort.column != self.column() || sort.is_arrival_order() => "",
            false => "▲",
            true => "▼",
        };
        format!("{}{mark}", self.header())
    }
}

/// Columns left out first, one at a time, when the table is too narrow for all of them.
const DROP_ORDER: [Col; 6] = [Col::Time, Col::Radio, Col::Vuln, Col::Mod, Col::Status, Col::Crc];
/// Room the Protocol column keeps before columns are left out (less when no name is that long).
const PROTOCOL_MIN: u16 = 20;
/// Room the Protocol column takes beyond its longest name (or [PROTOCOL_MIN]) when the table has
/// more: on a wide terminal it took all of it, and a name stood 100 columns from its frequency.
/// What is left stays at the end of the rows.
const PROTOCOL_SPARE: u16 = 4;
/// Widest Btn column; longer button names are cut with "…".
const BTN_MAX: u16 = 14;

/// The columns to show and their widths, plus the spacing between them, for a table `width`
/// columns wide, from each column's content width (header included): every column with the
/// widest spacing that fits, else the columns of [DROP_ORDER] left out until the Protocol column
/// has its room. The Protocol column takes what is left, up to its longest name and
/// [PROTOCOL_SPARE] more.
fn choose_columns(width: u16, content: &[(Col, u16)]) -> (Vec<(Col, u16)>, u16) {
    let fixed = |cols: &[(Col, u16)], spacing: u16| -> u16 {
        let widths: u16 = cols.iter().filter(|(c, _)| *c != Col::Protocol).map(|&(_, w)| w).sum();
        widths + spacing * (cols.len() as u16).saturating_sub(1)
    };
    let protocol = content.iter().find(|(c, _)| *c == Col::Protocol).map_or(0, |&(_, w)| w);
    let mut cols = content.to_vec();
    let spacing = match [3, 2, 1].into_iter().find(|&s| fixed(&cols, s) + protocol <= width) {
        Some(s) => s,
        None => {
            let need = protocol.min(PROTOCOL_MIN);
            for drop in DROP_ORDER {
                if fixed(&cols, 1) + need <= width {
                    break;
                }
                cols.retain(|(c, _)| *c != drop);
            }
            1
        }
    };
    let rest = width.saturating_sub(fixed(&cols, spacing));
    let wanted = protocol.max(PROTOCOL_MIN) + PROTOCOL_SPARE;
    for (c, w) in &mut cols {
        if *c == Col::Protocol {
            *w = rest.min(wanted);
        }
    }
    (cols, spacing)
}

/// One capture's table row: the text of each column (in [Col::ALL] order), whether it matches a
/// saved .fob, and whether it has a known vulnerability.
struct RowText {
    cells: [String; 12],
    saved: bool,
    vuln: bool,
}

fn row_text(capture: &Capture, saved: bool) -> RowText {
    // The text the filter looks in (app/list_view.rs), so what is seen is what is found.
    let cells = Col::ALL.map(|col| match col {
        // ★ = matches a saved .fob (protocol + serial/key + counter within ±50)
        Col::Id => format!("{:02}{}", capture.id, if saved { "★" } else { "" }),
        _ => col.column().text(capture),
    });
    RowText { cells, saved, vuln: capture.vuln_found() }
}

/// A table cell: its text cut to the column with "…", in the column's colour.
fn table_cell(col: Col, row: &RowText, capture: &Capture, width: u16) -> Cell<'static> {
    let content = &row.cells[Col::ALL.iter().position(|&c| c == col).unwrap_or(0)];
    let style = match col {
        Col::Mod => match capture.modulation() {
            ModulationType::Pwm => Style::default().fg(Color::Magenta),
            ModulationType::Manchester => Style::default().fg(Color::Cyan),
            ModulationType::DifferentialManchester => Style::default().fg(Color::Blue),
            ModulationType::Nrz => Style::default().fg(Color::Green),
            ModulationType::Miller => Style::default().fg(Color::Yellow),
            ModulationType::Unknown => Style::default().fg(Color::DarkGray),
        },
        Col::Crc if !capture.crc_checked() => Style::default().fg(Color::DarkGray),
        Col::Crc if capture.crc_valid => Style::default().fg(Color::Green),
        Col::Crc => Style::default().fg(Color::Red),
        Col::Status => match capture.status {
            _ if capture.own_tx.is_some() => Style::default().fg(Color::Cyan),
            CaptureStatus::Unknown => Style::default().fg(Color::DarkGray),
            CaptureStatus::Decoded => Style::default().fg(Color::Yellow),
            CaptureStatus::EncoderCapable => Style::default().fg(Color::Green),
        },
        Col::Vuln if row.vuln => Style::default().fg(Color::Green),
        Col::Vuln => Style::default().fg(Color::DarkGray),
        _ => theme::TEXT,
    };
    if col == Col::Id && row.saved {
        return Cell::from(Line::from(vec![
            Span::raw(format!("{:02}", capture.id)),
            Span::styled("★", Style::default().fg(SAVED_COLOR)),
        ]));
    }
    Cell::from(text::truncate_end(content, width as usize)).style(style)
}

/// The table's title: " Captures ", and under a filter or a sort what the list shows:
/// " Captures · 3 of 15 · filter: acme · sort: Serial ▼ ". For a box `width` columns wide: a long
/// filter is cut, and the sort (the header marks it too) goes before the count does.
fn list_title(shown: usize, count: usize, filter: &str, sort: Sort, width: usize) -> String {
    let filter = filter.trim();
    let room = width.saturating_sub(2);
    let sorted = (!sort.is_arrival_order())
        .then(|| format!("sort: {} {}", sort.column.title(), if sort.descending { "▼" } else { "▲" }));
    let title = |filter_room: Option<usize>, sorted: Option<&String>| {
        let mut parts = vec!["Captures".to_string()];
        if !filter.is_empty() {
            parts.push(format!("{shown} of {count}"));
            if let Some(room) = filter_room {
                parts.push(format!("filter: {}", text::truncate_end(filter, room)));
            }
        }
        parts.extend(sorted.cloned());
        format!(" {} ", parts.join(" · "))
    };
    if filter.is_empty() {
        return title(None, sorted.as_ref());
    }
    // The filter's text gets what the other parts leave, a few columns at least.
    const FILTER_MIN: usize = 6;
    for sorted in [sorted.as_ref(), None] {
        let fixed = text::width(&title(Some(0), sorted));
        if fixed + FILTER_MIN.min(text::width(filter)) <= room {
            return title(Some(room - fixed), sorted);
        }
    }
    title(None, None)
}

/// Render the compact signal table: the captures the list's filter keeps, in the order of its
/// sort (app/list_view.rs).
fn render_table(frame: &mut Frame, area: Rect, app: &App) {
    let view = app.visible_captures();
    let sort = app.list_view.sort;
    let filter = app.list_view.filter.as_str();
    let title = list_title(view.len(), app.captures.len(), filter, sort, area.width as usize);
    let block = Block::default().borders(Borders::ALL).title(title);
    let inner = block.inner(area);

    let rows: Vec<RowText> = app
        .captures
        .iter()
        .map(|c| row_text(c, app.saved_match(c.id).is_some()))
        .collect();
    // Content-sized columns: each as wide as its header and its widest value (the Radio column
    // only with several radios). Of every capture, not only the shown ones: the columns stay
    // where they are while a filter is typed.
    let show_radio = app.show_radio_column();
    let content: Vec<(Col, u16)> = Col::ALL
        .iter()
        .enumerate()
        .filter(|&(_, &col)| col != Col::Radio || show_radio)
        .map(|(i, &col)| {
            let widest = rows.iter().map(|r| text::width(&r.cells[i])).max().unwrap_or(0);
            let w = widest.max(text::width(&col.header_shown(sort))) as u16;
            (col, if col == Col::Btn { w.min(BTN_MAX) } else { w })
        })
        .collect();
    let (columns, spacing) = choose_columns(inner.width, &content);

    let header = Row::new(columns.iter().map(|&(col, _)| {
        Cell::from(col.header_shown(sort)).style(Style::default().add_modifier(Modifier::BOLD))
    }))
    .height(1);
    let table_rows = view.iter().map(|&i| {
        Row::new(columns.iter().map(|&(col, w)| table_cell(col, &rows[i], &app.captures[i], w))).height(1)
    });
    let widths: Vec<Constraint> = columns.iter().map(|&(_, w)| Constraint::Length(w)).collect();

    let table = Table::new(table_rows, widths)
        .header(header)
        .column_spacing(spacing)
        .block(block)
        .row_highlight_style(theme::HIGHLIGHT);

    // Start from where the table was last frame, kept so the view is filled (no blank rows under
    // the last capture while older ones are scrolled off, e.g. after a delete or in a taller
    // window); ratatui then scrolls just enough to keep the selection in view for this height.
    let visible_rows = inner.height.saturating_sub(1) as usize;
    let offset = app
        .captures_table_offset
        .get()
        .min(view.len().saturating_sub(visible_rows));
    let selected_row = app.selected_capture.and_then(|s| view.iter().position(|&i| i == s));
    let mut state = TableState::new()
        .with_offset(offset)
        .with_selected(selected_row);
    frame.render_stateful_widget(table, area, &mut state);
    app.captures_table_offset.set(state.offset());
    // Captures out of view: how many, above and below, on the bottom border (as the menus and
    // the file browser say it). Drawn after the table: only then is the offset known.
    if let Some(hint) = text::more_hint(state.offset(), visible_rows, view.len()) {
        let width = text::width(&hint) as u16;
        if area.width > width + 2 && area.height > 2 {
            frame.buffer_mut().set_string(area.right() - 1 - width, area.bottom() - 1, &hint, Style::default());
        }
    }
    // PgUp / PgDn move by what the table shows.
    app.list_page_rows.set(visible_rows.max(1));

    if view.is_empty() && inner.height >= 2 {
        let hint = if app.captures.is_empty() {
            let receiving = app.radios().iter().any(|r| r.state == RadioState::Receiving);
            empty_hint(app.radio_state() != RadioState::Disconnected, receiving)
        } else {
            NO_MATCH_HINT
        };
        // Under the header, after a blank row when there is room for it.
        let y = inner.y + if inner.height >= 3 { 2 } else { 1 };
        let row = Rect::new(inner.x, y, inner.width, 1);
        frame.render_widget(
            Paragraph::new(format!(" {}", text::truncate_end(hint, (row.width as usize).saturating_sub(1))))
                .style(Style::default().fg(Color::DarkGray)),
            row,
        );
    }
}

/// What the list says when its filter fits no capture.
const NO_MATCH_HINT: &str = "No capture fits the filter — press / to change it, Esc to clear it";

/// What an empty list says: the way to a first capture, by what the radios do. While one
/// receives it is the remote's turn ("press r" would stop the radio).
fn empty_hint(connected: bool, receiving: bool) -> &'static str {
    if receiving {
        "No captures yet — receiving: press a button on the remote"
    } else if connected {
        "No captures yet — press r to receive, or :load a file"
    } else {
        "No captures yet — :load a file, or :new to add a remote"
    }
}

/// Width the signal detail asks for so its rows rarely wrap, and the least the vulnerability
/// panel next to it keeps.
const SIGNAL_PREF_WIDTH: u16 = 54;
const VULN_MIN_WIDTH: u16 = 26;

/// Render the detail panel for the selected signal (left = signal info, right = vulnerability)
fn render_detail_panel(frame: &mut Frame, area: Rect, app: &App) {
    let capture = match app.selected_capture {
        Some(idx) if idx < app.captures.len() => &app.captures[idx],
        _ => return,
    };

    let [left, right] = detail_halves(area);

    render_signal_detail(
        frame,
        left,
        capture,
        app.saved_match(capture.id),
        app.key_search_planned(capture.id),
        app.hitag2_shown_key(capture),
    );
    render_vulnerability_panel(frame, right, capture);
}

fn label_style() -> Style {
    Style::default().fg(Color::DarkGray).add_modifier(Modifier::BOLD)
}

/// Column where the values of the signal detail start (after " Protocol:  ").
const VALUE_COL: usize = 12;

/// "Label:" and value spans of one field of the signal detail.
type Field = (&'static str, Vec<Span<'static>>);

/// One row of the signal detail: the first label padded to the value column, then the other
/// fields while they fit on the line, else on the next line under the values, so a field is never
/// split. A line still too wide is cut with "…".
fn detail_row(fields: Vec<Field>, max: usize) -> Vec<Line<'static>> {
    let mut lines = Vec::new();
    let mut cur: Vec<Span<'static>> = Vec::new();
    let mut used = 0;
    for (i, (label, value)) in fields.into_iter().enumerate() {
        let value_w: usize = value.iter().map(Span::width).sum();
        let head = if i == 0 {
            format!(" {label:<w$}", w = VALUE_COL - 1)
        } else if used + 2 + text::width(label) + 1 + value_w <= max {
            format!("  {label} ")
        } else {
            lines.push(Line::from(std::mem::take(&mut cur)));
            used = 0;
            format!("{:VALUE_COL$}{label} ", "")
        };
        used += text::width(&head) + value_w;
        cur.push(Span::styled(head, label_style()));
        cur.extend(value);
    }
    lines.push(Line::from(cur));
    lines.into_iter().map(|l| text::truncate_line(l, max)).collect()
}

fn value(s: impl Into<String>) -> Vec<Span<'static>> {
    vec![Span::styled(s.into(), theme::TEXT)]
}

/// A metadata value entered with `i`, if set.
fn meta(value: &Option<String>) -> Option<String> {
    value.as_deref().map(str::trim).filter(|s| !s.is_empty()).map(str::to_string)
}

/// "Vehicle:" row with what was entered with `i` (year, make, model, region, command), or None
/// when nothing was. A command that only repeats the button name is left out.
fn vehicle_fields(capture: &Capture) -> Option<Vec<Field>> {
    let car: Vec<String> = [&capture.year, &capture.make, &capture.model].into_iter().filter_map(meta).collect();
    let region = meta(&capture.region);
    let command = meta(&capture.command).filter(|c| !c.eq_ignore_ascii_case(capture.button_name()));
    if car.is_empty() && region.is_none() && command.is_none() {
        return None;
    }
    let mut fields = vec![("Vehicle:", value(if car.is_empty() { "-".to_string() } else { car.join(" ") }))];
    if let Some(region) = region {
        fields.push(("Region:", value(region)));
    }
    if let Some(command) = command {
        fields.push(("Cmd:", value(command)));
    }
    Some(fields)
}

/// The button's name with its code; the code alone where the name is the code ("Btn 0x3" of a
/// gate remote whose protocol does not name its buttons: the row already says "Btn:").
fn button_label(capture: &Capture) -> String {
    let name = capture.button_name();
    // "Btn 0x9" (a gate remote's code) and "Btn 4" (a numbered button, as the C shows it) say
    // nothing the code does not: "Btn: Btn 4 (0x04)" was the line.
    let numbered = |code: u8| {
        name.strip_prefix("Btn ").and_then(|digits| u8::from_str_radix(digits, 16).ok()) == Some(code)
    };
    match capture.button {
        Some(code) if name == crate::protocols::gate_buttons::hex_label(code) || numbered(code) => capture.button_hex(),
        Some(_) => format!("{name} ({})", capture.button_hex()),
        None => name.to_string(),
    }
}

/// The vulnerability panel's line for a remote KAT can encode, from its protocol's code label
/// (`ProtocolInfo::encryption`): "Encryption is broken" was said of fixed-code remotes too, which
/// have no encryption, and of remotes that send a plain counter.
fn emulation_banner(code: &str) -> &'static str {
    let code = code.to_ascii_lowercase();
    if code == "fixed code" {
        " Static code, no encryption — complete emulation of the keyfob is available"
    } else if code.contains("plaintext counter") {
        " No encryption, a counter that goes up with each press — complete emulation of the keyfob is available"
    } else {
        " Encryption is broken — complete emulation of the keyfob is available"
    }
}

/// The "HiTag2:" row: the remote's HITAG2 key in the standard byte order, which qDoctor shows and
/// doctor-firmware's key prompt takes, and for a Fiat remote also in the Fiat cipher's, which
/// its line in `hitag2_keys.txt` has. Twelve digits each, as that file and the status line write
/// a key; an epoch other than 0 is named, since the key signs the frame only with it.
fn hitag2_key_fields(key: ShownKey) -> Vec<Field> {
    let style = Style::default().fg(Color::Yellow);
    let mut fields = vec![("HiTag2:", vec![Span::styled(hitag2_keys::hex(&key.standard, ""), style)])];
    if let Some(fiat) = key.fiat_order() {
        fields.push(("Fiat order:", vec![Span::styled(hitag2_keys::hex(&fiat, ""), style)]));
    }
    if key.epoch != 0 {
        fields.push(("Epoch:", value(key.epoch.to_string())));
    }
    fields
}

/// The rows of the signal detail for an area `max` columns wide. `hitag2`: the remote's HITAG2
/// key, when KAT has it.
fn signal_detail_lines(
    capture: &Capture,
    saved: Option<&SavedFobMatch>,
    searching: bool,
    hitag2: Option<ShownKey>,
    max: usize,
) -> Vec<Line<'static>> {
    let accent_style = Style::default().fg(Color::Cyan);
    let dim = Style::default().fg(Color::DarkGray);
    let mut lines = Vec::new();

    // Protocol + Make (the one entered with `i`, else the protocol's)
    let make = meta(&capture.make).unwrap_or_else(|| capture.protocol_make().to_string());
    lines.extend(detail_row(
        vec![
            ("Protocol:", vec![Span::styled(capture.protocol_name().to_string(), accent_style)]),
            ("Make:", value(make)),
        ],
        max,
    ));
    if let Some(fields) = vehicle_fields(capture) {
        lines.extend(detail_row(fields, max));
    }

    let mut freq = vec![
        ("Freq:", value(format!("{} MHz", format_mhz(capture.frequency)))),
        ("Mod:", value(capture.modulation().to_string())),
    ];
    if let Some(radio) = &capture.radio {
        freq.push(("Radio:", value(radio.clone())));
    }
    lines.extend(detail_row(freq, max));
    if let Some(tx) = &capture.own_tx {
        lines.extend(detail_row(
            vec![("Own TX:", vec![Span::styled(format!("KAT's transmission from {tx}, heard back"), Style::default().fg(Color::Cyan))])],
            max,
        ));
    }

    // Enc + RF (the protocol's AM/FM) + Rx (the demodulator path that received it) or, for a
    // recording, its preset
    // A frame of the older PSA code is neither of PSA's two kinds (TEA/XOR).
    let older_code = capture.psa_older_code();
    let listed_older_code = older_code.is_some_and(|(_, remote)| remote.is_some());
    // The older code shows "older code" once deciphered (a key table is loaded, so the frame has a
    // serial), else "older code, not decoded".
    let older_code_label = if capture.serial.is_some() { "older code" } else { "older code, not decoded" };
    let mut enc = vec![
        ("Enc:", value(if listed_older_code { older_code_label } else { capture.encryption_type() })),
        // An unknown modulation is "-" like the detail's other empty fields (its display text,
        // "—", is what the protocol tables use).
        ("RF:", value(match capture.rf_modulation() {
            crate::capture::RfModulation::Unknown => "-".to_string(),
            rf => rf.to_string(),
        })),
    ];
    if let Some(rf) = capture.received_rf {
        enc.push(("Rx:", value(rf.to_string())));
    } else if let Some(preset) = &capture.rf_preset {
        // A recording: the preset it was made with, which is all that says how it was demodulated.
        enc.push(("Preset:", value(preset.label())));
    }
    lines.extend(detail_row(enc, max));

    let serial = match capture.serial {
        Some(_) => format!("0x{}", capture.serial_hex()),
        None => "-".to_string(),
    };
    lines.extend(detail_row(
        vec![
            ("Serial:", vec![Span::styled(serial, accent_style)]),
            ("Btn:", value(button_label(capture))),
        ],
        max,
    ));

    // Counter in hex with its decimal value (the saved-fob difference below is decimal)
    let counter = match capture.counter {
        Some(c) => format!("0x{c:04X} ({c})"),
        None => "-".to_string(),
    };
    let crc = if capture.protocol.is_none() {
        Span::styled("-", dim)
    } else if !capture.crc_checked() {
        Span::styled("none", dim)
    } else if capture.crc_valid {
        Span::styled("OK ✓", Style::default().fg(Color::Green))
    } else {
        Span::styled("FAIL ✗", Style::default().fg(Color::Red))
    };
    lines.extend(detail_row(
        vec![
            ("Counter:", value(counter)),
            ("CRC:", vec![crc]),
            ("Status:", value(capture.status.to_string())),
        ],
        max,
    ));

    // The whole frame: a PSA frame's last 16 bits are in `data_extra` (issue #6: "only 8 of the
    // 10 bytes").
    let key = if capture.data_count_bit > 0 {
        let (hex, bits) = capture.frame_hex();
        vec![
            Span::styled(format!("0x{hex}"), Style::default().fg(Color::Yellow)),
            Span::styled(format!("  ({bits} bits)"), dim),
        ]
    } else {
        vec![Span::styled("-", dim)]
    };
    lines.extend(detail_row(vec![("Key/Data:", key)], max));

    // The older PSA code: the remote's type is in the frame's header (separate fields, so a narrow
    // panel wraps between them rather than cutting the part number off).
    if let Some((ty, remote)) = older_code {
        let mut fields = vec![match remote {
            Some((remote, _)) => ("Remote:", value(remote)),
            None => ("Remote:", vec![Span::styled("a type not in KAT's list", dim)]),
        }];
        fields.push(("Type:", vec![Span::styled(format!("{ty:04X}"), accent_style)]));
        if let Some((_, part)) = remote {
            fields.push(("Part no.:", value(part)));
        }
        lines.extend(detail_row(fields, max));
    }

    // A key search's result (a "Recover key" action, or autobf), or that one is under way.
    let search = match &capture.key_recovery {
        Some(KeyRecovery::Recovered(how)) => Some(Span::styled(format!("key recovered ({how})"), Style::default().fg(Color::Green))),
        Some(KeyRecovery::NotFound(how)) => Some(Span::styled(format!("no key ({how} searched)"), Style::default().fg(Color::Yellow))),
        None if searching => Some(Span::styled("searching…", label_style())),
        None => None,
    };
    if let Some(span) = search {
        lines.extend(detail_row(vec![("Recovery:", vec![span])], max));
    }
    if let Some(key) = hitag2 {
        lines.extend(detail_row(hitag2_key_fields(key), max));
    }

    // Saved .fob this capture matches: file name, counter difference, saved vehicle info + notes
    if let Some(m) = saved {
        // A long file name is cut in the middle so the counter difference stays in view.
        let counter = format!("  ({})", m.counter_label());
        let name = text::truncate_middle(&m.name, max.saturating_sub(VALUE_COL + 2 + text::width(&counter)));
        lines.extend(detail_row(
            vec![(
                "Saved:",
                vec![
                    Span::styled(format!("★ {}", name), Style::default().fg(SAVED_COLOR)),
                    Span::styled(counter, theme::TEXT),
                ],
            )],
            max,
        ));
        let mut info = m.vehicle_label();
        if let Some(notes) = m.notes.as_deref() {
            if !info.is_empty() {
                info.push_str(" — ");
            }
            info.push_str(notes);
        }
        if !info.is_empty() {
            let indent = Span::raw(" ".repeat(VALUE_COL));
            lines.push(text::truncate_line(Line::from(vec![indent, Span::styled(info, dim)]), max));
        }
    }

    lines.extend(detail_row(vec![("Captured:", value(capture.timestamp_full()))], max));

    let raw = if capture.has_raw_data() {
        Span::styled(format!("✓ {} transitions", capture.raw_pair_count()), Style::default().fg(Color::Green))
    } else if capture.source_file.as_deref().is_some_and(|f| f.to_ascii_lowercase().ends_with(".sub")) {
        // Loaded from a Flipper key file: TX re-encodes from the fields (no replay).
        Span::styled("None (key file)", dim)
    } else {
        Span::styled("None", dim)
    };
    lines.extend(detail_row(vec![("Raw Data:", vec![raw])], max));

    // File path (imported .sub/.fob only; blank for live captures): its end is what tells files
    // apart, so a long path keeps it.
    let file = capture.source_file.as_deref().unwrap_or("");
    let file = text::truncate_start(file, max.saturating_sub(VALUE_COL));
    lines.extend(detail_row(vec![("File:", value(file))], max));
    lines
}

/// Left half: signal information (plus the saved .fob it matches, if any)
fn render_signal_detail(
    frame: &mut Frame,
    area: Rect,
    capture: &Capture,
    saved: Option<&SavedFobMatch>,
    searching: bool,
    hitag2: Option<ShownKey>,
) {
    let border_style = match capture.status {
        CaptureStatus::EncoderCapable => Style::default().fg(Color::Green),
        CaptureStatus::Decoded => Style::default().fg(Color::Yellow),
        CaptureStatus::Unknown => Style::default().fg(Color::DarkGray),
    };
    let mut block = Block::default()
        .borders(Borders::ALL)
        .border_style(border_style)
        .title(format!(" Signal #{:02} — {} ", capture.id, capture.protocol_name()));
    let inner = block.inner(area);
    let lines = signal_detail_lines(capture, saved, searching, hitag2, inner.width as usize);
    // Rows the panel has no room for (a short terminal): say so instead of dropping them silently.
    let hidden = lines.len().saturating_sub(inner.height as usize);
    if hidden > 0 {
        block = block.title_bottom(Line::from(format!(" +{hidden} more ")).right_aligned());
    }
    frame.render_widget(Paragraph::new(lines).block(block), area);
}

/// The lines of the vulnerability panel for an area `max` columns wide and `rows` high: the
/// matching CVEs (each once, though the database may scope one CVE in several rows), and a
/// "+N more" last line when they do not all fit.
fn vulnerability_lines(capture: &Capture, max: usize, rows: usize) -> Vec<Line<'static>> {
    let label = label_style();
    let mut seen = std::collections::HashSet::new();
    let vulns: Vec<&vuln_db::VulnEntry> = vuln_db::match_vulns(
        capture.year.as_deref(),
        capture.make.as_deref(),
        capture.model.as_deref(),
        capture.region.as_deref(),
    )
    .into_iter()
    .filter(|v| seen.insert(v.cve))
    .collect();

    let wrap = |line: Line<'static>| text::wrap_line(line, max, 1);
    let mut lines = Vec::new();

    // A key a brute force recovered, or one it could not find.
    match &capture.key_recovery {
        Some(KeyRecovery::Recovered(how)) => {
            let style = Style::default().fg(Color::Green).add_modifier(Modifier::BOLD);
            lines.extend(wrap(Line::styled(format!(" Key recovered by brute force ({how})"), style)));
        }
        Some(KeyRecovery::NotFound(how)) => {
            lines.extend(wrap(Line::styled(
                format!(" No key found: {how} searched every key it can try"),
                Style::default().fg(Color::Yellow),
            )));
            lines.push(Line::default());
        }
        None => {}
    }

    // When we can encode: complete emulation is available, and what the remote's code has
    // against it (a static code and a plain counter have no encryption to break).
    if capture.status == CaptureStatus::EncoderCapable {
        let emu_style = Style::default().fg(Color::Green).add_modifier(Modifier::BOLD);
        lines.extend(wrap(Line::styled(emulation_banner(capture.encryption_type()), emu_style)));
        lines.push(Line::default());
    }

    // An unknown signal: no protocol to judge yet. Analyze is how to work one out.
    if capture.protocol.is_none() && capture.has_raw_data() {
        lines.extend(wrap(Line::styled(
            " Not decoded. Press a to analyze it.",
            Style::default().fg(Color::Cyan),
        )));
        lines.push(Line::default());
    }

    // Line where each CVE starts, to count the ones a cut hides.
    let mut starts = Vec::new();
    if vulns.is_empty() {
        let has_meta = capture.year.is_some()
            || capture.make.is_some()
            || capture.model.is_some()
            || capture.region.is_some();
        let hint = if has_meta {
            " No matching CVE in database."
        } else {
            " Set Year, Make, Model, Region (press i) to check vulnerabilities."
        };
        lines.extend(wrap(Line::styled(hint, theme::TEXT)));
    } else {
        for (i, v) in vulns.iter().enumerate() {
            if i > 0 {
                lines.push(Line::default());
            }
            starts.push(lines.len());
            lines.extend(wrap(Line::from(vec![
                Span::styled(" CVE: ", label),
                Span::styled(v.cve, Style::default().fg(Color::Cyan)),
            ])));
            lines.extend(wrap(Line::from(vec![
                Span::styled(" Description: ", label),
                Span::styled(v.description, theme::TEXT),
            ])));
            lines.extend(wrap(Line::from(vec![
                Span::styled(" Source: ", label),
                Span::styled(v.url, theme::TEXT),
            ])));
        }
    }

    if lines.len() > rows && rows > 0 {
        let shown = rows - 1;
        let hidden_cves = starts.iter().filter(|&&s| s >= shown).count();
        let more = match hidden_cves {
            0 => format!(" … +{} more lines", lines.len() - shown),
            1 => " … +1 more CVE".to_string(),
            n => format!(" … +{n} more CVEs"),
        };
        lines.truncate(shown);
        lines.push(Line::styled(text::truncate_end(&more, max), Style::default().fg(Color::DarkGray)));
    }
    lines
}

/// Right half: vulnerability panel (matching CVEs or prompt to set Year/Make/Model)
fn render_vulnerability_panel(frame: &mut Frame, area: Rect, capture: &Capture) {
    let border_color = if capture.vuln_found() { Color::Green } else { Color::Yellow };
    let block = Block::default()
        .borders(Borders::ALL)
        .border_style(Style::default().fg(border_color))
        .title(" Vulnerability ");
    let inner = block.inner(area);
    let lines = vulnerability_lines(capture, inner.width as usize, inner.height as usize);
    frame.render_widget(Paragraph::new(lines).block(block), area);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::hitag2_keys::KeyOrder;

    /// The HiTag2 row: the key in the standard order, a Fiat remote's in the Fiat order too (its
    /// first two bytes moved to the end), on one line at the panel's usual width; an epoch only
    /// when it is not 0.
    #[test]
    fn the_hitag2_row_shows_the_key_in_its_orders() {
        let text = |fields: Vec<Field>, max: usize| -> Vec<String> {
            detail_row(fields, max).iter().map(|l| l.spans.iter().map(|s| s.content.as_ref()).collect()).collect()
        };
        let inner = usize::from(SIGNAL_PREF_WIDTH) - 2;
        let fiat = ShownKey { standard: [0xB3, 0xC1, 0x01, 0x2C, 0xF7, 0x7F], order: KeyOrder::FiatBcm, epoch: 0 };
        assert_eq!(text(hitag2_key_fields(fiat), inner), vec![" HiTag2:    B3C1012CF77F  Fiat order: 012CF77FB3C1"]);
        let renault = ShownKey { order: KeyOrder::Standard, epoch: 3, ..fiat };
        assert_eq!(text(hitag2_key_fields(renault), inner), vec![" HiTag2:    B3C1012CF77F  Epoch: 3"]);
        // Narrower than both keys: the Fiat order goes under the first, whole.
        assert_eq!(
            text(hitag2_key_fields(fiat), 40),
            vec![" HiTag2:    B3C1012CF77F", "            Fiat order: 012CF77FB3C1"]
        );
    }

    /// The columns with one radio (no Radio column).
    fn content(protocol: u16) -> Vec<(Col, u16)> {
        content_radios(protocol).into_iter().filter(|&(c, _)| c != Col::Radio).collect()
    }

    /// The columns with several radios.
    fn content_radios(protocol: u16) -> Vec<(Col, u16)> {
        // ID "01", Time, MHz "433.92", Radio "RTL-SDR", Serial, Btn "Unlock", Cnt, Mod "PWM",
        // CRC "FAIL", Status, Vuln
        let w = |c: Col| match c {
            Col::Id => 2,
            Col::Time => 8,
            Col::Protocol => protocol,
            Col::Freq => 6,
            Col::Radio => 7,
            Col::Serial => 7,
            Col::Btn => 6,
            Col::Cnt => 4,
            Col::Mod => 3,
            Col::Crc => 4,
            Col::Status => 7,
            Col::Vuln => 4,
        };
        Col::ALL.iter().map(|&c| (c, w(c))).collect()
    }

    fn names(cols: &[(Col, u16)]) -> Vec<Col> {
        cols.iter().map(|&(c, _)| c).collect()
    }

    fn total(cols: &[(Col, u16)], spacing: u16) -> u16 {
        cols.iter().map(|&(_, w)| w).sum::<u16>() + spacing * (cols.len() as u16 - 1)
    }

    #[test]
    fn the_title_says_what_the_list_shows() {
        let arrival = Sort::default();
        let by_serial = Sort { column: Column::Serial, descending: true };
        assert_eq!(list_title(15, 15, "", arrival, 80), " Captures ");
        assert_eq!(list_title(3, 15, "acme", arrival, 80), " Captures · 3 of 15 · filter: acme ");
        assert_eq!(list_title(15, 15, "  ", by_serial, 80), " Captures · sort: Serial ▼ ");
        assert_eq!(
            list_title(3, 15, " acme ", Sort { descending: false, ..by_serial }, 80),
            " Captures · 3 of 15 · filter: acme · sort: Serial ▲ "
        );
        assert_eq!(list_title(15, 15, "", Sort { column: Column::Id, descending: true }, 80), " Captures · sort: ID ▼ ");
        // A long filter is cut to the box, between its corners.
        let long = "protocol:acme serial:0a1b2c3 !status:unknown btn:unlock";
        let cut = list_title(1, 15, long, by_serial, 70);
        assert_eq!(text::width(&cut), 68, "{cut}");
        assert!(cut.starts_with(" Captures · 1 of 15 · filter: protocol:acme") && cut.ends_with("… · sort: Serial ▼ "), "{cut}");
        // With no room for a few columns of it beside the sort, the sort goes (the header marks
        // it), then the filter's text.
        let narrow = list_title(1, 15, long, by_serial, 50);
        assert_eq!(text::width(&narrow), 48, "{narrow}");
        assert!(narrow.ends_with("… ") && !narrow.contains("sort"), "{narrow}");
        assert_eq!(list_title(1, 15, long, by_serial, 30), " Captures · 1 of 15 ");
        assert_eq!(list_title(1, 15, "ab", by_serial, 35), " Captures · 1 of 15 · filter: ab ");
        assert_eq!(list_title(1, 15, "ab", by_serial, 34), " Captures · 1 of 15 ");
    }

    #[test]
    fn the_sorted_columns_header_carries_the_direction() {
        let by = |column, descending| Sort { column, descending };
        assert_eq!(Col::Serial.header_shown(by(Column::Serial, false)), "Serial▲");
        assert_eq!(Col::Serial.header_shown(by(Column::Serial, true)), "Serial▼");
        assert_eq!(Col::Cnt.header_shown(by(Column::Serial, true)), "Cnt");
        // The order of arrival is the list as it is: no mark. Newest first has one.
        assert_eq!(Col::Id.header_shown(Sort::default()), "ID");
        assert_eq!(Col::Id.header_shown(by(Column::Id, true)), "ID▼");
        // A column the list does not have (:sort make) marks none.
        assert!(Col::ALL.iter().all(|c| c.header_shown(by(Column::Make, false)) == c.header()));
        // The headers are the names the filter and :sort know.
        assert_eq!(Col::ALL.map(Col::header), ["ID", "Time", "Protocol", "MHz", "Radio", "Serial", "Btn", "Cnt", "Mod", "CRC", "Status", "Vuln"]);
    }

    #[test]
    fn wide_tables_show_every_column_with_room_to_spare() {
        let one_radio: Vec<Col> = Col::ALL.into_iter().filter(|&c| c != Col::Radio).collect();
        let (cols, spacing) = choose_columns(113, &content(9));
        assert_eq!(names(&cols), one_radio);
        assert_eq!(spacing, 3);
        // The Protocol column has its room and a little spare, not all that is left (32): the
        // rest stays at the end of the rows.
        let protocol = |cols: &[(Col, u16)]| cols.iter().find(|(c, _)| *c == Col::Protocol).unwrap().1;
        assert_eq!(protocol(&cols), PROTOCOL_MIN + PROTOCOL_SPARE);
        assert_eq!(total(&cols, spacing), 113 - 32 + 24);
        let (cols, spacing) = choose_columns(140, &content_radios(9));
        assert_eq!((names(&cols), spacing), (Col::ALL.to_vec(), 3));
        // A long name gets its whole width, however wide the terminal.
        assert_eq!(protocol(&choose_columns(213, &content(40)).0), 40 + PROTOCOL_SPARE);
    }

    /// With several radios the Radio column goes right after Time when the table is narrow.
    #[test]
    fn the_radio_column_goes_early() {
        let (cols, _) = choose_columns(73, &content_radios(31));
        let names = names(&cols);
        assert!(!names.contains(&Col::Time) && !names.contains(&Col::Radio), "{names:?}");
        assert!(names.contains(&Col::Vuln), "{names:?}");
        let (cols, _) = choose_columns(90, &content_radios(9));
        assert!(names_of(&cols).contains(&Col::Radio));
    }

    fn names_of(cols: &[(Col, u16)]) -> Vec<Col> {
        cols.iter().map(|&(c, _)| c).collect()
    }

    #[test]
    fn narrow_tables_drop_columns_in_order_and_protocol_takes_the_rest() {
        // 80-column terminal: 73 columns inside the table.
        let (cols, spacing) = choose_columns(73, &content(9));
        assert_eq!((names(&cols), spacing), (names(&content(9)), 1));
        assert_eq!(total(&cols, 1), 73);

        // A long protocol name: Time goes first, the name gets the rest.
        let (cols, _) = choose_columns(73, &content(31));
        assert!(!names(&cols).contains(&Col::Time));
        assert!(names(&cols).contains(&Col::Vuln));
        let protocol = cols.iter().find(|(c, _)| *c == Col::Protocol).unwrap().1;
        assert!(protocol >= PROTOCOL_MIN, "{protocol}");
        assert_eq!(total(&cols, 1), 73);

        // The smallest terminal (60 columns): only the identifying columns are left.
        let (cols, _) = choose_columns(53, &content(31));
        assert_eq!(names(&cols), vec![Col::Id, Col::Protocol, Col::Freq, Col::Serial, Col::Btn, Col::Cnt]);
        assert_eq!(total(&cols, 1), 53);
    }

    #[test]
    fn detail_panel_shrinks_then_hides() {
        let area = |h| Rect::new(0, 0, 75, h);
        // Tall, and a capture with much to show: full-height panel, the rest for the table.
        let (table, detail) = split_captures_area(area(40), Some(30));
        assert_eq!((table.height, detail.map(|d| d.height)), (22, Some(DETAIL_MAX_HEIGHT)));
        // The panel is no taller than what it shows: the table gets the rows it leaves.
        let (table, detail) = split_captures_area(area(40), Some(11));
        assert_eq!((table.height, detail.map(|d| d.height)), (29, Some(11)));
        // 80×24 terminal (17-row middle): the table keeps 5 captures, the panel gets the rest.
        let (table, detail) = split_captures_area(area(17), Some(11));
        assert_eq!((table.height, detail.map(|d| d.height)), (TABLE_MIN_HEIGHT, Some(9)));
        // No room for a useful panel, or nothing selected: the table gets everything.
        assert_eq!(split_captures_area(area(12), Some(11)), (area(12), None));
        assert_eq!(split_captures_area(area(40), None), (area(40), None));
    }

    /// The panel asks for the rows of its taller half and its borders, so the table gets the rest;
    /// captures alike ask for the same, so the table does not jump while the selection moves.
    #[test]
    fn the_detail_panel_asks_for_what_it_shows() {
        let unknown = Capture::from_pairs(1, 433_920_000, Vec::new());
        // Protocol, Freq, Enc, Serial, Counter, Key/Data, Captured, Raw Data, File.
        assert_eq!(detail_height(98, &unknown, None, false, None), 9 + 2);
        let mut other = Capture::from_pairs(2, 315_000_000, Vec::new());
        other.source_file = Some("a/folder/with/a/long/name/and_a_recording_in_it.sub".to_string());
        assert_eq!(detail_height(98, &other, None, false, None), 9 + 2);
        // A key search under way has a row of its own.
        assert_eq!(detail_height(98, &unknown, None, true, None), 10 + 2);
        // So has a HITAG2 key KAT knows.
        let key = ShownKey { standard: [0xB3, 0xC1, 0x01, 0x2C, 0xF7, 0x7F], order: KeyOrder::FiatBcm, epoch: 0 };
        assert_eq!(detail_height(98, &unknown, None, false, Some(key)), 10 + 2);
        // At the narrowest the vulnerability hint wraps over more rows, never past the full height
        // once the area is split.
        let narrow = detail_height(58, &unknown, None, false, None);
        assert!(narrow >= 11, "{narrow}");
        let (_, panel) = split_captures_area(Rect::new(0, 0, 58, 60), Some(u16::MAX));
        assert_eq!(panel.map(|p| p.height), Some(DETAIL_MAX_HEIGHT));
    }

    /// The banner says what a remote's code has against emulation: nothing for a static code or a
    /// plain counter, a cipher for the rest.
    #[test]
    fn the_emulation_banner_names_the_code() {
        assert!(emulation_banner("Fixed Code").starts_with(" Static code, no encryption"));
        assert!(emulation_banner("Rolling Code (plaintext counter)").starts_with(" No encryption, a counter"));
        for cipher in ["Rolling Code", "KeeLoq", "AES-128", "Rolling (clear counter)", "XOR whitening"] {
            assert!(emulation_banner(cipher).starts_with(" Encryption is broken"), "{cipher}");
        }
        // A capture KAT can encode, through its protocol's label (Princeton: fixed code).
        let mut princeton = Capture::from_pairs(1, 433_920_000, Vec::new());
        princeton.protocol = Some("Princeton".to_string());
        assert_eq!(princeton.encryption_type(), "Fixed Code");
        assert!(emulation_banner(princeton.encryption_type()).contains("no encryption"));
    }

    /// The detail's button: its name and its code; the code alone when the name is the code.
    #[test]
    fn the_button_shows_its_name_and_code() {
        let mut capture = Capture::from_pairs(1, 433_920_000, Vec::new());
        assert_eq!(button_label(&capture), "-");
        capture.protocol = Some("Kia V3/V4".to_string());
        capture.button = Some(0x01);
        assert_eq!(button_label(&capture), "Lock (0x01)");
        // A gate remote's button without a name: "Btn 0x9" in the table, "Btn: 0x09" here.
        capture.protocol = Some("KeeLoq (NICE_MHOUSE)".to_string());
        capture.button = Some(0x09);
        assert_eq!((capture.button_name(), button_label(&capture).as_str()), ("Btn 0x9", "0x09"));
        // A numbered button ("Btn 4", "Btn F3" as the C shows them): the code alone too.
        capture.protocol = Some("Princeton".to_string());
        capture.button = Some(0x04);
        assert_eq!((capture.button_name(), button_label(&capture).as_str()), ("Btn 4", "0x04"));
        capture.button = Some(0xF3);
        assert_eq!((capture.button_name(), button_label(&capture).as_str()), ("Btn F3", "0xF3"));
        // "Other" (a code with no number) keeps its name.
        capture.button = Some(0x55);
        assert_eq!(button_label(&capture), "Other (0x55)");
        // PSA's two frame types number their buttons differently: the frame decides the name. A
        // mode 0x23 frame (a Delphi remote's Lock and third button), then a TEA frame's code 1.
        capture.protocol = Some("PSA".to_string());
        (capture.data, capture.data_extra, capture.button) = (0xB6DA_FAEE_20BD_48CF, Some(0x61F7), Some(0x01));
        assert_eq!(button_label(&capture), "Lock (0x01)");
        (capture.data, capture.data_extra, capture.button) = (0xB6DA_3125_DF76_B704, Some(0xA3F7), Some(0x03));
        assert_eq!(button_label(&capture), "Button 3 (0x03)");
        (capture.data, capture.data_extra, capture.button) = (0xF47A_2EA1_1A4C_7CAF, Some(0x2927), Some(0x01));
        assert_eq!(button_label(&capture), "Unlock (0x01)");
    }

    /// An empty list says what to do next, and every hint fits the narrowest terminal's list.
    #[test]
    fn an_empty_list_says_what_to_do_next() {
        assert!(empty_hint(false, false).contains(":load a file, or :new"));
        assert!(empty_hint(true, false).contains("press r to receive"));
        // While a radio receives, `r` would stop it: the hint is about the remote.
        for connected in [false, true] {
            let hint = empty_hint(connected, true);
            assert!(hint.contains("receiving") && !hint.contains("press r"), "{hint}");
        }
        // 60 columns: 58 inside the border, one of them the leading space.
        for (connected, receiving) in [(false, false), (true, false), (true, true)] {
            assert!(text::width(empty_hint(connected, receiving)) <= 57);
        }
    }

    #[test]
    fn detail_rows_wrap_between_fields() {
        let row = || {
            vec![
                ("Protocol:", value("Kia V3/V4")),
                ("Make:", value("Kia/Hyundai")),
            ]
        };
        let strings = |lines: Vec<Line>| -> Vec<String> {
            lines.iter().map(|l| l.spans.iter().map(|s| s.content.as_ref()).collect()).collect()
        };
        assert_eq!(strings(detail_row(row(), 60)), vec![" Protocol:  Kia V3/V4  Make: Kia/Hyundai"]);
        assert_eq!(
            strings(detail_row(row(), 35)),
            vec![" Protocol:  Kia V3/V4", "            Make: Kia/Hyundai"]
        );
        // Too narrow even for one field: cut with "…".
        assert_eq!(strings(detail_row(row(), 18)), vec![" Protocol:  Kia V…", "            Make:…"]);
    }

    /// A PSA frame of the older code (GitHub issue #6): all 80 bits on the Key/Data row (it showed
    /// key1's 64, "128 bits"), the remote its header names, and its CRC as unread, not failed.
    #[test]
    fn the_detail_shows_a_psa_frame_whole_and_its_remote() {
        let text = |c: &Capture, max: usize| -> Vec<String> {
            signal_detail_lines(c, None, false, None, max).iter().map(|l| l.spans.iter().map(|s| s.content.as_ref()).collect()).collect()
        };
        let mut capture = Capture::from_pairs(1, 433_920_000, Vec::new());
        capture.protocol = Some("PSA".to_string());
        (capture.data, capture.data_count_bit, capture.data_extra) = (0x7CAA_A2C5_8774_1C19, 128, Some(0xB7BB));
        let wide = text(&capture, 200);
        let row = |label: &str| wide.iter().find(|l| l.trim_start().starts_with(label)).cloned().unwrap_or_default();
        assert!(row("Key/Data:").contains("0x7CAAA2C587741C19B7BB  (80 bits)"), "{wide:?}");
        assert!(row("Enc:").contains("older code, not decoded"), "{wide:?}");
        assert!(row("Remote:").contains("Citroën Xsara Picasso  Type: 07CA  Part no.: 6554RF"), "{wide:?}");
        assert!(row("Counter:").contains("CRC: none"), "{wide:?}");
        // The longest names (C3 XTR, Berlingo/Partner): whole in a 120-column terminal's panel (58
        // columns); in an 80-column one's (52) the name may be cut, never the type or part number.
        for (data, ty, part) in [(0xC1AA_18C8_F715_6A8A, "0C1A", "6554RE"), (0x69AA_0000_0000_0000, "069A", "6554RH / 6554RG")] {
            capture.data = data;
            let lines = text(&capture, 58);
            assert!(lines.iter().all(|l| !l.contains('…')), "{lines:?}");
            let lines = text(&capture, 52);
            assert!(lines.iter().any(|l| l.contains(&format!("Type: {ty}  Part no.: {part}"))), "{lines:?}");
        }
        // A newer frame (TEA, header F47A): no Remote row, its encryption as before.
        (capture.data, capture.data_extra) = (0xF47A_2EA1_1A4C_7CAF, Some(0x2927));
        let lines = text(&capture, 200);
        assert!(!lines.iter().any(|l| l.contains("Remote:")), "{lines:?}");
        assert!(lines.iter().any(|l| l.trim_start().starts_with("Enc:") && l.contains("TEA/XOR")), "{lines:?}");
    }
}
