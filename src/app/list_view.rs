//! What the captures list shows: the captures a filter keeps, in the order of a sort.
//!
//! `App::captures` stays in the order the captures arrived: deleting and its undo, the session
//! file and every index into the list depend on it. The list on screen is a view of it, the
//! indexes [`visible`] gives, worked out whenever it is asked for (a key, a frame).
//!
//! **Filter** (`/`, `:filter`): words separated by spaces; a capture is shown when every word fits
//! it. A word is looked for, in any case, in the capture's protocol, frequency (as the list shows
//! it: "433.92"), radio, serial, button, status, make and model. `serial:1a2b` looks in one
//! column ([`Column::named`] has the names), `!word` keeps the captures without it, and `#7`,
//! `id:7`, `id:3-9` pick by ID.
//!
//! **Sort** (`s`, `S`, `:sort`): by one column. A capture with no value there (no serial, no
//! counter) comes after the others in both directions, and equal ones stay in the order they
//! arrived.
//!
//! The selection is a capture the view shows, or none: the keys that move it step through the
//! view, and a filter that hides the selected capture moves the selection to the nearest one it
//! shows.

use crossterm::event::KeyCode;

use super::{App, InputMode};
use crate::capture::Capture;
use crate::radio::hopper::format_mhz;

/// A column of the list, or the vehicle and the file a capture carries besides: what `:sort` and
/// a filter word like `serial:1a2b` name.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Column {
    Id,
    Time,
    Protocol,
    Freq,
    Radio,
    Serial,
    Btn,
    Cnt,
    Mod,
    Crc,
    Status,
    Vuln,
    Make,
    Model,
    File,
}

/// Each column's names: the first is the one shown (the list's header, "sort: Serial"); all are
/// understood, in any case.
const COLUMN_NAMES: [(Column, &[&str]); 15] = [
    (Column::Id, &["ID"]),
    (Column::Time, &["Time"]),
    (Column::Protocol, &["Protocol", "proto"]),
    (Column::Freq, &["MHz", "freq", "frequency"]),
    (Column::Radio, &["Radio"]),
    (Column::Serial, &["Serial"]),
    (Column::Btn, &["Btn", "button"]),
    (Column::Cnt, &["Cnt", "counter"]),
    (Column::Mod, &["Mod", "coding"]),
    (Column::Crc, &["CRC"]),
    (Column::Status, &["Status"]),
    (Column::Vuln, &["Vuln"]),
    (Column::Make, &["Make"]),
    (Column::Model, &["Model"]),
    (Column::File, &["File"]),
];

/// The columns a plain filter word is looked for in: what tells one remote from another. The
/// others are asked for by name (`cnt:04d2`, `mod:pwm`): a word would fit too much by chance
/// (digits in a counter, "man" in Manchester, a folder of every file's path).
const PLAIN_COLUMNS: [Column; 8] = [
    Column::Protocol,
    Column::Freq,
    Column::Radio,
    Column::Serial,
    Column::Btn,
    Column::Status,
    Column::Make,
    Column::Model,
];

/// The columns `s` goes through, in the list's order (Radio only while the list has that column).
/// The time is left out (it is the order the captures arrived in, as the ID is), and Vuln.
const SORT_CYCLE: [Column; 10] = [
    Column::Id,
    Column::Protocol,
    Column::Freq,
    Column::Radio,
    Column::Serial,
    Column::Btn,
    Column::Cnt,
    Column::Mod,
    Column::Crc,
    Column::Status,
];

impl Column {
    /// The column a name means ("serial", "MHz", "proto"); None for no column's name.
    pub fn named(name: &str) -> Option<Column> {
        COLUMN_NAMES
            .iter()
            .find(|(_, names)| names.iter().any(|n| n.eq_ignore_ascii_case(name)))
            .map(|&(column, _)| column)
    }

    /// The column's name as the list's header has it.
    pub fn title(self) -> &'static str {
        COLUMN_NAMES.iter().find(|&&(column, _)| column == self).map_or("", |(_, names)| names[0])
    }

    /// The column's text for a capture as the list shows it ("-" for none); the filter looks in it.
    pub fn text(self, c: &Capture) -> String {
        let or_dash = |text: &Option<String>| text.clone().unwrap_or_else(|| "-".to_string());
        match self {
            Column::Id => format!("{:02}", c.id),
            Column::Time => c.timestamp_short(),
            Column::Protocol => c.protocol_name().to_string(),
            Column::Freq => format_mhz(c.frequency),
            Column::Radio => or_dash(&c.radio),
            Column::Serial => c.serial_hex(),
            Column::Btn => c.button_name().to_string(),
            Column::Cnt => c.counter_str(),
            Column::Mod => c.modulation().to_string(),
            Column::Crc => c.crc_status().to_string(),
            Column::Status => c.status_label().to_string(),
            Column::Vuln => if c.vuln_found() { "Yes" } else { "No" }.to_string(),
            Column::Make => or_dash(&c.make),
            Column::Model => or_dash(&c.model),
            Column::File => or_dash(&c.source_file),
        }
    }

    /// What the column is sorted by; None for a capture with no value there.
    fn sort_key(self, c: &Capture) -> Option<Key> {
        let text = |s: &str| Key::Text(s.to_lowercase());
        match self {
            Column::Id => Some(Key::Number(i64::from(c.id))),
            Column::Time => Some(Key::Number(c.timestamp.timestamp_millis())),
            Column::Protocol => c.protocol.as_deref().map(text),
            Column::Freq => Some(Key::Number(i64::from(c.frequency))),
            Column::Radio => c.radio.as_deref().map(text),
            Column::Serial => c.serial.map(|s| Key::Number(i64::from(s))),
            Column::Btn => c.button.map(|_| text(c.button_name())),
            Column::Cnt => c.counter.map(|n| Key::Number(i64::from(n))),
            Column::Mod => c.protocol_info().map(|info| text(&info.encoding.to_string())),
            // A failed check first: it is the one to look at.
            Column::Crc => c.crc_checked().then(|| Key::Number(i64::from(c.crc_valid))),
            Column::Status => Some(c.status_label()).filter(|&s| s != "Unknown").map(text),
            // The vulnerable ones first.
            Column::Vuln => Some(Key::Number(i64::from(!c.vuln_found()))),
            Column::Make => c.make.as_deref().map(text),
            Column::Model => c.model.as_deref().map(text),
            Column::File => c.source_file.as_deref().map(text),
        }
    }
}

/// A sort value: every capture's value of one column is of one kind.
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Key {
    Number(i64),
    Text(String),
}

/// The list's order: by a column, upwards or downwards. The default, by ID upwards, is the order
/// the captures arrived in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Sort {
    pub column: Column,
    pub descending: bool,
}

impl Default for Sort {
    fn default() -> Self {
        Sort { column: Column::Id, descending: false }
    }
}

impl Sort {
    /// The order the captures arrived in: no sort.
    pub fn is_arrival_order(self) -> bool {
        self == Sort::default()
    }

    /// What the status bar says when the sort is set.
    fn message(self) -> String {
        match (self.column, self.descending) {
            (Column::Id, false) => "Sorted by ID: the order the captures arrived in".to_string(),
            (Column::Id, true) => "Sorted by ID, newest first".to_string(),
            (column, false) => format!("Sorted by {}", column.title()),
            (column, true) => format!("Sorted by {}, descending", column.title()),
        }
    }
}

/// The list's filter and sort (`App::list_view`).
#[derive(Default)]
pub struct ListView {
    /// The filter as typed; empty: every capture is shown.
    pub filter: String,
    pub sort: Sort,
    /// While the filter is typed (`/`): the filter and the selected capture's ID from before,
    /// which Esc goes back to.
    before_edit: Option<(String, Option<u32>)>,
}

/// One word of a filter.
#[derive(Debug, PartialEq)]
struct Term {
    /// The column to look in; None: the columns a plain word is looked for in.
    column: Option<Column>,
    /// What to look for, in lower case.
    text: String,
    /// `!word`: the capture must not have it.
    exclude: bool,
}

/// The words of a filter. A word of nothing but its marks (`!`, `serial:`, as while it is typed)
/// asks for nothing yet.
fn terms(filter: &str) -> Vec<Term> {
    filter
        .split_whitespace()
        .filter_map(|word| {
            let (exclude, word) = match word.strip_prefix('!') {
                Some(rest) => (true, rest),
                None => (false, word),
            };
            let (column, text) = match (word.strip_prefix('#'), word.split_once(':')) {
                (Some(id), _) => (Some(Column::Id), id),
                // "12:30" names no column: it is a plain word.
                (None, Some((name, text))) => match Column::named(name) {
                    Some(column) => (Some(column), text),
                    None => (None, word),
                },
                (None, None) => (None, word),
            };
            (!text.is_empty()).then(|| Term { column, text: text.to_lowercase(), exclude })
        })
        .collect()
}

/// Whether an ID is the one a filter word names: "7", "07", or of a range "3-9", "10-", "-5".
fn id_matches(text: &str, id: u32) -> bool {
    let number = |s: &str| s.trim_start_matches('#').parse::<u32>().ok();
    match text.split_once('-') {
        Some((from, to)) => {
            let from = if from.is_empty() { Some(0) } else { number(from) };
            let to = if to.is_empty() { Some(u32::MAX) } else { number(to) };
            matches!((from, to), (Some(from), Some(to)) if from <= id && id <= to)
        }
        None => number(text) == Some(id),
    }
}

fn term_fits(term: &Term, c: &Capture) -> bool {
    let has = |column: Column| match column {
        Column::Id => id_matches(&term.text, c.id),
        _ => column.text(c).to_lowercase().contains(&term.text),
    };
    let found = match term.column {
        Some(column) => has(column),
        None => PLAIN_COLUMNS.iter().any(|&column| has(column)),
    };
    found != term.exclude
}

/// The captures the list shows under `filter` and `sort`: indexes into `captures`, in the order
/// of the list's rows.
pub(super) fn visible(captures: &[Capture], filter: &str, sort: Sort) -> Vec<usize> {
    let terms = terms(filter);
    let mut rows: Vec<usize> =
        (0..captures.len()).filter(|&i| terms.iter().all(|term| term_fits(term, &captures[i]))).collect();
    if sort.column == Column::Id {
        // The list's own order: IDs only grow along it.
        if sort.descending {
            rows.reverse();
        }
        return rows;
    }
    let mut keyed: Vec<(Option<Key>, usize)> =
        rows.into_iter().map(|i| (sort.column.sort_key(&captures[i]), i)).collect();
    // A stable sort: equal ones keep the order they arrived in.
    keyed.sort_by(|(a, _), (b, _)| match (a, b) {
        (Some(a), Some(b)) if sort.descending => b.cmp(a),
        (Some(a), Some(b)) => a.cmp(b),
        (Some(_), None) => std::cmp::Ordering::Less,
        (None, Some(_)) => std::cmp::Ordering::Greater,
        (None, None) => std::cmp::Ordering::Equal,
    });
    keyed.into_iter().map(|(_, i)| i).collect()
}

/// The capture `delta` rows from the selected one in the view, stopping at its ends; counted
/// from the first row when nothing is selected (or the selected capture is not shown). None for
/// an empty view.
pub(super) fn moved(view: &[usize], selected: Option<usize>, delta: isize) -> Option<usize> {
    let last = view.len().checked_sub(1)?;
    let from = selected.and_then(|s| view.iter().position(|&i| i == s)).unwrap_or(0);
    Some(view[from.saturating_add_signed(delta).min(last)])
}

/// The capture of the view nearest to list position `at`, that one itself when it is shown (of
/// two as near, the later): where the selection goes when a filter hides the selected capture.
pub(super) fn nearest(view: &[usize], at: usize) -> Option<usize> {
    view.iter().copied().min_by_key(|&i| (i.abs_diff(at), std::cmp::Reverse(i)))
}

/// Whether a capture just added at list position `new` takes the selection (in the list, not
/// under a menu, form or prompt: `mode`). It has to be shown. Then it does when nothing is
/// selected, or when the selection is on the row before it at the list's end: tail-follow, which
/// only the order of arrival has (newest first, the end is the top). Otherwise browsing would be
/// hijacked.
pub(super) fn follows_new(
    mode: InputMode,
    view: &[usize],
    selected: Option<usize>,
    new: usize,
    sort: Sort,
) -> bool {
    let Some(row) = view.iter().position(|&i| i == new).filter(|_| mode == InputMode::Normal) else {
        return false;
    };
    let Some(selected) = selected else { return true };
    let before = match (sort.column, sort.descending) {
        (Column::Id, false) => row.checked_sub(1).and_then(|r| view.get(r)),
        (Column::Id, true) => view.get(row + 1),
        _ => None,
    };
    before == Some(&selected)
}

/// The capture the Analyze overlay moves to from the one at list position `at`: the next one
/// `ok` accepts after it (`forward`) or before it in the view. A capture the view does not show
/// (opened by ID under a filter) moves through the shown ones in list order.
pub(super) fn neighbour(view: &[usize], at: usize, forward: bool, ok: impl Fn(usize) -> bool) -> Option<usize> {
    match view.iter().position(|&i| i == at) {
        Some(row) if forward => view[row + 1..].iter().copied().find(|&i| ok(i)),
        Some(row) => view[..row].iter().rev().copied().find(|&i| ok(i)),
        None if forward => view.iter().copied().filter(|&i| i > at && ok(i)).min(),
        None => view.iter().copied().filter(|&i| i < at && ok(i)).max(),
    }
}

/// The column `s` sorts by after `now`. A column only `:sort` reaches leads back to the ID.
pub(super) fn next_sort_column(now: Column, with_radio: bool) -> Column {
    let cycle: Vec<Column> = SORT_CYCLE.iter().copied().filter(|&c| c != Column::Radio || with_radio).collect();
    match cycle.iter().position(|&c| c == now) {
        Some(at) => cycle[(at + 1) % cycle.len()],
        None => Column::Id,
    }
}

/// `:sort [column] [asc|desc]`: nothing for the order of arrival; a direction alone sorts by ID.
pub(super) fn parse_sort(args: &[&str]) -> Result<Sort, String> {
    let usage = || {
        let names: Vec<String> = COLUMN_NAMES.iter().map(|(_, names)| names[0].to_lowercase()).collect();
        format!("Usage: :sort [column] [desc] (columns: {})", names.join(", "))
    };
    let direction = |word: &str| match word.to_ascii_lowercase().as_str() {
        "asc" | "up" => Some(false),
        "desc" | "down" | "rev" | "reverse" => Some(true),
        _ => None,
    };
    match args {
        [] => Ok(Sort::default()),
        [word] => match (Column::named(word), direction(word)) {
            (Some(column), _) => Ok(Sort { column, descending: false }),
            (None, Some(descending)) => Ok(Sort { column: Column::Id, descending }),
            (None, None) => Err(format!("Unknown column: {word}. {}", usage())),
        },
        [name, way] => match (Column::named(name), direction(way)) {
            (Some(column), Some(descending)) => Ok(Sort { column, descending }),
            (None, _) => Err(format!("Unknown column: {name}. {}", usage())),
            (_, None) => Err(usage()),
        },
        _ => Err(usage()),
    }
}

impl App {
    /// The captures the list shows: indexes into `captures`, in the order of its rows.
    pub fn visible_captures(&self) -> Vec<usize> {
        visible(&self.captures, &self.list_view.filter, self.list_view.sort)
    }

    /// The selection is a capture the list shows: after the filter changed, and once per pass of
    /// the main loop (a capture can stop fitting the filter: a key search names its protocol, `i`
    /// sets its make), it moves from a hidden capture to the nearest shown one.
    pub fn keep_selection_shown(&mut self) {
        // Without a filter every capture is shown.
        if self.list_view.filter.is_empty() {
            return;
        }
        let view = self.visible_captures();
        self.selected_capture = self.selected_capture.and_then(|at| nearest(&view, at));
    }

    /// Select the capture at `at` when the list shows it; under a filter that hides it the
    /// selection stays where it is.
    pub(super) fn select_if_shown(&mut self, at: usize) {
        if self.list_view.filter.is_empty() || self.visible_captures().contains(&at) {
            self.selected_capture = Some(at);
        }
    }

    /// Select a capture the user just added (a loaded file, `:new`). A filter that hides it goes:
    /// the list shows what the action gave. True when it did.
    pub(super) fn select_added(&mut self, at: usize) -> bool {
        let hidden = !self.list_view.filter.is_empty() && !self.visible_captures().contains(&at);
        if hidden {
            self.list_view.filter.clear();
        }
        self.selected_capture = Some(at);
        hidden
    }

    /// The row of the list the selection is on.
    pub(super) fn selected_row(&self) -> Option<usize> {
        let selected = self.selected_capture?;
        self.visible_captures().iter().position(|&i| i == selected)
    }

    /// Select what is on `row` of the list now, the last row when the list ends before it.
    pub(super) fn select_row(&mut self, row: usize) {
        let view = self.visible_captures();
        self.selected_capture = view.get(row).or(view.last()).copied();
    }

    // ── Filter ──

    /// `/`: type the list's filter (the one set is there to change). The list narrows while it
    /// is typed.
    pub fn open_filter(&mut self) {
        let selected = self.selected_capture.and_then(|i| self.captures.get(i)).map(|c| c.id);
        self.list_view.before_edit = Some((self.list_view.filter.clone(), selected));
        self.input_mode = InputMode::Filter;
    }

    /// A key of the filter line: Enter keeps the filter, Esc goes back to the one from before,
    /// Backspace on the empty line closes it as the command line's does.
    pub fn filter_key(&mut self, code: KeyCode) {
        match code {
            KeyCode::Enter => self.close_filter(true),
            KeyCode::Esc => self.close_filter(false),
            KeyCode::Backspace if self.list_view.filter.is_empty() => self.close_filter(true),
            KeyCode::Backspace => {
                self.list_view.filter.pop();
                self.filter_edited();
            }
            KeyCode::Char(c) => {
                self.list_view.filter.push(c);
                self.filter_edited();
            }
            _ => {}
        }
    }

    /// The filter line's text changed (a key, Ctrl-U, Ctrl-W): the selection is the shown capture
    /// nearest to the one selected when the line was opened, so taking the typed words back
    /// leads back to it.
    pub(super) fn filter_edited(&mut self) {
        let view = self.visible_captures();
        let before = self.list_view.before_edit.as_ref().and_then(|(_, id)| *id);
        let anchor = before.and_then(|id| self.captures.iter().position(|c| c.id == id)).or(self.selected_capture);
        self.selected_capture = match anchor {
            Some(at) => nearest(&view, at),
            None => view.first().copied(),
        };
    }

    /// The filter line's text, for Ctrl-U / Ctrl-W.
    pub(super) fn filter_text_mut(&mut self) -> &mut String {
        &mut self.list_view.filter
    }

    fn close_filter(&mut self, keep: bool) {
        let before = self.list_view.before_edit.take();
        if keep {
            self.list_view.filter = self.list_view.filter.trim().to_string();
        } else if let Some((filter, _)) = before.clone() {
            self.list_view.filter = filter;
        }
        // Esc: the capture selected before, when the list (still) shows it.
        let view = self.visible_captures();
        let selected_before = before
            .filter(|_| !keep)
            .and_then(|(_, id)| id)
            .and_then(|id| self.captures.iter().position(|c| c.id == id))
            .filter(|at| view.contains(at));
        match selected_before {
            Some(at) => self.selected_capture = Some(at),
            None => self.selected_capture = self.selected_capture.and_then(|at| nearest(&view, at)),
        }
        self.input_mode = InputMode::Normal;
    }

    /// Esc in the list: show every capture again. False when no filter was set.
    pub fn clear_filter(&mut self) -> bool {
        if self.list_view.filter.is_empty() {
            return false;
        }
        self.list_view.filter.clear();
        self.status_message = Some("Filter cleared".to_string());
        true
    }

    /// `:filter <words>` sets the filter, `:filter` alone clears it.
    pub(super) fn filter_command(&mut self, words: &str) {
        let words = words.trim();
        if words.is_empty() {
            if !self.clear_filter() {
                self.status_message = Some("No filter set (usage: :filter <words>, or / in the list)".to_string());
            }
            return;
        }
        self.list_view.filter = words.to_string();
        let view = self.visible_captures();
        self.selected_capture = match self.selected_capture {
            Some(at) => nearest(&view, at),
            None => view.first().copied(),
        };
        let total = self.captures.len();
        self.status_message = Some(match view.len() {
            0 => "No capture fits the filter".to_string(),
            shown => format!("Filter set: {shown} of {total} capture{} shown", if total == 1 { "" } else { "s" }),
        });
    }

    // ── Sort ──

    /// `s`: sort by the next column of the list; after the last, back to the order of arrival.
    pub fn sort_by_next_column(&mut self) {
        let column = next_sort_column(self.list_view.sort.column, self.show_radio_column());
        self.set_sort(Sort { column, descending: false });
    }

    /// `S`: the other way round.
    pub fn reverse_sort(&mut self) {
        let sort = self.list_view.sort;
        self.set_sort(Sort { descending: !sort.descending, ..sort });
    }

    /// `:sort [column] [desc]`.
    pub(super) fn sort_command(&mut self, args: &[&str]) {
        match parse_sort(args) {
            Ok(sort) => self.set_sort(sort),
            Err(usage) => self.last_error = Some(usage),
        }
    }

    fn set_sort(&mut self, sort: Sort) {
        self.list_view.sort = sort;
        self.status_message = Some(sort.message());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture::CaptureStatus;

    /// A capture as a decoder leaves it.
    fn capture(id: u32, protocol: &str, mhz: f64, serial: u32, button: u8, counter: u16) -> Capture {
        let mut c = Capture::from_pairs(id, (mhz * 1e6).round() as u32, Vec::new());
        c.protocol = Some(protocol.to_string());
        c.serial = Some(serial);
        c.button = Some(button);
        c.counter = Some(counter);
        c.status = CaptureStatus::Decoded;
        c
    }

    /// Six captures of made-up protocols (KAT's own button names: Lock, Unlock, Trunk): two
    /// presses of one remote, a remote on another frequency, an unknown signal, one heard by a
    /// named radio, one from a file with its vehicle set.
    fn list() -> Vec<Capture> {
        let mut list = vec![
            capture(1, "Acme V3", 433.92, 0x0A1_B2C3, 0x01, 0x0010),
            capture(2, "Borg V0", 315.0, 0x1A2_B3C4, 0x02, 0x0200),
            capture(3, "Acme V3", 433.92, 0x0A1_B2C3, 0x02, 0x0011),
            Capture::from_pairs(4, 433_920_000, Vec::new()),
            capture(5, "Acme V4", 433.92, 0x077_0001, 0x01, 0x0005),
            capture(6, "Zeta V0", 868.35, 0x0AB_CDEF, 0x04, 0x0001),
        ];
        list[4].radio = Some("RTL-SDR 2".to_string());
        list[5].make = Some("Fabrikam".to_string());
        list[5].model = Some("Swift".to_string());
        list[5].source_file = Some("/data/IMPORTS/FABRIKAM/remote_kd.sub".to_string());
        list
    }

    fn ids(captures: &[Capture], filter: &str, sort: Sort) -> Vec<u32> {
        visible(captures, filter, sort).into_iter().map(|i| captures[i].id).collect()
    }

    fn by(column: Column, descending: bool) -> Sort {
        Sort { column, descending }
    }

    #[test]
    fn no_filter_and_no_sort_is_the_list() {
        let list = list();
        assert_eq!(visible(&list, "", Sort::default()), vec![0, 1, 2, 3, 4, 5]);
        assert_eq!(visible(&list, "   ", Sort::default()), vec![0, 1, 2, 3, 4, 5]);
        assert_eq!(visible(&[], "acme", Sort::default()), Vec::<usize>::new());
    }

    #[test]
    fn a_word_is_looked_for_in_what_tells_remotes_apart() {
        let list = list();
        let all = Sort::default();
        assert_eq!(ids(&list, "acme", all), [1, 3, 5], "the protocol, in any case");
        assert_eq!(ids(&list, "ACME v3", all), [1, 3], "every word has to fit");
        assert_eq!(ids(&list, "315", all), [2], "the frequency as the list shows it");
        assert_eq!(ids(&list, "433.92", all), [1, 3, 4, 5]);
        assert_eq!(ids(&list, "a1b2", all), [1, 3], "the serial");
        assert_eq!(ids(&list, "unlock", all), [2, 3], "the button's name");
        assert_eq!(ids(&list, "rtl", all), [5], "the radio");
        assert_eq!(ids(&list, "unknown", all), [4], "an unknown signal");
        assert_eq!(ids(&list, "swift", all), [6], "the vehicle set with i");
        assert_eq!(ids(&list, "nothing-like-it", all), [0u32; 0]);
        // Not in the ID, the counter, the coding or the file: "0010" is #01's counter, "data" a
        // folder, and no coding is known of a made-up protocol.
        assert_eq!(ids(&list, "0010", all), [0u32; 0]);
        assert_eq!(ids(&list, "data", all), [0u32; 0]);
        assert_eq!(ids(&list, "mod:unknown", all), [1, 2, 3, 4, 5, 6]);
    }

    #[test]
    fn a_word_can_name_its_column_or_exclude() {
        let list = list();
        let all = Sort::default();
        assert_eq!(ids(&list, "serial:0001", all), [5]);
        assert_eq!(ids(&list, "Serial:0001", all), [5], "the column's name in any case");
        assert_eq!(ids(&list, "cnt:0010", all), [1]);
        assert_eq!(ids(&list, "counter:001", all), [1, 3, 6], "another name of the column");
        assert_eq!(ids(&list, "btn:lock", all), [1, 2, 3, 5], "a part of the text: Unlock too");
        assert_eq!(ids(&list, "mhz:868", all), [6]);
        assert_eq!(ids(&list, "file:fabrikam", all), [6]);
        assert_eq!(ids(&list, "serial:-", all), [4], "the dash of a capture without one");
        assert_eq!(ids(&list, "status:decoded", all), [1, 2, 3, 5, 6]);
        assert_eq!(ids(&list, "!acme", all), [2, 4, 6]);
        assert_eq!(ids(&list, "acme !unlock", all), [1, 5]);
        assert_eq!(ids(&list, "!proto:acme !status:unknown", all), [2, 6]);
        // By ID: one, or a range with either end open.
        assert_eq!(ids(&list, "#3", all), [3]);
        assert_eq!(ids(&list, "id:03", all), [3]);
        assert_eq!(ids(&list, "id:2-4", all), [2, 3, 4]);
        assert_eq!(ids(&list, "id:5-", all), [5, 6]);
        assert_eq!(ids(&list, "id:-2", all), [1, 2]);
        assert_eq!(ids(&list, "!id:2-5", all), [1, 6]);
        assert_eq!(ids(&list, "id:x", all), [0u32; 0]);
    }

    /// While a word is typed its marks alone ask for nothing: the list does not empty at "!" or
    /// "serial:".
    #[test]
    fn a_half_typed_word_asks_for_nothing() {
        let list = list();
        for filter in ["!", "serial:", "acme !", "#", "!cnt:"] {
            let expected: &[u32] = if filter.starts_with("acme") { &[1, 3, 5] } else { &[1, 2, 3, 4, 5, 6] };
            assert_eq!(ids(&list, filter, Sort::default()), expected, "{filter:?}");
        }
        // A colon after no column's name is part of a plain word.
        assert_eq!(terms("12:30"), vec![Term { column: None, text: "12:30".to_string(), exclude: false }]);
        assert_eq!(terms("time:12:30"), vec![Term { column: Some(Column::Time), text: "12:30".to_string(), exclude: false }]);
    }

    #[test]
    fn every_column_has_a_name_and_a_title() {
        for (column, names) in COLUMN_NAMES {
            assert_eq!(column.title(), names[0]);
            for name in names {
                assert_eq!(Column::named(name), Some(column), "{name}");
                assert_eq!(Column::named(&name.to_uppercase()), Some(column), "{name}");
            }
        }
        assert_eq!(Column::named("protocols"), None);
        assert_eq!(Column::named(""), None);
        assert!(SORT_CYCLE.iter().chain(&PLAIN_COLUMNS).all(|c| COLUMN_NAMES.iter().any(|(named, _)| named == c)));
    }

    #[test]
    fn sorting_puts_missing_values_last_and_keeps_ties_in_order() {
        let list = list();
        assert_eq!(ids(&list, "", by(Column::Id, true)), [6, 5, 4, 3, 2, 1], "newest first");
        // Serials as numbers; #01 and #03 are one remote and stay in the order they came.
        assert_eq!(ids(&list, "", by(Column::Serial, false)), [5, 1, 3, 6, 2, 4]);
        assert_eq!(ids(&list, "", by(Column::Serial, true)), [2, 6, 1, 3, 5, 4], "the one without a serial stays last");
        assert_eq!(ids(&list, "", by(Column::Cnt, false)), [6, 5, 1, 3, 2, 4]);
        assert_eq!(ids(&list, "", by(Column::Protocol, false)), [1, 3, 5, 2, 6, 4]);
        assert_eq!(ids(&list, "", by(Column::Protocol, true)), [6, 2, 5, 1, 3, 4]);
        assert_eq!(ids(&list, "", by(Column::Freq, false)), [2, 1, 3, 4, 5, 6]);
        assert_eq!(ids(&list, "", by(Column::Btn, false)), [1, 5, 6, 2, 3, 4], "Lock, Trunk, Unlock");
        assert_eq!(ids(&list, "", by(Column::Radio, false)), [5, 1, 2, 3, 4, 6]);
        assert_eq!(ids(&list, "", by(Column::Status, false)), [1, 2, 3, 5, 6, 4]);
        assert_eq!(ids(&list, "", by(Column::Make, true)), [6, 1, 2, 3, 4, 5]);
        // With a filter: the shown ones, sorted.
        assert_eq!(ids(&list, "acme", by(Column::Cnt, false)), [5, 1, 3]);
        assert_eq!(ids(&list, "acme", by(Column::Id, true)), [5, 3, 1]);
    }

    #[test]
    fn the_selection_moves_through_the_view() {
        // The view of a sorted, filtered list: list positions in the order of the rows.
        let view = [4, 0, 2, 5];
        assert_eq!(moved(&view, Some(0), 1), Some(2));
        assert_eq!(moved(&view, Some(0), -1), Some(4));
        assert_eq!(moved(&view, Some(4), -1), Some(4), "the first row is the end");
        assert_eq!(moved(&view, Some(5), 1), Some(5), "so is the last");
        assert_eq!(moved(&view, Some(4), 10), Some(5), "a page past the end");
        assert_eq!(moved(&view, Some(5), -10), Some(4));
        assert_eq!(moved(&view, None, 2), Some(2), "from the first row when nothing is selected");
        assert_eq!(moved(&view, Some(3), 1), Some(0), "a selection the view does not show: as none");
        assert_eq!(moved(&[], Some(3), 1), None);
        assert_eq!(moved(&[], None, 0), None);
        // A page of the whole list.
        let all: Vec<usize> = (0..30).collect();
        assert_eq!(moved(&all, None, 10), Some(10));
        assert_eq!(moved(&all, Some(25), 10), Some(29));
        assert_eq!(moved(&all, Some(5), -10), Some(0));
    }

    #[test]
    fn a_hidden_selection_moves_to_the_nearest_shown_capture() {
        assert_eq!(nearest(&[0, 2, 5], 2), Some(2), "shown: it stays");
        assert_eq!(nearest(&[0, 2, 5], 3), Some(2));
        assert_eq!(nearest(&[0, 2, 5], 4), Some(5));
        assert_eq!(nearest(&[0, 2, 6], 4), Some(6), "as near on both sides: the later one");
        assert_eq!(nearest(&[5, 0, 2], 1), Some(2), "whatever the order of the rows");
        assert_eq!(nearest(&[], 4), None);
    }

    #[test]
    fn a_new_capture_takes_the_selection_only_at_the_tail() {
        let arrival = Sort::default();
        let normal = InputMode::Normal;
        // The list as it was: nothing selected, or the selection on its last row.
        assert!(follows_new(normal, &[0], None, 0, arrival));
        assert!(follows_new(normal, &[0, 1, 2, 3, 4, 5], None, 5, arrival));
        assert!(follows_new(normal, &[0, 1, 2, 3, 4, 5], Some(4), 5, arrival), "selection on the last row: follow");
        assert!(!follows_new(normal, &[0, 1, 2, 3, 4, 5], Some(2), 5, arrival), "browsing: stay");
        assert!(!follows_new(InputMode::SignalMenu, &[0, 1, 2, 3, 4, 5], Some(4), 5, arrival), "an open menu keeps its capture");
        assert!(!follows_new(InputMode::Command, &[0, 1, 2, 3, 4, 5], None, 5, arrival));
        assert!(!follows_new(InputMode::Filter, &[0, 1, 2, 3, 4, 5], None, 5, arrival));
        // Under a filter: only a capture the list shows, and the row before it is the last shown one.
        assert!(!follows_new(normal, &[0, 2], Some(2), 5, arrival), "the filter hides the new one");
        assert!(!follows_new(normal, &[0, 2], None, 5, arrival));
        assert!(follows_new(normal, &[0, 2, 5], Some(2), 5, arrival));
        assert!(!follows_new(normal, &[0, 2, 5], Some(0), 5, arrival));
        assert!(follows_new(normal, &[5], None, 5, arrival), "the first one the filter shows");
        // Newest first: the tail is the top row.
        let newest_first = by(Column::Id, true);
        assert!(follows_new(normal, &[5, 4, 3], Some(4), 5, newest_first));
        assert!(!follows_new(normal, &[5, 4, 3], Some(3), 5, newest_first));
        // Sorted by a column, the new capture lands anywhere: it is selected only when nothing is.
        let by_serial = by(Column::Serial, false);
        assert!(!follows_new(normal, &[0, 2, 5], Some(2), 5, by_serial));
        assert!(follows_new(normal, &[0, 5, 2], None, 5, by_serial));
    }

    #[test]
    fn analyze_moves_to_the_neighbours_in_the_view() {
        let view = [4, 0, 2, 5];
        let any = |_: usize| true;
        assert_eq!(neighbour(&view, 0, true, any), Some(2));
        assert_eq!(neighbour(&view, 0, false, any), Some(4));
        assert_eq!(neighbour(&view, 4, false, any), None);
        assert_eq!(neighbour(&view, 5, true, any), None);
        assert_eq!(neighbour(&view, 4, true, |i| i != 0), Some(2), "over one without pulses");
        assert_eq!(neighbour(&view, 5, false, |i| i == 4), Some(4));
        // Opened by ID while the filter hides it: the shown ones around it in list order.
        assert_eq!(neighbour(&view, 3, true, any), Some(4));
        assert_eq!(neighbour(&view, 3, false, any), Some(2));
        assert_eq!(neighbour(&view, 6, true, any), None);
        assert_eq!(neighbour(&[], 3, true, any), None);
    }

    #[test]
    fn s_goes_through_the_lists_columns() {
        let mut seen = vec![Column::Id];
        loop {
            let next = next_sort_column(*seen.last().unwrap(), false);
            if next == Column::Id {
                break;
            }
            seen.push(next);
        }
        use Column::*;
        assert_eq!(seen, [Id, Protocol, Freq, Serial, Btn, Cnt, Mod, Crc, Status]);
        assert_eq!(next_sort_column(Freq, true), Radio, "with several radios the list has the column");
        assert_eq!(next_sort_column(Radio, false), Id, "a column the list does not have leads back");
        assert_eq!(next_sort_column(Make, false), Id, "so does one only :sort reaches");
    }

    #[test]
    fn the_sort_command_reads_a_column_and_a_direction() {
        assert_eq!(parse_sort(&[]), Ok(Sort::default()));
        assert_eq!(parse_sort(&["serial"]), Ok(by(Column::Serial, false)));
        assert_eq!(parse_sort(&["MHz", "DESC"]), Ok(by(Column::Freq, true)));
        assert_eq!(parse_sort(&["counter", "asc"]), Ok(by(Column::Cnt, false)));
        assert_eq!(parse_sort(&["desc"]), Ok(by(Column::Id, true)), "a direction alone: by ID");
        assert_eq!(parse_sort(&["make", "reverse"]), Ok(by(Column::Make, true)));
        let unknown = parse_sort(&["colour"]).unwrap_err();
        assert!(unknown.starts_with("Unknown column: colour. Usage: :sort [column] [desc] (columns: id, time, protocol, mhz,"), "{unknown}");
        assert!(parse_sort(&["serial", "sideways"]).unwrap_err().starts_with("Usage: :sort"));
        assert!(parse_sort(&["serial", "desc", "more"]).unwrap_err().starts_with("Usage: :sort"));
        assert!(Sort::default().is_arrival_order());
        assert!(!by(Column::Id, true).is_arrival_order());
        assert_eq!(by(Column::Serial, true).message(), "Sorted by Serial, descending");
        assert_eq!(by(Column::Freq, false).message(), "Sorted by MHz");
    }
}
