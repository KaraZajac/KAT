//! Keyboard input, commands and selection for [`App`]: what a key means (Ctrl-C, Ctrl / Alt
//! characters), capture-ID lists (`:replay 1-5`, `:delete 2 4`) and their summaries, delete with a
//! one-level undo (`u`), when a new capture takes the selection, the signal menu's capture and
//! highlighted action, list paging, the export and capture-metadata forms, the `:load` browser's
//! keys and path resolution, the help / License / Credits overlays, the quit and `:delete all`
//! confirmations, the settings dropdown's rows and the background-job status line.
//!
//! A child module of `app` (`mod input;`), so it can use App's fields; the main loop (`run_app`
//! in main.rs) dispatches keys here.

use std::collections::HashSet;
use std::path::{Path, PathBuf};

use anyhow::Result;
use crossterm::event::{KeyCode, KeyEvent, KeyModifiers};

use super::{
    tx_power_label, App, ExportFormat, InputMode, SettingsField, SignalAction, LNA_STEPS, PRESET_FREQUENCIES,
    TX_POWER_STEPS, VGA_STEPS,
};
use crate::capture::{ButtonCommand, Capture};
use crate::radio::hopper::format_mhz;
use crate::radio::iq_file::find_file;

/// Input state owned by [`App`] (its `input` field).
#[derive(Default)]
pub(super) struct InputState {
    /// The capture the open signal menu acts on (by ID: a capture arriving meanwhile cannot
    /// retarget it) and its highlighted action (kept when the list changes under it).
    menu_capture: Option<u32>,
    menu_action: Option<SignalAction>,
    /// Captures the last delete removed, with their positions in the list then (`u`).
    undo: Vec<(usize, Capture)>,
    /// Transmissions run since the queue was last empty ("Transmitting 3/20…").
    pub(super) tx_done: usize,
    /// Status shown when the queued transmissions are done ("Replayed 3 of 5 (#04, #05 not found)").
    pub(super) tx_summary: Option<String>,
}

/// What a key press means. Ctrl-C quits from the list (as `q` does, asking first while work runs)
/// and is Esc everywhere else; Ctrl-H is Backspace (terminals that send ^H for that key). Other
/// characters typed with Ctrl or Alt are dropped: they used to land in text fields as plain
/// letters (Ctrl-C typed "c", and a hex "C" in the "Send button…" code, so the next Enter sent
/// button 0x0C). Ctrl and Alt together are AltGr on Windows terminals: a typed character.
fn key_code_for(mode: InputMode, key: &KeyEvent) -> Option<KeyCode> {
    let ctrl = key.modifiers.contains(KeyModifiers::CONTROL);
    let alt = key.modifiers.contains(KeyModifiers::ALT);
    match key.code {
        KeyCode::Char(c) if ctrl && !alt => match c.to_ascii_lowercase() {
            'c' if mode == InputMode::Normal => Some(KeyCode::Char('q')),
            'c' => Some(KeyCode::Esc),
            'h' => Some(KeyCode::Backspace),
            _ => None,
        },
        KeyCode::Char(_) if alt && !ctrl => None,
        code => Some(code),
    }
}

// ── Capture-ID lists ─────────────────────────────────────────────────────────

/// One element of a capture-ID list: an ID, or a range in the order typed (`5-1` runs backwards).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum IdItem {
    One(u32),
    Range(u32, u32),
}

/// What an ID list looks like, for error messages.
const ID_LIST_HINT: &str = "e.g. 4, 1-3, 1,5 or #4";

/// Parse a capture-ID list: IDs (`4`, `#4`) and ranges (`1-5`, `5-1`, `#1-#5`, `1 - 5`) separated
/// by commas and/or spaces. Ranges stay ranges, resolved against the captures that exist
/// ([resolve_ids]): `1-4000000000` used to allocate 16 GB and abort without restoring the terminal.
fn parse_id_items(spec: &str) -> Result<Vec<IdItem>, String> {
    let id = |s: &str| s.strip_prefix('#').unwrap_or(s).parse::<u32>().ok();
    let mut items = Vec::new();
    for part in spec.split(',') {
        // Spaces separate IDs, except around a range's dash ("1 - 5").
        let mut tokens: Vec<String> = Vec::new();
        for word in part.split_whitespace() {
            match tokens.last_mut() {
                Some(prev) if prev.ends_with('-') || word.starts_with('-') => prev.push_str(word),
                _ => tokens.push(word.to_string()),
            }
        }
        for tok in tokens {
            let item = match tok.split_once('-') {
                Some((a, b)) => id(a).zip(id(b)).map(|(a, b)| IdItem::Range(a, b)),
                None => id(&tok).map(IdItem::One),
            };
            items.push(item.ok_or_else(|| format!("Invalid capture ID: {tok} ({ID_LIST_HINT})"))?);
        }
    }
    if items.is_empty() {
        return Err(format!("No capture ID given ({ID_LIST_HINT})"));
    }
    Ok(items)
}

/// An ID list resolved against the captures in the list.
#[derive(Debug, Default, PartialEq)]
struct IdSelection {
    /// IDs in the list, in the order the spec names them (a range in its direction; repeats kept).
    found: Vec<u32>,
    /// How many IDs the spec names (a range counts every number in it).
    requested: u64,
    /// IDs named that are not in the list, as ascending inclusive ranges.
    missing: Vec<(u32, u32)>,
}

impl IdSelection {
    /// Number of missing IDs.
    fn missing_count(&self) -> u64 {
        self.missing.iter().map(|&(a, b)| u64::from(b - a) + 1).sum()
    }
}

/// Resolve `items` against the IDs that exist, never listing the numbers of a range.
fn resolve_ids(items: &[IdItem], existing: &[u32]) -> IdSelection {
    let mut sorted = existing.to_vec();
    sorted.sort_unstable();
    sorted.dedup();
    let mut sel = IdSelection::default();
    let mut missing = Vec::new();
    for &item in items {
        let (from, to) = match item {
            IdItem::One(id) => (id, id),
            IdItem::Range(a, b) => (a, b),
        };
        let (lo, hi) = (from.min(to), from.max(to));
        sel.requested += u64::from(hi - lo) + 1;
        let inside = &sorted[sorted.partition_point(|&x| x < lo)..sorted.partition_point(|&x| x <= hi)];
        if from <= to {
            sel.found.extend_from_slice(inside);
        } else {
            sel.found.extend(inside.iter().rev());
        }
        // The gaps around them are missing.
        let mut next = u64::from(lo);
        for &x in inside {
            if u64::from(x) > next {
                missing.push((next as u32, x - 1));
            }
            next = u64::from(x) + 1;
        }
        if next <= u64::from(hi) {
            missing.push((next as u32, hi));
        }
    }
    sel.missing = merge_ranges(missing);
    sel
}

/// Sort inclusive ranges and merge the ones that overlap or touch.
fn merge_ranges(mut ranges: Vec<(u32, u32)>) -> Vec<(u32, u32)> {
    ranges.sort_unstable();
    let mut out: Vec<(u32, u32)> = Vec::new();
    for (a, b) in ranges {
        match out.last_mut() {
            Some(last) if u64::from(a) <= u64::from(last.1) + 1 => last.1 = last.1.max(b),
            _ => out.push((a, b)),
        }
    }
    out
}

/// "#04", "#04, #05", "#07-#12", joined by ", "; a long list ends with "…".
fn ranges_text(ranges: &[(u32, u32)]) -> String {
    const SHOWN: usize = 4;
    let mut parts: Vec<String> = ranges
        .iter()
        .take(SHOWN)
        .map(|&(a, b)| match b - a {
            0 => format!("#{a:02}"),
            1 => format!("#{a:02}, #{b:02}"),
            _ => format!("#{a:02}-#{b:02}"),
        })
        .collect();
    if ranges.len() > SHOWN {
        parts.push("…".to_string());
    }
    parts.join(", ")
}

/// [ranges_text] of a list of IDs (consecutive ones become a range).
fn ids_text(ids: &[u32]) -> String {
    ranges_text(&merge_ranges(ids.iter().map(|&id| (id, id)).collect()))
}

/// "Capture #05 not found", "Captures #04, #05 not found".
fn not_found_text(sel: &IdSelection) -> String {
    let noun = if sel.missing_count() == 1 { "Capture" } else { "Captures" };
    format!("{noun} {} not found", ranges_text(&sel.missing))
}

/// The end of an ID-list command: "Replayed 3 of 5 (#02: no raw signal; #04, #05 not found)".
fn list_summary(what: &str, sent: usize, sel: &IdSelection, failed: &[(u32, String)]) -> String {
    let mut notes: Vec<String> = failed.iter().take(2).map(|(id, why)| format!("#{id:02}: {why}")).collect();
    if failed.len() > 2 {
        notes.push(format!("{} more not sent", failed.len() - 2));
    }
    if !sel.missing.is_empty() {
        notes.push(format!("{} not found", ranges_text(&sel.missing)));
    }
    let mut text = format!("{what} {sent} of {}", sel.requested);
    if !notes.is_empty() {
        text.push_str(&format!(" ({})", notes.join("; ")));
    }
    text
}

// ── Selection ────────────────────────────────────────────────────────────────

/// Remove the items `gone` picks from `list`; returns them with their positions, and the selection
/// afterwards: the same item when it stays (deleting #2 with #5 selected used to move the
/// selection to #6), else the next remaining one, else the last.
fn remove_where<T>(
    list: &mut Vec<T>,
    gone: impl Fn(&T) -> bool,
    selected: Option<usize>,
) -> (Vec<(usize, T)>, Option<usize>) {
    let mut removed = Vec::new();
    let mut kept = Vec::with_capacity(list.len());
    let mut kept_before_selected = 0;
    for (pos, item) in std::mem::take(list).into_iter().enumerate() {
        if gone(&item) {
            removed.push((pos, item));
        } else {
            if selected.is_some_and(|s| pos < s) {
                kept_before_selected += 1;
            }
            kept.push(item);
        }
    }
    *list = kept;
    let selection = selected.filter(|_| !list.is_empty()).map(|_| kept_before_selected.min(list.len() - 1));
    (removed, selection)
}

/// Put removed items back at their old positions (ascending, so each lands where it was; items
/// added at the end since then stay after them).
fn restore_at<T>(list: &mut Vec<T>, removed: Vec<(usize, T)>) {
    for (pos, item) in removed {
        let at = pos.min(list.len());
        list.insert(at, item);
    }
}

/// Whether a capture just added at index `new` takes the selection: only in the list view (not
/// under a menu, form or prompt) and only when nothing is selected or the selection is on the row
/// before it, the old end of the list (tail-follow). Otherwise browsing was hijacked, and an open
/// signal menu retargeted.
fn follows_new_capture(mode: InputMode, selected: Option<usize>, new: usize) -> bool {
    mode == InputMode::Normal && selected.is_none_or(|s| s + 1 >= new)
}

/// The selection `delta` rows away (clamped to the list), from the top when nothing is selected.
fn moved_selection(selected: Option<usize>, len: usize, delta: isize) -> Option<usize> {
    let last = len.checked_sub(1)?;
    let from = selected.unwrap_or(0).min(last);
    Some(from.saturating_add_signed(delta).min(last))
}

/// The row to highlight in a (possibly changed) action list: the remembered action's, else the
/// first (never the neighbour that slid under the highlight, such as "Delete Signal").
fn highlight_row(actions: &[SignalAction], remembered: Option<SignalAction>) -> usize {
    remembered.and_then(|a| actions.iter().position(|&b| b == a)).unwrap_or(0)
}

// ── Overlays, forms, paths ───────────────────────────────────────────────────

/// Lines PgUp / PgDn move the License, Credits and help overlays (as the timing overlay).
const OVERLAY_PAGE: usize = 10;

/// The overlay scroll after a key (None: not a scroll key), never past `max`, the last useful
/// offset the renderer reported (Down used to scroll without limit, so Up then seemed dead).
fn overlay_scrolled(scroll: usize, max: usize, code: KeyCode) -> Option<usize> {
    let scroll = scroll.min(max);
    Some(match code {
        KeyCode::Up | KeyCode::Char('k') => scroll.saturating_sub(1),
        KeyCode::Down | KeyCode::Char('j') => (scroll + 1).min(max),
        KeyCode::PageUp => scroll.saturating_sub(OVERLAY_PAGE),
        KeyCode::PageDown => (scroll + OVERLAY_PAGE).min(max),
        KeyCode::Home | KeyCode::Char('g') => 0,
        KeyCode::End | KeyCode::Char('G') => max,
        _ => return None,
    })
}

/// The export form's fields in order: the file name, then the .fob vehicle details (the .sub
/// exports have only the file name).
const EXPORT_FIELDS: [InputMode; 7] = [
    InputMode::ExportFilename,
    InputMode::FobMetaYear,
    InputMode::FobMetaMake,
    InputMode::FobMetaModel,
    InputMode::FobMetaRegion,
    InputMode::FobMetaCommand,
    InputMode::FobMetaNotes,
];

/// The capture-metadata form's fields (`i`).
const CAPTURE_META_FIELDS: [InputMode; 5] = [
    InputMode::CaptureMetaYear,
    InputMode::CaptureMetaMake,
    InputMode::CaptureMetaModel,
    InputMode::CaptureMetaRegion,
    InputMode::CaptureMetaCommand,
];

/// The field `step` places after (1) or before (-1) `field`, wrapping around.
fn step_field(fields: &[InputMode], field: InputMode, step: isize) -> InputMode {
    let n = fields.len() as isize;
    let at = fields.iter().position(|m| *m == field).unwrap_or(0) as isize;
    fields[(at + step).rem_euclid(n) as usize]
}

/// A character typed into the export file name: a space becomes `_` (spaces were dropped without a
/// word); characters a portable file name should not have are dropped.
fn filename_char(c: char) -> Option<char> {
    match c {
        ' ' => Some('_'),
        c if c.is_alphanumeric() || matches!(c, '_' | '-' | '.') => Some(c),
        _ => None,
    }
}

/// A `:load` argument as a path: surrounding quotes removed (`:load "My File.sub"`), `~` and `~/…`
/// expanded, then found like `:iq` finds its file: as given (relative to the working directory),
/// else in each of `dirs` (the import and export folders); folders too. Not found: the expanded
/// path, for the error message.
fn resolve_user_path(arg: &str, home: Option<&Path>, dirs: &[&Path]) -> PathBuf {
    let arg = arg.trim();
    let unquoted = ['"', '\'']
        .iter()
        .find_map(|&q| arg.strip_prefix(q).and_then(|a| a.strip_suffix(q)))
        .unwrap_or(arg);
    let path = match (unquoted.strip_prefix('~'), home) {
        (Some(""), Some(home)) => home.to_path_buf(),
        (Some(rest), Some(home)) if rest.starts_with('/') => home.join(&rest[1..]),
        _ => PathBuf::from(unquoted),
    };
    if let Some(file) = find_file(&path.to_string_lossy(), dirs) {
        return file;
    }
    let mut candidates = vec![path.clone()];
    if path.is_relative() {
        candidates.extend(dirs.iter().map(|d| d.join(&path)));
    }
    candidates.into_iter().find(|p| p.is_dir()).unwrap_or(path)
}

/// Rows the `:load` browser shows at once (layout.rs draws as many).
const BROWSER_ROWS: usize = 16;

/// "Quit? The KeeLoq key search is still running", "… and the IQ recording are still running".
fn quit_question(jobs: &[String]) -> String {
    let list = match jobs {
        [] => return "Quit KAT?".to_string(),
        [one] => one.clone(),
        [rest @ .., last] => format!("{} and {last}", rest.join(", ")),
    };
    let mut chars = list.chars();
    let first: String = chars.next().map(|c| c.to_uppercase().collect()).unwrap_or_default();
    let verb = if jobs.len() == 1 { "is" } else { "are" };
    format!("Quit? {first}{} {verb} still running", chars.as_str())
}

// ── Settings dropdown ────────────────────────────────────────────────────────

/// A value the settings dropdown applies.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum SettingChoice {
    Freq(u32),
    Hop(bool),
    Lna(u32),
    Vga(u32),
    Amp(bool),
    /// An entry of [TX_POWER_STEPS].
    TxStep(usize),
    /// The current TX power when it is none of the steps (`:txpower 35`).
    TxPower(u32, bool),
}

/// Dropdown rows: `presets`, with the current value first when it is none of them, labelled
/// "… (current)". Returns the rows and the current value's row.
fn rows_with_current(
    presets: Vec<(String, SettingChoice)>,
    current: Option<usize>,
    current_row: (String, SettingChoice),
) -> (Vec<(String, SettingChoice)>, usize) {
    match current {
        Some(row) => (presets, row),
        None => {
            let (label, choice) = current_row;
            let mut rows = vec![(format!("{label} (current)"), choice)];
            rows.extend(presets);
            (rows, 0)
        }
    }
}

impl App {
    /// What a key press means in the current mode (`key_code_for`); None: ignore it.
    pub fn key_code(&self, key: &KeyEvent) -> Option<KeyCode> {
        key_code_for(self.input_mode, key)
    }

    /// A new user action starts: the last message and error go. Errors win in the status bar, so
    /// an old one used to hide every later success.
    pub fn clear_messages(&mut self) {
        self.last_error = None;
        self.status_message = None;
    }

    /// One-line progress of the background work: a KeeLoq or PSA key search, a transmit queue of
    /// two or more. For the status bar title; None when nothing runs.
    pub fn job_status(&self) -> Option<String> {
        let jobs: Vec<String> =
            [self.keeloq_key_recovery_progress(), self.psa_key_recovery_progress(), self.tx_progress()]
                .into_iter()
                .flatten()
                .collect();
        (!jobs.is_empty()).then(|| jobs.join(" · "))
    }

    // ── Quit ──

    /// Work that quitting would cut short, as the quit prompt names it.
    pub fn running_jobs(&self) -> Vec<String> {
        let mut jobs = Vec::new();
        if self.keeloq_bf_job.is_some() {
            jobs.push("the KeeLoq key search".to_string());
        }
        if self.psa_recover_job.is_some() {
            jobs.push("the PSA key search".to_string());
        }
        if self.rx.iq_job.as_ref().is_some_and(|j| !j.is_finished()) {
            jobs.push("the IQ file decode".to_string());
        }
        if self.radio.as_ref().is_some_and(|r| r.recording_status().is_some()) {
            jobs.push("the IQ recording".to_string());
        }
        match self.queued_transmits() {
            0 => {}
            1 => jobs.push("a transmission".to_string()),
            n => jobs.push(format!("the transmit queue ({n} left)")),
        }
        jobs
    }

    /// `q`, Ctrl-C, `:q`: true = quit now. While work runs, ask first instead (it used to be cut
    /// short at once).
    pub fn request_quit(&mut self) -> bool {
        if self.running_jobs().is_empty() {
            return true;
        }
        self.input_mode = InputMode::ConfirmQuit;
        false
    }

    /// The quit prompt's question, e.g. "Quit? The KeeLoq key search is still running".
    pub fn quit_question(&self) -> String {
        quit_question(&self.running_jobs())
    }

    /// y / n of the quit and `:delete all` prompts. True: quit now.
    pub fn confirm_prompt_key(&mut self, code: KeyCode) -> bool {
        let yes = matches!(code, KeyCode::Char('y' | 'Y'));
        let no = matches!(code, KeyCode::Char('n' | 'N') | KeyCode::Esc);
        match self.input_mode {
            InputMode::ConfirmQuit if yes => return true,
            InputMode::ConfirmQuit if no => self.input_mode = InputMode::Normal,
            InputMode::ConfirmDeleteAll if yes => {
                self.input_mode = InputMode::Normal;
                self.delete_all_captures();
            }
            InputMode::ConfirmDeleteAll if no => {
                self.input_mode = InputMode::Normal;
                self.status_message = Some("Delete cancelled".to_string());
            }
            _ => {}
        }
        false
    }

    // ── Overlays ──

    /// `?`, F1, `:help`: the help overlay.
    pub fn open_help(&mut self) {
        self.input_mode = InputMode::Help;
        self.overlay_scroll = 0;
    }

    /// Keys of the License, Credits and help overlays: Up / Down / PgUp / PgDn / Home / End scroll,
    /// stopping at the end the renderer reports (`overlay_scroll_max`); Esc, Enter and q close, ?
    /// closes the help.
    pub fn overlay_key(&mut self, code: KeyCode) {
        let close = matches!(code, KeyCode::Esc | KeyCode::Enter | KeyCode::Char('q'))
            || (code == KeyCode::Char('?') && self.input_mode == InputMode::Help);
        if close {
            self.input_mode = InputMode::Normal;
            self.overlay_scroll = 0;
        } else if let Some(scroll) = overlay_scrolled(self.overlay_scroll, self.overlay_scroll_max.get(), code) {
            self.overlay_scroll = scroll;
        }
    }

    // ── Captures list ──

    /// PgUp / PgDn (`pages` -1 / 1): a page of the list, the rows the last frame showed.
    pub fn page_captures(&mut self, pages: isize) {
        let rows = self.list_page_rows.get().max(1) as isize;
        self.select_capture_row(moved_selection(self.selected_capture, self.captures.len(), pages * rows));
    }

    /// Home / g (`last` false), End / G (true).
    pub fn select_capture_edge(&mut self, last: bool) {
        let end = self.captures.len().checked_sub(1);
        self.select_capture_row(end.map(|end| if last { end } else { 0 }));
    }

    fn select_capture_row(&mut self, row: Option<usize>) {
        if row.is_some() {
            self.selected_capture = row;
            self.ensure_selection_visible();
        }
    }

    /// A capture was just added at the end of the list: select it only per `follows_new_capture`.
    pub(super) fn follow_new_capture(&mut self) {
        let new = self.captures.len().saturating_sub(1);
        if follows_new_capture(self.input_mode, self.selected_capture, new) {
            self.selected_capture = Some(new);
            self.ensure_selection_visible();
        }
    }

    // ── Delete and undo ──

    /// `d`, `:delete <IDs>`, the menu's "Delete Signal": remove the captures an ID list names
    /// ("4", "1-3, 7", "4 5"; `:delete` took one ID only), keeping the selection on the same
    /// capture. `u` brings them back.
    pub(super) fn delete_capture(&mut self, spec: &str) -> Result<()> {
        let items = match parse_id_items(spec) {
            Ok(items) => items,
            Err(e) => {
                self.last_error = Some(e);
                return Ok(());
            }
        };
        let existing: Vec<u32> = self.captures.iter().map(|c| c.id).collect();
        let sel = resolve_ids(&items, &existing);
        if sel.found.is_empty() {
            self.last_error = Some(not_found_text(&sel));
            return Ok(());
        }
        let ids: HashSet<u32> = sel.found.iter().copied().collect();
        let (removed, selected) = remove_where(&mut self.captures, |c| ids.contains(&c.id), self.selected_capture);
        self.selected_capture = selected;
        let deleted: Vec<u32> = removed.iter().map(|(_, c)| c.id).collect();
        self.input.undo = removed;
        let mut msg = format!("Deleted {}", ids_text(&deleted));
        if !sel.missing.is_empty() {
            msg.push_str(&format!("; {} not found", ranges_text(&sel.missing)));
        }
        msg.push_str(" (u: undo)");
        self.status_message = Some(msg);
        Ok(())
    }

    /// `:delete all`: ask first (it wiped everything at once, and the autosave then deleted
    /// session.json).
    pub(super) fn request_delete_all(&mut self) {
        if self.captures.is_empty() {
            self.status_message = Some("No captures to delete".to_string());
        } else {
            self.input_mode = InputMode::ConfirmDeleteAll;
        }
    }

    /// Delete every capture (y at the `:delete all` prompt). `u` brings them back; the saved
    /// session follows on the next autosave (`:clear` also deletes the session file and backup).
    fn delete_all_captures(&mut self) {
        let count = self.captures.len();
        let (removed, _) = remove_where(&mut self.captures, |_| true, None);
        self.input.undo = removed;
        self.selected_capture = None;
        self.scroll_offset = 0;
        self.status_message = Some(format!("Deleted all {count} captures (u: undo)"));
    }

    /// `u`: put the captures the last delete removed back where they were, the first one selected.
    pub fn undo_delete(&mut self) {
        let removed = std::mem::take(&mut self.input.undo);
        let Some(first) = removed.first().map(|(pos, _)| *pos) else {
            self.status_message = Some("Nothing to undo".to_string());
            return;
        };
        let at = first.min(self.captures.len());
        let ids: Vec<u32> = removed.iter().map(|(_, c)| c.id).collect();
        restore_at(&mut self.captures, removed);
        self.selected_capture = Some(at);
        self.ensure_selection_visible();
        self.status_message = Some(format!("Restored {}", ids_text(&ids)));
    }

    /// `:clear` cannot be undone: forget the last delete.
    pub fn forget_deleted(&mut self) {
        self.input.undo.clear();
    }

    // ── Transmit lists ──

    /// Why nothing can be transmitted, if so (the per-capture functions say the same).
    fn tx_unavailable(&self) -> Option<&'static str> {
        match &self.radio {
            None => Some("No radio device connected"),
            Some(r) if !r.supports_tx() => Some("Transmit not available – RTL-SDR is receive-only"),
            Some(_) => None,
        }
    }

    /// `:replay` (`command` None) and `:lock` / `:unlock` / `:trunk` / `:panic` with an ID list:
    /// queue the captures it names, in its order. One capture keeps its own message; a list says
    /// what was queued and, once sent, ends with a summary such as "Sent Lock to 3 of 5 (#04, #05
    /// not found)" (only the last error used to show).
    pub(super) fn transmit_id_list(&mut self, spec: &str, command: Option<ButtonCommand>) -> Result<()> {
        let items = match parse_id_items(spec) {
            Ok(items) => items,
            Err(e) => {
                self.last_error = Some(e);
                return Ok(());
            }
        };
        if let Some(why) = self.tx_unavailable() {
            self.last_error = Some(why.to_string());
            return Ok(());
        }
        let existing: Vec<u32> = self.captures.iter().map(|c| c.id).collect();
        let sel = resolve_ids(&items, &existing);
        let send = |app: &mut App, id: u32| match command {
            Some(c) => app.transmit_one_command(id, c),
            None => app.replay_capture(id),
        };
        if sel.requested == 1 {
            // One capture: its own message, or why it was not sent ("Capture 7 not found").
            let id = sel.found.first().copied().or_else(|| sel.missing.first().map(|r| r.0)).unwrap_or_default();
            return send(self, id);
        }
        let mut sent = 0;
        let mut failed = Vec::new();
        for &id in &sel.found {
            let queued = self.queued_transmits();
            self.last_error = None;
            send(self, id)?;
            if self.queued_transmits() > queued {
                sent += 1;
            } else {
                failed.push((id, self.last_error.take().unwrap_or_else(|| "not sent".to_string())));
            }
        }
        let (doing, done) = match command {
            Some(c) => (format!("Sending {c:?} to"), format!("Sent {c:?} to")),
            None => ("Replaying".to_string(), "Replayed".to_string()),
        };
        self.clear_messages();
        if sent == 0 {
            self.last_error = Some(list_summary(&done, 0, &sel, &failed));
        } else {
            self.status_message = Some(format!("{}…", list_summary(&doing, sent, &sel, &failed)));
            self.input.tx_summary = Some(list_summary(&done, sent, &sel, &failed));
        }
        Ok(())
    }

    // ── Signal menu ──

    /// The capture the signal menu acts on: while the menu is open, the one it was opened for (a
    /// capture arriving meanwhile used to retarget Replay / Export / Delete); else the selected
    /// one. None when that capture is gone. The menu's header should show this one.
    pub fn signal_menu_capture(&self) -> Option<&Capture> {
        match self.input.menu_capture {
            Some(id) if self.input_mode == InputMode::SignalMenu => self.captures.iter().find(|c| c.id == id),
            _ => self.selected_capture.and_then(|i| self.captures.get(i)),
        }
    }

    /// Enter in the list: open the selected capture's action menu.
    pub fn open_signal_menu(&mut self) {
        let Some(id) = self.selected_capture.and_then(|i| self.captures.get(i)).map(|c| c.id) else {
            return;
        };
        self.input.menu_capture = Some(id);
        self.input_mode = InputMode::SignalMenu;
        self.signal_menu_index = 0;
        self.input.menu_action = self.available_signal_actions().first().copied();
    }

    /// Up (`down` false) / Down in the signal menu.
    pub fn signal_menu_move(&mut self, down: bool) {
        let actions = self.available_signal_actions();
        let last = actions.len().saturating_sub(1);
        self.signal_menu_index =
            if down { (self.signal_menu_index + 1).min(last) } else { self.signal_menu_index.saturating_sub(1) };
        self.input.menu_action = actions.get(self.signal_menu_index).copied();
    }

    /// Every tick while the menu is open: close it when its capture is gone, and keep the highlight
    /// on the same action when the list changes under it (a key search starting or ending, a key
    /// found), so Enter does what the user picked.
    pub fn sync_signal_menu(&mut self) {
        if self.input_mode != InputMode::SignalMenu {
            return;
        }
        if self.signal_menu_capture().is_none() {
            self.input_mode = InputMode::Normal;
            self.last_error = Some("The menu's capture is no longer in the list".to_string());
            return;
        }
        let actions = self.available_signal_actions();
        self.signal_menu_index = highlight_row(&actions, self.input.menu_action);
        self.input.menu_action = actions.get(self.signal_menu_index).copied();
    }

    // ── Forms ──

    /// The text of a form field.
    fn form_text(&mut self, field: InputMode) -> Option<&mut String> {
        Some(match field {
            InputMode::ExportFilename => &mut self.export_filename,
            InputMode::FobMetaYear => &mut self.fob_meta_year,
            InputMode::FobMetaMake => &mut self.fob_meta_make,
            InputMode::FobMetaModel => &mut self.fob_meta_model,
            InputMode::FobMetaRegion => &mut self.fob_meta_region,
            InputMode::FobMetaCommand => &mut self.fob_meta_command,
            InputMode::FobMetaNotes => &mut self.fob_meta_notes,
            InputMode::CaptureMetaYear => &mut self.capture_meta_year,
            InputMode::CaptureMetaMake => &mut self.capture_meta_make,
            InputMode::CaptureMetaModel => &mut self.capture_meta_model,
            InputMode::CaptureMetaRegion => &mut self.capture_meta_region,
            InputMode::CaptureMetaCommand => &mut self.capture_meta_command,
            _ => return None,
        })
    }

    /// Keys of the export form and the capture-metadata form (`i`): Tab / Down go to the next
    /// field and Shift-Tab / Up to the previous one (both forms only went forward); Enter goes to
    /// the next field or, on the last one, saves; Esc cancels. Each key is a new action, so the
    /// last message (such as "Filename cannot be empty") goes.
    pub fn form_field_key(&mut self, code: KeyCode) {
        self.clear_messages();
        let field = self.input_mode;
        let export = EXPORT_FIELDS.contains(&field);
        let fields: &[InputMode] = match (export, self.export_format) {
            (false, _) => &CAPTURE_META_FIELDS,
            (true, Some(ExportFormat::Fob)) => &EXPORT_FIELDS,
            (true, _) => &EXPORT_FIELDS[..1],
        };
        let is_year = matches!(field, InputMode::FobMetaYear | InputMode::CaptureMetaYear);
        match code {
            KeyCode::Esc if export => self.cancel_export_form(),
            KeyCode::Esc => self.cancel_capture_meta(),
            KeyCode::Tab | KeyCode::Down => self.input_mode = step_field(fields, field, 1),
            KeyCode::BackTab | KeyCode::Up => self.input_mode = step_field(fields, field, -1),
            KeyCode::Enter if field == InputMode::ExportFilename && self.export_filename.trim().is_empty() => {
                self.last_error = Some("Filename cannot be empty".to_string());
            }
            KeyCode::Enter if fields.last() != Some(&field) => self.input_mode = step_field(fields, field, 1),
            KeyCode::Enter if export => self.submit_export_form(),
            KeyCode::Enter => self.save_capture_meta(),
            KeyCode::Backspace => {
                if let Some(text) = self.form_text(field) {
                    text.pop();
                }
            }
            KeyCode::Char(c) => {
                let c = match field {
                    InputMode::ExportFilename => filename_char(c),
                    _ if is_year => c.is_ascii_digit().then_some(c),
                    _ => Some(c),
                };
                if let (Some(c), Some(text)) = (c, self.form_text(field)) {
                    if !is_year || text.len() < 4 {
                        text.push(c);
                    }
                }
            }
            _ => {}
        }
    }

    fn cancel_export_form(&mut self) {
        self.export_capture_id = None;
        self.export_format = None;
        self.input_mode = InputMode::Normal;
    }

    /// Enter on the export form's last field: write the file. A failed write keeps the form open
    /// with the reason (the error used to end KAT).
    fn submit_export_form(&mut self) {
        if self.export_filename.trim().is_empty() {
            self.last_error = Some("Filename cannot be empty".to_string());
            self.input_mode = InputMode::ExportFilename;
            return;
        }
        // The export functions report a failure in the status line themselves (and forget the
        // capture): keep the form and what was typed into it, so Enter can try again once the
        // name or the export folder is fixed.
        let (capture_id, format) = (self.export_capture_id, self.export_format);
        self.last_error = None;
        let written = match self.export_format {
            Some(ExportFormat::Fob) => self.complete_fob_export(),
            Some(ExportFormat::Flipper) => self.complete_flipper_export(),
            Some(ExportFormat::FlipperKey) => self.complete_flipper_key_export(),
            None => Ok(()),
        };
        if let Err(e) = written {
            self.last_error = Some(format!("Export failed: {e:#}"));
        }
        if self.last_error.is_some() {
            self.export_capture_id = capture_id;
            self.export_format = format;
        } else {
            self.input_mode = InputMode::Normal;
        }
    }

    // ── :load browser ──

    /// Keys of the `:load` browser: Up / Down / PgUp / PgDn / Home / End select; Enter opens a
    /// folder or loads a file; Right / l open a folder; Left / Backspace / h go to the parent.
    pub fn load_browser_key(&mut self, code: KeyCode) -> Result<()> {
        let selected = self.load_browser_selected;
        match code {
            KeyCode::Esc => self.close_load_browser(),
            KeyCode::Enter => self.load_browser_enter()?,
            KeyCode::Right | KeyCode::Char('l') => {
                if self.load_browser_entries.get(selected).is_some_and(|e| e.2) {
                    self.load_browser_enter()?;
                }
            }
            KeyCode::Left | KeyCode::Backspace | KeyCode::Char('h') => self.load_browser_up()?,
            KeyCode::Up | KeyCode::Char('k') => self.load_browser_select(selected.saturating_sub(1)),
            KeyCode::Down | KeyCode::Char('j') => self.load_browser_select(selected + 1),
            KeyCode::PageUp => self.load_browser_select(selected.saturating_sub(BROWSER_ROWS)),
            KeyCode::PageDown => self.load_browser_select(selected + BROWSER_ROWS),
            KeyCode::Home | KeyCode::Char('g') => self.load_browser_select(0),
            KeyCode::End | KeyCode::Char('G') => self.load_browser_select(usize::MAX),
            _ => {}
        }
        Ok(())
    }

    /// Select browser row `row` (clamped) and scroll it into view.
    fn load_browser_select(&mut self, row: usize) {
        self.load_browser_selected = row.min(self.load_browser_entries.len().saturating_sub(1));
        let selected = self.load_browser_selected;
        if selected < self.load_browser_scroll {
            self.load_browser_scroll = selected;
        } else if selected >= self.load_browser_scroll + BROWSER_ROWS {
            self.load_browser_scroll = selected + 1 - BROWSER_ROWS;
        }
    }

    /// Backspace / Left / h / "..": the parent folder, with the folder just left selected.
    pub(super) fn load_browser_up(&mut self) -> Result<()> {
        let Some(parent) = self.load_browser_cwd.parent().map(Path::to_path_buf) else {
            return Ok(());
        };
        let left = std::mem::replace(&mut self.load_browser_cwd, parent);
        self.load_browser_selected = 0;
        self.load_browser_scroll = 0;
        self.refresh_load_browser_entries()?;
        if let Some(row) = self.load_browser_entries.iter().position(|e| e.1 == left) {
            self.load_browser_select(row);
        }
        Ok(())
    }

    /// `:load`'s argument as a path (`resolve_user_path` with the home, import and export folders).
    pub(super) fn resolve_load_arg(&self, arg: &str) -> PathBuf {
        let (import, export) = (self.storage.import_dir(), self.storage.export_dir());
        resolve_user_path(arg, dirs::home_dir().as_deref(), &[import.as_path(), export.as_path()])
    }

    // ── Settings dropdown ──

    /// Rows of the settings dropdown for the selected setting, with the value Enter applies, and
    /// the row of the current value. A current value that is no preset (`:freq 433.42`, `:vga 30`,
    /// `:txpower 35`) is the first row, "… (current)": the first preset used to be highlighted
    /// instead, so Enter applied 300 MHz, 0 dB or the highest TX power.
    pub(super) fn settings_choices(&self) -> (Vec<(String, SettingChoice)>, usize) {
        let field = SettingsField::ALL[self.settings_field_index.min(SettingsField::ALL.len() - 1)];
        let on_off = |on: bool, choice: fn(bool) -> SettingChoice| {
            (vec![("ON".to_string(), choice(true)), ("OFF".to_string(), choice(false))], if on { 0 } else { 1 })
        };
        let gains = |steps: &[u32], gain: u32, choice: fn(u32) -> SettingChoice| {
            rows_with_current(
                steps.iter().map(|&g| (format!("{g} dB"), choice(g))).collect(),
                steps.iter().position(|&g| g == gain),
                (format!("{gain} dB"), choice(gain)),
            )
        };
        match field {
            SettingsField::Freq => rows_with_current(
                PRESET_FREQUENCIES.iter().map(|&(hz, label)| (label.to_string(), SettingChoice::Freq(hz))).collect(),
                PRESET_FREQUENCIES.iter().position(|&(hz, _)| hz == self.frequency),
                (format!("{} MHz", format_mhz(self.frequency)), SettingChoice::Freq(self.frequency)),
            ),
            SettingsField::Hop => on_off(self.rx.hop_enabled, SettingChoice::Hop),
            SettingsField::Lna => gains(&LNA_STEPS, self.lna_gain, SettingChoice::Lna),
            SettingsField::Vga => gains(&VGA_STEPS, self.vga_gain, SettingChoice::Vga),
            SettingsField::Amp => on_off(self.amp_enabled, SettingChoice::Amp),
            SettingsField::TxPower => rows_with_current(
                TX_POWER_STEPS.iter().enumerate().map(|(i, &(g, a))| (tx_power_label(g, a), SettingChoice::TxStep(i))).collect(),
                self.tx_power_value_index(),
                (tx_power_label(self.tx_vga_gain, self.tx_amp), SettingChoice::TxPower(self.tx_vga_gain, self.tx_amp)),
            ),
        }
    }

    /// The settings dropdown's rows for the selected setting: the renderer draws these, so the
    /// highlighted row (`settings_value_index`) is the value Enter applies.
    pub fn settings_value_rows(&self) -> Vec<String> {
        self.settings_choices().0.into_iter().map(|(label, _)| label).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn id_lists_take_commas_spaces_hashes_and_ranges_either_way() {
        use IdItem::{One, Range};
        assert_eq!(parse_id_items("4"), Ok(vec![One(4)]));
        assert_eq!(parse_id_items("1, 3,5"), Ok(vec![One(1), One(3), One(5)]));
        assert_eq!(parse_id_items(" 1 3  5 "), Ok(vec![One(1), One(3), One(5)]));
        assert_eq!(parse_id_items("#2 #4-#6, 9-7"), Ok(vec![One(2), Range(4, 6), Range(9, 7)]));
        assert_eq!(parse_id_items("1 - 5, 7 -8 10- 12"), Ok(vec![Range(1, 5), Range(7, 8), Range(10, 12)]));
        for bad in ["", " , ", "x", "1-", "-5", "1-2-3", "1, -5", "##4", "4294967296", "1.5"] {
            assert!(parse_id_items(bad).is_err(), "{bad:?}");
        }
    }

    #[test]
    fn id_lists_resolve_against_the_captures_without_listing_ranges() {
        let existing = [1, 2, 3, 6, 7];
        let sel = resolve_ids(&parse_id_items("1-5").unwrap(), &existing);
        assert_eq!(sel, IdSelection { found: vec![1, 2, 3], requested: 5, missing: vec![(4, 5)] });
        assert_eq!(sel.missing_count(), 2);
        // Backwards ranges run backwards; repeats are kept; missing IDs are merged.
        let sel = resolve_ids(&parse_id_items("7-6, 2, 2, 9, 8").unwrap(), &existing);
        assert_eq!(sel.found, vec![7, 6, 2, 2]);
        assert_eq!((sel.requested, sel.missing), (6, vec![(8, 9)]));
        // A huge range costs nothing (it used to allocate 16 GB and abort).
        let sel = resolve_ids(&parse_id_items("1-4000000000").unwrap(), &existing);
        assert_eq!(sel.found, vec![1, 2, 3, 6, 7]);
        assert_eq!(sel.requested, 4_000_000_000);
        assert_eq!(sel.missing, vec![(4, 5), (8, 4_000_000_000)]);
        let sel = resolve_ids(&[IdItem::Range(u32::MAX, 0)], &[0, u32::MAX]);
        assert_eq!((sel.found, sel.requested, sel.missing), (vec![u32::MAX, 0], 1 << 32, vec![(1, u32::MAX - 1)]));
    }

    #[test]
    fn id_texts_and_summaries() {
        assert_eq!(ids_text(&[4]), "#04");
        assert_eq!(ids_text(&[5, 4]), "#04, #05");
        assert_eq!(ids_text(&[1, 2, 3, 7, 12]), "#01-#03, #07, #12");
        assert_eq!(ranges_text(&[(1, 1), (3, 3), (5, 5), (7, 7), (9, 9)]), "#01, #03, #05, #07, …");
        let sel = resolve_ids(&parse_id_items("1-5").unwrap(), &[1, 2, 3]);
        assert_eq!(list_summary("Sent Lock to", 3, &sel, &[]), "Sent Lock to 3 of 5 (#04, #05 not found)");
        assert_eq!(not_found_text(&sel), "Captures #04, #05 not found");
        let failed = vec![(2, "no raw signal".to_string())];
        assert_eq!(list_summary("Replayed", 2, &sel, &failed), "Replayed 2 of 5 (#02: no raw signal; #04, #05 not found)");
        let all = resolve_ids(&parse_id_items("1-3").unwrap(), &[1, 2, 3]);
        assert_eq!(list_summary("Replayed", 3, &all, &[]), "Replayed 3 of 3");
        let one = resolve_ids(&parse_id_items("#9").unwrap(), &[1]);
        assert_eq!(not_found_text(&one), "Capture #09 not found");
    }

    #[test]
    fn deleting_keeps_the_selection_on_its_capture_and_undo_puts_them_back() {
        // #5 (row 4) selected; deleting #2 above it keeps #5 (it used to move to #6).
        let mut list = vec![1, 2, 3, 4, 5, 6];
        let (removed, selected) = remove_where(&mut list, |&c| c == 2, Some(4));
        assert_eq!((list.clone(), selected), (vec![1, 3, 4, 5, 6], Some(3)));
        restore_at(&mut list, removed);
        assert_eq!(list, vec![1, 2, 3, 4, 5, 6]);
        // The selected one goes: the next one is selected, or the one before at the end.
        let mut list = vec![1, 2, 3];
        assert_eq!(remove_where(&mut list, |&c| c == 2, Some(1)).1, Some(1));
        assert_eq!(list, vec![1, 3]);
        let mut list = vec![1, 2, 3];
        assert_eq!(remove_where(&mut list, |&c| c == 3, Some(2)).1, Some(1));
        let mut list = vec![1, 2, 3];
        assert_eq!(remove_where(&mut list, |_| true, Some(0)).1, None);
        let mut list = vec![1, 2, 3];
        assert_eq!(remove_where(&mut list, |&c| c == 1, None).1, None);
        let mut list = vec![1, 2, 3];
        assert_eq!(remove_where(&mut list, |&c| c == 1, Some(7)).1, Some(1), "a stale selection is clamped");
        // Several at once come back in place, also after new captures arrived at the end.
        let mut list = vec![1, 2, 3, 4, 5];
        let (removed, selected) = remove_where(&mut list, |&c| c % 2 == 0, Some(3));
        assert_eq!((list.clone(), selected), (vec![1, 3, 5], Some(2)));
        list.push(6);
        restore_at(&mut list, removed);
        assert_eq!(list, vec![1, 2, 3, 4, 5, 6]);
    }

    #[test]
    fn new_captures_take_the_selection_only_at_the_tail_of_the_list() {
        assert!(follows_new_capture(InputMode::Normal, None, 0));
        assert!(follows_new_capture(InputMode::Normal, None, 5));
        assert!(follows_new_capture(InputMode::Normal, Some(4), 5), "selection on the last row: follow");
        assert!(!follows_new_capture(InputMode::Normal, Some(2), 5), "browsing: stay");
        assert!(!follows_new_capture(InputMode::SignalMenu, Some(4), 5), "an open menu keeps its capture");
        assert!(!follows_new_capture(InputMode::Command, None, 5));
    }

    #[test]
    fn list_paging() {
        assert_eq!(moved_selection(None, 0, 10), None);
        assert_eq!(moved_selection(None, 30, 10), Some(10));
        assert_eq!(moved_selection(Some(25), 30, 10), Some(29));
        assert_eq!(moved_selection(Some(5), 30, -10), Some(0));
        assert_eq!(moved_selection(Some(99), 30, -10), Some(19), "a stale selection is clamped first");
    }

    #[test]
    fn overlay_scrolling_stops_at_the_end() {
        assert_eq!(overlay_scrolled(0, 40, KeyCode::Down), Some(1));
        assert_eq!(overlay_scrolled(40, 40, KeyCode::Down), Some(40));
        assert_eq!(overlay_scrolled(99, 40, KeyCode::Up), Some(39), "past the end: Up moves at once");
        assert_eq!(overlay_scrolled(35, 40, KeyCode::PageDown), Some(40));
        assert_eq!(overlay_scrolled(5, 40, KeyCode::PageUp), Some(0));
        assert_eq!(overlay_scrolled(5, 40, KeyCode::End), Some(40));
        assert_eq!(overlay_scrolled(5, 40, KeyCode::Char('g')), Some(0));
        assert_eq!(overlay_scrolled(5, 0, KeyCode::Down), Some(0), "text that fits does not scroll");
        assert_eq!(overlay_scrolled(5, 40, KeyCode::Char('x')), None);
    }

    #[test]
    fn ctrl_and_alt_keys() {
        let key = |code, modifiers| KeyEvent::new(code, modifiers);
        let (ctrl, alt, shift) = (KeyModifiers::CONTROL, KeyModifiers::ALT, KeyModifiers::SHIFT);
        let c = KeyCode::Char('c');
        assert_eq!(key_code_for(InputMode::Normal, &key(c, ctrl)), Some(KeyCode::Char('q')), "quits like q");
        assert_eq!(key_code_for(InputMode::SendButton, &key(c, ctrl)), Some(KeyCode::Esc), "no hex C");
        assert_eq!(key_code_for(InputMode::ExportFilename, &key(KeyCode::Char('C'), ctrl | shift)), Some(KeyCode::Esc));
        assert_eq!(key_code_for(InputMode::Command, &key(KeyCode::Char('h'), ctrl)), Some(KeyCode::Backspace));
        assert_eq!(key_code_for(InputMode::Command, &key(KeyCode::Char('d'), ctrl)), None);
        assert_eq!(key_code_for(InputMode::Normal, &key(KeyCode::Char('d'), alt)), None, "Alt-d deletes nothing");
        assert_eq!(key_code_for(InputMode::Command, &key(KeyCode::Char('@'), ctrl | alt)), Some(KeyCode::Char('@')), "AltGr");
        assert_eq!(key_code_for(InputMode::Command, &key(KeyCode::Char('A'), shift)), Some(KeyCode::Char('A')));
        assert_eq!(key_code_for(InputMode::Normal, &key(KeyCode::Up, ctrl)), Some(KeyCode::Up));
    }

    #[test]
    fn form_fields_and_file_name_characters() {
        assert_eq!(step_field(&EXPORT_FIELDS, InputMode::ExportFilename, 1), InputMode::FobMetaYear);
        assert_eq!(step_field(&EXPORT_FIELDS, InputMode::ExportFilename, -1), InputMode::FobMetaNotes);
        assert_eq!(step_field(&EXPORT_FIELDS, InputMode::FobMetaNotes, 1), InputMode::ExportFilename);
        assert_eq!(step_field(&EXPORT_FIELDS[..1], InputMode::ExportFilename, 1), InputMode::ExportFilename);
        assert_eq!(step_field(&CAPTURE_META_FIELDS, InputMode::CaptureMetaMake, -1), InputMode::CaptureMetaYear);
        assert_eq!(filename_char(' '), Some('_'));
        assert_eq!(filename_char('é'), Some('é'));
        assert_eq!(filename_char('.'), Some('.'));
        assert_eq!(filename_char('/'), None);
    }

    #[test]
    fn quit_questions() {
        let jobs = |v: &[&str]| v.iter().map(|s| s.to_string()).collect::<Vec<_>>();
        assert_eq!(quit_question(&[]), "Quit KAT?");
        assert_eq!(quit_question(&jobs(&["the KeeLoq key search"])), "Quit? The KeeLoq key search is still running");
        assert_eq!(
            quit_question(&jobs(&["the IQ file decode", "the IQ recording", "the transmit queue (5 left)"])),
            "Quit? The IQ file decode, the IQ recording and the transmit queue (5 left) are still running"
        );
    }

    #[test]
    fn the_menu_highlight_follows_its_action() {
        use SignalAction::*;
        let before = [CancelKeyRecovery, Replay, ExportFob, ExportFlipper, TimingAnalysis, Delete];
        let after = [Replay, ExportFob, ExportFlipper, TimingAnalysis, Delete];
        assert_eq!(highlight_row(&before, Some(ExportFlipper)), 3);
        assert_eq!(highlight_row(&after, Some(ExportFlipper)), 2, "the row moved up with its action");
        assert_eq!(highlight_row(&after, Some(CancelKeyRecovery)), 0, "gone: the top, not a neighbour");
        assert_eq!(highlight_row(&after, None), 0);
    }

    #[test]
    fn settings_rows_keep_a_current_value_that_is_no_preset() {
        let presets = || vec![("8 dB".to_string(), SettingChoice::Vga(8)), ("16 dB".to_string(), SettingChoice::Vga(16))];
        assert_eq!(rows_with_current(presets(), Some(1), ("16 dB".to_string(), SettingChoice::Vga(16))), (presets(), 1));
        let (rows, current) = rows_with_current(presets(), None, ("30 dB".to_string(), SettingChoice::Vga(30)));
        assert_eq!(current, 0);
        assert_eq!(rows[0], ("30 dB (current)".to_string(), SettingChoice::Vga(30)));
        assert_eq!(rows[1..], presets()[..]);
    }

    #[test]
    fn load_paths_resolve_like_iq_files() {
        let base = std::env::temp_dir().join(format!("kat_input_load_{}", std::process::id()));
        let (home, import, export) = (base.join("home"), base.join("import"), base.join("export"));
        for d in [&home, &import.join("FORD"), &export] {
            std::fs::create_dir_all(d).unwrap();
        }
        std::fs::write(import.join("My Fob.sub"), "x").unwrap();
        std::fs::write(export.join("rec.cs8"), "x").unwrap();
        std::fs::write(home.join("h.sub"), "x").unwrap();
        let dirs = [import.as_path(), export.as_path()];
        let resolve = |arg: &str| resolve_user_path(arg, Some(&home), &dirs);
        assert_eq!(resolve("\"My Fob.sub\""), import.join("My Fob.sub"), "quotes go; found in the import folder");
        assert_eq!(resolve(" 'My Fob.sub' "), import.join("My Fob.sub"));
        assert_eq!(resolve("rec.cs8"), export.join("rec.cs8"), "then in the export folder");
        assert_eq!(resolve("FORD"), import.join("FORD"), "folders too");
        assert_eq!(resolve("~"), home);
        assert_eq!(resolve("~/h.sub"), home.join("h.sub"));
        let absolute = import.join("My Fob.sub");
        assert_eq!(resolve(&absolute.to_string_lossy()), absolute);
        assert_eq!(resolve("nothing.sub"), PathBuf::from("nothing.sub"), "not found: as typed");
        let _ = std::fs::remove_dir_all(&base);
    }
}
