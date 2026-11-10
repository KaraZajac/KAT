//! `:new` — "Add Manually" remotes (`docs/add_manually.md`) — and the signal menu's "Send button…".
//!
//! - **`:new [type…]`** opens the type picker (Flipper-ARF's / doctor-firmware's "Add Manually"
//!   list, grouped as their menu orders it; types KAT cannot create are shown with the reason).
//!   Typing filters the list; Enter creates the selected type with a fresh random serial and the
//!   type's default button / counter / seed (doctor-firmware's "Add Manually"); Tab opens the
//!   fields first (ARF's set_key / set_serial / set_button / set_counter / set_seed screens).
//!   With words after `:new` that match exactly one available type, it is created directly.
//! - The new remote is the type's key file loaded like any key file (`capture_from_key`): a
//!   capture without raw pairs, transmitted through its protocol's encoder ("Send next code",
//!   "Send button…", exports). Its File line names the type.
//! - **Send button…** (gate / garage remotes, KeeLoq barrier and alarm manufacturers): the
//!   protocol's native buttons (`protocols::gate_buttons`), or a typed hex code; Enter sends the
//!   next code with that button.

use crossterm::event::KeyCode;

use super::{App, InputMode, SignalAction};
use crate::protocols::add_manually::{self, Field, Gen, RandSource, Rng, TYPES};
use crate::protocols::gate_buttons;

/// The `:new` type picker.
pub struct NewRemotePicker {
    /// Typed filter (words matched against the type names).
    pub filter: String,
    /// Selected row among the visible types.
    pub selected: usize,
    /// First visible row of the list.
    pub scroll: usize,
    /// Per [TYPES] entry: available, or why not.
    pub availability: Vec<Result<(), String>>,
}

/// A row of the picker: a group heading or a type (index into [TYPES]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PickerRow {
    Heading(add_manually::Group),
    Type(usize),
}

impl NewRemotePicker {
    /// The [TYPES] indices the filter shows, in menu order.
    pub fn visible(&self) -> Vec<usize> {
        add_manually::find(&self.filter)
    }

    /// The rows to draw: the visible types under their group headings.
    pub fn rows(&self) -> Vec<PickerRow> {
        let mut rows = Vec::new();
        let mut group = None;
        for i in self.visible() {
            if group != Some(TYPES[i].group) {
                group = Some(TYPES[i].group);
                rows.push(PickerRow::Heading(TYPES[i].group));
            }
            rows.push(PickerRow::Type(i));
        }
        rows
    }

    /// The selected type (index into [TYPES]).
    pub fn selected_type(&self) -> Option<usize> {
        self.visible().get(self.selected).copied()
    }
}

/// The fields of a type before it is created.
pub struct NewRemoteForm {
    /// Index into [TYPES].
    pub type_index: usize,
    /// The parameters drawn for it (edited fields are written back on Enter).
    pub gen: Gen,
    /// (field, hex text, width in bytes).
    pub fields: Vec<(Field, String, usize)>,
    pub focus: usize,
    pub error: Option<String>,
}

/// The "Send button…" list of a capture.
pub struct SendButtonMenu {
    pub capture_id: u32,
    /// (native code, label).
    pub buttons: Vec<(u8, String)>,
    pub selected: usize,
    /// A typed hex code (sent instead of the selected button when not empty).
    pub custom: String,
}

#[derive(Default)]
pub(super) struct AddManuallyState {
    picker: Option<NewRemotePicker>,
    form: Option<NewRemoteForm>,
    send_button: Option<SendButtonMenu>,
}

/// Rows the picker list shows at once (the UI draws as many).
pub const PICKER_VISIBLE_ROWS: usize = 18;

impl App {
    /// The open `:new` picker.
    pub fn new_remote_picker(&self) -> Option<&NewRemotePicker> {
        self.add_manually.picker.as_ref()
    }

    /// The open field form.
    pub fn new_remote_form(&self) -> Option<&NewRemoteForm> {
        self.add_manually.form.as_ref()
    }

    /// The open "Send button…" list.
    pub fn send_button_menu(&self) -> Option<&SendButtonMenu> {
        self.add_manually.send_button.as_ref()
    }

    /// `:new [type…]`: create the one available type the words name, else open the picker
    /// (filtered by them).
    pub(super) fn new_remote_command(&mut self, args: &[&str]) {
        let query = args.join(" ");
        let picker = NewRemotePicker {
            filter: query.clone(),
            selected: 0,
            scroll: 0,
            availability: TYPES.iter().map(add_manually::availability).collect(),
        };
        if !query.is_empty() {
            let matches = picker.visible();
            let available: Vec<usize> = matches.iter().copied().filter(|&i| picker.availability[i].is_ok()).collect();
            if let [i] = available[..] {
                self.create_remote(i, None);
                return;
            }
            if matches.is_empty() {
                self.last_error = Some(format!("No remote type matches \"{query}\" (:new lists them)"));
                return;
            }
        }
        self.add_manually.picker = Some(picker);
        self.input_mode = InputMode::NewRemote;
    }

    /// Create a remote of [TYPES]`[index]` from `gen` (None: freshly drawn parameters) and add it
    /// to the captures (selected).
    fn create_remote(&mut self, index: usize, gen: Option<Gen>) -> bool {
        // Creating is a new action: an older error must not hide its result.
        self.clear_messages();
        let ty = &TYPES[index];
        let mut rng = Rng::from_entropy();
        let gen = gen.unwrap_or_else(|| add_manually::fill(ty, &mut rng));
        let key = match add_manually::generate(&gen, &mut rng as &mut dyn RandSource) {
            Ok(k) => k,
            Err(e) => {
                self.last_error = Some(format!("{}: {e}", ty.name));
                return false;
            }
        };
        match crate::export::flipper::capture_from_key(&key, &self.protocols, self.next_capture_id) {
            Ok(mut capture) => {
                capture.source_file = Some(format!("Add Manually: {}", ty.name));
                self.next_capture_id += 1;
                let summary = format!(
                    "Created #{:02} {}: {} · serial 0x{} · button {} · counter {}",
                    capture.id,
                    ty.name,
                    capture.protocol_name(),
                    capture.serial_hex(),
                    capture.button_name(),
                    capture.counter_str()
                );
                self.captures.push(capture);
                self.selected_capture = Some(self.captures.len() - 1);
                self.status_message = Some(summary);
                true
            }
            Err(e) => {
                self.last_error = Some(format!("{}: {e:#}", ty.name));
                false
            }
        }
    }

    /// Keys of the `:new` picker, the field form and the "Send button…" list.
    pub fn handle_add_manually_key(&mut self, code: KeyCode) {
        match self.input_mode {
            InputMode::NewRemote => self.picker_key(code),
            InputMode::NewRemoteEdit => self.form_key(code),
            InputMode::SendButton => self.send_button_key(code),
            _ => {}
        }
    }

    /// Ctrl-U / Ctrl-W ([App::line_edit_key]) in the picker's filter, the form's focused field or
    /// the "Send button…" code.
    pub(super) fn add_manually_kill(&mut self, what: super::input::Kill) -> bool {
        match self.input_mode {
            InputMode::NewRemote => {
                let Some(p) = self.add_manually.picker.as_mut() else { return false };
                super::input::kill(&mut p.filter, what);
                // As after Backspace: the first match, from the top of the list.
                p.selected = 0;
                p.scroll = 0;
            }
            InputMode::NewRemoteEdit => {
                let Some(f) = self.add_manually.form.as_mut() else { return false };
                f.error = None;
                if let Some(field) = f.fields.get_mut(f.focus) {
                    super::input::kill(&mut field.1, what);
                }
            }
            InputMode::SendButton => {
                let Some(m) = self.add_manually.send_button.as_mut() else { return false };
                super::input::kill(&mut m.custom, what);
            }
            _ => return false,
        }
        true
    }

    fn picker_key(&mut self, code: KeyCode) {
        let Some(p) = self.add_manually.picker.as_mut() else {
            self.input_mode = InputMode::Normal;
            return;
        };
        let count = p.visible().len();
        let last = count.saturating_sub(1);
        match code {
            KeyCode::Esc => {
                self.add_manually.picker = None;
                self.input_mode = InputMode::Normal;
                return;
            }
            KeyCode::Up => p.selected = p.selected.saturating_sub(1),
            KeyCode::Down => p.selected = (p.selected + 1).min(last),
            KeyCode::PageUp => p.selected = p.selected.saturating_sub(PICKER_VISIBLE_ROWS / 2),
            KeyCode::PageDown => p.selected = (p.selected + PICKER_VISIBLE_ROWS / 2).min(last),
            KeyCode::Home => p.selected = 0,
            KeyCode::End => p.selected = last,
            KeyCode::Backspace => {
                p.filter.pop();
                p.selected = 0;
            }
            KeyCode::Char(c) => {
                p.filter.push(c);
                p.selected = 0;
            }
            KeyCode::Enter | KeyCode::Tab => {
                let Some(i) = p.selected_type() else { return };
                if let Err(e) = &p.availability[i] {
                    self.last_error = Some(format!("{}: {e}", TYPES[i].name));
                    return;
                }
                if code == KeyCode::Enter {
                    if self.create_remote(i, None) {
                        self.add_manually.picker = None;
                        self.input_mode = InputMode::Normal;
                    }
                } else {
                    self.open_form(i);
                }
                return;
            }
            _ => {}
        }
        // Keep the selection's row in view (rows include the group headings; a heading right above
        // the selection is scrolled in with it).
        let p = self.add_manually.picker.as_mut().expect("picker");
        let selected = p.selected_type().map(PickerRow::Type);
        let rows = p.rows();
        let row = rows.iter().position(|r| Some(*r) == selected).unwrap_or(0);
        if row < p.scroll {
            let heading_above = row > 0 && matches!(rows[row - 1], PickerRow::Heading(_));
            p.scroll = if heading_above { row - 1 } else { row };
        } else if row >= p.scroll + PICKER_VISIBLE_ROWS {
            p.scroll = row + 1 - PICKER_VISIBLE_ROWS;
        }
        p.scroll = p.scroll.min(rows.len().saturating_sub(PICKER_VISIBLE_ROWS));
    }

    /// The fields of [TYPES]`[index]` with freshly drawn values.
    fn open_form(&mut self, index: usize) {
        let gen = add_manually::fill(&TYPES[index], &mut Rng::from_entropy());
        let fields = gen
            .info
            .fields()
            .into_iter()
            .map(|(f, v, w)| (f, format!("{v:0width$X}", width = w * 2), w))
            .collect();
        self.add_manually.form = Some(NewRemoteForm { type_index: index, gen, fields, focus: 0, error: None });
        self.input_mode = InputMode::NewRemoteEdit;
    }

    fn form_key(&mut self, code: KeyCode) {
        let Some(f) = self.add_manually.form.as_mut() else {
            self.input_mode = InputMode::NewRemote;
            return;
        };
        let n = f.fields.len();
        // An error is about the key before this one: it goes with the next key (it stayed after
        // the field had been fixed).
        f.error = None;
        match code {
            KeyCode::Esc => {
                self.add_manually.form = None;
                self.input_mode = if self.add_manually.picker.is_some() { InputMode::NewRemote } else { InputMode::Normal };
            }
            KeyCode::Up | KeyCode::BackTab => f.focus = f.focus.checked_sub(1).unwrap_or(n.saturating_sub(1)),
            KeyCode::Down | KeyCode::Tab => f.focus = if n == 0 { 0 } else { (f.focus + 1) % n },
            KeyCode::Backspace => {
                if let Some(field) = f.fields.get_mut(f.focus) {
                    field.1.pop();
                }
            }
            KeyCode::Char(c) if c.is_ascii_hexdigit() => {
                if let Some(field) = f.fields.get_mut(f.focus) {
                    if field.1.len() < field.2 * 2 {
                        field.1.push(c.to_ascii_uppercase());
                    } else {
                        f.error = Some(format!("{} is {} hex digits", field.0.label(), field.2 * 2));
                    }
                }
            }
            KeyCode::Enter => {
                let mut gen = f.gen.clone();
                for (field, text, _) in &f.fields {
                    let value = if text.is_empty() { Ok(0) } else { u64::from_str_radix(text, 16) };
                    match value {
                        Ok(v) => gen.info.set(*field, v),
                        Err(_) => {
                            f.error = Some(format!("{}: not a hex number", field.label()));
                            return;
                        }
                    }
                }
                let index = f.type_index;
                if self.create_remote(index, Some(gen)) {
                    self.add_manually.form = None;
                    self.add_manually.picker = None;
                    self.input_mode = InputMode::Normal;
                }
            }
            _ => {}
        }
    }

    /// "Send button…" right after "Send next code" (the gate / garage / KeeLoq barrier and alarm
    /// menu with a transmitter) when the remote's encoder switches buttons.
    pub(super) fn add_send_button_action(&self, actions: &mut Vec<SignalAction>) {
        let Some(c) = self.signal_menu_capture() else { return };
        let Some(at) = actions.iter().position(|a| *a == SignalAction::SendNextCode) else { return };
        if !gate_buttons::tx_buttons(c.protocol_name(), &c.decoded_signal()).is_empty() {
            actions.insert(at + 1, SignalAction::SendButton);
        }
    }

    /// Open the "Send button…" list of a capture.
    pub(super) fn open_send_button_menu(&mut self, id: u32) {
        let Some(c) = self.captures.iter().find(|c| c.id == id) else {
            self.last_error = Some(format!("Capture #{id:02} not found"));
            return;
        };
        let protocol = c.protocol_name();
        let buttons: Vec<(u8, String)> = gate_buttons::tx_buttons(protocol, &c.decoded_signal())
            .into_iter()
            .map(|b| {
                let name = gate_buttons::button_label(protocol, b);
                let hex = format!("0x{b:02X}");
                let mut label = if name.eq_ignore_ascii_case(&hex) || name == gate_buttons::hex_label(b) {
                    name.to_string()
                } else {
                    format!("{name} ({hex})")
                };
                if Some(b) == c.button {
                    label.push_str("  ← last sent");
                }
                (b, label)
            })
            .collect();
        let selected = buttons.iter().position(|(b, _)| Some(*b) == c.button).unwrap_or(0);
        self.add_manually.send_button = Some(SendButtonMenu { capture_id: id, buttons, selected, custom: String::new() });
        self.input_mode = InputMode::SendButton;
    }

    fn send_button_key(&mut self, code: KeyCode) {
        let Some(m) = self.add_manually.send_button.as_mut() else {
            self.input_mode = InputMode::Normal;
            return;
        };
        let last = m.buttons.len().saturating_sub(1);
        match code {
            KeyCode::Esc => {
                self.add_manually.send_button = None;
                self.input_mode = InputMode::SignalMenu;
            }
            KeyCode::Up | KeyCode::Char('k') => m.selected = m.selected.saturating_sub(1),
            KeyCode::Down | KeyCode::Char('j') => m.selected = (m.selected + 1).min(last),
            KeyCode::Backspace => {
                m.custom.pop();
            }
            KeyCode::Char(c) if c.is_ascii_hexdigit() && m.custom.len() < 2 => m.custom.push(c.to_ascii_uppercase()),
            KeyCode::Enter => {
                let id = m.capture_id;
                let button = if m.custom.is_empty() {
                    m.buttons.get(m.selected).map(|b| b.0)
                } else {
                    u8::from_str_radix(&m.custom, 16).ok()
                };
                m.custom.clear();
                if let Some(button) = button {
                    // Stays open: the next button goes out with the next counter value.
                    if let Err(e) = self.transmit_button(id, Some(button)) {
                        self.last_error = Some(format!("{e:#}"));
                    }
                }
            }
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn picker_rows_group_the_filtered_types() {
        let p = NewRemotePicker { filter: "433".into(), selected: 0, scroll: 0, availability: Vec::new() };
        let rows = p.rows();
        assert!(matches!(rows[0], PickerRow::Heading(add_manually::Group::Rolling)));
        let headings = rows.iter().filter(|r| matches!(r, PickerRow::Heading(_))).count();
        assert_eq!(headings, 6);
        assert_eq!(rows.len() - headings, p.visible().len());
        let all = NewRemotePicker { filter: String::new(), ..p };
        assert_eq!(all.visible().len(), TYPES.len());
        assert_eq!(all.selected_type(), Some(0));
    }
}
