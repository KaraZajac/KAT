//! Styles shared by every view, so selections, inputs and text look the same everywhere.

use ratatui::style::{Color, Modifier, Style};

/// Normal text: the terminal's own foreground colour, readable on dark and light themes alike
/// (a hard-coded white all but disappears on a light background).
pub const TEXT: Style = Style::new();

/// The selected row of every table, list and menu.
pub const HIGHLIGHT: Style = Style::new().fg(Color::Black).bg(Color::Cyan).add_modifier(Modifier::BOLD);

/// Text cursor of every input (command line, forms, filters); drawn in the text colour.
pub const CURSOR: &str = "█";
