//! Terminal UI components.

mod add_manually;
mod analyze_overlay;
mod app_prompts;
mod captures_list;
mod command;
mod layout;
pub mod signal_menu;
mod text_overlay;
pub mod settings_menu;
mod status_bar;
mod text;
mod theme;

pub use layout::draw_ui;
