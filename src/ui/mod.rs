//! Terminal UI components.

mod add_manually;
mod app_prompts;
mod captures_list;
mod command;
mod layout;
pub mod signal_menu;
mod text_overlay;
pub mod settings_menu;
mod status_bar;

pub use layout::draw_ui;
