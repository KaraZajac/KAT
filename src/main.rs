//! KAT - Keyfob Analysis Toolkit
//!
//! A terminal UI application for capturing, decoding, and transmitting
//! keyfob signals using HackRF.

mod app;
mod capture;
mod export;
mod keystore;
mod protocols;
mod radio;
mod storage;
mod ui;
mod vuln_db;

use anyhow::Result;
use crossterm::{
    event::{self, Event, KeyCode, KeyEventKind},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use ratatui::{backend::CrosstermBackend, Terminal};
use std::io::{self, Write};
use std::panic;
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

use app::{App, InputMode, SettingsField, ExportFormat};
use ui::draw_ui;

const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Restore the terminal to normal state (for panic handler)
fn restore_terminal_panic() {
    // Disable raw mode first
    let _ = disable_raw_mode();
    
    // Write escape sequences directly to stdout
    let mut stdout = io::stdout();
    
    // Leave alternate screen: ESC [ ? 1049 l
    let _ = stdout.write_all(b"\x1b[?1049l");
    
    // Show cursor: ESC [ ? 25 h
    let _ = stdout.write_all(b"\x1b[?25h");
    
    let _ = stdout.flush();
}

fn main() -> Result<()> {
    // Check if we have a TTY first
    if !atty::is(atty::Stream::Stdout) {
        eprintln!("Error: KAT requires a terminal (TTY) to run.");
        eprintln!("Please run this program in a real terminal, not via a script or IDE runner.");
        std::process::exit(1);
    }

    // Set up panic hook to restore terminal on panic
    let default_panic = panic::take_hook();
    panic::set_hook(Box::new(move |panic_info| {
        restore_terminal_panic();
        default_panic(panic_info);
    }));

    // Initialize logging to a file (not stdout, which would corrupt TUI)
    let log_file = crate::storage::resolve_config_dir()
        .unwrap_or_else(|| std::path::PathBuf::from(".").join("KAT"))
        .join("kat.log");
    
    // Create log directory if needed
    if let Some(parent) = log_file.parent() {
        let _ = std::fs::create_dir_all(parent);
    }
    
    // Set up file-based logging
    if let Ok(file) = std::fs::File::create(&log_file) {
        tracing_subscriber::registry()
            .with(
                tracing_subscriber::EnvFilter::try_from_default_env()
                    .unwrap_or_else(|_| "kat=info".into()),
            )
            .with(
                tracing_subscriber::fmt::layer()
                    .with_target(false)
                    .with_writer(std::sync::Mutex::new(file))
                    .with_ansi(false)
            )
            .init();
    }

    tracing::info!("Starting KAT v{}", VERSION);

    // Setup terminal
    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen)?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    // Create app and run
    let mut app = App::new()?;
    let res = run_app(&mut terminal, &mut app);

    // Restore terminal properly using the terminal's backend
    disable_raw_mode()?;
    execute!(
        terminal.backend_mut(),
        LeaveAlternateScreen
    )?;
    terminal.show_cursor()?;

    if let Err(err) = res {
        eprintln!("Error: {err:?}");
        return Err(err);
    }

    Ok(())
}

fn run_app<B: ratatui::backend::Backend>(terminal: &mut Terminal<B>, app: &mut App) -> Result<()> {
    loop {
        terminal.draw(|f| draw_ui(f, app))?;

        if event::poll(std::time::Duration::from_millis(100))? {
            if let Event::Key(key) = event::read()? {
                if key.kind == KeyEventKind::Press {
                    match app.input_mode {
                        InputMode::Normal => match key.code {
                            KeyCode::Char('q') => return Ok(()),
                            KeyCode::Char(':') => {
                                app.input_mode = InputMode::Command;
                                app.command_input.clear();
                            }
                            KeyCode::Char('j') | KeyCode::Down => {
                                app.next_capture();
                            }
                            KeyCode::Char('k') | KeyCode::Up => {
                                app.previous_capture();
                            }
                            KeyCode::Char('r') => {
                                app.toggle_receiving()?;
                            }
                            KeyCode::Char('d') => {
                                let _ = app.delete_selected_capture();
                            }
                            KeyCode::Enter => {
                                // Open signal action menu if a capture is selected
                                if app.selected_capture.is_some() && !app.captures.is_empty() {
                                    app.input_mode = InputMode::SignalMenu;
                                    app.signal_menu_index = 0;
                                }
                            }
                            KeyCode::Char('i') => {
                                // Edit Year/Make/Model/Region for vuln lookup
                                if let Some(idx) = app.selected_capture {
                                    if let Some(capture) = app.captures.get(idx) {
                                        app.open_capture_meta_form(capture.id);
                                    }
                                }
                            }
                            KeyCode::Tab => {
                                // Open settings selector
                                app.input_mode = InputMode::SettingsSelect;
                                app.settings_field_index = 0;
                            }
                            _ => {}
                        },

                        InputMode::Command => match key.code {
                            KeyCode::Enter => {
                                let command = app.command_input.clone();
                                app.execute_command(&command)?;
                                if app.quit_requested {
                                    return Ok(());
                                }
                                app.command_input.clear();
                                if app.input_mode == InputMode::Command {
                                    app.input_mode = InputMode::Normal;
                                }
                                while app.has_pending_transmit() {
                                    terminal.draw(|f| draw_ui(f, app))?;
                                    app.run_one_pending_transmit()?;
                                }
                            }
                            KeyCode::Char(c) => {
                                app.command_input.push(c);
                            }
                            KeyCode::Backspace => {
                                app.command_input.pop();
                            }
                            KeyCode::Esc => {
                                app.command_input.clear();
                                app.input_mode = InputMode::Normal;
                            }
                            _ => {}
                        },

                        InputMode::SignalMenu => match key.code {
                            KeyCode::Up | KeyCode::Char('k') => {
                                if app.signal_menu_index > 0 {
                                    app.signal_menu_index -= 1;
                                }
                            }
                            KeyCode::Down | KeyCode::Char('j') => {
                                let len = app.available_signal_actions().len();
                                if len > 0 && app.signal_menu_index < len - 1 {
                                    app.signal_menu_index += 1;
                                }
                            }
                            KeyCode::Enter => {
                                app.execute_signal_action()?;
                                // Only return to Normal if the action didn't change
                                // input mode (e.g. ExportFob sets FobMetaYear)
                                if app.input_mode == InputMode::SignalMenu {
                                    app.input_mode = InputMode::Normal;
                                }
                                while app.has_pending_transmit() {
                                    terminal.draw(|f| draw_ui(f, app))?;
                                    app.run_one_pending_transmit()?;
                                }
                            }
                            KeyCode::Esc => {
                                app.input_mode = InputMode::Normal;
                            }
                            _ => {}
                        },

                        InputMode::SettingsSelect => match key.code {
                            KeyCode::Left | KeyCode::Char('h') => {
                                if app.settings_field_index > 0 {
                                    app.settings_field_index -= 1;
                                }
                            }
                            KeyCode::Right | KeyCode::Char('l') => {
                                if app.settings_field_index < SettingsField::ALL.len() - 1 {
                                    app.settings_field_index += 1;
                                }
                            }
                            KeyCode::Tab => {
                                // Cycle through fields
                                app.settings_field_index =
                                    (app.settings_field_index + 1) % SettingsField::ALL.len();
                            }
                            KeyCode::Enter => {
                                // Enter edit mode for this field
                                app.settings_value_index = app.current_settings_value_index();
                                app.input_mode = InputMode::SettingsEdit;
                            }
                            KeyCode::Esc => {
                                app.input_mode = InputMode::Normal;
                            }
                            _ => {}
                        },

                        InputMode::SettingsEdit => match key.code {
                            KeyCode::Up | KeyCode::Char('k') => {
                                if app.settings_value_index > 0 {
                                    app.settings_value_index -= 1;
                                }
                            }
                            KeyCode::Down | KeyCode::Char('j') => {
                                let max = app.settings_value_count();
                                if app.settings_value_index < max - 1 {
                                    app.settings_value_index += 1;
                                }
                            }
                            KeyCode::Enter => {
                                app.apply_settings_value()?;
                                app.input_mode = InputMode::SettingsSelect;
                            }
                            KeyCode::Esc => {
                                app.input_mode = InputMode::SettingsSelect;
                            }
                            _ => {}
                        },

                        // Startup: HackRF not detected — any key to dismiss
                        InputMode::HackRfNotDetected => {
                            app.input_mode = if app.pending_fob_files.is_empty() {
                                InputMode::Normal
                            } else {
                                InputMode::StartupImport
                            };
                        }

                        // Startup: found .fob files, y/n to import
                        InputMode::StartupImport => match key.code {
                            KeyCode::Char('y') | KeyCode::Char('Y') => {
                                app.import_fob_files()?;
                                app.input_mode = InputMode::Normal;
                            }
                            KeyCode::Char('n') | KeyCode::Char('N') | KeyCode::Esc => {
                                app.skip_fob_import();
                                app.input_mode = InputMode::Normal;
                            }
                            _ => {}
                        },

                        // Export: filename input
                        InputMode::ExportFilename => match key.code {
                            KeyCode::Enter => {
                                if app.export_filename.is_empty() {
                                    app.last_error = Some("Filename cannot be empty".to_string());
                                } else {
                                    match app.export_format {
                                        Some(ExportFormat::Fob) => {
                                            app.input_mode = InputMode::FobMetaYear;
                                        }
                                        Some(ExportFormat::Flipper) => {
                                            app.complete_flipper_export()?;
                                            app.input_mode = InputMode::Normal;
                                        }
                                        None => {
                                            app.input_mode = InputMode::Normal;
                                        }
                                    }
                                }
                            }
                            KeyCode::Char(c) => {
                                // Allow filesystem-safe characters
                                if c.is_alphanumeric() || c == '_' || c == '-' || c == '.' {
                                    app.export_filename.push(c);
                                }
                            }
                            KeyCode::Backspace => {
                                app.export_filename.pop();
                            }
                            KeyCode::Esc => {
                                app.export_capture_id = None;
                                app.export_format = None;
                                app.input_mode = InputMode::Normal;
                            }
                            _ => {}
                        },

                        // .fob export metadata: Year
                        InputMode::FobMetaYear => match key.code {
                            KeyCode::Enter => {
                                app.input_mode = InputMode::FobMetaMake;
                            }
                            KeyCode::Char(c) if c.is_ascii_digit() => {
                                if app.fob_meta_year.len() < 4 {
                                    app.fob_meta_year.push(c);
                                }
                            }
                            KeyCode::Backspace => {
                                app.fob_meta_year.pop();
                            }
                            KeyCode::Esc => {
                                app.export_capture_id = None;
                                app.input_mode = InputMode::Normal;
                            }
                            _ => {}
                        },

                        // .fob export metadata: Make
                        InputMode::FobMetaMake => match key.code {
                            KeyCode::Enter => {
                                app.input_mode = InputMode::FobMetaModel;
                            }
                            KeyCode::Char(c) => {
                                app.fob_meta_make.push(c);
                            }
                            KeyCode::Backspace => {
                                app.fob_meta_make.pop();
                            }
                            KeyCode::Esc => {
                                app.export_capture_id = None;
                                app.input_mode = InputMode::Normal;
                            }
                            _ => {}
                        },

                        // .fob export metadata: Model -> Region
                        InputMode::FobMetaModel => match key.code {
                            KeyCode::Enter => {
                                app.input_mode = InputMode::FobMetaRegion;
                            }
                            KeyCode::Char(c) => {
                                app.fob_meta_model.push(c);
                            }
                            KeyCode::Backspace => {
                                app.fob_meta_model.pop();
                            }
                            KeyCode::Esc => {
                                app.export_capture_id = None;
                                app.input_mode = InputMode::Normal;
                            }
                            _ => {}
                        },

                        // .fob export metadata: Region -> Notes
                        InputMode::FobMetaRegion => match key.code {
                            KeyCode::Enter => {
                                app.input_mode = InputMode::FobMetaNotes;
                            }
                            KeyCode::Char(c) => {
                                app.fob_meta_region.push(c);
                            }
                            KeyCode::Backspace => {
                                app.fob_meta_region.pop();
                            }
                            KeyCode::Esc => {
                                app.export_capture_id = None;
                                app.input_mode = InputMode::Normal;
                            }
                            _ => {}
                        },

                        // .fob export metadata: Notes -> Export
                        InputMode::FobMetaNotes => match key.code {
                            KeyCode::Enter => {
                                app.complete_fob_export()?;
                                app.input_mode = InputMode::Normal;
                            }
                            KeyCode::Char(c) => {
                                app.fob_meta_notes.push(c);
                            }
                            KeyCode::Backspace => {
                                app.fob_meta_notes.pop();
                            }
                            KeyCode::Esc => {
                                app.export_capture_id = None;
                                app.input_mode = InputMode::Normal;
                            }
                            _ => {}
                        },

                        // Capture metadata (Year/Make/Model/Region for vuln lookup)
                        InputMode::CaptureMetaYear => match key.code {
                            KeyCode::Enter => {
                                app.input_mode = InputMode::CaptureMetaMake;
                            }
                            KeyCode::Char(c) if c.is_ascii_digit() => {
                                if app.capture_meta_year.len() < 4 {
                                    app.capture_meta_year.push(c);
                                }
                            }
                            KeyCode::Backspace => {
                                app.capture_meta_year.pop();
                            }
                            KeyCode::Esc => {
                                app.cancel_capture_meta();
                            }
                            _ => {}
                        },
                        InputMode::CaptureMetaMake => match key.code {
                            KeyCode::Enter => {
                                app.input_mode = InputMode::CaptureMetaModel;
                            }
                            KeyCode::Char(c) => {
                                app.capture_meta_make.push(c);
                            }
                            KeyCode::Backspace => {
                                app.capture_meta_make.pop();
                            }
                            KeyCode::Esc => {
                                app.cancel_capture_meta();
                            }
                            _ => {}
                        },
                        InputMode::CaptureMetaModel => match key.code {
                            KeyCode::Enter => {
                                app.input_mode = InputMode::CaptureMetaRegion;
                            }
                            KeyCode::Char(c) => {
                                app.capture_meta_model.push(c);
                            }
                            KeyCode::Backspace => {
                                app.capture_meta_model.pop();
                            }
                            KeyCode::Esc => {
                                app.cancel_capture_meta();
                            }
                            _ => {}
                        },
                        InputMode::CaptureMetaRegion => match key.code {
                            KeyCode::Enter => {
                                app.save_capture_meta();
                            }
                            KeyCode::Char(c) => {
                                app.capture_meta_region.push(c);
                            }
                            KeyCode::Backspace => {
                                app.capture_meta_region.pop();
                            }
                            KeyCode::Esc => {
                                app.cancel_capture_meta();
                            }
                            _ => {}
                        },

                        InputMode::LoadFileBrowser => {
                        const VISIBLE: usize = 16;
                        match key.code {
                            KeyCode::Esc => {
                                app.close_load_browser();
                            }
                            KeyCode::Enter => {
                                let _ = app.load_browser_enter();
                            }
                            KeyCode::Up | KeyCode::Char('k') => {
                                if app.load_browser_selected > 0 {
                                    app.load_browser_selected -= 1;
                                    if app.load_browser_selected < app.load_browser_scroll {
                                        app.load_browser_scroll = app.load_browser_selected;
                                    }
                                }
                            }
                            KeyCode::Down | KeyCode::Char('j') => {
                                let max = app.load_browser_entries.len().saturating_sub(1);
                                if app.load_browser_selected < max {
                                    app.load_browser_selected += 1;
                                    if app.load_browser_selected >= app.load_browser_scroll + VISIBLE {
                                        app.load_browser_scroll = app.load_browser_selected - VISIBLE + 1;
                                    }
                                }
                            }
                            _ => {}
                        }
                    },

                        InputMode::License | InputMode::Credits => match key.code {
                            KeyCode::Esc | KeyCode::Enter => {
                                app.input_mode = InputMode::Normal;
                            }
                            KeyCode::Up | KeyCode::Char('k') => {
                                app.overlay_scroll = app.overlay_scroll.saturating_sub(1);
                            }
                            KeyCode::Down | KeyCode::Char('j') => {
                                app.overlay_scroll = app.overlay_scroll.saturating_add(1);
                            }
                            _ => {}
                        },
                    }
                }
            }
        }

        // Process any pending radio events
        app.process_radio_events()?;
    }
}
