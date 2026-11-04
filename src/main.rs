//! KAT - Keyfob Analysis Toolkit
//!
//! A terminal UI application for capturing, decoding, and transmitting
//! keyfob signals using HackRF.

mod analysis;
mod app;
mod capture;
mod export;
mod headless;
mod keystore;
mod protocols;
mod radio;
mod saved_fobs;
mod session;
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
use std::io::{self, IsTerminal, Write};
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

/// Command-line options (`kat --help`).
#[derive(Debug, Default, PartialEq)]
struct CliOptions {
    /// IQ recording to decode at startup (`--iq <file>`)
    iq: Option<std::path::PathBuf>,
    /// Its centre frequency (`--freq`, MHz or Hz)
    freq: Option<u32>,
    /// Its sample rate (`--rate`, e.g. 2M, 2.4M, 2048000)
    rate: Option<u32>,
}

#[derive(Debug, PartialEq)]
enum CliAction {
    Run(CliOptions),
    /// `--decode <path>...`: print decodes without the TUI.
    Decode(headless::DecodeOptions),
    /// `--keeloq-bf <path>...`: recover a magic-serial KeeLoq manufacturer key.
    KeeloqBf(headless::KeeloqBfOptions),
    Help,
    Version,
}

const USAGE: &str = "Usage: kat [--iq <file> [--freq <MHz|Hz>] [--rate <S/s>]]
       kat --decode <file|dir>... [--freq <MHz|Hz>] [--rate <S/s>] [--all] [--verbose]
       kat --keeloq-bf <file|dir>... [--type auto|6|7|8] [--save] [--freq <MHz|Hz>] [--rate <S/s>]

  --iq <file>     Decode an IQ recording at startup, with or without a radio attached:
                  .cs8 (hackrf_transfer), .cu8 (rtl_sdr / rtl_433), .cf32 (GNU Radio, URH)
  --decode <p>... Decode files without the TUI and print one line per distinct decode:
                  Flipper .sub (RAW or key files) and IQ recordings; directories are searched
                  recursively. --all prints every decode in stream order; --verbose
                  also prints each IQ burst and its pulses to stderr.
  --keeloq-bf <p>... Recover the manufacturer key of a \"magic serial\" KeeLoq remote (Flipper-ARF's
                  KeeLoq Decrypt) from two or more captures of it (.sub or IQ files).
                  --type: auto (6, then 7; the default), 6, 7 or 8 (8 searches 2^40 keys and
                  needs three captures); --save adds the key to keeloq_mfcodes_user as BF_<serial>.
  --freq <f>      Centre frequency of the recording, MHz (433.92) or Hz (433920000);
                  default: from the file name, else the configured frequency (433.92 MHz
                  for --decode)
  --rate <r>      Sample rate (2M, 2.4M, 2048000); default: from the file name, else 2M
  -h, --help      Show this help
  -V, --version   Show the version

In the app: :iq <file> [freq] [rate], :record [file] / :record stop, :hop on|off.";

/// The value of `--flag value` or `--flag=value`.
fn flag_value(inline: Option<String>, rest: &mut impl Iterator<Item = String>, flag: &str) -> Result<String, String> {
    inline.or_else(|| rest.next()).ok_or_else(|| format!("{flag} needs a value"))
}

/// Minimal argument parser (no extra crate needed for three flags).
fn parse_args(args: impl IntoIterator<Item = String>) -> Result<CliAction, String> {
    let mut opts = CliOptions::default();
    let mut decode: Option<Vec<std::path::PathBuf>> = None;
    let mut keeloq_bf: Option<Vec<std::path::PathBuf>> = None;
    let mut bf_types: Option<Vec<u32>> = None;
    let mut save = false;
    let mut all = false;
    let mut verbose = false;
    let mut it = args.into_iter();
    while let Some(arg) = it.next() {
        // Paths after --decode / --keeloq-bf (up to the next flag).
        if let Some(paths) = decode.as_mut().or(keeloq_bf.as_mut()) {
            if !arg.starts_with('-') {
                paths.push(std::path::PathBuf::from(arg));
                continue;
            }
        }
        let (flag, inline) = match arg.split_once('=') {
            Some((f, v)) if f.starts_with("--") => (f.to_string(), Some(v.to_string())),
            _ => (arg.clone(), None),
        };
        match flag.as_str() {
            "-h" | "--help" => return Ok(CliAction::Help),
            "-V" | "--version" => return Ok(CliAction::Version),
            "--iq" => opts.iq = Some(std::path::PathBuf::from(flag_value(inline, &mut it, "--iq")?)),
            "-d" | "--decode" => {
                let paths = decode.get_or_insert_with(Vec::new);
                if let Some(v) = inline {
                    paths.push(std::path::PathBuf::from(v));
                }
            }
            "--keeloq-bf" => {
                let paths = keeloq_bf.get_or_insert_with(Vec::new);
                if let Some(v) = inline {
                    paths.push(std::path::PathBuf::from(v));
                }
            }
            "--type" => bf_types = Some(headless::parse_bf_types(&flag_value(inline, &mut it, "--type")?)?),
            "--save" => save = true,
            "--all" => all = true,
            "-v" | "--verbose" => verbose = true,
            "--freq" => {
                let v = flag_value(inline, &mut it, "--freq")?;
                opts.freq = Some(radio::iq_file::parse_frequency_arg(&v).ok_or(format!("invalid frequency: {v}"))?);
            }
            "--rate" => {
                let v = flag_value(inline, &mut it, "--rate")?;
                opts.rate = Some(radio::iq_file::parse_rate_arg(&v).ok_or(format!("invalid sample rate: {v}"))?);
            }
            other => return Err(format!("unknown argument: {other}")),
        }
    }
    if let Some(paths) = keeloq_bf {
        if paths.is_empty() {
            return Err("--keeloq-bf needs at least one file or directory".to_string());
        }
        if decode.is_some() || opts.iq.is_some() || all || verbose {
            return Err("--keeloq-bf cannot be combined with --decode, --iq, --all or --verbose".to_string());
        }
        let types = bf_types.unwrap_or_else(|| protocols::keeloq_bf::AUTO_TYPES.to_vec());
        return Ok(CliAction::KeeloqBf(headless::KeeloqBfOptions { paths, freq: opts.freq, rate: opts.rate, types, save }));
    }
    if bf_types.is_some() || save {
        return Err("--type and --save are --keeloq-bf options".to_string());
    }
    if let Some(paths) = decode {
        if paths.is_empty() {
            return Err("--decode needs at least one file or directory".to_string());
        }
        if opts.iq.is_some() {
            return Err("--iq and --decode cannot be combined".to_string());
        }
        return Ok(CliAction::Decode(headless::DecodeOptions { paths, freq: opts.freq, rate: opts.rate, all, verbose }));
    }
    if all || verbose {
        return Err("--all and --verbose are --decode options".to_string());
    }
    if opts.iq.is_none() && (opts.freq.is_some() || opts.rate.is_some()) {
        return Err("--freq and --rate describe the --iq recording".to_string());
    }
    Ok(CliAction::Run(opts))
}

fn main() -> Result<()> {
    let cli = match parse_args(std::env::args().skip(1)) {
        Ok(CliAction::Run(opts)) => opts,
        Ok(CliAction::Decode(opts)) => std::process::exit(headless::run(&opts)),
        Ok(CliAction::KeeloqBf(opts)) => std::process::exit(headless::keeloq_bf(&opts)),
        Ok(CliAction::Help) => {
            println!("KAT v{VERSION} - Keyfob Analysis Toolkit\n\n{USAGE}");
            return Ok(());
        }
        Ok(CliAction::Version) => {
            println!("kat {VERSION}");
            return Ok(());
        }
        Err(e) => {
            eprintln!("kat: {e}\n\n{USAGE}");
            std::process::exit(2);
        }
    };
    if let Some(path) = &cli.iq {
        if !path.is_file() {
            eprintln!("kat: --iq {}: no such file", path.display());
            std::process::exit(2);
        }
    }

    // Check if we have a TTY first
    if !std::io::stdout().is_terminal() {
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
    if let Some(path) = cli.iq {
        // A file needs no radio: skip the "no device" warning.
        if app.input_mode == InputMode::HackRfNotDetected {
            app.input_mode = if app.pending_fob_files.is_empty() { InputMode::Normal } else { InputMode::StartupImport };
        }
        app.start_iq_file(path, cli.freq, cli.rate);
    }
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
                            // Next: session restore prompt, then import prompt, then normal.
                            app.input_mode = app.next_startup_mode();
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

                        // Session restore prompt, :clear confirmation, timing overlay (app/persistence.rs)
                        InputMode::SessionRestore
                        | InputMode::ConfirmClear
                        | InputMode::TimingAnalysis => {
                            app.handle_app_mode_key(key.code)?;
                        }

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
                                        Some(ExportFormat::FlipperKey) => {
                                            app.complete_flipper_key_export()?;
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

                        // .fob export metadata: Region -> Command
                        InputMode::FobMetaRegion => match key.code {
                            KeyCode::Enter => {
                                app.input_mode = InputMode::FobMetaCommand;
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

                        // .fob export metadata: Command -> Notes
                        InputMode::FobMetaCommand => match key.code {
                            KeyCode::Enter => {
                                app.input_mode = InputMode::FobMetaNotes;
                            }
                            KeyCode::Char(c) => {
                                app.fob_meta_command.push(c);
                            }
                            KeyCode::Backspace => {
                                app.fob_meta_command.pop();
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

                        // Capture metadata (Year/Make/Model/Region/Command for vuln lookup)
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
                                app.input_mode = InputMode::CaptureMetaCommand;
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
                        InputMode::CaptureMetaCommand => match key.code {
                            KeyCode::Enter => {
                                app.save_capture_meta();
                            }
                            KeyCode::Char(c) => {
                                app.capture_meta_command.push(c);
                            }
                            KeyCode::Backspace => {
                                app.capture_meta_command.pop();
                            }
                            KeyCode::Esc => {
                                app.cancel_capture_meta();
                            }
                            _ => {}
                        },

                        // :new picker / field form, "Send button…" list (app/add_manually.rs)
                        InputMode::NewRemote | InputMode::NewRemoteEdit | InputMode::SendButton => {
                            app.handle_add_manually_key(key.code);
                            while app.has_pending_transmit() {
                                terminal.draw(|f| draw_ui(f, app))?;
                                app.run_one_pending_transmit()?;
                            }
                        }

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
        // IQ file progress, recordings, hopping (src/app/rx.rs)
        app.poll_rx();
    }
}

#[cfg(test)]
mod cli_tests {
    use super::*;

    fn args(a: &[&str]) -> Vec<String> {
        a.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn parses_iq_options() {
        assert_eq!(parse_args(args(&[])), Ok(CliAction::Run(CliOptions::default())));
        assert_eq!(
            parse_args(args(&["--iq", "fob.cu8", "--freq", "433.92", "--rate=2.4M"])),
            Ok(CliAction::Run(CliOptions {
                iq: Some("fob.cu8".into()),
                freq: Some(433_920_000),
                rate: Some(2_400_000),
            }))
        );
        assert_eq!(parse_args(args(&["--help"])), Ok(CliAction::Help));
        assert_eq!(parse_args(args(&["-V"])), Ok(CliAction::Version));
        assert!(parse_args(args(&["--iq"])).is_err());
        assert!(parse_args(args(&["--freq", "433.92"])).is_err());
        assert!(parse_args(args(&["--bogus"])).is_err());
        assert!(parse_args(args(&["--iq", "x.cs8", "--rate", "fast"])).is_err());
    }

    #[test]
    fn parses_decode_options() {
        let decode = |paths: &[&str], freq, rate, all| {
            Ok(CliAction::Decode(headless::DecodeOptions {
                paths: paths.iter().map(std::path::PathBuf::from).collect(),
                freq,
                rate,
                all,
                verbose: false,
            }))
        };
        assert_eq!(parse_args(args(&["--decode", "a.sub", "dir"])), decode(&["a.sub", "dir"], None, None, false));
        assert_eq!(
            parse_args(args(&["--freq", "315", "-d", "x.cu8", "--all", "--rate=250k"])),
            decode(&["x.cu8"], Some(315_000_000), Some(250_000), true)
        );
        assert_eq!(parse_args(args(&["--decode=a.sub", "b.sub"])), decode(&["a.sub", "b.sub"], None, None, false));
        assert!(parse_args(args(&["--decode"])).is_err());
        assert_eq!(
            parse_args(args(&["--keeloq-bf", "a.cu8", "b.cu8", "--type", "8", "--save"])),
            Ok(CliAction::KeeloqBf(headless::KeeloqBfOptions {
                paths: vec!["a.cu8".into(), "b.cu8".into()],
                types: vec![8],
                save: true,
                ..Default::default()
            }))
        );
        assert_eq!(
            parse_args(args(&["--keeloq-bf", "dir"])),
            Ok(CliAction::KeeloqBf(headless::KeeloqBfOptions { paths: vec!["dir".into()], types: vec![6, 7], ..Default::default() }))
        );
        assert!(parse_args(args(&["--keeloq-bf", "x", "--type", "5"])).is_err());
        assert!(parse_args(args(&["--keeloq-bf", "x", "--decode", "y"])).is_err());
        assert!(parse_args(args(&["--save"])).is_err());
        assert!(parse_args(args(&["--all"])).is_err());
        assert!(parse_args(args(&["--iq", "x.cs8", "--decode", "y.sub"])).is_err());
    }
}
