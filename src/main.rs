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
mod web;

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

use app::{App, InputMode, SettingsField};
use ui::draw_ui;

const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Restore the terminal to normal state (panic hook, [TerminalGuard]); harmless when it is
/// already restored.
fn restore_terminal() {
    // Disable raw mode first
    let _ = disable_raw_mode();
    let mut stdout = io::stdout();
    let _ = execute!(stdout, LeaveAlternateScreen, crossterm::cursor::Show);
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
    /// `--web [addr]`: run the App with the web UI instead of the TUI.
    Web(std::net::SocketAddr),
    Help,
    Version,
}

const USAGE: &str = "Usage: kat [--iq <file> [--freq <MHz|Hz>] [--rate <S/s>]]
       kat --decode <file|dir>... [--freq <MHz|Hz>] [--rate <S/s>] [--all] [--verbose]
       kat --keeloq-bf <file|dir>... [--type auto|6|7|8] [--save] [--freq <MHz|Hz>] [--rate <S/s>]

  --iq <file>     Decode an IQ recording at startup, with or without a radio attached:
                  .cs8 (hackrf_transfer), .cu8 (rtl_sdr / rtl_433), .cf32 (GNU Radio, URH)
  -d, --decode <p>...
                  Decode files without the TUI and print one line per distinct decode:
                  Flipper .sub (RAW or key files) and IQ recordings; directories are searched
                  recursively (.fob files open in the app with :load). --all prints every
                  decode in stream order; -v, --verbose also prints each IQ burst and its
                  pulses to stderr.
  --keeloq-bf <p>...
                  Recover the manufacturer key of a \"magic serial\" KeeLoq remote (Flipper-ARF's
                  KeeLoq Decrypt) from two or more captures of it (.sub or IQ files).
                  --type: auto (6, then 7; the default), 6, 7 or 8 (8 searches 2^40 keys and
                  needs three captures); --save adds the key to keeloq_mfcodes_user as BF_<serial>.
  --freq <f>      Centre frequency of the recording, MHz (433.92) or Hz (433920000);
                  default: from the file name, else the configured frequency (433.92 MHz
                  for --decode)
  --rate <r>      Sample rate, 200k to 40M S/s (2M, 2.4M, 250k, 2048000); default: from the
                  file name, else 2M
  -h, --help      Show this help
  -V, --version   Show the version

Exit status: 0 on success; 1 when --decode could not read a file or found nothing to decode,
or --keeloq-bf found no key; 2 for a bad argument or a missing --iq / --keeloq-bf file.

In the app: :iq <file> [freq] [rate], :load <file>, :record [file] / :record stop, :hop on|off.";

/// Where a file KAT reads belongs on the command line (or in the app), by its extension.
fn file_hint(path: &str) -> String {
    let p = std::path::Path::new(path);
    if radio::iq_file::is_iq_path(p) {
        format!("kat --iq {path} decodes it in the app, kat --decode {path} prints its decodes")
    } else if export::has_extension(p, "sub") {
        format!("kat --decode {path} prints its decodes; in the app, :load {path} opens it")
    } else if export::has_extension(p, "fob") {
        format!("open it in the app with :load {path}")
    } else {
        "KAT reads .sub and .fob files and .cs8 / .cu8 / .cf32 recordings".to_string()
    }
}

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
            "--iq" => {
                let v = flag_value(inline, &mut it, "--iq")?;
                // A .sub or .fob would pass the existence check and then fail inside the TUI.
                if !radio::iq_file::is_iq_path(std::path::Path::new(&v)) {
                    return Err(format!("--iq {v}: not an IQ recording (.cs8, .cu8, .cf32); {}", file_hint(&v)));
                }
                opts.iq = Some(std::path::PathBuf::from(v));
            }
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
                opts.freq = Some(
                    radio::iq_file::parse_frequency_arg(&v)
                        .ok_or(format!("invalid frequency: {v} (MHz like 433.92, or Hz like 433920000)"))?,
                );
            }
            "--rate" => {
                let v = flag_value(inline, &mut it, "--rate")?;
                // Checked here: a rate the decoder refuses (below 200 kS/s) used to fail only later.
                let rate = radio::iq_file::parse_rate_arg(&v)
                    .filter(|&r| r >= radio::iq_file::MIN_FILE_RATE)
                    .ok_or(format!("invalid sample rate: {v} (200k to 40M S/s, e.g. 2M, 2.4M, 250k, 2048000)"))?;
                opts.rate = Some(rate);
            }
            "--web" => {
                // `--web` (default 0.0.0.0:8080), `--web=addr:port`, or `--web addr:port`.
                let v = inline.or_else(|| it.next().filter(|v| !v.starts_with('-')));
                let addr = match v {
                    Some(v) => v.parse::<std::net::SocketAddr>()
                        .map_err(|_| format!("invalid --web address: {v} (e.g. 0.0.0.0:8080)"))?,
                    None => "0.0.0.0:8080".parse().unwrap(),
                };
                return Ok(CliAction::Web(addr));
            }
            other if !other.starts_with('-') => {
                return Err(format!("unexpected argument {other}: {}", file_hint(other)));
            }
            other => return Err(format!("unknown option: {other}")),
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

/// Restores the terminal when `main` leaves it, whichever way (an error from `?`, a TUI error,
/// a panic unwinding); the panic hook restores it before the message is printed.
struct TerminalGuard;

impl Drop for TerminalGuard {
    fn drop(&mut self) {
        restore_terminal();
    }
}

fn main() -> Result<()> {
    let cli = match parse_args(std::env::args().skip(1)) {
        Ok(CliAction::Run(opts)) => opts,
        Ok(CliAction::Decode(opts)) => std::process::exit(headless::run(&opts)),
        Ok(CliAction::KeeloqBf(opts)) => {
            // A mistyped path used to be skipped silently, and the search ran on fewer captures.
            if let Some(path) = opts.paths.iter().find(|p| !p.exists()) {
                eprintln!("kat: --keeloq-bf {}: no such file or directory", path.display());
                std::process::exit(2);
            }
            std::process::exit(headless::keeloq_bf(&opts))
        }
        Ok(CliAction::Web(addr)) => {
            // The web UI does not need a TTY: run the App with the web server
            // instead of the TUI, on a tokio runtime.
            let rt = tokio::runtime::Runtime::new()?;
            rt.block_on(async {
                if let Err(e) = web::run_web(addr).await {
                    eprintln!("kat --web: {e:#}");
                    std::process::exit(1);
                }
            });
            return Ok(());
        }
        Ok(CliAction::Help) => {
            println!("KAT v{VERSION} - Keyfob Analysis Toolkit\n\n{USAGE}");
            return Ok(());
        }
        Ok(CliAction::Version) => {
            println!("kat {VERSION}");
            return Ok(());
        }
        Err(e) => {
            eprintln!("kat: {e}\nRun kat --help for the options.");
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
        restore_terminal();
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

    // Create the app (config, radio probe, saved session) before the terminal goes raw: an error
    // here (e.g. the config folder cannot be created) prints normally and leaves the shell usable.
    let mut app = App::new()?;
    if let Some(path) = cli.iq {
        // A file needs no radio: skip the "no device" warning, but not the restore and import
        // prompts (skipping the restore prompt left autosave off for the whole run).
        if app.input_mode == InputMode::HackRfNotDetected {
            app.input_mode = app.next_startup_mode();
        }
        app.start_iq_file(path, cli.freq, cli.rate);
    }

    // Setup terminal (restored by the guard on every way out, before an error is printed)
    enable_raw_mode()?;
    let _restore = TerminalGuard;
    execute!(io::stdout(), EnterAlternateScreen)?;
    let backend = CrosstermBackend::new(io::stdout());
    let mut terminal = Terminal::new(backend)?;

    // An error is printed once, by `main`'s return, after the guard restored the terminal and
    // the app saved the session.
    run_app(&mut terminal, &mut app)
}

/// Run an app action from the key loop; if it fails (a radio that won't start or retune, a file
/// that can't be read or written…), say so in the status bar and keep going instead of ending KAT.
macro_rules! report {
    ($app:expr, $action:expr) => {
        if let Err(e) = $action {
            tracing::warn!("{e:#}");
            $app.last_error = Some(format!("{e:#}"));
        }
    };
}

fn run_app<B: ratatui::backend::Backend>(terminal: &mut Terminal<B>, app: &mut App) -> Result<()> {
    loop {
        terminal.draw(|f| draw_ui(f, app))?;

        // Queued transmissions go out one per pass, each after a frame that shows it ("Transmitting
        // 3/20…"), with the keys typed meanwhile read in between: Esc cancels the rest. (The whole
        // queue used to run without reading keys, so they all ran afterwards: q quit KAT.)
        let transmitting = app.has_pending_transmit();
        let wait = std::time::Duration::from_millis(if transmitting { 0 } else { 100 });
        let had_event = event::poll(wait)?;
        if had_event {
            if let Event::Key(key) = event::read()? {
                // Ctrl-C, Ctrl-H; other Ctrl / Alt characters are dropped (app/input.rs).
                let code = app.key_code(&key).filter(|_| key.kind == KeyEventKind::Press);
                if let Some(code) = code {
                    // In the list every key is a new action: the last message or error goes.
                    if app.input_mode == InputMode::Normal {
                        app.clear_messages();
                    }
                    match app.input_mode {
                        InputMode::Normal => match code {
                            // Ctrl-C arrives as q; both ask first while work runs (app/input.rs).
                            KeyCode::Char('q') => {
                                if app.request_quit() {
                                    return Ok(());
                                }
                            }
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
                            KeyCode::PageDown => app.page_captures(1),
                            KeyCode::PageUp => app.page_captures(-1),
                            KeyCode::Home | KeyCode::Char('g') => app.select_capture_edge(false),
                            KeyCode::End | KeyCode::Char('G') => app.select_capture_edge(true),
                            KeyCode::Char('r') => {
                                report!(app, app.toggle_receiving());
                            }
                            KeyCode::Char('d') => {
                                let _ = app.delete_selected_capture();
                            }
                            KeyCode::Char('u') => app.undo_delete(),
                            KeyCode::Enter => {
                                // Action menu of the selected capture (it stays on that capture)
                                app.open_signal_menu();
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
                            KeyCode::Char('?') | KeyCode::F(1) => app.open_help(),
                            // Esc clears the message (above) and drops queued transmissions.
                            KeyCode::Esc => {
                                app.cancel_transmit_queue();
                            }
                            _ => {}
                        },

                        InputMode::Command => match code {
                            KeyCode::Enter => {
                                let command = app.command_input.clone();
                                report!(app, app.execute_command(&command));
                                if app.quit_requested {
                                    return Ok(());
                                }
                                app.command_input.clear();
                                if app.input_mode == InputMode::Command {
                                    app.input_mode = InputMode::Normal;
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

                        // The menu acts on the capture it was opened for, and its highlight stays on
                        // the same action when the list changes (app/input.rs).
                        InputMode::SignalMenu => match code {
                            KeyCode::Up | KeyCode::Char('k') => app.signal_menu_move(false),
                            KeyCode::Down | KeyCode::Char('j') => app.signal_menu_move(true),
                            KeyCode::Enter => {
                                report!(app, app.execute_signal_action());
                                // Only return to Normal if the action didn't change
                                // input mode (e.g. ExportFob sets FobMetaYear)
                                if app.input_mode == InputMode::SignalMenu {
                                    app.input_mode = InputMode::Normal;
                                }
                            }
                            KeyCode::Esc => {
                                app.input_mode = InputMode::Normal;
                            }
                            _ => {}
                        },

                        InputMode::SettingsSelect => match code {
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

                        InputMode::SettingsEdit => match code {
                            KeyCode::Up | KeyCode::Char('k') => {
                                if app.settings_value_index > 0 {
                                    app.settings_value_index -= 1;
                                }
                            }
                            KeyCode::Down | KeyCode::Char('j') => {
                                let max = app.settings_value_count();
                                if app.settings_value_index + 1 < max {
                                    app.settings_value_index += 1;
                                }
                            }
                            KeyCode::Enter => {
                                report!(app, app.apply_settings_value());
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
                        InputMode::StartupImport => match code {
                            KeyCode::Char('y') | KeyCode::Char('Y') => {
                                report!(app, app.import_fob_files());
                                app.input_mode = InputMode::Normal;
                            }
                            KeyCode::Char('n') | KeyCode::Char('N') | KeyCode::Esc => {
                                app.skip_fob_import();
                                app.input_mode = InputMode::Normal;
                            }
                            _ => {}
                        },

                        // `:clear` cannot be undone: its y also forgets the last delete (`u`).
                        InputMode::ConfirmClear if matches!(code, KeyCode::Char('y' | 'Y')) => {
                            app.forget_deleted();
                            report!(app, app.handle_app_mode_key(code));
                        }

                        // Session restore prompt, :clear confirmation, timing overlay (app/persistence.rs)
                        InputMode::SessionRestore
                        | InputMode::ConfirmClear
                        | InputMode::TimingAnalysis => {
                            report!(app, app.handle_app_mode_key(code));
                        }

                        // "Quit? … is still running", `:delete all` (y/n; app/input.rs)
                        InputMode::ConfirmQuit | InputMode::ConfirmDeleteAll => {
                            if app.confirm_prompt_key(code) {
                                return Ok(());
                            }
                        }

                        // Export form (file name, .fob vehicle details) and capture metadata (`i`):
                        // Tab / Shift-Tab / Up / Down move between fields (app/input.rs)
                        InputMode::ExportFilename
                        | InputMode::FobMetaYear
                        | InputMode::FobMetaMake
                        | InputMode::FobMetaModel
                        | InputMode::FobMetaRegion
                        | InputMode::FobMetaCommand
                        | InputMode::FobMetaNotes
                        | InputMode::CaptureMetaYear
                        | InputMode::CaptureMetaMake
                        | InputMode::CaptureMetaModel
                        | InputMode::CaptureMetaRegion
                        | InputMode::CaptureMetaCommand => app.form_field_key(code),

                        // :new picker / field form, "Send button…" list (app/add_manually.rs)
                        InputMode::NewRemote | InputMode::NewRemoteEdit | InputMode::SendButton => {
                            // In the list, Esc first drops a transmission not sent yet.
                            let cancelled = code == KeyCode::Esc
                                && app.input_mode == InputMode::SendButton
                                && app.cancel_transmit_queue();
                            if !cancelled {
                                app.handle_add_manually_key(code);
                            }
                        }

                        // Folders, parent (Left / Backspace), files (app/input.rs)
                        InputMode::LoadFileBrowser => app.load_browser_key(code)?,

                        // Scroll within the text, Esc / Enter / q close (app/input.rs)
                        InputMode::License | InputMode::Credits | InputMode::Help => app.overlay_key(code),
                    }
                }
            }
        }
        // After a key, draw its result (a prompt, a menu) before the next transmission blocks.
        if transmitting && !had_event {
            report!(app, app.step_transmit_queue());
        }

        // Process any pending radio events
        report!(app, app.process_radio_events());
        // IQ file progress, recordings, hopping (src/app/rx.rs)
        app.poll_rx();
        // The open signal menu follows its capture's actions (app/input.rs)
        app.sync_signal_menu();
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

    /// Mistakes get a message that says what to do instead.
    #[test]
    fn argument_errors_explain_the_fix() {
        let err = |a: &[&str]| parse_args(args(a)).unwrap_err();
        // A rate the decoder refuses is rejected up front, with the range.
        let e = err(&["--decode", "x.cu8", "--rate", "150k"]);
        assert!(e.contains("150k") && e.contains("200k to 40M"), "{e}");
        assert!(parse_args(args(&["--decode", "x.cu8", "--rate", "200k"])).is_ok());
        // A bare file: how to open it.
        let e = err(&["rec.cu8"]);
        assert!(e.contains("kat --iq rec.cu8") && e.contains("kat --decode rec.cu8"), "{e}");
        let e = err(&["Garage.SUB"]);
        assert!(e.contains("kat --decode Garage.SUB") && e.contains(":load"), "{e}");
        // --iq only takes IQ recordings (a .sub used to fail inside the TUI).
        let e = err(&["--iq", "foo.sub"]);
        assert!(e.contains("not an IQ recording") && e.contains("kat --decode foo.sub"), "{e}");
        assert!(err(&["--iq", "notes.fob"]).contains(":load notes.fob"));
        assert_eq!(err(&["--bogus"]), "unknown option: --bogus");
        assert!(err(&["--freq", "x"]).contains("433.92"));
    }

    /// The short aliases and the exit status are in --help.
    #[test]
    fn usage_lists_aliases_and_exit_status() {
        for needle in ["-d, --decode", "-v, --verbose", "-h, --help", "-V, --version", "Exit status: 0", "200k to 40M"] {
            assert!(USAGE.contains(needle), "{needle}");
        }
        assert_eq!(parse_args(args(&["-d", "a.sub", "-v"])).map(|a| matches!(a, CliAction::Decode(o) if o.verbose)), Ok(true));
    }
}
