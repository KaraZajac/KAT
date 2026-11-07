//! Web UI backend for KAT: exposes the [`App`] over HTTP + WebSocket so a
//! browser can drive the radio instead of the TUI.
//!
//! Enabled with `kat --web [addr]` (default `0.0.0.0:8080`). The App runs as
//! usual (radio thread feeds it captures); this module serves a JSON API and a
//! WebSocket that pushes every state change to the connected frontend.
//!
//! API (all JSON):
//!   GET  /api/state                  radio state, frequency, gains, capture count, rssi
//!   GET  /api/captures               the full capture list
//!   POST /api/command  {cmd}         run a TUI command ("freq 433.92", "hop", "lock 3", ...)
//!   POST /api/action   {action,id,button?}  per-capture action (replay, lock, unlock, trunk,
//!                                    panic, next_code, export_fob, export_flipper, delete)
//!   GET  /api/capture/:id            one capture with its raw pairs
//!   WS   /ws                         pushes {type:"state"} on every change + {type:"capture",...}
//!                                    for each new capture, live.

use std::net::SocketAddr;
use std::sync::{Arc, Mutex};

use anyhow::Result;
use axum::{
    extract::{
        ws::{Message, WebSocket, WebSocketUpgrade},
        Path, State,
    },
    response::{Html, IntoResponse, Json},
    routing::{get, post},
    Router,
};
use serde::{Deserialize, Serialize};

use crate::app::App;
use crate::capture::Capture;

/// Shared app handle for the HTTP handlers + the WS broadcaster.
pub type SharedApp = Arc<Mutex<App>>;

/// The frontend, embedded at compile time (single-file, no static dir needed).
const INDEX_HTML: &str = include_str!("web/index.html");

/// Build the web router.
pub fn router(app: SharedApp) -> Router {
    Router::new()
        .route("/", get(index))
        .route("/api/state", get(get_state))
        .route("/api/captures", get(get_captures))
        .route("/api/capture/:id", get(get_capture))
        .route("/api/command", post(post_command))
        .route("/api/action", post(post_action))
        .route("/api/rx", post(post_rx))
        .route("/ws", get(ws_handler))
        .with_state(app)
}

/// Serve the frontend.
async fn index() -> Html<&'static str> {
    Html(INDEX_HTML)
}

/// One capture as the frontend shows it.
#[derive(Serialize)]
struct CaptureOut {
    id: u32,
    timestamp: String,
    frequency: u32,
    freq_mhz: String,
    protocol: Option<String>,
    serial: Option<u32>,
    button: Option<u8>,
    counter: Option<u16>,
    crc_valid: bool,
    data: String,
    data_count_bit: usize,
    n_raw_pairs: usize,
}

impl From<&Capture> for CaptureOut {
    fn from(c: &Capture) -> Self {
        CaptureOut {
            id: c.id,
            timestamp: c.timestamp.format("%H:%M:%S").to_string(),
            frequency: c.frequency,
            freq_mhz: crate::radio::hopper::format_mhz(c.frequency),
            protocol: c.protocol.clone(),
            serial: c.serial,
            button: c.button,
            counter: c.counter,
            crc_valid: c.crc_valid,
            data: format!("0x{:X}", c.data),
            data_count_bit: c.data_count_bit,
            n_raw_pairs: c.raw_pairs.len(),
        }
    }
}

/// The app state the header shows.
#[derive(Serialize)]
struct StateOut {
    radio_state: String,
    device: Option<String>,
    can_transmit: bool,
    frequency: u32,
    freq_mhz: String,
    lna_gain: u32,
    vga_gain: u32,
    amp_enabled: bool,
    rssi: f32,
    n_captures: usize,
    status: Option<String>,
    error: Option<String>,
}

fn state_of(app: &App) -> StateOut {
    StateOut {
        radio_state: app.radio_state.to_string(),
        device: app.radio_device_name().map(|s| s.to_string()),
        can_transmit: app.can_transmit(),
        frequency: app.frequency,
        freq_mhz: crate::radio::hopper::format_mhz(app.frequency),
        lna_gain: app.lna_gain,
        vga_gain: app.vga_gain,
        amp_enabled: app.amp_enabled,
        rssi: app.rssi,
        n_captures: app.captures.len(),
        status: app.status_message.clone(),
        error: app.last_error.clone(),
    }
}

async fn get_state(State(app): State<SharedApp>) -> Json<StateOut> {
    let app = app.lock().unwrap();
    Json(state_of(&app))
}

async fn get_captures(State(app): State<SharedApp>) -> Json<Vec<CaptureOut>> {
    let app = app.lock().unwrap();
    Json(app.captures.iter().map(CaptureOut::from).collect())
}

async fn get_capture(State(app): State<SharedApp>, Path(id): Path<u32>) -> impl IntoResponse {
    let app = app.lock().unwrap();
    match app.captures.iter().find(|c| c.id == id) {
        Some(c) => Json(serde_json::json!({
            "capture": CaptureOut::from(c),
            "raw_pairs": c.raw_pairs.iter().map(|p| serde_json::json!({
                "level": p.level, "duration_us": p.duration_us,
            })).collect::<Vec<_>>(),
        })).into_response(),
        None => (axum::http::StatusCode::NOT_FOUND, "capture not found").into_response(),
    }
}

/// Toggle receiving on/off (the TUI's `r` key — not a `:command`, a direct App
/// call). The frontend's RX button hits this instead of sending "r" as a command
/// (which the App would reject as unknown).
async fn post_rx(State(app): State<SharedApp>) -> impl IntoResponse {
    let mut app = app.lock().unwrap();
    match app.toggle_receiving() {
        Ok(()) => Json(serde_json::json!({
            "ok": true,
            "radio_state": app.radio_state.to_string(),
            "status": app.status_message,
            "error": app.last_error,
        })),
        Err(e) => Json(serde_json::json!({"ok": false, "error": e.to_string()})),
    }
}

/// A TUI command from the frontend.
#[derive(Deserialize)]
struct CommandIn {
    cmd: String,
}

async fn post_command(
    State(app): State<SharedApp>,
    Json(body): Json<CommandIn>,
) -> impl IntoResponse {
    let mut app = app.lock().unwrap();
    let cmd = body.cmd.trim().to_string();
    if cmd.is_empty() {
        return Json(serde_json::json!({"ok": false, "error": "empty command"}));
    }
    match app.execute_command(&cmd) {
        Ok(()) => Json(serde_json::json!({
            "ok": true,
            "status": app.status_message,
            "error": app.last_error,
        })),
        Err(e) => Json(serde_json::json!({"ok": false, "error": e.to_string()})),
    }
}

/// A per-capture action from the frontend.
#[derive(Deserialize)]
struct ActionIn {
    action: String,
    id: u32,
    button: Option<u8>,
}

async fn post_action(
    State(app): State<SharedApp>,
    Json(body): Json<ActionIn>,
) -> impl IntoResponse {
    let mut app = app.lock().unwrap();
    let id = body.id;
    let result = match body.action.as_str() {
        "replay" => app.replay_capture(id),
        "next_code" => app.transmit_next_code(id),
        "button" => app.transmit_button(id, body.button),
        "lock" => app.transmit_button(id, Some(1)),
        "unlock" => app.transmit_button(id, Some(2)),
        "trunk" => app.transmit_button(id, Some(3)),
        "panic" => app.transmit_button(id, Some(4)),
        "export_fob" => app.export_fob(id),
        "export_flipper" => app.export_flipper(id),
        // delete_capture is pub(super); go through the public :delete command instead.
        "delete" => app.execute_command(&format!("delete {id}")),
        _ => {
            return Json(serde_json::json!({"ok": false, "error": "unknown action"}));
        }
    };
    match result {
        Ok(()) => Json(serde_json::json!({
            "ok": true,
            "status": app.status_message,
            "error": app.last_error,
        })),
        Err(e) => Json(serde_json::json!({"ok": false, "error": e.to_string()})),
    }
}

// ---------------------------------------------------------------------------
// WebSocket: push the state on connect and after every change, and each new
// capture live. The web task polls the App for new captures and broadcasts.
// ---------------------------------------------------------------------------
async fn ws_handler(ws: WebSocketUpgrade, State(app): State<SharedApp>) -> impl IntoResponse {
    ws.on_upgrade(move |socket| ws_loop(socket, app))
}

async fn ws_loop(mut socket: WebSocket, app: SharedApp) {
    // Send the current state + captures on connect.
    {
        let (state, captures) = {
            let a = app.lock().unwrap();
            (state_of(&a), a.captures.iter().map(CaptureOut::from).collect::<Vec<_>>())
        };
        let msg = serde_json::json!({"type": "init", "state": state, "captures": captures});
        if socket.send(Message::Text(msg.to_string())).await.is_err() {
            return;
        }
    }

    let mut last_n = 0usize;
    let mut last_state = String::new();
    loop {
        // Poll the app for new captures and state changes (~5 Hz, cheap).
        let (state, captures) = {
            let a = app.lock().unwrap();
            (state_of(&a), a.captures.iter().map(CaptureOut::from).collect::<Vec<_>>())
        };

        // New captures since last tick.
        if captures.len() > last_n {
            for c in &captures[last_n..] {
                let msg = serde_json::json!({"type": "capture", "capture": c});
                if socket.send(Message::Text(msg.to_string())).await.is_err() {
                    return;
                }
            }
            last_n = captures.len();
        }
        // A capture list shrink (delete) or any change: resync the whole list.
        if captures.len() < last_n {
            last_n = captures.len();
            let msg = serde_json::json!({"type": "captures", "captures": captures});
            if socket.send(Message::Text(msg.to_string())).await.is_err() {
                return;
            }
        }

        // State change (radio state, rssi, status, error).
        let state_json = serde_json::to_string(&state).unwrap_or_default();
        if state_json != last_state {
            last_state = state_json;
            let msg = serde_json::json!({"type": "state", "state": state});
            if socket.send(Message::Text(msg.to_string())).await.is_err() {
                return;
            }
        }

        tokio::time::sleep(std::time::Duration::from_millis(200)).await;
    }
}

/// Run the App with the web UI instead of the TUI (`kat --web`).
///
/// The App runs as usual: the radio thread feeds it captures, and a background
/// task drives `process_radio_events` (the TUI's main-loop equivalent) so the
/// app keeps decoding and matching. The web server runs on the main task.
pub async fn run_web(addr: SocketAddr) -> Result<()> {
    let app = Arc::new(Mutex::new(App::new()?));

    // Drive the app's main-loop work (process radio events, match saved fobs,
    // autosave) in the background — what the TUI's draw loop does per frame.
    {
        let app = Arc::clone(&app);
        tokio::spawn(async move {
            loop {
                {
                    let mut a = app.lock().unwrap();
                    let _ = a.process_radio_events();
                }
                tokio::time::sleep(std::time::Duration::from_millis(50)).await;
            }
        });
    }

    let router = router(app);
    let listener = tokio::net::TcpListener::bind(addr).await?;
    tracing::info!("KAT web UI on http://{addr}");
    axum::serve(listener, router).await?;
    Ok(())
}
