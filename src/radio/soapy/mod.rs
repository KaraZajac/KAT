//! Any radio SoapySDR drives (Airspy, SDRplay, LimeSDR, PlutoSDR, USRP, bladeRF …) as a receiver,
//! when SoapySDR is installed.
//!
//! KAT does not link SoapySDR. A helper copy of KAT (`kat --soapy-helper`, [helper]) loads the
//! system's SoapySDR library ([ffi]), lists the radios or receives from one, and streams the
//! samples to KAT through a pipe ([protocol]). The radios' own libraries thus run in the helper:
//! what they print goes to kat.log instead of the screen, a crash in one ends the helper and not
//! KAT, and SoapySDR's HackRF module does not share KAT's HackRF library (its listing shuts that
//! library's USB down). KAT leaves out the RTL-SDRs and HackRFs SoapySDR finds: it drives them
//! itself.
//!
//! The controller works as the RTL-SDR's (src/radio/rtlsdr.rs): the same receive pipeline (LO
//! fs/8 below the target), hopping by retunes between reads, `:record` of the raw samples (complex
//! f32, `.cf32`), one gain (KAT's VGA value, taken into the radio's range).

mod ffi;
mod helper;
mod protocol;

pub use helper::main as helper_main;

use anyhow::{Context, Result};
use std::io::{BufRead, BufReader, Write};
use std::process::{Child, Command as Process, Stdio};
use std::sync::mpsc::{self, Sender};
use std::sync::{
    atomic::{AtomicBool, AtomicU32, Ordering},
    Arc, Mutex,
};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crate::app::RadioEvent;

use super::demodulator::LevelDuration;
use super::devices::FoundRadio;
use super::dsp::IqFormat;
use super::hopper::HopPlan;
use super::recorder::{RecordingStatus, RecordingSummary};
use super::rx::{LiveRx, LiveShared};
use protocol::{Command, Listing, Ready, ERROR, IQ, READY, TUNE_FAILED};

/// The first argument that makes KAT the SoapySDR helper.
pub const HELPER_FLAG: &str = "--soapy-helper";

/// The sample rate KAT asks for; the radio's own is used when it has not that one.
const SAMPLE_RATE: u32 = 2_000_000;

/// How long listing the radios may take (a network module looks around for a while).
const FIND_TIMEOUT: Duration = Duration::from_secs(15);

/// Start a helper with these arguments, its stderr copied into kat.log.
fn spawn_helper(args: &[String], stdin: Stdio) -> std::io::Result<Child> {
    let exe = std::env::current_exe()?;
    let mut child = Process::new(exe)
        .arg(HELPER_FLAG)
        .args(args)
        .stdin(stdin)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()?;
    if let Some(stderr) = child.stderr.take() {
        thread::spawn(move || {
            for line in BufReader::new(stderr).lines().map_while(Result::ok) {
                tracing::info!("SoapySDR: {line}");
            }
        });
    }
    Ok(child)
}

/// The radios SoapySDR finds ([super::devices::discover]), but the RTL-SDRs and HackRFs KAT
/// drives itself; why none can be used when there is none.
pub fn find_devices() -> (Vec<FoundRadio>, Option<String>) {
    let listings = match list() {
        Ok(l) => l,
        Err(why) => return (Vec::new(), Some(why)),
    };
    let mut found = Vec::new();
    let mut problem = None;
    for l in listings {
        match l {
            Listing::Radio(args) => found.push(FoundRadio::soapy(args)),
            Listing::Missing(tried) => {
                tracing::info!("SoapySDR not installed: {tried}");
                problem = Some("not installed (KAT uses it when it is)".to_string());
            }
            Listing::Error(e) => problem = Some(format!("cannot list its radios ({e})")),
        }
    }
    if !found.is_empty() {
        tracing::info!("SoapySDR radio(s) found: {}", found.iter().map(FoundRadio::identity).collect::<Vec<_>>().join(", "));
        return (found, None);
    }
    (found, Some(problem.unwrap_or_else(|| "no radio found (KAT drives RTL-SDRs and HackRFs itself)".to_string())))
}

/// `kat --soapy-helper find`'s lines.
fn list() -> Result<Vec<Listing>, String> {
    let mut child = spawn_helper(&["find".to_string()], Stdio::null()).map_err(|e| format!("cannot start KAT's SoapySDR helper ({e})"))?;
    let Some(stdout) = child.stdout.take() else {
        let _ = wait_or_kill(&mut child, Duration::ZERO);
        return Err("no output from KAT's SoapySDR helper".to_string());
    };
    let (tx, rx) = mpsc::channel();
    thread::spawn(move || {
        let lines: Vec<String> = BufReader::new(stdout).lines().map_while(Result::ok).collect();
        let _ = tx.send(lines);
    });
    let lines = match rx.recv_timeout(FIND_TIMEOUT) {
        Ok(lines) => lines,
        Err(_) => {
            let _ = wait_or_kill(&mut child, Duration::ZERO);
            return Err(format!("listing its radios took over {} s", FIND_TIMEOUT.as_secs()));
        }
    };
    let status = wait_or_kill(&mut child, DEFAULT_TIMEOUTS.stop_grace);
    let listings: Vec<Listing> = lines.iter().filter_map(|l| serde_json::from_str(l).ok()).collect();
    if listings.is_empty() && !matches!(status, Ok(s) if s.success()) {
        return Err(format!("KAT's SoapySDR helper failed ({})", status.map_or_else(|e| e.to_string(), |s| s.to_string())));
    }
    Ok(listings)
}

/// A SoapySDR radio: receive only.
pub struct SoapyController {
    /// The arguments that open it (SoapySDR's: driver, serial, label …).
    device: Vec<(String, String)>,
    event_tx: Sender<RadioEvent>,
    receiving: Arc<AtomicBool>,
    rx_thread: Option<JoinHandle<()>>,
    /// The gain to set: dB, or None for the radio's automatic gain.
    gain: Arc<Mutex<Option<f64>>>,
    /// The sample rate the radio runs at (KAT's 2 MS/s until it says otherwise).
    rate: Arc<AtomicU32>,
    rssi_value: Arc<AtomicU32>,
    live: Arc<LiveShared>,
}

impl SoapyController {
    pub fn new(event_tx: Sender<RadioEvent>, device: Vec<(String, String)>) -> Self {
        let receiving = Arc::new(AtomicBool::new(false));
        let rssi_value = Arc::new(AtomicU32::new(0));
        let live = Arc::new(LiveShared::new(receiving.clone(), rssi_value.clone(), 433_920_000));
        Self {
            device,
            event_tx,
            receiving,
            rx_thread: None,
            gain: Arc::new(Mutex::new(None)),
            rate: Arc::new(AtomicU32::new(SAMPLE_RATE)),
            rssi_value,
            live,
        }
    }

    pub fn rssi_source(&self) -> Arc<AtomicU32> {
        self.rssi_value.clone()
    }

    pub fn supports_tx(&self) -> bool {
        false
    }

    pub fn start_receiving(&mut self, frequency: u32) -> Result<()> {
        if self.receiving.load(Ordering::SeqCst) {
            return Ok(());
        }
        // A receiver that stopped on its own leaves its thread behind.
        if let Some(handle) = self.rx_thread.take() {
            let _ = handle.join();
        }
        self.live.home.store(frequency, Ordering::SeqCst);
        self.receiving.store(true, Ordering::SeqCst);
        let (live, event_tx, gain, rate, device) =
            (self.live.clone(), self.event_tx.clone(), self.gain.clone(), self.rate.clone(), self.device.clone());
        match thread::Builder::new().name("kat-soapy-ctrl".to_string()).spawn(move || {
            if let Err(e) = run_receiver(&live, event_tx.clone(), &gain, &rate, &device) {
                live.receiving.store(false, Ordering::SeqCst);
                live.end_recording_with_rx();
                live.rssi.store(0, Ordering::Relaxed);
                tracing::warn!("SoapySDR receiver stopped: {e:#}");
                let _ = event_tx.send(RadioEvent::Error(format!("{e:#}")));
            }
        }) {
            Ok(handle) => {
                self.rx_thread = Some(handle);
                tracing::info!("Started SoapySDR receiving at {frequency} Hz");
                Ok(())
            }
            Err(e) => {
                self.receiving.store(false, Ordering::SeqCst);
                Err(e).context("Cannot start SoapySDR receiver thread")
            }
        }
    }

    pub fn stop_receiving(&mut self) -> Result<()> {
        self.receiving.store(false, Ordering::SeqCst);
        if let Some(handle) = self.rx_thread.take() {
            let _ = handle.join();
        }
        self.rssi_value.store(0, Ordering::Relaxed);
        self.live.end_recording_with_rx();
        tracing::info!("Stopped SoapySDR receiving");
        Ok(())
    }

    pub fn set_frequency(&mut self, frequency: u32) -> Result<()> {
        self.live.home.store(frequency, Ordering::SeqCst);
        self.live.target.store(frequency, Ordering::SeqCst);
        Ok(())
    }

    /// No-op: KAT receives with SoapySDR radios only.
    pub fn transmit(&mut self, _signal: &[LevelDuration], _frequency: u32) -> Result<()> {
        tracing::warn!("Transmit not available – SoapySDR radios are receive-only in KAT");
        Ok(())
    }

    /// The gain in dB: KAT's VGA setting, taken into the radio's range.
    pub fn set_gain(&mut self, gain: u32) -> Result<()> {
        *self.gain.lock().unwrap_or_else(|e| e.into_inner()) = Some(f64::from(gain));
        Ok(())
    }

    pub fn iq_format(&self) -> IqFormat {
        IqFormat::Cf32
    }

    pub fn sample_rate(&self) -> u32 {
        self.rate.load(Ordering::SeqCst)
    }

    pub fn rx_frequency(&self) -> u32 {
        self.live.target.load(Ordering::SeqCst)
    }

    pub fn set_hopping(&mut self, plan: Option<HopPlan>) {
        self.live.set_hop_plan(plan);
    }

    pub fn hop_state(&self) -> Option<(HopPlan, usize)> {
        self.live.hop_plan().map(|p| (p, self.live.hop_index.load(Ordering::SeqCst) as usize))
    }

    pub fn hop_pause(&mut self, dwells: u32) {
        self.live.hop_pause_dwells.store(dwells, Ordering::SeqCst);
    }

    pub fn start_recording(&mut self, path: std::path::PathBuf) -> Result<()> {
        self.live.start_recording(path)
    }

    pub fn stop_recording(&mut self) -> Option<RecordingSummary> {
        self.live.stop_recording()
    }

    pub fn recording_status(&self) -> Option<RecordingStatus> {
        self.live.recording_status()
    }

    pub fn take_ended_recording(&mut self) -> Option<RecordingSummary> {
        self.live.ended_recording.lock().ok().and_then(|mut g| g.take())
    }
}

impl Drop for SoapyController {
    fn drop(&mut self) {
        self.receiving.store(false, Ordering::SeqCst);
        if let Some(handle) = self.rx_thread.take() {
            let _ = handle.join();
        }
    }
}

/// Timeouts governing the receiver helper process.
#[derive(Debug, Clone, Copy)]
struct HelperTimeouts {
    /// How often the receiver loop wakes to check `live.receiving`, hop requests, and gain updates
    /// when no frame has arrived yet.
    poll: Duration,
    /// Maximum time to wait for the helper's `READY` frame after spawning it.
    ready: Duration,
    /// Maximum silence from a streaming helper (which normally sends a heartbeat every 50 ms)
    /// before treating the driver as stalled and killing the helper.
    stall: Duration,
    /// Grace period for the helper to exit cleanly after `Command::Quit` before killing it.
    stop_grace: Duration,
}

const DEFAULT_TIMEOUTS: HelperTimeouts = HelperTimeouts {
    poll: Duration::from_millis(50),
    ready: FIND_TIMEOUT,
    stall: Duration::from_secs(10),
    stop_grace: Duration::from_secs(3),
};

enum HelperFrame {
    Frame(u8, Vec<u8>),
    Eof,
    Error(std::io::Error),
}

fn format_duration(d: Duration) -> String {
    if d.subsec_nanos() == 0 {
        format!("{} s", d.as_secs())
    } else {
        format!("{} ms", d.as_millis())
    }
}

/// Owned handle to a running helper process: guarantees the child is stopped and reaped even on
/// early error returns.
struct HelperProcess {
    child: Child,
    stdin: Option<std::process::ChildStdin>,
    stopped: bool,
}

impl HelperProcess {
    fn new(mut child: Child) -> Self {
        let stdin = child.stdin.take();
        Self { child, stdin, stopped: false }
    }

    fn send(&mut self, c: Command) {
        if let Some(s) = self.stdin.as_mut() {
            let _ = s.write_all(c.line().as_bytes()).and_then(|_| s.flush());
        }
    }

    /// Ask the helper to stop, wait up to `grace` for a clean exit, then kill and reap it.
    /// The caller should drop its frame receiver first so the reader thread closes stdout if full,
    /// allowing a helper blocked on writing a frame to exit on broken pipe.
    fn stop(&mut self, grace: Duration) {
        if self.stopped {
            return;
        }
        self.stopped = true;
        if let Some(mut s) = self.stdin.take() {
            let _ = s.write_all(Command::Quit.line().as_bytes());
            let _ = s.flush();
        }
        let _ = wait_or_kill(&mut self.child, grace);
    }
}

impl Drop for HelperProcess {
    fn drop(&mut self) {
        if !self.stopped {
            self.stop(Duration::ZERO);
        }
    }
}

/// Wait up to `grace` for `child` to exit on its own; kill and reap it if it does not.
fn wait_or_kill(child: &mut Child, grace: Duration) -> std::io::Result<std::process::ExitStatus> {
    if !grace.is_zero() {
        let deadline = Instant::now() + grace;
        loop {
            if let Some(status) = child.try_wait()? {
                return Ok(status);
            }
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                break;
            }
            thread::sleep(remaining.min(Duration::from_millis(20)));
        }
    } else if let Some(status) = child.try_wait()? {
        return Ok(status);
    }
    tracing::warn!("SoapySDR helper did not stop: killed");
    let _ = child.kill();
    child.wait()
}

/// Read framed messages from the helper's stdout on a dedicated thread so the receiver loop can
/// always time out and react to stop requests even if a SoapySDR C call blocks indefinitely.
fn spawn_frame_reader(stdout: std::process::ChildStdout) -> std::io::Result<mpsc::Receiver<HelperFrame>> {
    let (tx, rx) = mpsc::sync_channel(16);
    thread::Builder::new().name("kat-soapy-io".to_string()).spawn(move || {
        let mut stdout = BufReader::with_capacity(1 << 20, stdout);
        loop {
            match protocol::read_frame(&mut stdout) {
                Ok(Some((tag, payload))) => {
                    if tx.send(HelperFrame::Frame(tag, payload)).is_err() {
                        break;
                    }
                }
                Ok(None) => {
                    let _ = tx.send(HelperFrame::Eof);
                    break;
                }
                Err(e) => {
                    let _ = tx.send(HelperFrame::Error(e));
                    break;
                }
            }
        }
    })?;
    Ok(rx)
}

struct ReceiverSession<'a> {
    live: &'a Arc<LiveShared>,
    event_tx: Sender<RadioEvent>,
    gain: &'a Mutex<Option<f64>>,
    rate: &'a AtomicU32,
    target: u32,
    applied: Option<f64>,
}

/// One receive run: a helper for the radio, its samples through the pipeline until RX stops.
fn run_receiver(
    live: &Arc<LiveShared>,
    event_tx: Sender<RadioEvent>,
    gain: &Mutex<Option<f64>>,
    rate: &AtomicU32,
    device: &[(String, String)],
) -> Result<()> {
    let target = live.start_frequency();
    live.target.store(target, Ordering::SeqCst);
    let applied = *gain.lock().unwrap_or_else(|e| e.into_inner());
    let child = spawn_helper(&protocol::rx_args(target, applied, device), Stdio::piped())
        .context("Cannot start KAT's SoapySDR helper")?;
    let session = ReceiverSession { live, event_tx, gain, rate, target, applied };
    run_receiver_with_child(session, child, DEFAULT_TIMEOUTS)
}

fn run_receiver_with_child(session: ReceiverSession<'_>, child: Child, timeouts: HelperTimeouts) -> Result<()> {
    let ReceiverSession { live, event_tx, gain, rate, target, mut applied } = session;
    let mut helper = HelperProcess::new(child);
    let stdout = helper.child.stdout.take().context("no output from the SoapySDR helper")?;
    let frame_rx = spawn_frame_reader(stdout).context("Cannot start SoapySDR frame reader")?;

    // Ready, or why not. Polled with a timeout so stop_receiving / Drop never deadlock if the
    // helper hangs while opening the radio or starting its stream.
    let ready_deadline = Instant::now() + timeouts.ready;
    let ready: Ready = loop {
        if !live.receiving.load(Ordering::SeqCst) {
            drop(frame_rx);
            helper.stop(timeouts.stop_grace);
            return Ok(());
        }
        let Some(remaining) = ready_deadline.checked_duration_since(Instant::now()) else {
            drop(frame_rx);
            helper.stop(Duration::ZERO);
            anyhow::bail!("SoapySDR: opening the radio took over {}", format_duration(timeouts.ready));
        };
        match frame_rx.recv_timeout(timeouts.poll.min(remaining)) {
            Ok(HelperFrame::Frame(READY, p)) => match serde_json::from_slice::<Ready>(&p) {
                Ok(ready) if ready.rate > 0 => break ready,
                Ok(_) => {
                    drop(frame_rx);
                    helper.stop(timeouts.stop_grace);
                    anyhow::bail!("SoapySDR: invalid sample rate (0 S/s)");
                }
                Err(e) => {
                    drop(frame_rx);
                    helper.stop(timeouts.stop_grace);
                    anyhow::bail!("KAT's SoapySDR helper said something KAT cannot read ({e})");
                }
            },
            Ok(HelperFrame::Frame(ERROR, p)) => {
                drop(frame_rx);
                helper.stop(timeouts.stop_grace);
                anyhow::bail!("SoapySDR: {}", String::from_utf8_lossy(&p));
            }
            Ok(HelperFrame::Frame(_, _)) => continue,
            Ok(HelperFrame::Eof) | Ok(HelperFrame::Error(_)) | Err(mpsc::RecvTimeoutError::Disconnected) => {
                drop(frame_rx);
                helper.stop(timeouts.stop_grace);
                anyhow::bail!("KAT's SoapySDR helper ended before the radio streamed (kat.log has why)");
            }
            Err(mpsc::RecvTimeoutError::Timeout) => continue,
        }
    };
    rate.store(ready.rate, Ordering::SeqCst);
    tracing::info!(
        "SoapySDR {} ({}) streaming: target={target} Hz, {} S/s, gain {}..{} dB",
        ready.driver,
        ready.hardware,
        ready.rate,
        ready.gain_min,
        ready.gain_max
    );

    let (buf_tx, buf_rx) = mpsc::sync_channel::<(Vec<u8>, u32, u64)>(32);
    let proc_live = live.clone();
    let sample_rate = ready.rate;
    let processor = thread::Builder::new().name("kat-soapy-rx".to_string()).spawn(move || {
        let mut rx = LiveRx::new(sample_rate, IqFormat::Cf32, target, event_tx);
        let mut current = target;
        for (buf, tag, blank) in buf_rx {
            if tag != current {
                rx.retuned(tag, blank, &proc_live);
                current = tag;
            }
            rx.process(&buf, &proc_live);
        }
        rx.finish();
    })?;

    let mut tag = target;
    let mut blank = 0u64;
    let mut gain_check = Instant::now();
    let mut last_frame = Instant::now();
    let mut failed = None;
    let mut kill_now = false;
    while live.receiving.load(Ordering::SeqCst) {
        let req = live.hop_request.swap(0, Ordering::SeqCst);
        if req != 0 {
            helper.send(Command::Tune(req));
        }
        if gain_check.elapsed() >= Duration::from_millis(100) {
            gain_check = Instant::now();
            let g = *gain.lock().unwrap_or_else(|e| e.into_inner());
            if g != applied {
                helper.send(Command::Gain(g));
                applied = g;
                live.floor_reset.store(true, Ordering::SeqCst);
            }
        }
        match frame_rx.recv_timeout(timeouts.poll) {
            Ok(HelperFrame::Frame(IQ, p)) => {
                last_frame = Instant::now();
                if p.len() >= 4 {
                    let t = u32::from_le_bytes([p[0], p[1], p[2], p[3]]);
                    if t != tag {
                        tag = t;
                        blank = u64::from(sample_rate) / 500; // 2 ms for the PLL, as the RTL-SDR
                    }
                    if p.len() > 4 {
                        let samples = p[4..].to_vec();
                        live.record(&samples);
                        if buf_tx.send((samples, tag, blank)).is_err() {
                            break;
                        }
                        blank = 0;
                    }
                }
            }
            Ok(HelperFrame::Frame(TUNE_FAILED, p)) => {
                last_frame = Instant::now();
                let hz = p.get(..4).map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]));
                tracing::warn!("SoapySDR retune to {hz:?} Hz failed");
                live.hop_failed.store(true, Ordering::SeqCst);
            }
            Ok(HelperFrame::Frame(ERROR, p)) => {
                failed = Some(String::from_utf8_lossy(&p).into_owned());
                break;
            }
            Ok(HelperFrame::Frame(_, _)) => {
                last_frame = Instant::now();
            }
            Ok(HelperFrame::Eof) | Err(mpsc::RecvTimeoutError::Disconnected) => {
                failed = Some("KAT's SoapySDR helper ended (kat.log has why)".to_string());
                break;
            }
            Ok(HelperFrame::Error(e)) => {
                failed = Some(format!("reading from KAT's SoapySDR helper failed ({e})"));
                break;
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {
                if last_frame.elapsed() >= timeouts.stall {
                    failed = Some(format!(
                        "no data from KAT's SoapySDR helper for {} (driver stalled)",
                        format_duration(timeouts.stall)
                    ));
                    kill_now = true;
                    break;
                }
            }
        }
    }
    drop(frame_rx);
    helper.stop(if kill_now { Duration::ZERO } else { timeouts.stop_grace });
    drop(buf_tx);
    let _ = processor.join();
    if let Some(why) = failed {
        anyhow::bail!("SoapySDR: {why}");
    }
    tracing::info!("SoapySDR receiver thread stopped");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A radio found by SoapySDR keeps the arguments that open it, its serial for config
    /// sections, and its label in `:radios`.
    #[test]
    fn a_soapy_radio_is_named_by_its_arguments() {
        let args = vec![
            ("driver".to_string(), "airspy".to_string()),
            ("label".to_string(), "AirSpy One [12345678]".to_string()),
            ("serial".to_string(), "a74068c82f531693".to_string()),
        ];
        let found = FoundRadio::soapy(args.clone());
        assert_eq!(found.soapy_args, args);
        assert_eq!(found.serial.as_deref(), Some("a74068c82f531693"));
        assert_eq!(found.identity(), "AirSpy One [12345678]");
        assert!(found.matches_serial("531693"));
        assert!(found.same_device(&FoundRadio::soapy(args)));
        let bare = FoundRadio::soapy(vec![("driver".to_string(), "plutosdr".to_string())]);
        assert_eq!(bare.identity(), "plutosdr");
        assert!(!found.same_device(&bare));
    }

    #[cfg(unix)]
    mod process_tests {
        use super::*;
        use std::path::PathBuf;

        struct TempFile(PathBuf);

        impl TempFile {
            fn write(name: &str, bytes: &[u8]) -> Self {
                let path = std::env::temp_dir().join(format!("kat-soapy-test-{}-{name}", std::process::id()));
                std::fs::write(&path, bytes).unwrap();
                Self(path)
            }
        }

        impl Drop for TempFile {
            fn drop(&mut self) {
                let _ = std::fs::remove_file(&self.0);
            }
        }

        fn ready_frame_bytes(rate: u32) -> Vec<u8> {
            let ready = Ready {
                rate,
                driver: "test".to_string(),
                hardware: "dummy".to_string(),
                gain_min: 0.0,
                gain_max: 20.0,
            };
            let payload = serde_json::to_vec(&ready).unwrap();
            let mut out = Vec::new();
            protocol::write_frame(&mut out, READY, &[&payload]).unwrap();
            out
        }

        fn spawn_sh(script: &str, arg: Option<&std::path::Path>) -> Child {
            let mut cmd = Process::new("sh");
            cmd.arg("-c").arg(script);
            if let Some(p) = arg {
                cmd.arg("_").arg(p);
            }
            cmd.stdin(Stdio::piped()).stdout(Stdio::piped()).stderr(Stdio::null()).spawn().unwrap()
        }

        struct TestState {
            live: Arc<LiveShared>,
            tx: mpsc::Sender<RadioEvent>,
            gain: Arc<Mutex<Option<f64>>>,
            rate: Arc<AtomicU32>,
        }

        impl TestState {
            fn new() -> Self {
                let receiving = Arc::new(AtomicBool::new(true));
                let rssi = Arc::new(AtomicU32::new(0));
                let live = Arc::new(LiveShared::new(receiving, rssi, 433_920_000));
                let (tx, _rx) = mpsc::channel();
                Self { live, tx, gain: Arc::new(Mutex::new(None)), rate: Arc::new(AtomicU32::new(0)) }
            }

            fn session(&self) -> ReceiverSession<'_> {
                ReceiverSession {
                    live: &self.live,
                    event_tx: self.tx.clone(),
                    gain: &self.gain,
                    rate: &self.rate,
                    target: 433_920_000,
                    applied: None,
                }
            }
        }

        fn fast_timeouts() -> HelperTimeouts {
            HelperTimeouts {
                poll: Duration::from_millis(15),
                ready: Duration::from_millis(250),
                stall: Duration::from_millis(250),
                stop_grace: Duration::from_millis(100),
            }
        }

        #[test]
        fn stop_unblocks_when_helper_hangs_before_ready() {
            let state = TestState::new();
            let receiving = state.live.receiving.clone();
            let child = spawn_sh("exec sleep 60", None);
            let started = Instant::now();
            let handle = thread::spawn(move || run_receiver_with_child(state.session(), child, fast_timeouts()));

            thread::sleep(Duration::from_millis(40));
            receiving.store(false, Ordering::SeqCst);
            let res = handle.join().unwrap();
            assert!(res.is_ok(), "stopping a hanging startup must return Ok: {res:?}");
            assert!(started.elapsed() < Duration::from_secs(2), "stop hung: {:?}", started.elapsed());
        }

        #[test]
        fn ready_timeout_kills_hung_helper_and_reports_error() {
            let state = TestState::new();
            let child = spawn_sh("exec sleep 60", None);
            let started = Instant::now();
            let err = run_receiver_with_child(state.session(), child, fast_timeouts()).unwrap_err();
            assert!(format!("{err:#}").contains("opening the radio took over"), "{err:#}");
            assert!(started.elapsed() < Duration::from_secs(2), "ready timeout hung: {:?}", started.elapsed());
        }

        #[test]
        fn stop_unblocks_when_helper_stalls_after_ready() {
            let frame = TempFile::write("stop-after-ready", &ready_frame_bytes(2_000_000));
            let state = TestState::new();
            let receiving = state.live.receiving.clone();
            let rate = state.rate.clone();
            let child = spawn_sh("cat \"$1\"; exec sleep 60", Some(&frame.0));
            let timeouts = HelperTimeouts { stall: Duration::from_secs(30), ..fast_timeouts() };
            let handle = thread::spawn(move || run_receiver_with_child(state.session(), child, timeouts));

            let wait_ready = Instant::now();
            while rate.load(Ordering::SeqCst) != 2_000_000 {
                assert!(wait_ready.elapsed() < Duration::from_secs(2), "never received READY");
                thread::sleep(Duration::from_millis(10));
            }
            let stop_start = Instant::now();
            receiving.store(false, Ordering::SeqCst);
            let res = handle.join().unwrap();
            assert!(res.is_ok(), "stopping a stalled stream must return Ok: {res:?}");
            assert!(stop_start.elapsed() < Duration::from_secs(2), "stop hung: {:?}", stop_start.elapsed());
        }

        #[test]
        fn stall_timeout_kills_silent_helper_after_ready() {
            let frame = TempFile::write("stall-after-ready", &ready_frame_bytes(2_000_000));
            let state = TestState::new();
            let child = spawn_sh("cat \"$1\"; exec sleep 60", Some(&frame.0));
            let started = Instant::now();
            let err = run_receiver_with_child(state.session(), child, fast_timeouts()).unwrap_err();
            assert_eq!(state.rate.load(Ordering::SeqCst), 2_000_000);
            assert!(format!("{err:#}").contains("driver stalled"), "{err:#}");
            assert!(started.elapsed() < Duration::from_secs(2), "stall timeout hung: {:?}", started.elapsed());
        }

        #[test]
        fn clean_quit_exits_responsive_helper_before_kill_grace() {
            let frame = TempFile::write("clean-quit", &ready_frame_bytes(2_000_000));
            let state = TestState::new();
            let receiving = state.live.receiving.clone();
            let rate = state.rate.clone();
            let child = spawn_sh(
                "cat \"$1\"; while IFS= read -r line; do [ \"$line\" = quit ] && exit 0; done",
                Some(&frame.0),
            );
            // Long kill grace proves the helper exited via Command::Quit rather than SIGKILL.
            let timeouts = HelperTimeouts { stop_grace: Duration::from_secs(5), ..fast_timeouts() };
            let handle = thread::spawn(move || run_receiver_with_child(state.session(), child, timeouts));

            let wait_ready = Instant::now();
            while rate.load(Ordering::SeqCst) != 2_000_000 {
                assert!(wait_ready.elapsed() < Duration::from_secs(2), "never received READY");
                thread::sleep(Duration::from_millis(10));
            }
            let stop_start = Instant::now();
            receiving.store(false, Ordering::SeqCst);
            let res = handle.join().unwrap();
            assert!(res.is_ok(), "{res:?}");
            assert!(
                stop_start.elapsed() < Duration::from_secs(2),
                "responsive helper waited for kill grace instead of quitting cleanly: {:?}",
                stop_start.elapsed()
            );
        }

        #[test]
        fn ready_with_zero_sample_rate_is_rejected() {
            let frame = TempFile::write("zero-rate", &ready_frame_bytes(0));
            let state = TestState::new();
            let child = spawn_sh("cat \"$1\"", Some(&frame.0));
            let err = run_receiver_with_child(state.session(), child, fast_timeouts()).unwrap_err();
            assert!(format!("{err:#}").contains("invalid sample rate"), "{err:#}");
        }

        #[test]
        fn early_error_kills_and_reaps_helper_on_drop() {
            let state = TestState::new();
            // Spawn without piped stdout so run_receiver_with_child returns early before stop().
            let child = Process::new("sh")
                .args(["-c", "exec sleep 60"])
                .stdin(Stdio::piped())
                .stdout(Stdio::null())
                .stderr(Stdio::null())
                .spawn()
                .unwrap();
            let started = Instant::now();
            let err = run_receiver_with_child(state.session(), child, fast_timeouts()).unwrap_err();
            assert!(format!("{err:#}").contains("no output"), "{err:#}");
            assert!(started.elapsed() < Duration::from_secs(2), "Drop cleanup hung: {:?}", started.elapsed());
        }
    }
}
