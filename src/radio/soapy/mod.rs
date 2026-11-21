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
    let stdout = child.stdout.take().ok_or("no output from KAT's SoapySDR helper")?;
    let (tx, rx) = mpsc::channel();
    thread::spawn(move || {
        let lines: Vec<String> = BufReader::new(stdout).lines().map_while(Result::ok).collect();
        let _ = tx.send(lines);
    });
    let lines = match rx.recv_timeout(FIND_TIMEOUT) {
        Ok(lines) => lines,
        Err(_) => {
            let _ = child.kill();
            let _ = child.wait();
            return Err(format!("listing its radios took over {} s", FIND_TIMEOUT.as_secs()));
        }
    };
    let status = child.wait();
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
        self.rx_thread = Some(thread::spawn(move || {
            if let Err(e) = run_receiver(&live, event_tx.clone(), &gain, &rate, &device) {
                live.receiving.store(false, Ordering::SeqCst);
                live.end_recording_with_rx();
                live.rssi.store(0, Ordering::Relaxed);
                tracing::warn!("SoapySDR receiver stopped: {e:#}");
                let _ = event_tx.send(RadioEvent::Error(format!("{e:#}")));
            }
        }));
        tracing::info!("Started SoapySDR receiving at {frequency} Hz");
        Ok(())
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
        if let Ok(mut g) = self.gain.lock() {
            *g = Some(f64::from(gain));
        }
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

/// Ask the helper to stop, then make sure it does. The caller first drops its end of the
/// helper's stdout: a helper blocked on writing a frame KAT no longer reads then fails the write
/// and ends, instead of waiting there to be killed.
fn end_helper(child: &mut Child, stdin: &mut Option<std::process::ChildStdin>) {
    if let Some(mut s) = stdin.take() {
        let _ = s.write_all(Command::Quit.line().as_bytes());
    }
    let deadline = Instant::now() + Duration::from_secs(3);
    while Instant::now() < deadline {
        if let Ok(Some(_)) = child.try_wait() {
            return;
        }
        thread::sleep(Duration::from_millis(20));
    }
    tracing::warn!("SoapySDR helper did not stop: killed");
    let _ = child.kill();
    let _ = child.wait();
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
    let mut applied = *gain.lock().unwrap();
    let mut child = spawn_helper(&protocol::rx_args(target, applied, device), Stdio::piped())
        .context("Cannot start KAT's SoapySDR helper")?;
    let mut stdin = child.stdin.take();
    let mut stdout = BufReader::with_capacity(1 << 20, child.stdout.take().context("no output from the SoapySDR helper")?);
    let send = |stdin: &mut Option<std::process::ChildStdin>, c: Command| {
        if let Some(s) = stdin.as_mut() {
            let _ = s.write_all(c.line().as_bytes()).and_then(|_| s.flush());
        }
    };

    // Ready, or why not.
    let ready: Ready = loop {
        if !live.receiving.load(Ordering::SeqCst) {
            drop(stdout);
            end_helper(&mut child, &mut stdin);
            return Ok(());
        }
        match protocol::read_frame(&mut stdout) {
            Ok(Some((READY, p))) => match serde_json::from_slice(&p) {
                Ok(ready) => break ready,
                Err(e) => {
                    drop(stdout);
                    end_helper(&mut child, &mut stdin);
                    anyhow::bail!("KAT's SoapySDR helper said something KAT cannot read ({e})");
                }
            },
            Ok(Some((ERROR, p))) => {
                drop(stdout);
                end_helper(&mut child, &mut stdin);
                anyhow::bail!("SoapySDR: {}", String::from_utf8_lossy(&p));
            }
            Ok(Some(_)) => continue,
            Ok(None) | Err(_) => {
                drop(stdout);
                end_helper(&mut child, &mut stdin);
                anyhow::bail!("KAT's SoapySDR helper ended before the radio streamed (kat.log has why)");
            }
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
    let mut failed = None;
    while live.receiving.load(Ordering::SeqCst) {
        let req = live.hop_request.swap(0, Ordering::SeqCst);
        if req != 0 {
            send(&mut stdin, Command::Tune(req));
        }
        if gain_check.elapsed() >= Duration::from_millis(100) {
            gain_check = Instant::now();
            let g = *gain.lock().unwrap();
            if g != applied {
                send(&mut stdin, Command::Gain(g));
                applied = g;
                live.floor_reset.store(true, Ordering::SeqCst);
            }
        }
        match protocol::read_frame(&mut stdout) {
            Ok(Some((IQ, p))) if p.len() >= 4 => {
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
            Ok(Some((TUNE_FAILED, p))) => {
                let hz = p.get(..4).map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]));
                tracing::warn!("SoapySDR retune to {hz:?} Hz failed");
                live.hop_failed.store(true, Ordering::SeqCst);
            }
            Ok(Some((ERROR, p))) => {
                failed = Some(String::from_utf8_lossy(&p).into_owned());
                break;
            }
            Ok(Some(_)) => {}
            Ok(None) => {
                failed = Some("KAT's SoapySDR helper ended (kat.log has why)".to_string());
                break;
            }
            Err(e) => {
                failed = Some(format!("reading from KAT's SoapySDR helper failed ({e})"));
                break;
            }
        }
    }
    drop(stdout);
    end_helper(&mut child, &mut stdin);
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
}
