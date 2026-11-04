//! HackRF device control.
//!
//! This module provides a high-level interface for controlling HackRF devices
//! using the `libhackrf` crate. Falls back to demo mode at runtime if no
//! HackRF hardware is detected.

use anyhow::Result;
use std::sync::mpsc::Sender;
use std::sync::{
    atomic::{AtomicBool, AtomicU32, Ordering},
    Arc, Mutex,
};
use std::thread::{self, JoinHandle};

use crate::app::RadioEvent;

use super::demodulator::LevelDuration;
use super::dsp::IqFormat;
use super::hopper::HopPlan;
use super::recorder::{RecordingStatus, RecordingSummary};
use super::rx::{lo_for, LiveRx, LiveShared};

/// Sample rate for HackRF (2 MHz is good for keyfob signals)
const SAMPLE_RATE: u32 = 2_000_000;

/// Shared gain/amp settings that can be updated while receiving
#[derive(Debug, Clone, Copy, PartialEq)]
struct GainSettings {
    lna_gain: u32,
    vga_gain: u32,
    amp_enabled: bool,
}

impl Default for GainSettings {
    fn default() -> Self {
        Self {
            lna_gain: 24,
            vga_gain: 20,
            amp_enabled: false,
        }
    }
}

/// HackRF controller for receiving and transmitting signals
pub struct HackRfController {
    /// Event sender for notifying the app
    event_tx: Sender<RadioEvent>,
    /// Whether we're currently receiving
    receiving: Arc<AtomicBool>,
    /// Receiver thread handle
    rx_thread: Option<JoinHandle<()>>,
    /// Current frequency
    frequency: Arc<Mutex<u32>>,
    /// State shared with the receive thread: pipeline control, hopping, recording (src/radio/rx.rs)
    live: Arc<LiveShared>,
    /// Whether HackRF is available
    hackrf_available: bool,
    /// Shared gain settings (read by receiver thread)
    gain_settings: Arc<Mutex<GainSettings>>,
    /// RSSI (f32 bits) written by RX callback, read by UI - never blocks
    rssi_value: Arc<AtomicU32>,
}

impl HackRfController {
    /// Create a new HackRF controller
    pub fn new(event_tx: Sender<RadioEvent>) -> Result<Self> {
        let receiving = Arc::new(AtomicBool::new(false));
        let rssi_value = Arc::new(AtomicU32::new(0));
        let live = Arc::new(LiveShared::new(receiving.clone(), rssi_value.clone(), 433_920_000));

        // Check if HackRF is available
        let hackrf_available = check_hackrf_available();

        if hackrf_available {
            tracing::info!("HackRF device detected");
        } else {
            tracing::warn!("HackRF not detected - running in demo mode");
        }

        Ok(Self {
            event_tx,
            receiving,
            rx_thread: None,
            frequency: Arc::new(Mutex::new(433_920_000)),
            live,
            hackrf_available,
            gain_settings: Arc::new(Mutex::new(GainSettings::default())),
            rssi_value,
        })
    }

    /// Shared atomic for RSSI (f32::to_bits); UI reads so callback never blocks on channel.
    pub fn rssi_source(&self) -> Arc<AtomicU32> {
        self.rssi_value.clone()
    }

    /// Check if HackRF is available
    #[allow(dead_code)]
    pub fn is_available(&self) -> bool {
        self.hackrf_available
    }

    /// HackRF supports transmit.
    pub fn supports_tx(&self) -> bool {
        true
    }

    /// Start receiving at the specified frequency
    pub fn start_receiving(&mut self, frequency: u32) -> Result<()> {
        if self.receiving.load(Ordering::SeqCst) {
            return Ok(());
        }

        *self.frequency.lock().unwrap() = frequency;
        self.live.home.store(frequency, Ordering::SeqCst);
        self.receiving.store(true, Ordering::SeqCst);

        let receiving = self.receiving.clone();
        let event_tx = self.event_tx.clone();
        let freq = self.frequency.clone();
        let live = self.live.clone();
        let hackrf_available = self.hackrf_available;
        let gain_settings = self.gain_settings.clone();

        self.rx_thread = Some(thread::spawn(move || {
            if hackrf_available {
                if let Err(e) = run_receiver_hackrf(live.clone(), event_tx.clone(), gain_settings) {
                    live.receiving.store(false, Ordering::SeqCst);
                    let _ = event_tx.send(RadioEvent::Error(format!("Receiver error: {}", e)));
                }
            } else {
                run_demo_receiver(receiving, event_tx, freq);
            }
        }));

        tracing::info!("Started receiving at {} Hz", frequency);
        Ok(())
    }

    /// Stop receiving
    pub fn stop_receiving(&mut self) -> Result<()> {
        self.receiving.store(false, Ordering::SeqCst);

        if let Some(handle) = self.rx_thread.take() {
            let _ = handle.join();
        }
        // A recording belongs to one RX session (its file has one centre frequency).
        self.live.end_recording_with_rx();

        tracing::info!("Stopped receiving");
        Ok(())
    }

    /// Set the receive frequency
    pub fn set_frequency(&mut self, frequency: u32) -> Result<()> {
        *self.frequency.lock().unwrap() = frequency;
        self.live.home.store(frequency, Ordering::SeqCst);
        self.live.target.store(frequency, Ordering::SeqCst);
        tracing::info!("Set frequency to {} Hz", frequency);
        Ok(())
    }

    /// Transmit a signal: OOK or 2-FSK per `settings.modulation`, at `settings`' TX power.
    pub fn transmit(
        &mut self,
        signal: &[LevelDuration],
        frequency: u32,
        settings: &super::tx::TxSettings,
    ) -> Result<()> {
        if !self.hackrf_available {
            tracing::warn!("HackRF not available - simulating transmission");
            return Ok(());
        }

        // Stop receiving first if we are
        let was_receiving = self.receiving.load(Ordering::SeqCst);
        if was_receiving {
            self.stop_receiving()?;
        }

        tracing::info!(
            "Transmitting {} level/duration pairs at {} Hz ({}, TXVGA {} dB, amp {})",
            signal.len(),
            frequency,
            settings.modulation,
            settings.vga_gain,
            if settings.amp { "on" } else { "off" }
        );

        let result = transmit_signal_hackrf(signal, frequency, settings);

        // Resume receiving if we were before (also after a failed transmission)
        if was_receiving {
            let freq = *self.frequency.lock().unwrap();
            self.start_receiving(freq)?;
        }

        result
    }

    /// Set LNA gain (0-40 dB, 8 dB steps)
    pub fn set_lna_gain(&mut self, gain: u32) -> Result<()> {
        tracing::info!("Set LNA gain to {} dB", gain);
        if let Ok(mut settings) = self.gain_settings.lock() {
            settings.lna_gain = gain;
        }
        Ok(())
    }

    /// Set VGA gain (0-62 dB, 2 dB steps)
    pub fn set_vga_gain(&mut self, gain: u32) -> Result<()> {
        tracing::info!("Set VGA gain to {} dB", gain);
        if let Ok(mut settings) = self.gain_settings.lock() {
            settings.vga_gain = gain;
        }
        Ok(())
    }

    /// Enable/disable the RF amplifier
    pub fn set_amp_enable(&mut self, enabled: bool) -> Result<()> {
        tracing::info!("Set amp enable to {}", enabled);
        if let Ok(mut settings) = self.gain_settings.lock() {
            settings.amp_enabled = enabled;
        }
        Ok(())
    }
}

impl Drop for HackRfController {
    fn drop(&mut self) {
        self.receiving.store(false, Ordering::SeqCst);
        if let Some(handle) = self.rx_thread.take() {
            let _ = handle.join();
        }
    }
}

/// Check if HackRF is available
fn check_hackrf_available() -> bool {
    // Try to open a HackRF device
    match libhackrf::HackRf::open() {
        Ok(_) => {
            tracing::debug!("HackRF opened successfully");
            true
        }
        Err(e) => {
            tracing::debug!("HackRF not available: {:?}", e);
            // Fallback: check via hackrf_info command
            match std::process::Command::new("hackrf_info")
                .stdout(std::process::Stdio::null())
                .stderr(std::process::Stdio::null())
                .status()
            {
                Ok(status) => status.success(),
                Err(_) => false,
            }
        }
    }
}

/// Run a demo receiver (no actual HackRF)
fn run_demo_receiver(
    receiving: Arc<AtomicBool>,
    _event_tx: Sender<RadioEvent>,
    _frequency: Arc<Mutex<u32>>,
) {
    tracing::info!("Demo receiver thread started (no HackRF)");

    while receiving.load(Ordering::SeqCst) {
        std::thread::sleep(std::time::Duration::from_millis(100));
    }

    tracing::info!("Demo receiver thread stopped");
}

/// HackRF retune bookkeeping between the control thread (which calls `set_freq` while streaming)
/// and the RX callback (which must know from which sample on the data is from the new frequency).
struct RetuneSync {
    /// (samples delivered so far, when the last buffer arrived)
    clock: Mutex<(u64, std::time::Instant)>,
    /// Retune done by the control thread: (new target, estimated first sample at the new frequency)
    marker: Mutex<Option<(u32, u64)>>,
}

/// Samples dropped before / after the estimated retune point. The boundary is estimated from the
/// arrival time of the last buffer (USB + scheduling latency of a few ms), the PLL settles in < 1 ms.
const RETUNE_GUARD_PRE_US: u64 = 2_000;
const RETUNE_GUARD_POST_US: u64 = 10_000;

/// Where to split a buffer covering samples `[start, start + len)` for a retune whose first
/// new-frequency sample is estimated at `boundary`: process `split` samples on the old frequency,
/// then retune the pipeline and drop `blank` samples. `None` if the boundary is past this buffer.
fn retune_split(start: u64, len: u64, boundary: u64) -> Option<(u64, u64)> {
    if boundary >= start + len {
        return None;
    }
    let per_us = SAMPLE_RATE as u64 / 1_000_000;
    let cut = boundary.saturating_sub(RETUNE_GUARD_PRE_US * per_us).max(start);
    let blank = (boundary + RETUNE_GUARD_POST_US * per_us).saturating_sub(cut);
    Some((cut - start, blank))
}

/// What the RX callback hands to the processing thread, in stream order.
enum RxMsg {
    /// Raw `.cs8` samples on the current frequency.
    Samples(Vec<u8>),
    /// From here on the samples are from `target`; drop `blank` of them first.
    Retuned { target: u32, blank: u64 },
}

/// Buffers queued between the RX callback and the processing thread (32 × 65 ms ≈ 2 s).
const RX_QUEUE_BUFFERS: usize = 32;

/// Shared state for RX callback (libhackrf requires fn pointers, not closures)
struct RxState {
    live: Arc<LiveShared>,
    /// To the processing thread (pipeline + hopper), which owns the `LiveRx`.
    queue: std::sync::mpsc::SyncSender<RxMsg>,
    retune: Arc<RetuneSync>,
}

impl RxState {
    /// Queue a buffer, split at a retune point `(target, samples before it, samples to blank)`.
    fn dispatch(&self, bytes: &[u8], split: Option<(u32, u64, u64)>) {
        match split {
            Some((target, split, blank)) => {
                let split = ((split * 2) as usize).min(bytes.len());
                self.queue_samples(&bytes[..split]);
                // Never lost (captures would carry the wrong frequency): wait for room if needed.
                let _ = self.queue.send(RxMsg::Retuned { target, blank });
                self.queue_samples(&bytes[split..]);
            }
            None => self.queue_samples(bytes),
        }
    }

    /// Queue samples without ever blocking the USB callback: if processing is 2 s behind, drop.
    fn queue_samples(&self, bytes: &[u8]) {
        if bytes.is_empty() {
            return;
        }
        if let Err(std::sync::mpsc::TrySendError::Full(_)) = self.queue.try_send(RxMsg::Samples(bytes.to_vec())) {
            tracing::warn!("HackRF RX: processing fell behind, {} samples dropped", bytes.len() / 2);
        }
    }
}

/// RX callback: record the raw buffer if recording, and queue it for the processing thread (which
/// runs the receive pipeline, emits captures and drives hopping), split at a retune point. The
/// pipeline doesn't run here: slicing a long burst can take longer than libhackrf's transfers
/// (4 × 65 ms) can wait.
fn rx_callback(
    _hackrf: &libhackrf::HackRf,
    buffer: &[num_complex::Complex<i8>],
    user_data: &dyn std::any::Any,
) {
    let arrived = std::time::Instant::now();
    let state = match user_data.downcast_ref::<RxState>() {
        Some(s) => s,
        None => return,
    };
    if state.live.receiving.load(Ordering::SeqCst) {
        // SAFETY: `Complex<i8>` is `#[repr(C)] { re: i8, im: i8 }` (no padding), so the buffer is
        // `2 * len` interleaved I/Q bytes, exactly what hackrf_transfer writes to a .cs8 file.
        let bytes = unsafe { std::slice::from_raw_parts(buffer.as_ptr() as *const u8, buffer.len() * 2) };
        state.live.record(bytes);
        let start = state.retune.clock.lock().map(|c| c.0).unwrap_or(0);
        let end = start + buffer.len() as u64;
        let split = match state.retune.marker.lock() {
            Ok(mut m) => match *m {
                Some((target, boundary)) => {
                    retune_split(start, buffer.len() as u64, boundary).map(|(split, blank)| {
                        *m = None;
                        (target, split, blank)
                    })
                }
                None => None,
            },
            Err(_) => None,
        };
        state.dispatch(bytes, split);
        if let Ok(mut c) = state.retune.clock.lock() {
            *c = (end, arrived);
        }
    }
}

/// Run the receiver loop with actual HackRF using libhackrf: configure, stream, and apply gain
/// changes and hop retunes while streaming.
fn run_receiver_hackrf(
    live: Arc<LiveShared>,
    event_tx: Sender<RadioEvent>,
    gain_settings: Arc<Mutex<GainSettings>>,
) -> Result<()> {
    use anyhow::Context;

    tracing::info!("HackRF receiver thread starting...");

    let hackrf = libhackrf::HackRf::open()
        .context("Failed to open HackRF device")?;

    let freq = live.start_frequency();
    live.target.store(freq, Ordering::SeqCst);
    let initial_gains = *gain_settings.lock().unwrap();
    tracing::info!(
        "Configuring HackRF: target={} Hz (LO {} Hz), sample_rate={} Hz, LNA={} dB, VGA={} dB, AMP={}",
        freq, lo_for(freq, SAMPLE_RATE), SAMPLE_RATE, initial_gains.lna_gain, initial_gains.vga_gain, initial_gains.amp_enabled
    );

    hackrf.set_sample_rate(SAMPLE_RATE)
        .context("Failed to set sample rate")?;
    // libhackrf already picks the baseband filter for the rate (1.75 MHz, the narrowest, which
    // still passes the ±0.9 MHz the detector searches); set it explicitly, but don't fail RX over it.
    if let Err(e) = hackrf.set_baseband_filter_bandwidth(1_750_000) {
        tracing::warn!("Failed to set baseband filter to 1.75 MHz: {:?}", e);
    }
    hackrf.set_freq(lo_for(freq, SAMPLE_RATE) as u64)
        .context("Failed to set frequency")?;
    hackrf.set_lna_gain(initial_gains.lna_gain)
        .context("Failed to set LNA gain")?;
    hackrf.set_rxvga_gain(initial_gains.vga_gain)
        .context("Failed to set RXVGA gain")?;
    hackrf.set_amp_enable(initial_gains.amp_enabled)
        .context("Failed to enable amp")?;

    tracing::info!("HackRF configured, starting RX (receive pipeline, target at +fs/8)...");

    let retune = Arc::new(RetuneSync {
        clock: Mutex::new((0, std::time::Instant::now())),
        marker: Mutex::new(None),
    });
    let (queue, queued) = std::sync::mpsc::sync_channel::<RxMsg>(RX_QUEUE_BUFFERS);
    let state = RxState { live: live.clone(), queue, retune: retune.clone() };

    // Start receiving
    hackrf.start_rx(rx_callback, state)
        .context("Failed to start RX")?;

    // Processing thread: the pipeline and hopper, fed in stream order. It ends (closing any open
    // burst) once `stop_rx` has dropped the callback state and the queue is drained.
    let proc_live = live.clone();
    let processor = thread::Builder::new().name("kat-hackrf-rx".to_string()).spawn(move || {
        let mut rx = LiveRx::new(SAMPLE_RATE, IqFormat::Cs8, freq, event_tx);
        for msg in queued {
            match msg {
                RxMsg::Samples(bytes) => rx.process(&bytes, &proc_live),
                RxMsg::Retuned { target, blank } => rx.retuned(target, blank, &proc_live),
            }
        }
        rx.finish();
    });
    let processor = match processor {
        Ok(p) => p,
        Err(e) => {
            let _ = hackrf.stop_rx();
            return Err(e).context("Failed to start the HackRF processing thread");
        }
    };

    // Track applied settings so we can detect changes
    let mut applied = initial_gains;
    let mut last_gain_check = std::time::Instant::now();

    // Until receiving is stopped: retune for hops (every 5 ms), apply gain changes (every 100 ms).
    while live.receiving.load(Ordering::SeqCst) {
        std::thread::sleep(std::time::Duration::from_millis(5));

        let req = live.hop_request.swap(0, Ordering::SeqCst);
        if req != 0 {
            let t = std::time::Instant::now();
            match hackrf.set_freq(lo_for(req, SAMPLE_RATE) as u64) {
                Ok(()) => {
                    let (end, at) = *retune.clock.lock().unwrap();
                    let elapsed = t.saturating_duration_since(at).as_secs_f64();
                    let boundary = end + (elapsed * SAMPLE_RATE as f64) as u64;
                    *retune.marker.lock().unwrap() = Some((req, boundary));
                    tracing::debug!("Retuned to {} Hz (LO {} Hz)", req, lo_for(req, SAMPLE_RATE));
                }
                Err(e) => {
                    tracing::warn!("Failed to retune to {} Hz: {:?}", req, e);
                    live.hop_failed.store(true, Ordering::SeqCst);
                }
            }
        }

        if last_gain_check.elapsed() < std::time::Duration::from_millis(100) {
            continue;
        }
        last_gain_check = std::time::Instant::now();
        let mut changed = false;

        // Check for gain/amp setting changes and apply them live
        if let Ok(current) = gain_settings.lock() {
            if current.lna_gain != applied.lna_gain {
                if let Err(e) = hackrf.set_lna_gain(current.lna_gain) {
                    tracing::warn!("Failed to set LNA gain to {}: {:?}", current.lna_gain, e);
                } else {
                    tracing::info!("Applied LNA gain: {} dB", current.lna_gain);
                    applied.lna_gain = current.lna_gain;
                    changed = true;
                }
            }
            if current.vga_gain != applied.vga_gain {
                if let Err(e) = hackrf.set_rxvga_gain(current.vga_gain) {
                    tracing::warn!("Failed to set VGA gain to {}: {:?}", current.vga_gain, e);
                } else {
                    tracing::info!("Applied VGA gain: {} dB", current.vga_gain);
                    applied.vga_gain = current.vga_gain;
                    changed = true;
                }
            }
            if current.amp_enabled != applied.amp_enabled {
                if let Err(e) = hackrf.set_amp_enable(current.amp_enabled) {
                    tracing::warn!("Failed to set amp to {}: {:?}", current.amp_enabled, e);
                } else {
                    tracing::info!("Applied amp: {}", if current.amp_enabled { "ON" } else { "OFF" });
                    applied.amp_enabled = current.amp_enabled;
                    changed = true;
                }
            }
        }
        if changed {
            // The noise floor moved with the gain: let the squelch re-learn it.
            live.floor_reset.store(true, Ordering::SeqCst);
        }
    }

    // Stop receiving: this drops the callback state, so the processing thread drains the queue,
    // emits a burst that was still open and ends.
    let stopped = hackrf.stop_rx();
    let _ = processor.join();
    stopped.context("Failed to stop RX")?;

    tracing::info!("HackRF receiver thread stopped");
    Ok(())
}

/// Receive-side controls (hopping, recording, retune info) used through `RadioDevice`.
impl HackRfController {
    /// IQ format of this device's raw samples (and of `:record` files).
    pub fn iq_format(&self) -> IqFormat {
        IqFormat::Cs8
    }

    /// Raw sample rate (S/s).
    pub fn sample_rate(&self) -> u32 {
        SAMPLE_RATE
    }

    /// Frequency being received right now (follows hops).
    pub fn rx_frequency(&self) -> u32 {
        self.live.target.load(Ordering::SeqCst)
    }

    /// Install (Some) or remove (None) a hop plan; applied live while receiving.
    pub fn set_hopping(&mut self, plan: Option<HopPlan>) {
        self.live.set_hop_plan(plan);
    }

    /// Current hop plan and the index being received.
    pub fn hop_state(&self) -> Option<(HopPlan, usize)> {
        self.live.hop_plan().map(|p| (p, self.live.hop_index.load(Ordering::SeqCst) as usize))
    }

    /// Hold the current hop frequency for `dwells` dwells (e.g. after a decode).
    pub fn hop_pause(&mut self, dwells: u32) {
        self.live.hop_pause_dwells.store(dwells, Ordering::SeqCst);
    }

    /// Start writing the raw IQ to `path`.
    pub fn start_recording(&mut self, path: std::path::PathBuf) -> Result<()> {
        self.live.start_recording(path)
    }

    pub fn stop_recording(&mut self) -> Option<RecordingSummary> {
        self.live.stop_recording()
    }

    pub fn recording_status(&self) -> Option<RecordingStatus> {
        self.live.recording_status()
    }

    /// A recording that ended because RX stopped (reported once).
    pub fn take_ended_recording(&mut self) -> Option<RecordingSummary> {
        self.live.ended_recording.lock().ok().and_then(|mut g| g.take())
    }
}

/// Shared state for TX callback: the sample stream, also held by the transmitting thread, which
/// watches its progress.
struct TxState {
    stream: Arc<super::tx::TxStream>,
}

/// TX callback function for libhackrf: hand out the next samples (zeros once they run out).
fn tx_callback(
    _hackrf: &libhackrf::HackRf,
    buffer: &mut [num_complex::Complex<i8>],
    user_data: &dyn std::any::Any,
) {
    if let Some(state) = user_data.downcast_ref::<TxState>() {
        state.stream.fill(buffer);
    }
}

/// HackRF baseband filter for TX: 1.75 MHz (±875 kHz) passes the waveform on its 500 kHz digital
/// offset (± at most 47.6 kHz deviation) and attenuates the DAC images 1.5 MHz away.
const TX_BASEBAND_FILTER_HZ: u32 = 1_750_000;
/// After the last sample has left the host, the HackRF still holds up to its 32 KB USB buffer
/// (≈ 8 ms at 2 MS/s); wait this long before stopping.
const TX_DRAIN_MS: u64 = 25;
/// Slack on top of the signal's air time before a transmission counts as stuck.
const TX_TIMEOUT_SLACK_MS: u64 = 2_000;

/// Transmit a signal via HackRF: the LO sits `TX_LO_OFFSET_HZ` below `frequency` and the
/// waveform (OOK or 2-FSK, `generate_tx_iq`) is shifted up digitally, so the LO leakage stays out
/// of the receiver's channel. Returns once the last sample is on air (tracked through the TX
/// callback, with a timeout), not after a fixed sleep.
fn transmit_signal_hackrf(
    signal: &[LevelDuration],
    frequency: u32,
    settings: &super::tx::TxSettings,
) -> Result<()> {
    use super::tx::{
        generate_tx_iq, tx_lo_frequency, TxStream, LIBHACKRF_TRANSFER_COUNT, TX_LO_OFFSET_HZ,
        TX_SAMPLE_RATE,
    };
    use anyhow::Context;
    use std::time::{Duration, Instant};

    let vga_gain = settings.vga_gain.min(47);
    let lo = tx_lo_frequency(frequency);
    tracing::info!(
        "Starting HackRF transmission: carrier {} Hz (LO {} Hz), {}, TXVGA {} dB, amp {}",
        frequency,
        lo,
        settings.modulation,
        vga_gain,
        if settings.amp { "on" } else { "off" }
    );

    // Generate the samples first: nothing is keyed until they are ready.
    let samples = generate_tx_iq(signal, settings.modulation, TX_SAMPLE_RATE, TX_LO_OFFSET_HZ);
    let total = samples.len();
    let stream = Arc::new(TxStream::new(samples));
    tracing::debug!("Generated {} TX samples", total);

    let hackrf = libhackrf::HackRf::open().context("Failed to open HackRF device")?;
    hackrf.set_sample_rate(TX_SAMPLE_RATE).context("Failed to set sample rate")?;
    hackrf
        .set_baseband_filter_bandwidth(TX_BASEBAND_FILTER_HZ)
        .context("Failed to set baseband filter")?;
    hackrf.set_freq(lo).context("Failed to set frequency")?;
    hackrf.set_txvga_gain(vga_gain).context("Failed to set TXVGA gain")?;
    hackrf.set_amp_enable(settings.amp).context("Failed to set amp")?;

    if let Err(e) = hackrf.start_tx(tx_callback, TxState { stream: Arc::clone(&stream) }) {
        let _ = hackrf.stop_tx(); // frees the callback context the wrapper already registered
        return Err(anyhow::Error::new(e).context("Failed to start TX"));
    }

    // Wait until the buffer holding the last sample has left the host (callback progress), then
    // for the device's own buffer to drain. The timeout only guards against a stalled stream.
    let air_ms = (total as u64 * 1000) / u64::from(TX_SAMPLE_RATE);
    let in_flight_ms = (LIBHACKRF_TRANSFER_COUNT as u64 * 131_072 * 1000) / u64::from(TX_SAMPLE_RATE);
    let deadline = Instant::now() + Duration::from_millis(air_ms + in_flight_ms + TX_TIMEOUT_SLACK_MS);
    while !stream.is_complete() && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(5));
    }
    let completed = stream.is_complete();
    if completed {
        std::thread::sleep(Duration::from_millis(TX_DRAIN_MS));
    }

    // libhackrf cancels the transfers and waits for them, so no TX callback runs after this; the
    // (vendored, fixed) wrapper then frees the callback context.
    let stopped = hackrf.stop_tx();
    let sent = stream.sent();
    stopped.context("Failed to stop TX")?;

    if !completed {
        if sent < total {
            anyhow::bail!("Transmission stalled: {sent} of {total} samples sent before the timeout");
        }
        tracing::warn!("TX callbacks stopped before the final buffer was confirmed; all samples were queued");
    }
    tracing::info!("Transmission complete ({} samples, {} ms on air)", total, air_ms);
    Ok(())
}

#[cfg(test)]
mod rx_tests {
    use super::*;

    #[test]
    fn retune_split_points() {
        // 2 MS/s: 2 ms before = 4000 samples, 10 ms after = 20000 samples.
        assert_eq!(retune_split(100_000, 131_072, 150_000), Some((46_000, 24_000)));
        // Boundary estimated just before this buffer: retune at its start, blank the rest.
        assert_eq!(retune_split(100_000, 131_072, 99_000), Some((0, 19_000)));
        // Boundary in a later buffer: nothing yet.
        assert_eq!(retune_split(100_000, 131_072, 231_072), None);
    }

    /// The callback queues samples and retune points in stream order, and a full queue drops
    /// samples instead of blocking the USB thread (but never a retune point).
    #[test]
    fn callback_queues_in_stream_order() {
        let (queue, rx) = std::sync::mpsc::sync_channel(3);
        let live = Arc::new(LiveShared::new(Arc::new(AtomicBool::new(true)), Arc::new(AtomicU32::new(0)), 433_920_000));
        let retune = Arc::new(RetuneSync { clock: Mutex::new((0, std::time::Instant::now())), marker: Mutex::new(None) });
        let state = RxState { live, queue, retune };
        state.dispatch(&[1, 2, 3, 4, 5, 6], Some((868_350_000, 1, 500)));
        let got: Vec<String> = rx
            .try_iter()
            .map(|m| match m {
                RxMsg::Samples(b) => format!("{b:?}"),
                RxMsg::Retuned { target, blank } => format!("{target}/{blank}"),
            })
            .collect();
        assert_eq!(got, ["[1, 2]", "868350000/500", "[3, 4, 5, 6]"]);
        for b in [[1u8, 2], [3, 4], [5, 6], [7, 8]] {
            state.dispatch(&b, None); // the 4th finds the queue (3) full: dropped, doesn't block
        }
        assert_eq!(rx.try_iter().count(), 3);
    }
}
