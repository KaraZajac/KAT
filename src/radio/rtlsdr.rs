//! RTL-SDR device control (receive-only).
//!
//! Uses the same receive pipeline as the HackRF path (src/radio/pipeline.rs, offset tuning: the LO
//! sits fs/8 below the target). A reader thread owns the device: it reads, hands the raw buffers to
//! the recorder and to a processing thread, and retunes between reads when the hopper asks, so every
//! buffer is tagged with the exact frequency it was received on. Transmit is not supported.

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

use rtl_sdr_rs::{DeviceId, RtlSdr, TunerGain};

/// Sample rate (2 MHz, same as HackRF path for keyfob signals)
const SAMPLE_RATE: u32 = 2_000_000;

/// Tuner gain setting for RTL-SDR (manual gain in 0.1 dB units; None = auto)
#[derive(Debug, Clone, Copy, Default)]
struct TunerGainSetting {
    /// Manual gain in tenths of dB, or None for auto
    gain_tenths_db: Option<i32>,
}

/// RTL-SDR controller: receive-only, no transmit.
pub struct RtlSdrController {
    event_tx: Sender<RadioEvent>,
    receiving: Arc<AtomicBool>,
    rx_thread: Option<JoinHandle<()>>,
    frequency: Arc<Mutex<u32>>,
    /// State shared with the receive threads: pipeline control, hopping, recording
    live: Arc<LiveShared>,
    rtlsdr_available: bool,
    /// Why no RTL-SDR could be opened at startup (for the no-device popup)
    open_error: Option<String>,
    gain_settings: Arc<Mutex<TunerGainSetting>>,
    /// RSSI (f32 bits) written by RX thread, read by UI - never blocks
    rssi_value: Arc<AtomicU32>,
}

impl RtlSdrController {
    /// Create a new RTL-SDR controller (receive-only).
    pub fn new(event_tx: Sender<RadioEvent>) -> Result<Self> {
        let receiving = Arc::new(AtomicBool::new(false));
        let rssi_value = Arc::new(AtomicU32::new(0));
        let live = Arc::new(LiveShared::new(receiving.clone(), rssi_value.clone(), 433_920_000));

        let probe = check_rtlsdr_available();
        let rtlsdr_available = probe.is_ok();

        if rtlsdr_available {
            tracing::info!("RTL-SDR device detected (receive-only)");
        } else {
            tracing::debug!("RTL-SDR not detected");
        }

        Ok(Self {
            event_tx,
            receiving,
            rx_thread: None,
            frequency: Arc::new(Mutex::new(433_920_000)),
            live,
            rtlsdr_available,
            open_error: probe.err(),
            gain_settings: Arc::new(Mutex::new(TunerGainSetting::default())),
            rssi_value,
        })
    }

    /// Why no RTL-SDR could be opened at startup, in words (None when one was found).
    pub fn open_error(&self) -> Option<&str> {
        self.open_error.as_deref()
    }

    /// Shared atomic for RSSI (f32::to_bits); UI reads so RX thread never blocks on channel.
    pub fn rssi_source(&self) -> Arc<AtomicU32> {
        self.rssi_value.clone()
    }

    /// Returns true if an RTL-SDR device was found.
    pub fn is_available(&self) -> bool {
        self.rtlsdr_available
    }

    /// RTL-SDR is receive-only; transmit is never supported.
    pub fn supports_tx(&self) -> bool {
        false
    }

    /// Start receiving at the specified frequency.
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
        let rtlsdr_available = self.rtlsdr_available;
        let gain_settings = self.gain_settings.clone();

        self.rx_thread = Some(thread::spawn(move || {
            if rtlsdr_available {
                if let Err(e) = run_receiver_rtlsdr(live.clone(), event_tx.clone(), gain_settings) {
                    // RX ended on its own (open failure, unplug): its recording ends and the meter
                    // drops; the app goes back to idle.
                    live.receiving.store(false, Ordering::SeqCst);
                    live.end_recording_with_rx();
                    live.rssi.store(0, Ordering::Relaxed);
                    tracing::warn!("RTL-SDR receiver stopped: {e:#}");
                    let _ = event_tx.send(RadioEvent::Error(format!("{e:#}")));
                }
            } else {
                run_demo_receiver(receiving, event_tx, freq);
            }
        }));

        tracing::info!("Started RTL-SDR receiving at {} Hz", frequency);
        Ok(())
    }

    /// Stop receiving.
    pub fn stop_receiving(&mut self) -> Result<()> {
        self.receiving.store(false, Ordering::SeqCst);

        if let Some(handle) = self.rx_thread.take() {
            let _ = handle.join();
        }
        // Nothing is received any more: the meter must not freeze at the last level (f32 0.0).
        self.rssi_value.store(0, Ordering::Relaxed);
        self.live.end_recording_with_rx();

        tracing::info!("Stopped RTL-SDR receiving");
        Ok(())
    }

    /// Set the receive frequency.
    pub fn set_frequency(&mut self, frequency: u32) -> Result<()> {
        *self.frequency.lock().unwrap() = frequency;
        self.live.home.store(frequency, Ordering::SeqCst);
        self.live.target.store(frequency, Ordering::SeqCst);
        tracing::info!("Set RTL-SDR frequency to {} Hz", frequency);
        Ok(())
    }

    /// No-op: RTL-SDR cannot transmit.
    pub fn transmit(&mut self, _signal: &[LevelDuration], _frequency: u32) -> Result<()> {
        tracing::warn!("Transmit not available – RTL-SDR is receive-only");
        Ok(())
    }

    /// Set tuner gain. For RTL-SDR we use a single gain; map LNA-style (0–40 dB) to tuner gain.
    /// Pass None for auto gain.
    pub fn set_tuner_gain_tenths_db(&mut self, gain_tenths_db: Option<i32>) -> Result<()> {
        tracing::info!(
            "Set RTL-SDR tuner gain to {}",
            gain_tenths_db
                .map(|g| format!("{} dB", g as f32 / 10.0))
                .unwrap_or_else(|| "auto".to_string())
        );
        if let Ok(mut settings) = self.gain_settings.lock() {
            settings.gain_tenths_db = gain_tenths_db;
        }
        Ok(())
    }

    /// Set gain from a 0–40 dB value (e.g. LNA-style); stored as tenths of dB for RTL-SDR.
    pub fn set_lna_gain(&mut self, gain: u32) -> Result<()> {
        self.set_tuner_gain_tenths_db(Some((gain * 10) as i32))
    }

    /// RTL-SDR has a single tuner gain; VGA change maps to same tuner gain.
    pub fn set_vga_gain(&mut self, gain: u32) -> Result<()> {
        self.set_tuner_gain_tenths_db(Some((gain * 10) as i32))
    }

    /// RTL-SDR has no separate amp enable; no-op for UI compatibility.
    pub fn set_amp_enable(&mut self, _enabled: bool) -> Result<()> {
        Ok(())
    }
}

impl Drop for RtlSdrController {
    fn drop(&mut self) {
        self.receiving.store(false, Ordering::SeqCst);
        if let Some(handle) = self.rx_thread.take() {
            let _ = handle.join();
        }
    }
}

/// Err(why not, in words) when no RTL-SDR can be opened.
fn check_rtlsdr_available() -> Result<(), String> {
    match RtlSdr::open(DeviceId::Index(0)) {
        Ok(mut dev) => {
            if let Err(e) = dev.close() {
                tracing::debug!("RTL-SDR close after probe: {:?}", e);
            }
            Ok(())
        }
        Err(e) => {
            // Logged at warn (the default `kat=info` filter drops debug), so kat.log tells a
            // permission problem or the DVB-T driver from a missing dongle.
            tracing::warn!("RTL-SDR not available: {:?}", e);
            Err(describe_open_error(&e.to_string()))
        }
    }
}

/// rtl-sdr-rs's open error in words: "No device found at index 0", "Error: Access" (libusb may
/// not open it) or "Resource busy" (claiming it failed: the kernel's DVB-T driver has it).
fn describe_open_error(e: &str) -> String {
    if e.starts_with("No device found") {
        "not found".to_string()
    } else if e.contains("Access") || e.contains("NotSupported") {
        // NotSupported: Windows' libusb, for a dongle without a WinUSB driver.
        format!("found but not accessible: {}", super::USB_ACCESS_HINT)
    } else if (e.contains("busy") || e.contains("Busy")) && cfg!(windows) {
        "found but busy: another program has it".to_string()
    } else if e.contains("busy") || e.contains("Busy") {
        "found but busy: the DVB-T driver (sudo rmmod dvb_usb_rtl28xxu) or another program has it".to_string()
    } else {
        format!("cannot be opened ({e})")
    }
}

fn run_demo_receiver(
    receiving: Arc<AtomicBool>,
    _event_tx: Sender<RadioEvent>,
    _frequency: Arc<Mutex<u32>>,
) {
    tracing::info!("Demo receiver thread started (no RTL-SDR)");

    while receiving.load(Ordering::SeqCst) {
        std::thread::sleep(std::time::Duration::from_millis(100));
    }

    tracing::info!("Demo receiver thread stopped");
}

/// Bytes per read: 64 KiB = 16 ms at 2 MS/s (fine-grained hop timing; the reader thread does nothing
/// else, so short reads don't lose samples).
const READ_LEN: usize = 65_536;
/// Bytes read and discarded right after a retune (PLL settling, ~4 ms at 2 MS/s).
const RETUNE_FLUSH_LEN: usize = 16_384;

/// Run the receiver with an RTL-SDR device: this thread reads (and retunes between reads); a
/// processing thread runs the pipeline on the tagged buffers.
fn run_receiver_rtlsdr(
    live: Arc<LiveShared>,
    event_tx: Sender<RadioEvent>,
    gain_settings: Arc<Mutex<TunerGainSetting>>,
) -> Result<()> {
    use anyhow::Context;

    tracing::info!("RTL-SDR receiver thread starting...");

    let mut sdr = RtlSdr::open(DeviceId::Index(0)).context("Failed to open RTL-SDR device")?;

    let freq = live.start_frequency();
    live.target.store(freq, Ordering::SeqCst);
    let initial_gain = *gain_settings.lock().unwrap();

    sdr.reset_buffer().context("Failed to reset RTL-SDR buffer")?;
    sdr.set_center_freq(lo_for(freq, SAMPLE_RATE)).context("Failed to set RTL-SDR frequency")?;
    sdr.set_sample_rate(SAMPLE_RATE).context("Failed to set RTL-SDR sample rate")?;
    sdr.set_bias_tee(false).context("Failed to set bias-tee")?;

    let apply_gain = |sdr: &mut RtlSdr, g: TunerGainSetting| -> Result<()> {
        match g.gain_tenths_db {
            Some(tenths) => sdr.set_tuner_gain(TunerGain::Manual(tenths)).context("Failed to set RTL-SDR gain"),
            None => sdr.set_tuner_gain(TunerGain::Auto).context("Failed to set RTL-SDR gain"),
        }
    };
    apply_gain(&mut sdr, initial_gain)?;
    let mut applied = initial_gain.gain_tenths_db;

    tracing::info!(
        "RTL-SDR configured: target={} Hz (LO {} Hz), sample_rate={} Hz",
        freq, lo_for(freq, SAMPLE_RATE), SAMPLE_RATE
    );

    // Processing thread: (raw buffer, frequency it was received on, samples to blank first).
    let (buf_tx, buf_rx) = std::sync::mpsc::sync_channel::<(Vec<u8>, u32, u64)>(32);
    let proc_live = live.clone();
    let processor = thread::Builder::new().name("kat-rtl-rx".to_string()).spawn(move || {
        let mut rx = LiveRx::new(SAMPLE_RATE, IqFormat::Cu8, freq, event_tx);
        let mut current = freq;
        for (buf, tag, blank) in buf_rx {
            if tag != current {
                rx.retuned(tag, blank, &proc_live);
                current = tag;
            }
            rx.process(&buf, &proc_live);
        }
        rx.finish();
    })?;

    let mut tag = freq;
    let mut blank = 0u64;
    let mut gain_check = std::time::Instant::now();
    // A failed read ends RX, as in rtl_sdr's sync loop ("sync read failed", break): an unplugged
    // dongle fails every read at once, which used to spin here and flood kat.log.
    let mut failed = None;
    while live.receiving.load(Ordering::SeqCst) {
        let req = live.hop_request.swap(0, Ordering::SeqCst);
        if req != 0 {
            match sdr.set_center_freq(lo_for(req, SAMPLE_RATE)) {
                Ok(()) => {
                    // Samples already in the USB pipeline are from the old frequency: drop some.
                    let mut flush = vec![0u8; RETUNE_FLUSH_LEN];
                    let _ = sdr.read_sync(&mut flush);
                    tag = req;
                    blank = SAMPLE_RATE as u64 / 500; // and 2 ms more for the PLL
                }
                Err(e) => {
                    tracing::warn!("RTL-SDR retune to {} Hz failed: {:?}", req, e);
                    live.hop_failed.store(true, Ordering::SeqCst);
                }
            }
        }
        if gain_check.elapsed() >= std::time::Duration::from_millis(100) {
            gain_check = std::time::Instant::now();
            let g = *gain_settings.lock().unwrap();
            if g.gain_tenths_db != applied {
                match apply_gain(&mut sdr, g) {
                    Ok(()) => {
                        applied = g.gain_tenths_db;
                        live.floor_reset.store(true, Ordering::SeqCst);
                    }
                    Err(e) => tracing::warn!("{e}"),
                }
            }
        }
        let mut buf = vec![0u8; READ_LEN];
        match sdr.read_sync(&mut buf) {
            Ok(n) if n > 0 => {
                buf.truncate(n);
                live.record(&buf);
                if buf_tx.send((buf, tag, blank)).is_err() {
                    break;
                }
                blank = 0;
            }
            Ok(_) => {}
            Err(e) => {
                failed = Some(e);
                break;
            }
        }
    }

    drop(buf_tx);
    let _ = processor.join();
    if let Some(e) = failed {
        let _ = sdr.close();
        anyhow::bail!("RTL-SDR read failed ({e}): unplugged, or a USB error");
    }
    sdr.close().context("Failed to close RTL-SDR")?;
    tracing::info!("RTL-SDR receiver thread stopped");
    Ok(())
}

/// Receive-side controls (hopping, recording, retune info) used through `RadioDevice`.
impl RtlSdrController {
    /// IQ format of this device's raw samples (and of `:record` files).
    pub fn iq_format(&self) -> IqFormat {
        IqFormat::Cu8
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

#[cfg(test)]
mod tests {
    use super::*;

    /// The no-device popup tells a missing dongle from a permission problem or the DVB-T driver
    /// (rtl-sdr-rs's open errors, as its `Display` writes them).
    #[test]
    fn open_errors_in_words() {
        assert_eq!(describe_open_error("No device found at index 0"), "not found");
        assert!(describe_open_error("Error: Access").contains(super::super::USB_ACCESS_HINT));
        assert!(describe_open_error("Error: NotSupported").contains(super::super::USB_ACCESS_HINT));
        assert!(describe_open_error("Resource busy").contains(if cfg!(windows) { "another program" } else { "dvb_usb_rtl28xxu" }));
        assert_eq!(describe_open_error("Pipe error"), "cannot be opened (Pipe error)");
    }
}
