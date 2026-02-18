//! RTL-SDR device control (receive-only).
//!
//! Provides an RX path compatible with the same IQ stream format and demodulators
//! as the HackRF path. Transmit is not supported; use HackRF for TX.

use anyhow::Result;
use std::sync::mpsc::Sender;
use std::sync::{
    atomic::{AtomicBool, AtomicU32, Ordering},
    Arc, Mutex,
};
use std::thread::{self, JoinHandle};

use crate::app::RadioEvent;
use crate::capture::{Capture, RfModulation, StoredLevelDuration};

use super::demodulator::Demodulator;
use super::demodulator::FmDemodulator;
use super::demodulator::LevelDuration;

use rtl_sdr_rs::{DeviceId, RtlSdr, TunerGain, DEFAULT_BUF_LENGTH};

/// Sample rate (2 MHz, same as HackRF path for keyfob signals)
const SAMPLE_RATE: u32 = 2_000_000;

/// Tuner gain setting for RTL-SDR (manual gain in 0.1 dB units; None = auto)
#[derive(Debug, Clone, Copy)]
struct TunerGainSetting {
    /// Manual gain in tenths of dB, or None for auto
    gain_tenths_db: Option<i32>,
}

impl Default for TunerGainSetting {
    fn default() -> Self {
        Self {
            gain_tenths_db: None, // auto
        }
    }
}

/// RTL-SDR controller: receive-only, no transmit.
pub struct RtlSdrController {
    event_tx: Sender<RadioEvent>,
    receiving: Arc<AtomicBool>,
    rx_thread: Option<JoinHandle<()>>,
    frequency: Arc<Mutex<u32>>,
    demodulator_am: Arc<Mutex<Demodulator>>,
    demodulator_fm: Arc<Mutex<FmDemodulator>>,
    rtlsdr_available: bool,
    gain_settings: Arc<Mutex<TunerGainSetting>>,
    /// RSSI (f32 bits) written by RX thread, read by UI - never blocks
    rssi_value: Arc<AtomicU32>,
}

impl RtlSdrController {
    /// Create a new RTL-SDR controller (receive-only).
    pub fn new(event_tx: Sender<RadioEvent>) -> Result<Self> {
        let demodulator_am = Demodulator::new(SAMPLE_RATE);
        let demodulator_fm = FmDemodulator::new(SAMPLE_RATE);

        let rtlsdr_available = check_rtlsdr_available();

        if rtlsdr_available {
            tracing::info!("RTL-SDR device detected (receive-only)");
        } else {
            tracing::debug!("RTL-SDR not detected");
        }

        Ok(Self {
            event_tx,
            receiving: Arc::new(AtomicBool::new(false)),
            rx_thread: None,
            frequency: Arc::new(Mutex::new(433_920_000)),
            demodulator_am: Arc::new(Mutex::new(demodulator_am)),
            demodulator_fm: Arc::new(Mutex::new(demodulator_fm)),
            rtlsdr_available,
            gain_settings: Arc::new(Mutex::new(TunerGainSetting::default())),
            rssi_value: Arc::new(AtomicU32::new(0)),
        })
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
        self.receiving.store(true, Ordering::SeqCst);

        let receiving = self.receiving.clone();
        let event_tx = self.event_tx.clone();
        let freq = self.frequency.clone();
        let demodulator_am = self.demodulator_am.clone();
        let demodulator_fm = self.demodulator_fm.clone();
        let rtlsdr_available = self.rtlsdr_available;
        let gain_settings = self.gain_settings.clone();
        let rssi_value = self.rssi_value.clone();

        self.rx_thread = Some(thread::spawn(move || {
            if rtlsdr_available {
                if let Err(e) = run_receiver_rtlsdr(
                    receiving.clone(),
                    event_tx.clone(),
                    freq,
                    demodulator_am,
                    demodulator_fm,
                    gain_settings,
                    rssi_value,
                ) {
                    let _ = event_tx.send(RadioEvent::Error(format!("RTL-SDR receiver error: {}", e)));
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

        tracing::info!("Stopped RTL-SDR receiving");
        Ok(())
    }

    /// Set the receive frequency.
    pub fn set_frequency(&mut self, frequency: u32) -> Result<()> {
        *self.frequency.lock().unwrap() = frequency;
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

fn check_rtlsdr_available() -> bool {
    match RtlSdr::open(DeviceId::Index(0)) {
        Ok(mut dev) => {
            if let Err(e) = dev.close() {
                tracing::debug!("RTL-SDR close after probe: {:?}", e);
            }
            true
        }
        Err(e) => {
            tracing::debug!("RTL-SDR not available: {:?}", e);
            false
        }
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

fn pairs_to_stored(pairs: &[LevelDuration]) -> Vec<StoredLevelDuration> {
    pairs
        .iter()
        .map(|p| StoredLevelDuration {
            level: p.level,
            duration_us: p.duration_us,
        })
        .collect()
}

/// Convert RTL-SDR u8 IQ (0–255) to signed i8 (-128..127) for the demodulators.
fn u8_iq_to_i8(buf: &[u8]) -> Vec<i8> {
    buf.iter()
        .map(|&b| (b as i16 - 128) as i8)
        .collect()
}

/// Average magnitude of interleaved I/Q i8 samples (0..~1).
fn compute_rssi_i8(samples: &[i8]) -> f32 {
    if samples.len() < 2 {
        return 0.0;
    }
    let mut sum = 0.0f32;
    let mut count = 0usize;
    for chunk in samples.chunks(2) {
        if chunk.len() < 2 {
            break;
        }
        let i = chunk[0] as f32 / 128.0;
        let q = chunk[1] as f32 / 128.0;
        sum += (i * i + q * q).sqrt();
        count += 1;
    }
    if count == 0 {
        0.0
    } else {
        sum / count as f32
    }
}

/// Run the receiver loop with an RTL-SDR device.
fn run_receiver_rtlsdr(
    receiving: Arc<AtomicBool>,
    event_tx: Sender<RadioEvent>,
    frequency: Arc<Mutex<u32>>,
    demodulator_am: Arc<Mutex<Demodulator>>,
    demodulator_fm: Arc<Mutex<FmDemodulator>>,
    gain_settings: Arc<Mutex<TunerGainSetting>>,
    rssi_value: Arc<AtomicU32>,
) -> Result<()> {
    use anyhow::Context;

    tracing::info!("RTL-SDR receiver thread starting...");

    let mut sdr = RtlSdr::open(DeviceId::Index(0)).context("Failed to open RTL-SDR device")?;

    let freq = *frequency.lock().unwrap();
    let initial_gain = *gain_settings.lock().unwrap();

    sdr.reset_buffer().context("Failed to reset RTL-SDR buffer")?;
    sdr.set_center_freq(freq).context("Failed to set RTL-SDR frequency")?;
    sdr.set_sample_rate(SAMPLE_RATE).context("Failed to set RTL-SDR sample rate")?;
    sdr.set_bias_tee(false).context("Failed to set bias-tee")?;

    if let Some(tenths) = initial_gain.gain_tenths_db {
        sdr.set_tuner_gain(TunerGain::Manual(tenths)).context("Failed to set RTL-SDR gain")?;
    } else {
        sdr.set_tuner_gain(TunerGain::Auto).context("Failed to set RTL-SDR gain")?;
    }

    tracing::info!(
        "RTL-SDR configured: freq={} Hz, sample_rate={} Hz",
        freq, SAMPLE_RATE
    );

    let capture_id = std::sync::atomic::AtomicU32::new(0);
    let mut buf = vec![0u8; DEFAULT_BUF_LENGTH];

    while receiving.load(Ordering::SeqCst) {
        match sdr.read_sync(&mut buf) {
            Ok(n) if n == buf.len() => {
                let current_freq = *frequency.lock().unwrap();
                let samples = u8_iq_to_i8(&buf[..n]);

                rssi_value.store(compute_rssi_i8(&samples).to_bits(), Ordering::Relaxed);

                if let Ok(mut demod) = demodulator_am.lock() {
                    if let Some(pairs) = demod.process_samples(&samples) {
                        let id = capture_id.fetch_add(1, Ordering::SeqCst);
                        let capture = Capture::from_pairs_with_rf(
                            id,
                            current_freq,
                            pairs_to_stored(&pairs),
                            Some(RfModulation::AM),
                        );
                        let _ = event_tx.send(RadioEvent::SignalCaptured(capture));
                    }
                }
                if let Ok(mut demod) = demodulator_fm.lock() {
                    if let Some(pairs) = demod.process_samples(&samples) {
                        let id = capture_id.fetch_add(1, Ordering::SeqCst);
                        let capture = Capture::from_pairs_with_rf(
                            id,
                            current_freq,
                            pairs_to_stored(&pairs),
                            Some(RfModulation::FM),
                        );
                        let _ = event_tx.send(RadioEvent::SignalCaptured(capture));
                    }
                }
            }
            Ok(_) => {}
            Err(e) => {
                tracing::warn!("RTL-SDR read error: {:?}", e);
            }
        }
    }

    sdr.close().context("Failed to close RTL-SDR")?;
    tracing::info!("RTL-SDR receiver thread stopped");
    Ok(())
}
