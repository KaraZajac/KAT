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
use crate::capture::{Capture, RfModulation, StoredLevelDuration};

use super::demodulator::Demodulator;
use super::demodulator::FmDemodulator;
use super::demodulator::LevelDuration;

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
    /// AM/OOK demodulator
    demodulator_am: Arc<Mutex<Demodulator>>,
    /// FM/2FSK demodulator
    demodulator_fm: Arc<Mutex<FmDemodulator>>,
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
        let demodulator_am = Demodulator::new(SAMPLE_RATE);
        let demodulator_fm = FmDemodulator::new(SAMPLE_RATE);

        // Check if HackRF is available
        let hackrf_available = check_hackrf_available();

        if hackrf_available {
            tracing::info!("HackRF device detected");
        } else {
            tracing::warn!("HackRF not detected - running in demo mode");
        }

        Ok(Self {
            event_tx,
            receiving: Arc::new(AtomicBool::new(false)),
            rx_thread: None,
            frequency: Arc::new(Mutex::new(433_920_000)),
            demodulator_am: Arc::new(Mutex::new(demodulator_am)),
            demodulator_fm: Arc::new(Mutex::new(demodulator_fm)),
            hackrf_available,
            gain_settings: Arc::new(Mutex::new(GainSettings::default())),
            rssi_value: Arc::new(AtomicU32::new(0)),
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
        self.receiving.store(true, Ordering::SeqCst);

        let receiving = self.receiving.clone();
        let event_tx = self.event_tx.clone();
        let freq = self.frequency.clone();
        let demodulator_am = self.demodulator_am.clone();
        let demodulator_fm = self.demodulator_fm.clone();
        let hackrf_available = self.hackrf_available;
        let gain_settings = self.gain_settings.clone();
        let rssi_value = self.rssi_value.clone();

        self.rx_thread = Some(thread::spawn(move || {
            if hackrf_available {
                if let Err(e) = run_receiver_hackrf(
                    receiving.clone(),
                    event_tx.clone(),
                    freq,
                    demodulator_am,
                    demodulator_fm,
                    gain_settings,
                    rssi_value,
                )
                {
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

        tracing::info!("Stopped receiving");
        Ok(())
    }

    /// Set the receive frequency
    pub fn set_frequency(&mut self, frequency: u32) -> Result<()> {
        *self.frequency.lock().unwrap() = frequency;
        tracing::info!("Set frequency to {} Hz", frequency);
        Ok(())
    }

    /// Transmit a signal
    pub fn transmit(&mut self, signal: &[LevelDuration], frequency: u32) -> Result<()> {
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
            "Transmitting {} level/duration pairs at {} Hz",
            signal.len(),
            frequency
        );

        transmit_signal_hackrf(signal, frequency)?;

        // Resume receiving if we were before
        if was_receiving {
            let freq = *self.frequency.lock().unwrap();
            self.start_receiving(freq)?;
        }

        Ok(())
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

/// Shared state for RX callback (libhackrf requires fn pointers, not closures)
struct RxState {
    receiving: Arc<AtomicBool>,
    event_tx: Sender<RadioEvent>,
    frequency: Arc<Mutex<u32>>,
    demodulator_am: Arc<Mutex<Demodulator>>,
    demodulator_fm: Arc<Mutex<FmDemodulator>>,
    capture_id: std::sync::atomic::AtomicU32,
    /// RSSI (f32 bits) written here so callback never blocks on channel
    rssi_value: Arc<AtomicU32>,
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

/// Compute average magnitude of IQ buffer (0..~1 for i8)
fn compute_rssi(buffer: &[num_complex::Complex<i8>]) -> f32 {
    if buffer.is_empty() {
        return 0.0;
    }
    let sum_mag: f32 = buffer
        .iter()
        .map(|c| {
            let i = c.re as f32 / 128.0;
            let q = c.im as f32 / 128.0;
            (i * i + q * q).sqrt()
        })
        .sum();
    sum_mag / buffer.len() as f32
}

/// RX callback: feed same IQ to AM and FM demodulators; emit a capture per path when signal complete.
fn rx_callback(
    _hackrf: &libhackrf::HackRf,
    buffer: &[num_complex::Complex<i8>],
    user_data: &dyn std::any::Any,
) {
    let state = match user_data.downcast_ref::<RxState>() {
        Some(s) => s,
        None => return,
    };
    if !state.receiving.load(Ordering::SeqCst) {
        return;
    }
    let current_freq = *state.frequency.lock().unwrap();
    let samples: Vec<i8> = buffer.iter().flat_map(|c| [c.re, c.im]).collect();

    state.rssi_value.store(compute_rssi(buffer).to_bits(), Ordering::Relaxed);

    if let Ok(mut demod) = state.demodulator_am.lock() {
        if let Some(pairs) = demod.process_samples(&samples) {
            let id = state.capture_id.fetch_add(1, Ordering::SeqCst);
            let capture = Capture::from_pairs_with_rf(
                id,
                current_freq,
                pairs_to_stored(&pairs),
                Some(RfModulation::AM),
            );
            let _ = state.event_tx.send(RadioEvent::SignalCaptured(capture));
        }
    }
    if let Ok(mut demod) = state.demodulator_fm.lock() {
        if let Some(pairs) = demod.process_samples(&samples) {
            let id = state.capture_id.fetch_add(1, Ordering::SeqCst);
            let capture = Capture::from_pairs_with_rf(
                id,
                current_freq,
                pairs_to_stored(&pairs),
                Some(RfModulation::FM),
            );
            let _ = state.event_tx.send(RadioEvent::SignalCaptured(capture));
        }
    }
}

/// Run the receiver loop with actual HackRF using libhackrf
fn run_receiver_hackrf(
    receiving: Arc<AtomicBool>,
    event_tx: Sender<RadioEvent>,
    frequency: Arc<Mutex<u32>>,
    demodulator_am: Arc<Mutex<Demodulator>>,
    demodulator_fm: Arc<Mutex<FmDemodulator>>,
    gain_settings: Arc<Mutex<GainSettings>>,
    rssi_value: Arc<AtomicU32>,
) -> Result<()> {
    use anyhow::Context;

    tracing::info!("HackRF receiver thread starting...");

    let hackrf = libhackrf::HackRf::open()
        .context("Failed to open HackRF device")?;

    let freq = *frequency.lock().unwrap();
    let initial_gains = *gain_settings.lock().unwrap();
    tracing::info!(
        "Configuring HackRF: freq={} Hz, sample_rate={} Hz, LNA={} dB, VGA={} dB, AMP={}",
        freq, SAMPLE_RATE, initial_gains.lna_gain, initial_gains.vga_gain, initial_gains.amp_enabled
    );

    hackrf.set_sample_rate(SAMPLE_RATE)
        .context("Failed to set sample rate")?;
    hackrf.set_freq(freq as u64)
        .context("Failed to set frequency")?;
    hackrf.set_lna_gain(initial_gains.lna_gain)
        .context("Failed to set LNA gain")?;
    hackrf.set_rxvga_gain(initial_gains.vga_gain)
        .context("Failed to set RXVGA gain")?;
    hackrf.set_amp_enable(initial_gains.amp_enabled)
        .context("Failed to enable amp")?;

    tracing::info!("HackRF configured, starting RX (AM + FM demodulators)...");

    let state = RxState {
        receiving: receiving.clone(),
        event_tx: event_tx.clone(),
        frequency: frequency.clone(),
        demodulator_am,
        demodulator_fm,
        capture_id: std::sync::atomic::AtomicU32::new(0),
        rssi_value,
    };


    // Start receiving
    hackrf.start_rx(rx_callback, state)
        .context("Failed to start RX")?;

    // Track applied settings so we can detect changes
    let mut applied = initial_gains;

    // Wait until receiving is stopped, applying gain changes live
    while receiving.load(Ordering::SeqCst) {
        std::thread::sleep(std::time::Duration::from_millis(100));

        // Check for gain/amp setting changes and apply them live
        if let Ok(current) = gain_settings.lock() {
            if current.lna_gain != applied.lna_gain {
                if let Err(e) = hackrf.set_lna_gain(current.lna_gain) {
                    tracing::warn!("Failed to set LNA gain to {}: {:?}", current.lna_gain, e);
                } else {
                    tracing::info!("Applied LNA gain: {} dB", current.lna_gain);
                    applied.lna_gain = current.lna_gain;
                }
            }
            if current.vga_gain != applied.vga_gain {
                if let Err(e) = hackrf.set_rxvga_gain(current.vga_gain) {
                    tracing::warn!("Failed to set VGA gain to {}: {:?}", current.vga_gain, e);
                } else {
                    tracing::info!("Applied VGA gain: {} dB", current.vga_gain);
                    applied.vga_gain = current.vga_gain;
                }
            }
            if current.amp_enabled != applied.amp_enabled {
                if let Err(e) = hackrf.set_amp_enable(current.amp_enabled) {
                    tracing::warn!("Failed to set amp to {}: {:?}", current.amp_enabled, e);
                } else {
                    tracing::info!("Applied amp: {}", if current.amp_enabled { "ON" } else { "OFF" });
                    applied.amp_enabled = current.amp_enabled;
                }
            }
        }
    }

    // Stop receiving
    hackrf.stop_rx().context("Failed to stop RX")?;

    tracing::info!("HackRF receiver thread stopped");
    Ok(())
}

/// Shared state for TX callback
struct TxState {
    samples: Vec<(i8, i8)>,
    sample_index: std::sync::atomic::AtomicUsize,
}

/// TX callback function for libhackrf
fn tx_callback(
    _hackrf: &libhackrf::HackRf,
    buffer: &mut [num_complex::Complex<i8>],
    user_data: &dyn std::any::Any,
) {
    use num_complex::Complex;
    
    // Downcast user_data to our state
    let state = match user_data.downcast_ref::<TxState>() {
        Some(s) => s,
        None => return,
    };

    let total = state.samples.len();
    
    for sample in buffer.iter_mut() {
        let idx = state.sample_index.fetch_add(1, Ordering::SeqCst);
        if idx < total {
            let (i, q) = state.samples[idx];
            *sample = Complex::new(i, q);
        } else {
            *sample = Complex::new(0, 0);
        }
    }
}

/// Transmit a signal via HackRF
fn transmit_signal_hackrf(signal: &[LevelDuration], frequency: u32) -> Result<()> {
    use anyhow::Context;

    tracing::info!("Starting HackRF transmission at maximum power...");

    // Open HackRF device
    let hackrf = libhackrf::HackRf::open()
        .context("Failed to open HackRF device")?;

    // Configure for TX with MAXIMUM POWER
    hackrf.set_sample_rate(SAMPLE_RATE)
        .context("Failed to set sample rate")?;
    
    hackrf.set_freq(frequency as u64)
        .context("Failed to set frequency")?;
    
    // Set TX VGA gain to maximum (47 dB is the max for HackRF)
    hackrf.set_txvga_gain(47)
        .context("Failed to set TXVGA gain")?;
    
    // Enable the RF amplifier for +14dB additional gain
    hackrf.set_amp_enable(true)
        .context("Failed to enable amp")?;

    // Generate TX samples
    let tx_samples = generate_tx_samples(signal, SAMPLE_RATE);
    let total_samples = tx_samples.len();

    tracing::debug!("Generated {} TX samples", total_samples);

    // Create state for callback
    let state = TxState {
        samples: tx_samples,
        sample_index: std::sync::atomic::AtomicUsize::new(0),
    };

    // Start transmitting
    hackrf.start_tx(tx_callback, state)
        .context("Failed to start TX")?;

    // Wait for transmission to complete (check sample_index through a loop)
    // We can't easily check completion with this API, so just wait based on expected time
    let duration_us: u32 = signal.iter().map(|s| s.duration_us).sum();
    let wait_ms = (duration_us / 1000).max(100);
    std::thread::sleep(std::time::Duration::from_millis(wait_ms as u64 + 100));

    // Stop transmitting
    hackrf.stop_tx().context("Failed to stop TX")?;

    tracing::info!("Transmission complete");
    Ok(())
}

/// Generate TX samples from level/duration pairs
fn generate_tx_samples(signal: &[LevelDuration], sample_rate: u32) -> Vec<(i8, i8)> {
    let mut samples = Vec::new();
    let samples_per_us = sample_rate as f64 / 1_000_000.0;

    for ld in signal {
        let num_samples = (ld.duration_us as f64 * samples_per_us) as usize;
        let value: i8 = if ld.level { 127 } else { 0 };

        // IQ samples
        for _ in 0..num_samples {
            samples.push((value, 0)); // I, Q (Q=0 for OOK)
        }
    }

    samples
}
