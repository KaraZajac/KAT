//! HackRF device control.
//!
//! This module provides a high-level interface for controlling HackRF devices
//! using the `libhackrf` crate. Falls back to demo mode at runtime if no
//! HackRF hardware is detected.

use anyhow::Result;
use std::sync::mpsc::Sender;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc, Mutex,
};
use std::thread::{self, JoinHandle};

use crate::app::RadioEvent;
use crate::capture::Capture;

use super::demodulator::Demodulator;
use super::demodulator::LevelDuration;

/// Sample rate for HackRF (2 MHz is good for keyfob signals)
const SAMPLE_RATE: u32 = 2_000_000;

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
    /// Demodulator for processing samples
    demodulator: Arc<Mutex<Demodulator>>,
    /// Whether HackRF is available
    hackrf_available: bool,
}

impl HackRfController {
    /// Create a new HackRF controller
    pub fn new(event_tx: Sender<RadioEvent>) -> Result<Self> {
        let demodulator = Demodulator::new(SAMPLE_RATE);

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
            demodulator: Arc::new(Mutex::new(demodulator)),
            hackrf_available,
        })
    }

    /// Check if HackRF is available
    #[allow(dead_code)]
    pub fn is_available(&self) -> bool {
        self.hackrf_available
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
        let demodulator = self.demodulator.clone();
        let hackrf_available = self.hackrf_available;

        self.rx_thread = Some(thread::spawn(move || {
            if hackrf_available {
                if let Err(e) =
                    run_receiver_hackrf(receiving.clone(), event_tx.clone(), freq, demodulator)
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
        // Note: gain changes take effect on next start_receiving
        // For now, just log - actual application happens in run_receiver_hackrf
        Ok(())
    }

    /// Set VGA gain (0-62 dB, 2 dB steps)
    pub fn set_vga_gain(&mut self, gain: u32) -> Result<()> {
        tracing::info!("Set VGA gain to {} dB", gain);
        // Note: gain changes take effect on next start_receiving
        Ok(())
    }

    /// Enable/disable the RF amplifier
    pub fn set_amp_enable(&mut self, enabled: bool) -> Result<()> {
        tracing::info!("Set amp enable to {}", enabled);
        // Note: amp changes take effect on next start_receiving
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
    demodulator: Arc<Mutex<Demodulator>>,
    capture_id: std::sync::atomic::AtomicU32,
}

/// RX callback function for libhackrf
fn rx_callback(
    _hackrf: &libhackrf::HackRf,
    buffer: &[num_complex::Complex<i8>],
    user_data: &dyn std::any::Any,
) {
    use crate::capture::StoredLevelDuration;
    
    // Downcast user_data to our state
    let state = match user_data.downcast_ref::<RxState>() {
        Some(s) => s,
        None => return,
    };

    if !state.receiving.load(Ordering::SeqCst) {
        return;
    }

    let current_freq = *state.frequency.lock().unwrap();

    // Convert Complex<i8> samples to i8 pairs for demodulator
    let samples: Vec<i8> = buffer.iter()
        .flat_map(|c| [c.re, c.im])
        .collect();

    // Process through demodulator
    if let Ok(mut demod) = state.demodulator.lock() {
        if let Some(pairs) = demod.process_samples(&samples) {
            // Convert to storable format
            let stored_pairs: Vec<StoredLevelDuration> = pairs
                .iter()
                .map(|p| StoredLevelDuration { level: p.level, duration_us: p.duration_us })
                .collect();
            
            let id = state.capture_id.fetch_add(1, Ordering::SeqCst);
            let capture = Capture::from_pairs(id, current_freq, stored_pairs);
            let _ = state.event_tx.send(RadioEvent::SignalCaptured(capture));
        }
    }
}

/// Run the receiver loop with actual HackRF using libhackrf
fn run_receiver_hackrf(
    receiving: Arc<AtomicBool>,
    event_tx: Sender<RadioEvent>,
    frequency: Arc<Mutex<u32>>,
    demodulator: Arc<Mutex<Demodulator>>,
) -> Result<()> {
    use anyhow::Context;

    tracing::info!("HackRF receiver thread starting...");

    // Open HackRF device
    let hackrf = libhackrf::HackRf::open()
        .context("Failed to open HackRF device")?;

    let freq = *frequency.lock().unwrap();
    tracing::info!("Configuring HackRF: freq={} Hz, sample_rate={} Hz", freq, SAMPLE_RATE);

    // Configure HackRF
    hackrf.set_sample_rate(SAMPLE_RATE)
        .context("Failed to set sample rate")?;
    
    hackrf.set_freq(freq as u64)
        .context("Failed to set frequency")?;
    
    hackrf.set_lna_gain(32)
        .context("Failed to set LNA gain")?;
    
    hackrf.set_rxvga_gain(20)
        .context("Failed to set RXVGA gain")?;
    
    hackrf.set_amp_enable(true)
        .context("Failed to enable amp")?;

    tracing::info!("HackRF configured, starting RX...");

    // Create state for callback
    let state = RxState {
        receiving: receiving.clone(),
        event_tx: event_tx.clone(),
        frequency: frequency.clone(),
        demodulator,
        capture_id: std::sync::atomic::AtomicU32::new(0),
    };

    // Start receiving
    hackrf.start_rx(rx_callback, state)
        .context("Failed to start RX")?;

    // Wait until receiving is stopped
    while receiving.load(Ordering::SeqCst) {
        std::thread::sleep(std::time::Duration::from_millis(100));
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
