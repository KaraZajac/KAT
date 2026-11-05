//! Receive-side glue shared by the HackRF callback, the RTL-SDR thread and IQ files.
//!
//! - [`burst_captures`]: a pipeline [`Burst`] → one `Capture` per demodulated path (AM / FM), with
//!   its on-air time in the receive session ([`new_session`]).
//! - [`LiveShared`]: what the app (through a controller) shares with a running receiver: home and
//!   current frequency, RSSI, the IQ recorder, the hop plan and hop requests, gain-change notices.
//! - [`LiveRx`]: per-receiver processing (pipeline + hopper): feeds buffers, emits captures, asks the
//!   device thread for retunes, and is told when a retune took effect.
//!
//! Offset tuning: live receivers put the LO [`tuning_offset`] (fs/8, 250 kHz at 2 MS/s) below the
//! target, so the DC spike / LO leakage of HackRF and RTL-SDR is 250 kHz from it (the pipeline's
//! detector leaves the LO bins out and searches the rest of the band); captures always carry the
//! target frequency, bursts the carrier's measured offset from it.

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::mpsc::Sender;
use std::sync::{Arc, Mutex};

use crate::app::RadioEvent;
use crate::capture::{Capture, OnAir, RfModulation, StoredLevelDuration};

use super::demodulator::LevelDuration;
use super::dsp::IqFormat;
use super::hopper::{HopPlan, Hopper};
use super::pipeline::{Burst, RxPipeline};
use super::recorder::{Recorder, RecordingStatus, RecordingSummary};

/// Where the target sits above the LO for live RX (fs/8: well inside the HackRF's 1.75 MHz baseband
/// filter, 250 kHz from the DC spike at 2 MS/s; fobs are found from −1.15 to +0.65 MHz around it).
pub fn tuning_offset(sample_rate: u32) -> f64 {
    sample_rate as f64 / 8.0
}

/// LO (centre) frequency for a target with offset tuning.
pub fn lo_for(target: u32, sample_rate: u32) -> u32 {
    (target as f64 - tuning_offset(sample_rate)).round().max(0.0) as u32
}

fn stored(pairs: &[LevelDuration]) -> Vec<StoredLevelDuration> {
    pairs.iter().map(|p| StoredLevelDuration { level: p.level, duration_us: p.duration_us }).collect()
}

/// A new receive session (one RX run, one IQ file): captures' on-air times compare within one.
pub fn new_session() -> u64 {
    static NEXT: AtomicU64 = AtomicU64::new(1);
    NEXT.fetch_add(1, Ordering::Relaxed)
}

/// One capture per path the burst carries (AM first), tagged with the burst's frequency and its
/// on-air time in `session`, whose samples come at `sample_rate`.
pub fn burst_captures(b: &Burst, session: u64, sample_rate: f64) -> Vec<Capture> {
    let start_us = (b.start_sample as f64 * 1e6 / sample_rate) as u64;
    let mut out = Vec::new();
    for (pairs, rf) in [(&b.am, RfModulation::AM), (&b.fm, RfModulation::FM)] {
        if let Some(p) = pairs {
            let mut capture = Capture::from_pairs_with_rf(0, b.frequency, stored(p), Some(rf));
            let length: u64 = p.iter().map(|x| u64::from(x.duration_us)).sum();
            capture.on_air = Some(OnAir { session, start_us, end_us: start_us + length });
            out.push(capture);
        }
    }
    out
}

/// State shared between a controller (app thread) and its receive thread(s).
pub struct LiveShared {
    pub receiving: Arc<AtomicBool>,
    pub rssi: Arc<AtomicU32>,
    /// Frequency set by the user (`:freq`); hopping returns here when switched off.
    pub home: AtomicU32,
    /// Frequency being received right now (follows hops).
    pub target: AtomicU32,
    /// Position of `target` in the hop list (UI).
    pub hop_index: AtomicU32,
    pub recorder: Mutex<Option<Recorder>>,
    /// Summary of a recording that ended because RX stopped (picked up by the app).
    pub ended_recording: Mutex<Option<RecordingSummary>>,
    pub hop_plan: Mutex<Option<HopPlan>>,
    /// Bumped whenever `hop_plan` changes.
    pub hop_generation: AtomicU32,
    /// Receive processing → device thread: retune to this target (0 = none).
    pub hop_request: AtomicU32,
    /// Device thread → processing: the last retune failed.
    pub hop_failed: AtomicBool,
    /// App → processing: hold the current hop frequency for this many dwells (after a decode).
    pub hop_pause_dwells: AtomicU32,
    /// App/controller → processing: gains changed, re-learn the noise floor.
    pub floor_reset: AtomicBool,
}

impl LiveShared {
    pub fn new(receiving: Arc<AtomicBool>, rssi: Arc<AtomicU32>, frequency: u32) -> Self {
        Self {
            receiving,
            rssi,
            home: AtomicU32::new(frequency),
            target: AtomicU32::new(frequency),
            hop_index: AtomicU32::new(0),
            recorder: Mutex::new(None),
            ended_recording: Mutex::new(None),
            hop_plan: Mutex::new(None),
            hop_generation: AtomicU32::new(0),
            hop_request: AtomicU32::new(0),
            hop_failed: AtomicBool::new(false),
            hop_pause_dwells: AtomicU32::new(0),
            floor_reset: AtomicBool::new(false),
        }
    }

    /// Install or clear the hop plan (takes effect in the receive thread).
    pub fn set_hop_plan(&self, plan: Option<HopPlan>) {
        *self.hop_plan.lock().unwrap() = plan;
        self.hop_generation.fetch_add(1, Ordering::SeqCst);
    }

    pub fn hop_plan(&self) -> Option<HopPlan> {
        self.hop_plan.lock().unwrap().clone()
    }

    /// Frequency a receiver should start on: the first hop frequency when hopping, else home.
    pub fn start_frequency(&self) -> u32 {
        match self.hop_plan.lock().unwrap().as_ref() {
            Some(p) if !p.frequencies.is_empty() => p.frequencies[0],
            _ => self.home.load(Ordering::SeqCst),
        }
    }

    /// Start recording to `path` (fails if one is already running). The file is created before
    /// the lock is taken, so the receive thread never waits on the file system.
    pub fn start_recording(&self, path: std::path::PathBuf) -> anyhow::Result<()> {
        if self.recorder.lock().unwrap().is_some() {
            anyhow::bail!("already recording");
        }
        let new = Recorder::start(path)?;
        let mut rec = self.recorder.lock().unwrap();
        if rec.is_some() {
            anyhow::bail!("already recording");
        }
        *rec = Some(new);
        Ok(())
    }

    pub fn stop_recording(&self) -> Option<RecordingSummary> {
        let rec = self.recorder.lock().unwrap().take();
        rec.map(Recorder::stop)
    }

    /// Stop a recording because RX stopped; the app picks the summary up later.
    pub fn end_recording_with_rx(&self) {
        if let Some(sum) = self.stop_recording() {
            *self.ended_recording.lock().unwrap() = Some(sum);
        }
    }

    pub fn recording_status(&self) -> Option<RecordingStatus> {
        self.recorder.lock().unwrap().as_ref().map(Recorder::status)
    }

    /// Hand the raw buffer to the recorder, if one is running. Never waits on the disk: the lock
    /// is only ever held for a status read or an install/remove, and `push` doesn't block.
    pub fn record(&self, bytes: &[u8]) {
        if let Ok(rec) = self.recorder.lock() {
            if let Some(r) = rec.as_ref() {
                r.push(bytes);
            }
        }
    }
}

/// Samples per hop-decision step (~16 ms at 2 MS/s).
const STEP_SAMPLES: usize = 32_768;

/// Per-receiver processing: pipeline + hopper.
pub struct LiveRx {
    pub pipeline: RxPipeline,
    hopper: Option<Hopper>,
    hop_gen: u32,
    /// A retune back to the home frequency was requested and hasn't taken effect yet.
    homing: bool,
    format: IqFormat,
    sample_rate: u32,
    events: Sender<RadioEvent>,
    /// This receive run ([new_session]).
    session: u64,
}

impl LiveRx {
    /// Processing for a device delivering `format` at `sample_rate`, tuned (with offset) to `target`.
    pub fn new(sample_rate: u32, format: IqFormat, target: u32, events: Sender<RadioEvent>) -> Self {
        Self {
            pipeline: RxPipeline::new(sample_rate, tuning_offset(sample_rate), target),
            hopper: None,
            hop_gen: u32::MAX,
            homing: false,
            format,
            sample_rate,
            events,
            session: new_session(),
        }
    }

    fn emit(&self, bursts: Vec<Burst>) {
        for b in bursts {
            tracing::debug!(
                "burst at {} Hz: {:.1} ms, SNR {:.1} dB, offset {:.1} kHz, dev {:?}, AM {} FM {}",
                b.frequency,
                b.duration_us as f64 / 1e3,
                b.snr_db,
                b.offset_hz / 1e3,
                b.deviation_hz,
                b.am.as_ref().map_or(0, |p| p.len()),
                b.fm.as_ref().map_or(0, |p| p.len())
            );
            for cap in burst_captures(&b, self.session, f64::from(self.sample_rate)) {
                let _ = self.events.send(RadioEvent::SignalCaptured(cap));
            }
        }
    }

    /// Pick up hop-plan changes, pause requests, failed retunes and gain changes.
    fn sync_control(&mut self, shared: &LiveShared) {
        let gen = shared.hop_generation.load(Ordering::SeqCst);
        if gen != self.hop_gen {
            self.hop_gen = gen;
            let current = self.pipeline.frequency();
            match shared.hop_plan() {
                Some(plan) if plan.frequencies.len() > 1 => {
                    let fs = self.sample_rate as f64;
                    match plan.frequencies.iter().position(|&f| f == current) {
                        Some(i) => {
                            shared.hop_index.store(i as u32, Ordering::SeqCst);
                            self.hopper = Some(Hopper::new(plan, fs, i));
                        }
                        None => {
                            // Not on the list: go to its first frequency now.
                            let mut h = Hopper::new(plan.clone(), fs, plan.frequencies.len() - 1);
                            if let Some(next) = h.advance(u64::MAX / 4, false) {
                                shared.hop_request.store(next, Ordering::SeqCst);
                            }
                            self.hopper = Some(h);
                        }
                    }
                }
                _ => self.hopper = None,
            }
        }
        if shared.hop_failed.swap(false, Ordering::SeqCst) {
            if let Some(h) = self.hopper.as_mut() {
                h.cancel();
            }
            self.homing = false;
        }
        // Not hopping: make sure we end up on the home frequency (also when hopping was switched
        // off while a hop retune was still on its way).
        if self.hopper.is_none() && !self.homing {
            let home = shared.home.load(Ordering::SeqCst);
            if home != self.pipeline.frequency() {
                shared.hop_request.store(home, Ordering::SeqCst);
                self.homing = true;
            }
        }
        let pause = shared.hop_pause_dwells.swap(0, Ordering::SeqCst);
        if pause > 0 {
            if let Some(h) = self.hopper.as_mut() {
                h.pause_dwells(pause as u64);
            }
        }
        if shared.floor_reset.swap(false, Ordering::SeqCst) {
            self.pipeline.reset_floor();
        }
    }

    /// Process a buffer received on the current frequency.
    pub fn process(&mut self, bytes: &[u8], shared: &LiveShared) {
        self.sync_control(shared);
        let bps = self.format.bytes_per_sample();
        for chunk in bytes.chunks(STEP_SAMPLES * bps) {
            let bursts = self.pipeline.process(chunk, self.format);
            self.emit(bursts);
            if let Some(h) = self.hopper.as_mut() {
                if let Some(next) = h.advance((chunk.len() / bps) as u64, self.pipeline.busy()) {
                    shared.hop_request.store(next, Ordering::SeqCst);
                }
            }
        }
        shared.rssi.store(self.pipeline.take_rssi().to_bits(), Ordering::Relaxed);
    }

    /// The device now delivers `target` (after a retune): drop `blank` settling samples.
    pub fn retuned(&mut self, target: u32, blank: u64, shared: &LiveShared) {
        let offset = tuning_offset(self.sample_rate);
        let bursts = self.pipeline.retune(target, offset, blank);
        self.emit(bursts);
        self.homing = false;
        if let Some(h) = self.hopper.as_mut() {
            h.hopped();
            if let Some(i) = h.plan().frequencies.iter().position(|&f| f == target) {
                shared.hop_index.store(i as u32, Ordering::SeqCst);
            }
        }
        shared.target.store(target, Ordering::SeqCst);
    }

    /// Close any open burst (receiver stopping).
    pub fn finish(&mut self) {
        let bursts = self.pipeline.flush();
        self.emit(bursts);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn offset_tuning_numbers() {
        assert_eq!(tuning_offset(2_000_000), 250_000.0);
        assert_eq!(lo_for(433_920_000, 2_000_000), 433_670_000);
        assert_eq!(lo_for(433_920_000, 2_048_000), 433_664_000);
    }

    /// Hop plan → retune requests → `retuned` → bursts tagged with the frequency they were heard on.
    #[test]
    fn hopping_requests_retunes_and_tags_bursts() {
        let (tx, rx) = std::sync::mpsc::channel();
        let shared = LiveShared::new(Arc::new(AtomicBool::new(true)), Arc::new(AtomicU32::new(0)), 433_920_000);
        let mut live = LiveRx::new(2_000_000, IqFormat::Cs8, 433_920_000, tx);
        shared.set_hop_plan(Some(HopPlan::new(vec![433_920_000, 868_350_000], 50)));
        // 60 ms of near-silence (tiny noise) → one hop request to 868.35 after the 50 ms dwell.
        let mut x = 0x2545_F491_4F6C_DD1Du64;
        let quiet: Vec<u8> = (0..240_000)
            .map(|_| {
                x ^= x << 13;
                x ^= x >> 7;
                x ^= x << 17;
                ((x >> 61) as i8 - 4) as u8
            })
            .collect();
        live.process(&quiet, &shared);
        assert_eq!(shared.hop_request.swap(0, Ordering::SeqCst), 868_350_000);
        live.retuned(868_350_000, 2_000, &shared);
        assert_eq!(shared.target.load(Ordering::SeqCst), 868_350_000);
        assert_eq!(shared.hop_index.load(Ordering::SeqCst), 1);
        // Switching hopping off asks to go home, once.
        shared.set_hop_plan(None);
        live.process(&quiet[..1000], &shared);
        assert_eq!(shared.hop_request.swap(0, Ordering::SeqCst), 433_920_000);
        live.process(&quiet[..1000], &shared);
        assert_eq!(shared.hop_request.load(Ordering::SeqCst), 0, "no repeated request while it is on its way");
        // A hop retune that lands after hopping was switched off still ends at home.
        live.retuned(868_350_000, 0, &shared);
        live.process(&quiet[..1000], &shared);
        assert_eq!(shared.hop_request.swap(0, Ordering::SeqCst), 433_920_000);
        live.retuned(433_920_000, 0, &shared);
        live.process(&quiet[..1000], &shared);
        assert_eq!(shared.hop_request.load(Ordering::SeqCst), 0);
        assert!(rx.try_recv().is_err(), "no captures from silence");
    }
}
