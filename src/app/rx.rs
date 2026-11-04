//! Receive features on the app side: IQ files (`:iq`, `kat --iq`, `.cs8`/`.cu8` in the `:load`
//! browser), IQ recording (`:record`), frequency hopping (`:hop`, the Hop setting), and the state the
//! header and status bar show. A child module of `app` (`mod rx;`), so it can use App's fields.

use std::path::PathBuf;

use anyhow::Result;

use super::{App, RadioDevice, RadioState};
use crate::radio::dsp::IqFormat;
use crate::radio::hopper::{format_mhz, parse_frequency_list, HopPlan, DECODE_PAUSE_DWELLS};
use crate::radio::iq_file::{find_file, parse_frequency_arg, parse_rate_arg, IqFileJob, IqFileSpec};
use crate::radio::recorder::{recording_file_name, resolve_record_path, RecordingStatus, RecordingSummary};
use crate::radio::rx::lo_for;

/// App-side receive state (the `rx` field of `App`).
pub struct RxExtras {
    /// IQ file being processed in the background.
    pub iq_job: Option<IqFileJob>,
    /// Captures in the list when the job started (to report how many it added).
    iq_captures_before: usize,
    /// Hopping switched on (`:hop on` / the Hop setting); kept by the radio across RX restarts.
    pub hop_enabled: bool,
    /// Frequencies and dwell set with `:hop <MHz> ...` / `:hop dwell` (None: the config's).
    hop_plan: Option<HopPlan>,
    /// `App::frequency` when hopping was switched on: a `:freq` change switches hopping off.
    hop_home: u32,
    /// Captures already looked at by the pause-after-decode logic.
    seen_captures: usize,
}

impl RxExtras {
    pub fn new() -> Self {
        Self { iq_job: None, iq_captures_before: 0, hop_enabled: false, hop_plan: None, hop_home: 0, seen_captures: 0 }
    }
}

impl Default for RxExtras {
    fn default() -> Self {
        Self::new()
    }
}

/// What the header shows about receiving.
pub struct RxHeader {
    /// Hopping: (frequency now, position, list length).
    pub hop: Option<(u32, usize, usize)>,
    pub recording: Option<RecordingStatus>,
    /// IQ file progress (0..=1) while one is processed.
    pub iq_progress: Option<f32>,
}

/// Receive-side controls, forwarded to the active device.
impl RadioDevice {
    pub fn iq_format(&self) -> IqFormat {
        match self {
            RadioDevice::HackRf(h) => h.iq_format(),
            RadioDevice::RtlSdr(r) => r.iq_format(),
        }
    }

    pub fn sample_rate(&self) -> u32 {
        match self {
            RadioDevice::HackRf(h) => h.sample_rate(),
            RadioDevice::RtlSdr(r) => r.sample_rate(),
        }
    }

    /// Frequency being received right now (follows hops).
    pub fn rx_frequency(&self) -> u32 {
        match self {
            RadioDevice::HackRf(h) => h.rx_frequency(),
            RadioDevice::RtlSdr(r) => r.rx_frequency(),
        }
    }

    pub fn set_hopping(&mut self, plan: Option<HopPlan>) {
        match self {
            RadioDevice::HackRf(h) => h.set_hopping(plan),
            RadioDevice::RtlSdr(r) => r.set_hopping(plan),
        }
    }

    pub fn hop_state(&self) -> Option<(HopPlan, usize)> {
        match self {
            RadioDevice::HackRf(h) => h.hop_state(),
            RadioDevice::RtlSdr(r) => r.hop_state(),
        }
    }

    pub fn hop_pause(&mut self, dwells: u32) {
        match self {
            RadioDevice::HackRf(h) => h.hop_pause(dwells),
            RadioDevice::RtlSdr(r) => r.hop_pause(dwells),
        }
    }

    pub fn start_recording(&mut self, path: PathBuf) -> Result<()> {
        match self {
            RadioDevice::HackRf(h) => h.start_recording(path),
            RadioDevice::RtlSdr(r) => r.start_recording(path),
        }
    }

    pub fn stop_recording(&mut self) -> Option<RecordingSummary> {
        match self {
            RadioDevice::HackRf(h) => h.stop_recording(),
            RadioDevice::RtlSdr(r) => r.stop_recording(),
        }
    }

    pub fn recording_status(&self) -> Option<RecordingStatus> {
        match self {
            RadioDevice::HackRf(h) => h.recording_status(),
            RadioDevice::RtlSdr(r) => r.recording_status(),
        }
    }

    pub fn take_ended_recording(&mut self) -> Option<RecordingSummary> {
        match self {
            RadioDevice::HackRf(h) => h.take_ended_recording(),
            RadioDevice::RtlSdr(r) => r.take_ended_recording(),
        }
    }
}

/// "12.3 MB"
fn size_str(bytes: u64) -> String {
    if bytes >= 1 << 30 {
        format!("{:.2} GB", bytes as f64 / (1u64 << 30) as f64)
    } else {
        format!("{:.1} MB", bytes as f64 / (1u64 << 20) as f64)
    }
}

fn recording_summary_text(prefix: &str, s: &RecordingSummary) -> String {
    let name = s.path.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
    let mut msg = format!("{prefix}: {name} ({})", size_str(s.bytes));
    if s.dropped_buffers > 0 {
        msg.push_str(&format!(", {} buffers dropped (disk too slow)", s.dropped_buffers));
    }
    if let Some(e) = &s.error {
        msg.push_str(&format!(", error: {e}"));
    }
    msg
}

impl App {
    /// The hop plan in use: set by `:hop`, else `[radio] hop_frequencies` / `hop_dwell_ms`.
    pub fn hop_plan(&self) -> HopPlan {
        self.rx.hop_plan.clone().unwrap_or_else(|| {
            let c = &self.storage.config;
            HopPlan::new(c.hop_frequencies.clone(), c.hop_dwell_ms)
        })
    }

    /// `:iq <path> [freq MHz|Hz] [rate]`, `:iq stop`
    pub(super) fn cmd_iq(&mut self, args: &[&str]) -> Result<()> {
        if args.is_empty() {
            self.last_error = Some("Usage: :iq <file.cs8|.cu8> [freq MHz] [rate, e.g. 2M] | :iq stop".to_string());
            return Ok(());
        }
        if args.len() == 1 && args[0].eq_ignore_ascii_case("stop") {
            match &self.rx.iq_job {
                Some(job) => {
                    job.cancel();
                    self.status_message = Some("Stopping IQ file...".to_string());
                }
                None => self.last_error = Some("No IQ file is being processed".to_string()),
            }
            return Ok(());
        }
        // The path may contain spaces: take the longest leading run of words naming a file.
        let import_dir = self.storage.import_dir().clone();
        let export_dir = self.storage.export_dir().clone();
        let mut found = None;
        for k in (1..=args.len()).rev() {
            let cand = args[..k].join(" ");
            let cand = cand.trim_matches('"');
            if let Some(p) = find_file(cand, &[import_dir.as_path(), export_dir.as_path()]) {
                found = Some((p, k));
                break;
            }
        }
        let Some((path, used)) = found else {
            self.last_error = Some(format!("File not found: {}", args.join(" ")));
            return Ok(());
        };
        let rest = &args[used..];
        let freq = match rest.first() {
            Some(s) => match parse_frequency_arg(s) {
                Some(f) => Some(f),
                None => {
                    self.last_error = Some(format!("Invalid frequency: {s}"));
                    return Ok(());
                }
            },
            None => None,
        };
        let rate = match rest.get(1) {
            Some(s) => match parse_rate_arg(s) {
                Some(r) => Some(r),
                None => {
                    self.last_error = Some(format!("Invalid sample rate: {s}"));
                    return Ok(());
                }
            },
            None => None,
        };
        self.start_iq_file(path, freq, rate);
        Ok(())
    }

    /// Process an IQ file in the background (`:iq`, `kat --iq`, the `:load` browser). Its captures
    /// arrive like live ones (decode, de-duplication, research mode); progress shows in the status bar.
    pub fn start_iq_file(&mut self, path: PathBuf, freq: Option<u32>, rate: Option<u32>) {
        self.last_error = None;
        if self.rx.iq_job.as_ref().is_some_and(|j| !j.is_finished()) {
            self.last_error = Some("An IQ file is already being processed (:iq stop)".to_string());
            return;
        }
        let spec = match IqFileSpec::resolve(path, freq, rate, self.frequency) {
            Ok(s) => s,
            Err(e) => {
                self.last_error = Some(e);
                return;
            }
        };
        let name = spec.path.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
        let desc = format!(
            "{} ({} MHz, {:.3} MS/s, {})",
            name,
            format_mhz(spec.target_hz),
            spec.sample_rate as f64 / 1e6,
            spec.format.extension()
        );
        match IqFileJob::spawn(spec, self.radio_event_tx.clone()) {
            Ok(job) => {
                self.rx.iq_captures_before = self.captures.len();
                self.rx.iq_job = Some(job);
                self.status_message = Some(format!("IQ file: {desc}"));
                tracing::info!("Processing IQ file {desc}");
            }
            Err(e) => self.last_error = Some(e),
        }
    }

    /// `:record [path]`, `:record stop`
    pub(super) fn cmd_record(&mut self, args: &[&str]) -> Result<()> {
        if args.first().is_some_and(|a| a.eq_ignore_ascii_case("stop")) {
            match self.radio.as_mut().and_then(|r| r.stop_recording()) {
                Some(sum) => self.status_message = Some(recording_summary_text("Recording saved", &sum)),
                None => self.last_error = Some("Not recording".to_string()),
            }
            return Ok(());
        }
        if self.radio.is_none() {
            self.last_error = Some("No radio device connected".to_string());
            return Ok(());
        }
        if self.rx.hop_enabled {
            self.last_error = Some("Stop hopping first (:hop off): a recording has one centre frequency".to_string());
            return Ok(());
        }
        if let Some(st) = self.radio.as_ref().and_then(|r| r.recording_status()) {
            self.last_error = Some(format!("Already recording to {} (:record stop)", st.path.display()));
            return Ok(());
        }
        if self.radio_state == RadioState::Idle {
            self.toggle_receiving()?;
        }
        if self.radio_state != RadioState::Receiving {
            if self.last_error.is_none() {
                self.last_error = Some("Receiver is not running".to_string());
            }
            return Ok(());
        }
        let Some(radio) = self.radio.as_mut() else {
            return Ok(());
        };
        let (format, rate) = (radio.iq_format(), radio.sample_rate());
        let target = self.frequency;
        let dir = self.storage.export_dir().clone();
        let path = match args.first() {
            Some(_) => resolve_record_path(&dir, &args.join(" "), format),
            None => dir.join(recording_file_name(lo_for(target, rate), rate, target, format)),
        };
        match radio.start_recording(path.clone()) {
            Ok(()) => {
                self.status_message = Some(format!(
                    "Recording raw IQ to {} (:record stop)",
                    path.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default()
                ));
                tracing::info!("Recording IQ to {:?}", path);
            }
            Err(e) => self.last_error = Some(format!("Cannot record to {}: {e}", path.display())),
        }
        Ok(())
    }

    /// `:hop`, `:hop on|off`, `:hop <MHz> <MHz> ...`, `:hop dwell <ms>`, `:hop default`
    pub(super) fn cmd_hop(&mut self, args: &[&str]) -> Result<()> {
        match args {
            [] => self.set_hopping(!self.rx.hop_enabled),
            [a] if a.eq_ignore_ascii_case("on") => self.set_hopping(true),
            [a] if a.eq_ignore_ascii_case("off") => self.set_hopping(false),
            [a] if a.eq_ignore_ascii_case("default") => {
                self.rx.hop_plan = None;
                self.set_hopping(true);
            }
            [a, ms] if a.eq_ignore_ascii_case("dwell") => match ms.parse::<u32>() {
                Ok(v) => {
                    let plan = HopPlan::new(self.hop_plan().frequencies, v);
                    let dwell = plan.dwell_ms;
                    self.rx.hop_plan = Some(plan);
                    if self.rx.hop_enabled {
                        self.set_hopping(true);
                    } else {
                        self.status_message = Some(format!("Hop dwell {dwell} ms"));
                    }
                }
                Err(_) => self.last_error = Some("Usage: :hop dwell <ms>".to_string()),
            },
            list => match parse_frequency_list(&list.join(" ")) {
                Ok(f) if f.len() >= 2 => {
                    self.rx.hop_plan = Some(HopPlan::new(f, self.hop_plan().dwell_ms));
                    self.set_hopping(true);
                }
                Ok(_) => self.last_error = Some("Give at least two frequencies to hop between".to_string()),
                Err(e) => self.last_error = Some(format!("{e}. Usage: :hop on|off | :hop <MHz> <MHz> ... | :hop dwell <ms>")),
            },
        }
        Ok(())
    }

    /// Switch frequency hopping on or off (`:hop`, the Hop setting).
    pub fn set_hopping(&mut self, on: bool) {
        let plan = self.hop_plan();
        let Some(radio) = self.radio.as_mut() else {
            self.last_error = Some("No radio device connected".to_string());
            return;
        };
        if on {
            if radio.recording_status().is_some() {
                self.last_error = Some("Stop recording first (:record stop)".to_string());
                return;
            }
            if plan.frequencies.len() < 2 {
                self.last_error = Some("The hop list needs at least two frequencies (:hop <MHz> <MHz> ...)".to_string());
                return;
            }
            let desc = plan.describe();
            radio.set_hopping(Some(plan));
            self.rx.hop_enabled = true;
            self.rx.hop_home = self.frequency;
            let when = if self.radio_state == RadioState::Receiving { "" } else { " (starts with RX: r)" };
            self.status_message = Some(format!("Hopping: {desc}{when}"));
        } else {
            radio.set_hopping(None);
            self.rx.hop_enabled = false;
            self.status_message = Some(format!("Hopping off, back on {} MHz", format_mhz(self.frequency)));
        }
    }

    /// Background receive work, called every main-loop tick: IQ file progress/result, recordings
    /// that ended with RX, hopping off after a manual `:freq`, and ProtoPirate's pause after a decode.
    pub fn poll_rx(&mut self) {
        if self.rx.iq_job.as_ref().is_some_and(|j| j.is_finished()) {
            let job = self.rx.iq_job.take().expect("finished job");
            let name = job.spec.path.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
            match job.join() {
                Ok(sum) => {
                    // The job's last captures may still be queued: take them in before counting.
                    let _ = self.process_radio_events();
                    let added = self.captures.len().saturating_sub(self.rx.iq_captures_before);
                    let verb = if sum.cancelled { "stopped" } else { "done" };
                    self.status_message = Some(format!(
                        "IQ file {verb}: {name}: {} burst(s), {} capture(s), {added} added to the list ({:.1} s of IQ in {:.1} s)",
                        sum.bursts,
                        sum.captures,
                        sum.seconds,
                        sum.elapsed.as_secs_f64()
                    ));
                }
                Err(e) => self.last_error = Some(format!("IQ file {name}: {e}")),
            }
        }
        if let Some(sum) = self.radio.as_mut().and_then(|r| r.take_ended_recording()) {
            self.status_message = Some(recording_summary_text("Recording stopped with RX", &sum));
        }
        if self.rx.hop_enabled && self.frequency != self.rx.hop_home {
            // `:freq` (or the Freq setting) picked one frequency: stop hopping.
            if let Some(radio) = self.radio.as_mut() {
                radio.set_hopping(None);
            }
            self.rx.hop_enabled = false;
            self.status_message = Some(format!("Hopping off: listening on {} MHz", format_mhz(self.frequency)));
        }
        if self.captures.len() < self.rx.seen_captures {
            self.rx.seen_captures = self.captures.len();
        }
        if self.rx.hop_enabled && self.radio_state == RadioState::Receiving {
            let decoded = self.captures[self.rx.seen_captures..].iter().any(|c| c.protocol.is_some());
            if decoded {
                if let Some(radio) = self.radio.as_mut() {
                    radio.hop_pause(DECODE_PAUSE_DWELLS as u32);
                }
            }
        }
        self.rx.seen_captures = self.captures.len();
    }

    /// Hopping, recording and IQ-file state for the header.
    pub fn rx_header(&self) -> RxHeader {
        let hop = if self.rx.hop_enabled {
            self.radio.as_ref().and_then(|r| {
                let (plan, idx) = r.hop_state()?;
                let f = if self.radio_state == RadioState::Receiving { r.rx_frequency() } else { plan.frequencies[0] };
                Some((f, idx, plan.frequencies.len()))
            })
        } else {
            None
        };
        RxHeader {
            hop,
            recording: self.radio.as_ref().and_then(|r| r.recording_status()),
            iq_progress: self.rx.iq_job.as_ref().map(|j| j.progress()),
        }
    }

    /// Bursts found so far by the running IQ file (status bar).
    pub fn iq_job_bursts(&self) -> Option<u32> {
        self.rx.iq_job.as_ref().map(|j| j.bursts())
    }
}
