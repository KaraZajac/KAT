//! Receive features on the app side: IQ files (`:iq`, `kat --iq`, `.cs8`/`.cu8` in the `:load`
//! browser), IQ recording (`:record`), frequency hopping (`:hop`, the Hop setting), and the state the
//! header and status bar show. A child module of `app` (`mod rx;`), so it can use App's fields.

use std::collections::HashMap;
use std::path::PathBuf;

use anyhow::Result;

use super::{App, RadioDevice, RadioState};
use crate::capture::{Capture, OnAir};
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
    /// With duplicates kept (`ignore_duplicates = false`): when each decoded signal (protocol,
    /// data, serial, button) was last on the air.
    recent: HashMap<(String, u64, Option<u32>, Option<u8>), OnAir>,
}

/// Repeats of one signal closer than this on the air are one press, as in Flipper-ARF's receive
/// history (`subghz_history_add_to_history`: 500 ms; Momentum uses 600 ms).
const REPEAT_WINDOW_US: u64 = 500_000;

/// A received burst (live or from an IQ file) as captures: one per distinct decode, as a RAW
/// `.sub` import gives and as ProtoPirate's receive history lists every decoded frame (a burst can
/// hold several: back-to-back presses with new counters, a fob alternating two frames). Each holds
/// the pulses of its frame and that frame's repeats, the last one also the burst's tail, so a
/// press that repeats one frame is one capture of the whole burst, on the air when its pulses were.
/// A frame that comes back later in the burst is left out (`ignore_duplicates`), or with duplicates
/// kept is a capture of its own, for [RxExtras::repeat_of_recent] to judge by its on-air time.
/// Nothing decoded: the KeeLoq keystore fallback, else one unknown capture of the burst (config
/// `show_unknown_signals`).
fn received_captures(
    protocols: &mut crate::protocols::ProtocolRegistry,
    burst: Capture,
    show_unknown: bool,
    keep_duplicates: bool,
) -> Vec<Capture> {
    use crate::capture::StoredLevelDuration;
    let pairs: Vec<crate::radio::LevelDuration> =
        burst.raw_pairs.iter().map(|p| crate::radio::LevelDuration::new(p.level, p.duration_us)).collect();
    let decodes = protocols.process_signal_stream_path(&pairs, burst.frequency, burst.received_rf);
    if decodes.is_empty() {
        let mut capture = burst;
        if let Some((name, decoded)) = protocols.process_signal_path(&pairs, capture.frequency, capture.received_rf) {
            capture.protocol = Some(name);
            super::files::set_decode(&mut capture, &decoded);
        }
        return if show_unknown || capture.protocol.is_some() { vec![capture] } else { Vec::new() };
    }
    // Consecutive decodes of the same frame (its repeats) form one group of pulses; segments are
    // contiguous from the burst's start.
    let key = |name: &str, d: &crate::protocols::DecodedSignal| (name.to_string(), d.data, d.extra, d.serial, d.button, d.counter);
    let mut groups: Vec<(String, crate::protocols::DecodedSignal, usize, usize)> = Vec::new(); // (name, decode, start, end)
    let mut at = 0;
    for (name, decoded, segment) in decodes {
        let end = at + segment.len();
        match groups.last_mut() {
            Some(g) if key(&g.0, &g.1) == key(&name, &decoded) => g.3 = end,
            _ => groups.push((name, decoded, at, end)),
        }
        at = end;
    }
    if let Some(last) = groups.last_mut() {
        last.3 = pairs.len();
    }
    // Time from the burst's start to each pair, for the groups' on-air times.
    let offsets: Vec<u64> = std::iter::once(0)
        .chain(burst.raw_pairs.iter().scan(0u64, |t, p| {
            *t += u64::from(p.duration_us);
            Some(*t)
        }))
        .collect();
    let mut seen = std::collections::HashSet::new();
    let mut out = Vec::new();
    for (name, decoded, start, end) in groups {
        if !seen.insert(key(&name, &decoded)) && !keep_duplicates {
            continue;
        }
        let mut capture = burst.clone();
        capture.raw_pairs = burst.raw_pairs[start..end]
            .iter()
            .map(|p| StoredLevelDuration { level: p.level, duration_us: p.duration_us })
            .collect();
        capture.on_air = burst.on_air.map(|t| OnAir {
            session: t.session,
            start_us: t.start_us + offsets[start],
            end_us: t.start_us + offsets[end],
        });
        capture.protocol = Some(name);
        super::files::set_decode(&mut capture, &decoded);
        out.push(capture);
    }
    out
}

impl App {
    /// [received_captures] of a burst into the list. A decoded capture equal to one in the list
    /// (protocol, data, serial, button) is skipped (config `ignore_duplicates`, the default). With
    /// duplicates kept, every press is listed, and only a repeat of a signal on the air less than
    /// 500 ms before is skipped, as a Flipper's receive history does: the same press still going,
    /// or reported twice (by the AM and FM paths, or on both sides of a long transmission's cut).
    pub(super) fn add_received_signal(&mut self, burst: Capture) {
        let show_unknown = self.storage.config.show_unknown_signals;
        let keep_duplicates = !self.storage.config.ignore_duplicates;
        let (mut added, mut duplicates) = (0, 0);
        for mut capture in received_captures(&mut self.protocols, burst, show_unknown, keep_duplicates) {
            let skip = if keep_duplicates {
                self.rx.repeat_of_recent(&capture)
            } else {
                self.capture_duplicate_of_existing(&capture)
            };
            if skip {
                duplicates += 1;
                continue;
            }
            capture.id = self.next_capture_id;
            self.next_capture_id += 1;
            // Autosaved to the session file on the next tick (app/persistence.rs).
            self.captures.push(capture);
            // Selected only when that takes nothing from the user (app/input.rs).
            self.follow_new_capture();
            added += 1;
        }
        match (added, duplicates) {
            (0, 0) => {}
            // Duplicates kept: the same press, still on the air.
            (0, _) if keep_duplicates => {}
            (0, _) => self.status_message = Some("Duplicate signal ignored".to_string()),
            (1, _) => self.status_message = Some("New signal captured".to_string()),
            (n, _) => self.status_message = Some(format!("{n} new signals captured")),
        }
    }
}

impl RxExtras {
    pub fn new() -> Self {
        Self {
            iq_job: None,
            iq_captures_before: 0,
            hop_enabled: false,
            hop_plan: None,
            hop_home: 0,
            seen_captures: 0,
            recent: HashMap::new(),
        }
    }

    /// Whether `capture` repeats a decoded signal that was on the air less than [REPEAT_WINDOW_US]
    /// before it in the same receive session; that sighting then lasts until the repeat's end.
    /// Otherwise `capture` becomes the signal's latest sighting. Unknown signals, and captures
    /// without an on-air time, never repeat.
    fn repeat_of_recent(&mut self, capture: &Capture) -> bool {
        let (Some(name), Some(t)) = (&capture.protocol, capture.on_air) else { return false };
        // Forget other sessions, and signals long gone.
        self.recent.retain(|_, s| s.session == t.session && s.end_us + 10 * REPEAT_WINDOW_US >= t.start_us);
        let key = (name.clone(), capture.data, capture.serial, capture.button);
        match self.recent.get_mut(&key) {
            Some(s) if t.start_us <= s.end_us + REPEAT_WINDOW_US => {
                s.end_us = s.end_us.max(t.end_us);
                true
            }
            _ => {
                self.recent.insert(key, t);
                false
            }
        }
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
        let default_name = recording_file_name(lo_for(target, rate), rate, target, format);
        let path = match args.first() {
            Some(_) => resolve_record_path(&dir, &args.join(" "), format, &default_name),
            None => dir.join(default_name),
        };
        // Never replace an earlier recording (`:record x` twice used to truncate the first).
        let path = crate::export::unique_path(&path);
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture::{RfModulation, StoredLevelDuration};

    /// A RAW `.sub` capture as one burst of KAT's AM path.
    fn burst(path: &str) -> Capture {
        let text = std::fs::read_to_string(path).unwrap();
        let crate::export::flipper::SubContent::Raw(sub) = crate::export::flipper::parse_sub_content(&text).unwrap() else {
            panic!("{path}: not RAW");
        };
        Capture::from_pairs_with_rf(0, sub.frequency, sub.pairs, Some(RfModulation::AM))
    }

    #[test]
    fn every_distinct_frame_of_a_burst_is_a_capture() {
        crate::protocols::keys::load_keystore_from_embedded();
        let mut reg = crate::protocols::ProtocolRegistry::new();
        // Kia V1: ten presses back to back (no gap over 1.7 ms) are ten captures, in order.
        let got = received_captures(&mut reg, burst("IMPORTS/KIA/KIA V1 N1 RAW.sub"), true, false);
        assert_eq!(got.len(), 10);
        assert!(got.iter().all(|c| c.protocol.as_deref() == Some("Kia V1") && c.serial == Some(0xB444706E)));
        assert!(got.windows(2).all(|w| w[0].counter < w[1].counter), "{:?}", got.iter().map(|c| c.counter).collect::<Vec<_>>());
        // Ford V0: three frames, each repeated; one capture each, and their pulses are the burst's.
        let b = burst("IMPORTS/FORD/3_unlock_ford.sub");
        let all = b.raw_pairs.len();
        let got = received_captures(&mut reg, b, true, false);
        assert_eq!(got.iter().map(|c| c.counter).collect::<Vec<_>>(), [Some(0x872C), Some(0x872D), Some(0x872E)]);
        assert_eq!(got.iter().map(|c| c.raw_pairs.len()).sum::<usize>(), all);
        // Nothing decodes: the whole burst as one unknown capture in research mode, else nothing.
        let noise: Vec<StoredLevelDuration> =
            (0..60).map(|i| StoredLevelDuration { level: i % 2 == 0, duration_us: 70 + 13 * (i % 7) }).collect();
        let b = Capture::from_pairs_with_rf(0, 433_920_000, noise, Some(RfModulation::AM));
        let got = received_captures(&mut reg, b.clone(), true, false);
        assert_eq!((got.len(), got[0].protocol.as_deref(), got[0].raw_pairs.len()), (1, None, 60));
        assert!(received_captures(&mut reg, b, false, false).is_empty());
    }

    /// Each capture of a burst is on the air when its own pulses were: back to back from the
    /// burst's start to its end.
    #[test]
    fn captures_carry_their_frames_on_air_time() {
        crate::protocols::keys::load_keystore_from_embedded();
        let mut reg = crate::protocols::ProtocolRegistry::new();
        let mut b = burst("IMPORTS/FORD/3_unlock_ford.sub");
        let total: u64 = b.raw_pairs.iter().map(|p| u64::from(p.duration_us)).sum();
        b.on_air = Some(OnAir { session: 7, start_us: 1_000_000, end_us: 1_000_000 + total });
        let got = received_captures(&mut reg, b, true, false);
        let times: Vec<OnAir> = got.iter().map(|c| c.on_air.unwrap()).collect();
        assert_eq!(times.len(), 3);
        assert_eq!((times[0].session, times[0].start_us, times[2].end_us), (7, 1_000_000, 1_000_000 + total));
        assert!(times.windows(2).all(|w| w[0].end_us == w[1].start_us && w[0].start_us < w[0].end_us), "{times:?}");
    }

    /// Two rounds of a four-button remote in one burst (`princeton_raw.sub` twice, 2.1 s apart):
    /// four captures with duplicates ignored; with them kept, eight (each button came back 2.1 s
    /// later: a new press), while the candidates of one press (Princeton's decodes differ in the
    /// measured TE) are still one capture.
    #[test]
    fn kept_duplicates_list_each_press_of_a_long_burst() {
        let path = "REFERENCES/Flipper-ARF/applications/debug/unit_tests/resources/unit_tests/subghz/princeton_raw.sub";
        if !std::path::Path::new(path).exists() {
            eprintln!("Skip: {path} not found (the references are not checked out)");
            return;
        }
        let mut reg = crate::protocols::ProtocolRegistry::new();
        let mut b = burst(path);
        let once = b.raw_pairs.clone();
        b.raw_pairs.extend(once);
        let total: u64 = b.raw_pairs.iter().map(|p| u64::from(p.duration_us)).sum();
        b.on_air = Some(OnAir { session: 3, start_us: 0, end_us: total });
        let buttons = |got: &[Capture]| got.iter().map(|c| c.button.unwrap()).collect::<Vec<_>>();
        // As add_received_signal filters them: against the list, or by the 500 ms rule.
        let mut listed: Vec<Capture> = Vec::new();
        for c in received_captures(&mut reg, b.clone(), true, false) {
            if !super::super::files::is_duplicate(&listed, &c) {
                listed.push(c);
            }
        }
        assert_eq!(buttons(&listed), [0xF3, 0xFC, 0x30, 0xC0]);
        let mut rx = RxExtras::new();
        let kept: Vec<Capture> = received_captures(&mut reg, b, true, true).into_iter().filter(|c| !rx.repeat_of_recent(c)).collect();
        assert_eq!(buttons(&kept), [0xF3, 0xFC, 0x30, 0xC0, 0xF3, 0xFC, 0x30, 0xC0]);
    }

    /// With duplicates kept, a signal repeats only within 500 ms of its last sighting in the same
    /// session (a press still on the air, the AM and FM paths, the overlap at a cut); a later press,
    /// another session, unknown signals and captures without an on-air time are new.
    #[test]
    fn repeats_within_half_a_second_are_one_press() {
        let mut rx = RxExtras::new();
        let at = |session: u64, start_ms: u64, end_ms: u64| {
            let mut c = Capture::from_pairs_with_rf(0, 433_920_000, Vec::new(), Some(RfModulation::AM));
            c.protocol = Some("Princeton".to_string());
            c.data = 0x95D5D4;
            c.serial = Some(0x95D5D);
            c.button = Some(4);
            c.on_air = Some(OnAir { session, start_us: start_ms * 1000, end_us: end_ms * 1000 });
            c
        };
        assert!(!rx.repeat_of_recent(&at(1, 0, 100)), "first sighting");
        assert!(rx.repeat_of_recent(&at(1, 0, 100)), "the same burst on the FM path");
        assert!(rx.repeat_of_recent(&at(1, 450, 550)), "350 ms after it ended: still that press");
        assert!(rx.repeat_of_recent(&at(1, 1000, 1100)), "450 ms after the extended sighting");
        assert!(rx.repeat_of_recent(&at(1, 900, 1200)), "overlapping (a transmission cut at 3 s)");
        assert!(!rx.repeat_of_recent(&at(1, 1800, 1900)), "600 ms later: a new press");
        assert!(!rx.repeat_of_recent(&at(2, 1850, 1950)), "another session");
        let mut other = at(2, 1900, 2000);
        other.button = Some(8);
        assert!(!rx.repeat_of_recent(&other), "another button");
        let mut unknown = at(2, 1950, 2050);
        unknown.protocol = None;
        assert!(!rx.repeat_of_recent(&unknown) && !rx.repeat_of_recent(&unknown), "unknown signals never repeat");
        let mut untimed = at(2, 0, 0);
        untimed.on_air = None;
        assert!(!rx.repeat_of_recent(&untimed) && !rx.repeat_of_recent(&untimed), "no on-air time");
    }
}
