//! "Recover key (KeeLoq magic serial)" — Flipper-ARF's KeeLoq Decrypt ("BF2"): find the
//! manufacturer key of a KeeLoq remote KAT has no key for, from two or more captures of it
//! ([`crate::protocols::keeloq_bf`], on all cores), save it as ARF does (`BF_<serial>` in the user
//! keystore) and decode the remote's captures with it.

use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::Instant;

use super::App;
use crate::capture::{Capture, CaptureStatus};
use crate::protocols::keeloq_bf::{self as bf, Control, Frame, Outcome};
use crate::protocols::keeloq_common::{add_keeloq_key, MfKey};

/// A running KeeLoq key search.
pub(super) struct KeeloqBfJob {
    pub(super) capture_id: u32,
    serial: u32,
    learnings: Vec<u32>,
    captures: usize,
    pub(super) ctl: Arc<Control>,
    handle: std::thread::JoinHandle<Outcome>,
    started: Instant,
}

/// A 64-bit KeeLoq frame (identified or not).
fn is_keeloq(c: &Capture) -> bool {
    c.data_count_bit == 64 && c.protocol.as_deref().is_some_and(|p| p.starts_with("KeeLoq"))
}

/// "6/7" for the status line.
fn types_label(learnings: &[u32]) -> String {
    learnings.iter().map(u32::to_string).collect::<Vec<_>>().join("/")
}

/// "12 s", "4 min", "1 h 05 min".
fn duration_label(secs: f64) -> String {
    let s = secs.max(0.0).round() as u64;
    match s {
        0..=99 => format!("{s} s"),
        100..=5999 => format!("{} min", (s + 30) / 60),
        _ => format!("{} h {:02} min", s / 3600, (s % 3600) / 60),
    }
}

impl App {
    /// Whether the signal menu offers the search for this capture: a KeeLoq frame no key decrypts.
    pub(super) fn keeloq_bf_offered(c: &Capture) -> bool {
        c.data_count_bit == 64 && c.protocol.as_deref() == Some(bf::UNKNOWN_NAME)
    }

    /// The captured frames of remote `serial`, one per hop code, oldest first.
    fn keeloq_frames_of(&self, serial: u32) -> Vec<Frame> {
        let mut captures: Vec<&Capture> =
            self.captures.iter().filter(|c| is_keeloq(c) && c.serial == Some(serial)).collect();
        captures.sort_by_key(|c| c.id);
        let mut frames: Vec<Frame> = Vec::new();
        for f in captures.into_iter().map(|c| Frame::from_data(c.data)) {
            if f.serial() == serial && !frames.iter().any(|g| g.hop == f.hop) {
                frames.push(f);
            }
        }
        frames
    }

    /// Start the search for the remote of capture `id` (`types`: learning types; Auto = 6, 7).
    pub(super) fn start_keeloq_key_recovery(&mut self, id: u32, types: Option<Vec<u32>>) {
        if self.psa_recover_job.is_some() || self.keeloq_bf_job.is_some() {
            self.last_error = Some("A key recovery is already running".to_string());
            return;
        }
        let Some(capture) = self.captures.iter().find(|c| c.id == id) else {
            self.last_error = Some(format!("Capture {id} not found"));
            return;
        };
        if !is_keeloq(capture) {
            self.last_error = Some(format!("Capture {id} is not a KeeLoq frame"));
            return;
        }
        let serial = Frame::from_data(capture.data).serial();
        let frames = self.keeloq_frames_of(serial);
        let learnings = types.unwrap_or_else(|| bf::AUTO_TYPES.to_vec());
        if let Err(e) = bf::check_input(&frames, &learnings) {
            self.last_error = Some(format!("Recover key of remote {serial:07X}: {e}"));
            return;
        }
        let ctl = Arc::new(Control::default());
        let worker = Arc::clone(&ctl);
        let (job_frames, job_types) = (frames.clone(), learnings.clone());
        let handle = std::thread::spawn(move || bf::search(&job_frames, &job_types, &worker));
        self.status_message = Some(format!(
            "Recovering the KeeLoq key of remote {serial:07X} from {} captures (type {})…",
            frames.len(),
            types_label(&learnings)
        ));
        self.keeloq_bf_job = Some(KeeloqBfJob {
            capture_id: id,
            serial,
            learnings,
            captures: frames.len(),
            ctl,
            handle,
            started: Instant::now(),
        });
    }

    /// Ask a running search to stop; false when none runs.
    pub(super) fn cancel_keeloq_key_recovery(&mut self) -> bool {
        let Some(job) = &self.keeloq_bf_job else {
            return false;
        };
        job.ctl.cancel.store(true, Ordering::Relaxed);
        self.status_message = Some("Cancelling key recovery…".to_string());
        true
    }

    /// Progress of a running search, or its result once it has finished (every tick).
    pub(super) fn poll_keeloq_key_recovery(&mut self) {
        let Some(job) = &self.keeloq_bf_job else {
            return;
        };
        if !job.handle.is_finished() {
            let tested = job.ctl.tested.load(Ordering::Relaxed);
            let total = job.ctl.total.load(Ordering::Relaxed).max(1);
            let rate = tested as f64 / job.started.elapsed().as_secs_f64().max(0.001);
            let left = if rate > 0.0 { duration_label(total.saturating_sub(tested) as f64 / rate) } else { "?".to_string() };
            self.status_message = Some(format!(
                "Recovering the KeeLoq key of remote {:07X} (type {})… {}%, {:.0} M keys/s, {} left",
                job.serial,
                types_label(&job.learnings),
                (tested * 100 / total).min(99),
                rate / 1e6,
                left
            ));
            return;
        }
        let Some(job) = self.keeloq_bf_job.take() else {
            return;
        };
        let secs = job.started.elapsed().as_secs_f64().max(0.001);
        match job.handle.join() {
            Ok(Outcome::Found(found)) => self.apply_recovered_key(job.serial, &found),
            Ok(Outcome::NotFound) => {
                let rate = job.ctl.tested.load(Ordering::Relaxed) as f64 / secs;
                let mut msg = format!("No magic-serial key fits remote {:07X} (type {})", job.serial, types_label(&job.learnings));
                if !job.learnings.contains(&8) {
                    if job.captures >= 3 && rate > 0.0 {
                        msg += &format!(
                            "; type 8 searches 2^40 keys, about {} here: :klbf {} 8",
                            duration_label(bf::space(8) as f64 / rate),
                            job.capture_id
                        );
                    } else {
                        msg += "; type 8 needs three captures of the remote";
                    }
                }
                self.last_error = Some(msg);
            }
            Ok(Outcome::Cancelled) => {
                self.status_message = Some(format!("Key recovery for remote {:07X} cancelled", job.serial));
            }
            Err(_) => self.last_error = Some("Key recovery thread failed".to_string()),
        }
    }

    /// Use the key with the closest counters: add it to this session and to the user keystore as
    /// `BF_<serial>` (Flipper-ARF's name), and decode the remote's captures with it.
    fn apply_recovered_key(&mut self, serial: u32, found: &[bf::Candidate]) {
        let best = &found[0];
        let name = format!("BF_{serial:07X}");
        add_keeloq_key(MfKey { name: name.clone(), learning: best.learning, key: best.man_key });
        let saved = match crate::keystore::append_user_key(self.storage.config_dir(), &name, best.learning, best.man_key) {
            Ok(path) => format!("saved as {name} in {}", path.display()),
            Err(e) => format!("not saved ({e}), usable until KAT exits"),
        };
        let mut decoded = 0;
        for c in self.captures.iter_mut().filter(|c| is_keeloq(c) && c.serial == Some(serial)) {
            let Some(d) = bf::decode_frame(c.data).filter(|d| d.crc_valid) else {
                continue;
            };
            c.protocol = d.protocol_display_name;
            c.serial = d.serial;
            c.button = d.button;
            c.counter = d.counter;
            c.crc_valid = true;
            c.status = if d.encoder_capable { CaptureStatus::EncoderCapable } else { CaptureStatus::Decoded };
            decoded += 1;
        }
        let also = if found.len() > 1 {
            format!(" ({} keys fit; the one with the closest counters is used)", found.len())
        } else {
            String::new()
        };
        self.status_message = Some(format!(
            "KeeLoq key of remote {serial:07X} recovered (type {}, {:016X}): {saved}; {decoded} capture(s) decoded{also}",
            best.learning, best.man_key
        ));
    }

    /// `:klbf [ID] [auto|6|7|8]` (default: the selected capture, Auto) or `:klbf stop`.
    pub(super) fn klbf_command(&mut self, args: &[&str]) {
        match args.first().map(|a| a.to_ascii_lowercase()).as_deref() {
            Some("stop") | Some("cancel") => {
                if !self.cancel_keeloq_key_recovery() {
                    self.last_error = Some("No KeeLoq key recovery is running".to_string());
                }
                return;
            }
            Some("auto") | None => {
                let Some(id) = self.selected_capture.and_then(|i| self.captures.get(i)).map(|c| c.id) else {
                    self.last_error = Some("Usage: :klbf [ID] [auto|6|7|8] — select a KeeLoq capture or give its ID".to_string());
                    return;
                };
                return self.start_keeloq_key_recovery(id, None);
            }
            Some(_) => {}
        }
        let Ok(id) = args[0].parse::<u32>() else {
            self.last_error = Some("Usage: :klbf [ID] [auto|6|7|8] | :klbf stop".to_string());
            return;
        };
        let types = match args.get(1).map(|t| crate::headless::parse_bf_types(t)) {
            None => None,
            Some(Ok(t)) => Some(t),
            Some(Err(e)) => {
                self.last_error = Some(e);
                return;
            }
        };
        self.start_keeloq_key_recovery(id, types);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn labels() {
        assert_eq!(types_label(&[6, 7]), "6/7");
        assert_eq!(duration_label(4.6), "5 s");
        assert_eq!(duration_label(1202.0), "20 min");
        assert_eq!(duration_label(7300.0), "2 h 01 min");
    }
}
