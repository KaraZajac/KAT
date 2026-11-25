//! "Recover key (JLR)" — the signal-menu action that recovers a Ford V1 (JLR) fob's car key from two
//! or more captured presses. The key is only 32-bit effective (see `docs/ford_v1_jlr.md` and
//! [`crate::protocols::jlr`]), so an exhaustive search of the $2^{32}$ key space finishes in about a
//! minute on a desktop CPU. Mirrors the HiTag2 search ([`super::hitag2_bf`]) as a background job with
//! progress and cancellation, but the recovered key is a vehicle secret, so it is saved and used but
//! never shown. On success the fob's captures are re-deciphered (button, counter, full ID) and become
//! sendable.

use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::Instant;

use super::keeloq_bf::duration_label;
use super::App;
use crate::capture::{Capture, CaptureStatus, KeyRecovery};
use crate::protocols::jlr::{self, RecoverControl, RECOVER_TOTAL};

/// The label stored on a capture's `key_recovery`.
const LABEL: &str = "JLR 32-bit key";

/// A running "Recover key (JLR)" search.
pub(super) struct JlrBfJob {
    /// The fob's captures (all share the folded ID); marked recovered/not-found when it ends.
    pub(super) capture_ids: Vec<u32>,
    /// The fob's folded ID (`data` low 16 bits); the full ID comes out of the recovered key.
    pub(super) folded: u16,
    pub(super) ctl: Arc<RecoverControl>,
    pub(super) handle: std::thread::JoinHandle<Vec<u32>>,
    pub(super) started: Instant,
}

fn is_jlr(c: &Capture) -> bool {
    c.protocol.as_deref() == Some(crate::protocols::ford_v1_jlr_name())
}

/// A capture's press as the recovery takes it: (folded ID, 8-byte enciphered payload).
fn press_of(c: &Capture) -> Option<(u16, [u8; 8])> {
    Some((c.data as u16, c.data_extra?.to_be_bytes()))
}

/// The distinct presses of the fob with folded ID `folded` among `captures`.
fn fob_presses(captures: &[Capture], folded: u16) -> Vec<(u16, [u8; 8])> {
    let mut out: Vec<(u16, [u8; 8])> = Vec::new();
    for c in captures.iter().filter(|c| is_jlr(c)) {
        if let Some(p) = press_of(c) {
            if p.0 == folded && !out.contains(&p) {
                out.push(p);
            }
        }
    }
    out
}

/// Whether "Recover key (JLR)" is offered for `c`: a Ford V1 (JLR) capture whose key is not known yet
/// (not already deciphered/sendable), with two or more distinct presses of its fob in `captures`.
fn offered(captures: &[Capture], c: &Capture) -> bool {
    is_jlr(c) && c.status != CaptureStatus::EncoderCapable && fob_presses(captures, c.data as u16).len() >= 2
}

impl App {
    /// Whether the signal menu offers "Recover key (JLR)" for `c`: a Ford V1 (JLR) capture whose key
    /// is not known yet (not already deciphered/sendable) and whose fob has two or more distinct
    /// presses in the session (needed to pin the 32-bit key uniquely).
    pub(super) fn jlr_bf_offered(&self, c: &Capture) -> bool {
        offered(&self.captures, c)
    }

    /// Start the $2^{32}$ search for the fob of capture `id` on a background thread.
    pub(super) fn start_jlr_key_recovery(&mut self, id: u32) {
        if self.long_key_search_runs() {
            self.last_error = Some("A key recovery is already running".to_string());
            return;
        }
        let Some(capture) = self.captures.iter().find(|c| c.id == id) else {
            self.last_error = Some(format!("Capture #{id:02} not found"));
            return;
        };
        if !is_jlr(capture) {
            self.last_error = Some("Recover key (JLR): not a Ford V1 (JLR) capture".to_string());
            return;
        }
        let folded = capture.data as u16;
        let presses = fob_presses(&self.captures, folded);
        if presses.len() < 2 {
            self.last_error = Some(
                "Recover key (JLR) needs two different presses of the fob: capture it once more (any button) and try again"
                    .to_string(),
            );
            return;
        }
        let capture_ids: Vec<u32> = self.captures.iter().filter(|c| is_jlr(c) && c.data as u16 == folded).map(|c| c.id).collect();
        let ctl = Arc::new(RecoverControl::default());
        let worker = Arc::clone(&ctl);
        let handle = std::thread::spawn(move || jlr::recover_with(&presses, &worker));
        self.status_message = Some(format!("Recovering the JLR car key of fob {folded:04X} (32-bit key search)…"));
        self.jlr_bf_job = Some(JlrBfJob { capture_ids, folded, ctl, handle, started: Instant::now() });
    }

    /// Ask a running JLR search to stop; false when none runs.
    pub(super) fn cancel_jlr_key_recovery(&mut self) -> bool {
        let Some(job) = &self.jlr_bf_job else {
            return false;
        };
        job.ctl.cancel.store(true, Ordering::Relaxed);
        self.status_message = Some("Cancelling key recovery…".to_string());
        true
    }

    /// Progress of a running JLR search, for [`App::job_status`].
    pub(super) fn jlr_key_recovery_progress(&self) -> Option<String> {
        let job = self.jlr_bf_job.as_ref().filter(|j| !j.handle.is_finished())?;
        let done = job.ctl.tested.load(Ordering::Relaxed).min(RECOVER_TOTAL);
        let rate = done as f64 / job.started.elapsed().as_secs_f64().max(0.001);
        let left = if done > 0 { duration_label((RECOVER_TOTAL - done) as f64 / rate) } else { "?".to_string() };
        Some(format!(
            "Recovering the JLR car key of fob {:04X}… {}%, at most {left} left{}",
            job.folded,
            (done * 100 / RECOVER_TOTAL).min(99),
            super::keeloq_bf::gpu_note()
        ))
    }

    /// Finish a JLR search once its thread has ended.
    pub(super) fn poll_jlr_key_recovery(&mut self) {
        if self.jlr_bf_job.as_ref().is_some_and(|j| j.handle.is_finished()) {
            if let Some(job) = self.jlr_bf_job.take() {
                self.finish_jlr_key_recovery(job);
            }
        }
    }

    fn finish_jlr_key_recovery(&mut self, job: JlrBfJob) {
        let took = duration_label(job.started.elapsed().as_secs_f64());
        match job.handle.join() {
            Ok(found) if found.len() == 1 => self.apply_jlr_key(job.folded, found[0], &took),
            Ok(found) if found.is_empty() => {
                if job.ctl.cancel.load(Ordering::Relaxed) {
                    self.status_message = Some(format!("Key recovery for fob {:04X} cancelled", job.folded));
                } else {
                    for c in self.captures.iter_mut().filter(|c| job.capture_ids.contains(&c.id)) {
                        c.key_recovery = Some(KeyRecovery::NotFound(LABEL.to_string()));
                    }
                    self.last_error = Some(format!(
                        "No 32-bit JLR key fits the presses of fob {:04X} (searched in {took}); the presses may be from more than one fob",
                        job.folded
                    ));
                }
            }
            Ok(found) => {
                self.last_error = Some(format!(
                    "{} candidate keys fit fob {:04X}; capture another press of it and try again",
                    found.len(),
                    job.folded
                ));
            }
            Err(_) => self.last_error = Some("Key recovery thread failed".to_string()),
        }
    }

    /// Save the recovered key (in-memory keystore and `jlr_keys`), re-decipher the fob's captures so
    /// they show the button, counter and full ID and can send, and report it. The key bytes are a
    /// vehicle secret and are never shown.
    fn apply_jlr_key(&mut self, folded: u16, x: u32, took: &str) {
        let key = jlr::candidate_key(x);
        let name = format!("fob {folded:04X}");
        crate::protocols::keys::get_keystore_mut().add_jlr_key(name.clone(), key);
        let saved = match crate::keystore::append_jlr_key(self.storage.config_dir(), &name, &key) {
            Ok(path) => format!("saved in {}", path.display()),
            Err(e) => format!("not saved ({e}), usable until KAT exits"),
        };
        let mut decoded = 0;
        let mut latest: Option<jlr::Press> = None;
        for c in self.captures.iter_mut().filter(|c| is_jlr(c) && c.data as u16 == folded) {
            let Some((f, payload)) = press_of(c) else {
                continue;
            };
            let Some(p) = jlr::try_decipher(&payload, f, &key) else {
                continue;
            };
            c.serial = Some(p.id);
            c.button = Some(p.button);
            c.counter = Some(p.counter as u16);
            c.status = CaptureStatus::EncoderCapable;
            c.key_recovery = Some(KeyRecovery::Recovered(LABEL.to_string()));
            // Remember the fob by its highest-counter press, so "Send next code" is ahead of every
            // captured press (the counter only moves forward).
            if latest.is_none_or(|b| p.counter > b.counter) {
                latest = Some(p);
            }
            decoded += 1;
        }
        if let Some(p) = latest {
            crate::protocols::keys::get_keystore_mut().remember_jlr_fob(p.id, key, p.counter, p.button);
        }
        let id_txt = latest.map_or_else(|| format!("{folded:04X}"), |p| format!("{:08X}", p.id));
        self.status_message = Some(format!(
            "JLR car key of fob {id_txt} recovered in {took} ({saved}); {decoded} capture{} can now decode and send (key not shown — vehicle secret)",
            crate::plural(decoded)
        ));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::jlr::{encipher, fold_id, plain_bytes, Press};

    fn jlr_capture(id: u32, press: Press, key: &[u8; 16]) -> Capture {
        let payload = encipher(&plain_bytes(press), key);
        let folded = fold_id(press.id);
        let data = ((0xD3950E26u64) << 24) | ((0x00u64) << 16) | u64::from(folded); // header|flags|folded, 7 bytes
        let mut c = Capture::from_pairs(id, 433_920_000, Vec::new());
        c.protocol = Some(crate::protocols::ford_v1_jlr_name().to_string());
        c.data = data;
        c.data_count_bit = 136;
        c.data_extra = Some(u64::from_be_bytes(payload));
        c.serial = Some(u32::from(folded));
        c.button = Some(0);
        c.counter = Some(0);
        c.crc_valid = true;
        c.status = CaptureStatus::Decoded;
        c
    }

    /// `fob_presses` gathers the distinct presses of one fob; a single repeat does not count twice.
    #[test]
    fn fob_presses_are_distinct_per_fob() {
        let key = [7u8; 16];
        let id = 0x1234_5678;
        let folded = fold_id(id);
        let p1 = jlr_capture(1, Press { button: 1, counter: 0x100, id }, &key);
        let p1_repeat = jlr_capture(2, Press { button: 1, counter: 0x100, id }, &key); // same payload
        let p2 = jlr_capture(3, Press { button: 2, counter: 0x102, id }, &key);
        let other = jlr_capture(4, Press { button: 1, counter: 0x200, id: 0xAAAA_BBBB }, &key);
        let caps = vec![p1, p1_repeat, p2, other];
        assert_eq!(fob_presses(&caps, folded).len(), 2, "two distinct presses of the fob (the repeat is one)");
        assert_eq!(fob_presses(&caps, fold_id(0xAAAA_BBBB)).len(), 1);
    }

    /// The menu offers "Recover key (JLR)" for a two-press fob with no key yet, and stops once the
    /// key is known (its captures deciphered → EncoderCapable) or there is only one press.
    #[test]
    fn offered_only_with_two_presses_and_no_key() {
        let key = [0x5Au8; 16];
        let id = 0x2F47_AB35;
        let caps = vec![
            jlr_capture(1, Press { button: 1, counter: 0x10, id }, &key),
            jlr_capture(2, Press { button: 2, counter: 0x12, id }, &key),
        ];
        assert!(offered(&caps, &caps[0]), "two presses, no key: offered");

        // Only one press of a fob: not offered.
        let one = vec![jlr_capture(3, Press { button: 1, counter: 0x10, id: 0x0BAD_F00D }, &key)];
        assert!(!offered(&one, &one[0]));

        // Key known (capture already deciphered and sendable): not offered.
        let mut known = caps.clone();
        known[0].status = CaptureStatus::EncoderCapable;
        assert!(!offered(&known, &known[0]));

        // A non-JLR capture is never offered this action.
        let mut other = caps[0].clone();
        other.protocol = Some("Ford V1".to_string());
        assert!(!offered(&caps, &other));
    }
}
