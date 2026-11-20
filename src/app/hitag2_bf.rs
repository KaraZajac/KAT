//! "Recover key (HiTag2)" — qDoctor's HiTag2 Hell key recovery
//! ([`crate::protocols::hitag2_hell`]: the attack of Verstegen, Verdult and Bokslag, on all cores)
//! for a Fiat V1, Renault V1, Fiat V2 or Hitag2 RKE remote KAT has no key for. It takes two captures
//! of the remote (two presses, any buttons): one frame fits about 65,000 keys, the second singles out
//! the remote's. The key is saved for the remote as Flipper-ARF and doctor-firmware save theirs
//! (`hitag2_keys.txt`, [`crate::protocols::hitag2_keys`]), and a Fiat or Renault remote's captures
//! then send fresh codes instead of replaying the captured frame. KAT does not transmit Hitag2 RKE:
//! there the key only confirms the remote's frames, and its IV is assumed to be Renault V1's (not
//! confirmed against a real remote). A search covers up to 2^19 layer-0
//! candidates, about 20 minutes on a 16-thread CPU with AVX2 (an hour without), and stops at the
//! key, so half that on average; it runs only when asked, never through `autobf`.

use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::Instant;

use super::keeloq_bf::duration_label;
use super::{App, InputMode};
use crate::capture::{Capture, KeyRecovery};
use crate::protocols::hitag2_hell::{self as hell, BruteForceControl, BruteForceOutcome, Hitag2Frame, Hitag2Remote, RecoveredKey};
use crate::protocols::hitag2_keys::{self, KeyOrder, ShownKey};

/// How the detail panel names the search ("Recovery: key recovered (HiTag2 Hell)").
pub(super) const LABEL: &str = "HiTag2 Hell";

/// [`App::hitag2_bf_offered`]'s last answer: (capture id, data, extra, key database version) and
/// whether the menu offers the search.
pub(super) type OfferMemo = ((u32, u64, Option<u64>, u64), bool);

/// [`App::hitag2_shown_key`]'s last answer, keyed as [`OfferMemo`]: the capture's key, if KAT has
/// one.
pub(super) type ShownKeyMemo = ((u32, u64, Option<u64>, u64), Option<ShownKey>);

/// What the overlay of a found key shows (ui/hitag2_key_overlay.rs): the key in the byte orders
/// the other tools take it in, which the status line has no room to tell apart.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Hitag2KeyReport {
    pub uid: u32,
    pub remote: Hitag2Remote,
    /// The key in the standard order, and for a Fiat remote in the Fiat cipher's too.
    pub key: ShownKey,
    /// How long the search took ("69 s", "12 min").
    pub took: String,
    /// "saved in <file>", or why it could not be.
    pub saved: String,
    /// How many of the remote's captures now carry the key.
    pub captures: usize,
}

/// A running HiTag2 key search.
pub(super) struct Hitag2BfJob {
    /// The capture it was started from, then the second press it pairs with.
    pub(super) capture_ids: [u32; 2],
    uid: u32,
    /// The first capture's protocol: the key is saved in that protocol's byte order.
    remote: Hitag2Remote,
    pub(super) ctl: Arc<BruteForceControl>,
    handle: std::thread::JoinHandle<BruteForceOutcome>,
    started: Instant,
}

/// A capture's frame in the attack's terms, or why the attack cannot take it.
fn frame_of(c: &Capture) -> Result<Hitag2Frame, String> {
    hell::frame_of(c.protocol_name(), &c.decoded_signal())
}

/// The protocols the attack takes (a cheap filter before [`frame_of`]).
fn is_hitag2_remote(c: &Capture) -> bool {
    matches!(c.protocol_name(), "Fiat V1" | "Renault V1" | "Fiat V2" | "Hitag2")
}

/// Whether the signal menu offers the search for this capture: a Fiat V1, Renault V1, Fiat V2 or
/// Hitag2 RKE frame the attack takes, whose key KAT does not know.
fn offered(c: &Capture) -> bool {
    is_hitag2_remote(c)
        && frame_of(c).is_ok()
        && !crate::protocols::hitag2_key_known(c.protocol_name(), &c.decoded_signal())
}

/// The second press to pair with capture `first` (frame `f1`): another capture of the same remote
/// (same UID and protocol family) with another hop; one with another button first (with Fiat V2
/// that proves the key for both buttons), then the newest.
fn partner_of(captures: &[Capture], first: &Capture, f1: &Hitag2Frame) -> Option<(u32, Hitag2Frame)> {
    captures
        .iter()
        .filter(|c| c.id != first.id && is_hitag2_remote(c))
        .filter_map(|c| frame_of(c).ok().map(|f| (c.id, f)))
        .filter(|(_, f)| hell::requests_for(f1, Some(f)).is_ok())
        .max_by_key(|&(id, f)| (f.btn != f1.btn, id))
}

/// Mark the captures of remote `uid` that now send fresh codes (its key is saved) as recovered by
/// the search; returns how many.
fn mark_recovered(captures: &mut [Capture], uid: u32) -> usize {
    let mut fresh = 0;
    for c in captures.iter_mut().filter(|c| is_hitag2_remote(c)) {
        let remote = frame_of(c).is_ok_and(|f| f.uid == uid);
        if remote && crate::protocols::hitag2_key_known(c.protocol_name(), &c.decoded_signal()) {
            c.key_recovery = Some(KeyRecovery::Recovered(LABEL.to_string()));
            fresh += 1;
        }
    }
    fresh
}

/// The byte order the remote's cipher takes the key in, which is the order its firmwares save it
/// in (see `hitag2_keys.rs`): the standard one for Renault V1, Fiat's for Fiat V1 and V2.
fn key_order(remote: Hitag2Remote) -> KeyOrder {
    match remote {
        Hitag2Remote::RenaultV1 | Hitag2Remote::Hitag2Rke => KeyOrder::Standard,
        Hitag2Remote::FiatV1 | Hitag2Remote::FiatV2 => KeyOrder::FiatBcm,
    }
}

/// The key's bytes as the remote's firmwares save them ([`key_order`]).
fn saved_bytes(remote: Hitag2Remote, key: u64) -> [u8; 6] {
    match key_order(remote) {
        KeyOrder::Standard => hell::key_bytes(key),
        KeyOrder::FiatBcm => hell::fiat_bcm_key(key),
    }
}

impl App {
    /// [`offered`], memoised: the menu asks on every frame it draws, and a Renault V1 frame costs
    /// a seed search.
    pub(super) fn hitag2_bf_offered(&self, c: &Capture) -> bool {
        if !is_hitag2_remote(c) {
            return false;
        }
        let memo_key = (c.id, c.data, c.data_extra, hitag2_keys::generation());
        if let Some((key, was_offered)) = self.hitag2_offer_memo.get() {
            if key == memo_key {
                return was_offered;
            }
        }
        let is_offered = offered(c);
        self.hitag2_offer_memo.set(Some((memo_key, is_offered)));
        is_offered
    }

    /// The capture's HITAG2 key for the detail panel ([`hitag2_keys::shown_key`]), memoised: the
    /// panel asks on every frame it draws, and a Fiat V1 frame without a key costs a pass over
    /// the references' key lists.
    pub fn hitag2_shown_key(&self, c: &Capture) -> Option<ShownKey> {
        if !is_hitag2_remote(c) {
            return None;
        }
        let memo_key = (c.id, c.data, c.data_extra, hitag2_keys::generation());
        if let Some((key, shown)) = self.hitag2_key_memo.get() {
            if key == memo_key {
                return shown;
            }
        }
        let shown = hitag2_keys::shown_key(c.protocol_name(), &c.decoded_signal());
        self.hitag2_key_memo.set(Some((memo_key, shown)));
        shown
    }

    /// The last key a search found, for its overlay ([`InputMode::Hitag2Key`]).
    pub fn hitag2_key_report(&self) -> Option<&Hitag2KeyReport> {
        self.hitag2_key_report.as_ref()
    }

    /// Whether a long key search (KeeLoq or HiTag2) or a PSA one the user asked for runs: the
    /// KeeLoq and HiTag2 searches start only when none does.
    pub(super) fn long_key_search_runs(&self) -> bool {
        self.psa_recover_job.as_ref().is_some_and(|j| !j.auto) || self.keeloq_bf_job.is_some() || self.hitag2_bf_job.is_some()
    }

    /// Start the search for the remote of capture `id`, paired with another capture of it.
    pub(super) fn start_hitag2_key_recovery(&mut self, id: u32) {
        // An autobf PSA search (well under a second) may finish beside it.
        if self.long_key_search_runs() {
            self.last_error = Some("A key recovery is already running".to_string());
            return;
        }
        let Some(capture) = self.captures.iter().find(|c| c.id == id) else {
            self.last_error = Some(format!("Capture #{id:02} not found"));
            return;
        };
        let f1 = match frame_of(capture) {
            Ok(f) => f,
            Err(e) => {
                self.last_error = Some(format!("Recover key (HiTag2): {e}"));
                return;
            }
        };
        let Some((partner, f2)) = partner_of(&self.captures, capture, &f1) else {
            self.last_error = Some(format!(
                "Recover key (HiTag2) needs a second press of remote {:08X}: capture it once more (any button) and try again",
                f1.uid
            ));
            return;
        };
        let requests = match hell::requests_for(&f1, Some(&f2)) {
            Ok(r) => r,
            Err(e) => {
                self.last_error = Some(format!("Recover key (HiTag2): {e}"));
                return;
            }
        };
        let ctl = Arc::new(BruteForceControl::default());
        let worker = Arc::clone(&ctl);
        let handle = std::thread::spawn(move || hell::recover_key_any(&requests, &worker));
        self.status_message =
            Some(format!("Recovering the HiTag2 key of remote {:08X} from #{id:02} and #{partner:02}…", f1.uid));
        self.hitag2_bf_job = Some(Hitag2BfJob {
            capture_ids: [id, partner],
            uid: f1.uid,
            remote: f1.remote,
            ctl,
            handle,
            started: Instant::now(),
        });
    }

    /// Ask a running search to stop; false when none runs.
    pub(super) fn cancel_hitag2_key_recovery(&mut self) -> bool {
        let Some(job) = &self.hitag2_bf_job else {
            return false;
        };
        job.ctl.cancel.store(true, Ordering::Relaxed);
        self.status_message = Some("Cancelling key recovery…".to_string());
        true
    }

    /// Progress of a running search, for [App::job_status] (the status bar title). The time left
    /// is to the end of the search; it stops at the key, on average halfway.
    pub(super) fn hitag2_key_recovery_progress(&self) -> Option<String> {
        let job = self.hitag2_bf_job.as_ref().filter(|j| !j.handle.is_finished())?;
        let total = hell::SEARCH_TOTAL;
        let done = job.ctl.progress.load(Ordering::Relaxed).min(total);
        let rate = f64::from(done) / job.started.elapsed().as_secs_f64().max(0.001);
        let left = if done > 0 { duration_label(f64::from(total - done) / rate) } else { "?".to_string() };
        Some(format!(
            "Recovering the HiTag2 key of remote {:08X}… {}%{}, at most {left} left",
            job.uid,
            (u64::from(done) * 100 / u64::from(total)).min(99),
            super::keeloq_bf::gpu_note()
        ))
    }

    /// The result of the search once it has finished, then the found key's overlay (every tick).
    pub(super) fn poll_hitag2_key_recovery(&mut self) {
        if self.hitag2_bf_job.as_ref().is_some_and(|j| j.handle.is_finished()) {
            if let Some(job) = self.hitag2_bf_job.take() {
                self.finish_hitag2_key_recovery(job);
            }
        }
        // The overlay opens once the list has the keys: over a menu, a form or a prompt it would
        // take theirs, and close on the Enter meant for them.
        if self.hitag2_key_report_due && self.input_mode == InputMode::Normal {
            self.hitag2_key_report_due = false;
            self.input_mode = InputMode::Hitag2Key;
            self.overlay_scroll = 0;
        }
    }

    /// What a finished search found: the key saved and shown, or why there is none.
    fn finish_hitag2_key_recovery(&mut self, job: Hitag2BfJob) {
        let took = duration_label(job.started.elapsed().as_secs_f64());
        let [first, second] = job.capture_ids;
        match job.handle.join() {
            Ok(BruteForceOutcome::Found(found)) => self.apply_hitag2_key(job.uid, job.remote, &found, &took),
            Ok(BruteForceOutcome::NotFound) => {
                for c in self.captures.iter_mut().filter(|c| job.capture_ids.contains(&c.id)) {
                    c.key_recovery = Some(KeyRecovery::NotFound(LABEL.to_string()));
                }
                let mut msg =
                    format!("No HiTag2 key fits #{first:02} and #{second:02} of remote {:08X} (searched in {took})", job.uid);
                if job.remote == Hitag2Remote::FiatV2 {
                    msg += "; Fiat V2's counter and button layout is not confirmed against a real key";
                }
                if job.remote == Hitag2Remote::Hitag2Rke {
                    msg += "; the Hitag2 RKE IV (counter and button, as Renault V1's) is not confirmed against a real remote";
                }
                self.last_error = Some(msg);
            }
            Ok(BruteForceOutcome::Cancelled) => {
                self.status_message = Some(format!("Key recovery for remote {:08X} cancelled", job.uid));
            }
            Err(_) => self.last_error = Some("Key recovery thread failed".to_string()),
        }
    }

    /// Save the key for the remote (this session and `hitag2_keys.txt`), mark the remote's
    /// captures that now send fresh codes as recovered, and have the overlay show the key.
    fn apply_hitag2_key(&mut self, uid: u32, remote: Hitag2Remote, found: &RecoveredKey, took: &str) {
        if found.uid != uid {
            // The C also accepts frames made with the complemented UID; KAT's encoders send the
            // captured UID, so the key would not serve them (and the firmwares would not use it).
            self.last_error = Some(format!(
                "HiTag2 key {:012X} of remote {:08X} works with the complemented UID {:08X}: KAT cannot send with it (not saved)",
                found.key, uid, found.uid
            ));
            return;
        }
        let key = saved_bytes(remote, found.key);
        let saved = match hitag2_keys::save_key(self.storage.config_dir(), uid, key, 0) {
            Ok(path) => format!("saved in {}", path.display()),
            Err(e) => format!("not saved ({e}), usable until KAT exits"),
        };
        let fresh = mark_recovered(&mut self.captures, uid);
        // Hitag2 RKE is decode-only (no verified transmitter): the key is recovered and saved, but
        // KAT does not send fresh codes for it, unlike Fiat V1 / Renault V1 / Fiat V2.
        let effect = if remote == Hitag2Remote::Hitag2Rke {
            "now carry the recovered key (decode only; no transmit)"
        } else {
            "can now send fresh codes"
        };
        self.status_message = Some(format!(
            "HiTag2 key of remote {uid:08X} recovered in {took}: {:012X} ({saved}); {fresh} capture{} {effect}",
            found.key,
            crate::plural(fresh)
        ));
        self.hitag2_key_report = Some(Hitag2KeyReport {
            uid,
            remote,
            key: ShownKey { standard: hell::key_bytes(found.key), order: key_order(remote), epoch: 0 },
            took: took.to_string(),
            saved,
            captures: fresh,
        });
        self.hitag2_key_report_due = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture::CaptureStatus;

    /// A Fiat V1 capture of remote `uid` (variant A, tail 2) signed with `key` (standard order).
    fn fiat_v1_capture(id: u32, uid: u32, btn: u8, ctrl: u16, key: u64) -> Capture {
        let hop = hell::authenticator(key, uid, u32::from(ctrl), btn);
        let mut raw = [0u8; 13];
        raw[1] = 0x01;
        raw[2..6].copy_from_slice(&uid.to_be_bytes());
        raw[6] = (btn << 4) | (ctrl >> 6) as u8;
        raw[7] = (((ctrl & 0x3F) << 2) as u8) | (hop >> 30) as u8;
        raw[8] = (hop >> 22) as u8;
        raw[9] = (hop >> 14) as u8;
        raw[10] = (hop >> 6) as u8;
        raw[11] = ((hop << 2) as u8) | 0x02;
        raw[12] = raw[..12].iter().fold(0x01, |a, b| a ^ b);
        let mut c = Capture::from_pairs(id, 433_920_000, Vec::new());
        c.protocol = Some("Fiat V1".to_string());
        (c.serial, c.button, c.counter, c.crc_valid) = (Some(uid), Some(btn), Some(ctrl), true);
        c.data = u64::from_be_bytes(raw[..8].try_into().unwrap());
        c.data_count_bit = 104;
        c.data_extra = Some(raw[8..].iter().fold(0u64, |a, &b| (a << 8) | u64::from(b)));
        c.status = CaptureStatus::EncoderCapable;
        c
    }

    /// Around a search, on a made-up Fiat V1 remote: offered while KAT can only replay it, paired
    /// with another press of it (another button first), and once its key is saved, no longer
    /// offered and its captures (not another remote's) marked as recovered.
    #[test]
    fn a_saved_key_ends_the_offer() {
        // A UID no other test uses: the key database is global.
        const UID: u32 = 0x5EED_0F1A;
        const KEY: u64 = 0x4B41_5431_2E38;
        let mut captures = vec![
            fiat_v1_capture(1, UID, 0x4, 0x101, KEY),
            fiat_v1_capture(2, UID, 0x4, 0x102, KEY),
            fiat_v1_capture(3, UID, 0x8, 0x103, KEY),
            fiat_v1_capture(4, UID + 1, 0x8, 0x104, KEY),
        ];
        assert!(offered(&captures[0]));
        let shown = |c: &Capture| hitag2_keys::shown_key(c.protocol_name(), &c.decoded_signal());
        assert_eq!(shown(&captures[0]), None, "no key yet: no row in the detail panel");
        let f1 = frame_of(&captures[0]).unwrap();
        assert_eq!((f1.uid, f1.cnt, f1.btn), (UID, 0x101, 0x4));
        let (partner, f2) = partner_of(&captures, &captures[0], &f1).unwrap();
        assert_eq!((partner, f2.btn), (3, 0x8), "another button first");
        assert_eq!(partner_of(&captures[..2], &captures[0], &f1).map(|(id, _)| id), Some(2));
        let lone = &captures[3];
        assert_eq!(partner_of(&captures, lone, &frame_of(lone).unwrap()), None, "no second press");

        crate::protocols::hitag2_keys::tests::remember_for_test(UID, saved_bytes(Hitag2Remote::FiatV1, KEY));
        assert!(!offered(&captures[0]));
        assert!(offered(&captures[3]), "another remote");
        assert_eq!(mark_recovered(&mut captures, UID), 3);
        assert_eq!(captures[0].key_recovery, Some(KeyRecovery::Recovered(LABEL.to_string())));
        assert_eq!(captures[3].key_recovery, None);
        // The detail panel's row: the key in the standard order, as the status line names it, and
        // in the Fiat order, as it was saved.
        let key = shown(&captures[0]).expect("the saved key signs the frame");
        assert_eq!(key, ShownKey { standard: hell::key_bytes(KEY), order: KeyOrder::FiatBcm, epoch: 0 });
        assert_eq!(hitag2_keys::hex(&key.standard, ""), format!("{KEY:012X}"));
        assert_eq!(key.fiat_order(), Some(saved_bytes(Hitag2Remote::FiatV1, KEY)));
        assert_eq!(hitag2_keys::hex(&key.fiat_order().unwrap(), " "), "54 31 2E 38 4B 41");
        assert_eq!(shown(&captures[2]), Some(key), "every press of the remote");
        assert_eq!(shown(&captures[3]), None, "another remote");
    }

    /// The key order the firmwares save: Renault V1 standard, Fiat V1 / V2 rotated (the orders of
    /// `hitag2_keys::KeyOrder`).
    #[test]
    fn keys_are_saved_in_the_protocols_order() {
        let std_key = [0xE9, 0x9B, 0xDE, 0x9C, 0x28, 0x13];
        assert_eq!(saved_bytes(Hitag2Remote::RenaultV1, 0xE99B_DE9C_2813), std_key);
        assert_eq!(saved_bytes(Hitag2Remote::FiatV1, 0xE99B_DE9C_2813), [0xDE, 0x9C, 0x28, 0x13, 0xE9, 0x9B]);
        assert_eq!(saved_bytes(Hitag2Remote::FiatV2, 0xE99B_DE9C_2813), KeyOrder::FiatBcm.arrange(std_key));
    }
}
