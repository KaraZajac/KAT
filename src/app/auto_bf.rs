//! `autobf` (config, on by default): KAT recovers the key of every capture a brute force can crack
//! without being asked. An undecrypted PSA frame (TEA) gets ProtoPirate's BF1/BF2 search, the same
//! search as the signal menu's "Recover key" and on the same background job: one search runs at a
//! time, the others wait here, oldest first, and a "Recover key" the user asks for goes first.
//! They run beside a long KeeLoq or HiTag2 search (well under a second each), so frames keep
//! showing up while one runs. HiTag2 keys (minutes of every core per remote) are only searched
//! when asked (`app/hitag2_bf.rs`).
//! A frame of PSA's adaptive AM path that the fast path cannot decrypt (`PSA [unchecked]`, see
//! `protocols/psa.rs`) waits here, outside the list, and joins it only if its key is found: a
//! genuine TEA frame decrypts in a fraction of a second, noise never does. A child module of `app`
//! (`mod auto_bf;`), so it can use App's fields.

use std::collections::VecDeque;

use super::App;
use crate::capture::{Capture, CaptureStatus, KeyRecovery};
use crate::protocols::{
    psa_needs_key_search, psa_search_label, PsaBruteForceHit, PsaBruteForceOutcome, PSA_SEARCH_LABEL_ALL,
};

/// A key search waiting for the background job.
pub(super) enum Waiting {
    /// A capture in the list. `asked`: by the "Recover key" action, which goes first and reports
    /// as that action always did.
    Listed { id: u32, asked: bool },
    /// A `PSA [unchecked]` frame, listed only if its key is found.
    Unchecked(Box<Capture>),
}

/// The searches waiting for the background job (`App::auto_bf`).
#[derive(Default)]
pub(super) struct AutoBf {
    waiting: VecDeque<Waiting>,
}

/// Give a capture the fields of its recovered PSA key: plain PSA, encoder-capable (so it can be
/// exported and transmitted, and counts as vulnerable), marked as recovered by the search.
pub(super) fn apply_psa_hit(c: &mut Capture, hit: &PsaBruteForceHit) {
    c.protocol = Some("PSA".to_string());
    c.serial = Some(hit.fields.serial);
    c.button = Some(hit.fields.button);
    // PSA counters are up to 28 bits; the capture keeps the low 16 like the decoder.
    c.counter = Some(hit.fields.counter as u16);
    c.crc_valid = true;
    c.status = CaptureStatus::EncoderCapable;
    c.key_recovery = Some(KeyRecovery::Recovered(psa_search_label(hit)));
}

impl App {
    /// Queue the key search of each listed capture in `ids` that needs one and has no result yet
    /// (`autobf`; nothing when it is off).
    pub(super) fn auto_bf_listed(&mut self, ids: impl IntoIterator<Item = u32>) {
        if !self.storage.config.autobf {
            return;
        }
        for id in ids {
            let Some(c) = self.captures.iter().find(|c| c.id == id) else { continue };
            if c.key_recovery.is_some() || !psa_needs_key_search(c.protocol_name(), c.serial, c.data_extra) {
                continue;
            }
            if !self.key_search_planned(id) {
                self.auto_bf.waiting.push_back(Waiting::Listed { id, asked: false });
            }
        }
    }

    /// Hold a `PSA [unchecked]` frame for its key search; a copy of one already held is dropped.
    pub(super) fn auto_bf_unchecked(&mut self, capture: Capture) {
        let same = |c: &Capture| c.data == capture.data && c.data_extra == capture.data_extra;
        let held = self.auto_bf.waiting.iter().any(|w| matches!(w, Waiting::Unchecked(c) if same(c)))
            || self.psa_recover_job.as_ref().and_then(|j| j.unchecked.as_deref()).is_some_and(same);
        if !held {
            self.auto_bf.waiting.push_back(Waiting::Unchecked(Box::new(capture)));
        }
    }

    /// "Recover key" while another search runs: first in line.
    pub(super) fn queue_asked_key_search(&mut self, id: u32) {
        self.auto_bf.waiting.retain(|w| !matches!(w, Waiting::Listed { id: i, .. } if *i == id));
        self.auto_bf.waiting.push_front(Waiting::Listed { id, asked: true });
    }

    /// Whether capture `id`'s key search waits or runs (the detail panel says so).
    pub fn key_search_planned(&self, id: u32) -> bool {
        self.key_search_runs_for(id)
            || self.auto_bf.waiting.iter().any(|w| matches!(w, Waiting::Listed { id: i, .. } if *i == id))
    }

    /// Start the next waiting search once no PSA search runs (every tick); a long KeeLoq or HiTag2
    /// search may run beside it.
    pub(super) fn auto_bf_tick(&mut self) {
        if self.psa_recover_job.is_some() {
            return;
        }
        while let Some(next) = self.auto_bf.waiting.pop_front() {
            match next {
                Waiting::Listed { id, asked } => {
                    // Deleted, or decrypted meanwhile: nothing to search.
                    let Some(c) = self.captures.iter().find(|c| c.id == id) else { continue };
                    if !psa_needs_key_search(c.protocol_name(), c.serial, c.data_extra) {
                        continue;
                    }
                    let (data, extra) = (c.data, c.data_extra.unwrap_or_default());
                    self.spawn_psa_job(data, extra, Some(id), None, !asked);
                    return;
                }
                Waiting::Unchecked(c) => {
                    let (data, extra) = (c.data, c.data_extra.unwrap_or_default());
                    self.spawn_psa_job(data, extra, None, Some(c), true);
                    return;
                }
            }
        }
    }

    /// The search of listed capture `id` finished (`auto`: started by `autobf`).
    pub(super) fn psa_search_done(&mut self, id: u32, auto: bool, outcome: std::thread::Result<PsaBruteForceOutcome>) {
        match outcome {
            Ok(PsaBruteForceOutcome::Found(hit)) => {
                if let Some(c) = self.captures.iter_mut().find(|c| c.id == id) {
                    apply_psa_hit(c, &hit);
                    self.status_message = Some(format!(
                        "PSA key recovered for capture {id} ({}): serial {:06X}, counter {:X}",
                        psa_search_label(&hit),
                        hit.fields.serial,
                        hit.fields.counter
                    ));
                }
            }
            Ok(PsaBruteForceOutcome::NotFound) => {
                if let Some(c) = self.captures.iter_mut().find(|c| c.id == id) {
                    c.key_recovery = Some(KeyRecovery::NotFound(PSA_SEARCH_LABEL_ALL.to_string()));
                }
                let msg = format!("No PSA key found for capture {id} (BF1 and BF2 searched)");
                if auto {
                    self.status_message = Some(msg);
                } else {
                    self.last_error = Some(msg);
                }
            }
            Ok(PsaBruteForceOutcome::Cancelled) => {
                self.status_message = Some(format!("Key recovery for capture {id} cancelled"));
            }
            Err(_) => self.last_error = Some("Key recovery thread failed".to_string()),
        }
    }

    /// The search of a `PSA [unchecked]` frame finished: listed with its fields if the key was
    /// found, dropped otherwise (it was noise).
    pub(super) fn unchecked_search_done(&mut self, mut frame: Capture, outcome: std::thread::Result<PsaBruteForceOutcome>) {
        match outcome {
            Ok(PsaBruteForceOutcome::Found(hit)) => {
                apply_psa_hit(&mut frame, &hit);
                if let Some(id) = self.list_received(frame) {
                    self.status_message = Some(format!(
                        "New signal captured: PSA #{id}, key recovered ({}): serial {:06X}",
                        psa_search_label(&hit),
                        hit.fields.serial
                    ));
                }
            }
            Ok(_) => tracing::debug!(
                "PSA [unchecked] frame {:016X}/{:X}: no key, dropped",
                frame.data,
                frame.data_extra.unwrap_or_default()
            ),
            Err(_) => self.last_error = Some("Key recovery thread failed".to_string()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::{PsaBruteForceControl, PsaBruteForceOutcome};

    /// A recovered key makes the capture plain PSA with its fields, encoder-capable and marked as
    /// recovered by BF1 (one of the tester's doctor-firmware key files: serial 0x01124A).
    #[test]
    fn a_recovered_key_fills_the_capture() {
        let mut c = Capture::from_pairs_with_rf(1, 433_920_000, Vec::new(), None);
        c.protocol = Some(crate::protocols::PSA_UNCHECKED_NAME.to_string());
        c.data = 0xF1BA_BAE6_408C_36EE;
        c.data_extra = Some(0x9BA4);
        let PsaBruteForceOutcome::Found(hit) =
            crate::protocols::psa_recover_key(c.data, 0x9BA4, &PsaBruteForceControl::default())
        else {
            panic!("no key");
        };
        apply_psa_hit(&mut c, &hit);
        assert_eq!((c.protocol.as_deref(), c.serial), (Some("PSA"), Some(0x01124A)));
        assert_eq!(c.status, CaptureStatus::EncoderCapable);
        assert_eq!(c.key_recovery, Some(KeyRecovery::Recovered("PSA BF1".to_string())));
        assert!(!psa_needs_key_search(c.protocol_name(), c.serial, c.data_extra));
    }
}
