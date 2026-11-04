//! Saved-fob matching for [`App`] (rules and index: [`crate::saved_fobs`]).
//!
//! The `.fob` files of the export and import directories are indexed on a background thread at
//! startup and again after each `.fob` export. Every tick, captures that are new (or whose decoded
//! fields changed, e.g. after a PSA key recovery) are matched against the index; matches are kept
//! per capture id for the capture list marker and the detail panel. A live capture that matches
//! gets a status message; restored and imported captures are annotated silently.

use std::collections::{HashMap, HashSet};
use std::sync::mpsc::{Receiver, TryRecvError};

use super::App;
use crate::capture::Capture;
use crate::protocols::ProtocolRegistry;
use crate::saved_fobs::{self, MatchKey, SavedFob, SavedFobMatch};

/// The decoded fields a match depends on; a change triggers a re-match.
type MatchSignature = (Option<String>, Option<u32>, Option<u16>, u64, usize);

/// Saved-fob index and match bookkeeping owned by [`App`].
#[derive(Default)]
pub struct SavedFobState {
    /// Indexed `.fob` files, canonical protocol names filled in.
    index: Vec<SavedFob>,
    /// An index has been installed (matching waits for the first scan).
    ready: bool,
    /// Running background scan.
    scan: Option<Receiver<Vec<SavedFob>>>,
    /// Another scan was requested while one was running.
    rescan_requested: bool,
    /// Match per capture id.
    matches: HashMap<u32, SavedFobMatch>,
    /// Capture ids matched against the current index, with the fields they were matched on.
    checked: HashMap<u32, MatchSignature>,
    /// Capture ids ever matched (a status message is shown only the first time).
    seen: HashSet<u32>,
    /// Captures with an id from here on that have no source file are live captures (status
    /// message on a match); set once the startup restore prompt is resolved.
    announce_from_id: Option<u32>,
}

impl SavedFobState {
    pub(super) fn announce_from(&mut self, id: u32) {
        self.announce_from_id = Some(id);
    }

    /// Drop every per-capture annotation (`:clear`).
    pub(super) fn forget_captures(&mut self) {
        self.matches.clear();
        self.checked.clear();
        self.seen.clear();
    }

    /// Use a freshly scanned index; every capture is matched again against it.
    fn install_index(&mut self, mut index: Vec<SavedFob>, registry: &ProtocolRegistry) {
        saved_fobs::canonicalize(&mut index, registry);
        self.index = index;
        self.ready = true;
        self.checked.clear();
    }

    /// Match the captures that are new or whose decoded fields changed since they were last
    /// matched. Returns the status message for a live capture that matches for the first time.
    fn update_matches(&mut self, captures: &[Capture], registry: &ProtocolRegistry) -> Option<String> {
        if !self.ready {
            return None;
        }
        let mut announce = None;
        for c in captures {
            let sig = signature(c);
            if self.checked.get(&c.id) == Some(&sig) {
                continue;
            }
            let found = MatchKey::for_capture(c, registry)
                .and_then(|key| saved_fobs::find_match(&self.index, &key));
            self.checked.insert(c.id, sig);
            let first_time = self.seen.insert(c.id);
            match found {
                Some(m) => {
                    let live = c.source_file.is_none()
                        && self.announce_from_id.is_some_and(|from| c.id >= from);
                    if first_time && live {
                        announce = Some(format!(
                            "Signal #{:02} matches saved fob {} ({})",
                            c.id,
                            m.name,
                            m.counter_label()
                        ));
                    }
                    self.matches.insert(c.id, m);
                }
                None => {
                    self.matches.remove(&c.id);
                }
            }
        }
        announce
    }
}

fn signature(c: &Capture) -> MatchSignature {
    (c.protocol.clone(), c.serial, c.counter, c.data, c.data_count_bit)
}

impl App {
    /// Saved file the capture matches, if any (capture list marker, detail panel).
    pub fn saved_match(&self, capture_id: u32) -> Option<&SavedFobMatch> {
        self.saved_fobs.matches.get(&capture_id)
    }

    /// Number of indexed `.fob` files (0 until the first scan finished).
    #[allow(dead_code)]
    pub fn saved_fob_count(&self) -> usize {
        self.saved_fobs.index.len()
    }

    /// (Re)index the export and import directories in the background.
    pub(super) fn start_saved_fob_scan(&mut self) {
        if self.saved_fobs.scan.is_some() {
            self.saved_fobs.rescan_requested = true;
            return;
        }
        let dirs = vec![self.storage.export_dir().clone(), self.storage.import_dir().clone()];
        self.saved_fobs.scan = Some(saved_fobs::spawn_scan(dirs));
    }

    /// A `.fob` was exported: re-index so it is matched from now on.
    pub(super) fn saved_fobs_rescan(&mut self) {
        self.start_saved_fob_scan();
    }

    /// Install a finished scan; every capture is matched again against the new index.
    pub(super) fn poll_saved_fob_index(&mut self) {
        let Some(rx) = &self.saved_fobs.scan else {
            return;
        };
        match rx.try_recv() {
            Ok(index) => {
                tracing::info!("Indexed {} saved .fob file(s)", index.len());
                self.saved_fobs.install_index(index, &self.protocols);
                self.saved_fobs.scan = None;
            }
            Err(TryRecvError::Empty) => return,
            Err(TryRecvError::Disconnected) => {
                tracing::warn!("Saved-fob index scan ended without a result");
                self.saved_fobs.scan = None;
            }
        }
        if std::mem::take(&mut self.saved_fobs.rescan_requested) {
            self.start_saved_fob_scan();
        }
    }

    /// Match new or changed captures; a live match gets a status message.
    pub(super) fn match_new_captures(&mut self) {
        if let Some(msg) = self.saved_fobs.update_matches(&self.captures, &self.protocols) {
            self.status_message = Some(msg);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture::{CaptureStatus, StoredLevelDuration};
    use std::path::PathBuf;

    fn fob(serial: u32, counter: u16) -> SavedFob {
        SavedFob {
            path: PathBuf::from(format!("/x/fob_{serial:X}.fob")),
            name: format!("fob_{serial:X}"),
            protocol: "Kia V1".to_string(),
            canonical: String::new(),
            serial: Some(serial),
            key: None,
            counter: Some(counter),
            year: None,
            make: None,
            model: None,
            region: None,
            command: None,
            notes: None,
        }
    }

    fn capture(id: u32, serial: Option<u32>, counter: u16, source: Option<&str>) -> Capture {
        let mut c = Capture::from_pairs(id, 315_000_000, vec![StoredLevelDuration { level: true, duration_us: 800 }]);
        c.protocol = Some("Kia V1".to_string());
        c.serial = serial;
        c.counter = Some(counter);
        c.data = 0x00B4_4470_6E01_6602;
        c.data_count_bit = 57;
        c.status = CaptureStatus::EncoderCapable;
        c.source_file = source.map(str::to_string);
        c
    }

    #[test]
    fn live_restored_and_imported_matches() {
        let reg = ProtocolRegistry::new();
        let mut st = SavedFobState::default();
        // Restored ids 1..=2, live captures from id 3 on.
        st.announce_from(3);
        let mut captures = vec![
            capture(1, Some(0xB444706E), 0x60, None),                 // restored
            capture(2, Some(0xB444706E), 0x61, Some("KIA/x.sub")), // restored import
        ];
        // No index yet: nothing happens.
        assert_eq!(st.update_matches(&captures, &reg), None);
        assert!(st.matches.is_empty());
        st.install_index(vec![fob(0xB444706E, 0x66)], &reg);
        // Restored captures are annotated silently.
        assert_eq!(st.update_matches(&captures, &reg), None);
        assert_eq!(st.matches[&1].counter_label(), "cnt -6");
        assert_eq!(st.matches[&2].counter_label(), "cnt -5");

        // A live capture of the same fob: annotated + status message, once.
        captures.push(capture(3, Some(0xB444706E), 0x69, None));
        assert_eq!(
            st.update_matches(&captures, &reg).as_deref(),
            Some("Signal #03 matches saved fob fob_B444706E (cnt +3)")
        );
        assert_eq!(st.update_matches(&captures, &reg), None);
        // An import added later: annotated, no message.
        captures.push(capture(4, Some(0xB444706E), 0x6A, Some("KIA/y.sub")));
        assert_eq!(st.update_matches(&captures, &reg), None);
        assert!(st.matches.contains_key(&4));
        // A live capture of another fob: no match, no message.
        captures.push(capture(5, Some(0x12345678), 0x69, None));
        assert_eq!(st.update_matches(&captures, &reg), None);
        assert!(!st.matches.contains_key(&5));

        // Re-index (after an export): everything is re-matched, nobody is announced again.
        st.install_index(vec![fob(0xB444706E, 0x66), fob(0x12345678, 0x68)], &reg);
        assert_eq!(st.update_matches(&captures, &reg), None);
        assert_eq!(st.matches[&5].counter_label(), "cnt +1");

        // Decoded fields that change later (e.g. a PSA key recovery adding the serial) re-match.
        captures.push(capture(6, None, 0x69, None));
        captures[5].data = 0; // key differs from every saved fob, and no serial yet
        assert_eq!(st.update_matches(&captures, &reg), None);
        assert!(!st.matches.contains_key(&6));
        captures[5].serial = Some(0xB444706E);
        assert_eq!(st.update_matches(&captures, &reg), None, "seen before: no message");
        assert!(st.matches.contains_key(&6));

        // :clear forgets every annotation.
        st.forget_captures();
        assert!(st.matches.is_empty() && st.checked.is_empty() && st.seen.is_empty());
    }
}
