//! Session persistence for [`App`] (file format and I/O: [`crate::session`]).
//!
//! - Startup: the saved session is loaded in [`App::init_app_extensions`]; if it has captures the
//!   user is asked "Restore N captures from your last session? (y/n)". The prompt comes after the
//!   no-device warning and before the import prompt (restored captures keep their ids, imports
//!   follow them, and import files already in the restored session are not offered again).
//!   Declining keeps the old file as `session.json.bak`; an unreadable file is kept the same way.
//! - Autosave: every UI tick compares a fingerprint of the captures with the last saved one and
//!   queues a snapshot for the writer thread when it changed, at most once a second
//!   ([`crate::session::Autosave`]); dropping the app (quit, error exit, panic unwind) saves once
//!   more and waits for the writer.
//! - `:clear` (with a y/n confirmation) empties the list and deletes the session file and backup.
//!
//! This module also hosts the per-tick hook and the key handling of the modes added in v1.5
//! (session restore prompt, `:clear` confirmation, Analyze overlay).

use std::collections::HashSet;
use std::path::{Path, PathBuf};
use std::time::Instant;

use anyhow::Result;
use chrono::{DateTime, Utc};
use crossterm::event::KeyCode;

use super::{App, InputMode};
use crate::capture::Capture;
use crate::plural;
use crate::session::{
    self, Autosave, LoadedSession, SessionLoad, SessionPaths, SessionSnapshot, SessionWriter, WriteJob,
};

/// Session state owned by [`App`].
#[derive(Default)]
pub struct SessionState {
    /// Session file paths in the config directory.
    paths: Option<SessionPaths>,
    /// Background writer; `None` when `session_autosave` is off.
    writer: Option<SessionWriter>,
    autosave: Autosave,
    /// Saved session waiting for the user's answer at startup.
    pending_restore: Option<LoadedSession>,
    /// The "only the newest max_captures are saved" notice was shown (reset below the limit).
    over_limit_noted: bool,
}

/// Drop the pending import files whose captures are already in the list (their `source_file`, the
/// path relative to the import directory, is present): after a restore they would be duplicates.
fn drop_restored_imports(pending: &mut Vec<PathBuf>, captures: &[Capture], import_dir: &Path) {
    let restored: HashSet<&str> = captures.iter().filter_map(|c| c.source_file.as_deref()).collect();
    pending.retain(|p| !restored.contains(App::path_relative_to_import(p, import_dir).as_str()));
}

/// What the startup restore prompt shows.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RestorePrompt {
    /// Captures that will be restored (the newest `max_captures` of the file).
    pub count: usize,
    /// Captures in the file.
    pub total: usize,
    pub saved_at: Option<DateTime<Utc>>,
}

impl App {
    /// Called once at the end of [`App::new`]: load the saved session (queueing its restore prompt)
    /// and start the session writer and the saved-fob index scan.
    pub(super) fn init_app_extensions(&mut self) {
        self.init_session();
        self.start_saved_fob_scan();
    }

    /// Per-tick work of the v1.5 features (called from `process_radio_events`, ~10×/s): install a
    /// finished saved-fob index, annotate new captures, autosave the session.
    pub(super) fn poll_app_extensions(&mut self) {
        self.poll_saved_fob_index();
        self.match_new_captures();
        self.session_tick();
    }

    /// Keys of the v1.5 modes (main loop dispatches them here).
    pub fn handle_app_mode_key(&mut self, code: KeyCode) -> Result<()> {
        match self.input_mode {
            // Only an explicit n declines: declining moves session.json to session.json.bak
            // (replacing an older backup), too much for a reflexive Esc, which does nothing here.
            InputMode::SessionRestore => match code {
                KeyCode::Char('y') | KeyCode::Char('Y') => self.restore_session(),
                KeyCode::Char('n') | KeyCode::Char('N') => self.decline_session_restore(),
                _ => {}
            },
            InputMode::ConfirmClear => match code {
                KeyCode::Char('y') | KeyCode::Char('Y') => self.confirm_clear(),
                KeyCode::Char('n') | KeyCode::Char('N') | KeyCode::Esc => {
                    self.input_mode = InputMode::Normal;
                    self.status_message = Some("Clear cancelled".to_string());
                }
                _ => {}
            },
            InputMode::Analyze => self.analyze_view_key(code),
            InputMode::Compare => self.compare_view_key(code),
            _ => {}
        }
        Ok(())
    }

    /// Mode after a startup prompt: restore prompt, then import prompt, then normal.
    pub fn next_startup_mode(&self) -> InputMode {
        if self.session.pending_restore.is_some() {
            InputMode::SessionRestore
        } else if !self.pending_fob_files.is_empty() {
            InputMode::StartupImport
        } else {
            InputMode::Normal
        }
    }

    /// Restore prompt contents, while the prompt is pending.
    pub fn session_restore_prompt(&self) -> Option<RestorePrompt> {
        let s = self.session.pending_restore.as_ref()?;
        let total = s.captures.len();
        let max = self.storage.config.max_captures;
        Some(RestorePrompt {
            count: if max == 0 { total } else { total.min(max) },
            total,
            saved_at: s.saved_at,
        })
    }

    /// Number of captures `:clear` would delete, and whether a saved session file exists.
    pub fn clear_prompt(&self) -> (usize, bool) {
        let files = self.session.paths.as_ref().is_some_and(session::session_files_exist);
        (self.captures.len(), files)
    }

    fn init_session(&mut self) {
        let paths = SessionPaths::in_dir(self.storage.config_dir());
        if self.storage.config.session_autosave {
            match session::load_session(&paths.file) {
                SessionLoad::Missing => {}
                SessionLoad::Loaded(s) if s.captures.is_empty() => {}
                SessionLoad::Loaded(s) => {
                    tracing::info!(
                        "Saved session: {} capture(s), format v{}, written by KAT {}",
                        s.captures.len(),
                        s.version,
                        s.kat_version.as_deref().unwrap_or("?")
                    );
                    if s.skipped > 0 || s.version > session::SESSION_VERSION {
                        // The next autosave rewrites the file without what this version could not
                        // read; keep the original.
                        tracing::warn!(
                            "Saved session v{}: {} unreadable capture(s); keeping a copy as {}",
                            s.version,
                            s.skipped,
                            session::SESSION_BACKUP
                        );
                        let _ = std::fs::copy(&paths.file, &paths.backup);
                    }
                    self.session.pending_restore = Some(s);
                }
                SessionLoad::Unreadable(why) => {
                    tracing::warn!("Saved session {:?} unreadable: {}", paths.file, why);
                    let kept = session::backup_session(&paths).is_ok();
                    self.last_error = Some(format!(
                        "Saved session unreadable ({}){}",
                        why,
                        if kept { "; kept as session.json.bak" } else { "" }
                    ));
                }
            }
            self.session.writer = Some(SessionWriter::spawn(paths.clone()));
            // The disk holds the old session, memory nothing: nothing to save until something changes
            // (and nothing at all while the restore prompt is pending).
            self.session
                .autosave
                .set_saved(session::fingerprint(&self.captures, self.next_capture_id));
        }
        self.session.paths = Some(paths);
        if self.session.pending_restore.is_some() {
            if self.input_mode != InputMode::HackRfNotDetected {
                self.input_mode = InputMode::SessionRestore;
            }
        } else {
            self.saved_fobs.announce_from(self.next_capture_id);
        }
    }

    /// 'y' at the restore prompt.
    fn restore_session(&mut self) {
        if let Some(s) = self.session.pending_restore.take() {
            let (total, skipped) = (s.captures.len(), s.skipped);
            let max = self.storage.config.max_captures;
            let added = session::restore_into(&mut self.captures, &mut self.next_capture_id, s, max);
            // autobf: restored undecrypted PSA captures without a search result get one.
            let ids: Vec<u32> = self.captures.iter().map(|c| c.id).collect();
            self.auto_bf_listed(ids);
            // Import files whose captures are already in the restored session are not offered again.
            let import_dir = self.storage.import_dir().clone();
            drop_restored_imports(&mut self.pending_fob_files, &self.captures, &import_dir);
            if added > 0 {
                self.selected_capture = Some(self.captures.len() - 1);
            }
            let mut msg = format!("Restored {added} capture{} from the last session", plural(added));
            if added < total {
                msg.push_str(&format!(" (newest {} of {}: max_captures)", added, total));
            }
            if skipped > 0 {
                msg.push_str(&format!("; {} unreadable skipped (copy in session.json.bak)", skipped));
            }
            self.status_message = Some(msg);
        }
        self.saved_fobs.announce_from(self.next_capture_id);
        self.input_mode = self.next_startup_mode();
    }

    /// 'n' at the restore prompt: start fresh, keep the old file as `session.json.bak`.
    fn decline_session_restore(&mut self) {
        if self.session.pending_restore.take().is_some() {
            if let Some(paths) = &self.session.paths {
                match session::backup_session(paths) {
                    Ok(()) => {
                        self.status_message = Some(
                            "New session started (previous one kept as session.json.bak)".to_string(),
                        )
                    }
                    Err(e) => self.last_error = Some(format!("Could not keep the old session: {}", e)),
                }
            }
        }
        self.saved_fobs.announce_from(self.next_capture_id);
        self.input_mode = self.next_startup_mode();
    }

    /// Debounced autosave + writer error reporting.
    fn session_tick(&mut self) {
        let Some(writer) = &self.session.writer else {
            return;
        };
        if let Some(e) = writer.poll_errors().pop() {
            tracing::warn!("{}", e);
            self.last_error = Some(e);
        }
        if self.session.pending_restore.is_some() {
            return; // never overwrite the saved session before the user answered
        }
        let max = self.storage.config.max_captures;
        let over_limit = max > 0 && self.captures.len() > max;
        if over_limit && !self.session.over_limit_noted {
            self.status_message = Some(format!(
                "The saved session keeps the newest {} captures (max_captures); export older ones to keep them",
                max
            ));
        }
        self.session.over_limit_noted = over_limit;
        let fingerprint = session::fingerprint(&self.captures, self.next_capture_id);
        let now = Instant::now();
        if self.session.autosave.due(fingerprint, now) {
            let snapshot = SessionSnapshot::new(
                &self.captures,
                self.next_capture_id,
                self.storage.config.max_captures,
            );
            writer.send(WriteJob::Save(snapshot));
            self.session.autosave.mark_written(fingerprint, now);
        }
    }

    /// Save once more and wait for the writer (on drop). A pending restore prompt leaves the file
    /// untouched.
    fn flush_session_on_exit(&mut self) {
        let Some(mut writer) = self.session.writer.take() else {
            return;
        };
        let last = self.session.pending_restore.is_none().then(|| {
            WriteJob::Save(SessionSnapshot::new(
                &self.captures,
                self.next_capture_id,
                self.storage.config.max_captures,
            ))
        });
        writer.finish(last);
    }

    /// `:clear`: ask for confirmation (or report that there is nothing to clear).
    pub(super) fn request_clear(&mut self) {
        let (captures, files) = self.clear_prompt();
        if captures == 0 && !files {
            self.status_message = Some("Nothing to clear".to_string());
            return;
        }
        self.input_mode = InputMode::ConfirmClear;
    }

    /// 'y' at the `:clear` confirmation: empty the list and delete the saved session (file + backup).
    fn confirm_clear(&mut self) {
        let count = self.captures.len();
        self.captures.clear();
        self.selected_capture = None;
        self.saved_fobs.forget_captures();
        self.close_analyze_view();
        self.close_compare_view();
        match (&self.session.writer, &self.session.paths) {
            // Through the writer, so a save still queued cannot recreate the file afterwards.
            (Some(writer), _) => writer.send(WriteJob::Clear),
            (None, Some(paths)) => {
                if let Err(e) = session::remove_session_files(paths) {
                    self.last_error = Some(format!("Could not delete the saved session: {}", e));
                }
            }
            (None, None) => {}
        }
        self.session
            .autosave
            .set_saved(session::fingerprint(&self.captures, self.next_capture_id));
        self.input_mode = InputMode::Normal;
        self.status_message = Some(format!("Cleared {count} capture{} and the saved session", plural(count)));
    }
}

impl Drop for App {
    /// Final session save on every exit path (quit, error return, panic unwind).
    fn drop(&mut self) {
        self.flush_session_on_exit();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture::StoredLevelDuration;

    #[test]
    fn restored_import_files_are_not_offered_again() {
        let import_dir = Path::new("/home/u/.config/KAT/import");
        let mut restored = Capture::from_pairs(1, 433_920_000, vec![StoredLevelDuration { level: true, duration_us: 1 }]);
        restored.source_file = Some("FORD/2008_Focus_Lock.sub".to_string());
        let live = Capture::from_pairs(2, 433_920_000, Vec::new());
        let mut pending = vec![
            import_dir.join("FORD/2008_Focus_Lock.sub"),
            import_dir.join("FORD/2008_Focus_Unlock.sub"),
            PathBuf::from("/elsewhere/x.fob"),
        ];
        drop_restored_imports(&mut pending, &[restored, live], import_dir);
        assert_eq!(
            pending,
            vec![import_dir.join("FORD/2008_Focus_Unlock.sub"), PathBuf::from("/elsewhere/x.fob")]
        );
    }
}
