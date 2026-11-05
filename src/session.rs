//! Saved session: every capture autosaved to `session.json` in KAT's config directory and offered
//! for restore at the next start, so quitting no longer loses what was not exported.
//!
//! **Format** (versioned JSON, [`SESSION_VERSION`]):
//!
//! ```text
//! { "format": "kat-session", "version": 1, "kat_version": "1.5.0",
//!   "saved_at": "2026-11-04T10:31:00Z", "next_capture_id": 42,
//!   "captures": [ <Capture exactly as its serde derive writes it>, ... ] }
//! ```
//!
//! Captures reuse `Capture`'s own serde derive, so every field (raw pairs, `data_extra`,
//! `received_rf`, status, user metadata, and any optional field added later) is saved without a
//! second schema. Loading is lenient: unknown fields are ignored, missing optional fields take
//! their defaults, a few required fields get safe defaults (see [`fill_capture_defaults`]), and a
//! capture that still fails to parse is skipped and counted instead of failing the whole file. A
//! file that is not a KAT session at all is reported as unreadable; the app then keeps it as
//! `session.json.bak` and starts normally.
//!
//! **Writes** are atomic (temp file in the same directory, fsync, rename, directory fsync) with
//! owner-only permissions (0600: captures hold rolling codes) and run on a background thread
//! ([`SessionWriter`]) so the UI never waits for the disk. An empty session removes the file
//! instead of writing an empty one. [`Autosave`] + [`fingerprint`] decide when a save is due (state
//! changed, at most once per interval); the app also saves once more on exit.

use std::collections::HashSet;
use std::fs;
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

use crate::capture::Capture;

/// Session file name inside the config directory.
pub const SESSION_FILE: &str = "session.json";
/// Where a declined or unreadable session is kept (one generation).
pub const SESSION_BACKUP: &str = "session.json.bak";
/// Value of the `format` field.
pub const SESSION_FORMAT: &str = "kat-session";
/// Current format version. Older files load; newer files load leniently (unknown fields ignored).
pub const SESSION_VERSION: u32 = 1;
/// Minimum time between two autosaves.
pub const AUTOSAVE_INTERVAL: Duration = Duration::from_secs(1);

const KAT_VERSION: &str = env!("CARGO_PKG_VERSION");

/// Paths of the session file, its temp file and its backup inside `config_dir`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SessionPaths {
    pub file: PathBuf,
    pub tmp: PathBuf,
    pub backup: PathBuf,
}

impl SessionPaths {
    pub fn in_dir(config_dir: &Path) -> Self {
        Self {
            file: config_dir.join(SESSION_FILE),
            tmp: config_dir.join(format!("{}.tmp", SESSION_FILE)),
            backup: config_dir.join(SESSION_BACKUP),
        }
    }
}

// ─── Saving ──────────────────────────────────────────────────────────────────

/// One saved state of the session (what goes into the file).
#[derive(Debug, Clone, Serialize)]
pub struct SessionSnapshot {
    pub format: String,
    pub version: u32,
    pub kat_version: String,
    pub saved_at: DateTime<Utc>,
    pub next_capture_id: u32,
    pub captures: Vec<Capture>,
}

/// The newest `max_captures` captures (list order; 0 = no limit).
pub fn newest(captures: &[Capture], max_captures: usize) -> &[Capture] {
    if max_captures == 0 || captures.len() <= max_captures {
        captures
    } else {
        &captures[captures.len() - max_captures..]
    }
}

impl SessionSnapshot {
    /// Snapshot of the newest `max_captures` captures (0 = all).
    pub fn new(captures: &[Capture], next_capture_id: u32, max_captures: usize) -> Self {
        Self {
            format: SESSION_FORMAT.to_string(),
            version: SESSION_VERSION,
            kat_version: KAT_VERSION.to_string(),
            saved_at: Utc::now(),
            next_capture_id,
            captures: newest(captures, max_captures).to_vec(),
        }
    }
}

/// Create `path` for writing with owner-only permissions (0600 on Unix), replacing a stale file.
fn create_private(path: &Path) -> io::Result<fs::File> {
    match fs::remove_file(path) {
        Ok(()) => {}
        Err(e) if e.kind() == io::ErrorKind::NotFound => {}
        Err(e) => return Err(e),
    }
    let mut opts = fs::OpenOptions::new();
    opts.write(true).create_new(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        opts.mode(0o600);
    }
    let file = opts.open(path)?;
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        file.set_permissions(fs::Permissions::from_mode(0o600))?;
    }
    Ok(file)
}

/// fsync a directory so a rename inside it is durable (best effort; a no-op off Unix).
fn sync_dir(dir: &Path) {
    #[cfg(unix)]
    if let Ok(d) = fs::File::open(dir) {
        let _ = d.sync_all();
    }
    #[cfg(not(unix))]
    let _ = dir;
}

fn remove_if_exists(path: &Path) -> io::Result<()> {
    match fs::remove_file(path) {
        Ok(()) => Ok(()),
        Err(e) if e.kind() == io::ErrorKind::NotFound => Ok(()),
        Err(e) => Err(e),
    }
}

/// Write the snapshot atomically: temp file (0600) in the same directory, fsync, rename over
/// `paths.file`, fsync the directory. An empty snapshot removes the session file instead.
pub fn write_session_atomic(paths: &SessionPaths, snapshot: &SessionSnapshot) -> io::Result<()> {
    if snapshot.captures.is_empty() {
        return remove_if_exists(&paths.file);
    }
    let dir = paths.file.parent().unwrap_or_else(|| Path::new("."));
    fs::create_dir_all(dir)?;
    let file = create_private(&paths.tmp)?;
    let mut out = io::BufWriter::new(file);
    let written = serde_json::to_writer(&mut out, snapshot)
        .map_err(io::Error::other)
        .and_then(|()| out.flush())
        .and_then(|()| out.get_ref().sync_all());
    if let Err(e) = written {
        drop(out);
        let _ = fs::remove_file(&paths.tmp);
        return Err(e);
    }
    drop(out);
    if let Err(e) = fs::rename(&paths.tmp, &paths.file) {
        let _ = fs::remove_file(&paths.tmp);
        return Err(e);
    }
    sync_dir(dir);
    Ok(())
}

/// Delete the session file, its temp file and its backup (`:clear`).
pub fn remove_session_files(paths: &SessionPaths) -> io::Result<()> {
    remove_if_exists(&paths.file)?;
    remove_if_exists(&paths.tmp)?;
    remove_if_exists(&paths.backup)
}

/// Move the session file to the backup name (replacing an older backup).
pub fn backup_session(paths: &SessionPaths) -> io::Result<()> {
    fs::rename(&paths.file, &paths.backup)
}

/// True if any session file (current or backup) exists.
pub fn session_files_exist(paths: &SessionPaths) -> bool {
    paths.file.exists() || paths.backup.exists()
}

// ─── Loading ─────────────────────────────────────────────────────────────────

/// A session read from disk.
#[derive(Debug, Clone)]
pub struct LoadedSession {
    /// `version` field (0 when absent).
    pub version: u32,
    pub kat_version: Option<String>,
    pub saved_at: Option<DateTime<Utc>>,
    /// At least one more than the highest capture id.
    pub next_capture_id: u32,
    /// Captures in file order, ids unique and non-zero.
    pub captures: Vec<Capture>,
    /// Captures that could not be parsed and were skipped.
    pub skipped: usize,
}

/// Result of [`load_session`].
#[derive(Debug)]
pub enum SessionLoad {
    /// No session file.
    Missing,
    Loaded(LoadedSession),
    /// The file exists but is not a readable KAT session (the message says why).
    Unreadable(String),
}

/// Fast path: a well-formed file (everything KAT itself writes) parses straight into captures.
#[derive(Deserialize)]
struct SessionFileTyped {
    format: String,
    #[serde(default)]
    version: Option<u32>,
    #[serde(default)]
    kat_version: Option<String>,
    #[serde(default)]
    saved_at: Option<DateTime<Utc>>,
    #[serde(default)]
    next_capture_id: Option<u32>,
    captures: Vec<Capture>,
}

/// Lenient path: every capture is checked on its own.
#[derive(Deserialize)]
struct SessionFileIn {
    #[serde(default)]
    format: Option<String>,
    #[serde(default)]
    version: Option<u32>,
    #[serde(default)]
    kat_version: Option<String>,
    #[serde(default)]
    saved_at: Option<serde_json::Value>,
    #[serde(default)]
    next_capture_id: Option<u32>,
    #[serde(default)]
    captures: Option<Vec<serde_json::Value>>,
}

/// Give a capture object safe defaults for fields older or hand-edited files may lack:
/// `raw_pairs` [], `crc_valid` false, `data` 0, `data_count_bit` 0, `status` (Decoded with a
/// protocol, else Unknown), `id` 0 (renumbered later), `timestamp` the session's save time.
/// `frequency` gets no default (a wrong one would transmit on the wrong band): without it the
/// capture is skipped.
pub fn fill_capture_defaults(obj: &mut serde_json::Map<String, serde_json::Value>, saved_at: DateTime<Utc>) {
    use serde_json::Value;
    let has_protocol = obj.get("protocol").is_some_and(|p| p.is_string());
    let defaults: [(&str, Value); 7] = [
        ("id", Value::from(0u32)),
        ("timestamp", Value::String(saved_at.to_rfc3339())),
        ("raw_pairs", Value::Array(Vec::new())),
        ("crc_valid", Value::Bool(false)),
        ("data", Value::from(0u64)),
        ("data_count_bit", Value::from(0u64)),
        ("status", Value::String(if has_protocol { "Decoded" } else { "Unknown" }.to_string())),
    ];
    for (key, value) in defaults {
        obj.entry(key.to_string()).or_insert(value);
    }
}

/// Make ids unique and non-zero (the UI and every command address captures by id) and compute
/// the next free id.
fn finish_load(
    version: Option<u32>,
    kat_version: Option<String>,
    saved_at: Option<DateTime<Utc>>,
    next_capture_id: Option<u32>,
    mut captures: Vec<Capture>,
    skipped: usize,
) -> LoadedSession {
    let mut next = captures.iter().map(|c| c.id).max().unwrap_or(0).saturating_add(1);
    next = next.max(next_capture_id.unwrap_or(1)).max(1);
    let mut seen = HashSet::new();
    for c in &mut captures {
        if c.id == 0 || !seen.insert(c.id) {
            c.id = next;
            seen.insert(next);
            next = next.saturating_add(1);
        }
    }
    LoadedSession {
        version: version.unwrap_or(0),
        kat_version,
        saved_at,
        next_capture_id: next,
        captures,
        skipped,
    }
}

/// Parse a session file's contents: the typed fast path first, then (for older, newer or damaged
/// files) the lenient path that fills defaults and skips captures that still do not parse.
pub fn parse_session(json: &str) -> Result<LoadedSession, String> {
    if let Ok(f) = serde_json::from_str::<SessionFileTyped>(json) {
        if f.format == SESSION_FORMAT {
            return Ok(finish_load(f.version, f.kat_version, f.saved_at, f.next_capture_id, f.captures, 0));
        }
    }
    let file: SessionFileIn =
        serde_json::from_str(json).map_err(|e| format!("not a KAT session file ({})", e))?;
    if let Some(format) = &file.format {
        if format != SESSION_FORMAT {
            return Err(format!("format is {:?}, not {:?}", format, SESSION_FORMAT));
        }
    } else if file.captures.is_none() {
        return Err("no \"format\" and no \"captures\": not a KAT session".to_string());
    }
    let saved_at = file
        .saved_at
        .as_ref()
        .and_then(|v| v.as_str())
        .and_then(|s| DateTime::parse_from_rfc3339(s).ok())
        .map(|d| d.with_timezone(&Utc));
    let fallback_time = saved_at.unwrap_or_else(Utc::now);

    let mut captures = Vec::new();
    let mut skipped = 0usize;
    for value in file.captures.unwrap_or_default() {
        let serde_json::Value::Object(mut obj) = value else {
            skipped += 1;
            continue;
        };
        fill_capture_defaults(&mut obj, fallback_time);
        match serde_json::from_value::<Capture>(serde_json::Value::Object(obj)) {
            Ok(c) => captures.push(c),
            Err(e) => {
                tracing::warn!("Session: skipping unreadable capture: {}", e);
                skipped += 1;
            }
        }
    }
    Ok(finish_load(file.version, file.kat_version, saved_at, file.next_capture_id, captures, skipped))
}

/// Read the session file at `path`.
pub fn load_session(path: &Path) -> SessionLoad {
    match fs::read_to_string(path) {
        Ok(text) => match parse_session(&text) {
            Ok(session) => SessionLoad::Loaded(session),
            Err(e) => SessionLoad::Unreadable(e),
        },
        Err(e) if e.kind() == io::ErrorKind::NotFound => SessionLoad::Missing,
        Err(e) => SessionLoad::Unreadable(e.to_string()),
    }
}

/// Add a loaded session's captures (its newest `max_captures`; 0 = all) to `captures`, renumbering
/// any id that is already taken, and move `next_capture_id` past every id. Returns how many captures
/// were added.
pub fn restore_into(
    captures: &mut Vec<Capture>,
    next_capture_id: &mut u32,
    session: LoadedSession,
    max_captures: usize,
) -> usize {
    let skip = if max_captures == 0 {
        0
    } else {
        session.captures.len().saturating_sub(max_captures)
    };
    let mut taken: HashSet<u32> = captures.iter().map(|c| c.id).collect();
    let mut next = (*next_capture_id)
        .max(session.next_capture_id)
        .max(captures.iter().chain(&session.captures).map(|c| c.id).max().unwrap_or(0).saturating_add(1))
        .max(1);
    let mut added = 0;
    for mut c in session.captures.into_iter().skip(skip) {
        if !taken.insert(c.id) {
            c.id = next;
            taken.insert(next);
            next = next.saturating_add(1);
        }
        captures.push(c);
        added += 1;
    }
    *next_capture_id = next;
    added
}

// ─── Autosave bookkeeping ────────────────────────────────────────────────────

/// Cheap fingerprint of the session state, computed every UI tick to notice changes without
/// serializing. It covers every field KAT changes after a capture is added (decode fields after a
/// PSA key recovery, user metadata) plus the list itself; raw pairs never change after capture, so
/// their length stands for them. A field missed here is still saved by the next other change and by
/// the save on exit.
pub fn fingerprint(captures: &[Capture], next_capture_id: u32) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut h = std::collections::hash_map::DefaultHasher::new();
    next_capture_id.hash(&mut h);
    captures.len().hash(&mut h);
    for c in captures {
        (c.id, c.timestamp.timestamp(), c.timestamp.timestamp_subsec_nanos(), c.frequency).hash(&mut h);
        (&c.protocol, c.serial, c.button, c.counter, c.crc_valid).hash(&mut h);
        (c.data, c.data_count_bit, c.data_extra, c.raw_pairs.len()).hash(&mut h);
        (c.status as u8, c.received_rf.map(|r| r as u8)).hash(&mut h);
        (&c.year, &c.make, &c.model, &c.region, &c.command, &c.source_file).hash(&mut h);
        c.key_recovery.hash(&mut h);
    }
    h.finish()
}

/// Debounce: a save is due when the state differs from the last saved one and at least the interval
/// has passed since the last write.
#[derive(Debug, Clone)]
pub struct Autosave {
    saved: Option<u64>,
    last_write: Option<Instant>,
    interval: Duration,
}

impl Default for Autosave {
    fn default() -> Self {
        Self::new(AUTOSAVE_INTERVAL)
    }
}

impl Autosave {
    pub fn new(interval: Duration) -> Self {
        Self { saved: None, last_write: None, interval }
    }

    /// The state with this fingerprint is what the disk holds (after a load, decline or clear).
    pub fn set_saved(&mut self, fingerprint: u64) {
        self.saved = Some(fingerprint);
    }

    pub fn due(&self, fingerprint: u64, now: Instant) -> bool {
        self.saved != Some(fingerprint)
            && self.last_write.is_none_or(|t| now.duration_since(t) >= self.interval)
    }

    /// A snapshot with this fingerprint was handed to the writer.
    pub fn mark_written(&mut self, fingerprint: u64, now: Instant) {
        self.saved = Some(fingerprint);
        self.last_write = Some(now);
    }
}

// ─── Background writer ───────────────────────────────────────────────────────

/// Work for the writer thread.
#[derive(Debug)]
pub enum WriteJob {
    /// Write this snapshot (an empty one removes the session file).
    Save(SessionSnapshot),
    /// Delete the session file, its temp file and its backup.
    Clear,
}

/// Background thread that performs [`WriteJob`]s in order. Queued saves are coalesced (only the
/// newest one after the last `Clear` is written); a `Clear` is never dropped. Errors come back
/// through [`SessionWriter::poll_errors`]. Dropping the writer (or [`SessionWriter::finish`])
/// waits for the queued work.
pub struct SessionWriter {
    tx: Option<Sender<WriteJob>>,
    errors: Receiver<String>,
    handle: Option<JoinHandle<()>>,
}

fn run_jobs(paths: &SessionPaths, jobs: Vec<WriteJob>, errors: &Sender<String>) {
    let last_clear = jobs.iter().rposition(|j| matches!(j, WriteJob::Clear));
    let last_save = jobs.iter().rposition(|j| matches!(j, WriteJob::Save(_)));
    if last_clear.is_some() {
        if let Err(e) = remove_session_files(paths) {
            let _ = errors.send(format!("could not delete the saved session: {}", e));
        }
    }
    if let Some(i) = last_save.filter(|&i| last_clear.is_none_or(|c| i > c)) {
        if let Some(WriteJob::Save(snapshot)) = jobs.into_iter().nth(i) {
            if let Err(e) = write_session_atomic(paths, &snapshot) {
                let _ = errors.send(format!("session autosave failed: {}", e));
            }
        }
    }
}

impl SessionWriter {
    pub fn spawn(paths: SessionPaths) -> Self {
        let (tx, rx) = mpsc::channel::<WriteJob>();
        let (err_tx, errors) = mpsc::channel::<String>();
        let handle = std::thread::Builder::new()
            .name("kat-session".to_string())
            .spawn(move || {
                while let Ok(first) = rx.recv() {
                    let mut jobs = vec![first];
                    jobs.extend(rx.try_iter());
                    run_jobs(&paths, jobs, &err_tx);
                }
            })
            .ok();
        if handle.is_none() {
            tracing::warn!("Could not start the session writer thread; autosave is off");
        }
        Self { tx: handle.as_ref().map(|_| tx), errors, handle }
    }

    /// Queue a job (ignored if the thread is gone).
    pub fn send(&self, job: WriteJob) {
        if let Some(tx) = &self.tx {
            let _ = tx.send(job);
        }
    }

    /// Errors reported since the last call.
    pub fn poll_errors(&self) -> Vec<String> {
        self.errors.try_iter().collect()
    }

    /// Queue `last` (if any), then wait until every queued job is done.
    pub fn finish(&mut self, last: Option<WriteJob>) {
        if let Some(job) = last {
            self.send(job);
        }
        self.tx = None;
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

impl Drop for SessionWriter {
    fn drop(&mut self) {
        self.finish(None);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture::{CaptureStatus, RfModulation, StoredLevelDuration};

    /// Fresh private directory under the system temp dir.
    fn temp_dir(tag: &str) -> PathBuf {
        let n = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let dir = std::env::temp_dir().join(format!("kat_session_{}_{}_{}", tag, std::process::id(), n));
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// A capture with every field set to a non-default value.
    fn full_capture(id: u32) -> Capture {
        let pairs = (0..50)
            .map(|i| StoredLevelDuration { level: i % 2 == 0, duration_us: 250 + i * 7 })
            .collect();
        let mut c = Capture::from_pairs_with_rf(id, 433_920_000 + id, pairs, Some(RfModulation::FM));
        c.timestamp = DateTime::parse_from_rfc3339("2026-11-04T10:31:07.123456789Z").unwrap().with_timezone(&Utc);
        c.protocol = Some("VAG".to_string());
        c.serial = Some(0x0ABC_DEF0 + id);
        c.button = Some(0x20);
        c.counter = Some(0xBEEF);
        c.crc_valid = true;
        c.data = 0x0123_4567_89AB_CDEF;
        c.data_count_bit = 80;
        c.data_extra = Some(0xFEDC_BA98_7654_3210);
        c.status = CaptureStatus::EncoderCapable;
        c.year = Some("2019".to_string());
        c.make = Some("VW".to_string());
        c.model = Some("Golf".to_string());
        c.region = Some("EU".to_string());
        c.command = Some("Lock".to_string());
        c.source_file = Some("VAG/Golf4.sub".to_string());
        c
    }

    fn as_json(c: &Capture) -> serde_json::Value {
        serde_json::to_value(c).unwrap()
    }

    #[test]
    fn round_trip_preserves_every_field() {
        let dir = temp_dir("roundtrip");
        let paths = SessionPaths::in_dir(&dir);
        let mut unknown = Capture::from_pairs(9, 315_000_000, vec![StoredLevelDuration { level: true, duration_us: 1 }]);
        unknown.received_rf = Some(RfModulation::Both);
        let mut decoded = full_capture(4);
        decoded.status = CaptureStatus::Decoded;
        decoded.received_rf = Some(RfModulation::AM);
        decoded.data_extra = None;
        let captures = vec![full_capture(3), decoded, unknown];
        let snap = SessionSnapshot::new(&captures, 17, 0);
        write_session_atomic(&paths, &snap).unwrap();
        assert!(!paths.tmp.exists(), "temp file must be renamed away");

        let SessionLoad::Loaded(loaded) = load_session(&paths.file) else { panic!("load") };
        assert_eq!(loaded.version, SESSION_VERSION);
        assert_eq!(loaded.kat_version.as_deref(), Some(KAT_VERSION));
        assert_eq!(loaded.next_capture_id, 17);
        assert_eq!(loaded.skipped, 0);
        assert!(loaded.saved_at.is_some());
        assert_eq!(loaded.captures.len(), 3);
        // Serde's own view of every field (including any field added to Capture later) is equal.
        for (a, b) in captures.iter().zip(&loaded.captures) {
            assert_eq!(as_json(a), as_json(b));
        }
        let c = &loaded.captures[0];
        assert_eq!(c.raw_pairs.len(), 50);
        assert_eq!(c.raw_pairs[49].duration_us, 250 + 49 * 7);
        assert_eq!(c.data_extra, Some(0xFEDC_BA98_7654_3210));
        assert_eq!(c.received_rf, Some(RfModulation::FM));
        assert_eq!(c.status, CaptureStatus::EncoderCapable);
        assert_eq!(c.timestamp, captures[0].timestamp);
        assert_eq!(c.command.as_deref(), Some("Lock"));
        let _ = fs::remove_dir_all(&dir);
    }

    #[cfg(unix)]
    #[test]
    fn session_file_is_owner_only_and_replaces_atomically() {
        use std::os::unix::fs::PermissionsExt;
        let dir = temp_dir("perms");
        let paths = SessionPaths::in_dir(&dir);
        // A pre-existing world-readable file and a stale temp file are both replaced.
        fs::write(&paths.file, "old").unwrap();
        fs::set_permissions(&paths.file, fs::Permissions::from_mode(0o644)).unwrap();
        fs::write(&paths.tmp, "stale").unwrap();
        write_session_atomic(&paths, &SessionSnapshot::new(&[full_capture(1)], 2, 0)).unwrap();
        let mode = fs::metadata(&paths.file).unwrap().permissions().mode() & 0o777;
        assert_eq!(mode, 0o600);
        assert!(!paths.tmp.exists());
        assert!(matches!(load_session(&paths.file), SessionLoad::Loaded(s) if s.captures.len() == 1));
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn empty_session_removes_the_file_and_clear_removes_all() {
        let dir = temp_dir("empty");
        let paths = SessionPaths::in_dir(&dir);
        write_session_atomic(&paths, &SessionSnapshot::new(&[full_capture(1)], 2, 0)).unwrap();
        assert!(paths.file.exists());
        write_session_atomic(&paths, &SessionSnapshot::new(&[], 2, 0)).unwrap();
        assert!(!paths.file.exists());
        assert!(matches!(load_session(&paths.file), SessionLoad::Missing));

        write_session_atomic(&paths, &SessionSnapshot::new(&[full_capture(1)], 2, 0)).unwrap();
        backup_session(&paths).unwrap();
        assert!(paths.backup.exists() && !paths.file.exists());
        write_session_atomic(&paths, &SessionSnapshot::new(&[full_capture(2)], 3, 0)).unwrap();
        assert!(session_files_exist(&paths));
        remove_session_files(&paths).unwrap();
        assert!(!session_files_exist(&paths));
        remove_session_files(&paths).unwrap(); // idempotent
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn old_versions_and_missing_fields_load() {
        // A version-0 style file: no format/version/next_capture_id, captures without any of the
        // optional fields, one without raw_pairs/crc_valid/status/id.
        let json = r#"{
            "captures": [
                {"id": 5, "timestamp": "2025-01-02T03:04:05Z", "frequency": 433920000,
                 "protocol": "Kia V1", "serial": 1234, "button": 1, "counter": 7, "crc_valid": true,
                 "data": 99, "data_count_bit": 57, "raw_pairs": [{"level": true, "duration_us": 800}],
                 "status": "EncoderCapable"},
                {"frequency": 315000000, "protocol": "PSA2", "data": 42, "data_count_bit": 80}
            ]
        }"#;
        let s = parse_session(json).unwrap();
        assert_eq!(s.version, 0);
        assert_eq!(s.skipped, 0);
        assert_eq!(s.captures.len(), 2);
        let a = &s.captures[0];
        assert_eq!((a.id, a.counter, a.status), (5, Some(7), CaptureStatus::EncoderCapable));
        assert!(a.data_extra.is_none() && a.received_rf.is_none() && a.year.is_none() && a.source_file.is_none());
        let b = &s.captures[1];
        assert_eq!(b.id, 6, "missing id renumbered after the highest");
        assert_eq!(b.status, CaptureStatus::Decoded);
        assert!(b.raw_pairs.is_empty() && !b.crc_valid && b.serial.is_none());
        assert_eq!(s.next_capture_id, 7);

        // A newer version with unknown fields still loads.
        let newer = r#"{"format": "kat-session", "version": 99, "next_capture_id": 3, "future": [1, 2],
            "captures": [{"id": 1, "timestamp": "2026-01-01T00:00:00Z", "frequency": 433920000,
            "protocol": null, "serial": null, "button": null, "counter": null, "crc_valid": false,
            "data": 0, "data_count_bit": 0, "raw_pairs": [], "status": "Unknown",
            "tx_profile": {"deviation_hz": 47600}}]}"#;
        let s = parse_session(newer).unwrap();
        assert_eq!((s.version, s.captures.len(), s.next_capture_id), (99, 1, 3));
    }

    #[test]
    fn corrupt_files_are_reported_not_fatal() {
        let dir = temp_dir("corrupt");
        let paths = SessionPaths::in_dir(&dir);
        for bad in [
            "",
            "garbage \u{0}\u{1}",
            "{\"format\": \"kat-session\", \"captures\": [",
            "[1, 2, 3]",
            "{\"format\": \"kat-fob\", \"version\": \"2.0\"}",
            "{\"hello\": 1}",
        ] {
            fs::write(&paths.file, bad).unwrap();
            assert!(matches!(load_session(&paths.file), SessionLoad::Unreadable(_)), "{bad:?}");
        }
        // Bad bytes that are not UTF-8.
        fs::write(&paths.file, [0xFFu8, 0xFE, 0x00, 0x7B]).unwrap();
        assert!(matches!(load_session(&paths.file), SessionLoad::Unreadable(_)));
        // One broken capture among good ones: the rest load, the broken one is counted.
        let good = serde_json::to_string(&SessionSnapshot::new(&[full_capture(1), full_capture(2)], 3, 0)).unwrap();
        let mut v: serde_json::Value = serde_json::from_str(&good).unwrap();
        v["captures"].as_array_mut().unwrap().push(serde_json::json!({"id": 9, "status": "Bogus", "frequency": 1}));
        v["captures"].as_array_mut().unwrap().push(serde_json::json!("not an object"));
        v["captures"][0]["frequency"] = serde_json::Value::Null; // no frequency → skipped
        let s = parse_session(&v.to_string()).unwrap();
        assert_eq!((s.captures.len(), s.skipped), (1, 3));
        assert_eq!(s.captures[0].id, 2);
        let _ = fs::remove_dir_all(&dir);
    }

    /// Diagnostic: load time of a worst-case session (100 captures × 4096 pairs).
    /// `cargo test session_load_time -- --ignored --nocapture` (add `--release` for release numbers)
    #[test]
    #[ignore]
    fn session_load_time() {
        let captures: Vec<Capture> = (1..=100)
            .map(|id| {
                let mut c = full_capture(id);
                c.raw_pairs = (0..4096u32)
                    .map(|i| StoredLevelDuration { level: i % 2 == 0, duration_us: 200 + (i * 37) % 900 })
                    .collect();
                c
            })
            .collect();
        let dir = temp_dir("loadtime");
        let paths = SessionPaths::in_dir(&dir);
        let t = Instant::now();
        write_session_atomic(&paths, &SessionSnapshot::new(&captures, 101, 0)).unwrap();
        let write = t.elapsed();
        let size = fs::metadata(&paths.file).unwrap().len();
        let t = Instant::now();
        let SessionLoad::Loaded(s) = load_session(&paths.file) else { panic!() };
        let load = t.elapsed();
        let t = Instant::now();
        let fp = fingerprint(&s.captures, 101);
        let fp_time = t.elapsed();
        let t = Instant::now();
        let snap = SessionSnapshot::new(&s.captures, 101, 0);
        let snap_time = t.elapsed();
        eprintln!(
            "size {} bytes, write {:?}, load {:?}, fingerprint {:?} ({fp:x}), snapshot clone {:?}, {} captures",
            size, write, load, fp_time, snap_time, snap.captures.len()
        );
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn duplicate_ids_are_renumbered() {
        let json = serde_json::to_string(&SessionSnapshot::new(&[full_capture(4), full_capture(4), full_capture(2)], 1, 0)).unwrap();
        let s = parse_session(&json).unwrap();
        let ids: Vec<u32> = s.captures.iter().map(|c| c.id).collect();
        assert_eq!(ids, vec![4, 5, 2]);
        assert_eq!(s.next_capture_id, 6);
    }

    #[test]
    fn max_captures_keeps_the_newest_on_save_and_restore() {
        let captures: Vec<Capture> = (1..=5).map(full_capture).collect();
        let snap = SessionSnapshot::new(&captures, 6, 3);
        assert_eq!(snap.captures.iter().map(|c| c.id).collect::<Vec<_>>(), vec![3, 4, 5]);
        assert_eq!(SessionSnapshot::new(&captures, 6, 0).captures.len(), 5);

        let loaded = parse_session(&serde_json::to_string(&SessionSnapshot::new(&captures, 6, 0)).unwrap()).unwrap();
        let mut list = Vec::new();
        let mut next = 1;
        assert_eq!(restore_into(&mut list, &mut next, loaded, 2), 2);
        assert_eq!(list.iter().map(|c| c.id).collect::<Vec<_>>(), vec![4, 5]);
        assert_eq!(next, 6);
    }

    #[test]
    fn restore_into_a_non_empty_list_keeps_ids_unique() {
        let loaded = parse_session(&serde_json::to_string(&SessionSnapshot::new(&[full_capture(1), full_capture(2)], 3, 0)).unwrap()).unwrap();
        let mut list = vec![full_capture(2)];
        let mut next = 3;
        assert_eq!(restore_into(&mut list, &mut next, loaded, 0), 2);
        assert_eq!(list.iter().map(|c| c.id).collect::<Vec<_>>(), vec![2, 1, 3]);
        assert_eq!(next, 4);
    }

    #[test]
    fn fingerprint_tracks_user_visible_changes() {
        let mut captures = vec![full_capture(1), full_capture(2)];
        let fp = fingerprint(&captures, 3);
        assert_eq!(fp, fingerprint(&captures.clone(), 3));
        assert_ne!(fp, fingerprint(&captures, 4));
        captures[1].model = Some("Passat".to_string());
        let fp2 = fingerprint(&captures, 3);
        assert_ne!(fp, fp2);
        captures[0].serial = None;
        let fp3 = fingerprint(&captures, 3);
        assert_ne!(fp2, fp3);
        assert_ne!(fingerprint(&captures[..1], 3), fingerprint(&captures, 3));
        // A key search that found nothing changes no other field, and must be saved too.
        captures[0].key_recovery = Some(crate::capture::KeyRecovery::NotFound("PSA BF1/BF2".to_string()));
        assert_ne!(fp3, fingerprint(&captures, 3));
    }

    #[test]
    fn autosave_debounces_to_the_interval() {
        let mut a = Autosave::new(Duration::from_secs(1));
        let t0 = Instant::now();
        a.set_saved(1);
        assert!(!a.due(1, t0), "unchanged state is not saved");
        assert!(a.due(2, t0));
        a.mark_written(2, t0);
        assert!(!a.due(3, t0 + Duration::from_millis(400)), "within the interval");
        assert!(a.due(3, t0 + Duration::from_millis(1000)));
        assert!(!a.due(2, t0 + Duration::from_secs(5)));
    }

    #[test]
    fn writer_thread_saves_coalesces_and_clears_in_order() {
        let dir = temp_dir("writer");
        let paths = SessionPaths::in_dir(&dir);
        let mut w = SessionWriter::spawn(paths.clone());
        for n in 1..=5 {
            let captures: Vec<Capture> = (1..=n).map(full_capture).collect();
            w.send(WriteJob::Save(SessionSnapshot::new(&captures, n + 1, 0)));
        }
        w.finish(None);
        let SessionLoad::Loaded(s) = load_session(&paths.file) else { panic!("saved") };
        assert_eq!(s.captures.len(), 5, "the newest snapshot wins");
        assert!(w.poll_errors().is_empty());

        // A clear queued after a save deletes everything; a save after the clear is written.
        fs::write(&paths.backup, "old").unwrap();
        let mut w = SessionWriter::spawn(paths.clone());
        w.send(WriteJob::Save(SessionSnapshot::new(&[full_capture(1)], 2, 0)));
        w.send(WriteJob::Clear);
        w.finish(None);
        assert!(!session_files_exist(&paths));
        let mut w = SessionWriter::spawn(paths.clone());
        w.send(WriteJob::Clear);
        w.finish(Some(WriteJob::Save(SessionSnapshot::new(&[full_capture(7)], 8, 0))));
        assert!(matches!(load_session(&paths.file), SessionLoad::Loaded(s) if s.captures[0].id == 7));
        drop(w);

        // Write errors are reported, not fatal: the "file" path is a directory.
        let bad = SessionPaths { file: dir.join("as_dir"), tmp: dir.join("as_dir.tmp"), backup: dir.join("x.bak") };
        fs::create_dir_all(bad.file.join("inner")).unwrap();
        let mut w = SessionWriter::spawn(bad);
        w.send(WriteJob::Save(SessionSnapshot::new(&[full_capture(1)], 2, 0)));
        w.finish(None);
        assert_eq!(w.poll_errors().len(), 1);
        let _ = fs::remove_dir_all(&dir);
    }
}
