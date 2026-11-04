//! IQ recording (`:record`): the raw samples exactly as the device delivers them (HackRF `.cs8`,
//! RTL-SDR `.cu8`), written by a separate thread so the receive path never waits for the disk.
//!
//! The receive thread hands each buffer over with [`Recorder::push`] (copy + `try_send` on a
//! bounded channel); if the writer falls more than [`QUEUE_BUFFERS`] buffers behind, buffers are
//! dropped and counted instead of blocking USB transfers.
//!
//! File names carry the true centre frequency of the samples (the LO, which with offset tuning is
//! below the target) and the rate, so `hackrf_transfer -t`, inspectrum or URH see the right
//! frequency; when the target differs, `rx<target>Hz` records it so `:iq` decodes the right channel:
//! `kat_433670000Hz_2000000sps_rx433920000Hz_20261104-153000.cs8`.

use std::fs::OpenOptions;
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{sync_channel, SyncSender, TrySendError};
use std::sync::Arc;
use std::thread::JoinHandle;

use super::dsp::IqFormat;

/// Buffers the writer may fall behind before buffers are dropped (≈ 4 s of HackRF at 2 MS/s).
pub const QUEUE_BUFFERS: usize = 64;

/// What a finished recording wrote.
#[derive(Debug, Clone)]
pub struct RecordingSummary {
    pub path: PathBuf,
    pub bytes: u64,
    pub dropped_buffers: u64,
    pub error: Option<String>,
}

/// A recording in progress (shared view for the UI).
#[derive(Debug, Clone)]
pub struct RecordingStatus {
    pub path: PathBuf,
    pub bytes: u64,
    pub dropped_buffers: u64,
}

/// An open recording.
pub struct Recorder {
    tx: Option<SyncSender<Vec<u8>>>,
    handle: Option<JoinHandle<Result<(), String>>>,
    path: PathBuf,
    bytes: Arc<AtomicU64>,
    dropped: Arc<AtomicU64>,
}

impl Recorder {
    /// Create `path` (and its directory) now, so errors surface immediately, and start the writer.
    /// An existing file is never truncated (the caller picks a free name, [resolve_record_path]).
    pub fn start(path: PathBuf) -> std::io::Result<Self> {
        if let Some(dir) = path.parent() {
            if !dir.as_os_str().is_empty() {
                std::fs::create_dir_all(dir)?;
            }
        }
        let file = OpenOptions::new().write(true).create_new(true).open(&path)?;
        let (tx, rx) = sync_channel::<Vec<u8>>(QUEUE_BUFFERS);
        let bytes = Arc::new(AtomicU64::new(0));
        let written = bytes.clone();
        let handle = std::thread::Builder::new()
            .name("kat-iq-writer".to_string())
            .spawn(move || {
                let mut w = BufWriter::with_capacity(1 << 20, file);
                for buf in rx {
                    w.write_all(&buf).map_err(|e| e.to_string())?;
                    written.fetch_add(buf.len() as u64, Ordering::Relaxed);
                }
                w.flush().map_err(|e| e.to_string())
            })?;
        Ok(Self { tx: Some(tx), handle: Some(handle), path, bytes, dropped: Arc::new(AtomicU64::new(0)) })
    }

    /// Queue a copy of `data` for writing; never blocks.
    pub fn push(&self, data: &[u8]) {
        if let Some(tx) = &self.tx {
            match tx.try_send(data.to_vec()) {
                Ok(()) => {}
                Err(TrySendError::Full(_)) | Err(TrySendError::Disconnected(_)) => {
                    self.dropped.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
    }

    pub fn status(&self) -> RecordingStatus {
        RecordingStatus {
            path: self.path.clone(),
            bytes: self.bytes.load(Ordering::Relaxed),
            dropped_buffers: self.dropped.load(Ordering::Relaxed),
        }
    }

    /// Close the channel, let the writer drain and flush, and report.
    pub fn stop(mut self) -> RecordingSummary {
        drop(self.tx.take());
        let error = match self.handle.take().map(|h| h.join()) {
            Some(Ok(Ok(()))) | None => None,
            Some(Ok(Err(e))) => Some(e),
            Some(Err(_)) => Some("writer thread panicked".to_string()),
        };
        RecordingSummary {
            path: self.path.clone(),
            bytes: self.bytes.load(Ordering::Relaxed),
            dropped_buffers: self.dropped.load(Ordering::Relaxed),
            error,
        }
    }
}

impl Drop for Recorder {
    fn drop(&mut self) {
        drop(self.tx.take());
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

/// `kat_<centre>Hz_<rate>sps[_rx<target>Hz]_<UTC timestamp>.<ext>`
pub fn recording_file_name(centre_hz: u32, rate: u32, target_hz: u32, format: IqFormat) -> String {
    let ts = chrono::Utc::now().format("%Y%m%d-%H%M%S");
    let target = if target_hz != centre_hz { format!("_rx{target_hz}Hz") } else { String::new() };
    format!("kat_{centre_hz}Hz_{rate}sps{target}_{ts}.{}", format.extension())
}

/// Resolve a user-supplied `:record` path: relative paths go into `dir`, a folder gets
/// `default_name` in it, and the extension always names the device's format, because `:iq` and
/// other tools read the samples by it: a wrong IQ extension is replaced (`x.cu8` on a HackRF holds
/// cs8 samples, which decode as garbage as cu8) and any other one is kept with the format's added
/// (`fob_433.92` → `fob_433.92.cs8`, not a `.92` file KAT cannot load). The caller still picks a
/// free name ([crate::export::unique_path]).
pub fn resolve_record_path(dir: &Path, user: &str, format: IqFormat, default_name: &str) -> PathBuf {
    let expanded = if let Some(rest) = user.strip_prefix("~/") {
        dirs::home_dir().map(|h| h.join(rest)).unwrap_or_else(|| PathBuf::from(user))
    } else {
        PathBuf::from(user)
    };
    let mut path = if expanded.is_absolute() { expanded } else { dir.join(expanded) };
    if path.is_dir() || user.ends_with('/') {
        return path.join(default_name);
    }
    match IqFormat::from_path(&path) {
        Some(f) if f == format => {}
        Some(_) => {
            path.set_extension(format.extension());
        }
        None => {
            let mut name = path.file_name().unwrap_or_default().to_os_string();
            name.push(".");
            name.push(format.extension());
            path.set_file_name(name);
        }
    }
    path
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("kat-rec-test-{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        dir
    }

    #[test]
    fn records_bytes_in_order() {
        let dir = scratch("order");
        let path = dir.join("x.cs8");
        let rec = Recorder::start(path.clone()).unwrap();
        for k in 0..10u8 {
            rec.push(&[k; 1000]);
        }
        let sum = rec.stop();
        assert_eq!(sum.bytes, 10_000);
        assert_eq!(sum.dropped_buffers, 0);
        assert!(sum.error.is_none());
        let data = std::fs::read(&path).unwrap();
        assert_eq!(data.len(), 10_000);
        assert!(data.chunks(1000).enumerate().all(|(k, c)| c.iter().all(|&b| b == k as u8)));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn file_names_and_paths() {
        let n = recording_file_name(433_670_000, 2_000_000, 433_920_000, IqFormat::Cs8);
        assert!(n.starts_with("kat_433670000Hz_2000000sps_rx433920000Hz_") && n.ends_with(".cs8"), "{n}");
        let n = recording_file_name(433_920_000, 2_048_000, 433_920_000, IqFormat::Cu8);
        assert!(n.starts_with("kat_433920000Hz_2048000sps_2") && n.ends_with(".cu8"), "{n}");
        let p = resolve_record_path(Path::new("/x/exports"), "mine", IqFormat::Cu8, "d.cu8");
        assert_eq!(p, PathBuf::from("/x/exports/mine.cu8"));
        // A wrong IQ extension is replaced by the device's format; the right one (any case) kept.
        let p = resolve_record_path(Path::new("/x/exports"), "/tmp/a.cs8", IqFormat::Cu8, "d.cu8");
        assert_eq!(p, PathBuf::from("/tmp/a.cu8"));
        let p = resolve_record_path(Path::new("/x/exports"), "x.cu8", IqFormat::Cs8, "d.cs8");
        assert_eq!(p, PathBuf::from("/x/exports/x.cs8"));
        let p = resolve_record_path(Path::new("/x/exports"), "Fob.CS8", IqFormat::Cs8, "d.cs8");
        assert_eq!(p, PathBuf::from("/x/exports/Fob.CS8"));
        // Any other extension stays and the format's is added.
        let p = resolve_record_path(Path::new("/x/exports"), "fob_433.92", IqFormat::Cs8, "d.cs8");
        assert_eq!(p, PathBuf::from("/x/exports/fob_433.92.cs8"));
        // A folder gets the default name.
        let p = resolve_record_path(Path::new("/x/exports"), "runs/", IqFormat::Cs8, "kat_1Hz.cs8");
        assert_eq!(p, PathBuf::from("/x/exports/runs/kat_1Hz.cs8"));
    }

    /// A recording never truncates an existing file.
    #[test]
    fn never_overwrites_a_file() {
        let dir = scratch("exists");
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("x.cs8");
        std::fs::write(&path, b"keep").unwrap();
        assert_eq!(Recorder::start(path.clone()).err().map(|e| e.kind()), Some(std::io::ErrorKind::AlreadyExists));
        assert_eq!(std::fs::read(&path).unwrap(), b"keep");
        let _ = std::fs::remove_dir_all(&dir);
    }
}
