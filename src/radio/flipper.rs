//! A Flipper Zero as KAT's radio, through its USB console.
//!
//! The Flipper's command line (the USB CDC serial port qFlipper also uses) has what a radio needs,
//! in every firmware KAT follows (official, Momentum, RogueMaster, Flipper-ARF, FoxFW,
//! doctor-firmware), with no Flipper app to install:
//!
//! - **Receive:** `subghz rx_raw <Hz>` streams the CC1101's demodulated output, `+350 -700 …` (HIGH
//!   / LOW µs; `.` where the Flipper's buffer overran), until Ctrl+C. The preset is the firmware's:
//!   OOK with a 270 kHz filter (official, Momentum, RogueMaster) or 650 kHz (Flipper-ARF, FoxFW).
//!   doctor-firmware's takes `<Hz> <device> <preset>`, FM presets included (config
//!   `flipper_preset`; KAT checks the firmware's usage text before using it). The stream never
//!   goes quiet: the AGC turns every pause into noise pulses, thousands a second. So the decoders
//!   read it as it comes ([StreamLocator]), each frame they find is cut out of the noise
//!   ([RxStream::frame_start]: the shortest stretch its decoder still reads, extended back over a
//!   regular preamble) and handed to the app as a burst, which decodes and lists it like a HackRF's.
//! - **Transmit:** KAT writes the transmission (its own encoders, copies and timing) as a RAW
//!   `.sub` on the SD card (`storage write_chunk`, into `/ext/apps_data/kat`, out of the Sub-GHz
//!   app's list) and plays it with `subghz tx_from_file <file> 1 0`. The file's preset sets the
//!   modulation ([tx_preset]): OOK, or 2-FSK at the transmission's deviation. Receiving pauses.
//! - **Not available:** a signal level (the meter stays empty), IQ (`:record`) and hopping (each
//!   retune restarts `rx_raw`). FM reception needs doctor-firmware; transmitting works in both
//!   modulations on every firmware (the region lock of stock firmware applies).
//!
//! One thread owns the port ([run_worker]); the controller's calls are requests to it, answered
//! when done, so `start_receiving` and `transmit` return the Flipper's refusal as an error. The
//! port is opened on first use ([FlipperController::connect]), not when KAT starts: until then
//! another program (qFlipper, a terminal) can have it, and KAT sends it nothing.

use std::collections::VecDeque;
use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, Sender, TryRecvError};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use anyhow::{anyhow, bail, Result};

use crate::app::RadioEvent;
use crate::capture::{Capture, OnAir, RfModulation, StoredLevelDuration};
use crate::protocols::{LocatedFrame, StreamLocator};

use super::demodulator::LevelDuration;
use super::devices::FoundRadio;
use super::dsp::IqFormat;
use super::hopper::HopPlan;
use super::preset::{self, PresetDef, RfPreset, TxModulation};
use super::recorder::{RecordingStatus, RecordingSummary};
use super::tx::TxSettings;

/// The console's prompt, after every command.
const PROMPT: &[u8] = b"\r\n>: ";
/// Where KAT puts the file it transmits: the Flipper's app-data folder.
const TX_DIR: &str = "/ext/apps_data/kat";
const TX_FILE: &str = "/ext/apps_data/kat/tx.sub";
/// `storage write_chunk` size (scripts/flipper_replay.py sends every test file this way).
const CHUNK: usize = 512;
/// How long one read of the port waits for bytes (the worker's reaction time).
const READ_TIMEOUT: Duration = Duration::from_millis(20);
/// Pulses after a frame's last one that go into its burst: the app's decode reads up to two more
/// (a frame reported on the middle of its last bit, a decoder holding a frame for one pulse).
const TAIL_PULSES: u64 = 2;
/// How far back from its last pulse a frame's start is looked for (about 3 s of dense noise).
const MAX_LOOKBACK: u64 = 20_000;
/// At most this many regular pulses are added before a frame (its preamble, see frame_start).
const MAX_PREAMBLE_EXTENSION: usize = 4_000;

// ---- the port ------------------------------------------------------------------------------------

/// Every Flipper's USB console, with its USB serial when the system gives it: `configured` (config
/// `flipper_port`) alone when one is, else the serial ports whose USB descriptor names a Flipper
/// (manufacturer "Flipper Devices Inc.", product "Flipper <name>"; its ST VCP id 0483:5740 where
/// the system gives no names), else on Linux the `/dev/serial/by-id/usb-Flipper_Devices_Inc._*`
/// links.
pub fn find_ports(configured: Option<&str>) -> Vec<(String, Option<String>)> {
    if let Some(p) = configured.map(str::trim).filter(|p| !p.is_empty()) {
        return vec![(p.to_string(), None)];
    }
    let mut found: Vec<(String, Option<String>)> = serialport::available_ports()
        .unwrap_or_default()
        .into_iter()
        .filter_map(|p| match &p.port_type {
            serialport::SerialPortType::UsbPort(u)
                if is_flipper(u.vid, u.pid, u.manufacturer.as_deref(), u.product.as_deref()) =>
            {
                Some((p.port_name, u.serial_number.clone().filter(|s| !s.trim().is_empty())))
            }
            _ => None,
        })
        // macOS lists each port twice; the call-out device (cu.*) is the one to open.
        .filter(|(name, _)| !name.starts_with("/dev/tty."))
        .collect();
    found.sort();
    found.dedup_by(|a, b| a.0 == b.0);
    #[cfg(target_os = "linux")]
    if found.is_empty() {
        if let Ok(dir) = std::fs::read_dir("/dev/serial/by-id") {
            found = dir
                .flatten()
                .filter(|e| e.file_name().to_string_lossy().starts_with("usb-Flipper_Devices_Inc."))
                .map(|e| (e.path().to_string_lossy().into_owned(), None))
                .collect();
            found.sort();
        }
    }
    found
}

/// The Flipper Zeros on USB ([super::devices::discover]), not opened: KAT opens a Flipper's
/// console when it first uses it ([FlipperController::connect]).
pub fn find_devices(configured: Option<&str>) -> (Vec<FoundRadio>, Option<String>) {
    // A configured port that is not there (on Windows a COM port is no file to look for).
    #[cfg(unix)]
    if let Some(p) = configured.map(str::trim).filter(|p| !p.is_empty() && !std::path::Path::new(p).exists()) {
        return (Vec::new(), Some(format!("not found (flipper_port {p} does not exist)")));
    }
    let found: Vec<FoundRadio> =
        find_ports(configured).into_iter().map(|(port, serial)| FoundRadio::flipper(port, serial)).collect();
    if found.is_empty() {
        (found, Some("not found".to_string()))
    } else {
        tracing::info!("Flipper(s) found: {}", found.iter().filter_map(|f| f.port.clone()).collect::<Vec<_>>().join(", "));
        (found, None)
    }
}

fn is_flipper(vid: u16, pid: u16, manufacturer: Option<&str>, product: Option<&str>) -> bool {
    let named = |s: Option<&str>| s.is_some_and(|s| s.contains("Flipper"));
    named(manufacturer) || named(product) || (vid == 0x0483 && pid == 0x5740)
}

/// Why `port` could not be opened, in words (the no-device popup and the log show it).
fn describe_open_error(port: &str, e: &serialport::Error) -> String {
    let text = e.to_string();
    if text.contains("busy") || text.contains("Busy") || (cfg!(windows) && text.contains("Access is denied")) {
        format!("{port} is busy: qFlipper or another program has the Flipper's console")
    } else if matches!(e.kind(), serialport::ErrorKind::Io(std::io::ErrorKind::PermissionDenied)) {
        if cfg!(target_os = "linux") {
            format!("{port} is not accessible: add yourself to the dialout group (sudo usermod -aG dialout $USER) and log in again")
        } else {
            format!("{port} is not accessible")
        }
    } else if matches!(e.kind(), serialport::ErrorKind::NoDevice) {
        format!("{port} is gone (unplugged?)")
    } else {
        format!("{port} cannot be opened ({text})")
    }
}

fn find_bytes(hay: &[u8], needle: &[u8]) -> Option<usize> {
    hay.windows(needle.len()).position(|w| w == needle)
}

/// Console output as text: no terminal colour codes, no echo of the command line, trimmed.
fn clean_output(bytes: &[u8], command: &str) -> String {
    let text = String::from_utf8_lossy(bytes);
    let mut out = String::with_capacity(text.len());
    let mut chars = text.chars().peekable();
    while let Some(c) = chars.next() {
        if c == '\u{1b}' {
            // ESC [ … letter
            if chars.peek() == Some(&'[') {
                for d in chars.by_ref() {
                    if d.is_ascii_alphabetic() {
                        break;
                    }
                }
            }
            continue;
        }
        out.push(c);
    }
    let out = out.trim_start_matches(['\r', '\n']);
    let out = out.strip_prefix(command).unwrap_or(out);
    out.replace('\r', "").trim().to_string()
}

/// A session on the Flipper's command line.
struct Console {
    port: Box<dyn serialport::SerialPort>,
    /// Bytes read and not consumed yet.
    pending: Vec<u8>,
    /// `/ext/apps_data/kat` exists (made once per session).
    tx_dir_ready: bool,
}

impl Console {
    fn open(path: &str) -> Result<Self> {
        // DTR on: the Flipper starts a console session when the host raises it (Linux does on
        // open anyway; serialport leaves it off on Windows otherwise).
        let port = serialport::new(path, 115_200)
            .timeout(READ_TIMEOUT)
            .dtr_on_open(true)
            .open()
            .map_err(|e| anyhow!(describe_open_error(path, &e)))?;
        Self::with_port(port)
    }

    /// A session on an open port: past the banner of a new connection, or whatever an earlier
    /// session left running (Ctrl+C ends an `rx_raw`), to a fresh prompt.
    fn with_port(port: Box<dyn serialport::SerialPort>) -> Result<Self> {
        let mut c = Self { port, pending: Vec::new(), tx_dir_ready: false };
        c.send(b"\x03\r")?;
        if c.read_until(PROMPT, Duration::from_secs(3))?.is_none() {
            bail!("no answer from its console (a Flipper's command line?)");
        }
        // The banner, Ctrl+C and Enter each end in a prompt: start clean once the console has been
        // quiet for 150 ms.
        let (mut quiet_from, give_up) = (Instant::now(), Instant::now() + Duration::from_secs(2));
        while quiet_from.elapsed() < Duration::from_millis(150) && Instant::now() < give_up {
            if c.read_more()? > 0 {
                quiet_from = Instant::now();
            }
        }
        c.pending.clear();
        Ok(c)
    }

    fn send(&mut self, bytes: &[u8]) -> Result<()> {
        self.port.write_all(bytes)?;
        self.port.flush()?;
        Ok(())
    }

    /// Read what has arrived (waits up to [READ_TIMEOUT]); the number of bytes added.
    fn read_more(&mut self) -> Result<usize> {
        let mut buf = [0u8; 8192];
        match self.port.read(&mut buf) {
            Ok(0) => bail!("the Flipper's console closed"),
            Ok(n) => {
                self.pending.extend_from_slice(&buf[..n]);
                Ok(n)
            }
            Err(e) if matches!(e.kind(), std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock | std::io::ErrorKind::Interrupted) => Ok(0),
            Err(e) => Err(anyhow!("reading the Flipper's console failed: {e} (unplugged?)")),
        }
    }

    /// Read until one of `tokens` arrives: which one, and the bytes before it (both consumed).
    fn read_until_any(&mut self, tokens: &[&[u8]], timeout: Duration) -> Result<Option<(usize, Vec<u8>)>> {
        let end = Instant::now() + timeout;
        loop {
            let first = tokens
                .iter()
                .enumerate()
                .filter_map(|(i, t)| find_bytes(&self.pending, t).map(|pos| (pos, i, t.len())))
                .min();
            if let Some((pos, i, len)) = first {
                let before = self.pending[..pos].to_vec();
                self.pending.drain(..pos + len);
                return Ok(Some((i, before)));
            }
            if Instant::now() >= end {
                return Ok(None);
            }
            self.read_more()?;
        }
    }

    fn read_until(&mut self, token: &[u8], timeout: Duration) -> Result<Option<Vec<u8>>> {
        Ok(self.read_until_any(&[token], timeout)?.map(|(_, b)| b))
    }

    /// One command; its output.
    fn command(&mut self, line: &str, timeout: Duration) -> Result<String> {
        self.send(format!("{line}\r").as_bytes())?;
        match self.read_until(PROMPT, timeout)? {
            Some(out) => Ok(clean_output(&out, line)),
            None => {
                // Still running (a transmission that never ends?): stop it, so the console is usable.
                let _ = self.send(b"\x03");
                let _ = self.read_until(PROMPT, Duration::from_secs(2));
                bail!("`{line}` did not finish within {} s", timeout.as_secs())
            }
        }
    }

    /// Whether this firmware's `rx_raw` takes a preset (doctor-firmware's). Not a frequency, so it
    /// prints its usage line and receives nothing.
    fn rx_presets_supported(&mut self) -> Result<bool> {
        Ok(self.command("subghz rx_raw kat", Duration::from_secs(3))?.contains("Preset"))
    }

    /// Start `rx_raw`: Ok once the Flipper says it listens; its answer as the error when it refuses.
    fn start_rx(&mut self, frequency: u32, preset: Option<&str>) -> Result<()> {
        let line = match preset {
            Some(p) => format!("subghz rx_raw {frequency} 0 {p}"),
            None => format!("subghz rx_raw {frequency}"),
        };
        self.send(format!("{line}\r").as_bytes())?;
        match self.read_until_any(&[b"Listening at", PROMPT], Duration::from_secs(3))? {
            Some((0, _)) => {
                // The rest of that line ("433920000. Press CTRL+C to stop").
                let _ = self.read_until(b"\n", Duration::from_secs(1))?;
                Ok(())
            }
            Some((_, out)) => {
                let why = clean_output(&out, &line);
                bail!("the Flipper refused `{line}`: {}", if why.is_empty() { "no reason given".to_string() } else { why })
            }
            None => bail!("no answer to `{line}`"),
        }
    }

    /// Stop `rx_raw` (Ctrl+C); what it printed until the prompt came back.
    fn stop_rx(&mut self) -> Result<Vec<u8>> {
        self.send(b"\x03")?;
        let mut out = std::mem::take(&mut self.pending);
        match self.read_until(PROMPT, Duration::from_secs(3))? {
            Some(rest) => {
                out.extend(rest);
                Ok(out)
            }
            None => bail!("rx_raw did not stop"),
        }
    }

    /// Write `data` to `path` on the Flipper (`storage write_chunk` appends, so remove it first).
    fn write_file(&mut self, path: &str, data: &[u8]) -> Result<()> {
        self.command(&format!("storage remove {path}"), Duration::from_secs(5))?;
        for (i, chunk) in data.chunks(CHUNK).enumerate() {
            let line = format!("storage write_chunk {path} {}", chunk.len());
            self.send(format!("{line}\r").as_bytes())?;
            match self.read_until_any(&[b"Ready\r\n", PROMPT], Duration::from_secs(5))? {
                Some((0, _)) => {}
                Some((_, out)) => bail!("writing {path} on the Flipper failed: {}", clean_output(&out, &line)),
                None => bail!("writing {path} on the Flipper: no answer"),
            }
            self.send(chunk)?;
            let out = self
                .read_until(PROMPT, Duration::from_secs(10))?
                .ok_or_else(|| anyhow!("writing {path} on the Flipper: no answer after byte {}", i * CHUNK))?;
            let text = clean_output(&out, "");
            if text.contains("rror") {
                bail!("writing {path} on the Flipper failed: {text}");
            }
        }
        Ok(())
    }

    /// Put `file` on the SD card and play it once; the Flipper's refusal as the error.
    fn transmit(&mut self, file: &str, air_us: u64) -> Result<()> {
        if !self.tx_dir_ready {
            // "already exists" when they do; a missing SD card shows in the write below.
            for dir in ["/ext/apps_data", TX_DIR] {
                self.command(&format!("storage mkdir {dir}"), Duration::from_secs(5))?;
            }
            self.tx_dir_ready = true;
        }
        self.write_file(TX_FILE, file.as_bytes())?;
        // tx_from_file waits 200 ms before it keys the radio.
        let timeout = Duration::from_micros(air_us) + Duration::from_secs(10);
        let out = self.command(&format!("subghz tx_from_file {TX_FILE} 1 0"), timeout)?;
        match tx_refusal(&out) {
            Some(why) => Err(anyhow!("the Flipper did not transmit: {why}")),
            None => Ok(()),
        }
    }
}

/// `tx_from_file`'s output when it did not transmit: its error line (`subghz tx_from_file: …`), the
/// region lock, or the usage text of a firmware without the command.
fn tx_refusal(out: &str) -> Option<String> {
    for line in out.lines().map(str::trim) {
        if let Some(why) = line.strip_prefix("subghz tx_from_file:") {
            return Some(why.trim().to_string());
        }
        if line.contains("restricted") {
            return Some(line.to_string());
        }
        if line.contains("command not found") || line.starts_with("Usage") || line.contains("Cmd list") {
            return Some("this firmware has no `subghz tx_from_file`".to_string());
        }
    }
    None
}

// ---- receive ---------------------------------------------------------------------------------------

/// What `rx_raw` prints, read as it arrives (a token may be cut between two reads).
#[derive(Default)]
struct RawParser {
    word: Vec<u8>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum RawItem {
    Pulse(LevelDuration),
    /// `.`: the Flipper's buffer overran, pulses are missing here.
    Break,
}

impl RawParser {
    fn push(&mut self, bytes: &[u8], out: &mut Vec<RawItem>) {
        for &b in bytes {
            if b.is_ascii_whitespace() {
                self.end_word(out);
            } else if self.word.len() < 16 {
                self.word.push(b);
            }
        }
    }

    /// A word ends (whitespace, or the stream stopped): a pulse, a break, or text to skip.
    fn end_word(&mut self, out: &mut Vec<RawItem>) {
        match self.word.as_slice() {
            b"." => out.push(RawItem::Break),
            [sign @ (b'+' | b'-'), digits @ ..] if !digits.is_empty() && digits.iter().all(u8::is_ascii_digit) => {
                let duration = std::str::from_utf8(digits).ok().and_then(|d| d.parse::<u32>().ok());
                if let Some(d) = duration.filter(|&d| d > 0) {
                    out.push(RawItem::Pulse(LevelDuration::new(*sign == b'+', d)));
                }
            }
            _ => {}
        }
        self.word.clear();
    }
}

/// Receive processing: the stream's recent pulses, the decoders reading it, and the frames found
/// that wait for the pulses after them (see the module docs).
struct RxStream {
    frequency: u32,
    rf: RfModulation,
    session: u64,
    events: Sender<RadioEvent>,
    /// Normal polarity; on FSK also inverted (either tone can be "on").
    locators: Vec<StreamLocator>,
    /// Per locator: the first pulse its decoders have read since they were last reset.
    since: Vec<u64>,
    /// Recent pulses and their start (µs into the session); `pulses[0]` is pulse number `first`.
    pulses: VecDeque<(LevelDuration, u64)>,
    first: u64,
    /// Session time at the end of the last pulse.
    now_us: u64,
    /// Frames found and waiting for [TAIL_PULSES] more: (first pulse, last pulse).
    waiting: Vec<(u64, u64)>,
}

impl RxStream {
    fn new(frequency: u32, rf: RfModulation, report_unchecked: bool, events: Sender<RadioEvent>) -> Self {
        let polarities: &[bool] = if rf == RfModulation::FM { &[false, true] } else { &[false] };
        let locators: Vec<StreamLocator> = polarities
            .iter()
            .map(|&invert| StreamLocator::new(frequency, rf, invert, report_unchecked))
            .collect();
        Self {
            frequency,
            rf,
            session: super::rx::new_session(),
            events,
            since: vec![0; locators.len()],
            locators,
            pulses: VecDeque::new(),
            first: 0,
            now_us: 0,
            waiting: Vec::new(),
        }
    }

    /// Number of the next pulse.
    fn next(&self) -> u64 {
        self.first + self.pulses.len() as u64
    }

    fn push(&mut self, p: LevelDuration) {
        let at = self.next();
        self.pulses.push_back((p, self.now_us));
        self.now_us += u64::from(p.duration_us);
        for j in 0..self.locators.len() {
            let found = self.locators[j].feed(p.level, p.duration_us);
            self.found(j, found, at);
        }
        self.emit_ready(false);
        self.forget_old();
    }

    /// The stream broke (an overrun) or ended: frames held back for another pulse are reported,
    /// every waiting frame goes out, and no frame reaches back across this point.
    fn brk(&mut self) {
        if let Some(at) = self.next().checked_sub(1) {
            for j in 0..self.locators.len() {
                let found = self.locators[j].flush();
                self.found(j, found, at);
            }
        }
        self.emit_ready(true);
        let next = self.next();
        self.since.iter_mut().for_each(|s| *s = next);
    }

    fn found(&mut self, j: usize, frames: Vec<LocatedFrame>, at: u64) {
        for f in frames {
            let start = self.frame_start(j, &f, at);
            tracing::debug!("Flipper stream: {} frame on pulses {start}..={at}", f.name);
            self.waiting.push((start, at));
            // An unchecked PSA frame resets no decoder (see StreamLocator).
            if !f.unchecked {
                self.since[j] = at + 1;
            }
        }
    }

    /// Where the frame that `f`'s decoder completed on pulse `at` starts: the decoders read from
    /// their last reset, through the noise before the frame, so its pulses are those of the
    /// shortest stretch ending at `at` that the decoder alone still reads (a binary search), plus
    /// the regular pulses right before (the rest of a preamble the decoder did not need: pulse
    /// `i` as long as pulse `i + 2` within 20 %, which noise rarely is twice in a row).
    fn frame_start(&mut self, j: usize, f: &LocatedFrame, at: u64) -> u64 {
        let lo = self.since[j].max(self.first).max(at.saturating_sub(MAX_LOOKBACK));
        if lo >= at {
            return at;
        }
        let pulses: Vec<LevelDuration> =
            self.pulses.range((lo - self.first) as usize..=(at - self.first) as usize).map(|(p, _)| *p).collect();
        let loc = &mut self.locators[j];
        if !loc.reads(f, &pulses) {
            return lo;
        }
        // reads(pulses[ok..]) holds, reads(pulses[no..]) does not (an empty stretch never reads).
        let (mut ok, mut no) = (0usize, pulses.len());
        while no - ok > 1 {
            let mid = ok + (no - ok) / 2;
            if loc.reads(f, &pulses[mid..]) {
                ok = mid;
            } else {
                no = mid;
            }
        }
        let regular = |i: usize| {
            let (a, b) = (pulses[i], pulses[i + 2]);
            a.level == b.level && a.duration_us.abs_diff(b.duration_us) <= (b.duration_us / 5).max(40)
        };
        let mut start = ok;
        while start > 0 && start + 1 < pulses.len() && ok - start < MAX_PREAMBLE_EXTENSION && regular(start - 1) {
            start -= 1;
        }
        lo + start as u64
    }

    /// Hand the app the frames whose tail has arrived (all of them when `all`).
    fn emit_ready(&mut self, all: bool) {
        let next = self.next();
        let mut i = 0;
        while i < self.waiting.len() {
            let (start, end) = self.waiting[i];
            if all || end + TAIL_PULSES < next {
                self.waiting.remove(i);
                self.emit(start, (end + TAIL_PULSES).min(next.saturating_sub(1)));
            } else {
                i += 1;
            }
        }
    }

    /// Pulses `start..=end` as a burst: a capture with the frequency, modulation and on-air time
    /// (in this receive session), as an SDR's burst reaches the app.
    fn emit(&mut self, start: u64, end: u64) {
        if start < self.first || start > end || end >= self.next() {
            return;
        }
        let (a, b) = ((start - self.first) as usize, (end - self.first) as usize);
        let pairs: Vec<StoredLevelDuration> = self
            .pulses
            .range(a..=b)
            .map(|(p, _)| StoredLevelDuration { level: p.level, duration_us: p.duration_us })
            .collect();
        let start_us = self.pulses[a].1;
        let end_us = self.pulses[b].1 + u64::from(self.pulses[b].0.duration_us);
        let mut capture = Capture::from_pairs_with_rf(0, self.frequency, pairs, Some(self.rf));
        capture.on_air = Some(OnAir { session: self.session, start_us, end_us });
        let _ = self.events.send(RadioEvent::SignalCaptured(capture));
    }

    /// Drop pulses no frame can reach back to any more (in batches).
    fn forget_old(&mut self) {
        if (self.pulses.len() as u64) < MAX_LOOKBACK + 8_192 {
            return;
        }
        let keep = self.waiting.iter().map(|w| w.0).fold(self.next().saturating_sub(MAX_LOOKBACK + 1), u64::min);
        while self.first < keep && !self.pulses.is_empty() {
            self.pulses.pop_front();
            self.first += 1;
        }
    }
}

/// A running `rx_raw`.
struct Receiving {
    frequency: u32,
    parser: RawParser,
    stream: RxStream,
}

impl Receiving {
    fn start(console: &mut Console, frequency: u32, setup: &WorkerSetup) -> Result<Self> {
        console.start_rx(frequency, setup.preset.as_deref())?;
        let mut r = Self {
            frequency,
            parser: RawParser::default(),
            stream: RxStream::new(frequency, setup.rf, setup.report_unchecked, setup.events.clone()),
        };
        let early = std::mem::take(&mut console.pending);
        r.feed(&early);
        Ok(r)
    }

    /// Read and process what arrived.
    fn pump(&mut self, console: &mut Console) -> Result<()> {
        console.read_more()?;
        let bytes = std::mem::take(&mut console.pending);
        // `rx_raw` prints pulses, dots and line breaks only: a prompt means it ended.
        if find_bytes(&bytes, b">: ").is_some() {
            self.feed(&bytes);
            self.stream.brk();
            bail!("the Flipper ended rx_raw");
        }
        self.feed(&bytes);
        Ok(())
    }

    fn feed(&mut self, bytes: &[u8]) {
        let mut items = Vec::new();
        self.parser.push(bytes, &mut items);
        for item in items {
            match item {
                RawItem::Pulse(p) => self.stream.push(p),
                RawItem::Break => self.stream.brk(),
            }
        }
    }

    fn stop(mut self, console: &mut Console) -> Result<()> {
        let out = console.stop_rx();
        if let Ok(bytes) = &out {
            self.feed(bytes);
        }
        let mut items = Vec::new();
        self.parser.end_word(&mut items);
        for item in items {
            if let RawItem::Pulse(p) = item {
                self.stream.push(p);
            }
        }
        self.stream.brk();
        out.map(|_| ())
    }
}

// ---- transmit ---------------------------------------------------------------------------------------

/// The preset a transmission's file names: AM650 for OOK (the bandwidth only matters to a
/// receiver); for 2-FSK the preset KAT knows with that deviation (FM238 and FM476 by name, which
/// every firmware has, others as their register image), else FM476's image with the nearest
/// DEVIATN value.
pub fn tx_preset(modulation: TxModulation) -> RfPreset {
    let TxModulation::Fsk { deviation_hz } = modulation else {
        return RfPreset::from_def(&preset::AM650);
    };
    let same = |d: &&&PresetDef| {
        matches!(d.modulation(), TxModulation::Fsk { deviation_hz: x } if (x - deviation_hz).abs() < 1.0)
    };
    if let Some(def) = preset::ALL_PRESETS.iter().filter(same).min_by_key(|d| !d.universal) {
        return RfPreset::from_def(def);
    }
    let error = |v: &u8| (preset::deviation_hz(*v) - deviation_hz).abs();
    let deviatn = (0u8..=0x77).filter(|v| v & 0x08 == 0).min_by(|a, b| error(a).total_cmp(&error(b))).unwrap_or(0x47);
    let mut registers = preset::FM476.registers.to_vec();
    for pair in registers.chunks_exact_mut(2) {
        if pair[0] == 0 && pair[1] == 0 {
            break;
        }
        if pair[0] == preset::REG_DEVIATN {
            pair[1] = deviatn;
        }
    }
    RfPreset {
        name: preset::CUSTOM_PRESET_NAME.to_string(),
        module: Some(preset::CUSTOM_PRESET_MODULE.to_string()),
        data: Some(registers),
    }
}

/// The RAW `.sub` KAT plays on the Flipper: the transmission at `frequency` with the preset of
/// its modulation ([tx_preset]); adjacent pulses of one level merged, 512 values a line as the
/// firmwares write them.
pub fn tx_sub_text(signal: &[LevelDuration], frequency: u32, modulation: TxModulation) -> String {
    let preset = tx_preset(modulation);
    let mut text = format!(
        "Filetype: Flipper SubGhz RAW File\nVersion: 1\nFrequency: {frequency}\nPreset: {}\n",
        preset.name
    );
    if let (true, Some(data)) = (preset.is_custom(), &preset.data) {
        let module = preset.module.as_deref().unwrap_or(preset::CUSTOM_PRESET_MODULE);
        text.push_str(&format!("Custom_preset_module: {module}\nCustom_preset_data: {}\n", preset::format_hex_bytes(data)));
    }
    text.push_str("Protocol: RAW\n");
    let mut values: Vec<i64> = Vec::with_capacity(signal.len());
    for p in signal.iter().filter(|p| p.duration_us > 0) {
        let v = if p.level { i64::from(p.duration_us) } else { -i64::from(p.duration_us) };
        match values.last_mut() {
            Some(last) if (*last > 0) == p.level => *last += v,
            _ => values.push(v),
        }
    }
    for line in values.chunks(512) {
        text.push_str("RAW_Data:");
        for v in line {
            text.push_str(&format!(" {v}"));
        }
        text.push('\n');
    }
    text
}

// ---- the worker -------------------------------------------------------------------------------------

enum Request {
    Receive { frequency: u32, done: Sender<Result<()>> },
    Stop { done: Sender<()> },
    Transmit { file: String, air_us: u64, done: Sender<Result<()>> },
}

/// What the worker needs besides the port.
struct WorkerSetup {
    /// doctor-firmware's `rx_raw` preset; None = the firmware's own (OOK).
    preset: Option<String>,
    rf: RfModulation,
    report_unchecked: bool,
    events: Sender<RadioEvent>,
    receiving: Arc<AtomicBool>,
}

impl WorkerSetup {
    /// Receiving ended on its own (unplugged, the console closed): the app goes back to idle.
    fn fail(&self, e: &anyhow::Error) {
        self.receiving.store(false, Ordering::SeqCst);
        tracing::warn!("Flipper receiver stopped: {e:#}");
        let _ = self.events.send(RadioEvent::Error(format!("Flipper: {e:#}")));
    }
}

/// The thread that owns the port: streams `rx_raw` while receiving, and answers requests between
/// reads. Ends when the controller is dropped.
fn run_worker(mut console: Console, requests: Receiver<Request>, setup: WorkerSetup) {
    let mut rx: Option<Receiving> = None;
    loop {
        let request = match rx.as_mut() {
            Some(r) => {
                if let Err(e) = r.pump(&mut console) {
                    rx = None;
                    setup.fail(&e);
                    continue;
                }
                match requests.try_recv() {
                    Ok(req) => req,
                    Err(TryRecvError::Empty) => continue,
                    Err(TryRecvError::Disconnected) => break,
                }
            }
            None => match requests.recv() {
                Ok(req) => req,
                Err(_) => break,
            },
        };
        match request {
            Request::Receive { frequency, done } => {
                if let Some(r) = rx.take() {
                    let _ = r.stop(&mut console);
                }
                let result = Receiving::start(&mut console, frequency, &setup).map(|r| rx = Some(r));
                setup.receiving.store(result.is_ok(), Ordering::SeqCst);
                let _ = done.send(result);
            }
            Request::Stop { done } => {
                if let Some(r) = rx.take() {
                    let _ = r.stop(&mut console);
                }
                setup.receiving.store(false, Ordering::SeqCst);
                let _ = done.send(());
            }
            Request::Transmit { file, air_us, done } => {
                let resume = rx.take().map(|r| {
                    let f = r.frequency;
                    let _ = r.stop(&mut console);
                    f
                });
                let result = console.transmit(&file, air_us);
                if let Some(f) = resume {
                    match Receiving::start(&mut console, f, &setup) {
                        Ok(r) => rx = Some(r),
                        Err(e) => setup.fail(&e),
                    }
                }
                let _ = done.send(result);
            }
        }
    }
    if let Some(r) = rx.take() {
        let _ = r.stop(&mut console);
    }
}

// ---- the controller ---------------------------------------------------------------------------------

/// A Flipper Zero on USB as KAT's radio (see the module docs).
pub struct FlipperController {
    /// The console's port; None: no Flipper found.
    port: Option<String>,
    /// What connecting needs: the app's events, the receive preset (config `flipper_preset`) and
    /// whether unchecked PSA frames are reported (`autobf`).
    events: Sender<RadioEvent>,
    preset: Option<String>,
    report_unchecked: bool,
    /// To the thread that owns the open console; None until connected ([Self::connect]).
    requests: Option<Sender<Request>>,
    worker: Option<JoinHandle<()>>,
    receiving: Arc<AtomicBool>,
    frequency: u32,
    open_error: Option<String>,
}

impl FlipperController {
    /// The Flipper on `port`, not connected yet: KAT opens its console when it first receives or
    /// transmits with it ([Self::connect]), so a Flipper plugged in beside an SDR stays free for
    /// qFlipper until it is used. `report_unchecked`: `autobf` is on (see [StreamLocator::new]).
    pub fn on_port(events: Sender<RadioEvent>, port: String, preset: Option<&str>, report_unchecked: bool) -> Self {
        Self {
            port: Some(port),
            events,
            preset: preset.map(str::trim).filter(|p| !p.is_empty()).map(str::to_string),
            report_unchecked,
            requests: None,
            worker: None,
            receiving: Arc::new(AtomicBool::new(false)),
            frequency: 433_920_000,
            open_error: None,
        }
    }

    /// Find the Flipper's console (`port` when configured) and connect to it at once. Never fails:
    /// a Flipper that cannot be used is [Self::is_available] false with [Self::open_error] saying why.
    #[cfg(all(test, unix))]
    pub fn new(events: Sender<RadioEvent>, port: Option<&str>, preset: Option<&str>, report_unchecked: bool) -> Self {
        let Some((path, _)) = find_ports(port).into_iter().next() else {
            let mut c = Self::on_port(events, String::new(), preset, report_unchecked);
            c.port = None;
            c.open_error = Some("not found".to_string());
            return c;
        };
        let mut c = Self::on_port(events, path, preset, report_unchecked);
        let _ = c.connect();
        c
    }

    /// Open the console, check the receive preset (doctor-firmware only) and start the thread that
    /// owns the port, unless that is done. Err: why the Flipper cannot be used (also kept as
    /// [Self::open_error]); the next use tries again (qFlipper closed, the Flipper plugged back in).
    pub fn connect(&mut self) -> Result<()> {
        if self.requests.is_some() {
            return Ok(());
        }
        let Some(path) = self.port.clone() else {
            bail!("no Flipper Zero found");
        };
        let result = Console::open(&path).and_then(|console| self.run_on(console));
        match &result {
            Ok(()) => {
                tracing::info!("Flipper Zero console on {path}");
                self.open_error = None;
            }
            Err(e) => {
                tracing::warn!("Flipper Zero on {path} not usable: {e:#}");
                self.open_error = Some(format!("{e:#}"));
            }
        }
        result
    }

    /// Whether KAT has the console open (from the first use until it quits).
    pub fn is_connected(&self) -> bool {
        self.requests.is_some()
    }

    /// Start the worker on an open console session.
    fn run_on(&mut self, mut console: Console) -> Result<()> {
        let (preset, rf) = match self.preset.as_deref() {
            None => (None, RfModulation::AM),
            Some(p) => {
                if !console.rx_presets_supported()? {
                    bail!("flipper_preset = {p}: this firmware's rx_raw takes no preset (doctor-firmware's does); remove it from config.ini");
                }
                (Some(p.to_string()), preset_rf(p))
            }
        };
        let (requests, inbox) = mpsc::channel();
        let setup = WorkerSetup {
            preset,
            rf,
            report_unchecked: self.report_unchecked,
            events: self.events.clone(),
            receiving: self.receiving.clone(),
        };
        let worker = std::thread::Builder::new()
            .name("kat-flipper".to_string())
            .spawn(move || run_worker(console, inbox, setup))?;
        self.requests = Some(requests);
        self.worker = Some(worker);
        Ok(())
    }

    /// The controller on an open console session.
    #[cfg(all(test, unix))]
    fn start(console: Console, events: Sender<RadioEvent>, preset: Option<&str>, report_unchecked: bool) -> Result<Self> {
        let mut c = Self::on_port(events, String::new(), preset, report_unchecked);
        c.port = None;
        c.run_on(console)?;
        Ok(c)
    }

    /// One request to the worker, waiting up to `timeout` for its answer.
    fn ask<T>(&self, make: impl FnOnce(Sender<T>) -> Request, timeout: Duration) -> Result<T> {
        let requests = self.requests.as_ref().ok_or_else(|| anyhow!("no Flipper"))?;
        let (done, answer) = mpsc::channel();
        requests.send(make(done)).map_err(|_| anyhow!("the Flipper's console is closed"))?;
        answer.recv_timeout(timeout).map_err(|_| anyhow!("the Flipper did not answer"))
    }

    /// Why the Flipper could not be used, in words (None when it can, or was not tried yet).
    pub fn open_error(&self) -> Option<&str> {
        self.open_error.as_deref()
    }

    /// Connected and usable.
    #[cfg(all(test, unix))]
    pub fn is_available(&self) -> bool {
        self.requests.is_some()
    }

    /// A Flipper transmits (connecting to it may still fail).
    pub fn supports_tx(&self) -> bool {
        self.port.is_some() || self.requests.is_some()
    }

    pub fn start_receiving(&mut self, frequency: u32) -> Result<()> {
        self.frequency = frequency;
        self.connect()?;
        self.ask(|done| Request::Receive { frequency, done }, Duration::from_secs(10))?
    }

    pub fn stop_receiving(&mut self) -> Result<()> {
        if self.requests.is_none() {
            return Ok(());
        }
        self.ask(|done| Request::Stop { done }, Duration::from_secs(10))
    }

    /// A new frequency; `rx_raw` restarts on it while receiving.
    pub fn set_frequency(&mut self, frequency: u32) -> Result<()> {
        self.frequency = frequency;
        if self.receiving.load(Ordering::SeqCst) {
            self.start_receiving(frequency)?;
        }
        Ok(())
    }

    /// Transmit `signal` on the Flipper (see the module docs); its refusal as the error.
    pub fn transmit(&mut self, signal: &[LevelDuration], frequency: u32, settings: &TxSettings) -> Result<()> {
        let air_us = super::tx::check_tx_air_time(signal)?;
        self.connect()?;
        let file = tx_sub_text(signal, frequency, settings.modulation);
        tracing::info!(
            "Transmitting {} level/duration pairs at {} Hz ({}) on the Flipper",
            signal.len(),
            frequency,
            settings.modulation
        );
        let timeout = Duration::from_micros(air_us) + Duration::from_secs(60);
        self.ask(|done| Request::Transmit { file, air_us, done }, timeout)?
    }

    pub fn iq_format(&self) -> IqFormat {
        IqFormat::Cs8
    }

    /// No IQ: the rate KAT's SDRs use, for display only.
    pub fn sample_rate(&self) -> u32 {
        2_000_000
    }

    pub fn rx_frequency(&self) -> u32 {
        self.frequency
    }

    /// Hopping is refused by the app before it gets here ([crate::app::RadioDevice::supports_hopping]).
    pub fn set_hopping(&mut self, _plan: Option<HopPlan>) {}

    pub fn hop_state(&self) -> Option<(HopPlan, usize)> {
        None
    }

    pub fn hop_pause(&mut self, _dwells: u32) {}

    pub fn start_recording(&mut self, _path: std::path::PathBuf) -> Result<()> {
        bail!("a Flipper gives pulses, not IQ: :record needs a HackRF or an RTL-SDR")
    }

    pub fn stop_recording(&mut self) -> Option<RecordingSummary> {
        None
    }

    pub fn recording_status(&self) -> Option<RecordingStatus> {
        None
    }

    pub fn take_ended_recording(&mut self) -> Option<RecordingSummary> {
        None
    }
}

impl Drop for FlipperController {
    fn drop(&mut self) {
        // The worker stops rx_raw and ends when its request channel closes.
        self.requests = None;
        if let Some(w) = self.worker.take() {
            let _ = w.join();
        }
    }
}

/// The modulation a receive preset demodulates: KAT's table for the names it knows, else by the
/// name (doctor-firmware's `AM…` presets are OOK, the rest 2-FSK).
fn preset_rf(name: &str) -> RfModulation {
    match preset::ALL_PRESETS.iter().find(|d| d.name.eq_ignore_ascii_case(name)) {
        Some(def) if def.modulation().is_fsk() => RfModulation::FM,
        Some(_) => RfModulation::AM,
        None if name.to_ascii_uppercase().starts_with("AM") => RfModulation::AM,
        None => RfModulation::FM,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::{FlipperKey, ProtocolRegistry};

    /// A made-up key's transmission, as its encoder sends it.
    fn upload(key: &str) -> Vec<LevelDuration> {
        let key = FlipperKey::parse(key).unwrap();
        let reg = ProtocolRegistry::new();
        let (name, d) = reg.decode_key(&key).unwrap();
        reg.get(&name).unwrap().encode(&d, d.button.unwrap_or(0)).unwrap()
    }

    const PRINCETON: &str = "Protocol: Princeton\nBit: 24\nKey: 00 00 00 00 00 5A 3C 81\nTE: 400\n";
    const SUBARU: &str = "Protocol: Subaru\nBit: 64\nKey: 4D 2A 19 B3 06 5C E7 21\n";

    /// Noise as a Flipper's CC1101 gives it between transmissions: short, irregular pulses.
    fn noise(n: usize, seed: &mut u64, level: &mut bool) -> Vec<LevelDuration> {
        (0..n)
            .map(|_| {
                *seed ^= *seed << 13;
                *seed ^= *seed >> 7;
                *seed ^= *seed << 17;
                *level = !*level;
                LevelDuration::new(*level, 15 + (*seed % 240) as u32)
            })
            .collect()
    }

    /// `rx_raw`'s text for `pulses`: `+n`/`-n` words, a line break every 256 (as the firmware).
    #[cfg(unix)]
    fn rx_raw_text(pulses: &[LevelDuration]) -> String {
        let mut t = String::new();
        for (i, p) in pulses.iter().enumerate() {
            t.push_str(&format!("{}{} ", if p.level { '+' } else { '-' }, p.duration_us));
            if i % 256 == 255 {
                t.push_str("\r\n\n");
            }
        }
        t
    }

    fn captures(events: &Receiver<RadioEvent>) -> Vec<Capture> {
        events
            .try_iter()
            .filter_map(|e| match e {
                RadioEvent::SignalCaptured(c) => Some(c),
                _ => None,
            })
            .collect()
    }

    fn pulses_of(c: &Capture) -> Vec<LevelDuration> {
        c.raw_pairs.iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect()
    }

    /// The words of `rx_raw`, cut anywhere between reads, with its banner and overrun dots.
    #[test]
    fn parses_rx_raw_output() {
        let text = b"subghz rx_raw 433920000\r\nListening at 433920000. Press CTRL+C to stop\r\n+350 -700 . +1200 -15\r\n\n+x -0 -42 ";
        for cut in 0..text.len() {
            let mut p = RawParser::default();
            let mut out = Vec::new();
            p.push(&text[..cut], &mut out);
            p.push(&text[cut..], &mut out);
            assert_eq!(
                out,
                vec![
                    RawItem::Pulse(LevelDuration::new(true, 350)),
                    RawItem::Pulse(LevelDuration::new(false, 700)),
                    RawItem::Break,
                    RawItem::Pulse(LevelDuration::new(true, 1200)),
                    RawItem::Pulse(LevelDuration::new(false, 15)),
                    RawItem::Pulse(LevelDuration::new(false, 42)),
                ],
                "cut at {cut}"
            );
        }
    }

    /// Frames in a stream of noise reach the app once each, cut out of the noise: the burst starts
    /// where the transmission does (Subaru's preamble included), decodes as the frame sent, and its
    /// on-air time is the transmission's.
    #[test]
    fn frames_are_cut_out_of_noise() {
        let (tx, events) = mpsc::channel();
        let mut rx = RxStream::new(433_920_000, RfModulation::AM, true, tx);
        let (mut seed, mut level) = (0x9E37_79B9_7F4A_7C15u64, false);
        let mut stream = noise(4000, &mut seed, &mut level);
        let mut sent = Vec::new();
        // How late the burst may start: Princeton's first copy has no sync before it (the decoders
        // read from the second, about 51 ms in); Subaru's preamble is recovered whole.
        for (key, max_lead) in [(PRINCETON, 60_000i64), (SUBARU, 2_000)] {
            let up = upload(key);
            // The noise before ends on the level the upload does not start with.
            if stream.last().is_some_and(|p| p.level == up[0].level) {
                stream.push(LevelDuration::new(!up[0].level, 180));
            }
            let start_us: u64 = stream.iter().map(|p| u64::from(p.duration_us)).sum();
            stream.extend_from_slice(&up);
            sent.push((key, max_lead, start_us, up.len()));
            level = up.last().unwrap().level;
            stream.extend(noise(4000, &mut seed, &mut level));
        }
        for p in &stream {
            rx.push(*p);
        }
        rx.brk();
        let caps = captures(&events);
        let mut reg = ProtocolRegistry::new();
        for (key, max_lead, start_us, len) in sent {
            let name = FlipperKey::parse(key).unwrap().protocol;
            let mine: Vec<&Capture> = caps
                .iter()
                .filter(|c| {
                    reg.process_signal_stream_path(&pulses_of(c), 433_920_000, Some(RfModulation::AM))
                        .iter()
                        .any(|(n, _, _)| *n == name)
                })
                .collect();
            assert!(!mine.is_empty(), "{name}: no burst decodes ({} bursts)", caps.len());
            let first = mine[0];
            let t = first.on_air.unwrap();
            // The burst begins within a few pulses of the transmission (not in the noise before).
            let lead = t.start_us as i64 - start_us as i64;
            assert!((-1_000..=max_lead).contains(&lead), "{name}: burst starts {lead} µs from the transmission");
            assert!(first.raw_pairs.len() <= len + 4, "{name}: {} pulses for a {len}-pulse upload", first.raw_pairs.len());
        }
        // Noise alone gives no burst: every burst decodes.
        for c in &caps {
            assert!(!reg.process_signal_stream_path(&pulses_of(c), 433_920_000, Some(RfModulation::AM)).is_empty());
        }
    }

    /// Flipper recordings streamed as `rx_raw` would give them: the frames the app gets from the
    /// bursts, against what KAT decodes from each file whole. (`--ignored --nocapture` prints the
    /// files that differ.)
    #[test]
    #[ignore]
    fn imports_through_the_stream() {
        use std::collections::BTreeSet;
        let mut files = Vec::new();
        let mut dirs = vec![std::path::PathBuf::from("IMPORTS")];
        while let Some(d) = dirs.pop() {
            for e in std::fs::read_dir(d).unwrap().flatten() {
                let p = e.path();
                if p.is_dir() {
                    dirs.push(p);
                } else if p.extension().is_some_and(|x| x == "sub") {
                    files.push(p);
                }
            }
        }
        files.sort();
        let (mut n, mut whole_total, mut missing_total, mut extra_total, mut bursts) = (0, 0, 0, 0, 0);
        for path in files {
            let Ok(sub) = crate::export::flipper::import_sub(&path) else { continue };
            if sub.pairs.len() < 50 || sub.preset.as_ref().and_then(|p| p.rf()) == Some(RfModulation::FM) {
                continue;
            }
            n += 1;
            let pairs: Vec<LevelDuration> = sub.pairs.iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect();
            let mut reg = ProtocolRegistry::new();
            let key = |(name, d, _): (String, crate::protocols::DecodedSignal, Vec<LevelDuration>)| (name, d.data);
            let whole: BTreeSet<(String, u64)> =
                reg.process_signal_stream_rf(&pairs, sub.frequency, Some(RfModulation::AM)).into_iter().map(key).collect();
            let (tx, events) = mpsc::channel();
            let mut rx = RxStream::new(sub.frequency, RfModulation::AM, false, tx);
            for p in &pairs {
                rx.push(*p);
            }
            rx.brk();
            let caps = captures(&events);
            bursts += caps.len();
            let streamed: BTreeSet<(String, u64)> = caps
                .iter()
                .flat_map(|c| reg.process_signal_stream_path(&pulses_of(c), sub.frequency, Some(RfModulation::AM)))
                .map(key)
                .collect();
            let missing: Vec<_> = whole.difference(&streamed).collect();
            let extra: Vec<_> = streamed.difference(&whole).collect();
            whole_total += whole.len();
            missing_total += missing.len();
            extra_total += extra.len();
            if !missing.is_empty() || !extra.is_empty() {
                println!("{}: {} decodes, missing {missing:?}, extra {extra:?}", path.display(), whole.len());
            }
        }
        println!("{n} AM recordings, {whole_total} distinct frames, {bursts} bursts: {missing_total} missing, {extra_total} extra");
        assert_eq!((missing_total, extra_total), (0, 0));
    }

    /// The transmitted file: preset by modulation, pulses merged, 512 values a line.
    #[test]
    fn transmission_files() {
        let signal = [
            LevelDuration::new(false, 1000),
            LevelDuration::new(true, 400),
            LevelDuration::new(true, 100),
            LevelDuration::new(false, 0),
            LevelDuration::new(false, 800),
        ];
        let ook = tx_sub_text(&signal, 433_920_000, TxModulation::Ook);
        assert_eq!(
            ook,
            "Filetype: Flipper SubGhz RAW File\nVersion: 1\nFrequency: 433920000\n\
             Preset: FuriHalSubGhzPresetOok650Async\nProtocol: RAW\nRAW_Data: -1000 500 -800\n"
        );
        let fm = |d: f64| tx_sub_text(&signal, 868_350_000, TxModulation::Fsk { deviation_hz: d });
        let fm476 = TxModulation::Fsk { deviation_hz: preset::deviation_hz(0x47) };
        assert!(tx_sub_text(&signal, 1, fm476).contains("Preset: FuriHalSubGhzPreset2FSKDev476Async\n"));
        let fm238 = preset::FM238.modulation();
        assert!(tx_sub_text(&signal, 1, fm238).contains("Preset: FuriHalSubGhzPreset2FSKDev238Async\n"));
        // A deviation no preset has: FM476's registers with the nearest DEVIATN (30 kHz: 0x41, 28.6 kHz).
        let odd = fm(30_000.0);
        assert!(odd.contains("Preset: FuriHalSubGhzPresetCustom\nCustom_preset_module: CC1101\n"), "{odd}");
        let regs = odd.lines().find_map(|l| l.strip_prefix("Custom_preset_data: ")).unwrap();
        let regs = preset::parse_hex_bytes(regs).unwrap();
        assert_eq!(preset::register_value(&regs, preset::REG_DEVIATN), Some(0x41));
        assert_eq!(preset::modulation_from_registers(&regs).map(|m| m.is_fsk()), Some(true));
        // 1100 alternating pulses: lines of 512, 512 and 76 values.
        let long: Vec<LevelDuration> = (0..1100).map(|i| LevelDuration::new(i % 2 == 0, 300)).collect();
        let counts: Vec<usize> = tx_sub_text(&long, 1, TxModulation::Ook)
            .lines()
            .filter_map(|l| l.strip_prefix("RAW_Data:"))
            .map(|l| l.split_whitespace().count())
            .collect();
        assert_eq!(counts, [512, 512, 76]);
    }

    #[test]
    fn refusals_and_output() {
        assert_eq!(tx_refusal("Listening at /ext/a.sub. Frequency=433920000, Protocol=RAW\n\nPress CTRL+C to stop\n\n..."), None);
        assert_eq!(
            tx_refusal("subghz tx_from_file: Frequency not supported").as_deref(),
            Some("Frequency not supported")
        );
        assert!(tx_refusal("Transmission on this frequency is restricted in your settings/region").is_some());
        assert!(tx_refusal("`subghz tx_from_file` command not found").is_some());
        assert_eq!(
            clean_output(b"\r\nsubghz tx_from_file x 1 0\r\nsubghz tx_from_file: \x1b[0;31mMissing Preset\x1b[0m\r\n", "subghz tx_from_file x 1 0"),
            "subghz tx_from_file: Missing Preset"
        );
        assert_eq!(preset_rf("FM476"), RfModulation::FM);
        assert_eq!(preset_rf("am650"), RfModulation::AM);
        assert_eq!(preset_rf("FM20KR"), RfModulation::FM);
        assert!(is_flipper(0x0483, 0x5740, None, None));
        assert!(is_flipper(0x1234, 0x0001, Some("Flipper Devices Inc."), None));
        assert!(!is_flipper(0x0483, 0x374B, Some("STMicroelectronics"), Some("STM32 STLink")));
    }

    /// A Flipper's console on a pseudo-terminal: it answers the commands KAT sends as the firmware
    /// does, streams `rx_script` for `rx_raw` until Ctrl+C, keeps written files and logs commands.
    #[cfg(unix)]
    mod fake {
        use super::*;
        use serialport::{SerialPort, TTYPort};
        use std::collections::HashMap;
        use std::sync::Mutex;

        pub struct Log {
            pub commands: Vec<String>,
            pub transmitted: Vec<String>,
            pub files: HashMap<String, Vec<u8>>,
        }

        pub struct Fake {
            port: TTYPort,
            rx_script: Vec<u8>,
            refuse_tx: bool,
            presets: bool,
            pub log: Arc<Mutex<Log>>,
        }

        fn write_all(port: &mut TTYPort, mut bytes: &[u8]) -> bool {
            let end = Instant::now() + Duration::from_secs(10);
            while !bytes.is_empty() && Instant::now() < end {
                match port.write(bytes) {
                    Ok(n) => bytes = &bytes[n..],
                    Err(e) if matches!(e.kind(), std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock | std::io::ErrorKind::Interrupted) => {}
                    Err(_) => return false,
                }
            }
            bytes.is_empty()
        }

        /// The console end for KAT, and the fake running on the other end.
        pub fn start(rx_script: Vec<u8>, refuse_tx: bool, presets: bool) -> (Box<dyn SerialPort>, Arc<Mutex<Log>>, std::thread::JoinHandle<()>) {
            let (mut ours, mut theirs) = TTYPort::pair().unwrap();
            ours.set_timeout(Duration::from_millis(10)).unwrap();
            theirs.set_timeout(READ_TIMEOUT).unwrap();
            let log = Arc::new(Mutex::new(Log { commands: Vec::new(), transmitted: Vec::new(), files: HashMap::new() }));
            let fake = Fake { port: ours, rx_script, refuse_tx, presets, log: log.clone() };
            let handle = std::thread::spawn(move || fake.run());
            (Box::new(theirs), log, handle)
        }

        impl Fake {
            fn out(&mut self, s: &[u8]) -> bool {
                write_all(&mut self.port, s)
            }

            fn read_byte(&mut self) -> Option<Option<u8>> {
                let mut b = [0u8; 1];
                match self.port.read(&mut b) {
                    Ok(1) => Some(Some(b[0])),
                    Ok(_) => None,
                    Err(e) if matches!(e.kind(), std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock | std::io::ErrorKind::Interrupted) => Some(None),
                    Err(_) => None,
                }
            }

            fn run(mut self) {
                if !self.out(b"\r\nWelcome to Flipper Zero Command Line Interface!\r\n\r\n>: ") {
                    return;
                }
                let mut line: Vec<u8> = Vec::new();
                loop {
                    let Some(byte) = self.read_byte() else { return };
                    let Some(b) = byte else { continue };
                    match b {
                        0x03 => {
                            line.clear();
                            if !self.out(b"\r\n>: ") {
                                return;
                            }
                        }
                        b'\r' => {
                            let cmd = String::from_utf8_lossy(&line).to_string();
                            line.clear();
                            if !self.out(b"\r\n") || !self.handle(&cmd) {
                                return;
                            }
                        }
                        _ => {
                            line.push(b);
                            if !self.out(&[b]) {
                                return;
                            }
                        }
                    }
                }
            }

            fn handle(&mut self, cmd: &str) -> bool {
                if !cmd.is_empty() {
                    self.log.lock().unwrap().commands.push(cmd.to_string());
                }
                let words: Vec<&str> = cmd.split_whitespace().collect();
                let reply: Vec<u8> = match words.as_slice() {
                    [] => Vec::new(),
                    ["subghz", "rx_raw", f, rest @ ..] => {
                        if f.parse::<u32>().is_err() || (!rest.is_empty() && !self.presets) {
                            let usage: &[u8] = if self.presets {
                                b"Usage: subghz rx_raw <Frequency: in Hz> [Device: 0 - internal, 1 - external] [Preset: AM270|AM650|FM238|FM476]\r\n"
                            } else {
                                b"Usage: subghz rx <Frequency: in Hz>\r\n"
                            };
                            usage.to_vec()
                        } else {
                            if !self.out(format!("Listening at {f}. Press CTRL+C to stop\r\n").as_bytes()) {
                                return false;
                            }
                            let script = self.rx_script.clone();
                            if !self.out(&script) {
                                return false;
                            }
                            // Stream until Ctrl+C.
                            loop {
                                match self.read_byte() {
                                    None => return false,
                                    Some(Some(0x03)) => break,
                                    Some(_) => {}
                                }
                            }
                            Vec::new()
                        }
                    }
                    ["storage", "mkdir", _] => b"Storage error: File/Dir already exists\r\n".to_vec(),
                    ["storage", "remove", path] => {
                        self.log.lock().unwrap().files.remove(*path);
                        Vec::new()
                    }
                    ["storage", "write_chunk", path, n] => {
                        let n: usize = n.parse().unwrap();
                        if !self.out(b"Ready\r\n") {
                            return false;
                        }
                        let mut data = Vec::with_capacity(n);
                        while data.len() < n {
                            match self.read_byte() {
                                None => return false,
                                Some(Some(b)) => data.push(b),
                                Some(None) => {}
                            }
                        }
                        self.log.lock().unwrap().files.entry(path.to_string()).or_default().extend(data);
                        Vec::new()
                    }
                    ["subghz", "tx_from_file", path, "1", "0"] => {
                        if self.refuse_tx {
                            b"Transmission on this frequency is restricted in your settings/region\r\n".to_vec()
                        } else {
                            let mut log = self.log.lock().unwrap();
                            let file = String::from_utf8_lossy(log.files.get(*path).map_or(&[][..], |f| f)).to_string();
                            log.transmitted.push(file);
                            format!("Listening at \x1b[0;33m{path}\x1b[0m. Frequency=433920000, Protocol=RAW\r\n\r\nPress CTRL+C to stop\r\n\r\n..").into_bytes()
                        }
                    }
                    _ => format!("`{cmd}` command not found\r\n").into_bytes(),
                };
                self.out(&reply) && self.out(b"\r\n>: ")
            }
        }
    }

    /// The whole path on a simulated Flipper: open, receive (a Princeton press in noise reaches the
    /// app), transmit (the file written and played, receiving resumed), a refusal, stop.
    #[cfg(unix)]
    #[test]
    fn drives_a_flipper_console() {
        let (mut seed, mut level) = (0x5851_F42D_4C95_7F2Du64, false);
        let mut pulses = noise(3000, &mut seed, &mut level);
        let up = upload(PRINCETON);
        if pulses.last().is_some_and(|p| p.level == up[0].level) {
            pulses.push(LevelDuration::new(!up[0].level, 180));
        }
        pulses.extend_from_slice(&up);
        level = up.last().unwrap().level;
        pulses.extend(noise(3000, &mut seed, &mut level));
        let (port, log, fake) = fake::start(rx_raw_text(&pulses).into_bytes(), false, false);
        let (tx, events) = mpsc::channel();
        let console = Console::with_port(port).unwrap();
        let mut flipper = FlipperController::start(console, tx, None, true).unwrap();
        assert!(flipper.is_available() && flipper.supports_tx());

        flipper.start_receiving(433_920_000).unwrap();
        let end = Instant::now() + Duration::from_secs(10);
        let mut got = Vec::new();
        while got.is_empty() && Instant::now() < end {
            std::thread::sleep(Duration::from_millis(50));
            got = captures(&events);
        }
        let mut reg = ProtocolRegistry::new();
        assert!(
            got.iter().any(|c| reg
                .process_signal_stream_path(&pulses_of(c), 433_920_000, Some(RfModulation::AM))
                .iter()
                .any(|(n, _, _)| n == "Princeton")),
            "no Princeton burst ({} bursts)",
            got.len()
        );

        let signal = upload(PRINCETON);
        let settings = TxSettings { modulation: TxModulation::Ook, vga_gain: 47, amp: true };
        flipper.transmit(&signal, 433_920_000, &settings).unwrap();
        {
            let log = log.lock().unwrap();
            assert_eq!(log.transmitted, vec![tx_sub_text(&signal, 433_920_000, TxModulation::Ook)]);
            let cmds = &log.commands;
            let tx_at = cmds.iter().position(|c| c.starts_with("subghz tx_from_file")).unwrap();
            assert_eq!(cmds[tx_at], format!("subghz tx_from_file {TX_FILE} 1 0"));
            assert!(cmds[..tx_at].iter().filter(|c| c.starts_with("subghz rx_raw 433920000")).count() == 1);
            assert!(cmds[tx_at..].iter().any(|c| c == "subghz rx_raw 433920000"), "receiving resumes: {cmds:?}");
        }
        flipper.stop_receiving().unwrap();
        drop(flipper);
        fake.join().unwrap_or(());

        // A firmware that refuses to transmit there: its words as the error.
        let (port, _log, _fake) = fake::start(Vec::new(), true, false);
        let (tx, _events) = mpsc::channel();
        let mut flipper = FlipperController::start(Console::with_port(port).unwrap(), tx, None, true).unwrap();
        let err = flipper.transmit(&signal, 433_920_000, &settings).unwrap_err().to_string();
        assert!(err.contains("restricted"), "{err}");
    }

    /// A configured port is opened by its name (as config `flipper_port` gives it); one that does
    /// not exist leaves the Flipper unavailable, saying why.
    #[cfg(unix)]
    #[test]
    fn opens_a_configured_port() {
        use serialport::{SerialPort, TTYPort};
        let (mut ours, theirs) = TTYPort::pair().unwrap();
        let path = theirs.name().unwrap();
        drop(theirs);
        ours.set_timeout(Duration::from_millis(10)).unwrap();
        // A console that answers every line with a prompt.
        let fake = std::thread::spawn(move || {
            let _ = ours.write_all(b"\r\n>: ");
            let mut b = [0u8; 64];
            loop {
                match ours.read(&mut b) {
                    Ok(0) => return,
                    Ok(n) => {
                        let prompts = b[..n].iter().filter(|&&c| c == b'\r' || c == 0x03).count();
                        for _ in 0..prompts {
                            let _ = ours.write_all(b"\r\n>: ");
                        }
                    }
                    Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                    Err(_) => return,
                }
            }
        });
        let (tx, _events) = mpsc::channel();
        let flipper = FlipperController::new(tx.clone(), Some(&path), None, true);
        assert!(flipper.is_available(), "{:?}", flipper.open_error());
        drop(flipper);
        let _ = fake.join();

        let missing = FlipperController::new(tx, Some("/dev/kat-no-such-port"), None, true);
        assert!(!missing.is_available());
        assert!(missing.open_error().unwrap().contains("/dev/kat-no-such-port"), "{:?}", missing.open_error());
    }

    /// A Flipper found beside other radios is not opened until it is used: qFlipper can have it
    /// until then. The first receive (or transmission) connects, and a failure says why and is
    /// tried again on the next use.
    #[cfg(unix)]
    #[test]
    fn connects_on_first_use() {
        // The pair's own end stays open (unread), so the fake keeps running while the controller
        // opens the terminal by its name, as it opens a Flipper's port.
        let (_port, log, _fake) = fake::start(Vec::new(), false, false);
        let path = _port.name().unwrap();
        let (tx, _events) = mpsc::channel();
        let mut flipper = FlipperController::on_port(tx.clone(), path, None, true);
        assert!(!flipper.is_connected() && flipper.supports_tx() && flipper.open_error().is_none());
        std::thread::sleep(Duration::from_millis(50));
        assert!(log.lock().unwrap().commands.is_empty(), "nothing sent before it is used");
        flipper.start_receiving(433_920_000).unwrap();
        assert!(flipper.is_connected());
        flipper.stop_receiving().unwrap();
        assert!(log.lock().unwrap().commands.iter().any(|c| c == "subghz rx_raw 433920000"));
        drop(flipper);

        let mut gone = FlipperController::on_port(tx, "/dev/kat-no-such-port".to_string(), None, true);
        let err = gone.start_receiving(433_920_000).unwrap_err().to_string();
        assert!(err.contains("/dev/kat-no-such-port") && !gone.is_connected(), "{err}");
        assert!(gone.open_error().is_some_and(|e| e.contains("/dev/kat-no-such-port")));
        assert!(gone.start_receiving(433_920_000).is_err(), "tried again, still not there");
    }

    /// `flipper_preset` needs a firmware whose rx_raw takes one (doctor-firmware's); with one, rx_raw
    /// gets it, and an FM preset receives FSK.
    #[cfg(unix)]
    #[test]
    fn receive_presets() {
        let (port, _log, _fake) = fake::start(Vec::new(), false, false);
        let (tx, _events) = mpsc::channel();
        let err = FlipperController::start(Console::with_port(port).unwrap(), tx, Some("FM476"), true).err().unwrap();
        assert!(err.to_string().contains("doctor-firmware"), "{err}");

        let (port, log, _fake) = fake::start(Vec::new(), false, true);
        let (tx, _events) = mpsc::channel();
        let mut flipper = FlipperController::start(Console::with_port(port).unwrap(), tx, Some("FM476"), true).unwrap();
        flipper.start_receiving(868_350_000).unwrap();
        flipper.stop_receiving().unwrap();
        assert!(log.lock().unwrap().commands.iter().any(|c| c == "subghz rx_raw 868350000 0 FM476"));
    }
}
