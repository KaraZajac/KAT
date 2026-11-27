//! A Flipper Zero's command line, without the port: the console session (prompt, commands,
//! `rx_raw`, the SD-card file KAT transmits, the firmware's refusals), the `rx_raw` stream parser,
//! the frames cut out of its noise ([RxStream]) and the RAW `.sub` a transmission is played from.
//!
//! The port is anything that reads and writes bytes ([Port]): desktop KAT opens the Flipper's USB
//! serial device (`flipper.rs`, the worker thread and the controller); KAT Mobile talks to it over
//! Android's USB host API (its `mobile_flipper.rs`), with this file copied unchanged into its core.
//! How it works is in `flipper.rs`'s module docs.

use std::collections::VecDeque;
use std::io::{Read, Write};
use std::time::{Duration, Instant};

use anyhow::{anyhow, bail, Result};

use crate::capture::{Capture, OnAir, RfModulation, StoredLevelDuration};
use crate::protocols::{LocatedFrame, StreamLocator};

use super::demodulator::LevelDuration;
use super::preset::{self, PresetDef, RfPreset, TxModulation};

/// The Flipper's console: a byte stream both ways. A read waits a little (desktop: 20 ms) and
/// returns `TimedOut` / `WouldBlock` when nothing came; `Ok(0)` means the console closed.
pub(crate) trait Port: Read + Write + Send {}
impl<T: Read + Write + Send + ?Sized> Port for T {}

/// Where [RxStream] hands each burst: a capture of its pulses, frequency and on-air time.
pub(crate) type BurstSink = Box<dyn FnMut(Capture) + Send>;

/// The console's prompt, after every command.
pub(crate) const PROMPT: &[u8] = b"\r\n>: ";
/// Where KAT puts the file it transmits: the Flipper's app-data folder.
pub(crate) const TX_DIR: &str = "/ext/apps_data/kat";
pub(crate) const TX_FILE: &str = "/ext/apps_data/kat/tx.sub";
/// `storage write_chunk` size (scripts/flipper_replay.py sends every test file this way).
pub(crate) const CHUNK: usize = 512;
/// How long one read of the port waits for bytes (the worker's reaction time).
pub(crate) const READ_TIMEOUT: Duration = Duration::from_millis(20);
/// Pulses after a frame's last one that go into its burst: the app's decode reads up to two more
/// (a frame reported on the middle of its last bit, a decoder holding a frame for one pulse).
pub(crate) const TAIL_PULSES: u64 = 2;
/// How far back from its last pulse a frame's start is looked for (about 3 s of dense noise).
pub(crate) const MAX_LOOKBACK: u64 = 20_000;
/// At most this many regular pulses are added before a frame (its preamble, see frame_start).
pub(crate) const MAX_PREAMBLE_EXTENSION: usize = 4_000;

// ---- the console ----------------------------------------------------------------------------------

pub(crate) fn find_bytes(hay: &[u8], needle: &[u8]) -> Option<usize> {
    hay.windows(needle.len()).position(|w| w == needle)
}

/// Console output as text: no terminal colour codes, no echo of the command line, trimmed.
pub(crate) fn clean_output(bytes: &[u8], command: &str) -> String {
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
pub(crate) struct Console {
    port: Box<dyn Port>,
    /// Bytes read and not consumed yet.
    pub(crate) pending: Vec<u8>,
    /// `/ext/apps_data/kat` exists (made once per session).
    tx_dir_ready: bool,
    /// True once the underlying serial port hits EOF or a fatal I/O error (e.g. USB unplug).
    pub(crate) broken: bool,
}

impl Console {
    /// A session on an open port: past the banner of a new connection, or whatever an earlier
    /// session left running (Ctrl+C ends an `rx_raw`), to a fresh prompt.
    pub(crate) fn with_port(port: Box<dyn Port>) -> Result<Self> {
        let mut c = Self { port, pending: Vec::new(), tx_dir_ready: false, broken: false };
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

    pub(crate) fn send(&mut self, bytes: &[u8]) -> Result<()> {
        if let Err(e) = self.port.write_all(bytes).and_then(|()| self.port.flush()) {
            self.broken = true;
            return Err(e.into());
        }
        Ok(())
    }

    /// Read what has arrived (waits up to [READ_TIMEOUT]); the number of bytes added.
    pub(crate) fn read_more(&mut self) -> Result<usize> {
        let mut buf = [0u8; 8192];
        match self.port.read(&mut buf) {
            Ok(0) => {
                self.broken = true;
                bail!("the Flipper's console closed");
            }
            Ok(n) => {
                self.pending.extend_from_slice(&buf[..n]);
                Ok(n)
            }
            Err(e) if matches!(e.kind(), std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock | std::io::ErrorKind::Interrupted) => Ok(0),
            Err(e) => {
                self.broken = true;
                Err(anyhow!("reading the Flipper's console failed: {e} (unplugged?)"))
            }
        }
    }

    /// Read until one of `tokens` arrives: which one, and the bytes before it (both consumed).
    pub(crate) fn read_until_any(&mut self, tokens: &[&[u8]], timeout: Duration) -> Result<Option<(usize, Vec<u8>)>> {
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

    pub(crate) fn read_until(&mut self, token: &[u8], timeout: Duration) -> Result<Option<Vec<u8>>> {
        Ok(self.read_until_any(&[token], timeout)?.map(|(_, b)| b))
    }

    /// One command; its output.
    pub(crate) fn command(&mut self, line: &str, timeout: Duration) -> Result<String> {
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
    pub(crate) fn rx_presets_supported(&mut self) -> Result<bool> {
        let out = self.command("subghz rx_raw kat", Duration::from_secs(3))?;
        if let Some(why) = console_refusal(&out) {
            bail!("the Flipper refused `subghz rx_raw`: {why}");
        }
        Ok(out.contains("Preset"))
    }

    /// Start `rx_raw`: Ok once the Flipper says it listens; its answer as the error when it refuses.
    pub(crate) fn start_rx(&mut self, frequency: u32, preset: Option<&str>) -> Result<()> {
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
                let text = clean_output(&out, &line);
                let why = console_refusal(&text)
                    .unwrap_or_else(|| if text.is_empty() { "no reason given".to_string() } else { text });
                bail!("the Flipper refused `{line}`: {why}")
            }
            None => bail!("no answer to `{line}`"),
        }
    }

    /// Stop `rx_raw` (Ctrl+C); what it printed until the prompt came back.
    pub(crate) fn stop_rx(&mut self) -> Result<Vec<u8>> {
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
    pub(crate) fn write_file(&mut self, path: &str, data: &[u8]) -> Result<()> {
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
    pub(crate) fn transmit(&mut self, file: &str, air_us: u64) -> Result<()> {
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

/// Why `tx_from_file` did not transmit, from its output: its error line (`subghz tx_from_file: …`),
/// the region lock, a refusal of the console ([console_refusal]), the usage text of a firmware
/// without the command. Every firmware KAT follows (official, Momentum, RogueMaster,
/// doctor-firmware, Flipper-ARF, FoxFW) prints "Listening at <file>. Frequency=…, Protocol=…" and
/// "Press CTRL+C to stop" when it starts sending (the region lock speaks after that line): output
/// without it is no transmission either, whatever it says.
pub(crate) fn tx_refusal(out: &str) -> Option<String> {
    if let Some(why) = console_refusal(out) {
        return Some(why);
    }
    for line in out.lines().map(str::trim) {
        if let Some(why) = line.strip_prefix("subghz tx_from_file:") {
            return Some(why.trim().to_string());
        }
        if line.contains("restricted") {
            return Some(line.to_string());
        }
        if line.starts_with("Usage") || line.contains("Cmd list") {
            return Some("this firmware has no `subghz tx_from_file`".to_string());
        }
    }
    if out.contains("Press CTRL+C to stop") {
        return None;
    }
    Some(match out.lines().map(str::trim).find(|l| !l.is_empty()) {
        Some(line) => format!("it did not start sending (it answered \"{line}\")"),
        None => "it did not start sending (no answer)".to_string(),
    })
}

/// What the command line of current firmwares (official 1.x, Momentum, RogueMaster) answers when it
/// does not run a command, in words: it runs no `subghz` command while an app is open on the
/// Flipper (`storage` it does), and loads `subghz` from the SD card, which fails when the card's
/// files are for another firmware version (a Flipper whose firmware was flashed without them).
/// Older firmwares say "`…` command not found" for a command they lack. None: no such answer.
pub(crate) fn console_refusal(text: &str) -> Option<String> {
    let why = if text.contains("cannot be run while an application is open") {
        "an app is open on the Flipper; close it (the console runs no Sub-GHz command while one is open)"
    } else if text.contains("failed to load external command") {
        "its `subghz` command did not load from the SD card, whose files are for another firmware \
         version; install the firmware together with its SD card files (qFlipper or the firmware's updater)"
    } else if text.contains("could not find command") || text.contains("command not found") {
        "this firmware has no `subghz` command"
    } else {
        return None;
    };
    Some(why.to_string())
}

// ---- receive ---------------------------------------------------------------------------------------

/// What `rx_raw` prints, read as it arrives (a token may be cut between two reads).
#[derive(Default)]
pub(crate) struct RawParser {
    word: Vec<u8>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum RawItem {
    Pulse(LevelDuration),
    /// `.`: the Flipper's buffer overran, pulses are missing here.
    Break,
}

impl RawParser {
    pub(crate) fn push(&mut self, bytes: &[u8], out: &mut Vec<RawItem>) {
        for &b in bytes {
            if b.is_ascii_whitespace() {
                self.end_word(out);
            } else if self.word.len() < 16 {
                self.word.push(b);
            }
        }
    }

    /// A word ends (whitespace, or the stream stopped): a pulse, a break, or text to skip.
    pub(crate) fn end_word(&mut self, out: &mut Vec<RawItem>) {
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
pub(crate) struct RxStream {
    frequency: u32,
    rf: RfModulation,
    session: u64,
    /// Where the bursts go (desktop: the app's event channel; KAT Mobile: the next pump's answer).
    sink: BurstSink,
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
    /// `session`: the receive session the bursts' on-air times count from (desktop:
    /// `radio::rx::new_session`).
    pub(crate) fn new(frequency: u32, rf: RfModulation, report_unchecked: bool, session: u64, sink: BurstSink) -> Self {
        let polarities: &[bool] = if rf == RfModulation::FM { &[false, true] } else { &[false] };
        let locators: Vec<StreamLocator> = polarities
            .iter()
            .map(|&invert| StreamLocator::new(frequency, rf, invert, report_unchecked))
            .collect();
        Self {
            frequency,
            rf,
            session,
            sink,
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

    pub(crate) fn push(&mut self, p: LevelDuration) {
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
    pub(crate) fn brk(&mut self) {
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
        (self.sink)(capture);
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
pub(crate) struct Receiving {
    pub(crate) frequency: u32,
    parser: RawParser,
    stream: RxStream,
}

impl Receiving {
    /// Start `rx_raw` at `frequency` (with doctor-firmware's `preset`), its pulses going to `stream`.
    pub(crate) fn start(console: &mut Console, frequency: u32, preset: Option<&str>, stream: RxStream) -> Result<Self> {
        console.start_rx(frequency, preset)?;
        let mut r = Self { frequency, parser: RawParser::default(), stream };
        let early = std::mem::take(&mut console.pending);
        r.feed(&early);
        Ok(r)
    }

    /// Read and process what arrived.
    pub(crate) fn pump(&mut self, console: &mut Console) -> Result<()> {
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

    pub(crate) fn stop(mut self, console: &mut Console) -> Result<()> {
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
pub(crate) fn tx_preset(modulation: TxModulation) -> RfPreset {
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
pub(crate) fn tx_sub_text(signal: &[LevelDuration], frequency: u32, modulation: TxModulation) -> String {
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
