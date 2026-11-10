//! Transmit queue, per-transmission modulation and TX power (the app side of KAT's transmitter).
//!
//! Every transmission is queued with the modulation its capture calls for
//! ([crate::capture::Capture::tx_modulation]: the capture's own preset, its receive path, or its
//! protocol's preset) and sent with the TX power settings (`tx_vga_gain` / `tx_amp` in
//! `config.ini`, `:txpower`, the TX settings tab). The queue runs first in, first out, so
//! `:replay 1-5` goes out in the order given.
//!
//! **Rolling codes advance.** After a transmission the references keep what was sent:
//! Flipper-ARF and doctor-firmware build the upload from the signal (the encoder's `deserialize`
//! advances the counter and writes the new `Key` back into it) and, when TX stops, save it to its
//! file for every `SubGhzProtocolTypeDynamic` protocol (`subghz_txrx_tx_stop`: "if protocol
//! dynamic then we save the last upload"); ProtoPirate's emulate scene writes `Cnt` + 1 and the
//! pressed `Btn` into the loaded file on every press. So the next press sends the code after.
//! KAT does the same for its encoders ("Send next code", "Send button…", TX Lock … Panic and the
//! `:lock` … `:panic` commands): it decodes the frame it is about to send with the protocol's own
//! decoder ([sent_frame]) and, once the transmission went out, stores it on the capture (counter,
//! frame, button, encoder state; [store_sent_frame]), which the session autosave keeps. Only
//! rolling codes change (the counter moved); fixed codes keep their capture, as the references'
//! static protocols do. Replay re-sends the recorded pulses and changes nothing.

use std::collections::VecDeque;

use anyhow::Result;

use super::{App, RadioState};
use crate::capture::Capture;
use crate::protocols::{DecodedSignal, ProtocolRegistry};
use crate::radio::devices::RadioKind;
use crate::radio::preset::TxModulation;
use crate::radio::tx::TxSettings;
use crate::radio::LevelDuration;

/// Highest HackRF TX VGA gain (dB).
pub const TX_VGA_MAX: u32 = 47;

/// TX power choices offered in the settings dropdown: (TX VGA dB, RF amp).
pub const TX_POWER_STEPS: [(u32, bool); 9] = [
    (47, true),
    (40, true),
    (30, true),
    (47, false),
    (40, false),
    (30, false),
    (20, false),
    (10, false),
    (0, false),
];

/// One queued transmission.
pub(super) struct PendingTransmit {
    pub(super) signal: Vec<LevelDuration>,
    pub(super) frequency: u32,
    pub(super) modulation: TxModulation,
    /// A rolling code from an encoder: the capture and the decode of the frame sent, stored on the
    /// capture once the transmission went out ([store_sent_frame]).
    pub(super) advance: Option<(u32, DecodedSignal)>,
}

/// `upload` on air, sent twice with a gap as the transmitter repeats it: adjacent pulses of the
/// same level are one pulse to a receiver (`merge`), as the protocol tests decode TX.
fn on_air(upload: &[LevelDuration], merge: bool) -> Vec<LevelDuration> {
    let gap = [LevelDuration::new(false, 20_000)];
    let mut stream: Vec<LevelDuration> = Vec::with_capacity(2 * upload.len() + 2);
    for p in upload.iter().chain(&gap).chain(upload).chain(&gap) {
        match stream.last_mut() {
            Some(last) if merge && last.level == p.level => last.duration_us += p.duration_us,
            _ => stream.push(LevelDuration::new(p.level, p.duration_us)),
        }
    }
    stream
}

/// The frame `upload` carries, the encoder's output for a capture decoded as `from`, sent with
/// `button`: what the encoder reports building ([crate::protocols::ProtocolDecoder::sent_signal],
/// PSA), else the last frame `protocol`'s own decoder reads from it with the capture's serial (the
/// pulses merged as on air, then as they are), completed with the encoder state the air does not
/// carry ([with_context]). None when it does not decode (a fixed code such as Revers RB2 or
/// Marantec24 whose decoder rejects its own upload, as in the references).
pub(super) fn sent_frame(
    registry: &mut ProtocolRegistry,
    protocol: &str,
    from: &DecodedSignal,
    button: u8,
    upload: &[LevelDuration],
) -> Option<DecodedSignal> {
    if let Some(sent) = registry.get(protocol).and_then(|p| p.sent_signal(from, button)) {
        return Some(sent);
    }
    let sent = [true, false].into_iter().find_map(|merge| {
        registry
            .decode_with(protocol, &on_air(upload, merge))
            .into_iter()
            .rev()
            .find(|d| from.serial.is_none() || d.serial == from.serial)
    })?;
    Some(with_context(registry, protocol, from, sent))
}

/// Encoder state a frame does not carry on air stays the capture's (FAAC SLH's seed, in
/// `extra`); when the frame alone does not give the counter, the protocol reads it again with
/// that state through its key file (its `Seed` decrypts the hop), as a reference reloading the
/// saved file does.
fn with_context(registry: &ProtocolRegistry, protocol: &str, from: &DecodedSignal, sent: DecodedSignal) -> DecodedSignal {
    let sent = DecodedSignal { extra: sent.extra.or(from.extra), ..sent };
    if sent.counter.is_some() || from.counter.is_none() {
        return sent;
    }
    registry
        .get(protocol)
        .and_then(|p| p.to_flipper_key(&sent).and_then(|key| p.from_flipper_key(&key)))
        .filter(|d| d.serial == sent.serial)
        .unwrap_or(sent)
}

/// Whether the frame sent is a rolling code's next one: its counter moved. Fixed codes (whose
/// "counter" is a DIP or channel field, if any) keep their capture, also after "Send button…".
pub(super) fn advances(from: &DecodedSignal, sent: &DecodedSignal) -> bool {
    sent.counter.is_some() && sent.counter != from.counter
}

/// Keep the frame sent on its capture, as the references save the last upload: counter, frame,
/// button and encoder state. The recording (raw pairs, which Replay sends), the protocol name,
/// frequency, preset and the user's metadata stay.
pub(super) fn store_sent_frame(capture: &mut Capture, sent: &DecodedSignal) {
    capture.serial = sent.serial.or(capture.serial);
    capture.button = sent.button.or(capture.button);
    capture.counter = sent.counter;
    capture.crc_valid = sent.crc_valid;
    capture.data = sent.data;
    capture.data_count_bit = sent.data_count_bit;
    capture.data_extra = sent.extra;
}

/// The transmit queue (first in, first out).
pub(super) type TransmitQueue = VecDeque<PendingTransmit>;

/// "47 dB + amp" / "30 dB".
pub fn tx_power_label(vga_gain: u32, amp: bool) -> String {
    format!("{vga_gain} dB{}", if amp { " + amp" } else { "" })
}

/// Parse `:txpower` arguments into (gain, amp), keeping the current value of whatever is not
/// given: `<gain>`, `<gain> amp on|off`, `<gain> on|off`, `amp on|off`. None = usage error (the
/// gain range is checked when it is applied).
fn parse_txpower(args: &[&str], gain: u32, amp: bool) -> Option<(u32, bool)> {
    let mut words: Vec<String> = args.iter().map(|w| w.to_ascii_lowercase()).collect();
    let mut gain = gain;
    let mut amp = amp;
    if let Some(g) = words.first().and_then(|w| w.parse::<u32>().ok()) {
        gain = g;
        words.remove(0);
    }
    let words: Vec<&str> = words.iter().map(String::as_str).collect();
    match words.as_slice() {
        [] if !args.is_empty() => {}
        ["amp", v] | [v] => match *v {
            "on" | "1" | "true" => amp = true,
            "off" | "0" | "false" => amp = false,
            _ => return None,
        },
        _ => return None,
    }
    Some((gain, amp))
}

impl App {
    /// Queue a transmission; the main loop draws the TX state, then runs it.
    pub(super) fn queue_transmit(
        &mut self,
        signal: Vec<LevelDuration>,
        frequency: u32,
        modulation: TxModulation,
    ) {
        self.queue_transmit_advancing(signal, frequency, modulation, None);
    }

    /// The update an encoder's transmission of `signal` for capture `id` (encoded from `from`
    /// with `protocol` and `button`) brings once it went out: the frame sent, when it is the
    /// rolling code's next one (see the module docs), so the next press sends the one after.
    pub(super) fn tx_advance(
        &mut self,
        id: u32,
        protocol: &str,
        from: &DecodedSignal,
        button: u8,
        signal: &[LevelDuration],
    ) -> Option<(u32, DecodedSignal)> {
        match sent_frame(&mut self.protocols, protocol, from, button, signal) {
            Some(sent) => advances(from, &sent).then_some((id, sent)),
            None => {
                tracing::warn!("{protocol}: the frame sent does not decode; capture {id} keeps its counter");
                None
            }
        }
    }

    /// [Self::queue_transmit] with the capture update to apply once it went out ([Self::tx_advance]).
    pub(super) fn queue_transmit_advancing(
        &mut self,
        signal: Vec<LevelDuration>,
        frequency: u32,
        modulation: TxModulation,
        advance: Option<(u32, DecodedSignal)>,
    ) {
        self.pending_transmit_queue.push_back(PendingTransmit { signal, frequency, modulation, advance });
        if self.pending_transmit_restore.is_none() {
            if let Some(i) = self.tx_radio() {
                self.pending_transmit_restore = Some((i, self.radios[i].state));
                self.radios[i].state = RadioState::Transmitting;
            }
        }
    }

    /// Why nothing can be transmitted, if so: no radio, or none that transmits (an RTL-SDR is
    /// receive-only).
    pub(super) fn tx_unavailable(&self) -> Option<&'static str> {
        if self.radios.is_empty() {
            Some(super::radios::NO_RADIO)
        } else if self.tx_radio().is_none() {
            Some("Transmit not available – RTL-SDR is receive-only")
        } else {
            None
        }
    }

    /// Whether a transmission may be queued now; sets `last_error` when not: no radio, an
    /// RTL-SDR (receive-only), or the transmitting radio recording (transmitting stops its RX,
    /// which used to end the recording without the user stopping it).
    pub(super) fn transmit_allowed(&mut self) -> bool {
        let why = match (self.tx_unavailable(), self.tx_radio().map(|i| &self.radios[i])) {
            (Some(why), _) => why.to_string(),
            (None, Some(r)) if r.device.recording_status().is_some() => {
                if self.several_radios() {
                    format!("{} is recording and transmits — :record stop first (or :txradio)", r.name)
                } else {
                    "Recording in progress — :record stop first".to_string()
                }
            }
            (None, _) => return true,
        };
        self.last_error = Some(why);
        false
    }

    /// Settings for one transmission: its modulation at the current TX power.
    pub fn tx_settings(&self, modulation: TxModulation) -> TxSettings {
        TxSettings { modulation, vga_gain: self.tx_vga_gain, amp: self.tx_amp }
    }

    /// True if there are queued transmits (UI should draw then call run_one_pending_transmit).
    pub fn has_pending_transmit(&self) -> bool {
        !self.pending_transmit_queue.is_empty()
    }

    /// Run the oldest queued transmit; restores radio_state when the queue is empty. Call after
    /// drawing. A failed transmission is reported in the status line (it no longer ends KAT).
    pub fn run_one_pending_transmit(&mut self) -> Result<()> {
        let Some(tx) = self.pending_transmit_queue.pop_front() else {
            return Ok(());
        };
        let settings = self.tx_settings(tx.modulation);
        // The radio the queue started on (`:txradio` waits for the queue).
        let radio = self.pending_transmit_restore.map(|(i, _)| i).or(self.tx_radio());
        let start = chrono::Utc::now();
        let result = match radio.and_then(|i| self.radios.get_mut(i)) {
            Some(r) => r.device.transmit(&tx.signal, tx.frequency, &settings),
            None => Ok(()),
        };
        if let Some(i) = radio {
            // KAT's other radios hear it: their captures of it are marked (app/radios.rs).
            self.note_transmission(i, tx.frequency, start);
        }
        if self.pending_transmit_queue.is_empty() {
            if let Some((i, prev)) = self.pending_transmit_restore.take() {
                if let Some(r) = self.radios.get_mut(i) {
                    r.state = prev;
                }
            }
        }
        match result {
            // Sent: a rolling code's capture now holds the frame that went out.
            Ok(()) => {
                if let Some((id, sent)) = &tx.advance {
                    if let Some(c) = self.captures.iter_mut().find(|c| c.id == *id) {
                        store_sent_frame(c, sent);
                    }
                }
            }
            Err(e) => self.last_error = Some(format!("Transmit failed: {e:#}")),
        }
        Ok(())
    }

    /// Whether the TX power setting does anything: the radio that transmits is a HackRF (a
    /// Flipper sends at its own power). Without radios it is kept for one plugged in later.
    pub fn tx_power_applies(&self) -> bool {
        match self.tx_radio() {
            Some(i) => self.radios[i].kind() == RadioKind::HackRf,
            None => self.radios.is_empty(),
        }
    }

    /// "TX power: 47 dB + amp", and who it is for when the radio that transmits does not use it.
    fn tx_power_text(&self) -> String {
        let power = tx_power_label(self.tx_vga_gain, self.tx_amp);
        match self.tx_radio() {
            Some(i) if self.radios[i].kind() == RadioKind::Flipper => {
                format!("TX power: {power} (a HackRF's: {} sends at its own power)", self.radios[i].name)
            }
            _ => format!("TX power: {power}"),
        }
    }

    /// Set the TX power used for every transmission (VGA 0–47 dB, RF amp on/off).
    pub(super) fn set_tx_power(&mut self, vga_gain: u32, amp: bool) {
        if vga_gain > TX_VGA_MAX {
            self.last_error = Some(format!("TX VGA gain must be 0-{TX_VGA_MAX} dB"));
            return;
        }
        self.tx_vga_gain = vga_gain;
        self.tx_amp = amp;
        self.status_message = Some(self.tx_power_text());
    }

    /// `:txpower [<0-47>] [amp on|off]` (arguments after the command word). Without arguments it
    /// shows the current setting; `amp on|off` alone keeps the gain, a gain alone keeps the amp.
    pub(super) fn txpower_command(&mut self, args: &[&str]) {
        if args.is_empty() {
            self.status_message = Some(self.tx_power_text());
            return;
        }
        match parse_txpower(args, self.tx_vga_gain, self.tx_amp) {
            Some((gain, amp)) => self.set_tx_power(gain, amp),
            None => self.last_error = Some("Usage: :txpower <0-47> [amp on|off]".to_string()),
        }
    }

    /// Settings dropdown: index of the current TX power in [TX_POWER_STEPS]; None when it is not
    /// listed (`:txpower 35`): the dropdown then shows it as a row of its own. Falling back to 0
    /// made Enter apply the first step, the maximum (47 dB + amp).
    pub(super) fn tx_power_value_index(&self) -> Option<usize> {
        TX_POWER_STEPS
            .iter()
            .position(|&(g, a)| g == self.tx_vga_gain && a == self.tx_amp)
    }

    /// Settings dropdown: apply [TX_POWER_STEPS]\[index\].
    pub(super) fn apply_tx_power_value(&mut self, index: usize) {
        if let Some(&(gain, amp)) = TX_POWER_STEPS.get(index) {
            self.set_tx_power(gain, amp);
        }
    }

    /// Transmissions waiting in the queue.
    pub(super) fn queued_transmits(&self) -> usize {
        self.pending_transmit_queue.len()
    }

    /// Run the next queued transmission. The main loop calls this once per pass, after drawing,
    /// so the UI shows the progress and reads keys between transmissions (Esc cancels the rest);
    /// when the queue runs dry, the batch's summary ("Replayed 3 of 5 …") becomes the status.
    pub fn step_transmit_queue(&mut self) -> Result<()> {
        if !self.has_pending_transmit() {
            return Ok(());
        }
        self.run_one_pending_transmit()?;
        self.input.tx_done += 1;
        if !self.has_pending_transmit() {
            self.input.tx_done = 0;
            if let Some(summary) = self.input.tx_summary.take() {
                self.status_message = Some(summary);
            }
        }
        Ok(())
    }

    /// Esc while transmissions are queued: drop the ones not sent yet. False when none were queued.
    pub fn cancel_transmit_queue(&mut self) -> bool {
        let left = self.queued_transmits();
        if left == 0 {
            return false;
        }
        let done = self.input.tx_done;
        self.pending_transmit_queue.clear();
        // As run_one_pending_transmit does when the queue empties.
        if let Some((i, prev)) = self.pending_transmit_restore.take() {
            if let Some(r) = self.radios.get_mut(i) {
                r.state = prev;
            }
        }
        self.input.tx_done = 0;
        self.input.tx_summary = None;
        self.status_message = Some(format!("Transmit cancelled after {} of {}", done, done + left));
        true
    }

    /// "Transmitting 3/20… (Esc: cancel)" while two or more queued transmissions run, for
    /// [App::job_status].
    pub(super) fn tx_progress(&self) -> Option<String> {
        let left = self.queued_transmits();
        let total = self.input.tx_done + left;
        (left > 0 && total > 1)
            .then(|| format!("Transmitting {}/{}… (Esc: cancel)", self.input.tx_done + 1, total))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn txpower_arguments() {
        assert_eq!(parse_txpower(&["30"], 47, true), Some((30, true)));
        assert_eq!(parse_txpower(&["30", "amp", "off"], 47, true), Some((30, false)));
        assert_eq!(parse_txpower(&["0", "AMP", "ON"], 47, false), Some((0, true)));
        assert_eq!(parse_txpower(&["20", "off"], 47, true), Some((20, false)));
        assert_eq!(parse_txpower(&["amp", "off"], 40, true), Some((40, false)));
        assert_eq!(parse_txpower(&["99"], 47, true), Some((99, true)), "range checked when applied");
        for bad in [&["amp"][..], &["loud"], &["30", "amp"], &["30", "amp", "maybe"], &["30", "40"], &[]] {
            assert_eq!(parse_txpower(bad, 47, true), None, "{bad:?}");
        }
    }

    /// Press `button` `presses` times on `c` as the app does (encode from the capture, keep the
    /// frame sent when it advanced); the counters sent, or None at the first press that did not
    /// advance.
    fn press(reg: &mut ProtocolRegistry, c: &mut Capture, button: Option<u8>, presses: usize) -> Option<Vec<u16>> {
        let protocol = c.protocol.clone().unwrap();
        let mut counters = Vec::new();
        for _ in 0..presses {
            let from = DecodedSignal { encoder_capable: true, ..c.decoded_signal() };
            let button = button.or(from.button).unwrap_or(0);
            let upload = reg.get(&protocol)?.encode(&from, button)?;
            let sent = sent_frame(reg, &protocol, &from, button, &upload).filter(|s| advances(&from, s))?;
            assert_eq!(sent.serial, from.serial, "{protocol}");
            store_sent_frame(c, &sent);
            counters.push(sent.counter?);
        }
        Some(counters)
    }

    /// "Send next code" / "Send button…": every remote type `:new` creates sends a new code on
    /// every press (it used to send the same frame again, so a new remote worked once), FAAC SLH
    /// keeping its seed; fixed codes keep their capture (Revers RB2 and Marantec24 do not decode
    /// their own upload, as in the references).
    #[test]
    fn rolling_codes_advance_on_every_press() {
        use crate::protocols::add_manually::{self, Rng, TYPES};
        crate::protocols::keys::load_keystore_from_embedded();
        let mut reg = ProtocolRegistry::new();
        let mut rolling = 0;
        for ty in TYPES.iter().filter(|ty| add_manually::availability(ty).is_ok()) {
            let gen = add_manually::fill(ty, &mut Rng::seeded(1));
            let key = add_manually::generate(&gen, &mut Rng::seeded(1)).unwrap();
            let mut c = crate::export::flipper::capture_from_key(&key, &reg, 1).unwrap();
            let before = c.clone();
            let protocol = c.protocol.clone().unwrap();
            let fixed = crate::protocols::protocol_info(&protocol).is_some_and(|i| i.encryption == "Fixed Code");
            match press(&mut reg, &mut c, None, 3) {
                Some(counters) => {
                    assert!(!fixed, "{}: a fixed code advanced", ty.name);
                    assert!(counters[0] != counters[1] && counters[1] != counters[2], "{}: {counters:X?}", ty.name);
                    assert_eq!(c.data_extra.is_some(), before.data_extra.is_some(), "{}: encoder state kept", ty.name);
                    rolling += 1;
                }
                None => {
                    assert!(fixed, "{}: did not advance", ty.name);
                    assert_eq!(c.decoded_signal(), before.decoded_signal(), "{}: a fixed code changed", ty.name);
                    // "Send button…" with any of its buttons does not change a fixed code either.
                    for b in crate::protocols::gate_buttons::tx_buttons(&protocol, &before.decoded_signal()) {
                        assert_eq!(press(&mut reg, &mut before.clone(), Some(b), 1), None, "{} button {b:#x}", ty.name);
                    }
                }
            }
        }
        assert!(rolling >= 20, "only {rolling} rolling types");
    }

    /// Car remotes (TX Lock … Panic, `:lock` …): IMPORTS key files advance on every press too.
    #[test]
    fn car_remotes_advance_on_every_press() {
        use crate::capture::ButtonCommand;
        crate::protocols::keys::load_keystore_from_embedded();
        let mut reg = ProtocolRegistry::new();
        for file in ["HYUNDAI/HYU V0 N1.sub", "HYUNDAI/HYU V2 N1.sub", "HYUNDAI/HYU V6_test.sub", "KIA/KIA V5 N3.sub", "VAG/Passat_b5_3bg.sub"] {
            let path = std::path::Path::new("IMPORTS").join(file);
            let Ok(crate::export::flipper::SubContent::Key(key)) = crate::export::flipper::read_sub(&path) else {
                eprintln!("Skip: {path:?} not found (run from crate root)");
                continue;
            };
            let mut c = crate::export::flipper::capture_from_key(&key, &reg, 1).unwrap();
            let start = c.counter;
            let lock = press(&mut reg, &mut c, Some(ButtonCommand::Lock.code()), 2).unwrap_or_else(|| panic!("{file}: Lock"));
            let unlock = press(&mut reg, &mut c, Some(ButtonCommand::Unlock.code()), 1).unwrap_or_else(|| panic!("{file}: Unlock"));
            let sent = [lock, unlock].concat();
            assert!(sent.windows(2).all(|w| w[0] != w[1]) && Some(sent[0]) != start, "{file}: {start:X?} → {sent:X?}");
        }
    }

    /// The capture keeps its recording (Replay), name, frequency and metadata; the fields become
    /// the frame sent.
    #[test]
    fn storing_the_frame_sent_keeps_the_recording() {
        use crate::capture::StoredLevelDuration;
        let mut c = Capture::from_pairs(4, 433_920_000, vec![StoredLevelDuration { level: true, duration_us: 400 }]);
        c.protocol = Some("KeeLoq (DoorHan)".into());
        c.make = Some("DoorHan".into());
        (c.serial, c.button, c.counter, c.data, c.data_extra) = (Some(0x123), Some(2), Some(7), 0xAA, Some(9));
        let sent = DecodedSignal {
            serial: Some(0x123),
            button: Some(4),
            counter: Some(8),
            crc_valid: true,
            data: 0xBB,
            data_count_bit: 66,
            encoder_capable: true,
            extra: Some(10),
            protocol_display_name: Some("KeeLoq".into()),
        };
        assert!(advances(&c.decoded_signal(), &sent));
        store_sent_frame(&mut c, &sent);
        assert_eq!((c.serial, c.button, c.counter, c.data, c.data_count_bit, c.data_extra), (Some(0x123), Some(4), Some(8), 0xBB, 66, Some(10)));
        assert_eq!((c.id, c.frequency, c.raw_pairs.len()), (4, 433_920_000, 1));
        assert_eq!((c.protocol.as_deref(), c.make.as_deref()), (Some("KeeLoq (DoorHan)"), Some("DoorHan")));
        // A fixed code (no counter, or one that did not move) is not a new code.
        assert!(!advances(&c.decoded_signal(), &DecodedSignal { counter: None, ..sent.clone() }));
        assert!(!advances(&c.decoded_signal(), &sent));
    }

    #[test]
    fn tx_power_steps_are_valid_and_labelled() {
        assert_eq!(TX_POWER_STEPS[0], (TX_VGA_MAX, true), "first entry = KAT ≤ 1.4 default");
        assert!(TX_POWER_STEPS.iter().all(|&(g, _)| g <= TX_VGA_MAX));
        assert_eq!(tx_power_label(47, true), "47 dB + amp");
        assert_eq!(tx_power_label(30, false), "30 dB");
    }
}
