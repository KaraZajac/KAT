//! VW-1: the oldest Volkswagen Group rolling-code remote (VW/Audi/Seat/Skoda, ~1998–2005) —
//! **decode and transmit**.
//!
//! **KAT's own decoder**, worked out from three recordings of one VW flip key a tester sent in
//! (`IMPORTS/VAG/old_vw_remote_hold_5lock_5unlock.sub` and the 300-press
//! `IMPORTS/VAG/more/old_vw_remote_{lock,unlock}_x300.sub`, 433.93 MHz, on-off keying, 628 presses
//! read). The key's transmitter is Hella's (labelled `HLO 1J0 959 753 A`, the Golf Mk4 / Bora
//! generation, radio approvals of 1998); its controller is a Motorola `ZC421689CDW`. No reference
//! in KAT's set reads the signal.
//!
//! The scheme is the "VW-1" of Garcia, Oswald, Kasper & Pavlidès, *Lock It and Still Lose It*
//! (USENIX Security 2016, §3.2): there is **no secret key** and no checksum, only an obfuscation of
//! the UID and a 24-bit linear feedback shift register. A single capture therefore yields the UID
//! and the register's state, and the next valid code is the state advanced a fixed number of ticks.
//! Not KAT's [VAG](super::vag) (the newer 80-bit AUT64/XTEA frame at 434.4 MHz).
//!
//! **Air format**: pulse-width coding at 667 bit/s; one unit is ~500 µs. A press is a start pulse
//! of three units of carrier and three of pause, then 60 data bits sent LSB-first within each byte
//! (a `0` is one unit of carrier and two of pause; a `1` is two of carrier and one of pause), then
//! a stop of two units of carrier and a ~10 ms pause. The frame is sent two or three times; presses
//! are about 90 ms apart.
//!
//! **Frame (60 bits)**:
//!
//! | Bits (LSB-first) | Field |
//! |---|---|
//! | 0..32 | UID, obfuscated: each of the four bytes XORed with `m` = byte4 ^ byte5 ^ byte6 |
//! | 32..56 | the 24-bit register state (bytes 4, 5, 6) |
//! | 56..60 | button: `1` Unlock, `2` Lock |
//!
//! **The rolling code**: the three state bytes are a 24-bit LFSR with feedback polynomial
//! x^24 + x^7 + x^2 + x + 1, clocked 23 times for each new press. Lock and Unlock share one
//! register. The obfuscation mask `m` is the XOR of the three state bytes, so the UID comes back by
//! XORing it into the first four bytes; the recorded key is UID `2AA93AC5`. All 628 recorded
//! presses place on one register run (lock 0..310, unlock 311..617, hold 622..631), each press one
//! step of 23 ticks on from the last.
//!
//! **What one remote cannot tell**: whether the polynomial and the 23-tick step are the same in
//! every such remote (they are the scheme's, per the paper, not the key's), and whether a
//! three-button variant sends other button codes.
//!
//! **Gate** (there is no checksum): the start pulse, 60 strict PWM bits each a short/long or
//! long/short pair, the stop carrier and a pause of 2.5 ms or more behind it (or the end of the
//! input), a non-zero state, and a button of `1` or `2`.
//!
//! **Transmit**: the captured remote's next press — the state advanced one step (23 ticks), the UID
//! re-obfuscated with the new state, and Lock or Unlock (any other command repeats the captured
//! button). The capture then keeps the frame sent ([ProtocolDecoder::advances]: the register
//! moved), so every press sends a new code. Not tried on a car.
//!
//! **Fields**: `data` = the 60 frame bits, `serial` = the UID, `button` = the button nibble,
//! `extra` = the 24-bit state; no counter (the state is a register, not a linear count).

use super::info::prelude::*;
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "Volkswagen",
    category: Category::Vehicle,
    buttons: Some(button_name),
    presets: &[&AM650],
};

const NAME: &str = "VW-1";
/// One unit of the pulse-width code: a bit is three units (1500 µs). The tester's recordings gave a
/// carrier/pause of ~500/1000 µs for a 0, ~1000/500 µs for a 1, and a ~1500/1500 µs start.
const UNIT: u32 = 500;
const FRAME_BITS: usize = 60;
/// The 24-bit LFSR feedback: x^24 + x^7 + x^2 + x + 1.
const POLY: u32 = 0x100_0087;
/// Ticks the register is clocked for each new press.
const STEP_TICKS: usize = 23;
const UNLOCK: u8 = 1;
const LOCK: u8 = 2;
/// Copies of the frame a press sends, and the pause between them (the recordings: 2–3 copies,
/// ~10 ms apart).
const COPIES: usize = 3;
const COPY_GAP: u32 = 10_000;
/// The pause that must close a frame (behind its stop carrier): the ~10 ms between copies and the
/// ~90 ms between presses are well above it, a data bit's pause well below. This is what tells a
/// VW-1 frame from another 500/1000 µs pulse-width signal that runs straight on (e.g. Nice One).
const GAP_MIN: u32 = 2_500;

fn button_name(_name: &str, code: u8) -> Option<&'static str> {
    match code {
        UNLOCK => Some("Unlock"),
        LOCK => Some("Lock"),
        _ => None,
    }
}

/// One LFSR clock: shift left, and on carry out XOR the feedback taps.
fn tick(mut state: u32) -> u32 {
    state <<= 1;
    if state & 0x100_0000 != 0 {
        state ^= POLY;
    }
    state & 0xFF_FFFF
}

/// The register advanced one press (23 ticks).
fn step(mut state: u32) -> u32 {
    for _ in 0..STEP_TICKS {
        state = tick(state);
    }
    state
}

/// A frame's fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Fields {
    /// The de-obfuscated 32-bit UID.
    uid: u32,
    /// The 24-bit register state.
    state: u32,
    button: u8,
}

impl Fields {
    /// The fields of 60 frame bits (stored LSB-first, bit 0 sent first), or `None` when the state
    /// is zero or the button is not Lock or Unlock.
    fn parse(frame: u64) -> Option<Self> {
        let byte = |k: u32| ((frame >> (8 * k)) & 0xFF) as u8;
        let (s0, s1, s2) = (byte(4), byte(5), byte(6));
        let state = u32::from(s0) | u32::from(s1) << 8 | u32::from(s2) << 16;
        let button = ((frame >> 56) & 0xF) as u8;
        if state == 0 || !matches!(button, UNLOCK | LOCK) {
            return None;
        }
        let m = s0 ^ s1 ^ s2;
        let uid = u32::from(byte(0) ^ m) << 24
            | u32::from(byte(1) ^ m) << 16
            | u32::from(byte(2) ^ m) << 8
            | u32::from(byte(3) ^ m);
        Some(Self { uid, state, button })
    }

    /// The 60 frame bits.
    fn frame(&self) -> u64 {
        let (s0, s1, s2) = (self.state as u8, (self.state >> 8) as u8, (self.state >> 16) as u8);
        let m = s0 ^ s1 ^ s2;
        let by = [
            ((self.uid >> 24) as u8) ^ m,
            ((self.uid >> 16) as u8) ^ m,
            ((self.uid >> 8) as u8) ^ m,
            (self.uid as u8) ^ m,
            s0,
            s1,
            s2,
        ];
        let mut frame = 0u64;
        for (k, b) in by.iter().enumerate() {
            frame |= u64::from(*b) << (8 * k);
        }
        frame | (u64::from(self.button & 0xF) << 56)
    }

    fn signal(&self) -> DecodedSignal {
        DecodedSignal {
            serial: Some(self.uid),
            button: Some(self.button),
            counter: None,
            crc_valid: false,
            data: self.frame(),
            data_count_bit: FRAME_BITS,
            encoder_capable: true,
            extra: Some(u64::from(self.state)),
            protocol_display_name: None,
        }
    }

    /// The remote's next press: KAT's Lock and Unlock; any other command keeps the captured button.
    fn next(&self, command: u8) -> Self {
        let button = match command {
            0x01 => LOCK,
            0x02 => UNLOCK,
            _ => self.button,
        };
        Self { button, state: step(self.state), ..*self }
    }
}

/// The pulses of a press: the frame [COPIES] times.
fn upload(frame: u64) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(COPIES * (2 + 2 * FRAME_BITS + 2));
    for copy in 0..COPIES {
        out.push(LevelDuration::new(true, 3 * UNIT));
        out.push(LevelDuration::new(false, 3 * UNIT));
        for i in 0..FRAME_BITS {
            let one = (frame >> i) & 1 == 1;
            out.push(LevelDuration::new(true, if one { 2 * UNIT } else { UNIT }));
            out.push(LevelDuration::new(false, if one { UNIT } else { 2 * UNIT }));
        }
        out.push(LevelDuration::new(true, 2 * UNIT));
        out.push(LevelDuration::new(false, if copy + 1 < COPIES { COPY_GAP } else { 100_000 }));
    }
    out
}

fn is_short(d: u32) -> bool {
    (250..=750).contains(&d)
}
fn is_long(d: u32) -> bool {
    (751..=1250).contains(&d)
}
fn is_start(d: u32) -> bool {
    (1251..=2200).contains(&d)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    /// Waiting for the start pulse's carrier.
    Idle,
    /// The start pulse's carrier was seen; waiting for its pause.
    StartLow,
    /// Waiting for a data bit's carrier.
    BitHigh,
    /// Waiting for a data bit's pause; the carrier before it was long (a 1).
    BitLow(bool),
    /// 60 bits are in; waiting for the stop carrier.
    Stop,
    /// The stop carrier is in; waiting for the closing pause.
    Gap,
}

/// VW-1 decoder: a state machine over the pulses, one frame per start pulse.
pub struct Vw1Decoder {
    step: Step,
    frame: u64,
    count: usize,
}

impl Vw1Decoder {
    pub fn new() -> Self {
        Self { step: Step::Idle, frame: 0, count: 0 }
    }

    /// A pulse that fits nowhere: start over, taking a start-length carrier as the start pulse.
    fn restart(&mut self, level: bool, duration: u32) {
        self.step = if level && is_start(duration) { Step::StartLow } else { Step::Idle };
    }
}

impl Default for Vw1Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for Vw1Decoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: UNIT, te_long: 2 * UNIT, te_delta: UNIT / 2, min_count_bit: FRAME_BITS }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        *self = Self::new();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Idle => {
                if level && is_start(duration) {
                    self.step = Step::StartLow;
                }
            }
            Step::StartLow => {
                if !level && is_start(duration) {
                    (self.step, self.frame, self.count) = (Step::BitHigh, 0, 0);
                } else {
                    self.restart(level, duration);
                }
            }
            Step::BitHigh => match (level, is_short(duration), is_long(duration)) {
                (true, true, _) => self.step = Step::BitLow(false),
                (true, _, true) => self.step = Step::BitLow(true),
                _ => self.restart(level, duration),
            },
            Step::BitLow(high_long) => {
                let bit = if !level && !high_long && is_long(duration) {
                    Some(false)
                } else if !level && high_long && is_short(duration) {
                    Some(true)
                } else {
                    None
                };
                match bit {
                    Some(b) => {
                        self.frame |= u64::from(b) << self.count;
                        self.count += 1;
                        self.step = if self.count == FRAME_BITS { Step::Stop } else { Step::BitHigh };
                    }
                    None => self.restart(level, duration),
                }
            }
            Step::Stop => {
                if level && (is_long(duration) || is_start(duration)) {
                    self.step = Step::Gap;
                } else {
                    self.restart(level, duration);
                }
            }
            Step::Gap => {
                if !level && duration >= GAP_MIN {
                    self.step = Step::Idle;
                    return Fields::parse(self.frame).map(|f| f.signal());
                }
                self.restart(level, duration);
            }
        }
        None
    }

    /// A frame whose stop carrier was the last pulse of the input (a recording cut right after it)
    /// is complete: only the pause behind it is missing.
    fn flush(&mut self) -> Option<DecodedSignal> {
        let done = self.step == Step::Gap;
        self.step = Step::Idle;
        done.then(|| Fields::parse(self.frame).map(|f| f.signal())).flatten()
    }

    /// VW-1 has no counter: its rolling state is the register (`extra`), which every press moves.
    fn advances(&self, from: &DecodedSignal, sent: &DecodedSignal) -> bool {
        sent.extra.is_some() && sent.extra != from.extra
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != FRAME_BITS {
            return None;
        }
        Some(upload(Fields::parse(decoded.data)?.next(button).frame()))
    }
}

#[cfg(test)]
mod tests {
    use super::super::f_common::test_support::*;
    use super::*;

    /// The first lock and unlock presses of the recorded remote (UID `2AA93AC5`).
    const LOCK_FRAME: u64 = 0x025A_E2FE_837C_EF6C;
    const UNLOCK_FRAME: u64 = 0x0107_5D6B_F40B_981B;

    fn fields(frame: u64) -> Fields {
        Fields::parse(frame).unwrap()
    }

    #[test]
    fn lfsr_and_fields() {
        // the register's period is the full 2^24 - 1 and the step is 23 ticks
        let f = fields(LOCK_FRAME);
        assert_eq!((f.uid, f.state, f.button), (0x2AA9_3AC5, 0x5AE2FE, LOCK));
        assert_eq!(step(f.state), 0x7AE29F);
        assert_eq!(f.frame(), LOCK_FRAME);
        let u = fields(UNLOCK_FRAME);
        assert_eq!((u.uid, u.state, u.button), (0x2AA9_3AC5, 0x075D6B, UNLOCK));
        assert_eq!(u.frame(), UNLOCK_FRAME);
        let d = f.signal();
        assert_eq!((d.serial, d.button, d.counter, d.extra), (Some(0x2AA9_3AC5), Some(LOCK), None, Some(0x5AE2FE)));
        assert_eq!((d.data, d.data_count_bit, d.encoder_capable), (LOCK_FRAME, 60, true));
        // every state and button gives a frame that reads back
        for state in [1u32, 0xFF_FFFF, 0x5AE2FE, 0x123456, 0xAAAAAA] {
            for button in [UNLOCK, LOCK] {
                let f = Fields { uid: 0x0F1E_2D3C, state, button };
                assert_eq!(Fields::parse(f.frame()), Some(f), "{state:X} {button}");
            }
        }
        // the register runs the full period
        let mut s = 1u32;
        let mut n = 0u64;
        loop {
            s = tick(s);
            n += 1;
            if s == 1 {
                break;
            }
        }
        assert_eq!(n, (1 << 24) - 1);
        // the gate: a zero state or an unknown button is rejected
        assert_eq!(Fields::parse(LOCK_FRAME & !(0xFF_FFFF << 32)), None, "zero state");
        assert_eq!(Fields::parse((LOCK_FRAME & !(0xF << 56)) | (3 << 56)), None, "button 3");
        assert_eq!((button_name(NAME, UNLOCK), button_name(NAME, LOCK), button_name(NAME, 3)), (Some("Unlock"), Some("Lock"), None));
    }

    #[test]
    fn decodes_what_it_sends() {
        let wire = upload(LOCK_FRAME);
        let got = feed_all(&mut Vw1Decoder::new(), &wire);
        assert_eq!(got.len(), COPIES);
        assert!(got.iter().all(|d| *d == fields(LOCK_FRAME).signal()));
        assert_eq!(feed_resetting(&mut Vw1Decoder::new(), &wire).len(), COPIES);
        // as a Flipper records it: carrier 60 µs short, pauses 60 µs long
        let skewed: Vec<_> = wire
            .iter()
            .map(|p| LevelDuration::new(p.level, if p.level { p.duration_us - 60 } else { p.duration_us + 60 }))
            .collect();
        assert_eq!(feed_resetting(&mut Vw1Decoder::new(), &skewed).len(), COPIES);
        // a bit pair whose two halves are the same length breaks that copy (the other two decode)
        let mut bad = upload(LOCK_FRAME);
        bad[2 + 2 * 10].duration_us = UNIT;
        bad[2 + 2 * 10 + 1].duration_us = UNIT;
        assert_eq!(feed_all(&mut Vw1Decoder::new(), &bad).len(), COPIES - 1);
        // 59 bits (a start pulse cut one bit short): no frame, also at the end of the input
        let one = &wire[..2 + 2 * (FRAME_BITS - 1)];
        let mut dec = Vw1Decoder::new();
        assert!(feed_all(&mut dec, one).is_empty() && dec.flush().is_none());
        // the input ends right after the stop carrier (a recording cut there): the flush gives it
        let cut = &wire[..2 + 2 * FRAME_BITS + 1];
        let mut dec = Vw1Decoder::new();
        assert!(feed_all(&mut dec, cut).is_empty());
        assert_eq!(dec.flush(), Some(fields(LOCK_FRAME).signal()));
        assert!(dec.flush().is_none());
    }

    #[test]
    fn sends_the_next_press() {
        let dec = Vw1Decoder::new();
        let captured = fields(LOCK_FRAME).signal();
        for (command, button) in [(0x01, LOCK), (0x02, UNLOCK), (0x04, LOCK), (0x08, LOCK)] {
            let got = feed_all(&mut Vw1Decoder::new(), &dec.encode(&captured, command).unwrap());
            assert_eq!(got.len(), COPIES, "{command}");
            let f = fields(got[0].data);
            assert_eq!((f.uid, f.state, f.button), (0x2AA9_3AC5, 0x7AE29F, button), "{command}");
        }
        // the unlock capture's next press, Unlock again
        let next = fields(UNLOCK_FRAME).next(0x02);
        assert_eq!(next.state, step(0x075D6B));
        assert!(dec.encode(&DecodedSignal::new(LOCK_FRAME, 64), 1).is_none());
    }

    /// The recordings: the held file (five Lock then five Unlock presses held down), and the two
    /// 300-press files. Every frame reads with UID `2AA93AC5`; consecutive presses step the register.
    #[test]
    fn imports_recordings() {
        let dir = std::path::Path::new("IMPORTS/VAG");
        let hold = dir.join("old_vw_remote_hold_5lock_5unlock.sub");
        if !hold.is_file() {
            eprintln!("Skip: IMPORTS not found (run from crate root)");
            return;
        }
        let files = [
            (hold, 100usize),
            (dir.join("more/old_vw_remote_lock_x300.sub"), 580),
            (dir.join("more/old_vw_remote_unlock_x300.sub"), 560),
        ];
        for (file, least) in files {
            let (_, wire) = sub_pulses(&file);
            let got = feed_resetting(&mut Vw1Decoder::new(), &wire);
            eprintln!("VW-1 {}: {} frames", file.display(), got.len());
            assert!(got.len() >= least, "{}: {}", file.display(), got.len());
            assert!(got.iter().all(|d| d.serial == Some(0x2AA9_3AC5)));
            assert!(got.iter().all(|d| matches!(d.button, Some(UNLOCK) | Some(LOCK))));
            // each new state is one step on from the previous distinct state
            let mut states: Vec<u32> = got.iter().map(|d| d.extra.unwrap() as u32).collect();
            states.dedup();
            assert!(states.windows(2).all(|w| step(w[0]) == w[1]), "{}", file.display());
        }
    }
}
