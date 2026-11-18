//! Opel Rolling: an old two-button Opel remote key (lock and unlock on a rubber pad, a red LED
//! beside it; the board is marked `28 5442 01`) — **decode and transmit**.
//!
//! **KAT's own decoder**, worked out from four recordings of one remote
//! (`IMPORTS/OPEL/opel_two_button_remote_*.sub` and `IMPORTS/OPEL/more/opel_two_button_remote_*.sub`,
//! 433.89 MHz, on-off keying, 399 presses read). No reference reads the signal; the name is KAT's.
//! Not [Opel V0](super::opel_v0) and not [Opel V1](super::opel_v1).
//!
//! **Air format**: the bit is the length of a whole period, in units of 150 µs. A 0 is two units
//! of pause and two of carrier (600 µs), a 1 three and three (900 µs). A press is the frame three
//! times. The first copy has 57 periods of 600 µs in front (carrier first), the other two have
//! four, behind a pause of 0.8 to 1.7 ms. Each copy starts with a sync: two units of carrier,
//! four of pause, four of carrier.
//!
//! **Frame (60 bits, 15 nibbles)**:
//!
//! | Nibble | Field |
//! |---|---|
//! | 0 | `A` Unlock, `B` Lock |
//! | 1..6 | the remote's serial (24 bits) |
//! | 7..12 | six nibbles of a plain value, each plus `r` (no carry) and some then inverted |
//! | 13 | `r` inverted: a nibble the remote picks anew for every press |
//! | 14 | check: start with 0; for each of the 14 nibbles before, rotate left by one bit and XOR the nibble in; invert |
//!
//! Which of the six nibbles are inverted depends on `r` alone ([FLAGS], one row of six bits per
//! `r`; for `r` below 8 the row is the three bits of `r`, each as a pair `01` or `10`). The plain
//! value of the recorded remote is `ACA` and a 12-bit counter, which goes up by one per press
//! (0x2F7 to 0x57F in the recordings, both buttons on one counter). All 399 frames fit.
//!
//! **While a button is held** the remote sends, after the press, a short frame every 40 ms:
//! two periods of 600 µs, the sync and 32 bits: `9`, the serial, and the same check without the
//! last inversion. It carries no counter. KAT lists it as the button `9`.
//!
//! **What one remote cannot tell**: whether [FLAGS] and the three nibbles `ACA` are the same in
//! every such remote, and whether the counter is longer than 12 bits.
//!
//! **Gate**: a period of 600 µs or more of them, the sync, 60 periods whose two halves are of one
//! length, and the check nibble. The held frame: 32 periods and a pause, its first nibble `9`,
//! its check.
//!
//! **Transmit**: the next press of the captured remote: the counter plus one (12 bits), Lock or
//! Unlock (another command sends the captured button again), `r` plus one. A held frame is not
//! sent. Not tried on a car.
//!
//! **Fields**: `data` = the 60 bits (the held frame: 32), `serial`, `button` = nibble 0,
//! `counter` = the 12 bits, `extra` = the plain value (24 bits).

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
    make: "Opel",
    category: Category::Vehicle,
    buttons: Some(button_name),
    presets: &[&AM650],
};

const NAME: &str = "Opel Rolling";
/// One unit. Half a period is two units (0) or three (1); the tester's analyzer gave 301 and
/// 449 µs, a Flipper recording 276/325 and 425/474 µs (carrier/pause).
const UNIT: u32 = 150;
const TOL: u32 = 60;
const FRAME_BITS: usize = 60;
const HELD_BITS: usize = 32;
/// Periods of 600 µs in front of the first copy and of the others (a held frame has two).
const LEAD_FIRST: usize = 57;
const LEAD_NEXT: usize = 4;
/// Carriers of two units KAT wants before the sync's pause: one period and the sync's own.
const LEAD_MIN: u8 = 2;
const COPIES: usize = 3;
/// The pause between copies: 823 to 1688 µs in the recordings, by the number of 1 bits in the frame.
const COPY_GAP: u32 = 1200;
const UNLOCK: u8 = 0xA;
const LOCK: u8 = 0xB;
const HELD: u8 = 0x9;

/// For `r` = 0..15: which of the six plain nibbles are inverted behind the addition (bit 5 = the first).
const FLAGS: [u8; 16] = [0x15, 0x16, 0x19, 0x1A, 0x25, 0x26, 0x29, 0x2A, 0x23, 0x2D, 0x27, 0x24, 0x3C, 0x1D, 0x1E, 0x35];

fn button_name(_name: &str, code: u8) -> Option<&'static str> {
    match code {
        UNLOCK => Some("Unlock"),
        LOCK => Some("Lock"),
        HELD => Some("Held"),
        _ => None,
    }
}

/// The check over `nibbles` nibbles of `value` (highest first), before the last inversion.
fn check(value: u64, nibbles: usize) -> u8 {
    (0..nibbles).rev().fold(0u8, |acc, i| (((acc << 1) | (acc >> 3)) & 0xF) ^ ((value >> (4 * i)) & 0xF) as u8)
}

/// A press frame's fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Fields {
    button: u8,
    serial: u32,
    /// The plain value, 24 bits: three nibbles and the 12-bit counter.
    plain: u32,
    r: u8,
}

impl Fields {
    /// The fields of 60 frame bits whose check nibble is right.
    fn parse(frame: u64) -> Option<Self> {
        if check(frame >> 4, 14) ^ 0xF != (frame & 0xF) as u8 {
            return None;
        }
        let r = (!(frame >> 4) & 0xF) as u8;
        let plain = (0..6).fold(0u32, |p, i| {
            let n = ((frame >> (28 - 4 * i)) & 0xF) as u8;
            let v = if (FLAGS[usize::from(r)] >> (5 - i)) & 1 == 1 { n ^ 0xF } else { n };
            (p << 4) | u32::from(v.wrapping_sub(r) & 0xF)
        });
        Some(Self { button: (frame >> 56) as u8 & 0xF, serial: ((frame >> 32) & 0xFF_FFFF) as u32, plain, r })
    }

    /// The 60 frame bits.
    fn frame(&self) -> u64 {
        let mut frame = (u64::from(self.button & 0xF) << 24) | u64::from(self.serial & 0xFF_FFFF);
        for i in 0..6 {
            let v = (((self.plain >> (20 - 4 * i)) & 0xF) as u8 + self.r) & 0xF;
            let n = if (FLAGS[usize::from(self.r & 0xF)] >> (5 - i)) & 1 == 1 { v ^ 0xF } else { v };
            frame = (frame << 4) | u64::from(n);
        }
        frame = (frame << 4) | u64::from(!self.r & 0xF);
        (frame << 4) | u64::from(check(frame, 14) ^ 0xF)
    }

    fn signal(&self) -> DecodedSignal {
        DecodedSignal {
            serial: Some(self.serial),
            button: Some(self.button),
            counter: Some((self.plain & 0xFFF) as u16),
            crc_valid: true,
            data: self.frame(),
            data_count_bit: FRAME_BITS,
            encoder_capable: true,
            extra: Some(u64::from(self.plain)),
            protocol_display_name: None,
        }
    }

    /// The remote's next press: KAT's Lock and Unlock; any other command is the captured button again.
    fn next(&self, command: u8) -> Self {
        let button = match command {
            0x01 => LOCK,
            0x02 => UNLOCK,
            _ => self.button,
        };
        let plain = (self.plain & 0xFF_F000) | ((self.plain + 1) & 0xFFF);
        Self { button, plain, r: (self.r + 1) & 0xF, ..*self }
    }
}

/// The decode of a held frame's 32 bits: `9`, the serial, the check.
fn held(frame: u64) -> Option<DecodedSignal> {
    ((frame >> 28) as u8 == HELD && check(frame >> 4, 7) == (frame & 0xF) as u8).then(|| DecodedSignal {
        serial: Some(((frame >> 4) & 0xFF_FFFF) as u32),
        button: Some(HELD),
        counter: None,
        crc_valid: true,
        data: frame,
        data_count_bit: HELD_BITS,
        encoder_capable: false,
        extra: None,
        protocol_display_name: None,
    })
}

/// The pulses of a press: the frame [COPIES] times and a pause.
fn upload(frame: u64) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(2 * (LEAD_FIRST + 2 * LEAD_NEXT + COPIES * (2 + FRAME_BITS)));
    for copy in 0..COPIES {
        for _ in 0..if copy == 0 { LEAD_FIRST } else { LEAD_NEXT } {
            out.push(LevelDuration::new(true, 2 * UNIT));
            out.push(LevelDuration::new(false, 2 * UNIT));
        }
        out.push(LevelDuration::new(true, 2 * UNIT));
        out.push(LevelDuration::new(false, 4 * UNIT));
        out.push(LevelDuration::new(true, 4 * UNIT));
        for i in (0..FRAME_BITS).rev() {
            let half = if (frame >> i) & 1 == 1 { 3 * UNIT } else { 2 * UNIT };
            out.push(LevelDuration::new(false, half));
            out.push(LevelDuration::new(true, half));
        }
        out.push(LevelDuration::new(false, if copy + 1 < COPIES { COPY_GAP } else { 10_000 }));
    }
    out
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    /// Counting periods of 600 µs: the carriers seen so far.
    Lead(u8),
    /// The sync's four units of carrier.
    SyncHigh,
    /// A period's pause.
    Low,
    /// A period's carrier; the pause before it was a long one or not.
    High(bool),
}

/// Opel Rolling decoder: a state machine over the pulses, one frame per sync.
pub struct OpelRollingDecoder {
    step: Step,
    frame: u64,
    count: usize,
}

fn is_units(duration: u32, units: u32) -> bool {
    duration.abs_diff(units * UNIT) <= TOL
}

impl OpelRollingDecoder {
    pub fn new() -> Self {
        Self { step: Step::Lead(0), frame: 0, count: 0 }
    }

    /// A pulse that fits nowhere: start over, with this pulse as the first carrier when it is one.
    fn restart(&mut self, level: bool, duration: u32) {
        self.step = Step::Lead(u8::from(level && is_units(duration, 2)));
    }

    /// The held frame, when 32 periods were read and the input goes on with something else.
    fn held_frame(&self) -> Option<DecodedSignal> {
        (self.step == Step::Low && self.count == HELD_BITS).then(|| held(self.frame)).flatten()
    }
}

impl Default for OpelRollingDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for OpelRollingDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: 2 * UNIT, te_long: 3 * UNIT, te_delta: TOL, min_count_bit: HELD_BITS }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        *self = Self::new();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Lead(highs) => match (level, is_units(duration, 2)) {
                (true, true) => self.step = Step::Lead(highs.saturating_add(1)),
                (false, true) if highs > 0 => {}
                (false, false) if highs >= LEAD_MIN && is_units(duration, 4) => self.step = Step::SyncHigh,
                _ => self.restart(level, duration),
            },
            Step::SyncHigh => {
                if level && is_units(duration, 4) {
                    (self.step, self.frame, self.count) = (Step::Low, 0, 0);
                } else {
                    self.restart(level, duration);
                }
            }
            Step::Low => {
                if !level && (is_units(duration, 2) || is_units(duration, 3)) {
                    self.step = Step::High(is_units(duration, 3));
                } else {
                    let out = self.held_frame();
                    self.restart(level, duration);
                    return out;
                }
            }
            Step::High(long) => {
                if !level || !is_units(duration, if long { 3 } else { 2 }) {
                    self.restart(level, duration);
                    return None;
                }
                self.frame = (self.frame << 1) | u64::from(long);
                self.count += 1;
                self.step = Step::Low;
                if self.count == FRAME_BITS {
                    self.step = Step::Lead(0);
                    return Fields::parse(self.frame).map(|f| f.signal());
                }
            }
        }
        None
    }

    fn flush(&mut self) -> Option<DecodedSignal> {
        let out = self.held_frame();
        self.step = Step::Lead(0);
        out
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

    /// Presses 0x421 (Unlock) and 0x2F7 (Lock) of the recorded remote, and its held frame.
    const UNLOCK_FRAME: u64 = 0xA71_597D_768E_F12B;
    const LOCK_FRAME: u64 = 0xB71_597D_857F_251B;
    const HELD_FRAME: u64 = 0x9715_97D6;

    fn fields(frame: u64) -> Fields {
        Fields::parse(frame).unwrap()
    }

    #[test]
    fn frame_fields() {
        let f = fields(UNLOCK_FRAME);
        assert_eq!((f.button, f.serial, f.plain, f.r), (0xA, 0x71597D, 0xACA421, 0xD));
        assert_eq!(f.frame(), UNLOCK_FRAME);
        let f = fields(LOCK_FRAME);
        assert_eq!((f.button, f.serial, f.plain, f.r), (0xB, 0x71597D, 0xACA2F7, 0xE));
        assert_eq!(f.frame(), LOCK_FRAME);
        let d = f.signal();
        assert_eq!((d.serial, d.button, d.counter, d.extra), (Some(0x71597D), Some(0xB), Some(0x2F7), Some(0xACA2F7)));
        assert_eq!((d.data, d.data_count_bit, d.crc_valid, d.encoder_capable), (LOCK_FRAME, 60, true, true));
        // every r and a spread of plain values give a frame that reads back
        for r in 0..16 {
            for plain in [0, 0xFF_FFFF, 0xACA2F7, 0x123456, 0x5A5A5A, 0x00_0FFF] {
                let f = Fields { button: 0xA, serial: 0x0F1E2D, plain, r };
                assert_eq!(Fields::parse(f.frame()), Some(f), "{r:X} {plain:X}");
            }
        }
        // one wrong bit anywhere: the check nibble does not fit
        for bit in 0..60 {
            assert_eq!(Fields::parse(UNLOCK_FRAME ^ (1 << bit)), None, "bit {bit}");
        }
        let h = held(HELD_FRAME).unwrap();
        assert_eq!((h.serial, h.button, h.counter, h.data_count_bit, h.encoder_capable), (Some(0x71597D), Some(9), None, 32, false));
        assert!(held(HELD_FRAME ^ 0x10).is_none() && held(0xA715_97D6).is_none());
        assert_eq!((button_name(NAME, 0xA), button_name(NAME, 0xB), button_name(NAME, 9)), (Some("Unlock"), Some("Lock"), Some("Held")));
    }

    /// A held frame as the remote sends it: two periods, the sync, 32 bits, a pause.
    fn held_wire(frame: u64, copies: usize) -> Vec<LevelDuration> {
        let mut out = Vec::new();
        for _ in 0..copies {
            for _ in 0..2 {
                out.extend([LevelDuration::new(true, 300), LevelDuration::new(false, 300)]);
            }
            out.extend([LevelDuration::new(true, 300), LevelDuration::new(false, 600), LevelDuration::new(true, 600)]);
            for i in (0..32).rev() {
                let half = if (frame >> i) & 1 == 1 { 450 } else { 300 };
                out.extend([LevelDuration::new(false, half), LevelDuration::new(true, half)]);
            }
            out.push(LevelDuration::new(false, 11_800));
        }
        out
    }

    #[test]
    fn decodes_what_it_sends() {
        let wire = upload(UNLOCK_FRAME);
        assert_eq!(wire.len(), 2 * (57 + 4 + 4) + 3 * (3 + 120 + 1));
        let got = feed_all(&mut OpelRollingDecoder::new(), &wire);
        assert_eq!(got.len(), 3);
        assert!(got.iter().all(|d| *d == fields(UNLOCK_FRAME).signal()));
        assert_eq!(feed_resetting(&mut OpelRollingDecoder::new(), &wire).len(), 3);
        // as a Flipper records it: carrier 25 µs short, pauses 25 µs long
        let skewed: Vec<_> = wire.iter().map(|p| LevelDuration::new(p.level, if p.level { p.duration_us - 25 } else { p.duration_us + 25 })).collect();
        assert_eq!(feed_resetting(&mut OpelRollingDecoder::new(), &skewed).len(), 3);
        // a period whose halves differ, a sync without its lead, a frame cut short
        let first = 2 * 57 + 3;
        let mut bad = wire.clone();
        bad[first + 2 * 20].duration_us = 450 + 300 - bad[first + 2 * 20].duration_us;
        assert_eq!(feed_all(&mut OpelRollingDecoder::new(), &bad).len(), 2);
        assert!(feed_all(&mut OpelRollingDecoder::new(), &wire[2 * 57..first + 120]).is_empty(), "no period of lead");
        assert_eq!(feed_all(&mut OpelRollingDecoder::new(), &wire[2 * 57 - 2..first + 120]).len(), 1, "one period of lead");
        assert!(feed_all(&mut OpelRollingDecoder::new(), &wire[..first + 118]).is_empty());
    }

    #[test]
    fn held_frames() {
        let wire = held_wire(HELD_FRAME, 5);
        let got = feed_resetting(&mut OpelRollingDecoder::new(), &wire);
        assert_eq!(got.len(), 5);
        assert_eq!((got[0].data, got[0].button), (HELD_FRAME, Some(9)));
        // the last one of an input that ends without its pause comes with the flush
        let mut dec = OpelRollingDecoder::new();
        assert!(feed_all(&mut dec, &wire[..wire.len() - 1]).len() == 4);
        assert_eq!(dec.flush().map(|d| d.data), Some(HELD_FRAME));
        assert!(dec.flush().is_none());
        // 31 or 33 periods, a wrong check
        assert!(feed_all(&mut OpelRollingDecoder::new(), &held_wire(HELD_FRAME ^ 1, 2)).is_empty());
        let mut short = held_wire(HELD_FRAME, 1);
        short.drain(7 + 62..7 + 64);
        assert!(feed_all(&mut OpelRollingDecoder::new(), &short).is_empty());
        assert!(OpelRollingDecoder::new().encode(&held(HELD_FRAME).unwrap(), 1).is_none());
    }

    #[test]
    fn sends_the_next_press() {
        let dec = OpelRollingDecoder::new();
        let captured = fields(UNLOCK_FRAME).signal();
        for (command, button) in [(0x01, 0xB), (0x02, 0xA), (0x04, 0xA), (0x08, 0xA)] {
            let got = feed_all(&mut OpelRollingDecoder::new(), &dec.encode(&captured, command).unwrap());
            assert_eq!(got.len(), 3, "{command}");
            assert_eq!((got[0].button, got[0].counter, got[0].extra), (Some(button), Some(0x422), Some(0xACA422)));
            assert_eq!(fields(got[0].data).r, 0xE);
        }
        // the 12 counter bits wrap, the three nibbles before them stay
        let last = Fields { plain: 0xACAFFF, r: 0xF, ..fields(UNLOCK_FRAME) };
        assert_eq!((last.next(2).plain, last.next(2).r), (0xACA000, 0));
        assert!(dec.encode(&DecodedSignal::new(UNLOCK_FRAME ^ 1, 60), 1).is_none());
        assert!(dec.encode(&DecodedSignal::new(UNLOCK_FRAME, 64), 1).is_none());
    }

    /// The recording: five held presses of Lock, each with its held frames behind it (296 bursts, 214
    /// of them read), then five presses of Unlock (four read).
    #[test]
    fn imports_recordings() {
        let dir = std::path::Path::new("IMPORTS/OPEL");
        let file = dir.join("opel_two_button_remote_lock_held_x5_unlock_x5.sub");
        if !file.is_file() {
            eprintln!("Skip: IMPORTS not found (run from crate root)");
            return;
        }
        let (_, wire) = sub_pulses(&file);
        let got = feed_resetting(&mut OpelRollingDecoder::new(), &wire);
        assert!(got.iter().all(|d| d.serial == Some(0x71597D)));
        let mut presses: Vec<_> = got.iter().filter(|d| d.data_count_bit == 60).map(|d| (d.button.unwrap(), d.counter.unwrap())).collect();
        presses.dedup();
        let want: Vec<_> = (0x576..=0x57A).map(|c| (0xB, c)).chain((0x57C..=0x57F).map(|c| (0xA, c))).collect();
        assert_eq!(presses, want);
        let held = got.iter().filter(|d| d.data_count_bit == 32).count();
        eprintln!("OPEL held frames: {held}");
        assert!(held == 214 && got.iter().filter(|d| d.data_count_bit == 32).all(|d| d.data == HELD_FRAME));
    }
}
