//! Suzuki TS002: the two-button remote key Calsonic Kansei made for Suzuki (the case says
//! "Calsonic Kansei Corp. TS002"; an NXP chip inside) — **decode and transmit**.
//!
//! **KAT's own decoder**, worked out from four recordings of one remote
//! (`IMPORTS/SUZUKI/suzuki_ts002_*.sub` and `IMPORTS/SUZUKI/more/suzuki_ts002_*.sub`, 433.91 MHz,
//! on-off keying, 230 presses). No reference reads the signal; the name is the marking on the case.
//!
//! **Air format**: pulse width in a cell of three units of 256.8 µs (440 units of a frame take
//! 112.975 ms in the recordings): a 0 is one unit of carrier and two of pause, a 1 two of carrier
//! and one of pause. A press is four 0 cells, then the frame four times, then one unit of carrier.
//! A frame is four units of carrier, four of pause, 72 data bits and 72 one bits. While the button
//! is held the frame goes on repeating (33 to 37 copies in presses of four to five seconds).
//!
//! **Frame (72 bits)**:
//!
//! | Bits | Field |
//! |---|---|
//! | 8 | `0x14` in every frame of the one remote recorded |
//! | 8 | button: `0x01` Lock, `0x02` Unlock; `0x20` is added once the button has been held |
//! | 24 | the remote's ID |
//! | 32 | the code |
//!
//! **The code is no cipher.** It holds the low eight bits of the press counter, inverted, in four
//! pairs, and between them the ID again, rotated left by that counter byte modulo 24 and XORed
//! with a mask of counter and button bits:
//!
//! ```text
//! code bit   31 30 | 29 28 27 26 | 25 24 | 23 22 | 21 20 | 19 18 | 17 16 | 15 14 | 13 12 11 10 | 9  8  | 7 6 5 4  | 3 2 1 0
//! holds      ~c1 ~c0 | r20..r23  | r0 r1 | ~c7 ~c6 | r2 r3 | r8 r9 | ~c3 ~c2 | r10 r11 | r12..r15 | ~c5 ~c4 | r16..r19 | r4..r7
//! ```
//!
//! `r0..r23` = the rotated ID (`r0` its first bit) XOR the mask, which is, from `r0`:
//! `0 0 b1 b0 | ~c5 ~c4 0 0 | ~c1 ~c0 1 0 | 0 ~c8 ~c7 ~c6 | 1 1 ~c3 ~c2 | 0 1 0 0`, with `b1 b0` the
//! button byte's two low bits and `c8` a ninth counter bit, which is nowhere else in the frame.
//! All 230 frames fit (counter 0x09D to 0x1A0, one step per press, across the four recordings).
//! The thirteen mask bits written as 0 and 1 did not change; they may hold more counter bits.
//!
//! **Gate**: 72 data bits and 72 one bits behind the sync, and the ten mask bits that repeat the
//! counter byte and the button must fit; an ID of all zeros or all ones, which is the same however
//! it is rotated, is not read. `crc_valid` says that the thirteen fixed mask bits are the recorded
//! remote's as well.
//!
//! **Transmit**: the next press of the captured remote: the counter plus one (nine bits), Lock or
//! Unlock (another command sends the captured button again, without the held flag), the thirteen
//! fixed mask bits as captured. Not tried on a car.
//!
//! **Fields**: `data` = the first 64 bits, `extra` = the last 8, `data_count_bit` = 72,
//! `serial` = the ID, `button` = the button byte, `counter` = the nine counter bits.

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
    make: "Suzuki",
    category: Category::Vehicle,
    buttons: Some(button_name),
    presets: &[&AM650],
};

const NAME: &str = "Suzuki TS002";
/// One unit: 440 units of a frame take 112.975 ms in the recordings.
const TE: u32 = 257;
const TOL: u32 = 110;
/// The sync's carrier and its pause are four units each (993 and 1044 µs in a Flipper recording).
const SYNC_MIN: u32 = 800;
const SYNC_MAX: u32 = 1300;
const DATA_BITS: usize = 72;
/// One bits behind the data.
const FILL_BITS: usize = 72;
/// Frames in a press, and the 0 cells before the first.
const COPIES: usize = 4;
const LEAD_ZEROS: usize = 4;
const HELD: u8 = 0x20;

/// Code bit that holds `r0..r23`.
const R_POS: [u8; 24] = [25, 24, 21, 20, 3, 2, 1, 0, 19, 18, 15, 14, 13, 12, 11, 10, 7, 6, 5, 4, 29, 28, 27, 26];
/// Code bit that holds the inverse of counter bit 0..7.
const C_POS: [u8; 8] = [30, 31, 16, 17, 8, 9, 22, 23];
/// Mask index (`r` number) that holds the inverse of counter bit 0..8.
const C_MASK: [u8; 9] = [9, 8, 19, 18, 5, 4, 15, 14, 13];
/// Mask indices of the button byte's bit 1 and bit 0.
const B_MASK: [u8; 2] = [2, 3];
/// The thirteen mask bits no counter or button bit was seen in (`r0` is bit 23), and their value
/// in the recorded remote.
const FIXED_MASK: u32 = 0xC3_38CF;
const FIXED_SEEN: u32 = 0x00_20C4;

fn button_name(_name: &str, code: u8) -> Option<&'static str> {
    match code {
        0x01 => Some("Lock"),
        0x02 => Some("Unlock"),
        0x21 => Some("Lock (held)"),
        0x22 => Some("Unlock (held)"),
        _ => None,
    }
}

fn rotl24(value: u32, by: u32) -> u32 {
    let by = by % 24;
    ((value << by) | (value >> (24 - by))) & 0xFF_FFFF
}

/// The mask's counter and button bits (`r0` is bit 23).
fn mask(counter: u16, button: u8) -> u32 {
    let mut m = 0;
    for (i, &r) in C_MASK.iter().enumerate() {
        m |= u32::from(!(counter >> i) & 1) << (23 - r);
    }
    m |= u32::from((button >> 1) & 1) << (23 - B_MASK[0]);
    m | u32::from(button & 1) << (23 - B_MASK[1])
}

/// A frame's fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Fields {
    head: u8,
    button: u8,
    id: u32,
    /// Nine bits.
    counter: u16,
    /// The thirteen fixed mask bits ([FIXED_MASK]).
    fixed: u32,
}

impl Fields {
    /// The fields of 72 frame bits, when the counter and the button fit the mask.
    fn parse(frame: u128) -> Option<Self> {
        let code = frame as u32;
        let id = ((frame >> 32) & 0xFF_FFFF) as u32;
        if id == 0 || id == 0xFF_FFFF {
            return None;
        }
        let button = (frame >> 56) as u8;
        let low = C_POS.iter().enumerate().fold(0u16, |c, (i, &p)| c | (u16::from((!code >> p) & 1 == 1) << i));
        let r = R_POS.iter().fold(0u32, |r, &p| (r << 1) | ((code >> p) & 1));
        let m = r ^ rotl24(id, u32::from(low));
        let counter = low | (u16::from((!m >> (23 - C_MASK[8])) & 1 == 1) << 8);
        (m & !FIXED_MASK == mask(counter, button)).then_some(Self {
            head: (frame >> 64) as u8,
            button,
            id,
            counter,
            fixed: m & FIXED_MASK,
        })
    }

    /// The 72 frame bits.
    fn frame(&self) -> u128 {
        let r = rotl24(self.id, u32::from(self.counter & 0xFF)) ^ mask(self.counter, self.button) ^ (self.fixed & FIXED_MASK);
        let mut code = 0u32;
        for (k, &p) in R_POS.iter().enumerate() {
            code |= ((r >> (23 - k)) & 1) << p;
        }
        for (i, &p) in C_POS.iter().enumerate() {
            code |= u32::from(!(self.counter >> i) & 1) << p;
        }
        (u128::from(self.head) << 64) | (u128::from(self.button) << 56) | (u128::from(self.id & 0xFF_FFFF) << 32) | u128::from(code)
    }

    fn of(decoded: &DecodedSignal) -> Option<Self> {
        if decoded.data_count_bit != DATA_BITS {
            return None;
        }
        Self::parse((u128::from(decoded.data) << 8) | u128::from(decoded.extra.unwrap_or(0) & 0xFF))
    }

    fn signal(&self) -> DecodedSignal {
        let frame = self.frame();
        DecodedSignal {
            serial: Some(self.id),
            button: Some(self.button),
            counter: Some(self.counter),
            crc_valid: self.fixed == FIXED_SEEN,
            data: (frame >> 8) as u64,
            data_count_bit: DATA_BITS,
            encoder_capable: true,
            extra: Some((frame & 0xFF) as u64),
            protocol_display_name: None,
        }
    }

    /// The remote's next press: KAT's Lock and Unlock; any other command is the captured button
    /// again, without the held flag.
    fn next(&self, command: u8) -> Self {
        let button = match command {
            0x01 | 0x02 => command,
            _ => self.button & !HELD,
        };
        Self { button, counter: (self.counter + 1) & 0x1FF, ..*self }
    }
}

/// The pulses of a press: four 0 cells, the frame [COPIES] times, a unit of carrier and a pause.
fn upload(frame: u128) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(2 * (LEAD_ZEROS + COPIES * (1 + DATA_BITS + FILL_BITS)) + 2);
    let mut cell = |one: bool| {
        out.push(LevelDuration::new(true, if one { 2 * TE } else { TE }));
        out.push(LevelDuration::new(false, if one { TE } else { 2 * TE }));
    };
    for _ in 0..LEAD_ZEROS {
        cell(false);
    }
    let mut copies = Vec::new();
    for _ in 0..COPIES {
        copies.push(LevelDuration::new(true, 4 * TE));
        copies.push(LevelDuration::new(false, 4 * TE));
        let mut cell = |one: bool| {
            copies.push(LevelDuration::new(true, if one { 2 * TE } else { TE }));
            copies.push(LevelDuration::new(false, if one { TE } else { 2 * TE }));
        };
        for i in (0..DATA_BITS).rev() {
            cell((frame >> i) & 1 == 1);
        }
        for _ in 0..FILL_BITS {
            cell(true);
        }
    }
    out.extend(copies);
    out.push(LevelDuration::new(true, TE));
    out.push(LevelDuration::new(false, 10_000));
    out
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    /// Waiting for the sync's carrier.
    Idle,
    /// The sync's pause.
    SyncLow,
    /// A cell's carrier.
    High,
    /// A cell's pause.
    Low,
}

/// Suzuki TS002 decoder: a state machine over the pulses, one frame per sync.
pub struct SuzukiTs002Decoder {
    step: Step,
    /// The carrier of the cell being read.
    high: u32,
    /// The data bits read.
    frame: u128,
    /// Cells read behind the sync.
    count: usize,
}

fn is_short(d: u32) -> bool {
    d.abs_diff(TE) <= TOL
}

fn is_long(d: u32) -> bool {
    d.abs_diff(2 * TE) <= TOL
}

fn is_sync(d: u32) -> bool {
    (SYNC_MIN..=SYNC_MAX).contains(&d)
}

impl SuzukiTs002Decoder {
    pub fn new() -> Self {
        Self { step: Step::Idle, high: 0, frame: 0, count: 0 }
    }

    /// A pulse that fits nowhere: start over, with this pulse as a sync's carrier when it is one.
    fn restart(&mut self, level: bool, duration: u32) {
        self.step = if level && is_sync(duration) { Step::SyncLow } else { Step::Idle };
    }
}

impl Default for SuzukiTs002Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for SuzukiTs002Decoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: TE, te_long: 2 * TE, te_delta: TOL, min_count_bit: DATA_BITS }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        *self = Self::new();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Idle => self.restart(level, duration),
            Step::SyncLow => {
                if !level && is_sync(duration) {
                    (self.step, self.frame, self.count) = (Step::High, 0, 0);
                } else {
                    self.restart(level, duration);
                }
            }
            Step::High => {
                if level && (is_short(duration) || is_long(duration)) {
                    (self.step, self.high) = (Step::Low, duration);
                } else {
                    self.restart(level, duration);
                }
            }
            Step::Low => {
                let bit = match (level, is_long(self.high), is_short(duration), is_long(duration)) {
                    (false, true, true, _) => true,
                    (false, false, _, true) => false,
                    _ => {
                        self.restart(level, duration);
                        return None;
                    }
                };
                if self.count < DATA_BITS {
                    self.frame = (self.frame << 1) | u128::from(bit);
                } else if !bit {
                    self.step = Step::Idle;
                    return None;
                }
                self.count += 1;
                self.step = Step::High;
                if self.count == DATA_BITS + FILL_BITS {
                    self.step = Step::Idle;
                    return Fields::parse(self.frame).map(|f| f.signal());
                }
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        Some(upload(Fields::of(decoded)?.next(button).frame()))
    }
}

#[cfg(test)]
mod tests {
    use super::super::f_common::test_support::*;
    use super::*;

    /// Presses 0x106 (Lock) and 0x09D (Unlock) of the recorded remote.
    const LOCK: u128 = 0x14_01AD_E998_7DE2_17C6;
    const UNLOCK: u128 = 0x14_02AD_E998_A458_C27B;

    fn fields(frame: u128) -> Fields {
        Fields::parse(frame).unwrap()
    }

    #[test]
    fn frame_fields() {
        let f = fields(LOCK);
        assert_eq!((f.head, f.button, f.id, f.counter, f.fixed), (0x14, 0x01, 0xADE998, 0x106, FIXED_SEEN));
        assert_eq!(f.frame(), LOCK);
        let f = fields(UNLOCK);
        assert_eq!((f.button, f.id, f.counter, f.fixed), (0x02, 0xADE998, 0x09D, FIXED_SEEN));
        assert_eq!(f.frame(), UNLOCK);
        let d = f.signal();
        assert_eq!((d.serial, d.button, d.counter, d.crc_valid), (Some(0xADE998), Some(2), Some(0x09D), true));
        assert_eq!((d.data, d.extra, d.data_count_bit), (0x1402_ADE9_98A4_58C2, Some(0x7B), 72));
        assert_eq!(Fields::of(&d), Some(f));
        // every counter and both buttons give a frame that reads back
        for counter in 0..0x200 {
            for button in [0x01, 0x02, 0x21, 0x22] {
                let f = Fields { head: 0x14, button, id: 0x5A_C3F1, counter, fixed: FIXED_SEEN };
                assert_eq!(Fields::parse(f.frame()), Some(f), "{counter:X} {button:X}");
            }
        }
        assert_eq!(INFO.buttons.unwrap()(NAME, 0x22), Some("Unlock (held)"));
    }

    #[test]
    fn the_gate() {
        // one wrong bit in the code, the ID or the button's low bits: the mask does not fit. But for
        // the ninth counter bit (code bit 12) and the ID bit that lies under it at this rotation: the
        // frame is then the same remote's with another ninth bit, or another remote's.
        for bit in (0..58).filter(|&bit| bit != 12 && bit != 36) {
            assert_eq!(Fields::parse(LOCK ^ (1 << bit)).filter(|f| f.fixed == FIXED_SEEN), None, "bit {bit}");
        }
        // the ten bits that repeat the counter byte and the button are the gate itself
        for bit in [30, 31, 16, 17, 8, 9, 22, 23, 18, 19, 4, 5, 2, 3, 10, 11, 20, 21] {
            assert_eq!(Fields::parse(LOCK ^ (1 << bit)), None, "bit {bit}");
        }
        // the ninth counter bit and the thirteen fixed bits are read, not checked
        assert_eq!(fields(LOCK ^ (1 << 12)).counter, 0x006);
        assert_eq!((fields(LOCK ^ (1 << 36)).counter, fields(LOCK ^ (1 << 36)).id), (0x006, 0xADE988));
        let other = fields(LOCK ^ (1 << 29));
        assert_eq!(other.fixed, FIXED_SEEN ^ 0x8);
        assert!(!other.signal().crc_valid);
        // the head byte and the held flag are free
        assert_eq!(fields(LOCK ^ (0x20 << 56)).button, 0x21);
        assert_eq!(fields(LOCK ^ (0xFF << 64)).head, 0xEB);
    }

    #[test]
    fn decodes_what_it_sends() {
        let wire = upload(LOCK);
        assert_eq!(wire.len(), 2 * (4 + 4 * 145) + 2);
        assert_eq!(wire.iter().map(|p| u64::from(p.duration_us)).sum::<u64>(), (12 + 4 * 440 + 1) * 257 + 10_000);
        let got = feed_all(&mut SuzukiTs002Decoder::new(), &wire);
        assert_eq!(got.len(), 4);
        assert!(got.iter().all(|d| *d == fields(LOCK).signal()));
        assert_eq!(feed_resetting(&mut SuzukiTs002Decoder::new(), &wire).len(), 4);
        // as a Flipper records it: carrier 20 µs short, pauses 20 µs long
        let skewed: Vec<_> = wire.iter().map(|p| LevelDuration::new(p.level, if p.level { p.duration_us - 20 } else { p.duration_us + 20 })).collect();
        assert_eq!(feed_all(&mut SuzukiTs002Decoder::new(), &skewed).len(), 4);
        // a zero among the one bits, a cell too long, a frame cut short
        let mut bad = wire.clone();
        (bad[8 + 2 + 2 * 100].duration_us, bad[8 + 2 + 2 * 100 + 1].duration_us) = (TE, 2 * TE);
        assert_eq!(feed_all(&mut SuzukiTs002Decoder::new(), &bad).len(), 3);
        let mut bad = wire.clone();
        bad[8 + 2 + 2 * 10].duration_us = 3 * TE;
        assert_eq!(feed_all(&mut SuzukiTs002Decoder::new(), &bad).len(), 3);
        assert!(feed_all(&mut SuzukiTs002Decoder::new(), &wire[..8 + 2 + 2 * 143]).is_empty());
    }

    #[test]
    fn sends_the_next_press() {
        let dec = SuzukiTs002Decoder::new();
        let captured = fields(LOCK).signal();
        for (command, button) in [(0x01, 0x01), (0x02, 0x02), (0x04, 0x01), (0x08, 0x01)] {
            let got = feed_all(&mut SuzukiTs002Decoder::new(), &dec.encode(&captured, command).unwrap());
            assert_eq!(got.len(), 4, "{command}");
            assert_eq!((got[0].button, got[0].counter, got[0].serial), (Some(button), Some(0x107), Some(0xADE998)));
            assert!(got[0].crc_valid);
        }
        // the remote's own next press, bit for bit (press 0x107 of the recording)
        assert_eq!(fields(LOCK).next(0x01).frame(), 0x14_01AD_E998_0BEE_BFB8);
        // a held frame sends the plain button; the counter wraps in nine bits; the fixed bits stay
        let held = Fields { button: 0x22, counter: 0x1FF, fixed: 0x8, ..fields(LOCK) };
        let next = held.next(0x08);
        assert_eq!((next.button, next.counter, next.fixed), (0x02, 0, 0x8));
        assert!(dec.encode(&DecodedSignal::new(0, 64), 1).is_none());
        assert!(dec.encode(&DecodedSignal::new(0, 72), 1).is_none(), "an ID of zeros");
        assert_eq!(Fields::parse(0x14_01FF_FFFF_0000_0000), None);
    }

    /// The recordings: every press is read, four frames each, and the counter goes up by one.
    #[test]
    fn imports_recordings() {
        let dir = std::path::Path::new("IMPORTS/SUZUKI");
        let file = dir.join("suzuki_ts002_unlock_lock_x10.sub");
        if !file.is_file() {
            eprintln!("Skip: IMPORTS not found (run from crate root)");
            return;
        }
        let (_, wire) = sub_pulses(&file);
        let got = feed_resetting(&mut SuzukiTs002Decoder::new(), &wire);
        assert_eq!(got.len(), 40);
        assert!(got.iter().all(|d| d.serial == Some(0xADE998) && d.crc_valid));
        let presses: Vec<_> = got.iter().step_by(4).map(|d| (d.counter.unwrap(), d.button.unwrap())).collect();
        let want: Vec<_> = (0..10).map(|i| (0x16F + i, if i % 2 == 0 { 2 } else { 1 })).collect();
        assert_eq!(presses, want);
        // each press is what the encoder makes of the one before
        for pair in got.chunks(4).collect::<Vec<_>>().windows(2) {
            let next = Fields::of(&pair[0][0]).unwrap().next(pair[1][0].button.unwrap());
            assert_eq!(next.signal(), pair[1][0]);
        }
    }
}
