//! Isuzu Rolling: an Isuzu remote that sends a 105-bit differential Manchester frame, eight
//! copies at a time — **decode and display only**.
//!
//! **KAT's own decoder**, worked out from two recordings of one remote (`IMPORTS/Isuzu/Isuzu_Lock.sub`
//! and `Isuzu_Unlock.sub`, 433.889 MHz, Flipper preset AM650, three presses each). No reference
//! reads the signal, the model and year are not known and no FCC filing has been matched to it;
//! the name is KAT's.
//!
//! **Air format**: on-off keying, differential Manchester, 405.6 µs per half cell (1233 bit/s):
//! the level changes between all cells, and once more in the middle of a cell for a 1. A frame is
//! 105 cells and one more half cell of carrier (so the frame's last carrier pulse is one, two or
//! three half cells long); 85.6 ms. The copies of a frame are 105.3 ms apart.
//!
//! **A press** (the same in all six): a frame with the button, eight times; while the button is
//! held another such frame (another code), until the press ends in the middle of a copy; then 39
//! one bits and, 1.2 ms behind them, a frame with no button, eight times.
//!
//! **Frame (105 bits)**:
//!
//! | Bits | Field | Notes |
//! |---|---|---|
//! | 0..63 | Code | another one in every frame |
//! | 64..91 | Fob | 28 bits, the same in all 15 frames (`4A0D2EC`) |
//! | 92..95 | Button | `4` in the lock recording, `2` in the unlock recording, `0` in the frame that ends a press |
//! | 96..100 | `00000` | in every frame |
//! | 101..104 | ? | changes with the frame; no CRC-4, sum or XOR of the bits before it |
//!
//! Which way round the bits are is KAT's choice: read the other way the fob ID and the code are
//! complemented, the buttons are `B`, `D` and `F` and the lead is zeros. This way the buttons are
//! single bits and the lead is ones.
//!
//! **Recording it.** Both recordings have carrier pulses 40 to 50 µs short and pauses as much too
//! long; the decoder takes each burst's skew out before it counts half cells (see `burst.rs`).
//!
//! **Gate** (no checksum is known, and there is no sync word): the burst ends with 105 cells and
//! the half cell of carrier, with a transition between all cells; in front of them is the pause
//! before the burst or one of three half cells (as behind the ones that lead the last frame of a
//! press); the burst comes out at 406 µs per half cell to within an eighth; and the same frame in
//! the burst before, less than 300 ms earlier. So the first copy of a frame is
//! not listed. (KAT starts every decoder afresh after a frame, so eight copies are listed four
//! times.)
//!
//! **Fields**: `data` = the 64-bit code, `serial` = the 28 fob bits, `button` = the nibble,
//! `extra` = frame bits 96..104, `data_count_bit` = 105, no counter.

use super::burst::{self, BurstReader};
use super::info::prelude::*;
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::DifferentialManchester,
    rf: Rf::AM,
    encryption: "Rolling code (cipher unknown)",
    make: "Isuzu",
    category: Category::Vehicle,
    buttons: Some(button_name),
    presets: &[&AM650],
};

const NAME: &str = "Isuzu Rolling";
const TE: u32 = 406;
const FRAME_BITS: usize = 105;
/// The copies of a frame start 105 ms apart; a copy counts as one when the frame before it is this near.
const COPY_WITHIN_US: u64 = 300_000;

fn button_name(_name: &str, code: u8) -> Option<&'static str> {
    match code {
        0x4 => Some("Lock"),
        0x2 => Some("Unlock"),
        0x0 => Some("Release"),
        _ => None,
    }
}

/// Isuzu Rolling decoder: reads a burst when it is over and reports a frame it has seen just before.
pub struct IsuzuRollingDecoder {
    reader: BurstReader,
    /// Microseconds fed so far.
    clock: u64,
    /// The last frame read and when its burst ended.
    last: Option<(Vec<bool>, u64)>,
}

impl IsuzuRollingDecoder {
    pub fn new() -> Self {
        Self { reader: BurstReader::skewed(TE, 3, 1024), clock: 0, last: None }
    }

    fn burst(&mut self, halves: &[bool]) -> Option<DecodedSignal> {
        let frame = read(halves)?;
        let copy = self.last.as_ref().is_some_and(|(f, t)| *f == frame && self.clock - t < COPY_WITHIN_US);
        let out = copy.then(|| signal(&frame));
        self.last = Some((frame, self.clock));
        out
    }
}

impl Default for IsuzuRollingDecoder {
    fn default() -> Self {
        Self::new()
    }
}

fn signal(frame: &[bool]) -> DecodedSignal {
    DecodedSignal {
        serial: Some(burst::number(&frame[64..92]) as u32),
        button: Some(burst::number(&frame[92..96]) as u8),
        counter: None,
        crc_valid: false,
        data: burst::number(&frame[..64]),
        data_count_bit: FRAME_BITS,
        encoder_capable: false,
        extra: Some(burst::number(&frame[96..])),
        protocol_display_name: None,
    }
}

/// The 105 bits a finished burst ends with, if it ends with a frame.
fn read(halves: &[bool]) -> Option<Vec<bool>> {
    // the frame's last half is carrier: the burst's last half, or the one before the pause that ended it
    for end in [halves.len(), halves.len().saturating_sub(1)] {
        let Some(start) = end.checked_sub(2 * FRAME_BITS + 1) else { continue };
        // in front of the frame: the pause before the burst, or three half cells of pause
        let paused = match start {
            1 => !halves[0],
            _ => start >= 3 && halves[start - 3..start].iter().all(|h| !h),
        };
        if !halves[end - 1] || !halves[start] || !paused {
            continue;
        }
        if let Some(frame) = burst::take(&burst::cells(&halves[start..end - 1]), 0, FRAME_BITS) {
            return Some(frame);
        }
    }
    None
}

impl ProtocolDecoder for IsuzuRollingDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: TE, te_long: 2 * TE, te_delta: TE / 2, min_count_bit: FRAME_BITS }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.reader.reset();
        self.clock = 0;
        self.last = None;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let ended = self.reader.feed(level, duration);
        let out = ended.and_then(|halves| self.burst(&halves));
        self.clock += u64::from(duration);
        out
    }

    fn flush(&mut self) -> Option<DecodedSignal> {
        let halves = self.reader.flush()?;
        self.burst(&halves)
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::super::f_common::test_support::*;
    use super::*;

    const CODE: u64 = 0x6A35_02F4_6918_6AD7;
    const FOB: u64 = 0x4A0_D2EC;

    fn frame_bits(code: u64, button: u64, tail: u64) -> Vec<bool> {
        let mut b = burst::pattern(code, 64);
        b.extend(burst::pattern(FOB, 28));
        b.extend(burst::pattern(button, 4));
        b.extend(burst::pattern(tail, 9));
        b
    }

    /// One frame on the air: a transition between all cells, one in a cell for a 1, a half cell of
    /// carrier behind the last, then `gap` of pause. Starts with carrier.
    fn frame_wire(bits: &[bool], gap: u32) -> Vec<LevelDuration> {
        let mut halves = Vec::new();
        let mut level = true;
        for &b in bits {
            halves.push(level);
            if b {
                level = !level;
            }
            halves.push(level);
            level = !level;
        }
        halves.push(true);
        let mut wire: Vec<LevelDuration> = Vec::new();
        for h in halves {
            match wire.last_mut() {
                Some(p) if p.level == h => p.duration_us += TE,
                _ => wire.push(LevelDuration::new(h, TE)),
            }
        }
        wire.push(LevelDuration::new(false, gap));
        wire
    }

    fn copies(bits: &[bool], n: usize) -> Vec<LevelDuration> {
        (0..n).flat_map(|_| frame_wire(bits, 9_200)).collect()
    }

    #[test]
    fn decodes_the_copies_behind_the_first() {
        // tails 1, 0 and 2 end the frame with one, two and three half cells of carrier
        for tail in [0x001, 0x000, 0x002, 0x00F] {
            let got = feed_all(&mut IsuzuRollingDecoder::new(), &copies(&frame_bits(CODE, 4, tail), 8));
            assert_eq!(got.len(), 7, "tail {tail:X}: {got:?}");
            let d = &got[0];
            assert_eq!((d.data, d.serial, d.button, d.counter), (CODE, Some(0x4A0_D2EC), Some(4), None));
            assert_eq!((d.extra, d.data_count_bit), (Some(tail), 105));
            assert!(!d.encoder_capable && !d.crc_valid);
        }
        assert_eq!(feed_resetting(&mut IsuzuRollingDecoder::new(), &copies(&frame_bits(CODE, 2, 9), 8)).len(), 4);
        // carrier pulses 45 µs short, pauses 45 µs long, as in the recordings
        let mut wire = copies(&frame_bits(CODE, 0, 0x00A), 3);
        for p in &mut wire {
            if p.duration_us < 5_000 {
                p.duration_us = if p.level { p.duration_us - 45 } else { p.duration_us + 45 };
            }
        }
        let got = feed_all(&mut IsuzuRollingDecoder::new(), &wire);
        assert_eq!(got.iter().map(|d| (d.data, d.button, d.extra)).collect::<Vec<_>>(), [(CODE, Some(0), Some(0x00A)); 2]);
        assert_eq!(INFO.buttons.unwrap()(NAME, 4), Some("Lock"));
        assert_eq!(INFO.buttons.unwrap()(NAME, 0), Some("Release"));
    }

    #[test]
    fn the_gate() {
        let bits = frame_bits(CODE, 4, 0x003);
        assert!(feed_all(&mut IsuzuRollingDecoder::new(), &copies(&bits, 1)).is_empty(), "one copy is not enough");
        // two frames that differ
        let mut wire = copies(&bits, 1);
        wire.extend(copies(&frame_bits(CODE ^ 0x10, 4, 0x003), 1));
        assert!(feed_all(&mut IsuzuRollingDecoder::new(), &wire).is_empty());
        // the same frame half a second later
        let mut wire = frame_wire(&bits, 500_000);
        wire.extend(copies(&bits, 1));
        assert!(feed_all(&mut IsuzuRollingDecoder::new(), &wire).is_empty());
        // a bit short, a bit long
        for n in [104, 106] {
            let mut other = bits.clone();
            other.resize(n, true);
            assert!(feed_all(&mut IsuzuRollingDecoder::new(), &copies(&other, 4)).is_empty(), "{n} bits");
        }
        // a long run of short pulses (another remote's lead) is no frame: not at 250 µs, and not at
        // 406 µs either, where 105 cells of it have no pause in front
        for te in [250, TE] {
            let lead: Vec<LevelDuration> = (0..300).map(|i| LevelDuration::new(i % 2 == 0, te)).collect();
            let mut wire = Vec::new();
            for _ in 0..4 {
                wire.extend(lead.iter().copied());
                wire.push(LevelDuration::new(false, 9_200));
            }
            assert!(feed_all(&mut IsuzuRollingDecoder::new(), &wire).is_empty(), "{te} µs");
        }
        // the frame behind 39 ones and three half cells of pause, as at the end of a press
        let mut wire = Vec::new();
        for _ in 0..2 {
            wire.extend((0..78).map(|i| LevelDuration::new(i % 2 == 0, TE)));
            wire.push(LevelDuration::new(false, 3 * TE));
            wire.extend(frame_wire(&bits, 9_200));
        }
        let wire = burst::merged(wire);
        assert_eq!(feed_all(&mut IsuzuRollingDecoder::new(), &wire).len(), 1);
        // a pulse of three half cells inside the frame
        let mut wire = copies(&bits, 4);
        for i in [10, 10 + wire.len() / 4, 10 + wire.len() / 2, 10 + 3 * wire.len() / 4] {
            wire[i].duration_us = 3 * TE;
        }
        assert!(feed_all(&mut IsuzuRollingDecoder::new(), &wire).is_empty());
        assert!(IsuzuRollingDecoder::new().encode(&DecodedSignal::new(0, 105), 4).is_none());
    }

    /// The recordings: one fob; the button's frames, then the frame without a button.
    #[test]
    fn imports_recordings() {
        for (file, button, frames) in [("Isuzu_Lock.sub", 4, 26), ("Isuzu_Unlock.sub", 2, 25)] {
            let path = std::path::Path::new("IMPORTS/Isuzu").join(file);
            if !path.is_file() {
                eprintln!("Skip: IMPORTS not found (run from crate root)");
                return;
            }
            let (_, wire) = sub_pulses(&path);
            let got = feed_resetting(&mut IsuzuRollingDecoder::new(), &wire);
            let mut seen: Vec<(u8, u64)> = got.iter().map(|d| (d.button.unwrap(), d.data)).collect();
            seen.dedup();
            let buttons: Vec<u8> = seen.iter().map(|s| s.0).collect();
            eprintln!("ISUZU {file}: {} frames, {} distinct, buttons {buttons:?}", got.len(), seen.len());
            assert!(got.iter().all(|d| d.serial == Some(0x4A0_D2EC) && d.extra.unwrap() >> 4 == 0), "{file}");
            assert!(buttons.iter().all(|b| *b == button || *b == 0) && buttons.contains(&button) && buttons.contains(&0), "{file}");
            assert_eq!(got.len(), frames, "{file}");
        }
    }
}
