//! Isuzu Static: the remote of an Isuzu FTS truck: a fixed 64-bit Manchester frame, sent for as
//! long as the button is held — **decode and display only**.
//!
//! **KAT's own decoder**, worked out from two recordings of one remote
//! (`IMPORTS/Isuzu/Isuzu_fts_lock.sub` and `Isuzu_fts_unlock.sub`, 433.92 MHz, Flipper preset
//! AM650, two presses each). No reference reads the signal and no FCC filing has been matched to
//! it; the name is KAT's.
//!
//! **Air format**: on-off keying, Manchester (carrier then pause is a 1), 523.5 µs per half bit
//! (955 bit/s). A frame is 64 bits and two 1 bits; 69.1 ms. It is sent every 70.2 ms while the
//! button is held (9 and 19 whole copies in the presses recorded) and stops in the middle of a
//! copy.
//!
//! **Frame (the 64 bits)**:
//!
//! | Byte | Lock | Unlock | Notes |
//! |---|---|---|---|
//! | 0..3 | `01 10 01 83` | `01 10 01 83` | the same in all four presses (the fob's ID, or part of the protocol: one remote cannot tell) |
//! | 4 | `25`, `27` | `41`, `43` | high nibble: the button; bit 1 is 0 in a recording's first press and 1 in its second |
//! | 5..7 | `0E 81 FF` | `0E 81 FF` | the same in all four presses |
//!
//! Nothing else changes from press to press: the code does not roll.
//!
//! **Gate** (no checksum is known, and there is no sync word): a burst that is exactly 64 bits
//! and two 1 bits without one Manchester violation; and the same frame in the burst before, less
//! than 200 ms earlier. So the first copy of a press is not listed. (KAT starts every decoder
//! afresh after a frame, so nine copies are listed four times.)
//!
//! **Fields**: `data` = the 64 bits, `button` = byte 4's high nibble, `data_count_bit` = 64, no
//! serial, no counter.

use super::burst::{self, BurstReader};
use super::info::prelude::*;
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Isuzu",
    category: Category::Vehicle,
    buttons: Some(button_name),
    presets: &[&AM650],
};

const NAME: &str = "Isuzu Static";
const TE: u32 = 523;
const TOL: u32 = 120;
const DATA_BITS: usize = 64;
/// The data and the two 1 bits behind it.
const FRAME_BITS: usize = DATA_BITS + 2;
/// The copies of a press start 70 ms apart; a copy counts as one when the frame before it is this near.
const COPY_WITHIN_US: u64 = 200_000;

fn button_name(_name: &str, code: u8) -> Option<&'static str> {
    match code {
        0x2 => Some("Lock"),
        0x4 => Some("Unlock"),
        _ => None,
    }
}

/// Isuzu Static decoder: reads a burst when it is over and reports a frame it has seen just before.
pub struct IsuzuStaticDecoder {
    reader: BurstReader,
    /// Microseconds fed so far.
    clock: u64,
    /// The last frame read and when its burst ended.
    last: Option<(u64, u64)>,
}

impl IsuzuStaticDecoder {
    pub fn new() -> Self {
        Self { reader: BurstReader::new(TE, TOL, 512), clock: 0, last: None }
    }

    fn burst(&mut self, halves: &[bool]) -> Option<DecodedSignal> {
        let data = read(halves)?;
        let copy = self.last.is_some_and(|(d, t)| d == data && self.clock - t < COPY_WITHIN_US);
        self.last = Some((data, self.clock));
        copy.then(|| signal(data))
    }
}

impl Default for IsuzuStaticDecoder {
    fn default() -> Self {
        Self::new()
    }
}

fn signal(data: u64) -> DecodedSignal {
    DecodedSignal {
        serial: None,
        button: Some(((data >> 28) & 0xF) as u8),
        counter: None,
        crc_valid: false,
        data,
        data_count_bit: DATA_BITS,
        encoder_capable: false,
        extra: None,
        protocol_display_name: None,
    }
}

/// The 64 bits of a finished burst, if it is one frame. On-off keying has one polarity.
fn read(halves: &[bool]) -> Option<u64> {
    for phase in 0..2 {
        let bits = burst::bits(halves, phase, false);
        if bits.len() != FRAME_BITS || burst::spare(halves.len(), phase, FRAME_BITS) > 1 {
            continue;
        }
        if let Some(frame) = burst::take(&bits, 0, FRAME_BITS) {
            if frame[DATA_BITS] && frame[DATA_BITS + 1] {
                return Some(burst::number(&frame[..DATA_BITS]));
            }
        }
    }
    None
}

impl ProtocolDecoder for IsuzuStaticDecoder {
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

    const LOCK: u64 = 0x0110_0183_250E_81FF;

    fn frame_bits(data: u64) -> Vec<bool> {
        let mut b = burst::pattern(data, 64);
        b.extend([true, true]);
        b
    }

    /// `n` frames of `bits`, 2.1 ms of pause between them.
    fn bursts(bits: &[bool], n: usize) -> Vec<LevelDuration> {
        let mut wire = vec![LevelDuration::new(false, 30_000)];
        for _ in 0..n {
            wire.extend(burst::wire(bits, TE, 2_100));
        }
        burst::merged(wire)
    }

    fn copies(data: u64, n: usize) -> Vec<LevelDuration> {
        bursts(&frame_bits(data), n)
    }

    #[test]
    fn decodes_the_copies_behind_the_first() {
        let got = feed_all(&mut IsuzuStaticDecoder::new(), &copies(LOCK, 9));
        assert_eq!(got.len(), 8, "{got:?}");
        let d = &got[0];
        assert_eq!((d.data, d.button, d.serial, d.counter), (LOCK, Some(2), None, None));
        assert_eq!((d.extra, d.data_count_bit), (None, 64));
        assert!(!d.encoder_capable && !d.crc_valid);
        assert_eq!(feed_resetting(&mut IsuzuStaticDecoder::new(), &copies(LOCK, 9)).len(), 4);
        let got = feed_all(&mut IsuzuStaticDecoder::new(), &copies(0x0110_0183_430E_81FF, 2));
        assert_eq!(got.iter().map(|d| d.button).collect::<Vec<_>>(), [Some(4)]);
        assert_eq!(INFO.buttons.unwrap()(NAME, 4), Some("Unlock"));
    }

    #[test]
    fn the_gate() {
        assert!(feed_all(&mut IsuzuStaticDecoder::new(), &copies(LOCK, 1)).is_empty(), "one copy is not enough");
        let mut wire = copies(LOCK, 1);
        wire.extend(copies(LOCK ^ 0x0200_0000, 1));
        assert!(feed_all(&mut IsuzuStaticDecoder::new(), &wire).is_empty(), "two frames that differ");
        let mut wire = copies(LOCK, 1);
        wire.push(LevelDuration::new(false, 400_000));
        wire.extend(copies(LOCK, 1));
        assert!(feed_all(&mut IsuzuStaticDecoder::new(), &wire).is_empty(), "the same frame, much later");
        // other bits where the two ones stand, a bit short, a bit long
        for bits in [
            [burst::pattern(LOCK, 64), vec![false, true]].concat(),
            [burst::pattern(LOCK, 64), vec![true, false]].concat(),
            [burst::pattern(LOCK, 63), vec![true, true]].concat(),
            [burst::pattern(LOCK, 64), vec![true, true, true]].concat(),
        ] {
            assert!(feed_all(&mut IsuzuStaticDecoder::new(), &bursts(&bits, 4)).is_empty(), "{} bits", bits.len());
        }
        // a frame may begin with a 1 bit
        let got = feed_all(&mut IsuzuStaticDecoder::new(), &copies(LOCK | 1 << 63, 3));
        assert_eq!(got.iter().map(|d| d.data).collect::<Vec<_>>(), [LOCK | 1 << 63; 2]);
        assert!(IsuzuStaticDecoder::new().encode(&DecodedSignal::new(0, 64), 2).is_none());
    }

    /// The recordings: two presses each; byte 4 is 25 then 27 (lock), 41 then 43 (unlock).
    #[test]
    fn imports_recordings() {
        for (file, frames, codes) in [
            ("Isuzu_fts_lock.sub", 8, [0x0110_0183_250E_81FF_u64, 0x0110_0183_270E_81FF]),
            ("Isuzu_fts_unlock.sub", 18, [0x0110_0183_410E_81FF, 0x0110_0183_430E_81FF]),
        ] {
            let path = std::path::Path::new("IMPORTS/Isuzu").join(file);
            if !path.is_file() {
                eprintln!("Skip: IMPORTS not found (run from crate root)");
                return;
            }
            let (_, wire) = sub_pulses(&path);
            let got = feed_resetting(&mut IsuzuStaticDecoder::new(), &wire);
            let mut seen: Vec<u64> = got.iter().map(|d| d.data).collect();
            seen.dedup();
            eprintln!("ISUZU {file}: {} frames, codes {seen:X?}", got.len());
            assert_eq!((got.len(), seen.as_slice()), (frames, &codes[..]), "{file}");
        }
    }
}
