//! Kia Forte: a 315 MHz Kia remote that sends an 88-bit Manchester frame four times a press —
//! **decode and display only**.
//!
//! **KAT's own decoder**, worked out from one recording sent in by the user on 2026-11-16
//! (`IMPORTS/KIA/more/Forte_315mhz_FM.sub`, one fob, ten presses, two buttons). No reference reads
//! the signal and no FCC filing has been matched to it; the name is the recording's.
//!
//! **Air format**: 2-FSK, Manchester, 489.8 µs per half bit (1021 bit/s). A press is four bursts:
//! a run of zero bits and the frame, then the frame alone three more times, one every 90.1 ms
//! (86.7 ms of frame, about 3.4 ms between). The four copies of a press are alike.
//!
//! **Frame (88 bits, 11 bytes)**:
//!
//! | Byte | Field | Notes |
//! |---|---|---|
//! | 0 | `01` | in every frame |
//! | 1 | Button | `01` in the first five presses, `02` in the last five |
//! | 2..4 | Fob | `99 0E CD` in every press of the one fob (an ID, or part of the protocol: one fob cannot tell) |
//! | 5..9 | Code | changes with every press |
//! | 10 | | high nibble changes, low nibble `D` in every press |
//!
//! **Gate** (no checksum is known, and there is no sync word): a burst that begins with zero bits,
//! at least the seven of byte 0, has its first one bit as the last bit of byte 0 and ends with the
//! 88th bit (at most 8 half bits behind it, an FM receiver's noise), without one Manchester
//! violation; and the same frame in the burst before it, less than 300 ms earlier. So the first
//! copy of a press is not listed, and a press of which one copy was caught is not read. (KAT
//! starts every decoder afresh after a frame, so a press of four copies is listed twice.) Either
//! polarity of the two tones is read: the one in which the burst begins with zeros.
//!
//! **Fields**: `button` = byte 1, `serial` = bytes 2..4, `data` = bytes 3..10, `extra` = bytes
//! 0..2, `data_count_bit` = 88, no counter.

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
    rf: Rf::FM,
    encryption: "Rolling code (cipher unknown)",
    make: "Kia",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&FM476],
};

const NAME: &str = "Kia Forte";
const TE: u32 = 490;
const TOL: u32 = 150;
const FRAME_BITS: usize = 88;
const MAX_SPARE: usize = 8;
/// The copies of a press start 90 ms apart; a copy counts as one when the frame before it is this near.
const COPY_WITHIN_US: u64 = 300_000;

/// Kia Forte decoder: reads a burst when it is over and reports a frame it has seen just before.
pub struct KiaForteDecoder {
    reader: BurstReader,
    /// Microseconds fed so far.
    clock: u64,
    /// The last frame read and when its burst ended.
    last: Option<(Vec<bool>, u64)>,
}

impl KiaForteDecoder {
    pub fn new() -> Self {
        Self { reader: BurstReader::new(TE, TOL, 1024), clock: 0, last: None }
    }

    fn burst(&mut self, halves: &[bool]) -> Option<DecodedSignal> {
        let frame = read(halves)?;
        let copy = self.last.as_ref().is_some_and(|(f, t)| *f == frame && self.clock - t < COPY_WITHIN_US);
        let out = copy.then(|| signal(&frame));
        self.last = Some((frame, self.clock));
        out
    }
}

impl Default for KiaForteDecoder {
    fn default() -> Self {
        Self::new()
    }
}

fn signal(frame: &[bool]) -> DecodedSignal {
    DecodedSignal {
        serial: Some(burst::number(&frame[16..40]) as u32),
        button: Some(burst::number(&frame[8..16]) as u8),
        counter: None,
        crc_valid: false,
        data: burst::number(&frame[24..]),
        data_count_bit: FRAME_BITS,
        encoder_capable: false,
        extra: Some(burst::number(&frame[..24])),
        protocol_display_name: None,
    }
}

/// The 88 bits of a finished burst, if it is a frame (alone, or behind zero bits).
fn read(halves: &[bool]) -> Option<Vec<bool>> {
    for phase in 0..2 {
        for inverted in [false, true] {
            let bits = burst::bits(halves, phase, inverted);
            // the first bit that is not a zero is the last bit of byte 0
            let Some(one) = bits.iter().position(|b| *b != Some(false)) else { continue };
            if one < 7 || bits[one] != Some(true) {
                continue;
            }
            let at = one - 7;
            if burst::spare(halves.len(), phase, at + FRAME_BITS) > MAX_SPARE {
                continue;
            }
            if let Some(frame) = burst::take(&bits, at, FRAME_BITS) {
                return Some(frame);
            }
        }
    }
    None
}

impl ProtocolDecoder for KiaForteDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: TE, te_long: 2 * TE, te_delta: TOL, min_count_bit: FRAME_BITS }
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

    const FIRST: [u8; 11] = [0x01, 0x01, 0x99, 0x0E, 0xCD, 0x69, 0xB5, 0xE4, 0x6B, 0x08, 0xAD];

    fn frame(bytes: [u8; 11]) -> Vec<bool> {
        bytes.iter().flat_map(|&b| burst::pattern(u64::from(b), 8)).collect()
    }

    /// A press: `zeros` zero bits and the frame, then the frame `copies - 1` more times.
    fn press(zeros: usize, bytes: [u8; 11], copies: usize) -> Vec<LevelDuration> {
        let mut first = vec![false; zeros];
        first.extend(frame(bytes));
        let mut wire = burst::wire(&first, TE, 3_400);
        for _ in 1..copies {
            wire.extend(burst::wire(&frame(bytes), TE, 3_400));
        }
        wire.push(LevelDuration::new(false, 600_000));
        burst::merged(wire)
    }

    #[test]
    fn decodes_the_copies_behind_the_first() {
        let got = feed_all(&mut KiaForteDecoder::new(), &press(59, FIRST, 4));
        assert_eq!(got.len(), 3, "{got:?}");
        let d = &got[0];
        assert_eq!((d.button, d.serial, d.counter), (Some(1), Some(0x99_0ECD), None));
        assert_eq!((d.extra, d.data, d.data_count_bit), (Some(0x01_0199), 0x0ECD_69B5_E46B_08AD, 88));
        assert!(got.iter().all(|x| x == d));
        // the second button, and no lead in front of the first copy
        let mut second = FIRST;
        second[1] = 0x02;
        let got = feed_all(&mut KiaForteDecoder::new(), &press(0, second, 2));
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].button, Some(2));
    }

    #[test]
    fn the_gate() {
        assert!(feed_all(&mut KiaForteDecoder::new(), &press(40, FIRST, 1)).is_empty(), "one copy is not enough");
        // two presses with different frames, each one copy
        let mut other = FIRST;
        other[6] ^= 0x10;
        let mut wire = press(40, FIRST, 1);
        wire.extend(press(40, other, 1));
        assert!(feed_all(&mut KiaForteDecoder::new(), &wire).is_empty());
        // the same frame twice, but half a second apart
        let mut wire = press(40, FIRST, 1);
        wire.extend(press(40, FIRST, 1));
        assert!(feed_all(&mut KiaForteDecoder::new(), &wire).is_empty());
        // byte 0 is not 01
        let mut head = FIRST;
        head[0] = 0x03;
        assert!(feed_all(&mut KiaForteDecoder::new(), &press(40, head, 4)).is_empty());
        // the other polarity of the tones reads the same frame
        let flipped: Vec<LevelDuration> = press(40, FIRST, 4).iter().map(|p| LevelDuration::new(!p.level, p.duration_us)).collect();
        let got = feed_all(&mut KiaForteDecoder::new(), &flipped);
        assert_eq!(got.iter().map(|d| d.data).collect::<Vec<_>>(), [0x0ECD_69B5_E46B_08AD; 3]);
        // a reset forgets the frame before
        let mut dec = KiaForteDecoder::new();
        assert!(feed_all(&mut dec, &press(40, FIRST, 1)[..]).is_empty());
        assert_eq!(feed_resetting(&mut KiaForteDecoder::new(), &press(40, FIRST, 4)).len(), 2);
        assert!(KiaForteDecoder::new().encode(&DecodedSignal::new(0, 88), 1).is_none());
    }

    /// The recording: ten presses of four copies, five with button 01, five with 02. With a reset
    /// after every frame (as the registry does) copies two and four of a press are listed.
    #[test]
    fn imports_recording() {
        let path = std::path::Path::new("IMPORTS/KIA/more/Forte_315mhz_FM.sub");
        if !path.is_file() {
            eprintln!("Skip: IMPORTS not found (run from crate root)");
            return;
        }
        let (_, wire) = sub_pulses(path);
        let got = feed_resetting(&mut KiaForteDecoder::new(), &wire);
        let mut frames: Vec<(u8, u64)> = got.iter().map(|d| (d.button.unwrap(), d.data)).collect();
        frames.dedup();
        let buttons: Vec<u8> = frames.iter().map(|f| f.0).collect();
        eprintln!("FORTE {} frames, {} distinct, buttons {buttons:?}", got.len(), frames.len());
        assert!(got.iter().all(|d| d.serial == Some(0x99_0ECD) && d.extra.unwrap() >> 16 == 1 && d.data & 0xF == 0xD));
        assert_eq!((got.len(), buttons), (20, vec![1, 1, 1, 1, 1, 2, 2, 2, 2, 2]));
        assert_eq!(feed_all(&mut KiaForteDecoder::new(), &wire).len(), 30);
    }
}
