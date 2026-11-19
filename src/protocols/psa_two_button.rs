//! PSA 2-Button: a two-button Valeo flip key for Citroën and Peugeot with a 76-bit frame —
//! **decode and display only**.
//!
//! **KAT's own decoder**, worked out from two recordings of one Citroën key that a tester sent in
//! (`IMPORTS/GROUPE PSA/citroen_two_button_valeo_key_unlock_x50.sub` and `..._lock_x50.sub`,
//! 433.9 MHz, on-off keying, 50 presses each). No reference reads the signal; the name is KAT's.
//! Not the 80-bit frame of [PSA](super::psa).
//!
//! **Air format**: Manchester (carrier then pause is a 1), 207 µs per half bit (2415 bit/s). A
//! press is about 78 ms of carrier, 6.9 ms of pause, then one frame: half a bit of carrier, the 76
//! bits, and 2.5 ms of carrier (twelve half bits).
//!
//! **Frame (76 bits)**:
//!
//! | Bits | Field |
//! |---|---|
//! | 8 | `CE` in every frame of the one key recorded |
//! | 4 | button: `C` Unlock, `3` Lock |
//! | 64 | the code: changes with every press |
//!
//! **The code** KAT cannot undo or continue, but it is no strong cipher's output: its eight bytes
//! are XORs of the press counter's low byte, a second byte that changes with the press (three bits
//! of it are a check on the counter), bytes that cancel when the two halves are XORed, and values
//! that stay. In 89 of the 96 frames read, the counter is the XOR of two bytes of that half-XOR and
//! a constant; what picks the arrangement of a press is not known (`docs/psa_two_button.md`).
//!
//! **Gate** (there is no checksum): a burst that begins behind a pause with the half bit of
//! carrier, holds exactly 76 Manchester bits without a violation, and is ended by a carrier pulse
//! of 2.0 to 3.3 ms.
//!
//! **Fields**: `data` = the code, `extra` = the 12 bits before it, `data_count_bit` = 76,
//! `button` = the button nibble, no serial, no counter.

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
    encryption: "Rolling code (cipher unknown)",
    make: "Peugeot/Citroen",
    category: Category::Vehicle,
    buttons: Some(button_name),
    presets: &[&AM650],
};

const NAME: &str = "PSA 2-Button";
/// Half a bit: a pulse and the pause behind it are 414 µs together in the recordings (a Flipper's
/// receiver gave 120 to 240 µs of carrier and 200 to 320 µs of pause for one half bit each).
const TE: u32 = 207;
const FRAME_BITS: usize = 76;
/// The carrier behind the frame: 2.5 ms, and 2.7 ms when the frame's last bit is a 0 (its second
/// half is carrier too).
const END_MIN: u32 = 2_000;
const END_MAX: u32 = 3_300;

fn button_name(_name: &str, code: u8) -> Option<&'static str> {
    match code {
        0xC => Some("Unlock"),
        0x3 => Some("Lock"),
        _ => None,
    }
}

/// PSA 2-Button decoder: reads a burst when the carrier behind the frame ends it.
pub struct PsaTwoButtonDecoder {
    reader: BurstReader,
}

impl PsaTwoButtonDecoder {
    pub fn new() -> Self {
        Self { reader: BurstReader::skewed(TE, 2, 1024) }
    }
}

impl Default for PsaTwoButtonDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// The frame of a burst that a long carrier pulse ended: the pause before the burst, the half bit
/// of carrier, 76 bits. The pulse that ended the burst is its last half: the second half of a last
/// 0 bit, or one half more behind a last 1 bit.
fn read(halves: &[bool]) -> Option<DecodedSignal> {
    let whole = 2 + 2 * FRAME_BITS;
    if !(halves.len() == whole || halves.len() == whole + 1) || halves[0] || !halves[1] {
        return None;
    }
    let frame = burst::take(&burst::bits(halves, 2, false), 0, FRAME_BITS)?;
    Some(DecodedSignal {
        serial: None,
        button: Some(burst::number(&frame[8..12]) as u8),
        counter: None,
        crc_valid: false,
        data: burst::number(&frame[12..]),
        data_count_bit: FRAME_BITS,
        encoder_capable: false,
        extra: Some(burst::number(&frame[..12])),
        protocol_display_name: None,
    })
}

impl ProtocolDecoder for PsaTwoButtonDecoder {
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
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let halves = self.reader.feed(level, duration)?;
        (level && (END_MIN..=END_MAX).contains(&duration)).then(|| read(&halves)).flatten()
    }

    /// A burst the input ends has no carrier behind it: no frame.
    fn flush(&mut self) -> Option<DecodedSignal> {
        self.reader.flush();
        None
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

    /// The first whole frame of the unlock recording.
    const HEAD: u64 = 0xCEC;
    const CODE: u64 = 0xB328_FDAE_A203_3C71;

    /// The pulses of a frame of `bits`: the pause, the half bit of carrier, the bits, `end` µs of
    /// carrier and silence.
    fn wire_of(bits: &[bool], pause: u32, end: u32) -> Vec<LevelDuration> {
        let mut wire = vec![LevelDuration::new(false, pause), LevelDuration::new(true, TE)];
        wire.extend(bits.iter().flat_map(|&b| [LevelDuration::new(b, TE), LevelDuration::new(!b, TE)]));
        wire.push(LevelDuration::new(true, end));
        wire.push(LevelDuration::new(false, 100_000));
        burst::merged(wire)
    }

    fn frame_bits(head: u64, code: u64) -> Vec<bool> {
        let mut bits = burst::pattern(head, 12);
        bits.extend(burst::pattern(code, 64));
        bits
    }

    fn press(head: u64, code: u64) -> Vec<LevelDuration> {
        wire_of(&frame_bits(head, code), 6_900, 2_500)
    }

    #[test]
    fn decodes_a_press() {
        let got = feed_all(&mut PsaTwoButtonDecoder::new(), &press(HEAD, CODE));
        assert_eq!(got.len(), 1, "{got:?}");
        let d = &got[0];
        assert_eq!((d.data, d.extra, d.data_count_bit), (CODE, Some(HEAD), 76));
        assert_eq!((d.serial, d.button, d.counter), (None, Some(0xC), None));
        assert!(!d.crc_valid && !d.encoder_capable);
        assert_eq!(feed_resetting(&mut PsaTwoButtonDecoder::new(), &press(HEAD, CODE)).len(), 1);
        // a frame that ends with a 0 bit: its second half is part of the carrier behind it
        let got = feed_all(&mut PsaTwoButtonDecoder::new(), &press(0xCE3, CODE ^ 1));
        assert_eq!(got.len(), 1);
        assert_eq!((got[0].data, got[0].button), (CODE ^ 1, Some(0x3)));
        // as a receiver gives it: carrier pulses 70 µs short, pauses 70 µs long
        let mut wire = press(HEAD, CODE);
        let n = wire.len();
        for p in &mut wire[1..n - 2] {
            p.duration_us = if p.level { p.duration_us - 70 } else { p.duration_us + 70 };
        }
        let got = feed_all(&mut PsaTwoButtonDecoder::new(), &wire);
        assert_eq!(got.iter().map(|d| d.data).collect::<Vec<_>>(), [CODE]);
        assert_eq!(button_name(NAME, 0xC), Some("Unlock"));
        assert_eq!(button_name(NAME, 0x3), Some("Lock"));
        assert_eq!(button_name(NAME, 0x1), None);
        assert!(PsaTwoButtonDecoder::new().encode(&got[0], 1).is_none());
    }

    #[test]
    fn the_gate() {
        let read_one = |wire: Vec<LevelDuration>| feed_all(&mut PsaTwoButtonDecoder::new(), &wire).len();
        let bits = frame_bits(HEAD, CODE);
        assert_eq!(read_one(wire_of(&bits, 6_900, 2_500)), 1);
        // the carrier behind the frame
        assert_eq!(read_one(wire_of(&bits, 6_900, 2_000)), 1);
        assert_eq!(read_one(wire_of(&bits, 6_900, 3_300)), 1);
        assert_eq!(read_one(wire_of(&bits, 6_900, 1_500)), 0, "a short end");
        assert_eq!(read_one(wire_of(&bits, 6_900, 4_500)), 0, "a long end");
        // a bit more or less
        assert_eq!(read_one(wire_of(&bits[1..], 6_900, 2_500)), 0, "75 bits");
        let mut longer = vec![true];
        longer.extend(&bits);
        assert_eq!(read_one(wire_of(&longer, 6_900, 2_500)), 0, "77 bits");
        // a pulse of another length in the frame: the burst begins behind it, and is too short
        let mut wire = wire_of(&bits, 6_900, 2_500);
        wire[40].duration_us = 4 * TE;
        assert_eq!(read_one(wire), 0);
        // no carrier behind the frame: the pause, or the end of the input
        let mut wire = wire_of(&bits, 6_900, 2_500);
        let n = wire.len();
        wire.truncate(n - 2);
        let mut dec = PsaTwoButtonDecoder::new();
        assert!(feed_all(&mut dec, &wire).is_empty() && dec.flush().is_none());
        wire.push(LevelDuration::new(false, 50_000));
        assert_eq!(read_one(wire), 0);
        // another speed
        let slow: Vec<LevelDuration> = wire_of(&bits, 6_900, 2_500)
            .into_iter()
            .map(|p| if p.duration_us < 1_000 { LevelDuration::new(p.level, p.duration_us * 5 / 4) } else { p })
            .collect();
        assert_eq!(read_one(slow), 0);
    }

    /// The two recordings: 50 presses each, every frame read with the head `CE`, the file's button
    /// and a code of its own.
    #[test]
    fn imports_recordings() {
        let dir = std::path::Path::new("IMPORTS/GROUPE PSA");
        let files = [("citroen_two_button_valeo_key_unlock_x50.sub", 0xC, 40), ("citroen_two_button_valeo_key_lock_x50.sub", 0x3, 49)];
        if !dir.join(files[0].0).is_file() {
            eprintln!("Skip: IMPORTS not found (run from crate root)");
            return;
        }
        let mut codes = std::collections::HashSet::new();
        for (file, button, least) in files {
            let (_, wire) = sub_pulses(&dir.join(file));
            let got = feed_resetting(&mut PsaTwoButtonDecoder::new(), &wire);
            eprintln!("PSA 2-Button {file}: {} frames", got.len());
            assert!(got.len() >= least && got.len() <= 50, "{file}: {}", got.len());
            assert!(got.iter().all(|d| d.extra == Some(0xCE0 | button) && d.button == Some(button as u8)));
            codes.extend(got.iter().map(|d| d.data));
            if button == 0xC {
                assert!(got.iter().any(|d| d.data == CODE));
            }
        }
        assert!(codes.len() >= 89, "{}", codes.len());
    }
}
