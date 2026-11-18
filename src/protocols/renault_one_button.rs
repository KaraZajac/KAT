//! Renault 1-Button: an old one-button Renault remote (the board is dated 1999; an 8-pin chip and
//! a SAW resonator) — **decode and display only**.
//!
//! **KAT's own decoder**, worked out from one recording of one remote
//! (`IMPORTS/RENAULT/renault_one_button_remote_x110.sub`, 433.87 MHz, on-off keying, 110 presses).
//! No reference reads the signal; the name is KAT's. None of the other Renault protocols.
//!
//! **Air format**: Manchester (carrier then pause is a 1), 473.5 µs per half bit (1056 bit/s). A
//! press is the frame two to four times, 14 ms of pause and 8.7 ms of carrier between the copies.
//! A copy is a run of 0 bits (13 before the first frame, 5 before the others), the frame, and
//! one more half bit of carrier.
//!
//! **Frame (70 bits)**:
//!
//! | Bits | Field |
//! |---|---|
//! | 2 | `11` |
//! | 24 | `0A84DC` in every frame of the one remote recorded: its ID |
//! | 36 | changes with every press: no bit of it follows from the press before (110 presses) |
//! | 8 | check: with two 0 bits in front the frame is nine bytes, and their XOR is 0 |
//!
//! **Gate**: a burst of Manchester bits without a violation that ends with the frame and the
//! half bit of carrier (with the pause behind it, a 1 bit); two or more 0 bits before the frame;
//! the `11`; the check; an ID that is not 0.
//!
//! **Fields**: the nine bytes: `data` = the first eight, `extra` = the check byte,
//! `data_count_bit` = 72; `serial` = the ID; no button and no counter.

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
    encryption: "Rolling Code",
    make: "Renault",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "Renault 1-Button";
/// Half a bit: a Flipper recorded 455 and 492 µs (carrier and pause), 938 and 958 µs for two.
const TE: u32 = 473;
const TOL: u32 = 120;
/// The frame with the two 0 bits in front of it: nine bytes.
const BYTES_BITS: usize = 72;
/// 0 bits KAT wants before those nine bytes (the remote sends 11, and 3 before a later copy).
const LEAD_MIN: usize = 2;

/// Renault 1-Button decoder: reads a burst when it is over.
pub struct RenaultOneButtonDecoder {
    reader: BurstReader,
}

impl RenaultOneButtonDecoder {
    pub fn new() -> Self {
        Self { reader: BurstReader::new(TE, TOL, 512) }
    }
}

impl Default for RenaultOneButtonDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// The decode of nine bytes (`data`: the first eight; `check`: the ninth) that pass the gate.
fn signal(data: u64, check: u8) -> Option<DecodedSignal> {
    let xor = data.to_be_bytes().iter().fold(check, |x, b| x ^ b);
    let serial = ((data >> 36) & 0xFF_FFFF) as u32;
    (xor == 0 && data >> 60 == 0x3 && serial != 0).then(|| DecodedSignal {
        serial: Some(serial),
        button: None,
        counter: None,
        crc_valid: true,
        data,
        data_count_bit: BYTES_BITS,
        encoder_capable: false,
        extra: Some(u64::from(check)),
        protocol_display_name: None,
    })
}

/// The frame of a finished burst: the 72 bits before its last bit, which is the closing carrier
/// and the pause (a 1), behind 0 bits. On-off keying has one polarity.
fn read(halves: &[bool]) -> Option<DecodedSignal> {
    for phase in 0..2 {
        let bits = burst::bits(halves, phase, false);
        let Some(start) = bits.len().checked_sub(BYTES_BITS + 1) else { continue };
        if bits.last() != Some(&Some(true)) || burst::run_before(&bits, start, false) < LEAD_MIN {
            continue;
        }
        if let Some(frame) = burst::take(&bits, start, BYTES_BITS) {
            if let Some(d) = signal(burst::number(&frame[..64]), burst::number(&frame[64..]) as u8) {
                return Some(d);
            }
        }
    }
    None
}

impl ProtocolDecoder for RenaultOneButtonDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: TE, te_long: 2 * TE, te_delta: TOL, min_count_bit: BYTES_BITS }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.reader.reset();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        self.reader.feed(level, duration).and_then(|halves| read(&halves))
    }

    fn flush(&mut self) -> Option<DecodedSignal> {
        self.reader.flush().and_then(|halves| read(&halves))
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

    /// The first press of the recording: nine bytes.
    const DATA: u64 = 0x30A8_4DCF_9013_E860;
    const CHECK: u8 = 0x11;

    /// `lead` 0 bits, the nine bytes and the closing half bit of carrier (with the pause, a 1 bit).
    fn frame_bits(data: u64, check: u8, lead: usize) -> Vec<bool> {
        let mut bits = vec![false; lead];
        bits.extend(burst::pattern(data, 64));
        bits.extend(burst::pattern(u64::from(check), 8));
        bits.push(true);
        bits
    }

    /// A later copy of a press: 8.7 ms of carrier, the bits, 14 ms of pause.
    fn copy(data: u64, check: u8, lead: usize) -> Vec<LevelDuration> {
        let bits = frame_bits(data, check, lead);
        let mut wire = vec![LevelDuration::new(true, 8_700)];
        wire.extend(burst::wire(&bits, TE, 14_000));
        wire
    }

    fn press(data: u64, check: u8) -> Vec<LevelDuration> {
        let mut wire = vec![LevelDuration::new(false, 30_000), LevelDuration::new(true, 1_900), LevelDuration::new(false, 1_420)];
        wire.extend(burst::wire(&frame_bits(data, check, 11), TE, 14_000));
        for _ in 0..3 {
            wire.extend(copy(data, check, 3));
        }
        burst::merged(wire)
    }

    #[test]
    fn decodes_a_press() {
        let got = feed_all(&mut RenaultOneButtonDecoder::new(), &press(DATA, CHECK));
        assert_eq!(got.len(), 4, "{got:?}");
        let d = &got[0];
        assert_eq!((d.data, d.extra, d.data_count_bit), (DATA, Some(0x11), 72));
        assert_eq!((d.serial, d.button, d.counter), (Some(0x0A84DC), None, None));
        assert!(d.crc_valid && !d.encoder_capable);
        assert_eq!(feed_resetting(&mut RenaultOneButtonDecoder::new(), &press(DATA, CHECK)).len(), 4);
        // a frame that ends with a 0 bit
        let (data, check) = (DATA ^ 0x100, CHECK ^ 0x01);
        assert_eq!(feed_all(&mut RenaultOneButtonDecoder::new(), &press(data, check)).len(), 4);
        // without the closing half bit of carrier, or without the pause behind it, there is no frame
        let mut bits = frame_bits(DATA, CHECK, 3);
        bits.pop();
        let wire = burst::merged([vec![LevelDuration::new(true, 8_700)], burst::wire(&bits, TE, 14_000)].concat());
        assert!(feed_all(&mut RenaultOneButtonDecoder::new(), &wire).is_empty());
        let mut wire = burst::merged(copy(DATA, CHECK, 3));
        wire.pop();
        let mut dec = RenaultOneButtonDecoder::new();
        assert!(feed_all(&mut dec, &wire).is_empty() && dec.flush().is_none());
        assert!(RenaultOneButtonDecoder::new().encode(&got[0], 1).is_none());
    }

    #[test]
    fn the_gate() {
        let read_one = |data: u64, check: u8, lead: usize| feed_all(&mut RenaultOneButtonDecoder::new(), &burst::merged(copy(data, check, lead))).len();
        assert_eq!(read_one(DATA, CHECK, 3), 1);
        assert_eq!(read_one(DATA, CHECK, 2), 1, "two 0 bits of lead");
        assert_eq!(read_one(DATA, CHECK, 1), 0, "one 0 bit of lead");
        assert_eq!(read_one(DATA, CHECK ^ 0x40, 3), 0, "the check");
        assert_eq!(read_one(DATA ^ 0x40, CHECK, 3), 0, "the check");
        // nine bytes whose XOR is 0 without the `11`, and with an ID of 0
        assert_eq!(read_one(0, 0, 3), 0, "a run of 0 bits");
        assert_eq!(read_one(DATA ^ (0x50 << 56), CHECK ^ 0x50, 3), 0, "another start than 11");
        assert_eq!(read_one(0x3000_0000_0000_0000, 0x30, 3), 0, "an ID of 0");
        assert_eq!(read_one(0x3000_0010_0000_0000, 0x20, 3), 1);
        // a pulse of another length in the frame: the burst ends there, and what is left is too short
        let mut wire = burst::merged(copy(DATA, CHECK, 3));
        let n = wire.len();
        wire[n - 40].duration_us = 3 * TE;
        assert!(feed_all(&mut RenaultOneButtonDecoder::new(), &wire).is_empty());
        // a longer run of bits that ends with a frame is read from its end
        let mut bits = burst::pattern(0x5A5A, 16);
        bits.extend(frame_bits(DATA, CHECK, 4));
        let wire = burst::merged([vec![LevelDuration::new(false, 20_000)], burst::wire(&bits, TE, 14_000)].concat());
        assert_eq!(feed_all(&mut RenaultOneButtonDecoder::new(), &wire).len(), 1);
    }

    /// The recording: 110 presses, each with its own frame, the ID `0A84DC` in all.
    #[test]
    fn imports_recording() {
        let path = std::path::Path::new("IMPORTS/RENAULT/renault_one_button_remote_x110.sub");
        if !path.is_file() {
            eprintln!("Skip: IMPORTS not found (run from crate root)");
            return;
        }
        let (_, wire) = sub_pulses(path);
        let got = feed_resetting(&mut RenaultOneButtonDecoder::new(), &wire);
        let mut frames: Vec<(u64, Option<u64>)> = got.iter().map(|d| (d.data, d.extra)).collect();
        frames.dedup();
        eprintln!("RENAULT: {} frames, {} presses", got.len(), frames.len());
        assert!(got.iter().all(|d| d.serial == Some(0x0A84DC)));
        assert_eq!(frames.len(), 110);
        assert_eq!(frames.iter().collect::<std::collections::HashSet<_>>().len(), 110);
        assert_eq!(frames[0], (DATA, Some(0x11)));
    }
}
