//! Lexus UX: the smart key of a 2021 Lexus UX 250 (433.66 MHz, two-tone FSK): a 256-bit frame sent
//! four times back to back, with a CRC-16 — **decode and display only**.
//!
//! **KAT's own decoder**, worked out from two recordings of one key
//! (`IMPORTS/Toyota + Lexus/Lexus UX 250 2021 lock.sub` and `... unlock.sub`, Flipper preset FM476,
//! five presses each). No reference reads the signal as it is recorded there; the name is KAT's.
//! FoxFW's [Lexus V0](super::lexus_v0) reads the first 224 bits of the same frame as they are sent,
//! without the differential step below (`docs/lexus_ux.md`).
//!
//! **Air format**: 2-FSK, 348.4 µs per half bit (1435 bit/s). Every bit has a change of tone in its
//! middle; the bit is whether the tone also changes at its start: no change is a 1, a change a 0
//! (differential Manchester: read as Manchester, each bit XORed with the one before). So the bits do
//! not depend on which tone is which. A press sends the frame four times without a gap (0.71 s).
//! The recordings' slicer made one tone about 120 µs short and the other about 120 µs long (226 /
//! 575 µs against 470 / 819 µs on average); the reader takes that skew out over the whole burst.
//!
//! **Frame (256 bits, 32 bytes)**:
//!
//! | Bits | Field |
//! |---|---|
//! | 0..7 | lead: 8 zeros |
//! | 8, 16, …, 248 | 1: every byte from byte 1 on begins with a 1 |
//! | 8..47 | `C4 80 80 88 80` in every frame |
//! | 49..52 | button: `0010` Lock, `0100` Unlock |
//! | 53..198 | the code: 128 bits (without the 1s at the bytes' starts), new with every press |
//! | 199 | 0 in every frame |
//! | 200..231 | `AF A6 9C 8B` in every frame of the one key: its ID |
//! | 232..235 | `1010` |
//! | 236..253 | CRC-16, 16 bits around the 1s at 240 and 248 |
//! | 254..255 | `11` |
//!
//! **CRC**: polynomial 0x1021 (CCITT), register 0 at the start, no reflection, nothing XORed at
//! the end, over bits 10..235 (the 1s at the bytes' starts included). It holds in all ten frames;
//! no other polynomial of 16 bits fits them. Its factor x + 1 gives every frame an even number of
//! 1 bits, so the sent tones end where they began and the four copies are sent alike.
//!
//! **Gate**: bits 1..47 (bit 0 has no bit before it at the start of a burst), the 1s at the
//! bytes' starts, the CRC and the closing `11`. One whole copy is enough.
//!
//! **Fields**: `serial` = bytes 25..28 (bits 200..231) as they stand, `button` = bits 49..52,
//! `data` = the code's first 64 bits, `extra` = its last 64, `data_count_bit` = 256, no counter,
//! `crc_valid` true. Bit 199 is not kept.

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
    rf: Rf::FM,
    encryption: "Rolling code (cipher unknown)",
    make: "Lexus",
    category: Category::Vehicle,
    buttons: Some(button_name),
    presets: &[&FM476],
};

const NAME: &str = "Lexus UX";
/// Half a bit: a press of four frames, 2048 half bits, lasts 713.6 ms.
const TE: u32 = 348;
const FRAME_BITS: usize = 256;
/// Frame bits 0..47: the lead and `C4 80 80 88 80`. The gate looks at bits 1..47.
const HEAD: u64 = 0x00C4_8080_8880;
const HEAD_BITS: usize = 48;
/// The CRC covers bits 10..235; its bits stand from 236 to 253, around 240 and 248.
const CRC_FROM: usize = 10;
const CRC_AT: usize = 236;
const CRC_END: usize = 254;
const CRC_POLY: u16 = 0x1021;
/// The button's bits, the code's (those of them that do not start a byte) and the ID's.
const BUTTON: std::ops::Range<usize> = 49..53;
const CODE: std::ops::Range<usize> = 53..199;
const ID: std::ops::Range<usize> = 200..232;

fn button_name(_name: &str, code: u8) -> Option<&'static str> {
    match code {
        0x2 => Some("Lock"),
        0x4 => Some("Unlock"),
        _ => None,
    }
}

/// Lexus UX decoder: reads a burst when it is over.
pub struct LexusUxDecoder {
    reader: BurstReader,
}

impl LexusUxDecoder {
    pub fn new() -> Self {
        Self { reader: BurstReader::skewed(TE, 2, 4096) }
    }
}

impl Default for LexusUxDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// CRC-16 (polynomial 0x1021, register 0 at the start) of `bits`, first bit first.
fn crc16(bits: &[bool]) -> u16 {
    bits.iter().fold(0, |crc, &b| (crc << 1) ^ if (crc >> 15 == 1) != b { CRC_POLY } else { 0 })
}

/// The bits of `range` that do not start a byte, in order.
fn unframed(frame: &[bool], range: std::ops::Range<usize>) -> Vec<bool> {
    range.filter(|i| i % 8 != 0).map(|i| frame[i]).collect()
}

/// Whether a frame whose bits 1..47 are the head passes the rest of the gate.
fn valid(frame: &[bool]) -> bool {
    (8..FRAME_BITS).step_by(8).all(|i| frame[i])
        && frame[CRC_END..] == [true, true]
        && crc16(&frame[CRC_FROM..CRC_AT]) == burst::number(&unframed(frame, CRC_AT..CRC_END)) as u16
}

fn signal(frame: &[bool]) -> DecodedSignal {
    let code = unframed(frame, CODE);
    DecodedSignal {
        serial: Some(burst::number(&frame[ID]) as u32),
        button: Some(burst::number(&frame[BUTTON]) as u8),
        counter: None,
        crc_valid: true,
        data: burst::number(&code[..64]),
        data_count_bit: FRAME_BITS,
        encoder_capable: false,
        extra: Some(burst::number(&code[64..])),
        protocol_display_name: None,
    }
}

/// The first whole frame of a finished burst. The Manchester bits of either phase, each XORed with
/// the one before it (the first bit of a burst has none: it is no bit).
fn read(halves: &[bool]) -> Option<DecodedSignal> {
    let head = burst::pattern(HEAD, HEAD_BITS);
    for phase in 0..2 {
        let sent = burst::bits(halves, phase, false);
        let bits: Vec<Option<bool>> = (0..sent.len())
            .map(|i| match (i.checked_sub(1).and_then(|j| sent[j]), sent[i]) {
                (Some(before), Some(now)) => Some(before != now),
                _ => None,
            })
            .collect();
        let Some(last) = bits.len().checked_sub(FRAME_BITS) else { continue };
        for at in 0..=last {
            if !burst::matches(&bits, at + 1, &head[1..]) {
                continue;
            }
            let Some(rest) = burst::take(&bits, at + 1, FRAME_BITS - 1) else { continue };
            let frame: Vec<bool> = std::iter::once(false).chain(rest).collect();
            if valid(&frame) {
                return Some(signal(&frame));
            }
        }
    }
    None
}

impl ProtocolDecoder for LexusUxDecoder {
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

    /// The first press of the lock recording.
    const ID_LOCK: u32 = 0xAFA6_9C8B;
    const CODE_HI: u64 = 0xBD58_A18F_C3D0_3C6F;
    const CODE_LO: u64 = 0x9DCE_65C4_4341_9258;

    /// A frame with `button`, the code `hi`/`lo` and `id` (its bytes' first bits set), the CRC
    /// filled in.
    fn frame(button: u8, hi: u64, lo: u64, id: u32) -> Vec<bool> {
        let mut f = vec![false; FRAME_BITS];
        f[..HEAD_BITS].copy_from_slice(&burst::pattern(HEAD, HEAD_BITS));
        for i in (8..FRAME_BITS).step_by(8) {
            f[i] = true;
        }
        f[BUTTON].copy_from_slice(&burst::pattern(u64::from(button), 4));
        let code = [burst::pattern(hi, 64), burst::pattern(lo, 64)].concat();
        for (i, b) in CODE.filter(|i| i % 8 != 0).zip(code) {
            f[i] = b;
        }
        f[ID].copy_from_slice(&burst::pattern(u64::from(id | 0x8080_8080), 32));
        f[232..236].copy_from_slice(&[true, false, true, false]);
        let crc = burst::pattern(u64::from(crc16(&f[CRC_FROM..CRC_AT])), 16);
        for (i, b) in (CRC_AT..CRC_END).filter(|i| i % 8 != 0).zip(crc) {
            f[i] = b;
        }
        f[CRC_END..].copy_from_slice(&[true, true]);
        f
    }

    /// The tones of `frames` back to back, behind a pause and with one behind them: each bit XORed
    /// into the one sent before it (the first against a 0), Manchester with HIGH then LOW for a 1.
    fn wire(frames: &[Vec<bool>]) -> Vec<LevelDuration> {
        let mut level = false;
        let sent: Vec<bool> = frames
            .iter()
            .flatten()
            .map(|&b| {
                level ^= b;
                level
            })
            .collect();
        let mut wire = vec![LevelDuration::new(false, 20_000)];
        wire.extend(burst::wire(&sent, TE, 20_000));
        burst::merged(wire)
    }

    fn press(f: &[bool]) -> Vec<LevelDuration> {
        wire(&vec![f.to_vec(); 4])
    }

    /// The pulses as the recordings' slicer gives them: HIGH `by` µs shorter, LOW `by` µs longer.
    fn skewed(mut wire: Vec<LevelDuration>, by: i64) -> Vec<LevelDuration> {
        let n = wire.len();
        for p in &mut wire[1..n - 1] {
            let d = i64::from(p.duration_us) + if p.level { -by } else { by };
            p.duration_us = d as u32;
        }
        wire
    }

    #[test]
    fn decodes_a_press_at_either_polarity_and_skew() {
        let f = frame(0x2, CODE_HI, CODE_LO, ID_LOCK);
        let got = feed_all(&mut LexusUxDecoder::new(), &press(&f));
        assert_eq!(got.len(), 1, "four copies, one decode: {got:?}");
        let d = &got[0];
        assert_eq!((d.serial, d.button, d.counter), (Some(ID_LOCK), Some(0x2), None));
        assert_eq!((d.data, d.extra, d.data_count_bit), (CODE_HI, Some(CODE_LO), 256));
        assert!(d.crc_valid && !d.encoder_capable);
        // the frame has an even number of 1 bits: every copy is sent alike
        assert_eq!(f.iter().filter(|&&b| b).count() % 2, 0);
        // the other tone HIGH
        let flipped: Vec<LevelDuration> = press(&f).iter().map(|p| LevelDuration::new(!p.level, p.duration_us)).collect();
        assert_eq!(feed_all(&mut LexusUxDecoder::new(), &flipped), got);
        // the recordings' skew, either way round
        assert_eq!(feed_all(&mut LexusUxDecoder::new(), &skewed(press(&f), 120)), got);
        assert_eq!(feed_all(&mut LexusUxDecoder::new(), &skewed(press(&f), -120)), got);
        // one copy is enough; the end of the input ends the burst
        let mut one = wire(std::slice::from_ref(&f));
        assert_eq!(feed_all(&mut LexusUxDecoder::new(), &one), got);
        one.pop();
        let mut dec = LexusUxDecoder::new();
        assert!(feed_all(&mut dec, &one).is_empty());
        assert_eq!(dec.flush(), Some(got[0].clone()));
        assert!(LexusUxDecoder::new().encode(d, 0x2).is_none());
        assert_eq!(button_name(NAME, 0x2), Some("Lock"));
        assert_eq!(button_name(NAME, 0x4), Some("Unlock"));
        assert_eq!(button_name(NAME, 0x1), None);
    }

    #[test]
    fn the_gate() {
        let f = frame(0x4, CODE_HI ^ 0x0123, !CODE_LO, ID_LOCK);
        let read_one = |f: &[bool]| feed_all(&mut LexusUxDecoder::new(), &wire(&[f.to_vec()])).len();
        assert_eq!(read_one(&f), 1);
        for (bit, what) in [
            (3, "the lead"),
            (20, "the head"),
            (64, "a byte's first bit"),
            (100, "the code: CRC"),
            (205, "the ID: CRC"),
            (233, "the 1010 before the CRC: CRC"),
            (240, "a byte's first bit in the CRC"),
            (250, "the CRC"),
            (254, "the closing 11"),
            (255, "the closing 11"),
        ] {
            let mut g = f.clone();
            g[bit] = !g[bit];
            assert_eq!(read_one(&g), 0, "bit {bit}: {what}");
        }
        // bit 0 is not looked at
        let mut g = f.clone();
        g[0] = true;
        assert_eq!(read_one(&g), 1);
        // a pulse of another length in the frame: what is left of it is no frame
        let mut w = wire(std::slice::from_ref(&f));
        w[200].duration_us = 5 * TE;
        assert!(feed_all(&mut LexusUxDecoder::new(), &w).is_empty());
        // a damaged copy among good ones
        let mut copies = vec![f.clone(); 3];
        copies[0][150] = !copies[0][150];
        assert_eq!(feed_all(&mut LexusUxDecoder::new(), &wire(&copies)).len(), 1);
        // another speed
        let slow: Vec<LevelDuration> = press(&f)
            .into_iter()
            .map(|p| if p.duration_us < 5_000 { LevelDuration::new(p.level, p.duration_us * 5 / 4) } else { p })
            .collect();
        assert!(feed_all(&mut LexusUxDecoder::new(), &slow).is_empty());
    }

    /// The CRC of the recordings' frames: bits 10..235 of the first lock frame, as KAT reads them.
    #[test]
    fn crc_of_a_recorded_frame() {
        let f = frame(0x2, CODE_HI, CODE_LO, ID_LOCK);
        assert_eq!(burst::number(&unframed(&f, CRC_AT..CRC_END)), 0x9609);
        assert_eq!(crc16(&[]), 0);
        // the check value of CRC-16/XMODEM
        let check: Vec<bool> = b"123456789".iter().flat_map(|&c| burst::pattern(u64::from(c), 8)).collect();
        assert_eq!(crc16(&check), 0x31C3);
    }

    /// The recordings: five presses each, every one read with the key's ID, the file's button, a
    /// code of its own and the CRC. (The unlock recording ends in its fifth press: the end of the
    /// input ends that burst.)
    #[test]
    fn imports_recordings() {
        let dir = std::path::Path::new("IMPORTS/Toyota + Lexus");
        let files = [("Lexus UX 250 2021 lock.sub", 0x2), ("Lexus UX 250 2021 unlock.sub", 0x4)];
        if !dir.join(files[0].0).is_file() {
            eprintln!("Skip: IMPORTS not found (run from crate root)");
            return;
        }
        let mut codes = std::collections::HashSet::new();
        for (file, button) in files {
            let (_, wire) = sub_pulses(&dir.join(file));
            let mut dec = LexusUxDecoder::new();
            let mut got = feed_resetting(&mut dec, &wire);
            got.extend(dec.flush());
            eprintln!("Lexus UX {file}: {} presses", got.len());
            assert_eq!(got.len(), 5, "{file}");
            assert!(got.iter().all(|d| d.serial == Some(ID_LOCK) && d.button == Some(button) && d.crc_valid), "{file}");
            codes.extend(got.iter().map(|d| (d.data, d.extra)));
            if button == 0x2 {
                assert_eq!((got[0].data, got[0].extra), (CODE_HI, Some(CODE_LO)));
            }
        }
        assert_eq!(codes.len(), 10);
        // through the registry, as the app imports the files: these decodes and nothing else
        for (file, button) in files {
            let got = registry_decodes(&dir.join(file));
            assert_eq!(got.len(), 5, "{file}: {got:?}");
            assert!(got.iter().all(|(name, d)| name == NAME && d.button == Some(button)), "{file}: {got:?}");
        }
    }

    /// FoxFW's Lexus V0 reads the same frame as it is sent, without the differential step: its
    /// 224 bits are bits 8..231 of the bits sent, so its `serial` is the ID as sent (`CAC4E8F2`, or
    /// `353B170D` after an odd number of 1 bits) and its `data` bytes 5..12 as sent. It needs 16
    /// short pulses before the frame, which a press's first copy does not have, and it reads
    /// nothing in the recordings: their HIGH half bits (226 µs on average) are under its 250 µs.
    #[test]
    fn lexus_v0_reads_the_frame_as_sent() {
        use super::super::lexus_v0::LexusV0Decoder;
        let f = frame(0x2, CODE_HI, CODE_LO, ID_LOCK);
        let got = feed_all(&mut LexusV0Decoder::new(), &press(&f));
        assert_eq!(got.len(), 3, "{got:?}");
        assert!(got.iter().all(|d| d.serial == Some(0xCAC4_E8F2) && d.data == 0x00E6_A6DE_C17A_A0AC && d.data_count_bit == 224));
        let path = std::path::Path::new("IMPORTS/Toyota + Lexus/Lexus UX 250 2021 lock.sub");
        if path.is_file() {
            assert!(feed_all(&mut LexusV0Decoder::new(), &sub_pulses(path).1).is_empty());
        }
    }
}
