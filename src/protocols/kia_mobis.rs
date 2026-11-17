//! Kia Mobis: the smart keys Hyundai Mobis builds for Kia (and, by the FCC filings, Hyundai),
//! 2017 to 2019 boards: a 104-bit Manchester frame with the press counter and the button in the
//! clear — **decode and display only**.
//!
//! **KAT's own decoder**, worked out from three recordings sent in by the user on 2026-11-16
//! (`IMPORTS/KIA/more/`: a Kia Niro, a Kia K7 and a 2019-2022 Kia Sportage QL, all 433.92 MHz, Flipper
//! preset FM476, ten presses each). No reference reads the signal; the name is KAT's. (FoxFW's
//! Ford V3 takes one clipped burst of two of the files for a frame of its own: both frames are 104
//! bits long.)
//!
//! **Air format**: 2-FSK, Manchester, 194.8 µs per half bit (2567 bit/s) on all three fobs. A burst
//! is 16 one bits, the word `0x6AC2`, then 72 bits; 40.5 ms. A press sends it up to three times,
//! 135 to 170 ms apart.
//!
//! **FCC filings**: the K7's remote is `TQ8-FOB-4F34` (Hyundai Mobis "Fob Smart Key", five
//! buttons: lock, unlock, panic, tailgate, remote start). Its test report shows what the recording
//! does: three bursts of 41.0 ms, 167.4 ms apart, 0.38 s a press; the lower tone 40.5 kHz under
//! 433.92 MHz, 98.4 kHz of bandwidth. The filings of `TQ8-FOB-4F17` to `-4F26` show the same
//! bursts (41.5 to 42.0 ms, the tones 80 kHz apart). The FCC IDs of the Niro's and the Sportage's
//! remotes are not known.
//!
//! **Frame (72 bits behind the word)**:
//!
//! | Bits | Field | Notes |
//! |---|---|---|
//! | 0..27 | Counter | up by one with every press on all three fobs (1 to 14 seen; the upper bits were always 0, so the width is a reading, not a finding) |
//! | 28..31 | Button | 1 or 2 seen (two of the recordings switch after five presses); which is which is not known |
//! | 32..71 | Code | changes with every press; a few of its first bits stay per fob. No fob ID in the clear, no checksum found (CRC-8 of every polynomial over the obvious spans, sum, XOR) |
//!
//! **Gate** (there is no checksum): at least 6 one bits, the 16-bit word, 72 bits without one
//! Manchester violation, and the burst ending with the frame: at most 8 half bits behind it (an FM
//! receiver's noise sets in as the carrier drops, and a few of its pulses pass for half bits).
//! Either polarity of the two tones is read.
//!
//! **Fields**: `data` = frame bits 8..71, `extra` = frame bits 0..7, `data_count_bit` = 72,
//! `counter` = the counter's low 16 bits, `button` = the button nibble, no serial.

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
    encryption: "Rolling code (cipher unknown), counter in the clear",
    make: "Kia/Hyundai",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&FM476],
};

const NAME: &str = "Kia Mobis";
const TE: u32 = 195;
const TOL: u32 = 60;
const WORD: u64 = 0x6AC2;
const MIN_ONES: usize = 6;
const DATA_BITS: usize = 72;
/// Half bits a burst may go on behind the frame (receiver noise as the carrier drops).
const MAX_SPARE: usize = 8;

/// Kia Mobis decoder: reads a burst when it is over.
pub struct KiaMobisDecoder {
    reader: BurstReader,
}

impl KiaMobisDecoder {
    pub fn new() -> Self {
        Self { reader: BurstReader::new(TE, TOL, 512) }
    }
}

impl Default for KiaMobisDecoder {
    fn default() -> Self {
        Self::new()
    }
}

fn signal(frame: &[bool]) -> DecodedSignal {
    let counter = burst::number(&frame[..28]);
    DecodedSignal {
        serial: None,
        button: Some(burst::number(&frame[28..32]) as u8),
        counter: Some(counter as u16),
        crc_valid: false,
        data: burst::number(&frame[8..]),
        data_count_bit: DATA_BITS,
        encoder_capable: false,
        extra: Some(burst::number(&frame[..8])),
        protocol_display_name: None,
    }
}

/// The frame of a finished burst, if it is one.
fn read(halves: &[bool]) -> Option<DecodedSignal> {
    let word = burst::pattern(WORD, 16);
    for phase in 0..2 {
        for inverted in [false, true] {
            let bits = burst::bits(halves, phase, inverted);
            let Some(last) = bits.len().checked_sub(16 + DATA_BITS) else { continue };
            for at in (0..=last).rev() {
                if burst::spare(halves.len(), phase, at + 16 + DATA_BITS) > MAX_SPARE {
                    break;
                }
                if burst::matches(&bits, at, &word) && burst::run_before(&bits, at, true) >= MIN_ONES {
                    if let Some(frame) = burst::take(&bits, at + 16, DATA_BITS) {
                        return Some(signal(&frame));
                    }
                }
            }
        }
    }
    None
}

impl ProtocolDecoder for KiaMobisDecoder {
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
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let halves = self.reader.feed(level, duration)?;
        read(&halves)
    }

    fn flush(&mut self) -> Option<DecodedSignal> {
        let halves = self.reader.flush()?;
        read(&halves)
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

    /// 16 ones, the word and a frame: counter 0x21 >> 4 = 2, button 1, as the Niro's second press.
    fn bits_of(ones: usize, frame: [u8; 9]) -> Vec<bool> {
        let mut b = vec![true; ones];
        b.extend(burst::pattern(WORD, 16));
        for byte in frame {
            b.extend(burst::pattern(u64::from(byte), 8));
        }
        b
    }

    const NIRO: [u8; 9] = [0x00, 0x00, 0x00, 0x21, 0x35, 0xDA, 0x9B, 0x0A, 0x9D];

    #[test]
    fn decodes_a_synthetic_burst_at_either_polarity() {
        let wire = burst::wire(&bits_of(16, NIRO), TE, 100_000);
        let got = feed_all(&mut KiaMobisDecoder::new(), &wire);
        assert_eq!(got.len(), 1, "{got:?}");
        let d = &got[0];
        assert_eq!((d.counter, d.button, d.serial), (Some(2), Some(1), None));
        assert_eq!((d.data, d.extra, d.data_count_bit), (0x0000_2135_DA9B_0A9D, Some(0), 72));
        assert!(!d.encoder_capable && !d.crc_valid);
        let flipped: Vec<LevelDuration> = wire.iter().map(|p| LevelDuration::new(!p.level, p.duration_us)).collect();
        let got = feed_all(&mut KiaMobisDecoder::new(), &flipped);
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].data, 0x0000_2135_DA9B_0A9D);
        // a frame that ends with a 0 bit ends HIGH: the gap is its own pulse
        let mut even = NIRO;
        even[8] = 0x9C;
        let got = feed_all(&mut KiaMobisDecoder::new(), &burst::wire(&bits_of(16, even), TE, 100_000));
        assert_eq!(got[0].data & 0xFF, 0x9C);
    }

    #[test]
    fn the_gate() {
        let dec = |bits: &[bool]| feed_all(&mut KiaMobisDecoder::new(), &burst::wire(bits, TE, 100_000)).len();
        assert_eq!(dec(&bits_of(6, NIRO)), 1, "six ones are enough");
        assert_eq!(dec(&bits_of(5, NIRO)), 0, "five are not");
        // a wrong word
        let mut b = bits_of(16, NIRO);
        b[16 + 15] = !b[16 + 15];
        assert_eq!(dec(&b), 0);
        // five bits too many, or one too few, behind the word
        let mut long = bits_of(16, NIRO);
        long.extend([true, false, true, true, false]);
        assert_eq!(dec(&long), 0);
        let mut short = bits_of(16, NIRO);
        short.pop();
        assert_eq!(dec(&short), 0);
        // one pulse out of tolerance splits the burst
        let mut wire = burst::wire(&bits_of(16, NIRO), TE, 100_000);
        let mid = wire.len() - 20;
        wire[mid].duration_us = 700;
        assert!(feed_all(&mut KiaMobisDecoder::new(), &wire).is_empty());
        assert!(KiaMobisDecoder::new().encode(&DecodedSignal::new(0, 72), 1).is_none());
    }

    /// The three recordings: counters go up by one per press, the K7 and the Sportage change
    /// button half way.
    #[test]
    fn imports_recordings() {
        let dir = std::path::Path::new("IMPORTS/KIA/more");
        if !dir.is_dir() {
            eprintln!("Skip: IMPORTS not found (run from crate root)");
            return;
        }
        for (file, frames, counters, buttons) in [
            ("KIA_Niro_late_model_433.92_FM.sub", 12, vec![2u16, 3, 4, 5, 6, 7, 8, 9, 10], vec![1u8]),
            ("Kia_K7_433.92_FM.sub", 12, vec![2, 3, 4, 5, 7, 9, 10], vec![1, 2]),
            ("2019-2022_Kia_Sportage_QL__433.92_FM.sub", 16, vec![5, 6, 7, 8, 9, 10, 12, 13, 14], vec![1, 2]),
        ] {
            let (_, wire) = sub_pulses(&dir.join(file));
            let got = feed_resetting(&mut KiaMobisDecoder::new(), &wire);
            let mut c: Vec<u16> = got.iter().map(|d| d.counter.unwrap()).collect();
            c.dedup();
            let mut b: Vec<u8> = got.iter().map(|d| d.button.unwrap()).collect();
            b.dedup();
            eprintln!("MOBIS {file}: {} frames, counters {c:?}, buttons {b:?}", got.len());
            assert_eq!((got.len(), c, b), (frames, counters, buttons), "{file}");
            assert!(got.iter().all(|d| d.extra == Some(0) && d.data >> 40 == u64::from(d.counter.unwrap()) << 4 | u64::from(d.button.unwrap())));
        }
    }
}
