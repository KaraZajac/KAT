//! Kia Denso: the smart keys DENSO Korea builds for Kia (EV6, K5, newer Forte and Niro by the parts
//! catalogues), 2019 on: a 112-bit Manchester frame sent five times back to back — **decode and
//! display only**.
//!
//! **KAT's own decoder**, worked out from two recordings sent in by the user on 2026-11-16
//! (`IMPORTS/KIA/more/22-26_Kia_EV6_433.92mhz_FM476.sub` and
//! `Random_late_model_KIA_remote_433.92mhz_FM476.sub`, ten presses each). No reference reads the
//! signal; the name is KAT's.
//!
//! **Air format**: 2-FSK, Manchester, 174.8 µs per half bit (2860 bit/s) on both fobs. The frame is
//! 112 bits (39.2 ms) and a press sends it again and again without a gap: the FCC filings of
//! `CQOFD00790`, `CQOFD01340` and `CQOFN01050` say "automatically off within 0.196 seconds", five
//! frames; the two tones are 60 kHz apart.
//!
//! **FCC filings**: the EV6's remote is `CQOFN01050` (DENSO Korea "Smart Key FOB", four buttons:
//! lock, unlock, panic, remote start). `CQOFD00790` has five (trunk as well) and `CQOFD01340`
//! seven (forward and backward as well); parts catalogues list all three for the EV6.
//!
//! **Frame (112 bits)**:
//!
//! | Bits | Field | Notes |
//! |---|---|---|
//! | 0..19 | Lead | 9 ones and `00100110101` (`0xFF935`), the same on both fobs |
//! | 20..47 | Fob | 28 bits, constant per fob (`9AC9D77`, `7B74E67`) |
//! | 48..57 | ? | `1000111110` on both fobs in every press; the button may be in here (the buttons pressed are not known) |
//! | 58..110 | Code | changes with every press |
//! | 111 | 0 | in every frame seen |
//!
//! **Gate** (no checksum is known): the 20-bit lead, 112 bits without one Manchester violation,
//! and the same 112 bits again right behind them. A press of which only one whole copy was caught
//! is not read. Either polarity of the two tones is read.
//!
//! **Fields**: `serial` = the 28 fob bits, `data` = frame bits 48..111, `extra` = frame bits 0..47,
//! `data_count_bit` = 112, no button, no counter.

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
    make: "Kia/Hyundai",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&FM476],
};

const NAME: &str = "Kia Denso";
const TE: u32 = 175;
const TOL: u32 = 55;
const LEAD: u64 = 0xF_F935;
const LEAD_BITS: usize = 20;
const FRAME_BITS: usize = 112;

/// Kia Denso decoder: reads a burst when it is over.
pub struct KiaDensoDecoder {
    reader: BurstReader,
}

impl KiaDensoDecoder {
    pub fn new() -> Self {
        Self { reader: BurstReader::new(TE, TOL, 4096) }
    }
}

impl Default for KiaDensoDecoder {
    fn default() -> Self {
        Self::new()
    }
}

fn signal(frame: &[bool]) -> DecodedSignal {
    DecodedSignal {
        serial: Some(burst::number(&frame[20..48]) as u32),
        button: None,
        counter: None,
        crc_valid: false,
        data: burst::number(&frame[48..]),
        data_count_bit: FRAME_BITS,
        encoder_capable: false,
        extra: Some(burst::number(&frame[..48])),
        protocol_display_name: None,
    }
}

/// The frame a finished burst repeats, if it repeats one.
fn read(halves: &[bool]) -> Option<DecodedSignal> {
    let lead = burst::pattern(LEAD, LEAD_BITS);
    for phase in 0..2 {
        for inverted in [false, true] {
            let bits = burst::bits(halves, phase, inverted);
            let Some(last) = bits.len().checked_sub(2 * FRAME_BITS) else { continue };
            for at in 0..=last {
                if !burst::matches(&bits, at, &lead) || !burst::matches(&bits, at + FRAME_BITS, &lead) {
                    continue;
                }
                let (Some(a), Some(b)) = (burst::take(&bits, at, FRAME_BITS), burst::take(&bits, at + FRAME_BITS, FRAME_BITS))
                else {
                    continue;
                };
                if a == b {
                    return Some(signal(&a));
                }
            }
        }
    }
    None
}

impl ProtocolDecoder for KiaDensoDecoder {
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

    /// The EV6's lead, fob bits and fixed bits with a made-up code.
    fn frame() -> Vec<bool> {
        let mut b = burst::pattern(0xFF_9359_AC9D_778F, 56);
        b.extend(burst::pattern(0x9A_5C3E_0172_B4D6 & !1, 56));
        b
    }

    #[test]
    fn decodes_two_copies_at_either_polarity() {
        let two: Vec<bool> = [frame(), frame()].concat();
        let wire = burst::wire(&two, TE, 100_000);
        let got = feed_all(&mut KiaDensoDecoder::new(), &wire);
        assert_eq!(got.len(), 1, "{got:?}");
        let d = &got[0];
        assert_eq!((d.serial, d.button, d.counter), (Some(0x9AC_9D77), None, None));
        assert_eq!((d.extra, d.data, d.data_count_bit), (Some(0xFF93_59AC_9D77), 0x8F9A_5C3E_0172_B4D6, 112));
        let flipped: Vec<LevelDuration> = wire.iter().map(|p| LevelDuration::new(!p.level, p.duration_us)).collect();
        assert_eq!(feed_all(&mut KiaDensoDecoder::new(), &flipped).len(), 1);
        // five copies are still one decode per burst
        let five: Vec<bool> = std::iter::repeat_with(frame).take(5).flatten().collect();
        assert_eq!(feed_all(&mut KiaDensoDecoder::new(), &burst::wire(&five, TE, 100_000)).len(), 1);
    }

    #[test]
    fn the_gate() {
        let dec = |bits: &[bool]| feed_all(&mut KiaDensoDecoder::new(), &burst::wire(bits, TE, 100_000)).len();
        assert_eq!(dec(&frame()), 0, "one copy is not enough");
        let mut other = frame();
        other[100] = !other[100];
        assert_eq!(dec(&[frame(), other].concat()), 0, "the copies differ");
        let mut lead = frame();
        lead[12] = !lead[12];
        assert_eq!(dec(&[lead.clone(), lead].concat()), 0, "another lead");
        assert!(KiaDensoDecoder::new().encode(&DecodedSignal::new(0, 112), 1).is_none());
    }

    #[test]
    fn imports_recordings() {
        let dir = std::path::Path::new("IMPORTS/KIA/more");
        if !dir.is_dir() {
            eprintln!("Skip: IMPORTS not found (run from crate root)");
            return;
        }
        for (file, serial, frames) in
            [("22-26_Kia_EV6_433.92mhz_FM476.sub", 0x9AC_9D77, 10), ("Random_late_model_KIA_remote_433.92mhz_FM476.sub", 0x7B7_4E67, 10)]
        {
            let (_, wire) = sub_pulses(&dir.join(file));
            let got = feed_resetting(&mut KiaDensoDecoder::new(), &wire);
            let mut codes: Vec<u64> = got.iter().map(|d| d.data).collect();
            codes.dedup();
            eprintln!("DENSO {file}: {} frames, {} distinct", got.len(), codes.len());
            assert!(got.iter().all(|d| d.serial == Some(serial) && d.data >> 54 == 0b10_0011_1110 && d.data & 1 == 0), "{file}");
            assert_eq!((got.len(), codes.len()), (frames, frames), "{file}");
        }
    }
}
