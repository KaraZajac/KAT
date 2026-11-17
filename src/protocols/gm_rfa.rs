//! GM RFA: the 315 MHz remotes Omron built for GM ("Remote Function Actuator" in their FCC
//! filings; Cadillac, GMC and their sister makes, about 2004 to 2017): an 84-bit Manchester frame
//! behind a long run of zeros — **decode and display only**.
//!
//! **KAT's own decoder**, worked out from two recordings (`IMPORTS/cadillac/more/`, a 2008-2012
//! Cadillac CTS sent in by the user on 2026-11-16, and `IMPORTS/gmc/15_gmc_acadia_rs2_l2_u2_t2_p2.sub`).
//! No reference reads the signal; it is neither Chevrolet V0 nor Opel V1, whose remotes run at the
//! same speed. The name is KAT's.
//!
//! **Air format**: on-off keying, Manchester (carrier then pause is a 1), 117.5 µs per half bit
//! (4255 bit/s) on both fobs. A burst is 51 zero bits, then 84 bits; 31.9 ms. While the button is
//! held a burst goes out every 100.3 ms. The FCC filings `OUC6000066`, `OUC6000223`, `OUC60270`
//! and `OUC60221` show the 32 ms word every 100 ms; `OUC6000223` gives 120 µs pulses in a 240 µs
//! period. The Cadillac CTS remote of the recording is `OUC6000066` (Omron "Remote Function
//! Actuator (RFA) Model D6 000066", filed in 2006 with a "GM Cadillac RFA User Manual"); the FCC
//! ID of the Acadia's remote is not known.
//!
//! **Recording it.** The signal is 8.5 kBaud, more than twice what the Flipper's AM650 preset
//! filters for (3.79 kBaud), and both recordings show it: the carrier pulses come out 10 to 50 µs
//! too long and the pauses as much too short, differently in every burst. The decoder takes each
//! burst's skew out before it counts half bits (see `burst.rs`); it reads 25 of the CTS file's 30
//! bursts and 38 of the Acadia file's 56. The others are broken up beyond that.
//!
//! **Frame (84 bits)**:
//!
//! | Bits | Field | Notes |
//! |---|---|---|
//! | 0..3 | Head | `1101` on both fobs |
//! | 4..19 | Fob | 16 bits, constant per fob (`FA04`, `0178`) |
//! | 20..83 | Code | 64 bits. Every burst of a press has another one (about half the bits differ), but the last two bursts of a press are alike. No button in the clear |
//!
//! **Gate** (no checksum is known): the burst ends with the frame (at most two half bits behind
//! it), the frame begins `1101` behind at least 20 zero bits, and its 84 bits have no Manchester
//! violation.
//!
//! **Fields**: `serial` = the 16 fob bits, `data` = the 64-bit code, `extra` = frame bits 0..19,
//! `data_count_bit` = 84, no button, no counter.

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
    make: "Cadillac/GMC/Chevrolet/Buick",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "GM RFA";
const TE: u32 = 118;
const MIN_ZEROS: usize = 20;
const HEAD: [bool; 4] = [true, true, false, true];
const FRAME_BITS: usize = 84;
const MAX_SPARE: usize = 2;

/// GM RFA decoder: reads a burst when it is over.
pub struct GmRfaDecoder {
    reader: BurstReader,
}

impl GmRfaDecoder {
    pub fn new() -> Self {
        Self { reader: BurstReader::skewed(TE, 2, 1024) }
    }
}

impl Default for GmRfaDecoder {
    fn default() -> Self {
        Self::new()
    }
}

fn signal(frame: &[bool]) -> DecodedSignal {
    DecodedSignal {
        serial: Some(burst::number(&frame[4..20]) as u32),
        button: None,
        counter: None,
        crc_valid: false,
        data: burst::number(&frame[20..]),
        data_count_bit: FRAME_BITS,
        encoder_capable: false,
        extra: Some(burst::number(&frame[..20])),
        protocol_display_name: None,
    }
}

/// The frame of a finished burst, if it is one. On-off keying has one polarity.
fn read(halves: &[bool]) -> Option<DecodedSignal> {
    for phase in 0..2 {
        let bits = burst::bits(halves, phase, false);
        // the frame is the last 84 bits of the burst
        for end in (FRAME_BITS + MIN_ZEROS..=bits.len()).rev() {
            if burst::spare(halves.len(), phase, end) > MAX_SPARE {
                break;
            }
            let at = end - FRAME_BITS;
            if burst::matches(&bits, at, &HEAD) && burst::run_before(&bits, at, false) >= MIN_ZEROS {
                if let Some(frame) = burst::take(&bits, at, FRAME_BITS) {
                    return Some(signal(&frame));
                }
            }
        }
    }
    None
}

impl ProtocolDecoder for GmRfaDecoder {
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

    fn burst_bits(zeros: usize, id: u16, code: u64) -> Vec<bool> {
        let mut b = vec![false; zeros];
        b.extend(HEAD);
        b.extend(burst::pattern(u64::from(id), 16));
        b.extend(burst::pattern(code, 64));
        b
    }

    #[test]
    fn decodes_a_synthetic_burst() {
        for code in [0x0123_4567_89AB_CDEF_u64, 0x0123_4567_89AB_CDEE] {
            let wire = burst::wire(&burst_bits(51, 0xFA04, code), TE, 68_000);
            let got = feed_all(&mut GmRfaDecoder::new(), &wire);
            assert_eq!(got.len(), 1, "{got:?}");
            let d = &got[0];
            assert_eq!((d.serial, d.data, d.extra, d.data_count_bit), (Some(0xFA04), code, Some(0xD_FA04), 84));
            assert!(d.button.is_none() && d.counter.is_none() && !d.encoder_capable);
        }
    }

    #[test]
    fn the_gate() {
        let dec = |bits: &[bool]| feed_all(&mut GmRfaDecoder::new(), &burst::wire(bits, TE, 68_000)).len();
        assert_eq!(dec(&burst_bits(20, 0x0178, 7)), 1, "twenty zeros are enough");
        assert_eq!(dec(&burst_bits(19, 0x0178, 7)), 0);
        let mut head = burst_bits(51, 0x0178, 7);
        head[53] = true; // 1111
        assert_eq!(dec(&head), 0, "another head");
        let mut long = burst_bits(51, 0x0178, 7);
        long.extend([true, false]);
        assert_eq!(dec(&long), 0, "two bits too many");
        // carrier pulses 45 µs too long, pauses 45 µs too short
        let mut wire = burst::wire(&burst_bits(51, 0x0178, 0xB6CC_AF76_9454_A44B), TE, 68_000);
        let last = wire.len() - 1;
        for p in &mut wire[..last] {
            p.duration_us = if p.level { p.duration_us + 45 } else { p.duration_us - 45 };
        }
        let got = feed_all(&mut GmRfaDecoder::new(), &wire);
        assert_eq!(got.iter().map(|d| (d.serial, d.data)).collect::<Vec<_>>(), [(Some(0x0178), 0xB6CC_AF76_9454_A44B)]);
        let mut short = burst_bits(51, 0x0178, 7);
        short.pop();
        assert_eq!(dec(&short), 0, "a bit short");
        // the other polarity is another signal
        let wire = burst::wire(&burst_bits(51, 0x0178, 7), TE, 68_000);
        let flipped: Vec<LevelDuration> = wire.iter().map(|p| LevelDuration::new(!p.level, p.duration_us)).collect();
        assert!(feed_all(&mut GmRfaDecoder::new(), &flipped).is_empty());
        assert!(GmRfaDecoder::new().encode(&DecodedSignal::new(0, 84), 1).is_none());
    }

    #[test]
    fn imports_recordings() {
        for (path, serial, frames, distinct) in [
            ("IMPORTS/cadillac/more/2008-2012_Cadilla_CTS_315mhz_AM650.sub", 0xFA04, 25, 15),
            ("IMPORTS/gmc/15_gmc_acadia_rs2_l2_u2_t2_p2.sub", 0x0178, 38, 29),
        ] {
            let path = std::path::Path::new(path);
            if !path.is_file() {
                eprintln!("Skip: IMPORTS not found (run from crate root)");
                return;
            }
            let (_, wire) = sub_pulses(path);
            let got = feed_resetting(&mut GmRfaDecoder::new(), &wire);
            let mut codes: Vec<u64> = got.iter().map(|d| d.data).collect();
            codes.sort_unstable();
            codes.dedup();
            eprintln!("GMRFA {}: {} frames, {} distinct codes", path.display(), got.len(), codes.len());
            assert!(got.iter().all(|d| d.serial == Some(serial) && d.extra == Some(0xD_0000 | u64::from(serial))), "{}", path.display());
            assert_eq!((got.len(), codes.len()), (frames, distinct), "{}", path.display());
        }
    }
}
