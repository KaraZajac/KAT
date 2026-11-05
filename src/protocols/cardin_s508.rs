//! Cardin S508 (S500 series, FCC ID TXQ508; also the 2-channel S504): 868 MHz rolling-code gate
//! remote — **decode and display only**, as the reference.
//!
//! Reference: RogueMaster `lib/subghz/protocols/cardin_s508.c` (half2me, Unleashed PR #1004; the
//! only firmware that ships it). Its header explains the format, reverse-engineered from FM12K
//! captures: a ~25 ms wake-up burst of long unequal pulses, a run of constant Manchester bits,
//! then the codeword (repeated while the button is held): a 12-bit fixed sync `110011110100` and a
//! 128-bit payload that is an AES-128 ciphertext — every bit changes between presses, there is no
//! clear serial, button or counter. The C has no encoder (its vtable is NULL, flags Decodable |
//! Save, no Add Manually): valid future codes cannot be generated without the receiver's key.
//!
//! **Air format** (FM, 868.35 MHz, ~12 kHz deviation): Manchester with a ~100 µs half-bit. The C
//! classifies pulses with per-level bands measured from a real capture (the 2-FSK slicer is
//! duty-cycle biased: HIGH half-bits ~110–150 µs, LOW ~75–120 µs; full cells ~225 / ~175 µs):
//! a pulse under 40 µs is invalid; HIGH: < 165 short, < 340 long; LOW: < 135 short, < 300 long;
//! anything longer is invalid. An invalid pulse (noise, gap, wake-up burst) re-synchronises.
//!
//! **Decoder** (`subghz_protocol_decoder_cardin_s508_feed`): each valid pulse is a Manchester
//! event (`level ? ShortHigh/LongHigh : ShortLow/LongLow`) for Flipper's toolbox decoder
//! (`manchester_advance`, KAT's [super::common::manchester_advance]); an invalid one resets the
//! sync search, the payload and the Manchester state. Decoded bits slide through a 12-bit window
//! until it reads the sync `0xCF4` or its complement `0x30B` (the toolbox decoder reads Cardin's
//! "1 = high then low" as 0, so real captures lock on the complement); the next 128 bits are the
//! payload, complemented back when the complement locked, and the frame is reported. The search
//! then restarts (the Manchester state carries on) for the next repeat. No checksum exists.
//!
//! **KAT fields**: `data` = payload bits 127..64 (the first 64 wire bits, the C's `data_2`),
//! `extra` = bits 63..0 (the C's `data`), `data_count_bit` = 128; `serial`, `button`, `counter`
//! None and `crc_valid` false (nothing in the clear, nothing to check); `encoder_capable` false
//! (decode-only: KAT offers Replay of the recorded pulses, which a rolling-code receiver rejects
//! once it has seen the code).
//!
//! **Key file** (`serialize` / `deserialize`): `Protocol: Cardin S508`, `Bit: 128`, `Key` = the
//! low 64 bits, `Data` = the high 64 bits (8 hex bytes); loading needs `Bit: 128` and `Data`.

use super::common::{manchester_advance, ManchesterEvent, ManchesterState};
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;
use crate::radio::preset::FM12K;

/// Metadata (see `info.rs`): `SubGhzProtocolFlag_868 | FM`, `SubGhzProtocolTypeDynamic`, read
/// with the FM12K preset (the C's header).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::FM,
    encryption: "AES-128",
    make: "Cardin",
    category: Category::Gate,
    buttons: None,
    presets: &[&FM12K],
};

const NAME: &str = "Cardin S508";
const BIT_COUNT: usize = 128;
const PULSE_MIN: u32 = 40;
const HI_SHORT_MAX: u32 = 165;
const HI_LONG_MAX: u32 = 340;
const LO_SHORT_MAX: u32 = 135;
const LO_LONG_MAX: u32 = 300;
const SYNC: u16 = 0x0CF4;
const SYNC_INV: u16 = 0x030B;
const SYNC_MASK: u16 = 0x0FFF;

/// Cardin S508 decoder (`SubGhzProtocolDecoderCardinS508`).
pub struct CardinS508Decoder {
    state: ManchesterState,
    sync_shift: u16,
    payload_count: u16,
    payload_hi: u64,
    payload_lo: u64,
    sync_locked: bool,
    inverted: bool,
}

/// `subghz_protocol_decoder_cardin_s508_classify`: Some(is_short) for a valid pulse.
fn classify(level: bool, duration: u32) -> Option<bool> {
    if duration < PULSE_MIN {
        return None;
    }
    let (short_max, long_max) = if level {
        (HI_SHORT_MAX, HI_LONG_MAX)
    } else {
        (LO_SHORT_MAX, LO_LONG_MAX)
    };
    if duration < short_max {
        Some(true)
    } else if duration < long_max {
        Some(false)
    } else {
        None
    }
}

fn signal(hi: u64, lo: u64) -> DecodedSignal {
    DecodedSignal {
        serial: None,
        button: None,
        counter: None,
        crc_valid: false,
        data: hi,
        data_count_bit: BIT_COUNT,
        encoder_capable: false,
        extra: Some(lo),
        protocol_display_name: None,
    }
}

impl CardinS508Decoder {
    pub fn new() -> Self {
        let mut d = Self {
            state: ManchesterState::Start1,
            sync_shift: 0,
            payload_count: 0,
            payload_hi: 0,
            payload_lo: 0,
            sync_locked: false,
            inverted: false,
        };
        d.reset_accumulators();
        d
    }

    /// `subghz_protocol_decoder_cardin_s508_reset_accumulators`.
    fn reset_accumulators(&mut self) {
        self.sync_shift = 0;
        self.sync_locked = false;
        self.inverted = false;
        self.payload_hi = 0;
        self.payload_lo = 0;
        self.payload_count = 0;
        self.state = manchester_advance(self.state, ManchesterEvent::Reset).0;
    }

    /// `subghz_protocol_decoder_cardin_s508_add_bit`: the frame when the 128th payload bit is in.
    fn add_bit(&mut self, bit: bool) -> Option<DecodedSignal> {
        if !self.sync_locked {
            self.sync_shift = ((self.sync_shift << 1) | u16::from(bit)) & SYNC_MASK;
            match self.sync_shift {
                SYNC => self.inverted = false,
                SYNC_INV => self.inverted = true,
                _ => return None,
            }
            self.sync_locked = true;
            self.payload_hi = 0;
            self.payload_lo = 0;
            self.payload_count = 0;
            return None;
        }
        self.payload_hi = (self.payload_hi << 1) | (self.payload_lo >> 63);
        self.payload_lo = (self.payload_lo << 1) | u64::from(bit);
        self.payload_count = self.payload_count.wrapping_add(1);
        if usize::from(self.payload_count) < BIT_COUNT {
            return None;
        }
        let (hi, lo) = if self.inverted {
            (!self.payload_hi, !self.payload_lo)
        } else {
            (self.payload_hi, self.payload_lo)
        };
        // Ready for the next repeat of the same press.
        self.sync_locked = false;
        self.sync_shift = 0;
        Some(signal(hi, lo))
    }
}

impl Default for CardinS508Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for CardinS508Decoder {
    fn name(&self) -> &'static str {
        NAME
    }

    /// Nominal timing only (the C classifies with the per-level bands, not these).
    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: 100,
            te_long: 200,
            te_delta: 50,
            min_count_bit: BIT_COUNT,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.reset_accumulators();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let Some(short) = classify(level, duration) else {
            // gap / wake-up burst / noise: re-sync and keep scanning for the sync
            self.reset_accumulators();
            return None;
        };
        let event = ManchesterEvent::from_half_bit(level, !short);
        let (next, bit) = manchester_advance(self.state, event);
        self.state = next;
        bit.and_then(|b| self.add_bit(b))
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != BIT_COUNT {
            return None;
        }
        let data = key.hex_bytes_field("Data")?;
        let hi = data
            .get(..8)?
            .iter()
            .fold(0u64, |acc, &b| (acc << 8) | u64::from(b));
        Some(signal(hi, key.key))
    }

    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != BIT_COUNT {
            return None;
        }
        let mut key = FlipperKey::for_protocol(NAME, BIT_COUNT, decoded.extra?);
        key.push_hex("Data", decoded.data, 8);
        Some(key)
    }
}

/// Synthetic captures for the tests and the fuzz tool.
#[cfg(test)]
pub(super) mod testing {
    use super::*;

    /// Pulses for a press: a 24-bit constant preamble (bits the toolbox decoder reads as 1s),
    /// then the codeword (sync + `hi`‖`lo`) sent `repeats` times, in Cardin's own convention (1 =
    /// HIGH then LOW half-bit, which the toolbox decoder reads as the complement) unless
    /// `flipper_convention`; durations (HIGH short, HIGH long, LOW short, LOW long) as given; a
    /// 5 ms LOW after.
    pub fn press(
        hi: u64,
        lo: u64,
        repeats: usize,
        flipper_convention: bool,
        t: [u32; 4],
    ) -> Vec<LevelDuration> {
        let mut bits: Vec<bool> = vec![flipper_convention; 24];
        for _ in 0..repeats {
            bits.extend((0..12).rev().map(|i| (SYNC >> i) & 1 == 1));
            bits.extend((0..64).rev().map(|i| (hi >> i) & 1 == 1));
            bits.extend((0..64).rev().map(|i| (lo >> i) & 1 == 1));
        }
        pulses(&bits, flipper_convention, t)
    }

    /// Manchester pulses of `bits` (see [press]), then a 5 ms LOW.
    pub fn pulses(bits: &[bool], flipper_convention: bool, t: [u32; 4]) -> Vec<LevelDuration> {
        // Half-bit levels: Cardin 1 = HIGH, LOW; Flipper 1 = LOW, HIGH.
        let mut halves: Vec<bool> = Vec::new();
        for &b in bits {
            let first_high = b != flipper_convention;
            halves.extend([first_high, !first_high]);
        }
        let mut out: Vec<LevelDuration> = Vec::new();
        let mut i = 0;
        while i < halves.len() {
            let level = halves[i];
            let run = if halves.get(i + 1) == Some(&level) {
                2
            } else {
                1
            };
            let d = match (level, run) {
                (true, 1) => t[0],
                (true, _) => t[1],
                (false, 1) => t[2],
                (false, _) => t[3],
            };
            out.push(LevelDuration::new(level, d));
            i += run;
        }
        out.push(LevelDuration::new(false, 5_000));
        out
    }
}

#[cfg(test)]
mod tests {
    use super::super::f_common::test_support::*;
    use super::*;

    const SYMMETRIC: [u32; 4] = [100, 200, 100, 200];
    /// The duty-cycle-biased timing the C's header describes for a real FM12K capture.
    const BIASED: [u32; 4] = [140, 225, 95, 175];

    #[test]
    fn decodes_both_conventions_and_timings() {
        let (hi, lo) = (0x0123_4567_89AB_CDEF, 0xFEDC_BA98_7654_3210);
        for flipper in [false, true] {
            for t in [SYMMETRIC, BIASED] {
                let got = feed_all(
                    &mut CardinS508Decoder::new(),
                    &testing::press(hi, lo, 3, flipper, t),
                );
                assert_eq!(got.len(), 3, "flipper {flipper} {t:?}");
                assert!(got.iter().all(|d| *d == signal(hi, lo)), "{got:?}");
            }
        }
    }

    #[test]
    fn classify_bands_match_the_c() {
        assert_eq!(classify(true, 39), None);
        assert_eq!(classify(true, 40), Some(true));
        assert_eq!(
            (classify(true, 164), classify(true, 165)),
            (Some(true), Some(false))
        );
        assert_eq!(
            (classify(true, 339), classify(true, 340)),
            (Some(false), None)
        );
        assert_eq!(
            (classify(false, 134), classify(false, 135)),
            (Some(true), Some(false))
        );
        assert_eq!(
            (classify(false, 299), classify(false, 300)),
            (Some(false), None)
        );
    }

    #[test]
    fn an_invalid_pulse_or_a_missing_sync_drops_the_frame() {
        let mut wire = testing::press(1, 2, 1, false, BIASED);
        let mid = wire.len() / 2;
        wire[mid].duration_us = 600;
        assert!(feed_all(&mut CardinS508Decoder::new(), &wire).is_empty());
        // The same bits without the sync: nothing.
        let mut bits = vec![true; 24];
        bits.extend((0..140).map(|i| i % 3 == 0));
        assert!(feed_all(
            &mut CardinS508Decoder::new(),
            &testing::pulses(&bits, true, SYMMETRIC)
        )
        .is_empty());
        // Decode-only, as the C.
        let d = feed_all(
            &mut CardinS508Decoder::new(),
            &testing::press(1, 2, 1, false, BIASED),
        );
        assert_eq!(d.len(), 1);
        assert!(!d[0].encoder_capable && !d[0].crc_valid && d[0].serial.is_none());
        assert!(CardinS508Decoder::new().encode(&d[0], 0).is_none());
    }

    #[test]
    fn key_file_round_trip() {
        let dec = CardinS508Decoder::new();
        let d = signal(0x1122_3344_5566_7788, 0x99AA_BBCC_DDEE_FF00);
        let text = super::super::flipper_key::testing::assert_round_trip(&dec, &d);
        assert!(
            text.contains("Protocol: Cardin S508\nBit: 128\nKey: 99 AA BB CC DD EE FF 00\nData: 11 22 33 44 55 66 77 88\n"),
            "{text}"
        );
        let mut no_data = FlipperKey::parse(&text).unwrap();
        no_data.fields.clear();
        assert!(dec.from_flipper_key(&no_data).is_none(), "the C needs Data");
        let mut wrong = FlipperKey::parse(&text).unwrap();
        wrong.bits = 64;
        assert!(dec.from_flipper_key(&wrong).is_none());
    }
}
