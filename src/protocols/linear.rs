//! Linear (10-bit DIP-switch fixed code, MultiCode/Linear 300 MHz-style) decoder + encoder.
//!
//! Port of Flipper-ARF's garage app `applications/system/subghz_garage/protocols/linear.c` (the
//! primary reference; doctor-firmware's copy has the same wire behaviour, ARF flags it 315 MHz only,
//! doctor 315 + 433.92).
//!
//! **Air format** (AM): `te_short` 500, `te_long` 1500, `te_delta` 350; MSB first, bit 1 = HIGH
//! 1500 and LOW 500, bit 0 = HIGH 500 and LOW 1500; the last bit's LOW is the guard (42·te = 21000 µs after a
//! 1, 44·te = 22000 µs after a 0). 10 bits.
//!
//! **Decoder** (`subghz_protocol_decoder_linear_feed`): a LOW within 21000 ± 5250 µs starts a frame;
//! HIGH/LOW pairs; a LOW ≥ 2500 µs ends it — it must itself be a guard (21000 ± 5250 µs, else the
//! frame is dropped), the last bit comes from its HIGH alone, and exactly 10 bits are reported. The
//! decoder then needs a new guard, so a frame right after a decode is skipped. No fields (the C
//! displays the inverted key as the DIP switches).
//!
//! **Encoder** (`subghz_protocol_encoder_linear_get_upload`, `encoder.repeat` = 3): the 10 bits with
//! the guard folded into the last bit's LOW. Fixed code: `button` is ignored.
//!
//! **Key file**: `Protocol: Linear`, `Bit: 10`, `Key`.

use super::g1_common::{fixed_code, generic_fields, generic_key, push, with_repeats, TX_REPEAT};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Linear",
    decoder: "Linear",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Linear",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "Linear";
const TE_SHORT: u32 = 500;
const TE_LONG: u32 = 1500;
const TE_DELTA: u32 = 350;
const MIN_COUNT_BIT: usize = 10;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    SaveDuration,
    CheckDuration,
}

/// Linear decoder (`SubGhzProtocolDecoderLinear`).
pub struct LinearDecoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
}

/// One `get_upload` frame.
fn build_frame(data: u64, bits: usize) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(bits * 2);
    for i in (1..bits).rev() {
        if (data >> i) & 1 == 1 {
            push(&mut out, true, TE_LONG);
            push(&mut out, false, TE_SHORT);
        } else {
            push(&mut out, true, TE_SHORT);
            push(&mut out, false, TE_LONG);
        }
    }
    if data & 1 == 1 {
        push(&mut out, true, TE_LONG);
        push(&mut out, false, TE_SHORT * 42);
    } else {
        push(&mut out, true, TE_SHORT);
        push(&mut out, false, TE_SHORT * 44);
    }
    out
}

impl LinearDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
        }
    }

    fn add_bit(&mut self, bit: u64) {
        self.decode_data = self.decode_data << 1 | bit;
        self.decode_count_bit = self.decode_count_bit.wrapping_add(1);
    }
}

impl Default for LinearDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for LinearDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: MIN_COUNT_BIT,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.step = Step::Reset;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                if !level && duration_diff!(duration, TE_SHORT * 42) < TE_DELTA * 15 {
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.step = Step::SaveDuration;
                }
            }
            Step::SaveDuration => {
                if level {
                    self.te_last = duration;
                    self.step = Step::CheckDuration;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::CheckDuration => {
                if level {
                    self.step = Step::Reset;
                    return None;
                }
                if duration >= TE_SHORT * 5 {
                    self.step = Step::Reset;
                    // The end LOW must be a guard time.
                    if duration_diff!(duration, TE_SHORT * 42) > TE_DELTA * 15 {
                        return None;
                    }
                    if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA {
                        self.add_bit(0);
                    } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA {
                        self.add_bit(1);
                    }
                    if self.decode_count_bit as usize == MIN_COUNT_BIT {
                        return Some(fixed_code(self.decode_data, MIN_COUNT_BIT));
                    }
                    return None;
                }
                if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA
                    && duration_diff!(duration, TE_LONG) < TE_DELTA
                {
                    self.add_bit(0);
                    self.step = Step::SaveDuration;
                } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA
                    && duration_diff!(duration, TE_SHORT) < TE_DELTA
                {
                    self.add_bit(1);
                    self.step = Step::SaveDuration;
                } else {
                    self.step = Step::Reset;
                }
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        Some(with_repeats(
            &build_frame(decoded.data, MIN_COUNT_BIT),
            TX_REPEAT,
        ))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        let (bits, data) = generic_fields(key);
        (bits == MIN_COUNT_BIT).then(|| fixed_code(data, bits))
    }

    fn to_flipper_key(&self, d: &DecodedSignal) -> Option<FlipperKey> {
        Some(generic_key(NAME, d))
    }
}

#[cfg(test)]
mod tests {
    use super::super::g1_common::test_support::*;
    use super::*;

    #[test]
    fn encode_decode_roundtrip() {
        for data in [0x1E4u64, 0x001, 0x3FE, 0x2AA] {
            let wire = LinearDecoder::new()
                .encode(&fixed_code(data, 10), 0)
                .unwrap();
            assert_eq!(wire.len(), 3 * 20);
            // Frame 1's guard starts the decoder, frame 2 decodes, its guard was consumed so
            // frame 3 is skipped.
            let got = feed_all(&mut LinearDecoder::new(), &wire);
            assert_eq!(
                got.iter().map(|d| d.data).collect::<Vec<_>>(),
                [data],
                "{data:03X}"
            );
            // Twice as many repeats: frames 2, 4 and 6 (each decode costs the next frame).
            let twice = with_repeats(&wire, 2);
            assert_eq!(feed_all(&mut LinearDecoder::new(), &twice).len(), 3);
        }
    }

    #[test]
    fn rejects_bad_guard_and_length() {
        let mut frame = build_frame(0x1E4, 10);
        // A 12 ms end LOW is not a guard (21000 ± 5250).
        let wire: Vec<LevelDuration> = std::iter::once(LevelDuration::new(false, 21_000))
            .chain(frame.iter().take(19).copied())
            .chain(std::iter::once(LevelDuration::new(false, 12_000)))
            .collect();
        assert!(feed_all(&mut LinearDecoder::new(), &wire).is_empty());
        // 9 bits.
        frame.drain(..2);
        let mut nine = vec![LevelDuration::new(false, 21_000)];
        nine.extend(frame);
        assert!(feed_all(&mut LinearDecoder::new(), &nine).is_empty());
    }

    #[test]
    fn unit_test_key_file() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Linear\nBit: 10\n\
                    Key: 00 00 00 00 00 00 01 E4\n";
        let dec = LinearDecoder::new();
        let d = dec
            .from_flipper_key(&FlipperKey::parse(text).unwrap())
            .unwrap();
        assert_eq!(d.data, 0x1E4);
        assert_eq!(
            feed_all(&mut LinearDecoder::new(), &dec.encode(&d, 0).unwrap())[0].data,
            0x1E4
        );
        let written = dec.to_flipper_key(&d).unwrap().to_text();
        let back = dec
            .from_flipper_key(&FlipperKey::parse(&written).unwrap())
            .unwrap();
        assert_eq!(format!("{back:?}"), format!("{d:?}"));
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&text.replace("Bit: 10", "Bit: 9")).unwrap())
            .is_none());
    }

    #[test]
    fn reference_raw_capture_decodes() {
        let Some(decodes) = decode_reference_capture("linear_raw.sub") else {
            return;
        };
        assert!(
            decodes.iter().any(|(n, _)| n == NAME),
            "{:?}",
            decodes.iter().map(|(n, _)| n).collect::<Vec<_>>()
        );
    }
}
