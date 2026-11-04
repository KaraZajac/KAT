//! Nice FLO (12/24-bit fixed code) decoder + encoder.
//!
//! Port of Flipper-ARF's garage app `applications/system/subghz_garage/protocols/nice_flo.c` (the
//! primary reference; same wire behaviour as ARF's lib copy). doctor-firmware's copy uses a wider
//! `te_delta` (250 instead of 200 µs) and a header window of 25200 ± 7250 µs (ARF: ± 7200); KAT
//! follows ARF.
//!
//! **Air format** (AM, 315/433.92 MHz): `te_short` 700, `te_long` 1400, `te_delta` 200. Header LOW
//! 36·te (25200 µs), start HIGH te, then MSB first: bit 1 = LOW 1400 + HIGH 700, bit 0 = LOW 700 +
//! HIGH 1400.
//!
//! **Decoder** (`subghz_protocol_decoder_nice_flo_feed`): header LOW within 25200 ± 7200 µs, start
//! HIGH |d − 700| < 200, LOW/HIGH pairs, and a LOW ≥ 2800 µs ends the frame, reported when it holds
//! at least 12 bits (the C sets no upper bound; `decode_count_bit` is a `uint8_t` and `data` keeps
//! the last 64 bits, as here). No fields.
//!
//! **Encoder** (`subghz_protocol_encoder_nice_flo_get_upload`, `encoder.repeat` = 3): header, start
//! bit, the key's bits, for 12..=24-bit keys (the C's `deserialize` rejects others, so longer
//! decodes are not transmittable). `button` is ignored (fixed code; ARF re-sends the capture).
//!
//! **Key file**: `Protocol: Nice FLO`, `Bit`, `Key`; loading needs 12..=24 bits, as the C.

use super::g1_common::{fixed_code, generic_fields, generic_key, push, with_repeats, TX_REPEAT};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Nice FLO",
    decoder: "Nice FLO",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Nice",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "Nice FLO";
const TE_SHORT: u32 = 700;
const TE_LONG: u32 = 1400;
const TE_DELTA: u32 = 200;
const MIN_COUNT_BIT: usize = 12;
const MAX_KEY_BIT: usize = 2 * MIN_COUNT_BIT;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    FoundStartBit,
    SaveDuration,
    CheckDuration,
}

/// Nice FLO decoder (`SubGhzProtocolDecoderNiceFlo`).
pub struct NiceFloDecoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
}

fn decoded(data: u64, bits: usize) -> DecodedSignal {
    DecodedSignal {
        encoder_capable: (MIN_COUNT_BIT..=MAX_KEY_BIT).contains(&bits),
        ..fixed_code(data, bits)
    }
}

/// One `get_upload` frame.
fn build_frame(data: u64, bits: usize) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(bits * 2 + 2);
    push(&mut out, false, TE_SHORT * 36);
    push(&mut out, true, TE_SHORT);
    for i in (0..bits).rev() {
        if (data >> i) & 1 == 1 {
            push(&mut out, false, TE_LONG);
            push(&mut out, true, TE_SHORT);
        } else {
            push(&mut out, false, TE_SHORT);
            push(&mut out, true, TE_LONG);
        }
    }
    out
}

impl NiceFloDecoder {
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

impl Default for NiceFloDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for NiceFloDecoder {
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
                if !level && duration_diff!(duration, TE_SHORT * 36) < TE_DELTA * 36 {
                    self.step = Step::FoundStartBit;
                }
            }
            Step::FoundStartBit => {
                if !level {
                    // stay
                } else if duration_diff!(duration, TE_SHORT) < TE_DELTA {
                    self.step = Step::SaveDuration;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::SaveDuration => {
                if !level {
                    if duration >= TE_SHORT * 4 {
                        self.step = Step::FoundStartBit;
                        if self.decode_count_bit as usize >= MIN_COUNT_BIT {
                            // callback → the app resets the receiver.
                            self.step = Step::Reset;
                            return Some(decoded(self.decode_data, self.decode_count_bit as usize));
                        }
                        return None;
                    }
                    self.te_last = duration;
                    self.step = Step::CheckDuration;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::CheckDuration => {
                if level {
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
        let bits = decoded.data_count_bit;
        if !(MIN_COUNT_BIT..=MAX_KEY_BIT).contains(&bits) {
            return None;
        }
        Some(with_repeats(&build_frame(decoded.data, bits), TX_REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        let (bits, data) = generic_fields(key);
        (MIN_COUNT_BIT..=MAX_KEY_BIT)
            .contains(&bits)
            .then(|| decoded(data, bits))
    }

    fn to_flipper_key(&self, d: &DecodedSignal) -> Option<FlipperKey> {
        Some(generic_key(NAME, d))
    }
}

#[cfg(test)]
mod tests {
    use super::super::g1_common::test_support::*;
    use super::*;

    fn on_air(d: &DecodedSignal) -> Vec<LevelDuration> {
        let mut wire = NiceFloDecoder::new().encode(d, 0).unwrap();
        push(&mut wire, false, 30_000);
        wire
    }

    #[test]
    fn encode_decode_roundtrip() {
        for (data, bits) in [(0xA5Cu64, 12), (0x02FEC4, 24), (0x1234, 16)] {
            let got = feed_all(&mut NiceFloDecoder::new(), &on_air(&decoded(data, bits)));
            assert_eq!(
                got.len(),
                2,
                "{bits} bits: the reset after a decode costs the next repeat"
            );
            assert!(got
                .iter()
                .all(|d| (d.data, d.data_count_bit) == (data, bits) && d.encoder_capable));
        }
        assert!(NiceFloDecoder::new().encode(&decoded(0, 25), 0).is_none());
    }

    #[test]
    fn longer_frames_decode_but_do_not_transmit() {
        let mut wire = build_frame(0x1_2345_6789, 33);
        push(&mut wire, false, 30_000);
        let got = feed_all(&mut NiceFloDecoder::new(), &wire);
        assert_eq!(
            (got[0].data, got[0].data_count_bit, got[0].encoder_capable),
            (0x1_2345_6789, 33, false)
        );
    }

    #[test]
    fn rejects_short_frames_and_bad_timing() {
        let mut wire = build_frame(0x3FF, 11);
        push(&mut wire, false, 30_000);
        assert!(
            feed_all(&mut NiceFloDecoder::new(), &wire).is_empty(),
            "11 bits"
        );
        // 950 µs is neither short (700 ± 200) nor long (1400 ± 200).
        let bad: Vec<LevelDuration> = on_air(&decoded(0x02FEC4, 24))
            .into_iter()
            .map(|p| {
                if p.duration_us == TE_LONG {
                    LevelDuration::new(p.level, 950)
                } else {
                    p
                }
            })
            .collect();
        assert!(feed_all(&mut NiceFloDecoder::new(), &bad).is_empty());
    }

    #[test]
    fn unit_test_key_file() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Nice FLO\nBit: 24\n\
                    Key: 00 00 00 00 00 02 FE C4\n";
        let dec = NiceFloDecoder::new();
        let d = dec
            .from_flipper_key(&FlipperKey::parse(text).unwrap())
            .unwrap();
        assert_eq!((d.data, d.data_count_bit), (0x02FEC4, 24));
        assert_eq!(
            feed_all(&mut NiceFloDecoder::new(), &on_air(&d))[0].data,
            0x02FEC4
        );
        let written = dec.to_flipper_key(&d).unwrap().to_text();
        let back = dec
            .from_flipper_key(&FlipperKey::parse(&written).unwrap())
            .unwrap();
        assert_eq!(format!("{back:?}"), format!("{d:?}"));
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&text.replace("Bit: 24", "Bit: 11")).unwrap())
            .is_none());
    }

    #[test]
    fn reference_raw_capture_decodes() {
        let Some(decodes) = decode_reference_capture("nice_flo_raw.sub") else {
            return;
        };
        assert!(
            decodes.iter().any(|(n, _)| n == NAME),
            "{:?}",
            decodes.iter().map(|(n, _)| n).collect::<Vec<_>>()
        );
    }
}
