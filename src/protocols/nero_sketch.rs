//! Nero Sketch decoder + encoder.
//!
//! Port of doctor-firmware `lib/subghz/protocols/nero_sketch.c` (not in Flipper-ARF's
//! `subghz_garage` RX groups; Flipper-ARF's lib copy and the garage copy decode and encode
//! identically).
//!
//! Air format (433.92 MHz, AM): PWM, te_short 330 µs / te_long 660 µs, te_delta 150. A frame is a
//! preamble of te_short HIGH/LOW pairs (the encoder sends 47; the decoder needs more than 40), a
//! start pulse HIGH `4·te_short` = 1320 µs + LOW te_short, then 40 bits, MSB first, each a HIGH
//! then a LOW: `1` = HIGH 660 + LOW 330, `0` = HIGH 330 + LOW 660 (all within `te_delta`), and a
//! stop pulse HIGH `3·te_short` = 990 µs + LOW te_short. A HIGH ≥ 960 µs (`2·te_short +
//! 2·te_delta`) is the stop pulse: the frame is reported when exactly 40 bits were read. No fields
//! (the C shows the key and its bit-reversal, "Yek"), no checksum.
//!
//! Encoder (`repeat` = 3, no counter, no button: `button` is ignored): preamble, start pulse, the
//! 40 bits, stop pulse.

use super::g2_common::{decoded, key_data, key_file, near, push, repeated, BitReg};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`): `SubGhzProtocolFlag_433 | AM`, static code.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Nero",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "Nero Sketch";
const TE_SHORT: u32 = 330;
const TE_LONG: u32 = 660;
const TE_DELTA: u32 = 150;
const MIN_COUNT_BIT: usize = 40;
const REPEAT: usize = 3;
const PREAMBLE_PAIRS: usize = 47;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    CheckPreambula,
    SaveDuration,
    CheckDuration,
}

/// Nero Sketch decoder (`SubGhzProtocolDecoderNeroSketch`).
pub struct NeroSketchDecoder {
    step: Step,
    te_last: u32,
    header_count: u16,
    reg: BitReg,
}

fn frame(data: u64) -> DecodedSignal {
    let mut d = decoded(data, MIN_COUNT_BIT);
    d.encoder_capable = true;
    d
}

impl NeroSketchDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            header_count: 0,
            reg: BitReg::default(),
        }
    }
}

impl Default for NeroSketchDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// `subghz_protocol_encoder_nero_sketch_get_upload` (one copy).
fn upload(data: u64) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(2 * (PREAMBLE_PAIRS + MIN_COUNT_BIT + 2));
    for _ in 0..PREAMBLE_PAIRS {
        push(&mut out, true, TE_SHORT);
        push(&mut out, false, TE_SHORT);
    }
    push(&mut out, true, TE_SHORT * 4);
    push(&mut out, false, TE_SHORT);
    for i in (0..MIN_COUNT_BIT).rev() {
        let one = (data >> i) & 1 != 0;
        push(&mut out, true, if one { TE_LONG } else { TE_SHORT });
        push(&mut out, false, if one { TE_SHORT } else { TE_LONG });
    }
    push(&mut out, true, TE_SHORT * 3);
    push(&mut out, false, TE_SHORT);
    out
}

impl ProtocolDecoder for NeroSketchDecoder {
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
                if level && near(duration, TE_SHORT, TE_DELTA) {
                    self.step = Step::CheckPreambula;
                    self.te_last = duration;
                    self.header_count = 0;
                }
            }
            Step::CheckPreambula => {
                if level {
                    if near(duration, TE_SHORT, TE_DELTA) || near(duration, TE_SHORT * 4, TE_DELTA)
                    {
                        self.te_last = duration;
                    } else {
                        self.step = Step::Reset;
                    }
                } else if near(duration, TE_SHORT, TE_DELTA) {
                    if near(self.te_last, TE_SHORT, TE_DELTA) {
                        self.header_count = self.header_count.wrapping_add(1);
                    } else if near(self.te_last, TE_SHORT * 4, TE_DELTA) {
                        if self.header_count > 40 {
                            self.step = Step::SaveDuration;
                            self.reg.clear();
                        } else {
                            self.step = Step::Reset;
                        }
                    } else {
                        self.step = Step::Reset;
                    }
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::SaveDuration => {
                if level {
                    if duration >= TE_SHORT * 2 + TE_DELTA * 2 {
                        self.step = Step::Reset;
                        let result = (self.reg.count as usize == MIN_COUNT_BIT)
                            .then(|| frame(self.reg.data));
                        self.reg.clear();
                        return result;
                    }
                    self.te_last = duration;
                    self.step = Step::CheckDuration;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::CheckDuration => {
                if !level {
                    if near(self.te_last, TE_SHORT, TE_DELTA) && near(duration, TE_LONG, TE_DELTA) {
                        self.reg.add(false);
                        self.step = Step::SaveDuration;
                    } else if near(self.te_last, TE_LONG, TE_DELTA)
                        && near(duration, TE_SHORT, TE_DELTA)
                    {
                        self.reg.add(true);
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
        (decoded.data_count_bit == MIN_COUNT_BIT).then(|| repeated(&upload(decoded.data), REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        let (data, _) = key_data(key, |bits| bits == MIN_COUNT_BIT)?;
        Some(frame(data))
    }

    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        Some(key_file(NAME, decoded))
    }
}

#[cfg(test)]
mod tests {
    use super::super::g2_common::test_util::*;
    use super::*;

    const KEY: u64 = 0xF0_1234_5678;

    #[test]
    fn encode_decode_roundtrip() {
        for key in [KEY, 0, 0xFF_FFFF_FFFF, 0xAA_AAAA_AAAA] {
            let d = frame(key);
            let tx = NeroSketchDecoder::new().encode(&d, 0x1).unwrap();
            let got = feed_all(&mut NeroSketchDecoder::new(), &tx);
            assert_eq!(got.len(), REPEAT, "{key:X}");
            for g in &got {
                assert_eq!(format!("{g:?}"), format!("{d:?}"));
            }
            assert_eq!(
                feed_with_resets(&mut NeroSketchDecoder::new(), &tx).len(),
                REPEAT
            );
        }
    }

    #[test]
    fn needs_the_preamble_and_exactly_40_bits() {
        // 40 preamble pairs are not enough (the C wants header_count > 40).
        let tx = upload(KEY);
        let few: Vec<LevelDuration> = tx[2 * (PREAMBLE_PAIRS - 40)..].to_vec();
        assert!(feed_all(&mut NeroSketchDecoder::new(), &few).is_empty());
        let ok: Vec<LevelDuration> = tx[2 * (PREAMBLE_PAIRS - 42)..].to_vec();
        assert_eq!(feed_all(&mut NeroSketchDecoder::new(), &ok).len(), 1);
        // 39 bits.
        let mut short = tx.clone();
        short.drain(2 * PREAMBLE_PAIRS + 2..2 * PREAMBLE_PAIRS + 4);
        assert!(feed_all(&mut NeroSketchDecoder::new(), &short).is_empty());
    }

    #[test]
    fn key_file_roundtrips() {
        let dec = NeroSketchDecoder::new();
        let d = frame(KEY);
        assert_key_roundtrip(&dec, &d);
        let text = dec.to_flipper_key(&d).unwrap().to_text();
        assert!(
            text.contains("Protocol: Nero Sketch\nBit: 40\nKey: 00 00 00 F0 12 34 56 78\n"),
            "{text}"
        );
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&text.replace("Bit: 40", "Bit: 41")).unwrap())
            .is_none());
    }

    #[test]
    fn decodes_unit_test_capture() {
        let Some(got) = registry_decodes(&format!("{UNIT_TESTS}/nero_sketch_raw.sub")) else {
            return;
        };
        let m: Vec<_> = got.iter().filter(|(n, _)| n == NAME).collect();
        assert!(
            !m.is_empty(),
            "{:?}",
            got.iter().map(|(n, _)| n).collect::<Vec<_>>()
        );
    }
}
