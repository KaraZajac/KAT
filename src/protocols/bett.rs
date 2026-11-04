//! BETT (Berner / Elka / Tedsen / Teletaster) decoder + encoder.
//!
//! Port of doctor-firmware `lib/subghz/protocols/bett.c` (not in Flipper-ARF's `subghz_garage`
//! app's RX groups; Flipper-ARF's `lib/subghz/protocols/bett.c` and the garage copy decode and
//! encode identically — they only drop the DIP-switch display).
//!
//! Air format (433.92 MHz, AM): PWM, te_short 340 µs / te_long 2000 µs, te_delta 150. 18 bits,
//! MSB first, each a HIGH then a LOW: `1` = HIGH 2000 + LOW 340, `0` = HIGH 340 + LOW 2000; the
//! last bit's LOW is extended by `7·te_long` into the frame gap (14 340 / 16 000 µs). The decoder
//! starts on a LOW of `44·te_short` ± `15·te_delta` (12 711..17 209 µs) and classifies each bit by
//! its HIGH (long within `3·te_delta`, short within `te_delta`); a LOW only has to be short or
//! long. The frame is reported on the next gap when exactly 18 bits were read.
//!
//! **C state machine kept**: after reporting, the C stays in its "LOW expected" step, where a
//! HIGH is ignored, so the next frame's first bit is lost (17 bits → back to the gap search); the
//! registry resets every decoder after a report anyway. No checksum: the 18 bits are nine
//! three-state DIP switches (2 bits each: `11` +, `10` 0, `00` −), which the C displays.
//!
//! Encoder (`repeat` = 3, no counter, no button field: `button` is ignored): the 18 bits as above.

use super::g2_common::{decoded, key_data, key_file, near, push, repeated, BitReg};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`): `SubGhzProtocolFlag_433 | AM`, static code (DIP switches).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Berner/Elka/Tedsen/Teletaster",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "BETT";
const TE_SHORT: u32 = 340;
const TE_LONG: u32 = 2000;
const TE_DELTA: u32 = 150;
const MIN_COUNT_BIT: usize = 18;
const REPEAT: usize = 3;
/// The frame gap: `te_short * 44` ± `te_delta * 15`.
const GAP: u32 = TE_SHORT * 44;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    SaveDuration,
    CheckDuration,
}

/// BETT decoder (`SubGhzProtocolDecoderBETT`).
pub struct BettDecoder {
    step: Step,
    reg: BitReg,
}

fn frame(data: u64) -> DecodedSignal {
    let mut d = decoded(data, MIN_COUNT_BIT);
    d.encoder_capable = true;
    d
}

impl BettDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            reg: BitReg::default(),
        }
    }
}

impl Default for BettDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// `subghz_protocol_encoder_bett_get_upload` (one copy).
fn upload(data: u64) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(2 * MIN_COUNT_BIT);
    for i in (1..MIN_COUNT_BIT).rev() {
        let one = (data >> i) & 1 != 0;
        push(&mut out, true, if one { TE_LONG } else { TE_SHORT });
        push(&mut out, false, if one { TE_SHORT } else { TE_LONG });
    }
    if data & 1 != 0 {
        push(&mut out, true, TE_LONG);
        push(&mut out, false, TE_SHORT + TE_LONG * 7);
    } else {
        push(&mut out, true, TE_SHORT);
        push(&mut out, false, TE_LONG + TE_LONG * 7);
    }
    out
}

impl ProtocolDecoder for BettDecoder {
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
                if !level && near(duration, GAP, TE_DELTA * 15) {
                    self.reg.clear();
                    self.step = Step::CheckDuration;
                }
            }
            Step::SaveDuration => {
                if !level {
                    if near(duration, GAP, TE_DELTA * 15) {
                        let result = if self.reg.count as usize == MIN_COUNT_BIT {
                            Some(frame(self.reg.data))
                        } else {
                            self.step = Step::Reset;
                            None
                        };
                        self.reg.clear();
                        return result;
                    } else if near(duration, TE_SHORT, TE_DELTA)
                        || near(duration, TE_LONG, TE_DELTA * 3)
                    {
                        self.step = Step::CheckDuration;
                    } else {
                        self.step = Step::Reset;
                    }
                }
            }
            Step::CheckDuration => {
                if level {
                    if near(duration, TE_LONG, TE_DELTA * 3) {
                        self.reg.add(true);
                        self.step = Step::SaveDuration;
                    } else if near(duration, TE_SHORT, TE_DELTA) {
                        self.reg.add(false);
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

    /// The unit-test key (`bett.sub`).
    const KEY: u64 = 0xB8B8;

    fn decode(wire: &[LevelDuration]) -> Vec<DecodedSignal> {
        feed_all(&mut BettDecoder::new(), wire)
    }

    /// The upload starts with the first bit (no gap); the first copy's own gap opens the second
    /// copy, which is reported at its gap; the third copy then loses its first bit (C quirk).
    #[test]
    fn encode_decode_roundtrip() {
        for key in [KEY, 0x3_FFFF, 0x0_0001, 0x2_AAAA, 0x1_5554] {
            let d = frame(key);
            let tx = BettDecoder::new().encode(&d, 0x1).unwrap();
            let got = decode(&tx);
            assert_eq!(got.len(), 1, "{key:X}");
            assert_eq!(format!("{:?}", got[0]), format!("{d:?}"));
            // With the registry's reset after each report, copy 3 decodes too when a gap leads in.
            let mut rx = vec![LevelDuration::new(false, GAP)];
            rx.extend(tx);
            assert_eq!(
                feed_with_resets(&mut BettDecoder::new(), &rx).len(),
                2,
                "{key:X}"
            );
        }
    }

    #[test]
    fn decodes_a_hand_built_frame_and_rejects_bad_ones() {
        let mut raw = vec![-15_000];
        for i in (0..18).rev() {
            let one = (KEY >> i) & 1 != 0;
            raw.push(if one { 1980 } else { 350 });
            raw.push(if i == 0 {
                -15_000
            } else if one {
                -330
            } else {
                -2010
            });
        }
        let got = decode(&wire(&raw));
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].data, KEY);
        let mut short = raw.clone();
        short.drain(1..3); // 17 bits
        assert!(decode(&wire(&short)).is_empty());
        let mut bad = raw.clone();
        bad[5] = 1000; // a HIGH that is neither short nor long
        assert!(decode(&wire(&bad)).is_empty());
    }

    #[test]
    fn key_file_loads_and_roundtrips() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: BETT\nBit: 18\n\
                    Key: 00 00 00 00 00 00 B8 B8\n";
        let dec = BettDecoder::new();
        let d = dec
            .from_flipper_key(&FlipperKey::parse(text).unwrap())
            .unwrap();
        assert_eq!(
            (d.data, d.data_count_bit, d.encoder_capable),
            (KEY, 18, true)
        );
        assert_key_roundtrip(&dec, &d);
    }

    #[test]
    fn decodes_unit_test_capture() {
        let Some(got) = registry_decodes(&format!("{UNIT_TESTS}/bett_raw.sub")) else {
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
