//! Elplast (Elplast / P-11B / 3BK / E.C.A.) decoder + encoder.
//!
//! Port of doctor-firmware `lib/subghz/protocols/elplast.c` (@xMasterX 2025; not in Flipper-ARF's
//! `subghz_garage` RX groups; Flipper-ARF's lib copy and the garage copy decode and encode
//! identically). The Flipper unit-test files are Unleashed's `elplast_raw.sub` / `elplast.sub`.
//!
//! Air format (433.92 MHz, AM): PWM, te_short 230 µs / te_long 1550 µs, te_delta 160. 18 bits, MSB
//! first, each a HIGH then a LOW: `1` = HIGH 1550 + LOW 230, `0` = HIGH 230 + LOW 1550 (all within
//! `te_delta`); the last bit's LOW is the frame gap `8·te_long` = 12 400 µs. The decoder waits for
//! a gap (12 400 ± 2080), reads the bits, and on the next gap classifies the last bit by its HIGH
//! alone and reports the frame when exactly 18 bits were read; it then looks for a gap again, so
//! of back-to-back frames every other one is reported. No fields (the C shows the key and its
//! bit-reversal, "Yek"), no checksum.
//!
//! Encoder (`repeat` = 3, no counter, no button: `button` is ignored): the 18 bits as above.

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
    make: "Elplast",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "Elplast";
const TE_SHORT: u32 = 230;
const TE_LONG: u32 = 1550;
const TE_DELTA: u32 = 160;
const MIN_COUNT_BIT: usize = 18;
const REPEAT: usize = 3;
/// The frame gap: `te_long * 8` ± `te_delta * 13`.
const GAP: u32 = TE_LONG * 8;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    SaveDuration,
    CheckDuration,
}

/// Elplast decoder (`SubGhzProtocolDecoderElplast`).
pub struct ElplastDecoder {
    step: Step,
    te_last: u32,
    reg: BitReg,
}

fn frame(data: u64) -> DecodedSignal {
    let mut d = decoded(data, MIN_COUNT_BIT);
    d.encoder_capable = true;
    d
}

impl ElplastDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            reg: BitReg::default(),
        }
    }
}

impl Default for ElplastDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// `subghz_protocol_encoder_elplast_get_upload` (one copy).
fn upload(data: u64) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(2 * MIN_COUNT_BIT);
    for i in (1..=MIN_COUNT_BIT).rev() {
        let one = (data >> (i - 1)) & 1 != 0;
        push(&mut out, true, if one { TE_LONG } else { TE_SHORT });
        let low = if i == 1 {
            GAP
        } else if one {
            TE_SHORT
        } else {
            TE_LONG
        };
        push(&mut out, false, low);
    }
    out
}

impl ProtocolDecoder for ElplastDecoder {
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
                if !level && near(duration, GAP, TE_DELTA * 13) {
                    self.reg.clear();
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
                if !level {
                    if near(self.te_last, TE_LONG, TE_DELTA) && near(duration, TE_SHORT, TE_DELTA) {
                        self.reg.add(true);
                        self.step = Step::SaveDuration;
                    } else if near(self.te_last, TE_SHORT, TE_DELTA)
                        && near(duration, TE_LONG, TE_DELTA)
                    {
                        self.reg.add(false);
                        self.step = Step::SaveDuration;
                    } else if near(duration, GAP, TE_DELTA * 13) {
                        if near(self.te_last, TE_LONG, TE_DELTA) {
                            self.reg.add(true);
                        }
                        if near(self.te_last, TE_SHORT, TE_DELTA) {
                            self.reg.add(false);
                        }
                        let result = (self.reg.count as usize == MIN_COUNT_BIT)
                            .then(|| frame(self.reg.data));
                        self.reg.clear();
                        self.step = Step::Reset;
                        return result;
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

    /// The Unleashed unit-test key (`elplast.sub`) and the C's second sample.
    const KEYS: [u64; 2] = [0x3_2C82, 0x3_2C0A];

    #[test]
    fn encode_decode_roundtrip() {
        for key in KEYS.into_iter().chain([0, 0x3_FFFF]) {
            let d = frame(key);
            let tx = ElplastDecoder::new().encode(&d, 0x1).unwrap();
            // The upload starts with the first bit: copy 1's gap opens copy 2 (reported at its
            // gap); the decoder then waits for another gap, so copy 3 is lost.
            let got = feed_all(&mut ElplastDecoder::new(), &tx);
            assert_eq!(got.len(), 1, "{key:X}");
            assert_eq!(format!("{:?}", got[0]), format!("{d:?}"));
            let mut rx = vec![LevelDuration::new(false, GAP)];
            rx.extend(tx);
            assert_eq!(
                feed_all(&mut ElplastDecoder::new(), &rx).len(),
                2,
                "copies 1 and 3"
            );
        }
    }

    #[test]
    fn hand_built_frame_and_rejections() {
        let mut raw = vec![-12_400];
        for i in (0..18).rev() {
            let one = (KEYS[0] >> i) & 1 != 0;
            raw.push(if one { 1540 } else { 240 });
            raw.push(if i == 0 {
                -12_400
            } else if one {
                -240
            } else {
                -1540
            });
        }
        let got = feed_all(&mut ElplastDecoder::new(), &wire(&raw));
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].data, KEYS[0]);
        let mut bad = raw.clone();
        bad[4] = -800; // a LOW neither short nor long
        assert!(feed_all(&mut ElplastDecoder::new(), &wire(&bad)).is_empty());
        let short: Vec<i32> = raw[..1].iter().chain(&raw[3..]).copied().collect(); // 17 bits
        assert!(feed_all(&mut ElplastDecoder::new(), &wire(&short)).is_empty());
    }

    #[test]
    fn key_file_loads_and_roundtrips() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Elplast\nBit: 18\n\
                    Key: 00 00 00 00 00 03 2C 82\n";
        let dec = ElplastDecoder::new();
        let d = dec
            .from_flipper_key(&FlipperKey::parse(text).unwrap())
            .unwrap();
        assert_eq!(
            (d.data, d.data_count_bit, d.encoder_capable),
            (KEYS[0], 18, true)
        );
        assert_key_roundtrip(&dec, &d);
    }

    /// Unleashed's `elplast_raw.sub` is not in REFERENCES: set `KAT_ELPLAST_RAW` to its path.
    #[test]
    fn decodes_unit_test_capture() {
        let Ok(path) = std::env::var("KAT_ELPLAST_RAW") else {
            eprintln!("Skip: KAT_ELPLAST_RAW not set");
            return;
        };
        let Some(got) = registry_decodes(&path) else {
            return;
        };
        let m: Vec<_> = got.iter().filter(|(n, _)| n == NAME).collect();
        assert!(
            !m.is_empty(),
            "{:?}",
            got.iter().map(|(n, _)| n).collect::<Vec<_>>()
        );
        assert!(m.iter().any(|(_, d)| d.data == KEYS[0]), "{m:?}");
    }
}
