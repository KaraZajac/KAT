//! Clemsa decoder + encoder.
//!
//! Port of Flipper-ARF `applications/system/subghz_garage/protocols/clemsa.c` (garage app, RX
//! group g8 + `tx_clemsa`); doctor-firmware's copy decodes and encodes identically.
//!
//! Air format (433.92 MHz, AM): PWM, te_short 385 µs / te_long 2695 µs, te_delta 150. 18 bits, MSB
//! first, each a HIGH then a LOW: `1` = HIGH 2695 + LOW 385, `0` = HIGH 385 + LOW 2695 (short
//! within `te_delta`, long within `3·te_delta`). The last bit's LOW is extended by `7·te_long` into
//! the frame gap; the decoder starts on a LOW of `51·te_short` ± `25·te_delta` (15 886..23 384 µs),
//! classifies the last bit by its HIGH alone when the gap arrives, reports the frame when exactly
//! 18 bits were read, and continues with the next frame at once (the gap doubles as its start).
//! The C sets that next step *after* its callback, where the Flipper receiver (and KAT's registry)
//! resets every decoder; KAT's `reset` right after a report therefore resumes there too.
//!
//! Fields (`subghz_protocol_clemsa_check_remote_controller`): serial = `(data >> 2) & 0xFFFF` (the
//! eight three-state DIP switches the C displays), button = `data & 3`. No checksum.
//!
//! Encoder (`repeat` = 3, no counter): the 18 bits as above. Button: the garage copy re-sends the
//! captured key; Flipper-ARF's `lib/subghz/protocols/clemsa.c` lets the D-pad pick 1, 2, 3 (OK =
//! captured) and rewrites the low 2 bits; KAT takes that union and accepts any 2-bit native code
//! (others keep the captured button).

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
    make: "Clemsa",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "Clemsa";
const TE_SHORT: u32 = 385;
const TE_LONG: u32 = 2695;
const TE_DELTA: u32 = 150;
const MIN_COUNT_BIT: usize = 18;
const REPEAT: usize = 3;
/// The frame gap: `te_short * 51` ± `te_delta * 25`.
const GAP: u32 = TE_SHORT * 51;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    SaveDuration,
    CheckDuration,
}

/// Clemsa decoder (`SubGhzProtocolDecoderClemsa`).
pub struct ClemsaDecoder {
    step: Step,
    te_last: u32,
    reg: BitReg,
    /// Set on a report: the C sets the step to SaveDuration *after* its callback, which is where
    /// the receiver resets every decoder, so the reset that follows a report resumes there.
    resume: bool,
}

fn frame(data: u64) -> DecodedSignal {
    let mut d = decoded(data, MIN_COUNT_BIT);
    d.serial = Some(((data >> 2) & 0xFFFF) as u32);
    d.button = Some((data & 0x3) as u8);
    d.encoder_capable = true;
    d
}

impl ClemsaDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            reg: BitReg::default(),
            resume: false,
        }
    }
}

impl Default for ClemsaDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// `subghz_protocol_encoder_clemsa_get_upload` (one copy).
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

impl ProtocolDecoder for ClemsaDecoder {
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
        self.step = if std::mem::take(&mut self.resume) {
            Step::SaveDuration
        } else {
            Step::Reset
        };
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        self.resume = false;
        match self.step {
            Step::Reset => {
                if !level && near(duration, GAP, TE_DELTA * 25) {
                    self.step = Step::SaveDuration;
                    self.reg.clear();
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
                    if near(self.te_last, TE_SHORT, TE_DELTA)
                        && near(duration, TE_LONG, TE_DELTA * 3)
                    {
                        self.reg.add(false);
                        self.step = Step::SaveDuration;
                    } else if near(self.te_last, TE_LONG, TE_DELTA * 3)
                        && near(duration, TE_SHORT, TE_DELTA)
                    {
                        self.reg.add(true);
                        self.step = Step::SaveDuration;
                    } else if near(duration, GAP, TE_DELTA * 25) {
                        // The last bit by its HIGH alone (neither: no bit; the C's step reset is
                        // overwritten below).
                        if near(self.te_last, TE_SHORT, TE_DELTA) {
                            self.reg.add(false);
                        } else if near(self.te_last, TE_LONG, TE_DELTA * 3) {
                            self.reg.add(true);
                        }
                        let result = (self.reg.count as usize == MIN_COUNT_BIT)
                            .then(|| frame(self.reg.data));
                        self.step = Step::SaveDuration;
                        self.resume = result.is_some();
                        self.reg.clear();
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

    /// 3 copies; a 2-bit `button` replaces the low 2 bits (ARF's D-pad).
    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        let data = if button <= 0x3 {
            (decoded.data & !0x3) | u64::from(button)
        } else {
            decoded.data
        };
        Some(repeated(&upload(data), REPEAT))
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

    /// The unit-test key (`clemsa.sub`).
    const KEY: u64 = 0x2_FCAA;

    #[test]
    fn encode_decode_roundtrip_every_button() {
        for key in [KEY, 0x0_0000, 0x3_FFFF, 0x1_5555] {
            let d = frame(key);
            for (button, want) in [(0u8, 0u8), (1, 1), (2, 2), (3, 3), (9, (key & 3) as u8)] {
                let tx = ClemsaDecoder::new().encode(&d, button).unwrap();
                // The upload starts with the first bit: copy 1's gap opens copy 2, and so on.
                let got = feed_all(&mut ClemsaDecoder::new(), &tx);
                assert_eq!(got.len(), REPEAT - 1, "{key:X}");
                let mut rx = vec![LevelDuration::new(false, GAP)];
                rx.extend(tx);
                let got = feed_all(&mut ClemsaDecoder::new(), &rx);
                assert_eq!(got.len(), REPEAT);
                for g in &got {
                    assert_eq!(
                        (g.serial, g.button),
                        (Some(((key >> 2) & 0xFFFF) as u32), Some(want))
                    );
                }
            }
        }
    }

    #[test]
    fn decodes_a_hand_built_frame_and_rejects_bad_ones() {
        let mut raw = vec![-19_000];
        for i in (0..18).rev() {
            let one = (KEY >> i) & 1 != 0;
            raw.push(if one { 2700 } else { 380 });
            raw.push(if i == 0 {
                -19_000
            } else if one {
                -390
            } else {
                -2690
            });
        }
        let got = feed_all(&mut ClemsaDecoder::new(), &wire(&raw));
        assert_eq!(got.len(), 1);
        assert_eq!((got[0].data, got[0].button), (KEY, Some(0b10)));
        let mut bad = raw.clone();
        bad[3] = 1500; // a HIGH neither short nor long
        assert!(feed_all(&mut ClemsaDecoder::new(), &wire(&bad)).is_empty());
        let mut long = raw.clone();
        long.splice(1..1, [380, -2690]); // 19 bits
        assert!(feed_all(&mut ClemsaDecoder::new(), &wire(&long)).is_empty());
    }

    #[test]
    fn key_file_loads_and_roundtrips() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Clemsa\nBit: 18\n\
                    Key: 00 00 00 00 00 02 FC AA\n";
        let dec = ClemsaDecoder::new();
        let d = dec
            .from_flipper_key(&FlipperKey::parse(text).unwrap())
            .unwrap();
        assert_eq!(
            (d.data, d.serial, d.button),
            (KEY, Some(0xBF2A), Some(0b10))
        );
        assert_key_roundtrip(&dec, &d);
    }

    #[test]
    fn decodes_unit_test_capture() {
        let Some(got) = registry_decodes(&format!("{UNIT_TESTS}/clemsa_raw.sub")) else {
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
