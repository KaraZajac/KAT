//! Mastercode (Clemsa MasterCode MV1 / MV12) decoder + encoder.
//!
//! Port of Flipper-ARF `applications/system/subghz_garage/protocols/mastercode.c` (garage app, RX
//! group g8 + `tx_mastercode`); doctor-firmware's copy decodes and encodes identically.
//!
//! Air format (433.92 MHz, AM): PWM, te_short 1072 µs / te_long 2145 µs, te_delta 150. 36 bits, MSB
//! first, each a HIGH then a LOW: `1` = HIGH 2145 + LOW 1072, `0` = HIGH 1072 + LOW 2145 (short
//! within `te_delta`, long within `8·te_delta`). The last bit's LOW is extended by `13·te_short`
//! into the frame gap; the decoder starts on a LOW of `15·te_short` ± `15·te_delta` (13 831..18 329
//! µs), classifies the last bit by its HIGH alone when the gap arrives, reports the frame when
//! exactly 36 bits were read, and continues with the next frame at once. The C sets that next step
//! *after* its callback, where the Flipper receiver (and KAT's registry) resets every decoder; KAT's
//! `reset` right after a report therefore resumes there too.
//!
//! Fields (`subghz_protocol_mastercode_check_remote_controller`): serial = `(data >> 4) & 0xFFFF`
//! (eight three-state DIP switches, displayed by the C), button = `(data >> 2) & 3`. No checksum.
//!
//! Encoder (`repeat` = 3, no counter): the 36 bits as above. The button is a fixed DIP code: the
//! references re-send the captured key (ARF's lib copy shows the D-pad but every direction sends
//! the original), so `button` is ignored.

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

const NAME: &str = "Mastercode";
const TE_SHORT: u32 = 1072;
const TE_LONG: u32 = 2145;
const TE_DELTA: u32 = 150;
const MIN_COUNT_BIT: usize = 36;
const REPEAT: usize = 3;
/// The frame gap: `te_short * 15` ± `te_delta * 15`.
const GAP: u32 = TE_SHORT * 15;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    SaveDuration,
    CheckDuration,
}

/// Mastercode decoder (`SubGhzProtocolDecoderMastercode`).
pub struct MastercodeDecoder {
    step: Step,
    te_last: u32,
    reg: BitReg,
    /// Set on a report: the C sets the step to SaveDuration *after* its callback, which is where
    /// the receiver resets every decoder, so the reset that follows a report resumes there.
    resume: bool,
}

fn frame(data: u64) -> DecodedSignal {
    let mut d = decoded(data, MIN_COUNT_BIT);
    d.serial = Some(((data >> 4) & 0xFFFF) as u32);
    d.button = Some(((data >> 2) & 0x3) as u8);
    d.encoder_capable = true;
    d
}

impl MastercodeDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            reg: BitReg::default(),
            resume: false,
        }
    }
}

impl Default for MastercodeDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// `subghz_protocol_encoder_mastercode_get_upload` (one copy).
fn upload(data: u64) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(2 * MIN_COUNT_BIT);
    for i in (1..MIN_COUNT_BIT).rev() {
        let one = (data >> i) & 1 != 0;
        push(&mut out, true, if one { TE_LONG } else { TE_SHORT });
        push(&mut out, false, if one { TE_SHORT } else { TE_LONG });
    }
    if data & 1 != 0 {
        push(&mut out, true, TE_LONG);
        push(&mut out, false, TE_SHORT + TE_SHORT * 13);
    } else {
        push(&mut out, true, TE_SHORT);
        push(&mut out, false, TE_LONG + TE_SHORT * 13);
    }
    out
}

impl ProtocolDecoder for MastercodeDecoder {
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
                if !level && near(duration, GAP, TE_DELTA * 15) {
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
                        && near(duration, TE_LONG, TE_DELTA * 8)
                    {
                        self.reg.add(false);
                        self.step = Step::SaveDuration;
                    } else if near(self.te_last, TE_LONG, TE_DELTA * 8)
                        && near(duration, TE_SHORT, TE_DELTA)
                    {
                        self.reg.add(true);
                        self.step = Step::SaveDuration;
                    } else if near(duration, GAP, TE_DELTA * 15) {
                        if near(self.te_last, TE_SHORT, TE_DELTA) {
                            self.reg.add(false);
                        } else if near(self.te_last, TE_LONG, TE_DELTA * 8) {
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

    /// The unit-test key (`mastercode.sub`).
    const KEY: u64 = 0xB_7E00_3C08;

    #[test]
    fn encode_decode_roundtrip() {
        for key in [KEY, 0xF_FFFF_FFFF, 0x0_0000_0001, 0x5_5555_5555] {
            let d = frame(key);
            for button in [0u8, 1, 3, 0xFF] {
                let tx = MastercodeDecoder::new().encode(&d, button).unwrap();
                let mut rx = vec![LevelDuration::new(false, GAP)];
                rx.extend(tx);
                let got = feed_all(&mut MastercodeDecoder::new(), &rx);
                assert_eq!(got.len(), REPEAT, "{key:X}");
                for g in &got {
                    assert_eq!(format!("{g:?}"), format!("{d:?}"), "button never changes");
                }
            }
        }
    }

    #[test]
    fn decodes_a_hand_built_frame_and_rejects_bad_ones() {
        let mut raw = vec![-16_000];
        for i in (0..36).rev() {
            let one = (KEY >> i) & 1 != 0;
            raw.push(if one { 2100 } else { 1080 });
            raw.push(if i == 0 {
                -16_000
            } else if one {
                -1060
            } else {
                -2200
            });
        }
        let got = feed_all(&mut MastercodeDecoder::new(), &wire(&raw));
        assert_eq!(got.len(), 1);
        assert_eq!(
            (got[0].data, got[0].serial, got[0].button),
            (KEY, Some(0x003C0), Some(0b10))
        );
        let mut bad = raw.clone();
        // HIGH and LOW both in the long window (2145 ± 1200): neither a 0 nor a 1.
        bad[1] = 1300;
        bad[2] = -1300;
        assert!(feed_all(&mut MastercodeDecoder::new(), &wire(&bad)).is_empty());
        let short: Vec<i32> = raw[..1].iter().chain(&raw[3..]).copied().collect(); // 35 bits
        assert!(feed_all(&mut MastercodeDecoder::new(), &wire(&short)).is_empty());
    }

    #[test]
    fn key_file_loads_and_roundtrips() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPresetOok270Async\nProtocol: Mastercode\nBit: 36\n\
                    Key: 00 00 00 0B 7E 00 3C 08\n";
        let dec = MastercodeDecoder::new();
        let d = dec
            .from_flipper_key(&FlipperKey::parse(text).unwrap())
            .unwrap();
        assert_eq!(
            (d.data, d.serial, d.button),
            (KEY, Some(0x03C0), Some(0b10))
        );
        assert_key_roundtrip(&dec, &d);
    }

    #[test]
    fn decodes_unit_test_capture() {
        let Some(got) = registry_decodes(&format!("{UNIT_TESTS}/mastercode_raw.sub")) else {
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
