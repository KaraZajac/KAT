//! Hörmann HSM (44-bit static code) decoder + encoder.
//!
//! Port of Flipper-ARF `applications/system/subghz_garage/protocols/hormann.c` (garage app, RX
//! group g9 + `tx_hormann`); doctor-firmware's copy decodes and encodes identically.
//!
//! Air format (433.92 / 868.35 MHz, AM): PWM, te_short 500 µs / te_long 1000 µs, te_delta 200. A
//! frame starts with a long HIGH start pulse of `24·te_short` = 12 000 µs (accepted ± 4800) and a
//! LOW te_short, then 44 bits, MSB first, each a HIGH then a LOW: `1` = HIGH 1000 + LOW 500, `0` =
//! HIGH 500 + LOW 1000 (all within `te_delta`). The next start pulse (a HIGH ≥ 2500 µs) ends the
//! frame, but only when the last 44 bits read match the HSM pattern `0xFF000000003` (top byte
//! `0xFF`, low two bits `11`); it is reported when at least 44 bits were read (the C's `>=`) and the
//! decoder then expects the start pulse's LOW. With the pattern unmatched, the long HIGH is taken
//! as a data HIGH and the frame breaks.
//!
//! Fields (`subghz_protocol_hormann_check_remote_controller`): button = `(data >> 8) & 0xF` (the
//! remote's four buttons are one-hot 0x1/0x2/0x4/0x8 per Flipper-ARF). No serial field, no
//! checksum beyond the fixed pattern.
//!
//! Encoder (`repeat` = 1, upload of 1801 entries, no counter): 20 × (start HIGH 12 000, LOW 500, 44
//! bits) and a final start HIGH. Button: the garage copy re-sends the captured key; Flipper-ARF's
//! `lib/subghz/protocols/hormann.c` lets the D-pad pick 0x1/0x2/0x4/0x8 (OK = captured) and
//! rewrites bits 8..11; KAT takes that union and accepts any 4-bit native code (others keep the
//! captured button).

use super::g2_common::{decoded, key_data, key_file, near, push, BitReg};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`): `SubGhzProtocolFlag_433 | 868 | AM`, static code.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Hormann",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "Hormann HSM";
const TE_SHORT: u32 = 500;
const TE_LONG: u32 = 1000;
const TE_DELTA: u32 = 200;
const MIN_COUNT_BIT: usize = 44;
/// `HORMANN_HSM_PATTERN`.
const PATTERN: u64 = 0xFF0_0000_0003;
/// Frames in the upload (`for repeat < 20`); the transmitter sends the upload once.
const FRAMES: usize = 20;
const START: u32 = TE_SHORT * 24;
const BUTTON_SHIFT: u32 = 8;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    FoundStartBit,
    SaveDuration,
    CheckDuration,
}

/// Hörmann HSM decoder (`SubGhzProtocolDecoderHormann`).
pub struct HormannDecoder {
    step: Step,
    te_last: u32,
    reg: BitReg,
}

fn frame(data: u64, bits: usize) -> DecodedSignal {
    let mut d = decoded(data, bits);
    d.button = Some(((data >> BUTTON_SHIFT) & 0xF) as u8);
    d.encoder_capable = bits == MIN_COUNT_BIT;
    d
}

impl HormannDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            reg: BitReg::default(),
        }
    }
}

impl Default for HormannDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// `subghz_protocol_encoder_hormann_get_upload`.
fn upload(data: u64) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity((MIN_COUNT_BIT * 2 + 2) * FRAMES + 1);
    for _ in 0..FRAMES {
        push(&mut out, true, START);
        push(&mut out, false, TE_SHORT);
        for i in (0..MIN_COUNT_BIT).rev() {
            let one = (data >> i) & 1 != 0;
            push(&mut out, true, if one { TE_LONG } else { TE_SHORT });
            push(&mut out, false, if one { TE_SHORT } else { TE_LONG });
        }
    }
    push(&mut out, true, START);
    out
}

impl ProtocolDecoder for HormannDecoder {
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
                if level && near(duration, START, TE_DELTA * 24) {
                    self.step = Step::FoundStartBit;
                }
            }
            Step::FoundStartBit => {
                if !level && near(duration, TE_SHORT, TE_DELTA) {
                    self.step = Step::SaveDuration;
                    self.reg.clear();
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::SaveDuration => {
                if level {
                    if duration >= TE_SHORT * 5 && self.reg.data & PATTERN == PATTERN {
                        self.step = Step::FoundStartBit;
                        // The register is cleared at the next start pulse's LOW.
                        let bits = self.reg.count as usize;
                        return (bits >= MIN_COUNT_BIT).then(|| frame(self.reg.data, bits));
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

    /// The 20-frame upload, sent once; a 4-bit `button` replaces bits 8..11 (ARF's D-pad).
    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        let mut data = decoded.data;
        if button <= 0xF {
            data = (data & !(0xF << BUTTON_SHIFT)) | (u64::from(button) << BUTTON_SHIFT);
        }
        Some(upload(data))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        let (data, bits) = key_data(key, |bits| bits == MIN_COUNT_BIT)?;
        Some(frame(data, bits))
    }

    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        Some(key_file(NAME, decoded))
    }
}

#[cfg(test)]
mod tests {
    use super::super::g2_common::test_util::*;
    use super::*;

    /// A frame with the HSM pattern and button 0x2.
    const KEY: u64 = 0xFF1_2345_6203;

    #[test]
    fn encode_decode_roundtrip_every_button() {
        let d = frame(KEY, MIN_COUNT_BIT);
        assert_eq!(d.button, Some(0x2));
        for (button, want) in [(1u8, 1u8), (2, 2), (4, 4), (8, 8), (0x10, 2)] {
            let tx = HormannDecoder::new().encode(&d, button).unwrap();
            assert_eq!(tx.len(), (MIN_COUNT_BIT * 2 + 2) * FRAMES + 1);
            // Each frame is reported at the next start pulse: 20 without resets …
            let got = feed_all(&mut HormannDecoder::new(), &tx);
            assert_eq!(got.len(), FRAMES);
            for g in &got {
                assert_eq!((g.button, g.data_count_bit), (Some(want), 44));
                assert_eq!(
                    g.data & !(0xF << BUTTON_SHIFT),
                    KEY & !(0xF << BUTTON_SHIFT)
                );
            }
            // … every other one with the registry's reset after each report.
            assert_eq!(
                feed_with_resets(&mut HormannDecoder::new(), &tx).len(),
                FRAMES / 2
            );
        }
    }

    #[test]
    fn needs_the_pattern() {
        // Same timing, top byte not 0xFF: the closing start pulse is read as a data HIGH.
        let mut tx = upload(0x7F1_2345_6203);
        tx.truncate((MIN_COUNT_BIT * 2 + 2) * 2 + 1);
        assert!(feed_all(&mut HormannDecoder::new(), &tx).is_empty());
        let mut tx = upload(0xFF1_2345_6201); // low bits 01
        tx.truncate((MIN_COUNT_BIT * 2 + 2) * 2 + 1);
        assert!(feed_all(&mut HormannDecoder::new(), &tx).is_empty());
    }

    #[test]
    fn hand_built_frame_with_capture_timing() {
        let mut raw = vec![11_800, -520];
        for i in (0..44).rev() {
            raw.extend(if (KEY >> i) & 1 != 0 {
                [980, -520]
            } else {
                [520, -980]
            });
        }
        raw.push(11_800);
        let got = feed_all(&mut HormannDecoder::new(), &wire(&raw));
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].data, KEY);
    }

    #[test]
    fn key_file_roundtrips() {
        let dec = HormannDecoder::new();
        let d = frame(KEY, MIN_COUNT_BIT);
        assert_key_roundtrip(&dec, &d);
        let text = dec.to_flipper_key(&d).unwrap().to_text();
        assert!(
            text.contains("Protocol: Hormann HSM\nBit: 44\nKey: 00 00 0F F1 23 45 62 03\n"),
            "{text}"
        );
    }

    /// `hormann_hsm_raw.sub` has comma-separated RAW_Data values.
    #[test]
    fn decodes_unit_test_capture() {
        let Some(got) = registry_decodes(&format!("{UNIT_TESTS}/hormann_hsm_raw.sub")) else {
            return;
        };
        let m: Vec<_> = got.iter().filter(|(n, _)| n == NAME).collect();
        assert!(
            !m.is_empty(),
            "{:?}",
            got.iter().map(|(n, _)| n).collect::<Vec<_>>()
        );
        assert!(m.iter().all(|(_, d)| d.data & PATTERN == PATTERN));
    }
}
