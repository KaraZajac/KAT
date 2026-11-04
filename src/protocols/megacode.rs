//! MegaCode (Linear MegaCode, 24-bit fixed code) decoder + encoder.
//!
//! Port of Flipper-ARF's garage app `applications/system/subghz_garage/protocols/megacode.c` (the
//! primary reference; doctor-firmware's copy has the same wire behaviour). Flipper-ARF's lib copy
//! adds an encoder button switch (the low 3 bits), which KAT takes.
//!
//! **Air format** (AM, 315 MHz): 24 slots of 6 ms, MSB first, each with one 1 ms HIGH pulse — at the
//! end of the slot for a 1 (`00000 1`), after 2 ms for a 0 (`00 1 000`); the start bit is always 1.
//! Frames are separated by ≈ 9 ms of silence (the guard LOW after the last pulse is 11 ms after a
//! final 1, 14 ms after a final 0, the next frame's leading 5 ms included).
//!
//! **Decoder** (`subghz_protocol_decoder_megacode_feed`): a LOW within 13000 ± 3400 µs, then a
//! HIGH of 1000 ± 200 µs = the start bit (1). Each following LOW, minus 3000 µs when the previous
//! bit was 0, must be ≈ 5000 (± 1000 → bit 1) or ≈ 2000 (± 400 → bit 0) and be followed by a
//! 1000 ± 200 µs HIGH. A LOW ≥ 10 ms ends the frame, reported at exactly 24 bits.
//!
//! **Fields** (`subghz_protocol_megacode_check_remote_controller`, when bit 23 is the only bit above
//! bit 22, i.e. `data >> 23 == 1`): `serial` = data >> 3 & 0xFFFF (remote code), `button` = data &
//! 7, `counter` = the 4-bit facility code the C keeps in `generic.cnt` (data >> 19 & 0xF);
//! otherwise all 0.
//!
//! **Encoder** (`subghz_protocol_encoder_megacode_get_upload`, built backwards as in the C,
//! `encoder.repeat` = 3): guard LOW, then per bit its HIGH pulse and the LOW to the next pulse; ends
//! on the last pulse. `button` 0..=7 replaces the low 3 bits (ARF lib's custom button), larger
//! codes keep the captured button.
//!
//! **Key file**: `Protocol: MegaCode`, `Bit: 24`, `Key`.

use super::g1_common::{generic_fields, generic_key, push, with_repeats, TX_REPEAT};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "MegaCode",
    decoder: "MegaCode",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Linear",
    category: Category::Gate,
    buttons: Some(super::g1_common::numbered_buttons),
    presets: &[&AM650],
};

const NAME: &str = "MegaCode";
const TE_SHORT: u32 = 1000;
const TE_LONG: u32 = 1000;
const TE_DELTA: u32 = 200;
const MIN_COUNT_BIT: usize = 24;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    FoundStartBit,
    SaveDuration,
    CheckDuration,
}

/// MegaCode decoder (`SubGhzProtocolDecoderMegaCode`).
pub struct MegaCodeDecoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
    last_bit: bool,
}

/// `subghz_protocol_megacode_check_remote_controller`: (serial, button, facility).
fn check_remote_controller(data: u64) -> (u32, u8, u16) {
    if data >> 23 == 1 {
        (
            ((data >> 3) & 0xFFFF) as u32,
            (data & 0b111) as u8,
            ((data >> 19) & 0xF) as u16,
        )
    } else {
        (0, 0, 0)
    }
}

fn decoded(data: u64) -> DecodedSignal {
    let (serial, btn, facility) = check_remote_controller(data);
    DecodedSignal {
        serial: Some(serial),
        button: Some(btn),
        counter: Some(facility),
        crc_valid: false,
        data,
        data_count_bit: MIN_COUNT_BIT,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

/// `get_upload`: one frame, built from the end as the C does.
fn build_frame(data: u64) -> Vec<LevelDuration> {
    let bits = MIN_COUNT_BIT;
    let mut rev: Vec<(bool, u32)> = Vec::with_capacity(bits * 2);
    rev.push((true, TE_SHORT)); // end level
    let mut last_bit = data & 1 == 1;
    for i in 1..bits {
        if (data >> i) & 1 == 1 {
            rev.push((false, if last_bit { TE_SHORT * 5 } else { TE_SHORT * 2 }));
            last_bit = true;
        } else {
            rev.push((false, if last_bit { TE_SHORT * 8 } else { TE_SHORT * 5 }));
            last_bit = false;
        }
        rev.push((true, TE_SHORT));
    }
    rev.push((
        false,
        if data & 1 == 1 {
            TE_SHORT * 11
        } else {
            TE_SHORT * 14
        },
    ));
    let mut out = Vec::with_capacity(rev.len());
    for &(level, duration) in rev.iter().rev() {
        push(&mut out, level, duration);
    }
    out
}

impl MegaCodeDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
            last_bit: false,
        }
    }

    fn add_bit(&mut self, bit: u64) {
        self.decode_data = self.decode_data << 1 | bit;
        self.decode_count_bit = self.decode_count_bit.wrapping_add(1);
    }
}

impl Default for MegaCodeDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for MegaCodeDecoder {
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
                if !level && duration_diff!(duration, TE_SHORT * 13) < TE_DELTA * 17 {
                    self.step = Step::FoundStartBit;
                }
            }
            Step::FoundStartBit => {
                if level && duration_diff!(duration, TE_SHORT) < TE_DELTA {
                    self.step = Step::SaveDuration;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.add_bit(1);
                    self.last_bit = true;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::SaveDuration => {
                if !level {
                    if duration >= TE_SHORT * 10 {
                        self.step = Step::Reset;
                        if self.decode_count_bit as usize == MIN_COUNT_BIT {
                            return Some(decoded(self.decode_data));
                        }
                        return None;
                    }
                    self.te_last = if self.last_bit {
                        duration
                    } else {
                        duration.wrapping_sub(TE_SHORT * 3)
                    };
                    self.step = Step::CheckDuration;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::CheckDuration => {
                if level {
                    if duration_diff!(self.te_last, TE_SHORT * 5) < TE_DELTA * 5
                        && duration_diff!(duration, TE_SHORT) < TE_DELTA
                    {
                        self.add_bit(1);
                        self.last_bit = true;
                        self.step = Step::SaveDuration;
                    } else if duration_diff!(self.te_last, TE_SHORT * 2) < TE_DELTA * 2
                        && duration_diff!(duration, TE_SHORT) < TE_DELTA
                    {
                        self.add_bit(0);
                        self.last_bit = false;
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

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        let data = if button <= 0b111 {
            (decoded.data & !0b111) | button as u64
        } else {
            decoded.data
        };
        Some(with_repeats(&build_frame(data), TX_REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        let (bits, data) = generic_fields(key);
        (bits == MIN_COUNT_BIT).then(|| decoded(data))
    }

    fn to_flipper_key(&self, d: &DecodedSignal) -> Option<FlipperKey> {
        Some(generic_key(NAME, d))
    }
}

#[cfg(test)]
mod tests {
    use super::super::g1_common::test_support::*;
    use super::*;

    /// The example of megacode.c's comment: facility 3, remote 17316, button 1.
    #[test]
    fn fields_follow_the_reference_example() {
        let data = 1 << 23 | 3 << 19 | 17316 << 3 | 1;
        assert_eq!(check_remote_controller(data), (17316, 1, 3));
        assert_eq!(
            check_remote_controller(0x123456),
            (0, 0, 0),
            "start bit missing"
        );
        // The slot pattern of the comment (first 6 slots: 1 0 0 1 1 0).
        let frame = build_frame(data);
        let pattern: Vec<(bool, u32)> = frame
            .iter()
            .take(7)
            .map(|p| (p.level, p.duration_us))
            .collect();
        assert_eq!(
            pattern,
            [
                (false, 11000),
                (true, 1000),
                (false, 2000),
                (true, 1000),
                (false, 5000),
                (true, 1000),
                (false, 8000)
            ]
        );
    }

    #[test]
    fn encode_decode_roundtrip() {
        let d = decoded(0x8AE2D2);
        for button in [d.button.unwrap(), 0, 5, 7, 9] {
            let mut wire = MegaCodeDecoder::new().encode(&d, button).unwrap();
            push(&mut wire, false, 12_000);
            let got = feed_all(&mut MegaCodeDecoder::new(), &wire);
            assert_eq!(
                got.len(),
                2,
                "btn {button}: the reset after a decode skips one repeat"
            );
            let want = if button <= 7 {
                button
            } else {
                d.button.unwrap()
            };
            assert!(got
                .iter()
                .all(|f| f.serial == d.serial && f.counter == d.counter && f.button == Some(want)));
        }
    }

    #[test]
    fn rejects_double_pulses_and_short_frames() {
        let mut wire = build_frame(0x8AE2D2);
        // A 3.5 ms LOW after a 1 bit is neither 5 ms ± 1 nor 2 ms ± 0.4.
        let idx = wire
            .iter()
            .position(|p| !p.level && p.duration_us == 5000)
            .unwrap();
        wire[idx] = LevelDuration::new(false, 3500);
        push(&mut wire, false, 12_000);
        assert!(feed_all(&mut MegaCodeDecoder::new(), &wire).is_empty());
        // 23 slots.
        let mut short = build_frame(0x8AE2D2);
        short.truncate(short.len() - 2);
        push(&mut short, false, 12_000);
        assert!(feed_all(&mut MegaCodeDecoder::new(), &short).is_empty());
    }

    #[test]
    fn unit_test_key_file() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: MegaCode\nBit: 24\n\
                    Key: 00 00 00 00 00 8A E2 D2\n";
        let dec = MegaCodeDecoder::new();
        let d = dec
            .from_flipper_key(&FlipperKey::parse(text).unwrap())
            .unwrap();
        assert_eq!(
            (d.data, d.serial, d.button, d.counter),
            (0x8AE2D2, Some(0x5C5A), Some(2), Some(1))
        );
        let mut wire = dec.encode(&d, d.button.unwrap()).unwrap();
        push(&mut wire, false, 12_000);
        assert_eq!(
            feed_all(&mut MegaCodeDecoder::new(), &wire)[0].data,
            0x8AE2D2
        );
        let written = dec.to_flipper_key(&d).unwrap().to_text();
        let back = dec
            .from_flipper_key(&FlipperKey::parse(&written).unwrap())
            .unwrap();
        assert_eq!(format!("{back:?}"), format!("{d:?}"));
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&text.replace("Bit: 24", "Bit: 25")).unwrap())
            .is_none());
    }

    #[test]
    fn reference_raw_capture_decodes() {
        let Some(decodes) = decode_reference_capture("megacode_raw.sub") else {
            return;
        };
        assert!(
            decodes.iter().any(|(n, _)| n == NAME),
            "{:?}",
            decodes.iter().map(|(n, _)| n).collect::<Vec<_>>()
        );
    }
}
