//! Dooya 40-bit blind / shutter remote.
//!
//! Primary reference: Flipper-ARF's garage app copy
//! `REFERENCES/Flipper-ARF/applications/system/subghz_garage/protocols/dooya.c`. doctor-firmware's
//! `lib/subghz/protocols/dooya.c` additionally drops degenerate keys
//! (`subghz_protocol_blocks_key_is_degenerate`: all-0/1, 0xAA…/0x55…, a run of four equal
//! nibbles); the garage copy (and Flipper-ARF's `lib` copy) report them — KAT follows the garage
//! copy. Flipper-ARF's `lib` copy adds a button switch to the encoder (d-pad: Up_Long 0x11,
//! Down_Long 0x33, Stop 0x55, P2 0xCC), which KAT takes (union).
//!
//! **Line coding**: te_short = 366 µs, te_long = 733 µs, te_delta = 120 µs. Frame: LOW header
//! (12·te_long + the last bit's LOW, from the encoder), HIGH 13·te_short (4 758 µs) start, LOW
//! 2·te_long (1 466 µs), then 40 bits MSB first, each HIGH then LOW: 1 = HIGH 733 + LOW 366,
//! 0 = HIGH 366 + LOW 733.
//!
//! **Decoder** (`subghz_protocol_decoder_dooya_feed`): Reset needs a LOW within 20·te_delta of
//! 12·te_long; then HIGHs within 5·te_delta of the start pulse are skipped and a LOW within
//! 3·te_delta of 2·te_long starts the data. A short half must be within te_delta, a long one
//! within 2·te_delta. A LOW of ≥ 4·te_long ends the frame (the last bit read from its HIGH) and
//! it is reported when exactly 40 bits were read. No checksum.
//!
//! **Fields** (`subghz_protocol_dooya_check_remote_controller`): serial = bits 16..39 (24 bits);
//! counter = the C's `cnt` = channel: bits 8..11 when bits 12..15 are non-zero (multi-channel
//! console), else 0xFF (single channel); button = bits 0..7 (0x11 Up_Long, 0x1E Up_Short, 0x33
//! Down_Long, 0x3C Down_Short, 0x55 Stop, 0x79 Up+Down, 0x80 Up+Stop, 0x81 Down+Stop, 0xCC P2).
//!
//! **Encoder** (`subghz_protocol_encoder_dooya_get_upload`): LOW 12·te_long + (bit 0 ? te_long :
//! te_short), HIGH 4 758, LOW 1 466, the 40 bit pairs; the upload is cut after `2·40 + 2`
//! entries, so the last bit's LOW is not sent (the next copy's header follows its HIGH); sent 3
//! times. Static. `button`: the captured code, or one of Flipper-ARF's d-pad codes 0x11 / 0x33 /
//! 0x55 / 0xCC written into bits 0..7; anything else keeps the captured byte.
//!
//! **Key file**: `Protocol: Dooya`, `Bit: 40`, `Key` (Flipper-ARF's generic serialize).

use super::g3_common::{duration_diff, repeat_upload, UploadBuilder};
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Dooya",
    decoder: "Dooya",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Dooya",
    category: Category::Gate,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// `subghz_protocol_dooya_get_name_button`.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x11 => "Up_Long",
        0x1E => "Up_Short",
        0x33 => "Down_Long",
        0x3C => "Down_Short",
        0x55 => "Stop",
        0x79 => "Up+Down",
        0x80 => "Up+Stop",
        0x81 => "Down+Stop",
        0xCC => "P2",
        _ => "Unknown",
    })
}

const TE_SHORT: u32 = 366;
const TE_LONG: u32 = 733;
const TE_DELTA: u32 = 120;
const BIT_COUNT: usize = 40;
/// `DOYA_SINGLE_CHANNEL`.
const SINGLE_CHANNEL: u16 = 0xFF;
/// `subghz_protocol_encoder_dooya_alloc`: repeat = 3.
const REPEAT: usize = 3;
/// Flipper-ARF's d-pad button codes (Up_Long, Down_Long, Stop, P2).
const DPAD_BUTTONS: [u8; 4] = [0x11, 0x33, 0x55, 0xCC];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    FoundStartBit,
    SaveDuration,
    CheckDuration,
}

/// Dooya decoder (`SubGhzProtocolDecoderDooya`).
pub struct DooyaDecoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
}

/// (serial, channel, button) — `subghz_protocol_dooya_check_remote_controller`.
fn fields(data: u64) -> (u32, u16, u8) {
    let channel = if (data >> 12) & 0x0F != 0 {
        ((data >> 8) & 0x0F) as u16
    } else {
        SINGLE_CHANNEL
    };
    ((data >> 16) as u32, channel, (data & 0xFF) as u8)
}

fn signal(data: u64) -> DecodedSignal {
    let (serial, channel, button) = fields(data);
    DecodedSignal {
        serial: Some(serial),
        button: Some(button),
        counter: Some(channel),
        crc_valid: false,
        data,
        data_count_bit: BIT_COUNT,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

/// One upload (`get_upload`, cut to `2·40 + 2` entries).
fn upload(data: u64) -> Vec<LevelDuration> {
    let mut entries = Vec::with_capacity(2 * BIT_COUNT + 3);
    let header_tail = if data & 1 == 1 { TE_LONG } else { TE_SHORT };
    entries.push((false, TE_LONG * 12 + header_tail));
    entries.push((true, TE_SHORT * 13));
    entries.push((false, TE_LONG * 2));
    for i in (0..BIT_COUNT).rev() {
        if (data >> i) & 1 == 1 {
            entries.push((true, TE_LONG));
            entries.push((false, TE_SHORT));
        } else {
            entries.push((true, TE_SHORT));
            entries.push((false, TE_LONG));
        }
    }
    entries.truncate(2 * BIT_COUNT + 2);
    let mut b = UploadBuilder::new();
    for (level, duration) in entries {
        b.push(level, duration);
    }
    b.finish()
}

impl DooyaDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
        }
    }

    fn add_bit(&mut self, bit: u64) {
        self.decode_data = (self.decode_data << 1) | bit;
        self.decode_count_bit = self.decode_count_bit.wrapping_add(1);
    }
}

impl Default for DooyaDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for DooyaDecoder {
    fn name(&self) -> &'static str {
        "Dooya"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: BIT_COUNT,
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
                if !level && duration_diff(duration, TE_LONG * 12) < TE_DELTA * 20 {
                    self.step = Step::FoundStartBit;
                }
            }
            Step::FoundStartBit => {
                if !level {
                    if duration_diff(duration, TE_LONG * 2) < TE_DELTA * 3 {
                        self.step = Step::SaveDuration;
                        self.decode_data = 0;
                        self.decode_count_bit = 0;
                    } else {
                        self.step = Step::Reset;
                    }
                } else if duration_diff(duration, TE_SHORT * 13) < TE_DELTA * 5 {
                    // stay
                } else {
                    self.step = Step::Reset;
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
                if duration >= TE_LONG * 4 {
                    if duration_diff(self.te_last, TE_SHORT) < TE_DELTA {
                        self.add_bit(0);
                    } else if duration_diff(self.te_last, TE_LONG) < TE_DELTA * 2 {
                        self.add_bit(1);
                    } else {
                        self.step = Step::Reset;
                        return None;
                    }
                    self.step = Step::FoundStartBit;
                    if usize::from(self.decode_count_bit) == BIT_COUNT {
                        return Some(signal(self.decode_data));
                    }
                    return None;
                }
                if duration_diff(self.te_last, TE_SHORT) < TE_DELTA
                    && duration_diff(duration, TE_LONG) < TE_DELTA * 2
                {
                    self.add_bit(0);
                    self.step = Step::SaveDuration;
                } else if duration_diff(self.te_last, TE_LONG) < TE_DELTA * 2
                    && duration_diff(duration, TE_SHORT) < TE_DELTA
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

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != BIT_COUNT {
            return None;
        }
        let data = if DPAD_BUTTONS.contains(&button) {
            (decoded.data & !0xFF) | u64::from(button)
        } else {
            decoded.data
        };
        Some(repeat_upload(&upload(data), REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        (key.bits == BIT_COUNT).then(|| signal(key.key))
    }

    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        (decoded.data_count_bit == BIT_COUNT).then(|| {
            FlipperKey::new(
                "Dooya",
                BIT_COUNT,
                decoded.data,
                433_920_000,
                "FuriHalSubGhzPresetOok650Async",
            )
        })
    }
}

#[cfg(test)]
mod tests {
    use super::super::g3_common::test_support::*;
    use super::*;

    /// dooya.sub (Flipper unit test).
    const KEY_FILE: &str = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
        Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Dooya\nBit: 40\n\
        Key: 00 00 00 E1 DC 03 05 11\n";

    #[test]
    fn unit_test_key_file_loads_and_encodes() {
        let d = DooyaDecoder::new();
        let s = d
            .from_flipper_key(&FlipperKey::parse(KEY_FILE).unwrap())
            .unwrap();
        assert_eq!(
            (s.data, s.serial, s.counter, s.button),
            (
                0xE1_DC03_0511,
                Some(0xE1DC03),
                Some(SINGLE_CHANNEL),
                Some(0x11)
            )
        );
        // Copies 1 and 2 are closed by the next copy's header; the last copy's final LOW is not
        // in the upload (the C cuts it), so it needs the silence after the transmission.
        let mut tx = d.encode(&s, 0x11).unwrap();
        assert_eq!(feed_all(&mut DooyaDecoder::new(), &tx).len(), 2);
        tx.push(LevelDuration::new(false, 20_000));
        let frames = feed_all(&mut DooyaDecoder::new(), &tx);
        assert_eq!(frames.len(), 3);
        assert!(frames.iter().all(|f| f.data == s.data));
        let text = d.to_flipper_key(&s).unwrap().to_text();
        assert!(text.contains("Protocol: Dooya\nBit: 40\nKey: 00 00 00 E1 DC 03 05 11\n"));
        let back = d
            .from_flipper_key(&FlipperKey::parse(&text).unwrap())
            .unwrap();
        assert_eq!(format!("{back:?}"), format!("{s:?}"));
    }

    #[test]
    fn button_switch_and_channel() {
        let s = signal(0xE1_DC03_1533);
        assert_eq!(
            s.counter,
            Some(0x5),
            "multi-channel console: channel in bits 8..11"
        );
        for (button, want) in [
            (0x33u8, 0x33u8),
            (0x11, 0x11),
            (0x55, 0x55),
            (0xCC, 0xCC),
            (0x1E, 0x33),
            (0x00, 0x33),
        ] {
            let mut tx = DooyaDecoder::new().encode(&s, button).unwrap();
            tx.push(LevelDuration::new(false, 20_000));
            let f = &feed_all(&mut DooyaDecoder::new(), &tx)[0];
            assert_eq!(f.button, Some(want), "button {button:#x}");
            assert_eq!(f.data >> 8, s.data >> 8);
        }
    }

    #[test]
    fn upload_layout() {
        let one = upload(0xE1_DC03_0511);
        assert_eq!(one.len(), 82);
        assert_eq!((one[0].level, one[0].duration_us), (false, 733 * 12 + 733));
        assert_eq!((one[1].level, one[1].duration_us), (true, 4758));
        assert_eq!((one[2].level, one[2].duration_us), (false, 1466));
        assert!(one[81].level, "ends with the last bit's HIGH");
        assert_eq!(upload(0xE1_DC03_0510)[0].duration_us, 733 * 12 + 366);
    }

    #[test]
    fn rejects_bad_frames() {
        let mut p = upload(0xE1_DC03_0511);
        p.push(LevelDuration::new(false, 20_000));
        let mut short = p.clone();
        short.drain(3..5);
        assert!(feed_all(&mut DooyaDecoder::new(), &short).is_empty());
        let mut bad = p.clone();
        bad[10].duration_us = 1100;
        assert!(feed_all(&mut DooyaDecoder::new(), &bad).is_empty());
        let mut bad = p;
        bad[2].duration_us = 2500;
        assert!(feed_all(&mut DooyaDecoder::new(), &bad).is_empty());
    }

    #[test]
    fn decodes_the_flipper_unit_test_capture() {
        let Some(path) = unit_test_file("dooya_raw.sub") else {
            return;
        };
        let frames = decoder_decodes(&mut DooyaDecoder::new(), &path);
        assert!(
            !frames.is_empty(),
            "the reference unit test needs ≥ 1 frame"
        );
        let names: Vec<String> = registry_decodes(&path)
            .into_iter()
            .map(|(n, _)| n)
            .collect();
        assert!(names.iter().any(|n| n == "Dooya"), "{names:?}");
    }
}
