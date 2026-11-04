//! Hay21 21-bit remote (button, serial byte and a 4-bit rolling counter).
//!
//! Primary reference: Flipper-ARF's garage app copy
//! `REFERENCES/Flipper-ARF/applications/system/subghz_garage/protocols/hay21.c`; doctor-firmware
//! and Flipper-ARF `lib/subghz/protocols/hay21.c` decode and transmit identically.
//!
//! **Line coding**: te_short = 300 µs, te_long = 700 µs, te_delta = 150 µs. Each bit is a HIGH
//! then a LOW: 1 = HIGH 700 + LOW 300, 0 = HIGH 300 + LOW 700; the last bit's LOW is the
//! 6·te_long (4 200 µs) gap between frames. 21 bits, MSB first, no preamble.
//!
//! **Decoder** (`subghz_protocol_decoder_hay21_feed`): Reset waits for a LOW within 3·te_delta
//! (450 µs) of the 4 200 µs gap. Bits are HIGH/LOW pairs within te_delta. A LOW within
//! 2·te_delta of the gap instead of a bit LOW closes the frame (the last bit is read from its
//! HIGH: ≈700 → 1, ≈300 → 0) and reports it when exactly 21 bits were read; the decoder then
//! goes back to Reset, so the gap that ended a frame does not open the next one and only every
//! other frame of a held button is reported (as in the C). No checksum.
//!
//! **Fields** (`subghz_protocol_hay21_remote_controller`): button = bits 13..20 (0x5A On/Off,
//! 0xC3 Mode, 0x88 Hold), serial = bits 5..12, counter = bits 1..4.
//!
//! **Encoder** (`subghz_protocol_encoder_hay21_get_upload`): counter + 1 (above 0xF wraps to 0),
//! key = button << 13 | serial << 5 | counter << 1 | 1 (bit 0 always 1), upload = the 21 bit
//! pairs with the gap after the last, sent 3 times. `button` (native code) is used when both it
//! and the captured button are one of 0x5A / 0xC3 / 0x88 (the references' custom-button switch
//! moves between those three), otherwise the captured button is sent.
//!
//! **Key file**: `Protocol: Hay21`, `Bit: 21`, `Key` (Flipper-ARF's generic serialize).

use super::g3_common::{duration_diff, repeat_upload, UploadBuilder};
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Hay21",
    decoder: "Hay21",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "Hay21",
    category: Category::Gate,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// `subghz_protocol_hay21_get_button_name`.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x5A => "On/Off",
        0xC3 => "Mode",
        0x88 => "Hold",
        _ => "Unknown",
    })
}

const TE_SHORT: u32 = 300;
const TE_LONG: u32 = 700;
const TE_DELTA: u32 = 150;
const BIT_COUNT: usize = 21;
/// Gap between frames: 6·te_long.
const GAP: u32 = TE_LONG * 6;
/// `subghz_protocol_encoder_hay21_alloc`: repeat = 3.
const REPEAT: usize = 3;
const BUTTONS: [u8; 3] = [0x5A, 0xC3, 0x88];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    SaveDuration,
    CheckDuration,
}

/// Hay21 decoder (`SubGhzProtocolDecoderHay21`).
pub struct Hay21Decoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
}

/// (button, serial, counter) — `subghz_protocol_hay21_remote_controller`.
fn fields(data: u64) -> (u8, u32, u16) {
    (
        ((data >> 13) & 0xFF) as u8,
        ((data >> 5) & 0xFF) as u32,
        ((data >> 1) & 0xF) as u16,
    )
}

fn signal(data: u64) -> DecodedSignal {
    let (button, serial, counter) = fields(data);
    DecodedSignal {
        serial: Some(serial),
        button: Some(button),
        counter: Some(counter),
        crc_valid: false,
        data,
        data_count_bit: BIT_COUNT,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

/// One upload: 21 bit pairs, the last LOW being the gap.
fn upload(key: u64) -> Vec<LevelDuration> {
    let mut b = UploadBuilder::new();
    for i in (0..BIT_COUNT).rev() {
        let last = i == 0;
        if (key >> i) & 1 == 1 {
            b.push(true, TE_LONG);
            b.push(false, if last { GAP } else { TE_SHORT });
        } else {
            b.push(true, TE_SHORT);
            b.push(false, if last { GAP } else { TE_LONG });
        }
    }
    b.finish()
}

impl Hay21Decoder {
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

impl Default for Hay21Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for Hay21Decoder {
    fn name(&self) -> &'static str {
        "Hay21"
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
                if !level && duration_diff(duration, GAP) < TE_DELTA * 3 {
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
                if duration_diff(self.te_last, TE_LONG) < TE_DELTA
                    && duration_diff(duration, TE_SHORT) < TE_DELTA
                {
                    self.add_bit(1);
                    self.step = Step::SaveDuration;
                } else if duration_diff(self.te_last, TE_SHORT) < TE_DELTA
                    && duration_diff(duration, TE_LONG) < TE_DELTA
                {
                    self.add_bit(0);
                    self.step = Step::SaveDuration;
                } else if duration_diff(duration, GAP) < TE_DELTA * 2 {
                    if duration_diff(self.te_last, TE_LONG) < TE_DELTA {
                        self.add_bit(1);
                    }
                    if duration_diff(self.te_last, TE_SHORT) < TE_DELTA {
                        self.add_bit(0);
                    }
                    let found = (usize::from(self.decode_count_bit) == BIT_COUNT)
                        .then(|| signal(self.decode_data));
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.step = Step::Reset;
                    return found;
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
        let (captured_button, serial, captured_counter) = fields(decoded.data);
        let button = if BUTTONS.contains(&captured_button) && BUTTONS.contains(&button) {
            button
        } else {
            captured_button
        };
        let counter = u32::from(decoded.counter.unwrap_or(captured_counter));
        // `cnt + mult > 0xF → 0`, mult = 1.
        let counter = if counter + 1 > 0xF { 0 } else { counter + 1 };
        let key =
            (u64::from(button) << 13) | (u64::from(serial) << 5) | (u64::from(counter) << 1) | 1;
        Some(repeat_upload(&upload(key), REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        (key.bits == BIT_COUNT).then(|| signal(key.key))
    }

    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        (decoded.data_count_bit == BIT_COUNT).then(|| {
            FlipperKey::new(
                "Hay21",
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

    fn key(button: u8, serial: u8, counter: u8, bit0: u64) -> u64 {
        (u64::from(button) << 13) | (u64::from(serial) << 5) | (u64::from(counter) << 1) | bit0
    }

    #[test]
    fn synthetic_frames_decode_every_other_one() {
        // gap, frame, frame, frame: the gap before frame 1 opens it; frame 1's gap closes it and
        // the decoder returns to Reset, so frame 2 is skipped and its gap opens frame 3.
        let data = key(0x5A, 0x42, 0x5, 0);
        let mut p = vec![LevelDuration::new(false, GAP)];
        for _ in 0..3 {
            p.extend(upload(data));
        }
        let frames = feed_all(&mut Hay21Decoder::new(), &p);
        assert_eq!(frames.len(), 2);
        let f = &frames[0];
        assert_eq!((f.data, f.data_count_bit), (data, 21));
        assert_eq!(
            (f.button, f.serial, f.counter),
            (Some(0x5A), Some(0x42), Some(5))
        );
        assert!(f.encoder_capable && !f.crc_valid);
    }

    #[test]
    fn encode_round_trip() {
        let captured = signal(key(0xC3, 0xBD, 0xF, 0));
        for (button, want) in [(0xC3u8, 0xC3u8), (0x5A, 0x5A), (0x88, 0x88), (0x11, 0xC3)] {
            let mut p = vec![LevelDuration::new(false, GAP)];
            p.extend(Hay21Decoder::new().encode(&captured, button).unwrap());
            let frames = feed_all(&mut Hay21Decoder::new(), &p);
            assert_eq!(frames.len(), 2, "button {button:#x}");
            for f in &frames {
                assert_eq!(f.button, Some(want));
                assert_eq!(f.serial, Some(0xBD));
                assert_eq!(f.counter, Some(0), "0xF + 1 wraps to 0");
                assert_eq!(f.data & 1, 1, "the encoder always sets bit 0");
            }
        }
        // A captured button outside the three is kept.
        let other = signal(key(0x12, 0x01, 3, 1));
        let tx = Hay21Decoder::new().encode(&other, 0x5A).unwrap();
        let mut p = vec![LevelDuration::new(false, GAP)];
        p.extend(tx);
        let f = &feed_all(&mut Hay21Decoder::new(), &p)[0];
        assert_eq!((f.button, f.counter), (Some(0x12), Some(4)));
    }

    #[test]
    fn upload_is_the_c_layout() {
        let tx = upload(key(0x88, 0x42, 1, 1));
        assert_eq!(tx.len(), 42);
        assert_eq!((tx[41].level, tx[41].duration_us), (false, 4200));
        assert_eq!(
            Hay21Decoder::new()
                .encode(&signal(key(0x88, 0x42, 1, 1)), 0x88)
                .unwrap()
                .len(),
            126
        );
    }

    #[test]
    fn rejects_bad_frames() {
        let data = key(0x5A, 0x42, 0x5, 0);
        let mut p = vec![LevelDuration::new(false, GAP)];
        p.extend(upload(data));
        let mut bad = p.clone();
        bad[7].duration_us = 500; // between short and long
        assert!(feed_all(&mut Hay21Decoder::new(), &bad).is_empty());
        let mut short = p.clone();
        short.drain(1..3); // 20 bits
        assert!(feed_all(&mut Hay21Decoder::new(), &short).is_empty());
        let mut bad_gap = p;
        bad_gap[0].duration_us = 5000;
        assert!(feed_all(&mut Hay21Decoder::new(), &bad_gap).is_empty());
    }

    #[test]
    fn key_file_round_trip() {
        let d = Hay21Decoder::new();
        let s = signal(key(0x5A, 0xBD, 0xF, 1));
        let text = d.to_flipper_key(&s).unwrap().to_text();
        assert!(
            text.contains("Protocol: Hay21\nBit: 21\nKey: 00 00 00 00 00 0B 57 BF\n"),
            "{text}"
        );
        let back = d
            .from_flipper_key(&FlipperKey::parse(&text).unwrap())
            .unwrap();
        assert_eq!(format!("{back:?}"), format!("{s:?}"));
    }

    #[test]
    fn decodes_the_flipper_unit_test_capture() {
        let Some(path) = unit_test_file("hay21_raw.sub") else {
            return;
        };
        let frames = decoder_decodes(&mut Hay21Decoder::new(), &path);
        assert!(
            !frames.is_empty(),
            "the reference unit test needs ≥ 1 frame"
        );
        let names: Vec<String> = registry_decodes(&path)
            .into_iter()
            .map(|(n, _)| n)
            .collect();
        assert!(names.iter().any(|n| n == "Hay21"), "{names:?}");
    }
}
