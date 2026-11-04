//! Roger (Roger Technology) 28-bit fixed-code gate remote.
//!
//! Primary reference: Flipper-ARF's garage app copy
//! `REFERENCES/Flipper-ARF/applications/system/subghz_garage/protocols/roger.c`; doctor-firmware
//! and Flipper-ARF `lib/subghz/protocols/roger.c` decode and transmit identically.
//!
//! **Line coding**: te_short = 500 µs, te_long = 1000 µs, te_delta = 270 µs. Each bit is a HIGH
//! then a LOW: 1 = HIGH 1000 + LOW 500, 0 = HIGH 500 + LOW 1000; the last bit's LOW is the
//! 19·te_short (9 500 µs) gap. 28 bits, MSB first.
//!
//! **Decoder** (`subghz_protocol_decoder_roger_feed`): Reset waits for a LOW within
//! 5·te_delta (1 350 µs) of the gap; bits are HIGH/LOW pairs within te_delta. A gap-length LOW
//! closes the frame: the last bit is read from its HIGH with two independent tests (≈1000 → 1,
//! then ≈500 → 0; a 731–769 µs HIGH passes both and adds two bits, as in the C), and the frame
//! is reported when exactly 28 bits were read. The decoder then returns to Reset, so only every
//! other frame of a held button is reported (as in the C). No checksum.
//!
//! **Fields** (`subghz_protocol_roger_check_remote_controller`): serial = bits 12..27, button =
//! bits 8..11 (1, 2, 4, 8), "End" = bits 0..7 (the button again, or 0x20 / 0x23 on some
//! remotes).
//!
//! **Encoder** (`subghz_protocol_encoder_roger_get_upload`): static. With the captured button
//! the key is sent unchanged. Another button (KAT's native code, one of 1, 2, 4, 8, accepted
//! when the captured button is one of them — the references' custom-button switch) rebuilds
//! the key as the C does: End == captured button → serial ‖ button ‖ button; End 0x23 and
//! button 1 → End 0x20; End 0x20 and button 2 → End 0x23; any other End is sent unchanged
//! ("no custom button allowed"). Upload = 28 bit pairs with the gap after the last, 3 times.
//!
//! **Key file**: `Protocol: Roger`, `Bit: 28`, `Key` (Flipper-ARF's generic serialize).

use super::g3_common::{duration_diff, repeat_upload, UploadBuilder};
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Roger",
    decoder: "Roger",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Roger",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

const TE_SHORT: u32 = 500;
const TE_LONG: u32 = 1000;
const TE_DELTA: u32 = 270;
const BIT_COUNT: usize = 28;
/// Gap after a frame: 19·te_short.
const GAP: u32 = TE_SHORT * 19;
/// `subghz_protocol_encoder_roger_alloc`: repeat = 3.
const REPEAT: usize = 3;
const BUTTONS: [u8; 4] = [0x1, 0x2, 0x4, 0x8];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    SaveDuration,
    CheckDuration,
}

/// Roger decoder (`SubGhzProtocolDecoderRoger`).
pub struct RogerDecoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
}

/// (serial, button) — `subghz_protocol_roger_check_remote_controller`.
fn fields(data: u64) -> (u32, u8) {
    ((data >> 12) as u32, ((data >> 8) & 0xF) as u8)
}

fn signal(data: u64) -> DecodedSignal {
    let (serial, button) = fields(data);
    DecodedSignal {
        serial: Some(serial),
        button: Some(button),
        counter: None,
        crc_valid: false,
        data,
        data_count_bit: BIT_COUNT,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

/// The key `get_upload` sends for `button`.
fn key_for_button(data: u64, button: u8) -> u64 {
    let (serial, captured) = fields(data);
    let serial = u64::from(serial);
    let end = data & 0xFF;
    let btn = u64::from(button);
    if end == u64::from(captured) {
        (serial << 12) | (btn << 8) | btn
    } else if end == 0x23 && button == 0x1 {
        (serial << 12) | (btn << 8) | 0x20
    } else if end == 0x20 && button == 0x2 {
        (serial << 12) | (btn << 8) | 0x23
    } else {
        data
    }
}

/// One upload: 28 bit pairs, the last LOW being the gap.
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

impl RogerDecoder {
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

impl Default for RogerDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for RogerDecoder {
    fn name(&self) -> &'static str {
        "Roger"
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
                if !level && duration_diff(duration, GAP) < TE_DELTA * 5 {
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
                } else if duration_diff(duration, GAP) < TE_DELTA * 5 {
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
        let (_, captured) = fields(decoded.data);
        let button = if BUTTONS.contains(&captured) && BUTTONS.contains(&button) {
            button
        } else {
            captured
        };
        Some(repeat_upload(
            &upload(key_for_button(decoded.data, button)),
            REPEAT,
        ))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        (key.bits == BIT_COUNT).then(|| signal(key.key))
    }

    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        (decoded.data_count_bit == BIT_COUNT).then(|| {
            FlipperKey::new(
                "Roger",
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

    /// roger.sub (Flipper unit test).
    const KEY_FILE: &str = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
        Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Roger\nBit: 28\n\
        Key: 00 00 00 00 05 AB A1 01\n";

    fn decode_tx(tx: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut p = vec![LevelDuration::new(false, GAP)];
        p.extend_from_slice(tx);
        feed_all(&mut RogerDecoder::new(), &p)
    }

    #[test]
    fn unit_test_key_file_loads_and_encodes() {
        let d = RogerDecoder::new();
        let s = d
            .from_flipper_key(&FlipperKey::parse(KEY_FILE).unwrap())
            .unwrap();
        assert_eq!(
            (s.data, s.serial, s.button),
            (0x5AB_A101, Some(0x5ABA), Some(1))
        );
        let frames = decode_tx(&d.encode(&s, 1).unwrap());
        // gap + 3 copies: copies 1 and 3 are reported (the decoder is in Reset after a frame).
        assert_eq!(frames.len(), 2);
        assert!(frames.iter().all(|f| f.data == s.data));
        let text = d.to_flipper_key(&s).unwrap().to_text();
        assert!(text.contains("Protocol: Roger\nBit: 28\nKey: 00 00 00 00 05 AB A1 01\n"));
        let back = d
            .from_flipper_key(&FlipperKey::parse(&text).unwrap())
            .unwrap();
        assert_eq!(format!("{back:?}"), format!("{s:?}"));
    }

    #[test]
    fn button_switch_follows_the_end_byte_rules() {
        // End == button: rebuilt with the new button in both places.
        let s = signal(0x0DF_E101);
        for (button, want) in [
            (1u8, 0x0DF_E101u64),
            (2, 0x0DF_E202),
            (4, 0x0DF_E404),
            (8, 0x0DF_E808),
            (3, 0x0DF_E101),
        ] {
            let f = &decode_tx(&RogerDecoder::new().encode(&s, button).unwrap())[0];
            assert_eq!(f.data, want, "button {button}");
        }
        // End 0x20 (button 1) ↔ End 0x23 (button 2).
        let a = signal(0x23F_9120);
        assert_eq!(
            decode_tx(&RogerDecoder::new().encode(&a, 2).unwrap())[0].data,
            0x23F_9223
        );
        assert_eq!(
            decode_tx(&RogerDecoder::new().encode(&a, 4).unwrap())[0].data,
            0x23F_9120
        );
        let b = signal(0x23F_9223);
        assert_eq!(
            decode_tx(&RogerDecoder::new().encode(&b, 1).unwrap())[0].data,
            0x23F_9120
        );
    }

    #[test]
    fn last_bit_window_overlap_adds_two_bits() {
        // A 750 µs last HIGH is within te_delta of both 500 and 1000: the C adds 1 then 0.
        let mut d = RogerDecoder::new();
        let mut p = vec![LevelDuration::new(false, GAP)];
        for i in (1..27).rev() {
            let bit = (0x5AB_A101u64 >> i) & 1 == 1;
            p.push(LevelDuration::new(true, if bit { 1000 } else { 500 }));
            p.push(LevelDuration::new(false, if bit { 500 } else { 1000 }));
        }
        p.push(LevelDuration::new(true, 750));
        p.push(LevelDuration::new(false, GAP));
        let frames = feed_all(&mut d, &p);
        assert_eq!(frames.len(), 1);
        assert_eq!(
            frames[0].data,
            (((0x5AB_A101u64 >> 1) & 0x3FF_FFFF) << 2) | 0b10
        );
    }

    #[test]
    fn rejects_bad_frames() {
        let tx = upload(0x5AB_A101);
        let mut bad = tx.clone();
        bad[4].duration_us = 1400; // HIGH out of both windows
        assert!(decode_tx(&bad).is_empty());
        let mut short = tx.clone();
        short.drain(0..2); // 27 bits
        assert!(decode_tx(&short).is_empty());
        assert!(
            feed_all(&mut RogerDecoder::new(), &tx).is_empty(),
            "no gap before the frame"
        );
    }

    #[test]
    fn decodes_the_flipper_unit_test_capture() {
        let Some(path) = unit_test_file("roger_raw.sub") else {
            return;
        };
        let frames = decoder_decodes(&mut RogerDecoder::new(), &path);
        assert!(
            !frames.is_empty(),
            "the reference unit test needs ≥ 1 frame"
        );
        let names: Vec<String> = registry_decodes(&path)
            .into_iter()
            .map(|(n, _)| n)
            .collect();
        assert!(names.iter().any(|n| n == "Roger"), "{names:?}");
    }
}
