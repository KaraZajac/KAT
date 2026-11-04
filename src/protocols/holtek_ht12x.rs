//! Holtek_HT12X (HT12E / HT12A 12-bit fixed code) decoder + encoder.
//!
//! Port of Flipper-ARF's garage app `applications/system/subghz_garage/protocols/holtek_ht12x.c`
//! (the primary reference; doctor-firmware's copy has the same wire behaviour). Flipper-ARF's lib
//! copy adds an encoder button switch (the low nibble), which KAT takes.
//!
//! **Air format** (AM/FM, 315/433.92/868 MHz): te ≈ 320 µs (`te_short` 320, `te_long` 640,
//! `te_delta` 200); header LOW 36·te, start HIGH te, then MSB first: bit 1 = LOW 2·te + HIGH te,
//! bit 0 = LOW te + HIGH 2·te. 12 bits: 8 address (DIP) bits + 4 data bits (active low B1..B4).
//!
//! **Decoder** (`subghz_protocol_decoder_holtek_th12x_feed`): header LOW within 8960 ± 4000 µs,
//! start HIGH |d − 320| < 200; bit 1 = LOW long (± 400) + HIGH short (± 200), bit 0 = LOW short +
//! HIGH long; a LOW ≥ 3400 µs ends the frame. A 12-bit frame is reported only when it equals the
//! previous one (`last_data`, non-zero); TE = (start bit + every LOW/HIGH of the frame) / 37. After
//! the callback the C stores `last_data` and waits for the next start bit (the app's reset in the
//! callback does not clear `last_data` in this protocol, nor does KAT's `reset()`), so every
//! repeat after the first decodes.
//!
//! **Fields** (`subghz_protocol_holtek_th12x_check_remote_controller`): `button` = low nibble (a 0
//! bit = that button pressed: 0x7 = B1, 0xB = B2, 0xD = B3, 0xE = B4), `counter` = the 8-bit DIP
//! address the C keeps in `generic.cnt` (bits 4..11); `extra` = TE.
//!
//! **Encoder** (`subghz_protocol_encoder_holtek_th12x_get_upload`, `encoder.repeat` = 3): header
//! LOW 36·TE, start HIGH TE, 12 bits with the key's TE. `button` 0..=15 replaces the low nibble
//! (ARF lib's custom button: B1 0x7, B2 0xB, B3 0xD, B4 0xE), larger codes keep the captured one.
//!
//! **Key file**: `Protocol: Holtek_HT12X`, `Bit: 12`, `Key`, `TE` (required on load).

use super::g1_common::{generic_fields, generic_key, push, u32_line, with_repeats, TX_REPEAT};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`). The references flag it AM and FM; KAT transmits OOK (AM650) unless the
/// capture says otherwise, with FM476 as the FSK preset.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Holtek_HT12X",
    decoder: "Holtek_HT12X",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::Both,
    encryption: "Fixed Code",
    make: "Holtek",
    category: Category::Gate,
    buttons: Some(native_buttons),
    presets: &[&AM650, &FM476],
};

/// HT12X data nibble: the buttons whose bit is 0 (the C's `event_serialize`).
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    const NAMES: [&str; 16] = [
        "B1 B2 B3 B4",
        "B1 B2 B3",
        "B1 B2 B4",
        "B1 B2",
        "B1 B3 B4",
        "B1 B3",
        "B1 B4",
        "B1",
        "B2 B3 B4",
        "B2 B3",
        "B2 B4",
        "B2",
        "B3 B4",
        "B3",
        "B4",
        "None",
    ];
    Some(NAMES.get(code as usize).copied().unwrap_or("Other"))
}

const NAME: &str = "Holtek_HT12X";
const TE_SHORT: u32 = 320;
const TE_LONG: u32 = 640;
const TE_DELTA: u32 = 200;
const MIN_COUNT_BIT: usize = 12;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    FoundStartBit,
    SaveDuration,
    CheckDuration,
}

/// Holtek_HT12X decoder (`SubGhzProtocolDecoderHoltek_HT12X`).
pub struct HoltekHt12xDecoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
    te: u32,
    /// Kept across resets, as in the C (its reset only rewinds `parser_step`).
    last_data: u32,
    /// The last `feed` reported a frame (see `g1_common`): keep the post-callback state.
    just_emitted: bool,
}

fn decoded(data: u64, te: u32) -> DecodedSignal {
    DecodedSignal {
        serial: None,
        button: Some((data & 0x0F) as u8),
        counter: Some(((data >> 4) & 0xFF) as u16),
        crc_valid: false,
        data,
        data_count_bit: MIN_COUNT_BIT,
        encoder_capable: te != 0,
        extra: Some(te as u64),
        protocol_display_name: None,
    }
}

/// One `get_upload` frame with the key's TE.
fn build_frame(data: u64, te: u32) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(MIN_COUNT_BIT * 2 + 2);
    push(&mut out, false, te.wrapping_mul(36));
    push(&mut out, true, te);
    for i in (0..MIN_COUNT_BIT).rev() {
        if (data >> i) & 1 == 1 {
            push(&mut out, false, te.wrapping_mul(2));
            push(&mut out, true, te);
        } else {
            push(&mut out, false, te);
            push(&mut out, true, te.wrapping_mul(2));
        }
    }
    out
}

impl HoltekHt12xDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
            te: 0,
            last_data: 0,
            just_emitted: false,
        }
    }

    fn add_bit(&mut self, bit: u64) {
        self.decode_data = self.decode_data << 1 | bit;
        self.decode_count_bit = self.decode_count_bit.wrapping_add(1);
    }
}

impl Default for HoltekHt12xDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for HoltekHt12xDecoder {
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
        if std::mem::take(&mut self.just_emitted) {
            return;
        }
        // subghz_protocol_decoder_holtek_th12x_reset: parser_step only (last_data is kept).
        self.step = Step::Reset;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        self.just_emitted = false;
        match self.step {
            Step::Reset => {
                if !level && duration_diff!(duration, TE_SHORT * 28) < TE_DELTA * 20 {
                    self.step = Step::FoundStartBit;
                }
            }
            Step::FoundStartBit => {
                if level && duration_diff!(duration, TE_SHORT) < TE_DELTA {
                    self.step = Step::SaveDuration;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.te = duration;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::SaveDuration => {
                if !level {
                    if duration >= TE_SHORT * 10 + TE_DELTA {
                        let mut out = None;
                        if self.decode_count_bit as usize == MIN_COUNT_BIT {
                            if self.last_data as u64 == self.decode_data && self.last_data != 0 {
                                self.te /= self.decode_count_bit as u32 * 3 + 1;
                                out = Some(decoded(self.decode_data, self.te));
                            }
                            self.last_data = self.decode_data as u32;
                        }
                        self.decode_data = 0;
                        self.decode_count_bit = 0;
                        self.te = 0;
                        self.step = Step::FoundStartBit;
                        self.just_emitted = out.is_some();
                        return out;
                    }
                    self.te_last = duration;
                    self.te = self.te.wrapping_add(duration);
                    self.step = Step::CheckDuration;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::CheckDuration => {
                if level {
                    self.te = self.te.wrapping_add(duration);
                    if duration_diff!(self.te_last, TE_LONG) < TE_DELTA * 2
                        && duration_diff!(duration, TE_SHORT) < TE_DELTA
                    {
                        self.add_bit(1);
                        self.step = Step::SaveDuration;
                    } else if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA
                        && duration_diff!(duration, TE_LONG) < TE_DELTA * 2
                    {
                        self.add_bit(0);
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
        let te = decoded.extra? as u32;
        if te == 0 {
            return None;
        }
        let data = if button <= 0x0F {
            (decoded.data & !0x0F) | button as u64
        } else {
            decoded.data
        };
        Some(with_repeats(&build_frame(data, te), TX_REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        let (bits, data) = generic_fields(key);
        if bits != MIN_COUNT_BIT {
            return None;
        }
        Some(decoded(data, u32_line(key, "TE")?))
    }

    fn to_flipper_key(&self, d: &DecodedSignal) -> Option<FlipperKey> {
        let mut key = generic_key(NAME, d);
        key.push("TE", (d.extra? as u32).to_string());
        Some(key)
    }
}

#[cfg(test)]
mod tests {
    use super::super::g1_common::test_support::*;
    use super::*;

    fn on_air(d: &DecodedSignal, button: u8) -> Vec<LevelDuration> {
        let mut wire = HoltekHt12xDecoder::new().encode(d, button).unwrap();
        push(&mut wire, false, 10_000);
        wire
    }

    #[test]
    fn labels() {
        assert_eq!(native_buttons(NAME, 0x7), Some("B1"));
        assert_eq!(native_buttons(NAME, 0xE), Some("B4"));
        assert_eq!(native_buttons(NAME, 0x3), Some("B1 B2"));
        assert_eq!(native_buttons(NAME, 0xF), Some("None"));
    }

    /// 3 repeats: the first sets `last_data`, the second and third decode; TE comes back.
    #[test]
    fn encode_decode_roundtrip() {
        let d = decoded(0xFFB, 205);
        for (button, want) in [(0xB, 0xFFBu64), (0x7, 0xFF7), (0xE, 0xFFE), (0x20, 0xFFB)] {
            let got = feed_all(&mut HoltekHt12xDecoder::new(), &on_air(&d, button));
            assert_eq!(got.len(), 2, "btn {button:#x}");
            for f in &got {
                assert_eq!((f.data, f.counter, f.extra), (want, Some(0xFF), Some(205)));
            }
        }
    }

    /// `last_data` survives the registry's reset (the C's reset leaves it), so after a decode by
    /// any protocol the next identical frame decodes at once.
    #[test]
    fn last_data_survives_reset() {
        let frame = build_frame(0x5A7, 320);
        let mut dec = HoltekHt12xDecoder::new();
        assert!(feed_all(&mut dec, &frame).is_empty());
        assert!(
            feed_all(&mut dec, &[LevelDuration::new(false, 10_000)]).is_empty(),
            "first frame stored"
        );
        dec.reset();
        let mut wire = frame.clone();
        push(&mut wire, false, 10_000);
        assert_eq!(feed_all(&mut dec, &wire).len(), 1);
    }

    #[test]
    fn rejects_mismatch_and_bad_timing() {
        let mut wire = build_frame(0x5A7, 320);
        wire.extend(build_frame(0x5A6, 320));
        push(&mut wire, false, 10_000);
        assert!(feed_all(&mut HoltekHt12xDecoder::new(), &wire).is_empty());
        let bad: Vec<LevelDuration> = with_repeats(&build_frame(0x5A7, 320), 3)
            .into_iter()
            .map(|p| {
                if p.duration_us == 640 {
                    LevelDuration::new(p.level, 1100)
                } else {
                    p
                }
            })
            .collect();
        assert!(feed_all(&mut HoltekHt12xDecoder::new(), &bad).is_empty());
    }

    #[test]
    fn unit_test_key_file() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Holtek_HT12X\nBit: 12\n\
                    Key: 00 00 00 00 00 00 0F FB\nTE: 205\n";
        let dec = HoltekHt12xDecoder::new();
        let d = dec
            .from_flipper_key(&FlipperKey::parse(text).unwrap())
            .unwrap();
        assert_eq!(
            (d.data, d.button, d.counter, d.extra),
            (0xFFB, Some(0xB), Some(0xFF), Some(205))
        );
        assert_eq!(
            feed_all(&mut HoltekHt12xDecoder::new(), &on_air(&d, 0xB))[0].data,
            0xFFB
        );
        let written = dec.to_flipper_key(&d).unwrap().to_text();
        assert!(
            written.ends_with("Key: 00 00 00 00 00 00 0F FB\nTE: 205\n"),
            "{written}"
        );
        let back = dec
            .from_flipper_key(&FlipperKey::parse(&written).unwrap())
            .unwrap();
        assert_eq!(format!("{back:?}"), format!("{d:?}"));
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&text.replace("TE: 205\n", "")).unwrap())
            .is_none());
    }

    #[test]
    fn reference_raw_capture_decodes() {
        let Some(decodes) = decode_reference_capture("holtek_ht12x_raw.sub") else {
            return;
        };
        assert!(
            decodes.iter().any(|(n, _)| n == NAME),
            "{:?}",
            decodes.iter().map(|(n, _)| n).collect::<Vec<_>>()
        );
    }
}
