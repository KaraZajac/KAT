//! CAME decoder + encoder: 12/24-bit fixed codes, plus the 18-bit "Airforce" and 25/42-bit
//! "Prastel" frames the same decoder reads.
//!
//! Port of Flipper-ARF's garage app `applications/system/subghz_garage/protocols/came.c` (the
//! primary reference; decodes like ARF's lib copy). doctor-firmware's copy differs on the wire:
//! it no longer decodes 42-bit frames (moved to its `prastel.c`) and only reports frames passing
//! `subghz_protocol_came_payload_ok` (no degenerate key; a 12-bit frame needs a ≥ 13120 µs header
//! and fewer than two 0x5/0xA nibbles). KAT follows ARF: every 12/18/24/25/42-bit frame.
//!
//! **Air format** (AM, 315/433.92 MHz): `te_short` 320, `te_long` 640, `te_delta` 150. Header LOW
//! (24 te for 24/42 bits = 24320 µs, 47 te for 12/18 bits, 36 te for 25 bits), start HIGH te, then
//! MSB first: bit 1 = LOW 640 + HIGH 320, bit 0 = LOW 320 + HIGH 640.
//!
//! **Decoder** (`subghz_protocol_decoder_came_feed`): header LOW within 17920 ± 9450 µs, start HIGH
//! |d − 320| < 150, then LOW/HIGH pairs; a LOW ≥ 1280 µs ends the frame, which is reported when it
//! holds exactly 12, 18, 24, 25 or 42 bits. No field split (`serial`/`button` stay empty, as the C's
//! display); the C names 18-bit frames "Airforce" and 25/42-bit frames "Prastel" — KAT shows them as
//! "CAME [Airforce]" / "CAME [Prastel]" (the key file still says `Protocol: CAME`). In Flipper-ARF's
//! garage app CAME is registered before its 42-bit "Prastel" protocol and wins frames both decode.
//!
//! **Encoder** (`subghz_protocol_encoder_came_get_upload`, `encoder.repeat` = 3): header, start bit,
//! the key's bits; keys up to 42 bits. CAME has no button field: `button` is ignored (ARF's lib
//! encoder also re-sends the captured code for every D-pad direction).
//!
//! **Key file**: `Protocol: CAME`, `Bit`, `Key` (Flipper-ARF's generic serialize); loading rejects
//! more than 42 bits, as the C's `deserialize`.

use super::g1_common::{fixed_code, generic_fields, generic_key, push, with_repeats, TX_REPEAT};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "CAME",
    decoder: "CAME",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "CAME",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "CAME";
const TE_SHORT: u32 = 320;
const TE_LONG: u32 = 640;
const TE_DELTA: u32 = 150;
const MIN_COUNT_BIT: usize = 12;
const CAME_24_COUNT_BIT: usize = 24;
const PRASTEL_25_COUNT_BIT: usize = 25;
const PRASTEL_42_COUNT_BIT: usize = 42;
const AIRFORCE_COUNT_BIT: usize = 18;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    FoundStartBit,
    SaveDuration,
    CheckDuration,
}

/// CAME decoder (`SubGhzProtocolDecoderCame`).
pub struct CameDecoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
}

/// The display variant `get_string` names by bit count.
fn display_name(bits: usize) -> Option<String> {
    match bits {
        PRASTEL_25_COUNT_BIT | PRASTEL_42_COUNT_BIT => Some("CAME [Prastel]".to_string()),
        AIRFORCE_COUNT_BIT => Some("CAME [Airforce]".to_string()),
        _ => None,
    }
}

fn decoded(data: u64, bits: usize) -> DecodedSignal {
    DecodedSignal {
        encoder_capable: bits <= PRASTEL_42_COUNT_BIT,
        protocol_display_name: display_name(bits),
        ..fixed_code(data, bits)
    }
}

/// `get_upload`'s header length in te for a bit count.
fn header_te(bits: usize) -> u32 {
    match bits {
        CAME_24_COUNT_BIT | PRASTEL_42_COUNT_BIT => 76,
        MIN_COUNT_BIT | AIRFORCE_COUNT_BIT => 47,
        PRASTEL_25_COUNT_BIT => 36,
        _ => 16,
    }
}

/// One `get_upload` frame.
fn build_frame(data: u64, bits: usize) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(bits * 2 + 2);
    push(&mut out, false, TE_SHORT * header_te(bits));
    push(&mut out, true, TE_SHORT);
    for i in (0..bits).rev() {
        if (data >> i) & 1 == 1 {
            push(&mut out, false, TE_LONG);
            push(&mut out, true, TE_SHORT);
        } else {
            push(&mut out, false, TE_SHORT);
            push(&mut out, true, TE_LONG);
        }
    }
    out
}

impl CameDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
        }
    }

    fn add_bit(&mut self, bit: u64) {
        self.decode_data = self.decode_data << 1 | bit;
        self.decode_count_bit = self.decode_count_bit.wrapping_add(1);
    }
}

impl Default for CameDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for CameDecoder {
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
                if !level && duration_diff!(duration, TE_SHORT * 56) < TE_DELTA * 63 {
                    self.step = Step::FoundStartBit;
                }
            }
            Step::FoundStartBit => {
                if !level {
                    // stay
                } else if duration_diff!(duration, TE_SHORT) < TE_DELTA {
                    self.step = Step::SaveDuration;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::SaveDuration => {
                if !level {
                    if duration >= TE_SHORT * 4 {
                        self.step = Step::FoundStartBit;
                        let bits = self.decode_count_bit as usize;
                        if matches!(
                            bits,
                            MIN_COUNT_BIT
                                | AIRFORCE_COUNT_BIT
                                | PRASTEL_25_COUNT_BIT
                                | PRASTEL_42_COUNT_BIT
                                | CAME_24_COUNT_BIT
                        ) {
                            // callback → the app resets the receiver (came_reset).
                            self.step = Step::Reset;
                            return Some(decoded(self.decode_data, bits));
                        }
                        return None;
                    }
                    self.te_last = duration;
                    self.step = Step::CheckDuration;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::CheckDuration => {
                if level {
                    if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA
                        && duration_diff!(duration, TE_LONG) < TE_DELTA
                    {
                        self.add_bit(0);
                        self.step = Step::SaveDuration;
                    } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA
                        && duration_diff!(duration, TE_SHORT) < TE_DELTA
                    {
                        self.add_bit(1);
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
        let bits = decoded.data_count_bit;
        if bits > PRASTEL_42_COUNT_BIT {
            return None;
        }
        Some(with_repeats(&build_frame(decoded.data, bits), TX_REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        let (bits, data) = generic_fields(key);
        (bits <= PRASTEL_42_COUNT_BIT).then(|| decoded(data, bits))
    }

    fn to_flipper_key(&self, d: &DecodedSignal) -> Option<FlipperKey> {
        Some(generic_key(NAME, d))
    }
}

#[cfg(test)]
mod tests {
    use super::super::g1_common::test_support::*;
    use super::*;

    /// The encoder's repeats with a trailing silence (the decoder needs a gap after each frame).
    fn on_air(d: &DecodedSignal) -> Vec<LevelDuration> {
        let mut wire = CameDecoder::new().encode(d, 0).unwrap();
        push(&mut wire, false, 30_000);
        wire
    }

    /// Every decoded length round-trips; the 3 repeats give 2 decodes (the reset inside the
    /// callback consumes the next frame's header).
    #[test]
    fn encode_decode_roundtrip() {
        for (data, bits, name) in [
            (0xABCu64, 12, None),
            (0x2_A5A5, 18, Some("CAME [Airforce]")),
            (0x6AB234, 24, None),
            (0x1A5_A5A5, 25, Some("CAME [Prastel]")),
            (0x2AB_CDEF_0123, 42, Some("CAME [Prastel]")),
        ] {
            let got = feed_all(&mut CameDecoder::new(), &on_air(&decoded(data, bits)));
            assert_eq!(got.len(), 2, "{bits} bits");
            for d in &got {
                assert_eq!((d.data, d.data_count_bit), (data, bits));
                assert_eq!(d.protocol_display_name.as_deref(), name);
                assert!(
                    d.serial.is_none() && d.button.is_none() && !d.crc_valid && d.encoder_capable
                );
            }
        }
    }

    #[test]
    fn header_lengths_follow_the_bit_count() {
        assert_eq!(build_frame(0, 24)[0].duration_us, 24320);
        assert_eq!(build_frame(0, 12)[0].duration_us, 15040);
        assert_eq!(build_frame(0, 25)[0].duration_us, 11520);
        assert_eq!(build_frame(0, 30)[0].duration_us, 5120);
    }

    #[test]
    fn rejects_other_lengths_and_bad_timing() {
        // 20 bits is not a CAME length (the 5120 µs default header is outside the window, too).
        let mut wire = vec![LevelDuration::new(false, 20_000)];
        wire.extend(build_frame(0xABCDE, 20).into_iter().skip(1));
        push(&mut wire, false, 30_000);
        assert!(feed_all(&mut CameDecoder::new(), &wire).is_empty());
        // A short pulse of 480 µs (|480 − 320| ≥ 150 and |480 − 640| ≥ 150) breaks the frame.
        let bad: Vec<LevelDuration> = on_air(&decoded(0x6AB234, 24))
            .into_iter()
            .enumerate()
            .map(|(i, p)| {
                if i == 20 {
                    LevelDuration::new(p.level, 480)
                } else {
                    p
                }
            })
            .collect();
        assert_eq!(
            feed_all(&mut CameDecoder::new(), &bad).len(),
            1,
            "only the intact repeat decodes"
        );
    }

    #[test]
    fn unit_test_key_file() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: CAME\nBit: 24\n\
                    Key: 00 00 00 00 00 6A B2 34\n";
        let dec = CameDecoder::new();
        let d = dec
            .from_flipper_key(&FlipperKey::parse(text).unwrap())
            .unwrap();
        assert_eq!((d.data, d.data_count_bit), (0x6AB234, 24));
        let got = feed_all(&mut CameDecoder::new(), &on_air(&d));
        assert_eq!(got[0].data, 0x6AB234);
        let written = dec.to_flipper_key(&d).unwrap().to_text();
        assert!(
            written.contains("Protocol: CAME\nBit: 24\nKey: 00 00 00 00 00 6A B2 34\n"),
            "{written}"
        );
        let back = dec
            .from_flipper_key(&FlipperKey::parse(&written).unwrap())
            .unwrap();
        assert_eq!(format!("{back:?}"), format!("{d:?}"));
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&text.replace("Bit: 24", "Bit: 43")).unwrap())
            .is_none());
    }

    #[test]
    fn reference_raw_capture_decodes() {
        let Some(decodes) = decode_reference_capture("came_raw.sub") else {
            return;
        };
        assert!(
            decodes.iter().any(|(n, _)| n.starts_with(NAME)),
            "{:?}",
            decodes.iter().map(|(n, _)| n).collect::<Vec<_>>()
        );
    }
}
