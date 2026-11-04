//! LinearDelta3 (Linear Delta-3, 8-bit DIP-switch fixed code) decoder + encoder.
//!
//! Port of Flipper-ARF's garage app `applications/system/subghz_garage/protocols/linear_delta3.c`
//! (the primary reference; doctor-firmware's copy has the same wire behaviour).
//!
//! **Air format** (AM): `te_short` 500, `te_long` 2000, `te_delta` 150; MSB first, bit 1 = HIGH
//! 500 and LOW 3500, bit 0 = HIGH 2000 and LOW 2000; the last bit's LOW is the guard (73·te = 36500 µs after
//! a 1, 70·te = 35000 µs after a 0). 8 bits.
//!
//! **Decoder** (`subghz_protocol_decoder_linear_delta3_feed`): a LOW within 35000 ± 3600 µs starts a
//! frame; HIGH/LOW pairs; a LOW ≥ 5000 µs ends it (the last bit from its HIGH alone). An 8-bit frame
//! is reported only when it equals the previous 8-bit frame (`last_data`, non-zero). After every
//! 8-bit frame the C keeps collecting **without clearing the bits**, so the next frame reads as 16
//! bits and is dropped, and the one after it needs a new start LOW: a long transmission decodes
//! every third frame from the fifth on (frames 2 and 5 match). The app's reset inside the callback
//! is undone by the code that follows it (`parser_step = SaveDuration`, `last_data = data`).
//!
//! **Encoder** (`subghz_protocol_encoder_linear_delta3_get_upload`, `encoder.repeat` = 3): the 8
//! bits with the guard folded into the last LOW. Fixed code: `button` is ignored. (One 3-frame
//! burst therefore never decodes on a Flipper or in KAT; a held button does.)
//!
//! **Key file**: `Protocol: LinearDelta3`, `Bit: 8`, `Key`.

use super::g1_common::{fixed_code, generic_fields, generic_key, push, with_repeats, TX_REPEAT};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "LinearDelta3",
    decoder: "LinearDelta3",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Linear",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "LinearDelta3";
const TE_SHORT: u32 = 500;
const TE_LONG: u32 = 2000;
const TE_DELTA: u32 = 150;
const MIN_COUNT_BIT: usize = 8;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    SaveDuration,
    CheckDuration,
}

/// LinearDelta3 decoder (`SubGhzProtocolDecoderLinearDelta3`).
pub struct LinearDelta3Decoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
    last_data: u32,
    /// The last `feed` reported a frame (see `g1_common`): keep the post-callback state.
    just_emitted: bool,
}

/// One `get_upload` frame.
fn build_frame(data: u64) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(MIN_COUNT_BIT * 2);
    for i in (1..MIN_COUNT_BIT).rev() {
        if (data >> i) & 1 == 1 {
            push(&mut out, true, TE_SHORT);
            push(&mut out, false, TE_SHORT * 7);
        } else {
            push(&mut out, true, TE_LONG);
            push(&mut out, false, TE_LONG);
        }
    }
    if data & 1 == 1 {
        push(&mut out, true, TE_SHORT);
        push(&mut out, false, TE_SHORT * 73);
    } else {
        push(&mut out, true, TE_LONG);
        push(&mut out, false, TE_SHORT * 70);
    }
    out
}

impl LinearDelta3Decoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
            last_data: 0,
            just_emitted: false,
        }
    }

    fn add_bit(&mut self, bit: u64) {
        self.decode_data = self.decode_data << 1 | bit;
        self.decode_count_bit = self.decode_count_bit.wrapping_add(1);
    }
}

impl Default for LinearDelta3Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for LinearDelta3Decoder {
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
        // subghz_protocol_decoder_linear_delta3_reset
        self.step = Step::Reset;
        self.last_data = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        self.just_emitted = false;
        match self.step {
            Step::Reset => {
                if !level && duration_diff!(duration, TE_SHORT * 70) < TE_DELTA * 24 {
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
                if duration >= TE_SHORT * 10 {
                    self.step = Step::Reset;
                    if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA {
                        self.add_bit(1);
                    } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA {
                        self.add_bit(0);
                    }
                    let mut out = None;
                    if self.decode_count_bit as usize == MIN_COUNT_BIT {
                        if self.last_data as u64 == self.decode_data && self.last_data != 0 {
                            out = Some(fixed_code(self.decode_data, MIN_COUNT_BIT));
                        }
                        // (the app's reset inside the callback is overwritten right here)
                        self.step = Step::SaveDuration;
                        self.last_data = self.decode_data as u32;
                    }
                    self.just_emitted = out.is_some();
                    return out;
                }
                if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA
                    && duration_diff!(duration, TE_SHORT * 7) < TE_DELTA
                {
                    self.add_bit(1);
                    self.step = Step::SaveDuration;
                } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA
                    && duration_diff!(duration, TE_LONG) < TE_DELTA
                {
                    self.add_bit(0);
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

    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        Some(with_repeats(&build_frame(decoded.data), TX_REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        let (bits, data) = generic_fields(key);
        (bits == MIN_COUNT_BIT).then(|| fixed_code(data, bits))
    }

    fn to_flipper_key(&self, d: &DecodedSignal) -> Option<FlipperKey> {
        Some(generic_key(NAME, d))
    }
}

#[cfg(test)]
mod tests {
    use super::super::g1_common::test_support::*;
    use super::*;

    /// One 3-frame burst never decodes (see the module doc); a held button (5 bursts) decodes
    /// frames 5, 8, 11 and 14 — the registry's reset after each decode keeps that cadence.
    #[test]
    fn encode_decode_roundtrip() {
        for data in [0xD0u64, 0x01, 0x7F, 0xFF] {
            let burst = LinearDelta3Decoder::new()
                .encode(&fixed_code(data, 8), 0)
                .unwrap();
            assert_eq!(burst.len(), 3 * 16);
            assert!(feed_all(&mut LinearDelta3Decoder::new(), &burst).is_empty());
            let held = with_repeats(&burst, 5);
            let mut dec = LinearDelta3Decoder::new();
            let mut got = Vec::new();
            for p in &held {
                if let Some(d) = dec.feed(p.level, p.duration_us) {
                    got.push(d.data);
                    dec.reset();
                }
            }
            assert_eq!(got, [data; 4], "{data:02X}");
        }
    }

    #[test]
    fn rejects_bad_timing() {
        let held = with_repeats(&build_frame(0xD0), 15);
        // 2600 µs HIGH is neither 500 nor 2000 (± 150).
        let bad: Vec<LevelDuration> = held
            .iter()
            .map(|p| {
                if p.level && p.duration_us == TE_LONG {
                    LevelDuration::new(true, 2600)
                } else {
                    *p
                }
            })
            .collect();
        assert!(feed_all(&mut LinearDelta3Decoder::new(), &bad).is_empty());
        // Alternating codes never confirm each other.
        let mut mixed = Vec::new();
        for i in 0..15 {
            for p in build_frame(if i % 2 == 0 { 0xD0 } else { 0xD1 }) {
                push(&mut mixed, p.level, p.duration_us);
            }
        }
        assert!(feed_all(&mut LinearDelta3Decoder::new(), &mixed).is_empty());
    }

    #[test]
    fn unit_test_key_file() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: LinearDelta3\nBit: 8\n\
                    Key: 00 00 00 00 00 00 00 D0\n";
        let dec = LinearDelta3Decoder::new();
        let d = dec
            .from_flipper_key(&FlipperKey::parse(text).unwrap())
            .unwrap();
        assert_eq!(d.data, 0xD0);
        let held = with_repeats(&dec.encode(&d, 0).unwrap(), 2);
        assert_eq!(
            feed_all(&mut LinearDelta3Decoder::new(), &held)[0].data,
            0xD0
        );
        let written = dec.to_flipper_key(&d).unwrap().to_text();
        let back = dec
            .from_flipper_key(&FlipperKey::parse(&written).unwrap())
            .unwrap();
        assert_eq!(format!("{back:?}"), format!("{d:?}"));
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&text.replace("Bit: 8", "Bit: 10")).unwrap())
            .is_none());
    }

    #[test]
    fn reference_raw_capture_decodes() {
        let Some(decodes) = decode_reference_capture("linear_delta3_raw.sub") else {
            return;
        };
        assert!(
            decodes.iter().any(|(n, _)| n == NAME),
            "{:?}",
            decodes.iter().map(|(n, _)| n).collect::<Vec<_>>()
        );
    }
}
