//! SMC5326 (SMC5326 / AX5326 trinary DIP-switch 25-bit fixed code) decoder + encoder.
//!
//! Port of Flipper-ARF's garage app `applications/system/subghz_garage/protocols/smc5326.c` (the
//! primary reference; doctor-firmware's and ARF's lib copies have the same wire behaviour; no copy
//! can switch buttons).
//!
//! **Air format** (AM, 315/433.92/868 MHz): te ≈ 300 µs (`te_short` 300, `te_long` 900, `te_delta`
//! 200); bit 1 = HIGH 3·te + LOW te, bit 0 = HIGH te + LOW 3·te, MSB first; stop HIGH te and a guard
//! LOW of 25·te. 25 bits: eight trinary DIP switches (2 bits each: 11 = +, 10 = o, 00 = −) in bits
//! 9..24, four button pairs in bits 1..8 (11 = pressed: B1..B4), and bit 0.
//!
//! **Decoder** (`subghz_protocol_decoder_smc5326_feed`): a LOW within 7200 ± 2400 µs starts a frame;
//! bit 0 = HIGH short (± 200) + LOW long (± 600), bit 1 = HIGH long + LOW short; a LOW ≥ 1800 µs
//! ends it. A 25-bit frame is reported only when it equals the previous one (`last_data`,
//! non-zero); TE = (every HIGH/LOW of the frame, stop bit included) / 101. After a decode the app's
//! reset needs a new start LOW, with `last_data` kept by the code after the callback.
//!
//! **Fields**: none in the C's generic block (it only displays the DIP rows and pressed buttons);
//! `extra` = TE.
//!
//! **Encoder** (`subghz_protocol_encoder_smc5326_get_upload`, `encoder.repeat` = 3): 25 bits with the
//! key's TE, stop HIGH TE, guard LOW 25·TE. `button` is ignored.
//!
//! **Key file**: `Protocol: SMC5326`, `Bit: 25`, `Key`, `TE` (required on load).

use super::g1_common::{
    fixed_code, generic_fields, generic_key, push, u32_line, with_repeats, TX_REPEAT,
};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "SMC5326",
    decoder: "SMC5326",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Generic (SMC5326)",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "SMC5326";
const TE_SHORT: u32 = 300;
const TE_LONG: u32 = 900;
const TE_DELTA: u32 = 200;
const MIN_COUNT_BIT: usize = 25;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    SaveDuration,
    CheckDuration,
}

/// SMC5326 decoder (`SubGhzProtocolDecoderSMC5326`).
pub struct Smc5326Decoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
    te: u32,
    last_data: u32,
    /// The last `feed` reported a frame (see `g1_common`): keep the post-callback state.
    just_emitted: bool,
}

fn decoded(data: u64, te: u32) -> DecodedSignal {
    DecodedSignal {
        encoder_capable: te != 0,
        extra: Some(te as u64),
        ..fixed_code(data, MIN_COUNT_BIT)
    }
}

/// One `get_upload` frame with the key's TE.
fn build_frame(data: u64, te: u32) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(MIN_COUNT_BIT * 2 + 2);
    for i in (0..MIN_COUNT_BIT).rev() {
        if (data >> i) & 1 == 1 {
            push(&mut out, true, te.wrapping_mul(3));
            push(&mut out, false, te);
        } else {
            push(&mut out, true, te);
            push(&mut out, false, te.wrapping_mul(3));
        }
    }
    push(&mut out, true, te);
    push(&mut out, false, te.wrapping_mul(25));
    out
}

impl Smc5326Decoder {
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

impl Default for Smc5326Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for Smc5326Decoder {
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
        // subghz_protocol_decoder_smc5326_reset
        self.step = Step::Reset;
        self.last_data = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        self.just_emitted = false;
        match self.step {
            Step::Reset => {
                if !level && duration_diff!(duration, TE_SHORT * 24) < TE_DELTA * 12 {
                    self.step = Step::SaveDuration;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.te = 0;
                }
            }
            Step::SaveDuration => {
                if level {
                    self.te_last = duration;
                    self.te = self.te.wrapping_add(duration);
                    self.step = Step::CheckDuration;
                }
            }
            Step::CheckDuration => {
                if level {
                    self.step = Step::Reset;
                    return None;
                }
                if duration >= TE_LONG * 2 {
                    self.step = Step::SaveDuration;
                    let mut out = None;
                    if self.decode_count_bit as usize == MIN_COUNT_BIT {
                        if self.last_data as u64 == self.decode_data && self.last_data != 0 {
                            self.te /= self.decode_count_bit as u32 * 4 + 1;
                            out = Some(decoded(self.decode_data, self.te));
                            // callback → the app resets the receiver (smc5326_reset).
                            self.step = Step::Reset;
                        }
                        self.last_data = self.decode_data as u32;
                    }
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.te = 0;
                    self.just_emitted = out.is_some();
                    return out;
                }
                self.te = self.te.wrapping_add(duration);
                if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA
                    && duration_diff!(duration, TE_LONG) < TE_DELTA * 3
                {
                    self.add_bit(0);
                    self.step = Step::SaveDuration;
                } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA * 3
                    && duration_diff!(duration, TE_SHORT) < TE_DELTA
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

    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        let te = decoded.extra? as u32;
        if te == 0 {
            return None;
        }
        Some(with_repeats(&build_frame(decoded.data, te), TX_REPEAT))
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

    /// 3 repeats: frame 1's guard starts the decoder, frame 2 is stored, frame 3 decodes.
    #[test]
    fn encode_decode_roundtrip() {
        for (data, te) in [(0x17D5580u64, 210), (0x0AAAAAB, 300), (0x1FFFFFF, 350)] {
            let wire = Smc5326Decoder::new().encode(&decoded(data, te), 0).unwrap();
            assert_eq!(wire.len(), 3 * 52);
            let got = feed_all(&mut Smc5326Decoder::new(), &wire);
            assert_eq!(got.len(), 1, "{data:07X}");
            assert_eq!(
                (got[0].data, got[0].data_count_bit, got[0].extra),
                (data, 25, Some(te as u64))
            );
        }
    }

    #[test]
    fn rejects_mismatch_and_bad_timing() {
        let mut wire = vec![LevelDuration::new(false, 7000)];
        wire.extend(build_frame(0x17D5580, 300));
        wire.extend(build_frame(0x17D5581, 300));
        assert!(feed_all(&mut Smc5326Decoder::new(), &wire).is_empty());
        // 24 bits.
        let short: Vec<LevelDuration> = with_repeats(&build_frame(0x17D5580, 300)[2..], 4);
        assert!(feed_all(&mut Smc5326Decoder::new(), &short).is_empty());
        // A 1700 µs HIGH (neither 300 ± 200 nor 900 ± 600) breaks every frame.
        let bad: Vec<LevelDuration> = with_repeats(&build_frame(0x17D5580, 300), 4)
            .into_iter()
            .map(|p| {
                if p.level && p.duration_us == 900 {
                    LevelDuration::new(true, 1700)
                } else {
                    p
                }
            })
            .collect();
        assert!(feed_all(&mut Smc5326Decoder::new(), &bad).is_empty());
    }

    #[test]
    fn unit_test_key_file() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: SMC5326\nBit: 25\n\
                    Key: 00 00 00 00 01 7D 55 80\nTE: 210\n";
        let dec = Smc5326Decoder::new();
        let d = dec
            .from_flipper_key(&FlipperKey::parse(text).unwrap())
            .unwrap();
        assert_eq!((d.data, d.extra), (0x17D5580, Some(210)));
        assert_eq!(
            feed_all(&mut Smc5326Decoder::new(), &dec.encode(&d, 0).unwrap())[0].data,
            0x17D5580
        );
        let written = dec.to_flipper_key(&d).unwrap().to_text();
        assert!(
            written.ends_with("Key: 00 00 00 00 01 7D 55 80\nTE: 210\n"),
            "{written}"
        );
        let back = dec
            .from_flipper_key(&FlipperKey::parse(&written).unwrap())
            .unwrap();
        assert_eq!(format!("{back:?}"), format!("{d:?}"));
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&text.replace("TE: 210\n", "")).unwrap())
            .is_none());
    }

    #[test]
    fn reference_raw_capture_decodes() {
        let Some(decodes) = decode_reference_capture("smc5326_raw.sub") else {
            return;
        };
        assert!(
            decodes.iter().any(|(n, _)| n == NAME),
            "{:?}",
            decodes.iter().map(|(n, _)| n).collect::<Vec<_>>()
        );
    }
}
