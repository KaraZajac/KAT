//! Intertechno V3: the self-learning Intertechno (KlikAanKlikUit "new") code of 433 MHz sockets,
//! switches and dimmers — 32-bit on/off frames and 36-bit dimming frames — decoder + encoder.
//!
//! Port of doctor-firmware `lib/subghz/protocols/intertechno_v3.c` (the stock Flipper protocol,
//! registered by doctor, Momentum, RogueMaster and FoxFW; Flipper-ARF ships its `lib` and garage-app
//! copies unregistered). Every copy decodes and encodes identically; they differ in style (doctor /
//! RogueMaster / FoxFW use `common.c`, FoxFW loads the encoder from an SD plugin) and in the display
//! (doctor / Momentum / RogueMaster add the channel and On/Off / dimming level, ARF only `SN:`).
//! ARF's `lib` copy turns on the custom-button D-pad, but "every direction re-sends the originally
//! captured code unchanged"; no copy switches buttons. Unit-test captures:
//! `intertechno_v3_raw.sub`, `intertechno_v3.sub`.
//!
//! **Air format** (AM; the C's flags: 315, 433 and 868 MHz): T = 275 µs. A frame is a start bit
//! (HIGH T, LOW 10·T) then 32 or 36 cells of four pulses: 0 = T,T,T,5T; 1 = T,5T,T,T; the dimming
//! frames' 28th cell is the "dimm" cell T,T,T,T. The encoder starts each copy with a stop bit
//! HIGH T + LOW 38·T, which also ends the previous frame.
//!
//! **Decoder** (`subghz_protocol_decoder_intertechno_v3_feed`): Reset waits for a LOW with
//! |d − 37·T| < 2250; then HIGH T ± 150 and LOW |d − 10·T| < 450 (the start bit); then cells:
//! HIGH T, LOW (saved), HIGH T, LOW. The saved LOW and the second HIGH make the bit: T ± 150 + T =
//! 0, 5·T ± 300 + T = 1, and — only as the 28th bit — T ± 300 + T = 0 (the dimm cell); the closing
//! LOW must be T ± 150 or 5·T ± 300. A LOW ≥ 11·T where a cell's first LOW belongs ends the frame:
//! exactly 32 or 36 bits are reported. Without a report the decoder goes on from the start bit (the
//! long LOW was the next frame's stop bit); with one, the callback's receiver reset sends it back to
//! Reset, so the stop bit that closed a frame does not open the next one and only every other copy
//! of a held button decodes. No check.
//!
//! **Fields** (`subghz_protocol_intertechno_v3_check_remote_controller`): 32 bits — serial = bits
//! 31..6 (26 bits), "all channels" = bit 5, button = bit 4 (1 On, 0 Off), channel = ~bits 3..0;
//! 36 bits — serial = bits 35..10, "all channels" = bit 9, channel = ~bits 7..4, button = bits
//! 3..0 (the dimming level, shown as 6.67 % steps). The C keeps the channel in its `cnt` (32 when
//! all channels): KAT's `counter` (as Dooya's channel).
//!
//! **Encoder** (`subghz_protocol_encoder_intertechno_v3_get_upload`, `encoder.repeat` = 3): HIGH T,
//! LOW 38·T, HIGH T, LOW 10·T, then the cells (in a 36-bit frame bit 8 is always sent as the dimm
//! cell); the captured code (`button` is ignored, as in every reference).
//!
//! **Key file**: `Protocol: Intertechno_V3`, `Bit: 32` or `36` (both deserializers reject other
//! counts), `Key`.

use super::f_common::{add_bit, push, with_repeats};
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

/// Metadata (see `info.rs`): `SubGhzProtocolFlag_315 | 433 | 868 | AM`, `SubGhzProtocolTypeStatic`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Intertechno",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

pub(super) const NAME: &str = "Intertechno_V3";
const TE_SHORT: u32 = 275;
const TE_LONG: u32 = 1375;
const TE_DELTA: u32 = 150;
const MIN_COUNT_BIT: usize = 32;
/// `INTERTECHNO_V3_DIMMING_COUNT_BIT`.
const DIMMING_COUNT_BIT: usize = 36;
/// The C's `cnt` for a frame addressed to all channels (`1 << 5`).
pub(super) const ALL_CHANNELS: u16 = 1 << 5;
/// `subghz_protocol_encoder_common_alloc(…, 3, 256)`.
const TX_REPEAT: usize = 3;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    StartSync,
    FoundSync,
    StartDuration,
    SaveDuration,
    CheckDuration,
    EndDuration,
}

/// Intertechno V3 decoder (`SubGhzProtocolDecoderIntertechno_V3`).
pub struct IntertechnoV3Decoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
}

/// (serial, channel `cnt`, button) of a frame (`check_remote_controller`).
fn fields(data: u64, bits: usize) -> (u32, u16, u8) {
    match bits {
        MIN_COUNT_BIT => {
            let cnt = if (data >> 5) & 1 == 1 { ALL_CHANNELS } else { (!data & 0xF) as u16 };
            (((data >> 6) & 0x3FF_FFFF) as u32, cnt, ((data >> 4) & 1) as u8)
        }
        DIMMING_COUNT_BIT => {
            let cnt = if (data >> 9) & 1 == 1 { ALL_CHANNELS } else { (!(data >> 4) & 0xF) as u16 };
            (((data >> 10) & 0x3FF_FFFF) as u32, cnt, (data & 0xF) as u8)
        }
        _ => (0, 0, 0),
    }
}

/// A decode (or a loaded key) of `data` with `bits` (32 or 36) bits.
fn decoded(data: u64, bits: usize) -> DecodedSignal {
    let (serial, channel, button) = fields(data, bits);
    DecodedSignal {
        serial: Some(serial),
        button: Some(button),
        counter: Some(channel),
        crc_valid: false,
        data,
        data_count_bit: bits,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

/// One `get_upload` frame of `bits` bits.
fn build_frame(data: u64, bits: usize) -> Vec<LevelDuration> {
    let (t, t5) = (TE_SHORT, TE_LONG);
    let mut out = Vec::with_capacity(4 + bits * 4);
    // stop bit (it closes the previous frame), then the start bit
    for (k, d) in [t, t * 38, t, t * 10].into_iter().enumerate() {
        push(&mut out, k % 2 == 0, d);
    }
    for i in (1..=bits).rev() {
        let cell = if bits == DIMMING_COUNT_BIT && i == 9 {
            [t, t, t, t] // dimm
        } else if (data >> (i - 1)) & 1 == 1 {
            [t, t5, t, t]
        } else {
            [t, t, t, t5]
        };
        for (k, d) in cell.into_iter().enumerate() {
            push(&mut out, k % 2 == 0, d);
        }
    }
    out
}

impl IntertechnoV3Decoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
        }
    }
}

impl Default for IntertechnoV3Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for IntertechnoV3Decoder {
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

    /// `subghz_protocol_decoder_common_reset`: the parser step only.
    fn reset(&mut self) {
        self.step = Step::Reset;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let short = |d: u32| duration_diff!(d, TE_SHORT) < TE_DELTA;
        match self.step {
            Step::Reset => {
                if !level && duration_diff!(duration, TE_SHORT * 37) < TE_DELTA * 15 {
                    self.step = Step::StartSync;
                }
            }
            Step::StartSync => {
                self.step = if level && short(duration) { Step::FoundSync } else { Step::Reset };
            }
            Step::FoundSync => {
                if !level && duration_diff!(duration, TE_SHORT * 10) < TE_DELTA * 3 {
                    self.step = Step::StartDuration;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::StartDuration => {
                self.step = if level && short(duration) { Step::SaveDuration } else { Step::Reset };
            }
            Step::SaveDuration => {
                if level {
                    self.step = Step::Reset;
                    return None;
                }
                if duration >= TE_SHORT * 11 {
                    self.step = Step::StartSync;
                    let bits = usize::from(self.decode_count_bit);
                    if bits == MIN_COUNT_BIT || bits == DIMMING_COUNT_BIT {
                        // callback → the app resets the receiver (common_reset).
                        self.step = Step::Reset;
                        return Some(decoded(self.decode_data, bits));
                    }
                    return None;
                }
                self.te_last = duration;
                self.step = Step::CheckDuration;
            }
            Step::CheckDuration => {
                if !level || !short(duration) {
                    self.step = Step::Reset;
                } else if short(self.te_last) {
                    add_bit(&mut self.decode_data, &mut self.decode_count_bit, false);
                    self.step = Step::EndDuration;
                } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA * 2 {
                    add_bit(&mut self.decode_data, &mut self.decode_count_bit, true);
                    self.step = Step::EndDuration;
                } else if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA * 2 && self.decode_count_bit == 27 {
                    // dimm state
                    add_bit(&mut self.decode_data, &mut self.decode_count_bit, false);
                    self.step = Step::EndDuration;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::EndDuration => {
                self.step = if !level && (short(duration) || duration_diff!(duration, TE_LONG) < TE_DELTA * 2) {
                    Step::StartDuration
                } else {
                    Step::Reset
                };
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        let bits = decoded.data_count_bit;
        (bits == MIN_COUNT_BIT || bits == DIMMING_COUNT_BIT).then(|| with_repeats(&build_frame(decoded.data, bits), TX_REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        (key.bits == MIN_COUNT_BIT || key.bits == DIMMING_COUNT_BIT).then(|| decoded(key.key, key.bits))
    }

    fn to_flipper_key(&self, d: &DecodedSignal) -> Option<FlipperKey> {
        let bits = d.data_count_bit;
        (bits == MIN_COUNT_BIT || bits == DIMMING_COUNT_BIT).then(|| FlipperKey::for_protocol(NAME, bits, d.data))
    }
}

#[cfg(test)]
pub(super) mod testing {
    //! Frames for other modules' tests and the parity tools.
    use super::*;

    /// A decode of `data` with `bits` (32 or 36) bits, as a key file gives it.
    pub fn frame(data: u64, bits: usize) -> DecodedSignal {
        decoded(data & ((1u64 << bits) - 1), bits)
    }
}

#[cfg(test)]
mod tests {
    use super::super::f_common::test_support::*;
    use super::*;

    const UNIT_TEST_KEY: &str = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
        Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Intertechno_V3\nBit: 32\nKey: 00 00 00 00 3F 86 C5 9F\n";

    /// `n` copies of the upload's frame, closed by a final stop bit.
    fn on_air(data: u64, bits: usize, n: usize) -> Vec<LevelDuration> {
        let mut wire = Vec::new();
        for _ in 0..n {
            for p in build_frame(data, bits) {
                push(&mut wire, p.level, p.duration_us);
            }
        }
        push(&mut wire, true, TE_SHORT);
        push(&mut wire, false, TE_SHORT * 38);
        wire
    }

    #[test]
    fn fields_of_the_c_examples() {
        // The C's comment: Key:0x3F86C59F (32 bits) and Key:0x42D2E8856 (36 bits).
        let d = testing::frame(0x3F86_C59F, 32);
        assert_eq!((d.serial, d.button, d.counter), (Some(0x0FE1B16), Some(1), Some(0)));
        let d = testing::frame(0x4_2D2E_8856, 36);
        assert_eq!((d.serial, d.button, d.counter), (Some(0x10B4BA2), Some(6), Some(0xA)));
        // "all channels" bits.
        assert_eq!(testing::frame(0x20, 32).counter, Some(ALL_CHANNELS));
        assert_eq!(testing::frame(0x200, 36).counter, Some(ALL_CHANNELS));
    }

    #[test]
    fn encode_decode_roundtrip() {
        for (data, bits) in [(0x3F86_C59Fu64, 32usize), (0, 32), (0xFFFF_FFFF, 32), (0x4_2D2E_8856, 36), (0xF_FFFF_FEFF, 36)] {
            // Copies 1 and 3 decode: a report's reset loses the stop bit copy 2 needs.
            let got = feed_all(&mut IntertechnoV3Decoder::new(), &on_air(data, bits, 3));
            assert_eq!(got.len(), 2, "{data:X}/{bits}");
            for d in &got {
                assert_eq!(*d, testing::frame(data, bits));
            }
        }
        // A 36-bit frame's bit 8 is sent as the dimm cell, which reads as 0.
        let got = feed_all(&mut IntertechnoV3Decoder::new(), &on_air(0x100, 36, 1));
        assert_eq!(got.iter().map(|d| d.data).collect::<Vec<_>>(), [0]);
    }

    #[test]
    fn upload_layout() {
        let up = IntertechnoV3Decoder::new().encode(&testing::frame(0x8000_0000, 32), 0).unwrap();
        let p = |i: usize| (up[i].level, up[i].duration_us);
        assert_eq!((p(0), p(1), p(2), p(3)), ((true, 275), (false, 10_450), (true, 275), (false, 2750)));
        // bit 31 = 1: T, 5T, T, T; bit 30 = 0: T, T, T, 5T.
        assert_eq!((p(4), p(5), p(6), p(7)), ((true, 275), (false, 1375), (true, 275), (false, 275)));
        assert_eq!((p(8), p(9), p(10), p(11)), ((true, 275), (false, 275), (true, 275), (false, 1375)));
        // 132 entries a copy (each copy starts HIGH), 3 copies: 396 pulses, as the C.
        assert_eq!(up.len(), 396);
        // 36 bits: the 28th cell (i = 9) is T,T,T,T.
        let up = IntertechnoV3Decoder::new().encode(&testing::frame(0xF_FFFF_FFFF, 36), 0).unwrap();
        let cell = 4 + (36 - 9) * 4;
        assert!(up[cell..cell + 4].iter().all(|p| p.duration_us == 275));
        assert_eq!(up[cell - 3].duration_us, 1375);
    }

    #[test]
    fn rejects_other_lengths_and_timing() {
        // 31 / 33 / 35 bits never report.
        for bits in [31usize, 33, 35] {
            let mut w = Vec::new();
            for p in build_frame(0x1234_5678, bits) {
                push(&mut w, p.level, p.duration_us);
            }
            push(&mut w, true, TE_SHORT);
            push(&mut w, false, TE_SHORT * 38);
            assert!(feed_all(&mut IntertechnoV3Decoder::new(), &w).is_empty(), "{bits} bits");
        }
        // Stop-bit window 7926..=12424 µs.
        for (g, ok) in [(7925u32, false), (7926, true), (12_424, true), (12_425, false)] {
            let mut w = on_air(0x3F86_C59F, 32, 1);
            w[1].duration_us = g;
            assert_eq!(feed_all(&mut IntertechnoV3Decoder::new(), &w).len(), usize::from(ok), "stop {g}");
        }
        // Start-bit LOW window 2301..=3199 µs.
        for (g, ok) in [(2300u32, false), (2301, true), (3199, true), (3200, false)] {
            let mut w = on_air(0x3F86_C59F, 32, 1);
            w[3].duration_us = g;
            assert_eq!(feed_all(&mut IntertechnoV3Decoder::new(), &w).len(), usize::from(ok), "start {g}");
        }
        // The frame ends on a LOW ≥ 3025 µs (11·T) where a cell's first LOW belongs.
        let mut w = on_air(0x3F86_C59F, 32, 1);
        let n = w.len();
        w[n - 1].duration_us = 3025;
        assert_eq!(feed_all(&mut IntertechnoV3Decoder::new(), &w).len(), 1);
        w[n - 1].duration_us = 3024;
        assert!(feed_all(&mut IntertechnoV3Decoder::new(), &w).is_empty());
    }

    /// The wide "dimm" window (saved LOW T ± 300) applies only to the 28th bit.
    #[test]
    fn dimm_window_only_on_the_28th_bit() {
        let frame = build_frame(0, 36);
        // The 28th cell's saved LOW is entry 4 + 27·4 + 1; 500 µs is outside T ± 150, inside T ± 300.
        let mut w = frame.clone();
        w[4 + 27 * 4 + 1].duration_us = 500;
        w.extend([LevelDuration::new(true, 275), LevelDuration::new(false, 10_450)]);
        assert_eq!(feed_all(&mut IntertechnoV3Decoder::new(), &w).len(), 1);
        let mut w = frame;
        w[4 + 26 * 4 + 1].duration_us = 500;
        w.extend([LevelDuration::new(true, 275), LevelDuration::new(false, 10_450)]);
        assert!(feed_all(&mut IntertechnoV3Decoder::new(), &w).is_empty());
    }

    #[test]
    fn key_file_roundtrip() {
        let dec = IntertechnoV3Decoder::new();
        let d = dec.from_flipper_key(&FlipperKey::parse(UNIT_TEST_KEY).unwrap()).unwrap();
        assert_eq!((d.data, d.data_count_bit, d.serial, d.button), (0x3F86_C59F, 32, Some(0x0FE1B16), Some(1)));
        let text = super::super::flipper_key::testing::assert_round_trip(&dec, &d);
        assert!(text.contains("Protocol: Intertechno_V3\nBit: 32\nKey: 00 00 00 00 3F 86 C5 9F\n"), "{text}");
        let dimm = testing::frame(0x4_2D2E_8856, 36);
        let text = super::super::flipper_key::testing::assert_round_trip(&dec, &dimm);
        assert!(text.contains("Bit: 36\nKey: 00 00 00 04 2D 2E 88 56\n"), "{text}");
        for bits in ["24", "34"] {
            let k = FlipperKey::parse(&UNIT_TEST_KEY.replace("Bit: 32", &format!("Bit: {bits}"))).unwrap();
            assert!(dec.from_flipper_key(&k).is_none());
        }
    }

    /// Flipper's `subghz_decoder_intertechno_v3_test` capture through the whole registry: 12 frames
    /// (as doctor / ARF / Momentum / RogueMaster / FoxFW), all the unit-test key, nothing else.
    #[test]
    fn unit_test_capture_decodes() {
        let Some(path) = reference_file(ARF_UNIT_TESTS, "intertechno_v3_raw.sub") else {
            return;
        };
        let got = registry_decodes(&path);
        let ours: Vec<&DecodedSignal> = got.iter().filter(|(n, _)| n == NAME).map(|(_, d)| d).collect();
        assert_eq!(ours.len(), 12);
        assert!(ours.iter().all(|d| d.data == 0x3F86_C59F && d.data_count_bit == 32));
        assert_eq!(got.len(), ours.len(), "another protocol took a frame");
    }

    /// `subghz_encoder_intertechno_v3_test`: the unit-test key's upload decodes back once through the
    /// registry (copy 1, closed by copy 2's stop bit; copy 3 has no stop bit after it), as the C.
    #[test]
    fn unit_test_key_encodes_back() {
        let reg = super::super::ProtocolRegistry::new();
        let (name, d) = reg.decode_key(&FlipperKey::parse(UNIT_TEST_KEY).unwrap()).unwrap();
        assert_eq!(name, NAME);
        let tx = reg.get(&name).unwrap().encode(&d, 0).unwrap();
        let mut reg = super::super::ProtocolRegistry::new();
        let back = reg.process_signal_stream(&tx, 433_920_000);
        assert_eq!(back.len(), 1, "{back:?}");
        assert!(back.iter().all(|(n, x, _)| n == NAME && x.data == 0x3F86_C59F), "{back:?}");
    }

    #[test]
    fn jittered_frames_decode() {
        let mut rng = Rng(0x17E2_7EC4);
        for i in 0..50 {
            let bits = if i % 3 == 0 { 36 } else { 32 };
            let data = rng.below(1 << bits) & !(u64::from(bits == 36) << 8);
            let wire = jitter(&on_air(data, bits, 2), &mut rng, 100);
            let got = feed_all(&mut IntertechnoV3Decoder::new(), &wire);
            assert!(!got.is_empty(), "{data:X}/{bits}");
            assert!(got.iter().all(|d| d.data == data && d.data_count_bit == bits));
        }
    }
}
