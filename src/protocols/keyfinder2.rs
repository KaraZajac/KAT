//! KeyFinder2: 11-bit fixed code of a second key-finder family (3 wide header bits, 8 narrow bits
//! ending in a 3-bit colour button), decoder + encoder.
//!
//! Port of Flipper-ARF's garage app `applications/system/subghz_garage/protocols/keyfinder2.c`
//! (registered in RX group g1, TX plugin `tx_keyfinder2`). doctor-firmware, RogueMaster and FoxFW
//! register the same decoder and encoder: feed, upload and fields are identical, they differ in
//! style only (doctor and RogueMaster flag it a sensor; FoxFW loads its encoder and its text from SD
//! plugins with the same code). No copy has custom buttons.
//!
//! **Air format** (AM, 433.92 MHz): 11 bits MSB first, each a HIGH then a LOW. The first 3 (the
//! header) are wide: 1 = HIGH 1500 + LOW 2500, 0 = HIGH 2500 + LOW 1500; the other 8 narrow
//! (te_short = 350 µs, te_long = 1020 µs): 1 = HIGH 350 + LOW 1020, 0 = HIGH 1020 + LOW 350. No
//! preamble and no gap: the encoder's copies follow each other directly.
//!
//! **Decoder** (`subghz_protocol_decoder_keyfinder2_feed`): Reset waits for a HIGH within 350 µs of
//! 1500 or 2500 (it starts the first header bit); header bits take HIGH/LOW pairs within 350 µs of
//! 1500 / 2500 (short + long = 1, long + short = 0), the narrow bits pairs within te_delta = 150 µs
//! of 350 / 1020; anything else returns to Reset. The 11th bit's LOW reports the frame and the
//! decoder returns to Reset, so every copy decodes. Nothing is checked beyond the timing.
//!
//! **Fields** (`subghz_protocol_keyfinder2_check_remote_controller`): the C sets the button only,
//! btn = data & 0x7 (3 Purple, 4 Blue, 5 Green, 6 Red, else "Unknown"), and shows the key as
//! data & 0x7FF. KAT: `serial` None, `button` = btn, `crc_valid` false.
//!
//! **Encoder** (`subghz_protocol_encoder_keyfinder2_get_upload`, `encoder.repeat` = 5): the 11
//! bits as above, 5 copies back to back. The captured code is sent (`button` is ignored: no copy
//! of the C switches buttons).
//!
//! **Key file**: `Protocol: KeyFinder2`, `Bit: 11`, `Key` (`subghz_block_generic_serialize`);
//! loading requires `Bit: 11`.

use super::f_common::{add_bit, push, with_repeats};
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

/// Metadata (see `info.rs`): `SubGhzProtocolFlag_433 | AM`, `SubGhzProtocolTypeStatic`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "KeyFinder",
    category: Category::Gate,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

pub(super) const NAME: &str = "KeyFinder2";
const TE_SHORT: u32 = 350;
const TE_LONG: u32 = 1020;
const TE_DELTA: u32 = 150;
const MIN_COUNT_BIT: usize = 11;
/// `KEYFINDER2_TE_WIDE_SHORT` / `_LONG` / `_DELTA`, `KEYFINDER2_HEADER_BITS`.
const TE_WIDE_SHORT: u32 = 1500;
const TE_WIDE_LONG: u32 = 2500;
const TE_WIDE_DELTA: u32 = 350;
const HEADER_BITS: usize = 3;
/// `subghz_protocol_encoder_keyfinder2_alloc`: repeat = 5.
const TX_REPEAT: usize = 5;

/// `subghz_protocol_keyfinder2_color` (codes it does not name: "Btn 0xN").
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    match code {
        3 => Some("Purple"),
        4 => Some("Blue"),
        5 => Some("Green"),
        6 => Some("Red"),
        _ => None,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    SaveWide,
    CheckWide,
    SaveDuration,
    CheckDuration,
}

/// KeyFinder2 decoder (`SubGhzProtocolDecoderKeyFinder2`).
pub struct KeyFinder2Decoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
}

/// A decode (or a loaded key) of `data`.
fn decoded(data: u64) -> DecodedSignal {
    DecodedSignal {
        serial: None,
        button: Some((data & 0x7) as u8),
        counter: None,
        crc_valid: false,
        data,
        data_count_bit: MIN_COUNT_BIT,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

/// One `get_upload` copy: the header bits wide, the rest narrow.
fn build_frame(data: u64) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(MIN_COUNT_BIT * 2);
    for i in (1..=MIN_COUNT_BIT).rev() {
        let (short, long) = if i > MIN_COUNT_BIT - HEADER_BITS {
            (TE_WIDE_SHORT, TE_WIDE_LONG)
        } else {
            (TE_SHORT, TE_LONG)
        };
        if (data >> (i - 1)) & 1 == 1 {
            push(&mut out, true, short);
            push(&mut out, false, long);
        } else {
            push(&mut out, true, long);
            push(&mut out, false, short);
        }
    }
    out
}

impl KeyFinder2Decoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
        }
    }
}

impl Default for KeyFinder2Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for KeyFinder2Decoder {
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

    /// `subghz_protocol_decoder_keyfinder2_reset` (doctor: `common_reset`): the parser step only.
    fn reset(&mut self) {
        self.step = Step::Reset;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let wide = |d: u32, t: u32| duration_diff!(d, t) < TE_WIDE_DELTA;
        let narrow = |d: u32, t: u32| duration_diff!(d, t) < TE_DELTA;
        match self.step {
            Step::Reset => {
                if level && (wide(duration, TE_WIDE_SHORT) || wide(duration, TE_WIDE_LONG)) {
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.te_last = duration;
                    self.step = Step::CheckWide;
                }
            }
            Step::SaveWide | Step::SaveDuration => {
                if level {
                    self.te_last = duration;
                    self.step = if self.step == Step::SaveWide { Step::CheckWide } else { Step::CheckDuration };
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::CheckWide => {
                if !level && wide(self.te_last, TE_WIDE_SHORT) && wide(duration, TE_WIDE_LONG) {
                    add_bit(&mut self.decode_data, &mut self.decode_count_bit, true);
                } else if !level && wide(self.te_last, TE_WIDE_LONG) && wide(duration, TE_WIDE_SHORT) {
                    add_bit(&mut self.decode_data, &mut self.decode_count_bit, false);
                } else {
                    self.step = Step::Reset;
                    return None;
                }
                self.step = if usize::from(self.decode_count_bit) == HEADER_BITS {
                    Step::SaveDuration
                } else {
                    Step::SaveWide
                };
            }
            Step::CheckDuration => {
                if !level && narrow(self.te_last, TE_SHORT) && narrow(duration, TE_LONG) {
                    add_bit(&mut self.decode_data, &mut self.decode_count_bit, true);
                } else if !level && narrow(self.te_last, TE_LONG) && narrow(duration, TE_SHORT) {
                    add_bit(&mut self.decode_data, &mut self.decode_count_bit, false);
                } else {
                    self.step = Step::Reset;
                    return None;
                }
                if usize::from(self.decode_count_bit) == MIN_COUNT_BIT {
                    // callback (its receiver reset), then the C's own step = Reset
                    self.step = Step::Reset;
                    return Some(decoded(self.decode_data));
                }
                self.step = Step::SaveDuration;
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        (decoded.data_count_bit == MIN_COUNT_BIT)
            .then(|| with_repeats(&build_frame(decoded.data), TX_REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        (key.bits == MIN_COUNT_BIT).then(|| decoded(key.key))
    }

    fn to_flipper_key(&self, d: &DecodedSignal) -> Option<FlipperKey> {
        (d.data_count_bit == MIN_COUNT_BIT).then(|| FlipperKey::for_protocol(NAME, MIN_COUNT_BIT, d.data))
    }
}

#[cfg(test)]
mod tests {
    use super::super::f_common::test_support::*;
    use super::*;
    use crate::protocols::ProtocolRegistry;

    /// The transmission after a silence (the C's encoder round trip feeds the upload as it is).
    fn on_air(data: u64) -> Vec<LevelDuration> {
        let mut wire = vec![LevelDuration::new(false, 20_000)];
        for p in KeyFinder2Decoder::new().encode(&decoded(data), 0).unwrap() {
            push(&mut wire, p.level, p.duration_us);
        }
        wire
    }

    /// Every copy decodes (the 11th bit's LOW reports, the next copy's first HIGH opens a frame), as
    /// in the C's round trip; on air the last copy's final LOW runs into the silence after the
    /// transmission, so a receiver gets 4 of the 5.
    #[test]
    fn encode_decode_roundtrip() {
        for data in [0x5A6u64, 0x000, 0x7FF, 0x403, 0x0AC] {
            let got = feed_all(&mut KeyFinder2Decoder::new(), &on_air(data));
            assert_eq!(got.len(), 5, "{data:03X}");
            for d in &got {
                assert_eq!((d.data, d.data_count_bit), (data, 11));
                assert_eq!((d.serial, d.button), (None, Some((data & 7) as u8)));
                assert!(d.encoder_capable && !d.crc_valid && d.counter.is_none());
            }
            let mut silence_after = on_air(data);
            push(&mut silence_after, false, 20_000);
            assert_eq!(feed_all(&mut KeyFinder2Decoder::new(), &silence_after).len(), 4);
        }
    }

    #[test]
    fn upload_layout() {
        let up = KeyFinder2Decoder::new().encode(&decoded(0x401), 0).unwrap();
        let p = |i: usize| (up[i].level, up[i].duration_us);
        assert_eq!((p(0), p(1)), ((true, 1500), (false, 2500))); // bit 10 = 1, wide
        assert_eq!((p(2), p(3)), ((true, 2500), (false, 1500))); // bit 9 = 0, wide
        assert_eq!((p(4), p(5)), ((true, 2500), (false, 1500))); // bit 8 = 0, wide
        assert_eq!((p(6), p(7)), ((true, 1020), (false, 350))); // bit 7 = 0, narrow
        assert_eq!((p(20), p(21)), ((true, 350), (false, 1020))); // bit 0 = 1
        // 22 entries a copy, no gap; 5 copies (LOW then HIGH at each seam: nothing merges).
        assert_eq!(up.len(), 22 * 5);
        assert_eq!(native_buttons(NAME, 6), Some("Red"));
        assert_eq!(native_buttons(NAME, 7), None);
    }

    #[test]
    fn windows_and_lengths() {
        let one = |data: u64| {
            let mut w = vec![LevelDuration::new(false, 20_000)];
            for p in build_frame(data) {
                push(&mut w, p.level, p.duration_us);
            }
            w
        };
        // 0x5A6 = 101 1010 0110: [1] HIGH 1500 [2] LOW 2500 (wide 1), [3] HIGH 2500 [4] LOW 1500
        // (wide 0), [7] HIGH 350 [8] LOW 1020 (narrow 1), [9] HIGH 1020 [10] LOW 350 (narrow 0).
        for (i, d, ok) in [(1usize, 1150u32, false), (1, 1151, true), (1, 1849, true), (1, 1850, false),
                           (2, 2150, false), (2, 2151, true), (2, 2849, true), (2, 2850, false),
                           (3, 2151, true), (3, 2849, true), (3, 2850, false),
                           (7, 200, false), (7, 201, true), (7, 499, true), (7, 500, false),
                           (8, 870, false), (8, 871, true), (8, 1169, true), (8, 1170, false),
                           (10, 200, false), (10, 201, true)] {
            let mut w = one(0x5A6);
            w[i].duration_us = d;
            assert_eq!(feed_all(&mut KeyFinder2Decoder::new(), &w).len(), usize::from(ok), "pulse {i} = {d}");
        }
        // A wide first HIGH can open the frame as either bit, the rest must follow: 10 bits only.
        let mut short = one(0x5A6);
        short.drain(19..21);
        assert!(feed_all(&mut KeyFinder2Decoder::new(), &short).is_empty());
        // Narrow bits where the header should be: nothing.
        let narrow: Vec<LevelDuration> = one(0x5A6)
            .into_iter()
            .map(|p| match p.duration_us {
                1500 => LevelDuration::new(p.level, 350),
                2500 => LevelDuration::new(p.level, 1020),
                _ => p,
            })
            .collect();
        assert!(feed_all(&mut KeyFinder2Decoder::new(), &narrow).is_empty());
    }

    #[test]
    fn key_file_round_trip() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: KeyFinder2\nBit: 11\n\
                    Key: 00 00 00 00 00 00 05 A6\n";
        let dec = KeyFinder2Decoder::new();
        let d = dec.from_flipper_key(&FlipperKey::parse(text).unwrap()).unwrap();
        assert_eq!((d.data, d.button), (0x5A6, Some(6)));
        let written = super::super::flipper_key::testing::assert_round_trip(&dec, &d);
        assert!(written.contains("Protocol: KeyFinder2\nBit: 11\nKey: 00 00 00 00 00 00 05 A6\n"), "{written}");
        assert!(dec.from_flipper_key(&FlipperKey::parse(&text.replace("Bit: 11", "Bit: 12")).unwrap()).is_none());
    }

    #[test]
    fn jittered_frames_decode() {
        let mut rng = Rng(0x0CE2_F1D0);
        for _ in 0..50 {
            let data = rng.below(0x800);
            let wire = jitter(&on_air(data), &mut rng, 100);
            let got = feed_all(&mut KeyFinder2Decoder::new(), &wire);
            assert_eq!(got.len(), 5, "{data:03X}");
            assert!(got.iter().all(|d| d.data == data));
        }
    }

    /// Through the whole registry: KeyFinder2's frames and nothing else.
    #[test]
    fn registry_reads_keyfinder2_only() {
        let mut rng = Rng(0x0433_0002);
        for _ in 0..20 {
            let data = rng.below(0x800);
            let mut reg = ProtocolRegistry::new();
            let got: Vec<String> = reg
                .process_signal_stream_rf(&on_air(data), 433_920_000, Some(crate::capture::RfModulation::AM))
                .into_iter()
                .map(|(n, d, _)| format!("{n} {:03X}", d.data))
                .collect();
            assert_eq!(got, vec![format!("{NAME} {data:03X}"); 5]);
        }
    }
}
