//! KeyFinder: 24-bit fixed code of the four-colour key-finder remotes (a 20-bit serial and a 4-bit
//! ID per colour), decoder + encoder.
//!
//! Port of Flipper-ARF's garage app `applications/system/subghz_garage/protocols/keyfinder.c`
//! (@xMasterX, 2026.03; registered in RX group g1, TX plugin `tx_keyfinder`). doctor-firmware,
//! RogueMaster, Momentum and FoxFW register the same decoder and encoder: feed, upload and fields
//! are identical, they differ in style only (doctor and RogueMaster flag it a sensor; FoxFW loads
//! its encoder and its text from SD plugins, `enc_keyfinder.c` / `decode_keyfinder.c`, with the
//! same code). Flipper-ARF's own `lib/subghz/protocols/keyfinder.c` (which ARF's registry leaves
//! out) is the one copy that does more: custom buttons on TX, the D-pad re-encoding the key with
//! one of the four colours of the C's key samples (UP 0xE Red, DOWN 0xB Purple, LEFT 0xD Green,
//! RIGHT 0xC Blue; OK = the captured ID). KAT takes the union: the garage app's decoder, text and
//! upload, plus that button switch.
//!
//! **Air format** (AM, 433.92 MHz): te_short = 400 µs, te_long = 1200 µs, te_delta = 150 µs. 24
//! bits MSB first, each a HIGH then a LOW: 1 = HIGH 400 + LOW 1200, 0 = HIGH 1200 + LOW 400; then
//! three HIGH 400 + LOW 400 pairs, a HIGH 400 and the 4000 µs (10·te_short) gap.
//!
//! **Decoder** (`subghz_protocol_decoder_keyfinder_feed`): Reset waits for a LOW with |d − 4000| <
//! 750; then bit pairs (HIGH and LOW within te_delta of 400 / 1200: short + long = 1, long + short
//! = 0, anything else back to Reset). Once 24 bits are in, a HIGH 400 ± 150 counts an end pulse, a
//! LOW 400 ± 150 is skipped, anything else clears the frame; the fourth end pulse arms Finish, and
//! the next pulse (whatever it is: on air, the gap) reports the 24 bits. The C keeps its end-pulse
//! counter outside `reset()` (which clears the parser step only), so KAT's reset does the same.
//! A report consumes the gap that would open the next copy: of the encoder's 5 back-to-back copies
//! the 2nd and the 4th decode (as in the C).
//!
//! **Fields** (`subghz_protocol_keyfinder_check_remote_controller`): serial = data >> 4 (20 bits),
//! ID (button) = data & 0xF; the C also shows the key bit-reversed ("Yek"). No checksum.
//!
//! **Encoder** (`subghz_protocol_encoder_keyfinder_get_upload`, `encoder.repeat` = 5): the 24 bits,
//! the three end pairs, the end HIGH and the gap, 5 copies. `button` (KAT's native code): Flipper-ARF
//! lib's re-encoding, data = serial << 4 | button, for 0..=0xF (its D-pad offers the four colours,
//! `DPAD`); anything else keeps the captured ID. Static code: no counter.
//!
//! **Key file**: `Protocol: KeyFinder`, `Bit: 24`, `Key` (`subghz_block_generic_serialize`);
//! loading requires `Bit: 24` (`subghz_block_generic_deserialize_check_count_bit`).

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

pub(super) const NAME: &str = "KeyFinder";
const TE_SHORT: u32 = 400;
const TE_LONG: u32 = 1200;
const TE_DELTA: u32 = 150;
const MIN_COUNT_BIT: usize = 24;
/// `subghz_protocol_encoder_keyfinder_alloc`: repeat = 5.
const TX_REPEAT: usize = 5;
/// Flipper-ARF lib's custom buttons (`subghz_protocol_keyfinder_get_btn_code`): UP Red, DOWN Purple,
/// LEFT Green, RIGHT Blue.
pub(super) const DPAD: [u8; 4] = [0xE, 0xB, 0xD, 0xC];

/// The colours of the C's key samples (Flipper-ARF lib's button comments).
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    match code {
        0xE => Some("Red"),
        0xB => Some("Purple"),
        0xD => Some("Green"),
        0xC => Some("Blue"),
        _ => None,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    SaveDuration,
    CheckDuration,
    Finish,
}

/// KeyFinder decoder (`SubGhzProtocolDecoderKeyFinder`).
pub struct KeyFinderDecoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
    /// The C's `end_count`: end pulses seen after the 24th bit (survives `reset()`, as in the C).
    end_count: u8,
}

/// A decode (or a loaded key) of `data`, with the C's serial / ID split.
fn decoded(data: u64) -> DecodedSignal {
    DecodedSignal {
        serial: Some((data >> 4) as u32),
        button: Some((data & 0xF) as u8),
        counter: None,
        crc_valid: false,
        data,
        data_count_bit: MIN_COUNT_BIT,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

/// One `get_upload` copy.
fn build_frame(data: u64) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(MIN_COUNT_BIT * 2 + 8);
    for i in (0..MIN_COUNT_BIT).rev() {
        if (data >> i) & 1 == 1 {
            push(&mut out, true, TE_SHORT);
            push(&mut out, false, TE_LONG);
        } else {
            push(&mut out, true, TE_LONG);
            push(&mut out, false, TE_SHORT);
        }
    }
    for _ in 0..3 {
        push(&mut out, true, TE_SHORT);
        push(&mut out, false, TE_SHORT);
    }
    push(&mut out, true, TE_SHORT);
    push(&mut out, false, TE_SHORT * 10);
    out
}

impl KeyFinderDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
            end_count: 0,
        }
    }

    /// Frame cleared, back to Reset (the C's in-feed clear; `reset()` keeps all but the step).
    fn clear(&mut self) {
        self.decode_data = 0;
        self.decode_count_bit = 0;
        self.end_count = 0;
        self.step = Step::Reset;
    }
}

impl Default for KeyFinderDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for KeyFinderDecoder {
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

    /// `subghz_protocol_decoder_keyfinder_reset` (doctor: `common_reset`): the parser step only.
    fn reset(&mut self) {
        self.step = Step::Reset;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let short = |d: u32| duration_diff!(d, TE_SHORT) < TE_DELTA;
        let long = |d: u32| duration_diff!(d, TE_LONG) < TE_DELTA;
        match self.step {
            Step::Reset => {
                if !level && duration_diff!(duration, TE_SHORT * 10) < TE_DELTA * 5 {
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.step = Step::SaveDuration;
                }
            }
            Step::SaveDuration => {
                if usize::from(self.decode_count_bit) == MIN_COUNT_BIT {
                    if level && short(duration) {
                        self.end_count = self.end_count.wrapping_add(1);
                        if self.end_count == 4 {
                            self.step = Step::Finish;
                            self.end_count = 0;
                        }
                    } else if !level && short(duration) {
                        // the LOW between end pulses
                    } else {
                        self.clear();
                    }
                } else if level {
                    self.te_last = duration;
                    self.step = Step::CheckDuration;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::CheckDuration => {
                if !level && short(self.te_last) && long(duration) {
                    add_bit(&mut self.decode_data, &mut self.decode_count_bit, true);
                    self.step = Step::SaveDuration;
                } else if !level && long(self.te_last) && short(duration) {
                    add_bit(&mut self.decode_data, &mut self.decode_count_bit, false);
                    self.step = Step::SaveDuration;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::Finish => {
                // The pulse after the fourth end pulse reports the frame; the C then clears it
                // (on top of the callback's receiver reset).
                let out = (usize::from(self.decode_count_bit) == MIN_COUNT_BIT).then(|| decoded(self.decode_data));
                self.clear();
                return out;
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
        // Flipper-ARF lib: data = serial << 4 | btn (serial = data >> 4), the captured ID unless a
        // button is chosen.
        let btn = if button <= 0xF { button } else { (decoded.data & 0xF) as u8 };
        let data = (decoded.data & !0xF) | u64::from(btn);
        Some(with_repeats(&build_frame(data), TX_REPEAT))
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

    /// The C's Red sample: serial C396F, ID E.
    const RED: u64 = 0xC396FE;

    fn on_air(data: u64, button: u8) -> Vec<LevelDuration> {
        let mut wire = vec![LevelDuration::new(false, 20_000)];
        for p in KeyFinderDecoder::new().encode(&decoded(data), button).unwrap() {
            push(&mut wire, p.level, p.duration_us);
        }
        wire
    }

    /// 5 copies: the 1st has no gap before it, the 2nd's report consumes the gap the 3rd needs, the
    /// 4th decodes, the 5th's gap was consumed (as the C's encoder round trip: 2 decodes).
    #[test]
    fn encode_decode_roundtrip() {
        for data in [RED, 0x000001, 0xFFFFFF, 0x5A5A5A] {
            let got = feed_all(&mut KeyFinderDecoder::new(), &on_air(data, 0xFF));
            assert_eq!(got.len(), 2, "{data:06X}");
            for d in &got {
                assert_eq!((d.data, d.data_count_bit), (data, 24));
                assert_eq!((d.serial, d.button), (Some((data >> 4) as u32), Some((data & 0xF) as u8)));
                assert!(d.encoder_capable && !d.crc_valid && d.counter.is_none());
            }
        }
    }

    #[test]
    fn upload_layout() {
        let up = KeyFinderDecoder::new().encode(&decoded(0x800001), 0x1).unwrap();
        let p = |i: usize| (up[i].level, up[i].duration_us);
        assert_eq!(p(0), (true, 400)); // bit 23 = 1
        assert_eq!(p(1), (false, 1200));
        assert_eq!(p(2), (true, 1200)); // bit 22 = 0
        assert_eq!(p(3), (false, 400));
        assert_eq!(p(47), (false, 1200)); // bit 0 = 1
        assert_eq!((p(48), p(53), p(54), p(55)), ((true, 400), (false, 400), (true, 400), (false, 4000)));
        // 56 entries a copy (the C's 60-entry buffer), 5 copies.
        assert_eq!(up.len(), 56 * 5);
    }

    /// Flipper-ARF lib's custom buttons: the low nibble is replaced, the serial kept.
    #[test]
    fn buttons_switch_the_id() {
        for &b in &DPAD {
            let got = feed_all(&mut KeyFinderDecoder::new(), &on_air(RED, b));
            assert_eq!(got.len(), 2);
            assert!(got.iter().all(|d| d.serial == Some(0xC396F) && d.button == Some(b)), "{b:X}");
        }
        // The captured ID (any value above 0xF keeps it).
        let pulses = |button: u8| -> Vec<(bool, u32)> {
            let up = KeyFinderDecoder::new().encode(&decoded(RED), button).unwrap();
            up.iter().map(|p| (p.level, p.duration_us)).collect()
        };
        assert_eq!(pulses(0xE), pulses(0x10));
        // A switch sends exactly the upload of the key with that ID (Flipper-ARF lib's
        // serial << 4 | btn), which the garage app's encoder sends for such a key.
        let mut rng = Rng(0x00C0_10E5);
        for _ in 0..20 {
            let data = rng.below(0x100_0000);
            for b in 0..=0xFu8 {
                let switched = KeyFinderDecoder::new().encode(&decoded(data), b).unwrap();
                let keyed = KeyFinderDecoder::new().encode(&decoded(data & !0xF | u64::from(b)), 0xFF).unwrap();
                let flat = |up: &[LevelDuration]| up.iter().map(|p| (p.level, p.duration_us)).collect::<Vec<_>>();
                assert_eq!(flat(&switched), flat(&keyed), "{data:06X} button {b:X}");
            }
        }
        assert_eq!(native_buttons(NAME, 0xE), Some("Red"));
        assert_eq!(native_buttons(NAME, 0xC), Some("Blue"));
        assert_eq!(native_buttons(NAME, 0x5), None);
    }

    #[test]
    fn end_pulses_and_lengths() {
        let one = |data: u64| {
            let mut w = vec![LevelDuration::new(false, 4000)];
            for p in build_frame(data) {
                push(&mut w, p.level, p.duration_us);
            }
            push(&mut w, true, 400);
            w
        };
        assert_eq!(feed_all(&mut KeyFinderDecoder::new(), &one(RED)).len(), 1);
        // Three end pulses only: the gap clears the frame.
        let mut three = one(RED);
        three.drain(49..51);
        assert!(feed_all(&mut KeyFinderDecoder::new(), &three).is_empty());
        // 23 bits: the first end pulse is read as a bit HIGH, its LOW breaks it.
        let mut short = one(RED);
        short.drain(1..3);
        assert!(feed_all(&mut KeyFinderDecoder::new(), &short).is_empty());
        // Gap window 3251..=4749 (strict, as the C).
        for (g, ok) in [(3250u32, false), (3251, true), (4749, true), (4750, false)] {
            let mut w = one(RED);
            w[0].duration_us = g;
            assert_eq!(feed_all(&mut KeyFinderDecoder::new(), &w).len(), usize::from(ok), "gap {g}");
        }
        // Symbol windows 251..=549 / 1051..=1349.
        for (i, d, ok) in [(1usize, 250u32, false), (1, 251, true), (1, 549, true), (1, 550, false),
                           (2, 1050, false), (2, 1051, true), (2, 1349, true), (2, 1350, false)] {
            let mut w = one(RED);
            w[i].duration_us = d;
            assert_eq!(feed_all(&mut KeyFinderDecoder::new(), &w).len(), usize::from(ok), "pulse {i} = {d}");
        }
    }

    /// The end-pulse counter survives `reset()`, as the C's `end_count`.
    #[test]
    fn end_count_survives_reset() {
        let mut dec = KeyFinderDecoder::new();
        let mut w = vec![LevelDuration::new(false, 4000)];
        for p in build_frame(RED) {
            push(&mut w, p.level, p.duration_us);
        }
        // 24 bits and two end pulses, then a reset (another decoder's report).
        for p in &w[..1 + 48 + 3] {
            assert!(dec.feed(p.level, p.duration_us).is_none());
        }
        assert_eq!(dec.end_count, 2);
        dec.reset();
        // The next frame needs only two end pulses: Finish on the second, report on the LOW after it.
        let got: Vec<DecodedSignal> = w[..1 + 48 + 4].iter().filter_map(|p| dec.feed(p.level, p.duration_us)).collect();
        assert_eq!(got.len(), 1);
    }

    #[test]
    fn key_file_round_trip() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: KeyFinder\nBit: 24\n\
                    Key: 00 00 00 00 00 C3 96 FE\n";
        let dec = KeyFinderDecoder::new();
        let d = dec.from_flipper_key(&FlipperKey::parse(text).unwrap()).unwrap();
        assert_eq!((d.data, d.serial, d.button), (RED, Some(0xC396F), Some(0xE)));
        let written = super::super::flipper_key::testing::assert_round_trip(&dec, &d);
        assert!(written.contains("Protocol: KeyFinder\nBit: 24\nKey: 00 00 00 00 00 C3 96 FE\n"), "{written}");
        assert!(dec.from_flipper_key(&FlipperKey::parse(&text.replace("Bit: 24", "Bit: 11")).unwrap()).is_none());
    }

    #[test]
    fn jittered_frames_decode() {
        let mut rng = Rng(0x00CE_F1D0);
        for _ in 0..50 {
            let data = rng.below(0x100_0000);
            let wire = jitter(&on_air(data, 0xFF), &mut rng, 100);
            let got = feed_all(&mut KeyFinderDecoder::new(), &wire);
            assert_eq!(got.len(), 2, "{data:06X}");
            assert!(got.iter().all(|d| d.data == data));
        }
    }

    /// Through the whole registry: KeyFinder's frames and nothing else.
    #[test]
    fn registry_reads_keyfinder_only() {
        let mut rng = Rng(0x0433_0001);
        for _ in 0..20 {
            let data = rng.below(0x100_0000);
            let mut reg = ProtocolRegistry::new();
            let got: Vec<String> = reg
                .process_signal_stream_rf(&on_air(data, 0xFF), 433_920_000, Some(crate::capture::RfModulation::AM))
                .into_iter()
                .map(|(n, d, _)| format!("{n} {:06X}", d.data))
                .collect();
            assert_eq!(got, vec![format!("{NAME} {data:06X}"); 2]);
        }
    }
}
