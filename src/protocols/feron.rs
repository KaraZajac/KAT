//! Feron: 32-bit fixed code of Feron LED-light remotes (a 16-bit serial and a 16-bit command),
//! decoder + encoder.
//!
//! Port of doctor-firmware `lib/subghz/protocols/feron.c` (@xMasterX, 2025.04; the stock Flipper
//! protocol, registered by doctor, Momentum, RogueMaster and FoxFW; Flipper-ARF ships its `lib` and
//! garage-app copies unregistered). Every copy decodes and encodes identically; they differ in style
//! (doctor / RogueMaster / FoxFW use `common.c`, FoxFW loads the encoder from an SD plugin) and in
//! the display text (doctor / Momentum / RogueMaster: `Serial: 0x…` and `Command: 0x…`; ARF:
//! `SN:0x…`). Unit-test captures: `feron_raw.sub`, `feron.sub`.
//!
//! **Air format** (AM, 433.92 MHz): 32 bits MSB first, each a HIGH then a LOW: 0 = HIGH 350 + LOW
//! 750, 1 = HIGH 750 + LOW 350. The last bit's LOW is replaced by an end marker LOW 500 + HIGH 500,
//! then a 4500 µs (6·te_long) gap.
//!
//! **Decoder** (`subghz_protocol_decoder_feron_feed`): Reset waits for a LOW with |d − 4500| < 600;
//! then HIGH (saved) + LOW pairs: HIGH 350 ± 150 + LOW 750 ± 150 = 0, HIGH 750 ± 150 + LOW 350 ±
//! 150 = 1; otherwise a LOW |d − 500| < 150 is the end marker: the saved HIGH adds its bit (≈ 350 → 0,
//! ≈ 750 → 1), and exactly 32 bits are reported. Anything else resets. After the marker the
//! decoder is back in Reset, and the marker's HIGH and the gap that follows open the next frame,
//! so every repeat decodes (each copy after the first: the first has no gap before it). No check.
//!
//! **Fields** (`subghz_protocol_feron_check_remote_controller`): serial = data >> 16; the C shows
//! the low 16 bits as the command (a command byte and its complement in the C's samples) but stores
//! no button: KAT reports serial only. `data` = the 32 bits.
//!
//! **Encoder** (`subghz_protocol_encoder_feron_get_upload`, `encoder.repeat` = 3): the 32 bit pairs,
//! the last one's LOW replaced by LOW 500, HIGH 500, LOW 4500. `button` is ignored (the C has no
//! button override).
//!
//! **Key file**: `Protocol: Feron`, `Bit: 32` (required), `Key` (the generic serialize).

use super::f_common::{add_bit, push, with_repeats};
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

/// Metadata (see `info.rs`): `SubGhzProtocolFlag_433 | AM` (+ Sensors), `SubGhzProtocolTypeStatic`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Feron",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

pub(super) const NAME: &str = "Feron";
const TE_SHORT: u32 = 350;
const TE_LONG: u32 = 750;
const TE_DELTA: u32 = 150;
const MIN_COUNT_BIT: usize = 32;
/// The end marker: LOW then HIGH of te_short + 150.
const MARK: u32 = TE_SHORT + 150;
/// The gap after a frame: 6·te_long.
const GAP: u32 = TE_LONG * 6;
/// `subghz_protocol_encoder_common_alloc(…, 3, 256)`.
const TX_REPEAT: usize = 3;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    SaveDuration,
    CheckDuration,
}

/// Feron decoder (`SubGhzProtocolDecoderFeron`).
pub struct FeronDecoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
}

/// A decode (or a loaded key) of `data` (32 bits).
fn decoded(data: u64) -> DecodedSignal {
    DecodedSignal {
        serial: Some((data >> 16) as u32),
        button: None,
        counter: None,
        crc_valid: false,
        data,
        data_count_bit: MIN_COUNT_BIT,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

/// One `get_upload` frame (the generic deserialize leaves `data_count_bit` = 32).
fn build_frame(data: u64) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(MIN_COUNT_BIT * 2 + 2);
    for i in (1..=MIN_COUNT_BIT).rev() {
        let one = (data >> (i - 1)) & 1 == 1;
        push(&mut out, true, if one { TE_LONG } else { TE_SHORT });
        if i == 1 {
            push(&mut out, false, MARK);
            push(&mut out, true, MARK);
            push(&mut out, false, GAP);
        } else {
            push(&mut out, false, if one { TE_SHORT } else { TE_LONG });
        }
    }
    out
}

impl FeronDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
        }
    }
}

impl Default for FeronDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for FeronDecoder {
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
        match self.step {
            Step::Reset => {
                if !level && duration_diff!(duration, GAP) < TE_DELTA * 4 {
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
                let high_short = duration_diff!(self.te_last, TE_SHORT) < TE_DELTA;
                let high_long = duration_diff!(self.te_last, TE_LONG) < TE_DELTA;
                if high_short && duration_diff!(duration, TE_LONG) < TE_DELTA {
                    add_bit(&mut self.decode_data, &mut self.decode_count_bit, false);
                    self.step = Step::SaveDuration;
                } else if high_long && duration_diff!(duration, TE_SHORT) < TE_DELTA {
                    add_bit(&mut self.decode_data, &mut self.decode_count_bit, true);
                    self.step = Step::SaveDuration;
                } else if duration_diff!(duration, MARK) < TE_DELTA {
                    // End of the key: LOW 500 (here), then HIGH 500.
                    if high_short {
                        add_bit(&mut self.decode_data, &mut self.decode_count_bit, false);
                    }
                    if high_long {
                        add_bit(&mut self.decode_data, &mut self.decode_count_bit, true);
                    }
                    let out = (usize::from(self.decode_count_bit) == MIN_COUNT_BIT).then(|| decoded(self.decode_data));
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.step = Step::Reset;
                    return out;
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
        (decoded.data_count_bit == MIN_COUNT_BIT).then(|| with_repeats(&build_frame(decoded.data), TX_REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        (key.bits == MIN_COUNT_BIT).then(|| decoded(key.key))
    }

    fn to_flipper_key(&self, d: &DecodedSignal) -> Option<FlipperKey> {
        (d.data_count_bit == MIN_COUNT_BIT).then(|| FlipperKey::for_protocol(NAME, MIN_COUNT_BIT, d.data))
    }
}

#[cfg(test)]
pub(super) mod testing {
    //! Frames for other modules' tests and the parity tools.
    use super::*;

    /// A decode of the 32-bit `data`, as a key file gives it.
    pub fn frame(data: u64) -> DecodedSignal {
        decoded(data & 0xFFFF_FFFF)
    }
}

#[cfg(test)]
mod tests {
    use super::super::f_common::test_support::*;
    use super::*;

    const UNIT_TEST_KEY: &str = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
        Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Feron\nBit: 32\nKey: 00 00 00 00 63 38 84 7B\n";

    fn on_air(data: u64) -> Vec<LevelDuration> {
        let mut wire = vec![LevelDuration::new(false, GAP)];
        for p in FeronDecoder::new().encode(&testing::frame(data), 0).unwrap() {
            push(&mut wire, p.level, p.duration_us);
        }
        wire
    }

    #[test]
    fn encode_decode_roundtrip() {
        // The C's samples: ON, OFF, brightness up, R.
        for data in [0x6338_857Au64, 0x6338_847B, 0x6338_8679, 0x6338_708F, 0, 0xFFFF_FFFF] {
            let got = feed_all(&mut FeronDecoder::new(), &on_air(data));
            // A gap before the first copy: every copy decodes.
            assert_eq!(got.len(), 3, "{data:08X}");
            for d in &got {
                assert_eq!((d.data, d.data_count_bit), (data, 32));
                assert_eq!(d.serial, Some((data >> 16) as u32));
                assert!(d.encoder_capable && d.button.is_none() && !d.crc_valid);
            }
        }
    }

    #[test]
    fn upload_layout() {
        let up = FeronDecoder::new().encode(&testing::frame(0x8000_0001), 0).unwrap();
        let p = |i: usize| (up[i].level, up[i].duration_us);
        assert_eq!(p(0), (true, 750)); // bit 31 = 1
        assert_eq!(p(1), (false, 350));
        assert_eq!(p(2), (true, 350)); // bit 30 = 0
        assert_eq!(p(3), (false, 750));
        // bit 0 = 1: HIGH 750, then LOW 500, HIGH 500, LOW 4500.
        assert_eq!((p(62), p(63), p(64), p(65)), ((true, 750), (false, 500), (true, 500), (false, 4500)));
        // 66 entries a copy (the next copy starts HIGH), 3 copies.
        assert_eq!(up.len(), 66 * 3);
    }

    #[test]
    fn rejects_wrong_length_and_timing() {
        let wire = on_air(0x6338_847B);
        assert_eq!(feed_all(&mut FeronDecoder::new(), &wire).len(), 3);
        // A missing bit pair in the first copy: 31 bits, dropped.
        let mut short = wire.clone();
        short.drain(1..3);
        assert_eq!(feed_all(&mut FeronDecoder::new(), &short).len(), 2);
        // Gap window 3901..=5099 µs.
        for (g, ok) in [(3900u32, false), (3901, true), (5099, true), (5100, false)] {
            let mut w = wire.clone();
            w[0].duration_us = g;
            assert_eq!(feed_all(&mut FeronDecoder::new(), &w).len(), if ok { 3 } else { 2 }, "gap {g}");
        }
        // The end marker's LOW (351..=649 µs) is tried after the two bit cells: after a last HIGH of
        // 750 a LOW under 500 is a 1 cell (the frame then misses its end, and the gap after the
        // marker's HIGH goes to the cell check: the next copy loses its sync too), after a HIGH of 350
        // a LOW over 600 is a 0 cell. A LOW in neither window resets, and the gap still opens the
        // next copy.
        for (data, m, decodes) in [
            (0x6338_847Bu64, 500u32, 3usize),
            (0x6338_847B, 649, 3),
            (0x6338_847B, 650, 2),
            (0x6338_847B, 499, 1),
            (0x6338_847A, 351, 3),
            (0x6338_847A, 600, 3),
            (0x6338_847A, 350, 2),
            (0x6338_847A, 601, 1),
        ] {
            let mut w = on_air(data);
            w[64].duration_us = m;
            assert_eq!(feed_all(&mut FeronDecoder::new(), &w).len(), decodes, "{data:08X}, marker {m}");
        }
    }

    #[test]
    fn key_file_roundtrip() {
        let dec = FeronDecoder::new();
        let d = dec.from_flipper_key(&FlipperKey::parse(UNIT_TEST_KEY).unwrap()).unwrap();
        assert_eq!((d.data, d.data_count_bit, d.serial), (0x6338_847B, 32, Some(0x6338)));
        let text = super::super::flipper_key::testing::assert_round_trip(&dec, &d);
        assert!(text.contains("Protocol: Feron\nBit: 32\nKey: 00 00 00 00 63 38 84 7B\n"), "{text}");
        assert!(dec.from_flipper_key(&FlipperKey::parse(&UNIT_TEST_KEY.replace("Bit: 32", "Bit: 24")).unwrap()).is_none());
    }

    /// Flipper's `subghz_decoder_feron_test` capture through the whole registry: 102 Feron frames (as
    /// doctor / ARF / Momentum / RogueMaster / FoxFW), all the unit-test key, nothing else.
    #[test]
    fn unit_test_capture_decodes() {
        let Some(path) = reference_file(ARF_UNIT_TESTS, "feron_raw.sub") else {
            return;
        };
        let got = registry_decodes(&path);
        let ours: Vec<&DecodedSignal> = got.iter().filter(|(n, _)| n == NAME).map(|(_, d)| d).collect();
        assert_eq!(ours.len(), 102);
        assert!(ours.iter().all(|d| d.data == 0x6338_847B && d.serial == Some(0x6338)));
        assert_eq!(got.len(), ours.len(), "another protocol took a frame");
    }

    /// `subghz_encoder_feron_test`: the unit-test key encodes and decodes back through the registry
    /// (copies 2 and 3, as the C's round trip).
    #[test]
    fn unit_test_key_encodes_back() {
        let reg = super::super::ProtocolRegistry::new();
        let (name, d) = reg.decode_key(&FlipperKey::parse(UNIT_TEST_KEY).unwrap()).unwrap();
        assert_eq!(name, NAME);
        let tx = reg.get(&name).unwrap().encode(&d, 0).unwrap();
        let mut reg = super::super::ProtocolRegistry::new();
        let back = reg.process_signal_stream(&tx, 433_920_000);
        assert_eq!(back.len(), 2, "{back:?}");
        assert!(back.iter().all(|(n, x, _)| n == NAME && x.data == 0x6338_847B), "{back:?}");
    }

    /// Jitter ±100 µs. The last bit is 0 here: after a last HIGH of 750 the C reads a marker LOW
    /// under 500 as a 1 cell (see `rejects_wrong_length_and_timing`).
    #[test]
    fn jittered_frames_decode() {
        let mut rng = Rng(0xFE40_0001);
        for _ in 0..50 {
            let data = rng.below(1 << 32) & !1;
            let wire = jitter(&on_air(data), &mut rng, 100);
            let got = feed_all(&mut FeronDecoder::new(), &wire);
            assert!(!got.is_empty(), "{data:08X}");
            assert!(got.iter().all(|d| d.data == data));
        }
    }
}
