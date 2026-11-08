//! Doorbell32: 32-bit fixed code of wireless doorbells (Princeton-style 1:3 PWM with a measured
//! Te), decoder + encoder.
//!
//! Port of Flipper-ARF's garage app `applications/system/subghz_garage/protocols/doorbell32.c`
//! (registered in RX group g13, TX plugin `tx_doorbell32`). doctor-firmware, RogueMaster and FoxFW
//! register the same decoder and encoder: feed, upload, fields and the `TE` line are identical, they
//! differ in style only (doctor and RogueMaster flag it a sensor and write `TE` through
//! `common_serialize_te`; FoxFW loads its encoder and its text from SD plugins with the same code).
//!
//! **Air format** (AM, 433.92 MHz): Te nominally 515 µs (te_long 1545 µs), measured per frame and
//! different between units. 32 bits MSB first, each a HIGH then a LOW: 0 = HIGH Te + LOW 3·Te,
//! 1 = HIGH 3·Te + LOW Te; then a closing HIGH Te and the guard, a LOW of 14·Te (doctor's comment:
//! Princeton's grammar with 32 bits and a 14-Te guard instead of 24 bits and 31 Te, so Princeton
//! syncs on the guard and drops the frame on its bit count).
//!
//! **Decoder** (`subghz_protocol_decoder_doorbell32_feed`): Reset waits for a LOW with |d − 7210| <
//! 1400 (14·te_short ± 14·te_delta/2); then HIGH/LOW pairs: HIGH |d − 515| < 200 + LOW |d − 1545|
//! < 400 → 0, HIGH |d − 1545| < 400 + LOW |d − 515| < 200 → 1, anything else back to Reset. Every
//! HIGH and every bit LOW is summed into `te`. A LOW ≥ 3090 µs (2·te_long) ends the frame: with
//! exactly 32 bits equal to the previous frame (`last_data`, 32 bits, non-zero) it is reported with
//! Te = te / 129 (32 bits of 4 Te plus the closing HIGH). The frame then becomes `last_data` and
//! the decoder goes on with the next frame (SaveDuration), overriding the callback's receiver reset;
//! KAT keeps that state through the registry's reset that follows its own report (`just_emitted`).
//! `reset()` clears the parser step only, so `last_data` also survives other protocols' reports.
//! A held button reports every frame from the third on: the first has no guard before it, the
//! second only fills `last_data` (8 of the encoder's 10 copies, as in the C).
//!
//! **Fields**: none in the C (it shows the key, the measured Te and the guard "GT:Te*14"); `extra` =
//! the Te (the `TE` line). No checksum: the repeat is the only check.
//!
//! **Encoder** (`subghz_protocol_encoder_doorbell32_get_upload`, `encoder.repeat` = 10): the 32
//! bits at Te, the closing HIGH Te and the 14·Te guard, 10 copies; Te = the key's `TE` line, 515 µs
//! when it has none (or it does not parse). The C sends a `TE: 0` key as zero-length pulses; KAT
//! returns no upload for it. `button` is ignored (no button field).
//!
//! **Key file**: `Protocol: Doorbell32`, `Bit: 32`, `Key`, `TE` (the decoder's serialize writes the
//! measured Te); loading requires `Bit: 32` (the decoder's deserialize ignores `TE`, the encoder
//! reads it; KAT keeps it when present and writes it back).

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
    make: "Doorbell",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

pub(super) const NAME: &str = "Doorbell32";
const TE_SHORT: u32 = 515;
const TE_LONG: u32 = 1545;
const TE_DELTA: u32 = 200;
const MIN_COUNT_BIT: usize = 32;
/// `DOORBELL32_GUARD_TIME`: the guard LOW in Te.
const GUARD_TIME: u32 = 14;
/// `subghz_protocol_encoder_doorbell32_alloc`: repeat = 10.
const TX_REPEAT: usize = 10;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    SaveDuration,
    CheckDuration,
}

/// Doorbell32 decoder (`SubGhzProtocolDecoderDoorbell32`).
pub struct Doorbell32Decoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
    /// The C's `te`: the sum of the frame's pulses, then the mean.
    te: u32,
    /// The C's `uint32_t last_data`: the previous frame (survives `reset()`).
    last_data: u32,
    /// The last `feed` reported a frame: keep the post-callback state through the registry's reset.
    just_emitted: bool,
}

/// A decode (or a loaded key) of `data` at Te `te` (None: a key file without `TE`).
fn decoded(data: u64, te: Option<u32>) -> DecodedSignal {
    DecodedSignal {
        serial: None,
        button: None,
        counter: None,
        crc_valid: false,
        data,
        data_count_bit: MIN_COUNT_BIT,
        encoder_capable: true,
        extra: te.map(u64::from),
        protocol_display_name: None,
    }
}

/// One `get_upload` copy at Te `te`.
fn build_frame(data: u64, te: u32) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(MIN_COUNT_BIT * 2 + 2);
    // uint32_t arithmetic, as the C (a key file's TE is not range-checked)
    let (te3, guard) = (te.wrapping_mul(3), te.wrapping_mul(GUARD_TIME));
    for i in (0..MIN_COUNT_BIT).rev() {
        if (data >> i) & 1 == 1 {
            push(&mut out, true, te3);
            push(&mut out, false, te);
        } else {
            push(&mut out, true, te);
            push(&mut out, false, te3);
        }
    }
    push(&mut out, true, te);
    push(&mut out, false, guard);
    out
}

impl Doorbell32Decoder {
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
}

impl Default for Doorbell32Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for Doorbell32Decoder {
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

    /// `subghz_protocol_decoder_doorbell32_reset` (doctor: `common_reset`): the parser step only
    /// (skipped right after this decoder's own report: the C code after its callback overrides it).
    fn reset(&mut self) {
        if std::mem::take(&mut self.just_emitted) {
            return;
        }
        self.step = Step::Reset;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        self.just_emitted = false;
        match self.step {
            Step::Reset => {
                if !level && duration_diff!(duration, TE_SHORT * GUARD_TIME) < TE_DELTA * GUARD_TIME / 2 {
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
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::CheckDuration => {
                if level {
                    self.step = Step::Reset;
                    return None;
                }
                if duration >= TE_LONG * 2 {
                    // The guard ends the frame: a repeat of the previous one is reported.
                    let repeat = usize::from(self.decode_count_bit) == MIN_COUNT_BIT
                        && u64::from(self.last_data) == self.decode_data
                        && self.last_data != 0;
                    let count = u32::from(self.decode_count_bit);
                    let out = repeat.then(|| decoded(self.decode_data, Some(self.te / (count * 4 + 1))));
                    // After the callback (whose receiver reset this overrides).
                    self.last_data = self.decode_data as u32;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.te = 0;
                    self.step = Step::SaveDuration;
                    self.just_emitted = out.is_some();
                    return out;
                }
                self.te = self.te.wrapping_add(duration);
                if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA && duration_diff!(duration, TE_LONG) < TE_DELTA * 2 {
                    add_bit(&mut self.decode_data, &mut self.decode_count_bit, false);
                    self.step = Step::SaveDuration;
                } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA * 2 && duration_diff!(duration, TE_SHORT) < TE_DELTA {
                    add_bit(&mut self.decode_data, &mut self.decode_count_bit, true);
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
        let te = decoded.extra.map_or(TE_SHORT, |t| t as u32);
        (te > 0).then(|| with_repeats(&build_frame(decoded.data, te), TX_REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        (key.bits == MIN_COUNT_BIT).then(|| decoded(key.key, key.u32_field("TE")))
    }

    fn to_flipper_key(&self, d: &DecodedSignal) -> Option<FlipperKey> {
        if d.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        let mut key = FlipperKey::for_protocol(NAME, MIN_COUNT_BIT, d.data);
        if let Some(te) = d.extra {
            key.push_u32("TE", te as u32);
        }
        Some(key)
    }
}

#[cfg(test)]
mod tests {
    use super::super::f_common::test_support::*;
    use super::*;
    use crate::protocols::ProtocolRegistry;

    fn on_air(data: u64, te: Option<u32>) -> Vec<LevelDuration> {
        let mut wire = vec![LevelDuration::new(false, 20_000)];
        for p in Doorbell32Decoder::new().encode(&decoded(data, te), 0).unwrap() {
            push(&mut wire, p.level, p.duration_us);
        }
        wire
    }

    /// 10 copies: the 1st has no guard before it, the 2nd is stored, the 3rd..10th repeat it.
    #[test]
    fn encode_decode_roundtrip() {
        // The guard window reads Te 416..=614 µs (14·Te within 7210 ± 1400, exclusive).
        for (data, te) in [(0x1234_5678u64, Some(480)), (0x0000_0001, None), (0xFFFF_FFFF, Some(420)), (0x8000_0000, Some(610))] {
            let got = feed_all(&mut Doorbell32Decoder::new(), &on_air(data, te));
            assert_eq!(got.len(), 8, "{data:08X}");
            for d in &got {
                assert_eq!((d.data, d.data_count_bit), (data, 32));
                // The mean of exact pulses is the Te sent.
                assert_eq!(d.extra, Some(u64::from(te.unwrap_or(515))));
                assert!(d.encoder_capable && !d.crc_valid && d.serial.is_none() && d.button.is_none());
            }
            // Decoder alone, reset after each report (the C oracle's `--protocol`): the same 8.
            assert_eq!(feed_resetting(&mut Doorbell32Decoder::new(), &on_air(data, te)).len(), 8);
        }
    }

    #[test]
    fn upload_layout() {
        let up = Doorbell32Decoder::new().encode(&decoded(0x8000_0001, Some(500)), 0).unwrap();
        let p = |i: usize| (up[i].level, up[i].duration_us);
        assert_eq!((p(0), p(1)), ((true, 1500), (false, 500))); // bit 31 = 1
        assert_eq!((p(2), p(3)), ((true, 500), (false, 1500))); // bit 30 = 0
        assert_eq!((p(62), p(63)), ((true, 1500), (false, 500))); // bit 0 = 1
        assert_eq!((p(64), p(65)), ((true, 500), (false, 7000)));
        // 66 entries a copy (the C's 68-entry buffer), 10 copies.
        assert_eq!(up.len(), 66 * 10);
        // No TE line: 515 µs; TE 0: nothing to send.
        let up = Doorbell32Decoder::new().encode(&decoded(1, None), 0).unwrap();
        assert_eq!(up[65].duration_us, 515 * 14);
        assert!(Doorbell32Decoder::new().encode(&decoded(1, Some(0)), 0).is_none());
        assert!(Doorbell32Decoder::new().encode(&DecodedSignal::new(1, 24), 0).is_none());
    }

    #[test]
    fn needs_a_repeat_of_32_bits() {
        let frame = |data: u64| {
            let mut w = vec![LevelDuration::new(false, 7210)];
            for p in build_frame(data, 515) {
                push(&mut w, p.level, p.duration_us);
            }
            w
        };
        // One frame after a guard: stored, not reported; a second, equal one: reported.
        let one = frame(0xCAFE_F00D);
        assert!(feed_all(&mut Doorbell32Decoder::new(), &one).is_empty());
        let mut two = one.clone();
        two.extend(build_frame(0xCAFE_F00D, 515));
        assert_eq!(feed_all(&mut Doorbell32Decoder::new(), &two).len(), 1);
        // Two different codes: nothing; a zero key never reports.
        let mut diff = one.clone();
        diff.extend(build_frame(0xCAFE_F00C, 515));
        assert!(feed_all(&mut Doorbell32Decoder::new(), &diff).is_empty());
        let mut zero = frame(0);
        zero.extend(build_frame(0, 515));
        assert!(feed_all(&mut Doorbell32Decoder::new(), &zero).is_empty());
        // 31 bits twice: nothing.
        let mut short = frame(0xCAFE_F00D);
        short.drain(1..3);
        let mut short2 = short.clone();
        short2.extend(short[1..].iter().copied());
        assert!(feed_all(&mut Doorbell32Decoder::new(), &short2).is_empty());
        // Guard window 5811..=8609 (strict, as the C), end threshold 3090.
        for (g, ok) in [(5810u32, false), (5811, true), (8609, true), (8610, false)] {
            let mut w = two.clone();
            w[0].duration_us = g;
            assert_eq!(feed_all(&mut Doorbell32Decoder::new(), &w).len(), usize::from(ok), "guard {g}");
        }
        for (g, ok) in [(3089u32, false), (3090, true)] {
            let mut w = two.clone();
            w.last_mut().unwrap().duration_us = g;
            w[66].duration_us = g;
            assert_eq!(feed_all(&mut Doorbell32Decoder::new(), &w).len(), usize::from(ok), "end {g}");
        }
    }

    /// `last_data` survives another protocol's report (the registry's reset), and the decoder's own
    /// report keeps its post-callback step (SaveDuration).
    #[test]
    fn repeat_memory_survives_reset() {
        let mut dec = Doorbell32Decoder::new();
        let mut w = vec![LevelDuration::new(false, 7210)];
        w.extend(build_frame(0x0BAD_CAFE, 515));
        for p in &w {
            assert!(dec.feed(p.level, p.duration_us).is_none());
        }
        assert_eq!(dec.last_data, 0x0BAD_CAFE);
        dec.reset(); // another decoder reported
        let mut w2 = vec![LevelDuration::new(false, 7210)];
        w2.extend(build_frame(0x0BAD_CAFE, 515));
        let got: Vec<DecodedSignal> = w2.iter().filter_map(|p| dec.feed(p.level, p.duration_us)).collect();
        assert_eq!(got.len(), 1);
        // Its own report: the registry's reset is skipped, the next copy follows the guard.
        dec.reset();
        let got: Vec<DecodedSignal> = build_frame(0x0BAD_CAFE, 515)
            .iter()
            .filter_map(|p| dec.feed(p.level, p.duration_us))
            .collect();
        assert_eq!(got.len(), 1);
    }

    #[test]
    fn key_file_round_trip() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Doorbell32\nBit: 32\n\
                    Key: 00 00 00 00 12 34 56 78\nTE: 480\n";
        let dec = Doorbell32Decoder::new();
        let d = dec.from_flipper_key(&FlipperKey::parse(text).unwrap()).unwrap();
        assert_eq!((d.data, d.extra), (0x1234_5678, Some(480)));
        let written = super::super::flipper_key::testing::assert_round_trip(&dec, &d);
        assert!(written.contains("Protocol: Doorbell32\nBit: 32\nKey: 00 00 00 00 12 34 56 78\nTE: 480\n"), "{written}");
        // No TE: loads (sent at 515 µs) and is written back without one; Bit must be 32.
        let no_te = dec.from_flipper_key(&FlipperKey::parse(&text.replace("TE: 480\n", "")).unwrap()).unwrap();
        assert_eq!(no_te.extra, None);
        assert!(!super::super::flipper_key::testing::assert_round_trip(&dec, &no_te).contains("TE:"));
        assert!(dec.from_flipper_key(&FlipperKey::parse(&text.replace("Bit: 32", "Bit: 24")).unwrap()).is_none());
    }

    #[test]
    fn jittered_frames_decode() {
        let mut rng = Rng(0x0D00_BE11);
        for _ in 0..50 {
            let data = 1 + rng.below(0xFFFF_FFFF);
            let te = 420 + rng.below(160) as u32;
            let wire = jitter(&on_air(data, Some(te)), &mut rng, 60);
            let got = feed_all(&mut Doorbell32Decoder::new(), &wire);
            assert!(got.len() >= 6, "{data:08X} at {te}: {}", got.len());
            assert!(got.iter().all(|d| d.data == data && d.extra.unwrap().abs_diff(u64::from(te)) < 20));
        }
    }

    /// Through the whole registry: Doorbell32's frames and nothing else (Princeton syncs on the guard
    /// and drops the 32-bit frames on its bit count).
    #[test]
    fn registry_reads_doorbell32_only() {
        let mut rng = Rng(0x0433_0032);
        for _ in 0..20 {
            let data = 1 + rng.below(0xFFFF_FFFF);
            let te = 450 + rng.below(130) as u32;
            let mut reg = ProtocolRegistry::new();
            let got: Vec<String> = reg
                .process_signal_stream_rf(&on_air(data, Some(te)), 433_920_000, Some(crate::capture::RfModulation::AM))
                .into_iter()
                .map(|(n, d, _)| format!("{n} {:08X}", d.data))
                .collect();
            assert_eq!(got, vec![format!("{NAME} {data:08X}"); 8], "Te {te}");
        }
    }
}
