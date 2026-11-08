//! Legrand: 18-bit fixed code, decoder + encoder.
//!
//! Port of doctor-firmware `lib/subghz/protocols/legrand.c` (the stock Flipper protocol; doctor,
//! Momentum, RogueMaster and FoxFW register it, Flipper-ARF ships its `lib` and garage-app copies
//! unregistered). Every copy decodes and encodes identically: they differ in style only (doctor,
//! RogueMaster and FoxFW use the shared `common.c` helpers; FoxFW loads the same encoder from an SD
//! plugin), in the display text (ARF's `lib` copy leaves out the `Te:` line the others show) and in
//! the "Sensors" filter flag. Unit-test captures: `legrand_raw.sub`, `legrand_2E37F_raw.sub` (the
//! same recording) and the key files `legrand.sub`, `legrand_2E37F.sub`.
//!
//! **Air format** (AM, 433.92 MHz): te ≈ 375 µs (measured per frame). A frame is a sync LOW of
//! 16·te, then 18 cells MSB first, each a LOW then a HIGH: 1 = LOW te + HIGH 3·te, 0 = LOW 3·te +
//! HIGH te. On air the first cell's LOW is part of the sync LOW (the encoder sends them back to
//! back), so the decoder reads the first bit from its HIGH alone.
//!
//! **Decoder** (`subghz_protocol_decoder_legrand_feed`): Reset waits for a LOW with |d − 6000| <
//! 1200; FirstBit takes a HIGH |d − 375| < 150 as 0 or |d − 1125| < 450 as 1 (anything else
//! resets); then LOW/HIGH cells: LOW 1125 ± 450 + HIGH 375 ± 150 = 0, LOW 375 ± 150 + HIGH 1125 ±
//! 450 = 1, anything else resets. The 18th bit ends the frame (the next pulse must be a sync): it is
//! reported when it equals the previous 18-bit frame (`last_data`, non-zero — the repeat is the only
//! check), and becomes `last_data` either way. A broken cell leaves `last_data` alone; only a reset
//! clears it. te = (each first-bit HIGH × 4 or HIGH ÷ 3 × 4, plus every later pulse) / 72. The
//! callback's receiver reset (Reset, `last_data` 0) is overridden by the `last_data = …` after it,
//! so a held button reports every frame after the first: KAT keeps that state through the
//! registry's reset that follows its own report (`just_emitted`).
//!
//! **Fields**: none in the C (no serial or button: the 18 bits are the code); `extra` = the
//! measured te.
//!
//! **Encoder** (`subghz_protocol_encoder_legrand_get_upload`, `encoder.repeat` = 3): LOW 16·te,
//! then the 18 cells, at the key's TE (the C uses it as given). `button` is ignored.
//!
//! **Key file**: `Protocol: Legrand`, `Bit: 18`, `Key`, `TE` (the decoder's serialize writes the
//! measured te). Both the decoder's and the encoder's deserialize require `Bit: 18` and the `TE`
//! line.

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
    make: "Legrand",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

pub(super) const NAME: &str = "Legrand";
const TE_SHORT: u32 = 375;
const TE_LONG: u32 = 1125;
const TE_DELTA: u32 = 150;
const MIN_COUNT_BIT: usize = 18;
/// `subghz_protocol_encoder_legrand_alloc`: repeat = 3.
const TX_REPEAT: usize = 3;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    FirstBit,
    SaveDuration,
    CheckDuration,
}

/// Legrand decoder (`SubGhzProtocolDecoderLegrand`).
pub struct LegrandDecoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
    /// The C's `te`: the sum of the frame's pulses (first bit scaled), then the mean.
    te: u32,
    /// The C's `uint32_t last_data`: the previous 18-bit frame.
    last_data: u32,
    /// The last `feed` reported a frame: keep the post-callback state through the registry's reset.
    just_emitted: bool,
}

/// A decode (or a loaded key) of `data` (18 bits) measured at `te`.
fn decoded(data: u64, te: u32) -> DecodedSignal {
    DecodedSignal {
        serial: None,
        button: None,
        counter: None,
        crc_valid: false,
        data,
        data_count_bit: MIN_COUNT_BIT,
        encoder_capable: true,
        extra: Some(u64::from(te)),
        protocol_display_name: None,
    }
}

/// One `get_upload` frame at `te` (the sync LOW and the first cell's LOW merge).
fn build_frame(data: u64, te: u32) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(MIN_COUNT_BIT * 2);
    push(&mut out, false, te.wrapping_mul(16));
    for i in (0..MIN_COUNT_BIT).rev() {
        if (data >> i) & 1 == 1 {
            push(&mut out, false, te);
            push(&mut out, true, te.wrapping_mul(3));
        } else {
            push(&mut out, false, te.wrapping_mul(3));
            push(&mut out, true, te);
        }
    }
    out
}

impl LegrandDecoder {
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

impl Default for LegrandDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for LegrandDecoder {
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

    /// `subghz_protocol_decoder_legrand_reset` (skipped right after this decoder's own report: the
    /// C code after its callback overrides the reset).
    fn reset(&mut self) {
        if std::mem::take(&mut self.just_emitted) {
            return;
        }
        self.step = Step::Reset;
        self.last_data = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        self.just_emitted = false;
        match self.step {
            Step::Reset => {
                if !level && duration_diff!(duration, TE_SHORT * 16) < TE_DELTA * 8 {
                    self.step = Step::FirstBit;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.te = 0;
                }
            }
            Step::FirstBit => {
                if level {
                    // long low that is part of the sync, then a short high
                    if duration_diff!(duration, TE_SHORT) < TE_DELTA {
                        add_bit(&mut self.decode_data, &mut self.decode_count_bit, false);
                        self.te = self.te.wrapping_add(duration.wrapping_mul(4));
                    }
                    // short low that is part of the sync, then a long high
                    if duration_diff!(duration, TE_LONG) < TE_DELTA * 3 {
                        add_bit(&mut self.decode_data, &mut self.decode_count_bit, true);
                        self.te = self.te.wrapping_add(duration / 3 * 4);
                    }
                    if self.decode_count_bit > 0 {
                        self.step = Step::SaveDuration;
                        return None;
                    }
                }
                self.step = Step::Reset;
            }
            Step::SaveDuration => {
                if !level {
                    self.te_last = duration;
                    self.te = self.te.wrapping_add(duration);
                    self.step = Step::CheckDuration;
                    return None;
                }
                self.step = Step::Reset;
            }
            Step::CheckDuration => {
                if level {
                    let mut found = false;
                    if duration_diff!(self.te_last, TE_LONG) < TE_DELTA * 3
                        && duration_diff!(duration, TE_SHORT) < TE_DELTA
                    {
                        found = true;
                        add_bit(&mut self.decode_data, &mut self.decode_count_bit, false);
                    }
                    if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA
                        && duration_diff!(duration, TE_LONG) < TE_DELTA * 3
                    {
                        found = true;
                        add_bit(&mut self.decode_data, &mut self.decode_count_bit, true);
                    }
                    if found {
                        self.te = self.te.wrapping_add(duration);
                        if usize::from(self.decode_count_bit) < MIN_COUNT_BIT {
                            self.step = Step::SaveDuration;
                            return None;
                        }
                        // A full frame: reported only when it repeats the previous one.
                        let out = (self.last_data != 0 && u64::from(self.last_data) == self.decode_data).then(|| {
                            self.te /= u32::from(self.decode_count_bit) * 4;
                            decoded(self.decode_data, self.te)
                        });
                        // After the callback (whose receiver reset this overrides).
                        self.last_data = self.decode_data as u32;
                        self.step = Step::Reset;
                        self.just_emitted = out.is_some();
                        return out;
                    }
                }
                self.step = Step::Reset;
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
        Some(with_repeats(&build_frame(decoded.data, te), TX_REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != MIN_COUNT_BIT {
            return None;
        }
        Some(decoded(key.key, key.u32_field("TE")?))
    }

    fn to_flipper_key(&self, d: &DecodedSignal) -> Option<FlipperKey> {
        let mut key = FlipperKey::new(NAME, d.data_count_bit, d.data, 433_920_000, "FuriHalSubGhzPresetOok650Async");
        key.push("TE", (d.extra? as u32).to_string());
        Some(key)
    }
}

#[cfg(test)]
pub(super) mod testing {
    //! Frames for other modules' tests and the parity tools.
    use super::*;

    /// A decode of `data` at `te`, as a key file gives it.
    pub fn frame(data: u64, te: u32) -> DecodedSignal {
        decoded(data & 0x3_FFFF, te)
    }

    /// The encoder's upload for `data` at `te`, with a trailing silence.
    pub fn on_air(data: u64, te: u32) -> Vec<LevelDuration> {
        let mut wire = LegrandDecoder::new().encode(&frame(data, te), 0).unwrap();
        push(&mut wire, false, 30_000);
        wire
    }
}

#[cfg(test)]
mod tests {
    use super::super::f_common::test_support::*;
    use super::*;

    const UNIT_TEST_KEY: &str = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
        Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Legrand\nBit: 18\nKey: 00 00 00 00 00 02 E3 7F\nTE: 358\n";

    /// 3 copies: the first is stored, the second and third repeat it (2 reports, as the C's
    /// merged-pulse round trip).
    #[test]
    fn encode_decode_roundtrip() {
        // The merged sync (17 or 19·te) must stay inside 4801..=7199 µs.
        for (data, te) in [(0x2E37Fu64, 358), (0x00001, 375), (0x3FFFF, 300), (0x20000, 400)] {
            let got = feed_all(&mut LegrandDecoder::new(), &testing::on_air(data, te));
            assert_eq!(got.len(), 2, "{data:05X} at {te}");
            for d in &got {
                assert_eq!((d.data, d.data_count_bit), (data, 18));
                // The mean of exact pulses (the first bit's HIGH ÷ 3 × 4 truncates) is te.
                assert!(d.extra.unwrap().abs_diff(u64::from(te)) <= 1, "{:?}", d.extra);
                assert!(d.encoder_capable && d.serial.is_none() && d.button.is_none());
            }
        }
        // All-zero frames never report (last_data must be non-zero).
        assert!(feed_all(&mut LegrandDecoder::new(), &testing::on_air(0, 375)).is_empty());
    }

    #[test]
    fn upload_layout() {
        let up = LegrandDecoder::new().encode(&testing::frame(0x2E37F, 358), 0).unwrap();
        let p = |i: usize| (up[i].level, up[i].duration_us);
        // sync 16·te merged with bit 17 (1)'s LOW te, its HIGH 3·te, bit 16 (0): LOW 3·te + HIGH te.
        assert_eq!(p(0), (false, 16 * 358 + 358));
        assert_eq!(p(1), (true, 3 * 358));
        assert_eq!(p(2), (false, 3 * 358));
        assert_eq!(p(3), (true, 358));
        // 36 entries a copy (the copies' syncs merge with their first LOW), 3 copies.
        assert_eq!(up.len(), 36 * 3);
        assert_eq!(p(35), (true, 3 * 358)); // bit 0 = 1
        assert_eq!(p(36), (false, 16 * 358 + 358));
    }

    #[test]
    fn rejects_bad_sync_cells_and_single_frames() {
        // One frame alone: stored, never reported. (No silence before a frame: it would merge
        // with the sync LOW.)
        let mut once = build_frame(0x2E37F, 375);
        push(&mut once, false, 30_000);
        assert!(feed_all(&mut LegrandDecoder::new(), &once).is_empty());
        // Two different codes in a row: neither is reported; the same code twice is.
        let pair = |a: u64, b: u64| {
            let mut w = Vec::new();
            for p in build_frame(a, 375).into_iter().chain(build_frame(b, 375)) {
                push(&mut w, p.level, p.duration_us);
            }
            push(&mut w, false, 30_000);
            feed_all(&mut LegrandDecoder::new(), &w).len()
        };
        assert_eq!((pair(0x2E37F, 0x2E37E), pair(0x2E37F, 0x2E37F)), (0, 1));
        // Sync window 4801..=7199 µs (the merged sync is 16·te + te or + 3·te).
        for (sync, ok) in [(4800u32, false), (4801, true), (7199, true), (7200, false)] {
            let mut w = testing::on_air(0x15555, 375);
            for p in w.iter_mut().filter(|p| !p.level && p.duration_us > 3000 && p.duration_us < 30_000) {
                p.duration_us = sync;
            }
            assert_eq!(feed_all(&mut LegrandDecoder::new(), &w).len(), if ok { 2 } else { 0 }, "sync {sync}");
        }
        // A long HIGH of 1575 µs is outside 1125 ± 450.
        let mut w = testing::on_air(0x2E37F, 375);
        w[3].duration_us = 1575;
        assert!(feed_all(&mut LegrandDecoder::new(), &w).len() < 2);
    }

    /// A broken frame between two good ones keeps `last_data`: the third frame still pairs with
    /// the first (the C clears it only on reset).
    #[test]
    fn broken_frame_keeps_the_memory() {
        let good = build_frame(0x0F0F0, 375);
        let mut bad = good.clone();
        bad[7].duration_us = 2000;
        let mut w = Vec::new();
        for p in good.iter().chain(&bad).chain(&good) {
            push(&mut w, p.level, p.duration_us);
        }
        push(&mut w, false, 30_000);
        assert_eq!(feed_all(&mut LegrandDecoder::new(), &w).len(), 1);
    }

    #[test]
    fn key_file_roundtrip() {
        let dec = LegrandDecoder::new();
        let d = dec.from_flipper_key(&FlipperKey::parse(UNIT_TEST_KEY).unwrap()).unwrap();
        assert_eq!((d.data, d.data_count_bit, d.extra), (0x2E37F, 18, Some(358)));
        let text = super::super::flipper_key::testing::assert_round_trip(&dec, &d);
        assert!(text.contains("Protocol: Legrand\nBit: 18\nKey: 00 00 00 00 00 02 E3 7F\nTE: 358\n"), "{text}");
        // TE and Bit: 18 are required (both deserializers).
        let no_te = UNIT_TEST_KEY.replace("TE: 358\n", "");
        assert!(dec.from_flipper_key(&FlipperKey::parse(&no_te).unwrap()).is_none());
        assert!(dec.from_flipper_key(&FlipperKey::parse(&UNIT_TEST_KEY.replace("Bit: 18", "Bit: 24")).unwrap()).is_none());
    }

    /// Flipper's `subghz_decoder_legrand_test` captures through the whole registry: 175 Legrand
    /// frames each, all 0x2E37F, with the doctor oracle's Te values (1 × 356, 168 × 357, 6 × 358;
    /// ARF / Momentum / RogueMaster / FoxFW give the same), and nothing else.
    #[test]
    fn unit_test_captures_decode() {
        for file in ["legrand_raw.sub", "legrand_2E37F_raw.sub"] {
            let Some(path) = reference_file(ARF_UNIT_TESTS, file) else {
                return;
            };
            let got = registry_decodes(&path);
            let ours: Vec<&DecodedSignal> = got.iter().filter(|(n, _)| n == NAME).map(|(_, d)| d).collect();
            assert!(ours.iter().all(|d| d.data == 0x2E37F), "{file}");
            let te = |t: u64| ours.iter().filter(|d| d.extra == Some(t)).count();
            assert_eq!((ours.len(), te(356), te(357), te(358)), (175, 1, 168, 6), "{file}");
            assert_eq!(got.len(), ours.len(), "{file}: another protocol took a frame");
        }
    }

    /// `subghz_encoder_legrand_test`: the unit-test key encodes and decodes back through the
    /// registry (on air, equal levels merged: the C's own unmerged round trip fails).
    #[test]
    fn unit_test_key_encodes_back() {
        let reg = super::super::ProtocolRegistry::new();
        let (name, d) = reg.decode_key(&FlipperKey::parse(UNIT_TEST_KEY).unwrap()).unwrap();
        assert_eq!(name, NAME);
        let tx = reg.get(&name).unwrap().encode(&d, 0).unwrap();
        let mut reg = super::super::ProtocolRegistry::new();
        let back = reg.process_signal_stream(&tx, 433_920_000);
        assert_eq!(back.len(), 2, "{back:?}");
        assert!(back.iter().all(|(n, x, _)| n == NAME && x.data == 0x2E37F && x.extra == Some(358)), "{back:?}");
    }

    #[test]
    fn jittered_frames_decode() {
        let mut rng = Rng(0x1E6_2A4D);
        for _ in 0..50 {
            let data = 1 + rng.below(0x3_FFFF);
            let te = 330 + rng.below(40) as u32; // merged sync ≤ 19·369 + 60 < 7200
            let wire = jitter(&testing::on_air(data, te), &mut rng, 60);
            let got = feed_all(&mut LegrandDecoder::new(), &wire);
            assert!(!got.is_empty(), "{data:05X} at {te}");
            assert!(got.iter().all(|d| d.data == data));
        }
    }
}
