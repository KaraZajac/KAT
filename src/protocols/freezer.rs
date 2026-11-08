//! Freezer: 24-bit fixed code (a 20-bit serial and a 4-bit button), decoder + encoder.
//!
//! Port of RogueMaster `lib/subghz/protocols/freezer.c` (no other reference has it; its unit-test
//! captures are RogueMaster's `unit_tests/subghz/freezer.sub` and `freezer_raw.sub`). RogueMaster
//! registers it after Cardin S508, before Faac TM.
//!
//! **Air format** (AM, 433.92 MHz): a carrier-off gap of at least 30·te_short (7500 µs), one short
//! HIGH that belongs to no cell, then 24 cells MSB first, each a LOW then a HIGH: short LOW + long
//! HIGH = 1, long LOW + short HIGH = 0. Nominal short 250 µs, long 500 µs; the encoder closes each
//! frame with a 9000 µs (36·te_short) gap.
//!
//! **Decoder** (`subghz_protocol_decoder_freezer_feed`): the symbols are classified by a split
//! rather than by windows (the C: "the measured classes, 158..359 and 392..594, are only 33us
//! apart while each spreads over 200us"): short = 120..=375 µs, long = 376..=700 µs, anything else
//! breaks the frame; refusing everything over 700 µs keeps Princeton frames (long symbol ≈ 1170 µs)
//! out. Reset waits for a LOW ≥ 7500 µs; Start needs a short HIGH (the lone pulse); then cells. A
//! LOW ≥ 7500 µs ends the frame: a 24-bit frame equal to the previous 24-bit frame (`last_data`,
//! non-zero) is reported ("a remote repeats its frame for as long as the button is held, a stray run
//! of plausible cells does not"); the frame then becomes `last_data` (any other length clears it)
//! and the decoder waits for the next lone HIGH. A broken cell clears `last_data`. The callback's
//! receiver reset (`freezer_reset`: Reset, `last_data` 0) is overridden by the code after it, so a
//! held button reports every frame after the first: KAT keeps that state through the registry's
//! reset that follows its own report (`just_emitted`).
//!
//! **Fields** (`subghz_protocol_freezer_check_remote_controller`): serial = bits 23..4 (20 bits),
//! button = bits 3..0; no counter, no checksum (the repeat is the only check).
//!
//! **Encoder** (`subghz_protocol_encoder_freezer_get_upload`, `encoder.repeat` = 3): lone HIGH 250,
//! the 24 cells (1 = LOW 250 + HIGH 500, 0 = LOW 500 + HIGH 250), LOW 9000; the captured code (the C
//! has no button override, so `button` is ignored).
//!
//! **Key file**: `Protocol: Freezer`, `Bit: 24` (required), `Key` (the generic serialize).

use super::f_common::{add_bit, push, with_repeats};
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
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
    make: "Freezer",
    category: Category::Gate,
    buttons: Some(super::g1_common::numbered_buttons),
    presets: &[&AM650],
};

pub(super) const NAME: &str = "Freezer";
const TE_SHORT: u32 = 250;
const TE_LONG: u32 = 500;
const TE_DELTA: u32 = 120;
const MIN_COUNT_BIT: usize = 24;
/// `FREEZER_SYMBOL_MIN` / `_SPLIT` / `_MAX`.
const SYMBOL_MIN: u32 = 120;
const SYMBOL_SPLIT: u32 = 375;
const SYMBOL_MAX: u32 = 700;
/// `FREEZER_GAP_MIN` (30·te_short) and the encoder's `FREEZER_GAP_DURATION` (36·te_short).
const GAP_MIN: u32 = TE_SHORT * 30;
const GAP_DURATION: u32 = TE_SHORT * 36;
/// `subghz_protocol_encoder_common_alloc(…, 3, …)`.
const TX_REPEAT: usize = 3;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    Start,
    SaveDuration,
    CheckDuration,
}

/// Freezer decoder (`SubGhzProtocolDecoderFreezer`).
pub struct FreezerDecoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
    last_data: u64,
    /// The last `feed` reported a frame: keep the post-callback state through the registry's reset.
    just_emitted: bool,
}

fn is_short(d: u32) -> bool {
    (SYMBOL_MIN..=SYMBOL_SPLIT).contains(&d)
}

fn is_long(d: u32) -> bool {
    d > SYMBOL_SPLIT && d <= SYMBOL_MAX
}

/// A decode (or a loaded key) of the 24-bit `data`, with the C's serial / button split.
fn decoded(data: u64) -> DecodedSignal {
    let data = data & 0xFF_FFFF;
    DecodedSignal {
        serial: Some(((data >> 4) & 0xF_FFFF) as u32),
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

/// One `get_upload` frame.
fn build_frame(data: u64) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(MIN_COUNT_BIT * 2 + 2);
    push(&mut out, true, TE_SHORT);
    for i in (0..MIN_COUNT_BIT).rev() {
        if (data >> i) & 1 == 1 {
            push(&mut out, false, TE_SHORT);
            push(&mut out, true, TE_LONG);
        } else {
            push(&mut out, false, TE_LONG);
            push(&mut out, true, TE_SHORT);
        }
    }
    push(&mut out, false, GAP_DURATION);
    out
}

impl FreezerDecoder {
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
}

impl Default for FreezerDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for FreezerDecoder {
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

    /// `subghz_protocol_decoder_freezer_reset` (skipped right after this decoder's own report: the
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
                if !level && duration >= GAP_MIN {
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.step = Step::Start;
                }
            }
            Step::Start => {
                self.step = if level && is_short(duration) { Step::SaveDuration } else { Step::Reset };
            }
            // a cell opens on the LOW
            Step::SaveDuration => {
                if level {
                    self.step = Step::Reset;
                    return None;
                }
                if duration >= GAP_MIN {
                    let full = self.decode_count_bit as usize == MIN_COUNT_BIT;
                    let out = (full && self.decode_data == self.last_data && self.last_data != 0)
                        .then(|| decoded(self.decode_data));
                    // After the callback (whose receiver reset this overrides).
                    self.last_data = if full { self.decode_data } else { 0 };
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.step = Step::Start;
                    self.just_emitted = out.is_some();
                    return out;
                }
                self.te_last = duration;
                self.step = Step::CheckDuration;
            }
            // and closes on the HIGH
            Step::CheckDuration => {
                if !level {
                    self.step = Step::Reset;
                    return None;
                }
                if is_short(self.te_last) && is_long(duration) {
                    add_bit(&mut self.decode_data, &mut self.decode_count_bit, true);
                    self.step = Step::SaveDuration;
                } else if is_long(self.te_last) && is_short(duration) {
                    add_bit(&mut self.decode_data, &mut self.decode_count_bit, false);
                    self.step = Step::SaveDuration;
                } else {
                    self.last_data = 0;
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
        (decoded.data_count_bit == MIN_COUNT_BIT)
            .then(|| with_repeats(&build_frame(decoded.data), TX_REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        (key.bits == MIN_COUNT_BIT).then(|| decoded(key.key))
    }

    fn to_flipper_key(&self, d: &DecodedSignal) -> Option<FlipperKey> {
        Some(FlipperKey::new(NAME, d.data_count_bit, d.data, 433_920_000, "FuriHalSubGhzPresetOok650Async"))
    }
}

#[cfg(test)]
mod tests {
    use super::super::f_common::test_support::*;
    use super::*;

    /// RogueMaster's unit-test resources (REFERENCES is a gitignored clone).
    const RM_UNIT_TESTS: &str =
        "REFERENCES/RogueMaster/applications/debug/unit_tests/resources/unit_tests/subghz";

    fn on_air(data: u64) -> Vec<LevelDuration> {
        let mut wire = vec![LevelDuration::new(false, 20_000)];
        for p in FreezerDecoder::new().encode(&decoded(data), 0).unwrap() {
            push(&mut wire, p.level, p.duration_us);
        }
        wire
    }

    /// 3 copies: the first is stored, the second and third repeat it.
    #[test]
    fn encode_decode_roundtrip() {
        for data in [0xC31C1Fu64, 0x000001, 0xFFFFFF, 0x5A5A5A] {
            let got = feed_all(&mut FreezerDecoder::new(), &on_air(data));
            assert_eq!(got.len(), 2, "{data:06X}");
            for d in &got {
                assert_eq!((d.data, d.data_count_bit), (data, 24));
                assert_eq!((d.serial, d.button), (Some((data >> 4) as u32 & 0xFFFFF), Some(data as u8 & 0xF)));
            }
        }
    }

    #[test]
    fn upload_layout() {
        let up = FreezerDecoder::new().encode(&decoded(0x800001), 0).unwrap();
        let p = |i: usize| (up[i].level, up[i].duration_us);
        assert_eq!(p(0), (true, 250));
        assert_eq!(p(1), (false, 250)); // bit 23 = 1
        assert_eq!(p(2), (true, 500));
        assert_eq!(p(3), (false, 500)); // bit 22 = 0
        assert_eq!(p(4), (true, 250));
        // 50 entries a copy: lone HIGH, 48 cell halves, the gap; 3 copies.
        assert_eq!(up.len(), 50 * 3);
        assert_eq!(p(49), (false, 9000));
    }

    #[test]
    fn needs_a_repeat_and_clean_symbols() {
        // One frame alone (then silence): stored, never reported.
        let mut once = vec![LevelDuration::new(false, 20_000)];
        for p in build_frame(0xC31C1F) {
            push(&mut once, p.level, p.duration_us);
        }
        assert!(feed_all(&mut FreezerDecoder::new(), &once).is_empty());
        // Two different codes in a row: neither is reported.
        let mut two = once.clone();
        for p in build_frame(0xC31C1E) {
            push(&mut two, p.level, p.duration_us);
        }
        assert!(feed_all(&mut FreezerDecoder::new(), &two).is_empty());
        // A symbol over 700 µs (Princeton's long) breaks the frame and clears the memory.
        let bad: Vec<LevelDuration> = on_air(0xC31C1F)
            .into_iter()
            .enumerate()
            .map(|(i, p)| if i == 60 { LevelDuration::new(p.level, 1170) } else { p })
            .collect();
        assert!(feed_all(&mut FreezerDecoder::new(), &bad).len() < 2);
    }

    #[test]
    fn key_file_roundtrip() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Freezer\nBit: 24\n\
                    Key: 00 00 00 00 00 C3 1C 1F\n";
        let dec = FreezerDecoder::new();
        let d = dec.from_flipper_key(&FlipperKey::parse(text).unwrap()).unwrap();
        assert_eq!((d.data, d.serial, d.button), (0xC31C1F, Some(0xC31C1), Some(0xF)));
        let written = dec.to_flipper_key(&d).unwrap().to_text();
        assert!(written.contains("Protocol: Freezer\nBit: 24\nKey: 00 00 00 00 00 C3 1C 1F\n"), "{written}");
        assert!(dec.from_flipper_key(&FlipperKey::parse(&text.replace("Bit: 24", "Bit: 25")).unwrap()).is_none());
    }

    /// RogueMaster's unit test: `freezer_raw.sub` decodes as Freezer 0xC31C1F through the whole
    /// registry (the C oracle reports it 5 times), and the key file `freezer.sub` loads.
    #[test]
    fn rogue_master_unit_test_captures() {
        let Some(raw) = reference_file(RM_UNIT_TESTS, "freezer_raw.sub") else {
            return;
        };
        let got = registry_decodes(&raw);
        let ours: Vec<_> = got.iter().filter(|(n, _)| n == NAME).collect();
        assert_eq!(ours.len(), 5, "{:?}", got.iter().map(|(n, _)| n).collect::<Vec<_>>());
        assert!(ours.iter().all(|(_, d)| d.data == 0xC31C1F));
        assert_eq!(got.len(), ours.len(), "{:?}", got.iter().map(|(n, _)| n).collect::<Vec<_>>());
        let Some(key) = reference_file(RM_UNIT_TESTS, "freezer.sub") else {
            return;
        };
        let key = FlipperKey::parse(&std::fs::read_to_string(key).unwrap()).unwrap();
        let reg = crate::protocols::ProtocolRegistry::new();
        let (name, d) = reg.decode_key(&key).expect("freezer.sub loads");
        assert_eq!(name, NAME);
        assert!(d.data_count_bit == 24 && d.encoder_capable);
    }

    #[test]
    fn jittered_frames_decode() {
        let mut rng = Rng(0x0F2E_E2E2);
        for _ in 0..50 {
            let data = 1 + rng.below(0xFF_FFFF);
            let wire = jitter(&on_air(data), &mut rng, 60);
            let got = feed_all(&mut FreezerDecoder::new(), &wire);
            assert!(!got.is_empty(), "{data:06X}");
            assert!(got.iter().all(|d| d.data == data));
        }
    }
}
