//! Proteco: 24-bit fixed code of Proteco MOVER gate remotes (a 22-bit serial and a 2-bit button),
//! decoder + encoder.
//!
//! Port of RogueMaster `lib/subghz/protocols/proteco.c` (no other reference has it; its unit-test
//! captures are RogueMaster's `unit_tests/subghz/proteco.sub`, `proteco_raw.sub`,
//! `proteco_b_raw.sub` and `proteco_c_raw.sub`: buttons A, B and C of one TX3 remote). RogueMaster
//! registers it last, after Faac TM.
//!
//! **Air format** (AM, 433.92 MHz): CAME's line coding at a Te of about 440 µs instead of 320 µs
//! ("the CAME decoder accepts the guard and start bit and then rejects every long pulse"): a guard
//! LOW of 20.5·Te (about 9.0 ms), a start HIGH of Te, then 24 cells MSB first, each a LOW then a
//! HIGH: LOW 2·Te + HIGH Te = 1, LOW Te + HIGH 2·Te = 0. The frame repeats while the button is
//! held; the gap that closes one frame is the next one's guard.
//!
//! **Decoder** (`subghz_protocol_decoder_proteco_feed`): Reset waits for a LOW ≥ 4·te_short
//! (1760 µs) and keeps it as the guard; the start bit is a HIGH of 440 ± 150 µs; then cells, a LOW
//! of 440 ± 150 with a HIGH of 880 ± 150 (0) or a LOW of 880 ± 150 with a HIGH of 440 ± 150 (1). A
//! LOW ≥ 1760 µs ends the frame: with exactly 24 bits and a guard that fits the frame's own clock
//! it is reported. The clock is the mean Te of the frame, the sum of its start bit and cells over
//! 3·24 + 1; the guard must be 18.5..=22.5 of it (`PROTECO_GUARD_TE_X2_MIN` / `_MAX`, in half Te:
//! "it is 20.4-20.6 Te on the fob and comes from the same oscillator, so this both admits a remote
//! off clock and keeps out a foreign 24 bit frame that merely happens to sit near 9 ms"). The
//! closing LOW then becomes the next frame's guard and the decoder waits for the start bit. That
//! code runs after the callback and overrides the callback's receiver reset, so a held button
//! reports every frame: KAT keeps that state through the registry's reset that follows its own
//! report (`just_emitted`). There is no repeat check and no checksum.
//!
//! **Fields** (`subghz_protocol_decoder_proteco_get_string`): serial = bits 23..2 (22 bits);
//! button = the low 2 bits through the C's table, 1..=4 ("so that 0 keeps its 'no button seen yet'
//! meaning"): raw 2 → 1 (A, the gate), raw 0 → 2 (B), raw 1 → 3 (C), raw 3 → 4 ("A, B and C are a
//! TX3; the fourth code is unverified"). No counter.
//!
//! **Encoder** (`subghz_protocol_encoder_proteco_get_upload`, `encoder.repeat` = 10): LOW 9020
//! (te_short·41/2), HIGH 440, the 24 cells (1 = LOW 880 + HIGH 440, 0 = LOW 440 + HIGH 880); the
//! upload ends on the last cell's HIGH. `button` 1..=4 (the C's custom buttons and button editor)
//! rewrites the low 2 bits; any other value sends the code as it is.
//!
//! **Key file**: `Protocol: Proteco`, `Bit: 24` (required), `Key` (the generic serialize).

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
    make: "Proteco",
    category: Category::Gate,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

pub(super) const NAME: &str = "Proteco";
const TE_SHORT: u32 = 440;
const TE_LONG: u32 = 880;
const TE_DELTA: u32 = 150;
const MIN_COUNT_BIT: usize = 24;
/// A LOW this long (4·te_short) is a guard: it ends a frame and opens the next.
const GUARD_MIN: u32 = TE_SHORT * 4;
/// `PROTECO_GUARD_TE_X2` and its `_MIN` / `_MAX`: the guard in half Te.
const GUARD_TE_X2: u32 = 41;
const GUARD_TE_X2_MIN: u32 = 37;
const GUARD_TE_X2_MAX: u32 = 45;
/// `PROTECO_BTN_MASK`: the low 2 bits are the button.
const BTN_MASK: u64 = 0x3;
/// `subghz_protocol_proteco_btn`: the low 2 bits → the button, 1..=4.
const BTN: [u8; 4] = [0x2, 0x3, 0x1, 0x4];
/// `subghz_protocol_proteco_btn_raw`: the button → the low 2 bits.
const BTN_RAW: [u64; 5] = [0x0, 0x2, 0x0, 0x1, 0x3];
/// The remote's buttons as the C counts them (its custom buttons walk these four).
pub(super) const BUTTONS: [u8; 4] = [1, 2, 3, 4];
/// `subghz_protocol_encoder_common_alloc(…, 10, 50)`.
const TX_REPEAT: usize = 10;

/// The TX3's buttons (RogueMaster's "A (gate) = 0x2, B = 0x0, C = 0x1"); the fourth code has no
/// name ("unverified").
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    match code {
        1 => Some("A"),
        2 => Some("B"),
        3 => Some("C"),
        _ => None,
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    FoundStartBit,
    SaveDuration,
    CheckDuration,
}

/// Proteco decoder (`SubGhzProtocolDecoderProteco`).
pub struct ProtecoDecoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
    /// The sum of the frame's pulses (the C's `te`, a `uint32_t`).
    te: u32,
    /// The LOW that opened the frame.
    guard: u32,
    /// The last `feed` reported a frame: keep the post-callback state through the registry's reset.
    just_emitted: bool,
}

/// A decode (or a loaded key) of the 24-bit `data`, with the C's serial / button split.
fn decoded(data: u64) -> DecodedSignal {
    let data = data & 0xFF_FFFF;
    DecodedSignal {
        serial: Some(((data >> 2) & 0x3F_FFFF) as u32),
        button: Some(BTN[(data & BTN_MASK) as usize]),
        counter: None,
        crc_valid: false,
        data,
        data_count_bit: MIN_COUNT_BIT,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

/// The code sent for `button`: 1..=4 rewrites the low 2 bits, anything else keeps `data`.
fn with_button(data: u64, button: u8) -> u64 {
    match BTN_RAW.get(usize::from(button)) {
        Some(&raw) if button >= 1 => (data & !BTN_MASK) | raw,
        _ => data,
    }
}

/// One `get_upload` frame: guard, start bit, 24 cells.
fn build_frame(data: u64) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(MIN_COUNT_BIT * 2 + 2);
    push(&mut out, false, TE_SHORT * GUARD_TE_X2 / 2);
    push(&mut out, true, TE_SHORT);
    for i in (0..MIN_COUNT_BIT).rev() {
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

impl ProtecoDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
            te: 0,
            guard: 0,
            just_emitted: false,
        }
    }

    /// `subghz_protocol_proteco_guard_ok`: is the LOW that opened this frame 20.5 Te of the frame's
    /// own clock? (The C's `uint32_t` arithmetic, so the same rounding.)
    fn guard_ok(&self) -> bool {
        // start bit plus 3 Te per bit
        let te = self.te / (u32::from(self.decode_count_bit) * 3 + 1);
        self.guard >= te.wrapping_mul(GUARD_TE_X2_MIN) / 2 && self.guard <= te.wrapping_mul(GUARD_TE_X2_MAX) / 2
    }
}

impl Default for ProtecoDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for ProtecoDecoder {
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

    /// `subghz_protocol_decoder_common_reset` (skipped right after this decoder's own report: the C
    /// code after its callback overrides the reset).
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
                if !level && duration >= GUARD_MIN {
                    self.guard = duration;
                    self.step = Step::FoundStartBit;
                }
            }
            Step::FoundStartBit => {
                if level && duration_diff!(duration, TE_SHORT) < TE_DELTA {
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.te = duration;
                    self.step = Step::SaveDuration;
                } else {
                    self.step = Step::Reset;
                }
            }
            // a cell opens on the LOW
            Step::SaveDuration => {
                if level {
                    self.step = Step::Reset;
                    return None;
                }
                if duration >= GUARD_MIN {
                    // this gap ends the frame and is the next frame's guard
                    let out = (self.decode_count_bit as usize == MIN_COUNT_BIT && self.guard_ok())
                        .then(|| decoded(self.decode_data));
                    // After the callback (whose receiver reset this overrides).
                    self.guard = duration;
                    self.step = Step::FoundStartBit;
                    self.just_emitted = out.is_some();
                    return out;
                }
                self.te_last = duration;
                self.te = self.te.wrapping_add(duration);
                self.step = Step::CheckDuration;
            }
            // and closes on the HIGH
            Step::CheckDuration => {
                let bit = if !level {
                    None
                } else if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA
                    && duration_diff!(duration, TE_LONG) < TE_DELTA
                {
                    Some(false)
                } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA
                    && duration_diff!(duration, TE_SHORT) < TE_DELTA
                {
                    Some(true)
                } else {
                    None
                };
                match bit {
                    Some(bit) => {
                        add_bit(&mut self.decode_data, &mut self.decode_count_bit, bit);
                        self.te = self.te.wrapping_add(duration);
                        self.step = Step::SaveDuration;
                    }
                    None => self.step = Step::Reset,
                }
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        (decoded.data_count_bit == MIN_COUNT_BIT)
            .then(|| with_repeats(&build_frame(with_button(decoded.data, button)), TX_REPEAT))
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

    /// The encoder's transmission as a receiver sees it, closed by the silence after it.
    fn on_air(data: u64, button: u8) -> Vec<LevelDuration> {
        let mut wire = ProtecoDecoder::new().encode(&decoded(data), button).unwrap();
        push(&mut wire, false, 20_000);
        wire
    }

    /// 10 copies, each closed by the next one's guard (the last by the silence): all 10 decode.
    #[test]
    fn encode_decode_roundtrip() {
        for data in [0xDF2396u64, 0x000001, 0xFFFFFF, 0x5A5A5A, 0x000000] {
            let got = feed_all(&mut ProtecoDecoder::new(), &on_air(data, 0));
            assert_eq!(got.len(), 10, "{data:06X}");
            for d in &got {
                assert_eq!((d.data, d.data_count_bit), (data, 24));
                assert_eq!(d.serial, Some((data >> 2) as u32));
                assert_eq!(d.button, Some(BTN[(data & 3) as usize]));
                assert!(d.encoder_capable && !d.crc_valid && d.counter.is_none());
            }
        }
    }

    #[test]
    fn upload_layout() {
        let up = ProtecoDecoder::new().encode(&decoded(0x800001), 0).unwrap();
        let p = |i: usize| (up[i].level, up[i].duration_us);
        assert_eq!(p(0), (false, 9020));
        assert_eq!(p(1), (true, 440));
        assert_eq!(p(2), (false, 880)); // bit 23 = 1
        assert_eq!(p(3), (true, 440));
        assert_eq!(p(4), (false, 440)); // bit 22 = 0
        assert_eq!(p(5), (true, 880));
        assert_eq!(p(49), (true, 440)); // bit 0 = 1
        // 50 entries a copy: guard, start bit, 48 cell halves; 10 copies.
        assert_eq!(up.len(), 50 * 10);
        assert_eq!(p(50), (false, 9020));
    }

    /// The button table and its inverse, the names, and the encoder's button switch.
    #[test]
    fn buttons() {
        for raw in 0..4u64 {
            let d = decoded(0xDF2394 | raw);
            let btn = d.button.unwrap();
            assert!(BUTTONS.contains(&btn));
            assert_eq!(BTN_RAW[usize::from(btn)], raw);
            assert_eq!(with_button(d.data, btn), d.data, "the captured button sends the capture");
        }
        // RogueMaster's captures: A = 0x2, B = 0x0, C = 0x1.
        assert_eq!(decoded(0xDF2396).button.and_then(|b| native_buttons(NAME, b)), Some("A"));
        assert_eq!(decoded(0xDF2394).button.and_then(|b| native_buttons(NAME, b)), Some("B"));
        assert_eq!(decoded(0xDF2395).button.and_then(|b| native_buttons(NAME, b)), Some("C"));
        assert_eq!(decoded(0xDF2397).button.and_then(|b| native_buttons(NAME, b)), None);
        assert_eq!(decoded(0xDF2397).button, Some(4));
        for (button, want) in [(1u8, 0xDF2396u64), (2, 0xDF2394), (3, 0xDF2395), (4, 0xDF2397), (0, 0xDF2396), (5, 0xDF2396), (0xFF, 0xDF2396)] {
            let got = feed_all(&mut ProtecoDecoder::new(), &on_air(0xDF2396, button));
            assert_eq!(got.len(), 10, "button {button}");
            assert!(got.iter().all(|d| d.data == want && d.serial == Some(0x37C8E5)), "button {button}");
        }
    }

    /// The guard is measured in the frame's own Te: 18.5..=22.5 of it.
    #[test]
    fn guard_follows_the_frames_clock() {
        // One frame at `te`, opened by `guard` and closed by a gap.
        let frame = |te: u32, guard: u32| {
            let mut wire = vec![LevelDuration::new(true, te), LevelDuration::new(false, guard), LevelDuration::new(true, te)];
            for i in (0..24).rev() {
                let (low, high) = if (0xDF2396u32 >> i) & 1 == 1 { (2 * te, te) } else { (te, 2 * te) };
                wire.push(LevelDuration::new(false, low));
                wire.push(LevelDuration::new(true, high));
            }
            wire.push(LevelDuration::new(false, 20_000));
            wire
        };
        let reads = |te: u32, guard: u32| feed_all(&mut ProtecoDecoder::new(), &frame(te, guard)).len() == 1;
        for te in [380u32, 440, 500] {
            assert!(reads(te, te * 41 / 2), "te {te}");
            assert!(reads(te, te * 37 / 2) && reads(te, te * 45 / 2), "te {te}: the window's ends");
            assert!(!reads(te, te * 37 / 2 - 1) && !reads(te, te * 45 / 2 + 1), "te {te}: outside");
        }
        // CAME's 24-bit header (76·320 µs) in front of cells at this Te, and the nominal 9 ms guard
        // in front of cells clocked at 380 µs (23.7 of them): not this remote.
        assert!(!reads(440, 24_320));
        assert!(!reads(380, 9020));
        // 23 and 25 bits: nothing.
        for cells in [23usize, 25] {
            let mut wire = vec![LevelDuration::new(false, 9020), LevelDuration::new(true, 440)];
            for _ in 0..cells {
                wire.push(LevelDuration::new(false, 880));
                wire.push(LevelDuration::new(true, 440));
            }
            wire.push(LevelDuration::new(false, 9020));
            assert!(feed_all(&mut ProtecoDecoder::new(), &wire).is_empty(), "{cells} bits");
        }
    }

    #[test]
    fn key_file_roundtrip() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Proteco\nBit: 24\n\
                    Key: 00 00 00 00 00 DF 23 96\n";
        let dec = ProtecoDecoder::new();
        let d = dec.from_flipper_key(&FlipperKey::parse(text).unwrap()).unwrap();
        assert_eq!((d.data, d.serial, d.button), (0xDF2396, Some(0x37C8E5), Some(1)));
        let written = dec.to_flipper_key(&d).unwrap().to_text();
        assert!(written.contains("Protocol: Proteco\nBit: 24\nKey: 00 00 00 00 00 DF 23 96\n"), "{written}");
        assert!(dec.from_flipper_key(&FlipperKey::parse(&text.replace("Bit: 24", "Bit: 25")).unwrap()).is_none());
    }

    /// RogueMaster's unit test: its three RAW recordings (buttons A, B, C of one remote) decode as
    /// Proteco through the whole registry, as often as the C oracle reports them (4, 11 and 9
    /// times), and the key file `proteco.sub` loads.
    #[test]
    fn rogue_master_unit_test_captures() {
        for (file, key, button, times) in [
            ("proteco_raw.sub", 0xDF2396u64, 1u8, 4usize),
            ("proteco_b_raw.sub", 0xDF2394, 2, 11),
            ("proteco_c_raw.sub", 0xDF2395, 3, 9),
        ] {
            let Some(raw) = reference_file(RM_UNIT_TESTS, file) else {
                return;
            };
            let got = registry_decodes(&raw);
            let names = || got.iter().map(|(n, _)| n.as_str()).collect::<Vec<_>>();
            let ours: Vec<_> = got.iter().filter(|(n, _)| n == NAME).collect();
            assert_eq!(ours.len(), times, "{file}: {:?}", names());
            assert!(ours.iter().all(|(_, d)| d.data == key && d.serial == Some(0x37C8E5) && d.button == Some(button)), "{file}");
            assert_eq!(got.len(), ours.len(), "{file}: {:?}", names());
        }
        let Some(key) = reference_file(RM_UNIT_TESTS, "proteco.sub") else {
            return;
        };
        let key = FlipperKey::parse(&std::fs::read_to_string(key).unwrap()).unwrap();
        let reg = crate::protocols::ProtocolRegistry::new();
        let (name, d) = reg.decode_key(&key).expect("proteco.sub loads");
        assert_eq!(name, NAME);
        assert!(d.data == 0xDF2396 && d.data_count_bit == 24 && d.encoder_capable);
    }

    /// The encoder's transmission through the whole registry: Proteco alone reads it (CAME takes
    /// the guard and the start bit, then drops the frame at its first long pulse).
    #[test]
    fn the_registry_reads_the_upload_as_proteco() {
        let mut rng = Rng(0x9207_EC0A);
        for _ in 0..40 {
            let data = rng.below(0x100_0000);
            let mut reg = crate::protocols::ProtocolRegistry::new();
            let got = reg.process_signal_stream(&on_air(data, 0), 433_920_000);
            assert_eq!(got.len(), 10, "{data:06X}: {:?}", got.iter().map(|(n, ..)| n).collect::<Vec<_>>());
            assert!(got.iter().all(|(n, d, _)| n == NAME && d.data == data), "{data:06X}");
        }
    }

    #[test]
    fn jittered_frames_decode() {
        let mut rng = Rng(0x0F2E_E2E3);
        for _ in 0..50 {
            let data = rng.below(0x100_0000);
            let wire = jitter(&on_air(data, 0), &mut rng, 100);
            let got = feed_all(&mut ProtecoDecoder::new(), &wire);
            assert!(!got.is_empty(), "{data:06X}");
            assert!(got.iter().all(|d| d.data == data));
        }
    }
}
