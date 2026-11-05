//! Hollarm 42-bit fixed code (Hollarm GSM alarm remotes).
//!
//! Reference: Flipper-ARF `lib/subghz/protocols/hollarm.c` (@xMasterX, Unleashed lineage; its
//! garage-app copy `applications/system/subghz_garage/protocols/hollarm.c`, Momentum's and
//! RogueMaster's decode and encode identically; ARF ships it unregistered, Momentum / RogueMaster /
//! the stock firmware register it). doctor-firmware's copy differs on the wire in three places,
//! all on the decoder side (its encoder is the same): the header gap window is |d − 2400| < 200
//! instead of < 400, it rejects "degenerate" keys (`subghz_protocol_blocks_key_is_degenerate`),
//! and after a frame it keeps decoding from the gap that closed it, so every repeat of a held
//! button decodes (24 frames in the unit-test capture vs 12) — its callback's receiver reset is
//! overridden by the `parser_step = SaveDuration` that follows it. KAT follows Flipper-ARF (the
//! reference priority, and the copy the other three firmwares ship): a decode ends in Reset, as
//! KAT's registry resets every decoder after a decode.
//!
//! **Air format** (AM, 433.92 MHz): te_short = 200 µs, te_long = 1000 µs, te_delta = 200 µs. Each
//! bit is a 200 µs HIGH followed by a LOW: 0 = LOW 1000, 1 = LOW 1600 (8·te_short); the last bit's
//! LOW is the 2400 µs (12·te_short) gap. 42 bits, MSB first: the 40-bit key followed by two 0
//! bits.
//!
//! **Decoder** (`subghz_protocol_decoder_hollarm_feed`): Reset waits for a LOW with
//! |d − 2400| < 400; then HIGH/LOW pairs (HIGH |d − 200| < 200; LOW |d − 1000| < 200 → 0,
//! |d − 1600| < 200 → 1). A LOW with |d − 2400| < 200 closes the frame and adds a final 0 bit;
//! with exactly 42 bits, key = bits >> 2 (40 bits) is reported when its byte sum holds, and the
//! decoder returns to Reset (so a held button reports every other frame).
//!
//! **Frame** (40-bit key, the C's comments and `subghz_protocol_hollarm_remote_controller`):
//! bits 39..16 = serial (the C: (key & 0xFFFFFFF0000) >> 16), bits 15..12 = a fixed nibble,
//! bits 11..8 = button, bits 7..0 = (byte 4 + byte 3 + byte 2 + byte 1) & 0xFF.
//!
//! **KAT fields**: `serial`, `button` as the C; `counter` None; `data` = the 40-bit key,
//! `data_count_bit` = 42 (the C's bit count, `Bit: 42` in key files); `crc_valid` = the byte sum
//! holds (always for decodes: the C drops the others).
//!
//! **Encoder** (`subghz_protocol_encoder_hollarm_get_upload`, `encoder.repeat` = 3): key =
//! (key >> 12) << 12 | btn << 8 plus the new byte sum, then the 42 bits of key << 2 as pulses,
//! the last LOW being the 2400 µs gap; 3 copies. `button` is KAT's native code: 0..=0xF is sent
//! (the references' button editor takes any 4-bit value; their D-pad cycles 0x2 Arm / 0x1 Disarm /
//! 0x4 Ringtone/Alarm / 0x8 Ring), anything else keeps the captured button. Static: no counter.
//!
//! **Key file**: `Protocol: Hollarm`, `Bit: 42`, `Key` = the 40-bit key (generic serialize);
//! loading requires `Bit: 42`.

use super::f_common::{add_bit, push, with_repeats};
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
/// `SubGhzProtocolFlag_433 | AM`, `SubGhzProtocolTypeStatic`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Hollarm",
    category: Category::Gate,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

const NAME: &str = "Hollarm";
const TE_SHORT: u32 = 200;
const TE_LONG: u32 = 1000;
const TE_DELTA: u32 = 200;
const BIT_COUNT: usize = 42;
/// `subghz_protocol_encoder_hollarm_alloc`: repeat = 3.
const REPEAT: usize = 3;
/// The gap after the last bit: 12·te_short.
const GAP: u32 = TE_SHORT * 12;
/// The buttons of the remote (the C's custom-button D-pad cycles these four).
pub(super) const DPAD: [u8; 4] = [0x2, 0x1, 0x4, 0x8];

/// `subghz_protocol_hollarm_get_button_name` (its one two-line label on one line).
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    const NAMES: [&str; 16] = [
        "Unknown",
        "Disarm",
        "Arm",
        "0x3",
        "Ringtone/Alarm",
        "0x5",
        "0x6",
        "0x7",
        "Ring",
        "Settings mode",
        "Exit settings",
        "Vibro sens. setting",
        "Not used (in settings)",
        "Volume setting",
        "0xE",
        "0xF",
    ];
    Some(NAMES.get(usize::from(code)).copied().unwrap_or(NAMES[0]))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    SaveDuration,
    CheckDuration,
}

/// Hollarm decoder (`SubGhzProtocolDecoderHollarm`).
pub struct HollarmDecoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
}

/// (byte 4 + byte 3 + byte 2 + byte 1) & 0xFF of a 40-bit key.
fn bytesum(key: u64) -> u8 {
    [32, 24, 16, 8]
        .iter()
        .fold(0u8, |s, &shift| s.wrapping_add((key >> shift) as u8))
}

fn signal(key: u64) -> DecodedSignal {
    DecodedSignal {
        serial: Some(((key & 0xFFF_FFFF_0000) >> 16) as u32),
        button: Some(((key >> 8) & 0xF) as u8),
        counter: None,
        crc_valid: bytesum(key) == key as u8,
        data: key,
        data_count_bit: BIT_COUNT,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

/// The key the encoder sends for `btn` (new button, new byte sum).
fn rebuild(key: u64, btn: u8) -> u64 {
    let new_key = ((key >> 12) << 12) | (u64::from(btn) << 8);
    new_key | u64::from(bytesum(new_key))
}

/// One upload: the 42 bits of `key << 2`, the gap after the last.
fn upload(key: u64) -> Vec<LevelDuration> {
    let wire = key << 2;
    let mut out = Vec::with_capacity(2 * BIT_COUNT);
    for i in (1..=BIT_COUNT).rev() {
        let one = (wire >> (i - 1)) & 1 == 1;
        push(&mut out, true, TE_SHORT);
        push(
            &mut out,
            false,
            if i == 1 {
                GAP
            } else if one {
                TE_SHORT * 8
            } else {
                TE_LONG
            },
        );
    }
    out
}

impl HollarmDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
        }
    }
}

impl Default for HollarmDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for HollarmDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: BIT_COUNT,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.step = Step::Reset;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                // Flipper-ARF: te_delta·2 (doctor-firmware: te_delta).
                if !level && duration.abs_diff(GAP) < TE_DELTA * 2 {
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
                let high_ok = self.te_last.abs_diff(TE_SHORT) < TE_DELTA;
                if high_ok && duration.abs_diff(TE_LONG) < TE_DELTA {
                    add_bit(&mut self.decode_data, &mut self.decode_count_bit, false);
                    self.step = Step::SaveDuration;
                } else if high_ok && duration.abs_diff(TE_SHORT * 8) < TE_DELTA {
                    add_bit(&mut self.decode_data, &mut self.decode_count_bit, true);
                    self.step = Step::SaveDuration;
                } else if duration.abs_diff(GAP) < TE_DELTA {
                    // End of the key: the last pulse reads as a 0 bit.
                    add_bit(&mut self.decode_data, &mut self.decode_count_bit, false);
                    let key = (usize::from(self.decode_count_bit) == BIT_COUNT)
                        .then_some(self.decode_data >> 2);
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.step = Step::Reset;
                    return key.filter(|&k| bytesum(k) == k as u8).map(signal);
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

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != BIT_COUNT {
            return None;
        }
        let captured = ((decoded.data >> 8) & 0xF) as u8;
        let btn = if button <= 0xF { button } else { captured };
        Some(with_repeats(&upload(rebuild(decoded.data, btn)), REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        (key.bits == BIT_COUNT).then(|| signal(key.key))
    }

    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        (decoded.data_count_bit == BIT_COUNT)
            .then(|| FlipperKey::for_protocol(NAME, BIT_COUNT, decoded.data))
    }
}

/// Frame builders for the other F modules' tests and the fuzz tool.
#[cfg(test)]
pub(super) mod testing {
    use super::*;

    /// A key with a 28-bit `serial_and_fixed` (bits 39..12) and `btn`, byte sum filled in.
    pub fn frame(serial_and_fixed: u32, btn: u8) -> DecodedSignal {
        signal(rebuild(
            u64::from(serial_and_fixed & 0x0FFF_FFFF) << 12,
            btn & 0xF,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::super::f_common::test_support::*;
    use super::*;

    /// The Flipper unit-test key file (`hollarm.sub`).
    const UNIT_TEST_KEY: &str = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
        Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Hollarm\nBit: 42\nKey: 00 00 00 ED 24 3C 22 6F\n";

    #[test]
    fn fields_of_the_unit_test_key_and_the_c_examples() {
        let d = HollarmDecoder::new()
            .from_flipper_key(&FlipperKey::parse(UNIT_TEST_KEY).unwrap())
            .unwrap();
        assert_eq!((d.data, d.data_count_bit), (0xED_243C_226F, 42));
        assert_eq!((d.serial, d.button), (Some(0xED243C), Some(0x2)));
        assert_eq!(native_buttons(NAME, 0x2), Some("Arm"));
        assert!(d.crc_valid && d.encoder_capable && d.counter.is_none());
        // The C's comment: F0B93422FF / F0B93421FE / F0B9342401 / F0B9342805.
        for (key, btn) in [
            (0xF0_B934_22FFu64, 2u8),
            (0xF0_B934_21FE, 1),
            (0xF0_B934_2401, 4),
            (0xF0_B934_2805, 8),
        ] {
            assert_eq!(bytesum(key), key as u8);
            assert_eq!(rebuild(0xF0_B934_2200, btn), key);
        }
    }

    #[test]
    fn synthetic_frames_decode_for_every_button() {
        for btn in 0..=0xFu8 {
            let d = testing::frame(0x0F0B_9342, btn);
            let mut wire = vec![LevelDuration::new(false, GAP)];
            wire.extend(upload(d.data));
            let got = feed_all(&mut HollarmDecoder::new(), &wire);
            assert_eq!(got, std::slice::from_ref(&d), "btn {btn:#x}");
            assert_eq!((got[0].serial, got[0].button), (Some(0xF0B934), Some(btn)));
        }
    }

    #[test]
    fn held_button_reports_every_other_frame() {
        let d = testing::frame(0x0123_4567, 0x1);
        let mut wire = vec![LevelDuration::new(false, GAP)];
        for _ in 0..4 {
            wire.extend(upload(d.data));
        }
        // Flipper-ARF ends each decode in Reset: frames 1 and 3 (doctor-firmware would report 4).
        assert_eq!(feed_all(&mut HollarmDecoder::new(), &wire).len(), 2);
    }

    #[test]
    fn encoder_switches_buttons_and_rewrites_the_sum() {
        let d = HollarmDecoder::new()
            .from_flipper_key(&FlipperKey::parse(UNIT_TEST_KEY).unwrap())
            .unwrap();
        for (button, want) in [
            (0x1u8, 0x1u8),
            (0x4, 0x4),
            (0x8, 0x8),
            (0x9, 0x9),
            (0x2, 0x2),
            (0x10, 0x2),
        ] {
            let tx = HollarmDecoder::new().encode(&d, button).unwrap();
            assert_eq!(
                tx.len(),
                3 * 2 * BIT_COUNT,
                "alternating levels, nothing merged"
            );
            let mut wire = vec![LevelDuration::new(false, GAP)];
            wire.extend(tx);
            let got = feed_all(&mut HollarmDecoder::new(), &wire);
            assert_eq!(got.len(), 2, "button {button:#x}");
            for f in &got {
                assert_eq!(
                    (f.serial, f.button),
                    (d.serial, Some(want)),
                    "button {button:#x}"
                );
                assert!(f.crc_valid);
            }
        }
    }

    #[test]
    fn upload_shape_matches_the_c() {
        let one = upload(0x80_0000_0000);
        assert_eq!(one.len(), 84);
        assert_eq!((one[0].duration_us, one[1].duration_us), (TE_SHORT, 1600));
        assert_eq!(
            (one[2].duration_us, one[3].duration_us),
            (TE_SHORT, TE_LONG)
        );
        // The two trailing 0 bits: LOW 1000, then the gap.
        assert_eq!((one[81].duration_us, one[83].duration_us), (TE_LONG, GAP));
    }

    #[test]
    fn rejects_bad_sum_wrong_length_and_timing() {
        let d = testing::frame(0x0ABC_DEF1, 0x8);
        let mut wire = vec![LevelDuration::new(false, GAP)];
        wire.extend(upload(d.data));
        assert_eq!(feed_all(&mut HollarmDecoder::new(), &wire).len(), 1);
        let mut bad = vec![LevelDuration::new(false, GAP)];
        bad.extend(upload(d.data ^ 0x100_0000));
        assert!(
            feed_all(&mut HollarmDecoder::new(), &bad).is_empty(),
            "byte sum fails"
        );
        let mut short = wire.clone();
        short.drain(1..3);
        assert!(
            feed_all(&mut HollarmDecoder::new(), &short).is_empty(),
            "41 bits"
        );
        let mut wide = wire.clone();
        wide[1].duration_us = 400;
        assert!(
            feed_all(&mut HollarmDecoder::new(), &wide).is_empty(),
            "HIGH 400 is outside 200 ± 200"
        );
        // Header window (ARF: 2001..=2799 µs), end gap (2201..=2599 µs).
        for (g, ok) in [(2000u32, false), (2001, true), (2799, true), (2800, false)] {
            let mut w = wire.clone();
            w[0].duration_us = g;
            assert_eq!(
                feed_all(&mut HollarmDecoder::new(), &w).len(),
                usize::from(ok),
                "header {g}"
            );
        }
        for (g, ok) in [(2200u32, false), (2201, true), (2599, true), (2600, false)] {
            let mut w = wire.clone();
            w.last_mut().unwrap().duration_us = g;
            assert_eq!(
                feed_all(&mut HollarmDecoder::new(), &w).len(),
                usize::from(ok),
                "end gap {g}"
            );
        }
        assert!(HollarmDecoder::new()
            .encode(&DecodedSignal::new(d.data, 40), 1)
            .is_none());
    }

    #[test]
    fn key_file_round_trip() {
        let dec = HollarmDecoder::new();
        let d = testing::frame(0x0F0B_9342, 0x4);
        let text = super::super::flipper_key::testing::assert_round_trip(&dec, &d);
        assert!(
            text.contains("Protocol: Hollarm\nBit: 42\nKey: 00 00 00 F0 B9 34 24 01\n"),
            "{text}"
        );
        let mut wrong = FlipperKey::parse(&text).unwrap();
        wrong.bits = 40;
        assert!(dec.from_flipper_key(&wrong).is_none());
    }

    /// Flipper's `subghz_decoder_hollarm_test` capture through the whole registry: 12 Hollarm frames,
    /// as Flipper-ARF's decoder (doctor-firmware: 24), all with the unit-test key's serial.
    #[test]
    fn unit_test_capture_decodes() {
        let Some(path) = reference_file(ARF_UNIT_TESTS, "hollarm_raw.sub") else {
            return;
        };
        let decodes = registry_decodes(&path);
        let hollarm: Vec<&DecodedSignal> = decodes
            .iter()
            .filter(|(n, _)| n == NAME)
            .map(|(_, d)| d)
            .collect();
        assert_eq!(hollarm.len(), 12, "{decodes:?}");
        assert!(hollarm.iter().all(|d| d.crc_valid));
        assert_eq!(
            decodes.len(),
            hollarm.len(),
            "another protocol took a frame: {decodes:?}"
        );
    }

    /// `subghz_encoder_hollarm_test`: the unit-test key encodes and decodes back through the registry.
    #[test]
    fn unit_test_key_encodes_back() {
        let reg = super::super::ProtocolRegistry::new();
        let (name, d) = reg
            .decode_key(&FlipperKey::parse(UNIT_TEST_KEY).unwrap())
            .unwrap();
        assert_eq!(name, NAME);
        let tx = reg
            .get(&name)
            .unwrap()
            .encode(&d, d.button.unwrap())
            .unwrap();
        let mut reg = super::super::ProtocolRegistry::new();
        let back = reg.process_signal_stream(&tx, 433_920_000);
        // As in the C's encoder test: copy 1 has no gap before it (its closing gap opens copy 2),
        // and the decode of copy 2 consumes the gap copy 3 would need.
        assert_eq!(back.len(), 1, "{back:?}");
        assert!(
            back.iter().all(|(n, x, _)| n == NAME && x.data == d.data),
            "{back:?}"
        );
    }
}
