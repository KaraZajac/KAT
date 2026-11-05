//! GangQi 34-bit fixed code (GangQi / G-series GSM alarm remotes).
//!
//! Reference: doctor-firmware `lib/subghz/protocols/gangqi.c` (@xMasterX, Unleashed lineage).
//! Flipper-ARF's `lib/subghz/protocols/gangqi.c`, its garage-app copy
//! `applications/system/subghz_garage/protocols/gangqi.c` (registered in RX group g10 with a TX
//! plugin), Momentum's and RogueMaster's copies decode and encode identically; they differ only in
//! the display text and the custom-button bookkeeping (the garage plugin builds compile the
//! custom-button functions as stubs, so the garage TX plugin always sends button 0 — the build
//! artefact the gate round did not copy; KAT sends the captured or requested button, as doctor).
//!
//! **Air format** (AM, 433.92 MHz): PWM, te_short = 500 µs, te_long = 1200 µs, te_delta = 200 µs.
//! Each bit is a HIGH then a LOW: 1 = HIGH 1200 + LOW 500, 0 = HIGH 500 + LOW 1200; the last bit's
//! LOW is the inter-frame gap (the encoder sends 4·te_short + te_delta = 2200 µs). 34 bits, MSB
//! first, no preamble.
//!
//! **Decoder** (`subghz_protocol_decoder_gangqi_feed`): Reset waits for a LOW with
//! |d − 2·te_long| < 3·te_delta (1801–2999 µs); bits are HIGH/LOW pairs within te_delta. A
//! gap-length LOW closes the frame: the last bit is read from its HIGH alone with two independent
//! tests (≈500 → 0, then ≈1200 → 1), and the frame is reported when exactly 34 bits were read. The
//! decoder then returns to Reset, so the gap that closed a frame does not open the next one: only
//! every other frame of a held button is reported (as in the C). The C checks nothing else.
//!
//! **Frame** (`subghz_protocol_gangqi_remote_controller` + the generation code
//! `subghz_txrx_gen_serial_gangqi`): bits 33..18 = 16-bit serial, bits 17..14 = constant 0xD,
//! bits 13..10 = button, bits 9..2 = byte sum, bits 1..0 = 0. The C reports `serial` =
//! (data & 0xFFFFF0000) >> 16 (the 16-bit serial and the top two bits of the constant) and
//! `btn` = (data >> 10) & 0xF. Byte sum: "type 1" (original remotes) =
//! 0xC8 − serial_high − serial_low − (0xD0 | btn); "type 2" (a "backdoor" sum receivers accept)
//! = 0x02 + serial_high + serial_low + (0xD0 | btn). The C only displays both sums.
//!
//! **KAT fields**: `serial` and `button` as the C; `counter` None; `data` = the 34-bit frame;
//! `crc_valid` = the frame's bits 9..2 equal the type-1 or type-2 sum of its serial and button.
//! KAT reports only frames whose sum is valid (the C's displayed sums, KAT's checksum policy for
//! gate remotes; every frame of the unit-test capture passes).
//!
//! **Encoder** (`subghz_protocol_encoder_gangqi_get_upload`, `encoder.repeat` = 3): the key is
//! rebuilt from the frame's serial and the button to send, as the C does: data =
//! (data >> 14) << 14 | btn << 10 | sum_type1 << 2 (so a type-2 frame is sent with the type-1
//! sum, and bits 1..0 are cleared), then 34 bit pairs with the 2200 µs gap after the last bit, 3
//! times. `button` is KAT's native code: 0..=0xF is sent (the references' button editor takes any
//! 4-bit value; their D-pad cycles 0xD Arm / 0xE Disarm / 0xB Alarm / 0x7 Ring), anything else
//! keeps the captured button. Static code: no counter.
//!
//! **Key file**: `Protocol: GangQi`, `Bit: 34`, `Key` (`subghz_block_generic_serialize`); loading
//! requires `Bit: 34` (`subghz_block_generic_deserialize_check_count_bit`).

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
    make: "GangQi",
    category: Category::Gate,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

const NAME: &str = "GangQi";
const TE_SHORT: u32 = 500;
const TE_LONG: u32 = 1200;
const TE_DELTA: u32 = 200;
const BIT_COUNT: usize = 34;
/// `subghz_protocol_encoder_gangqi_alloc`: repeat = 3.
const REPEAT: usize = 3;
/// The gap after the last bit: te_short·4 + te_delta.
const GAP: u32 = TE_SHORT * 4 + TE_DELTA;
/// The buttons of the remote (the C's custom-button D-pad cycles these four).
pub(super) const DPAD: [u8; 4] = [0xD, 0xE, 0xB, 0x7];

/// `subghz_protocol_gangqi_get_button_name`.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    const NAMES: [&str; 16] = [
        "Unknown",
        "Exit settings",
        "Volume setting",
        "0x3",
        "Vibro sens. setting",
        "Settings mode",
        "Ringtone setting",
        "Ring",
        "0x8",
        "0x9",
        "0xA",
        "Alarm",
        "0xC",
        "Arm",
        "Disarm",
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

/// GangQi decoder (`SubGhzProtocolDecoderGangQi`).
pub struct GangQiDecoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
}

/// (type-1, type-2) byte sums of a frame's serial and button.
fn sums(data: u64) -> (u8, u8) {
    let serial = ((data >> 18) & 0xFFFF) as u16;
    let [high, low] = serial.to_be_bytes();
    let const_and_button = 0xD0 | ((data >> 10) & 0xF) as u8;
    let type1 = 0xC8u8
        .wrapping_sub(high)
        .wrapping_sub(low)
        .wrapping_sub(const_and_button);
    let type2 = 0x02u8
        .wrapping_add(high)
        .wrapping_add(low)
        .wrapping_add(const_and_button);
    (type1, type2)
}

/// The frame's byte sum is one of the two the C displays.
fn sum_ok(data: u64) -> bool {
    let field = ((data >> 2) & 0xFF) as u8;
    let (type1, type2) = sums(data);
    field == type1 || field == type2
}

fn signal(data: u64) -> DecodedSignal {
    DecodedSignal {
        serial: Some(((data & 0xF_FFFF_0000) >> 16) as u32),
        button: Some(((data >> 10) & 0xF) as u8),
        counter: None,
        crc_valid: sum_ok(data),
        data,
        data_count_bit: BIT_COUNT,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

/// The key the encoder sends for `btn` (`subghz_protocol_encoder_gangqi_get_upload`).
fn rebuild(data: u64, btn: u8) -> u64 {
    let with_btn = ((data >> 14) << 14) | (u64::from(btn) << 10);
    let (type1, _) = sums(with_btn);
    with_btn | (u64::from(type1) << 2)
}

/// One upload: 34 bit pairs, the gap after the last.
fn upload(data: u64) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(2 * BIT_COUNT);
    for i in (1..=BIT_COUNT).rev() {
        let one = (data >> (i - 1)) & 1 == 1;
        push(&mut out, true, if one { TE_LONG } else { TE_SHORT });
        push(
            &mut out,
            false,
            if i == 1 {
                GAP
            } else if one {
                TE_SHORT
            } else {
                TE_LONG
            },
        );
    }
    out
}

impl GangQiDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
        }
    }
}

impl Default for GangQiDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for GangQiDecoder {
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
        let gap = |d: u32| d.abs_diff(TE_LONG * 2) < TE_DELTA * 3;
        match self.step {
            Step::Reset => {
                if !level && gap(duration) {
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
                let short = |d: u32| d.abs_diff(TE_SHORT) < TE_DELTA;
                let long = |d: u32| d.abs_diff(TE_LONG) < TE_DELTA;
                if short(self.te_last) && long(duration) {
                    add_bit(&mut self.decode_data, &mut self.decode_count_bit, false);
                    self.step = Step::SaveDuration;
                } else if long(self.te_last) && short(duration) {
                    add_bit(&mut self.decode_data, &mut self.decode_count_bit, true);
                    self.step = Step::SaveDuration;
                } else if gap(duration) {
                    // Two independent tests, as the C.
                    if short(self.te_last) {
                        add_bit(&mut self.decode_data, &mut self.decode_count_bit, false);
                    }
                    if long(self.te_last) {
                        add_bit(&mut self.decode_data, &mut self.decode_count_bit, true);
                    }
                    let frame = (usize::from(self.decode_count_bit) == BIT_COUNT)
                        .then_some(self.decode_data);
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.step = Step::Reset;
                    // KAT: only frames whose byte sum checks (see the module doc).
                    return frame.filter(|&d| sum_ok(d)).map(signal);
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
        let captured = ((decoded.data >> 10) & 0xF) as u8;
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

    /// A frame as the generation code builds one (type-1 sum) for `serial` and `btn`.
    pub fn frame(serial: u16, btn: u8) -> DecodedSignal {
        signal(rebuild((u64::from(serial) << 18) | (0xD << 14), btn & 0xF))
    }

    /// One copy of the C's upload layout for any 34-bit `data` (sum not rebuilt).
    pub fn raw_upload(data: u64) -> Vec<LevelDuration> {
        upload(data)
    }
}

#[cfg(test)]
mod tests {
    use super::super::f_common::test_support::*;
    use super::*;

    /// The Flipper unit-test key file (`gangqi.sub`).
    const UNIT_TEST_KEY: &str = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
        Preset: FuriHalSubGhzPresetOok650Async\nProtocol: GangQi\nBit: 34\nKey: 00 00 00 02 9D DB 77 38\n";

    #[test]
    fn fields_and_sums_of_the_unit_test_key() {
        let d = GangQiDecoder::new()
            .from_flipper_key(&FlipperKey::parse(UNIT_TEST_KEY).unwrap())
            .unwrap();
        assert_eq!((d.data, d.data_count_bit), (0x2_9DDB_7738, 34));
        // The C's get_string: "Serial: 0x29DDB", "Btn: 0xD - Arm", Sum (type 1) = the frame's.
        assert_eq!((d.serial, d.button), (Some(0x29DDB), Some(0xD)));
        assert_eq!(native_buttons(NAME, 0xD), Some("Arm"));
        assert_eq!(sums(d.data).0, ((d.data >> 2) & 0xFF) as u8);
        assert!(d.crc_valid && d.encoder_capable && d.counter.is_none());
        // The encoder re-sends this key unchanged (type-1 sum, low bits clear).
        assert_eq!(rebuild(d.data, 0xD), d.data);
    }

    #[test]
    fn synthetic_frames_decode_for_every_button() {
        for btn in 0..=0xFu8 {
            let d = testing::frame(0xA5C3, btn);
            assert!(d.crc_valid, "btn {btn:#x}");
            let mut wire = vec![LevelDuration::new(false, GAP)];
            wire.extend(upload(d.data));
            let got = feed_all(&mut GangQiDecoder::new(), &wire);
            assert_eq!(got, std::slice::from_ref(&d), "btn {btn:#x}");
            assert_eq!(got[0].button, Some(btn));
            assert_eq!(got[0].serial, Some((0xA5C3 << 2) | 0x3));
        }
        // A type-2 ("backdoor") sum decodes too.
        let d = testing::frame(0x1234, 0xE);
        let type2 = (d.data & !0x3FC) | (u64::from(sums(d.data).1) << 2);
        let mut wire = vec![LevelDuration::new(false, GAP)];
        wire.extend(upload(type2));
        let got = feed_all(&mut GangQiDecoder::new(), &wire);
        assert_eq!(got.len(), 1);
        assert!(got[0].crc_valid);
    }

    #[test]
    fn held_button_reports_every_other_frame() {
        let d = testing::frame(0x0F0F, 0x7);
        let mut wire = vec![LevelDuration::new(false, GAP)];
        wire.extend(GangQiDecoder::new().encode(&d, 0x7).unwrap());
        // 3 copies; the gap closing copy 1 does not open copy 2 (C behaviour), copy 3 has a gap
        // before it (the one closing copy 2).
        assert_eq!(feed_all(&mut GangQiDecoder::new(), &wire).len(), 2);
    }

    #[test]
    fn encoder_switches_buttons_and_rewrites_the_sum() {
        let d = testing::frame(0xBEEF, 0xD);
        for (button, want) in [
            (0xEu8, 0xEu8),
            (0xB, 0xB),
            (0x7, 0x7),
            (0x5, 0x5),
            (0xD, 0xD),
            (0x10, 0xD),
            (0xFF, 0xD),
        ] {
            let tx = GangQiDecoder::new().encode(&d, button).unwrap();
            assert_eq!(
                tx.len(),
                3 * 2 * BIT_COUNT,
                "alternating levels, nothing merged"
            );
            let mut wire = vec![LevelDuration::new(false, GAP)];
            wire.extend(tx);
            let got = feed_all(&mut GangQiDecoder::new(), &wire);
            assert!(!got.is_empty(), "button {button:#x}");
            for f in &got {
                assert_eq!(
                    (f.serial, f.button),
                    (d.serial, Some(want)),
                    "button {button:#x}"
                );
                assert!(f.crc_valid);
            }
        }
        // A type-2 frame is sent with the type-1 sum; the two low bits are cleared.
        let type2 = (d.data & !0x3FF) | (u64::from(sums(d.data).1) << 2) | 0x3;
        assert_eq!(rebuild(type2, 0xD), d.data);
    }

    #[test]
    fn upload_shape_matches_the_c() {
        let d = testing::frame(0x8001, 0xD);
        let one = upload(d.data);
        assert_eq!(one.len(), 68);
        // MSB first: bit 33 = 1 (serial 0x8001).
        assert_eq!(
            (one[0].level, one[0].duration_us, one[1].duration_us),
            (true, TE_LONG, TE_SHORT)
        );
        // The last bit's LOW is the 2200 µs gap (bits 1..0 are 0: HIGH 500).
        assert_eq!((one[66].duration_us, one[67].duration_us), (TE_SHORT, 2200));
    }

    #[test]
    fn rejects_wrong_length_timing_and_sum() {
        let d = testing::frame(0x4242, 0xE);
        let mut wire = vec![LevelDuration::new(false, GAP)];
        wire.extend(upload(d.data));
        assert_eq!(feed_all(&mut GangQiDecoder::new(), &wire).len(), 1);
        // A bad sum (the C would report it; KAT does not).
        let mut bad = vec![LevelDuration::new(false, GAP)];
        bad.extend(upload(d.data ^ 0x4));
        assert!(feed_all(&mut GangQiDecoder::new(), &bad).is_empty());
        // One bit short.
        let mut short = wire.clone();
        short.drain(1..3);
        assert!(feed_all(&mut GangQiDecoder::new(), &short).is_empty());
        // A stretched HIGH (1400 µs is outside 1200 ± 200, exclusive).
        let mut stretched = wire.clone();
        stretched[5].duration_us = 1400;
        assert!(feed_all(&mut GangQiDecoder::new(), &stretched).is_empty());
        // Gap window: 1801..=2999 µs opens a frame, 1800 and 3000 do not.
        for (g, ok) in [(1800u32, false), (1801, true), (2999, true), (3000, false)] {
            let mut w = wire.clone();
            w[0].duration_us = g;
            assert_eq!(
                feed_all(&mut GangQiDecoder::new(), &w).len(),
                usize::from(ok),
                "gap {g}"
            );
        }
        assert!(GangQiDecoder::new()
            .encode(&DecodedSignal::new(d.data, 33), 1)
            .is_none());
    }

    #[test]
    fn key_file_round_trip() {
        let dec = GangQiDecoder::new();
        let d = testing::frame(0x3C3C, 0xB);
        let text = super::super::flipper_key::testing::assert_round_trip(&dec, &d);
        assert!(
            text.contains("Protocol: GangQi\nBit: 34\nKey: 00 00 00 00 F0 F3 6D D4\n"),
            "{text}"
        );
        let mut wrong = FlipperKey::parse(&text).unwrap();
        wrong.bits = 35;
        assert!(dec.from_flipper_key(&wrong).is_none());
    }

    /// Flipper's `subghz_decoder_gangqi_test` capture through the whole registry: every frame is
    /// GangQi (doctor/ARF/Momentum/RogueMaster report 25), and the unit-test key's code is among them.
    #[test]
    fn unit_test_capture_decodes() {
        let Some(path) = reference_file(ARF_UNIT_TESTS, "gangqi_raw.sub") else {
            return;
        };
        let decodes = registry_decodes(&path);
        let gangqi: Vec<&DecodedSignal> = decodes
            .iter()
            .filter(|(n, _)| n == NAME)
            .map(|(_, d)| d)
            .collect();
        assert_eq!(gangqi.len(), 25, "{decodes:?}");
        assert!(gangqi.iter().all(|d| d.crc_valid));
        assert_eq!(
            decodes.len(),
            gangqi.len(),
            "another protocol took a frame: {decodes:?}"
        );
    }

    /// `subghz_encoder_gangqi_test`: the unit-test key encodes and decodes back through the registry.
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
        // The bare upload, as `protocols::tests::flipper_unit_test_encoders` feeds it: copy 1 has
        // no gap before it (its closing gap opens copy 2), the decode of copy 2 consumes copy 3's.
        let mut reg = super::super::ProtocolRegistry::new();
        let back = reg.process_signal_stream(&tx, 433_920_000);
        assert_eq!(back.len(), 1, "{back:?}");
        assert!(
            back.iter().all(|(n, x, _)| n == NAME && x.data == d.data),
            "{back:?}"
        );
    }
}
