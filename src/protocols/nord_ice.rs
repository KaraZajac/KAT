//! Nord ICE: 33-bit fixed code of Nord ICE remotes (four one-hot buttons), decoder + encoder.
//!
//! References (all @xMasterX, 2026.03; `SUBGHZ_PROTOCOL_NORD_ICE_NAME` = "Nord ICE"):
//! - Flipper-ARF's garage app `applications/system/subghz_garage/protocols/nord_ice.c`, registered
//!   there (RX group g12) as a decoder only (its TX code is compiled out: `SUBGHZ_GARAGE_RX_ONLY`):
//!   the first version — te 300 / 800 µs, one gap window 7500 ± 1650 µs, back to Reset after a
//!   frame, the key reported as received. Momentum's, Flipper-ARF `lib`'s (unregistered there) and
//!   FoxFW's decoders are the same; their encoders send that key once per repeat with a 7500 µs gap.
//! - doctor-firmware `lib/subghz/protocols/nord_ice.c` and RogueMaster's (the same code with
//!   comments, "measured off six captures of one remote"): the second version — te 270 / 760 µs, two
//!   gaps 7440 and 8980 µs (± 450), decoding on from the gap that closed a frame, bit 14 (the
//!   "frame flag") cleared, a 4-bit one-hot button, and an encoder that sends the pair the remote
//!   sends with the button switchable. RogueMaster's "Add Manually" creates Nord ICE remotes for it.
//!
//! **The remote** (RogueMaster's notes): a press sends pairs of frames, `[key | 1 << 14] gap 8980
//! [key] gap 7440`, repeated while held; the two frames differ only in bit 14, so the flag is not
//! part of the key.
//!
//! **Union.** KAT decodes what either version decodes and reports doctor's key:
//! - a HIGH/LOW pair is a bit when either version reads it (0 = short + long: 270 ± 150 / 760 ± 150,
//!   or 300 ± 150 / 800 ± 150; 1 = long + short likewise), the last bit's HIGH alone likewise;
//! - a LOW is a gap when either reads it: 5851..=9149 µs (the garage's 7500 ± 1650) or doctor's
//!   8531..=9429 µs;
//! - after the gap that closes a frame KAT keeps decoding (doctor), so every frame of a held button
//!   is reported, not every other one (the garage);
//! - the reported key has bit 14 cleared (doctor): both frames of a pair are one remote. The garage
//!   app shows the flagged frame's bits as they came.
//!
//! Every frame either version reports, KAT reports at the same pulse (the garage's with bit 14
//! cleared): the two versions' bit and gap windows are disjoint from each other's other classes, so
//! both versions classify every pulse they accept as KAT does.
//!
//! **Decoder** (`subghz_protocol_decoder_nord_ice_feed`): Reset waits for a gap; then HIGH/LOW
//! pairs; any pulse that is neither a bit half nor a gap → Reset. A gap closes the frame: the last
//! bit is read from its HIGH, and with exactly 33 bits the key (bit 14 cleared) is reported; the
//! gap then opens the next frame. The C's callback resets the receiver and the code after it
//! (`parser_step = SaveDuration`) overrides that: KAT keeps that state through the registry's
//! reset that follows its own report (`just_emitted`). No checksum.
//!
//! **Fields** (doctor's `subghz_protocol_nord_ice_check_remote_controller`): bits 32..15 and 8..0
//! = the 27-bit serial ((data >> 15) << 9 | data & 0x1FF), bits 14..13 = the frame field (bit 13 is
//! set on every frame), bits 12..9 = the button, one hot (0x4, 0x8, 0x1, 0x2 in the order they sit
//! on the remote). The garage's version shows bits 14..9 as a 6-bit button. KAT: `serial`, `button`
//! as doctor; `counter` None; `crc_valid` false (no check).
//!
//! **Encoder** (doctor's / RogueMaster's `subghz_protocol_encoder_nord_ice_get_upload`,
//! `encoder.repeat` = 3): bit 14 cleared and the button put in bits 12..9, then the flagged frame
//! with the 8980 µs gap and the plain frame with the 7440 µs gap (33 bits each: 1 = HIGH 760 + LOW
//! 270, 0 = HIGH 270 + LOW 760, the last LOW being the gap); 3 copies of the pair. `button` is KAT's
//! native code: 0..=0xF is sent (doctor's signal-settings button editor takes any 4-bit value; its
//! custom-button D-pad walks 0x4 → 0x8 → 0x1 → 0x2 from the captured button), anything else keeps
//! the captured button. The older encoders (garage source, Momentum, ARF `lib`, FoxFW) send one
//! frame with a 7500 µs gap: the pair is what the remote sends. Doctor also writes the sent key back
//! to the file (flag cleared, the sent button), which is the key KAT keeps.
//!
//! **Key file**: `Protocol: Nord ICE`, `Bit: 33`, `Key` (`subghz_block_generic_serialize`;
//! doctor's serializer also writes `Serial` / `Btn` / `Cnt`, which KAT does not); loading requires
//! `Bit: 33`. A key saved with bit 14 set (an older build, or the garage app) loads with it
//! cleared, as doctor's display and encoder clear it.

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
    make: "Nord ICE",
    category: Category::Gate,
    buttons: Some(super::g1_common::numbered_buttons),
    presets: &[&AM650],
};

pub(super) const NAME: &str = "Nord ICE";
const BIT_COUNT: usize = 33;
/// `NORD_ICE_FRAME_FLAG`.
const FRAME_FLAG: u64 = 1 << 14;
/// `NORD_ICE_BTN_SHIFT` / `NORD_ICE_BTN_MASK`.
const BTN_SHIFT: u32 = 9;
const BTN_MASK: u64 = 0xF << BTN_SHIFT;
/// doctor's `NORD_ICE_GAP_FLAGGED` / `NORD_ICE_GAP_PLAIN`: the gaps the encoder closes the two
/// frames with.
const GAP_FLAGGED: u32 = 8980;
const GAP_PLAIN: u32 = 7440;
/// `subghz_protocol_encoder_common_alloc(…, 3, …)`.
const REPEAT: usize = 3;
/// The remote's buttons in the order doctor's custom-button D-pad walks them.
pub(super) const DPAD: [u8; 4] = [0x4, 0x8, 0x1, 0x2];

/// One version's windows: `DURATION_DIFF(d, centre) < delta`.
#[derive(Debug, Clone, Copy)]
struct Windows {
    te_short: u32,
    te_long: u32,
    te_delta: u32,
    /// (centre, delta) of each gap the version accepts.
    gaps: &'static [(u32, u32)],
}

/// doctor-firmware / RogueMaster.
const DOCTOR: Windows = Windows {
    te_short: 270,
    te_long: 760,
    te_delta: 150,
    gaps: &[(GAP_PLAIN, 450), (GAP_FLAGGED, 450)],
};
/// The Flipper-ARF garage app (= Momentum, ARF `lib`, FoxFW): gap te_short·25 ± te_delta·11.
const GARAGE: Windows = Windows {
    te_short: 300,
    te_long: 800,
    te_delta: 150,
    gaps: &[(300 * 25, 150 * 11)],
};

impl Windows {
    fn short(&self, d: u32) -> bool {
        d.abs_diff(self.te_short) < self.te_delta
    }

    fn long(&self, d: u32) -> bool {
        d.abs_diff(self.te_long) < self.te_delta
    }

    fn gap(&self, d: u32) -> bool {
        self.gaps.iter().any(|&(c, delta)| d.abs_diff(c) < delta)
    }
}

/// Which version(s) the decoder runs: KAT's union, or one reference alone (parity tests).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Variant {
    Union,
    #[cfg_attr(not(test), allow(dead_code))]
    Doctor,
    #[cfg_attr(not(test), allow(dead_code))]
    Garage,
}

impl Variant {
    fn windows(self) -> &'static [Windows] {
        match self {
            Variant::Union => &[DOCTOR, GARAGE],
            Variant::Doctor => &[DOCTOR],
            Variant::Garage => &[GARAGE],
        }
    }

    /// doctor: decode on from the closing gap and clear the flag; the garage: neither.
    fn doctor_semantics(self) -> bool {
        self != Variant::Garage
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    SaveDuration,
    CheckDuration,
}

/// Nord ICE decoder (`SubGhzProtocolDecoderNord_Ice`).
pub struct NordIceDecoder {
    variant: Variant,
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
    /// The last `feed` reported a frame: keep the post-callback state through the registry's reset.
    just_emitted: bool,
}

/// A decode (or a loaded key) of `data`, with doctor's serial and button.
fn signal(data: u64) -> DecodedSignal {
    DecodedSignal {
        serial: Some((((data >> 15) << 9) | (data & 0x1FF)) as u32),
        button: Some(((data >> BTN_SHIFT) & 0xF) as u8),
        counter: None,
        crc_valid: false,
        data,
        data_count_bit: BIT_COUNT,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

/// `subghz_protocol_encoder_nord_ice_add_frame`: 33 bits, the last LOW being `gap`.
fn add_frame(out: &mut Vec<LevelDuration>, data: u64, gap: u32) {
    for i in (1..=BIT_COUNT).rev() {
        let last = i == 1;
        if (data >> (i - 1)) & 1 == 1 {
            push(out, true, DOCTOR.te_long);
            push(out, false, if last { gap } else { DOCTOR.te_short });
        } else {
            push(out, true, DOCTOR.te_short);
            push(out, false, if last { gap } else { DOCTOR.te_long });
        }
    }
}

/// The key doctor's encoder sends (and writes back) for `button`.
fn key_for(data: u64, button: u8) -> u64 {
    let data = data & !FRAME_FLAG;
    let captured = ((data >> BTN_SHIFT) & 0xF) as u8;
    let btn = if button <= 0xF { button } else { captured };
    (data & !BTN_MASK) | (u64::from(btn) << BTN_SHIFT)
}

/// One `get_upload`: the flagged frame, then the plain one.
fn upload(key: u64) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(4 * BIT_COUNT);
    add_frame(&mut out, key | FRAME_FLAG, GAP_FLAGGED);
    add_frame(&mut out, key, GAP_PLAIN);
    out
}

impl NordIceDecoder {
    pub fn new() -> Self {
        Self::variant(Variant::Union)
    }

    pub(super) fn variant(variant: Variant) -> Self {
        Self {
            variant,
            step: Step::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
            just_emitted: false,
        }
    }

    fn bit(&self, high: u32, low: u32) -> Option<bool> {
        let w = self.variant.windows();
        if w.iter().any(|w| w.short(high) && w.long(low)) {
            Some(false)
        } else if w.iter().any(|w| w.long(high) && w.short(low)) {
            Some(true)
        } else {
            None
        }
    }

    fn last_bit(&self, high: u32) -> Option<bool> {
        let w = self.variant.windows();
        if w.iter().any(|w| w.short(high)) {
            Some(false)
        } else if w.iter().any(|w| w.long(high)) {
            Some(true)
        } else {
            None
        }
    }

    fn gap(&self, d: u32) -> bool {
        self.variant.windows().iter().any(|w| w.gap(d))
    }
}

impl Default for NordIceDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for NordIceDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: DOCTOR.te_short,
            te_long: DOCTOR.te_long,
            te_delta: DOCTOR.te_delta,
            min_count_bit: BIT_COUNT,
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
                if !level && self.gap(duration) {
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
                if let Some(bit) = self.bit(self.te_last, duration) {
                    add_bit(&mut self.decode_data, &mut self.decode_count_bit, bit);
                    self.step = Step::SaveDuration;
                } else if self.gap(duration) {
                    // End of the key: the last bit is read from its HIGH alone.
                    if let Some(bit) = self.last_bit(self.te_last) {
                        add_bit(&mut self.decode_data, &mut self.decode_count_bit, bit);
                    }
                    let doctor = self.variant.doctor_semantics();
                    let out = (usize::from(self.decode_count_bit) == BIT_COUNT).then(|| {
                        signal(if doctor { self.decode_data & !FRAME_FLAG } else { self.decode_data })
                    });
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    // doctor: the gap opens the next frame (over the callback's reset); the garage:
                    // Reset.
                    self.step = if doctor { Step::SaveDuration } else { Step::Reset };
                    self.just_emitted = doctor && out.is_some();
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

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        (decoded.data_count_bit == BIT_COUNT)
            .then(|| with_repeats(&upload(key_for(decoded.data, button)), REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        (key.bits == BIT_COUNT).then(|| signal(key.key & !FRAME_FLAG))
    }

    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        (decoded.data_count_bit == BIT_COUNT)
            .then(|| FlipperKey::for_protocol(NAME, BIT_COUNT, decoded.data))
    }
}

#[cfg(test)]
mod tests {
    use super::super::f_common::test_support::*;
    use super::*;

    /// (level, µs) of each pulse (`LevelDuration` has no `PartialEq`).
    fn levels(wire: &[LevelDuration]) -> Vec<(bool, u32)> {
        wire.iter().map(|p| (p.level, p.duration_us)).collect()
    }

    /// RogueMaster's / doctor's key samples: one remote, buttons 1..4 (the plain frames).
    const SAMPLES: [(u64, u8); 4] = [(0x9467_288A, 0x4), (0x9467_308A, 0x8), (0x9467_228A, 0x1), (0x9467_248A, 0x2)];

    /// `pairs` pairs of a held button, after the previous pair's closing gap.
    fn press(key: u64, pairs: usize) -> Vec<LevelDuration> {
        let mut wire = vec![LevelDuration::new(false, GAP_PLAIN)];
        for p in with_repeats(&upload(key), pairs) {
            push(&mut wire, p.level, p.duration_us);
        }
        wire
    }

    /// The garage version's single frame (its encoder: te 300 / 800, gap 7500).
    fn garage_frame(data: u64) -> Vec<LevelDuration> {
        let mut out = Vec::new();
        for i in (1..=BIT_COUNT).rev() {
            let one = (data >> (i - 1)) & 1 == 1;
            push(&mut out, true, if one { 800 } else { 300 });
            push(&mut out, false, if i == 1 { 7500 } else if one { 300 } else { 800 });
        }
        out
    }

    #[test]
    fn fields_of_the_c_samples() {
        for (key, btn) in SAMPLES {
            let d = signal(key);
            // doctor's oracle on 0x9467288A: "Serial: 0x2519C8A / Btn: 4".
            assert_eq!((d.serial, d.button), (Some(0x2519C8A), Some(btn)), "{key:08X}");
            assert!(!d.crc_valid && d.counter.is_none() && d.encoder_capable);
            assert_eq!(key & FRAME_FLAG, 0);
            assert_ne!(key & (1 << 13), 0, "bit 13 is set on every frame");
        }
    }

    /// doctor's oracle on its own upload of 0x9467288A: 5 frames (the first flagged frame has no gap
    /// before it), the garage's 3 (every other frame, the plain ones).
    #[test]
    fn every_frame_of_a_press_decodes() {
        let tx = NordIceDecoder::new().encode(&signal(0x9467_288A), 0x4).unwrap();
        let union = feed_resetting(&mut NordIceDecoder::new(), &tx);
        let doctor = feed_resetting(&mut NordIceDecoder::variant(Variant::Doctor), &tx);
        let garage = feed_resetting(&mut NordIceDecoder::variant(Variant::Garage), &tx);
        assert_eq!((union.len(), doctor.len(), garage.len()), (5, 5, 3));
        assert!(union.iter().chain(&doctor).chain(&garage).all(|d| d.data == 0x9467_288A));
        // After the previous pair's gap: all 6 frames.
        assert_eq!(feed_all(&mut NordIceDecoder::new(), &press(0x9467_288A, 3)).len(), 6);
    }

    #[test]
    fn upload_matches_doctor() {
        // doctor's encoder on 0x9467288A: 396 pulses (3 × 2 × 66), the flagged frame 0x9467688A first.
        let up = NordIceDecoder::new().encode(&signal(0x9467_288A), 0x4).unwrap();
        assert_eq!(up.len(), 396);
        let p = |i: usize| (up[i].level, up[i].duration_us);
        // 0x9467688A as 33 bits: 0 1 0 0 1 …
        assert_eq!([p(0), p(1), p(2), p(3)], [(true, 270), (false, 760), (true, 760), (false, 270)]);
        assert_eq!([p(64), p(65), p(130), p(131)], [(true, 270), (false, 8980), (true, 270), (false, 7440)]);
        let mut flagged = Vec::new();
        add_frame(&mut flagged, 0x9467_688A, GAP_FLAGGED);
        assert_eq!(levels(&up[..66]), levels(&flagged));
    }

    #[test]
    fn encoder_switches_buttons() {
        let d = signal(0x9467_288A);
        for (button, want) in [(0x8u8, 0x8u8), (0x1, 0x1), (0x2, 0x2), (0x4, 0x4), (0x3, 0x3), (0x10, 0x4)] {
            let tx = NordIceDecoder::new().encode(&d, button).unwrap();
            let got = feed_resetting(&mut NordIceDecoder::new(), &tx);
            assert_eq!(got.len(), 5, "button {button:#x}");
            assert!(got.iter().all(|x| x.button == Some(want) && x.serial == d.serial), "button {button:#x}");
            // What the app reads back after sending (and doctor writes back to the file).
            assert_eq!(got[0], signal(key_for(d.data, button)));
        }
        // The flag of a stored key never reaches the plain frame.
        let tx = NordIceDecoder::new().encode(&signal(0x9467_688A), 0x10).unwrap();
        assert_eq!(levels(&tx), levels(&NordIceDecoder::new().encode(&d, 0x10).unwrap()));
    }

    #[test]
    fn union_reads_the_garage_version_and_clears_the_flag() {
        // The garage encoder's single frames (here the flagged one), back to back after a gap.
        let mut wire = vec![LevelDuration::new(false, 7500)];
        for _ in 0..3 {
            for p in garage_frame(0x9467_688A) {
                push(&mut wire, p.level, p.duration_us);
            }
        }
        let garage = feed_resetting(&mut NordIceDecoder::variant(Variant::Garage), &wire);
        assert_eq!(garage.len(), 2, "the garage reads frames 1 and 3, as sent");
        assert!(garage.iter().all(|d| d.data == 0x9467_688A));
        let union = feed_resetting(&mut NordIceDecoder::new(), &wire);
        assert_eq!(union.len(), 3, "KAT reads every frame");
        assert!(union.iter().all(|d| d.data == 0x9467_288A && d.button == Some(0x4)));
        // doctor's windows alone: 300 is short and 800 long for it too, but its gaps (6991..=7889,
        // 8531..=9429 µs) do not take 7500 ± 1650 at its edges.
        let mut edge = wire.clone();
        for p in edge.iter_mut().filter(|p| p.duration_us == 7500) {
            p.duration_us = 6000;
        }
        assert!(feed_resetting(&mut NordIceDecoder::variant(Variant::Doctor), &edge).is_empty());
        assert_eq!(feed_resetting(&mut NordIceDecoder::new(), &edge).len(), 3);
    }

    #[test]
    fn windows_of_the_union() {
        let one = |high: u32, low: u32| NordIceDecoder::new().bit(high, low);
        assert_eq!((one(121, 760), one(449, 760), one(120, 760), one(450, 760)), (Some(false), Some(false), None, None));
        assert_eq!((one(270, 611), one(270, 949), one(270, 610), one(270, 950)), (Some(false), Some(false), None, None));
        assert_eq!((one(611, 270), one(949, 449), one(949, 450)), (Some(true), Some(true), None));
        // A pair only one version reads as a whole: 130 is short for doctor only, 940 long for the
        // garage only.
        assert_eq!(one(130, 940), None);
        let gap = |d: u32| NordIceDecoder::new().gap(d);
        assert_eq!((gap(5850), gap(5851), gap(9149), gap(9150), gap(9429), gap(9430)), (false, true, true, true, true, false));
    }

    #[test]
    fn rejects_wrong_length_and_timing() {
        let wire = press(0x9467_248A, 1);
        assert_eq!(feed_all(&mut NordIceDecoder::new(), &wire).len(), 2);
        let mut short = wire.clone();
        short.drain(1..3);
        assert_eq!(feed_all(&mut NordIceDecoder::new(), &short).len(), 1, "32 bits, then the plain frame");
        let mut wide = wire.clone();
        wide[1].duration_us = 1000;
        assert_eq!(feed_all(&mut NordIceDecoder::new(), &wide).len(), 1, "HIGH 1000 is no bit");
        assert!(NordIceDecoder::new().encode(&DecodedSignal::new(0x9467_248A, 32), 1).is_none());
    }

    #[test]
    fn key_file_round_trip() {
        let dec = NordIceDecoder::new();
        let d = signal(0x9467_228A);
        let text = super::super::flipper_key::testing::assert_round_trip(&dec, &d);
        assert!(text.contains("Protocol: Nord ICE\nBit: 33\nKey: 00 00 00 00 94 67 22 8A\n"), "{text}");
        // A key saved with the flag set loads as the plain key.
        let flagged = FlipperKey::parse(&text.replace("94 67 22 8A", "94 67 62 8A")).unwrap();
        assert_eq!(dec.from_flipper_key(&flagged).unwrap(), d);
        let mut wrong = FlipperKey::parse(&text).unwrap();
        wrong.bits = 32;
        assert!(dec.from_flipper_key(&wrong).is_none());
    }

    /// RogueMaster's "Add Manually" Nord ICE key ((rand() & 0x1FFFF81FF) | 0x2400: button 0x2, bit 13
    /// set, bit 14 clear) loads, sends every button and decodes back through the whole registry.
    #[test]
    fn add_manually_key_loads_and_transmits() {
        let reg = super::super::ProtocolRegistry::new();
        for rand in [0x7FFF_FFFFu64, 0, 0x1234_5678, 0x0BAD_F00D] {
            let key = FlipperKey::for_protocol(NAME, 33, (rand & 0x1_FFFF_81FF) | 0x2400);
            let (name, d) = reg.decode_key(&key).expect("loads");
            assert_eq!((name.as_str(), d.button), (NAME, Some(0x2)));
            for &button in &DPAD {
                let tx = reg.get(&name).unwrap().encode(&d, button).unwrap();
                let mut stream = vec![LevelDuration::new(false, 20_000)];
                for p in tx {
                    push(&mut stream, p.level, p.duration_us);
                }
                push(&mut stream, false, 20_000);
                let mut r = super::super::ProtocolRegistry::new();
                let hits = r.process_signal_stream(&stream, 433_920_000);
                // 4 of the 6 frames: the first has no gap before it, the last one's gap runs into the
                // silence after it.
                assert_eq!(hits.len(), 4, "{rand:#x} {button:#x}: {hits:?}");
                assert!(hits.iter().all(|(n, x, _)| n == NAME && x.serial == d.serial && x.button == Some(button)));
            }
        }
    }

    #[test]
    fn jittered_presses_decode() {
        let mut rng = Rng(0x40D1_CE33);
        for _ in 0..50 {
            let key = key_for(rng.next() & 0x1_FFFF_FFFF, DPAD[rng.below(4) as usize]);
            let wire = jitter(&press(key, 2), &mut rng, 100);
            let got = feed_all(&mut NordIceDecoder::new(), &wire);
            assert!(got.len() >= 3, "{key:09X}");
            assert!(got.iter().all(|d| d.data == key));
        }
    }
}

#[cfg(test)]
mod parity {
    //! Oracle-parity dump for this round's home protocols (Honeywell Sec, Honeywell, Treadmill37,
    //! Nord ICE), `#[ignore]`d; see their docs, "Validation". `HB_FILES=<list>` (one RAW `.sub`
    //! path per line): each decoder alone over each file, reset after each decode like the C oracle's
    //! `decode --protocol NAME`, one line per decode: `<file> | <decoder> | Protocol: …; Bit: …;
    //! Key: …` plus doctor's optional `Serial` / `Btn` / `Cnt` (written when one is non-zero). The
    //! decoders: the four registered ones and each reference version alone ("Honeywell Sec/doctor",
    //! "Honeywell Sec/foxfw", "Nord ICE/doctor", "Nord ICE/garage"); `HB_PROTO=<decoder>` keeps one.
    use super::super::f_common::test_support::{feed_resetting, sub_pulses};
    use super::super::{honeywell, honeywell_wdb, treadmill37, ProtocolDecoder};
    use super::{NordIceDecoder, Variant};

    const DECODERS: [&str; 8] = [
        "Honeywell Sec",
        "Honeywell Sec/doctor",
        "Honeywell Sec/foxfw",
        "Honeywell",
        "Treadmill37",
        "Nord ICE",
        "Nord ICE/doctor",
        "Nord ICE/garage",
    ];

    fn decoder(name: &str) -> Box<dyn ProtocolDecoder> {
        match name {
            "Honeywell Sec" => Box::new(honeywell::HoneywellDecoder::new()),
            "Honeywell Sec/doctor" => Box::new(honeywell::HoneywellDecoder::single(false)),
            "Honeywell Sec/foxfw" => Box::new(honeywell::HoneywellDecoder::single(true)),
            "Honeywell" => Box::new(honeywell_wdb::HoneywellWdbDecoder::new()),
            "Treadmill37" => Box::new(treadmill37::Treadmill37Decoder::new()),
            "Nord ICE" => Box::new(NordIceDecoder::new()),
            "Nord ICE/doctor" => Box::new(NordIceDecoder::variant(Variant::Doctor)),
            "Nord ICE/garage" => Box::new(NordIceDecoder::variant(Variant::Garage)),
            _ => unreachable!("{name}"),
        }
    }

    #[test]
    #[ignore]
    fn hb_parity_decode_dump() {
        let Ok(list) = std::env::var("HB_FILES") else {
            return;
        };
        let only = std::env::var("HB_PROTO").ok();
        let names: Vec<&str> = DECODERS.iter().copied().filter(|n| only.as_deref().is_none_or(|o| o == *n)).collect();
        for file in std::fs::read_to_string(list).unwrap().lines().filter(|l| !l.trim().is_empty()) {
            let (_, wire) = sub_pulses(std::path::Path::new(file));
            for &name in &names {
                let mut dec = decoder(name);
                for d in feed_resetting(dec.as_mut(), &wire) {
                    let key = dec.to_flipper_key(&d).expect("key file");
                    let bytes: Vec<String> = key.key.to_be_bytes().iter().map(|b| format!("{b:02X}")).collect();
                    let mut line = format!("{file} | {name} | Protocol: {}; Bit: {}; Key: {}", key.protocol, key.bits, bytes.join(" "));
                    let (sn, btn, cnt) = (d.serial.unwrap_or(0), d.button.unwrap_or(0), d.counter.unwrap_or(0));
                    if sn != 0 || btn != 0 || cnt != 0 {
                        line += &format!("; Serial: {sn}; Btn: {btn}; Cnt: {cnt}");
                    }
                    println!("{line}");
                }
            }
        }
    }
}
