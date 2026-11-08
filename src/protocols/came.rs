//! CAME decoder + encoder: 12/24-bit fixed codes, plus the 18-bit "Airforce" and 25/42-bit
//! "Prastel" frames the same decoder reads.
//!
//! Port of Flipper-ARF's garage app `applications/system/subghz_garage/protocols/came.c` (the
//! primary reference; decodes like ARF's lib copy). doctor-firmware's copy differs on the wire:
//! it no longer decodes 42-bit frames (moved to its `prastel.c`) and only reports frames passing
//! `subghz_protocol_came_payload_ok` (no degenerate key; a 12-bit frame needs a ≥ 13120 µs header
//! and fewer than two 0x5/0xA nibbles). KAT follows ARF: every 12/18/24/25/42-bit frame.
//!
//! **Air format** (AM, 315/433.92 MHz): `te_short` 320, `te_long` 640, `te_delta` 150. Header LOW
//! (24 te for 24/42 bits = 24320 µs, 47 te for 12/18 bits, 36 te for 25 bits), start HIGH te, then
//! MSB first: bit 1 = LOW 640 + HIGH 320, bit 0 = LOW 320 + HIGH 640.
//!
//! **Decoder** (`subghz_protocol_decoder_came_feed`): header LOW within 17920 ± 9450 µs, start HIGH
//! |d − 320| < 150, then LOW/HIGH pairs; a LOW ≥ 1280 µs ends the frame, which is reported when it
//! holds exactly 12, 18, 24, 25 or 42 bits. No field split (`serial`/`button` stay empty, as the C's
//! display); the C names 18-bit frames "Airforce" and 25/42-bit frames "Prastel" — KAT shows them as
//! "CAME [Airforce]" / "CAME [Prastel]" (the key file still says `Protocol: CAME`). In Flipper-ARF's
//! garage app CAME is registered before its 42-bit "Prastel" protocol and wins frames both decode.
//!
//! **Faac TM frames (RogueMaster, Nov 2026).** A Faac TM remote (`faac_tm.rs`: 12 DIP bits in the
//! same short/long grammar, Te measured per frame, nominally 400 µs) running fast fits CAME's
//! windows up to Te ≈ 395 µs. RogueMaster's `came.c` tells them apart by the gap after the frame,
//! counted in the frame's own Te (gap and Te come from one oscillator, so the ratio carries no clock
//! error): CAME repeats every 47 Te, Faac TM every 35. A 12-bit frame whose mean Te (the start bit
//! and every data pulse summed, / 37) is 340 µs or more — slower than any CAME remote, 320 µs
//! nominal — is reported only when the gap that ends it lies within 42..=52 Te; otherwise it is left
//! to Faac TM (registered after CAME, as in RogueMaster). The last frame of a slow press, followed by
//! silence, is therefore not CAME's either. Frames under 340 µs and the 18/24/25/42-bit lengths are
//! unaffected. KAT adds this to Flipper-ARF's decoder (the union of what the references read).
//!
//! **Encoder** (`subghz_protocol_encoder_came_get_upload`, `encoder.repeat` = 3): header, start bit,
//! the key's bits; keys up to 42 bits. CAME has no button field: `button` is ignored (ARF's lib
//! encoder also re-sends the captured code for every D-pad direction).
//!
//! **Key file**: `Protocol: CAME`, `Bit`, `Key` (Flipper-ARF's generic serialize); loading rejects
//! more than 42 bits, as the C's `deserialize`.

use super::g1_common::{fixed_code, generic_fields, generic_key, push, with_repeats, TX_REPEAT};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "CAME",
    decoder: "CAME",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "CAME",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "CAME";
const TE_SHORT: u32 = 320;
const TE_LONG: u32 = 640;
const TE_DELTA: u32 = 150;
const MIN_COUNT_BIT: usize = 12;
const CAME_24_COUNT_BIT: usize = 24;
const PRASTEL_25_COUNT_BIT: usize = 25;
const PRASTEL_42_COUNT_BIT: usize = 42;
const AIRFORCE_COUNT_BIT: usize = 18;
/// RogueMaster `CAME_12_FOREIGN_TE_MIN`: a 12-bit frame this slow (mean Te, µs) may be Faac TM's.
const FOREIGN_TE_MIN: u32 = 340;
/// RogueMaster `CAME_12_GAP_TE_MIN` / `_MAX`: CAME repeats every 47 Te.
const GAP_TE_MIN: u32 = 42;
const GAP_TE_MAX: u32 = 52;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    FoundStartBit,
    SaveDuration,
    CheckDuration,
}

/// CAME decoder (`SubGhzProtocolDecoderCame`).
pub struct CameDecoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
    /// RogueMaster's `te`: the sum of the frame's pulses (start bit included).
    te: u32,
}

/// The display variant `get_string` names by bit count.
fn display_name(bits: usize) -> Option<String> {
    match bits {
        PRASTEL_25_COUNT_BIT | PRASTEL_42_COUNT_BIT => Some("CAME [Prastel]".to_string()),
        AIRFORCE_COUNT_BIT => Some("CAME [Airforce]".to_string()),
        _ => None,
    }
}

fn decoded(data: u64, bits: usize) -> DecodedSignal {
    DecodedSignal {
        encoder_capable: bits <= PRASTEL_42_COUNT_BIT,
        protocol_display_name: display_name(bits),
        ..fixed_code(data, bits)
    }
}

/// `get_upload`'s header length in te for a bit count.
fn header_te(bits: usize) -> u32 {
    match bits {
        CAME_24_COUNT_BIT | PRASTEL_42_COUNT_BIT => 76,
        MIN_COUNT_BIT | AIRFORCE_COUNT_BIT => 47,
        PRASTEL_25_COUNT_BIT => 36,
        _ => 16,
    }
}

/// One `get_upload` frame.
fn build_frame(data: u64, bits: usize) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(bits * 2 + 2);
    push(&mut out, false, TE_SHORT * header_te(bits));
    push(&mut out, true, TE_SHORT);
    for i in (0..bits).rev() {
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

impl CameDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
            te: 0,
        }
    }

    fn add_bit(&mut self, bit: u64) {
        self.decode_data = self.decode_data << 1 | bit;
        self.decode_count_bit = self.decode_count_bit.wrapping_add(1);
    }

    /// RogueMaster `subghz_protocol_came_is_foreign_12bit`: a 12-bit frame slower than any CAME
    /// remote whose closing `gap` is not CAME's 47-Te repeat (see the module doc).
    fn is_foreign_12bit(&self, gap: u32) -> bool {
        if self.decode_count_bit as usize != MIN_COUNT_BIT {
            return false;
        }
        // start bit plus 3 Te per bit
        let te = self.te / (MIN_COUNT_BIT as u32 * 3 + 1);
        if te < FOREIGN_TE_MIN {
            return false;
        }
        let gap_te = gap / te;
        !(GAP_TE_MIN..=GAP_TE_MAX).contains(&gap_te)
    }
}

impl Default for CameDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for CameDecoder {
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

    fn reset(&mut self) {
        self.step = Step::Reset;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                if !level && duration_diff!(duration, TE_SHORT * 56) < TE_DELTA * 63 {
                    self.step = Step::FoundStartBit;
                }
            }
            Step::FoundStartBit => {
                if !level {
                    // stay
                } else if duration_diff!(duration, TE_SHORT) < TE_DELTA {
                    self.step = Step::SaveDuration;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.te = duration;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::SaveDuration => {
                if !level {
                    if duration >= TE_SHORT * 4 {
                        self.step = Step::FoundStartBit;
                        let bits = self.decode_count_bit as usize;
                        if matches!(
                            bits,
                            MIN_COUNT_BIT
                                | AIRFORCE_COUNT_BIT
                                | PRASTEL_25_COUNT_BIT
                                | PRASTEL_42_COUNT_BIT
                                | CAME_24_COUNT_BIT
                        ) && !self.is_foreign_12bit(duration)
                        {
                            // callback → the app resets the receiver (came_reset).
                            self.step = Step::Reset;
                            return Some(decoded(self.decode_data, bits));
                        }
                        return None;
                    }
                    self.te_last = duration;
                    self.te = self.te.wrapping_add(duration);
                    self.step = Step::CheckDuration;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::CheckDuration => {
                if level {
                    self.te = self.te.wrapping_add(duration);
                    if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA
                        && duration_diff!(duration, TE_LONG) < TE_DELTA
                    {
                        self.add_bit(0);
                        self.step = Step::SaveDuration;
                    } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA
                        && duration_diff!(duration, TE_SHORT) < TE_DELTA
                    {
                        self.add_bit(1);
                        self.step = Step::SaveDuration;
                    } else {
                        self.step = Step::Reset;
                    }
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
        let bits = decoded.data_count_bit;
        if bits > PRASTEL_42_COUNT_BIT {
            return None;
        }
        Some(with_repeats(&build_frame(decoded.data, bits), TX_REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        let (bits, data) = generic_fields(key);
        (bits <= PRASTEL_42_COUNT_BIT).then(|| decoded(data, bits))
    }

    fn to_flipper_key(&self, d: &DecodedSignal) -> Option<FlipperKey> {
        Some(generic_key(NAME, d))
    }
}

#[cfg(test)]
pub(super) mod testing {
    //! Frames for other modules' tests.
    use super::*;

    /// The encoder's upload for a `bits`-bit `data` (3 copies), with a trailing silence.
    pub fn on_air(data: u64, bits: usize) -> Vec<LevelDuration> {
        let mut wire = CameDecoder::new().encode(&decoded(data, bits), 0).unwrap();
        push(&mut wire, false, 30_000);
        wire
    }
}

#[cfg(test)]
mod tests {
    use super::super::g1_common::test_support::*;
    use super::*;

    /// The encoder's repeats with a trailing silence (the decoder needs a gap after each frame).
    fn on_air(d: &DecodedSignal) -> Vec<LevelDuration> {
        let mut wire = CameDecoder::new().encode(d, 0).unwrap();
        push(&mut wire, false, 30_000);
        wire
    }

    /// Every decoded length round-trips; the 3 repeats give 2 decodes (the reset inside the
    /// callback consumes the next frame's header).
    #[test]
    fn encode_decode_roundtrip() {
        for (data, bits, name) in [
            (0xABCu64, 12, None),
            (0x2_A5A5, 18, Some("CAME [Airforce]")),
            (0x6AB234, 24, None),
            (0x1A5_A5A5, 25, Some("CAME [Prastel]")),
            (0x2AB_CDEF_0123, 42, Some("CAME [Prastel]")),
        ] {
            let got = feed_all(&mut CameDecoder::new(), &on_air(&decoded(data, bits)));
            assert_eq!(got.len(), 2, "{bits} bits");
            for d in &got {
                assert_eq!((d.data, d.data_count_bit), (data, bits));
                assert_eq!(d.protocol_display_name.as_deref(), name);
                assert!(
                    d.serial.is_none() && d.button.is_none() && !d.crc_valid && d.encoder_capable
                );
            }
        }
    }

    #[test]
    fn header_lengths_follow_the_bit_count() {
        assert_eq!(build_frame(0, 24)[0].duration_us, 24320);
        assert_eq!(build_frame(0, 12)[0].duration_us, 15040);
        assert_eq!(build_frame(0, 25)[0].duration_us, 11520);
        assert_eq!(build_frame(0, 30)[0].duration_us, 5120);
    }

    #[test]
    fn rejects_other_lengths_and_bad_timing() {
        // 20 bits is not a CAME length (the 5120 µs default header is outside the window, too).
        let mut wire = vec![LevelDuration::new(false, 20_000)];
        wire.extend(build_frame(0xABCDE, 20).into_iter().skip(1));
        push(&mut wire, false, 30_000);
        assert!(feed_all(&mut CameDecoder::new(), &wire).is_empty());
        // A short pulse of 480 µs (|480 − 320| ≥ 150 and |480 − 640| ≥ 150) breaks the frame.
        let bad: Vec<LevelDuration> = on_air(&decoded(0x6AB234, 24))
            .into_iter()
            .enumerate()
            .map(|(i, p)| {
                if i == 20 {
                    LevelDuration::new(p.level, 480)
                } else {
                    p
                }
            })
            .collect();
        assert_eq!(
            feed_all(&mut CameDecoder::new(), &bad).len(),
            1,
            "only the intact repeat decodes"
        );
    }

    /// A 12-bit frame of `data` at a uniform `te` (start bit, 12 short/long pairs), closed by `gap`.
    fn frame_at(data: u64, te: u32, gap: u32) -> Vec<LevelDuration> {
        let mut w = vec![LevelDuration::new(false, te * 47)];
        push(&mut w, true, te);
        for i in (0..12).rev() {
            let (low, high) = if (data >> i) & 1 == 1 { (te * 2, te) } else { (te, te * 2) };
            push(&mut w, false, low);
            push(&mut w, true, high);
        }
        push(&mut w, false, gap);
        w
    }

    /// RogueMaster's foreign-frame rule: a 12-bit frame at Te ≥ 340 µs is CAME's only when its gap
    /// is 42..=52 Te; nominal CAME timing and the other lengths are unaffected.
    #[test]
    fn slow_12bit_frames_need_came_gap() {
        let got = |w: &[LevelDuration]| feed_all(&mut CameDecoder::new(), w).len();
        // Nominal 320 µs: any closing gap.
        assert_eq!(got(&frame_at(0xA5C, 320, 320 * 35)), 1);
        assert_eq!(got(&frame_at(0xA5C, 320, 60_000)), 1);
        // 380 µs (a fast Faac TM fits CAME's windows): CAME's 47 Te gap yes, Faac TM's 35 Te or
        // silence no.
        assert_eq!(got(&frame_at(0xA5C, 380, 380 * 47)), 1);
        assert_eq!(got(&frame_at(0xA5C, 380, 380 * 35)), 0);
        assert_eq!(got(&frame_at(0xA5C, 380, 60_000)), 0);
        // 24 bits at 380 µs: not tested (header inside the 17920 ± 9450 µs window).
        let mut w = vec![LevelDuration::new(false, 380 * 47)];
        push(&mut w, true, 380);
        for i in (0..24).rev() {
            let (low, high) = if (0x6AB234u64 >> i) & 1 == 1 { (760, 380) } else { (380, 760) };
            push(&mut w, false, low);
            push(&mut w, true, high);
        }
        push(&mut w, false, 380 * 35);
        assert_eq!(got(&w), 1);
    }

    #[test]
    fn unit_test_key_file() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: CAME\nBit: 24\n\
                    Key: 00 00 00 00 00 6A B2 34\n";
        let dec = CameDecoder::new();
        let d = dec
            .from_flipper_key(&FlipperKey::parse(text).unwrap())
            .unwrap();
        assert_eq!((d.data, d.data_count_bit), (0x6AB234, 24));
        let got = feed_all(&mut CameDecoder::new(), &on_air(&d));
        assert_eq!(got[0].data, 0x6AB234);
        let written = dec.to_flipper_key(&d).unwrap().to_text();
        assert!(
            written.contains("Protocol: CAME\nBit: 24\nKey: 00 00 00 00 00 6A B2 34\n"),
            "{written}"
        );
        let back = dec
            .from_flipper_key(&FlipperKey::parse(&written).unwrap())
            .unwrap();
        assert_eq!(format!("{back:?}"), format!("{d:?}"));
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&text.replace("Bit: 24", "Bit: 43")).unwrap())
            .is_none());
    }

    #[test]
    fn reference_raw_capture_decodes() {
        let Some(decodes) = decode_reference_capture("came_raw.sub") else {
            return;
        };
        assert!(
            decodes.iter().any(|(n, _)| n.starts_with(NAME)),
            "{:?}",
            decodes.iter().map(|(n, _)| n).collect::<Vec<_>>()
        );
    }
}
