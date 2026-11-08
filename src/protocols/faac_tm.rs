//! Faac TM: 12-bit DIP-switch fixed code (FAAC TM gate remotes), decoder + encoder.
//!
//! Port of RogueMaster `lib/subghz/protocols/faac_tm.c` (added in November 2026 with RogueMaster's
//! merge of Unleashed's dev branch; no other reference has it). The C registers it last, after
//! Freezer, and teaches its CAME decoder to leave Faac TM frames alone (see `came.rs`).
//!
//! **Air format** (AM, 433.92 / 868 MHz; the C's flags `433 | 868 | AM`): Te is nominally 400 µs and
//! measured per frame. A frame is a 35·Te LOW header, a Te HIGH start bit, then 12 bits MSB first,
//! each a LOW then a HIGH: 1 = LOW Te + HIGH 2·Te, 0 = LOW 2·Te + HIGH Te. It is CAME's grammar,
//! slower and with a shorter header; a fast Faac TM remote (Te up to ≈ 395 µs) fits CAME's windows.
//!
//! **Decoder** (`subghz_protocol_decoder_faac_tm_feed`): Reset waits for a LOW with |d − 14000| <
//! 4200 (35·te_short ± 35·te_delta); FoundStartBit skips LOWs and takes a HIGH |d − 400| < 120 as
//! the start bit (any other HIGH resets); then LOW/HIGH pairs: LOW 400 ± 120 + HIGH 800 ± 120 = 1,
//! LOW 800 ± 120 + HIGH 400 ± 120 = 0, anything else resets. A LOW ≥ 1600 µs ends the frame
//! (FoundStartBit next); with exactly 12 bits it is reported, Te = (the start bit and every data
//! pulse) / 37. The app's callback resets the receiver, so a report returns to Reset: the gap that
//! closed the frame was the next frame's header, so of a press's back-to-back copies every other one
//! decodes (as in RogueMaster). The windows read Te 341..=459 µs.
//!
//! **Registry.** After every stock fixed code, as RogueMaster registers it; CAME (before it) leaves
//! slow 12-bit frames with a non-CAME gap to it (`came.rs`). One difference from RogueMaster's
//! registry remains, for remotes running fast (Te ≤ 370 µs, where the 35-Te header also opens
//! Holtek_HT12X's 4960..12960 µs header window and every Faac TM pulse fits HT12X's wide windows):
//! KAT's Holtek_HT12X is Flipper-ARF's, whose reset keeps its repeat memory, so after a Faac TM
//! report it can confirm the next copy as an HT12X frame (inverted bits: HT12X's 1 is LOW long +
//! HIGH short). RogueMaster's HT12X clears that memory on reset, so its receiver reports every
//! copy as Faac TM. The first copy of such a press is Faac TM's in both.
//!
//! **Fields**: none in the C, which shows the key, the measured Te and the 12 DIP switches in the
//! order they are sent (`1` = on: the key's bits, MSB first); `extra` = the measured Te.
//!
//! **Encoder** (`subghz_protocol_encoder_faac_tm_get_upload`, `encoder.repeat` = 3): header LOW
//! 35·Te, start HIGH Te, the 12 bits; Te = the key's TE when 100 < TE < 1000, else 400. `button` is
//! ignored (no button field: the DIP switches are the code).
//!
//! **Key file**: `Protocol: Faac TM`, `Bit: 12`, `Key`, `TE` (the decoder's serialize writes the
//! measured Te). On load `TE` is optional, as the C encoder reads it (its decoder's deserialize
//! needs the line): "Add Manually" files carry `TE: 0` and are sent at the nominal 400 µs.

use super::f_common::{add_bit, push, with_repeats};
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

/// Metadata (see `info.rs`): `SubGhzProtocolFlag_433 | 868 | AM`, `SubGhzProtocolTypeStatic`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "FAAC",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

pub(super) const NAME: &str = "Faac TM";
const TE_SHORT: u32 = 400;
const TE_LONG: u32 = 800;
const TE_DELTA: u32 = 120;
const MIN_COUNT_BIT: usize = 12;
/// `FAAC_TM_HEADER_TE`: the header LOW in Te.
const HEADER_TE: u32 = 35;
/// `FAAC_TM_TE_MIN` / `_MAX`: a key file's TE is used only strictly between these.
const TE_MIN: u32 = 100;
const TE_MAX: u32 = 1000;
/// `subghz_protocol_encoder_common_alloc(…, 3, …)`.
const TX_REPEAT: usize = 3;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    FoundStartBit,
    SaveDuration,
    CheckDuration,
}

/// Faac TM decoder (`SubGhzProtocolDecoderFaacTm`).
pub struct FaacTmDecoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
    /// The C's `te`: the sum of the frame's pulses, then the mean.
    te: u32,
}

/// A decode (or a loaded key) of the 12-bit `data` at Te `te` (0 = none given).
fn decoded(data: u64, te: u32) -> DecodedSignal {
    DecodedSignal {
        serial: None,
        button: None,
        counter: None,
        crc_valid: false,
        data: data & 0xFFF,
        data_count_bit: MIN_COUNT_BIT,
        encoder_capable: true,
        extra: Some(u64::from(te)),
        protocol_display_name: None,
    }
}

/// The encoder's Te for a key's TE (the C's deserialize: the nominal rate unless 100 < TE < 1000).
fn tx_te(te: u32) -> u32 {
    if te > TE_MIN && te < TE_MAX {
        te
    } else {
        TE_SHORT
    }
}

/// One `get_upload` frame at Te `te`.
fn build_frame(data: u64, te: u32) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(MIN_COUNT_BIT * 2 + 2);
    push(&mut out, false, te * HEADER_TE);
    push(&mut out, true, te);
    for i in (0..MIN_COUNT_BIT).rev() {
        if (data >> i) & 1 == 1 {
            push(&mut out, false, te);
            push(&mut out, true, te * 2);
        } else {
            push(&mut out, false, te * 2);
            push(&mut out, true, te);
        }
    }
    out
}

impl FaacTmDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
            te: 0,
        }
    }
}

impl Default for FaacTmDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for FaacTmDecoder {
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
                if !level && duration_diff!(duration, TE_SHORT * HEADER_TE) < TE_DELTA * HEADER_TE {
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
                        if self.decode_count_bit as usize == MIN_COUNT_BIT {
                            // start bit plus 3 Te per bit
                            let te = self.te / (MIN_COUNT_BIT as u32 * 3 + 1);
                            // callback → the app resets the receiver (common_reset).
                            self.step = Step::Reset;
                            return Some(decoded(self.decode_data, te));
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
                        add_bit(&mut self.decode_data, &mut self.decode_count_bit, true);
                        self.step = Step::SaveDuration;
                    } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA
                        && duration_diff!(duration, TE_SHORT) < TE_DELTA
                    {
                        add_bit(&mut self.decode_data, &mut self.decode_count_bit, false);
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
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        let te = tx_te(decoded.extra.unwrap_or(0) as u32);
        Some(with_repeats(&build_frame(decoded.data, te), TX_REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        (key.bits == MIN_COUNT_BIT).then(|| decoded(key.key, key.u32_field("TE").unwrap_or(0)))
    }

    fn to_flipper_key(&self, d: &DecodedSignal) -> Option<FlipperKey> {
        let mut key = FlipperKey::new(NAME, d.data_count_bit, d.data, 433_920_000, "FuriHalSubGhzPresetOok650Async");
        key.push("TE", d.extra.unwrap_or(0).to_string());
        Some(key)
    }
}

#[cfg(test)]
pub(super) mod testing {
    //! Frames for other modules' tests.
    use super::*;

    /// The encoder's upload for `data` at Te `te`, with a trailing silence.
    pub fn on_air(data: u64, te: u32) -> Vec<LevelDuration> {
        let mut wire = FaacTmDecoder::new().encode(&decoded(data, te), 0).unwrap();
        push(&mut wire, false, 30_000);
        wire
    }
}

#[cfg(test)]
mod tests {
    use super::super::f_common::test_support::*;
    use super::*;
    use crate::protocols::ProtocolRegistry;

    /// 3 copies: the first decodes, its closing gap is the second's header, the third decodes.
    #[test]
    fn encode_decode_roundtrip() {
        // The C's windows read Te 341..=459 µs (long pulses 2·Te within 800 ± 120, exclusive).
        for (data, te) in [(0xA5Cu64, 400), (0x001, 345), (0xFFE, 455), (0x800, 0)] {
            let wire = testing::on_air(data, te);
            let got = feed_all(&mut FaacTmDecoder::new(), &wire);
            assert_eq!(got.len(), 2, "{data:03X} at {te}");
            let sent = tx_te(te);
            for d in &got {
                assert_eq!((d.data, d.data_count_bit), (data, 12));
                // The mean of exact pulses is the Te sent.
                assert_eq!(d.extra, Some(u64::from(sent)));
                assert!(d.encoder_capable && d.serial.is_none() && d.button.is_none());
            }
        }
    }

    #[test]
    fn upload_layout() {
        let up = FaacTmDecoder::new().encode(&decoded(0x801, 400), 0).unwrap();
        let p = |i: usize| (up[i].level, up[i].duration_us);
        // header, start, bit 1 (LOW Te, HIGH 2Te), ten 0 bits, a final 1.
        assert_eq!(p(0), (false, 14_000));
        assert_eq!(p(1), (true, 400));
        assert_eq!(p(2), (false, 400));
        assert_eq!(p(3), (true, 800));
        assert_eq!(p(4), (false, 800));
        assert_eq!(p(5), (true, 400));
        // 26 entries a copy (the copies' header LOWs follow a HIGH: nothing merges), 3 copies.
        assert_eq!(up.len(), 26 * 3);
        // Key-file TE 0 (Add Manually) or out of range → 400; in range → itself.
        assert_eq!(tx_te(0), 400);
        assert_eq!(tx_te(100), 400);
        assert_eq!(tx_te(1000), 400);
        assert_eq!(tx_te(101), 101);
    }

    #[test]
    fn rejects_other_lengths_and_bad_timing() {
        // 11 and 13 bits (a frame of `n` bits built by hand at 400 µs).
        for n in [11usize, 13] {
            let mut w = vec![LevelDuration::new(false, 14_000)];
            push(&mut w, true, 400);
            for i in 0..n {
                let one = i % 2 == 0;
                push(&mut w, false, if one { 400 } else { 800 });
                push(&mut w, true, if one { 800 } else { 400 });
            }
            push(&mut w, false, 30_000);
            assert!(feed_all(&mut FaacTmDecoder::new(), &w).is_empty(), "{n} bits");
        }
        // CAME's nominal 320/640 µs: 640 is not a long Faac TM pulse (680..920).
        let mut came = vec![LevelDuration::new(false, 15_040)];
        push(&mut came, true, 320);
        for i in (0..12).rev() {
            let (low, high) = if (0xA5Cu64 >> i) & 1 == 1 { (640, 320) } else { (320, 640) };
            push(&mut came, false, low);
            push(&mut came, true, high);
        }
        push(&mut came, false, 30_000);
        assert!(feed_all(&mut FaacTmDecoder::new(), &came).is_empty());
    }

    #[test]
    fn key_file_roundtrip() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 868000000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Faac TM\nBit: 12\n\
                    Key: 00 00 00 00 00 00 0A 5C\nTE: 0\n";
        let dec = FaacTmDecoder::new();
        let d = dec.from_flipper_key(&FlipperKey::parse(text).unwrap()).unwrap();
        assert_eq!((d.data, d.data_count_bit, d.extra), (0xA5C, 12, Some(0)));
        // TE 0 is sent at 400 µs.
        let up = dec.encode(&d, 0).unwrap();
        assert_eq!((up[1].level, up[1].duration_us), (true, 400));
        let written = dec.to_flipper_key(&d).unwrap().to_text();
        assert!(written.contains("Protocol: Faac TM\nBit: 12\nKey: 00 00 00 00 00 00 0A 5C\nTE: 0\n"), "{written}");
        // No TE line: loads (the encoder's reading), sent at 400 µs; Bit must be 12.
        let no_te = text.replace("TE: 0\n", "");
        assert_eq!(dec.from_flipper_key(&FlipperKey::parse(&no_te).unwrap()).unwrap().extra, Some(0));
        assert!(dec.from_flipper_key(&FlipperKey::parse(&text.replace("Bit: 12", "Bit: 24")).unwrap()).is_none());
    }

    /// Through the whole registry: Faac TM frames at their nominal and fast rates decode as Faac TM
    /// (CAME leaves slow 12-bit frames with a 35-Te gap alone); CAME frames stay CAME's.
    #[test]
    fn registry_keeps_faac_tm_and_came_apart() {
        let names = |wire: &[LevelDuration]| -> Vec<String> {
            let mut reg = ProtocolRegistry::new();
            reg.process_signal_stream_rf(wire, 433_920_000, Some(crate::capture::RfModulation::AM))
                .into_iter()
                .map(|(n, _, _)| n)
                .collect()
        };
        for te in [375, 400, 430, 455] {
            let got = names(&testing::on_air(0x5A3, te));
            assert_eq!(got, [NAME, NAME], "Te {te}");
        }
        // A remote running fast enough (Te ≤ 370 µs) for its 35-Te header to open Holtek_HT12X's
        // header window: the first copy is Faac TM's; HT12X (Flipper-ARF's, whose reset keeps its
        // repeat memory) may take a repeat (module doc).
        let got = names(&testing::on_air(0x5A3, 345));
        assert_eq!(got.first().map(String::as_str), Some(NAME), "{got:?}");
        let came = crate::protocols::came::testing::on_air(0x5A3, 12);
        let got = names(&came);
        assert!(!got.is_empty() && got.iter().all(|n| n == "CAME"), "{got:?}");
    }

    #[test]
    fn jittered_frames_decode() {
        let mut rng = Rng(0xFAAC_7E57);
        for _ in 0..50 {
            let data = rng.below(0x1000);
            let te = 370 + rng.below(61) as u32;
            let wire = jitter(&testing::on_air(data, te), &mut rng, 25);
            let got = feed_all(&mut FaacTmDecoder::new(), &wire);
            assert!(!got.is_empty(), "{data:03X} at {te}");
            assert!(got.iter().all(|d| d.data == data));
        }
    }
}
