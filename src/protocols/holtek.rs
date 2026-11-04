//! Holtek (HT640 / HT6xx 40-bit fixed code) decoder + encoder.
//!
//! Port of Flipper-ARF's garage app `applications/system/subghz_garage/protocols/holtek.c` (the
//! primary reference; doctor-firmware's copy has the same wire behaviour). Flipper-ARF's lib copy
//! adds an encoder button switch (moving the button code to another of the four positions), which
//! KAT takes as "send this native button code".
//!
//! **Air format** (AM, 433.92 MHz): `te_short` 430, `te_long` 870, `te_delta` 100. Header LOW 36·te
//! (15480 µs), start HIGH te, then MSB first: bit 1 = LOW 870 + HIGH 430, bit 0 = LOW 430 + HIGH 870.
//! 40 bits whose top nibble is 0x5 (`HOLTEK_HEADER`).
//!
//! **Decoder** (`subghz_protocol_decoder_holtek_feed`): header LOW within 15480 ± 3600 µs, start
//! HIGH |d − 430| < 100, LOW/HIGH pairs (short ± 100, long ± 200); a LOW ≥ 4400 µs ends the frame,
//! reported at exactly 40 bits with the 0x5 header nibble. The decoder then waits for the next
//! start bit directly (the C sets `FoundStartBit` after the callback, overriding the app's reset).
//!
//! **Fields** (`subghz_protocol_holtek_check_remote_controller`): `serial` = bits 16..35 mirrored
//! (20 bits); the low 16 bits are four button nibbles (0xA = not pressed); `button` = position << 4
//! | code of the first non-0xA nibble from the low end (position 1..4; 0 when none). The code is
//! shown as ON (0xE) / OFF (0xB).
//!
//! **Encoder** (`subghz_protocol_encoder_holtek_get_upload`, `encoder.repeat` = 3): header, start
//! bit, 40 bits. The captured button re-sends the key as is (the garage app); another native code
//! (position 1..=4 << 4 | code ≠ 0xA) rewrites the button word to 0xAAAA with that code at that
//! position (ARF lib); other codes, or a key without the 0x5 header, keep the captured key.
//!
//! **Key file**: `Protocol: Holtek`, `Bit: 40`, `Key`.

use super::g1_common::{generic_fields, generic_key, push, reverse_key, with_repeats, TX_REPEAT};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Holtek",
    decoder: "Holtek",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Holtek",
    category: Category::Gate,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// Holtek buttons: position (1..4) and ON (0xE) / OFF (0xB), as the C displays them.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    const ON: [&str; 4] = ["B1 ON", "B2 ON", "B3 ON", "B4 ON"];
    const OFF: [&str; 4] = ["B1 OFF", "B2 OFF", "B3 OFF", "B4 OFF"];
    const PLAIN: [&str; 4] = ["B1", "B2", "B3", "B4"];
    let pos = (code >> 4) as usize;
    Some(match (pos, code & 0xF) {
        (0, 0) => "None",
        (1..=4, 0xE) => ON[pos - 1],
        (1..=4, 0xB) => OFF[pos - 1],
        (1..=4, _) => PLAIN[pos - 1],
        _ => "Other",
    })
}

const NAME: &str = "Holtek";
const TE_SHORT: u32 = 430;
const TE_LONG: u32 = 870;
const TE_DELTA: u32 = 100;
const MIN_COUNT_BIT: usize = 40;
const HEADER_MASK: u64 = 0xF0_0000_0000;
const HEADER: u64 = 0x50_0000_0000;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    FoundStartBit,
    SaveDuration,
    CheckDuration,
}

/// Holtek decoder (`SubGhzProtocolDecoderHoltek`).
pub struct HoltekDecoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
    /// The last `feed` reported a frame (see `g1_common`): keep the post-callback state.
    just_emitted: bool,
}

/// `subghz_protocol_holtek_check_remote_controller`: (serial, button).
fn check_remote_controller(data: u64) -> (u32, u8) {
    if data & HEADER_MASK != HEADER {
        return (0, 0);
    }
    let serial = reverse_key((data >> 16) & 0xFFFFF, 20) as u32;
    let word = (data & 0xFFFF) as u16;
    let btn = (0..4)
        .map(|pos| (pos, ((word >> (4 * pos)) & 0xF) as u8))
        .find(|&(_, nibble)| nibble != 0xA)
        .map_or(0, |(pos, nibble)| (pos as u8 + 1) << 4 | nibble);
    (serial, btn)
}

fn decoded(data: u64) -> DecodedSignal {
    let (serial, btn) = check_remote_controller(data);
    DecodedSignal {
        serial: Some(serial),
        button: Some(btn),
        counter: None,
        crc_valid: false,
        data,
        data_count_bit: MIN_COUNT_BIT,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

/// The key with button word 0xAAAA + `code` at `position` (1..=4).
fn with_button(data: u64, position: u8, code: u8) -> u64 {
    let shift = 4 * (position as u64 - 1);
    let word = (0xAAAAu64 & !(0xF << shift)) | ((code as u64 & 0xF) << shift);
    (data & !0xFFFF) | word
}

/// One `get_upload` frame.
fn build_frame(data: u64) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(MIN_COUNT_BIT * 2 + 2);
    push(&mut out, false, TE_SHORT * 36);
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

impl HoltekDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
            just_emitted: false,
        }
    }

    fn add_bit(&mut self, bit: u64) {
        self.decode_data = self.decode_data << 1 | bit;
        self.decode_count_bit = self.decode_count_bit.wrapping_add(1);
    }
}

impl Default for HoltekDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for HoltekDecoder {
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
        if std::mem::take(&mut self.just_emitted) {
            return;
        }
        self.step = Step::Reset;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        self.just_emitted = false;
        match self.step {
            Step::Reset => {
                if !level && duration_diff!(duration, TE_SHORT * 36) < TE_DELTA * 36 {
                    self.step = Step::FoundStartBit;
                }
            }
            Step::FoundStartBit => {
                if level && duration_diff!(duration, TE_SHORT) < TE_DELTA {
                    self.step = Step::SaveDuration;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::SaveDuration => {
                if !level {
                    if duration >= TE_SHORT * 10 + TE_DELTA {
                        let mut out = None;
                        if self.decode_count_bit as usize == MIN_COUNT_BIT
                            && self.decode_data & HEADER_MASK == HEADER
                        {
                            out = Some(decoded(self.decode_data));
                        }
                        // After the callback (and the app's reset in it) the C waits for the
                        // next start bit.
                        self.decode_data = 0;
                        self.decode_count_bit = 0;
                        self.step = Step::FoundStartBit;
                        self.just_emitted = out.is_some();
                        return out;
                    }
                    self.te_last = duration;
                    self.step = Step::CheckDuration;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::CheckDuration => {
                if level {
                    if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA
                        && duration_diff!(duration, TE_LONG) < TE_DELTA * 2
                    {
                        self.add_bit(0);
                        self.step = Step::SaveDuration;
                    } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA * 2
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

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        // The captured button re-sends the key untouched (the garage app's encoder); another
        // native code rewrites the button word.
        let (position, code) = (button >> 4, button & 0xF);
        let captured = check_remote_controller(decoded.data).1;
        let data = if button != captured
            && decoded.data & HEADER_MASK == HEADER
            && (1..=4).contains(&position)
            && code != 0xA
        {
            with_button(decoded.data, position, code)
        } else {
            decoded.data
        };
        Some(with_repeats(&build_frame(data), TX_REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        let (bits, data) = generic_fields(key);
        (bits == MIN_COUNT_BIT).then(|| decoded(data))
    }

    fn to_flipper_key(&self, d: &DecodedSignal) -> Option<FlipperKey> {
        Some(generic_key(NAME, d))
    }
}

#[cfg(test)]
mod tests {
    use super::super::g1_common::test_support::*;
    use super::*;

    #[test]
    fn fields_and_labels() {
        // Unit-test key 0x50_0000_AABA: button 2 (second nibble) OFF.
        assert_eq!(check_remote_controller(0x50_0000_AABA), (0, 0x2B));
        assert_eq!(check_remote_controller(0x5A_BCDE_AAAE).1, 0x1E);
        assert_eq!(check_remote_controller(0x5A_BCDE_AAAA).1, 0);
        assert_eq!(
            check_remote_controller(0x4A_BCDE_AAAE),
            (0, 0),
            "no 0x5 header"
        );
        assert_eq!(check_remote_controller(0x50_0001_AAAE).0, 0x80000);
        assert_eq!(native_buttons("Holtek", 0x2B), Some("B2 OFF"));
        assert_eq!(native_buttons("Holtek", 0x1E), Some("B1 ON"));
        assert_eq!(native_buttons("Holtek", 0x43), Some("B4"));
        assert_eq!(native_buttons("Holtek", 0), Some("None"));
    }

    #[test]
    fn encode_decode_roundtrip_with_button_positions() {
        let d = decoded(0x5A_BCDE_AABA);
        for (button, want) in [
            (0x2B, 0x5A_BCDE_AABAu64),
            (0x1E, 0x5A_BCDE_AAAE),
            (0x4B, 0x5A_BCDE_BAAA),
            (0x5B, 0x5A_BCDE_AABA),
            (0x1A, 0x5A_BCDE_AABA),
        ] {
            let mut wire = HoltekDecoder::new().encode(&d, button).unwrap();
            push(&mut wire, false, 20_000);
            // The decoder resumes at the next start bit after a decode: all 3 repeats decode.
            let got = feed_all(&mut HoltekDecoder::new(), &wire);
            assert_eq!(
                got.iter().map(|f| f.data).collect::<Vec<_>>(),
                [want; 3],
                "btn {button:02X}"
            );
        }
        // Two buttons held (0xBAAE): the captured button (0x1E) re-sends the key untouched.
        let two = decoded(0x5A_BCDE_BAAE);
        assert_eq!(two.button, Some(0x1E));
        let pairs = |w: Vec<LevelDuration>| {
            w.iter()
                .map(|p| (p.level, p.duration_us))
                .collect::<Vec<_>>()
        };
        assert_eq!(
            pairs(HoltekDecoder::new().encode(&two, 0x1E).unwrap()),
            pairs(with_repeats(&build_frame(0x5A_BCDE_BAAE), TX_REPEAT))
        );
    }

    #[test]
    fn registry_reset_keeps_the_post_callback_state() {
        let d = decoded(0x5A_BCDE_AABA);
        let mut wire = HoltekDecoder::new().encode(&d, 0x2B).unwrap();
        push(&mut wire, false, 20_000);
        let mut dec = HoltekDecoder::new();
        let mut n = 0;
        for p in &wire {
            if dec.feed(p.level, p.duration_us).is_some() {
                n += 1;
                dec.reset();
            }
        }
        assert_eq!(n, 3);
    }

    #[test]
    fn rejects_wrong_header_nibble_and_timing() {
        let mut wire = build_frame(0x4A_BCDE_AABA);
        push(&mut wire, false, 20_000);
        assert!(
            feed_all(&mut HoltekDecoder::new(), &wire).is_empty(),
            "header nibble 4"
        );
        let mut bad = build_frame(0x5A_BCDE_AABA);
        bad[10] = LevelDuration::new(bad[10].level, 650); // neither 430 ± 100 nor 870 ± 200
        push(&mut bad, false, 20_000);
        assert!(feed_all(&mut HoltekDecoder::new(), &bad).is_empty());
    }

    #[test]
    fn unit_test_key_file() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Holtek\nBit: 40\n\
                    Key: 00 00 00 50 00 00 AA BA\n";
        let dec = HoltekDecoder::new();
        let d = dec
            .from_flipper_key(&FlipperKey::parse(text).unwrap())
            .unwrap();
        assert_eq!(
            (d.data, d.serial, d.button),
            (0x50_0000_AABA, Some(0), Some(0x2B))
        );
        let mut wire = dec.encode(&d, d.button.unwrap()).unwrap();
        push(&mut wire, false, 20_000);
        assert_eq!(
            feed_all(&mut HoltekDecoder::new(), &wire)[0].data,
            0x50_0000_AABA
        );
        let written = dec.to_flipper_key(&d).unwrap().to_text();
        let back = dec
            .from_flipper_key(&FlipperKey::parse(&written).unwrap())
            .unwrap();
        assert_eq!(format!("{back:?}"), format!("{d:?}"));
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&text.replace("Bit: 40", "Bit: 24")).unwrap())
            .is_none());
    }

    #[test]
    fn reference_raw_capture_decodes() {
        let Some(decodes) = decode_reference_capture("holtek_raw.sub") else {
            return;
        };
        assert!(
            decodes.iter().any(|(n, _)| n == NAME),
            "{:?}",
            decodes.iter().map(|(n, _)| n).collect::<Vec<_>>()
        );
    }
}
