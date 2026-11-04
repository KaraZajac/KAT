//! Revers RB2 / RB2M 64-bit Manchester fixed-code gate remote.
//!
//! Primary reference: Flipper-ARF's garage app copy
//! `REFERENCES/Flipper-ARF/applications/system/subghz_garage/protocols/revers_rb2.c`; the
//! doctor-firmware and Flipper-ARF `lib/subghz/protocols/revers_rb2.c` copies decode and
//! transmit identically (doctor-firmware also flags it `SubGhzProtocolFlag_ReversRB2`).
//!
//! **Line coding**: Manchester, te_short = 250 µs (half bit), te_long = 500 µs, te_delta =
//! 160 µs (so a 340–410 µs pulse counts as short: the short window is tested first). Flipper's
//! convention: a 1 is LOW then HIGH. 64 bits, MSB first; frames start with ones (key bits
//! 48..55 are 0xFF) and end with the 10-bit stop code 0x200.
//!
//! **Decoder** (`subghz_protocol_decoder_revers_rb2_feed`): Reset waits for a LOW within
//! te_delta of 600 µs (the 320 µs end LOW of the previous frame plus the first half bit, or a
//! LongLow inside a frame), resets the Manchester decoder, then counts 4 alternating short
//! pulses (`header_count`; the level of the last counted pulse is kept in `te_last`, which the
//! C's reset does not clear — ported as is), after which the data starts as 0xF (4 bits). Bits
//! come from Flipper's Manchester decoder (`manchester_advance`, level → Short/Long Low/High);
//! a pulse that is neither short nor long goes back to Reset. A 65th bit clears the shift
//! register; at exactly 64 bits the frame is reported when bits 48..55 are 0xFF and the low 10
//! bits are 0x200 (and the register and Manchester state are cleared).
//!
//! **Fields** (`subghz_protocol_revers_rb2_remote_controller`): serial = bits 10..47
//! (`((data << 16) >> 16) >> 10`, kept to 32 bits as the C's `uint32_t serial`).
//!
//! **Encoder** (`subghz_protocol_encoder_revers_rb2_get_upload`): the 64 bits through Flipper's
//! Manchester encoder, the closing half bit kept only when it is HIGH, then LOW 320 µs; sent 3
//! times. Static code: no counter, no button.
//!
//! **Key file**: `Protocol: Revers_RB2`, `Bit: 64`, `Key` (Flipper-ARF's generic serialize).

use super::common::{ManchesterEvent, ManchesterState};
use super::g3_common::{duration_diff, repeat_upload, ManchesterEncoder, UploadBuilder};
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Revers_RB2",
    decoder: "Revers_RB2",
    aliases: &["Revers RB2"],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Revers",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

const TE_SHORT: u32 = 250;
const TE_LONG: u32 = 500;
const TE_DELTA: u32 = 160;
const BIT_COUNT: usize = 64;
/// The header LOW the decoder looks for.
const HEADER_LOW: u32 = 600;
/// LOW closing every upload.
const END_LOW: u32 = 320;
/// `subghz_protocol_encoder_revers_rb2_alloc`: repeat = 3.
const REPEAT: usize = 3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    Header,
    DecoderData,
}

/// Revers RB2 decoder (`SubGhzProtocolDecoderRevers_RB2`).
pub struct ReversRb2Decoder {
    step: Step,
    /// Level of the last counted header pulse (0/1); survives `reset()` as in the C.
    te_last: u32,
    header_count: u16,
    manchester: ManchesterState,
    decode_data: u64,
    decode_count_bit: u8,
}

fn signal(data: u64) -> DecodedSignal {
    DecodedSignal {
        serial: Some((((data << 16) >> 16) >> 10) as u32),
        button: None,
        counter: None,
        crc_valid: false,
        data,
        data_count_bit: BIT_COUNT,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

/// One upload (`get_upload`).
fn upload(data: u64) -> Vec<LevelDuration> {
    let mut enc = ManchesterEncoder::new();
    let mut results = Vec::with_capacity(2 * BIT_COUNT);
    for i in (0..BIT_COUNT).rev() {
        enc.encode_bit((data >> i) & 1 == 1, &mut results);
    }
    let mut b = UploadBuilder::new();
    for r in results {
        let p = r.pulse(TE_SHORT, TE_LONG);
        b.push(p.level, p.duration_us);
    }
    // The closing half bit is kept only when HIGH (a LOW one is overwritten by the end LOW).
    let last = enc.finish().pulse(TE_SHORT, TE_LONG);
    if last.level {
        b.push(true, last.duration_us);
    }
    b.push(false, END_LOW);
    b.finish()
}

impl ReversRb2Decoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            header_count: 0,
            manchester: ManchesterState::Mid1,
            decode_data: 0,
            decode_count_bit: 0,
        }
    }

    /// `subghz_protocol_decoder_revers_rb2_addbit`.
    fn add_bit(&mut self, bit: bool) -> Option<DecodedSignal> {
        self.decode_data = (self.decode_data << 1) | u64::from(bit);
        self.decode_count_bit = self.decode_count_bit.wrapping_add(1);
        if self.decode_count_bit >= 65 {
            self.decode_data = 0;
            self.decode_count_bit = 0;
            return None;
        }
        if usize::from(self.decode_count_bit) < BIT_COUNT {
            return None;
        }
        let preamble = (self.decode_data >> 48) & 0xFF;
        let stop_code = self.decode_data & 0x3FF;
        if preamble == 0xFF && stop_code == 0x200 {
            let found = signal(self.decode_data);
            self.decode_data = 0;
            self.decode_count_bit = 0;
            self.manchester = ManchesterState::Mid1;
            return Some(found);
        }
        None
    }

    /// One header pulse: counts level alternations of short pulses.
    fn header_pulse(&mut self, level: bool, duration: u32) {
        if duration_diff(duration, TE_SHORT) < TE_DELTA {
            let expected_last = if level { 0 } else { 1 };
            if self.te_last == expected_last {
                self.header_count += 1;
            }
            self.te_last = u32::from(level);
        } else {
            self.header_count = 0;
            self.te_last = 0;
            self.step = Step::Reset;
        }
    }
}

impl Default for ReversRb2Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for ReversRb2Decoder {
    fn name(&self) -> &'static str {
        "Revers_RB2"
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

    /// `subghz_protocol_decoder_revers_rb2_reset`: step, header count and Manchester state only.
    fn reset(&mut self) {
        self.step = Step::Reset;
        self.header_count = 0;
        self.manchester = ManchesterState::Mid1;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                if !level && duration_diff(duration, HEADER_LOW) < TE_DELTA {
                    self.step = Step::Header;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.manchester = ManchesterState::Mid1;
                }
                None
            }
            Step::Header => {
                self.header_pulse(level, duration);
                if self.header_count == 4 {
                    self.header_count = 0;
                    self.decode_data = 0xF;
                    self.decode_count_bit = 4;
                    self.step = Step::DecoderData;
                }
                None
            }
            Step::DecoderData => {
                let long = if duration_diff(duration, TE_SHORT) < TE_DELTA {
                    false
                } else if duration_diff(duration, TE_LONG) < TE_DELTA {
                    true
                } else {
                    self.step = Step::Reset;
                    return None;
                };
                let event = ManchesterEvent::from_half_bit(level, long);
                match self.manchester.advance(event) {
                    Some(bit) => self.add_bit(bit),
                    None => None,
                }
            }
        }
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != BIT_COUNT {
            return None;
        }
        Some(repeat_upload(&upload(decoded.data), REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        (key.bits == BIT_COUNT).then(|| signal(key.key))
    }

    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        (decoded.data_count_bit == BIT_COUNT).then(|| {
            FlipperKey::new(
                "Revers_RB2",
                BIT_COUNT,
                decoded.data,
                433_920_000,
                "FuriHalSubGhzPresetOok650Async",
            )
        })
    }
}

#[cfg(test)]
mod tests {
    use super::super::g3_common::test_support::*;
    use super::*;

    /// revers_rb2.sub (Flipper unit test).
    const KEY_FILE: &str = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
        Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Revers_RB2\nBit: 64\n\
        Key: FF FF FF FF 39 F9 0A 00\n";

    /// Frames the decoder reports for `pulses`, reset after every frame as the registry (and the
    /// Flipper app's receiver callback) does.
    fn decode_with_resets(pulses: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut d = ReversRb2Decoder::new();
        let mut out = Vec::new();
        for p in pulses {
            if let Some(f) = d.feed(p.level, p.duration_us) {
                out.push(f);
                d.reset();
            }
        }
        out
    }

    #[test]
    fn unit_test_key_file_encodes_and_decodes_back() {
        let d = ReversRb2Decoder::new();
        let s = d
            .from_flipper_key(&FlipperKey::parse(KEY_FILE).unwrap())
            .unwrap();
        assert_eq!(s.data, 0xFFFF_FFFF_39F9_0A00);
        assert_eq!(s.serial, Some(((0xFFFF_39F9_0A00u64) >> 10) as u32));
        // The upload is the 3 copies the C sends. Like the C decoder, a fresh decoder locks on
        // only after sliding its 65-bit window over enough frames (the header path needs a
        // reset decoder at a frame's 570 µs LOW), so 3 copies alone are not reported: hold the
        // button (keep repeating the copy) to see it decode, as with the real remote.
        let tx = d.encode(&s, 0).unwrap();
        assert_eq!(merged(&tx).len(), tx.len());
        assert!(feed_all(&mut ReversRb2Decoder::new(), &tx).is_empty());
        let held = repeat_upload(&upload(s.data), 70);
        for frames in [
            feed_all(&mut ReversRb2Decoder::new(), &held),
            decode_with_resets(&held),
        ] {
            assert!(!frames.is_empty());
            for f in &frames {
                assert_eq!((f.data, f.data_count_bit, f.serial), (s.data, 64, s.serial));
            }
        }
        let text = d.to_flipper_key(&s).unwrap().to_text();
        assert!(text.contains("Protocol: Revers_RB2\nBit: 64\nKey: FF FF FF FF 39 F9 0A 00\n"));
        let back = d
            .from_flipper_key(&FlipperKey::parse(&text).unwrap())
            .unwrap();
        assert_eq!(format!("{back:?}"), format!("{s:?}"));
    }

    #[test]
    fn upload_layout() {
        // Key ends in a 0 bit: its LOW half becomes the 320 µs end LOW; three copies glue the
        // 320 µs LOW to the next copy's first LOW half (570 µs, the decoder's header LOW).
        let one = upload(0xFFFF_FFFF_39F9_0A00);
        assert_eq!((one[0].level, one[0].duration_us), (false, 250));
        let last = one.last().unwrap();
        assert_eq!((last.level, last.duration_us), (false, 320));
        assert!(one[one.len() - 2].level);
        let tx = repeat_upload(&one, 3);
        assert!(tx.iter().any(|p| !p.level && p.duration_us == 570));
        // A key ending in 1 keeps its HIGH closing half before the end LOW.
        let one = upload(0xFFFF_FFFF_39F9_0A01);
        let n = one.len();
        assert_eq!((one[n - 2].level, one[n - 2].duration_us), (true, 250));
    }

    #[test]
    fn synthetic_frames_need_preamble_and_stop_code() {
        for data in [
            0xFFFF_FFFF_39F9_0A00u64,
            0xFFFF_1234_5678_9E00,
            0xFFFF_0000_0000_0200,
        ] {
            assert_eq!(data & 0x3FF, 0x200);
            let frames = decode_with_resets(&repeat_upload(&upload(data), 70));
            assert!(!frames.is_empty(), "{data:016X}");
            assert!(frames.iter().all(|f| f.data == data), "{data:016X}");
        }
        // Wrong stop code / preamble: never reported.
        for data in [0xFFFF_FFFF_39F9_0B00u64, 0xFF7F_FFFF_39F9_0A00] {
            assert!(decode_with_resets(&repeat_upload(&upload(data), 70)).is_empty());
        }
        // A stretched pulse sends the decoder back to Reset.
        let mut p = repeat_upload(&upload(0xFFFF_FFFF_39F9_0A00), 1);
        let mut d = ReversRb2Decoder::new();
        p.insert(0, LevelDuration::new(false, 600));
        p[3].duration_us = 900;
        for x in &p[..4] {
            d.feed(x.level, x.duration_us);
        }
        assert_eq!(d.step, Step::Reset);
    }

    #[test]
    fn reset_keeps_te_last() {
        let mut d = ReversRb2Decoder::new();
        d.feed(false, 600);
        d.feed(true, 250);
        assert_eq!(d.te_last, 1);
        d.reset();
        assert_eq!((d.step, d.header_count, d.te_last), (Step::Reset, 0, 1));
    }

    #[test]
    fn decodes_the_flipper_unit_test_capture() {
        let Some(path) = unit_test_file("revers_rb2_raw.sub") else {
            return;
        };
        let frames = decoder_decodes(&mut ReversRb2Decoder::new(), &path);
        assert!(
            !frames.is_empty(),
            "the reference unit test needs ≥ 1 frame"
        );
        let names: Vec<String> = registry_decodes(&path)
            .into_iter()
            .map(|(n, _)| n)
            .collect();
        assert!(names.iter().any(|n| n == "Revers_RB2"), "{names:?}");
    }
}
