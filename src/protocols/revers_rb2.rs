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
//! **KAT's stream reader** (not in the C; from rtl_433's `conf/Revers-RB2.conf` and the
//! captures). The remote sends its message over and over without a gap: 32 ones, a 22-bit id, the
//! stop code. The C's header is any LOW of 440-760 µs with four shorts behind it, and that is
//! every 0 with three ones behind it, in the id as well; whether the 4 bits it then assumes are
//! the right number depends on `te_last`. A start in the wrong place is cured only by the 65th
//! bit, which moves the window by one bit per message, so the C reads 13 of the about 60 messages
//! of Flipper's unit-test capture, none of the 3 copies its own encoder sends, and none of the 8
//! messages of the recording `rtl_433_tests/tests/revers/rb2/01/revers_rb2_3abcde_433.92M_250k.cu8`
//! (id 3ABCDE: `0 1111` is in the id). KAT therefore reads the stream a second way beside the C's:
//! every short or long pulse goes through a Manchester decoder of its own, with no header, and a
//! message is reported when the last 74 bits are the stop code of the message before, 32 ones, 22
//! bits and the stop code. That is rtl_433's rule, and it needs the end of one message and the
//! whole of the next. The stop code before the ones is what keeps this from a frame of another
//! Manchester remote of these timings that comes behind a long preamble (Ford V3's European
//! frames): a preamble holds no stop code. A pulse that is neither short nor long, and a pulse
//! the Manchester decoder has no place for, start the count again. `reset()` keeps the
//! Manchester state and the last ten bits, since the pulses go on: the registry resets every
//! decoder after a message, and the next message is checked against the stop code of that one.
//! What the C's path reports is reported as before (a message whose bits 32..47 are not ones is
//! read by that path alone).
//!
//! [`is_stream`] is the same test for another decoder's frame: Ford V3's European path takes a
//! run of 30 shorts and the 104 bits behind it without any check, which is this stream whenever
//! its pulses are within 60 µs of 240 and 480 µs (ProtoPirate's and doctor-firmware's C name the
//! two rtl_433_tests recordings Ford V3).
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
/// The 10 bits every message ends with.
const STOP_CODE: u64 = 0x200;
/// Bits the stream reader needs: the stop code of the message before, and a message.
const STREAM_BITS: u8 = 74;

/// A message as the remote sends it: 32 ones, 22 bits of id, the stop code. (The C asks for bits
/// 48..55 and the stop code.)
const fn is_message(data: u64) -> bool {
    data >> 32 == 0xFFFF_FFFF && data & 0x3FF == STOP_CODE
}

/// True when the first `count` bits of `raw` (MSB first) are a cut of this remote's stream: they
/// repeat after 64 bits, and those 64 bits, begun at the right bit, are a message.
pub(super) fn is_stream(raw: &[u8], count: usize) -> bool {
    let bit = |i: usize| (raw[i / 8] >> (7 - i % 8)) & 1;
    if count < BIT_COUNT || raw.len() * 8 < count {
        return false;
    }
    if (BIT_COUNT..count).any(|i| bit(i) != bit(i - BIT_COUNT)) {
        return false;
    }
    let word = (0..BIT_COUNT).fold(0u64, |acc, i| (acc << 1) | u64::from(bit(i)));
    (0..BIT_COUNT as u32).any(|r| is_message(word.rotate_left(r)))
}

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
    /// KAT's stream reader (see the module doc): its Manchester state, the bits it read and how
    /// many of them follow one another without a break (at most [`STREAM_BITS`]).
    stream_manchester: ManchesterState,
    stream_data: u128,
    stream_bits: u8,
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
            stream_manchester: ManchesterState::Mid1,
            stream_data: 0,
            stream_bits: 0,
        }
    }

    /// One pulse for KAT's stream reader (see the module doc).
    fn stream_pulse(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let long = if duration_diff(duration, TE_SHORT) < TE_DELTA {
            false
        } else if duration_diff(duration, TE_LONG) < TE_DELTA {
            true
        } else {
            self.stream_manchester = ManchesterState::Mid1;
            self.stream_bits = 0;
            return None;
        };
        let event = ManchesterEvent::from_half_bit(level, long);
        let Some(bit) = self.stream_manchester.advance(event) else {
            // Mid1 without a bit is the decoder's answer to a pulse that fits nowhere.
            if self.stream_manchester == ManchesterState::Mid1 {
                self.stream_bits = 0;
            }
            return None;
        };
        self.stream_data = (self.stream_data << 1) | u128::from(bit);
        self.stream_bits = (self.stream_bits + 1).min(STREAM_BITS);
        let message = self.stream_data as u64;
        let before = (self.stream_data >> BIT_COUNT) as u64 & 0x3FF;
        (self.stream_bits == STREAM_BITS && before == STOP_CODE && is_message(message))
            .then(|| signal(message))
    }

    /// `subghz_protocol_decoder_revers_rb2_feed`.
    fn c_feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
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
        if preamble == 0xFF && stop_code == STOP_CODE {
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
    /// The stream reader keeps its Manchester state and its last ten bits, which are the stop
    /// code the next message is checked against.
    fn reset(&mut self) {
        self.step = Step::Reset;
        self.header_count = 0;
        self.manchester = ManchesterState::Mid1;
        self.stream_bits = self.stream_bits.min(STREAM_BITS - BIT_COUNT as u8);
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        // Both readers take every pulse; a message both complete on this pulse is reported once.
        let stream = self.stream_pulse(level, duration);
        self.c_feed(level, duration).or(stream)
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

    /// `count` messages one behind the other without a gap, as the remote sends them.
    fn stream(data: u64, count: usize) -> Vec<LevelDuration> {
        let mut enc = ManchesterEncoder::new();
        let mut results = Vec::new();
        for _ in 0..count {
            for i in (0..BIT_COUNT).rev() {
                enc.encode_bit((data >> i) & 1 == 1, &mut results);
            }
        }
        let mut b = UploadBuilder::new();
        for r in results {
            let p = r.pulse(TE_SHORT, TE_LONG);
            b.push(p.level, p.duration_us);
        }
        b.finish()
    }

    /// The C's reader alone.
    fn c_decodes(pulses: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut d = ReversRb2Decoder::new();
        pulses.iter().filter_map(|p| d.c_feed(p.level, p.duration_us)).collect()
    }

    /// The remote's stream is read from its second message on, wherever the capture starts. The
    /// two messages are those of rtl_433_tests' recordings (`tests/revers/rb2/01`).
    #[test]
    fn stream_is_read_wherever_it_starts() {
        for (data, id) in [(0xFFFF_FFFF_048D_1600u64, 0x01_2345u32), (0xFFFF_FFFF_EAF3_7A00, 0x3A_BCDE)] {
            assert!(is_message(data));
            let pulses = stream(data, 8);
            // Every message but the first, which has no stop code before it.
            for frames in [feed_all(&mut ReversRb2Decoder::new(), &pulses), decode_with_resets(&pulses)] {
                assert_eq!(frames.len(), 7, "{data:016X}");
                for f in &frames {
                    assert_eq!((f.data, f.data_count_bit), (data, 64));
                    assert_eq!(f.serial, Some(0xFFC0_0000 | id));
                }
            }
            // Begun at any pulse of the first message, at most two messages are lost.
            let first = stream(data, 1).len();
            for cut in 0..first {
                let frames = decode_with_resets(&pulses[cut..]);
                assert!(frames.len() >= 6, "{data:016X} from pulse {cut}: {}", frames.len());
                assert!(frames.iter().all(|f| f.data == data), "{data:016X} from pulse {cut}");
            }
            // A pulse that is neither short nor long costs the message it is in, and the next
            // one too when it is in the stop code.
            let mut broken = pulses.clone();
            broken[3 * first + 20].duration_us = 900;
            assert_eq!(decode_with_resets(&broken).len(), 6, "{data:016X}");
            let mut broken = pulses.clone();
            broken[4 * first - 12].duration_us = 900;
            assert_eq!(decode_with_resets(&broken).len(), 5, "{data:016X}");
        }
        // The C's reader starts at a 0 with three ones behind it. The id 3ABCDE holds one, and
        // none of its eight messages is read.
        assert!(c_decodes(&stream(0xFFFF_FFFF_EAF3_7A00, 8)).is_empty());
        // One message, or ones without a stop code before them, is not enough.
        assert!(decode_with_resets(&stream(0xFFFF_FFFF_048D_1600, 1)).is_empty());
        let mut led = stream(0xFFFF_FFFF_FFFF_FFFF, 1);
        led.extend(stream(0xFFFF_FFFF_048D_1600, 1));
        assert!(decode_with_resets(&merged(&led)).is_empty());
    }

    #[test]
    fn stream_test_for_other_decoders() {
        // What Ford V3's European path reads from the two recordings: the stream from the first
        // long pulse behind the ones.
        let ford = [0x04, 0x8D, 0x16, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x04, 0x8D, 0x16, 0x00, 0xFF];
        assert!(is_stream(&ford, 104));
        let ford = [0x57, 0x9B, 0xD0, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0x57, 0x9B, 0xD0, 0x07, 0xFF];
        assert!(is_stream(&ford, 104));
        // It does not repeat; it repeats and holds no stop code; no 32 ones; too short.
        let mut other = ford;
        other[12] = 0x7F;
        assert!(!is_stream(&other, 104));
        let other = [0x57, 0x9B, 0xD0, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0x57, 0x9B, 0xD0, 0x0F, 0xFF];
        assert!(!is_stream(&other, 104));
        let other = [0x04, 0x8D, 0x16, 0x00, 0xFF, 0xFF, 0xFF, 0x7F, 0x04, 0x8D, 0x16, 0x00, 0xFF];
        assert!(!is_stream(&other, 104));
        assert!(!is_stream(&ford[..7], 56));
        assert!(!is_stream(&ford[..8], 104));
        // A real Ford V3 frame (IMPORTS/LDV).
        let ldv = [0x44, 0x43, 0xA8, 0x8F, 0x90, 0x65, 0x68, 0x00, 0x00, 0x09, 0x48, 0xED, 0xD7];
        assert!(!is_stream(&ldv, 104));
    }

    #[test]
    fn unit_test_key_file_encodes_and_decodes_back() {
        let d = ReversRb2Decoder::new();
        let s = d
            .from_flipper_key(&FlipperKey::parse(KEY_FILE).unwrap())
            .unwrap();
        assert_eq!(s.data, 0xFFFF_FFFF_39F9_0A00);
        assert_eq!(s.serial, Some(((0xFFFF_39F9_0A00u64) >> 10) as u32));
        // The upload is the 3 copies the C sends. The C's reader locks on only after sliding
        // its 65-bit window over enough frames (the header path needs a reset decoder at a
        // frame's 570 µs LOW), so it reports none of the 3 copies; KAT's stream reader reports
        // the second and the third.
        let tx = d.encode(&s, 0).unwrap();
        assert_eq!(merged(&tx).len(), tx.len());
        assert!(c_decodes(&tx).is_empty());
        for frames in [feed_all(&mut ReversRb2Decoder::new(), &tx), decode_with_resets(&tx)] {
            assert_eq!(frames.len(), 2);
            for f in &frames {
                assert_eq!((f.data, f.data_count_bit, f.serial), (s.data, 64, s.serial));
            }
        }
        // Held for 70 copies: every copy but the first, and some of them by the C's reader too.
        let held = repeat_upload(&upload(s.data), 70);
        assert!(!c_decodes(&held).is_empty());
        for frames in [
            feed_all(&mut ReversRb2Decoder::new(), &held),
            decode_with_resets(&held),
        ] {
            assert_eq!(frames.len(), 69);
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
        assert!(frames.iter().all(|f| f.data == 0xFFFF_FFFF_39F9_0A00));
        let (_, raw) = crate::export::flipper::import_sub_raw(&path).expect("import .sub");
        let pairs: Vec<LevelDuration> =
            raw.iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect();
        // About 60 messages. The C's reader alone finds 1 (13 when it is reset after each, as in
        // the registry before the stream reader); with the stream reader it is 51, and no other
        // decoder takes one.
        assert_eq!(c_decodes(&pairs).len(), 1);
        assert_eq!((frames.len(), decode_with_resets(&pairs).len()), (51, 51));
        let found = registry_decodes(&path);
        assert_eq!(found.len(), 51);
        assert!(found.iter().all(|(n, d)| n == "Revers_RB2" && d.data == 0xFFFF_FFFF_39F9_0A00), "{found:?}");
    }
}
