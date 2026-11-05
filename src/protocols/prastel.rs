//! Prastel 42-bit rolling code (gate remotes).
//!
//! Primary reference: Flipper-ARF's garage app copy
//! `REFERENCES/Flipper-ARF/applications/system/subghz_garage/protocols/prastel.c` (Unleashed
//! port); doctor-firmware `lib/subghz/protocols/prastel.c` is the same protocol with an older
//! decoder gate and serial layout, and is the reference that registers its encoder (the garage
//! app compiles its copy of the encoder out). Flipper-ARF's `lib/subghz` has no prastel.c (its
//! CAME decoder reports 42-bit frames as a CAME variant named "Prastel").
//!
//! **Line coding** (CAME timing): te_short = 320 µs, te_long = 640 µs, te_delta = 150 µs. Each
//! bit is a LOW then a HIGH: 1 = LOW 640 + HIGH 320, 0 = LOW 320 + HIGH 640. A frame is a LOW
//! header (24 320 µs from the encoder), a 320 µs HIGH start bit and 42 data bits, MSB first.
//!
//! **Decoder** (`subghz_protocol_decoder_prastel_feed`, garage copy): Reset waits for a LOW
//! longer than 12·te_short and shorter than 86·te_short (3 840–27 520 µs, exclusive; the
//! garage copy widened doctor-firmware's |d − 17 920| < 9 450 window so the first frame after
//! the remote's ~4.8 ms preamble gap is not lost), then a HIGH within te_delta of te_short.
//! Bits are LOW/HIGH pairs; a LOW ≥ 4·te_short ends the frame and reports it when exactly 42
//! bits were read (and goes back to waiting for a start bit). No checksum is checked.
//!
//! **Fields** (garage `subghz_protocol_prastel_remote_controller`): the 42 bits are unpacked
//! into 7 bytes `p[]` (`data << 2`, MSB first) and unscrambled (three 40-bit right rotations
//! over p[2..6], bit reversals, and an LCG `x = 21x + 1` walk that turns the two counter bytes
//! back into the counter). serial = (p1 & 0xF0) << 8 | p3 | (p4 << 12) & 0xF0000 |
//! (p2 & 0x0F) << 8 | (p2 & 0x80) << 13 (doctor-firmware: p0 << 24 | p1 << 8 | p3 |
//! (p4 << 12) & 0xF0000); button = 1..4 from the low nibble of p4 (raw 0xE, 0xD, 0xB, 0x7;
//! any other nibble reads 0); counter = p5 | p6 << 8 (16 bits).
//!
//! **Encoder** (garage `subghz_protocol_encoder_prastel_deserialize`, built in doctor-firmware):
//! counter + 1 (0xFFFF wraps to 0, as `cnt + mult > 0xFFFF → 0` with mult 1), the button's raw
//! nibble written into p4, re-scrambled (`subghz_protocol_prastel_scramble`, which also sets the
//! counter parity bit) and packed back into 42 bits; upload = LOW 24 320 µs, HIGH 320 µs, 42 bit
//! pairs, sent 3 times. `button` (KAT's native code = the decoded 1..4 button) picks the raw
//! nibble, as the union of the two references' custom-button switches: the captured code keeps
//! the captured nibble; from a recognised button (1..4) any other 1..4 is sent with the garage
//! copy's `btn_to_raw` (0xE, 0xD, 0xB, 0x7); doctor-firmware's switch sends 0xB (button 3) from
//! any other non-zero nibble and 0xC (button 0) from 0xB; anything else keeps the captured nibble.
//!
//! **Key file**: `Protocol: Prastel`, `Bit: 42`, `Key` = the 42-bit frame (Flipper-ARF's
//! `subghz_block_generic_serialize`).

use super::g3_common::{duration_diff, repeat_upload, UploadBuilder};
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Prastel",
    decoder: "Prastel",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "Prastel",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

const TE_SHORT: u32 = 320;
const TE_LONG: u32 = 640;
const TE_DELTA: u32 = 150;
const BIT_COUNT: usize = 42;
const KEY_MASK: u64 = 0x3FF_FFFF_FFFF;
/// Encoder header: 76·te_short (24 320 µs, the CAME 24-bit header).
const HEADER_TE: u32 = 76;
/// `subghz_protocol_encoder_prastel_alloc`: repeat = 3.
const REPEAT: usize = 3;

/// `subghz_protocol_prastel_btn_from_raw`'s table, indexed by raw nibble − 7.
const BTN_MAP: [u8; 8] = [0x04, 0x00, 0x00, 0x00, 0x03, 0x00, 0x02, 0x01];
/// `subghz_protocol_prastel_btn_to_raw`'s table, indexed by button 1..4.
const RAW_MAP: [u8; 5] = [0x00, 0x0E, 0x0D, 0x0B, 0x07];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    FoundStartBit,
    SaveDuration,
    CheckDuration,
}

/// Prastel decoder (`SubGhzProtocolDecoderPrastel`).
pub struct PrastelDecoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
}

/// Decoded fields of a 42-bit frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Fields {
    serial: u32,
    button: u8,
    counter: u16,
    /// Low nibble of the unscrambled p[4] (the button as sent).
    raw_button: u8,
}

fn reverse_byte(v: u8) -> u8 {
    v.reverse_bits()
}

fn parity(v: u8) -> u8 {
    let v = v ^ (v >> 4);
    ((0x6996u16 >> (v & 0x0F)) & 1) as u8
}

/// `subghz_protocol_prastel_unpack`: 7 bytes of `data << 2`, MSB first.
fn unpack(data: u64) -> [u8; 7] {
    let payload = (((data >> 30) & 0xFFF) << 32) | u64::from((data << 2) as u32);
    let mut p = [0u8; 7];
    for (i, b) in p.iter_mut().enumerate() {
        *b = (payload >> (0x30 - 8 * i as u32)) as u8;
    }
    p
}

/// `subghz_protocol_prastel_pack`.
fn pack(p: &[u8; 7]) -> u64 {
    let mut payload = 0u64;
    for (i, &b) in p.iter().enumerate() {
        payload |= u64::from(b) << (0x30 - 8 * i as u32);
    }
    (payload >> 2) & KEY_MASK
}

/// `subghz_protocol_prastel_unscramble`.
fn unscramble(p: &mut [u8; 7]) {
    let mut carry = 0u8;
    for _ in 0..3 {
        for b in p.iter_mut().take(7).skip(2) {
            let cur = *b;
            *b = (carry << 7) | (cur >> 1);
            carry = cur & 1;
        }
    }

    p[0] = reverse_byte(p[0]);
    let t2 = p[2] & 0xDF;
    p[1] = reverse_byte(p[1]);
    p[3] = reverse_byte(p[3]);
    p[4] = reverse_byte(p[4]);
    let swapped = t2.rotate_left(4);
    p[2] = reverse_byte(!swapped);

    let p6 = p[6];
    p[5] = reverse_byte(p[5]);
    if p6 & 1 != 0 {
        p[5] ^= 1;
    }

    // Counter: p[5] picks how often the LCG x = 21x + 1 runs over p[6].
    let t = p[5].wrapping_mul(21).wrapping_add(1).wrapping_mul(21);
    let mut x = reverse_byte(p6);
    let n = t.wrapping_add(1);
    if n != 0 {
        let limit = !t;
        let mut i = 0u8;
        loop {
            x = x.wrapping_mul(21).wrapping_add(1);
            i = i.wrapping_add(1);
            if limit <= i {
                break;
            }
        }
    }
    p[5] = n ^ 0xA7;
    p[6] = x ^ 0x69;
}

/// `subghz_protocol_prastel_scramble`: exact inverse of [unscramble] (plus the parity bit).
pub(super) fn scramble(p: &mut [u8; 7]) {
    let mut n = p[5] ^ 0xA7;
    let mut x = p[6] ^ 0x69;

    let mut i = 0u8;
    while n > i {
        x = x.wrapping_mul(21).wrapping_add(1);
        i += 1;
    }
    for _ in 0..254 {
        n = n.wrapping_mul(21).wrapping_add(1);
    }

    let flip = x & 0x80 != 0;

    p[0] = reverse_byte(p[0]);
    p[1] = reverse_byte(p[1]);
    p[3] = reverse_byte(p[3]);
    p[4] = reverse_byte(p[4]);

    if flip {
        n ^= 1;
    }
    if p[2] == 0 {
        p[2] = 0xF0;
    }
    let w = !reverse_byte(p[2]);
    p[2] = ((w << 4) & 0xD0) | ((w >> 4) & 0x0F);
    p[5] = reverse_byte(n);
    p[6] = reverse_byte(x);

    let parity_5 = parity(p[5]);
    let parity_6 = parity(p[6]);

    let mut carry = 0u8;
    for _ in 0..3 {
        for idx in (2..=6).rev() {
            let cur = p[idx];
            p[idx] = (cur << 1) | carry;
            carry = cur >> 7;
        }
    }

    if parity_5 == parity_6 {
        p[6] &= !0x04;
    } else {
        p[6] |= 0x04;
    }
}

/// `subghz_protocol_prastel_btn_from_raw`.
fn btn_from_raw(raw: u8) -> u8 {
    let idx = (raw & 0x0F).wrapping_sub(7);
    if idx <= 7 {
        BTN_MAP[idx as usize]
    } else {
        0
    }
}

/// `subghz_protocol_prastel_btn_to_raw` (0 = none).
pub(super) fn btn_to_raw(btn: u8) -> u8 {
    if (1..=4).contains(&btn) {
        RAW_MAP[btn as usize]
    } else {
        0
    }
}

/// `subghz_protocol_prastel_remote_controller` (garage copy).
fn fields(data: u64) -> Fields {
    let mut p = unpack(data);
    unscramble(&mut p);
    let serial = ((u32::from(p[1]) & 0xF0) << 8)
        | u32::from(p[3])
        | ((u32::from(p[4]) << 12) & 0xF0000)
        | ((u32::from(p[2]) & 0x0F) << 8)
        | ((u32::from(p[2]) & 0x80) << 13);
    Fields {
        serial,
        button: btn_from_raw(p[4]),
        counter: u16::from(p[5]) | (u16::from(p[6]) << 8),
        raw_button: p[4] & 0x0F,
    }
}

/// `subghz_protocol_prastel_gen_data`: the frame with the raw button nibble and counter replaced.
fn gen_data(data: u64, raw_button: u8, counter: u16) -> u64 {
    let mut p = unpack(data);
    unscramble(&mut p);
    p[4] = (p[4] & 0xF0) | (raw_button & 0x0F);
    p[5] = counter as u8;
    p[6] = (counter >> 8) as u8;
    scramble(&mut p);
    pack(&p)
}

/// The raw nibble to send for KAT's native button code (see the module doc).
fn raw_for_button(button: u8, captured: &Fields) -> u8 {
    if button == captured.button {
        return captured.raw_button;
    }
    // Garage copy: a recognised button (1..4) switches to any other one (`btn_to_raw`).
    if (1..=4).contains(&captured.button) && (1..=4).contains(&button) {
        return btn_to_raw(button);
    }
    // doctor-firmware: a non-zero raw nibble switches to 0xB, and 0xB to 0xC.
    if captured.raw_button != 0 {
        let other = if captured.raw_button == 0x0B {
            0x0C
        } else {
            0x0B
        };
        if button == btn_from_raw(other) {
            return other;
        }
    }
    captured.raw_button
}

/// One upload (`subghz_protocol_encoder_prastel_get_upload`).
fn upload(data: u64) -> Vec<LevelDuration> {
    let mut b = UploadBuilder::new();
    b.push(false, TE_SHORT * HEADER_TE);
    b.push(true, TE_SHORT);
    for i in (0..BIT_COUNT).rev() {
        if (data >> i) & 1 == 1 {
            b.push(false, TE_LONG);
            b.push(true, TE_SHORT);
        } else {
            b.push(false, TE_SHORT);
            b.push(true, TE_LONG);
        }
    }
    b.finish()
}

fn signal(data: u64) -> DecodedSignal {
    let f = fields(data);
    DecodedSignal {
        serial: Some(f.serial),
        button: Some(f.button),
        counter: Some(f.counter),
        crc_valid: false,
        data,
        data_count_bit: BIT_COUNT,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

impl PrastelDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
        }
    }

    fn add_bit(&mut self, bit: u64) {
        self.decode_data = (self.decode_data << 1) | bit;
        self.decode_count_bit = self.decode_count_bit.wrapping_add(1);
    }
}

impl Default for PrastelDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for PrastelDecoder {
    fn name(&self) -> &'static str {
        "Prastel"
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
                if !level && duration > TE_SHORT * 12 && duration < TE_SHORT * 86 {
                    self.step = Step::FoundStartBit;
                }
            }
            Step::FoundStartBit => {
                if !level {
                    // stay
                } else if duration_diff(duration, TE_SHORT) < TE_DELTA {
                    self.step = Step::SaveDuration;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::SaveDuration => {
                if !level {
                    if duration >= TE_SHORT * 4 {
                        self.step = Step::FoundStartBit;
                        if usize::from(self.decode_count_bit) == BIT_COUNT {
                            return Some(signal(self.decode_data));
                        }
                        return None;
                    }
                    self.te_last = duration;
                    self.step = Step::CheckDuration;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::CheckDuration => {
                if level {
                    if duration_diff(self.te_last, TE_SHORT) < TE_DELTA
                        && duration_diff(duration, TE_LONG) < TE_DELTA
                    {
                        self.add_bit(0);
                        self.step = Step::SaveDuration;
                    } else if duration_diff(self.te_last, TE_LONG) < TE_DELTA
                        && duration_diff(duration, TE_SHORT) < TE_DELTA
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
        if decoded.data_count_bit != BIT_COUNT {
            return None;
        }
        let data = decoded.data & KEY_MASK;
        let captured = fields(data);
        let counter = decoded.counter.unwrap_or(captured.counter);
        // `cnt + mult > 0xFFFF → 0`, mult = 1.
        let counter = counter.checked_add(1).unwrap_or(0);
        let raw = raw_for_button(button, &captured);
        Some(repeat_upload(&upload(gen_data(data, raw, counter)), REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        // subghz_block_generic_deserialize_check_count_bit(.., 42)
        (key.bits == BIT_COUNT).then(|| signal(key.key))
    }

    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        (decoded.data_count_bit == BIT_COUNT).then(|| {
            FlipperKey::new(
                "Prastel",
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

    /// A frame built the way the encoder builds one: serial bytes, raw button, counter.
    fn frame(p1_hi: u8, p2: u8, p3: u8, p4_hi: u8, raw: u8, counter: u16) -> u64 {
        let mut p = [
            0u8,
            p1_hi << 4,
            p2,
            p3,
            (p4_hi << 4) | raw,
            counter as u8,
            (counter >> 8) as u8,
        ];
        scramble(&mut p);
        pack(&p)
    }

    #[test]
    fn scramble_unscramble_round_trip() {
        // Every counter value and button nibble comes back (p[2] bit 5 is not carried: the
        // unscramble masks it, so use fields with it clear).
        for counter in (0..=0xFFFFu32).step_by(97).chain([0, 1, 0xFFFE, 0xFFFF]) {
            for raw in [0x7u8, 0xB, 0xC, 0xD, 0xE] {
                let data = frame(0xA, 0x5C, 0x3D, 0x9, raw, counter as u16);
                assert_eq!(data & !KEY_MASK, 0);
                let f = fields(data);
                assert_eq!(f.counter, counter as u16, "counter {counter:#x}");
                assert_eq!(f.raw_button, raw);
                assert_eq!(f.button, btn_from_raw(raw));
                // gen_data with the same values reproduces the frame.
                assert_eq!(gen_data(data, raw, counter as u16), data);
            }
        }
    }

    #[test]
    fn button_tables() {
        assert_eq!([0x7, 0xB, 0xD, 0xE].map(btn_from_raw), [4, 3, 2, 1]);
        assert_eq!([0x0, 0x6, 0x8, 0xC, 0xF].map(btn_from_raw), [0; 5]);
        assert_eq!(
            (1..=4).map(btn_to_raw).collect::<Vec<_>>(),
            [0xE, 0xD, 0xB, 0x7]
        );
        assert_eq!(btn_to_raw(0), 0);
        assert_eq!(btn_to_raw(5), 0);
    }

    #[test]
    fn synthetic_frame_decodes() {
        let data = frame(0x3, 0x41, 0x77, 0x5, 0xD, 0x1234);
        let mut d = PrastelDecoder::new();
        // Preamble gap (~4.8 ms, accepted by the garage copy), one frame, then the next header.
        let mut p = vec![LevelDuration::new(true, 2000)];
        p.extend(upload(data));
        p.push(LevelDuration::new(false, 24_320));
        let frames = feed_all(&mut d, &p);
        assert_eq!(frames.len(), 1);
        let f = &frames[0];
        assert_eq!((f.data, f.data_count_bit), (data, 42));
        assert_eq!(f.button, Some(2));
        assert_eq!(f.counter, Some(0x1234));
        assert!(f.encoder_capable && !f.crc_valid);
        // A 4.8 ms preamble gap is a valid header for the garage decoder.
        let mut p = vec![
            LevelDuration::new(true, 2000),
            LevelDuration::new(false, 4800),
        ];
        p.extend(upload(data).into_iter().skip(1));
        p.push(LevelDuration::new(false, 24_320));
        assert_eq!(feed_all(&mut d, &p).len(), 1);
    }

    #[test]
    fn encode_round_trip_bumps_counter_and_switches_buttons() {
        let data = frame(0x3, 0x41, 0x77, 0x5, 0xD, 0xFFFE);
        let captured = signal(data);
        assert_eq!(captured.button, Some(2));
        for (button, want_btn, want_raw) in [
            (2u8, 2u8, 0xDu8), // captured
            (1, 1, 0xE),
            (3, 3, 0xB),
            (4, 4, 0x7),
            (9, 2, 0xD), // unknown → captured
        ] {
            let tx = PrastelDecoder::new().encode(&captured, button).unwrap();
            let frames = feed_all(&mut PrastelDecoder::new(), &tx);
            // Copies 1 and 2 are reported at the next copy's header; the last copy has no
            // trailing gap.
            assert_eq!(frames.len(), 2, "button {button}");
            for f in &frames {
                assert_eq!(f.button, Some(want_btn));
                assert_eq!(fields(f.data).raw_button, want_raw);
                assert_eq!(f.counter, Some(0xFFFF));
                assert_eq!(f.serial, captured.serial);
            }
        }
        // 0xFFFF + 1 wraps to 0.
        let wrapped = signal(gen_data(data, 0xD, 0xFFFF));
        let tx = PrastelDecoder::new().encode(&wrapped, 2).unwrap();
        let mut p = tx.clone();
        p.push(LevelDuration::new(false, 24_320));
        let frames = feed_all(&mut PrastelDecoder::new(), &p);
        assert!(frames.iter().all(|f| f.counter == Some(0)));
        // Doctor-firmware's 0xB ↔ 0xC switch: button 0 from a captured raw 0xB.
        let b = signal(gen_data(data, 0xB, 5));
        let tx = PrastelDecoder::new().encode(&b, 0).unwrap();
        let f = &feed_all(&mut PrastelDecoder::new(), &tx)[0];
        assert_eq!((fields(f.data).raw_button, f.button), (0xC, Some(0)));
        // A captured unmapped nibble (button 0) is kept when 0 is passed back; doctor-firmware's
        // switch sends 0xB (button 3) from it; buttons 1/2/4 are not reachable from it.
        let c = signal(gen_data(data, 0x8, 5));
        assert_eq!(c.button, Some(0));
        for (button, want_raw) in [(0u8, 0x8u8), (3, 0xB), (1, 0x8), (4, 0x8)] {
            let tx = PrastelDecoder::new().encode(&c, button).unwrap();
            let f = &feed_all(&mut PrastelDecoder::new(), &tx)[0];
            assert_eq!(fields(f.data).raw_button, want_raw, "button {button}");
        }
        // Raw nibble 0: neither reference switches it.
        let z = signal(gen_data(data, 0x0, 5));
        let tx = PrastelDecoder::new().encode(&z, 3).unwrap();
        assert_eq!(
            fields(feed_all(&mut PrastelDecoder::new(), &tx)[0].data).raw_button,
            0x0
        );
    }

    #[test]
    fn upload_shape_matches_the_c() {
        let data = frame(0x3, 0x41, 0x77, 0x5, 0xD, 7);
        let one = upload(data);
        assert_eq!((one[0].level, one[0].duration_us), (false, 24_320));
        assert!(one[1].level);
        // 2 + 42·2 entries, none merged (LOW/HIGH alternate).
        assert_eq!(one.len(), 86);
        let tx = PrastelDecoder::new().encode(&signal(data), 2).unwrap();
        assert_eq!(tx.len(), 3 * 86);
    }

    #[test]
    fn rejects_wrong_length_and_timing() {
        let data = frame(0x3, 0x41, 0x77, 0x5, 0xD, 0x55);
        let mut p = upload(data);
        p.push(LevelDuration::new(false, 24_320));
        // One bit short: the 41-bit frame is not reported.
        let mut short = p.clone();
        short.drain(2..4);
        assert!(feed_all(&mut PrastelDecoder::new(), &short).is_empty());
        // A stretched data pulse resets the decoder.
        let mut bad = p.clone();
        bad[20].duration_us = 1000;
        assert!(feed_all(&mut PrastelDecoder::new(), &bad).is_empty());
        // Header outside 3 840..27 520 µs.
        let mut bad = p.clone();
        bad[0].duration_us = 30_000;
        assert!(feed_all(&mut PrastelDecoder::new(), &bad).is_empty());
        assert!(PrastelDecoder::new()
            .encode(&DecodedSignal::new(data, 41), 1)
            .is_none());
    }

    #[test]
    fn key_file_round_trip() {
        let data = frame(0x3, 0x41, 0x77, 0x5, 0xD, 0x0102);
        let d = PrastelDecoder::new();
        let key = d.to_flipper_key(&signal(data)).unwrap();
        let text = key.to_text();
        assert!(
            text.contains("Protocol: Prastel\nBit: 42\nKey: 00 00 "),
            "{text}"
        );
        let back = d
            .from_flipper_key(&FlipperKey::parse(&text).unwrap())
            .unwrap();
        assert_eq!(format!("{back:?}"), format!("{:?}", signal(data)));
        let mut wrong = FlipperKey::parse(&text).unwrap();
        wrong.bits = 25;
        assert!(d.from_flipper_key(&wrong).is_none());
    }
}
