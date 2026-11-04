//! Ditec GOL4 decoder + encoder (rolling code, keyless bit shuffle + LCG).
//!
//! Primary reference: Flipper-ARF's garage app,
//! `applications/system/subghz_garage/protocols/ditec_gol4.c` (decode-only there: the garage app
//! has no TX plugin for it). Union with doctor-firmware's `lib/subghz/protocols/ditec_gol4.c`,
//! which transmits; the two copies decode and encode identically (the garage `reset()` also
//! clears the bit buffer, which the header clears anyway).
//!
//! **Air format** (AM 433 MHz): PWM with the bit's LOW first, te 400/1100 µs (te_delta 200). A
//! 24.2 ms LOW gap (± 4.4 ms), an 800 µs HIGH start bit (± 200), 54 bits MSB first (1 = LOW 400 +
//! HIGH 1100, 0 = LOW 1100 + HIGH 400). The frame is reported only when the next gap (stored LOW
//! 22 ms ± 3.3 ms), start bit and first LOW follow — exactly as the C: on the LOW after an
//! unmatched HIGH it checks the *previous* LOW against 22 ms. In a Flipper receiver (which resets
//! after each decode) every second copy of a transmission decodes and the last copy never does.
//!
//! **Fields** (`subghz_protocol_ditec_gol4_decode_key`, no key needed): the 54 bits as 7 bytes
//! MSB first; bytes 2..6 rotate right 3 bits (with the C's carry chaining), bytes 0, 1, 3..6 are
//! bit-reversed, byte 2 is masked 0xDF, nibble-swapped, inverted and bit-reversed; then byte 5
//! (flipped by byte 6's MSB) and byte 6 undo an LCG `x → 21x + 1` (254 and byte-5 steps) and are
//! XORed with 0xA7/0x69. Serial = bytes `0 ‖ 4 ‖ 1 ‖ 3`, button = byte 2's low nibble (1, 2, 4, 8;
//! 0 = programming), counter = byte 5 | byte 6 << 8. No checksum (`crc_valid` false); every frame
//! is reported.
//!
//! **Encoder** (`deserialize` + `encode_key` + `get_upload`): decode the key, counter + 1 (the
//! firmware's multiplier, 1), button = the requested native nibble (0..=0xF; else the captured
//! one), re-encode (button 0 is sent as 0xF0); upload = LOW 24200, HIGH 800, 54 bits; 4 copies
//! (`encoder.repeat` = 4).

use super::g6_common::{self, push};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`): SubGhzProtocolFlag_AM | 433.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Ditec GOL4",
    decoder: "Ditec GOL4",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "Ditec",
    category: Category::Gate,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// The button nibble ("Basic set | 0x1 | 0x2 | 0x4 | 0x8 | 0x0 PROG").
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x1 => "Button 1",
        0x2 => "Button 2",
        0x4 => "Button 3",
        0x8 => "Button 4",
        0x0 => "Prog",
        _ => "Other",
    })
}

const TE_SHORT: u32 = 400;
const TE_LONG: u32 = 1100;
const TE_DELTA: u32 = 200;
const MIN_COUNT_BIT: u8 = 54;
/// The encoder's `encoder.repeat`.
const REPEAT: usize = 4;

/// `gol4_bit_parity`.
fn parity(b: u8) -> u8 {
    (b.count_ones() & 1) as u8
}

/// `gol4_lcg_step`: `steps` times x → 21x + 1 (mod 256).
fn lcg_step(seed: u8, steps: u8) -> u8 {
    (0..steps).fold(seed, |x, _| x.wrapping_mul(21).wrapping_add(1))
}

/// `gol4_lcg_inverse`.
fn lcg_inverse(target: u8, steps: u8) -> u8 {
    if steps == 0 {
        target
    } else {
        lcg_step(target, (256 - u16::from(steps)) as u8)
    }
}

/// `gol4_rolling_decode` (`gol4_decode_rotate_and_bitrev` + `gol4_decode_lcg_xor`).
fn rolling_decode(raw: &mut [u8; 7]) {
    let mut carry = 0u8;
    for _ in 0..3 {
        for b in &mut raw[2..7] {
            let new_carry = *b & 1;
            *b = (*b >> 1) | (carry << 7);
            carry = new_carry;
        }
    }
    for i in [0, 1, 3, 4] {
        raw[i] = raw[i].reverse_bits();
    }
    let b2 = (raw[2] & 0xDF).rotate_left(4);
    raw[2] = (!b2).reverse_bits();
    raw[5] = raw[5].reverse_bits();
    raw[6] = raw[6].reverse_bits();

    if raw[6] & 0x80 != 0 {
        raw[5] ^= 1;
    }
    raw[5] = lcg_inverse(raw[5], 0xFE);
    raw[6] = lcg_inverse(raw[6], raw[5]);
    raw[5] ^= 0xA7;
    raw[6] ^= 0x69;
}

/// `gol4_rolling_encode` (`gol4_encode_lcg_xor` + `gol4_encode_bitrev_and_rotate`).
fn rolling_encode(raw: &mut [u8; 7]) {
    let dec5 = raw[5] ^ 0xA7;
    let dec6 = raw[6] ^ 0x69;
    let enc6 = lcg_step(dec6, dec5);
    let mut enc5 = lcg_step(dec5, 0xFE);
    if enc6 & 0x80 != 0 {
        enc5 ^= 1;
    }
    raw[5] = enc5;
    raw[6] = enc6;

    for i in [0, 1, 3, 4] {
        raw[i] = raw[i].reverse_bits();
    }
    if raw[2] == 0 {
        raw[2] = 0xF0;
    }
    raw[2] = (!raw[2].reverse_bits()).rotate_left(4) & 0xDF;
    raw[5] = raw[5].reverse_bits();
    raw[6] = raw[6].reverse_bits();
    let (p5, p6) = (parity(raw[5]), parity(raw[6]));
    let mut carry = 0u8;
    for _ in 0..3 {
        for b in raw[2..7].iter_mut().rev() {
            let new_carry = (*b >> 7) & 1;
            *b = (*b << 1) | carry;
            carry = new_carry;
        }
    }
    raw[6] = if p5 == p6 { raw[6] & 0xFB } else { raw[6] | 0x04 };
}

/// The 54 frame bits as 7 bytes MSB first (`bits_to_raw`; the last 2 bits are 0).
fn to_raw(data: u64) -> [u8; 7] {
    let b = ((data & ((1 << 54) - 1)) << 10).to_be_bytes();
    [b[0], b[1], b[2], b[3], b[4], b[5], b[6]]
}

/// `raw_to_bits` + `bits_to_data`: the first 54 bits of 7 bytes.
fn from_raw(raw: &[u8; 7]) -> u64 {
    u64::from_be_bytes([raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], raw[6], 0]) >> 10
}

/// Serial, button and counter of a frame (`subghz_protocol_ditec_gol4_decode_key`).
fn decode_key(data: u64) -> (u32, u8, u16) {
    let mut raw = to_raw(data);
    rolling_decode(&mut raw);
    let serial = u32::from_be_bytes([raw[0], raw[4], raw[1], raw[3]]);
    (serial, raw[2] & 0x0F, u16::from_le_bytes([raw[5], raw[6]]))
}

/// The frame for serial, button and counter (`subghz_protocol_ditec_gol4_encode_key` after its
/// counter step).
fn encode_key(serial: u32, button: u8, counter: u16) -> u64 {
    let s = serial.to_be_bytes();
    let c = counter.to_le_bytes();
    let mut raw = [s[0], s[2], button & 0x0F, s[3], s[1], c[0], c[1]];
    rolling_encode(&mut raw);
    from_raw(&raw)
}

/// doctor-firmware `subghz_protocol_ditec_gol4_create_data` (the SubGHz app's "Add Manually"):
/// `encode_key` with its counter step (+ 1); `Bit: 54`, `Key`.
pub(super) fn create_data(serial: u32, btn: u8, cnt: u16) -> FlipperKey {
    let next = g6_common::counter_increment(u32::from(cnt)) as u16;
    FlipperKey::for_protocol("Ditec GOL4", MIN_COUNT_BIT as usize, encode_key(serial, btn, next))
}

/// `subghz_protocol_encoder_ditec_gol4_get_upload` (one copy).
fn build_upload(data: u64) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(2 + 2 * MIN_COUNT_BIT as usize);
    push(&mut out, false, TE_LONG * 22);
    push(&mut out, true, TE_SHORT * 2);
    for i in (0..u32::from(MIN_COUNT_BIT)).rev() {
        if (data >> i) & 1 != 0 {
            push(&mut out, false, TE_SHORT);
            push(&mut out, true, TE_LONG);
        } else {
            push(&mut out, false, TE_LONG);
            push(&mut out, true, TE_SHORT);
        }
    }
    out
}

/// The decode of a key (`get_string`).
fn signal(data: u64) -> DecodedSignal {
    let (serial, button, counter) = decode_key(data);
    DecodedSignal {
        serial: Some(serial),
        button: Some(button),
        counter: Some(counter),
        crc_valid: false,
        data,
        data_count_bit: MIN_COUNT_BIT as usize,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

/// Decoder steps (`DitecGOL4DecoderStep`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    StartBit,
    SaveDuration,
    CheckDuration,
}

/// Ditec GOL4 decoder (`SubGhzProtocolDecoderDitecGOL4`).
pub struct DitecGol4Decoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    /// `SubGhzBlockDecoder.decode_count_bit` is a `uint8_t`.
    decode_count_bit: u8,
}

impl DitecGol4Decoder {
    pub fn new() -> Self {
        Self { step: Step::Reset, te_last: 0, decode_data: 0, decode_count_bit: 0 }
    }

    fn add_bit(&mut self, bit: u64) {
        self.decode_data = (self.decode_data << 1) | bit;
        self.decode_count_bit = self.decode_count_bit.wrapping_add(1);
    }
}

impl Default for DitecGol4Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for DitecGol4Decoder {
    fn name(&self) -> &'static str {
        "Ditec GOL4"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: MIN_COUNT_BIT as usize,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    /// The garage copy's reset (parser step and bit buffer).
    fn reset(&mut self) {
        self.step = Step::Reset;
        self.decode_data = 0;
        self.decode_count_bit = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                if !level && duration_diff!(duration, TE_LONG * 22) < TE_LONG * 4 {
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.step = Step::StartBit;
                }
            }
            Step::StartBit => {
                self.step = if level && duration_diff!(duration, TE_SHORT * 2) < TE_DELTA {
                    Step::SaveDuration
                } else {
                    Step::Reset
                };
            }
            Step::SaveDuration => {
                if !level {
                    self.te_last = duration;
                    self.step = Step::CheckDuration;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::CheckDuration => {
                if level {
                    if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA
                        && duration_diff!(duration, TE_LONG) < TE_DELTA
                    {
                        self.add_bit(1);
                        self.step = Step::SaveDuration;
                    } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA
                        && duration_diff!(duration, TE_SHORT) < TE_DELTA
                    {
                        self.add_bit(0);
                        self.step = Step::SaveDuration;
                    }
                    // Anything else: stay here (the C does not leave this step).
                } else {
                    let mut result = None;
                    if duration_diff!(self.te_last, TE_LONG * 20) < TE_LONG * 3 {
                        if self.decode_count_bit == MIN_COUNT_BIT {
                            result = Some(signal(self.decode_data));
                        }
                        self.decode_data = 0;
                        self.decode_count_bit = 0;
                    }
                    self.step = Step::Reset;
                    return result;
                }
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != MIN_COUNT_BIT as usize {
            return None;
        }
        let (serial, captured, counter) = decode_key(decoded.data);
        let btn = if button <= 0x0F { button } else { captured };
        let next = g6_common::counter_increment(u32::from(counter)) as u16;
        Some(build_upload(encode_key(serial, btn, next)).repeat(REPEAT))
    }

    /// The decoder's deserialize (`Bit` must be 54) + `get_string`.
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        (key.bits == MIN_COUNT_BIT as usize).then(|| signal(key.key))
    }

    /// The serialize: `Protocol`, `Bit`, `Key`.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        (decoded.data_count_bit == MIN_COUNT_BIT as usize).then(|| {
            FlipperKey::new("Ditec GOL4", MIN_COUNT_BIT as usize, decoded.data, 433_920_000, "FuriHalSubGhzPresetOok650Async")
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::g6_common::{decode_all, tuples};

    /// The C's samples of one remote (comment in `decode_key`): its decoded arrays read
    /// `CC 34 7x 83 09 Fx C1`, i.e. serial 0xCC093483 in display order, the button nibble, and a
    /// counter 0xC1Fx. Each key decodes so and re-encodes to itself.
    #[test]
    fn decodes_the_c_samples() {
        for (key, button) in [
            (0xCCB2F83208122u64, 0x1u8),
            (0xCCB2F8320ED66, 0x1),
            (0xCCB37832104A6, 0x2),
            (0xCCB3B8320DB4E, 0x4),
            (0xCCB3D8320E855, 0x8),
            (0xCCB1F832103B9, 0x0),
        ] {
            let (serial, b, c) = decode_key(key);
            assert_eq!((serial, b, c >> 8), (0xCC09_3483, button, 0xC1), "{key:X}");
            assert_eq!(encode_key(serial, b, c), key, "{key:X} re-encodes");
        }
    }

    #[test]
    fn encode_decode_round_trip() {
        for (serial, button, counter) in [(0xCC09_3483u32, 0x1u8, 0xC1F8u16), (0x1234_5678, 0x0, 0xFFFF), (0, 0x8, 0), (0xFFFF_FFFF, 0x4, 0x8000)] {
            let data = encode_key(serial, button, counter);
            assert!(data < 1 << 54);
            assert_eq!(decode_key(data), (serial, button, counter));
        }
    }

    /// The encoder's 4 copies: copies 1 and 3 decode in a Flipper-style receiver (each frame is
    /// reported at the next copy's first LOW, which the reset then swallows), with counter + 1 and
    /// the requested or captured button.
    #[test]
    fn encoder_upload_decodes() {
        let src = signal(encode_key(0xCC09_3483, 0x2, 0x00FF));
        for (button, want) in [(0x2u8, 0x2u8), (0x8, 0x8), (0x0, 0x0), (0x10, 0x2)] {
            let wire = DitecGol4Decoder::new().encode(&src, button).unwrap();
            assert_eq!(wire.len(), REPEAT * 110);
            let got = decode_all(&mut DitecGol4Decoder::new(), &wire);
            assert_eq!(got.len(), 2, "button {button}");
            for d in &got {
                assert_eq!((d.serial, d.button, d.counter), (Some(0xCC09_3483), Some(want), Some(0x0100)));
            }
        }
        let wrap = signal(encode_key(1, 1, 0xFFFF));
        let got = decode_all(&mut DitecGol4Decoder::new(), &DitecGol4Decoder::new().encode(&wrap, 1).unwrap());
        assert_eq!(got[0].counter, Some(0));
    }

    #[test]
    fn upload_layout_matches_the_c() {
        let t = tuples(&build_upload(encode_key(0xCC09_3483, 0x1, 0xC1F8)));
        assert_eq!(t.len(), 110);
        assert_eq!(&t[..2], &[(false, 24_200), (true, 800)]);
        assert!(t[2..].chunks(2).all(|p| p == [(false, 400), (true, 1100)] || p == [(false, 1100), (true, 400)]));
    }

    /// A frame needs the next gap + start bit + LOW to be reported; wrong bit counts and pulses
    /// out of tolerance are rejected.
    #[test]
    fn framing_and_negative_cases() {
        let one = build_upload(encode_key(0x0102_0304, 0x4, 0x1234));
        assert!(decode_all(&mut DitecGol4Decoder::new(), &one).is_empty(), "no trailing gap");
        let mut two = one.clone();
        two.extend(one.iter().take(3).copied());
        let got = decode_all(&mut DitecGol4Decoder::new(), &two);
        assert_eq!(got.len(), 1);
        assert_eq!((got[0].serial, got[0].button, got[0].counter), (Some(0x0102_0304), Some(0x4), Some(0x1234)));
        // 53 bits.
        let mut short: Vec<_> = one[..one.len() - 2].to_vec();
        short.extend(one.iter().take(3).copied());
        assert!(decode_all(&mut DitecGol4Decoder::new(), &short).is_empty());
        // Start bit out of tolerance (800 ± 200).
        let mut start = two.clone();
        start[1].duration_us = 1050;
        assert!(decode_all(&mut DitecGol4Decoder::new(), &start).is_empty());
        // A gap of 27 ms opens a frame (24.2 ± 4.4) but cannot close one (22 ± 3.3 ms).
        let mut late = two.clone();
        late[110].duration_us = 27_000;
        assert!(decode_all(&mut DitecGol4Decoder::new(), &late).is_empty());
    }

    const KEY: &str = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
        Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Ditec GOL4\nBit: 54\n\
        Key: 00 0C CB 2F 83 20 ED 66\n";

    /// No Flipper unit test covers Ditec GOL4; a key file in the reference's format (the C's
    /// sample key) loads, encodes and round-trips.
    #[test]
    fn key_file_loads_encodes_and_round_trips() {
        let dec = DitecGol4Decoder::new();
        let d = dec.from_flipper_key(&FlipperKey::parse(KEY).unwrap()).unwrap();
        assert_eq!((d.serial, d.button, d.data_count_bit), (Some(0xCC09_3483), Some(1), 54));
        assert!(dec.encode(&d, 1).is_some());
        assert!(dec.from_flipper_key(&FlipperKey::parse(&KEY.replace("Bit: 54", "Bit: 52")).unwrap()).is_none());
        let written = dec.to_flipper_key(&d).unwrap().to_text();
        assert_eq!(written, KEY);
        let b = dec.from_flipper_key(&FlipperKey::parse(&written).unwrap()).unwrap();
        assert_eq!(format!("{d:?}"), format!("{b:?}"));
    }
}
