//! Honda V7 protocol decoder (**decode only**, as the reference)
//!
//! Ported from FoxFW `lib/subghz/protocols/honda_v7.c` and `honda_v7.h` (itself a port of the
//! KeeLoq-RX `arke_honda.py` `HondaV7` class); no other reference carries it. The C has no encoder
//! (`.encoder = NULL`), so neither does KAT.
//!
//! **Air format** (AM/OOK, 315 / 433.92 MHz): PWM with te 250 / 480 µs, tolerance 100 µs
//! (inclusive). A bit is a HIGH + LOW pair that are both short (0) or both long (1); a mixed pair
//! ends the frame. 88 bits are read, LSB first within each byte of the buffer, and every buffer
//! byte is bit-reversed (`rev8`) to its natural form.
//!
//! **Decoder** (`feed`): Reset wants a short HIGH; the preamble counts short-HIGH / short-LOW
//! pairs (at least 20 are needed). The first LOW that is not a short one after a short HIGH ends
//! it: a LOW of neither width starts the data with an empty buffer, and a long HIGH + long LOW
//! pair starts it with a first bit of 1. In `Data` a short pair is a 0 and a long pair a 1; a mixed
//! pair or an invalid LOW ends the frame, which is then tried when at least 64 bits were read.
//! 88 bits are read at most and emit as soon as the 88th pair is in; `validate_emit` needs all 88.
//!
//! **Frame** (natural bytes `nat[0..11]`): `button = nat[0] >> 4`, `serial = (nat[0] & 0xF) << 24
//! | nat[1..4]` (28 bits), `counter = nat[4..7]` (24 bits), `mode = nat[7] >> 4`,
//! `checksum = nat[8]`, `nat[9..11]` unread. The check is the XOR of `nat[0..8]` against
//! `nat[8]`; emission also needs a button of 1..9 and a serial that is neither 0 nor
//! 0x0FFFFFFF.
//!
//! **KAT fields**: `data` = `nat[0..8]` big-endian (the C `generic.data`), `data_count_bit` 88,
//! `serial` (28 bits), `button` (4 bits), `counter` = the low 16 bits of the 24-bit counter (the
//! full counter is `data[39:16]`, the mode `data[7:4]`), `extra` = `nat[8..11]` (checksum and the
//! two unread bytes, the C `tail`), `crc_valid` true. Key files: Protocol "Honda V7", Bit 88,
//! Key, Serial, Btn, Cnt (24 bits), Mode, CRC, Tail.
//!
//! No capture of this protocol exists in the test corpus: the port is validated against the C
//! with synthetic pulse sequences only.

use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (see `info.rs`): the C's `SubGhzProtocolFlag_433 | 315 | AM`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "Honda/Acura",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "Honda V7";
const TS: u32 = 250;
const TL: u32 = 480;
const TD: u32 = 100;
const FRAME_BITS: u16 = 88;
const MIN_BITS: u16 = 64;
const MIN_PRE: u16 = 20;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    Preamble,
    Data,
}

/// Honda V7 decoder (matches SubGhzProtocolDecoderHondaV7).
pub struct HondaV7Decoder {
    step: Step,
    pre: u16,
    bit_count: u16,
    last_high: u32,
    buf: [u8; 16],
}

/// `honda_v7_classify`: 1 short, 2 long, 0 neither.
fn classify(dur: u32) -> u8 {
    if dur.abs_diff(TS) <= TD {
        1
    } else if dur.abs_diff(TL) <= TD {
        2
    } else {
        0
    }
}

/// The check: XOR of the first 8 natural bytes.
fn xor8(nat: &[u8]) -> u8 {
    nat[..8].iter().fold(0, |a, &b| a ^ b)
}

/// Fields of the 8 natural bytes `data` (big-endian) and the 24-bit `tail` (checksum first).
fn fields(data: u64, tail: u64) -> DecodedSignal {
    let nat = data.to_be_bytes();
    let counter = (u32::from(nat[4]) << 16) | (u32::from(nat[5]) << 8) | u32::from(nat[6]);
    DecodedSignal {
        serial: Some((u32::from(nat[0] & 0x0F) << 24) | (u32::from(nat[1]) << 16) | (u32::from(nat[2]) << 8) | u32::from(nat[3])),
        button: Some(nat[0] >> 4),
        counter: Some(counter as u16),
        crc_valid: xor8(&nat) == (tail >> 16) as u8,
        data,
        data_count_bit: usize::from(FRAME_BITS),
        encoder_capable: false,
        extra: Some(tail & 0xFF_FFFF),
        protocol_display_name: None,
    }
}

impl HondaV7Decoder {
    pub fn new() -> Self {
        Self { step: Step::Reset, pre: 0, bit_count: 0, last_high: 0, buf: [0; 16] }
    }

    /// `honda_v7_reset_acc`.
    fn reset_acc(&mut self) {
        self.step = Step::Reset;
        self.pre = 0;
        self.bit_count = 0;
        self.last_high = 0;
        self.buf = [0; 16];
    }

    /// `honda_v7_validate_emit`.
    fn validate_emit(&self) -> Option<DecodedSignal> {
        if self.bit_count < FRAME_BITS {
            return None;
        }
        let mut nat = [0u8; 11];
        for (n, b) in nat.iter_mut().zip(&self.buf) {
            *n = b.reverse_bits();
        }
        let button = nat[0] >> 4;
        let serial = (u32::from(nat[0] & 0x0F) << 24) | (u32::from(nat[1]) << 16) | (u32::from(nat[2]) << 8) | u32::from(nat[3]);
        if button == 0 || button > 0x9 {
            return None;
        }
        if serial == 0 || serial == 0x0FFF_FFFF {
            return None;
        }
        if xor8(&nat) != nat[8] {
            return None;
        }
        let data = nat[..8].iter().fold(0u64, |a, &b| (a << 8) | u64::from(b));
        let tail = (u64::from(nat[8]) << 16) | (u64::from(nat[9]) << 8) | u64::from(nat[10]);
        Some(fields(data, tail))
    }
}

impl Default for HondaV7Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for HondaV7Decoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: TS, te_long: TL, te_delta: TD, min_count_bit: usize::from(FRAME_BITS) }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.reset_acc();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let c = classify(duration);
        match self.step {
            Step::Reset => {
                if level && c == 1 {
                    self.last_high = duration;
                    self.pre = 0;
                    self.step = Step::Preamble;
                }
                None
            }
            Step::Preamble => {
                if !level {
                    if classify(self.last_high) == 1 && c == 1 {
                        self.pre += 1;
                    } else if c == 0 {
                        if self.pre >= MIN_PRE {
                            self.bit_count = 0;
                            self.buf = [0; 16];
                            self.step = Step::Data;
                        } else {
                            self.reset_acc();
                        }
                    } else if self.pre >= MIN_PRE && classify(self.last_high) == 2 && c == 2 {
                        self.bit_count = 0;
                        self.buf = [0; 16];
                        self.buf[0] |= 0x01;
                        self.bit_count = 1;
                        self.step = Step::Data;
                    } else {
                        self.reset_acc();
                    }
                } else {
                    self.last_high = duration;
                }
                None
            }
            Step::Data => {
                if level {
                    self.last_high = duration;
                    return None;
                }
                let hc = classify(self.last_high);
                if hc == 1 && c == 1 {
                    if self.bit_count < FRAME_BITS {
                        self.bit_count += 1;
                    }
                } else if hc == 2 && c == 2 {
                    if self.bit_count < FRAME_BITS {
                        let bi = usize::from(self.bit_count >> 3);
                        if bi < 16 {
                            self.buf[bi] |= 1 << (self.bit_count & 7);
                        }
                        self.bit_count += 1;
                    }
                } else {
                    let out = if self.bit_count >= MIN_BITS { self.validate_emit() } else { None };
                    self.reset_acc();
                    return out;
                }
                if self.bit_count >= FRAME_BITS {
                    let out = self.validate_emit();
                    self.reset_acc();
                    return out;
                }
                None
            }
        }
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }

    /// `subghz_protocol_decoder_honda_v7_deserialize` (the checksum is re-verified).
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != usize::from(FRAME_BITS) {
            return None;
        }
        let tail = u64::from(key.u32_field("Tail")?);
        let d = fields(key.key_u64()?, tail);
        let crc = u64::from(key.u32_field("CRC")?);
        (crc == tail >> 16).then_some(d)
    }

    /// `subghz_protocol_decoder_honda_v7_serialize`.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != usize::from(FRAME_BITS) {
            return None;
        }
        let tail = decoded.extra?;
        let nat = decoded.data.to_be_bytes();
        let counter = (u32::from(nat[4]) << 16) | (u32::from(nat[5]) << 8) | u32::from(nat[6]);
        let mut key = FlipperKey::for_protocol(NAME, usize::from(FRAME_BITS), decoded.data);
        key.push_u32("Serial", decoded.serial?);
        key.push_u32("Btn", decoded.button?.into());
        key.push_u32("Cnt", counter);
        key.push_u32("Mode", u32::from(nat[7] >> 4));
        key.push_u32("CRC", (tail >> 16) as u32);
        key.push_u32("Tail", tail as u32);
        Some(key)
    }
}

#[cfg(test)]
mod tests {
    use super::super::honda_synth::{decode, dump, Pulse};
    use super::*;

    const PRESET: &str = "FuriHalSubGhzPresetOok650Async";

    /// Natural bytes for the made-up fields, with a valid checksum (nat[8]) and 2 filler bytes.
    fn natural(button: u8, serial: u32, counter: u32, mode: u8) -> [u8; 11] {
        let mut n = [0u8; 11];
        n[0] = (button << 4) | ((serial >> 24) & 0x0F) as u8;
        n[1] = (serial >> 16) as u8;
        n[2] = (serial >> 8) as u8;
        n[3] = serial as u8;
        n[4] = (counter >> 16) as u8;
        n[5] = (counter >> 8) as u8;
        n[6] = counter as u8;
        n[7] = mode << 4;
        n[8] = xor8(&n);
        n[9] = 0xA5;
        n[10] = 0x3C;
        n
    }

    /// Preamble of `pre` short pairs, then 88 bits (bit i of the buffer = bit (i % 8) of the
    /// bit-reversed natural byte, i.e. natural bits MSB first), then a closing long gap.
    fn burst(nat: &[u8; 11], pre: usize, bits: usize) -> Vec<Pulse> {
        let mut p: Vec<Pulse> = Vec::new();
        for _ in 0..pre {
            p.push((true, 250));
            p.push((false, 250));
        }
        // An invalid LOW ends the preamble (the C starts the data on it, bit count 0).
        p.push((true, 250));
        p.push((false, 2000));
        for i in 0..bits {
            let byte = nat[i / 8].reverse_bits();
            let one = (byte >> (i % 8)) & 1 == 1;
            let w = if one { 480 } else { 250 };
            p.push((true, w));
            p.push((false, w));
        }
        p.push((true, 250));
        p.push((false, 9000));
        p
    }

    #[test]
    fn honda_v7_decodes_a_synthetic_frame() {
        let nat = natural(0x5, 0x0ABC_DEF1, 0x01_2345, 0xB);
        let pulses = burst(&nat, 24, 88);
        dump("honda_v7_good", &pulses, PRESET);
        let got = decode(&mut HondaV7Decoder::new(), &pulses);
        assert_eq!(got.len(), 1);
        let d = &got[0];
        assert_eq!((d.serial, d.button, d.counter), (Some(0x0ABC_DEF1), Some(5), Some(0x2345)));
        assert_eq!((d.data_count_bit, d.crc_valid), (88, true));
        assert_eq!(d.data, u64::from_be_bytes(nat[..8].try_into().unwrap()));
        assert_eq!(d.extra, Some((u64::from(nat[8]) << 16) | 0xA53C));
        assert!(!HondaV7Decoder::new().supports_encoding());
    }

    #[test]
    fn honda_v7_gating() {
        let nat = natural(0x5, 0x0ABC_DEF1, 0x01_2345, 0xB);
        let mut bad = nat;
        bad[8] ^= 0x01; // checksum
        let p = burst(&bad, 24, 88);
        dump("honda_v7_bad_checksum", &p, PRESET);
        assert!(decode(&mut HondaV7Decoder::new(), &p).is_empty());
        let p = burst(&nat, 10, 88);
        dump("honda_v7_short_preamble", &p, PRESET);
        assert!(decode(&mut HondaV7Decoder::new(), &p).is_empty());
        let p = burst(&nat, 24, 70);
        dump("honda_v7_short_frame", &p, PRESET);
        assert!(decode(&mut HondaV7Decoder::new(), &p).is_empty());
        // Button 0 and button 0xA are not accepted even with a valid checksum.
        for b in [0u8, 0xA] {
            let p = burst(&natural(b, 0x0ABC_DEF1, 1, 1), 24, 88);
            assert!(decode(&mut HondaV7Decoder::new(), &p).is_empty(), "button {b}");
        }
        let p = burst(&natural(5, 0, 1, 1), 24, 88);
        assert!(decode(&mut HondaV7Decoder::new(), &p).is_empty());
    }

    #[test]
    fn honda_v7_key_file_round_trip() {
        use crate::protocols::flipper_key::testing::assert_round_trip;
        let nat = natural(0x9, 0x0123_4567, 0xFE_DCBA, 0x3);
        let data = u64::from_be_bytes(nat[..8].try_into().unwrap());
        let tail = (u64::from(nat[8]) << 16) | (u64::from(nat[9]) << 8) | u64::from(nat[10]);
        let d = fields(data, tail);
        let text = assert_round_trip(&HondaV7Decoder::new(), &d);
        assert!(text.contains("Protocol: Honda V7\nBit: 88\n"), "{text}");
        assert!(text.contains("Cnt: 16702650\n"), "{text}");
    }
}
