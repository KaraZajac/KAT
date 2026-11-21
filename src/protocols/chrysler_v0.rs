//! Chrysler V0 protocol decoder/encoder
//!
//! The signal and the key files are ProtoPirate's `protocols/chrysler_v0.c` (HEAD); Flipper-ARF
//! ships the same file as `lib/subghz/protocols/chrysler.c` ("Chrysler", FOBIK GQ43VT) and
//! doctor-firmware as `chrysler_v0.c`, with the same decoder. Used by older Chrysler/Dodge/Jeep
//! FOBIK keyfobs. (The newer FCA fobs in `IMPORTS/chrysler` and `IMPORTS/jeep` are Fiat V2 frames;
//! no reference's Chrysler V0 decodes them, in either polarity.) **KAT reads the frame by the rule
//! it is built by, not by the references' model** (below; `docs/chrysler_v0.md`).
//!
//! **Signal** (the C's): PWM with a short HIGH pulse and TWO long-LOW symbols. A "1" bit is
//! HIGH≈600µs (te_one_short) + LOW≈3400µs (te_long_a); a "0" bit is HIGH≈300µs (te_short) +
//! LOW≈3700µs (te_long_b). te_delta≈150, long_delta≈400, te_gap≈8000, frame_gap≈15600. A press of
//! the fob is 24 preamble pairs (short HIGH + long_b LOW), then twice a short HIGH, the 15.6 ms
//! frame gap and the 80 bits; it ends at the second copy's last bit. The C's state machine, which
//! KAT keeps, reads the copy behind the preamble.
//!
//! **Frame**: 80 bits, the first 64 in `data`, the last 16 (the C's `Key_2`) in `extra`. Each byte
//! is sent LSB first. Byte 0 is the sum (mod 256) of `n` and `p1..p8`; byte 1 is `n`, the low byte
//! of a 16-bit counter that goes down by one with every press; bytes 2..9 are `p1 ^ n` … `p8 ^ n`.
//! When the sum is odd, bytes 1..9 go out backwards, the 72 bits last bit first (the C's payload
//! B; an even sum is its payload A). In the one fob recorded, the Chrysler PT Cruiser 2006's
//! (`IMPORTS/chrysler`: 732 frames in six recordings), `p1..p4` are `FD 3B A1 C3`, `p5` is the
//! button (`20` Lock, `10` Unlock), `p6` the counter's high byte and `p7 p8` `A1 C3` again. Every
//! one of its frames is built so, and each press re-encoded is the next press it sent, all 80 bits
//! (tests below). [Volvo S40](super::volvo_s40) reads the same rule in a 64-bit frame.
//!
//! **Fields**: `serial` = `p1..p4`, `button` = `p5` (named for the two codes the fob sends; any
//! other is "Other"), `counter` = `p6 << 8 | n`, `crc_valid` = the sum. The decoder reports a frame
//! whose sum holds and whose `p1..p4` are not `00000000` or `FFFFFFFF`.
//!
//! **Where KAT departs from the references.** The C reads byte 0 as a 6-bit seed
//! (`reverse6(key[0] >> 2)`: the sum's low six bits) and two header bits (the sum's top two,
//! backwards), and bytes 1..9 under one of sixteen XOR masks (`xor_table[seed & 0x0F]`, a nibble of
//! it flipped for Lock) that stand in for `n` over runs of presses. So its serials (SnA, plaintext
//! bytes 0..3 of payload A; SnB, bytes 0, 1, 2 and 7 of payload B) change with the count, 160 for
//! the one fob; its counter (`Cnt`) is the seed; its checks (A: `p0 ^ 0xC3 == p4` and `p0 ^ p5`
//! 0x04 / 0x08; B: `p0 == p4` and `p0 ^ p5 == 0x62`) hold for that fob's bytes and fail for a fob
//! whose `p3` or `p4` differ; and it does not test the sum. KAT reads a frame whose sum holds,
//! whatever its fixed bytes, and none whose sum fails (one the C reads when a bit its checks do not
//! cover is wrong).
//!
//! **Encoder**: the fob's next press. The counter one lower (borrowing from `p6`), `p5` the
//! requested button (KAT's Lock and Unlock: `20` and `10`; Trunk and Panic, whose codes no
//! recording shows, keep the captured one), the other bytes as captured, the new sum and the order
//! it gives; sent as the fob sends a press, once (`tx_copies.rs`: the upload is a whole press). The
//! references build payload A from the captured plaintext with the seed + 1 (ProtoPirate's emulate
//! scene, doctor-firmware: the captured press again) or + 2 (Flipper-ARF: an earlier press), then
//! payload B from A's plaintext, a frame no fob sends.
//!
//! **Key files** (`subghz_protocol_decoder_chrysler_v0_serialize` / `_deserialize`; Flipper-ARF's
//! "Chrysler" writes the same lines): the C's, written with the C's model, since the firmwares read
//! `Plain_x`, `Btn` and `Cnt` back when they send from a file: `Protocol: Chrysler V0`, `Bit: 80`,
//! `Key` = the first 64 frame bits, then `Key_2` (the last 16 bits, big-endian), `Plain_A` (frame
//! A) or `Plain_B` (frame B) — the C's 9 plaintext bytes — and `Serial` (SnA / SnB), `Btn` (1 Lock,
//! 2 Unlock), `Cnt` (the 6-bit seed). Two reference quirks are not copied: the C writes
//! Key_2/Plain_x after `flipper_format_rewind`, i.e. over the start of its own buffer (KAT writes
//! them after Key, the order the C means and doctor-firmware's fixed copy writes), and its decoder
//! instance keeps the plaintext of earlier frames — even ones that failed the check — so a
//! reference file may also carry the other frame's Plain_x, with that frame's SnB as Serial (KAT
//! writes what a fresh decoder writes). Loading (`_deserialize`) reads Key + Key_2 (required) with
//! any Bit (the C forces 80) and reads the frame by the rule; the file's other lines are not
//! applied. A frame whose sum fails loads with `crc_valid` false (the C loads frames whose check
//! fails too, shown as `Chk: ERR`) and is not sent on.

use super::{ProtocolDecoder, ProtocolTiming, DecodedSignal, FlipperKey};
use crate::radio::demodulator::LevelDuration;
use crate::duration_diff;

use super::info::prelude::*;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Chrysler V0",
    decoder: "Chrysler V0",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "Chrysler/Dodge/Jeep",
    category: Category::Vehicle,
    buttons: Some(buttons),
    presets: &[&AM650],
};

/// The button byte `p5` of the one fob recorded: Lock and Unlock.
const LOCK: u8 = 0x20;
const UNLOCK: u8 = 0x10;

/// Names of the button byte: the two codes the recorded fob sends; any other is not known.
fn buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        LOCK => "Lock",
        UNLOCK => "Unlock",
        _ => "Other",
    })
}

const TE_SHORT: u32 = 0x12C; // 300
const TE_DELTA: u32 = 0x96; // 150
const TE_LONG_A: u32 = 0xD48; // 3400
const TE_LONG_B: u32 = 0xE74; // 3700
const TE_LONG_DELTA: u32 = 0x190; // 400
const TE_GAP: u32 = 0x1F40; // 8000
const TE_ONE_SHORT: u32 = 0x258; // 600
const FRAME_GAP: u32 = 0x3CF0; // 15600
const PREAMBLE_PAIRS: usize = 24;
const DECODE_BIT_COUNT: usize = 0x50; // 80

/// The rule read back from `frame`, the sum byte and the bytes behind it as received (MSB first),
/// of any length (Volvo S40's frame is 8 bytes): the counter byte `n`, the plaintext bytes behind
/// it, and whether the sum holds. None for a frame of fewer than two bytes. Each byte is sent LSB
/// first; when the sum is odd the bytes behind it go out backwards, so as received they are the
/// bytes in reverse order, each MSB first.
pub(super) fn unmask(frame: &[u8]) -> Option<(u8, Vec<u8>, bool)> {
    let (&first, body) = frame.split_first()?;
    let sum = first.reverse_bits();
    let bytes: Vec<u8> = if sum & 1 == 0 {
        body.iter().map(|b| b.reverse_bits()).collect()
    } else {
        body.iter().rev().copied().collect()
    };
    let (&n, rest) = bytes.split_first()?;
    let plain: Vec<u8> = rest.iter().map(|b| b ^ n).collect();
    let total = plain.iter().fold(n, |s, &b| s.wrapping_add(b));
    Some((n, plain, total == sum))
}

/// The rule: the frame of counter byte `n` and the plaintext bytes, as received (MSB first).
pub(super) fn mask(n: u8, plain: &[u8]) -> Vec<u8> {
    let sum = plain.iter().fold(n, |s, &b| s.wrapping_add(b));
    let natural: Vec<u8> = std::iter::once(n).chain(plain.iter().map(|b| b ^ n)).collect();
    let body: Vec<u8> = if sum & 1 == 0 {
        natural.iter().map(|b| b.reverse_bits()).collect()
    } else {
        natural.iter().rev().copied().collect()
    };
    std::iter::once(sum.reverse_bits()).chain(body).collect()
}

/// An 80-bit frame read by the rule.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Frame {
    /// The counter's low byte.
    n: u8,
    /// `p1..p8`: the fixed code, the button, the counter's high byte, two more fixed bytes.
    plain: [u8; 8],
    /// Whether byte 0 is the sum of `n` and `p1..p8`.
    sum_ok: bool,
}

impl Frame {
    /// The frame `data` (its first 64 bits) + `data_2` (the last 16).
    fn read(data: u64, data_2: u16) -> Self {
        let mut bytes = data.to_be_bytes().to_vec();
        bytes.extend(data_2.to_be_bytes());
        let (n, plain, sum_ok) = unmask(&bytes).expect("ten bytes");
        Self { n, plain: plain.try_into().expect("eight bytes"), sum_ok }
    }

    /// The bits the fob sends for `n` and `plain` (with their sum): the first 64 and the last 16.
    fn bits(&self) -> (u64, u16) {
        let b = mask(self.n, &self.plain);
        (u64::from_be_bytes(b[..8].try_into().expect("ten bytes")), u16::from_be_bytes([b[8], b[9]]))
    }

    fn serial(&self) -> u32 {
        u32::from_be_bytes([self.plain[0], self.plain[1], self.plain[2], self.plain[3]])
    }

    fn counter(&self) -> u16 {
        u16::from_be_bytes([self.plain[5], self.n])
    }

    /// The decoder reports the frame (and the encoder sends on from it): the sum holds and the ID
    /// is neither `00000000` nor `FFFFFFFF`.
    fn readable(&self) -> bool {
        self.sum_ok && self.serial() != 0 && self.serial() != u32::MAX
    }

    /// The fob's next press with button byte `button`: the counter one lower.
    fn next(&self, button: u8) -> Self {
        let [hi, n] = self.counter().wrapping_sub(1).to_be_bytes();
        let mut plain = self.plain;
        plain[4] = button;
        plain[5] = hi;
        Self { n, plain, sum_ok: true }
    }
}

/// XOR table (chrysler_v0_xor_table) — indexed by `seed & 0x0F`. The C's model, for key files.
const XOR_TABLE: [u8; 16] = [
    0x0F, 0x02, 0x40, 0x0C, 0x30, 0x0E, 0x70, 0x08, 0x10, 0x0A, 0x50, 0xF4, 0x2F, 0xF6, 0x6F, 0xF0,
];

/// Decoder steps (matches Chrysler_V0DecoderStep)
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Seek,
    Data,
}

/// Chrysler V0 protocol decoder
pub struct ChryslerV0Decoder {
    step: DecoderStep,
    packet_bit_count: u16,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
    data_2: u16,
}

/// What the C's `decode_packet` makes of a frame (the key files' lines).
struct Decoded {
    /// The C's structural check (`Chk`); KAT does not gate on it.
    #[cfg_attr(not(test), allow(dead_code))]
    check_ok: bool,
    button: u8,
    /// Transform seed (reversed 6-bit counter); the C's `Cnt` field.
    seed: u8,
    /// Serial: SnA (payload A) or SnB (payload B).
    serial: u32,
}

impl ChryslerV0Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            packet_bit_count: 0,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
            data_2: 0,
        }
    }

    fn is_short(d: u32) -> bool {
        duration_diff!(d, TE_SHORT) <= TE_DELTA
    }

    fn is_long_mark(d: u32) -> bool {
        duration_diff!(d, TE_LONG_A) <= TE_LONG_DELTA || duration_diff!(d, TE_LONG_B) <= TE_LONG_DELTA
    }

    /// Reverse the low 6 bits of `value` (chrysler_v0_reverse6).
    fn reverse6(value: u32) -> u8 {
        let mut out: u8 = 0;
        let mut v = value;
        for _ in 0..6 {
            out = (out << 1) | ((v & 1) as u8);
            v >>= 1;
        }
        out
    }

    /// Transform 9 bytes with the seed-derived XOR mask (chrysler_v0_transform_block).
    /// `button == 1` (Lock) flips a nibble of the mask depending on the seed parity.
    fn transform_block(input: &[u8; 9], key: u8, button: u8) -> [u8; 9] {
        let mut mask = XOR_TABLE[(key & 0x0F) as usize];
        if button == 1 {
            mask ^= if (key & 1) != 0 { 0xF0 } else { 0x0F };
        }
        let mut out = [0u8; 9];
        for i in 0..9 {
            out[i] = input[i] ^ mask;
        }
        out
    }

    /// The C's model, port of chrysler_v0_decode_packet (for the key files' lines): seed, button,
    /// check and serial from the 64-bit `data` (payload bytes 0..7) plus the 16-bit `data_2`
    /// (payload bytes 8,9).
    fn decode_packet(data: u64, data_2: u16) -> Decoded {
        let key = data.to_be_bytes();
        let key2 = data_2;
        let seed = Self::reverse6((key[0] >> 2) as u32);

        let b1_xor_b6 = key[6] ^ key[1];
        let msb_set = (key[0] & 0x80) != 0;

        let mut check_ok;
        let mut button: u8;

        if msb_set {
            let key2_low = (key2 & 0xFF) as u8;
            check_ok = (key[1] == key[5]) && (b1_xor_b6 == 0x62);
            button = if (key2_low ^ key[4]) == 0x10 { 2 } else { 1 };
        } else {
            check_ok = false;
            button = 1;

            if (key[1] ^ 0xC3) == key[5] {
                if b1_xor_b6 == 0x04 {
                    check_ok = true;
                } else {
                    check_ok = b1_xor_b6 == 0x08;
                    if b1_xor_b6 == 0x08 {
                        button = 2;
                    }
                }
            } else if b1_xor_b6 == 0x08 {
                button = 2;
            }
            // (b1_xor_b6 == 0x04 with mismatched key[5]: button stays 1, check_ok stays false.)
        }

        let decoded = Self::transform_block(&Self::encoded(data, data_2), seed, button);

        let serial = if (seed & 1) != 0 {
            // Payload B: SnB = plaintext bytes 0, 1, 2, 7.
            ((decoded[0] as u32) << 24)
                | ((decoded[1] as u32) << 16)
                | ((decoded[2] as u32) << 8)
                | (decoded[7] as u32)
        } else {
            // Payload A: SnA = plaintext bytes 0..3 (the C keeps it in generic.cnt).
            ((decoded[0] as u32) << 24)
                | ((decoded[1] as u32) << 16)
                | ((decoded[2] as u32) << 8)
                | (decoded[3] as u32)
        };

        Decoded {
            check_ok,
            button,
            seed,
            serial,
        }
    }

    /// The nine frame bytes behind byte 0, as received (`encoded` in the C).
    fn encoded(data: u64, data_2: u16) -> [u8; 9] {
        let key = data.to_be_bytes();
        [key[1], key[2], key[3], key[4], key[5], key[6], key[7], (data_2 >> 8) as u8, data_2 as u8]
    }

    /// The C's 9 plaintext bytes of the frame (`decoded` in `chrysler_v0_decode_packet`: Plain_A
    /// for an even seed, Plain_B for an odd one).
    fn plaintext(data: u64, data_2: u16) -> [u8; 9] {
        let d = Self::decode_packet(data, data_2);
        Self::transform_block(&Self::encoded(data, data_2), d.seed, d.button)
    }

    /// Build a DecodedSignal from a committed frame, if the decoder reports it.
    fn commit(&self) -> Option<DecodedSignal> {
        let s = Self::frame_signal(self.decode_data, self.data_2);
        s.encoder_capable.then_some(s)
    }

    /// The decode of frame `data` + `data_2` by the rule (`crc_valid` = the sum).
    fn frame_signal(data: u64, data_2: u16) -> DecodedSignal {
        let f = Frame::read(data, data_2);
        DecodedSignal {
            serial: Some(f.serial()),
            button: Some(f.plain[4]),
            counter: Some(f.counter()),
            crc_valid: f.sum_ok,
            // 80-bit frame: report the most-significant 64 bits (payload bytes 0..7).
            data,
            data_count_bit: DECODE_BIT_COUNT,
            encoder_capable: f.readable(),
            // The low 16 bits (data_2 = payload bytes 8,9), for the encoder and the key file.
            extra: Some(u64::from(data_2)),
            protocol_display_name: None,
        }
    }

    /// The button byte for KAT's `button`: Lock (0x01) and Unlock (0x02) are the recorded fob's
    /// `20` and `10`, either native code itself; the captured code, and any other button (Trunk,
    /// Panic: their codes are not known), keep the captured byte.
    fn button_byte(button: u8, captured: u8) -> u8 {
        match button {
            _ if button == captured => captured,
            0x01 | LOCK => LOCK,
            0x02 | UNLOCK => UNLOCK,
            _ => captured,
        }
    }

    /// Read one bit out of an MSB-first payload (chrysler_v0_payload_get_bit).
    fn payload_get_bit(payload: &[u8; 10], index: u8) -> u8 {
        let byte = payload[(index >> 3) as usize];
        let shift = 7 - (index & 7);
        (byte >> shift) & 1
    }

    /// ADD_LEVEL-style merge: combine adjacent same-level pulses.
    fn add_level(signal: &mut Vec<LevelDuration>, level: bool, duration: u32) {
        if let Some(last) = signal.last_mut() {
            if last.level == level {
                *last = LevelDuration::new(level, last.duration_us + duration);
                return;
            }
        }
        signal.push(LevelDuration::new(level, duration));
    }

    /// Emit one 80-bit PWM payload frame (without the leading preamble).
    fn emit_payload(signal: &mut Vec<LevelDuration>, payload: &[u8; 10]) {
        for bit in 0..80u8 {
            let value = Self::payload_get_bit(payload, bit);
            if value != 0 {
                Self::add_level(signal, true, TE_ONE_SHORT);
                Self::add_level(signal, false, TE_LONG_A);
            } else {
                Self::add_level(signal, true, TE_SHORT);
                Self::add_level(signal, false, TE_LONG_B);
            }
        }
    }

    /// Emit the preamble (24 short-HIGH + long_b-LOW pairs).
    fn emit_preamble(signal: &mut Vec<LevelDuration>) {
        for _ in 0..PREAMBLE_PAIRS {
            Self::add_level(signal, true, TE_SHORT);
            Self::add_level(signal, false, TE_LONG_B);
        }
    }

    /// One press as the fob sends it: the preamble, then twice a short HIGH, the frame gap and the
    /// 80 bits; the frame gap closes it (the fob's last bit runs into silence). 372 pulses.
    fn press(data: u64, data_2: u16) -> Vec<LevelDuration> {
        let mut payload = [0u8; 10];
        payload[..8].copy_from_slice(&data.to_be_bytes());
        payload[8..].copy_from_slice(&data_2.to_be_bytes());
        let mut signal = Vec::with_capacity(PREAMBLE_PAIRS * 2 + 2 * (2 + 160));
        Self::emit_preamble(&mut signal);
        for _ in 0..2 {
            Self::add_level(&mut signal, true, TE_SHORT);
            Self::add_level(&mut signal, false, FRAME_GAP);
            Self::emit_payload(&mut signal, &payload);
        }
        Self::add_level(&mut signal, false, FRAME_GAP);
        signal
    }
}

impl ProtocolDecoder for ChryslerV0Decoder {
    fn name(&self) -> &'static str {
        "Chrysler V0"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG_A,
            te_delta: TE_DELTA,
            min_count_bit: DECODE_BIT_COUNT,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.packet_bit_count = 0;
        self.te_last = 0;
        self.decode_data = 0;
        self.decode_count_bit = 0;
        self.data_2 = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::Reset => {
                if level && Self::is_short(duration) {
                    self.packet_bit_count = 0;
                    self.te_last = duration;
                    self.step = DecoderStep::Seek;
                }
            }

            DecoderStep::Seek => {
                if level {
                    self.te_last = duration;
                    return None;
                }

                if Self::is_long_mark(duration) {
                    if Self::is_short(self.te_last) {
                        self.packet_bit_count += 1;
                    } else if self.packet_bit_count > 0x0F {
                        self.data_2 = 0;
                        self.step = DecoderStep::Data;
                        self.decode_data = 1;
                        self.decode_count_bit = 1;
                    } else {
                        self.packet_bit_count = 0;
                        self.step = DecoderStep::Seek;
                    }
                    return None;
                }

                if duration > TE_GAP && self.packet_bit_count > 0x0F {
                    self.decode_data = 0;
                    self.data_2 = 0;
                    self.decode_count_bit = 0;
                    self.step = DecoderStep::Data;
                    return None;
                }

                self.step = DecoderStep::Reset;
                self.packet_bit_count = 0;
            }

            DecoderStep::Data => {
                if level {
                    self.te_last = duration;
                    return None;
                }

                let count = self.decode_count_bit;

                if duration > TE_GAP {
                    let result = if count as usize > 0x4F { self.commit() } else { None };
                    self.step = DecoderStep::Reset;
                    self.packet_bit_count = 0;
                    return result;
                }

                let bit_value: u8;
                if self.te_last < TE_SHORT {
                    if !Self::is_short(self.te_last) || !Self::is_long_mark(duration) {
                        let result = if count as usize > 0x4F { self.commit() } else { None };
                        self.step = DecoderStep::Reset;
                        self.packet_bit_count = 0;
                        return result;
                    }
                    bit_value = 1;
                } else {
                    if self.te_last > 0x2EE || !Self::is_long_mark(duration) {
                        let result = if count as usize > 0x4F { self.commit() } else { None };
                        self.step = DecoderStep::Reset;
                        self.packet_bit_count = 0;
                        return result;
                    }
                    bit_value = if Self::is_short(self.te_last) { 1 } else { 0 };
                }

                let bit = (bit_value ^ 1) as u64;
                let new_count = count.wrapping_add(1);
                if count <= 0x3F {
                    self.decode_data = (self.decode_data << 1) | bit;
                    self.decode_count_bit = new_count;
                    return None;
                }

                self.data_2 = (self.data_2 << 1) | (bit as u16);
                self.decode_count_bit = new_count;
                if new_count as usize != DECODE_BIT_COUNT {
                    return None;
                }

                let result = self.commit();
                self.decode_data = 0;
                self.data_2 = 0;
                self.decode_count_bit = 0;
                self.step = DecoderStep::Reset;
                self.packet_bit_count = 0;
                return result;
            }
        }

        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    /// The fob's next press (see the module doc, "Encoder").
    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != DECODE_BIT_COUNT {
            return None;
        }
        let frame = Frame::read(decoded.data, u16::try_from(decoded.extra?).ok()?);
        if !frame.readable() {
            return None;
        }
        let (data, data_2) = frame.next(Self::button_byte(button, frame.plain[4])).bits();
        Some(Self::press(data, data_2))
    }

    /// `subghz_protocol_decoder_chrysler_v0_deserialize` (see the module doc, "Key files").
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        let data = key.key_u64()?;
        let key2: [u8; 2] = key.hex_bytes_field("Key_2")?.try_into().ok()?;
        Some(Self::frame_signal(data, u16::from_be_bytes(key2)))
    }

    /// `subghz_protocol_decoder_chrysler_v0_serialize`: the C's lines, from the C's model (see the
    /// module doc, "Key files").
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != DECODE_BIT_COUNT {
            return None;
        }
        let data_2 = u16::try_from(decoded.extra?).ok()?;
        let d = Self::decode_packet(decoded.data, data_2);
        let mut key = FlipperKey::for_protocol("Chrysler V0", DECODE_BIT_COUNT, decoded.data);
        key.push_hex_bytes("Key_2", &data_2.to_be_bytes());
        let plain = if d.seed & 1 != 0 { "Plain_B" } else { "Plain_A" };
        key.push_hex_bytes(plain, &Self::plaintext(decoded.data, data_2));
        key.push_u32("Serial", d.serial);
        key.push_u32("Btn", d.button.into());
        key.push_u32("Cnt", d.seed.into());
        Some(key)
    }
}

impl Default for ChryslerV0Decoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::super::f_common::test_support::*;
    use super::*;

    /// The first two presses of `IMPORTS/chrysler/Chrysler PT Cruiser 2006 close.sub` (`data`,
    /// `extra`): counters 3B85 and 3B84, sums E0 (even: sent forwards, the C's payload A) and DF
    /// (odd: backwards, payload B).
    const CLOSE_1: (u64, u16) = (0x07A1_1E7D_2462_A57D, 0x2462);
    const CLOSE_2: (u64, u16) = (0xFB47_25BF_A447_25BF, 0x7984);

    /// The PT Cruiser fob's bytes behind `n` for a button and the counter's high byte.
    fn pt(button: u8, hi: u8) -> [u8; 8] {
        [0xFD, 0x3B, 0xA1, 0xC3, button, hi, 0xA1, 0xC3]
    }

    /// The frame the rule makes of `n` and `plain`, as a decode.
    fn signal(n: u8, plain: &[u8; 8]) -> DecodedSignal {
        let (data, data_2) = Frame { n, plain: *plain, sum_ok: true }.bits();
        ChryslerV0Decoder::frame_signal(data, data_2)
    }

    fn data_2(d: &DecodedSignal) -> u16 {
        d.extra.unwrap() as u16
    }

    /// reverse6 is its own inverse on 6-bit values.
    #[test]
    fn reverse6_is_involution() {
        for v in 0u32..64 {
            let r = ChryslerV0Decoder::reverse6(v) as u32;
            assert_eq!(ChryslerV0Decoder::reverse6(r) as u32, v, "reverse6 not involutive at {v}");
        }
    }

    /// transform_block is its own inverse for a given (seed, button) — XOR with a constant mask.
    #[test]
    fn transform_block_round_trips() {
        let plain: [u8; 9] = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99];
        for &button in &[1u8, 2u8] {
            for seed in 0u8..64 {
                let enc = ChryslerV0Decoder::transform_block(&plain, seed, button);
                let dec = ChryslerV0Decoder::transform_block(&enc, seed, button);
                assert_eq!(dec, plain, "transform not invertible seed={seed} button={button}");
            }
        }
    }

    /// Two recorded frames by the rule (`docs/chrysler_v0.md`, "Frame"), and what the C makes of
    /// them: payloads A and B, seeds 20 and 1F, serials A11E7D24 and 4725BF79.
    #[test]
    fn two_recorded_frames() {
        for ((data, data_2), n, sum_odd, c_seed, c_serial) in
            [(CLOSE_1, 0x85u8, false, 0x20u8, 0xA11E_7D24u32), (CLOSE_2, 0x84, true, 0x1F, 0x4725_BF79)]
        {
            let f = Frame::read(data, data_2);
            assert_eq!((f.n, f.plain, f.sum_ok), (n, pt(LOCK, 0x3B), true));
            assert_eq!(data >> 63 == 1, sum_odd, "the sum's lowest bit is sent first");
            assert_eq!(f.bits(), (data, data_2));
            let d = ChryslerV0Decoder::frame_signal(data, data_2);
            assert_eq!(
                (d.serial, d.button, d.counter, d.crc_valid, d.encoder_capable),
                (Some(0xFD3B_A1C3), Some(LOCK), Some(0x3B00 | u16::from(n)), true, true)
            );
            let c = ChryslerV0Decoder::decode_packet(data, data_2);
            assert_eq!((c.check_ok, c.seed, c.serial, c.button), (true, c_seed, c_serial, 1));
        }
    }

    /// Frames of any count and of other fixed bytes read back, sent either way round, and one bit
    /// changed anywhere fails the sum.
    #[test]
    fn the_rule_reads_back() {
        for counter in [0u16, 1, 0x00FF, 0x0100, 0x3B84, 0x3B85, 0x8000, 0xFFFF] {
            let [hi, n] = counter.to_be_bytes();
            for plain in [pt(LOCK, hi), pt(UNLOCK, hi), [0x12, 0x34, 0x56, 0x78, 0x40, hi, 0x9A, 0xBC]] {
                let d = signal(n, &plain);
                let serial = u32::from_be_bytes([plain[0], plain[1], plain[2], plain[3]]);
                assert_eq!((d.serial, d.button, d.counter, d.crc_valid), (Some(serial), Some(plain[4]), Some(counter), true));
                for bit in 0..80 {
                    let (x, y) = if bit < 16 { (d.data, data_2(&d) ^ 1 << bit) } else { (d.data ^ 1 << (bit - 16), data_2(&d)) };
                    assert!(!Frame::read(x, y).sum_ok, "{counter:04X} {plain:02X?}: bit {bit}");
                }
            }
        }
    }

    /// The press `press` builds of `frame`.
    fn press_of(frame: (u64, u16)) -> Vec<LevelDuration> {
        ChryslerV0Decoder::press(frame.0, frame.1)
    }

    /// What the decoder reports: a frame whose sum holds, of any fixed bytes (another fob's, which
    /// the C's checks reject), and not one whose sum fails (a recorded frame with a bit the C's
    /// checks do not cover changed: the C reads it, with another serial), nor an ID of 00000000 or
    /// FFFFFFFF.
    #[test]
    fn what_the_decoder_reports() {
        let read = |frame: (u64, u16)| feed_all(&mut ChryslerV0Decoder::new(), &press_of(frame));
        let got = read(CLOSE_1);
        assert_eq!(got, [ChryslerV0Decoder::frame_signal(CLOSE_1.0, CLOSE_1.1)], "the copy behind the preamble, once");
        assert_eq!(feed_resetting(&mut ChryslerV0Decoder::new(), &press_of(CLOSE_1)), got);
        // byte 2 (the C's plaintext byte 1): no check of the C's covers it
        let bad = (CLOSE_1.0 ^ 1 << 47, CLOSE_1.1);
        let c = ChryslerV0Decoder::decode_packet(bad.0, bad.1);
        assert!(c.check_ok && c.serial != 0xA11E_7D24, "the C reads it, with another serial");
        assert!(read(bad).is_empty());
        // another fob's bytes: the sum holds, the C's checks fail
        let other = signal(0x42, &[0x12, 0x34, 0x56, 0x78, LOCK, 0x05, 0x9A, 0xBC]);
        assert!(!ChryslerV0Decoder::decode_packet(other.data, data_2(&other)).check_ok);
        assert_eq!(read((other.data, data_2(&other))), [other]);
        // the ID 00000000 and FFFFFFFF
        for id in [0u8, 0xFF] {
            let s = signal(0x42, &[id, id, id, id, LOCK, 0x05, 0xA1, 0xC3]);
            assert!(s.crc_valid && !s.encoder_capable);
            assert!(read((s.data, data_2(&s))).is_empty());
        }
    }

    /// The recordings of the PT Cruiser fob, the presses each holds and the button its name gives.
    const RECORDINGS: [(&str, u8, usize); 6] = [
        ("Chrysler PT Cruiser 2006 open.sub", UNLOCK, 10),
        ("Chrysler PT Cruiser 2006 close.sub", LOCK, 10),
        ("more/PT_300xLCK.sub", LOCK, 308),
        ("more/PT_300xUNL.sub", UNLOCK, 28),
        ("more/PT_LCK100.sub", LOCK, 105),
        ("more/PT_LCK250.sub", LOCK, 271),
    ];

    /// Each recording's presses, in order; None without IMPORTS.
    fn recorded() -> Option<Vec<(&'static str, u8, Vec<DecodedSignal>)>> {
        let mut out = Vec::new();
        for (rel, button, count) in RECORDINGS {
            let path = std::path::Path::new("IMPORTS/chrysler").join(rel);
            if !path.is_file() {
                eprintln!("Skip: IMPORTS not found (run from crate root)");
                return None;
            }
            let (_, wire) = sub_pulses(&path);
            let got = feed_resetting(&mut ChryslerV0Decoder::new(), &wire);
            assert_eq!(got.len(), count, "{rel}");
            out.push((rel, button, got));
        }
        Some(out)
    }

    /// Every frame of the PT Cruiser fob (`IMPORTS/chrysler`, 732 in six recordings) follows the
    /// rule with `FD 3B A1 C3`, the recording's button, the counter's high byte and `A1 C3` again;
    /// the counter goes down from press to press (by one, or by as many as the recording missed).
    /// KAT shows one serial where the C's model shows 160.
    #[test]
    fn the_pt_cruiser_recordings() {
        let Some(files) = recorded() else { return };
        let mut c_serials = std::collections::HashSet::new();
        let mut frames = 0;
        for (rel, button, got) in &files {
            for (i, d) in got.iter().enumerate() {
                let f = Frame::read(d.data, data_2(d));
                assert!(f.sum_ok, "{rel}");
                assert_eq!(f.plain, pt(*button, f.plain[5]), "{rel}");
                assert_eq!((d.serial, d.button), (Some(0xFD3B_A1C3), Some(*button)), "{rel}");
                if i > 0 {
                    assert!(d.counter < got[i - 1].counter, "{rel}: the counter goes down");
                }
                c_serials.insert(ChryslerV0Decoder::decode_packet(d.data, data_2(d)).serial);
                frames += 1;
            }
        }
        assert_eq!(frames, 732);
        assert_eq!(c_serials.len(), 160);
    }

    /// Re-encoding each press with its own button gives the next press the fob sent, all 80 bits,
    /// wherever the recording holds it (the counter one lower).
    #[test]
    fn reencode_reproduces_the_fobs_next_press() {
        let Some(files) = recorded() else { return };
        let mut pairs = 0;
        for (rel, _, got) in &files {
            for w in got.windows(2) {
                let (now, next) = (&w[0], &w[1]);
                if next.counter != Some(now.counter.unwrap().wrapping_sub(1)) {
                    continue;
                }
                let sent = feed_all(&mut ChryslerV0Decoder::new(), &ChryslerV0Decoder::new().encode(now, now.button.unwrap()).unwrap());
                assert_eq!(sent, [next.clone()], "{rel}");
                pairs += 1;
            }
        }
        assert_eq!(pairs, 721);
    }

    /// One counter for every button: the open recording's last press (Unlock, 3B86) and the close
    /// recording's first (Lock, 3B85) are one count apart, and that Unlock press re-encoded with
    /// Lock is the Lock press, all 80 bits.
    #[test]
    fn a_button_change_continues_the_count() {
        let Some(files) = recorded() else { return };
        let (last_open, first_close) = (files[0].2.last().unwrap(), &files[1].2[0]);
        assert_eq!((last_open.button, last_open.counter), (Some(UNLOCK), Some(0x3B86)));
        assert_eq!((first_close.button, first_close.counter), (Some(LOCK), Some(0x3B85)));
        assert_eq!((first_close.data, data_2(first_close)), CLOSE_1);
        let sent = feed_all(&mut ChryslerV0Decoder::new(), &ChryslerV0Decoder::new().encode(last_open, 0x01).unwrap());
        assert_eq!(sent, [first_close.clone()]);
    }

    /// The upload is a press as the fob sends it: pulse for pulse, by kind, the second press of the
    /// close recording (24 preamble pairs; a short carrier and the 15.6 ms gap before each of the
    /// two copies of the frame), ending at the frame gap.
    #[test]
    fn the_upload_is_the_fobs_press() {
        let path = std::path::Path::new("IMPORTS/chrysler/Chrysler PT Cruiser 2006 close.sub");
        if !path.is_file() {
            eprintln!("Skip: IMPORTS not found (run from crate root)");
            return;
        }
        let kind = |p: &LevelDuration| match (p.level, p.duration_us) {
            (true, 150..=450) => 's',
            (true, 451..=750) => 'l',
            (false, 3_000..=4_100) => 'p',
            (false, 12_000..=20_000) => 'g',
            _ => '?',
        };
        let up = ChryslerV0Decoder::new().encode(&ChryslerV0Decoder::frame_signal(CLOSE_1.0, CLOSE_1.1), 0x01).unwrap();
        assert_eq!(up.len(), 372);
        let shape: String = up.iter().map(kind).collect();
        assert_eq!(&shape[..50], format!("{}sg", "sp".repeat(24)));
        assert_eq!(&shape[210..212], "sg", "the second copy");
        assert!(shape[50..210].chars().chain(shape[212..371].chars()).all(|c| "slp".contains(c)));
        assert!(shape.ends_with('g'), "the frame gap closes it");
        let (_, wire) = sub_pulses(path);
        let recording: String = wire.iter().map(kind).collect();
        assert!(recording.contains(&shape[..371]), "the recording's second press");
        assert_eq!(feed_all(&mut ChryslerV0Decoder::new(), &up), [ChryslerV0Decoder::frame_signal(CLOSE_2.0, CLOSE_2.1)]);
    }

    /// Lock and Unlock set the button byte (and with it the sum and the order); Trunk, Panic and
    /// the captured code keep it; the counter borrows from its high byte and wraps at zero; a frame
    /// whose sum fails is not sent on.
    #[test]
    fn encode_sends_the_next_count_and_the_requested_button() {
        for (counter, next) in [(0x3B85u16, 0x3B84u16), (0x3B00, 0x3AFF), (0x0000, 0xFFFF)] {
            let [hi, n] = counter.to_be_bytes();
            for captured in [LOCK, UNLOCK, 0x40] {
                let now = signal(n, &pt(captured, hi));
                for (cmd, want) in [(0x01u8, LOCK), (0x02, UNLOCK), (0x04, captured), (0x08, captured), (captured, captured), (LOCK, LOCK), (UNLOCK, UNLOCK)] {
                    let got = feed_all(&mut ChryslerV0Decoder::new(), &ChryslerV0Decoder::new().encode(&now, cmd).unwrap());
                    assert_eq!(got, [signal(next.to_be_bytes()[1], &pt(want, next.to_be_bytes()[0]))], "{counter:04X} {captured:02X} {cmd:02X}");
                }
            }
        }
        let bad = ChryslerV0Decoder::frame_signal(CLOSE_1.0 ^ 1 << 47, CLOSE_1.1);
        assert!(!bad.crc_valid && !bad.encoder_capable);
        assert!(ChryslerV0Decoder::new().encode(&bad, 0x01).is_none());
        let mut short = ChryslerV0Decoder::frame_signal(CLOSE_1.0, CLOSE_1.1);
        short.data_count_bit = 64;
        assert!(ChryslerV0Decoder::new().encode(&short, 0x01).is_none());
    }

    /// Key files keep the C's lines (its plaintext, SnA / SnB, button and seed, which the firmwares
    /// read back when they send from a file); frames A and B round-trip; Flipper-ARF's "Chrysler"
    /// name loads; Key_2 is required and any Bit is taken; a frame whose sum fails loads flagged and
    /// is not sent on; TX from a loaded key sends the next press.
    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::{assert_round_trip, key};
        let dec = ChryslerV0Decoder::new();
        let sig = ChryslerV0Decoder::frame_signal(CLOSE_1.0, CLOSE_1.1);
        let text = assert_round_trip(&dec, &sig);
        let want = format!(
            "Protocol: Chrysler V0\nBit: 80\nKey: 07 A1 1E 7D 24 62 A5 7D\nKey_2: 24 62\n\
             Plain_A: A1 1E 7D 24 62 A5 7D 24 62\nSerial: {}\nBtn: 1\nCnt: 32\n",
            0xA11E_7D24u32
        );
        assert!(text.ends_with(&want), "{text}\nwant tail:\n{want}");
        let b = ChryslerV0Decoder::frame_signal(CLOSE_2.0, CLOSE_2.1);
        let text_b = assert_round_trip(&dec, &b);
        let want_b = format!("Plain_B: 47 25 BF A4 47 25 BF 79 84\nSerial: {}\nBtn: 1\nCnt: 31\n", 0x4725_BF79u32);
        assert!(text_b.ends_with(&want_b), "{text_b}\nwant tail:\n{want_b}");

        // Flipper-ARF's name, through the registry.
        let arf = key(&text.replace("Protocol: Chrysler V0", "Protocol: Chrysler"));
        let (name, loaded) = crate::protocols::ProtocolRegistry::new().decode_key(&arf).unwrap();
        assert_eq!((name.as_str(), &loaded), ("Chrysler V0", &sig));
        // The C forces 80 bits whatever the Bit line says; Key_2 is required.
        let mut any_bits = arf.clone();
        any_bits.bits = 64;
        assert_eq!(dec.from_flipper_key(&any_bits).as_ref(), Some(&sig));
        let mut no_key2 = arf.clone();
        no_key2.fields.retain(|(n, _)| n != "Key_2");
        assert!(dec.from_flipper_key(&no_key2).is_none());
        // A frame whose sum fails loads flagged and is not sent on.
        let mut bad = arf.clone();
        bad.key ^= 1 << 47;
        bad.key_bytes = bad.key.to_be_bytes().to_vec();
        let loaded_bad = dec.from_flipper_key(&bad).unwrap();
        assert!(!loaded_bad.crc_valid && !loaded_bad.encoder_capable);
        assert!(dec.encode(&loaded_bad, 0x01).is_none());

        // TX from the loaded key: the fob's next press.
        let next = feed_all(&mut ChryslerV0Decoder::new(), &dec.encode(&loaded, 0x01).unwrap());
        assert_eq!(next, [b]);
    }

    /// Reference-oracle vectors: `KAT_KEY_VECTORS=<dir> cargo test oracle_vectors -- --ignored`.
    #[test]
    #[ignore]
    fn oracle_vectors() {
        let dec = ChryslerV0Decoder::new();
        for (n, (frame, button)) in [(CLOSE_1, 0x01u8), (CLOSE_2, 0x02)].into_iter().enumerate() {
            let upload = dec.encode(&ChryslerV0Decoder::frame_signal(frame.0, frame.1), button).unwrap();
            let sent = feed_all(&mut ChryslerV0Decoder::new(), &upload).remove(0);
            let mut key = dec.to_flipper_key(&sent).unwrap();
            key.frequency = 433_920_000;
            key.set_preset(&crate::radio::preset::RfPreset::from_def(INFO.presets[0]));
            super::super::ford_v0::write_oracle_vector(&format!("chrysler_v0_{}", n + 1), &upload, &key);
        }
    }
}
