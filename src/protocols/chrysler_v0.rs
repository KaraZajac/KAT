//! Chrysler V0 protocol decoder/encoder
//!
//! Port of ProtoPirate `protocols/chrysler_v0.c` (HEAD). Flipper-ARF ships the same file as
//! `lib/subghz/protocols/chrysler.c` ("Chrysler", FOBIK GQ43VT) and doctor-firmware as
//! `chrysler_v0.c`; their decoders are identical to ProtoPirate's, their encoders differ only in the
//! counter step and D-pad buttons (below). Used by older Chrysler/Dodge/Jeep FOBIK keyfobs. (The
//! newer FCA fobs in `IMPORTS/chrysler` and `IMPORTS/jeep` are Fiat V2 frames; no reference's
//! Chrysler V0 decodes them, in either polarity.)
//!
//! Protocol: PWM with a short HIGH pulse and TWO long-LOW symbols. A "1" payload bit is
//! HIGH≈600µs (te_one_short) + LOW≈3400µs (te_long_a); a "0" payload bit is HIGH≈300µs
//! (te_short) + LOW≈3700µs (te_long_b). te_delta≈150, long_delta≈400, te_gap≈8000,
//! frame_gap≈15600. ~24 preamble pairs (short HIGH + long_b LOW) precede each frame.
//!
//! Frame: 80 bits. The first 64 bits are `decode_data` (payload bytes 0..7), the last 16 bits
//! are `data_2` (payload bytes 8,9). The 80-bit frame exceeds u64, so [DecodedSignal::data]
//! reports the most-significant 64 bits and `data_count_bit = 80` (matches psa.rs/kia_v6.rs).
//!
//! Crypto (proprietary seed-XOR, ported exactly from chrysler_v0.c `decode`):
//! - `seed = reverse6(key[0] >> 2)` — a 6-bit reversed counter used as the transform key.
//! - `transform_block`: XOR all 9 transformed bytes with `xor_table[seed & 0x0F]`, with an extra
//!   nibble flip when the (Lock) button is set.
//! - Dual payload A (seed even, plaintext SnA) / B (seed odd, plaintext SnB). The frame is
//!   reported only when the structural `check_ok` passes (as in the C), so it does not false-match
//!   other protocols. `crc_valid` reflects that `check_ok`.
//!
//! Fields (as the C serializes them): `serial` = SnA (payload A: plaintext bytes 0..3) or SnB
//! (payload B: plaintext bytes 0,1,2,7); `button` = 1 Lock / 2 Unlock (same as KAT's generic
//! codes); `counter` = the 6-bit seed (the C's `Cnt`); `extra` = the last 16 frame bits (`Key_2`).
//!
//! RF: AM, 315 + 433.92 MHz. Encoder (`subghz_protocol_encoder_chrysler_v0_deserialize` +
//! `chrysler_v0_build_upload`): plaintext recovered from the captured frame, button flip
//! (`plain_a[5] ^= 0x0C`, `plain_b[3] ^= 0x30`) when the requested button differs, then payload A
//! (even counter) and B (counter − 1) each sent as 24 preamble pairs + short + 15.6ms gap + 80
//! PWM bits + short + 15.6ms gap (424 entries; the Flipper sends the upload `repeat` = 2 times while
//! held, KAT returns it once). Buttons: KAT Lock → 1, Unlock → 2 (ProtoPirate emulate, ARF and
//! doctor D-pads); Trunk/Panic keep the captured button.
//!
//! **Deviation (counter), from real captures:** KAT sends seed − 1, made even: A(N−2) + B(N−3)
//! from a payload-A capture, A(N−1) + B(N−2) from payload B. The payload of the captured type is
//! then the fob's next press of that type; the other one carries the captured plaintext, as in the
//! C, which A and B frames don't share. The Chrysler PT Cruiser 2006 fob in `IMPORTS/chrysler`
//! sends one payload per press, alternating A and B with the seed counting **down** by one (A(N),
//! B(N−1), A(N−2)…), and re-encoding each of its presses reproduces its press two later bit for
//! bit. The references count up: ProtoPirate's emulate scene and doctor-firmware's encoder add 1
//! to `Cnt`, which re-sends the captured press; Flipper-ARF adds 2, which sends an earlier press.
//! A fob counting down has already sent both.
//!
//! **Key files** (`subghz_protocol_decoder_chrysler_v0_serialize` / `_deserialize`; Flipper-ARF's
//! "Chrysler" writes the same lines): `Protocol: Chrysler V0`, `Bit: 80`, `Key` =
//! the first 64 frame bits, then `Key_2` (the last 16 bits, big-endian), `Plain_A` (frame A) or
//! `Plain_B` (frame B) — the frame's 9 plaintext bytes — and `Serial` (SnA / SnB), `Btn`, `Cnt`
//! (the 6-bit seed). Two reference quirks are not copied: the C writes Key_2/Plain_x after
//! `flipper_format_rewind`, i.e. over the start of its own buffer (KAT writes them after Key, the
//! order the C means and doctor-firmware's fixed copy writes), and its decoder instance keeps the
//! plaintext of earlier frames — even ones that failed the check — so a reference file may also
//! carry the other frame's Plain_x, with that frame's SnB as Serial (KAT writes what a fresh
//! decoder writes; on load the frame decides). Loading (`_deserialize`) reads Key + Key_2 (required)
//! with any Bit (the C forces 80) and decodes the frame as the decoder does; the C loads frames
//! whose structure check fails too (shown as `Chk: ERR`), so KAT loads them with `crc_valid`
//! false. The file's Serial/Btn/Cnt are not applied (the C's decoder derives them from the frame).

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
    buttons: None,
    presets: &[&AM650],
};

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

/// XOR table (chrysler_v0_xor_table) — indexed by `seed & 0x0F`.
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

/// Result of `decode_packet`: structural fields extracted from a candidate frame.
struct Decoded {
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

    /// Port of chrysler_v0_decode_packet: extract seed/button/check_ok/serial/counter from the
    /// 64-bit `data` (payload bytes 0..7) plus the 16-bit `data_2` (payload bytes 8,9).
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

        let encoded: [u8; 9] = [
            key[1],
            key[2],
            key[3],
            key[4],
            key[5],
            key[6],
            key[7],
            (key2 >> 8) as u8,
            (key2 & 0xFF) as u8,
        ];
        let decoded = Self::transform_block(&encoded, seed, button);

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

    /// Build a DecodedSignal from a committed frame, if the structural check passes.
    fn commit(&self) -> Option<DecodedSignal> {
        let s = Self::frame_signal(self.decode_data, self.data_2);
        s.crc_valid.then_some(s)
    }

    /// The decode of frame `data` + `data_2` (`crc_valid` = the C's `check_ok`).
    fn frame_signal(data: u64, data_2: u16) -> DecodedSignal {
        let d = Self::decode_packet(data, data_2);
        DecodedSignal {
            serial: Some(d.serial),
            button: Some(d.button),
            counter: Some(d.seed as u16),
            crc_valid: d.check_ok,
            // 80-bit frame: report the most-significant 64 bits (payload bytes 0..7).
            data,
            data_count_bit: DECODE_BIT_COUNT,
            encoder_capable: true,
            // Stash the low 16 bits (data_2 = payload bytes 8,9) for the encoder.
            extra: Some(data_2 as u64),
            protocol_display_name: None,
        }
    }

    /// The frame's 9 plaintext bytes (`decoded` in `chrysler_v0_decode_packet`: Plain_A for an even
    /// seed, Plain_B for an odd one).
    fn plaintext(data: u64, data_2: u16) -> [u8; 9] {
        let d = Self::decode_packet(data, data_2);
        let key = data.to_be_bytes();
        let encoded = [key[1], key[2], key[3], key[4], key[5], key[6], key[7], (data_2 >> 8) as u8, data_2 as u8];
        Self::transform_block(&encoded, d.seed, d.button)
    }

    /// Map a KAT generic button to a Chrysler button code (1 = Lock, 2 = Unlock); the fob has no
    /// other buttons in any reference, so anything else keeps the captured one.
    fn map_button(button: u8, original: u8) -> u8 {
        match button {
            0x01 => 1,
            0x02 => 2,
            _ => original,
        }
    }

    /// Read one bit out of an MSB-first payload (chrysler_v0_payload_get_bit).
    fn payload_get_bit(payload: &[u8; 10], index: u8) -> u8 {
        let byte = payload[(index >> 3) as usize];
        let shift = 7 - (index & 7);
        (byte >> shift) & 1
    }

    /// Build a 10-byte payload from the 9 plaintext bytes (chrysler_v0_build_payload).
    fn build_payload(plain: &[u8; 9], counter: u8, button: u8, header_low2: u8) -> [u8; 10] {
        let transformed = Self::transform_block(plain, counter, button);
        let mut out = [0u8; 10];
        out[0] = (Self::reverse6(counter as u32) << 2) | (header_low2 & 0x03);
        out[1..10].copy_from_slice(&transformed);
        out
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

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        // Rebuild the 10-byte payload from the captured frame, re-derive seed/plaintext, then emit
        // preamble + dual PWM frames (subghz_protocol_encoder_chrysler_v0_deserialize +
        // chrysler_v0_build_upload).
        if decoded.data_count_bit != DECODE_BIT_COUNT {
            return None;
        }
        let data = decoded.data;
        let data_2 = decoded.extra.unwrap_or(0) as u16;
        let key = data.to_be_bytes();
        let key2 = data_2;

        // Header low-2 bits live in the top byte's low nibble (matches encoder deserialize:
        // plain_header = (data >> 56) & 0x03).
        let header_low2 = (key[0]) & 0x03;
        let seed = Self::reverse6((key[0] >> 2) as u32);

        // Determine the originally-transmitted button (matches encoder deserialize).
        let original_button: u8 = if (key[0] & 0x80) == 0 {
            if (key[1] ^ key[6]) == 0x08 { 2 } else { 1 }
        } else if (((key2 & 0xFF) as u8) ^ key[4]) == 0x10 {
            2
        } else {
            1
        };

        // Recover the plaintext (Plain_A/Plain_B both default to this in the C when no explicit
        // Plain_A/B is stored — which is our case).
        let encoded: [u8; 9] = [
            key[1],
            key[2],
            key[3],
            key[4],
            key[5],
            key[6],
            key[7],
            (key2 >> 8) as u8,
            (key2 & 0xFF) as u8,
        ];
        let generated = Self::transform_block(&encoded, seed, original_button);
        let mut plain_a = generated;
        let mut plain_b = generated;

        // Apply the requested button (KAT button → Chrysler 1/2).
        let tx_button = Self::map_button(button, original_button);
        if tx_button != original_button {
            plain_a[5] ^= 0x0C;
            plain_b[3] ^= 0x30;
        }

        // Counter: seed − 1 (6-bit), made even = counter_a; counter_b = counter_a − 1. The payload
        // of the captured type is then the fob's next press of that type (real fobs count the seed
        // down; see the module doc — the references count up). The seed comes from the frame
        // itself (the C's default `Cnt`; it equals `decoded.counter`, which older KAT .fob files
        // filled with SnA instead).
        let counter = ((seed as u32) + 0x3F) & 0x3F;
        let mut counter_a = (counter & 0x3F) as u8;
        if (counter_a & 1) != 0 {
            counter_a = counter_a.wrapping_sub(1) & 0x3F;
        }
        let counter_b = if counter_a == 0 { 0x3F } else { counter_a - 1 };

        let payload_a = Self::build_payload(&plain_a, counter_a, tx_button, header_low2);
        let payload_b = Self::build_payload(&plain_b, counter_b, tx_button, header_low2);

        let mut signal = Vec::with_capacity(PREAMBLE_PAIRS * 4 + 80 * 4 + 16);

        // Frame A: preamble, short + frame_gap, payload A, short + frame_gap.
        Self::emit_preamble(&mut signal);
        Self::add_level(&mut signal, true, TE_SHORT);
        Self::add_level(&mut signal, false, FRAME_GAP);
        Self::emit_payload(&mut signal, &payload_a);
        Self::add_level(&mut signal, true, TE_SHORT);
        Self::add_level(&mut signal, false, FRAME_GAP);

        // Frame B: preamble, short + frame_gap, payload B, short + frame_gap.
        Self::emit_preamble(&mut signal);
        Self::add_level(&mut signal, true, TE_SHORT);
        Self::add_level(&mut signal, false, FRAME_GAP);
        Self::emit_payload(&mut signal, &payload_b);
        Self::add_level(&mut signal, true, TE_SHORT);
        Self::add_level(&mut signal, false, FRAME_GAP);

        Some(signal)
    }

    /// `subghz_protocol_decoder_chrysler_v0_deserialize` (see the module doc, "Key files").
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        let data = key.key_u64()?;
        let key2: [u8; 2] = key.hex_bytes_field("Key_2")?.try_into().ok()?;
        Some(Self::frame_signal(data, u16::from_be_bytes(key2)))
    }

    /// `subghz_protocol_decoder_chrysler_v0_serialize` (see the module doc, "Key files").
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
    use super::*;

    /// Decode a stream of pairs with a fresh decoder, returning the first frame.
    fn decode_stream(pairs: &[LevelDuration]) -> Option<DecodedSignal> {
        let mut dec = ChryslerV0Decoder::new();
        for p in pairs {
            if let Some(sig) = dec.feed(p.level, p.duration_us) {
                return Some(sig);
            }
        }
        None
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
    /// A captured payload-A frame (plaintext satisfying the A check for `button`), as the fob
    /// would send it with seed `counter_a`.
    fn captured_a(counter_a: u8, button: u8) -> (DecodedSignal, [u8; 9]) {
        let p0 = 0x5Au8;
        let mut plain = [0u8; 9];
        plain[0] = p0;
        plain[1] = 0x11;
        plain[2] = 0x22;
        plain[3] = 0x33;
        plain[4] = p0 ^ 0xC3; // plain[0]^0xC3 == plain[4]
        plain[5] = p0 ^ if button == 1 { 0x04 } else { 0x08 }; // b1^b6: 0x04 Lock / 0x08 Unlock
        plain[6] = 0x66;
        plain[7] = 0x77;
        plain[8] = 0x88;
        let payload = ChryslerV0Decoder::build_payload(&plain, counter_a, button, 0x02);
        let data = u64::from_be_bytes(payload[..8].try_into().unwrap());
        let data_2 = (payload[8] as u16) << 8 | payload[9] as u16;
        let d = ChryslerV0Decoder::decode_packet(data, data_2);
        assert!(d.check_ok && d.button == button && d.seed == counter_a);
        let sig = DecodedSignal {
            serial: Some(d.serial),
            button: Some(d.button),
            counter: Some(d.seed as u16),
            crc_valid: true,
            data,
            data_count_bit: DECODE_BIT_COUNT,
            encoder_capable: true,
            extra: Some(data_2 as u64),
            protocol_display_name: None,
        };
        (sig, plain)
    }

    /// Decoding reports the C's fields: SnA as serial, the 6-bit seed as counter.
    #[test]
    fn decodes_payload_a_fields() {
        let (sig, plain) = captured_a(0x14, 1);
        let mut burst = Vec::new();
        ChryslerV0Decoder::emit_preamble(&mut burst);
        ChryslerV0Decoder::add_level(&mut burst, true, TE_SHORT);
        ChryslerV0Decoder::add_level(&mut burst, false, FRAME_GAP);
        let mut payload = [0u8; 10];
        payload[..8].copy_from_slice(&sig.data.to_be_bytes());
        payload[8..].copy_from_slice(&(sig.extra.unwrap() as u16).to_be_bytes());
        ChryslerV0Decoder::emit_payload(&mut burst, &payload);
        ChryslerV0Decoder::add_level(&mut burst, true, TE_SHORT);
        ChryslerV0Decoder::add_level(&mut burst, false, FRAME_GAP);
        let got = decode_stream(&burst).expect("frame decodes");
        assert_eq!(got.data, sig.data);
        assert_eq!(got.extra, sig.extra);
        assert_eq!(got.serial, Some(u32::from_be_bytes([plain[0], plain[1], plain[2], plain[3]])));
        assert_eq!((got.button, got.counter, got.crc_valid), (Some(1), Some(0x14), true));
    }

    /// Transmitting from a payload-A capture of seed N sends A(N − 2), the fob's next payload-A
    /// press, with the same plaintext; Lock/Unlock select the button, Trunk/Panic keep the captured
    /// one.
    #[test]
    fn encode_sends_next_seed_and_requested_button() {
        for counter_a in (0..64u8).step_by(2) {
            for captured_button in [1u8, 2] {
                let (sig, _) = captured_a(counter_a, captured_button);
                for (cmd, want) in [(0x01u8, 1u8), (0x02, 2), (0x04, captured_button), (0x08, captured_button)] {
                    let burst = ChryslerV0Decoder::new().encode(&sig, cmd).expect("encodes");
                    assert_eq!(burst.len(), 424);
                    let got = decode_stream(&burst).expect("payload A decodes");
                    assert_eq!(got.counter, Some(((counter_a + 62) & 0x3F) as u16));
                    assert_eq!(got.button, Some(want));
                    assert_eq!(got.serial, sig.serial);
                }
            }
        }
    }

    /// Every decode of an upload, in order.
    fn decode_all(pairs: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = ChryslerV0Decoder::new();
        pairs.iter().filter_map(|p| dec.feed(p.level, p.duration_us)).collect()
    }

    /// Key files (ProtoPirate `subghz_protocol_decoder_chrysler_v0_serialize` / `_deserialize`):
    /// frames A and B round-trip with the C's lines (Key_2 big-endian, the frame's Plain_x),
    /// Flipper-ARF's "Chrysler" name loads, a frame failing the check loads flagged, Key_2 is
    /// required, and TX from a loaded key decodes back.
    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::{assert_round_trip, key};
        let dec = ChryslerV0Decoder::new();
        let (sig, plain) = captured_a(0x14, 1);
        let text = assert_round_trip(&dec, &sig);
        let k = sig.data.to_be_bytes();
        let p: Vec<String> = plain.iter().map(|b| format!("{b:02X}")).collect();
        let x = sig.extra.unwrap() as u16;
        let want = format!(
            "Protocol: Chrysler V0\nBit: 80\nKey: {}\nKey_2: {:02X} {:02X}\nPlain_A: {}\nSerial: {}\nBtn: 1\nCnt: 20\n",
            k.iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(" "),
            x >> 8,
            x & 0xFF,
            p.join(" "),
            sig.serial.unwrap()
        );
        assert!(text.ends_with(&want), "{text}\nwant tail:\n{want}");

        // A payload-B frame (odd seed; B check: plain[4] == plain[0], plain[5] ^ plain[0] == 0x62,
        // Unlock when plain[8] ^ plain[3] == 0x10) writes Plain_B and SnB.
        for button in [1u8, 2] {
            let mut plain = [0x5A, 0x11, 0x22, 0x33, 0x5A, 0x5A ^ 0x62, 0x66, 0x77, 0x00];
            plain[8] = plain[3] ^ if button == 2 { 0x10 } else { 0x20 };
            let payload = ChryslerV0Decoder::build_payload(&plain, 0x15, button, 0x02);
            let data = u64::from_be_bytes(payload[..8].try_into().unwrap());
            let b = ChryslerV0Decoder::frame_signal(data, u16::from_be_bytes([payload[8], payload[9]]));
            assert!(b.crc_valid && b.button == Some(button) && b.counter == Some(0x15));
            assert_eq!(b.serial, Some(0x5A11_2277), "SnB = plaintext bytes 0, 1, 2, 7");
            let text = assert_round_trip(&dec, &b);
            let p: Vec<String> = plain.iter().map(|x| format!("{x:02X}")).collect();
            assert!(text.contains(&format!("\nPlain_B: {}\nSerial: 1511072375\n", p.join(" "))), "{text}");
        }

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
        // A frame failing the structure check loads (the C shows Chk: ERR).
        let mut bad = arf.clone();
        bad.key ^= 0x00FF_0000_0000_0000;
        bad.key_bytes = bad.key.to_be_bytes().to_vec();
        assert!(!dec.from_flipper_key(&bad).unwrap().crc_valid);

        // TX from the loaded key: the next payload-A press (seed − 2) decodes back.
        let next = decode_all(&dec.encode(&loaded, 0x01).unwrap()).remove(0);
        assert_eq!((next.serial, next.counter), (sig.serial, Some(0x12)));
    }

    /// The real PT Cruiser 2006 fob (`IMPORTS/chrysler`, ten presses per file, ~1.5 s apart): one
    /// payload per press, A and B alternating, the seed counting down by one. Re-encoding each
    /// press with its own button reproduces, in the payload of its type, the fob's press two later
    /// bit for bit.
    #[test]
    fn reencode_reproduces_the_fobs_next_press() {
        for rel in ["Chrysler PT Cruiser 2006 open.sub", "Chrysler PT Cruiser 2006 close.sub"] {
            let path = std::path::Path::new("IMPORTS/chrysler").join(rel);
            if !path.exists() {
                eprintln!("Skip: {path:?} not found (run from crate root)");
                return;
            }
            let (freq, raw) = crate::export::flipper::import_sub_raw(&path).unwrap();
            let pairs: Vec<LevelDuration> =
                raw.iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect();
            let presses: Vec<DecodedSignal> = crate::protocols::ProtocolRegistry::new()
                .process_signal_stream(&pairs, freq)
                .into_iter()
                .map(|(name, d, _)| {
                    assert_eq!(name, "Chrysler V0", "{rel}");
                    d
                })
                .collect();
            assert_eq!(presses.len(), 10, "{rel}");
            for w in presses.windows(2) {
                assert_eq!(w[1].counter, Some((w[0].counter.unwrap() + 63) & 0x3F), "{rel}: seed − 1 per press");
            }
            for (now, later) in presses.iter().zip(&presses[2..]) {
                let button = now.button.unwrap(); // 1 Lock / 2 Unlock = KAT's codes 0x01 / 0x02
                let sent = decode_all(&ChryslerV0Decoder::new().encode(now, button).unwrap());
                let same_type = sent
                    .iter()
                    .find(|d| d.counter.unwrap() % 2 == now.counter.unwrap() % 2)
                    .unwrap_or_else(|| panic!("{rel}: no payload of the captured type in {sent:?}"));
                assert_eq!((same_type.data, same_type.extra), (later.data, later.extra), "{rel}");
            }
        }
    }

    /// Reference-oracle vectors: `KAT_KEY_VECTORS=<dir> cargo test oracle_vectors -- --ignored`.
    #[test]
    #[ignore]
    fn oracle_vectors() {
        let dec = ChryslerV0Decoder::new();
        for (n, (seed, button)) in [(0x14u8, 1u8), (0x22, 2)].into_iter().enumerate() {
            let (sig, _) = captured_a(seed, button);
            let upload = dec.encode(&sig, 0x40).unwrap();
            let sent = decode_all(&upload).remove(0);
            let mut key = dec.to_flipper_key(&sent).unwrap();
            key.frequency = 433_920_000;
            key.set_preset(&crate::radio::preset::RfPreset::from_def(INFO.presets[0]));
            super::super::ford_v0::write_oracle_vector(&format!("chrysler_v0_{}", n + 1), &upload, &key);
        }
    }
}
