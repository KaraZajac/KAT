//! Chrysler V0 protocol decoder/encoder
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/chrysler_v0.c` and
//! `chrysler_v0.h`. Used by Chrysler/Dodge/Jeep keyfobs.
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
//! - Dual payload A (seed even, carries serial+counter) / B (seed odd, carries serial). The frame
//!   is gated on a structural `check_ok` (matches the C), so it does not false-match other
//!   protocols. `crc_valid` reflects that `check_ok`.
//!
//! RF: AM, 315 + 433.92 MHz. Encoder present (ENABLE_EMULATE_FEATURE): builds preamble + dual
//! 80-bit PWM frames with frame gaps.

use super::{ProtocolDecoder, ProtocolTiming, DecodedSignal};
use crate::radio::demodulator::LevelDuration;
use crate::duration_diff;

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
    /// Transform seed (reversed 6-bit counter); retained for diagnostics.
    #[allow(dead_code)]
    seed: u8,
    /// Serial: SnA (counter-frame) or SnB (serial-frame).
    serial: u32,
    /// Rolling counter (only meaningful for the A/even frame).
    counter: u32,
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

        let (serial, counter) = if (seed & 1) != 0 {
            // Payload B: serial only.
            let sn_b = ((decoded[0] as u32) << 24)
                | ((decoded[1] as u32) << 16)
                | ((decoded[2] as u32) << 8)
                | (decoded[7] as u32);
            (sn_b, 0u32)
        } else {
            // Payload A: serial (SnA) + rolling counter.
            let sn_a = ((decoded[0] as u32) << 24)
                | ((decoded[1] as u32) << 16)
                | ((decoded[2] as u32) << 8)
                | (decoded[3] as u32);
            let cnt = sn_a;
            (sn_a, cnt)
        };

        Decoded {
            check_ok,
            button,
            seed,
            serial,
            counter,
        }
    }

    /// Build a DecodedSignal from a committed frame, if the structural check passes.
    fn commit(&self) -> Option<DecodedSignal> {
        let d = Self::decode_packet(self.decode_data, self.data_2);
        if !d.check_ok {
            return None;
        }
        Some(DecodedSignal {
            serial: Some(d.serial),
            button: Some(d.button),
            counter: Some((d.counter & 0xFFFF) as u16),
            crc_valid: d.check_ok,
            // 80-bit frame: report the most-significant 64 bits (payload bytes 0..7).
            data: self.decode_data,
            data_count_bit: DECODE_BIT_COUNT,
            encoder_capable: true,
            // Stash the low 16 bits (data_2 = payload bytes 8,9) for the encoder.
            extra: Some(self.data_2 as u64),
            protocol_display_name: None,
        })
    }

    /// Map a KAT generic button to a Chrysler button code (1=Lock, 2=Unlock).
    fn map_button(button: u8) -> u8 {
        match button {
            0x01 => 1, // Lock
            0x02 => 2, // Unlock
            0x04 => 1, // Trunk → Lock (Chrysler V0 only models Lock/Unlock)
            0x08 => 2, // Panic → Unlock
            _ => 1,
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
        // Rebuild the 10-byte payload A (high 64 bits + low 16 bits) from the decoded frame,
        // re-derive seed/plaintext, then emit preamble + dual PWM frames (matches
        // chrysler_v0_build_upload, ENABLE_EMULATE_FEATURE).
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
        let tx_button = Self::map_button(button);
        if tx_button != original_button {
            plain_a[5] ^= 0x0C;
            plain_b[3] ^= 0x30;
        }

        // Counter handling (matches encoder deserialize): counter_a is the even counter, counter_b
        // is counter_a - 1 (wrapping in 6 bits). Derive the base counter from the seed.
        let counter = (seed as u32) & 0x3F;
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

    /// Build a payload-A frame exactly as the encoder does (even counter, MSB-clear path), encode
    /// it to a PWM burst, decode it back, and confirm the frame, check_ok, button and serial all
    /// survive the round trip. Payload A is the first frame the encoder emits, so `decode_stream`
    /// returns it.
    ///
    /// Payload-A check_ok requires (in the stored = transformed bytes): `(key[1]^0xC3)==key[5]`
    /// and `key[6]^key[1]==0x04`. Since transform XORs every byte with a constant mask, this is
    /// equivalent to a check on the *plaintext*: `plain[0]^0xC3==plain[4]` and `plain[5]^plain[0]==0x04`.
    #[test]
    fn encode_decode_round_trip() {
        // Choose an even counter so the encoder's counter_a equals it (exact round trip), and pick
        // plaintext satisfying the payload-A invariants.
        let counter_a: u8 = 0x14; // even
        let button: u8 = 1; // Lock (matches the b1^b6==0x04 branch which keeps button=1)
        let header_low2: u8 = 0x02;

        let p0 = 0x5Au8;
        let mut plain = [0x00u8; 9];
        plain[0] = p0;
        plain[1] = 0x11;
        plain[2] = 0x22;
        plain[3] = 0x33;
        plain[4] = p0 ^ 0xC3; // plain[0]^0xC3 == plain[4]
        plain[5] = p0 ^ 0x04; // plain[5]^plain[0] == 0x04
        plain[6] = 0x66;
        plain[7] = 0x77;
        plain[8] = 0x88;

        // Build the 10-byte payload the way the encoder/transmitter does.
        let payload = ChryslerV0Decoder::build_payload(&plain, counter_a, button, header_low2);
        let data = u64::from_be_bytes([
            payload[0], payload[1], payload[2], payload[3], payload[4], payload[5], payload[6],
            payload[7],
        ]);
        let data_2: u16 = ((payload[8] as u16) << 8) | payload[9] as u16;

        // Sanity: the raw frame must pass the structural check before we exercise the codec.
        let d = ChryslerV0Decoder::decode_packet(data, data_2);
        assert!(d.check_ok, "constructed payload-A vector should satisfy check_ok");
        assert_eq!(d.button, button, "constructed button mismatch");
        assert_eq!(d.seed & 1, 0, "payload A must have an even seed");

        let decoded = DecodedSignal {
            serial: Some(d.serial),
            button: Some(d.button),
            counter: Some((d.counter & 0xFFFF) as u16),
            crc_valid: true,
            data,
            data_count_bit: DECODE_BIT_COUNT,
            encoder_capable: true,
            extra: Some(data_2 as u64),
            protocol_display_name: None,
        };

        let dec = ChryslerV0Decoder::new();
        // Encode with the same (Lock) button so the plaintext is not perturbed.
        let burst = dec.encode(&decoded, 0x01).expect("encode should succeed");
        assert!(!burst.is_empty());

        let got = decode_stream(&burst).expect("encoded burst should decode");
        assert!(got.crc_valid, "decoded frame should pass check");
        // The high 64 bits (payload bytes 0..7) must reproduce the original frame.
        assert_eq!(got.data, data, "round-trip data mismatch");
        assert_eq!(got.button, decoded.button, "button mismatch");
        assert_eq!(got.serial, decoded.serial, "serial mismatch");
    }
}
