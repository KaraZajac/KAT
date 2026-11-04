//! Audi protocol decoder/encoder — EXPERIMENTAL (Pandora RE).
//!
//! Ported from Flipper-ARF: `lib/subghz/protocols/audi.c` / `audi.h` (`AUDI_PROTOCOL_NAME =
//! "Audi"`). The C was reverse-engineered from the PANDORA_MAX alarm firmware (button/counter
//! dispatcher 0xf152, {b3,b4,b5} scramble + parity engine 0xf8fc, PWM bit-banger 0xbd64); Pandora
//! units emulate the OEM fob. Flipper-ARF ships it DISABLED (commented out of its
//! `protocol_items.c`). It is the only copy in REFERENCES, so it is followed as written.
//!
//! This is NOT KAT's `VAG` protocol (`vag.rs`: 300/600 or 500/1000µs Manchester, 80 bits,
//! AUT64/TEA): Audi is 550/1100µs PWM behind a 2500/850µs header, 64 bits.
//!
//! # Wire (C encoder, AM/OOK, 315 / 433 / 868 MHz)
//!
//! - PWM, Te = 550µs (te_delta 137µs): `bit 1 = HIGH 1100 + LOW 550`, `bit 0 = HIGH 550 + LOW
//!   1100`.
//! - Per repeat: header `HIGH 2500 → LOW 850`, 64 data bits (8 bytes, LSB-FIRST within each byte),
//!   then a `LOW 850` inter-repeat gap (which merges with the last bit's LOW). 5 repeats per press.
//!
//! # Payload (8 on-wire bytes; `data` = b0..b7 MSB-first, exactly the C's `generic.data`)
//!
//! - b0..b2 = 24-bit UID → `serial`
//! - b3..b5 = rolling payload, SCRAMBLED on the wire: 4 rounds of a 24-bit left shift whose
//!   carry-in is the parity `P = b3.3 ^ b4.4 ^ b4.6 ^ b4.7 ^ b5.4 ^ b5.7` of the current block (the
//!   final carry-out is dropped and recovered on decode from `P`, bit 0 of the scrambled b3).
//!   `counter` = plaintext `b3:b4` after un-scrambling (16 bits, as the C's `cnt`).
//! - b6 = button (0x01 Lock, 0x02 Unlock, 0x04 Trunk) → `button`
//! - b7 = checksum = `sum(b0..b6) & 0xFF` over the ON-WIRE (scrambled) bytes.
//!
//! The scramble is a bijection on 24 bits, so the C's "parity OK" self-check after un-scrambling is
//! satisfied BY CONSTRUCTION for every input (`parity_self_check_is_vacuous` proves it) — it is not
//! a gate. The only content check the frame carries is the 8-bit checksum.
//!
//! # Gating (KAT is stricter than the C)
//!
//! The C reports any 64-bit frame with `UID ∉ {0, 0xFFFFFF}` and only DISPLAYS the checksum
//! (`CS:[OK|opaque]`). KAT emits only when ALL of these hold:
//! 1. header HIGH ≥ 1900µs then LOW 850 ± 274µs (as the C);
//! 2. EXACTLY 64 strict PWM bits (each `HIGH short + LOW long` or `HIGH long + LOW short`, ±137µs),
//!    terminated by a gap / new header (a 65th bit rejects the frame; the C would report the first
//!    64);
//! 3. `UID ∉ {0, 0xFFFFFF}` (as the C);
//! 4. checksum `b7 == sum(b0..b6)` — random pass 1/256;
//! 5. button `b6 ∈ {0x01, 0x02, 0x04}` — the only codes the RE'd emitter produces; random pass
//!    3/256.
//!
//! Content gates 4+5 pass a random 64-bit frame with p ≈ 4.6e-5 (1/21 800); gates 1-2 (a 2.5 ms
//! header + 64 complementary PWM pairs at 550/1100µs) essentially never occur in noise.
//!
//! # Encoder
//!
//! Supported, as in the C. The stored on-wire bytes (`data`) are replayed byte-identically when the
//! requested button maps to the captured b6 (the C's OK button). A different button re-derives the
//! frame like the C D-pad path: un-scramble, set plaintext `b3:b4` = counter + 1 (the C's
//! `rolling_counter_mult` default), set b6, re-scramble, recompute b7. KAT generic buttons map
//! 1:1 (Lock 0x01, Unlock 0x02, Trunk 0x04); any other code (e.g. Panic 0x08) keeps the captured
//! button. Waveform = the C upload: 5 × (`HIGH 2500, LOW 850`, 64 PWM bits LSB-first, `LOW 850`).
//!
//! # Deviations from the C
//!
//! - Final-bit fix: the C classifies the last bit only when it is a 0 (`HIGH short` + merged gap);
//!   a final `HIGH long` + merged gap (`550 + 850 = 1400µs` LOW) falls into its error branch with
//!   63 bits, so the C silently drops every frame whose b7 MSB is 1. KAT decodes the final bit from
//!   the HIGH width whenever the LOW is a gap (> te_long + te_delta).
//! - Exact-length + checksum + button gates (above); the C's "lone short LOW after a short HIGH
//!   counts as a final 0" is narrowed to real gaps.
//! - The C's repeated-frame de-dup is not ported (KAT resets decoders after every emission).

use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Audi",
    decoder: "Audi",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "Audi",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&AM650],
};

const TE_SHORT: u32 = 550;
const TE_LONG: u32 = 1100;
const TE_DELTA: u32 = 137;
const HEADER_HIGH: u32 = 2500;
const HEADER_LOW: u32 = 850;
const GAP_US: u32 = 850;
/// Boundary between the 2500µs header HIGH and a data cell (≤ 1100 + delta).
const HEADER_MIN_US: u32 = 1900;
/// A LOW longer than this after a data HIGH is the frame-ending gap (last bit's LOW + gap).
const GAP_MIN_US: u32 = TE_LONG + TE_DELTA;
const WIRE_BYTES: usize = 8;
const WIRE_BITS: usize = 64;
const REPEATS: usize = 5;
const SCRAMBLE_ROUNDS: usize = 4;

/// On-wire button codes (b6).
const BTN_0: u8 = 0x01; // Lock
const BTN_1: u8 = 0x02; // Unlock
const BTN_2: u8 = 0x04; // Trunk

/// Decoder steps (AudiDecoderStep).
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    HeaderLow,
    Data,
}

/// Audi decoder (SubGhzProtocolDecoderAudi).
pub struct AudiDecoder {
    step: DecoderStep,
    pending_high: u32,
    have_high: bool,
    bytes: [u8; WIRE_BYTES],
    bit_count: usize,
}

#[inline]
fn bit(x: u8, n: u8) -> u8 {
    (x >> n) & 1
}

/// P = bit3(b3) ^ bit4(b4) ^ bit6(b4) ^ bit7(b4) ^ bit4(b5) ^ bit7(b5) (audi_parity).
fn parity(b3: u8, b4: u8, b5: u8) -> u8 {
    bit(b3, 3) ^ bit(b4, 4) ^ bit(b4, 6) ^ bit(b4, 7) ^ bit(b5, 4) ^ bit(b5, 7)
}

/// One forward scramble round over {b3,b4,b5} (audi_scramble_iteration).
fn scramble_round(block: &mut [u8; 3]) {
    let mut carry = parity(block[0], block[1], block[2]);
    for b in block.iter_mut() {
        let input = *b;
        *b = (input << 1) | carry;
        carry = input >> 7;
    }
    // The final carry (old bit7 of b5) is dropped; it is recovered from P on decode.
}

/// One inverse round (audi_unscramble_iteration). Returns the C's self-check
/// (recomputed parity == seed), which holds by construction.
fn unscramble_round(block: &mut [u8; 3]) -> bool {
    let (o0, o1, o2) = (block[0], block[1], block[2]);
    let p = o0 & 1;
    let in0 = (o0 >> 1) | ((o1 & 1) << 7);
    let in1 = (o1 >> 1) | ((o2 & 1) << 7);
    let in2_low7 = o2 >> 1;
    let known = bit(in0, 3) ^ bit(in1, 4) ^ bit(in1, 6) ^ bit(in1, 7) ^ bit(in2_low7, 4);
    let in2 = in2_low7 | ((p ^ known) << 7);
    *block = [in0, in1, in2];
    parity(in0, in1, in2) == p
}

/// sum(b0..b6) & 0xFF (audi_checksum).
fn checksum(raw: &[u8; WIRE_BYTES]) -> u8 {
    raw[..7].iter().fold(0u8, |acc, &x| acc.wrapping_add(x))
}

/// Scramble b3..b5 (4 rounds) and set the checksum (audi_encrypt).
fn encrypt(raw: &mut [u8; WIRE_BYTES]) {
    let mut block = [raw[3], raw[4], raw[5]];
    for _ in 0..SCRAMBLE_ROUNDS {
        scramble_round(&mut block);
    }
    raw[3..6].copy_from_slice(&block);
    raw[7] = checksum(raw);
}

/// Un-scramble b3..b5 in place (audi_decrypt_block). Returns the (always-true) parity self-check.
fn decrypt_block(raw: &mut [u8; WIRE_BYTES]) -> bool {
    let mut block = [raw[3], raw[4], raw[5]];
    let mut ok = true;
    for _ in 0..SCRAMBLE_ROUNDS {
        ok = unscramble_round(&mut block) && ok;
    }
    raw[3..6].copy_from_slice(&block);
    ok
}

fn uid(raw: &[u8; WIRE_BYTES]) -> u32 {
    u32::from_be_bytes([0, raw[0], raw[1], raw[2]])
}

/// audi_frame_valid: UID not all-zero / all-one.
fn frame_valid(raw: &[u8; WIRE_BYTES]) -> bool {
    let u = uid(raw);
    u != 0 && u != 0xFF_FFFF
}

/// Plaintext rolling counter (b3:b4 after un-scrambling), as the C's `cnt`.
fn plain_counter(raw: &[u8; WIRE_BYTES]) -> u16 {
    let mut plain = *raw;
    decrypt_block(&mut plain);
    u16::from_be_bytes([plain[3], plain[4]])
}

impl AudiDecoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            pending_high: 0,
            have_high: false,
            bytes: [0u8; WIRE_BYTES],
            bit_count: 0,
        }
    }

    fn is_short(d: u32) -> bool {
        duration_diff!(d, TE_SHORT) < TE_DELTA
    }

    fn is_long(d: u32) -> bool {
        duration_diff!(d, TE_LONG) < TE_DELTA
    }

    fn data_reset(&mut self) {
        self.bit_count = 0;
        self.have_high = false;
        self.pending_high = 0;
        self.bytes = [0u8; WIRE_BYTES];
    }

    /// LSB-first within each byte (audi_add_bit). Bits past 64 only bump the count, which makes
    /// the frame fail the exact-length gate.
    fn add_bit(&mut self, value: bool) {
        if self.bit_count < WIRE_BITS && value {
            self.bytes[self.bit_count >> 3] |= 1 << (self.bit_count & 7);
        }
        self.bit_count += 1;
    }

    /// Button name for display (audi_button_name).
    #[allow(dead_code)]
    fn button_name(btn: u8) -> &'static str {
        match btn {
            BTN_0 => "Lock",
            BTN_1 => "Unlock",
            BTN_2 => "Trunk",
            _ => "Unknown",
        }
    }

    /// KAT generic (or identical native) code → on-wire b6. `None` keeps the captured button.
    fn map_button(button: u8) -> Option<u8> {
        match button {
            BTN_0 | BTN_1 | BTN_2 => Some(button),
            _ => None,
        }
    }

    /// Full KAT validity gate for a 64-bit on-wire frame.
    fn validate(raw: &[u8; WIRE_BYTES]) -> Option<DecodedSignal> {
        if !frame_valid(raw) || raw[7] != checksum(raw) {
            return None;
        }
        if !matches!(raw[6], BTN_0 | BTN_1 | BTN_2) {
            return None;
        }
        Some(Self::build_signal(raw))
    }

    fn build_signal(raw: &[u8; WIRE_BYTES]) -> DecodedSignal {
        DecodedSignal {
            serial: Some(uid(raw)),
            button: Some(raw[6]),
            counter: Some(plain_counter(raw)),
            crc_valid: true,
            data: u64::from_be_bytes(*raw),
            data_count_bit: WIRE_BITS,
            encoder_capable: true,
            extra: None,
            protocol_display_name: None,
        }
    }

    /// End of frame (the C's audi_report + reset): emit only an exactly-64-bit valid frame.
    fn finish_frame(&mut self) -> Option<DecodedSignal> {
        let result = if self.bit_count == WIRE_BITS {
            Self::validate(&self.bytes)
        } else {
            None
        };
        self.data_reset();
        result
    }

    fn enc_add_level(signal: &mut Vec<LevelDuration>, level: bool, duration: u32) {
        if let Some(last) = signal.last_mut() {
            if last.level == level {
                *last = LevelDuration::new(level, last.duration_us + duration);
                return;
            }
        }
        signal.push(LevelDuration::new(level, duration));
    }

    /// One repeat of the C upload: header, 64 PWM bits LSB-first, inter-repeat gap.
    fn enc_frame(signal: &mut Vec<LevelDuration>, raw: &[u8; WIRE_BYTES]) {
        Self::enc_add_level(signal, true, HEADER_HIGH);
        Self::enc_add_level(signal, false, HEADER_LOW);
        for i in 0..WIRE_BITS {
            if (raw[i >> 3] >> (i & 7)) & 1 != 0 {
                Self::enc_add_level(signal, true, TE_LONG);
                Self::enc_add_level(signal, false, TE_SHORT);
            } else {
                Self::enc_add_level(signal, true, TE_SHORT);
                Self::enc_add_level(signal, false, TE_LONG);
            }
        }
        Self::enc_add_level(signal, false, GAP_US);
    }
}

impl ProtocolDecoder for AudiDecoder {
    fn name(&self) -> &'static str {
        "Audi"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: WIRE_BITS,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        // C flags: SubGhzProtocolFlag_315 | _433 | _868 (AM).
        &[315_000_000, 433_920_000, 868_350_000]
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.data_reset();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::Reset => {
                if level && duration >= HEADER_MIN_US {
                    self.step = DecoderStep::HeaderLow;
                }
                None
            }
            DecoderStep::HeaderLow => {
                if !level && duration_diff!(duration, HEADER_LOW) < TE_DELTA * 2 {
                    self.step = DecoderStep::Data;
                    self.data_reset();
                } else {
                    self.step = DecoderStep::Reset;
                }
                None
            }
            DecoderStep::Data => {
                if level {
                    if duration >= HEADER_MIN_US {
                        // A new header mid-stream: close the frame, re-sync on this header.
                        let result = self.finish_frame();
                        self.step = DecoderStep::HeaderLow;
                        return result;
                    }
                    if Self::is_short(duration) || Self::is_long(duration) {
                        self.pending_high = duration;
                        self.have_high = true;
                        return None;
                    }
                    let result = self.finish_frame();
                    self.step = DecoderStep::Reset;
                    return result;
                }

                if !self.have_high {
                    // A lone LOW (no preceding HIGH): end of frame / gap.
                    let result = self.finish_frame();
                    self.step = DecoderStep::Reset;
                    return result;
                }
                self.have_high = false;

                let high_long = Self::is_long(self.pending_high);
                let high_short = Self::is_short(self.pending_high);
                if high_long && Self::is_short(duration) {
                    self.add_bit(true);
                    None
                } else if high_short && Self::is_long(duration) {
                    self.add_bit(false);
                    None
                } else if duration > GAP_MIN_US {
                    // Last bit followed by the inter-repeat gap: its LOW merged with the gap, so
                    // the HIGH width alone gives the bit (the C only handles the bit-0 case).
                    self.add_bit(high_long);
                    let result = self.finish_frame();
                    self.step = DecoderStep::Reset;
                    result
                } else {
                    let result = self.finish_frame();
                    self.step = DecoderStep::Reset;
                    result
                }
            }
        }
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        let mut raw = decoded.data.to_be_bytes();
        if !frame_valid(&raw) {
            return None;
        }

        let original_btn = raw[6];
        let new_btn = Self::map_button(button).unwrap_or(original_btn);
        if new_btn != original_btn {
            // audi_reencode: advance the plaintext counter for a genuine (non-replay) press.
            let base = decoded.counter.unwrap_or_else(|| plain_counter(&raw));
            let new_cnt = base.wrapping_add(1);
            let mut plain = raw;
            decrypt_block(&mut plain);
            plain[3..5].copy_from_slice(&new_cnt.to_be_bytes());
            plain[6] = new_btn;
            encrypt(&mut plain);
            raw = plain;
        }

        let mut signal = Vec::with_capacity(REPEATS * (2 + WIRE_BITS * 2 + 1));
        for _ in 0..REPEATS {
            Self::enc_frame(&mut signal, &raw);
        }
        Some(signal)
    }
}

impl Default for AudiDecoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::vag::VagDecoder;

    /// Build the on-wire bytes for (uid, plaintext b3..b5, button) like the C encoder.
    fn wire_frame(uid24: u32, plain345: [u8; 3], btn: u8) -> [u8; WIRE_BYTES] {
        let u = uid24.to_be_bytes();
        let mut raw = [
            u[1],
            u[2],
            u[3],
            plain345[0],
            plain345[1],
            plain345[2],
            btn,
            0,
        ];
        encrypt(&mut raw);
        raw
    }

    fn decode_all(dec: &mut dyn ProtocolDecoder, signal: &[LevelDuration]) -> Vec<DecodedSignal> {
        dec.reset();
        let mut out = Vec::new();
        for ld in signal {
            if let Some(d) = dec.feed(ld.level, ld.duration_us) {
                out.push(d);
                dec.reset();
            }
        }
        out
    }

    fn one_frame(raw: &[u8; WIRE_BYTES]) -> Vec<LevelDuration> {
        let mut s = Vec::new();
        AudiDecoder::enc_frame(&mut s, raw);
        s
    }

    /// `audi_encoder_build_upload` + `audi_append_data_pairs`, transcribed verbatim from the C
    /// (no merging of adjacent same-level entries, exactly as the C fills its upload array).
    fn c_build_upload(raw: &[u8; WIRE_BYTES]) -> Vec<LevelDuration> {
        let mut upload = Vec::new();
        for _ in 0..REPEATS {
            upload.push(LevelDuration::new(true, HEADER_HIGH));
            upload.push(LevelDuration::new(false, HEADER_LOW));
            for bit_index in 0..WIRE_BITS {
                let bit = ((raw[bit_index >> 3] >> (bit_index & 7)) & 1) != 0;
                if bit {
                    upload.push(LevelDuration::new(true, TE_LONG));
                    upload.push(LevelDuration::new(false, TE_SHORT));
                } else {
                    upload.push(LevelDuration::new(true, TE_SHORT));
                    upload.push(LevelDuration::new(false, TE_LONG));
                }
            }
            upload.push(LevelDuration::new(false, GAP_US));
        }
        upload
    }

    /// The C's own upload (unmerged: the last bit's LOW and the 850µs gap arrive as two LOWs)
    /// decodes every repeat, for frames ending in either bit value.
    #[test]
    fn decodes_verbatim_c_upload() {
        for (uid24, plain, btn) in [
            (0x12_3456u32, [0x00u8, 0x2A, 0x77], BTN_0),
            (0xAB_CDEF, [0x12, 0x34, 0x56], BTN_1),
            (0x31_4159, [0x26, 0x53, 0x58], BTN_2),
            (0x00_0001, [0xFF, 0xFF, 0x00], BTN_1),
        ] {
            let raw = wire_frame(uid24, plain, btn);
            let frames = decode_all(&mut AudiDecoder::new(), &c_build_upload(&raw));
            assert_eq!(
                frames.len(),
                REPEATS,
                "uid {uid24:#X} (b7 = {:#04X})",
                raw[7]
            );
            assert!(frames.iter().all(|f| f.data == u64::from_be_bytes(raw)));
        }
    }

    #[test]
    fn scramble_is_a_bijection_and_inverts() {
        let mut seed: u32 = 0xA5A5_1234;
        for _ in 0..200_000 {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let block = [(seed >> 8) as u8, (seed >> 16) as u8, (seed >> 24) as u8];
            let mut b = block;
            scramble_round(&mut b);
            assert!(unscramble_round(&mut b));
            assert_eq!(b, block, "unscramble(scramble(x)) == x");
            let mut b = block;
            unscramble_round(&mut b);
            scramble_round(&mut b);
            assert_eq!(b, block, "scramble(unscramble(x)) == x");
        }
    }

    /// The C's "P:OK/BAD" flag can never be BAD: un-scrambling solves for the dropped bit so the
    /// recomputed parity always equals the seed. It carries no validation strength.
    #[test]
    fn parity_self_check_is_vacuous() {
        let mut seed: u32 = 0x0BAD_CAFE;
        for _ in 0..50_000 {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let b = seed.to_le_bytes();
            let mut raw: [u8; WIRE_BYTES] = [b[0], b[1], b[2], b[3], b[1], b[2], b[3], b[0]];
            assert!(decrypt_block(&mut raw));
        }
    }

    #[test]
    fn encode_decode_roundtrip_replay() {
        // Both final-bit values (b7 MSB 0/1) must round-trip (the C drops the b7.7 = 1 case).
        let mut seen_msb = [false; 2];
        for (uid24, plain) in [
            (0x12_3456u32, [0x00u8, 0x2A, 0x77]),
            (0xAB_CDEF, [0x12, 0x34, 0x56]),
            (0x00_0001, [0xFF, 0xFF, 0x00]),
            (0x7F_FFFE, [0x80, 0x01, 0xC3]),
            (0x55_AA55, [0x01, 0x00, 0x10]),
            (0x31_4159, [0x26, 0x53, 0x58]),
        ] {
            for btn in [BTN_0, BTN_1, BTN_2] {
                let raw = wire_frame(uid24, plain, btn);
                seen_msb[(raw[7] >> 7) as usize] = true;
                let captured = AudiDecoder::build_signal(&raw);
                let sig = AudiDecoder::new().encode(&captured, btn).expect("encode");
                let frames = decode_all(&mut AudiDecoder::new(), &sig);
                assert_eq!(frames.len(), REPEATS, "uid {uid24:#X} btn {btn:#X}");
                for f in &frames {
                    assert_eq!(f.data, u64::from_be_bytes(raw), "byte-identical replay");
                    assert_eq!(f.serial, Some(uid24));
                    assert_eq!(f.button, Some(btn));
                    assert_eq!(f.counter, Some(u16::from_be_bytes([plain[0], plain[1]])));
                    assert!(f.crc_valid && f.encoder_capable);
                }
            }
        }
        assert!(seen_msb[0] && seen_msb[1], "cover both final-bit values");
    }

    #[test]
    fn encode_button_change_rescrambles_and_advances_counter() {
        let raw = wire_frame(0x12_3456, [0x00, 0x2A, 0x77], BTN_0);
        let captured = AudiDecoder::build_signal(&raw);
        let sig = AudiDecoder::new().encode(&captured, 0x02).expect("encode");
        let f = decode_all(&mut AudiDecoder::new(), &sig).remove(0);
        assert_eq!(f.button, Some(BTN_1));
        assert_eq!(f.counter, Some(0x002B));
        assert_eq!(f.serial, Some(0x12_3456));
        // b5 plaintext is preserved across the re-encode.
        let mut plain = f.data.to_be_bytes();
        decrypt_block(&mut plain);
        assert_eq!(plain[5], 0x77);
        // Panic (no Audi code) keeps the captured button: exact replay.
        let sig = AudiDecoder::new().encode(&captured, 0x08).expect("encode");
        let f = decode_all(&mut AudiDecoder::new(), &sig).remove(0);
        assert_eq!(f.data, u64::from_be_bytes(raw));
    }

    #[test]
    fn synthetic_frame_decodes_known_fields() {
        let raw = wire_frame(0xC0_FFEE, [0xBE, 0xEF, 0x01], BTN_2);
        let frames = decode_all(&mut AudiDecoder::new(), &one_frame(&raw));
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].serial, Some(0xC0_FFEE));
        assert_eq!(frames[0].button, Some(BTN_2));
        assert_eq!(frames[0].counter, Some(0xBEEF));
        assert_eq!(frames[0].data_count_bit, 64);

        // Waveform pinned to the C: header, then b0 = 0xC0 LSB-first = six 0-bits (HIGH 550 +
        // LOW 1100) followed by two 1-bits (HIGH 1100 + LOW 550).
        let s = one_frame(&raw);
        let got: Vec<(bool, u32)> = s[..16].iter().map(|p| (p.level, p.duration_us)).collect();
        let mut want = vec![(true, HEADER_HIGH), (false, HEADER_LOW)];
        for _ in 0..6 {
            want.extend([(true, TE_SHORT), (false, TE_LONG)]);
        }
        want.extend([(true, TE_LONG), (false, TE_SHORT)]);
        assert_eq!(got, want);
    }

    #[test]
    fn rejects_bad_checksum_button_and_length() {
        let raw = wire_frame(0x12_3456, [0x00, 0x2A, 0x77], BTN_0);

        let mut bad = raw;
        bad[1] ^= 0x04; // one flipped bit → checksum mismatch
        assert!(
            decode_all(&mut AudiDecoder::new(), &one_frame(&bad)).is_empty(),
            "checksum"
        );

        let mut bad_btn = raw;
        bad_btn[6] = 0x08;
        bad_btn[7] = checksum(&bad_btn);
        assert!(
            decode_all(&mut AudiDecoder::new(), &one_frame(&bad_btn)).is_empty(),
            "button"
        );

        // 65 PWM bits (an extra bit 0 appended before the gap) → not exactly 64.
        let mut s = Vec::new();
        AudiDecoder::enc_add_level(&mut s, true, HEADER_HIGH);
        AudiDecoder::enc_add_level(&mut s, false, HEADER_LOW);
        for i in 0..WIRE_BITS + 1 {
            let one = i < WIRE_BITS && (raw[i >> 3] >> (i & 7)) & 1 != 0;
            AudiDecoder::enc_add_level(&mut s, true, if one { TE_LONG } else { TE_SHORT });
            AudiDecoder::enc_add_level(&mut s, false, if one { TE_SHORT } else { TE_LONG });
        }
        AudiDecoder::enc_add_level(&mut s, false, GAP_US);
        assert!(
            decode_all(&mut AudiDecoder::new(), &s).is_empty(),
            "65 bits"
        );

        // Truncated: 63 bits then the gap.
        let mut s = one_frame(&raw);
        s.truncate(2 + 63 * 2);
        AudiDecoder::enc_add_level(&mut s, false, 5000);
        assert!(
            decode_all(&mut AudiDecoder::new(), &s).is_empty(),
            "63 bits"
        );
    }

    /// Audi must not steal VAG frames: KAT's VAG Type 2 encoder output (300/600µs Manchester,
    /// keyless TEA) and a Type 3/4-shaped burst (500/1000µs Manchester, layout of
    /// `encode_type3_4`; the real one needs AUT64 keys) must not decode as Audi in either
    /// polarity, for many payloads.
    #[test]
    fn does_not_decode_vag_waveforms() {
        let vag = VagDecoder::new();
        let mut seed: u32 = 0x5EED_0001;
        let mut tested_type2 = 0;
        for _ in 0..200 {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let hi = seed;
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let key1 = ((hi as u64) << 32) | seed as u64;
            let captured = DecodedSignal {
                serial: Some(hi),
                button: Some(0x20),
                counter: Some(seed as u16),
                crc_valid: true,
                data: key1,
                data_count_bit: 80,
                encoder_capable: true,
                extra: Some(2),
                protocol_display_name: None,
            };
            let mut waveforms = Vec::new();
            if let Some(sig) = vag.encode(&captured, 0x02) {
                tested_type2 += 1;
                waveforms.push(sig);
            }
            let mut t34 = Vec::new();
            for _ in 0..2 {
                for _ in 0..45 {
                    t34.push(LevelDuration::new(true, 500));
                    t34.push(LevelDuration::new(false, 500));
                }
                t34.push(LevelDuration::new(true, 1000));
                t34.push(LevelDuration::new(false, 500));
                for _ in 0..3 {
                    t34.push(LevelDuration::new(true, 750));
                    t34.push(LevelDuration::new(false, 750));
                }
                for i in (0..80).rev() {
                    let one = i >= 16 && (key1 >> (i - 16)) & 1 == 1;
                    t34.push(LevelDuration::new(one, 500));
                    t34.push(LevelDuration::new(!one, 500));
                }
                t34.push(LevelDuration::new(false, 6000));
            }
            waveforms.push(t34);
            for sig in &waveforms {
                assert!(decode_all(&mut AudiDecoder::new(), sig).is_empty());
                let inv: Vec<LevelDuration> = sig
                    .iter()
                    .map(|p| LevelDuration::new(!p.level, p.duration_us))
                    .collect();
                assert!(decode_all(&mut AudiDecoder::new(), &inv).is_empty());
            }
        }
        assert!(tested_type2 > 0, "VAG Type 2 encoder produced waveforms");
    }

    /// Content-gate strength: random 8-byte frames pass UID + checksum + button with
    /// p ≈ (1/256)·(3/256) ≈ 4.6e-5 (expected ≈ 46 of 1M).
    #[test]
    fn content_gate_rate_matches_estimate() {
        let mut seed: u32 = 0x1234_ABCD;
        let mut next = || {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            seed
        };
        let mut passed = 0;
        for _ in 0..1_000_000 {
            let (a, b) = (next().to_le_bytes(), next().to_le_bytes());
            let raw = [a[0], a[1], a[2], a[3], b[0], b[1], b[2], b[3]];
            if AudiDecoder::validate(&raw).is_some() {
                passed += 1;
            }
        }
        assert!((10..=120).contains(&passed), "passed {passed} of 1M");
    }

    /// Random pulse noise (uniform 50..6000µs, and runs snapped to 550/1100µs with 2500/850µs
    /// headers sprinkled in) never decodes.
    #[test]
    fn random_noise_never_decodes() {
        let mut seed: u32 = 0xDEAD_BEEF;
        let mut level = true;
        let mut sig = Vec::with_capacity(200_000);
        for i in 0..200_000u32 {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let r = seed;
            let d = if (i / 1000) % 2 == 0 {
                50 + r % 5950
            } else if r % 97 == 0 {
                if level {
                    HEADER_HIGH
                } else {
                    HEADER_LOW
                }
            } else {
                (if r & 1 == 0 { TE_SHORT } else { TE_LONG }) + (r >> 8) % 160 - 80
            };
            sig.push(LevelDuration::new(level, d));
            level = !level;
        }
        assert!(decode_all(&mut AudiDecoder::new(), &sig).is_empty());
    }

    /// The real VAG captures (incl. the five that decode as nothing today) must not produce an
    /// Audi frame, standalone (no registry masking) and in either polarity.
    #[test]
    fn no_decode_on_vag_imports() {
        let Ok(dir) = std::fs::read_dir("IMPORTS/VAG") else {
            eprintln!("Skip: IMPORTS/VAG not found (run from crate root)");
            return;
        };
        let mut checked = 0;
        for entry in dir.flatten() {
            let path = entry.path();
            if path.extension().map_or(true, |e| e != "sub") {
                continue;
            }
            let Ok((_, raw)) = crate::export::flipper::import_sub_raw(&path) else {
                continue;
            };
            for inv in [false, true] {
                let sig: Vec<LevelDuration> = raw
                    .iter()
                    .map(|p| LevelDuration::new(p.level != inv, p.duration_us))
                    .collect();
                assert!(
                    decode_all(&mut AudiDecoder::new(), &sig).is_empty(),
                    "{path:?} inv={inv}"
                );
            }
            checked += 1;
        }
        assert!(
            checked >= 5,
            "expected the VAG RAW captures, checked {checked}"
        );
    }
}
