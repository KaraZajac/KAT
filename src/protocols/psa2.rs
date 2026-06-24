//! PSA2 (Peugeot/Citroën — "PSA OLD") protocol decoder/encoder
//!
//! Aligned with Flipper-ARF reference: `lib/subghz/protocols/psa2.c` and `psa2.h`
//! (internal name `SUBGHZ_PROTOCOL_PSA2_NAME = "PSA OLD"`). This is the OLDER PSA variant,
//! distinct from KAT's existing `psa` (modified-TEA/XEA) decoder.
//!
//! Protocol characteristics:
//! - Manchester encoding: 250/500µs symbol (Pattern 1, standard rate) or 125/250µs (Pattern 2,
//!   half rate). Canonical Flipper `manchester_advance` table (events ShortLow=0, ShortHigh=2,
//!   LongLow=4, LongHigh=6), seeded `ManchesterStateMid1`.
//! - 128-bit frame = key1 (64 bits) + key2/validation word. The decoder collects 64 bits → key1,
//!   then 16 more (to 80 bits = `KEY2_BITS`) → the 16-bit validation field / key2_low.
//! - RF: AM (OOK). Frequency: 433.92 MHz.
//! - Crypto: TEA (Tiny Encryption Algorithm) with a dual brute-force fallback (BF1
//!   0x23000000–0x24000000, BF2 0xF3000000–0xF4000000) and a mode23/mode36 selector, validated
//!   via a nibble checksum.
//!
//! ## Decoder structure & PERFORMANCE
//! The C live decoder (`subghz_protocol_decoder_psa2_feed`) only ever runs the cheap mode23 XOR
//! path (`psa_decrypt_fast`) per frame — it NEVER runs the TEA brute force. The brute force
//! (`psa_decrypt_full`, marked `__attribute__((unused))`) is reserved for the deferred-decrypt
//! UI button, not the streaming decoder. KAT mirrors this exactly. Per frame:
//!
//! 1. Manchester-collect to exactly 80 bits (key1 + validation) with end-of-packet detection.
//! 2. Run the O(1) `direct_xor_decrypt` (mode23) gated on its nibble checksum.
//! 3. Emit only on a successful, field-bearing decrypt (see `finalize_frame` for why the C's bare
//!    `(validation & 0xF) == 0xA` emission is not reproduced in KAT's feed-all model).
//!
//! The bounded TEA brute force (BF1/BF2, ~16.7M iters each) is ported faithfully in
//! `Psa2Decoder::decrypt_full` but is NEVER called from `feed()` — only the cheap O(1) gate runs
//! per pulse, so the test sweep stays fast.
//!
//! Decoder steps: State0 (wait preamble) → State1/State3 (count preamble pulses) →
//! State2/State4 (Manchester decode + decrypt). Encoder supported (mode23 path).

use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

// Standard-rate timings (Pattern 1)
const TE_SHORT: u32 = 250;
const TE_LONG: u32 = 500;
const TE_DELTA: u32 = 100;
const MIN_COUNT_BIT: usize = 128;

// Half-rate timings (Pattern 2 / State 3-4)
const TE_SHORT_HALF: u32 = 125;
const TE_LONG_HALF: u32 = 250;
const TOL_HALF: u32 = 50;

// End-of-packet markers
const TE_END_1000: u32 = 1000;
const TE_END_500: u32 = 500;

// Bit counts
const KEY1_BITS: usize = 64; // 0x40
const KEY2_BITS: usize = 80; // 0x50
const MAX_BITS: usize = 121; // 0x79

// Preamble pulse-count thresholds (C: PSA_PATTERN_THRESHOLD_1/2)
const PATTERN_THRESHOLD_1: u16 = 0x46;
const PATTERN_THRESHOLD_2: u16 = 0x45;

// Validation nibble for the mode23 path: (validation_field & 0xF) == 0xA
const VALID_NIBBLE: u16 = 0xA;

// "decrypted" success marker (C uses 0x50)
const DECRYPTED_OK: u16 = 0x50;

// Mode selectors (stored as ASCII chars in firmware)
const MODE_23: u8 = 0x23; // '#'
const MODE_36: u8 = 0x36; // '6'

// PSA2 button codes are Lock=0, Unlock=1, Trunk=2 (psa_button_name). Decodes whose 4-bit button
// exceeds this are coincidental matches on unrelated Manchester data and are rejected.
const BTN_MAX_VALID: u8 = 0x2;

// TEA constants
const TEA_DELTA: u32 = 0x9E3779B9;
const TEA_ROUNDS: u32 = 32;

// BF1 brute-force range + constants (FUN_08028f94 / FUN_080291c0)
const BF1_START: u32 = 0x2300_0000;
const BF1_END: u32 = 0x2400_0000;
const BF1_CONST_U4: u32 = 0x0E0F_5C41;
const BF1_CONST_U5: u32 = 0x0F5C_4123;
const BF1_KEY_SCHEDULE: [u32; 4] = [0x4A43_4915, 0xD674_3C2B, 0x1F29_D308, 0xE6B7_9A64];

// BF2 brute-force range + key schedule (FUN_080290f8)
const BF2_START: u32 = 0xF300_0000;
const BF2_END: u32 = 0xF400_0000;
const BF2_KEY_SCHEDULE: [u32; 4] = [0x4039_C240, 0xEDA9_2CAB, 0x4306_C02A, 0x0219_2A04];

/// Canonical Flipper Manchester states (lib/toolbox/manchester_decoder.c order:
/// Start1=0, Mid1=1, Mid0=2, Start0=3).
#[derive(Debug, Clone, Copy, PartialEq)]
enum ManchesterState {
    Start1 = 0,
    Mid1 = 1,
    Mid0 = 2,
    Start0 = 3,
}

/// Decoder states (matches PSADecoderState0-4 in psa2.c).
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderState {
    /// State0: wait for preamble start.
    WaitEdge,
    /// State1: count standard-rate (250µs) preamble pulses.
    CountPattern250,
    /// State2: receive key1 + key2/validation at standard rate.
    DecodeManchester250,
    /// State3: count half-rate (125µs) preamble pulses.
    CountPattern125,
    /// State4: receive key1 + key2/validation at half rate.
    DecodeManchester125,
}

/// Result of a successful decrypt: (serial, button, counter, crc, type).
type DecryptResult = (u32, u8, u32, u16, u8);

/// PSA2 ("PSA OLD") protocol decoder.
pub struct Psa2Decoder {
    state: DecoderState,
    prev_duration: u32,
    manchester_state: ManchesterState,
    pattern_counter: u16,
    data_low: u32,
    data_high: u32,
    bit_count: usize,
    // Decoded fields
    key1_low: u32,
    key1_high: u32,
    validation_field: u16,
    key2_low: u32,
    key2_high: u32,
}

impl Psa2Decoder {
    pub fn new() -> Self {
        Self {
            state: DecoderState::WaitEdge,
            prev_duration: 0,
            manchester_state: ManchesterState::Mid1,
            pattern_counter: 0,
            data_low: 0,
            data_high: 0,
            bit_count: 0,
            key1_low: 0,
            key1_high: 0,
            validation_field: 0,
            key2_low: 0,
            key2_high: 0,
        }
    }

    fn near(dur: u32, target: u32, tol: u32) -> bool {
        duration_diff!(dur, target) <= tol
    }

    // =========================================================================
    // Manchester — canonical Flipper transition table (transitions[] in
    // manchester_decoder.c). `event` ∈ {0,2,4,6}; returns Some(bit) on emit.
    // =========================================================================
    fn manchester_advance(&mut self, event: u8) -> Option<bool> {
        const TRANSITIONS: [u8; 4] = [0b0000_0001, 0b1001_0001, 0b1001_1011, 0b1111_1011];
        let state_idx = self.manchester_state as usize;
        let new_idx = (TRANSITIONS[state_idx] >> event) & 0x3;
        let new_state = match new_idx {
            0 => ManchesterState::Start1,
            1 => ManchesterState::Mid1,
            2 => ManchesterState::Mid0,
            _ => ManchesterState::Start0,
        };

        if new_idx as usize == state_idx {
            // No progress → reset to Mid1, emit nothing.
            self.manchester_state = ManchesterState::Mid1;
            return None;
        }
        self.manchester_state = new_state;
        match new_state {
            ManchesterState::Mid0 => Some(false),
            ManchesterState::Mid1 => Some(true),
            _ => None,
        }
    }

    fn manchester_reset(&mut self) {
        self.manchester_state = ManchesterState::Mid1;
    }

    /// Shift one decoded bit into the 64-bit accumulator (data_high:data_low),
    /// latching key1 at 64 bits (matches psa2.c State2/State4 add path).
    fn add_bit(&mut self, bit: bool) {
        let carry = (self.data_low >> 31) & 1;
        self.data_low = (self.data_low << 1) | (bit as u32);
        self.data_high = (self.data_high << 1) | carry;
        self.bit_count += 1;
        if self.bit_count == KEY1_BITS {
            self.key1_low = self.data_low;
            self.key1_high = self.data_high;
            self.data_low = 0;
            self.data_high = 0;
        }
    }

    fn init_preamble_state(&mut self) {
        self.data_low = 0;
        self.data_high = 0;
        self.pattern_counter = 0;
        self.bit_count = 0;
        self.manchester_reset();
    }

    // =========================================================================
    // CRYPTO PRIMITIVES (faithful to psa2.c)
    // =========================================================================

    /// TEA encrypt (FUN_08028e14): dynamic key index `sum&3` then `(sum>>11)&3`.
    fn tea_encrypt(v0: &mut u32, v1: &mut u32, key: &[u32; 4]) {
        let (mut a, mut b) = (*v0, *v1);
        let mut sum: u32 = 0;
        for _ in 0..TEA_ROUNDS {
            let t = key[(sum & 3) as usize].wrapping_add(sum);
            sum = sum.wrapping_add(TEA_DELTA);
            a = a.wrapping_add(t ^ ((b >> 5) ^ (b << 4)).wrapping_add(b));
            let t = key[((sum >> 11) & 3) as usize].wrapping_add(sum);
            b = b.wrapping_add(t ^ ((a >> 5) ^ (a << 4)).wrapping_add(a));
        }
        *v0 = a;
        *v1 = b;
    }

    /// TEA decrypt (FUN_08028e14 inverse): unwinds the encrypt rounds.
    fn tea_decrypt(v0: &mut u32, v1: &mut u32, key: &[u32; 4]) {
        let (mut a, mut b) = (*v0, *v1);
        let mut sum: u32 = TEA_DELTA.wrapping_mul(TEA_ROUNDS);
        for _ in 0..TEA_ROUNDS {
            let t = key[((sum >> 11) & 3) as usize].wrapping_add(sum);
            sum = sum.wrapping_sub(TEA_DELTA);
            b = b.wrapping_sub(t ^ ((a >> 5) ^ (a << 4)).wrapping_add(a));
            let t = key[(sum & 3) as usize].wrapping_add(sum);
            a = a.wrapping_sub(t ^ ((b >> 5) ^ (b << 4)).wrapping_add(b));
        }
        *v0 = a;
        *v1 = b;
    }

    /// Byte-sum CRC over 7 bytes of TEA output (FUN_08028e60).
    fn calculate_tea_crc(v0: u32, v1: u32) -> u8 {
        let mut crc: u32 = ((v0 >> 24) & 0xFF) + ((v0 >> 16) & 0xFF) + ((v0 >> 8) & 0xFF) + (v0 & 0xFF);
        crc += ((v1 >> 24) & 0xFF) + ((v1 >> 16) & 0xFF) + ((v1 >> 8) & 0xFF);
        (crc & 0xFF) as u8
    }

    /// CRC-16/BUYPASS (poly 0x8005, init 0, no reflection) (FUN_08029098).
    fn calculate_crc16_bf2(data: &[u8]) -> u16 {
        let mut crc: u16 = 0;
        for &byte in data {
            crc ^= (byte as u16) << 8;
            for _ in 0..8 {
                if crc & 0x8000 != 0 {
                    crc = (crc << 1) ^ 0x8005;
                } else {
                    crc <<= 1;
                }
            }
        }
        crc
    }

    /// Fill buf[0..9] from key1/key2 (FUN_080291c0 first loop / psa_setup_byte_buffer).
    /// key1 big-endian reversed into buf[7..0]; key2_low low/high bytes into buf[9]/buf[8].
    fn setup_byte_buffer(buf: &mut [u8], key1_low: u32, key1_high: u32, key2_low: u32) {
        for i in 0..8usize {
            let shift = i * 8;
            let b = if shift < 32 {
                (key1_low >> shift) as u8
            } else {
                (key1_high >> (shift - 32)) as u8
            };
            buf[7 - i] = b;
        }
        buf[9] = (key2_low & 0xFF) as u8;
        buf[8] = ((key2_low >> 8) & 0xFF) as u8;
    }

    /// Nibble checksum over buf[2..8] → buf[11] (FUN_08028cf8).
    fn calculate_checksum(buf: &mut [u8]) {
        let mut sum: u32 = 0;
        for &b in buf.iter().take(8).skip(2) {
            sum += (b & 0xF) as u32 + ((b >> 4) & 0xF) as u32;
        }
        buf[11] = (sum.wrapping_mul(0x10) & 0xFF) as u8;
    }

    /// XOR decrypt second stage (FUN_08028d54 + psa_copy_reverse FUN_08028d24).
    fn second_stage_xor_decrypt(buf: &mut [u8]) {
        // psa_copy_reverse
        let t = [
            buf[5], buf[4], buf[3], buf[2], buf[9], buf[8], buf[7], buf[6],
        ];
        buf[2] = t[0] ^ t[6];
        buf[3] = t[2] ^ t[0];
        buf[4] = t[6] ^ t[3];
        buf[5] = t[7] ^ t[1];
        buf[6] = t[3] ^ t[1];
        buf[7] = t[6] ^ t[4] ^ t[5];
    }

    /// Inverse of `second_stage_xor_decrypt`, used by the encoder.
    fn second_stage_xor_encrypt(buf: &mut [u8]) {
        let e6 = buf[8];
        let e7 = buf[9];
        let (p0, p1, p2, p3, p4, p5) = (buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
        let e5 = p5 ^ e7 ^ e6;
        let e0 = p2 ^ e5;
        let e2 = p4 ^ e0;
        let e4 = p3 ^ e2;
        let e3 = p0 ^ e5;
        let e1 = p1 ^ e3;
        buf[2] = e0;
        buf[3] = e1;
        buf[4] = e2;
        buf[5] = e3;
        buf[6] = e4;
        buf[7] = e5;
    }

    /// Pack buf[2..9] into two TEA words (FUN_08028f4c).
    fn prepare_tea_data(buf: &[u8]) -> (u32, u32) {
        let w0 = ((buf[2] as u32) << 24) | ((buf[3] as u32) << 16) | ((buf[4] as u32) << 8) | buf[5] as u32;
        let w1 = ((buf[6] as u32) << 24) | ((buf[7] as u32) << 16) | ((buf[8] as u32) << 8) | buf[9] as u32;
        (w0, w1)
    }

    /// Unpack two TEA words back into buf[2..9] (FUN_08028e88).
    fn unpack_tea_result(buf: &mut [u8], v0: u32, v1: u32) {
        buf[2] = (v0 >> 24) as u8;
        buf[3] = (v0 >> 16) as u8;
        buf[4] = (v0 >> 8) as u8;
        buf[5] = v0 as u8;
        buf[6] = (v1 >> 24) as u8;
        buf[7] = (v1 >> 16) as u8;
        buf[8] = (v1 >> 8) as u8;
        buf[9] = v1 as u8;
    }

    // =========================================================================
    // FIELD EXTRACTION (FUN_08028f10)
    // =========================================================================

    fn extract_fields_mode23(buf: &[u8]) -> DecryptResult {
        let button = buf[8] & 0xF;
        let serial = ((buf[2] as u32) << 16) | ((buf[3] as u32) << 8) | buf[4] as u32;
        let counter = (buf[6] as u32) | ((buf[5] as u32) << 8);
        let crc = buf[7] as u16;
        (serial, button, counter, crc, MODE_23)
    }

    fn extract_fields_mode36(buf: &[u8]) -> DecryptResult {
        let button = (buf[5] >> 4) & 0xF;
        let serial = ((buf[2] as u32) << 16) | ((buf[3] as u32) << 8) | buf[4] as u32;
        let counter = ((buf[7] as u32) << 8)
            | ((buf[6] as u32) << 16)
            | (buf[8] as u32)
            | (((buf[5] as u32) & 0xF) << 24);
        let crc = buf[9] as u16;
        (serial, button, counter, crc, MODE_36)
    }

    // =========================================================================
    // DECRYPTION PATHS (faithful to psa2.c)
    // =========================================================================

    /// key2-high gate (matches KAT's sibling `psa::direct_xor_allowed_by_key2`). The PSA2 C
    /// `psa_direct_xor_decrypt` validates only on the 4-bit checksum nibble `(checksum ^ key2_high)
    /// & 0xF0 == 0`, which has a ~1/16 false-positive rate on arbitrary 80-bit Manchester data. In
    /// KAT's "feed every frequency-compatible decoder, first match wins" model that lets PSA2 steal
    /// unrelated 250/500µs frames (e.g. VAG at 434 MHz). This precondition — the exact filter KAT's
    /// existing `psa` decoder already applies before its XOR path — is added on top of the C gate to
    /// suppress those false positives without dropping any genuine PSA2 decode.
    fn direct_xor_allowed_by_key2(key2_high_byte: u8) -> bool {
        let lo = key2_high_byte & 0xF;
        if lo < 3 {
            return true;
        }
        if lo < 7 && (key2_high_byte & 0xC) != 0 {
            return true;
        }
        false
    }

    /// mode23 XOR path (FUN_08028d98 / psa_direct_xor_decrypt). O(1) — the only path the
    /// live decoder runs. Returns the extracted fields on checksum validation.
    ///
    /// Beyond the C's nibble-checksum gate, this applies two false-positive suppressors required by
    /// KAT's feed-all-decoders model: the `direct_xor_allowed_by_key2` precondition and a valid-
    /// button check (PSA2 buttons are Lock=0, Unlock=1, Trunk=2 only — see `psa_button_name`). Both
    /// cleanly separate genuine PSA2 frames from coincidental matches on unrelated Manchester data.
    fn direct_xor_decrypt(key1_low: u32, key1_high: u32, key2_low: u32) -> Option<DecryptResult> {
        let mut buf = [0u8; 48];
        Self::setup_byte_buffer(&mut buf, key1_low, key1_high, key2_low);

        let key2_high = buf[8];
        if !Self::direct_xor_allowed_by_key2(key2_high) {
            return None;
        }

        Self::calculate_checksum(&mut buf);
        let checksum = buf[11];
        let validation = (checksum ^ key2_high) & 0xF0;

        if validation == 0 {
            // Firmware: update buf[8] high nibble before XOR stage.
            buf[8] = (buf[8] & 0x0F) | (checksum & 0xF0);
            buf[13] = buf[9] ^ buf[8];
            Self::second_stage_xor_decrypt(&mut buf);
            let fields = Self::extract_fields_mode23(&buf);
            // Reject implausible button codes (PSA2: Lock=0/Unlock=1/Trunk=2).
            if fields.1 > BTN_MAX_VALID {
                return None;
            }
            return Some(fields);
        }
        None
    }

    /// BF1 brute force, range 0x23000000–0x24000000 (FUN_08028f94).
    ///
    /// PERFORMANCE: up to ~16.7M TEA iterations. NEVER called from `feed()`. Only invoked from
    /// [`Self::decrypt_full`], which itself runs only after the cheap structural gate. Returns
    /// `(result, seed)` on success.
    fn brute_force_decrypt_bf1(key1_low: u32, key1_high: u32, key2_low: u32) -> Option<(DecryptResult, u32)> {
        let mut buf = [0u8; 48];
        Self::setup_byte_buffer(&mut buf, key1_low, key1_high, key2_low);
        let (w0, w1) = Self::prepare_tea_data(&buf);

        for counter in BF1_START..BF1_END {
            // Derive the working key with two TEA encrypts.
            let (mut wk2, mut wk3) = (BF1_CONST_U4, counter);
            Self::tea_encrypt(&mut wk2, &mut wk3, &BF1_KEY_SCHEDULE);
            let (mut wk0, mut wk1) = ((counter << 8) | 0x0E, BF1_CONST_U5);
            Self::tea_encrypt(&mut wk0, &mut wk1, &BF1_KEY_SCHEDULE);
            let wkey = [wk0, wk1, wk2, wk3];

            let (mut dv0, mut dv1) = (w0, w1);
            Self::tea_decrypt(&mut dv0, &mut dv1, &wkey);

            if (counter & 0xFFFFFF) == (dv0 >> 8) {
                let crc = Self::calculate_tea_crc(dv0, dv1);
                if crc == (dv1 & 0xFF) as u8 {
                    let mut out = [0u8; 48];
                    Self::unpack_tea_result(&mut out, dv0, dv1);
                    return Some((Self::extract_fields_mode36(&out), counter));
                }
            }
        }
        None
    }

    /// BF2 brute force, range 0xF3000000–0xF4000000 (FUN_080290f8).
    ///
    /// PERFORMANCE: up to ~16.7M TEA iterations. NEVER called from `feed()` (see BF1 note).
    fn brute_force_decrypt_bf2(key1_low: u32, key1_high: u32, key2_low: u32) -> Option<(DecryptResult, u32)> {
        let mut buf = [0u8; 48];
        Self::setup_byte_buffer(&mut buf, key1_low, key1_high, key2_low);
        let (w0, w1) = Self::prepare_tea_data(&buf);

        for counter in BF2_START..BF2_END {
            let wkey = [
                BF2_KEY_SCHEDULE[0] ^ counter,
                BF2_KEY_SCHEDULE[1] ^ counter,
                BF2_KEY_SCHEDULE[2] ^ counter,
                BF2_KEY_SCHEDULE[3] ^ counter,
            ];
            let (mut dv0, mut dv1) = (w0, w1);
            Self::tea_decrypt(&mut dv0, &mut dv1, &wkey);

            if (counter & 0xFFFFFF) == (dv0 >> 8) {
                let crc_buf = [
                    (dv0 >> 24) as u8,
                    (dv0 >> 16) as u8,
                    (dv0 >> 8) as u8,
                    dv0 as u8,
                    (dv1 >> 24) as u8,
                    (dv1 >> 16) as u8,
                ];
                let crc16 = Self::calculate_crc16_bf2(&crc_buf);
                let expected = ((dv1 & 0xFF) | (((dv1 >> 16) & 0xFF) << 8)) as u16;
                if crc16 == expected {
                    let mut out = [0u8; 48];
                    Self::unpack_tea_result(&mut out, dv0, dv1);
                    return Some((Self::extract_fields_mode36(&out), counter));
                }
            }
        }
        None
    }

    /// Full decrypt router (FUN_080291c0, the `__attribute__((unused))` `psa_decrypt_full`):
    /// try XOR (mode23), then BF1, then BF2.
    ///
    /// PERFORMANCE: this can run the bounded brute force (~33M TEA iterations worst case). It is
    /// NOT part of the live `feed()` path — `feed()` uses only `direct_xor_decrypt`. This mirrors
    /// the C, where `psa_decrypt_full` is unused by the decoder and the feed callback calls
    /// `psa_decrypt_fast` (XOR only). Exposed for completeness / offline decrypt; gate any caller
    /// behind the structural frame check so the brute force never runs on arbitrary data.
    #[allow(dead_code)]
    fn decrypt_full(key1_low: u32, key1_high: u32, key2_low: u32) -> Option<DecryptResult> {
        if let Some(r) = Self::direct_xor_decrypt(key1_low, key1_high, key2_low) {
            return Some(r);
        }
        if let Some((r, _seed)) = Self::brute_force_decrypt_bf1(key1_low, key1_high, key2_low) {
            return Some(r);
        }
        if let Some((r, _seed)) = Self::brute_force_decrypt_bf2(key1_low, key1_high, key2_low) {
            return Some(r);
        }
        None
    }

    /// Build the encoded key material for a mode23 frame (psa_build_encrypt_mode23 /
    /// FUN_08029028). Returns `(key1_high, key1_low, validation_field)`.
    fn encode_mode23(serial: u32, button: u8, counter: u16) -> (u32, u32, u16) {
        let mut buf = [0u8; 48];
        buf[2] = (serial >> 16) as u8;
        buf[3] = (serial >> 8) as u8;
        buf[4] = serial as u8;
        buf[5] = (counter >> 8) as u8;
        buf[6] = counter as u8;
        buf[7] = 0; // CRC placeholder
        buf[8] = button & 0xF;
        buf[9] = 0; // key2_low low byte
        Self::second_stage_xor_encrypt(&mut buf);
        Self::calculate_checksum(&mut buf);
        buf[8] = (buf[8] & 0x0F) | (buf[11] & 0xF0);
        buf[13] = buf[9] ^ buf[8];
        // buf[0]/buf[1] preamble bytes (no original key material → derive from data).
        buf[0] = buf[2] ^ buf[6];
        buf[1] = buf[3] ^ buf[7];

        let key1_high =
            ((buf[0] as u32) << 24) | ((buf[1] as u32) << 16) | ((buf[2] as u32) << 8) | buf[3] as u32;
        let key1_low =
            ((buf[4] as u32) << 24) | ((buf[5] as u32) << 16) | ((buf[6] as u32) << 8) | buf[7] as u32;
        let validation = ((buf[8] as u16) << 8) | buf[9] as u16;
        (key1_high, key1_low, validation)
    }

    /// Map KAT's generic button command to a PSA2 button code (Lock=0, Unlock=1, Trunk=2).
    fn map_button(button: u8) -> u8 {
        match button {
            0x01 => 0x0, // Lock
            0x02 => 0x1, // Unlock
            0x04 => 0x2, // Trunk
            0x08 => 0x2, // Panic → Trunk (PSA2 has no panic code)
            b => b & 0x0F,
        }
    }

    /// Finalize a collected 80-bit frame: latch validation/key2, run the cheap mode23 XOR gate,
    /// emit only on successful decryption. NEVER runs the brute force.
    ///
    /// The C feed (`subghz_protocol_decoder_psa2_feed`) also emits on a bare
    /// `(validation_field & 0xF) == 0xA` even when the XOR decrypt fails — but that path yields a
    /// raw, *undecrypted* frame (no serial/button/counter) that the firmware UI keeps so the user
    /// can later run the brute-force button. KAT's pipeline feeds every frequency-compatible
    /// decoder and reports the first that fires, so a 1/16-probability nibble match with no fields
    /// is indistinguishable from noise and steals unrelated 250/500µs frames (e.g. VAG at 434 MHz).
    /// We therefore gate emission strictly on a successful, field-bearing decrypt — the same path
    /// that produces the genuine GROUPE PSA decodes — which is the meaningful half of the C gate.
    fn finalize_frame(&mut self) -> Option<DecodedSignal> {
        // C: validation_field = decode_data_low & 0xFFFF; key2_low = decode_data_low.
        self.validation_field = (self.data_low & 0xFFFF) as u16;
        self.key2_low = self.data_low;
        self.key2_high = self.data_high;

        // C key2_low for decrypt is the 16-bit validation word in the low position.
        let decrypt =
            Self::direct_xor_decrypt(self.key1_low, self.key1_high, self.validation_field as u32);

        // Reset collection regardless (the C feed always rewinds to State0 after a frame attempt).
        self.data_low = 0;
        self.data_high = 0;
        self.bit_count = 0;
        self.state = DecoderState::WaitEdge;

        let (serial, button, counter, _crc, _type) = decrypt?;
        let _ = (DECRYPTED_OK, VALID_NIBBLE); // C markers documented; emission gated on decrypt.
        let data = ((self.key1_high as u64) << 32) | self.key1_low as u64;

        Some(DecodedSignal {
            serial: Some(serial),
            button: Some(button),
            counter: Some(counter as u16),
            crc_valid: true,
            data,
            data_count_bit: MIN_COUNT_BIT,
            encoder_capable: true,
            extra: None,
            protocol_display_name: None,
        })
    }
}

impl ProtocolDecoder for Psa2Decoder {
    fn name(&self) -> &'static str {
        "PSA2"
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
        &[433_920_000]
    }

    fn reset(&mut self) {
        self.state = DecoderState::WaitEdge;
        self.prev_duration = 0;
        self.manchester_state = ManchesterState::Mid1;
        self.pattern_counter = 0;
        self.data_low = 0;
        self.data_high = 0;
        self.bit_count = 0;
        self.key1_low = 0;
        self.key1_high = 0;
        self.validation_field = 0;
        self.key2_low = 0;
        self.key2_high = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.state {
            // State0: detect preamble pattern type.
            DecoderState::WaitEdge => {
                if !level {
                    return None;
                }
                self.init_preamble_state();
                self.prev_duration = duration;
                if Self::near(duration, TE_SHORT, TE_DELTA) {
                    self.state = DecoderState::CountPattern250;
                } else if Self::near(duration, TE_SHORT_HALF, TOL_HALF) {
                    self.state = DecoderState::CountPattern125;
                }
            }

            // State1: count standard-rate (250µs) preamble pulses.
            DecoderState::CountPattern250 => {
                if level {
                    return None;
                }
                if Self::near(duration, TE_SHORT, TE_DELTA) {
                    if Self::near(self.prev_duration, TE_SHORT, TE_DELTA) {
                        self.pattern_counter += 1;
                    }
                    self.prev_duration = duration;
                    return None;
                }
                if Self::near(duration, TE_LONG, TE_DELTA) {
                    if self.pattern_counter > PATTERN_THRESHOLD_1 {
                        self.data_low = 0;
                        self.data_high = 0;
                        self.bit_count = 0;
                        self.manchester_reset();
                        self.state = DecoderState::DecodeManchester250;
                    }
                    self.pattern_counter = 0;
                    self.prev_duration = duration;
                    return None;
                }
                self.state = DecoderState::WaitEdge;
                self.pattern_counter = 0;
            }

            // State2: receive key1 + key2/validation at standard rate.
            DecoderState::DecodeManchester250 => {
                if self.bit_count >= MAX_BITS {
                    self.state = DecoderState::WaitEdge;
                    return None;
                }
                // End-of-packet detection at KEY2_BITS.
                if level && self.bit_count == KEY2_BITS && Self::near(duration, TE_END_1000, 199) {
                    return self.finalize_frame();
                }

                let event: Option<u8>;
                if Self::near(duration, TE_SHORT, TE_DELTA) {
                    event = Some(((level as u8 ^ 1) & 0x7F) << 1);
                } else if Self::near(duration, TE_LONG, TE_DELTA) {
                    event = Some(if level { 4 } else { 6 });
                } else {
                    // Out-of-range low pulse: secondary end marker when 80 bits collected. The C
                    // also gates this on a (stale) nibble==0xA; emission is now decided in
                    // finalize_frame (decrypt-or-reject), so we trigger on the end geometry alone.
                    if !level && Self::near(duration, TE_END_1000, 199) && self.bit_count == KEY2_BITS
                    {
                        return self.finalize_frame();
                    }
                    return None;
                }

                if let Some(ev) = event {
                    if self.bit_count < KEY2_BITS {
                        if let Some(bit) = self.manchester_advance(ev) {
                            self.add_bit(bit);
                        }
                    }
                }
                self.prev_duration = duration;
            }

            // State3: count half-rate (125µs) preamble pulses.
            DecoderState::CountPattern125 => {
                if level {
                    return None;
                }
                if Self::near(duration, TE_SHORT_HALF, TOL_HALF) {
                    if Self::near(self.prev_duration, TE_SHORT_HALF, TOL_HALF) {
                        self.pattern_counter += 1;
                    } else {
                        self.pattern_counter = 0;
                    }
                    self.prev_duration = duration;
                    return None;
                }
                if (TE_LONG_HALF..0x12C).contains(&duration) {
                    if self.pattern_counter > PATTERN_THRESHOLD_2 {
                        self.data_low = 0;
                        self.data_high = 0;
                        self.bit_count = 0;
                        self.manchester_reset();
                        self.state = DecoderState::DecodeManchester125;
                    }
                    self.pattern_counter = 0;
                    self.prev_duration = duration;
                    return None;
                }
                self.state = DecoderState::WaitEdge;
            }

            // State4: receive key1 + key2/validation at half rate.
            DecoderState::DecodeManchester125 => {
                if self.bit_count >= MAX_BITS {
                    self.state = DecoderState::WaitEdge;
                    return None;
                }
                if !level {
                    let event: Option<u8> = if Self::near(duration, TE_SHORT_HALF, TOL_HALF) {
                        Some(((level as u8 ^ 1) & 0x7F) << 1)
                    } else if (TE_LONG_HALF..0x12C).contains(&duration) {
                        Some(if level { 4 } else { 6 })
                    } else {
                        None
                    };
                    if let Some(ev) = event {
                        if let Some(bit) = self.manchester_advance(ev) {
                            self.add_bit(bit);
                        }
                    } else {
                        return None;
                    }
                } else {
                    // Rising edge: end-of-packet at 500µs.
                    if Self::near(duration, TE_END_500, 99) {
                        if self.bit_count != KEY2_BITS {
                            return None;
                        }
                        return self.finalize_frame();
                    }
                }
                self.prev_duration = duration;
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        let serial = decoded.serial?;
        // Increment counter on TX (matches FUN_080299a0 rolling-counter increment).
        let counter = decoded.counter.unwrap_or(0).wrapping_add(1);

        let (key1_high, key1_low, validation) =
            Self::encode_mode23(serial, Self::map_button(button), counter);

        // mode23 timings: te=250µs, sync long=500µs, end=1000µs.
        let te = TE_SHORT;
        let te_long_sync = TE_LONG;
        let end_dur = TE_END_1000;

        let mut signal = Vec::with_capacity(600);

        // Preamble: 80 pairs of (HIGH te)+(LOW te).
        for _ in 0..80 {
            signal.push(LevelDuration::new(true, te));
            signal.push(LevelDuration::new(false, te));
        }

        // Sync: (LOW te) + (HIGH te_long) + (LOW te).
        signal.push(LevelDuration::new(false, te));
        signal.push(LevelDuration::new(true, te_long_sync));
        signal.push(LevelDuration::new(false, te));

        // key1: 64 bits MSB-first. bit=1 → (HIGH,LOW); bit=0 → (LOW,HIGH).
        let k1 = ((key1_high as u64) << 32) | key1_low as u64;
        for bit in (0..64).rev() {
            let b = (k1 >> bit) & 1 == 1;
            signal.push(LevelDuration::new(b, te));
            signal.push(LevelDuration::new(!b, te));
        }

        // validation_field: 16 bits MSB-first.
        for bit in (0..16).rev() {
            let b = (validation >> bit) & 1 == 1;
            signal.push(LevelDuration::new(b, te));
            signal.push(LevelDuration::new(!b, te));
        }

        // End burst: (HIGH end) + (LOW end).
        signal.push(LevelDuration::new(true, end_dur));
        signal.push(LevelDuration::new(false, end_dur));

        Some(signal)
    }
}

impl Default for Psa2Decoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tea_round_trip() {
        // TEA encrypt then decrypt must be the identity for any key.
        let key = BF1_KEY_SCHEDULE;
        let (mut v0, mut v1) = (0x1234_5678u32, 0x9ABC_DEF0u32);
        let (o0, o1) = (v0, v1);
        Psa2Decoder::tea_encrypt(&mut v0, &mut v1, &key);
        Psa2Decoder::tea_decrypt(&mut v0, &mut v1, &key);
        assert_eq!((v0, v1), (o0, o1), "TEA encrypt/decrypt not invertible");
    }

    #[test]
    fn mode23_encode_decode_round_trip() {
        // Encode (serial,button,counter) → key1/validation, then run the mode23 XOR decrypt and
        // confirm the fields (and a passing nibble checksum) come back exactly. Validates TEA-free
        // mode23 path: second_stage_xor + nibble checksum + field packing.
        for &(serial, btn, cnt) in &[
            (0x99EB25u32, 0u8, 0x039Bu16),
            (0x123456, 2, 0x0042),
            (0xABCDEF, 1, 0x1234),
            (0x0000FF, 0, 0x0001),
        ] {
            let (k1h, k1l, vf) = Psa2Decoder::encode_mode23(serial, btn, cnt);
            let decrypt = Psa2Decoder::direct_xor_decrypt(k1l, k1h, vf as u32);
            assert!(
                decrypt.is_some(),
                "mode23 XOR decrypt failed to validate for serial={serial:06X} btn={btn} cnt={cnt:04X}"
            );
            let (ds, db, dc, _crc, ty) = decrypt.unwrap();
            assert_eq!(ds, serial, "serial mismatch");
            assert_eq!(db, btn, "button mismatch");
            assert_eq!(dc, cnt as u32, "counter mismatch");
            assert_eq!(ty, MODE_23, "mode mismatch");
        }
    }

    #[test]
    fn checksum_matches_reference() {
        // Spot-check the nibble checksum against the hand-computed C formula.
        let mut buf = [0u8; 48];
        for (i, b) in buf.iter_mut().enumerate().take(8).skip(2) {
            *b = (i as u8) * 0x11; // 0x22,0x33,0x44,0x55,0x66,0x77
        }
        Psa2Decoder::calculate_checksum(&mut buf);
        // bytes 0x22,0x33,0x44,0x55,0x66,0x77 → nibble sum = (2+2)+(3+3)+(4+4)+(5+5)+(6+6)+(7+7) = 54
        let expected = ((54u32 * 0x10) & 0xFF) as u8; // 54*16 = 864 = 0x360 → 0x60
        assert_eq!(buf[11], expected);
    }

    #[test]
    fn encode_emits_manchester_frame() {
        // The encoder produces a non-trivial 250/500µs Manchester upload.
        let dec = Psa2Decoder::new();
        let decoded = DecodedSignal {
            serial: Some(0x99EB25),
            button: Some(0x01),
            counter: Some(0x039A),
            crc_valid: true,
            data: 0,
            data_count_bit: MIN_COUNT_BIT,
            encoder_capable: true,
            extra: None,
            protocol_display_name: None,
        };
        let sig = dec.encode(&decoded, 0x01).expect("encode should succeed");
        // 80 preamble pairs (160) + 3 sync + 64*2 + 16*2 + 2 end = 160+3+128+32+2 = 325.
        assert_eq!(sig.len(), 325, "unexpected upload length");
        assert!(sig.iter().all(|p| p.duration_us > 0));
    }
}
