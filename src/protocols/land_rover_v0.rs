//! Land Rover V0 protocol decoder/encoder
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/land_rover_v0.c` and
//! `land_rover_v0.h`. **Differential** Manchester (NOT the Flipper transition table): te_short 250,
//! te_long 500, te_delta 100, with a ~750µs sync pulse and a ≥64-pair short preamble. FM.
//!
//! Frame: 81 bits = an 80-bit body (`raw[0..10]`) plus one trailing `extra_bit`. The 64-bit key
//! reported as `DecodedSignal.data` is `raw[0..8]` big-endian; `raw[8..10]` is a 16-bit `tail`.
//! Field layout in the key bytes:
//!   * bytes 0..3  → 24-bit command_signature (Lock = 0xC20363, Unlock = 0xA285E3)
//!   * bytes 3..6  → 24-bit serial
//!   * byte 6 + byte 7 MSB → 9-bit counter = `(b6 << 1) | (b7 >> 7)`
//!   * byte 7 bits 0x78 → 3 reserved bits (must be 0)
//!   * byte 7 bits 0x07 → 3-bit check
//! The check is a proprietary 3-bit polynomial over the counter (`calculate_check`); the tail is
//! 0xFFFF or 0x7FFF depending on a 1-bit parity of the counter (`calculate_tail`). Emission is gated
//! on the reserved bits being zero, the check matching, the tail matching, and `extra_bit` set —
//! so Land Rover V0 is strongly validated and will not false-match. Encoder supported.
//!
//! Decode steps: Reset → PreambleLow/PreambleHigh (count short pairs, ≥64) → SyncLow → Data.
//! The Data step ports the C `process_transition`/`add_decoded_bit` differential machine directly:
//! it tracks `previous_bit`, skips the initial boundary short-high pad (`boundary_pad_skipped`),
//! and completes short half-bits via `pending_short`. Bit 0 (= 1) is seeded on entry to Data.
//!
//! Note on the C encoder (faithfully ported): `build_upload` forces frame bit 1 = 0 regardless of
//! the key, so combined with the seeded bit 0 = 1 the top two frame bits are always `10`. The
//! Unlock signature 0xA285E3 satisfies this and round-trips cleanly; the Lock signature 0xC20363
//! (whose bit 1 is 1) is emitted as 0x820363 by the reference encoder. We reproduce that behaviour.

use super::{ProtocolDecoder, ProtocolTiming, DecodedSignal};
use crate::radio::demodulator::LevelDuration;
use crate::duration_diff;

const TE_SHORT: u32 = 250;
const TE_LONG: u32 = 500;
const TE_DELTA: u32 = 100;
const SYNC_US: u32 = 750;
const SYNC_DELTA_US: u32 = 120;
const MIN_PREAMBLE_PAIRS: u16 = 64;
const COUNT_BIT: usize = 81;
const GAP_US: u32 = 50_000;

/// TX preamble pair count (matches LAND_ROVER_V0_PREAMBLE_PAIRS).
const TX_PREAMBLE_PAIRS: usize = 319;

// Button signatures (LAND_ROVER_V0_SIG_*).
const SIG_UNLOCK: u32 = 0x00A2_85E3;
const SIG_LOCK: u32 = 0x00C2_0363;

// Land Rover button codes (LAND_ROVER_V0_BTN_*).
const LR_BTN_UNKNOWN: u8 = 0x00;
const LR_BTN_LOCK: u8 = 0x02;
const LR_BTN_UNLOCK: u8 = 0x04;

#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    PreambleLow,
    PreambleHigh,
    SyncLow,
    Data,
}

pub struct LandRoverV0Decoder {
    step: DecoderStep,
    preamble_count: u16,
    raw: [u8; 10],
    bit_count: u8,
    extra_bit: bool,
    previous_bit: bool,
    boundary_pad_skipped: bool,
    pending_short: bool,
}

impl LandRoverV0Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            preamble_count: 0,
            raw: [0; 10],
            bit_count: 0,
            extra_bit: false,
            previous_bit: true,
            boundary_pad_skipped: false,
            pending_short: false,
        }
    }

    fn is_short(d: u32) -> bool {
        duration_diff!(d, TE_SHORT) < TE_DELTA
    }
    fn is_long(d: u32) -> bool {
        duration_diff!(d, TE_LONG) < TE_DELTA
    }
    fn is_sync(d: u32) -> bool {
        duration_diff!(d, SYNC_US) < SYNC_DELTA_US
    }

    /// Reset the per-frame differential-Manchester state (matches the SyncLow init in the C feed).
    fn begin_frame(&mut self) {
        self.raw = [0; 10];
        self.bit_count = 0;
        self.extra_bit = false;
        self.previous_bit = true;
        self.boundary_pad_skipped = false;
        self.pending_short = false;
    }

    /// Append one decoded bit (matches `land_rover_v0_add_decoded_bit`).
    /// Bits 0..80 pack MSB-first into `raw`; bit 80 is the trailing `extra_bit`.
    fn add_decoded_bit(&mut self, bit: bool) -> bool {
        if self.bit_count < 80 {
            let byte_index = (self.bit_count / 8) as usize;
            let bit_index = 7 - (self.bit_count % 8);
            if bit {
                self.raw[byte_index] |= 1u8 << bit_index;
            }
        } else if self.bit_count == 80 {
            self.extra_bit = bit;
        } else {
            return false;
        }
        self.bit_count += 1;
        true
    }

    /// Differential-Manchester transition handler (direct port of `land_rover_v0_process_transition`).
    fn process_transition(&mut self, level: bool, duration: u32) -> bool {
        if !self.boundary_pad_skipped {
            if level && Self::is_short(duration) {
                self.boundary_pad_skipped = true;
                return true;
            }
            self.boundary_pad_skipped = true;
        }

        if self.pending_short {
            if !self.previous_bit && !level && Self::is_short(duration) {
                self.pending_short = false;
                return self.add_decoded_bit(false);
            } else if self.previous_bit && level && Self::is_short(duration) {
                self.pending_short = false;
                return self.add_decoded_bit(true);
            }
            return false;
        }

        if !self.previous_bit {
            if level && Self::is_long(duration) {
                self.previous_bit = true;
                return self.add_decoded_bit(true);
            } else if level && Self::is_short(duration) {
                self.pending_short = true;
                return true;
            }
            return false;
        }

        if !level && Self::is_long(duration) {
            self.previous_bit = false;
            return self.add_decoded_bit(false);
        } else if !level && Self::is_short(duration) {
            self.pending_short = true;
            return true;
        }

        false
    }

    /// 3-bit check polynomial over the 9-bit counter (matches `land_rover_v0_calculate_check`).
    fn calculate_check(count: u32) -> u8 {
        let c0 = ((count >> 1) ^ (count >> 2) ^ (count >> 3) ^ (count >> 4) ^ (count >> 6)) & 1;
        let c1 = ((count >> 0)
            ^ (count >> 2)
            ^ (count >> 3)
            ^ (count >> 4)
            ^ (count >> 5)
            ^ (count >> 6)
            ^ 1)
            & 1;
        let c2 = ((count >> 1) ^ (count >> 3) ^ (count >> 4) ^ (count >> 5) ^ (count >> 6)) & 1;
        (c0 | (c1 << 1) | (c2 << 2)) as u8
    }

    /// MSB selector for the 16-bit tail (matches `land_rover_v0_calculate_tail_msb`).
    fn calculate_tail_msb(count: u32) -> bool {
        (((count >> 0) ^ (count >> 2) ^ (count >> 4) ^ (count >> 5)) & 1) != 0
    }

    /// 16-bit tail value (matches `land_rover_v0_calculate_tail`).
    fn calculate_tail(count: u32) -> u16 {
        if Self::calculate_tail_msb(count) {
            0xFFFF
        } else {
            0x7FFF
        }
    }

    /// Map a 24-bit command signature to a Land Rover button (matches
    /// `land_rover_v0_button_from_signature`).
    fn button_from_signature(signature: u32) -> u8 {
        match signature {
            SIG_UNLOCK => LR_BTN_UNLOCK,
            SIG_LOCK => LR_BTN_LOCK,
            _ => LR_BTN_UNKNOWN,
        }
    }

    /// Counter packed in key bytes 6 + 7-MSB (matches the C count extraction).
    fn count_from_bytes(b: &[u8; 8]) -> u32 {
        ((b[6] as u32) << 1) | ((b[7] >> 7) & 1) as u32
    }

    /// Validate a frame (matches `land_rover_v0_validate_frame`).
    /// Returns (check_ok, tail_ok).
    fn validate_frame(key: u64, tail: u16, extra_bit: bool) -> (bool, bool) {
        let b = key.to_be_bytes();
        let count = Self::count_from_bytes(&b);
        let expected_check = Self::calculate_check(count);
        let expected_tail = Self::calculate_tail(count);

        let check_ok = (b[7] & 0x78) == 0 && (b[7] & 0x07) == expected_check;
        let tail_ok = tail == expected_tail && extra_bit;
        (check_ok, tail_ok)
    }

    /// Finish a frame: validate, then build the decoded signal (matches
    /// `land_rover_v0_finish_frame` + `parse_key_fields`). Returns None when invalid (gated).
    fn finish_frame(&self) -> Option<DecodedSignal> {
        let key = u64::from_be_bytes([
            self.raw[0],
            self.raw[1],
            self.raw[2],
            self.raw[3],
            self.raw[4],
            self.raw[5],
            self.raw[6],
            self.raw[7],
        ]);
        let tail = ((self.raw[8] as u16) << 8) | self.raw[9] as u16;

        let (check_ok, tail_ok) = Self::validate_frame(key, tail, self.extra_bit);
        if !(check_ok && tail_ok) {
            return None;
        }

        let b = key.to_be_bytes();
        let signature = ((b[0] as u32) << 16) | ((b[1] as u32) << 8) | b[2] as u32;
        let serial = ((b[3] as u32) << 16) | ((b[4] as u32) << 8) | b[5] as u32;
        let count = Self::count_from_bytes(&b);
        let button = Self::button_from_signature(signature);

        Some(DecodedSignal {
            serial: Some(serial),
            button: Some(button),
            counter: Some(count as u16),
            crc_valid: true, // check + tail + reserved-bits validated above
            data: key,
            data_count_bit: COUNT_BIT,
            encoder_capable: true,
            // Stash the 16-bit tail so the encoder can rebuild the full frame without recomputation.
            extra: Some(tail as u64),
            protocol_display_name: None,
        })
    }

    /// Map KAT's generic button command to a Land Rover signature (Lock/Unlock).
    /// KAT: Lock=0x01, Unlock=0x02, Trunk=0x04, Panic=0x08. Land Rover V0 only defines Lock/Unlock,
    /// so Trunk/Panic have no signature and fall back to the decoded frame's signature in `encode`.
    fn signature_from_button(button: u8) -> u32 {
        match button {
            0x01 => SIG_LOCK,   // KAT Lock
            0x02 => SIG_UNLOCK, // KAT Unlock
            _ => 0,
        }
    }

    /// Build the 64-bit key from fields (matches `land_rover_v0_build_key`).
    fn build_key(signature: u32, serial: u32, count: u32) -> u64 {
        let mut b = [0u8; 8];
        b[0] = (signature >> 16) as u8;
        b[1] = (signature >> 8) as u8;
        b[2] = signature as u8;
        b[3] = (serial >> 16) as u8;
        b[4] = (serial >> 8) as u8;
        b[5] = serial as u8;
        b[6] = (count >> 1) as u8;
        let counter_lsb = (count & 1) != 0;
        let check = Self::calculate_check(count);
        b[7] = (if counter_lsb { 0x80 } else { 0x00 }) | check;
        u64::from_be_bytes(b)
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

    /// Emit one differential-Manchester bit (matches `land_rover_v0_encoder_add_bit`).
    /// Returns the new `previous_bit`.
    fn enc_add_bit(signal: &mut Vec<LevelDuration>, previous_bit: bool, bit: bool) -> bool {
        match (previous_bit, bit) {
            (false, false) => {
                Self::enc_add_level(signal, true, TE_SHORT);
                Self::enc_add_level(signal, false, TE_SHORT);
            }
            (false, true) => {
                Self::enc_add_level(signal, true, TE_LONG);
            }
            (true, false) => {
                Self::enc_add_level(signal, false, TE_LONG);
            }
            (true, true) => {
                Self::enc_add_level(signal, false, TE_SHORT);
                Self::enc_add_level(signal, true, TE_SHORT);
            }
        }
        bit
    }
}

impl ProtocolDecoder for LandRoverV0Decoder {
    fn name(&self) -> &'static str {
        "Land Rover V0"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: COUNT_BIT,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.preamble_count = 0;
        self.raw = [0; 10];
        self.bit_count = 0;
        self.extra_bit = false;
        self.previous_bit = true;
        self.boundary_pad_skipped = false;
        self.pending_short = false;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::Reset => {
                if level && Self::is_short(duration) {
                    self.preamble_count = 0;
                    self.step = DecoderStep::PreambleLow;
                }
            }

            DecoderStep::PreambleLow => {
                if !level && Self::is_short(duration) {
                    self.preamble_count += 1;
                    self.step = DecoderStep::PreambleHigh;
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::PreambleHigh => {
                if level && Self::is_short(duration) {
                    self.step = DecoderStep::PreambleLow;
                } else if level
                    && Self::is_sync(duration)
                    && self.preamble_count >= MIN_PREAMBLE_PAIRS
                {
                    self.step = DecoderStep::SyncLow;
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::SyncLow => {
                if !level && Self::is_sync(duration) {
                    self.begin_frame();
                    self.add_decoded_bit(true); // seed bit 0 = 1
                    self.step = DecoderStep::Data;
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::Data => {
                if !self.process_transition(level, duration) {
                    self.step = DecoderStep::Reset;
                    return None;
                }

                if self.bit_count as usize == COUNT_BIT {
                    let result = self.finish_frame();
                    self.step = DecoderStep::Reset;
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
        // Derive fields from the decoded signal, preferring the requested button's signature.
        let serial = decoded.serial? & 0x00FF_FFFF;
        let count = (decoded.counter.unwrap_or(0) as u32) & 0x1FF;

        // Pick the command signature: requested button first, else the decoded frame's signature.
        let mut signature = Self::signature_from_button(button);
        if signature == 0 {
            let b = decoded.data.to_be_bytes();
            signature = ((b[0] as u32) << 16) | ((b[1] as u32) << 8) | b[2] as u32;
            // If still not a known Land Rover signature, refuse (matches C: command_signature==0 → error).
            if Self::button_from_signature(signature) == LR_BTN_UNKNOWN {
                return None;
            }
        }

        let key = Self::build_key(signature, serial, count);
        let key_bytes = key.to_be_bytes();
        let tail = Self::calculate_tail(count);

        // Capacity: preamble pairs (2 levels) + sync (3) + ~81 bits (≤2 levels each) + gap.
        let mut signal = Vec::with_capacity(TX_PREAMBLE_PAIRS * 2 + 3 + COUNT_BIT * 2 + 1);

        // Preamble: alternating short high/low pairs.
        for _ in 0..TX_PREAMBLE_PAIRS {
            Self::enc_add_level(&mut signal, true, TE_SHORT);
            Self::enc_add_level(&mut signal, false, TE_SHORT);
        }

        // Sync: high 750, low 750, then a boundary short-high pad (skipped by the decoder).
        Self::enc_add_level(&mut signal, true, SYNC_US);
        Self::enc_add_level(&mut signal, false, SYNC_US);
        Self::enc_add_level(&mut signal, true, TE_SHORT);

        // Differential-Manchester body. previous_bit starts true (bit 0 = 1 is implied by the sync
        // trailing short). Bit index 1 is forced to 0 (matches the C build_upload), then bits 2..63
        // come from the key bytes.
        let mut previous_bit = true;
        previous_bit = Self::enc_add_bit(&mut signal, previous_bit, false);
        for bit_index in 2..64u8 {
            let byte_index = (bit_index / 8) as usize;
            let bit_in_byte = 7 - (bit_index % 8);
            let bit = (key_bytes[byte_index] >> bit_in_byte) & 1 != 0;
            previous_bit = Self::enc_add_bit(&mut signal, previous_bit, bit);
        }

        // 16-bit tail, MSB first.
        for bit_index in 0..16u8 {
            let bit = (tail >> (15 - bit_index)) & 1 != 0;
            previous_bit = Self::enc_add_bit(&mut signal, previous_bit, bit);
        }

        // Trailing extra bit = 1.
        let _ = Self::enc_add_bit(&mut signal, previous_bit, true);

        // Inter-frame gap.
        Self::enc_add_level(&mut signal, false, GAP_US);

        Some(signal)
    }
}

impl Default for LandRoverV0Decoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Drive a full encoded upload through the decoder and return the first decode.
    fn decode_upload(upload: &[LevelDuration]) -> Option<DecodedSignal> {
        let mut dec = LandRoverV0Decoder::new();
        for ld in upload {
            if let Some(sig) = dec.feed(ld.level, ld.duration_us) {
                return Some(sig);
            }
        }
        None
    }

    /// The 3-bit check polynomial matches the C reference for a couple of hand traces.
    #[test]
    fn check_polynomial_matches_reference() {
        // count = 0: c0=0, c1=(0^1)&1=1, c2=0 → 0b010 = 2
        assert_eq!(LandRoverV0Decoder::calculate_check(0), 0b010);
        // Spot-check internal consistency: build_key embeds calculate_check in byte 7.
        for count in 0..0x200u32 {
            let key = LandRoverV0Decoder::build_key(SIG_UNLOCK, 0x123456, count);
            let b = key.to_be_bytes();
            assert_eq!(
                b[7] & 0x07,
                LandRoverV0Decoder::calculate_check(count),
                "check embedded in key byte 7 must equal calculate_check(count) for count={count}"
            );
            assert_eq!(b[7] & 0x78, 0, "reserved bits must be zero for count={count}");
        }
    }

    /// The tail is 0xFFFF or 0x7FFF per the counter parity, matching the reference.
    #[test]
    fn tail_matches_reference() {
        for count in 0..0x200u32 {
            let tail = LandRoverV0Decoder::calculate_tail(count);
            assert!(tail == 0xFFFF || tail == 0x7FFF);
            let msb_set = (tail >> 15) & 1 == 1;
            assert_eq!(msb_set, LandRoverV0Decoder::calculate_tail_msb(count));
        }
    }

    /// Primary correctness check: encode an Unlock frame and decode it back. The Unlock signature
    /// 0xA285E3 satisfies the encoder's forced-bit-1 = 0 invariant, so it round-trips cleanly:
    /// serial, button, counter, key and the 3-bit check all survive.
    #[test]
    fn unlock_round_trip_preserves_fields() {
        let serial = 0x00AB_CDEF & 0x00FF_FFFF;
        let counter = 0x123u16; // 9-bit counter
        let key = LandRoverV0Decoder::build_key(SIG_UNLOCK, serial, counter as u32);

        let decoded_in = DecodedSignal {
            serial: Some(serial),
            button: Some(LR_BTN_UNLOCK),
            counter: Some(counter),
            crc_valid: true,
            data: key,
            data_count_bit: COUNT_BIT,
            encoder_capable: true,
            extra: Some(LandRoverV0Decoder::calculate_tail(counter as u32) as u64),
            protocol_display_name: None,
        };

        let dec = LandRoverV0Decoder::new();
        // KAT Unlock command = 0x02.
        let upload = dec.encode(&decoded_in, 0x02).expect("encode Unlock");
        let out = decode_upload(&upload).expect("decode the encoded Unlock frame");

        assert!(out.crc_valid, "decoded frame must pass check+tail gating");
        assert_eq!(out.data, key, "64-bit key must survive the round trip");
        assert_eq!(out.serial, Some(serial), "serial must survive");
        assert_eq!(out.counter, Some(counter), "counter must survive");
        assert_eq!(out.button, Some(LR_BTN_UNLOCK), "Unlock button must survive");
        assert_eq!(out.data_count_bit, COUNT_BIT);
        // The stashed tail must match the counter-derived tail.
        assert_eq!(out.extra, Some(LandRoverV0Decoder::calculate_tail(counter as u32) as u64));
        // The embedded 3-bit check must match the recomputed value.
        let b = out.data.to_be_bytes();
        assert_eq!(b[7] & 0x07, LandRoverV0Decoder::calculate_check(counter as u32));
        assert_eq!(b[7] & 0x78, 0, "reserved bits zero");
    }

    /// Round-trip across many serial/counter values for Unlock.
    #[test]
    fn unlock_round_trip_many() {
        let mut state: u32 = 0x1357_9BDF;
        let mut next = || {
            // xorshift for deterministic pseudo-random coverage
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            state
        };

        for _ in 0..256 {
            let serial = next() & 0x00FF_FFFF;
            let counter = (next() & 0x1FF) as u16;
            let key = LandRoverV0Decoder::build_key(SIG_UNLOCK, serial, counter as u32);
            let decoded_in = DecodedSignal {
                serial: Some(serial),
                button: Some(LR_BTN_UNLOCK),
                counter: Some(counter),
                crc_valid: true,
                data: key,
                data_count_bit: COUNT_BIT,
                encoder_capable: true,
                extra: Some(LandRoverV0Decoder::calculate_tail(counter as u32) as u64),
                protocol_display_name: None,
            };
            let dec = LandRoverV0Decoder::new();
            let upload = dec.encode(&decoded_in, 0x02).expect("encode");
            let out = decode_upload(&upload)
                .unwrap_or_else(|| panic!("decode failed for serial={serial:06X} counter={counter:03X}"));
            assert_eq!(out.data, key);
            assert_eq!(out.serial, Some(serial));
            assert_eq!(out.counter, Some(counter));
            assert_eq!(out.button, Some(LR_BTN_UNLOCK));
            assert!(out.crc_valid);
        }
    }

    /// Faithful-port note: the reference encoder forces frame bit 1 = 0. The Lock signature
    /// 0xC20363 has bit 1 = 1, so the emitted frame's signature becomes 0x820363, which maps to
    /// LR_BTN_UNKNOWN. We assert this exact reference behaviour rather than a "fixed" version.
    #[test]
    fn lock_signature_forced_bit_matches_reference_quirk() {
        let serial = 0x0012_3456;
        let counter = 100u16;
        let key = LandRoverV0Decoder::build_key(SIG_LOCK, serial, counter as u32);
        let decoded_in = DecodedSignal {
            serial: Some(serial),
            button: Some(LR_BTN_LOCK),
            counter: Some(counter),
            crc_valid: true,
            data: key,
            data_count_bit: COUNT_BIT,
            encoder_capable: true,
            extra: Some(LandRoverV0Decoder::calculate_tail(counter as u32) as u64),
            protocol_display_name: None,
        };
        let dec = LandRoverV0Decoder::new();
        // KAT Lock command = 0x01.
        let upload = dec.encode(&decoded_in, 0x01).expect("encode Lock");
        let out = decode_upload(&upload).expect("frame still decodes (check/tail valid)");
        // Top byte 0xC2 -> 0x82 because bit 1 is forced to 0 by the encoder.
        let b = out.data.to_be_bytes();
        assert_eq!(b[0], 0x82, "encoder forces frame bit 1 = 0, turning 0xC2 into 0x82");
        assert_eq!(out.button, Some(LR_BTN_UNKNOWN), "0x820363 is not a known signature");
        // Serial / counter / check are still intact.
        assert_eq!(out.serial, Some(serial));
        assert_eq!(out.counter, Some(counter));
        assert!(out.crc_valid);
    }

    /// A frame with a deliberately wrong check must NOT decode (gating prevents false matches).
    #[test]
    fn bad_check_is_rejected() {
        let serial = 0x00AB_CDEF;
        let counter = 0x055u16;
        let mut key = LandRoverV0Decoder::build_key(SIG_UNLOCK, serial, counter as u32);
        // Corrupt the 3-bit check in byte 7 (XOR a bit so it no longer matches).
        key ^= 0x01;
        let decoded_in = DecodedSignal {
            serial: Some(serial),
            button: Some(LR_BTN_UNLOCK),
            counter: Some(counter),
            crc_valid: false,
            data: key,
            data_count_bit: COUNT_BIT,
            encoder_capable: true,
            extra: None,
            protocol_display_name: None,
        };
        // Hand-build an upload that transmits this corrupted key verbatim (no re-derivation):
        // reuse the encoder's level helpers but bypass build_key by injecting the raw key.
        let key_bytes = key.to_be_bytes();
        let tail = LandRoverV0Decoder::calculate_tail(counter as u32);
        let mut signal: Vec<LevelDuration> = Vec::new();
        for _ in 0..TX_PREAMBLE_PAIRS {
            LandRoverV0Decoder::enc_add_level(&mut signal, true, TE_SHORT);
            LandRoverV0Decoder::enc_add_level(&mut signal, false, TE_SHORT);
        }
        LandRoverV0Decoder::enc_add_level(&mut signal, true, SYNC_US);
        LandRoverV0Decoder::enc_add_level(&mut signal, false, SYNC_US);
        LandRoverV0Decoder::enc_add_level(&mut signal, true, TE_SHORT);
        let mut prev = true;
        prev = LandRoverV0Decoder::enc_add_bit(&mut signal, prev, false);
        for bit_index in 2..64u8 {
            let bi = (bit_index / 8) as usize;
            let bib = 7 - (bit_index % 8);
            let bit = (key_bytes[bi] >> bib) & 1 != 0;
            prev = LandRoverV0Decoder::enc_add_bit(&mut signal, prev, bit);
        }
        for bit_index in 0..16u8 {
            let bit = (tail >> (15 - bit_index)) & 1 != 0;
            prev = LandRoverV0Decoder::enc_add_bit(&mut signal, prev, bit);
        }
        let _ = LandRoverV0Decoder::enc_add_bit(&mut signal, prev, true);
        LandRoverV0Decoder::enc_add_level(&mut signal, false, GAP_US);

        // The decoded count comes from the (corrupted) byte 7, so the embedded check now mismatches
        // calculate_check(count) → rejected. Note: corrupting bit 0 of the check does not change
        // the counter (counter uses byte 7 MSB only), so calculate_check(count) stays the same.
        let _ = decoded_in;
        assert!(
            decode_upload(&signal).is_none(),
            "frame with a corrupted check must be rejected by the gate"
        );
    }
}
