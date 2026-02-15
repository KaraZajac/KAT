//! Ford V0 protocol decoder/encoder
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/ford_v0.c` and `ford_v0.h`.
//! Ford uses Flipper's lib/toolbox/manchester_decoder.h (ManchesterState, ManchesterEvent,
//! manchester_advance). We use a separate FordV0ManchesterState and the same event mapping:
//! level ? ManchesterEventShortLow : ManchesterEventShortHigh (short/long).
//!
//! Protocol: 250/500µs Manchester, 80 bits (64 key1 + 16 key2), CRC matrix, BS magic,
//! 6 bursts, 4 preamble pairs, 3500µs gap. te_delta 100µs, gap tolerance 250µs.

use super::{ProtocolDecoder, ProtocolTiming, DecodedSignal};
use crate::radio::demodulator::LevelDuration;
use crate::duration_diff;

const TE_SHORT: u32 = 250;
const TE_LONG: u32 = 500;
/// Timing tolerance (µs). Ref ford_v0.c uses 100; real-world captures (e.g. IMPORTS/FORD/3_unlock_ford.sub)
/// can have preamble "long" pulses ~387–397µs (103–113µs from 500), so we use 120 to decode them.
const TE_DELTA: u32 = 120;
const MIN_COUNT_BIT: usize = 64;
const TOTAL_BURSTS: u8 = 6;
const TX_REPEAT: usize = 1;
const PREAMBLE_PAIRS: usize = 4;
const GAP_US: u32 = 3500;
const GAP_TOLERANCE: u32 = 250; // ref: DURATION_DIFF(duration, gap_threshold) < 250

// CRC matrix for Ford V0 — GF(2) matrix multiplication
// Copied directly from protopirate's ford_v0.c
const CRC_MATRIX: [u8; 64] = [
    0xDA, 0xB5, 0x55, 0x6A, 0xAA, 0xAA, 0xAA, 0xD5,
    0xB6, 0x6C, 0xCC, 0xD9, 0x99, 0x99, 0x99, 0xB3,
    0x71, 0xE3, 0xC3, 0xC7, 0x87, 0x87, 0x87, 0x8F,
    0x0F, 0xE0, 0x3F, 0xC0, 0x7F, 0x80, 0x7F, 0x80,
    0x00, 0x1F, 0xFF, 0xC0, 0x00, 0x7F, 0xFF, 0x80,
    0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0x80,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F,
    0x23, 0x12, 0x94, 0x84, 0x35, 0xF4, 0x55, 0x84,
];

/// Ford V0 Manchester state machine. Transition table matches Flipper's
/// manchester_decoder.h (ProtoPirate ford_v0.c uses it). Separate from Fiat and common.
#[derive(Debug, Clone, Copy, PartialEq)]
enum FordV0ManchesterState {
    Mid0 = 0,
    Mid1 = 1,
    Start0 = 2,
    Start1 = 3,
}

/// Decoder step states (matches protopirate's FordV0DecoderStep)
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Preamble,
    PreambleCheck,
    Gap,
    Data,
}

/// Ford V0 protocol decoder (matches SubGhzProtocolDecoderFordV0)
pub struct FordV0Decoder {
    step: DecoderStep,
    manchester_state: FordV0ManchesterState,
    /// Two 64-bit shift registers as in C (ford_v0_add_bit); combined = (data_high<<32)|data_low for key1
    data_low: u64,
    data_high: u64,
    bit_count: u8,
    header_count: u16,
    te_last: u32,
    key1: u64,
    key2: u16,
    serial: u32,
    button: u8,
    counter: u32,
    bs_magic: u8,
}

impl FordV0Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            manchester_state: FordV0ManchesterState::Mid1,
            data_low: 0,
            data_high: 0,
            bit_count: 0,
            header_count: 0,
            te_last: 0,
            key1: 0,
            key2: 0,
            serial: 0,
            button: 0,
            counter: 0,
            bs_magic: 0,
        }
    }

    /// Add a bit (matches ford_v0_add_bit in C exactly)
    fn add_bit(&mut self, bit: bool) {
        let low = self.data_low as u32;
        self.data_low = (self.data_low << 1) | (if bit { 1 } else { 0 });
        self.data_high = (self.data_high << 1) | ((low >> 31) & 1) as u64;
        self.bit_count += 1;
    }

    /// Process data at 64 and 80 bits (matches ford_v0_process_data)
    fn process_data(&mut self) -> bool {
        if self.bit_count == 64 {
            let combined = (self.data_high << 32) | self.data_low;
            self.key1 = !combined;
            self.data_low = 0;
            self.data_high = 0;
            return false;
        }

        if self.bit_count == 80 {
            let key2_raw = (self.data_low & 0xFFFF) as u16;
            self.key2 = !key2_raw;

            // Decode serial, button, counter, bs_magic from key1+key2
            let (serial, button, count, bs_magic) =
                Self::decode_ford_v0(self.key1, self.key2);
            self.serial = serial;
            self.button = button;
            self.counter = count;
            self.bs_magic = bs_magic;
            return true;
        }

        false
    }

    /// Manchester state machine (Flipper manchester_advance; ProtoPirate ford_v0.c feed).
    /// Event: 0=ShortLow, 1=ShortHigh, 2=LongLow, 3=LongHigh. Level mapping: level ? 0/2 : 1/3.
    /// Returns Some(bit) when a data bit is produced.
    fn manchester_advance(&mut self, event: u8) -> Option<bool> {
        let (new_state, emit) = match (self.manchester_state, event) {
            // State Mid0: currently in middle of a 0-bit (signal is LOW)
            (FordV0ManchesterState::Mid0, 0) => (FordV0ManchesterState::Mid0, false),   // ShortLow: error, stay
            (FordV0ManchesterState::Mid0, 1) => (FordV0ManchesterState::Start1, true),  // ShortHigh: emit
            (FordV0ManchesterState::Mid0, 2) => (FordV0ManchesterState::Mid0, false),   // LongLow: error
            (FordV0ManchesterState::Mid0, 3) => (FordV0ManchesterState::Mid1, true),    // LongHigh: emit

            // State Mid1: currently in middle of a 1-bit (signal is HIGH)
            (FordV0ManchesterState::Mid1, 0) => (FordV0ManchesterState::Start0, true),  // ShortLow: emit
            (FordV0ManchesterState::Mid1, 1) => (FordV0ManchesterState::Mid1, false),   // ShortHigh: error, stay
            (FordV0ManchesterState::Mid1, 2) => (FordV0ManchesterState::Mid0, true),    // LongLow: emit
            (FordV0ManchesterState::Mid1, 3) => (FordV0ManchesterState::Mid1, false),   // LongHigh: error

            // State Start0: at start of a 0-bit (signal is HIGH, waiting for H→L)
            (FordV0ManchesterState::Start0, 0) => (FordV0ManchesterState::Mid0, false), // ShortLow: complete 0
            (FordV0ManchesterState::Start0, 1) => (FordV0ManchesterState::Mid0, false), // error → reset
            (FordV0ManchesterState::Start0, 2) => (FordV0ManchesterState::Mid0, false), // error
            (FordV0ManchesterState::Start0, 3) => (FordV0ManchesterState::Mid1, false), // error

            // State Start1: at start of a 1-bit (signal is LOW, waiting for L→H)
            (FordV0ManchesterState::Start1, 0) => (FordV0ManchesterState::Mid0, false), // error
            (FordV0ManchesterState::Start1, 1) => (FordV0ManchesterState::Mid1, false), // ShortHigh: complete 1
            (FordV0ManchesterState::Start1, 2) => (FordV0ManchesterState::Mid0, false), // error
            (FordV0ManchesterState::Start1, 3) => (FordV0ManchesterState::Mid1, false), // error

            _ => (FordV0ManchesterState::Mid1, false),
        };

        self.manchester_state = new_state;

        if emit {
            // Bit value: 1 for High events (1,3), 0 for Low events (0,2)
            Some((event & 1) == 1)
        } else {
            None
        }
    }

    // =========================================================================
    // CRC functions
    // =========================================================================

    /// Population count for a byte
    fn popcount8(mut x: u8) -> u8 {
        let mut count = 0u8;
        while x != 0 {
            count += x & 1;
            x >>= 1;
        }
        count
    }

    /// Calculate CRC using GF(2) matrix multiplication.
    /// buf must have at least 9 bytes; CRC is computed over buf[1..=8].
    fn calculate_crc(buf: &[u8]) -> u8 {
        let mut crc = 0u8;
        for row in 0..8 {
            let mut xor_sum = 0u8;
            for col in 0..8 {
                xor_sum ^= CRC_MATRIX[row * 8 + col] & buf[col + 1];
            }
            let parity = Self::popcount8(xor_sum) & 1;
            if parity != 0 {
                crc |= 1 << row;
            }
        }
        crc
    }

    /// Calculate CRC for transmission (key1 bytes + BS byte, XOR 0x80)
    fn calculate_crc_for_tx(key1: u64, bs: u8) -> u8 {
        let mut buf = [0u8; 16];
        for i in 0..8 {
            buf[i] = (key1 >> (56 - i * 8)) as u8;
        }
        buf[8] = bs;
        Self::calculate_crc(&buf) ^ 0x80
    }

    /// Verify CRC of received key1 + key2
    fn verify_crc(key1: u64, key2: u16) -> bool {
        let mut buf = [0u8; 16];
        for i in 0..8 {
            buf[i] = (key1 >> (56 - i * 8)) as u8;
        }
        buf[8] = (key2 >> 8) as u8; // BS byte
        let calculated_crc = Self::calculate_crc(&buf);
        let received_crc = (key2 as u8) ^ 0x80;
        calculated_crc == received_crc
    }

    // =========================================================================
    // BS calculation
    // =========================================================================

    /// Calculate BS = (count_low_byte + bs_magic + (button << 4)) with overflow handling
    fn calculate_bs(count: u32, button: u8, bs_magic: u8) -> u8 {
        let result: u16 = (count as u16 & 0xFF)
            .wrapping_add(bs_magic as u16)
            .wrapping_add((button as u16) << 4);
        (result as u8).wrapping_sub(if result & 0xFF00 != 0 { 0x80 } else { 0 })
    }

    // =========================================================================
    // Decode function — extract serial/button/counter/bs_magic from key1+key2
    // Matches protopirate's decode_ford_v0() exactly
    // =========================================================================

    fn decode_ford_v0(key1: u64, key2: u16) -> (u32, u8, u32, u8) {
        let mut buf = [0u8; 13];

        // Extract key1 bytes (big-endian)
        for i in 0..8 {
            buf[i] = (key1 >> (56 - i * 8)) as u8;
        }
        // Extract key2 bytes
        buf[8] = (key2 >> 8) as u8;
        buf[9] = key2 as u8;

        // BS parity calculation
        let bs = buf[8];
        let mut tmp = bs;
        let mut parity = 0u8;
        let parity_any: u8 = if tmp != 0 { 1 } else { 0 };
        while tmp != 0 {
            parity ^= tmp & 1;
            tmp >>= 1;
        }
        buf[11] = if parity_any != 0 { parity } else { 0 };

        // XOR decryption based on parity bit
        let (xor_byte, limit) = if buf[11] != 0 {
            (buf[7], 7usize)
        } else {
            (buf[6], 6usize)
        };

        for idx in 1..limit {
            buf[idx] ^= xor_byte;
        }

        if buf[11] == 0 {
            buf[7] ^= xor_byte;
        }

        // Bit-interleave swap of buf[6] and buf[7]
        let orig_b7 = buf[7];
        buf[7] = (orig_b7 & 0xAA) | (buf[6] & 0x55);
        let mixed = (buf[6] & 0xAA) | (orig_b7 & 0x55);
        buf[12] = mixed;
        buf[6] = mixed;

        // Extract serial (stored little-endian in buf[1..5], convert to big-endian)
        let serial_le = (buf[1] as u32)
            | ((buf[2] as u32) << 8)
            | ((buf[3] as u32) << 16)
            | ((buf[4] as u32) << 24);
        let serial = ((serial_le & 0xFF) << 24)
            | (((serial_le >> 8) & 0xFF) << 16)
            | (((serial_le >> 16) & 0xFF) << 8)
            | ((serial_le >> 24) & 0xFF);

        // Extract button (high nibble of buf[5])
        let button = (buf[5] >> 4) & 0x0F;

        // Extract counter (20-bit)
        let count = ((buf[5] as u32 & 0x0F) << 16) | ((buf[6] as u32) << 8) | (buf[7] as u32);

        // Calculate BS magic number for this fob
        let bs_magic = bs
            .wrapping_add(if bs & 0x80 != 0 { 0x80 } else { 0 })
            .wrapping_sub(button << 4)
            .wrapping_sub(count as u8);

        (serial, button, count, bs_magic)
    }

    // =========================================================================
    // Encode function — rebuild key1 from serial/button/counter/bs
    // Matches protopirate's encode_ford_v0() exactly
    // =========================================================================

    fn encode_ford_v0(
        header_byte: u8,
        serial: u32,
        button: u8,
        count: u32,
        bs: u8,
    ) -> u64 {
        let mut buf = [0u8; 8];

        buf[0] = header_byte;

        // Serial in big-endian
        buf[1] = (serial >> 24) as u8;
        buf[2] = (serial >> 16) as u8;
        buf[3] = (serial >> 8) as u8;
        buf[4] = serial as u8;

        // Button + counter high nibble
        buf[5] = ((button & 0x0F) << 4) | ((count >> 16) as u8 & 0x0F);

        let count_mid = (count >> 8) as u8;
        let count_low = count as u8;

        // Bit-interleave: split even/odd bits between the two counter bytes
        let post_xor_6 = (count_mid & 0xAA) | (count_low & 0x55);
        let post_xor_7 = (count_low & 0xAA) | (count_mid & 0x55);

        // Calculate BS parity
        let mut parity = 0u8;
        let mut tmp = bs;
        while tmp != 0 {
            parity ^= tmp & 1;
            tmp >>= 1;
        }
        let parity_bit = if bs != 0 { parity != 0 } else { false };

        // XOR encryption based on parity (inverse of decode)
        if parity_bit {
            let xor_byte = post_xor_7;
            buf[1] ^= xor_byte;
            buf[2] ^= xor_byte;
            buf[3] ^= xor_byte;
            buf[4] ^= xor_byte;
            buf[5] ^= xor_byte;
            buf[6] = post_xor_6 ^ xor_byte;
            buf[7] = post_xor_7;
        } else {
            let xor_byte = post_xor_6;
            buf[1] ^= xor_byte;
            buf[2] ^= xor_byte;
            buf[3] ^= xor_byte;
            buf[4] ^= xor_byte;
            buf[5] ^= xor_byte;
            buf[6] = post_xor_6;
            buf[7] = post_xor_7 ^ xor_byte;
        }

        // Pack into u64
        let mut key1 = 0u64;
        for b in &buf {
            key1 = (key1 << 8) | (*b as u64);
        }
        key1
    }

    // =========================================================================
    // Encoder signal builder — differential Manchester with ADD_LEVEL merging
    // Matches protopirate's subghz_protocol_encoder_ford_v0_get_upload()
    // =========================================================================

    fn build_upload(key1: u64, key2: u16) -> Vec<LevelDuration> {
        let mut signal = Vec::with_capacity(1024);

        // Transmitted data is bit-inverted
        let tx_key1 = !key1;
        let tx_key2 = !key2;

        for burst in 0..TOTAL_BURSTS {
            // Preamble start: short HIGH + long LOW
            Self::add_level(&mut signal, true, TE_SHORT);
            Self::add_level(&mut signal, false, TE_LONG);

            // Preamble pairs: long HIGH + long LOW
            for _ in 0..PREAMBLE_PAIRS {
                Self::add_level(&mut signal, true, TE_LONG);
                Self::add_level(&mut signal, false, TE_LONG);
            }

            // End preamble: short HIGH + gap LOW
            Self::add_level(&mut signal, true, TE_SHORT);
            Self::add_level(&mut signal, false, GAP_US);

            // First data bit (bit 62 of tx_key1; bit 63 is implicit from gap)
            let first_bit = ((tx_key1 >> 62) & 1) == 1;
            if first_bit {
                Self::add_level(&mut signal, true, TE_LONG);
            } else {
                Self::add_level(&mut signal, true, TE_SHORT);
                Self::add_level(&mut signal, false, TE_LONG);
            }

            let mut prev_bit = first_bit;

            // Encode remaining key1 bits (61 down to 0) — differential Manchester
            for bit_pos in (0..62).rev() {
                let curr_bit = ((tx_key1 >> bit_pos) & 1) == 1;
                Self::encode_diff_bit(&mut signal, prev_bit, curr_bit);
                prev_bit = curr_bit;
            }

            // Encode key2 bits (15 down to 0)
            for bit_pos in (0..16).rev() {
                let curr_bit = ((tx_key2 >> bit_pos) & 1) == 1;
                Self::encode_diff_bit(&mut signal, prev_bit, curr_bit);
                prev_bit = curr_bit;
            }

            // Inter-burst gap (except for last burst)
            if burst < TOTAL_BURSTS - 1 {
                Self::add_level(&mut signal, false, TE_LONG * 100);
            }
        }

        signal
    }

    /// Encode one differential Manchester bit transition
    fn encode_diff_bit(signal: &mut Vec<LevelDuration>, prev: bool, curr: bool) {
        match (prev, curr) {
            (false, false) => {
                // 0→0: mid-bit transition only (HIGH short, LOW short)
                Self::add_level(signal, true, TE_SHORT);
                Self::add_level(signal, false, TE_SHORT);
            }
            (false, true) => {
                // 0→1: transition at start (extends to LONG HIGH)
                Self::add_level(signal, true, TE_LONG);
            }
            (true, false) => {
                // 1→0: transition at start (extends to LONG LOW)
                Self::add_level(signal, false, TE_LONG);
            }
            (true, true) => {
                // 1→1: mid-bit transition only (LOW short, HIGH short)
                Self::add_level(signal, false, TE_SHORT);
                Self::add_level(signal, true, TE_SHORT);
            }
        }
    }

    /// ADD_LEVEL equivalent: merge adjacent same-level pulses for efficiency.
    /// This matches protopirate's ADD_LEVEL macro behavior.
    fn add_level(signal: &mut Vec<LevelDuration>, level: bool, duration: u32) {
        if let Some(last) = signal.last_mut() {
            if last.level == level {
                *last = LevelDuration::new(level, last.duration_us + duration);
                return;
            }
        }
        signal.push(LevelDuration::new(level, duration));
    }

    /// Get button name for Ford V0
    #[allow(dead_code)]
    fn button_name(btn: u8) -> &'static str {
        match btn {
            0x01 => "Lock",
            0x02 => "Unlock",
            0x04 => "Boot",
            _ => "Unknown",
        }
    }
}

impl ProtocolDecoder for FordV0Decoder {
    fn name(&self) -> &'static str {
        "Ford V0"
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
        &[315_000_000, 433_920_000] // 315 MHz (US) and 433.92 MHz (EU)
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.te_last = 0;
        self.manchester_state = FordV0ManchesterState::Mid1;
        self.data_low = 0;
        self.data_high = 0;
        self.bit_count = 0;
        self.header_count = 0;
        self.key1 = 0;
        self.key2 = 0;
        self.serial = 0;
        self.button = 0;
        self.counter = 0;
        self.bs_magic = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            // C: level && DURATION_DIFF(duration, te_short) < te_delta → Preamble.
            // Also allow level && long so we can re-sync when capture starts mid-preamble.
            DecoderStep::Reset => {
                if level && duration_diff!(duration, TE_SHORT) < TE_DELTA {
                    self.data_low = 0;
                    self.data_high = 0;
                    self.step = DecoderStep::Preamble;
                    self.te_last = duration;
                    self.header_count = 0;
                    self.bit_count = 0;
                    self.manchester_state = FordV0ManchesterState::Mid1;
                } else if level && duration_diff!(duration, TE_LONG) < TE_DELTA {
                    // Alternative: long HIGH (e.g. preamble) → Preamble so next LOW can sync
                    self.data_low = 0;
                    self.data_high = 0;
                    self.step = DecoderStep::Preamble;
                    self.te_last = duration;
                    self.header_count = 0;
                    self.bit_count = 0;
                    self.manchester_state = FordV0ManchesterState::Mid1;
                }
            }

            // C: !level, long → PreambleCheck; else → Reset
            DecoderStep::Preamble => {
                if !level {
                    if duration_diff!(duration, TE_LONG) < TE_DELTA {
                        self.te_last = duration;
                        self.step = DecoderStep::PreambleCheck;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                }
            }

            // C: level, long → header_count++, Preamble; level, short → Gap; else → Reset
            DecoderStep::PreambleCheck => {
                if level {
                    if duration_diff!(duration, TE_LONG) < TE_DELTA {
                        self.header_count += 1;
                        self.te_last = duration;
                        self.step = DecoderStep::Preamble;
                    } else if duration_diff!(duration, TE_SHORT) < TE_DELTA {
                        self.step = DecoderStep::Gap;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                }
            }

            // C: !level && DURATION_DIFF(duration, 3500) < 250 → Data; !level && duration > 3750 → Reset
            DecoderStep::Gap => {
                if !level && duration_diff!(duration, GAP_US) < GAP_TOLERANCE {
                    self.data_low = 1;
                    self.data_high = 0;
                    self.bit_count = 1;
                    self.step = DecoderStep::Data;
                } else if !level && duration > GAP_US + GAP_TOLERANCE {
                    self.step = DecoderStep::Reset;
                }
            }

            // C: DURATION_DIFF(duration, te_short) < te_delta → short event; te_long → long event.
            // Real-world .sub captures (e.g. IMPORTS/FORD/3_unlock_ford.sub) have inter-burst gaps
            // (~51 ms) between the 6 repeats; skip those so we keep collecting bits across bursts.
            DecoderStep::Data => {
                let event = if duration_diff!(duration, TE_SHORT) < TE_DELTA {
                    if level { 0 } else { 1 }
                } else if duration_diff!(duration, TE_LONG) < TE_DELTA {
                    if level { 2 } else { 3 }
                } else if duration >= 5_000 {
                    // Inter-burst or long gap: skip without resetting so next burst continues the bit stream
                    return None;
                } else {
                    self.step = DecoderStep::Reset;
                    return None;
                };

                if let Some(data_bit) = self.manchester_advance(event) {
                    self.add_bit(data_bit);

                    if self.process_data() {
                        let crc_ok = Self::verify_crc(self.key1, self.key2);
                        let result = DecodedSignal {
                            serial: Some(self.serial),
                            button: Some(self.button),
                            counter: Some(self.counter as u16),
                            crc_valid: crc_ok,
                            data: self.key1,
                            data_count_bit: MIN_COUNT_BIT,
                            encoder_capable: true,
                            extra: None,
                        };

                        self.data_low = 0;
                        self.data_high = 0;
                        self.bit_count = 0;
                        self.step = DecoderStep::Reset;
                        return Some(result);
                    }
                }

                self.te_last = duration;
            }
        }

        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        let serial = decoded.serial?;

        // Use the same counter as the decoded signal (no increment).
        // Reference ford_v0.c encoder uses instance->count from the loaded file as-is;
        // replay and different-button TX both use that count so the vehicle accepts it.
        let count = (if self.counter != 0 {
            self.counter
        } else {
            decoded.counter.unwrap_or(0) as u32
        }) & 0xFFFFF; // 20-bit

        // Use stored bs_magic (or default to 0x6F for backward compatibility)
        let bs_magic = if self.bs_magic != 0 { self.bs_magic } else { 0x6F };

        // Calculate BS from count + button + bs_magic (matches C ford_v0_calculate_bs)
        let bs = Self::calculate_bs(count, button, bs_magic);

        // Extract header byte from the original key1 (first byte)
        let header_byte = (decoded.data >> 56) as u8;

        // Encode key1 from fields (same count, new button)
        let new_key1 = Self::encode_ford_v0(header_byte, serial, button, count, bs);

        // Calculate CRC for key2
        let crc = Self::calculate_crc_for_tx(new_key1, bs);
        let new_key2 = ((bs as u16) << 8) | (crc as u16);

        // Build one 6-burst block and repeat TX_REPEAT times (matches reference encoder.repeat = 10)
        let single = Self::build_upload(new_key1, new_key2);
        let mut signal = Vec::with_capacity(single.len() * TX_REPEAT);
        for _ in 0..TX_REPEAT {
            signal.extend_from_slice(&single);
        }
        Some(signal)
    }
}
