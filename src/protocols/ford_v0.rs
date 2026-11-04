//! Ford V0 protocol decoder/encoder
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/ford_v0.c` and `ford_v0.h`.
//!
//! Protocol: 250/500µs Manchester, 80 bits (64 key1 + 16 key2), CRC matrix, 6 bursts, 4 preamble
//! pairs, 3500µs gap. te_delta 100µs in the C (KAT uses 120, see `TE_DELTA`), gap tolerance 250µs.
//!
//! **Manchester**: the shared Flipper `manchester_advance` port in `common.rs`. Event mapping is
//! ProtoPirate's `pp_manchester_event()` (ford_v0.c:707): `level ? ShortLow : ShortHigh` (same for
//! long). The state is reset to Mid1 (`ManchesterEventReset`) when a preamble starts
//! (ford_v0.c:662) and on decoder reset (ford_v0.c:633); it is not touched at the 3500µs gap, so
//! data decoding starts in Mid1 with key1's first wire bit seeded as 1 (ford_v0.c:697). The first
//! HIGH after the gap is that seeded bit's first half; Mid1 drops it and locks on the next LOW.
//!
//! **Frame**: on air `~key1` (64 bits, MSB first; bit 63 is not sent, the decoder seeds it) then
//! `~key2` (16 bits). key2 = checksum byte (BS) << 8 | CRC byte. key1 bytes (`decode_ford_v0`):
//! buf[0] header (0x55 on every capture in IMPORTS/FORD); buf[1..5] serial; buf[5] = button field
//! (b5[7:3]) | counter[18:16] (b5[2:0]); buf[6..8] counter[15:0] — buf[1..8] are XOR-scrambled and
//! buf[6]/buf[7] bit-interleaved, keyed by the BS parity.
//!
//! **Buttons**: the C reads a 4-bit field `(buf[5] >> 3) & 0x0F` (Panic=1, Lock=2, Unlock=4,
//! Boot=8). Real captures show b5[7:3] is a one-hot field with a fifth bit, b5[7] = Remote start
//! (IMPORTS/FORD/16_ford_escape_rs2_*.sub; the BS checksum includes it). KAT decodes all five bits
//! and reports them as KAT's generic button bits (`BUTTON_MAP`): Lock 0x01, Unlock 0x02,
//! Trunk/Boot 0x04, Panic 0x08, Remote start 0x10 — bitwise, so the encoder can map back.
//!
//! **CRC / gating**: CRC = GF(2) matrix over key1 bytes 1..7 + BS; key2's low byte carries CRC ^
//! 0x80. Like the C (`ford_v0_verify_crc`, ford_v0.c:216) only the low 7 bits are compared — bit 7
//! of the CRC byte is not the matrix output on real fobs (it differs on ~40% of genuine frames).
//! The C emits every 80-bit frame and only displays CRC OK/BAD; KAT emits only CRC-valid frames
//! (every genuine frame in IMPORTS/FORD passes), so `crc_valid` is true on everything emitted.
//!
//! **DecodedSignal**: `data` = key1 (de-inverted, the C's `generic.data`), `data_count_bit` = 64
//! (as the C, which stores key2 as separate Checksum/CRC fields), `extra` = key2 (so the encoder can
//! recover the full 19-bit counter and the captured button field), `counter` = counter[15:0].
//!
//! **Encoder** (subghz_protocol_encoder_ford_v0_get_upload): counter + 1 (ProtoPirate's emulate
//! scene writes Cnt = counter + 1 before the encoder reads it; Flipper-ARF and doctor-firmware add
//! `rolling_counter_mult` in the encoder), requested button (generic code → button field as the
//! emulate scene maps Up/OK/Down/Left; other codes keep the captured field), BS =
//! `ford_v0_calculate_checksum`, CRC = matrix ^ 0x80, then 6 bursts of preamble + gap + the
//! transition-coded ~key1 bits 62..0 and ~key2. Matches the C upload, including its first-bit
//! special case (only the wire-bit-62 = 0 branch, i.e. key1 bit 62 = 1, is reachable with the 0x55
//! header; the other branch emits a long HIGH the decoder cannot lock onto), except that a final
//! wire bit 0 is finished with its HIGH half as on real captures (see `build_upload`).
//!
//! **Deviations from C**: `TE_DELTA` 120 (captures with ~390µs longs); Reset also accepts a long
//! HIGH (re-sync mid-preamble); CRC-gated emission; 5-bit button field (Remote start); the BS
//! checksum adds the 5-bit field (identical to the C for its four buttons); the encoder's trailing
//! HIGH half-bit after a final 0 (without it the C's bursts lose their last bit to the gap
//! whenever the CRC's last wire bit is 0).

use super::common::{ManchesterEvent, ManchesterState};
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

/// The C checks only the low 7 bits of the CRC byte (`ford_v0_verify_crc`).
const CRC_CHECK_MASK: u8 = 0x7F;

/// On-air button field b5[7:3] (one-hot) ↔ KAT generic button bit. The C names field values
/// 1/2/4/8 Panic/Lock/Unlock/Boot; 0x10 (b5[7]) is Remote start (seen on real captures only).
const BUTTON_MAP: [(u8, u8); 5] = [
    (0x01, 0x08), // Panic
    (0x02, 0x01), // Lock
    (0x04, 0x02), // Unlock
    (0x08, 0x04), // Boot / Trunk
    (0x10, 0x10), // Remote start (no KAT generic code)
];

/// Decoder step states (matches protopirate's FordV0DecoderStep)
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Preamble,
    PreambleCheck,
    Gap,
    Data,
}

/// Fields recovered from key1 + key2 by `decode_ford_v0`.
#[derive(Debug, Clone, Copy, PartialEq)]
struct FordV0Fields {
    serial: u32,
    /// On-air button field b5[7:3] (5-bit one-hot).
    button_field: u8,
    /// 19-bit counter: b5[2:0] << 16 | de-interleaved buf[6..8].
    count: u32,
}

/// Ford V0 protocol decoder (matches SubGhzProtocolDecoderFordV0)
pub struct FordV0Decoder {
    step: DecoderStep,
    manchester_state: ManchesterState,
    /// Two 64-bit shift registers as in C (ford_v0_add_bit); combined = (data_high<<32)|data_low for key1
    data_low: u64,
    data_high: u64,
    bit_count: u8,
    header_count: u16,
    te_last: u32,
    key1: u64,
    key2: u16,
}

impl FordV0Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            manchester_state: ManchesterState::Mid1,
            data_low: 0,
            data_high: 0,
            bit_count: 0,
            header_count: 0,
            te_last: 0,
            key1: 0,
            key2: 0,
        }
    }

    fn is_short(duration: u32) -> bool {
        duration_diff!(duration, TE_SHORT) < TE_DELTA
    }

    fn is_long(duration: u32) -> bool {
        duration_diff!(duration, TE_LONG) < TE_DELTA
    }

    /// Add a bit (matches ford_v0_add_bit in C exactly)
    fn add_bit(&mut self, bit: bool) {
        let low = self.data_low as u32;
        self.data_low = (self.data_low << 1) | (if bit { 1 } else { 0 });
        self.data_high = (self.data_high << 1) | ((low >> 31) & 1) as u64;
        self.bit_count += 1;
    }

    /// Process data at 64 and 80 bits (matches ford_v0_process_data). Returns true once key2 is
    /// complete (80 bits).
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
            return true;
        }

        false
    }

    /// Build the decoded signal for a complete frame, or None if the CRC check fails.
    fn frame_signal(key1: u64, key2: u16) -> Option<DecodedSignal> {
        if !Self::verify_crc(key1, key2) {
            return None;
        }
        let fields = Self::decode_ford_v0(key1, key2);
        Some(DecodedSignal {
            serial: Some(fields.serial),
            button: Some(Self::field_to_button(fields.button_field)),
            counter: Some(fields.count as u16),
            crc_valid: true,
            data: key1,
            data_count_bit: MIN_COUNT_BIT,
            encoder_capable: true,
            extra: Some(key2 as u64),
            protocol_display_name: None,
        })
    }

    // =========================================================================
    // Buttons
    // =========================================================================

    /// On-air button field (b5[7:3]) → KAT generic button bits.
    fn field_to_button(field: u8) -> u8 {
        BUTTON_MAP
            .iter()
            .filter(|(f, _)| field & f != 0)
            .fold(0, |acc, (_, generic)| acc | generic)
    }

    /// KAT generic button bits → on-air button field (inverse of `field_to_button`).
    fn button_to_field(button: u8) -> u8 {
        BUTTON_MAP
            .iter()
            .filter(|(_, generic)| button & generic != 0)
            .fold(0, |acc, (f, _)| acc | f)
    }

    // =========================================================================
    // CRC functions
    // =========================================================================

    /// Calculate CRC using GF(2) matrix multiplication.
    /// buf must have at least 9 bytes; CRC is computed over buf[1..=8].
    fn calculate_crc(buf: &[u8]) -> u8 {
        let mut crc = 0u8;
        for row in 0..8 {
            let mut xor_sum = 0u8;
            for col in 0..8 {
                xor_sum ^= CRC_MATRIX[row * 8 + col] & buf[col + 1];
            }
            if xor_sum.count_ones() & 1 != 0 {
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

    /// Verify CRC of received key1 + key2 (ford_v0_verify_crc: low 7 bits only).
    fn verify_crc(key1: u64, key2: u16) -> bool {
        let mut buf = [0u8; 16];
        for i in 0..8 {
            buf[i] = (key1 >> (56 - i * 8)) as u8;
        }
        buf[8] = (key2 >> 8) as u8; // BS byte
        let calculated_crc = Self::calculate_crc(&buf);
        let received_crc = (key2 as u8) ^ 0x80;
        (calculated_crc & CRC_CHECK_MASK) == (received_crc & CRC_CHECK_MASK)
    }

    // =========================================================================
    // Checksum (BS byte)
    // =========================================================================

    /// ford_v0_calculate_checksum: byte sum of count + serial + (button << 3), truncated to u8.
    /// `button_field` is the 5-bit field (the C's 4-bit `button` plus Remote start), so the sum is
    /// the C's for its four buttons and matches the real Remote-start frames too.
    fn calculate_checksum(serial: u32, count: u32, button_field: u8) -> u8 {
        let sum: u32 = ((count >> 24) & 0xFF)
            .wrapping_add((count >> 16) & 0xFF)
            .wrapping_add((count >> 8) & 0xFF)
            .wrapping_add(count & 0xFF)
            .wrapping_add((serial >> 24) & 0xFF)
            .wrapping_add((serial >> 16) & 0xFF)
            .wrapping_add((serial >> 8) & 0xFF)
            .wrapping_add(serial & 0xFF)
            .wrapping_add(((button_field & 0x1F) as u32) << 3);
        (sum & 0xFF) as u8
    }

    // =========================================================================
    // Decode function — extract serial/button/counter from key1+key2
    // Matches protopirate's decode_ford_v0()
    // =========================================================================

    fn decode_ford_v0(key1: u64, key2: u16) -> FordV0Fields {
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

        // Button field b5[7:3] (C: (buf[5] >> 3) & 0x0F, which drops the Remote-start bit)
        let button_field = buf[5] >> 3;

        // 19-bit counter (C: ((buf[5] & 0x07) << 16) | (buf[6] << 8) | buf[7])
        let count = ((buf[5] as u32 & 0x07) << 16) | ((buf[6] as u32) << 8) | (buf[7] as u32);

        FordV0Fields { serial, button_field, count }
    }

    // =========================================================================
    // Encode function — rebuild key1 from serial/button/counter/checksum
    // Matches protopirate's encode_ford_v0()
    // =========================================================================

    fn encode_ford_v0(
        header_byte: u8,
        serial: u32,
        button_field: u8,
        count: u32,
        checksum: u8,
    ) -> u64 {
        let mut buf = [0u8; 8];

        buf[0] = header_byte;

        // Serial in big-endian
        buf[1] = (serial >> 24) as u8;
        buf[2] = (serial >> 16) as u8;
        buf[3] = (serial >> 8) as u8;
        buf[4] = serial as u8;

        // Button field + counter[18:16] (C: ((button & 0x0F) << 3) | ((count >> 16) & 0x0F))
        buf[5] = ((button_field & 0x1F) << 3) | ((count >> 16) as u8 & 0x07);

        let count_mid = (count >> 8) as u8;
        let count_low = count as u8;

        // Bit-interleave: split even/odd bits between the two counter bytes
        let post_xor_6 = (count_mid & 0xAA) | (count_low & 0x55);
        let post_xor_7 = (count_low & 0xAA) | (count_mid & 0x55);

        // Calculate BS parity
        let parity_bit = checksum != 0 && checksum.count_ones() & 1 != 0;

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

        u64::from_be_bytes(buf)
    }

    // =========================================================================
    // Encoder signal builder — transition-coded Manchester with ADD_LEVEL merging
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

            // Encode remaining key1 bits (61 down to 0)
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

            // KAT addition: finish a final 0 bit (LOW then HIGH on air) with its HIGH half, as real
            // fobs do (IMPORTS/FORD/2008_Focus_Lock.sub ends "... -504 +257 -51810"). The C upload
            // stops after the LOW half, which then merges into the gap and the last bit is lost.
            if !prev_bit {
                Self::add_level(&mut signal, true, TE_SHORT);
            }

            // Inter-burst gap (except for last burst)
            if burst < TOTAL_BURSTS - 1 {
                Self::add_level(&mut signal, false, TE_LONG * 100);
            }
        }

        signal
    }

    /// Emit the pulse(s) between two consecutive bits: the second half of `prev` then the first
    /// half of `curr` (1 = HIGH,LOW; 0 = LOW,HIGH on air), as the C's (prev, curr) cases do.
    fn encode_diff_bit(signal: &mut Vec<LevelDuration>, prev: bool, curr: bool) {
        match (prev, curr) {
            (false, false) => {
                Self::add_level(signal, true, TE_SHORT);
                Self::add_level(signal, false, TE_SHORT);
            }
            (false, true) => {
                Self::add_level(signal, true, TE_LONG);
            }
            (true, false) => {
                Self::add_level(signal, false, TE_LONG);
            }
            (true, true) => {
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
        self.manchester_state.advance(ManchesterEvent::Reset); // ford_v0.c:633 → Mid1
        self.data_low = 0;
        self.data_high = 0;
        self.bit_count = 0;
        self.header_count = 0;
        self.key1 = 0;
        self.key2 = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            // C: level && short → Preamble, Manchester reset (ford_v0.c:655-666).
            // KAT also accepts a long HIGH so a capture that starts mid-preamble can still sync.
            DecoderStep::Reset => {
                if level && (Self::is_short(duration) || Self::is_long(duration)) {
                    self.data_low = 0;
                    self.data_high = 0;
                    self.step = DecoderStep::Preamble;
                    self.te_last = duration;
                    self.header_count = 0;
                    self.bit_count = 0;
                    self.manchester_state.advance(ManchesterEvent::Reset);
                }
            }

            // C: !level, long → PreambleCheck; else → Reset
            DecoderStep::Preamble => {
                if !level {
                    if Self::is_long(duration) {
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
                    if Self::is_long(duration) {
                        self.header_count += 1;
                        self.te_last = duration;
                        self.step = DecoderStep::Preamble;
                    } else if Self::is_short(duration) {
                        self.step = DecoderStep::Gap;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                }
            }

            // C: !level && DURATION_DIFF(duration, 3500) < 250 → Data with the first bit seeded as 1;
            // !level && duration > 3750 → Reset
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

            // C (ford_v0.c:707-712): pp_manchester_event() = level ? ShortLow : ShortHigh (same for
            // long); any other duration (incl. the inter-burst gap) → Reset.
            DecoderStep::Data => {
                let event = if Self::is_short(duration) {
                    ManchesterEvent::from_half_bit(!level, false)
                } else if Self::is_long(duration) {
                    ManchesterEvent::from_half_bit(!level, true)
                } else {
                    self.step = DecoderStep::Reset;
                    return None;
                };

                if let Some(data_bit) = self.manchester_state.advance(event) {
                    self.add_bit(data_bit);

                    if self.process_data() {
                        let result = Self::frame_signal(self.key1, self.key2);
                        self.data_low = 0;
                        self.data_high = 0;
                        self.bit_count = 0;
                        self.step = DecoderStep::Reset;
                        return result;
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

        // The captured counter + 1: the emulate scene writes Cnt = counter + 1 (unmasked, as the C;
        // ARF/doctor add the rolling multiplier). With key2 in `extra`, recover the full 19-bit
        // counter and the captured button field from the frame itself; otherwise fall back to the
        // 16-bit counter / reported button.
        let (count, captured_field) = match decoded.extra {
            Some(key2) => {
                let fields = Self::decode_ford_v0(decoded.data, key2 as u16);
                (fields.count, fields.button_field)
            }
            None => (
                decoded.counter.unwrap_or(0) as u32,
                Self::button_to_field(decoded.button.unwrap_or(0)),
            ),
        };

        // Kept within the 19 counter bits (the C would spill a carry out of 0x7FFFF into the
        // button field's low bit and the checksum).
        let count = count.wrapping_add(1) & 0x7_FFFF;

        // KAT generic Lock/Unlock/Trunk/Panic → button field; anything else keeps the captured one.
        let button_field = match button {
            0x01 | 0x02 | 0x04 | 0x08 => Self::button_to_field(button),
            _ => captured_field,
        };

        // Checksum (BS) from serial + count + button (matches ford_v0_calculate_checksum in C)
        let checksum = Self::calculate_checksum(serial, count, button_field);

        // Header byte from the original key1 (first byte)
        let header_byte = (decoded.data >> 56) as u8;

        let new_key1 = Self::encode_ford_v0(header_byte, serial, button_field, count, checksum);

        // key2 = checksum << 8 | (CRC ^ 0x80)
        let crc = Self::calculate_crc_for_tx(new_key1, checksum);
        let new_key2 = ((checksum as u16) << 8) | (crc as u16);

        // One 6-burst block (the C's encoder.repeat replays it; KAT sends it TX_REPEAT times).
        let single = Self::build_upload(new_key1, new_key2);
        let mut signal = Vec::with_capacity(single.len() * TX_REPEAT);
        for _ in 0..TX_REPEAT {
            signal.extend_from_slice(&single);
        }
        Some(signal)
    }
}

impl Default for FordV0Decoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::export::flipper::import_sub_raw;
    use crate::protocols::ProtocolRegistry;
    use std::collections::BTreeSet;
    use std::path::Path;

    /// A genuine frame from IMPORTS/FORD/2008_Focus_Lock.sub (Lock, counter 0x08780).
    const FOCUS_LOCK_KEY1: u64 = 0x55C6_833B_D595_0785;
    const FOCUS_LOCK_KEY2: u16 = 0x6EF9;

    fn feed_all(dec: &mut FordV0Decoder, signal: &[LevelDuration]) -> Vec<DecodedSignal> {
        signal.iter().filter_map(|p| dec.feed(p.level, p.duration_us)).collect()
    }

    fn real_decodes(rel: &str) -> Option<Vec<DecodedSignal>> {
        let path = Path::new("IMPORTS/FORD").join(rel);
        if !path.exists() {
            eprintln!("Skip: {:?} not found (run from crate root)", path);
            return None;
        }
        let (freq, raw) = import_sub_raw(&path).unwrap();
        let pairs: Vec<LevelDuration> =
            raw.iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect();
        let results = ProtocolRegistry::new().process_signal_stream(&pairs, freq);
        let names: Vec<&str> = results.iter().map(|(n, _, _)| n.as_str()).collect();
        assert!(names.iter().all(|n| *n == "Ford V0"), "{rel}: unexpected decodes {names:?}");
        Some(results.into_iter().map(|(_, d, _)| d).collect())
    }

    #[test]
    fn decode_fields_of_real_frame() {
        assert!(FordV0Decoder::verify_crc(FOCUS_LOCK_KEY1, FOCUS_LOCK_KEY2));
        let f = FordV0Decoder::decode_ford_v0(FOCUS_LOCK_KEY1, FOCUS_LOCK_KEY2);
        assert_eq!(f, FordV0Fields { serial: 0x4306_BE50, button_field: 0x02, count: 0x08780 });
        assert_eq!(FordV0Decoder::field_to_button(f.button_field), 0x01, "C button 2 = Lock");
    }

    /// The CRC byte's bit 7 is not covered (C masks 0x7F): a genuine frame whose bit 7 differs
    /// from the matrix output must still verify, while any low-7-bit error must not.
    #[test]
    fn crc_checks_low_seven_bits_only() {
        // 2008_Focus_Lock.sub frame with counter 0x0877F: bit 7 of the CRC byte differs.
        let (key1, key2) = (0x556C_2991_7F3F_F82Fu64, 0x6DD2u16);
        let mut buf = [0u8; 16];
        buf[..8].copy_from_slice(&key1.to_be_bytes());
        buf[8] = (key2 >> 8) as u8;
        assert_ne!(FordV0Decoder::calculate_crc(&buf), (key2 as u8) ^ 0x80, "bit 7 differs");
        assert!(FordV0Decoder::verify_crc(key1, key2));
        for bit in 0..7 {
            assert!(!FordV0Decoder::verify_crc(key1, key2 ^ (1 << bit)), "CRC bit {bit}");
        }
        assert!(!FordV0Decoder::verify_crc(key1 ^ (1 << 20), key2), "key1 corruption");
    }

    #[test]
    fn button_field_mapping_round_trips() {
        for field in 0u8..0x20 {
            let generic = FordV0Decoder::field_to_button(field);
            assert_eq!(FordV0Decoder::button_to_field(generic), field);
        }
        assert_eq!(FordV0Decoder::field_to_button(0x01), 0x08, "Panic");
        assert_eq!(FordV0Decoder::field_to_button(0x04), 0x02, "Unlock");
        assert_eq!(FordV0Decoder::field_to_button(0x08), 0x04, "Boot/Trunk");
        assert_eq!(FordV0Decoder::field_to_button(0x10), 0x10, "Remote start");
    }

    /// Encoder → decoder through the full 6-burst waveform: every burst decodes to the same frame,
    /// the requested button and the counter advanced by one press.
    #[test]
    fn encode_decode_roundtrip() {
        let decoded = FordV0Decoder::frame_signal(FOCUS_LOCK_KEY1, FOCUS_LOCK_KEY2).unwrap();
        for (button, field) in [(0x01u8, 0x02u8), (0x02, 0x04), (0x04, 0x08), (0x08, 0x01)] {
            let upload = FordV0Decoder::new().encode(&decoded, button).expect("encode");
            let frames = feed_all(&mut FordV0Decoder::new(), &upload);
            assert_eq!(frames.len(), TOTAL_BURSTS as usize, "button {button:#04x}");
            for f in &frames {
                assert!(f.crc_valid);
                assert_eq!(f.serial, Some(0x4306_BE50));
                assert_eq!(f.button, Some(button));
                assert_eq!(f.counter, Some(0x8781));
                let key2 = f.extra.unwrap() as u16;
                let fields = FordV0Decoder::decode_ford_v0(f.data, key2);
                assert_eq!((fields.button_field, fields.count), (field, 0x08781));
            }
        }
    }

    /// Encoding a genuine press (counter n) with its own button reproduces the fob's NEXT press
    /// (counter n + 1, captured in the same file) exactly: key1, the checksum byte and the CRC's 7
    /// checked bits.
    #[test]
    fn reencode_reproduces_next_genuine_press() {
        let pairs: [((u64, u16), (u64, u16), u8); 7] = [
            ((0x556C_2991_7F3F_F82F, 0x6DD2), (FOCUS_LOCK_KEY1, FOCUS_LOCK_KEY2), 0x01), // Focus Lock
            ((0x556E_2B93_7D0D_FF2D, 0x7656), (0x5590_D56D_83F3_D3FE, 0x778D), 0x02), // Focus Unlock
            ((0x55C1_843C_D2C2_8205, 0xA055), (0x55C4_8139_D7C7_0487, 0xA130), 0x04), // Focus Trunk
            ((0x55CE_9E19_9FA6_86AB, 0xEBCF), (0x5565_35B2_340D_AA2D, 0xECBD), 0x02), // 3_unlock_ford
            ((0x5503_E802_A347_57F1, 0x6939), (0x5506_ED07_A642_52FE, 0x6AB8), 0x01), // Mondeo
            ((0x55DA_59BD_6189_7199, 0xB099), (0x55AA_29CD_11F9_E970, 0xB1F6), 0x01), // ts_x4
            ((0x55C4_9FFF_87B8_CD38, 0xD68E), (0x5508_5333_4B74_F4CE, 0xD74B), 0x10), // escape RS
        ];
        for ((key1, key2), (next1, next2), button) in pairs {
            let decoded = FordV0Decoder::frame_signal(key1, key2).expect("genuine frame");
            assert_eq!(decoded.button, Some(button));
            let upload = FordV0Decoder::new().encode(&decoded, button).unwrap();
            let out = feed_all(&mut FordV0Decoder::new(), &upload);
            let first = out.first().expect("re-encoded frame decodes");
            assert_eq!(first.data, next1, "key1 of the next press");
            let new_key2 = first.extra.unwrap() as u16;
            assert_eq!(new_key2 >> 8, next2 >> 8, "checksum byte");
            assert_eq!(new_key2 & 0x7F, next2 & 0x7F, "CRC (checked bits)");
        }
    }

    /// Without `extra` the encoder falls back to the reported counter/button.
    #[test]
    fn encode_without_extra_uses_reported_fields() {
        let mut decoded = FordV0Decoder::frame_signal(FOCUS_LOCK_KEY1, FOCUS_LOCK_KEY2).unwrap();
        decoded.extra = None;
        let upload = FordV0Decoder::new().encode(&decoded, 0x00).unwrap();
        let out = feed_all(&mut FordV0Decoder::new(), &upload);
        // The next genuine Lock press in 2008_Focus_Lock.sub (counter 0x08781).
        assert_eq!(out[0].data, 0x55C0_853D_D393_8306, "16-bit counter suffices for this frame");
    }

    /// A single flipped data bit on air must be rejected by the CRC gate (the C would emit it).
    #[test]
    fn rejects_corrupted_frame() {
        let good = FordV0Decoder::build_upload(FOCUS_LOCK_KEY1, FOCUS_LOCK_KEY2);
        assert_eq!(feed_all(&mut FordV0Decoder::new(), &good).len(), TOTAL_BURSTS as usize);
        // Corrupt one key1 bit but keep the original key2 (checksum + CRC).
        let bad = FordV0Decoder::build_upload(FOCUS_LOCK_KEY1 ^ (1 << 33), FOCUS_LOCK_KEY2);
        assert!(feed_all(&mut FordV0Decoder::new(), &bad).is_empty());
    }

    /// Real captures: every decode is a CRC-valid Ford V0 frame with one serial per fob, the button
    /// named in the file and a counter that only moves forward.
    #[test]
    fn decodes_real_captures() {
        let cases: [(&str, u32, &[u8]); 7] = [
            ("2008_Focus_Lock.sub", 0x4306_BE50, &[0x01]),
            ("2008_Focus_Unlock.sub", 0x4306_BE50, &[0x02]),
            ("2008_Focus_Trunk.sub", 0x4306_BE50, &[0x04]),
            ("3_unlock_ford.sub", 0x4818_9F19, &[0x02]),
            ("Ford Mondeo.sub", 0x54BF_55F4, &[0x01]),
            ("0000_Ford_ts_x4.sub", 0x43C0_24F8, &[0x01, 0x02]),
            // rs2 l2 u2 t2 p2: Remote start, Lock, Unlock, Trunk, Panic
            ("16_ford_escape_rs2_l2_u2_t2_p2.sub", 0xFCA7_C7BF, &[0x10, 0x01, 0x02, 0x04, 0x08]),
        ];
        for (rel, serial, buttons) in cases {
            let Some(decodes) = real_decodes(rel) else { continue };
            assert!(decodes.len() >= 3, "{rel}: only {} decodes", decodes.len());
            assert!(decodes.iter().all(|d| d.crc_valid), "{rel}: CRC");
            assert!(decodes.iter().all(|d| d.serial == Some(serial)), "{rel}: serial");
            let seen: BTreeSet<u8> = decodes.iter().filter_map(|d| d.button).collect();
            let want: BTreeSet<u8> = buttons.iter().copied().collect();
            assert_eq!(seen, want, "{rel}: buttons");
            let counters: Vec<u16> = decodes.iter().filter_map(|d| d.counter).collect();
            assert!(counters.windows(2).all(|w| w[0] <= w[1]), "{rel}: counters {counters:?}");
            // Button order in the 16_ford_escape capture follows the file name.
            if rel.starts_with("16_") {
                let mut order: Vec<u8> = decodes.iter().filter_map(|d| d.button).collect();
                order.dedup();
                assert_eq!(order, buttons, "{rel}: press order");
            }
        }
    }
}
