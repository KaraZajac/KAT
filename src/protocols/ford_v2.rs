//! Ford V2 protocol decoder/encoder
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/ford_v2.c` and `ford_v2.h`.
//! Manchester 200/400µs (te_delta 260 → threshold ~460µs between short/long), 104 bits (13 bytes), FM.
//! Frame begins with a 16-bit Manchester sync that equals ~0x7FA7 (the decoder matches the *inverted*
//! shift register against 0x8058); the two sync bytes 0x7F 0xA7 head the 13-byte buffer. Data bits are
//! inverted before packing (`data_bit = !data_bit`). Structure is validated by the two sync bytes plus a
//! known button code. Encoder supported (matches subghz_protocol_encoder_ford_v2).
//!
//! Decoder steps: Reset → Preamble (≥64 shorts) → Sync (find 0x7FA7) → Data (11 bytes).

use super::{ProtocolDecoder, ProtocolTiming, DecodedSignal};
use crate::radio::demodulator::LevelDuration;
use crate::duration_diff;

const TE_SHORT: u32 = 200;
const TE_LONG: u32 = 400;
const TE_DELTA: u32 = 260;
const DATA_BITS: usize = 104;
const DATA_BYTES: usize = 13;
const PREAMBLE_MIN: u16 = 64;
const SYNC_0: u8 = 0x7F;
const SYNC_1: u8 = 0xA7;
const SYNC_BITS: u8 = 16;
const INTER_BURST_GAP_US: u32 = 15000;

// Encoder constants (subghz_protocol_encoder_ford_v2)
const ENC_TE_SHORT: u32 = 240;
const ENC_PREAMBLE_PAIRS: usize = 70;
const ENC_BURST_COUNT: usize = 6;
const ENC_INTER_BURST_GAP_US: u32 = 16000;
const ENC_SYNC_LO_US: u32 = 476;
const TAIL_RAW_BYTES: usize = 5;

/// Inverted 16-bit sync the decoder matches against (= !0x7FA7).
const SYNC_SHIFT16_INV: u16 = !(((SYNC_0 as u16) << 8) | SYNC_1 as u16);

#[derive(Debug, Clone, Copy, PartialEq)]
enum ManchesterState {
    Mid0 = 0,
    Mid1 = 1,
    Start0 = 2,
    Start1 = 3,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Preamble,
    Sync,
    Data,
}

pub struct FordV2Decoder {
    step: DecoderStep,
    manchester_state: ManchesterState,
    preamble_count: u16,
    raw_bytes: [u8; DATA_BYTES],
    byte_count: usize,
    decode_data: u16,
    decode_count_bit: usize,
    sync_shift: u16,
    sync_bit_count: u8,
}

impl FordV2Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            manchester_state: ManchesterState::Mid1,
            preamble_count: 0,
            raw_bytes: [0; DATA_BYTES],
            byte_count: 0,
            decode_data: 0,
            decode_count_bit: 0,
            sync_shift: 0,
            sync_bit_count: 0,
        }
    }

    fn reset_state(&mut self) {
        self.step = DecoderStep::Reset;
        self.manchester_state = ManchesterState::Mid1;
        self.preamble_count = 0;
        self.raw_bytes = [0; DATA_BYTES];
        self.byte_count = 0;
        self.decode_data = 0;
        self.decode_count_bit = 0;
        self.sync_shift = 0;
        self.sync_bit_count = 0;
    }

    fn is_short(d: u32) -> bool {
        duration_diff!(d, TE_SHORT) < TE_DELTA
    }

    fn is_long(d: u32) -> bool {
        duration_diff!(d, TE_LONG) < TE_DELTA
    }

    /// Flipper Manchester transition table (same as Ford V0). Event 0=ShortLow,1=ShortHigh,
    /// 2=LongLow,3=LongHigh. Returns Some(bit) when a bit emits.
    fn manchester_advance(&mut self, event: u8) -> Option<bool> {
        let (new_state, emit) = match (self.manchester_state, event) {
            (ManchesterState::Mid0, 0) => (ManchesterState::Mid0, false),
            (ManchesterState::Mid0, 1) => (ManchesterState::Start1, true),
            (ManchesterState::Mid0, 2) => (ManchesterState::Mid0, false),
            (ManchesterState::Mid0, 3) => (ManchesterState::Mid1, true),

            (ManchesterState::Mid1, 0) => (ManchesterState::Start0, true),
            (ManchesterState::Mid1, 1) => (ManchesterState::Mid1, false),
            (ManchesterState::Mid1, 2) => (ManchesterState::Mid0, true),
            (ManchesterState::Mid1, 3) => (ManchesterState::Mid1, false),

            (ManchesterState::Start0, 0) => (ManchesterState::Mid0, false),
            (ManchesterState::Start0, 1) => (ManchesterState::Mid0, false),
            (ManchesterState::Start0, 2) => (ManchesterState::Mid0, false),
            (ManchesterState::Start0, 3) => (ManchesterState::Mid1, false),

            (ManchesterState::Start1, 0) => (ManchesterState::Mid0, false),
            (ManchesterState::Start1, 1) => (ManchesterState::Mid1, false),
            (ManchesterState::Start1, 2) => (ManchesterState::Mid0, false),
            (ManchesterState::Start1, 3) => (ManchesterState::Mid1, false),

            _ => (ManchesterState::Mid1, false),
        };
        self.manchester_state = new_state;
        if emit { Some((event & 1) == 1) } else { None }
    }

    /// Map (level, duration) → Manchester event. Ford V2 polarity: level ? High : Low.
    fn pulse_event(level: bool, duration: u32) -> Option<u8> {
        if Self::is_short(duration) {
            Some(if level { 1 } else { 0 })
        } else if Self::is_long(duration) {
            Some(if level { 3 } else { 2 })
        } else {
            None
        }
    }

    fn button_is_valid(btn: u8) -> bool {
        matches!(btn, 0x10 | 0x11 | 0x13 | 0x14 | 0x15)
    }

    /// Enter the Sync step from the preamble, seeding state for the triggering long-low pulse.
    fn enter_sync_from_preamble(&mut self, level: bool, duration: u32) {
        self.step = DecoderStep::Sync;
        self.decode_data = 0;
        self.decode_count_bit = 0;
        self.byte_count = 0;
        self.sync_shift = 0;
        self.sync_bit_count = 0;
        self.raw_bytes = [0; DATA_BYTES];
        self.manchester_state = ManchesterState::Mid1;

        if let Some(event) = Self::pulse_event(level, duration) {
            // Low event (0/2) seeds Mid0 (matches the C `if(ev==ShortLow||ev==LongLow) state=Mid0`).
            if event == 0 || event == 2 {
                self.manchester_state = ManchesterState::Mid0;
            }
            self.feed_event(event);
        } else {
            self.reset_state();
        }
    }

    /// Process one Manchester event in Sync or Data step. Returns Some when a frame commits.
    fn feed_event(&mut self, event: u8) -> Option<DecodedSignal> {
        if self.step == DecoderStep::Sync {
            if let Some(bit) = self.manchester_advance(event) {
                self.sync_shift = (self.sync_shift << 1) | (bit as u16);
                if self.sync_bit_count < SYNC_BITS {
                    self.sync_bit_count += 1;
                }
                if self.sync_bit_count >= SYNC_BITS && self.sync_shift == SYNC_SHIFT16_INV {
                    // Enter data: prime sync bytes and bit counter.
                    self.raw_bytes = [0; DATA_BYTES];
                    self.raw_bytes[0] = SYNC_0;
                    self.raw_bytes[1] = SYNC_1;
                    self.byte_count = 2;
                    self.step = DecoderStep::Data;
                    self.decode_data = 0;
                    self.decode_count_bit = SYNC_BITS as usize;
                }
            }
            return None;
        }

        // Data step
        if let Some(bit) = self.manchester_advance(event) {
            let data_bit = !bit; // Ford V2 inverts decoded bits
            self.decode_data = (self.decode_data << 1) | (data_bit as u16);
            self.decode_count_bit += 1;

            if self.decode_count_bit & 7 == 0 {
                let byte_val = (self.decode_data & 0xFF) as u8;
                if self.byte_count < DATA_BYTES {
                    self.raw_bytes[self.byte_count] = byte_val;
                    self.byte_count += 1;
                }
                self.decode_data = 0;

                if self.byte_count == DATA_BYTES {
                    let result = self.commit_frame();
                    self.reset_state();
                    return result;
                }
            }
        }
        None
    }

    /// Validate sync bytes + structure and build the decoded signal.
    fn commit_frame(&self) -> Option<DecodedSignal> {
        let k = &self.raw_bytes;
        if k[0] != SYNC_0 || k[1] != SYNC_1 {
            return None;
        }
        if !Self::button_is_valid(k[6]) {
            return None;
        }

        let serial = ((k[2] as u32) << 24)
            | ((k[3] as u32) << 16)
            | ((k[4] as u32) << 8)
            | (k[5] as u32);
        let counter = (((k[7] & 0x7F) as u16) << 9) | ((k[8] as u16) << 1) | ((k[9] >> 7) as u16);

        // Top 8 bytes → 64-bit data for display/export (matches generic.data).
        let mut data = 0u64;
        for &byte in k.iter().take(8) {
            data = (data << 8) | byte as u64;
        }
        // Tail raw bytes k[8..13] → extra (40 bits) so the encoder can rebuild the full frame.
        let mut extra = 0u64;
        for &byte in k.iter().skip(8).take(TAIL_RAW_BYTES) {
            extra = (extra << 8) | byte as u64;
        }

        Some(DecodedSignal {
            serial: Some(serial),
            button: Some(k[6]),
            counter: Some(counter),
            crc_valid: true, // validated by sync bytes + button structure
            data,
            data_count_bit: DATA_BITS,
            encoder_capable: true,
            extra: Some(extra),
            protocol_display_name: None,
        })
    }

    /// Map KAT's generic button command to a Ford V2 button code.
    fn map_button(button: u8) -> u8 {
        match button {
            0x01 => 0x10, // Lock
            0x02 => 0x11, // Unlock
            0x04 => 0x13, // Trunk
            0x08 => 0x14, // Panic
            b if Self::button_is_valid(b) => b, // already a Ford V2 code
            _ => 0x11,
        }
    }

    fn parity8(mut v: u8) -> u8 {
        let mut p = 0u8;
        while v != 0 {
            p ^= v & 1;
            v >>= 1;
        }
        p
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

    fn enc_manchester_bit(signal: &mut Vec<LevelDuration>, bit: bool) {
        if bit {
            Self::enc_add_level(signal, true, ENC_TE_SHORT);
            Self::enc_add_level(signal, false, ENC_TE_SHORT);
        } else {
            Self::enc_add_level(signal, false, ENC_TE_SHORT);
            Self::enc_add_level(signal, true, ENC_TE_SHORT);
        }
    }
}

impl ProtocolDecoder for FordV2Decoder {
    fn name(&self) -> &'static str {
        "Ford V2"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: DATA_BITS,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.reset_state();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::Reset => {
                if Self::is_short(duration) {
                    self.preamble_count = 1;
                    self.step = DecoderStep::Preamble;
                }
            }

            DecoderStep::Preamble => {
                if Self::is_short(duration) {
                    if self.preamble_count < u16::MAX {
                        self.preamble_count += 1;
                    }
                } else if !level && Self::is_long(duration) {
                    if self.preamble_count >= PREAMBLE_MIN {
                        self.enter_sync_from_preamble(level, duration);
                    } else {
                        self.reset_state();
                    }
                } else {
                    self.reset_state();
                }
            }

            DecoderStep::Sync | DecoderStep::Data => {
                if let Some(event) = Self::pulse_event(level, duration) {
                    if let Some(result) = self.feed_event(event) {
                        return Some(result);
                    }
                } else {
                    // Non-short/long pulse (gap/out-of-range) ends the attempt.
                    let _ = duration >= INTER_BURST_GAP_US;
                    self.reset_state();
                }
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        // Rebuild the 13-byte frame: bytes 0..8 from data, tail bytes 8..13 from extra.
        let mut raw = [0u8; DATA_BYTES];
        for (i, b) in raw.iter_mut().take(8).enumerate() {
            *b = (decoded.data >> (56 - i * 8)) as u8;
        }
        let extra = decoded.extra.unwrap_or(0);
        for (i, b) in raw.iter_mut().skip(8).take(TAIL_RAW_BYTES).enumerate() {
            *b = (extra >> (32 - i * 8)) as u8;
        }

        // Apply the requested button and refresh the byte-7 parity MSB.
        raw[6] = Self::map_button(button);
        if !Self::button_is_valid(raw[6]) {
            return None;
        }
        let parity_msb = Self::parity8(raw[6]) << 7;
        raw[7] = (raw[7] & 0x7F) | parity_msb;
        // Ensure sync header is present.
        raw[0] = SYNC_0;
        raw[1] = SYNC_1;

        let mut signal = Vec::with_capacity(ENC_BURST_COUNT * (ENC_PREAMBLE_PAIRS * 2 + DATA_BITS * 2 + 4));
        for burst in 0..ENC_BURST_COUNT {
            // Preamble: 70 pairs of (low short, high short)
            for _ in 0..ENC_PREAMBLE_PAIRS {
                Self::enc_add_level(&mut signal, false, ENC_TE_SHORT);
                Self::enc_add_level(&mut signal, true, ENC_TE_SHORT);
            }
            // Sync low + high short
            Self::enc_add_level(&mut signal, false, ENC_SYNC_LO_US);
            Self::enc_add_level(&mut signal, true, ENC_TE_SHORT);
            // Data bits 1..103 (bit 0 implied by the high short above)
            for bit_pos in 1..DATA_BITS {
                let byte_idx = bit_pos / 8;
                let bit_idx = 7 - (bit_pos % 8);
                let bit = (raw[byte_idx] >> bit_idx) & 1 != 0;
                Self::enc_manchester_bit(&mut signal, bit);
            }
            if burst + 1 < ENC_BURST_COUNT {
                Self::enc_add_level(&mut signal, true, ENC_INTER_BURST_GAP_US);
            }
        }
        Some(signal)
    }
}

impl Default for FordV2Decoder {
    fn default() -> Self {
        Self::new()
    }
}
