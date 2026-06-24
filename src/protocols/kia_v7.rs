//! Kia V7 protocol decoder/encoder
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/kia_v7.c` and `kia_v7.h`.
//! Manchester 250/500µs, 64 bits, FM. The decoded 64-bit word is bit-inverted (`~data`); a valid
//! frame has a fixed high byte 0x4C and a CRC8 (poly 0x7F, init 0x4C) over bytes 0..7. Emission is
//! gated on header + CRC, so Kia V7 is strongly validated and will not false-match.
//!
//! Decoder steps: Reset → Preamble (short pairs, ≥16) → SyncLow → Data. The preamble→sync transition
//! preloads four seed bits (1,0,1,1 = the inverted header's top nibble 0xB) before collecting the
//! remaining 60 Manchester bits. Encoder supported.

use super::{ProtocolDecoder, ProtocolTiming, DecodedSignal};
use super::common::crc8;
use crate::radio::demodulator::LevelDuration;
use crate::duration_diff;

const TE_SHORT: u32 = 250;
const TE_LONG: u32 = 500;
const TE_DELTA: u32 = 100;
const KEY_BITS: usize = 64;
const HEADER: u8 = 0x4C;
const PREAMBLE_MIN_PAIRS: u16 = 16;
const TAIL_GAP_US: u32 = 2000;
const TX_PREAMBLE_PAIRS: usize = 32;

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
    SyncLow,
    Data,
}

pub struct KiaV7Decoder {
    step: DecoderStep,
    manchester_state: ManchesterState,
    te_last: u32,
    preamble_count: u16,
    decode_data: u64,
    decode_count_bit: usize,
}

impl KiaV7Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            manchester_state: ManchesterState::Mid1,
            te_last: 0,
            preamble_count: 0,
            decode_data: 0,
            decode_count_bit: 0,
        }
    }

    fn is_short(d: u32) -> bool {
        duration_diff!(d, TE_SHORT) < TE_DELTA
    }
    fn is_long(d: u32) -> bool {
        duration_diff!(d, TE_LONG) < TE_DELTA
    }

    fn add_bit(&mut self, bit: bool) {
        self.decode_data = (self.decode_data << 1) | (bit as u64);
        self.decode_count_bit += 1;
    }

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

    /// Decode the 8 plaintext bytes of the (already inverted) key into fields.
    /// Returns (serial, button, counter, crc_valid).
    fn decode_key(data: u64) -> (u32, u8, u16, bool) {
        let bytes = data.to_be_bytes();
        let serial = (((bytes[3] as u32) << 20)
            | ((bytes[4] as u32) << 12)
            | ((bytes[5] as u32) << 4)
            | ((bytes[6] as u32) >> 4))
            & 0x0FFF_FFFF;
        let counter = ((bytes[1] as u16) << 8) | bytes[2] as u16;
        let button = bytes[6] & 0x0F;
        let crc_calc = crc8(&bytes[0..7], 0x7F, 0x4C);
        let crc_valid = crc_calc == bytes[7];
        (serial, button, counter, crc_valid)
    }

    /// Rebuild the 64-bit key from fields (matches kia_v7_encode_key).
    fn encode_key(serial: u32, button: u8, counter: u16) -> u64 {
        let serial = serial & 0x0FFF_FFFF;
        let button = button & 0x0F;
        let mut bytes = [0u8; 8];
        bytes[0] = HEADER;
        bytes[1] = (counter >> 8) as u8;
        bytes[2] = counter as u8;
        bytes[3] = (serial >> 20) as u8;
        bytes[4] = (serial >> 12) as u8;
        bytes[5] = (serial >> 4) as u8;
        bytes[6] = (((serial & 0x0F) as u8) << 4) | button;
        bytes[7] = crc8(&bytes[0..7], 0x7F, 0x4C);
        u64::from_be_bytes(bytes)
    }

    /// Map KAT generic button command to a Kia V7 4-bit button code.
    fn map_button(button: u8) -> u8 {
        match button {
            0x01 => 0x01, // Lock
            0x02 => 0x02, // Unlock
            0x04 => 0x03, // Trunk
            0x08 => 0x08, // Trunk/aux
            b => b & 0x0F,
        }
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
}

impl ProtocolDecoder for KiaV7Decoder {
    fn name(&self) -> &'static str {
        "Kia V7"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: KEY_BITS,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.manchester_state = ManchesterState::Mid1;
        self.te_last = 0;
        self.preamble_count = 0;
        self.decode_data = 0;
        self.decode_count_bit = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::Reset => {
                if level && Self::is_short(duration) {
                    self.step = DecoderStep::Preamble;
                    self.te_last = duration;
                    self.preamble_count = 0;
                    self.manchester_state = ManchesterState::Mid1;
                }
            }

            DecoderStep::Preamble => {
                if level {
                    if Self::is_long(duration) && Self::is_short(self.te_last) {
                        if self.preamble_count > (PREAMBLE_MIN_PAIRS - 1) {
                            self.decode_data = 0;
                            self.decode_count_bit = 0;
                            self.preamble_count = 0;
                            // Seed the inverted-header top nibble (1,0,1,1).
                            self.add_bit(true);
                            self.add_bit(false);
                            self.add_bit(true);
                            self.add_bit(true);
                            self.te_last = duration;
                            self.step = DecoderStep::SyncLow;
                        } else {
                            self.step = DecoderStep::Reset;
                        }
                    } else if Self::is_short(duration) {
                        self.te_last = duration;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else if Self::is_short(duration) && Self::is_short(self.te_last) {
                    self.preamble_count += 1;
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::SyncLow => {
                if !level && Self::is_short(duration) && Self::is_long(self.te_last) {
                    self.te_last = duration;
                    self.step = DecoderStep::Data;
                }
            }

            DecoderStep::Data => {
                let event = if Self::is_short(duration) {
                    Some(if level { 1 } else { 0 })
                } else if Self::is_long(duration) {
                    Some(if level { 3 } else { 2 })
                } else {
                    None
                };

                if let Some(ev) = event {
                    if let Some(bit) = self.manchester_advance(ev) {
                        self.add_bit(bit);
                    }
                }

                if self.decode_count_bit == KEY_BITS {
                    let candidate = !self.decode_data;
                    let hdr = (candidate >> 56) as u8;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.step = DecoderStep::Reset;

                    if hdr == HEADER {
                        let (serial, button, counter, crc_valid) = Self::decode_key(candidate);
                        if crc_valid {
                            return Some(DecodedSignal {
                                serial: Some(serial),
                                button: Some(button),
                                counter: Some(counter),
                                crc_valid: true,
                                data: candidate,
                                data_count_bit: KEY_BITS,
                                encoder_capable: true,
                                extra: None,
                                protocol_display_name: None,
                            });
                        }
                    }
                }
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        let serial = decoded.serial?;
        let counter = decoded.counter.unwrap_or(0);
        let key = Self::encode_key(serial, Self::map_button(button), counter);

        let mut signal = Vec::with_capacity(TX_PREAMBLE_PAIRS * 2 + KEY_BITS * 2 + 4);
        // Preamble: alternating short pulses.
        for _ in 0..TX_PREAMBLE_PAIRS {
            Self::enc_add_level(&mut signal, true, TE_SHORT);
            Self::enc_add_level(&mut signal, false, TE_SHORT);
        }
        // Standalone high short (merges with first data-bit high to form the long sync pulse).
        Self::enc_add_level(&mut signal, true, TE_SHORT);
        // Manchester data, MSB first: bit 1 → (H,L), bit 0 → (L,H).
        for bit in (0..KEY_BITS).rev() {
            let value = (key >> bit) & 1 != 0;
            if value {
                Self::enc_add_level(&mut signal, true, TE_SHORT);
                Self::enc_add_level(&mut signal, false, TE_SHORT);
            } else {
                Self::enc_add_level(&mut signal, false, TE_SHORT);
                Self::enc_add_level(&mut signal, true, TE_SHORT);
            }
        }
        // Trailing high short + tail gap.
        Self::enc_add_level(&mut signal, true, TE_SHORT);
        Self::enc_add_level(&mut signal, false, TAIL_GAP_US);
        Some(signal)
    }
}

impl Default for KiaV7Decoder {
    fn default() -> Self {
        Self::new()
    }
}
