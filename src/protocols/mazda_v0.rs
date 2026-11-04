//! Mazda V0 protocol decoder + encoder
//!
//! Port of ProtoPirate `REFERENCES/ProtoPirate/protocols/mazda_v0.c` as of ProtoPirate HEAD
//! (dc3446b; the file is functionally unchanged since 6758a4e "Fix Mazda V0 decoder and add
//! encoder"). Flipper-ARF `lib/subghz/protocols/mazda_v0.c` is the same decoder/encoder (plus
//! custom-button glue) and agrees with it.
//!
//! History: KAT's previous Mazda V0 was a port of ProtoPirate's ORIGINAL pair-based decoder
//! (@lupettohf, ProtoPirate 9ac1ea4), which ProtoPirate replaced in 6758a4e with the Manchester
//! decoder below. That pair-based decoder never decodes the air format the encoder emits: it
//! loses the frame's final bit to the trailing gap and so stops at 79 collected bits, one short
//! of its 80-bit completion minimum (the same algorithm survives as Flipper-ARF's Mazda Siemens
//! decoder — see `mazda_siemens.rs`).
//!
//! RF / timing (`subghz_protocol_mazda_v0_const`, protocol flags):
//! - 315 + 433.92 MHz, AM and FM.
//! - Manchester, te_short = 250µs, te_long = 500µs. Every pulse is classified with a tolerance of
//!   te_delta + 20 = 120µs (`mazda_v0_get_event`, strict `<`).
//! - Air bit (encoder): 1 = HIGH te then LOW te, 0 = LOW te then HIGH te, MSB first.
//!
//! Air frame (`mazda_v0_build_upload`), one upload:
//! `12 × 0xFF` | LOW 52 000µs gap (0xCB20) | `0xFF 0xFF 0xD7` (sync) | the 8 key bytes
//! one's-complemented (`~byte`) | `0x5A` tail | LOW 52 000µs gap. The Flipper TX loop sends the
//! upload `repeat` = 10 times; KAT's `encode()` returns the single upload, like other KAT encoders.
//!
//! Decoder (`subghz_protocol_decoder_mazda_v0_feed`), Flipper Manchester machine with
//! ProtoPirate's level→event mapping `level ? ShortLow : ShortHigh` (and Long likewise):
//! 1. Reset: a HIGH short pulse enters Preamble with the Manchester state seeded to Mid1.
//! 2. Preamble: every decoded bit shifts into an 8-bit pattern; 1s are counted, and a 0 seen
//!    before 16 ones restarts the count. Once more than 15 ones were seen and the pattern equals
//!    the sync byte 0xD7, switch to Data. A pulse that is neither short nor long → Reset.
//! 3. Data: collect exactly 64 bits MSB-first; key = !bits. Emit only if the checksum holds.
//!
//! Key layout (`mazda_v0_decode_key` / `mazda_v0_encode_key`), bytes b0..b7 of the 64-bit key:
//! - parity(b7) odd → XOR b0..b5 with b6; even → XOR b0..b4 and b6 with b5 (whitening).
//! - then serial = b0..b3 (32 bits), button = b4 >> 4 (4 bits), counter = 20 bits:
//!   `(b4 & 0x0F) << 16 | mid << 8 | lo` with `lo = (b5 & 0x55) | (b6 & 0xAA)` and
//!   `mid = (b6 & 0x55) | (b5 & 0xAA)` (bit-interleaved counter bytes).
//! - b7 = checksum = low byte of the sum of the four serial bytes, the two low counter bytes and
//!   `(button << 4) | counter[19:16]` (`mazda_v0_calculate_checksum`).
//! - Buttons (`mazda_v0_get_button_name`): 0x1 Lock, 0x2 Unlock, 0x4 Trunk, 0x8 Remote.
//!
//! `DecodedSignal` packing: `data` = the 64-bit key exactly as the C stores it (`generic.data`,
//! the whitened key = complement of the on-air data bits); `serial`/`button` as above; `counter` =
//! the LOW 16 bits of the 20-bit counter (KAT's counter is u16) — the top nibble stays in `data`
//! and the encoder recovers it from there. `data_count_bit` = 64.
//!
//! Gating: the C emits only on a valid checksum and so does KAT. Together with the ≥16-ones +
//! 0xD7 sync structure and the exact 64-bit length this keeps the decoder from firing on noise or
//! other 250/500µs protocols.
//!
//! Encoder (`subghz_protocol_encoder_mazda_v0_deserialize` + ProtoPirate's emulate scene):
//! decode the captured key, let the DecodedSignal fields override it (as the C lets the file's
//! Serial/Btn/Cnt fields override the Key), map KAT's generic button (0x01 Lock, 0x02 Unlock,
//! 0x04 Trunk, 0x08 Panic→Remote; anything else keeps the captured button — the emulate scene's
//! Up/OK/Down/Right mapping), bump the counter by one for the fresh press (the emulate scene does
//! `current_counter++`; Flipper-ARF does `cnt + mult`), mask to 20 bits, re-encode the key with
//! `mazda_v0_encode_key` and emit `mazda_v0_build_upload`. Adjacent same-level half-bits are
//! merged into single pulses (the C emits them separately; on air they are identical).
//!
//! Deviations from the C: none in the decoded result or the air waveform. The u16 preamble
//! counter wraps explicitly (the C relies on unsigned wraparound).

use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

const TE_SHORT: u32 = 250;
const TE_LONG: u32 = 500;
const TE_DELTA: u32 = 100;
const MIN_COUNT_BIT: usize = 64;

/// Pulse classification tolerance used by `mazda_v0_get_event` and the Reset step.
const EVENT_TOLERANCE: u32 = TE_DELTA + 20;
/// `MAZDA_V0_PREAMBLE_ONES`: more than 15 ones must precede the sync byte.
const PREAMBLE_ONES: u16 = 16;
/// `MAZDA_V0_SYNC_BYTE`.
const SYNC_BYTE: u8 = 0xD7;
/// `MAZDA_V0_TAIL_BYTE`.
const TAIL_BYTE: u8 = 0x5A;
/// `MAZDA_V0_GAP_US` (0xCB20).
const GAP_US: u32 = 52_000;
/// 0xFF wake-up bytes before the first gap (`mazda_v0_build_upload`).
const TX_PREAMBLE_BYTES: usize = 12;
/// The counter is 20 bits wide.
const COUNTER_MASK: u32 = 0xF_FFFF;

/// Frame button codes (`mazda_v0_get_button_name`).
const BTN_LOCK: u8 = 0x1;
const BTN_UNLOCK: u8 = 0x2;
const BTN_TRUNK: u8 = 0x4;
const BTN_REMOTE: u8 = 0x8;

/// Flipper `ManchesterEvent` values (shift amounts into the transition table).
const EVENT_SHORT_LOW: u8 = 0;
const EVENT_SHORT_HIGH: u8 = 2;
const EVENT_LONG_LOW: u8 = 4;
const EVENT_LONG_HIGH: u8 = 6;

/// Canonical Flipper Manchester states (lib/toolbox/manchester_decoder.c order).
#[derive(Debug, Clone, Copy, PartialEq)]
enum ManchesterState {
    Start1 = 0,
    Mid1 = 1,
    Mid0 = 2,
    Start0 = 3,
}

/// Decoder steps (`MazdaV0DecoderStep`).
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Preamble,
    Data,
}

/// Mazda V0 protocol decoder/encoder.
pub struct MazdaV0Decoder {
    step: DecoderStep,
    manchester_state: ManchesterState,
    preamble_count: u16,
    preamble_pattern: u8,
    decode_data: u64,
    decode_count_bit: usize,
}

impl MazdaV0Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            manchester_state: ManchesterState::Start1,
            preamble_count: 0,
            preamble_pattern: 0,
            decode_data: 0,
            decode_count_bit: 0,
        }
    }

    /// `mazda_v0_get_event`: classify a pulse into a Flipper Manchester event, with ProtoPirate's
    /// inverted level mapping (`level ? ShortLow : ShortHigh`). `None` = neither short nor long.
    fn get_event(duration: u32, level: bool) -> Option<u8> {
        if duration_diff!(duration, TE_SHORT) < EVENT_TOLERANCE {
            Some(if level {
                EVENT_SHORT_LOW
            } else {
                EVENT_SHORT_HIGH
            })
        } else if duration_diff!(duration, TE_LONG) < EVENT_TOLERANCE {
            Some(if level {
                EVENT_LONG_LOW
            } else {
                EVENT_LONG_HIGH
            })
        } else {
            None
        }
    }

    /// Flipper `manchester_advance` (canonical transition table). Returns the decoded bit, if any.
    fn manchester_advance(&mut self, event: u8) -> Option<bool> {
        const TRANSITIONS: [u8; 4] = [0b0000_0001, 0b1001_0001, 0b1001_1011, 0b1111_1011];
        let state_idx = self.manchester_state as u8;
        let new_idx = (TRANSITIONS[state_idx as usize] >> event) & 0x3;
        if new_idx == state_idx {
            // No progress → reset to Mid1, emit nothing.
            self.manchester_state = ManchesterState::Mid1;
            return None;
        }
        self.manchester_state = match new_idx {
            0 => ManchesterState::Start1,
            1 => ManchesterState::Mid1,
            2 => ManchesterState::Mid0,
            _ => ManchesterState::Start0,
        };
        match self.manchester_state {
            ManchesterState::Mid0 => Some(false),
            ManchesterState::Mid1 => Some(true),
            _ => None,
        }
    }

    /// `subghz_protocol_blocks_parity8`: 1 if the byte has an odd number of set bits.
    fn parity8(byte: u8) -> bool {
        byte.count_ones() & 1 == 1
    }

    /// `mazda_v0_calculate_checksum`.
    fn calculate_checksum(serial: u32, button: u8, counter: u32) -> u8 {
        let counter = counter & COUNTER_MASK;
        let sum = (serial >> 24)
            + ((serial >> 16) & 0xFF)
            + ((serial >> 8) & 0xFF)
            + (serial & 0xFF)
            + ((counter >> 8) & 0xFF)
            + (counter & 0xFF)
            + ((((counter >> 16) & 0x0F) | (((button & 0x0F) as u32) << 4)) & 0xFF);
        sum as u8
    }

    /// `mazda_v0_decode_key`: un-whiten the key and extract (serial, button, 20-bit counter).
    fn decode_key(key: u64) -> (u32, u8, u32) {
        let mut b = key.to_be_bytes();
        let parity = Self::parity8(b[7]);
        let limit = if parity { 6 } else { 5 };
        let mask = b[limit];
        for byte in b.iter_mut().take(limit) {
            *byte ^= mask;
        }
        if !parity {
            b[6] ^= mask;
        }
        let counter_lo = (b[5] & 0x55) | (b[6] & 0xAA);
        let counter_mid = (b[6] & 0x55) | (b[5] & 0xAA);
        let serial = u32::from_be_bytes([b[0], b[1], b[2], b[3]]);
        let button = (b[4] >> 4) & 0x0F;
        let counter =
            (((b[4] & 0x0F) as u32) << 16) | ((counter_mid as u32) << 8) | counter_lo as u32;
        (serial, button, counter)
    }

    /// `mazda_v0_encode_key`: build the whitened 64-bit key from the fields.
    fn encode_key(serial: u32, button: u8, counter: u32) -> u64 {
        let counter = counter & COUNTER_MASK;
        let button = button & 0x0F;
        let mut b = [0u8; 8];
        b[..4].copy_from_slice(&serial.to_be_bytes());
        b[4] = (button << 4) | ((counter >> 16) & 0x0F) as u8;
        b[5] = (counter >> 8) as u8;
        b[6] = counter as u8;
        b[7] = Self::calculate_checksum(serial, button, counter);

        let stored_5 = (b[6] & 0x55) | (b[5] & 0xAA);
        let stored_6 = (b[6] & 0xAA) | (b[5] & 0x55);
        let xor_mask = stored_5 ^ stored_6;
        let replace_second = !Self::parity8(b[7]);
        let forward_mask = if replace_second { stored_5 } else { stored_6 };
        b[5] = if replace_second { stored_5 } else { xor_mask };
        b[6] = if replace_second { xor_mask } else { stored_6 };
        for byte in b.iter_mut().take(5) {
            *byte ^= forward_mask;
        }
        u64::from_be_bytes(b)
    }

    /// Map KAT's generic button command to a Mazda V0 button code (ProtoPirate emulate scene:
    /// Up=Lock 0x1, OK=Unlock 0x2, Down=Trunk 0x4, Right=Remote 0x8, anything else = captured).
    fn map_button(button: u8, captured: u8) -> u8 {
        match button {
            0x01 => BTN_LOCK,
            0x02 => BTN_UNLOCK,
            0x04 => BTN_TRUNK,
            0x08 => BTN_REMOTE,
            _ => captured & 0x0F,
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

    /// `mazda_v0_append_byte` (`pp_emit_byte_manchester`): MSB first, 1 → (H,L), 0 → (L,H).
    fn enc_add_byte(signal: &mut Vec<LevelDuration>, value: u8) {
        for bit in (0..8).rev() {
            let one = (value >> bit) & 1 == 1;
            Self::enc_add_level(signal, one, TE_SHORT);
            Self::enc_add_level(signal, !one, TE_SHORT);
        }
    }

    /// `mazda_v0_build_upload` for a 64-bit key.
    fn build_upload(key: u64) -> Vec<LevelDuration> {
        let mut signal = Vec::with_capacity((TX_PREAMBLE_BYTES + 12) * 16 + 2);
        for _ in 0..TX_PREAMBLE_BYTES {
            Self::enc_add_byte(&mut signal, 0xFF);
        }
        Self::enc_add_level(&mut signal, false, GAP_US);
        Self::enc_add_byte(&mut signal, 0xFF);
        Self::enc_add_byte(&mut signal, 0xFF);
        Self::enc_add_byte(&mut signal, SYNC_BYTE);
        for byte in key.to_be_bytes() {
            Self::enc_add_byte(&mut signal, !byte);
        }
        Self::enc_add_byte(&mut signal, TAIL_BYTE);
        Self::enc_add_level(&mut signal, false, GAP_US);
        signal
    }

    /// Validate a complete 64-bit frame (Data step, count == 64).
    fn finish_frame(&self) -> Option<DecodedSignal> {
        let key = !self.decode_data;
        let (serial, button, counter) = Self::decode_key(key);
        if Self::calculate_checksum(serial, button, counter) != key as u8 {
            return None;
        }
        Some(DecodedSignal {
            serial: Some(serial),
            button: Some(button),
            counter: Some(counter as u16),
            crc_valid: true,
            data: key,
            data_count_bit: MIN_COUNT_BIT,
            encoder_capable: true,
            extra: None,
            protocol_display_name: None,
        })
    }
}

impl ProtocolDecoder for MazdaV0Decoder {
    fn name(&self) -> &'static str {
        "Mazda V0"
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
        // SubGhzProtocolFlag_315 | SubGhzProtocolFlag_433.
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.manchester_state = ManchesterState::Start1;
        self.preamble_count = 0;
        self.preamble_pattern = 0;
        self.decode_data = 0;
        self.decode_count_bit = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::Reset => {
                if level && duration_diff!(duration, TE_SHORT) < EVENT_TOLERANCE {
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.step = DecoderStep::Preamble;
                    self.manchester_state = ManchesterState::Mid1;
                    self.preamble_count = 0;
                    self.preamble_pattern = 0;
                }
            }

            DecoderStep::Preamble => {
                let Some(event) = Self::get_event(duration, level) else {
                    self.step = DecoderStep::Reset;
                    return None;
                };
                if let Some(bit) = self.manchester_advance(event) {
                    self.preamble_pattern = (self.preamble_pattern << 1) | bit as u8;
                    // C: a 0 with `preamble_count <= MAZDA_V0_PREAMBLE_ONES - 1` restarts the
                    // search; the sync needs `preamble_count > MAZDA_V0_PREAMBLE_ONES - 1`.
                    if bit {
                        self.preamble_count = self.preamble_count.wrapping_add(1);
                    } else if self.preamble_count < PREAMBLE_ONES {
                        self.preamble_count = 0;
                        self.preamble_pattern = 0;
                        return None;
                    }
                    if self.preamble_pattern == SYNC_BYTE && self.preamble_count >= PREAMBLE_ONES {
                        self.decode_data = 0;
                        self.decode_count_bit = 0;
                        self.step = DecoderStep::Data;
                    }
                }
            }

            DecoderStep::Data => {
                let Some(event) = Self::get_event(duration, level) else {
                    self.step = DecoderStep::Reset;
                    return None;
                };
                if let Some(bit) = self.manchester_advance(event) {
                    self.decode_data = (self.decode_data << 1) | bit as u64;
                    self.decode_count_bit += 1;
                    if self.decode_count_bit == MIN_COUNT_BIT {
                        let result = self.finish_frame();
                        self.reset();
                        return result;
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
        // subghz_block_generic_deserialize_check_count_bit: the key must be a 64-bit Mazda V0 key.
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        // mazda_v0_decode_key(Key), then the Serial/Btn/Cnt fields override it. KAT's counter is
        // the low 16 bits of the 20-bit counter, so the top nibble comes from the key.
        let (key_serial, key_button, key_counter) = Self::decode_key(decoded.data);
        let serial = decoded.serial.unwrap_or(key_serial);
        let captured_button = decoded.button.map(|b| b & 0x0F).unwrap_or(key_button);
        let counter = match decoded.counter {
            Some(low) => (key_counter & 0xF_0000) | low as u32,
            None => key_counter,
        };
        let button = Self::map_button(button, captured_button);
        // Fresh press: the emulate scene bumps the counter; the encoder masks it to 20 bits.
        let counter = counter.wrapping_add(1) & COUNTER_MASK;
        let key = Self::encode_key(serial, button, counter);
        Some(Self::build_upload(key))
    }
}

impl Default for MazdaV0Decoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::ProtocolRegistry;

    /// (serial, button, counter, key) produced by the UNMODIFIED ProtoPirate `mazda_v0.c`
    /// (`mazda_v0_encode_key`), compiled on the host. Covers both whitening branches
    /// (checksum parity odd/even), every button code and 20-bit counter extremes.
    const C_KEY_VECTORS: [(u32, u8, u32, u64); 8] = [
        (0x1234_5678, 0x1, 0x00001, 0x1234_5678_1001_0025),
        (0x1234_5678, 0x2, 0xABCDE, 0xEEC8_AA84_D6FC_62D8),
        (0xDEAD_BEEF, 0x4, 0xFFFFF, 0x2152_4110_B000_FF85),
        (0x0000_0000, 0x8, 0x00000, 0x0000_0000_8000_0080),
        (0xCAFE_BABE, 0x1, 0x7F3A1, 0x695D_191D_B4A3_52EB),
        (0x0A1B_2C3D, 0x2, 0x10203, 0x0918_2F3E_2203_01B4),
        (0xFFFF_FFFF, 0xF, 0xFFFFF, 0x0000_0000_00FF_00F9),
        (0x8000_0001, 0x4, 0x00FFF, 0xDF5F_5F5E_1F5F_F0CF),
    ];

    fn decode_all(pulses: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = MazdaV0Decoder::new();
        pulses
            .iter()
            .filter_map(|p| dec.feed(p.level, p.duration_us))
            .collect()
    }

    fn captured(serial: u32, button: u8, counter: u32) -> DecodedSignal {
        DecodedSignal {
            serial: Some(serial),
            button: Some(button),
            counter: Some(counter as u16),
            crc_valid: true,
            data: MazdaV0Decoder::encode_key(serial, button, counter),
            data_count_bit: MIN_COUNT_BIT,
            encoder_capable: true,
            extra: None,
            protocol_display_name: None,
        }
    }

    /// Small deterministic generator (xorshift64) for randomized round trips.
    fn rng(seed: u64) -> impl FnMut() -> u64 {
        let mut s = seed;
        move || {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            s
        }
    }

    /// Re-read an upload as Manchester bytes: split every pulse into 250µs half-bits, pair the
    /// halves ((H,L) = 1, (L,H) = 0) and return the byte blocks that precede each LOW gap. A gap
    /// longer than GAP_US has absorbed the final LOW half-bit of the block before it.
    fn upload_blocks(upload: &[LevelDuration]) -> Vec<Vec<u8>> {
        let mut blocks = Vec::new();
        let mut halves: Vec<bool> = Vec::new();
        for p in upload {
            let is_gap = p.duration_us >= GAP_US;
            let half_bits = if is_gap {
                assert!(!p.level, "gaps are LOW");
                (p.duration_us - GAP_US) / TE_SHORT
            } else {
                assert_eq!(p.duration_us % TE_SHORT, 0, "only 250/500µs pulses");
                p.duration_us / TE_SHORT
            };
            for _ in 0..half_bits {
                halves.push(p.level);
            }
            if is_gap {
                assert_eq!(halves.len() % 16, 0, "whole bytes before each gap");
                let bytes = halves
                    .chunks(16)
                    .map(|byte| {
                        byte.chunks(2).fold(0u8, |acc, half| {
                            assert_ne!(half[0], half[1], "every bit has a mid-bit transition");
                            (acc << 1) | half[0] as u8
                        })
                    })
                    .collect();
                blocks.push(bytes);
                halves.clear();
            }
        }
        assert!(halves.is_empty(), "the upload ends with the trailing gap");
        blocks
    }

    #[test]
    fn key_codec_matches_reference_c() {
        for &(serial, button, counter, key) in &C_KEY_VECTORS {
            assert_eq!(
                MazdaV0Decoder::encode_key(serial, button, counter),
                key,
                "encode_key({serial:08X}, {button:X}, {counter:05X})"
            );
            assert_eq!(
                MazdaV0Decoder::decode_key(key),
                (serial, button, counter),
                "decode_key({key:016X})"
            );
            assert_eq!(
                MazdaV0Decoder::calculate_checksum(serial, button, counter),
                key as u8
            );
        }
    }

    /// The waveform is exactly `mazda_v0_build_upload`: 12×0xFF, 52ms gap, FF FF D7, the eight
    /// complemented key bytes, 0x5A, 52ms gap.
    #[test]
    fn upload_layout_matches_reference() {
        let key = C_KEY_VECTORS[1].3;
        let upload = MazdaV0Decoder::build_upload(key);
        assert!(
            upload[0].level && upload[0].duration_us == TE_SHORT,
            "starts with a HIGH half-bit"
        );
        let gaps: Vec<u32> = upload
            .iter()
            .filter(|p| p.duration_us > 2 * TE_LONG)
            .map(|p| p.duration_us)
            .collect();
        // The first gap swallows the last 0xFF bit's LOW half; 0x5A ends on a HIGH half.
        assert_eq!(gaps, vec![GAP_US + TE_SHORT, GAP_US]);

        let blocks = upload_blocks(&upload);
        assert_eq!(blocks.len(), 2);
        assert_eq!(blocks[0], vec![0xFF; TX_PREAMBLE_BYTES]);
        let mut frame = vec![0xFF, 0xFF, SYNC_BYTE];
        frame.extend(key.to_be_bytes().iter().map(|b| !b));
        frame.push(TAIL_BYTE);
        assert_eq!(blocks[1], frame);
    }

    #[test]
    fn encode_decode_round_trip() {
        let mut next = rng(0x9E37_79B9_7F4A_7C15);
        for _ in 0..500 {
            let serial = next() as u32;
            let button = [BTN_LOCK, BTN_UNLOCK, BTN_TRUNK, BTN_REMOTE][(next() % 4) as usize];
            let counter = next() as u32 & COUNTER_MASK;
            let key = MazdaV0Decoder::encode_key(serial, button, counter);
            let frames = decode_all(&MazdaV0Decoder::build_upload(key));
            assert_eq!(
                frames.len(),
                1,
                "one frame for {serial:08X}/{button:X}/{counter:05X}"
            );
            let f = &frames[0];
            assert_eq!(f.serial, Some(serial));
            assert_eq!(f.button, Some(button));
            assert_eq!(f.counter, Some(counter as u16));
            assert_eq!(f.data, key);
            assert_eq!(
                MazdaV0Decoder::decode_key(f.data).2,
                counter,
                "20-bit counter in data"
            );
            assert!(f.crc_valid && f.encoder_capable);
            assert_eq!(f.data_count_bit, 64);
        }
    }

    /// `encode()` = fresh press: generic button mapped, counter + 1 (20-bit), serial kept.
    #[test]
    fn encode_maps_button_and_bumps_counter() {
        let dec = MazdaV0Decoder::new();
        let cases = [
            // (captured button, captured counter, generic command, expected button, expected counter)
            (BTN_LOCK, 0x1_FFFF, 0x02, BTN_UNLOCK, 0x2_0000),
            (BTN_UNLOCK, 0xF_FFFF, 0x01, BTN_LOCK, 0x0_0000),
            (BTN_LOCK, 0x0_0041, 0x04, BTN_TRUNK, 0x0_0042),
            (BTN_LOCK, 0x7_1234, 0x08, BTN_REMOTE, 0x7_1235),
            (BTN_TRUNK, 0x0_0005, 0x10, BTN_TRUNK, 0x0_0006), // unknown command keeps captured
        ];
        for (cap_btn, cap_cnt, command, want_btn, want_cnt) in cases {
            let upload = dec
                .encode(&captured(0x0A1B_2C3D, cap_btn, cap_cnt), command)
                .expect("encodes");
            let frames = decode_all(&upload);
            assert_eq!(frames.len(), 1);
            let f = &frames[0];
            assert_eq!(f.serial, Some(0x0A1B_2C3D));
            assert_eq!(f.button, Some(want_btn), "command {command:#04X}");
            assert_eq!(MazdaV0Decoder::decode_key(f.data).2, want_cnt);
            assert_eq!(f.counter, Some(want_cnt as u16));
        }
    }

    /// KAT's u16 counter field overrides the low 16 bits; the top nibble comes from the key.
    #[test]
    fn encode_takes_counter_field_low_bits_and_key_high_nibble() {
        let mut sig = captured(0x0102_0304, BTN_LOCK, 0x3_FFFE);
        sig.counter = Some(0x1234);
        let frames = decode_all(&MazdaV0Decoder::new().encode(&sig, 0x01).unwrap());
        assert_eq!(MazdaV0Decoder::decode_key(frames[0].data).2, 0x3_1235);

        sig.data_count_bit = 0;
        assert!(
            MazdaV0Decoder::new().encode(&sig, 0x01).is_none(),
            "needs a 64-bit key"
        );
    }

    #[test]
    fn rejects_corrupted_checksum() {
        let key = C_KEY_VECTORS[0].3;
        assert_eq!(decode_all(&MazdaV0Decoder::build_upload(key)).len(), 1);
        for flip in [1u64 << 1, 1 << 7, 1 << 20, 1 << 39, 1 << 63] {
            let bad = key ^ flip;
            let (s, b, c) = MazdaV0Decoder::decode_key(bad);
            assert_ne!(
                MazdaV0Decoder::calculate_checksum(s, b, c),
                bad as u8,
                "flip {flip:#x} must break the checksum"
            );
            assert!(
                decode_all(&MazdaV0Decoder::build_upload(bad)).is_empty(),
                "corrupted key {bad:016X} must be rejected"
            );
        }
    }

    /// The Preamble step needs > 15 ones followed by the exact 0xD7 sync.
    #[test]
    fn rejects_wrong_sync_or_short_preamble() {
        let key = C_KEY_VECTORS[4].3;
        let frame = |lead: &[u8], sync: u8| {
            let mut s = Vec::new();
            for &b in lead {
                MazdaV0Decoder::enc_add_byte(&mut s, b);
            }
            MazdaV0Decoder::enc_add_byte(&mut s, sync);
            for b in key.to_be_bytes() {
                MazdaV0Decoder::enc_add_byte(&mut s, !b);
            }
            MazdaV0Decoder::enc_add_byte(&mut s, TAIL_BYTE);
            MazdaV0Decoder::enc_add_level(&mut s, false, GAP_US);
            s
        };
        assert_eq!(
            decode_all(&frame(&[0xFF, 0xFF], SYNC_BYTE)).len(),
            1,
            "control"
        );
        assert!(
            decode_all(&frame(&[0xFF, 0xFF], 0xD6)).is_empty(),
            "wrong sync"
        );
        assert!(
            decode_all(&frame(&[0xFF, 0xFF], 0x57)).is_empty(),
            "wrong sync"
        );
        assert!(
            decode_all(&frame(&[0xFF], SYNC_BYTE)).is_empty(),
            "too few preamble ones"
        );
    }

    /// Real receivers jitter: every 250/500µs pulse may be off by up to ±100µs.
    #[test]
    fn decodes_with_timing_jitter() {
        let (serial, button, counter, key) = C_KEY_VECTORS[2];
        let mut next = rng(42);
        let jittered: Vec<LevelDuration> = MazdaV0Decoder::build_upload(key)
            .iter()
            .map(|p| {
                if p.duration_us >= GAP_US {
                    *p
                } else {
                    let j = (next() % 201) as u32; // 0..=200 → -100..=+100µs
                    LevelDuration::new(p.level, p.duration_us + j - 100)
                }
            })
            .collect();
        let frames = decode_all(&jittered);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].serial, Some(serial));
        assert_eq!(frames[0].button, Some(button));
        assert_eq!(frames[0].counter, Some(counter as u16));
    }

    /// Through the registry (both supported bands) the frame is claimed by "Mazda V0".
    #[test]
    fn registry_decodes_encoder_output() {
        let (serial, button, counter, _) = C_KEY_VECTORS[5];
        let upload = MazdaV0Decoder::new()
            .encode(&captured(serial, button, counter), 0x02)
            .unwrap();
        for freq in [433_920_000, 315_000_000] {
            let mut reg = ProtocolRegistry::new();
            let results = reg.process_signal_stream(&upload, freq);
            let names: Vec<&str> = results.iter().map(|(n, _, _)| n.as_str()).collect();
            assert_eq!(names, vec!["Mazda V0"], "at {freq} Hz");
            let d = &results[0].1;
            assert_eq!(d.serial, Some(serial));
            assert_eq!(d.button, Some(BTN_UNLOCK));
            assert_eq!(d.counter, Some((counter + 1) as u16));
        }
    }
}
