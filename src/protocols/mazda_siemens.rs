//! Mazda Siemens protocol decoder/encoder
//!
//! Ported from Flipper-ARF: `lib/subghz/protocols/mazda_siemens.c` / `mazda_siemens.h`
//! (`SUBGHZ_PROTOCOL_MAZDA_SIEMENS_NAME = "MazdaSiemens"`). This is the Siemens/VDO keyfob
//! cipher used on some Mazda vehicles — a DIFFERENT protocol from KAT's existing "Mazda V0"
//! (Pandora) decoder, even though both ride a 250/500µs pair-based stream at 433.92 MHz FM.
//!
//! Profile (matches the C const block):
//! - te_short = 250µs, te_long = 500µs, te_delta = 100µs, 64-bit frame.
//! - Pair-based decoder: `feed()` ignores `level` and interprets raw durations in pairs
//!   (`process_pair`), collecting bits with *inverted* polarity (`state_bit == 0` → stored 1).
//! - Preamble: ≥13 short/short pairs, then a short→long transition starts data; the first
//!   collected bit is a 1 (sync). A 14-byte buffer accumulates; on a non-matching pair the
//!   frame is checked: discard the leading sync byte, take 8 bytes, deobfuscate, validate.
//! - Siemens obfuscation (the layer ported here, `mazda_xor_deobfuscate`):
//!     parity = byte_parity(data[7]); odd → mask = data[6], XOR bytes 0..6;
//!     even → mask = data[5], XOR bytes 0..5 and byte 6. Then bit-deinterleave bytes 5/6
//!     via `(old5 & 0xAA)|(old6 & 0x55)` / `(old5 & 0x55)|(old6 & 0xAA)`.
//!   The inner Siemens cipher's plaintext is left as-is (no key); only this obfuscation/
//!   interleave layer is reversed, matching the C.
//! - Gate: additive checksum `sum(data[0..7]) == data[7]` (matches the C `mazda_check_completion`),
//!   plus the structural preamble/sync/bit-count constraints. This is what keeps it from
//!   false-matching other 250/500µs Manchester protocols.
//! - Fields (`mazda_parse_data`): serial = data >> 32, button = (data >> 24) & 0xFF,
//!   counter = (data >> 8) & 0xFFFF. Button codes: 0x10 Lock, 0x20 Unlock, 0x40 Trunk.
//! - RF: FM (`SubGhzProtocolFlag_FM`); 433.92 MHz only (`SubGhzProtocolFlag_433`).
//!
//! Encoder (`subghz_protocol_encoder_mazda_siemens_get_upload`): increments the counter byte,
//! recomputes the checksum, bit-interleaves + XORs (`mazda_xor_obfuscate`), then emits a 12-byte
//! 0xFF preamble, a 50ms gap, `0xFF 0xFF 0xD7`, the 8 obfuscated bytes transmitted as `255 - byte`,
//! a `0x5A` tail byte, and a trailing 50ms gap. Manchester per byte: bit 1 → (H,L), bit 0 → (L,H),
//! all at te_short. Implemented faithfully in `encode()`.
//!
//! Note: as in the C, the decoder and encoder are NOT a clean raw-timing round-trip pair (the
//! decoder targets real fob air-format; the encoder generates a TX upload). The encode↔decode
//! unit test therefore validates the obfuscate/deobfuscate (XOR + interleave) cipher layer,
//! which is a true inverse.

use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

const TE_SHORT: u32 = 250;
const TE_LONG: u32 = 500;
const TE_DELTA: u32 = 100;
const MIN_COUNT_BIT: usize = 64;

const PREAMBLE_MIN: u16 = 13;
const COMPLETION_MIN: u16 = 80;
const COMPLETION_MAX: u16 = 105;
const DATA_BUFFER_SIZE: usize = 14;

// Encoder constants (mazda_siemens.c).
const TX_PREAMBLE_BYTES: usize = 12;
const TX_GAP_US: u32 = 50_000;
const TX_SYNC_BYTE: u8 = 0xD7;
const TX_TAIL_BYTE: u8 = 0x5A;

/// Button codes carried in the frame (mazda_get_btn_name in the C).
const BTN_LOCK: u8 = 0x10;
const BTN_UNLOCK: u8 = 0x20;
const BTN_TRUNK: u8 = 0x40;

/// Decoder states (matches mazda_siemens.c MazdaSiemensDecoderStep).
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    PreambleSave,
    PreambleCheck,
    DataSave,
    DataCheck,
}

/// Mazda Siemens protocol decoder.
pub struct MazdaSiemensDecoder {
    step: DecoderStep,
    te_last: u32,
    preamble_count: u16,
    bit_counter: u16,
    prev_state: u8,
    data_buffer: [u8; DATA_BUFFER_SIZE],
}

impl MazdaSiemensDecoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            te_last: 0,
            preamble_count: 0,
            bit_counter: 0,
            prev_state: 0,
            data_buffer: [0u8; DATA_BUFFER_SIZE],
        }
    }

    #[inline]
    fn is_short(duration: u32) -> bool {
        duration_diff!(duration, TE_SHORT) < TE_DELTA
    }

    #[inline]
    fn is_long(duration: u32) -> bool {
        duration_diff!(duration, TE_LONG) < TE_DELTA
    }

    /// Collect one bit into the buffer with inverted polarity (mazda_collect_bit).
    /// `state_bit == 0` stores a 1.
    fn collect_bit(&mut self, state_bit: u8) {
        let byte_idx = (self.bit_counter >> 3) as usize;
        if byte_idx < DATA_BUFFER_SIZE {
            self.data_buffer[byte_idx] <<= 1;
            if state_bit == 0 {
                self.data_buffer[byte_idx] |= 1;
            }
        }
        self.bit_counter += 1;
    }

    /// Process a duration pair (mazda_process_pair). Returns true if the pair was valid.
    fn process_pair(&mut self, dur_first: u32, dur_second: u32) -> bool {
        let first_short = Self::is_short(dur_first);
        let first_long = Self::is_long(dur_first);
        let second_short = Self::is_short(dur_second);
        let second_long = Self::is_long(dur_second);

        if first_long && second_short {
            self.collect_bit(0);
            self.collect_bit(1);
            self.prev_state = 1;
            return true;
        }

        if first_short && second_long {
            self.collect_bit(1);
            self.prev_state = 0;
            return true;
        }

        if first_short && second_short {
            let ps = self.prev_state;
            self.collect_bit(ps);
            return true;
        }

        if first_long && second_long {
            self.collect_bit(0);
            self.collect_bit(1);
            self.prev_state = 0;
            return true;
        }

        false
    }

    /// Validate a complete frame (mazda_check_completion). On success returns a DecodedSignal.
    fn check_completion(&self) -> Option<DecodedSignal> {
        if self.bit_counter < COMPLETION_MIN || self.bit_counter > COMPLETION_MAX {
            return None;
        }

        // Shift buffer by 1 byte (discard the sync/header byte).
        let mut data = [0u8; 8];
        for i in 0..8 {
            data[i] = self.data_buffer[i + 1];
        }

        Self::xor_deobfuscate(&mut data);

        // Additive checksum: sum(data[0..7]) must equal data[7].
        let mut checksum: u8 = 0;
        for &b in data.iter().take(7) {
            checksum = checksum.wrapping_add(b);
        }
        if checksum != data[7] {
            return None;
        }

        // Pack into u64 (big-endian byte order).
        let packed = u64::from_be_bytes(data);

        let (serial, button, counter) = Self::parse_fields(packed);

        Some(DecodedSignal {
            serial: Some(serial),
            button: Some(button),
            counter: Some(counter),
            crc_valid: true,
            data: packed,
            data_count_bit: MIN_COUNT_BIT,
            encoder_capable: true,
            extra: None,
            protocol_display_name: None,
        })
    }

    /// Field extraction (mazda_parse_data).
    fn parse_fields(packed: u64) -> (u32, u8, u16) {
        let serial = (packed >> 32) as u32;
        let button = ((packed >> 24) & 0xFF) as u8;
        let counter = ((packed >> 8) & 0xFFFF) as u16;
        (serial, button, counter)
    }

    /// Byte parity: XOR-fold to a single bit (mazda_byte_parity).
    fn byte_parity(mut val: u8) -> u8 {
        val ^= val >> 4;
        val ^= val >> 2;
        val ^= val >> 1;
        val & 1
    }

    /// Siemens RX deobfuscation (mazda_xor_deobfuscate):
    /// parity-dependent XOR mask, then deinterleave bytes 5/6.
    fn xor_deobfuscate(data: &mut [u8; 8]) {
        let parity = Self::byte_parity(data[7]);

        if parity != 0 {
            // Odd parity: mask = byte[6], XOR bytes 0..6.
            let mask = data[6];
            for i in 0..6 {
                data[i] ^= mask;
            }
        } else {
            // Even parity: mask = byte[5], XOR bytes 0..5 and byte[6].
            let mask = data[5];
            for i in 0..5 {
                data[i] ^= mask;
            }
            data[6] ^= mask;
        }

        // Bit deinterleave bytes 5/6.
        let old5 = data[5];
        let old6 = data[6];
        data[5] = (old5 & 0xAA) | (old6 & 0x55);
        data[6] = (old5 & 0x55) | (old6 & 0xAA);
    }

    /// Siemens TX obfuscation (mazda_xor_obfuscate): interleave bytes 5/6, then
    /// parity-dependent XOR mask. Inverse of `xor_deobfuscate`.
    fn xor_obfuscate(data: &mut [u8; 8]) {
        let old5 = data[5];
        let old6 = data[6];
        data[5] = (old5 & 0xAA) | (old6 & 0x55);
        data[6] = (old5 & 0x55) | (old6 & 0xAA);

        let parity = Self::byte_parity(data[7]);

        if parity != 0 {
            let mask = data[6];
            for i in 0..6 {
                data[i] ^= mask;
            }
        } else {
            let mask = data[5];
            for i in 0..5 {
                data[i] ^= mask;
            }
            data[6] ^= mask;
        }
    }

    /// Button name for display (mazda_get_btn_name).
    #[allow(dead_code)]
    fn get_button_name(btn: u8) -> &'static str {
        match btn {
            BTN_LOCK => "Lock",
            BTN_UNLOCK => "Unlock",
            BTN_TRUNK => "Trunk",
            _ => "Unknown",
        }
    }

    /// Map KAT's generic button command to a Mazda Siemens frame button code.
    fn map_button(button: u8) -> u8 {
        match button {
            0x01 => BTN_LOCK,   // Lock
            0x02 => BTN_UNLOCK, // Unlock
            0x04 => BTN_TRUNK,  // Trunk
            BTN_LOCK | BTN_UNLOCK | BTN_TRUNK => button, // already a frame code
            _ => BTN_UNLOCK,
        }
    }

    /// Encode one byte as Manchester, MSB-first: bit 1 → (H,L), bit 0 → (L,H) (mazda_encode_byte).
    fn enc_byte(signal: &mut Vec<LevelDuration>, byte: u8) {
        for bit in (0..8).rev() {
            if (byte >> bit) & 1 != 0 {
                signal.push(LevelDuration::new(true, TE_SHORT));
                signal.push(LevelDuration::new(false, TE_SHORT));
            } else {
                signal.push(LevelDuration::new(false, TE_SHORT));
                signal.push(LevelDuration::new(true, TE_SHORT));
            }
        }
    }
}

impl ProtocolDecoder for MazdaSiemensDecoder {
    fn name(&self) -> &'static str {
        "Mazda Siemens"
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
        // SubGhzProtocolFlag_433 only.
        &[433_920_000]
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.te_last = 0;
        self.preamble_count = 0;
        self.bit_counter = 0;
        self.prev_state = 0;
        self.data_buffer = [0u8; DATA_BUFFER_SIZE];
    }

    fn feed(&mut self, _level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::Reset => {
                if Self::is_short(duration) {
                    self.te_last = duration;
                    self.preamble_count = 0;
                    self.step = DecoderStep::PreambleCheck;
                }
            }

            DecoderStep::PreambleSave => {
                self.te_last = duration;
                self.step = DecoderStep::PreambleCheck;
            }

            DecoderStep::PreambleCheck => {
                if Self::is_short(self.te_last) && Self::is_short(duration) {
                    self.preamble_count += 1;
                    self.step = DecoderStep::PreambleSave;
                } else if Self::is_short(self.te_last)
                    && Self::is_long(duration)
                    && self.preamble_count >= PREAMBLE_MIN
                {
                    // Preamble → data: seed the leading sync bit (a 1).
                    self.bit_counter = 1;
                    self.data_buffer = [0u8; DATA_BUFFER_SIZE];
                    self.collect_bit(1);
                    self.prev_state = 0;
                    self.step = DecoderStep::DataSave;
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::DataSave => {
                self.te_last = duration;
                self.step = DecoderStep::DataCheck;
            }

            DecoderStep::DataCheck => {
                if self.process_pair(self.te_last, duration) {
                    self.step = DecoderStep::DataSave;
                } else {
                    let result = self.check_completion();
                    self.step = DecoderStep::Reset;
                    if result.is_some() {
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
        // Rebuild the 8 cleartext bytes from the decoded 64-bit word.
        let mut data = decoded.data.to_be_bytes();

        // Apply the requested button into the frame's button field (byte index 3 = data >> 24).
        data[3] = Self::map_button(button);

        // Increment the counter byte (mazda_siemens.c get_upload): data[6]++ with carry into data[5].
        let (new6, carry) = data[6].overflowing_add(1);
        data[6] = new6;
        if carry {
            data[5] = data[5].wrapping_add(1);
        }

        // Recompute the additive checksum over bytes 0..7.
        let mut checksum: u8 = 0;
        for &b in data.iter().take(7) {
            checksum = checksum.wrapping_add(b);
        }
        data[7] = checksum;

        // Obfuscate (interleave + XOR) for transmission.
        let mut tx_data = data;
        Self::xor_obfuscate(&mut tx_data);

        // Build the upload: 12x 0xFF preamble, gap, 0xFF 0xFF, sync 0xD7,
        // 8 data bytes transmitted inverted (255 - byte), tail 0x5A, trailing gap.
        let mut signal: Vec<LevelDuration> = Vec::with_capacity((TX_PREAMBLE_BYTES + 12) * 16 + 4);
        for _ in 0..TX_PREAMBLE_BYTES {
            Self::enc_byte(&mut signal, 0xFF);
        }
        signal.push(LevelDuration::new(false, TX_GAP_US));
        Self::enc_byte(&mut signal, 0xFF);
        Self::enc_byte(&mut signal, 0xFF);
        Self::enc_byte(&mut signal, TX_SYNC_BYTE);
        for &b in tx_data.iter() {
            Self::enc_byte(&mut signal, 255 - b);
        }
        Self::enc_byte(&mut signal, TX_TAIL_BYTE);
        signal.push(LevelDuration::new(false, TX_GAP_US));

        Some(signal)
    }
}

impl Default for MazdaSiemensDecoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The obfuscate/deobfuscate (XOR + bit-interleave) layer is a true inverse for every
    /// valid cleartext (checksum byte fixes the parity branch). This is the cipher-layer
    /// round-trip the decoder relies on; validates the ported Siemens obfuscation.
    #[test]
    fn xor_layer_round_trips() {
        // Deterministic LCG over many cleartexts: 7 data bytes + matching additive checksum.
        let mut seed: u32 = 0x1234_5678;
        let mut next = || {
            seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            (seed >> 16) as u8
        };
        for _ in 0..20_000 {
            let mut clear = [0u8; 8];
            let mut sum: u8 = 0;
            for b in clear.iter_mut().take(7) {
                *b = next();
                sum = sum.wrapping_add(*b);
            }
            clear[7] = sum;

            let mut buf = clear;
            MazdaSiemensDecoder::xor_obfuscate(&mut buf);
            MazdaSiemensDecoder::xor_deobfuscate(&mut buf);
            assert_eq!(buf, clear, "obfuscate→deobfuscate must be identity");
        }
    }

    /// Deobfuscate is the exact inverse of obfuscate for both parity branches (odd: byte7
    /// has odd popcount; even: zero). Pins the parity-dependent mask selection.
    #[test]
    fn xor_both_parity_branches() {
        // Even-parity checksum byte (0x00 → parity 0): mask = byte[5].
        let mut even = [0x11u8, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x00];
        let orig_even = even;
        MazdaSiemensDecoder::xor_obfuscate(&mut even);
        MazdaSiemensDecoder::xor_deobfuscate(&mut even);
        assert_eq!(even, orig_even);
        assert_eq!(MazdaSiemensDecoder::byte_parity(0x00), 0);

        // Odd-parity checksum byte (0x01 → parity 1): mask = byte[6].
        let mut odd = [0xAAu8, 0xBB, 0xCC, 0xDD, 0xEE, 0x12, 0x34, 0x01];
        let orig_odd = odd;
        MazdaSiemensDecoder::xor_obfuscate(&mut odd);
        MazdaSiemensDecoder::xor_deobfuscate(&mut odd);
        assert_eq!(odd, orig_odd);
        assert_eq!(MazdaSiemensDecoder::byte_parity(0x01), 1);
    }

    /// Field extraction matches mazda_parse_data: serial = >>32, button = >>24, counter = >>8.
    #[test]
    fn parse_fields_layout() {
        let packed: u64 = 0x1234_5678_2000_06_3A;
        let (serial, button, counter) = MazdaSiemensDecoder::parse_fields(packed);
        assert_eq!(serial, 0x1234_5678);
        assert_eq!(button, 0x20);
        assert_eq!(counter, 0x0006);
    }

    /// The encoder produces a non-empty Manchester upload with the expected leading 0xFF
    /// preamble (all alternating short pulses) and a 50ms gap. Smoke test of the TX path.
    #[test]
    fn encode_produces_upload() {
        let dec = MazdaSiemensDecoder::new();
        let signal = DecodedSignal {
            serial: Some(0x1234_5678),
            button: Some(0x02),
            counter: Some(0x0005),
            crc_valid: true,
            data: 0x1234_5678_2000_05_00,
            data_count_bit: MIN_COUNT_BIT,
            encoder_capable: true,
            extra: None,
            protocol_display_name: None,
        };
        let up = dec.encode(&signal, 0x02).expect("encode should succeed");
        assert!(!up.is_empty());
        // First 16 pulses are the first 0xFF preamble byte: alternating H/L shorts.
        for (i, p) in up.iter().take(16).enumerate() {
            assert_eq!(p.duration_us, TE_SHORT);
            assert_eq!(p.level, i % 2 == 0, "0xFF Manchester alternates H,L starting high");
        }
        // A 50ms gap is present somewhere in the upload.
        assert!(up.iter().any(|p| p.duration_us == TX_GAP_US));
    }
}
