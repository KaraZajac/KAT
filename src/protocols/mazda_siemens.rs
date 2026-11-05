//! Mazda Siemens protocol decoder/encoder
//!
//! Ported from Flipper-ARF: `lib/subghz/protocols/mazda_siemens.c` / `mazda_siemens.h`
//! (`SUBGHZ_PROTOCOL_MAZDA_SIEMENS_NAME = "MazdaSiemens"`), the Siemens/VDO keyfob layer used on
//! some Mazda vehicles. Its air format is IDENTICAL to "Mazda V0" (`mazda_v0.rs`, ProtoPirate's
//! encoder/decoder): both reference encoders emit the same waveform (verified by compiling the two
//! C references on the host), so the two KAT decoders accept the same frames; only the field
//! interpretation differs (here: 8-bit button byte and 16-bit counter; Mazda V0: button nibble +
//! 20-bit counter). Flipper-ARF's `mazda_infinity.c` sends the same frame bytes with other gaps.
//!
//! Profile (matches the C const block):
//! - te_short = 250µs, te_long = 500µs, te_delta = 100µs, 64-bit frame.
//! - Pair-based decoder: `feed()` ignores `level` and interprets raw durations in pairs
//!   (`process_pair`), collecting bits with *inverted* polarity (`state_bit == 0` → stored 1).
//! - Preamble: ≥13 short/short pairs, then a short→long transition starts data; the first
//!   collected bit is a 1 (sync). A 14-byte buffer accumulates; on a non-matching pair the
//!   frame is checked: discard the leading sync byte, take 8 bytes, deobfuscate, validate.
//! - Siemens obfuscation (the layer ported here, `mazda_xor_deobfuscate`):
//!   parity = byte_parity(data[7]); odd → mask = data[6], XOR bytes 0..6;
//!   even → mask = data[5], XOR bytes 0..5 and byte 6. Then bit-deinterleave bytes 5/6
//!   via `(old5 & 0xAA)|(old6 & 0x55)` / `(old5 & 0x55)|(old6 & 0xAA)`.
//!   The inner Siemens cipher's plaintext is left as-is (no key); only this obfuscation/
//!   interleave layer is reversed, matching the C.
//! - Gate: additive checksum `sum(data[0..7]) == data[7]` (matches the C `mazda_check_completion`),
//!   plus the structural preamble/sync/bit-count constraints. This is what keeps it from
//!   false-matching other 250/500µs Manchester protocols.
//! - Fields (`mazda_parse_data`): serial = data >> 32 (bytes 0-3), button = (data >> 24) & 0xFF
//!   (byte 4), counter = (data >> 8) & 0xFFFF (bytes 5-6), checksum = byte 7. `data` holds the
//!   de-obfuscated cleartext (`generic.data` in the C). Button codes: 0x10 Lock, 0x20 Unlock,
//!   0x40 Trunk.
//! - RF: FM (`SubGhzProtocolFlag_FM`); 315 + 433.92 MHz (`SubGhzProtocolFlag_315 | _433`).
//!
//! **Deviation from the C (deliberate): `COMPLETION_MIN` is 79, not the C's 80.** The C decoder
//! cannot decode the C encoder's own frames: a bit is only collected when the pair holding its
//! half-bits is classified, and the frame's last half-bit merges into the trailing LOW gap, so the
//! pair holding the final `0x5A` tail bit never classifies. A complete frame therefore always
//! ends with `bit_counter` = 79 (7 sync bits at positions 1-7, 64 data bits at 8-71, 7 of the 8 tail
//! bits), one below `MAZDA_COMPLETION_MIN` = 80. Lowering the minimum by one is the smallest fix:
//! the data bytes (buffer[1..9]) are complete at 72, nothing else changes, and KAT still accepts
//! every frame the C accepts (80..=105). Verified on the host: the unmodified C decoder decodes 0
//! of 2000 C-encoder frames; with only this constant changed it decodes 5000/5000 (and 300/300
//! frames from ProtoPirate's Mazda V0 encoder).
//!
//! Encoder (`subghz_protocol_encoder_mazda_siemens_deserialize` + `_get_upload`): rebuild the
//! cleartext from the fields — bytes 0-3 serial, byte 4 button, bytes 5-6 counter — recompute the
//! checksum, bit-interleave + XOR (`mazda_xor_obfuscate`), then emit a 12-byte 0xFF preamble, a
//! 50ms gap, `0xFF 0xFF 0xD7`, the 8 obfuscated bytes transmitted as `255 - byte`, a `0x5A` tail
//! byte, and a trailing 50ms gap. Manchester per byte: bit 1 → (H,L), bit 0 → (L,H), all at
//! te_short; adjacent equal levels are merged (identical on air). The fresh press bumps the
//! 16-bit counter by one (the C adds the rolling-counter multiplier, 1 by default, when no
//! explicit Cnt is supplied). KAT's generic button maps like the C's D-pad remap: 0x01 → 0x10
//! Lock, 0x02 → 0x20 Unlock, 0x04 → 0x40 Trunk; anything else (incl. 0x08 Panic, which the fob
//! lacks) keeps the captured button. The Flipper TX loop repeats the upload 4 times; KAT's
//! `encode()` returns the single upload, like other KAT encoders.
//!
//! Key files (`subghz_protocol_decoder_mazda_siemens_serialize` / `_deserialize`): `Protocol:
//! MazdaSiemens` (ARF's name; KAT's key-file name table maps it back), `Bit: 64`, `Key` = the
//! cleartext, then `Serial`, `Btn` and `Cnt` (decimal). Loading parses a 64-bit key
//! (`mazda_parse_data`); the C loads any value, and `crc_valid` reports the additive checksum the
//! decoder gates on. The Serial/Btn/Cnt lines are not read back (they are overrides for ARF's
//! encoder, applied at transmit time; the decoder's deserialize takes the key).

use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Mazda Siemens",
    decoder: "Mazda Siemens",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::FM,
    encryption: "Siemens XOR",
    make: "Mazda",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&FM476],
};

/// Mazda Siemens / Mazda Infinity buttons.
pub(super) fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x10 => "Lock",
        0x20 => "Unlock",
        0x40 => "Trunk",
        _ => "Other",
    })
}

const TE_SHORT: u32 = 250;
const TE_LONG: u32 = 500;
const TE_DELTA: u32 = 100;
const MIN_COUNT_BIT: usize = 64;

const PREAMBLE_MIN: u16 = 13;
/// The C's `MAZDA_COMPLETION_MIN` is 80, which no frame can reach (the final tail bit is lost to
/// the trailing gap, see the module docs); 79 lets the reference encoder's frames decode.
const COMPLETION_MIN: u16 = 79;
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
        // uint16_t in the C: wrap instead of panicking on an endless run of valid pairs.
        self.bit_counter = self.bit_counter.wrapping_add(1);
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
        data.copy_from_slice(&self.data_buffer[1..9]);

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

        Some(Self::signal_from_packed(packed))
    }

    /// The decode of a cleartext frame (`mazda_parse_data`). `crc_valid` = the additive checksum
    /// (always true for a frame the decoder reports).
    fn signal_from_packed(packed: u64) -> DecodedSignal {
        let (serial, button, counter) = Self::parse_fields(packed);
        let bytes = packed.to_be_bytes();
        let checksum = bytes[..7].iter().fold(0u8, |acc, &b| acc.wrapping_add(b));
        DecodedSignal {
            serial: Some(serial),
            button: Some(button),
            counter: Some(counter),
            crc_valid: checksum == bytes[7],
            data: packed,
            data_count_bit: MIN_COUNT_BIT,
            encoder_capable: true,
            extra: None,
            protocol_display_name: None,
        }
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
            for b in &mut data[..6] {
                *b ^= mask;
            }
        } else {
            // Even parity: mask = byte[5], XOR bytes 0..5 and byte[6].
            let mask = data[5];
            for b in &mut data[..5] {
                *b ^= mask;
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
            for b in &mut data[..6] {
                *b ^= mask;
            }
        } else {
            let mask = data[5];
            for b in &mut data[..5] {
                *b ^= mask;
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

    /// Map KAT's generic button command to a Mazda Siemens frame button code (the C's D-pad
    /// remap: Up=Lock 0x10, Down=Unlock 0x20, Left=Trunk 0x40, anything else = captured button).
    fn map_button(button: u8, captured: u8) -> u8 {
        match button {
            0x01 => BTN_LOCK,   // Lock
            0x02 => BTN_UNLOCK, // Unlock
            0x04 => BTN_TRUNK,  // Trunk
            BTN_LOCK | BTN_UNLOCK | BTN_TRUNK => button, // already a frame code
            _ => captured,
        }
    }

    /// Cleartext frame as `get_upload` rebuilds it from the generic fields: bytes 0-3 serial,
    /// byte 4 button, bytes 5-6 counter, byte 7 additive checksum.
    fn build_cleartext(serial: u32, button: u8, counter: u16) -> [u8; 8] {
        let mut data = [0u8; 8];
        data[..4].copy_from_slice(&serial.to_be_bytes());
        data[4] = button;
        data[5..7].copy_from_slice(&counter.to_be_bytes());
        data[7] = data[..7].iter().fold(0u8, |sum, &b| sum.wrapping_add(b));
        data
    }

    /// Append a level, merging it into the previous pulse when the level is unchanged.
    fn enc_add_level(signal: &mut Vec<LevelDuration>, level: bool, duration: u32) {
        if let Some(last) = signal.last_mut() {
            if last.level == level {
                *last = LevelDuration::new(level, last.duration_us + duration);
                return;
            }
        }
        signal.push(LevelDuration::new(level, duration));
    }

    /// Encode one byte as Manchester, MSB-first: bit 1 → (H,L), bit 0 → (L,H) (mazda_encode_byte).
    fn enc_byte(signal: &mut Vec<LevelDuration>, byte: u8) {
        for bit in (0..8).rev() {
            let one = (byte >> bit) & 1 != 0;
            Self::enc_add_level(signal, one, TE_SHORT);
            Self::enc_add_level(signal, !one, TE_SHORT);
        }
    }

    /// The upload `get_upload` builds for a cleartext frame: 12x 0xFF preamble, gap, 0xFF 0xFF,
    /// sync 0xD7, the 8 obfuscated bytes transmitted inverted (255 - byte), tail 0x5A, gap.
    fn build_upload(clear: &[u8; 8]) -> Vec<LevelDuration> {
        let mut tx_data = *clear;
        Self::xor_obfuscate(&mut tx_data);

        let mut signal: Vec<LevelDuration> = Vec::with_capacity((TX_PREAMBLE_BYTES + 12) * 16 + 2);
        for _ in 0..TX_PREAMBLE_BYTES {
            Self::enc_byte(&mut signal, 0xFF);
        }
        Self::enc_add_level(&mut signal, false, TX_GAP_US);
        Self::enc_byte(&mut signal, 0xFF);
        Self::enc_byte(&mut signal, 0xFF);
        Self::enc_byte(&mut signal, TX_SYNC_BYTE);
        for &b in tx_data.iter() {
            Self::enc_byte(&mut signal, 255 - b);
        }
        Self::enc_byte(&mut signal, TX_TAIL_BYTE);
        Self::enc_add_level(&mut signal, false, TX_GAP_US);
        signal
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
        // SubGhzProtocolFlag_315 | SubGhzProtocolFlag_433.
        &[315_000_000, 433_920_000]
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
                    self.preamble_count = self.preamble_count.wrapping_add(1);
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
        // subghz_block_generic_deserialize_check_count_bit: the Key must be a 64-bit frame.
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        // mazda_parse_data(Key), then the Serial/Btn/Cnt fields override it (C deserialize).
        let (key_serial, key_button, key_counter) = Self::parse_fields(decoded.data);
        let serial = decoded.serial.unwrap_or(key_serial);
        let captured_button = decoded.button.unwrap_or(key_button);
        // Fresh press without an explicit Cnt: cnt + rolling multiplier (1), 16-bit.
        let counter = decoded.counter.unwrap_or(key_counter).wrapping_add(1);

        // The button goes into byte 4 (`data >> 24`); bytes 0-3 stay the serial.
        let clear =
            Self::build_cleartext(serial, Self::map_button(button, captured_button), counter);
        Some(Self::build_upload(&clear))
    }

    /// `subghz_protocol_decoder_mazda_siemens_deserialize`: a 64-bit cleartext key.
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != MIN_COUNT_BIT {
            return None;
        }
        Some(Self::signal_from_packed(key.key_u64()?))
    }

    /// `subghz_protocol_decoder_mazda_siemens_serialize`: Key, then Serial, Btn, Cnt.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        let (serial, button, counter) = Self::parse_fields(decoded.data);
        let mut key = FlipperKey::for_protocol("MazdaSiemens", MIN_COUNT_BIT, decoded.data);
        key.push_u32("Serial", serial);
        key.push_u32("Btn", button.into());
        key.push_u32("Cnt", counter.into());
        Some(key)
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
        let packed: u64 = 0x1234_5678_2000_063A;
        let (serial, button, counter) = MazdaSiemensDecoder::parse_fields(packed);
        assert_eq!(serial, 0x1234_5678);
        assert_eq!(button, 0x20);
        assert_eq!(counter, 0x0006);
    }

    /// (serial, button, counter, cleartext, on-air data bytes) produced by the UNMODIFIED
    /// Flipper-ARF `mazda_siemens.c` (`get_upload` + `mazda_xor_obfuscate`, air = 255 - tx),
    /// compiled on the host.
    #[rustfmt::skip]
    const C_VECTORS: [(u32, u8, u16, u64, u64); 6] = [
        (0x1234_5678, 0x20, 0x0006, 0x1234_5678_2000_063A, 0xE9CF_AD83_DBFB_F9C5),
        (0xDEAD_BEEF, 0x10, 0xFFFF, 0xDEAD_BEEF_10FF_FF46, 0xDEAD_BEEF_10FF_00B9),
        (0x0000_0000, 0x40, 0x0000, 0x0000_0000_4000_0040, 0xFFFF_FFFF_BFFF_FFBF),
        (0xCAFE_BABE, 0x20, 0x1234, 0xCAFE_BABE_2012_34A6, 0x2317_5357_C9E9_D959),
        (0x0A1B_2C3D, 0x10, 0x8001, 0x0A1B_2C3D_1080_011F, 0xF5E4_D3C2_EF7E_FFE0),
        (0xFFFF_FFFF, 0x40, 0x7FFF, 0xFFFF_FFFF_407F_FFBA, 0xFFFF_FFFF_407F_0045),
    ];

    fn captured(serial: u32, button: u8, counter: u16) -> DecodedSignal {
        let clear = MazdaSiemensDecoder::build_cleartext(serial, button, counter);
        DecodedSignal {
            serial: Some(serial),
            button: Some(button),
            counter: Some(counter),
            crc_valid: true,
            data: u64::from_be_bytes(clear),
            data_count_bit: MIN_COUNT_BIT,
            encoder_capable: true,
            extra: None,
            protocol_display_name: None,
        }
    }

    fn decode_all(pulses: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = MazdaSiemensDecoder::new();
        pulses
            .iter()
            .filter_map(|p| dec.feed(p.level, p.duration_us))
            .collect()
    }

    /// Re-read an upload as Manchester bytes ((H,L) = 1, (L,H) = 0, 250µs half-bits) and return
    /// the byte blocks before each LOW gap (a gap longer than TX_GAP_US absorbed a LOW half-bit).
    fn upload_blocks(upload: &[LevelDuration]) -> Vec<Vec<u8>> {
        let mut blocks = Vec::new();
        let mut halves: Vec<bool> = Vec::new();
        for p in upload {
            let is_gap = p.duration_us >= TX_GAP_US;
            let half_bits = if is_gap {
                assert!(!p.level, "gaps are LOW");
                (p.duration_us - TX_GAP_US) / TE_SHORT
            } else {
                assert_eq!(p.duration_us % TE_SHORT, 0, "only 250/500µs pulses");
                p.duration_us / TE_SHORT
            };
            for _ in 0..half_bits {
                halves.push(p.level);
            }
            if is_gap {
                assert_eq!(halves.len() % 16, 0, "whole bytes before each gap");
                blocks.push(
                    halves
                        .chunks(16)
                        .map(|byte| {
                            byte.chunks(2).fold(0u8, |acc, half| {
                                assert_ne!(half[0], half[1], "mid-bit transition");
                                (acc << 1) | half[0] as u8
                            })
                        })
                        .collect(),
                );
                halves.clear();
            }
        }
        assert!(halves.is_empty(), "the upload ends with the trailing gap");
        blocks
    }

    /// Cleartext and air bytes match the reference C, and the waveform is `get_upload`'s:
    /// 12×0xFF, gap, FF FF D7, eight inverted obfuscated bytes, 0x5A, gap.
    #[test]
    fn upload_matches_reference_c() {
        for &(serial, button, counter, clear, air) in &C_VECTORS {
            let bytes = MazdaSiemensDecoder::build_cleartext(serial, button, counter);
            assert_eq!(
                u64::from_be_bytes(bytes),
                clear,
                "cleartext for {serial:08X}"
            );
            let blocks = upload_blocks(&MazdaSiemensDecoder::build_upload(&bytes));
            assert_eq!(blocks.len(), 2);
            assert_eq!(blocks[0], vec![0xFF; TX_PREAMBLE_BYTES]);
            let mut frame = vec![0xFF, 0xFF, TX_SYNC_BYTE];
            frame.extend_from_slice(&air.to_be_bytes());
            frame.push(TX_TAIL_BYTE);
            assert_eq!(blocks[1], frame, "air bytes for {serial:08X}");
        }
    }

    /// encode() → decode() through this decoder: fields survive, the generic button lands in
    /// byte 4 and the counter is bumped by one.
    #[test]
    fn encode_decode_round_trip() {
        let dec = MazdaSiemensDecoder::new();
        let mut seed: u64 = 0x0DDB_A11C_0FFE_E123;
        let mut next = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            seed
        };
        for _ in 0..500 {
            let serial = next() as u32;
            let captured_button = [BTN_LOCK, BTN_UNLOCK, BTN_TRUNK][(next() % 3) as usize];
            let counter = next() as u16;
            let (command, want_button) =
                [(0x01, BTN_LOCK), (0x02, BTN_UNLOCK), (0x04, BTN_TRUNK)][(next() % 3) as usize];
            let upload = dec
                .encode(&captured(serial, captured_button, counter), command)
                .expect("encodes");
            let frames = decode_all(&upload);
            assert_eq!(frames.len(), 1, "one frame for {serial:08X}");
            let f = &frames[0];
            assert_eq!(f.serial, Some(serial));
            assert_eq!(f.button, Some(want_button));
            assert_eq!(f.counter, Some(counter.wrapping_add(1)));
            let want =
                MazdaSiemensDecoder::build_cleartext(serial, want_button, counter.wrapping_add(1));
            assert_eq!(f.data, u64::from_be_bytes(want));
            assert!(f.crc_valid && f.encoder_capable);
        }
    }

    /// Regression: the button used to be written into byte 3 (the serial's low byte).
    #[test]
    fn encode_keeps_serial_and_writes_button_byte() {
        let dec = MazdaSiemensDecoder::new();
        let sig = captured(0x1234_5678, BTN_UNLOCK, 0x00FF);
        let frames = decode_all(&dec.encode(&sig, 0x01).unwrap());
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].serial, Some(0x1234_5678));
        assert_eq!(frames[0].button, Some(BTN_LOCK));
        assert_eq!(frames[0].counter, Some(0x0100));
        // serial 12345678 | button 10 | counter 0100 | checksum 25
        assert_eq!(frames[0].data, 0x1234_5678_1001_0025);

        // Panic (absent on this fob) and unknown codes keep the captured button.
        for command in [0x08, 0x00, 0x80] {
            let frames = decode_all(&dec.encode(&sig, command).unwrap());
            assert_eq!(frames[0].button, Some(BTN_UNLOCK), "command {command:#04X}");
        }
        let mut no_key = sig.clone();
        no_key.data_count_bit = 0;
        assert!(dec.encode(&no_key, 0x01).is_none(), "needs a 64-bit frame");
    }

    /// Pins the C defect behind the COMPLETION_MIN deviation: a complete reference-encoder frame
    /// ends with exactly 79 collected bits (the final tail bit is swallowed by the gap).
    #[test]
    fn reference_frame_completes_at_79_bits() {
        let (serial, button, counter, clear, _) = C_VECTORS[0];
        let upload = MazdaSiemensDecoder::build_upload(&MazdaSiemensDecoder::build_cleartext(
            serial, button, counter,
        ));
        let mut dec = MazdaSiemensDecoder::new();
        let mut hit = None;
        for p in &upload {
            if let Some(d) = dec.feed(p.level, p.duration_us) {
                hit = Some((d, dec.bit_counter));
            }
        }
        let (decoded, bits) = hit.expect("decodes");
        assert_eq!(bits, 79, "C requires 80 here, which no frame reaches");
        assert_eq!(decoded.data, clear);
    }

    #[test]
    fn rejects_corrupted_checksum() {
        let (serial, button, counter, _, _) = C_VECTORS[3];
        let good = MazdaSiemensDecoder::build_cleartext(serial, button, counter);
        assert_eq!(
            decode_all(&MazdaSiemensDecoder::build_upload(&good)).len(),
            1
        );
        for (byte, mask) in [(7usize, 0x01u8), (7, 0x80), (0, 0x10), (4, 0x40), (6, 0x01)] {
            let mut bad = good;
            bad[byte] ^= mask;
            assert!(
                decode_all(&MazdaSiemensDecoder::build_upload(&bad)).is_empty(),
                "byte {byte} ^ {mask:#04X} must be rejected"
            );
        }
    }

    /// Same air format as Mazda V0, which sits first in the registry and completes the frame
    /// earlier (it stops after 64 data bits; this decoder waits for the trailing gap), so the
    /// registry reports such a frame as "Mazda V0" with the equivalent fields.
    #[test]
    fn registry_reports_shared_frames_as_mazda_v0() {
        use crate::protocols::ProtocolRegistry;
        let (serial, button, counter, _, _) = C_VECTORS[4];
        let upload = MazdaSiemensDecoder::build_upload(&MazdaSiemensDecoder::build_cleartext(
            serial, button, counter,
        ));
        let mut reg = ProtocolRegistry::new();
        let results = reg.process_signal_stream(&upload, 433_920_000);
        let names: Vec<&str> = results.iter().map(|(n, _, _)| n.as_str()).collect();
        assert_eq!(names, vec!["Mazda V0"]);
        let d = &results[0].1;
        assert_eq!(d.serial, Some(serial));
        assert_eq!(
            d.button,
            Some(button >> 4),
            "Mazda V0 reads the button nibble"
        );
        assert_eq!(d.counter, Some(counter));
    }

    /// Key files: the C's lines round-trip the decode (also through the registry's name table),
    /// a bad checksum loads with crc_valid false like the C, and TX from a loaded key decodes.
    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::{assert_round_trip, key};
        use crate::protocols::ProtocolRegistry;
        let dec = MazdaSiemensDecoder::new();
        let (serial, button, counter, clear, _) = C_VECTORS[0];
        let upload = MazdaSiemensDecoder::build_upload(&MazdaSiemensDecoder::build_cleartext(serial, button, counter));
        let decoded = decode_all(&upload).remove(0);
        assert_eq!(decoded.data, clear);
        let text = assert_round_trip(&dec, &decoded);
        assert!(
            text.ends_with(
                "Protocol: MazdaSiemens\nBit: 64\nKey: 12 34 56 78 20 00 06 3A\nSerial: 305419896\nBtn: 32\nCnt: 6\n"
            ),
            "{text}"
        );
        let reg = ProtocolRegistry::new();
        assert_eq!(reg.decode_key(&key(&text)), Some(("Mazda Siemens".to_string(), decoded.clone())));
        let mut bad = key(&text);
        bad.key ^= 1;
        bad.key_bytes = bad.key.to_be_bytes().to_vec();
        assert!(!dec.from_flipper_key(&bad).unwrap().crc_valid);
        bad.bits = 63;
        assert!(dec.from_flipper_key(&bad).is_none());

        let loaded = dec.from_flipper_key(&key(&text)).unwrap();
        let tx = decode_all(&dec.encode(&loaded, 0x01).unwrap());
        assert_eq!((tx[0].serial, tx[0].button, tx[0].counter), (Some(serial), Some(0x10), Some(counter + 1)));
    }

    /// Oracle vectors (`KAT_KEY_VECTORS=<dir> cargo test mazda_siemens_key_vectors -- --ignored`).
    #[test]
    #[ignore]
    fn mazda_siemens_key_vectors() {
        use super::super::bmw::{key_vector_dir, write_key_vector};
        let dec = MazdaSiemensDecoder::new();
        for (n, i) in [(1, 0), (2, 3)] {
            let (serial, button, counter, _, _) = C_VECTORS[i];
            let upload = MazdaSiemensDecoder::build_upload(&MazdaSiemensDecoder::build_cleartext(serial, button, counter));
            let decoded = decode_all(&upload).remove(0);
            let key = dec.to_flipper_key(&decoded).unwrap();
            write_key_vector(&key_vector_dir(), &format!("mazda_siemens_{n}"), 433_920_000, "FuriHalSubGhzPreset2FSKDev476Async", &upload, &key);
        }
    }
}
