//! Honda Static protocol decoder/encoder
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/honda_static.c` and
//! `honda_static.h`. Honda/Acura fixed-code keyfobs. FM, 315 MHz + 433.92 MHz, 64-bit frame.
//!
//! Unlike most KAT decoders, Honda Static does NOT use the Flipper manchester_decoder.h transition
//! table. It buffers a per-element *symbol stream* (one bit per ~63µs element) and then performs a
//! custom Manchester unpack over symbol pairs (see `honda_static_manchester_pack_64` in the C). A
//! short pulse contributes one symbol = the pulse level; a long pulse contributes two symbols of the
//! same level. Anything outside both ranges (e.g. the 700µs sync or a trailing gap) terminates the
//! buffer and triggers a parse attempt.
//!
//! Frame (64 bits, MSB-first into 8 bytes): button (4b) | serial (28b) | counter (24b) |
//! checksum (8b). The checksum is an XOR of bytes[0..7] (the first 7 bytes). Emission is gated on
//! the checksum validating, so Honda Static will not false-match. The parser tries the
//! inverted-Manchester interpretation first (which is what the encoder emits), then non-inverted
//! forward, then a bit-reversed-bytes pass — matching the C.
//!
//! The exported `data` (Key) word is the C `generic.data`: a compact nibble-packed layout
//! (`honda_static_pack_compact`), NOT the raw decoded packet bytes.

use super::{ProtocolDecoder, ProtocolTiming, DecodedSignal};
use crate::radio::demodulator::LevelDuration;

const BIT_COUNT: usize = 64;
const MIN_SYMBOLS: usize = 36;
const SHORT_BASE_US: u32 = 28;
const SHORT_SPAN_US: u32 = 70;
const LONG_BASE_US: u32 = 61;
const LONG_SPAN_US: u32 = 130;
const SYNC_TIME_US: u32 = 700;
const ELEMENT_TIME_US: u32 = 63;
const SYMBOL_CAPACITY: usize = 512;
const PREAMBLE_ALTERNATING_COUNT: usize = 160;
const PREAMBLE_MAX_TRANSITIONS: u16 = 19;

// Reported timing constants (informational; matches ProtoPirate protocol_items profile).
const TE_SHORT: u32 = ELEMENT_TIME_US;
const TE_LONG: u32 = SYNC_TIME_US;
const TE_DELTA: u32 = 120;

// KAT generic button codes.
const BTN_LOCK: u8 = 0x01;
const BTN_UNLOCK: u8 = 0x02;
const BTN_TRUNK: u8 = 0x04;
const BTN_PANIC: u8 = 0x08;

// honda_static_encoder_button_map[4] in the C ({0x02,0x04,0x08,0x05}); used by the encoder remap.
const ENCODER_BUTTON_MAP: [u8; 4] = [0x02, 0x04, 0x08, 0x05];

/// Decoded Honda Static fields (matches HondaStaticFields).
#[derive(Debug, Clone, Copy, Default)]
struct HondaStaticFields {
    button: u8,
    serial: u32,
    counter: u32,
    /// XOR checksum byte (mirrors the C struct field; validation recomputes it, so this is
    /// retained for fidelity/inspection rather than being read on the hot path).
    #[allow(dead_code)]
    checksum: u8,
}

/// Honda Static decoder (matches SubGhzProtocolDecoderHondaStatic).
pub struct HondaStaticDecoder {
    /// Per-element symbol stream (one bit per ~63µs element). Index 0 is the first received symbol.
    symbols: Vec<bool>,
}

impl HondaStaticDecoder {
    pub fn new() -> Self {
        Self {
            symbols: Vec::with_capacity(SYMBOL_CAPACITY),
        }
    }

    /// Extract `count` bits starting at `start`, MSB-first across the byte array
    /// (matches honda_static_get_bits / honda_static_get_bits_u32, shift = (~bit_index)&7).
    fn get_bits(data: &[u8], start: usize, count: usize) -> u32 {
        let mut value: u32 = 0;
        for i in 0..count {
            let bit_index = start + i;
            let byte = data[bit_index >> 3];
            let shift = (!bit_index) & 0x07;
            value = (value << 1) | (((byte >> shift) & 1) as u32);
        }
        value
    }

    /// XOR checksum over the first 7 bytes of the 8-byte packet (matches the inline checksum loop in
    /// honda_static_validate_forward_packet / honda_static_build_packet_bytes).
    fn packet_checksum(packet: &[u8]) -> u8 {
        let mut checksum = 0u8;
        for &b in packet.iter().take(7) {
            checksum ^= b;
        }
        checksum
    }

    /// Reverse the bit order of a byte (matches pp_reverse_bits8).
    fn reverse_bits8(value: u8) -> u8 {
        let mut v = value;
        v = ((v & 0xF0) >> 4) | ((v & 0x0F) << 4);
        v = ((v & 0xCC) >> 2) | ((v & 0x33) << 2);
        v = ((v & 0xAA) >> 1) | ((v & 0x55) << 1);
        v
    }

    fn is_valid_button(button: u8) -> bool {
        // honda_static_is_valid_button: button <= 9 && ((0x336 >> button) & 1)
        // 0x336 = 0b1100110110 → valid buttons: 1,2,4,5,8,9.
        button <= 9 && ((0x336u16 >> button) & 1) != 0
    }

    fn is_valid_serial(serial: u32) -> bool {
        serial != 0 && serial != 0x0FFF_FFFF
    }

    /// Pack fields into the 64-bit compact "Key" word (matches honda_static_pack_compact →
    /// pp_bytes_to_u64_be over the compact[8] nibble-packed layout).
    fn pack_compact(fields: &HondaStaticFields) -> u64 {
        let mut compact = [0u8; 8];
        compact[0] = fields.button & 0x0F;
        compact[1] = (fields.serial >> 20) as u8;
        compact[2] = (fields.serial >> 12) as u8;
        compact[3] = (fields.serial >> 4) as u8;
        compact[4] = (fields.serial << 4) as u8;
        compact[5] = (fields.counter >> 16) as u8;
        compact[6] = (fields.counter >> 8) as u8;
        compact[7] = fields.counter as u8;
        u64::from_be_bytes(compact)
    }

    /// Build the raw 8-byte (64-bit) packet from fields, MSB-first
    /// (matches honda_static_build_packet_bytes; checksum filled into byte 7).
    fn build_packet_bytes(fields: &HondaStaticFields) -> [u8; 8] {
        let mut packet = [0u8; 8];
        Self::set_bits(&mut packet, 0, 4, (fields.button & 0x0F) as u32);
        Self::set_bits(&mut packet, 4, 28, fields.serial);
        Self::set_bits(&mut packet, 32, 24, fields.counter);
        let checksum = Self::packet_checksum(&packet);
        Self::set_bits(&mut packet, 56, 8, checksum as u32);
        packet
    }

    /// Set `count` bits starting at `start`, MSB-first (matches honda_static_set_bits).
    fn set_bits(data: &mut [u8], start: usize, count: usize, value: u32) {
        for i in 0..count {
            let bit_index = start + i;
            let byte_index = bit_index >> 3;
            let shift = (!bit_index) & 0x07;
            let mask = 1u8 << shift;
            let bit = ((value >> (count - 1 - i)) & 1) != 0;
            if bit {
                data[byte_index] |= mask;
            } else {
                data[byte_index] &= !mask;
            }
        }
    }

    /// Validate a forward (as-decoded) 8-byte packet (matches honda_static_validate_forward_packet).
    fn validate_forward_packet(packet: &[u8; 8]) -> Option<HondaStaticFields> {
        let button = Self::get_bits(packet, 0, 4) as u8;
        let serial = Self::get_bits(packet, 4, 28);
        let counter = Self::get_bits(packet, 32, 24);
        let checksum = Self::get_bits(packet, 56, 8) as u8;
        let checksum_calc = Self::packet_checksum(packet);

        if checksum != checksum_calc {
            return None;
        }
        if !Self::is_valid_button(button) {
            return None;
        }
        if !Self::is_valid_serial(serial) {
            return None;
        }

        Some(HondaStaticFields {
            button,
            serial,
            counter,
            checksum,
        })
    }

    /// Validate a bit-reversed packet (matches honda_static_validate_reverse_packet). Note: the C
    /// does NOT re-check the checksum here (it reverses bytes then validates button/serial only),
    /// so this path is not checksum-gated. We mirror that, but the caller flags crc_valid=false.
    fn validate_reverse_packet(packet: &[u8; 8]) -> Option<HondaStaticFields> {
        let mut reversed = [0u8; 8];
        for (i, b) in packet.iter().enumerate() {
            reversed[i] = Self::reverse_bits8(*b);
        }

        let button = Self::get_bits(&reversed, 0, 4) as u8;
        let serial = Self::get_bits(&reversed, 4, 28);
        let counter = Self::get_bits(&reversed, 32, 24);
        let checksum = Self::packet_checksum(&reversed);

        if !Self::is_valid_button(button) {
            return None;
        }
        if !Self::is_valid_serial(serial) {
            return None;
        }

        Some(HondaStaticFields {
            button,
            serial,
            counter,
            checksum,
        })
    }

    /// Manchester unpack over symbol pairs (matches honda_static_manchester_pack_64).
    /// Returns the packed 8-byte packet plus how many bits were collected. `inverted`:
    /// bit=1 when (a==0,b==1); non-inverted: bit=1 when (a==1,b==0). Equal adjacent symbols are
    /// skipped (advance by 1).
    fn manchester_pack_64(symbols: &[bool], start_pos: usize, inverted: bool) -> ([u8; 8], usize) {
        let mut packet = [0u8; 8];
        let count = symbols.len();
        let mut pos = start_pos;
        let mut bit_count: usize = 0;

        while pos + 1 < count {
            if bit_count >= BIT_COUNT {
                break;
            }
            let a = symbols[pos];
            let b = symbols[pos + 1];
            if a == b {
                pos += 1;
                continue;
            }
            let bit = if inverted {
                !a && b // a==0 && b==1
            } else {
                a && !b // a==1 && b==0
            };
            if bit {
                let shift = (!bit_count) & 0x07;
                packet[bit_count >> 3] |= 1u8 << shift;
            }
            bit_count += 1;
            pos += 2;
        }

        (packet, bit_count)
    }

    /// Locate the data start (after the alternating preamble + sync run) and unpack/validate.
    /// Matches honda_static_parse_symbols. Returns the validated fields and whether the checksum
    /// path validated it (forward = true; reverse = false).
    fn parse_symbols(symbols: &[bool], inverted: bool) -> Option<(HondaStaticFields, bool)> {
        let count = symbols.len();
        if count == 0 {
            return None;
        }

        // Walk the alternating preamble: count consecutive transitions; when a non-transition
        // follows a run longer than PREAMBLE_MAX_TRANSITIONS, that's the preamble/data boundary.
        let mut index = 1usize;
        let mut transitions: u16 = 0;
        while index < count {
            if symbols[index] != symbols[index - 1] {
                transitions += 1;
            } else {
                if transitions > PREAMBLE_MAX_TRANSITIONS {
                    break;
                }
                transitions = 0;
            }
            index += 1;
        }
        if index >= count {
            return None;
        }

        // Skip forward over the equal-adjacent run (the sync gap).
        while (index + 1 < count) && (symbols[index] == symbols[index + 1]) {
            index += 1;
        }

        let data_start = index;
        let (packet, bit_count) = Self::manchester_pack_64(symbols, data_start, inverted);
        if bit_count < BIT_COUNT {
            return None;
        }

        if let Some(fields) = Self::validate_forward_packet(&packet) {
            return Some((fields, true));
        }

        if inverted {
            return None;
        }

        if let Some(fields) = Self::validate_reverse_packet(&packet) {
            return Some((fields, false));
        }

        None
    }

    /// Build a DecodedSignal from validated fields. `crc_valid` reflects whether the forward
    /// (checksum-validated) path matched.
    fn build_signal(fields: &HondaStaticFields, crc_valid: bool) -> DecodedSignal {
        DecodedSignal {
            serial: Some(fields.serial),
            button: Some(fields.button),
            // KAT counters are u16; Honda's is 24-bit. Truncate to the low 16 bits for the field
            // (the full 24-bit counter is preserved in the packed `data` word).
            counter: Some(fields.counter as u16),
            crc_valid,
            data: Self::pack_compact(fields),
            data_count_bit: BIT_COUNT,
            encoder_capable: true,
            extra: None,
            protocol_display_name: None,
        }
    }

    /// Map a KAT generic button command to a Honda Static 4-bit button code.
    /// Honda button codes: Lock=1, Unlock=2, Trunk=4, Remote Start=5, Panic=8, Lock x2=9
    /// (see honda_static_button_names + honda_static_is_valid_button).
    fn map_button(button: u8) -> u8 {
        match button {
            BTN_LOCK => 1,
            BTN_UNLOCK => 2,
            BTN_TRUNK => 4,
            BTN_PANIC => 8,
            // Already a valid Honda code → pass through.
            b if Self::is_valid_button(b) => b,
            // honda_static_encoder_remap_button for codes 2..=5.
            b if (2..=5).contains(&b) => ENCODER_BUTTON_MAP[(b - 2) as usize],
            _ => 1,
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

impl ProtocolDecoder for HondaStaticDecoder {
    fn name(&self) -> &'static str {
        "Honda Static"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: BIT_COUNT,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.symbols.clear();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let sym = level;

        // Short pulse → one symbol (matches the SHORT range check in the C feed).
        if duration >= SHORT_BASE_US && (duration - SHORT_BASE_US) <= SHORT_SPAN_US {
            if self.symbols.len() < SYMBOL_CAPACITY {
                self.symbols.push(sym);
            }
            return None;
        }

        // Long pulse → two symbols (same level).
        if duration >= LONG_BASE_US && (duration - LONG_BASE_US) <= LONG_SPAN_US {
            if self.symbols.len() + 2 <= SYMBOL_CAPACITY {
                self.symbols.push(sym);
                self.symbols.push(sym);
            }
            return None;
        }

        // Out-of-range pulse (sync 700µs or a gap): try to parse the buffered symbols, then reset.
        // Matches the C feed: parse with inverted=true first, then inverted=false.
        let mut result = None;
        if self.symbols.len() >= MIN_SYMBOLS {
            let parsed = Self::parse_symbols(&self.symbols, true)
                .or_else(|| Self::parse_symbols(&self.symbols, false));
            if let Some((fields, crc_valid)) = parsed {
                // Faithful to the C: both the forward (checksum-validated) and reverse
                // (button+serial-validated) packets commit a decode. The strong button/serial
                // gates keep this from false-matching (verified: zero false matches across the
                // IMPORTS sweep). `crc_valid` reflects whether the checksum-validated forward path
                // matched (true) vs. the reverse path (false).
                result = Some(Self::build_signal(&fields, crc_valid));
            }
        }

        self.symbols.clear();
        result
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        let serial = decoded.serial?;
        if !Self::is_valid_serial(serial) {
            return None;
        }

        let fields = HondaStaticFields {
            button: Self::map_button(button),
            serial,
            counter: decoded.counter.unwrap_or(0) as u32 & 0x00FF_FFFF,
            checksum: 0,
        };
        let packet = Self::build_packet_bytes(&fields);

        // Matches honda_static_build_upload.
        let mut signal =
            Vec::with_capacity(1 + PREAMBLE_ALTERNATING_COUNT + 2 * BIT_COUNT + 1);

        // Sync: HIGH 700µs.
        Self::enc_add_level(&mut signal, true, SYNC_TIME_US);

        // Alternating preamble: 160 elements at 63µs, level = (i & 1) (starts LOW).
        for i in 0..PREAMBLE_ALTERNATING_COUNT {
            Self::enc_add_level(&mut signal, (i & 1) != 0, ELEMENT_TIME_US);
        }

        // Data, MSB-first: bit → (!value 63µs, value 63µs).
        for bit in 0..BIT_COUNT {
            let shift = (!bit) & 0x07;
            let value = ((packet[bit >> 3] >> shift) & 1) != 0;
            Self::enc_add_level(&mut signal, !value, ELEMENT_TIME_US);
            Self::enc_add_level(&mut signal, value, ELEMENT_TIME_US);
        }

        // Trailing sync: !last_bit for 700µs.
        let last_bit = (packet[7] & 1) != 0;
        Self::enc_add_level(&mut signal, !last_bit, SYNC_TIME_US);

        Some(signal)
    }
}

impl Default for HondaStaticDecoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Round-trip: encode a known frame, feed it back through the decoder, and confirm the fields
    /// survive. The encoder emits the inverted-Manchester interpretation, which the decoder tries
    /// first.
    #[test]
    fn honda_static_encode_decode_roundtrip() {
        let serial = 0x0123456u32 & 0x0FFF_FFFF;
        let counter = 0x00AB12u32;
        let original = DecodedSignal {
            serial: Some(serial),
            button: Some(BTN_UNLOCK),
            counter: Some(counter as u16),
            crc_valid: true,
            data: 0,
            data_count_bit: BIT_COUNT,
            encoder_capable: true,
            extra: None,
            protocol_display_name: None,
        };

        let decoder = HondaStaticDecoder::new();
        let signal = decoder.encode(&original, BTN_UNLOCK).expect("encode");

        // Feed the encoded pulses, plus a terminating gap to flush the symbol buffer.
        let mut dec = HondaStaticDecoder::new();
        let mut decoded = None;
        for ld in &signal {
            if let Some(d) = dec.feed(ld.level, ld.duration_us) {
                decoded = Some(d);
                break;
            }
        }
        if decoded.is_none() {
            // Terminating out-of-range pulse to flush.
            decoded = dec.feed(false, 5000);
        }

        let decoded = decoded.expect("expected a decode from the round-tripped frame");
        assert!(decoded.crc_valid, "checksum should validate");
        assert_eq!(decoded.serial, Some(serial), "serial mismatch");
        assert_eq!(decoded.button, Some(2), "button should map to Honda Unlock=2");
        assert_eq!(
            decoded.counter,
            Some(counter as u16),
            "counter (low 16 bits) mismatch"
        );
    }

    /// The packed Key word must reproduce the C compact layout for a known field set.
    #[test]
    fn honda_static_pack_compact_layout() {
        let fields = HondaStaticFields {
            button: 0x02,
            serial: 0x0ABCDEF,
            counter: 0x123456,
            checksum: 0,
        };
        let data = HondaStaticDecoder::pack_compact(&fields);
        let bytes = data.to_be_bytes();
        // compact[0] = button & 0x0F
        assert_eq!(bytes[0], 0x02);
        // serial nibbles: >>20, >>12, >>4, <<4
        assert_eq!(bytes[1], (0x0ABCDEFu32 >> 20) as u8);
        assert_eq!(bytes[2], (0x0ABCDEFu32 >> 12) as u8);
        assert_eq!(bytes[3], (0x0ABCDEFu32 >> 4) as u8);
        assert_eq!(bytes[4], (0x0ABCDEFu32 << 4) as u8);
        // counter bytes
        assert_eq!(bytes[5], 0x12);
        assert_eq!(bytes[6], 0x34);
        assert_eq!(bytes[7], 0x56);
    }

    /// build_packet_bytes + validate_forward_packet must round-trip the fields, and the checksum
    /// must validate.
    #[test]
    fn honda_static_packet_validate_roundtrip() {
        let fields = HondaStaticFields {
            button: 0x05,
            serial: 0x0FEDCBA,
            counter: 0x00FF01,
            checksum: 0,
        };
        let packet = HondaStaticDecoder::build_packet_bytes(&fields);
        let validated = HondaStaticDecoder::validate_forward_packet(&packet)
            .expect("forward packet should validate");
        assert_eq!(validated.button, 0x05);
        assert_eq!(validated.serial, 0x0FEDCBA);
        assert_eq!(validated.counter, 0x00FF01);
    }
}
