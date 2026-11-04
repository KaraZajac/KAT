//! Honda V1 protocol decoder/encoder
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/honda_v1.c` and
//! `honda_v1.h`. Honda/Acura counter-based (rolling-code) keyfobs, a DIFFERENT protocol from `honda_static`.
//! AM (OOK), 315 MHz + 433.92 MHz. The on-wire frame carries 68 bits: 64-bit data + a 4-bit
//! CRC-fold nibble.
//!
//! **Encoding**: short/long pulse PWM. te_short=1000µs, te_long=2000µs, te_delta=400µs,
//! te_end=3500µs, te_short_min=600µs. Each on-wire symbol is one pulse whose width selects 0/1,
//! but the demodulated stream is glued together by a "pending bit" timing accumulator (see `feed`)
//! before being classified.
//!
//! **Pending-bit accumulation** (matches `subghz_protocol_decoder_honda_v1_feed`): sub-`te_delta`
//! runts are summed into `pending`; a HIGH level keeps extending the running HIGH pulse; a LOW
//! level flushes the accumulated HIGH pulse (if it reached `te_short_min`) as a synthetic symbol,
//! then the LOW pulse itself is classified. The symbol layer (`honda_v1_symbol`) walks a
//! Reset→Preamble→Data state machine; in the Data step a short pulse toggles a `data_pending` flag
//! and, when paired, emits the level as a bit, while a long pulse emits directly — this is the
//! exact pending-bit logic ported from the C.
//!
//! **Frame / fields**: after the end gap (>te_end), `commit` requires ≥68 collected bits, then
//! left-shifts the 12-byte bit buffer by `max(1, bit_count-68)` to drop leading preamble leakage
//! and align the trailing frame. `data` = first 8 bytes (64 bits), `k2` (CRC nibble) = byte 8's
//! high nibble. Fields (matches `honda_v1_decode_fields`): serial = data[63:36] (28b),
//! button = data[31:28] (nibble), counter = data[15:0] (16b).
//!
//! **Validation / gating**: a button-code table (Unlock=0, Lock=8, Trunk=9, Panic=10) plus a
//! CRC-fold checksum (`honda_v1_checksum*`). Emission is gated on the button being valid (matches
//! the C `commit`); `crc_valid` reflects whether the received CRC nibble matches either wire-order
//! checksum (`honda_v1_crc_valid`). The strong button gate keeps Honda V1 from false-matching.
//!
//! **Encoder** (`subghz_protocol_encoder_honda_v1_deserialize` + ProtoPirate's emulate scene;
//! Flipper-ARF's copy does the same with the counter bump inline): the Key's fields, overridden by
//! the DecodedSignal's serial/button/counter; KAT's button 0x01 Lock → 8, 0x02 Unlock → 0,
//! 0x04 Trunk → 9, 0x08 Panic → 10 (emulate Up/OK/Down/Left; any other code keeps the captured
//! button, and an invalid button falls back to the Key's, then Unlock, as in the C); counter + 1
//! (the emulate scene's `current_counter++`, masked to 16 bits); the 64-bit key rebuilt from the
//! button table (`honda_v1_build_key`); then `honda_v1_build_upload`: a 180-element short-pair
//! preamble whose last element is a 5000µs LOW + 4 PWM frames (2 per checksum wire value). The
//! Flipper TX loop sends the upload `repeat` = 1 time.
//!
//! **Key files** (`subghz_protocol_decoder_honda_v1_serialize` / `_deserialize`, the same in all
//! three references): Protocol "Honda V1", Bit 68, Key = the 64 data bits, then Serial, Btn, Cnt,
//! Crc and Key_2 (both the received CRC nibble, decimal) and `Disp: Honda V1 - <button>`. Loading
//! needs Bit 68; the fields come from the Key (the C ignores Serial/Btn/Cnt here), the nibble from
//! Key_2, else Crc, else the first wire-order checksum, and `crc_valid` is the C's check of it.
//! KAT does not keep the received nibble, only whether it matched, so it writes the first
//! wire-order checksum (the one the encoder sends first) for a CRC-valid frame and, for a frame
//! that failed, the lowest nibble that fails the check — the reload reports the same status.

use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Honda V1",
    decoder: "Honda V1",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "Honda/Acura",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// Honda V1 buttons.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x0 => "Unlock",
        0x8 => "Lock",
        0x9 => "Trunk",
        0xA => "Panic",
        _ => "Other",
    })
}

const BIT_COUNT: usize = 68;
const TE_SHORT: u32 = 1000;
const TE_LONG: u32 = 2000;
const TE_DELTA: u32 = 400;
const TE_SHORT_MIN: u32 = 600;
const TE_END: u32 = 3500;
const VALID_MAX: u8 = 0x4B; // honda_v1_add_bit cap (75)
const NIBBLE_MASK: u8 = 0x0F;
const SERIAL_MASK: u32 = 0x0FFF_FFFF;
const COUNTER_MASK: u16 = 0xFFFF;
const BUTTON_MAX: u8 = 10;
const BUTTON_VALID_MASK: u16 = 0x701; // bits set for Unlock(0), Lock(8), Trunk(9), Panic(10)
const DECODE_BUFFER_BYTES: usize = 12;

// Encoder constants (honda_v1.c, ENABLE_EMULATE_FEATURE).
const PREAMBLE_UPLOAD_COUNT: usize = 180;
const FRAME_SYMBOLS: usize = 80;
const FRAME_START: usize = 12;
const FRAME_SYNC_DROP: usize = 2;
const FRAME_REPEAT_PER_CRC: usize = 2;
const FRAME_GAP_US: u32 = 5000;
const FRAME_CRC_INDEX: usize = 8;

// HondaV1Button codes (the on-wire button nibble at data[31:28]).
const BTN_CODE_UNLOCK: u8 = 0;
const BTN_CODE_LOCK: u8 = 8;
const BTN_CODE_TRUNK: u8 = 9;
const BTN_CODE_PANIC: u8 = 10;

// honda_v1_button_codes[] (24-bit table values, used to rebuild the key in the encoder).
const BUTTON_CODE_UNLOCK: u32 = 0x0008_0808;
const BUTTON_CODE_LOCK: u32 = 0x0008_8888;
const BUTTON_CODE_TRUNK: u32 = 0x0009_9190;
const BUTTON_CODE_PANIC: u32 = 0x000F_A7A0;
const BUTTON_FALLBACK_CODE: u32 = 0x0008_8888;

// KAT generic button codes.
const BTN_LOCK: u8 = 0x01;
const BTN_UNLOCK: u8 = 0x02;
const BTN_TRUNK: u8 = 0x04;
const BTN_PANIC: u8 = 0x08;

/// Decoder steps (matches HondaV1DecoderStep).
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Preamble,
    Data,
}

/// Honda V1 decoder (matches SubGhzProtocolDecoderHondaV1).
pub struct HondaV1Decoder {
    step: DecoderStep,
    preamble_count: u8,
    preamble_has_long: bool,
    data_pending: bool,
    last_level: bool,
    bits: [u8; DECODE_BUFFER_BYTES],
    bit_count: u8,
    // Pending-bit timing accumulator (decoder-level, persists across symbol resets).
    pending: u32,
    pending_valid: bool,
}

impl HondaV1Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            preamble_count: 0,
            preamble_has_long: false,
            data_pending: false,
            last_level: false,
            bits: [0u8; DECODE_BUFFER_BYTES],
            bit_count: 0,
            pending: 0,
            pending_valid: false,
        }
    }

    /// honda_v1_button_valid: button <= 10 && ((0x701 >> button) & 1).
    fn button_valid(b: u8) -> bool {
        if b > BUTTON_MAX {
            return false;
        }
        ((BUTTON_VALID_MASK >> b) & 1) != 0
    }

    /// honda_v1_duration_is: |d - t| <= te_delta (saturating both directions).
    fn duration_is(d: u32, t: u32) -> bool {
        if d >= t {
            (d - t) <= TE_DELTA
        } else {
            (t - d) <= TE_DELTA
        }
    }

    /// honda_v1_crc_fold.
    fn crc_fold(v: u16) -> u8 {
        let lo = (v & (NIBBLE_MASK as u16)) as u8;
        let hi = v >> 4;
        let s: i32 = if (hi & 1) != 0 {
            lo as i32
        } else {
            -(lo as i32)
        };
        let mut out = ((s - (hi as i32)) & 7) as u8;
        out |= (((v >> 3) & 1) as u8) << 3;
        if ((v >> 1) & 1) != 0 && (((v >> 4) ^ (v >> 5)) & 1) != 0 {
            out ^= 0x04;
        }
        out & NIBBLE_MASK
    }

    /// honda_v1_checksum_base.
    fn checksum_base(data: u64) -> u8 {
        let a = Self::crc_fold((data & (COUNTER_MASK as u64)) as u16);
        let b = Self::crc_fold(((data >> 40) & 0xFF) as u16);
        (a ^ b ^ 1) & NIBBLE_MASK
    }

    /// honda_v1_checksum_alternate.
    fn checksum_alternate(checksum: u8) -> u8 {
        let mut mask = 0x09u8;
        if (checksum & 1) == 0 {
            mask = if (checksum & 2) != 0 { 0x0B } else { NIBBLE_MASK };
        }
        (checksum ^ mask) & NIBBLE_MASK
    }

    /// honda_v1_checksum_wire_order → (first, second).
    fn checksum_wire_order(data: u64) -> (u8, u8) {
        let checksum = Self::checksum_base(data);
        let other = Self::checksum_alternate(checksum);
        if (checksum & 0x08) != 0 {
            (other, checksum)
        } else {
            (checksum, other)
        }
    }

    /// honda_v1_crc_valid: received nibble matches either wire-order checksum.
    fn crc_valid(data: u64, crc: u8) -> bool {
        let (first, second) = Self::checksum_wire_order(data);
        let crc = crc & NIBBLE_MASK;
        crc == first || crc == second
    }

    /// honda_v1_decode_fields. Returns (serial, button, counter).
    fn decode_fields(data: u64) -> (u32, u8, u16) {
        let low = (data & 0xFFFF_FFFF) as u32;
        let serial = ((data >> 36) & (SERIAL_MASK as u64)) as u32;
        let button = ((low >> 28) & (NIBBLE_MASK as u32)) as u8;
        let counter = (low & (COUNTER_MASK as u32)) as u16;
        (serial, button, counter)
    }

    /// honda_v1_button_code (encoder).
    fn button_code(button: u8) -> u32 {
        if !Self::button_valid(button) {
            return BUTTON_FALLBACK_CODE;
        }
        match button {
            BTN_CODE_UNLOCK => BUTTON_CODE_UNLOCK,
            BTN_CODE_LOCK => BUTTON_CODE_LOCK,
            BTN_CODE_TRUNK => BUTTON_CODE_TRUNK,
            BTN_CODE_PANIC => BUTTON_CODE_PANIC,
            _ => BUTTON_FALLBACK_CODE,
        }
    }

    /// honda_v1_build_key.
    fn build_key(serial: u32, button: u8, counter: u16) -> u64 {
        let table = Self::button_code(button);
        let low = ((table & (COUNTER_MASK as u32)) << 16) | (counter as u32);
        let high = ((serial & SERIAL_MASK) << 4) | (table >> 16);
        ((high as u64) << 32) | (low as u64)
    }

    /// The Btn value the emulate scene writes for KAT's generic command, then the encoder's
    /// fallbacks: an invalid button takes the Key's, and if that is invalid too, Unlock.
    fn map_button(button: u8, captured: u8, key_button: u8) -> u8 {
        let b = match button {
            BTN_LOCK => BTN_CODE_LOCK,
            BTN_UNLOCK => BTN_CODE_UNLOCK,
            BTN_TRUNK => BTN_CODE_TRUNK,
            BTN_PANIC => BTN_CODE_PANIC,
            _ => captured & NIBBLE_MASK,
        };
        if Self::button_valid(b) {
            b
        } else if Self::button_valid(key_button) {
            key_button
        } else {
            BTN_CODE_UNLOCK
        }
    }

    /// honda_v1_state_reset (symbol-layer state only; does NOT touch pending accumulator).
    fn state_reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.preamble_count = 0;
        self.preamble_has_long = false;
        self.data_pending = false;
        self.last_level = false;
        self.bit_count = 0;
        self.bits = [0u8; DECODE_BUFFER_BYTES];
    }

    /// honda_v1_add_bit: MSB-first into bits[], capped at VALID_MAX.
    fn add_bit(&mut self, bit: bool) {
        if self.bit_count > VALID_MAX {
            return;
        }
        if bit {
            let byte = (self.bit_count >> 3) as usize;
            let shift = (!self.bit_count) & 0x07;
            self.bits[byte] |= 1u8 << shift;
        }
        self.bit_count += 1;
    }

    /// honda_v1_commit: align trailing 68-bit frame, validate button, return signal on success.
    fn commit(&mut self) -> Option<DecodedSignal> {
        if (self.bit_count as usize) < BIT_COUNT {
            return None;
        }

        let mut aligned = self.bits;

        let mut shift_count = self.bit_count - BIT_COUNT as u8;
        if shift_count < 1 {
            shift_count = 1;
        }

        for _ in 0..shift_count {
            for i in 0..(DECODE_BUFFER_BYTES - 1) {
                aligned[i] = (aligned[i] << 1) | (aligned[i + 1] >> 7);
            }
            aligned[DECODE_BUFFER_BYTES - 1] <<= 1;
        }

        let button = aligned[4] >> 4;
        if !Self::button_valid(button) {
            return None;
        }

        let data = u64::from_be_bytes(aligned[0..8].try_into().unwrap());
        let k2 = aligned[8] >> 4;
        Some(Self::signal(data, k2))
    }

    /// The decode of 64-bit key `data` with received CRC nibble `k2` (`honda_v1_decode_fields`
    /// + `honda_v1_crc_valid`).
    fn signal(data: u64, k2: u8) -> DecodedSignal {
        let (serial, btn, counter) = Self::decode_fields(data);
        DecodedSignal {
            serial: Some(serial),
            button: Some(btn),
            counter: Some(counter),
            crc_valid: Self::crc_valid(data, k2),
            data,
            data_count_bit: BIT_COUNT,
            encoder_capable: true,
            extra: None,
            protocol_display_name: None,
        }
    }

    /// honda_v1_symbol: classify a (level, duration) pulse and drive the state machine.
    /// Returns Some(signal) when the end-gap path commits a frame.
    fn symbol(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let sh = Self::duration_is(duration, TE_SHORT);
        let lg = Self::duration_is(duration, TE_LONG);

        if !sh && !lg {
            let mut result = None;
            if !level && duration > TE_END && self.step == DecoderStep::Data {
                result = self.commit();
            }
            self.state_reset();
            return result;
        }

        if self.step == DecoderStep::Reset {
            if level {
                self.step = DecoderStep::Preamble;
                self.preamble_count = 1;
                self.last_level = level;
            }
            return None;
        }

        if self.step == DecoderStep::Preamble {
            if lg {
                // honda_v1.c: if(preamble_count < 0xFF) preamble_count++ (saturating).
                self.preamble_count = self.preamble_count.saturating_add(1);
                self.preamble_has_long = true;
                self.last_level = level;
                return None;
            }

            if sh {
                if self.preamble_has_long && self.preamble_count > 5 {
                    self.step = DecoderStep::Data;
                    self.bit_count = 0;
                    self.bits = [0u8; DECODE_BUFFER_BYTES];
                    self.data_pending = true;
                    self.last_level = level;
                    return None;
                }

                self.preamble_count = self.preamble_count.saturating_add(1);
                self.last_level = level;
                return None;
            }

            self.state_reset();
            return None;
        }

        // Data step: pending-bit accumulation.
        if sh {
            if self.data_pending {
                self.add_bit(level);
                self.data_pending = false;
                self.last_level = level;
            } else {
                self.data_pending = true;
                self.last_level = level;
            }
        } else {
            // long pulse
            if self.data_pending {
                self.add_bit(level);
            } else {
                self.add_bit(self.last_level);
            }
            self.last_level = level;
        }

        None
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

    /// honda_v1_append_frame: emit one PWM frame (12-symbol sync header + 68 data symbols),
    /// dropping the first FRAME_SYNC_DROP entries, then a tail + inter-frame gap.
    fn append_frame(signal: &mut Vec<LevelDuration>, frame: &[u8; 9]) {
        // Build the per-frame symbol stream with merge semantics (pp_emit_merge).
        let mut generated: Vec<LevelDuration> = Vec::with_capacity(FRAME_SYMBOLS * 2);
        for bit_index in 0..FRAME_SYMBOLS {
            let bit = if bit_index >= FRAME_START {
                let data_index = (bit_index - FRAME_START) >> 3;
                let shift = (11i32 - bit_index as i32) & 0x07;
                ((frame[data_index] >> shift) & 0x01) != 0
            } else {
                ((!bit_index) & 0x01) != 0
            };
            // bit -> (level=bit, te)(level=!bit, te), merged.
            Self::enc_add_level(&mut generated, bit, TE_SHORT);
            Self::enc_add_level(&mut generated, !bit, TE_SHORT);
        }

        if generated.len() <= FRAME_SYNC_DROP {
            return;
        }

        // Copy generated[FRAME_SYNC_DROP..] into the upload (still merging at the seam).
        for ld in &generated[FRAME_SYNC_DROP..] {
            Self::enc_add_level(signal, ld.level, ld.duration_us);
        }

        // Tail: !last_level for te_short; if that was low, add a high te_short; then the gap.
        let last_level = signal.last().map(|l| l.level).unwrap_or(false);
        let tail_level = !last_level;
        Self::enc_add_level(signal, tail_level, TE_SHORT);
        if !tail_level {
            Self::enc_add_level(signal, true, TE_SHORT);
        }
        Self::enc_add_level(signal, false, FRAME_GAP_US);
    }
}

impl ProtocolDecoder for HondaV1Decoder {
    fn name(&self) -> &'static str {
        "Honda V1"
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
        self.pending = 0;
        self.pending_valid = false;
        self.state_reset();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        // Pending-bit timing accumulation (subghz_protocol_decoder_honda_v1_feed).
        if duration < TE_DELTA {
            self.pending = self.pending.saturating_add(duration);
            self.pending_valid = true;
            return None;
        }

        let mut result = None;

        if self.pending_valid {
            let p = self.pending;
            if level {
                self.pending = p.saturating_add(duration);
                self.pending_valid = true;
                return None;
            }
            if p >= TE_SHORT_MIN {
                result = self.symbol(true, p);
            }
            self.pending = 0;
            self.pending_valid = false;
        }

        if level {
            self.pending = duration;
            self.pending_valid = true;
            return result;
        }

        // A symbol committed on the flushed HIGH pulse takes priority (the C calls back there);
        // otherwise classify this LOW pulse.
        let low_result = self.symbol(false, duration);
        result.or(low_result)
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        // subghz_block_generic_deserialize_check_count_bit(.., 68) + honda_v1_decode_fields(Key),
        // then the file's Serial/Btn/Cnt (Cnt = counter + 1 from the emulate scene).
        if decoded.data_count_bit != BIT_COUNT {
            return None;
        }
        let (key_serial, key_button, key_counter) = Self::decode_fields(decoded.data);
        let serial = decoded.serial.unwrap_or(key_serial) & SERIAL_MASK;
        let btn = Self::map_button(button, decoded.button.unwrap_or(key_button), key_button);
        let counter = decoded.counter.unwrap_or(key_counter).wrapping_add(1) & COUNTER_MASK;
        let data = Self::build_key(serial, btn, counter);

        let mut frame = [0u8; 9];
        frame[0..8].copy_from_slice(&data.to_be_bytes());
        let (first, second) = Self::checksum_wire_order(data);

        let mut signal: Vec<LevelDuration> = Vec::with_capacity(PREAMBLE_UPLOAD_COUNT + 8 * FRAME_SYMBOLS);

        // Preamble: 180 short entries (90 H/L pairs); the final LOW becomes a 5000µs gap.
        for _ in 0..(PREAMBLE_UPLOAD_COUNT / 2) {
            signal.push(LevelDuration::new(true, TE_SHORT));
            signal.push(LevelDuration::new(false, TE_SHORT));
        }
        if let Some(last) = signal.last_mut() {
            *last = LevelDuration::new(false, FRAME_GAP_US);
        }

        // 4 frames: 2 per checksum wire value (first, then second).
        for &crc in &[first, second] {
            frame[FRAME_CRC_INDEX] = crc << 4;
            for _ in 0..FRAME_REPEAT_PER_CRC {
                Self::append_frame(&mut signal, &frame);
            }
        }

        Some(signal)
    }

    /// `subghz_protocol_decoder_honda_v1_deserialize` (see the module doc, Key files).
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != BIT_COUNT {
            return None;
        }
        let data = key.key_u64()?;
        let k2 = match key.u32_field("Key_2").or_else(|| key.u32_field("Crc")) {
            Some(crc) => crc as u8 & NIBBLE_MASK,
            None => Self::checksum_wire_order(data).0,
        };
        Some(Self::signal(data, k2))
    }

    /// `subghz_protocol_decoder_honda_v1_serialize` (see the module doc, Key files).
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != BIT_COUNT {
            return None;
        }
        let data = decoded.data;
        let (serial, btn, counter) = Self::decode_fields(data);
        let crc = if decoded.crc_valid {
            Self::checksum_wire_order(data).0
        } else {
            (0..=NIBBLE_MASK).find(|&n| !Self::crc_valid(data, n))?
        };
        let mut key = FlipperKey::for_protocol("Honda V1", BIT_COUNT, data);
        key.push_u32("Serial", serial);
        key.push_u32("Btn", btn.into());
        key.push_u32("Cnt", counter.into());
        key.push_u32("Crc", crc.into());
        key.push_u32("Key_2", crc.into());
        let name = match btn {
            BTN_CODE_UNLOCK => "Unlock",
            BTN_CODE_LOCK => "Lock",
            BTN_CODE_TRUNK => "Trunk",
            BTN_CODE_PANIC => "Panic",
            _ => "Unknown",
        };
        key.push("Disp", format!("Honda V1 - {name}"));
        Some(key)
    }
}

impl Default for HondaV1Decoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// build_key + decode_fields must round-trip the fields, and the encoder's CRC nibble must
    /// validate against honda_v1_crc_valid.
    #[test]
    fn honda_v1_key_field_roundtrip() {
        let serial = 0x0ABCDEFu32 & SERIAL_MASK;
        let counter = 0x1234u16;
        let data = HondaV1Decoder::build_key(serial, BTN_CODE_TRUNK, counter);
        let (s, b, c) = HondaV1Decoder::decode_fields(data);
        assert_eq!(s, serial, "serial mismatch");
        assert_eq!(b, BTN_CODE_TRUNK, "button mismatch");
        assert_eq!(c, counter, "counter mismatch");

        let (first, _second) = HondaV1Decoder::checksum_wire_order(data);
        assert!(
            HondaV1Decoder::crc_valid(data, first),
            "wire-order checksum should validate"
        );
    }

    /// Full encode → decode round-trip as ProtoPirate transmits (encoder deserialize + emulate
    /// scene): feed the pulses back through the decoder (pending-bit accumulator + symbol state
    /// machine) and check the requested button, the counter + 1 and a valid checksum.
    #[test]
    fn honda_v1_encode_decode_roundtrip() {
        let serial = 0x0123456u32 & SERIAL_MASK;
        let data = HondaV1Decoder::build_key(serial, BTN_CODE_TRUNK, 0xFFFF);
        let (s, b, c) = HondaV1Decoder::decode_fields(data);
        let original = DecodedSignal {
            serial: Some(s),
            button: Some(b),
            counter: Some(c),
            crc_valid: true,
            data,
            data_count_bit: BIT_COUNT,
            encoder_capable: true,
            extra: None,
            protocol_display_name: None,
        };
        for (button, code) in [
            (BTN_LOCK, BTN_CODE_LOCK),
            (BTN_UNLOCK, BTN_CODE_UNLOCK),
            (BTN_TRUNK, BTN_CODE_TRUNK),
            (BTN_PANIC, BTN_CODE_PANIC),
            (0x40, BTN_CODE_TRUNK),
        ] {
            let signal = HondaV1Decoder::new().encode(&original, button).expect("encode");
            let mut dec = HondaV1Decoder::new();
            let frames: Vec<_> = signal
                .iter()
                .filter_map(|ld| dec.feed(ld.level, ld.duration_us))
                .collect();
            assert_eq!(frames.len(), 4, "2 frames per checksum wire value");
            for f in &frames {
                assert!(f.crc_valid, "checksum should validate");
                assert_eq!(f.serial, Some(serial));
                assert_eq!(f.button, Some(code), "button {button:#04x}");
                assert_eq!(f.counter, Some(0x0000), "0xFFFF + 1 wraps in 16 bits");
                assert_eq!(f.data_count_bit, BIT_COUNT);
            }
        }
    }

    /// Key files: ProtoPirate's lines round-trip for a CRC-valid and a CRC-failed frame; a
    /// reference file loads (Key_2 first, then Crc, then the computed checksum); a loaded key
    /// transmits a frame that decodes back.
    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::{assert_round_trip, key};
        let dec = HondaV1Decoder::new();
        let data = HondaV1Decoder::build_key(0x0123456, BTN_CODE_TRUNK, 0x1233);
        let src = HondaV1Decoder::signal(data, HondaV1Decoder::checksum_wire_order(data).0);
        let decoded = {
            let tx = dec.encode(&src, 0x40).unwrap();
            let mut rx = HondaV1Decoder::new();
            tx.iter().find_map(|ld| rx.feed(ld.level, ld.duration_us)).expect("decodes")
        };
        assert!(decoded.crc_valid);
        let text = assert_round_trip(&dec, &decoded);
        let crc = HondaV1Decoder::checksum_wire_order(decoded.data).0;
        let want = format!(
            "Protocol: Honda V1\nBit: 68\nKey: {}\nSerial: 1193046\nBtn: 9\nCnt: 4660\n\
             Crc: {crc}\nKey_2: {crc}\nDisp: Honda V1 - Trunk\n",
            decoded.data.to_be_bytes().map(|b| format!("{b:02X}")).join(" ")
        );
        assert!(text.ends_with(&want), "{text}");
        let failed = HondaV1Decoder::signal(decoded.data, (0..16).find(|&n| !HondaV1Decoder::crc_valid(decoded.data, n)).unwrap());
        assert!(!failed.crc_valid);
        assert_round_trip(&dec, &failed);

        // A reference-written file: Key_2 wins over Crc; without either the checksum is computed.
        let mut file = key(&format!(
            "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
             Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Honda V1\nBit: 68\nKey: {}\n\
             Serial: 1\nBtn: 0\nCnt: 0\nCrc: {}\nKey_2: {crc}\n",
            decoded.data.to_be_bytes().map(|b| format!("{b:02X}")).join(" "),
            (crc + 1) & 0x0F
        ));
        assert_eq!(dec.from_flipper_key(&file), Some(decoded.clone()), "fields from the Key, nibble from Key_2");
        file.fields.retain(|(n, _)| n != "Key_2" && n != "Crc");
        assert_eq!(dec.from_flipper_key(&file), Some(decoded.clone()));
        file.bits = 64;
        assert!(dec.from_flipper_key(&file).is_none());

        let tx = dec.encode(&decoded, 0x01).unwrap();
        let mut rx = HondaV1Decoder::new();
        let again = tx.iter().find_map(|ld| rx.feed(ld.level, ld.duration_us)).unwrap();
        assert_eq!((again.serial, again.button, again.counter), (Some(0x0123456), Some(BTN_CODE_LOCK), Some(0x1235)));
    }

    /// Oracle vectors (see `fiat_v0::write_oracle_vector`): Trunk and Lock frames.
    #[test]
    #[ignore]
    fn oracle_vectors() {
        for (name, button, counter) in [("honda_v1_1", BTN_CODE_TRUNK, 0x1233u16), ("honda_v1_2", BTN_CODE_LOCK, 0x00FF)] {
            let data = HondaV1Decoder::build_key(0x0123456, button, counter);
            let src = HondaV1Decoder::signal(data, HondaV1Decoder::checksum_wire_order(data).0);
            let up = HondaV1Decoder::new().encode(&src, 0x40).unwrap();
            crate::protocols::fiat_v0::write_oracle_vector(
                name,
                433_920_000,
                "FuriHalSubGhzPresetOok650Async",
                &up,
                &mut HondaV1Decoder::new(),
            );
        }
    }

    /// Button validity mask must match honda_v1_button_valid (0x701 → Unlock/Lock/Trunk/Panic).
    #[test]
    fn honda_v1_button_validity() {
        assert!(HondaV1Decoder::button_valid(BTN_CODE_UNLOCK));
        assert!(HondaV1Decoder::button_valid(BTN_CODE_LOCK));
        assert!(HondaV1Decoder::button_valid(BTN_CODE_TRUNK));
        assert!(HondaV1Decoder::button_valid(BTN_CODE_PANIC));
        // Invalid codes.
        assert!(!HondaV1Decoder::button_valid(1));
        assert!(!HondaV1Decoder::button_valid(7));
        assert!(!HondaV1Decoder::button_valid(11));
    }
}
