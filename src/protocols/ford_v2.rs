//! Ford V2 protocol decoder/encoder
//!
//! Decoder: port of ProtoPirate `REFERENCES/ProtoPirate/protocols/ford_v2.c` as of ProtoPirate
//! HEAD (dc3446b; the decode logic is unchanged since KAT's 4c95c85 port — later commits only
//! touched allocation/repeat plumbing). Flipper-ARF's `lib/subghz/protocols/ford_v2.c` has the
//! same decoder. Encoder: Flipper-ARF's, a superset of ProtoPirate's (see Encoder).
//!
//! Manchester 200/400µs (te_delta 260 → threshold ~460µs between short/long), 104 bits (13
//! bytes), FM, 315 + 433.92 MHz. Frame begins with a 16-bit Manchester sync that equals ~0x7FA7
//! (the decoder matches the *inverted* shift register against 0x8058); the two sync bytes 0x7F
//! 0xA7 head the 13-byte buffer. Data bits are inverted before packing (`data_bit = !data_bit`).
//! Fields: serial = raw[2..6], button = raw[6] (0x10 Lock, 0x11 Unlock, 0x13 Trunk, 0x14 Panic,
//! 0x15 Remote start), raw[7] bit 7 = parity of the button byte, counter (16 bits) =
//! raw[7] bits 6..0 ‖ raw[8] ‖ raw[9] bit 7, tail = the remaining 31 bits. The frame is plaintext.
//! The C's structure check (`ford_v2_decoder_extract_from_raw`) is the two sync bytes plus a known
//! button (its counter re-checks are tautologies), so that is the emission gate.
//!
//! Decoder steps: Reset → Preamble (≥64 shorts) → Sync (find 0x7FA7) → Data (11 bytes).
//!
//! **Manchester**: the shared Flipper `manchester_advance` port in `common.rs`. Event mapping is
//! `level ? ShortHigh : ShortLow` / `level ? LongHigh : LongLow`. Entering Sync from the preamble
//! resets the state to Mid1 and forces Mid0 when the triggering (long LOW) event is a Low event,
//! then feeds that event; `reset_state` resets to Mid1. The encoder's 1 = HIGH,LOW decodes as 0
//! under this mapping, which is why the decoder inverts data bits and matches the sync against
//! `!0x7FA7`.
//!
//! **DecodedSignal**: `data` = raw[0..8] (the C's `Key`), `extra` = raw[8..13] (`TailRaw`, 40
//! bits), `serial`/`button`/`counter` as above, `data_count_bit` = 104.
//!
//! **Encoder**: ProtoPirate's is a plain replay (it rebuilds the frame from Key + TailRaw, re-patches
//! the button parity bit and ignores the emulate scene's Btn/Cnt). Flipper-ARF's
//! (`ford_v2_encoder_deserialize_validate_and_pack`) also applies the file's Serial, re-packs
//! Cnt + 1 (16 bits; `cnt + rolling_counter_mult`) and maps the D-pad to a button with the parity
//! re-patched, so KAT follows ARF: serial ← `DecodedSignal.serial`, counter ← counter + 1, button
//! ← KAT's 0x01 Lock → 0x10, 0x02 Unlock → 0x11, 0x04 Trunk → 0x13, 0x08 Panic → 0x14 (ARF's
//! Up/Down/Left map; any other code keeps the captured button, as ARF's OK). An invalid
//! resulting button is refused, as in the C. Upload (`ford_v2_encoder_build_upload`): 6 bursts of
//! 70 × (LOW 240, HIGH 240), LOW 476, HIGH 240, then frame bits 1..103 as (bit, 240µs)(!bit,
//! 240µs), bursts separated by HIGH 16 000µs (merged into the adjacent HIGH half-bits, as the C's
//! `ford_v2_encoder_add_level` merges every same-level element). The Flipper TX loop repeats the
//! upload `repeat` = 10 times; KAT's `encode()` returns the single upload.
//!
//! **Key files** (`subghz_protocol_decoder_ford_v2_serialize` / `_deserialize`; Flipper-ARF's are
//! the same): `Protocol: Ford V2`, `Bit: 104`, `Key` = raw[0..8], then `Serial`, `Btn` (raw[6]),
//! `Cnt` (the 16-bit counter), `Tail31` (decimal) and `TailRaw` = raw[8..13]. Loading needs Bit 104
//! and rebuilds the frame from Key + TailRaw (all-zero when the line is missing, as in the C); the
//! fields come from the frame (the C's decoder ignores the file's Serial/Btn/Cnt) and a frame that
//! fails the C's structure check (sync bytes + known button) is rejected, as the C rejects it.

use super::common::{ManchesterEvent, ManchesterState};
use super::{ProtocolDecoder, ProtocolTiming, DecodedSignal, FlipperKey};
use crate::radio::demodulator::LevelDuration;
use crate::duration_diff;

use super::info::prelude::*;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Ford V2",
    decoder: "Ford V2",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::FM,
    encryption: "Rolling Code",
    make: "Ford",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&F4],
};

/// Ford V2 buttons.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x10 => "Lock",
        0x11 => "Unlock",
        0x13 => "Trunk",
        0x14 => "Panic",
        0x15 => "Remote start",
        _ => "Other",
    })
}

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
        self.manchester_state.advance(ManchesterEvent::Reset);
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

    /// Map (level, duration) → Manchester event. Ford V2 polarity: level ? High : Low.
    fn pulse_event(level: bool, duration: u32) -> Option<ManchesterEvent> {
        if Self::is_short(duration) {
            Some(ManchesterEvent::from_half_bit(level, false))
        } else if Self::is_long(duration) {
            Some(ManchesterEvent::from_half_bit(level, true))
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
        self.manchester_state.advance(ManchesterEvent::Reset);

        if let Some(event) = Self::pulse_event(level, duration) {
            // A Low event seeds Mid0 (the C `if(ev == ShortLow || ev == LongLow) state = Mid0`).
            if matches!(event, ManchesterEvent::ShortLow | ManchesterEvent::LongLow) {
                self.manchester_state = ManchesterState::Mid0;
            }
            self.feed_event(event);
        } else {
            self.reset_state();
        }
    }

    /// Process one Manchester event in Sync or Data step. Returns Some when a frame commits.
    fn feed_event(&mut self, event: ManchesterEvent) -> Option<DecodedSignal> {
        if self.step == DecoderStep::Sync {
            if let Some(bit) = self.manchester_state.advance(event) {
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
        if let Some(bit) = self.manchester_state.advance(event) {
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
        Self::frame_signal(&self.raw_bytes)
    }

    /// The decode of a 13-byte frame, or None when it fails the structure check (sync bytes +
    /// known button, `ford_v2_decoder_extract_from_raw`).
    fn frame_signal(k: &[u8; DATA_BYTES]) -> Option<DecodedSignal> {
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

    /// Map KAT's generic button command to a Ford V2 button code (ARF's D-pad map; any other
    /// code keeps the captured button, like ARF's OK).
    fn map_button(button: u8, captured: u8) -> u8 {
        match button {
            0x01 => 0x10, // Lock
            0x02 => 0x11, // Unlock
            0x04 => 0x13, // Trunk
            0x08 => 0x14, // Panic
            _ => captured,
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
        // ford_v2_encoder_rebuild_raw_from_payload: bytes 0..8 from Key, 8..13 from TailRaw.
        let mut raw = [0u8; DATA_BYTES];
        raw[..8].copy_from_slice(&decoded.data.to_be_bytes());
        let extra = decoded.extra.unwrap_or(0);
        raw[8..].copy_from_slice(&extra.to_be_bytes()[3..]);

        // ARF overrides: Serial, then Cnt + 1 re-packed (raw[7] bit 7 and raw[9] bits 6..0 kept).
        if let Some(serial) = decoded.serial {
            raw[2..6].copy_from_slice(&serial.to_be_bytes());
        }
        if let Some(cnt) = decoded.counter {
            let cnt = cnt.wrapping_add(1);
            raw[7] = (raw[7] & 0x80) | ((cnt >> 9) & 0x7F) as u8;
            raw[8] = (cnt >> 1) as u8;
            raw[9] = (raw[9] & 0x7F) | (((cnt & 1) as u8) << 7);
        }
        // Button (captured = the Btn field, else the packed byte) with the parity bit re-patched.
        raw[6] = Self::map_button(button, decoded.button.unwrap_or(raw[6]));
        raw[7] = (raw[7] & 0x7F) | (Self::parity8(raw[6]) << 7);
        if !Self::button_is_valid(raw[6]) {
            return None;
        }

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

    /// `subghz_protocol_decoder_ford_v2_deserialize` (see the module doc, "Key files").
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != DATA_BITS {
            return None;
        }
        let mut raw = [0u8; DATA_BYTES];
        raw[..8].copy_from_slice(&key.key_u64()?.to_be_bytes());
        if let Some(tail) = key.hex_bytes_field("TailRaw").filter(|t| t.len() == TAIL_RAW_BYTES) {
            raw[8..].copy_from_slice(&tail);
        }
        Self::frame_signal(&raw)
    }

    /// `subghz_protocol_decoder_ford_v2_serialize` (see the module doc, "Key files").
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        let extra = decoded.extra.unwrap_or(0);
        if decoded.data_count_bit != DATA_BITS || extra >> (TAIL_RAW_BYTES * 8) != 0 {
            return None;
        }
        let mut raw = [0u8; DATA_BYTES];
        raw[..8].copy_from_slice(&decoded.data.to_be_bytes());
        raw[8..].copy_from_slice(&extra.to_be_bytes()[3..]);
        let d = Self::frame_signal(&raw)?;
        let tail31 = u32::from_be_bytes([raw[9] & 0x7F, raw[10], raw[11], raw[12]]);
        let mut key = FlipperKey::for_protocol("Ford V2", DATA_BITS, decoded.data);
        key.push_u32("Serial", d.serial?);
        key.push_u32("Btn", raw[6].into());
        key.push_u32("Cnt", d.counter?.into());
        key.push_u32("Tail31", tail31);
        key.push_hex_bytes("TailRaw", &raw[8..]);
        Some(key)
    }
}

impl Default for FordV2Decoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 13-byte frame: sync, serial, button, counter (bytes 7..9) and a tail.
    fn frame_bytes(button: u8, counter: u16) -> [u8; DATA_BYTES] {
        let mut raw = [
            SYNC_0, SYNC_1, 0x12, 0x34, 0x56, 0x78, button, 0x00, 0x00, 0x00, 0x9A, 0xBC, 0xDE,
        ];
        raw[7] = ((counter >> 9) & 0x7F) as u8 | (FordV2Decoder::parity8(button) << 7);
        raw[8] = (counter >> 1) as u8;
        raw[9] = (((counter & 1) as u8) << 7) | 0x35;
        raw
    }

    fn signal_for(raw: &[u8; DATA_BYTES]) -> DecodedSignal {
        let mut data = 0u64;
        for &b in raw.iter().take(8) {
            data = (data << 8) | b as u64;
        }
        let mut extra = 0u64;
        for &b in raw.iter().skip(8) {
            extra = (extra << 8) | b as u64;
        }
        let counter = (((raw[7] & 0x7F) as u16) << 9) | ((raw[8] as u16) << 1) | ((raw[9] >> 7) as u16);
        DecodedSignal {
            serial: Some(0x1234_5678),
            button: Some(raw[6]),
            counter: Some(counter),
            crc_valid: true,
            data,
            data_count_bit: DATA_BITS,
            encoder_capable: true,
            extra: Some(extra),
            protocol_display_name: None,
        }
    }

    /// One encoder burst (70-pair preamble, 476µs sync LOW, short HIGH, bits 1..103 as
    /// (bit, !bit) at 240µs) for an arbitrary frame, followed by a long HIGH gap.
    fn burst_wave(raw: &[u8; DATA_BYTES]) -> Vec<LevelDuration> {
        let mut wave = Vec::new();
        for _ in 0..ENC_PREAMBLE_PAIRS {
            FordV2Decoder::enc_add_level(&mut wave, false, ENC_TE_SHORT);
            FordV2Decoder::enc_add_level(&mut wave, true, ENC_TE_SHORT);
        }
        FordV2Decoder::enc_add_level(&mut wave, false, ENC_SYNC_LO_US);
        FordV2Decoder::enc_add_level(&mut wave, true, ENC_TE_SHORT);
        for bit_pos in 1..DATA_BITS {
            let bit = (raw[bit_pos / 8] >> (7 - bit_pos % 8)) & 1 != 0;
            FordV2Decoder::enc_manchester_bit(&mut wave, bit);
        }
        FordV2Decoder::enc_add_level(&mut wave, true, ENC_INTER_BURST_GAP_US);
        wave
    }

    fn feed_all(wave: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = FordV2Decoder::new();
        wave.iter().filter_map(|p| dec.feed(p.level, p.duration_us)).collect()
    }

    /// Encoder → decoder through the full 6-burst waveform (ARF encoder): every burst decodes to
    /// the requested button (parity re-patched) with the counter advanced by one (16 bits, carry
    /// across the raw[7]/raw[8]/raw[9] split) and the serial and tail bits kept.
    #[test]
    fn encode_decode_roundtrip() {
        for counter in [0x2468u16, 0x01FF, 0xFFFF] {
            let original = signal_for(&frame_bytes(0x11, counter));
            for (button, code) in [(0x01u8, 0x10u8), (0x02, 0x11), (0x04, 0x13), (0x08, 0x14), (0x40, 0x11)] {
                let upload = FordV2Decoder::new().encode(&original, button).unwrap();
                let frames = feed_all(&upload);
                assert_eq!(frames.len(), ENC_BURST_COUNT, "button {button:#04x}");
                let expected = signal_for(&frame_bytes(code, counter.wrapping_add(1)));
                for f in &frames {
                    assert_eq!(f.serial, Some(0x1234_5678));
                    assert_eq!(f.button, Some(code));
                    assert_eq!(f.counter, Some(counter.wrapping_add(1)));
                    assert_eq!(f.data, expected.data, "bytes 0..8 (incl. refreshed parity)");
                    assert_eq!(f.extra, expected.extra, "tail bytes 8..13");
                }
            }
        }
        // A captured Remote start (0x15) is kept for a non-generic command; an invalid captured
        // button is refused, as in the C.
        let rs = signal_for(&frame_bytes(0x15, 7));
        let frames = feed_all(&FordV2Decoder::new().encode(&rs, 0x40).unwrap());
        assert_eq!(frames[0].button, Some(0x15));
        let mut bad = signal_for(&frame_bytes(0x11, 7));
        bad.button = Some(0x12);
        assert!(FordV2Decoder::new().encode(&bad, 0x40).is_none());
    }

    /// Structure gate: a valid burst decodes; a wrong sync byte or an unknown button does not.
    #[test]
    fn rejects_bad_sync_or_button() {
        let good = frame_bytes(0x10, 0x2468);
        assert_eq!(feed_all(&burst_wave(&good)).len(), 1, "control burst decodes");

        let mut bad_sync = good;
        bad_sync[1] = 0xA6;
        assert!(feed_all(&burst_wave(&bad_sync)).is_empty(), "sync 0x7FA6");

        let bad_button = frame_bytes(0x12, 0x2468);
        assert!(feed_all(&burst_wave(&bad_button)).is_empty(), "button 0x12");
    }

    /// Key files (ProtoPirate `subghz_protocol_decoder_ford_v2_serialize` / `_deserialize`): a
    /// decoded burst round-trips with the C's lines, the C's structure check rejects a bad
    /// frame, a file without TailRaw loads with a zero tail, and TX from a loaded key decodes back.
    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::{assert_round_trip, key};
        let dec = FordV2Decoder::new();
        let d = feed_all(&burst_wave(&frame_bytes(0x11, 0x2468))).pop().expect("decodes");
        let text = assert_round_trip(&dec, &d);
        // raw = 7F A7 12 34 56 78 11 12 34 35 9A BC DE; Tail31 = 0x359ABCDE.
        assert!(
            text.ends_with(
                "Protocol: Ford V2\nBit: 104\nKey: 7F A7 12 34 56 78 11 12\nSerial: 305419896\nBtn: 17\n\
                 Cnt: 9320\nTail31: 899333342\nTailRaw: 34 35 9A BC DE\n"
            ),
            "{text}"
        );
        let file = key(&text);
        let (name, loaded) = crate::protocols::ProtocolRegistry::new().decode_key(&file).unwrap();
        assert_eq!((name.as_str(), &loaded), ("Ford V2", &d));

        // Missing TailRaw: the C keeps a zero tail (counter LSB and tail bits 0).
        let mut no_tail = file.clone();
        no_tail.fields.retain(|(n, _)| n != "TailRaw");
        let t = dec.from_flipper_key(&no_tail).unwrap();
        assert_eq!((t.serial, t.button, t.extra), (d.serial, d.button, Some(0)));
        // A frame that fails the structure check (unknown button 0x12) is rejected; so is Bit 64.
        let mut bad = file.clone();
        bad.key = (bad.key & !0xFF00) | 0x1200;
        bad.key_bytes = bad.key.to_be_bytes().to_vec();
        assert!(dec.from_flipper_key(&bad).is_none());
        let mut wrong = file.clone();
        wrong.bits = 64;
        assert!(dec.from_flipper_key(&wrong).is_none());

        // TX from the loaded key: Lock with the counter bumped.
        let next = feed_all(&dec.encode(&loaded, 0x01).unwrap()).remove(0);
        assert_eq!((next.serial, next.button, next.counter), (d.serial, Some(0x10), Some(0x2469)));
    }

    /// Reference-oracle vectors: `KAT_KEY_VECTORS=<dir> cargo test oracle_vectors -- --ignored`.
    #[test]
    #[ignore]
    fn oracle_vectors() {
        let dec = FordV2Decoder::new();
        for (n, (button, counter)) in [(0x11u8, 0x2468u16), (0x14, 0x01FF)].into_iter().enumerate() {
            let d = feed_all(&burst_wave(&frame_bytes(button, counter))).pop().unwrap();
            let upload = dec.encode(&d, 0x40).unwrap();
            let sent = feed_all(&upload).remove(0);
            let mut key = dec.to_flipper_key(&sent).unwrap();
            key.frequency = 433_920_000;
            key.set_preset(&crate::radio::preset::RfPreset::from_def(INFO.presets[0]));
            super::super::ford_v0::write_oracle_vector(&format!("ford_v2_{}", n + 1), &upload, &key);
        }
    }
}
