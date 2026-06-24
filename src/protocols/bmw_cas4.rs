//! BMW CAS4 protocol decoder
//!
//! Aligned with Flipper-ARF reference: `lib/subghz/protocols/bmw_cas4.c` and `bmw_cas4.h`.
//! Manchester 500/1000µs (te_delta 150), 64 bits (8 bytes), AM/OOK. The CAS4 rolling cipher's
//! manufacturer key is not available, so the encrypted portion is left as-is — the frame is only
//! framed and validated, not decrypted. Emission is gated on two fixed marker bytes
//! (byte[0]==0x30 && byte[6]==0xC5), which makes the protocol specific and prevents false matches.
//! Decode-only: the reference encoder is a non-functional stub (`yield` returns reset,
//! `deserialize` returns error), so `supports_encoding()` is false and `encode()` is None.
//!
//! Decoder steps: Reset → Preamble (≥10 pulses of 300-700µs) → Data. The preamble→data transition
//! is triggered by a long low gap (≥1800µs). Manchester polarity is `level ? Low : High` (Flipper
//! manchester_decoder.h event order: 0=ShortLow, 1=ShortHigh, 2=LongLow, 3=LongHigh), the same
//! mapping as Ford V0 / common.

use super::{ProtocolDecoder, ProtocolTiming, DecodedSignal};
use crate::radio::demodulator::LevelDuration;
use crate::duration_diff;

const TE_SHORT: u32 = 500;
const TE_LONG: u32 = 1000;
const TE_DELTA: u32 = 150;
const DATA_BITS: usize = 64;
const DATA_BYTES: usize = 8;

// Preamble pulse window and minimum count (BMW_CAS4_PREAMBLE_PULSE_MIN/MAX, BMW_CAS4_PREAMBLE_MIN).
const PREAMBLE_PULSE_MIN: u32 = 300;
const PREAMBLE_PULSE_MAX: u32 = 700;
const PREAMBLE_MIN: u16 = 10;
// Long low gap that separates the preamble from the data burst (BMW_CAS4_GAP_MIN).
const GAP_MIN: u32 = 1800;

// Fixed validation markers (BMW_CAS4_BYTE0_MARKER, BMW_CAS4_BYTE6_MARKER).
const BYTE0_MARKER: u8 = 0x30;
const BYTE6_MARKER: u8 = 0xC5;

/// Manchester state machine (Flipper manchester_decoder.h transition table).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum ManchesterState {
    Mid0 = 0,
    Mid1 = 1,
    Start0 = 2,
    Start1 = 3,
}

/// Decoder step states (matches BmwCas4DecoderStep in bmw_cas4.c).
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Preamble,
    Data,
}

/// BMW CAS4 protocol decoder (matches SubGhzProtocolDecoderBmwCas4).
pub struct BmwCas4Decoder {
    step: DecoderStep,
    manchester_state: ManchesterState,
    preamble_count: u16,
    raw_data: [u8; DATA_BYTES],
    bit_count: usize,
    decode_data: u64,
}

impl BmwCas4Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            manchester_state: ManchesterState::Mid1,
            preamble_count: 0,
            raw_data: [0; DATA_BYTES],
            bit_count: 0,
            decode_data: 0,
        }
    }

    /// Reset accumulators (matches subghz_protocol_decoder_bmw_cas4_reset).
    fn reset_state(&mut self) {
        self.step = DecoderStep::Reset;
        self.manchester_state = ManchesterState::Mid1;
        self.preamble_count = 0;
        self.raw_data = [0; DATA_BYTES];
        self.bit_count = 0;
        self.decode_data = 0;
    }

    fn is_short(d: u32) -> bool {
        duration_diff!(d, TE_SHORT) < TE_DELTA
    }

    fn is_long(d: u32) -> bool {
        duration_diff!(d, TE_LONG) < TE_DELTA
    }

    /// True when the duration is within the preamble pulse window (300-700µs).
    fn is_preamble_pulse(d: u32) -> bool {
        d >= PREAMBLE_PULSE_MIN && d <= PREAMBLE_PULSE_MAX
    }

    /// Flipper Manchester transition table. Event 0=ShortLow, 1=ShortHigh, 2=LongLow, 3=LongHigh.
    /// Returns Some(bit) when a bit emits.
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

    /// Map (level, duration) → Manchester event. BMW CAS4 polarity: level ? Low : High
    /// (matches the C `event = level ? ManchesterEventShortLow : ManchesterEventShortHigh`).
    fn pulse_event(level: bool, duration: u32) -> Option<u8> {
        if Self::is_short(duration) {
            Some(if level { 0 } else { 1 })
        } else if Self::is_long(duration) {
            Some(if level { 2 } else { 3 })
        } else {
            None
        }
    }

    /// Append a decoded bit MSB-first into raw_data and the 64-bit accumulator
    /// (matches the C bit-packing in the Data step).
    fn add_bit(&mut self, bit: bool) {
        if self.bit_count < DATA_BITS {
            let byte_idx = self.bit_count / 8;
            let bit_pos = 7 - (self.bit_count % 8);
            if bit {
                self.raw_data[byte_idx] |= 1 << bit_pos;
            }
            self.decode_data = (self.decode_data << 1) | (bit as u64);
        }
        self.bit_count += 1;
    }

    /// Validate the fixed markers and build the decoded signal.
    /// Fields: serial = bytes[1..4] (24-bit), button = byte[7], counter = byte[5].
    /// The CAS4 rolling cipher is left undecrypted; the markers serve as the integrity gate.
    fn build_signal(&self) -> DecodedSignal {
        let b = &self.raw_data;
        let serial = ((b[1] as u32) << 16) | ((b[2] as u32) << 8) | (b[3] as u32);
        let counter = b[5] as u16;
        let button = b[7];

        DecodedSignal {
            serial: Some(serial),
            button: Some(button),
            counter: Some(counter),
            crc_valid: true, // fixed markers (byte[0]==0x30 && byte[6]==0xC5) validated the frame
            data: self.decode_data,
            data_count_bit: DATA_BITS,
            encoder_capable: false,
            extra: None,
            protocol_display_name: None,
        }
    }
}

impl ProtocolDecoder for BmwCas4Decoder {
    fn name(&self) -> &'static str {
        "BMW CAS4"
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
        // SubGhzProtocolFlag_433 only (no 315 listed in the C).
        &[433_920_000]
    }

    fn reset(&mut self) {
        self.reset_state();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            // Begin on a high preamble pulse within the 300-700µs window.
            DecoderStep::Reset => {
                if level && Self::is_preamble_pulse(duration) {
                    self.step = DecoderStep::Preamble;
                    self.preamble_count = 1;
                }
            }

            DecoderStep::Preamble => {
                if Self::is_preamble_pulse(duration) {
                    self.preamble_count += 1;
                } else if !level && duration >= GAP_MIN {
                    if self.preamble_count >= PREAMBLE_MIN {
                        // Enter data: clear accumulators and reset the Manchester state.
                        self.bit_count = 0;
                        self.decode_data = 0;
                        self.raw_data = [0; DATA_BYTES];
                        self.manchester_state = ManchesterState::Mid1;
                        self.step = DecoderStep::Data;
                    } else {
                        self.reset_state();
                    }
                } else {
                    self.reset_state();
                }
            }

            DecoderStep::Data => {
                if self.bit_count >= DATA_BITS {
                    self.reset_state();
                    return None;
                }

                let event = Self::pulse_event(level, duration);
                if let Some(ev) = event {
                    if let Some(bit) = self.manchester_advance(ev) {
                        self.add_bit(bit);

                        if self.bit_count == DATA_BITS {
                            let valid = self.raw_data[0] == BYTE0_MARKER
                                && self.raw_data[6] == BYTE6_MARKER;
                            let result = if valid { Some(self.build_signal()) } else { None };
                            self.reset_state();
                            return result;
                        }
                    }
                } else {
                    // Out-of-range pulse aborts the frame (matches the C ManchesterEventReset path).
                    self.reset_state();
                }
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }
}

impl Default for BmwCas4Decoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Map a Manchester event index back to its (level, duration) pulse, using the decoder's
    /// polarity (level ? Low : High): 0=ShortLow(level,500), 1=ShortHigh(!level,500),
    /// 2=LongLow(level,1000), 3=LongHigh(!level,1000).
    fn event_pulse(event: u8) -> (bool, u32) {
        match event {
            0 => (true, TE_SHORT),
            1 => (false, TE_SHORT),
            2 => (true, TE_LONG),
            3 => (false, TE_LONG),
            _ => unreachable!(),
        }
    }

    /// Pure transition table (same as `manchester_advance`) for the test encoder. Returns
    /// (next_state, Some(bit)) when a bit emits.
    fn advance(state: ManchesterState, event: u8) -> (ManchesterState, Option<bool>) {
        use ManchesterState::{Mid0, Mid1, Start0, Start1};
        let (ns, emit) = match (state, event) {
            (Mid0, 0) => (Mid0, false),
            (Mid0, 1) => (Start1, true),
            (Mid0, 2) => (Mid0, false),
            (Mid0, 3) => (Mid1, true),
            (Mid1, 0) => (Start0, true),
            (Mid1, 1) => (Mid1, false),
            (Mid1, 2) => (Mid0, true),
            (Mid1, 3) => (Mid1, false),
            (Start0, 0) => (Mid0, false),
            (Start0, 1) => (Mid0, false),
            (Start0, 2) => (Mid0, false),
            (Start0, 3) => (Mid1, false),
            (Start1, 0) => (Mid0, false),
            (Start1, 1) => (Mid1, false),
            (Start1, 2) => (Mid0, false),
            (Start1, 3) => (Mid1, false),
            _ => (Mid1, false),
        };
        (ns, if emit { Some((event & 1) == 1) } else { None })
    }

    /// Faithfully Manchester-encode `bits` into level/duration pulses by *driving the decoder's
    /// own transition table*. This guarantees the produced waveform is one the decoder accepts —
    /// it does not assume any closed-form biphase rule (the Flipper table is differential, so a
    /// fixed per-bit pattern does not exist). Backtracking DFS over the tiny state space (with a
    /// visited set on (state, last_level, bit_index) to break no-emit cycles) finds a pulse stream
    /// whose decoded bits equal `bits`, respecting physical level alternation between pulses.
    ///
    /// Note: the decoder starts at Mid1, which can only emit a `0` first — so the first bit must be
    /// 0 (true for any BMW CAS4 frame: byte[0]==0x30 begins with bit 0).
    fn manchester_encode_bits(bits: &[bool]) -> Option<Vec<(bool, u32)>> {
        use std::collections::HashSet;

        fn dfs(
            state: ManchesterState,
            last_level: Option<bool>,
            i: usize,
            bits: &[bool],
            acc: &mut Vec<u8>,
            seen: &mut std::collections::HashSet<(ManchesterState, Option<bool>, usize)>,
        ) -> bool {
            if i == bits.len() {
                return true;
            }
            let key = (state, last_level, i);
            if seen.contains(&key) {
                return false;
            }
            seen.insert(key);
            for event in 0u8..4 {
                let (level, _dur) = event_pulse(event);
                if last_level == Some(level) {
                    continue; // physical OOK stream must alternate level between pulses
                }
                let (ns, bit) = advance(state, event);
                match bit {
                    None => {
                        acc.push(event);
                        if dfs(ns, Some(level), i, bits, acc, seen) {
                            return true;
                        }
                        acc.pop();
                    }
                    Some(b) if b == bits[i] => {
                        acc.push(event);
                        if dfs(ns, Some(level), i + 1, bits, acc, seen) {
                            return true;
                        }
                        acc.pop();
                    }
                    _ => {}
                }
            }
            false
        }

        let mut events: Vec<u8> = Vec::new();
        let mut seen: HashSet<(ManchesterState, Option<bool>, usize)> = HashSet::new();
        if !dfs(ManchesterState::Mid1, None, 0, bits, &mut events, &mut seen) {
            return None;
        }

        // Translate events → pulses, merging adjacent equal levels (a repeated level is a long).
        let mut merged: Vec<(bool, u32)> = Vec::new();
        for ev in events {
            let (lvl, dur) = event_pulse(ev);
            if let Some(last) = merged.last_mut() {
                if last.0 == lvl {
                    last.1 += dur;
                    continue;
                }
            }
            merged.push((lvl, dur));
        }
        Some(merged)
    }

    /// Build a full BMW CAS4 frame: preamble pulses, long low gap, then Manchester data.
    fn build_frame(bytes: &[u8; DATA_BYTES]) -> Vec<(bool, u32)> {
        let mut pairs: Vec<(bool, u32)> = Vec::new();
        // Preamble: 12 high pulses of ~500µs separated by short lows (within the 300-700 window).
        for _ in 0..12 {
            pairs.push((true, 500));
            pairs.push((false, 500));
        }
        // Long low gap (≥1800µs) ending the preamble. Overwrite the last low with the gap.
        if let Some(last) = pairs.last_mut() {
            if !last.0 {
                last.1 = GAP_MIN + 200;
            }
        }
        // Manchester-encoded data bits, MSB-first.
        let mut bits: Vec<bool> = Vec::with_capacity(DATA_BITS);
        for &byte in bytes.iter() {
            for i in (0..8).rev() {
                bits.push((byte >> i) & 1 != 0);
            }
        }
        let data = manchester_encode_bits(&bits).expect("test frame must be encodable (first bit 0)");
        pairs.extend(data);
        // Trailing gap to flush.
        pairs.push((false, GAP_MIN + 500));
        pairs
    }

    /// Feed a pair stream through a fresh decoder and return the first decode (trying both
    /// polarities, matching the registry behaviour).
    fn decode_pairs(pairs: &[(bool, u32)]) -> Option<DecodedSignal> {
        for invert in [false, true] {
            let mut dec = BmwCas4Decoder::new();
            for &(lvl, dur) in pairs {
                let level = if invert { !lvl } else { lvl };
                if let Some(sig) = dec.feed(level, dur) {
                    return Some(sig);
                }
            }
        }
        None
    }

    #[test]
    fn decodes_synthetic_frame_with_markers() {
        // byte[0]=0x30 and byte[6]=0xC5 are the required markers; the rest is the (encrypted)
        // payload. serial = bytes[1..4], counter = byte[5], button = byte[7].
        let frame: [u8; DATA_BYTES] = [0x30, 0x12, 0x34, 0x56, 0xAB, 0x07, 0xC5, 0x02];
        let pairs = build_frame(&frame);
        let sig = decode_pairs(&pairs).expect("synthetic BMW CAS4 frame should decode");

        assert!(sig.crc_valid, "fixed markers should set crc_valid=true");
        assert_eq!(sig.data_count_bit, DATA_BITS);
        assert_eq!(sig.data, u64::from_be_bytes(frame), "data must equal the 64-bit frame");
        assert_eq!(sig.serial, Some(0x123456), "serial = bytes[1..4]");
        assert_eq!(sig.counter, Some(0x07), "counter = byte[5]");
        assert_eq!(sig.button, Some(0x02), "button = byte[7]");
    }

    #[test]
    fn rejects_frame_with_wrong_markers() {
        // Same structure but byte[0] and byte[6] are NOT the markers → must not decode.
        let frame: [u8; DATA_BYTES] = [0x31, 0x12, 0x34, 0x56, 0xAB, 0x07, 0xC4, 0x02];
        let pairs = build_frame(&frame);
        assert!(
            decode_pairs(&pairs).is_none(),
            "a frame with wrong marker bytes must be rejected"
        );
    }
}
