//! Opel V0 protocol decoder (receive only, as in FoxFW; experimental)
//!
//! Ported from FoxFW: `REFERENCES/FoxFW/lib/subghz/protocols/opel_v0.c`
//! (`SUBGHZ_PROTOCOL_OPEL_V0_NAME = "Opel V0"`, `433 | AM`, Dynamic; its encoder is an SD plugin
//! that transmits nothing, see "Transmit"). A different protocol from KAT's "Opel V1" (XTEA,
//! Manchester 120/240 µs) and from rtl_433's "Opel-Mokka". The C calls itself a first-pass decoder:
//! "the exact encoding is not fully understood".
//!
//! # Wire format (AM/OOK, ~1200 Hz carrier bursts, burst-width coding)
//! The carrier (about 830 µs per cycle) is gated into bursts about 5400 µs apart. The decoder:
//! - merges marks separated by spaces under 120 µs into one burst (the tiny space is added to it),
//! - needs a preamble: bursts (separated by spaces of 120..6999 µs) whose widths add up to 50000 µs,
//! - then every space of 4000..=7000 µs closes a burst (> 500 µs) as one bit, a space above
//!   20000 µs closes the last burst and the frame; a space of 7001..=20000 µs extends the current
//!   burst; a mark that makes a burst longer than 10000 µs ends the frame early and restarts the
//!   preamble.
//! - burst width above 3200 µs is a 1, otherwise a 0; at most 150 bursts are kept.
//! - the burst and preamble sums are the C's `uint32_t`: they wrap, as there, on absurd durations.
//!
//! # Gating (as in the C)
//! Of the collected bits only the first 96 count and the count must be 62..=66 (the "modal" 64),
//! and an all-zero or all-one frame is dropped. There is no checksum.
//!
//! # DecodedSignal packing
//! `data` = the first 64 bits (left-aligned when fewer), `data_count_bit` = the count (62..=66),
//! `extra` = the C's `seed`: the bits after the 64th, left-aligned in 32 bits (only when more than
//! 64). `crc_valid` is false (no check): the interface shows "-".
//!
//! # Key files
//! `serialize` is the common one: `Protocol: Opel V0`, `Bit: <count>`, `Key` = `data` (the `seed` is not
//! written). The C's `deserialize` demands exactly `min_count_bit_for_found` = 40 bits, which no
//! decoded frame has (62..=66), so a key KAT saves is never read back by the C; `from_flipper_key`
//! follows it and only loads `Bit: 40`.
//!
//! # Transmit
//! None, as in FoxFW, whose encoder is a stub. The protocol has the `Send` flag and an encoder in
//! the SD plugin `protocols_ext/enc_opel_v0.c` ("Encoder stubs (no encoding yet)"), but its
//! `deserialize` returns `SubGhzProtocolStatusErrorEncoderGetUpload` for every key file and its
//! `yield` sends nothing: a Flipper offers Send and fails every time. The host oracle
//! (`subghz_oracle_foxfwenc encode`) reports "deserialize failed (status -12)" for each of 100
//! made-up key files (40, 48, 62..=66 and 96 bits), so `supports_encoding()` stays false.

use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Opel V0",
    decoder: "Opel V0",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Unknown",
    make: "Opel",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&AM650],
};

const MERGE_THRESHOLD: u32 = 120;
const GAP_MIN: u32 = 4000;
const GAP_MAX: u32 = 7000;
const PREAMBLE_MIN: u32 = 50000;
const BIT_THRESHOLD: u32 = 3200;
const MIN_BITS: usize = 40;
const MAX_BITS: usize = 150;
const FRAME_GAP: u32 = 20000;
const FRAME_BITS: usize = 64;
const FRAME_DELTA: usize = 2;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    Preamble,
    Data,
}

/// Opel V0 decoder (matches `SubGhzProtocolDecoderOpelV0`).
pub struct OpelV0Decoder {
    step: Step,
    current_burst: u32,
    preamble_len: u32,
    bit_count: usize,
    burst_widths: [u16; MAX_BITS],
}

impl OpelV0Decoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            current_burst: 0,
            preamble_len: 0,
            bit_count: 0,
            burst_widths: [0; MAX_BITS],
        }
    }

    /// `subghz_protocol_opel_v0_process_frame`.
    fn process_frame(&self) -> Option<DecodedSignal> {
        if self.bit_count < MIN_BITS {
            return None;
        }
        let count = self.bit_count.min(96);
        let mut data = 0u64;
        let mut data_2 = 0u32;
        for i in 0..count {
            let bit = (self.burst_widths[i] as u32 > BIT_THRESHOLD) as u64;
            if i < 64 {
                data = (data << 1) | bit;
            } else {
                data_2 = (data_2 << 1) | bit as u32;
            }
        }
        if !(FRAME_BITS - FRAME_DELTA..=FRAME_BITS + FRAME_DELTA).contains(&count) {
            return None;
        }
        let used_mask = if count >= 64 { u64::MAX } else { (1u64 << count) - 1 };
        let all_zero = data == 0 && data_2 == 0;
        let all_one = data == used_mask && (count <= 64 || data_2 == (1u32 << (count - 64)) - 1);
        if all_zero || all_one {
            return None;
        }
        if count < 64 {
            data <<= 64 - count;
        }
        let extra = if count > 64 { Some((data_2 << (32 - (count - 64))) as u64) } else { None };
        Some(DecodedSignal {
            serial: None,
            button: None,
            counter: None,
            crc_valid: false,
            data,
            data_count_bit: count,
            encoder_capable: false,
            extra,
            protocol_display_name: None,
        })
    }

    /// Store the current burst as the next bit (the C's `current_burst > 500` guard).
    fn push_burst(&mut self) {
        if self.current_burst > 500 && self.bit_count < MAX_BITS {
            self.burst_widths[self.bit_count] = self.current_burst.min(0xFFFF) as u16;
            self.bit_count += 1;
        }
    }
}

impl Default for OpelV0Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for OpelV0Decoder {
    fn name(&self) -> &'static str {
        "Opel V0"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: 830, te_long: 5400, te_delta: 300, min_count_bit: MIN_BITS }
    }

    fn supported_frequencies(&self) -> &[u32] {
        // C flags: SubGhzProtocolFlag_433 | SubGhzProtocolFlag_AM.
        &[433_920_000]
    }

    fn reset(&mut self) {
        self.step = Step::Reset;
        self.current_burst = 0;
        self.preamble_len = 0;
        self.bit_count = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                if level {
                    self.current_burst = duration;
                    self.preamble_len = 0;
                    self.bit_count = 0;
                    self.step = Step::Preamble;
                }
            }
            // Both accumulators are the C's `uint32_t`: every addition wraps as it does there.
            Step::Preamble => {
                if level || duration < MERGE_THRESHOLD {
                    // A mark, or a tiny space inside the carrier: part of the burst.
                    self.current_burst = self.current_burst.wrapping_add(duration);
                } else if duration < GAP_MAX {
                    self.preamble_len = self.preamble_len.wrapping_add(self.current_burst);
                    if self.preamble_len >= PREAMBLE_MIN {
                        self.step = Step::Data;
                        self.bit_count = 0;
                    }
                    self.current_burst = 0;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::Data => {
                if level {
                    self.current_burst = self.current_burst.wrapping_add(duration);
                    if self.current_burst > 10000 {
                        // A very long burst: end this frame, start over as a preamble.
                        let out = self.process_frame();
                        self.preamble_len = self.current_burst;
                        self.current_burst = 0;
                        self.bit_count = 0;
                        self.step = Step::Preamble;
                        return out;
                    }
                } else if duration < MERGE_THRESHOLD {
                    self.current_burst = self.current_burst.wrapping_add(duration);
                } else if (GAP_MIN..=GAP_MAX).contains(&duration) {
                    self.push_burst();
                    self.current_burst = 0;
                } else if duration > FRAME_GAP {
                    self.push_burst();
                    let out = self.process_frame();
                    self.current_burst = 0;
                    self.bit_count = 0;
                    self.preamble_len = 0;
                    self.step = Step::Reset;
                    return out;
                } else if self.current_burst > 0 {
                    // Between the burst-gap and frame-gap ranges: treated as part of the burst.
                    self.current_burst = self.current_burst.wrapping_add(duration);
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

    /// `subghz_protocol_decoder_opel_v0_deserialize` (see "Key files": the C needs 40 bits).
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != MIN_BITS {
            return None;
        }
        Some(DecodedSignal::new(key.key_u64()?, MIN_BITS))
    }

    /// The common `subghz_protocol_decoder_common_serialize`.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        Some(FlipperKey::for_protocol("Opel V0", decoded.data_count_bit, decoded.data))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Test vectors: made-up field values, built as pulse sequences the way the C expects them, written
    // as Flipper RAW files and decoded by the FoxFW C decoder (host oracle, `--registry all
    // --protocol "Opel V0"`); the expectations below are the fields the oracle printed. Levels
    // alternate from `.0`.

    /// `opel_64`: first pulse HIGH, 156 pulses.
    const OPEL_64: (bool, &[u32]) = (
        true,
        &[
        55000, 5400, 2500, 5400, 2500, 5400, 1750, 40, 1750, 5400, 3500, 5400, 2500, 5400, 2500, 5400,
        3500, 5400, 1750, 40, 1750, 5400, 2500, 5400, 2500, 5400, 3500, 5400, 3500, 5400, 1750, 40,
        1750, 5400, 2500, 5400, 2500, 5400, 2500, 5400, 3500, 5400, 1250, 40, 1250, 5400, 2500, 5400,
        2500, 5400, 2500, 5400, 3500, 5400, 1250, 40, 1250, 5400, 3500, 5400, 3500, 5400, 3500, 5400,
        3500, 5400, 1750, 40, 1750, 5400, 3500, 5400, 2500, 5400, 3500, 5400, 2500, 5400, 1250, 40,
        1250, 5400, 2500, 5400, 3500, 5400, 2500, 5400, 3500, 5400, 1750, 40, 1750, 5400, 3500, 5400,
        3500, 5400, 3500, 5400, 3500, 5400, 1750, 40, 1750, 5400, 2500, 5400, 3500, 5400, 2500, 5400,
        3500, 5400, 1250, 40, 1250, 5400, 3500, 5400, 2500, 5400, 2500, 5400, 3500, 5400, 1750, 40,
        1750, 5400, 2500, 5400, 2500, 5400, 3500, 5400, 3500, 5400, 1250, 40, 1250, 5400, 3500, 5400,
        2500, 5400, 3500, 5400, 2500, 5400, 1250, 40, 1250, 5400, 3500, 25000,
        ],
    );

    /// `opel_66`: first pulse HIGH, 160 pulses.
    const OPEL_66: (bool, &[u32]) = (
        true,
        &[
        55000, 5400, 2500, 5400, 3500, 5400, 1250, 40, 1250, 5400, 3500, 5400, 3500, 5400, 2500, 5400,
        2500, 5400, 1250, 40, 1250, 5400, 2500, 5400, 3500, 5400, 3500, 5400, 2500, 5400, 1250, 40,
        1250, 5400, 3500, 5400, 3500, 5400, 2500, 5400, 2500, 5400, 1750, 40, 1750, 5400, 2500, 5400,
        2500, 5400, 3500, 5400, 3500, 5400, 1250, 40, 1250, 5400, 2500, 5400, 3500, 5400, 3500, 5400,
        3500, 5400, 1250, 40, 1250, 5400, 3500, 5400, 3500, 5400, 2500, 5400, 2500, 5400, 1250, 40,
        1250, 5400, 3500, 5400, 3500, 5400, 2500, 5400, 3500, 5400, 1250, 40, 1250, 5400, 3500, 5400,
        3500, 5400, 2500, 5400, 3500, 5400, 1750, 40, 1750, 5400, 3500, 5400, 3500, 5400, 3500, 5400,
        2500, 5400, 1250, 40, 1250, 5400, 3500, 5400, 3500, 5400, 2500, 5400, 2500, 5400, 1250, 40,
        1250, 5400, 3500, 5400, 3500, 5400, 3500, 5400, 3500, 5400, 1250, 40, 1250, 5400, 2500, 5400,
        2500, 5400, 3500, 5400, 2500, 5400, 1750, 40, 1750, 5400, 3500, 5400, 2500, 5400, 3500, 25000,
        ],
    );

    /// `opel_62`: first pulse HIGH, 150 pulses.
    const OPEL_62: (bool, &[u32]) = (
        true,
        &[
        55000, 5400, 3500, 5400, 3500, 5400, 1250, 40, 1250, 5400, 3500, 5400, 2500, 5400, 2500, 5400,
        2500, 5400, 1250, 40, 1250, 5400, 3500, 5400, 3500, 5400, 2500, 5400, 3500, 5400, 1250, 40,
        1250, 5400, 2500, 5400, 2500, 5400, 2500, 5400, 3500, 5400, 1750, 40, 1750, 5400, 2500, 5400,
        3500, 5400, 2500, 5400, 2500, 5400, 1250, 40, 1250, 5400, 3500, 5400, 2500, 5400, 2500, 5400,
        2500, 5400, 1250, 40, 1250, 5400, 2500, 5400, 2500, 5400, 2500, 5400, 2500, 5400, 1750, 40,
        1750, 5400, 3500, 5400, 2500, 5400, 2500, 5400, 2500, 5400, 1250, 40, 1250, 5400, 3500, 5400,
        3500, 5400, 2500, 5400, 3500, 5400, 1750, 40, 1750, 5400, 2500, 5400, 2500, 5400, 3500, 5400,
        2500, 5400, 1750, 40, 1750, 5400, 3500, 5400, 2500, 5400, 3500, 5400, 2500, 5400, 1750, 40,
        1750, 5400, 3500, 5400, 3500, 5400, 3500, 5400, 3500, 5400, 1250, 40, 1250, 5400, 3500, 5400,
        3500, 5400, 2500, 5400, 2500, 25000,
        ],
    );

    /// `opel_61`: first pulse HIGH, 148 pulses.
    const OPEL_61: (bool, &[u32]) = (
        true,
        &[
        55000, 5400, 2500, 5400, 3500, 5400, 1750, 40, 1750, 5400, 2500, 5400, 2500, 5400, 2500, 5400,
        3500, 5400, 1750, 40, 1750, 5400, 3500, 5400, 2500, 5400, 3500, 5400, 3500, 5400, 1250, 40,
        1250, 5400, 3500, 5400, 2500, 5400, 2500, 5400, 3500, 5400, 1250, 40, 1250, 5400, 3500, 5400,
        3500, 5400, 2500, 5400, 3500, 5400, 1750, 40, 1750, 5400, 3500, 5400, 3500, 5400, 2500, 5400,
        2500, 5400, 1750, 40, 1750, 5400, 2500, 5400, 2500, 5400, 2500, 5400, 3500, 5400, 1750, 40,
        1750, 5400, 2500, 5400, 2500, 5400, 3500, 5400, 2500, 5400, 1750, 40, 1750, 5400, 3500, 5400,
        3500, 5400, 2500, 5400, 2500, 5400, 1250, 40, 1250, 5400, 2500, 5400, 3500, 5400, 3500, 5400,
        2500, 5400, 1750, 40, 1750, 5400, 3500, 5400, 2500, 5400, 2500, 5400, 2500, 5400, 1250, 40,
        1250, 5400, 2500, 5400, 2500, 5400, 2500, 5400, 3500, 5400, 1750, 40, 1750, 5400, 3500, 5400,
        3500, 5400, 3500, 25000,
        ],
    );

    /// `opel_67`: first pulse HIGH, 162 pulses.
    const OPEL_67: (bool, &[u32]) = (
        true,
        &[
        55000, 5400, 3500, 5400, 2500, 5400, 1750, 40, 1750, 5400, 2500, 5400, 2500, 5400, 2500, 5400,
        3500, 5400, 1250, 40, 1250, 5400, 2500, 5400, 2500, 5400, 2500, 5400, 3500, 5400, 1750, 40,
        1750, 5400, 2500, 5400, 2500, 5400, 2500, 5400, 3500, 5400, 1250, 40, 1250, 5400, 2500, 5400,
        2500, 5400, 2500, 5400, 3500, 5400, 1250, 40, 1250, 5400, 2500, 5400, 2500, 5400, 2500, 5400,
        3500, 5400, 1750, 40, 1750, 5400, 2500, 5400, 2500, 5400, 3500, 5400, 2500, 5400, 1250, 40,
        1250, 5400, 2500, 5400, 3500, 5400, 2500, 5400, 2500, 5400, 1250, 40, 1250, 5400, 2500, 5400,
        3500, 5400, 3500, 5400, 3500, 5400, 1750, 40, 1750, 5400, 3500, 5400, 3500, 5400, 3500, 5400,
        2500, 5400, 1250, 40, 1250, 5400, 2500, 5400, 2500, 5400, 3500, 5400, 3500, 5400, 1750, 40,
        1750, 5400, 3500, 5400, 3500, 5400, 2500, 5400, 2500, 5400, 1750, 40, 1750, 5400, 2500, 5400,
        3500, 5400, 2500, 5400, 3500, 5400, 1750, 40, 1750, 5400, 2500, 5400, 2500, 5400, 3500, 5400,
        3500, 25000,
        ],
    );

    /// `opel_zero`: first pulse HIGH, 156 pulses.
    const OPEL_ZERO: (bool, &[u32]) = (
        true,
        &[
        55000, 5400, 2500, 5400, 2500, 5400, 1250, 40, 1250, 5400, 2500, 5400, 2500, 5400, 2500, 5400,
        2500, 5400, 1250, 40, 1250, 5400, 2500, 5400, 2500, 5400, 2500, 5400, 2500, 5400, 1250, 40,
        1250, 5400, 2500, 5400, 2500, 5400, 2500, 5400, 2500, 5400, 1250, 40, 1250, 5400, 2500, 5400,
        2500, 5400, 2500, 5400, 2500, 5400, 1250, 40, 1250, 5400, 2500, 5400, 2500, 5400, 2500, 5400,
        2500, 5400, 1250, 40, 1250, 5400, 2500, 5400, 2500, 5400, 2500, 5400, 2500, 5400, 1250, 40,
        1250, 5400, 2500, 5400, 2500, 5400, 2500, 5400, 2500, 5400, 1250, 40, 1250, 5400, 2500, 5400,
        2500, 5400, 2500, 5400, 2500, 5400, 1250, 40, 1250, 5400, 2500, 5400, 2500, 5400, 2500, 5400,
        2500, 5400, 1250, 40, 1250, 5400, 2500, 5400, 2500, 5400, 2500, 5400, 2500, 5400, 1250, 40,
        1250, 5400, 2500, 5400, 2500, 5400, 2500, 5400, 2500, 5400, 1250, 40, 1250, 5400, 2500, 5400,
        2500, 5400, 2500, 5400, 2500, 5400, 1250, 40, 1250, 5400, 2500, 25000,
        ],
    );

    /// `opel_ones`: first pulse HIGH, 156 pulses.
    const OPEL_ONES: (bool, &[u32]) = (
        true,
        &[
        55000, 5400, 3500, 5400, 3500, 5400, 1750, 40, 1750, 5400, 3500, 5400, 3500, 5400, 3500, 5400,
        3500, 5400, 1750, 40, 1750, 5400, 3500, 5400, 3500, 5400, 3500, 5400, 3500, 5400, 1750, 40,
        1750, 5400, 3500, 5400, 3500, 5400, 3500, 5400, 3500, 5400, 1750, 40, 1750, 5400, 3500, 5400,
        3500, 5400, 3500, 5400, 3500, 5400, 1750, 40, 1750, 5400, 3500, 5400, 3500, 5400, 3500, 5400,
        3500, 5400, 1750, 40, 1750, 5400, 3500, 5400, 3500, 5400, 3500, 5400, 3500, 5400, 1750, 40,
        1750, 5400, 3500, 5400, 3500, 5400, 3500, 5400, 3500, 5400, 1750, 40, 1750, 5400, 3500, 5400,
        3500, 5400, 3500, 5400, 3500, 5400, 1750, 40, 1750, 5400, 3500, 5400, 3500, 5400, 3500, 5400,
        3500, 5400, 1750, 40, 1750, 5400, 3500, 5400, 3500, 5400, 3500, 5400, 3500, 5400, 1750, 40,
        1750, 5400, 3500, 5400, 3500, 5400, 3500, 5400, 3500, 5400, 1750, 40, 1750, 5400, 3500, 5400,
        3500, 5400, 3500, 5400, 3500, 5400, 1750, 40, 1750, 5400, 3500, 25000,
        ],
    );

    /// `opel_nopre`: first pulse HIGH, 156 pulses.
    const OPEL_NOPRE: (bool, &[u32]) = (
        true,
        &[
        40000, 5400, 2500, 5400, 2500, 5400, 1750, 40, 1750, 5400, 3500, 5400, 2500, 5400, 2500, 5400,
        3500, 5400, 1750, 40, 1750, 5400, 2500, 5400, 2500, 5400, 3500, 5400, 3500, 5400, 1750, 40,
        1750, 5400, 2500, 5400, 2500, 5400, 2500, 5400, 3500, 5400, 1250, 40, 1250, 5400, 2500, 5400,
        2500, 5400, 2500, 5400, 3500, 5400, 1250, 40, 1250, 5400, 3500, 5400, 3500, 5400, 3500, 5400,
        3500, 5400, 1750, 40, 1750, 5400, 3500, 5400, 2500, 5400, 3500, 5400, 2500, 5400, 1250, 40,
        1250, 5400, 2500, 5400, 3500, 5400, 2500, 5400, 3500, 5400, 1750, 40, 1750, 5400, 3500, 5400,
        3500, 5400, 3500, 5400, 3500, 5400, 1750, 40, 1750, 5400, 2500, 5400, 3500, 5400, 2500, 5400,
        3500, 5400, 1250, 40, 1250, 5400, 3500, 5400, 2500, 5400, 2500, 5400, 3500, 5400, 1750, 40,
        1750, 5400, 2500, 5400, 2500, 5400, 3500, 5400, 3500, 5400, 1250, 40, 1250, 5400, 3500, 5400,
        2500, 5400, 3500, 5400, 2500, 5400, 1250, 40, 1250, 5400, 3500, 25000,
        ],
    );

    fn run(v: (bool, &[u32])) -> Vec<DecodedSignal> {
        let mut dec = OpelV0Decoder::new();
        let mut level = v.0;
        let mut out = Vec::new();
        for &d in v.1 {
            if let Some(s) = dec.feed(level, d) {
                out.push(s);
            }
            level = !level;
        }
        out
    }

    #[test]
    fn decodes_the_oracle_frames() {
        assert_eq!(run(OPEL_64), vec![DecodedSignal { serial: None, button: None, counter: None, crc_valid: false, data: 0x333885fa2fea99a9, data_count_bit: 64, encoder_capable: false, extra: None, protocol_display_name: None }], "opel_64");
        assert_eq!(run(OPEL_66), vec![DecodedSignal { serial: None, button: None, counter: None, crc_valid: false, data: 0x58664cec6b7cc78b, data_count_bit: 66, encoder_capable: false, extra: Some(0x40000000), protocol_display_name: None }], "opel_66");
        assert_eq!(run(OPEL_62), vec![DecodedSignal { serial: None, button: None, counter: None, crc_valid: false, data: 0xd0d0d100c365afb0, data_count_bit: 62, encoder_capable: false, extra: None, protocol_display_name: None }], "opel_62");
    }

    #[test]
    fn rejects_what_the_c_rejects() {
        assert!(run(OPEL_61).is_empty(), "opel_61: 61 bursts");
        assert!(run(OPEL_67).is_empty(), "opel_67: 67 bursts");
        assert!(run(OPEL_ZERO).is_empty(), "opel_zero: all-zero frame");
        assert!(run(OPEL_ONES).is_empty(), "opel_ones: all-one frame");
        assert!(run(OPEL_NOPRE).is_empty(), "opel_nopre: preamble of 40 ms (<50 ms)");
    }

    /// No encoder: FoxFW's refuses every key file (see the module doc, "Transmit").
    #[test]
    fn receive_only() {
        let d = OpelV0Decoder::new();
        assert!(!d.supports_encoding());
        assert!(d.encode(&DecodedSignal::new(0, 0), 1).is_none());
        let forty = d.from_flipper_key(&FlipperKey::for_protocol("Opel V0", 40, 0x12_3456_789A)).unwrap();
        assert!(!forty.encoder_capable && d.encode(&forty, 0).is_none());
    }

    /// The burst and preamble sums are the C's `uint32_t` and wrap (no overflow panic in a debug
    /// build): marks of half the range added in the preamble, the preamble sum taking a second
    /// huge burst, and a mark added to a burst that tiny and mid-range spaces have grown.
    #[test]
    fn sums_wrap_like_the_c() {
        let half = u32::MAX / 2 + 1;
        let mut d = OpelV0Decoder::new();
        // Reset → Preamble, then two marks: 2 · (2^31 + 1) wraps to 2.
        for (level, duration) in [(true, half + 1), (true, half + 1)] {
            assert!(d.feed(level, duration).is_none());
        }
        assert_eq!((d.step, d.current_burst), (Step::Preamble, 2));
        // A burst of 3·10^9 µs closes the preamble (sum ≥ 50000) → Data; another one ends the
        // frame and becomes the next preamble sum, which the following burst then overflows.
        let mut d = OpelV0Decoder::new();
        let big = 3_000_000_000;
        for (level, duration) in [(true, big), (false, 200), (true, big), (true, big), (false, 200)] {
            assert!(d.feed(level, duration).is_none());
        }
        assert_eq!(d.preamble_len, big.wrapping_add(big));
        // Data: a burst grown by spaces, then a mark of u32::MAX.
        let mut d = OpelV0Decoder::new();
        for (level, duration) in [(true, 60_000), (false, 200), (true, 9_000), (false, 100), (false, 15_000), (true, u32::MAX)] {
            d.feed(level, duration);
        }
        assert_eq!(d.preamble_len, 9_000u32.wrapping_add(100).wrapping_add(15_000).wrapping_add(u32::MAX));
        // Any mix of huge pulses goes through.
        let mut d = OpelV0Decoder::new();
        for i in 0..64u32 {
            d.feed(i % 3 != 1, if i % 5 == 0 { u32::MAX } else { half });
        }
    }

    /// Oracle comparison: `KAT_OPEL_V0_ENCODE=<file listing key files>` prints KAT's transmission for
    /// each (`KATENC <file> | (none)`: as `subghz_oracle_foxfwenc encode`, which fails every Opel V0
    /// key file with status -12).
    #[test]
    #[ignore = "oracle harness"]
    fn encode_key_files() {
        let Ok(list) = std::env::var("KAT_OPEL_V0_ENCODE") else { return };
        let dec = OpelV0Decoder::new();
        for file in std::fs::read_to_string(list).unwrap().lines().filter(|l| !l.is_empty()) {
            let key = FlipperKey::parse(&std::fs::read_to_string(file).unwrap()).unwrap();
            let out = dec.from_flipper_key(&key).and_then(|d| dec.encode(&d, d.button.unwrap_or(0)));
            assert!(out.is_none(), "{file}");
            println!("KATENC {file} | (none)");
        }
    }

    #[test]
    fn key_file_follows_the_c() {
        let dec = OpelV0Decoder::new();
        let d = run(OPEL_66).remove(0);
        let key = dec.to_flipper_key(&d).unwrap();
        assert_eq!((key.protocol.as_str(), key.bits, key.key), ("Opel V0", 66, d.data));
        assert!(key.fields.is_empty(), "the seed is not written");
        // The C's deserialize only takes 40 bits, so a 66-bit key does not load.
        assert!(dec.from_flipper_key(&key).is_none());
        let mut forty = key.clone();
        forty.bits = 40;
        assert_eq!(dec.from_flipper_key(&forty), Some(DecodedSignal::new(d.data, 40)));
    }
}
