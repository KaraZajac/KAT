//! Renault V2 (Clio 3) protocol decoder (decode-only, identify-only)
//!
//! Ported from FoxFW: `REFERENCES/FoxFW/lib/subghz/protocols/renault_v2.c`
//! (`RENAULT_V2_PROTOCOL_NAME = "Renault V2"`, `SubGhzProtocolFlag_433 | 315 | AM`, Dynamic, no
//! encoder). This is **not** KAT's "Renault V0" / "Renault V1": those are the Hitag2-style
//! 433 MHz frames, this one is the Manchester fob with the fixed 16-bit sync `0x71EA`.
//!
//! # Wire format (AM/OOK, Manchester-ish, te 230/460 µs, tolerance 60 µs)
//! 82 bits = `[sync 0x71EA : 16][encrypted payload : 66]`. The whole payload is encrypted: there is
//! no serial, button, counter or checksum in the clear, so the decoder only tells that a Renault V2
//! fob was seen and surfaces the raw 82 bits.
//!
//! The bit engine is a stateless half-bit accumulator (the C calls it "Ansatz C"): the first
//! SHORT pulse starts a frame and counts as a first half; a LONG pulse at a bit boundary is one
//! bit equal to its level; a SHORT pairs with the next SHORT and gives one bit equal to the level
//! of the second; a LONG while a half bit is pending, or any other duration, ends the frame
//! (`process`). 190 collected bits also end it.
//!
//! # Gating (as in the C)
//! `process` needs at least 60 bits, finds the first exact `0x71EA` in `bits[0..=n-60]`, and
//! requires `n - sync_pos` to be 82..=86 (the full frame present, over-long captures rejected).
//!
//! # DecodedSignal packing
//! `data` = the first 64 wire bits (sync + 48 payload bits), `data_count_bit` = 82, `extra` = the
//! last 18 wire bits (the C's `Data2`). No serial/button/counter (the C sets them to 0), and no
//! check: `crc_valid` is false and the interface shows "-" ([super::info::NO_CHECKSUM]).
//!
//! # Key files
//! `serialize` writes `Protocol: Renault V2`, `Bit: 82`, `Key` = `data` and `Data2` = the last 18 bits
//! (decimal). `deserialize` needs exactly 82 bits and reads `Data2` (masked to 18 bits, 0 when absent).
//!
//! # Encoder
//! None (the C's `.encoder` is NULL). `supports_encoding()` is false.

use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Renault V2",
    decoder: "Renault V2",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Encrypted",
    make: "Renault",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&AM650],
};

const TE_SHORT: u32 = 230;
const TE_LONG: u32 = 460;
const TE_DELTA: u32 = 60;
const DATA_BITS: usize = 82;
const MIN_PROCESS: usize = 60;
const ARR_SIZE: usize = 200;
const OVERFLOW: usize = 190;
const LEN_TOL: usize = 4;
const SYNC: u16 = 0x71EA;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    Collect,
}

/// The decode of a frame: the first 64 wire bits and the last 18.
fn signal(data: u64, tail: u64) -> DecodedSignal {
    DecodedSignal {
        serial: None,
        button: None,
        counter: None,
        crc_valid: false,
        data,
        data_count_bit: DATA_BITS,
        encoder_capable: false,
        extra: Some(tail),
        protocol_display_name: None,
    }
}

/// Renault V2 decoder (matches `SubGhzProtocolDecoderRenaultV2`).
pub struct RenaultV2Decoder {
    step: Step,
    half_bit: bool,
    all_bits: [u8; ARR_SIZE],
    all_bit_count: usize,
}

impl RenaultV2Decoder {
    pub fn new() -> Self {
        Self { step: Step::Reset, half_bit: false, all_bits: [0; ARR_SIZE], all_bit_count: 0 }
    }

    /// `renault_v2_process`: look for the sync in the collected bits and report the frame.
    fn process(&self) -> Option<DecodedSignal> {
        let n = self.all_bit_count;
        if n < MIN_PROCESS {
            return None;
        }
        let search_end = n - MIN_PROCESS;
        let bits = &self.all_bits;
        let sync_pos = (0..=search_end).find(|&i| {
            (0..16).all(|j| bits[i + j] == ((SYNC >> (15 - j)) & 1) as u8)
        })?;
        let data_after = n - sync_pos;
        if !(DATA_BITS..=DATA_BITS + LEN_TOL).contains(&data_after) {
            return None;
        }
        let mut data = 0u64;
        for i in 0..64 {
            data = (data << 1) | (bits[sync_pos + i] & 1) as u64;
        }
        let mut tail = 0u64;
        for i in 64..DATA_BITS {
            tail = (tail << 1) | (bits[sync_pos + i] & 1) as u64;
        }
        Some(signal(data, tail))
    }

    /// End of a frame: `process`, then back to `Reset`.
    fn finish(&mut self) -> Option<DecodedSignal> {
        let out = self.process();
        self.step = Step::Reset;
        out
    }
}

impl Default for RenaultV2Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for RenaultV2Decoder {
    fn name(&self) -> &'static str {
        "Renault V2"
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
        // C flags: SubGhzProtocolFlag_433 | SubGhzProtocolFlag_315 | AM.
        &[433_920_000, 315_000_000]
    }

    fn reset(&mut self) {
        self.step = Step::Reset;
        self.half_bit = false;
        self.all_bit_count = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        if self.step == Step::Reset {
            if duration_diff!(duration, TE_SHORT) <= TE_DELTA {
                self.step = Step::Collect;
                self.all_bit_count = 0;
                self.half_bit = true;
            }
            return None;
        }

        // Collect
        if self.all_bit_count >= OVERFLOW {
            return self.finish();
        }
        let is_short = duration_diff!(duration, TE_SHORT) <= TE_DELTA;
        let is_long = duration_diff!(duration, TE_LONG) <= TE_DELTA;
        if is_long {
            if self.half_bit {
                // A LONG after a half bit is a Manchester error.
                return self.finish();
            }
            if self.all_bit_count < ARR_SIZE {
                self.all_bits[self.all_bit_count] = level as u8;
                self.all_bit_count += 1;
            }
        } else if is_short {
            if self.half_bit {
                if self.all_bit_count < ARR_SIZE {
                    self.all_bits[self.all_bit_count] = level as u8;
                    self.all_bit_count += 1;
                }
                self.half_bit = false;
            } else {
                self.half_bit = true;
            }
        } else {
            return self.finish();
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }

    /// `subghz_protocol_decoder_renault_v2_deserialize` (see "Key files").
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != DATA_BITS {
            return None;
        }
        let tail = key.u32_field("Data2").unwrap_or(0) & 0x3_FFFF;
        Some(signal(key.key_u64()?, tail as u64))
    }

    /// `subghz_protocol_decoder_renault_v2_serialize`.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != DATA_BITS {
            return None;
        }
        let mut key = FlipperKey::for_protocol("Renault V2", DATA_BITS, decoded.data);
        key.push_u32("Data2", decoded.extra? as u32);
        Some(key)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Test vectors: made-up field values, built as pulse sequences the way the C expects them, written
    // as Flipper RAW files and decoded by the FoxFW C decoder (host oracle, `--registry all
    // --protocol "Renault V2"`); the expectations below are the fields the oracle printed. Levels
    // alternate from `.0`.

    /// `rv2_ok`: first pulse LOW, 131 pulses.
    const RV2_OK: (bool, &[u32]) = (
        false,
        &[
        230, 230, 460, 460, 230, 230, 460, 230, 230, 460, 460, 460, 230, 230, 230, 230,
        460, 230, 230, 230, 230, 460, 230, 230, 230, 230, 230, 230, 460, 460, 460, 460,
        460, 230, 230, 460, 230, 230, 230, 230, 460, 460, 460, 230, 230, 460, 460, 460,
        230, 230, 460, 230, 230, 230, 230, 230, 230, 460, 230, 230, 230, 230, 460, 460,
        230, 230, 460, 460, 460, 230, 230, 460, 460, 460, 460, 230, 230, 460, 460, 460,
        230, 230, 460, 230, 230, 460, 460, 460, 230, 230, 460, 460, 460, 230, 230, 230,
        230, 230, 230, 460, 230, 230, 230, 230, 460, 460, 230, 230, 230, 230, 230, 230,
        460, 230, 230, 230, 230, 460, 230, 230, 230, 230, 460, 230, 230, 230, 230, 230,
        230, 460, 3000,
        ],
    );

    /// `rv2_tol`: first pulse LOW, 138 pulses.
    const RV2_TOL: (bool, &[u32]) = (
        false,
        &[
        230, 230, 460, 460, 230, 230, 460, 230, 230, 460, 460, 460, 230, 230, 230, 230,
        460, 230, 230, 230, 230, 460, 230, 230, 230, 230, 230, 230, 460, 460, 460, 460,
        460, 230, 230, 460, 230, 230, 230, 230, 460, 460, 460, 230, 230, 460, 460, 460,
        230, 230, 460, 230, 230, 230, 230, 230, 230, 460, 230, 230, 230, 230, 460, 460,
        230, 230, 460, 460, 460, 230, 230, 460, 460, 460, 460, 230, 230, 460, 460, 460,
        230, 230, 460, 230, 230, 460, 460, 460, 230, 230, 460, 460, 460, 230, 230, 230,
        230, 230, 230, 460, 230, 230, 230, 230, 460, 460, 230, 230, 230, 230, 230, 230,
        460, 230, 230, 230, 230, 460, 230, 230, 230, 230, 460, 230, 230, 230, 230, 230,
        230, 460, 460, 230, 230, 230, 230, 230, 230, 3000,
        ],
    );

    /// `rv2_bad_sync`: first pulse LOW, 131 pulses.
    const RV2_BAD_SYNC: (bool, &[u32]) = (
        false,
        &[
        230, 230, 460, 460, 230, 230, 460, 230, 230, 460, 460, 460, 230, 230, 230, 230,
        460, 230, 230, 230, 230, 460, 230, 230, 230, 230, 230, 230, 460, 460, 460, 460,
        230, 230, 460, 460, 230, 230, 230, 230, 460, 460, 460, 230, 230, 460, 460, 460,
        230, 230, 460, 230, 230, 230, 230, 230, 230, 460, 230, 230, 230, 230, 460, 460,
        230, 230, 460, 460, 460, 230, 230, 460, 460, 460, 460, 230, 230, 460, 460, 460,
        230, 230, 460, 230, 230, 460, 460, 460, 230, 230, 460, 460, 460, 230, 230, 230,
        230, 230, 230, 460, 230, 230, 230, 230, 460, 460, 230, 230, 230, 230, 230, 230,
        460, 230, 230, 230, 230, 460, 230, 230, 230, 230, 460, 230, 230, 230, 230, 230,
        230, 460, 3000,
        ],
    );

    /// `rv2_short`: first pulse LOW, 126 pulses.
    const RV2_SHORT: (bool, &[u32]) = (
        false,
        &[
        230, 230, 460, 460, 230, 230, 460, 230, 230, 460, 460, 460, 230, 230, 230, 230,
        460, 230, 230, 230, 230, 460, 230, 230, 230, 230, 230, 230, 460, 460, 460, 460,
        460, 230, 230, 460, 230, 230, 230, 230, 460, 460, 460, 230, 230, 460, 460, 460,
        230, 230, 460, 230, 230, 230, 230, 230, 230, 460, 230, 230, 230, 230, 460, 460,
        230, 230, 460, 460, 460, 230, 230, 460, 460, 460, 460, 230, 230, 460, 460, 460,
        230, 230, 460, 230, 230, 460, 460, 460, 230, 230, 460, 460, 460, 230, 230, 230,
        230, 230, 230, 460, 230, 230, 230, 230, 460, 460, 230, 230, 230, 230, 230, 230,
        460, 230, 230, 230, 230, 460, 230, 230, 230, 230, 460, 230, 230, 3000,
        ],
    );

    /// `rv2_long`: first pulse LOW, 140 pulses.
    const RV2_LONG: (bool, &[u32]) = (
        false,
        &[
        230, 230, 460, 460, 230, 230, 460, 230, 230, 460, 460, 460, 230, 230, 230, 230,
        460, 230, 230, 230, 230, 460, 230, 230, 230, 230, 230, 230, 460, 460, 460, 460,
        460, 230, 230, 460, 230, 230, 230, 230, 460, 460, 460, 230, 230, 460, 460, 460,
        230, 230, 460, 230, 230, 230, 230, 230, 230, 460, 230, 230, 230, 230, 460, 460,
        230, 230, 460, 460, 460, 230, 230, 460, 460, 460, 460, 230, 230, 460, 460, 460,
        230, 230, 460, 230, 230, 460, 460, 460, 230, 230, 460, 460, 460, 230, 230, 230,
        230, 230, 230, 460, 230, 230, 230, 230, 460, 460, 230, 230, 230, 230, 230, 230,
        460, 230, 230, 230, 230, 460, 230, 230, 230, 230, 460, 230, 230, 230, 230, 230,
        230, 460, 460, 230, 230, 230, 230, 230, 230, 230, 230, 3000,
        ],
    );

    fn run(v: (bool, &[u32])) -> Vec<DecodedSignal> {
        let mut dec = RenaultV2Decoder::new();
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
        assert_eq!(run(RV2_OK), vec![DecodedSignal { serial: None, button: None, counter: None, crc_valid: false, data: 0x71ea74b0ed2965a1, data_count_bit: 82, encoder_capable: false, extra: Some(0x378e1), protocol_display_name: None }], "rv2_ok");
        assert_eq!(run(RV2_TOL), vec![DecodedSignal { serial: None, button: None, counter: None, crc_valid: false, data: 0x71ea74b0ed2965a1, data_count_bit: 82, encoder_capable: false, extra: Some(0x378e1), protocol_display_name: None }], "rv2_tol");
    }

    #[test]
    fn rejects_what_the_c_rejects() {
        assert!(run(RV2_BAD_SYNC).is_empty(), "rv2_bad_sync: sync 0x71EB");
        assert!(run(RV2_SHORT).is_empty(), "rv2_short: 79 bits after the sync");
        assert!(run(RV2_LONG).is_empty(), "rv2_long: 87 bits after the sync (over the 82+4 window)");
    }

    #[test]
    fn decode_only() {
        let d = RenaultV2Decoder::new();
        assert!(!d.supports_encoding());
        assert!(d.encode(&DecodedSignal::new(0, 0), 1).is_none());
    }

    #[test]
    fn key_file_round_trip() {
        use crate::protocols::flipper_key::testing::assert_round_trip;
        let d = run(RV2_OK).remove(0);
        let text = assert_round_trip(&RenaultV2Decoder::new(), &d);
        assert!(text.contains("Protocol: Renault V2\nBit: 82\nKey: 71 EA 74 B0 ED 29 65 A1\nData2: 227553\n"), "{text}");
        let mut wrong = FlipperKey::parse(&text).unwrap();
        wrong.bits = 64;
        assert!(RenaultV2Decoder::new().from_flipper_key(&wrong).is_none(), "the C needs 82 bits");
    }
}
