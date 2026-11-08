//! Renault V3 (Megane) protocol decoder (decode-only)
//!
//! Ported from FoxFW: `REFERENCES/FoxFW/lib/subghz/protocols/renault_v3.c`
//! (`RENAULT_V3_PROTOCOL_NAME = "Renault V3"`, `433 | 315 | AM | FM`, Dynamic, no encoder). A
//! different protocol from KAT's "Renault V0" / "Renault V1" (Hitag2-style) and from "Renault V2".
//!
//! # Wire format (2-FSK, NRZ, te 52/104 µs, tolerance 22 µs)
//! Variable 64..=160-bit frame behind a preamble of short pulses:
//! `[sync : 16][serial : 16][encrypted remainder]` with sync `0xC969` (type A) or `0xCB2D` (type B).
//! Only the 16-bit serial is in the clear; button and counter sit inside the encrypted remainder.
//! There is no checksum.
//!
//! The bit engine is an NRZ quantiser: after at least 10 SHORT pulses (any level) the first
//! non-SHORT pulse starts the data and every pulse then adds `round(duration / 52)` bits of its
//! level (at most 8 per pulse; a longer pulse, or a pulse that rounds to 0, ends the frame).
//! 250 collected bits also end it. The preamble is abandoned after 5000 pulses.
//!
//! # Gating (as in the C)
//! `process` needs at least 64 bits, finds the first exact sync A or B in `bits[0..=n-64]`,
//! requires `n - sync_pos` in 64..=160, and rejects the degenerate serials `0x0000`, `0xFFFF`,
//! `0x5555` and `0xAAAA`.
//!
//! # DecodedSignal packing
//! `data` = the first 64 frame bits (sync, serial, 32 encrypted bits), `data_count_bit` = the
//! frame length (64..=160), `serial` = the 16-bit serial, `extra` = frame bits 64..127 (right
//! aligned, what is there). **Deviation:** the C also keeps frame bits 128..159 (`Frame`, 20
//! bytes); `DecodedSignal` has no room for them, they are encrypted and not shown. The sync tells
//! type A from B. No button or counter, no check: `crc_valid` is false.
//!
//! # Encoder
//! None (the C's `.encoder` is NULL).

use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Renault V3",
    decoder: "Renault V3",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Nrz,
    rf: Rf::Both,
    encryption: "Encrypted",
    make: "Renault",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&AM650, &FM476],
};

const TE_SHORT: u32 = 52;
const TE_DELTA: u32 = 22;
const MIN_DATA_BITS: usize = 64;
const MAX_DATA_BITS: usize = 160;
const MIN_PREAMBLE: u32 = 10;
const ARR_SIZE: usize = 256;
const OVERFLOW: usize = 250;
const SYNC_A: u16 = 0xC969;
const SYNC_B: u16 = 0xCB2D;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    Preamble,
    Collect,
}

/// Renault V3 decoder (matches `SubGhzProtocolDecoderRenaultV3`).
pub struct RenaultV3Decoder {
    step: Step,
    preamble_count: u32,
    all_bits: [u8; ARR_SIZE],
    all_bit_count: usize,
}

/// Structurally meaningless 16-bit serials.
fn serial_degenerate(s: u16) -> bool {
    s == 0x0000 || s == 0xFFFF || s == 0x5555 || s == 0xAAAA
}

impl RenaultV3Decoder {
    pub fn new() -> Self {
        Self { step: Step::Reset, preamble_count: 0, all_bits: [0; ARR_SIZE], all_bit_count: 0 }
    }

    /// `renault_v3_append_n`.
    fn append_n(&mut self, n: u32, level: bool) {
        for _ in 0..n {
            if self.all_bit_count < ARR_SIZE {
                self.all_bits[self.all_bit_count] = level as u8;
                self.all_bit_count += 1;
            }
        }
    }

    fn sync_at(&self, i: usize, sync: u16) -> bool {
        (0..16).all(|j| self.all_bits[i + j] == ((sync >> (15 - j)) & 1) as u8)
    }

    /// `renault_v3_process`.
    fn process(&self) -> Option<DecodedSignal> {
        let n = self.all_bit_count;
        if n < MIN_DATA_BITS {
            return None;
        }
        let search_end = n - MIN_DATA_BITS;
        let sync_pos =
            (0..=search_end).find(|&i| self.sync_at(i, SYNC_A) || self.sync_at(i, SYNC_B))?;
        let data_after = n - sync_pos;
        if !(MIN_DATA_BITS..=MAX_DATA_BITS).contains(&data_after) {
            return None;
        }
        let data_len = data_after;
        let mut data = 0u64;
        for i in 0..64 {
            data = (data << 1) | (self.all_bits[sync_pos + i] & 1) as u64;
        }
        let serial = ((data >> 32) & 0xFFFF) as u16;
        if serial_degenerate(serial) {
            return None;
        }
        let mut extra = 0u64;
        for i in 64..data_len.min(128) {
            extra = (extra << 1) | (self.all_bits[sync_pos + i] & 1) as u64;
        }
        Some(DecodedSignal {
            serial: Some(serial as u32),
            button: None,
            counter: None,
            crc_valid: false,
            data,
            data_count_bit: data_len,
            encoder_capable: false,
            extra: Some(extra),
            protocol_display_name: None,
        })
    }

    fn finish(&mut self) -> Option<DecodedSignal> {
        let out = self.process();
        self.step = Step::Reset;
        out
    }
}

impl Default for RenaultV3Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for RenaultV3Decoder {
    fn name(&self) -> &'static str {
        "Renault V3"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: 104,
            te_delta: TE_DELTA,
            min_count_bit: MIN_DATA_BITS,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        // C flags: SubGhzProtocolFlag_433 | SubGhzProtocolFlag_315 | AM | FM.
        &[433_920_000, 315_000_000]
    }

    fn reset(&mut self) {
        self.step = Step::Reset;
        self.preamble_count = 0;
        self.all_bit_count = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let is_short = duration_diff!(duration, TE_SHORT) <= TE_DELTA;
        match self.step {
            Step::Reset => {
                if is_short {
                    self.step = Step::Preamble;
                    self.preamble_count = 1;
                }
                None
            }
            Step::Preamble => {
                if is_short {
                    self.preamble_count += 1;
                    if self.preamble_count > 5000 {
                        self.step = Step::Reset;
                    }
                } else if self.preamble_count >= MIN_PREAMBLE {
                    // The first non-SHORT pulse after enough preamble starts the NRZ data.
                    self.all_bit_count = 0;
                    let n = ((duration + TE_SHORT / 2) / TE_SHORT).min(8);
                    self.append_n(n, level);
                    self.step = Step::Collect;
                } else {
                    self.step = Step::Reset;
                }
                None
            }
            Step::Collect => {
                let n = (duration + TE_SHORT / 2) / TE_SHORT;
                if n == 0 || duration > TE_SHORT * 8 {
                    return self.finish();
                }
                self.append_n(n, level);
                if self.all_bit_count >= OVERFLOW {
                    return self.finish();
                }
                None
            }
        }
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Test vectors: made-up field values, built as pulse sequences the way the C expects them, written
    // as Flipper RAW files and decoded by the FoxFW C decoder (host oracle, `--registry all
    // --protocol "Renault V3"`); the expectations below are the fields the oracle printed. Levels
    // alternate from `.0`.

    /// `rv3_a`: first pulse HIGH, 56 pulses.
    const RV3_A: (bool, &[u32]) = (
        true,
        &[
        52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 104, 104, 52, 104,
        52, 52, 104, 52, 52, 104, 52, 104, 156, 52, 104, 52, 312, 52, 208, 52,
        104, 364, 52, 52, 104, 52, 52, 104, 208, 52, 52, 52, 156, 104, 52, 52,
        104, 52, 156, 52, 52, 156, 52, 2000,
        ],
    );

    /// `rv3_b`: first pulse HIGH, 52 pulses.
    const RV3_B: (bool, &[u32]) = (
        true,
        &[
        52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 104, 104, 52, 52,
        104, 104, 52, 52, 104, 52, 104, 104, 52, 156, 104, 52, 52, 104, 52, 260,
        52, 104, 104, 104, 52, 52, 104, 52, 52, 104, 52, 104, 52, 52, 156, 52,
        104, 104, 52, 2000,
        ],
    );

    /// `rv3_long`: first pulse HIGH, 102 pulses.
    const RV3_LONG: (bool, &[u32]) = (
        true,
        &[
        52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 104, 104, 52, 104,
        52, 52, 104, 52, 52, 104, 52, 52, 104, 52, 104, 52, 52, 104, 52, 208,
        52, 52, 52, 416, 52, 104, 156, 104, 52, 156, 104, 260, 52, 52, 104, 52,
        52, 104, 104, 52, 104, 104, 52, 104, 52, 156, 52, 208, 52, 52, 104, 52,
        104, 52, 52, 104, 156, 52, 52, 104, 208, 104, 104, 52, 52, 104, 104, 52,
        52, 104, 52, 52, 52, 52, 260, 52, 52, 52, 104, 52, 52, 52, 104, 156,
        208, 260, 52, 104, 104, 2000,
        ],
    );

    /// `rv3_degenerate`: first pulse HIGH, 67 pulses.
    const RV3_DEGENERATE: (bool, &[u32]) = (
        true,
        &[
        52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 104, 104, 52, 104,
        52, 52, 104, 52, 52, 104, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52,
        52, 52, 52, 52, 52, 52, 104, 52, 104, 52, 104, 52, 52, 52, 156, 104,
        156, 52, 52, 52, 104, 104, 52, 104, 52, 104, 52, 52, 104, 260, 208, 104,
        52, 52, 2000,
        ],
    );

    /// `rv3_bad_sync`: first pulse HIGH, 56 pulses.
    const RV3_BAD_SYNC: (bool, &[u32]) = (
        true,
        &[
        52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 104, 104, 52, 104,
        52, 52, 104, 52, 52, 52, 104, 104, 156, 52, 104, 52, 312, 52, 208, 52,
        104, 364, 52, 52, 104, 52, 52, 104, 208, 52, 52, 52, 156, 104, 52, 52,
        104, 52, 156, 52, 52, 156, 52, 2000,
        ],
    );

    /// `rv3_short`: first pulse HIGH, 46 pulses.
    const RV3_SHORT: (bool, &[u32]) = (
        true,
        &[
        52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 104, 104, 52, 104,
        52, 52, 104, 52, 52, 104, 52, 104, 156, 52, 104, 52, 312, 52, 208, 52,
        104, 364, 52, 52, 104, 52, 52, 104, 208, 52, 52, 52, 104, 2000,
        ],
    );

    /// `rv3_toolong`: first pulse HIGH, 94 pulses.
    const RV3_TOOLONG: (bool, &[u32]) = (
        true,
        &[
        52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 104, 104, 52, 104,
        52, 52, 104, 52, 52, 104, 52, 104, 156, 52, 104, 52, 312, 52, 208, 52,
        104, 364, 52, 52, 104, 52, 52, 104, 208, 52, 52, 52, 156, 104, 52, 52,
        104, 52, 156, 52, 52, 156, 52, 104, 104, 52, 52, 52, 104, 208, 52, 52,
        52, 52, 52, 104, 208, 52, 52, 104, 260, 208, 52, 156, 208, 104, 156, 104,
        208, 52, 52, 104, 104, 156, 156, 52, 208, 104, 208, 52, 52, 2000,
        ],
    );

    /// `rv3_nopre`: first pulse HIGH, 52 pulses.
    const RV3_NOPRE: (bool, &[u32]) = (
        true,
        &[
        52, 52, 52, 52, 52, 52, 52, 52, 104, 104, 52, 104, 52, 52, 104, 52,
        52, 104, 52, 104, 156, 52, 104, 52, 312, 52, 208, 52, 104, 364, 52, 52,
        104, 52, 52, 104, 208, 52, 52, 52, 156, 104, 52, 52, 104, 52, 156, 52,
        52, 156, 52, 2000,
        ],
    );

    fn run(v: (bool, &[u32])) -> Vec<DecodedSignal> {
        let mut dec = RenaultV3Decoder::new();
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
        assert_eq!(run(RV3_A), vec![DecodedSignal { serial: Some(0x3b7e), button: None, counter: None, crc_valid: false, data: 0xc9693b7ef602d3d7, data_count_bit: 80, encoder_capable: false, extra: Some(0x2dd1), protocol_display_name: None }], "rv3_a");
        assert_eq!(run(RV3_B), vec![DecodedSignal { serial: Some(0x91a4), button: None, counter: None, crc_valid: false, data: 0xcb2d91a4132d25d9, data_count_bit: 64, encoder_capable: false, extra: Some(0x0), protocol_display_name: None }], "rv3_b");
        assert_eq!(run(RV3_LONG), vec![DecodedSignal { serial: Some(0x6d21), button: None, counter: None, crc_valid: false, data: 0xc9696d2140272305, data_count_bit: 160, encoder_capable: false, extra: Some(0xa6c910b69d3cd34a), protocol_display_name: None }], "rv3_long");
    }

    #[test]
    fn rejects_what_the_c_rejects() {
        assert!(run(RV3_DEGENERATE).is_empty(), "rv3_degenerate: serial 0x5555");
        assert!(run(RV3_BAD_SYNC).is_empty(), "rv3_bad_sync: sync 0xC96B");
        assert!(run(RV3_SHORT).is_empty(), "rv3_short: 63 bits");
        assert!(run(RV3_TOOLONG).is_empty(), "rv3_toolong: 161 bits");
        assert!(run(RV3_NOPRE).is_empty(), "rv3_nopre: 8 preamble pulses");
    }

    #[test]
    fn decode_only() {
        let d = RenaultV3Decoder::new();
        assert!(!d.supports_encoding());
        assert!(d.encode(&DecodedSignal::new(0, 0), 1).is_none());
    }
}
