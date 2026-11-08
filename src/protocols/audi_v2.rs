//! Audi V2 (FoxFW `audi_v2.c`, `AUDI_V2_PROTOCOL_NAME = "Audi V2"`): AM Manchester VAG key fob —
//! **decode only**, as the reference (its encoder is NULL).
//!
//! Ported from FoxFW (itself a port of the firmware's `audi_v2.c` via KeeLoq-RX `arke_vag.py`).
//! **Frame** (OOK, 315 / 433 MHz): at least 40 short pulses (280 µs ±100), then one long pulse
//! (560 µs ±100) that primes the Manchester decoder, then 72 Manchester bits = a 64-bit key and an
//! 8-bit trailer. A pulse that is neither short nor long ends the frame.
//!
//! **Check**: the trailer must be `0x93` (the C's `crc_ok`); frames with another trailer are not
//! reported. The 64-bit body is encrypted, so no serial, button or counter exists.
//!
//! **Decoder** (`subghz_protocol_decoder_audi_v2_feed`): Reset: any short pulse starts the preamble
//! count. Preamble: shorts are counted; a long pulse with a count of 40 or more primes
//! `manchester_advance(Mid1, Long{High,Low})` and starts the data; anything else resets. Data:
//! each pulse is a Manchester event (`level ? ShortHigh/LongHigh : ShortLow/LongLow`) for the
//! toolbox decoder ([super::common::manchester_advance]); the first 64 bits are the key, the next 8
//! the trailer; an invalid pulse resets.
//!
//! **KAT fields**: `data` = the 64-bit key, `data_count_bit` = 72, `extra` = the trailer (0x93),
//! `crc_valid` = true (the trailer gate), `serial`/`button`/`counter` None.
//!
//! No deviation from the C. The only IMPORTS capture that decodes is a 315 MHz VW Atlas recording
//! with a single frame (see `docs/audi_v2.md`).

use super::common::{manchester_advance, ManchesterEvent, ManchesterState};
use super::foxfw_common::duration_diff;
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (see `info.rs`): `SubGhzProtocolFlag_433 | _315 | _AM`, dynamic, encrypted body.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Encrypted (rolling)",
    make: "Audi",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "Audi V2";
const TE_SHORT: u32 = 280;
const TE_LONG: u32 = 560;
const TE_DELTA: u32 = 100;
const PREAMBLE: u16 = 40;
const DATA_BITS: u16 = 72;
const TRAILER: u8 = 0x93;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    Preamble,
    Data,
}

/// Audi V2 decoder (`SubGhzProtocolDecoderAudiV2`).
pub struct AudiV2Decoder {
    step: Step,
    preamble_count: u16,
    state: ManchesterState,
    gdata: u64,
    acc: u64,
    bit_count: u16,
}

/// `audi_v2_event`: the Manchester event of a pulse, None (the C's `ManchesterEventReset`) if it
/// is neither short nor long.
fn event(duration: u32, level: bool) -> Option<ManchesterEvent> {
    if duration_diff(duration, TE_SHORT) <= TE_DELTA {
        return Some(ManchesterEvent::from_half_bit(level, false));
    }
    if duration_diff(duration, TE_LONG) <= TE_DELTA {
        return Some(ManchesterEvent::from_half_bit(level, true));
    }
    None
}

impl AudiV2Decoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            preamble_count: 0,
            state: ManchesterState::Mid1,
            gdata: 0,
            acc: 0,
            bit_count: 0,
        }
    }

    /// `audi_v2_complete`: the frame when the trailer is 0x93.
    fn complete(&self) -> Option<DecodedSignal> {
        if self.bit_count < DATA_BITS {
            return None;
        }
        let trailer = (self.acc & 0xFF) as u8;
        if trailer != TRAILER {
            return None;
        }
        Some(signal(self.gdata, trailer))
    }
}

/// A decode of the 64-bit key and its trailer (`crc_valid` = the trailer is 0x93).
fn signal(key: u64, trailer: u8) -> DecodedSignal {
    DecodedSignal {
        serial: None,
        button: None,
        counter: None,
        crc_valid: trailer == TRAILER,
        data: key,
        data_count_bit: usize::from(DATA_BITS),
        encoder_capable: false,
        extra: Some(u64::from(trailer)),
        protocol_display_name: None,
    }
}

impl Default for AudiV2Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for AudiV2Decoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: usize::from(DATA_BITS),
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.step = Step::Reset;
        self.preamble_count = 0;
        self.gdata = 0;
        self.acc = 0;
        self.bit_count = 0;
        self.state = ManchesterState::Mid1;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                if duration_diff(duration, TE_SHORT) <= TE_DELTA {
                    self.preamble_count = 1;
                    self.step = Step::Preamble;
                }
            }
            Step::Preamble => {
                if duration_diff(duration, TE_SHORT) <= TE_DELTA {
                    self.preamble_count = self.preamble_count.wrapping_add(1);
                } else if duration_diff(duration, TE_LONG) <= TE_DELTA
                    && self.preamble_count >= PREAMBLE
                {
                    self.step = Step::Data;
                    self.acc = 0;
                    self.bit_count = 0;
                    self.gdata = 0;
                    // prime Manchester with the long pulse that ended the preamble
                    self.state = manchester_advance(
                        ManchesterState::Mid1,
                        ManchesterEvent::from_half_bit(level, true),
                    )
                    .0;
                } else {
                    self.step = Step::Reset;
                    self.preamble_count = 0;
                }
            }
            Step::Data => {
                let ev = event(duration, level);
                if let Some(ev) = ev {
                    let (next, bit) = manchester_advance(self.state, ev);
                    self.state = next;
                    if let Some(bit) = bit {
                        self.acc = (self.acc << 1) | u64::from(bit);
                        self.bit_count += 1;
                        if self.bit_count == 64 {
                            self.gdata = self.acc;
                            self.acc = 0;
                        }
                    }
                }
                if self.bit_count >= DATA_BITS {
                    let out = self.complete();
                    self.acc = 0;
                    self.bit_count = 0;
                    self.step = Step::Reset;
                    return out;
                }
                if ev.is_none() {
                    self.acc = 0;
                    self.bit_count = 0;
                    self.step = Step::Reset;
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

    /// `subghz_protocol_decoder_audi_v2_deserialize`: `Bit: 72`, the key, and the optional
    /// `Trailer` (0 when absent); the C accepts any trailer, `crc_valid` says whether it is 0x93.
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != usize::from(DATA_BITS) {
            return None;
        }
        let trailer = key.u32_field("Trailer").map_or(0, |t| (t & 0xFF) as u8);
        Some(signal(key.key, trailer))
    }

    /// `subghz_protocol_decoder_audi_v2_serialize`: the generic Key, then `Trailer`.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != usize::from(DATA_BITS) {
            return None;
        }
        let mut key = FlipperKey::for_protocol(NAME, usize::from(DATA_BITS), decoded.data);
        key.push_u32("Trailer", (decoded.extra? & 0xFF) as u32);
        Some(key)
    }
}

/// Synthetic captures for the tests.
#[cfg(test)]
pub(super) mod testing {
    use super::*;
    use crate::protocols::foxfw_common::test_support::manchester_pulses;

    /// Pulses of a frame: `n_short` 280 µs preamble pulses (alternating, starting HIGH), the 560 µs
    /// priming long pulse, the 72 bits (`key`, `trailer`) as the pulses that make the toolbox
    /// Manchester decoder emit them, then a 20 ms LOW.
    pub fn frame(key: u64, trailer: u8, n_short: usize) -> Vec<LevelDuration> {
        let mut out: Vec<LevelDuration> = (0..n_short)
            .map(|k| LevelDuration::new(k % 2 == 0, TE_SHORT))
            .collect();
        let level = n_short.is_multiple_of(2);
        out.push(LevelDuration::new(level, TE_LONG));
        let state =
            manchester_advance(ManchesterState::Mid1, ManchesterEvent::from_half_bit(level, true)).0;
        let bits: Vec<bool> = (0..64)
            .rev()
            .map(|i| (key >> i) & 1 == 1)
            .chain((0..8).rev().map(|i| (trailer >> i) & 1 == 1))
            .collect();
        out.extend(manchester_pulses(state, level, &bits, TE_SHORT, TE_LONG));
        out.push(LevelDuration::new(false, 20_000));
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::foxfw_common::test_support::*;

    const KEY_A: u64 = 0x0123_4567_89AB_CDEF;
    const KEY_B: u64 = 0xA5A5_5A5A_1234_5678;

    /// Made-up frames the FoxFW oracle decodes (name, pulses, expected key) and frames it rejects.
    fn cases() -> Vec<(&'static str, Vec<LevelDuration>, Option<u64>)> {
        let mut bad_sync = testing::frame(KEY_A, TRAILER, 45);
        bad_sync[45].duration_us = 800; // the long pulse that ends the preamble is neither
        let mut short_frame = testing::frame(KEY_A, TRAILER, 45);
        let n = short_frame.len();
        short_frame.drain(n - 13..n - 1); // 12 pulses fewer: the 72 bits never complete
        vec![
            ("a", testing::frame(KEY_A, TRAILER, 45), Some(KEY_A)),
            ("b", testing::frame(KEY_B, TRAILER, 61), Some(KEY_B)),
            ("min_preamble", testing::frame(KEY_B, TRAILER, 40), Some(KEY_B)),
            ("bad_trailer", testing::frame(KEY_A, TRAILER ^ 0x01, 45), None),
            ("short_preamble", testing::frame(KEY_A, TRAILER, 39), None),
            ("bad_sync", bad_sync, None),
            ("short_frame", short_frame, None),
        ]
    }

    #[test]
    fn key_file_round_trip_and_decode_only() {
        let dec = AudiV2Decoder::new();
        let text = crate::protocols::flipper_key::testing::assert_round_trip(&dec, &signal(KEY_A, TRAILER));
        // the lines the C's serialize writes (checked on the oracle's output)
        assert!(text.contains("Protocol: Audi V2\nBit: 72\nKey: 01 23 45 67 89 AB CD EF\nTrailer: 147\n"), "{text}");
        // the C loads any trailer; only 0x93 counts as the check
        let mut other = signal(KEY_A, 0x00);
        other.crc_valid = false;
        assert_eq!(crate::protocols::flipper_key::testing::assert_round_trip(&dec, &other).matches("Trailer: 0").count(), 1);
        let mut wrong_bits = FlipperKey::parse(&text).unwrap();
        wrong_bits.bits = 64;
        assert!(dec.from_flipper_key(&wrong_bits).is_none());
        assert!(!dec.supports_encoding() && dec.encode(&signal(KEY_A, TRAILER), 1).is_none());
    }

    #[test]
    fn synthetic_frames_match_the_foxfw_oracle() {
        for (name, wire, want) in cases() {
            dump(&format!("audi_v2_{name}.sub"), 433_920_000, "FuriHalSubGhzPresetOok650Async", &wire);
            let got = decode_all(&mut AudiV2Decoder::new(), &wire);
            eprintln!("CASE {name}: {:?}", got.iter().map(|d| (format!("{:016X}", d.data), d.extra)).collect::<Vec<_>>());
            assert_eq!(got.len(), usize::from(want.is_some()), "{name}");
            if let Some(key) = want {
                assert_eq!(got[0].data, key, "{name}");
            }
        }
    }
}

