//! Honda V4 protocol decoder (**decode only**, as the reference)
//!
//! Ported from FoxFW `lib/subghz/protocols/honda_v4.c` and `honda_v4.h` (itself a port of the
//! KeeLoq-RX `arke_honda.py` `HondaV4` class); no other reference carries it. The C has no encoder
//! (`.encoder = NULL`), so neither does KAT.
//!
//! **Air format** (AM/OOK, 315 / 433.92 MHz): PWM where the bit is carried by the width of the LOW
//! pulse (750 µs = 0, 1100 µs = 1, tolerance below 200 µs) and every bit ends with one clocking
//! HIGH pulse. A burst starts with at least three cycles of ~1500 µs LOW + 1500 µs HIGH
//! (tolerance below 300 µs).
//!
//! **Decoder** (`feed`): Reset waits for a ~1500 µs LOW; `PreambleHigh` wants the matching HIGH
//! and counts it; `PreambleOrDataLow` takes another ~1500 µs LOW back to `PreambleHigh`, or, once
//! three preamble HIGHs were counted, a short/long LOW as the first data bit. 51 bits are read.
//! A frame is also emitted when the data ends early on a gap or on the next preamble LOW
//! (`DataLow`), in which case the bits read so far must be at least 48, and the C still reports
//! 51 bits.
//!
//! **Frame** (51 bits): `serial(19) | button(8) | inv(8) | counter(16)`. The only check is the
//! inverse-byte invariant `inv == 0xFF - button` (bits 23..16 against bits 31..24), the "CRC" of
//! this protocol; emission is gated on it. The data word is the 51 bits as read.
//!
//! **KAT fields**: `data` = the 51-bit word, `data_count_bit` = 51, `serial`, `button` and
//! `counter` as above, `crc_valid` true (every emitted frame passed the invariant). Key files
//! (`serialize` / `deserialize`): Protocol "Honda V4", Bit 51, Key, Serial, Btn, Cnt.
//!
//! No capture of this protocol exists in the test corpus: the port is validated against the C
//! with synthetic pulse sequences only (see the tests and `docs/honda_v4.md`).

use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (see `info.rs`): the C's `SubGhzProtocolFlag_433 | 315 | AM`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "Honda/Acura",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "Honda V4";
const TE_SHORT: u32 = 750;
const TE_LONG: u32 = 1100;
const TE_DELTA: u32 = 200;
const PRE: u32 = 1500;
const PRE_D: u32 = 300;
const BITS: u32 = 51;
const MIN_BITS: u32 = 48;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    PreambleHigh,
    PreambleOrDataLow,
    DataHigh,
    DataLow,
}

/// Honda V4 decoder (matches SubGhzProtocolDecoderHondaV4).
pub struct HondaV4Decoder {
    step: Step,
    pre: u32,
    te_last: u32,
    data: u64,
    cnt_bits: u32,
}

/// `honda_v4_validate`: enough bits and the inverse-byte invariant.
fn validate(data: u64, nbits: u32) -> bool {
    if nbits < MIN_BITS {
        return false;
    }
    let btn = (data >> 24) as u8;
    let inv = (data >> 16) as u8;
    inv == 0xFF - btn
}

/// The fields of a 51-bit word (`honda_v4_emit` / `deserialize`).
fn fields(data: u64) -> DecodedSignal {
    DecodedSignal {
        serial: Some(((data >> 32) & 0x7FFFF) as u32),
        button: Some((data >> 24) as u8),
        counter: Some(data as u16),
        crc_valid: true,
        data,
        data_count_bit: BITS as usize,
        encoder_capable: false,
        extra: None,
        protocol_display_name: None,
    }
}

impl HondaV4Decoder {
    pub fn new() -> Self {
        Self { step: Step::Reset, pre: 0, te_last: 0, data: 0, cnt_bits: 0 }
    }

    /// `honda_v4_emit`.
    fn emit(data: u64, nbits: u32) -> Option<DecodedSignal> {
        validate(data, nbits).then(|| fields(data))
    }
}

impl Default for HondaV4Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for HondaV4Decoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: TE_SHORT, te_long: TE_LONG, te_delta: TE_DELTA, min_count_bit: BITS as usize }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.step = Step::Reset;
        self.pre = 0;
        self.te_last = 0;
        self.data = 0;
        self.cnt_bits = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let is_pre = duration.abs_diff(PRE) < PRE_D;
        let is_s = duration.abs_diff(TE_SHORT) < TE_DELTA;
        let is_l = duration.abs_diff(TE_LONG) < TE_DELTA;
        let mut out = None;

        match self.step {
            Step::Reset => {
                if !level && is_pre {
                    self.pre = 0;
                    self.data = 0;
                    self.cnt_bits = 0;
                    self.step = Step::PreambleHigh;
                }
            }
            Step::PreambleHigh => {
                if level && is_pre {
                    self.pre += 1;
                    self.step = Step::PreambleOrDataLow;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::PreambleOrDataLow => {
                if !level {
                    if is_pre {
                        self.step = Step::PreambleHigh;
                    } else if self.pre >= 3 && (is_s || is_l) {
                        self.te_last = duration;
                        self.data = 0;
                        self.cnt_bits = 0;
                        self.step = Step::DataHigh;
                    } else {
                        self.step = Step::Reset;
                    }
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::DataHigh => {
                if level {
                    let bit = u64::from(self.te_last.abs_diff(TE_LONG) < TE_DELTA);
                    self.data = (self.data << 1) | bit;
                    self.cnt_bits += 1;
                    if self.cnt_bits >= BITS {
                        out = Self::emit(self.data, self.cnt_bits);
                        self.step = Step::Reset;
                    } else {
                        self.step = Step::DataLow;
                    }
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::DataLow => {
                if !level {
                    if is_s || is_l {
                        self.te_last = duration;
                        self.step = Step::DataHigh;
                    } else if is_pre {
                        out = Self::emit(self.data, self.cnt_bits);
                        self.pre = 0;
                        self.data = 0;
                        self.cnt_bits = 0;
                        self.step = Step::PreambleHigh;
                    } else {
                        out = Self::emit(self.data, self.cnt_bits);
                        self.step = Step::Reset;
                    }
                } else {
                    self.step = Step::Reset;
                }
            }
        }
        out
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }

    /// `subghz_protocol_decoder_honda_v4_deserialize`: the 51-bit key, re-checked.
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != BITS as usize {
            return None;
        }
        let data = key.key_u64()?;
        validate(data, BITS).then(|| fields(data))
    }

    /// `subghz_protocol_decoder_honda_v4_serialize`.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != BITS as usize {
            return None;
        }
        let mut key = FlipperKey::for_protocol(NAME, BITS as usize, decoded.data);
        key.push_u32("Serial", decoded.serial?);
        key.push_u32("Btn", decoded.button?.into());
        key.push_u32("Cnt", decoded.counter?.into());
        Some(key)
    }
}

#[cfg(test)]
mod tests {
    use super::super::honda_synth::{decode, dump, Pulse};
    use super::*;

    /// Pulses of one burst: `presyms` preamble cycles, then each bit as LOW (750 / 1100) + HIGH
    /// 750 µs, and a closing gap.
    fn burst(data: u64, bits: u32, presyms: usize) -> Vec<Pulse> {
        let mut p: Vec<Pulse> = Vec::new();
        for _ in 0..presyms {
            p.push((false, 1500));
            p.push((true, 1500));
        }
        for i in (0..bits).rev() {
            p.push((false, if (data >> i) & 1 == 1 { 1100 } else { 750 }));
            p.push((true, 750));
        }
        p.push((false, 9000));
        p
    }

    /// serial(19) | button | !button | counter.
    fn word(serial: u32, btn: u8, cnt: u16) -> u64 {
        (u64::from(serial) << 32) | (u64::from(btn) << 24) | (u64::from(0xFF - btn) << 16) | u64::from(cnt)
    }

    /// Made-up values; the oracle (subghz_oracle_foxfw, Honda V4) reported
    /// `Key: 07 12 34 5C A3 12 A5 00`-style frames with the same Serial / Btn / Cnt.
    #[test]
    fn honda_v4_decodes_a_synthetic_frame() {
        let w = word(0x1_2345, 0x5C, 0x12A5);
        let pulses = burst(w, 51, 4);
        dump("honda_v4_good", &pulses, "FuriHalSubGhzPresetOok650Async");
        let got = decode(&mut HondaV4Decoder::new(), &pulses);
        assert_eq!(got.len(), 1);
        let d = &got[0];
        assert_eq!((d.serial, d.button, d.counter), (Some(0x1_2345), Some(0x5C), Some(0x12A5)));
        assert_eq!((d.data, d.data_count_bit, d.crc_valid), (w, 51, true));
        assert_eq!(d.data, 0x0001_2345_5CA3_12A5);
    }

    /// The inverse-byte invariant gates emission; so do a short preamble, a wrong preamble pulse
    /// and a frame cut below 48 bits.
    #[test]
    fn honda_v4_gating() {
        let w = word(0x1_2345, 0x5C, 0x12A5);
        let mut bad = burst(w ^ (1 << 20), 51, 4);
        dump("honda_v4_bad_inverse", &bad, "FuriHalSubGhzPresetOok650Async");
        assert!(decode(&mut HondaV4Decoder::new(), &bad).is_empty());
        bad = burst(w, 51, 2);
        dump("honda_v4_short_preamble", &bad, "FuriHalSubGhzPresetOok650Async");
        assert!(decode(&mut HondaV4Decoder::new(), &bad).is_empty());
        let mut wrong_sync = burst(w, 51, 4);
        wrong_sync[6] = (false, 2000); // 4th low of the preamble is neither 1500 nor a bit width
        dump("honda_v4_wrong_sync", &wrong_sync, "FuriHalSubGhzPresetOok650Async");
        assert!(decode(&mut HondaV4Decoder::new(), &wrong_sync).is_empty());
        let short = burst(w, 44, 4);
        dump("honda_v4_short_frame", &short, "FuriHalSubGhzPresetOok650Async");
        assert!(decode(&mut HondaV4Decoder::new(), &short).is_empty());
    }

    /// A frame of 48..50 bits ended by a gap is still emitted, reported as 51 bits (as the C).
    #[test]
    fn honda_v4_early_end_reports_51_bits() {
        // 48 bits only: the C checks the invariant on the 48-bit word, which therefore must hold
        // there (bits 23..16 of the 48-bit word are 0xFF - bits 31..24).
        let w48 = (word(0x1_2345, 0x5C, 0x12A5)) & 0xFFFF_FFFF_FFFF;
        let pulses = burst(w48, 48, 4);
        dump("honda_v4_48bit", &pulses, "FuriHalSubGhzPresetOok650Async");
        let got = decode(&mut HondaV4Decoder::new(), &pulses);
        assert_eq!(got.len(), 1);
        assert_eq!((got[0].data, got[0].data_count_bit), (w48, 51));
    }

    #[test]
    fn honda_v4_key_file_round_trip() {
        use crate::protocols::flipper_key::testing::assert_round_trip;
        let d = fields(word(0x7_FFFF, 0x01, 0xFFFF));
        let text = assert_round_trip(&HondaV4Decoder::new(), &d);
        assert!(text.contains("Protocol: Honda V4\nBit: 51\nKey: 00 07 FF FF 01 FE FF FF\n"), "{text}");
        assert!(!HondaV4Decoder::new().supports_encoding());
    }
}
