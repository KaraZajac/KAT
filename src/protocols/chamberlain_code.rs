//! Cham_Code (Chamberlain / LiftMaster 7, 8 and 9 DIP-switch fixed code) decoder + encoder.
//!
//! Port of Flipper-ARF's garage app `applications/system/subghz_garage/protocols/chamberlain_code.c`
//! (the primary reference; doctor-firmware's and ARF's lib copies have the same wire behaviour; no
//! copy can switch buttons).
//!
//! **Air format** (AM, 315 MHz): 1 ms units. Every symbol is a LOW then a HIGH run of 4 units:
//! stop `0001` (LOW 3 ms + HIGH 1 ms), DIP 1 `0011` (2 + 2), DIP 0 `0111` (1 + 3). A frame is
//! - 9 DIP: stop, d8..d0, stop (11 symbols, `0x10000000001` around the DIP nibbles);
//! - 8 DIP: stop, d7..d3, stop, d1, d0, stop (10 symbols — d2, "DIP 6", is not sent);
//! - 7 DIP: stop, d6..d1, stop, stop, d0, stop (11 symbols).
//!
//! Each frame follows 36 ms of silence (the first stop's LOW merges with it: 39 ms).
//!
//! **Decoder** (`subghz_protocol_decoder_chamb_code_feed`): a LOW within 39000 ± 4000 µs, then a
//! 1 ms HIGH (the first stop symbol); each LOW + HIGH pair (± 200 µs) adds a symbol nibble; a LOW
//! longer than 5 ms ends the frame, which with 10..=11 symbols must match one of the three stop-symbol masks
//! (7, then 8, then 9 DIP) and decode every DIP nibble (`check_mask_and_parse`). The decoder then
//! needs a new 39 ms LOW. No fields: `data` is the DIP code (bit i = DIP i; for 8 DIP, bit 2 = 0).
//!
//! **Encoder** (`subghz_protocol_encoder_chamb_code_get_upload`, `encoder.repeat` = 3): 36 zero units
//! of guard + the frame's 44 (7/9 DIP) or 40 (8 DIP) units, run-length coded at 1000 µs per unit
//! (`subghz_protocol_blocks_get_upload_from_bit_array`). Only 7/8/9-bit keys transmit (the C's
//! `get_upload` fails otherwise); `button` is ignored.
//!
//! **Key file**: `Protocol: Cham_Code`, `Bit`, `Key`; loading rejects more than 10 bits, as the C.

use super::g1_common::{fixed_code, generic_fields, generic_key, push, with_repeats, TX_REPEAT};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Cham_Code",
    decoder: "Cham_Code",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Chamberlain/LiftMaster",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "Cham_Code";
const TE_SHORT: u32 = 1000;
const TE_LONG: u32 = 3000;
const TE_DELTA: u32 = 200;
const MIN_COUNT_BIT: usize = 10;

const BIT_STOP: u64 = 0b0001;
const BIT_1: u64 = 0b0011;
const BIT_0: u64 = 0b0111;

const CODE_7_MASK: u64 = 0xF000000FF0F;
const CODE_8_MASK: u64 = 0xF00000F00F;
const CODE_9_MASK: u64 = 0xF000000000F;
const CODE_7_MASK_CHECK: u64 = 0x10000001101;
const CODE_8_MASK_CHECK: u64 = 0x1000001001;
const CODE_9_MASK_CHECK: u64 = 0x10000000001;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    FoundStartBit,
    SaveDuration,
    CheckDuration,
}

/// Cham_Code decoder (`SubGhzProtocolDecoderChamb_Code`).
pub struct ChamberlainCodeDecoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
}

/// `subghz_protocol_chamb_bit_to_code`: one symbol nibble per bit (bit i → nibble i).
fn bit_to_code(data: u64, size: usize) -> u64 {
    (0..size).fold(0u64, |acc, i| {
        acc << 4
            | if (data >> (size - i - 1)) & 1 == 0 {
                BIT_0
            } else {
                BIT_1
            }
    })
}

/// `subghz_protocol_chamb_code_to_bit`: the DIP bits of `size` symbol nibbles, or None when a
/// nibble is not a DIP symbol.
fn code_to_bit(data: u64, size: usize) -> Option<u64> {
    let mut tmp = data;
    let mut res = 0u64;
    for i in 0..size {
        match tmp & 0xF {
            BIT_0 => {}
            BIT_1 => res |= 1 << i,
            _ => return None,
        }
        tmp >>= 4;
    }
    Some(res)
}

/// `subghz_protocol_decoder_chamb_code_check_mask_and_parse`: (DIP code, DIP count).
fn check_mask_and_parse(data: u64, count: usize) -> Option<(u64, usize)> {
    if count > MIN_COUNT_BIT + 1 {
        return None;
    }
    let (symbols, bits) = if data & CODE_7_MASK == CODE_7_MASK_CHECK {
        let d = data & !CODE_7_MASK;
        ((d >> 12) | ((d >> 4) & 0xF), 7)
    } else if data & CODE_8_MASK == CODE_8_MASK_CHECK {
        let d = data & !CODE_8_MASK;
        (d >> 4 | BIT_0 << 8, 8)
    } else if data & CODE_9_MASK == CODE_9_MASK_CHECK {
        ((data & !CODE_9_MASK) >> 4, 9)
    } else {
        return None;
    };
    Some((code_to_bit(symbols, bits)?, bits))
}

/// The 36-unit guard + frame bits of `get_upload` (None for a key that is not 7/8/9 bits).
fn frame_units(data: u64, bits: usize) -> Option<Vec<bool>> {
    let code = bit_to_code(data, bits);
    let (frame, len) = match bits {
        7 => (
            ((code >> 4) << 16) | (code & 0xF) << 4 | CODE_7_MASK_CHECK,
            44,
        ),
        8 => (
            ((code >> 12) << 16) | (code & 0xFF) << 4 | CODE_8_MASK_CHECK,
            40,
        ),
        9 => ((code << 4) | CODE_9_MASK_CHECK, 44),
        _ => return None,
    };
    let mut units = vec![false; 36];
    units.extend((0..len).rev().map(|i| (frame >> i) & 1 == 1));
    Some(units)
}

/// `subghz_protocol_blocks_get_upload_from_bit_array` (left aligned): runs of equal units.
fn build_frame(data: u64, bits: usize) -> Option<Vec<LevelDuration>> {
    let mut out = Vec::new();
    for unit in frame_units(data, bits)? {
        push(&mut out, unit, TE_SHORT);
    }
    Some(out)
}

impl ChamberlainCodeDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
        }
    }

    fn add_symbol(&mut self, symbol: u64) {
        self.decode_data = self.decode_data << 4 | symbol;
        self.decode_count_bit = self.decode_count_bit.wrapping_add(1);
    }
}

impl Default for ChamberlainCodeDecoder {
    fn default() -> Self {
        Self::new()
    }
}

fn decoded(data: u64, bits: usize) -> DecodedSignal {
    DecodedSignal {
        encoder_capable: (7..=9).contains(&bits),
        ..fixed_code(data, bits)
    }
}

impl ProtocolDecoder for ChamberlainCodeDecoder {
    fn name(&self) -> &'static str {
        NAME
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
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.step = Step::Reset;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                if !level && duration_diff!(duration, TE_SHORT * 39) < TE_DELTA * 20 {
                    self.step = Step::FoundStartBit;
                }
            }
            Step::FoundStartBit => {
                if level && duration_diff!(duration, TE_SHORT) < TE_DELTA {
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.add_symbol(BIT_STOP);
                    self.step = Step::SaveDuration;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::SaveDuration => {
                if !level {
                    if duration > TE_SHORT * 5 {
                        let mut out = None;
                        if self.decode_count_bit as usize >= MIN_COUNT_BIT {
                            if let Some((code, bits)) = check_mask_and_parse(
                                self.decode_data,
                                self.decode_count_bit as usize,
                            ) {
                                out = Some(decoded(code, bits));
                            }
                        }
                        self.step = Step::Reset;
                        return out;
                    }
                    self.te_last = duration;
                    self.step = Step::CheckDuration;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::CheckDuration => {
                if level {
                    if duration_diff!(self.te_last, TE_SHORT * 3) < TE_DELTA
                        && duration_diff!(duration, TE_SHORT) < TE_DELTA
                    {
                        self.add_symbol(BIT_STOP);
                        self.step = Step::SaveDuration;
                    } else if duration_diff!(self.te_last, TE_SHORT * 2) < TE_DELTA
                        && duration_diff!(duration, TE_SHORT * 2) < TE_DELTA
                    {
                        self.add_symbol(BIT_1);
                        self.step = Step::SaveDuration;
                    } else if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA
                        && duration_diff!(duration, TE_SHORT * 3) < TE_DELTA
                    {
                        self.add_symbol(BIT_0);
                        self.step = Step::SaveDuration;
                    } else {
                        self.step = Step::Reset;
                    }
                } else {
                    self.step = Step::Reset;
                }
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit > MIN_COUNT_BIT {
            return None;
        }
        Some(with_repeats(
            &build_frame(decoded.data, decoded.data_count_bit)?,
            TX_REPEAT,
        ))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        let (bits, data) = generic_fields(key);
        (bits <= MIN_COUNT_BIT).then(|| decoded(data, bits))
    }

    fn to_flipper_key(&self, d: &DecodedSignal) -> Option<FlipperKey> {
        Some(generic_key(NAME, d))
    }
}

#[cfg(test)]
mod tests {
    use super::super::g1_common::test_support::*;
    use super::*;

    fn on_air(data: u64, bits: usize) -> Vec<LevelDuration> {
        let mut wire = ChamberlainCodeDecoder::new()
            .encode(&decoded(data, bits), 0)
            .unwrap();
        push(&mut wire, false, 40_000);
        wire
    }

    #[test]
    fn symbol_layout() {
        // 9 DIP 0b1_0110_0101: stop, d8..d0, stop.
        let units = frame_units(0b1_0110_0101, 9).unwrap();
        let tail: String = units[36..]
            .iter()
            .map(|&u| if u { '1' } else { '0' })
            .collect();
        assert_eq!(tail, "00010011011100110011011101110011011100110001");
        // 8 DIP: bit 2 is not sent and decodes as 0.
        assert_eq!(check_mask_and_parse(0x0137_3731_3731, 10), None);
        let wire = on_air(0b1111_1111, 8);
        let got = feed_all(&mut ChamberlainCodeDecoder::new(), &wire);
        assert_eq!((got[0].data, got[0].data_count_bit), (0b1111_1011, 8));
    }

    /// 3 repeats + trailing silence: the first frame decodes at the next frame's guard, the reset
    /// needs a new 39 ms LOW (frame 2's pulses are skipped), frame 3 decodes at the silence.
    #[test]
    fn encode_decode_roundtrip() {
        for (data, bits) in [
            (0b101_1010u64, 7),
            (0b0110_0011, 8),
            (0b1_0110_0101, 9),
            (0, 9),
            (0x7F, 7),
        ] {
            let got = feed_all(&mut ChamberlainCodeDecoder::new(), &on_air(data, bits));
            assert_eq!(got.len(), 2, "{bits}-bit {data:b}");
            assert!(got
                .iter()
                .all(|d| (d.data, d.data_count_bit) == (data, bits) && d.encoder_capable));
        }
        assert!(ChamberlainCodeDecoder::new()
            .encode(&decoded(0x3FF, 10), 0)
            .is_none());
    }

    #[test]
    fn rejects_broken_frames() {
        // A symbol of LOW 1.5 ms + HIGH 2.5 ms is none of the three.
        let mut wire = on_air(0b1_0110_0101, 9);
        wire[3] = LevelDuration::new(wire[3].level, 1500);
        assert_eq!(
            feed_all(&mut ChamberlainCodeDecoder::new(), &wire).len(),
            1,
            "frame 1 lost"
        );
        // Without the closing stop symbol the masks do not match.
        let units = frame_units(0b1_0110_0101, 9).unwrap();
        let mut short = Vec::new();
        for u in &units[..units.len() - 4] {
            push(&mut short, *u, TE_SHORT);
        }
        push(&mut short, false, 40_000);
        assert!(feed_all(&mut ChamberlainCodeDecoder::new(), &short).is_empty());
    }

    #[test]
    fn key_file_roundtrip() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 315000000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Cham_Code\nBit: 9\n\
                    Key: 00 00 00 00 00 00 01 65\n";
        let dec = ChamberlainCodeDecoder::new();
        let d = dec
            .from_flipper_key(&FlipperKey::parse(text).unwrap())
            .unwrap();
        assert_eq!((d.data, d.data_count_bit), (0x165, 9));
        let written = dec.to_flipper_key(&d).unwrap().to_text();
        assert!(written.contains("Protocol: Cham_Code\nBit: 9\nKey: 00 00 00 00 00 00 01 65\n"));
        let back = dec
            .from_flipper_key(&FlipperKey::parse(&written).unwrap())
            .unwrap();
        assert_eq!(format!("{back:?}"), format!("{d:?}"));
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&text.replace("Bit: 9", "Bit: 11")).unwrap())
            .is_none());
    }
}
