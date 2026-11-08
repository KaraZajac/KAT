//! BMW V2 (FoxFW `bmw_v2.c`, `BMW_V2_PROTOCOL_NAME = "BMW V2"`): AM Manchester BMW key fob, 56 bits
//! — **decode only**, as the reference (its encoder is NULL).
//!
//! Ported from FoxFW (itself a port of the firmware's `bmw_v2.c` via KeeLoq-RX `arke_mitsu_bmw.py`).
//! It is not KAT's "BMW" module (`bmw.rs`, Flipper-ARF's 62 µs 128-bit frame): this one is
//! 500 / 1000 µs, 56 bits, and bmw.rs treats every pulse of 400 µs or more as a gap, so the two read
//! different frames.
//!
//! **Air format** (OOK, 315 / 433 MHz): te 500 / 1000 µs (±150); a LOW of more than 1700 µs is the
//! inter-frame gap. A half-period accumulator (`te_last`): a long pulse is one bit, two short pulses
//! are one bit; the bit's value is the inverse of the completing pulse's level (`level ? 0 : 1`).
//! A long pulse arriving while a short is pending completes a bit and leaves the third half pending
//! (`te_last = te_short`). The first pulse after the gap only primes the accumulator.
//!
//! **Frame** (56 bits): `header 0xCF | rolling:40 | trailer 0x3A`. The header and trailer are fixed
//! framing bytes, not a checksum: they gate the frame (so there is no CRC and `crc_valid` stays
//! false). The "serial" is carved from the encrypted rolling code (its upper 24 bits) and the
//! counter is its lower 16 bits, so both change with every press; a serial of 0x000000, 0xFFFFFF,
//! 0x555555 or 0xAAAAAA is rejected.
//!
//! **Decoder** (`subghz_protocol_decoder_bmw_v2_feed`): a frame is evaluated when a pulse that is
//! neither short nor long arrives with 56 or more bits collected (the last 56 are used), or as soon
//! as more than 64 bits are in.
//!
//! **KAT fields**: `data` = the 56-bit frame, `data_count_bit` = 56, `serial` = the 24-bit carved
//! value, `counter` = the 16-bit carved value, `button` None, `crc_valid` false.
//!
//! No deviation from the C. IMPORTS holds no BMW V2 capture (the oracle finds none).

use super::foxfw_common::duration_diff;
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (see `info.rs`): flags 433 | 315 | AM, dynamic.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Encrypted (rolling)",
    make: "BMW",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "BMW V2";
const TE_SHORT: u32 = 500;
const TE_LONG: u32 = 1000;
const TE_DELTA: u32 = 150;
const DATA_BITS: u8 = 56;
const GAP_MIN: u32 = 1700;
const HEADER: u8 = 0xCF;
const TRAILER: u8 = 0x3A;
const DEGENERATE: [u32; 4] = [0x000000, 0xFFFFFF, 0x555555, 0xAAAAAA];

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    WaitGap,
    Manchester,
}

/// BMW V2 decoder (`SubGhzProtocolDecoderBmwV2`).
pub struct BmwV2Decoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
}

impl BmwV2Decoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
        }
    }

    /// `bmw_v2_try_emit`: right-align to 56 bits, gate on the header and trailer, carve the fields.
    fn try_emit(&self) -> Option<DecodedSignal> {
        if self.decode_count_bit < DATA_BITS {
            return None;
        }
        let mut data = self.decode_data;
        if self.decode_count_bit > DATA_BITS {
            data >>= self.decode_count_bit - DATA_BITS;
        }
        let header = ((data >> 48) & 0xFF) as u8;
        let trailer = (data & 0xFF) as u8;
        if header != HEADER || trailer != TRAILER {
            return None;
        }
        let serial = (((data >> 8) & 0xFF_FFFF_FFFF) >> 16) as u32 & 0xFF_FFFF;
        if DEGENERATE.contains(&serial) {
            return None;
        }
        Some(signal(data & 0x00FF_FFFF_FFFF_FFFF))
    }

    /// The start of a frame: wait for the first pulse after the gap.
    fn start_gap(&mut self) {
        self.step = Step::WaitGap;
        self.decode_data = 0;
        self.decode_count_bit = 0;
        self.te_last = 0;
    }

    fn add_bit(&mut self, level: bool) {
        self.decode_data = (self.decode_data << 1) | u64::from(!level);
        self.decode_count_bit = self.decode_count_bit.wrapping_add(1);
    }
}

/// A decode of the 56-bit frame: the carved 24-bit serial and 16-bit counter (of the rolling code).
fn signal(data: u64) -> DecodedSignal {
    let rolling = (data >> 8) & 0xFF_FFFF_FFFF;
    DecodedSignal {
        serial: Some(((rolling >> 16) & 0xFF_FFFF) as u32),
        button: None,
        counter: Some((rolling & 0xFFFF) as u16),
        crc_valid: false,
        data,
        data_count_bit: usize::from(DATA_BITS),
        encoder_capable: false,
        extra: None,
        protocol_display_name: None,
    }
}

impl Default for BmwV2Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for BmwV2Decoder {
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
        self.te_last = 0;
        self.decode_data = 0;
        self.decode_count_bit = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                if !level && duration > GAP_MIN {
                    self.start_gap();
                }
            }
            Step::WaitGap => {
                if level {
                    let is_short = duration_diff(duration, TE_SHORT) <= TE_DELTA;
                    let is_long = duration_diff(duration, TE_LONG) <= TE_DELTA;
                    if is_short || is_long {
                        self.te_last = duration;
                        self.step = Step::Manchester;
                    } else {
                        self.step = Step::Reset;
                    }
                } else if duration <= GAP_MIN {
                    // (another gap keeps waiting)
                    self.step = Step::Reset;
                }
            }
            Step::Manchester => {
                let is_short = duration_diff(duration, TE_SHORT) <= TE_DELTA;
                let is_long = duration_diff(duration, TE_LONG) <= TE_DELTA;
                if !is_short && !is_long {
                    let out = self.try_emit();
                    self.step = Step::Reset;
                    if !level && duration > GAP_MIN {
                        self.start_gap();
                    }
                    return out;
                }
                if is_short {
                    if self.te_last == 0 {
                        self.te_last = duration;
                    } else {
                        self.add_bit(level);
                        self.te_last = 0;
                    }
                } else {
                    self.add_bit(level);
                    self.te_last = if self.te_last != 0 { TE_SHORT } else { 0 };
                }
                if self.decode_count_bit > 64 {
                    let out = self.try_emit();
                    self.step = Step::Reset;
                    return out;
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

    /// `subghz_protocol_decoder_bmw_v2_deserialize`: `Bit: 56` and the key; the fields are carved
    /// from it (the extra lines are not read, and no header / trailer / serial gate is applied).
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        (key.bits == usize::from(DATA_BITS)).then(|| signal(key.key))
    }

    /// `subghz_protocol_decoder_bmw_v2_serialize`: the generic Key, then `Serial`, `Cnt`, `Hdr`, `Trl`.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != usize::from(DATA_BITS) {
            return None;
        }
        let mut key = FlipperKey::for_protocol(NAME, usize::from(DATA_BITS), decoded.data);
        key.push_u32("Serial", decoded.serial?);
        key.push_u32("Cnt", u32::from(decoded.counter?));
        key.push_u32("Hdr", ((decoded.data >> 48) & 0xFF) as u32);
        key.push_u32("Trl", (decoded.data & 0xFF) as u32);
        Some(key)
    }
}

/// Synthetic captures for the tests.
#[cfg(test)]
pub(super) mod testing {
    use super::*;
    use crate::protocols::foxfw_common::test_support::{bits_of, pulses_for_bits};

    /// The decoder's accumulator for the bit builder: `te_last != 0` is `pending`.
    pub fn step(pending: bool, lvl: bool, long: bool) -> Option<(bool, Option<bool>)> {
        let bit = Some(!lvl);
        Some(if long {
            (pending, bit)
        } else if pending {
            (false, bit)
        } else {
            (true, None)
        })
    }

    /// Pulses of a 56-bit frame `0xCF | rolling | 0x3A` (given whole as `frame56`): a 2 ms LOW gap,
    /// a 500 µs HIGH priming pulse, the bits as short/long pulses, a 3 ms LOW (non-matching pulse
    /// that triggers the evaluation).
    pub fn frame(frame56: u64) -> Vec<LevelDuration> {
        frame_bits(&bits_of(frame56, 56))
    }

    /// [frame] for any bit string.
    pub fn frame_bits(bits: &[bool]) -> Vec<LevelDuration> {
        let mut out = vec![
            LevelDuration::new(false, 2_000),
            LevelDuration::new(true, TE_SHORT),
        ];
        out.extend(pulses_for_bits(true, true, bits, TE_SHORT, TE_LONG, step));
        let last = out.last().is_none_or(|p| p.level);
        out.push(LevelDuration::new(!last, 3_000));
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::foxfw_common::test_support::*;

    const FRAME_A: u64 = 0x00CF_9A3C_5E7F_213A;
    const FRAME_B: u64 = 0x00CF_0123_4567_893A;

    /// Made-up frames: (name, pulses, expected (data, serial, counter)).
    #[allow(clippy::type_complexity)]
    fn cases() -> Vec<(&'static str, Vec<LevelDuration>, Option<(u64, u32, u16)>)> {
        // The C right-aligns what it collected to 56 bits, so extra bits belong at the END of the
        // frame (the last ones are dropped); extra bits in front shift the header and fail.
        let mut frame_then_eight = bits_of(FRAME_B, 56);
        frame_then_eight.extend(bits_of(0xA5, 8)); // 64 bits: the most the register holds
        let mut frame_then_nine = bits_of(FRAME_B, 56);
        frame_then_nine.extend(bits_of(0x1A5, 9)); // 65 bits: evaluated at once, the first is lost
        let mut frame_then_four = bits_of(FRAME_A, 56);
        frame_then_four.extend(bits_of(0xA, 4));
        let mut four_then_frame = bits_of(0xA, 4);
        four_then_frame.extend(bits_of(FRAME_A, 56));
        vec![
            ("a", testing::frame(FRAME_A), Some((FRAME_A, 0x9A3C5E, 0x7F21))),
            ("b", testing::frame(FRAME_B), Some((FRAME_B, 0x012345, 0x6789))),
            ("four_trailing_bits", testing::frame_bits(&frame_then_four), Some((FRAME_A, 0x9A3C5E, 0x7F21))),
            ("eight_trailing_bits", testing::frame_bits(&frame_then_eight), Some((FRAME_B, 0x012345, 0x6789))),
            ("nine_trailing_bits", testing::frame_bits(&frame_then_nine), None),
            ("four_leading_bits", testing::frame_bits(&four_then_frame), None),
            ("bad_header", testing::frame(FRAME_A ^ (1 << 48)), None),
            ("bad_trailer", testing::frame(FRAME_A ^ 1), None),
            ("serial_555555", testing::frame(0x00CF_5555_55AA_AA3A), None),
            ("serial_ffffff", testing::frame(0x00CF_FFFF_FF12_343A), None),
            ("serial_aaaaaa", testing::frame(0x00CF_AAAA_AA12_343A), None),
            ("serial_000000", testing::frame(0x00CF_0000_0012_343A), None),
            ("short_frame", testing::frame_bits(&bits_of(FRAME_A >> 1, 55)), None),
        ]
    }

    #[test]
    fn key_file_round_trip_and_decode_only() {
        let dec = BmwV2Decoder::new();
        let d = signal(FRAME_A);
        let text = crate::protocols::flipper_key::testing::assert_round_trip(&dec, &d);
        // the lines the C's serialize writes (checked on the oracle's output)
        assert!(text.contains("Protocol: BMW V2\nBit: 56\nKey: 00 CF 9A 3C 5E 7F 21 3A\nSerial: 10107998\nCnt: 32545\nHdr: 207\nTrl: 58\n"), "{text}");
        let mut wrong_bits = FlipperKey::parse(&text).unwrap();
        wrong_bits.bits = 64;
        assert!(dec.from_flipper_key(&wrong_bits).is_none());
        assert!(!dec.supports_encoding() && dec.encode(&d, 1).is_none());
    }

    #[test]
    fn synthetic_frames_match_the_foxfw_oracle() {
        for (name, wire, want) in cases() {
            dump(&format!("bmw_v2_{name}.sub"), 433_920_000, "FuriHalSubGhzPresetOok650Async", &wire);
            let got = decode_all(&mut BmwV2Decoder::new(), &wire);
            eprintln!("CASE {name}: {:?}", got.iter().map(|d| (format!("{:016X}", d.data), d.serial, d.counter)).collect::<Vec<_>>());
            assert_eq!(got.len(), usize::from(want.is_some()), "{name}");
            if let Some((data, serial, counter)) = want {
                let d = &got[0];
                assert_eq!((d.data, d.serial, d.counter), (data, Some(serial), Some(counter)), "{name}");
                assert_eq!(d.data_count_bit, 56);
            }
        }
    }
}
