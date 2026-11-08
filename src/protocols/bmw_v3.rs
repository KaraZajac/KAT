//! BMW/Mini V3 (FoxFW `bmw_v3.c`, `BMW_V3_PROTOCOL_NAME = "BMW/Mini V3"`): 2-FSK BMW / Mini key fob,
//! 0xCC header + 64 bits — **decode only**, as the reference (its encoder is NULL).
//!
//! Ported from FoxFW (itself a port of the firmware's `bmw_v3.c` via KeeLoq-RX `arke_bmw.py`). It
//! is not KAT's "BMW" module (`bmw.rs`, Flipper-ARF's 62 µs AM frame with the 0x599E sync): this
//! one is 95 / 190 µs (±40) 2-FSK with an 8-bit 0xCC header.
//!
//! **Air format** (flags 433 | 315 | AM | FM; meaningful on 2-FSK captures): a LOW of more than
//! 350 µs is the gap before a frame. A half-period accumulator (`te_last`) as in BMW V2: a long
//! pulse is one bit, two short pulses are one bit, a long pulse with a short pending completes a
//! bit and leaves the third half pending; the bit's value is `level ? 0 : 1` of the completing
//! pulse. The first pulse after the gap only primes the accumulator.
//!
//! **Decoder** (`subghz_protocol_decoder_bmw_v3_feed`): the header is located by sliding an 8-bit
//! window over the bits (up to 32 bits; the window needs at least 8) until its low byte is 0xCC;
//! the accumulator is then cleared and the next 64 bits are the payload. A pulse that is neither
//! short nor long resets (and starts a new gap wait when it is a long LOW).
//!
//! **Frame** (64 payload bits, MSB first): bits 63..56 button, 55..40 serial (16 bits), and below
//! them the 40-bit rolling code the C calls AES-encrypted; its "counter" field is bits 31..16 of
//! the payload, cut out of that block as the C's field split does. The frame is reported only for
//! button 0x02, 0x04, 0x08 or 0x10; no check exists.
//!
//! **KAT fields**: `data` = the 64-bit payload, `data_count_bit` = 64, `button` = the button byte,
//! `serial` = the 16-bit serial, `counter` = the 16-bit counter field, `crc_valid` false.
//!
//! No deviation from the C. IMPORTS holds no BMW/Mini V3 capture (the oracle finds none).

use super::foxfw_common::duration_diff;
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (see `info.rs`): flags 433 | 315 | AM | FM, dynamic.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::Both,
    encryption: "AES (rolling)",
    make: "BMW / Mini",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&FM476, &AM650],
};

const NAME: &str = "BMW/Mini V3";
const TE_SHORT: u32 = 95;
const TE_LONG: u32 = 190;
const TE_DELTA: u32 = 40;
const DATA_BITS: u16 = 64;
const HEADER: u8 = 0xCC;
const GAP: u32 = 350;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    WaitGap,
    FindHeader,
    CollectData,
}

/// BMW/Mini V3 decoder (`SubGhzProtocolDecoderBmwV3`).
pub struct BmwV3Decoder {
    step: Step,
    data: u64,
    nbit: u16,
    te_last: u32,
}

/// Valid buttons of the C (`bmw_v3_button_valid`).
fn button_valid(btn: u8) -> bool {
    matches!(btn, 0x02 | 0x04 | 0x08 | 0x10)
}

/// A decode of the 64-bit payload: button, 16-bit serial and the 16-bit counter field.
fn signal(data: u64) -> DecodedSignal {
    DecodedSignal {
        serial: Some(((((data >> 48) & 0xFF) << 8) | ((data >> 40) & 0xFF)) as u32),
        button: Some(((data >> 56) & 0xFF) as u8),
        counter: Some(((data >> 16) & 0xFFFF) as u16),
        crc_valid: false,
        data,
        data_count_bit: usize::from(DATA_BITS),
        encoder_capable: false,
        extra: None,
        protocol_display_name: None,
    }
}

impl BmwV3Decoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            data: 0,
            nbit: 0,
            te_last: 0,
        }
    }

    fn start_gap(&mut self) {
        self.step = Step::WaitGap;
        self.data = 0;
        self.nbit = 0;
        self.te_last = 0;
    }

    /// `bmw_v3_add_bit`: accumulate one bit and drive the header search / payload collection.
    fn add_bit(&mut self, bit: bool) -> Option<DecodedSignal> {
        self.data = (self.data << 1) | u64::from(bit);
        self.nbit = self.nbit.wrapping_add(1);

        if self.step == Step::FindHeader {
            if self.nbit >= 8 && (self.data & 0xFF) as u8 == HEADER {
                self.data = 0;
                self.nbit = 0;
                self.step = Step::CollectData;
            } else if self.nbit > 32 {
                self.step = Step::Reset;
            }
        } else if self.step == Step::CollectData && self.nbit == DATA_BITS {
            let data = self.data;
            let btn = ((data >> 56) & 0xFF) as u8;
            self.step = Step::Reset;
            if button_valid(btn) {
                return Some(signal(data));
            }
        }
        None
    }
}

impl Default for BmwV3Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for BmwV3Decoder {
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
        self.data = 0;
        self.nbit = 0;
        self.te_last = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                if !level && duration > GAP {
                    self.start_gap();
                }
            }
            Step::WaitGap => {
                if level {
                    if duration_diff(duration, TE_SHORT) <= TE_DELTA
                        || duration_diff(duration, TE_LONG) <= TE_DELTA
                    {
                        self.te_last = duration;
                        self.step = Step::FindHeader;
                    } else {
                        self.step = Step::Reset;
                    }
                } else if duration <= GAP {
                    self.step = Step::Reset;
                }
            }
            Step::FindHeader | Step::CollectData => {
                let is_short = duration_diff(duration, TE_SHORT) <= TE_DELTA;
                let is_long = duration_diff(duration, TE_LONG) <= TE_DELTA;
                if !is_short && !is_long {
                    self.step = Step::Reset;
                    if !level && duration > GAP {
                        self.start_gap();
                    }
                    return None;
                }
                if is_short {
                    if self.te_last == 0 {
                        self.te_last = duration;
                    } else {
                        let out = self.add_bit(!level);
                        self.te_last = 0;
                        return out;
                    }
                } else if self.te_last != 0 {
                    let out = self.add_bit(!level);
                    self.te_last = TE_SHORT;
                    return out;
                } else {
                    let out = self.add_bit(!level);
                    self.te_last = 0;
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

    /// `subghz_protocol_decoder_bmw_v3_deserialize`: `Bit: 64` and the key; the fields are carved
    /// from it (the extra lines are not read, and the button whitelist is not applied).
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        (key.bits == usize::from(DATA_BITS)).then(|| signal(key.key))
    }

    /// `subghz_protocol_decoder_bmw_v3_serialize`: the generic Key, then `Serial`, `Btn`, `Cnt`.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != usize::from(DATA_BITS) {
            return None;
        }
        let mut key = FlipperKey::for_protocol(NAME, usize::from(DATA_BITS), decoded.data);
        key.push_u32("Serial", decoded.serial?);
        key.push_u32("Btn", u32::from(decoded.button?));
        key.push_u32("Cnt", u32::from(decoded.counter?));
        Some(key)
    }
}

/// Synthetic captures for the tests.
#[cfg(test)]
pub(super) mod testing {
    use super::*;
    use crate::protocols::foxfw_common::test_support::{bits_of, pulses_for_bits};

    /// Pulses of `0xCC` + a 64-bit payload: a 500 µs LOW gap, a 95 µs HIGH priming pulse, the 72 bits
    /// as short/long pulses (same accumulator as BMW V2), a 500 µs LOW.
    pub fn frame(payload: u64) -> Vec<LevelDuration> {
        frame_with_header(HEADER, payload)
    }

    /// [frame] with another header byte.
    pub fn frame_with_header(header: u8, payload: u64) -> Vec<LevelDuration> {
        let mut bits = bits_of(u64::from(header), 8);
        bits.extend(bits_of(payload, 64));
        frame_bits(&bits)
    }

    /// [frame] for any bit string (header and payload included).
    pub fn frame_bits(bits: &[bool]) -> Vec<LevelDuration> {
        let mut out = vec![
            LevelDuration::new(false, 500),
            LevelDuration::new(true, TE_SHORT),
        ];
        out.extend(pulses_for_bits(
            true,
            true,
            bits,
            TE_SHORT,
            TE_LONG,
            super::super::bmw_v2::testing::step,
        ));
        let last = out.last().is_none_or(|p| p.level);
        out.push(LevelDuration::new(!last, 500));
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::foxfw_common::test_support::*;

    const PAYLOAD_A: u64 = 0x04AB_CD12_3456_789A;
    const PAYLOAD_B: u64 = 0x1001_0203_0405_0607;

    /// Made-up frames: (name, pulses, expected (payload, serial, button, counter)).
    #[allow(clippy::type_complexity)]
    fn cases() -> Vec<(&'static str, Vec<LevelDuration>, Option<(u64, u32, u8, u16)>)> {
        let mut junk_header = bits_of(0x15, 5);
        junk_header.extend(bits_of(u64::from(HEADER), 8));
        junk_header.extend(bits_of(PAYLOAD_B, 64));
        let mut far_header: Vec<bool> = (0..30).map(|i| i % 2 == 0).collect(); // 1010...
        far_header.extend(bits_of(u64::from(HEADER), 8));
        far_header.extend(bits_of(PAYLOAD_B, 64));
        let mut short_payload = bits_of(u64::from(HEADER), 8);
        short_payload.extend(bits_of(PAYLOAD_A >> 1, 63));
        vec![
            ("a", testing::frame(PAYLOAD_A), Some((PAYLOAD_A, 0xABCD, 0x04, 0x3456))),
            ("b", testing::frame(PAYLOAD_B), Some((PAYLOAD_B, 0x0102, 0x10, 0x0405))),
            ("button_02", testing::frame(0x02FE_DCBA_9876_5432), Some((0x02FE_DCBA_9876_5432, 0xFEDC, 0x02, 0x9876))),
            ("button_08", testing::frame(0x08F0_E1D2_C3B4_A596), Some((0x08F0_E1D2_C3B4_A596, 0xF0E1, 0x08, 0xC3B4))),
            ("junk_before_header", testing::frame_bits(&junk_header), Some((PAYLOAD_B, 0x0102, 0x10, 0x0405))),
            ("bad_button", testing::frame(0x03AB_CD12_3456_789A), None),
            ("bad_header", testing::frame_with_header(0xCD, PAYLOAD_A), None),
            ("header_too_late", testing::frame_bits(&far_header), None),
            ("short_payload", testing::frame_bits(&short_payload), None),
        ]
    }

    #[test]
    fn key_file_round_trip_and_decode_only() {
        let dec = BmwV3Decoder::new();
        let d = signal(PAYLOAD_A);
        let text = crate::protocols::flipper_key::testing::assert_round_trip(&dec, &d);
        // the lines the C's serialize writes (checked on the oracle's output)
        assert!(text.contains("Protocol: BMW/Mini V3\nBit: 64\nKey: 04 AB CD 12 34 56 78 9A\nSerial: 43981\nBtn: 4\nCnt: 13398\n"), "{text}");
        let mut wrong_bits = FlipperKey::parse(&text).unwrap();
        wrong_bits.bits = 56;
        assert!(dec.from_flipper_key(&wrong_bits).is_none());
        assert!(!dec.supports_encoding() && dec.encode(&d, 1).is_none());
    }

    #[test]
    fn synthetic_frames_match_the_foxfw_oracle() {
        for (name, wire, want) in cases() {
            dump(&format!("bmw_v3_{name}.sub"), 433_920_000, "FuriHalSubGhzPreset2FSKDev476Async", &wire);
            let got = decode_all(&mut BmwV3Decoder::new(), &wire);
            eprintln!("CASE {name}: {:?}", got.iter().map(|d| (format!("{:016X}", d.data), d.serial, d.button, d.counter)).collect::<Vec<_>>());
            assert_eq!(got.len(), usize::from(want.is_some()), "{name}");
            if let Some((payload, serial, button, counter)) = want {
                let d = &got[0];
                assert_eq!((d.data, d.serial, d.button, d.counter), (payload, Some(serial), Some(button), Some(counter)), "{name}");
                assert_eq!(d.data_count_bit, 64);
            }
        }
    }
}
