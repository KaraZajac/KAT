//! iDo 117/111 decoder (decode-only).
//!
//! Port of doctor-firmware `lib/subghz/protocols/ido.c` (not in Flipper-ARF's `subghz_garage` RX
//! groups; Flipper-ARF's lib copy and the garage copy decode identically). The protocol is
//! `SubGhzProtocolTypeDynamic` with flags `433 | AM | Decodable | Save` — no `Load`/`Send`, and the
//! encoder table is all NULL — so KAT decodes and saves it but cannot transmit it.
//!
//! Air format (433.92 MHz, AM): PWM, te_short 450 µs / te_long 1450 µs, te_delta 150. A frame
//! starts with a HIGH and a LOW of `10·te_short` = 4500 µs (each ± 750), then bits, each a HIGH
//! then a LOW: `0` = HIGH 450 (± 150) + LOW 1450 (± 450), `1` = HIGH 450 (± 450) + LOW 450
//! (± 150). A HIGH ≥ 2400 µs (`5·te_short + te_delta`, the next frame's preamble) ends the frame,
//! which is reported when **at least** 48 bits were read (the C's `>=`; only the last 64 stay in
//! the register).
//!
//! Fields (`subghz_protocol_ido_check_remote_controller`): the frame is read bit-reversed
//! (`reverse_key(data, bits)`); its low 24 bits are the fixed part (serial = low 20 bits, button =
//! the next 4) and the following 24 bits the hopping code, shown by the C as "Hop". KAT reports
//! serial and button; the hop stays in `data`. No checksum.

use super::g2_common::{decoded, key_data, key_file, near, reverse_key, BitReg};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`): `SubGhzProtocolFlag_433 | AM`, dynamic (hopping) code, decode-only.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "iDo",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "iDo 117/111";
const TE_SHORT: u32 = 450;
const TE_LONG: u32 = 1450;
const TE_DELTA: u32 = 150;
const MIN_COUNT_BIT: usize = 48;
const PREAMBLE: u32 = TE_SHORT * 10;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    FoundPreambula,
    SaveDuration,
    CheckDuration,
}

/// iDo 117/111 decoder (`SubGhzProtocolDecoderIDo`).
pub struct IdoDecoder {
    step: Step,
    te_last: u32,
    reg: BitReg,
}

fn frame(data: u64, bits: usize) -> DecodedSignal {
    let fix = reverse_key(data, bits) & 0xFF_FFFF;
    let mut d = decoded(data, bits);
    d.serial = Some((fix & 0xF_FFFF) as u32);
    d.button = Some(((fix >> 20) & 0xF) as u8);
    d
}

impl IdoDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            reg: BitReg::default(),
        }
    }
}

impl Default for IdoDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for IdoDecoder {
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
                if level && near(duration, PREAMBLE, TE_DELTA * 5) {
                    self.step = Step::FoundPreambula;
                }
            }
            Step::FoundPreambula => {
                if !level && near(duration, PREAMBLE, TE_DELTA * 5) {
                    self.step = Step::SaveDuration;
                    self.reg.clear();
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::SaveDuration => {
                if level {
                    if duration >= TE_SHORT * 5 + TE_DELTA {
                        self.step = Step::FoundPreambula;
                        let bits = self.reg.count as usize;
                        let result = (bits >= MIN_COUNT_BIT).then(|| frame(self.reg.data, bits));
                        self.reg.clear();
                        return result;
                    }
                    self.te_last = duration;
                    self.step = Step::CheckDuration;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::CheckDuration => {
                if !level {
                    if near(self.te_last, TE_SHORT, TE_DELTA)
                        && near(duration, TE_LONG, TE_DELTA * 3)
                    {
                        self.reg.add(false);
                        self.step = Step::SaveDuration;
                    } else if near(self.te_last, TE_SHORT, TE_DELTA * 3)
                        && near(duration, TE_SHORT, TE_DELTA)
                    {
                        self.reg.add(true);
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
        false
    }

    /// The references have no iDo encoder.
    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }

    /// The decoder's `deserialize` (exactly 48 bits); the C cannot load it for transmission.
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        let (data, bits) = key_data(key, |bits| bits == MIN_COUNT_BIT)?;
        Some(frame(data, bits))
    }

    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        Some(key_file(NAME, decoded))
    }
}

#[cfg(test)]
mod tests {
    use super::super::g2_common::test_util::*;
    use super::*;

    /// A frame from the C's layout: preamble, bits, next preamble.
    fn frame_wire(key: u64, bits: usize) -> Vec<LevelDuration> {
        let mut raw = vec![4500, -4500];
        for i in (0..bits).rev() {
            raw.extend(if (key >> i) & 1 != 0 {
                [450, -450]
            } else {
                [450, -1450]
            });
        }
        raw.extend([4500, -4500]);
        wire(&raw)
    }

    #[test]
    fn decodes_a_hand_built_frame() {
        let key = 0x8E1C_2A3B_4D5Fu64;
        let got = feed_all(&mut IdoDecoder::new(), &frame_wire(key, 48));
        assert_eq!(got.len(), 1);
        let d = &got[0];
        assert_eq!(
            (d.data, d.data_count_bit, d.encoder_capable),
            (key, 48, false)
        );
        let fix = reverse_key(key, 48) & 0xFF_FFFF;
        assert_eq!(
            (d.serial, d.button),
            (Some((fix & 0xF_FFFF) as u32), Some((fix >> 20) as u8))
        );
        // ≥ 48: a 50-bit frame is reported too (the C's `>=`).
        assert_eq!(
            feed_all(&mut IdoDecoder::new(), &frame_wire(key, 50))[0].data_count_bit,
            50
        );
        // 47 bits are not.
        assert!(feed_all(&mut IdoDecoder::new(), &frame_wire(key, 47)).is_empty());
        // A LOW outside both windows breaks the frame.
        let mut bad = frame_wire(key, 48);
        bad[5].duration_us = 900;
        assert!(feed_all(&mut IdoDecoder::new(), &bad).is_empty());
    }

    #[test]
    fn decode_only() {
        let dec = IdoDecoder::new();
        let d = frame(0x8E1C_2A3B_4D5F, 48);
        assert!(!dec.supports_encoding() && !d.encoder_capable);
        assert!(dec.encode(&d, 0).is_none());
        assert_key_roundtrip(&dec, &d);
        let text = dec.to_flipper_key(&d).unwrap().to_text();
        assert!(text.contains("Protocol: iDo 117/111\nBit: 48\n"), "{text}");
    }

    #[test]
    fn decodes_unit_test_capture() {
        let Some(got) = registry_decodes(&format!("{UNIT_TESTS}/ido_117_111_raw.sub")) else {
            return;
        };
        let m: Vec<_> = got.iter().filter(|(n, _)| n == NAME).collect();
        assert!(
            !m.is_empty(),
            "{:?}",
            got.iter().map(|(n, _)| n).collect::<Vec<_>>()
        );
    }
}
