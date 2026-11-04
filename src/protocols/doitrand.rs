//! Doitrand decoder + encoder.
//!
//! Port of Flipper-ARF `applications/system/subghz_garage/protocols/doitrand.c` (garage app, RX
//! group g7 + `tx_doitrand`); doctor-firmware's copy decodes and encodes identically.
//!
//! Air format (433.92 MHz, AM): PWM with the gap first, te_short 400 µs / te_long 1100 µs,
//! te_delta 150. A frame is a LOW header of `62·te_short` = 24 800 µs (accepted ± 4500), a HIGH
//! start pulse (encoder: `2·te_short − 100` = 700 µs; decoder: 800 ± 450), then 37 bits, MSB
//! first, each a LOW then a HIGH: `1` = LOW 1100 + HIGH 400, `0` = LOW 400 + HIGH 1100 (short
//! within `te_delta`, long within `3·te_delta`). A LOW ≥ 4150 µs (`10·te_short + te_delta`) ends
//! the frame, which is reported when exactly 37 bits were read; the decoder then expects the next
//! start pulse at once.
//!
//! Fields (`subghz_protocol_doitrand_check_remote_controller`): button = `(data >> 18) & 3`, and
//! the C's `cnt` = `(data >> 24) | ((data >> 15) & 1)` — not a counter but the 10 DIP switches the
//! C displays (`DIP:` order 0x001, 0x100, 0x080, 0x040, 0x020, 0x1000, 0x800, 0x400, 0x200, 0x002);
//! KAT reports it in `counter` as the C's generic block does. No checksum.
//!
//! Encoder (`repeat` = 3, no counter): header, start pulse, 37 bits. Button: the garage copy
//! re-sends the captured key; Flipper-ARF's `lib/subghz/protocols/doitrand.c` lets the D-pad pick
//! 1, 2, 3 (OK = captured) and rewrites bits 18..19; KAT takes that union and accepts any 2-bit
//! native code (others keep the captured button).

use super::g2_common::{decoded, key_data, key_file, near, push, repeated, BitReg};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`): `SubGhzProtocolFlag_433 | AM`, static code.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Doitrand",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "Doitrand";
const TE_SHORT: u32 = 400;
const TE_LONG: u32 = 1100;
const TE_DELTA: u32 = 150;
const MIN_COUNT_BIT: usize = 37;
const REPEAT: usize = 3;
const HEADER: u32 = TE_SHORT * 62;
const BUTTON_SHIFT: u32 = 18;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    FoundStartBit,
    SaveDuration,
    CheckDuration,
}

/// Doitrand decoder (`SubGhzProtocolDecoderDoitrand`).
pub struct DoitrandDecoder {
    step: Step,
    te_last: u32,
    reg: BitReg,
}

fn frame(data: u64, bits: usize) -> DecodedSignal {
    let mut d = decoded(data, bits);
    d.button = Some(((data >> BUTTON_SHIFT) & 0x3) as u8);
    d.counter = Some(((data >> 24) | ((data >> 15) & 0x1)) as u16);
    d.encoder_capable = bits == MIN_COUNT_BIT;
    d
}

impl DoitrandDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            reg: BitReg::default(),
        }
    }
}

impl Default for DoitrandDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// `subghz_protocol_encoder_doitrand_get_upload` (one copy).
fn upload(data: u64) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(2 * MIN_COUNT_BIT + 2);
    push(&mut out, false, HEADER);
    push(&mut out, true, TE_SHORT * 2 - 100);
    for i in (0..MIN_COUNT_BIT).rev() {
        let one = (data >> i) & 1 != 0;
        push(&mut out, false, if one { TE_LONG } else { TE_SHORT });
        push(&mut out, true, if one { TE_SHORT } else { TE_LONG });
    }
    out
}

impl ProtocolDecoder for DoitrandDecoder {
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
                if !level && near(duration, HEADER, TE_DELTA * 30) {
                    self.step = Step::FoundStartBit;
                }
            }
            Step::FoundStartBit => {
                if level && near(duration, TE_SHORT * 2, TE_DELTA * 3) {
                    self.step = Step::SaveDuration;
                    self.reg.clear();
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::SaveDuration => {
                if !level {
                    if duration >= TE_SHORT * 10 + TE_DELTA {
                        self.step = Step::FoundStartBit;
                        let result = (self.reg.count as usize == MIN_COUNT_BIT)
                            .then(|| frame(self.reg.data, MIN_COUNT_BIT));
                        self.reg.clear();
                        return result;
                    }
                    self.te_last = duration;
                    self.step = Step::CheckDuration;
                }
            }
            Step::CheckDuration => {
                if level {
                    if near(self.te_last, TE_SHORT, TE_DELTA)
                        && near(duration, TE_LONG, TE_DELTA * 3)
                    {
                        self.reg.add(false);
                        self.step = Step::SaveDuration;
                    } else if near(self.te_last, TE_LONG, TE_DELTA * 3)
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
        true
    }

    /// 3 copies; a 2-bit `button` replaces bits 18..19 (ARF's D-pad), others keep the capture's.
    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        let mut data = decoded.data;
        if button <= 0x3 {
            data = (data & !(0x3 << BUTTON_SHIFT)) | (u64::from(button) << BUTTON_SHIFT);
        }
        Some(repeated(&upload(data), REPEAT))
    }

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

    /// The unit-test key (`doitrand.sub`) and samples from the C's comment table.
    const KEYS: [u64; 3] = [0x1E_6008_2F5F, 0x00_0008_2F5F, 0x1F_E208_AF5F];

    #[test]
    fn encode_decode_roundtrip_every_button() {
        for key in KEYS {
            let d = frame(key, MIN_COUNT_BIT);
            for (button, want) in [(0u8, 0u8), (1, 1), (2, 2), (3, 3), (4, d.button.unwrap())] {
                let tx = DoitrandDecoder::new().encode(&d, button).unwrap();
                assert_eq!(
                    (tx[0].level, tx[0].duration_us, tx[1].duration_us),
                    (false, HEADER, 700)
                );
                // Each copy is reported at the next copy's header; the last needs trailing silence.
                let mut rx = tx.clone();
                rx.push(LevelDuration::new(false, HEADER));
                let got = feed_all(&mut DoitrandDecoder::new(), &rx);
                assert_eq!(got.len(), REPEAT);
                for g in &got {
                    assert_eq!(g.button, Some(want));
                    assert_eq!(g.counter, d.counter, "the DIP field never changes");
                    assert_eq!(
                        g.data & !(0x3 << BUTTON_SHIFT),
                        key & !(0x3 << BUTTON_SHIFT)
                    );
                }
                // With a reset after each report (registry), every other copy is reported.
                assert_eq!(feed_with_resets(&mut DoitrandDecoder::new(), &rx).len(), 2);
            }
        }
    }

    #[test]
    fn fields_follow_the_c_layout() {
        // 1FE208AF5F => DIP 11111111000100000, key 10 (C comment table).
        let d = frame(0x1F_E208_AF5F, MIN_COUNT_BIT);
        assert_eq!(d.button, Some(0b10));
        assert_eq!(
            d.counter,
            Some(((0x1F_E208_AF5Fu64 >> 24) | ((0x1F_E208_AF5Fu64 >> 15) & 1)) as u16)
        );
        let hand: Vec<i32> = std::iter::once(-24_800)
            .chain(std::iter::once(800))
            .chain((0..37).rev().flat_map(|i| {
                if (KEYS[0] >> i) & 1 != 0 {
                    [-1100, 400]
                } else {
                    [-400, 1100]
                }
            }))
            .chain(std::iter::once(-24_800))
            .collect();
        let got = feed_all(&mut DoitrandDecoder::new(), &wire(&hand));
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].data, KEYS[0]);
    }

    #[test]
    fn rejects_short_frames_and_bad_start_bit() {
        let mut rx = upload(KEYS[0]);
        rx.drain(2..4); // 36 bits
        rx.push(LevelDuration::new(false, HEADER));
        assert!(feed_all(&mut DoitrandDecoder::new(), &rx).is_empty());
        let mut rx = upload(KEYS[0]);
        rx[1].duration_us = 1300; // start pulse outside 800 ± 450
        rx.push(LevelDuration::new(false, HEADER));
        assert!(feed_all(&mut DoitrandDecoder::new(), &rx).is_empty());
    }

    #[test]
    fn key_file_loads_and_roundtrips() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Doitrand\nBit: 37\n\
                    Key: 00 00 00 1E 60 08 2F 5F\n";
        let dec = DoitrandDecoder::new();
        let d = dec
            .from_flipper_key(&FlipperKey::parse(text).unwrap())
            .unwrap();
        assert_eq!((d.data, d.button), (KEYS[0], Some(0b10)));
        assert_key_roundtrip(&dec, &d);
    }

    #[test]
    fn decodes_unit_test_capture() {
        let Some(got) = registry_decodes(&format!("{UNIT_TESTS}/doitrand_raw.sub")) else {
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
