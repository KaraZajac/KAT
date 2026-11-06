//! Nero Radio decoder + encoder.
//!
//! Port of doctor-firmware `lib/subghz/protocols/nero_radio.c` (not in Flipper-ARF's
//! `subghz_garage` RX groups; Flipper-ARF's lib copy and the garage copy decode identically).
//!
//! Air format (433.92 MHz, AM): PWM, te_short 200 µs / te_long 400 µs, te_delta 80. A frame is a
//! preamble of te_short HIGH/LOW pairs (the encoder sends 49; the decoder needs more than 40), a
//! start pulse HIGH 830 µs (decoder: `4·te_short` ± 80) + LOW te_short, then 56 (or 57) bits, MSB
//! first, each a HIGH then a LOW: `1` = HIGH 400 + LOW 200, `0` = HIGH 200 + LOW 400 (all within
//! `te_delta`). The last bit's LOW is the stop gap (encoder: 4600 µs, 1300 µs for 57-bit keys); a
//! LOW ≥ 900 µs classifies the last bit by its HIGH and ends the frame, which is reported with 56
//! or 57 bits (RogueMaster's bound; doctor / ARF / Momentum use 1250 µs, which drops the first
//! frame of every press in `nero_radio_raw.sub`: those end in 1048–1234 µs LOWs, the others in
//! 4.4–4.7 ms; a data LOW is at most 480 µs, so 900 cuts no frame).
//!
//! Fields (`subghz_protocol_nero_radio_parse_data`, from the C's sample table `serial(28) |
//! button(4) | serial/const(16) | "crc??"(8)`): button = `(data >> 24) & 0xF`; KAT's serial is the
//! 28-bit (29 for 57-bit frames) `data >> 28`. The C's "Sn" (`generic.data_2`) is that serial `<<
//! 16 | (data >> 8) & 0xFFFF`, 44 bits — too wide for KAT's 32-bit serial; it is `serial << 16 |
//! data >> 8 & 0xFFFF` of the reported fields. The low byte is shown as "CRC?" by the C and never
//! checked: its algorithm is unknown (no CRC-8 / LFSR-digest / sum model fits the C's six sample
//! frames from two remotes), so KAT cannot gate on it and reports every frame, like the C.
//!
//! Encoder (`repeat` = 3, no counter): preamble, start pulse, the bits, stop gap. Button: Flipper-ARF's
//! `lib/subghz/protocols/nero_radio.c` lets the D-pad pick buttons 1–4 (OK = captured) and rewrites
//! bits 24..27 — without touching the "CRC?" byte; doctor-firmware re-sends the key. KAT takes the
//! union and accepts any 4-bit native code (others keep the captured button), sending the captured
//! check byte unchanged as ARF does.

use super::g2_common::{bit, decoded, key_data, key_file, near, push, repeated, BitReg};
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
    make: "Nero",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "Nero Radio";
const TE_SHORT: u32 = 200;
const TE_LONG: u32 = 400;
const TE_DELTA: u32 = 80;
const MIN_COUNT_BIT: usize = 56;
/// The 57-bit variant (the decoder and both `deserialize`s accept it).
const LONG_COUNT_BIT: usize = 57;
const REPEAT: usize = 3;
const PREAMBLE_PAIRS: usize = 49;
const START: u32 = 830;
/// RogueMaster's value (doctor/ARF: 1250): the first frame of each press in the Flipper unit-test
/// capture ends in a 1048–1234 µs LOW; data LOWs are at most 480 µs (RogueMaster nero_radio.c).
const STOP: u32 = 900;
const BUTTON_SHIFT: u32 = 24;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    CheckPreambula,
    SaveDuration,
    CheckDuration,
}

/// Nero Radio decoder (`SubGhzProtocolDecoderNeroRadio`).
pub struct NeroRadioDecoder {
    step: Step,
    te_last: u32,
    header_count: u16,
    reg: BitReg,
}

fn bits_ok(bits: usize) -> bool {
    bits == MIN_COUNT_BIT || bits == LONG_COUNT_BIT
}

fn frame(data: u64, bits: usize) -> DecodedSignal {
    let mut d = decoded(data, bits);
    d.serial = Some((data >> 28) as u32);
    d.button = Some(((data >> BUTTON_SHIFT) & 0xF) as u8);
    d.encoder_capable = bits_ok(bits);
    d
}

impl NeroRadioDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            header_count: 0,
            reg: BitReg::default(),
        }
    }
}

impl Default for NeroRadioDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// `subghz_protocol_encoder_nero_radio_get_upload` (one copy).
fn upload(data: u64, bits: usize) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(2 * (PREAMBLE_PAIRS + 1 + bits));
    for _ in 0..PREAMBLE_PAIRS {
        push(&mut out, true, TE_SHORT);
        push(&mut out, false, TE_SHORT);
    }
    push(&mut out, true, START);
    push(&mut out, false, TE_SHORT);
    for i in (1..bits).rev() {
        let one = bit(data, i);
        push(&mut out, true, if one { TE_LONG } else { TE_SHORT });
        push(&mut out, false, if one { TE_SHORT } else { TE_LONG });
    }
    push(
        &mut out,
        true,
        if data & 1 != 0 { TE_LONG } else { TE_SHORT },
    );
    push(
        &mut out,
        false,
        if bits == LONG_COUNT_BIT {
            1300
        } else {
            TE_SHORT * 23
        },
    );
    out
}

impl ProtocolDecoder for NeroRadioDecoder {
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
                if level && near(duration, TE_SHORT, TE_DELTA) {
                    self.step = Step::CheckPreambula;
                    self.te_last = duration;
                    self.header_count = 0;
                }
            }
            Step::CheckPreambula => {
                if level {
                    if near(duration, TE_SHORT, TE_DELTA) || near(duration, TE_SHORT * 4, TE_DELTA)
                    {
                        self.te_last = duration;
                    } else {
                        self.step = Step::Reset;
                    }
                } else if near(duration, TE_SHORT, TE_DELTA) {
                    if near(self.te_last, TE_SHORT, TE_DELTA) {
                        self.header_count = self.header_count.wrapping_add(1);
                    } else if near(self.te_last, TE_SHORT * 4, TE_DELTA) {
                        if self.header_count > 40 {
                            self.step = Step::SaveDuration;
                            self.reg.clear();
                        } else {
                            self.step = Step::Reset;
                        }
                    } else {
                        self.step = Step::Reset;
                    }
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::SaveDuration => {
                if level {
                    self.te_last = duration;
                    self.step = Step::CheckDuration;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::CheckDuration => {
                if !level {
                    if duration >= STOP {
                        if near(self.te_last, TE_SHORT, TE_DELTA) {
                            self.reg.add(false);
                        } else if near(self.te_last, TE_LONG, TE_DELTA) {
                            self.reg.add(true);
                        }
                        self.step = Step::Reset;
                        let bits = self.reg.count as usize;
                        let result = bits_ok(bits).then(|| frame(self.reg.data, bits));
                        self.reg.clear();
                        return result;
                    } else if near(self.te_last, TE_SHORT, TE_DELTA)
                        && near(duration, TE_LONG, TE_DELTA)
                    {
                        self.reg.add(false);
                        self.step = Step::SaveDuration;
                    } else if near(self.te_last, TE_LONG, TE_DELTA)
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

    /// 3 copies; a 4-bit `button` replaces bits 24..27 (ARF's D-pad; the check byte is kept).
    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        let bits = decoded.data_count_bit;
        if !bits_ok(bits) {
            return None;
        }
        let mut data = decoded.data;
        if button <= 0xF {
            data = (data & !(0xF << BUTTON_SHIFT)) | (u64::from(button) << BUTTON_SHIFT);
        }
        Some(repeated(&upload(data, bits), REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        let (data, bits) = key_data(key, bits_ok)?;
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

    /// The unit-test remote and the second remote quoted in nero_radio.c.
    const KEYS: [u64; 6] = [
        0x0057_2505_0104_9DD3,
        0x0057_2505_0204_9D13,
        0x0036_E4E8_0104_A644,
        0x0036_E4E8_0204_A684,
        0x0036_E4E8_0304_A604,
        0x0036_E4E8_0404_A6E4,
    ];

    #[test]
    fn fields_follow_the_c_table() {
        let d = frame(KEYS[2], 56);
        assert_eq!((d.serial, d.button), (Some(0x36E_4E80), Some(1)));
        // The C's "Sn" (data_2) from the reported fields.
        let sn = (u64::from(d.serial.unwrap()) << 16) | ((d.data >> 8) & 0xFFFF);
        assert_eq!(sn, ((KEYS[2] >> 28) << 16) | ((KEYS[2] >> 8) & 0xFFFF));
        assert!(!d.crc_valid, "the check byte is not verified");
    }

    #[test]
    fn encode_decode_roundtrip_every_button_and_length() {
        for key in KEYS {
            for bits in [56, 57] {
                let d = frame(key, bits);
                for (button, want) in [
                    (1u8, 1u8),
                    (2, 2),
                    (3, 3),
                    (4, 4),
                    (0xF, 0xF),
                    (0x10, d.button.unwrap()),
                ] {
                    let tx = NeroRadioDecoder::new().encode(&d, button).unwrap();
                    let got = feed_all(&mut NeroRadioDecoder::new(), &tx);
                    assert_eq!(got.len(), REPEAT, "{key:X}/{bits}");
                    for g in &got {
                        assert_eq!((g.data_count_bit, g.button), (bits, Some(want)));
                        assert_eq!(
                            g.data & !(0xF << BUTTON_SHIFT),
                            key & !(0xF << BUTTON_SHIFT),
                            "check byte kept"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn rejects_short_preamble_and_wrong_length() {
        let tx = upload(KEYS[0], 56);
        let few: Vec<LevelDuration> = tx[2 * (PREAMBLE_PAIRS - 40)..].to_vec();
        assert!(feed_all(&mut NeroRadioDecoder::new(), &few).is_empty());
        let mut short = tx.clone();
        short.drain(2 * PREAMBLE_PAIRS + 2..2 * PREAMBLE_PAIRS + 6); // 54 bits
        assert!(feed_all(&mut NeroRadioDecoder::new(), &short).is_empty());
        let mut bad = tx.clone();
        bad[2 * PREAMBLE_PAIRS + 4].duration_us = 300; // HIGH between short and long
        assert!(feed_all(&mut NeroRadioDecoder::new(), &bad).is_empty());
    }

    #[test]
    fn key_files_roundtrip() {
        let dec = NeroRadioDecoder::new();
        for bits in [56, 57] {
            let d = frame(KEYS[0], bits);
            assert_key_roundtrip(&dec, &d);
        }
        let text = dec.to_flipper_key(&frame(KEYS[0], 56)).unwrap().to_text();
        assert!(
            text.contains("Protocol: Nero Radio\nBit: 56\nKey: 00 57 25 05 01 04 9D D3\n"),
            "{text}"
        );
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&text.replace("Bit: 56", "Bit: 58")).unwrap())
            .is_none());
    }

    #[test]
    fn decodes_unit_test_capture() {
        let Some(got) = registry_decodes(&format!("{UNIT_TESTS}/nero_radio_raw.sub")) else {
            return;
        };
        let m: Vec<_> = got.iter().filter(|(n, _)| n == NAME).collect();
        assert!(
            !m.is_empty(),
            "{:?}",
            got.iter().map(|(n, _)| n).collect::<Vec<_>>()
        );
        assert!(
            m.iter()
                .all(|(_, d)| d.data == KEYS[0] || d.data == KEYS[1]),
            "{m:?}"
        );
    }
}
