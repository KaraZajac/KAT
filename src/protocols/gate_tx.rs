//! GateTX (24-bit fixed code) decoder + encoder.
//!
//! Port of Flipper-ARF's garage app `applications/system/subghz_garage/protocols/gate_tx.c` (the
//! primary reference; doctor-firmware's copy has the same wire behaviour). Flipper-ARF's lib copy
//! adds an encoder button switch (the button nibble of the bit-reversed key), which KAT takes.
//!
//! **Air format** (AM, 433.92 MHz): `te_short` 350, `te_long` 700, `te_delta` 100. Header LOW 49·te
//! (17150 µs), start HIGH 700, then MSB first: bit 1 = LOW 700 + HIGH 350, bit 0 = LOW 350 + HIGH
//! 700.
//!
//! **Decoder** (`subghz_protocol_decoder_gate_tx_feed`): header LOW within 16450 ± 4700 µs, start
//! HIGH |d − 700| < 300, then LOW/HIGH pairs (short ± 100, long ± 300); a LOW ≥ 3600 µs ends the
//! frame, reported when it holds exactly 24 bits. (A HIGH while waiting for a LOW is ignored, as
//! in the C.)
//!
//! **Fields** (`subghz_protocol_gate_tx_check_remote_controller`, on the 24-bit reversed key `r`):
//! `serial` = (r & 0xFF) << 12 | (r >> 8 & 0xFF) << 4 | (r >> 20 & 0xF), `button` = r >> 16 & 0xF.
//!
//! **Encoder** (`subghz_protocol_encoder_gate_tx_get_upload`, `encoder.repeat` = 3): header, start
//! bit, 24 bits. `button` 0..=15 replaces the button nibble (ARF lib's custom button: bits 16..19
//! of the reversed key); larger codes keep the captured key.
//!
//! **Key file**: `Protocol: GateTX`, `Bit: 24`, `Key`.

use super::g1_common::{generic_fields, generic_key, push, reverse_key, with_repeats, TX_REPEAT};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "GateTX",
    decoder: "GateTX",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Gate-TX",
    category: Category::Gate,
    buttons: Some(super::g1_common::numbered_buttons),
    presets: &[&AM650],
};

const NAME: &str = "GateTX";
const TE_SHORT: u32 = 350;
const TE_LONG: u32 = 700;
const TE_DELTA: u32 = 100;
const MIN_COUNT_BIT: usize = 24;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    FoundStartBit,
    SaveDuration,
    CheckDuration,
}

/// GateTX decoder (`SubGhzProtocolDecoderGateTx`).
pub struct GateTxDecoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
}

/// `subghz_protocol_gate_tx_check_remote_controller`: (serial, button).
fn check_remote_controller(data: u64, bits: usize) -> (u32, u8) {
    let r = reverse_key(data, bits as u8) as u32;
    let serial = (r & 0xFF) << 12 | ((r >> 8) & 0xFF) << 4 | ((r >> 20) & 0x0F);
    (serial, ((r >> 16) & 0x0F) as u8)
}

fn decoded(data: u64) -> DecodedSignal {
    let (serial, btn) = check_remote_controller(data, MIN_COUNT_BIT);
    DecodedSignal {
        serial: Some(serial),
        button: Some(btn),
        counter: None,
        crc_valid: false,
        data,
        data_count_bit: MIN_COUNT_BIT,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

/// ARF lib's button switch: the nibble at bits 16..19 of the reversed key.
fn with_button(data: u64, btn: u8) -> u64 {
    let r = reverse_key(data, MIN_COUNT_BIT as u8) as u32;
    let r = (r & !(0x0F << 16)) | (btn as u32 & 0x0F) << 16;
    reverse_key(r as u64, MIN_COUNT_BIT as u8)
}

/// One `get_upload` frame.
fn build_frame(data: u64) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(MIN_COUNT_BIT * 2 + 2);
    push(&mut out, false, TE_SHORT * 49);
    push(&mut out, true, TE_LONG);
    for i in (0..MIN_COUNT_BIT).rev() {
        if (data >> i) & 1 == 1 {
            push(&mut out, false, TE_LONG);
            push(&mut out, true, TE_SHORT);
        } else {
            push(&mut out, false, TE_SHORT);
            push(&mut out, true, TE_LONG);
        }
    }
    out
}

impl GateTxDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
        }
    }

    fn add_bit(&mut self, bit: u64) {
        self.decode_data = self.decode_data << 1 | bit;
        self.decode_count_bit = self.decode_count_bit.wrapping_add(1);
    }
}

impl Default for GateTxDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for GateTxDecoder {
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
                if !level && duration_diff!(duration, TE_SHORT * 47) < TE_DELTA * 47 {
                    self.step = Step::FoundStartBit;
                }
            }
            Step::FoundStartBit => {
                if level && duration_diff!(duration, TE_LONG) < TE_DELTA * 3 {
                    self.step = Step::SaveDuration;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::SaveDuration => {
                if !level {
                    if duration >= TE_SHORT * 10 + TE_DELTA {
                        self.step = Step::FoundStartBit;
                        let mut out = None;
                        if self.decode_count_bit as usize == MIN_COUNT_BIT {
                            out = Some(decoded(self.decode_data));
                            // callback → the app resets the receiver.
                            self.step = Step::Reset;
                        }
                        self.decode_data = 0;
                        self.decode_count_bit = 0;
                        return out;
                    }
                    self.te_last = duration;
                    self.step = Step::CheckDuration;
                }
            }
            Step::CheckDuration => {
                if level {
                    if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA
                        && duration_diff!(duration, TE_LONG) < TE_DELTA * 3
                    {
                        self.add_bit(0);
                        self.step = Step::SaveDuration;
                    } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA * 3
                        && duration_diff!(duration, TE_SHORT) < TE_DELTA
                    {
                        self.add_bit(1);
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

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        let data = if button <= 0x0F {
            with_button(decoded.data, button)
        } else {
            decoded.data
        };
        Some(with_repeats(&build_frame(data), TX_REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        let (bits, data) = generic_fields(key);
        (bits == MIN_COUNT_BIT).then(|| decoded(data))
    }

    fn to_flipper_key(&self, d: &DecodedSignal) -> Option<FlipperKey> {
        Some(generic_key(NAME, d))
    }
}

#[cfg(test)]
mod tests {
    use super::super::g1_common::test_support::*;
    use super::*;

    fn on_air(d: &DecodedSignal, button: u8) -> Vec<LevelDuration> {
        let mut wire = GateTxDecoder::new().encode(d, button).unwrap();
        push(&mut wire, false, 20_000);
        wire
    }

    #[test]
    fn fields_and_button_switch() {
        let d = decoded(0x028FF3);
        let (serial, btn) = check_remote_controller(0x028FF3, 24);
        assert_eq!((d.serial, d.button), (Some(serial), Some(btn)));
        // The identity for the captured button, and a different nibble otherwise.
        assert_eq!(with_button(0x028FF3, btn), 0x028FF3);
        for b in 0..16u8 {
            let k = with_button(0x028FF3, b);
            assert_eq!(check_remote_controller(k, 24), (serial, b));
        }
    }

    #[test]
    fn encode_decode_roundtrip() {
        let d = decoded(0x028FF3);
        for button in [d.button.unwrap(), 0x1, 0x8, 0x40] {
            let got = feed_all(&mut GateTxDecoder::new(), &on_air(&d, button));
            assert_eq!(got.len(), 2);
            let want = if button <= 0xF {
                button
            } else {
                d.button.unwrap()
            };
            assert!(
                got.iter()
                    .all(|f| f.serial == d.serial && f.button == Some(want)),
                "{button:#x}"
            );
        }
    }

    #[test]
    fn rejects_wrong_length_and_start_bit() {
        let mut wire = build_frame(0x028FF3);
        wire[1] = LevelDuration::new(true, 350); // start bit must be ~700 µs
        push(&mut wire, false, 20_000);
        assert!(feed_all(&mut GateTxDecoder::new(), &wire).is_empty());
        let mut short = build_frame(0x028FF3);
        short.truncate(2 + 2 * 23); // 23 bits
        push(&mut short, false, 20_000);
        assert!(feed_all(&mut GateTxDecoder::new(), &short).is_empty());
    }

    #[test]
    fn unit_test_key_file() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: GateTX\nBit: 24\n\
                    Key: 00 00 00 00 00 02 8F F3\n";
        let dec = GateTxDecoder::new();
        let d = dec
            .from_flipper_key(&FlipperKey::parse(text).unwrap())
            .unwrap();
        assert_eq!(d.data, 0x028FF3);
        assert_eq!(
            feed_all(&mut GateTxDecoder::new(), &on_air(&d, d.button.unwrap()))[0].data,
            0x028FF3
        );
        let written = dec.to_flipper_key(&d).unwrap().to_text();
        let back = dec
            .from_flipper_key(&FlipperKey::parse(&written).unwrap())
            .unwrap();
        assert_eq!(format!("{back:?}"), format!("{d:?}"));
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&text.replace("Bit: 24", "Bit: 12")).unwrap())
            .is_none());
    }

    #[test]
    fn reference_raw_capture_decodes() {
        let Some(decodes) = decode_reference_capture("gate_tx_raw.sub") else {
            return;
        };
        assert!(
            decodes.iter().any(|(n, _)| n == NAME),
            "{:?}",
            decodes.iter().map(|(n, _)| n).collect::<Vec<_>>()
        );
    }
}
