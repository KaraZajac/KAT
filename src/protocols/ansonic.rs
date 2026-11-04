//! Ansonic decoder + encoder.
//!
//! Port of Flipper-ARF `applications/system/subghz_garage/protocols/ansonic.c` (garage app, RX
//! group g8 + `tx_ansonic`); doctor-firmware's copy decodes and encodes identically.
//!
//! Air format (315 / 433.92 MHz): PWM with the gap first, te_short 555 µs / te_long 1111 µs,
//! te_delta 120. A frame is a LOW header of `35·te_short` = 19 425 µs (accepted ± 4200), a HIGH
//! start pulse of te_short, then 12 bits, MSB first, each a LOW then a HIGH: `1` = LOW 555 + HIGH
//! 1111, `0` = LOW 1111 + HIGH 555 (both within `te_delta`). A LOW ≥ 2220 µs (`4·te_short`) ends
//! the frame, which is reported when **at least** 12 bits were read (the C's `>=`); the decoder then
//! expects the next start pulse.
//!
//! Fields (`subghz_protocol_ansonic_check_remote_controller`): button = `(data >> 1) & 3`, and the
//! C's `cnt` = `data & 0xFFF` — the 10 DIP switches the C displays (`DIP:` order 0x800 … 0x010,
//! 0x001, 0x008), reported in `counter` as the C's generic block does. No checksum. A decode with
//! more than 12 bits cannot be loaded back (the C's `deserialize` wants exactly 12), so it is not
//! encoder-capable.
//!
//! **RF**: the references disagree — Flipper-ARF (garage app and lib) flag Ansonic
//! `SubGhzProtocolFlag_FM`, doctor-firmware `SubGhzProtocolFlag_AM`; the Flipper unit-test capture
//! (`ansonic_raw.sub`) was recorded with the OOK650 preset, which the UberGuidoZ Ansonic
//! brute-force files use too. KAT takes the union: AM/FM, transmitting AM650 unless the capture says otherwise.
//!
//! Encoder (`repeat` = 3, no counter): header, start pulse, 12 bits. Button: the garage copy
//! re-sends the captured key; Flipper-ARF's `lib/subghz/protocols/ansonic.c` lets the D-pad pick
//! 1, 2, 3 (OK = captured) and rewrites bits 1..2; KAT takes that union and accepts any 2-bit
//! native code (others keep the captured button).

use super::g2_common::{decoded, key_data, key_file, near, push, repeated, BitReg};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`): flags 433 | 315 | FM (ARF) / AM (doctor) → AM/FM, static code.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::Both,
    encryption: "Fixed Code",
    make: "Ansonic",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650, &FM476],
};

const NAME: &str = "Ansonic";
const TE_SHORT: u32 = 555;
const TE_LONG: u32 = 1111;
const TE_DELTA: u32 = 120;
const MIN_COUNT_BIT: usize = 12;
const REPEAT: usize = 3;
const HEADER: u32 = TE_SHORT * 35;
const BUTTON_SHIFT: u32 = 1;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    FoundStartBit,
    SaveDuration,
    CheckDuration,
}

/// Ansonic decoder (`SubGhzProtocolDecoderAnsonic`).
pub struct AnsonicDecoder {
    step: Step,
    te_last: u32,
    reg: BitReg,
}

fn frame(data: u64, bits: usize) -> DecodedSignal {
    let mut d = decoded(data, bits);
    d.button = Some(((data >> BUTTON_SHIFT) & 0x3) as u8);
    d.counter = Some((data & 0xFFF) as u16);
    d.encoder_capable = bits == MIN_COUNT_BIT;
    d
}

impl AnsonicDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            reg: BitReg::default(),
        }
    }
}

impl Default for AnsonicDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// `subghz_protocol_encoder_ansonic_get_upload` (one copy).
fn upload(data: u64) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(2 * MIN_COUNT_BIT + 2);
    push(&mut out, false, HEADER);
    push(&mut out, true, TE_SHORT);
    for i in (0..MIN_COUNT_BIT).rev() {
        let one = (data >> i) & 1 != 0;
        push(&mut out, false, if one { TE_SHORT } else { TE_LONG });
        push(&mut out, true, if one { TE_LONG } else { TE_SHORT });
    }
    out
}

impl ProtocolDecoder for AnsonicDecoder {
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
                if !level && near(duration, HEADER, TE_DELTA * 35) {
                    self.step = Step::FoundStartBit;
                }
            }
            Step::FoundStartBit => {
                if !level {
                    // A LOW is ignored here.
                } else if near(duration, TE_SHORT, TE_DELTA) {
                    self.step = Step::SaveDuration;
                    self.reg.clear();
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::SaveDuration => {
                if !level {
                    if duration >= TE_SHORT * 4 {
                        self.step = Step::FoundStartBit;
                        // The register is cleared at the next start pulse, not here.
                        let bits = self.reg.count as usize;
                        return (bits >= MIN_COUNT_BIT).then(|| frame(self.reg.data, bits));
                    }
                    self.te_last = duration;
                    self.step = Step::CheckDuration;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::CheckDuration => {
                if level {
                    if near(self.te_last, TE_SHORT, TE_DELTA) && near(duration, TE_LONG, TE_DELTA) {
                        self.reg.add(true);
                        self.step = Step::SaveDuration;
                    } else if near(self.te_last, TE_LONG, TE_DELTA)
                        && near(duration, TE_SHORT, TE_DELTA)
                    {
                        self.reg.add(false);
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

    /// 3 copies; a 2-bit `button` replaces bits 1..2 (ARF's D-pad).
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

    /// The unit-test key (`ansonic.sub`) and the C's comment sample (`AAA`).
    const KEYS: [u64; 2] = [0x55A, 0xAAA];

    #[test]
    fn encode_decode_roundtrip_every_button() {
        for key in KEYS {
            let d = frame(key, MIN_COUNT_BIT);
            for (button, want) in [
                (0u8, 0u8),
                (1, 1),
                (2, 2),
                (3, 3),
                (0x80, d.button.unwrap()),
            ] {
                let mut rx = AnsonicDecoder::new().encode(&d, button).unwrap();
                assert_eq!((rx[0].duration_us, rx[1].duration_us), (HEADER, TE_SHORT));
                rx.push(LevelDuration::new(false, HEADER));
                let got = feed_all(&mut AnsonicDecoder::new(), &rx);
                assert_eq!(got.len(), REPEAT);
                for g in &got {
                    assert_eq!(g.button, Some(want));
                    assert_eq!(
                        g.data & !(0x3 << BUTTON_SHIFT),
                        key & !(0x3 << BUTTON_SHIFT)
                    );
                    assert!(g.encoder_capable);
                }
            }
        }
    }

    #[test]
    fn longer_frames_are_reported_but_not_encodable() {
        let mut raw = vec![-19_425, 555];
        for i in (0..13).rev() {
            raw.extend(if (0x155Au64 >> i) & 1 != 0 {
                [-555, 1111]
            } else {
                [-1111, 555]
            });
        }
        raw.push(-19_425);
        let got = feed_all(&mut AnsonicDecoder::new(), &wire(&raw));
        assert_eq!(got.len(), 1);
        assert_eq!(
            (got[0].data, got[0].data_count_bit, got[0].encoder_capable),
            (0x155A, 13, false)
        );
        assert!(AnsonicDecoder::new().encode(&got[0], 1).is_none());
        // 11 bits are too few; a pulse outside both windows breaks the frame.
        let short: Vec<i32> = raw[..2].iter().chain(&raw[6..]).copied().collect();
        assert!(feed_all(&mut AnsonicDecoder::new(), &wire(&short)).is_empty());
        let mut bad = raw.clone();
        bad[5] = 800;
        assert!(feed_all(&mut AnsonicDecoder::new(), &wire(&bad)).is_empty());
    }

    #[test]
    fn key_file_loads_and_roundtrips() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Ansonic\nBit: 12\n\
                    Key: 00 00 00 00 00 00 05 5A\n";
        let dec = AnsonicDecoder::new();
        let d = dec
            .from_flipper_key(&FlipperKey::parse(text).unwrap())
            .unwrap();
        assert_eq!(
            (d.data, d.button, d.counter),
            (0x55A, Some(0b01), Some(0x55A))
        );
        assert_key_roundtrip(&dec, &d);
    }

    #[test]
    fn decodes_unit_test_capture() {
        let Some(got) = registry_decodes(&format!("{UNIT_TESTS}/ansonic_raw.sub")) else {
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
