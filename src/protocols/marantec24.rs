//! Marantec24 decoder + encoder.
//!
//! Port of Flipper-ARF `applications/system/subghz_garage/protocols/marantec24.c` (garage app, RX
//! group g9 + `tx_marantec24`); doctor-firmware's copy decodes and encodes identically. Despite the
//! name it is not a Marantec format: the C notes it comes from a Chinese cloner remote that claims
//! to be a Marantec replica (24-bit fixed code, @xMasterX 2024/2025).
//!
//! Air format (868.35 MHz flag, AM): PWM, te_short 800 µs / te_long 1600 µs, te_delta 200 (strict
//! `<`). A bit is a HIGH pulse and a LOW gap: `1` = HIGH 800 + LOW 3200 (`2·te_long`), `0` = HIGH
//! 1600 + LOW 2400 (`3·te_short`); the last bit's LOW is the frame gap, `9·te_long + te_short`
//! = 15 200 µs. The decoder waits for a LOW of 15 200 ± `4·te_delta` µs (14 401..15 999), reads 24
//! bits, and reports the frame on the next such gap (classifying the last bit by its HIGH alone);
//! it then looks for a gap again, so of back-to-back frames every other one is reported. 24 bits,
//! MSB first: serial = `data >> 4` (20 bits), button = `data & 0xF`. No checksum.
//!
//! **Gap window (RogueMaster's)**: the garage app, doctor-firmware, ARF and Momentum centre the
//! window on `9·te_long` = 14 400 µs (13 601..15 199, strict `<`), 800 µs from their own encoders'
//! 15 200 µs gap, so none of them reads its own upload (nor KAT's, which sends the C's unchanged),
//! and the real remote's gaps (15 118..15 188 µs in `marantec24_raw.sub`) clear the bound by 12 µs
//! at worst. RogueMaster re-centres the window on the encoder's gap ("nothing was ever measured in
//! the old low half"); KAT does the same since v1.8.7: every frame of the capture still decodes,
//! and every firmware's transmission does too.
//!
//! Encoder (`repeat` = 3, no counter): the 24-bit key as above. Button: the garage copy re-sends
//! the captured key; Flipper-ARF's `lib/subghz/protocols/marantec24.c` lets the D-pad pick 0x8,
//! 0x4, 0xC, 0x0 and rebuilds `serial << 4 | btn`; KAT takes that union and accepts any 4-bit
//! native code (others keep the captured button).

use super::g2_common::{decoded, key_data, key_file, near, push, repeated, BitReg};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`): `SubGhzProtocolFlag_868 | AM`, static code.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Marantec",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "Marantec24";
const TE_SHORT: u32 = 800;
const TE_LONG: u32 = 1600;
const TE_DELTA: u32 = 200;
const MIN_COUNT_BIT: usize = 24;
const REPEAT: usize = 3;
/// The gap the encoder sends after the last bit: `te_long * 9 + te_short` (= 15 200 µs; the real
/// remote's gaps measure 15 118..15 188 µs in `marantec24_raw.sub`).
const TX_GAP: u32 = TE_LONG * 9 + TE_SHORT;
/// The gap the decoder looks for: RogueMaster's `TX_GAP` ± `te_delta * 4` (14 401..15 999 µs). The
/// garage app / doctor / ARF / Momentum copies centre it on `te_long * 9` (13 601..15 199), 12 µs
/// over the remote's longest gap and under every firmware's own 15 200 µs upload.
const GAP: u32 = TX_GAP;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    SaveDuration,
    CheckDuration,
}

/// Marantec24 decoder (`SubGhzProtocolDecoderMarantec24`).
pub struct Marantec24Decoder {
    step: Step,
    te_last: u32,
    reg: BitReg,
}

fn frame(data: u64) -> DecodedSignal {
    let mut d = decoded(data, MIN_COUNT_BIT);
    d.serial = Some((data >> 4) as u32);
    d.button = Some((data & 0xF) as u8);
    d.encoder_capable = true;
    d
}

impl Marantec24Decoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            reg: BitReg::default(),
        }
    }
}

impl Default for Marantec24Decoder {
    fn default() -> Self {
        Self::new()
    }
}

/// `subghz_protocol_encoder_marantec24_get_upload` (one copy).
fn upload(data: u64) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(2 * MIN_COUNT_BIT);
    for i in (1..=MIN_COUNT_BIT).rev() {
        let one = (data >> (i - 1)) & 1 != 0;
        push(&mut out, true, if one { TE_SHORT } else { TE_LONG });
        let low = if i == 1 {
            TX_GAP
        } else if one {
            TE_LONG * 2
        } else {
            TE_SHORT * 3
        };
        push(&mut out, false, low);
    }
    out
}

impl ProtocolDecoder for Marantec24Decoder {
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
                if !level && near(duration, GAP, TE_DELTA * 4) {
                    self.reg.clear();
                    self.step = Step::SaveDuration;
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
                    if near(self.te_last, TE_LONG, TE_DELTA)
                        && near(duration, TE_SHORT * 3, TE_DELTA)
                    {
                        self.reg.add(false);
                        self.step = Step::SaveDuration;
                    } else if near(self.te_last, TE_SHORT, TE_DELTA)
                        && near(duration, TE_LONG * 2, TE_DELTA)
                    {
                        self.reg.add(true);
                        self.step = Step::SaveDuration;
                    } else if near(duration, GAP, TE_DELTA * 4) {
                        if near(self.te_last, TE_LONG, TE_DELTA) {
                            self.reg.add(false);
                        }
                        if near(self.te_last, TE_SHORT, TE_DELTA) {
                            self.reg.add(true);
                        }
                        let result = (self.reg.count as usize == MIN_COUNT_BIT)
                            .then(|| frame(self.reg.data));
                        self.reg.clear();
                        self.step = Step::Reset;
                        return result;
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

    /// 3 copies of the 24-bit key; a 4-bit `button` replaces the button nibble (ARF's D-pad).
    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        let data = if button <= 0xF {
            (decoded.data & !0xF) | u64::from(button)
        } else {
            decoded.data
        };
        Some(repeated(&upload(data), REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        let (data, _) = key_data(key, |bits| bits == MIN_COUNT_BIT)?;
        Some(frame(data))
    }

    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        Some(key_file(NAME, decoded))
    }
}

#[cfg(test)]
mod tests {
    use super::super::g2_common::test_util::*;
    use super::*;

    /// The unit-test key (`marantec24.sub`) and the samples quoted in marantec24.c.
    const KEYS: [u64; 4] = [0xAC05C4, 0xAC05C8, 0xAC05CC, 0xAC05C0];

    fn decode(wire: &[LevelDuration]) -> Vec<DecodedSignal> {
        feed_all(&mut Marantec24Decoder::new(), wire)
    }

    /// The C's upload as received (the decoder's window is centred on its gap since v1.8.7, so
    /// nothing needs shaving any more).
    fn received(wire: Vec<LevelDuration>) -> Vec<LevelDuration> {
        wire
    }

    #[test]
    fn encode_decode_roundtrip_every_button() {
        for key in KEYS {
            let d = frame(key);
            for (button, want) in [
                (0x8u8, 0x8u8),
                (0x4, 0x4),
                (0xC, 0xC),
                (0x0, 0x0),
                (0x7, 0x7),
                (0x10, (key & 0xF) as u8),
            ] {
                let tx = Marantec24Decoder::new().encode(&d, button).unwrap();
                // The exact C upload decodes (the window is centred on its gap since v1.8.7).
                assert!(!decode(&tx).is_empty());
                let mut rx = received(tx);
                rx.insert(0, LevelDuration::new(false, GAP));
                // A frame's end gap is consumed by its report (the C returns to the gap search),
                // so copies 1 and 3 are reported.
                let got = decode(&rx);
                assert_eq!(got.len(), 2, "{key:X} btn {button:X}");
                for g in &got {
                    assert_eq!((g.serial, g.button), (Some((key >> 4) as u32), Some(want)));
                    assert_eq!(g.data, (key & !0xF) | u64::from(want));
                }
            }
        }
    }

    /// Hand-built frame from the C's layout with the real remote's gap (≈15 170 µs).
    #[test]
    fn decodes_a_hand_built_frame() {
        let key = 0xAC05C8u64;
        let mut raw = vec![-15_170];
        for i in (0..24).rev() {
            let one = (key >> i) & 1 != 0;
            raw.push(if one { 790 } else { 1610 });
            raw.push(if i == 0 {
                -15_170
            } else if one {
                -3210
            } else {
                -2390
            });
        }
        let got = decode(&wire(&raw));
        assert_eq!(got.len(), 1);
        assert_eq!(
            (got[0].data, got[0].serial, got[0].button),
            (key, Some(0xAC05C), Some(0x8))
        );
        assert!(!got[0].crc_valid, "no checksum in this format");
    }

    #[test]
    fn rejects_wrong_length_and_timing() {
        let mut rx = received(upload(0xAC05C4));
        rx.insert(0, LevelDuration::new(false, GAP));
        rx.remove(1); // drop the first bit's HIGH: the frame breaks
        assert!(decode(&rx).is_empty());
        let mut rx = received(upload(0xAC05C4));
        rx.insert(0, LevelDuration::new(false, GAP));
        rx[4].duration_us = 2800; // a LOW between 2400+200 and 3200-200
        assert!(decode(&rx).is_empty());
    }

    #[test]
    fn key_file_loads_and_roundtrips() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Marantec24\nBit: 24\n\
                    Key: 00 00 00 00 00 AC 05 C4\n";
        let dec = Marantec24Decoder::new();
        let d = dec
            .from_flipper_key(&FlipperKey::parse(text).unwrap())
            .unwrap();
        assert_eq!(
            (d.data, d.serial, d.button),
            (0xAC05C4, Some(0xAC05C), Some(0x4))
        );
        assert_key_roundtrip(&dec, &d);
        assert!(dec.encode(&d, 0x4).is_some());
    }

    #[test]
    fn decodes_unit_test_capture() {
        let Some(got) = registry_decodes(&format!("{UNIT_TESTS}/marantec24_raw.sub")) else {
            return;
        };
        let m: Vec<_> = got.iter().filter(|(n, _)| n == NAME).collect();
        assert!(
            !m.is_empty(),
            "{:?}",
            got.iter().map(|(n, _)| n).collect::<Vec<_>>()
        );
        assert!(m.iter().all(|(_, d)| d.data >> 4 == 0xAC05C), "{m:?}");
    }
}
