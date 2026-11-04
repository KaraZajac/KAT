//! Dickert MAHS decoder + encoder.
//!
//! Port of Flipper-ARF `applications/system/subghz_garage/protocols/dickert_mahs.c` (garage app,
//! RX group g9 + `tx_dickert_mahs`); doctor-firmware's copy decodes and encodes identically (it
//! also prints the DIP switches).
//!
//! Air format (433.92 MHz, AM): te_short 400 µs / te_long 800 µs, te_delta 100. A frame is a LOW
//! header of `50·te_long` = 40 000 µs (accepted ± 7000; the encoder sends `112·te_short` = 44 800
//! µs), a HIGH start pulse of te_short, then bits as LOW/HIGH pairs whose sum is 1200 ± 100 µs: a
//! LOW of te_long is a `1`, a LOW of te_short a `0` (a pair with a LOW in neither window adds no
//! bit). The first pair that does not fit ends the frame; on the *next* pulse the C reports what
//! it collected when that is at least 36 bits (`>=`), MSB first (only the last 64 bits stay in the
//! register).
//!
//! Content: 18 three-state switches, 2 bits each (`11` +, `01` 0, `00` −; `10` is shown as `?`):
//! 10 user DIP switches then the 8-symbol factory code, which doctor-firmware displays. No
//! checksum, no button field (the C clears serial and button).
//!
//! **Reset**: the C's `reset` only rewinds the step and keeps the bit count, so a frame that
//! completed just before a receiver reset is reported on the next pulse. KAT's `reset` also clears
//! the count and the pair buffer, so a frame never leaks into the next capture (the registry is
//! reused across captures).
//!
//! Encoder (`repeat` = 3, no counter, `button` ignored): header, start pulse, the bits (`1` = LOW
//! 800 + HIGH 400, `0` = LOW 400 + HIGH 800). Keys of 36..=63 bits load and send (the C's
//! `deserialize` accepts ≥ 36 bits and its 128-entry upload holds 63).

use super::g2_common::{bit, decoded, key_data, key_file, near, push, repeated, BitReg};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`): `SubGhzProtocolFlag_433 | AM`, static code (DIP switches).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Dickert",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "Dickert_MAHS";
const TE_SHORT: u32 = 400;
const TE_LONG: u32 = 800;
const TE_DELTA: u32 = 100;
const MIN_COUNT_BIT: usize = 36;
const REPEAT: usize = 3;
/// The 128-entry upload holds the header, the start pulse and 63 bits.
const MAX_TX_BITS: usize = 63;
const HEADER: u32 = TE_LONG * 50;
const TX_HEADER: u32 = TE_SHORT * 112;
const PAIR: u32 = 1200;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    Initial,
    Recording,
}

/// Dickert MAHS decoder (`SubGhzProtocolDecoderDickertMAHS`).
pub struct DickertMahsDecoder {
    step: Step,
    reg: BitReg,
    tmp: [u32; 2],
    tmp_cnt: u8,
    /// Set when the reporting pulse is also a header: the C checks the header *after* its
    /// callback (where the receiver resets every decoder), so the reset that follows resumes at
    /// the start pulse.
    resume_initial: bool,
}

fn frame(data: u64, bits: usize) -> DecodedSignal {
    let mut d = decoded(data, bits);
    d.encoder_capable = (MIN_COUNT_BIT..=MAX_TX_BITS).contains(&bits);
    d
}

impl DickertMahsDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            reg: BitReg::default(),
            tmp: [0; 2],
            tmp_cnt: 0,
            resume_initial: false,
        }
    }
}

impl Default for DickertMahsDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// `subghz_protocol_encoder_dickert_mahs_get_upload` (one copy).
fn upload(data: u64, bits: usize) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(2 * bits + 2);
    push(&mut out, false, TX_HEADER);
    push(&mut out, true, TE_SHORT);
    for i in (0..bits).rev() {
        let one = bit(data, i);
        push(&mut out, false, if one { TE_LONG } else { TE_SHORT });
        push(&mut out, true, if one { TE_SHORT } else { TE_LONG });
    }
    out
}

impl ProtocolDecoder for DickertMahsDecoder {
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
        // The C rewinds the step only; see the module doc.
        self.step = if std::mem::take(&mut self.resume_initial) {
            Step::Initial
        } else {
            Step::Reset
        };
        self.reg.clear();
        self.tmp_cnt = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        self.resume_initial = false;
        let mut result = None;
        match self.step {
            Step::Reset => {
                let bits = self.reg.count as usize;
                if bits >= MIN_COUNT_BIT {
                    result = Some(frame(self.reg.data, bits));
                    self.reg.clear();
                }
                if !level && near(duration, HEADER, TE_DELTA * 70) {
                    self.step = Step::Initial;
                    self.resume_initial = result.is_some();
                }
            }
            Step::Initial => {
                if !level {
                    // A LOW is ignored here.
                } else if near(duration, TE_SHORT, TE_DELTA) {
                    self.step = Step::Recording;
                    self.reg.clear();
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::Recording => {
                if (!level && self.tmp_cnt == 0) || (level && self.tmp_cnt == 1) {
                    self.tmp[self.tmp_cnt as usize] = duration;
                    self.tmp_cnt += 1;
                    if self.tmp_cnt == 2 {
                        self.tmp_cnt = 0;
                        if near(self.tmp[0].wrapping_add(self.tmp[1]), PAIR, TE_DELTA) {
                            if near(self.tmp[0], TE_LONG, TE_DELTA) {
                                self.reg.add(true);
                            } else if near(self.tmp[0], TE_SHORT, TE_DELTA) {
                                self.reg.add(false);
                            }
                        } else {
                            self.step = Step::Reset;
                        }
                    }
                } else {
                    self.tmp_cnt = 0;
                    self.step = Step::Reset;
                }
            }
        }
        result
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        let bits = decoded.data_count_bit;
        (MIN_COUNT_BIT..=MAX_TX_BITS)
            .contains(&bits)
            .then(|| repeated(&upload(decoded.data, bits), REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        let (data, bits) = key_data(key, |bits| bits >= MIN_COUNT_BIT)?;
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

    /// The unit-test key (`dickert_mahs.sub`).
    const KEY: u64 = 0x1_5557_5515;

    #[test]
    fn encode_decode_roundtrip() {
        for (key, bits) in [
            (KEY, 36),
            (0xF_FFFF_FFFF, 36),
            (0x0_0000_0000, 36),
            (0x7FFF_FFFF_FFFF_FFFF, 63),
        ] {
            let d = frame(key, bits);
            assert!(d.encoder_capable);
            let tx = DickertMahsDecoder::new().encode(&d, 0x2).unwrap();
            assert_eq!(
                (tx[0].duration_us, tx[1].duration_us),
                (TX_HEADER, TE_SHORT)
            );
            // Copy n ends at copy n+1's header + start pulse and is reported on the pulse after;
            // the C then waits for a header, so copy n+1 is lost: copies 1 and 3 (with silence).
            let mut rx = tx.clone();
            rx.extend([
                LevelDuration::new(false, TX_HEADER),
                LevelDuration::new(true, TE_SHORT),
                LevelDuration::new(false, 400),
            ]);
            let got = feed_all(&mut DickertMahsDecoder::new(), &rx);
            assert_eq!(got.len(), 2, "{key:X}");
            for g in &got {
                assert_eq!(format!("{g:?}"), format!("{d:?}"));
            }
        }
        assert!(
            DickertMahsDecoder::new().encode(&frame(0, 64), 0).is_none(),
            "upload holds 63 bits"
        );
        assert!(DickertMahsDecoder::new().encode(&frame(0, 35), 0).is_none());
    }

    #[test]
    fn hand_built_frame_and_rejections() {
        let mut raw = vec![-40_000, 410];
        for i in (0..36).rev() {
            raw.extend(if (KEY >> i) & 1 != 0 {
                [-790, 410]
            } else {
                [-410, 790]
            });
        }
        raw.extend([-40_000, 400, -800]);
        let got = feed_all(&mut DickertMahsDecoder::new(), &wire(&raw));
        assert_eq!(got.len(), 1);
        assert_eq!((got[0].data, got[0].data_count_bit), (KEY, 36));
        // A pair summing outside 1200 ± 100 ends the frame early (34 bits): nothing reported.
        let mut bad = raw.clone();
        bad[71] = 500;
        assert!(feed_all(&mut DickertMahsDecoder::new(), &wire(&bad)).is_empty());
        // Header too short.
        let mut bad = raw.clone();
        bad[0] = -30_000;
        assert!(feed_all(&mut DickertMahsDecoder::new(), &wire(&bad)).is_empty());
    }

    #[test]
    fn reset_does_not_leak_a_frame_into_the_next_capture() {
        let mut raw = vec![-40_000, 400];
        for i in (0..36).rev() {
            raw.extend(if (KEY >> i) & 1 != 0 {
                [-800, 400]
            } else {
                [-400, 800]
            });
        }
        let mut dec = DickertMahsDecoder::new();
        assert!(
            feed_all(&mut dec, &wire(&raw)).is_empty(),
            "frame not terminated yet"
        );
        dec.reset();
        assert!(dec.feed(false, 400).is_none());
    }

    #[test]
    fn key_file_loads_and_roundtrips() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Dickert_MAHS\nBit: 36\n\
                    Key: 00 00 00 01 55 57 55 15\n";
        let dec = DickertMahsDecoder::new();
        let d = dec
            .from_flipper_key(&FlipperKey::parse(text).unwrap())
            .unwrap();
        assert_eq!(
            (d.data, d.data_count_bit, d.encoder_capable),
            (KEY, 36, true)
        );
        assert_key_roundtrip(&dec, &d);
        let long = dec
            .from_flipper_key(&FlipperKey::parse(&text.replace("Bit: 36", "Bit: 40")).unwrap())
            .unwrap();
        assert_eq!(long.data_count_bit, 40);
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&text.replace("Bit: 36", "Bit: 35")).unwrap())
            .is_none());
    }

    #[test]
    fn decodes_unit_test_capture() {
        let Some(got) = registry_decodes(&format!("{UNIT_TESTS}/dickert_raw.sub")) else {
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
