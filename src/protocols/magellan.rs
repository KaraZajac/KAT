//! Magellan: Paradox Magellan wireless security sensors and remotes (door contacts, motion and smoke
//! detectors, REM1 / REM2 keyfobs): a 32-bit frame of a 16-bit serial, an 8-bit event and a CRC-8,
//! decoder + encoder.
//!
//! Port of doctor-firmware `lib/subghz/protocols/magellan.c` (the stock Flipper protocol,
//! registered by doctor, Momentum, RogueMaster and FoxFW; Flipper-ARF ships its `lib` and garage-app
//! copies unregistered). Every copy decodes, checks and encodes identically, and every copy has the
//! encoder (`SubGhzProtocolFlag_Send`; FoxFW loads it from an SD plugin). They differ in style
//! (`common.c` helpers in doctor / RogueMaster / FoxFW) and in the display (doctor / Momentum /
//! RogueMaster spell out the event, ARF shows `SN:0x… Btn:…`). ARF's `lib` copy turns on the
//! custom-button D-pad but re-sends the captured frame on every direction ("the embedded CRC8 …
//! would need recomputation for a different event code"); no copy switches events. Unit-test
//! captures: `magellan_raw.sub`, `magellan.sub`.
//!
//! **Air format** (AM, 433.92 MHz): te_short = 200 µs, te_long = 400 µs. A preamble HIGH 800 + LOW
//! 200, twelve HIGH 200 + LOW 200 pairs and HIGH 200 + LOW 400; a start bit HIGH 1200 + LOW 400;
//! 32 bits MSB first, each a HIGH then a LOW: 1 = HIGH 200 + LOW 400, 0 = HIGH 400 + LOW 200; a
//! stop bit HIGH 200 + LOW 40 000.
//!
//! **Decoder** (`subghz_protocol_decoder_magellan_feed`): Reset waits for a HIGH 200 ± 100; then
//! HIGH (saved) + LOW pairs: 200 ± 100 + 200 ± 100 counts a preamble pair, 200 ± 100 + 400 ± 200
//! after more than 10 pairs ends the preamble; the start bit is HIGH 1200 ± 300 + LOW 400 ± 200;
//! then bits (HIGH 200 ± 100 + LOW 400 ± 100 = 1, HIGH 400 ± 100 + LOW 200 ± 100 = 0). A LOW ≥ 1200
//! after a bit's HIGH ends the frame: exactly 32 bits whose low byte is the CRC-8 (poly 0x31, init
//! 0) of the three bytes above it are reported. Anything else resets.
//!
//! **Fields** (`subghz_protocol_magellan_check_remote_controller`): the 24 bits above the CRC,
//! bit-reversed: serial = the low 16 bits, event = the high 8 bits (the C's `btn`; type in the high
//! nibble — Door, Motion, Smoke Alarm, REM1, REM2 — and status in the low one, labelled as the C's
//! `get_event_serialize`). `crc_valid` = the CRC holds (always for decodes).
//!
//! **Encoder** (`subghz_protocol_encoder_magellan_get_upload`, `encoder.repeat` = 3): preamble, start
//! bit, the 32 bits of the key as given, stop bit. `button` is ignored: the captured frame (and its
//! CRC) is sent, as in every reference.
//!
//! **Key file**: `Protocol: Magellan`, `Bit: 32` (required), `Key`.

use std::sync::OnceLock;

use super::f_common::{add_bit, push, with_repeats};
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

/// Metadata (see `info.rs`): `SubGhzProtocolFlag_433 | AM` (+ Sensors), `SubGhzProtocolTypeStatic`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Paradox",
    category: Category::Gate,
    buttons: Some(event_label),
    presets: &[&AM650],
};

pub(super) const NAME: &str = "Magellan";
const TE_SHORT: u32 = 200;
const TE_LONG: u32 = 400;
const TE_DELTA: u32 = 100;
const MIN_COUNT_BIT: usize = 32;
/// `subghz_protocol_encoder_common_alloc(…, 3, 256)`.
const TX_REPEAT: usize = 3;

/// `subghz_protocol_magellan_get_event_serialize` (the code, not its comment: type 0x6 is "REM2 -
/// Off1", 0x7 is "Unknown").
fn event_text(event: u8) -> String {
    let ty = event >> 4;
    let event_type = match ty {
        0x0 => "Nothing",
        0x1 => "Door",
        0x2 => "Motion",
        0x3 => "Smoke Alarm",
        0x4 => "REM1",
        0x5 => return "REM1 - Off1".to_string(),
        0x6 => return "REM2 - Off1".to_string(),
        _ => "Unknown",
    };
    let remote = ty > 0x3;
    let subtype = match event & 0xF {
        0x0 => if remote { "Arm1" } else { "Sealed" },
        0x1 => if remote { "Btn1" } else { "Alarm" },
        0x2 => if remote { "Btn2" } else { "Tamper" },
        0x3 => if remote { "Btn3" } else { "Alarm + Tamper" },
        0x8 => "Reset",
        0x9 => "LowBatt",
        0xA => "BattOk",
        0xB => "Learn",
        _ => "Unknown",
    };
    format!("{event_type} - {subtype}")
}

/// The event's label (`Stat:` in the C's text) for the button column.
fn event_label(_name: &str, code: u8) -> Option<&'static str> {
    static LABELS: OnceLock<Vec<String>> = OnceLock::new();
    Some(&LABELS.get_or_init(|| (0..=255u8).map(event_text).collect())[usize::from(code)])
}

/// `subghz_protocol_magellan_crc8`: CRC-8, poly 0x31, init 0, MSB first.
fn crc8(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |mut crc, &b| {
        crc ^= b;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 { (crc << 1) ^ 0x31 } else { crc << 1 };
        }
        crc
    })
}

/// `subghz_protocol_magellan_check_crc`: the low byte is the CRC of the three above it.
fn crc_ok(data: u64) -> bool {
    (data & 0xFF) as u8 == crc8(&[(data >> 24) as u8, (data >> 16) as u8, (data >> 8) as u8])
}

/// (serial, event): the 24 bits above the CRC reversed (`subghz_protocol_blocks_reverse_key`).
fn fields(data: u64) -> (u32, u8) {
    let rev = ((data >> 8) as u32 & 0xFF_FFFF).reverse_bits() >> 8;
    (rev & 0xFFFF, (rev >> 16) as u8)
}

/// A decode (or a loaded key) of `data` (32 bits).
fn decoded(data: u64) -> DecodedSignal {
    let (serial, event) = fields(data);
    DecodedSignal {
        serial: Some(serial),
        button: Some(event),
        counter: None,
        crc_valid: crc_ok(data),
        data,
        data_count_bit: MIN_COUNT_BIT,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

/// One `get_upload` frame.
fn build_frame(data: u64) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(32 + MIN_COUNT_BIT * 2);
    let pair = |out: &mut Vec<LevelDuration>, high: u32, low: u32| {
        push(out, true, high);
        push(out, false, low);
    };
    pair(&mut out, TE_SHORT * 4, TE_SHORT);
    for _ in 0..12 {
        pair(&mut out, TE_SHORT, TE_SHORT);
    }
    pair(&mut out, TE_SHORT, TE_LONG);
    // start bit
    pair(&mut out, TE_LONG * 3, TE_LONG);
    for i in (0..MIN_COUNT_BIT).rev() {
        if (data >> i) & 1 == 1 {
            pair(&mut out, TE_SHORT, TE_LONG);
        } else {
            pair(&mut out, TE_LONG, TE_SHORT);
        }
    }
    // stop bit
    pair(&mut out, TE_SHORT, TE_LONG * 100);
    out
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    CheckPreambula,
    FoundPreambula,
    SaveDuration,
    CheckDuration,
}

/// Magellan decoder (`SubGhzProtocolDecoderMagellan`).
pub struct MagellanDecoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
    header_count: u16,
}

impl MagellanDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
            header_count: 0,
        }
    }
}

impl Default for MagellanDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for MagellanDecoder {
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

    /// `subghz_protocol_decoder_common_reset`: the parser step only.
    fn reset(&mut self) {
        self.step = Step::Reset;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let near = |d: u32, te: u32, delta: u32| duration_diff!(d, te) < delta;
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
                    self.te_last = duration;
                } else if near(self.te_last, TE_SHORT, TE_DELTA) && near(duration, TE_SHORT, TE_DELTA) {
                    // found header
                    self.header_count = self.header_count.wrapping_add(1);
                } else if near(self.te_last, TE_SHORT, TE_DELTA)
                    && near(duration, TE_LONG, TE_DELTA * 2)
                    && self.header_count > 10
                {
                    self.step = Step::FoundPreambula;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::FoundPreambula => {
                if level {
                    self.te_last = duration;
                } else if near(self.te_last, TE_SHORT * 6, TE_DELTA * 3) && near(duration, TE_LONG, TE_DELTA * 2) {
                    self.step = Step::SaveDuration;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
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
                if level {
                    self.step = Step::Reset;
                } else if near(self.te_last, TE_SHORT, TE_DELTA) && near(duration, TE_LONG, TE_DELTA) {
                    add_bit(&mut self.decode_data, &mut self.decode_count_bit, true);
                    self.step = Step::SaveDuration;
                } else if near(self.te_last, TE_LONG, TE_DELTA) && near(duration, TE_SHORT, TE_DELTA) {
                    add_bit(&mut self.decode_data, &mut self.decode_count_bit, false);
                    self.step = Step::SaveDuration;
                } else if duration >= TE_LONG * 3 {
                    // found the stop bit
                    let out = (usize::from(self.decode_count_bit) == MIN_COUNT_BIT && crc_ok(self.decode_data))
                        .then(|| decoded(self.decode_data));
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.step = Step::Reset;
                    return out;
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
        (decoded.data_count_bit == MIN_COUNT_BIT).then(|| with_repeats(&build_frame(decoded.data), TX_REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        (key.bits == MIN_COUNT_BIT).then(|| decoded(key.key))
    }

    fn to_flipper_key(&self, d: &DecodedSignal) -> Option<FlipperKey> {
        (d.data_count_bit == MIN_COUNT_BIT).then(|| FlipperKey::for_protocol(NAME, MIN_COUNT_BIT, d.data))
    }
}

#[cfg(test)]
pub(super) mod testing {
    //! Frames for other modules' tests and the parity tools.
    use super::*;

    /// The 32-bit frame of `serial` and `event`, its CRC filled in.
    pub fn frame(serial: u16, event: u8) -> DecodedSignal {
        let rev = (u32::from(event) << 16 | u32::from(serial)).reverse_bits() >> 8;
        let data = u64::from(rev) << 8;
        decoded(data | u64::from(crc8(&[(data >> 24) as u8, (data >> 16) as u8, (data >> 8) as u8])))
    }

    /// A key-file decode of any 32-bit `data` (the CRC as it is).
    pub fn raw(data: u64) -> DecodedSignal {
        decoded(data & 0xFFFF_FFFF)
    }
}

#[cfg(test)]
mod tests {
    use super::super::f_common::test_support::*;
    use super::*;

    const UNIT_TEST_KEY: &str = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
        Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Magellan\nBit: 32\nKey: 00 00 00 00 37 AE 48 28\n";

    #[test]
    fn fields_crc_and_labels_of_the_c_example() {
        // The C's comment: 0x37AE4828 → 0x37AE48 reversed = 0x1275EC: event 0x12, serial 0x75EC.
        let d = testing::raw(0x37AE_4828);
        assert_eq!((d.serial, d.button, d.crc_valid), (Some(0x75EC), Some(0x12), true));
        assert_eq!(testing::frame(0x75EC, 0x12), d);
        assert_eq!(event_label(NAME, 0x12), Some("Door - Tamper"));
        assert_eq!(event_label(NAME, 0x13), Some("Door - Alarm + Tamper"));
        assert_eq!(event_label(NAME, 0x41), Some("REM1 - Btn1"));
        assert_eq!(event_label(NAME, 0x58), Some("REM1 - Off1"));
        assert_eq!(event_label(NAME, 0x6B), Some("REM2 - Off1"));
        assert_eq!(event_label(NAME, 0x70), Some("Unknown - Arm1"));
        assert_eq!(event_label(NAME, 0x0C), Some("Nothing - Unknown"));
        assert!(!testing::raw(0x37AE_4829).crc_valid);
    }

    #[test]
    fn encode_decode_roundtrip() {
        for (serial, event) in [(0x75ECu16, 0x12u8), (0x75EC, 0x13), (0, 0), (0xFFFF, 0xFF), (0x1234, 0x4B)] {
            let d = testing::frame(serial, event);
            // Every copy decodes (each starts with its preamble, the stop bit's LOW ends it).
            let got = feed_all(&mut MagellanDecoder::new(), &MagellanDecoder::new().encode(&d, 0).unwrap());
            assert_eq!(got, vec![d.clone(); 3], "{serial:04X}/{event:02X}");
        }
    }

    #[test]
    fn upload_layout() {
        let up = MagellanDecoder::new().encode(&testing::raw(0x8000_0001), 0).unwrap();
        let p = |i: usize| (up[i].level, up[i].duration_us);
        assert_eq!((p(0), p(1)), ((true, 800), (false, 200)));
        assert_eq!((p(26), p(27)), ((true, 200), (false, 400)));
        assert_eq!((p(28), p(29)), ((true, 1200), (false, 400)));
        assert_eq!((p(30), p(31)), ((true, 200), (false, 400))); // bit 31 = 1
        assert_eq!((p(32), p(33)), ((true, 400), (false, 200))); // bit 30 = 0
        assert_eq!((p(94), p(95)), ((true, 200), (false, 40_000)));
        // 96 entries a copy, 3 copies: 288 pulses, as the C.
        assert_eq!(up.len(), 288);
    }

    #[test]
    fn rejects_bad_crc_short_preamble_and_wrong_length() {
        let good = testing::frame(0x75EC, 0x12);
        let wire = MagellanDecoder::new().encode(&good, 0).unwrap();
        assert_eq!(feed_all(&mut MagellanDecoder::new(), &wire).len(), 3);
        // A wrong CRC: the C drops the frame (the encoder sends it as it is).
        let bad = MagellanDecoder::new().encode(&testing::raw(good.data ^ 1), 0).unwrap();
        assert!(feed_all(&mut MagellanDecoder::new(), &bad).is_empty());
        // More than 10 preamble pairs are needed: 10 (cut two of the twelve from every copy).
        let cut: Vec<LevelDuration> = wire
            .chunks(96)
            .flat_map(|c| c[..2].iter().chain(&c[6..]).copied().collect::<Vec<_>>())
            .collect();
        assert!(feed_all(&mut MagellanDecoder::new(), &cut).is_empty());
        let cut1: Vec<LevelDuration> = wire
            .chunks(96)
            .flat_map(|c| c[..2].iter().chain(&c[4..]).copied().collect::<Vec<_>>())
            .collect();
        assert_eq!(feed_all(&mut MagellanDecoder::new(), &cut1).len(), 3, "11 pairs are enough");
        // 31 bits (a bit pair removed): dropped.
        let mut short = wire[..96].to_vec();
        short.drain(30..32);
        assert!(feed_all(&mut MagellanDecoder::new(), &short).is_empty());
        // The stop LOW must be ≥ 1200 µs.
        let mut w = wire[..96].to_vec();
        w[95].duration_us = 1199;
        assert!(feed_all(&mut MagellanDecoder::new(), &w).is_empty());
        w[95].duration_us = 1200;
        assert_eq!(feed_all(&mut MagellanDecoder::new(), &w).len(), 1);
    }

    #[test]
    fn key_file_roundtrip() {
        let dec = MagellanDecoder::new();
        let d = dec.from_flipper_key(&FlipperKey::parse(UNIT_TEST_KEY).unwrap()).unwrap();
        assert_eq!((d.data, d.data_count_bit, d.serial, d.button), (0x37AE_4828, 32, Some(0x75EC), Some(0x12)));
        let text = super::super::flipper_key::testing::assert_round_trip(&dec, &d);
        assert!(text.contains("Protocol: Magellan\nBit: 32\nKey: 00 00 00 00 37 AE 48 28\n"), "{text}");
        assert!(dec.from_flipper_key(&FlipperKey::parse(&UNIT_TEST_KEY.replace("Bit: 32", "Bit: 24")).unwrap()).is_none());
    }

    /// Flipper's `subghz_decoder_magellan_test` capture through the whole registry: 8 frames (as
    /// doctor / ARF / Momentum / RogueMaster / FoxFW), all 0x37AEC852 (event 0x13 "Door - Alarm +
    /// Tamper", serial 117/236), nothing else.
    #[test]
    fn unit_test_capture_decodes() {
        let Some(path) = reference_file(ARF_UNIT_TESTS, "magellan_raw.sub") else {
            return;
        };
        let got = registry_decodes(&path);
        let ours: Vec<&DecodedSignal> = got.iter().filter(|(n, _)| n == NAME).map(|(_, d)| d).collect();
        assert_eq!(ours.len(), 8);
        assert!(ours.iter().all(|d| d.data == 0x37AE_C852 && d.serial == Some(0x75EC) && d.button == Some(0x13)));
        assert_eq!(got.len(), ours.len(), "another protocol took a frame");
    }

    /// `subghz_encoder_magellan_test`: the unit-test key's upload decodes back three times through
    /// the registry, as the C.
    #[test]
    fn unit_test_key_encodes_back() {
        let reg = super::super::ProtocolRegistry::new();
        let (name, d) = reg.decode_key(&FlipperKey::parse(UNIT_TEST_KEY).unwrap()).unwrap();
        assert_eq!(name, NAME);
        let tx = reg.get(&name).unwrap().encode(&d, 0).unwrap();
        let mut reg = super::super::ProtocolRegistry::new();
        let back = reg.process_signal_stream(&tx, 433_920_000);
        assert_eq!(back.len(), 3, "{back:?}");
        assert!(back.iter().all(|(n, x, _)| n == NAME && x.data == 0x37AE_4828), "{back:?}");
    }

    #[test]
    fn jittered_frames_decode() {
        let mut rng = Rng(0x4A6E_11A2);
        for _ in 0..50 {
            let d = testing::frame(rng.next() as u16, rng.next() as u8);
            let wire = jitter(&MagellanDecoder::new().encode(&d, 0).unwrap(), &mut rng, 50);
            let got = feed_all(&mut MagellanDecoder::new(), &wire);
            assert!(!got.is_empty(), "{:08X}", d.data);
            assert!(got.iter().all(|x| *x == d));
        }
    }
}
