//! Marantec (garage doors) decoder + encoder.
//!
//! Port of Flipper-ARF `applications/system/subghz_garage/protocols/marantec.c` (the garage app
//! copy, registered in RX group g9 and the `tx_marantec` plugin). doctor-firmware's
//! `lib/subghz/protocols/marantec.c` and ARF's `lib/subghz/protocols/marantec.c` decode and build
//! the same upload (ARF's lib copy only exposes the D-pad, and every direction re-sends the
//! captured button: "the frame carries an embedded CRC8 that would need to be recomputed").
//!
//! Air format (433.92 MHz, AM): Manchester, te_short 1000 µs / te_long 2000 µs, te_delta 200
//! (strict `<`), Flipper's convention (a 1 is a LOW half then a HIGH half). A frame starts with a
//! 10 000 µs LOW header (accepted within ±1600 µs); the decoder starts with the register holding
//! the first bit (`1`) and the Manchester machine at Mid1, so the first HIGH half that follows
//! completes that bit. 49 bits, MSB first:
//!
//! ```text
//! bit 48 | 47..20       | 19..16 | 15..8  | 7..0
//!   1    | serial (hi)  | button | serial | CRC-8
//! ```
//!
//! `get_string`: Sn = `((data >> 12) & 0xFFFFFF00) | ((data >> 8) & 0xFF)` (bits 43..20 above bits
//! 15..8), Btn = `(data >> 16) & 0xF`, CRC = CRC-8 (poly 0x1D, init 0x01, MSB first) of the six
//! bytes `data >> 48 … data >> 8`, compared with the low byte. The C only *displays*
//! "CRC: … Valid/Invalid" and reports every 49-bit frame; KAT reports only CRC-valid frames (KAT
//! policy). Every known real frame carries a valid CRC: the two remotes quoted in the C
//! (`1307EDF6486C5`, `1303EFAFD8683`) and the unit-test remote (`1300710DF869F`).
//!
//! Decoder (`subghz_protocol_decoder_marantec_feed`): a LOW ≥ 4200 µs (`2·te_long + te_delta`) ends
//! the frame — reported when exactly 49 bits were collected — and restarts collection at once (the
//! gap doubles as the next header); any other out-of-tolerance pulse returns to the header search.
//!
//! Encoder (`subghz_protocol_encoder_marantec_get_upload`, `repeat` = 3): Flipper's Manchester
//! encoder over the 49 bits, except that the first half-bit of the top bit is replaced by a
//! 10 000 µs LOW (the header) and the final half-bit is only sent when it is HIGH. No counter, and
//! the button cannot be changed (the references re-send the captured frame; `button` is ignored).

use super::common::{ManchesterEvent, ManchesterState};
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
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Marantec",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "Marantec";
const TE_SHORT: u32 = 1000;
const TE_LONG: u32 = 2000;
const TE_DELTA: u32 = 200;
const MIN_COUNT_BIT: usize = 49;
const REPEAT: usize = 3;
/// Header LOW: `te_long * 5`.
const HEADER: u32 = TE_LONG * 5;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    DecoderData,
}

/// Marantec decoder (`SubGhzProtocolDecoderMarantec`).
pub struct MarantecDecoder {
    step: Step,
    reg: BitReg,
    manchester: ManchesterState,
}

/// `subghz_protocol_marantec_crc8`: poly 0x1D, init 0x01, over `data >> 48 … data >> 8`.
pub(super) fn crc8(data: u64) -> u8 {
    let mut crc = 0x01u8;
    for shift in [48, 40, 32, 24, 16, 8] {
        crc ^= (data >> shift) as u8;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0x1D
            } else {
                crc << 1
            };
        }
    }
    crc
}

fn crc_ok(data: u64) -> bool {
    crc8(data) == data as u8
}

/// The decode the C reports for a 49-bit frame (`subghz_protocol_marantec_remote_controller`).
fn frame(data: u64) -> DecodedSignal {
    let mut d = decoded(data, MIN_COUNT_BIT);
    d.serial = Some((((data >> 12) & 0xFFFF_FF00) | ((data >> 8) & 0xFF)) as u32);
    d.button = Some(((data >> 16) & 0xF) as u8);
    d.crc_valid = crc_ok(data);
    d.encoder_capable = true;
    d
}

impl MarantecDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            reg: BitReg::default(),
            manchester: ManchesterState::Mid1,
        }
    }

    /// The frame restarts with the leading `1` already in the register.
    fn restart_frame(&mut self) {
        self.reg = BitReg { data: 1, count: 1 };
        self.manchester = ManchesterState::Mid1;
    }
}

impl Default for MarantecDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// Flipper's `ManchesterEncoderState` + `manchester_encoder_advance` / `_finish`
/// (`lib/toolbox/manchester_encoder.c`). A result is (level, long).
#[derive(Default)]
struct ManchesterEncoder {
    step: u8,
    prev_bit: bool,
}

impl ManchesterEncoder {
    /// `manchester_encoder_advance`: (advanced, result). When it did not advance, the caller emits
    /// the result and calls again with the same bit.
    fn advance(&mut self, bit: bool) -> (bool, (bool, bool)) {
        match self.step {
            0 => {
                self.prev_bit = bit;
                self.step = 1;
                // ShortLow for a 1, ShortHigh for a 0.
                (true, (!bit, false))
            }
            1 => {
                // (prev << 1) + curr: 00 ShortLow, 01 LongLow, 10 LongHigh, 11 ShortHigh.
                let result = (self.prev_bit, self.prev_bit != bit);
                if bit == self.prev_bit {
                    self.step = 2;
                    (false, result)
                } else {
                    self.prev_bit = bit;
                    (true, result)
                }
            }
            _ => {
                self.prev_bit = bit;
                self.step = 1;
                (true, (!bit, false))
            }
        }
    }

    /// `manchester_encoder_finish`: ShortLow after a 0, ShortHigh after a 1.
    fn finish(&mut self) -> (bool, bool) {
        self.step = 0;
        (self.prev_bit, false)
    }
}

/// `subghz_protocol_encoder_marantec_add_duration_to_upload`.
fn half(result: (bool, bool)) -> (bool, u32) {
    (result.0, if result.1 { TE_LONG } else { TE_SHORT })
}

/// `subghz_protocol_encoder_marantec_get_upload` (one copy of the upload).
fn upload(data: u64) -> Vec<LevelDuration> {
    let bits = MIN_COUNT_BIT;
    let bit = |i: usize| (data >> i) & 1 != 0;
    let mut enc = ManchesterEncoder::default();
    let mut out = Vec::with_capacity(2 * bits + 2);
    // The top bit's first half is never emitted (step 0 always advances); the header replaces it.
    let (advanced, result) = enc.advance(bit(bits - 1));
    if !advanced {
        let (l, d) = half(result);
        push(&mut out, l, d);
        enc.advance(bit(bits - 1));
    }
    push(&mut out, false, HEADER);
    for i in (1..bits).rev() {
        let (advanced, result) = enc.advance(bit(i - 1));
        let result = if advanced {
            result
        } else {
            let (l, d) = half(result);
            push(&mut out, l, d);
            enc.advance(bit(i - 1)).1
        };
        let (l, d) = half(result);
        push(&mut out, l, d);
    }
    let (l, d) = half(enc.finish());
    if l {
        push(&mut out, l, d);
    }
    out
}

impl ProtocolDecoder for MarantecDecoder {
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
        // subghz_protocol_decoder_marantec_reset: step + Manchester state (ManchesterEventReset).
        self.step = Step::Reset;
        self.manchester = ManchesterState::Mid1;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                if !level && near(duration, HEADER, TE_DELTA * 8) {
                    self.step = Step::DecoderData;
                    self.restart_frame();
                }
                None
            }
            Step::DecoderData => {
                let mut result = None;
                let event = if !level {
                    if near(duration, TE_SHORT, TE_DELTA) {
                        Some(ManchesterEvent::ShortLow)
                    } else if near(duration, TE_LONG, TE_DELTA) {
                        Some(ManchesterEvent::LongLow)
                    } else if duration >= TE_LONG * 2 + TE_DELTA {
                        if self.reg.count as usize == MIN_COUNT_BIT {
                            // KAT gate: the C reports every 49-bit frame and only displays the CRC.
                            let d = frame(self.reg.data);
                            result = d.crc_valid.then_some(d);
                        }
                        self.restart_frame();
                        None
                    } else {
                        self.step = Step::Reset;
                        None
                    }
                } else if near(duration, TE_SHORT, TE_DELTA) {
                    Some(ManchesterEvent::ShortHigh)
                } else if near(duration, TE_LONG, TE_DELTA) {
                    Some(ManchesterEvent::LongHigh)
                } else {
                    self.step = Step::Reset;
                    None
                };
                if let Some(event) = event {
                    if let Some(bit) = self.manchester.advance(event) {
                        self.reg.add(bit);
                    }
                }
                result
            }
        }
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    /// The C encoder's 3 copies of the upload for the captured 49-bit key (no button or counter
    /// change: `button` is ignored, as in the references).
    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        (decoded.data_count_bit == MIN_COUNT_BIT).then(|| repeated(&upload(decoded.data), REPEAT))
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

    /// The unit-test key (`marantec.sub`) and the two remotes quoted in marantec.c.
    const KEYS: [u64; 3] = [0x1_3007_10DF_869F, 0x1_307E_DF64_86C5, 0x1_303E_FAFD_8683];

    fn decode(wire: &[LevelDuration]) -> Vec<DecodedSignal> {
        feed_all(&mut MarantecDecoder::new(), wire)
    }

    #[test]
    fn crc_holds_on_every_known_frame() {
        for key in KEYS {
            assert!(crc_ok(key), "{key:X}");
        }
        let d = frame(0x1_307E_DF64_86C5);
        assert_eq!((d.serial, d.button), (Some(0x07ED_F686), Some(0x4)));
    }

    /// encode → decode: each copy of the upload is reported at the next copy's header (the last
    /// one needs the silence after the transmission).
    #[test]
    fn encode_decode_roundtrip() {
        for key in KEYS {
            let d = frame(key);
            let mut wire = MarantecDecoder::new().encode(&d, 0x3).unwrap();
            assert_eq!(wire[0].duration_us, HEADER);
            assert_eq!(decode(&wire).len(), REPEAT - 1);
            wire.push(LevelDuration::new(false, 20_000));
            let got = decode(&wire);
            assert_eq!(got.len(), REPEAT);
            for g in &got {
                assert_eq!(
                    format!("{g:?}"),
                    format!("{d:?}"),
                    "button is never changed"
                );
            }
        }
    }

    /// A synthetic frame built from the C's layout (Manchester by hand, not via the encoder).
    #[test]
    fn decodes_a_hand_built_frame() {
        let key = KEYS[1];
        let mut halves = vec![(false, HEADER), (true, TE_SHORT)]; // header, 2nd half of bit 48
        for i in (0..48).rev() {
            let one = (key >> i) & 1 != 0;
            halves.push((!one, TE_SHORT));
            halves.push((one, TE_SHORT));
        }
        halves.push((false, 15_000));
        let mut w: Vec<LevelDuration> = Vec::new();
        for (l, d) in halves {
            push(&mut w, l, d);
        }
        let got = decode(&w);
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].data, key);
        assert!(got[0].crc_valid);
    }

    #[test]
    fn rejects_bad_crc_and_bad_timing() {
        let good = KEYS[0];
        let mut wire = upload(good ^ 0x100); // one data bit flipped, CRC not updated
        wire.push(LevelDuration::new(false, 20_000));
        assert!(
            decode(&wire).is_empty(),
            "CRC-invalid frame must not be reported"
        );
        // Stretch one data pulse beyond te_long + te_delta: the frame is dropped.
        let mut wire = upload(good);
        wire[10].duration_us = TE_LONG + TE_DELTA + 50;
        wire.push(LevelDuration::new(false, 20_000));
        assert!(decode(&wire).is_empty());
    }

    #[test]
    fn key_file_loads_and_roundtrips() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Marantec\nBit: 49\n\
                    Key: 00 01 30 07 10 DF 86 9F\n";
        let dec = MarantecDecoder::new();
        let d = dec
            .from_flipper_key(&FlipperKey::parse(text).unwrap())
            .unwrap();
        assert_eq!((d.data, d.data_count_bit, d.crc_valid), (KEYS[0], 49, true));
        assert_eq!(
            (d.serial, d.button),
            (Some(0x0071_0D86), Some(0xF)),
            "Sn: 0x0710D86"
        );
        assert_key_roundtrip(&dec, &d);
        let key = dec.to_flipper_key(&d).unwrap();
        assert_eq!(
            (key.protocol.as_str(), key.bits, key.fields.len()),
            ("Marantec", 49, 0)
        );
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&text.replace("Bit: 49", "Bit: 48")).unwrap())
            .is_none());
    }

    /// Flipper unit test `marantec_raw.sub` (REFERENCES): the registry reports Marantec frames of
    /// the unit-test remote.
    #[test]
    fn decodes_unit_test_capture() {
        let Some(got) = registry_decodes(&format!("{UNIT_TESTS}/marantec_raw.sub")) else {
            return;
        };
        let m: Vec<_> = got.iter().filter(|(n, _)| n == NAME).collect();
        assert!(
            !m.is_empty(),
            "{:?}",
            got.iter().map(|(n, _)| n).collect::<Vec<_>>()
        );
        assert!(m.iter().all(|(_, d)| d.data == KEYS[0] && d.crc_valid));
    }
}
