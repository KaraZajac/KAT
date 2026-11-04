//! CAME TWEE (CAME Twin / TOP44 54-bit "rolling" fixed code) decoder + encoder.
//!
//! Port of Flipper-ARF's garage app `applications/system/subghz_garage/protocols/came_twee.c` (the
//! primary reference), with two additions from the other copies (KAT takes the union):
//! - doctor-firmware's decoder also starts on the 12 ms header of TOP44FGN remotes (12000 ± 2500 µs)
//!   besides the original 51 ms one; the garage and ARF lib copies only accept 51 ms;
//! - Flipper-ARF's lib encoder can send another button (bits 6..9 of the XOR-decoded value).
//!
//! doctor's encoder sends the 15 parcels once (`encoder.repeat` = 1); KAT follows ARF (3).
//!
//! **Air format** (AM, 433.92 MHz): Manchester, `te_short` 500 / `te_long` 1000 µs (`te_delta`
//! 250), 54 bits per parcel, each bit sent inverted (Flipper's Manchester convention: 1 = LOW then
//! HIGH). A remote sends 15 parcels `0x003FFF72 << 32 | (code ^ MAGIC[i])` for i = 14..0 (a
//! "decreasing counter"), each followed by a 51 ms LOW.
//!
//! **Decoder** (`subghz_protocol_decoder_came_twee_feed`): a header LOW (51000 ± 5000 µs, or the
//! TOP44FGN 12000 ± 2500 µs) primes Flipper's Manchester decoder (LongLow, LongHigh, ShortLow);
//! pulses within ±250 µs of 500/1000 are Manchester events, any other HIGH or short LOW resets; a
//! LOW ≥ 2250 µs ends a parcel, reported when it holds exactly 54 bits (then the decoder re-primes
//! for the next parcel). Every parcel is reported.
//!
//! **Fields** (`subghz_protocol_came_twee_remote_controller`): parcel index = low nibble; `serial`
//! = low 32 bits XOR `MAGIC[index]`; `button` = (serial / 4 >> 4) & 0xF; `counter` = the 10-bit DIP
//! code the C keeps in `generic.cnt` (bit-mirrored high half of serial / 4, >> 6). A low nibble of
//! 0xF indexes past the C's 15-entry table (undefined); KAT XORs with 0 there.
//!
//! **Encoder** (`subghz_protocol_encoder_came_twee_get_upload`, `encoder.repeat` = 3): fields from
//! the key, `button` (0..=15) written into serial bits 6..9 (ARF lib), then the 15 parcels with
//! Flipper's Manchester encoder (`manchester_encoder.c`; the final half-bit is kept only when HIGH)
//! and a 51 ms LOW after each.
//!
//! **Key file**: `Protocol: CAME TWEE`, `Bit: 54`, `Key`.

use super::common::{ManchesterEvent, ManchesterState};
use super::g1_common::{generic_fields, generic_key, push, reverse_key, with_repeats, TX_REPEAT};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "CAME TWEE",
    decoder: "CAME TWEE",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "CAME",
    category: Category::Gate,
    buttons: Some(super::g1_common::numbered_buttons),
    presets: &[&AM650],
};

const NAME: &str = "CAME TWEE";
const TE_SHORT: u32 = 500;
const TE_LONG: u32 = 1000;
const TE_DELTA: u32 = 250;
const MIN_COUNT_BIT: usize = 54;
/// The parcel mask the encoder ORs over each XORed code.
const PARCEL_MASK: u64 = 0x003F_FF72_0000_0000;
const PARCELS: usize = 15;

/// `came_twee_magic_numbers_xor` ("rainbow table").
const MAGIC: [u32; PARCELS] = [
    0x0E0E0E00, 0x1D1D1D11, 0x2C2C2C22, 0x3B3B3B33, 0x4A4A4A44, 0x59595955, 0x68686866, 0x77777777,
    0x86868688, 0x95959599, 0xA4A4A4AA, 0xB3B3B3BB, 0xC2C2C2CC, 0xD1D1D1DD, 0xE0E0E0EE,
];

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    DecoderData,
}

/// CAME TWEE decoder (`SubGhzProtocolDecoderCameTwee`).
pub struct CameTweeDecoder {
    step: Step,
    decode_data: u64,
    decode_count_bit: u8,
    manchester: ManchesterState,
}

/// `subghz_protocol_came_twee_remote_controller`: (serial, button, DIP).
fn remote_controller(data: u64) -> (u32, u8, u16) {
    let index = (data & 0xF) as usize;
    let serial = (data & 0xFFFF_FFFF) as u32 ^ MAGIC.get(index).copied().unwrap_or(0);
    let quarter = serial / 4;
    let btn = ((quarter >> 4) & 0x0F) as u8;
    let dip = reverse_key((quarter >> 16) as u64, 16) as u16 >> 6;
    (serial, btn, dip)
}

fn decoded(data: u64) -> DecodedSignal {
    let (serial, btn, dip) = remote_controller(data);
    DecodedSignal {
        serial: Some(serial),
        button: Some(btn),
        counter: Some(dip),
        crc_valid: false,
        data,
        data_count_bit: MIN_COUNT_BIT,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

/// Flipper's `ManchesterEncoderResult` (`manchester_encoder.h`).
#[derive(Debug, Clone, Copy, PartialEq)]
enum EncResult {
    ShortLow = 0b00,
    LongLow = 0b01,
    LongHigh = 0b10,
    ShortHigh = 0b11,
}

impl EncResult {
    fn from_bits(b: u8) -> Self {
        match b & 3 {
            0b00 => Self::ShortLow,
            0b01 => Self::LongLow,
            0b10 => Self::LongHigh,
            _ => Self::ShortHigh,
        }
    }

    /// `subghz_protocol_encoder_came_twee_add_duration_to_upload`.
    fn level_duration(self) -> (bool, u32) {
        match self {
            Self::ShortLow => (false, TE_SHORT),
            Self::LongLow => (false, TE_LONG),
            Self::LongHigh => (true, TE_LONG),
            Self::ShortHigh => (true, TE_SHORT),
        }
    }
}

/// Port of Flipper's `manchester_encoder.c`.
struct ManchesterEncoder {
    prev_bit: bool,
    step: u8,
}

impl ManchesterEncoder {
    fn new() -> Self {
        Self {
            prev_bit: false,
            step: 0,
        }
    }

    /// `manchester_encoder_advance`: (advanced, result).
    fn advance(&mut self, curr: bool) -> (bool, EncResult) {
        match self.step {
            0 => {
                self.prev_bit = curr;
                self.step = 1;
                (
                    true,
                    if curr {
                        EncResult::ShortLow
                    } else {
                        EncResult::ShortHigh
                    },
                )
            }
            1 => {
                let result = EncResult::from_bits((self.prev_bit as u8) << 1 | curr as u8);
                if curr == self.prev_bit {
                    self.step = 2;
                    (false, result)
                } else {
                    self.prev_bit = curr;
                    (true, result)
                }
            }
            _ => {
                self.prev_bit = curr;
                self.step = 1;
                (
                    true,
                    if curr {
                        EncResult::ShortLow
                    } else {
                        EncResult::ShortHigh
                    },
                )
            }
        }
    }

    /// `manchester_encoder_finish`.
    fn finish(&mut self) -> EncResult {
        self.step = 0;
        EncResult::from_bits((self.prev_bit as u8) << 1 | self.prev_bit as u8)
    }
}

/// `get_upload`: the 15 parcels of one transmission (C entries, equal levels merged).
fn build_upload(serial: u32) -> Vec<LevelDuration> {
    let mut out = Vec::new();
    let mut enc = ManchesterEncoder::new();
    let emit = |out: &mut Vec<LevelDuration>, r: EncResult| {
        let (level, duration) = r.level_duration();
        push(out, level, duration);
    };
    for i in (0..PARCELS).rev() {
        let parcel = PARCEL_MASK | (serial ^ MAGIC[i]) as u64;
        for b in (0..MIN_COUNT_BIT).rev() {
            let curr = (parcel >> b) & 1 == 0;
            let (advanced, r) = enc.advance(curr);
            if !advanced {
                emit(&mut out, r);
                let (_, r2) = enc.advance(curr);
                emit(&mut out, r2);
            } else {
                emit(&mut out, r);
            }
        }
        let fin = enc.finish();
        if fin.level_duration().0 {
            emit(&mut out, fin);
        }
        push(&mut out, false, TE_LONG * 51);
    }
    out
}

impl CameTweeDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            decode_data: 0,
            decode_count_bit: 0,
            manchester: ManchesterState::Mid1,
        }
    }

    /// The three events the C feeds after a header / parcel end.
    fn prime(&mut self) {
        self.manchester.advance(ManchesterEvent::LongLow);
        self.manchester.advance(ManchesterEvent::LongHigh);
        self.manchester.advance(ManchesterEvent::ShortLow);
    }
}

impl Default for CameTweeDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for CameTweeDecoder {
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
        self.manchester.advance(ManchesterEvent::Reset);
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let mut out = None;
        let mut event = None;
        match self.step {
            Step::Reset => {
                // Original TWEE: 51 ms header (all copies); TOP44FGN: 12 ms (doctor-firmware).
                if !level
                    && (duration_diff!(duration, TE_LONG * 51) < TE_DELTA * 20
                        || duration_diff!(duration, TE_LONG * 12) < TE_DELTA * 10)
                {
                    self.step = Step::DecoderData;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.prime();
                }
            }
            Step::DecoderData => {
                if !level {
                    if duration_diff!(duration, TE_SHORT) < TE_DELTA {
                        event = Some(ManchesterEvent::ShortLow);
                    } else if duration_diff!(duration, TE_LONG) < TE_DELTA {
                        event = Some(ManchesterEvent::LongLow);
                    } else if duration >= TE_LONG * 2 + TE_DELTA {
                        if self.decode_count_bit as usize == MIN_COUNT_BIT {
                            out = Some(decoded(self.decode_data));
                            // callback → the app resets the receiver (came_twee_reset).
                            self.reset();
                        }
                        self.decode_data = 0;
                        self.decode_count_bit = 0;
                        self.prime();
                    } else {
                        self.step = Step::Reset;
                    }
                } else if duration_diff!(duration, TE_SHORT) < TE_DELTA {
                    event = Some(ManchesterEvent::ShortHigh);
                } else if duration_diff!(duration, TE_LONG) < TE_DELTA {
                    event = Some(ManchesterEvent::LongHigh);
                } else {
                    self.step = Step::Reset;
                }
                if let Some(event) = event {
                    if let Some(bit) = self.manchester.advance(event) {
                        self.decode_data = self.decode_data << 1 | !bit as u64;
                        self.decode_count_bit = self.decode_count_bit.wrapping_add(1);
                    }
                }
            }
        }
        out
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        let (mut serial, _, _) = remote_controller(decoded.data);
        if button <= 0x0F {
            serial = (serial & !(0x0F << 6)) | (button as u32) << 6;
        }
        Some(with_repeats(&build_upload(serial), TX_REPEAT))
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

    /// The parcel table from came_twee.c's comment (DIP Off ON Off ON Off ON Off Off Off Off).
    const PARCELS_EXAMPLE: [u64; 15] = [
        0x003FFF72E04A6FEE,
        0x003FFF72D17B5EDD,
        0x003FFF72C2684DCC,
        0x003FFF72B3193CBB,
        0x003FFF72A40E2BAA,
        0x003FFF72953F1A99,
        0x003FFF72862C0988,
        0x003FFF7277DDF877,
        0x003FFF7268C2E766,
        0x003FFF7259F3D655,
        0x003FFF724AE0C544,
        0x003FFF723B91B433,
        0x003FFF722C86A322,
        0x003FFF721DB79211,
        0x003FFF720EA48100,
    ];

    #[test]
    fn every_parcel_gives_the_same_fields() {
        for p in PARCELS_EXAMPLE {
            assert_eq!(
                remote_controller(p),
                (0x00AA8F00, 0xC, 0b0101010000),
                "{p:016X}"
            );
        }
    }

    /// Encode → decode: all 15 parcels of a transmission come back, in order, with the fields.
    #[test]
    fn encode_decode_roundtrip() {
        let d = decoded(PARCELS_EXAMPLE[0]);
        let mut wire = vec![LevelDuration::new(false, 51_000)];
        for p in build_upload(d.serial.unwrap()) {
            push(&mut wire, p.level, p.duration_us);
        }
        let mut dec = CameTweeDecoder::new();
        let mut got = Vec::new();
        for p in &wire {
            if let Some(f) = dec.feed(p.level, p.duration_us) {
                got.push(f.data);
                dec.reset(); // the registry resets every decoder after a decode
            }
        }
        // Each decode's reset costs the next parcel (its 51 ms gap was the decode's end gap).
        let want: Vec<u64> = PARCELS_EXAMPLE.iter().copied().step_by(2).collect();
        assert_eq!(got, want);
    }

    /// The full TX upload (3 × 15 parcels) and the button switch (bits 6..9 of serial).
    #[test]
    fn encode_repeats_and_switches_buttons() {
        let d = decoded(PARCELS_EXAMPLE[3]);
        let one = build_upload(d.serial.unwrap());
        let full = CameTweeDecoder::new()
            .encode(&d, d.button.unwrap())
            .unwrap();
        // Every parcel starts with bit 53 = 1 (sent inverted: ShortHigh), so nothing merges at the
        // seams between the 51 ms gap and the next transmission.
        assert!(one[0].level);
        assert_eq!(full.len(), one.len() * 3);
        let switched = CameTweeDecoder::new().encode(&d, 0x3).unwrap();
        let got = feed_all(&mut CameTweeDecoder::new(), &switched);
        assert!(!got.is_empty());
        assert!(got
            .iter()
            .all(|f| f.button == Some(0x3) && f.counter == Some(0b0101010000)));
    }

    #[test]
    fn top44_twelve_ms_header_and_rejects() {
        let serial = remote_controller(PARCELS_EXAMPLE[5]).0;
        let one = build_upload(serial);
        // 12 ms header (doctor-firmware's TOP44FGN path).
        let mut wire = vec![LevelDuration::new(false, 12_000)];
        wire.extend(one.iter().copied());
        assert!(!feed_all(&mut CameTweeDecoder::new(), &wire).is_empty());
        // A 30 ms header is neither.
        wire[0] = LevelDuration::new(false, 30_000);
        assert!(feed_all(&mut CameTweeDecoder::new(), &wire[..120]).is_empty());
        // One bad HIGH (1600 µs) in the first parcel: that parcel is lost.
        let mut bad = vec![LevelDuration::new(false, 51_000)];
        bad.extend(one.iter().copied());
        let idx = bad.iter().position(|p| p.level).unwrap();
        bad[idx] = LevelDuration::new(true, 1600);
        let got = feed_all(&mut CameTweeDecoder::new(), &bad);
        assert!(got.iter().all(|f| f.data != PARCELS_EXAMPLE[0]));
    }

    #[test]
    fn unit_test_key_file() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: CAME TWEE\nBit: 54\n\
                    Key: 00 00 00 00 E7 1E 05 2E\n";
        let dec = CameTweeDecoder::new();
        let d = dec
            .from_flipper_key(&FlipperKey::parse(text).unwrap())
            .unwrap();
        assert_eq!(d.data, 0xE71E052E);
        let got = feed_all(
            &mut CameTweeDecoder::new(),
            &dec.encode(&d, d.button.unwrap()).unwrap(),
        );
        assert!(!got.is_empty());
        assert!(got
            .iter()
            .all(|f| (f.serial, f.button, f.counter) == (d.serial, d.button, d.counter)));
        let written = dec.to_flipper_key(&d).unwrap().to_text();
        assert!(written.contains("Protocol: CAME TWEE\nBit: 54\nKey: 00 00 00 00 E7 1E 05 2E\n"));
        let back = dec
            .from_flipper_key(&FlipperKey::parse(&written).unwrap())
            .unwrap();
        assert_eq!(format!("{back:?}"), format!("{d:?}"));
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&text.replace("Bit: 54", "Bit: 53")).unwrap())
            .is_none());
    }

    #[test]
    fn reference_raw_capture_decodes() {
        let Some(decodes) = decode_reference_capture("came_twee_raw.sub") else {
            return;
        };
        assert!(
            decodes.iter().any(|(n, _)| n == NAME),
            "{:?}",
            decodes.iter().map(|(n, _)| n).collect::<Vec<_>>()
        );
    }
}
