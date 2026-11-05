//! Hörmann BiSecur (868 MHz garage-door remotes): 176-bit FSK Manchester frames.
//!
//! Reference: Momentum-Firmware `lib/subghz/protocols/hormann_bisecur.c`; RogueMaster's copy is
//! byte-identical. RogueMaster registers it; Momentum (and Unleashed) ship it commented out of
//! their registry ("disabled ... due to flash space constraints and very limited usefulness",
//! Momentum's CHANGELOG), which KAT does not share. The Flipper unit-test resources of both trees
//! carry two key files and one RAW capture (no unit test references them).
//!
//! **Air format** (FM, 868 MHz): te_short = 208 µs, te_long = 416 µs, te_delta = 104 µs. Preamble:
//! 21 LOW/HIGH pairs of te_short (the first LOW 312 µs, the last HIGH 4·te_long = 1664 µs), then
//! two LOW/HIGH pairs of te_long, then 176 bits of Manchester (Flipper `manchester_encoder`, bits
//! inverted: a 0 bit is sent LOW then HIGH), MSB first, the last half-bit stretched by 312 µs.
//!
//! **Decoder** (`subghz_protocol_decoder_hormann_bisecur_feed`): Reset → a LOW of |d − 312| < 104
//! → any number of |d − 208| < 104 pulses → a HIGH of |d − 1664| < 104 → a LOW of |d − 416| < 104
//! → HIGH, LOW, HIGH of |d − 416| < 104 (the C's `sync_cnt` countdown) → data. Data pulses are
//! short (|d − 208| < 104, or anything longer than 208 µs for the 176th bit) or long
//! (|d − 416| < 104), events `level ? ShortLow/LongLow : ShortHigh/LongHigh`, fed to the C's own
//! Manchester table (`manchester_advance_alt`, CodeAllNight's fix of the toolbox decoder; ported
//! as [advance_alt]). Any other pulse resets the decoder (data cleared). The 176th bit completes
//! the frame: the C calls back (the receiver then resets the decoder).
//!
//! **Frame** (22 bytes, `subghz_protocol_hormann_bisecur_parse_data`): byte 0 = type (0x50 or
//! 0x70), bytes 1..4 = serial, bytes 5..12 and 13..20 = two 64-bit encrypted (rolling) blocks the
//! C shows as `Key:` lines, byte 21 = CRC: type 0x50 → !(CRC-8 poly 0x07 init 0x00 over bytes
//! 1..20 ^ 0x55), type 0x70 → reflected CRC-8 (poly 0x07, init 0xFF) over bytes 0..20. The C's
//! display shows "CRC OK" / "WRONG" (any other type is WRONG); its decoder reports every frame.
//!
//! **KAT fields** (176 bits do not fit `data` + `extra`, so KAT keeps the C's own split):
//! `serial` = bytes 1..4 (the C's `generic.serial`), `data` = bytes 5..12 (`generic.data`),
//! `extra` = bytes 13..20 (`generic.data_2`), `button` = byte 0, the frame type (labelled
//! "Type 0x50" / "Type 0x70"), `counter` None (encrypted), `crc_valid` = the CRC holds, and byte
//! 21 is that CRC. KAT reports only frames whose CRC holds (KAT's policy for a check the C
//! displays; both real captures pass) and, for the same reason, loads only key files whose CRC
//! holds — such a file is all KAT could re-send without the CRC byte (the C loads and re-sends any
//! 176-bit key).
//!
//! **Encoder** (`subghz_protocol_encoder_hormann_bisecur_get_upload`): no key and no counter — it
//! re-sends the stored frame, as the C (Dynamic type; KAT: "Send next code" = replay of the
//! decoded frame, not a new code). Upload = the preamble and sync above, the 176 Manchester bits
//! (`!bit` through `manchester_encoder`), the closing half-bit + 312 µs; `encoder.repeat` = 1 (the
//! C: "original remote does 3 repeats", left at 1 for the 504 ms carrier-off gap it cannot send).
//! `button` is ignored.
//!
//! **Key file**: `Protocol: Hormann BiSecur`, `Bit: 176`, `Key` = the 22 bytes
//! (`subghz_block_generic_serialize`, Key updated with all 22 bytes); loading needs `Bit: 176`.

use super::common::{ManchesterEvent, ManchesterState};
use super::f_common::push;
use super::g3_common::ManchesterEncoder;
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
/// `SubGhzProtocolFlag_868 | FM`, `SubGhzProtocolTypeDynamic`. FSK preset: F3 (±31.74 kHz); the
/// references' capture preset (DEVIATN 0x40, ±25.39 kHz) sits exactly between F2 and F3, and KAT
/// transmits with a capture's own preset whenever it has one.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::FM,
    encryption: "AES-128",
    make: "Hormann",
    category: Category::Gate,
    buttons: Some(type_label),
    presets: &[&F3],
};

const NAME: &str = "Hormann BiSecur";
const TE_SHORT: u32 = 208;
const TE_LONG: u32 = 416;
const TE_DELTA: u32 = 104;
const HALF_SHORT: u32 = TE_SHORT / 2;
const BIT_COUNT: usize = 176;
const LEN: usize = BIT_COUNT / 8;

/// The `button` slot holds the frame type.
fn type_label(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x50 => "Type 0x50",
        0x70 => "Type 0x70",
        _ => "Type ?",
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    PreambleShort,
    PreambleVeryLong,
    PreambleLong,
    Data,
}

/// Hörmann BiSecur decoder (`SubGhzProtocolDecoderHormannBiSecur`).
pub struct HormannBiSecurDecoder {
    step: Step,
    sync_cnt: u8,
    state: ManchesterState,
    data: [u8; LEN],
    count: usize,
}

/// `manchester_advance_alt` (hormann_bisecur.c): `(next state, bit)`.
fn advance_alt(state: ManchesterState, event: ManchesterEvent) -> (ManchesterState, Option<bool>) {
    use ManchesterEvent::*;
    use ManchesterState::*;
    match (state, event) {
        (_, Reset) => (Mid1, None),
        (Start1, ShortLow) => (Mid1, Some(false)),
        (Start1, LongLow) => (Start0, Some(false)),
        (Start1, _) => (Mid1, None),
        (Mid1, ShortHigh) => (Start1, None),
        (Mid1, ShortLow) => (Start0, None),
        (Mid1, _) => (Mid1, None),
        (Start0, ShortHigh) => (Mid0, Some(true)),
        (Start0, LongHigh) => (Start1, Some(true)),
        (Start0, _) => (Mid1, None),
        (Mid0, ShortLow) => (Start0, None),
        (Mid0, ShortHigh) => (Start1, None),
        (Mid0, _) => (Mid1, None),
    }
}

/// `subghz_protocol_blocks_crc8`.
fn crc8(bytes: &[u8], poly: u8, init: u8) -> u8 {
    bytes.iter().fold(init, |mut r, &b| {
        r ^= b;
        for _ in 0..8 {
            r = if r & 0x80 != 0 {
                (r << 1) ^ poly
            } else {
                r << 1
            };
        }
        r
    })
}

/// `subghz_protocol_blocks_crc8le`: reflected CRC-8 (init and polynomial bit-reversed).
fn crc8le(bytes: &[u8], poly: u8, init: u8) -> u8 {
    let poly = poly.reverse_bits();
    bytes.iter().fold(init.reverse_bits(), |mut r, &b| {
        r ^= b;
        for _ in 0..8 {
            r = if r & 1 != 0 { (r >> 1) ^ poly } else { r >> 1 };
        }
        r
    })
}

/// `subghz_protocol_decoder_hormann_bisecur_crc`: the CRC byte of a frame of a known type.
fn crc(frame: &[u8; LEN]) -> Option<u8> {
    match frame[0] {
        0x50 => Some(!(crc8(&frame[1..21], 0x07, 0x00) ^ 0x55)),
        0x70 => Some(crc8le(&frame[..21], 0x07, 0xFF)),
        _ => None,
    }
}

/// `subghz_protocol_decoder_hormann_bisecur_check_crc`.
fn crc_ok(frame: &[u8; LEN]) -> bool {
    crc(frame) == Some(frame[21])
}

fn be_u64(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0u64, |acc, &b| (acc << 8) | u64::from(b))
}

/// The decode of a frame (see the module doc for the packing).
fn signal(frame: &[u8; LEN]) -> DecodedSignal {
    DecodedSignal {
        serial: Some(be_u64(&frame[1..5]) as u32),
        button: Some(frame[0]),
        counter: None,
        crc_valid: crc_ok(frame),
        data: be_u64(&frame[5..13]),
        data_count_bit: BIT_COUNT,
        encoder_capable: true,
        extra: Some(be_u64(&frame[13..21])),
        protocol_display_name: None,
    }
}

/// The 22 bytes a decode stands for (the CRC recomputed from its type), when it is a CRC-valid
/// 176-bit frame.
fn frame_of(d: &DecodedSignal) -> Option<[u8; LEN]> {
    if d.data_count_bit != BIT_COUNT {
        return None;
    }
    let mut f = [0u8; LEN];
    f[0] = d.button?;
    f[1..5].copy_from_slice(&d.serial?.to_be_bytes());
    f[5..13].copy_from_slice(&d.data.to_be_bytes());
    f[13..21].copy_from_slice(&d.extra?.to_be_bytes());
    f[21] = crc(&f)?;
    Some(f)
}

/// `subghz_protocol_encoder_hormann_bisecur_get_upload` (one copy; adjacent levels merged).
fn upload(frame: &[u8; LEN]) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(2 * 21 + 4 + 2 * BIT_COUNT + 1);
    for i in 0..21 {
        push(
            &mut out,
            false,
            if i == 0 {
                TE_SHORT + HALF_SHORT
            } else {
                TE_SHORT
            },
        );
        push(&mut out, true, if i == 20 { TE_LONG * 4 } else { TE_SHORT });
    }
    for _ in 0..2 {
        push(&mut out, false, TE_LONG);
        push(&mut out, true, TE_LONG);
    }
    let mut enc = ManchesterEncoder::new();
    for i in (1..=BIT_COUNT).rev() {
        let bit_index = i - 1;
        let byte = frame[LEN - 1 - bit_index / 8];
        let bit = (byte >> (bit_index & 7)) & 1 == 0; // !bit_read
        let (advanced, r) = enc.advance(bit);
        let p = r.pulse(TE_SHORT, TE_LONG);
        push(&mut out, p.level, p.duration_us);
        if !advanced {
            let (_, r) = enc.advance(bit);
            let p = r.pulse(TE_SHORT, TE_LONG);
            push(&mut out, p.level, p.duration_us);
        }
    }
    let last = enc.finish().pulse(TE_SHORT, TE_LONG);
    push(
        &mut out,
        last.level,
        last.duration_us + TE_SHORT + HALF_SHORT,
    );
    out
}

impl HormannBiSecurDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            sync_cnt: 0,
            state: ManchesterState::Start1,
            data: [0; LEN],
            count: 0,
        }
    }

    /// `subghz_protocol_decoder_hormann_bisecur_add_bit`: true when the 176th bit is in.
    fn add_bit(&mut self, bit: bool) -> bool {
        if self.count >= BIT_COUNT {
            return false;
        }
        if bit {
            self.data[self.count / 8] |= 1 << (7 - self.count % 8);
        }
        self.count += 1;
        self.count >= BIT_COUNT
    }
}

impl Default for HormannBiSecurDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for HormannBiSecurDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: BIT_COUNT,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    /// `subghz_protocol_decoder_hormann_bisecur_reset` (`sync_cnt` is left as it is, as in the C).
    fn reset(&mut self) {
        self.step = Step::Reset;
        self.data = [0; LEN];
        self.count = 0;
        self.state = ManchesterState::Start1;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let near = |d: u32, t: u32| d.abs_diff(t) < TE_DELTA;
        match self.step {
            Step::Reset => {
                if !level && near(duration, TE_SHORT + HALF_SHORT) {
                    self.step = Step::PreambleShort;
                }
            }
            Step::PreambleShort => {
                if near(duration, TE_SHORT) {
                    // the pattern repeats around 21 times
                } else if level && near(duration, TE_LONG * 4) {
                    self.step = Step::PreambleVeryLong;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::PreambleVeryLong => {
                if !level && near(duration, TE_LONG) {
                    self.sync_cnt = 3;
                    self.step = Step::PreambleLong;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::PreambleLong => {
                // `level == (sync_cnt-- & 1)`: the countdown happens whatever the outcome.
                let want = self.sync_cnt & 1 == 1;
                self.sync_cnt = self.sync_cnt.wrapping_sub(1);
                if level == want && near(duration, TE_LONG) {
                    if self.sync_cnt == 0 {
                        self.state = advance_alt(self.state, ManchesterEvent::Reset).0;
                        self.step = Step::Data;
                    }
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::Data => {
                let mut event = ManchesterEvent::Reset;
                if near(duration, TE_SHORT) || (self.count == BIT_COUNT - 1 && duration > TE_SHORT)
                {
                    event = if level {
                        ManchesterEvent::ShortLow
                    } else {
                        ManchesterEvent::ShortHigh
                    };
                }
                if near(duration, TE_LONG) {
                    event = if level {
                        ManchesterEvent::LongLow
                    } else {
                        ManchesterEvent::LongHigh
                    };
                }
                if event == ManchesterEvent::Reset {
                    self.reset();
                    return None;
                }
                let (next, bit) = advance_alt(self.state, event);
                self.state = next;
                if let Some(bit) = bit {
                    if self.add_bit(bit) {
                        let frame = self.data;
                        // The receiver resets the decoder after the C's callback.
                        self.reset();
                        // KAT: only frames whose CRC holds (see the module doc).
                        return crc_ok(&frame).then(|| signal(&frame));
                    }
                }
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        frame_of(decoded).map(|f| upload(&f))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != BIT_COUNT || key.key_bytes.len() < LEN {
            return None;
        }
        let frame: [u8; LEN] = key.key_bytes[..LEN].try_into().ok()?;
        crc_ok(&frame).then(|| signal(&frame))
    }

    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        let frame = frame_of(decoded)?;
        let mut key = FlipperKey::for_protocol(NAME, BIT_COUNT, be_u64(&frame[..8]));
        key.key_bytes = frame.to_vec();
        Some(key)
    }
}

/// Frame builders for the fuzz tool.
#[cfg(test)]
pub(super) mod testing {
    use super::*;

    /// The decode of `bytes` with its CRC byte made valid (type 0x50 or 0x70 expected).
    pub fn with_valid_crc(mut bytes: [u8; LEN]) -> DecodedSignal {
        bytes[21] = crc(&bytes).expect("type 0x50 or 0x70");
        signal(&bytes)
    }

    /// The C's upload of any 22 bytes (CRC not rebuilt).
    pub fn raw_upload(bytes: &[u8; LEN]) -> Vec<LevelDuration> {
        upload(bytes)
    }
}

#[cfg(test)]
mod tests {
    use super::super::f_common::test_support::*;
    use super::*;

    const PRESET: &str = "Preset: FuriHalSubGhzPresetCustom\nCustom_preset_module: CC1101\n\
        Custom_preset_data: 02 0D 0B 06 08 32 07 04 14 00 13 02 12 04 11 83 10 67 15 40 18 18 19 16 1D 91 1C 00 \
        1B 07 20 FB 22 10 21 56 00 00 C0 00 00 00 00 00 00 00\n";

    /// The two key files of the Momentum / RogueMaster unit-test resources.
    fn unit_test_key(name: &str) -> String {
        let key = match name {
            "080ED969" => "70 08 0E D9 69 CA EE 02 6C F1 0E CB 11 5E F5 F8 17 02 6D EC 16 4B",
            _ => "50 1C 84 A5 5B 24 54 0D E5 44 C0 70 69 73 7F 2C 7E B3 43 18 BB 1C",
        };
        format!(
            "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 868276000\n{PRESET}\
             Protocol: Hormann BiSecur\nBit: 176\nKey: {key}\n"
        )
    }

    fn frame(hex: &str) -> [u8; LEN] {
        super::super::flipper_key::parse_hex_vec(hex)
            .unwrap()
            .try_into()
            .unwrap()
    }

    #[test]
    fn crc_of_both_frame_types() {
        let a = frame("70 08 0E D9 69 CA EE 02 6C F1 0E CB 11 5E F5 F8 17 02 6D EC 16 4B");
        let b = frame("50 1C 84 A5 5B 24 54 0D E5 44 C0 70 69 73 7F 2C 7E B3 43 18 BB 1C");
        assert_eq!((crc(&a), crc(&b)), (Some(0x4B), Some(0x1C)));
        let mut c = b;
        c[0] = 0x60;
        assert_eq!(crc(&c), None, "unknown type: the C shows WRONG");
        // crc8le = the bit-reflected CRC-8 (check against a bitwise definition).
        let reflected = |bytes: &[u8]| {
            let mut r: u8 = 0xFF;
            for &byte in bytes {
                for i in 0..8 {
                    let bit = (byte >> i) & 1 ^ (r & 1);
                    r >>= 1;
                    if bit == 1 {
                        r ^= 0xE0;
                    }
                }
            }
            r
        };
        assert_eq!(crc8le(&a[..21], 0x07, 0xFF), reflected(&a[..21]));
    }

    #[test]
    fn unit_test_key_files_load_with_the_c_fields() {
        let dec = HormannBiSecurDecoder::new();
        let a = dec
            .from_flipper_key(&FlipperKey::parse(&unit_test_key("080ED969")).unwrap())
            .unwrap();
        // The C's get_string: "Type:0x70 Sn:0x080ED969 / Key:CAEE026CF10ECB11 / Key:5EF5F81702 6DEC16".
        assert_eq!((a.button, a.serial), (Some(0x70), Some(0x080E_D969)));
        assert_eq!(
            (a.data, a.extra),
            (0xCAEE_026C_F10E_CB11, Some(0x5EF5_F817_026D_EC16))
        );
        assert!(a.crc_valid && a.encoder_capable && a.counter.is_none());
        assert_eq!(type_label(NAME, 0x70), Some("Type 0x70"));
        let b = dec
            .from_flipper_key(&FlipperKey::parse(&unit_test_key("1C84A55B")).unwrap())
            .unwrap();
        assert_eq!((b.button, b.serial), (Some(0x50), Some(0x1C84_A55B)));
        assert_eq!(
            (b.data, b.extra),
            (0x2454_0DE5_44C0_7069, Some(0x737F_2C7E_B343_18BB))
        );
        // A key whose CRC fails is not loaded (KAT keeps no CRC byte), nor a wrong bit count.
        let bad = unit_test_key("1C84A55B").replace("BB 1C", "BB 1D");
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&bad).unwrap())
            .is_none());
        let short = unit_test_key("1C84A55B").replace("Bit: 176", "Bit: 168");
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&short).unwrap())
            .is_none());
    }

    #[test]
    fn key_file_round_trip_writes_all_22_bytes() {
        let dec = HormannBiSecurDecoder::new();
        for name in ["080ED969", "1C84A55B"] {
            let text = unit_test_key(name);
            let d = dec
                .from_flipper_key(&FlipperKey::parse(&text).unwrap())
                .unwrap();
            let written = super::super::flipper_key::testing::assert_round_trip(&dec, &d);
            let key_line = text.lines().find(|l| l.starts_with("Key:")).unwrap();
            assert!(
                written.contains(&format!(
                    "Protocol: Hormann BiSecur\nBit: 176\n{key_line}\n"
                )),
                "{written}"
            );
        }
    }

    /// Encoder → decoder, for both types and random payloads; the upload's shape is the C's.
    #[test]
    fn encode_decode_round_trip() {
        let mut rng = Rng(0xB15E_C0DE);
        for i in 0..64 {
            let mut bytes = [0u8; LEN];
            for b in &mut bytes {
                *b = rng.next() as u8;
            }
            bytes[0] = if i % 2 == 0 { 0x50 } else { 0x70 };
            let d = testing::with_valid_crc(bytes);
            let tx = HormannBiSecurDecoder::new().encode(&d, 0).unwrap();
            // Preamble: 312 LOW first, the 1664 HIGH, then the long sync pairs.
            assert_eq!((tx[0].level, tx[0].duration_us), (false, 312));
            assert_eq!((tx[41].level, tx[41].duration_us), (true, 1664));
            assert!(tx[42..45].iter().all(|p| p.duration_us == TE_LONG));
            let got = feed_all(&mut HormannBiSecurDecoder::new(), &tx);
            assert_eq!(got, std::slice::from_ref(&d), "frame {i}");
        }
    }

    #[test]
    fn rejects_bad_crc_broken_preamble_and_glitches() {
        let d = HormannBiSecurDecoder::new()
            .from_flipper_key(&FlipperKey::parse(&unit_test_key("1C84A55B")).unwrap())
            .unwrap();
        let tx = HormannBiSecurDecoder::new().encode(&d, 0).unwrap();
        assert_eq!(feed_all(&mut HormannBiSecurDecoder::new(), &tx).len(), 1);
        // A frame whose CRC fails (the C would report it, "CRC WRONG").
        let mut f = frame_of(&d).unwrap();
        f[7] ^= 0x10;
        assert!(feed_all(&mut HormannBiSecurDecoder::new(), &upload(&f)).is_empty());
        // No 1664 µs HIGH: never synchronises.
        let mut no_sync = tx.clone();
        no_sync[41].duration_us = 1400;
        assert!(feed_all(&mut HormannBiSecurDecoder::new(), &no_sync).is_empty());
        // A 312 µs data pulse is neither short (< 312) nor long (> 312): reset.
        let mut glitch = tx.clone();
        glitch[100].duration_us = 312;
        assert!(feed_all(&mut HormannBiSecurDecoder::new(), &glitch).is_empty());
        // The last half-bit may be arbitrarily long.
        let mut long_tail = tx.clone();
        long_tail.last_mut().unwrap().duration_us = 50_000;
        assert_eq!(
            feed_all(&mut HormannBiSecurDecoder::new(), &long_tail).len(),
            1
        );
    }

    /// The Momentum / RogueMaster RAW capture through the whole registry (FSK preset → FM gate):
    /// the C reports 3 frames, all the key file `hormann_bisecur_1C84A55B.sub`.
    #[test]
    fn unit_test_capture_decodes() {
        let Some(path) = reference_file(MOMENTUM_UNIT_TESTS, "hormann_bisecur_1C84A55B_raw.sub")
        else {
            return;
        };
        let decodes = registry_decodes(&path);
        let key = HormannBiSecurDecoder::new()
            .from_flipper_key(&FlipperKey::parse(&unit_test_key("1C84A55B")).unwrap())
            .unwrap();
        assert_eq!(decodes.len(), 3, "{decodes:?}");
        assert!(
            decodes.iter().all(|(n, d)| n == NAME && *d == key),
            "{decodes:?}"
        );
    }
}
