//! KingGates Stylo4k: KeeLoq (simple learning) gate/garage remotes.
//!
//! References: Flipper-ARF's garage app `applications/system/subghz_garage/protocols/
//! kinggates_stylo_4k.c` (primary) and doctor-firmware `lib/subghz/protocols/kinggates_stylo_4k.c`.
//! Same air format, frame, check and upload. The garage copy (like ARF's lib copy) keeps walking
//! the type-10 keystore entries until one passes the check, doctor-firmware stops at the first one
//! (the same with KAT's single KingGates key); doctor caches the analysis and formats the display
//! differently.
//!
//! **Air format** (`subghz_protocol_decoder_kinggates_stylo_4k_feed`): te_short 400 µs, te_long
//! 1100 µs, te_delta 140 µs. Preamble HIGH 400 ± 140 / LOW 400 ± 140 pulses; after more than 2 of
//! them a 2200 ± 280 µs pulse is the header, then the decoder waits (ignoring anything else) for
//! the 800 ± 280 µs HIGH start bit. Bits are LOW-first PWM: 1 = LOW 400 ± 140 + HIGH 1100 ± 280,
//! 0 = LOW 1100 ± 280 + HIGH 400 ± 140. The first 53 bits are stored when the 53rd arrives; a LOW
//! of 3300 µs or more ends the frame, and exactly 89 bits are a frame.
//!
//! **Frame** (89 bits): a 53-bit fix sent LSB first — `reverse_key(fix, 53)` = 16 zero bits |
//! serial[31:16] | button (4) | 1 | serial[15:0] — then 36 bits: the 32-bit KeeLoq hop
//! bit-reversed, followed by 4 zero bits. Hop plaintext (simple learning): button << 28 | 0xC << 24
//! | serial[7:0] << 16 | counter (16 bits).
//!
//! **Key**: the C takes the keystore entries of learning type 10
//! (KEELOQ_LEARNING_SIMPLE_KINGGATES). KAT's embedded keystore holds that key as
//! "Kingates_Stylo4k" typed 1 (its type-10 slot is ProtoPirate's KIA key, which the KeeLoq view
//! drops), so it is found by type 10 or by that name. Evidence: Flipper's unit-test capture
//! `kinggates_stylo4k_raw.sub` decrypts with it and passes the C's check (button 0xE, nibble 0xC,
//! serial byte 0x27); the C oracle given the key typed 10 decodes the same fields.
//!
//! **Gating**: the C calls back for every 89-bit frame and, when the check fails, only displays
//! zeroed fields; per KAT policy a frame is reported only when the hop decrypts to the frame's
//! button, the 0xC nibble and the serial's low byte (16 bits, `crc_valid`).
//!
//! **KAT fields**: `data` = the first 64 wire bits (fix53 << 11 | hop36 >> 25), `extra` = the last
//! 25 wire bits, `data_count_bit` = 89. The key file's `Key` is the C `generic.data` (fix53) and
//! `Data` the C `generic.data_2` (hop36). serial = 32 bits, button = 4 bits, counter = 16 bits.
//!
//! **Encoder** (`subghz_protocol_encoder_kinggates_stylo_4k_deserialize` + `gen_data` +
//! `get_upload`): the frame is analysed again, counter + 1 (0 past 0xFFFF), button = the requested
//! native code (4 bits; the C's custom buttons cycle 0xE/0xD/0xB/0x7, any other code keeps the
//! captured button), fix rebuilt, hop re-encrypted with the first KingGates key. Upload: LOW 9500,
//! 12 × (HIGH 400, LOW 400) with the last LOW 2200, HIGH 800, 53 + 36 bits; `encoder.repeat` = 3.
//! (Flipper-ARF's garage TX plugin is built without `SUBGHZ_GARAGE_HAS_CUSTOM_BTN` and
//! `SUBGHZ_GARAGE_HAS_LIB_EXTENSIONS`; their fallbacks make it send button 0 and take the "OFEX"
//! counter path (0xFFFE). KAT follows the source's intended path — doctor-firmware's behaviour.)

use super::common::DecodedSignal;
use super::flipper_key::FlipperKey;
use super::g5_common::{hex8_field, key8, next_counter, push};
use super::keeloq_common::{
    keeloq_decrypt, keeloq_encrypt, keeloq_keystore, learning_types as lt, reverse_key,
};
use super::{ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "KingGates Stylo4k",
    decoder: "KingGates Stylo4k",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "KeeLoq",
    make: "KingGates",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "KingGates Stylo4k";
const TE_SHORT: u32 = 400;
const TE_LONG: u32 = 1100;
const TE_DELTA: u32 = 140;
const MIN_COUNT_BIT: usize = 89;
const FIX_BITS: usize = 53;
const HOP_BITS: usize = 36;
/// Wire bits that do not fit `data` (89 - 64), right-aligned in `extra`.
const EXTRA_BITS: usize = MIN_COUNT_BIT - 64;
const FIX_MASK: u64 = (1 << FIX_BITS) - 1;
const HOP_MASK: u64 = (1 << HOP_BITS) - 1;
/// KeeLoq keystore name of the KingGates key in KAT's embedded keystore.
const KEY_NAME: &str = "Kingates_Stylo4k";
/// C `encoder.repeat`.
const TX_REPEAT: usize = 3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    CheckPreambula,
    CheckStartBit,
    SaveDuration,
    CheckDuration,
}

/// The KingGates keys in keystore order (the C's type-10 walk; see the module doc).
fn kinggates_keys() -> impl Iterator<Item = u64> {
    keeloq_keystore()
        .iter()
        .filter(|e| e.learning == lt::SIMPLE_KINGGATES || e.name == KEY_NAME)
        .map(|e| e.key)
}

/// `(fix53, hop36)` → KAT's `(data, extra)`.
fn pack(fix53: u64, hop36: u64) -> (u64, u64) {
    let wire = (u128::from(fix53 & FIX_MASK) << HOP_BITS) | u128::from(hop36 & HOP_MASK);
    (
        (wire >> EXTRA_BITS) as u64,
        (wire as u64) & ((1 << EXTRA_BITS) - 1),
    )
}

/// KAT's `(data, extra)` → `(fix53, hop36)` (the C's `generic.data`, `generic.data_2`).
fn unpack(data: u64, extra: u64) -> (u64, u64) {
    let wire = (u128::from(data) << EXTRA_BITS) | u128::from(extra & ((1 << EXTRA_BITS) - 1));
    ((wire >> HOP_BITS) as u64 & FIX_MASK, wire as u64 & HOP_MASK)
}

/// A frame that passed `subghz_protocol_kinggates_stylo_4k_remote_controller`'s check.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Fields {
    serial: u32,
    btn: u8,
    cnt: u16,
}

/// `subghz_protocol_kinggates_stylo_4k_remote_controller`: None when no key passes the check.
fn analyze(fix53: u64, hop36: u64) -> Option<Fields> {
    let hop = reverse_key(hop36 >> 4, 32) as u32;
    let fix = reverse_key(fix53, FIX_BITS);
    let btn = ((fix >> 17) & 0x0F) as u32;
    let serial = (((fix >> 5) & 0xFFFF_0000) | (fix & 0xFFFF)) as u32;
    kinggates_keys().find_map(|key| {
        let decrypt = keeloq_decrypt(hop, key);
        (decrypt >> 28 == btn
            && (decrypt >> 24) & 0x0F == 0x0C
            && (decrypt >> 16) & 0xFF == serial & 0xFF)
            .then_some(Fields {
                serial,
                btn: btn as u8,
                cnt: decrypt as u16,
            })
    })
}

fn signal(fix53: u64, hop36: u64) -> Option<DecodedSignal> {
    let f = analyze(fix53, hop36)?;
    let (data, extra) = pack(fix53, hop36);
    Some(DecodedSignal {
        serial: Some(f.serial),
        button: Some(f.btn),
        counter: Some(f.cnt),
        crc_valid: true,
        data,
        data_count_bit: MIN_COUNT_BIT,
        encoder_capable: true,
        extra: Some(extra),
        protocol_display_name: None,
    })
}

/// `subghz_protocol_kinggates_stylo_4k_gen_data`: the (fix53, hop36) for a press.
fn gen_data(serial: u32, btn: u8, cnt: u16) -> Option<(u64, u64)> {
    let btn = u32::from(btn & 0x0F);
    let hop = (btn << 28) | (0x0C << 24) | ((serial & 0xFF) << 16) | u32::from(cnt);
    let fix = (u64::from((serial >> 16) & 0xFFFF) << 21)
        | (u64::from(btn) << 17)
        | (1 << 16)
        | u64::from(serial & 0xFFFF);
    // The C encrypts with the first type-10 entry.
    let key = kinggates_keys().next()?;
    let encrypted = reverse_key(u64::from(keeloq_encrypt(hop, key)), 32);
    Some((reverse_key(fix, FIX_BITS), encrypted << 4))
}

/// `subghz_protocol_kinggates_stylo_4k_create_data` (the SubGHz app's "Add Manually"): the counter
/// as given (not advanced), the button unmasked as the C shifts it, the hop encrypted with the
/// first KingGates key; `Bit: 89`, `Key` (fix53), `Data` (hop36, 8 hex bytes). None without the key.
pub(super) fn create_data(serial: u32, btn: u8, cnt: u16) -> Option<FlipperKey> {
    let decrypt = (u32::from(btn) << 28) | (0x0C << 24) | ((serial & 0xFF) << 16) | u32::from(cnt);
    let hop36 = reverse_key(u64::from(keeloq_encrypt(decrypt, kinggates_keys().next()?)), 32) << 4;
    let fix = (u64::from((serial >> 16) & 0xFFFF) << 21)
        | (u64::from(btn) << 17)
        | (1 << 16)
        | u64::from(serial & 0xFFFF);
    let mut k = FlipperKey::for_protocol(NAME, MIN_COUNT_BIT, reverse_key(fix, FIX_BITS));
    k.push_hex("Data", hop36, 8);
    Some(k)
}

/// One copy of `subghz_protocol_encoder_kinggates_stylo_4k_get_upload`.
fn upload_once(fix53: u64, hop36: u64, out: &mut Vec<LevelDuration>) {
    push(out, false, 9500);
    for i in 0..12 {
        push(out, true, TE_SHORT);
        push(out, false, if i == 11 { TE_LONG * 2 } else { TE_SHORT });
    }
    push(out, true, TE_SHORT * 2);
    for (word, bits) in [(fix53, FIX_BITS), (hop36, HOP_BITS)] {
        for i in (0..bits).rev() {
            if (word >> i) & 1 == 1 {
                push(out, false, TE_SHORT);
                push(out, true, TE_LONG);
            } else {
                push(out, false, TE_LONG);
                push(out, true, TE_SHORT);
            }
        }
    }
}

pub struct KingGatesStylo4kDecoder {
    step: Step,
    header_count: u16,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: usize,
    /// C `generic.data_2` while receiving: the 53-bit fix, stored at the 53rd bit.
    fix53: u64,
}

impl KingGatesStylo4kDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            header_count: 0,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
            fix53: 0,
        }
    }

    fn add_bit(&mut self, bit: u64) {
        self.decode_data = (self.decode_data << 1) | bit;
        self.decode_count_bit += 1;
    }
}

impl Default for KingGatesStylo4kDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for KingGatesStylo4kDecoder {
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
        *self = Self::new();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                if level && duration_diff!(duration, TE_SHORT) < TE_DELTA {
                    self.step = Step::CheckPreambula;
                    self.header_count = self.header_count.wrapping_add(1);
                }
            }
            Step::CheckPreambula => {
                if !level && duration_diff!(duration, TE_SHORT) < TE_DELTA {
                    self.step = Step::Reset;
                    return None;
                }
                // (No level test here in the C.)
                if self.header_count > 2 && duration_diff!(duration, TE_LONG * 2) < TE_DELTA * 2 {
                    self.step = Step::CheckStartBit;
                } else {
                    self.step = Step::Reset;
                    self.header_count = 0;
                }
            }
            Step::CheckStartBit => {
                if level && duration_diff!(duration, TE_SHORT * 2) < TE_DELTA * 2 {
                    self.step = Step::SaveDuration;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.header_count = 0;
                }
            }
            Step::SaveDuration => {
                if !level {
                    if duration >= TE_LONG * 3 {
                        let result = if self.decode_count_bit == MIN_COUNT_BIT {
                            // C: data = data_2 (the fix), data_2 = decode_data (the hop part).
                            signal(self.fix53, self.decode_data)
                        } else {
                            None
                        };
                        self.step = Step::Reset;
                        self.decode_data = 0;
                        self.decode_count_bit = 0;
                        self.header_count = 0;
                        return result;
                    }
                    self.te_last = duration;
                    self.step = Step::CheckDuration;
                } else {
                    self.step = Step::Reset;
                    self.header_count = 0;
                }
            }
            Step::CheckDuration => {
                if level {
                    if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA
                        && duration_diff!(duration, TE_LONG) < TE_DELTA * 2
                    {
                        self.add_bit(1);
                        self.step = Step::SaveDuration;
                    } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA * 2
                        && duration_diff!(duration, TE_SHORT) < TE_DELTA
                    {
                        self.add_bit(0);
                        self.step = Step::SaveDuration;
                    } else {
                        self.step = Step::Reset;
                        self.header_count = 0;
                    }
                    if self.decode_count_bit == FIX_BITS {
                        self.fix53 = self.decode_data;
                        self.decode_data = 0;
                    }
                } else {
                    self.step = Step::Reset;
                    self.header_count = 0;
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
        let (fix53, hop36) = unpack(decoded.data, decoded.extra?);
        let f = analyze(fix53, hop36)?;
        let btn = if button <= 0x0F { button } else { f.btn };
        let cnt = next_counter(u32::from(f.cnt), 0xFFFF) as u16;
        let (fix53, hop36) = gen_data(f.serial, btn, cnt)?;
        let mut out = Vec::with_capacity(TX_REPEAT * 204);
        for _ in 0..TX_REPEAT {
            upload_once(fix53, hop36, &mut out);
        }
        Some(out)
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        // `subghz_protocol_decoder_kinggates_stylo_4k_deserialize`: Bit must be 89, Key + Data.
        if key.bits != MIN_COUNT_BIT {
            return None;
        }
        signal(key8(key)? & FIX_MASK, hex8_field(key, "Data")? & HOP_MASK)
    }

    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        // `..._serialize`: generic block (Key = the fix) + "Data" = the hop part.
        let (fix53, hop36) = unpack(decoded.data, decoded.extra?);
        let mut k = FlipperKey::new(NAME, MIN_COUNT_BIT, fix53, 0, "");
        k.push_hex("Data", hop36, 8);
        Some(k)
    }
}

/// A key file for a random valid frame (oracle driver fixtures).
#[cfg(test)]
pub(super) fn fixture_key(r: u64) -> FlipperKey {
    let cnt = if r & 0x30 == 0 {
        0xFFFF
    } else {
        (r >> 16) as u16
    };
    let (fix53, hop36) = gen_data((r >> 32) as u32, (r & 0x0F) as u8, cnt).unwrap();
    KingGatesStylo4kDecoder::new()
        .to_flipper_key(&signal(fix53, hop36).unwrap())
        .unwrap()
}

/// The KingGates key entry (tests).
#[cfg(test)]
fn kinggates_entry() -> &'static super::keeloq_common::MfKey {
    super::g5_common::keystore_entry(|e| e.name == KEY_NAME).expect("Kingates_Stylo4k key")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::export::flipper::import_sub_raw;
    use crate::protocols::ProtocolRegistry;
    use std::path::Path;

    fn decode_all(signal: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = KingGatesStylo4kDecoder::new();
        let mut out = Vec::new();
        for ld in signal {
            if let Some(d) = dec.feed(ld.level, ld.duration_us) {
                out.push(d);
                dec.reset();
            }
        }
        out
    }

    /// One remote transmission (an upload copy) followed by a gap.
    fn capture(serial: u32, btn: u8, cnt: u16) -> Vec<LevelDuration> {
        let (fix53, hop36) = gen_data(serial, btn, cnt).unwrap();
        let mut s = Vec::new();
        upload_once(fix53, hop36, &mut s);
        s.push(LevelDuration::new(false, 9500));
        s
    }

    /// The frame of Flipper's unit-test capture (`kinggates_stylo4k_raw.sub`).
    const UT_FIX: u64 = 0x001C_9037_F0C8_0000;
    const UT_HOP: u64 = 0x3_6F25_0780;

    #[test]
    fn decodes_the_unit_test_frame_layout() {
        let d = signal(UT_FIX, UT_HOP).expect("passes the C check");
        assert_eq!(
            (d.serial, d.button, d.counter),
            (Some(0x130F_8127), Some(0xE), Some(0x0B9D))
        );
        assert_eq!(unpack(d.data, d.extra.unwrap()), (UT_FIX, UT_HOP));
        assert_eq!(d.data_count_bit, 89);
        // gen_data reproduces the captured frame for the captured fields.
        assert_eq!(gen_data(0x130F_8127, 0xE, 0x0B9D), Some((UT_FIX, UT_HOP)));
        assert_eq!(kinggates_entry().key, 0x414C_4558_3139_3830);
    }

    #[test]
    fn decodes_synthetic_frames() {
        for (serial, btn, cnt) in [
            (0x130F_8127u32, 0xEu8, 0x0B9Cu16),
            (0x0001_2345, 0x7, 0),
            (0xFFFF_FFFF, 0xB, 0xFFFF),
        ] {
            let d = decode_all(&capture(serial, btn, cnt));
            assert_eq!(d.len(), 1);
            assert_eq!(
                (d[0].serial, d[0].button, d[0].counter),
                (Some(serial), Some(btn), Some(cnt))
            );
            assert!(d[0].crc_valid && d[0].encoder_capable);
        }
    }

    #[test]
    fn rejects_bad_frames() {
        // A flipped hop bit fails the check (the C would report it with zeroed fields).
        let (fix53, hop36) = gen_data(0x0001_2345, 0xD, 0x10).unwrap();
        let mut s = Vec::new();
        upload_once(fix53, hop36 ^ 1 << 20, &mut s);
        s.push(LevelDuration::new(false, 9500));
        assert!(decode_all(&s).is_empty());
        // A different button in the fix fails it too.
        let mut s = Vec::new();
        upload_once(
            reverse_key(reverse_key(fix53, 53) ^ 0x3 << 17, 53),
            hop36,
            &mut s,
        );
        s.push(LevelDuration::new(false, 9500));
        assert!(decode_all(&s).is_empty());
        // 88 bits (one bit dropped) is not a frame.
        let mut s = capture(0x0001_2345, 0xD, 0x10);
        s.drain(30..32);
        assert!(decode_all(&s).is_empty());
        // Header gap 2600 µs (outside 2200 ± 280).
        let mut s = capture(0x0001_2345, 0xD, 0x10);
        s[24].duration_us = 2600;
        assert!(decode_all(&s).is_empty());
    }

    #[test]
    fn encode_round_trip_every_button() {
        let dec = KingGatesStylo4kDecoder::new();
        let captured = decode_all(&capture(0x130F_8127, 0xE, 0x0B9C))
            .pop()
            .unwrap();
        for button in [0xEu8, 0xD, 0xB, 0x7, 0x3] {
            let mut wire = dec.encode(&captured, button).unwrap();
            assert_eq!(wire.len(), 3 * 204);
            wire.push(LevelDuration::new(false, 9500));
            let got = decode_all(&wire);
            assert_eq!(got.len(), 3, "button {button:#x}");
            for d in &got {
                assert_eq!(
                    (d.serial, d.button, d.counter),
                    (Some(0x130F_8127), Some(button), Some(0x0B9D))
                );
            }
        }
        // Out-of-range code keeps the captured button; counter wraps past 0xFFFF.
        let top = decode_all(&capture(0x0055_AA00, 0x7, 0xFFFF))
            .pop()
            .unwrap();
        let next = decode_all(&dec.encode(&top, 0x42).unwrap()).pop().unwrap();
        assert_eq!((next.button, next.counter), (Some(0x7), Some(0)));
    }

    const UT_KEY_FILE: &str =
        "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
        Preset: FuriHalSubGhzPresetOok650Async\nProtocol: KingGates Stylo4k\nBit: 89\n\
        Key: 00 1C 90 37 F0 C8 00 00\nData: 00 00 00 03 6F 25 07 80\n";

    #[test]
    fn key_file_round_trip() {
        let dec = KingGatesStylo4kDecoder::new();
        let key = FlipperKey::parse(UT_KEY_FILE).unwrap();
        let d = dec.from_flipper_key(&key).unwrap();
        assert_eq!(
            (d.serial, d.button, d.counter),
            (Some(0x130F_8127), Some(0xE), Some(0x0B9D))
        );
        let back = dec.to_flipper_key(&d).unwrap();
        assert!(back.to_text().contains(
            "Protocol: KingGates Stylo4k\nBit: 89\nKey: 00 1C 90 37 F0 C8 00 00\n\
             Data: 00 00 00 03 6F 25 07 80\n"
        ));
        let again = dec
            .from_flipper_key(&FlipperKey::parse(&back.to_text()).unwrap())
            .unwrap();
        assert_eq!(format!("{again:?}"), format!("{d:?}"));
        // Failing check, wrong bit count, missing Data.
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&UT_KEY_FILE.replace("07 80", "07 90")).unwrap())
            .is_none());
        assert!(dec
            .from_flipper_key(
                &FlipperKey::parse(&UT_KEY_FILE.replace("Bit: 89", "Bit: 88")).unwrap()
            )
            .is_none());
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&UT_KEY_FILE.replace("Data:", "Dat:")).unwrap())
            .is_none());
        let reg = ProtocolRegistry::new();
        assert_eq!(
            reg.decode_key(&key).map(|(n, _)| n),
            Some("KingGates Stylo4k".to_string())
        );
    }

    #[test]
    fn decodes_flipper_unit_test_capture() {
        let path = Path::new(
            "REFERENCES/Flipper-ARF/applications/debug/unit_tests/resources/unit_tests/subghz/kinggates_stylo4k_raw.sub",
        );
        if !path.exists() {
            eprintln!("Skip: {path:?} not found");
            return;
        }
        let (freq, raw) = import_sub_raw(path).unwrap();
        let pairs: Vec<LevelDuration> = raw
            .iter()
            .map(|p| LevelDuration::new(p.level, p.duration_us))
            .collect();
        let mut reg = ProtocolRegistry::new();
        let got = reg.process_signal_stream(&pairs, freq);
        let kg: Vec<_> = got.iter().filter(|(n, _, _)| n == NAME).collect();
        assert!(
            !kg.is_empty(),
            "{:?}",
            got.iter().map(|(n, _, _)| n).collect::<Vec<_>>()
        );
        for (_, d, _) in kg {
            assert_eq!(
                (d.serial, d.button, d.counter),
                (Some(0x130F_8127), Some(0xE), Some(0x0B9D))
            );
        }
    }
}
