//! Jarolift: KeeLoq (normal learning) roller-shutter remotes.
//!
//! References: Flipper-ARF's garage app `applications/system/subghz_garage/protocols/jarolift.c`
//! (primary) and doctor-firmware `lib/subghz/protocols/jarolift.c`. Same air format, frame, check
//! and upload; they differ only in the display text and custom-button bookkeeping.
//!
//! **Air format** (`subghz_protocol_decoder_jarolift_feed`): te_short 400 µs, te_long 800 µs,
//! te_delta 167 µs (both sides). Preamble pulses HIGH 400 / LOW 400; with EXACTLY 8 of them counted,
//! a LOW of 4000 ± 1002 µs (te_long·5 ± te_delta·6) starts the data. Bit 1 = HIGH 400 + LOW 800,
//! bit 0 = HIGH 800 + LOW 400. The first 64 bits are stored when the 65th LOW arrives; a LOW of
//! 2400 µs or more ends the frame and adds a last bit from its HIGH (800 → 0, 400 → 1); exactly
//! 72 bits are a frame.
//!
//! **Frame**: 64 bits sent LSB first — `reverse_key(key, 64)` = button (4) | serial (28) | KeeLoq
//! hop (32) — then 8 "grouping" bits. Hop plaintext (normal learning with the Jarolift key):
//! group channels 0-7 (8 bits) << 24 | serial[7:0] << 16 | counter (16 bits). The C's "Group" is
//! `(plaintext >> 24) << 8 | (reverse_key(group bits, 8) >> 8)`; the second term is always 0, so
//! the received grouping byte is dropped and the encoder sends 0x00 there (mirrored).
//!
//! **Key**: the first keystore entry of learning type 11 (KEELOQ_LEARNING_NORMAL_JAROLIFT,
//! "Jarolift"; the C stops at the first one).
//!
//! **Gating**: the C calls back for every 72-bit frame and zeroes serial/button/counter/group
//! when `(plaintext >> 16) & 0xFF != serial & 0xFF` (display only); per KAT policy a frame is
//! reported only when that 8-bit check passes (`crc_valid`).
//!
//! **KAT fields**: `data` = the first 64 wire bits (C `generic.data`, key-file `Key`), `extra` =
//! the 8 grouping bits as received (C `generic.data_2`, key-file `Data`), `data_count_bit` = 72.
//! serial = 28 bits, button = 4 bits (1 Learn, 2 Down, 4 Stop, 8 Up), counter = 16 bits.
//!
//! **Encoder** (`subghz_protocol_encoder_jarolift_deserialize` + `gen_data` + `get_upload`): the
//! frame is analysed again, counter + 1 (0 past 0xFFFF), button = the requested native code (4
//! bits; the C's custom buttons cycle 1/2/4/8, any other code keeps the captured button), the hop
//! re-encrypted (plaintext channels from the received hop, grouping byte 0x00). Upload: LOW 14400,
//! HIGH 1500, LOW 400, 8 × (HIGH 400, LOW 400) with the last LOW 3800, 64 + 8 bits;
//! `encoder.repeat` = 3. (Flipper-ARF's garage TX plugin is built without
//! `SUBGHZ_GARAGE_HAS_CUSTOM_BTN` / `SUBGHZ_GARAGE_HAS_LIB_EXTENSIONS`; their fallbacks make it
//! send button 0 and take the "OFEX" counter path. KAT follows the source's intended path —
//! doctor-firmware's behaviour.)

use super::common::DecodedSignal;
use super::flipper_key::FlipperKey;
use super::g5_common::{hex8_field, key8, keystore_entry, next_counter, push};
use super::keeloq_common::{
    keeloq_decrypt, keeloq_encrypt, keeloq_normal_learning, learning_types as lt, reverse_key,
};
use super::{ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Jarolift",
    decoder: "Jarolift",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "KeeLoq",
    make: "Jarolift",
    category: Category::Gate,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// `subghz_protocol_jarolift_get_button_name`.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x1 => "Learn",
        0x2 => "Down",
        0x4 => "Stop",
        0x8 => "Up",
        _ => "Unkn",
    })
}

const NAME: &str = "Jarolift";
const TE_SHORT: u32 = 400;
const TE_LONG: u32 = 800;
const TE_DELTA: u32 = 167;
const MIN_COUNT_BIT: usize = 72;
/// C `encoder.repeat`.
const TX_REPEAT: usize = 3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    CheckPreambula,
    SaveDuration,
    CheckDuration,
}

/// The first type-11 keystore entry's key (the C stops at the first one).
fn jarolift_key() -> Option<u64> {
    keystore_entry(|e| e.learning == lt::NORMAL_JAROLIFT).map(|e| e.key)
}

/// A frame that passed `subghz_protocol_jarolift_remote_controller`'s check.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Fields {
    serial: u32,
    btn: u8,
    cnt: u16,
    /// C `generic.seed` ("Group"): plaintext channels 0-7 << 8 | (grouping byte >> 8 = 0).
    group: u32,
}

/// `subghz_protocol_jarolift_remote_controller`: None when the check fails.
fn analyze(data: u64, data_2: u64) -> Option<Fields> {
    let group = reverse_key(data_2, 8) as u32;
    let key = reverse_key(data, 64);
    let serial = ((key >> 32) & 0x0FFF_FFFF) as u32;
    let hop = key as u32;
    let decrypt = keeloq_decrypt(hop, keeloq_normal_learning(serial, jarolift_key()?));
    ((decrypt >> 16) & 0xFF == serial & 0xFF).then_some(Fields {
        serial,
        btn: ((key >> 60) & 0x0F) as u8,
        cnt: decrypt as u16,
        group: ((decrypt >> 24) << 8) | (group >> 8),
    })
}

fn signal(data: u64, data_2: u64) -> Option<DecodedSignal> {
    let f = analyze(data, data_2)?;
    Some(DecodedSignal {
        serial: Some(f.serial),
        button: Some(f.btn),
        counter: Some(f.cnt),
        crc_valid: true,
        data,
        data_count_bit: MIN_COUNT_BIT,
        encoder_capable: true,
        extra: Some(data_2 & 0xFF),
        protocol_display_name: None,
    })
}

/// `subghz_protocol_jarolift_gen_data`: (data, data_2) for a press; None when the hop encrypts
/// to 0 (the C's failure path) or the key is missing.
fn gen_data(serial: u32, btn: u8, cnt: u16, group: u32) -> Option<(u64, u64)> {
    let hop_decrypted = (((group >> 8) & 0xFF) << 24) | ((serial & 0xFF) << 16) | u32::from(cnt);
    let man = keeloq_normal_learning(serial, jarolift_key()?);
    let hop_encrypted = keeloq_encrypt(hop_decrypted, man);
    if hop_encrypted == 0 {
        return None;
    }
    let fix = (u64::from(btn & 0x0F) << 60)
        | (u64::from(serial & 0x0FFF_FFFF) << 32)
        | u64::from(hop_encrypted);
    Some((
        reverse_key(fix, 64),
        reverse_key(u64::from(group & 0xFF), 8),
    ))
}

/// `subghz_protocol_jarolift_create_data` (the SubGHz app's "Add Manually"): serial & 0xFFFFF00,
/// group 0x0100 (channel 1), the counter as given (not advanced), the hop under normal learning
/// with the type-11 key; `Bit: 72`, `Key`, `Data` (the grouping byte, 8 hex bytes). None without
/// the key.
pub(super) fn create_data(serial: u32, btn: u8, cnt: u16) -> Option<FlipperKey> {
    let serial = serial & 0x0FFF_FF00;
    let group: u32 = 0x0100;
    let hop_decrypted = (((group >> 8) & 0xFF) << 24) | ((serial & 0xFF) << 16) | u32::from(cnt);
    let hop = keeloq_encrypt(hop_decrypted, keeloq_normal_learning(serial, jarolift_key()?));
    let fix = (u64::from(btn & 0x0F) << 60) | (u64::from(serial) << 32) | u64::from(hop);
    let mut k = FlipperKey::for_protocol(NAME, MIN_COUNT_BIT, reverse_key(fix, 64));
    k.push_hex("Data", reverse_key(u64::from(group & 0xFF), 8), 8);
    Some(k)
}

/// One copy of `subghz_protocol_encoder_jarolift_get_upload`.
fn upload_once(data: u64, data_2: u64, out: &mut Vec<LevelDuration>) {
    push(out, false, TE_LONG * 18);
    push(out, true, 1500);
    push(out, false, TE_SHORT);
    for i in 0..8 {
        push(out, true, TE_SHORT);
        push(out, false, if i == 7 { 3800 } else { TE_SHORT });
    }
    for (word, bits) in [(data, 64), (data_2, 8)] {
        for i in (0..bits).rev() {
            if (word >> i) & 1 == 1 {
                push(out, true, TE_SHORT);
                push(out, false, TE_LONG);
            } else {
                push(out, true, TE_LONG);
                push(out, false, TE_SHORT);
            }
        }
    }
}

pub struct JaroliftDecoder {
    step: Step,
    header_count: u16,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: usize,
    /// C `generic.data`: the first 64 bits, stored at the 65th LOW.
    data: u64,
}

impl JaroliftDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            header_count: 0,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
            data: 0,
        }
    }

    fn add_bit(&mut self, bit: u64) {
        self.decode_data = (self.decode_data << 1) | bit;
        self.decode_count_bit += 1;
    }
}

impl Default for JaroliftDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for JaroliftDecoder {
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
                } else if !level
                    && self.header_count == 8
                    && duration_diff!(duration, TE_LONG * 5) < TE_DELTA * 6
                {
                    // Gap after the header: 4000 ± 1002 µs.
                    self.step = Step::SaveDuration;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.header_count = 0;
                } else {
                    self.step = Step::Reset;
                    self.header_count = 0;
                }
            }
            Step::SaveDuration => {
                if level {
                    self.te_last = duration;
                    self.step = Step::CheckDuration;
                } else {
                    self.header_count = 0;
                    self.step = Step::Reset;
                }
            }
            Step::CheckDuration => {
                if !level {
                    if self.decode_count_bit == 64 {
                        self.data = self.decode_data;
                        self.decode_data = 0;
                    }
                    if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA
                        && duration_diff!(duration, TE_LONG) < TE_DELTA
                    {
                        self.add_bit(1);
                        self.step = Step::SaveDuration;
                    } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA
                        && duration_diff!(duration, TE_SHORT) < TE_DELTA
                    {
                        self.add_bit(0);
                        self.step = Step::SaveDuration;
                    } else {
                        if duration >= TE_LONG * 3 {
                            // End of frame: the last bit from its HIGH.
                            if duration_diff!(self.te_last, TE_LONG) < TE_DELTA {
                                self.add_bit(0);
                            } else if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA {
                                self.add_bit(1);
                            }
                            let result = if self.decode_count_bit == MIN_COUNT_BIT {
                                // KAT: report only frames that pass the C's check.
                                signal(self.data, self.decode_data)
                            } else {
                                None
                            };
                            self.step = Step::Reset;
                            self.decode_data = 0;
                            self.decode_count_bit = 0;
                            self.header_count = 0;
                            return result;
                        }
                        self.step = Step::Reset;
                        self.header_count = 0;
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
        let f = analyze(decoded.data, decoded.extra?)?;
        let btn = if button <= 0x0F { button } else { f.btn };
        let cnt = next_counter(u32::from(f.cnt), 0xFFFF) as u16;
        let (data, data_2) = gen_data(f.serial, btn, cnt, f.group)?;
        let mut out = Vec::with_capacity(TX_REPEAT * 165);
        for _ in 0..TX_REPEAT {
            upload_once(data, data_2, &mut out);
        }
        Some(out)
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        // `subghz_protocol_decoder_jarolift_deserialize`: Bit must be 72, Key + Data (8 bytes).
        if key.bits != MIN_COUNT_BIT {
            return None;
        }
        signal(key8(key)?, hex8_field(key, "Data")?)
    }

    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        // `..._serialize`: generic block + "Data" = the grouping bits.
        let mut k = FlipperKey::new(NAME, MIN_COUNT_BIT, decoded.data, 0, "");
        k.push_hex("Data", decoded.extra?, 8);
        Some(k)
    }
}

/// A key file for a random valid frame (oracle driver fixtures): random serial, button,
/// counter, channels and received grouping byte.
#[cfg(test)]
pub(super) fn fixture_key(r: u64) -> FlipperKey {
    let cnt = if r & 0x30 == 0 {
        0xFFFF
    } else {
        (r >> 16) as u16
    };
    let serial = ((r >> 32) as u32) & 0x0FFF_FFFF;
    let channels = u32::from((r >> 8) as u8);
    let (data, _) = gen_data(serial, (r & 0x0F) as u8, cnt, channels << 8).unwrap();
    let group_byte = (r >> 24) & 0xFF;
    JaroliftDecoder::new()
        .to_flipper_key(&signal(data, group_byte).unwrap())
        .unwrap()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::ProtocolRegistry;

    fn decode_all(signal: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = JaroliftDecoder::new();
        let mut out = Vec::new();
        for ld in signal {
            if let Some(d) = dec.feed(ld.level, ld.duration_us) {
                out.push(d);
                dec.reset();
            }
        }
        out
    }

    /// One transmission followed by a gap.
    fn capture(
        serial: u32,
        btn: u8,
        cnt: u16,
        channels: u32,
        group_byte: u64,
    ) -> Vec<LevelDuration> {
        let (data, _) = gen_data(serial, btn, cnt, channels << 8).unwrap();
        let mut s = Vec::new();
        upload_once(data, reverse_key(group_byte, 8), &mut s);
        push(&mut s, false, 14_400); // the silence after the last bit's LOW
        s
    }

    /// The C's key samples (reversed key 0x821EB600EAC2EAD4 …: serial 0x06D7841, button Up,
    /// counters 0059..005B, group 0100) are not re-checkable (no Jarolift key in the comment's
    /// remote); a synthetic remote with the same layout is used instead.
    #[test]
    fn decodes_synthetic_frames() {
        for (serial, btn, cnt, channels) in [
            (0x06D_7841u32, 0x8u8, 0x0059u16, 0x01u32),
            (0x0FFF_FFFF, 0x2, 0xFFFF, 0xFF),
            (0x1, 0x4, 0, 0x80),
        ] {
            let d = decode_all(&capture(serial, btn, cnt, channels, 0x00));
            assert_eq!(d.len(), 1);
            let d = &d[0];
            assert_eq!(
                (d.serial, d.button, d.counter),
                (Some(serial), Some(btn), Some(cnt))
            );
            assert_eq!((d.data_count_bit, d.extra), (72, Some(0)));
            assert!(d.crc_valid && d.encoder_capable);
            assert_eq!(analyze(d.data, 0).unwrap().group, channels << 8);
        }
        // A received grouping byte is kept in `extra` but does not enter "Group" (C: `group >> 8`).
        let d = decode_all(&capture(0x0123_4567, 0x1, 7, 0x03, 0xA5))
            .pop()
            .unwrap();
        assert_eq!(d.extra, Some(reverse_key(0xA5, 8)));
        assert_eq!(analyze(d.data, d.extra.unwrap()).unwrap().group, 0x0300);
    }

    #[test]
    fn rejects_bad_frames() {
        // Failing check: hop for another serial.
        let (data, _) = gen_data(0x0123_4567, 0x1, 7, 0x100).unwrap();
        let key = reverse_key(data, 64);
        let other = reverse_key(key ^ (1 << 40), 64); // serial bit flipped, hop unchanged
        let mut s = Vec::new();
        upload_once(other, 0, &mut s);
        push(&mut s, false, 14_400); // the silence after the last bit's LOW
        assert!(decode_all(&s).is_empty());
        // 9 preamble pulses (the C needs exactly 8).
        let mut s = capture(0x0123_4567, 0x1, 7, 0x1, 0);
        s.splice(
            3..3,
            [
                LevelDuration::new(true, 400),
                LevelDuration::new(false, 400),
            ],
        );
        assert!(decode_all(&s).is_empty());
        // 71 bits.
        let mut s = capture(0x0123_4567, 0x1, 7, 0x1, 0);
        s.drain(30..32);
        assert!(decode_all(&s).is_empty());
    }

    #[test]
    fn encode_round_trip_every_button() {
        let dec = JaroliftDecoder::new();
        let captured = decode_all(&capture(0x06D_7841, 0x8, 0x0059, 0x01, 0x00))
            .pop()
            .unwrap();
        for button in [0x1u8, 0x2, 0x4, 0x8] {
            let mut wire = dec.encode(&captured, button).unwrap();
            assert_eq!(wire[0].duration_us, 14_400);
            push(&mut wire, false, 14_400);
            let got = decode_all(&wire);
            assert_eq!(got.len(), 3, "button {button}");
            for d in &got {
                assert_eq!(
                    (d.serial, d.button, d.counter, d.extra),
                    (Some(0x06D_7841), Some(button), Some(0x005A), Some(0))
                );
                assert_eq!(analyze(d.data, 0).unwrap().group, 0x0100);
            }
        }
        // Copy boundaries merge the last bit's LOW into the next 14400 µs LOW.
        let wire = dec.encode(&captured, 0x8).unwrap();
        assert!(
            wire.iter()
                .filter(|l| !l.level && l.duration_us > 14_400)
                .count()
                == 2
        );
        // Out-of-range code keeps the captured button; 16-bit counter wrap; grouping byte → 0x00.
        let top = decode_all(&capture(0x0ABC_DEF0, 0x4, 0xFFFF, 0x10, 0x5A))
            .pop()
            .unwrap();
        let mut wire = dec.encode(&top, 0x10).unwrap();
        push(&mut wire, false, 14_400);
        let next = decode_all(&wire).pop().unwrap();
        assert_eq!(
            (next.button, next.counter, next.extra),
            (Some(0x4), Some(0), Some(0))
        );
    }

    #[test]
    fn key_file_round_trip() {
        let dec = JaroliftDecoder::new();
        let d = decode_all(&capture(0x06D_7841, 0x8, 0x0059, 0x01, 0x00))
            .pop()
            .unwrap();
        let k = dec.to_flipper_key(&d).unwrap();
        let text = k.to_text();
        assert!(text.contains("Protocol: Jarolift\nBit: 72\nKey: "));
        assert!(text.ends_with("Data: 00 00 00 00 00 00 00 00\n"), "{text}");
        let again = dec
            .from_flipper_key(&FlipperKey::parse(&text).unwrap())
            .unwrap();
        assert_eq!(format!("{again:?}"), format!("{d:?}"));
        // Bit count, missing Data, failing check.
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&text.replace("Bit: 72", "Bit: 64")).unwrap())
            .is_none());
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&text.replace("Data:", "Dta:")).unwrap())
            .is_none());
        let flipped = FlipperKey {
            key: d.data ^ 1 << 63,
            ..FlipperKey::parse(&text).unwrap()
        }; // hop bit 0
        let flipped = FlipperKey {
            key_bytes: flipped.key.to_be_bytes().to_vec(),
            ..flipped
        };
        assert!(dec.from_flipper_key(&flipped).is_none());
        let reg = ProtocolRegistry::new();
        assert_eq!(
            reg.decode_key(&FlipperKey::parse(&text).unwrap())
                .map(|(n, _)| n),
            Some("Jarolift".to_string())
        );
    }

    #[test]
    fn button_names() {
        assert_eq!(
            crate::protocols::native_button_name("Jarolift", 0x8),
            Some("Up")
        );
        assert_eq!(
            crate::protocols::native_button_name("Jarolift", 0x3),
            Some("Unkn")
        );
    }
}
