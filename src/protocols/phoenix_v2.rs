//! Phoenix V2 ("V2 Phoenix") 52-bit gate remote with an obfuscated 16-bit counter.
//!
//! Primary reference: Flipper-ARF's garage app copy
//! `REFERENCES/Flipper-ARF/applications/system/subghz_garage/protocols/phoenix_v2.c`; the
//! doctor-firmware and Flipper-ARF `lib/subghz/protocols/phoenix_v2.c` copies decode and
//! transmit identically (they only differ in display text and in how the original button is
//! remembered for the button switch).
//!
//! **Line coding**: te_short = 427 µs, te_long = 853 µs, te_delta = 100 µs. A frame is a LOW
//! header of 60·te_short (25 620 µs), a HIGH start pulse of 6·te_short (2 562 µs) and 52 data
//! bits, MSB first, each a LOW then a HIGH: 1 = LOW 427 + HIGH 853, 0 = LOW 853 + HIGH 427.
//!
//! **Decoder** (`subghz_protocol_decoder_phoenix_v2_feed`): Reset needs a LOW within
//! 30·te_delta (3 000 µs) of the header, then a HIGH within 4·te_delta of the start pulse. In a
//! bit the short half must be within te_delta and the long half within 3·te_delta. A LOW of
//! ≥ 10·te_short + te_delta (4 370 µs) ends the frame and reports it when exactly 52 bits were
//! read. A HIGH while waiting for a bit's LOW is ignored, as in the C. No checksum.
//!
//! **Fields** (`subghz_protocol_phoenix_v2_check_remote_controller`): `rev` = the 52 bits
//! reversed over 56 bits (`subghz_protocol_blocks_reverse_key(data, 52 + 4)`, so the low
//! nibble of `rev` is always 0); serial = rev bits 0..31, button = rev bits 32..35, counter =
//! `decrypt_counter(rev)`: the encrypted counter (rev bits 40..55) un-shuffled over 16 rounds
//! with the serial's top two bytes as XOR keys.
//!
//! **Encoder** (`subghz_protocol_encoder_phoenix_v2_get_upload`): counter mode 0 (default)
//! adds 1 (0xFFFF wraps to 0); a key file's `CounterMode: 1` steps 0 → 1 → 0xFFFE → 0xFFFF → 0
//! and `CounterMode: 2` counts 0..4 (KAT keeps the mode in `extra`). The counter is re-encrypted
//! with the serial and the frame rebuilt as reverse_key(enc << 40 | button << 32 | serial, 56);
//! the upload is LOW 25 620, HIGH 2 562, the low 52 bits, sent 3 times. `button` (native code):
//! one of 1, 2, 4, 8, 3 when the captured button is one of them (the remote's buttons, all
//! reachable through the references' custom-button switch), otherwise the captured button.
//!
//! **Key file**: `Protocol: Phoenix_V2`, `Bit: 52`, `Key` (Flipper-ARF's generic serialize), plus
//! `CounterMode` when the loaded file had one.

use super::g3_common::{duration_diff, repeat_upload, reverse_key, UploadBuilder};
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Phoenix_V2",
    decoder: "Phoenix_V2",
    aliases: &["Phoenix V2", "V2 Phoenix"],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "Phoenix",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

const TE_SHORT: u32 = 427;
const TE_LONG: u32 = 853;
const TE_DELTA: u32 = 100;
const BIT_COUNT: usize = 52;
/// `subghz_protocol_encoder_phoenix_v2_alloc`: repeat = 3.
const REPEAT: usize = 3;
/// The buttons the references' custom-button switch moves between.
const BUTTONS: [u8; 5] = [0x1, 0x2, 0x4, 0x8, 0x3];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    FoundStartBit,
    SaveDuration,
    CheckDuration,
}

/// Phoenix V2 decoder (`SubGhzProtocolDecoderPhoenix_V2`).
pub struct PhoenixV2Decoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
}

/// `subghz_protocol_phoenix_v2_decrypt_counter`.
fn decrypt_counter(full_key: u64) -> u16 {
    let encrypted = ((full_key >> 40) & 0xFFFF) as u16;
    let mut byte1 = (encrypted >> 8) as u8;
    let mut byte2 = encrypted as u8;
    let xor_key1 = (full_key >> 24) as u8;
    let xor_key2 = ((full_key >> 16) & 0xFF) as u8;
    for _ in 0..16 {
        let msb_of_byte1 = byte1 & 0x80;
        let lsb_of_byte2 = byte2 & 1;
        byte2 = (byte2 >> 1) | msb_of_byte1;
        byte1 = (byte1 << 1) | lsb_of_byte2;
        if msb_of_byte1 == 0 {
            byte1 ^= xor_key1;
            byte2 = (byte2 ^ xor_key2) & 0x7F;
        }
    }
    (u16::from(byte2) << 8) | u16::from(byte1)
}

/// `subghz_protocol_phoenix_v2_encrypt_counter`.
fn encrypt_counter(full_key: u64, counter: u16) -> u16 {
    let xor_key1 = (full_key >> 24) as u8;
    let xor_key2 = ((full_key >> 16) & 0xFF) as u8;
    let mut byte2 = (counter >> 8) as u8;
    let mut byte1 = counter as u8;
    for _ in 0..16 {
        let msb_of_prev_byte1 = byte2 & 0x80;
        if msb_of_prev_byte1 == 0 {
            byte2 ^= xor_key2;
            byte1 ^= xor_key1;
        }
        let lsb_of_current_byte1 = byte1 & 1;
        byte2 = (byte2 << 1) | lsb_of_current_byte1;
        byte1 = (byte1 >> 1) | msb_of_prev_byte1;
    }
    (u16::from(byte1) << 8) | u16::from(byte2)
}

/// (serial, button, counter) — `subghz_protocol_phoenix_v2_check_remote_controller`.
fn fields(data: u64) -> (u32, u8, u16) {
    let rev = reverse_key(data, BIT_COUNT as u8 + 4);
    (rev as u32, ((rev >> 32) & 0xF) as u8, decrypt_counter(rev))
}

/// The counter step of `get_upload` for a `CounterMode` (OFEX / mult overrides not modelled:
/// the rolling-counter multiplier is the default 1).
fn next_counter(counter: u16, mode: u64) -> u16 {
    match mode {
        0 => counter.checked_add(1).unwrap_or(0),
        1 => {
            if counter == 0xFFFF {
                0
            } else if counter >= 1 && counter != 0xFFFE {
                0xFFFE
            } else {
                counter + 1
            }
        }
        _ => {
            if counter >= 4 {
                0
            } else {
                counter + 1
            }
        }
    }
}

/// The 56-bit frame value `get_upload` builds (the low 52 bits go on air).
fn build_key(data: u64, button: u8, counter: u16) -> u64 {
    let rev = reverse_key(data, BIT_COUNT as u8 + 4);
    let serial = rev & 0xFFFF_FFFF;
    let enc = encrypt_counter(rev, counter);
    reverse_key(
        (u64::from(enc) << 40) | (u64::from(button) << 32) | serial,
        BIT_COUNT as u8 + 4,
    )
}

/// `subghz_protocol_phoenix_v2_create_data` (the SubGHz app's "Add Manually"): button 1, the counter
/// as given (not advanced) encrypted with the serial; `Bit: 52`, `Key`.
pub(super) fn create_data(serial: u32, cnt: u16) -> FlipperKey {
    let plain = (u64::from(cnt) << 40) | (1 << 32) | u64::from(serial);
    let enc = encrypt_counter(plain, cnt);
    let data = reverse_key((u64::from(enc) << 40) | (1 << 32) | u64::from(serial), BIT_COUNT as u8 + 4)
        & ((1 << BIT_COUNT) - 1);
    FlipperKey::for_protocol("Phoenix_V2", BIT_COUNT, data)
}

/// One upload (`get_upload`): header, start pulse, the low 52 bits.
fn upload(key: u64) -> Vec<LevelDuration> {
    let mut b = UploadBuilder::new();
    b.push(false, TE_SHORT * 60);
    b.push(true, TE_SHORT * 6);
    for i in (0..BIT_COUNT).rev() {
        if (key >> i) & 1 == 0 {
            b.push(false, TE_LONG);
            b.push(true, TE_SHORT);
        } else {
            b.push(false, TE_SHORT);
            b.push(true, TE_LONG);
        }
    }
    b.finish()
}

fn signal(data: u64, counter_mode: Option<u64>) -> DecodedSignal {
    let (serial, button, counter) = fields(data);
    DecodedSignal {
        serial: Some(serial),
        button: Some(button),
        counter: Some(counter),
        crc_valid: false,
        data,
        data_count_bit: BIT_COUNT,
        encoder_capable: true,
        extra: counter_mode,
        protocol_display_name: None,
    }
}

impl PhoenixV2Decoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
        }
    }

    fn add_bit(&mut self, bit: u64) {
        self.decode_data = (self.decode_data << 1) | bit;
        self.decode_count_bit = self.decode_count_bit.wrapping_add(1);
    }
}

impl Default for PhoenixV2Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for PhoenixV2Decoder {
    fn name(&self) -> &'static str {
        "Phoenix_V2"
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

    fn reset(&mut self) {
        self.step = Step::Reset;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                if !level && duration_diff(duration, TE_SHORT * 60) < TE_DELTA * 30 {
                    self.step = Step::FoundStartBit;
                }
            }
            Step::FoundStartBit => {
                if level && duration_diff(duration, TE_SHORT * 6) < TE_DELTA * 4 {
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
                        let found = (usize::from(self.decode_count_bit) == BIT_COUNT)
                            .then(|| signal(self.decode_data, None));
                        self.decode_data = 0;
                        self.decode_count_bit = 0;
                        return found;
                    }
                    self.te_last = duration;
                    self.step = Step::CheckDuration;
                }
            }
            Step::CheckDuration => {
                if level {
                    if duration_diff(self.te_last, TE_SHORT) < TE_DELTA
                        && duration_diff(duration, TE_LONG) < TE_DELTA * 3
                    {
                        self.add_bit(1);
                        self.step = Step::SaveDuration;
                    } else if duration_diff(self.te_last, TE_LONG) < TE_DELTA * 3
                        && duration_diff(duration, TE_SHORT) < TE_DELTA
                    {
                        self.add_bit(0);
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
        if decoded.data_count_bit != BIT_COUNT {
            return None;
        }
        let (_, captured_button, captured_counter) = fields(decoded.data);
        let button = if BUTTONS.contains(&captured_button) && BUTTONS.contains(&button) {
            button
        } else {
            captured_button
        };
        let counter = next_counter(
            decoded.counter.unwrap_or(captured_counter),
            decoded.extra.unwrap_or(0),
        );
        Some(repeat_upload(
            &upload(build_key(decoded.data, button, counter)),
            REPEAT,
        ))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != BIT_COUNT {
            return None;
        }
        let mode = key.u32_field("CounterMode").map(|m| u64::from(m as u8));
        Some(signal(key.key, mode))
    }

    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != BIT_COUNT {
            return None;
        }
        let mut k = FlipperKey::new(
            "Phoenix_V2",
            BIT_COUNT,
            decoded.data,
            433_920_000,
            "FuriHalSubGhzPresetOok650Async",
        );
        if let Some(mode) = decoded.extra {
            k.push("CounterMode", mode.to_string());
        }
        Some(k)
    }
}

#[cfg(test)]
mod tests {
    use super::super::g3_common::test_support::*;
    use super::*;

    /// phoenix_v2.sub (Flipper unit test).
    const KEY_FILE: &str = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
        Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Phoenix_V2\nBit: 52\n\
        Key: 00 0F 0F BD 7E 7F 10 AE\n";

    #[test]
    fn counter_cipher_matches_the_c_samples() {
        // check_remote_controller's samples: serial B9615720, encrypted C63E → 025C, …
        let serial = 0xB961_5720u64;
        for (enc, dec) in [
            (0xC63Eu16, 0x025Cu16),
            (0xBCC1, 0x025D),
            (0x3341, 0x025E),
            (0x49BE, 0x025F),
            (0x99D3, 0x0260),
            (0xE32C, 0x0261),
        ] {
            let rev = (u64::from(enc) << 40) | (1 << 32) | serial;
            assert_eq!(decrypt_counter(rev), dec, "{enc:04X}");
            assert_eq!(encrypt_counter(rev, dec), enc, "{dec:04X}");
        }
        // encrypt/decrypt are inverses for every counter.
        for c in 0..=0xFFFFu16 {
            let rev = (u64::from(encrypt_counter(serial, c)) << 40) | serial;
            assert_eq!(decrypt_counter(rev), c);
        }
    }

    #[test]
    fn unit_test_key_file_loads_and_encodes() {
        let d = PhoenixV2Decoder::new();
        let key = FlipperKey::parse(KEY_FILE).unwrap();
        let s = d.from_flipper_key(&key).unwrap();
        assert_eq!(s.data, 0x000F_0FBD_7E7F_10AE);
        let (serial, button, counter) = fields(s.data);
        assert_eq!(
            (s.serial, s.button, s.counter),
            (Some(serial), Some(button), Some(counter))
        );
        // The upload decodes back (2 of the 3 copies: the last has no trailing gap) with the
        // counter + 1 and the same serial / button.
        let tx = d.encode(&s, button).unwrap();
        let frames = feed_all(&mut PhoenixV2Decoder::new(), &tx);
        assert_eq!(frames.len(), 2);
        for f in &frames {
            assert_eq!((f.serial, f.button), (s.serial, s.button));
            assert_eq!(f.counter, Some(counter.wrapping_add(1)));
        }
        // Key file round trip.
        let text = d.to_flipper_key(&s).unwrap().to_text();
        assert!(text.contains("Protocol: Phoenix_V2\nBit: 52\nKey: 00 0F 0F BD 7E 7F 10 AE\n"));
        let back = d
            .from_flipper_key(&FlipperKey::parse(&text).unwrap())
            .unwrap();
        assert_eq!(format!("{back:?}"), format!("{s:?}"));
    }

    #[test]
    fn synthetic_frame_decodes_with_fields() {
        let serial = 0x1234_5670u64;
        let rev = (u64::from(encrypt_counter(serial, 0x0ABC)) << 40) | (0x4 << 32) | serial;
        let data = reverse_key(rev, 56) & ((1 << 52) - 1);
        let mut p = upload(data);
        p.push(LevelDuration::new(false, 25_620));
        let frames = feed_all(&mut PhoenixV2Decoder::new(), &p);
        assert_eq!(frames.len(), 1);
        let f = &frames[0];
        assert_eq!((f.data, f.data_count_bit), (data, 52));
        assert_eq!(
            (f.serial, f.button, f.counter),
            (Some(0x1234_5670), Some(4), Some(0x0ABC))
        );
    }

    #[test]
    fn encode_switches_buttons_and_counter_modes() {
        let serial = 0x0BAD_F00Du64 & !0xF;
        let rev = (u64::from(encrypt_counter(serial, 0xFFFF)) << 40) | (0x2 << 32) | serial;
        let s = signal(reverse_key(rev, 56) & ((1 << 52) - 1), None);
        let decode_tx = |s: &DecodedSignal, button: u8| {
            let tx = PhoenixV2Decoder::new().encode(s, button).unwrap();
            feed_all(&mut PhoenixV2Decoder::new(), &tx)[0].clone()
        };
        for (button, want) in [(2u8, 2u8), (1, 1), (4, 4), (8, 8), (3, 3), (5, 2), (0, 2)] {
            let f = decode_tx(&s, button);
            assert_eq!(f.button, Some(want), "button {button}");
            assert_eq!(f.serial, s.serial);
            assert_eq!(f.counter, Some(0), "0xFFFF + 1 wraps");
        }
        // A captured button outside the switch set is always kept.
        let rev5 = (u64::from(encrypt_counter(serial, 7)) << 40) | (0x5 << 32) | serial;
        let s5 = signal(reverse_key(rev5, 56) & ((1 << 52) - 1), None);
        assert_eq!(decode_tx(&s5, 1).button, Some(5));
        // CounterMode 1 / 2.
        for (mode, from, to) in [
            (1u64, 0u16, 1u16),
            (1, 1, 0xFFFE),
            (1, 0x25C, 0xFFFE),
            (1, 0xFFFE, 0xFFFF),
            (1, 0xFFFF, 0),
            (2, 3, 4),
            (2, 4, 0),
            (2, 9, 0),
            (0, 9, 10),
        ] {
            let r = (u64::from(encrypt_counter(serial, from)) << 40) | (0x1 << 32) | serial;
            let s = signal(reverse_key(r, 56) & ((1 << 52) - 1), Some(mode));
            assert_eq!(
                decode_tx(&s, 1).counter,
                Some(to),
                "mode {mode} from {from:#x}"
            );
        }
    }

    #[test]
    fn rejects_bad_frames() {
        let data = 0x000F_0FBD_7E7F_10AE;
        let mut p = upload(data);
        p.push(LevelDuration::new(false, 25_620));
        // Start pulse out of window.
        let mut bad = p.clone();
        bad[1].duration_us = 3200;
        assert!(feed_all(&mut PhoenixV2Decoder::new(), &bad).is_empty());
        // One bit missing.
        let mut short = p.clone();
        short.drain(2..4);
        assert!(feed_all(&mut PhoenixV2Decoder::new(), &short).is_empty());
        // Header too short.
        let mut bad = p;
        bad[0].duration_us = 20_000;
        assert!(feed_all(&mut PhoenixV2Decoder::new(), &bad).is_empty());
    }

    #[test]
    fn decodes_the_flipper_unit_test_capture() {
        let Some(path) = unit_test_file("phoenix_v2_raw.sub") else {
            return;
        };
        let frames = decoder_decodes(&mut PhoenixV2Decoder::new(), &path);
        assert!(
            !frames.is_empty(),
            "the reference unit test needs ≥ 1 frame"
        );
        let names: Vec<String> = registry_decodes(&path)
            .into_iter()
            .map(|(n, _)| n)
            .collect();
        assert!(names.iter().any(|n| n == "Phoenix_V2"), "{names:?}");
    }
}
