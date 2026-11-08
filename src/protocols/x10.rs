//! X10: 32-bit home-automation remote code (X10 RF: house code, unit, On / Off / Dim / Bright),
//! decoder + encoder.
//!
//! Port of Flipper-ARF's garage app `applications/system/subghz_garage/protocols/x10.c`
//! (@CodeAllNight's decoder; the app registers it in RX group g1 and ships a TX plugin,
//! `tx_x10`, so it decodes and encodes). Momentum, RogueMaster and FoxFW's `lib` carry the same
//! decoder — reset, validate, feed and the field split are identical, only the formatting differs —
//! without an encoder (their encoder vtable is NULL; FoxFW's SD plugin `enc_x10.c` is NULL too, its
//! `decode_x10.c` prints the garage app's text). FoxFW's SubRATT random-key app vendors the garage
//! file with `type` Static instead of Dynamic. doctor-firmware does not have X10.
//!
//! **Air format** (AM; the C flags 315 MHz, "technically it is 310MHz only"): te_short = 600 µs,
//! te_long = 1800 µs, te_delta = 100 µs. A preamble HIGH 9600 µs (16·te_short) and LOW 4800 µs
//! (8·te_short), then 32 bits MSB first, each a HIGH 600 followed by a LOW 600 (0) or 1800 (1), and
//! a closing HIGH 600. The encoder ends each copy with a LOW of 43200 µs (72·te_short).
//!
//! **Decoder** (`subghz_protocol_decoder_x10_feed`): Reset waits for a HIGH with |d − 9600| < 700;
//! the next pulse must be a LOW with |d − 4800| < 500 (anything else resets); then HIGH/LOW pairs:
//! HIGH |d − 600| < 100, LOW |d − 600| < 100 → 0, |d − 1800| < 200 → 1, anything else resets.
//! After 32 bits the closing HIGH (|d − 600| < 100) ends the frame; it is reported when byte 2 is
//! the complement of byte 1 and byte 4 the complement of byte 3
//! (`subghz_protocol_x10_validate`). The decoder then resets (data, count and step: the C's reset is
//! complete), so every copy of a press decodes.
//!
//! **Frame** (the C's comment, from the CM17A protocol): `SSSSXBXX ssssxbxx DBOQBXXX dboqbxxx`, the
//! lower-case bytes the complements. S = house code (the C's `serial`, shown through the letter
//! table "MNOPCDABEFGHKLIJ"), B = unit bits, Q = Off, D = dimmer.
//! `subghz_protocol_x10_check_remote_controller`: serial = bits 31..28, btn = bits 26..24 |
//! (bits 15..11 << 3) (byte 1's low three bits and byte 3's high five). The C displays the unit as
//! 1 + (Q-bit 0x10 → 1, 0x08 → 2, 0x40 → 4, byte-1 bit 0x04 → 8), Off when btn & 0x20, and the two
//! dimmer codes btn 0x98 = Dim, 0x88 = Bright.
//!
//! **KAT fields**: `serial` and `button` as the C (labels: "Button 3 Off", "Dim", "Bright", the C's
//! text); `counter` None; `data` = the 32 bits; `crc_valid` = both complement checks hold (always
//! for decodes: the C drops the others).
//!
//! **Encoder** (`subghz_protocol_encoder_x10_get_upload`, `encoder.repeat` = 3): the preamble, the
//! 32 bits, the closing HIGH and the 43200 µs LOW, 3 copies. The captured code is sent: no copy
//! of the C can change the button (`button` is ignored).
//!
//! **Key file**: `Protocol: X10`, `Bit: 32`, `Key` (`subghz_block_generic_serialize`); loading
//! requires `Bit: 32` (the decoder's and the encoder's deserialize).

use super::f_common::{add_bit, push, with_repeats};
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

/// Metadata (see `info.rs`): `SubGhzProtocolFlag_315 | AM` (type Dynamic in the C, a static code).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "X10",
    category: Category::Gate,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

pub(super) const NAME: &str = "X10";
const TE_SHORT: u32 = 600;
const TE_LONG: u32 = 1800;
const TE_DELTA: u32 = 100;
const MIN_COUNT_BIT: usize = 32;
/// `subghz_protocol_encoder_x10_alloc`: repeat = 3.
const TX_REPEAT: usize = 3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    FoundPreambula,
    SaveDuration,
    CheckDuration,
}

/// X10 decoder (`SubGhzProtocolDecoderX10`).
pub struct X10Decoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
}

/// `subghz_protocol_x10_validate`'s complement checks (byte 2 = !byte 1, byte 4 = !byte 3).
fn complements_hold(data: u64) -> bool {
    ((data >> 24) ^ (data >> 16)) & 0xFF == 0xFF && ((data >> 8) ^ data) & 0xFF == 0xFF
}

/// `subghz_protocol_x10_check_remote_controller`: (serial = house code, btn).
fn fields(data: u64) -> (u32, u8) {
    let serial = ((data & 0xF000_0000) >> 28) as u32;
    let btn = (((data & 0x0700_0000) >> 24) | ((data & 0xF800) >> 8)) as u8;
    (serial, btn)
}

/// The unit number the C displays for `btn` (1..=16).
pub(super) fn unit(btn: u8) -> u8 {
    1 + (((btn & 0x10) >> 4) | ((btn & 0x08) >> 2) | ((btn & 0x40) >> 4) | ((btn & 0x04) << 1))
}

/// `subghz_protocol_decoder_x10_get_string`'s "Button:%ld %s" (Dim / Bright for the two dimmer
/// codes, which the C shows as button 0).
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    const LABELS: [&str; 32] = [
        "Button 1 On", "Button 1 Off", "Button 2 On", "Button 2 Off", "Button 3 On", "Button 3 Off",
        "Button 4 On", "Button 4 Off", "Button 5 On", "Button 5 Off", "Button 6 On", "Button 6 Off",
        "Button 7 On", "Button 7 Off", "Button 8 On", "Button 8 Off", "Button 9 On", "Button 9 Off",
        "Button 10 On", "Button 10 Off", "Button 11 On", "Button 11 Off", "Button 12 On",
        "Button 12 Off", "Button 13 On", "Button 13 Off", "Button 14 On", "Button 14 Off",
        "Button 15 On", "Button 15 Off", "Button 16 On", "Button 16 Off",
    ];
    Some(match code {
        0x98 => "Dim",
        0x88 => "Bright",
        _ => LABELS[usize::from(unit(code) - 1) * 2 + usize::from(code & 0x20 != 0)],
    })
}

/// A decode (or a loaded key) of `data` (the C keeps the key file's whole 64-bit value).
fn decoded(data: u64) -> DecodedSignal {
    let (serial, btn) = fields(data);
    DecodedSignal {
        serial: Some(serial),
        button: Some(btn),
        counter: None,
        crc_valid: complements_hold(data),
        data,
        data_count_bit: MIN_COUNT_BIT,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

/// One `get_upload` copy.
fn build_frame(data: u64) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(MIN_COUNT_BIT * 2 + 4);
    push(&mut out, true, TE_SHORT * 16);
    push(&mut out, false, TE_SHORT * 8);
    for i in (0..MIN_COUNT_BIT).rev() {
        push(&mut out, true, TE_SHORT);
        push(&mut out, false, if (data >> i) & 1 == 1 { TE_LONG } else { TE_SHORT });
    }
    push(&mut out, true, TE_SHORT);
    push(&mut out, false, TE_SHORT * 72);
    out
}

impl X10Decoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
        }
    }

    /// `subghz_protocol_decoder_x10_reset`.
    fn clear(&mut self) {
        self.decode_data = 0;
        self.decode_count_bit = 0;
        self.step = Step::Reset;
    }
}

impl Default for X10Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for X10Decoder {
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
        self.clear();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                if level && duration_diff!(duration, TE_SHORT * 16) < TE_DELTA * 7 {
                    self.step = Step::FoundPreambula;
                }
            }
            Step::FoundPreambula => {
                if !level && duration_diff!(duration, TE_SHORT * 8) < TE_DELTA * 5 {
                    self.step = Step::SaveDuration;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                } else {
                    self.clear();
                }
            }
            Step::SaveDuration => {
                if level && duration_diff!(duration, TE_SHORT) < TE_DELTA {
                    if usize::from(self.decode_count_bit) == MIN_COUNT_BIT {
                        // The closing HIGH: report a valid frame; the C resets (fully) either way.
                        let data = self.decode_data;
                        self.clear();
                        return complements_hold(data).then(|| decoded(data));
                    }
                    self.te_last = duration;
                    self.step = Step::CheckDuration;
                } else {
                    self.clear();
                }
            }
            Step::CheckDuration => {
                let high_ok = duration_diff!(self.te_last, TE_SHORT) < TE_DELTA;
                if !level && high_ok && duration_diff!(duration, TE_SHORT) < TE_DELTA {
                    add_bit(&mut self.decode_data, &mut self.decode_count_bit, false);
                    self.step = Step::SaveDuration;
                } else if !level && high_ok && duration_diff!(duration, TE_LONG) < TE_DELTA * 2 {
                    add_bit(&mut self.decode_data, &mut self.decode_count_bit, true);
                    self.step = Step::SaveDuration;
                } else {
                    self.clear();
                }
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        (decoded.data_count_bit == MIN_COUNT_BIT)
            .then(|| with_repeats(&build_frame(decoded.data), TX_REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        (key.bits == MIN_COUNT_BIT).then(|| decoded(key.key))
    }

    fn to_flipper_key(&self, d: &DecodedSignal) -> Option<FlipperKey> {
        (d.data_count_bit == MIN_COUNT_BIT).then(|| FlipperKey::for_protocol(NAME, MIN_COUNT_BIT, d.data))
    }
}

#[cfg(test)]
mod tests {
    use super::super::f_common::test_support::*;
    use super::*;
    use crate::protocols::ProtocolRegistry;

    /// The C's `CHANNEL_LETTERS`: the house code letter of each `serial`.
    const CHANNEL_LETTERS: &[u8; 16] = b"MNOPCDABEFGHKLIJ";

    /// House A (serial 6), unit 1, On: the frame `60 9F 00 FF`.
    const A1_ON: u64 = 0x609F_00FF;

    /// Frame builders.
    mod testing {
        /// The 32-bit frame of house code `house` (0..=15, the C's serial) and command byte `cmd`
        /// (byte 3), with byte 1's low bits `b1_low` (bit 2 = unit bit 8) and both complements.
        pub fn frame(house: u8, b1_low: u8, cmd: u8) -> u64 {
            let b1 = (house & 0xF) << 4 | (b1_low & 0xF);
            u64::from(b1) << 24 | u64::from(!b1) << 16 | u64::from(cmd) << 8 | u64::from(!cmd)
        }
    }

    fn on_air(data: u64) -> Vec<LevelDuration> {
        let mut wire = vec![LevelDuration::new(false, 20_000)];
        for p in X10Decoder::new().encode(&decoded(data), 0).unwrap() {
            push(&mut wire, p.level, p.duration_us);
        }
        wire
    }

    /// Every copy decodes (the C's reset after a frame is complete and the preamble opens each).
    #[test]
    fn encode_decode_roundtrip() {
        for data in [A1_ON, testing::frame(0xF, 0x4, 0x98), testing::frame(0x0, 0x0, 0x88), testing::frame(0x9, 0x3, 0x7C)] {
            let got = feed_all(&mut X10Decoder::new(), &on_air(data));
            assert_eq!(got.len(), 3, "{data:08X}");
            for d in &got {
                assert_eq!((d.data, d.data_count_bit), (data, 32));
                assert_eq!((d.serial, d.button), (Some(fields(data).0), Some(fields(data).1)));
                assert!(d.crc_valid && d.encoder_capable && d.counter.is_none());
            }
        }
    }

    #[test]
    fn upload_layout() {
        let up = X10Decoder::new().encode(&decoded(0x8000_0001), 0).unwrap();
        let p = |i: usize| (up[i].level, up[i].duration_us);
        assert_eq!(p(0), (true, 9600));
        assert_eq!(p(1), (false, 4800));
        assert_eq!(p(2), (true, 600));
        assert_eq!(p(3), (false, 1800)); // bit 31 = 1
        assert_eq!(p(5), (false, 600)); // bit 30 = 0
        assert_eq!(p(65), (false, 1800)); // bit 0 = 1
        assert_eq!(p(66), (true, 600));
        assert_eq!(p(67), (false, 43_200));
        // 68 entries a copy (the C's 70-entry buffer), nothing merges at the seams; 3 copies.
        assert_eq!(up.len(), 68 * 3);
        assert!(X10Decoder::new().encode(&DecodedSignal::new(A1_ON, 24), 0).is_none());
    }

    /// The C's fields and display (button number, Off bit, the two dimmer codes, house letters).
    #[test]
    fn fields_and_labels_follow_the_c() {
        let d = decoded(A1_ON);
        assert_eq!((d.serial, d.button), (Some(6), Some(0x00)));
        assert_eq!(CHANNEL_LETTERS[6], b'A');
        assert_eq!(native_buttons(NAME, 0x00), Some("Button 1 On"));
        // byte 3 = 0x20 (Off), byte 1 bit 2 (unit 9..16): "Button 9 Off".
        let d = decoded(testing::frame(0x6, 0x4, 0x20));
        assert_eq!(d.button, Some(0x24));
        assert_eq!(native_buttons(NAME, 0x24), Some("Button 9 Off"));
        // Every unit bit set: unit 16.
        assert_eq!(unit(0x10 | 0x08 | 0x40 | 0x04), 16);
        assert_eq!(native_buttons(NAME, 0x98), Some("Dim"));
        assert_eq!(native_buttons(NAME, 0x88), Some("Bright"));
        assert_eq!(fields(testing::frame(0x0, 0x0, 0x98)).1, 0x98);
        assert_eq!(fields(testing::frame(0x0, 0x0, 0x88)).1, 0x88);
        // The serial is the top nibble only; every label is defined.
        assert_eq!(fields(0xFFFF_FFFF), (0xF, 0xFF));
        assert!((0..=255u8).all(|c| native_buttons(NAME, c).is_some()));
    }

    #[test]
    fn rejects_broken_complements_wrong_length_and_timing() {
        // Byte 2 not the complement of byte 1 / byte 4 of byte 3.
        for bad in [A1_ON ^ 0x0001_0000, A1_ON ^ 0x0000_0001] {
            assert!(feed_all(&mut X10Decoder::new(), &on_air(bad)).is_empty(), "{bad:08X}");
        }
        // 31 bits: the closing HIGH arrives as bit 32's HIGH, then the gap breaks the frame.
        let mut short = on_air(A1_ON);
        short.drain(3..5);
        assert!(feed_all(&mut X10Decoder::new(), &short[..68]).is_empty());
        // Preamble windows: HIGH 8901..=10299, LOW 4301..=5299 (strict, as the C).
        for (i, d, ok) in [(1usize, 8900u32, false), (1, 8901, true), (1, 10_299, true), (1, 10_300, false),
                           (2, 4300, false), (2, 4301, true), (2, 5299, true), (2, 5300, false)] {
            let mut w = on_air(A1_ON);
            w.truncate(69);
            w[i].duration_us = d;
            assert_eq!(feed_all(&mut X10Decoder::new(), &w).len(), usize::from(ok), "pulse {i} = {d}");
        }
        // Bit windows: HIGH 501..=699, LOW 0 501..=699, LOW 1 1601..=1999 (A1_ON's bit 31 is 0,
        // bit 30 is 1).
        for (i, d, ok) in [(3usize, 500u32, false), (3, 501, true), (3, 699, true), (3, 700, false),
                           (4, 500, false), (4, 501, true), (4, 699, true), (4, 700, false),
                           (6, 1600, false), (6, 1601, true), (6, 1999, true), (6, 2000, false)] {
            let mut w = on_air(A1_ON);
            w.truncate(69);
            w[i].duration_us = d;
            assert_eq!(feed_all(&mut X10Decoder::new(), &w).len(), usize::from(ok), "pulse {i} = {d}");
        }
    }

    #[test]
    fn key_file_round_trip() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 310000000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: X10\nBit: 32\n\
                    Key: 00 00 00 00 60 9F 00 FF\n";
        let dec = X10Decoder::new();
        let d = dec.from_flipper_key(&FlipperKey::parse(text).unwrap()).unwrap();
        assert_eq!((d.data, d.serial, d.button, d.crc_valid), (A1_ON, Some(6), Some(0), true));
        let written = super::super::flipper_key::testing::assert_round_trip(&dec, &d);
        assert!(written.contains("Protocol: X10\nBit: 32\nKey: 00 00 00 00 60 9F 00 FF\n"), "{written}");
        assert!(dec.from_flipper_key(&FlipperKey::parse(&text.replace("Bit: 32", "Bit: 24")).unwrap()).is_none());
        // A key whose complements fail loads too (the C's deserialize checks the bit count only).
        let odd = dec.from_flipper_key(&FlipperKey::parse(&text.replace("00 FF\n", "00 FE\n")).unwrap()).unwrap();
        assert!(!odd.crc_valid);
    }

    #[test]
    fn jittered_frames_decode() {
        let mut rng = Rng(0x0010_C0DE);
        for _ in 0..50 {
            let data = testing::frame(rng.below(16) as u8, rng.below(16) as u8, rng.next() as u8);
            let wire = jitter(&on_air(data), &mut rng, 80);
            let got = feed_all(&mut X10Decoder::new(), &wire);
            assert_eq!(got.len(), 3, "{data:08X}");
            assert!(got.iter().all(|d| d.data == data));
        }
    }

    /// Through the whole registry (AM capture at 310 MHz): every copy is X10's and nothing else
    /// reads the transmission.
    #[test]
    fn registry_reads_x10_only() {
        let mut rng = Rng(0x0310_0000);
        for _ in 0..20 {
            let data = testing::frame(rng.below(16) as u8, rng.below(16) as u8, rng.next() as u8);
            let mut reg = ProtocolRegistry::new();
            let got: Vec<String> = reg
                .process_signal_stream_rf(&on_air(data), 310_000_000, Some(crate::capture::RfModulation::AM))
                .into_iter()
                .map(|(n, d, _)| format!("{n} {:08X}", d.data))
                .collect();
            assert_eq!(got, vec![format!("{NAME} {data:08X}"); 3]);
        }
    }
}
