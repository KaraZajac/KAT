//! Allstar Firefly 318ALD31K (Supertex ED-9) 9-position trinary DIP gate/garage remote.
//!
//! Primary reference: Flipper-ARF's garage app copy
//! `REFERENCES/Flipper-ARF/applications/system/subghz_garage/protocols/allstar_firefly.c`
//! (decode only in the garage app: no TX plugin); doctor-firmware
//! `lib/subghz/protocols/allstar_firefly.c` decodes identically and registers the encoder, which
//! KAT takes (union). Flipper-ARF's `lib/subghz` copy is the same protocol (315 MHz flag only).
//!
//! **Line coding**: te_short = 600 µs, te_long = 4000 µs, te_delta = 300 µs. Each bit is a HIGH
//! then a LOW: 1 = HIGH 4000 + LOW 600, 0 = HIGH 600 + LOW 4000; frames are separated by a
//! LOW of 50·te_short (30 000 µs; the encoder sends 30 400). 18 bits, MSB first: nine trinary
//! DIP switches, two bits each ('+' = 11, '0' = 10, '−' = 00).
//!
//! **Decoder** (`subghz_protocol_decoder_allstar_firefly_feed`): Reset waits for a LOW within
//! 5·te_delta (1 500 µs) of the 30 ms gap; bits are HIGH/LOW pairs within te_delta. A gap-length
//! LOW closes the frame (the last bit read from its HIGH) and reports it when exactly 18 bits
//! were read, then the decoder returns to Reset (every other frame of a burst is reported, as in
//! the C). No checksum. (The garage copy reads the last bit with `else if`, doctor-firmware with
//! two `if`s; the two HIGH windows do not overlap, so they behave the same.)
//!
//! **Fields**: none (the references display the key, its bit reversal and the DIP pattern: switch
//! 1 is bits 17..16).
//!
//! **Encoder** (doctor-firmware `subghz_protocol_encoder_allstar_firefly_get_upload`): the 18
//! bit pairs, the last LOW being 50·te_short + 400 = 30 400 µs; sent 5 times. Static code, no
//! button or counter.
//!
//! **Key file**: `Protocol: Allstar Firefly`, `Bit: 18`, `Key`.

use super::g3_common::{duration_diff, repeat_upload, UploadBuilder};
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Allstar Firefly",
    decoder: "Allstar Firefly",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Allstar",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

const TE_SHORT: u32 = 600;
const TE_LONG: u32 = 4000;
const TE_DELTA: u32 = 300;
const BIT_COUNT: usize = 18;
/// Gap the decoder looks for: 50·te_short.
const GAP: u32 = TE_SHORT * 50;
/// Gap the encoder sends after the last bit.
const TX_GAP: u32 = TE_SHORT * 50 + 400;
/// `subghz_protocol_encoder_allstar_firefly_alloc`: repeat = 5.
const REPEAT: usize = 5;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    SaveDuration,
    CheckDuration,
}

/// Allstar Firefly decoder (`SubGhzProtocolDecoderAllstarFirefly`).
pub struct AllstarFireflyDecoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
}

fn signal(data: u64) -> DecodedSignal {
    DecodedSignal {
        serial: None,
        button: None,
        counter: None,
        crc_valid: false,
        data,
        data_count_bit: BIT_COUNT,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

/// One upload: 18 bit pairs, the last LOW being the gap.
fn upload(key: u64) -> Vec<LevelDuration> {
    let mut b = UploadBuilder::new();
    for i in (0..BIT_COUNT).rev() {
        let last = i == 0;
        if (key >> i) & 1 == 1 {
            b.push(true, TE_LONG);
            b.push(false, if last { TX_GAP } else { TE_SHORT });
        } else {
            b.push(true, TE_SHORT);
            b.push(false, if last { TX_GAP } else { TE_LONG });
        }
    }
    b.finish()
}

impl AllstarFireflyDecoder {
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

impl Default for AllstarFireflyDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for AllstarFireflyDecoder {
    fn name(&self) -> &'static str {
        "Allstar Firefly"
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
                if !level && duration_diff(duration, GAP) < TE_DELTA * 5 {
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
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
                if level {
                    self.step = Step::Reset;
                    return None;
                }
                if duration_diff(self.te_last, TE_LONG) < TE_DELTA
                    && duration_diff(duration, TE_SHORT) < TE_DELTA
                {
                    self.add_bit(1);
                    self.step = Step::SaveDuration;
                } else if duration_diff(self.te_last, TE_SHORT) < TE_DELTA
                    && duration_diff(duration, TE_LONG) < TE_DELTA
                {
                    self.add_bit(0);
                    self.step = Step::SaveDuration;
                } else if duration_diff(duration, GAP) < TE_DELTA * 5 {
                    if duration_diff(self.te_last, TE_LONG) < TE_DELTA {
                        self.add_bit(1);
                    } else if duration_diff(self.te_last, TE_SHORT) < TE_DELTA {
                        self.add_bit(0);
                    }
                    let found = (usize::from(self.decode_count_bit) == BIT_COUNT)
                        .then(|| signal(self.decode_data));
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.step = Step::Reset;
                    return found;
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
        (decoded.data_count_bit == BIT_COUNT).then(|| repeat_upload(&upload(decoded.data), REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        (key.bits == BIT_COUNT).then(|| signal(key.key))
    }

    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        (decoded.data_count_bit == BIT_COUNT).then(|| {
            FlipperKey::new(
                "Allstar Firefly",
                BIT_COUNT,
                decoded.data,
                318_000_000,
                "FuriHalSubGhzPresetOok650Async",
            )
        })
    }
}

#[cfg(test)]
mod tests {
    use super::super::g3_common::test_support::*;
    use super::*;

    /// The key file allstar_firefly.c's header comment shows.
    const KEY_FILE: &str = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 318000000\n\
        Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Allstar Firefly\nKey: 00 00 00 00 00 00 34 B9\n\
        Bit: 18\n";

    #[test]
    fn key_file_loads_encodes_and_decodes_back() {
        let d = AllstarFireflyDecoder::new();
        let s = d
            .from_flipper_key(&FlipperKey::parse(KEY_FILE).unwrap())
            .unwrap();
        assert_eq!((s.data, s.data_count_bit), (0x34B9, 18));
        let tx = d.encode(&s, 0).unwrap();
        assert_eq!(tx.len(), 5 * 36);
        let mut p = vec![LevelDuration::new(false, TX_GAP)];
        p.extend(tx);
        // gap + 5 copies: copies 1, 3 and 5 are reported.
        let frames = feed_all(&mut AllstarFireflyDecoder::new(), &p);
        assert_eq!(frames.len(), 3);
        assert!(frames
            .iter()
            .all(|f| f.data == 0x34B9 && f.encoder_capable && !f.crc_valid));
        let text = d.to_flipper_key(&s).unwrap().to_text();
        assert!(
            text.contains("Protocol: Allstar Firefly\nBit: 18\nKey: 00 00 00 00 00 00 34 B9\n"),
            "{text}"
        );
        let back = d
            .from_flipper_key(&FlipperKey::parse(&text).unwrap())
            .unwrap();
        assert_eq!(format!("{back:?}"), format!("{s:?}"));
    }

    #[test]
    fn synthetic_frame_and_rejections() {
        let data = 0b11_00_00_10_10_10_11_11_00u64;
        let mut p = vec![LevelDuration::new(false, GAP)];
        p.extend(upload(data));
        let frames = feed_all(&mut AllstarFireflyDecoder::new(), &p);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].data, data);
        // 17 bits.
        let mut short = p.clone();
        short.drain(1..3);
        assert!(feed_all(&mut AllstarFireflyDecoder::new(), &short).is_empty());
        // A HIGH between the windows.
        let mut bad = p.clone();
        bad[5].duration_us = 2000;
        assert!(feed_all(&mut AllstarFireflyDecoder::new(), &bad).is_empty());
        // Gap too short.
        let mut bad = p;
        bad[0].duration_us = 20_000;
        assert!(feed_all(&mut AllstarFireflyDecoder::new(), &bad).is_empty());
    }
}
