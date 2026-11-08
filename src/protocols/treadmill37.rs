//! Treadmill37: 37-bit fixed code of QH-433 treadmill remotes, decoder + encoder.
//!
//! Reference: doctor-firmware `lib/subghz/protocols/treadmill37.c` (@xMasterX, 2026;
//! `SUBGHZ_PROTOCOL_TREADMILL37_NAME` = "Treadmill37"), which doctor registers. The copies in
//! Flipper-ARF `lib` (unregistered there), the garage app (`applications/system/subghz_garage/
//! protocols/`, not in its registry), Momentum, RogueMaster and FoxFW (encoder in its SD plugin
//! `enc_treadmill37`) have the same feed and upload code; they differ only in style (allocation
//! helpers, the hash function).
//!
//! **Air format** (AM, 433.92 MHz): PWM, te_short 300 µs, te_long 900 µs, te_delta 150 µs. Each bit
//! is a HIGH then a LOW: 1 = HIGH 900 + LOW 300, 0 = HIGH 300 + LOW 900; the last bit's LOW is the
//! 6000 µs (20·te_short) gap between frames. 37 bits, MSB first, no preamble.
//!
//! **Decoder** (`subghz_protocol_decoder_treadmill37_feed`): Reset waits for a LOW with
//! |d − 6000| < 600 (4·te_delta); then HIGH/LOW pairs within te_delta (short + long = 0, long +
//! short = 1). A gap-window LOW closes the frame: the last bit is read from its HIGH alone (≈300 → 0,
//! ≈900 → 1, two independent tests as in the C), and the frame is reported when exactly 37 bits were
//! read. The decoder then returns to Reset, so the gap that closed a frame does not open the next one:
//! a held button reports every other frame. No checksum.
//!
//! **Fields** (`subghz_protocol_treadmill37_check_remote_controller`): serial = bits 36..17
//! (`data >> 17`, 20 bits), a 16-bit command at bits 16..1 that the C keeps in `generic.cnt` and
//! displays as `Btn: %04lX`, and a 0 stop bit. Its six key samples' command fields (0x0C18, 0x030F,
//! 0x0A16, 0x0612, 0xAAB6, 0x99A5) all have low byte = high byte + 0x0C; the C checks nothing. KAT:
//! `serial` as the C, `counter` = the 16-bit command (`generic.cnt`; KAT's `button` is 8 bits),
//! `button` None, `crc_valid` false (no check).
//!
//! **Encoder** (`subghz_protocol_encoder_treadmill37_get_upload`, `encoder.repeat` = 3): the 37 bits
//! as pulses, the last LOW being the 6000 µs gap; the captured code as stored (no button to switch:
//! `button` is ignored).
//!
//! **Key file**: `Protocol: Treadmill37`, `Bit: 37`, `Key` (`subghz_block_generic_serialize`;
//! doctor's serializer also writes `Serial` / `Btn` / `Cnt`, which KAT does not); loading requires
//! `Bit: 37`.

use super::f_common::{add_bit, push, with_repeats};
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (see `info.rs`): `SubGhzProtocolFlag_433 | AM`, `SubGhzProtocolTypeStatic`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Treadmill (QH-433)",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

pub(super) const NAME: &str = "Treadmill37";
const TE_SHORT: u32 = 300;
const TE_LONG: u32 = 900;
const TE_DELTA: u32 = 150;
const BIT_COUNT: usize = 37;
/// The gap after the last bit: 20·te_short; the decoder's window is ± 4·te_delta.
const GAP: u32 = TE_SHORT * 20;
const GAP_DELTA: u32 = TE_DELTA * 4;
/// `subghz_protocol_encoder_common_alloc(…, 3, 128)`.
const REPEAT: usize = 3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    SaveDuration,
    CheckDuration,
}

/// Treadmill37 decoder (`SubGhzProtocolDecoderTreadmill37`).
pub struct Treadmill37Decoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
}

fn is_short(d: u32) -> bool {
    d.abs_diff(TE_SHORT) < TE_DELTA
}

fn is_long(d: u32) -> bool {
    d.abs_diff(TE_LONG) < TE_DELTA
}

/// A decode (or a loaded key) of `data`, with the C's serial and command (`generic.cnt`).
fn signal(data: u64) -> DecodedSignal {
    DecodedSignal {
        serial: Some((data >> 17) as u32),
        button: None,
        counter: Some((data >> 1) as u16),
        crc_valid: false,
        data,
        data_count_bit: BIT_COUNT,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

/// One `get_upload`: the 37 bits, the gap after the last.
fn upload(data: u64) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(2 * BIT_COUNT);
    for i in (1..=BIT_COUNT).rev() {
        let one = (data >> (i - 1)) & 1 == 1;
        push(&mut out, true, if one { TE_LONG } else { TE_SHORT });
        push(
            &mut out,
            false,
            if i == 1 {
                GAP
            } else if one {
                TE_SHORT
            } else {
                TE_LONG
            },
        );
    }
    out
}

impl Treadmill37Decoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
        }
    }
}

impl Default for Treadmill37Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for Treadmill37Decoder {
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

    /// `subghz_protocol_decoder_common_reset`.
    fn reset(&mut self) {
        self.step = Step::Reset;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                if !level && duration.abs_diff(GAP) < GAP_DELTA {
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
                if is_short(self.te_last) && is_long(duration) {
                    add_bit(&mut self.decode_data, &mut self.decode_count_bit, false);
                    self.step = Step::SaveDuration;
                } else if is_long(self.te_last) && is_short(duration) {
                    add_bit(&mut self.decode_data, &mut self.decode_count_bit, true);
                    self.step = Step::SaveDuration;
                } else if duration.abs_diff(GAP) < GAP_DELTA {
                    // End of the key: the last bit is read from its HIGH alone.
                    if is_short(self.te_last) {
                        add_bit(&mut self.decode_data, &mut self.decode_count_bit, false);
                    }
                    if is_long(self.te_last) {
                        add_bit(&mut self.decode_data, &mut self.decode_count_bit, true);
                    }
                    let out = (usize::from(self.decode_count_bit) == BIT_COUNT).then(|| signal(self.decode_data));
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
        (decoded.data_count_bit == BIT_COUNT).then(|| with_repeats(&upload(decoded.data), REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        (key.bits == BIT_COUNT).then(|| signal(key.key))
    }

    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        (decoded.data_count_bit == BIT_COUNT)
            .then(|| FlipperKey::for_protocol(NAME, BIT_COUNT, decoded.data))
    }
}

#[cfg(test)]
mod tests {
    use super::super::f_common::test_support::*;
    use super::*;

    /// The C's key samples (serial 0xC0000, six commands).
    const SAMPLES: [u64; 6] = [0x18_0000_1830, 0x18_0000_061E, 0x18_0000_142C, 0x18_0000_0C24, 0x18_0001_556C, 0x18_0001_334A];

    fn on_air(data: u64, copies: usize) -> Vec<LevelDuration> {
        let mut wire = vec![LevelDuration::new(false, GAP)];
        for p in with_repeats(&upload(data), copies) {
            push(&mut wire, p.level, p.duration_us);
        }
        wire
    }

    #[test]
    fn fields_of_the_c_samples() {
        let want = [0x0C18u16, 0x030F, 0x0A16, 0x0612, 0xAAB6, 0x99A5];
        for (data, cnt) in SAMPLES.into_iter().zip(want) {
            let got = feed_all(&mut Treadmill37Decoder::new(), &on_air(data, 1));
            assert_eq!(got.len(), 1, "{data:010X}");
            let d = &got[0];
            assert_eq!((d.data, d.data_count_bit, d.serial, d.counter), (data, 37, Some(0xC0000), Some(cnt)));
            assert!(!d.crc_valid && d.button.is_none() && d.encoder_capable);
        }
    }

    #[test]
    fn held_button_reports_every_other_frame() {
        // The gap that closes a frame does not open the next: copies 1 and 3 of 4.
        let got = feed_all(&mut Treadmill37Decoder::new(), &on_air(SAMPLES[4], 4));
        assert_eq!(got.len(), 2);
        assert!(got.iter().all(|d| d.data == SAMPLES[4]));
    }

    #[test]
    fn upload_matches_the_c() {
        let up = Treadmill37Decoder::new().encode(&signal(SAMPLES[0]), 0).unwrap();
        assert_eq!(up.len(), 3 * 2 * BIT_COUNT, "alternating levels, nothing merged");
        let p = |i: usize| (up[i].level, up[i].duration_us);
        // 0x1800001830 as 37 bits: 1 1 0 0 0 …
        assert_eq!([p(0), p(1), p(2), p(3), p(4), p(5)], [(true, 900), (false, 300), (true, 900), (false, 300), (true, 300), (false, 900)]);
        // The stop bit (0) and the gap.
        assert_eq!([p(72), p(73)], [(true, 300), (false, 6000)]);
    }

    #[test]
    fn rejects_wrong_length_and_timing() {
        let data = SAMPLES[1];
        assert_eq!(feed_all(&mut Treadmill37Decoder::new(), &on_air(data, 1)).len(), 1);
        let mut short = on_air(data, 1);
        short.drain(1..3);
        assert!(feed_all(&mut Treadmill37Decoder::new(), &short).is_empty(), "36 bits");
        let mut wide = on_air(data, 1);
        wide[1].duration_us = 1050;
        assert!(feed_all(&mut Treadmill37Decoder::new(), &wide).is_empty(), "HIGH 1050 is outside 900 ± 150");
        // Gap window 5401..=6599 µs, at both ends.
        for (g, ok) in [(5400u32, false), (5401, true), (6599, true), (6600, false)] {
            let mut w = on_air(data, 1);
            w[0].duration_us = g;
            assert_eq!(feed_all(&mut Treadmill37Decoder::new(), &w).len(), usize::from(ok), "header {g}");
            let mut w = on_air(data, 1);
            w.last_mut().unwrap().duration_us = g;
            assert_eq!(feed_all(&mut Treadmill37Decoder::new(), &w).len(), usize::from(ok), "end gap {g}");
        }
        assert!(Treadmill37Decoder::new().encode(&DecodedSignal::new(data, 36), 0).is_none());
    }

    #[test]
    fn key_file_round_trip() {
        let dec = Treadmill37Decoder::new();
        let text = super::super::flipper_key::testing::assert_round_trip(&dec, &signal(SAMPLES[5]));
        assert!(text.contains("Protocol: Treadmill37\nBit: 37\nKey: 00 00 00 18 00 01 33 4A\n"), "{text}");
        let mut wrong = FlipperKey::parse(&text).unwrap();
        wrong.bits = 36;
        assert!(dec.from_flipper_key(&wrong).is_none());
    }

    /// The upload through KAT's whole registry: only Treadmill37 reads it (copy 1 has no gap before it,
    /// copy 2 decodes, and its decode takes the gap copy 3 would need).
    #[test]
    fn upload_decodes_back_through_the_registry() {
        for data in SAMPLES {
            let tx = Treadmill37Decoder::new().encode(&signal(data), 0).unwrap();
            let mut reg = super::super::ProtocolRegistry::new();
            let hits = reg.process_signal_stream(&tx, 433_920_000);
            assert_eq!(hits.len(), 1, "{hits:?}");
            assert!(hits.iter().all(|(n, x, _)| n == NAME && x.data == data), "{hits:?}");
        }
    }

    #[test]
    fn jittered_frames_decode() {
        let mut rng = Rng(0x7EAD_3711);
        for _ in 0..50 {
            let data = rng.next() & 0x1F_FFFF_FFFE;
            let wire = jitter(&on_air(data, 3), &mut rng, 100);
            let got = feed_all(&mut Treadmill37Decoder::new(), &wire);
            assert!(!got.is_empty(), "{data:010X}");
            assert!(got.iter().all(|d| d.data == data));
        }
    }
}
