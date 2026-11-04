//! Princeton (PT2262 / PT2260 / EV1527-style 24-bit fixed code) decoder + encoder.
//!
//! Port of Flipper-ARF's garage app `applications/system/subghz_garage/protocols/princeton.c`
//! (the primary reference; its lib copy `lib/subghz/protocols/princeton.c` decodes identically).
//! doctor-firmware's copy differs on the wire in one place: it also drops "degenerate" keys
//! (`subghz_protocol_blocks_key_is_degenerate`: all-0/all-1/0xAA../0x55.. or a run of 4 equal
//! nibbles). KAT follows ARF and keeps them (they are valid codes, e.g. `0x0000F1`).
//!
//! **Air format** (AM): PWM with te ≈ 390 µs (`te_short` 390, `te_long` 1170, `te_delta` 300); bit 1
//! = HIGH 3·te + LOW te, bit 0 = HIGH te + LOW 3·te, MSB first; a stop HIGH te, then a guard LOW of
//! `guard_time`·te (default 30, valid 15..=72). 24 data bits.
//!
//! **Decoder** (`subghz_protocol_decoder_princeton_feed`): a LOW within 14040 ± 10800 µs starts a
//! frame; bit 0 = HIGH short (|d − 390| < 300) + LOW long (|d − 1170| < 900), bit 1 = HIGH long +
//! LOW short; a LOW ≥ 2340 µs ends it. A 24-bit frame is reported only when it equals the previous
//! one (`last_data`, non-zero) — the C's false-positive defence; so the second identical frame
//! decodes. TE is estimated as (sum of every HIGH/LOW of the frame, stop bit included) / 97 and the
//! guard time as round(guard LOW / TE) (outside 15..=72 → 30).
//!
//! **Fields** (`subghz_protocol_princeton_check_remote_controller`): low byte 0x30/0xC0 → 16-bit
//! serial `data >> 8` + 8-bit button; low byte 0x03/0x0C → serial `data >> 8`, button `| 0xF0`
//! (0xF3/0xFC); otherwise 20-bit serial `data >> 4` + 4-bit button. `counter` is unused; `extra`
//! carries the TE (low 32 bits) and the guard time (bits 32..63) the encoder needs.
//!
//! **Encoder** (`subghz_protocol_encoder_princeton_get_upload`, `encoder.repeat` = 3): the key is
//! rebuilt from serial + button (the C's custom-button path: `button` = the native code to send —
//! a 4-bit code for a 4-bit remote, one of 0x30/0xC0/0xF3/0xFC for the 8-bit type; anything else
//! keeps the captured button), then 24 bits, stop HIGH te and guard LOW te·guard_time, sent 3
//! times. Needs the TE (a key file without `TE` does not load, as in the C).
//!
//! **Key file** (`serialize` / `deserialize`): `Protocol: Princeton`, `Bit: 24`, `Key`, `TE`,
//! `Guard_time` (optional on load; out of 15..=72 → 30).

use super::g1_common::{generic_fields, generic_key, push, u32_line, with_repeats, TX_REPEAT};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Princeton",
    decoder: "Princeton",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Generic (PT2262)",
    category: Category::Gate,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// Princeton buttons as the C displays them: a 4-bit number, or one of the 8-bit codes.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x30 => "Btn 30",
        0xC0 => "Btn C0",
        0xF3 => "Btn F3",
        0xFC => "Btn FC",
        _ => super::g1_common::numbered_button(code),
    })
}

const NAME: &str = "Princeton";
const TE_SHORT: u32 = 390;
const TE_LONG: u32 = 1170;
const TE_DELTA: u32 = 300;
const MIN_COUNT_BIT: usize = 24;
/// `PRINCETON_GUARD_TIME_DEFALUT` (guard = 30·te).
const GUARD_TIME_DEFAULT: u32 = 30;

/// The 8-bit button codes of the "second encoding type".
const BTN_8BIT: [u8; 4] = [0x30, 0xC0, 0xF3, 0xFC];

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    SaveDuration,
    CheckDuration,
}

/// Princeton decoder (`SubGhzProtocolDecoderPrinceton`).
pub struct PrincetonDecoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    /// `SubGhzBlockDecoder.decode_count_bit` is a `uint8_t`.
    decode_count_bit: u8,
    te: u32,
    last_data: u32,
    guard_time: u32,
    /// The last `feed` reported a frame: the app's reset already ran inside the callback (see
    /// `g1_common`), so the registry's reset that follows must keep the post-callback state.
    just_emitted: bool,
}

/// `subghz_protocol_princeton_check_remote_controller`: (serial, button).
fn check_remote_controller(data: u64) -> (u32, u8) {
    match (data & 0xFF) as u8 {
        b @ (0x30 | 0xC0) => ((data >> 8) as u32, b),
        b @ (0x03 | 0x0C) => ((data >> 8) as u32, b | 0xF0),
        _ => ((data >> 4) as u32, (data & 0xF) as u8),
    }
}

/// The key rebuilt from serial and button (`get_upload`'s reconstruction).
fn build_key(serial: u32, btn: u8) -> u64 {
    match btn {
        0x30 | 0xC0 => (serial as u64) << 8 | btn as u64,
        0xF3 | 0xFC => (serial as u64) << 8 | (btn & 0xF) as u64,
        _ => (serial as u64) << 4 | btn as u64,
    }
}

/// Guard time as the decoder estimates it / the key file gives it (15..=72, else 30).
fn sanitize_guard_time(gt: u32) -> u32 {
    if (15..=72).contains(&gt) {
        gt
    } else {
        GUARD_TIME_DEFAULT
    }
}

fn pack_extra(te: u32, guard_time: u32) -> u64 {
    te as u64 | (guard_time as u64) << 32
}

/// The decode for a key, with its TE and guard time.
fn decoded(data: u64, te: u32, guard_time: u32) -> DecodedSignal {
    let (serial, btn) = check_remote_controller(data);
    DecodedSignal {
        serial: Some(serial),
        button: Some(btn),
        counter: None,
        crc_valid: false,
        data,
        data_count_bit: MIN_COUNT_BIT,
        encoder_capable: te != 0,
        extra: Some(pack_extra(te, guard_time)),
        protocol_display_name: None,
    }
}

/// The button to send: `requested` when it is a code of the captured button's type.
fn pick_button(captured: u8, requested: u8) -> u8 {
    let eight_bit = BTN_8BIT.contains(&captured);
    if (eight_bit && BTN_8BIT.contains(&requested)) || (!eight_bit && requested <= 0xF) {
        requested
    } else {
        captured
    }
}

/// One `get_upload` frame: 24 bits, stop bit, guard.
fn build_frame(data: u64, te: u32, guard_time: u32) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(MIN_COUNT_BIT * 2 + 2);
    for i in (0..MIN_COUNT_BIT).rev() {
        if (data >> i) & 1 == 1 {
            push(&mut out, true, te.wrapping_mul(3));
            push(&mut out, false, te);
        } else {
            push(&mut out, true, te);
            push(&mut out, false, te.wrapping_mul(3));
        }
    }
    push(&mut out, true, te);
    push(&mut out, false, te.wrapping_mul(guard_time));
    out
}

impl PrincetonDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
            te: 0,
            last_data: 0,
            guard_time: GUARD_TIME_DEFAULT,
            just_emitted: false,
        }
    }

    fn add_bit(&mut self, bit: u64) {
        self.decode_data = self.decode_data << 1 | bit;
        self.decode_count_bit = self.decode_count_bit.wrapping_add(1);
    }
}

impl Default for PrincetonDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for PrincetonDecoder {
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
        if std::mem::take(&mut self.just_emitted) {
            return;
        }
        // subghz_protocol_decoder_princeton_reset
        self.step = Step::Reset;
        self.last_data = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        self.just_emitted = false;
        match self.step {
            Step::Reset => {
                if !level && duration_diff!(duration, TE_SHORT * 36) < TE_DELTA * 36 {
                    self.step = Step::SaveDuration;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.te = 0;
                    self.guard_time = GUARD_TIME_DEFAULT;
                }
            }
            Step::SaveDuration => {
                if level {
                    self.te_last = duration;
                    self.te = self.te.wrapping_add(duration);
                    self.step = Step::CheckDuration;
                }
            }
            Step::CheckDuration => {
                if level {
                    self.step = Step::Reset;
                    return None;
                }
                if duration >= TE_LONG * 2 {
                    self.step = Step::SaveDuration;
                    let mut out = None;
                    if self.decode_count_bit as usize == MIN_COUNT_BIT {
                        if self.last_data as u64 == self.decode_data && self.last_data != 0 {
                            self.te /= self.decode_count_bit as u32 * 4 + 1;
                            let gt = if self.te == 0 {
                                GUARD_TIME_DEFAULT
                            } else {
                                // roundf((float)duration / te), then the 15..=72 check.
                                let q = (duration as f32 / self.te as f32).round();
                                q as u32
                            };
                            self.guard_time = sanitize_guard_time(gt);
                            out = Some(decoded(self.decode_data, self.te, self.guard_time));
                            // callback → the app resets the receiver (princeton_reset).
                            self.step = Step::Reset;
                        }
                        self.last_data = self.decode_data as u32;
                    }
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.te = 0;
                    self.just_emitted = out.is_some();
                    return out;
                }
                self.te = self.te.wrapping_add(duration);
                if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA
                    && duration_diff!(duration, TE_LONG) < TE_DELTA * 3
                {
                    self.add_bit(0);
                    self.step = Step::SaveDuration;
                } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA * 3
                    && duration_diff!(duration, TE_SHORT) < TE_DELTA
                {
                    self.add_bit(1);
                    self.step = Step::SaveDuration;
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
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        let extra = decoded.extra?;
        let te = extra as u32;
        if te == 0 {
            return None;
        }
        let guard_time = sanitize_guard_time((extra >> 32) as u32);
        let (serial, captured) = check_remote_controller(decoded.data);
        let data = build_key(serial, pick_button(captured, button));
        Some(with_repeats(&build_frame(data, te, guard_time), TX_REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        let (bits, data) = generic_fields(key);
        if bits != MIN_COUNT_BIT {
            return None;
        }
        let te = u32_line(key, "TE")?;
        let guard_time =
            u32_line(key, "Guard_time").map_or(GUARD_TIME_DEFAULT, sanitize_guard_time);
        Some(decoded(data, te, guard_time))
    }

    fn to_flipper_key(&self, d: &DecodedSignal) -> Option<FlipperKey> {
        let extra = d.extra?;
        let mut key = generic_key(NAME, d);
        key.push("TE", (extra as u32).to_string());
        key.push("Guard_time", ((extra >> 32) as u32).to_string());
        Some(key)
    }
}

#[cfg(test)]
mod tests {
    use super::super::g1_common::test_support::*;
    use super::*;

    fn frame(data: u64) -> DecodedSignal {
        decoded(data, 400, 30)
    }

    #[test]
    fn fields_follow_check_remote_controller() {
        assert_eq!(check_remote_controller(0x95D5D4), (0x95D5D, 0x4));
        assert_eq!(check_remote_controller(0x1234C0), (0x1234, 0xC0));
        assert_eq!(check_remote_controller(0x123430), (0x1234, 0x30));
        assert_eq!(check_remote_controller(0x123403), (0x1234, 0xF3));
        assert_eq!(check_remote_controller(0x12340C), (0x1234, 0xFC));
        for (serial, btn) in [
            (0x95D5D, 0x4),
            (0x1234, 0xC0),
            (0x1234, 0xF3),
            (0x1234, 0xFC),
        ] {
            assert_eq!(
                check_remote_controller(build_key(serial, btn)),
                (serial, btn)
            );
        }
        assert_eq!(native_buttons(NAME, 0x4), Some("Btn 4"));
        assert_eq!(native_buttons(NAME, 0xF3), Some("Btn F3"));
        assert_eq!(native_buttons(NAME, 0x55), Some("Other"));
        assert_eq!(
            crate::protocols::native_button_name("Princeton", 0x8),
            Some("Btn 8")
        );
    }

    /// Encoder → decoder: of the 3 repeats, the first only primes the decoder (its guard is the
    /// start LOW), the second sets `last_data` and the third, identical, decodes.
    #[test]
    fn encode_decode_roundtrip() {
        let d = frame(0x95D5D4);
        let wire = PrincetonDecoder::new().encode(&d, 0x4).unwrap();
        assert_eq!(wire.len(), 3 * (24 * 2 + 2));
        let got = feed_all(&mut PrincetonDecoder::new(), &wire);
        assert_eq!(got.len(), 1);
        assert_eq!(
            (got[0].data, got[0].serial, got[0].button),
            (0x95D5D4, Some(0x95D5D), Some(0x4))
        );
        // TE estimated back from the wire; guard = 30·te.
        assert_eq!(got[0].extra, Some(pack_extra(400, 30)));
        assert!(got[0].encoder_capable && !got[0].crc_valid);
    }

    /// Button switching: a 4-bit code for a 4-bit remote, an 8-bit code for the second type.
    #[test]
    fn encode_switches_buttons_within_the_type() {
        let cases = [
            (0x95D5D4u64, 0x8u8, 0x95D5D8u64),
            (0x95D5D4, 0x30, 0x95D5D4), // 8-bit code on a 4-bit remote: keep the button
            (0x1234C0, 0x30, 0x123430),
            (0x1234C0, 0xF3, 0x123403),
            (0x1234C0, 0x2, 0x1234C0), // 4-bit code on an 8-bit remote: keep the button
        ];
        for (data, button, want) in cases {
            let wire = PrincetonDecoder::new()
                .encode(&frame(data), button)
                .unwrap();
            let got = feed_all(&mut PrincetonDecoder::new(), &wire);
            assert_eq!(got[0].data, want, "{data:06X} btn {button:02X}");
        }
    }

    /// Guard time and TE travel through the decode (estimated, then re-sent).
    #[test]
    fn guard_time_is_estimated_and_reused() {
        let d = decoded(0xA5A5A1, 350, 40);
        let wire = PrincetonDecoder::new().encode(&d, 0x1).unwrap();
        let got = feed_all(&mut PrincetonDecoder::new(), &wire);
        assert_eq!(got[0].extra, Some(pack_extra(350, 40)));
    }

    #[test]
    fn rejects_single_frames_and_bad_timing() {
        let one = build_frame(0x95D5D4, 400, 30);
        let mut wire = vec![LevelDuration::new(false, 12000)];
        wire.extend_from_slice(&one);
        assert!(
            feed_all(&mut PrincetonDecoder::new(), &wire).is_empty(),
            "one frame is not enough"
        );
        // Two different frames do not confirm each other.
        wire.extend_from_slice(&build_frame(0x95D5D8, 400, 30));
        assert!(feed_all(&mut PrincetonDecoder::new(), &wire).is_empty());
        // Too-long HIGH pulses (≥ 1170 + 900) break the frame.
        let bad: Vec<LevelDuration> = with_repeats(&one, 3)
            .iter()
            .map(|p| {
                LevelDuration::new(
                    p.level,
                    if p.level && p.duration_us > 1000 {
                        2200
                    } else {
                        p.duration_us
                    },
                )
            })
            .collect();
        assert!(feed_all(&mut PrincetonDecoder::new(), &bad).is_empty());
        // An all-zero key never confirms (last_data must be non-zero).
        let zeros = with_repeats(&build_frame(0, 400, 30), 3);
        assert!(feed_all(&mut PrincetonDecoder::new(), &zeros).is_empty());
    }

    #[test]
    fn jittered_frames_still_decode() {
        let wire = with_repeats(&build_frame(0x3C5A0F, 390, 30), 3);
        let got = feed_all(&mut PrincetonDecoder::new(), &jitter(&wire, 120));
        assert_eq!(got.iter().map(|d| d.data).collect::<Vec<_>>(), [0x3C5A0F]);
    }

    /// The unit-test key file loads, encodes and the encoder output decodes back.
    #[test]
    fn unit_test_key_file() {
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                    Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Princeton\nBit: 24\n\
                    Key: 00 00 00 00 00 95 D5 D4\nTE: 400\n";
        let key = FlipperKey::parse(text).unwrap();
        let dec = PrincetonDecoder::new();
        let d = dec.from_flipper_key(&key).unwrap();
        assert_eq!(
            (d.data, d.serial, d.button, d.extra),
            (0x95D5D4, Some(0x95D5D), Some(4), Some(pack_extra(400, 30)))
        );
        let got = feed_all(
            &mut PrincetonDecoder::new(),
            &dec.encode(&d, d.button.unwrap()).unwrap(),
        );
        assert_eq!(got[0].data, 0x95D5D4);
        // Round trip through the key file KAT writes (TE + Guard_time lines).
        let written = dec.to_flipper_key(&d).unwrap();
        assert!(written
            .to_text()
            .ends_with("Key: 00 00 00 00 00 95 D5 D4\nTE: 400\nGuard_time: 30\n"));
        let back = dec
            .from_flipper_key(&FlipperKey::parse(&written.to_text()).unwrap())
            .unwrap();
        assert_eq!(format!("{back:?}"), format!("{d:?}"));
        // No TE → not loadable; wrong bit count → not loadable.
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&text.replace("TE: 400\n", "")).unwrap())
            .is_none());
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&text.replace("Bit: 24", "Bit: 25")).unwrap())
            .is_none());
        // Guard_time out of range → 30.
        let gt = dec
            .from_flipper_key(&FlipperKey::parse(&format!("{text}Guard_time: 90\n")).unwrap())
            .unwrap();
        assert_eq!(gt.extra, Some(pack_extra(400, 30)));
    }

    /// Flipper-ARF's `princeton_raw.sub` (the unit test's capture) decodes through the registry.
    #[test]
    fn reference_raw_capture_decodes() {
        let Some(decodes) = decode_reference_capture("princeton_raw.sub") else {
            return;
        };
        let ours: Vec<_> = decodes.iter().filter(|(n, _)| n == NAME).collect();
        assert!(
            !ours.is_empty(),
            "{:?}",
            decodes.iter().map(|(n, _)| n).collect::<Vec<_>>()
        );
    }
}
