//! Somfy Keytis — Somfy RTS garage/gate remotes (Keytis): the Somfy Telis frame followed by a
//! 24-bit "press duration counter", 80 bits on the air.
//!
//! Port of `somfy_keytis.c` from Flipper-ARF's garage app
//! (`applications/system/subghz_garage/protocols/`, the primary reference). Flipper-ARF lib's and
//! doctor-firmware's copies decode and transmit the same frames; their encoders add a D-pad swap
//! of buttons 0x4 ↔ 0x3, which KAT's native-button parameter covers.
//!
//! **Air format** (AM/OOK): as Somfy Telis (see `somfy_telis.rs`): wake-up, 2560 µs hardware sync
//! pairs (12 in the first frame, 6 in repeats), software sync HIGH 4550 + LOW 640, Manchester
//! te_short 640 / te_long 1280 µs (te_delta 250), then 56 frame bits and 24 press-duration bits,
//! then ≥ 1530 µs LOW.
//!
//! **Decoder** (`subghz_protocol_decoder_somfy_keytis_feed`): Somfy Telis's state machine, except
//! that `header_count` is not cleared when the data starts (it only drops on a broken sync pair),
//! bits 0–55 go to the frame and bits 56+ to the press-duration counter, and a frame needs exactly
//! 80 bits. Its checksum test reads `generic.data` — the previously reported frame (0 at start) —
//! instead of the frame just received (somfy_telis.c checks `decoder.decode_data`): the C reports
//! any 80-bit frame after a valid one, and after reporting a corrupt frame it reports nothing more
//! until the app restarts. KAT checks the frame being decoded, so it reports the same frames on
//! streams of intact frames (every capture in the corpus, oracle-checked) and never a corrupt one;
//! see docs/somfy_keytis.md.
//!
//! **Frame / fields** (`check_remote_controller`): `plain = data ^ data >> 8` of the 56-bit frame;
//! `button = plain >> 48 & 0xF` (the key byte's low nibble: 3 Prog, 4 Key_1), `counter` = 16-bit
//! rolling code (`plain >> 24`), `serial` = address (`plain & 0xFFFFFF`). The 24-bit press
//! duration counter (`11 cnt4 00 0000 cnt8 crc4`, 0xC40019 on the first parcel of a press) is
//! display-only. **Packing**: `data` = the 56-bit frame (the references' `Key`), `extra` = the press
//! duration counter (their `Duration_Counter`), `data_count_bit` = 80.
//!
//! **Encoder** (`_gen_data` + `_get_upload`): fields from the frame; `button` = native code 0x0–0xF
//! (anything else keeps the captured one); rolling code + 1 (0xFFFF → 0); the frame is rebuilt as
//! `0xA:button | 0xF:crc | counter | address` (key nibble 0xA and ctrl nibble 0xF forced, as the C
//! does), the press duration counter is always 0xC40019. Upload: wake-up HIGH 9415 + LOW 89565,
//! 12 sync pairs, software sync, 56 + 24 bits, LOW 1920; twice: 6 sync pairs, software sync, bits,
//! LOW 1920; the last LOW is stretched by 30415 − 1920; `encoder.repeat` = 3 copies. (The garage
//! app's TX plugins are built without `SUBGHZ_GARAGE_HAS_LIB_EXTENSIONS`, which would put the
//! counter in "OFEX" mode; KAT follows the source's normal mode.)
//!
//! **Key file** (`serialize`): `Protocol: Somfy Keytis`, `Bit: 80`, `Key` (56-bit frame),
//! `Duration_Counter` (decimal); loading requires `Bit: 80` and `Duration_Counter`.

use super::common::{ManchesterEvent, ManchesterState};
use super::flipper_key::FlipperKey;
use super::g4_common::{
    push, somfy_crc_ok, somfy_obfuscate, somfy_push_bits, somfy_push_silence, SOMFY_SILENCE,
    SOMFY_SOFT_SYNC, SOMFY_TE_DELTA, SOMFY_TE_LONG, SOMFY_TE_SHORT, SOMFY_WAKEUP_HIGH,
    SOMFY_WAKEUP_LOW,
};
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`). Flags: 433 / 868 MHz, AM.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Somfy Keytis",
    decoder: "Somfy Keytis",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "Somfy",
    category: Category::Gate,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// `subghz_protocol_somfy_keytis_get_name_button`.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    const NAMES: [&str; 16] = [
        "Unknown", "0x01", "0x02", "Prog", "Key_1", "0x05", "0x06", "0x07", "0x08", "0x09", "0x0A",
        "0x0B", "0x0C", "0x0D", "0x0E", "0x0F",
    ];
    Some(NAMES.get(code as usize).copied().unwrap_or(NAMES[0]))
}

const NAME: &str = "Somfy Keytis";
const BIT_COUNT: u8 = 80;
/// Bits of the Telis-format frame; the rest is the press duration counter.
const FRAME_BITS: u8 = 56;
const PDC_BITS: u32 = 24;
/// The press duration counter the encoder sends (first parcel of a press).
const PDC_FIRST: u64 = 0xC4_0019;
/// Gap after a frame inside the upload (te_short × 3).
const FRAME_GAP: u32 = SOMFY_TE_SHORT * 3;
const TX_REPEAT: usize = 3;

/// `SomfyKeytisDecoderStep` (`StartDecode` is never entered).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    CheckPreambula,
    FoundPreambula,
    DecoderData,
}

/// Somfy Keytis decoder (`SubGhzProtocolDecoderSomfyKeytis`).
pub struct SomfyKeytisDecoder {
    step: Step,
    header_count: u16,
    manchester: ManchesterState,
    decode_data: u64,
    /// `SubGhzBlockDecoder.decode_count_bit` is a `uint8_t`.
    decode_count_bit: u8,
    press_duration_counter: u32,
}

impl SomfyKeytisDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            header_count: 0,
            manchester: ManchesterState::Start1,
            decode_data: 0,
            decode_count_bit: 0,
            press_duration_counter: 0,
        }
    }

    /// Manchester Reset + LongHigh (the C's two `manchester_advance` calls).
    fn prime_manchester(&mut self) {
        self.manchester = ManchesterState::Mid1;
        self.manchester.advance(ManchesterEvent::LongHigh);
    }
}

impl Default for SomfyKeytisDecoder {
    fn default() -> Self {
        Self::new()
    }
}

fn is_sync(duration: u32) -> bool {
    duration_diff!(duration, SOMFY_TE_SHORT * 4) < SOMFY_TE_DELTA * 4
}

/// Fields of a frame (`subghz_protocol_somfy_keytis_check_remote_controller`).
fn decoded_from_data(data: u64, pdc: u32) -> DecodedSignal {
    let plain = data ^ (data >> 8);
    DecodedSignal {
        serial: Some((plain & 0xFF_FFFF) as u32),
        button: Some(((plain >> 48) & 0xF) as u8),
        counter: Some(((plain >> 24) & 0xFFFF) as u16),
        crc_valid: somfy_crc_ok(data),
        data,
        data_count_bit: BIT_COUNT as usize,
        encoder_capable: true,
        extra: Some(pdc as u64),
        protocol_display_name: None,
    }
}

/// `subghz_protocol_somfy_keytis_gen_data`: the next frame with `button`.
fn next_frame(data: u64, button: Option<u8>) -> u64 {
    let plain = data ^ (data >> 8);
    let button = button.unwrap_or(((plain >> 48) & 0xF) as u8);
    let counter = ((plain >> 24) & 0xFFFF) as u32;
    let counter = if counter + 1 > 0xFFFF { 0 } else { counter + 1 };
    let serial = (plain & 0xFF_FFFF) as u32;
    somfy_obfuscate([
        0xA0 | button,
        0xF0,
        (counter >> 8) as u8,
        counter as u8,
        (serial >> 16) as u8,
        (serial >> 8) as u8,
        serial as u8,
    ])
}

/// doctor-firmware `subghz_protocol_somfy_keytis_create_data` (the SubGHz app's "Add Manually"):
/// `gen_data` — key nibble 0xA with `btn`, ctrl 0xF, counter + 1 (0 past 0xFFFF), the 24-bit
/// address, the 0xC40019 tail as `Duration_Counter`; `Bit: 80`, `Key`, `Duration_Counter`.
pub(super) fn create_data(serial: u32, btn: u8, cnt: u16) -> FlipperKey {
    let counter = cnt.checked_add(1).unwrap_or(0);
    let data = somfy_obfuscate([
        0xA0 | btn,
        0xF0,
        (counter >> 8) as u8,
        counter as u8,
        (serial >> 16) as u8,
        (serial >> 8) as u8,
        serial as u8,
    ]);
    let mut key = FlipperKey::for_protocol(NAME, BIT_COUNT as usize, data);
    key.push("Duration_Counter", 0xC4_0019u32.to_string());
    key
}

/// One frame of the upload after its hardware sync pairs, ending in the 1920 µs gap.
fn push_frame(out: &mut Vec<LevelDuration>, data: u64, sync_pairs: usize) {
    for _ in 0..sync_pairs {
        push(out, true, SOMFY_TE_SHORT * 4);
        push(out, false, SOMFY_TE_SHORT * 4);
    }
    push(out, true, SOMFY_SOFT_SYNC);
    push(out, false, SOMFY_TE_SHORT);
    somfy_push_bits(out, data, FRAME_BITS as u32);
    somfy_push_bits(out, PDC_FIRST, PDC_BITS);
    somfy_push_silence(out, FRAME_GAP);
}

/// `subghz_protocol_encoder_somfy_keytis_get_upload` (one copy).
fn push_upload(out: &mut Vec<LevelDuration>, data: u64) {
    push(out, true, SOMFY_WAKEUP_HIGH);
    push(out, false, SOMFY_WAKEUP_LOW);
    push_frame(out, data, 12);
    for _ in 0..2 {
        push_frame(out, data, 6);
    }
    // The final gap becomes the inter-frame silence.
    somfy_push_silence(out, SOMFY_SILENCE - FRAME_GAP);
}

impl ProtocolDecoder for SomfyKeytisDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: SOMFY_TE_SHORT,
            te_long: SOMFY_TE_LONG,
            te_delta: SOMFY_TE_DELTA,
            min_count_bit: BIT_COUNT as usize,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    /// `subghz_protocol_decoder_somfy_keytis_reset`: step and Manchester state only.
    fn reset(&mut self) {
        self.step = Step::Reset;
        self.manchester = ManchesterState::Mid1;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                if level && is_sync(duration) {
                    self.step = Step::FoundPreambula;
                    self.header_count = self.header_count.wrapping_add(1);
                }
            }
            Step::FoundPreambula => {
                if !level && is_sync(duration) {
                    self.step = Step::CheckPreambula;
                } else {
                    self.header_count = 0;
                    self.step = Step::Reset;
                }
            }
            Step::CheckPreambula => {
                if level {
                    if is_sync(duration) {
                        self.step = Step::FoundPreambula;
                        self.header_count = self.header_count.wrapping_add(1);
                    } else if self.header_count > 1
                        && duration_diff!(duration, SOMFY_TE_SHORT * 7) < SOMFY_TE_DELTA * 4
                    {
                        // Unlike Telis, header_count is not cleared here.
                        self.step = Step::DecoderData;
                        self.decode_data = 0;
                        self.decode_count_bit = 0;
                        self.press_duration_counter = 0;
                        self.prime_manchester();
                    }
                }
            }
            Step::DecoderData => {
                let short = duration_diff!(duration, SOMFY_TE_SHORT) < SOMFY_TE_DELTA;
                let long = duration_diff!(duration, SOMFY_TE_LONG) < SOMFY_TE_DELTA;
                if !short && !long {
                    if level || duration < SOMFY_TE_LONG + SOMFY_TE_DELTA {
                        self.step = Step::Reset;
                        return None;
                    }
                    // End of frame: exactly 80 bits; KAT checks this frame's checksum (the C reads
                    // the previously reported one, see the module doc).
                    let result = (self.decode_count_bit == BIT_COUNT
                        && somfy_crc_ok(self.decode_data))
                    .then(|| decoded_from_data(self.decode_data, self.press_duration_counter));
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.prime_manchester();
                    self.step = Step::Reset;
                    return result;
                }
                if let Some(bit) = self
                    .manchester
                    .advance(ManchesterEvent::from_half_bit(level, long))
                {
                    if self.decode_count_bit < FRAME_BITS {
                        self.decode_data = self.decode_data << 1 | bit as u64;
                    } else {
                        self.press_duration_counter = self.press_duration_counter << 1 | bit as u32;
                    }
                    self.decode_count_bit = self.decode_count_bit.wrapping_add(1);
                }
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != BIT_COUNT as usize {
            return None;
        }
        let data = next_frame(decoded.data, (button <= 0xF).then_some(button));
        let mut out = Vec::new();
        for _ in 0..TX_REPEAT {
            push_upload(&mut out, data);
        }
        Some(out)
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != BIT_COUNT as usize {
            return None;
        }
        let pdc = key.u32_field("Duration_Counter")?;
        Some(decoded_from_data(key.key, pdc))
    }

    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != BIT_COUNT as usize {
            return None;
        }
        let mut key = FlipperKey::new(
            NAME,
            BIT_COUNT as usize,
            decoded.data,
            0,
            "FuriHalSubGhzPresetOok650Async",
        );
        key.push(
            "Duration_Counter",
            (decoded.extra.unwrap_or(0) as u32).to_string(),
        );
        Some(key)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::ProtocolRegistry;

    fn feed_all(dec: &mut SomfyKeytisDecoder, wire: &[LevelDuration]) -> Vec<DecodedSignal> {
        wire.iter()
            .filter_map(|p| dec.feed(p.level, p.duration_us))
            .collect()
    }

    /// The C comment's capture: 0xA453537C4B9855 + C40019 = key A, button 4 (Key_1), ctrl F,
    /// checksum 7, rolling 0x002F, address 0x37D3CD.
    const FRAME: u64 = 0xA4_5353_7C4B_9855;

    fn one_frame(data: u64, pdc: u64, sync_pairs: usize) -> Vec<LevelDuration> {
        let mut out = Vec::new();
        for _ in 0..sync_pairs {
            push(&mut out, true, 2560);
            push(&mut out, false, 2560);
        }
        push(&mut out, true, SOMFY_SOFT_SYNC);
        push(&mut out, false, SOMFY_TE_SHORT);
        somfy_push_bits(&mut out, data, 56);
        somfy_push_bits(&mut out, pdc, 24);
        somfy_push_silence(&mut out, FRAME_GAP);
        out
    }

    #[test]
    fn decodes_the_c_comment_frame() {
        let got = feed_all(
            &mut SomfyKeytisDecoder::new(),
            &one_frame(FRAME, 0xC8_0026, 12),
        );
        assert_eq!(got.len(), 1);
        let d = &got[0];
        assert_eq!(
            (d.data, d.extra, d.data_count_bit),
            (FRAME, Some(0xC8_0026), 80)
        );
        assert_eq!(
            (d.serial, d.button, d.counter),
            (Some(0x37_D3CD), Some(4), Some(0x2F))
        );
        assert!(d.crc_valid && d.encoder_capable);
    }

    #[test]
    fn rejects_bad_checksum_and_wrong_length() {
        assert!(feed_all(
            &mut SomfyKeytisDecoder::new(),
            &one_frame(FRAME ^ 1, 0xC4_0019, 12)
        )
        .is_empty());
        // A good frame right after a corrupt one still decodes (the C would not; see module doc).
        let mut wire = one_frame(FRAME ^ 1, 0xC4_0019, 12);
        wire.extend(one_frame(FRAME, 0xC4_0019, 6));
        assert_eq!(feed_all(&mut SomfyKeytisDecoder::new(), &wire).len(), 1);
        // A Telis-length (56-bit) frame is not a Keytis frame.
        let mut telis = Vec::new();
        for _ in 0..2 {
            push(&mut telis, true, 2560);
            push(&mut telis, false, 2560);
        }
        push(&mut telis, true, SOMFY_SOFT_SYNC);
        push(&mut telis, false, SOMFY_TE_SHORT);
        somfy_push_bits(&mut telis, FRAME, 56);
        somfy_push_silence(&mut telis, SOMFY_SILENCE);
        assert!(feed_all(&mut SomfyKeytisDecoder::new(), &telis).is_empty());
    }

    /// `header_count` survives a frame: a repeat with a single sync pair still decodes, unlike
    /// Somfy Telis.
    #[test]
    fn header_count_is_kept_across_frames() {
        let mut wire = one_frame(FRAME, 0xC4_0019, 12);
        wire.extend(one_frame(FRAME, 0xC8_0026, 1));
        assert_eq!(feed_all(&mut SomfyKeytisDecoder::new(), &wire).len(), 2);
        assert!(feed_all(
            &mut SomfyKeytisDecoder::new(),
            &one_frame(FRAME, 0xC8_0026, 1)
        )
        .is_empty());
    }

    #[test]
    fn encode_round_trip_every_button() {
        let src = decoded_from_data(FRAME, 0xC8_0026);
        for button in 0u8..=0xF {
            let wire = SomfyKeytisDecoder::new().encode(&src, button).unwrap();
            let got = feed_all(&mut SomfyKeytisDecoder::new(), &wire);
            assert_eq!(got.len(), 9, "3 copies x 3 frames");
            for d in &got {
                assert_eq!(
                    (d.button, d.counter, d.serial),
                    (Some(button), Some(0x30), Some(0x37_D3CD))
                );
                assert_eq!(d.extra, Some(PDC_FIRST));
                let plain = d.data ^ (d.data >> 8);
                assert_eq!(
                    (plain >> 52, (plain >> 44) & 0xF),
                    (0xA, 0xF),
                    "key 0xA, ctrl 0xF"
                );
            }
        }
        let keep = SomfyKeytisDecoder::new().encode(&src, 0x40).unwrap();
        assert_eq!(
            feed_all(&mut SomfyKeytisDecoder::new(), &keep)[0].button,
            Some(4)
        );
        // Counter wrap.
        let wrap = decoded_from_data(somfy_obfuscate([0xA3, 0xF0, 0xFF, 0xFF, 1, 2, 3]), 0);
        let got = feed_all(
            &mut SomfyKeytisDecoder::new(),
            &SomfyKeytisDecoder::new().encode(&wrap, 3).unwrap(),
        );
        assert_eq!((got[0].counter, got[0].button), (Some(0), Some(3)));
    }

    #[test]
    fn upload_layout_matches_the_c() {
        let wire = SomfyKeytisDecoder::new()
            .encode(&decoded_from_data(FRAME, 0), 4)
            .unwrap();
        assert_eq!((wire[0].level, wire[0].duration_us), (true, 9415));
        assert_eq!((wire[1].level, wire[1].duration_us), (false, 89565));
        assert!(wire.windows(2).all(|w| w[0].level != w[1].level));
        assert_eq!(
            wire.iter()
                .filter(|p| p.level && p.duration_us == SOMFY_SOFT_SYNC)
                .count(),
            9
        );
        let last = wire.last().unwrap();
        assert!(
            !last.level
                && (last.duration_us == SOMFY_SILENCE || last.duration_us == SOMFY_SILENCE + 640)
        );
        // Sync pairs: 12, then 6, 6 per copy.
        let first_soft = wire
            .iter()
            .position(|p| p.duration_us == SOMFY_SOFT_SYNC)
            .unwrap();
        assert_eq!(first_soft, 2 + 24);
    }

    #[test]
    fn key_file_round_trip() {
        let dec = SomfyKeytisDecoder::new();
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433420000\n\
            Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Somfy Keytis\nBit: 80\n\
            Key: 00 A4 53 53 7C 4B 98 55\nDuration_Counter: 12845081\n";
        let key = FlipperKey::parse(text).unwrap();
        let d = dec.from_flipper_key(&key).unwrap();
        assert_eq!(
            (d.data, d.extra, d.button, d.crc_valid),
            (FRAME, Some(0xC4_0019), Some(4), true)
        );
        let out = dec.to_flipper_key(&d).unwrap().to_text();
        assert!(
            out.contains("Protocol: Somfy Keytis\nBit: 80\nKey: 00 A4 53 53 7C 4B 98 55\nDuration_Counter: 12845081\n"),
            "{out}"
        );
        let back = dec
            .from_flipper_key(&FlipperKey::parse(&out).unwrap())
            .unwrap();
        assert_eq!(
            (
                back.data,
                back.data_count_bit,
                back.serial,
                back.button,
                back.counter,
                back.crc_valid,
                back.encoder_capable,
                back.extra
            ),
            (
                d.data,
                d.data_count_bit,
                d.serial,
                d.button,
                d.counter,
                d.crc_valid,
                d.encoder_capable,
                d.extra
            )
        );
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&text.replace("Duration_Counter", "X")).unwrap())
            .is_none());
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&text.replace("Bit: 80", "Bit: 56")).unwrap())
            .is_none());
        assert_eq!(
            ProtocolRegistry::new().decode_key(&key).map(|(n, _)| n),
            Some(NAME.to_string())
        );
    }

    /// Flipper's unit test: `somfy_keytis_raw.sub` must decode (skipped without REFERENCES).
    #[test]
    fn decodes_flipper_unit_test_capture() {
        let path = std::path::Path::new(
            "REFERENCES/Flipper-ARF/applications/debug/unit_tests/resources/unit_tests/subghz/somfy_keytis_raw.sub",
        );
        if !path.exists() {
            eprintln!("Skip: {path:?} not found");
            return;
        }
        let (freq, raw) = crate::export::flipper::import_sub_raw(path).unwrap();
        let pairs: Vec<LevelDuration> = raw
            .iter()
            .map(|p| LevelDuration::new(p.level, p.duration_us))
            .collect();
        assert!(
            !feed_all(&mut SomfyKeytisDecoder::new(), &pairs).is_empty(),
            "no Somfy Keytis frame"
        );
        let mut reg = ProtocolRegistry::new();
        let hits = reg.process_signal_stream(&pairs, freq);
        assert!(
            hits.iter().any(|(n, _, _)| n == NAME),
            "{:?}",
            hits.iter().map(|h| &h.0).collect::<Vec<_>>()
        );
    }
}
