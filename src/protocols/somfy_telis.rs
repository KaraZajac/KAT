//! Somfy Telis — Somfy RTS (Radio Technology Somfy) remotes for roller shutters, blinds, awnings
//! and gate motors (Telis, Situo, Smoove, …; 433.42 MHz, 868 MHz in some markets): 56-bit frame,
//! XOR-chain obfuscation, 4-bit checksum, 16-bit rolling code.
//!
//! Port of `somfy_telis.c` from Flipper-ARF's garage app
//! (`applications/system/subghz_garage/protocols/`, the primary reference), which matches
//! Flipper-ARF lib's and doctor-firmware's `lib/subghz/protocols/somfy_telis.c` on the air (the
//! copies differ only in display text and build shims). Frame description in the C:
//! pushstack.wordpress.com/somfy-rts-protocol.
//!
//! **Air format** (AM/OOK): wake-up HIGH 9415 + LOW 89565 (first frame only), hardware sync pairs
//! HIGH/LOW 2560 µs (2 in the first frame, 7 in repeats), software sync HIGH 4550 + LOW 640, then
//! 56 bits Manchester at te_short 640 / te_long 1280 µs (te_delta 250, strict `<`; Flipper
//! convention 1 = LOW then HIGH), then ≥ 1530 µs LOW (30415 µs inter-frame silence).
//!
//! **Decoder** (`subghz_protocol_decoder_somfy_telis_feed`): HIGH 2560 ± 1000 → `FoundPreambula`
//! (`header_count` + 1); LOW 2560 ± 1000 → `CheckPreambula`, anything else clears the count and
//! resets; in `CheckPreambula` a HIGH 2560 ± 1000 is the next sync pair, a HIGH 4480 ± 1000 after
//! more than one pair starts the data (Manchester primed with Reset + LongHigh, count cleared);
//! other pulses leave it waiting. Data pulses feed Flipper's Manchester machine; a LOW ≥ 1530 µs
//! ends the frame: exactly 56 bits whose de-obfuscated checksum holds are reported (the C's gate).
//! `reset` (C and KAT) only rewinds the step and the Manchester state; `header_count` survives.
//!
//! **Frame**: obfuscated bytes `f[i] = p[i] ^ f[i-1]` of the plain frame
//! `p = key8 | ctrl4 crc4 | rolling16 | address24` (`data ^ data >> 8` undoes it). Checksum: XOR of
//! every nibble of the plain frame with the checksum nibble cleared. Fields
//! (`check_remote_controller`): `button` = ctrl (`plain >> 44 & 0xF`; 1 My, 2 Up, 4 Down, 8 Prog,
//! …), `counter` = rolling code, `serial` = address (as stored, the C's `data & 0xFFFFFF`).
//! `data` = the 56-bit frame as received (the references' `Key`), `data_count_bit` = 56.
//!
//! **Encoder** (`_gen_data` + `_get_upload`): fields from the frame; `button` = native ctrl code
//! 0x0–0xF (the references' button editor takes any 4-bit value; their D-pad offers 1/2/4/8),
//! anything else keeps the captured button; rolling code + 1 (0xFFFF → 0); key byte kept; checksum
//! and obfuscation rebuilt. Upload: wake-up, 2 hardware sync pairs, software sync, 56 bits,
//! silence 30415; then twice: 7 sync pairs, software sync, 56 bits, silence; `encoder.repeat` = 3
//! copies. (The garage app's TX plugins are built without `SUBGHZ_GARAGE_HAS_LIB_EXTENSIONS` /
//! `_CUSTOM_BTN`, which would put the counter in "OFEX" mode and send button 0; KAT follows the
//! source's normal mode, as Flipper-ARF's firmware and doctor-firmware run it.)
//!
//! **Key file** (`subghz_block_generic_serialize`): `Protocol: Somfy Telis`, `Bit: 56`, `Key`;
//! loading requires `Bit: 56`.

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
    name: "Somfy Telis",
    decoder: "Somfy Telis",
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

/// `subghz_protocol_somfy_telis_get_name_button`.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    const NAMES: [&str; 16] = [
        "Unknown", "My", "Up", "My+Up", "Down", "My+Down", "Up+Down", "0x07", "Prog", "Sun+Flag",
        "Flag", "0x0B", "0x0C", "0x0D", "0x0E", "0x0F",
    ];
    Some(NAMES.get(code as usize).copied().unwrap_or(NAMES[0]))
}

const NAME: &str = "Somfy Telis";
const BIT_COUNT: u8 = 56;
const TX_REPEAT: usize = 3;

/// `SomfyTelisDecoderStep` (`StartDecode` is never entered).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    CheckPreambula,
    FoundPreambula,
    DecoderData,
}

/// Somfy Telis decoder (`SubGhzProtocolDecoderSomfyTelis`).
pub struct SomfyTelisDecoder {
    step: Step,
    header_count: u16,
    manchester: ManchesterState,
    decode_data: u64,
    /// `SubGhzBlockDecoder.decode_count_bit` is a `uint8_t`.
    decode_count_bit: u8,
}

impl SomfyTelisDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            header_count: 0,
            manchester: ManchesterState::Start1,
            decode_data: 0,
            decode_count_bit: 0,
        }
    }

    /// Manchester Reset + LongHigh (the C's two `manchester_advance` calls).
    fn prime_manchester(&mut self) {
        self.manchester = ManchesterState::Mid1;
        self.manchester.advance(ManchesterEvent::LongHigh);
    }
}

impl Default for SomfyTelisDecoder {
    fn default() -> Self {
        Self::new()
    }
}

fn is_sync(duration: u32) -> bool {
    duration_diff!(duration, SOMFY_TE_SHORT * 4) < SOMFY_TE_DELTA * 4
}

/// Fields of a received frame (`subghz_protocol_somfy_telis_check_remote_controller`).
fn decoded_from_data(data: u64) -> DecodedSignal {
    let plain = data ^ (data >> 8);
    DecodedSignal {
        serial: Some((plain & 0xFF_FFFF) as u32),
        button: Some(((plain >> 44) & 0xF) as u8),
        counter: Some(((plain >> 24) & 0xFFFF) as u16),
        crc_valid: somfy_crc_ok(data),
        data,
        data_count_bit: BIT_COUNT as usize,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

/// `subghz_protocol_somfy_telis_gen_data` for a received frame: next frame with `button`.
fn next_frame(data: u64, button: Option<u8>) -> u64 {
    let plain = data ^ (data >> 8);
    let button = button.unwrap_or(((plain >> 44) & 0xF) as u8);
    let counter = ((plain >> 24) & 0xFFFF) as u32;
    let counter = if counter + 1 > 0xFFFF { 0 } else { counter + 1 };
    let serial = (plain & 0xFF_FFFF) as u32;
    somfy_obfuscate([
        (plain >> 48) as u8,
        button << 4,
        (counter >> 8) as u8,
        counter as u8,
        (serial >> 16) as u8,
        (serial >> 8) as u8,
        serial as u8,
    ])
}

/// `subghz_protocol_somfy_telis_create_data` (the SubGHz app's "Add Manually", `gen_data` with
/// `new_remote`): key byte 0xA7, `btn`, counter + 1 (0 past 0xFFFF), the 24-bit address; `Bit: 56`,
/// `Key`.
pub(super) fn create_data(serial: u32, btn: u8, cnt: u16) -> FlipperKey {
    let counter = cnt.checked_add(1).unwrap_or(0);
    let data = somfy_obfuscate([
        0xA7,
        btn << 4,
        (counter >> 8) as u8,
        counter as u8,
        (serial >> 16) as u8,
        (serial >> 8) as u8,
        serial as u8,
    ]);
    FlipperKey::for_protocol(NAME, BIT_COUNT as usize, data)
}

/// One frame of the upload after its hardware sync pairs.
fn push_frame(out: &mut Vec<LevelDuration>, data: u64, sync_pairs: usize) {
    for _ in 0..sync_pairs {
        push(out, true, SOMFY_TE_SHORT * 4);
        push(out, false, SOMFY_TE_SHORT * 4);
    }
    push(out, true, SOMFY_SOFT_SYNC);
    push(out, false, SOMFY_TE_SHORT);
    somfy_push_bits(out, data, BIT_COUNT as u32);
    somfy_push_silence(out, SOMFY_SILENCE);
}

/// `subghz_protocol_encoder_somfy_telis_get_upload` (one copy).
fn push_upload(out: &mut Vec<LevelDuration>, data: u64) {
    push(out, true, SOMFY_WAKEUP_HIGH);
    push(out, false, SOMFY_WAKEUP_LOW);
    push_frame(out, data, 2);
    for _ in 0..2 {
        push_frame(out, data, 7);
    }
}

impl ProtocolDecoder for SomfyTelisDecoder {
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

    /// `subghz_protocol_decoder_somfy_telis_reset`: step and Manchester state only.
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
                        self.step = Step::DecoderData;
                        self.decode_data = 0;
                        self.decode_count_bit = 0;
                        self.header_count = 0;
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
                    // End of frame: exactly 56 bits with a valid checksum (the C's gate).
                    let result = (self.decode_count_bit == BIT_COUNT
                        && somfy_crc_ok(self.decode_data))
                    .then(|| decoded_from_data(self.decode_data));
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
                    self.decode_data = self.decode_data << 1 | bit as u64;
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
        (key.bits == BIT_COUNT as usize).then(|| decoded_from_data(key.key))
    }

    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        (decoded.data_count_bit == BIT_COUNT as usize).then(|| {
            FlipperKey::new(
                NAME,
                BIT_COUNT as usize,
                decoded.data,
                0,
                "FuriHalSubGhzPresetOok650Async",
            )
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::ProtocolRegistry;

    fn feed_all(dec: &mut SomfyTelisDecoder, wire: &[LevelDuration]) -> Vec<DecodedSignal> {
        wire.iter()
            .filter_map(|p| dec.feed(p.level, p.duration_us))
            .collect()
    }

    /// pushstack.wordpress.com / the C comment: 0xA7232323312222 = key A7, ctrl 8 (Prog),
    /// rolling 0, address 0x121300.
    const FRAME: u64 = 0xA7_2323_2331_2222;

    fn one_frame(data: u64) -> Vec<LevelDuration> {
        let mut out = Vec::new();
        push_frame(&mut out, data, 2);
        out
    }

    #[test]
    fn decodes_a_synthetic_frame() {
        let got = feed_all(&mut SomfyTelisDecoder::new(), &one_frame(FRAME));
        assert_eq!(got.len(), 1);
        let d = &got[0];
        assert_eq!((d.data, d.data_count_bit), (FRAME, 56));
        assert_eq!(
            (d.serial, d.button, d.counter),
            (Some(0x12_1300), Some(8), Some(0))
        );
        assert!(d.crc_valid && d.encoder_capable);
    }

    #[test]
    fn rejects_bad_checksum_short_preamble_and_wrong_length() {
        // A flipped bit in the last byte: checksum fails. (A flip in an earlier byte changes two
        // plain bits of the same nibble rank, which the nibble-XOR checksum cannot see.)
        assert!(feed_all(&mut SomfyTelisDecoder::new(), &one_frame(FRAME ^ 1)).is_empty());
        assert!(somfy_crc_ok(FRAME ^ 1 << 20));
        // One hardware sync pair only (`header_count > 1` needed).
        let mut wire = Vec::new();
        push_frame(&mut wire, FRAME, 1);
        assert!(feed_all(&mut SomfyTelisDecoder::new(), &wire).is_empty());
        // 55 bits.
        let mut wire = Vec::new();
        push(&mut wire, true, 2560);
        push(&mut wire, false, 2560);
        push(&mut wire, true, 2560);
        push(&mut wire, false, 2560);
        push(&mut wire, true, SOMFY_SOFT_SYNC);
        push(&mut wire, false, SOMFY_TE_SHORT);
        somfy_push_bits(&mut wire, FRAME, 55);
        somfy_push_silence(&mut wire, SOMFY_SILENCE);
        assert!(feed_all(&mut SomfyTelisDecoder::new(), &wire).is_empty());
        // Off-timing data pulses.
        let slow: Vec<LevelDuration> = one_frame(FRAME)
            .into_iter()
            .map(|p| {
                if p.duration_us == 1280 {
                    LevelDuration::new(p.level, 1600)
                } else {
                    p
                }
            })
            .collect();
        assert!(feed_all(&mut SomfyTelisDecoder::new(), &slow).is_empty());
    }

    #[test]
    fn encode_round_trip_every_button() {
        let src = decoded_from_data(FRAME);
        for button in 0u8..=0xF {
            let wire = SomfyTelisDecoder::new().encode(&src, button).unwrap();
            let got = feed_all(&mut SomfyTelisDecoder::new(), &wire);
            assert_eq!(got.len(), 9, "3 copies x 3 frames");
            for d in &got {
                assert_eq!(
                    (d.button, d.counter, d.serial),
                    (Some(button), Some(1), Some(0x12_1300))
                );
                assert_eq!(d.data >> 48, 0xA7, "key byte kept");
            }
        }
        // Out-of-range button: captured one (8); counter wraps.
        let wrap = decoded_from_data(somfy_obfuscate([0xA7, 0x20, 0xFF, 0xFF, 0x01, 0x02, 0x03]));
        assert_eq!(wrap.counter, Some(0xFFFF));
        let got = feed_all(
            &mut SomfyTelisDecoder::new(),
            &SomfyTelisDecoder::new().encode(&wrap, 0x10).unwrap(),
        );
        assert_eq!(
            (got[0].button, got[0].counter, got[0].serial),
            (Some(2), Some(0), Some(0x01_0203))
        );
    }

    #[test]
    fn upload_layout_matches_the_c() {
        let wire = SomfyTelisDecoder::new()
            .encode(&decoded_from_data(FRAME), 8)
            .unwrap();
        assert_eq!((wire[0].level, wire[0].duration_us), (true, 9415));
        assert_eq!((wire[1].level, wire[1].duration_us), (false, 89565));
        assert!(wire.windows(2).all(|w| w[0].level != w[1].level));
        let syncs = wire
            .iter()
            .filter(|p| p.level && p.duration_us == SOMFY_SOFT_SYNC)
            .count();
        assert_eq!(syncs, 9);
        let wakeups = wire.iter().filter(|p| p.duration_us == 89565).count();
        assert_eq!(wakeups, 3);
        assert!(!wire.last().unwrap().level && wire.last().unwrap().duration_us >= SOMFY_SILENCE);
    }

    #[test]
    fn key_file_round_trip() {
        let dec = SomfyTelisDecoder::new();
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433420000\n\
            Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Somfy Telis\nBit: 56\n\
            Key: 00 A7 23 23 23 31 22 22\n";
        let key = FlipperKey::parse(text).unwrap();
        let d = dec.from_flipper_key(&key).unwrap();
        assert_eq!((d.data, d.button, d.crc_valid), (FRAME, Some(8), true));
        let out = dec.to_flipper_key(&d).unwrap().to_text();
        assert!(
            out.contains("Protocol: Somfy Telis\nBit: 56\nKey: 00 A7 23 23 23 31 22 22\n"),
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
        let mut wrong = key.clone();
        wrong.bits = 80;
        assert!(dec.from_flipper_key(&wrong).is_none());
        assert_eq!(
            ProtocolRegistry::new().decode_key(&key).map(|(n, _)| n),
            Some(NAME.to_string())
        );
    }

    /// Flipper's unit test: `somfy_telis_raw.sub` must decode (skipped without REFERENCES).
    #[test]
    fn decodes_flipper_unit_test_capture() {
        let path = std::path::Path::new(
            "REFERENCES/Flipper-ARF/applications/debug/unit_tests/resources/unit_tests/subghz/somfy_telis_raw.sub",
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
            !feed_all(&mut SomfyTelisDecoder::new(), &pairs).is_empty(),
            "no Somfy Telis frame"
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
