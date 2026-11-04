//! Power Smart roller-shutter remote: 64-bit Manchester frame with a sync word and inverted
//! copies of its fields.
//!
//! Primary reference: doctor-firmware `lib/subghz/protocols/power_smart.c` (Flipper-ARF's garage
//! app ships a copy it does not build; Flipper-ARF's `lib/subghz` copy decodes and transmits the
//! same frames, its custom-button d-pad only re-sends the captured button).
//!
//! **Line coding**: Manchester, te_short = 225 µs, te_long = 450 µs, te_delta = 100 µs (short
//! within te_delta, long within 2·te_delta: 125–325 / 250–650 µs, short tested first). Flipper's
//! convention (1 = LOW then HIGH), and every decoded bit is inverted (`decode_data << 1 | !bit`;
//! the encoder Manchester-encodes the inverted key bits).
//!
//! **Frame**: `0xFD XXXX YY AA ZZZZ WW`: sync bytes 0xFD (bits 56..63) and 0xAA (bits 24..31),
//! XXXX = !ZZZZ and YY = (!WW) − 1 — `subghz_protocol_power_smart_check_valid`.
//!
//! **Decoder** (`subghz_protocol_decoder_power_smart_feed`): every short/long pulse is a
//! Manchester event; on each event (a bit decoded or not) the 64-bit shift register is tested:
//! sync bytes in place and check_valid → reported (and the register cleared). Any other pulse
//! clears the register and resets the Manchester decoder. `reset()` only resets the Manchester
//! state (the register survives it, as in the C). No preamble or length gate beyond that.
//!
//! **Fields** (`subghz_protocol_power_smart_remote_controller`): button = K1K2 = bit 55 ‖ bit 40
//! (1 Down, 2 Up, 3 Stop, 0 Unknown); serial = (data >> 33) & 0x3FFF00 | (data >> 32) & 0xFF;
//! counter = the C's `cnt`, bits 49..54: the 6-bit channel mask the references display.
//! `crc_valid` = true (check_valid passed).
//!
//! **Encoder** (`subghz_protocol_encoder_power_smart_get_upload`): 8 copies of the 64 inverted
//! bits through Flipper's Manchester encoder, the closing half kept only when HIGH, then LOW
//! 1111·te_long (499 950 µs); the whole upload sent 3 times. Static: the button is spread over
//! the inverted copies, so the references re-send the captured frame (`button` is ignored).
//!
//! **Key file**: `Protocol: Power Smart`, `Bit: 64`, `Key`, then doctor-firmware's generic
//! serialize adds `Serial` / `Btn` / `Cnt` (decimal) when any of them is non-zero.

use super::common::{ManchesterEvent, ManchesterState};
use super::g3_common::{duration_diff, repeat_upload, ManchesterEncoder, UploadBuilder};
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Power Smart",
    decoder: "Power Smart",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Power Smart",
    category: Category::Gate,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// `subghz_protocol_power_smart_get_name_button` (K1K2).
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code & 0x3 {
        1 => "Down",
        2 => "Up",
        3 => "Stop",
        _ => "Unknown",
    })
}

const TE_SHORT: u32 = 225;
const TE_LONG: u32 = 450;
const TE_DELTA: u32 = 100;
const BIT_COUNT: usize = 64;
const HEADER: u64 = 0xFD00_0000_AA00_0000;
const HEADER_MASK: u64 = 0xFF00_0000_FF00_0000;
/// Frame copies per upload.
const COPIES: usize = 8;
/// LOW after the copies: 1111·te_long.
const END_LOW: u32 = TE_LONG * 1111;
/// `subghz_protocol_encoder_power_smart_alloc`: repeat = 3.
const REPEAT: usize = 3;

/// Power Smart decoder (`SubGhzProtocolDecoderPowerSmart`).
pub struct PowerSmartDecoder {
    manchester: ManchesterState,
    decode_data: u64,
}

/// `subghz_protocol_power_smart_check_valid`.
fn check_valid(packet: u64) -> bool {
    let data_1 = (packet >> 40) & 0xFFFF;
    let data_2 = (!packet >> 8) & 0xFFFF;
    let data_3 = (packet >> 32) as u8;
    let data_4 = ((!packet) as u8).wrapping_sub(1);
    data_1 == data_2 && data_3 == data_4
}

/// (button, serial, channel mask) — `subghz_protocol_power_smart_remote_controller`.
fn fields(data: u64) -> (u8, u32, u16) {
    let button = (((data >> 54) & 0x02) | ((data >> 40) & 0x1)) as u8;
    let serial = (((data >> 33) & 0x3F_FF00) | ((data >> 32) & 0xFF)) as u32;
    let channel = ((data >> 49) & 0x3F) as u16;
    (button, serial, channel)
}

fn signal(data: u64) -> DecodedSignal {
    let (button, serial, channel) = fields(data);
    DecodedSignal {
        serial: Some(serial),
        button: Some(button),
        counter: Some(channel),
        crc_valid: check_valid(data) && data & HEADER_MASK == HEADER,
        data,
        data_count_bit: BIT_COUNT,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

/// One upload (`get_upload`).
fn upload(data: u64) -> Vec<LevelDuration> {
    let mut enc = ManchesterEncoder::new();
    let mut results = Vec::with_capacity(2 * BIT_COUNT * COPIES);
    for _ in 0..COPIES {
        for i in (0..BIT_COUNT).rev() {
            enc.encode_bit((data >> i) & 1 == 0, &mut results);
        }
    }
    let mut b = UploadBuilder::new();
    for r in results {
        let p = r.pulse(TE_SHORT, TE_LONG);
        b.push(p.level, p.duration_us);
    }
    let last = enc.finish().pulse(TE_SHORT, TE_LONG);
    if last.level {
        b.push(true, last.duration_us);
    }
    b.push(false, END_LOW);
    b.finish()
}

impl PowerSmartDecoder {
    pub fn new() -> Self {
        Self {
            manchester: ManchesterState::Mid1,
            decode_data: 0,
        }
    }
}

impl Default for PowerSmartDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for PowerSmartDecoder {
    fn name(&self) -> &'static str {
        "Power Smart"
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

    /// `subghz_protocol_decoder_power_smart_reset`: the Manchester state only.
    fn reset(&mut self) {
        self.manchester = ManchesterState::Mid1;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let long = if duration_diff(duration, TE_SHORT) < TE_DELTA {
            false
        } else if duration_diff(duration, TE_LONG) < TE_DELTA * 2 {
            true
        } else {
            self.decode_data = 0;
            self.manchester = ManchesterState::Mid1;
            return None;
        };
        if let Some(bit) = self
            .manchester
            .advance(ManchesterEvent::from_half_bit(level, long))
        {
            self.decode_data = (self.decode_data << 1) | u64::from(!bit);
        }
        if self.decode_data & HEADER_MASK == HEADER && check_valid(self.decode_data) {
            let found = signal(self.decode_data);
            self.decode_data = 0;
            return Some(found);
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
        // subghz_block_generic_deserialize_check_count_bit(.., 64); the C decodes the fields
        // from Key (the Serial/Btn/Cnt lines are informational).
        (key.bits == BIT_COUNT).then(|| signal(key.key))
    }

    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != BIT_COUNT {
            return None;
        }
        let mut k = FlipperKey::new(
            "Power Smart",
            BIT_COUNT,
            decoded.data,
            433_920_000,
            "FuriHalSubGhzPresetOok650Async",
        );
        // doctor-firmware subghz_block_generic_write_optional_fields.
        let (button, serial, channel) = fields(decoded.data);
        if serial != 0 || button != 0 || channel != 0 {
            k.push("Serial", serial.to_string());
            k.push("Btn", button.to_string());
            k.push("Cnt", channel.to_string());
        }
        Some(k)
    }
}

#[cfg(test)]
mod tests {
    use super::super::g3_common::test_support::*;
    use super::*;

    /// power_smart.sub (Flipper unit test).
    const KEY_FILE: &str = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
        Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Power Smart\nBit: 64\n\
        Key: FD C1 36 AC AA 3E C9 52\n";

    #[test]
    fn check_valid_matches_the_c_examples() {
        // remote_controller's examples (K1K2 = 01 / 11 / 10 …).
        for key in [
            0xFD21_37AC_AADE_C852u64,
            0xFDA1_37AC_AA5E_C852,
            0xFDA1_36AC_AA5E_C952,
            0xFDC1_36AC_AA3E_C952,
        ] {
            assert!(check_valid(key), "{key:016X}");
        }
        assert!(!check_valid(0xFD21_37AC_AADE_C853));
        assert_eq!(fields(0xFD21_37AC_AADE_C852).0, 0b01);
        assert_eq!(fields(0xFDA1_37AC_AA5E_C852).0, 0b11);
        assert_eq!(fields(0xFDA1_36AC_AA5E_C952).0, 0b10);
    }

    #[test]
    fn unit_test_key_file_encodes_and_decodes_back() {
        let d = PowerSmartDecoder::new();
        let s = d
            .from_flipper_key(&FlipperKey::parse(KEY_FILE).unwrap())
            .unwrap();
        assert_eq!(s.data, 0xFDC1_36AC_AA3E_C952);
        assert!(s.crc_valid);
        let tx = d.encode(&s, 0).unwrap();
        let frames = feed_all(&mut PowerSmartDecoder::new(), &tx);
        // 8 copies per upload; the first copy of each upload is only partly read (the decoder locks
        // during it), so 7 per upload: 21, as the reference's own encoder test decodes.
        assert_eq!(frames.len(), 21);
        assert!(frames.iter().all(|f| f.data == s.data && f.crc_valid));
        let text = d.to_flipper_key(&s).unwrap().to_text();
        assert!(
            text.contains("Protocol: Power Smart\nBit: 64\nKey: FD C1 36 AC AA 3E C9 52\n"),
            "{text}"
        );
        let (b, sn, ch) = fields(s.data);
        assert!(
            text.contains(&format!("Serial: {sn}\nBtn: {b}\nCnt: {ch}\n")),
            "{text}"
        );
        let back = d
            .from_flipper_key(&FlipperKey::parse(&text).unwrap())
            .unwrap();
        assert_eq!(format!("{back:?}"), format!("{s:?}"));
    }

    #[test]
    fn upload_layout() {
        let one = upload(0xFDC1_36AC_AA3E_C952);
        let last = one.last().unwrap();
        assert_eq!((last.level, last.duration_us), (false, 499_950));
        assert!(one.iter().all(
            |p| [225, 450, 499_950].contains(&p.duration_us) || p.duration_us == 225 + 499_950
        ));
    }

    #[test]
    fn rejects_corrupted_frames() {
        // A frame whose inverted copy does not match never reports.
        let tx = repeat_upload(&upload(0xFDC1_36AC_AA3E_C953), 3);
        assert!(feed_all(&mut PowerSmartDecoder::new(), &tx).is_empty());
        // A pulse outside both windows clears the register.
        let mut d = PowerSmartDecoder::new();
        d.decode_data = 0x00FD_C136_ACAA_3EC9;
        d.feed(true, 800);
        assert_eq!(d.decode_data, 0);
    }

    #[test]
    fn reset_keeps_the_register() {
        let mut d = PowerSmartDecoder::new();
        d.decode_data = 0x1234;
        d.manchester = ManchesterState::Start0;
        d.reset();
        assert_eq!(
            (d.decode_data, d.manchester),
            (0x1234, ManchesterState::Mid1)
        );
    }

    #[test]
    fn decodes_the_flipper_unit_test_capture() {
        let Some(path) = unit_test_file("power_smart_raw.sub") else {
            return;
        };
        let frames = decoder_decodes(&mut PowerSmartDecoder::new(), &path);
        assert!(
            !frames.is_empty(),
            "the reference unit test needs ≥ 1 frame"
        );
        assert!(frames.iter().all(|f| f.crc_valid));
        let names: Vec<String> = registry_decodes(&path)
            .into_iter()
            .map(|(n, _)| n)
            .collect();
        assert!(names.iter().any(|n| n == "Power Smart"), "{names:?}");
    }
}
