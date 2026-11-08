//! Cardin S466-TX2 garage-door remote (receive only).
//!
//! Port of rtl_433 `src/devices/cardin.c` (`REFERENCES/rtl_433`, GPL-2.0; r_device "Cardin
//! S466-TX2", model "Cardin-S466", decoder 17, enabled by default). No Flipper reference has it
//! (RogueMaster's "Cardin S508" is a different, 868 MHz rolling-code remote). The remote sends on
//! 27.195 MHz (rtl_433: "Remember to set the correct freq with -f 27.195M"), which the HackRF
//! receives; like every rtl_433 port the decoder runs on any frequency.
//!
//! **Air format**: OOK_PULSE_PWM, short 730 µs (→ 1), long 1400 µs (→ 0), sync 6150 µs,
//! `gap_limit` 1600 µs, `reset_limit` 32000 µs, no tolerance, so the slicer splits at the midpoints:
//! a pulse under 1065 µs is a 1, up to 3775 µs a 0, longer a sync (a new row). rtl_433: "Similar to
//! an EV1527 / SC2260, but there is a 6152 us sync pulse first, then 24 bit of 732 us / 1412 us
//! leading-gap PWM."
//!
//! **Frame** (`cardin_decode`): row 0 must hold exactly 24 bits (`DECODE_ABORT_LENGTH`). The low six
//! bits of byte 2 are the button code, one of 0x03, 0x06, 0x09, 0x0C; the nine tri-state DIP
//! switches are bit pairs (first bit 0 = `-`, 1 then 0 = `o`, 1 then 1 = `+`) and the fourth
//! combination (0 then 1) is refused (`DECODE_ABORT_EARLY`):
//!
//! | DIP | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
//! |---|---|---|---|---|---|---|---|---|---|
//! | first bit | b0 0x08 | b0 0x10 | b0 0x20 | b0 0x40 | b0 0x80 | b2 0x80 | b0 0x01 | b0 0x02 | b0 0x04 |
//! | second bit | b1 0x08 | b1 0x10 | b1 0x20 | b1 0x40 | b1 0x80 | b2 0x40 | b1 0x01 | b1 0x02 | b1 0x04 |
//!
//! No checksum: the button set and the DIP rule are the only checks.
//!
//! **Fields**: `serial` = the 18 DIP bits (`b0 << 10 | b1 << 2 | b2 >> 6`); `button` = rtl_433's
//! `rbutton` index ((byte 2 & 0x0F) / 3 − 1): 0 "11R", 1 "10R", 2 "01R", 3 "00L?" (the right
//! button's two switches, or the left button); `data` = the 24 bits; `crc_valid` false.
//!
//! **Encoder**: none — receive only, as rtl_433 (Replay sends the recorded pulses).

use super::rtl433::{frame_signal, BitBuffer, Decode, Device, Ends, Gaps, Modulation, Receiver};
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// rtl_433's model name.
pub(super) const NAME: &str = "Cardin-S466";

/// Metadata (see `info.rs`): OOK PWM, fixed code (tri-state DIP switches), garage-door remote.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &["Cardin S466-TX2"],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Cardin",
    category: Category::Gate,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// rtl_433's `rbutton` strings, by index.
const RBUTTON: [&str; 4] = ["11R", "10R", "01R", "00L?"];

fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    RBUTTON.get(usize::from(code)).copied()
}

/// `r_device cardin`.
const DEVICE: Device = Device {
    modulation: Modulation::OokPwm,
    short_width: 730,
    long_width: 1400,
    reset_limit: 32000,
    gap_limit: 1600,
    sync_width: 6150,
    tolerance: 0,
    gaps: Gaps::Merged,
    ends: Ends::Ook,
    resync: true,
};

/// The (first, second) bit masks of DIP switches 1..9 in bytes (0, 1), except DIP 6 (byte 2).
const DIP_MASKS: [u8; 9] = [8, 16, 32, 64, 128, 0, 1, 2, 4];

/// One DIP switch's (first, second) bits.
fn dip_bits(b: &[u8], k: usize) -> (bool, bool) {
    if k == 5 {
        (b[2] & 128 != 0, b[2] & 64 != 0)
    } else {
        (b[0] & DIP_MASKS[k] != 0, b[1] & DIP_MASKS[k] != 0)
    }
}

/// rtl_433's `dipswitch` string: `-`, `o` or `+` per switch.
#[cfg(test)]
fn dip_string(b: &[u8]) -> String {
    (0..9)
        .map(|k| match dip_bits(b, k) {
            (false, _) => '-',
            (true, false) => 'o',
            (true, true) => '+',
        })
        .collect()
}

/// `cardin_decode`.
fn cardin_decode(bits: &BitBuffer) -> Option<DecodedSignal> {
    if bits.bits_per_row(0) != 24 {
        return None; // DECODE_ABORT_LENGTH
    }
    let b = bits.row(0);
    if !matches!(b[2] & 0x3f, 0x03 | 0x06 | 0x09 | 0x0c) {
        return None; // DECODE_ABORT_EARLY
    }
    if (0..9).any(|k| dip_bits(b, k) == (false, true)) {
        return None; // DECODE_ABORT_EARLY: the fourth tri-state option
    }
    let mut d = frame_signal(&b[..3], 24);
    d.serial = Some(u32::from(b[0]) << 10 | u32::from(b[1]) << 2 | u32::from(b[2] >> 6));
    d.button = Some((b[2] & 0x0f) / 3 - 1);
    Some(d)
}

/// The 24 frame bits of a decode as bytes (for the DIP string).
#[cfg(test)]
fn frame_bytes(d: &DecodedSignal) -> [u8; 3] {
    let v = d.data as u32;
    [(v >> 16) as u8, (v >> 8) as u8, v as u8]
}

/// rtl_433's JSON for a decode (`data_print_jsons`, no `time`), for the oracle comparisons.
#[cfg(test)]
pub(super) fn rtl433_json(d: &DecodedSignal) -> String {
    format!(
        "{{\"model\":\"{NAME}\",\"dipswitch\":\"{}\",\"rbutton\":\"{}\"}}",
        dip_string(&frame_bytes(d)),
        RBUTTON[usize::from(d.button.unwrap_or(0) & 3)]
    )
}

/// Cardin S466-TX2 decoder (rtl_433 `cardin.c`).
pub struct CardinS466Decoder {
    rx: Receiver,
}

impl CardinS466Decoder {
    pub fn new() -> Self {
        Self { rx: Receiver::new(&DEVICE) }
    }
}

impl Default for CardinS466Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for CardinS466Decoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: 730, te_long: 1400, te_delta: 335, min_count_bit: 24 }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.rx.reset(&mut Decode(cardin_decode));
    }

    fn feed(&mut self, level: bool, duration_us: u32) -> Option<DecodedSignal> {
        self.rx.feed(&mut Decode(cardin_decode), level, duration_us)
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    /// Receive only, as rtl_433.
    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }
}

#[cfg(test)]
pub(super) mod tests {
    use super::*;
    use crate::protocols::rtl433::test_util::decode_all;

    /// The three frame bytes for nine DIP switches (`-`, `o`, `+`) and an `rbutton` index.
    pub(crate) fn frame(dips: &str, button: u8) -> [u8; 3] {
        let mut b = [0u8, 0, 3 * (button + 1)];
        for (k, c) in dips.chars().enumerate() {
            let (first, second) = match c {
                '-' => (false, false),
                'o' => (true, false),
                '+' => (true, true),
                _ => panic!("{c}"),
            };
            let (m0, m1, i0, i1) = if k == 5 { (128, 64, 2, 2) } else { (DIP_MASKS[k], DIP_MASKS[k], 0, 1) };
            if first {
                b[i0] |= m0;
            }
            if second {
                b[i1] |= m1;
            }
        }
        b
    }

    /// The air frames: a 6150 µs sync pulse and its gap, then 24 bits (1 = 732 µs pulse + 1412 µs
    /// gap, 0 = 1412 + 732), `repeats` times with `pause` µs between them, then silence.
    pub(crate) fn wire(bytes: &[u8; 3], repeats: usize, pause: u32) -> Vec<LevelDuration> {
        let mut w = vec![LevelDuration::new(false, 50_000)];
        for r in 0..repeats {
            if r > 0 {
                w.last_mut().unwrap().duration_us = pause;
            }
            w.push(LevelDuration::new(true, 6150));
            w.push(LevelDuration::new(false, 1412));
            for i in 0..24 {
                let one = bytes[i / 8] >> (7 - i % 8) & 1 == 1;
                let (h, l) = if one { (732, 1412) } else { (1412, 732) };
                w.push(LevelDuration::new(true, h));
                w.push(LevelDuration::new(false, l));
            }
        }
        w.last_mut().unwrap().duration_us = 60_000;
        w
    }

    #[test]
    fn decodes_buttons_and_dip_switches() {
        for dips in ["---------", "+++++++++", "ooooooooo", "+o-+o-+o-", "-o+--+o+-"] {
            for button in 0..4u8 {
                let f = frame(dips, button);
                let got = decode_all(&mut CardinS466Decoder::new(), &wire(&f, 1, 0));
                assert_eq!(got.len(), 1, "{dips} {button}");
                let d = &got[0];
                assert_eq!((d.data, d.data_count_bit), (u64::from(f[0]) << 16 | u64::from(f[1]) << 8 | u64::from(f[2]), 24));
                assert_eq!(d.button, Some(button));
                assert_eq!(d.serial, Some(u32::from(f[0]) << 10 | u32::from(f[1]) << 2 | u32::from(f[2] >> 6)));
                assert_eq!(dip_string(&f), dips);
                assert!(!d.crc_valid && !d.encoder_capable);
            }
        }
        assert_eq!(native_buttons(NAME, 3), Some("00L?"));
        let d = &decode_all(&mut CardinS466Decoder::new(), &wire(&frame("+o-+o-+o-", 1), 1, 0))[0];
        assert_eq!(rtl433_json(d), r#"{"model":"Cardin-S466","dipswitch":"+o-+o-+o-","rbutton":"10R"}"#);
    }

    /// Repeats inside one package (gaps under 32 ms): rtl_433 decodes row 0 once per package.
    #[test]
    fn one_decode_per_package() {
        let f = frame("o+-o+-o+-", 2);
        assert_eq!(decode_all(&mut CardinS466Decoder::new(), &wire(&f, 4, 20_000)).len(), 1);
        // Packages apart (a gap over 32 ms): one decode each.
        assert_eq!(decode_all(&mut CardinS466Decoder::new(), &wire(&f, 3, 40_000)).len(), 3);
    }

    #[test]
    fn rejects_failed_checks() {
        // A button code outside 0x03 / 0x06 / 0x09 / 0x0C.
        let mut f = frame("+--------", 0);
        f[2] = (f[2] & 0xc0) | 0x05;
        assert!(decode_all(&mut CardinS466Decoder::new(), &wire(&f, 1, 0)).is_empty());
        // The fourth tri-state option (0 then 1) on DIP 1 and on DIP 6.
        let mut f = frame("---------", 1);
        f[1] |= 8;
        assert!(decode_all(&mut CardinS466Decoder::new(), &wire(&f, 1, 0)).is_empty());
        let mut f = frame("---------", 1);
        f[2] |= 64;
        assert!(decode_all(&mut CardinS466Decoder::new(), &wire(&f, 1, 0)).is_empty());
        // 23 bits: the last bit's pulse dropped.
        let f = frame("+o-+o-+o-", 0);
        let mut w = wire(&f, 1, 0);
        let n = w.len();
        w.truncate(n - 2);
        w.last_mut().unwrap().duration_us = 60_000;
        assert!(decode_all(&mut CardinS466Decoder::new(), &w).is_empty());
    }
}
