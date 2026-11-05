//! Markisol / E-Motion / BOFU / Rollerhouse / BF-30x / BF-415 curtain remotes (receive only; kept
//! but NOT registered).
//!
//! Port of rtl_433 `src/devices/markisol.c` (`REFERENCES/rtl_433`, GPL-2.0; model "Markisol",
//! r_device "Markisol, E-Motion, BOFU, Rollerhouse, BF-30x, BF-415 curtain remote", `-R 191`).
//! rtl_433 ships it disabled (`.disabled = 1`: "excessive false positives, see issue #3611"; it
//! runs only when asked for with `-R 191`), and KAT mirrors that the way it mirrors Flipper-ARF's
//! disabled protocols: the module, its tests and docs stay, the registry does not list it
//! (`Box::new(markisol::MarkisolDecoder::new())` enables it). No recordings exist; validated on
//! synthetic frames through rtl_433 itself (see `docs/markisol.md`).
//!
//! **Air format**: OOK_PULSE_PWM, short 368 µs, long 704 µs, sync 5628 µs, `gap_limit` =
//! `reset_limit` = 2000 µs, no tolerance (so the slicer splits at the midpoints: a pulse under
//! 537 µs = 1, up to 3166 µs = 0, longer = sync). A frame is HIGH 4886 (sync), LOW 2470 (ends
//! the previous message), HIGH 1647 (read as a 0 bit), LOW 315, then 40 bits (0 = HIGH 670 + LOW
//! 320, 1 = HIGH 348 + LOW 642). So a frame is decoded at the gap after the next frame's sync (or
//! at its own final gap), from a 41-bit row (42 with a trailing pulse).
//!
//! **Frame** (`markisol_decode`): the first row of 41 or 42 bits, shifted by the spurious first
//! bit, each byte bit-reversed and inverted: 16-bit address, channel / zone / control (interwoven)
//! and a checksum byte; all five bytes must sum to 1 (mod 256), else the frame is dropped.
//!
//! **Fields**: `serial` = rtl_433 `id` (address, 16 bits); `button` = the 4-bit control code
//! (labels: rtl_433's `control` strings); `counter`: none; rtl_433's `channel` (`data` bits
//! 16..19 of byte 2) and `zone` have no field. `data` = the five decoded bytes (40 bits);
//! `crc_valid` = the checksum passed.
//!
//! **Encoder**: none — receive only, as rtl_433.

use super::rtl433::{frame_signal, reverse8, BitBuffer, Decode, Device, Ends, Gaps, Modulation, Receiver};
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// rtl_433's model name.
pub(super) const NAME: &str = "Markisol";

/// Metadata (see `info.rs`) for when it is enabled: OOK PWM, fixed code, curtain remote.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &["Markisol, E-Motion, BOFU, Rollerhouse, BF-30x, BF-415 curtain remote"],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Markisol",
    category: Category::Gate,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// rtl_433's `control_strs`.
const CONTROL: [&str; 16] = [
    "Limit (0)", "Down (1)", "? (2)", "H-Down (3)", "Confirm (4)", "Stop (5)", "? (6)", "? (7)", "? (8)", "? (9)",
    "? (10)", "? (11)", "Up (12)", "Limit (13)", "H-Up (14)", "? (15)",
];

fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    CONTROL.get(usize::from(code)).copied()
}

/// `r_device markisol`.
const DEVICE: Device = Device {
    modulation: Modulation::OokPwm,
    short_width: 368,
    long_width: 704,
    reset_limit: 2000,
    gap_limit: 2000,
    sync_width: 5628,
    tolerance: 0,
    gaps: Gaps::Merged,
    ends: Ends::Ook,
    resync: true,
};

/// `markisol_decode`.
fn markisol_decode(bits: &BitBuffer) -> Option<DecodedSignal> {
    let mut buf = [0u8; 5];
    let mut cksum: u8 = 0;
    let mut found = false;
    for i in 0..bits.num_rows() {
        if bits.bits_per_row(i) == 41 || bits.bits_per_row(i) == 42 {
            let b = bits.row(i);
            for j in 0..5 {
                // Shift the stream to discard the spurious first bit.
                let v = (b[j] << 1).wrapping_add(b[j + 1] >> 7);
                buf[j] = !reverse8(v);
                cksum = cksum.wrapping_add(buf[j]);
            }
            found = true;
            break;
        }
    }
    if !found || cksum != 1 {
        return None; // DECODE_ABORT_EARLY / DECODE_FAIL_MIC
    }
    let address = u32::from(buf[0]) << 8 | u32::from(buf[1]);
    let control = ((buf[2] >> 4) & !2) | ((buf[3] & 0x10) >> 3);
    let mut d = frame_signal(&buf, 40);
    d.serial = Some(address);
    d.button = Some(control);
    d.crc_valid = true;
    Some(d)
}

/// Markisol decoder (rtl_433 `markisol.c`).
pub struct MarkisolDecoder {
    rx: Receiver,
}

impl MarkisolDecoder {
    pub fn new() -> Self {
        Self { rx: Receiver::new(&DEVICE) }
    }
}

impl Default for MarkisolDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for MarkisolDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: 348, te_long: 670, te_delta: 160, min_count_bit: 40 }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.rx.reset(&mut Decode(markisol_decode));
    }

    fn feed(&mut self, level: bool, duration_us: u32) -> Option<DecodedSignal> {
        self.rx.feed(&mut Decode(markisol_decode), level, duration_us)
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }
}

#[cfg(test)]
pub(super) mod tests {
    use super::*;
    use crate::protocols::rtl433::test_util::decode_all;

    /// The five frame bytes for an address / channel / zone / control (byte 3's unknown bits
    /// 0x01, as the C's author saw), checksum making the sum 1.
    pub(crate) fn frame_bytes(address: u16, channel: u8, zone: u8, control: u8) -> [u8; 5] {
        let z = zone - 1;
        let b2 = (channel & 0xf) | ((z & 1) << 5) | ((control & !2) << 4);
        let b3 = 0x01 | ((control & 2) << 3) | ((z & 2) << 6);
        let mut f = [(address >> 8) as u8, address as u8, b2, b3, 0];
        let sum = f[..4].iter().fold(0u8, |a, &b| a.wrapping_add(b));
        f[4] = 1u8.wrapping_sub(sum);
        f
    }

    /// The air frames: per frame HIGH 4886, LOW 2470, HIGH 1647, LOW 315, then the 40 wire bits
    /// (each byte inverted and LSB first); `repeats` frames, then silence.
    pub(crate) fn wire(bytes: &[u8; 5], repeats: usize) -> Vec<LevelDuration> {
        let mut w = vec![LevelDuration::new(false, 20_000)];
        for _ in 0..repeats {
            w.extend([LevelDuration::new(true, 4886), LevelDuration::new(false, 2470), LevelDuration::new(true, 1647), LevelDuration::new(false, 315)]);
            for &b in bytes {
                let wire_byte = reverse8(!b);
                for i in (0..8).rev() {
                    let (h, l) = if (wire_byte >> i) & 1 == 1 { (348, 642) } else { (670, 320) };
                    w.push(LevelDuration::new(true, h));
                    w.push(LevelDuration::new(false, l));
                }
            }
        }
        w.push(LevelDuration::new(false, 50_000));
        w
    }

    #[test]
    fn decodes_every_control_channel_and_zone() {
        for control in 0..16u8 {
            for (channel, zone) in [(1u8, 1u8), (15, 4), (7, 2), (3, 3)] {
                let f = frame_bytes(0xBEEF, channel, zone, control);
                for repeats in [1, 3] {
                    let got = decode_all(&mut MarkisolDecoder::new(), &wire(&f, repeats));
                    assert_eq!(got.len(), repeats, "control {control} ch {channel} zone {zone}");
                    let d = &got[0];
                    assert_eq!((d.serial, d.button, d.crc_valid), (Some(0xBEEF), Some(control), true));
                    assert_eq!(((d.data >> 16) & 0xF) as u8, channel);
                    let zone_got = (((d.data >> 16) & 0x20) >> 5) + (((d.data >> 8) & 0x80) >> 6) + 1;
                    assert_eq!(zone_got as u8, zone);
                    assert_eq!(native_buttons(NAME, control), Some(CONTROL[usize::from(control)]));
                }
            }
        }
    }

    #[test]
    fn failed_checks_are_dropped() {
        let mut f = frame_bytes(0x1234, 2, 1, 12);
        f[4] ^= 0x10;
        assert!(decode_all(&mut MarkisolDecoder::new(), &wire(&f, 2)).is_empty(), "checksum");
        // 39 bits: the row is too short.
        let f = frame_bytes(0x1234, 2, 1, 12);
        let mut w = wire(&f, 1);
        w.truncate(w.len() - 3);
        w.push(LevelDuration::new(false, 50_000));
        assert!(decode_all(&mut MarkisolDecoder::new(), &w).is_empty());
    }
}
