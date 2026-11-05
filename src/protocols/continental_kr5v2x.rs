//! Continental KR5V2X car remote decoder (receive only).
//!
//! Port of rtl_433 `src/devices/continental_car_remote.c` (`continental_car_remote_decode`,
//! r_device "Continental KR5V2X Car Remote (-f 313.8M -s 1024k)", model `Continental-KR5V2X`):
//! Continental 72147-SNA-A01, FCC ID KR5V2X, an OEM remote for Honda (313.8 MHz). rtl_433 calls it a
//! rolling code with an unencrypted sequence number.
//!
//! **Modulation** — FSK, rtl_433's Manchester zero-bit slicer (`FSK_PULSE_MANCHESTER_ZEROBIT`,
//! short 100 µs, long 200 µs, reset limit 1500 µs, no tolerance), see [super::rtl433]: the upper
//! tone is HIGH, a falling data edge is `1`, a rising one `0`, every message starts with the
//! slicer's hard-coded `0`, and a lower-tone stretch over 1.5 ms (or the end of the carrier) ends
//! the message. A press sends a long wake-up (`0x87` repeated, ~1000 bits) with the frame, then the
//! frame again after a ~1.6 ms gap; Trunk/Panic long presses repeat that every ~95 ms.
//!
//! **Frame** — the decoder looks for the last 20 bits of the preamble, `0xF0F06`, in row 0 and reads
//! the 112 bits after it (14 bytes, MSB first):
//!
//! | bytes | field |
//! |-------|-------|
//! | 0..3  | remote ID (32 bits) |
//! | 4     | unknown |
//! | 5     | button (high nibble), unknown (low nibble) |
//! | 6..8  | sequence (24 bits, plain counter) |
//! | 9..12 | encrypted code (32 bits) |
//! | 13    | checksum: the 14 bytes XOR to 0 |
//!
//! **Gating** (the C's checks, in order; a frame failing one is dropped, as rtl_433 returns
//! `DECODE_ABORT_*` / `DECODE_FAIL_SANITY` / `DECODE_FAIL_MIC`): row 0 has ≥ 132 bits; ≥ 112 bits
//! follow the pattern (when it is missing `bitbuffer_search` returns the row length, which fails
//! this too); ID, button and sequence non-zero, ID and encrypted ≠ `0x0FFFFFFF` (the C compares the
//! 32-bit values with seven F's; kept as is, so an all-ones ID passes) and sequence ≠ `0xFFFFFF`;
//! then the XOR checksum. rtl_433 outputs `mic: CHECKSUM`.
//!
//! **Field mapping** — `serial` = ID (rtl_433 `id`, printed `%08X`), `button` = the button nibble
//! (`button_code`: 1 Lock, 3 Unlock, 9 Trunk Long Press, 0xA Trunk/Panic Short Press, 0xB Panic
//! Long Press, others "?"), `counter` = the low 16 bits of the 24-bit `sequence`, `crc_valid` = true
//! (the checksum passed). `data` = bytes 0..7, `extra` = bytes 8..13 (48 bits, right-aligned),
//! `data_count_bit` = 112: rtl_433's `sequence` = `(data & 0xFFFF) << 8 | extra >> 40` and
//! `encrypted` = `(extra >> 8) & 0xFFFFFFFF`.
//!
//! Receive only (rtl_433 has no encoder): the app offers Replay of the recorded pulses.
//!
//! rtl_433 also has an alternate decoder for the same FCC ID, "Honda Keyfob KR5V2X/1X"
//! (`honda_kr5v2x1x.rs`, 60/120 µs, a different layout and CRC); it decodes none of the
//! Continental recordings.

use super::rtl433::{frame_signal, xor_bytes, BitBuffer, Decode, Device, Ends, Gaps, Modulation, Receiver};
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

const NAME: &str = "Continental-KR5V2X";

/// Metadata (see `info.rs`): FSK (2-FSK, ~±45 kHz on the recordings → FM476), receive only.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &["Continental KR5V2X Car Remote"],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::FM,
    encryption: "Rolling Code",
    make: "Honda/Acura",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&FM476],
};

/// rtl_433's `button_str`.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x1 => "Lock",
        0x3 => "Unlock",
        0x9 => "Trunk Long Press",
        0xA => "Trunk/Panic Short Press",
        0xB => "Panic Long Press",
        _ => "Other",
    })
}

/// r_device timing: `short_width` (half-bit), `long_width`, `reset_limit` (µs).
const SHORT_US: u32 = 100;
const LONG_US: u32 = 200;
const RESET_US: u32 = 1500;
const FRAME_BITS: usize = 112;

/// The `r_device`; a burst's FM pairs are one package, consecutive LOWs one gap (see `rtl433`).
const DEVICE: Device = Device {
    modulation: Modulation::FskManchesterZerobit,
    short_width: SHORT_US,
    long_width: LONG_US,
    reset_limit: RESET_US,
    gap_limit: 0,
    sync_width: 0,
    tolerance: 0,
    gaps: Gaps::Merged,
    ends: Ends::None,
    resync: false,
};

/// `continental_car_remote_decode` on one message's bitbuffer.
fn decode(bits: &BitBuffer) -> Option<DecodedSignal> {
    let len = bits.bits_per_row(0);
    if len < 132 {
        return None; // DECODE_ABORT_LENGTH
    }
    let offset = bits.search(0, 0, &[0xF0, 0xF0, 0x60], 20) + 20;
    if len < offset + FRAME_BITS {
        return None; // DECODE_ABORT_EARLY
    }
    let mut b = [0u8; 14];
    bits.extract_bytes(0, offset, &mut b, FRAME_BITS);
    let id = u32::from_be_bytes([b[0], b[1], b[2], b[3]]);
    let button = b[5] >> 4;
    let sequence = u32::from_be_bytes([0, b[6], b[7], b[8]]);
    let encrypted = u32::from_be_bytes([b[9], b[10], b[11], b[12]]);
    if id == 0
        || button == 0
        || sequence == 0
        || id == 0x0FFF_FFFF
        || encrypted == 0x0FFF_FFFF
        || sequence == 0xFF_FFFF
    {
        return None; // DECODE_FAIL_SANITY
    }
    if xor_bytes(&b) != 0 {
        return None; // DECODE_FAIL_MIC
    }
    let mut d = frame_signal(&b, FRAME_BITS);
    d.serial = Some(id);
    d.button = Some(button);
    d.counter = Some(sequence as u16);
    d.crc_valid = true;
    Some(d)
}

/// Continental KR5V2X decoder: rtl_433's zero-bit slicer + `continental_car_remote_decode`.
pub struct ContinentalKr5v2xDecoder {
    rx: Receiver,
}

impl ContinentalKr5v2xDecoder {
    pub fn new() -> Self {
        Self { rx: Receiver::new(&DEVICE) }
    }
}

impl Default for ContinentalKr5v2xDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for ContinentalKr5v2xDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    /// Manchester half-bit / full bit; the slicer splits them at 1.5 half-bits (± 50 µs).
    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: SHORT_US,
            te_long: LONG_US,
            te_delta: SHORT_US / 2,
            min_count_bit: FRAME_BITS,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.rx.reset(&mut Decode(decode));
    }

    fn feed(&mut self, level: bool, duration_us: u32) -> Option<DecodedSignal> {
        self.rx.feed(&mut Decode(decode), level, duration_us)
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    /// rtl_433 decodes only.
    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }
}

/// rtl_433's JSON fields of a decode, rebuilt from KAT's (the field mapping, for the parity tests).
#[cfg(test)]
pub(super) fn rtl433_fields(d: &DecodedSignal) -> String {
    let extra = d.extra.unwrap_or(0);
    format!(
        "id={:08X} encrypted={:08X} sequence={} button_code={}",
        d.serial.unwrap_or(0),
        (extra >> 8) & 0xFFFF_FFFF,
        (d.data & 0xFFFF) << 8 | extra >> 40,
        d.button.unwrap_or(0),
    )
}

#[cfg(test)]
mod tests {
    use super::super::rtl433::test_util::*;
    use super::*;

    /// The 14 frame bytes for these fields, checksum included.
    fn frame(id: u32, button: u8, sequence: u32, encrypted: u32) -> [u8; 14] {
        let mut b = [0u8; 14];
        b[..4].copy_from_slice(&id.to_be_bytes());
        b[4] = 0x10;
        b[5] = button << 4;
        b[6..9].copy_from_slice(&sequence.to_be_bytes()[1..]);
        b[9..13].copy_from_slice(&encrypted.to_be_bytes());
        b[13] = xor_bytes(&b[..13]);
        b
    }

    /// Message bits like the recordings' repeat (`00 F0 F0 F0 6` + frame), after a wake-up of
    /// `wake` `0x87` bytes (the first message of a press).
    fn message(frame: &[u8; 14], wake: usize) -> Vec<bool> {
        let mut bytes = vec![0x00];
        bytes.extend(std::iter::repeat_n(0x87, wake));
        bytes.extend([0xF0, 0xF0, 0xF0, 0x60]);
        let head = bytes.len() * 8 - 4;
        let mut bits = bits_of_bytes(&bytes, head);
        bits.extend(bits_of_bytes(frame, 112));
        bits
    }

    fn air(frame: &[u8; 14], wake: usize) -> Vec<LevelDuration> {
        with_idle(
            &wire(&manchester_zerobit(&message(frame, wake), SHORT_US, 0)),
            100_000,
        )
    }

    #[test]
    fn decodes_frames_from_the_c_layout() {
        for (button, wake) in [
            (0x1, 0),
            (0x3, 3),
            (0x9, 0),
            (0xA, 125),
            (0xB, 140),
            (0x5, 0),
        ] {
            let f = frame(0xF42E_6789, button, 1522, 0x9391_771C);
            let got = decode_all(&mut ContinentalKr5v2xDecoder::new(), &air(&f, wake));
            assert_eq!(got.len(), 1, "button {button:#x}, wake-up {wake}");
            let d = &got[0];
            assert_eq!(
                (d.serial, d.button, d.counter, d.crc_valid),
                (Some(0xF42E_6789), Some(button), Some(1522), true)
            );
            assert_eq!((d.data_count_bit, d.encoder_capable), (112, false));
            let p = frame_signal(&f, 112);
            assert_eq!((p.data, p.extra), (d.data, d.extra));
            assert_eq!(
                rtl433_fields(d),
                format!("id=F42E6789 encrypted=9391771C sequence=1522 button_code={button}")
            );
        }
        // A 24-bit sequence: the counter keeps its low 16 bits, `extra` the rest.
        let d = &decode_all(
            &mut ContinentalKr5v2xDecoder::new(),
            &air(&frame(0x2A79_1780, 1, 0x12_D483, 0xC5B7_F6D2), 0),
        )[0];
        assert_eq!(
            (
                d.counter,
                rtl433_fields(d).contains(&format!("sequence={}", 0x12_D483))
            ),
            (Some(0xD483), true)
        );
        // The C's seven-F sanity constant: an all-ones ID passes, 0x0FFFFFFF does not.
        assert_eq!(
            decode_all(
                &mut ContinentalKr5v2xDecoder::new(),
                &air(&frame(u32::MAX, 1, 7, 1), 0)
            )
            .len(),
            1
        );
        assert!(decode_all(
            &mut ContinentalKr5v2xDecoder::new(),
            &air(&frame(0x0FFF_FFFF, 1, 7, 1), 0)
        )
        .is_empty());
    }

    #[test]
    fn rejects_bad_frames() {
        let good = frame(0xF42E_6789, 1, 1522, 0x9391_771C);
        let mut dec = ContinentalKr5v2xDecoder::new();
        assert_eq!(decode_all(&mut dec, &air(&good, 0)).len(), 1);
        // Checksum, sanity (zero ID / button / sequence, seven-F encrypted, all-ones sequence).
        let mut bad = good;
        bad[7] ^= 0x04;
        assert!(decode_all(&mut dec, &air(&bad, 0)).is_empty(), "checksum");
        for f in [
            frame(0, 1, 1522, 1),
            frame(0xF42E_6789, 0, 1522, 1),
            frame(0xF42E_6789, 1, 0, 1),
            frame(0xF42E_6789, 1, 1522, 0x0FFF_FFFF),
            frame(0xF42E_6789, 1, 0xFF_FFFF, 1),
        ] {
            assert!(decode_all(&mut dec, &air(&f, 0)).is_empty(), "{f:02X?}");
        }
        // Wrong preamble, a truncated frame, the wrong rate.
        let mut bits = message(&good, 0);
        bits[34] = !bits[34];
        assert!(
            decode_all(
                &mut dec,
                &with_idle(&wire(&manchester_zerobit(&bits, SHORT_US, 0)), 100_000)
            )
            .is_empty(),
            "preamble"
        );
        let bits = message(&good, 0);
        let short = &bits[..bits.len() - 3];
        assert!(
            decode_all(
                &mut dec,
                &with_idle(&wire(&manchester_zerobit(short, SHORT_US, 0)), 100_000)
            )
            .is_empty(),
            "truncated"
        );
        assert!(
            decode_all(
                &mut dec,
                &with_idle(&wire(&manchester_zerobit(&bits, 160, 0)), 100_000)
            )
            .is_empty(),
            "160 µs half-bits"
        );
        // A lower-tone stretch over the reset limit inside the frame ends the message there.
        let mut wire = wire(&manchester_zerobit(&bits, SHORT_US, 0));
        let k = (wire.len() * 3 / 4..).find(|&i| !wire[i].level).unwrap();
        wire[k].duration_us = 1_600;
        assert!(
            decode_all(&mut dec, &with_idle(&wire, 100_000)).is_empty(),
            "split by a gap"
        );
    }

    #[test]
    fn registry_reports_it_on_fm() {
        let f = frame(0x2A79_1780, 3, 54405, 0xB900_57D2);
        let got = registry(
            &air(&f, 125),
            313_800_000,
            Some(crate::capture::RfModulation::FM),
        );
        assert_eq!(
            got.iter().map(|(n, _)| n.as_str()).collect::<Vec<_>>(),
            [NAME]
        );
        assert_eq!(got[0].1.serial, Some(0x2A79_1780));
    }

    /// rtl_433_tests `continental/72147-SNA-A01`: the seven recordings decode through KAT's IQ
    /// pipeline with the fields of rtl_433's `.json` files.
    #[test]
    #[ignore = "IQ recordings, ~5 s in a debug build"]
    fn decodes_rtl433_recordings() {
        let json = |v: &serde_json::Value| {
            format!(
                "id={} encrypted={} sequence={} button_code={}",
                v["id"].as_str().unwrap(),
                v["encrypted"].as_str().unwrap(),
                v["sequence"],
                v["button_code"]
            )
        };
        if let Some(n) = check_recordings(&["continental"], NAME, NAME, rtl433_fields, json) {
            assert_eq!(n, 7);
        }
    }
}
