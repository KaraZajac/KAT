//! Honda keyfob KR5V2X / KR5V1X decoder (receive only).
//!
//! Port of rtl_433 `src/devices/continental_car_remote.c` (`honda_keyfob_decode`, r_device "Honda
//! Keyfob KR5V2X/1X (-f 433.6M -s 1024k)", model `Honda-KR5V2X1X`): Honda keyfobs FCC ID KR5V2X
//! (313.55 / 314.15 MHz) and KR5V1X (433.66 / 434.18 MHz). rtl_433 added it (#3578, from PR #2767)
//! as an alternate decoding of the same physical remotes as `Continental-KR5V2X`; the two disagree
//! on bit layout and checksum.
//!
//! **Modulation** — FSK, rtl_433's Manchester zero-bit slicer (`FSK_PULSE_MANCHESTER_ZEROBIT`,
//! short 60 µs, long 120 µs, reset limit 1500 µs, no tolerance), see [super::rtl433].
//!
//! **Frame** — the decoder needs exactly one row of 150..=184 bits (a message: from the start of the
//! carrier or a lower-tone stretch over 1.5 ms to the next), finds the 24-bit manufacturer code
//! `EC 0F 62` and reads 120 bits from its last byte on (15 bytes, MSB first):
//!
//! | byte | field |
//! |------|-------|
//! | 0    | `0x62`, the manufacturer code's last byte |
//! | 1    | packet index (a press sends the packet twice, `0x08` then `0x0A`; not output by rtl_433) |
//! | 2..5 | device ID (32 bits) |
//! | 6    | event (command) |
//! | 7..9 | counter (24 bits) |
//! | 10..13 | rolling code (32 bits) |
//! | 14   | CRC-8 of bytes 0..13, OpenSafety polynomial 0x2F, init 0 |
//!
//! **Gating** (the C's checks; a failing frame is dropped): one row, 150..=184 bits, the code found
//! with 136 bits from its start in the row, and the CRC (`mic: CRC`).
//!
//! **Field mapping** — `serial` = device ID (rtl_433 `id`, `%08x`), `button` = the event byte
//! (`event`: 0x21 Lock, 0x22 Unlock, 0x24 Trunk, 0x27 Emergency, 0x2D RemoteStart, others "?"),
//! `counter` = the low 16 bits of the 24-bit `counter`, `crc_valid` = true (the CRC passed).
//! `data` = bytes 0..7, `extra` = bytes 8..14 (56 bits), `data_count_bit` = 120: rtl_433's `code` =
//! `(extra >> 8) & 0xFFFFFFFF`, its counter = `(data & 0xFF) << 16 | extra >> 40`.
//!
//! Receive only (rtl_433 has no encoder). No recording exists (rtl_433_tests has none, and the
//! decoder reads none of the 100 recordings in REFERENCES): the tests use frames built from the C.

use super::common::crc8;
use super::rtl433::{frame_signal, BitBuffer, Decode, Device, Ends, Gaps, Modulation, Receiver};
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

const NAME: &str = "Honda-KR5V2X1X";

/// Metadata (see `info.rs`): FSK of unknown deviation (no recording) → FM476, receive only.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &["Honda Keyfob KR5V2X/1X"],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::FM,
    encryption: "Rolling Code",
    make: "Honda/Acura",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&FM476],
};

/// rtl_433's `event` names.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x21 => "Lock",
        0x22 => "Unlock",
        0x24 => "Trunk",
        0x27 => "Emergency",
        0x2D => "RemoteStart",
        _ => "Other",
    })
}

/// r_device timing: `short_width` (half-bit), `long_width`, `reset_limit` (µs).
const SHORT_US: u32 = 60;
const LONG_US: u32 = 120;
const RESET_US: u32 = 1500;
const FRAME_BITS: usize = 120;

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

/// `honda_keyfob_decode` on one message's bitbuffer.
fn decode(bits: &BitBuffer) -> Option<DecodedSignal> {
    if bits.num_rows() > 1 {
        return None; // DECODE_ABORT_EARLY
    }
    let len = bits.bits_per_row(0);
    if !(150..=184).contains(&len) {
        return None; // DECODE_ABORT_LENGTH
    }
    let offset = bits.search(0, 0, &[0xEC, 0x0F, 0x62], 24);
    if offset + 16 + FRAME_BITS > len {
        return None; // DECODE_ABORT_EARLY
    }
    let mut b = [0u8; 15];
    bits.extract_bytes(0, offset + 16, &mut b, FRAME_BITS);
    if crc8(&b[..14], 0x2F, 0x00) != b[14] {
        return None; // DECODE_FAIL_MIC
    }
    let device_id = u32::from_be_bytes([b[2], b[3], b[4], b[5]]);
    let counter = u32::from_be_bytes([0, b[7], b[8], b[9]]);
    let mut d = frame_signal(&b, FRAME_BITS);
    d.serial = Some(device_id);
    d.button = Some(b[6]);
    d.counter = Some(counter as u16);
    d.crc_valid = true;
    Some(d)
}

/// Honda KR5V2X/1X decoder: rtl_433's zero-bit slicer + `honda_keyfob_decode`.
pub struct HondaKr5v2x1xDecoder {
    rx: Receiver,
}

impl HondaKr5v2x1xDecoder {
    pub fn new() -> Self {
        Self { rx: Receiver::new(&DEVICE) }
    }
}

impl Default for HondaKr5v2x1xDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for HondaKr5v2x1xDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    /// Manchester half-bit / full bit; the slicer splits them at 1.5 half-bits (± 30 µs).
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
    let event = match d.button.and_then(|b| native_buttons(NAME, b)) {
        Some("Other") | None => "?",
        Some(name) => name,
    };
    format!(
        "id={:08x} event={event} counter={:06x} code={:08x}",
        d.serial.unwrap_or(0),
        (d.data & 0xFF) << 16 | extra >> 40,
        (extra >> 8) & 0xFFFF_FFFF,
    )
}

#[cfg(test)]
mod tests {
    use super::super::rtl433::test_util::*;
    use super::*;

    /// The 15 bytes from the manufacturer code's last byte on, CRC included.
    fn frame(index: u8, id: u32, event: u8, counter: u32, code: u32) -> [u8; 15] {
        let mut b = [0u8; 15];
        b[0] = 0x62;
        b[1] = index;
        b[2..6].copy_from_slice(&id.to_be_bytes());
        b[6] = event;
        b[7..10].copy_from_slice(&counter.to_be_bytes()[1..]);
        b[10..14].copy_from_slice(&code.to_be_bytes());
        b[14] = crc8(&b[..14], 0x2F, 0x00);
        b
    }

    /// A message: the hard-coded 0, `pre` alternating preamble bits, `EC 0F` and the frame.
    fn message(frame: &[u8; 15], pre: usize) -> Vec<bool> {
        let mut bits = vec![false];
        bits.extend((0..pre).map(|i| i % 2 == 0));
        bits.extend(bits_of_bytes(&[0xEC, 0x0F], 16));
        bits.extend(bits_of_bytes(frame, 120));
        bits
    }

    fn air(frame: &[u8; 15], pre: usize) -> Vec<LevelDuration> {
        with_idle(&wire(&manchester_zerobit(&message(frame, pre), SHORT_US, 0)), 50_000)
    }

    #[test]
    fn decodes_frames_from_the_c_layout() {
        for (event, pre) in [
            (0x21, 13),
            (0x22, 20),
            (0x24, 47),
            (0x27, 30),
            (0x2D, 13),
            (0x55, 13),
        ] {
            let f = frame(0x08, 0x8E12_A7C4, event, 0x01_02F3, 0xDEAD_BEEF);
            let got = decode_all(&mut HondaKr5v2x1xDecoder::new(), &air(&f, pre));
            assert_eq!(got.len(), 1, "event {event:#x}, preamble {pre}");
            let d = &got[0];
            assert_eq!(
                (d.serial, d.button, d.counter, d.crc_valid),
                (Some(0x8E12_A7C4), Some(event), Some(0x02F3), true)
            );
            assert_eq!((d.data_count_bit, d.encoder_capable), (120, false));
            let p = frame_signal(&f, 120);
            assert_eq!((p.data, p.extra), (d.data, d.extra));
            let name = if event == 0x55 {
                "?"
            } else {
                native_buttons(NAME, event).unwrap()
            };
            assert_eq!(
                rtl433_fields(d),
                format!("id=8e12a7c4 event={name} counter=0102f3 code=deadbeef")
            );
        }
    }

    #[test]
    fn rejects_bad_frames() {
        let good = frame(0x0A, 0x0123_4567, 0x22, 77, 0x1357_9BDF);
        let mut dec = HondaKr5v2x1xDecoder::new();
        assert_eq!(decode_all(&mut dec, &air(&good, 13)).len(), 1);
        let mut bad = good;
        bad[9] ^= 0x10;
        assert!(decode_all(&mut dec, &air(&bad, 13)).is_empty(), "CRC");
        // Row length: 150..=184 bits (13 → 150, 47 → 184).
        assert!(decode_all(&mut dec, &air(&good, 12)).is_empty(), "149 bits");
        assert!(decode_all(&mut dec, &air(&good, 48)).is_empty(), "185 bits");
        // Manufacturer code, timing.
        let mut bits = message(&good, 13);
        bits[20] = !bits[20];
        assert!(
            decode_all(
                &mut dec,
                &with_idle(&wire(&manchester_zerobit(&bits, SHORT_US, 0)), 50_000)
            )
            .is_empty(),
            "code"
        );
        assert!(
            decode_all(
                &mut dec,
                &with_idle(&wire(&manchester_zerobit(&message(&good, 13), 100, 0)), 50_000)
            )
            .is_empty(),
            "100 µs"
        );
    }

    /// rtl_433_tests has no recording of this decoder; the Continental recordings are the same FCC
    /// ID, and rtl_433 (`-R 327`) decodes none of them as Honda-KR5V2X1X: neither does KAT.
    #[test]
    #[ignore = "IQ recordings, ~5 s in a debug build"]
    fn rtl433_recordings_as_rtl433() {
        let json = |v: &serde_json::Value| {
            format!(
                "id={:08x} event={} counter={:06x} code={:08x}",
                v["id"].as_i64().unwrap() & 0xFFFF_FFFF,
                v["event"].as_str().unwrap(),
                v["counter"].as_i64().unwrap(),
                v["code"].as_i64().unwrap() & 0xFFFF_FFFF
            )
        };
        if let Some(n) = check_recordings(&["continental"], NAME, NAME, rtl433_fields, json) {
            assert_eq!(n, 0);
        }
    }

    #[test]
    fn registry_reports_it_on_fm() {
        let f = frame(0x08, 0x0BAD_CAFE, 0x21, 4242, 0x0F0F_A5A5);
        let got = registry(
            &air(&f, 30),
            433_660_000,
            Some(crate::capture::RfModulation::FM),
        );
        assert_eq!(
            got.iter().map(|(n, _)| n.as_str()).collect::<Vec<_>>(),
            [NAME]
        );
        assert_eq!(got[0].1.serial, Some(0x0BAD_CAFE));
    }
}
