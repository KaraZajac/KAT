//! MIC 6SC2 car remote decoder (receive only).
//!
//! Port of rtl_433 `src/devices/mic6sc2_car_remote.c` (`six_sc_two_car_remote_decode`, r_device
//! "MIC 6SC2 Car Remote (-f 315.1M)", model `MIC6SC2-CarRemote`): 6SC2 CMGU / 6SC2 CDFA remotes
//! (an MSOP-10 chip, perhaps from MIC, Master Instrument Corporation), 315 MHz, up to 4 buttons, a
//! rolling code with an unencrypted sequence number.
//!
//! **Modulation** — OOK, rtl_433's Manchester zero-bit slicer (`OOK_PULSE_MANCHESTER_ZEROBIT`,
//! short 250 µs, reset limit 10 ms, no tolerance), see [super::rtl433]: carrier on is HIGH, a
//! falling data edge is `1`, a rising one `0`, every message starts with the slicer's hard-coded
//! `0` (the first bit of the `0x55` preamble), and a gap over 10 ms ends it.
//!
//! **Frame** — 80 bits, bytes "reflected" (each sent LSB first):
//!
//! | byte | field |
//! |------|-------|
//! | 0..1 | preamble `0x55 0x54` |
//! | 2..5 | encrypted (32 bits, `rev8(b5) rev8(b4) rev8(b3) rev8(b2)`) |
//! | 6    | button (`rev8(b6) & 0xF`), unknown (the other nibble) |
//! | 7..8 | sequence (16 bits, `rev8(b8) rev8(b7)`) |
//! | 9    | checksum: bytes 2..10 XOR to 0 (byte 10 holds the row's bits past 80, normally none) |
//!
//! **Gating** (the C's checks; a failing frame is dropped): the first row of ≥ 48 bits
//! (`bitbuffer_find_repeated_row(…, 1, 48)`, which counts the row itself), ≤ 88 bits, preamble
//! `55 54`, XOR (`mic: CHECKSUM`).
//!
//! **Field mapping** — no ID in the frame: `serial` = None. `button` = rtl_433 `button_code` (1
//! Unlock, 2 Lock, 3 Trunk, 4 Panic, others "?"), `counter` = `sequence` (16 bits), `crc_valid` =
//! true (the XOR passed); rtl_433's `encrypted` is in `data`. `data`/`extra` = the row (48..=88 bits,
//! normally 80: the first 64 bits in `data`, the rest right-aligned in `extra`), `data_count_bit` =
//! its length.
//!
//! Receive only (rtl_433 has no encoder). Tested on frames built from the C and on rtl_433_tests'
//! six recordings (`tests/6sc2/01`, `02`), which decode with rtl_433's fields.

use super::rtl433::{frame_signal, xor_bytes, BitBuffer, Decode, Device, Ends, Gaps, Modulation, Receiver};
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

const NAME: &str = "MIC6SC2-CarRemote";

/// Metadata (see `info.rs`): OOK, receive only.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &["MIC 6SC2 Car Remote"],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "MIC",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// rtl_433's `button_str`.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        1 => "Unlock",
        2 => "Lock",
        3 => "Trunk",
        4 => "Panic",
        _ => "Other",
    })
}

/// r_device timing: `short_width` (half-bit) and `reset_limit` (µs); no `long_width`.
const SHORT_US: u32 = 250;
const RESET_US: u32 = 10_000;
/// The C's minimum row length; frames are 80 bits.
const MIN_BITS: usize = 48;
const MAX_BITS: usize = 88;

/// The `r_device`. Its 10 ms reset limit ends a message wherever rtl_433's OOK package would end,
/// so the package ends are not modelled (nor the 1200-pulse cut); consecutive LOWs are one gap (see
/// `rtl433`).
const DEVICE: Device = Device {
    modulation: Modulation::OokManchesterZerobit,
    short_width: SHORT_US,
    long_width: 0,
    reset_limit: RESET_US,
    gap_limit: 0,
    sync_width: 0,
    tolerance: 0,
    gaps: Gaps::Merged,
    ends: Ends::None,
    resync: false,
};

/// `six_sc_two_car_remote_decode` on one message's bitbuffer.
fn decode(bits: &BitBuffer) -> Option<DecodedSignal> {
    let row = bits.find_repeated_row(1, MIN_BITS)?; // DECODE_ABORT_EARLY
    let len = bits.bits_per_row(row);
    if len > MAX_BITS {
        return None; // DECODE_ABORT_LENGTH
    }
    let b = bits.row(row);
    if b[0] != 0x55 || b[1] != 0x54 {
        return None; // DECODE_FAIL_SANITY
    }
    if xor_bytes(&b[2..11]) != 0 {
        return None; // DECODE_FAIL_MIC
    }
    let button = b[6].reverse_bits() & 0xF;
    let sequence = u16::from_be_bytes([b[8].reverse_bits(), b[7].reverse_bits()]);
    let mut d = frame_signal(b, len);
    d.button = Some(button);
    d.counter = Some(sequence);
    d.crc_valid = true;
    Some(d)
}

/// MIC 6SC2 decoder: rtl_433's zero-bit slicer + `six_sc_two_car_remote_decode`.
pub struct Mic6sc2Decoder {
    rx: Receiver,
}

impl Mic6sc2Decoder {
    pub fn new() -> Self {
        Self { rx: Receiver::new(&DEVICE) }
    }
}

impl Default for Mic6sc2Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for Mic6sc2Decoder {
    fn name(&self) -> &'static str {
        NAME
    }

    /// Manchester half-bit / full bit; the slicer splits them at 1.5 half-bits (± 125 µs).
    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: SHORT_US,
            te_long: 2 * SHORT_US,
            te_delta: SHORT_US / 2,
            min_count_bit: MIN_BITS,
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
    // Bytes 2..5 of the row (left-aligned: rows under 64 bits sit right-aligned in `data`).
    let b = (d.data << 64usize.saturating_sub(d.data_count_bit)).to_be_bytes();
    let encrypted = u32::from_be_bytes([
        b[5].reverse_bits(),
        b[4].reverse_bits(),
        b[3].reverse_bits(),
        b[2].reverse_bits(),
    ]);
    format!(
        "encrypted={encrypted:08X} button_code={} sequence={}",
        d.button.unwrap_or(0),
        d.counter.unwrap_or(0)
    )
}

#[cfg(test)]
mod tests {
    use super::super::rtl433::test_util::*;
    use super::*;

    /// The 10 frame bytes (wire order) for these fields, checksum included.
    fn frame(encrypted: u32, button: u8, unknown: u8, sequence: u16) -> [u8; 10] {
        let e = encrypted.to_be_bytes();
        let s = sequence.to_be_bytes();
        let mut b = [
            0x55,
            0x54,
            e[3],
            e[2],
            e[1],
            e[0],
            unknown << 4 | button,
            s[1],
            s[0],
            0,
        ];
        for x in &mut b[2..9] {
            *x = x.reverse_bits();
        }
        b[9] = xor_bytes(&b[2..9]);
        b
    }

    fn air(frame: &[u8], bits: usize) -> Vec<LevelDuration> {
        with_idle(&wire(&manchester_zerobit(&bits_of_bytes(frame, bits), SHORT_US, 0)), 20_000)
    }

    #[test]
    fn decodes_frames_from_the_c_layout() {
        for button in [1, 2, 3, 4, 9] {
            let f = frame(0x1234_ABCD, button, 0x6, 0xBEEF);
            let got = decode_all(&mut Mic6sc2Decoder::new(), &air(&f, 80));
            assert_eq!(got.len(), 1, "button {button}");
            let d = &got[0];
            assert_eq!(
                (d.serial, d.button, d.counter, d.crc_valid),
                (None, Some(button), Some(0xBEEF), true)
            );
            assert_eq!((d.data_count_bit, d.encoder_capable), (80, false));
            let p = frame_signal(&f, 80);
            assert_eq!((p.data, p.extra), (d.data, d.extra));
            assert_eq!(
                rtl433_fields(d),
                format!("encrypted=1234ABCD button_code={button} sequence=48879")
            );
        }
        // The frame repeated after more than 10 ms: one decode per frame.
        let f = frame(0x0BAD_F00D, 2, 0, 7);
        let mut wire = air(&f, 80);
        wire.extend(air(&f, 80));
        assert_eq!(decode_all(&mut Mic6sc2Decoder::new(), &wire).len(), 2);
    }

    #[test]
    fn rejects_bad_frames() {
        let good = frame(0x1234_ABCD, 1, 0, 0x0102);
        let mut dec = Mic6sc2Decoder::new();
        assert_eq!(decode_all(&mut dec, &air(&good, 80)).len(), 1);
        let mut bad = good;
        bad[4] ^= 0x01;
        assert!(decode_all(&mut dec, &air(&bad, 80)).is_empty(), "checksum");
        let mut bad = good;
        bad[1] = 0x56;
        assert!(decode_all(&mut dec, &air(&bad, 80)).is_empty(), "preamble");
        // Rows of 48..=88 bits only: trailing bits past 80 must be zero for the XOR (byte 10), and
        // two frames less than 10 ms apart are one too-long row.
        let mut long = good.to_vec();
        long.push(0x00);
        assert_eq!(
            decode_all(&mut dec, &air(&long, 88)).len(),
            1,
            "88 bits, byte 10 zero"
        );
        long[10] = 0x80;
        assert!(
            decode_all(&mut dec, &air(&long, 81)).is_empty(),
            "81st bit set"
        );
        let mut two = wire(&manchester_zerobit(&bits_of_bytes(&good, 80), SHORT_US, 0));
        two.push(LevelDuration::new(false, 9_000));
        two.extend(wire(&manchester_zerobit(&bits_of_bytes(&good, 80), SHORT_US, 0)));
        assert!(
            decode_all(&mut dec, &with_idle(&two, 20_000)).is_empty(),
            "9 ms apart"
        );
        assert!(decode_all(&mut dec, &air(&good, 47)).is_empty(), "47 bits");
        assert!(
            decode_all(
                &mut dec,
                &with_idle(&wire(&manchester_zerobit(&bits_of_bytes(&good, 80), 400, 0)), 20_000)
            )
            .is_empty(),
            "400 µs"
        );
    }

    /// rtl_433_tests `6sc2/01`, `02` (skipped when REFERENCES is absent, e.g. in CI): every
    /// recording decodes with the fields of rtl_433's `.json`.
    #[test]
    #[ignore = "IQ recordings"]
    fn decodes_rtl433_recordings() {
        let json = |v: &serde_json::Value| {
            format!(
                "encrypted={} button_code={} sequence={}",
                v["encrypted"].as_str().unwrap(),
                v["button_code"],
                v["sequence"]
            )
        };
        if let Some(n) = check_recordings(&["6sc2"], NAME, NAME, rtl433_fields, json) {
            assert!(n > 0);
        }
    }

    #[test]
    fn registry_reports_it_on_am() {
        let f = frame(0xCAFE_0001, 4, 0, 0x2222);
        let got = registry(
            &air(&f, 80),
            315_100_000,
            Some(crate::capture::RfModulation::AM),
        );
        assert_eq!(
            got.iter().map(|(n, _)| n.as_str()).collect::<Vec<_>>(),
            [NAME]
        );
        assert_eq!(got[0].1.button, Some(4));
    }
}
