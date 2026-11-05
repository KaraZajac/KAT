//! Siemens 5WY72XX car remote (receive only).
//!
//! Port of rtl_433 `src/devices/siemens_5wy72xx.c` (`r_device` "Siemens 5WY72XX Car Remote",
//! model `Siemens-5WY72XX`; FCC ID M3N5WY72XX, the Siemens OEM remote of DaimlerChrysler's SKREEK
//! CS and RS platforms, 315 MHz). No Flipper reference has it.
//!
//! **Modulation** — `OOK_PULSE_MANCHESTER_ZEROBIT`: short_width 220 µs, reset_limit 10 000 µs, no
//! tolerance (no pulse is rejected); a gap > 10 ms ends the message.
//!
//! **Frame** (`siemens_5wy72xx_car_remote_decode`) — one row of at least 113 bits. The 96-bit
//! payload follows the first `0x6001` (16 bits) found in the row (`bitbuffer_search`; when there is
//! none, the bits past the row are read — zeros — and the sum check fails): `IIIIIIII bb SSSS
//! EEEEEEEE CC` as bytes b0..b11 — b0..b3 the ID (little endian: rtl_433 prints b3 b2 b1 b0),
//! b4 the button flags, b5 b6 the 16-bit sequence (big endian), b7..b10 the encrypted part
//! (printed b10..b7), b11 the XOR check. Checks: the 12 bytes do not sum to 0 or 12 × 0xFF
//! (`DECODE_FAIL_SANITY`); their XOR is 0 (`DECODE_FAIL_MIC`). Button flags: 0x01 Lock, 0x02
//! Unlock, 0x04 Trunk, 0x08 Panic, 0x10 Left Door, 0x20 Right Door, joined with "; " (none: "").
//!
//! **Fields** — `serial` = the ID (b3 b2 b1 b0 as rtl_433 prints it), `button` = b4, `counter` =
//! the sequence. `data` = payload bytes b0..b7 (the first 64 payload bits), `extra` = b8..b11,
//! `data_count_bit` = 96; rtl_433's `encrypted` field is b10 b9 b8 b7 of them. `crc_valid` = true
//! (the XOR check passed).
//!
//! **Encoder** — receive only, as rtl_433.

use super::rtl433::{button_table, frame_signal, join_names, BitBuffer, DecodeFn, Device, Ends, Gaps, Modulation, Receiver};
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

const NAME: &str = "Siemens-5WY72XX";

/// Metadata (see `info.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &["Siemens 5WY72XX Car Remote"],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "Chrysler/Dodge",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// The `r_device`.
const DEVICE: Device = Device {
    modulation: Modulation::OokManchesterZerobit,
    short_width: 220,
    long_width: 0,
    reset_limit: 10_000,
    gap_limit: 0,
    sync_width: 0,
    tolerance: 0,
    gaps: Gaps::AtOnce,
    ends: Ends::Ook,
    resync: false,
};

/// rtl_433's button flag names (bit 0 first).
const BUTTONS: [&str; 6] = ["Lock", "Unlock", "Trunk", "Panic", "Left Door", "Right Door"];

/// rtl_433's `button_str`.
fn button_str(code: u8) -> String {
    join_names(BUTTONS.iter().enumerate().filter(|(i, _)| code >> i & 1 == 1).map(|(_, n)| *n), "")
}

fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    static NAMES: std::sync::OnceLock<Vec<&'static str>> = std::sync::OnceLock::new();
    Some(button_table(&NAMES, button_str)[code as usize])
}

/// `siemens_5wy72xx_car_remote_decode`.
#[derive(Debug, Default)]
struct Siemens;

impl DecodeFn for Siemens {
    fn message(&mut self, bits: &mut BitBuffer) -> Option<DecodedSignal> {
        if bits.bits_per_row(0) < 113 || bits.num_rows() > 1 {
            return None; // DECODE_ABORT_LENGTH
        }
        let offset = bits.search(0, 0, &[0x60, 0x01], 16) + 16;
        let mut b = [0u8; 12];
        bits.extract_bytes(0, offset, &mut b, 96);
        let sum: u32 = b.iter().map(|&x| u32::from(x)).sum();
        if sum == 0 || sum == 0xff * 12 {
            return None; // DECODE_FAIL_SANITY
        }
        if b.iter().fold(0u8, |a, &x| a ^ x) != 0 {
            return None; // DECODE_FAIL_MIC
        }
        let mut d = frame_signal(&b, 96);
        d.serial = Some(u32::from_le_bytes([b[0], b[1], b[2], b[3]]));
        d.button = Some(b[4]);
        d.counter = Some(u16::from_be_bytes([b[5], b[6]]));
        d.crc_valid = true;
        Some(d)
    }
}

/// Siemens 5WY72XX decoder (receive only).
pub struct Siemens5wy72xxDecoder {
    rx: Receiver,
    decode: Siemens,
}

impl Siemens5wy72xxDecoder {
    pub fn new() -> Self {
        Self { rx: Receiver::new(&DEVICE), decode: Siemens }
    }
}

impl Default for Siemens5wy72xxDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for Siemens5wy72xxDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: 220, te_long: 440, te_delta: 0, min_count_bit: 113 }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.rx.reset(&mut self.decode);
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        self.rx.feed(&mut self.decode, level, duration)
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    /// Receive only, as rtl_433.
    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }
}

/// rtl_433's JSON for a decode (`data_print_jsons`, no `time`), for the oracle comparisons.
#[cfg(test)]
pub(super) fn rtl433_json(d: &DecodedSignal) -> String {
    let btn = d.button.unwrap_or(0);
    let extra = d.extra.unwrap_or(0);
    // b7 = data's low byte, b8..b10 = extra's top three bytes
    let encrypted = (extra >> 8 & 0xFF) << 24 | (extra >> 16 & 0xFF) << 16 | (extra >> 24 & 0xFF) << 8 | (d.data & 0xFF);
    format!(
        "{{\"model\":\"{NAME}\",\"id\":\"{:08X}\",\"encrypted\":\"{encrypted:08X}\",\"button_code\":{btn},\"button_str\":\"{}\",\"sequence\":{},\"mic\":\"CHECKSUM\"}}",
        d.serial.unwrap_or(0),
        button_str(btn),
        d.counter.unwrap_or(0)
    )
}

#[cfg(test)]
mod tests {
    use super::super::rtl433::test_util::*;
    use super::*;

    /// Payload bytes b0..b11 for an ID, button, sequence and encrypted word (XOR byte computed).
    fn payload(id: u32, button: u8, seq: u16, enc: u32) -> [u8; 12] {
        let i = id.to_le_bytes();
        let e = enc.to_le_bytes();
        let mut b = [i[0], i[1], i[2], i[3], button, (seq >> 8) as u8, seq as u8, e[0], e[1], e[2], e[3], 0];
        b[11] = b[..11].iter().fold(0, |a, &x| a ^ x);
        b
    }

    /// The zerobit row: the hard-coded 0, `lead` filler bits, 0x6001, the payload, `tail` bits.
    fn row(lead: &[bool], b: &[u8; 12], tail: &[bool]) -> Vec<bool> {
        let mut r = vec![false];
        r.extend_from_slice(lead);
        r.extend(bits_of(0x6001, 16));
        r.extend(bits_of_bytes(b, 96));
        r.extend_from_slice(tail);
        r
    }

    fn decode(r: &[bool]) -> Vec<DecodedSignal> {
        feed_pairs(&mut Siemens5wy72xxDecoder::new(), &manchester_zerobit(r, 220, 20_000))
    }

    #[test]
    fn decodes_the_recorded_trunk_frame() {
        // g001: ID 8D221B7A, encrypted D116471E, Trunk, sequence 977
        let got = decode(&row(&[true, false, true, false], &payload(0x8D22_1B7A, 4, 977, 0xD116_471E), &[true]));
        assert_eq!(got.len(), 1);
        let d = &got[0];
        assert_eq!((d.serial, d.button, d.counter, d.crc_valid), (Some(0x8D22_1B7A), Some(4), Some(977), true));
        assert_eq!((d.data, d.extra, d.data_count_bit), (0x7A1B_228D_0403_D11E, Some(0x4716_D186), 96));
        assert_eq!(
            rtl433_json(d),
            r#"{"model":"Siemens-5WY72XX","id":"8D221B7A","encrypted":"D116471E","button_code":4,"button_str":"Trunk","sequence":977,"mic":"CHECKSUM"}"#
        );
    }

    #[test]
    fn decodes_every_button_flag() {
        for button in [1u8, 2, 3, 4, 7, 8, 0x10, 0x20, 0x3F, 0x40, 0] {
            let got = decode(&row(&[], &payload(0x0102_0304, button, 0xBEEF, 0x5555_AAAA), &[]));
            assert_eq!(got.len(), 1, "{button:#x}");
            assert_eq!(got[0].button, Some(button));
        }
        assert_eq!(button_str(7), "Lock; Unlock; Trunk");
        assert_eq!(button_str(0x30), "Left Door; Right Door");
        assert_eq!(button_str(0x40), "");
        assert_eq!(native_buttons(NAME, 3), Some("Lock; Unlock"));
    }

    #[test]
    fn rejects_failed_checks() {
        let good = payload(0x8D22_1B7A, 4, 977, 0xD116_471E);
        // XOR check
        let mut bad = good;
        bad[6] ^= 0x10;
        assert!(decode(&row(&[], &bad, &[])).is_empty());
        // all-zero / all-FF payloads (XOR 0) fail the sum check
        assert!(decode(&row(&[], &[0; 12], &[])).is_empty());
        assert!(decode(&row(&[], &[0xFF; 12], &[])).is_empty());
        // 112 bits
        let r = row(&[], &good, &[]);
        assert!(decode(&r[..112]).is_empty());
        // no 0x6001 pattern: the payload is read past the row (zeros) and fails
        let mut r = row(&[], &good, &[]);
        r[2] = false;
        assert!(decode(&r).is_empty());
        // a gap over 10 ms inside splits the message
        let mut pairs = manchester_zerobit(&row(&[], &good, &[]), 220, 20_000);
        pairs[30].1 = 10_001;
        assert!(feed_pairs(&mut Siemens5wy72xxDecoder::new(), &pairs).is_empty());
    }

    /// rtl_433_tests `siemens/5WY72XX/01`: KAT's registry decodes of the six recordings equal
    /// rtl_433's events.
    #[test]
    #[ignore]
    fn decodes_the_rtl_433_recordings() {
        let Some(root) = rtl433_tests() else {
            eprintln!("Skip: rtl_433_tests not found");
            return;
        };
        let dir = root.join("siemens/5WY72XX/01");
        for f in 1..=6 {
            let path = dir.join(format!("g00{f}_315.1M_250k.cu8"));
            let got: Vec<serde_json::Value> = registry_decodes(&path)
                .iter()
                .map(|(n, d)| {
                    assert_eq!(n, NAME, "g00{f}");
                    serde_json::from_str(&rtl433_json(d)).unwrap()
                })
                .collect();
            assert_eq!(distinct(&got), distinct(&expected_json(&path)), "g00{f}");
        }
    }

    /// Fuzz dump for the C harness (see `rtl433::test_util::dump_fuzz_trains`).
    #[test]
    #[ignore]
    fn fuzz_dump() {
        let interesting = [1, 219, 220, 221, 329, 330, 331, 440, 9999, 10_000, 10_001];
        dump_fuzz_trains(
            "siemens",
            20_000,
            |rng, i| {
                let mut b = payload(rng.next() as u32, rng.below(256) as u8, rng.next() as u16, rng.next() as u32);
                if rng.chance(10) {
                    b = [0; 12];
                }
                if rng.chance(15) {
                    b[rng.below(12) as usize] ^= 1 << rng.below(8);
                }
                let lead: Vec<bool> = (0..rng.below(20)).map(|_| rng.chance(50)).collect();
                let tail: Vec<bool> = (0..rng.below(6)).map(|_| rng.chance(50)).collect();
                let mut r = row(&lead, &b, &tail);
                if rng.chance(10) {
                    let k = 1 + rng.below(r.len() as u64 - 1) as usize;
                    r[k] = !r[k];
                }
                let half = rng.jitter(220, if i % 3 == 0 { 60 } else { 15 });
                let mut pairs = manchester_zerobit(&r, half, 12_000);
                if rng.chance(30) {
                    let again = pairs.clone();
                    pairs.extend(again);
                }
                for p in pairs.iter_mut() {
                    *p = (rng.jitter(p.0, 12), rng.jitter(p.1, 12));
                }
                if rng.chance(60) {
                    mutate(rng, &mut pairs, &interesting);
                }
                pairs
            },
            || Box::new(Siemens5wy72xxDecoder::new()),
            rtl433_json,
        );
    }
}
