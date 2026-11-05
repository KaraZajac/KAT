//! Astrostart 2000 / 3000 remote-starter remote (receive only).
//!
//! Port of rtl_433 `src/devices/astrostart_2000.c` (`r_device` "Astrostart 2000 Car Remote",
//! model `Astrostart-2000`; Astroflex, FCC ID J5F-TX2000, 372.4 MHz). No Flipper reference has it.
//!
//! **Modulation** — `OOK_PULSE_PPM`: short_width 326 µs (gap → 0), long_width 526 µs (gap → 1),
//! reset_limit = gap_limit = 541 µs, tolerance 80 µs: a gap in 247–405 µs is a 0, 447–605 µs a 1
//! (exclusive bounds, as the slicer's), any other gap below 541 µs starts a new row, and a gap of
//! 541 µs or more ends the message (after adding its 1 bit when it is also below 606 µs, as in the
//! C). The pulses (~100 µs) only carry the gaps.
//!
//! **Frame** (`astrostart_2000_decode`) — exactly one row of exactly 52 bits: `B X IIII c`: byte 0
//! the button code, byte 1 its inverse, bytes 2–5 the 32-bit ID, then a 4-bit checksum of the ID
//! (both nibbles of bytes 2–5 added mod 16). Checks (`DECODE_FAIL_MIC`): byte 0 = !byte 1, the
//! checksum. Buttons: rtl_433's map (Lock 2B 03 27 0F 35 37; Panic 1F 35 0D 25 15 2D; Start 13 37 2D
//! 33 3D 3B; Stop 2F 0D 33 03 1D 17; Trunk 23 25 3D 1D 27 07; Unlock 0B 15 3B 17 07 0F; Multiple
//! 3F), names joined with "; ", anything else "?". The remote sends a primary and a secondary ID
//! (the recordings: 3410DE88 and 0410DE88).
//!
//! **Fields** — `serial` = the ID, `button` = byte 0, no counter (fixed code). `data` = the 52 row
//! bits right-aligned (`data_count_bit` 52). `crc_valid` = true (inverse byte + checksum passed).
//!
//! **Encoder** — receive only, as rtl_433.

use super::rtl433::{button_table, frame_signal, join_names, BitBuffer, DecodeFn, Device, Ends, Gaps, Modulation, Receiver};
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

const NAME: &str = "Astrostart-2000";

/// Metadata (see `info.rs`). Gap-width PPM; like GM, the closest line-coding label is PWM.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &["Astrostart 2000 Car Remote"],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Astroflex",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// The `r_device`.
const DEVICE: Device = Device {
    modulation: Modulation::OokPpm,
    short_width: 326,
    long_width: 526,
    reset_limit: 541,
    gap_limit: 541,
    sync_width: 0,
    tolerance: 80,
    gaps: Gaps::AtOnce,
    ends: Ends::Ook,
    resync: false,
};

/// rtl_433's `button_map`.
const BUTTON_MAP: [(&str, &[u8]); 7] = [
    ("Lock", &[0x2b, 0x03, 0x27, 0x0f, 0x35, 0x37]),
    ("Panic", &[0x1f, 0x35, 0x0d, 0x25, 0x15, 0x2d]),
    ("Start", &[0x13, 0x37, 0x2d, 0x33, 0x3d, 0x3b]),
    ("Stop", &[0x2f, 0x0d, 0x33, 0x03, 0x1d, 0x17]),
    ("Trunk", &[0x23, 0x25, 0x3d, 0x1d, 0x27, 0x07]),
    ("Unlock", &[0x0b, 0x15, 0x3b, 0x17, 0x07, 0x0f]),
    ("Multiple", &[0x3F]),
];

/// rtl_433's `button_str`.
fn button_str(code: u8) -> String {
    join_names(BUTTON_MAP.iter().filter(|(_, v)| v.contains(&code)).map(|(n, _)| *n), "?")
}

fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    static NAMES: std::sync::OnceLock<Vec<&'static str>> = std::sync::OnceLock::new();
    Some(button_table(&NAMES, button_str)[code as usize])
}

/// The ID checksum: both nibbles of bytes 2..5 added mod 16.
fn checksum(b: &[u8]) -> u8 {
    b[2..6].iter().fold(0u8, |c, &x| ((c + (x >> 4)) & 0xf).wrapping_add(x) & 0xf)
}

/// `astrostart_2000_decode`.
#[derive(Debug, Default)]
struct Astrostart;

impl DecodeFn for Astrostart {
    fn message(&mut self, bits: &mut BitBuffer) -> Option<DecodedSignal> {
        if bits.bits_per_row(0) != 52 {
            return None; // DECODE_ABORT_LENGTH
        }
        if bits.num_rows() != 1 {
            return None; // DECODE_ABORT_EARLY
        }
        let b: [u8; 7] = bits.bytes(0);
        if b[0] != !b[1] {
            return None; // DECODE_FAIL_MIC
        }
        if b[6] >> 4 != checksum(&b) {
            return None; // DECODE_FAIL_MIC
        }
        let mut d = frame_signal(&b, 52);
        d.serial = Some(u32::from_be_bytes([b[2], b[3], b[4], b[5]]));
        d.button = Some(b[0]);
        d.crc_valid = true;
        Some(d)
    }
}

/// Astrostart 2000 decoder (receive only).
pub struct Astrostart2000Decoder {
    rx: Receiver,
    decode: Astrostart,
}

impl Astrostart2000Decoder {
    pub fn new() -> Self {
        Self { rx: Receiver::new(&DEVICE), decode: Astrostart }
    }
}

impl Default for Astrostart2000Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for Astrostart2000Decoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: 326, te_long: 526, te_delta: 80, min_count_bit: 52 }
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
    format!(
        "{{\"model\":\"{NAME}\",\"id\":\"{:08X}\",\"button_code\":{btn},\"button_str\":\"{}\",\"mic\":\"CHECKSUM\"}}",
        d.serial.unwrap_or(0),
        button_str(btn)
    )
}

#[cfg(test)]
mod tests {
    use super::super::rtl433::test_util::*;
    use super::*;

    /// The 52 frame bits of a button code and ID (checksum computed).
    fn frame(button: u8, id: u32) -> Vec<bool> {
        let i = id.to_be_bytes();
        let mut b = [button, !button, i[0], i[1], i[2], i[3], 0];
        b[6] = checksum(&b) << 4;
        bits_of_bytes(&b, 52)
    }

    /// PPM: a 100 µs pulse before every bit's gap (326 / 526 µs), then one pulse ending in `end`.
    fn ppm(bits: &[bool], zero: u32, one: u32, end: u32) -> Vec<(u32, u32)> {
        let mut pairs: Vec<(u32, u32)> = bits.iter().map(|&b| (100, if b { one } else { zero })).collect();
        pairs.push((100, end));
        pairs
    }

    fn decode(pairs: &[(u32, u32)]) -> Vec<DecodedSignal> {
        feed_pairs(&mut Astrostart2000Decoder::new(), pairs)
    }

    #[test]
    fn decodes_the_recorded_lock_frame() {
        let got = decode(&ppm(&frame(0x2B, 0x3410_DE88), 300, 500, 10_000));
        assert_eq!(got.len(), 1);
        let d = &got[0];
        assert_eq!((d.serial, d.button, d.counter, d.crc_valid), (Some(0x3410_DE88), Some(0x2B), None, true));
        assert_eq!((d.data, d.data_count_bit), (0x2_BD43_410D_E883, 52));
        assert_eq!(rtl433_json(d), r#"{"model":"Astrostart-2000","id":"3410DE88","button_code":43,"button_str":"Lock","mic":"CHECKSUM"}"#);
    }

    #[test]
    fn decodes_every_mapped_button() {
        for (name, codes) in BUTTON_MAP {
            for &code in codes {
                let got = decode(&ppm(&frame(code, 0x0410_DE88), 326, 526, 20_000));
                assert_eq!(got.iter().map(|d| d.button).collect::<Vec<_>>(), [Some(code)], "{name} {code:#x}");
                assert!(button_str(code).contains(name));
            }
        }
        assert_eq!(button_str(0x35), "Lock; Panic");
        assert_eq!(button_str(0x00), "?");
        assert_eq!(native_buttons(NAME, 0x2F), Some("Stop"));
    }

    #[test]
    fn gap_windows_are_the_slicers() {
        // (checksum 3: the 52nd bit is a 1)
        let bits = frame(0x13, 0x3410_DE88);
        // 0 = 247..405 µs, 1 = 447..605 µs (exclusive bounds of 326±80 / 526±80)
        assert_eq!(decode(&ppm(&bits, 247, 447, 10_000)).len(), 1);
        assert_eq!(decode(&ppm(&bits, 405, 540, 10_000)).len(), 1);
        assert!(decode(&ppm(&bits, 246, 526, 10_000)).is_empty());
        assert!(decode(&ppm(&bits, 326, 446, 10_000)).is_empty());
        // a 1-gap of 541..605 µs is a 1 AND the end of the message
        let mut pairs = ppm(&bits, 326, 526, 10_000);
        pairs[51].1 = 560;
        assert_eq!(decode(&pairs).len(), 1, "the 52nd bit ends the message");
        pairs[20].1 = 560;
        assert!(decode(&pairs).is_empty(), "an early 541..605 gap cuts the frame");
    }

    #[test]
    fn rejects_failed_checks() {
        let mut bits = frame(0x2B, 0x3410_DE88);
        bits[9] = !bits[9]; // inverse byte
        assert!(decode(&ppm(&bits, 326, 526, 10_000)).is_empty());
        let mut bits = frame(0x2B, 0x3410_DE88);
        bits[30] = !bits[30]; // ID: checksum
        assert!(decode(&ppm(&bits, 326, 526, 10_000)).is_empty());
        let bits = frame(0x2B, 0x3410_DE88);
        assert!(decode(&ppm(&bits[..51], 326, 526, 10_000)).is_empty());
        // a 200 µs gap (no symbol) starts a new row: two rows are rejected
        let mut pairs = ppm(&bits, 326, 526, 10_000);
        pairs.insert(0, (100, 200));
        assert!(decode(&pairs).is_empty());
    }

    /// rtl_433_tests `astroflex/astrostart_2000/01`: KAT's registry decodes of the five recordings
    /// equal rtl_433's events.
    #[test]
    #[ignore]
    fn decodes_the_rtl_433_recordings() {
        let Some(root) = rtl433_tests() else {
            eprintln!("Skip: rtl_433_tests not found");
            return;
        };
        let dir = root.join("astroflex/astrostart_2000/01");
        for f in ["lock", "lock_secondary", "panic", "start", "stop"] {
            let path = dir.join(format!("{f}_372.4M_250k.cu8"));
            let got: Vec<serde_json::Value> = registry_decodes(&path)
                .iter()
                .map(|(n, d)| {
                    assert_eq!(n, NAME, "{f}");
                    serde_json::from_str(&rtl433_json(d)).unwrap()
                })
                .collect();
            assert_eq!(distinct(&got), distinct(&expected_json(&path)), "{f}");
        }
    }

    /// Fuzz dump for the C harness (see `rtl433::test_util::dump_fuzz_trains`).
    #[test]
    #[ignore]
    fn fuzz_dump() {
        let interesting = [1, 246, 247, 248, 405, 406, 446, 447, 540, 541, 542, 605, 606, 607];
        dump_fuzz_trains(
            "astrostart",
            20_000,
            |rng, i| {
                let button = if rng.chance(70) {
                    let (_, codes) = BUTTON_MAP[rng.below(7) as usize];
                    codes[rng.below(codes.len() as u64) as usize]
                } else {
                    rng.below(256) as u8
                };
                let mut bits = frame(button, rng.next() as u32);
                if rng.chance(15) {
                    let k = rng.below(52) as usize;
                    bits[k] = !bits[k];
                }
                if rng.chance(5) {
                    bits.truncate(48 + rng.below(4) as usize);
                }
                let spread = if i % 3 == 0 { 90 } else { 30 };
                let mut pairs = ppm(&bits, rng.jitter(326, spread), rng.jitter(526, spread), 600 + rng.below(20_000) as u32);
                for p in pairs.iter_mut() {
                    *p = (rng.jitter(p.0, 30), rng.jitter(p.1, 25));
                }
                if rng.chance(30) {
                    let again = pairs.clone();
                    pairs.extend(again);
                }
                if rng.chance(60) {
                    mutate(rng, &mut pairs, &interesting);
                }
                pairs
            },
            || Box::new(Astrostart2000Decoder::new()),
            rtl433_json,
        );
    }
}
