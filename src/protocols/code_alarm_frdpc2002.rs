//! Code Alarm FRDPC2002 car remote (receive only).
//!
//! Port of rtl_433 `src/devices/code_alarm_car_remote.c` (`r_device` "Code Alarm FRDPC2002 Car
//! Remote", model `CodeAlarm-FRDPC2002`; FRDPC2002 / GOH-FRDPC2002 car-alarm remotes). No Flipper
//! reference has it.
//!
//! **Modulation** — `OOK_PULSE_MANCHESTER_ZEROBIT`: short_width 550 µs, long_width 1100 µs,
//! reset_limit 1600 µs, tolerance 100 µs: a pulse or gap outside 450–1200 µs closes the row (the
//! C's "[1]10 transition" bit included); a gap > 1600 µs ends the message. Rolling code (rtl_433:
//! "the manual tied to the FCC ID states a 36-bit rolling code").
//!
//! **Frame** (`code_alarm_frdpc2000_car_remote_decode`) — row 0 of the message, exactly 60 bits
//! (other rows are not looked at): `PPPP uuuu bbbb IIIIIIII uuuu`. Checks (`DECODE_FAIL_SANITY`):
//! bytes 0 and 1 are 0x00; the 40 bits from bit 19 (rtl_433's `data`) do not sum to 0 or ≥ 5 × 0xFF.
//! The ID is the 36-bit code from bit 23, folded: `(c0^c1) << 16 | (c1^c2) << 8 | (c2^c3)` (24 bits).
//! Button = the high nibble of the bit-19 data: rtl_433's map (Multiple 7; Lock 6, 4; Panic 1, 3;
//! Start 0, 3; Unlock 5, 4 — so 4 is "Lock; Unlock", 3 "Panic; Start"; anything else "?").
//!
//! **Fields** — `serial` = the 24-bit ID, `button` = the 4-bit code, no counter (the rolling part
//! is not decoded by rtl_433). `data` = the 60 row bits right-aligned (`data_count_bit` 60);
//! rtl_433's `data` field is bits 19..58 of it. `crc_valid` = false (no integrity check in the C).
//!
//! **Encoder** — receive only, as rtl_433.

use super::rtl433::{button_table, frame_signal, join_names, BitBuffer, DecodeFn, Device, Ends, Gaps, Modulation, Receiver};
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

const NAME: &str = "CodeAlarm-FRDPC2002";

/// Metadata (see `info.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &["Code Alarm FRDPC2002 Car Remote"],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "Code Alarm",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// The `r_device`.
const DEVICE: Device = Device {
    modulation: Modulation::OokManchesterZerobit,
    short_width: 550,
    long_width: 1100,
    reset_limit: 1600,
    gap_limit: 0,
    sync_width: 0,
    tolerance: 100,
    gaps: Gaps::AtOnce,
    ends: Ends::Ook,
    resync: false,
};

/// rtl_433's `button_map`.
const BUTTON_MAP: [(&str, &[u8]); 5] = [
    ("Multiple", &[0x7]),
    ("Lock", &[0x6, 0x4]),
    ("Panic", &[0x1, 0x3]),
    ("Start", &[0x0, 0x3]),
    ("Unlock", &[0x5, 0x4]),
];

/// rtl_433's `button_str`.
fn button_str(code: u8) -> String {
    join_names(BUTTON_MAP.iter().filter(|(_, v)| v.contains(&code)).map(|(n, _)| *n), "?")
}

fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    static NAMES: std::sync::OnceLock<Vec<&'static str>> = std::sync::OnceLock::new();
    Some(button_table(&NAMES, button_str)[code as usize])
}

/// `code_alarm_frdpc2000_car_remote_decode`.
#[derive(Debug, Default)]
struct CodeAlarm;

impl DecodeFn for CodeAlarm {
    fn message(&mut self, bits: &mut BitBuffer) -> Option<DecodedSignal> {
        if bits.bits_per_row(0) != 60 {
            return None; // DECODE_ABORT_LENGTH
        }
        if bits.byte(0, 0) != 0x00 || bits.byte(0, 1) != 0x00 {
            return None; // DECODE_FAIL_SANITY
        }
        let mut bytes = [0u8; 5];
        bits.extract_bytes(0, 19, &mut bytes, 40);
        let sum: u32 = bytes.iter().map(|&b| u32::from(b)).sum();
        if sum == 0 || sum >= 0xff * 5 {
            return None; // DECODE_FAIL_SANITY
        }
        // manual tied to FCC id states a 36bit rolling code
        let mut code = [0u8; 5];
        bits.extract_bytes(0, 23, &mut code, 36);
        let id = u32::from(code[0] ^ code[1]) << 16 | u32::from(code[1] ^ code[2]) << 8 | u32::from(code[2] ^ code[3]);
        let mut d = frame_signal(&bits.bytes::<8>(0), 60);
        d.serial = Some(id);
        d.button = Some(bytes[0] >> 4);
        Some(d)
    }
}

/// Code Alarm FRDPC2002 decoder (receive only).
pub struct CodeAlarmFrdpc2002Decoder {
    rx: Receiver,
    decode: CodeAlarm,
}

impl CodeAlarmFrdpc2002Decoder {
    pub fn new() -> Self {
        Self { rx: Receiver::new(&DEVICE), decode: CodeAlarm }
    }
}

impl Default for CodeAlarmFrdpc2002Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for CodeAlarmFrdpc2002Decoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: 550, te_long: 1100, te_delta: 100, min_count_bit: 60 }
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
        "{{\"model\":\"{NAME}\",\"id\":\"{:06X}\",\"button_code\":{btn},\"button_str\":\"{}\",\"data\":\"{:010X}\"}}",
        d.serial.unwrap_or(0),
        button_str(btn),
        d.data >> 1 & 0xFF_FFFF_FFFF
    )
}

#[cfg(test)]
mod tests {
    use super::super::rtl433::test_util::*;
    use super::*;

    /// A 60-bit row: 19 zero bits (the C checks the first 16), then 41 bits whose top 40 are
    /// rtl_433's `data` (`data40`), the last bit free.
    fn row(data40: u64, last: bool) -> Vec<bool> {
        let mut r = vec![false; 19];
        r.extend(bits_of(data40, 40));
        r.push(last);
        r
    }

    /// One message: the row as zerobit Manchester at 550 µs half-bits, ended by a 5 ms gap.
    fn message(r: &[bool]) -> Vec<(u32, u32)> {
        manchester_zerobit(r, 550, 5000)
    }

    /// The recordings' Lock frame: data 657E65FD7C, ID B1B988, button 6.
    const LOCK: u64 = 0x65_7E65_FD7C;

    #[test]
    fn decodes_the_recorded_lock_frame() {
        let got = feed_pairs(&mut CodeAlarmFrdpc2002Decoder::new(), &message(&row(LOCK, false)));
        assert_eq!(got.len(), 1);
        let d = &got[0];
        assert_eq!((d.serial, d.button, d.counter, d.crc_valid), (Some(0xB1B988), Some(6), None, false));
        assert_eq!((d.data, d.data_count_bit, d.extra), (LOCK << 1, 60, None));
        assert_eq!(
            rtl433_json(d),
            r#"{"model":"CodeAlarm-FRDPC2002","id":"B1B988","button_code":6,"button_str":"Lock","data":"657E65FD7C"}"#
        );
    }

    #[test]
    fn button_names_follow_the_map() {
        let names: Vec<String> = (0..9).map(button_str).collect();
        assert_eq!(names, ["Start", "Panic", "?", "Panic; Start", "Lock; Unlock", "Unlock", "Lock", "Multiple", "?"]);
        for code in 0..16u8 {
            let data = LOCK & !(0xF << 36) | u64::from(code) << 36;
            let got = feed_pairs(&mut CodeAlarmFrdpc2002Decoder::new(), &message(&row(data, true)));
            assert_eq!(got.iter().map(|d| d.button).collect::<Vec<_>>(), [Some(code)], "{code}");
            assert_eq!(native_buttons(NAME, code), Some(button_str(code).as_str()));
        }
    }

    #[test]
    fn rejects_failed_checks() {
        let decode = |r: &[bool]| feed_pairs(&mut CodeAlarmFrdpc2002Decoder::new(), &message(r));
        // a 1 in the first 16 bits
        let mut r = row(LOCK, false);
        r[9] = true;
        assert!(decode(&r).is_empty());
        // data that sums to 0 (all zero) or to 5 × 0xFF
        assert!(decode(&row(0, false)).is_empty());
        assert!(decode(&row(0xFF_FFFF_FFFF, true)).is_empty());
        // 59 / 61 bits
        assert!(decode(&row(LOCK, false)[..59]).is_empty());
        let mut long = row(LOCK, false);
        long.push(true);
        assert!(decode(&long).is_empty());
        // a 1300 µs pulse (outside 450..1200) closes the row early
        let mut pairs = message(&row(LOCK, false));
        pairs[20].0 = 1300;
        assert!(feed_pairs(&mut CodeAlarmFrdpc2002Decoder::new(), &pairs).is_empty());
    }

    /// rtl_433_tests `code-alarm/frdpc2002/01`: KAT's registry decodes of the five recordings equal
    /// rtl_433's events.
    #[test]
    #[ignore]
    fn decodes_the_rtl_433_recordings() {
        let Some(root) = rtl433_tests() else {
            eprintln!("Skip: rtl_433_tests not found");
            return;
        };
        let dir = root.join("code-alarm/frdpc2002/01");
        for f in ["lock", "lock_unlock", "panic", "start", "unlock"] {
            let path = dir.join(format!("{f}_433.92M_250k.cu8"));
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
        let interesting = [1, 449, 450, 451, 824, 825, 826, 1100, 1199, 1200, 1201, 1600, 1601];
        dump_fuzz_trains(
            "codealarm",
            20_000,
            |rng, i| {
                let data = match rng.below(10) {
                    0 => rng.next() & 0xFF_FFFF_FFFF,
                    1 => 0,
                    _ => LOCK ^ (rng.next() & 0x0F_FFFF_FFFF),
                };
                let mut r = row(data, rng.chance(50));
                if rng.chance(15) {
                    let k = 1 + rng.below(59) as usize;
                    r[k] = !r[k];
                }
                if rng.chance(5) {
                    r.truncate(56 + rng.below(6) as usize);
                }
                let half = rng.jitter(550, if i % 3 == 0 { 140 } else { 40 });
                let mut pairs = manchester_zerobit(&r, half, 1700);
                if rng.chance(40) {
                    let again = pairs.clone();
                    pairs.extend(again);
                }
                for p in pairs.iter_mut() {
                    *p = (rng.jitter(p.0, 20), rng.jitter(p.1, 20));
                }
                if rng.chance(60) {
                    mutate(rng, &mut pairs, &interesting);
                }
                pairs
            },
            || Box::new(CodeAlarmFrdpc2002Decoder::new()),
            rtl433_json,
        );
    }
}
