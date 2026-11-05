//! Chrysler car remote, 315 MHz (receive only) — not KAT's "Chrysler V0".
//!
//! Port of rtl_433 `src/devices/chrysler_car_remote.c` (`r_device` "Chrysler Car Remote", model
//! `Chrysler-CarRemote`; part numbers 56008761, 56008762 (FCC ID GQ43VT7T), 04686366, 56021903AA).
//! A different air format from ProtoPirate's Chrysler V0 (300/600 µs pulses + 3.4 ms gaps, 80 bits),
//! which does not decode these remotes. No Flipper reference has it.
//!
//! **Modulation** — `OOK_PULSE_PWM`: short_width 350 µs (pulse → 1), long_width 700 µs (→ 0),
//! sync_width 7350 µs, tolerance 100 µs, so a pulse in 251–449 µs is a 1, 601–799 µs a 0, 7251–7449
//! µs a sync; a pulse ≤ 250 µs is ignored, any other starts a new row. A gap > 4200 µs (gap_limit)
//! starts a new row, a gap > 17 500 µs (reset_limit) ends the message. A press is one message: a
//! sync pulse, a 25-bit preamble row, then the packet twice (49 and 48 bits), 16 ms apart.
//!
//! **Frame** (`chrysler_car_remote_decode`) — every row of 48 or more bits, inverted: the 48 bits
//! that end one bit before the row's end (49 or more bits) or the whole 48-bit row, each byte
//! bit-reversed: `IIIIIIII bbbb x d xx CC` — a 32-bit ID, the 4-bit button code, the multi-press
//! bit (0x04 of byte 4), and an 8-bit checksum = the sum of bytes 0–4. Checks: checksum
//! (`DECODE_FAIL_MIC`); the sum not 0 or 5 × 0xFF, at least one of Unlock 0x1 / Lock 0x2 / Panic 0x4
//! pressed, and the multi-press bit set exactly when more than one is (`DECODE_FAIL_SANITY`). Every
//! valid row is an event (rtl_433 usually outputs the two packets of a press).
//!
//! **Fields** — `serial` = the ID, `button` = the 4-bit code (names: Unlock 1, Lock 2, Panic 4,
//! joined with "; "), no counter (fixed code). `data` = the 48 wire bits the event was read from
//! (before inversion and bit reversal), right-aligned, `data_count_bit` = 48. `crc_valid` = true
//! (checksum passed).
//!
//! **Reports** — row-wise ([super::rtl433]): each valid packet is reported as soon as the
//! next row starts (or the message ends), so KAT reports both packets of a press as rtl_433 does.
//!
//! **Encoder** — receive only, as rtl_433.

use super::rtl433::{button_table, frame_signal, join_names, BitBuffer, DecodeFn, Device, Ends, Gaps, Modulation, Receiver};
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

const NAME: &str = "Chrysler-CarRemote";

/// Metadata (see `info.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &["Chrysler Car Remote"],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Chrysler/Dodge/Jeep",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// The `r_device`.
const DEVICE: Device = Device {
    modulation: Modulation::OokPwm,
    short_width: 350,
    long_width: 700,
    reset_limit: 17_500,
    gap_limit: 4200,
    sync_width: 7350,
    tolerance: 100,
    gaps: Gaps::AtOnce,
    ends: Ends::Ook,
    resync: false,
};

/// rtl_433's `button_strings` (masks 0x1, 0x2, 0x4).
const BUTTONS: [&str; 3] = ["Unlock", "Lock", "Panic"];

/// rtl_433's `button_str`.
fn button_str(code: u8) -> String {
    join_names(BUTTONS.iter().enumerate().filter(|(i, _)| code >> i & 1 == 1).map(|(_, n)| *n), "")
}

fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    static NAMES: std::sync::OnceLock<Vec<&'static str>> = std::sync::OnceLock::new();
    Some(button_table(&NAMES, button_str)[code as usize])
}

/// `chrysler_car_remote_decode`, one row at a time.
#[derive(Debug, Default)]
struct Chrysler;

impl DecodeFn for Chrysler {
    const ROW_WISE: bool = true;

    fn row(&mut self, bits: &BitBuffer, row: usize) -> Option<DecodedSignal> {
        let n = bits.bits_per_row(row);
        // try 49 bits from the end, or exactly 48
        let offset = match n {
            49.. => 49,
            48 => 48,
            _ => return None,
        };
        let pos = n - offset;
        let mut wire = [0u8; 6];
        bits.extract_bytes(row, pos, &mut wire, 48);
        // bitbuffer_invert + reflect_bytes
        let b = wire.map(|w| (!w).reverse_bits());
        let sum: u32 = b[..5].iter().map(|&x| u32::from(x)).sum();
        if sum & 0xff != u32::from(b[5]) {
            return None; // DECODE_FAIL_MIC
        }
        if sum == 0 || sum == 0xff * 5 {
            return None; // DECODE_FAIL_SANITY
        }
        let button = b[4] >> 4;
        let multi_press = b[4] & 0x4 != 0;
        let matches = (button & 0x7).count_ones();
        if matches == 0 || (matches > 1 && !multi_press) || (matches == 1 && multi_press) {
            return None; // DECODE_FAIL_SANITY
        }
        let mut d = frame_signal(&wire, 48);
        d.serial = Some(u32::from_be_bytes([b[0], b[1], b[2], b[3]]));
        d.button = Some(button);
        d.crc_valid = true;
        Some(d)
    }
}

/// Chrysler car remote decoder (receive only).
pub struct ChryslerCarRemoteDecoder {
    rx: Receiver,
    decode: Chrysler,
}

impl ChryslerCarRemoteDecoder {
    pub fn new() -> Self {
        Self { rx: Receiver::new(&DEVICE), decode: Chrysler }
    }
}

impl Default for ChryslerCarRemoteDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for ChryslerCarRemoteDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: 350, te_long: 700, te_delta: 100, min_count_bit: 48 }
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

    /// The 48 wire bits of a packet (decoded bytes → inverted, bit-reversed bytes; checksum
    /// computed unless given).
    fn packet(id: u32, b4: u8, sum: Option<u8>) -> Vec<bool> {
        let i = id.to_be_bytes();
        let mut b = [i[0], i[1], i[2], i[3], b4, 0];
        b[5] = sum.unwrap_or_else(|| b[..5].iter().fold(0u8, |a, &x| a.wrapping_add(x)));
        bits_of_bytes(&b.map(|x| !x.reverse_bits()), 48)
    }

    /// A press as the recordings show it: sync, 25-bit preamble, the packet with one more bit,
    /// the packet again, 16 ms apart; 1 = 330 µs pulse + 3830 µs gap, 0 = 650 + 3510.
    fn press(p1: &[bool], p2: &[bool]) -> Vec<(u32, u32)> {
        let bit = |b: bool| if b { (330, 3830) } else { (650, 3510) };
        let mut pairs = vec![(7377, 4128)];
        pairs.extend(std::iter::repeat_n((325, 3835), 25));
        pairs.last_mut().unwrap().1 = 16_300;
        pairs.extend(p1.iter().map(|&b| bit(b)));
        pairs.push(bit(true));
        pairs.last_mut().unwrap().1 = 16_290;
        pairs.extend(p2.iter().map(|&b| bit(b)));
        pairs.last_mut().unwrap().1 = 80_000;
        pairs
    }

    fn decode(pairs: &[(u32, u32)]) -> Vec<DecodedSignal> {
        feed_pairs(&mut ChryslerCarRemoteDecoder::new(), pairs)
    }

    #[test]
    fn decodes_both_packets_of_a_press() {
        // 01/lock: ID 401A7FED, Lock; wire FD A7 01 48 FB 98
        let p = packet(0x401A_7FED, 0x20, None);
        assert_eq!(bits_of(0xFDA7_0148_FB98, 48), p);
        let got = decode(&press(&p, &p));
        assert_eq!(got.len(), 2);
        for d in &got {
            assert_eq!((d.serial, d.button, d.counter, d.crc_valid), (Some(0x401A_7FED), Some(2), None, true));
            assert_eq!((d.data, d.data_count_bit), (0xFDA7_0148_FB98, 48));
            assert_eq!(
                rtl433_json(d),
                r#"{"model":"Chrysler-CarRemote","id":"401A7FED","button_code":2,"button_str":"Lock","mic":"CHECKSUM"}"#
            );
        }
    }

    #[test]
    fn button_and_multi_press_rules() {
        let id = 0x40FC_4ACD;
        for (b4, ok) in [(0x10, true), (0x20, true), (0x40, true), (0x34, true), (0x74, true), (0x30, false), (0x14, false), (0x00, false), (0x04, false), (0x80, false), (0x9B, true)] {
            let p = packet(id, b4, None);
            let got = decode(&press(&p, &p));
            assert_eq!(got.len(), if ok { 2 } else { 0 }, "{b4:#x}");
        }
        assert_eq!(button_str(3), "Unlock; Lock");
        assert_eq!(native_buttons(NAME, 4), Some("Panic"));
    }

    #[test]
    fn rejects_failed_checks() {
        let id = 0x403F_7033;
        // checksum
        let bad = packet(id, 0x40, Some(0x12));
        assert!(decode(&press(&bad, &bad)).is_empty());
        // one good packet, one bad: one event
        let good = packet(id, 0x40, None);
        assert_eq!(decode(&press(&bad, &good)).len(), 1);
        // sum 0 / 5 × 0xFF never pass (no button either)
        assert!(decode(&press(&packet(0, 0, None), &packet(0, 0, None))).is_empty());
        // 47-bit rows are skipped
        assert!(decode(&press(&good[..46], &good[..47])).is_empty());
        // a 200 µs pulse is ignored (the bit is lost), a 500 µs pulse splits the row
        let mut pairs = press(&good, &good);
        pairs[60].0 = 200;
        assert_eq!(decode(&pairs).len(), 1, "first packet one bit short");
        let mut pairs = press(&good, &good);
        pairs[60].0 = 500;
        assert_eq!(decode(&pairs).len(), 1);
        // a gap over 17.5 ms between the packets ends the message: still both packets
        let mut pairs = press(&good, &good);
        pairs[74].1 = 17_501;
        assert_eq!(decode(&pairs).len(), 2);
        // ... but inside a packet it cuts it
        let mut pairs = press(&good, &good);
        pairs[90].1 = 17_501;
        assert_eq!(decode(&pairs).len(), 1);
    }

    /// The pulse detector's package limit: an OOK package ends at its 1200th pulse (its message
    /// with it), so a packet across that pulse is lost, as in rtl_433.
    #[test]
    fn package_ends_at_1200_pulses() {
        let good = packet(0x403F_7033, 0x40, None);
        let bit = |b: bool| if b { (330, 3830) } else { (650, 3510) };
        let mut pairs = vec![(7377, 4128)];
        pairs.extend(std::iter::repeat_n((325, 3835), 9));
        for _ in 0..30 {
            pairs.last_mut().unwrap().1 = 5000;
            pairs.extend(good.iter().map(|&b| bit(b)));
        }
        pairs.last_mut().unwrap().1 = 80_000;
        // packets 1..24 end by pulse 1162; packet 25 spans pulses 1163..1210
        assert_eq!(decode(&pairs).len(), 29);
    }

    /// rtl_433_tests `chrysler_car_remote` 01..05: KAT's registry decodes of the seven recordings
    /// equal rtl_433's events (the 920 kS/s recordings, carriers 74–384 kHz off the centre).
    #[test]
    #[ignore]
    fn decodes_the_rtl_433_recordings() {
        let Some(root) = rtl433_tests() else {
            eprintln!("Skip: rtl_433_tests not found");
            return;
        };
        for f in ["01/lock", "01/panic", "01/unlock", "02/panic", "03/panic", "04/panic", "05/panic"] {
            let path = root.join(format!("chrysler_car_remote/{f}_315.1M_920k.cu8"));
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
        let interesting = [1, 249, 250, 251, 449, 450, 451, 599, 600, 601, 799, 800, 801, 4199, 4200, 4201, 7250, 7251, 7449, 7450, 9_999, 10_001, 17_499, 17_500, 17_501];
        dump_fuzz_trains(
            "chrysler",
            20_000,
            |rng, i| {
                let b4 = if rng.chance(70) { [0x10, 0x20, 0x40, 0x34, 0x74, 0x24][rng.below(6) as usize] | (rng.below(4) as u8) } else { rng.below(256) as u8 };
                let id = rng.next() as u32;
                let sum = rng.chance(10).then(|| rng.below(256) as u8);
                let mut p1 = packet(id, b4, sum);
                let mut p2 = p1.clone();
                for p in [&mut p1, &mut p2] {
                    if rng.chance(15) {
                        let k = rng.below(48) as usize;
                        p[k] = !p[k];
                    }
                    if rng.chance(5) {
                        p.truncate(45 + rng.below(3) as usize);
                    }
                }
                let mut pairs = press(&p1, &p2);
                if rng.chance(20) {
                    pairs.remove(0); // no sync
                }
                if rng.chance(8) {
                    // a long press: past 50 rows and 1200 pulses
                    let one = pairs.clone();
                    for _ in 0..rng.below(25) {
                        pairs.extend(one.iter().skip(1));
                    }
                }
                let spread = if i % 3 == 0 { 120 } else { 30 };
                for p in pairs.iter_mut() {
                    *p = (rng.jitter(p.0, spread), rng.jitter(p.1, 200));
                }
                if rng.chance(60) {
                    mutate(rng, &mut pairs, &interesting);
                }
                pairs
            },
            || Box::new(ChryslerCarRemoteDecoder::new()),
            rtl433_json,
        );
    }
}
