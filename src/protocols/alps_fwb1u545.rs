//! Alps FWB1U545 car remote (receive only).
//!
//! Port of rtl_433 `src/devices/alps_fwb1u545.c` (`r_device` "Alps FWB1U545 Car Remote", model
//! `Alps-FWB1U545`; FCC ID CWTWB1U545, an Alps Electric OEM remote for Honda). No Flipper reference
//! has it.
//!
//! **Modulation** — `FSK_PULSE_MANCHESTER_ZEROBIT`: short_width 500 µs, reset_limit 1500 µs, no
//! tolerance (the slicer rejects no pulse). rtl_433's zerobit slicer ([super::rtl433])
//! runs on KAT's FM pairs (higher tone = pulse, as rtl_433's FSK detector); a gap > 1500 µs ends
//! the message. The recordings are 433.92 MHz 2-FSK, about ±30–37 kHz.
//!
//! **Frame** (`alps_fwb1u545_car_remote_decode`) — exactly one row of exactly 76 bits:
//! `PP IIIIIIII bb bb SSSS C`: byte 0 the 0x55 preamble byte, bytes 1–4 the 32-bit ID, byte 5 the
//! button code, byte 6 its copy, bytes 7–8 the 16-bit sequence, then 4 unknown bits. Checks
//! (`DECODE_FAIL_SANITY`): byte 0 = 0x55, byte 6 = byte 5, ID ≠ 0 and ≠ FFFFFFFF. Button =
//! byte 5 >> 4: 0xE Lock, 0xC Panic, 0x5 Panic Held, 0x1 Unlock, else "?". The long 500/500 µs
//! preamble is its own message (it ends at the ~2.5 ms gap before the frame) and fails the length
//! check.
//!
//! **Fields** — `serial` = ID, `button` = byte 5 >> 4, `counter` = sequence. `data` = the first 64
//! frame bits (bytes 0–7), `extra` = the last 12 (byte 8 and the unknown nibble), right-aligned;
//! `data_count_bit` = 76. `crc_valid` = false: rtl_433 reports no integrity check for this remote
//! (its checks are sanity checks; it prints no `mic` field). rtl_433's `button_str` is the native
//! button name.
//!
//! **Encoder** — receive only, as rtl_433 (KAT offers Replay of the recorded pulses).

use super::rtl433::{frame_signal, BitBuffer, DecodeFn, Device, Ends, Gaps, Modulation, Receiver};
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

const NAME: &str = "Alps-FWB1U545";

/// Metadata (see `info.rs`): 2-FSK; F3 (±31.74 kHz) is the closest preset to the recordings'
/// deviation.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &["Alps FWB1U545 Car Remote"],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::FM,
    encryption: "Rolling Code",
    make: "Honda",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&F3],
};

/// The `r_device`; a burst's FM pairs are one package (see `rtl433`).
const DEVICE: Device = Device {
    modulation: Modulation::FskManchesterZerobit,
    short_width: 500,
    long_width: 0,
    reset_limit: 1500,
    gap_limit: 0,
    sync_width: 0,
    tolerance: 0,
    gaps: Gaps::AtOnce,
    ends: Ends::None,
    resync: false,
};

/// rtl_433's `button_str`.
fn button_str(code: u8) -> &'static str {
    match code {
        0xe => "Lock",
        0xc => "Panic",
        0x5 => "Panic Held",
        0x1 => "Unlock",
        _ => "?",
    }
}

fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(button_str(code))
}

/// `alps_fwb1u545_car_remote_decode`.
#[derive(Debug, Default)]
struct Alps;

impl DecodeFn for Alps {
    fn message(&mut self, bits: &mut BitBuffer) -> Option<DecodedSignal> {
        if bits.bits_per_row(0) != 76 || bits.num_rows() > 1 {
            return None; // DECODE_ABORT_LENGTH
        }
        let b: [u8; 10] = bits.bytes(0);
        // check preamble and button XOR
        if b[0] != 0x55 || b[5] != b[6] {
            return None; // DECODE_FAIL_SANITY
        }
        let id = u32::from_be_bytes([b[1], b[2], b[3], b[4]]);
        if id == 0 || id == 0xffff_ffff {
            return None; // DECODE_FAIL_SANITY
        }
        let mut d = frame_signal(&b, 76);
        d.serial = Some(id);
        d.button = Some(b[5] >> 4);
        d.counter = Some(u16::from_be_bytes([b[7], b[8]]));
        Some(d)
    }
}

/// Alps FWB1U545 decoder (receive only).
pub struct AlpsFwb1u545Decoder {
    rx: Receiver,
    decode: Alps,
}

impl AlpsFwb1u545Decoder {
    pub fn new() -> Self {
        Self { rx: Receiver::new(&DEVICE), decode: Alps }
    }
}

impl Default for AlpsFwb1u545Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for AlpsFwb1u545Decoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: 500, te_long: 1000, te_delta: 0, min_count_bit: 76 }
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
        "{{\"model\":\"{NAME}\",\"id\":\"{:08X}\",\"button_code\":{btn},\"button_str\":\"{}\",\"sequence\":{}}}",
        d.serial.unwrap_or(0),
        button_str(btn),
        d.counter.unwrap_or(0)
    )
}

#[cfg(test)]
mod tests {
    use super::super::rtl433::test_util::*;
    use super::*;

    /// The 76 frame bits for an ID, button and sequence (C layout; `button` is byte 5's high nibble,
    /// `low` its low nibble, `tail` the unknown nibble).
    fn frame(id: u32, button: u8, low: u8, seq: u16, tail: u8) -> Vec<bool> {
        let b5 = button << 4 | low;
        let bytes = [0x55, (id >> 24) as u8, (id >> 16) as u8, (id >> 8) as u8, id as u8, b5, b5, (seq >> 8) as u8, seq as u8, tail << 4];
        bits_of_bytes(&bytes, 76)
    }

    /// A transmission as the recordings show it: 500/500 µs preamble, a 2.5 ms gap, the frame.
    fn transmission(row: &[bool], half: u32) -> Vec<(u32, u32)> {
        let mut pairs = vec![(half, half); 200];
        pairs.last_mut().unwrap().1 = 2500;
        pairs.extend(manchester_zerobit(row, half, 80_000));
        pairs
    }

    #[test]
    fn decodes_every_button() {
        for (button, label) in [(0xE, "Lock"), (0xC, "Panic"), (0x5, "Panic Held"), (0x1, "Unlock"), (0x7, "?")] {
            let row = frame(0x4817_E61E, button, 0x3, 34_788, 0xA);
            let got = feed_pairs(&mut AlpsFwb1u545Decoder::new(), &transmission(&row, 500));
            assert_eq!(got.len(), 1, "{label}");
            let d = &got[0];
            assert_eq!((d.serial, d.button, d.counter), (Some(0x4817_E61E), Some(button), Some(34_788)));
            assert_eq!((d.data_count_bit, d.crc_valid, d.encoder_capable), (76, false, false));
            // bytes 0..7 = 55 48 17 E6 1E b5 b5 87, then E4 and the unknown nibble A
            let b5 = u64::from(button << 4 | 3);
            assert_eq!(d.data, 0x55_4817_E61E_u64 << 24 | b5 << 16 | b5 << 8 | 0x87);
            assert_eq!(d.extra, Some(0xE4A));
            assert_eq!(native_buttons(NAME, button), Some(label));
            assert!(rtl433_json(d).contains(&format!("\"button_str\":\"{label}\"")));
        }
    }

    #[test]
    fn tolerates_timing_like_the_c() {
        // No tolerance: only the 1.5 × short edge spacing matters.
        let row = frame(0x0102_0304, 0x1, 0, 7, 0);
        for half in [420, 500, 580] {
            let got = feed_pairs(&mut AlpsFwb1u545Decoder::new(), &transmission(&row, half));
            assert_eq!(got.len(), 1, "half {half}");
            assert_eq!(got[0].serial, Some(0x0102_0304));
        }
    }

    #[test]
    fn rejects_failed_checks() {
        let decode = |row: &[bool]| feed_pairs(&mut AlpsFwb1u545Decoder::new(), &transmission(row, 500));
        // button copy differs
        let mut row = frame(0x4817_E61E, 0xE, 0, 1, 0);
        row[47] = !row[47];
        assert!(decode(&row).is_empty());
        // preamble byte
        let mut row = frame(0x4817_E61E, 0xE, 0, 1, 0);
        row[1] = !row[1];
        assert!(decode(&row).is_empty());
        // ID 0 and FFFFFFFF
        assert!(decode(&frame(0, 0xE, 0, 1, 0)).is_empty());
        assert!(decode(&frame(0xFFFF_FFFF, 0xE, 0, 1, 0)).is_empty());
        // 75 / 77 bits
        let row = frame(0x4817_E61E, 0xE, 0, 1, 0);
        assert!(decode(&row[..75]).is_empty());
        let mut long = row.clone();
        long.push(true);
        assert!(decode(&long).is_empty());
        // a gap over 1500 µs inside the frame splits it
        let mut pairs = transmission(&row, 500);
        let n = pairs.len();
        pairs[n - 20].1 = 1600;
        assert!(feed_pairs(&mut AlpsFwb1u545Decoder::new(), &pairs).is_empty());
    }

    /// rtl_433_tests `alps/fwb1u545/01`: the four recordings through KAT's receive pipeline and the
    /// registry give rtl_433's events (`.json` next to each file).
    #[test]
    #[ignore]
    fn decodes_the_rtl_433_recordings() {
        let Some(root) = rtl433_tests() else {
            eprintln!("Skip: rtl_433_tests not found");
            return;
        };
        let dir = root.join("alps/fwb1u545/01");
        for f in ["lock", "panic", "panic_held", "unlock"] {
            let path = dir.join(format!("{f}_433.92M_250k.cu8"));
            let got: Vec<serde_json::Value> = registry_decodes(&path)
                .iter()
                .map(|(n, d)| {
                    assert_eq!(n, NAME, "{f}");
                    serde_json::from_str(&rtl433_json(d)).unwrap()
                })
                .collect();
            let want = expected_json(&path);
            assert_eq!(distinct(&got), distinct(&want), "{f}");
        }
    }

    /// Fuzz dump for the C harness (see `rtl433::test_util::dump_fuzz_trains`).
    #[test]
    #[ignore]
    fn fuzz_dump() {
        let interesting = [1, 499, 500, 501, 749, 750, 751, 1000, 1499, 1500, 1501, 2500];
        dump_fuzz_trains(
            "alps",
            20_000,
            |rng, i| {
                let id = match rng.below(20) {
                    0 => 0,
                    1 => 0xFFFF_FFFF,
                    _ => rng.next() as u32,
                };
                let button = [0xE, 0xC, 0x5, 0x1][rng.below(4) as usize] ^ if rng.chance(10) { rng.below(16) as u8 } else { 0 };
                let mut row = frame(id, button, rng.below(16) as u8, rng.next() as u16, rng.below(16) as u8);
                if rng.chance(15) {
                    let k = 1 + rng.below(75) as usize;
                    row[k] = !row[k];
                }
                if rng.chance(5) {
                    row.truncate(70 + rng.below(6) as usize);
                }
                let half = rng.jitter(500, if i % 3 == 0 { 120 } else { 30 });
                let mut pairs = if rng.chance(50) { transmission(&row, half) } else { manchester_zerobit(&row, half, 3000) };
                for p in pairs.iter_mut() {
                    *p = (rng.jitter(p.0, 15), rng.jitter(p.1, 15));
                }
                if rng.chance(60) {
                    mutate(rng, &mut pairs, &interesting);
                }
                pairs
            },
            || Box::new(AlpsFwb1u545Decoder::new()),
            rtl433_json,
        );
    }
}
