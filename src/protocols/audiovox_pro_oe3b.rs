//! Audiovox PRO-OE3B / PRO-OE4B car remote (receive only).
//!
//! Port of rtl_433 `src/devices/audiovox_pro_oe3b.c` (`r_device` "Audiovox PRO-OE3B Car Remote",
//! model `Audiovox-PROOE3B`; FCC ID BGAOE3B, AVX01BT3CL3, ~303 MHz). No Flipper reference has it.
//!
//! **Modulation** — `OOK_PULSE_PWM`: short_width 445 µs (pulse → 1), long_width 895 µs (→ 0),
//! sync_width 1368 µs, reset_limit = gap_limit = 1790 µs, no tolerance, so the slicer's split points
//! apply: a pulse up to 670 µs is a 1, 671–1131 µs a 0, longer a sync (a new row). A gap > 1790 µs
//! ends the message. rtl_433 runs it at priority 10 (only when no priority-0 decoder decoded the
//! package); in KAT it is in the last registry block anyway.
//!
//! **Frame** (`audiovox_pro_oe3b_decode`) — exactly one row of exactly 25 bits. Checks
//! (`DECODE_FAIL_SANITY`): on the wire bits, byte 2 has none of the 0xAA bits set and is not 0x55;
//! after inverting every bit, the 16-bit ID (bytes 0–1) is not 0 or FFFF, and at least one button
//! flag of inverted byte 2 is set: 0x01 Lock, 0x04 Unlock, 0x10 Option, 0x40 Trunk (several can be
//! pressed at once; rtl_433 joins the names with "; "). Fixed code, repeated while held.
//!
//! **Fields** — `serial` = the ID, `button` = the pressed flags packed in rtl_433's order: bit 0
//! Lock, bit 1 Unlock, bit 2 Option, bit 3 Trunk (rtl_433 prints only the names). `data` = the 25
//! wire bits (before the decoder's inversion) right-aligned, `data_count_bit` = 25. `crc_valid` =
//! false (no integrity check in the C).
//!
//! **Encoder** — receive only, as rtl_433.

use super::rtl433::{button_table, frame_signal, join_names, BitBuffer, DecodeFn, Device, Ends, Gaps, Modulation, Receiver};
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

const NAME: &str = "Audiovox-PROOE3B";

/// Metadata (see `info.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &["Audiovox PRO-OE3B Car Remote"],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Audiovox",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// The `r_device`.
const DEVICE: Device = Device {
    modulation: Modulation::OokPwm,
    short_width: 445,
    long_width: 895,
    reset_limit: 1790,
    gap_limit: 1790,
    sync_width: 1368,
    tolerance: 0,
    gaps: Gaps::AtOnce,
    ends: Ends::Ook,
    resync: false,
};

/// rtl_433's `button_strings`, in mask order (0x01, 0x04, 0x10, 0x40 of the inverted byte 2).
const BUTTONS: [&str; 4] = ["Lock", "Unlock", "Option", "Trunk"];

/// rtl_433's `button_str` for KAT's packed flags (bit i = `BUTTONS[i]`).
fn button_str(flags: u8) -> String {
    join_names(BUTTONS.iter().enumerate().filter(|(i, _)| flags >> i & 1 == 1).map(|(_, n)| *n), "")
}

fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    static NAMES: std::sync::OnceLock<Vec<&'static str>> = std::sync::OnceLock::new();
    Some(button_table(&NAMES, button_str)[code as usize])
}

/// `audiovox_pro_oe3b_decode`.
#[derive(Debug, Default)]
struct Audiovox;

impl DecodeFn for Audiovox {
    fn message(&mut self, bits: &mut BitBuffer) -> Option<DecodedSignal> {
        if bits.bits_per_row(0) != 25 {
            return None; // DECODE_ABORT_LENGTH
        }
        if bits.num_rows() != 1 {
            return None; // DECODE_ABORT_EARLY
        }
        let wire: [u8; 4] = bits.bytes(0);
        if bits.byte(0, 2) & 0xaa != 0 || bits.byte(0, 2) == 0x55 {
            return None; // DECODE_FAIL_SANITY
        }
        bits.invert();
        let b: [u8; 3] = bits.bytes(0);
        let id = u16::from_be_bytes([b[0], b[1]]);
        if id == 0 || id == 0xffff {
            return None; // DECODE_FAIL_SANITY
        }
        // mask 0x01, 0x04, 0x10, 0x40 → flags bit 0..3
        let flags = (0..4).filter(|i| b[2] >> (2 * i) & 1 == 1).fold(0u8, |f, i| f | 1 << i);
        if flags == 0 {
            return None; // DECODE_FAIL_SANITY
        }
        let mut d = frame_signal(&wire, 25);
        d.serial = Some(u32::from(id));
        d.button = Some(flags);
        Some(d)
    }
}

/// Audiovox PRO-OE3B decoder (receive only).
pub struct AudiovoxProOe3bDecoder {
    rx: Receiver,
    decode: Audiovox,
}

impl AudiovoxProOe3bDecoder {
    pub fn new() -> Self {
        Self { rx: Receiver::new(&DEVICE), decode: Audiovox }
    }
}

impl Default for AudiovoxProOe3bDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for AudiovoxProOe3bDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: 445, te_long: 895, te_delta: 0, min_count_bit: 25 }
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
    format!(
        "{{\"model\":\"{NAME}\",\"id\":\"{:04X}\",\"button_str\":\"{}\"}}",
        d.serial.unwrap_or(0),
        button_str(d.button.unwrap_or(0))
    )
}

#[cfg(test)]
mod tests {
    use super::super::rtl433::test_util::*;
    use super::*;

    /// The 25 wire bits for an ID and packed button flags (`last`: the free wire bit 24).
    fn frame(id: u16, flags: u8, last: bool) -> Vec<bool> {
        let inv2 = (0..4).filter(|i| flags >> i & 1 == 1).fold(0xAAu8, |b, i| b | 1 << (2 * i));
        let wire = [!(id >> 8) as u8, !id as u8, !inv2, if last { 0x80 } else { 0 }];
        bits_of_bytes(&wire, 25)
    }

    /// PWM as the remote sends it: 1 = 460 µs pulse + 870 µs gap, 0 = 920 + 425, then 10 ms.
    fn wire_pairs(bits: &[bool]) -> Vec<(u32, u32)> {
        let mut pairs: Vec<(u32, u32)> = bits.iter().map(|&b| if b { (460, 870) } else { (920, 425) }).collect();
        pairs.last_mut().unwrap().1 = 10_000;
        pairs
    }

    fn decode(pairs: &[(u32, u32)]) -> Vec<DecodedSignal> {
        feed_pairs(&mut AudiovoxProOe3bDecoder::new(), pairs)
    }

    #[test]
    fn decodes_the_recorded_lock_frame() {
        let got = decode(&wire_pairs(&frame(0x2EA9, 0b0001, true)));
        assert_eq!(got.len(), 1);
        let d = &got[0];
        assert_eq!((d.serial, d.button, d.crc_valid, d.encoder_capable), (Some(0x2EA9), Some(1), false, false));
        assert_eq!((d.data, d.data_count_bit), (0xD1_5654 << 1 | 1, 25));
        assert_eq!(rtl433_json(d), r#"{"model":"Audiovox-PROOE3B","id":"2EA9","button_str":"Lock"}"#);
    }

    #[test]
    fn decodes_every_button_combination() {
        for flags in 1..16u8 {
            let got = decode(&wire_pairs(&frame(0x1234, flags, flags & 1 == 0)));
            assert_eq!(got.iter().map(|d| d.button).collect::<Vec<_>>(), [Some(flags)], "{flags:#b}");
        }
        assert_eq!(button_str(0b1010), "Unlock; Trunk");
        assert_eq!(native_buttons(NAME, 0b0100), Some("Option"));
    }

    #[test]
    fn rejects_failed_checks() {
        // no button
        assert!(decode(&wire_pairs(&frame(0x2EA9, 0, false))).is_empty());
        // a 0xAA bit of byte 2 set on the wire
        let mut bits = frame(0x2EA9, 1, false);
        bits[16] = true;
        assert!(decode(&wire_pairs(&bits)).is_empty());
        // ID 0000 / FFFF
        assert!(decode(&wire_pairs(&frame(0, 1, false))).is_empty());
        assert!(decode(&wire_pairs(&frame(0xFFFF, 1, false))).is_empty());
        // 24 / 26 bits
        let bits = frame(0x2EA9, 1, false);
        assert!(decode(&wire_pairs(&bits[..24])).is_empty());
        let mut long = bits.clone();
        long.push(true);
        assert!(decode(&wire_pairs(&long)).is_empty());
        // a sync pulse inside makes two rows
        let mut pairs = wire_pairs(&bits);
        pairs[10].0 = 1400;
        assert!(decode(&pairs).is_empty());
        // gap boundaries: 1790 µs keeps the message, 1791 ends it
        let mut pairs = wire_pairs(&bits);
        pairs[12].1 = 1790;
        assert_eq!(decode(&pairs).len(), 1);
        pairs[12].1 = 1791;
        assert!(decode(&pairs).is_empty());
    }

    /// rtl_433's own pulses for the first frame of `audiovox/PRO-OE3B/01/lock_303.3M_250k.cu8`
    /// (`rtl_433 -w ook:…`, µs; the last gap is the pulse detector's end of package).
    const RTL_433_LOCK_PULSES: &str = "472 872 456 424 920 860 468 416 920 432 912 428 920 864 464 424 920 864 464 424 \
        920 860 468 424 916 864 464 872 468 428 916 424 920 864 464 428 916 868 464 428 916 864 464 428 920 428 916 428 920 10004";

    #[test]
    fn decodes_rtl_433_pulses_of_the_lock_recording() {
        let v: Vec<u32> = RTL_433_LOCK_PULSES.split_whitespace().map(|x| x.parse().unwrap()).collect();
        let pairs: Vec<(u32, u32)> = v.chunks(2).map(|c| (c[0], c[1])).collect();
        let got = decode(&pairs);
        assert_eq!(got.iter().map(rtl433_json).collect::<Vec<_>>(), [r#"{"model":"Audiovox-PROOE3B","id":"2EA9","button_str":"Lock"}"#]);
    }

    /// rtl_433_tests `audiovox/PRO-OE3B/01` through KAT's receive pipeline and the registry. KAT's
    /// AM chain distorts these frames: it misses the first pulse of each and shortens the long
    /// (920 µs) pulses near its end to 230–460 µs (the remote's carrier drifts several kHz during a
    /// press — rtl_433 measures −92 → −96 kHz over three frames — while rtl_433 slices the full-band
    /// envelope). So KAT decodes Lock and Option with rtl_433's fields, no Unlock, and one corrupt
    /// Lock frame (ID 0029) that passes the C's weak checks (the C decoder reads KAT's pulses the
    /// same way). On rtl_433's pulses the decoder gives all 26 events. Pinned so that a receive-chain
    /// change shows here.
    #[test]
    #[ignore]
    fn decodes_the_rtl_433_recordings() {
        let Some(root) = rtl433_tests() else {
            eprintln!("Skip: rtl_433_tests not found");
            return;
        };
        let dir = root.join("audiovox/PRO-OE3B/01");
        let corrupt: serde_json::Value = serde_json::from_str(r#"{"model":"Audiovox-PROOE3B","id":"0029","button_str":"Lock"}"#).unwrap();
        for (f, found, extra) in [("lock", true, Some(&corrupt)), ("option", true, None), ("unlock", false, None)] {
            let path = dir.join(format!("{f}_303.3M_250k.cu8"));
            let got: Vec<serde_json::Value> = registry_decodes(&path)
                .iter()
                .map(|(n, d)| {
                    assert_eq!(n, NAME, "{f}");
                    serde_json::from_str(&rtl433_json(d)).unwrap()
                })
                .collect();
            let mut want: Vec<serde_json::Value> = if found { distinct(&expected_json(&path)) } else { Vec::new() };
            want.extend(extra.cloned());
            assert_eq!(distinct(&got), want, "{f}");
        }
    }

    /// Fuzz dump for the C harness (see `rtl433::test_util::dump_fuzz_trains`).
    #[test]
    #[ignore]
    fn fuzz_dump() {
        let interesting = [1, 669, 670, 671, 672, 1130, 1131, 1132, 1133, 1789, 1790, 1791];
        dump_fuzz_trains(
            "audiovox",
            20_000,
            |rng, i| {
                let mut bits = frame(rng.next() as u16, rng.below(16) as u8, rng.chance(50));
                if rng.chance(20) {
                    let k = rng.below(25) as usize;
                    bits[k] = !bits[k];
                }
                if rng.chance(5) {
                    bits.truncate(22 + rng.below(3) as usize);
                }
                let (one, zero, spread) = if i % 3 == 0 { (445, 895, 200) } else { (460, 920, 40) };
                let mut pairs: Vec<(u32, u32)> = bits
                    .iter()
                    .map(|&b| if b { (rng.jitter(one, spread), rng.jitter(870, 300)) } else { (rng.jitter(zero, spread), rng.jitter(425, 200)) })
                    .collect();
                pairs.last_mut().unwrap().1 = 1791 + rng.below(20_000) as u32;
                if rng.chance(40) {
                    let again = pairs.clone();
                    pairs.extend(again);
                }
                if rng.chance(60) {
                    mutate(rng, &mut pairs, &interesting);
                }
                pairs
            },
            || Box::new(AudiovoxProOe3bDecoder::new()),
            rtl433_json,
        );
    }
}
