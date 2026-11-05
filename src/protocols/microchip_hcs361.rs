//! Microchip HCS361 KeeLoq code-hopping encoder (receive only).
//!
//! Port of rtl_433 `src/devices/hcs361.c` (`REFERENCES/rtl_433`, GPL-2.0; model
//! "Microchip-HCS361"): six `r_device`s with one decoder, for the chip's EEPROM-selected line
//! codings (Jeep / Chrysler, Marelli and Leer remotes, 315 MHz):
//!
//! | r_device (rtl_433 `-R`) | slicer | short / long / gap / reset / tolerance / sync (µs) |
//! |---|---|---|
//! | Sync, 833 bit/s (306) | OOK PWM | 400 / 800 / 1200 / 7200 / 140 / 4000 |
//! | Sync, 1667 bit/s (307) | OOK PWM | 200 / 400 / 600 / 13600 / 70 / 2000 |
//! | No Sync, 833 bit/s (308) | OOK PWM | 200 / 400 / 1200 / 6800 / 140 / – |
//! | No Sync, 1667 bit/s (309) | OOK PWM | 100 / 200 / 600 / 6600 / 70 / – |
//! | PIWM, 2500 bit/s (310) | OOK PIWM_DC | 400 / 800 / – / 45600 / 140 |
//! | PIWM, 5000 bit/s (311) | OOK PIWM_DC | 200 / 400 / – / 45200 / 70 |
//!
//! Each runs its own slicer over the pulses (`rtl433`), as rtl_433 runs every enabled
//! device on a package; the decode is reported by the first that completes one (on a shared
//! pulse, in the order above). PWM: short pulse = 1, long = 0 (bit = pulse + gap); PIWM_DC: every
//! pulse and every gap is a symbol, short = 1, long = 0.
//!
//! **Frame** (`hcs361_decode`): row 0 = preamble, row 1 = exactly 67 bits, LSB-first fields:
//! 32-bit encrypted hop | 28-bit serial | 4-bit button (S3, S0, S1, S2) | VLOW | 2-bit CRC.
//! Checks, as the C: a 6-bit preamble row must be 0xFC, a 7-bit one 0xFE, a 12-bit one 0xFFF0
//! (PWM) or 0xAAA0 (PIWM) (the C checks no other preamble length; KAT holds every length to the
//! same all-1s / 1010… rule, see [preamble_is_plausible]); the data must not be all
//! 0xFF; the 2-bit CRC over the 65 bits (accepting the battery-low variant of the last bit, as
//! the datasheet suggests); the serial must not be 0. Frames failing a check are dropped.
//!
//! **Fields**: the first 64 data bits are the KeeLoq code word an HCS200 sends, so the frame goes
//! through KAT's KeeLoq identification ([super::rtl433::keeloq_chip_signal]): shown as
//! `Microchip-HCS361 [KeeLoq (<manufacturer>)]` with the decrypted counter when a keystore key
//! fits, else `Microchip-HCS361 [KeeLoq (Unknown)]`. `serial` = rtl_433 `id` (28 bits); `button`
//! = KeeLoq's function code (`fix >> 28` = S2 S1 S0 S3 from bit 3 down), which rtl_433 prints
//! reordered as S3 S2 S1 S0: rtl `button` = `(btn & 1) << 3 | btn >> 1`; `counter` = the decrypted
//! counter (identified frames); `data` = the first 64 wire bits (hop, then serial and button; the
//! rtl_433 `encrypted` value is their first 32 bits reversed), `extra` = VLOW, CRC bits (3 bits),
//! `data_count_bit` = 67; `crc_valid` = the CRC passed (always, for a reported frame). rtl_433's
//! `battery_ok` (the VLOW bit) is in `extra` bit 2.
//!
//! **Encoder**: none — receive only, as rtl_433 (KAT's KeeLoq encoder sends the HCS200 coding).

use super::rtl433::{keeloq_chip_signal, reverse8, BitBuffer, Decode, Device, Ends, Gaps, Modulation, Receiver};
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// rtl_433's model name.
pub(super) const NAME: &str = "Microchip-HCS361";

/// Metadata (see `info.rs`): OOK, KeeLoq, receive only.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[
        "Microchip HCS361 KeeLoq Hopping Encoder based remotes (Sync, 833 bit/s)",
        "Microchip HCS361 KeeLoq Hopping Encoder based remotes (Sync, 1667 bit/s)",
        "Microchip HCS361 KeeLoq Hopping Encoder based remotes (No Sync, 833 bit/s)",
        "Microchip HCS361 KeeLoq Hopping Encoder based remotes (No Sync, 1667 bit/s)",
        "Microchip HCS361 KeeLoq Hopping Encoder based remotes (PIWM, 2500 bit/s)",
        "Microchip HCS361 KeeLoq Hopping Encoder based remotes (PIWM, 5000 bit/s)",
    ],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "KeeLoq",
    make: "KeeLoq",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&AM650],
};

/// Data row length (64 + VLOW + 2 CRC bits).
const FRAME_BITS: usize = 67;

/// An `r_device` of the chip; KAT's stream is read with merged gaps, OOK packages and resync (see
/// `rtl433`).
const fn dev(modulation: Modulation, short_width: u32, long_width: u32, gap_limit: u32, reset_limit: u32, tolerance: u32, sync_width: u32) -> Device {
    Device { modulation, short_width, long_width, reset_limit, gap_limit, sync_width, tolerance, gaps: Gaps::Merged, ends: Ends::Ook, resync: true }
}

/// The six `r_device`s, in rtl_433's order.
const DEVICES: [Device; 6] = [
    dev(Modulation::OokPwm, 400, 800, 1200, 7200, 140, 4000),    // hcs361_txwak_0_bsel_0: Sync, 833 bit/s
    dev(Modulation::OokPwm, 200, 400, 600, 13600, 70, 2000),     // hcs361_txwak_0_bsel_1: Sync, 1667 bit/s
    dev(Modulation::OokPwm, 200, 400, 1200, 6800, 140, 0),       // hcs361_txwak_1_bsel_0: No Sync, 833 bit/s
    dev(Modulation::OokPwm, 100, 200, 600, 6600, 70, 0),         // hcs361_txwak_1_bsel_1: No Sync, 1667 bit/s
    dev(Modulation::OokPiwmDc, 400, 800, 4000, 45600, 140, 4000), // hcs361_vpwm_1_bsel_0: PIWM, 2500 bit/s
    dev(Modulation::OokPiwmDc, 200, 400, 2000, 45200, 70, 2000), // hcs361_vpwm_1_bsel_1: PIWM, 5000 bit/s
];

/// KAT gate (not in the C): the preamble row is all 1s (PWM) or 1010… (PIWM), which the C requires
/// of the 6-, 7- and 12-bit rows it checks, at any length. Without it, the PWM r_device with sync
/// reads one row in four of Opel V1's 120/240 µs Manchester (IMPORTS `HOLDEN/Holden Barina 2016
/// AU.sub`) as a 67-bit frame that passes the 2-bit CRC (and a chance KeeLoq key); every
/// rtl_433_tests HCS361 recording has a 6-, 7-, 12- or 19-bit preamble of that form.
fn preamble_is_plausible(row: &[u8], bits: usize) -> bool {
    let bit = |i: usize| (row[i / 8] >> (7 - i % 8)) & 1 == 1;
    (0..bits).all(bit) || (0..bits).all(|i| bit(i) == (i % 2 == 0))
}

/// `hcs361_decode`: the checks of the module doc on an End-of-Message bit buffer.
fn hcs361_decode(bits: &BitBuffer) -> Option<DecodedSignal> {
    if bits.num_rows() < 2 || bits.bits_per_row(1) != FRAME_BITS {
        return None; // DECODE_ABORT_LENGTH
    }
    let r0 = bits.row(0);
    match bits.bits_per_row(0) {
        // Sync: 6-bit preamble row.
        6 if r0[0] != 0xFC => return None,
        // No sync, short header (truncated all-ones).
        7 if r0[0] != 0xFE => return None,
        // No sync: PIWM 0xAAA0 or PWM 0xFFF0.
        12 => {
            let preamble = u16::from(r0[0]) << 8 | u16::from(r0[1]);
            if preamble != 0xAAA0 && preamble != 0xFFF0 {
                return None;
            }
        }
        // KAT gate (not in the C): a preamble row of any other length is held to the same rule.
        n if !preamble_is_plausible(r0, n) => return None,
        _ => {}
    }
    let b = bits.row(1);
    if b[..8].iter().all(|&x| x == 0xFF) {
        return None; // DECODE_FAIL_SANITY: data all 0xff
    }
    let actual_crc = i32::from((b[8] >> 5) & 0x3);
    let mut crc: i32 = 0;
    let mut crc_bat_low: i32 = 0;
    for i in 0..65 {
        let bit = i32::from(b[i / 8] >> (7 - (i % 8)));
        let crc_bit = ((crc >> 1) ^ bit) & 0x1;
        // The datasheet recommends checking the final CRC bit for a flipped battery-low flag.
        if i == 64 {
            let crc_bit_bat_low = ((crc >> 1) ^ !bit) & 0x1;
            crc_bat_low = crc_bit_bat_low | (((crc_bit_bat_low ^ crc) << 1) & 0x2);
        }
        crc = crc_bit | (((crc_bit ^ crc) << 1) & 0x2);
    }
    if actual_crc != crc && actual_crc != crc_bat_low {
        return None; // DECODE_FAIL_MIC
    }
    let serial = u32::from(reverse8(b[7] & 0xF0)) << 24
        | u32::from(reverse8(b[6])) << 16
        | u32::from(reverse8(b[5])) << 8
        | u32::from(reverse8(b[4]));
    if serial == 0 {
        return None; // DECODE_FAIL_SANITY: serial all 0x00
    }
    Some(keeloq_chip_signal(NAME, b, FRAME_BITS))
}

/// Microchip HCS361 decoder (rtl_433 `hcs361.c`, all six `r_device`s).
pub struct MicrochipHcs361Decoder {
    /// One receiver per `r_device`, in rtl_433's order.
    variants: Vec<Receiver>,
}

impl MicrochipHcs361Decoder {
    pub fn new() -> Self {
        Self { variants: DEVICES.iter().map(Receiver::new).collect() }
    }
}

#[cfg(test)]
impl MicrochipHcs361Decoder {
    /// One `r_device` only (index in the module doc's table), for the oracle comparisons.
    pub(super) fn only(variant: usize) -> Self {
        let mut d = Self::new();
        let v = d.variants.swap_remove(variant);
        d.variants = vec![v];
        d
    }
}

impl Default for MicrochipHcs361Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for MicrochipHcs361Decoder {
    fn name(&self) -> &'static str {
        NAME
    }

    /// The No Sync, 833 bit/s timing (the variant that reads the Jeep remotes of rtl_433_tests).
    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: 200, te_long: 400, te_delta: 140, min_count_bit: FRAME_BITS }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        for v in &mut self.variants {
            v.reset(&mut Decode(hcs361_decode));
        }
    }

    fn feed(&mut self, level: bool, duration_us: u32) -> Option<DecodedSignal> {
        let mut out = None;
        for v in &mut self.variants {
            let r = v.feed(&mut Decode(hcs361_decode), level, duration_us);
            out = out.or(r);
        }
        if out.is_some() {
            // The reset that follows keeps every r_device's package, as in rtl_433.
            for v in &mut self.variants {
                v.own_report();
            }
        }
        out
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::keeloq_common::{keeloq_encrypt, keeloq_keystore, keeloq_magic_serial_type3_learning};
    use crate::protocols::rtl433::test_util::{decode_all, expected_json, rtl433_tests};

    /// The 67 data bits (LSB-first fields) of a frame, CRC computed as the decoder checks it.
    fn frame_bits(hop: u32, serial: u32, btn: u8, vlow: u8) -> Vec<bool> {
        let fix = (u32::from(btn) << 28) | (serial & 0x0FFF_FFFF);
        let mut bits: Vec<bool> = (0..32).map(|i| (hop >> i) & 1 == 1).collect();
        bits.extend((0..32).map(|i| (fix >> i) & 1 == 1));
        bits.push(vlow == 1);
        let mut crc: i32 = 0;
        for &bit in &bits {
            let crc_bit = ((crc >> 1) ^ i32::from(bit)) & 1;
            crc = crc_bit | (((crc_bit ^ crc) << 1) & 2);
        }
        bits.push((crc >> 1) & 1 == 1);
        bits.push(crc & 1 == 1);
        bits
    }

    /// PWM upload: `pre` preamble 1s, then (sync) a 10·TE pulse + 10·TE header gap, or (no
    /// sync) the header gap after the preamble; the data; then silence.
    fn pwm_wire(te: u32, pre: usize, sync: bool, bits: &[bool]) -> Vec<LevelDuration> {
        let mut w = vec![LevelDuration::new(false, 50_000)];
        for i in 0..pre {
            w.push(LevelDuration::new(true, te));
            let last = i + 1 == pre;
            w.push(LevelDuration::new(false, if last && !sync { 10 * te } else { 2 * te }));
        }
        if sync {
            w.push(LevelDuration::new(true, 10 * te));
            w.push(LevelDuration::new(false, 10 * te));
        }
        for &b in bits {
            let (h, l) = if b { (te, 2 * te) } else { (2 * te, te) };
            w.push(LevelDuration::new(true, h));
            w.push(LevelDuration::new(false, l));
        }
        w.push(LevelDuration::new(false, 50_000));
        w
    }

    /// PIWM upload: preamble symbols 1010…10 (12), a 10·TE pulse + 10·TE gap, then each data bit
    /// as one symbol (pulses and gaps alternating), then silence.
    fn piwm_wire(te: u32, bits: &[bool]) -> Vec<LevelDuration> {
        let mut w = vec![LevelDuration::new(false, 60_000)];
        for i in 0..12 {
            w.push(LevelDuration::new(i % 2 == 0, if i % 2 == 0 { te } else { 2 * te }));
        }
        w.push(LevelDuration::new(true, 10 * te));
        w.push(LevelDuration::new(false, 10 * te));
        for (i, &b) in bits.iter().enumerate() {
            w.push(LevelDuration::new(i % 2 == 0, if b { te } else { 2 * te }));
        }
        w.push(LevelDuration::new(false, 50_000));
        w
    }

    fn rtl_button(btn: u8) -> u8 {
        ((btn & 1) << 3) | (btn >> 1)
    }

    #[test]
    fn decodes_every_variant_and_identifies_the_keeloq_manufacturer() {
        crate::protocols::keys::load_keystore_from_embedded();
        // A Magic_3 remote (the HCS200 key the rtl_433_tests HCS200 recordings decrypt with).
        let entry = keeloq_keystore().iter().find(|e| e.name == "Magic_3").expect("Magic_3");
        let serial = 0x0D_0921;
        let btn = 0x2;
        let fix = (u32::from(btn) << 28) | serial;
        let plain = (u32::from(btn) << 28) | ((serial & 0x3FF) << 16) | 0x0ABE;
        let hop = keeloq_encrypt(plain, keeloq_magic_serial_type3_learning(fix, entry.key));
        let bits = frame_bits(hop, serial, btn, 0);
        let wires = [
            ("Sync 833", pwm_wire(400, 6, true, &bits)),
            ("Sync 1667", pwm_wire(200, 6, true, &bits)),
            ("No Sync 833", pwm_wire(200, 10, false, &bits)),
            ("No Sync 1667", pwm_wire(100, 10, false, &bits)),
            ("PIWM 2500", piwm_wire(400, &bits)),
            ("PIWM 5000", piwm_wire(200, &bits)),
        ];
        for (label, wire) in wires {
            let got = decode_all(&mut MicrochipHcs361Decoder::new(), &wire);
            assert_eq!(got.len(), 1, "{label}: {got:?}");
            let d = &got[0];
            assert_eq!(d.protocol_display_name.as_deref(), Some("Microchip-HCS361 [KeeLoq (Magic_3)]"), "{label}");
            assert_eq!((d.serial, d.button, d.counter), (Some(serial), Some(btn), Some(0x0ABE)), "{label}");
            assert!(d.crc_valid && !d.encoder_capable, "{label}");
            assert_eq!((d.data_count_bit, d.extra.map(|e| e >> 2)), (67, Some(0)), "{label}");
        }
    }

    #[test]
    fn unknown_remotes_report_serial_button_and_hop() {
        crate::protocols::keys::load_keystore_from_embedded();
        for (btn, vlow) in [(0x1u8, 0u8), (0x2, 1), (0x4, 0), (0x8, 1), (0xA, 0), (0xF, 0)] {
            let bits = frame_bits(0xD6D5_8B8A, 0x00A0_145B, btn, vlow);
            let got = decode_all(&mut MicrochipHcs361Decoder::new(), &pwm_wire(200, 7, false, &bits));
            assert_eq!(got.len(), 1, "btn {btn:X}");
            let d = &got[0];
            assert_eq!((d.serial, d.button), (Some(0x00A0_145B), Some(btn)));
            if d.counter.is_none() {
                assert_eq!(d.protocol_display_name.as_deref(), Some("Microchip-HCS361 [KeeLoq (Unknown)]"));
            }
            // rtl_433's `encrypted`: the first 32 wire bits, reversed.
            assert_eq!(((d.data >> 32) as u32).reverse_bits(), 0xD6D5_8B8A);
            assert_eq!(d.extra.map(|e| (e >> 2) as u8), Some(vlow));
        }
    }

    #[test]
    fn failed_checks_are_dropped() {
        let bits = frame_bits(0x1234_5678, 0x0123_4567, 0x2, 0);
        // CRC: flip one CRC bit (the battery-low variant only covers the VLOW bit).
        let mut bad = bits.clone();
        bad[66] ^= true;
        assert!(decode_all(&mut MicrochipHcs361Decoder::new(), &pwm_wire(200, 10, false, &bad)).is_empty());
        // A flipped VLOW bit still passes (rtl_433 accepts the battery-low CRC).
        let mut vlow = bits.clone();
        vlow[64] ^= true;
        assert_eq!(decode_all(&mut MicrochipHcs361Decoder::new(), &pwm_wire(200, 10, false, &vlow)).len(), 1);
        // Serial 0.
        let zero = frame_bits(0x1234_5678, 0, 0x2, 0);
        assert!(decode_all(&mut MicrochipHcs361Decoder::new(), &pwm_wire(200, 10, false, &zero)).is_empty());
        // 66 data bits.
        assert!(decode_all(&mut MicrochipHcs361Decoder::new(), &pwm_wire(200, 10, false, &bits[..66])).is_empty());
        // A 6-bit preamble row that is not all ones (a long pulse inside it).
        let mut w = pwm_wire(200, 6, true, &bits);
        w[3] = LevelDuration::new(true, 400);
        assert!(decode_all(&mut MicrochipHcs361Decoder::new(), &w).is_empty());
        // KAT gate: a preamble length the C doesn't check (10 bits) is held to the same rule.
        assert!(!decode_all(&mut MicrochipHcs361Decoder::new(), &pwm_wire(200, 10, false, &bits)).is_empty());
        let mut w = pwm_wire(200, 10, false, &bits);
        w[3] = LevelDuration::new(true, 400);
        assert!(decode_all(&mut MicrochipHcs361Decoder::new(), &w).is_empty());
        // Timing off by 40 %: no variant reads it.
        let w: Vec<LevelDuration> = pwm_wire(200, 10, false, &bits).iter().map(|l| LevelDuration::new(l.level, l.duration_us * 7 / 5)).collect();
        let got = decode_all(&mut MicrochipHcs361Decoder::new(), &w);
        assert!(got.is_empty(), "{got:?}");
    }

    /// rtl_433_tests `Microchip/HCS361`: every recording through KAT's IQ pipeline and the
    /// registry; every distinct rtl_433 frame (id, button, encrypted) must be decoded as
    /// Microchip-HCS361 with the same serial, button (KeeLoq order) and hop. `04/unlock` starts
    /// with ~87 ms of zero bytes and a settling receiver (the RTL-SDR's start-up), which KAT's
    /// receiver does not learn its noise floor from.
    #[test]
    #[ignore]
    fn rtl_433_tests_recordings() {
        let Some(root) = rtl433_tests() else {
            eprintln!("Skip: rtl_433_tests not found");
            return;
        };
        crate::protocols::keys::load_keystore_from_embedded();
        let dir = root.join("Microchip/HCS361");
        if !dir.is_dir() {
            return;
        }
        let mut files = Vec::new();
        for sub in std::fs::read_dir(&dir).unwrap().flatten().filter(|e| e.path().is_dir()) {
            for f in std::fs::read_dir(sub.path()).unwrap().flatten() {
                if f.path().extension().is_some_and(|e| e == "cu8") {
                    files.push(f.path());
                }
            }
        }
        files.sort();
        assert_eq!(files.len(), 10);
        let mut reg = crate::protocols::ProtocolRegistry::new();
        let mut failures = Vec::new();
        for cu8 in &files {
            let r = crate::headless::decode_file(&mut reg, cu8, None, None, false).unwrap();
            let decodes: Vec<(String, u64)> = r.decodes.into_iter().map(|d| (d.protocol, d.data)).collect();
            let got: Vec<(u32, u8, u32)> = decodes
                .iter()
                .filter(|(n, _)| n.starts_with("Microchip-HCS361 ["))
                .map(|&(_, data)| {
                    let (fix, hop) = crate::protocols::keeloq::fix_hop(data);
                    (fix & 0x0FFF_FFFF, rtl_button((fix >> 28) as u8), hop)
                })
                .collect();
            for v in expected_json(cu8).iter().filter(|v| v["model"] == NAME) {
                let id = u32::from_str_radix(v["id"].as_str().unwrap(), 16).unwrap();
                let button = v["button"].as_u64().unwrap() as u8;
                let hop = u32::from_str_radix(v["encrypted"].as_str().unwrap(), 16).unwrap();
                if !got.contains(&(id, button, hop)) {
                    failures.push(format!("{}: rtl_433 {id:08X} btn {button} {hop:08X}, KAT {got:X?}", cu8.display()));
                }
            }
            // Nothing else decodes these recordings.
            let others: Vec<&str> = decodes.iter().map(|(n, _)| n.as_str()).filter(|p| !p.starts_with("Microchip-HCS361")).collect();
            if !others.is_empty() {
                failures.push(format!("{}: also {others:?}", cu8.display()));
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }
}
