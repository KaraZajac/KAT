//! Microchip HCS362 KeeLoq code-hopping encoder (receive only).
//!
//! Port of rtl_433 `src/devices/hcs362.c` (`REFERENCES/rtl_433`, GPL-2.0; model
//! "Microchip-HCS362"), its two `r_device`s with one decoder:
//!
//! | r_device (rtl_433 `-R`) | slicer | short / long / gap / reset / tolerance (µs) |
//! |---|---|---|
//! | Microchip HCS362 KeeLoq PWM (329) | OOK PWM | 200 / 400 / 550 / 900 / 50 |
//! | Microchip HCS362 KeeLoq MC (330) | OOK PCM | 214 / 214 / 600 / 900 / 50 |
//!
//! Each runs its own slicer (`rtl433`); the first to complete a frame reports it.
//!
//! **PWM mode**: a 12-pulse preamble row (0xFFF, checked), then exactly 69 data bits (short = 1).
//! **MC mode**: the PCM slicer measures the bit period on the 12-pulse preamble (its row must be
//! 16..32 raw bits starting 0xAAAAAA), a long gap starts the data row (142..148 raw bits, starting
//! with the "10" start bit); the row is Manchester-decoded from raw bit 2 (at most 72 bits, the
//! decoder needs its input position ≥ 70 — the C's check) and inverted (G.E. Thomas: HIGH-LOW = 1).
//!
//! **Frame** (both modes, `hcs362_decode`), LSB-first fields: 32-bit encrypted hop | 28-bit
//! serial | 4-bit function code (S3, S0, S1, S2) | VLOW | CRC0, CRC1 | QUEUE0, QUEUE1. Checks, as
//! the C: bytes 1..7 not all 0xFF; the 2-bit CRC (datasheet Equation 3-1 over the first 65 bits).
//! Frames failing a check are dropped.
//!
//! **Fields**: as Microchip-HCS361 ([super::rtl433::keeloq_chip_signal]): shown as
//! `Microchip-HCS362 [KeeLoq (<manufacturer>)]` (counter decrypted) or `[KeeLoq (Unknown)]`;
//! `serial` = rtl_433 `id`; `button` = KeeLoq's function code (rtl `button` =
//! `(btn & 1) << 3 | btn >> 1`); `data` = the first 64 frame bits, `extra` = VLOW, CRC0, CRC1,
//! QUEUE0, QUEUE1 (5 bits: rtl_433's `battery_ok` = !VLOW, `repeat` = QUEUE1 << 1 | QUEUE0),
//! `data_count_bit` = 69; `crc_valid` = the CRC passed.
//!
//! **Encoder**: none — receive only, as rtl_433.

use super::rtl433::{keeloq_chip_signal, BitBuffer, Decode, Device, Ends, Gaps, Modulation, Receiver};
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// rtl_433's model name.
pub(super) const NAME: &str = "Microchip-HCS362";

/// Metadata (see `info.rs`): OOK, KeeLoq, receive only (PWM or Manchester per the chip's EEPROM).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &["Microchip HCS362 KeeLoq PWM", "Microchip HCS362 KeeLoq MC"],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "KeeLoq",
    make: "KeeLoq",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&AM650],
};

/// Frame length (64 + VLOW + 2 CRC + 2 queue bits).
const FRAME_BITS: usize = 69;

/// `hcs362_pwm`.
const PWM: Device = Device {
    modulation: Modulation::OokPwm,
    short_width: 200,
    long_width: 400,
    reset_limit: 900,
    gap_limit: 550,
    sync_width: 0,
    tolerance: 50,
    gaps: Gaps::Merged,
    ends: Ends::Ook,
    resync: true,
};
/// `hcs362_mc`.
/// The PCM slicer needs the whole package (it measures the bit period on it).
const MC: Device = Device {
    modulation: Modulation::OokPcm,
    short_width: 214,
    long_width: 214,
    reset_limit: 900,
    gap_limit: 600,
    sync_width: 0,
    tolerance: 50,
    gaps: Gaps::Merged,
    ends: Ends::Ook,
    resync: true,
};

/// `hcs362_crc`: datasheet Equation 3-1 over the first 65 bits; `CRC1 << 1 | CRC0`.
fn hcs362_crc(b: &[u8]) -> u8 {
    let (mut crc0, mut crc1) = (0u8, 0u8);
    for n in 0..65 {
        let d = (b[n / 8] >> (7 - (n % 8))) & 1;
        let next_crc1 = crc0 ^ d;
        let next_crc0 = crc0 ^ d ^ crc1;
        crc0 = next_crc0;
        crc1 = next_crc1;
    }
    (crc1 << 1) | crc0
}

/// `hcs362_decode` (`mc`: the OOK_PULSE_PCM device).
fn hcs362_decode(bits: &BitBuffer, mc: bool) -> Option<DecodedSignal> {
    let mut msg = BitBuffer::new();
    let b: &[u8] = if mc {
        // Preamble: 12 short pulses (24 raw bits ± 8).
        if !(16..=32).contains(&bits.bits_per_row(0)) {
            return None; // DECODE_ABORT_LENGTH
        }
        let r0 = bits.row(0);
        if r0[0] != 0xAA || r0[1] != 0xAA || r0[2] != 0xAA {
            return None; // DECODE_ABORT_EARLY: preamble invalid
        }
        // 71 data + start/stop bits, MC coded (the stop bit usually truncated to one raw bit).
        if !(71 * 2..=72 * 2 + 4).contains(&bits.bits_per_row(1)) {
            return None;
        }
        if bits.row(1)[0] & 0xC0 != 0x80 {
            return None; // start bit not found
        }
        // MC decode, excluding the start bit.
        let len = bits.manchester_decode(1, 2, &mut msg, 72);
        if len < 69 + 1 {
            return None;
        }
        msg.invert(); // G.E. Thomas, not IEEE 802.3
        msg.row(0)
    } else {
        if bits.bits_per_row(0) != 12 || bits.bits_per_row(1) != FRAME_BITS {
            return None;
        }
        let r0 = bits.row(0);
        if r0[0] != 0xFF || r0[1] & 0xF0 != 0xF0 {
            return None; // preamble not found
        }
        bits.row(1)
    };
    if b[1..8].iter().all(|&x| x == 0xFF) {
        return None; // DECODE_FAIL_SANITY: data all 0xff
    }
    // Status byte: bit 64 VLOW, 65 CRC0, 66 CRC1, 67 QUEUE0, 68 QUEUE1.
    let actual_crc = ((b[8] >> 6) & 1) | (((b[8] >> 5) & 1) << 1);
    if actual_crc != hcs362_crc(b) {
        return None; // DECODE_FAIL_MIC
    }
    Some(keeloq_chip_signal(NAME, b, FRAME_BITS))
}

/// Microchip HCS362 decoder (rtl_433 `hcs362.c`, PWM and MC `r_device`s).
pub struct MicrochipHcs362Decoder {
    /// The `r_device`s run: [PWM, MC] (tests switch one off).
    enabled: [bool; 2],
    pwm: Receiver,
    mc: Receiver,
}

/// `hcs362_decode` for each `r_device`.
fn decode_pwm(bits: &BitBuffer) -> Option<DecodedSignal> {
    hcs362_decode(bits, false)
}

fn decode_mc(bits: &BitBuffer) -> Option<DecodedSignal> {
    hcs362_decode(bits, true)
}

impl MicrochipHcs362Decoder {
    pub fn new() -> Self {
        Self { enabled: [true, true], pwm: Receiver::new(&PWM), mc: Receiver::new(&MC) }
    }
}

#[cfg(test)]
impl MicrochipHcs362Decoder {
    /// One `r_device` only (`mc`: hcs362_mc, else hcs362_pwm), for the oracle comparisons.
    pub(super) fn only(mc: bool) -> Self {
        Self { enabled: [!mc, mc], ..Self::new() }
    }
}

impl Default for MicrochipHcs362Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for MicrochipHcs362Decoder {
    fn name(&self) -> &'static str {
        NAME
    }

    /// The PWM device's timing.
    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: 200, te_long: 400, te_delta: 50, min_count_bit: FRAME_BITS }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.pwm.reset(&mut Decode(decode_pwm));
        self.mc.reset(&mut Decode(decode_mc));
    }

    fn feed(&mut self, level: bool, duration_us: u32) -> Option<DecodedSignal> {
        let mut out = None;
        if self.enabled[0] {
            out = self.pwm.feed(&mut Decode(decode_pwm), level, duration_us);
        }
        if self.enabled[1] {
            let r = self.mc.feed(&mut Decode(decode_mc), level, duration_us);
            out = out.or(r);
        }
        if out.is_some() {
            // The reset that follows keeps both r_devices' packages, as in rtl_433.
            self.pwm.own_report();
            self.mc.own_report();
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
    use crate::protocols::rtl433::test_util::{decode_all, expected_json, nrz, rtl433_tests};

    /// The 69 frame bits (LSB-first fields), CRC per the datasheet.
    fn frame_bits(hop: u32, serial: u32, btn: u8, vlow: u8, queue: u8) -> Vec<bool> {
        let fix = (u32::from(btn) << 28) | (serial & 0x0FFF_FFFF);
        let mut bits: Vec<bool> = (0..32).map(|i| (hop >> i) & 1 == 1).collect();
        bits.extend((0..32).map(|i| (fix >> i) & 1 == 1));
        bits.push(vlow == 1);
        let mut bytes = [0u8; 9];
        for (i, &b) in bits.iter().enumerate() {
            bytes[i / 8] |= u8::from(b) << (7 - i % 8);
        }
        let crc = hcs362_crc(&bytes);
        bits.extend([crc & 1, crc >> 1, queue & 1, (queue >> 1) & 1].map(|b| b == 1));
        bits
    }

    /// PWM upload: 12 × (TE, TE) preamble, the last gap 3·TE, then 1 = TE + 2·TE, 0 = 2·TE + TE.
    fn pwm_wire(te: u32, bits: &[bool]) -> Vec<LevelDuration> {
        let mut w = vec![LevelDuration::new(false, 50_000)];
        for i in 0..12 {
            w.push(LevelDuration::new(true, te));
            w.push(LevelDuration::new(false, if i == 11 { 3 * te } else { te }));
        }
        for &b in bits {
            let (h, l) = if b { (te, 2 * te) } else { (2 * te, te) };
            w.push(LevelDuration::new(true, h));
            w.push(LevelDuration::new(false, l));
        }
        w.push(LevelDuration::new(false, 70_000));
        w
    }

    /// MC upload: 12 × (TE, TE) preamble, the last gap 4·TE, start bit 1, data (1 = HIGH-LOW),
    /// a stop half-bit HIGH, then the guard.
    fn mc_wire(te: u32, bits: &[bool]) -> Vec<LevelDuration> {
        let mut raw: Vec<bool> = Vec::new();
        for i in 0..12 {
            raw.extend([true, false]);
            if i == 11 {
                raw.extend([false, false, false]);
            }
        }
        raw.extend([true, false]);
        for &b in bits {
            raw.extend([b, !b]);
        }
        raw.push(true);
        let mut w = vec![LevelDuration::new(false, 50_000)];
        w.extend(nrz(&raw, te));
        w.push(LevelDuration::new(false, 70_000));
        w
    }

    fn magic_3_frame(btn: u8, counter: u16) -> (u32, Vec<bool>) {
        let entry = keeloq_keystore().iter().find(|e| e.name == "Magic_3").expect("Magic_3");
        let serial = 0x0010_9C11;
        let fix = (u32::from(btn) << 28) | serial;
        let plain = (u32::from(btn) << 28) | ((serial & 0x3FF) << 16) | u32::from(counter);
        let hop = keeloq_encrypt(plain, keeloq_magic_serial_type3_learning(fix, entry.key));
        (serial, frame_bits(hop, serial, btn, 0, 0))
    }

    #[test]
    fn decodes_pwm_and_mc_frames() {
        crate::protocols::keys::load_keystore_from_embedded();
        for btn in [0x1u8, 0x2, 0x4, 0x8, 0x6] {
            let (serial, bits) = magic_3_frame(btn, 0x1234);
            for (label, wire) in [("PWM", pwm_wire(200, &bits)), ("PWM 188", pwm_wire(188, &bits)), ("MC", mc_wire(214, &bits)), ("MC 216", mc_wire(216, &bits))] {
                let got = decode_all(&mut MicrochipHcs362Decoder::new(), &wire);
                assert_eq!(got.len(), 1, "{label} btn {btn}: {got:?}");
                let d = &got[0];
                assert_eq!(d.protocol_display_name.as_deref(), Some("Microchip-HCS362 [KeeLoq (Magic_3)]"), "{label}");
                assert_eq!((d.serial, d.button, d.counter), (Some(serial), Some(btn), Some(0x1234)), "{label}");
                assert_eq!((d.data_count_bit, d.extra), (69, Some(u64::from(bits[65]) << 3 | u64::from(bits[66]) << 2)), "{label}");
                assert!(d.crc_valid && !d.encoder_capable);
            }
        }
        // VLOW and the queue bits are carried in `extra`.
        let bits = frame_bits(0x875C_5D67, 0x0010_9C11, 0x6, 1, 2);
        let d = &decode_all(&mut MicrochipHcs362Decoder::new(), &mc_wire(214, &bits))[0];
        assert_eq!(d.extra.map(|e| (e >> 4, e & 3)), Some((1, 0b01)));
    }

    #[test]
    fn failed_checks_are_dropped() {
        let bits = frame_bits(0x1234_5678, 0x0A55_AA55, 0x4, 0, 0);
        for i in [0usize, 31, 40, 64, 65, 66] {
            let mut bad = bits.clone();
            bad[i] ^= true;
            // One flipped bit: the 2-bit CRC catches it (and a flipped CRC bit fails it).
            for wire in [pwm_wire(200, &bad), mc_wire(214, &bad)] {
                assert!(decode_all(&mut MicrochipHcs362Decoder::new(), &wire).is_empty(), "bit {i}");
            }
        }
        // 68 bits.
        assert!(decode_all(&mut MicrochipHcs362Decoder::new(), &pwm_wire(200, &bits[..68])).is_empty());
        // All-0xFF data.
        let ones = vec![true; 69];
        assert!(decode_all(&mut MicrochipHcs362Decoder::new(), &pwm_wire(200, &ones)).is_empty());
        // MC frame without the start bit.
        let mut w = mc_wire(214, &bits);
        let start = w.iter().position(|l| !l.level && l.duration_us == 4 * 214).unwrap();
        w[start + 1].duration_us = 214 * 3;
        assert!(decode_all(&mut MicrochipHcs362Decoder::new(), &w).is_empty());
    }

    /// rtl_433_tests `hcs362/01`: each recording (PWM and MC mode) through KAT's IQ pipeline; the
    /// rtl_433 frame (id, button, encrypted) must be decoded with the same serial, button (KeeLoq
    /// order) and hop.
    #[test]
    #[ignore]
    fn rtl_433_tests_recordings() {
        let Some(root) = rtl433_tests() else {
            eprintln!("Skip: rtl_433_tests not found");
            return;
        };
        crate::protocols::keys::load_keystore_from_embedded();
        let dir = root.join("hcs362/01");
        if !dir.is_dir() {
            return;
        }
        let mut files: Vec<_> = std::fs::read_dir(&dir).unwrap().flatten().map(|e| e.path()).filter(|p| p.extension().is_some_and(|e| e == "cu8")).collect();
        files.sort();
        assert_eq!(files.len(), 4);
        let mut reg = crate::protocols::ProtocolRegistry::new();
        let mut failures = Vec::new();
        for cu8 in &files {
            let r = crate::headless::decode_file(&mut reg, cu8, None, None, false).unwrap();
            let got: Vec<(u32, u8, u32)> = r
                .decodes
                .iter()
                .filter(|d| d.protocol.starts_with("Microchip-HCS362 ["))
                .map(|d| {
                    let (fix, hop) = crate::protocols::keeloq::fix_hop(d.data);
                    (fix & 0x0FFF_FFFF, (((fix >> 28) as u8 & 1) << 3) | ((fix >> 28) as u8 >> 1), hop)
                })
                .collect();
            let want = &expected_json(cu8)[0];
            let id = u32::from_str_radix(want["id"].as_str().unwrap(), 16).unwrap();
            let button = want["button"].as_u64().unwrap() as u8;
            let hop = u32::from_str_radix(want["encrypted"].as_str().unwrap(), 16).unwrap();
            // rtl_433 reports each of the 8 repeats.
            let n = got.iter().filter(|&&g| g == (id, button, hop)).count();
            if n < 8 {
                failures.push(format!("{}: rtl_433 {id:07X} btn {button} {hop:08X} ×8, KAT {n}× ({got:X?})", cu8.display()));
            }
            let others: Vec<&str> = r.decodes.iter().map(|d| d.protocol.as_str()).filter(|p| !p.starts_with("Microchip-HCS362")).collect();
            if !others.is_empty() {
                failures.push(format!("{}: also {others:?}", cu8.display()));
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }
}
