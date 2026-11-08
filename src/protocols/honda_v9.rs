//! Honda V9 protocol decoder (**decode only**, as the reference)
//!
//! Ported from FoxFW `lib/subghz/protocols/honda_v9.c` and `honda_v9.h` (itself a port of the
//! KeeLoq-RX `arke_honda.py` `HondaV9` class); no other reference carries it. The C has no encoder
//! (`.encoder = NULL`), so neither does KAT.
//!
//! **Air format** (FSK, captured as OOK in the source; 315 / 433.92 MHz): Manchester with te
//! 63 / 126 µs, tolerance 25 µs (inclusive). Every pulse is stored as half-bit samples of its
//! level (a short pulse is 1 sample, a long one 2, at most 512 samples). A pulse of neither width
//! ends the burst: with at least 36 samples the burst is decoded and the buffer cleared.
//!
//! **Decoder** (`man_decode`): the first run of alternating samples must be at least 20 long;
//! if it reaches 110 samples (55 preamble cycles) the data starts at sample 110, otherwise at
//! the end of the alternating run (skipping a further run of equal samples). Pairs of samples are
//! then read as Manchester bits, `(0, 1)` = 1 and `(1, 0)` = 0 in normal polarity, the reverse in
//! inverted polarity, until two equal samples, or 112 bits. At least 64 bits must be read and 112
//! are needed to go on. Both polarities are tried (normal first); the first that gives a frame
//! wins.
//!
//! **Frame** (14 bytes, MSB first): `hdr = d[0] >> 4`, `button = d[0] & 0xF`, `serial =
//! d[1] << 12 | d[2] << 4 | d[3] >> 4` (20 bits), `flags = d[3] & 0xF`, `fingerprint = d[4..8]`,
//! `checksum = d[8]`, `counter = d[10..12]`; `d[9]`, `d[12]`, `d[13]` are carried but unread.
//! Emission needs a button of at most 9, a serial that is neither 0 nor 0xFFFFF, a fingerprint
//! that is neither 0 nor 0xFFFFFFFF, and the checksum `XOR(d[0..8])` within Hamming distance 1 of
//! `d[8]`: **a frame one bit off is emitted too**, flagged by `crc_valid` false.
//!
//! **KAT fields**: `data` = `d[0..8]` big-endian (the C `generic.data`), `data_count_bit` 112,
//! `serial`, `button`, `counter`, `crc_valid` = the checksum matches exactly, `extra` = `d[8..14]`
//! (48 bits, the C `tail`). Key files: Protocol "Honda V9", Bit 112, Key, Serial, Btn, Cnt,
//! Fingerprint, CRC, TailHi, TailLo.
//!
//! No capture of this protocol exists in the test corpus: the port is validated against the C
//! with synthetic pulse sequences only.

use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (see `info.rs`): the C's `SubGhzProtocolFlag_433 | 315 | AM | FM`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::Both,
    encryption: "Rolling Code",
    make: "Honda/Acura",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&HONDA1, &AM650],
};

const NAME: &str = "Honda V9";
const TS: u32 = 63;
const TL: u32 = 126;
const TD: u32 = 25;
const MIN_PRE: usize = 20;
const PRE_CYC: usize = 55;
const FRAME_BITS: usize = 112;
const MIN_BITS: usize = 64;
const FRAME_BYTES: usize = 14;
const HB_CAP: usize = 512;

/// Honda V9 decoder (matches SubGhzProtocolDecoderHondaV9).
pub struct HondaV9Decoder {
    /// Half-bit samples, one entry per sample (0 or 1).
    hb: Vec<u8>,
}

/// `honda_v9_classify`: 1 short, 2 long, 0 neither.
fn classify(dur: u32) -> u8 {
    if dur.abs_diff(TS) <= TD {
        1
    } else if dur.abs_diff(TL) <= TD {
        2
    } else {
        0
    }
}

/// `honda_v9_man_decode`: the decoded bytes and the bit count (0 on failure).
fn man_decode(hb: &[u8], invert: bool) -> (usize, [u8; FRAME_BYTES + 2]) {
    let mut decoded = [0u8; FRAME_BYTES + 2];
    let n = hb.len();
    let mut alt_end = 1;
    while alt_end < n && hb[alt_end] != hb[alt_end - 1] {
        alt_end += 1;
    }
    if alt_end == n || alt_end < MIN_PRE {
        return (0, decoded);
    }
    let mut i;
    if alt_end >= PRE_CYC * 2 {
        i = PRE_CYC * 2;
    } else {
        i = alt_end;
        while i + 1 < n && hb[i] == hb[i + 1] {
            i += 1;
        }
    }
    let mut bit_count = 0;
    while i + 1 < n && bit_count < FRAME_BITS {
        let (h0, h1) = (hb[i], hb[i + 1]);
        if h0 == h1 {
            break;
        }
        let bit = if !invert { h0 == 0 && h1 == 1 } else { h0 == 1 && h1 == 0 };
        if bit {
            decoded[bit_count >> 3] |= 1 << (7 - (bit_count & 7));
        }
        bit_count += 1;
        i += 2;
    }
    if bit_count < MIN_BITS {
        return (0, decoded);
    }
    (bit_count, decoded)
}

/// Fields from `data` = `d[0..8]` and `tail` = `d[8..14]`.
fn fields(data: u64, tail: u64) -> DecodedSignal {
    let d = data.to_be_bytes();
    let serial = (u32::from(d[1]) << 12) | (u32::from(d[2]) << 4) | u32::from(d[3] >> 4);
    let xchk = d.iter().fold(0, |a, &b| a ^ b);
    let checksum = (tail >> 40) as u8;
    DecodedSignal {
        serial: Some(serial),
        button: Some(d[0] & 0x0F),
        counter: Some((tail >> 16) as u16),
        crc_valid: xchk == checksum,
        data,
        data_count_bit: FRAME_BITS,
        encoder_capable: false,
        extra: Some(tail & 0xFFFF_FFFF_FFFF),
        protocol_display_name: None,
    }
}

impl HondaV9Decoder {
    pub fn new() -> Self {
        Self { hb: Vec::with_capacity(HB_CAP) }
    }

    /// `honda_v9_try_polarity`.
    fn try_polarity(&self, invert: bool) -> Option<DecodedSignal> {
        let (bit_count, d) = man_decode(&self.hb, invert);
        if bit_count < FRAME_BITS {
            return None;
        }
        let button = d[0] & 0x0F;
        let serial = (u32::from(d[1]) << 12) | (u32::from(d[2]) << 4) | u32::from(d[3] >> 4);
        let fingerprint = u32::from_be_bytes([d[4], d[5], d[6], d[7]]);
        let checksum = d[8];
        let xchk = d[..8].iter().fold(0, |a, &b| a ^ b);
        let dist = (xchk ^ checksum).count_ones();
        if button > 0x9 {
            return None;
        }
        if serial == 0 || serial == 0x000F_FFFF {
            return None;
        }
        if fingerprint == 0 || fingerprint == 0xFFFF_FFFF {
            return None;
        }
        if xchk != checksum && dist > 1 {
            return None;
        }
        let data = u64::from_be_bytes(d[..8].try_into().expect("8 bytes"));
        let tail = d[8..14].iter().fold(0u64, |a, &b| (a << 8) | u64::from(b));
        Some(fields(data, tail))
    }

    /// `honda_v9_try_decode`.
    fn try_decode(&self) -> Option<DecodedSignal> {
        if self.hb.len() < MIN_PRE + 16 {
            return None;
        }
        self.try_polarity(false).or_else(|| self.try_polarity(true))
    }
}

impl Default for HondaV9Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for HondaV9Decoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: TS, te_long: TL, te_delta: TD, min_count_bit: FRAME_BITS }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.hb.clear();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let lvl = u8::from(level);
        match classify(duration) {
            1 => {
                if self.hb.len() < HB_CAP {
                    self.hb.push(lvl);
                }
                None
            }
            2 => {
                if self.hb.len() + 2 <= HB_CAP {
                    self.hb.push(lvl);
                    self.hb.push(lvl);
                }
                None
            }
            _ => {
                let out = if self.hb.len() >= MIN_PRE + 16 { self.try_decode() } else { None };
                self.hb.clear();
                out
            }
        }
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }

    /// Rebuild a decode from a key file written by [Self::to_flipper_key] (Key = `d[0..8]`,
    /// TailHi / TailLo = `d[8..14]`).
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != FRAME_BITS {
            return None;
        }
        let tail = (u64::from(key.u32_field("TailHi")?) << 32) | u64::from(key.u32_field("TailLo")?);
        if tail >> 48 != 0 {
            return None;
        }
        Some(fields(key.key_u64()?, tail))
    }

    /// `subghz_protocol_decoder_honda_v9_serialize`.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != FRAME_BITS {
            return None;
        }
        let tail = decoded.extra?;
        let mut key = FlipperKey::for_protocol(NAME, FRAME_BITS, decoded.data);
        key.push_u32("Serial", decoded.serial?);
        key.push_u32("Btn", decoded.button?.into());
        key.push_u32("Cnt", decoded.counter?.into());
        key.push_u32("Fingerprint", decoded.data as u32);
        key.push_u32("CRC", (tail >> 40) as u32);
        key.push_u32("TailHi", (tail >> 32) as u32);
        key.push_u32("TailLo", tail as u32);
        Some(key)
    }
}

#[cfg(test)]
mod tests {
    use super::super::honda_synth::{bits_of, decode, dump, halves_to_pulses, Pulse};
    use super::*;

    const PRESET: &str = "FuriHalSubGhzPresetOok650Async";

    /// 14 frame bytes for made-up fields: header nibble 0xA, the given button, serial (20 bits),
    /// flags 3, fingerprint, checksum = XOR of bytes 0..8, a filler byte, the counter, 2 more.
    fn frame(button: u8, serial: u32, fingerprint: u32, counter: u16) -> [u8; 14] {
        let mut d = [0u8; 14];
        d[0] = 0xA0 | button;
        d[1] = (serial >> 12) as u8;
        d[2] = (serial >> 4) as u8;
        d[3] = (((serial & 0xF) as u8) << 4) | 0x3;
        d[4..8].copy_from_slice(&fingerprint.to_be_bytes());
        d[8] = d[..8].iter().fold(0, |a, &b| a ^ b);
        d[9] = 0x5A;
        d[10..12].copy_from_slice(&counter.to_be_bytes());
        d[12] = 0x11;
        d[13] = 0x22;
        d
    }

    /// Half-bit samples: `cycles` preamble cycles (low, high), then the frame as Manchester pairs
    /// ((0,1) = 1, or inverted), then a closing gap.
    fn burst(d: &[u8; 14], cycles: usize, invert: bool, nbits: usize) -> Vec<Pulse> {
        let mut data_halves: Vec<bool> = Vec::new();
        for b in bits_of(d).into_iter().take(nbits) {
            let one = b != invert;
            data_halves.extend(if one { [false, true] } else { [true, false] });
        }
        // The preamble alternates and ends on the level the data starts with, so that the first
        // equal pair of samples (where the C finds the data) is the preamble / data junction.
        let len = 2 * cycles;
        let first = data_halves[0];
        let mut h: Vec<bool> = (0..len).map(|k| if (len - 1 - k).is_multiple_of(2) { first } else { !first }).collect();
        h.extend(data_halves);
        let mut p = halves_to_pulses(&h, 63);
        p.push((false, 9000));
        p
    }

    #[test]
    fn honda_v9_decodes_synthetic_frames_in_both_polarities() {
        let d = frame(0x5, 0x9_ABCD, 0x1234_5678, 0x0BEE);
        for (name, invert, cycles) in [("normal_55", false, 55), ("inverted_55", true, 55), ("normal_30", false, 30)] {
            let pulses = burst(&d, cycles, invert, 112);
            dump(&format!("honda_v9_{name}"), &pulses, PRESET);
            let got = decode(&mut HondaV9Decoder::new(), &pulses);
            assert_eq!(got.len(), 1, "{name}");
            let g = &got[0];
            assert_eq!((g.serial, g.button, g.counter), (Some(0x9_ABCD), Some(5), Some(0x0BEE)), "{name}");
            assert_eq!((g.data_count_bit, g.crc_valid), (112, true), "{name}");
            assert_eq!(g.data, u64::from_be_bytes(d[..8].try_into().unwrap()), "{name}");
            assert_eq!(g.extra, Some(d[8..14].iter().fold(0u64, |a, &b| (a << 8) | u64::from(b))), "{name}");
        }
    }

    #[test]
    fn honda_v9_checksum_tolerance_and_gating() {
        let d = frame(0x5, 0x9_ABCD, 0x1234_5678, 0x0BEE);
        // One checksum bit off: still emitted, flagged by crc_valid false.
        let mut one_off = d;
        one_off[8] ^= 0x04;
        let p = burst(&one_off, 55, false, 112);
        dump("honda_v9_checksum_1bit", &p, PRESET);
        let got = decode(&mut HondaV9Decoder::new(), &p);
        assert_eq!(got.len(), 1);
        assert!(!got[0].crc_valid);
        // Two bits off: dropped.
        let mut two_off = d;
        two_off[8] ^= 0x06;
        let p = burst(&two_off, 55, false, 112);
        dump("honda_v9_checksum_2bit", &p, PRESET);
        assert!(decode(&mut HondaV9Decoder::new(), &p).is_empty());
        // Short frame (100 bits), button 0xA, serial 0, fingerprint 0.
        let p = burst(&d, 55, false, 100);
        dump("honda_v9_short_frame", &p, PRESET);
        assert!(decode(&mut HondaV9Decoder::new(), &p).is_empty());
        for (name, f) in [
            ("button", frame(0xA, 0x9_ABCD, 0x1234_5678, 1)),
            ("serial", frame(0x5, 0, 0x1234_5678, 1)),
            ("fingerprint", frame(0x5, 0x9_ABCD, 0, 1)),
        ] {
            let p = burst(&f, 55, false, 112);
            dump(&format!("honda_v9_bad_{name}"), &p, PRESET);
            assert!(decode(&mut HondaV9Decoder::new(), &p).is_empty(), "{name}");
        }
        // Too short a preamble run (10 cycles = 20 samples + junction) still needs 20 alternating
        // samples; 5 cycles is rejected.
        let p = burst(&d, 5, false, 112);
        dump("honda_v9_short_preamble", &p, PRESET);
        assert!(decode(&mut HondaV9Decoder::new(), &p).is_empty());
    }

    #[test]
    fn honda_v9_key_file_round_trip() {
        use crate::protocols::flipper_key::testing::assert_round_trip;
        let f = frame(0x2, 0x1_2345, 0xDEAD_BEEF, 7);
        let data = u64::from_be_bytes(f[..8].try_into().unwrap());
        let tail = f[8..14].iter().fold(0u64, |a, &b| (a << 8) | u64::from(b));
        let text = assert_round_trip(&HondaV9Decoder::new(), &fields(data, tail));
        assert!(text.contains("Protocol: Honda V9\nBit: 112\n"), "{text}");
    }
}
