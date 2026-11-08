//! Honda V10 ("V2H4") protocol decoder (**decode only**, as the reference)
//!
//! Ported from FoxFW `lib/subghz/protocols/honda_v10.c` and `honda_v10.h` (itself a port of the
//! KeeLoq-RX `arke_honda.py` `HondaV10` class); no other reference carries it. The C has no
//! encoder (`.encoder = NULL`), so neither does KAT. **There is no data check**: emission is
//! gated on structure (flags, button, serial), so `crc_valid` is always false.
//!
//! **Air format** (2-FSK, 315 / 433.92 MHz): Manchester with te 104 / 208 µs, tolerance 35 µs
//! (inclusive). Like Honda V9, every pulse is stored as half-bit samples (short = 1, long = 2,
//! at most 2048), and a pulse of neither width ends the burst: with at least 36 samples the burst
//! is scanned and the buffer cleared.
//!
//! **Decoder** (`try_decode`): a sliding scan over start offsets, both polarities (normal first).
//! At each offset `man_decode` looks for a run of at least 20 alternating samples that is
//! followed by something, skips the equal pair that ends it and reads Manchester pairs
//! (`(0, 1)` = 1 in normal polarity) until two equal samples or 96 bits. At least 88 bits are
//! needed; a frame shorter than 96 bits is zero-padded. After a frame the scan resumes where it
//! stopped, so **one burst could hold several frames**; but every frame fires the receiver's
//! callback, and a receiver resets all its decoders on a callback (the Flipper's does, and so does
//! the oracle's and KAT's registry), which empties the sample buffer under the running scan: only
//! the first frame of a burst is ever reported, and `feed` returns it and stops the scan.
//!
//! **Frame** (12 bytes, MSB first): `serial = d[0..3]` (24 bits), `button = d[3] >> 4`,
//! `flags = d[3] & 0xF`, `counter = 0x00FFFFFF ^ d[4..7]` (24 bits, stored inverted),
//! `status = d[7]`, `tail = d[8..12]` (a per-counter encrypted MAC). Emission needs `flags == 7`,
//! a button of 0xF or 0xE, and a serial that is neither 0 nor 0xFFFFFF.
//!
//! **KAT fields**: `data` = `d[0..8]` big-endian (the C `generic.data`), `data_count_bit` 96,
//! `serial` (24 bits), `button`, `counter` = the low 16 bits of the 24-bit counter (the full
//! value is `0xFFFFFF ^ data[31:8]`), `extra` = the tail, `crc_valid` false. Key files: Protocol
//! "Honda V10", Bit 96, Key, Serial, Btn, Cnt (24 bits), Flags, Status, Tail.
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
    presets: &[&FM476, &AM650],
};

const NAME: &str = "Honda V10";
const TS: u32 = 104;
const TL: u32 = 208;
const TD: u32 = 35;
const MIN_PRE: usize = 20;
const FRAME_BITS: usize = 96;
const MIN_BITS: usize = 88;
const FRAME_BYTES: usize = 12;
const HB_CAP: usize = 2048;

/// Honda V10 decoder (matches SubGhzProtocolDecoderHondaV10).
pub struct HondaV10Decoder {
    /// Half-bit samples, one entry per sample (0 or 1).
    hb: Vec<u8>,
}

/// `honda_v10_classify`: 1 short, 2 long, 0 neither.
fn classify(dur: u32) -> u8 {
    if dur.abs_diff(TS) <= TD {
        1
    } else if dur.abs_diff(TL) <= TD {
        2
    } else {
        0
    }
}

/// `honda_v10_man_decode`: (bit count, 0 on failure; the decoded bytes; the sample consumed).
fn man_decode(hb: &[u8], start_ofs: usize, invert: bool) -> (usize, [u8; FRAME_BYTES + 1], usize) {
    let mut decoded = [0u8; FRAME_BYTES + 1];
    let n = hb.len();
    if start_ofs + 1 >= n {
        return (0, decoded, start_ofs);
    }
    let mut alt_start = start_ofs;
    while alt_start + 1 < n {
        let mut alt_end = alt_start + 1;
        while alt_end < n && hb[alt_end] != hb[alt_end - 1] {
            alt_end += 1;
        }
        let run_len = alt_end - alt_start;
        if run_len >= MIN_PRE && alt_end < n {
            let mut i = alt_end;
            while i + 1 < n && hb[i] == hb[i + 1] {
                i += 1;
            }
            let mut bc = 0;
            decoded = [0u8; FRAME_BYTES + 1];
            while i + 1 < n && bc < FRAME_BITS {
                let (h0, h1) = (hb[i], hb[i + 1]);
                if h0 == h1 {
                    break;
                }
                let bv = if !invert { h0 == 0 && h1 == 1 } else { h0 == 1 && h1 == 0 };
                if bv {
                    decoded[bc >> 3] |= 1 << (7 - (bc & 7));
                }
                bc += 1;
                i += 2;
            }
            return (if bc >= MIN_BITS { bc } else { 0 }, decoded, i);
        }
        alt_start = alt_end;
    }
    (0, decoded, n)
}

/// Fields from `data` = `d[0..8]` and the 32-bit `tail`.
fn fields(data: u64, tail: u32) -> DecodedSignal {
    let d = data.to_be_bytes();
    let counter = 0x00FF_FFFF ^ ((u32::from(d[4]) << 16) | (u32::from(d[5]) << 8) | u32::from(d[6]));
    DecodedSignal {
        serial: Some((u32::from(d[0]) << 16) | (u32::from(d[1]) << 8) | u32::from(d[2])),
        button: Some(d[3] >> 4),
        counter: Some(counter as u16),
        crc_valid: false,
        data,
        data_count_bit: FRAME_BITS,
        encoder_capable: false,
        extra: Some(u64::from(tail)),
        protocol_display_name: None,
    }
}

impl HondaV10Decoder {
    pub fn new() -> Self {
        Self { hb: Vec::with_capacity(HB_CAP) }
    }

    /// `honda_v10_try_polarity_at`: the frame at `start_ofs`, if one passes the gate. (The C also
    /// returns where the scan resumes; KAT stops at the first frame, so it is not needed.)
    fn try_polarity_at(&self, invert: bool, start_ofs: usize) -> Option<DecodedSignal> {
        let (bit_count, d, _consumed) = man_decode(&self.hb, start_ofs, invert);
        if bit_count == 0 {
            return None;
        }
        let serial = (u32::from(d[0]) << 16) | (u32::from(d[1]) << 8) | u32::from(d[2]);
        let button = d[3] >> 4;
        let flags = d[3] & 0x0F;
        if (button != 0xF && button != 0xE) || serial == 0 || serial == 0x00FF_FFFF || flags != 0x7 {
            return None;
        }
        let data = u64::from_be_bytes(d[..8].try_into().expect("8 bytes"));
        let tail = u32::from_be_bytes([d[8], d[9], d[10], d[11]]);
        Some(fields(data, tail))
    }

    /// `honda_v10_try_decode`: the sliding scan, up to the first frame (see the module doc).
    fn try_decode(&self) -> Option<DecodedSignal> {
        let n = self.hb.len();
        if n < MIN_PRE + 16 {
            return None;
        }
        let mut ofs = 0;
        while ofs + MIN_PRE + 16 <= n {
            if let Some(found) = self.try_polarity_at(false, ofs).or_else(|| self.try_polarity_at(true, ofs)) {
                return Some(found);
            }
            let mut skip = ofs + 1;
            while skip < n && self.hb[skip] != self.hb[skip - 1] {
                skip += 1;
            }
            ofs = if skip > ofs { skip } else { ofs + 1 };
        }
        None
    }
}

impl Default for HondaV10Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for HondaV10Decoder {
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
            }
            2 => {
                if self.hb.len() + 2 <= HB_CAP {
                    self.hb.push(lvl);
                    self.hb.push(lvl);
                }
            }
            _ => {
                let out = if self.hb.len() >= MIN_PRE + 16 { self.try_decode() } else { None };
                self.hb.clear();
                return out;
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }

    /// Rebuild a decode from a key file written by [Self::to_flipper_key]; the C's gate (flags 7,
    /// button 0xE / 0xF) is re-applied.
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != FRAME_BITS {
            return None;
        }
        let data = key.key_u64()?;
        let d = fields(data, key.u32_field("Tail")?);
        let b3 = data.to_be_bytes()[3];
        ((b3 >> 4) >= 0xE && b3 & 0xF == 7).then_some(d)
    }

    /// `subghz_protocol_decoder_honda_v10_serialize`.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != FRAME_BITS {
            return None;
        }
        let d = decoded.data.to_be_bytes();
        let counter = 0x00FF_FFFF ^ ((u32::from(d[4]) << 16) | (u32::from(d[5]) << 8) | u32::from(d[6]));
        let mut key = FlipperKey::for_protocol(NAME, FRAME_BITS, decoded.data);
        key.push_u32("Serial", decoded.serial?);
        key.push_u32("Btn", decoded.button?.into());
        key.push_u32("Cnt", counter);
        key.push_u32("Flags", u32::from(d[3] & 0xF));
        key.push_u32("Status", u32::from(d[7]));
        key.push_u32("Tail", decoded.extra? as u32);
        Some(key)
    }
}

#[cfg(test)]
mod tests {
    use super::super::honda_synth::{bits_of, decode, dump, halves_to_pulses, Pulse};
    use super::*;

    const PRESET: &str = "FuriHalSubGhzPresetFSK476Async";

    /// 12 frame bytes: serial, button / flags, the counter stored inverted, status, tail.
    fn frame(serial: u32, button: u8, flags: u8, counter: u32, status: u8, tail: u32) -> [u8; 12] {
        let mut d = [0u8; 12];
        d[0] = (serial >> 16) as u8;
        d[1] = (serial >> 8) as u8;
        d[2] = serial as u8;
        d[3] = (button << 4) | flags;
        let inv = 0x00FF_FFFF ^ counter;
        d[4] = (inv >> 16) as u8;
        d[5] = (inv >> 8) as u8;
        d[6] = inv as u8;
        d[7] = status;
        d[8..12].copy_from_slice(&tail.to_be_bytes());
        d
    }

    /// Frames' Manchester halves after a preamble that ends on the level the data starts with.
    fn frame_halves(d: &[u8; 12], cycles: usize, invert: bool, nbits: usize) -> Vec<bool> {
        let mut data_halves: Vec<bool> = Vec::new();
        for b in bits_of(d).into_iter().take(nbits) {
            let one = b != invert;
            data_halves.extend(if one { [false, true] } else { [true, false] });
        }
        let len = 2 * cycles;
        let first = data_halves[0];
        let mut h: Vec<bool> = (0..len).map(|k| if (len - 1 - k).is_multiple_of(2) { first } else { !first }).collect();
        h.extend(data_halves);
        h
    }

    fn burst(frames: &[Vec<bool>]) -> Vec<Pulse> {
        let mut h: Vec<bool> = Vec::new();
        for f in frames {
            h.extend(f);
        }
        let mut p = halves_to_pulses(&h, 104);
        p.push((false, 9000));
        p
    }

    #[test]
    fn honda_v10_decodes_synthetic_frames() {
        let d = frame(0x00A1_B2C3, 0xF, 0x7, 0x0001_2345, 0x42, 0x0BAD_F00D);
        for (name, invert) in [("normal", false), ("inverted", true)] {
            let pulses = burst(&[frame_halves(&d, 30, invert, 96)]);
            dump(&format!("honda_v10_{name}"), &pulses, PRESET);
            let got = decode(&mut HondaV10Decoder::new(), &pulses);
            assert_eq!(got.len(), 1, "{name}");
            let g = &got[0];
            assert_eq!((g.serial, g.button, g.counter), (Some(0x00A1_B2C3), Some(0xF), Some(0x2345)), "{name}");
            assert_eq!((g.data_count_bit, g.crc_valid, g.extra), (96, false, Some(0x0BAD_F00D)), "{name}");
            assert_eq!(g.data, u64::from_be_bytes(d[..8].try_into().unwrap()), "{name}");
        }
        // Button 0xE is accepted too.
        let e = frame(0x00A1_B2C3, 0xE, 0x7, 0x00FF_FFFF, 0x42, 1);
        let pulses = burst(&[frame_halves(&e, 30, false, 96)]);
        let got = decode(&mut HondaV10Decoder::new(), &pulses);
        assert_eq!((got.len(), got[0].button, got[0].counter), (1, Some(0xE), Some(0xFFFF)));
    }

    /// A 90-bit frame is accepted, zero-padded to 96; two frames in one burst are both found.
    #[test]
    fn honda_v10_short_frames_and_two_frames_per_burst() {
        let d = frame(0x00A1_B2C3, 0xF, 0x7, 0x0001_2345, 0x42, 0x0BAD_F00D);
        let pulses = burst(&[frame_halves(&d, 30, false, 90)]);
        dump("honda_v10_90bit", &pulses, PRESET);
        let got = decode(&mut HondaV10Decoder::new(), &pulses);
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].extra, Some(u64::from(u32::from_be_bytes([d[8], d[9], d[10], d[11] & 0xC0]))));
        // Two frames in one burst: only the first is reported (the oracle's receiver resets its
        // decoders on the first callback).
        let e = frame(0x00A1_B2C4, 0xE, 0x7, 0x0001_2346, 0x43, 0x0BAD_F00E);
        let pulses = burst(&[frame_halves(&d, 30, false, 96), frame_halves(&e, 30, false, 96)]);
        dump("honda_v10_two_frames", &pulses, PRESET);
        let got = decode(&mut HondaV10Decoder::new(), &pulses);
        assert_eq!(got.iter().map(|g| g.serial.unwrap()).collect::<Vec<_>>(), [0x00A1_B2C3]);
    }

    #[test]
    fn honda_v10_gating() {
        let good = frame(0x00A1_B2C3, 0xF, 0x7, 0x0001_2345, 0x42, 0x0BAD_F00D);
        let cases = [
            ("flags", frame(0x00A1_B2C3, 0xF, 0x6, 1, 0, 0), 96, 30),
            ("button", frame(0x00A1_B2C3, 0xD, 0x7, 1, 0, 0), 96, 30),
            ("serial_zero", frame(0, 0xF, 0x7, 1, 0, 0), 96, 30),
            ("serial_ff", frame(0x00FF_FFFF, 0xF, 0x7, 1, 0, 0), 96, 30),
            ("short_frame", good, 80, 30),
            ("short_preamble", good, 96, 4),
        ];
        for (name, d, nbits, cycles) in cases {
            let pulses = burst(&[frame_halves(&d, cycles, false, nbits)]);
            dump(&format!("honda_v10_bad_{name}"), &pulses, PRESET);
            assert!(decode(&mut HondaV10Decoder::new(), &pulses).is_empty(), "{name}");
        }
    }

    #[test]
    fn honda_v10_key_file_round_trip() {
        use crate::protocols::flipper_key::testing::assert_round_trip;
        let d = frame(0x00A1_B2C3, 0xE, 0x7, 0x00AB_CDEF, 0x42, 0x0BAD_F00D);
        let g = fields(u64::from_be_bytes(d[..8].try_into().unwrap()), 0x0BAD_F00D);
        let text = assert_round_trip(&HondaV10Decoder::new(), &g);
        assert!(text.contains("Protocol: Honda V10\nBit: 96\n"), "{text}");
        assert!(text.contains(&format!("Cnt: {}\n", 0x00AB_CDEFu32)), "{text}");
    }
}
