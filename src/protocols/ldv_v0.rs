//! LDV V0 decoder (FoxFW, decode-only).
//!
//! Ported from FoxFW `lib/subghz/protocols/ldv_v0.c` (`SUBGHZ_PROTOCOL_LDV_V0_NAME = "LDV V0"`; the
//! header credits the firmware `ldv_v0.c` via KeeLoq-RX `arke_misc.py` class `LdvV0`, "LDV / Maxus").
//! No other reference has it. AM, 315 / 433.92 MHz.
//!
//! **Relation to Ford V3** (`ford_v3.rs`): the LDV T80 fob captures in `IMPORTS/LDV` are the same
//! on-air frame that Ford V3's EU path reads (its 104-bit key there is the bit-inverse of this
//! decoder's 128-bit key, e.g. `44 43 A8 8F 90 …` vs `BB BC 57 70 6F …`), at 250/500 µs against Ford
//! V3's 240/480. Ford V3 is registered first and completes at bit 104, before this decoder reaches
//! bit 128, so on a capture whose pulses Ford V3 accepts, Ford V3 keeps the frame, exactly as in
//! FoxFW. What this decoder adds: it tolerates the timing Ford V3 rejects (short 131..=369 µs, long
//! 381..=619 µs against Ford V3's 180..=300 / 420..=540), it reads the whole 128-bit frame, and it
//! gates on the frame's fixed fields (below), where Ford V3 has no check at all. Its fields are the
//! LDV ones (serial = the first 32 bits, 2-bit button, 11-bit counter, 40-bit hop).
//!
//! **Line code** (`feed`): Manchester at te 250 / 500 µs, delta 120 (strict `<`), through Flipper's
//! `manchester_advance` with the event `level ? ShortLow : ShortHigh` (the "standard Flipper
//! mapping"). A HIGH short pulse starts a preamble (that first pulse is not fed to the Manchester
//! state, which starts at Mid1). The preamble is a run of at least 48 identical Manchester bits; the
//! first differing bit marks the start of the data and is discarded. A pulse longer than 2000 µs, or
//! one that is neither short nor long, drops back to Reset. 128 data bits are collected (the first
//! 64 in `data`, the next 64 in `data_2`).
//!
//! **Frame** (16 bytes): `serial32 | b4 | b5 | byte6 | 0xFF | 0xFF | byte9 | byte10 | hop40`, with
//! `byte6 = 0x80 | (btn << 3) | 0x07`. The gate: `byte6` bit 7 set, `byte6 & 7 == 7`, bytes 7 and 8
//! both 0xFF, and the button in `byte6` (bits 3..4) equals the button duplicated in the counter field
//! (`((byte9 << 8) | byte10) >> 3 & 3`). `cnt = ((byte9 << 8) | byte10) >> 5`. No CRC.
//!
//! **DecodedSignal**: `data` = bytes 0..7, `extra` = bytes 8..15 (the C's `Data2Hi/Lo`),
//! `data_count_bit` = 128, `serial`, `button` (0..3), `counter` (11 bits). The 40-bit hop is the low
//! 40 bits of `extra`.

use super::common::{ManchesterEvent, ManchesterState};
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "LDV V0",
    decoder: "LDV V0",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "LDV",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&AM650],
};

const TE_SHORT: u32 = 250;
const TE_LONG: u32 = 500;
const TE_DELTA: u32 = 120;
const PREAMBLE_MIN: u16 = 48;
const DATA_BITS: usize = 128;

#[derive(Clone, Copy, PartialEq)]
enum Step {
    Reset,
    Preamble,
    Data,
}

pub struct LdvV0Decoder {
    step: Step,
    manchester: ManchesterState,
    preamble_count: u16,
    last_bit_value: u8,
    data: u64,
    data_2: u64,
    bit_count: usize,
}

impl LdvV0Decoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            manchester: ManchesterState::Mid1,
            preamble_count: 0,
            last_bit_value: 0,
            data: 0,
            data_2: 0,
            bit_count: 0,
        }
    }

    /// `ldv_v0_valid`: the structural fixed-field gate.
    fn valid(&self) -> bool {
        let byte6 = ((self.data >> 8) & 0xFF) as u8;
        let byte7 = (self.data & 0xFF) as u8;
        let byte8 = ((self.data_2 >> 56) & 0xFF) as u8;
        let byte9 = ((self.data_2 >> 48) & 0xFF) as u8;
        let byte10 = ((self.data_2 >> 40) & 0xFF) as u8;
        if byte6 & 0x80 == 0 || byte6 & 0x07 != 0x07 || byte7 != 0xFF || byte8 != 0xFF {
            return false;
        }
        let byte6_btn = (byte6 >> 3) & 0x03;
        let cnt_field = (u16::from(byte9) << 8) | u16::from(byte10);
        byte6_btn == ((cnt_field >> 3) & 0x03) as u8
    }

    /// `ldv_v0_commit` (+ `ldv_v0_extract`).
    fn commit(&self) -> Option<DecodedSignal> {
        if self.bit_count < DATA_BITS || !self.valid() {
            return None;
        }
        let byte6 = ((self.data >> 8) & 0xFF) as u8;
        let cnt_field = ((self.data_2 >> 40) & 0xFFFF) as u16;
        Some(DecodedSignal {
            serial: Some((self.data >> 32) as u32),
            button: Some((byte6 >> 3) & 0x03),
            counter: Some(cnt_field >> 5),
            crc_valid: false,
            data: self.data,
            data_count_bit: DATA_BITS,
            encoder_capable: false,
            extra: Some(self.data_2),
            protocol_display_name: None,
        })
    }

    fn clear_frame(&mut self) {
        self.data = 0;
        self.data_2 = 0;
        self.bit_count = 0;
    }
}

impl Default for LdvV0Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for LdvV0Decoder {
    fn name(&self) -> &'static str {
        "LDV V0"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: DATA_BITS,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.step = Step::Reset;
        self.manchester = ManchesterState::Mid1;
        self.preamble_count = 0;
        self.last_bit_value = 0;
        self.clear_frame();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let short = duration.abs_diff(TE_SHORT) < TE_DELTA;
        let long = duration.abs_diff(TE_LONG) < TE_DELTA;
        // Standard Flipper mapping: `level ? ShortLow : ShortHigh`.
        let event = if short || long {
            ManchesterEvent::from_half_bit(!level, long)
        } else {
            ManchesterEvent::Reset
        };

        match self.step {
            Step::Reset => {
                if level && short {
                    self.preamble_count = 0;
                    self.clear_frame();
                    self.step = Step::Preamble;
                    self.manchester = ManchesterState::Mid1;
                }
            }
            Step::Preamble => {
                if duration > TE_LONG * 4 || event == ManchesterEvent::Reset {
                    self.step = Step::Reset;
                    return None;
                }
                if let Some(bit) = self.manchester.advance(event) {
                    let bval = u8::from(bit);
                    if bval == self.last_bit_value && self.preamble_count > 0 {
                        self.preamble_count = self.preamble_count.wrapping_add(1);
                    } else if self.preamble_count == 0 {
                        self.last_bit_value = bval;
                        self.preamble_count = 1;
                    } else if self.preamble_count >= PREAMBLE_MIN {
                        self.clear_frame();
                        self.step = Step::Data;
                    } else {
                        self.last_bit_value = bval;
                        self.preamble_count = 1;
                    }
                }
            }
            Step::Data => {
                if duration > TE_LONG * 4 {
                    let r = self.commit();
                    self.step = Step::Reset;
                    return r;
                }
                if event == ManchesterEvent::Reset {
                    self.step = Step::Reset;
                    return None;
                }
                if let Some(bit) = self.manchester.advance(event) {
                    let b = u64::from(bit);
                    if self.bit_count < 64 {
                        self.data = (self.data << 1) | b;
                    } else {
                        self.data_2 = (self.data_2 << 1) | b;
                    }
                    self.bit_count += 1;
                    if self.bit_count == DATA_BITS {
                        let r = self.commit();
                        self.clear_frame();
                        self.step = Step::Reset;
                        return r;
                    }
                }
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
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::dd_testutil::{bits_of, dump_sub, flipper_manchester, Pulse};

    const PRESET: &str = "FuriHalSubGhzPresetOok650Async";

    /// Made-up 16-byte frame: serial, b4, b5, byte6 (button `btn`), 0xFF 0xFF, byte9/10 (counter
    /// `cnt`, button copy `btn_copy`), 40-bit hop.
    fn frame(serial: u32, btn: u8, btn_copy: u8, cnt: u16, hop: u64, sep: u8) -> Vec<u8> {
        let byte6 = 0x80 | (btn << 3) | 0x07;
        let cnt_field = (cnt << 5) | (u16::from(btn_copy) << 3);
        let mut bytes = serial.to_be_bytes().to_vec();
        bytes.extend([0x5A, 0xC3, byte6, sep, 0xFF]);
        bytes.extend(cnt_field.to_be_bytes());
        bytes.extend(&hop.to_be_bytes()[3..]);
        assert_eq!(bytes.len(), 16);
        bytes
    }

    /// A leading HIGH pulse (which only starts the preamble), then the Manchester bit stream of
    /// the C's mapping: `preamble_bits` ones, the flip (a zero), the 128 data bits, a 5 ms gap.
    fn wire(bytes: &[u8], short: u32, preamble_bits: usize) -> Vec<Pulse> {
        let mut bits: Vec<u8> = vec![1; preamble_bits];
        bits.push(0); // the flip that marks the data start
        for b in bytes {
            bits.extend(bits_of(u64::from(*b), 8));
        }
        let mut p: Vec<Pulse> = vec![(true, short)];
        flipper_manchester(&mut p, ManchesterState::Mid1, &bits, short, short * 2);
        let end = !p.last().unwrap().0;
        p.push((end, 5000));
        p
    }

    fn run(p: &[Pulse]) -> Vec<DecodedSignal> {
        let mut d = LdvV0Decoder::new();
        p.iter().filter_map(|&(l, us)| d.feed(l, us)).collect()
    }

    #[test]
    fn decodes_a_made_up_frame() {
        let bytes = frame(0x0102_0304, 2, 2, 0x155, 0x12_3456_789A, 0xFF);
        let p = wire(&bytes, 250, 60);
        dump_sub("ldv_v0_ok", 433_920_000, PRESET, &p);
        let r = run(&p);
        assert_eq!(r.len(), 1, "{r:?}");
        let s = &r[0];
        assert_eq!(s.serial, Some(0x0102_0304));
        assert_eq!(s.button, Some(2));
        assert_eq!(s.counter, Some(0x155));
        assert_eq!(s.data, u64::from_be_bytes(bytes[..8].try_into().unwrap()));
        assert_eq!(s.extra, Some(u64::from_be_bytes(bytes[8..].try_into().unwrap())));
        assert_eq!(s.data_count_bit, 128);
    }

    #[test]
    fn reads_timing_ford_v3_rejects() {
        // 300/600 µs: inside this decoder's 250 +- 120 / 500 +- 120 windows, outside Ford V3's
        // 240 +- 60 / 480 +- 60; 330/660 is outside the long window (381..=619) here too.
        let bytes = frame(0x0102_0304, 1, 1, 0x7FF, 0x00_0000_0001, 0xFF);
        let p = wire(&bytes, 330, 60);
        dump_sub("ldv_v0_330", 433_920_000, PRESET, &p);
        assert_eq!(run(&p).len(), 0);
        let p = wire(&bytes, 300, 60);
        dump_sub("ldv_v0_300", 433_920_000, PRESET, &p);
        let r = run(&p);
        assert_eq!(r.len(), 1);
        assert_eq!(r[0].counter, Some(0x7FF));
    }

    #[test]
    fn the_registry_leaves_nominal_timing_to_ford_v3() {
        let names = crate::protocols::dd_testutil::registry_names;
        let bytes = frame(0x0102_0304, 1, 1, 0x7FF, 1, 0xFF);
        // 250/500 µs: Ford V3 completes at bit 104, before this decoder reaches bit 128.
        let got = names(&wire(&bytes, 250, 60), 433_920_000);
        assert!(!got.is_empty() && got.iter().all(|n| n == "Ford V3"), "{got:?}");
        // 300/600 µs is outside Ford V3's windows, inside this decoder's.
        assert_eq!(names(&wire(&bytes, 300, 60), 433_920_000), ["LDV V0"]);
    }

    #[test]
    fn rejects_bad_fixed_fields_short_preamble_and_short_frames() {
        let bad_sep = frame(0x0102_0304, 2, 2, 0x155, 1, 0xFE);
        let p = wire(&bad_sep, 250, 60);
        dump_sub("ldv_v0_badsep", 433_920_000, PRESET, &p);
        assert!(run(&p).is_empty(), "separator byte 0xFE");
        let bad_btn = frame(0x0102_0304, 2, 1, 0x155, 1, 0xFF);
        let p = wire(&bad_btn, 250, 60);
        dump_sub("ldv_v0_badbtn", 433_920_000, PRESET, &p);
        assert!(run(&p).is_empty(), "button copy differs");
        let ok = frame(0x0102_0304, 2, 2, 0x155, 1, 0xFF);
        let p = wire(&ok, 250, 40);
        dump_sub("ldv_v0_short_preamble", 433_920_000, PRESET, &p);
        assert!(run(&p).is_empty(), "40-bit preamble");
        let p = wire(&ok[..15], 250, 60);
        dump_sub("ldv_v0_120bits", 433_920_000, PRESET, &p);
        assert!(run(&p).is_empty(), "120-bit frame");
    }
}
