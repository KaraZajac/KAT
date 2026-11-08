//! Honda V6 protocol decoder (**decode only**, as the reference)
//!
//! Ported from FoxFW `lib/subghz/protocols/honda_v6.c` and `honda_v6.h` (itself a port of the
//! KeeLoq-RX `arke_honda.py` `HondaV6` class); no other reference carries it. The C has no encoder
//! (`.encoder = NULL`), so neither does KAT. **No path carries a check**: emission is gated on
//! structure only, so `crc_valid` is always false and the frames are plausibility reads, not
//! verified ones.
//!
//! One decoder, three paths, told apart by the pulses that open the burst:
//!
//! * **Variant 0, "stock45"** (FM, 434 MHz, Manchester te 100 / 200 µs, delta 45, inclusive): at
//!   least 20 short pulses (`header`), then a long one starts the Manchester machine
//!   (`{0x01, 0x91, 0x9B, 0xFB}`, with the toolbox's reset-to-start on a no-transition pulse).
//!   43..48 bits at the end of the burst give a 45-bit word `norm`: counter = `norm[44:41]`,
//!   `hop` = `norm[40:0]`, serial = `hop >> 9` (32 bits, must not be 0), button from `hop` bits
//!   34 / 29 / 31 (0x02 / 0x01 / 0x10, else 0x03). `data_count_bit` 45.
//! * **Variant 1, "pandora"** (118..140 bits, Pandora-TS skeleton): the burst may contain a
//!   1000..2000 µs mid gap after 45..65 bits, and ends 78 bits after it. A frame under 135 bits
//!   is de-drifted (shifted right by up to 16) to 135 and must carry the skeleton
//!   `lo[63:40] == 0x1E1E1F`, `lo[7:0] == 0x51`, `hi[7:0] == 0xFE`. Serial = `lo[63:32]`, counter
//!   = the top `bits - 119` (at most 16) bits of `hi`, button 0. `data_count_bit` 135..140; the
//!   high word is kept in `extra`.
//! * **Variant 2, "313"** (3-level trits, pulse widths 800 / 1200 / 1600 µs, +-250): at least 15
//!   short (800 µs) pulses that are not 100 µs shorts, then trits (0 / 1 / 2) packed 2 bits each,
//!   MSB first, into `d2` (trits 0..31) and `d1` (trits 32..63), at most 60. A pulse that is not a
//!   trit ends the burst; with at least 30 trits and a non-zero `d2[63:32]` the frame is emitted:
//!   serial = `d2[63:32]`, counter = the trit count (the C extracts no counter), button 0,
//!   `data_count_bit` = 2 x trits (60..120), the `d1` word in `extra`.
//!
//! The variant is not stored by KAT: the bit count tells it (45, 135..140, or even 60..120).
//!
//! **Evidence.** FoxFW's decoder reads 16 frames in 5 unrelated captures of the corpus (three
//! Toyota Estima files, an Opel Vectra and a Subaru recording), all through the 313 path with
//! keys such as `40 00 00 14 00 00 00 10`: noise, not Honda keyfobs. The module is therefore
//! **not registered**: with no check and a one-pulse-class preamble, registering it would
//! relabel those Toyota / Fiat / Renault decodes. See `docs/honda_v6.md`.
//!
//! Key files are not supported: the C's key file keeps only the low data word.

use super::info::prelude::*;
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
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

const NAME: &str = "Honda V6";
const TS: u32 = 100;
const TL: u32 = 200;
const TD: u32 = 45;
const MIN_BITS: u16 = 43;
const MIN_PRE: u32 = 20;
const PAN_MIN: u16 = 118;
const PAN_MAX: u16 = 140;
const PAN_HALFA_MIN: u16 = 45;
const PAN_HALFA_MAX: u16 = 65;
const PAN_HALFB: u16 = 78;
const PAN_MIDGAP_MIN: u32 = 1000;
const PAN_MIDGAP_MAX: u32 = 2000;

const SHORT_LOW: u8 = 0;
const SHORT_HIGH: u8 = 2;
const LONG_LOW: u8 = 4;
const LONG_HIGH: u8 = 6;
const RESET_EV: u8 = 8;

const T313_S: u32 = 800;
const T313_M: u32 = 1200;
const T313_L: u32 = 1600;
const T313_D: u32 = 250;
const T313_MINPRE: u32 = 15;
const T313_MINTRIT: u16 = 30;
const T313_MAXTRIT: u16 = 60;

const MAN_TABLE: [u8; 4] = [0x01, 0x91, 0x9B, 0xFB];

#[derive(Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    CheckPre,
    Collect,
    Pre313,
    Data313,
}

/// Honda V6 decoder (matches SubGhzProtocolDecoderHondaV6).
pub struct HondaV6Decoder {
    step: Step,
    header: u32,
    man_state: u8,
    lo: u64,
    hi: u64,
    bit_count: u16,
    midgap: bool,
    halfa: u16,
    trit_count: u16,
    d2: u64,
    d1: u64,
}

/// `hv6_man_advance`: Some(bit) when the machine yields one.
fn man_advance(state: &mut u8, event: u8) -> Option<u8> {
    if event == RESET_EV {
        *state = 1;
        return None;
    }
    let ns = (MAN_TABLE[usize::from(*state & 3)] >> event) & 0x3;
    if ns == *state {
        *state = 1;
        return None;
    }
    *state = ns;
    match ns {
        2 => Some(0),
        1 => Some(1),
        _ => None,
    }
}

/// `hv6_skeleton_ok`.
fn skeleton_ok(hi: u64, lo: u64) -> bool {
    ((lo >> 40) & 0xFF_FFFF) == 0x1E_1E1F && (lo & 0xFF) == 0x51 && (hi & 0xFF) == 0xFE
}

/// `hv6_undrift`: a short pandora frame shifted to 135 bits, when the skeleton then matches.
fn undrift(hi: u64, lo: u64, bit_count: u16) -> Option<(u64, u64)> {
    if !(PAN_MIN..135).contains(&bit_count) {
        return None;
    }
    let shift = u32::from(135 - bit_count);
    if shift > 16 {
        return None;
    }
    let new_lo = (lo >> shift) | (hi << (64 - shift));
    let new_hi = hi >> shift;
    skeleton_ok(new_hi, new_lo).then_some((new_hi, new_lo))
}

/// `hv6_identify_btn`.
fn identify_btn(hop: u64) -> u8 {
    if (hop >> 34) & 1 == 1 {
        0x02
    } else if (hop >> 29) & 1 == 1 {
        0x01
    } else if (hop >> 31) & 1 == 1 {
        0x10
    } else {
        0x03
    }
}

/// `hv6_pan_cnt_width`.
fn pan_cnt_width(bc: u16) -> u16 {
    if bc < 119 {
        0
    } else {
        (bc - 119).min(16)
    }
}

/// `hv6_pan_counter`.
fn pan_counter(hi: u64, bc: u16) -> u32 {
    let w = pan_cnt_width(bc);
    if w == 0 {
        return 0;
    }
    let n = i32::from(bc.min(128));
    let shift = n - 64 - i32::from(w);
    if shift < 0 {
        return 0;
    }
    ((hi >> shift) & ((1u64 << w) - 1)) as u32
}

/// `hv6_classify313`: trit 0 / 1 / 2, or 3 when the width is none of them.
fn classify313(duration: u32) -> u8 {
    if duration.abs_diff(T313_S) <= T313_D {
        0
    } else if duration.abs_diff(T313_M) <= T313_D {
        1
    } else if duration.abs_diff(T313_L) <= T313_D {
        2
    } else {
        3
    }
}

/// `hv6_emit`: the C's `generic.data` is `data_lo`; the high word goes to `extra`.
fn emit(serial: u32, btn: u8, cnt: u32, data_lo: u64, data_hi: u64, bits: u16) -> DecodedSignal {
    DecodedSignal {
        serial: Some(serial),
        button: Some(btn),
        counter: Some(cnt as u16),
        crc_valid: false,
        data: data_lo,
        data_count_bit: usize::from(bits),
        encoder_capable: false,
        extra: Some(data_hi),
        protocol_display_name: None,
    }
}

impl HondaV6Decoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            header: 0,
            man_state: 1,
            lo: 0,
            hi: 0,
            bit_count: 0,
            midgap: false,
            halfa: 0,
            trit_count: 0,
            d2: 0,
            d1: 0,
        }
    }

    /// `hv6_man_adv`: classify a short / long pulse and advance the machine.
    fn man_adv(&mut self, level: bool, duration: u32) -> Option<u8> {
        let ev = if duration.abs_diff(TS) <= TD {
            if level { SHORT_HIGH } else { SHORT_LOW }
        } else if duration.abs_diff(TL) <= TD {
            if level { LONG_HIGH } else { LONG_LOW }
        } else {
            return None;
        };
        man_advance(&mut self.man_state, ev)
    }

    /// `hv6_append`.
    fn append(&mut self, bit: u8) {
        if self.bit_count < 128 {
            self.hi = (self.hi << 1) | (self.lo >> 63);
            self.lo = (self.lo << 1) | u64::from(bit);
        }
        self.bit_count += 1;
    }

    /// `hv6_process_fsk`.
    fn process_fsk(&self) -> Option<DecodedSignal> {
        let mut bc = self.bit_count;
        let is_stock = (MIN_BITS..=48).contains(&bc);
        let is_pan = (PAN_MIN..=PAN_MAX).contains(&bc);
        if !is_stock && !is_pan {
            return None;
        }
        let (mut hi, mut lo) = (self.hi, self.lo);
        if is_pan {
            if bc < 135 {
                let (nhi, nlo) = undrift(hi, lo, bc)?;
                hi = nhi;
                lo = nlo;
                bc = 135;
            } else if !skeleton_ok(hi, lo) {
                return None;
            }
        }
        if is_stock {
            let norm = lo & 0x1FFF_FFFF_FFFF; // low 45 bits
            let cnt = ((norm >> 41) & 0xF) as u32;
            let hop = norm & 0x1FF_FFFF_FFFF; // low 41 bits
            let btn = identify_btn(hop);
            let serial = (hop >> 9) as u32;
            if serial == 0 {
                return None;
            }
            Some(emit(serial, btn, cnt, norm, 0, 45))
        } else {
            let serial = ((lo >> 32) & 0xFFFF_FFFF) as u32;
            let cnt = pan_counter(hi, bc);
            Some(emit(serial, 0, cnt, lo, hi, bc))
        }
    }

    /// `hv6_add_trit`.
    fn add_trit(&mut self, trit: u8) {
        if self.trit_count >= T313_MAXTRIT {
            return;
        }
        let idx = u32::from(self.trit_count);
        if idx < 32 {
            self.d2 |= u64::from(trit & 3) << (62 - idx * 2);
        } else if idx < 64 {
            self.d1 |= u64::from(trit & 3) << (62 - (idx - 32) * 2);
        }
        self.trit_count += 1;
    }

    /// `hv6_process313`.
    fn process313(&self) -> Option<DecodedSignal> {
        if self.trit_count < T313_MINTRIT {
            return None;
        }
        let serial = ((self.d2 >> 32) & 0xFFFF_FFFF) as u32;
        if serial == 0 {
            return None;
        }
        Some(emit(serial, 0, u32::from(self.trit_count), self.d2, self.d1, self.trit_count * 2))
    }

    fn man_reset(&mut self) {
        self.man_state = 1;
    }
}

impl Default for HondaV6Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for HondaV6Decoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: TS, te_long: TL, te_delta: TD, min_count_bit: 45 }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.step = Step::Reset;
        self.header = 0;
        self.man_state = 1;
        self.lo = 0;
        self.hi = 0;
        self.bit_count = 0;
        self.midgap = false;
        self.halfa = 0;
        self.trit_count = 0;
        self.d2 = 0;
        self.d1 = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let is_s = duration.abs_diff(TS) <= TD;
        let is_l = duration.abs_diff(TL) <= TD;
        let is_s313 = duration.abs_diff(T313_S) <= T313_D;
        let trit313 = classify313(duration);

        match self.step {
            Step::Reset => {
                if level && is_s {
                    self.step = Step::CheckPre;
                    self.header = 0;
                    self.lo = 0;
                    self.hi = 0;
                    self.bit_count = 0;
                    self.midgap = false;
                    self.man_reset();
                }
                // 313 MHz 3-level preamble: a short-313 pulse that is not a 434 short.
                if is_s313 && !is_s {
                    self.step = Step::Pre313;
                    self.header = 1;
                }
                None
            }
            Step::CheckPre => {
                if is_s {
                    self.header += 1;
                } else if is_l && self.header >= MIN_PRE {
                    self.step = Step::Collect;
                    self.lo = 0;
                    self.hi = 0;
                    self.bit_count = 0;
                    self.midgap = false;
                    self.halfa = 0;
                    self.man_reset();
                    let ev = if level { LONG_HIGH } else { LONG_LOW };
                    if let Some(bit) = man_advance(&mut self.man_state, ev) {
                        self.append(bit);
                    }
                } else if !is_s {
                    self.step = Step::Reset;
                }
                None
            }
            Step::Collect => {
                if is_s || is_l {
                    if let Some(bit) = self.man_adv(level, duration) {
                        self.append(bit);
                    }
                    if self.midgap && self.halfa > 0 && self.bit_count >= self.halfa + PAN_HALFB {
                        let out = self.process_fsk();
                        self.step = Step::Reset;
                        return out;
                    }
                    None
                } else {
                    let is_midgap = !level && (PAN_MIDGAP_MIN..=PAN_MIDGAP_MAX).contains(&duration);
                    if is_midgap
                        && !self.midgap
                        && (PAN_HALFA_MIN..=PAN_HALFA_MAX).contains(&self.bit_count)
                    {
                        self.midgap = true;
                        self.halfa = self.bit_count;
                        self.man_reset();
                        return None;
                    }
                    if !level {
                        if let Some(bit) = man_advance(&mut self.man_state, LONG_LOW) {
                            self.append(bit);
                        }
                    }
                    let out = self.process_fsk();
                    self.step = Step::Reset;
                    out
                }
            }
            Step::Pre313 => {
                if is_s313 {
                    self.header += 1;
                } else if self.header >= T313_MINPRE && trit313 <= 2 {
                    self.step = Step::Data313;
                    self.trit_count = 0;
                    self.d1 = 0;
                    self.d2 = 0;
                    self.add_trit(trit313);
                } else {
                    self.step = Step::Reset;
                }
                None
            }
            Step::Data313 => {
                if trit313 <= 2 {
                    self.add_trit(trit313);
                    None
                } else {
                    let out = self.process313();
                    self.step = Step::Reset;
                    out
                }
            }
        }
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
    use super::super::honda_synth::{decode, dump, halves_to_pulses, Pulse};
    use super::*;

    const PRESET: &str = "FuriHalSubGhzPresetOok650Async";

    /// 313 path: `shorts` 800 µs pulses (alternating levels), then trits, then a gap.
    fn trits_burst(shorts: usize, trits: &[u8]) -> Vec<Pulse> {
        let mut p: Vec<Pulse> = (0..shorts).map(|i| (i % 2 == 0, 800)).collect();
        for (i, &t) in trits.iter().enumerate() {
            p.push(((shorts + i).is_multiple_of(2), [800, 1200, 1600][usize::from(t)]));
        }
        p.push((false, 5000));
        p
    }

    /// `n` made-up trits, the first of which is not 0 (a 0 would be read as one more preamble pulse).
    fn trit_stream(n: usize) -> Vec<u8> {
        (0..n).map(|i| ((i * 5 + 2) % 3) as u8).collect()
    }

    #[test]
    fn honda_v6_313_trit_path() {
        let trits = trit_stream(40);
        let serial = trits[..16].iter().fold(0u32, |a, &t| (a << 2) | u32::from(t));
        let pulses = trits_burst(20, &trits);
        dump("honda_v6_trits", &pulses, PRESET);
        let got = decode(&mut HondaV6Decoder::new(), &pulses);
        assert_eq!(got.len(), 1);
        let d = &got[0];
        assert_eq!(d.serial, Some(serial));
        assert_eq!((d.data_count_bit, d.counter, d.button, d.crc_valid), (80, Some(40), Some(0), false));
        assert_eq!(d.data >> 32, u64::from(serial));
        assert!(!HondaV6Decoder::new().supports_encoding());
    }

    #[test]
    fn honda_v6_313_gating() {
        let trits = trit_stream(40);
        // Too few preamble pulses, too few trits, a zero serial.
        let few_pre = trits_burst(10, &trits);
        dump("honda_v6_short_preamble", &few_pre, PRESET);
        assert!(decode(&mut HondaV6Decoder::new(), &few_pre).is_empty());
        let few_trits = trits_burst(20, &trits[..25]);
        dump("honda_v6_short_frame", &few_trits, PRESET);
        assert!(decode(&mut HondaV6Decoder::new(), &few_trits).is_empty());
        let zero = trits_burst(20, &[0u8; 40]);
        dump("honda_v6_zero_serial", &zero, PRESET);
        assert!(decode(&mut HondaV6Decoder::new(), &zero).is_empty());
    }

    /// FSK burst: a high half, `pairs` low/high preamble pairs (all 100 µs shorts), then `bits` as
    /// Manchester, in V6's polarity (1 = low, high; 0 = high, low), which with this prefix makes
    /// the first (long) data pulse start the Manchester machine on a 0 bit.
    fn fsk_halves(pairs: usize, bits: &[bool]) -> Vec<bool> {
        let mut h = vec![true];
        for _ in 0..pairs {
            h.extend([false, true]);
        }
        h.extend(bits.iter().flat_map(|&b| if b { [false, true] } else { [true, false] }));
        h
    }

    fn msb_bits(v: u128, n: usize) -> Vec<bool> {
        (0..n).rev().map(|i| (v >> i) & 1 == 1).collect()
    }

    /// 434 MHz "stock45" path: 45 bits, counter nibble, 41-bit hop word (serial << 9 | filler).
    #[test]
    fn honda_v6_stock45_path() {
        let serial: u64 = 0x03A5_1C7F; // bit 25 set: hop bit 34, so button 0x02
        let word: u64 = (0x5u64 << 41) | (serial << 9) | 0x1A5;
        let h = fsk_halves(12, &msb_bits(u128::from(word), 45));
        let mut pulses = halves_to_pulses(&h, 100);
        pulses.push((false, 5000));
        dump("honda_v6_stock45", &pulses, PRESET);
        let got = decode(&mut HondaV6Decoder::new(), &pulses);
        assert_eq!(got.len(), 1);
        let d = &got[0];
        assert_eq!((d.serial, d.button, d.counter), (Some(0x03A5_1C7F), Some(0x02), Some(5)));
        assert_eq!((d.data, d.data_count_bit, d.extra, d.crc_valid), (word, 45, Some(0), false));
        // Too few preamble shorts: not read. A zero serial: not read.
        let mut few = halves_to_pulses(&fsk_halves(5, &msb_bits(u128::from(word), 45)), 100);
        few.push((false, 5000));
        dump("honda_v6_stock45_short_preamble", &few, PRESET);
        assert!(decode(&mut HondaV6Decoder::new(), &few).is_empty());
        let zero: u64 = (0x5u64 << 41) | 0x1A5;
        let mut z = halves_to_pulses(&fsk_halves(12, &msb_bits(u128::from(zero), 45)), 100);
        z.push((false, 5000));
        dump("honda_v6_stock45_zero_serial", &z, PRESET);
        assert!(decode(&mut HondaV6Decoder::new(), &z).is_empty());
    }

    /// "Pandora" path: 128 bits with a mid gap after 50, de-drifted by 7 to the skeleton
    /// `0x1E1E1F.. 0x51 / 0xFE`.
    #[test]
    fn honda_v6_pandora_path() {
        let nhi: u64 = 0x00A5_0000_0000_00FE; // counter 0x00A5 (top bits 0), skeleton byte 0xFE
        let nlo: u64 = 0x1E1E_1FAB_CDEF_1251; // serial 0x1E1E1FAB, skeleton 1E1E1F .. 51
        let target = (u128::from(nhi) << 64) | u128::from(nlo);
        let w = (target << 7) | 0x2B; // 7 drifted bits
        let bits = msb_bits(w, 128);
        let mut pulses = halves_to_pulses(&fsk_halves(12, &bits[..50]), 100);
        pulses.push((false, 1500));
        let mut second = vec![true];
        second.extend(bits[50..].iter().flat_map(|&b| if b { [false, true] } else { [true, false] }));
        pulses.extend(halves_to_pulses(&second, 100));
        pulses.push((false, 5000));
        dump("honda_v6_pandora", &pulses, PRESET);
        let got = decode(&mut HondaV6Decoder::new(), &pulses);
        assert_eq!(got.len(), 1);
        let d = &got[0];
        assert_eq!((d.serial, d.button, d.counter), (Some(0x1E1E_1FAB), Some(0), Some(0x00A5)));
        assert_eq!((d.data, d.data_count_bit, d.extra, d.crc_valid), (nlo, 135, Some(nhi), false));
        // A damaged skeleton byte is not read.
        let mut bad = bits.clone();
        for b in bad.iter_mut().skip(120).take(1) {
            *b = !*b;
        }
        let mut p = halves_to_pulses(&fsk_halves(12, &bad[..50]), 100);
        p.push((false, 1500));
        let mut second = vec![true];
        second.extend(bad[50..].iter().flat_map(|&b| if b { [false, true] } else { [true, false] }));
        p.extend(halves_to_pulses(&second, 100));
        p.push((false, 5000));
        dump("honda_v6_pandora_bad_skeleton", &p, PRESET);
        assert!(decode(&mut HondaV6Decoder::new(), &p).is_empty());
    }
}
