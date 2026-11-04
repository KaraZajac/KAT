//! Subaru protocol decoder/encoder.
//!
//! Port of ProtoPirate `protocols/subaru.c` (HEAD; primary reference), plus the frame reader of
//! Flipper-ARF `lib/subghz/protocols/subaru.c` as a second path so KAT also reads what ARF reads.
//! doctor-firmware `subaru.c` is ProtoPirate's file with its own D-pad codes.
//!
//! # Air format
//!
//! PWM with a constant ~2400µs bit period, 64 bits MSB first: preamble of ~76 HIGH/LOW pairs of
//! 1600µs, a 2800µs LOW gap, a 2800µs HIGH sync, then per bit a LOW + HIGH pair — `1` = LOW 1600 +
//! HIGH 800, `0` = LOW 800 + HIGH 1600 (the real Impreza fob in `IMPORTS/SUBARU` sends exactly
//! this, followed by 32-bit repeat frames that no reference decodes).
//!
//! * **ProtoPirate path** (`subaru_feed`): HIGH 1600±200 starts; ≥21 LOW 1600 preamble halves;
//!   LOW gap 2800±800; HIGH sync 2800±800; then each bit is a LOW (800 or 1600) followed by the
//!   complementary HIGH; the frame is reported on its 64th bit.
//! * **Flipper-ARF path** (its older reader): HIGH 1600±250 starts; >20 preamble halves; LOW gap
//!   and HIGH sync 2000..3500µs; a LOW 1600 after the sync; then the bit is the HIGH width alone
//!   (800 = 1, 1600 = 0) with any 800/1600 LOW between; reported on the first pulse >3000µs after
//!   ≥64 bits (extra bits ignored). It reads ARF's own encoder output (HIGH-first bits with a fixed
//!   800µs LOW), which the ProtoPirate path rejects; on a genuine fob frame the ProtoPirate path
//!   completes first and the ARF path is reset, so a frame is reported once.
//!
//! # Fields (`subaru_decode_fields_exact`)
//!
//! Key bytes `b0..b7`: button = `b0 & 0x0F`, serial = `b1 b2 b3`, 16-bit counter: the low byte is
//! stored inverted in `b4..b6` bits, the high byte is XOR-masked with the serial rotated left by
//! `4 + lo` (`REG_SH1/REG_SH2`). ProtoPirate's extra validity expression is ignored by the C (and is
//! false on every real frame), so frames are reported unconditionally (`crc_valid` = true).
//!
//! Button codes (native, shown via `native_button_name`): 1 Lock, 2 Unlock, 3 Trunk, 4 Panic —
//! ProtoPirate's emulate scene and Flipper-ARF's encoder/labels. (ProtoPirate's decoder label table
//! `names[btn & 3]`, which doctor's D-pad follows, disagrees; no capture settles it.)
//!
//! # Encoder (`subghz_protocol_encoder_subaru_deserialize` + `_get_upload`)
//!
//! Serial / button / counter + 1 are written back into the key (`b0` low nibble, `b1..b3`,
//! `subaru_encode_count`) and sent as ProtoPirate's upload: 75 × (LOW 1600, HIGH 1600), LOW 2800,
//! HIGH 2800, 64 data pairs (LOW, HIGH as above), LOW 2800, HIGH 2800 (282 entries; the Flipper
//! repeats it while held — PP `repeat` 10 — KAT returns it once). Flipper-ARF builds a different
//! HIGH-first upload that ProtoPirate cannot read; KAT transmits ProtoPirate's, which matches the
//! real fob's waveform. KAT generic buttons map Lock → 1, Unlock → 2, Trunk → 3, Panic → 4.
//!
//! **Deviation (counter encoding):** `subaru_encode_count` leaves the eight bits of `T1`/`T2` that
//! the decoder does not read at 1. In the real Impreza capture those bits carry the counter's low
//! byte inverted (`T1` bits 2,3,6,7 = `~lo` bits 0,1,4,5; `T2` bits 0,1,4,5 = `~lo` bits 2,3,6,7 —
//! the layout ProtoPirate's unused validity expression also describes). With that, re-encoding each
//! of the five captured frames reproduces it bit for bit and encoding its successor's counter gives
//! the next captured frame exactly (9/9); ProtoPirate's encoder gets 0/9. ARF's `encode_count` is
//! ProtoPirate's minus the `rel` bits and gets 0/9 too.

use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

const TE_SHORT: u32 = 800;
const TE_LONG: u32 = 1600;
const TE_DELTA: u32 = 200;
const MIN_COUNT_BIT: u8 = 64;

const PREAMBLE_PAIRS: usize = 75;
const GAP_US: u32 = 2800;
const SYNC_US: u32 = 2800;
const GAP_SYNC_DELTA: u32 = 800;
const PREAMBLE_MIN: u16 = 0x15;

/// Flipper-ARF reader constants (`subghz_protocol_subaru_const.te_delta` = 250 there).
const ARF_TE_DELTA: u32 = 250;
const ARF_GAP_MIN: u32 = 2000;
const ARF_GAP_MAX: u32 = 3500;
const ARF_END_US: u32 = 3000;

fn diff(a: u32, b: u32) -> u32 {
    a.abs_diff(b)
}

/// SubaruDecoderStep (ProtoPirate)
#[derive(Debug, Clone, Copy, PartialEq)]
enum PpStep {
    Reset,
    CheckPreamble,
    FoundGap,
    FoundSync,
    SaveDuration,
}

/// SubaruDecoderStep (Flipper-ARF)
#[derive(Debug, Clone, Copy, PartialEq)]
enum ArfStep {
    Reset,
    CheckPreamble,
    FoundGap,
    FoundSync,
    SaveDuration,
    CheckDuration,
}

/// Decoded key fields.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Fields {
    serial: u32,
    btn: u8,
    cnt: u16,
}

/// Subaru protocol decoder
pub struct SubaruDecoder {
    // ProtoPirate path (SubGhzBlockDecoder fields)
    pp_step: PpStep,
    pp_te_last: u32,
    pp_header_count: u16,
    pp_data: u64,
    pp_bits: u8,
    // Flipper-ARF path
    arf_step: ArfStep,
    arf_header_count: u16,
    arf_data: u64,
    arf_bits: u8,
}

impl SubaruDecoder {
    pub fn new() -> Self {
        Self {
            pp_step: PpStep::Reset,
            pp_te_last: 0,
            pp_header_count: 0,
            pp_data: 0,
            pp_bits: 0,
            arf_step: ArfStep::Reset,
            arf_header_count: 0,
            arf_data: 0,
            arf_bits: 0,
        }
    }

    /// subaru_rotate_left_3bytes
    fn rotate_left_3bytes(b0: &mut u8, b1: &mut u8, b2: &mut u8, count: u8) {
        for _ in 0..count {
            let t = *b0;
            *b0 = (*b0 << 1) | (*b1 >> 7);
            *b1 = (*b1 << 1) | (*b2 >> 7);
            *b2 = (*b2 << 1) | (t >> 7);
        }
    }

    /// subaru_decode_fields_exact (the validity result is unused by the C and dropped here).
    fn decode_fields(key: u64) -> Fields {
        let kb = key.to_be_bytes();
        let mut lo: u8 = 0;
        for (byte, mask, bit) in [
            (4, 0x40, 0x01),
            (4, 0x80, 0x02),
            (5, 0x01, 0x04),
            (5, 0x02, 0x08),
            (6, 0x01, 0x10),
            (6, 0x02, 0x20),
            (5, 0x40, 0x40),
            (5, 0x80, 0x80),
        ] {
            if kb[byte] & mask == 0 {
                lo |= bit;
            }
        }
        let reg_sh1 = ((kb[7] & 0x0F) << 4) | (kb[5] & 0x0C) | ((kb[6] >> 6) & 0x03);
        let reg_sh2 = ((kb[6] & 0x3C) << 2) | ((kb[7] >> 4) & 0x0F);
        let (mut ser0, mut ser1, mut ser2) = (kb[3], kb[1], kb[2]);
        // `(uint8_t)(4 + lo)` in the C: wraps for lo >= 0xFC.
        Self::rotate_left_3bytes(&mut ser0, &mut ser1, &mut ser2, 4u8.wrapping_add(lo));
        let t1 = ser1 ^ reg_sh1;
        let t2 = ser2 ^ reg_sh2;
        let mut hi: u8 = 0;
        for (t, mask, bit) in [
            (t1, 0x10, 0x04),
            (t1, 0x20, 0x08),
            (t2, 0x80, 0x02),
            (t2, 0x40, 0x01),
            (t1, 0x01, 0x40),
            (t1, 0x02, 0x80),
            (t2, 0x08, 0x20),
            (t2, 0x04, 0x10),
        ] {
            if t & mask == 0 {
                hi |= bit;
            }
        }
        Fields {
            serial: (kb[1] as u32) << 16 | (kb[2] as u32) << 8 | kb[3] as u32,
            btn: kb[0] & 0x0F,
            cnt: (hi as u16) << 8 | lo as u16,
        }
    }

    /// subaru_encode_count, with the capture-proven T1/T2 bits (module doc, "Deviation").
    fn encode_count(kb: &mut [u8; 8], count: u16) {
        let lo = count as u8;
        let hi = (count >> 8) as u8;

        kb[4] &= !0xC0;
        kb[5] &= !0xC3;
        kb[6] &= !0x03;
        for (byte, set, bit) in [
            (4, 0x40, 0x01),
            (4, 0x80, 0x02),
            (5, 0x01, 0x04),
            (5, 0x02, 0x08),
            (6, 0x01, 0x10),
            (6, 0x02, 0x20),
            (5, 0x40, 0x40),
            (5, 0x80, 0x80),
        ] {
            if lo & bit == 0 {
                kb[byte] |= set;
            }
        }

        let (mut ser0, mut ser1, mut ser2) = (kb[3], kb[1], kb[2]);
        // `(uint8_t)(4 + lo)` in the C: wraps for lo >= 0xFC.
        Self::rotate_left_3bytes(&mut ser0, &mut ser1, &mut ser2, 4u8.wrapping_add(lo));

        let rel = ser0 ^ kb[0];
        kb[4] = (kb[4] & 0xC0) | ((rel >> 2) & 0x3F);
        kb[5] = (kb[5] & 0xCF) | ((rel << 4) & 0x30);

        let mut t1: u8 = 0xFF;
        let mut t2: u8 = 0xFF;
        for (is_t1, clear, bit) in [
            (true, 0x10, 0x04),
            (true, 0x20, 0x08),
            (false, 0x80, 0x02),
            (false, 0x40, 0x01),
            (true, 0x01, 0x40),
            (true, 0x02, 0x80),
            (false, 0x08, 0x20),
            (false, 0x04, 0x10),
        ] {
            if hi & bit != 0 {
                if is_t1 {
                    t1 &= !clear;
                } else {
                    t2 &= !clear;
                }
            }
        }
        // KAT: the bits the decoder does not read carry ~lo (ProtoPirate leaves them at 1).
        let nlo = !lo;
        t1 = (t1 & 0x33) | ((nlo << 2) & 0xCC);
        t2 = (t2 & 0xCC) | ((nlo >> 2) & 0x33);

        let new_sh1 = t1 ^ ser1;
        let new_sh2 = t2 ^ ser2;

        kb[5] &= !0x0C;
        kb[6] &= !0xC0;
        kb[7] = (kb[7] & 0xF0) | ((new_sh1 >> 4) & 0x0F);
        if new_sh1 & 0x04 != 0 {
            kb[5] |= 0x04;
        }
        if new_sh1 & 0x08 != 0 {
            kb[5] |= 0x08;
        }
        if new_sh1 & 0x02 != 0 {
            kb[6] |= 0x80;
        }
        if new_sh1 & 0x01 != 0 {
            kb[6] |= 0x40;
        }
        kb[6] = (kb[6] & 0xC3) | ((new_sh2 >> 2) & 0x3C);
        kb[7] = (kb[7] & 0x0F) | ((new_sh2 << 4) & 0xF0);
    }

    /// A decoded 64-bit frame.
    fn frame(key: u64) -> DecodedSignal {
        let f = Self::decode_fields(key);
        DecodedSignal {
            serial: Some(f.serial),
            button: Some(f.btn),
            counter: Some(f.cnt),
            // No checksum: reported unconditionally, like both references.
            crc_valid: true,
            data: key,
            data_count_bit: MIN_COUNT_BIT as usize,
            encoder_capable: true,
            extra: None,
            protocol_display_name: None,
        }
    }

    fn reset_pp(&mut self) {
        self.pp_step = PpStep::Reset;
    }

    fn reset_arf(&mut self) {
        self.arf_step = ArfStep::Reset;
        self.arf_header_count = 0;
        self.arf_bits = 0;
        self.arf_data = 0;
    }

    /// subghz_protocol_decoder_subaru_feed (ProtoPirate).
    fn feed_pp(&mut self, level: bool, duration: u32) -> Option<u64> {
        match self.pp_step {
            PpStep::Reset => {
                if level && diff(duration, TE_LONG) < TE_DELTA {
                    self.pp_data = 0;
                    self.pp_bits = 0;
                    self.pp_header_count = 0;
                    self.pp_te_last = duration;
                    self.pp_step = PpStep::CheckPreamble;
                }
            }
            PpStep::CheckPreamble => {
                if level {
                    return None;
                }
                if diff(duration, TE_LONG) < TE_DELTA && diff(self.pp_te_last, TE_LONG) < TE_DELTA {
                    self.pp_header_count = self.pp_header_count.wrapping_add(1);
                    return None;
                }
                if self.pp_header_count >= PREAMBLE_MIN && diff(duration, GAP_US) < GAP_SYNC_DELTA {
                    self.pp_te_last = duration;
                    self.pp_step = PpStep::FoundGap;
                } else {
                    self.pp_step = PpStep::Reset;
                }
            }
            PpStep::FoundGap => {
                self.pp_step = if level
                    && diff(duration, SYNC_US) < GAP_SYNC_DELTA
                    && diff(self.pp_te_last, GAP_US) < GAP_SYNC_DELTA
                {
                    PpStep::FoundSync
                } else {
                    PpStep::Reset
                };
            }
            PpStep::FoundSync => {
                if !level
                    && (diff(duration, TE_LONG) < TE_DELTA || diff(duration, TE_SHORT) < TE_DELTA)
                {
                    self.pp_te_last = duration;
                    self.pp_step = PpStep::SaveDuration;
                } else {
                    self.pp_step = PpStep::Reset;
                }
            }
            PpStep::SaveDuration => {
                let mut next = PpStep::Reset;
                if level {
                    let bit = if diff(duration, TE_LONG) < TE_DELTA {
                        (diff(self.pp_te_last, TE_SHORT) < TE_DELTA).then_some(false)
                    } else if diff(duration, TE_SHORT) < TE_DELTA {
                        (diff(self.pp_te_last, TE_LONG) < TE_DELTA).then_some(true)
                    } else {
                        None
                    };
                    if let Some(bit) = bit {
                        self.pp_data = (self.pp_data << 1) | bit as u64;
                        self.pp_bits = self.pp_bits.wrapping_add(1);
                        next = PpStep::FoundSync;
                    }
                }
                self.pp_step = next;
                if (MIN_COUNT_BIT..=80).contains(&self.pp_bits) {
                    let key = self.pp_data;
                    self.pp_data = 0;
                    self.pp_bits = 0;
                    self.pp_step = PpStep::Reset;
                    return Some(key);
                }
            }
        }
        None
    }

    /// subaru_add_bit (Flipper-ARF): MSB-first into 8 bytes, capped at 64 bits.
    fn arf_add_bit(&mut self, bit: bool) {
        if self.arf_bits < 64 {
            let shift = 63 - self.arf_bits;
            if bit {
                self.arf_data |= 1 << shift;
            } else {
                self.arf_data &= !(1 << shift);
            }
            self.arf_bits += 1;
        }
    }

    /// The ARF reader's end-of-frame check (a pulse > 3000µs).
    fn arf_finish(&mut self) -> Option<u64> {
        let key = (self.arf_bits >= 64).then_some(self.arf_data);
        self.arf_step = ArfStep::Reset;
        key
    }

    /// subghz_protocol_decoder_subaru_feed (Flipper-ARF).
    fn feed_arf(&mut self, level: bool, duration: u32) -> Option<u64> {
        let near = |d: u32, t: u32| diff(d, t) < ARF_TE_DELTA;
        let gap = |d: u32| d > ARF_GAP_MIN && d < ARF_GAP_MAX;
        match self.arf_step {
            ArfStep::Reset => {
                if level && near(duration, TE_LONG) {
                    self.arf_step = ArfStep::CheckPreamble;
                    self.arf_header_count = 1;
                }
            }
            ArfStep::CheckPreamble => {
                if near(duration, TE_LONG) {
                    self.arf_header_count = self.arf_header_count.wrapping_add(1);
                } else if !level && gap(duration) && self.arf_header_count > 20 {
                    self.arf_step = ArfStep::FoundGap;
                } else {
                    self.arf_step = ArfStep::Reset;
                }
            }
            ArfStep::FoundGap => {
                self.arf_step = if level && gap(duration) {
                    ArfStep::FoundSync
                } else {
                    ArfStep::Reset
                };
            }
            ArfStep::FoundSync => {
                if !level && near(duration, TE_LONG) {
                    self.arf_step = ArfStep::SaveDuration;
                    self.arf_bits = 0;
                    self.arf_data = 0;
                } else {
                    self.arf_step = ArfStep::Reset;
                }
            }
            ArfStep::SaveDuration => {
                if !level {
                    self.arf_step = ArfStep::Reset;
                } else if near(duration, TE_SHORT) {
                    self.arf_add_bit(true);
                    self.arf_step = ArfStep::CheckDuration;
                } else if near(duration, TE_LONG) {
                    self.arf_add_bit(false);
                    self.arf_step = ArfStep::CheckDuration;
                } else if duration > ARF_END_US {
                    return self.arf_finish();
                } else {
                    self.arf_step = ArfStep::Reset;
                }
            }
            ArfStep::CheckDuration => {
                if level {
                    self.arf_step = ArfStep::Reset;
                } else if near(duration, TE_SHORT) || near(duration, TE_LONG) {
                    self.arf_step = ArfStep::SaveDuration;
                } else if duration > ARF_END_US {
                    return self.arf_finish();
                } else {
                    self.arf_step = ArfStep::Reset;
                }
            }
        }
        None
    }

    /// KAT generic button → Subaru code (ProtoPirate emulate / Flipper-ARF custom buttons).
    fn map_button(button: u8, captured: u8) -> u8 {
        match button {
            0x01 => 1,
            0x02 => 2,
            0x04 => 3,
            0x08 => 4,
            _ => captured,
        }
    }

    /// Key for (serial, button, counter) built on the captured key, as the C encoder does.
    fn build_key(captured_key: u64, serial: u32, btn: u8, cnt: u16) -> u64 {
        let mut kb = captured_key.to_be_bytes();
        kb[0] = (kb[0] & 0xF0) | (btn & 0x0F);
        kb[1] = (serial >> 16) as u8;
        kb[2] = (serial >> 8) as u8;
        kb[3] = serial as u8;
        Self::encode_count(&mut kb, cnt);
        u64::from_be_bytes(kb)
    }

    /// subghz_protocol_encoder_subaru_get_upload (282 entries).
    fn build_upload(key: u64) -> Vec<LevelDuration> {
        let mut up = Vec::with_capacity(2 * PREAMBLE_PAIRS + 4 + 128);
        for _ in 0..PREAMBLE_PAIRS {
            up.push(LevelDuration::new(false, TE_LONG));
            up.push(LevelDuration::new(true, TE_LONG));
        }
        up.push(LevelDuration::new(false, GAP_US));
        up.push(LevelDuration::new(true, SYNC_US));
        for i in (0..64).rev() {
            let one = (key >> i) & 1 == 1;
            up.push(LevelDuration::new(false, if one { TE_LONG } else { TE_SHORT }));
            up.push(LevelDuration::new(true, if one { TE_SHORT } else { TE_LONG }));
        }
        up.push(LevelDuration::new(false, GAP_US));
        up.push(LevelDuration::new(true, SYNC_US));
        up
    }
}

impl ProtocolDecoder for SubaruDecoder {
    fn name(&self) -> &'static str {
        "Subaru"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: MIN_COUNT_BIT as usize,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[433_920_000, 315_000_000] // 433.92 MHz (EU/AU) and 315 MHz (US/JP)
    }

    fn reset(&mut self) {
        self.reset_pp();
        self.reset_arf();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let pp = self.feed_pp(level, duration);
        let arf = self.feed_arf(level, duration);
        if let Some(key) = pp {
            // A genuine frame completes here first; drop the ARF reader's copy of it.
            self.reset_arf();
            return Some(Self::frame(key));
        }
        if let Some(key) = arf {
            self.reset_pp();
            return Some(Self::frame(key));
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != MIN_COUNT_BIT as usize {
            return None;
        }
        let key = decoded.data;
        let from_key = Self::decode_fields(key);
        let serial = decoded.serial.unwrap_or(from_key.serial) & 0xFF_FFFF;
        let btn = Self::map_button(button, decoded.button.unwrap_or(from_key.btn)) & 0x0F;
        // Next press: counter + 1 (ProtoPirate's emulate scene / ARF's rolling multiplier).
        let cnt = decoded.counter.unwrap_or(from_key.cnt).wrapping_add(1);
        Some(Self::build_upload(Self::build_key(key, serial, btn, cnt)))
    }
}

impl Default for SubaruDecoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::export::flipper::import_sub_raw;
    use std::path::Path;

    /// The five consecutive frames of `IMPORTS/SUBARU/Subaro Impreza RAW .sub` (counters
    /// 0xC90A..=0xC90E, serial 0xCB715E, button 1), as ProtoPirate's and ARF's C decoders read them.
    const IMPREZA: [(u64, u16); 5] = [
        (0xE1CB_715E_4FD1_C3F8, 0xC90A),
        (0xE1CB_715E_16DD_F787, 0xC90B),
        (0xE1CB_715E_E4C0_9F78, 0xC90C),
        (0xE1CB_715E_80F4_4BA6, 0xC90D),
        (0xE1CB_715E_49CC_E71A, 0xC90E),
    ];

    fn decode_all(pairs: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut d = SubaruDecoder::new();
        pairs
            .iter()
            .filter_map(|p| d.feed(p.level, p.duration_us))
            .collect()
    }

    #[test]
    fn decodes_impreza_capture_like_the_references() {
        let path = Path::new("IMPORTS/SUBARU/Subaro Impreza RAW .sub");
        if !path.exists() {
            eprintln!("Skip: {path:?} not found (run from crate root)");
            return;
        }
        let (_, raw) = import_sub_raw(path).unwrap();
        let pairs: Vec<LevelDuration> = raw
            .iter()
            .map(|p| LevelDuration::new(p.level, p.duration_us))
            .collect();
        let frames = decode_all(&pairs);
        let got: Vec<(u64, u16)> = frames.iter().map(|f| (f.data, f.counter.unwrap())).collect();
        assert_eq!(got, IMPREZA.to_vec());
        for f in &frames {
            assert_eq!((f.serial, f.button, f.data_count_bit), (Some(0xCB_715E), Some(1), 64));
        }
    }

    /// The capture-derived counter encoding reproduces every real frame from its own fields and
    /// each successor from its predecessor (module doc, "Deviation").
    #[test]
    fn encode_count_reproduces_real_frames() {
        for (i, &(key, cnt)) in IMPREZA.iter().enumerate() {
            assert_eq!(SubaruDecoder::decode_fields(key).cnt, cnt);
            assert_eq!(SubaruDecoder::build_key(key, 0xCB_715E, 1, cnt), key, "same counter");
            if let Some(&(next_key, next_cnt)) = IMPREZA.get(i + 1) {
                assert_eq!(SubaruDecoder::build_key(key, 0xCB_715E, 1, next_cnt), next_key);
            }
        }
    }

    /// Transmitting from a capture sends the next counter with the requested button, and the
    /// upload decodes back (ProtoPirate path) to exactly that frame.
    #[test]
    fn encode_decode_round_trip() {
        let mut x: u64 = 0x9E37_79B9_7F4A_7C15;
        for _ in 0..2000 {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            let key = x;
            let cap = SubaruDecoder::frame(key);
            for (cmd, want) in [(0x01u8, 1u8), (0x02, 2), (0x04, 3), (0x08, 4), (0x40, cap.button.unwrap())] {
                let up = SubaruDecoder::new().encode(&cap, cmd).unwrap();
                assert_eq!(up.len(), 282);
                let frames = decode_all(&up);
                assert_eq!(frames.len(), 1);
                let f = &frames[0];
                assert_eq!(f.serial, cap.serial);
                assert_eq!(f.button, Some(want));
                assert_eq!(f.counter, Some(cap.counter.unwrap().wrapping_add(1)));
                assert_eq!(f.data >> 60, key >> 60, "b0 high nibble kept");
            }
        }
    }

    /// Flipper-ARF's encoder output (HIGH-first bits, fixed 800µs LOW, 20 preamble pairs) is read by
    /// the ARF path; ProtoPirate's path alone cannot read it.
    #[test]
    fn reads_flipper_arf_encoder_output() {
        let key = 0xE1CB_715E_0FD1_F7A7u64;
        let mut up = Vec::new();
        for _ in 0..20 {
            up.push(LevelDuration::new(true, 1600));
            up.push(LevelDuration::new(false, 1600));
        }
        up.push(LevelDuration::new(true, 1600));
        up.push(LevelDuration::new(false, 2500));
        up.push(LevelDuration::new(true, 2500));
        up.push(LevelDuration::new(false, 1600));
        for i in (0..64).rev() {
            up.push(LevelDuration::new(true, if (key >> i) & 1 == 1 { 800 } else { 1600 }));
            up.push(LevelDuration::new(false, 800));
        }
        up.push(LevelDuration::new(true, 1600));
        up.push(LevelDuration::new(false, 5000));
        let frames = decode_all(&up);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].data, key);
        let mut pp = SubaruDecoder::new();
        assert!(up.iter().all(|p| pp.feed_pp(p.level, p.duration_us).is_none()));
    }
    #[test]
    fn upload_layout_matches_c() {
        let up = SubaruDecoder::build_upload(0x8000_0000_0000_0001);
        assert_eq!(up.len(), 282);
        assert!(!up[0].level && up[1].level && up[0].duration_us == 1600);
        assert_eq!((up[150].level, up[150].duration_us), (false, 2800));
        assert_eq!((up[151].level, up[151].duration_us), (true, 2800));
        // bit 63 = 1: LOW 1600 + HIGH 800; bit 62 = 0: LOW 800 + HIGH 1600
        assert_eq!((up[152].duration_us, up[153].duration_us), (1600, 800));
        assert_eq!((up[154].duration_us, up[155].duration_us), (800, 1600));
        assert_eq!((up[280].level, up[281].level), (false, true));
    }
}
