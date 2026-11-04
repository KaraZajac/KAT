//! Mercedes V1 protocol decoder/encoder
//!
//! Aligned with Flipper-ARF reference: `REFERENCES/Flipper-ARF/lib/subghz/protocols/mercedes_v1.c`
//! (`SUBGHZ_PROTOCOL_MERCEDES_V1_NAME = "Mercedes V1"`). A near-identical copy lives in
//! `REFERENCES/doctor-firmware/lib/subghz/protocols/mercedes_v1.c`; the decoder, field layout,
//! payload rebuild and burst builder are identical. They differ only in the custom-button glue
//! (ARF: inline UP=Lock(0)/DOWN=Unlock(1), default = original bit; doctor: the same codes from the
//! `custom_btn.c` map). This port follows the **ARF** copy (self-contained; its "default = keep the
//! original button" path matches KAT's "unknown button → keep the captured button" rule).
//!
//! # Wire format (FM, 433 MHz, Manchester, TE = 50µs)
//! - Chips: 25–79µs = 1T, 80–140µs = 2T; any pulse ≥280µs is a gap (reset); anything else resets.
//! - Preamble: alternating 1T pulses (the encoder sends 561 "1" bits = HIGH 50 / LOW 50 pairs);
//!   the decoder needs ≥32 pairs (counted on each 1T LOW).
//! - A 2T pulse after the preamble starts the data: its first chip closes the preamble, its second
//!   chip opens the first data pair. Each chip pair is a logical bit: HIGH→LOW = 1, LOW→HIGH = 0.
//!   On air the encoder sends `air_bit = !stored_bit`; stored bit 0 is always 1, so the first air
//!   bit (LOW→HIGH) is what merges with the last preamble LOW into the 2T sync LOW.
//! - 122 payload bits, MSB first: bits 0..63 → `hi`, bits 64..121 → `lo` bits 63..6.
//!
//! # Normalisation (matches `mercedes_v1_to_storage`)
//! If received bit 0 is 0 the frame is inverted (`hi = !hi; lo ^= 0xFFFF_FFFF_FFFF_FFC0`, the 58
//! valid `lo` bits), so both on-air polarities yield the same stored frame (the C's second
//! "invert and retry" attempt is equivalent to the first and changes nothing).
//!
//! # Frame / fields (stored bit numbering, `mercedes_v1_parse_fields`)
//! - bit 0: always 1 · bit 1: button (0 = Lock, 1 = Unlock) · bit 2: counter state
//! - bit 3: check = `!(bit1 ^ bit2)` · bit 13: repeat flag (`Rpt = bit13 ? 0 : 1`)
//! - bits 16..47: serial (32-bit) · counter (10-bit): bit 48 → 0x200, bit 49 → 0x100,
//!   bits 57..64 → counter bits 0..7 (LSB first)
//! - bits 67..120: 54-bit hop code (`HOP_MASK`) · bit 121: always 1
//!
//! `DecodedSignal` packing: `data` = stored `hi` (first 64 bits), `data_count_bit` = 122,
//! `extra` = stored `lo >> 6` (the remaining 58 bits, right-aligned). `counter` = the 10-bit counter.
//!
//! # Gating
//! The C accepts a 122-bit frame when (after normalisation) bit 0 = 1, bit 3 = `!(bit1 ^ bit2)` and
//! bit 121 = 1 — only 2 effective bits on top of the structure (≥32-pair 50µs preamble, 2T sync,
//! 122 gap-free Manchester pairs). A rejected frame whose trailing run of identical bits is ≥32 is
//! treated as more preamble (`mercedes_v1_resync_rejected_frame`). KAT additionally rejects (and
//! resyncs) any frame whose leading or trailing run of identical bits is ≥32 (`PREAMBLE_MIN`):
//! such a frame is preamble decoded as data (e.g. a jitter-stretched 2T pulse mid-preamble makes
//! the C accept an all-ones frame, which passes its field check). A genuine frame cannot start
//! with a 32-bit constant run unless its fields are degenerate, and ends with one only if the last
//! 31 hop bits are constant.
//!
//! **Exact length (KAT).** A validated frame is held until the next pulse and emitted only if that
//! pulse is a gap (≥280µs); if more chips follow, the frame is longer than 122 bits and is dropped.
//! BMW (`bmw.rs`) uses the same template at 62µs chips — inside this decoder's 25–79 / 80–140µs
//! windows — and its 128-bit frame would otherwise be cut at bit 122 and accepted by this weak
//! field check (≈1 in 2 BMW frames), before BMW itself completes. Genuine Mercedes V1 bursts end on
//! a HIGH chip followed by the LOW repeat gap, so they are unaffected.
//!
//! # Encoder (matches `mercedes_v1_get_upload` / `_emit_burst`, repeat = `MERCEDES_V1_REPEAT_DEFAULT`)
//! The hop code is opaque (no key), so TX replays the stored 122 bits. KAT Lock (0x01) / Unlock
//! (0x02) re-stamp the button bit (and its check bit 3) via `mercedes_v1_rebuild_payload`, exactly
//! like the C d-pad; any other code keeps the captured button. No counter bump (the C has none).
//! Upload = 561 preamble "1" bits + 122 air bits (`!stored`), then a 50ms LOW gap; the C's
//! default repeat count (4) is emitted back to back.
//!
//! # Live capture note
//! KAT's live demodulator debounces pulses shorter than 40µs, which leaves only ~10µs of margin
//! under the 50µs chip; live decoding needs a clean, strong signal. Raw `.sub` imports are not
//! debounced and are unaffected.

use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

const TE_US: u32 = 50;
const TE1_MIN_US: u32 = 25;
const TE1_MAX_US: u32 = 79;
const TE2_MIN_US: u32 = 80;
const TE2_MAX_US: u32 = 140;
const GAP_US: u32 = 280;
const PREAMBLE_MIN: u16 = 32;
const PAYLOAD_BITS: u8 = 122;
const CNT_MASK: u32 = 0x3FF;
const HOP_MASK: u64 = 0x3F_FFFF_FFFF_FFFF;
const PREAMBLE_TX_BITS: usize = 561;
const REPEAT_DEFAULT: usize = 4;
const REPEAT_GAP_US: u32 = 50_000;
const LO_INVERT_MASK: u64 = 0xFFFF_FFFF_FFFF_FFC0;

// KAT generic button codes.
const BTN_LOCK: u8 = 0x01;
const BTN_UNLOCK: u8 = 0x02;

// Protocol button bit values.
const MB_LOCK: u8 = 0;
const MB_UNLOCK: u8 = 1;

/// Decoder steps (matches `MercedesV1DecoderStep`).
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    /// Expecting a 1T LOW (preamble) or a 2T LOW (sync).
    PreambleLow,
    /// Expecting a 1T HIGH (preamble) or a 2T HIGH (sync).
    PreambleHigh,
    Data,
    /// A validated 122-bit frame is pending; it is emitted only if the next pulse is a gap.
    AwaitGap,
}

/// Parsed fields of a stored frame (`mercedes_v1_parse_fields`).
#[derive(Debug, Clone, Copy, PartialEq)]
struct Fields {
    serial: u32,
    btn: u8,
    /// The C's `generic.extra` ("Rpt"): `bit13 ? 0 : 1`.
    repeat_flag: u8,
    cnt: u32,
    hop: u64,
}

fn get_bit(hi: u64, lo: u64, i: u8) -> bool {
    if i < 64 {
        (hi >> (63 - i)) & 1 != 0
    } else {
        (lo >> (63 - (i - 64))) & 1 != 0
    }
}

fn set_bit(hi: &mut u64, lo: &mut u64, i: u8, value: bool) {
    let (word, shift) = if i < 64 {
        (hi, 63 - i)
    } else {
        (lo, 63 - (i - 64))
    };
    let mask = 1u64 << shift;
    if value {
        *word |= mask;
    } else {
        *word &= !mask;
    }
}

/// `mercedes_v1_bits`: `len` (≤16) bits starting at `start`, MSB first.
fn bits(hi: u64, lo: u64, start: u8, len: u8) -> u16 {
    let mut value = 0u16;
    for i in 0..len {
        value = (value << 1) | get_bit(hi, lo, start + i) as u16;
    }
    value
}

fn invert(hi: &mut u64, lo: &mut u64) {
    *hi = !*hi;
    *lo ^= LO_INVERT_MASK;
}

fn get_counter(hi: u64, lo: u64) -> u32 {
    let mut cnt = if get_bit(hi, lo, 48) { 0x200 } else { 0 };
    if get_bit(hi, lo, 49) {
        cnt |= 0x100;
    }
    for k in 0..8u8 {
        if get_bit(hi, lo, 57 + k) {
            cnt |= 1 << k;
        }
    }
    cnt & CNT_MASK
}

fn set_counter(hi: &mut u64, lo: &mut u64, cnt: u32) {
    let cnt = cnt & CNT_MASK;
    set_bit(hi, lo, 48, cnt & 0x200 != 0);
    set_bit(hi, lo, 49, cnt & 0x100 != 0);
    for k in 0..8u8 {
        set_bit(hi, lo, 57 + k, (cnt >> k) & 1 != 0);
    }
}

/// `mercedes_v1_fields_valid`.
fn fields_valid(hi: u64, lo: u64) -> bool {
    let btn = get_bit(hi, lo, 1);
    let counter_state = get_bit(hi, lo, 2);
    // C: bit 3 must equal !(bit1 ^ bit2), i.e. bit1 == bit2 XNOR.
    get_bit(hi, lo, 0) && get_bit(hi, lo, 3) == (btn == counter_state) && get_bit(hi, lo, 121)
}

/// `mercedes_v1_to_storage`: normalise polarity on bit 0, then validate.
fn to_storage(hi: &mut u64, lo: &mut u64) -> bool {
    if !get_bit(*hi, *lo, 0) {
        invert(hi, lo);
    }
    fields_valid(*hi, *lo)
}

fn parse_fields(hi: u64, lo: u64) -> Fields {
    let serial = ((bits(hi, lo, 16, 16) as u32) << 16) | bits(hi, lo, 32, 16) as u32;
    let mut hop = 0u64;
    for i in 67..=120u8 {
        hop = (hop << 1) | get_bit(hi, lo, i) as u64;
    }
    Fields {
        serial,
        btn: get_bit(hi, lo, 1) as u8,
        repeat_flag: if get_bit(hi, lo, 13) { 0 } else { 1 },
        cnt: get_counter(hi, lo),
        hop: hop & HOP_MASK,
    }
}

/// `mercedes_v1_rebuild_payload`.
fn rebuild_payload(hi: &mut u64, lo: &mut u64, btn: u8, repeat_flag: u8, cnt: u32) {
    let type_bit = btn & 1 != 0;
    let state_bit = get_bit(*hi, *lo, 2);
    set_bit(hi, lo, 0, true);
    set_bit(hi, lo, 1, type_bit);
    set_bit(hi, lo, 3, !(type_bit ^ state_bit));
    set_bit(hi, lo, 13, repeat_flag & 1 == 0);
    set_counter(hi, lo, cnt);
    set_bit(hi, lo, 121, true);
}

/// Length of the run of identical bits at the start (`leading`) or end of the 122-bit frame.
fn constant_run(hi: u64, lo: u64, leading: bool) -> u16 {
    let idx = |n: u8| if leading { n } else { PAYLOAD_BITS - 1 - n };
    let first = get_bit(hi, lo, idx(0));
    let mut run = 0u16;
    for n in 0..PAYLOAD_BITS {
        if get_bit(hi, lo, idx(n)) != first {
            break;
        }
        run += 1;
    }
    run
}

/// Mercedes V1 decoder (matches `SubGhzProtocolDecoderMercedesV1`).
pub struct MercedesV1Decoder {
    step: DecoderStep,
    hi: u64,
    lo: u64,
    preamble_count: u16,
    /// First chip of the current Manchester pair (C: `int8_t pair_chip`, -1 = none).
    pair_chip: Option<bool>,
    nbits: u8,
    /// Frame validated at bit 122, waiting for the end-of-frame gap (see "Exact length").
    pending: Option<DecodedSignal>,
}

impl MercedesV1Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            hi: 0,
            lo: 0,
            preamble_count: 0,
            pair_chip: None,
            nbits: 0,
            pending: None,
        }
    }

    /// `mercedes_v1_reset_soft`.
    fn reset_soft(&mut self) {
        self.step = DecoderStep::Reset;
        self.hi = 0;
        self.lo = 0;
        self.preamble_count = 0;
        self.pair_chip = None;
        self.nbits = 0;
        self.pending = None;
    }

    fn chips(duration: u32) -> u8 {
        if (TE1_MIN_US..=TE1_MAX_US).contains(&duration) {
            1
        } else if (TE2_MIN_US..=TE2_MAX_US).contains(&duration) {
            2
        } else {
            0
        }
    }

    /// `mercedes_v1_resync_rejected_frame`: a trailing constant run ≥32 is more preamble.
    fn resync_rejected_frame(&mut self) {
        let last = get_bit(self.hi, self.lo, PAYLOAD_BITS - 1);
        let run = constant_run(self.hi, self.lo, false);
        self.reset_soft();
        if run >= PREAMBLE_MIN {
            self.preamble_count = run;
            self.step = if last {
                DecoderStep::PreambleHigh
            } else {
                DecoderStep::PreambleLow
            };
        }
    }

    /// `mercedes_v1_try_callback` (+ the KAT constant-run gate).
    fn try_callback(&mut self) -> Option<DecodedSignal> {
        let (mut hi, mut lo) = (self.hi, self.lo);
        let mut valid = to_storage(&mut hi, &mut lo);
        if !valid {
            // C retry: invert the raw frame and normalise again (equivalent to the first try).
            hi = self.hi;
            lo = self.lo;
            invert(&mut hi, &mut lo);
            valid = to_storage(&mut hi, &mut lo);
        }
        let preamble_like = constant_run(self.hi, self.lo, true) >= PREAMBLE_MIN
            || constant_run(self.hi, self.lo, false) >= PREAMBLE_MIN;
        if !valid || preamble_like {
            self.resync_rejected_frame();
            return None;
        }

        let f = parse_fields(hi, lo);
        self.reset_soft();
        Some(DecodedSignal {
            serial: Some(f.serial),
            button: Some(f.btn),
            counter: Some(f.cnt as u16),
            crc_valid: true,
            data: hi,
            data_count_bit: PAYLOAD_BITS as usize,
            encoder_capable: true,
            extra: Some(lo >> 6),
            protocol_display_name: None,
        })
    }

    /// `mercedes_v1_add_logical_bit`.
    fn add_logical_bit(&mut self, bit: bool) -> Option<DecodedSignal> {
        set_bit(&mut self.hi, &mut self.lo, self.nbits, bit);
        self.nbits += 1;
        if self.nbits == PAYLOAD_BITS {
            return self.try_callback();
        }
        None
    }

    /// `mercedes_v1_add_chip`: HIGH→LOW = 1, LOW→HIGH = 0, equal chips reset.
    fn add_chip(&mut self, chip: bool) -> Option<DecodedSignal> {
        match self.pair_chip.take() {
            None => {
                self.pair_chip = Some(chip);
                None
            }
            Some(first) if first != chip => self.add_logical_bit(first),
            Some(_) => {
                self.reset_soft();
                None
            }
        }
    }

    /// `mercedes_v1_enter_data`.
    fn enter_data(&mut self, level: bool) {
        self.step = DecoderStep::Data;
        self.hi = 0;
        self.lo = 0;
        self.nbits = 0;
        self.pair_chip = Some(level);
    }

    /// Map a KAT generic button to the protocol button bit (ARF `mercedes_v1_get_btn_code`).
    fn map_button(button: u8, original: u8) -> u8 {
        match button {
            BTN_LOCK => MB_LOCK,
            BTN_UNLOCK => MB_UNLOCK,
            _ => original & 1,
        }
    }

    /// `mercedes_v1_emit_merge`.
    fn enc_add_level(signal: &mut Vec<LevelDuration>, level: bool, duration: u32) {
        if let Some(last) = signal.last_mut() {
            if last.level == level {
                *last = LevelDuration::new(level, last.duration_us + duration);
                return;
            }
        }
        signal.push(LevelDuration::new(level, duration));
    }

    /// `mercedes_v1_emit_bit`: 1 → HIGH, LOW; 0 → LOW, HIGH.
    fn enc_bit(signal: &mut Vec<LevelDuration>, bit: bool) {
        Self::enc_add_level(signal, bit, TE_US);
        Self::enc_add_level(signal, !bit, TE_US);
    }

    /// `mercedes_v1_emit_burst`: 561 preamble "1" bits + 122 air bits (`!stored`).
    fn enc_burst(signal: &mut Vec<LevelDuration>, hi: u64, lo: u64) {
        for _ in 0..PREAMBLE_TX_BITS {
            Self::enc_bit(signal, true);
        }
        for i in 0..PAYLOAD_BITS {
            Self::enc_bit(signal, !get_bit(hi, lo, i));
        }
    }
}

impl ProtocolDecoder for MercedesV1Decoder {
    fn name(&self) -> &'static str {
        "Mercedes V1"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_US,
            te_long: TE_US * 2,
            // Approximate: classification uses the explicit 25–79 / 80–140µs chip windows.
            te_delta: 30,
            min_count_bit: PAYLOAD_BITS as usize,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        // C flags: SubGhzProtocolFlag_433 | SubGhzProtocolFlag_FM.
        &[433_920_000]
    }

    fn reset(&mut self) {
        self.reset_soft();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        if self.step == DecoderStep::AwaitGap {
            if duration >= GAP_US {
                // Exactly 122 bits then the end-of-frame gap: emit.
                let decoded = self.pending.take();
                self.reset_soft();
                return decoded;
            }
            // More chips follow: the frame is longer than 122 bits (e.g. BMW) — drop it and treat
            // this pulse as fresh input.
            self.reset_soft();
        }
        if duration >= GAP_US {
            self.reset_soft();
            return None;
        }
        let n = Self::chips(duration);
        if n == 0 {
            if self.step != DecoderStep::Reset {
                self.reset_soft();
            }
            return None;
        }

        match self.step {
            DecoderStep::Reset => {
                if n == 1 {
                    self.preamble_count = 0;
                    self.step = if level {
                        DecoderStep::PreambleLow
                    } else {
                        DecoderStep::PreambleHigh
                    };
                }
            }
            DecoderStep::PreambleLow => {
                if !level && n == 1 {
                    self.preamble_count = self.preamble_count.saturating_add(1);
                    self.step = DecoderStep::PreambleHigh;
                } else if !level && n == 2 && self.preamble_count >= PREAMBLE_MIN {
                    self.enter_data(false);
                } else {
                    self.reset_soft();
                }
            }
            DecoderStep::PreambleHigh => {
                if level && n == 1 {
                    self.step = DecoderStep::PreambleLow;
                } else if level && n == 2 && self.preamble_count >= PREAMBLE_MIN {
                    self.enter_data(true);
                } else {
                    self.reset_soft();
                }
            }
            DecoderStep::Data => {
                for i in 0..n {
                    if self.step != DecoderStep::Data {
                        break;
                    }
                    if let Some(decoded) = self.add_chip(level) {
                        if i + 1 < n {
                            // The pulse carries another chip after bit 122: frame is too long.
                            self.reset_soft();
                        } else {
                            self.pending = Some(decoded);
                            self.step = DecoderStep::AwaitGap;
                        }
                        break;
                    }
                }
            }
            DecoderStep::AwaitGap => unreachable!("handled at the top of feed()"),
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != PAYLOAD_BITS as usize {
            return None;
        }
        let mut hi = decoded.data;
        let mut lo = (decoded.extra? << 6) & LO_INVERT_MASK;

        let f = parse_fields(hi, lo);
        let btn = Self::map_button(button, f.btn);
        if btn != f.btn {
            rebuild_payload(&mut hi, &mut lo, btn, f.repeat_flag, f.cnt);
        }

        let burst_len = 2 * (PREAMBLE_TX_BITS + PAYLOAD_BITS as usize) + 1;
        let mut signal = Vec::with_capacity(burst_len * REPEAT_DEFAULT);
        for _ in 0..REPEAT_DEFAULT {
            Self::enc_burst(&mut signal, hi, lo);
            Self::enc_add_level(&mut signal, false, REPEAT_GAP_US);
        }
        Some(signal)
    }
}

impl Default for MercedesV1Decoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::ProtocolRegistry;

    const SERIAL: u32 = 0x1A2B_3C4D;
    const COUNTER: u32 = 0x2D7;
    const HOP: u64 = 0x2B_5E9C_14F3_A6D1; // 54 bits

    /// Build a valid stored frame from fields (filler bits use a fixed pattern).
    fn make_frame(
        btn: u8,
        state: bool,
        repeat_flag: u8,
        serial: u32,
        cnt: u32,
        hop: u64,
    ) -> (u64, u64) {
        let (mut hi, mut lo) = (0u64, 0u64);
        // Filler for the bits with no known meaning (4..12, 14, 15, 50..56, 65, 66).
        for i in [4u8, 6, 7, 10, 12, 15, 50, 53, 54, 56, 66] {
            set_bit(&mut hi, &mut lo, i, true);
        }
        set_bit(&mut hi, &mut lo, 2, state);
        for i in 0..32u8 {
            set_bit(&mut hi, &mut lo, 16 + i, (serial >> (31 - i)) & 1 != 0);
        }
        for i in 0..54u8 {
            set_bit(&mut hi, &mut lo, 67 + i, (hop >> (53 - i)) & 1 != 0);
        }
        rebuild_payload(&mut hi, &mut lo, btn, repeat_flag, cnt);
        (hi, lo)
    }

    fn signal_for(hi: u64, lo: u64) -> DecodedSignal {
        let f = parse_fields(hi, lo);
        DecodedSignal {
            serial: Some(f.serial),
            button: Some(f.btn),
            counter: Some(f.cnt as u16),
            crc_valid: true,
            data: hi,
            data_count_bit: PAYLOAD_BITS as usize,
            encoder_capable: true,
            extra: Some(lo >> 6),
            protocol_display_name: None,
        }
    }

    fn decode_all(pairs: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = MercedesV1Decoder::new();
        pairs
            .iter()
            .filter_map(|p| dec.feed(p.level, p.duration_us))
            .collect()
    }

    /// Independent wire builder: `preamble_pairs` HIGH/LOW pairs, then the air bits
    /// (`!stored`, HIGH→LOW = 1), then a 1ms gap.
    fn build_wire(hi: u64, lo: u64, preamble_pairs: usize) -> Vec<LevelDuration> {
        let mut halves: Vec<bool> = Vec::new();
        for _ in 0..preamble_pairs {
            halves.extend([true, false]);
        }
        for i in 0..PAYLOAD_BITS {
            let air = !get_bit(hi, lo, i);
            halves.extend([air, !air]);
        }
        let mut out = Vec::new();
        for h in halves {
            MercedesV1Decoder::enc_add_level(&mut out, h, TE_US);
        }
        MercedesV1Decoder::enc_add_level(&mut out, false, 1000);
        out
    }

    #[test]
    fn field_layout_roundtrip() {
        let (hi, lo) = make_frame(MB_UNLOCK, true, 1, SERIAL, COUNTER, HOP);
        assert!(fields_valid(hi, lo));
        let f = parse_fields(hi, lo);
        assert_eq!(
            f,
            Fields {
                serial: SERIAL,
                btn: MB_UNLOCK,
                repeat_flag: 1,
                cnt: COUNTER,
                hop: HOP
            }
        );
        assert_eq!(lo & !LO_INVERT_MASK, 0, "only 58 bits of lo are used");
    }

    #[test]
    fn synthetic_frame_decodes() {
        let (hi, lo) = make_frame(MB_LOCK, false, 0, SERIAL, COUNTER, HOP);
        let decoded = decode_all(&build_wire(hi, lo, 40));
        assert_eq!(decoded.len(), 1);
        let d = &decoded[0];
        assert_eq!((d.data, d.extra), (hi, Some(lo >> 6)));
        assert_eq!(d.serial, Some(SERIAL));
        assert_eq!(d.button, Some(MB_LOCK));
        assert_eq!(d.counter, Some(COUNTER as u16));
        assert_eq!(d.data_count_bit, 122);
        assert!(d.crc_valid && d.encoder_capable);
    }

    #[test]
    fn inverted_polarity_decodes_same_frame() {
        let (hi, lo) = make_frame(MB_UNLOCK, true, 1, SERIAL, COUNTER, HOP);
        let wire: Vec<LevelDuration> = build_wire(hi, lo, 40)
            .iter()
            .map(|p| LevelDuration::new(!p.level, p.duration_us))
            .collect();
        let decoded = decode_all(&wire);
        assert_eq!(decoded.len(), 1);
        assert_eq!((decoded[0].data, decoded[0].extra), (hi, Some(lo >> 6)));
    }

    #[test]
    fn encode_decode_roundtrip_keeps_captured_button() {
        let (hi, lo) = make_frame(MB_LOCK, true, 0, SERIAL, COUNTER, HOP);
        let enc = MercedesV1Decoder::new();
        let wire = enc.encode(&signal_for(hi, lo), 0x08).expect("encode");
        let decoded = decode_all(&wire);
        assert_eq!(
            decoded.len(),
            REPEAT_DEFAULT,
            "one decode per repeated burst"
        );
        for d in &decoded {
            assert_eq!((d.data, d.extra), (hi, Some(lo >> 6)));
            assert_eq!(
                d.counter,
                Some(COUNTER as u16),
                "C does not bump the counter"
            );
        }
    }

    #[test]
    fn encode_restamps_button_and_check_bit() {
        let (hi, lo) = make_frame(MB_LOCK, false, 1, SERIAL, COUNTER, HOP);
        let enc = MercedesV1Decoder::new();
        let wire = enc.encode(&signal_for(hi, lo), BTN_UNLOCK).expect("encode");
        let d = &decode_all(&wire)[0];
        let (exp_hi, exp_lo) = make_frame(MB_UNLOCK, false, 1, SERIAL, COUNTER, HOP);
        assert_eq!((d.data, d.extra), (exp_hi, Some(exp_lo >> 6)));
        assert_eq!(d.button, Some(MB_UNLOCK));
        // Only bits 1 and 3 differ from the captured frame.
        assert_eq!(d.data ^ hi, (1u64 << 62) | (1u64 << 60));
    }

    #[test]
    fn encoder_burst_structure() {
        let (hi, lo) = make_frame(MB_LOCK, false, 0, SERIAL, COUNTER, HOP);
        let wire = MercedesV1Decoder::new()
            .encode(&signal_for(hi, lo), 0x08)
            .unwrap();
        let gaps = wire
            .iter()
            .filter(|p| !p.level && p.duration_us == REPEAT_GAP_US)
            .count();
        assert_eq!(gaps, REPEAT_DEFAULT);
        // Preamble: 560 clean 1T pairs, then the 2T sync LOW.
        assert!(wire[..1120].iter().all(|p| p.duration_us == TE_US));
        assert_eq!((wire[1120].level, wire[1120].duration_us), (true, TE_US));
        assert_eq!(
            (wire[1121].level, wire[1121].duration_us),
            (false, 2 * TE_US)
        );
    }

    #[test]
    fn rejects_bad_check_bits() {
        let (hi, lo) = make_frame(MB_LOCK, true, 0, SERIAL, COUNTER, HOP);
        let (mut bad3_hi, mut bad3_lo) = (hi, lo);
        set_bit(&mut bad3_hi, &mut bad3_lo, 3, !get_bit(hi, lo, 3));
        assert!(
            decode_all(&build_wire(bad3_hi, bad3_lo, 40)).is_empty(),
            "bit 3 check"
        );
        let (mut bad121_hi, mut bad121_lo) = (hi, lo);
        set_bit(&mut bad121_hi, &mut bad121_lo, 121, false);
        assert!(
            decode_all(&build_wire(bad121_hi, bad121_lo, 40)).is_empty(),
            "bit 121"
        );
    }

    #[test]
    fn rejects_short_preamble() {
        let (hi, lo) = make_frame(MB_LOCK, true, 0, SERIAL, COUNTER, HOP);
        // The sync LOW is reached after `pairs - 1` counted 1T LOWs.
        assert!(decode_all(&build_wire(hi, lo, 32)).is_empty());
        assert_eq!(decode_all(&build_wire(hi, lo, 33)).len(), 1);
    }

    #[test]
    fn stretched_preamble_pulse_is_not_a_frame() {
        // A jitter-stretched (2T) HIGH mid-preamble makes the decoder enter data early and read
        // 122 preamble pairs as all-ones — a frame the C's field check accepts. KAT's constant-run
        // gate rejects it and resyncs, so the genuine frame that follows still decodes.
        let all_ones = (u64::MAX, LO_INVERT_MASK);
        assert!(
            fields_valid(all_ones.0, all_ones.1),
            "the C would accept this frame"
        );

        let (hi, lo) = make_frame(MB_UNLOCK, false, 1, SERIAL, COUNTER, HOP);
        let mut wire = build_wire(hi, lo, 400);
        let idx = 2 * 100; // a preamble HIGH
        assert!(wire[idx].level && wire[idx].duration_us == TE_US);
        wire[idx] = LevelDuration::new(true, 2 * TE_US);
        let decoded = decode_all(&wire);
        assert_eq!(decoded.len(), 1, "only the genuine frame decodes");
        assert_eq!((decoded[0].data, decoded[0].extra), (hi, Some(lo >> 6)));
    }

    /// Cross-protocol guard for all three decoders of this port (Mercedes V0/V1, Opel V1): no
    /// other registry protocol's own encoder output, at each of its frequencies, may decode as one
    /// of them. Ford V1 (65/130µs) and Honda Static (63µs) share Mercedes V1's chip windows; VAG
    /// and BMW-style 500µs streams overlap Mercedes V0; 250/500µs and 125/250µs streams overlap
    /// Opel V1. Ford V1 / VAG get `extra` shaped so their encoders produce output.
    #[test]
    fn foreign_encoder_output_is_never_claimed() {
        use std::panic::{catch_unwind, AssertUnwindSafe};
        const MINE: [&str; 3] = ["Mercedes V0", "Mercedes V1", "Opel V1"];
        let reg0 = ProtocolRegistry::new();
        for name in reg0.list_protocols() {
            if MINE.contains(&name) {
                continue;
            }
            let dec = reg0.get(name).unwrap();
            if !dec.supports_encoding() {
                continue;
            }
            let extras: &[u64] = match name {
                "Ford V1" => &[(1 << 40) | (0x5A << 32) | 0x1234], // strict-branch flag
                "VAG" => &[0x0001, 0x0002, 0x0003, 0x0004, 0x0104], // type | key_idx << 8
                _ => &[0x1234_5678_9ABC],
            };
            for &extra in extras {
                let src = DecodedSignal {
                    serial: Some(0x0012_3456),
                    button: Some(0x01),
                    counter: Some(5),
                    crc_valid: true,
                    data: 0xA5C3_1E7B_92D4_6F08,
                    data_count_bit: dec.timing().min_count_bit,
                    encoder_capable: true,
                    extra: Some(extra),
                    protocol_display_name: None,
                };
                for btn in [0x01u8, 0x02] {
                    // Another encoder panicking on these generic inputs is not this test's concern.
                    let Ok(Some(wire)) = catch_unwind(AssertUnwindSafe(|| dec.encode(&src, btn)))
                    else {
                        continue;
                    };
                    for &f in dec.supported_frequencies() {
                        let mut reg = ProtocolRegistry::new();
                        for (got, _, _) in reg.process_signal_stream(&wire, f) {
                            assert!(
                                !MINE.contains(&got.as_str()),
                                "{name} (extra {extra:#X}, btn {btn}) at {f} Hz decoded as {got}"
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn registry_reports_mercedes_v1() {
        let (hi, lo) = make_frame(MB_LOCK, true, 0, SERIAL, COUNTER, HOP);
        let wire = MercedesV1Decoder::new()
            .encode(&signal_for(hi, lo), 0x08)
            .unwrap();
        let mut reg = ProtocolRegistry::new();
        let results = reg.process_signal_stream(&wire, 433_920_000);
        let names: Vec<&str> = results.iter().map(|(n, _, _)| n.as_str()).collect();
        assert_eq!(names, vec!["Mercedes V1"; REPEAT_DEFAULT]);
    }

    /// BMW (62µs chips, same decoder template) lies inside this decoder's chip windows. Without the
    /// exact-length rule Mercedes V1 cut ~half of BMW's 128-bit frames at bit 122 and accepted them
    /// (its field check is only ~2 bits) before BMW completed. Every BMW frame must decode as BMW.
    #[test]
    fn bmw_frames_are_never_claimed() {
        let reg0 = ProtocolRegistry::new();
        let bmw = reg0.get("BMW").expect("BMW registered");
        let mut x: u64 = 0x9E37_79B9_7F4A_7C15;
        for _ in 0..64 {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            let serial = (x >> 32) as u32;
            let cnt = ((x >> 20) as u32) & 0x1FFFF;
            let hop = x & 0x7FFF_FFFF_FFFF;
            let hi = (0x599Eu64 << 48) | ((serial as u64) << 16) | (0x09u64 << 8) | 0x89;
            let lo = (((cnt >> 1) as u64) << 48) | (((cnt & 1) as u64) << 47) | hop;
            let src = DecodedSignal {
                serial: Some(serial),
                button: Some(1),
                counter: Some(cnt as u16),
                crc_valid: true,
                data: hi,
                data_count_bit: 128,
                encoder_capable: true,
                extra: Some(lo),
                protocol_display_name: None,
            };
            for btn in [0x01u8, 0x02, 0x04] {
                let wire = bmw.encode(&src, btn).expect("BMW encodes");
                let mut reg = ProtocolRegistry::new();
                let names: Vec<String> = reg
                    .process_signal_stream(&wire, 433_920_000)
                    .into_iter()
                    .map(|(n, _, _)| n)
                    .collect();
                assert!(
                    !names.is_empty() && names.iter().all(|n| n == "BMW"),
                    "BMW frame (serial {serial:08X}, btn {btn}) decoded as {names:?}"
                );
            }
        }
    }
}
