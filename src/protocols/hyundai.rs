//! Hyundai protocol decoder/encoder — EXPERIMENTAL (Pandora RE).
//!
//! Ported from Flipper-ARF: `lib/subghz/protocols/hundai.c` / `hundai.h`. Flipper-ARF names it
//! "Hundai" (`HUNDAI_PROTOCOL_NAME`); KAT uses the corrected spelling "Hyundai". The C was
//! reverse-engineered from the PANDORA_MAX alarm firmware (button/session dispatcher 0xf236,
//! Manchester bit-banger 0x8290) and describes the NEW Hyundai fob, distinct from the Kia V0–V7
//! family. Flipper-ARF ships it DISABLED (commented out of its `protocol_items.c`). It is the only
//! copy in REFERENCES, so it is followed as written.
//!
//! **Status: NOT registered** in `ProtocolRegistry::new()` (module kept so its tests run). The
//! frame has no checksum, so a decode cannot be self-validated; see "Gating" and the registry
//! comment.
//!
//! # Wire (C encoder; Manchester Te = 500µs, te_delta 125µs; AM/OOK; 315 / 433 MHz)
//!
//! `bit 1 = HIGH 500 → LOW 500`, `bit 0 = LOW 500 → HIGH 500`, bytes MSB-first. Each press sends
//! the frame TWICE:
//!
//! ```text
//! LOW 4500 | HIGH 1000 | LOW 500 | 32 × 0x55 | 9 body bytes
//! LOW 2500 | HIGH 1000 | LOW 500 | 32 × 0x55 | 9 body bytes | LOW 2500
//! ```
//!
//! # Body (72 bits)
//!
//! - b0..b3 = UID → `serial` (the C's 32-bit UID view); b4, b8 = more UID / a possible check byte
//!   (the C names b8 "check byte" but computes nothing from it)
//! - b5 low nibble = button (0x01 Lock, 0x02 Unlock, 0x09 Trunk) → `button`; high nibble kept
//! - b6 = session counter → `counter`
//! - b7 high nibble = rolling key index 1..15 (never 0); low nibble = channel / DIP
//!
//! Packing: `data` = b0..b7 (first 64 wire bits, MSB-first), `extra` = b8 (right-aligned),
//! `data_count_bit` = 72.
//!
//! # Decoder
//!
//! As the C: after a LOW ≥ 1500µs, half-bit cells are collected (short pulse = 1 cell, long = 2);
//! any other pulse ends the collection and the body is decoded from the TAIL of the cells (pairs
//! `H,L → 1`, `L,H → 0`), anchored on the 0x55 preamble right before it. No Flipper
//! `manchester_advance` state machine is involved: the C never calls it (it notes the firmware
//! machine cannot lock onto the merged 0x55 preamble), so neither does this port. Polarity is the
//! level as captured.
//!
//! # Gating (KAT is much stricter than the C — the frame carries no checksum)
//!
//! The C reports any tail with 4 bytes of 0x55 + 72 valid Manchester bits + `UID ∉ {0,
//! 0xFFFFFFFF}`; the "check byte" b8 and the duplicate frame are only DISPLAYED. KAT emits only
//! when:
//! 1. the body is exactly the last 72 bits before the gap and every pair is valid Manchester;
//! 2. it is immediately preceded by an unbroken 0x55 alternation of 128..=257 bits — at least 16
//!    of the 32 preamble bytes (C: 4), and no more than the 32 bytes + the header HIGH/LOW (which
//!    decodes as one extra `1`), so long-preamble 500/1000µs protocols (Kia V2's 252 long pairs)
//!    are rejected;
//! 3. `UID ∉ {0, 0xFFFFFFFF}` (as the C) and key index `b7 >> 4 != 0` (RE invariant; 15/16);
//! 4. the SAME 72-bit body is decoded twice in a row (the fob always sends each press twice; the
//!    C's "Dup: OK"). KAT therefore emits once per press, on the second copy; a lone frame never
//!    decodes. Two unrelated structurally-valid frames match with p = 2^-72.
//!
//! Noise essentially never yields 128+ alternating Manchester bits followed by 72 valid pairs;
//! the residual risk is another 500µs-Manchester protocol with a 16–32-byte 0x55 preamble, a
//! 72-bit body and an identical repeat. Buttons are NOT whitelisted (the RE only maps three; a
//! Panic/remote-start code probably exists). `crc_valid` = true reflects the duplicate check.
//!
//! # Encoder
//!
//! Supported, as in the C: rolling code with no known key, so the default is a byte-identical
//! REPLAY of the 9 bytes. If the requested button maps to a different b5 nibble (the C's D-pad
//! path) the nibble is re-stamped and b6 advanced by 1, rolling the b7 key index (1..15, skipping
//! 0) when b6 wraps. KAT generic buttons map Lock 0x01 → 0x01, Unlock 0x02 → 0x02, Trunk 0x04 →
//! 0x09 (native 0x09 passes through); any other code keeps the captured button. Waveform = the C
//! upload (both frames + trailing LOW 2500). Needs `extra` (b8); returns `None` without it.
//!
//! # Deviations from the C
//!
//! - Final-bit fix: when the last body bit is 1 its trailing LOW half-cell merges into the gap
//!   and is never pushed, so the C's tail window is one cell off and it silently drops every
//!   frame whose b8 is odd. KAT also tries the alignment with that merged half-cell restored (the
//!   0x55 anchor admits only the correct one).
//! - Gates 2 (min 16 bytes, max 257 bits instead of 4 bytes), 3 (key index) and 4 (duplicate
//!   required) are KAT additions; the C's re-report suppression is subsumed by gate 4.

use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Hyundai",
    decoder: "Hyundai",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "Hyundai",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&AM650],
};

const TE_SHORT: u32 = 500;
const TE_LONG: u32 = 1000;
const TE_DELTA: u32 = 125;
const HDR_LOW: u32 = 4500;
const HDR_HIGH: u32 = 1000;
const HDR_LOW2: u32 = 500;
const IFG_LOW: u32 = 2500;
const PREAMBLE_BYTES: usize = 32;
const PREAMBLE_BYTE: u8 = 0x55;
const WIRE_BYTES: usize = 9;
const WIRE_BITS: usize = 72;
/// A LOW at least this long is a header / inter-frame gap (not a Manchester cell).
const BOUNDARY_MIN_US: u32 = 1500;
const BODY_CELLS: usize = WIRE_BITS * 2;
/// KAT gate: at least 16 preamble bytes of 0x55 alternation right before the body (C: 4 bytes).
const MIN_PREAMBLE_BITS: usize = 128;
/// KAT gate: the 32-byte preamble + the header HIGH 1000 / LOW 500 (decodes as one more `1`).
const MAX_PREAMBLE_BITS: usize = PREAMBLE_BYTES * 8 + 1;
/// HUNDAI_CELL_CAPACITY: sliding cell window (holds the whole frame plus margin).
const CELL_CAPACITY: usize = (PREAMBLE_BYTES + WIRE_BYTES + 4) * 16 + 8;

/// b5 low-nibble button codes.
const BTN_0: u8 = 0x01; // Lock
const BTN_1: u8 = 0x02; // Unlock
const BTN_2: u8 = 0x09; // Trunk

/// Decoder steps (HundaiDecoderStep).
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Data,
}

/// Hyundai decoder (SubGhzProtocolDecoderHundai).
pub struct HyundaiDecoder {
    step: DecoderStep,
    /// Sliding window of half-bit cells (1 = HIGH, 0 = LOW).
    cells: [u8; CELL_CAPACITY],
    cell_count: usize,
    /// Last structurally valid body, awaiting its duplicate.
    prev_frame: Option<[u8; WIRE_BYTES]>,
}

fn uid(raw: &[u8; WIRE_BYTES]) -> u32 {
    u32::from_be_bytes([raw[0], raw[1], raw[2], raw[3]])
}

/// hundai_frame_valid: UID not all-zero / all-one.
fn frame_valid(raw: &[u8; WIRE_BYTES]) -> bool {
    let u = uid(raw);
    u != 0 && u != u32::MAX
}

fn key_index(raw: &[u8; WIRE_BYTES]) -> u8 {
    raw[7] >> 4
}

/// hundai_advance_counter: b6++, and on wrap advance the b7 key index (1..15, never 0).
fn advance_counter(raw: &mut [u8; WIRE_BYTES]) {
    raw[6] = raw[6].wrapping_add(1);
    if raw[6] == 0 {
        let mut hi = (key_index(raw) + 1) & 0x0F;
        if hi == 0 {
            hi = 1;
        }
        raw[7] = (hi << 4) | (raw[7] & 0x0F);
    }
}

impl HyundaiDecoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            cells: [0u8; CELL_CAPACITY],
            cell_count: 0,
            prev_frame: None,
        }
    }

    fn is_short(d: u32) -> bool {
        duration_diff!(d, TE_SHORT) < TE_DELTA
    }

    fn is_long(d: u32) -> bool {
        duration_diff!(d, TE_LONG) < TE_DELTA
    }

    /// Button name for display (hundai_button_name).
    #[allow(dead_code)]
    fn button_name(btn: u8) -> &'static str {
        match btn {
            BTN_0 => "Lock",
            BTN_1 => "Unlock",
            BTN_2 => "Trunk",
            _ => "Unknown",
        }
    }

    /// KAT generic button (or native nibble) → b5 nibble. `None` keeps the captured button.
    fn map_button(button: u8) -> Option<u8> {
        match button {
            0x01 => Some(BTN_0),
            0x02 => Some(BTN_1),
            0x04 | BTN_2 => Some(BTN_2),
            _ => None,
        }
    }

    /// hundai_push_cell (sliding window).
    fn push_cell(&mut self, level: bool) {
        let v = level as u8;
        if self.cell_count < CELL_CAPACITY {
            self.cells[self.cell_count] = v;
            self.cell_count += 1;
        } else {
            self.cells.copy_within(1.., 0);
            self.cells[CELL_CAPACITY - 1] = v;
        }
    }

    /// Decode the tail of `seq` as body + preamble anchor; apply gates 1-3.
    fn analyze(seq: &[u8]) -> Option<[u8; WIRE_BYTES]> {
        if seq.len() < BODY_CELLS + 2 * MIN_PREAMBLE_BITS {
            return None;
        }
        let body_start = seq.len() - BODY_CELLS;

        let mut frame = [0u8; WIRE_BYTES];
        for bit in 0..WIRE_BITS {
            let (a, b) = (seq[body_start + 2 * bit], seq[body_start + 2 * bit + 1]);
            if a == b {
                return None;
            }
            frame[bit / 8] = (frame[bit / 8] << 1) | a;
        }

        // Unbroken 0x55 alternation right before the body: ..., 0, 1 | body.
        let mut run = 0usize;
        let mut expected = 1u8;
        let mut i = body_start;
        while i >= 2 {
            let (a, b) = (seq[i - 2], seq[i - 1]);
            if a == b || a != expected {
                break;
            }
            run += 1;
            expected ^= 1;
            i -= 2;
        }
        if !(MIN_PREAMBLE_BITS..=MAX_PREAMBLE_BITS).contains(&run) {
            return None;
        }

        if !frame_valid(&frame) || key_index(&frame) == 0 {
            return None;
        }
        Some(frame)
    }

    /// hundai_try_decode at the end-of-frame pulse. Also tries the alignment where the body's
    /// final half-cell merged into the terminating pulse (C bug fix, see module docs).
    fn try_decode(&self, term_level: bool) -> Option<[u8; WIRE_BYTES]> {
        let mut seq = self.cells[..self.cell_count].to_vec();
        if let Some(f) = Self::analyze(&seq) {
            return Some(f);
        }
        seq.push(term_level as u8);
        Self::analyze(&seq)
    }

    fn build_signal(raw: &[u8; WIRE_BYTES]) -> DecodedSignal {
        let mut first8 = [0u8; 8];
        first8.copy_from_slice(&raw[..8]);
        DecodedSignal {
            serial: Some(uid(raw)),
            button: Some(raw[5] & 0x0F),
            counter: Some(raw[6] as u16),
            crc_valid: true, // duplicate-confirmed (no checksum exists)
            data: u64::from_be_bytes(first8),
            data_count_bit: WIRE_BITS,
            encoder_capable: true,
            extra: Some(raw[8] as u64),
            protocol_display_name: None,
        }
    }

    fn enc_add_level(signal: &mut Vec<LevelDuration>, level: bool, duration: u32) {
        if let Some(last) = signal.last_mut() {
            if last.level == level {
                *last = LevelDuration::new(level, last.duration_us + duration);
                return;
            }
        }
        signal.push(LevelDuration::new(level, duration));
    }

    /// hundai_emit_byte_msb: bit 1 = (H,L), bit 0 = (L,H).
    fn enc_byte(signal: &mut Vec<LevelDuration>, byte: u8) {
        for bit in (0..8).rev() {
            let one = (byte >> bit) & 1 != 0;
            Self::enc_add_level(signal, one, TE_SHORT);
            Self::enc_add_level(signal, !one, TE_SHORT);
        }
    }

    /// hundai_emit_frame: header (LOW 4500 first / 2500 repeat, HIGH 1000, LOW 500), preamble,
    /// body.
    fn enc_frame(signal: &mut Vec<LevelDuration>, raw: &[u8; WIRE_BYTES], first: bool) {
        Self::enc_add_level(signal, false, if first { HDR_LOW } else { IFG_LOW });
        Self::enc_add_level(signal, true, HDR_HIGH);
        Self::enc_add_level(signal, false, HDR_LOW2);
        for _ in 0..PREAMBLE_BYTES {
            Self::enc_byte(signal, PREAMBLE_BYTE);
        }
        for &b in raw.iter() {
            Self::enc_byte(signal, b);
        }
    }
}

impl ProtocolDecoder for HyundaiDecoder {
    fn name(&self) -> &'static str {
        "Hyundai"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: WIRE_BITS,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        // C flags: SubGhzProtocolFlag_315 | SubGhzProtocolFlag_433 (AM).
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.cell_count = 0;
        self.prev_frame = None;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let gap = !level && duration >= BOUNDARY_MIN_US;
        match self.step {
            DecoderStep::Reset => {
                if gap {
                    self.cell_count = 0;
                    self.step = DecoderStep::Data;
                }
                None
            }
            DecoderStep::Data => {
                if Self::is_short(duration) {
                    self.push_cell(level);
                    return None;
                }
                if Self::is_long(duration) {
                    self.push_cell(level);
                    self.push_cell(level);
                    return None;
                }
                // Header/inter-frame gap or noise: decode the tail, then restart collection
                // (a gap starts the next frame's window right away).
                let frame = self.try_decode(level);
                self.cell_count = 0;
                self.step = if gap {
                    DecoderStep::Data
                } else {
                    DecoderStep::Reset
                };
                let frame = frame?;
                if self.prev_frame == Some(frame) {
                    self.prev_frame = None;
                    return Some(Self::build_signal(&frame));
                }
                self.prev_frame = Some(frame);
                None
            }
        }
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        let b8 = decoded.extra?;
        let mut raw = [0u8; WIRE_BYTES];
        raw[..8].copy_from_slice(&decoded.data.to_be_bytes());
        raw[8] = (b8 & 0xFF) as u8;
        if !frame_valid(&raw) {
            return None;
        }

        let original = raw[5] & 0x0F;
        let new_btn = Self::map_button(button).unwrap_or(original);
        if new_btn != original {
            raw[5] = (raw[5] & 0xF0) | new_btn;
            advance_counter(&mut raw);
        }

        let mut signal = Vec::with_capacity(2 * (3 + (PREAMBLE_BYTES + WIRE_BYTES) * 16) + 1);
        Self::enc_frame(&mut signal, &raw, true);
        Self::enc_frame(&mut signal, &raw, false);
        Self::enc_add_level(&mut signal, false, IFG_LOW);
        Some(signal)
    }
}

impl Default for HyundaiDecoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::kia_v2::KiaV2Decoder;

    const SAMPLE: [u8; WIRE_BYTES] = [0x3C, 0x91, 0x7E, 0x05, 0xA2, 0x41, 0x17, 0x2B, 0x6D];

    fn captured(raw: &[u8; WIRE_BYTES]) -> DecodedSignal {
        HyundaiDecoder::build_signal(raw)
    }

    fn decode_all(dec: &mut dyn ProtocolDecoder, signal: &[LevelDuration]) -> Vec<DecodedSignal> {
        dec.reset();
        let mut out = Vec::new();
        for ld in signal {
            if let Some(d) = dec.feed(ld.level, ld.duration_us) {
                out.push(d);
                dec.reset();
            }
        }
        out
    }

    /// Frame with a custom preamble length (in 0x55 bytes), followed by a 2500µs gap.
    fn frame_with_preamble(raw: &[u8; WIRE_BYTES], preamble_bytes: usize) -> Vec<LevelDuration> {
        let mut s = Vec::new();
        HyundaiDecoder::enc_add_level(&mut s, false, HDR_LOW);
        HyundaiDecoder::enc_add_level(&mut s, true, HDR_HIGH);
        HyundaiDecoder::enc_add_level(&mut s, false, HDR_LOW2);
        for _ in 0..preamble_bytes {
            HyundaiDecoder::enc_byte(&mut s, PREAMBLE_BYTE);
        }
        for &b in raw.iter() {
            HyundaiDecoder::enc_byte(&mut s, b);
        }
        HyundaiDecoder::enc_add_level(&mut s, false, IFG_LOW);
        s
    }

    #[test]
    fn encode_decode_roundtrip_both_final_bits() {
        for b8 in [0x6Du8, 0x6C] {
            // b8 odd = final bit 1 (the case the C drops), even = final bit 0.
            let mut raw = SAMPLE;
            raw[8] = b8;
            let sig = HyundaiDecoder::new()
                .encode(&captured(&raw), 0x08)
                .expect("encode");
            let frames = decode_all(&mut HyundaiDecoder::new(), &sig);
            assert_eq!(frames.len(), 1, "one decode per press (b8={b8:#04X})");
            let f = &frames[0];
            assert_eq!(f.serial, Some(0x3C91_7E05));
            assert_eq!(f.button, Some(0x01));
            assert_eq!(f.counter, Some(0x17));
            assert_eq!(f.data, 0x3C91_7E05_A241_172B);
            assert_eq!(f.extra, Some(b8 as u64));
            assert_eq!(f.data_count_bit, 72);
            assert!(f.crc_valid && f.encoder_capable);
        }
    }

    /// `hundai_encoder_build_upload` / `hundai_emit_frame` / `hundai_emit_byte_msb`, transcribed
    /// verbatim from the C (no merging of adjacent same-level entries).
    fn c_build_upload(raw: &[u8; WIRE_BYTES]) -> Vec<LevelDuration> {
        fn emit_byte_msb(upload: &mut Vec<LevelDuration>, byte: u8) {
            for bit in 0..8 {
                if (byte >> (7 - bit)) & 1 != 0 {
                    upload.push(LevelDuration::new(true, TE_SHORT));
                    upload.push(LevelDuration::new(false, TE_SHORT));
                } else {
                    upload.push(LevelDuration::new(false, TE_SHORT));
                    upload.push(LevelDuration::new(true, TE_SHORT));
                }
            }
        }
        let mut upload = Vec::new();
        for first_frame in [true, false] {
            upload.push(LevelDuration::new(
                false,
                if first_frame { HDR_LOW } else { IFG_LOW },
            ));
            upload.push(LevelDuration::new(true, HDR_HIGH));
            upload.push(LevelDuration::new(false, HDR_LOW2));
            for _ in 0..PREAMBLE_BYTES {
                emit_byte_msb(&mut upload, PREAMBLE_BYTE);
            }
            for &b in raw.iter() {
                emit_byte_msb(&mut upload, b);
            }
        }
        upload.push(LevelDuration::new(false, IFG_LOW));
        upload
    }

    /// The C's own upload (unmerged) decodes to the captured bytes, once per press, for both
    /// final-bit values of b8.
    #[test]
    fn decodes_verbatim_c_upload() {
        for b8 in [0x6Du8, 0x6C, 0x01, 0xFE] {
            let mut raw = SAMPLE;
            raw[8] = b8;
            let frames = decode_all(&mut HyundaiDecoder::new(), &c_build_upload(&raw));
            assert_eq!(frames.len(), 1, "b8 = {b8:#04X}");
            assert_eq!(frames[0].data, 0x3C91_7E05_A241_172B);
            assert_eq!(frames[0].extra, Some(b8 as u64));
        }
    }

    /// The upload matches the C layout: LOW 4500, HIGH 1000, then the header LOW 500 merged with
    /// the first 0x55 half-cell, and the 0x55 preamble as 1000µs alternation; b8 = 0x6D ends in a
    /// 1-bit whose LOW half merges into the inter-frame / trailing 2500µs LOW.
    #[test]
    fn waveform_matches_c_upload_layout() {
        let sig = HyundaiDecoder::new()
            .encode(&captured(&SAMPLE), 0x08)
            .unwrap();
        let got: Vec<(bool, u32)> = sig[..6].iter().map(|p| (p.level, p.duration_us)).collect();
        assert_eq!(
            got,
            vec![
                (false, HDR_LOW),
                (true, 1000),
                (false, 1000),
                (true, 1000),
                (false, 1000),
                (true, 1000)
            ]
        );
        let merged_gap = (false, TE_SHORT + IFG_LOW);
        assert_eq!(
            sig.iter()
                .filter(|p| (p.level, p.duration_us) == merged_gap)
                .count(),
            2
        );
        assert_eq!(
            sig.last().map(|p| (p.level, p.duration_us)),
            Some(merged_gap)
        );
    }

    #[test]
    fn encode_button_change_restamps_and_advances() {
        // Trunk (generic 0x04 → nibble 0x09); b6 0xFF wraps → key index 2 → 3, channel kept.
        let mut raw = SAMPLE;
        raw[6] = 0xFF;
        let sig = HyundaiDecoder::new()
            .encode(&captured(&raw), 0x04)
            .expect("encode");
        let f = decode_all(&mut HyundaiDecoder::new(), &sig).remove(0);
        let bytes = f.data.to_be_bytes();
        assert_eq!(f.button, Some(BTN_2));
        assert_eq!(bytes[5], 0x49, "high nibble of b5 preserved");
        assert_eq!(f.counter, Some(0x00));
        assert_eq!(bytes[7], 0x3B, "key index 2 → 3, channel nibble 0xB kept");
        // Key index 15 rolls to 1, never 0.
        let mut r = raw;
        r[7] = 0xF4;
        advance_counter(&mut r);
        assert_eq!((r[6], r[7]), (0x00, 0x14));
        // Same button = byte-identical replay (no counter bump).
        let sig = HyundaiDecoder::new()
            .encode(&captured(&SAMPLE), 0x01)
            .expect("encode");
        let f = decode_all(&mut HyundaiDecoder::new(), &sig).remove(0);
        assert_eq!(f.data, 0x3C91_7E05_A241_172B);
    }

    #[test]
    fn synthetic_frames_with_short_preamble_decode() {
        // 16 of the 32 preamble bytes survive (e.g. receiver AGC settling): still accepted.
        let mut s = frame_with_preamble(&SAMPLE, 16);
        s.extend(frame_with_preamble(&SAMPLE, 32));
        let frames = decode_all(&mut HyundaiDecoder::new(), &s);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].serial, Some(0x3C91_7E05));
    }

    #[test]
    fn rejects_single_frame_and_mismatched_pair() {
        let single = frame_with_preamble(&SAMPLE, 32);
        assert!(
            decode_all(&mut HyundaiDecoder::new(), &single).is_empty(),
            "no duplicate"
        );

        let mut other = SAMPLE;
        other[2] ^= 0x01;
        let mut s = frame_with_preamble(&SAMPLE, 32);
        s.extend(frame_with_preamble(&other, 32));
        assert!(
            decode_all(&mut HyundaiDecoder::new(), &s).is_empty(),
            "frames differ"
        );
    }

    #[test]
    fn rejects_bad_preamble_lengths_and_fields() {
        let pair = |raw: &[u8; WIRE_BYTES], pre: usize| {
            let mut s = frame_with_preamble(raw, pre);
            s.extend(frame_with_preamble(raw, pre));
            s
        };
        // Too little preamble (C's 4-byte anchor would accept this).
        assert!(decode_all(&mut HyundaiDecoder::new(), &pair(&SAMPLE, 8)).is_empty());
        // Too much alternation (Kia V2-style long preamble).
        assert!(decode_all(&mut HyundaiDecoder::new(), &pair(&SAMPLE, 40)).is_empty());
        // UID all-zero.
        let mut z = SAMPLE;
        z[..4].copy_from_slice(&[0; 4]);
        assert!(decode_all(&mut HyundaiDecoder::new(), &pair(&z, 32)).is_empty());
        // Key index 0.
        let mut k = SAMPLE;
        k[7] = 0x0B;
        assert!(decode_all(&mut HyundaiDecoder::new(), &pair(&k, 32)).is_empty());
        // The same corrupted body pulse in both copies (a cell gained or lost): the duplicate
        // matches, but the Manchester / anchor alignment must reject it.
        let mut one = frame_with_preamble(&SAMPLE, 32);
        let idx = one.len() - 10;
        let d = if one[idx].duration_us == TE_SHORT {
            TE_LONG
        } else {
            TE_SHORT
        };
        one[idx] = LevelDuration::new(one[idx].level, d);
        let mut s = one.clone();
        s.extend(one);
        assert!(decode_all(&mut HyundaiDecoder::new(), &s).is_empty());
    }

    /// Kia V2 (500/1000µs Manchester, 252 long preamble pairs, 2 bursts) must not decode as
    /// Hyundai, even framed by silence so the Hyundai collector starts on it.
    #[test]
    fn does_not_decode_kia_v2_encoder_output() {
        let kia = KiaV2Decoder::new();
        let mut seed: u32 = 0xC0DE_0002;
        for _ in 0..100 {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let captured = DecodedSignal {
                serial: Some(seed),
                button: Some((seed >> 28) as u8),
                counter: Some((seed & 0xFFF) as u16),
                crc_valid: true,
                data: 0,
                data_count_bit: 53,
                encoder_capable: true,
                extra: None,
                protocol_display_name: None,
            };
            let mut sig = vec![LevelDuration::new(false, 5000)];
            sig.extend(kia.encode(&captured, 0x01).unwrap());
            sig.push(LevelDuration::new(false, 5000));
            let mut twice = sig.clone();
            twice.extend(sig);
            assert!(decode_all(&mut HyundaiDecoder::new(), &twice).is_empty());
        }
    }

    /// Random pulse noise (uniform 50..6000µs, and runs snapped to 500/1000µs with ≥1500µs LOW
    /// gaps sprinkled in so the collector keeps restarting) never decodes.
    #[test]
    fn random_noise_never_decodes() {
        let mut seed: u32 = 0xDEAD_BEEF;
        let mut level = true;
        let mut sig = Vec::with_capacity(200_000);
        for i in 0..200_000u32 {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let r = seed;
            let d = if (i / 1000) % 2 == 0 {
                50 + r % 5950
            } else if !level && r.is_multiple_of(61) {
                IFG_LOW
            } else {
                (if r & 1 == 0 { TE_SHORT } else { TE_LONG }) + (r >> 8) % 160 - 80
            };
            sig.push(LevelDuration::new(level, d));
            level = !level;
        }
        assert!(decode_all(&mut HyundaiDecoder::new(), &sig).is_empty());
    }

    /// The real Hyundai / Genesis captures that decode as nothing today, and the Kia captures
    /// (which must not be stolen), must not produce a Hyundai frame — standalone (no registry
    /// masking) and in either polarity.
    #[test]
    fn no_decode_on_hyundai_genesis_kia_imports() {
        let mut checked = 0;
        for dir in ["IMPORTS/HYUNDAI", "IMPORTS/genesis", "IMPORTS/KIA"] {
            let Ok(rd) = std::fs::read_dir(dir) else {
                eprintln!("Skip: {dir} not found (run from crate root)");
                continue;
            };
            for entry in rd.flatten() {
                let path = entry.path();
                if path.extension().is_none_or(|e| e != "sub") {
                    continue;
                }
                let Ok((_, raw)) = crate::export::flipper::import_sub_raw(&path) else {
                    continue;
                };
                for inv in [false, true] {
                    let sig: Vec<LevelDuration> = raw
                        .iter()
                        .map(|p| LevelDuration::new(p.level != inv, p.duration_us))
                        .collect();
                    assert!(
                        decode_all(&mut HyundaiDecoder::new(), &sig).is_empty(),
                        "{path:?} inv={inv}"
                    );
                }
                checked += 1;
            }
        }
        assert!(checked == 0 || checked >= 15, "checked {checked} captures");
    }
}
