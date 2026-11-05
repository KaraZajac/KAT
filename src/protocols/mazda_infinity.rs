//! Mazda Infinity protocol decoder/encoder — EXPERIMENTAL (Pandora RE).
//!
//! Ported from Flipper-ARF: `lib/subghz/protocols/mazda_infinity.c` / `mazda_infinity.h`
//! (`MAZDA_INFINITY_PROTOCOL_NAME = "Mazda Infinity"`). The C was reverse-engineered from the
//! PANDORA_MAX alarm firmware (TX dispatcher 0xf0d0, RF bit-banger 0x8fc4, whitening/interleave
//! 0x12a90, popcount parity 0x10994); Pandora units emulate the OEM fob. Flipper-ARF ships it
//! DISABLED (commented out of its `protocol_items.c`). It is the only copy in REFERENCES
//! (ProtoPirate / doctor-firmware have no "Mazda Infinity"), so it is followed as written.
//!
//! # Wire (C encoder, Manchester Te = 250µs, te_delta 62µs, AM/OOK, 315/433 MHz)
//!
//! `bit 1 = HIGH 250 → LOW 250`, `bit 0 = LOW 250 → HIGH 250`, every byte MSB-first:
//!
//! ```text
//! 12 × 0xFF (preamble, raw) | LOW 5000 gap | FF FF D7 (sync, raw) | 8 payload bytes (one's
//! complemented on air) | 0x5A (trailer, raw)
//! ```
//!
//! The C upload is ONE such frame and the Flipper plays it `MAZDA_INF_DEFAULT_REPEAT = 4` times.
//!
//! # Logical payload (8 bytes, `data` = b0..b7 MSB-first, as the C's `generic.data`)
//!
//! - b0..b3 = UID / serial (32-bit `serial`)
//! - b4 = button byte (0x10 Lock, 0x20 Unlock, 0x40 Trunk — "best-effort", raw b4 is kept)
//! - b5:b6 = 16-bit counter (high:low)
//! - b7 = checksum = `sum(b0..b6) & 0xFF`
//!
//! Building the wire: checksum → counter interleave (`b5' = b5&0xAA | b6&0x55`,
//! `b6' = b5&0x55 | b6&0xAA`, self-inverse) → whitening keyed by `popcount(b7) & 1` (odd: b0..b5
//! ^= b6; even: b0..b4, b6 ^= b5; self-inverse, b7 untouched) → one's complement of all 8 bytes.
//! Decoding reverses it (`wire_to_logical`).
//!
//! # Relationship to Mazda V0 / Mazda Siemens
//!
//! This is the SAME logical layer as KAT's `mazda_v0.rs` / `mazda_siemens.rs` (identical parity-
//! keyed XOR, b5/b6 interleave, additive checksum, 0x10/0x20/0x40 buttons) and the same air format
//! that ProtoPirate's current `mazda_v0.c` encoder and Flipper-ARF's `mazda_siemens.c` encoder
//! emit (12×0xFF, gap, FF FF D7, inverted bytes, 0x5A). KAT's Mazda V0 (a port of ProtoPirate's
//! current decoder) decodes this format too and completes on the 64th payload bit, before the
//! trailer this decoder waits for — so on a shared frame Mazda V0 keeps first-match priority and
//! Mazda Infinity only reports frames Mazda V0 and Mazda Siemens both reject. The unit test
//! `does_not_change_mazda_v0_or_siemens_results` checks that registering Mazda Infinity after them
//! leaves their first-match results unchanged, and `decodes_mazda_siemens_encoder_output` shows
//! Mazda Infinity decodes the Siemens TX upload. Field reading differs slightly from ProtoPirate's
//! Mazda V0 (full b4 byte + 16-bit counter vs. a b4 nibble + 20-bit counter).
//!
//! # Decoder (mirrors the C)
//!
//! No Flipper `manchester_advance` state machine is involved: the C never calls it (it decodes
//! half-cells itself), so neither does this port. Polarity is the level as captured.
//! Half-bit Manchester "cells" are pushed into a 384-cell (12-byte) sliding window — a short
//! pulse is one cell, a long pulse two identical cells — and after EVERY push the window is
//! decoded as sync(3) + data(8) + trailer(1) bytes (pairs `H,L → 1`, `L,H → 0`; a pair of equal
//! cells is invalid). Any pulse that is neither short nor long ends the collection (the C also
//! tries a final decode there). After an emission the window is cleared, so each frame is emitted
//! once; the C's cross-repeat "identical frame" de-dup is not ported (KAT resets every decoder
//! after an emission, which would clear that state anyway, and other KAT decoders emit every
//! repeat).
//!
//! # Gating (KAT is stricter than the C)
//!
//! The C requires sync `FF FF D7` + trailer `0x5A` + 96 valid Manchester pairs + `UID ∉ {0,
//! 0xFFFFFFFF}`, and only DISPLAYS the checksum (`CS:[OK|BAD]`). KAT additionally REJECTS on the
//! checksum. Strength on a random 96-bit Manchester window: 32 fixed sync/trailer bits (2^-32) ×
//! checksum (2^-8) ≈ 2^-40 per alignment; random noise rarely even yields 96 valid pairs. Buttons
//! are not whitelisted (a Remote/Panic code such as 0x80 may exist on real fobs).
//!
//! # Encoder
//!
//! Supported, as in the C: the logical payload is re-derived to the wire (checksum, interleave,
//! whitening and inversion are recomputed). If the requested button maps to the captured b4 the
//! frame is replayed byte-identically; otherwise b4 is re-stamped and the 16-bit counter is
//! advanced by 1 (the C's `rolling_counter_mult` default) before re-deriving. KAT generic buttons
//! map Lock 0x01 → 0x10, Unlock 0x02 → 0x20, Trunk 0x04 → 0x40 (native codes pass through); any
//! other code keeps the captured button. The waveform is the C upload repeated 4 times (the
//! Flipper's default repeat — the upload itself is a single frame) plus a trailing 5 ms LOW so the
//! last frame is terminated.
//!
//! # Deviations from the C
//!
//! - Checksum is a hard gate (C: display only).
//! - Repeated-frame de-dup dropped: every repeat is emitted (KAT resets decoders after every
//!   emission, as for its other protocols).

use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Mazda Infinity",
    decoder: "Mazda Infinity",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "Mazda",
    category: Category::Vehicle,
    buttons: Some(super::mazda_siemens::native_buttons),
    presets: &[&AM650],
};

const TE_SHORT: u32 = 250;
const TE_LONG: u32 = 500;
const TE_DELTA: u32 = 62;
/// Preamble → sync gap emitted by the C encoder (and our trailing terminator).
const GAP_US: u32 = 5000;
const PREAMBLE_BYTES: usize = 12;
const PAYLOAD_BYTES: usize = 8;
const PAYLOAD_BITS: usize = 64;
const SYNC: [u8; 3] = [0xFF, 0xFF, 0xD7];
const TRAILER: u8 = 0x5A;
/// sync(3) + data(8) + trailer(1).
const WINDOW_BYTES: usize = 12;
/// Two half-bit cells per bit.
const WINDOW_CELLS: usize = WINDOW_BYTES * 8 * 2;
/// MAZDA_INF_DEFAULT_REPEAT: how many times the Flipper plays the single-frame upload.
const TX_REPEAT: usize = 4;

/// On-wire button codes (b4).
const BTN_LOCK: u8 = 0x10;
const BTN_UNLOCK: u8 = 0x20;
const BTN_TRUNK: u8 = 0x40;

/// Decoder steps (MazdaInfDecoderStep).
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Data,
}

/// Mazda Infinity decoder (SubGhzProtocolDecoderMazdaInfinity).
pub struct MazdaInfinityDecoder {
    step: DecoderStep,
    /// Sliding window of half-bit cells (1 = HIGH, 0 = LOW).
    cells: [u8; WINDOW_CELLS],
    cell_count: usize,
}

/// popcount(v) & 1 (mazda_inf_parity).
fn parity8(mut v: u8) -> u8 {
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    v & 1
}

/// sum(b0..b6) & 0xFF (mazda_inf_checksum).
fn checksum(b: &[u8; PAYLOAD_BYTES]) -> u8 {
    b[..7].iter().fold(0u8, |acc, &x| acc.wrapping_add(x))
}

/// Counter interleave of b5/b6 (mazda_inf_interleave; self-inverse).
fn interleave(b: &mut [u8; PAYLOAD_BYTES]) {
    let b5 = b[5];
    let b6 = b[6];
    b[5] = (b5 & 0xAA) | (b6 & 0x55);
    b[6] = (b5 & 0x55) | (b6 & 0xAA);
}

/// Data whitening keyed by popcount(b7) & 1 (mazda_inf_whiten; self-inverse, b7 untouched).
fn whiten(b: &mut [u8; PAYLOAD_BYTES]) {
    if parity8(b[7]) == 1 {
        let mask = b[6];
        for x in b.iter_mut().take(6) {
            *x ^= mask;
        }
    } else {
        let mask = b[5];
        for x in b.iter_mut().take(5) {
            *x ^= mask;
        }
        b[6] ^= mask;
    }
}

/// logical → checksum → interleave → whiten → one's complement (mazda_inf_logical_to_wire).
fn logical_to_wire(logical: &[u8; PAYLOAD_BYTES]) -> [u8; PAYLOAD_BYTES] {
    let mut b = *logical;
    b[7] = checksum(&b);
    interleave(&mut b);
    whiten(&mut b);
    for x in b.iter_mut() {
        *x = !*x;
    }
    b
}

/// wire → un-invert → un-whiten → de-interleave (mazda_inf_wire_to_logical).
fn wire_to_logical(wire: &[u8; PAYLOAD_BYTES]) -> [u8; PAYLOAD_BYTES] {
    let mut b = *wire;
    for x in b.iter_mut() {
        *x = !*x;
    }
    whiten(&mut b);
    interleave(&mut b);
    b
}

fn uid(logical: &[u8; PAYLOAD_BYTES]) -> u32 {
    u32::from_be_bytes([logical[0], logical[1], logical[2], logical[3]])
}

fn counter(logical: &[u8; PAYLOAD_BYTES]) -> u16 {
    u16::from_be_bytes([logical[5], logical[6]])
}

/// mazda_inf_frame_valid: UID not all-zero / all-one.
fn frame_valid(logical: &[u8; PAYLOAD_BYTES]) -> bool {
    let u = uid(logical);
    u != 0 && u != u32::MAX
}

impl MazdaInfinityDecoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            cells: [0u8; WINDOW_CELLS],
            cell_count: 0,
        }
    }

    fn is_short(d: u32) -> bool {
        duration_diff!(d, TE_SHORT) < TE_DELTA
    }

    fn is_long(d: u32) -> bool {
        duration_diff!(d, TE_LONG) < TE_DELTA
    }

    fn clear_cells(&mut self) {
        self.cell_count = 0;
    }

    /// Button name for display (mazda_inf_button_name).
    #[allow(dead_code)]
    fn button_name(btn: u8) -> &'static str {
        match btn {
            BTN_LOCK => "Lock",
            BTN_UNLOCK => "Unlock",
            BTN_TRUNK => "Trunk",
            _ => "Unknown",
        }
    }

    /// Map a KAT generic button (Lock 0x01, Unlock 0x02, Trunk 0x04) or a native b4 code to the
    /// on-wire b4 code. `None` = no mapping (keep the captured button).
    fn map_button(button: u8) -> Option<u8> {
        match button {
            0x01 => Some(BTN_LOCK),
            0x02 => Some(BTN_UNLOCK),
            0x04 => Some(BTN_TRUNK),
            BTN_LOCK | BTN_UNLOCK | BTN_TRUNK => Some(button),
            _ => None,
        }
    }

    /// Decode `bytes` MSB-first bytes from `cells[off..]` (mazda_inf_cells_to_bytes).
    /// `HIGH,LOW → 1`, `LOW,HIGH → 0`; equal cells → invalid.
    fn cells_to_bytes(cells: &[u8], off: usize, out: &mut [u8]) -> bool {
        for (byte_idx, out_byte) in out.iter_mut().enumerate() {
            let mut v = 0u8;
            for bit in 0..8 {
                let i = off + (byte_idx * 8 + bit) * 2;
                let (first, second) = (cells[i], cells[i + 1]);
                if first == second {
                    return false;
                }
                v = (v << 1) | first;
            }
            *out_byte = v;
        }
        true
    }

    /// mazda_inf_try_decode: decode the full window at offset 0 (the window slides by one cell per
    /// push, so every alignment is tried as cells arrive). Same checks as the C, ordered to fail
    /// fast: sync bytes, trailer, then the payload.
    fn try_decode(&self) -> Option<DecodedSignal> {
        if self.cell_count < WINDOW_CELLS {
            return None;
        }
        let mut win = [0u8; WINDOW_BYTES];
        for (i, &expected) in SYNC.iter().enumerate() {
            if !Self::cells_to_bytes(&self.cells, i * 16, &mut win[i..i + 1]) || win[i] != expected
            {
                return None;
            }
        }
        let t = WINDOW_BYTES - 1;
        if !Self::cells_to_bytes(&self.cells, t * 16, &mut win[t..])
            || win[t] != TRAILER
            || !Self::cells_to_bytes(&self.cells, 3 * 16, &mut win[3..t])
        {
            return None;
        }
        let mut wire = [0u8; PAYLOAD_BYTES];
        wire.copy_from_slice(&win[3..3 + PAYLOAD_BYTES]);
        let logical = wire_to_logical(&wire);
        if !frame_valid(&logical) {
            return None;
        }
        // KAT gate: the C only displays CS:[OK|BAD]; reject a bad checksum.
        if logical[7] != checksum(&logical) {
            return None;
        }
        Some(Self::build_signal(&logical))
    }

    fn build_signal(logical: &[u8; PAYLOAD_BYTES]) -> DecodedSignal {
        DecodedSignal {
            serial: Some(uid(logical)),
            button: Some(logical[4]),
            counter: Some(counter(logical)),
            crc_valid: true,
            data: u64::from_be_bytes(*logical),
            data_count_bit: PAYLOAD_BITS,
            encoder_capable: true,
            extra: None,
            protocol_display_name: None,
        }
    }

    /// Push one cell into the sliding window, then try to decode (mazda_inf_push_cell).
    fn push_cell(&mut self, level: bool) -> Option<DecodedSignal> {
        let v = level as u8;
        if self.cell_count < WINDOW_CELLS {
            self.cells[self.cell_count] = v;
            self.cell_count += 1;
        } else {
            self.cells.copy_within(1.., 0);
            self.cells[WINDOW_CELLS - 1] = v;
        }
        self.try_decode()
    }

    /// Push the cells for one pulse (1 for short, 2 for long). Returns the first decode, after
    /// which the window is cleared so the same frame cannot be re-found by the end-of-collection
    /// re-check (the C suppresses that duplicate with its `last_logical` de-dup).
    fn push_pulse(&mut self, level: bool, cells: usize) -> Option<DecodedSignal> {
        for _ in 0..cells {
            if let Some(sig) = self.push_cell(level) {
                self.clear_cells();
                self.step = DecoderStep::Reset;
                return Some(sig);
            }
        }
        None
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

    /// mazda_inf_emit_byte: MSB-first Manchester, bit 1 = (H,L), bit 0 = (L,H).
    fn enc_byte(signal: &mut Vec<LevelDuration>, byte: u8) {
        for bit in (0..8).rev() {
            let one = (byte >> bit) & 1 != 0;
            Self::enc_add_level(signal, one, TE_SHORT);
            Self::enc_add_level(signal, !one, TE_SHORT);
        }
    }

    /// One C upload (mazda_inf_encoder_build_upload): preamble, gap, sync, data, trailer.
    fn enc_frame(signal: &mut Vec<LevelDuration>, wire: &[u8; PAYLOAD_BYTES]) {
        for _ in 0..PREAMBLE_BYTES {
            Self::enc_byte(signal, 0xFF);
        }
        Self::enc_add_level(signal, false, GAP_US);
        for &b in SYNC.iter() {
            Self::enc_byte(signal, b);
        }
        for &b in wire.iter() {
            Self::enc_byte(signal, b);
        }
        Self::enc_byte(signal, TRAILER);
    }
}

impl ProtocolDecoder for MazdaInfinityDecoder {
    fn name(&self) -> &'static str {
        "Mazda Infinity"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: PAYLOAD_BITS,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        // C flags: SubGhzProtocolFlag_315 | SubGhzProtocolFlag_433 (AM).
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.clear_cells();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let cells = if Self::is_short(duration) {
            1
        } else if Self::is_long(duration) {
            2
        } else {
            0
        };

        match self.step {
            DecoderStep::Reset => {
                if cells > 0 {
                    self.clear_cells();
                    self.step = DecoderStep::Data;
                    return self.push_pulse(level, cells);
                }
                None
            }
            DecoderStep::Data => {
                if cells > 0 {
                    return self.push_pulse(level, cells);
                }
                // Inter-frame gap or noise: final decode attempt, then reset.
                let result = self.try_decode();
                self.clear_cells();
                self.step = DecoderStep::Reset;
                result
            }
        }
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        let mut logical = decoded.data.to_be_bytes();
        if !frame_valid(&logical) {
            return None;
        }

        let original_btn = logical[4];
        let new_btn = Self::map_button(button).unwrap_or(original_btn);
        if new_btn != original_btn {
            // Fresh press on another button: re-stamp b4 and advance the 16-bit counter by the
            // C's default rolling_counter_mult (1).
            let base = decoded.counter.unwrap_or_else(|| counter(&logical));
            let new_cnt = base.wrapping_add(1);
            logical[4] = new_btn;
            logical[5..7].copy_from_slice(&new_cnt.to_be_bytes());
        }
        logical[7] = checksum(&logical);
        let wire = logical_to_wire(&logical);

        let mut signal = Vec::with_capacity(TX_REPEAT * (PREAMBLE_BYTES + WINDOW_BYTES) * 16 + 2);
        for _ in 0..TX_REPEAT {
            Self::enc_frame(&mut signal, &wire);
        }
        Self::enc_add_level(&mut signal, false, GAP_US);
        Some(signal)
    }
}

impl Default for MazdaInfinityDecoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::mazda_siemens::MazdaSiemensDecoder;
    use crate::protocols::mazda_v0::MazdaV0Decoder;

    fn logical_frame(serial: u32, btn: u8, cnt: u16) -> [u8; PAYLOAD_BYTES] {
        let mut b = [0u8; PAYLOAD_BYTES];
        b[0..4].copy_from_slice(&serial.to_be_bytes());
        b[4] = btn;
        b[5..7].copy_from_slice(&cnt.to_be_bytes());
        b[7] = checksum(&b);
        b
    }

    fn signal_for(logical: &[u8; PAYLOAD_BYTES]) -> DecodedSignal {
        MazdaInfinityDecoder::build_signal(logical)
    }

    /// Feed a waveform through a fresh decoder; collect every emitted frame (resetting after each
    /// emission, as the registry does).
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

    /// A single C-format frame (one upload) followed by a terminating gap.
    fn one_frame(logical: &[u8; PAYLOAD_BYTES]) -> Vec<LevelDuration> {
        let mut s = Vec::new();
        MazdaInfinityDecoder::enc_frame(&mut s, &logical_to_wire(logical));
        MazdaInfinityDecoder::enc_add_level(&mut s, false, GAP_US);
        s
    }

    #[test]
    fn whitening_round_trips() {
        // logical → wire → logical is the identity for any payload whose b7 is the checksum.
        let mut seed: u32 = 0x1357_9BDF;
        for _ in 0..20_000 {
            seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            let s = seed;
            seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            let logical = logical_frame(s, (seed >> 24) as u8, (seed >> 8) as u16);
            assert_eq!(wire_to_logical(&logical_to_wire(&logical)), logical);
        }
    }

    #[test]
    fn encode_decode_roundtrip_replay() {
        let logical = logical_frame(0x1A2B_3C4D, BTN_UNLOCK, 0x0123);
        let dec = MazdaInfinityDecoder::new();
        // Same button (generic Unlock → 0x20) = byte-identical replay.
        let sig = dec.encode(&signal_for(&logical), 0x02).expect("encode");
        let mut rx = MazdaInfinityDecoder::new();
        let frames = decode_all(&mut rx, &sig);
        assert_eq!(frames.len(), TX_REPEAT, "every repeat decodes");
        for f in &frames {
            assert_eq!(f.serial, Some(0x1A2B_3C4D));
            assert_eq!(f.button, Some(BTN_UNLOCK));
            assert_eq!(f.counter, Some(0x0123));
            assert_eq!(f.data, u64::from_be_bytes(logical));
            assert!(f.crc_valid && f.encoder_capable);
            assert_eq!(f.data_count_bit, 64);
        }
    }

    /// The upload matches the C layout: 12 × 0xFF as alternating 250µs H/L (bit 1 = HIGH first),
    /// the last preamble LOW merged into the 5 ms gap, then the sync's first HIGH.
    #[test]
    fn waveform_matches_c_upload_layout() {
        let logical = logical_frame(0x1A2B_3C4D, BTN_UNLOCK, 0x0123);
        let sig = MazdaInfinityDecoder::new()
            .encode(&signal_for(&logical), 0x02)
            .unwrap();
        for (i, p) in sig[..191].iter().enumerate() {
            assert_eq!(
                (p.level, p.duration_us),
                (i % 2 == 0, TE_SHORT),
                "preamble pulse {i}"
            );
        }
        assert_eq!(
            (sig[191].level, sig[191].duration_us),
            (false, TE_SHORT + GAP_US)
        );
        assert_eq!((sig[192].level, sig[192].duration_us), (true, TE_SHORT));
        assert_eq!(
            sig.last().map(|p| p.level),
            Some(false),
            "terminated by a LOW gap"
        );
    }

    #[test]
    fn encode_button_change_advances_counter() {
        let logical = logical_frame(0x0BAD_F00D, BTN_LOCK, 0xFFFF);
        let dec = MazdaInfinityDecoder::new();
        let sig = dec.encode(&signal_for(&logical), 0x04).expect("encode");
        let frames = decode_all(&mut MazdaInfinityDecoder::new(), &sig);
        let f = frames.first().expect("decode");
        assert_eq!(f.button, Some(BTN_TRUNK));
        assert_eq!(f.counter, Some(0x0000), "16-bit counter wraps");
        assert_eq!(f.serial, Some(0x0BAD_F00D));
        // Unknown generic code (Panic 0x08) keeps the captured button and replays.
        let sig = dec.encode(&signal_for(&logical), 0x08).expect("encode");
        let f = decode_all(&mut MazdaInfinityDecoder::new(), &sig).remove(0);
        assert_eq!(f.button, Some(BTN_LOCK));
        assert_eq!(f.counter, Some(0xFFFF));
    }

    #[test]
    fn synthetic_wire_frame_decodes_with_known_fields() {
        // Hand-built logical payload → C wire → decode. Covers both whitening branches.
        for (serial, btn, cnt) in [
            (0x1234_5678u32, BTN_LOCK, 0x0001u16),
            (0x8765_4321, BTN_TRUNK, 0xBEEF),
            (0x0000_0001, 0x80, 0x7F00),
        ] {
            let logical = logical_frame(serial, btn, cnt);
            let frames = decode_all(&mut MazdaInfinityDecoder::new(), &one_frame(&logical));
            assert_eq!(frames.len(), 1, "serial {serial:#X}");
            assert_eq!(frames[0].serial, Some(serial));
            assert_eq!(frames[0].button, Some(btn));
            assert_eq!(frames[0].counter, Some(cnt));
            // Even without a reset after the emission, the frame is emitted exactly once.
            let mut dec = MazdaInfinityDecoder::new();
            let n = one_frame(&logical)
                .iter()
                .filter_map(|p| dec.feed(p.level, p.duration_us))
                .count();
            assert_eq!(n, 1);
        }
    }

    #[test]
    fn rejects_bad_checksum_sync_and_trailer() {
        let logical = logical_frame(0x1234_5678, BTN_LOCK, 0x0042);
        let good = logical_to_wire(&logical);

        // Corrupt one payload bit on the wire: the checksum gate (KAT-only) rejects it.
        let mut bad = good;
        bad[2] ^= 0x10;
        let mut s = Vec::new();
        MazdaInfinityDecoder::enc_frame(&mut s, &bad);
        MazdaInfinityDecoder::enc_add_level(&mut s, false, GAP_US);
        assert!(
            decode_all(&mut MazdaInfinityDecoder::new(), &s).is_empty(),
            "bad checksum"
        );

        // Wrong trailer byte.
        let mut s = Vec::new();
        for _ in 0..PREAMBLE_BYTES {
            MazdaInfinityDecoder::enc_byte(&mut s, 0xFF);
        }
        MazdaInfinityDecoder::enc_add_level(&mut s, false, GAP_US);
        for &b in SYNC.iter() {
            MazdaInfinityDecoder::enc_byte(&mut s, b);
        }
        for &b in good.iter() {
            MazdaInfinityDecoder::enc_byte(&mut s, b);
        }
        MazdaInfinityDecoder::enc_byte(&mut s, 0x5B);
        MazdaInfinityDecoder::enc_add_level(&mut s, false, GAP_US);
        assert!(
            decode_all(&mut MazdaInfinityDecoder::new(), &s).is_empty(),
            "bad trailer"
        );

        // Wrong sync byte.
        let mut s = Vec::new();
        for &b in [0xFFu8, 0xFF, 0xD6].iter().chain(good.iter()) {
            MazdaInfinityDecoder::enc_byte(&mut s, b);
        }
        MazdaInfinityDecoder::enc_byte(&mut s, TRAILER);
        MazdaInfinityDecoder::enc_add_level(&mut s, false, GAP_US);
        assert!(
            decode_all(&mut MazdaInfinityDecoder::new(), &s).is_empty(),
            "bad sync"
        );
    }

    #[test]
    fn rejects_all_zero_uid() {
        let logical = logical_frame(0, BTN_LOCK, 0x0042);
        assert!(decode_all(&mut MazdaInfinityDecoder::new(), &one_frame(&logical)).is_empty());
    }

    /// `mazda_inf_encoder_build_upload` / `mazda_inf_emit_byte` / `mazda_inf_logical_to_wire`,
    /// transcribed verbatim from the C (one frame, no merging of adjacent same-level entries).
    fn c_build_upload(logical: &[u8; PAYLOAD_BYTES]) -> Vec<LevelDuration> {
        fn emit_byte(upload: &mut Vec<LevelDuration>, byte: u8) {
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
        // logical -> checksum -> interleave -> whiten -> 255 - x (written out from the C).
        let mut b = *logical;
        b[7] = b[..7].iter().fold(0u8, |a, &x| a.wrapping_add(x));
        let (b5, b6) = (b[5], b[6]);
        b[5] = (b5 & 0xAA) | (b6 & 0x55);
        b[6] = (b5 & 0x55) | (b6 & 0xAA);
        if b[7].count_ones() & 1 == 1 {
            for k in 0..6 {
                b[k] ^= b[6];
            }
        } else {
            for k in [0, 1, 2, 3, 4, 6] {
                b[k] ^= b[5];
            }
        }
        let mut upload = Vec::new();
        for _ in 0..PREAMBLE_BYTES {
            emit_byte(&mut upload, 0xFF);
        }
        upload.push(LevelDuration::new(false, GAP_US));
        for byte in [0xFF, 0xFF, 0xD7] {
            emit_byte(&mut upload, byte);
        }
        for &x in b.iter() {
            emit_byte(&mut upload, 255 - x);
        }
        emit_byte(&mut upload, 0x5A);
        upload
    }

    /// The C's own single-frame upload (unmerged, no trailing gap: the window completes on the
    /// trailer's last cell) decodes to the logical payload, for both whitening branches.
    #[test]
    fn decodes_verbatim_c_upload() {
        let mut parities = [false; 2];
        for (serial, btn, cnt) in [
            (0x1234_5678u32, BTN_LOCK, 0x0001u16),
            (0x8765_4321, BTN_TRUNK, 0xBEEF),
            (0x0BAD_F00D, BTN_UNLOCK, 0x7F00),
            (0x0000_0001, 0x80, 0x0102),
        ] {
            let logical = logical_frame(serial, btn, cnt);
            parities[parity8(logical[7]) as usize] = true;
            let frames = decode_all(&mut MazdaInfinityDecoder::new(), &c_build_upload(&logical));
            assert_eq!(frames.len(), 1, "serial {serial:#X}");
            assert_eq!(frames[0].data, u64::from_be_bytes(logical));
        }
        assert!(parities[0] && parities[1], "cover both whitening branches");
    }

    /// Registry-style first match: every decoder sees every pulse, the first `Some` in vec order
    /// wins, then all reset. Returns the winners' names.
    fn first_match(
        decoders: &mut [Box<dyn ProtocolDecoder>],
        signal: &[LevelDuration],
    ) -> Vec<&'static str> {
        decoders.iter_mut().for_each(|d| d.reset());
        let mut out = Vec::new();
        for p in signal {
            let mut winner = None;
            for d in decoders.iter_mut() {
                if d.feed(p.level, p.duration_us).is_some() && winner.is_none() {
                    winner = Some(d.name());
                }
            }
            if let Some(w) = winner {
                out.push(w);
                decoders.iter_mut().for_each(|d| d.reset());
            }
        }
        out
    }

    /// Mazda V0 / Mazda Siemens must not be affected: on this air format (both polarities, many
    /// payloads) their first-match results with Mazda Infinity registered after them equal their
    /// results without it. Mazda V0 is a port of ProtoPirate's current Manchester mazda_v0.c, whose
    /// encoder emits this same frame; it fires on the 64th payload bit, before the trailer Mazda
    /// Infinity waits for, so it keeps these frames and Mazda Infinity only reports frames that
    /// Mazda V0 and Mazda Siemens both reject.
    #[test]
    fn does_not_change_mazda_v0_or_siemens_results() {
        let mut seed: u32 = 0x00C0_FFEE;
        let mut decoded = 0;
        for _ in 0..300 {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let serial = (seed | 1) & 0x7FFF_FFFF;
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let btn = [BTN_LOCK, BTN_UNLOCK, BTN_TRUNK][(seed % 3) as usize];
            let cnt = (seed >> 8) as u16;
            let logical = logical_frame(serial, btn, cnt);
            let sig = MazdaInfinityDecoder::new()
                .encode(&signal_for(&logical), btn)
                .unwrap();
            for inv in [false, true] {
                let s: Vec<LevelDuration> = sig
                    .iter()
                    .map(|p| LevelDuration::new(p.level != inv, p.duration_us))
                    .collect();
                let mut without: Vec<Box<dyn ProtocolDecoder>> = vec![
                    Box::new(MazdaV0Decoder::new()),
                    Box::new(MazdaSiemensDecoder::new()),
                ];
                let mut with: Vec<Box<dyn ProtocolDecoder>> = vec![
                    Box::new(MazdaV0Decoder::new()),
                    Box::new(MazdaSiemensDecoder::new()),
                    Box::new(MazdaInfinityDecoder::new()),
                ];
                let base = first_match(&mut without, &s);
                let all = first_match(&mut with, &s);
                decoded += all.len();
                let others: Vec<&str> =
                    all.into_iter().filter(|n| *n != "Mazda Infinity").collect();
                assert_eq!(others, base, "serial {serial:#X} inv={inv}");
            }
        }
        assert!(decoded > 0, "the frames were actually decoded (by Mazda V0 or Mazda Infinity)");
    }

    /// Mazda Siemens' (Flipper-ARF) encoder emits the same air format, and Mazda Infinity decodes
    /// it to exactly the logical payload Siemens built (button in byte 4, 16-bit counter + 1).
    #[test]
    fn decodes_mazda_siemens_encoder_output() {
        let logical = logical_frame(0x1234_5678, BTN_UNLOCK, 0x0005);
        let sie = MazdaSiemensDecoder::new()
            .encode(&signal_for(&logical), 0x02)
            .unwrap();
        let mut expected = logical;
        let cnt = u16::from_be_bytes([expected[5], expected[6]]).wrapping_add(1);
        expected[5..7].copy_from_slice(&cnt.to_be_bytes());
        expected[7] = checksum(&expected);
        let frames = decode_all(&mut MazdaInfinityDecoder::new(), &sie);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].data, u64::from_be_bytes(expected));
        assert_eq!(frames[0].counter, Some(0x0006));
    }

    /// Content-gate strength once sync + trailer matched: random wire payloads pass the
    /// checksum with p ≈ 1/256 (expected ≈ 3906 of 1M).
    #[test]
    fn content_gate_rate_matches_estimate() {
        let mut seed: u32 = 0x0F1E_2D3C;
        let mut next = || {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            seed
        };
        let mut passed = 0u32;
        for _ in 0..1_000_000 {
            let (a, b) = (next().to_le_bytes(), next().to_le_bytes());
            let wire = [a[0], a[1], a[2], a[3], b[0], b[1], b[2], b[3]];
            let logical = wire_to_logical(&wire);
            if frame_valid(&logical) && logical[7] == checksum(&logical) {
                passed += 1;
            }
        }
        assert!((3500..=4300).contains(&passed), "passed {passed} of 1M");
    }

    /// Random pulse noise (uniform 50..6000µs, and runs snapped to 250/500µs so the cell window
    /// fills with random Manchester) never decodes.
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
            } else {
                (if r & 1 == 0 { TE_SHORT } else { TE_LONG }) + (r >> 8) % 80 - 40
            };
            sig.push(LevelDuration::new(level, d));
            level = !level;
        }
        assert!(decode_all(&mut MazdaInfinityDecoder::new(), &sig).is_empty());
    }

    /// The five Mazda RAW captures (decode as nothing today; they look like FSK fobs recorded
    /// with an OOK preset) must not produce a Mazda Infinity frame, standalone, either polarity.
    #[test]
    fn no_decode_on_mazda_imports() {
        let Ok(dir) = std::fs::read_dir("IMPORTS/Mazda") else {
            eprintln!("Skip: IMPORTS/Mazda not found (run from crate root)");
            return;
        };
        let mut checked = 0;
        for entry in dir.flatten() {
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
                    decode_all(&mut MazdaInfinityDecoder::new(), &sig).is_empty(),
                    "{path:?} inv={inv}"
                );
            }
            checked += 1;
        }
        assert!(
            checked >= 5,
            "expected the Mazda RAW captures, checked {checked}"
        );
    }
}
