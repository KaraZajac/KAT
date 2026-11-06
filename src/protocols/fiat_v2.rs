//! Fiat V2 protocol decoder/encoder (FCA: Fiat / Chrysler / Jeep)
//!
//! Decoder aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/fiat_v2.c` and
//! `fiat_v2.h` (primary; decode-only there). The encoder is doctor-firmware's
//! `lib/subghz/protocols/fiat_v2.c` replay encoder (see Encoder). Flipper-ARF's
//! `lib/subghz/protocols/fiat_v2.c` has the same decoder as ProtoPirate; its encoder was not used
//! (HITAG2 hop recompute with Fiat V1 known keys that KAT does not carry, and its wire polarity is
//! the opposite of what real fobs send). AM (OOK), 315 / 433.92 MHz (C flags 315|433).
//!
//! **Timing**: Manchester, te_short=210µs, te_long=420µs, te_delta=100µs (`DURATION_DIFF < delta`:
//! short 111..=309µs, long 321..=519µs). A LOW ≥900µs is the frame boundary.
//!
//! **Decoder** (ported from the C): every short pulse pushes one half-bit "cell" of its level,
//! every long pulse two; the last 224 cells (112 bits) form a sliding window that is tried after
//! each push, first with bit = first cell of each pair, then inverted. Any other pulse ends the
//! run; a LOW ≥900µs first pushes one LOW cell (the frame's last half-bit merged into the gap).
//! Identical consecutive frames are suppressed until `reset()` (the C's `last_raw_data` dedup).
//!
//! **Frame** (112 bits = 14 bytes, MSB first):
//! - raw[0]=0x00, raw[1]=0x01 markers (15 zero bits + a one: the frame's lead-in)
//! - raw[2..=5] UID (serial), raw[6] high nibble = type (0xD = "FCA" layout)
//! - raw[7] bits 7:6 = button selector (1 Trunk, 2 Lock, 3 Unlock); `button` = raw[7] as in the C
//! - non-FCA: counter = `~((raw[7]&0x3F)<<5 | raw[8]>>3) & 0x7FF`, hop = raw[9..=12]
//! - FCA:     counter = `~(raw[8]<<6 | raw[9]>>2) & 0x3FFF`,       hop = raw[10..=13]
//!
//! **Packing**: `data` = raw[0..8] (first 64 wire bits), `data_count_bit` = 112, `extra` =
//! raw[8..14] (48 bits, right-aligned). `counter` holds the C's 11/14-bit counter.
//!
//! **Validation / gating**: the C's `fiat_v2_frame_valid` (both markers, selector ∈ {1,2,3}, UID ≠
//! 0 / 0xFFFFFFFF) plus one KAT-specific structural gate: the valid window must end at the end of
//! the burst (at most one trailing cell, i.e. doctor-firmware's stop pulse, before the terminating
//! pulse). Reason: the sliding window treats any ≥15-bit Manchester run + a one as the markers, so
//! the C would also match the middle of other protocols' frames — on `IMPORTS/LDV/LDV-T80_unlock`
//! it matches 15 preamble bits + the first 97 bits of a 104-bit Ford V3 frame and, being complete
//! first, would steal that frame from Ford V3 in KAT's first-match registry. Real Fiat V2 bursts
//! carry exactly one frame between long gaps (`IMPORTS/chrysler/19_chrysler_pacifica*`,
//! `IMPORTS/jeep/22_jeep_renegade*`: 47 and 29 frames, all still decode), so the gate costs
//! nothing on genuine captures. doctor-firmware's extra type-nibble exclusions (0x1/0x7/0xA) were
//! NOT adopted: the genuine Jeep Renegade frames have type 0x7. Nor its FCA-layout gate
//! `(raw[7] & 0x3F) == 0` (the counter has left raw[7] there), for which no capture exists.
//!
//! **Encoder** — frame content from Flipper-ARF, waveform from doctor-firmware:
//! - Flipper-ARF (`subghz_protocol_encoder_fiat_v2_deserialize`, "hop recompute"): if one of the
//!   Fiat V1 known HITAG2 keys (ARF's `fiat_v1_known_keys`, 8 keys) with one of the 4 IV
//!   normalisations (`fiat_v2_iv_button` A/B × `fiat_v2_iv_control` 1/2, epoch 0) reproduces the
//!   captured hop, the frame is forward-encoded: the requested button selector (KAT 0x01 Lock → 2,
//!   0x02 Unlock → 3, 0x04 Trunk → 1 — ARF's Down/Up/Left; anything else keeps the captured
//!   selector, ARF's OK), the counter + 1 (ARF's car-emulate scene writes Cnt = counter + 1 per
//!   press), then the hop recomputed with `fiat_v1_compute_auth` (= `hitag2::fiat_bcm_authenticator`)
//!   and patched in; markers, UID and type nibble are kept. After ARF's keys KAT tries the key
//!   saved for the remote (`hitag2_keys.txt`, Fiat V1's byte order, e.g. one "Recover key
//!   (HiTag2)" found; `hitag2_keys.rs`) the same way. Otherwise (no key matches — every frame in
//!   IMPORTS) the 14 captured bytes are replayed unchanged, as ARF and doctor-firmware do.
//! - Waveform: doctor-firmware `fiat_v2_encoder_build_upload`. Per copy: LOW 2000µs, 112 bits
//!   with bit b sent as (!b, b) half-bits of 210µs, a HIGH 210µs stop pulse if the last half-bit
//!   was low, LOW 2000µs; same-level runs merged; 6 copies (doctor's default repeat). That polarity
//!   is the one the real FCA captures above use (they all decode through the C's inverted path);
//!   Flipper-ARF's upload uses the opposite polarity behind a 2033µs carrier lead copied from its
//!   Fiat V1 encoder, so KAT keeps doctor-firmware's (deviation from ARF, from capture evidence).
//!
//! **Key files** (`subghz_protocol_decoder_fiat_v2_serialize` / `_deserialize`): KAT writes
//! ProtoPirate's lines — Protocol "Fiat V2", Bit 112, Key = UID << 32 | hop, then Serial (UID),
//! Btn (the raw button byte; the C writes `generic.btn` and then updates the same line with
//! `button`, the same value), Cnt, Hop (decimal) and Raw (the 14 wire bytes). Loading follows all
//! three references: Bit 112 and a Raw line passing `fiat_v2_frame_valid` are required (no Raw =
//! no key, as in the C), and the decode is the Raw frame's. Flipper-ARF's / doctor-firmware's
//! files (Raw first, ARF's optional Hitag2 lines) load the same way; KAT keeps no key from them
//! (its encoder re-discovers ARF's known keys from the frame itself).

use std::collections::VecDeque;

use super::hitag2;
use super::hitag2_keys::{self, KeyOrder};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`); the IMPORTS captures that decode as Fiat V2 are Chrysler/Jeep.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Fiat V2",
    decoder: "Fiat V2",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "Fiat/Chrysler/Jeep",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// Button selector = top two bits of the raw button byte.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code >> 6 {
        2 => "Lock",
        3 => "Unlock",
        1 => "Trunk",
        _ => "Other",
    })
}

const TE_SHORT: u32 = 210;
const TE_LONG: u32 = 420;
const TE_DELTA: u32 = 100;
const BOUNDARY_MIN_US: u32 = 900;
const WIRE_BITS: usize = 112;
const WIRE_BYTES: usize = 14;
const WIRE_CELLS: usize = WIRE_BITS * 2;
const MARKER0: u8 = 0x00;
const MARKER1: u8 = 0x01;
const BTN_SHIFT: u8 = 6;
const SEL_TRUNK: u8 = 0x1;
const SEL_LOCK: u8 = 0x2;
const SEL_UNLOCK: u8 = 0x3;
const FCA_TYPE_NIBBLE: u8 = 0xD0;
/// KAT gate: cells allowed between the end of the valid window and the end of the burst
/// (doctor-firmware's encoder appends a one-cell stop pulse after a frame ending low).
const MAX_TRAILING_CELLS: usize = 1;

// Encoder (doctor-firmware fiat_v2.c).
const ENC_GAP_US: u32 = 2000;
const ENC_REPEAT: usize = 6;

/// Flipper-ARF `fiat_v1_known_keys` (`FIAT_V1_KNOWN_KEY_COUNT` = 8: ProtoPirate's Fiat V1 list
/// without its 9th key), the keys ARF's Fiat V2 encoder tries in this order.
const ARF_KNOWN_KEYS: [[u8; 6]; 8] = [
    [0xB7, 0x92, 0x80, 0xAE, 0xCC, 0x37],
    [0xD4, 0x24, 0x28, 0xF7, 0xD9, 0x66],
    [0x4D, 0x34, 0x3F, 0xD4, 0xE7, 0xB6],
    [0x6D, 0x6B, 0xF2, 0x1D, 0x3A, 0x1A],
    [0xA3, 0xF3, 0xAC, 0xF7, 0xB9, 0x10],
    [0x4D, 0x49, 0x4B, 0x52, 0x4F, 0x4E],
    [0xCD, 0x49, 0x4B, 0x52, 0x4F, 0x4E],
    [0x33, 0xFA, 0x2F, 0xCD, 0xC3, 0x3B],
];
/// `FIAT_V2_IV_COMBO_COUNT`.
const IV_COMBO_COUNT: u8 = 4;

#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Data,
}

/// Fiat V2 decoder (matches SubGhzProtocolDecoderFiatV2, plus the end-of-burst gate).
pub struct FiatV2Decoder {
    step: DecoderStep,
    cells: VecDeque<bool>,
    /// Most recent valid window and how many cells were pushed after it.
    pending: Option<[u8; WIRE_BYTES]>,
    cells_since_pending: usize,
    last_raw: Option<[u8; WIRE_BYTES]>,
}

impl FiatV2Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            cells: VecDeque::with_capacity(WIRE_CELLS + 1),
            pending: None,
            cells_since_pending: 0,
            last_raw: None,
        }
    }

    fn is_short(duration: u32) -> bool {
        duration_diff!(duration, TE_SHORT) < TE_DELTA
    }

    fn is_long(duration: u32) -> bool {
        duration_diff!(duration, TE_LONG) < TE_DELTA
    }

    fn uid(raw: &[u8; WIRE_BYTES]) -> u32 {
        u32::from_be_bytes([raw[2], raw[3], raw[4], raw[5]])
    }

    fn is_fca(raw: &[u8; WIRE_BYTES]) -> bool {
        raw[6] & 0xF0 == FCA_TYPE_NIBBLE
    }

    /// fiat_v2_hop: raw[10..=13] for FCA frames, raw[9..=12] otherwise.
    fn hop(raw: &[u8; WIRE_BYTES]) -> u32 {
        let start = if Self::is_fca(raw) { 10 } else { 9 };
        u32::from_be_bytes([raw[start], raw[start + 1], raw[start + 2], raw[start + 3]])
    }

    /// fiat_v2_counter: 14-bit (FCA) or 11-bit counter, stored bit-inverted on the wire.
    fn counter(raw: &[u8; WIRE_BYTES]) -> u16 {
        if Self::is_fca(raw) {
            let raw_cnt = (u32::from(raw[8]) << 6) | u32::from(raw[9] >> 2);
            (!raw_cnt & 0x3FFF) as u16
        } else {
            let raw_cnt = (u32::from(raw[7] & 0x3F) << 5) | u32::from(raw[8] >> 3);
            (!raw_cnt & 0x7FF) as u16
        }
    }

    /// fiat_v2_iv_button (ARF): combo bit 0 = the raw selector, or its one-hot remap.
    fn iv_button(raw: &[u8; WIRE_BYTES], combo: u8) -> u8 {
        let sel = (raw[7] >> BTN_SHIFT) & 0x0F;
        if combo & 0x01 == 0 {
            return sel;
        }
        match sel {
            SEL_TRUNK => 0x2,
            SEL_LOCK => 0x4,
            SEL_UNLOCK => 0x8,
            _ => 0x0,
        }
    }

    /// fiat_v2_iv_control (ARF): combo bit 1 = the 10-bit counter, or its complement.
    fn iv_control(raw: &[u8; WIRE_BYTES], combo: u8) -> u16 {
        let cnt = Self::counter(raw) & 0x3FF;
        if combo & 0x02 == 0 {
            cnt
        } else {
            !cnt & 0x3FF
        }
    }

    fn compute_hop(raw: &[u8; WIRE_BYTES], key: &[u8; 6], combo: u8) -> u32 {
        let (btn, ctrl) = (Self::iv_button(raw, combo), Self::iv_control(raw, combo));
        hitag2::fiat_bcm_authenticator(Self::uid(raw), btn, ctrl, key, 0)
    }

    /// ARF's key auto-discovery: the first known key × IV combo (key-major) that reproduces the
    /// captured hop (`fiat_v2_key_matches_combo`).
    fn find_known_key(raw: &[u8; WIRE_BYTES]) -> Option<(&'static [u8; 6], u8)> {
        let hop = Self::hop(raw);
        ARF_KNOWN_KEYS.iter().find_map(|key| {
            (0..IV_COMBO_COUNT)
                .find(|&combo| Self::compute_hop(raw, key, combo) == hop)
                .map(|combo| (key, combo))
        })
    }

    /// The key `encode()` uses: ARF's known keys ([`Self::find_known_key`]), then the key saved
    /// for the remote (`hitag2_keys.txt`, e.g. by "Recover key (HiTag2)"; Fiat V1's byte order),
    /// with the first IV combo that reproduces the captured hop.
    fn find_key(raw: &[u8; WIRE_BYTES]) -> Option<([u8; 6], u8)> {
        if let Some((key, combo)) = Self::find_known_key(raw) {
            return Some((*key, combo));
        }
        let hop = Self::hop(raw);
        hitag2_keys::keys_for(Self::uid(raw), KeyOrder::FiatBcm).into_iter().find_map(|(key, _)| {
            (0..IV_COMBO_COUNT).find(|&combo| Self::compute_hop(raw, &key, combo) == hop).map(|combo| (key, combo))
        })
    }

    /// Whether `encode()` sends fresh frames of this decode (a key verifies) instead of replaying
    /// it: "Recover key (HiTag2)" has nothing left to find.
    pub(super) fn has_key(decoded: &DecodedSignal) -> bool {
        Self::raw_from_signal(decoded).is_some_and(|raw| Self::frame_valid(&raw) && Self::find_key(&raw).is_some())
    }

    /// fiat_v2_patch_button: selector into raw[7] bits 7:6.
    fn patch_button(raw: &mut [u8; WIRE_BYTES], selector: u8) {
        raw[7] = (raw[7] & 0x3F) | ((selector & 0x03) << BTN_SHIFT);
    }

    /// fiat_v2_patch_counter: the inverse of `counter` for both layouts.
    fn patch_counter(raw: &mut [u8; WIRE_BYTES], counter: u32) {
        if Self::is_fca(raw) {
            let raw_cnt = !counter & 0x3FFF;
            raw[8] = (raw_cnt >> 6) as u8;
            raw[9] = (raw[9] & 0x03) | (((raw_cnt & 0x3F) as u8) << 2);
        } else {
            let raw_cnt = !counter & 0x7FF;
            raw[7] = (raw[7] & 0xC0) | ((raw_cnt >> 5) as u8 & 0x3F);
            raw[8] = (raw[8] & 0x07) | (((raw_cnt & 0x1F) as u8) << 3);
        }
    }

    /// fiat_v2_patch_hop: raw[10..=13] (FCA) or raw[9..=12].
    fn patch_hop(raw: &mut [u8; WIRE_BYTES], hop: u32) {
        let start = if Self::is_fca(raw) { 10 } else { 9 };
        raw[start..start + 4].copy_from_slice(&hop.to_be_bytes());
    }

    /// fiat_v2_frame_valid: markers, button selector, UID not all-0 / all-1.
    fn frame_valid(raw: &[u8; WIRE_BYTES]) -> bool {
        if raw[0] != MARKER0 || raw[1] != MARKER1 {
            return false;
        }
        if !matches!(raw[7] >> BTN_SHIFT, SEL_TRUNK | SEL_LOCK | SEL_UNLOCK) {
            return false;
        }
        let uid = Self::uid(raw);
        uid != 0 && uid != u32::MAX
    }

    /// fiat_v2_try_decode: the 224-cell window, normal polarity first, then inverted.
    fn try_decode(&self) -> Option<[u8; WIRE_BYTES]> {
        if self.cells.len() != WIRE_CELLS {
            return None;
        }
        let mut raw = [0u8; WIRE_BYTES];
        for bit_index in 0..WIRE_BITS {
            let first = self.cells[bit_index * 2];
            if first == self.cells[bit_index * 2 + 1] {
                return None;
            }
            if first {
                raw[bit_index >> 3] |= 1 << (7 - (bit_index & 7));
            }
        }
        if Self::frame_valid(&raw) {
            return Some(raw);
        }
        let inverted = raw.map(|b| !b);
        Self::frame_valid(&inverted).then_some(inverted)
    }

    /// fiat_v2_push_cell (sliding window) + the end-of-burst bookkeeping.
    fn push_cell(&mut self, level: bool) {
        self.cells.push_back(level);
        if self.cells.len() > WIRE_CELLS {
            self.cells.pop_front();
        }
        if let Some(raw) = self.try_decode() {
            self.pending = Some(raw);
            self.cells_since_pending = 0;
        } else if self.pending.is_some() {
            self.cells_since_pending += 1;
            if self.cells_since_pending > MAX_TRAILING_CELLS {
                self.pending = None;
            }
        }
    }

    fn clear_cells(&mut self) {
        self.cells.clear();
        self.pending = None;
        self.cells_since_pending = 0;
    }

    /// fiat_v2_feed_data_pulse. Returns `false` when the pulse ended the run; the frame found at
    /// the end of the run (if any) is left in `hit`.
    fn feed_data_pulse(
        &mut self,
        level: bool,
        duration: u32,
        hit: &mut Option<[u8; WIRE_BYTES]>,
    ) -> bool {
        if Self::is_short(duration) {
            self.push_cell(level);
            return true;
        }
        if Self::is_long(duration) {
            self.push_cell(level);
            self.push_cell(level);
            return true;
        }
        // End of the burst: a window that ended here (or one cell earlier) is the frame.
        *hit = self.pending.take();
        if hit.is_none() && !level && duration >= BOUNDARY_MIN_US {
            // The frame's last half-bit (LOW) merged into the boundary gap.
            self.push_cell(false);
            *hit = self.pending.take();
        }
        self.clear_cells();
        false
    }

    /// fiat_v2_commit: dedup against the previous frame, then build the signal.
    fn commit(&mut self, raw: [u8; WIRE_BYTES]) -> Option<DecodedSignal> {
        if self.last_raw == Some(raw) {
            return None;
        }
        self.last_raw = Some(raw);
        Some(Self::build_signal(&raw))
    }

    fn build_signal(raw: &[u8; WIRE_BYTES]) -> DecodedSignal {
        let mut data_bytes = [0u8; 8];
        data_bytes.copy_from_slice(&raw[..8]);
        let mut extra_bytes = [0u8; 8];
        extra_bytes[2..].copy_from_slice(&raw[8..]);
        DecodedSignal {
            serial: Some(Self::uid(raw)),
            button: Some(raw[7]),
            counter: Some(Self::counter(raw)),
            // No CRC in this frame; emission is gated on markers / selector / UID / structure.
            crc_valid: false,
            data: u64::from_be_bytes(data_bytes),
            data_count_bit: WIRE_BITS,
            encoder_capable: true,
            extra: Some(u64::from_be_bytes(extra_bytes)),
            protocol_display_name: None,
        }
    }

    /// Rebuild the 14 wire bytes from `data` (raw[0..8]) and `extra` (raw[8..14]).
    fn raw_from_signal(decoded: &DecodedSignal) -> Option<[u8; WIRE_BYTES]> {
        if decoded.data_count_bit != WIRE_BITS {
            return None;
        }
        let extra = decoded.extra?;
        if extra >> 48 != 0 {
            return None;
        }
        let mut raw = [0u8; WIRE_BYTES];
        raw[..8].copy_from_slice(&decoded.data.to_be_bytes());
        raw[8..].copy_from_slice(&extra.to_be_bytes()[2..]);
        Some(raw)
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

    /// doctor-firmware fiat_v2_encoder_build_upload (one copy).
    fn append_frame(signal: &mut Vec<LevelDuration>, raw: &[u8; WIRE_BYTES]) {
        Self::enc_add_level(signal, false, ENC_GAP_US);
        let mut last_high = false;
        for bit_index in 0..WIRE_BITS {
            let bit = (raw[bit_index >> 3] >> (7 - (bit_index & 7))) & 1 != 0;
            let first_high = !bit;
            Self::enc_add_level(signal, first_high, TE_SHORT);
            Self::enc_add_level(signal, !first_high, TE_SHORT);
            last_high = !first_high;
        }
        if !last_high {
            Self::enc_add_level(signal, true, TE_SHORT);
        }
        Self::enc_add_level(signal, false, ENC_GAP_US);
    }
}

impl ProtocolDecoder for FiatV2Decoder {
    fn name(&self) -> &'static str {
        "Fiat V2"
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
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.clear_cells();
        self.last_raw = None;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let mut hit = None;
        match self.step {
            DecoderStep::Reset => {
                if Self::is_short(duration) || Self::is_long(duration) {
                    self.clear_cells();
                    self.step = DecoderStep::Data;
                    self.feed_data_pulse(level, duration, &mut hit);
                }
            }
            DecoderStep::Data => {
                if !self.feed_data_pulse(level, duration, &mut hit) {
                    self.step = DecoderStep::Reset;
                }
            }
        }
        hit.and_then(|raw| self.commit(raw))
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        let mut raw = Self::raw_from_signal(decoded)?;
        if !Self::frame_valid(&raw) {
            return None;
        }
        // Flipper-ARF hop recompute when a known or saved key reproduces the captured hop;
        // otherwise the captured frame is replayed unchanged.
        if let Some((key, combo)) = Self::find_key(&raw) {
            let selector = match button {
                0x01 => SEL_LOCK,
                0x02 => SEL_UNLOCK,
                0x04 => SEL_TRUNK,
                _ => (raw[7] >> BTN_SHIFT) & 0x03,
            };
            let counter = u32::from(decoded.counter.unwrap_or_else(|| Self::counter(&raw))) + 1;
            Self::patch_button(&mut raw, selector);
            Self::patch_counter(&mut raw, counter);
            let hop = Self::compute_hop(&raw, &key, combo);
            Self::patch_hop(&mut raw, hop);
        }
        let mut signal = Vec::with_capacity(ENC_REPEAT * (WIRE_BITS * 2 + 3));
        for _ in 0..ENC_REPEAT {
            Self::append_frame(&mut signal, &raw);
        }
        Some(signal)
    }

    /// `subghz_protocol_decoder_fiat_v2_deserialize`: Bit 112 and a valid Raw frame.
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != WIRE_BITS {
            return None;
        }
        let raw = <[u8; WIRE_BYTES]>::try_from(key.hex_bytes_field("Raw")?).ok()?;
        Self::frame_valid(&raw).then(|| Self::build_signal(&raw))
    }

    /// `subghz_protocol_decoder_fiat_v2_serialize` (see the module doc).
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        let raw = Self::raw_from_signal(decoded)?;
        if !Self::frame_valid(&raw) {
            return None;
        }
        let (uid, hop) = (Self::uid(&raw), Self::hop(&raw));
        let mut key = FlipperKey::for_protocol("Fiat V2", WIRE_BITS, (u64::from(uid) << 32) | u64::from(hop));
        key.push_u32("Serial", uid);
        key.push_u32("Btn", raw[7].into());
        key.push_u32("Cnt", Self::counter(&raw).into());
        key.push_u32("Hop", hop);
        key.push_hex_bytes("Raw", &raw);
        Some(key)
    }
}

impl Default for FiatV2Decoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::export::flipper::import_sub_raw;
    use crate::protocols::ProtocolRegistry;
    use std::path::Path;

    /// A frame in the real (Chrysler Pacifica) layout: type 5, selector 3 (Unlock), ends in bit 1.
    const RAW_PACIFICA: [u8; WIRE_BYTES] = [
        0x00, 0x01, 0xD7, 0x37, 0xA2, 0xC3, 0x5E, 0xFB, 0x20, 0x23, 0x5C, 0x16, 0x7D, 0xE9,
    ];

    /// Physical half-bit cells of a frame as real fobs send it: bit b -> (!b, b).
    fn frame_cells(raw: &[u8; WIRE_BYTES]) -> Vec<bool> {
        (0..WIRE_BITS)
            .flat_map(|i| {
                let bit = (raw[i >> 3] >> (7 - (i & 7))) & 1 != 0;
                [!bit, bit]
            })
            .collect()
    }

    /// Cells -> merged level/duration pulses (1 cell = te_short).
    fn cells_to_pulses(cells: &[bool]) -> Vec<LevelDuration> {
        let mut out: Vec<LevelDuration> = Vec::new();
        for &c in cells {
            FiatV2Decoder::enc_add_level(&mut out, c, TE_SHORT);
        }
        out
    }

    fn decode(pulses: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut decoder = FiatV2Decoder::new();
        pulses
            .iter()
            .filter_map(|p| decoder.feed(p.level, p.duration_us))
            .collect()
    }

    /// Burst framed by 20 ms LOW gaps (a trailing LOW half-bit merges into the closing gap).
    fn gapped(cells: &[bool]) -> Vec<LevelDuration> {
        let mut pulses = vec![LevelDuration::new(false, 20_000)];
        for p in cells_to_pulses(cells) {
            FiatV2Decoder::enc_add_level(&mut pulses, p.level, p.duration_us);
        }
        FiatV2Decoder::enc_add_level(&mut pulses, false, 20_000);
        pulses
    }

    #[test]
    fn synthetic_frame_decodes_with_fields() {
        let decoded = decode(&gapped(&frame_cells(&RAW_PACIFICA)));
        assert_eq!(decoded.len(), 1);
        let d = &decoded[0];
        assert_eq!(d.serial, Some(0xD737_A2C3));
        assert_eq!(d.button, Some(0xFB));
        assert_eq!(d.button.map(|b| b >> BTN_SHIFT), Some(SEL_UNLOCK));
        assert_eq!(d.counter, Some(0x09B)); // ~((0x3B << 5) | (0x20 >> 3)) & 0x7FF
        assert_eq!(d.data, 0x0001_D737_A2C3_5EFB);
        assert_eq!(d.extra, Some(0x2023_5C16_7DE9));
        assert_eq!(d.data_count_bit, 112);
        assert_eq!(FiatV2Decoder::hop(&RAW_PACIFICA), 0x235C_167D);
    }

    #[test]
    fn fca_layout_fields() {
        let mut raw = RAW_PACIFICA;
        raw[6] = 0xD2; // FCA type nibble
        assert!(FiatV2Decoder::is_fca(&raw));
        assert_eq!(FiatV2Decoder::hop(&raw), 0x5C16_7DE9);
        assert_eq!(
            FiatV2Decoder::counter(&raw),
            (!((0x20u32 << 6) | (0x23 >> 2)) & 0x3FFF) as u16
        );
    }

    #[test]
    fn decodes_either_polarity_and_stretched_first_pulse() {
        // Normal-polarity window (bit = first cell).
        let normal: Vec<bool> = frame_cells(&RAW_PACIFICA).iter().map(|c| !c).collect();
        let mut pulses = vec![LevelDuration::new(true, 20_000)];
        pulses.extend(cells_to_pulses(&normal));
        pulses.push(LevelDuration::new(true, 20_000));
        assert_eq!(decode(&pulses).len(), 1);
        // Real captures sometimes stretch the first HIGH to a long (one extra cell, 225 total):
        // the sliding window still lands on the frame.
        let mut cells = vec![true];
        cells.extend(frame_cells(&RAW_PACIFICA));
        let decoded = decode(&gapped(&cells));
        assert_eq!(decoded.len(), 1);
        assert_eq!(decoded[0].serial, Some(0xD737_A2C3));
    }

    #[test]
    fn frame_ending_low_merged_into_gap_decodes() {
        // Last bit 0 -> last half-bit LOW, swallowed by the boundary gap (C boundary push).
        let mut raw = RAW_PACIFICA;
        raw[13] = 0xE8;
        let decoded = decode(&gapped(&frame_cells(&raw)));
        assert_eq!(decoded.len(), 1);
        assert_eq!(decoded[0].extra.map(|e| e as u8), Some(0xE8));
    }

    #[test]
    fn encode_decode_roundtrip() {
        for last in [0xE9u8, 0xE8] {
            let mut raw = RAW_PACIFICA;
            raw[13] = last; // ends high, and ends low (doctor's stop pulse is emitted)
            let original = FiatV2Decoder::build_signal(&raw);
            let signal = FiatV2Decoder::new().encode(&original, 0x01).unwrap();
            // Six copies, each a separate decode once the decoder is reset (registry behaviour).
            let mut decoder = FiatV2Decoder::new();
            let mut hits = Vec::new();
            for p in &signal {
                if let Some(d) = decoder.feed(p.level, p.duration_us) {
                    decoder.reset();
                    hits.push(d);
                }
            }
            assert_eq!(hits.len(), ENC_REPEAT, "last byte {last:#x}");
            for d in &hits {
                assert_eq!(d.data, original.data);
                assert_eq!(d.extra, original.extra);
                assert_eq!(d.button, original.button, "button is never re-stamped");
            }
            // Without a reset the C's dedup reports the repeated frame once.
            assert_eq!(decode(&signal).len(), 1);
        }
    }

    /// Oracle vectors (see `fiat_v0::write_oracle_vector`): the Pacifica frame (doctor-firmware
    /// replay upload) and its FCA-layout variant.
    #[test]
    #[ignore]
    fn oracle_vectors() {
        let mut fca = RAW_PACIFICA;
        fca[6] = 0xD2;
        fca[7] &= 0xC0; // doctor-firmware's FCA gate: the counter has left raw[7]
        for (name, raw) in [("fiat_v2_1", RAW_PACIFICA), ("fiat_v2_2", fca)] {
            let up = FiatV2Decoder::new().encode(&FiatV2Decoder::build_signal(&raw), 0x40).unwrap();
            crate::protocols::fiat_v0::write_oracle_vector(
                name,
                433_920_000,
                "FuriHalSubGhzPresetOok650Async",
                &up,
                &mut FiatV2Decoder::new(),
            );
        }
    }

    /// Key files: ProtoPirate's lines round-trip (both layouts), a Flipper-ARF file (Raw first)
    /// loads, a key without Raw or with an invalid Raw does not (as in the C), and a loaded key
    /// replays a frame that decodes back.
    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::{assert_round_trip, key};
        let dec = FiatV2Decoder::new();
        let decoded = decode(&gapped(&frame_cells(&RAW_PACIFICA))).remove(0);
        let text = assert_round_trip(&dec, &decoded);
        assert!(
            text.ends_with(
                "Protocol: Fiat V2\nBit: 112\nKey: D7 37 A2 C3 23 5C 16 7D\nSerial: 3610747587\n\
                 Btn: 251\nCnt: 155\nHop: 593237629\nRaw: 00 01 D7 37 A2 C3 5E FB 20 23 5C 16 7D E9\n"
            ),
            "{text}"
        );
        let mut fca = RAW_PACIFICA;
        fca[6] = 0xD2;
        assert_round_trip(&dec, &FiatV2Decoder::build_signal(&fca));

        let arf = key(
            "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
             Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Fiat V2\nBit: 112\n\
             Key: D7 37 A2 C3 23 5C 16 7D\nRaw: 00 01 D7 37 A2 C3 5E FB 20 23 5C 16 7D E9\n\
             Hop: 593237629\nBtn: 251\nSerial: 3610747587\nCnt: 155\n",
        );
        assert_eq!(dec.from_flipper_key(&arf), Some(decoded.clone()));
        let mut no_raw = arf.clone();
        no_raw.fields.retain(|(n, _)| n != "Raw");
        assert!(dec.from_flipper_key(&no_raw).is_none());
        let mut bad = no_raw.clone();
        bad.push("Raw", "00 03 D7 37 A2 C3 5E FB 20 23 5C 16 7D E9");
        assert!(dec.from_flipper_key(&bad).is_none(), "marker byte");
        let mut bits = arf.clone();
        bits.bits = 104;
        assert!(dec.from_flipper_key(&bits).is_none());

        let loaded = dec.from_flipper_key(&arf).unwrap();
        let tx = decode(&dec.encode(&loaded, 0x01).unwrap());
        assert_eq!(tx, vec![loaded], "unknown key: the frame is replayed");
    }

    /// Flipper-ARF hop recompute: a frame whose hop one of the known keys reproduces (any of the
    /// 4 IV combos, both layouts) is forward-encoded with the requested selector, counter + 1 and
    /// the hop recomputed with the same key/combo; other fields are kept.
    #[test]
    fn known_key_frames_are_forward_encoded() {
        for (fca, key_idx, combo) in [(true, 0usize, 0u8), (true, 3, 1), (false, 5, 2), (false, 7, 3)] {
            let mut raw = RAW_PACIFICA;
            if !fca {
                raw[6] = 0x7A; // non-FCA layout (type nibble 0x7, as on the Jeep Renegade)
            }
            let key = &ARF_KNOWN_KEYS[key_idx];
            let hop = FiatV2Decoder::compute_hop(&raw, key, combo);
            FiatV2Decoder::patch_hop(&mut raw, hop);
            let original = FiatV2Decoder::build_signal(&raw);
            assert_eq!(FiatV2Decoder::find_known_key(&raw).map(|(_, c)| c), Some(combo));
            for (button, selector) in [(0x01u8, SEL_LOCK), (0x02, SEL_UNLOCK), (0x04, SEL_TRUNK), (0x08, raw[7] >> 6)] {
                let signal = FiatV2Decoder::new().encode(&original, button).unwrap();
                let out = decode(&signal);
                assert_eq!(out.len(), 1);
                let got = FiatV2Decoder::raw_from_signal(&out[0]).unwrap();
                assert_eq!(got[7] >> 6, selector, "button {button:#04x}");
                assert_eq!(FiatV2Decoder::counter(&got), original.counter.unwrap() + 1);
                assert_eq!(FiatV2Decoder::hop(&got), FiatV2Decoder::compute_hop(&got, key, combo));
                assert_eq!(got[..7], raw[..7], "markers, UID and type kept");
            }
        }
    }

    /// A key saved for the remote (`hitag2_keys.txt`, Fiat V1's byte order, e.g. by "Recover key
    /// (HiTag2)") is used like ARF's known keys: fresh frames instead of the replay.
    #[test]
    fn saved_key_frames_are_forward_encoded() {
        // A UID no other test uses: the key database is global.
        let uid = 0x5EED_0F2Au32;
        let key = [0x54, 0x31, 0x2E, 0x38, 0x4B, 0x41];
        let mut raw = RAW_PACIFICA;
        raw[2..6].copy_from_slice(&uid.to_be_bytes());
        let hop = FiatV2Decoder::compute_hop(&raw, &key, 1);
        FiatV2Decoder::patch_hop(&mut raw, hop);
        let original = FiatV2Decoder::build_signal(&raw);
        assert_eq!(FiatV2Decoder::find_key(&raw), None);
        assert!(!FiatV2Decoder::has_key(&original));
        hitag2_keys::tests::remember_for_test(uid, key);
        assert_eq!(FiatV2Decoder::find_key(&raw), Some((key, 1)));
        assert!(FiatV2Decoder::has_key(&original));
        let out = decode(&FiatV2Decoder::new().encode(&original, 0x02).unwrap());
        let got = FiatV2Decoder::raw_from_signal(&out[0]).unwrap();
        assert_eq!((got[7] >> 6, FiatV2Decoder::counter(&got)), (SEL_UNLOCK, original.counter.unwrap() + 1));
        assert_eq!(FiatV2Decoder::hop(&got), FiatV2Decoder::compute_hop(&got, &key, 1));
    }

    #[test]
    fn gate_rejects_corrupted_frames() {
        let mut bad_marker = RAW_PACIFICA;
        bad_marker[1] = 0x03;
        let mut zero_uid = RAW_PACIFICA;
        zero_uid[2..6].copy_from_slice(&[0, 0, 0, 0]);
        let mut ones_uid = RAW_PACIFICA;
        ones_uid[2..6].copy_from_slice(&[0xFF; 4]);
        let mut no_button = RAW_PACIFICA;
        no_button[7] &= 0x3F;
        for raw in [bad_marker, zero_uid, ones_uid, no_button] {
            assert!(!FiatV2Decoder::frame_valid(&raw));
            assert!(decode(&gapped(&frame_cells(&raw))).is_empty(), "{raw:02X?}");
        }
        // A broken Manchester pair inside the window.
        let mut cells = frame_cells(&RAW_PACIFICA);
        cells[101] = cells[100];
        assert!(decode(&gapped(&cells)).is_empty());
    }

    #[test]
    fn gate_rejects_window_inside_a_longer_burst() {
        // The C would accept this window; KAT requires it to end the burst (LDV / Ford V3 steal).
        let mut cells = frame_cells(&RAW_PACIFICA);
        for _ in 0..8 {
            cells.extend([true, false]);
        }
        assert!(decode(&gapped(&cells)).is_empty());
        // One trailing cell (doctor's HIGH stop pulse after a frame ending low) is tolerated...
        let mut raw = RAW_PACIFICA;
        raw[13] = 0xE8;
        let mut one = frame_cells(&raw);
        one.push(true);
        assert_eq!(decode(&gapped(&one)).len(), 1);
        // ...three are not.
        let mut three = frame_cells(&raw);
        three.extend([true, false, true]);
        assert!(decode(&gapped(&three)).is_empty());
    }

    fn registry_decodes(rel: &str) -> Option<Vec<(String, DecodedSignal)>> {
        let path = Path::new("IMPORTS").join(rel);
        if !path.exists() {
            eprintln!("Skip: {:?} not found (run from crate root)", path);
            return None;
        }
        let (freq, raw_pairs) = import_sub_raw(&path).unwrap();
        let pairs: Vec<LevelDuration> = raw_pairs
            .iter()
            .map(|p| LevelDuration::new(p.level, p.duration_us))
            .collect();
        let mut reg = ProtocolRegistry::new();
        Some(
            reg.process_signal_stream(&pairs, freq)
                .into_iter()
                .map(|(n, d, _)| (n, d))
                .collect(),
        )
    }

    #[test]
    fn decodes_chrysler_pacifica_and_jeep_renegade_subs() {
        // Genuine FCA keyfob captures: one 112-bit frame per burst, stable UID, selector following
        // the recorded button sequence (Unlock=3, Lock=2, Trunk=1).
        for (rel, uid) in [
            (
                "chrysler/19_chrysler_pacifica_ul2_l2_t2_rs2_sd2_sd2_p2_t2.sub",
                0xD737_A2C3u32,
            ),
            ("jeep/22_jeep_renegade_ul2_l2_rs2_p2.sub", 0xD08B_E613),
        ] {
            let Some(results) = registry_decodes(rel) else {
                continue;
            };
            let fiat: Vec<_> = results.iter().filter(|(n, _)| n == "Fiat V2").collect();
            assert!(
                !fiat.is_empty(),
                "{rel}: {:?}",
                results.iter().map(|(n, _)| n).collect::<Vec<_>>()
            );
            assert_eq!(fiat.len(), results.len(), "{rel}: only Fiat V2 expected");
            assert!(
                fiat.iter().all(|(_, d)| d.serial == Some(uid)),
                "{rel}: stable UID"
            );
            let first = fiat[0].1.button.unwrap() >> BTN_SHIFT;
            assert_eq!(first, SEL_UNLOCK, "{rel}: recording starts with Unlock");
        }
    }

    #[test]
    fn does_not_steal_ldv_ford_v3_frames() {
        for rel in ["LDV/LDV-T80_lock.sub", "LDV/LDV-T80_unlock.sub"] {
            let Some(results) = registry_decodes(rel) else {
                continue;
            };
            let names: Vec<&str> = results.iter().map(|(n, _)| n.as_str()).collect();
            assert!(!names.contains(&"Fiat V2"), "{rel}: {names:?}");
            assert!(names.contains(&"Ford V3"), "{rel}: {names:?}");
        }
    }
}
