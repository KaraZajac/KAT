//! Mercedes V0 protocol decoder/encoder
//!
//! Aligned with Flipper-ARF reference: `REFERENCES/Flipper-ARF/lib/subghz/protocols/mercedes_v0.c`
//! (`SUBGHZ_PROTOCOL_MERCEDES_V0_NAME = "Mercedes V0"`). A near-identical copy lives in
//! `REFERENCES/doctor-firmware/lib/subghz/protocols/mercedes_v0.c`; the decoder, frame validation
//! and burst builder are byte-identical between the two. They differ only in the custom-button UI
//! glue: ARF maps the d-pad inline (UP=Lock 0xC, DOWN=Unlock 0x8, LEFT=Aux 0xA, OK=replay the
//! captured command, `set_max(3)`), while doctor moves the map into `custom_btn.c` (OK=Unlock,
//! UP=Lock, DOWN=0xA — no "replay original" option). This port follows the **ARF** copy: it is
//! self-contained and its "OK = original command" path is exactly KAT's "unknown button → keep the
//! captured button" rule.
//!
//! ARF's disabled `mercedes.c` (Pandora-firmware RE) describes the same wire format (3700µs lead,
//! 55 preamble pairs, 112 Manchester bits, bit1 = LOW→HIGH). It is not ported separately. Its
//! "shared Pandora struct" field layout (b0..b3 UID, b4 button, b5..b6 counter, b7 sum) contradicts
//! V0's top-nibble command, so no serial/counter is extracted (the C sets `serial = 0`,
//! `cnt_is_available = false`).
//!
//! # Wire format (FM, 433 MHz, Manchester, TE = 500µs)
//! - `3700µs` LOW lead (TX start), then 55 × (HIGH 500 / LOW 500) preamble pairs.
//! - Sync: an extra LOW 500 (merges with the last preamble LOW into a 1000µs LOW) + HIGH 500.
//! - 112 data bits, MSB first, each `(!bit, bit)` for 500µs: bit 1 = LOW→HIGH, bit 0 = HIGH→LOW.
//!
//! # Decoder (matches `mercedes_v0_feed`)
//! Pulses are classified as chips: 350–650µs = 1 chip, 850–1150µs = 2 chips, anything else resets
//! (two consecutive pulses of the same level also reset). Before activation, single-chip pulses
//! count `preamble_edges` (saturating at 114); a 2-chip pulse activates the decoder only if the count
//! is in the edge window **108..=113** (the encoder produces 109). Its first chip ends the preamble
//! and its second opens the sync pair; that first completed chip pair is the sync bit and sets
//! `invert = !bit`, so either polarity decodes the same frame. Each further chip pair must be of opposite levels and yields
//! `bit = !first && second` (LOW→HIGH = 1), XORed with `invert`. Bits are stored MSB first: bits
//! 0..63 → `hi`, bits 64..111 → `lo` bits 63..16 (`lo & 0xFFFF == 0`).
//!
//! # Frame / fields (112 bits)
//! - bits 111..108 (`hi >> 60`): command nibble = `button` — 0xC Lock, 0x8 Unlock, 0xA "??"/Aux,
//!   0x0 Follow-up, 0x4 (whitelisted, unnamed / "Raw").
//! - remaining 108 bits: opaque rolling payload (replayed as-is; no key, no CRC).
//!
//! `DecodedSignal` packing: `data` = `hi` (first 64 wire bits), `data_count_bit` = 112,
//! `extra` = `lo >> 16` (the last 48 wire bits, right-aligned). `serial`/`counter` = `None`.
//!
//! # Gating
//! The C emits when the command nibble is in the PIC receive classifier's whitelist
//! `{0xC, 0x8, 0x0, 0xA, 0x4}` after the strict structure above (108–113 preamble edges, 2-chip
//! sync, exactly 112 clean Manchester pairs with alternating levels). KAT additionally rejects a
//! degenerate payload whose 108 bits after the command nibble are all identical (a run of preamble
//! pairs decoded as data; e.g. all-zero data would otherwise pass with command 0x0).
//! `crc_valid` = true means "passed the whitelist/structure gate" (the protocol has no CRC).
//!
//! # Encoder (matches `mercedes_v0_encoder_upload` / `_burst`)
//! Byte-exact replay of the captured 112 bits with only the command nibble re-stamped:
//! KAT Lock (0x01) → 0xC, Unlock (0x02) → 0x8, Trunk (0x04) → 0xA (the C's "Auxiliary"/"??" — the
//! code's real function is unverified); any other code keeps the captured command. Burst = 3700µs
//! LOW + 55 preamble pairs + sync + 112 Manchester bits. When Unlock is explicitly selected the C
//! appends a 300ms LOW gap and a second burst with command 0x0 (the "Follow-up" frame); KAT does
//! the same. No counter bump (rolling payload is opaque). Single upload (the C default
//! `Repeat` is 1).
//!
//! # Deviations from the C
//! - Degenerate-payload rejection (see Gating).
//! - Frame-final half-bit at a gap: when the 112th bit is `0` (HIGH→LOW in normal polarity) its
//!   LOW half merges into the trailing silence, and the C (which resets on any non-chip pulse)
//!   can never complete that frame. KAT completes the last bit when 111 bits are collected, the
//!   pending chip is the bit's first half and the next pulse is a gap (≥1150µs) of the opposite
//!   level. This only affects the final bit; the gate is unchanged.
//!
//! # Key files
//! The C serialize is the generic one, and the references' generic writers differ: Flipper-ARF
//! writes `Protocol: Mercedes V0`, `Bit: 112` and `Key` (the first 64 bits) only, losing the last
//! 48; doctor-firmware's adds `Serial: 0` / `Btn` (command nibble) / `Cnt: 0` (when the command is
//! non-zero), `TE: 500` and `Key_2` (the second word: bits 64..111 in its top 48 bits). KAT writes
//! doctor's lines, so its files reload completely; ARF reads only Bit/Key from them. Loading
//! follows `mercedes_v0_deserialize`: `Key` + `Key_2` (doctor's `Key2` too; zero when absent, as
//! ARF's own load leaves it), refused unless 112 bits, `Key_2 & 0xFFFF == 0` and a whitelisted
//! command nibble (`mercedes_v0_data_valid`). KAT's degenerate-payload gate is a receive-side
//! filter and is not applied to a loaded key.

use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`): FM, transmitted with doctor-firmware's "mercedes" preset (FM20K).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Mercedes V0",
    decoder: "Mercedes V0",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::FM,
    encryption: "Rolling Code",
    make: "Mercedes-Benz",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&FM20K],
};

/// Mercedes V0 buttons.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0xC => "Lock",
        0x8 => "Unlock",
        0xA => "Aux",
        0x0 => "Follow-up",
        _ => "Other",
    })
}

const TE_US: u32 = 500;
/// Frame length in bits (MERCEDES_V0_BITS).
const BITS: u8 = 112;
const PREAMBLE_MIN_EDGES: u16 = 108;
const PREAMBLE_MAX_EDGES: u16 = 113;
const TX_PREAMBLE_BITS: usize = 55;
const TX_START_US: u32 = 3700;
const COMMAND_GAP_US: u32 = 300_000;

// Chip classification windows (exclusive bounds, as in the C).
const CHIP1_MIN_US: u32 = 350;
const CHIP1_MAX_US: u32 = 650;
const CHIP2_MIN_US: u32 = 850;
const CHIP2_MAX_US: u32 = 1150;
/// KAT deviation: minimum gap duration that may complete the frame-final half bit.
const FINAL_GAP_MIN_US: u32 = CHIP2_MAX_US;

// Command nibbles (top nibble of the frame).
const CMD_LOCK: u8 = 0xC;
const CMD_UNLOCK: u8 = 0x8;
const CMD_AUX: u8 = 0xA;
const CMD_FOLLOW_UP: u8 = 0x0;
const CMD_RAW: u8 = 0x4;

// KAT generic button codes.
const BTN_LOCK: u8 = 0x01;
const BTN_UNLOCK: u8 = 0x02;
const BTN_TRUNK: u8 = 0x04;

/// Mask of the 60 bits of `hi` below the command nibble.
const HI_TAIL_MASK: u64 = 0x0FFF_FFFF_FFFF_FFFF;
/// Mask of the 48 valid bits of `lo` (bits 63..16).
const LO_VALID_MASK: u64 = 0xFFFF_FFFF_FFFF_0000;

/// Mercedes V0 decoder (matches `MercedesV0Decoder`).
pub struct MercedesV0Decoder {
    hi: u64,
    lo: u64,
    preamble_edges: u16,
    nbits: u8,
    /// First chip of the current Manchester pair (C: `int8_t pair_chip`, -1 = none).
    pair_chip: Option<bool>,
    active: bool,
    sync_pending: bool,
    invert: bool,
    have_level: bool,
    last_level: bool,
}

impl MercedesV0Decoder {
    pub fn new() -> Self {
        Self {
            hi: 0,
            lo: 0,
            preamble_edges: 0,
            nbits: 0,
            pair_chip: None,
            active: false,
            sync_pending: false,
            invert: false,
            have_level: false,
            last_level: false,
        }
    }

    /// `mercedes_v0_reset` (note: `last_level` is intentionally kept, as in the C).
    fn reset_state(&mut self) {
        self.hi = 0;
        self.lo = 0;
        self.preamble_edges = 0;
        self.nbits = 0;
        self.pair_chip = None;
        self.active = false;
        self.sync_pending = false;
        self.invert = false;
        self.have_level = false;
    }

    fn chips(duration: u32) -> u8 {
        if duration > CHIP1_MIN_US && duration < CHIP1_MAX_US {
            1
        } else if duration > CHIP2_MIN_US && duration < CHIP2_MAX_US {
            2
        } else {
            0
        }
    }

    /// `mercedes_v0_prefix_valid`: nibble whitelist from the PIC receive classifier.
    fn prefix_valid(hi: u64) -> bool {
        matches!(
            (hi >> 60) as u8,
            CMD_LOCK | CMD_UNLOCK | CMD_FOLLOW_UP | CMD_AUX | CMD_RAW
        )
    }

    /// KAT gate: the 108 payload bits after the command nibble are all 0 or all 1.
    fn payload_degenerate(hi: u64, lo: u64) -> bool {
        let tail = hi & HI_TAIL_MASK;
        let lo = lo & LO_VALID_MASK;
        (tail == 0 && lo == 0) || (tail == HI_TAIL_MASK && lo == LO_VALID_MASK)
    }

    /// Store one decoded bit MSB-first; returns true when the 112th bit has been stored.
    fn store_bit(&mut self, value: bool) -> bool {
        if value {
            if self.nbits < 64 {
                self.hi |= 1u64 << (63 - self.nbits);
            } else {
                self.lo |= 1u64 << (127 - self.nbits);
            }
        }
        self.nbits += 1;
        self.nbits == BITS
    }

    /// Validate a complete 112-bit frame and build the decoded signal.
    fn finish_frame(&self) -> Option<DecodedSignal> {
        if !Self::prefix_valid(self.hi) || Self::payload_degenerate(self.hi, self.lo) {
            return None;
        }
        Some(Self::frame_signal(self.hi, self.lo))
    }

    /// The decode of a 112-bit frame (`hi`, `lo` with the last 48 bits in its top 48).
    fn frame_signal(hi: u64, lo: u64) -> DecodedSignal {
        DecodedSignal {
            serial: None,
            button: Some((hi >> 60) as u8),
            counter: None,
            crc_valid: true,
            data: hi,
            data_count_bit: BITS as usize,
            encoder_capable: true,
            extra: Some(lo >> 16),
            protocol_display_name: None,
        }
    }

    /// KAT deviation: complete the frame-final bit whose second half merged into the trailing gap.
    fn complete_final_bit_at_gap(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        if !self.active
            || self.sync_pending
            || self.nbits != BITS - 1
            || duration < FINAL_GAP_MIN_US
        {
            return None;
        }
        let first = self.pair_chip?;
        if first == level {
            return None;
        }
        let bit = !first && level;
        if self.store_bit(bit ^ self.invert) {
            return self.finish_frame();
        }
        None
    }

    /// Map a KAT generic button to `(command, selected)` (ARF d-pad: UP/DOWN/LEFT; else original).
    fn map_button(button: u8, captured: u8) -> (u8, bool) {
        match button {
            BTN_LOCK => (CMD_LOCK, true),
            BTN_UNLOCK => (CMD_UNLOCK, true),
            BTN_TRUNK => (CMD_AUX, true),
            _ => (captured, false),
        }
    }

    /// `mercedes_v0_encoder_append`: merge adjacent same-level durations.
    fn enc_add_level(signal: &mut Vec<LevelDuration>, level: bool, duration: u32) {
        if let Some(last) = signal.last_mut() {
            if last.level == level {
                *last = LevelDuration::new(level, last.duration_us + duration);
                return;
            }
        }
        signal.push(LevelDuration::new(level, duration));
    }

    /// `mercedes_v0_encoder_burst`: lead gap + preamble + sync + 112 Manchester bits.
    fn enc_burst(signal: &mut Vec<LevelDuration>, hi: u64, lo: u64) {
        Self::enc_add_level(signal, false, TX_START_US);
        for _ in 0..TX_PREAMBLE_BITS {
            Self::enc_add_level(signal, true, TE_US);
            Self::enc_add_level(signal, false, TE_US);
        }
        Self::enc_add_level(signal, false, TE_US);
        Self::enc_add_level(signal, true, TE_US);
        for i in 0..BITS as u32 {
            let bit = if i < 64 {
                (hi >> (63 - i)) & 1 != 0
            } else {
                (lo >> (127 - i)) & 1 != 0
            };
            Self::enc_add_level(signal, !bit, TE_US);
            Self::enc_add_level(signal, bit, TE_US);
        }
    }
}

impl ProtocolDecoder for MercedesV0Decoder {
    fn name(&self) -> &'static str {
        "Mercedes V0"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_US,
            te_long: TE_US * 2,
            te_delta: 150,
            min_count_bit: BITS as usize,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        // C flags: SubGhzProtocolFlag_433 | SubGhzProtocolFlag_FM.
        &[433_920_000]
    }

    fn reset(&mut self) {
        self.reset_state();
        self.last_level = false;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let chips = Self::chips(duration);
        if chips == 0 {
            let result = self.complete_final_bit_at_gap(level, duration);
            self.reset_state();
            return result;
        }
        if self.have_level && self.last_level == level {
            self.reset_state();
        }
        self.last_level = level;
        self.have_level = true;

        if !self.active {
            if chips == 1 {
                if self.preamble_edges <= PREAMBLE_MAX_EDGES {
                    self.preamble_edges += 1;
                }
            } else if (PREAMBLE_MIN_EDGES..=PREAMBLE_MAX_EDGES).contains(&self.preamble_edges) {
                self.active = true;
                self.sync_pending = true;
                self.pair_chip = Some(level);
            } else {
                self.reset_state();
            }
            return None;
        }

        for _ in 0..chips {
            let first = match self.pair_chip.take() {
                None => {
                    self.pair_chip = Some(level);
                    continue;
                }
                Some(first) => first,
            };
            if first == level {
                self.reset_state();
                return None;
            }
            let bit = !first && level;
            if self.sync_pending {
                self.invert = !bit;
                self.sync_pending = false;
                continue;
            }
            if self.store_bit(bit ^ self.invert) {
                let result = self.finish_frame();
                self.reset_state();
                return result;
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        // mercedes_v0_data_valid: exact bit count + whitelisted command nibble.
        if decoded.data_count_bit != BITS as usize {
            return None;
        }
        let hi = decoded.data;
        let lo = (decoded.extra? << 16) & LO_VALID_MASK;
        if !Self::prefix_valid(hi) {
            return None;
        }

        let captured = (hi >> 60) as u8;
        let (command, selected) = Self::map_button(button, captured);
        let tail = hi & HI_TAIL_MASK;

        let burst_len = 1 + 2 * TX_PREAMBLE_BITS + 2 + 2 * BITS as usize;
        let mut signal = Vec::with_capacity(2 * burst_len + 1);
        Self::enc_burst(&mut signal, tail | ((command as u64) << 60), lo);
        if selected && command == CMD_UNLOCK {
            Self::enc_add_level(&mut signal, false, COMMAND_GAP_US);
            Self::enc_burst(&mut signal, tail, lo);
        }
        Some(signal)
    }

    /// `mercedes_v0_deserialize` (see the module doc, "Key files").
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        let hi = key.key_u64()?;
        let lo = key.hex_field("Key_2").or_else(|| key.hex_field("Key2")).unwrap_or(0);
        // mercedes_v0_data_valid
        if key.bits != BITS as usize || lo & 0xFFFF != 0 || !Self::prefix_valid(hi) {
            return None;
        }
        Some(Self::frame_signal(hi, lo))
    }

    /// doctor-firmware's `subghz_block_generic_serialize` of the decode (see "Key files").
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != BITS as usize {
            return None;
        }
        let hi = decoded.data;
        let lo = (decoded.extra? << 16) & LO_VALID_MASK;
        let command = (hi >> 60) as u32;
        let mut key = FlipperKey::for_protocol("Mercedes V0", BITS as usize, hi);
        if command != 0 {
            key.push_u32("Serial", 0);
            key.push_u32("Btn", command);
            key.push_u32("Cnt", 0);
        }
        key.push_u32("TE", TE_US);
        if lo != 0 {
            key.push_hex("Key_2", lo, 8);
        }
        Some(key)
    }
}

impl Default for MercedesV0Decoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::ProtocolRegistry;

    // A plausible frame: Lock command, arbitrary rolling payload.
    const HI_LOCK: u64 = 0xC5A3_19E7_4B2D_80F6;
    const EXTRA: u64 = 0x9D41_7E2C_B308; // 48 bits

    fn signal_for(hi: u64, extra: u64) -> DecodedSignal {
        DecodedSignal {
            serial: None,
            button: Some((hi >> 60) as u8),
            counter: None,
            crc_valid: true,
            data: hi,
            data_count_bit: BITS as usize,
            encoder_capable: true,
            extra: Some(extra),
            protocol_display_name: None,
        }
    }

    fn decode_all(pairs: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = MercedesV0Decoder::new();
        pairs
            .iter()
            .filter_map(|p| dec.feed(p.level, p.duration_us))
            .collect()
    }

    /// Independent wire builder (not the encoder): explicit half-bit chips per the documented
    /// format, with a configurable preamble pair count.
    fn build_wire(hi: u64, extra: u64, preamble_pairs: usize) -> Vec<LevelDuration> {
        let mut halves: Vec<bool> = Vec::new();
        for _ in 0..preamble_pairs {
            halves.push(true);
            halves.push(false);
        }
        halves.push(false); // sync LOW (merges with the last preamble LOW)
        halves.push(true); // sync HIGH
        let lo = extra << 16;
        for i in 0..112u32 {
            let bit = if i < 64 {
                (hi >> (63 - i)) & 1
            } else {
                (lo >> (127 - i)) & 1
            } != 0;
            halves.push(!bit);
            halves.push(bit);
        }
        let mut out = vec![LevelDuration::new(false, TX_START_US)];
        for h in halves {
            MercedesV0Decoder::enc_add_level(&mut out, h, TE_US);
        }
        out
    }

    #[test]
    fn synthetic_frame_decodes() {
        let wire = build_wire(HI_LOCK, EXTRA, 55);
        let decoded = decode_all(&wire);
        assert_eq!(decoded.len(), 1, "expected exactly one frame");
        let d = &decoded[0];
        assert_eq!(d.data, HI_LOCK);
        assert_eq!(d.extra, Some(EXTRA));
        assert_eq!(d.button, Some(CMD_LOCK));
        assert_eq!(d.data_count_bit, 112);
        assert!(d.crc_valid && d.encoder_capable);
    }

    #[test]
    fn inverted_polarity_decodes_same_frame() {
        // The sync pair sets `invert`, so an inverted capture yields the identical frame.
        let wire: Vec<LevelDuration> = build_wire(HI_LOCK, EXTRA, 55)
            .iter()
            .map(|p| LevelDuration::new(!p.level, p.duration_us))
            .collect();
        let decoded = decode_all(&wire);
        assert_eq!(decoded.len(), 1);
        assert_eq!(decoded[0].data, HI_LOCK);
        assert_eq!(decoded[0].extra, Some(EXTRA));
    }

    #[test]
    fn encode_decode_roundtrip_replays_captured_command() {
        // An unmapped KAT button (Panic 0x08) keeps the captured command → byte-exact replay.
        let enc = MercedesV0Decoder::new();
        let wire = enc
            .encode(&signal_for(HI_LOCK, EXTRA), 0x08)
            .expect("encode");
        assert_eq!(
            (wire[0].level, wire[0].duration_us),
            (false, TX_START_US),
            "3700µs TX start"
        );
        let decoded = decode_all(&wire);
        assert_eq!(decoded.len(), 1);
        assert_eq!(decoded[0].data, HI_LOCK);
        assert_eq!(decoded[0].extra, Some(EXTRA));
    }

    #[test]
    fn encode_restamps_command_nibble() {
        let enc = MercedesV0Decoder::new();
        let src = signal_for(HI_LOCK, EXTRA);
        let wire = enc.encode(&src, BTN_TRUNK).expect("encode");
        let decoded = decode_all(&wire);
        assert_eq!(decoded.len(), 1);
        assert_eq!(decoded[0].button, Some(CMD_AUX));
        assert_eq!(
            decoded[0].data,
            (HI_LOCK & HI_TAIL_MASK) | ((CMD_AUX as u64) << 60)
        );
        assert_eq!(
            decoded[0].extra,
            Some(EXTRA),
            "payload tail must be replayed unchanged"
        );
    }

    #[test]
    fn unlock_sends_follow_up_burst() {
        // Selected Unlock: 0x8 frame, 300ms gap, then the 0x0 "Follow-up" frame (C behaviour).
        let enc = MercedesV0Decoder::new();
        let wire = enc
            .encode(&signal_for(HI_LOCK, EXTRA), BTN_UNLOCK)
            .expect("encode");
        assert!(
            wire.iter()
                .any(|p| !p.level && p.duration_us >= COMMAND_GAP_US),
            "300ms command gap present"
        );
        let decoded = decode_all(&wire);
        let cmds: Vec<u8> = decoded.iter().map(|d| d.button.unwrap()).collect();
        assert_eq!(cmds, vec![CMD_UNLOCK, CMD_FOLLOW_UP]);
        assert!(decoded
            .iter()
            .all(|d| d.data & HI_TAIL_MASK == HI_LOCK & HI_TAIL_MASK));
    }

    #[test]
    fn final_zero_bit_completes_at_trailing_gap() {
        // Last bit 0 → its LOW half merges into the trailing silence. The C cannot complete such a
        // frame; KAT finishes the final bit on the gap (documented deviation).
        let extra = EXTRA & !1; // bit 111 (last wire bit) = 0
        let mut wire = build_wire(HI_LOCK, extra, 55);
        let last = wire.pop().unwrap();
        assert!(
            !last.level && last.duration_us == TE_US,
            "frame ends with the LOW half of a 0"
        );
        wire.push(LevelDuration::new(false, TE_US + 20_000));
        let decoded = decode_all(&wire);
        assert_eq!(decoded.len(), 1);
        assert_eq!(decoded[0].extra, Some(extra));
    }

    #[test]
    fn rejects_non_whitelisted_command() {
        for nibble in [0x1u64, 0x3, 0x7, 0xF] {
            let hi = (HI_LOCK & HI_TAIL_MASK) | (nibble << 60);
            assert!(
                decode_all(&build_wire(hi, EXTRA, 55)).is_empty(),
                "nibble {nibble:X}"
            );
        }
    }

    #[test]
    fn rejects_preamble_outside_edge_window() {
        // Single-chip edges before the 2-chip sync LOW = 2·pairs − 1 (the last preamble LOW merges
        // into the sync). 54 pairs → 107 edges (too short); 58 pairs → 115 (too long);
        // 55..=57 pairs → 109..=113 edges are accepted.
        assert!(decode_all(&build_wire(HI_LOCK, EXTRA, 54)).is_empty());
        assert!(decode_all(&build_wire(HI_LOCK, EXTRA, 58)).is_empty());
        for pairs in 55..=57 {
            assert_eq!(
                decode_all(&build_wire(HI_LOCK, EXTRA, pairs)).len(),
                1,
                "{pairs} pairs"
            );
        }
    }

    #[test]
    fn rejects_corrupted_manchester_pair() {
        // Stretch one mid-frame 500µs chip to 1000µs: the pair loses its mid-bit transition.
        let mut wire = build_wire(HI_LOCK, EXTRA, 55);
        let idx = wire.len() - 40;
        let p = wire[idx];
        if p.duration_us == TE_US {
            wire[idx] = LevelDuration::new(p.level, 2 * TE_US);
        } else {
            wire[idx] = LevelDuration::new(p.level, TE_US);
        }
        assert!(decode_all(&wire).is_empty());
    }

    #[test]
    fn rejects_degenerate_payload() {
        // Command 0x0 + all-zero payload = an endless run of preamble-like pairs (KAT gate).
        assert!(decode_all(&build_wire(0, 0, 55)).is_empty());
        let ones = ((CMD_LOCK as u64) << 60) | HI_TAIL_MASK;
        assert!(decode_all(&build_wire(ones, 0xFFFF_FFFF_FFFF, 55)).is_empty());
    }

    #[test]
    fn registry_reports_mercedes_v0() {
        // No earlier decoder in the registry may steal the frame at 433.92 MHz.
        let wire = MercedesV0Decoder::new()
            .encode(&signal_for(HI_LOCK, EXTRA), 0x08)
            .unwrap();
        let mut reg = ProtocolRegistry::new();
        let results = reg.process_signal_stream(&wire, 433_920_000);
        let names: Vec<&str> = results.iter().map(|(n, _, _)| n.as_str()).collect();
        assert_eq!(names, vec!["Mercedes V0"]);
        assert_eq!(results[0].1.data, HI_LOCK);
    }

    /// Key files: doctor-firmware's lines round-trip every field, Flipper-ARF's (no Key_2) load
    /// with a zero tail, invalid keys are refused, and TX from a loaded key decodes.
    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::{assert_round_trip, key};
        let dec = MercedesV0Decoder::new();
        let decoded = decode_all(&build_wire(HI_LOCK, EXTRA, 55)).remove(0);
        let text = assert_round_trip(&dec, &decoded);
        assert!(
            text.ends_with(
                "Protocol: Mercedes V0\nBit: 112\nKey: C5 A3 19 E7 4B 2D 80 F6\nSerial: 0\n\
                 Btn: 12\nCnt: 0\nTE: 500\nKey_2: 9D 41 7E 2C B3 08 00 00\n"
            ),
            "{text}"
        );
        // Command 0x0 (Follow-up): doctor writes no Serial/Btn/Cnt.
        let follow = MercedesV0Decoder::frame_signal(HI_LOCK & HI_TAIL_MASK, EXTRA << 16);
        assert!(assert_round_trip(&dec, &follow).ends_with("\nTE: 500\nKey_2: 9D 41 7E 2C B3 08 00 00\n"));

        // Flipper-ARF's file: Key only, the 48-bit tail is lost (zero), as ARF's own load has it.
        let arf = key("Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                       Preset: FuriHalSubGhzPreset2FSKDev476Async\nProtocol: Mercedes V0\n\
                       Bit: 112\nKey: C5 A3 19 E7 4B 2D 80 F6\n");
        let d = dec.from_flipper_key(&arf).unwrap();
        assert_eq!((d.button, d.data, d.extra), (Some(CMD_LOCK), HI_LOCK, Some(0)));
        // mercedes_v0_data_valid: command whitelist, bit count, clean low 16 bits of Key_2.
        let mut bad = arf.clone();
        bad.key = 0x1000_0000_0000_0000;
        bad.key_bytes = bad.key.to_be_bytes().to_vec();
        assert!(dec.from_flipper_key(&bad).is_none());
        let mut bad = arf.clone();
        bad.bits = 64;
        assert!(dec.from_flipper_key(&bad).is_none());
        let mut bad = arf.clone();
        bad.push("Key_2", "00 00 00 00 00 00 00 01");
        assert!(dec.from_flipper_key(&bad).is_none());

        // TX from the reloaded decode (Lock re-stamped, payload replayed).
        let reloaded = dec.from_flipper_key(&key(&text)).unwrap();
        let tx = decode_all(&dec.encode(&reloaded, BTN_LOCK).unwrap());
        assert_eq!((tx[0].data, tx[0].extra), (HI_LOCK, Some(EXTRA)));
    }

    /// Oracle vectors (`KAT_KEY_VECTORS=<dir> cargo test mercedes_v0_key_vectors -- --ignored`).
    #[test]
    #[ignore]
    fn mercedes_v0_key_vectors() {
        use super::super::bmw::{key_vector_dir, write_key_vector};
        let dec = MercedesV0Decoder::new();
        let frames = [(1, HI_LOCK, EXTRA), (2, 0x8A51_7C30_1E2F_9D44u64, 0x31A7_F00D_5E21u64)];
        for (n, hi, extra) in frames {
            let wire = dec.encode(&signal_for(hi, extra), 0x08).unwrap();
            let decoded = decode_all(&wire).remove(0);
            let key = dec.to_flipper_key(&decoded).unwrap();
            write_key_vector(&key_vector_dir(), &format!("mercedes_v0_{n}"), 433_920_000, "FuriHalSubGhzPreset2FSKDev476Async", &wire, &key);
        }
    }
}
