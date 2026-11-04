//! Renault V1 (ARF) protocol decoder/encoder — Flipper-ARF's firmware "Renault V1": the 82-bit
//! type-table Renault frames (types 0x04/0x0C/0x1A/0x3B/0x3F and "Dynamic") on the Renault V0
//! Manchester wire.
//!
//! Ported from `REFERENCES/Flipper-ARF/lib/subghz/protocols/renault_v1.c` (decode path, replay
//! encoder). ARF registers it right after its Renault V0 under the name "Renault V1"; KAT names it
//! "Renault V1 (ARF)" because KAT's "Renault V1" is ProtoPirate's unrelated HITAG2/PCF79xx decoder
//! (`renault_v1.rs`). History: these types were part of ProtoPirate's Renault V0 until ProtoPirate
//! 1287d10 ("move newer Renault variants into Renault V1") replaced them with the HITAG2 decoder;
//! ARF instead split them out of its Renault V0 into this decoder, and its Renault V0 (like KAT's
//! `renault_v0.rs`) now only accepts type 0x13. ARF's firmware still decodes them, so KAT does too
//! (union of capabilities). No capture of such a remote exists in `IMPORTS/`.
//!
//! RF: 315 / 433.92 / 868.35 MHz, AM or FM (C flags).
//!
//! ## Wire (`subghz_protocol_decoder_renault_v1_feed`)
//! A HIGH ≥ 800 µs is the sync that starts a burst. Pulses are classified against three TE
//! profiles in order — (125, 250, ±69), (124, 248, ±60), (108, 250, ±55) µs — with the C's exact
//! unsigned comparisons, which works out to short = 53..=180 µs, long = 181..=319 µs. Manchester
//! is ARF's inline table `{0x01, 0x91, 0x9B, 0xFB}` with Flipper's state numbering, entry state
//! Mid1, events `level ? ShortLow(0)/LongLow(4) : ShortHigh(2)/LongHigh(6)`; a transition to the
//! same state is ignored; entering Mid1 records a 1, Mid0 a 0 (up to 112 bits). A burst ends —
//! and the buffered bits are judged (`renault_v1_decode_candidate`) — on, in this order: a LOW of
//! 1200..=2000 µs once ≥ 96 bits are buffered (then wait for the next sync), any pulse ≥ 5000 µs
//! (a HIGH one is also a new sync), more than 109 buffered bits, or any pulse that fits no TE
//! profile (a HIGH ≥ 800 µs is also a new sync).
//!
//! ## Frame (82 bits after a 12/16/18/20-bit preamble)
//! The candidate is the LAST 82 bits of the burst: `offset = bits − 82` must be 12, 16, 18 or 20
//! (the preamble bits themselves are not checked). `data` = 64 bits, `key2` = 18 bits:
//!   data[63:40] serial (24), data[39:32] button (8), data[31:24] counter (8), data[23:0] + key2 =
//!   42 "payload" bits. checksum = XOR of the 8 data bytes, key2[17:10] and key2[9:2];
//!   low6 = checksum & 0x3F, hi2xor = (checksum >> 6) ^ (key2 & 3).
//! - Typed (`renault_v1_find_type_by_checks` + `renault_v1_type_button_valid`): low6 is one of
//!   0x04 / 0x0C / 0x1A / 0x3B / 0x3F (C1), hi2xor equals the type's value (0 / 0 / 3 / 0 / 0; C2),
//!   the button is in the type's range (0x04..=0x0B / 0xC4..=0xCB / 0x44..=0x4B / 0x44..=0x4B /
//!   0xC4..=0xCB) and the preamble has the type's length (16 / 18 / 12 / 20 / 20).
//! - Otherwise, unless low6 == 0x13 (that tag belongs to Renault V0), "Dynamic": a button in
//!   0x04..=0x0B / 0x44..=0x4B / 0xC4..=0xCB, a non-zero serial and hi2xor ∈ {0, 3}; any valid
//!   preamble length. A Dynamic frame is emitted only when two consecutive candidates are
//!   identical (data, key2, tag, preamble; `renault_v1_confirm_attempt`).
//!
//! A frame identical to the last emitted one is suppressed (`packet_bit_count` / data / key2).
//! `reset()` mirrors the C reset: it clears the pending Dynamic candidate and key2 but keeps the
//! last data and `packet_bit_count`, so after a registry reset the same frame is reported again
//! unless its key2 is 0 (C quirk, kept).
//!
//! Buttons (ARF names): low nibble 4..7 "Lock", 8..B "Unlock" (the high nibble is the type family).
//!
//! ## DecodedSignal packing
//!   serial = serial (24 bits), button = raw button byte, counter = counter byte,
//!   crc_valid = C1 && C2 (always true on emission), data = the 64 data bits, data_count_bit = 82,
//!   extra = key2 (bits 0..17, the last 18 wire bits) | preamble length << 24 (needed to re-encode
//!   Dynamic frames, whose preamble is not implied by the type), encoder_capable = true,
//!   protocol_display_name = "Renault V1 (ARF) [0x04]" … "[0x3F]" or "[Dynamic 0xNN]" (NN = the
//!   checksum tag), which `ProtocolRegistry::get()` resolves to this decoder.
//!
//! ## Encoder (`subghz_protocol_encoder_renault_v1_deserialize`, replay)
//! ARF transmits these frames as a byte-identical REPLAY (its D-pad is disabled; any requested
//! button replays the capture). The frame must still classify; the preamble is the stored one, or
//! the type's default. Upload (`renault_v1_build_upload`), per burst ×3: HIGH 1000 µs, then
//! `preamble` 1-bits and the 82 frame bits through ARF's two-state emitter
//! (`renault_v1_emit_decoded_bit`: state 1, bit 1 → LOW te, HIGH te; state 1, bit 0 → LOW 2te,
//! state 2; state 2, bit 0 → HIGH te, LOW te; state 2, bit 1 → HIGH 2te, state 1), a closing
//! HIGH te if in state 2, LOW 1500 µs; te = 140 µs for a 12-bit preamble, else 125 µs; adjacent
//! levels merged. The whole upload is repeated 10 times (RENAULT_V1_REPLAY_REPEAT, the minimum the
//! C allows). Like the C (600-entry upload buffer), a frame whose upload needs more than 600 levels
//! is not transmitted.
//!
//! Not ported (see `docs/renault_v1_arf.md`):
//! - The Fiat-V1-key search on 3 guessed hop slices × 4 IV combos
//!   (`renault_v1_verify_hitag2_key`, + the UID key database): ARF marks it "pending validation
//!   against a real capture", and ARF only displays its result (the encoder stays replay-only).
//! - The "Seed BF" forward re-encoder (`subghz_protocol_renault_v1_run_seed_bf`,
//!   `renault_v1_encoder_reencode_seed`): it runs ProtoPirate's PCF seed model on the 82-bit frame
//!   packed into 11 bytes, then re-encrypts with a different UID than the one it searched with
//!   (the BF uses data[63:32] = serial ‖ button, the re-encode the 24-bit serial alone), so the
//!   "next code" cannot be consistent with the captured one.
//!
//! Overlap with ProtoPirate's HITAG2 Renault V1: on its own this decoder (exactly like ARF's C)
//! also reads two consecutive HITAG2/PCF long frames (1500/1000 µs header, 104 bits) as a Dynamic
//! frame. In KAT's registry the HITAG2 decoder (`renault_v1.rs`, earlier in the list) completes
//! each long frame no later than this decoder's burst end, and the reset after its hit drops the
//! pending Dynamic candidate, so such frames stay "Renault V1" (tested).
//!
//! Deviations from C: none in emitted frames (the C fires its callback inside the same feed call).

use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// RENAULT_V1_MIN_BITS: data (64) + key2 (18).
const FRAME_BITS: usize = 0x52;
const DATA_BITS: usize = 64;
const KEY2_BITS: usize = 18;
const KEY2_MASK: u32 = (1 << KEY2_BITS) - 1;
/// RENAULT_V1_DECODER_BIT_LIMIT: more buffered bits than this ends the burst.
const DECODER_BIT_LIMIT: usize = 0x6D;
/// RENAULT_V1_DECODED_BITS_MAX.
const DECODED_BITS_MAX: usize = 0x70;
const SYNC_MIN_US: u32 = 0x320;
const GAP_RESET_US: u32 = 5000;
const END_BURST_MIN_BITS: usize = 96;
const END_BURST_MIN_US: u32 = 1200;
const END_BURST_MAX_US: u32 = 2000;

/// `renault_v1_te_profiles`: (te_short, te_long, te_delta), tried in order.
const TE_PROFILES: [(u32, u32, u32); 3] = [(0x7D, 0xFA, 0x45), (124, 248, 60), (108, 250, 55)];

/// `renault_v1_decoder_state_table` (Flipper's transition table; states Start1 0, Mid1 1, Mid0 2,
/// Start0 3; events ShortLow 0, ShortHigh 2, LongLow 4, LongHigh 6).
const STATE_TABLE: [u8; 4] = [0x01, 0x91, 0x9B, 0xFB];
const STATE_MID1: u8 = 1;
const STATE_MID0: u8 = 2;

/// Encoder (`renault_v1_build_upload` / `renault_v1_upload_shape_for_preamble`).
const TE_DEFAULT_US: u32 = 125;
const TE_PREAMBLE_12_US: u32 = 140;
const TX_SYNC_HIGH_US: u32 = 1000;
const TX_BURST_COUNT: usize = 3;
const TX_INTER_BURST_LOW_US: u32 = 1500;
const TX_FINAL_LOW_US: u32 = 1500;
/// RENAULT_V1_UPLOAD_CAPACITY: the C's upload buffer (levels after merging).
const TX_UPLOAD_CAPACITY: usize = 0x258;
/// RENAULT_V1_REPLAY_REPEAT: the C never sends fewer repeats.
const TX_REPEAT: usize = 10;

/// Bit position of the preamble length in `DecodedSignal::extra` (below it: key2).
const EXTRA_PREAMBLE_SHIFT: u32 = 24;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FrameType {
    T04,
    T0C,
    T1A,
    T3B,
    T3F,
    Dynamic,
}

/// One row of `renault_v1_types`: type, checksum low6 (= the type value), checksum hi2 xor.
struct TypeEntry {
    id: FrameType,
    low6: u8,
    high2_xor: u8,
}

const TYPES: [TypeEntry; 5] = [
    TypeEntry {
        id: FrameType::T04,
        low6: 0x04,
        high2_xor: 0x00,
    },
    TypeEntry {
        id: FrameType::T0C,
        low6: 0x0C,
        high2_xor: 0x00,
    },
    TypeEntry {
        id: FrameType::T1A,
        low6: 0x1A,
        high2_xor: 0x03,
    },
    TypeEntry {
        id: FrameType::T3B,
        low6: 0x3B,
        high2_xor: 0x00,
    },
    TypeEntry {
        id: FrameType::T3F,
        low6: 0x3F,
        high2_xor: 0x00,
    },
];

/// `RenaultV1DecodeAttempt` (the fields the decoder keeps).
#[derive(Debug, Clone, Copy, PartialEq)]
struct Attempt {
    data: u64,
    key2: u32,
    serial: u32,
    button: u8,
    counter: u8,
    frame_type: FrameType,
    /// Type value, or the checksum low6 for Dynamic frames.
    type_tag: u8,
    preamble_bits: u8,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Data,
}

pub struct RenaultV1ArfDecoder {
    step: DecoderStep,
    manchester_state: u8,
    decoded_bits: [u8; DECODED_BITS_MAX],
    decoded_bit_count: usize,
    /// Last emitted frame (`generic.data`, `key2`, `packet_bit_count != 0`): duplicate memory.
    data: u64,
    key2: u32,
    packet_bit_count: usize,
    /// Dynamic candidate waiting for its confirmation (`pending_attempt`).
    pending: Option<Attempt>,
}

impl RenaultV1ArfDecoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            manchester_state: STATE_MID1,
            decoded_bits: [0; DECODED_BITS_MAX],
            decoded_bit_count: 0,
            data: 0,
            key2: 0,
            packet_bit_count: 0,
            pending: None,
        }
    }

    /// `renault_v1_type_button_valid`.
    fn type_button_valid(frame_type: FrameType, button: u8) -> bool {
        match frame_type {
            FrameType::T04 => (0x04..=0x0B).contains(&button),
            FrameType::T0C | FrameType::T3F => (0xC4..=0xCB).contains(&button),
            FrameType::T1A | FrameType::T3B => (0x44..=0x4B).contains(&button),
            FrameType::Dynamic => false,
        }
    }

    /// `renault_v1_button_valid_generic`.
    fn button_valid_generic(button: u8) -> bool {
        (0x04..=0x0B).contains(&button)
            || (0x44..=0x4B).contains(&button)
            || (0xC4..=0xCB).contains(&button)
    }

    /// `renault_v1_preamble_bits_valid`.
    fn preamble_bits_valid(bits: usize) -> bool {
        matches!(bits, 12 | 16 | 18 | 20)
    }

    /// `renault_v1_default_preamble_bits`.
    fn default_preamble_bits(frame_type: FrameType) -> u8 {
        match frame_type {
            FrameType::T04 => 16,
            FrameType::T0C => 18,
            FrameType::T1A => 12,
            FrameType::T3B | FrameType::T3F => 20,
            FrameType::Dynamic => 0,
        }
    }

    /// `renault_v1_type_preamble_bits_valid`.
    fn type_preamble_bits_valid(frame_type: FrameType, bits: u8) -> bool {
        if frame_type == FrameType::Dynamic {
            Self::preamble_bits_valid(bits as usize)
        } else {
            bits == Self::default_preamble_bits(frame_type)
        }
    }

    /// `renault_v1_checksum`: XOR of the 8 data bytes, key2[17:10] and key2[9:2].
    fn checksum(data: u64, key2: u32) -> u8 {
        let folded = data.to_be_bytes().iter().fold(0u8, |acc, b| acc ^ b);
        folded ^ ((key2 >> 10) & 0xFF) as u8 ^ ((key2 >> 2) & 0xFF) as u8
    }

    /// `renault_v1_classify_frame`: the typed table (C1 + C2 + per-type button), else the Dynamic
    /// rules. `None` = rejected. The preamble length is checked separately.
    fn classify_frame(data: u64, key2: u32) -> Option<Attempt> {
        let serial = (data >> 40) as u32;
        let button = (data >> 32) as u8;
        let counter = (data >> 24) as u8;
        let checksum = Self::checksum(data, key2);
        let low6 = checksum & 0x3F;
        let high2_xor = ((checksum >> 6) & 0x03) ^ (key2 & 0x03) as u8;
        let attempt = |frame_type, type_tag| Attempt {
            data,
            key2,
            serial,
            button,
            counter,
            frame_type,
            type_tag,
            preamble_bits: 0,
        };

        // renault_v1_find_type_by_checks: the first entry whose low6 matches decides (C2 fails
        // when its hi2 xor does not).
        if let Some(entry) = TYPES.iter().find(|t| t.low6 == low6) {
            if entry.high2_xor == high2_xor && Self::type_button_valid(entry.id, button) {
                return Some(attempt(entry.id, entry.low6));
            }
        }
        // [SPLIT] the 0x13 tag belongs to Renault V0.
        if low6 == 0x13 {
            return None;
        }
        let dynamic_c1 = Self::button_valid_generic(button) && serial != 0 && serial <= 0xFF_FFFF;
        let dynamic_c2 = high2_xor == 0x00 || high2_xor == 0x03;
        (dynamic_c1 && dynamic_c2).then(|| attempt(FrameType::Dynamic, low6))
    }

    /// `renault_v1_classify_event` over the TE profiles: the Manchester event, or `None`.
    fn classify_event(duration: u32, level: bool) -> Option<u8> {
        let short_event = if level { 0 } else { 2 };
        let long_event = if level { 4 } else { 6 };
        TE_PROFILES
            .iter()
            .find_map(|&(te_short, te_long, te_delta)| {
                if duration < te_short {
                    return (te_short - duration <= te_delta).then_some(short_event);
                }
                if duration < te_long {
                    let short_delta = duration - te_short;
                    let long_inv_delta = te_long - duration;
                    if short_delta <= te_delta {
                        return Some(if long_inv_delta > te_delta {
                            short_event
                        } else {
                            long_event
                        });
                    }
                    return (long_inv_delta <= te_delta).then_some(long_event);
                }
                (duration - te_long <= te_delta).then_some(long_event)
            })
    }

    /// `renault_v1_attempt_at_offset`: the 82 bits after `offset` preamble bits.
    fn attempt_at_offset(&self, offset: usize) -> Option<Attempt> {
        if offset + FRAME_BITS > self.decoded_bit_count {
            return None;
        }
        let bits = &self.decoded_bits[offset..offset + FRAME_BITS];
        let data = bits[..DATA_BITS]
            .iter()
            .fold(0u64, |acc, &b| (acc << 1) | (b & 1) as u64);
        let key2 = bits[DATA_BITS..]
            .iter()
            .fold(0u32, |acc, &b| (acc << 1) | (b & 1) as u32);
        let mut attempt = Self::classify_frame(data, key2)?;
        attempt.preamble_bits = offset as u8;
        Self::type_preamble_bits_valid(attempt.frame_type, attempt.preamble_bits).then_some(attempt)
    }

    /// `renault_v1_confirm_attempt`: typed frames pass; a Dynamic frame passes only when it
    /// repeats the pending candidate.
    fn confirm_attempt(&mut self, attempt: &Attempt) -> bool {
        if attempt.frame_type != FrameType::Dynamic {
            self.pending = None;
            return true;
        }
        if self.pending == Some(*attempt) {
            self.pending = None;
            return true;
        }
        self.pending = Some(*attempt);
        false
    }

    /// `renault_v1_decode_candidate` (+ `renault_v1_apply_attempt` and the callback).
    fn decode_candidate(&mut self) -> Option<DecodedSignal> {
        // C: `if(decoded_bit_count <= 0x51) return;`
        if self.decoded_bit_count < FRAME_BITS {
            return None;
        }
        let tail_offset = self.decoded_bit_count - FRAME_BITS;
        let attempt = match Self::preamble_bits_valid(tail_offset)
            .then(|| self.attempt_at_offset(tail_offset))
            .flatten()
        {
            Some(attempt) => attempt,
            None => {
                self.pending = None;
                self.packet_bit_count = 0;
                return None;
            }
        };
        if !self.confirm_attempt(&attempt) {
            self.decoded_bit_count = 0;
            return None;
        }
        if self.packet_bit_count != 0 && self.data == attempt.data && self.key2 == attempt.key2 {
            self.decoded_bit_count = 0;
            return None;
        }
        self.data = attempt.data;
        self.key2 = attempt.key2;
        self.packet_bit_count = FRAME_BITS;
        self.decoded_bit_count = 0;
        Some(Self::decoded_signal(&attempt))
    }

    fn decoded_signal(attempt: &Attempt) -> DecodedSignal {
        // ARF refuses to transmit a frame whose upload overflows its buffer (see Encoder).
        let encodable =
            Self::build_upload(attempt.data, attempt.key2, attempt.preamble_bits).is_some();
        let label = if attempt.frame_type == FrameType::Dynamic {
            format!("Dynamic 0x{:02X}", attempt.type_tag)
        } else {
            format!("0x{:02X}", attempt.type_tag)
        };
        DecodedSignal {
            serial: Some(attempt.serial),
            button: Some(attempt.button),
            counter: Some(attempt.counter as u16),
            crc_valid: true,
            data: attempt.data,
            data_count_bit: FRAME_BITS,
            encoder_capable: encodable,
            extra: Some(
                attempt.key2 as u64 | ((attempt.preamble_bits as u64) << EXTRA_PREAMBLE_SHIFT),
            ),
            protocol_display_name: Some(format!("Renault V1 (ARF) [{label}]")),
        }
    }

    fn start_burst(&mut self) {
        self.step = DecoderStep::Data;
        self.decoded_bit_count = 0;
        self.manchester_state = STATE_MID1;
    }

    /// `renault_v1_emit`: merge with the previous level, else append — refused once the C's
    /// 600-entry buffer is full.
    fn emit(signal: &mut Vec<LevelDuration>, level: bool, duration: u32) -> Option<()> {
        if let Some(last) = signal.last_mut() {
            if last.level == level {
                *last = LevelDuration::new(level, last.duration_us + duration);
                return Some(());
            }
        }
        if signal.len() >= TX_UPLOAD_CAPACITY {
            return None;
        }
        signal.push(LevelDuration::new(level, duration));
        Some(())
    }

    /// `renault_v1_emit_decoded_bit` (ARF's two-state emitter, see module doc).
    fn emit_decoded_bit(
        signal: &mut Vec<LevelDuration>,
        state: &mut u8,
        te: u32,
        bit: bool,
    ) -> Option<()> {
        if *state == 1 {
            if bit {
                Self::emit(signal, false, te)?;
                return Self::emit(signal, true, te);
            }
            *state = 2;
            return Self::emit(signal, false, te * 2);
        }
        if bit {
            *state = 1;
            return Self::emit(signal, true, te * 2);
        }
        Self::emit(signal, true, te)?;
        Self::emit(signal, false, te)
    }

    /// `renault_v1_get_bit_msb82`.
    fn bit_msb82(data: u64, key2: u32, index: usize) -> bool {
        if index < DATA_BITS {
            (data >> (63 - index)) & 1 == 1
        } else {
            (key2 >> (FRAME_BITS - 1 - index)) & 1 == 1
        }
    }

    /// `renault_v1_build_upload` for one frame (3 bursts); `None` when the C's buffer overflows.
    fn build_upload(data: u64, key2: u32, preamble_bits: u8) -> Option<Vec<LevelDuration>> {
        if !Self::preamble_bits_valid(preamble_bits as usize) {
            return None;
        }
        let te = if preamble_bits == 12 {
            TE_PREAMBLE_12_US
        } else {
            TE_DEFAULT_US
        };
        let mut s = Vec::with_capacity(TX_UPLOAD_CAPACITY);
        for burst in 0..TX_BURST_COUNT {
            Self::emit(&mut s, true, TX_SYNC_HIGH_US)?;
            let mut state = 1u8;
            for _ in 0..preamble_bits {
                Self::emit_decoded_bit(&mut s, &mut state, te, true)?;
            }
            for index in 0..FRAME_BITS {
                Self::emit_decoded_bit(&mut s, &mut state, te, Self::bit_msb82(data, key2, index))?;
            }
            if state == 2 {
                Self::emit(&mut s, true, te)?;
            }
            let trailing_low = if burst + 1 < TX_BURST_COUNT {
                TX_INTER_BURST_LOW_US
            } else {
                TX_FINAL_LOW_US
            };
            Self::emit(&mut s, false, trailing_low)?;
        }
        Some(s)
    }
}

impl ProtocolDecoder for RenaultV1ArfDecoder {
    fn name(&self) -> &'static str {
        "Renault V1 (ARF)"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_PROFILES[0].0,
            te_long: TE_PROFILES[0].1,
            te_delta: TE_PROFILES[0].2,
            min_count_bit: FRAME_BITS,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[315_000_000, 433_920_000, 868_350_000]
    }

    /// `subghz_protocol_decoder_renault_v1_reset`: the last emitted data and `packet_bit_count`
    /// survive, key2 and the pending Dynamic candidate do not.
    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.manchester_state = STATE_MID1;
        self.decoded_bit_count = 0;
        self.key2 = 0;
        self.pending = None;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        if self.step == DecoderStep::Reset {
            if level && duration >= SYNC_MIN_US {
                self.start_burst();
            }
            return None;
        }

        // renault_v1_is_end_burst
        if !level
            && self.decoded_bit_count >= END_BURST_MIN_BITS
            && (END_BURST_MIN_US..=END_BURST_MAX_US).contains(&duration)
        {
            let result = self.decode_candidate();
            self.step = DecoderStep::Reset;
            return result;
        }

        if duration >= GAP_RESET_US {
            let result = self.decode_candidate();
            self.step = DecoderStep::Reset;
            if level && duration >= SYNC_MIN_US {
                self.start_burst();
            }
            return result;
        }

        if self.decoded_bit_count > DECODER_BIT_LIMIT {
            let result = self.decode_candidate();
            self.step = DecoderStep::Reset;
            return result;
        }

        let Some(event) = Self::classify_event(duration, level) else {
            let starts_next_burst = level && duration >= SYNC_MIN_US;
            let result = self.decode_candidate();
            if starts_next_burst {
                self.start_burst();
            } else {
                self.step = DecoderStep::Reset;
            }
            return result;
        };

        let state = self.manchester_state & 0x03;
        let next_state = (STATE_TABLE[state as usize] >> event) & 0x03;
        if next_state == state {
            return None;
        }
        self.manchester_state = next_state;
        if next_state == STATE_MID1 || next_state == STATE_MID0 {
            self.decoded_bits[self.decoded_bit_count] = u8::from(next_state == STATE_MID1);
            self.decoded_bit_count += 1;
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    /// Replay (`subghz_protocol_encoder_renault_v1_deserialize` without a seed): the requested
    /// button is ignored, as ARF disables the D-pad for these frames.
    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != FRAME_BITS {
            return None;
        }
        let extra = decoded.extra?;
        let key2 = (extra as u32) & KEY2_MASK;
        let stored_preamble = ((extra >> EXTRA_PREAMBLE_SHIFT) & 0xFF) as u8;
        let attempt = Self::classify_frame(decoded.data, key2)?;
        let preamble_bits = if Self::preamble_bits_valid(stored_preamble as usize) {
            stored_preamble
        } else {
            Self::default_preamble_bits(attempt.frame_type)
        };
        let upload = Self::build_upload(decoded.data, key2, preamble_bits)?;
        let mut signal = Vec::with_capacity(upload.len() * TX_REPEAT);
        for _ in 0..TX_REPEAT {
            signal.extend(
                upload
                    .iter()
                    .map(|ld| LevelDuration::new(ld.level, ld.duration_us)),
            );
        }
        Some(signal)
    }
}

impl Default for RenaultV1ArfDecoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::ProtocolRegistry;

    /// One C vector per type (frames built for the compiled ARF renault_v1.c, which encoded and
    /// decoded them): (data, key2, preamble, display tag, upload levels, upload total µs, first 6,
    /// last 4 pulses of one upload).
    /// (data, key2, preamble, display tag, upload levels, upload total µs, first 6, last 4).
    type CVector = (u64, u32, u8, &'static str, usize, i64, [i64; 6], [i64; 4]);

    const C_VECTORS: [CVector; 6] = [
        (
            0x560D_8304_EC72_18DE,
            0x35352,
            16,
            "0x04",
            468,
            81_375,
            [1000, -125, 125, -125, 125, -125],
            [250, -250, 125, -1500],
        ),
        (
            0x17F1_5BC6_A890_6A20,
            0x32B3C,
            18,
            "0x0C",
            480,
            82_875,
            [1000, -125, 125, -125, 125, -125],
            [125, -125, 125, -1500],
        ),
        (
            0x045B_A649_45B1_2CEA,
            0x2B8DB,
            12,
            "0x1A",
            420,
            86_460,
            [1000, -140, 140, -140, 140, -140],
            [280, -140, 140, -1500],
        ),
        (
            0x2AAE_4649_D26A_167B,
            0x1F561,
            20,
            "0x3B",
            468,
            84_000,
            [1000, -125, 125, -125, 125, -125],
            [125, -125, 250, -1500],
        ),
        (
            0xFB05_51CA_E5D2_5F8D,
            0x27D81,
            20,
            "0x3F",
            498,
            84_000,
            [1000, -125, 125, -125, 125, -125],
            [125, -125, 250, -1500],
        ),
        (
            0x32FC_A308_1300_75FF,
            0x2ABB5,
            16,
            "Dynamic 0x3B",
            480,
            81_000,
            [1000, -125, 125, -125, 125, -125],
            [250, -250, 250, -1500],
        ),
    ];

    fn pulses(signal: &[LevelDuration]) -> Vec<i64> {
        signal
            .iter()
            .map(|ld| {
                if ld.level {
                    ld.duration_us as i64
                } else {
                    -(ld.duration_us as i64)
                }
            })
            .collect()
    }

    fn signal_for(data: u64, key2: u32, preamble: u8) -> DecodedSignal {
        DecodedSignal {
            serial: None,
            button: None,
            counter: None,
            crc_valid: true,
            data,
            data_count_bit: FRAME_BITS,
            encoder_capable: true,
            extra: Some(key2 as u64 | (preamble as u64) << EXTRA_PREAMBLE_SHIFT),
            protocol_display_name: None,
        }
    }

    /// One 3-burst upload of an arbitrary frame (ARF's emitter, valid for frames that do not
    /// classify too).
    fn upload(data: u64, key2: u32, preamble: u8) -> Vec<LevelDuration> {
        RenaultV1ArfDecoder::build_upload(data, key2, preamble).expect("fits the C buffer")
    }

    fn decode_all(dec: &mut RenaultV1ArfDecoder, signal: &[LevelDuration]) -> Vec<DecodedSignal> {
        std::iter::once(LevelDuration::new(false, 20_000))
            .chain(
                signal
                    .iter()
                    .map(|ld| LevelDuration::new(ld.level, ld.duration_us)),
            )
            .filter_map(|ld| dec.feed(ld.level, ld.duration_us))
            .collect()
    }

    fn registry_names(signal: &[LevelDuration], freq: u32) -> Vec<String> {
        let mut stream = vec![LevelDuration::new(false, 30_000)];
        stream.extend(
            signal
                .iter()
                .map(|ld| LevelDuration::new(ld.level, ld.duration_us)),
        );
        ProtocolRegistry::new()
            .process_signal_stream(&stream, freq)
            .into_iter()
            .map(|r| r.0)
            .collect()
    }

    /// Frame with a chosen checksum: the last data byte closes low6 = `tag` and
    /// (checksum >> 6) ^ (key2 & 3) = `hi2xor`.
    fn make(serial: u32, button: u8, counter: u8, key2: u32, tag: u8, hi2xor: u8) -> u64 {
        let data = ((serial as u64) << 40)
            | ((button as u64) << 32)
            | ((counter as u64) << 24)
            | 0x00A5_5A00;
        let want = (tag & 0x3F) | ((hi2xor ^ (key2 & 3) as u8) << 6);
        data | (want ^ RenaultV1ArfDecoder::checksum(data, key2)) as u64
    }

    /// The three TE profiles amount to short = 53..=180 µs and long = 181..=319 µs (the C's
    /// unsigned comparisons); HIGH maps to the *Low events, LOW to the *High events.
    #[test]
    fn classify_event_windows_match_c() {
        for (d, high, low) in [
            (52, None, None),
            (53, Some(0), Some(2)),
            (125, Some(0), Some(2)),
            (180, Some(0), Some(2)),
            (181, Some(4), Some(6)),
            (250, Some(4), Some(6)),
            (319, Some(4), Some(6)),
            (320, None, None),
        ] {
            assert_eq!(
                RenaultV1ArfDecoder::classify_event(d, true),
                high,
                "HIGH {d}"
            );
            assert_eq!(
                RenaultV1ArfDecoder::classify_event(d, false),
                low,
                "LOW {d}"
            );
        }
    }

    /// Each type (and a Dynamic frame) replays exactly like the compiled ARF encoder (one upload =
    /// 3 bursts; pulse count, total time and both ends), the replay is sent 10 times, and decoding
    /// it reports the frame once (duplicates suppressed) with ARF's fields.
    #[test]
    fn c_vectors_encode_and_decode() {
        for (data, key2, pre, tag, levels, total, first, last) in C_VECTORS {
            let one = upload(data, key2, pre);
            let p = pulses(&one);
            assert_eq!(p.len(), levels, "{tag}");
            assert_eq!(p.iter().map(|v| v.abs()).sum::<i64>(), total, "{tag}");
            assert_eq!(&p[..6], &first, "{tag}");
            assert_eq!(&p[p.len() - 4..], &last, "{tag}");

            let tx = RenaultV1ArfDecoder::new()
                .encode(&signal_for(data, key2, pre), 0x01)
                .unwrap();
            assert_eq!(
                pulses(&tx),
                p.repeat(TX_REPEAT),
                "{tag}: 10 identical uploads"
            );

            let out = decode_all(&mut RenaultV1ArfDecoder::new(), &tx);
            assert_eq!(out.len(), 1, "{tag}");
            let d = &out[0];
            assert_eq!(d.serial, Some((data >> 40) as u32));
            assert_eq!(d.button, Some((data >> 32) as u8));
            assert_eq!(d.counter, Some(((data >> 24) & 0xFF) as u16));
            assert_eq!(d.data, data);
            assert_eq!(d.data_count_bit, 82);
            assert_eq!(d.extra, Some(key2 as u64 | (pre as u64) << 24));
            assert!(d.crc_valid && d.encoder_capable);
            assert_eq!(
                d.protocol_display_name.as_deref(),
                Some(format!("Renault V1 (ARF) [{tag}]").as_str())
            );
        }
    }

    /// A Dynamic frame is reported only when two consecutive candidates are identical; another
    /// frame in between, or a reset, restarts the confirmation. Typed frames need no
    /// confirmation.
    #[test]
    fn dynamic_frames_need_two_identical_bursts() {
        let (data, key2, pre) = (C_VECTORS[5].0, C_VECTORS[5].1, C_VECTORS[5].2);
        let burst = {
            // first burst of the upload: up to and including its trailing LOW 1500.
            let up = upload(data, key2, pre);
            let end = up
                .iter()
                .position(|ld| !ld.level && ld.duration_us == 1500)
                .unwrap();
            up[..=end].to_vec()
        };
        let other = make(0x123456, 0x46, 7, 0x15555, 0x2A, 0);
        let other_burst: Vec<LevelDuration> = {
            let up = upload(other, 0x15555, 16);
            let end = up
                .iter()
                .position(|ld| !ld.level && ld.duration_us == 1500)
                .unwrap();
            up[..=end].to_vec()
        };
        let cat = |parts: &[&[LevelDuration]]| -> Vec<LevelDuration> {
            parts.iter().flat_map(|p| p.iter().copied()).collect()
        };
        let mut dec = RenaultV1ArfDecoder::new();
        assert!(decode_all(&mut dec, &burst).is_empty(), "single burst");
        let mut dec = RenaultV1ArfDecoder::new();
        assert_eq!(
            decode_all(&mut dec, &cat(&[&burst, &burst])).len(),
            1,
            "two bursts"
        );
        let mut dec = RenaultV1ArfDecoder::new();
        assert!(
            decode_all(&mut dec, &cat(&[&burst, &other_burst, &burst])).is_empty(),
            "an intervening frame replaces the pending candidate"
        );
        let mut dec = RenaultV1ArfDecoder::new();
        assert!(decode_all(&mut dec, &burst).is_empty());
        dec.reset();
        assert!(
            decode_all(&mut dec, &burst).is_empty(),
            "reset drops the pending candidate"
        );
        assert_eq!(decode_all(&mut dec, &burst).len(), 1);
        // Typed: one burst is enough.
        let (t_data, t_key2, t_pre) = (C_VECTORS[1].0, C_VECTORS[1].1, C_VECTORS[1].2);
        let up = upload(t_data, t_key2, t_pre);
        let end = up
            .iter()
            .position(|ld| !ld.level && ld.duration_us == 1500)
            .unwrap();
        assert_eq!(
            decode_all(&mut RenaultV1ArfDecoder::new(), &up[..=end]).len(),
            1
        );
    }

    /// Typed frames need their type's preamble length; Dynamic frames accept 12/16/18/20; other
    /// lengths never decode.
    #[test]
    fn preamble_length_rules() {
        for (data, key2, pre, tag, ..) in C_VECTORS {
            for len in [10u8, 12, 14, 16, 18, 20, 22] {
                let sig: Vec<LevelDuration> =
                    if len % 2 == 0 && (12..=20).contains(&len) && len != 14 {
                        upload(data, key2, len)
                    } else {
                        continue;
                    };
                let n = decode_all(&mut RenaultV1ArfDecoder::new(), &sig.repeat(2)).len();
                let expected = if tag.starts_with("Dynamic") || len == pre {
                    1
                } else {
                    0
                };
                assert_eq!(n, expected, "{tag} with a {len}-bit preamble");
            }
        }
    }

    /// The checksum/button gates: tag 0x13 (Renault V0's), hi2xor 1/2, buttons outside the
    /// whitelist and a zero serial are rejected (Dynamic path), as are typed buttons outside the
    /// type's range unless the Dynamic rules accept them.
    #[test]
    fn gates_reject_invalid_frames() {
        let key2 = 0x2_1234;
        let rejected = [
            ("tag 0x13", make(0x123456, 0x06, 1, key2, 0x13, 0)),
            ("hi2xor 1", make(0x123456, 0x06, 1, key2, 0x2A, 1)),
            ("hi2xor 2", make(0x123456, 0x46, 1, key2, 0x2A, 2)),
            ("button 0x13", make(0x123456, 0x13, 1, key2, 0x2A, 0)),
            ("button 0x80", make(0x123456, 0x80, 1, key2, 0x2A, 0)),
            ("serial 0", make(0, 0x06, 1, key2, 0x2A, 0)),
            (
                "type 0x0C, hi2xor 3",
                make(0x123456, 0x0C, 1, key2, 0x0C, 1),
            ),
        ];
        for (what, data) in rejected {
            assert!(
                RenaultV1ArfDecoder::classify_frame(data, key2).is_none(),
                "{what}"
            );
            let sig: Vec<LevelDuration> = upload(data, key2, 16).repeat(2);
            assert!(
                decode_all(&mut RenaultV1ArfDecoder::new(), &sig).is_empty(),
                "{what}"
            );
            assert!(RenaultV1ArfDecoder::new()
                .encode(&signal_for(data, key2, 16), 0x01)
                .is_none());
        }
        // Type 0x04 tag with a 0x4x button: not type 0x04, but a valid Dynamic frame.
        let data = make(0x123456, 0x46, 1, key2, 0x04, 0);
        let a = RenaultV1ArfDecoder::classify_frame(data, key2).unwrap();
        assert_eq!((a.frame_type, a.type_tag), (FrameType::Dynamic, 0x04));
    }

    /// Mutual exclusivity with Renault V0 (claimed by ARF, which registers V1 right after V0):
    /// a type-0x13 frame from KAT's Renault V0 encoder decodes as Renault V0 only and this
    /// decoder rejects it on its own; ARF's frames of every type decode as Renault V1 (ARF) only
    /// and Renault V0 rejects them; the same 0x13 frame sent with ARF's emitter is Renault V0's.
    #[test]
    fn renault_v0_and_arf_are_mutually_exclusive() {
        let reg = ProtocolRegistry::new();
        let v0 = reg.get("Renault V0").unwrap();
        let key2 = 0x0_8421;
        let v0_frame = make(0x2468AC, 0x06, 0x31, key2, 0x13, 0);
        let v0_sig = DecodedSignal {
            serial: Some(0x2468AC),
            button: Some(0x06),
            counter: Some(0x31),
            crc_valid: true,
            data: v0_frame,
            data_count_bit: 82,
            encoder_capable: true,
            extra: Some(key2 as u64),
            protocol_display_name: None,
        };
        let v0_tx = v0.encode(&v0_sig, 0x40).expect("Renault V0 replays it");
        for freq in [315_000_000u32, 433_920_000, 868_350_000] {
            let names = registry_names(&v0_tx, freq);
            assert!(
                !names.is_empty() && names.iter().all(|n| n == "Renault V0"),
                "{names:?}"
            );
            let arf_tx = upload(v0_frame, key2, 16).repeat(2);
            let names = registry_names(&arf_tx, freq);
            assert!(
                !names.is_empty() && names.iter().all(|n| n == "Renault V0"),
                "{names:?}"
            );
        }
        assert!(decode_all(&mut RenaultV1ArfDecoder::new(), &v0_tx).is_empty());

        for (data, key2, pre, tag, ..) in C_VECTORS {
            let tx = RenaultV1ArfDecoder::new()
                .encode(&signal_for(data, key2, pre), 0x01)
                .unwrap();
            for freq in [315_000_000u32, 433_920_000, 868_350_000] {
                let names = registry_names(&tx, freq);
                assert!(!names.is_empty(), "{tag} at {freq}");
                let want = format!("Renault V1 (ARF) [{tag}]");
                assert!(
                    names.iter().all(|n| *n == want),
                    "{tag} at {freq}: {names:?}"
                );
            }
            let mut v0_dec = crate::protocols::ProtocolRegistry::new();
            let v0_only: Vec<_> = v0_dec
                .process_signal_stream(&tx, 433_920_000)
                .into_iter()
                .filter(|r| r.0 == "Renault V0")
                .collect();
            assert!(v0_only.is_empty(), "{tag}");
        }
    }

    /// Neighbouring air formats stay with their own decoders in the registry: ProtoPirate's HITAG2
    /// Renault V1 (1500/1000 µs header, 104 bits) and Fiat V1 type B (850 µs lead, 100/200 µs
    /// Manchester, inside this decoder's TE windows).
    ///
    /// On its own this decoder — exactly like the compiled ARF C, which reports the same frame —
    /// reads two consecutive HITAG2 long frames as a "Dynamic" frame (tag 0x17, 16-bit preamble).
    /// In the registry ProtoPirate's Renault V1 completes each long frame no later than this
    /// decoder's burst end, wins by order, and the reset after its hit drops the pending Dynamic
    /// candidate, so the HITAG2 frames stay "Renault V1".
    #[test]
    fn hitag2_renault_and_fiat_v1_frames_are_not_claimed() {
        let reg = ProtocolRegistry::new();
        let r1 = reg.get("Renault V1").unwrap();
        let fiat = reg.get("Fiat V1").unwrap();
        let payload: [u8; 11] = [
            0x5E, 0xED, 0x12, 0x34, 0x1B, 0x0E, 0x96, 0x95, 0x69, 0x6A, 0x80,
        ];
        let mut extra = 0u64;
        for b in &payload[6..] {
            extra = (extra << 8) | *b as u64;
        }
        let data = (1u64 << 48) | payload[..6].iter().fold(0u64, |a, &b| (a << 8) | b as u64);
        let r1_sig = DecodedSignal {
            serial: None,
            button: None,
            counter: None,
            crc_valid: true,
            data,
            data_count_bit: 104,
            encoder_capable: true,
            extra: Some(extra),
            protocol_display_name: None,
        };
        let r1_tx = r1.encode(&r1_sig, 0x01).expect("Renault V1 replays it");
        for freq in [315_000_000u32, 433_920_000, 868_350_000] {
            let names = registry_names(&r1_tx, freq);
            assert!(
                !names.is_empty() && names.iter().all(|n| n == "Renault V1"),
                "{names:?}"
            );
        }
        let standalone = decode_all(&mut RenaultV1ArfDecoder::new(), &r1_tx);
        assert_eq!(standalone.len(), 1);
        assert_eq!(
            standalone[0].data, 0xBB44_8D06_C3A5_A55A,
            "same frame as the C"
        );
        assert_eq!(standalone[0].extra, Some(0x16A80 | 16 << 24));
        assert_eq!(
            standalone[0].protocol_display_name.as_deref(),
            Some("Renault V1 (ARF) [Dynamic 0x17]")
        );

        for variant in [0u64, 1] {
            let raw: [u8; 13] = [
                0x00, 0x01, 0x12, 0x34, 0x56, 0x78, 0x44, 0x06, 0x76, 0xAE, 0x31, 0xAA, 0x09,
            ];
            let fiat_sig = DecodedSignal {
                serial: None,
                button: None,
                counter: None,
                crc_valid: true,
                data: u64::from_be_bytes(raw[..8].try_into().unwrap()),
                data_count_bit: 104,
                encoder_capable: true,
                extra: Some(
                    raw[8..].iter().fold(0u64, |a, &b| (a << 8) | b as u64) | variant << 40,
                ),
                protocol_display_name: None,
            };
            let tx = fiat.encode(&fiat_sig, 0x01).expect("Fiat V1 transmits");
            let names = registry_names(&tx, 433_920_000);
            assert!(
                !names.is_empty() && names.iter().all(|n| n == "Fiat V1"),
                "{names:?}"
            );
            assert!(decode_all(&mut RenaultV1ArfDecoder::new(), &tx).is_empty());
        }
    }

    /// Reset (as in the C): the pending Dynamic candidate and key2 are cleared, the last data is
    /// kept — so after a registry reset the same frame is reported again, except (C quirk) when
    /// its key2 is 0.
    #[test]
    fn reset_semantics_follow_c() {
        for key2 in [0x1_2345u32, 0] {
            let data = make(0x13579B, 0x07, 0x42, key2, 0x04, 0);
            let up = upload(data, key2, 16);
            let mut dec = RenaultV1ArfDecoder::new();
            assert_eq!(decode_all(&mut dec, &up).len(), 1, "3 bursts, deduplicated");
            dec.reset();
            let again = decode_all(&mut dec, &up).len();
            assert_eq!(again, if key2 == 0 { 0 } else { 1 }, "key2 {key2:#X}");
        }
    }

    /// A decodable frame whose upload overflows ARF's 600-level buffer (the compiled C refuses to
    /// transmit FFFFFF04000000F8 / key2 0 / 20-bit preamble: status -1) is reported as not
    /// encodable, and encode() refuses it.
    #[test]
    fn upload_overflow_is_not_encodable() {
        let (data, key2) = (0xFFFF_FF04_0000_00F8u64, 0u32);
        let mut attempt = RenaultV1ArfDecoder::classify_frame(data, key2).unwrap();
        assert_eq!(attempt.frame_type, FrameType::Dynamic);
        attempt.preamble_bits = 20;
        assert!(RenaultV1ArfDecoder::build_upload(data, key2, 20).is_none());
        let d = RenaultV1ArfDecoder::decoded_signal(&attempt);
        assert!(!d.encoder_capable);
        assert!(RenaultV1ArfDecoder::new().encode(&d, 0x01).is_none());
        // With a 12-bit preamble the same frame fits.
        assert!(RenaultV1ArfDecoder::build_upload(data, key2, 12).is_some());
    }

    /// Cross-protocol guard (as `foreign_encoder_output_is_never_claimed` does for Mercedes/Opel):
    /// no other registry encoder's output, at any of its frequencies, decodes as Renault V1 (ARF).
    /// Renault V0 shares the wire and its encoder replays whatever 82 bits it is given, so it is
    /// fed a valid type-0x13 frame (a non-0x13 "V0" frame is by definition an ARF frame; Renault V0
    /// never decodes one) — the 0x13 exclusivity is checked in its own test above.
    #[test]
    fn foreign_encoder_output_is_never_claimed() {
        use std::panic::{catch_unwind, AssertUnwindSafe};
        let reg0 = ProtocolRegistry::new();
        let v0_key2 = 0x0_8421;
        let v0_data = make(0x2468AC, 0x06, 0x31, v0_key2, 0x13, 0);
        for name in reg0.list_protocols() {
            if name == "Renault V1 (ARF)" {
                continue;
            }
            let dec = reg0.get(name).unwrap();
            if !dec.supports_encoding() {
                continue;
            }
            let cases: Vec<(u64, u64)> = match name {
                "Renault V0" => vec![(v0_data, v0_key2 as u64)],
                "Ford V1" => vec![(0xA5C3_1E7B_92D4_6F08, (1 << 40) | (0x5A << 32) | 0x1234)],
                "VAG" => [0x0001u64, 0x0002, 0x0003, 0x0004, 0x0104]
                    .iter()
                    .map(|&e| (0xA5C3_1E7B_92D4_6F08, e))
                    .collect(),
                _ => vec![
                    (0xA5C3_1E7B_92D4_6F08, 0x1234_5678_9ABC),
                    (0x0123_4567_89AB_CDEF, 0x0F0F_0F0F),
                ],
            };
            for (data, extra) in cases {
                let src = DecodedSignal {
                    serial: Some(0x0012_3456),
                    button: Some(0x01),
                    counter: Some(5),
                    crc_valid: true,
                    data,
                    data_count_bit: dec.timing().min_count_bit,
                    encoder_capable: true,
                    extra: Some(extra),
                    protocol_display_name: None,
                };
                for btn in [0x01u8, 0x02] {
                    let Ok(Some(wire)) = catch_unwind(AssertUnwindSafe(|| dec.encode(&src, btn)))
                    else {
                        continue;
                    };
                    for &f in dec.supported_frequencies() {
                        let mut reg = ProtocolRegistry::new();
                        for (got, _, _) in reg.process_signal_stream(&wire, f) {
                            assert!(
                                !got.starts_with("Renault V1 (ARF)"),
                                "{name} (data {data:#X}, btn {btn}) at {f} Hz decoded as {got}"
                            );
                        }
                    }
                }
            }
        }
    }

    /// Display names carry the type; the registry resolves them (and "Renault V1" stays
    /// ProtoPirate's HITAG2 decoder).
    #[test]
    fn registry_resolves_display_names() {
        let reg = ProtocolRegistry::new();
        for name in [
            "Renault V1 (ARF) [0x0C]",
            "Renault V1 (ARF) [Dynamic 0x2A]",
            "renault v1 (arf)",
        ] {
            assert_eq!(
                reg.get(name).map(|d| d.name()),
                Some("Renault V1 (ARF)"),
                "{name}"
            );
        }
        assert_eq!(reg.get("Renault V1").map(|d| d.name()), Some("Renault V1"));
        assert_eq!(
            crate::protocols::native_button_name("Renault V1 (ARF) [0x0C]", 0xC6),
            Some("Lock")
        );
        assert_eq!(
            crate::protocols::native_button_name("Renault V1 (ARF)", 0x4A),
            Some("Unlock")
        );
    }
}
