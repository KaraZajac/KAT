//! Renault V0 protocol decoder/encoder
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/renault_v0.c` and
//! `renault_v0.h` (primary — the decoder, model and encoder are ported from it line by line).
//! doctor-firmware's `lib/subghz/protocols/renault_v0.c` is the same code (only the Flipper UI
//! glue differs); it was used solely to confirm the TX counter policy (see Encoder). Flipper-ARF's
//! copy is ProtoPirate's pre-1287d10 decoder ("move newer Renault variants into Renault V1": its
//! extra checksum types 04/0C/1A/3B/3F belong to Renault V1 now, and ARF itself only accepts type
//! 13 in V0) with a different frame extraction; KAT adds that extraction as a fallback (see
//! Frame), so it decodes exactly the union of ProtoPirate's and ARF's frames (checked against
//! both compiled C decoders on ~1400 randomized bursts). ARF's two extra timing profiles only add
//! 53..=55µs short pulses; not adopted (combined with ProtoPirate's extraction they would decode
//! frames neither reference decodes). AM (OOK), 315 / 433.92 / 868.35 MHz (C flags 315|433|868).
//!
//! **Timing / classification** (`renault_v0_classify_event`): te_short=125µs, te_long=250µs,
//! te_delta=69µs, evaluated with the C's exact unsigned comparisons, which works out to
//! short = 56..=180µs, long = 181..=319µs, anything else ends the run. A HIGH pulse ≥800µs is the
//! sync that (re)starts the data step.
//!
//! **Manchester**: the C does NOT call Flipper's `manchester_advance`; it inlines its own lookup
//! `renault_v0_decoder_state_table = {0x01, 0x91, 0x9B, 0xFB}` — numerically Flipper's transition
//! table — with Flipper's state numbering (Start1=0, Mid1=1, Mid0=2, Start0=3) and event codes
//! (ShortLow=0, ShortHigh=2, LongLow=4, LongHigh=6; `level ? Low : High`). One difference from
//! Flipper: when the looked-up state equals the current state the event is simply ignored (Flipper
//! would reset to Mid1). Entering Mid1 emits a 1, entering Mid0 emits a 0. This module ports that
//! inline machine literally (it does not use KAT's shared `ManchesterState` table).
//!
//! **Frame** (82 bits): decoded bits are buffered (max 110) from the sync. On a run end (or when
//! more than 109 bits are buffered) `renault_v0_decode_candidate` needs >81 bits, counts the
//! leading 1s as preamble (must be ≥10, and ≥82 bits must follow), then takes the next 64 bits as
//! the key and the following 18 bits as `Key2`. Because every leading 1 is absorbed as preamble,
//! the key's MSB (serial bit 23) must be 0 for this reading. Flipper-ARF instead reads the run
//! tail-anchored (`renault_v0_attempt_at_offset`): the last 82 bits, after exactly 16 preamble
//! bits, with type-13 buttons 0x06/0x0A only; KAT tries that when ProtoPirate's reading fails, so
//! frames with serial bit 23 = 1 decode too (as in ARF).
//! - key[63:40] serial (24-bit), key[39:32] button (0x05 Trunk, 0x06 Lock, 0x0A Unlock),
//!   key[31:24] counter (8-bit), key[23:0] + Key2[17:0] = 42 parity bits.
//!
//! **Packing**: `data` = the 64-bit key (first 64 wire bits), `data_count_bit` = 82, `extra` =
//! Key2 (18 bits, right-aligned). `button` keeps the Renault code (as the C's `generic.btn`).
//!
//! **Validation / gating** (`renault_v0_type13_valid`, the C's emission gate): checksum = XOR of
//! the 8 key bytes and Key2[17:10] and Key2[9:2]; C1 requires `checksum & 0x3F == 0x13`, C2
//! requires `Key2 & 3 == checksum >> 6`, and the button must be 0x05/0x06/0x0A. Together with the
//! ≥800µs sync, ≥10-bit preamble and 82-bit structure this is an 8-bit check plus a 3/256 button
//! whitelist, so Renault V0 does not false-match. The C also displays an `IC` flag
//! (`renault_v0_model_matches`: does the frame equal the algebraic key model rebuilt from serial /
//! button / counter?) but never gates on it, and its encoder explicitly supports frames that fail
//! it (replayed verbatim) — so KAT does not gate on IC either; it only selects the TX path.
//!
//! **Encoder** (`renault_v0_build_upload` + encoder deserialize + ProtoPirate's emulate scene):
//! - "Rolling" frames (IC holds): the key is rebuilt with `renault_v0_build_key` from the serial,
//!   the requested button (Lock→0x06, Unlock→0x0A, Trunk→0x05, anything else keeps the captured
//!   code) and the counter **+1 (mod 256)** — ProtoPirate's emulate scene bumps `Cnt` that way
//!   before each TX and doctor-firmware's encoder bumps it itself. If the rebuilt frame fails the
//!   type-13 checksum the C refuses to transmit; KAT returns `None` likewise. (For a given fob the
//!   model's checksum only closes for Lock/Unlock or for Trunk, not both.)
//! - Other frames: the captured key + Key2 are replayed unchanged (button/counter ignored).
//! - Waveform: 3 bursts of [HIGH 1000µs sync, 16 preamble 1-bits, 82 data bits (key MSB-first then
//!   Key2), closing HIGH 125µs if the last half-bit was low, LOW 25000µs (last burst: 250µs)],
//!   encoded with the C's two-state emitter and same-level merging (`pp_emit_merge`). C repeat = 1.
//!
//! **Key files** (`subghz_protocol_decoder_renault_v0_serialize` / `_deserialize`): KAT writes
//! ProtoPirate's lines — Protocol "Renault V0", Bit 82, Key = the 64-bit key, then Serial, Btn,
//! Cnt, Key2 (the 18-bit Key2, decimal) and `Disp: Renault V0 - Trunk|Lock|Unlock|?`. Loading
//! follows the C: Bit 82, a Key2 line and the type-13 gate are required, the fields come from
//! the Key (Serial/Btn/Cnt are not read). doctor-firmware writes the same Key/Key2; Flipper-ARF's
//! older Renault V0 adds Preamble/Rolling lines (ignored here) and also loads only type 13.

use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Renault V0",
    decoder: "Renault V0",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "Renault",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// Renault V0 buttons.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x06 => "Lock",
        0x0A => "Unlock",
        0x05 => "Trunk",
        _ => "Other",
    })
}

const TE_SHORT: u32 = 125; // 0x7D
const TE_LONG: u32 = 250; // 0xFA
const TE_DELTA: u32 = 69; // 0x45
const SYNC_MIN_US: u32 = 800; // 0x320

/// Key (64) + Key2 (18) bits.
const FRAME_BITS: usize = 82; // 0x52
const KEY_BITS: usize = 64;
const KEY2_BITS: usize = 18;
const KEY2_MASK: u64 = (1 << KEY2_BITS) - 1;
/// decode_candidate requires `bit_count > 0x51` and `bit_count - preamble > 0x51`.
const CANDIDATE_MIN_BITS: usize = 0x52;
/// decode_candidate requires `preamble > 9`.
const PREAMBLE_MIN_ONES: usize = 10;
/// Flipper-ARF: a type-13 frame read tail-anchored must follow exactly 16 preamble bits.
const ARF_TYPE13_PREAMBLE_BITS: usize = 16;
/// The feed decodes + resets once more than 0x6D bits are buffered.
const DECODER_BIT_LIMIT: usize = 0x6D;
const DECODED_BITS_MAX: usize = 0x70;

// Encoder (renault_v0_build_upload).
const TX_SYNC_HIGH_US: u32 = 1000;
const TX_PREAMBLE_BITS: usize = 16;
const TX_BURST_COUNT: usize = 3;
const TX_INTER_BURST_US: u32 = 25_000; // 0x61A8
const TX_FINAL_LOW_US: u32 = 250;

/// renault_v0_decoder_state_table (Flipper's transition table, Flipper state numbering).
const STATE_TABLE: [u8; 4] = [0x01, 0x91, 0x9B, 0xFB];
const STATE_MID1: u8 = 1;
const STATE_MID0: u8 = 2;
// Event codes (Flipper ManchesterEvent values).
const EVENT_SHORT_LOW: u8 = 0;
const EVENT_SHORT_HIGH: u8 = 2;
const EVENT_LONG_LOW: u8 = 4;
const EVENT_LONG_HIGH: u8 = 6;

// Renault V0 button codes.
const BTN_TRUNK: u8 = 0x05;
const BTN_LOCK: u8 = 0x06;
const BTN_UNLOCK: u8 = 0x0A;

// KAT generic button codes.
const GENERIC_LOCK: u8 = 0x01;
const GENERIC_UNLOCK: u8 = 0x02;
const GENERIC_TRUNK: u8 = 0x04;

/// renault_v0_matrix: 42 rows of a 64-bit (low, high) mask over the 64 key-model monomials.
const MATRIX: [(u32, u32); 42] = [
    (0x0000_0001, 0x0000_0000),
    (0x0400_0029, 0x0000_0000),
    (0x0000_001B, 0x0000_0000),
    (0x0000_0000, 0x0000_0000),
    (0x0000_0001, 0x0000_0000),
    (0x0522_0124, 0x0000_0000),
    (0x0000_0001, 0x0000_0000),
    (0x0008_8410, 0x0000_0000),
    (0x6013_2D1D, 0x0000_0000),
    (0x6017_0F87, 0x0000_1004),
    (0x0000_0000, 0x0000_0000),
    (0x0020_00A9, 0x0000_0000),
    (0x2086_3E01, 0x0000_100C),
    (0x24BB_3755, 0x0000_0004),
    (0x6401_99A4, 0x0000_0004),
    (0x2422_5C43, 0x0000_1004),
    (0x6078_86F1, 0x0000_100C),
    (0x6007_A101, 0x0000_000C),
    (0x6667_2A10, 0x0000_0004),
    (0x4651_623F, 0x0000_1008),
    (0x4338_0BBF, 0x0000_1008),
    (0x2023_7F84, 0x0000_1000),
    (0x4245_755E, 0x0000_1008),
    (0x60AA_F581, 0x0000_0004),
    (0x2272_2DAD, 0x0000_000C),
    (0x27C6_17F7, 0x0000_0000),
    (0x46DE_8F1B, 0x0000_000C),
    (0x231D_EC51, 0x0000_0000),
    (0x03AC_AA0B, 0x0000_0008),
    (0x22D2_BF81, 0x0000_0004),
    (0x626E_F6AE, 0x0000_100C),
    (0x4044_1F95, 0x0000_000C),
    (0x0000_0001, 0x0000_0000),
    (0x0000_0000, 0x0000_0000),
    (0x20B9_A590, 0x0000_0008),
    (0x656C_8E86, 0x0000_1008),
    (0x6012_9F96, 0x0000_000C),
    (0x2368_F667, 0x0000_1000),
    (0x442A_1A5C, 0x0000_0000),
    (0x04C4_3242, 0x0000_100C),
    (0x2219_8640, 0x0000_1000),
    (0x23D6_B958, 0x0000_1008),
];

#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Data,
}

/// Renault V0 decoder (matches SubGhzProtocolDecoderRenaultV0).
pub struct RenaultV0Decoder {
    step: DecoderStep,
    manchester_state: u8,
    decoded_bits: [u8; DECODED_BITS_MAX],
    decoded_bit_count: usize,
}

impl RenaultV0Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            manchester_state: STATE_MID1,
            decoded_bits: [0; DECODED_BITS_MAX],
            decoded_bit_count: 0,
        }
    }

    /// renault_v0_parse_fields: (serial 24-bit, button, counter).
    fn parse_fields(data: u64) -> (u32, u8, u8) {
        ((data >> 40) as u32, (data >> 32) as u8, (data >> 24) as u8)
    }

    fn button_valid(button: u8) -> bool {
        matches!(button, BTN_TRUNK | BTN_LOCK | BTN_UNLOCK)
    }

    /// renault_v0_checksum: XOR of the 8 key bytes, Key2[17:10] and Key2[9:2].
    fn checksum(data: u64, key2: u32) -> u8 {
        let mut checksum = data.to_be_bytes().iter().fold(0u8, |acc, &b| acc ^ b);
        checksum ^= ((key2 >> 10) & 0xFF) as u8;
        checksum ^= ((key2 >> 2) & 0xFF) as u8;
        checksum
    }

    /// renault_v0_type13_valid: C1 (`checksum & 0x3F == 0x13`) + C2 (`Key2 & 3 == checksum >> 6`)
    /// + button whitelist. This is the C's emission gate.
    fn type13_valid(data: u64, key2: u32) -> bool {
        let checksum = Self::checksum(data, key2);
        if checksum & 0x3F != 0x13 {
            return false;
        }
        if key2 & 0x03 != u32::from(checksum >> 6) {
            return false;
        }
        Self::button_valid((data >> 32) as u8)
    }

    /// renault_v0_build_key: rebuild key + Key2 from serial / button / counter. The 7 model
    /// variables are (button == Unlock) and counter bits 0..5; the 64 monomials (constant, 7
    /// linear, 21 quadratic, 35 cubic) select matrix columns, each matrix row yields one parity
    /// bit; counter bits 6/7 flip parity bits 41/40.
    fn build_key(serial: u32, button: u8, counter: u8) -> (u64, u32) {
        let mut vars = [0u8; 7];
        vars[0] = u8::from(button == BTN_UNLOCK);
        for bit in 0..6 {
            vars[bit + 1] = (counter >> bit) & 1;
        }

        let mut mask: u64 = 1; // monomial 0: constant term
        let mut mask_bit = 1u32;
        for &v in &vars {
            if v != 0 {
                mask |= 1u64 << mask_bit;
            }
            mask_bit += 1;
        }
        for i in 0..6 {
            for j in (i + 1)..7 {
                if vars[i] & vars[j] != 0 {
                    mask |= 1u64 << mask_bit;
                }
                mask_bit += 1;
            }
        }
        for i in 0..6 {
            for j in (i + 1)..7 {
                for k in (j + 1)..7 {
                    if vars[i] & vars[j] & vars[k] != 0 {
                        mask |= 1u64 << mask_bit;
                    }
                    mask_bit += 1;
                }
            }
        }
        debug_assert_eq!(mask_bit, 64);

        let mut parity_bits = [0u8; 42];
        for (row, &(low, high)) in MATRIX.iter().enumerate() {
            let row_mask = (u64::from(high) << 32) | u64::from(low);
            parity_bits[row] = ((row_mask & mask).count_ones() & 1) as u8;
        }
        if counter & 0x40 != 0 {
            parity_bits[41] ^= 1;
        }
        if counter >> 7 != 0 {
            parity_bits[40] ^= 1;
        }

        let mut data_low = u32::from(counter) << 24;
        // C: `(serial << 8U) | button` on uint32_t (serial bits above 23 fall off).
        let data_high = (serial << 8) | u32::from(button);
        for (i, &p) in parity_bits.iter().take(24).enumerate() {
            if p != 0 {
                data_low |= 1 << (23 - i);
            }
        }
        let mut key2 = 0u32;
        for (i, &p) in parity_bits.iter().enumerate().skip(24) {
            if p != 0 {
                key2 |= 1 << (41 - i);
            }
        }
        ((u64::from(data_high) << 32) | u64::from(data_low), key2)
    }

    /// renault_v0_model_matches (the C's displayed-only `IC` check; selects the TX path).
    fn model_matches(data: u64, key2: u32) -> bool {
        let (serial, button, counter) = Self::parse_fields(data);
        Self::build_key(serial, button, counter) == (data, key2)
    }

    /// renault_v0_classify_event, with the C's exact unsigned comparisons. Returns the Flipper
    /// event code (`level ? Low : High`).
    fn classify_event(duration: u32, level: bool) -> Option<u8> {
        let short_event = if level {
            EVENT_SHORT_LOW
        } else {
            EVENT_SHORT_HIGH
        };
        let long_event = if level {
            EVENT_LONG_LOW
        } else {
            EVENT_LONG_HIGH
        };
        if duration < TE_SHORT {
            if TE_SHORT - duration > TE_DELTA {
                return None;
            }
            return Some(short_event);
        }
        if duration < TE_LONG {
            let short_delta = duration - TE_SHORT;
            let long_inv_delta = TE_LONG - duration;
            if short_delta <= TE_DELTA {
                return Some(if long_inv_delta > TE_DELTA {
                    short_event
                } else {
                    long_event
                });
            }
            if long_inv_delta <= TE_DELTA {
                return Some(long_event);
            }
            return None;
        }
        if duration - TE_LONG > TE_DELTA {
            return None;
        }
        Some(long_event)
    }

    fn start_data(&mut self) {
        self.step = DecoderStep::Data;
        self.decoded_bit_count = 0;
        self.manchester_state = STATE_MID1;
    }

    /// renault_v0_decode_candidate: preamble strip, key/Key2 extraction and the type-13 gate.
    /// The 82-bit frame (key, Key2) starting at `offset` of the run, if it passes the type-13 gate.
    fn frame_at(bits: &[u8], offset: usize) -> Option<DecodedSignal> {
        let payload = bits.get(offset..offset + FRAME_BITS)?;
        let data = payload[..KEY_BITS]
            .iter()
            .fold(0u64, |acc, &b| (acc << 1) | u64::from(b & 1));
        let key2 = payload[KEY_BITS..]
            .iter()
            .fold(0u32, |acc, &b| (acc << 1) | u32::from(b & 1));
        Self::type13_valid(data, key2).then(|| Self::build_signal(data, key2))
    }

    /// ProtoPirate `renault_v0_decode_candidate`: the leading 1s are the preamble (≥ 10) and the
    /// frame follows them. A frame whose key MSB (serial bit 23) is 1 is absorbed into that
    /// preamble and never decodes there, so (union with Flipper-ARF's `renault_v0_decode_candidate`
    /// \+ `renault_v0_attempt_at_offset`) a run is also read tail-anchored — its last 82 bits, with
    /// exactly the 16-bit type-13 preamble before them (`renault_v0_type_preamble_bits_valid`) —
    /// under ARF's type-13 gate: the same checksum, buttons 0x06/0x0A only
    /// (`renault_v0_type_button_valid`; ARF does not accept Trunk 0x05 as type 13).
    fn decode_candidate(&self) -> Option<DecodedSignal> {
        let bit_count = self.decoded_bit_count;
        if bit_count < CANDIDATE_MIN_BITS {
            return None;
        }
        let bits = &self.decoded_bits[..bit_count];
        let preamble = bits.iter().take_while(|&&b| b == 1).count();
        if preamble >= PREAMBLE_MIN_ONES && bit_count - preamble >= CANDIDATE_MIN_BITS {
            if let Some(signal) = Self::frame_at(bits, preamble) {
                return Some(signal);
            }
        }
        if bit_count - FRAME_BITS == ARF_TYPE13_PREAMBLE_BITS {
            return Self::frame_at(bits, ARF_TYPE13_PREAMBLE_BITS)
                .filter(|d| matches!(d.button, Some(BTN_LOCK | BTN_UNLOCK)));
        }
        None
    }

    fn build_signal(data: u64, key2: u32) -> DecodedSignal {
        let (serial, button, counter) = Self::parse_fields(data);
        DecodedSignal {
            serial: Some(serial),
            button: Some(button),
            counter: Some(u16::from(counter)),
            // C1 + C2 (the type-13 checksum) are validated before emission.
            crc_valid: true,
            data,
            data_count_bit: FRAME_BITS,
            encoder_capable: true,
            extra: Some(u64::from(key2)),
            protocol_display_name: None,
        }
    }

    /// Map KAT's generic button to a Renault V0 code; unknown codes keep the captured button.
    fn map_button(button: u8, captured: u8) -> u8 {
        match button {
            GENERIC_LOCK => BTN_LOCK,
            GENERIC_UNLOCK => BTN_UNLOCK,
            GENERIC_TRUNK => BTN_TRUNK,
            _ => captured,
        }
    }

    /// pp_emit_merge: append a level, merging with the previous entry when the level matches.
    fn emit(signal: &mut Vec<LevelDuration>, level: bool, duration: u32) {
        if let Some(last) = signal.last_mut() {
            if last.level == level {
                *last = LevelDuration::new(level, last.duration_us + duration);
                return;
            }
        }
        signal.push(LevelDuration::new(level, duration));
    }

    /// renault_v0_emit_decoded_bit: two-state Manchester emitter (state 1 = line high after a
    /// 1-bit, state 2 = line low after a 0-bit).
    fn emit_decoded_bit(signal: &mut Vec<LevelDuration>, state: &mut u8, bit: bool) {
        if *state == 1 {
            if bit {
                Self::emit(signal, false, TE_SHORT);
                Self::emit(signal, true, TE_SHORT);
            } else {
                *state = 2;
                Self::emit(signal, false, TE_LONG);
            }
        } else if bit {
            *state = 1;
            Self::emit(signal, true, TE_LONG);
        } else {
            Self::emit(signal, true, TE_SHORT);
            Self::emit(signal, false, TE_SHORT);
        }
    }

    /// renault_v0_get_bit_msb82: bit `index` of key ‖ Key2, MSB first.
    fn frame_bit(data: u64, key2: u32, index: usize) -> bool {
        if index < KEY_BITS {
            (data >> (63 - index)) & 1 != 0
        } else {
            (key2 >> (FRAME_BITS - 1 - index)) & 1 != 0
        }
    }

    /// renault_v0_build_upload: 3 bursts of sync + preamble + 82 data bits + trailer.
    fn build_upload(data: u64, key2: u32) -> Vec<LevelDuration> {
        let mut signal = Vec::with_capacity(600);
        for burst in 0..TX_BURST_COUNT {
            Self::emit(&mut signal, true, TX_SYNC_HIGH_US);
            let mut state = 1u8;
            for _ in 0..TX_PREAMBLE_BITS {
                Self::emit_decoded_bit(&mut signal, &mut state, true);
            }
            for index in 0..FRAME_BITS {
                Self::emit_decoded_bit(&mut signal, &mut state, Self::frame_bit(data, key2, index));
            }
            if state == 2 {
                Self::emit(&mut signal, true, TE_SHORT);
            }
            let trailing_low = if burst + 1 < TX_BURST_COUNT {
                TX_INTER_BURST_US
            } else {
                TX_FINAL_LOW_US
            };
            Self::emit(&mut signal, false, trailing_low);
        }
        signal
    }
}

impl ProtocolDecoder for RenaultV0Decoder {
    fn name(&self) -> &'static str {
        "Renault V0"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: FRAME_BITS,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[315_000_000, 433_920_000, 868_350_000]
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.manchester_state = STATE_MID1;
        self.decoded_bit_count = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        if self.step == DecoderStep::Reset {
            if level && duration >= SYNC_MIN_US {
                self.start_data();
            }
            return None;
        }

        if self.decoded_bit_count > DECODER_BIT_LIMIT {
            let result = self.decode_candidate();
            self.step = DecoderStep::Reset;
            return result;
        }

        let event = match Self::classify_event(duration, level) {
            Some(event) => event,
            None => {
                let result = self.decode_candidate();
                if level && duration >= SYNC_MIN_US {
                    self.start_data();
                } else {
                    self.step = DecoderStep::Reset;
                }
                return result;
            }
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

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        let mut data = decoded.data;
        let mut key2 = (decoded.extra? & KEY2_MASK) as u32;
        // The C encoder refuses frames that fail the type-13 gate.
        if !Self::type13_valid(data, key2) {
            return None;
        }

        if Self::model_matches(data, key2) {
            // Rolling frame: re-mint with the requested button and the next counter.
            let (serial, captured_button, counter) = Self::parse_fields(data);
            let new_button = Self::map_button(button, captured_button);
            let (new_data, new_key2) = Self::build_key(serial, new_button, counter.wrapping_add(1));
            if !Self::type13_valid(new_data, new_key2) {
                return None;
            }
            data = new_data;
            key2 = new_key2;
        }
        // Non-model frames are replayed verbatim (the C ignores Btn/Cnt for them).

        Some(Self::build_upload(data, key2))
    }

    /// `subghz_protocol_decoder_renault_v0_deserialize` (see the module doc, Key files).
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != FRAME_BITS {
            return None;
        }
        let data = key.key_u64()?;
        let key2 = (u64::from(key.u32_field("Key2")?) & KEY2_MASK) as u32;
        Self::type13_valid(data, key2).then(|| Self::build_signal(data, key2))
    }

    /// `subghz_protocol_decoder_renault_v0_serialize` (see the module doc, Key files).
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        let data = decoded.data;
        let key2 = (decoded.extra? & KEY2_MASK) as u32;
        if decoded.data_count_bit != FRAME_BITS || !Self::type13_valid(data, key2) {
            return None;
        }
        let (serial, button, counter) = Self::parse_fields(data);
        let mut key = FlipperKey::for_protocol("Renault V0", FRAME_BITS, data);
        key.push_u32("Serial", serial);
        key.push_u32("Btn", button.into());
        key.push_u32("Cnt", counter.into());
        key.push_u32("Key2", key2);
        let name = match button {
            BTN_TRUNK => "Trunk",
            BTN_LOCK => "Lock",
            BTN_UNLOCK => "Unlock",
            _ => "?",
        };
        key.push("Disp", format!("Renault V0 - {name}"));
        Some(key)
    }
}

impl Default for RenaultV0Decoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Serial whose bytes XOR to 0x82: the key model then closes the type-13 checksum for
    /// Lock/Unlock at every counter (MSB 0 so the preamble does not absorb it).
    const SERIAL: u32 = 0x12_34_A4;

    fn decode_all(signal: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut decoder = RenaultV0Decoder::new();
        let mut out = Vec::new();
        for ld in signal {
            if let Some(d) = decoder.feed(ld.level, ld.duration_us) {
                out.push(d);
            }
        }
        // Trailing silence flushes the last burst (its upload ends on a 250µs low).
        if let Some(d) = decoder.feed(false, 50_000) {
            out.push(d);
        }
        out
    }

    fn model_frame(button: u8, counter: u8) -> DecodedSignal {
        let (data, key2) = RenaultV0Decoder::build_key(SERIAL, button, counter);
        assert!(RenaultV0Decoder::type13_valid(data, key2));
        RenaultV0Decoder::build_signal(data, key2)
    }

    #[test]
    fn classify_event_matches_c_ranges() {
        assert_eq!(RenaultV0Decoder::classify_event(55, true), None);
        assert_eq!(
            RenaultV0Decoder::classify_event(56, true),
            Some(EVENT_SHORT_LOW)
        );
        assert_eq!(
            RenaultV0Decoder::classify_event(180, false),
            Some(EVENT_SHORT_HIGH)
        );
        assert_eq!(
            RenaultV0Decoder::classify_event(181, true),
            Some(EVENT_LONG_LOW)
        );
        assert_eq!(
            RenaultV0Decoder::classify_event(319, false),
            Some(EVENT_LONG_HIGH)
        );
        assert_eq!(RenaultV0Decoder::classify_event(320, false), None);
    }

    /// A serial with MSB 1 is swallowed by ProtoPirate's leading-1s preamble rule; Flipper-ARF's
    /// tail-anchored reading (16 preamble bits + the last 82 bits) still decodes every burst.
    #[test]
    fn serial_msb_one_decodes_tail_anchored() {
        let serial = 0x92_B4_A4; // bytes XOR to 0x82 like SERIAL, MSB set
        let (data, key2) = RenaultV0Decoder::build_key(serial, BTN_UNLOCK, 0x21);
        assert!(RenaultV0Decoder::type13_valid(data, key2));
        let original = RenaultV0Decoder::build_signal(data, key2);
        let upload = RenaultV0Decoder::new().encode(&original, 0x00).unwrap();
        let decoded = decode_all(&upload);
        // Bursts 1 and 2 (the last one ends on a 250µs LOW that decodes as one more bit, so its
        // tail is no longer 16 + 82 bits — the same in the C; ARF also reports a repeat only once).
        assert_eq!(decoded.len(), 2);
        for d in &decoded {
            assert_eq!(d.serial, Some(serial));
            assert_eq!(d.button, Some(BTN_UNLOCK));
            assert_eq!(d.counter, Some(0x22));
        }
    }

    /// Key files: ProtoPirate's lines round-trip (model and non-model frames); the C's gates
    /// (Bit 82, Key2 present, type-13 checksum) reject bad files; a loaded key transmits.
    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::{assert_round_trip, key};
        let dec = RenaultV0Decoder::new();
        let decoded = decode_all(&dec.encode(&model_frame(BTN_LOCK, 0x41), GENERIC_LOCK).unwrap()).remove(0);
        let text = assert_round_trip(&dec, &decoded);
        let key2 = decoded.extra.unwrap();
        assert!(
            text.ends_with(&format!(
                "Protocol: Renault V0\nBit: 82\nKey: {}\nSerial: 1193124\nBtn: 6\nCnt: 66\n\
                 Key2: {key2}\nDisp: Renault V0 - Lock\n",
                decoded.data.to_be_bytes().map(|b| format!("{b:02X}")).join(" ")
            )),
            "{text}"
        );
        // A frame that is not the key model (replayed verbatim on TX) round-trips too.
        let other = RenaultV0Decoder::build_signal(decoded.data ^ (1 << 20), (key2 as u32) ^ (1 << 14));
        assert!(RenaultV0Decoder::type13_valid(other.data, other.extra.unwrap() as u32));
        assert_round_trip(&dec, &other);

        let good = key(&text);
        let mut no_key2 = good.clone();
        no_key2.fields.retain(|(n, _)| n != "Key2");
        assert!(dec.from_flipper_key(&no_key2).is_none(), "Key2 is required");
        let mut bad = no_key2.clone();
        bad.push_u32("Key2", (key2 as u32) ^ 1);
        assert!(dec.from_flipper_key(&bad).is_none(), "type-13 checksum");
        let mut bits = good.clone();
        bits.bits = 64;
        assert!(dec.from_flipper_key(&bits).is_none());

        let loaded = dec.from_flipper_key(&good).unwrap();
        let again = decode_all(&dec.encode(&loaded, GENERIC_UNLOCK).unwrap()).remove(0);
        assert_eq!((again.serial, again.button, again.counter), (Some(SERIAL), Some(BTN_UNLOCK), Some(0x43)));
    }

    /// Oracle vectors (see `fiat_v0::write_oracle_vector`): Lock and Unlock model frames.
    #[test]
    #[ignore]
    fn oracle_vectors() {
        for (name, button, generic) in [("renault_v0_1", BTN_LOCK, GENERIC_LOCK), ("renault_v0_2", BTN_UNLOCK, GENERIC_UNLOCK)] {
            let up = RenaultV0Decoder::new().encode(&model_frame(button, 0x41), generic).unwrap();
            crate::protocols::fiat_v0::write_oracle_vector(
                name,
                433_920_000,
                "FuriHalSubGhzPresetOok650Async",
                &up,
                &mut RenaultV0Decoder::new(),
            );
        }
    }

    #[test]
    fn key_model_closes_checksum_for_lock_and_unlock() {
        for counter in 0..=255u8 {
            for button in [BTN_LOCK, BTN_UNLOCK] {
                let (data, key2) = RenaultV0Decoder::build_key(SERIAL, button, counter);
                assert!(
                    RenaultV0Decoder::type13_valid(data, key2),
                    "btn {button:#x} cnt {counter}"
                );
                assert!(RenaultV0Decoder::model_matches(data, key2));
                assert_eq!(
                    RenaultV0Decoder::parse_fields(data),
                    (SERIAL, button, counter)
                );
                assert!(key2 < (1 << KEY2_BITS));
            }
        }
    }

    #[test]
    fn encode_decode_roundtrip_bumps_counter_and_restamps_button() {
        let captured = model_frame(BTN_LOCK, 0x41);
        let signal = RenaultV0Decoder::new()
            .encode(&captured, GENERIC_UNLOCK)
            .unwrap();
        // 3 bursts, each opened by the 1000µs sync.
        assert_eq!(
            signal
                .iter()
                .filter(|ld| ld.level && ld.duration_us == TX_SYNC_HIGH_US)
                .count(),
            3
        );
        let decoded = decode_all(&signal);
        assert_eq!(decoded.len(), 3, "one decode per burst");
        for d in &decoded {
            assert_eq!(d.serial, Some(SERIAL));
            assert_eq!(d.button, Some(BTN_UNLOCK));
            assert_eq!(d.counter, Some(0x42));
            assert_eq!(d.data_count_bit, FRAME_BITS);
            assert!(d.crc_valid && d.encoder_capable);
            let key2 = d.extra.unwrap() as u32;
            assert!(RenaultV0Decoder::model_matches(d.data, key2));
        }
    }

    #[test]
    fn encode_keeps_captured_button_for_unknown_command_and_wraps_counter() {
        let captured = model_frame(BTN_UNLOCK, 0xFF);
        let signal = RenaultV0Decoder::new().encode(&captured, 0x08).unwrap();
        let decoded = decode_all(&signal);
        assert_eq!(decoded[0].button, Some(BTN_UNLOCK));
        assert_eq!(decoded[0].counter, Some(0x00));
    }

    #[test]
    fn encode_refuses_button_whose_model_frame_fails_checksum() {
        // For this fob the model closes the checksum for Lock/Unlock but not for Trunk; the C
        // encoder errors out in that case and so does KAT.
        let captured = model_frame(BTN_LOCK, 0x10);
        assert!(RenaultV0Decoder::new()
            .encode(&captured, GENERIC_TRUNK)
            .is_none());
    }

    #[test]
    fn non_model_frame_is_replayed_verbatim() {
        // Flip two parity bits in the key and re-close the checksum through Key2 (C1/C2 still
        // hold, IC does not): the encoder must replay it unchanged.
        let base = model_frame(BTN_LOCK, 0x07);
        let data = base.data ^ 0x0000_0000_0000_0003;
        let mut key2 = base.extra.unwrap() as u32;
        let mut found = None;
        for k2_low in 0..(1u32 << 10) {
            let candidate = (key2 & !0x3FF) | k2_low;
            if RenaultV0Decoder::type13_valid(data, candidate)
                && !RenaultV0Decoder::model_matches(data, candidate)
            {
                found = Some(candidate);
                break;
            }
        }
        key2 = found.expect("a checksum-valid non-model Key2 exists");
        let frame = RenaultV0Decoder::build_signal(data, key2);
        let signal = RenaultV0Decoder::new()
            .encode(&frame, GENERIC_UNLOCK)
            .unwrap();
        let decoded = decode_all(&signal);
        assert_eq!(decoded.len(), 3);
        assert_eq!(decoded[0].data, data);
        assert_eq!(decoded[0].extra, Some(u64::from(key2)));
        assert_eq!(decoded[0].button, Some(BTN_LOCK));
        assert_eq!(decoded[0].counter, Some(0x07));
    }

    #[test]
    fn synthetic_upload_decodes() {
        let frame = model_frame(BTN_UNLOCK, 0x2C);
        let signal = RenaultV0Decoder::build_upload(frame.data, frame.extra.unwrap() as u32);
        let decoded = decode_all(&signal);
        assert_eq!(decoded.len(), 3);
        assert_eq!(decoded[0].data, frame.data);
        assert_eq!(decoded[0].extra, frame.extra);
        assert_eq!(decoded[0].counter, Some(0x2C));
    }

    #[test]
    fn corrupted_frame_is_rejected() {
        let frame = model_frame(BTN_LOCK, 0x33);
        let key2 = frame.extra.unwrap() as u32;
        // A single flipped key bit breaks the type-13 checksum.
        for bit in [0u32, 17, 30, 45] {
            let data = frame.data ^ (1u64 << bit);
            assert!(!RenaultV0Decoder::type13_valid(data, key2));
            assert!(
                decode_all(&RenaultV0Decoder::build_upload(data, key2)).is_empty(),
                "bit {bit}"
            );
        }
        // A non-whitelisted button is rejected even with a closed checksum.
        let data = (frame.data & !0x0000_00FF_0000_0000) | (0x16u64 << 32); // 0x06 ^ 0x10
        let key2_fixed = key2 ^ (0x10 << 2); // re-close C1 through Key2[9:2]
        assert_eq!(RenaultV0Decoder::checksum(data, key2_fixed) & 0x3F, 0x13);
        assert!(!RenaultV0Decoder::type13_valid(data, key2_fixed));
        assert!(decode_all(&RenaultV0Decoder::build_upload(data, key2_fixed)).is_empty());
    }

    #[test]
    fn requires_sync_pulse() {
        let frame = model_frame(BTN_LOCK, 0x01);
        let mut signal = RenaultV0Decoder::build_upload(frame.data, frame.extra.unwrap() as u32);
        // Shorten every sync below 800µs: nothing may decode.
        for ld in signal.iter_mut() {
            if ld.level && ld.duration_us >= SYNC_MIN_US {
                *ld = LevelDuration::new(true, 700);
            }
        }
        assert!(decode_all(&signal).is_empty());
    }
}
