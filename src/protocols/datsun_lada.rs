//! Datsun Lada protocol decoder/encoder
//!
//! Ported from doctor-firmware `lib/subghz/protocols/datsun_lada.c` / `datsun_lada.h`
//! (`SUBGHZ_PROTOCOL_DATSUN_LADA_NAME = "Datsun Lada"`), the only copy in REFERENCES (ProtoPirate and
//! Flipper-ARF have no such protocol). AvtoVAZ-built Datsun (on-DO / mi-DO) and Lada keyfobs; FM,
//! 433.92 MHz only (C flags `SubGhzProtocolFlag_433 | SubGhzProtocolFlag_FM`).
//!
//! **PHY** — Manchester with a 250 µs half-bit. Widths are classified with the C's fixed windows:
//! short = 141..339 µs, long = 361..749 µs (`te_delta` 110 is otherwise unused). On air each bit is
//! the half-bit pair (!v, v), i.e. 1 = LOW→HIGH, 0 = HIGH→LOW. The decoder inverts the level
//! (`level = !level`) and records the first half-bit of every cell: after a bit, a short pulse is the
//! cell's second half and is skipped (`marker`), a long pulse spans two cells and yields the next bit.
//! Pulses < 50 µs are glitches: they are dropped and the NEXT pulse is lengthened by the pulse that
//! preceded the glitch (C `glue`, mirrored exactly). Sync = an RF-LOW long pulse right after a short
//! one; the frame ends on the first pulse that is neither short nor long and is accepted only if
//! exactly 88 bits were collected.
//!
//! **Air frame** (C `datsun_lada_transmit`): 200 µs HIGH, then twice
//! [2 ms LOW, 125 × 250 µs toggles starting HIGH, 500 µs LOW (sync), 250 µs HIGH, 88 bits, 4 ms HIGH],
//! then 4 (or 32 for selector 3) short "tail" packets [2 ms LOW, 29 toggles, 500 µs LOW, 250 µs HIGH,
//! 24 bits, 36 ms HIGH]. Tails carry counter-low ⊕ step and never decode (24 bits ≠ 88).
//!
//! **Frame** — 88 bits = 11 bytes `r[0..11]`, sent `r[10]` first, each byte MSB-first:
//! - `r[10..=7]` — 32-bit serial (`r[10]` = MSB) → `serial`.
//! - `r[6]` — high nibble = button (`r[6] & 0xF0` → `button`: 0x40 BTN0, 0x80 BTN1, 0x20 BTN3,
//!   0x60 BTN5 in the C's custom-button map). The low nibble feeds the cipher state.
//! - `r[6..=1]` read as a 48-bit word F (`r[6]` = MSB): F\[41:34\] = counter low byte (as recovered by
//!   `learn`), F\[33:2\] = 32-bit authenticator, F\[1:0\] = `10` (forced by the C encoder).
//! - `r[0]` — XOR of `r[1..=10]` (C check `XOR(r[0..10]) == r[10]`, i.e. the XOR of all 11 bytes is 0).
//!
//! **Cipher / `learn`** — `datsun_lada_predicate` is the Hitag2 filter function (tables fa = 0x2C79,
//! fb = 0x6671, fc = 0x7907287B) over a 48-bit register `r[5..=10]`, and `datsun_lada_loop_carry` is the
//! Hitag2 LFSR feedback (same 16 taps, mirrored bit order). `learn` loads the register with the fixed
//! 16-bit word 0x4606 + the serial, absorbs 32 bits of serial ⊕ filter ⊕ a 32-bit `state` rebuilt from
//! the frame (counter low byte + button bits), then compares 32 keystream bits with F\[33:2\]. It is a
//! strong check (0 of 20 000 random XOR-valid frames pass in a model of the C) but it only verifies
//! frames whose counter bits 13:8 are zero: the C encoder mixes counter bits 13:8 (and the low nibble
//! of button codes 0x72/0x6A/0x5A/0x32) into the state, those bits are not in the frame, and `learn`
//! assumes zero. Even the C's own encoder output is "Check failed" for counter ≥ 256 or button 0x60.
//!
//! **Gating** — exactly the C's emission rule (sync + exactly 88 bits + XOR checksum, 1/256) plus one
//! KAT addition: a frame behind fewer than 64 consecutive short pulses must also pass `learn`. The C
//! encoder sends 125 (29 only on the 24-bit tails). Without the gate, the Fiat Panda capture (Fiat
//! V1, Marelli PCF7946 — the same 11-byte Hitag2-family frame with a 29-toggle pre-sync run) yields
//! an XOR-valid 88-bit "Datsun Lada" frame in inverted polarity, which does not verify. A real Lada
//! capture (a doctor-firmware user's, not in IMPORTS) shows why the gate is not a plain minimum:
//! noise cut the preamble of the first frame of two presses to 25 and 59 shorts, doctor-firmware
//! reads both, and both verify. `learn` is otherwise NOT a gate (it would reject genuine frames,
//! see above); it drives `crc_valid` / `encoder_capable` and whether `counter` is reported,
//! mirroring the C display ("Cnt" when learnable, "Check failed" otherwise).
//!
//! **DecodedSignal packing** (88 bits > 64): `data` = `r[10..=3]` (first 64 wire bits, big-endian),
//! `extra` = `r[2] << 16 | r[1] << 8 | r[0]` (last 24 wire bits, right-aligned), `data_count_bit` = 88.
//! `counter` = `learn` state counter (14-bit field; only its low byte is recoverable from a frame).
//!
//! **Encoder** — the C rolling-code encoder (`datsun_lada_build` + `datsun_lada_transmit`): the captured
//! frame is the "learned" anchor; `learn` must succeed (KAT has no saved "Learned"/"Cnt" checkpoint,
//! so this is the C's no-checkpoint path), counter = `counter` (or the learned one) + 1, masked to 14
//! bits, then the frame is rebuilt for the chosen button and the full upload above is emitted (C repeat
//! count 1). KAT buttons → C codes: Lock → 0x40 (BTN0, selector 0), Unlock → 0x80 (BTN1, selector 1),
//! Trunk → 0x20 (BTN3, selector 3, long tail), Panic → 0x60 (BTN5, selector 5). The C only labels these
//! BTN0/1/3/5, so the Lock/Unlock/Trunk/Panic assignment is an unverified guess. Native codes pass
//! through; any other code keeps the captured button, which must be one of the four (else no encode,
//! as in the C).
//!
//! **Key files** (`subghz_protocol_decoder_datsun_lada_serialize` / `_deserialize`, doctor-firmware's
//! generic writer): `Protocol: Datsun Lada`, `Bit: 88`, `Key` = `r[10..=3]`, then the generic
//! `Serial`, `Btn` (`r[6] & 0xF0`) and `Cnt` (the `learn` state counter, written even when the frame
//! does not verify, as the C does), and `Raw` (the 11 frame bytes `r[0..11]`). The C's `Learned`
//! checkpoint (written by its encoder after a transmission: the anchor frame + `Cnt`) is not
//! produced by decodes and is not carried: KAT loads a file that has one from its `Raw` frame
//! alone, so a frame that `learn` cannot verify (counter ≥ 256, button 0x60) shows "no counter"
//! where doctor-firmware would show the checkpoint's `Cnt`, and does not transmit. Loading follows
//! `datsun_lada_read`: the `Protocol` line must be exactly "Datsun Lada", 88 bits, and `Raw` must be
//! present and pass the XOR checksum.

use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Datsun Lada",
    decoder: "Datsun Lada",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::FM,
    encryption: "HITAG2",
    make: "Datsun/Lada",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&FM476],
};

/// The C only labels these BTN0/1/3/5; the Lock/Unlock/Trunk/Panic meaning is unverified.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x40 => "Btn0",
        0x80 => "Btn1",
        0x20 => "Btn3",
        0x60 => "Btn5",
        _ => "Other",
    })
}

const TE_SHORT: u32 = 250;
const TE_LONG: u32 = 500;
const TE_DELTA: u32 = 110;
const FRAME_BYTES: usize = 11;
const FRAME_BITS: usize = 88;
const COUNTER_MASK: u16 = 0x3FFF;
/// Pulses shorter than this are glitches (C `duration < 50`).
const GLITCH_US: u32 = 50;
/// KAT gate: consecutive short pulses required before the sync (C encoder sends 125).
const PREAMBLE_MIN_SHORTS: u16 = 64;
/// C `DATSUN_LADA_UPLOAD_CAPACITY` (merged entries; the largest possible upload is 3169).
const UPLOAD_CAPACITY: usize = 3200;

/// `datsun_lada_button_code` table, indexed by selector.
const BUTTON_CODES: [u8; 8] = [0x40, 0x80, 0x20, 0x20, 0x72, 0x6A, 0x5A, 0x32];

// Encoder waveform (datsun_lada_transmit / datsun_lada_preamble).
const TX_LEAD_HIGH_US: u32 = 200;
const TX_PREAMBLE_LOW_US: u32 = 2000;
const TX_MAIN_TOGGLES: usize = 125;
const TX_MAIN_FRAMES: usize = 2;
const TX_MAIN_END_HIGH_US: u32 = 4000;
const TX_TAIL_TOGGLES: usize = 29;
const TX_TAIL_END_HIGH_US: u32 = 36_000;
const TX_TAIL_LIMIT: usize = 16;
const TX_TAIL_LIMIT_LONG: usize = 128;
const TX_TAIL_STEP: usize = 4;

// KAT generic button codes.
const BTN_LOCK: u8 = 0x01;
const BTN_UNLOCK: u8 = 0x02;
const BTN_TRUNK: u8 = 0x04;
const BTN_PANIC: u8 = 0x08;

type Frame = [u8; FRAME_BYTES];

#[inline]
fn bit(value: u8, index: u8) -> u8 {
    (value >> index) & 1
}

#[inline]
fn mask(value: u8) -> u8 {
    1u8 << (value & 7)
}

/// `datsun_lada_predicate` — the Hitag2 non-linear filter over `r[5..=10]`.
fn predicate(r: &Frame) -> u8 {
    let acc = bit(r[10], 5) | (bit(r[10], 4) << 1) | (bit(r[10], 2) << 2);
    let mut flag = u8::from(mask(acc) & (if bit(r[10], 1) != 0 { 0x2C } else { 0x79 }) != 0);
    let acc = bit(r[9], 7) | (bit(r[9], 3) << 1) | (bit(r[9], 1) << 2);
    if mask(acc) & (if bit(r[9], 0) != 0 { 0x66 } else { 0x71 }) != 0 {
        flag |= 2;
    }
    let acc = bit(r[8], 6) | (bit(r[8], 2) << 1) | (bit(r[8], 0) << 2);
    if mask(acc) & (if bit(r[7], 5) != 0 { 0x66 } else { 0x71 }) != 0 {
        flag |= 4;
    }
    let acc = bit(r[7], 3) | (bit(r[7], 2) << 1) | (bit(r[7], 0) << 2);
    if mask(acc) & (if bit(r[6], 6) != 0 { 0x66 } else { 0x71 }) != 0 {
        flag |= 8;
    }
    let acc = bit(r[6], 5) | (bit(r[5], 4) << 1) | (bit(r[5], 3) << 2);
    let slot = if mask(acc) & (if bit(r[5], 1) != 0 { 0x2C } else { 0x79 }) != 0 {
        if bit(flag, 3) != 0 {
            0x79
        } else {
            0x07
        }
    } else if bit(flag, 3) != 0 {
        0x28
    } else {
        0x7B
    };
    u8::from(mask(flag) & slot != 0)
}

/// `datsun_lada_loop_carry` — parity of the Hitag2 feedback taps.
fn loop_carry(r: &Frame) -> u8 {
    let mut mixed = (r[10] & 0xB3) ^ (r[9] & 0x80) ^ (r[8] & 0x83) ^ (r[7] & 0x22) ^ (r[5] & 0x73);
    let mut carry = u8::from(mask(mixed) & (if bit(mixed, 3) != 0 { 0x69 } else { 0x96 }) != 0);
    mixed >>= 4;
    if mask(mixed) & (if bit(mixed, 3) != 0 { 0x69 } else { 0x96 }) != 0 {
        carry ^= 1;
    }
    carry
}

/// `datsun_lada_read32` (little-endian).
fn read32(r: &Frame, off: usize) -> u32 {
    u32::from_le_bytes([r[off], r[off + 1], r[off + 2], r[off + 3]])
}

/// `datsun_lada_write32` (little-endian).
fn write32(r: &mut Frame, off: usize, word: u32) {
    r[off..off + 4].copy_from_slice(&word.to_le_bytes());
}

/// `datsun_lada_shift_left`: `r[first..=last]` as a little-endian word, shifted left by one with
/// `carry` entering at the bottom.
fn shift_left(r: &mut Frame, first: usize, last: usize, mut carry: u8) {
    for byte in r.iter_mut().take(last + 1).skip(first) {
        let next = *byte >> 7;
        *byte = (*byte << 1) | (carry & 1);
        carry = next;
    }
}

/// `datsun_lada_shift_right`: `r[1..=6]` as a big-endian 48-bit word (`r[6]` = MSB), shifted right by 2.
fn shift_right(r: &mut Frame) {
    for _ in 0..2 {
        let mut carry = 0u8;
        for i in (1..=6).rev() {
            let next = r[i] & 1;
            r[i] = (r[i] >> 1) | (carry << 7);
            carry = next;
        }
    }
}

/// `datsun_lada_rounds`: 32 absorb rounds (filter ⊕ stack MSB ⊕ state MSB) then 32 output rounds.
fn rounds(r: &mut Frame, stack: &mut u32, state: &mut u32) {
    for _ in 0..32 {
        let hit = predicate(r) ^ ((*stack >> 31) as u8) ^ ((*state >> 31) as u8);
        shift_left(r, 5, 10, hit);
        *stack <<= 1;
        *state = state.rotate_left(1);
    }
    for _ in 0..32 {
        *stack = (*stack << 1) | u32::from(predicate(r));
        let carry = loop_carry(r);
        shift_left(r, 5, 10, carry);
    }
}

/// `datsun_lada_restore`.
fn restore(r: &mut Frame, stack: u32, state: u32, saved: u8) {
    write32(r, 1, stack);
    r[5] = state as u8;
    r[6] = saved;
    shift_left(r, 1, 6, 0);
    shift_left(r, 1, 6, 0);
}

/// `datsun_lada_checksum`: XOR of `r[0..10]` must equal `r[10]`.
fn checksum(frame: &Frame) -> bool {
    frame[..10].iter().fold(0u8, |acc, &b| acc ^ b) == frame[10]
}

/// `datsun_lada_learn`: recover the cipher state from the frame and verify the 32-bit authenticator.
/// Returns (verified, state).
fn learn(frame: &Frame) -> (bool, u32) {
    let mut r = *frame;
    shift_right(&mut r);
    let mut state =
        u32::from(r[5]) | (u32::from(r[6] & 3) << 8) | (u32::from((r[6] << 2) & 0xF0) << 24);
    let mut stack = read32(&r, 7);
    r[6] = 0x46;
    r[5] = 6;
    rounds(&mut r, &mut stack, &mut state);
    (stack == read32(&r, 1), state)
}

/// `datsun_lada_counter`: 14-bit counter from the state (bits 7:0 and 15:10).
fn counter_of(state: u32) -> u16 {
    ((state & 0xFF) | ((state >> 2) & 0x3F00)) as u16
}

/// `datsun_lada_build`: rebuild a frame from the learned anchor for `counter` / `selector`.
/// Returns (air frame, tail seed).
fn build(learned: &Frame, state: u32, counter: u16, selector: u8) -> (Frame, u8) {
    let mut r: Frame = [0; FRAME_BYTES];
    r[7..].copy_from_slice(&learned[7..]);
    let code = BUTTON_CODES[usize::from(selector & 7)];
    let button = (learned[6] & 0x0F) | code;
    r[6] = button;
    let mut state = (state & 0x00FF_0000)
        | u32::from(counter & 0xFF)
        | ((u32::from((counter >> 6) & 0xFC) | u32::from((learned[6] >> 2) & 3)) << 8)
        | ((((state >> 24) & 0x0F) | u32::from(code)) << 24);
    shift_right(&mut r);
    let saved = r[6];
    let mut stack = read32(&r, 7);
    r[6] = 0x46;
    r[5] = 6;
    rounds(&mut r, &mut stack, &mut state);
    restore(&mut r, stack, state, saved);
    if stack == read32(&r, 1) {
        // Idempotent; kept to mirror the C.
        restore(&mut r, stack, state, saved);
    }
    r[7..].copy_from_slice(&learned[7..]);
    r[6] = button;
    r[1] = (r[1] & 0xFE) | 2;
    r[0] = r[1..].iter().fold(0u8, |acc, &b| acc ^ b);
    (r, state as u8)
}

#[inline]
fn short_width(width: u32) -> bool {
    width > 140 && width < 340
}

#[inline]
fn long_width(width: u32) -> bool {
    width > 360 && width < 750
}

/// `datsun_lada_emit`: append, merging with the previous entry when the level repeats.
fn emit(signal: &mut Vec<LevelDuration>, level: bool, duration: u32) {
    if let Some(last) = signal.last_mut() {
        if last.level == level {
            *last = LevelDuration::new(level, last.duration_us + duration);
            return;
        }
    }
    signal.push(LevelDuration::new(level, duration));
}

/// `datsun_lada_preamble`.
fn tx_preamble(signal: &mut Vec<LevelDuration>, toggles: usize) {
    emit(signal, false, TX_PREAMBLE_LOW_US);
    for i in 0..toggles {
        emit(signal, i & 1 == 0, TE_SHORT);
    }
    emit(signal, false, TE_LONG);
    emit(signal, true, TE_SHORT);
}

/// `datsun_lada_window`: bytes sent last-index first, MSB-first, each bit as (!v, v).
fn tx_window(signal: &mut Vec<LevelDuration>, frame: &[u8]) {
    let bytes = frame.len();
    for i in 0..bytes * 8 {
        let value = bit(frame[bytes - 1 - (i >> 3)], 7 - (i & 7) as u8) != 0;
        emit(signal, !value, TE_SHORT);
        emit(signal, value, TE_SHORT);
    }
}

/// `datsun_lada_transmit`: two main frames, then the counter tails.
fn transmit(air: &Frame, tail_seed: u8, selector: u8) -> Vec<LevelDuration> {
    let mut signal = Vec::new();
    emit(&mut signal, true, TX_LEAD_HIGH_US);
    for _ in 0..TX_MAIN_FRAMES {
        tx_preamble(&mut signal, TX_MAIN_TOGGLES);
        tx_window(&mut signal, air);
        emit(&mut signal, true, TX_MAIN_END_HIGH_US);
    }
    let limit = if selector == 3 {
        TX_TAIL_LIMIT_LONG
    } else {
        TX_TAIL_LIMIT
    };
    for step in (0..limit).step_by(TX_TAIL_STEP) {
        let t2 = ((air[6] >> 2) & 3).wrapping_add(step as u8);
        let t1 = tail_seed;
        let t0 = t1 ^ t2;
        tx_preamble(&mut signal, TX_TAIL_TOGGLES);
        tx_window(&mut signal, &[t0, t1, t2]);
        emit(&mut signal, true, TX_TAIL_END_HIGH_US);
    }
    signal
}

/// Decoder steps (matches `DatsunLadaDecoderStep`).
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Data,
}

/// Datsun Lada decoder (matches `SubGhzProtocolDecoderDatsunLada`).
pub struct DatsunLadaDecoder {
    step: DecoderStep,
    te_last: u32,
    decode_count_bit: usize,
    receive_data: Frame,
    marker: bool,
    glue: bool,
    /// KAT preamble gate: consecutive short pulses seen so far.
    short_run: u16,
    /// Short pulses that preceded the current frame's sync (the preamble gate, applied at the end).
    sync_run: u16,
}

impl DatsunLadaDecoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            te_last: 0,
            decode_count_bit: 0,
            receive_data: [0; FRAME_BYTES],
            marker: false,
            glue: false,
            short_run: 0,
            sync_run: 0,
        }
    }

    /// `datsun_lada_fields` (+ KAT packing).
    fn signal_from_frame(raw: &Frame) -> DecodedSignal {
        let (learnable, state) = learn(raw);
        DecodedSignal {
            serial: Some(u32::from_be_bytes([raw[10], raw[9], raw[8], raw[7]])),
            button: Some(raw[6] & 0xF0),
            counter: learnable.then(|| counter_of(state)),
            crc_valid: learnable,
            data: u64::from_be_bytes([
                raw[10], raw[9], raw[8], raw[7], raw[6], raw[5], raw[4], raw[3],
            ]),
            data_count_bit: FRAME_BITS,
            encoder_capable: learnable,
            extra: Some((u64::from(raw[2]) << 16) | (u64::from(raw[1]) << 8) | u64::from(raw[0])),
            protocol_display_name: None,
        }
    }

    /// Inverse of the packing in `signal_from_frame`.
    fn frame_from_signal(decoded: &DecodedSignal) -> Option<Frame> {
        if decoded.data_count_bit != FRAME_BITS {
            return None;
        }
        let extra = decoded.extra?;
        let mut raw: Frame = [0; FRAME_BYTES];
        for (i, b) in decoded.data.to_be_bytes().iter().enumerate() {
            raw[10 - i] = *b;
        }
        raw[2] = (extra >> 16) as u8;
        raw[1] = (extra >> 8) as u8;
        raw[0] = extra as u8;
        Some(raw)
    }

    /// KAT generic button → Datsun Lada code (native codes pass through; others keep `original`).
    fn map_button(button: u8, original: u8) -> u8 {
        match button {
            BTN_LOCK => 0x40,
            BTN_UNLOCK => 0x80,
            BTN_TRUNK => 0x20,
            BTN_PANIC => 0x60,
            0x20 | 0x40 | 0x60 | 0x80 => button,
            _ => original,
        }
    }

    /// Button → transmit selector (C `subghz_protocol_encoder_datsun_lada_deserialize`).
    fn selector_for(button: u8) -> Option<u8> {
        match button {
            0x40 => Some(0),
            0x80 => Some(1),
            0x20 => Some(3),
            0x60 => Some(5),
            _ => None,
        }
    }
}

impl ProtocolDecoder for DatsunLadaDecoder {
    fn name(&self) -> &'static str {
        "Datsun Lada"
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
        &[433_920_000]
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.te_last = 0;
        self.decode_count_bit = 0;
        self.receive_data = [0; FRAME_BYTES];
        self.marker = false;
        self.glue = false;
        self.short_run = 0;
        self.sync_run = 0;
    }

    /// `datsun_lada_receive` + `datsun_lada_fields`.
    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let mut duration = duration;
        if duration < GLITCH_US {
            self.glue = true;
            return None;
        }
        let previous = self.te_last;
        if self.glue {
            duration = duration.saturating_add(previous);
            self.glue = false;
        }
        self.te_last = duration;
        let level = !level;

        let run_before = self.short_run;
        self.short_run = if short_width(duration) {
            self.short_run.saturating_add(1)
        } else {
            0
        };

        if self.step == DecoderStep::Reset {
            self.decode_count_bit = 0;
            if level && short_width(previous) && long_width(duration) {
                self.step = DecoderStep::Data;
                self.marker = false;
                self.receive_data = [0; FRAME_BYTES];
                self.sync_run = run_before;
            }
            return None;
        }
        if self.decode_count_bit == 0 && long_width(previous) && short_width(duration) {
            self.marker = false;
            return None;
        }
        if short_width(duration) || long_width(duration) {
            if short_width(duration) && self.marker {
                self.marker = false;
                return None;
            }
            if self.decode_count_bit >= FRAME_BITS {
                self.step = DecoderStep::Reset;
                self.decode_count_bit = 0;
                return None;
            }
            let byte = 10 - (self.decode_count_bit >> 3);
            if level {
                self.receive_data[byte] |= 1 << (7 - (self.decode_count_bit & 7));
            }
            self.decode_count_bit += 1;
            self.marker = true;
            return None;
        }
        let found = self.decode_count_bit == FRAME_BITS && checksum(&self.receive_data);
        self.step = DecoderStep::Reset;
        self.decode_count_bit = 0;
        if !found {
            return None;
        }
        // KAT preamble gate: behind fewer than 64 short pulses the frame must verify (`learn`).
        let signal = Self::signal_from_frame(&self.receive_data);
        (self.sync_run >= PREAMBLE_MIN_SHORTS || signal.crc_valid).then_some(signal)
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        let raw = Self::frame_from_signal(decoded)?;
        if !checksum(&raw) {
            return None;
        }
        // No "Learned"/"Cnt" checkpoint in KAT: the C then requires learn() on the captured frame.
        let (learnable, state) = learn(&raw);
        if !learnable {
            return None;
        }
        let selector = Self::selector_for(Self::map_button(button, raw[6] & 0xF0))?;
        let counter = decoded
            .counter
            .map_or_else(|| counter_of(state), |c| c & COUNTER_MASK);
        let counter = (counter + 1) & COUNTER_MASK;
        let (air, tail_seed) = build(&raw, state, counter, selector);
        let signal = transmit(&air, tail_seed, selector);
        // C overflow check (cannot trigger: the largest possible upload is 3169 entries).
        (signal.len() <= UPLOAD_CAPACITY).then_some(signal)
    }

    /// `subghz_protocol_decoder_datsun_lada_deserialize` (`datsun_lada_read`; see "Key files").
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.protocol != "Datsun Lada" || key.bits != FRAME_BITS {
            return None;
        }
        key.key_u64()?;
        let raw: Frame = key.hex_bytes_field("Raw")?.try_into().ok()?;
        checksum(&raw).then(|| Self::signal_from_frame(&raw))
    }

    /// `subghz_protocol_decoder_datsun_lada_serialize` (see the module doc, "Key files").
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        let raw = Self::frame_from_signal(decoded)?;
        let serial = u32::from_be_bytes([raw[10], raw[9], raw[8], raw[7]]);
        let button = u32::from(raw[6] & 0xF0);
        let counter = u32::from(counter_of(learn(&raw).1));
        let mut key = FlipperKey::for_protocol("Datsun Lada", FRAME_BITS, decoded.data);
        if serial != 0 || button != 0 || counter != 0 {
            key.push_u32("Serial", serial);
            key.push_u32("Btn", button);
            key.push_u32("Cnt", counter);
        }
        key.push_hex_bytes("Raw", &raw);
        Some(key)
    }
}

impl Default for DatsunLadaDecoder {
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

    /// Wire order (r[10] first) → frame array.
    fn frame_from_wire(wire: [u8; FRAME_BYTES]) -> Frame {
        let mut f: Frame = [0; FRAME_BYTES];
        for (i, b) in wire.iter().enumerate() {
            f[10 - i] = *b;
        }
        f
    }

    /// Learnable anchor produced by `build` (vector cross-checked against a Python model of the C):
    /// serial 0x12345678, button byte 0x41, counter 0x5A.
    fn anchor() -> Frame {
        frame_from_wire([
            0x12, 0x34, 0x56, 0x78, 0x41, 0x6A, 0x2A, 0x5C, 0xE7, 0xA2, 0x10,
        ])
    }

    fn decode_all(signal: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = DatsunLadaDecoder::new();
        signal
            .iter()
            .filter_map(|ld| dec.feed(ld.level, ld.duration_us))
            .collect()
    }

    fn pairs(signal: Option<Vec<LevelDuration>>) -> Option<Vec<(bool, u32)>> {
        signal.map(|s| s.iter().map(|ld| (ld.level, ld.duration_us)).collect())
    }

    #[test]
    fn datsun_lada_build_matches_reference_vectors() {
        // Seed frame: serial 12 34 56 78, button byte 0x41 (low bits 01 = counter bits 7:6).
        let seed = frame_from_wire([
            0x12,
            0x34,
            0x56,
            0x78,
            0x41,
            0,
            0,
            0,
            0,
            0,
            0x12 ^ 0x34 ^ 0x56 ^ 0x78 ^ 0x41,
        ]);
        assert!(checksum(&seed));
        let (seed_ok, seed_state) = learn(&seed);
        assert!(!seed_ok);
        let (air, tail_seed) = build(&seed, seed_state, 0x5A, 0);
        assert_eq!(air, anchor());
        assert_eq!(tail_seed, 0x5A);
        assert!(checksum(&air));
        let (ok, state) = learn(&air);
        assert!(ok, "learn() must verify a C-built frame with counter < 256");
        assert_eq!(counter_of(state), 0x5A);

        // Next press for each selector (vectors from the C model).
        let cases: [(u8, [u8; FRAME_BYTES], bool); 4] = [
            (
                0,
                [
                    0x12, 0x34, 0x56, 0x78, 0x41, 0x6F, 0xBB, 0x7C, 0xD4, 0x86, 0xB3,
                ],
                true,
            ),
            (
                1,
                [
                    0x12, 0x34, 0x56, 0x78, 0x81, 0x6F, 0xC0, 0x03, 0xEF, 0x66, 0xAC,
                ],
                true,
            ),
            (
                3,
                [
                    0x12, 0x34, 0x56, 0x78, 0x21, 0x6F, 0x01, 0x4D, 0x5C, 0x3E, 0x68,
                ],
                true,
            ),
            // Selector 5 (code 0x6A) is never learnable: its low nibble is not in the frame.
            (
                5,
                [
                    0x12, 0x34, 0x56, 0x78, 0x6B, 0x6C, 0x3D, 0x20, 0x0A, 0xF2, 0xEA,
                ],
                false,
            ),
        ];
        for (selector, wire, learnable) in cases {
            let (next, _) = build(&air, state, 0x5B, selector);
            assert_eq!(next, frame_from_wire(wire), "selector {selector}");
            assert_eq!(learn(&next).0, learnable, "selector {selector}");
        }
    }

    #[test]
    fn datsun_lada_learn_counter_limit() {
        // A C-built frame with counter >= 256 fails the C's own learn() (bits 13:8 not in frame).
        let air = anchor();
        let (_, state) = learn(&air);
        let (high, _) = build(&air, state, 0x015B, 0);
        assert!(checksum(&high));
        assert!(!learn(&high).0);
    }

    #[test]
    fn datsun_lada_decodes_synthetic_transmission() {
        let air = anchor();
        let signal = transmit(&air, 0x5A, 0);
        let decodes = decode_all(&signal);
        // Two main frames; the 24-bit tails never decode.
        assert_eq!(decodes.len(), 2);
        for d in &decodes {
            assert_eq!(DatsunLadaDecoder::frame_from_signal(d), Some(air));
            assert_eq!(d.serial, Some(0x1234_5678));
            assert_eq!(d.button, Some(0x40));
            assert_eq!(d.counter, Some(0x5A));
            assert!(d.crc_valid && d.encoder_capable);
            assert_eq!(d.data_count_bit, FRAME_BITS);
        }
    }

    #[test]
    fn datsun_lada_encode_decode_roundtrip() {
        let original = DatsunLadaDecoder::signal_from_frame(&anchor());
        let enc = DatsunLadaDecoder::new();
        for (button, code, wire) in [
            (
                BTN_LOCK,
                0x40,
                [
                    0x12, 0x34, 0x56, 0x78, 0x41, 0x6F, 0xBB, 0x7C, 0xD4, 0x86, 0xB3,
                ],
            ),
            (
                BTN_UNLOCK,
                0x80,
                [
                    0x12, 0x34, 0x56, 0x78, 0x81, 0x6F, 0xC0, 0x03, 0xEF, 0x66, 0xAC,
                ],
            ),
            (
                BTN_TRUNK,
                0x20,
                [
                    0x12, 0x34, 0x56, 0x78, 0x21, 0x6F, 0x01, 0x4D, 0x5C, 0x3E, 0x68,
                ],
            ),
        ] {
            let signal = enc.encode(&original, button).expect("encode");
            assert!(
                signal.windows(2).all(|w| w[0].level != w[1].level),
                "merged levels"
            );
            let decodes = decode_all(&signal);
            assert_eq!(decodes.len(), 2, "button {button:#x}");
            let d = &decodes[0];
            assert_eq!(
                DatsunLadaDecoder::frame_from_signal(d),
                Some(frame_from_wire(wire))
            );
            assert_eq!(d.serial, Some(0x1234_5678));
            assert_eq!(d.button, Some(code));
            // The encoder bumps the counter for a fresh press.
            assert_eq!(d.counter, Some(0x5B));
            assert!(d.crc_valid);
        }
        // Panic (0x60) encodes, but the result is not learnable (as in the C).
        let signal = enc.encode(&original, BTN_PANIC).expect("encode panic");
        let decodes = decode_all(&signal);
        assert_eq!(decodes.len(), 2);
        assert_eq!(decodes[0].button, Some(0x60));
        assert!(
            !decodes[0].crc_valid && decodes[0].counter.is_none() && !decodes[0].encoder_capable
        );
        // A native code passes through; an unknown code keeps the captured button (0x40).
        assert_eq!(
            pairs(enc.encode(&original, 0x80)),
            pairs(enc.encode(&original, BTN_UNLOCK))
        );
        assert_eq!(
            pairs(enc.encode(&original, 0x10)),
            pairs(enc.encode(&original, BTN_LOCK))
        );
    }

    #[test]
    fn datsun_lada_rejects_bad_frames() {
        let air = anchor();
        // One flipped bit breaks the XOR checksum.
        let mut bad = air;
        bad[4] ^= 0x10;
        assert!(decode_all(&transmit(&bad, 0x5A, 0)).is_empty());

        // Short preamble (29 toggles, like the tails / Fiat V1): KAT's gate accepts the frame only
        // if `learn` verifies it. A real Lada capture (a doctor-firmware user's, not in IMPORTS)
        // has first frames whose preamble noise cut to 25 and 59 shorts; doctor-firmware reads
        // them. An XOR-valid frame that does not verify (the Fiat Panda one below) needs >= 64.
        let short = |frame: &Frame, toggles: usize| {
            let mut signal = Vec::new();
            tx_preamble(&mut signal, toggles);
            tx_window(&mut signal, frame);
            emit(&mut signal, true, TX_MAIN_END_HIGH_US);
            decode_all(&signal)
        };
        assert!(learn(&air).0);
        for toggles in [25, 29, 59] {
            let d = short(&air, toggles);
            assert_eq!(d.len(), 1, "{toggles} toggles");
            assert!(d[0].crc_valid && DatsunLadaDecoder::frame_from_signal(&d[0]) == Some(air));
        }
        let fiat = frame_from_wire([
            0xEF, 0xB8, 0x2D, 0x66, 0x22, 0x34, 0xA6, 0x6F, 0x54, 0xFA, 0x6D,
        ]);
        assert!(short(&fiat, 29).is_empty() && short(&fiat, 63).is_empty());
        let unverified = short(&fiat, 65); // odd: an even count merges its last LOW into the sync
        assert!(unverified.len() == 1 && !unverified[0].crc_valid);

        // 87 or 89 bits instead of exactly 88.
        let bits: Vec<bool> = (0..FRAME_BITS)
            .map(|i| bit(air[10 - (i >> 3)], 7 - (i & 7) as u8) != 0)
            .collect();
        for n in [FRAME_BITS - 1, FRAME_BITS + 1] {
            let mut frame_bits = bits.clone();
            frame_bits.resize(n, false);
            let mut signal = Vec::new();
            tx_preamble(&mut signal, TX_MAIN_TOGGLES);
            for &v in &frame_bits {
                emit(&mut signal, !v, TE_SHORT);
                emit(&mut signal, v, TE_SHORT);
            }
            emit(&mut signal, true, TX_MAIN_END_HIGH_US);
            assert!(decode_all(&signal).is_empty(), "{n} bits");
        }

        // Encoder refuses frames that are not learnable or fail the checksum.
        let mut sig = DatsunLadaDecoder::signal_from_frame(&air);
        sig.extra = Some(sig.extra.unwrap() ^ 1);
        assert!(DatsunLadaDecoder::new().encode(&sig, BTN_LOCK).is_none());
        let fiat = frame_from_wire([
            0xEF, 0xB8, 0x2D, 0x66, 0x22, 0x34, 0xA6, 0x6F, 0x54, 0xFA, 0x6D,
        ]);
        assert!(checksum(&fiat) && !learn(&fiat).0);
        let sig = DatsunLadaDecoder::signal_from_frame(&fiat);
        assert!(!sig.encoder_capable && !sig.crc_valid && sig.counter.is_none());
        assert!(DatsunLadaDecoder::new().encode(&sig, BTN_LOCK).is_none());
    }

    #[test]
    fn datsun_lada_glitch_glue() {
        // C `glue`: a sub-50 µs glitch is dropped and the NEXT pulse is lengthened by the pulse
        // before the glitch (250 + 250 → 500), which also breaks the preamble run.
        let mut dec = DatsunLadaDecoder::new();
        assert!(dec.feed(true, 250).is_none());
        assert!(dec.feed(false, 20).is_none());
        assert!(dec.glue);
        assert!(dec.feed(true, 250).is_none());
        assert!(!dec.glue);
        assert_eq!(dec.te_last, 500);
        assert_eq!(dec.short_run, 0);

        // A glitch early in the first preamble still leaves >= 64 shorts before the sync, so both
        // main frames decode unchanged.
        let air = anchor();
        let mut signal = transmit(&air, 0x5A, 0);
        signal.insert(40, LevelDuration::new(true, 20));
        let decodes = decode_all(&signal);
        assert_eq!(decodes.len(), 2);
        assert!(decodes
            .iter()
            .all(|d| DatsunLadaDecoder::frame_from_signal(d) == Some(air)));
    }

    #[test]
    fn datsun_lada_ignores_fiat_panda_capture() {
        // Fiat V1 (Marelli PCF7946) shares this 11-byte frame shape; in inverted polarity the Fiat
        // Panda capture contains an XOR-valid 88-bit frame after a 29-toggle run. The preamble gate
        // must reject it, and the registry must still report Fiat V1.
        let path = Path::new("IMPORTS/FIAT/Fiat Panda.sub");
        if !path.exists() {
            eprintln!("Skip: {:?} not found (run from crate root)", path);
            return;
        }
        let (freq, raw_pairs) = import_sub_raw(path).unwrap();
        for invert in [false, true] {
            let mut dec = DatsunLadaDecoder::new();
            let hits = raw_pairs
                .iter()
                .filter_map(|p| dec.feed(p.level ^ invert, p.duration_us))
                .count();
            assert_eq!(
                hits, 0,
                "Datsun Lada must not decode Fiat Panda (invert={invert})"
            );
        }
        let pairs: Vec<LevelDuration> = raw_pairs
            .iter()
            .map(|p| LevelDuration::new(p.level, p.duration_us))
            .collect();
        let mut reg = ProtocolRegistry::new();
        let names: Vec<String> = reg
            .process_signal_stream(&pairs, freq)
            .into_iter()
            .map(|(n, _, _)| n)
            .collect();
        assert!(names.iter().any(|n| n == "Fiat V1"), "got {names:?}");
        assert!(!names.iter().any(|n| n == "Datsun Lada"), "got {names:?}");
    }

    #[test]
    fn datsun_lada_registry_claims_own_transmission() {
        // The full C upload must decode as Datsun Lada through the registry (no earlier decoder
        // steals it).
        let original = DatsunLadaDecoder::signal_from_frame(&anchor());
        let signal = DatsunLadaDecoder::new()
            .encode(&original, BTN_LOCK)
            .unwrap();
        let mut reg = ProtocolRegistry::new();
        let names: Vec<String> = reg
            .process_signal_stream(&signal, 433_920_000)
            .into_iter()
            .map(|(n, _, _)| n)
            .collect();
        assert!(
            !names.is_empty() && names.iter().all(|n| n == "Datsun Lada"),
            "got {names:?}"
        );
    }

    /// Key files: doctor-firmware's lines round-trip learnable and unlearnable frames, the C's
    /// Protocol/Raw/checksum checks refuse bad files, and TX from a loaded key decodes.
    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::{assert_round_trip, key};
        let dec = DatsunLadaDecoder::new();
        let air = anchor();
        let decoded = decode_all(&transmit(&air, 0x5A, 0)).remove(0);
        assert!(decoded.crc_valid);
        let text = assert_round_trip(&dec, &decoded);
        assert!(
            text.ends_with(
                "Protocol: Datsun Lada\nBit: 88\nKey: 12 34 56 78 41 6A 2A 5C\nSerial: 305419896\n\
                 Btn: 64\nCnt: 90\nRaw: 10 A2 E7 5C 2A 6A 41 78 56 34 12\n"
            ),
            "{text}"
        );
        // A frame learn() cannot verify still round-trips (no counter, not encoder-capable); the
        // C writes its state counter anyway.
        let mut odd = air;
        odd[3] ^= 0x01;
        odd[0] ^= 0x01;
        let unverified = DatsunLadaDecoder::signal_from_frame(&odd);
        assert!(!unverified.crc_valid && unverified.counter.is_none());
        assert!(assert_round_trip(&dec, &unverified).contains("\nCnt: "));

        // datsun_lada_read: exact Protocol name, Raw present with a valid XOR.
        let mut k = key(&text);
        k.protocol = "datsun lada".into();
        assert!(dec.from_flipper_key(&k).is_none());
        let mut k = key(&text);
        k.fields.retain(|(n, _)| n != "Raw");
        assert!(dec.from_flipper_key(&k).is_none());
        let mut k = key(&text);
        k.fields.retain(|(n, _)| n != "Raw");
        k.push("Raw", "11 A2 E7 5C 2A 6A 41 78 56 34 12");
        assert!(dec.from_flipper_key(&k).is_none());
        // A doctor encoder checkpoint (Learned + Cnt) is ignored: the Raw frame decides.
        let mut k = key(&text);
        k.push("Learned", "10 A2 E7 5C 2A 6A 41 78 56 34 12");
        assert_eq!(dec.from_flipper_key(&k), Some(decoded.clone()));

        // TX from the loaded key: the next code (counter + 1) decodes.
        let loaded = dec.from_flipper_key(&key(&text)).unwrap();
        let tx = decode_all(&dec.encode(&loaded, BTN_LOCK).unwrap());
        assert_eq!((tx[0].serial, tx[0].counter), (Some(0x1234_5678), Some(0x5B)));
        // doctor-firmware's encoder (C oracle) given KAT's file for the frame one press later
        // (counter 0x5B) transmits Raw 91 C6 2C 7E 3F 73 41 78 56 34 12 (counter 0x5C); KAT's
        // transmission from the same file is that frame.
        let reloaded = dec.from_flipper_key(&key(&assert_round_trip(&dec, &tx[0]))).unwrap();
        let tx2 = decode_all(&dec.encode(&reloaded, BTN_LOCK).unwrap());
        assert_eq!(
            DatsunLadaDecoder::frame_from_signal(&tx2[0]).unwrap(),
            [0x91, 0xC6, 0x2C, 0x7E, 0x3F, 0x73, 0x41, 0x78, 0x56, 0x34, 0x12]
        );
    }

    /// Oracle vectors (`KAT_KEY_VECTORS=<dir> cargo test datsun_lada_key_vectors -- --ignored`).
    #[test]
    #[ignore]
    fn datsun_lada_key_vectors() {
        use super::super::bmw::{key_vector_dir, write_key_vector};
        let dec = DatsunLadaDecoder::new();
        let original = DatsunLadaDecoder::signal_from_frame(&anchor());
        for (n, btn) in [(1, BTN_LOCK), (2, BTN_UNLOCK)] {
            let wire = dec.encode(&original, btn).unwrap();
            let decoded = decode_all(&wire).remove(0);
            let key = dec.to_flipper_key(&decoded).unwrap();
            write_key_vector(&key_vector_dir(), &format!("datsun_lada_{n}"), 433_920_000, "FuriHalSubGhzPreset2FSKDev476Async", &wire, &key);
        }
    }
}
