//! Toyota / Lexus protocol decoder + encoder
//!
//! Ported from Flipper-ARF `lib/subghz/protocols/toyota.c` / `toyota.h` (ARF commit e4610aae
//! "Update 3", 2026-10-28), which is doctor-firmware's `toyota.c` (commit a3d8b7bc6, protocol
//! name "Toyota Lexus") with only the Flipper custom-button glue changed. That file REPLACED
//! ARF's earlier dual-variant KeeLoq decoder (Corolla 400/800µs PWM + Tundra NRZ, decode-only),
//! which ARF kept only as the uncompiled `toyota-old.c.bak`; KAT mirrors the replacement.
//!
//! Air format (both references): ~793/1587µs pulses, read level-agnostically — two short pulses
//! are a 0 bit, one long pulse is a 1 bit (bi-phase-space: the level toggles on every bit
//! boundary and again mid-bit for a 0). A press is 10 back-to-back copies of a 96-bit payload,
//! then 10 copies of its "second half" twin, with no gaps. The decoder slides a 120-bit window
//! (payload + its next 3 bytes) and accepts it when byte 0 is 0x00, byte 1 has its MSB set and
//! the first 3 bytes repeat at bytes 12..15.
//!
//! Payload obfuscation: a 7-bit counter selects which of 8 mask rows are XORed over the 12 bytes,
//! and bytes 1 and 3 get a counter/half "mix" (0x09 for odd counters, ^0x05 for the second
//! half). `toyota_recover` brute-forces all 128 counters × 2 halves for the unique one whose
//! plaintext keeps the frame's fixed id (24 bits from bytes 4/6/8/11) and has the "Estima"
//! button byte 0xD1. Recovered frames report the rebuilt PRIMARY frame (counter, first half),
//! the 24-bit fixed id as serial, button 0xD1 and the counter; the encoder re-sends the plaintext
//! at counter-1 (the counter counts DOWN, mod 128) as 10× primary + 10× second half, exactly the
//! C upload. Unrecoverable frames are reported as the C reports them (raw window, the C's
//! button-from-payload, no counter) but are not encoder-capable (`toyota_encode_action` refuses).
//!
//! The C's command selection (1-3) is only range-checked: it never reaches the transmitted frame,
//! so `encode` ignores KAT's button argument and always re-sends the frame's own button byte.
//!
//! KAT differences (no effect on decoded fields or the upload):
//! - The C suppresses re-reporting the same payload until a ≥20 ms gap (`have_reported`); KAT's
//!   registry resets every decoder after each hit anyway, so this decoder reports every aligned
//!   window and the app's import dedup collapses repeats.
//! - ARF's own `generic.c` does not persist `data_2` (`Key_2`), so in ARF a saved Toyota capture
//!   with a non-zero tail cannot be re-loaded for TX; doctor-firmware's `generic.c` (which this
//!   code was written against) does. KAT keeps all 96 bits (`data` + `extra`), like doctor.
//! - The upload's framing-pulse branches (`framing` 1/2) and the non-carried phase reset are
//!   unreachable from `toyota_encode_action` (it always passes framing 0 with `carry_phase`), so
//!   they are not ported.
//!
//! Key files (`subghz_protocol_decoder_toyota_serialize` / `_deserialize`): `Protocol: Toyota`
//! (doctor-firmware's "Toyota Lexus" loads too, through KAT's key-file name table), `Bit: 96`, `Key`
//! = payload bytes 0..7, `Key_2` = `00 00 00 00` + bytes 8..11, then `Btn` and `ToyotaBtn` (the
//! published button) and `Disp` ("Toyota - Estima|Cmd|Unknown"). The C's serialize writes `Key_2`
//! only for a zero tail and relies on the generic writer for the rest — doctor's writes it, ARF's
//! does not (the gap noted above) — so KAT always writes it, in the layout ARF's own encoder
//! persists (`toyota_write_state`). Loading follows the decoder's deserialize: `Key_2` required with
//! its high half zero, 96 bits, a key starting 0x00 with bit 55 set unless the file has the
//! encoder's `Copy` line, and the `ToyotaBtn` / `Btn` checks (`toyota_read_button`); the decode is
//! then the decoder's own for that payload (the `Btn` command never reaches the frame). FoxFW still
//! registers ARF's retired KeeLoq decoder under the name "Toyota": its keys (`Bit` 66..=68, `Tail`)
//! go to KAT's port of it, "Toyota (FoxFW)" (`toyota_foxfw.rs`).

use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Toyota",
    decoder: "Toyota",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::DifferentialManchester,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "Toyota/Lexus",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// toyota_button_name: the Estima button bytes, or the C's command 1-3 ("Cmd").
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0xD1 | 0xD4 | 0x99 | 0x9C => "Estima",
        1 => "Cmd 1",
        2 => "Cmd 2",
        3 => "Cmd 3",
        _ => "Unknown",
    })
}

const PAYLOAD_BYTES: usize = 12;
const FRAME_BYTES: usize = 15;
const COUNT_BIT: usize = PAYLOAD_BYTES * 8;
const FRAME_BITS: usize = FRAME_BYTES * 8;
const REPEAT_BYTES: usize = 3;
const BURST_COUNT: usize = 10;
/// Encoder buffer size in the C (`TOYOTA_UPLOAD_MAX`); the upload never exceeds it.
const UPLOAD_MAX: usize = 4096;
const FOB_PHASE: u8 = 3;
const TICK_US: u32 = 32;
const SHORT_TICK_MIN: u32 = 18;
const SHORT_TICK_MAX: u32 = 29;
const LONG_TICK_MIN: u32 = 39;
const LONG_TICK_MAX: u32 = 57;
const ESTIMA_XOR: u8 = 0x05;
const RETOUCH_XOR: u8 = 0x09;
const BASE_BUTTON: u8 = 0xD1;

/// `TOYOTA_US(count)`: fob timer counts → µs.
const fn toyota_us(count: u32) -> u32 {
    ((count >> 8) & 0xFF) * 254 + if count & 0xFF != 0 { (count & 0xFF) - 1 } else { 0 }
}

const TE_SHORT: u32 = toyota_us(0x0320); // 793
const TE_LONG: u32 = toyota_us(0x0640); // 1587
const TE_DELTA: u32 = 185;

/// `toyota_masks`: one 12-byte row (stride 16) per selector bit.
const MASKS: [[u8; 16]; 8] = [
    [0x00, 0x32, 0x41, 0x11, 0x32, 0x42, 0x2D, 0x42, 0x18, 0x41, 0x04, 0x20, 0x00, 0x00, 0x00, 0x00],
    [0x00, 0x19, 0x41, 0x50, 0x0A, 0x12, 0x00, 0x13, 0x12, 0x51, 0x44, 0x04, 0x00, 0x00, 0x00, 0x00],
    [0x00, 0x4E, 0x00, 0x41, 0x10, 0x06, 0x0D, 0x41, 0x00, 0x19, 0x4C, 0x00, 0x00, 0x00, 0x00, 0x00],
    [0x00, 0x2C, 0x41, 0x00, 0x3A, 0x40, 0x08, 0x12, 0x08, 0x4D, 0x48, 0x00, 0x00, 0x00, 0x00, 0x00],
    [0x00, 0x16, 0x41, 0x41, 0x1A, 0x50, 0x04, 0x03, 0x02, 0x15, 0x4C, 0x00, 0x00, 0x00, 0x00, 0x00],
    [0x00, 0x09, 0x41, 0x41, 0x38, 0x16, 0x2C, 0x43, 0x08, 0x08, 0x48, 0x00, 0x00, 0x00, 0x00, 0x00],
    [0x00, 0x2B, 0x41, 0x50, 0x1A, 0x06, 0x2D, 0x11, 0x12, 0x04, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00],
    [0x00, 0x21, 0x41, 0x00, 0x20, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
];

type Payload = [u8; PAYLOAD_BYTES];

/// `toyota_selector` terms in C order: (byte, bit mask, XOR when the bit is set (true) or clear
/// (false), selector bits). The C repeats the `b8 & 0x08 → ^0x04` term, which cancels itself.
const SELECTOR_TERMS: [(usize, u8, bool, u8); 18] = [
    (11, 0x20, false, 0x01),
    (11, 0x04, true, 0x02),
    (11, 0x20, true, 0x04),
    (9, 0x08, true, 0x04),
    (8, 0x08, true, 0x04),
    (8, 0x08, true, 0x04),
    (11, 0x20, true, 0x08),
    (11, 0x04, true, 0x08),
    (9, 0x40, true, 0x08),
    (11, 0x04, false, 0x10),
    (10, 0x40, true, 0x10),
    (9, 0x08, true, 0x10),
    (11, 0x04, true, 0x20),
    (9, 0x40, true, 0x20),
    (8, 0x08, true, 0x20),
    (11, 0x20, true, 0x40),
    (9, 0x40, true, 0x40),
    (7, 0x10, true, 0x40),
];

/// `toyota_selector`: mask selector derived from bits of bytes 7..11.
fn selector(block: &Payload) -> u8 {
    SELECTOR_TERMS.iter().fold(0, |s, &(byte, mask, when_set, x)| {
        if (block[byte] & mask != 0) == when_set {
            s ^ x
        } else {
            s
        }
    })
}

/// `toyota_apply_masks`.
fn apply_masks(block: &mut Payload, selector: u8) {
    for (bit, row) in MASKS.iter().enumerate() {
        if selector & (1 << bit) == 0 {
            continue;
        }
        for (b, m) in block.iter_mut().zip(row.iter()) {
            *b ^= m;
        }
    }
}

/// `toyota_transform`: the "standard" view of a wire block (its own selector's masks removed).
fn transform(wire: &Payload) -> Payload {
    let mut block = *wire;
    let s = selector(&block);
    apply_masks(&mut block, s);
    block
}

/// `toyota_fixed_id`: 24-bit fixed id from bytes 4 (low nibble), 6 (high nibble), 8 and 11.
fn fixed_id(data: &Payload) -> u32 {
    (((data[4] & 0x0F) as u32) << 20)
        | (((data[6] >> 4) as u32) << 16)
        | ((data[8] as u32) << 8)
        | data[11] as u32
}

/// `toyota_counter_mix`.
fn counter_mix(counter: u8, second_half: bool) -> u8 {
    let mut mix = if counter & 1 != 0 { RETOUCH_XOR } else { 0 };
    if second_half {
        mix ^= ESTIMA_XOR;
    }
    mix
}

/// `toyota_apply_mix`.
fn apply_mix(data: &mut Payload, mix: u8) {
    data[1] ^= mix;
    data[3] ^= mix;
}

/// `toyota_recover`: the unique (plaintext, counter, second_half) for a wire payload, or `None`
/// when no candidate — or more than one — keeps the fixed id with button 0xD1.
fn recover(wire: &Payload) -> Option<(Payload, u8, bool)> {
    let fixed = fixed_id(&transform(wire));
    let mut found = None;
    for candidate in 0u8..128 {
        for half in [false, true] {
            let mut block = *wire;
            apply_masks(&mut block, candidate);
            apply_mix(&mut block, counter_mix(candidate, half));
            if fixed_id(&block) != fixed || block[3] != BASE_BUTTON {
                continue;
            }
            if found.is_some() {
                return None;
            }
            found = Some((block, candidate, half));
        }
    }
    found
}

/// `toyota_build_frame`: wire payload for a plaintext at (counter, half).
fn build_frame(plain: &Payload, counter: u8, second_half: bool) -> Payload {
    let mut out = *plain;
    apply_mix(&mut out, counter_mix(counter, second_half));
    apply_masks(&mut out, counter);
    out
}

/// `toyota_button_is_estima`.
fn button_is_estima(button: u8) -> bool {
    matches!(button, 0xD1 | 0xD4 | 0x99 | 0x9C)
}

/// `toyota_button_from_payload`: the C's `generic.btn` for a frame.
fn button_from_payload(payload: &Payload) -> u8 {
    if let Some((plain, _, _)) = recover(payload) {
        return plain[3];
    }
    let block = transform(payload);
    let mask = block[2] & 0x0E;
    if mask & 0x08 != 0 {
        3
    } else if mask & 0x04 != 0 {
        2
    } else if mask & 0x02 != 0 {
        1
    } else if button_is_estima(block[3]) {
        block[3]
    } else {
        0
    }
}

/// `toyota_generic_from_bytes`: (`data` = bytes 0..8, `extra` = bytes 8..12 right-aligned).
fn split_payload(p: &Payload) -> (u64, u64) {
    let data = p[..8].iter().fold(0u64, |acc, &b| (acc << 8) | b as u64);
    let tail = p[8..].iter().fold(0u64, |acc, &b| (acc << 8) | b as u64);
    (data, tail)
}

/// `toyota_bytes_from_generic`.
fn join_payload(data: u64, tail: u64) -> Payload {
    let mut p = [0u8; PAYLOAD_BYTES];
    for (i, b) in p[..8].iter_mut().enumerate() {
        *b = (data >> (56 - 8 * i)) as u8;
    }
    for (i, b) in p[8..].iter_mut().enumerate() {
        *b = (tail >> (24 - 8 * i)) as u8;
    }
    p
}

/// `toyota_merged_preamble`: durations accepted as a "long" besides the long tick window.
fn merged_preamble(duration: u32) -> bool {
    [TE_LONG + TE_SHORT, 2 * TE_LONG, 2 * TE_LONG + TE_SHORT]
        .iter()
        .any(|&m| duration >= m - TE_DELTA && duration <= m + TE_DELTA)
}

/// `ToyotaUpload` (only the members the reachable encoder path uses).
struct Upload {
    samples: Vec<LevelDuration>,
    phase: u8,
}

impl Upload {
    /// `toyota_upload_add`: a zero duration only sets the running level; a same-level sample is
    /// merged into the previous one.
    fn add(&mut self, level: bool, duration: u32) -> bool {
        if duration == 0 {
            return true;
        }
        if let Some(last) = self.samples.last_mut() {
            if last.level == level {
                last.duration_us += duration;
                return true;
            }
        }
        if self.samples.len() >= UPLOAD_MAX {
            return false;
        }
        self.samples.push(LevelDuration::new(level, duration));
        true
    }

    fn high_low(&mut self, high_us: u32, low_us: u32) -> bool {
        self.add(true, high_us) && self.add(false, low_us)
    }

    fn low_high(&mut self, high_us: u32, low_us: u32) -> bool {
        self.add(false, low_us) && self.add(true, high_us)
    }

    /// `toyota_encode_payload` with framing 0 and a carried phase (the only way
    /// `toyota_encode_action` calls it): 10 copies of the payload, MSB first.
    fn encode_payload(&mut self, payload: &Payload) -> bool {
        let mut state = self.phase;
        for _ in 0..BURST_COUNT {
            for &byte in payload {
                for bit in (0..8).rev() {
                    let one = (byte >> bit) & 1 != 0;
                    if state == 1 {
                        state = 3;
                    }
                    if state == 0 {
                        if one {
                            if !self.high_low(TE_LONG, 0) {
                                return false;
                            }
                            state = 1;
                        } else if !self.high_low(TE_SHORT, TE_SHORT) {
                            return false;
                        }
                    }
                    if state == 3 {
                        if one {
                            if !self.low_high(0, TE_LONG) {
                                return false;
                            }
                            state = 0;
                        } else if !self.low_high(TE_SHORT, TE_SHORT) {
                            return false;
                        }
                    }
                }
            }
        }
        self.phase = state;
        true
    }
}

/// `toyota_encode_action` (command already validated): the upload for the next code after
/// `key`, plus that next code's primary frame (the C writes it back as the new key).
fn encode_action(key: &Payload) -> Option<(Vec<LevelDuration>, Payload)> {
    let (plain, counter, _) = recover(key)?;
    let counter = counter.wrapping_sub(1) & 0x7F;
    let primary = build_frame(&plain, counter, false);
    let second = build_frame(&plain, counter, true);
    let mut upload = Upload {
        samples: Vec::with_capacity(2 * BURST_COUNT * COUNT_BIT * 2),
        phase: FOB_PHASE,
    };
    let ok = upload.encode_payload(&primary)
        && !upload.samples.is_empty()
        && upload.encode_payload(&second);
    if !ok {
        return None;
    }
    Some((upload.samples, primary))
}

/// Toyota / Lexus decoder (`SubGhzProtocolDecoderToyota`, minus the report dedup).
pub struct ToyotaDecoder {
    frame: [u8; FRAME_BYTES],
    bit_count: usize,
    short_pending: bool,
}

impl ToyotaDecoder {
    pub fn new() -> Self {
        Self {
            frame: [0; FRAME_BYTES],
            bit_count: 0,
            short_pending: false,
        }
    }

    /// `subghz_protocol_decoder_toyota_reset`.
    fn reset_state(&mut self) {
        self.frame = [0; FRAME_BYTES];
        self.bit_count = 0;
        self.short_pending = false;
    }

    /// `toyota_add_bit`: append a bit, sliding the 120-bit window once full; returns the frame
    /// when the window is aligned on a payload.
    fn add_bit(&mut self, bit: bool) -> Option<DecodedSignal> {
        if self.bit_count == FRAME_BITS {
            for i in 0..FRAME_BYTES - 1 {
                self.frame[i] = (self.frame[i] << 1) | (self.frame[i + 1] >> 7);
            }
            self.frame[FRAME_BYTES - 1] <<= 1;
            self.bit_count -= 1;
        }
        let byte = &mut self.frame[self.bit_count >> 3];
        let mask = 1u8 << (7 - (self.bit_count & 7));
        if bit {
            *byte |= mask;
        } else {
            *byte &= !mask;
        }
        self.bit_count += 1;
        if self.bit_count == FRAME_BITS
            && self.frame[0] == 0
            && self.frame[1] & 0x80 != 0
            && self.frame[..REPEAT_BYTES] == self.frame[PAYLOAD_BYTES..PAYLOAD_BYTES + REPEAT_BYTES]
        {
            return Some(self.accept());
        }
        None
    }

    /// `toyota_zero_prefix`: the last 8 bits are all zero.
    fn zero_prefix(&self) -> bool {
        if self.bit_count < 8 {
            return false;
        }
        (self.bit_count - 8..self.bit_count).all(|bit| (self.frame[bit / 8] >> (7 - bit % 8)) & 1 == 0)
    }

    /// `toyota_accept` → the fields the C publishes (`generic.data/data_2/btn/cnt/serial`).
    fn accept(&self) -> DecodedSignal {
        let mut wire = [0u8; PAYLOAD_BYTES];
        wire.copy_from_slice(&self.frame[..PAYLOAD_BYTES]);
        payload_signal(&wire)
    }
}

/// The decode the C publishes for an aligned 96-bit payload (`toyota_accept`).
fn payload_signal(wire: &Payload) -> DecodedSignal {
    match recover(wire) {
        Some((plain, counter, _)) => {
            let (data, tail) = split_payload(&build_frame(&plain, counter, false));
            DecodedSignal {
                serial: Some(fixed_id(&plain)),
                button: Some(plain[3]),
                counter: Some(counter as u16),
                crc_valid: true,
                data,
                data_count_bit: COUNT_BIT,
                encoder_capable: true,
                extra: Some(tail),
                protocol_display_name: None,
            }
        }
        None => {
            // The C publishes the raw window with serial/cnt 0 and displays the fixed id of the
            // standard transform; KAT reports that displayed id and no counter.
            let (data, tail) = split_payload(wire);
            DecodedSignal {
                serial: Some(fixed_id(&transform(wire))),
                button: Some(button_from_payload(wire)),
                counter: None,
                crc_valid: false,
                data,
                data_count_bit: COUNT_BIT,
                encoder_capable: false,
                extra: Some(tail),
                protocol_display_name: None,
            }
        }
    }
}

impl ProtocolDecoder for ToyotaDecoder {
    fn name(&self) -> &'static str {
        "Toyota"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: COUNT_BIT,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        // C flags: SubGhzProtocolFlag_315 | SubGhzProtocolFlag_433 (AM).
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.reset_state();
    }

    /// `subghz_protocol_decoder_toyota_feed` (the level is ignored).
    fn feed(&mut self, _level: bool, duration: u32) -> Option<DecodedSignal> {
        let ticks = duration / TICK_US;
        let short_pulse = ticks > SHORT_TICK_MIN && ticks <= SHORT_TICK_MAX;
        let long_pulse =
            (ticks > LONG_TICK_MIN && ticks <= LONG_TICK_MAX) || merged_preamble(duration);

        if !short_pulse && !long_pulse {
            self.reset_state();
            return None;
        }
        if short_pulse {
            let out = if self.short_pending {
                self.add_bit(false)
            } else {
                None
            };
            self.short_pending = !self.short_pending;
            out
        } else {
            if self.short_pending {
                if !self.zero_prefix() {
                    self.reset_state();
                }
                self.short_pending = false;
            }
            self.add_bit(true)
        }
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    /// Re-send the stored frame at counter-1 (`subghz_protocol_encoder_toyota_deserialize` →
    /// `toyota_get_upload`). `None` if the stored frame is not recoverable. The button argument is
    /// ignored: the C's command selection never reaches the transmitted frame.
    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        // The C deserializers require exactly 96 bits and a 32-bit tail (Key_2 high half zero).
        if decoded.data_count_bit != COUNT_BIT {
            return None;
        }
        let tail = decoded.extra.unwrap_or(0);
        if tail >> 32 != 0 {
            return None;
        }
        let key = join_payload(decoded.data, tail);
        encode_action(&key).map(|(upload, _)| upload)
    }

    /// `subghz_protocol_decoder_toyota_deserialize` (see the module doc, "Key files"). A key of 66..=68
    /// bits is FoxFW's "Toyota" and goes to Toyota (FoxFW).
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if super::toyota_foxfw::KEY_FILE_BITS.contains(&key.bits) {
            return super::toyota_foxfw::ToyotaFoxfwDecoder::new().from_flipper_key(key);
        }
        let have_copy = key.hex_bytes_field("Copy").is_some_and(|c| c.len() == PAYLOAD_BYTES);
        let tail = key.hex_field("Key_2")?;
        if tail >> 32 != 0 || key.bits != COUNT_BIT {
            return None;
        }
        let data = key.key_u64()?;
        if !have_copy && (data >> 56 != 0 || data & (1 << 55) == 0) {
            return None;
        }
        let payload = join_payload(data, tail);
        // toyota_read_button: the saved ToyotaBtn and the selected Btn must be consistent.
        let mut previous = u32::from(button_from_payload(&payload));
        if let Some(saved) = key.u32_field("ToyotaBtn") {
            if saved > 255 || (saved > 3 && !button_is_estima(saved as u8)) || (!have_copy && saved != previous) {
                return None;
            }
            previous = saved;
        }
        let selected = key.u32_field("Btn").unwrap_or(previous);
        let previous = previous as u8;
        let change_valid = selected <= 255
            && ((1..=3).contains(&selected)
                || (button_is_estima(previous) && (selected == u32::from(previous) || selected == u32::from(previous ^ ESTIMA_XOR)))
                || (previous == 0 && selected == 0));
        change_valid.then(|| payload_signal(&payload))
    }

    /// `subghz_protocol_decoder_toyota_serialize`: Key, Key_2, Btn, ToyotaBtn, Disp.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        let tail = decoded.extra.unwrap_or(0);
        if decoded.data_count_bit != COUNT_BIT || tail >> 32 != 0 {
            return None;
        }
        let payload = join_payload(decoded.data, tail);
        let button = decoded.button.unwrap_or_else(|| button_from_payload(&payload));
        let name = if button_is_estima(button) {
            "Estima"
        } else if (1..=3).contains(&button) {
            "Cmd"
        } else {
            "Unknown"
        };
        let mut key = FlipperKey::for_protocol("Toyota", COUNT_BIT, decoded.data);
        key.push_hex("Key_2", tail, 8);
        key.push_u32("Btn", button.into());
        key.push_u32("ToyotaBtn", button.into());
        key.push("Disp", format!("Toyota - {name}"));
        Some(key)
    }
}

impl Default for ToyotaDecoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::ProtocolRegistry;

    fn feed_all(pulses: &[LevelDuration]) -> Vec<DecodedSignal> {
        // Like the registry: reset after every hit.
        let mut dec = ToyotaDecoder::new();
        let mut out = Vec::new();
        for p in pulses {
            if let Some(d) = dec.feed(p.level, p.duration_us) {
                out.push(d);
                dec.reset();
            }
        }
        out
    }

    fn signal(data: u64, tail: u64) -> DecodedSignal {
        DecodedSignal {
            serial: None,
            button: None,
            counter: None,
            crc_valid: true,
            data,
            data_count_bit: COUNT_BIT,
            encoder_capable: true,
            extra: Some(tail),
            protocol_display_name: None,
        }
    }

    type Fields = (u64, Option<u64>, Option<u32>, Option<u8>, Option<u16>, bool, bool);
    fn fields(v: &[DecodedSignal]) -> Vec<Fields> {
        v.iter()
            .map(|d| (d.data, d.extra, d.serial, d.button, d.counter, d.crc_valid, d.encoder_capable))
            .collect()
    }

    fn pairs(v: &[LevelDuration]) -> Vec<(bool, u32)> {
        v.iter().map(|p| (p.level, p.duration_us)).collect()
    }

    fn hex(p: &Payload) -> String {
        p.iter().map(|b| format!("{b:02X}")).collect()
    }

    fn payload(s: &str) -> Payload {
        let mut p = [0u8; PAYLOAD_BYTES];
        for (i, b) in p.iter_mut().enumerate() {
            *b = u8::from_str_radix(&s[2 * i..2 * i + 2], 16).unwrap();
        }
        p
    }

    /// FNV-1a over (level byte, duration LE u32) — the same digest the C-oracle script printed.
    fn digest(pulses: &[LevelDuration]) -> u64 {
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        for p in pulses {
            let mut bytes = vec![p.level as u8];
            bytes.extend_from_slice(&p.duration_us.to_le_bytes());
            for b in bytes {
                h ^= b as u64;
                h = h.wrapping_mul(0x0000_0100_0000_01b3);
            }
        }
        h
    }

    #[test]
    fn timing_constants_match_c_static_asserts() {
        assert_eq!(TE_SHORT, 793);
        assert_eq!(TE_LONG, 1587);
    }

    /// Golden vectors from the unmodified C (ARF and doctor-firmware `toyota.c` compiled on the host
    /// with doctor-firmware's generic block; identical output): stored primary key → upload length,
    /// FNV-1a digest and new primary key of `subghz_protocol_encoder_toyota_deserialize`, plus the
    /// serial/counter the C decoder reports for every aligned window of that upload.
    /// (stored key, upload len, digest, new key, serial, counter of the new key)
    type Golden = (&'static str, usize, u64, &'static str, u32, u16);
    const GOLDEN_0: Golden =
        ("00EC40D1430B796999D3D27B", 3000, 0xE5AF_1585_3D33_8F95, "00984088695D543A838E9E5F", 0x37_997B, 0x7F);
    const GOLDEN_1: Golden =
        ("00F6EDC9509A9EA673125889", 2940, 0x41A5_4FE1_73EB_19FD, "00CDACD162D8B3E46B535CA9", 0x2B_6BA9, 0x00);
    const GOLDEN_2: Golden =
        ("00AF0188EAB8D57F359EB046", 2920, 0x16CF_EA72_7B6A_96D5, "00944090D8FAF83D2DDFB466", 0x0F_2F62, 0x7E);
    const GOLDEN_3: Golden =
        ("00979A81140FA798B9EE0339", 2860, 0x0ECC_667D_2E47_3F1D, "00E39AD83E598ACBA3B34F1D", 0xE8_AB39, 0x3F);

    #[test]
    fn encoder_matches_c_golden_uploads() {
        for (key, len, dig, new_key, serial, counter) in [GOLDEN_0, GOLDEN_1, GOLDEN_2, GOLDEN_3] {
            let k = payload(key);
            let (data, tail) = split_payload(&k);
            let up = ToyotaDecoder::new().encode(&signal(data, tail), 0x01).expect("encodes");
            assert_eq!(up.len(), len, "upload length for {key}");
            assert_eq!(digest(&up), dig, "upload digest for {key}");
            let (_, next) = encode_action(&k).unwrap();
            assert_eq!(hex(&next), new_key, "new key for {key}");

            // Every aligned window of the upload decodes to the new primary frame (C kat-mode
            // decode of the same upload reports these fields).
            let hits = feed_all(&up);
            assert_eq!(hits.len(), 10, "hits for {key}");
            let (nd, nt) = split_payload(&payload(new_key));
            for h in hits {
                assert_eq!((h.data, h.extra), (nd, Some(nt)));
                assert_eq!(h.serial, Some(serial));
                assert_eq!(h.button, Some(BASE_BUTTON));
                assert_eq!(h.counter, Some(counter));
                assert!(h.crc_valid && h.encoder_capable);
                assert_eq!(h.data_count_bit, 96);
            }
        }
    }

    #[test]
    fn recover_inverts_build_frame() {
        // Plaintexts like the C oracle's generator (byte0 0x00, byte1 MSB, button 0xD1). For a
        // random plaintext the C's recovery keys on the fixed id of the selector-derived standard
        // transform, so a unique recovery is not always the generating counter (the C oracle
        // measures ~1.5%); it must always re-encode to the same wire, though. With the
        // selector-input bits (b7&10 b8&08 b9&08 b9&40 b10&40 b11&04 b11&20) set to pattern 0x4E
        // or 0x5A (found with the C oracle) every (counter, half) is recovered exactly.
        let mut x: u64 = 0x2545_F491_4F6C_DD1D;
        let mut next = || {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            (x >> 24) as u8
        };
        let set = |p: &mut Payload, m: u8| {
            for (i, (byte, bit)) in
                [(7, 0x10), (8, 0x08), (9, 0x08), (9, 0x40), (10, 0x40), (11, 0x04), (11, 0x20)]
                    .into_iter()
                    .enumerate()
            {
                p[byte] = (p[byte] & !bit) | if m & (1 << i) != 0 { bit } else { 0 };
            }
        };
        let mut unique = 0;
        for round in 0..400 {
            let mut plain = [0u8; PAYLOAD_BYTES];
            for b in plain.iter_mut() {
                *b = next();
            }
            plain[0] = 0;
            plain[1] |= 0x80;
            plain[3] = BASE_BUTTON;
            let counter = next() & 0x7F;
            for half in [false, true] {
                let wire = build_frame(&plain, counter, half);
                if let Some((p, c, h)) = recover(&wire) {
                    assert_eq!(build_frame(&p, c, h), wire);
                    unique += 1;
                }
            }
            if round >= 6 {
                continue;
            }
            set(&mut plain, if round % 2 == 0 { 0x4E } else { 0x5A });
            for counter in 0..128 {
                for half in [false, true] {
                    let wire = build_frame(&plain, counter, half);
                    assert_eq!(recover(&wire), Some((plain, counter, half)));
                }
            }
        }
        assert!(unique > 100, "only {unique} unique recoveries");
    }

    /// Bi-phase-space air bits (1 = one long pulse, 0 = two shorts), levels alternating.
    fn biphase(bytes: &[u8]) -> Vec<LevelDuration> {
        let mut out: Vec<LevelDuration> = Vec::new();
        let mut level = false;
        for &byte in bytes {
            for bit in (0..8).rev() {
                let n = if (byte >> bit) & 1 != 0 { 1 } else { 2 };
                for _ in 0..n {
                    let d = if n == 1 { TE_LONG } else { TE_SHORT };
                    out.push(LevelDuration::new(level, d));
                    level = !level;
                }
            }
        }
        out
    }

    #[test]
    fn unrecoverable_frame_is_reported_but_not_transmittable() {
        // A structurally valid stream whose payload has no unique (counter, half) plaintext.
        let wire = payload("0080000000000000000000FF");
        assert!(recover(&wire).is_none());
        let pulses = biphase(&[wire, wire, wire].concat());
        let hits = feed_all(&pulses);
        assert!(!hits.is_empty());
        let (d, t) = split_payload(&wire);
        for h in &hits {
            assert_eq!((h.data, h.extra), (d, Some(t)));
            assert!(!h.crc_valid && !h.encoder_capable);
            assert_eq!(h.counter, None);
            assert_eq!(h.button, Some(button_from_payload(&wire)));
            assert_eq!(h.serial, Some(fixed_id(&transform(&wire))));
        }
        assert!(ToyotaDecoder::new().encode(&hits[0], 0x01).is_none());
    }

    #[test]
    fn level_agnostic_and_needs_aligned_repeat() {
        let (up, next) = encode_action(&payload(GOLDEN_0.0)).unwrap();
        // Inverted levels decode identically (the C ignores the level).
        let inv: Vec<LevelDuration> =
            up.iter().map(|p| LevelDuration::new(!p.level, p.duration_us)).collect();
        assert_eq!(fields(&feed_all(&up)), fields(&feed_all(&inv)));
        // One copy, or a copy followed by only 2 of the 3 repeat bytes, never fires; payload +
        // 3 repeat bytes does.
        assert!(feed_all(&biphase(&next)).is_empty());
        assert!(feed_all(&biphase(&[&next[..], &next[..2]].concat())).is_empty());
        assert_eq!(feed_all(&biphase(&[&next[..], &next[..3]].concat())).len(), 1);
        // A payload whose byte 0 is not 0x00 never aligns.
        let mut odd = next;
        odd[0] = 0x01;
        assert!(feed_all(&biphase(&[odd, odd, odd].concat())).is_empty());
        // Out-of-window pulses reset the window.
        let mut broken = up.clone();
        for p in broken.iter_mut().step_by(150) {
            p.duration_us = 400;
        }
        assert!(feed_all(&broken).is_empty());
    }

    #[test]
    fn encode_rejects_bad_shapes() {
        let key = payload(GOLDEN_0.0);
        let (d, t) = split_payload(&key);
        let dec = ToyotaDecoder::new();
        let mut s = signal(d, t);
        s.data_count_bit = 68;
        assert!(dec.encode(&s, 1).is_none());
        let s = signal(d, t | (1 << 40));
        assert!(dec.encode(&s, 1).is_none());
        // Every KAT button produces the same upload (the payload carries no command).
        let a = dec.encode(&signal(d, t), 0x01).unwrap();
        for b in [0x02, 0x04, 0x08, 0xD1] {
            assert_eq!(pairs(&dec.encode(&signal(d, t), b).unwrap()), pairs(&a));
        }
    }

    /// Cross-protocol guard for Toyota, Porsche Cayenne and Land Rover RKE (same shape as the
    /// Mercedes/Opel guard in mercedes_v1.rs): no other registry protocol's own encoder output, at
    /// each of its frequencies, decodes as one of them; and each of their encoders' output decodes
    /// only as itself.
    #[test]
    fn foreign_encoder_output_is_never_claimed() {
        use std::panic::{catch_unwind, AssertUnwindSafe};
        const MINE: [&str; 3] = ["Toyota", "Porsche Cayenne", "Land Rover RKE"];
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
        // Own output → only itself (Toyota needs a recoverable key; the others take any fields).
        // A protocol left out of the registry (e.g. Land Rover RKE, which ARF never registers) is
        // skipped.
        let toyota_key = split_payload(&payload(GOLDEN_2.0));
        for name in MINE {
            let Some(dec) = reg0.get(name) else {
                continue;
            };
            let src = DecodedSignal {
                serial: Some(0x0012_3456),
                button: Some(0x02),
                counter: Some(0x0321),
                crc_valid: true,
                data: if name == "Toyota" { toyota_key.0 } else { 0xA5C3_1E7B_92D4_6F08 },
                data_count_bit: dec.timing().min_count_bit,
                encoder_capable: true,
                extra: Some(if name == "Toyota" { toyota_key.1 } else { 0x2 }),
                protocol_display_name: None,
            };
            let wire = dec.encode(&src, 0x01).expect("encodes");
            for &f in dec.supported_frequencies() {
                let mut reg = ProtocolRegistry::new();
                let names: Vec<String> =
                    reg.process_signal_stream(&wire, f).into_iter().map(|(n, _, _)| n).collect();
                assert!(!names.is_empty() && names.iter().all(|n| n == name), "{name} at {f}: {names:?}");
            }
        }
    }

    /// The registry reports Toyota for its own upload, and no other protocol claims it.
    #[test]
    fn registry_decodes_toyota_upload_only_as_toyota() {
        let key = payload(GOLDEN_1.0);
        let (up, _) = encode_action(&key).unwrap();
        for f in [315_000_000, 433_920_000, 312_000_000, 314_350_000] {
            let mut reg = ProtocolRegistry::new();
            let names: Vec<String> =
                reg.process_signal_stream(&up, f).into_iter().map(|(n, _, _)| n).collect();
            assert!(!names.is_empty() && names.iter().all(|n| n == "Toyota"), "{f}: {names:?}");
        }
    }

    /// Key files: recovered and unrecovered frames round-trip with the C's lines (Key_2 always
    /// written), doctor-firmware's "Toyota Lexus" name loads, the C's Key_2/start/button checks
    /// refuse bad files, and TX from a loaded key matches TX from the decode.
    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::{assert_round_trip, key};
        let dec = ToyotaDecoder::new();
        let (data, tail) = split_payload(&payload(GOLDEN_0.0));
        let up = dec.encode(&signal(data, tail), 0x01).unwrap();
        let decoded = feed_all(&up).remove(0);
        assert!(decoded.encoder_capable);
        let text = assert_round_trip(&dec, &decoded);
        let p = join_payload(decoded.data, decoded.extra.unwrap());
        let bytes = |b: &[u8]| b.iter().map(|x| format!("{x:02X}")).collect::<Vec<_>>().join(" ");
        assert!(
            text.ends_with(&format!(
                "Protocol: Toyota\nBit: 96\nKey: {}\nKey_2: 00 00 00 00 {}\nBtn: 209\nToyotaBtn: 209\n\
                 Disp: Toyota - Estima\n",
                bytes(&p[..8]),
                bytes(&p[8..])
            )),
            "{text}"
        );
        // An unrecoverable window (the C's raw publish) round-trips too.
        let mut raw = p;
        raw[5] ^= 0x40;
        let odd = payload_signal(&raw);
        if !odd.crc_valid {
            assert_round_trip(&dec, &odd);
        }
        // doctor-firmware's name and generic lines (Serial/Cnt/Type) load the same frame.
        let reg = crate::protocols::ProtocolRegistry::new();
        let doctor = text.replace("Protocol: Toyota\n", "Protocol: Toyota Lexus\n").replace(
            "Key_2:",
            "Serial: 1234\nCnt: 1\nType: 31\nKey_2:",
        );
        assert_eq!(reg.decode_key(&key(&doctor)), Some(("Toyota".to_string(), decoded.clone())));
        // Refusals (toyota deserialize): Key_2 missing or with a non-zero high half, a key not
        // starting 0x00/bit 55, an inconsistent ToyotaBtn, an invalid Btn change.
        let mut k = key(&text);
        k.fields.retain(|(n, _)| n != "Key_2");
        assert!(dec.from_flipper_key(&k).is_none());
        assert!(dec.from_flipper_key(&key(&text.replace("Key_2: 00 00 00 00", "Key_2: 00 00 00 01"))).is_none());
        assert!(dec.from_flipper_key(&key(&text.replace("ToyotaBtn: 209", "ToyotaBtn: 212"))).is_none());
        assert!(dec.from_flipper_key(&key(&text.replace("Btn: 209\nToyota", "Btn: 7\nToyota"))).is_none());
        // Btn 2 (a command) is a valid selection; the payload decides the decode.
        assert_eq!(dec.from_flipper_key(&key(&text.replace("Btn: 209\nToyota", "Btn: 2\nToyota"))), Some(decoded.clone()));

        let loaded = dec.from_flipper_key(&key(&text)).unwrap();
        assert_eq!(pairs(&dec.encode(&loaded, 0x01).unwrap()), pairs(&dec.encode(&decoded, 0x01).unwrap()));
    }

    /// Oracle vectors (`KAT_KEY_VECTORS=<dir> cargo test toyota_key_vectors -- --ignored`).
    #[test]
    #[ignore]
    fn toyota_key_vectors() {
        use super::super::bmw::{key_vector_dir, write_key_vector};
        let dec = ToyotaDecoder::new();
        for (n, g) in [(1, GOLDEN_0), (2, GOLDEN_2)] {
            let (data, tail) = split_payload(&payload(g.0));
            let up = dec.encode(&signal(data, tail), 0x01).unwrap();
            let decoded = feed_all(&up).remove(0);
            let key = dec.to_flipper_key(&decoded).unwrap();
            write_key_vector(&key_vector_dir(), &format!("toyota_{n}"), 433_920_000, "FuriHalSubGhzPresetOok650Async", &up, &key);
        }
    }
}
