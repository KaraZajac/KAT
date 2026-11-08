//! Toyota (FoxFW): FoxFW's "Toyota", a KeeLoq-layout 400/800 µs PWM decoder (Prius 2006 / Corolla
//! Verso) with a 200/390 µs NRZ variant (Tundra) — **decode and display only**.
//!
//! Ported from FoxFW `lib/subghz/protocols/toyota.c` / `toyota.h` (`SUBGHZ_PROTOCOL_TOYOTA_NAME =
//! "Toyota"`; flags 433 | 315 | AM | Decodable | Load | Save | Send, type Dynamic). Its decoder is
//! Flipper-ARF's retired dual-variant KeeLoq Toyota (`toyota-old.c.bak`, which ARF replaced with the
//! Toyota/Lexus protocol of KAT's `toyota.rs`) line for line, comments stripped, plus the `Tail` key-file
//! line (`generic.data_2`). FoxFW's encoder is an SD-card plugin (`SUBGHZ_ENCODER_EXTERNAL("enc_toyota")`)
//! that KAT does not port: `supports_encoding()` is false and `encode` returns None.
//!
//! **Name.** "Toyota" is KAT's `toyota.rs` (Flipper-ARF / doctor-firmware's 96-bit ~793/1587 µs Estima
//! frame, a different air format), so this decoder registers as "Toyota (FoxFW)", as "Renault V1 (ARF)"
//! sits beside ProtoPirate's "Renault V1". Its decodes carry that name as `protocol_display_name`, so a
//! key file (`Protocol: Toyota`, handed over by `toyota.rs`) loads back under it.
//!
//! **Variant selection** (reset step; LOWs are ignored there): a HIGH with |d - 200| < 60
//! (`toyota_const_b`) and d < 310 starts variant B; one with |d - 400| < 100 (`toyota_const_a`) and
//! d >= 310 starts variant C (the C's `fits_a` branch sets `variant = 2`); any other HIGH is ignored.
//! The selecting HIGH is the first preamble HIGH.
//!
//! **Variant B** (Tundra). Preamble: pairs of a short HIGH (|d - 200| < 60) and a long LOW
//! (|d - 390| < 60), counted; any other HIGH, a LOW with no HIGH before it, or any other LOW resets. A
//! LOW of 1700..=2200 µs after at least 10 pairs starts the data (after fewer: reset). Data: every pulse,
//! HIGH or LOW, is one bit, 1 when longer than 287 µs; a pulse of 1700 µs or more ends the frame (no
//! report: the 67th bit has already reported it) and one outside 100..=500 µs resets. The 67th bit
//! reports the frame, then the decoder resets.
//!
//! **Variant C** (Prius / Corolla Verso). Preamble: pairs of a HIGH of 200..=550 µs and a LOW of
//! 200..=650 µs (the HIGH is judged on its LOW), counted; a LOW of 1050..=1500 µs after at least 6
//! pairs starts the data (after fewer: reset); any other LOW, or a LOW with no HIGH before it, resets.
//! Data: PWM on the HIGH, which must be 200..=1050 µs, a 1 when longer than 600 µs; the first 68 bits
//! are kept, later HIGHs are accepted and dropped. LOWs are separators and are not measured, except
//! (in this order): a LOW of 1050..=1500 µs reports the frame when it has 66 or more bits and starts
//! the next frame's data at once (no preamble); any other LOW over 1000 µs reports it the same way and
//! returns to the preamble step, skipping the reset step's variant test (the next HIGH is taken
//! whatever its length and judged on the LOW after it). A HIGH outside 200..=1050 µs ends the frame
//! the same way and is kept as the first HIGH of a new preamble. Each Prius frame has 69 data HIGHs
//! (68 kept) and is reported on the long LOW after it (8.3 ms between the frames of a press).
//!
//! **Variant A** (Corolla, 400/800 µs LS/SL pairs, `toyota_feed_variant_a`, and its button list in
//! `toyota_frame_plausible`) is dead code in the C: nothing ever sets its steps (the reset step sends the
//! 400 µs path to variant C, as the C's comment on its encoder says), so it is not ported.
//!
//! **Fields** (`toyota_decode_and_fire` for B, `toyota_decode_and_fire_c` for C): bits in arrival
//! order, the first the most significant: hop = bits 0..32, serial = bits 32..60 (28 bits), button =
//! bits 60..64. `toyota_frame_plausible` drops a frame whose serial is 0 or 0x0FFFFFFF, whose hop is 0 or
//! 0xFFFFFFFF, or whose button is outside its variant's list: C 0xB Lock, 0xE Unlock, 0xD Trunk, 0x7
//! Panic; B 0xA Lock, 0x5 Unlock, 0xF, 0x4 (the last two named nowhere in FoxFW). No checksum, and the
//! hop is not decrypted (a long HIGH is a 1 here, the inverse of Flipper's KeeLoq decoder).
//!
//! **KAT fields**: `data` = hop << 32 | serial << 4 | button (the C's `generic.data`, the frame's first
//! 64 bits), `data_count_bit` = the bit count (B 67, C 66..=68), `extra` = the bit_count - 64 trailing
//! bits (`generic.data_2`, the key file's `Tail`), `serial`, `button`, `counter` = the variant (the C's
//! `generic.cnt`: 1 = B, 2 = C), `crc_valid` false, `encoder_capable` false.
//!
//! **Callback semantics**: the C sets its state after the decode callback, where the receiver resets
//! every decoder: after a frame reported on a sync LOW the next frame's data follows at once, after one
//! reported on a long LOW or HIGH a preamble step. KAT keeps that state through the registry's reset
//! that follows its own report (`just_emitted`). `reset()` leaves `variant` alone, as the C's does;
//! every way out of the reset step sets it again first, so the stale value is never read.
//!
//! **Key file** (`serialize` / `deserialize`): `Protocol: Toyota`, `Bit`, `Key` (8 bytes; FoxFW's
//! generic serializer writes no Serial / Btn / Cnt), then `Tail` (decimal). Loading needs `Bit` >= 66
//! and an 8-byte `Key`, re-derives hop / serial / button from it and reads `Tail` (0 when absent, as in
//! files saved before FoxFW v2.0.12). KAT's `toyota.rs` owns the name and hands this decoder the keys of
//! 66..=68 bits (its own have 96).
//!
//! **Deviation**: the C's deserialize takes the variant from the decoder instance's previous `cnt` (the
//! file has none); KAT derives it from the button (the two lists do not overlap), so a reloaded key keeps
//! the variant it was decoded with; a button in neither list loads as variant 0. Nothing else differs.
//!
//! **Evidence**: `IMPORTS/Toyota + Lexus/Toyota_Prius2006_lock.sub` / `_unlock.sub`, 8 frames each (68
//! bits, hop 0x3F2E5098 / 0x29CD45FF, serial 0x7231804, button 0xB / 0xE, tail 0xF), identical to the
//! host build of FoxFW's C frame for frame; so are the tests' synthetic bursts (both variants, every
//! button, the frame ends and restarts, the refused frames) and 416 perturbed copies of the 16 base
//! frames (jitter, scaling, dropped / split pulses, truncation, noise, repeats).
//!
//! **Kia V3** has variant C's air format (HIGH 400 + LOW 1200 sync, 400/800 µs PWM) and its lock /
//! unlock buttons read 0xB / 0xD here: FoxFW's decoder alone reports the frames of
//! `IMPORTS/KIA/KIA V3 N1 RAW.sub` (its full registry reports KIA/HYU V3/V4, which completes first).
//! Those files are FSK, and a `.sub` import feeds this AM-only decoder only AM captures. On KAT's own
//! AM receive path Kia V3/V4 (FM-flagged) runs only in the ungated second pass: `NOT_FINAL_ON_GATED_PASS`
//! (`mod.rs`) runs that pass when this decoder's frames are all the gated pass found, so an OOK Kia V3
//! fob still decodes as Kia V3 (the registry order then gives Kia V3/V4 the frame, as in FoxFW).

use super::foxfw_common::duration_diff;
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (see `info.rs`): `SubGhzProtocolFlag_433 | 315 | AM`, `SubGhzProtocolTypeDynamic`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "Toyota",
    category: Category::Vehicle,
    buttons: Some(button_name),
    presets: &[&AM650],
};

const NAME: &str = "Toyota (FoxFW)";
/// `SUBGHZ_PROTOCOL_TOYOTA_NAME`, the name FoxFW writes in key files.
const KEY_FILE_NAME: &str = "Toyota";

/// `toyota_const_a` (only its short window is used: the reset step's variant C test).
const A_TE_SHORT: u32 = 400;
const A_TE_LONG: u32 = 800;
const A_TE_DELTA: u32 = 100;
/// `toyota_const_b`.
const B_TE_SHORT: u32 = 200;
const B_TE_LONG: u32 = 390;
const B_TE_DELTA: u32 = 60;
const B_MIN_COUNT_BIT: u8 = 67;
const B_NRZ_MIDPOINT: u32 = 287;
const B_SYNC_GAP_MIN: u32 = 1700;
const B_SYNC_GAP_MAX: u32 = 2200;
const B_PREAMBLE_MIN: u16 = 10;
const B_DATA_PULSE_MIN: u32 = 100;
const B_DATA_PULSE_MAX: u32 = 500;
const B_BITS: u8 = 67;
const VARIANT_THRESH: u32 = 310;
const C_BITS: u8 = 66;
const C_FRAME_BITS: u8 = 68;
const C_PREAMBLE_MIN: u16 = 6;
const C_SYNC_MIN: u32 = 1050;
const C_SYNC_MAX: u32 = 1500;
const C_HIGH_MIDPOINT: u32 = 600;
const C_PRE_HIGH_MIN: u32 = 200;
const C_PRE_HIGH_MAX: u32 = 550;
const C_PRE_LOW_MIN: u32 = 200;
const C_PRE_LOW_MAX: u32 = 650;
const C_DATA_HIGH_MIN: u32 = 200;
const C_DATA_HIGH_MAX: u32 = 1050;
const C_END_LOW: u32 = 1000;

const B_BTN_LOCK: u8 = 0x0A;
const B_BTN_UNLOCK: u8 = 0x05;
const C_BTN_LOCK: u8 = 0x0B;
const C_BTN_UNLOCK: u8 = 0x0E;
const C_BTN_TRUNK: u8 = 0x0D;
const C_BTN_PANIC: u8 = 0x07;
/// `toyota_frame_plausible`'s button lists.
const B_BUTTONS: [u8; 4] = [B_BTN_LOCK, B_BTN_UNLOCK, 0x0F, 0x04];
const C_BUTTONS: [u8; 4] = [C_BTN_LOCK, C_BTN_UNLOCK, C_BTN_TRUNK, C_BTN_PANIC];

/// `variant` (the C's `generic.cnt`): 1 = B (Tundra), 2 = C (Prius / Corolla Verso).
const VARIANT_B: u8 = 1;
const VARIANT_C: u8 = 2;

/// The `Bit` values of the key files this decoder writes (`toyota.rs` hands these keys over).
pub(super) const KEY_FILE_BITS: std::ops::RangeInclusive<usize> = C_BITS as usize..=C_FRAME_BITS as usize;

/// The C's button `#define`s (the two variants' codes do not overlap); "Unknown" for the rest.
fn button_name(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        C_BTN_LOCK | B_BTN_LOCK => "Lock",
        C_BTN_UNLOCK | B_BTN_UNLOCK => "Unlock",
        C_BTN_TRUNK => "Trunk",
        C_BTN_PANIC => "Panic",
        _ => "Unknown",
    })
}

/// `te_is_short` / `te_is_long`: |d - te| < delta.
fn near(duration: u32, te: u32, delta: u32) -> bool {
    duration_diff(duration, te) < delta
}

/// `toyota_frame_plausible` (variants B and C; variant A's list is unreachable).
fn plausible(serial: u32, button: u8, hop: u32, variant: u8) -> bool {
    if serial == 0 || serial == 0x0FFF_FFFF || hop == 0 || hop == 0xFFFF_FFFF {
        return false;
    }
    let list = if variant == VARIANT_C { &C_BUTTONS } else { &B_BUTTONS };
    list.contains(&(button & 0x0F))
}

/// The variant a button belongs to (KAT's stand-in for the C's stale `cnt`, see the module doc).
fn variant_of(button: u8) -> u8 {
    if C_BUTTONS.contains(&button) {
        VARIANT_C
    } else if B_BUTTONS.contains(&button) {
        VARIANT_B
    } else {
        0
    }
}

/// A decode (or a loaded key) of `data` (`generic.data`) with `bits` bits, the `tail`
/// (`generic.data_2`) and the variant (`generic.cnt`).
fn decoded(data: u64, bits: usize, tail: u32, variant: u8) -> DecodedSignal {
    DecodedSignal {
        serial: Some(((data >> 4) & 0x0FFF_FFFF) as u32),
        button: Some((data & 0x0F) as u8),
        counter: Some(u16::from(variant)),
        crc_valid: false,
        data,
        data_count_bit: bits,
        encoder_capable: false,
        extra: Some(u64::from(tail)),
        protocol_display_name: Some(NAME.to_string()),
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    PreambleB,
    DataB,
    PreambleC,
    DataC,
}

/// Toyota (FoxFW) decoder (`SubGhzProtocolDecoderToyota`).
pub struct ToyotaFoxfwDecoder {
    step: Step,
    /// The 72-bit shift register (`bits_hi` holds the bits shifted out of `bits_lo`).
    bits_lo: u64,
    bits_hi: u8,
    bit_count: u8,
    te_last: u32,
    have_high: bool,
    preamble_count: u16,
    /// [VARIANT_B] or [VARIANT_C]; 0 before the first frame. `reset()` keeps it, as the C's does.
    variant: u8,
    /// The last `feed` reported a frame: keep the post-callback state through the registry's reset.
    just_emitted: bool,
}

impl ToyotaFoxfwDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            bits_lo: 0,
            bits_hi: 0,
            bit_count: 0,
            te_last: 0,
            have_high: false,
            preamble_count: 0,
            variant: 0,
            just_emitted: false,
        }
    }

    /// `subghz_protocol_decoder_toyota_reset` (`variant` is not cleared).
    fn c_reset(&mut self) {
        self.step = Step::Reset;
        self.bits_lo = 0;
        self.bits_hi = 0;
        self.bit_count = 0;
        self.te_last = 0;
        self.have_high = false;
        self.preamble_count = 0;
    }

    /// `toyota_push_bit`.
    fn push_bit(&mut self, bit: bool) {
        let carry = (self.bits_lo >> 63) as u8 & 1;
        self.bits_hi = (self.bits_hi << 1) | carry;
        self.bits_lo = (self.bits_lo << 1) | u64::from(bit);
        self.bit_count += 1;
    }

    /// `toyota_extract`: `length` bits from frame bit `offset` (0 = the first received), MSB first.
    fn extract(&self, offset: u8, length: u8) -> u32 {
        let total = i32::from(self.bit_count);
        (0..length).fold(0u32, |acc, i| {
            let pos = (total - 1) - i32::from(offset + i);
            let bit = if pos >= 64 {
                u32::from((self.bits_hi >> (pos - 64)) & 1)
            } else if pos >= 0 {
                ((self.bits_lo >> pos) & 1) as u32
            } else {
                0
            };
            (acc << 1) | bit
        })
    }

    /// The fields both fire functions extract, None when `toyota_frame_plausible` refuses them.
    fn frame(&self, variant: u8) -> Option<DecodedSignal> {
        let hop = self.extract(0, 32);
        let serial = self.extract(32, 28);
        let button = (self.extract(60, 4) & 0x0F) as u8;
        if !plausible(serial, button, hop, variant) {
            return None;
        }
        let data = (u64::from(hop) << 32) | (u64::from(serial) << 4) | u64::from(button);
        let tail = if self.bit_count > 64 { self.extract(64, self.bit_count - 64) } else { 0 };
        Some(decoded(data, usize::from(self.bit_count), tail, variant))
    }

    /// `toyota_decode_and_fire`, reached from variant B only (`toyota_const_b`: 67 bits at least).
    fn decode_and_fire(&mut self) -> Option<DecodedSignal> {
        if self.bit_count < B_MIN_COUNT_BIT {
            return None;
        }
        self.frame(self.variant)
    }

    /// `toyota_decode_and_fire_c`.
    fn decode_and_fire_c(&mut self) -> Option<DecodedSignal> {
        if self.bit_count < C_BITS {
            return None;
        }
        let out = self.frame(VARIANT_C)?;
        self.variant = VARIANT_C;
        Some(out)
    }

    /// `toyota_start_data_c`.
    fn start_data_c(&mut self) {
        self.bits_lo = 0;
        self.bits_hi = 0;
        self.bit_count = 0;
        self.have_high = false;
        self.step = Step::DataC;
    }

    /// `toyota_feed_variant_b`.
    fn feed_variant_b(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::PreambleB => {
                if level {
                    if near(duration, B_TE_SHORT, B_TE_DELTA) {
                        self.te_last = duration;
                        self.have_high = true;
                    } else {
                        self.c_reset();
                    }
                    return None;
                }
                if !self.have_high {
                    self.c_reset();
                    return None;
                }
                self.have_high = false;
                if (B_SYNC_GAP_MIN..=B_SYNC_GAP_MAX).contains(&duration) {
                    if self.preamble_count >= B_PREAMBLE_MIN {
                        self.bits_lo = 0;
                        self.bits_hi = 0;
                        self.bit_count = 0;
                        self.have_high = false;
                        self.step = Step::DataB;
                    } else {
                        self.c_reset();
                    }
                    return None;
                }
                if near(duration, B_TE_LONG, B_TE_DELTA) {
                    self.preamble_count = self.preamble_count.wrapping_add(1);
                    return None;
                }
                self.c_reset();
                None
            }
            Step::DataB => {
                if duration >= B_SYNC_GAP_MIN {
                    let out = if self.bit_count >= B_MIN_COUNT_BIT { self.decode_and_fire() } else { None };
                    self.c_reset();
                    return out;
                }
                if !(B_DATA_PULSE_MIN..=B_DATA_PULSE_MAX).contains(&duration) {
                    self.c_reset();
                    return None;
                }
                self.push_bit(duration > B_NRZ_MIDPOINT);
                if self.bit_count >= B_BITS {
                    let out = self.decode_and_fire();
                    self.c_reset();
                    return out;
                }
                None
            }
            _ => None,
        }
    }

    /// `toyota_feed_variant_c`. Where the C reports, its callback resets the receiver; the code after
    /// it sets the state kept here.
    fn feed_variant_c(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::PreambleC => {
                if level {
                    self.te_last = duration;
                    self.have_high = true;
                    return None;
                }
                if !self.have_high {
                    self.c_reset();
                    return None;
                }
                self.have_high = false;
                if (C_SYNC_MIN..=C_SYNC_MAX).contains(&duration) {
                    if self.preamble_count >= C_PREAMBLE_MIN {
                        self.start_data_c();
                    } else {
                        self.c_reset();
                    }
                    return None;
                }
                if (C_PRE_HIGH_MIN..=C_PRE_HIGH_MAX).contains(&self.te_last)
                    && (C_PRE_LOW_MIN..=C_PRE_LOW_MAX).contains(&duration)
                {
                    self.preamble_count = self.preamble_count.wrapping_add(1);
                    return None;
                }
                self.c_reset();
                None
            }
            Step::DataC => {
                if level {
                    if (C_DATA_HIGH_MIN..=C_DATA_HIGH_MAX).contains(&duration) {
                        if self.bit_count < C_FRAME_BITS {
                            self.push_bit(duration > C_HIGH_MIDPOINT);
                        }
                        self.have_high = true;
                        self.te_last = duration;
                        return None;
                    }
                    // An out-of-range HIGH ends the frame and opens a new preamble with itself.
                    let out = if self.bit_count >= C_BITS { self.decode_and_fire_c() } else { None };
                    self.c_reset();
                    self.variant = VARIANT_C;
                    self.step = Step::PreambleC;
                    self.te_last = duration;
                    self.have_high = true;
                    return out;
                }
                self.have_high = false;
                // The sync window first: the next frame's data follows at once.
                if (C_SYNC_MIN..=C_SYNC_MAX).contains(&duration) {
                    let out = if self.bit_count >= C_BITS { self.decode_and_fire_c() } else { None };
                    self.start_data_c();
                    return out;
                }
                // Any other long LOW ends the frame; back to the preamble (not the reset step).
                if duration > C_END_LOW {
                    let out = if self.bit_count >= C_BITS { self.decode_and_fire_c() } else { None };
                    self.c_reset();
                    self.variant = VARIANT_C;
                    self.step = Step::PreambleC;
                    self.preamble_count = 0;
                    return out;
                }
                None
            }
            _ => None,
        }
    }
}

impl Default for ToyotaFoxfwDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for ToyotaFoxfwDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    /// `toyota_const_a` (variant C's nominal PWM; its short window selects variant C) and variant C's
    /// minimum frame.
    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: A_TE_SHORT, te_long: A_TE_LONG, te_delta: A_TE_DELTA, min_count_bit: usize::from(C_BITS) }
    }

    fn supported_frequencies(&self) -> &[u32] {
        // C flags: SubGhzProtocolFlag_433 | SubGhzProtocolFlag_315 | AM.
        &[315_000_000, 433_920_000]
    }

    /// `subghz_protocol_decoder_toyota_reset` (skipped right after this decoder's own report: the C
    /// code after its callback overrides the reset).
    fn reset(&mut self) {
        if std::mem::take(&mut self.just_emitted) {
            return;
        }
        self.c_reset();
    }

    /// `subghz_protocol_decoder_toyota_feed`.
    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        self.just_emitted = false;
        if self.step == Step::Reset {
            if !level {
                return None;
            }
            let fits_b = near(duration, B_TE_SHORT, B_TE_DELTA) && duration < VARIANT_THRESH;
            let fits_a = near(duration, A_TE_SHORT, A_TE_DELTA) && duration >= VARIANT_THRESH;
            if fits_b || fits_a {
                self.variant = if fits_b { VARIANT_B } else { VARIANT_C };
                self.te_last = duration;
                self.have_high = true;
                self.preamble_count = 0;
                self.step = if fits_b { Step::PreambleB } else { Step::PreambleC };
            }
            return None;
        }
        // Dispatched on `variant` as in the C (variant 0, `toyota_feed_variant_a`, is never reached).
        let out = match self.variant {
            VARIANT_B => self.feed_variant_b(level, duration),
            VARIANT_C => self.feed_variant_c(level, duration),
            _ => None,
        };
        self.just_emitted = out.is_some();
        out
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    /// FoxFW's encoder is an SD-card plugin outside this port.
    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }

    /// `subghz_protocol_decoder_toyota_deserialize`: `Bit` >= 66, the 8-byte `Key`, optional `Tail`
    /// (see the module doc for the variant).
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits < usize::from(C_BITS) {
            return None;
        }
        let data = key.key_u64()?;
        let tail = key.u32_field("Tail").unwrap_or(0);
        Some(decoded(data, key.bits, tail, variant_of((data & 0x0F) as u8)))
    }

    /// `subghz_protocol_decoder_toyota_serialize`: the generic `Protocol` / `Bit` / `Key`, then `Tail`.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit < usize::from(C_BITS) {
            return None;
        }
        let mut key = FlipperKey::for_protocol(KEY_FILE_NAME, decoded.data_count_bit, decoded.data);
        key.push_u32("Tail", decoded.extra.unwrap_or(0) as u32);
        Some(key)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::f_common::test_support::sub_pulses;
    use crate::protocols::flipper_key::testing::{assert_round_trip, key};
    use crate::protocols::foxfw_common::test_support::{decode_all, dump};
    use crate::protocols::ProtocolRegistry;

    fn wire(raw: &[i32]) -> Vec<LevelDuration> {
        raw.iter().map(|&v| LevelDuration::new(v > 0, v.unsigned_abs())).collect()
    }

    /// A variant C frame shaped like the Prius's: a 10 ms LOW, `pre` preamble pairs (HIGH 380 / LOW
    /// 440), the sync (HIGH 380 / LOW 1250), then one HIGH per bit (800 = 1, 400 = 0) followed by a
    /// LOW of 450 / 850; the last bit's LOW is the 8 ms end gap.
    fn frame_c(bits: &[bool], pre: usize) -> Vec<i32> {
        let mut w = vec![-10_000];
        for _ in 0..pre {
            w.extend([380, -440]);
        }
        w.extend([380, -1250]);
        for &b in bits {
            w.extend(if b { [800, -450] } else { [400, -850] });
        }
        *w.last_mut().unwrap() = -8_000;
        w
    }

    /// A variant B frame: a 10 ms LOW, `pre` preamble pairs (HIGH 200 / LOW 390), the sync (HIGH 200
    /// / LOW 1940), then one pulse per bit (390 = 1, 200 = 0, levels alternating from HIGH) and a
    /// 10 ms gap.
    fn frame_b(bits: &[bool], pre: usize) -> Vec<i32> {
        let mut w = vec![-10_000];
        for _ in 0..pre {
            w.extend([200, -390]);
        }
        w.extend([200, -1940]);
        for (i, &b) in bits.iter().enumerate() {
            let us = if b { 390 } else { 200 };
            w.push(if i.is_multiple_of(2) { us } else { -us });
        }
        w.push(if bits.len().is_multiple_of(2) { 10_000 } else { -10_000 });
        w
    }

    /// A frame's bits: hop (32), serial (28), button (4), then `tail_len` bits of `tail`.
    fn bits(hop: u32, serial: u32, button: u8, tail: u32, tail_len: usize) -> Vec<bool> {
        let mut out: Vec<bool> = (0..32).rev().map(|i| (hop >> i) & 1 == 1).collect();
        out.extend((0..28).rev().map(|i| (serial >> i) & 1 == 1));
        out.extend((0..4).rev().map(|i| (button >> i) & 1 == 1));
        out.extend((0..tail_len).rev().map(|i| (tail >> i) & 1 == 1));
        out
    }

    /// The accepted synthetic frames (made-up values): C at 66 / 67 / 68 bits with each button, B
    /// at 67 bits with each button, and C with 69 data HIGHs.
    fn synth_cases() -> Vec<(String, Vec<i32>)> {
        let mut out = Vec::new();
        for (len, tail, tail_len) in [(66, 0b10, 2), (67, 0b101, 3), (68, 0b1011, 4)] {
            for btn in C_BUTTONS {
                out.push((format!("c{len}_{btn:x}"), frame_c(&bits(0x1234_ABCD, 0x0A5_C3E1, btn, tail, tail_len), 8)));
            }
        }
        for btn in B_BUTTONS {
            out.push((format!("b67_{btn:x}"), frame_b(&bits(0x5EED_1234, 0x0BE_EF42, btn, 0b011, 3), 12)));
        }
        out.push(("c69".into(), frame_c(&bits(0x1234_ABCD, 0x0A5_C3E1, 0xB, 0b10110, 5), 8)));
        out
    }

    /// Frames the C refuses: button 0x1, serial 0, hop 0xFFFFFFFF, 65 bits, a 1600 µs sync, 5
    /// preamble pairs; variant B with C's Lock code 0xB, and with 66 bits.
    fn reject_cases() -> Vec<(String, Vec<i32>)> {
        let (hop, serial) = (0x1234_ABCD, 0x0A5_C3E1);
        let mut bad_sync = frame_c(&bits(hop, serial, 0xB, 0b1011, 4), 8);
        bad_sync[18] = -1600;
        vec![
            ("c68_btn1".into(), frame_c(&bits(hop, serial, 0x1, 0b1011, 4), 8)),
            ("c68_serial0".into(), frame_c(&bits(hop, 0, 0xB, 0b1011, 4), 8)),
            ("c68_hopff".into(), frame_c(&bits(0xFFFF_FFFF, serial, 0xB, 0b1011, 4), 8)),
            ("c65".into(), frame_c(&bits(hop, serial, 0xB, 0b1011, 4)[..65], 8)),
            ("c68_sync1600".into(), bad_sync),
            ("c68_pre5".into(), frame_c(&bits(hop, serial, 0xB, 0b1011, 4), 5)),
            ("b67_btnb".into(), frame_b(&bits(0x5EED_1234, 0x0BE_EF42, 0xB, 0b011, 3), 12)),
            ("b66".into(), frame_b(&bits(0x5EED_1234, 0x0BE_EF42, 0xA, 0b011, 3)[..66], 12)),
        ]
    }

    /// Bursts that exercise the C's frame ends and restarts.
    fn sequence_cases() -> Vec<(String, Vec<i32>)> {
        let (hop, serial) = (0x1234_ABCD, 0x0A5_C3E1);
        let frame = |btn, tail, len| frame_c(&bits(hop, serial, btn, tail, len), 8);
        // A sync LOW instead of the end gap, then the next frame's data with no preamble.
        let mut sync_restart = frame(0xB, 0b1011, 4);
        *sync_restart.last_mut().unwrap() = -1250;
        sync_restart.extend(&frame(0xE, 0b0110, 4)[19..]);
        // A 1200 µs HIGH after the last bit's separator, then a full frame.
        let mut long_high = frame(0xD, 0b101, 3);
        *long_high.last_mut().unwrap() = -450;
        long_high.push(1200);
        long_high.extend(frame(0x7, 0b101, 3));
        // A 1020 µs LOW (over 1000, under the sync window) ends the frame; the next one follows at
        // once with 6 preamble pairs, the first HIGH 250 µs (a variant B HIGH to the reset step).
        let mut end_1020 = frame(0x7, 0b10, 2);
        *end_1020.last_mut().unwrap() = -1020;
        let mut next = frame_c(&bits(hop, serial, 0xB, 0b10, 2), 6);
        next[1] = 250;
        end_1020.extend(&next[1..]);
        vec![("sync_restart".into(), sync_restart), ("long_high".into(), long_high), ("end_1020".into(), end_1020)]
    }

    /// One frame FoxFW's C (host oracle, `subghz_oracle_foxfw decode --protocol Toyota`) reports for a
    /// synthetic burst of [synth_cases]: the pulses (HIGH positive) and the oracle's `Bit`, `Key` and
    /// `Tail` (serial and button are the key's bits 4..32 and 0..4, as the C extracts them).
    struct Case {
        name: &'static str,
        bits: usize,
        key: u64,
        tail: u64,
        wire: &'static [i32],
    }

    const ACCEPT: &[Case] = &[
        Case { name: "b67_4", bits: 67, key: 0x5EED1234_0BEEF424, tail: 3, wire: &[
            -10000, 200, -390, 200, -390, 200, -390, 200, -390, 200, -390, 200, -390, 200, -390, 200,
            -390, 200, -390, 200, -390, 200, -390, 200, -390, 200, -1940, 200, -390, 200, -390, 390,
            -390, 390, -200, 390, -390, 390, -200, 390, -390, 200, -390, 200, -200, 200, -390, 200,
            -200, 390, -200, 200, -200, 390, -390, 200, -390, 200, -200, 200, -200, 200, -200, 390,
            -200, 390, -390, 390, -390, 390, -200, 390, -390, 390, -200, 390, -390, 390, -390, 200,
            -390, 200, -200, 200, -200, 390, -200, 200, -390, 200, -200, 200, -390, 390, -10000,
        ] },
        Case { name: "b67_5", bits: 67, key: 0x5EED1234_0BEEF425, tail: 3, wire: &[
            -10000, 200, -390, 200, -390, 200, -390, 200, -390, 200, -390, 200, -390, 200, -390, 200,
            -390, 200, -390, 200, -390, 200, -390, 200, -390, 200, -1940, 200, -390, 200, -390, 390,
            -390, 390, -200, 390, -390, 390, -200, 390, -390, 200, -390, 200, -200, 200, -390, 200,
            -200, 390, -200, 200, -200, 390, -390, 200, -390, 200, -200, 200, -200, 200, -200, 390,
            -200, 390, -390, 390, -390, 390, -200, 390, -390, 390, -200, 390, -390, 390, -390, 200,
            -390, 200, -200, 200, -200, 390, -200, 200, -390, 200, -390, 200, -390, 390, -10000,
        ] },
        Case { name: "b67_a", bits: 67, key: 0x5EED1234_0BEEF42A, tail: 3, wire: &[
            -10000, 200, -390, 200, -390, 200, -390, 200, -390, 200, -390, 200, -390, 200, -390, 200,
            -390, 200, -390, 200, -390, 200, -390, 200, -390, 200, -1940, 200, -390, 200, -390, 390,
            -390, 390, -200, 390, -390, 390, -200, 390, -390, 200, -390, 200, -200, 200, -390, 200,
            -200, 390, -200, 200, -200, 390, -390, 200, -390, 200, -200, 200, -200, 200, -200, 390,
            -200, 390, -390, 390, -390, 390, -200, 390, -390, 390, -200, 390, -390, 390, -390, 200,
            -390, 200, -200, 200, -200, 390, -200, 390, -200, 390, -200, 200, -390, 390, -10000,
        ] },
        Case { name: "b67_f", bits: 67, key: 0x5EED1234_0BEEF42F, tail: 3, wire: &[
            -10000, 200, -390, 200, -390, 200, -390, 200, -390, 200, -390, 200, -390, 200, -390, 200,
            -390, 200, -390, 200, -390, 200, -390, 200, -390, 200, -1940, 200, -390, 200, -390, 390,
            -390, 390, -200, 390, -390, 390, -200, 390, -390, 200, -390, 200, -200, 200, -390, 200,
            -200, 390, -200, 200, -200, 390, -390, 200, -390, 200, -200, 200, -200, 200, -200, 390,
            -200, 390, -390, 390, -390, 390, -200, 390, -390, 390, -200, 390, -390, 390, -390, 200,
            -390, 200, -200, 200, -200, 390, -200, 390, -390, 390, -390, 200, -390, 390, -10000,
        ] },
        Case { name: "c66_7", bits: 66, key: 0x1234ABCD_0A5C3E17, tail: 2, wire: &[
            -10000, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380,
            -440, 380, -1250, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 400, -850, 800,
            -450, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 400, -850, 800, -450, 400,
            -850, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800,
            -450, 800, -450, 800, -450, 800, -450, 400, -850, 400, -850, 800, -450, 800, -450, 400,
            -850, 800, -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 800,
            -450, 400, -850, 400, -850, 800, -450, 400, -850, 800, -450, 800, -450, 800, -450, 400,
            -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 800, -450, 800, -450, 800,
            -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 800, -450, 800,
            -450, 800, -450, 800, -450, 400, -8000,
        ] },
        Case { name: "c66_b", bits: 66, key: 0x1234ABCD_0A5C3E1B, tail: 2, wire: &[
            -10000, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380,
            -440, 380, -1250, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 400, -850, 800,
            -450, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 400, -850, 800, -450, 400,
            -850, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800,
            -450, 800, -450, 800, -450, 800, -450, 400, -850, 400, -850, 800, -450, 800, -450, 400,
            -850, 800, -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 800,
            -450, 400, -850, 400, -850, 800, -450, 400, -850, 800, -450, 800, -450, 800, -450, 400,
            -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 800, -450, 800, -450, 800,
            -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 400, -850, 800,
            -450, 800, -450, 800, -450, 400, -8000,
        ] },
        Case { name: "c66_d", bits: 66, key: 0x1234ABCD_0A5C3E1D, tail: 2, wire: &[
            -10000, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380,
            -440, 380, -1250, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 400, -850, 800,
            -450, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 400, -850, 800, -450, 400,
            -850, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800,
            -450, 800, -450, 800, -450, 800, -450, 400, -850, 400, -850, 800, -450, 800, -450, 400,
            -850, 800, -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 800,
            -450, 400, -850, 400, -850, 800, -450, 400, -850, 800, -450, 800, -450, 800, -450, 400,
            -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 800, -450, 800, -450, 800,
            -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 800, -450, 400,
            -850, 800, -450, 800, -450, 400, -8000,
        ] },
        Case { name: "c66_e", bits: 66, key: 0x1234ABCD_0A5C3E1E, tail: 2, wire: &[
            -10000, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380,
            -440, 380, -1250, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 400, -850, 800,
            -450, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 400, -850, 800, -450, 400,
            -850, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800,
            -450, 800, -450, 800, -450, 800, -450, 400, -850, 400, -850, 800, -450, 800, -450, 400,
            -850, 800, -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 800,
            -450, 400, -850, 400, -850, 800, -450, 400, -850, 800, -450, 800, -450, 800, -450, 400,
            -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 800, -450, 800, -450, 800,
            -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 800, -450, 800,
            -450, 400, -850, 800, -450, 400, -8000,
        ] },
        Case { name: "c67_7", bits: 67, key: 0x1234ABCD_0A5C3E17, tail: 5, wire: &[
            -10000, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380,
            -440, 380, -1250, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 400, -850, 800,
            -450, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 400, -850, 800, -450, 400,
            -850, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800,
            -450, 800, -450, 800, -450, 800, -450, 400, -850, 400, -850, 800, -450, 800, -450, 400,
            -850, 800, -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 800,
            -450, 400, -850, 400, -850, 800, -450, 400, -850, 800, -450, 800, -450, 800, -450, 400,
            -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 800, -450, 800, -450, 800,
            -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 800, -450, 800,
            -450, 800, -450, 800, -450, 400, -850, 800, -8000,
        ] },
        Case { name: "c67_b", bits: 67, key: 0x1234ABCD_0A5C3E1B, tail: 5, wire: &[
            -10000, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380,
            -440, 380, -1250, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 400, -850, 800,
            -450, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 400, -850, 800, -450, 400,
            -850, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800,
            -450, 800, -450, 800, -450, 800, -450, 400, -850, 400, -850, 800, -450, 800, -450, 400,
            -850, 800, -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 800,
            -450, 400, -850, 400, -850, 800, -450, 400, -850, 800, -450, 800, -450, 800, -450, 400,
            -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 800, -450, 800, -450, 800,
            -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 400, -850, 800,
            -450, 800, -450, 800, -450, 400, -850, 800, -8000,
        ] },
        Case { name: "c67_d", bits: 67, key: 0x1234ABCD_0A5C3E1D, tail: 5, wire: &[
            -10000, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380,
            -440, 380, -1250, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 400, -850, 800,
            -450, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 400, -850, 800, -450, 400,
            -850, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800,
            -450, 800, -450, 800, -450, 800, -450, 400, -850, 400, -850, 800, -450, 800, -450, 400,
            -850, 800, -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 800,
            -450, 400, -850, 400, -850, 800, -450, 400, -850, 800, -450, 800, -450, 800, -450, 400,
            -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 800, -450, 800, -450, 800,
            -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 800, -450, 400,
            -850, 800, -450, 800, -450, 400, -850, 800, -8000,
        ] },
        Case { name: "c67_e", bits: 67, key: 0x1234ABCD_0A5C3E1E, tail: 5, wire: &[
            -10000, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380,
            -440, 380, -1250, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 400, -850, 800,
            -450, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 400, -850, 800, -450, 400,
            -850, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800,
            -450, 800, -450, 800, -450, 800, -450, 400, -850, 400, -850, 800, -450, 800, -450, 400,
            -850, 800, -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 800,
            -450, 400, -850, 400, -850, 800, -450, 400, -850, 800, -450, 800, -450, 800, -450, 400,
            -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 800, -450, 800, -450, 800,
            -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 800, -450, 800,
            -450, 400, -850, 800, -450, 400, -850, 800, -8000,
        ] },
        Case { name: "c68_7", bits: 68, key: 0x1234ABCD_0A5C3E17, tail: 11, wire: &[
            -10000, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380,
            -440, 380, -1250, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 400, -850, 800,
            -450, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 400, -850, 800, -450, 400,
            -850, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800,
            -450, 800, -450, 800, -450, 800, -450, 400, -850, 400, -850, 800, -450, 800, -450, 400,
            -850, 800, -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 800,
            -450, 400, -850, 400, -850, 800, -450, 400, -850, 800, -450, 800, -450, 800, -450, 400,
            -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 800, -450, 800, -450, 800,
            -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 800, -450, 800,
            -450, 800, -450, 800, -450, 400, -850, 800, -450, 800, -8000,
        ] },
        Case { name: "c68_b", bits: 68, key: 0x1234ABCD_0A5C3E1B, tail: 11, wire: &[
            -10000, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380,
            -440, 380, -1250, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 400, -850, 800,
            -450, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 400, -850, 800, -450, 400,
            -850, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800,
            -450, 800, -450, 800, -450, 800, -450, 400, -850, 400, -850, 800, -450, 800, -450, 400,
            -850, 800, -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 800,
            -450, 400, -850, 400, -850, 800, -450, 400, -850, 800, -450, 800, -450, 800, -450, 400,
            -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 800, -450, 800, -450, 800,
            -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 400, -850, 800,
            -450, 800, -450, 800, -450, 400, -850, 800, -450, 800, -8000,
        ] },
        Case { name: "c68_d", bits: 68, key: 0x1234ABCD_0A5C3E1D, tail: 11, wire: &[
            -10000, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380,
            -440, 380, -1250, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 400, -850, 800,
            -450, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 400, -850, 800, -450, 400,
            -850, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800,
            -450, 800, -450, 800, -450, 800, -450, 400, -850, 400, -850, 800, -450, 800, -450, 400,
            -850, 800, -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 800,
            -450, 400, -850, 400, -850, 800, -450, 400, -850, 800, -450, 800, -450, 800, -450, 400,
            -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 800, -450, 800, -450, 800,
            -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 800, -450, 400,
            -850, 800, -450, 800, -450, 400, -850, 800, -450, 800, -8000,
        ] },
        Case { name: "c68_e", bits: 68, key: 0x1234ABCD_0A5C3E1E, tail: 11, wire: &[
            -10000, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380,
            -440, 380, -1250, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 400, -850, 800,
            -450, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 400, -850, 800, -450, 400,
            -850, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800,
            -450, 800, -450, 800, -450, 800, -450, 400, -850, 400, -850, 800, -450, 800, -450, 400,
            -850, 800, -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 800,
            -450, 400, -850, 400, -850, 800, -450, 400, -850, 800, -450, 800, -450, 800, -450, 400,
            -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 800, -450, 800, -450, 800,
            -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 800, -450, 800,
            -450, 400, -850, 800, -450, 400, -850, 800, -450, 800, -8000,
        ] },
        Case { name: "c69", bits: 68, key: 0x1234ABCD_0A5C3E1B, tail: 11, wire: &[
            -10000, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380,
            -440, 380, -1250, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 400, -850, 800,
            -450, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 400, -850, 800, -450, 400,
            -850, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800,
            -450, 800, -450, 800, -450, 800, -450, 400, -850, 400, -850, 800, -450, 800, -450, 400,
            -850, 800, -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 800,
            -450, 400, -850, 400, -850, 800, -450, 400, -850, 800, -450, 800, -450, 800, -450, 400,
            -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 800, -450, 800, -450, 800,
            -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 400, -850, 800,
            -450, 800, -450, 800, -450, 400, -850, 800, -450, 800, -450, 400, -8000,
        ] },
    ];

    const REJECT: &[(&str, &[i32])] = &[
        ("b66", &[
            -10000, 200, -390, 200, -390, 200, -390, 200, -390, 200, -390, 200, -390, 200, -390, 200,
            -390, 200, -390, 200, -390, 200, -390, 200, -390, 200, -1940, 200, -390, 200, -390, 390,
            -390, 390, -200, 390, -390, 390, -200, 390, -390, 200, -390, 200, -200, 200, -390, 200,
            -200, 390, -200, 200, -200, 390, -390, 200, -390, 200, -200, 200, -200, 200, -200, 390,
            -200, 390, -390, 390, -390, 390, -200, 390, -390, 390, -200, 390, -390, 390, -390, 200,
            -390, 200, -200, 200, -200, 390, -200, 390, -200, 390, -200, 200, -390, 10000,
        ]),
        ("b67_btnb", &[
            -10000, 200, -390, 200, -390, 200, -390, 200, -390, 200, -390, 200, -390, 200, -390, 200,
            -390, 200, -390, 200, -390, 200, -390, 200, -390, 200, -1940, 200, -390, 200, -390, 390,
            -390, 390, -200, 390, -390, 390, -200, 390, -390, 200, -390, 200, -200, 200, -390, 200,
            -200, 390, -200, 200, -200, 390, -390, 200, -390, 200, -200, 200, -200, 200, -200, 390,
            -200, 390, -390, 390, -390, 390, -200, 390, -390, 390, -200, 390, -390, 390, -390, 200,
            -390, 200, -200, 200, -200, 390, -200, 390, -200, 390, -390, 200, -390, 390, -10000,
        ]),
        ("c65", &[
            -10000, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380,
            -440, 380, -1250, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 400, -850, 800,
            -450, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 400, -850, 800, -450, 400,
            -850, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800,
            -450, 800, -450, 800, -450, 800, -450, 400, -850, 400, -850, 800, -450, 800, -450, 400,
            -850, 800, -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 800,
            -450, 400, -850, 400, -850, 800, -450, 400, -850, 800, -450, 800, -450, 800, -450, 400,
            -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 800, -450, 800, -450, 800,
            -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 400, -850, 800,
            -450, 800, -450, 800, -8000,
        ]),
        ("c68_btn1", &[
            -10000, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380,
            -440, 380, -1250, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 400, -850, 800,
            -450, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 400, -850, 800, -450, 400,
            -850, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800,
            -450, 800, -450, 800, -450, 800, -450, 400, -850, 400, -850, 800, -450, 800, -450, 400,
            -850, 800, -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 800,
            -450, 400, -850, 400, -850, 800, -450, 400, -850, 800, -450, 800, -450, 800, -450, 400,
            -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 800, -450, 800, -450, 800,
            -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 400, -850, 400,
            -850, 800, -450, 800, -450, 400, -850, 800, -450, 800, -8000,
        ]),
        ("c68_hopff", &[
            -10000, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380,
            -440, 380, -1250, 800, -450, 800, -450, 800, -450, 800, -450, 800, -450, 800, -450, 800,
            -450, 800, -450, 800, -450, 800, -450, 800, -450, 800, -450, 800, -450, 800, -450, 800,
            -450, 800, -450, 800, -450, 800, -450, 800, -450, 800, -450, 800, -450, 800, -450, 800,
            -450, 800, -450, 800, -450, 800, -450, 800, -450, 800, -450, 800, -450, 800, -450, 800,
            -450, 800, -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 800,
            -450, 400, -850, 400, -850, 800, -450, 400, -850, 800, -450, 800, -450, 800, -450, 400,
            -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 800, -450, 800, -450, 800,
            -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 400, -850, 800,
            -450, 800, -450, 800, -450, 400, -850, 800, -450, 800, -8000,
        ]),
        ("c68_pre5", &[
            -10000, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -1250, 400, -850, 400,
            -850, 400, -850, 800, -450, 400, -850, 400, -850, 800, -450, 400, -850, 400, -850, 400,
            -850, 800, -450, 800, -450, 400, -850, 800, -450, 400, -850, 400, -850, 800, -450, 400,
            -850, 800, -450, 400, -850, 800, -450, 400, -850, 800, -450, 800, -450, 800, -450, 800,
            -450, 400, -850, 400, -850, 800, -450, 800, -450, 400, -850, 800, -450, 400, -850, 400,
            -850, 400, -850, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 400, -850, 800,
            -450, 400, -850, 800, -450, 800, -450, 800, -450, 400, -850, 400, -850, 400, -850, 400,
            -850, 800, -450, 800, -450, 800, -450, 800, -450, 800, -450, 400, -850, 400, -850, 400,
            -850, 400, -850, 800, -450, 800, -450, 400, -850, 800, -450, 800, -450, 800, -450, 400,
            -850, 800, -450, 800, -8000,
        ]),
        ("c68_serial0", &[
            -10000, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380,
            -440, 380, -1250, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 400, -850, 800,
            -450, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 400, -850, 800, -450, 400,
            -850, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800,
            -450, 800, -450, 800, -450, 800, -450, 400, -850, 400, -850, 800, -450, 800, -450, 400,
            -850, 800, -450, 400, -850, 400, -850, 400, -850, 400, -850, 400, -850, 400, -850, 400,
            -850, 400, -850, 400, -850, 400, -850, 400, -850, 400, -850, 400, -850, 400, -850, 400,
            -850, 400, -850, 400, -850, 400, -850, 400, -850, 400, -850, 400, -850, 400, -850, 400,
            -850, 400, -850, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 800,
            -450, 800, -450, 800, -450, 400, -850, 800, -450, 800, -8000,
        ]),
        ("c68_sync1600", &[
            -10000, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380, -440, 380,
            -440, 380, -1600, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 400, -850, 800,
            -450, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 400, -850, 800, -450, 400,
            -850, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800, -450, 400, -850, 800,
            -450, 800, -450, 800, -450, 800, -450, 400, -850, 400, -850, 800, -450, 800, -450, 400,
            -850, 800, -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 400, -850, 800,
            -450, 400, -850, 400, -850, 800, -450, 400, -850, 800, -450, 800, -450, 800, -450, 400,
            -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 800, -450, 800, -450, 800,
            -450, 400, -850, 400, -850, 400, -850, 400, -850, 800, -450, 800, -450, 400, -850, 800,
            -450, 800, -450, 800, -450, 400, -850, 800, -450, 800, -8000,
        ]),
    ];

    #[test]
    fn decodes_what_the_c_decoder_decodes() {
        for c in ACCEPT {
            let got = decode_all(&mut ToyotaFoxfwDecoder::new(), &wire(c.wire));
            assert_eq!(got.len(), 1, "{}: {got:?}", c.name);
            let d = &got[0];
            assert_eq!((d.data, d.data_count_bit, d.extra), (c.key, c.bits, Some(c.tail)), "{}", c.name);
            assert_eq!((d.serial, d.button), (Some((c.key >> 4) as u32 & 0x0FFF_FFFF), Some(c.key as u8 & 0x0F)), "{}", c.name);
            let variant = if c.name.starts_with('b') { VARIANT_B } else { VARIANT_C };
            assert_eq!(d.counter, Some(u16::from(variant)), "{}", c.name);
            assert_eq!(d.protocol_display_name.as_deref(), Some(NAME));
            assert!(!d.crc_valid && !d.encoder_capable, "{}", c.name);
        }
    }

    #[test]
    fn rejects_what_the_c_decoder_rejects() {
        for (name, w) in REJECT {
            let got = decode_all(&mut ToyotaFoxfwDecoder::new(), &wire(w));
            assert!(got.is_empty(), "{name}: {got:?}");
        }
    }

    /// The vectors above are the builders' output (which the other tests and the oracle dump use).
    #[test]
    fn vectors_are_the_builders_output() {
        let accept = synth_cases();
        assert_eq!(accept.len(), ACCEPT.len());
        for c in ACCEPT {
            assert_eq!(accept.iter().find(|(n, _)| n == c.name).map(|(_, w)| w.as_slice()), Some(c.wire), "{}", c.name);
        }
        let reject = reject_cases();
        assert_eq!(reject.len(), REJECT.len());
        for (name, w) in REJECT {
            assert_eq!(reject.iter().find(|(n, _)| n == name).map(|(_, w)| w.as_slice()), Some(*w), "{name}");
        }
    }

    /// The oracle's frames for [sequence_cases]: a frame reported on a sync LOW is followed by the
    /// next frame's data at once; one ended by a 1200 µs HIGH or a 1020 µs LOW by a preamble step
    /// (where a 250 µs first HIGH counts, which the reset step would take for variant B). The whole
    /// registry gives the same: this decoder's post-callback state survives the registry's reset.
    #[test]
    fn frame_ends_and_restarts_as_in_the_c() {
        type Frame = (usize, u64, u64);
        let want: &[(&str, &[Frame])] = &[
            ("sync_restart", &[(68, 0x1234ABCD_0A5C3E1B, 11), (68, 0x1234ABCD_0A5C3E1E, 6)]),
            ("long_high", &[(67, 0x1234ABCD_0A5C3E1D, 5), (67, 0x1234ABCD_0A5C3E17, 5)]),
            ("end_1020", &[(66, 0x1234ABCD_0A5C3E17, 2), (66, 0x1234ABCD_0A5C3E1B, 2)]),
        ];
        let cases = sequence_cases();
        assert_eq!(cases.len(), want.len());
        for &(name, frames) in want {
            let w = wire(&cases.iter().find(|(n, _)| n == name).unwrap().1);
            let got: Vec<Frame> = decode_all(&mut ToyotaFoxfwDecoder::new(), &w)
                .iter()
                .map(|d| (d.data_count_bit, d.data, d.extra.unwrap()))
                .collect();
            assert_eq!(got, frames, "{name}");
            let reg: Vec<(String, u64)> = ProtocolRegistry::new()
                .process_signal_stream(&w, 433_920_000)
                .into_iter()
                .map(|(n, d, _)| (n, d.data))
                .collect();
            let reg_want: Vec<(String, u64)> = frames.iter().map(|f| (NAME.to_string(), f.1)).collect();
            assert_eq!(reg, reg_want, "{name}");
        }
    }

    /// The whole registry reports every accepted frame as Toyota (FoxFW), at both bands, and none of
    /// the rejected ones.
    #[test]
    fn registry_reports_them() {
        for freq in [315_000_000, 433_920_000] {
            for c in ACCEPT {
                let got: Vec<(String, u64)> = ProtocolRegistry::new()
                    .process_signal_stream(&wire(c.wire), freq)
                    .into_iter()
                    .map(|(n, d, _)| (n, d.data))
                    .collect();
                assert_eq!(got, vec![(NAME.to_string(), c.key)], "{} at {freq}", c.name);
            }
            for (name, w) in REJECT {
                let got = ProtocolRegistry::new().process_signal_stream(&wire(w), freq);
                assert!(got.iter().all(|(n, _, _)| n != NAME), "{name} at {freq}");
            }
        }
    }

    /// FoxFW's C on the two Prius 2006 captures (`decode --protocol Toyota`): 8 frames each, `Bit: 68`,
    /// `Tail: 15`, keys 3F2E50987231804B (Lock) and 29CD45FF7231804E (Unlock).
    #[test]
    fn prius_captures_match_the_reference() {
        for (file, key) in [
            ("Toyota_Prius2006_lock.sub", 0x3F2E5098_7231804B_u64),
            ("Toyota_Prius2006_unlock.sub", 0x29CD45FF_7231804E),
        ] {
            let path = std::path::Path::new("IMPORTS/Toyota + Lexus").join(file);
            if !path.exists() {
                eprintln!("Skip: {path:?} not found (run from crate root)");
                return;
            }
            let (_, pulses) = sub_pulses(&path);
            let got = decode_all(&mut ToyotaFoxfwDecoder::new(), &pulses);
            assert_eq!(got.len(), 8, "{file}");
            for d in &got {
                assert_eq!((d.data, d.data_count_bit, d.extra), (key, 68, Some(0xF)), "{file}");
                assert_eq!((d.serial, d.button, d.counter), (Some(0x723_1804), Some(key as u8 & 0x0F), Some(2)), "{file}");
            }
        }
    }

    /// Key files: FoxFW's `Protocol: Toyota` / `Bit` / `Key` / `Tail` round-trip, and load through the
    /// registry (KAT's Toyota decoder hands 66..=68-bit keys here) under this decoder's name.
    #[test]
    fn key_files() {
        let dec = ToyotaFoxfwDecoder::new();
        let reg = ProtocolRegistry::new();
        for c in ACCEPT {
            let d = decode_all(&mut ToyotaFoxfwDecoder::new(), &wire(c.wire)).remove(0);
            let text = assert_round_trip(&dec, &d);
            let hex: Vec<String> = c.key.to_be_bytes().iter().map(|b| format!("{b:02X}")).collect();
            let lines = format!("Protocol: Toyota\nBit: {}\nKey: {}\nTail: {}\n", c.bits, hex.join(" "), c.tail);
            assert!(text.ends_with(&lines), "{}: {text}", c.name);
            assert_eq!(reg.decode_key(&key(&text)), Some((NAME.to_string(), d)), "{}", c.name);
        }
        // The oracle's `serialize` output for the Prius Lock frame.
        let prius = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                     Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Toyota\nBit: 68\n\
                     Key: 3F 2E 50 98 72 31 80 4B\nTail: 15\n";
        let (name, d) = reg.decode_key(&key(prius)).unwrap();
        assert_eq!(name, NAME);
        assert_eq!((d.serial, d.button, d.counter, d.extra), (Some(0x723_1804), Some(0xB), Some(2), Some(0xF)));
        // No `Tail` (files saved before FoxFW v2.0.12): 0.
        assert_eq!(reg.decode_key(&key(&prius.replace("Tail: 15\n", ""))).unwrap().1.extra, Some(0));
        // The C's `data_count_bit < TOYOTA_C_BITS`: refused.
        assert!(dec.from_flipper_key(&key(&prius.replace("Bit: 68", "Bit: 65"))).is_none());
        assert!(reg.decode_key(&key(&prius.replace("Bit: 68", "Bit: 65"))).is_none());
        // A 96-bit key is Toyota's own (it needs `Key_2`): not handed over.
        assert!(reg.decode_key(&key(&prius.replace("Bit: 68", "Bit: 96"))).is_none());
        // The variant comes from the button: B's 0xA, then a code in neither list.
        assert_eq!(dec.from_flipper_key(&key(&prius.replace("80 4B", "80 4A"))).unwrap().counter, Some(1));
        assert_eq!(dec.from_flipper_key(&key(&prius.replace("80 4B", "80 41"))).unwrap().counter, Some(0));
        assert!(dec.to_flipper_key(&DecodedSignal::new(0, 64)).is_none());
    }

    /// KAT's own AM receive path (live RX, IQ files): this decoder alone reads an OOK Kia V3 fob's
    /// frames (Kia V3 has the Prius air format, and Kia's lock / unlock codes 0xB / 0xD pass its
    /// list), but they still decode as Kia V3 (`NOT_FINAL_ON_GATED_PASS` sends them to the ungated
    /// pass, where Kia V3/V4 completes first), while the Prius frames decode as Toyota (FoxFW).
    #[test]
    fn am_receive_path_leaves_kia_v3_frames_to_kia_v3_v4() {
        use crate::capture::RfModulation::AM;
        let _keystore = crate::protocols::keeloq_common::KEYSTORE_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        crate::protocols::keys::load_keystore_from_embedded();
        let kia = std::path::Path::new("IMPORTS/KIA/KIA V3 N1 RAW.sub");
        let prius = std::path::Path::new("IMPORTS/Toyota + Lexus/Toyota_Prius2006_lock.sub");
        if !kia.exists() || !prius.exists() {
            eprintln!("Skip: IMPORTS not found (run from crate root)");
            return;
        }
        let mut reg = ProtocolRegistry::new();
        let (freq, pulses) = sub_pulses(kia);
        assert!(!decode_all(&mut ToyotaFoxfwDecoder::new(), &pulses).is_empty());
        let names: Vec<String> = reg.process_signal_stream_path(&pulses, freq, Some(AM)).into_iter().map(|(n, ..)| n).collect();
        assert!(!names.is_empty() && names.iter().all(|n| n == "Kia V3"), "{names:?}");
        assert_eq!(reg.process_signal_path(&pulses, freq, Some(AM)).map(|(n, _)| n).as_deref(), Some("Kia V3"));
        let (freq, pulses) = sub_pulses(prius);
        let names: Vec<String> = reg.process_signal_stream_path(&pulses, freq, Some(AM)).into_iter().map(|(n, ..)| n).collect();
        assert_eq!(names, vec![NAME.to_string(); 8]);
        assert_eq!(reg.process_signal_path(&pulses, freq, Some(AM)).map(|(n, _)| n).as_deref(), Some(NAME));
    }

    #[test]
    fn decode_only_with_the_c_button_names() {
        let dec = ToyotaFoxfwDecoder::new();
        assert!(!dec.supports_encoding());
        assert!(dec.encode(&DecodedSignal::new(0x3F2E5098_7231804B, 68), 0x0B).is_none());
        let names: Vec<_> = [0xB, 0xE, 0xD, 0x7, 0xA, 0x5, 0xF, 0x4, 0x1]
            .iter()
            .map(|&c| crate::protocols::native_button_name(NAME, c))
            .collect();
        let want = ["Lock", "Unlock", "Trunk", "Panic", "Lock", "Unlock", "Unknown", "Unknown", "Unknown"];
        assert_eq!(names, want.map(Some));
    }

    /// Writes the synthetic bursts as RAW `.sub` files into `$KAT_SYNTH_DIR` for the oracle
    /// (`cargo test toyota_foxfw_synth_dump -- --ignored`, then `subghz_oracle_foxfw decode --protocol
    /// Toyota <file>`).
    #[test]
    #[ignore]
    fn toyota_foxfw_synth_dump() {
        let preset = "FuriHalSubGhzPresetOok650Async";
        let all = [("", synth_cases()), ("reject_", reject_cases()), ("seq_", sequence_cases())];
        for (prefix, cases) in all {
            for (name, w) in cases {
                dump(&format!("toyota_foxfw_{prefix}{name}.sub"), 433_920_000, preset, &wire(&w));
            }
        }
    }

    /// Prints this decoder's frames (alone, reset after each, as the oracle's `--protocol` run) for
    /// every `.sub` in `$KAT_SYNTH_DIR`: `<file> | <key-file lines>` or `<file> | (none)`
    /// (`cargo test toyota_foxfw_decode_dir -- --ignored --nocapture`).
    #[test]
    #[ignore]
    fn toyota_foxfw_decode_dir() {
        let Ok(dir) = std::env::var("KAT_SYNTH_DIR") else { return };
        let mut paths: Vec<_> = std::fs::read_dir(dir).unwrap().flatten().map(|e| e.path()).collect();
        paths.retain(|p| p.extension().is_some_and(|e| e == "sub"));
        paths.sort();
        let dec = ToyotaFoxfwDecoder::new();
        for path in paths {
            let (_, pulses) = sub_pulses(&path);
            let name = path.file_name().unwrap().to_string_lossy().to_string();
            let frames = decode_all(&mut ToyotaFoxfwDecoder::new(), &pulses);
            if frames.is_empty() {
                println!("{name} | (none)");
            }
            for d in frames {
                let text = dec.to_flipper_key(&d).unwrap().to_text();
                let lines: Vec<&str> = text.lines().skip(3).collect();
                println!("{name} | {} | cnt={:?}", lines.join("; "), d.counter);
            }
        }
    }
}
