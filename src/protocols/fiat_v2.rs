//! Fiat V2 protocol decoder/encoder (FCA: Fiat / Chrysler / Jeep)
//!
//! Decoder aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/fiat_v2.c` and
//! `fiat_v2.h` (primary; decode-only there). The encoder is doctor-firmware's replay encoder (see
//! Encoder): `lib/subghz/protocols/hitag2.c`, which holds its Fiat V2 code since 2026-11
//! (`fiat_v2.c` before). Flipper-ARF's
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
//! The non-FCA layout is the C's reading of a real frame, the Maserati keys' HiTag2 frame (see the
//! deviation below): its selector and its counter read right on the three buttons recorded, its
//! hop does not. None of Flipper-ARF's known keys reproduces that hop, in either layout.
//!
//! **Packing**: `data` = raw[0..8] (first 64 wire bits), `data_count_bit` = 112, `extra` =
//! raw[8..14] (48 bits, right-aligned; bit 48 set on a Maserati key's frame, see below). `counter`
//! holds the C's 11/14-bit counter, or a deviation's (below).
//!
//! **Deviation: the FCA fobs' own layout.** Five real fobs, all recorded press by press and named
//! after the buttons pressed, fit neither of the C's layouts: a 2019 Chrysler Pacifica
//! (`chrysler/19_chrysler_pacifica*`) and a 2022 Jeep Renegade (`jeep/22_jeep_renegade*`), and a
//! key specialist's 2016–2025 Alfa Romeo Giulia and Stelvio (`ALFA ROMEO/more/2016-2025_*`) and
//! 2019–2025 Jeep Wrangler (`jeep/more/2019-2025_*`), 20 presses of each button. Read with the C's
//! non-FCA layout, as all three references do, their counter is one value for the whole recording
//! (0x09B on the Pacifica, 0x080 on the Renegade) and their Panic and Remote start presses carry
//! Unlock's selector. Their bytes 6..9 are, bit-inverted, a flag word and a 16-bit word:
//! ```text
//! !raw[6..8]   top nibble: one value per fob (0xA Pacifica and Giulia, 0x8 Renegade, 0x4 Wrangler,
//!              0x0 Stelvio) | button flags, one of bits 11..5 | 0b00100
//! !raw[8..10]  counter, 14 bits: up by one with every press, across all buttons (0x37F7 … 0x3805
//!              over the Pacifica's 15 presses, 0x1F5 … 0x1FC on the Renegade; on the three new
//!              fobs 171 of 171 steps, and on from one recording to the next) | 2 bits that change
//!              from press to press with no pattern
//! raw[10..14]  hop
//! ```
//! The flags follow the recordings' names press for press: bit 8 Unlock, 6 Lock, 7 Trunk, 11
//! Remote start, 9 and 10 the two sliding doors, 5 Panic. The C's FCA layout (type 0xD) is this
//! one: its 14-bit counter (`raw[8] << 6 | raw[9] >> 2`) and its hop are these, and its selector
//! (raw[7] bits 7:6: 1 Trunk, 2 Lock, 3 Unlock) is the Trunk and Lock flags seen through two
//! bits; only its gate, the one top nibble 0xD, reads no fob in IMPORTS. (FoxFW's Jeep V0,
//! `jeep_v0.c`, reads the Renegade's frames this way, inverted, with the four codes that fob has;
//! it takes the 16-bit word's high byte for a constant 0x07 and so reads no other fob.) A frame
//! whose flag word has that shape (low five bits `00100`, exactly one button bit), whatever its top
//! nibble, is read so: `counter` = the 14 bits, `button` = the number of the flag (1 Panic, 2
//! Lock, 3 Trunk, 4 Unlock, 5 and 6 Sliding door, 7 Remote start; a selector byte is never below
//! 0x40, so the two kinds of code do not meet). `serial`, `data`, `extra`, the key file
//! (ProtoPirate's lines, from the frame's bytes) and the replay are as before; such a frame is
//! never forward-encoded with the other layouts' fields.
//!
//! **Deviation: the Maserati keys' HiTag2 frame.** A key specialist's 2016–2021 Maserati Ghibli
//! and Levante keys (`MASERATI/more/2016-2021_maserati_{ghibli,levante}_*`, two keys, 122 presses
//! recorded press by press and named after the buttons pressed) send the frame of the HiTag2
//! remotes in Garcia et al., "Lock It and Still Lose It" (USENIX Security 2016, Fig. 11: `0x0001 |
//! UID 32 | btn 4 | lctr 10 | ks 32 | 0 | chk 8`, the check "computed by simply XORing each byte";
//! KAT's `hitag2_rke.rs` reads that frame), with seven bits between its UID and its button (with the
//! button, the FCA fobs' flag word) and two after its check. In the window's bits (bit 0 = raw[0]'s
//! top bit), read inverted, as the FCA fobs' fields above are:
//! ```text
//! bits 48..=58    the flag word's bits 15..5: top nibble 0xC on both keys | button flags, one of
//!                 bits 11..5 (8 Unlock, 7 Trunk, 6 Lock; 5 is the FCA fobs' Panic, no Maserati
//!                 Panic press was recorded); bits 8..5 are the paper's 4-bit btn
//! bits 59..=68    lctr: the counter's low 10 bits, up by one with every press (119 of 119 steps
//!                 within the recordings and from one recording to the next of a session)
//! bit  69         1 (fixed: constant on all 122 presses)
//! bits 70..=100   the rolling part (a HiTag2 hop); see the note below
//! bit  101        1 (the paper's 0, inverted; fixed on all 122 presses)
//! bits 102..=109  chk: the XOR of bits 16..=101 taken as bytes from bit 16 (the last six bits padded
//!                 with two zeros); holds on all 122 presses
//! bits 110..=111  10 (fixed)
//! ```
//! A frame that carries the check (bits 69 and 101, bits 110..=111 as above, the XOR in bits
//! 102..=109, and one button flag) is read so: `button` = the number of its flag, as in the FCA
//! fobs' layout (2 Lock, 3 Trunk, 4 Unlock), `counter` = lctr, `crc_valid` = true (`extra` bit
//! [`HITAG2_FLAG`] marks such a frame for `info.rs`). The check is tried before the FCA fobs'
//! layout: with lctr 0x80..=0x9F a Maserati frame has the FCA fobs' flag-word shape (its low five
//! bits are lctr's top five), while an FCA fob's frame passes the check's 11 bits once in 2048
//! frames (none of the FCA fobs' frames in IMPORTS does). A frame without the check is read as
//! before; nothing is gated on it. `serial`, `data`, the key file and the replay are as before; such
//! a frame is never forward-encoded (the recompute writes the C's fields), and "Recover key
//! (HiTag2)" does not take it (`hitag2_hell.rs`).
//!
//! **The hop is not decoded.** This frame is 7 bits longer than the references' Fiat V2 frame, and
//! the 32 cipher-output (hop) bits do not sit at the references' offset: fixed bits fall at 69 and
//! 101, leaving 31 varying bits (70..=100) between the counter and the check, and the C's hop window
//! (raw[9..=12] = bits 72..=103) runs into the fixed bit and the check. KAT does not read a hop from
//! these frames and does not forward-encode or recover a key from them — `button`, `counter` and the
//! XOR check need none of it. Reading the hop (so the HiTag2 key could be recovered, as for the other
//! Fiat/Renault remotes) would take working out its bit layout in this frame; it is left undone.
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

/// Button selector = top two bits of the raw button byte; below 0x40, the number of the flag a
/// frame in the FCA fobs' layout carries ([FiatV2Decoder::fca_fields]).
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match (code, code >> 6) {
        (1, _) => "Panic",
        (2, _) => "Lock",
        (3, _) => "Trunk",
        (4, _) => "Unlock",
        (5, _) => "Sliding door 1",
        (6, _) => "Sliding door 2",
        (7, _) => "Remote start",
        (_, 2) => "Lock",
        (_, 3) => "Unlock",
        (_, 1) => "Trunk",
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
/// The `extra` bit set on a frame in the Maserati keys' HiTag2 frame, whose check KAT verified
/// (module doc; `info.rs` reads it): just above the frame's bytes 8..14.
pub(super) const HITAG2_FLAG: u32 = 48;
/// KAT gate: cells allowed between the end of the valid window and the end of the burst
/// (doctor-firmware's encoder appends a one-cell stop pulse after a frame ending low).
const MAX_TRAILING_CELLS: usize = 1;

// Encoder (doctor-firmware hitag2.c, `fiat_v2.c` before 2026-11).
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

    /// The fields of a frame in the layout the FCA fobs in IMPORTS send (module doc): (button = the
    /// number of its one flag, 1..=7 for bits 5..=11; the 14-bit counter). The flag word's shape
    /// decides, not its top nibble, which differs from fob to fob. None for any other frame.
    fn fca_fields(raw: &[u8; WIRE_BYTES]) -> Option<(u8, u16)> {
        let flags = !u16::from_be_bytes([raw[6], raw[7]]);
        let buttons = (flags >> 5) & 0x7F;
        if flags & 0x1F != 0x04 || buttons.count_ones() != 1 {
            return None;
        }
        Some((buttons.trailing_zeros() as u8 + 1, !u16::from_be_bytes([raw[8], raw[9]]) >> 2))
    }

    /// The fields of a frame in the Maserati keys' HiTag2 frame (module doc): (button = the number
    /// of its one flag, as in [Self::fca_fields]; the 10-bit lctr). None for a frame without that
    /// frame's check: bit 101 and bits 110..=111, and the XOR in bits 102..=109.
    fn hitag2_fields(raw: &[u8; WIRE_BYTES]) -> Option<(u8, u16)> {
        // Inverted, as the fields are read: bit 101 = 1, bits 110..=111 = 10, and the XOR of bits
        // 16..=101 as bytes (bytes 2..=11 and byte 12's top six bits) = bits 102..=109.
        let t = raw.map(|b| !b);
        let xor = t[2..12].iter().fold(t[12] & 0xFC, |x, &b| x ^ b);
        let check = ((t[12] & 0x03) << 6) | (t[13] >> 2);
        if t[12] & 0x04 == 0 || t[13] & 0x03 != 0b10 || xor != check {
            return None;
        }
        let flags = u16::from_be_bytes([t[6], t[7]]);
        let buttons = (flags >> 5) & 0x7F;
        if buttons.count_ones() != 1 {
            return None;
        }
        Some((buttons.trailing_zeros() as u8 + 1, ((flags & 0x1F) << 5) | u16::from(t[8] >> 3)))
    }

    /// Whether a frame is in the Maserati keys' HiTag2 frame, whose hop KAT does not decode in this
    /// longer frame, so the key search cannot take it (`hitag2_hell.rs`).
    pub(super) fn sends_hitag2_frame(raw: &[u8; WIRE_BYTES]) -> bool {
        Self::hitag2_fields(raw).is_some()
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
        Self::key_of(decoded).is_some()
    }

    /// The HITAG2 key [`Self::find_key`] finds for this decode, in Fiat V1's byte order: the key
    /// the detail panel shows ([`hitag2_keys::shown_key`]).
    pub(super) fn key_of(decoded: &DecodedSignal) -> Option<[u8; 6]> {
        let raw = Self::raw_from_signal(decoded)?;
        if !Self::frame_valid(&raw) {
            return None;
        }
        Self::find_key(&raw).map(|(key, _)| key)
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
        // The C's fields, or a deviation's: the Maserati keys' HiTag2 frame (its check tried first)
        // or the FCA fobs' layout (module doc).
        let hitag2 = Self::hitag2_fields(raw);
        let (button, counter) =
            hitag2.or_else(|| Self::fca_fields(raw)).unwrap_or((raw[7], Self::counter(raw)));
        DecodedSignal {
            serial: Some(Self::uid(raw)),
            button: Some(button),
            counter: Some(counter),
            // Only the Maserati keys' frame carries a check; emission is gated on markers /
            // selector / UID / structure, never on it.
            crc_valid: hitag2.is_some(),
            data: u64::from_be_bytes(data_bytes),
            data_count_bit: WIRE_BITS,
            encoder_capable: true,
            extra: Some(u64::from_be_bytes(extra_bytes) | (u64::from(hitag2.is_some()) << HITAG2_FLAG)),
            protocol_display_name: None,
        }
    }

    /// Rebuild the 14 wire bytes from `data` (raw[0..8]) and `extra` (raw[8..14], and the
    /// [`HITAG2_FLAG`] bit, which is not a frame bit).
    fn raw_from_signal(decoded: &DecodedSignal) -> Option<[u8; WIRE_BYTES]> {
        if decoded.data_count_bit != WIRE_BITS {
            return None;
        }
        let extra = decoded.extra?;
        if extra >> (HITAG2_FLAG + 1) != 0 {
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

    /// The input ended right after a frame, before the pulse that ends its burst: KAT holds the
    /// frame until then (it must end the burst, see the module doc), where the C has already
    /// reported it when its window completed.
    fn flush(&mut self) -> Option<DecodedSignal> {
        let raw = self.pending.take()?;
        self.clear_cells();
        self.step = DecoderStep::Reset;
        self.commit(raw)
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
        // otherwise the captured frame is replayed unchanged. A frame in the FCA fobs' layout or
        // the Maserati keys' HiTag2 frame is always replayed: the recompute's fields are the C's
        // (and it would leave the HiTag2 frame's check as it was).
        let own_layout = Self::fca_fields(&raw).is_some() || Self::hitag2_fields(&raw).is_some();
        if let Some((key, combo)) = Self::find_key(&raw).filter(|_| !own_layout) {
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
        // The FCA fobs' layout: flag word !0x5EFB = 0xA104 (Unlock); !0x2023 = 0xDFDC, whose top
        // 14 bits are the counter.
        assert_eq!((d.button, d.counter), (Some(4), Some(0x37F7)));
        // The C's fields for the same bytes (its non-FCA layout), which the key file keeps.
        assert_eq!(RAW_PACIFICA[7] >> BTN_SHIFT, SEL_UNLOCK);
        assert_eq!(FiatV2Decoder::counter(&RAW_PACIFICA), 0x09B); // ~((0x3B << 5) | (0x20 >> 3)) & 0x7FF
        assert_eq!(d.data, 0x0001_D737_A2C3_5EFB);
        assert_eq!(d.extra, Some(0x2023_5C16_7DE9));
        assert_eq!(d.data_count_bit, 112);
        assert_eq!(FiatV2Decoder::hop(&RAW_PACIFICA), 0x235C_167D);
    }

    /// A recording that ends right after a frame (no closing gap): the frame is reported when the
    /// input ends ([crate::protocols::ProtocolDecoder::flush]), alone and in the registry.
    #[test]
    fn a_frame_the_input_ends_after_is_reported() {
        let mut pulses = vec![LevelDuration::new(false, 20_000)];
        for p in cells_to_pulses(&frame_cells(&RAW_PACIFICA)) {
            FiatV2Decoder::enc_add_level(&mut pulses, p.level, p.duration_us);
        }
        let mut decoder = FiatV2Decoder::new();
        assert!(pulses.iter().all(|p| decoder.feed(p.level, p.duration_us).is_none()));
        assert_eq!(decoder.flush().map(|d| d.serial), Some(Some(0xD737_A2C3)));
        assert!(decoder.flush().is_none());
        let mut reg = crate::protocols::ProtocolRegistry::new();
        let names: Vec<String> = reg.process_signal_stream(&pulses, 433_920_000).into_iter().map(|(n, _, _)| n).collect();
        assert_eq!(names, ["Fiat V2"]);
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
        for (type_byte, key_idx, combo) in [(0x3E, 0usize, 0u8), (0x3E, 3, 1), (0x1A, 5, 2), (0x1A, 7, 3)] {
            // The C's non-FCA layout, with a flag word of a shape the FCA fobs do not send (their
            // frames are replayed, below): selector 3, low five bits 00011.
            let mut raw = RAW_PACIFICA;
            raw[6] = type_byte;
            raw[7] = 0xE3;
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
        // A frame in the FCA fobs' layout is replayed as it is even when a key fits its hop under
        // the C's layout: the selector and the 11-bit counter the recompute would write are not
        // where that fob keeps its button and counter.
        let mut raw = RAW_PACIFICA;
        let hop = FiatV2Decoder::compute_hop(&raw, &ARF_KNOWN_KEYS[0], 0);
        FiatV2Decoder::patch_hop(&mut raw, hop);
        assert!(FiatV2Decoder::find_known_key(&raw).is_some() && FiatV2Decoder::fca_fields(&raw).is_some());
        let original = FiatV2Decoder::build_signal(&raw);
        let out = decode(&FiatV2Decoder::new().encode(&original, 0x01).unwrap());
        assert_eq!(out, vec![original]);
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
        raw[6] = 0x3E;
        raw[7] = 0xE3; // not the FCA fobs' flag word: those frames are replayed
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
            assert_eq!(fiat[0].1.button, Some(4), "{rel}: recording starts with Unlock");
        }
    }

    const PANIC: u8 = 1;
    const LOCK: u8 = 2;
    const TRUNK: u8 = 3;
    const UNLOCK: u8 = 4;
    const DOOR_1: u8 = 5;
    const DOOR_2: u8 = 6;
    const START: u8 = 7;

    /// The two real fobs' presses, as their recordings are named: the button flags follow the
    /// names, the 14-bit counter goes up by one with every press (module doc). Read with the C's
    /// layout the counter was one value per recording and Panic / Remote start were "Unlock".
    #[test]
    fn chrysler_and_jeep_presses_follow_their_recordings_names() {
        let cases: [(&str, &[(u8, u16)]); 2] = [
            (
                // ul2 l2 t2 rs2 sd2 sd2 p2 t2: one Panic press was sent, the counter goes straight
                // on to the Trunk press after it
                "chrysler/19_chrysler_pacifica_ul2_l2_t2_rs2_sd2_sd2_p2_t2.sub",
                &[
                    (UNLOCK, 0x37F7), (UNLOCK, 0x37F8), (LOCK, 0x37F9), (LOCK, 0x37FA), (TRUNK, 0x37FB),
                    (TRUNK, 0x37FC), (START, 0x37FD), (START, 0x37FE), (DOOR_1, 0x37FF), (DOOR_1, 0x3800),
                    (DOOR_2, 0x3801), (DOOR_2, 0x3802), (PANIC, 0x3803), (TRUNK, 0x3804), (TRUNK, 0x3805),
                ],
            ),
            (
                // ul2 l2 rs2 p2
                "jeep/22_jeep_renegade_ul2_l2_rs2_p2.sub",
                &[
                    (UNLOCK, 0x1F5), (UNLOCK, 0x1F6), (LOCK, 0x1F7), (LOCK, 0x1F8), (START, 0x1F9),
                    (START, 0x1FA), (PANIC, 0x1FB), (PANIC, 0x1FC),
                ],
            ),
        ];
        for (rel, want) in cases {
            let Some(results) = registry_decodes(rel) else { continue };
            let mut presses: Vec<(u8, u16)> = results.iter().map(|(_, d)| (d.button.unwrap(), d.counter.unwrap())).collect();
            presses.dedup();
            assert_eq!(presses, want, "{rel}");
            for (name, d) in &results {
                assert_eq!(crate::protocols::native_button_name(name, d.button.unwrap()).is_some(), true);
            }
        }
        let name = |code| native_buttons("Fiat V2", code);
        assert_eq!([name(1), name(4), name(7)], [Some("Panic"), Some("Unlock"), Some("Remote start")]);
        assert_eq!([name(0x40), name(0x80), name(0xFB)], [Some("Trunk"), Some("Lock"), Some("Unlock")]);
        // The flag word's top nibble differs from fob to fob (raw 0xB on the Wrangler, 0xF on the
        // Stelvio): any one is read in the fobs' layout.
        let mut other = RAW_PACIFICA;
        other[6] = 0x3E;
        assert_eq!(FiatV2Decoder::fca_fields(&other), Some((UNLOCK, 0x37F7)));
        // A flag word of another shape: the C's fields.
        let mut odd = RAW_PACIFICA;
        odd[7] = 0xE3; // low five bits 00011
        assert_eq!(FiatV2Decoder::fca_fields(&odd), None);
        assert_eq!(FiatV2Decoder::build_signal(&odd).button, Some(0xE3));
        let mut two_flags = RAW_PACIFICA;
        two_flags[7] = 0xBB & !0x80; // Lock's and Trunk's flags
        assert_eq!(FiatV2Decoder::fca_fields(&two_flags), None);
    }

    /// Every press of one recording (Fiat V2 alone, which is quick in a debug build): (serial,
    /// button, counter, whether the frame carries the Maserati keys' check), copies of a press
    /// listed once.
    fn fiat_v2_presses(rel: &str) -> Option<Vec<(u32, u8, u16, bool)>> {
        let path = Path::new("IMPORTS").join(rel);
        if !path.exists() {
            eprintln!("Skip: {:?} not found (run from crate root)", path);
            return None;
        }
        let (_, raw_pairs) = import_sub_raw(&path).unwrap();
        let mut decoder = FiatV2Decoder::new();
        let mut decoded: Vec<DecodedSignal> =
            raw_pairs.iter().filter_map(|p| decoder.feed(p.level, p.duration_us)).collect();
        decoded.extend(decoder.flush());
        let mut presses: Vec<(u32, u8, u16, bool)> = decoded
            .iter()
            .map(|d| (d.serial.unwrap(), d.button.unwrap(), d.counter.unwrap(), d.crc_valid))
            .collect();
        presses.dedup();
        Some(presses)
    }

    /// A key specialist's Alfa Romeo Giulia, Alfa Romeo Stelvio and Jeep Wrangler fobs (top
    /// nibbles 0xA, 0x0 and 0x4; 20 presses of each button, one recording per button): every press
    /// reads as the button its recording is named after, and the counter goes up by one with every
    /// press, on from one recording to the next as they were made (the Wrangler's Lock recording
    /// was made after its Unlock one, with presses between them that were not recorded).
    #[test]
    fn alfa_romeo_and_jeep_wrangler_presses_follow_their_recordings_names() {
        let fobs: [&[(&str, u8)]; 3] = [
            &[
                ("ALFA ROMEO/more/2016-2025_alfa_romeo_giulia_lock.sub", LOCK),
                ("ALFA ROMEO/more/2016-2025_alfa_romeo_giulia_unlock.sub", UNLOCK),
                ("ALFA ROMEO/more/2016-2025_alfa_romeo_giulia_trunk.sub", TRUNK),
            ],
            &[
                ("ALFA ROMEO/more/2016-2025_alfa_romeo_stelvio_lock.sub", LOCK),
                ("ALFA ROMEO/more/2016-2025_alfa_romeo_stelvio_unlock.sub", UNLOCK),
                ("ALFA ROMEO/more/2016-2025_alfa_romeo_stelvio_trunk.sub", TRUNK),
            ],
            &[
                ("jeep/more/2019-2025_jeep_wrangler_unlock.sub", UNLOCK),
                ("jeep/more/2019-2025_jeep_wrangler_lock.sub", LOCK),
            ],
        ];
        let mut serials = Vec::new();
        for (i, fob) in fobs.iter().enumerate() {
            let mut last: Option<u16> = None;
            for &(rel, button) in fob.iter() {
                let Some(presses) = fiat_v2_presses(rel) else { return };
                assert_eq!(presses.len(), 20, "{rel}");
                assert!(presses.iter().all(|&(_, b, ..)| b == button), "{rel}: {presses:?}");
                // No frame of these fobs passes the Maserati keys' check (module doc).
                assert!(presses.iter().all(|&(.., checked)| !checked), "{rel}");
                let counters: Vec<u16> = presses.iter().map(|&(_, _, c, _)| c).collect();
                assert!(counters.windows(2).all(|w| w[1] == w[0] + 1), "{rel}: {counters:X?}");
                match last {
                    // The Wrangler (the third fob): presses between its two recordings.
                    Some(prev) if i == 2 => assert!(counters[0] > prev, "{rel}"),
                    Some(prev) => assert_eq!(counters[0], prev + 1, "{rel}"),
                    None => {}
                }
                last = counters.last().copied();
                assert!(presses.iter().all(|&(s, ..)| s == presses[0].0), "{rel}: one fob");
                if serials.last() != Some(&presses[0].0) {
                    serials.push(presses[0].0);
                }
            }
        }
        assert_eq!(serials.len(), 3, "three fobs, one serial each");
    }

    /// The key specialist's Maserati Ghibli and Levante keys send the HiTag2 frame (module doc):
    /// every press carries its check and reads as its recording is named, by its button flag, and
    /// lctr goes up by one with every press, on from one recording to the next of a session. The
    /// Ghibli was recorded in two sessions, with 12 presses between them that were not recorded.
    #[test]
    fn maserati_keys_send_the_hitag2_frame() {
        let sessions: [&[(&str, u8, usize)]; 3] = [
            &[
                ("MASERATI/more/2016-2021_maserati_ghibli_lock.sub", LOCK, 20),
                ("MASERATI/more/2016-2021_maserati_ghibli_unlock.sub", UNLOCK, 20),
                ("MASERATI/more/2016-2021_maserati_ghibli_trunk.sub", TRUNK, 20),
            ],
            &[
                ("MASERATI/more/2016-2021_maserati_ghibli_lock_2.sub", LOCK, 10),
                ("MASERATI/more/2016-2021_maserati_ghibli_unlock_2.sub", UNLOCK, 10),
                ("MASERATI/more/2016-2021_maserati_ghibli_trunk_2.sub", TRUNK, 10),
            ],
            &[
                ("MASERATI/more/2016-2021_maserati_levante_lock.sub", LOCK, 10),
                ("MASERATI/more/2016-2021_maserati_levante_unlock.sub", UNLOCK, 12),
                ("MASERATI/more/2016-2021_maserati_levante_trunk.sub", TRUNK, 10),
            ],
        ];
        // (serial, lctr of its last press) of each key seen so far
        let mut keys: Vec<(u32, u16)> = Vec::new();
        for session in sessions {
            let mut last: Option<(u32, u16)> = None;
            for &(rel, button, count) in session {
                let Some(presses) = fiat_v2_presses(rel) else { return };
                assert_eq!(presses.len(), count, "{rel}");
                let serial = presses[0].0;
                assert!(presses.iter().all(|&(s, b, _, checked)| s == serial && b == button && checked), "{rel}: {presses:X?}");
                let counters: Vec<u16> = presses.iter().map(|&(_, _, c, _)| c).collect();
                assert!(counters.windows(2).all(|w| w[1] == w[0] + 1), "{rel}: {counters:X?}");
                match last {
                    Some((s, prev)) => assert_eq!((serial, counters[0]), (s, prev + 1), "{rel}"),
                    None => {
                        if let Some(&(_, end)) = keys.iter().find(|&&(s, _)| s == serial) {
                            assert_eq!(counters[0], end + 13, "{rel}: 12 presses between the sessions");
                        }
                    }
                }
                last = Some((serial, *counters.last().unwrap()));
            }
            let (serial, end) = last.unwrap();
            keys.retain(|&(s, _)| s != serial);
            keys.push((serial, end));
        }
        assert_eq!(keys.len(), 2, "the Ghibli and the Levante");
    }

    /// Set `width` bits of `frame` from bit `at` (bit 0 = the top bit of byte 0) to `value`.
    fn put_bits(frame: &mut [u8; WIRE_BYTES], at: usize, width: usize, value: u64) {
        for i in 0..width {
            let (pos, mask) = (at + i, 0x80u8 >> ((at + i) & 7));
            if (value >> (width - 1 - i)) & 1 != 0 {
                frame[pos >> 3] |= mask;
            } else {
                frame[pos >> 3] &= !mask;
            }
        }
    }

    /// A made-up frame in the Maserati keys' HiTag2 frame (module doc; UID and keystream made up),
    /// as KAT reads it (the fields inverted).
    fn hitag2_frame(flag: u8, lctr: u16) -> [u8; WIRE_BYTES] {
        let mut t = [0u8; WIRE_BYTES];
        put_bits(&mut t, 0, 16, 0xFFFE);
        put_bits(&mut t, 16, 32, 0x1234_5678);
        put_bits(&mut t, 48, 4, 0xC);
        put_bits(&mut t, 52, 7, 1 << (flag - 1));
        put_bits(&mut t, 59, 10, lctr.into());
        put_bits(&mut t, 69, 32, 0x5A5A_C3C3);
        put_bits(&mut t, 101, 1, 1);
        let xor = t[2..12].iter().fold(t[12] & 0xFC, |x, &b| x ^ b);
        put_bits(&mut t, 102, 8, xor.into());
        put_bits(&mut t, 110, 2, 0b10);
        t.map(|b| !b)
    }

    /// A HiTag2 frame with lctr 0x080..=0x09F has the FCA fobs' flag-word shape (00100): its check
    /// decides. Each part of the check is needed; a frame without it is read in the FCA fobs'
    /// layout, as before. The C's reading of the same frame, Panic's, the replay, and the key
    /// search's refusal.
    #[test]
    fn the_hitag2_frames_check_comes_before_the_fca_layout() {
        let raw = hitag2_frame(LOCK, 0x090);
        assert!(FiatV2Decoder::frame_valid(&raw));
        assert_eq!(FiatV2Decoder::hitag2_fields(&raw), Some((LOCK, 0x090)));
        assert!(FiatV2Decoder::fca_fields(&raw).is_some(), "the FCA fobs' shape");
        let d = FiatV2Decoder::build_signal(&raw);
        assert_eq!((d.button, d.counter, d.crc_valid), (Some(LOCK), Some(0x090), true));
        assert_eq!(d.extra.map(|e| e >> HITAG2_FLAG), Some(1));
        assert_eq!(FiatV2Decoder::raw_from_signal(&d), Some(raw));
        // The C's reading: Lock's selector, the 11-bit counter.
        assert_eq!((raw[7] >> BTN_SHIFT, FiatV2Decoder::counter(&raw)), (SEL_LOCK, 0x090));
        // From the air, and replayed unchanged.
        let decoded = decode(&gapped(&frame_cells(&raw)));
        assert_eq!(decoded, vec![d.clone()]);
        assert_eq!(decode(&FiatV2Decoder::new().encode(&d, 0x02).unwrap()), vec![d.clone()]);
        // bit 101, bits 110 / 111, two bits of the XOR
        for (byte, bit) in [(12usize, 0x04u8), (13, 0x02), (13, 0x01), (12, 0x01), (13, 0x80)] {
            let mut bad = raw;
            bad[byte] ^= bit;
            assert_eq!(FiatV2Decoder::hitag2_fields(&bad), None, "byte {byte} bit {bit:#04x}");
            let other = FiatV2Decoder::build_signal(&bad);
            assert!(!other.crc_valid && other.extra.unwrap() >> HITAG2_FLAG == 0);
            assert_eq!(other.button, Some(LOCK), "the FCA fobs' layout reads the same flag");
        }
        // Panic, flag bit 5, which no recorded Maserati press shows: the C would read Unlock's
        // selector and lctr + 0x400.
        let panic = hitag2_frame(PANIC, 0x1C5);
        assert_eq!(FiatV2Decoder::hitag2_fields(&panic), Some((PANIC, 0x1C5)));
        assert_eq!((panic[7] >> BTN_SHIFT, FiatV2Decoder::counter(&panic)), (SEL_UNLOCK, 0x5C5));
        // "Recover key (HiTag2)" would read the C's hop: it does not take the frame.
        assert!(crate::protocols::hitag2_hell::frame_of("Fiat V2", &d).is_err());
        let mut not_hitag2 = raw;
        not_hitag2[13] ^= 0x80;
        assert!(crate::protocols::hitag2_hell::frame_of("Fiat V2", &FiatV2Decoder::build_signal(&not_hitag2)).is_ok());
    }

    #[test]
    fn does_not_steal_the_ldv_frames() {
        // The LDV T80's frames, which Ford V3's EU path reads too (they list as LDV V0).
        for rel in ["LDV/LDV-T80_lock.sub", "LDV/LDV-T80_unlock.sub"] {
            let Some(results) = registry_decodes(rel) else {
                continue;
            };
            let names: Vec<&str> = results.iter().map(|(n, _)| n.as_str()).collect();
            assert!(!names.contains(&"Fiat V2"), "{rel}: {names:?}");
            assert!(names.contains(&"LDV V0"), "{rel}: {names:?}");
        }
    }

    /// Research harness (ignored; needs IMPORTS, a release build and minutes on the GPU/CPU HiTag2
    /// Hell search). Recovers the 48-bit key of each Maserati key from its recordings, to confirm
    /// the HiTag2 frame carries the attack's inputs and to pin the keystream polarity and the 4-bit
    /// IV button. The attack needs the frame's keystream (the 32 bits at 69..=100), the counter
    /// (`lctr`, below 1024) and the IV button. The recovered key goes to a file (`KAT_MASERATI_OUT`,
    /// else a temp file); only a redacted line is printed. Run:
    ///   KAT_MASERATI_OUT=/path cargo test --release --bin kat -- --ignored --nocapture maserati_hitag2_key
    #[test]
    #[ignore]
    fn maserati_hitag2_key_recovers_from_the_recordings() {
        use crate::protocols::hitag2_hell as hell;
        use hell::{BruteForceControl, BruteForceOutcome, Hitag2Request, SecondFrame};
        use std::fmt::Write as _;

        // (raw, button code 1..=7, lctr) for each distinct press of a recording.
        fn presses(rel: &str) -> Vec<([u8; WIRE_BYTES], u8, u16)> {
            let (_, pairs) = import_sub_raw(&Path::new("IMPORTS").join(rel)).unwrap();
            let mut dec = FiatV2Decoder::new();
            let mut sigs: Vec<DecodedSignal> =
                pairs.iter().filter_map(|p| dec.feed(p.level, p.duration_us)).collect();
            sigs.extend(dec.flush());
            let mut out: Vec<([u8; WIRE_BYTES], u8, u16)> = Vec::new();
            for d in &sigs {
                let raw = FiatV2Decoder::raw_from_signal(d).unwrap();
                let (btn, lctr) = FiatV2Decoder::hitag2_fields(&raw).expect("a Maserati frame");
                if out.last().map(|p| (p.1, p.2)) != Some((btn, lctr)) {
                    out.push((raw, btn, lctr));
                }
            }
            out
        }
        // `width` frame bits from bit `at` (bit 0 = MSB of byte 0), read in one polarity (the
        // wire bytes, or their inverse). The HiTag2 frame's fields: keystream 69..=100, lctr
        // 59..=68, the paper's 4-bit button 55..=58 (one-hot; Unlock 8 inverted, Trunk 4, Lock 2).
        fn field(raw: &[u8; WIRE_BYTES], invert: bool, at: usize, width: usize) -> u32 {
            let t = if invert { raw.map(|b| !b) } else { *raw };
            (at..at + width).fold(0u32, |v, i| (v << 1) | u32::from((t[i >> 3] >> (7 - (i & 7))) & 1))
        }
        let ks = |raw: &[u8; WIRE_BYTES], inv| field(raw, inv, 69, 32);
        let cnt = |raw: &[u8; WIRE_BYTES], inv| field(raw, inv, 59, 10);
        let btn4 = |raw: &[u8; WIRE_BYTES], inv| field(raw, inv, 55, 4) as u8;

        let keys: [(&str, &[&str]); 2] = [
            (
                "ghibli",
                &[
                    "MASERATI/more/2016-2021_maserati_ghibli_lock.sub",
                    "MASERATI/more/2016-2021_maserati_ghibli_unlock.sub",
                    "MASERATI/more/2016-2021_maserati_ghibli_trunk.sub",
                    "MASERATI/more/2016-2021_maserati_ghibli_lock_2.sub",
                    "MASERATI/more/2016-2021_maserati_ghibli_unlock_2.sub",
                    "MASERATI/more/2016-2021_maserati_ghibli_trunk_2.sub",
                ],
            ),
            (
                "levante",
                &[
                    "MASERATI/more/2016-2021_maserati_levante_lock.sub",
                    "MASERATI/more/2016-2021_maserati_levante_unlock.sub",
                    "MASERATI/more/2016-2021_maserati_levante_trunk.sub",
                ],
            ),
        ];
        if !Path::new("IMPORTS").join(keys[0].1[0]).exists() {
            eprintln!("Skip: IMPORTS not found (run from crate root)");
            return;
        }

        // bits 69 and 101 are fixed framing bits, so the keystream is the 31 bits 70..=100: the
        // cipher's ks0..ks30, MSB first, with ks31 (the LSB) not transmitted. The attack needs all
        // 32 bits, so brute the missing LSB of each search frame (4 combinations); verify across
        // every press on the 31 known bits only (the recovered key's own ks31 per press).
        let h31 = |raw: &[u8; WIRE_BYTES], inv| field(raw, inv, 70, 31);
        let mut report = String::new();
        // The wire polarity found no key (a full 328 s scan); the cipher uses the inverted frame.
        let invert = true;
        // Ghibli Unlock, a counter pair spanning both recording sessions (well separated).
        let (name, rels) = keys[0];
        let all: Vec<_> = rels.iter().flat_map(|r| presses(r)).collect();
        let uid = FiatV2Decoder::uid(&all[0].0);
        let unlock: Vec<_> = all.iter().filter(|p| p.1 == UNLOCK).collect();
        let a = *unlock.iter().min_by_key(|p| cnt(&p.0, invert)).unwrap();
        let b = *unlock.iter().max_by_key(|p| cnt(&p.0, invert)).unwrap();
        eprintln!("{name}: pair counters {:03X},{:03X}; h31 a={:08X} b={:08X}", cnt(&a.0, invert), cnt(&b.0, invert), h31(&a.0, invert), h31(&b.0, invert));
        let _ = (ks, btn4, h31);
        // SMART SWEEP. The cipher injects key32 ^ iv, so for a same-button pair the button term is a
        // constant XOR the search folds into the key: a key fits the two Unlock frames exactly when
        // the COUNTER's position in the nonce is right, whatever the button. `recover_key_any`
        // verifies a list of requests sharing the first frame's hop in ONE kernel scan, so every
        // counter shift 0..=22 is tested together (iv = lctr << s, button 0; `full_nonce` makes
        // equiv_key exact). The 31 transmitted keystream bits (70..=100) are the cipher's ks with one
        // bit dropped and some bit order; loop the readings (straight / reversed, drop LSB / MSB),
        // and for frame A brute the dropped bit (it fixes the hop → its own scan); frame B's dropped
        // bit rides along as extra requests. Stop at the first reading+shift that recovers a key.
        let iv_parts = |iv: u32| (iv >> 4, (iv & 0xF) as u8);
        let rev31 = |x: u32| x.reverse_bits() >> 1;
        // Variant toggles (set per run, no recompile): KAT_CMPL complements the keystream bits
        // (wire- vs inverted-space), KAT_CREV feeds the counter with its 10 bits reversed.
        let cmpl = std::env::var("KAT_CMPL").is_ok();
        let crev = std::env::var("KAT_CREV").is_ok();
        eprintln!("{name}: variant cmpl={cmpl} crev={crev}");
        let bits = |raw: &[u8; WIRE_BYTES]| {
            let x = field(raw, invert, 70, 31);
            if cmpl { !x & 0x7FFF_FFFF } else { x }
        };
        let rev10 = |x: u32| x.reverse_bits() >> 22;
        let ctr = |raw: &[u8; WIRE_BYTES]| {
            let c = cnt(raw, invert);
            if crev { rev10(c) } else { c }
        };
        // v = the 31 known keystream bits in ks order (ks0..ks30 for drop_lsb, else ks1..ks31).
        let v = |raw: &[u8; WIRE_BYTES], reversed: bool| if reversed { rev31(bits(raw)) } else { bits(raw) };
        // (name, reversed, drop_lsb); (straight, drop_lsb) already failed, so it is last.
        let readings = [("rev,dropLSB", true, true), ("straight,dropMSB", false, false), ("rev,dropMSB", true, false), ("straight,dropLSB", false, true)];
        let hop_of = |vv: u32, g: u32, drop_lsb: bool| if drop_lsb { (vv << 1) | g } else { (g << 31) | vv };
        let known = |auth: u32, drop_lsb: bool| if drop_lsb { auth >> 1 } else { auth & 0x7FFF_FFFF };
        let mut hit: Option<(&str, u32, bool, hell::RecoveredKey)> = None; // (reading, shift, drop_lsb, key)
        'scan: for (rname, reversed, drop_lsb) in readings {
            for a_g in [0u32, 1] {
                let hop_a = hop_of(v(&a.0, reversed), a_g, drop_lsb);
                let mut reqs = Vec::new();
                let mut meta = Vec::new(); // shift per request
                for s in 0..=22u32 {
                    let (iva, ivb) = (u64::from(ctr(&a.0)) << s, u64::from(ctr(&b.0)) << s);
                    if iva >> 32 != 0 || ivb >> 32 != 0 {
                        continue;
                    }
                    let (ca, ba) = iv_parts(iva as u32);
                    let (cb, bb) = iv_parts(ivb as u32);
                    for b_g in [0u32, 1] {
                        reqs.push(Hitag2Request {
                            hop: hop_a,
                            uid,
                            cnt: ca,
                            btn: ba,
                            second: Some(SecondFrame { hop: hop_of(v(&b.0, reversed), b_g, drop_lsb), cnt: cb, btn: bb }),
                            full_nonce: true,
                        });
                        meta.push(s);
                    }
                }
                eprintln!("{name}: reading [{rname}] a_g={a_g}: one scan over {} configs …", reqs.len());
                let t0 = std::time::Instant::now();
                match hell::recover_key_any(&reqs, &BruteForceControl::default()) {
                    BruteForceOutcome::Found(rk) => {
                        let s = meta[rk.request];
                        eprintln!("{name}: [{rname}] KEY at shift s={s}, a_g={a_g} in {:.0}s", t0.elapsed().as_secs_f64());
                        writeln!(report, "{name}: reading [{rname}] shift s={s} a_g={a_g}").unwrap();
                        hit = Some((rname, s, drop_lsb, rk));
                        break 'scan;
                    }
                    other => eprintln!("{name}: [{rname}] a_g={a_g}: no key ({other:?}, {:.0}s)", t0.elapsed().as_secs_f64()),
                }
            }
        }
        if let Some((rname, s, drop_lsb, rk)) = hit {
            let reversed = rname.starts_with("rev");
            let unlock: Vec<_> = all.iter().filter(|p| p.1 == UNLOCK).collect();
            let unlock_ok = unlock.iter().filter(|p| {
                let (c, bt) = iv_parts(ctr(&p.0) << s);
                known(hell::authenticator(rk.key, rk.uid, c, bt), drop_lsb) == v(&p.0, reversed)
            }).count();
            writeln!(report, "  verifies Unlock (iv=lctr<<{s}): {unlock_ok}/{}", unlock.len()).unwrap();
            eprintln!("{name}: [{rname}] shift {s} verifies {unlock_ok}/{} Unlock", unlock.len());
            let mut codes: Vec<u8> = all.iter().map(|p| p.1).collect();
            codes.sort_unstable();
            codes.dedup();
            for code in &codes {
                let g: Vec<_> = all.iter().filter(|p| p.1 == *code).collect();
                let d = (0u32..1 << 16).find(|&d| {
                    g.iter().all(|p| {
                        let (c, bt) = iv_parts((ctr(&p.0) << s) ^ d);
                        known(hell::authenticator(rk.key, rk.uid, c, bt), drop_lsb) == v(&p.0, reversed)
                    })
                });
                writeln!(report, "  {} (code {code}): button XOR offset {:?} ({} presses)", native_buttons("Fiat V2", *code).unwrap_or("?"), d.map(|d| format!("{d:#X}")), g.len()).unwrap();
            }
            writeln!(report, "  key[standard] = {:02X?}", hell::key_bytes(rk.key)).unwrap();
            writeln!(report, "  key[fiat]     = {:02X?}", hell::fiat_bcm_key(rk.key)).unwrap();
            eprintln!("{name}: layout found (reading [{rname}], shift {s}); button offsets + key written; key withheld");
        } else {
            eprintln!("{name}: no (reading x counter shift) fits; the cipher/IV differs more than the sweep covers");
        }
        let out = std::env::var("KAT_MASERATI_OUT").unwrap_or_else(|_| {
            std::env::temp_dir().join("maserati_hitag2.txt").to_string_lossy().into_owned()
        });
        std::fs::write(&out, &report).unwrap();
        eprintln!("key material written to {out}");
    }
}
