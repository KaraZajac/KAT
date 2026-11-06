//! Mazda V2 protocol decoder/encoder (Flipper-ARF "Mazda V2").
//!
//! Ported from Flipper-ARF: `lib/subghz/protocols/mazda_v2.c` / `mazda_v2.h`
//! (`SUBGHZ_PROTOCOL_MAZDA_V2_NAME = "Mazda V2"`). doctor-firmware ships the same file: timing,
//! sample parser, sync, field layout, both 256×6 lookup tables and the encoder are identical; the
//! copies differ only in the Flipper custom-button UI mapping (ARF: UP→Lock, DOWN→Unlock,
//! LEFT→Trunk; doctor: UP→Lock, OK→Unlock, DOWN→Trunk via a different custom-button API). KAT
//! follows ARF, whose explicit Lock/Unlock/Trunk mapping lines up with KAT's generic button codes.
//! ProtoPirate has no Mazda V2. This is a different protocol from KAT's "Mazda V0" / "Mazda
//! Siemens" (250/500µs pair decoders at 433.92 MHz), which this decoder does not affect.
//!
//! **Timing** (AM and FM, 315 + 433.92 MHz): TE = 500µs, 2TE = 1000µs. The decoder classifies raw
//! durations with the C's thresholds: ≤249µs is ignored outright (glitch filter, no state change),
//! ≤749µs is short, ≤1499µs is long, anything longer is an end-of-burst gap.
//!
//! **Line code / sampler** (`subghz_protocol_decoder_mazda_v2_feed`): each bit lasts 2TE, every bit
//! boundary has a transition and the level of the SECOND half equals the bit. The decoder therefore
//! stores one level sample per bit: a long pulse stores its level; of two shorts the first is
//! skipped (`skip_flag`) and the second stored. The first ≤749µs pulse after idle only arms the
//! parser (it is the lead-in half bit). Samples go into a 512-entry buffer, which is scanned
//! (`mazda_v2_try_parse`) every ~10 pulses once >110 samples are buffered, when the buffer nears
//! full (≥508) and at every gap (>70 samples).
//!
//! **Frame**: preamble of ≥100 identical samples (the encoder sends 180 short pairs), the 8-bit
//! sync 0xEC (samples `1,1,1,0,1,1,0,0`), then 63 payload bits; 71 bits after the preamble in all.
//! A capture with inverted levels shows a preamble of 1s and the inverted sync 0x13, and its
//! payload is inverted back. The 63 bits are stored left-aligned in `data` (`value << 1`, so
//! `data[0]` is always 0 and is never transmitted), `data_count_bit` = 63 (as the C):
//! ```text
//! [63:56] button byte   (btn & 0x7F: 0x04 Lock, 0x08 Unlock, 0x10 Trunk,
//!                        0x06/0x0A/0x0E = "alt" Lock/Unlock/Trunk; bit 7 is carried through)
//! [55:48] hdr
//! [47:40] hop0  [39:32] hop1  [31:27] hop2 (5b)
//! [26:19] idx   (8-bit rolling index = counter)
//! [18:16] hop3 (3b)  [15:8] hop4  [7:1] hop5 (7b)  [0] 0
//! ```
//! `DecodedSignal`: `button` = the full button byte, `counter` = idx, `serial` = None — the frame
//! has no plaintext serial (the C's `generic.serial = idx` is a placeholder it never displays), so
//! KAT does not present the counter as a serial. `extra` is unused (all 63 bits live in `data`).
//!
//! **Gating**: ≥100 identical preamble samples immediately before the sync, the exact 8-bit sync
//! (0xEC, or 0x13 for inverted levels), a valid button byte (one of the six codes above), and
//! value ≠ 0 / 0xFFFFFFFF00000002. There is no CRC; `crc_valid` reports the structural checks.
//!
//! **Encoder** (`subghz_protocol_encoder_mazda_v2_deserialize` + `mazda_v2_build_upload`): a real
//! rolling-code generator. It bumps idx by 1 (the C's default rolling-counter multiplier), re-stamps
//! the button for a Lock/Unlock/Trunk command (keeping the alt family and bit 7 of the captured
//! button; other commands keep the captured button), and derives the next hop from the two 256×6
//! lookup tables (default + lock) — `mazda_v2_layout_is_inline`/`inline_apply_idx` for "inline"
//! frames, else `advance_hop` (table row XOR the captured delta, re-mixed by `mix_hop` when the
//! idx bits change). Upload: preamble, sync 0xEC + 63 payload bits through the C's two-state level
//! encoder, a one-bit tail and a 3000µs HIGH gap, repeated 3 times (the C's default `Repeat`).
//! The app keeps the frame sent on the capture (`app/tx.rs`), so the next press sends the code
//! after.
//!
//! **Deviations from C**:
//! - Sync check: the C's two-flag pattern loop accepts the normal sync only as an exact 0xEC, but
//!   once the first sample matches the inverted pattern it accepts ANY remaining 7 bits (an
//!   inverted frame is then gated only by its first sync sample). KAT requires the full inverted
//!   pattern 0x13. Genuine inverted frames carry the full pattern, so this only rejects noise.
//! - Encoder preamble: the C emits 180 (L,H) pairs and then starts the sync's first `1` bit with a
//!   full 1000µs HIGH, so on air the last preamble HIGH merges into a 1500µs pulse — one µs over the
//!   decoder's own 1499µs gap threshold, i.e. the C encoder's output does not decode through the C
//!   decoder at exact timing. The trailing preamble HIGH is a redundant half bit (the decoder treats
//!   the leading LOW as the lead-in and samples the LOWs), so KAT drops it: the preamble is
//!   `L, (H,L)×179` and the sync starts with a clean 1000µs HIGH.
//! - Encoder lead-in: the preamble's leading LOW is what arms the sampler, and on air it merges
//!   into what precedes the copy (the carrier is off). Behind silence (KAT's own receiver) the
//!   sampler arms on the first HIGH instead, samples the wrong halves and reads the sync as 0xEC
//!   behind a preamble of ones, which the parser rejects; behind noise (a Flipper's receiver
//!   before a transmission) the noise's parity decides. The C encoder sends it so, hence its
//!   first copy decodes by luck and the later copies, behind the 3000µs HIGH gap that idles the
//!   sampler, always (KAT 1.8.6 sent the same). KAT now starts every copy with a 3000µs HIGH,
//!   which the sampler takes for a gap in every state: the LOW after it is a pulse of its own.
//!   (A 1000µs lead-in, tried first, is a sample when noise has armed the sampler and fails on
//!   one parity.) The Flipper's receiver drops the first ~40 ms of every transmission while its
//!   AGC settles, lead-in included, so on it the first copy stays a matter of luck either way;
//!   KAT's receiver reads every copy.
//! - The C's 149-tick same-key debounce is omitted: KAT resets every decoder after each emission
//!   (which in the C also clears `last_key`), so it could never fire.
//!
//! **Key files** (`subghz_protocol_decoder_mazda_v2_serialize` / `_deserialize`): `Protocol: Mazda
//! V2`, `Bit: 63`, `Key` = the left-aligned value, then `Hdr` (decimal) and `Disp` ("Mazda V2 -
//! Lock|Unlock|Trunk|Unknown"). The key holds the whole frame, so loading is the decoder's own field
//! split (`mazda_v2_decode_key`) of a 63-bit key; the C loads any value, and `crc_valid` reports the
//! decoder's button/value gate on it. The `Btn`/`Cnt`/`Hdr` lines that ARF's encoder reads back as
//! overrides (and doctor-firmware's generic writer adds) are not applied: like the C's decoder
//! deserialize, the key is the frame.

use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Mazda V2",
    decoder: "Mazda V2",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::DifferentialManchester,
    rf: Rf::Both,
    encryption: "Rolling Code",
    make: "Mazda",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&AM650, &FM476],
};

/// Bit 7 is carried through; 0x06/0x0A/0x0E are the "alt" Lock/Unlock/Trunk family.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code & 0x7F {
        0x04 | 0x06 => "Lock",
        0x08 | 0x0A => "Unlock",
        0x10 | 0x0E => "Trunk",
        _ => "Other",
    })
}

const TE_SHORT: u32 = 500;
const TE_LONG: u32 = 1000;
const TE_GAP: u32 = 3000;
const TE_DELTA: u32 = 250;
/// Pulses ≤ this are ignored entirely (`MAZDA_V2_DUR_FILTER` 0xF9).
const DUR_FILTER: u32 = 0xF9;
/// Upper bound of a short pulse (`MAZDA_V2_DUR_SHORT` 0x2ED).
const DUR_SHORT: u32 = 0x2ED;
/// Upper bound of a long pulse (`MAZDA_V2_DUR_LONG` 0x5DB); longer is a gap.
const DUR_LONG: u32 = 0x5DB;
const SAMPLE_CAP: usize = 512;
/// Buffer level that forces a parse (and a reset if nothing is found) (`0x1FC`).
const SAMPLE_PARSE_MIN: usize = 0x1FC;
/// Sync + payload samples (`0x47`).
const FRAME_SAMPLES: usize = 0x47;
/// try_parse needs more than this many samples (`0x46`).
const PARSE_MIN_SAMPLES: usize = 0x46;
/// Periodic parse: more than this many samples (`0x6E`) ...
const PERIODIC_PARSE_SAMPLES: usize = 0x6E;
/// ... and more than this many pulses since the last periodic parse.
const PERIODIC_PARSE_PULSES: u16 = 9;
/// The preamble run before the sync must be longer than this (`0x63`).
const PREAMBLE_MIN_SAMPLES: usize = 0x63;
const PAYLOAD_SAMPLES: usize = 63;
const MIN_COUNT_BIT: usize = 63;
const SYNC: u8 = 0xEC;
const PAYLOAD_BITS: usize = 71;
const PREAMBLE_PAIRS: usize = 180;
const REPEAT_DEFAULT: usize = 3;
const INVALID_VALUE: u64 = 0xFFFF_FFFF_0000_0002;

// Protocol button codes (low 7 bits of the button byte).
const BTN_LOCK: u8 = 0x04;
const BTN_UNLOCK: u8 = 0x08;
const BTN_TRUNK: u8 = 0x10;
const BTN_LOCK_ALT: u8 = 0x06;
const BTN_UNLOCK_ALT: u8 = 0x0A;
const BTN_TRUNK_ALT: u8 = 0x0E;

// KAT generic button codes (encode() argument).
const KAT_BTN_LOCK: u8 = 0x01;
const KAT_BTN_UNLOCK: u8 = 0x02;
const KAT_BTN_TRUNK: u8 = 0x04;

const PAT_A: [u8; 8] = [1, 1, 1, 0, 1, 1, 0, 0];
const PAT_B: [u8; 8] = [0, 0, 0, 1, 0, 0, 1, 1];
const XOR_LO: [u8; 6] = [0x10, 0x00, 0x02, 0x00, 0x18, 0x6A];
const XOR_HI: [u8; 6] = [0x00, 0x00, 0x02, 0x03, 0x33, 0x08];

// `mazda_v2_table_default` / `mazda_v2_table_lock`, generated verbatim from the C source (see the
// `tables_match_c_source_checksums` test).
#[rustfmt::skip]
const TABLE_DEFAULT: [[u8; 6]; 256] = [
    [0xB9, 0x3B, 0x01, 0x03, 0x41, 0x81], [0xEC, 0xAE, 0x0C, 0x02, 0xF5, 0xF4],
    [0x5A, 0x21, 0x74, 0x02, 0x59, 0x6B], [0xE4, 0xC4, 0xEE, 0x02, 0x9F, 0xA3],
    [0x43, 0xFB, 0xCD, 0x00, 0x8C, 0x51], [0x2F, 0xF7, 0xBF, 0x00, 0x90, 0xDB],
    [0x4C, 0x95, 0xEE, 0x00, 0xAC, 0x2E], [0x21, 0xC3, 0xBA, 0x00, 0xA0, 0x22],
    [0xD1, 0xB4, 0x0B, 0x03, 0x4D, 0x1B], [0x27, 0x13, 0xB4, 0x03, 0xD9, 0x84],
    [0x40, 0x8B, 0x20, 0x02, 0xB1, 0xA6], [0x64, 0x57, 0x4D, 0x03, 0x69, 0x93],
    [0x31, 0x99, 0x29, 0x00, 0x21, 0xE1], [0x57, 0x94, 0x45, 0x03, 0x10, 0xFA],
    [0xFA, 0x48, 0x86, 0x00, 0x6B, 0x46], [0x65, 0xFB, 0x71, 0x03, 0x03, 0xB2],
    [0xB8, 0xE0, 0x11, 0x03, 0x3E, 0x69], [0x87, 0x7C, 0x34, 0x00, 0xC5, 0x4B],
    [0xF3, 0x9F, 0x52, 0x02, 0x0A, 0x64], [0x81, 0x45, 0x35, 0x00, 0x05, 0x05],
    [0xEF, 0x90, 0x9F, 0x00, 0x25, 0xAF], [0xD8, 0xDE, 0x01, 0x02, 0x3C, 0x51],
    [0x07, 0xF0, 0xEF, 0x00, 0xE6, 0x7A], [0xCF, 0xEF, 0xD1, 0x02, 0x78, 0x52],
    [0x6B, 0x43, 0xA2, 0x03, 0x62, 0x79], [0xE5, 0xD8, 0xD1, 0x01, 0x52, 0x17],
    [0x50, 0x65, 0x8C, 0x02, 0x4B, 0xB0], [0x5C, 0x6D, 0xBA, 0x01, 0x4A, 0x65],
    [0x17, 0x5B, 0x62, 0x00, 0xD5, 0xA4], [0x83, 0x04, 0xAB, 0x03, 0xC2, 0x8F],
    [0xA8, 0x70, 0x3C, 0x00, 0x0D, 0xAB], [0xD4, 0x5D, 0xB3, 0x03, 0x36, 0xC9],
    [0xB9, 0x3B, 0x01, 0x02, 0x62, 0x89], [0xED, 0xAE, 0x0C, 0x03, 0xA0, 0xA6],
    [0x5A, 0x21, 0x74, 0x03, 0x7A, 0x63], [0xE5, 0xC4, 0xEE, 0x03, 0xCA, 0xF1],
    [0x43, 0xFB, 0xCD, 0x01, 0xAF, 0x59], [0x2E, 0xF7, 0xBF, 0x01, 0xC5, 0x89],
    [0x4C, 0x95, 0xEE, 0x01, 0x8F, 0x26], [0x20, 0xC3, 0xBA, 0x01, 0xF5, 0x70],
    [0xD1, 0xB4, 0x0B, 0x02, 0x6E, 0x13], [0x26, 0x13, 0xB4, 0x02, 0x8C, 0xD6],
    [0x40, 0x8B, 0x20, 0x03, 0x92, 0xAE], [0x65, 0x57, 0x4D, 0x02, 0x3C, 0xC1],
    [0x31, 0x99, 0x29, 0x01, 0x02, 0xE9], [0x56, 0x94, 0x45, 0x02, 0x45, 0xA8],
    [0xFA, 0x48, 0x86, 0x01, 0x48, 0x4E], [0x64, 0xFB, 0x71, 0x02, 0x56, 0xE0],
    [0xB8, 0xE0, 0x11, 0x02, 0x1D, 0x61], [0x86, 0x7C, 0x34, 0x01, 0x90, 0x19],
    [0xF3, 0x9F, 0x52, 0x03, 0x29, 0x6C], [0x80, 0x45, 0x35, 0x01, 0x50, 0x57],
    [0xEF, 0x90, 0x9F, 0x01, 0x06, 0xA7], [0xD9, 0xDE, 0x01, 0x03, 0x69, 0x03],
    [0x07, 0xF0, 0xEF, 0x01, 0xC5, 0x72], [0xCE, 0xEF, 0xD1, 0x03, 0x2D, 0x00],
    [0x6B, 0x43, 0xA2, 0x02, 0x41, 0x71], [0xE4, 0xD8, 0xD1, 0x00, 0x07, 0x45],
    [0x50, 0x65, 0x8C, 0x03, 0x68, 0xB8], [0x5D, 0x6D, 0xBA, 0x00, 0x1F, 0x37],
    [0x17, 0x5B, 0x62, 0x01, 0xF6, 0xAC], [0x8C, 0x04, 0xAB, 0x00, 0x15, 0xD1],
    [0xA8, 0x70, 0x3C, 0x01, 0x2E, 0xA3], [0xDB, 0x5D, 0xB3, 0x00, 0xE1, 0x97],
    [0xB9, 0x39, 0x01, 0x01, 0x69, 0xA1], [0xEE, 0xAC, 0x0D, 0x01, 0xA9, 0xA4],
    [0x5A, 0x23, 0x76, 0x03, 0x42, 0x43], [0xE6, 0xC6, 0xED, 0x02, 0xF0, 0xFB],
    [0x43, 0xF9, 0xC9, 0x00, 0x82, 0x23], [0x2D, 0xF5, 0xBA, 0x01, 0xEA, 0xD9],
    [0x4C, 0x97, 0xE8, 0x03, 0x91, 0x54], [0x23, 0xC1, 0xBD, 0x02, 0xE9, 0x28],
    [0xD1, 0xB6, 0x03, 0x01, 0x68, 0x5D], [0x25, 0x11, 0xBD, 0x00, 0x88, 0xB2],
    [0x40, 0x89, 0x2A, 0x03, 0xA7, 0xE8], [0x66, 0x55, 0x46, 0x03, 0x0B, 0xAD],
    [0x31, 0x9B, 0x25, 0x00, 0x22, 0xF5], [0x55, 0x96, 0x48, 0x02, 0x67, 0x9E],
    [0xFA, 0x4A, 0x88, 0x03, 0x5B, 0x5A], [0x67, 0xF9, 0x7E, 0x01, 0x47, 0xDE],
    [0xB8, 0xE2, 0x01, 0x01, 0x0D, 0x05], [0x85, 0x7E, 0x25, 0x03, 0x82, 0x57],
    [0xF3, 0x9D, 0x40, 0x03, 0x0A, 0x00], [0x83, 0x47, 0x26, 0x00, 0x71, 0x11],
    [0xEF, 0x92, 0x8B, 0x00, 0x30, 0x91], [0xDA, 0xDC, 0x14, 0x03, 0x5D, 0x1F],
    [0x07, 0xF2, 0xF9, 0x03, 0xC0, 0x4C], [0xCD, 0xED, 0xC6, 0x00, 0x2A, 0x14],
    [0x6B, 0x41, 0xBA, 0x01, 0x5C, 0x73], [0xE7, 0xDA, 0xC8, 0x02, 0x18, 0x6D],
    [0x50, 0x67, 0x96, 0x03, 0x46, 0xB2], [0x5E, 0x6F, 0xA1, 0x01, 0x33, 0x17],
    [0x17, 0x59, 0x7E, 0x00, 0xCD, 0xFC], [0x8D, 0x06, 0xB6, 0x00, 0xC1, 0x9F],
    [0xA8, 0x72, 0x22, 0x03, 0x26, 0xFB], [0xDA, 0x5F, 0xAC, 0x03, 0x06, 0xD1],
    [0xB9, 0x38, 0x01, 0x00, 0x7D, 0xB1], [0xEF, 0xAD, 0x0D, 0x00, 0xCB, 0xEE],
    [0x5A, 0x22, 0x76, 0x02, 0x56, 0x53], [0xE7, 0xC7, 0xED, 0x03, 0x92, 0xB1],
    [0x43, 0xF8, 0xC9, 0x01, 0x96, 0x33], [0x2C, 0xF4, 0xBA, 0x00, 0x88, 0x93],
    [0x4C, 0x96, 0xE8, 0x02, 0x85, 0x44], [0x22, 0xC0, 0xBD, 0x03, 0x8B, 0x62],
    [0xD1, 0xB7, 0x03, 0x00, 0x7C, 0x4D], [0x24, 0x10, 0xBD, 0x01, 0xEA, 0xF8],
    [0x40, 0x88, 0x2A, 0x02, 0xB3, 0xF8], [0x67, 0x54, 0x46, 0x02, 0x69, 0xE7],
    [0x31, 0x9A, 0x25, 0x01, 0x36, 0xE5], [0x54, 0x97, 0x48, 0x03, 0x05, 0xD4],
    [0xFA, 0x4B, 0x88, 0x02, 0x4F, 0x4A], [0x66, 0xF8, 0x7E, 0x00, 0x25, 0x94],
    [0xB8, 0xE3, 0x01, 0x00, 0x19, 0x15], [0x84, 0x7F, 0x25, 0x02, 0xE0, 0x1D],
    [0xF3, 0x9C, 0x40, 0x02, 0x1E, 0x10], [0x82, 0x46, 0x26, 0x01, 0x13, 0x5B],
    [0xEF, 0x93, 0x8B, 0x01, 0x24, 0x81], [0xDB, 0xDD, 0x14, 0x02, 0x3F, 0x55],
    [0x07, 0xF3, 0xF9, 0x02, 0xD4, 0x5C], [0xCC, 0xEC, 0xC6, 0x01, 0x48, 0x5E],
    [0x6B, 0x40, 0xBA, 0x00, 0x48, 0x63], [0xE6, 0xDB, 0xC8, 0x03, 0x7A, 0x27],
    [0x50, 0x66, 0x96, 0x02, 0x52, 0xA2], [0x5F, 0x6E, 0xA1, 0x00, 0x51, 0x5D],
    [0x17, 0x58, 0x7E, 0x01, 0xD9, 0xEC], [0x8E, 0x07, 0xB6, 0x01, 0x4E, 0xE1],
    [0xA8, 0x73, 0x22, 0x02, 0x32, 0xEB], [0xD9, 0x5E, 0xAC, 0x02, 0x89, 0xAF],
    [0xBD, 0x3B, 0x01, 0x02, 0x57, 0x8B], [0xE8, 0xAE, 0x0C, 0x03, 0xE3, 0xFE],
    [0x5E, 0x21, 0x74, 0x03, 0x4F, 0x61], [0xE8, 0xC4, 0xE5, 0x02, 0x4F, 0xC3],
    [0x4C, 0xFB, 0xCE, 0x00, 0x1E, 0x29], [0x21, 0xF7, 0xBC, 0x00, 0x74, 0xF9],
    [0x5E, 0x95, 0xE0, 0x02, 0x6A, 0x36], [0x31, 0xC3, 0xB5, 0x03, 0x12, 0x4A],
    [0xDF, 0xB4, 0x0B, 0x01, 0xD7, 0x65], [0x2B, 0x13, 0xB5, 0x00, 0x37, 0x8A],
    [0x4E, 0x8B, 0x22, 0x03, 0x18, 0xD0], [0x68, 0x57, 0x4E, 0x03, 0xB4, 0x95],
    [0x23, 0x99, 0x2D, 0x01, 0xD9, 0x97], [0x47, 0x94, 0x40, 0x03, 0x9C, 0xFC],
    [0xE8, 0x48, 0x80, 0x02, 0xA0, 0x38], [0x75, 0xFB, 0x76, 0x00, 0xBC, 0xBC],
    [0xB6, 0xE0, 0x09, 0x01, 0xB2, 0x3D], [0x85, 0x7C, 0x37, 0x02, 0x4E, 0x51],
    [0xFD, 0x9F, 0x48, 0x03, 0xB5, 0x38], [0x8D, 0x45, 0x2E, 0x00, 0xCE, 0x29],
    [0xFD, 0x90, 0x83, 0x01, 0xCB, 0xF3], [0xC8, 0xDE, 0x1C, 0x02, 0xA6, 0x7D],
    [0x06, 0xF2, 0xE5, 0x01, 0x4A, 0x0C], [0xCF, 0xED, 0xDB, 0x03, 0xA2, 0x7E],
    [0x6A, 0x41, 0xA8, 0x02, 0xCE, 0x0F], [0xE5, 0xDA, 0xDB, 0x00, 0x88, 0x3B],
    [0x43, 0x65, 0x8D, 0x00, 0x7A, 0x8A], [0x41, 0x6C, 0xB9, 0x02, 0x29, 0x53],
    [0x17, 0x5A, 0x61, 0x02, 0x84, 0x92], [0x83, 0x04, 0xAA, 0x02, 0x43, 0xB9],
    [0xA8, 0x71, 0x3F, 0x02, 0x5C, 0x9D], [0xDB, 0x5C, 0xB0, 0x03, 0x93, 0xA9],
    [0xA5, 0x38, 0x02, 0x03, 0x5F, 0xC5], [0xF2, 0xAD, 0x0E, 0x03, 0x9F, 0xC0],
    [0x56, 0x21, 0x75, 0x00, 0x94, 0x6D], [0xFA, 0xC7, 0xEE, 0x00, 0xC6, 0x9F],
    [0x43, 0xFB, 0xCC, 0x00, 0x2E, 0x6F], [0x2E, 0xF7, 0xBE, 0x00, 0x44, 0xBF],
    [0x4C, 0x96, 0xEB, 0x00, 0xE3, 0x6A], [0x23, 0xC0, 0xBE, 0x01, 0x9B, 0x16],
    [0xCD, 0xB7, 0x00, 0x03, 0x5E, 0x39], [0x39, 0x10, 0xBE, 0x02, 0xBE, 0xD6],
    [0x5C, 0x88, 0x29, 0x01, 0x91, 0x8C], [0x7A, 0x54, 0x45, 0x01, 0x3D, 0xC9],
    [0x31, 0x9A, 0x26, 0x03, 0x50, 0xCB], [0x55, 0x97, 0x4B, 0x01, 0x15, 0xA0],
    [0xFA, 0x4B, 0x8B, 0x00, 0x29, 0x64], [0x67, 0xF8, 0x7D, 0x02, 0x35, 0xE0],
    [0xA4, 0xE3, 0x02, 0x03, 0x3B, 0x61], [0x8A, 0x7C, 0x35, 0x02, 0x7E, 0x17],
    [0xEF, 0x9C, 0x43, 0x01, 0x3C, 0x64], [0x9F, 0x46, 0x25, 0x02, 0x47, 0x75],
    [0xEF, 0x93, 0x88, 0x03, 0x42, 0xAF], [0xDA, 0xDD, 0x17, 0x00, 0x2F, 0x21],
    [0x07, 0xF3, 0xFA, 0x00, 0xB2, 0x72], [0xCD, 0xEC, 0xC5, 0x03, 0x58, 0x2A],
    [0x77, 0x40, 0xB9, 0x03, 0x6A, 0x17], [0xFB, 0xDB, 0xCB, 0x00, 0x2E, 0x09],
    [0x4C, 0x65, 0x8F, 0x00, 0x4A, 0xCC], [0x41, 0x6D, 0xB9, 0x03, 0x3D, 0x43],
    [0x17, 0x5B, 0x61, 0x03, 0x90, 0x82], [0x8C, 0x04, 0xA8, 0x02, 0x73, 0xFF],
    [0xA8, 0x70, 0x3F, 0x03, 0x48, 0x8D], [0xDB, 0x5D, 0xB0, 0x02, 0x87, 0xB9],
    [0xA5, 0x39, 0x02, 0x02, 0x4B, 0xD5], [0xF2, 0xAC, 0x0E, 0x02, 0x8B, 0xD0],
    [0x46, 0x23, 0x75, 0x00, 0x60, 0x37], [0xFA, 0xC6, 0xEE, 0x01, 0xD2, 0x8F],
    [0x43, 0xF9, 0xCA, 0x02, 0xE4, 0x0D], [0x2D, 0xF5, 0xB9, 0x03, 0x8C, 0xF7],
    [0x4C, 0x97, 0xEB, 0x01, 0xF7, 0x7A], [0x23, 0xC1, 0xBE, 0x00, 0x8F, 0x06],
    [0xCD, 0xB6, 0x00, 0x02, 0x4A, 0x29], [0x39, 0x11, 0xBE, 0x03, 0xAA, 0xC6],
    [0x5C, 0x89, 0x29, 0x00, 0x85, 0x9C], [0x7A, 0x55, 0x45, 0x00, 0x29, 0xD9],
    [0x31, 0x9B, 0x26, 0x02, 0x44, 0xDB], [0x55, 0x96, 0x4B, 0x00, 0x01, 0xB0],
    [0xFA, 0x4A, 0x8B, 0x01, 0x3D, 0x74], [0x67, 0xF9, 0x7D, 0x03, 0x21, 0xF0],
    [0xA4, 0xE2, 0x02, 0x02, 0x2F, 0x71], [0x99, 0x7E, 0x26, 0x00, 0xA0, 0x23],
    [0xEF, 0x9D, 0x43, 0x00, 0x28, 0x74], [0x9F, 0x47, 0x25, 0x03, 0x53, 0x65],
    [0xEF, 0x92, 0x88, 0x02, 0x56, 0xBF], [0xDA, 0xDC, 0x17, 0x01, 0x3B, 0x31],
    [0x07, 0xF2, 0xFA, 0x01, 0xA6, 0x62], [0xCD, 0xED, 0xC5, 0x02, 0x4C, 0x3A],
    [0x77, 0x41, 0xB9, 0x02, 0x7E, 0x07], [0xFB, 0xDA, 0xCB, 0x01, 0x3A, 0x19],
    [0x4C, 0x67, 0x95, 0x00, 0x64, 0xC6], [0x42, 0x6F, 0xA2, 0x02, 0x11, 0x63],
    [0x17, 0x59, 0x7D, 0x02, 0xAB, 0xD2], [0x8D, 0x06, 0xB5, 0x02, 0xA7, 0xB1],
    [0xA8, 0x72, 0x21, 0x01, 0x40, 0xD5], [0xDA, 0x5F, 0xAF, 0x01, 0x60, 0xFF],
    [0xA5, 0x38, 0x02, 0x03, 0x5F, 0xC5], [0xF3, 0xAD, 0x0E, 0x03, 0xE9, 0x9A],
    [0x46, 0x22, 0x75, 0x01, 0x74, 0x27], [0xFB, 0xC7, 0xEE, 0x00, 0xB0, 0xC5],
    [0x43, 0xF8, 0xCA, 0x03, 0xF0, 0x1D], [0x2C, 0xF4, 0xB9, 0x02, 0xEE, 0xBD],
    [0x4C, 0x96, 0xEB, 0x00, 0xE3, 0x6A], [0x22, 0xC0, 0xBE, 0x01, 0xED, 0x4C],
    [0xCD, 0xB7, 0x00, 0x03, 0x5E, 0x39], [0x38, 0x10, 0xBE, 0x02, 0xC8, 0x8C],
    [0x5C, 0x88, 0x29, 0x01, 0x91, 0x8C], [0x7B, 0x54, 0x45, 0x01, 0x4B, 0x93],
    [0x31, 0x9A, 0x26, 0x03, 0x50, 0xCB], [0x54, 0x97, 0x4B, 0x01, 0x63, 0xFA],
    [0xFA, 0x4B, 0x8B, 0x00, 0x29, 0x64], [0x66, 0xF8, 0x7D, 0x02, 0x43, 0xBA],
    [0xA4, 0xE3, 0x02, 0x03, 0x3B, 0x61], [0x98, 0x7F, 0x26, 0x01, 0xC2, 0x69],
    [0xEF, 0x9C, 0x43, 0x01, 0x3C, 0x64], [0x9E, 0x46, 0x25, 0x02, 0x31, 0x2F],
    [0xEF, 0x93, 0x88, 0x03, 0x42, 0xAF], [0xDB, 0xDD, 0x17, 0x00, 0x59, 0x7B],
    [0x07, 0xF3, 0xFA, 0x00, 0xB2, 0x72], [0xCC, 0xEC, 0xC5, 0x03, 0x2E, 0x70],
    [0x77, 0x40, 0xB9, 0x03, 0x6A, 0x17], [0xFA, 0xDB, 0xCB, 0x00, 0x58, 0x53],
    [0x4C, 0x66, 0x95, 0x01, 0x70, 0xD6], [0x43, 0x6E, 0xA2, 0x03, 0x73, 0x29],
    [0x17, 0x58, 0x7D, 0x03, 0xBF, 0xC2], [0x8E, 0x07, 0xB5, 0x03, 0x28, 0xCF],
    [0xA8, 0x73, 0x21, 0x00, 0x54, 0xC5], [0xD9, 0x5E, 0xAF, 0x00, 0xEF, 0x81],
];

#[rustfmt::skip]
const TABLE_LOCK: [[u8; 6]; 256] = [
    [0xA1, 0x3B, 0x06, 0x00, 0x68, 0xED], [0xF6, 0xAE, 0x0A, 0x00, 0xA8, 0xE8],
    [0x42, 0x21, 0x71, 0x02, 0x43, 0x0F], [0xFE, 0xC4, 0xEA, 0x03, 0xF1, 0xB7],
    [0x43, 0xFB, 0xCC, 0x02, 0x2E, 0x55], [0x2E, 0xF7, 0xBE, 0x02, 0x44, 0x85],
    [0x4C, 0x95, 0xEF, 0x02, 0x0E, 0x2A], [0x20, 0xC3, 0xBB, 0x02, 0x74, 0x7C],
    [0xC9, 0xB4, 0x0A, 0x01, 0x71, 0x2D], [0x3E, 0x13, 0xB5, 0x01, 0x93, 0xE8],
    [0x58, 0x8B, 0x21, 0x00, 0x8D, 0x90], [0x7D, 0x57, 0x4C, 0x01, 0x23, 0xFF],
    [0x31, 0x99, 0x28, 0x02, 0x83, 0xE5], [0x56, 0x94, 0x44, 0x01, 0xC4, 0xA4],
    [0xFA, 0x48, 0x87, 0x02, 0xC9, 0x42], [0x64, 0xFB, 0x70, 0x01, 0xD7, 0xEC],
    [0xA0, 0xE0, 0x10, 0x01, 0x02, 0x5F], [0x9E, 0x7C, 0x35, 0x02, 0x8F, 0x27],
    [0xEB, 0x9F, 0x53, 0x00, 0x36, 0x52], [0x98, 0x45, 0x34, 0x02, 0x4F, 0x69],
    [0xEF, 0x90, 0x9E, 0x02, 0x87, 0xAB], [0xD8, 0xDE, 0x00, 0x00, 0x4A, 0x45],
    [0x07, 0xF0, 0xEE, 0x02, 0x44, 0x7E], [0xCE, 0xEF, 0xD0, 0x00, 0xAC, 0x0C],
    [0x73, 0x43, 0xA3, 0x01, 0x5E, 0x4F], [0xFC, 0xD8, 0xD0, 0x03, 0x18, 0x7B],
    [0x48, 0x65, 0x8D, 0x00, 0x77, 0x86], [0x45, 0x6C, 0xB9, 0x00, 0x04, 0x19],
    [0x17, 0x5A, 0x61, 0x01, 0x73, 0xB0], [0x8C, 0x05, 0xA8, 0x00, 0x90, 0xCD],
    [0xA8, 0x71, 0x3F, 0x01, 0xAB, 0xBF], [0xDB, 0x5C, 0xB0, 0x00, 0x64, 0x8B],
    [0xA1, 0x38, 0x02, 0x01, 0x72, 0x8F], [0xF6, 0xAD, 0x0E, 0x01, 0xB2, 0x8A],
    [0x42, 0x22, 0x75, 0x03, 0x59, 0x6D], [0xFE, 0xC7, 0xEE, 0x02, 0xEB, 0xD5],
    [0x43, 0xFB, 0xCE, 0x00, 0x3E, 0x55], [0x2E, 0xF7, 0xBC, 0x00, 0x54, 0x85],
    [0x4C, 0x95, 0xED, 0x00, 0x1E, 0x2A], [0x20, 0xC3, 0xB9, 0x00, 0x64, 0x7C],
    [0xC9, 0xB4, 0x08, 0x03, 0x61, 0x2D], [0x3E, 0x13, 0xB7, 0x03, 0x83, 0xE8],
    [0x58, 0x8B, 0x23, 0x02, 0x9D, 0x90], [0x7D, 0x57, 0x4E, 0x03, 0x33, 0xFF],
    [0x31, 0x99, 0x2A, 0x00, 0x93, 0xE5], [0x56, 0x94, 0x46, 0x03, 0xD4, 0xA4],
    [0xFA, 0x48, 0x85, 0x00, 0xD9, 0x42], [0x64, 0xFB, 0x72, 0x03, 0xC7, 0xEC],
    [0xA0, 0xE0, 0x12, 0x03, 0x12, 0x5F], [0x9E, 0x7C, 0x37, 0x00, 0x9F, 0x27],
    [0xEB, 0x9F, 0x51, 0x02, 0x26, 0x52], [0x98, 0x45, 0x36, 0x00, 0x5F, 0x69],
    [0xEF, 0x90, 0x9C, 0x00, 0x97, 0xAB], [0xD9, 0xDE, 0x02, 0x02, 0xF8, 0x0F],
    [0x07, 0xF0, 0xEC, 0x00, 0x54, 0x7E], [0xCE, 0xEF, 0xD2, 0x02, 0xBC, 0x0C],
    [0x73, 0x43, 0xA1, 0x03, 0x4E, 0x4F], [0xFC, 0xD8, 0xD2, 0x01, 0x08, 0x7B],
    [0x48, 0x65, 0x8F, 0x02, 0x67, 0x86], [0x45, 0x6D, 0xB9, 0x01, 0x10, 0x09],
    [0x17, 0x5B, 0x61, 0x00, 0x67, 0xA0], [0x8C, 0x04, 0xA8, 0x01, 0x84, 0xDD],
    [0xA8, 0x70, 0x3F, 0x00, 0xBF, 0xAF], [0xDB, 0x5D, 0xB0, 0x01, 0x70, 0x9B],
    [0xA1, 0x39, 0x02, 0x00, 0x66, 0x9F], [0xF6, 0xAC, 0x0E, 0x00, 0xA6, 0x9A],
    [0x42, 0x23, 0x75, 0x02, 0x4D, 0x7D], [0xFE, 0xC6, 0xEE, 0x03, 0xFF, 0xC5],
    [0x43, 0xF9, 0xCA, 0x01, 0x13, 0x2F], [0x2D, 0xF5, 0xB9, 0x00, 0x7B, 0xD5],
    [0x4C, 0x97, 0xEB, 0x02, 0x00, 0x58], [0x23, 0xC1, 0xBE, 0x03, 0x78, 0x24],
    [0xC9, 0xB6, 0x00, 0x00, 0x67, 0x63], [0x3D, 0x11, 0xBE, 0x01, 0x87, 0x8C],
    [0x58, 0x89, 0x29, 0x02, 0xA8, 0xD6], [0x7E, 0x55, 0x45, 0x02, 0x04, 0x93],
    [0x31, 0x9B, 0x26, 0x01, 0xB3, 0xF9], [0x55, 0x96, 0x4B, 0x03, 0xF6, 0x92],
    [0xFA, 0x4A, 0x8B, 0x02, 0xCA, 0x56], [0x67, 0xF9, 0x7D, 0x00, 0xD6, 0xD2],
    [0xA0, 0xE2, 0x02, 0x00, 0x02, 0x3B], [0x9D, 0x7E, 0x26, 0x02, 0x8D, 0x69],
    [0xEB, 0x9D, 0x43, 0x02, 0x05, 0x3E], [0x9B, 0x47, 0x25, 0x01, 0x7E, 0x2F],
    [0xEF, 0x92, 0x88, 0x01, 0xA1, 0x9D], [0xDA, 0xDC, 0x17, 0x02, 0xCC, 0x13],
    [0x07, 0xF2, 0xFA, 0x02, 0x51, 0x40], [0xCD, 0xED, 0xC5, 0x01, 0xBB, 0x18],
    [0x73, 0x41, 0xB9, 0x00, 0x53, 0x4D], [0xFF, 0xDA, 0xCB, 0x03, 0x17, 0x53],
    [0x48, 0x67, 0x95, 0x02, 0x49, 0x8C], [0x46, 0x6F, 0xA2, 0x00, 0x3C, 0x29],
    [0x17, 0x59, 0x7D, 0x01, 0x5C, 0xF0], [0x8D, 0x06, 0xB5, 0x01, 0x50, 0x93],
    [0xA8, 0x72, 0x21, 0x02, 0xB7, 0xF7], [0xDA, 0x5F, 0xAF, 0x02, 0x97, 0xDD],
    [0xA1, 0x38, 0x02, 0x01, 0x72, 0x8F], [0xF7, 0xAD, 0x0E, 0x01, 0xC4, 0xD0],
    [0x42, 0x22, 0x75, 0x03, 0x59, 0x6D], [0xFF, 0xC7, 0xEE, 0x02, 0x9D, 0x8F],
    [0x43, 0xF8, 0xCA, 0x00, 0x07, 0x3F], [0x2C, 0xF4, 0xB9, 0x01, 0x19, 0x9F],
    [0x4C, 0x96, 0xEB, 0x03, 0x14, 0x48], [0x22, 0xC0, 0xBE, 0x02, 0x1A, 0x6E],
    [0xC9, 0xB7, 0x00, 0x01, 0x73, 0x73], [0x3C, 0x10, 0xBE, 0x00, 0xE5, 0xC6],
    [0x58, 0x88, 0x29, 0x03, 0xBC, 0xC6], [0x7F, 0x54, 0x45, 0x03, 0x66, 0xD9],
    [0x31, 0x9A, 0x26, 0x00, 0xA7, 0xE9], [0x54, 0x97, 0x4B, 0x02, 0x94, 0xD8],
    [0xFA, 0x4B, 0x8B, 0x03, 0xDE, 0x46], [0x66, 0xF8, 0x7D, 0x01, 0xB4, 0x98],
    [0xA0, 0xE3, 0x02, 0x01, 0x16, 0x2B], [0x9C, 0x7F, 0x26, 0x03, 0xEF, 0x23],
    [0xEB, 0x9C, 0x43, 0x03, 0x11, 0x2E], [0x9A, 0x46, 0x25, 0x00, 0x1C, 0x65],
    [0xEF, 0x93, 0x88, 0x00, 0xB5, 0x8D], [0xDB, 0xDD, 0x17, 0x03, 0xAE, 0x59],
    [0x07, 0xF3, 0xFA, 0x03, 0x45, 0x50], [0xCC, 0xEC, 0xC5, 0x00, 0xD9, 0x52],
    [0x73, 0x40, 0xB9, 0x01, 0x47, 0x5D], [0xFE, 0xDB, 0xCB, 0x02, 0x75, 0x19],
    [0x48, 0x66, 0x95, 0x03, 0x5D, 0x9C], [0x47, 0x6E, 0xA2, 0x01, 0x5E, 0x63],
    [0x17, 0x58, 0x7D, 0x00, 0x48, 0xE0], [0x8E, 0x07, 0xB5, 0x00, 0xDF, 0xED],
    [0xA8, 0x73, 0x21, 0x03, 0xA3, 0xE7], [0xD9, 0x5E, 0xAF, 0x03, 0x18, 0xA3],
    [0xA5, 0x3B, 0x02, 0x03, 0x58, 0xB5], [0xF0, 0xAE, 0x0F, 0x02, 0xEC, 0xC0],
    [0x46, 0x21, 0x77, 0x02, 0x40, 0x5F], [0xF8, 0xC4, 0xED, 0x02, 0x86, 0x97],
    [0x43, 0xFB, 0xCE, 0x01, 0xD1, 0x3F], [0x2F, 0xF7, 0xBC, 0x01, 0xCD, 0xB5],
    [0x4C, 0x95, 0xED, 0x01, 0xF1, 0x40], [0x21, 0xC3, 0xB9, 0x01, 0xFD, 0x4C],
    [0xCD, 0xB4, 0x08, 0x03, 0x54, 0x2F], [0x3B, 0x13, 0xB7, 0x03, 0xC0, 0xB0],
    [0x5C, 0x8B, 0x23, 0x02, 0xA8, 0x92], [0x78, 0x57, 0x4E, 0x03, 0x70, 0xA7],
    [0x31, 0x99, 0x2A, 0x01, 0x7C, 0x8F], [0x57, 0x94, 0x46, 0x02, 0x4D, 0x94],
    [0xFA, 0x48, 0x85, 0x01, 0x36, 0x28], [0x65, 0xFB, 0x72, 0x02, 0x5E, 0xDC],
    [0xA4, 0xE0, 0x12, 0x03, 0x27, 0x5D], [0x9B, 0x7C, 0x37, 0x00, 0xDC, 0x7F],
    [0xEF, 0x9F, 0x51, 0x02, 0x13, 0x50], [0x9D, 0x45, 0x36, 0x00, 0x1C, 0x31],
    [0xEF, 0x90, 0x9C, 0x01, 0x78, 0xC1], [0xD8, 0xDE, 0x02, 0x03, 0x61, 0x3F],
    [0x07, 0xF0, 0xEC, 0x01, 0xBB, 0x14], [0xCF, 0xEF, 0xD2, 0x03, 0x25, 0x3C],
    [0x77, 0x43, 0xA1, 0x03, 0x7B, 0x4D], [0xF9, 0xD8, 0xD2, 0x01, 0x4B, 0x23],
    [0x4C, 0x65, 0x8F, 0x02, 0x52, 0x84], [0x40, 0x6D, 0xB9, 0x01, 0x53, 0x51],
    [0x17, 0x5B, 0x61, 0x01, 0x88, 0xCA], [0x83, 0x04, 0xA8, 0x02, 0x9F, 0xE1],
    [0xA8, 0x70, 0x3F, 0x01, 0x50, 0xC5], [0xD4, 0x5D, 0xB0, 0x02, 0x6B, 0xA7],
    [0xA5, 0x3B, 0x02, 0x02, 0x7B, 0xBD], [0xF1, 0xAE, 0x0F, 0x03, 0xB9, 0x92],
    [0x46, 0x21, 0x77, 0x03, 0x63, 0x57], [0xF9, 0xC4, 0xED, 0x03, 0xD3, 0xC5],
    [0x43, 0xFB, 0xCE, 0x00, 0xF2, 0x37], [0x2E, 0xF7, 0xBC, 0x00, 0x98, 0xE7],
    [0x4C, 0x95, 0xED, 0x00, 0xD2, 0x48], [0x20, 0xC3, 0xB9, 0x00, 0xA8, 0x1E],
    [0xCD, 0xB4, 0x08, 0x02, 0x77, 0x27], [0x3A, 0x13, 0xB7, 0x02, 0x95, 0xE2],
    [0x5C, 0x8B, 0x23, 0x03, 0x8B, 0x9A], [0x79, 0x57, 0x4E, 0x02, 0x25, 0xF5],
    [0x31, 0x99, 0x2A, 0x00, 0x5F, 0x87], [0x56, 0x94, 0x46, 0x03, 0x18, 0xC6],
    [0xFA, 0x48, 0x85, 0x00, 0x15, 0x20], [0x64, 0xFB, 0x72, 0x03, 0x0B, 0x8E],
    [0xA4, 0xE0, 0x12, 0x02, 0x04, 0x55], [0x9A, 0x7C, 0x37, 0x01, 0x89, 0x2D],
    [0xEF, 0x9F, 0x51, 0x03, 0x30, 0x58], [0x9C, 0x45, 0x36, 0x01, 0x49, 0x63],
    [0xEF, 0x90, 0x9C, 0x00, 0x5B, 0xC9], [0xD9, 0xDE, 0x02, 0x02, 0x34, 0x6D],
    [0x07, 0xF0, 0xEC, 0x00, 0x98, 0x1C], [0xCE, 0xEF, 0xD2, 0x02, 0x70, 0x6E],
    [0x77, 0x43, 0xA1, 0x02, 0x58, 0x45], [0xF8, 0xD8, 0xD2, 0x00, 0x1E, 0x71],
    [0x4C, 0x65, 0x8F, 0x03, 0x71, 0x8C], [0x41, 0x6D, 0xB9, 0x00, 0x06, 0x03],
    [0x17, 0x5B, 0x61, 0x00, 0xAB, 0xC2], [0x8C, 0x04, 0xA8, 0x01, 0x48, 0xBF],
    [0xA8, 0x70, 0x3F, 0x00, 0x73, 0xCD], [0xDB, 0x5D, 0xB0, 0x01, 0xBC, 0xF9],
    [0xA5, 0x39, 0x02, 0x01, 0x70, 0x95], [0xF2, 0xAC, 0x0E, 0x01, 0xB0, 0x90],
    [0x46, 0x23, 0x75, 0x03, 0x5B, 0x77], [0xFA, 0xC6, 0xEE, 0x02, 0xE9, 0xCF],
    [0x43, 0xF9, 0xCA, 0x01, 0xDF, 0x4D], [0x2D, 0xF5, 0xB9, 0x00, 0xB7, 0xB7],
    [0x4C, 0x97, 0xEB, 0x02, 0xCC, 0x3A], [0x23, 0xC1, 0xBE, 0x03, 0xB4, 0x46],
    [0xCD, 0xB6, 0x00, 0x01, 0x71, 0x69], [0x39, 0x11, 0xBE, 0x00, 0x91, 0x86],
    [0x5C, 0x89, 0x29, 0x03, 0xBE, 0xDC], [0x7A, 0x55, 0x45, 0x03, 0x12, 0x99],
    [0x31, 0x9B, 0x26, 0x01, 0x7F, 0x9B], [0x55, 0x96, 0x4B, 0x03, 0x3A, 0xF0],
    [0xFA, 0x4A, 0x8B, 0x02, 0x06, 0x34], [0x67, 0xF9, 0x7D, 0x00, 0x1A, 0xB0],
    [0xA4, 0xE2, 0x02, 0x01, 0x14, 0x31], [0x99, 0x7E, 0x26, 0x03, 0x9B, 0x63],
    [0xEF, 0x9D, 0x43, 0x03, 0x13, 0x34], [0x9F, 0x47, 0x25, 0x00, 0x68, 0x25],
    [0xEF, 0x92, 0x88, 0x01, 0x6D, 0xFF], [0xDA, 0xDC, 0x17, 0x02, 0x00, 0x71],
    [0x07, 0xF2, 0xFA, 0x02, 0x9D, 0x22], [0xCD, 0xED, 0xC5, 0x01, 0x77, 0x7A],
    [0x77, 0x41, 0xB9, 0x01, 0x45, 0x47], [0xFB, 0xDA, 0xCB, 0x02, 0x01, 0x59],
    [0x4C, 0x67, 0x95, 0x03, 0x5F, 0x86], [0x42, 0x6F, 0xA2, 0x01, 0x2A, 0x23],
    [0x17, 0x59, 0x7D, 0x01, 0x90, 0x92], [0x8D, 0x06, 0xB5, 0x01, 0x9C, 0xF1],
    [0xA8, 0x72, 0x21, 0x02, 0x7B, 0x95], [0xDA, 0x5F, 0xAF, 0x02, 0x5B, 0xBF],
    [0xA5, 0x38, 0x02, 0x00, 0x64, 0x85], [0xF3, 0xAD, 0x0E, 0x00, 0xD2, 0xDA],
    [0x46, 0x22, 0x75, 0x02, 0x4F, 0x67], [0xFB, 0xC7, 0xEE, 0x03, 0x8B, 0x85],
    [0x43, 0xFB, 0xCE, 0x00, 0x3E, 0x55], [0x2E, 0xF7, 0xBC, 0x00, 0x54, 0x85],
    [0x4C, 0x96, 0xE8, 0x01, 0xA6, 0x76], [0x20, 0xC3, 0xB9, 0x00, 0x64, 0x7C],
    [0xC9, 0xB4, 0x08, 0x03, 0x61, 0x2D], [0x3E, 0x13, 0xB7, 0x03, 0x83, 0xE8],
    [0x44, 0x88, 0x2A, 0x00, 0x4A, 0xA2], [0x7D, 0x57, 0x4E, 0x03, 0x33, 0xFF],
    [0x31, 0x99, 0x2A, 0x00, 0x93, 0xE5], [0x56, 0x94, 0x46, 0x03, 0xD4, 0xA4],
    [0xFA, 0x48, 0x85, 0x00, 0xD9, 0x42], [0x64, 0xFB, 0x72, 0x03, 0xC7, 0xEC],
    [0xA0, 0xE0, 0x12, 0x03, 0x12, 0x5F], [0x9E, 0x7C, 0x37, 0x00, 0x9F, 0x27],
    [0xF7, 0x9C, 0x40, 0x00, 0x33, 0x5A], [0x98, 0x45, 0x36, 0x00, 0x5F, 0x69],
    [0xEF, 0x90, 0x9C, 0x00, 0x97, 0xAB], [0xD8, 0xDE, 0x02, 0x02, 0x5A, 0x45],
    [0x07, 0xF0, 0xEC, 0x00, 0x54, 0x7E], [0xCE, 0xEF, 0xD2, 0x02, 0xBC, 0x0C],
    [0x73, 0x43, 0xA1, 0x03, 0x4E, 0x4F], [0xFC, 0xD8, 0xD2, 0x01, 0x08, 0x7B],
    [0x48, 0x65, 0x8F, 0x02, 0x67, 0x86], [0x45, 0x6C, 0xBB, 0x02, 0x14, 0x19],
    [0x17, 0x5A, 0x63, 0x03, 0x63, 0xB0], [0x8C, 0x05, 0xAA, 0x02, 0x80, 0xCD],
    [0xA8, 0x71, 0x3D, 0x03, 0xBB, 0xBF], [0xD9, 0x5E, 0xAC, 0x01, 0xAA, 0x9D],
];

/// Mazda V2 decoder (matches `SubGhzProtocolDecoderMazdaV2`).
pub struct MazdaV2Decoder {
    /// `decoder.parser_step`: false = idle (waiting for the lead-in short), true = sampling.
    sampling: bool,
    /// Set after the first of two short pulses (that half bit is skipped).
    skip_flag: bool,
    samples: [u8; SAMPLE_CAP],
    sample_count: usize,
    /// Pulses since the last periodic parse (`short_count`).
    short_count: u16,
}

impl MazdaV2Decoder {
    pub fn new() -> Self {
        Self {
            sampling: false,
            skip_flag: false,
            samples: [0; SAMPLE_CAP],
            sample_count: 0,
            short_count: 0,
        }
    }

    fn btn7(button: u8) -> u8 {
        button & 0x7F
    }

    fn button_is_lock(button: u8) -> bool {
        matches!(Self::btn7(button), BTN_LOCK | BTN_LOCK_ALT)
    }

    fn button_is_valid(button: u8) -> bool {
        matches!(
            Self::btn7(button),
            BTN_LOCK | BTN_UNLOCK | BTN_TRUNK | BTN_LOCK_ALT | BTN_UNLOCK_ALT | BTN_TRUNK_ALT
        )
    }

    /// `mazda_v2_table_row`: Lock buttons use the lock table, everything else the default table.
    fn table_row(button: u8, idx: u8) -> [u8; 6] {
        if Self::button_is_lock(button) {
            TABLE_LOCK[idx as usize]
        } else {
            TABLE_DEFAULT[idx as usize]
        }
    }

    /// `mazda_v2_layout_is_inline`.
    fn layout_is_inline(hop0: u8, button: u8) -> bool {
        matches!(Self::btn7(button), BTN_LOCK | BTN_UNLOCK | BTN_TRUNK) && (hop0 & 0x3C) == 0
    }

    /// `mazda_v2_inline_apply_idx`.
    fn inline_apply_idx(hop: &mut [u8; 6], idx: u8) {
        if (hop[0] & 0xF0) != 0xC0 {
            return;
        }
        if idx < 0x40 {
            hop[0] = 0xC0 | (idx & 1);
            hop[2] = idx & 7;
        } else {
            hop[0] = 0xC0 | ((idx & 1) << 1);
            hop[2] = idx & 0x18;
        }
        let hi = hop[1] & 0xE0;
        let b1 = (idx >> 6) & 1;
        let b3 = idx & 1;
        let b4 = (!(idx >> 1)) & 1;
        hop[1] = hi | (b4 << 4) | (b3 << 3) | (1 << 2) | (b1 << 1);
    }

    /// `mazda_v2_extract`: (hop[6], idx, hdr, button) from the left-aligned 64-bit value.
    fn extract(data: u64) -> ([u8; 6], u8, u8, u8) {
        let hop = [
            ((data >> 40) & 0xFF) as u8,
            ((data >> 32) & 0xFF) as u8,
            ((data >> 27) & 0x1F) as u8,
            ((data >> 16) & 0x07) as u8,
            ((data >> 8) & 0xFF) as u8,
            ((data >> 1) & 0x7F) as u8,
        ];
        let idx = ((data >> 19) & 0xFF) as u8;
        let hdr = ((data >> 48) & 0xFF) as u8;
        let button = ((data >> 56) & 0xFF) as u8;
        (hop, idx, hdr, button)
    }

    /// `mazda_v2_pack` (inverse of `extract`; bit 0 is always 0).
    fn pack(hop: &[u8; 6], idx: u8, hdr: u8, button: u8) -> u64 {
        let hop2 = hop[2] & 0x1F;
        let hop3 = hop[3] & 0x07;
        let hop5 = hop[5] & 0x7F;
        let hi = (hop[1] as u32)
            | ((hop[0] as u32) << 8)
            | ((hdr as u32) << 16)
            | ((button as u32) << 24);
        let mut lo = ((hop5 as u32) << 1) | ((hop[4] as u32) << 8);
        lo |= ((hop3 | (idx << 3)) as u32) << 16;
        lo |= (((idx >> 5) | (hop2 << 3)) as u32) << 24;
        ((hi as u64) << 32) | lo as u64
    }

    /// `mazda_v2_mix_hop`: re-mix the captured delta when the idx bits [2:1] change.
    fn mix_hop(hop: &mut [u8; 6], idx: u8, btn7: u8, old_bits: u8, delta_in: &[u8; 6]) {
        let mut delta = *delta_in;

        let mut r2: u8 = if Self::button_is_lock(btn7) || idx > 0xDF {
            0
        } else {
            1
        };
        let flag13: u8 = (delta[2] == 13) as u8;
        r2 ^= 1;
        if flag13 != r2 {
            let pad = if old_bits > 1 { &XOR_HI } else { &XOR_LO };
            for (d, p) in delta.iter_mut().zip(pad.iter()) {
                *d ^= p;
            }
        }

        let lr_idx = idx >> 1;
        let mut r3 = old_bits;
        let r7 = (idx >> 1) & 3;

        let mut out0: u8 = if r2 != 0 { 0x28 } else { 0x38 };
        if r3 <= 1 {
            out0 = delta[0];
        }
        if lr_idx & 2 != 0 {
            out0 = 0;
        }

        let mut out1 = delta[1];
        if r3 & 1 != 0 {
            out1 ^= 4;
        }
        if lr_idx & 1 != 0 {
            out1 ^= 4;
        }

        let out2: u8 = if r2 == 0 { 0x0F } else { 0x0D };
        let mut out3 = delta[3];
        let lr2: u8 = if r2 == 0 { 2 } else { 1 };
        if r3 > 1 {
            out3 ^= lr2;
        }
        if lr_idx & 2 != 0 {
            out3 ^= lr2;
        }

        let mut out4 = delta[4];
        let xor_a: u8 = if r2 == 0 { 0x55 } else { 0x7E };
        let xor_b: u8 = if r2 == 0 { 0x54 } else { 0x36 };
        let mut walk = delta[5];
        let mut walked = false;
        while r3 != r7 {
            let bit0_clear = (r3 & 1) == 0;
            r3 = (r3 + 1) & 3;
            if bit0_clear {
                out4 ^= 0xDC;
                walk ^= 0x60;
            } else {
                out4 ^= xor_a;
                walk ^= xor_b;
            }
            walked = true;
        }

        let mixed = [
            out0,
            out1,
            out2,
            out3,
            out4,
            if walked { walk } else { delta[5] },
        ];
        for (h, m) in hop.iter_mut().zip(mixed.iter()) {
            *h ^= m;
        }
    }

    /// `mazda_v2_advance_hop`: the table row for the new idx, carrying the captured fob's delta.
    fn advance_hop(
        new_idx: u8,
        button: u8,
        mismatch: bool,
        xor_idx_bits: u8,
        delta: &[u8; 6],
    ) -> [u8; 6] {
        let mut hop = Self::table_row(button, new_idx);
        if !mismatch {
            return hop;
        }
        if (new_idx >> 1) & 3 == xor_idx_bits {
            for (h, d) in hop.iter_mut().zip(delta.iter()) {
                *h ^= d;
            }
            return hop;
        }
        Self::mix_hop(&mut hop, new_idx, button & 0x7F, xor_idx_bits, delta);
        hop
    }

    /// `mazda_v2_get_btn_code` (ARF mapping UP=Lock, DOWN=Unlock, LEFT=Trunk) for a KAT generic
    /// command; keeps the captured button's alt family. Other commands keep the captured button.
    fn map_button(button: u8, original: u8) -> u8 {
        let original = Self::btn7(original);
        let alt = matches!(original, BTN_LOCK_ALT | BTN_UNLOCK_ALT | BTN_TRUNK_ALT);
        let (lock, unlock, trunk) = if alt {
            (BTN_LOCK_ALT, BTN_UNLOCK_ALT, BTN_TRUNK_ALT)
        } else {
            (BTN_LOCK, BTN_UNLOCK, BTN_TRUNK)
        };
        match button {
            KAT_BTN_LOCK => lock,
            KAT_BTN_UNLOCK => unlock,
            KAT_BTN_TRUNK => trunk,
            _ => original,
        }
    }

    /// The next key for a fresh press (`subghz_protocol_encoder_mazda_v2_deserialize`): idx + 1,
    /// optional button re-stamp, hop advanced through the lookup tables.
    fn next_key(data: u64, button: u8) -> u64 {
        let (hop, idx, hdr, captured_btn) = Self::extract(data);
        let inline_layout = Self::layout_is_inline(hop[0], captured_btn);

        let row = Self::table_row(captured_btn, idx);
        let mut delta = [0u8; 6];
        for i in 0..6 {
            delta[i] = hop[i] ^ row[i];
        }
        let mismatch = delta.iter().any(|&d| d != 0);
        let xor_idx_bits = (idx >> 1) & 3;

        let btn_hi = captured_btn & 0x80;
        let btn = (Self::map_button(button, captured_btn) & 0x7F) | btn_hi;
        // furi_hal_subghz_get_rolling_counter_mult() defaults to 1.
        let cnt = idx.wrapping_add(1);

        let new_hop = if inline_layout {
            let mut h = hop;
            Self::inline_apply_idx(&mut h, cnt);
            h
        } else {
            Self::advance_hop(cnt, btn, mismatch, xor_idx_bits, &delta)
        };
        Self::pack(&new_hop, cnt, hdr, btn)
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

    /// `mazda_v2_add_bit`: two-state level encoder. `high` = the line currently ends HIGH (C state
    /// 1); low is C state 2. Each bit ends on its own value.
    fn enc_add_bit(signal: &mut Vec<LevelDuration>, high: &mut bool, bit: bool) {
        if !*high {
            if bit {
                Self::enc_add_level(signal, true, TE_LONG);
                *high = true;
            } else {
                Self::enc_add_level(signal, true, TE_SHORT);
                Self::enc_add_level(signal, false, TE_SHORT);
            }
        } else if !bit {
            Self::enc_add_level(signal, false, TE_LONG);
            *high = false;
        } else {
            Self::enc_add_level(signal, false, TE_SHORT);
            Self::enc_add_level(signal, true, TE_SHORT);
        }
    }

    /// One upload copy (`mazda_v2_build_upload`, with KAT's preamble fixes — see module docs).
    fn append_upload(signal: &mut Vec<LevelDuration>, data: u64) {
        // KAT: a HIGH the sampler takes for a gap in every state (> DUR_LONG: idle stays idle,
        // armed parses and idles), so that the preamble's leading LOW is a pulse of its own on
        // air and arms the sampler, whatever preceded the copy (silence, noise, the gap).
        Self::enc_add_level(signal, true, TE_GAP);
        // C: PREAMBLE_PAIRS × (L,H). The final HIGH is dropped so the sync's first HIGH stays 1000µs.
        for i in 0..PREAMBLE_PAIRS {
            Self::enc_add_level(signal, false, TE_SHORT);
            if i + 1 < PREAMBLE_PAIRS {
                Self::enc_add_level(signal, true, TE_SHORT);
            }
        }

        let mut bytes = [0u8; 9];
        bytes[0] = SYNC;
        bytes[1..].copy_from_slice(&data.to_be_bytes());
        let mut high = false;
        for bit_i in 0..PAYLOAD_BITS {
            let bit = (bytes[bit_i / 8] >> (7 - bit_i % 8)) & 1 != 0;
            Self::enc_add_bit(signal, &mut high, bit);
        }

        // Tail half-bit pair, then the 3000µs HIGH gap.
        if high {
            Self::enc_add_level(signal, false, TE_SHORT);
            Self::enc_add_level(signal, true, TE_SHORT);
        } else {
            Self::enc_add_level(signal, true, TE_SHORT);
            Self::enc_add_level(signal, false, TE_SHORT);
        }
        Self::enc_add_level(signal, true, TE_GAP);
    }

    /// Strict sync match: Some(false) for 0xEC, Some(true) for the inverted 0x13, else None.
    /// (Tighter than the C, whose inverted path checks only the first sync sample.)
    fn sync_match(window: &[u8]) -> Option<bool> {
        if window == PAT_A {
            Some(false)
        } else if window == PAT_B {
            Some(true)
        } else {
            None
        }
    }

    fn signal_from_value(value: u64) -> DecodedSignal {
        let (_hop, idx, _hdr, button) = Self::extract(value);
        DecodedSignal {
            // No plaintext serial in this frame (the C's `serial = idx` is a placeholder).
            serial: None,
            button: Some(button),
            counter: Some(idx as u16),
            // No CRC: preamble + sync + button validated the frame (a loaded key is not gated, as
            // in the C; this reports the same button/value checks on it).
            crc_valid: Self::button_is_valid(button) && value != 0 && value != INVALID_VALUE,
            data: value,
            data_count_bit: MIN_COUNT_BIT,
            encoder_capable: true,
            extra: None,
            protocol_display_name: None,
        }
    }

    /// `mazda_v2_try_parse`: scan the sample buffer for preamble + sync + 63 payload samples.
    fn try_parse(&mut self) -> Option<DecodedSignal> {
        let sample_count = self.sample_count;
        if sample_count <= PARSE_MIN_SAMPLES {
            return None;
        }
        let remain = sample_count - FRAME_SAMPLES;
        for start in 0..=remain {
            let invert = match Self::sync_match(&self.samples[start..start + 8]) {
                Some(invert) => invert,
                None => continue,
            };
            // Preamble samples are 0 (normal) or 1 (inverted levels).
            let pol = invert as u8;
            let preamble = self.samples[..start]
                .iter()
                .rev()
                .take_while(|&&s| s == pol)
                .count();
            if preamble <= PREAMBLE_MIN_SAMPLES {
                continue;
            }

            let mut value = 0u64;
            for &s in &self.samples[start + 8..start + 8 + PAYLOAD_SAMPLES] {
                value = (value << 1) | (s ^ invert as u8) as u64;
            }
            value <<= 1;
            if value == 0 || value == INVALID_VALUE {
                continue;
            }
            if !Self::button_is_valid((value >> 56) as u8) {
                continue;
            }

            // Consume the frame, keep any leftover samples (the C memmove).
            let consumed = start + FRAME_SAMPLES;
            self.samples.copy_within(consumed..sample_count, 0);
            self.sample_count = sample_count - consumed;
            return Some(Self::signal_from_value(value));
        }
        None
    }

    /// Periodic / buffer-full parse shared by the short-pulse path and `mazda_v2_after_store`.
    fn maybe_parse(&mut self) -> Option<DecodedSignal> {
        if self.short_count > PERIODIC_PARSE_PULSES && self.sample_count > PERIODIC_PARSE_SAMPLES {
            self.short_count = 0;
            if let Some(decoded) = self.try_parse() {
                return Some(decoded);
            }
        }
        if self.sample_count >= SAMPLE_PARSE_MIN {
            let result = self.try_parse();
            if self.sample_count >= SAMPLE_PARSE_MIN {
                self.reset_parser();
            }
            return result;
        }
        None
    }

    /// `mazda_v2_store_sample` + `mazda_v2_after_store`.
    fn store_sample(&mut self, level: bool) -> Option<DecodedSignal> {
        if self.sample_count < SAMPLE_CAP {
            self.samples[self.sample_count] = level as u8;
            self.sample_count += 1;
        }
        self.skip_flag = false;
        self.short_count = self.short_count.wrapping_add(1);
        self.maybe_parse()
    }

    /// parser_step = 0, sample_count = 0, skip_flag = 0 (short_count is kept, as in the C).
    fn reset_parser(&mut self) {
        self.sampling = false;
        self.sample_count = 0;
        self.skip_flag = false;
    }
}

impl ProtocolDecoder for MazdaV2Decoder {
    fn name(&self) -> &'static str {
        "Mazda V2"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: MIN_COUNT_BIT,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        // SubGhzProtocolFlag_315 | SubGhzProtocolFlag_433.
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.reset_parser();
        self.short_count = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        if duration <= DUR_FILTER {
            return None;
        }

        if !self.sampling {
            if duration > DUR_SHORT {
                return None;
            }
            // Lead-in half bit arms the sampler.
            self.sampling = true;
            self.sample_count = 0;
            self.short_count = 0;
            self.skip_flag = false;
            return None;
        }

        if duration <= DUR_SHORT {
            if self.skip_flag {
                return self.store_sample(level);
            }
            self.skip_flag = true;
            self.short_count = self.short_count.wrapping_add(1);
            return self.maybe_parse();
        }

        if duration <= DUR_LONG {
            return self.store_sample(level);
        }

        // Gap: final parse of the burst, then back to idle.
        let result = if self.sample_count > PARSE_MIN_SAMPLES {
            self.try_parse()
        } else {
            None
        };
        self.reset_parser();
        result
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        let data = Self::next_key(decoded.data, button);
        let mut signal =
            Vec::with_capacity(REPEAT_DEFAULT * (PREAMBLE_PAIRS * 2 + PAYLOAD_BITS * 2 + 3));
        for _ in 0..REPEAT_DEFAULT {
            Self::append_upload(&mut signal, data);
        }
        Some(signal)
    }

    /// `subghz_protocol_decoder_mazda_v2_deserialize`: a 63-bit key, split into fields.
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != MIN_COUNT_BIT {
            return None;
        }
        Some(Self::signal_from_value(key.key_u64()?))
    }

    /// `subghz_protocol_decoder_mazda_v2_serialize`: Key, then `Hdr` and the `Disp` line.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        let (_hop, _idx, hdr, button) = Self::extract(decoded.data);
        let name = match Self::btn7(button) {
            BTN_LOCK | BTN_LOCK_ALT => "Lock",
            BTN_UNLOCK | BTN_UNLOCK_ALT => "Unlock",
            BTN_TRUNK | BTN_TRUNK_ALT => "Trunk",
            _ => "Unknown",
        };
        let mut key = FlipperKey::for_protocol("Mazda V2", MIN_COUNT_BIT, decoded.data);
        key.push_u32("Hdr", hdr.into());
        key.push("Disp", format!("Mazda V2 - {name}"));
        Some(key)
    }
}

impl Default for MazdaV2Decoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::ProtocolRegistry;

    /// A captured Unlock frame (button 0x08, hdr 0x21, idx 0x83, inline layout: hop0 = 0xC1).
    const KEY: u64 = 0x0821_C13B_8C1C_4180;

    fn captured(data: u64) -> DecodedSignal {
        MazdaV2Decoder::signal_from_value(data)
    }

    fn decode_all(signal: &[LevelDuration], invert: bool) -> Vec<DecodedSignal> {
        let mut dec = MazdaV2Decoder::new();
        let mut out: Vec<DecodedSignal> = signal
            .iter()
            .filter_map(|ld| dec.feed(ld.level ^ invert, ld.duration_us))
            .collect();
        // Trailing silence (a LOW in the signal's own polarity) flushes any unfinished burst.
        out.extend(dec.feed(invert, 10_000));
        out
    }

    fn one_copy(data: u64) -> Vec<LevelDuration> {
        let mut signal = Vec::new();
        MazdaV2Decoder::append_upload(&mut signal, data);
        signal
    }

    /// `before` then one copy, as a receiver sees them: adjacent equal levels merged.
    fn on_air(before: &[(bool, u32)]) -> Vec<LevelDuration> {
        let mut out: Vec<LevelDuration> = before.iter().map(|&(l, d)| LevelDuration::new(l, d)).collect();
        for p in one_copy(KEY) {
            match out.last_mut() {
                Some(last) if last.level == p.level => last.duration_us += p.duration_us,
                _ => out.push(p),
            }
        }
        out
    }

    /// On air the carrier is off before a transmission: a copy behind 20 ms of silence, equal
    /// levels merged, must still decode (KAT 1.8.6 and the C encoder start with the preamble's
    /// LOW, which merges into that silence; neither decoder read the result).
    #[test]
    fn upload_decodes_after_silence() {
        let signal = on_air(&[(false, 20_000)]);
        assert_eq!((signal[0].level, signal[0].duration_us), (false, 20_000));
        assert_eq!((signal[1].level, signal[1].duration_us), (true, TE_GAP));
        let decoded = decode_all(&signal, false);
        assert_eq!(decoded.len(), 1);
        assert_eq!(decoded[0].data, KEY);
    }

    /// A Flipper's receiver puts out noise pulses before a transmission, which arm the sampler
    /// with either parity (the pulses of the test rig's recording of KAT 1.8.6's upload, and the
    /// same with one more pair). The lead-in must make the copy decode behind both; the C's
    /// upload decodes behind one, a 1000µs lead-in behind the other.
    #[test]
    fn upload_decodes_after_noise_of_either_parity() {
        let noise = [(true, 191), (false, 848), (true, 63), (false, 870), (true, 169), (false, 68), (true, 65), (false, 870), (true, 131), (false, 622)];
        let mut other = noise.to_vec();
        other.extend([(false, 300), (true, 200)]);
        for before in [&noise[..], &other[..]] {
            let mut signal = vec![LevelDuration::new(false, 20_000)];
            signal.extend(on_air(before));
            let decoded = decode_all(&signal, false);
            assert_eq!(decoded.len(), 1, "{} noise pulses", before.len());
            assert_eq!(decoded[0].data, KEY);
        }
    }

    /// Byte sum and FNV-1a-32 of each table, computed from the C source text by the generator
    /// script (`static const uint8_t mazda_v2_table_{default,lock}[256][6]`, identical in the ARF
    /// and doctor-firmware copies). Any transcription slip changes these.
    #[test]
    fn tables_match_c_source_checksums() {
        fn sums(table: &[[u8; 6]; 256]) -> (u32, u32) {
            let mut sum = 0u32;
            let mut fnv = 0x811C_9DC5u32;
            for b in table.iter().flatten() {
                sum += *b as u32;
                fnv = (fnv ^ *b as u32).wrapping_mul(0x0100_0193);
            }
            (sum, fnv)
        }
        assert_eq!(sums(&TABLE_DEFAULT), (163_700, 0x8902_ACF5));
        assert_eq!(sums(&TABLE_LOCK), (164_706, 0x15AB_6859));
        assert_eq!(TABLE_DEFAULT[0], [0xB9, 0x3B, 0x01, 0x03, 0x41, 0x81]);
        assert_eq!(TABLE_DEFAULT[255], [0xD9, 0x5E, 0xAF, 0x00, 0xEF, 0x81]);
        assert_eq!(TABLE_LOCK[0], [0xA1, 0x3B, 0x06, 0x00, 0x68, 0xED]);
        assert_eq!(TABLE_LOCK[255], [0xD9, 0x5E, 0xAC, 0x01, 0xAA, 0x9D]);
    }

    #[test]
    fn extract_pack_roundtrip() {
        let (hop, idx, hdr, btn) = MazdaV2Decoder::extract(KEY);
        assert_eq!((idx, hdr, btn), (0x83, 0x21, 0x08));
        assert!(MazdaV2Decoder::layout_is_inline(hop[0], btn));
        assert_eq!(MazdaV2Decoder::pack(&hop, idx, hdr, btn), KEY);
    }

    /// `next_key` against golden vectors produced by compiling the C reference's own
    /// extract/pack/table_row/inline_apply_idx/advance_hop/mix_hop with a harness that mirrors
    /// `subghz_protocol_encoder_mazda_v2_deserialize` (rolling multiplier 1). Cases cover inline
    /// layouts, table-exact rows (default + lock), idx wrap / idx > 0xDF, alt buttons, bit 7 and
    /// random frames. Columns: keep (unmapped command), Lock, Unlock, Trunk.
    #[test]
    fn next_key_matches_c_reference() {
        const KAT_BUTTONS: [u8; 4] = [0x08, KAT_BTN_LOCK, KAT_BTN_UNLOCK, KAT_BTN_TRUNK];
        #[rustfmt::skip]
        const GOLDEN: [(u64, [u64; 4]); 40] = [
            (0x085CC1A519FA77AA, [0x085CC0B6020277AA, 0x045CC0B6020277AA, 0x085CC0B6020277AA, 0x105CC0B6020277AA]),
            (0x105CC1A51BF277AA, [0x105CC2AEC3FA77AA, 0x045CC2AEC3FA77AA, 0x085CC2AEC3FA77AA, 0x105CC2AEC3FA77AA]),
            (0x043301A5188277AA, [0x043301A5188A77AA, 0x043301A5188A77AA, 0x083301A5188A77AA, 0x103301A5188A77AA]),
            (0x0821D1B458434D36, [0x08212713A04BD908, 0x04213E13A84993D0, 0x08212713A04BD908, 0x10212713A04BD908]),
            (0x0821B8E088833ED2, [0x0821877CA088C596, 0x04219E7CA88A8F4E, 0x0821877CA088C596, 0x1021877CA088C596]),
            (0x0821D8DE08AA3CA2, [0x082107F078B0E6F4, 0x042107F070B244FC, 0x082107F078B0E6F4, 0x102107F078B0E6F4]),
            (0x0821D1B459426E26, [0x08212613A14A8CAC, 0x04213E13B94B83D0, 0x08212613A14A8CAC, 0x10212613A14A8CAC]),
            (0x0821B8E089821DC2, [0x0821867CA1899032, 0x04219E7CB9889F4E, 0x0821867CA1899032, 0x1021867CA1899032]),
            (0x0421C9B45041715A, [0x04213E13A84993D0, 0x04213E13A84993D0, 0x08212713A04BD908, 0x10212713A04BD908]),
            (0x0421A0E0808102BE, [0x04219E7CA88A8F4E, 0x04219E7CA88A8F4E, 0x0821877CA088C596, 0x1021877CA088C596]),
            (0x0421D8DE00A84A8A, [0x042107F070B244FC, 0x042107F070B244FC, 0x082107F078B0E6F4, 0x102107F078B0E6F4]),
            (0x0421C9B44143615A, [0x04213E13B94B83D0, 0x04213E13B94B83D0, 0x08212613A14A8CAC, 0x10212613A14A8CAC]),
            (0x0A403C126FFD9954, [0x0A408173700462FC, 0x06408973580753F0, 0x0A408173700462FC, 0x0E408173700462FC]),
            (0x0E403C126F2D9954, [0x0E404C7437347B2A, 0x06404C742F353E12, 0x0A404C7437347B2A, 0x0E404C7437347B2A]),
            (0x841B77AE0BF34DAC, [0x841B06837BFBF624, 0x841B06837BFBF624, 0x881B068363FA673C, 0x901B068363FA673C]),
            (0x0A07CE91E5906136, [0x0A07BE4BD59B1A14, 0x0607BD484D981438, 0x0A07BE4BD59B1A14, 0x0E07BE4BD59B1A14]),
            (0x08EB16E0A1C54AEC, [0x08EB997B39CF0C84, 0x04EB817B21CE03F8, 0x08EB997B39CF0C84, 0x10EB997B39CF0C84]),
            (0x10D2E144D6E8F2CE, [0x10D2A83476F3C9C6, 0x04D2A83466F3C156, 0x08D2A83476F3C9C6, 0x10D2A83476F3C9C6]),
            (0x84AA4E85B0D6E28A, [0x84AA438C10DE91B4, 0x84AA438C10DE91B4, 0x88AA5A8D08DFDF4C, 0x90AA5A8D08DFDF4C]),
            (0x06F474FFB8E8AB14, [0x06F4A88F90F27F20, 0x06F4A88F90F27F20, 0x0AF4A88E98F0EA18, 0x0EF4A88E98F0EA18]),
            (0x84BC79F8ADA711FC, [0x84BC4FB65DAD7EB4, 0x84BC4FB65DAD7EB4, 0x88BC4CB5F5AF652C, 0x90BC4CB5F5AF652C]),
            (0x8A2199E83F5A101E, [0x8A2131225F616D16, 0x862131213F62AE4A, 0x8A2131225F616D16, 0x8E2131225F616D16]),
            (0x0E3CFC387DFAE6B8, [0x0E3C9D586E007FC8, 0x063C8D587E035C9C, 0x0A3C9D586E007FC8, 0x0E3C9D586E007FC8]),
            (0x085639B16B714B4E, [0x0856A502DB7B21F2, 0x0456A502C37AB0EA, 0x0856A502DB7B21F2, 0x1056A502DB7B21F2]),
            (0x8A4B0EC76D00616C, [0x8A4B59520D08A166, 0x864B5A51050887C2, 0x8A4B59520D08A166, 0x8E4B59520D08A166]),
            (0x841A50D59C0AA21A, [0x841AF65ED412CA30, 0x841AF65ED412CA30, 0x881AFE5EDC13DD98, 0x901AFE5EDC13DD98]),
            (0x0A1ECD02ED7C0CBC, [0x0A1E9C1D6D875716, 0x061E8C1EFD8670AA, 0x0A1E9C1D6D875716, 0x0E1E9C1D6D875716]),
            (0x8A86B3D3CDBDF612, [0x8A864F7BB5C791C0, 0x86865F7865C6BBB0, 0x8A864F7BB5C791C0, 0x8E864F7BB5C791C0]),
            (0x0E81DA7F6993082C, [0x0E81A9A55199715A, 0x0681B1A549987E26, 0x0A81A9A55199715A, 0x0E81A9A55199715A]),
            (0x0E716E1E6CED8136, [0x0E71A86F84F542BE, 0x0671A86E94F57D1E, 0x0A71A86F84F542BE, 0x0E71A86F84F542BE]),
            (0x0A07B27A78869B5A, [0x0A078DE6508D601E, 0x060794E6588F2AC6, 0x0A078DE6508D601E, 0x0E078DE6508D601E]),
            (0x102B4C36D0DB7FEC, [0x102B170468E0B5C6, 0x042B170560E220FE, 0x082B170468E0B5C6, 0x102B170468E0B5C6]),
            (0x08C78FB373FFBFF6, [0x08C785D274053416, 0x04C78DD27C0423BE, 0x08C785D274053416, 0x10C785D274053416]),
            (0x04A69BA126CD29B2, [0x04A63C18C6D4B318, 0x04A63C18C6D4B318, 0x08A62C18D6D7904C, 0x10A62C18D6D7904C]),
            (0x101CE28856D20E5E, [0x101CEC80EED87B14, 0x041CEC80EEDB4094, 0x081CEC80EED87B14, 0x101CEC80EED87B14]),
            (0x0AB911499417AAB8, [0x0AB9A7AC1C1EAAFC, 0x06B9B7AC5C1E6354, 0x0AB9A7AC1C1EAAFC, 0x0EB9A7AC1C1EAAFC]),
            (0x10C88EAB2F9AD4BA, [0x10C8EF7A2FA2C102, 0x04C8EF798FA1140A, 0x08C8EF7A2FA2C102, 0x10C8EF7A2FA2C102]),
            (0x0E0F3414C47C9C0C, [0x0E0F8E0B3487C7A6, 0x060F8C0BFC854AB2, 0x0A0F8E0B3487C7A6, 0x0E0F8E0B3487C7A6]),
            (0x0A674CF841EE8AB8, [0x0A67A88899F76D9C, 0x0667A88891F5CF94, 0x0A67A88899F76D9C, 0x0E67A88899F76D9C]),
            (0x088EB22E1EA5A35E, [0x088E8760E6AECE42, 0x048E8760E6ADF5C2, 0x088E8760E6AECE42, 0x108E8760E6AECE42]),
        ];
        for (data, outs) in GOLDEN {
            for (kat_btn, want) in KAT_BUTTONS.iter().zip(outs) {
                assert_eq!(
                    MazdaV2Decoder::next_key(data, *kat_btn),
                    want,
                    "next_key({data:#018X}, {kat_btn:#04X})"
                );
            }
        }
    }

    /// One upload copy equals the C `mazda_v2_build_upload` output for KEY with the final preamble
    /// HIGH dropped (adjacent levels merged): entry count, total duration and an FNV-1a-64 over
    /// `(level << 32 | duration)` little-endian words, all computed by the C harness.
    #[test]
    fn upload_matches_c_build_upload() {
        let signal = one_copy(KEY);
        // KAT's 3000µs lead-in HIGH precedes the C's upload (its 480 entries, 254 500µs).
        assert_eq!((signal[0].level, signal[0].duration_us), (true, TE_GAP));
        let signal = &signal[1..];
        let mut fnv = 0xCBF2_9CE4_8422_2325u64;
        for ld in signal {
            let word = ((ld.level as u64) << 32) | ld.duration_us as u64;
            for b in word.to_le_bytes() {
                fnv = (fnv ^ b as u64).wrapping_mul(0x0000_0100_0000_01B3);
            }
        }
        let total: u64 = signal.iter().map(|ld| ld.duration_us as u64).sum();
        assert_eq!(
            (signal.len(), total, fnv),
            (480, 254_500, 0xA6E3_CCAF_8A08_BC73)
        );
        // Preamble starts with the LOW lead-in and the sync's first HIGH is a clean 1000µs.
        assert_eq!((signal[0].level, signal[0].duration_us), (false, TE_SHORT));
        assert_eq!(
            (signal[359].level, signal[359].duration_us),
            (true, TE_LONG)
        );
        assert!(signal
            .iter()
            .all(|ld| ld.duration_us <= DUR_LONG || ld.duration_us == TE_GAP));
    }

    #[test]
    fn encode_decode_roundtrip() {
        let encoder = MazdaV2Decoder::new();
        // KEY (inline layout) plus non-inline frames on the default and lock tables.
        for key in [
            KEY,
            0x0821_D1B4_5843_4D36,
            0x0421_C9B4_5041_715A,
            0x8A40_3C12_6FFD_992A,
        ] {
            for (kat_btn, want_btn7) in [
                (KAT_BTN_LOCK, None),
                (KAT_BTN_UNLOCK, None),
                (KAT_BTN_TRUNK, None),
                (0x08, Some(MazdaV2Decoder::btn7((key >> 56) as u8))),
            ] {
                let signal = encoder.encode(&captured(key), kat_btn).expect("encode");
                let want = MazdaV2Decoder::next_key(key, kat_btn);
                let decodes = decode_all(&signal, false);
                assert_eq!(
                    decodes.len(),
                    REPEAT_DEFAULT,
                    "key {key:#018X} btn {kat_btn:#04X}"
                );
                for d in &decodes {
                    assert_eq!(d.data, want);
                    assert_eq!(d.data_count_bit, MIN_COUNT_BIT);
                    assert_eq!(
                        d.counter,
                        Some((((key >> 19) as u8).wrapping_add(1)) as u16)
                    );
                    assert_eq!(d.button, Some((want >> 56) as u8));
                    assert_eq!(d.serial, None);
                    assert!(d.crc_valid && d.encoder_capable);
                    if let Some(b7) = want_btn7 {
                        assert_eq!(MazdaV2Decoder::btn7((want >> 56) as u8), b7);
                    }
                }
            }
        }
    }

    #[test]
    fn decodes_inverted_levels() {
        let decodes = decode_all(&one_copy(KEY), true);
        assert_eq!(decodes.len(), 1);
        assert_eq!(decodes[0].data, KEY);
        assert_eq!(decodes[0].counter, Some(0x83));
        assert_eq!(decodes[0].button, Some(0x08));
    }

    /// Build one copy with a custom preamble length / sync byte (same line code as append_upload).
    fn frame(data: u64, preamble_pairs: usize, sync: u8) -> Vec<LevelDuration> {
        let mut signal = Vec::new();
        for i in 0..preamble_pairs {
            MazdaV2Decoder::enc_add_level(&mut signal, false, TE_SHORT);
            if i + 1 < preamble_pairs {
                MazdaV2Decoder::enc_add_level(&mut signal, true, TE_SHORT);
            }
        }
        let mut bytes = [0u8; 9];
        bytes[0] = sync;
        bytes[1..].copy_from_slice(&data.to_be_bytes());
        let mut high = false;
        for bit_i in 0..PAYLOAD_BITS {
            let bit = (bytes[bit_i / 8] >> (7 - bit_i % 8)) & 1 != 0;
            MazdaV2Decoder::enc_add_bit(&mut signal, &mut high, bit);
        }
        MazdaV2Decoder::enc_add_level(&mut signal, !high, TE_SHORT);
        MazdaV2Decoder::enc_add_level(&mut signal, high, TE_SHORT);
        MazdaV2Decoder::enc_add_level(&mut signal, true, TE_GAP);
        signal
    }

    fn pulses(signal: &[LevelDuration]) -> Vec<(bool, u32)> {
        signal.iter().map(|ld| (ld.level, ld.duration_us)).collect()
    }

    #[test]
    fn rejects_corrupted_frames() {
        assert_eq!(
            pulses(&frame(KEY, PREAMBLE_PAIRS, SYNC)),
            pulses(&one_copy(KEY)[1..]),
            "helper mirrors append_upload (after its lead-in HIGH)"
        );
        assert_eq!(
            decode_all(&frame(KEY, PREAMBLE_PAIRS, SYNC), false).len(),
            1
        );
        // One sync bit flipped.
        assert!(decode_all(&frame(KEY, PREAMBLE_PAIRS, SYNC ^ 0x01), false).is_empty());
        assert!(decode_all(&frame(KEY, PREAMBLE_PAIRS, SYNC ^ 0x40), false).is_empty());
        // Invalid button byte (0x05 / 0x7F are not Lock/Unlock/Trunk codes).
        assert!(decode_all(
            &frame((KEY & !(0xFF << 56)) | (0x05 << 56), 180, SYNC),
            false
        )
        .is_empty());
        assert!(decode_all(
            &frame((KEY & !(0xFF << 56)) | (0x7F << 56), 180, SYNC),
            false
        )
        .is_empty());
        // Preamble boundary: the run before the sync must exceed 99 samples (pairs - 1 samples).
        assert!(decode_all(&frame(KEY, 100, SYNC), false).is_empty());
        assert_eq!(decode_all(&frame(KEY, 101, SYNC), false).len(), 1);
    }

    /// The C accepts an inverted-level frame whose sync only starts like 0x13 (it checks just the
    /// first sample once the inverted path is taken); KAT requires the full inverted sync.
    #[test]
    fn inverted_sync_is_checked_in_full() {
        assert_eq!(decode_all(&frame(KEY, PREAMBLE_PAIRS, SYNC), true).len(), 1);
        assert!(decode_all(&frame(KEY, PREAMBLE_PAIRS, SYNC ^ 0x01), true).is_empty());
        assert!(decode_all(&frame(KEY, PREAMBLE_PAIRS, SYNC ^ 0x10), true).is_empty());
    }

    /// Through the registry the encoder output decodes as Mazda V2 (and nothing earlier in the
    /// registry claims it) at both supported bands.
    #[test]
    fn registry_decodes_encoder_output() {
        let signal = MazdaV2Decoder::new()
            .encode(&captured(0x0821_D1B4_5843_4D36), KAT_BTN_LOCK)
            .unwrap();
        for freq in [315_000_000, 433_920_000] {
            let mut reg = ProtocolRegistry::new();
            let names: Vec<String> = reg
                .process_signal_stream(&signal, freq)
                .into_iter()
                .map(|(n, _, _)| n)
                .collect();
            assert_eq!(names, vec!["Mazda V2"; REPEAT_DEFAULT], "at {freq} Hz");
        }
    }

    /// Mazda Siemens (250/500µs) TX frames are all "short" pulses to the Mazda V2 sampler, so they
    /// can never form the 0xEC sync: Mazda V2 must stay silent on them.
    #[test]
    fn ignores_mazda_siemens_frames() {
        let reg = ProtocolRegistry::new();
        let siemens = reg.get("Mazda Siemens").expect("Mazda Siemens registered");
        let seed = DecodedSignal {
            serial: Some(0x1234_5678),
            button: Some(0x20),
            counter: Some(5),
            crc_valid: true,
            data: 0x1234_5678_2000_0500,
            data_count_bit: 64,
            encoder_capable: true,
            extra: None,
            protocol_display_name: None,
        };
        for btn in [0x01, 0x02, 0x04] {
            let signal = siemens.encode(&seed, btn).expect("siemens encode");
            assert!(decode_all(&signal, false).is_empty());
            assert!(decode_all(&signal, true).is_empty());
        }
    }

    /// Key files: the frame round-trips with the C's lines, an ARF-written key loads (extra lines
    /// such as doctor's generic fields are ignored), and TX from it decodes.
    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::{assert_round_trip, key};
        let dec = MazdaV2Decoder::new();
        let wire = dec.encode(&captured(KEY), 0x40).unwrap();
        let decoded = decode_all(&wire, false).remove(0);
        let text = assert_round_trip(&dec, &decoded);
        let hex = |v: u64| v.to_be_bytes().iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(" ");
        assert!(
            text.ends_with(&format!(
                "Protocol: Mazda V2\nBit: 63\nKey: {}\nHdr: 33\nDisp: Mazda V2 - Unlock\n",
                hex(decoded.data)
            )),
            "{text}"
        );
        // A doctor-firmware file of the same frame (generic Serial/Btn/Cnt/Extra/Key_2 lines too).
        let doctor = key(&format!(
            "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
             Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Mazda V2\nBit: 63\nKey: {}\n\
             Serial: 131\nBtn: 8\nCnt: 131\nExtra: 33\nKey_2: 00 00 C1 3B 8C 1C 41 80\n\
             Hdr: 33\nDisp: Mazda V2 - Unlock\n",
            hex(KEY)
        ));
        assert_eq!(dec.from_flipper_key(&doctor), Some(captured(KEY)));
        let mut wrong = doctor.clone();
        wrong.bits = 64;
        assert!(dec.from_flipper_key(&wrong).is_none());
        // An invalid button byte loads (the C does not gate), flagged by crc_valid.
        let mut odd = doctor.clone();
        odd.key = 0x7F21_C13B_8C1C_4180;
        odd.key_bytes = odd.key.to_be_bytes().to_vec();
        assert!(!dec.from_flipper_key(&odd).unwrap().crc_valid);

        // TX from the loaded key: the next code (idx + 1) decodes — the frame doctor-firmware's
        // encoder (C oracle) transmits from the same key: 08 21 C0 34 04 24 41 80.
        let loaded = dec.from_flipper_key(&doctor).unwrap();
        let tx = decode_all(&dec.encode(&loaded, 0x40).unwrap(), false);
        assert_eq!(tx[0].counter, Some(0x84));
        assert_eq!(tx[0].button, Some(0x08));
        assert_eq!(tx[0].data, 0x0821_C034_0424_4180);
    }

    /// Oracle vectors (`KAT_KEY_VECTORS=<dir> cargo test mazda_v2_key_vectors -- --ignored`).
    #[test]
    #[ignore]
    fn mazda_v2_key_vectors() {
        use super::super::bmw::{key_vector_dir, write_key_vector};
        let dec = MazdaV2Decoder::new();
        for (n, key_value) in [(1, KEY), (2, 0x0421_C9B4_5041_715A)] {
            let mut one = Vec::new();
            MazdaV2Decoder::append_upload(&mut one, key_value);
            let decoded = decode_all(&one, false).remove(0);
            let key = dec.to_flipper_key(&decoded).unwrap();
            write_key_vector(&key_vector_dir(), &format!("mazda_v2_{n}"), 433_920_000, "FuriHalSubGhzPresetOok650Async", &one, &key);
        }
    }
}
