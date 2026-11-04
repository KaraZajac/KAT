//! Fiat V1 protocol decoder/encoder (Magneti Marelli BCM / PCF7946, HITAG2 authenticator).
//!
//! Aligned with the CURRENT ProtoPirate reference `REFERENCES/ProtoPirate/protocols/fiat_v1.c`
//! (rewritten upstream in 62b31bf "Fix Fiat V1 decoder and add HITAG2 key TX support" and
//! 1287d10 "fix Fiat V1 encoder"; later commits only drop `furi_check`s). Original C
//! implementation by @lupettohf. Found on Fiat Panda, Grande Punto and other Fiat/Lancia/Alfa BCM
//! keys (~2003-2012). The other references carry the same decoder and add two capabilities that
//! KAT takes from them (union of capabilities):
//! - Flipper-ARF `lib/subghz/protocols/fiat_v1.c` (ProtoPirate's file with ARF glue): with no
//!   verifying key the encoder REPLAYS the captured frame byte-for-byte (ProtoPirate refuses);
//!   doctor-firmware's `fiat_v1.c` does the same under its "Hitag2 replay only" setting. ARF's
//!   "Hitag2 BF" scene (`applications/main/subghz/helpers/subghz_hitag2_bf.c`) also searches
//!   keys beyond the embedded list: L2 = its flash dictionary (`subghz_hitag2_bf_dict.c`; the L3
//!   SD file it ships, `assets/hitag2`, holds the same keys) and L4 = UID/known-key heuristics.
//!   ARF runs that cascade when the user asks; KAT runs L1-L4 on every transmit (≈1000
//!   authenticators, well under a millisecond in release). ARF's L5 (Hitag2Hell, which needs a
//!   second frame of the same remote and hours of CPU) is not ported (see `docs/fiat_v1.md`).
//! - ARF/doctor-firmware keep a per-UID key database (`hitag2_key_db.c`) and doctor-firmware a
//!   typed-in "Hitag2 Key"; KAT has no key store or key entry, so those sources are not ported.
//!
//! RF: 315 / 433.92 MHz, AM (OOK), Manchester. Two TE variants are decoded in parallel:
//!   Variant A (e.g. Panda):        te 250/500 µs ±100 µs
//!   Variant B (e.g. Grande Punto): te 100/200 µs ±50 µs
//!
//! ## Decoder (`fiat_v1_feed_data_pulse` / `fiat_v1_try_decode_window`)
//! No preamble/sync detection: every pulse is split into half-bit *cells* per variant (short = 1
//! cell, long = 2 cells, carrying the pulse level) and appended to a sliding 208-cell window. A
//! non-matching pulse clears that variant's window; a LOW ≥ 800 µs first pushes one LOW cell (the
//! last half-bit merged into the inter-frame gap). Whenever a window holds 208 cells it is read as
//! 104 Manchester bits (cell pair `(first, second)` must differ; bit = `first`, then retried with
//! the bit inverted) and the 13-byte frame is validated.
//!
//! ## Frame (104 wire bits = 13 bytes, MSB first; the C's "102 logical bits" omit the 2 tail bits)
//!   raw[0..2]   0x00 0x01 (sync/family)
//!   raw[2..6]   UID (32 bits)
//!   raw[6]      button(4) ‖ control[9:6]
//!   raw[7]      control[5:0] ‖ hop[31:30]
//!   raw[8..11]  hop[29:6]
//!   raw[11]     hop[5:0] ‖ tail(2)
//!   raw[12]     XOR = 0x01 ^ raw[0] ^ … ^ raw[11]
//! Buttons (C names): 0x1 Close, 0x2 Trunk, 0x4 Lock, 0x8 Unlock. `control` is a 10-bit counter,
//! `hop` the 32-bit HITAG2 authenticator (see `hitag2::fiat_bcm_authenticator`).
//!
//! Gating (`fiat_v1_frame_valid`): raw[0..2] == 00 01, XOR byte, button ∈ {1,2,4,8}, UID not
//! 0x00000000 / 0xFFFFFFFF. Duplicate suppression (`last_raw_data`) is kept but cleared by
//! `reset()`, exactly as in the C reset (so in KAT's registry, which resets every decoder after a
//! hit, each repeat is reported).
//!
//! ## HITAG2 key resolution ([`FiatV1Decoder::resolve_key`], run by `encode()`)
//! 1. ProtoPirate `fiat_v1_resolve_hitag2_key` (no file): the 9 embedded keys, epoch 0, for each
//!    key the frame's button first, then any of 1/2/4/8 (ARF's L1 is the first 8 of these).
//! 2. ARF L2: the 95-entry flash dictionary in order, epoch 0, the frame's own button/control
//!    (`subghz_hitag2_bf_verify_cap_v1`).
//! 3. ARF L4 (`subghz_hitag2_bf_run_l4`): (a) key = (UID ^ mask) ‖ tail for 12 masks × 20 tails,
//!    (b) last byte of each of ARF's 8 known keys ±1..±16, (c) those 8 keys with epoch 1..7.
//!
//! The first match is ProtoPirate's "KEY:OK" / ARF's "found"; a hop matches a wrong key by
//! chance with probability ≈ 2^-32 per candidate (≈ 2^-20 per frame over the whole list).
//!
//! ## DecodedSignal packing
//!   serial = UID, button = Fiat button code (1/2/4/8, not KAT's generic code), counter = control
//!   (10 bits), crc_valid = XOR ok (always true on emission), data_count_bit = 104,
//!   data  = wire bits 0..63 (raw[0..8], big-endian; top 16 bits are the 0x0001 sync),
//!   extra = wire bits 64..103 right-aligned (raw[8..13]) | TE variant << 40 (0 = A, 1 = B).
//!   encoder_capable = always true: a resolved key gives fresh frames, otherwise the ARF replay.
//!
//! ## Encoder (`subghz_protocol_encoder_fiat_v1_deserialize` + emulate plugin)
//! - **Key resolved** (ProtoPirate): a FRESH frame — requested button (KAT Lock/Unlock/Trunk →
//!   Fiat 0x4/0x8/0x2; Panic/unknown keep the captured button), control = captured + 1 (the
//!   emulate plugin's `current_counter++`, masked to 10 bits), hop =
//!   fiat_bcm_authenticator(uid, btn, control, key, epoch), captured tail bits.
//! - **No key** (ARF `TX(replay)`; doctor-firmware with "Hitag2 replay only"): the captured 13-byte
//!   frame byte-for-byte, any requested button. ARF always emits variant A timing for it;
//!   doctor-firmware (and KAT) use the captured TE variant (identical for variant-A captures).
//!
//! Upload per frame: lead HIGH (A 2033 / B 850 µs), 104 Manchester bits at te_short (bit → `bit`
//! level then `!bit` level), LOW gap (A 3252 / B 1230 µs), adjacent levels merged; repeated 6×
//! (`FIAT_V1_ENC_DEFAULT_REPEAT`, all three references).
//!
//! Deviations from C: none in accepted frames. Only the first frame per pulse is returned (the C
//! can fire one callback per variant on the same pulse); the window is a ring buffer instead of a
//! `memmove`, and the window read bails out once the 16-bit 0x0001 prefix mismatches (the C
//! rejects the same windows in `fiat_v1_frame_valid`).
//!
//! ## Key files (`subghz_protocol_decoder_fiat_v1_serialize` / `_deserialize`)
//! KAT writes ProtoPirate's lines: Protocol "Fiat V1", Bit 102 (the C's logical bits), Key =
//! UID << 32 | hop, then Serial (UID), Btn, Cnt (control), Type (TE variant: 0 = A, 1 = B), Hop,
//! XOR, Tail Bits (decimal), Raw (the 13 wire bytes), `Hitag2 Key` + `Hitag2 Epoch` when one of
//! ProtoPirate's embedded keys verifies the hop (the C decoder's own check, epoch 0), and
//! `Disp: Fiat V1 - Unlock|Lock|Trunk|Close|Unknown`. Loading needs Bit 102; with Raw (any
//! reference's file) the frame is Raw and must pass `fiat_v1_frame_valid`; without Raw it is
//! rebuilt from Key / Serial / Btn / Cnt / Hop / XOR / Tail Bits (default 2) and validated, as the
//! C code intends — ProtoPirate's own decoder reads those lines after the failed Raw lookup
//! without rewinding, so in practice it needs Raw; Flipper-ARF's encoder rebuilds from them.
//! Type missing (Flipper-ARF's files) = variant B, ProtoPirate's default. A `Hitag2 Key` in the
//! file is not kept: KAT has no key store (see above), so TX resolves the key from its own lists
//! and otherwise replays the frame.

use super::hitag2;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Fiat V1",
    decoder: "Fiat V1",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "HITAG2",
    make: "Fiat",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// Fiat V1 buttons.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x4 => "Lock",
        0x8 => "Unlock",
        0x2 => "Trunk",
        0x1 => "Close",
        _ => "Other",
    })
}

const WIRE_BITS: usize = 104;
const WIRE_BYTES: usize = 13;
const WIRE_CELLS: usize = WIRE_BITS * 2;
/// `FIAT_V1_LOGICAL_BITS`: the key file's Bit (the 104 wire bits without the 2 tail bits).
const LOGICAL_BITS: usize = 102;
/// `FIAT_V1_DEFAULT_TAIL_BITS`: a key file without Raw / Tail Bits.
const DEFAULT_TAIL_BITS: u8 = 2;
const VARIANT_COUNT: usize = 2;
/// LOW pulse at least this long closes a frame (pushes one LOW boundary cell).
const BOUNDARY_MIN_US: u32 = 800;

const VARIANT_A: u8 = 0;
const VARIANT_B: u8 = 1;

/// (te_short, te_long, te_delta) per variant: A = 250/500 ±100, B = 100/200 ±50.
const VARIANT_TIMING: [(u32, u32, u32); VARIANT_COUNT] = [(250, 500, 100), (100, 200, 50)];

/// Encoder lead HIGH and trailing LOW gap per variant.
const ENC_LEAD_US: [u32; VARIANT_COUNT] = [2033, 850];
const ENC_GAP_US: [u32; VARIANT_COUNT] = [3252, 1230];
const ENC_REPEAT: usize = 6;

/// Bit position of the TE variant in `DecodedSignal::extra` (below it: wire bits 64..103).
const EXTRA_VARIANT_SHIFT: u32 = 40;
const EXTRA_TAIL_MASK: u64 = (1 << EXTRA_VARIANT_SHIFT) - 1;

/// ARF's embedded key list is the first 8 of [`KNOWN_KEYS`] (the base of its L4 mutations).
const ARF_KNOWN_KEY_COUNT: usize = 8;

/// Flipper-ARF Hitag2 BF L2: `subghz_hitag2_bf_flash_dict` (verbatim order, incl. its duplicate
/// "MIKRON" entry; 87 keys beyond [`KNOWN_KEYS`]).
const ARF_DICT_KEYS: [[u8; 6]; 95] = [
    [0x4D, 0x49, 0x4B, 0x52, 0x4F, 0x4E],
    [0x4D, 0x49, 0x4B, 0x52, 0x4F, 0x4D],
    [0x4D, 0x69, 0x6B, 0x72, 0x6F, 0x6E],
    [0x6D, 0x69, 0x6B, 0x72, 0x6F, 0x6E],
    [0x44, 0x45, 0x4C, 0x50, 0x48, 0x49],
    [0x44, 0x65, 0x6C, 0x70, 0x68, 0x69],
    [0x4D, 0x41, 0x52, 0x45, 0x4C, 0x4C],
    [0x4D, 0x61, 0x72, 0x65, 0x6C, 0x6C],
    [0x4D, 0x41, 0x52, 0x45, 0x4C, 0x49],
    [0x43, 0x4F, 0x4E, 0x54, 0x49, 0x4E],
    [0x53, 0x49, 0x45, 0x4D, 0x45, 0x4E],
    [0x42, 0x4F, 0x53, 0x43, 0x48, 0x21],
    [0x56, 0x44, 0x4F, 0x43, 0x41, 0x52],
    [0x46, 0x49, 0x41, 0x54, 0x30, 0x30],
    [0x46, 0x69, 0x61, 0x74, 0x56, 0x31],
    [0x41, 0x4C, 0x46, 0x41, 0x52, 0x4F],
    [0x4C, 0x41, 0x4E, 0x43, 0x49, 0x41],
    [0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
    [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF],
    [0x00, 0x00, 0x00, 0x00, 0x00, 0x01],
    [0x01, 0x02, 0x03, 0x04, 0x05, 0x06],
    [0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA],
    [0x55, 0x55, 0x55, 0x55, 0x55, 0x55],
    [0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE],
    [0xCA, 0xFE, 0xBA, 0xBE, 0xDE, 0xAD],
    [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC],
    [0x4D, 0x49, 0x4B, 0x52, 0x4F, 0x4E],
    [0xBD, 0xF5, 0xE8, 0x46, 0xC3, 0x99],
    [0x9A, 0xC4, 0x99, 0x9C, 0x1E, 0x1C],
    [0x40, 0x00, 0x00, 0x00, 0x00, 0x00],
    [0x21, 0x8F, 0x24, 0x0F, 0x59, 0xC3],
    [0x53, 0x03, 0x64, 0x3F, 0xFB, 0xE3],
    [0x8A, 0x8B, 0x94, 0x00, 0x00, 0x00],
    [0xE7, 0x81, 0xA3, 0xD5, 0xBE, 0xF3],
    [0xAD, 0x2E, 0xCE, 0x5F, 0x6A, 0x1F],
    [0xCD, 0x27, 0x40, 0x89, 0x81, 0xF4],
    [0xB7, 0x92, 0x80, 0xAE, 0xCC, 0x37],
    [0x37, 0xCC, 0xAE, 0x80, 0x92, 0xB7],
    [0xB7, 0x92, 0x80, 0xAE, 0xCC, 0x38],
    [0xB7, 0x92, 0x80, 0xAE, 0xCC, 0x36],
    [0xB8, 0x92, 0x80, 0xAE, 0xCC, 0x37],
    [0xD4, 0x24, 0x28, 0xF7, 0xD9, 0x66],
    [0x66, 0xD9, 0xF7, 0x28, 0x24, 0xD4],
    [0xD4, 0x24, 0x28, 0xF7, 0xD9, 0x67],
    [0xD4, 0x24, 0x28, 0xF7, 0xD9, 0x65],
    [0x4D, 0x34, 0x3F, 0xD4, 0xE7, 0xB6],
    [0xB6, 0xE7, 0xD4, 0x3F, 0x34, 0x4D],
    [0x4D, 0x34, 0x3F, 0xD4, 0xE7, 0xB7],
    [0x6D, 0x6B, 0xF2, 0x1D, 0x3A, 0x1A],
    [0x1A, 0x3A, 0x1D, 0xF2, 0x6B, 0x6D],
    [0x6D, 0x6B, 0xF2, 0x1D, 0x3A, 0x1B],
    [0xA3, 0xF3, 0xAC, 0xF7, 0xB9, 0x10],
    [0x10, 0xB9, 0xF7, 0xAC, 0xF3, 0xA3],
    [0xA3, 0xF3, 0xAC, 0xF7, 0xB9, 0x11],
    [0x33, 0xFA, 0x2F, 0xCD, 0xC3, 0x3B],
    [0x3B, 0xC3, 0xCD, 0x2F, 0xFA, 0x33],
    [0x33, 0xFA, 0x2F, 0xCD, 0xC3, 0x3A],
    [0x00, 0x00, 0x00, 0x00, 0x00, 0x02],
    [0x00, 0x00, 0x00, 0x00, 0x00, 0x03],
    [0x00, 0x00, 0x00, 0x00, 0x00, 0x04],
    [0x00, 0x00, 0x00, 0x00, 0x00, 0x08],
    [0x00, 0x00, 0x00, 0x00, 0x00, 0x10],
    [0x00, 0x00, 0x00, 0x00, 0x01, 0x00],
    [0x00, 0x00, 0x00, 0x01, 0x00, 0x00],
    [0x00, 0x00, 0x01, 0x00, 0x00, 0x00],
    [0x00, 0x01, 0x00, 0x00, 0x00, 0x00],
    [0x01, 0x00, 0x00, 0x00, 0x00, 0x00],
    [0x00, 0x00, 0x00, 0x00, 0x00, 0x99],
    [0x00, 0x00, 0x00, 0x00, 0x99, 0x99],
    [0x00, 0x00, 0x00, 0x99, 0x99, 0x99],
    [0x99, 0x99, 0x99, 0x99, 0x99, 0x99],
    [0x00, 0x00, 0x00, 0x00, 0x00, 0x11],
    [0x11, 0x11, 0x11, 0x11, 0x11, 0x11],
    [0x00, 0x00, 0x00, 0x00, 0x00, 0x22],
    [0x22, 0x22, 0x22, 0x22, 0x22, 0x22],
    [0x00, 0x00, 0x00, 0x00, 0x00, 0x33],
    [0x33, 0x33, 0x33, 0x33, 0x33, 0x33],
    [0x00, 0x00, 0x00, 0x00, 0x00, 0x44],
    [0x44, 0x44, 0x44, 0x44, 0x44, 0x44],
    [0x00, 0x00, 0x00, 0x00, 0x00, 0x77],
    [0x77, 0x77, 0x77, 0x77, 0x77, 0x77],
    [0x00, 0x00, 0x00, 0x00, 0x00, 0x88],
    [0x88, 0x88, 0x88, 0x88, 0x88, 0x88],
    [0x00, 0x00, 0x00, 0x00, 0x00, 0xEE],
    [0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE],
    [0x00, 0x01, 0x02, 0x03, 0x04, 0x05],
    [0x10, 0x20, 0x30, 0x40, 0x50, 0x60],
    [0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45],
    [0x12, 0x34, 0x56, 0x78, 0x90, 0xAB],
    [0x8D, 0xCA, 0x22, 0x11, 0x00, 0xFE],
    [0x7B, 0x3F, 0x91, 0x2E, 0xC5, 0x08],
    [0xF1, 0xE2, 0xD3, 0xC4, 0xB5, 0xA6],
    [0x1F, 0x2E, 0x3D, 0x4C, 0x5B, 0x6A],
    [0xE3, 0xD1, 0xB4, 0x7C, 0x8F, 0x25],
    [0x9C, 0x4A, 0x71, 0xE8, 0x03, 0x56],
];

/// ARF L4 strategy 1 (`subghz_hitag2_bf_run_l4`): key[0..4] = UID ^ mask, key[4..6] = tail.
const ARF_L4_XOR_MASKS: [u32; 12] = [
    0x00000000, 0xFFFFFFFF, 0xA5A5A5A5, 0x5A5A5A5A, 0x12345678, 0xDEADBEEF, 0xCAFEBABE, 0x1F2E3D4C,
    0x87654321, 0xF0F0F0F0, 0x0F0F0F0F, 0xC3C3C3C3,
];
const ARF_L4_TAILS: [[u8; 2]; 20] = [
    [0x00, 0x00],
    [0xFF, 0xFF],
    [0x00, 0xFF],
    [0xFF, 0x00],
    [0xAA, 0x55],
    [0x55, 0xAA],
    [0x12, 0x34],
    [0xAB, 0xCD],
    [0xDE, 0xAD],
    [0xBE, 0xEF],
    [0xCA, 0xFE],
    [0xBA, 0xBE],
    [0x00, 0x01],
    [0x00, 0x02],
    [0x00, 0x08],
    [0x00, 0x10],
    [0x01, 0x00],
    [0x10, 0x00],
    [0x00, 0x99],
    [0x99, 0x00],
];

const BUTTON_CLOSE: u8 = 0x1;
const BUTTON_TRUNK: u8 = 0x2;
const BUTTON_LOCK: u8 = 0x4;
const BUTTON_UNLOCK: u8 = 0x8;
const BUTTONS: [u8; 4] = [BUTTON_CLOSE, BUTTON_TRUNK, BUTTON_LOCK, BUTTON_UNLOCK];

/// `fiat_v1_known_keys` — embedded HITAG2 keys (epoch 0).
const KNOWN_KEYS: [[u8; 6]; 9] = [
    [0xB7, 0x92, 0x80, 0xAE, 0xCC, 0x37],
    [0xD4, 0x24, 0x28, 0xF7, 0xD9, 0x66],
    [0x4D, 0x34, 0x3F, 0xD4, 0xE7, 0xB6],
    [0x6D, 0x6B, 0xF2, 0x1D, 0x3A, 0x1A],
    [0xA3, 0xF3, 0xAC, 0xF7, 0xB9, 0x10],
    [0x4D, 0x49, 0x4B, 0x52, 0x4F, 0x4E],
    [0xCD, 0x49, 0x4B, 0x52, 0x4F, 0x4E],
    [0x33, 0xFA, 0x2F, 0xCD, 0xC3, 0x3B],
    [0xF6, 0x1A, 0xEF, 0x9C, 0xD0, 0x1B],
];

#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Data,
}

/// Sliding window of the last `WIRE_CELLS` half-bit cells (ring buffer; `start` = oldest cell
/// once full).
#[derive(Clone)]
struct CellWindow {
    cells: [bool; WIRE_CELLS],
    count: usize,
    start: usize,
}

impl CellWindow {
    fn new() -> Self {
        Self {
            cells: [false; WIRE_CELLS],
            count: 0,
            start: 0,
        }
    }

    fn clear(&mut self) {
        self.count = 0;
        self.start = 0;
    }

    fn push(&mut self, level: bool) {
        if self.count < WIRE_CELLS {
            self.cells[self.count] = level;
            self.count += 1;
        } else {
            self.cells[self.start] = level;
            self.start = (self.start + 1) % WIRE_CELLS;
        }
    }

    fn cell(&self, index: usize) -> bool {
        self.cells[(self.start + index) % WIRE_CELLS]
    }

    /// `fiat_v1_try_decode_window` bit extraction: 104 cell pairs → 13 bytes, or `None` when a
    /// pair is not a Manchester transition (or the 0x0001 prefix already mismatches).
    fn read_frame(&self, invert: bool) -> Option<[u8; WIRE_BYTES]> {
        if self.count != WIRE_CELLS {
            return None;
        }
        let mut raw = [0u8; WIRE_BYTES];
        for bit_index in 0..WIRE_BITS {
            let first = self.cell(bit_index * 2);
            let second = self.cell(bit_index * 2 + 1);
            if first == second {
                return None;
            }
            if first != invert {
                raw[bit_index >> 3] |= 1 << (7 - (bit_index & 7));
            }
            if bit_index == 15 && (raw[0] != 0x00 || raw[1] != 0x01) {
                return None;
            }
        }
        Some(raw)
    }
}

/// Fields of a 13-byte Fiat V1 frame.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Fields {
    uid: u32,
    button: u8,
    control: u16,
    hop: u32,
    tail: u8,
}

pub struct FiatV1Decoder {
    step: DecoderStep,
    windows: [CellWindow; VARIANT_COUNT],
    last_raw: Option<[u8; WIRE_BYTES]>,
}

impl FiatV1Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            windows: [CellWindow::new(), CellWindow::new()],
            last_raw: None,
        }
    }

    fn is_short(variant: usize, duration: u32) -> bool {
        let (te_short, _, te_delta) = VARIANT_TIMING[variant];
        duration_diff!(duration, te_short) < te_delta
    }

    fn is_long(variant: usize, duration: u32) -> bool {
        let (_, te_long, te_delta) = VARIANT_TIMING[variant];
        duration_diff!(duration, te_long) < te_delta
    }

    /// `fiat_v1_duration_is_pulse`: short or long for any variant.
    fn is_pulse(duration: u32) -> bool {
        (0..VARIANT_COUNT).any(|v| Self::is_short(v, duration) || Self::is_long(v, duration))
    }

    fn button_valid(button: u8) -> bool {
        BUTTONS.contains(&button)
    }

    /// `fiat_v1_frame_xor`: 0x01 ^ raw[0] ^ … ^ raw[11].
    fn frame_xor(raw: &[u8; WIRE_BYTES]) -> u8 {
        raw[..WIRE_BYTES - 1].iter().fold(0x01, |acc, b| acc ^ b)
    }

    fn fields(raw: &[u8; WIRE_BYTES]) -> Fields {
        Fields {
            uid: u32::from_be_bytes([raw[2], raw[3], raw[4], raw[5]]),
            button: raw[6] >> 4,
            control: (((raw[6] & 0x0F) as u16) << 6) | (raw[7] >> 2) as u16,
            hop: (((raw[7] & 0x03) as u32) << 30)
                | ((raw[8] as u32) << 22)
                | ((raw[9] as u32) << 14)
                | ((raw[10] as u32) << 6)
                | (raw[11] >> 2) as u32,
            tail: raw[11] & 0x03,
        }
    }

    /// `fiat_v1_frame_valid`.
    fn frame_valid(raw: &[u8; WIRE_BYTES]) -> bool {
        if raw[0] != 0x00 || raw[1] != 0x01 {
            return false;
        }
        if Self::frame_xor(raw) != raw[12] {
            return false;
        }
        if !Self::button_valid(raw[6] >> 4) {
            return false;
        }
        let uid = Self::fields(raw).uid;
        uid != 0 && uid != u32::MAX
    }

    /// `fiat_v1_build_raw`.
    fn build_raw(uid: u32, button: u8, control: u16, hop: u32, tail: u8) -> [u8; WIRE_BYTES] {
        let control = control as u32;
        let mut raw = [0u8; WIRE_BYTES];
        raw[1] = 0x01;
        raw[2..6].copy_from_slice(&uid.to_be_bytes());
        raw[6] = ((button & 0x0F) << 4) | ((control >> 6) & 0x0F) as u8;
        raw[7] = (((control & 0x3F) << 2) as u8) | ((hop >> 30) & 0x03) as u8;
        raw[8] = (hop >> 22) as u8;
        raw[9] = (hop >> 14) as u8;
        raw[10] = (hop >> 6) as u8;
        raw[11] = ((hop << 2) as u8) | (tail & 0x03);
        raw[12] = Self::frame_xor(&raw);
        raw
    }

    /// `fiat_v1_resolve_hitag2_key` without a file: index of the first embedded key whose
    /// authenticator (epoch 0) equals the hop for the frame's button or any of 1/2/4/8.
    fn resolve_known_key(f: &Fields) -> Option<usize> {
        let matches = |key: &[u8; 6], button: u8| {
            hitag2::fiat_bcm_authenticator(f.uid, button, f.control & 0x03FF, key, 0) == f.hop
        };
        KNOWN_KEYS
            .iter()
            .position(|key| matches(key, f.button) || BUTTONS.iter().any(|&b| matches(key, b)))
    }

    /// ARF L4 candidates (`subghz_hitag2_bf_run_l4`, in its order): (key, epoch).
    fn arf_heuristic_candidates(uid: u32) -> impl Iterator<Item = ([u8; 6], u32)> {
        let uid_derived = ARF_L4_XOR_MASKS.into_iter().flat_map(move |mask| {
            ARF_L4_TAILS.into_iter().map(move |tail| {
                let p = (uid ^ mask).to_be_bytes();
                ([p[0], p[1], p[2], p[3], tail[0], tail[1]], 0)
            })
        });
        let last_byte = (0..ARF_KNOWN_KEY_COUNT).flat_map(|k| {
            (-16i32..=16).filter(|&d| d != 0).map(move |d| {
                let mut key = KNOWN_KEYS[k];
                key[5] = key[5].wrapping_add(d as u8);
                (key, 0)
            })
        });
        let epochs = (0..ARF_KNOWN_KEY_COUNT)
            .flat_map(|k| (1u32..=7).map(move |epoch| (KNOWN_KEYS[k], epoch)));
        uid_derived.chain(last_byte).chain(epochs)
    }

    /// The HITAG2 key (and epoch) for a frame: ProtoPirate's embedded keys, then ARF's L2
    /// dictionary and L4 heuristics (see module doc). `None` = the C's "KEY:??".
    fn resolve_key(f: &Fields) -> Option<([u8; 6], u32)> {
        if let Some(index) = Self::resolve_known_key(f) {
            return Some((KNOWN_KEYS[index], 0));
        }
        // subghz_hitag2_bf_verify_cap_v1: the capture's own button and control.
        let verifies = |key: &[u8; 6], epoch: u32| {
            hitag2::fiat_bcm_authenticator(f.uid, f.button, f.control & 0x03FF, key, epoch) == f.hop
        };
        ARF_DICT_KEYS
            .iter()
            .map(|key| (*key, 0))
            .chain(Self::arf_heuristic_candidates(f.uid))
            .find(|(key, epoch)| verifies(key, *epoch))
    }

    /// Build the DecodedSignal for a validated frame (`fiat_v1_decode_fields`).
    fn decoded_signal(raw: &[u8; WIRE_BYTES], variant: u8) -> DecodedSignal {
        let f = Self::fields(raw);
        let data = u64::from_be_bytes([
            raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], raw[6], raw[7],
        ]);
        let tail40 = raw[8..13]
            .iter()
            .fold(0u64, |acc, &b| (acc << 8) | b as u64);
        DecodedSignal {
            serial: Some(f.uid),
            button: Some(f.button),
            counter: Some(f.control),
            crc_valid: true,
            data,
            data_count_bit: WIRE_BITS,
            // A resolved key gives fresh frames; otherwise ARF's byte-identical replay.
            encoder_capable: true,
            extra: Some(tail40 | ((variant as u64) << EXTRA_VARIANT_SHIFT)),
            protocol_display_name: None,
        }
    }

    /// Rebuild the 13-byte wire frame and TE variant from a DecodedSignal (`data` + `extra`).
    fn raw_from_signal(decoded: &DecodedSignal) -> Option<([u8; WIRE_BYTES], u8)> {
        if decoded.data_count_bit != WIRE_BITS {
            return None;
        }
        let extra = decoded.extra?;
        let mut raw = [0u8; WIRE_BYTES];
        raw[..8].copy_from_slice(&decoded.data.to_be_bytes());
        raw[8..].copy_from_slice(&(extra & EXTRA_TAIL_MASK).to_be_bytes()[3..8]);
        let variant = if (extra >> EXTRA_VARIANT_SHIFT) as u8 == VARIANT_A {
            VARIANT_A
        } else {
            VARIANT_B
        };
        Some((raw, variant))
    }

    /// Commit a candidate window (`fiat_v1_commit`). `Ok(Some)` = new frame, `Ok(None)` = valid
    /// duplicate (accepted silently, as in C), `Err(())` = invalid.
    fn commit(&mut self, raw: [u8; WIRE_BYTES], variant: u8) -> Result<Option<DecodedSignal>, ()> {
        if !Self::frame_valid(&raw) {
            return Err(());
        }
        if self.last_raw == Some(raw) {
            return Ok(None);
        }
        self.last_raw = Some(raw);
        Ok(Some(Self::decoded_signal(&raw, variant)))
    }

    /// `fiat_v1_try_decode`: non-inverted window first, inverted only if that was not valid.
    fn try_decode(&mut self, variant: usize) -> Option<DecodedSignal> {
        for invert in [false, true] {
            if let Some(raw) = self.windows[variant].read_frame(invert) {
                if let Ok(result) = self.commit(raw, variant as u8) {
                    return result;
                }
            }
        }
        None
    }

    /// `fiat_v1_push_cell`.
    fn push_cell(&mut self, variant: usize, level: bool) -> Option<DecodedSignal> {
        self.windows[variant].push(level);
        self.try_decode(variant)
    }

    /// `fiat_v1_feed_data_pulse`. Returns (matched any variant, first frame decoded on this pulse).
    fn feed_data_pulse(&mut self, level: bool, duration: u32) -> (bool, Option<DecodedSignal>) {
        let mut matched = false;
        let mut result: Option<DecodedSignal> = None;
        for variant in 0..VARIANT_COUNT {
            let pushes = if Self::is_short(variant, duration) {
                1
            } else if Self::is_long(variant, duration) {
                2
            } else {
                0
            };
            if pushes > 0 {
                matched = true;
                for _ in 0..pushes {
                    let decoded = self.push_cell(variant, level);
                    if result.is_none() {
                        result = decoded;
                    }
                }
            } else {
                if !level && duration >= BOUNDARY_MIN_US {
                    let decoded = self.push_cell(variant, false);
                    if result.is_none() {
                        result = decoded;
                    }
                }
                self.windows[variant].clear();
            }
        }
        (matched, result)
    }

    fn clear_all_cells(&mut self) {
        for window in &mut self.windows {
            window.clear();
        }
    }

    /// KAT generic button → Fiat V1 code (emulate plugin: Up=Lock 0x4, OK=Unlock 0x8,
    /// Down=Trunk 0x2). `None` keeps the captured button.
    fn map_button(button: u8) -> Option<u8> {
        match button {
            0x01 => Some(BUTTON_LOCK),
            0x02 => Some(BUTTON_UNLOCK),
            0x04 => Some(BUTTON_TRUNK),
            _ => None,
        }
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

    /// `fiat_v1_encoder_build_upload` for one frame, appended to `signal`.
    fn append_upload(signal: &mut Vec<LevelDuration>, raw: &[u8; WIRE_BYTES], variant: u8) {
        let v = variant as usize;
        let te = VARIANT_TIMING[v].0;
        Self::enc_add_level(signal, true, ENC_LEAD_US[v]);
        for bit_index in 0..WIRE_BITS {
            let bit = (raw[bit_index >> 3] >> (7 - (bit_index & 7))) & 1 != 0;
            Self::enc_add_level(signal, bit, te);
            Self::enc_add_level(signal, !bit, te);
        }
        Self::enc_add_level(signal, false, ENC_GAP_US[v]);
    }
}

impl ProtocolDecoder for FiatV1Decoder {
    fn name(&self) -> &'static str {
        "Fiat V1"
    }

    fn timing(&self) -> ProtocolTiming {
        let (te_short, te_long, te_delta) = VARIANT_TIMING[VARIANT_A as usize];
        ProtocolTiming {
            te_short,
            te_long,
            te_delta,
            min_count_bit: WIRE_BITS,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.clear_all_cells();
        self.last_raw = None;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::Reset => {
                if Self::is_pulse(duration) {
                    self.clear_all_cells();
                    self.step = DecoderStep::Data;
                    return self.feed_data_pulse(level, duration).1;
                }
                None
            }
            DecoderStep::Data => {
                let (matched, result) = self.feed_data_pulse(level, duration);
                if !matched {
                    self.step = DecoderStep::Reset;
                }
                result
            }
        }
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        let (raw, variant) = Self::raw_from_signal(decoded)?;
        if !Self::frame_valid(&raw) {
            return None;
        }
        let captured = Self::fields(&raw);
        let tx_raw = match Self::resolve_key(&captured) {
            Some((key, epoch)) => {
                let tx_button = Self::map_button(button).unwrap_or(captured.button);
                let tx_control =
                    decoded.counter.unwrap_or(captured.control).wrapping_add(1) & 0x03FF;
                let uid = captured.uid;
                let hop = hitag2::fiat_bcm_authenticator(uid, tx_button, tx_control, &key, epoch);
                Self::build_raw(captured.uid, tx_button, tx_control, hop, captured.tail)
            }
            // ProtoPirate refuses here; ARF replays the captured frame (any button).
            None => raw,
        };

        let mut signal = Vec::with_capacity(ENC_REPEAT * (2 * WIRE_BITS + 2));
        for _ in 0..ENC_REPEAT {
            Self::append_upload(&mut signal, &tx_raw, variant);
        }
        Some(signal)
    }

    /// `subghz_protocol_decoder_fiat_v1_deserialize` (see the module doc, Key files).
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != LOGICAL_BITS {
            return None;
        }
        let variant = if key.u32_field("Type") == Some(VARIANT_A.into()) {
            VARIANT_A
        } else {
            VARIANT_B
        };
        let file_raw = key.hex_bytes_field("Raw").and_then(|b| <[u8; WIRE_BYTES]>::try_from(b).ok());
        let raw = match file_raw {
            Some(raw) => raw,
            None => {
                // fiat_v1_rebuild_raw from the Key and the field lines.
                let data = key.key_u64()?;
                let uid = key.u32_field("Serial").unwrap_or((data >> 32) as u32);
                let hop = key.u32_field("Hop").unwrap_or(data as u32);
                let button = key.u32_field("Btn").unwrap_or(0) as u8;
                let control = (key.u32_field("Cnt").unwrap_or(0) & 0x03FF) as u16;
                let tail = key.u32_field("Tail Bits").map_or(DEFAULT_TAIL_BITS, |t| (t & 0x03) as u8);
                let mut raw = Self::build_raw(uid, button, control, hop, tail);
                if let Some(xor) = key.u32_field("XOR").filter(|&x| x != 0) {
                    raw[12] = xor as u8;
                }
                raw
            }
        };
        Self::frame_valid(&raw).then(|| Self::decoded_signal(&raw, variant))
    }

    /// `subghz_protocol_decoder_fiat_v1_serialize` (see the module doc, Key files).
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        let (raw, variant) = Self::raw_from_signal(decoded)?;
        if !Self::frame_valid(&raw) {
            return None;
        }
        let f = Self::fields(&raw);
        let mut key = FlipperKey::for_protocol("Fiat V1", LOGICAL_BITS, (u64::from(f.uid) << 32) | u64::from(f.hop));
        key.push_u32("Serial", f.uid);
        key.push_u32("Btn", f.button.into());
        key.push_u32("Cnt", f.control.into());
        key.push_u32("Type", variant.into());
        key.push_u32("Hop", f.hop);
        key.push_u32("XOR", raw[12].into());
        key.push_u32("Tail Bits", f.tail.into());
        key.push_hex_bytes("Raw", &raw);
        if let Some(index) = Self::resolve_known_key(&f) {
            key.push_hex_bytes("Hitag2 Key", &KNOWN_KEYS[index]);
            key.push_u32("Hitag2 Epoch", 0);
        }
        let name = match f.button {
            BUTTON_UNLOCK => "Unlock",
            BUTTON_LOCK => "Lock",
            BUTTON_TRUNK => "Trunk",
            BUTTON_CLOSE => "Close",
            _ => "Unknown",
        };
        key.push("Disp", format!("Fiat V1 - {name}"));
        Some(key)
    }
}

impl Default for FiatV1Decoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn decode_all(dec: &mut FiatV1Decoder, signal: &[LevelDuration]) -> Vec<DecodedSignal> {
        signal
            .iter()
            .filter_map(|ld| dec.feed(ld.level, ld.duration_us))
            .collect()
    }

    /// A frame signed with known key `key_index` (epoch 0), as a DecodedSignal of `variant`.
    fn signed_frame(
        uid: u32,
        button: u8,
        control: u16,
        key_index: usize,
        variant: u8,
    ) -> DecodedSignal {
        let hop = hitag2::fiat_bcm_authenticator(uid, button, control, &KNOWN_KEYS[key_index], 0);
        let raw = FiatV1Decoder::build_raw(uid, button, control, hop, 2);
        assert!(FiatV1Decoder::frame_valid(&raw));
        FiatV1Decoder::decoded_signal(&raw, variant)
    }

    /// One upload (lead + frame + gap) for an arbitrary raw frame.
    fn upload(raw: &[u8; WIRE_BYTES], variant: u8) -> Vec<LevelDuration> {
        let mut signal = Vec::new();
        FiatV1Decoder::append_upload(&mut signal, raw, variant);
        signal
    }

    fn pulses(signal: &[LevelDuration]) -> Vec<i32> {
        signal
            .iter()
            .map(|ld| {
                if ld.level {
                    ld.duration_us as i32
                } else {
                    -(ld.duration_us as i32)
                }
            })
            .collect()
    }

    /// build_raw matches `fiat_v1_build_raw` (vector from the C), and the field extraction
    /// inverts it.
    #[test]
    fn build_raw_matches_c_and_round_trips() {
        let raw = FiatV1Decoder::build_raw(0x12345678, 0x4, 0x101, 0x9DAB8C6A, 2);
        assert_eq!(
            raw,
            [0x00, 0x01, 0x12, 0x34, 0x56, 0x78, 0x44, 0x06, 0x76, 0xAE, 0x31, 0xAA, 0x09]
        );
        assert!(FiatV1Decoder::frame_valid(&raw));
        let f = FiatV1Decoder::fields(&raw);
        assert_eq!(
            f,
            Fields {
                uid: 0x12345678,
                button: 0x4,
                control: 0x101,
                hop: 0x9DAB8C6A,
                tail: 2
            }
        );
    }

    /// The upload matches the C `fiat_v1_encoder_build_upload` (pp_emit_merge) exactly for both TE
    /// variants. Vectors produced by compiling fiat_v1.c: hop = known key 0 (B) / key 3 (A).
    #[test]
    fn upload_matches_c_reference() {
        const C_UPLOAD_B: [i32; 162] = [
            850, -100, 100, -100, 100, -100, 100, -100, 100, -100, 100, -100, 100, -100, 100, -100,
            100, -100, 100, -100, 100, -100, 100, -100, 100, -100, 100, -100, 100, -100, 200, -200,
            100, -100, 100, -100, 200, -200, 100, -100, 200, -200, 100, -100, 100, -100, 200, -100,
            100, -200, 200, -200, 100, -100, 100, -100, 200, -200, 200, -200, 200, -100, 100, -200,
            100, -100, 200, -100, 100, -100, 100, -100, 100, -200, 100, -100, 100, -100, 100, -100,
            200, -200, 100, -100, 100, -100, 200, -200, 100, -100, 100, -100, 100, -100, 100, -100,
            100, -100, 100, -100, 200, -100, 100, -200, 100, -100, 200, -100, 100, -100, 100, -200,
            200, -100, 100, -200, 200, -200, 200, -200, 200, -100, 100, -100, 100, -200, 100, -100,
            100, -100, 200, -100, 100, -200, 100, -100, 100, -100, 200, -100, 100, -200, 200, -200,
            200, -200, 200, -200, 100, -100, 100, -100, 100, -100, 100, -100, 200, -200, 100, -100,
            200, -1330,
        ];
        const C_UPLOAD_A: [i32; 168] = [
            2033, -250, 250, -250, 250, -250, 250, -250, 250, -250, 250, -250, 250, -250, 250,
            -250, 250, -250, 250, -250, 250, -250, 250, -250, 250, -250, 250, -250, 250, -250, 500,
            -250, 250, -250, 250, -250, 250, -500, 500, -250, 250, -250, 250, -250, 250, -250, 250,
            -500, 500, -250, 250, -250, 250, -500, 250, -250, 250, -250, 250, -250, 250, -250, 500,
            -500, 500, -250, 250, -500, 500, -500, 500, -250, 250, -500, 250, -250, 500, -250, 250,
            -500, 250, -250, 250, -250, 500, -500, 250, -250, 250, -250, 500, -500, 250, -250, 250,
            -250, 500, -250, 250, -250, 250, -500, 250, -250, 500, -500, 250, -250, 500, -500, 500,
            -250, 250, -500, 250, -250, 250, -250, 500, -500, 250, -250, 250, -250, 250, -250, 500,
            -500, 250, -250, 250, -250, 250, -250, 500, -500, 250, -250, 500, -250, 250, -500, 250,
            -250, 250, -250, 500, -250, 250, -250, 250, -250, 250, -500, 250, -250, 500, -250, 250,
            -500, 250, -250, 500, -500, 250, -250, 250, -3252,
        ];
        let raw_b = FiatV1Decoder::build_raw(0x12345678, 0x4, 0x101, 0x9DAB8C6A, 2);
        assert_eq!(
            0x9DAB8C6A,
            hitag2::fiat_bcm_authenticator(0x12345678, 0x4, 0x101, &KNOWN_KEYS[0], 0)
        );
        assert_eq!(pulses(&upload(&raw_b, VARIANT_B)), C_UPLOAD_B.to_vec());

        let raw_a = FiatV1Decoder::build_raw(0xEFB82D66, 0x2, 0x08E, 0x4B1084C7, 2);
        assert_eq!(
            raw_a,
            [0x00, 0x01, 0xEF, 0xB8, 0x2D, 0x66, 0x22, 0x39, 0x2C, 0x42, 0x13, 0x1E, 0x64]
        );
        assert_eq!(
            0x4B1084C7,
            hitag2::fiat_bcm_authenticator(0xEFB82D66, 0x2, 0x08E, &KNOWN_KEYS[3], 0)
        );
        assert_eq!(pulses(&upload(&raw_a, VARIANT_A)), C_UPLOAD_A.to_vec());
    }

    /// Key files: ProtoPirate's lines (Raw, the verifying embedded key) round-trip for both
    /// variants; a Flipper-ARF file (no Type: variant B) and a file without Raw load; a loaded key
    /// transmits a fresh frame that decodes back.
    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::{assert_round_trip, key};
        let dec = FiatV1Decoder::new();
        let raw_a = FiatV1Decoder::build_raw(0xEFB82D66, BUTTON_TRUNK, 0x08E, 0x4B1084C7, 2);
        let mut stream = vec![LevelDuration::new(false, 20_000)];
        stream.extend(upload(&raw_a, VARIANT_A));
        let decoded = decode_all(&mut FiatV1Decoder::new(), &stream).remove(0);
        let text = assert_round_trip(&dec, &decoded);
        assert!(
            text.ends_with(
                "Protocol: Fiat V1\nBit: 102\nKey: EF B8 2D 66 4B 10 84 C7\nSerial: 4021824870\n\
                 Btn: 2\nCnt: 142\nType: 0\nHop: 1259373767\nXOR: 100\nTail Bits: 2\n\
                 Raw: 00 01 EF B8 2D 66 22 39 2C 42 13 1E 64\nHitag2 Key: 6D 6B F2 1D 3A 1A\n\
                 Hitag2 Epoch: 0\nDisp: Fiat V1 - Trunk\n"
            ),
            "{text}"
        );
        // Variant B, a frame no embedded key verifies (no Hitag2 lines), another button.
        let raw_b = FiatV1Decoder::build_raw(0x12345678, BUTTON_CLOSE, 0x3FF, 0xDEADBEEF, 1);
        let b = FiatV1Decoder::decoded_signal(&raw_b, VARIANT_B);
        let text = assert_round_trip(&dec, &b);
        assert!(text.contains("\nType: 1\n") && !text.contains("Hitag2") && text.ends_with("Disp: Fiat V1 - Close\n"), "{text}");

        // Flipper-ARF's serialize: Raw first, no Type / Disp → ProtoPirate's default variant B.
        let arf = key(
            "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
             Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Fiat V1\nBit: 102\n\
             Key: EF B8 2D 66 4B 10 84 C7\nRaw: 00 01 EF B8 2D 66 22 39 2C 42 13 1E 64\n\
             Hop: 1259373767\nXOR: 100\nTail Bits: 2\nSerial: 4021824870\nBtn: 2\nCnt: 142\n\
             Hitag2 Key: 6D 6B F2 1D 3A 1A\nHitag2 Epoch: 0\n",
        );
        assert_eq!(dec.from_flipper_key(&arf), Some(FiatV1Decoder::decoded_signal(&raw_a, VARIANT_B)));
        // No Raw: rebuilt from the field lines.
        let mut no_raw = dec.to_flipper_key(&decoded).unwrap();
        no_raw.fields.retain(|(n, _)| n != "Raw");
        assert_eq!(dec.from_flipper_key(&no_raw), Some(decoded.clone()));
        // Bad frames and other Bit counts are rejected, as in the C.
        let mut bad = dec.to_flipper_key(&decoded).unwrap();
        bad.fields.retain(|(n, _)| n != "Raw");
        bad.push("Raw", "00 01 EF B8 2D 66 22 39 2C 42 13 1E 65");
        assert!(dec.from_flipper_key(&bad).is_none(), "XOR mismatch");
        let mut bits = dec.to_flipper_key(&decoded).unwrap();
        bits.bits = 104;
        assert!(dec.from_flipper_key(&bits).is_none());

        // TX from the loaded key: embedded key 3 signs a fresh Lock frame (control + 1).
        let loaded = dec.from_flipper_key(&key(&text_of(&dec, &decoded))).unwrap();
        let tx = decode_all(&mut FiatV1Decoder::new(), &dec.encode(&loaded, 0x01).unwrap()).remove(0);
        assert_eq!((tx.serial, tx.button, tx.counter), (Some(0xEFB82D66), Some(BUTTON_LOCK), Some(0x08F)));
    }

    fn text_of(dec: &FiatV1Decoder, d: &DecodedSignal) -> String {
        let mut k = dec.to_flipper_key(d).unwrap();
        k.frequency = 433_920_000;
        k.to_text()
    }

    /// Oracle vectors (see `fiat_v0::write_oracle_vector`): a variant-A frame signed with
    /// embedded key 3, and a variant-B frame no key verifies (replayed).
    #[test]
    #[ignore]
    fn oracle_vectors() {
        let signed = FiatV1Decoder::build_raw(0xEFB82D66, BUTTON_TRUNK, 0x08E, 0x4B1084C7, 2);
        let plain = FiatV1Decoder::build_raw(0x12345678, BUTTON_CLOSE, 0x3FF, 0xDEADBEEF, 1);
        for (name, raw, variant) in [("fiat_v1_1", signed, VARIANT_A), ("fiat_v1_2", plain, VARIANT_B)] {
            let up = upload(&raw, variant);
            crate::protocols::fiat_v0::write_oracle_vector(
                name,
                433_920_000,
                "FuriHalSubGhzPresetOok650Async",
                &up,
                &mut FiatV1Decoder::new(),
            );
        }
    }

    /// Synthetic frames of both variants decode with the right fields, variant and key status.
    #[test]
    fn synthetic_frames_decode_both_variants() {
        for (variant, key_index) in [(VARIANT_A, 3usize), (VARIANT_B, 0usize)] {
            let sig = signed_frame(0xCAFE1234, BUTTON_UNLOCK, 0x2A5, key_index, variant);
            let (raw, v) = FiatV1Decoder::raw_from_signal(&sig).unwrap();
            assert_eq!(v, variant);
            let mut dec = FiatV1Decoder::new();
            let mut stream = vec![LevelDuration::new(false, 20_000)];
            stream.extend(upload(&raw, variant));
            let out = decode_all(&mut dec, &stream);
            assert_eq!(out.len(), 1, "variant {variant}: exactly one frame");
            let d = &out[0];
            assert_eq!(d.serial, Some(0xCAFE1234));
            assert_eq!(d.button, Some(BUTTON_UNLOCK));
            assert_eq!(d.counter, Some(0x2A5));
            assert_eq!(d.data_count_bit, 104);
            assert_eq!(d.data, sig.data);
            assert_eq!(d.extra, sig.extra);
            assert!(d.crc_valid && d.encoder_capable, "known key must verify");
        }
    }

    /// Inverted-polarity capture of the same frame is recovered by the inverted window read.
    #[test]
    fn inverted_window_decodes() {
        let sig = signed_frame(0x0BADF00D, BUTTON_LOCK, 0x011, 5, VARIANT_A);
        let (raw, _) = FiatV1Decoder::raw_from_signal(&sig).unwrap();
        let inverted: Vec<LevelDuration> = upload(&raw, VARIANT_A)
            .iter()
            .map(|ld| LevelDuration::new(!ld.level, ld.duration_us))
            .chain(std::iter::once(LevelDuration::new(true, 5_000)))
            .collect();
        let mut dec = FiatV1Decoder::new();
        let out = decode_all(&mut dec, &inverted);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].serial, Some(0x0BADF00D));
        assert_eq!(out[0].button, Some(BUTTON_LOCK));
    }

    /// Duplicate suppression holds until reset(); reset() clears it (as the C reset does).
    #[test]
    fn duplicate_frames_suppressed_until_reset() {
        let sig = signed_frame(0x11223344, BUTTON_TRUNK, 0x3FF, 1, VARIANT_B);
        let (raw, _) = FiatV1Decoder::raw_from_signal(&sig).unwrap();
        let burst: Vec<LevelDuration> = (0..3).flat_map(|_| upload(&raw, VARIANT_B)).collect();
        let mut dec = FiatV1Decoder::new();
        assert_eq!(decode_all(&mut dec, &burst).len(), 1);
        dec.reset();
        assert_eq!(decode_all(&mut dec, &burst).len(), 1);
    }

    /// Gate: a corrupted XOR, a bad sync byte, an invalid button or a degenerate UID is rejected.
    #[test]
    fn corrupted_frames_rejected() {
        let good = FiatV1Decoder::build_raw(0x12345678, BUTTON_LOCK, 0x101, 0x9DAB8C6A, 2);
        let mut bad_xor = good;
        bad_xor[12] ^= 0x40;
        let mut bad_sync = good;
        bad_sync[1] = 0x03;
        bad_sync[12] = FiatV1Decoder::frame_xor(&bad_sync);
        let mut bad_button = good;
        bad_button[6] = (0x3 << 4) | (bad_button[6] & 0x0F);
        bad_button[12] = FiatV1Decoder::frame_xor(&bad_button);
        let zero_uid = FiatV1Decoder::build_raw(0, BUTTON_LOCK, 0x101, 0x9DAB8C6A, 2);
        for (what, raw) in [
            ("xor", bad_xor),
            ("sync", bad_sync),
            ("button", bad_button),
            ("uid", zero_uid),
        ] {
            assert!(
                !FiatV1Decoder::frame_valid(&raw),
                "{what} must fail frame_valid"
            );
            let mut dec = FiatV1Decoder::new();
            let mut stream = upload(&raw, VARIANT_A);
            stream.push(LevelDuration::new(true, 5_000));
            assert!(
                decode_all(&mut dec, &stream).is_empty(),
                "{what}-corrupted frame must not decode"
            );
        }
        // Sanity: the uncorrupted frame does decode through the same path.
        let mut dec = FiatV1Decoder::new();
        assert_eq!(decode_all(&mut dec, &upload(&good, VARIANT_A)).len(), 1);
    }

    /// Unknown key: ProtoPirate refuses to transmit, Flipper-ARF replays the captured frame
    /// byte-for-byte (any button); KAT does the latter, in the captured TE variant
    /// (doctor-firmware's replay; ARF itself always uses variant A timing).
    #[test]
    fn unknown_key_replays_captured_frame() {
        for variant in [VARIANT_A, VARIANT_B] {
            let raw = FiatV1Decoder::build_raw(0x12345678, BUTTON_LOCK, 0x101, 0xDEADBEEF, 2);
            let sig = FiatV1Decoder::decoded_signal(&raw, variant);
            assert_eq!(
                FiatV1Decoder::resolve_key(&FiatV1Decoder::fields(&raw)),
                None
            );
            assert!(sig.encoder_capable, "replay is always possible");
            let one = upload(&raw, variant);
            let expected: Vec<LevelDuration> = (0..ENC_REPEAT).flat_map(|_| one.clone()).collect();
            let enc = FiatV1Decoder::new();
            for btn in [0x01u8, 0x02, 0x04, 0x08, 0x40] {
                let got = enc.encode(&sig, btn).unwrap();
                assert_eq!(
                    pulses(&got),
                    pulses(&expected),
                    "variant {variant} btn {btn:#X}"
                );
            }
        }
    }

    /// Keys from Flipper-ARF's Hitag2 BF cascade beyond ProtoPirate's list: an L2 dictionary key
    /// ("DELPHI"), L4 strategy 1 (UID ^ mask ‖ tail), strategy 2 (a known key's last byte - 5) and
    /// strategy 3 (a known key with epoch 3). Each resolves to (key, epoch) — the same answers the
    /// compiled ARF cascade gives — and transmits a fresh frame signed with it.
    #[test]
    fn arf_cascade_keys_resolve_and_sign() {
        let uid = 0x1357_9BDF;
        let mut strategy2 = KNOWN_KEYS[2];
        strategy2[5] = strategy2[5].wrapping_sub(5);
        let p = (uid ^ 0xDEAD_BEEFu32).to_be_bytes();
        let cases: [([u8; 6], u32); 4] = [
            (*b"DELPHI", 0),
            ([p[0], p[1], p[2], p[3], 0xCA, 0xFE], 0),
            (strategy2, 0),
            (KNOWN_KEYS[4], 3),
        ];
        for (key, epoch) in cases {
            let hop = hitag2::fiat_bcm_authenticator(uid, BUTTON_TRUNK, 0x155, &key, epoch);
            let raw = FiatV1Decoder::build_raw(uid, BUTTON_TRUNK, 0x155, hop, 1);
            assert_eq!(
                FiatV1Decoder::resolve_key(&FiatV1Decoder::fields(&raw)),
                Some((key, epoch)),
                "key {key:02X?} epoch {epoch}"
            );
            let sig = FiatV1Decoder::decoded_signal(&raw, VARIANT_A);
            let tx = FiatV1Decoder::new().encode(&sig, 0x01).unwrap();
            let mut dec = FiatV1Decoder::new();
            let first = decode_all(&mut dec, &tx).remove(0);
            let (tx_raw, _) = FiatV1Decoder::raw_from_signal(&first).unwrap();
            let f = FiatV1Decoder::fields(&tx_raw);
            assert_eq!((f.button, f.control), (BUTTON_LOCK, 0x156));
            assert_eq!(
                f.hop,
                hitag2::fiat_bcm_authenticator(uid, BUTTON_LOCK, 0x156, &key, epoch)
            );
        }
        // ProtoPirate's 9th key is not in ARF's list but is found first by ProtoPirate's step.
        let hop = hitag2::fiat_bcm_authenticator(uid, BUTTON_LOCK, 0x001, &KNOWN_KEYS[8], 0);
        let raw = FiatV1Decoder::build_raw(uid, BUTTON_LOCK, 0x001, hop, 2);
        assert_eq!(
            FiatV1Decoder::resolve_key(&FiatV1Decoder::fields(&raw)),
            Some((KNOWN_KEYS[8], 0))
        );
    }

    /// encode → decode: a fresh frame with the mapped button, control + 1 and a hop that still
    /// verifies with the same known key; 6 repeats in the captured TE variant.
    #[test]
    fn encode_decode_round_trip() {
        for variant in [VARIANT_A, VARIANT_B] {
            let captured = signed_frame(0x5A5A1234, BUTTON_LOCK, 0x3FF, 7, variant);
            let enc = FiatV1Decoder::new();
            // KAT Unlock (0x02) → Fiat 0x8; control 0x3FF + 1 wraps to 0x000.
            let signal = enc.encode(&captured, 0x02).expect("known key → encodable");
            let mut dec = FiatV1Decoder::new();
            let mut out = Vec::new();
            for ld in &signal {
                if let Some(d) = dec.feed(ld.level, ld.duration_us) {
                    out.push(d);
                    dec.reset(); // registry behaviour: every repeat reported
                }
            }
            assert_eq!(
                out.len(),
                ENC_REPEAT,
                "variant {variant}: one decode per repeat"
            );
            let d = &out[0];
            assert_eq!(d.serial, Some(0x5A5A1234));
            assert_eq!(d.button, Some(BUTTON_UNLOCK));
            assert_eq!(d.counter, Some(0x000));
            assert!(d.encoder_capable, "fresh hop must verify with the same key");
            let (raw, v) = FiatV1Decoder::raw_from_signal(d).unwrap();
            assert_eq!(v, variant);
            let f = FiatV1Decoder::fields(&raw);
            assert_eq!(
                f.hop,
                hitag2::fiat_bcm_authenticator(0x5A5A1234, BUTTON_UNLOCK, 0, &KNOWN_KEYS[7], 0)
            );
            assert_eq!(f.tail, 2);
            // Panic has no Fiat V1 code: the captured button is kept.
            let panic = enc.encode(&captured, 0x08).unwrap();
            let mut dec = FiatV1Decoder::new();
            let first = decode_all(&mut dec, &panic).remove(0);
            assert_eq!(first.button, Some(BUTTON_LOCK));
        }
    }

    /// Real capture: IMPORTS/FIAT/Fiat Panda.sub decodes as Fiat V1 (variant A, UID EFB82D66,
    /// button 0x2, control 0x08D). Its hop verifies against none of ProtoPirate's 9 keys (the C
    /// prints KEY:??) nor ARF's L2 dictionary / L4 heuristics, so ProtoPirate cannot transmit it;
    /// KAT (like Flipper-ARF) replays the captured frame.
    #[test]
    fn fiat_panda_capture_decodes() {
        use crate::export::flipper::import_sub_raw;
        use crate::protocols::ProtocolRegistry;
        let path = std::path::Path::new("IMPORTS/FIAT/Fiat Panda.sub");
        if !path.exists() {
            eprintln!("Skip: {:?} not found (run from crate root)", path);
            return;
        }
        let (freq, raw_pairs) = import_sub_raw(path).unwrap();
        let pairs: Vec<LevelDuration> = raw_pairs
            .iter()
            .map(|p| LevelDuration::new(p.level, p.duration_us))
            .collect();
        let mut reg = ProtocolRegistry::new();
        let results = reg.process_signal_stream(&pairs, freq);
        let fiat: Vec<_> = results.iter().filter(|(n, _, _)| n == "Fiat V1").collect();
        assert!(
            !fiat.is_empty(),
            "expected Fiat V1 decodes, got {:?}",
            results.iter().map(|r| &r.0).collect::<Vec<_>>()
        );
        for (_, d, _) in &fiat {
            assert_eq!(d.serial, Some(0xEFB82D66));
            assert_eq!(d.button, Some(BUTTON_TRUNK));
            assert_eq!(d.counter, Some(0x08D));
            assert_eq!(
                d.extra.map(|e| e >> EXTRA_VARIANT_SHIFT),
                Some(VARIANT_A as u64)
            );
            assert!(d.encoder_capable, "ARF replay");
            let (raw, variant) = FiatV1Decoder::raw_from_signal(d).unwrap();
            assert_eq!(
                FiatV1Decoder::resolve_key(&FiatV1Decoder::fields(&raw)),
                None
            );
            let tx = FiatV1Decoder::new().encode(d, 0x02).unwrap();
            let mut dec = FiatV1Decoder::new();
            let replayed = decode_all(&mut dec, &tx);
            assert_eq!(replayed.len(), 1, "6 identical repeats, deduplicated");
            assert_eq!(
                FiatV1Decoder::raw_from_signal(&replayed[0]),
                Some((raw, variant))
            );
        }
    }

    /// The old preamble/sync decoder false-matched this VW Suran capture twice; the gated decoder
    /// must not.
    #[test]
    fn vag_suran_capture_is_not_fiat_v1() {
        use crate::export::flipper::import_sub_raw;
        use crate::protocols::ProtocolRegistry;
        let path = std::path::Path::new("IMPORTS/VAG/Test_55_unlock_and_55_lock_suran.sub");
        if !path.exists() {
            eprintln!("Skip: {:?} not found (run from crate root)", path);
            return;
        }
        let (freq, raw_pairs) = import_sub_raw(path).unwrap();
        let pairs: Vec<LevelDuration> = raw_pairs
            .iter()
            .map(|p| LevelDuration::new(p.level, p.duration_us))
            .collect();
        let mut reg = ProtocolRegistry::new();
        let names: Vec<String> = reg
            .process_signal_stream(&pairs, freq)
            .into_iter()
            .map(|r| r.0)
            .collect();
        assert!(
            !names.iter().any(|n| n == "Fiat V1"),
            "Fiat V1 must not match the Suran capture: {names:?}"
        );
    }
}
