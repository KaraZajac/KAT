//! VAG (VW/Audi/Seat/Skoda) protocol decoder/encoder.
//!
//! Port of ProtoPirate `protocols/vag.c` (HEAD; the primary reference) with the AUT64 cipher of
//! `protocols/aut64.c` ([`super::aut64`]). Flipper-ARF `lib/subghz/protocols/vag.c` runs the same
//! state machine; KAT additionally takes ARF's 315 MHz band and its Lock/Unlock/Trunk remap
//! (`vag_custom_to_btn`). doctor-firmware `vag.c` is the same decoder with a wider type search.
//!
//! # Air format
//!
//! Manchester, 80 bits = Key1 (64) + Key2 (16), in two families:
//!
//! * **Type 1/2** — 300/600µs half-bits. ≥151 preamble pairs of 300µs, a 600µs LOW, then a
//!   16-bit prefix (`0xAF3F` = Type 1 / AUT64, `0xAF1C` = Type 2 / XTEA), Key1 and Key2 sent
//!   **inverted**, and a ≥4ms LOW gap. The decoder sees the prefix's last 15 bits (`0x2F3F` /
//!   `0x2F1C`); without a prefix match it keeps counting (vag_type 0, emitted undecrypted).
//! * **Type 3/4** — 500/1000µs half-bits. ≥31 preamble pairs of 500µs, sync HIGH 1000 + LOW 500,
//!   three 750µs HIGH/LOW pairs, then Key1/Key2 **not** inverted; the frame completes on the 80th
//!   bit (the first bit is implied by the sync, as in the C).
//!
//! Timing windows are the C's: Type 1/2 short |d−300|<100, long d>400 && |d−600|<200, preamble gap
//! |d−600|<200, end gap ≥4001µs; Type 3/4 short |d−500|<100, long |d−1000|<200, sync 750±150. As
//! in the C, a pulse that is neither short nor long is ignored inside the data states (not a reset).
//!
//! Key1 byte 0 is the vehicle/type byte (0x00 "VW Passat", 0xC0 VW, 0xC1 Audi, 0xC2 Seat, 0xC3
//! Skoda); Key1 bytes 1..7 + Key2's high byte are the 8-byte encrypted block; Key2's low byte is the
//! dispatch byte (Type 1/2: 0x2A Lock, 0x1C Unlock, 0x46 Boot; Type 3/4: 0x2B/0x1D/0x47).
//!
//! # Decryption (`vag_parse_data`)
//!
//! * Type 1: dispatch must be a Type 1/2 value; AUT64 keys 1,2,3 (`key_idx` 0..2) are tried and the
//!   first whose plaintext passes `vag_button_valid` wins. Serial = bytes 0..3 (BE), counter = bytes
//!   4..6 (LE, 24 bit), button = the **whole** byte 7 (0x10/0x20/0x40; ARF keeps only the high
//!   nibble — ProtoPirate's full byte is kept, the label uses the high nibble).
//! * Type 2: XTEA (fixed key `0B46502D 5E253718 2BF93A19 622C1206`, 32 rounds; `vag_tea_*`); the
//!   plaintext button nibble must match the dispatch byte (`vag_button_matches`). Button = high
//!   nibble of byte 7.
//! * Type 3 (every 500µs frame): AUT64 key 3 (→ reported as Type 4), then key 2, then key 1, each
//!   accepted on `vag_button_valid`; Type 4: key 3 + `vag_button_matches`.
//!
//! Like ProtoPirate, every 80-bit frame is reported; frames that do not decrypt carry no fields
//! (`crc_valid = false`, not encoder-capable). ARF/doctor report only decrypted frames.
//!
//! # DecodedSignal mapping
//!
//! `data` = Key1, `data_count_bit` = 80, `serial`/`button` as above, `counter` = low 16 bits of
//! the 24-bit counter. `extra` (persisted in `.fob` as `data_extra`):
//!
//! ```text
//! bits  0..15  Key2 (right-aligned rest of the 80-bit frame)
//! bits 16..23  vag_type (0 = no prefix, 1..4)
//! bits 24..31  key_idx (0..2 = AUT64 key 1..3, 0xFF = none / XTEA)
//! bits 32..39  counter bits 16..23
//! bit  63      layout flag (set)
//! ```
//!
//! KAT ≤ v1.3.0 stored `vag_type | key_idx << 8` (flag clear, no Key2); [`VagDecoder::encode`]
//! still accepts that layout.
//!
//! # Encoder (`subghz_protocol_encoder_vag_deserialize` + `vag_encoder_build_*`)
//!
//! Re-encrypts serial / counter + 1 (24-bit) / button with the stored type and key index (a
//! missing key index is recovered by re-parsing Key1/Key2, as the C does) and builds the C upload:
//! Type 1/2 = 220 × (HIGH 300, LOW 300), LOW 300, HIGH 300, prefix + ~Key1 + ~Key2 Manchester at
//! 300µs (bit 1 = HIGH,LOW), LOW 6000 (635 entries); Type 3/4 = twice [45 × (HIGH 500, LOW 500),
//! HIGH 1000, LOW 500, 3 × (HIGH 750, LOW 750), Key1 + Key2 Manchester at 500µs, LOW 10000] (518
//! entries). KAT returns that upload with same-level runs merged (the C emits each half-bit
//! separately; on air they are one pulse). The Flipper loops the upload while the button is held
//! (PP `repeat` 10, ARF 1); KAT returns it once. KAT's generic button maps like ProtoPirate's
//! emulate scene (ARF's D-pad gives the same codes): Lock → 0x20/2, Unlock → 0x10/1, Trunk →
//! 0x40/4 (full-byte form for Type 1, nibble form otherwise), Panic → 8 (nibble form only, as in
//! ProtoPirate — no reference decodes it back); anything else keeps the captured button.
//!
//! **Deviation:** both references turn a Type 1 frame whose type byte is 0x00 into a Type 2 (XTEA,
//! prefix 0xAF1C) frame before transmitting ("Passat … key2 issue"). The real Type 1 captures
//! `IMPORTS/VAG/Passat_2015.sub` and `Vw pasat B7.sub` both carry type byte 0x00 and decrypt as Type 1
//! with AUT64 key 1 (consistent serial, consecutive counters), i.e. those fobs speak Type 1; KAT
//! therefore keeps Type 1. (Using the full-byte button form for Type 1 avoids the nibble-button
//! problem that conversion papered over.)
//!
//! # Key files
//!
//! ProtoPirate's `subghz_protocol_decoder_vag_serialize`: "VAG", Bit 80, Key = Key1, `Key2: 00 00
//! 00 00 00 00 <Key2>`, `Type`, and for decrypted frames Serial, Btn, Cnt (24 bit) and `KeyIdx`.
//! Loading is `_deserialize`: Key1/Key2/Type → `vag_parse_data` (Flipper-ARF's "VAG GROUP" files
//! carry the same lines; it also writes BtnFlags, ignored). Also read: `Key_2` for `Key2`, as in
//! `IMPORTS/VAG/Passat_b5_3bg.sub` (whose `Type: 11011` the C truncates to 3), which loads as the
//! Type 4 frame of `IMPORTS/VAG/Vw_passat_b5_3bg.sub`, and `IMPORTS/VAG/Audi A3 2012 key.sub`
//! (`Type: 7428`, i.e. Type 4). That writer stores `Type` as (last `Key_2` byte << 8) | type; the
//! Audi file loads with serial 3BF8E083, Unlock, counter 019C61, and its sender's screenshot of a
//! later press of the same fob reads "Ser:3BF8E083 Btn:10:OP Cnt:019C63". `Type: 4` is parsed as the decoder parses
//! every 500 µs frame (as Type 3, key 3 first): ProtoPirate's own deserialize parses it strictly as
//! Type 4 and loses frames whose dispatch byte is not a Type 3/4 one (some of `Golf4.sub`'s). Serial/
//! Btn/Cnt that differ from the decrypted ones rebuild the frame, as ProtoPirate's encoder
//! transmits a saved key.

use super::aut64;
use super::common::{ManchesterEvent, ManchesterState};
use super::keys;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`); "VW" is ProtoPirate's catalog alias.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "VAG",
    decoder: "VAG",
    aliases: &["VW"],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "AUT64/XTEA",
    make: "VW/Audi/Seat/Skoda",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// Types 2/3/4 report the decrypted button nibble, Type 1 the whole byte (button in the high
/// nibble; ProtoPirate's vag_button_name, which calls 4/0x40 "Boot").
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match if code > 0x0F { code >> 4 } else { code } {
        0x1 => "Unlock",
        0x2 => "Lock",
        0x4 => "Trunk",
        _ => "Other",
    })
}

// subghz_protocol_vag_const (reported via timing())
const TE_SHORT: u32 = 500;
const TE_LONG: u32 = 1000;
const TE_DELTA: u32 = 80;
const MIN_COUNT_BIT: usize = 80;

const T12_TE_SHORT: u32 = 300;
const T12_TE_LONG: u32 = 600;
const T12_TE_DELTA: u32 = 100;
const T12_GAP_DELTA: u32 = 200;
const T12_PREAMBLE_MIN: u16 = 151;

const T34_TE_SHORT: u32 = 500;
const T34_TE_LONG: u32 = 1000;
const T34_TE_DELTA: u32 = 100;
const T34_LONG_DELTA: u32 = 200;
const T34_SYNC: u32 = 750;
const T34_SYNC_DELTA: u32 = 150;
const T34_PREAMBLE_MIN: u16 = 31;
const T34_SYNC_PAIRS: u8 = 3;

const DATA_GAP_MIN: u32 = 4001;
const TOTAL_BITS: u8 = 80;
const KEY1_BITS: u8 = 64;
const PREFIX_BITS: u8 = 15;
const BIT_LIMIT: u8 = 96;
const FRAME_PREFIX_T1: u32 = 0x2F3F;
const FRAME_PREFIX_T2: u32 = 0x2F1C;

const TEA_DELTA: u32 = 0x9E37_79B9;
const TEA_ROUNDS: usize = 32;
/// vag_tea_key_schedule
const TEA_KEY_SCHEDULE: [u32; 4] = [0x0B46_502D, 0x5E25_3718, 0x2BF9_3A19, 0x622C_1206];

/// `extra` layout flag (see the module doc); clear = KAT ≤ v1.3.0 `type | key_idx << 8`.
const EXTRA_LAYOUT_FLAG: u64 = 1 << 63;
/// `key_idx` value meaning "no AUT64 key" (XTEA, or not decrypted).
const NO_KEY: u8 = 0xFF;

/// `DURATION_DIFF`
fn diff(a: u32, b: u32) -> u32 {
    a.abs_diff(b)
}

/// VAGDecoderStep
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Preamble1,
    Data1,
    Preamble2,
    Sync2A,
    Sync2B,
    Sync2C,
    Data2,
}

/// Result of `vag_parse_data`.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Parsed {
    /// Type after parsing (Type 3 becomes 4 when key 3 decrypts it).
    vag_type: u8,
    decrypted: bool,
    serial: u32,
    /// 24-bit counter.
    cnt: u32,
    btn: u8,
    key_idx: u8,
}

/// Fields the encoder works from (`SubGhzProtocolEncoderVAG` after deserialize).
#[derive(Debug, Clone, Copy, PartialEq)]
struct EncodeFields {
    vag_type: u8,
    key_idx: u8,
    /// Key1 byte 0 (vehicle/type byte), transmitted unchanged.
    type_byte: u8,
    serial: u32,
    cnt: u32,
    btn: u8,
}

/// VAG protocol decoder
pub struct VagDecoder {
    step: DecoderStep,
    manchester: ManchesterState,
    data_low: u32,
    data_high: u32,
    bit_count: u8,
    key1_low: u32,
    key1_high: u32,
    key2_low: u32,
    te_last: u32,
    header_count: u16,
    mid_count: u8,
    vag_type: u8,
}

impl VagDecoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            manchester: ManchesterState::Mid1,
            data_low: 0,
            data_high: 0,
            bit_count: 0,
            key1_low: 0,
            key1_high: 0,
            key2_low: 0,
            te_last: 0,
            header_count: 0,
            mid_count: 0,
            vag_type: 0,
        }
    }

    // ---- crypto (vag_tea_*, vag_aut64_*) ----

    /// vag_tea_decrypt
    fn tea_decrypt(v0: &mut u32, v1: &mut u32) {
        let k = &TEA_KEY_SCHEDULE;
        let mut sum = TEA_DELTA.wrapping_mul(TEA_ROUNDS as u32);
        for _ in 0..TEA_ROUNDS {
            *v1 = v1.wrapping_sub(
                ((*v0 << 4) ^ (*v0 >> 5)).wrapping_add(*v0)
                    ^ sum.wrapping_add(k[((sum >> 11) & 3) as usize]),
            );
            sum = sum.wrapping_sub(TEA_DELTA);
            *v0 = v0.wrapping_sub(
                ((*v1 << 4) ^ (*v1 >> 5)).wrapping_add(*v1)
                    ^ sum.wrapping_add(k[(sum & 3) as usize]),
            );
        }
    }

    /// vag_tea_encrypt
    fn tea_encrypt(v0: &mut u32, v1: &mut u32) {
        let k = &TEA_KEY_SCHEDULE;
        let mut sum: u32 = 0;
        for _ in 0..TEA_ROUNDS {
            *v0 = v0.wrapping_add(
                ((*v1 << 4) ^ (*v1 >> 5)).wrapping_add(*v1)
                    ^ sum.wrapping_add(k[(sum & 3) as usize]),
            );
            sum = sum.wrapping_add(TEA_DELTA);
            *v1 = v1.wrapping_add(
                ((*v0 << 4) ^ (*v0 >> 5)).wrapping_add(*v0)
                    ^ sum.wrapping_add(k[((sum >> 11) & 3) as usize]),
            );
        }
    }

    /// vag_tea_* (XTEA) over an 8-byte block (big-endian halves, as vag.c packs v0/v1).
    fn tea_block(block: &mut [u8; 8], encrypt: bool) {
        let mut v0 = u32::from_be_bytes([block[0], block[1], block[2], block[3]]);
        let mut v1 = u32::from_be_bytes([block[4], block[5], block[6], block[7]]);
        if encrypt {
            Self::tea_encrypt(&mut v0, &mut v1);
        } else {
            Self::tea_decrypt(&mut v0, &mut v1);
        }
        block[..4].copy_from_slice(&v0.to_be_bytes());
        block[4..].copy_from_slice(&v1.to_be_bytes());
    }

    /// vag_aut64_decrypt / vag_aut64_encrypt: AUT64 with keystore key `key_index + 1`; false when
    /// that key is not loaded.
    fn aut64_block(block: &mut [u8; 8], key_index: u8, encrypt: bool) -> bool {
        let store = keys::get_keystore();
        let Some(key) = store.get_vag_key(key_index.wrapping_add(1)) else {
            return false;
        };
        if encrypt {
            aut64::aut64_encrypt(key, block);
        } else {
            aut64::aut64_decrypt(key, block);
        }
        true
    }

    // ---- parse (vag_parse_data and helpers) ----

    fn dispatch_type_1_2(dispatch: u8) -> bool {
        matches!(dispatch, 0x2A | 0x1C | 0x46)
    }

    fn dispatch_type_3_4(dispatch: u8) -> bool {
        matches!(dispatch, 0x2B | 0x1D | 0x47)
    }

    /// vag_button_valid
    fn button_valid(dec: &[u8; 8]) -> bool {
        matches!(dec[7] >> 4, 1 | 2 | 4) || dec[7] == 0
    }

    /// vag_button_matches
    fn button_matches(dec: &[u8; 8], dispatch: u8) -> bool {
        let expected = (dispatch >> 4) & 0xF;
        (dec[7] >> 4) == expected || (dec[7] == 0 && expected == 2)
    }

    /// vag_fill_from_decrypted: serial BE bytes 0..3, counter LE bytes 4..6, button = byte 7 >> 4.
    fn fill_from_decrypted(p: &mut Parsed, dec: &[u8; 8]) {
        p.serial = u32::from_be_bytes([dec[0], dec[1], dec[2], dec[3]]);
        p.cnt = dec[4] as u32 | (dec[5] as u32) << 8 | (dec[6] as u32) << 16;
        p.btn = dec[7] >> 4;
        p.decrypted = true;
    }

    /// vag_parse_data for `vag_type` over Key1 (64) + Key2 (16).
    fn parse_data(vag_type: u8, key1: u64, key2: u16) -> Parsed {
        let mut p = Parsed {
            vag_type,
            decrypted: false,
            serial: 0,
            cnt: 0,
            btn: 0,
            key_idx: NO_KEY,
        };
        let dispatch = key2 as u8;
        let k = key1.to_be_bytes();
        // Encrypted block: Key1 bytes 1..7 + Key2's high byte.
        let block = [k[1], k[2], k[3], k[4], k[5], k[6], k[7], (key2 >> 8) as u8];

        match vag_type {
            1 if Self::dispatch_type_1_2(dispatch) => {
                for key_idx in 0..3u8 {
                    let mut dec = block;
                    if !Self::aut64_block(&mut dec, key_idx, false) {
                        continue;
                    }
                    if Self::button_valid(&dec) {
                        p.serial = u32::from_be_bytes([dec[0], dec[1], dec[2], dec[3]]);
                        p.cnt = dec[4] as u32 | (dec[5] as u32) << 8 | (dec[6] as u32) << 16;
                        p.btn = dec[7];
                        p.key_idx = key_idx;
                        p.decrypted = true;
                        return p;
                    }
                }
            }
            2 if Self::dispatch_type_1_2(dispatch) => {
                let mut dec = block;
                Self::tea_block(&mut dec, false);
                if Self::button_matches(&dec, dispatch) {
                    Self::fill_from_decrypted(&mut p, &dec);
                    p.key_idx = NO_KEY;
                    return p;
                }
            }
            3 => {
                for (key_idx, as_type) in [(2u8, 4u8), (1, 3), (0, 3)] {
                    let mut dec = block;
                    if Self::aut64_block(&mut dec, key_idx, false) && Self::button_valid(&dec) {
                        p.vag_type = as_type;
                        p.key_idx = key_idx;
                        Self::fill_from_decrypted(&mut p, &dec);
                        return p;
                    }
                }
            }
            4 if Self::dispatch_type_3_4(dispatch) => {
                let mut dec = block;
                if Self::aut64_block(&mut dec, 2, false) && Self::button_matches(&dec, dispatch) {
                    p.key_idx = 2;
                    Self::fill_from_decrypted(&mut p, &dec);
                    return p;
                }
            }
            _ => {}
        }
        Parsed {
            vag_type: p.vag_type,
            decrypted: false,
            serial: 0,
            cnt: 0,
            btn: 0,
            key_idx: NO_KEY,
        }
    }

    // ---- decoder ----

    /// Manchester data step shared by Data1/Data2: shift one decoded bit into data_high:data_low.
    fn push_bit(&mut self, bit: bool) {
        let carry = self.data_low >> 31;
        self.data_low = (self.data_low << 1) | bit as u32;
        self.data_high = (self.data_high << 1) | carry;
        self.bit_count = self.bit_count.wrapping_add(1);
    }

    /// A completed 80-bit frame (the C's callback point).
    fn emit(&self) -> DecodedSignal {
        let key1 = ((self.key1_high as u64) << 32) | self.key1_low as u64;
        let key2 = (self.key2_low & 0xFFFF) as u16;
        let p = Self::parse_data(self.vag_type, key1, key2);
        Self::signal_from(key1, key2, &p)
    }

    fn signal_from(key1: u64, key2: u16, p: &Parsed) -> DecodedSignal {
        let cnt_hi = if p.decrypted { (p.cnt >> 16) & 0xFF } else { 0 };
        let extra = EXTRA_LAYOUT_FLAG
            | (cnt_hi as u64) << 32
            | (p.key_idx as u64) << 24
            | (p.vag_type as u64) << 16
            | key2 as u64;
        DecodedSignal {
            serial: p.decrypted.then_some(p.serial),
            button: p.decrypted.then_some(p.btn),
            counter: p.decrypted.then_some((p.cnt & 0xFFFF) as u16),
            crc_valid: p.decrypted,
            data: key1,
            data_count_bit: TOTAL_BITS as usize,
            encoder_capable: p.decrypted,
            extra: Some(extra),
            protocol_display_name: None,
        }
    }

    /// Reset-step entry into Preamble1/Preamble2.
    fn start_preamble(&mut self, step: DecoderStep, duration: u32) {
        self.step = step;
        self.data_low = 0;
        self.data_high = 0;
        self.header_count = 0;
        self.mid_count = 0;
        self.bit_count = 0;
        self.vag_type = 0;
        self.te_last = duration;
        self.manchester = ManchesterState::Mid1;
    }

    // ---- encoder ----

    /// Decode `extra`: (vag_type, key_idx, Key2 if known, counter bits 16..23).
    fn parse_extra(extra: u64) -> (u8, u8, Option<u16>, u32) {
        if extra & EXTRA_LAYOUT_FLAG != 0 {
            (
                (extra >> 16) as u8,
                (extra >> 24) as u8,
                Some(extra as u16),
                ((extra >> 32) & 0xFF) as u32,
            )
        } else {
            // KAT <= v1.3.0: vag_type | key_idx << 8.
            (extra as u8, (extra >> 8) as u8, None, 0)
        }
    }

    /// Resolve what the C encoder works from: stored fields (or a re-parse of Key1/Key2 when the
    /// key index or the fields are missing, as `subghz_protocol_encoder_vag_deserialize` does).
    fn encode_fields(decoded: &DecodedSignal) -> Option<EncodeFields> {
        let (vag_type, mut key_idx, key2, cnt_hi) = Self::parse_extra(decoded.extra?);
        let key1 = decoded.data;
        let mut serial = decoded.serial;
        let mut btn = decoded.button;
        let mut cnt = decoded.counter.map(|c| cnt_hi << 16 | c as u32);
        if key_idx == NO_KEY || serial.is_none() || cnt.is_none() {
            if let Some(key2) = key2 {
                let p = Self::parse_data(vag_type, key1, key2);
                if p.decrypted {
                    key_idx = p.key_idx;
                    if serial.is_none() || cnt.is_none() {
                        serial = Some(p.serial);
                        cnt = Some(p.cnt);
                        btn = Some(p.btn);
                    }
                }
            }
        }
        Some(EncodeFields {
            vag_type,
            key_idx,
            type_byte: (key1 >> 56) as u8,
            serial: serial?,
            cnt: cnt?,
            btn: btn.unwrap_or(0),
        })
    }

    /// KAT generic button → VAG button, following ProtoPirate's emulate mapping
    /// (`emu_button_for_protocol`): full-byte codes for Type 1, nibble codes otherwise.
    fn map_button(vag_type: u8, captured: u8, button: u8) -> u8 {
        let full_byte = vag_type == 1;
        match (button, full_byte) {
            (0x01, true) => 0x20,
            (0x02, true) => 0x10,
            (0x04, true) => 0x40,
            (0x01, false) => 0x2,
            (0x02, false) => 0x1,
            (0x04, false) => 0x4,
            (0x08, false) => 0x8,
            _ => captured,
        }
    }

    /// vag_get_dispatch_byte
    fn dispatch_byte(btn: u8, vag_type: u8) -> u8 {
        let t12 = vag_type == 1 || vag_type == 2;
        match (btn, t12) {
            (0x20 | 2, true) => 0x2A,
            (0x40 | 4, true) => 0x46,
            (0x10 | 1, true) => 0x1C,
            (_, true) => 0x2A,
            (0x20 | 2, false) => 0x2B,
            (0x40 | 4, false) => 0x47,
            (0x10 | 1, false) => 0x1D,
            (_, false) => 0x2B,
        }
    }

    /// vag_btn_to_byte
    fn btn_to_byte(btn: u8, vag_type: u8) -> u8 {
        if vag_type == 1 {
            return btn;
        }
        match btn {
            0x1 => 0x10,
            0x2 => 0x20,
            0x4 => 0x40,
            _ => btn,
        }
    }

    /// Plaintext → encrypted Key1/Key2 (the common part of vag_encoder_build_type*).
    fn build_keys(f: &EncodeFields) -> Option<(u64, u16)> {
        let btn_byte = Self::btn_to_byte(f.btn, f.vag_type);
        let dispatch = Self::dispatch_byte(btn_byte, f.vag_type);
        let s = f.serial.to_be_bytes();
        let mut block = [
            s[0],
            s[1],
            s[2],
            s[3],
            f.cnt as u8,
            (f.cnt >> 8) as u8,
            (f.cnt >> 16) as u8,
            btn_byte,
        ];
        match f.vag_type {
            2 => Self::tea_block(&mut block, true),
            1 => {
                let key_idx = if f.key_idx != NO_KEY { f.key_idx } else { 0 };
                if !Self::aut64_block(&mut block, key_idx, true) {
                    return None;
                }
            }
            _ => {
                let key_idx = if f.key_idx != NO_KEY {
                    f.key_idx
                } else if f.vag_type == 4 {
                    2
                } else {
                    1
                };
                if !Self::aut64_block(&mut block, key_idx, true) {
                    return None;
                }
            }
        }
        let key1 = u64::from_be_bytes([
            f.type_byte,
            block[0],
            block[1],
            block[2],
            block[3],
            block[4],
            block[5],
            block[6],
        ]);
        let key2 = (block[7] as u16) << 8 | dispatch as u16;
        Some((key1, key2))
    }

    /// pp_emit_manchester_bit over the low `bits` bits of `value`, MSB first (bit 1 = HIGH, LOW).
    fn emit_manchester(upload: &mut Vec<LevelDuration>, value: u64, bits: u32, te: u32) {
        for i in (0..bits).rev() {
            let bit = (value >> i) & 1 == 1;
            upload.push(LevelDuration::new(bit, te));
            upload.push(LevelDuration::new(!bit, te));
        }
    }

    /// pp_emit_short_pairs
    fn emit_pairs(upload: &mut Vec<LevelDuration>, te: u32, pairs: usize) {
        for _ in 0..pairs {
            upload.push(LevelDuration::new(true, te));
            upload.push(LevelDuration::new(false, te));
        }
    }

    /// vag_encoder_build_type1 / _type2 upload (635 entries).
    fn build_upload_type12(vag_type: u8, key1: u64, key2: u16) -> Vec<LevelDuration> {
        let mut up = Vec::with_capacity(635);
        Self::emit_pairs(&mut up, 300, 220);
        up.push(LevelDuration::new(false, 300));
        up.push(LevelDuration::new(true, 300));
        let prefix: u16 = if vag_type == 2 { 0xAF1C } else { 0xAF3F };
        Self::emit_manchester(&mut up, prefix as u64, 16, 300);
        Self::emit_manchester(&mut up, !key1, 64, 300);
        Self::emit_manchester(&mut up, !key2 as u64, 16, 300);
        up.push(LevelDuration::new(false, 6000));
        up
    }

    /// vag_encoder_build_type3_4 upload (518 entries: the frame twice).
    fn build_upload_type34(key1: u64, key2: u16) -> Vec<LevelDuration> {
        let mut up = Vec::with_capacity(518);
        for _ in 0..2 {
            Self::emit_pairs(&mut up, 500, 45);
            up.push(LevelDuration::new(true, 1000));
            up.push(LevelDuration::new(false, 500));
            Self::emit_pairs(&mut up, 750, 3);
            Self::emit_manchester(&mut up, key1, 64, 500);
            Self::emit_manchester(&mut up, key2 as u64, 16, 500);
            up.push(LevelDuration::new(false, 10000));
        }
        up
    }

    /// Merge adjacent same-level entries: the C upload emits each Manchester half-bit separately
    /// (e.g. LOW 300 + LOW 300); on air that is one LOW 600, which is what KAT returns.
    fn merge_levels(upload: Vec<LevelDuration>) -> Vec<LevelDuration> {
        let mut out: Vec<LevelDuration> = Vec::with_capacity(upload.len());
        for p in upload {
            match out.last_mut() {
                Some(last) if last.level == p.level => last.duration_us += p.duration_us,
                _ => out.push(p),
            }
        }
        out
    }

    /// The C encoder after deserialize: the type's builder. Returns the C upload (unmerged) and
    /// the transmitted Key1/Key2.
    fn build(f: &EncodeFields) -> Option<(Vec<LevelDuration>, u64, u16)> {
        // Unknown type: the C defaults to Type 1.
        let mut f = *f;
        if !(1..=4).contains(&f.vag_type) {
            f.vag_type = 1;
        }
        let (key1, key2) = Self::build_keys(&f)?;
        let upload = if f.vag_type <= 2 {
            Self::build_upload_type12(f.vag_type, key1, key2)
        } else {
            Self::build_upload_type34(key1, key2)
        };
        Some((upload, key1, key2))
    }
}

impl ProtocolDecoder for VagDecoder {
    fn name(&self) -> &'static str {
        "VAG"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: MIN_COUNT_BIT,
        }
    }

    /// Type 1/2 frames are 300/600 µs; `timing()` is the 500/1000 µs of Type 3/4.
    fn timing_variants(&self) -> Vec<ProtocolTiming> {
        vec![ProtocolTiming {
            te_short: T12_TE_SHORT,
            te_long: T12_TE_LONG,
            te_delta: T12_TE_DELTA,
            min_count_bit: MIN_COUNT_BIT,
        }]
    }

    fn supported_frequencies(&self) -> &[u32] {
        // ProtoPirate flags 433 MHz; Flipper-ARF adds 315 MHz. (The registry matches ±2%, so
        // 433.92 also covers the 434.42 MHz European fobs.)
        &[315_000_000, 433_920_000, 434_420_000]
    }

    fn reset(&mut self) {
        // subghz_protocol_decoder_vag_reset (key1/key2 are left as they were, like the C).
        self.step = DecoderStep::Reset;
        self.data_low = 0;
        self.data_high = 0;
        self.bit_count = 0;
        self.vag_type = 0;
        self.header_count = 0;
        self.mid_count = 0;
        self.manchester = ManchesterState::Mid1;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::Reset => {
                if !level {
                    return None;
                }
                if diff(duration, T12_TE_SHORT) < T12_TE_DELTA {
                    self.start_preamble(DecoderStep::Preamble1, duration);
                } else if diff(duration, T34_TE_SHORT) < T34_TE_DELTA {
                    self.start_preamble(DecoderStep::Preamble2, duration);
                }
            }

            DecoderStep::Preamble1 => {
                if level {
                    return None;
                }
                if diff(duration, T12_TE_SHORT) < T12_TE_DELTA {
                    if diff(self.te_last, T12_TE_SHORT) < T12_TE_DELTA {
                        self.te_last = duration;
                        self.header_count += 1;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                    return None;
                }
                self.step = DecoderStep::Reset;
                if self.header_count >= T12_PREAMBLE_MIN
                    && diff(duration, T12_TE_LONG) < T12_GAP_DELTA
                    && diff(self.te_last, T12_TE_SHORT) < T12_TE_DELTA
                {
                    self.step = DecoderStep::Data1;
                }
            }

            DecoderStep::Data1 => {
                if self.bit_count >= BIT_LIMIT {
                    self.step = DecoderStep::Reset;
                    return None;
                }
                // ProtoPirate's mapping: level ? ShortLow : ShortHigh.
                let long = if diff(duration, T12_TE_SHORT) < T12_TE_DELTA {
                    Some(false)
                } else if duration > T12_TE_SHORT + T12_TE_DELTA
                    && diff(duration, T12_TE_LONG) < T12_GAP_DELTA
                {
                    Some(true)
                } else {
                    None
                };
                if let Some(long) = long {
                    let event = ManchesterEvent::from_half_bit(!level, long);
                    if let Some(bit) = self.manchester.advance(event) {
                        self.push_bit(bit);
                        if self.bit_count == PREFIX_BITS {
                            let prefix = self.data_low;
                            if self.data_high == 0
                                && (prefix == FRAME_PREFIX_T1 || prefix == FRAME_PREFIX_T2)
                            {
                                self.vag_type = if prefix == FRAME_PREFIX_T1 { 1 } else { 2 };
                                self.data_low = 0;
                                self.data_high = 0;
                                self.bit_count = 0;
                            }
                        } else if self.bit_count == KEY1_BITS {
                            self.key1_low = !self.data_low;
                            self.key1_high = !self.data_high;
                            self.data_low = 0;
                            self.data_high = 0;
                        }
                    }
                    return None;
                }
                if level || duration < DATA_GAP_MIN {
                    return None;
                }
                let mut result = None;
                if self.bit_count == TOTAL_BITS {
                    self.key2_low = !self.data_low & 0xFFFF;
                    result = Some(self.emit());
                }
                self.data_low = 0;
                self.data_high = 0;
                self.bit_count = 0;
                self.step = DecoderStep::Reset;
                return result;
            }

            DecoderStep::Preamble2 => {
                if !level {
                    if diff(duration, T34_TE_SHORT) < T34_TE_DELTA
                        && diff(self.te_last, T34_TE_SHORT) < T34_TE_DELTA
                    {
                        self.te_last = duration;
                        self.header_count += 1;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                    return None;
                }
                if self.header_count >= T34_PREAMBLE_MIN
                    && diff(duration, T34_TE_LONG) < T34_LONG_DELTA
                    && diff(self.te_last, T34_TE_SHORT) < T34_TE_DELTA
                {
                    self.te_last = duration;
                    self.step = DecoderStep::Sync2A;
                }
            }

            DecoderStep::Sync2A => {
                if !level
                    && diff(duration, T34_TE_SHORT) < T34_TE_DELTA
                    && diff(self.te_last, T34_TE_LONG) < T34_LONG_DELTA
                {
                    self.te_last = duration;
                    self.step = DecoderStep::Sync2B;
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::Sync2B => {
                if level && diff(duration, T34_SYNC) < T34_SYNC_DELTA {
                    self.te_last = duration;
                    self.step = DecoderStep::Sync2C;
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::Sync2C => {
                if !level
                    && diff(duration, T34_SYNC) < T34_SYNC_DELTA
                    && diff(self.te_last, T34_SYNC) < T34_SYNC_DELTA
                {
                    self.mid_count += 1;
                    self.step = DecoderStep::Sync2B;
                    if self.mid_count == T34_SYNC_PAIRS {
                        self.data_low = 1;
                        self.data_high = 0;
                        self.bit_count = 1;
                        self.manchester = ManchesterState::Mid1;
                        self.step = DecoderStep::Data2;
                    }
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::Data2 => {
                let long = if diff(duration, T34_TE_SHORT) < T34_TE_DELTA {
                    Some(false)
                } else if diff(duration, T34_TE_LONG) < T34_LONG_DELTA {
                    Some(true)
                } else {
                    None
                };
                if let Some(long) = long {
                    let event = ManchesterEvent::from_half_bit(!level, long);
                    if let Some(bit) = self.manchester.advance(event) {
                        self.push_bit(bit);
                        if self.bit_count == KEY1_BITS {
                            self.key1_low = self.data_low;
                            self.key1_high = self.data_high;
                            self.data_low = 0;
                            self.data_high = 0;
                        }
                    }
                }
                if self.bit_count != TOTAL_BITS {
                    return None;
                }
                self.key2_low = self.data_low & 0xFFFF;
                self.vag_type = 3;
                let result = self.emit();
                self.data_low = 0;
                self.data_high = 0;
                self.bit_count = 0;
                self.step = DecoderStep::Reset;
                return Some(result);
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        let mut f = Self::encode_fields(decoded)?;
        f.cnt = f.cnt.wrapping_add(1) & 0xFF_FFFF;
        f.btn = Self::map_button(f.vag_type, f.btn, button);
        Self::build(&f).map(|(upload, _, _)| Self::merge_levels(upload))
    }

    /// Key files: `subghz_protocol_decoder_vag_deserialize` — Bit 80, Key = Key1, `Key2` (8 hex
    /// bytes, the low two used), `Type` read as the C does (`(uint8_t)` of the number), then
    /// `vag_parse_data`. `Key_2` is accepted for `Key2` (`IMPORTS/VAG/Passat_b5_3bg.sub`, written by
    /// a build no reference matches, spells it so; its `Type: 11011` is Type 3 to the C). The C
    /// encoder transmits from Serial/Btn/Cnt (+ KeyIdx): when those differ from the decrypted
    /// fields, KAT rebuilds Key1/Key2 from them the same way.
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != MIN_COUNT_BIT {
            return None;
        }
        let key1 = key.key_u64()?;
        let key2 = key.hex_field("Key2").or_else(|| key.hex_field("Key_2")).unwrap_or(0) as u16;
        // A 500 µs frame is parsed as Type 3 (key 3 first, reported as Type 4), as the decoder
        // does: the C saves the post-parse Type 4, which its own deserialize then parses strictly
        // and rejects when the dispatch byte is not a Type 3/4 one (e.g. some IMPORTS/VAG/Golf4.sub
        // frames).
        let vag_type = match key.u32_field("Type").unwrap_or(0) as u8 {
            4 => 3,
            t => t,
        };
        let p = Self::parse_data(vag_type, key1, key2);
        let d = Self::signal_from(key1, key2, &p);
        let (serial, button, counter) = (key.u32_field("Serial"), key.u32_field("Btn"), key.u32_field("Cnt"));
        if serial.is_none() && button.is_none() && counter.is_none() {
            return Some(d);
        }
        // Flipper-ARF stores a Type 1 button as its high nibble (`dec[7] >> 4`: `Btn: 2` for
        // 0x20); ProtoPirate and KAT keep the whole byte. Read ARF's form as the byte it means, or
        // the frame is rebuilt around plaintext 0x02, which no key decrypts.
        let button = button.map(|b| if p.vag_type == 1 && b < 0x10 { b << 4 } else { b });
        let now = p.decrypted.then_some((p.serial, p.btn, p.cnt));
        let want = (
            serial.or(now.map(|n| n.0)),
            button.map(|b| b as u8).or(now.map(|n| n.1)),
            counter.map(|c| c & 0xFF_FFFF).or(now.map(|n| n.2)),
        );
        let (Some(serial), Some(btn), Some(cnt)) = want else { return Some(d) };
        let f = EncodeFields {
            vag_type: p.vag_type,
            key_idx: key.u32_field("KeyIdx").map_or(p.key_idx, |k| k as u8),
            type_byte: (key1 >> 56) as u8,
            serial,
            cnt,
            btn,
        };
        if Some((serial, btn, cnt)) == now || !(1..=4).contains(&f.vag_type) {
            return Some(d);
        }
        let (k1, k2) = Self::build_keys(&f)?;
        Some(Self::signal_from(k1, k2, &Self::parse_data(f.vag_type, k1, k2)))
    }

    /// Key files: `subghz_protocol_decoder_vag_serialize` — "VAG", Bit 80, Key = Key1, `Key2` (8 hex
    /// bytes, Key2 in the last two), `Type` (after parsing: 3 → 4 for key 3), and for a decrypted
    /// frame Serial, Btn, Cnt (24 bit; `pp_serialize_fields`) and `KeyIdx` (0..2, 255 = XTEA).
    /// None for a capture without Key2 (KAT ≤ 1.3 `extra` layout).
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        let (vag_type, key_idx, key2, cnt_hi) = Self::parse_extra(decoded.extra?);
        let mut key = FlipperKey::for_protocol("VAG", MIN_COUNT_BIT, decoded.data);
        key.push_hex("Key2", key2? as u64, 8);
        key.push_u32("Type", vag_type.into());
        if let (Some(serial), Some(btn), Some(cnt)) = (decoded.serial, decoded.button, decoded.counter) {
            key.push_u32("Serial", serial);
            key.push_u32("Btn", btn.into());
            key.push_u32("Cnt", cnt_hi << 16 | cnt as u32);
            key.push_u32("KeyIdx", key_idx.into());
        }
        Some(key)
    }
}

impl Default for VagDecoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::export::flipper::import_sub_raw;
    use std::path::Path;

    fn load_keys() {
        keys::load_keystore_from_embedded();
    }

    /// Merge adjacent same-level entries (what the RF looks like once transmitted).
    fn merged(pairs: &[LevelDuration]) -> Vec<LevelDuration> {
        let mut out: Vec<LevelDuration> = Vec::with_capacity(pairs.len());
        for p in pairs {
            match out.last_mut() {
                Some(last) if last.level == p.level => last.duration_us += p.duration_us,
                _ => out.push(LevelDuration::new(p.level, p.duration_us)),
            }
        }
        out
    }

    fn decode_all(pairs: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = VagDecoder::new();
        pairs
            .iter()
            .filter_map(|p| dec.feed(p.level, p.duration_us))
            .collect()
    }

    fn sub_frames(rel: &str) -> Option<Vec<DecodedSignal>> {
        let path = Path::new("IMPORTS").join(rel);
        if !path.exists() {
            eprintln!("Skip: {path:?} not found (run from crate root)");
            return None;
        }
        let (_, raw) = import_sub_raw(&path).unwrap();
        let pairs: Vec<LevelDuration> = raw
            .iter()
            .map(|p| LevelDuration::new(p.level, p.duration_us))
            .collect();
        Some(decode_all(&pairs))
    }

    fn fields(f: &DecodedSignal) -> (u32, u8, u32, u8, u8) {
        let (vag_type, key_idx, _, cnt_hi) = VagDecoder::parse_extra(f.extra.unwrap());
        (
            f.serial.unwrap(),
            f.button.unwrap(),
            cnt_hi << 16 | f.counter.unwrap() as u32,
            vag_type,
            key_idx,
        )
    }

    /// Real captures, cross-checked against the compiled ProtoPirate vag.c (host harness): frame
    /// count, Key1/Key2, type, serial, counter, button and key index of every decoded frame.
    #[test]
    fn decodes_real_captures_like_protopirate() {
        load_keys();
        // (file, frames, decrypted frames, a known frame: key1, key2, type, serial, cnt, btn, kidx)
        type Frame = (u64, u16, u8, u32, u32, u8, u8);
        let cases: [(&str, usize, usize, Frame); 7] = [
            ("VAG/Vw_passat_b5_3bg.sub", 27, 27, (0xC09E_BD07_92CA_7FC3, 0xA72B, 4, 0x0C21_578B, 0x012B06, 2, 2)),
            ("VAG/Golf4.sub", 38, 36, (0xC0C2_7ADB_B703_4BD1, 0x292B, 4, 0x0A2F_9F3C, 0x005196, 2, 2)),
            ("VAG/VW_Skoda_Seat_2008.sub", 31, 31, (0xC0E4_3A01_2D9E_B632, 0x2A1D, 4, 0x1530_2419, 0x000038, 1, 2)),
            ("SKODA/Skoda_Fabia_2007 RAW.sub", 5, 5, (0xC0D6_1B7A_4201_7AFA, 0x7B2B, 4, 0x0768_7620, 0x00847A, 0, 2)),
            ("VAG/Passat_2015.sub", 8, 8, (0x00D7_11F8_6F28_58A1, 0x662A, 1, 0xFD77_9294, 0x009392, 0x20, 0)),
            ("VAG/Vw pasat B7.sub", 12, 12, (0x009E_E4BD_B292_62F3, 0x832A, 1, 0x3C07_EAA9, 0x018299, 0x20, 0)),
            ("VAG/VAG_2.sub", 8, 8, (0x0072_44E6_232F_D8DC, 0xFE1C, 2, 0x0DED_2BA3, 0x002FEF, 1, NO_KEY)),
        ];
        for (rel, n, n_dec, (k1, k2, ty, ser, cnt, btn, kidx)) in cases {
            let Some(frames) = sub_frames(rel) else { continue };
            assert_eq!(frames.len(), n, "{rel}: frame count");
            assert_eq!(frames.iter().filter(|f| f.crc_valid).count(), n_dec, "{rel}: decrypted");
            let f = frames
                .iter()
                .find(|f| f.data == k1 && f.extra.map(|e| e as u16) == Some(k2))
                .unwrap_or_else(|| panic!("{rel}: frame {k1:016X}/{k2:04X} missing"));
            assert!(f.crc_valid && f.encoder_capable, "{rel}");
            assert_eq!(fields(f), (ser, btn, cnt, ty, kidx), "{rel}");
        }
        // The decoded key file IMPORTS/VAG/Passat_b5_3bg.sub (Key C0 9E BD 07 92 CA 7F C3, Key_2
        // …A7 2B, Type 3) is one of Vw_passat_b5_3bg.sub's frames (checked above): Type 3 → 4.
        let p = VagDecoder::parse_data(3, 0xC09E_BD07_92CA_7FC3, 0xA72B);
        assert_eq!((p.decrypted, p.vag_type, p.serial, p.cnt, p.btn, p.key_idx), (true, 4, 0x0C21_578B, 0x012B06, 2, 2));
    }

    #[test]
    fn test55_type2_capture() {
        load_keys();
        let Some(frames) = sub_frames("VAG/Test_55_unlock_and_55_lock_suran.sub") else { return };
        assert_eq!(frames.len(), 69);
        assert!(frames.iter().all(|f| f.crc_valid && f.serial == Some(0x10C7_0EF7)));
        assert!(frames.iter().all(|f| VagDecoder::parse_extra(f.extra.unwrap()).0 == 2));
    }

    /// Undecryptable frames are still reported (ProtoPirate shows them as "corrupted"), carry
    /// Key1/Key2/type in `data`/`extra`, but no fields and no encoder.
    #[test]
    fn undecrypted_frame_is_reported_without_fields() {
        load_keys();
        let key1 = 0xC012_3456_789A_BCDEu64;
        let key2 = 0x5A00u16; // dispatch 0x00: no type accepts it
        let wire = merged(&VagDecoder::build_upload_type34(key1, key2));
        let frames = decode_all(&wire);
        assert_eq!(frames.len(), 2);
        for f in &frames {
            assert_eq!(f.data, key1);
            assert!(!f.crc_valid && !f.encoder_capable && f.serial.is_none());
            let (ty, kidx, k2, _) = VagDecoder::parse_extra(f.extra.unwrap());
            assert_eq!((ty, kidx, k2), (3, NO_KEY, Some(key2)));
            assert!(VagDecoder::new().encode(f, 0x01).is_none());
        }
    }

    fn captured(f: &EncodeFields, key1: u64, key2: u16) -> DecodedSignal {
        let p = Parsed {
            vag_type: f.vag_type,
            decrypted: true,
            serial: f.serial,
            cnt: f.cnt,
            btn: f.btn,
            key_idx: f.key_idx,
        };
        VagDecoder::signal_from(key1, key2, &p)
    }

    /// Encoder → decoder round trip: the transmitted frame decodes to the same serial, counter + 1
    /// (24-bit, carried through `extra`) and the requested button. Covered: the key layouts the C
    /// decoder always reads back — Type 1 with AUT64 key 1 (Passat captures), Type 2 (XTEA) and
    /// Type 3/4 with key 3 (every 500µs capture). Frames under the other keys are tried after a
    /// first key whose weak button check passes ~19% of random plaintexts, so the C itself
    /// misreads some of them (KAT matches the compiled C there too; checked offline).
    #[test]
    fn encode_decode_round_trip_all_types() {
        load_keys();
        let mut x: u64 = 0x2545_F491_4F6C_DD1D;
        let mut next = || {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            x
        };
        for i in 0..400 {
            let r = next();
            let (vag_type, key_idx, decoded_type) =
                [(1u8, 0u8, 1u8), (2, NO_KEY, 2), (3, 2, 4), (4, 2, 4)][i % 4];
            // Type 3/4 imply Key1's first bit (1) from the sync, so their type byte is 0xC0..0xC3.
            let type_byte = [0xC0u8, 0xC1, 0xC2, 0xC3, 0x00][(r >> 8) as usize % if vag_type <= 2 { 5 } else { 4 }];
            let btn = if vag_type == 1 { 0x20 } else { 2 };
            let f = EncodeFields {
                vag_type,
                key_idx,
                type_byte,
                serial: (r >> 16) as u32,
                cnt: ((r >> 40) as u32) & 0xFF_FFFF,
                btn,
            };
            let (_, k1, k2) = VagDecoder::build(&f).expect("captured frame builds");
            let cap = captured(&f, k1, k2);
            for (cmd, want_nibble) in [(0x01u8, 2u8), (0x02, 1), (0x04, 4)] {
                let up = VagDecoder::new().encode(&cap, cmd).expect("encodes");
                let (_, k1n, k2n) = VagDecoder::build(&EncodeFields {
                    cnt: (f.cnt + 1) & 0xFF_FFFF,
                    btn: VagDecoder::map_button(vag_type, f.btn, cmd),
                    ..f
                })
                .unwrap();
                let c_upload = if vag_type <= 2 {
                    VagDecoder::build_upload_type12(vag_type, k1n, k2n)
                } else {
                    VagDecoder::build_upload_type34(k1n, k2n)
                };
                assert_eq!(c_upload.len(), if vag_type <= 2 { 635 } else { 518 });
                let pulses = |v: &[LevelDuration]| v.iter().map(|p| (p.level, p.duration_us)).collect::<Vec<_>>();
                assert_eq!(pulses(&up), pulses(&merged(&c_upload)), "the C upload, same-level runs merged");
                let frames = decode_all(&up);
                assert_eq!(frames.len(), if vag_type <= 2 { 1 } else { 2 }, "type {vag_type}");
                for d in &frames {
                    let want_btn = if vag_type == 1 { want_nibble << 4 } else { want_nibble };
                    assert_eq!(
                        fields(d),
                        (f.serial, want_btn, (f.cnt + 1) & 0xFF_FFFF, decoded_type, key_idx),
                        "type {vag_type} cmd {cmd}"
                    );
                    assert_eq!(d.data >> 56, type_byte as u64);
                }
            }
        }
    }

    /// A Type 1 capture with type byte 0x00 (Passat_2015.sub) is re-sent as Type 1 (prefix 0xAF3F,
    /// AUT64), not converted to Type 2 like the references do (module doc).
    #[test]
    fn passat_type1_stays_type1() {
        load_keys();
        let Some(frames) = sub_frames("VAG/Passat_2015.sub") else { return };
        let f = &frames[0];
        let up = VagDecoder::new().encode(f, 0x02).unwrap();
        let back = decode_all(&up);
        assert_eq!(back.len(), 1);
        let (ser, btn, cnt, ty, kidx) = fields(&back[0]);
        let (ser0, _, cnt0, _, _) = fields(f);
        assert_eq!((ser, btn, cnt, ty, kidx), (ser0, 0x10, cnt0 + 1, 1, 0));
    }

    /// `.fob` files from KAT <= v1.3.0 stored `extra = vag_type | key_idx << 8` (no Key2).
    #[test]
    fn legacy_extra_layout_still_encodes() {
        load_keys();
        let f = EncodeFields { vag_type: 2, key_idx: NO_KEY, type_byte: 0, serial: 0x1234_5678, cnt: 0x1234, btn: 1 };
        let (_, k1, _) = VagDecoder::build(&f).unwrap();
        let legacy = DecodedSignal {
            serial: Some(f.serial),
            button: Some(1),
            counter: Some(0x1234),
            crc_valid: true,
            data: k1,
            data_count_bit: 80,
            encoder_capable: true,
            extra: Some(0x0002 | (NO_KEY as u64) << 8),
            protocol_display_name: None,
        };
        let back = decode_all(&VagDecoder::new().encode(&legacy, 0x01).unwrap());
        assert_eq!(back.len(), 1);
        assert_eq!(fields(&back[0]), (0x1234_5678, 2, 0x1235, 2, NO_KEY));
        assert_eq!(VagDecoder::parse_extra(0x0104), (4, 1, None, 0));
    }
    /// Cross-protocol guard for VAG, Subaru and Chrysler V0: no other registry protocol's encoder
    /// output, at each of its frequencies, decodes as one of them (VAG reports undecrypted 80-bit
    /// frames like ProtoPirate, so its framing alone must keep it off foreign waveforms); and their
    /// own encoder output is reported by the full registry as themselves.
    #[test]
    fn registry_guard_vag_subaru_chrysler() {
        use crate::protocols::ProtocolRegistry;
        use std::panic::{catch_unwind, AssertUnwindSafe};
        load_keys();
        const MINE: [&str; 3] = ["VAG", "Subaru", "Chrysler V0"];
        let reg0 = ProtocolRegistry::new();
        for name in reg0.list_protocols() {
            if MINE.contains(&name) {
                continue;
            }
            let dec = reg0.get(name).unwrap();
            if !dec.supports_encoding() {
                continue;
            }
            for extra in [0x1234_5678_9ABCu64, (1 << 40) | (0x5A << 32) | 0x1234] {
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
                for btn in [0x01u8, 0x02, 0x04] {
                    let Ok(Some(wire)) = catch_unwind(AssertUnwindSafe(|| dec.encode(&src, btn))) else {
                        continue;
                    };
                    for &f in dec.supported_frequencies() {
                        let mut reg = ProtocolRegistry::new();
                        for (got, _, _) in reg.process_signal_stream(&wire, f) {
                            assert!(!MINE.contains(&got.as_str()), "{name} (btn {btn}) at {f} Hz decoded as {got}");
                        }
                    }
                }
            }
        }

        // Own output → reported as ourselves by the whole registry.
        let check = |name: &str, wire: &[LevelDuration], want: usize| {
            for f in reg0.get(name).unwrap().supported_frequencies() {
                let names: Vec<String> = ProtocolRegistry::new()
                    .process_signal_stream(wire, *f)
                    .into_iter()
                    .map(|(n, _, _)| n)
                    .collect();
                assert_eq!(names, vec![name.to_string(); want], "{name} at {f} Hz");
            }
        };
        let vag = reg0.get("VAG").unwrap();
        for (vag_type, key_idx, type_byte, btn, frames) in
            [(1u8, 0u8, 0x00u8, 0x20u8, 1usize), (2, NO_KEY, 0x00, 2, 1), (4, 2, 0xC0, 2, 2)]
        {
            let f = EncodeFields { vag_type, key_idx, type_byte, serial: 0x0A2F_9F3C, cnt: 0x5196, btn };
            let (_, k1, k2) = VagDecoder::build(&f).unwrap();
            for cmd in [0x01u8, 0x02, 0x04] {
                check("VAG", &vag.encode(&captured(&f, k1, k2), cmd).unwrap(), frames);
            }
        }
        let subaru = reg0.get("Subaru").unwrap();
        let mut x: u64 = 0x1234_5678_9ABC_DEF1;
        for _ in 0..50 {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            let cap = DecodedSignal {
                serial: None,
                button: None,
                counter: None,
                crc_valid: true,
                data: x,
                data_count_bit: 64,
                encoder_capable: true,
                extra: None,
                protocol_display_name: None,
            };
            for cmd in [0x01u8, 0x02, 0x04, 0x08] {
                check("Subaru", &subaru.encode(&cap, cmd).unwrap(), 1);
            }
        }
        // Chrysler V0: a payload-A capture (plaintext 5A 11 22 33 99 5E 66 77 88, Lock, seed 0x14).
        // The encoder's payload B reuses A's plaintext, which B's check rejects (as in the C), so
        // one frame per upload.
        let chrysler = reg0.get("Chrysler V0").unwrap();
        let cap = DecodedSignal {
            serial: Some(0x5A11_2233),
            button: Some(1),
            counter: Some(0x14),
            crc_valid: true,
            data: 0x2A65_2E1D_0CA6_6159,
            data_count_bit: 80,
            encoder_capable: true,
            extra: Some(0x48B7),
            protocol_display_name: None,
        };
        for cmd in [0x01u8, 0x02, 0x04] {
            check("Chrysler V0", &chrysler.encode(&cap, cmd).unwrap(), 1);
        }
    }

    /// Upload layout matches vag_encoder_build_type1/2/3_4 entry for entry.
    #[test]
    fn upload_layout_matches_c() {
        let up = VagDecoder::build_upload_type12(1, 0x0123_4567_89AB_CDEF, 0x2A55);
        assert_eq!(up.len(), 635);
        assert!(up[..440].chunks(2).all(|p| p[0].level && !p[1].level && p[0].duration_us == 300));
        assert_eq!((up[440].level, up[441].level), (false, true));
        assert_eq!((up[634].level, up[634].duration_us), (false, 6000));
        // prefix 0xAF3F: first bit 1 → HIGH,LOW
        assert!(up[442].level && !up[443].level);
        let up = VagDecoder::build_upload_type34(0, 0);
        assert_eq!(up.len(), 518);
        assert_eq!((up[90].level, up[90].duration_us, up[91].duration_us), (true, 1000, 500));
        assert_eq!((up[258].level, up[258].duration_us), (false, 10000));
    }

    /// Flipper-ARF ("VAG GROUP") writes a Type 1 button as its high nibble (`Btn: 2` for 0x20).
    /// Its file of KAT's own upload (the oracle's serialize) must load as that frame, not as one
    /// rebuilt around plaintext 0x02 (KAT 1.8.6: another Key1, no fields, no TX).
    #[test]
    fn arf_type1_key_file_loads() {
        use crate::protocols::flipper_key::testing::key;
        crate::protocols::keys::load_keystore_from_embedded();
        let dec = VagDecoder::new();
        let arf = key(
            "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 434420000\n\
             Preset: FuriHalSubGhzPresetOok650Async\nProtocol: VAG GROUP\nBit: 80\n\
             Key: 00 D7 11 F8 6F 28 58 A1\nKey2: 00 00 00 00 00 00 66 2A\nType: 1\nKeyIdx: 0\n\
             Cnt: 37778\nSerial: 4252471956\nBtn: 2\nBtnFlags: 0\n",
        );
        let d = dec.from_flipper_key(&arf).expect("loads");
        assert_eq!((d.data, d.serial, d.button, d.counter), (0x00D7_11F8_6F28_58A1, Some(0xFD77_9294), Some(0x20), Some(0x9392)));
        assert!(d.crc_valid && d.encoder_capable);
    }

    /// Key files: real decodes of every type round-trip with ProtoPirate's lines; the IMPORTS key
    /// file loads as the matching RAW capture's frame; undecrypted frames keep Key2/Type only;
    /// edited fields rebuild the frame; TX from a loaded key decodes back.
    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::{assert_round_trip, key};
        load_keys();
        let dec = VagDecoder::new();
        let Some(passat) = sub_frames("VAG/Vw_passat_b5_3bg.sub") else { return };
        for rel in ["VAG/Passat_2015.sub", "VAG/VAG_2.sub", "VAG/Golf4.sub"] {
            for f in sub_frames(rel).unwrap_or_default() {
                assert_round_trip(&dec, &f);
            }
        }
        // IMPORTS/VAG/Passat_b5_3bg.sub is a frame of Vw_passat_b5_3bg.sub.
        let file = key(
            "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 434390000\n\
             Preset: FuriHalSubGhzPresetOok650Async\nProtocol: VAG\nBit: 80\nKey: C0 9E BD 07 92 CA 7F C3\n\
             Key_2: 00 00 00 00 00 00 A7 2B\nType: 11011\n",
        );
        let l = dec.from_flipper_key(&file).unwrap();
        let raw = passat.iter().find(|f| f.data == 0xC09E_BD07_92CA_7FC3).unwrap();
        assert_eq!(&l, raw);
        assert_eq!(fields(&l), (0x0C21_578B, 2, 0x012B06, 4, 2));
        let text = assert_round_trip(&dec, &l);
        assert!(
            text.ends_with(
                "Protocol: VAG\nBit: 80\nKey: C0 9E BD 07 92 CA 7F C3\nKey2: 00 00 00 00 00 00 A7 2B\nType: 4\n\
                 Serial: 203511691\nBtn: 2\nCnt: 76550\nKeyIdx: 2\n"
            ),
            "{text}"
        );
        // An undecrypted frame: Key2 and Type only, and it reloads as such.
        let undec = VagDecoder::signal_from(0xC012_3456_789A_BCDE, 0x5A00, &VagDecoder::parse_data(3, 0xC012_3456_789A_BCDE, 0x5A00));
        let t = assert_round_trip(&dec, &undec);
        assert!(t.ends_with("Key2: 00 00 00 00 00 00 5A 00\nType: 3\n"), "{t}");
        // Edited counter: rebuilt like the C encoder (same key index), decodes to the new counter.
        let e = dec.from_flipper_key(&key(&text.replace("Cnt: 76550", "Cnt: 76600"))).unwrap();
        assert_eq!(fields(&e), (0x0C21_578B, 2, 76600, 4, 2));
        assert!(dec.from_flipper_key(&key(&text.replace("Bit: 80", "Bit: 64"))).is_none());
        // TX from the loaded key: counter + 1.
        let tx = decode_all(&dec.encode(&l, 0x02).unwrap());
        assert_eq!(fields(&tx[0]), (0x0C21_578B, 1, 0x012B07, 4, 2));
    }

    /// Oracle vectors (`#[ignore]`d): `KAT_KEY_VECTORS=<dir> cargo test vag::tests::key_vectors --
    /// --ignored` writes `vag_<n>.sub` / `.kat.txt` (a Type 1 and a Type 3/4 upload).
    #[test]
    #[ignore]
    fn key_vectors() {
        let Some(dir) = std::env::var_os("KAT_KEY_VECTORS") else { return };
        let dir = std::path::PathBuf::from(dir);
        load_keys();
        let fields = [
            EncodeFields { vag_type: 1, key_idx: 0, type_byte: 0x00, serial: 0xFD77_9294, cnt: 0x009392, btn: 0x20 },
            EncodeFields { vag_type: 4, key_idx: 2, type_byte: 0xC0, serial: 0x0C21_578B, cnt: 0x012B06, btn: 2 },
        ];
        for (n, f) in fields.iter().enumerate() {
            let (upload, _, _) = VagDecoder::build(f).unwrap();
            let upload = VagDecoder::merge_levels(upload);
            let d = decode_all(&upload).remove(0);
            let key = VagDecoder::new().to_flipper_key(&d).unwrap();
            crate::protocols::kia_v1::tests::write_key_vector(
                &dir,
                &format!("vag_{n}"),
                434_420_000,
                "FuriHalSubGhzPresetOok650Async",
                &upload,
                &key,
            );
        }
    }
}
