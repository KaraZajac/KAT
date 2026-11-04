//! BMW protocol decoder/encoder (Flipper-ARF "BMW").
//!
//! Ported from Flipper-ARF: `lib/subghz/protocols/bmw.c` / `bmw.h`
//! (`SUBGHZ_PROTOCOL_BMW_NAME = "BMW"`). doctor-firmware ships the same file; the two copies are
//! identical on the wire (timing, preamble, sync, 128-bit layout, gating, encoder upload) and differ
//! only in the Flipper custom-button UI plumbing: ARF maps UP→Lock, DOWN→Unlock, LEFT→Trunk with
//! `BMW_BTN_MAX 3`, doctor uses a generic button map with `BMW_BTN_MAX 2`. KAT follows ARF — its
//! explicit Lock/Unlock/Trunk mapping lines up 1:1 with KAT's generic button codes (and with ARF's
//! own button-label table). ProtoPirate has no BMW protocol.
//!
//! ARF added this protocol in October 2026 as the replacement for its BMW CAS4 decoder
//! (`bmw_cas4.c`, disabled in ARF's registry since September 2026); KAT followed and retired its
//! BMW CAS4 port. Every BMW chip is 40–160µs and anything ≥400µs is an end-of-frame gap here, so
//! slower (e.g. 500/1000µs) Manchester streams never enter this decoder.
//!
//! **Timing** (AM/OOK, 433.92 MHz only — `SubGhzProtocolFlag_433 | SubGhzProtocolFlag_AM`):
//! TE = 62µs. A pulse of 40–90µs is one chip (1T), 100–160µs is two chips (2T); any pulse ≥400µs is
//! the frame gap (`bmw_finish_gap`); every other duration resets. KAT's live demodulator debounces
//! pulses <40µs, so live capture of this 62µs protocol needs a clean, strong signal; raw `.sub`
//! imports are unaffected.
//!
//! **Line code**: Manchester chips, decoded pairwise (`bmw_add_chip`): chips (0,1) = bit 1
//! (low→high), (1,0) = bit 0 (high→low); (0,0)/(1,1) reset.
//!
//! **Decoder steps** (`BMWDecoderStep`): Reset → optional AGC burst (a 2T high, then lows) →
//! preamble of alternating 1T chips (32 ≤ counted lows ≤ 280) → the first 2T pulse enters Header
//! with its second chip as the first half of the first header bit → Header hunts (≤40 bits) for the
//! 16-bit sync 0x599E, or 0xA661 (inverted polarity: all following bits are then inverted) → Data
//! collects bits into a 128-bit register seeded with the 16 sync bits. A frame is evaluated at
//! exactly 128 bits; a ≥400µs gap may complete the last bit when its second chip merged into the gap.
//!
//! **Frame (128 bits, MSB first)** — `data` = high 64 bits, `extra` = low 64 bits:
//! ```text
//! data : [63:48] sync 0x599E | [47:16] serial (32) | [15:8] button byte | [7:0] fixed 0x89
//! extra: [63:48] counter[16:1] | [47] counter[0] | [46:0] hop (47-bit rolling code)
//! ```
//! Button byte = `0x01 | (xx << 3)` (template 0x01 under mask 0xE7); xx: 1=Unlock, 2=Lock, 3=Trunk.
//! `DecodedSignal.button` carries xx (the protocol's own code), `counter` the LOW 16 bits of the
//! 17-bit counter (the full counter stays in `extra`), `data_count_bit` = 128.
//!
//! **Gating** (`bmw_fields_valid`, all required): preamble 32–280 chips, sync 0x599E/0xA661 found
//! within 40 header bits, exactly 128 bits, button byte matching the 0x01/0xE7 template with xx≠0,
//! and fixed byte 0x89. There is no CRC; `crc_valid` reports these structural checks.
//!
//! **Encoder** (`bmw_get_upload`): the hop is encrypted and no key is available, so — exactly as the
//! C — it rebuilds the payload from the captured fields (serial, counter and hop unchanged, NO
//! counter bump) and only re-stamps the button when a Lock/Unlock/Trunk command is requested
//! (any other command keeps the captured button). Upload: 212 preamble "1" bits, the 128 frame bits
//! (bit 1 → L,H; bit 0 → H,L; 62µs chips, adjacent levels merged), then a 200ms low gap; the C
//! yields that upload `BMW_REPEAT_DEFAULT` = 3 times, so the KAT waveform holds 3 copies.
//!
//! **Deviations from C**: none on the wire. The C's per-frame `te_last` bookkeeping is unused and
//! omitted; the Flipper custom-button/global-override plumbing is replaced by KAT's generic button
//! mapping.
//!
//! **Key files**: `bmw.c`'s serialize is the generic one, and the two references' generic writers
//! differ. Flipper-ARF writes only `Protocol: BMW`, `Bit: 128` and `Key` (the high 64 bits), so an
//! ARF-saved key loses the counter and hop; doctor-firmware's writer adds the decoded fields and the
//! second word. KAT writes doctor's lines — `Serial`, `Btn` (xx), `Cnt` (17-bit), `ValidationField`
//! (the fixed 0x89), `TE` (62), `Key_2` (the low 64 bits) and `Key_3` (the 47-bit hop), each
//! omitted when zero, as doctor does — so its files reload completely; ARF reads only Bit/Key from
//! them. Loading follows `subghz_protocol_decoder_bmw_deserialize`: exactly 128 bits, `Key_2` (or
//! doctor's `Key2`) as the low half (zero when absent, as ARF's own load leaves it), an
//! inverted-sync payload flipped back (`bmw_normalize_stored_payload`), fields parsed from the
//! payload (the file's Serial/Btn/Cnt are not read back, as in the C). The C loads any payload;
//! `crc_valid` reports the decoder's structural gate on it.

use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "BMW",
    decoder: "BMW",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "BMW",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// BMW buttons.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x1 => "Unlock",
        0x2 => "Lock",
        0x3 => "Trunk",
        _ => "Other",
    })
}

const TE_US: u32 = 62;
const TE1_MIN_US: u32 = 40;
const TE1_MAX_US: u32 = 90;
const TE2_MIN_US: u32 = 100;
const TE2_MAX_US: u32 = 160;
/// `subghz_protocol_bmw_const.te_delta` (informational: the decoder uses the TE1/TE2 windows).
const TE_DELTA: u32 = 25;
const GAP_US: u32 = 400;
const PREAMBLE_MIN: u16 = 32;
const PREAMBLE_MAX: u16 = 280;
const PREAMBLE_BITS: usize = 212;
const SYNC: u16 = 0x599E;
const SYNC_INV: u16 = 0xA661;
const HEADER_HUNT_MAX: u16 = 40;
const PAYLOAD_BITS: usize = 128;
const FIXED_VALUE: u8 = 0x89;
const CNT_MASK: u32 = 0x1FFFF;
const HOP_MASK: u64 = 0x7FFF_FFFF_FFFF;
const BTN_TEMPLATE: u8 = 0x01;
const BTN_TEMPLATE_MASK: u8 = 0xE7;
const BTN_XX_MASK: u8 = 0x3;
const REPEAT_GAP_US: u32 = 200_000;
const REPEAT_DEFAULT: usize = 3;

// Protocol button codes (the 2-bit `xx` field of the button byte).
const BTN_UNLOCK: u8 = 0x1;
const BTN_LOCK: u8 = 0x2;
const BTN_TRUNK: u8 = 0x3;

// KAT generic button codes (encode() argument).
const KAT_BTN_LOCK: u8 = 0x01;
const KAT_BTN_UNLOCK: u8 = 0x02;
const KAT_BTN_TRUNK: u8 = 0x04;

/// Decoder steps (matches `BMWDecoderStep`).
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Agc,
    WaitPreambleHigh,
    PreambleLow,
    PreambleHigh,
    Header,
    Data,
}

/// BMW decoder (matches `SubGhzProtocolDecoderBMW`).
pub struct BmwDecoder {
    step: DecoderStep,
    /// Low 64 bits of the 128-bit shift register (`decoder.decode_data`).
    decode_data: u64,
    decode_count_bit: usize,
    /// High 64 bits of the 128-bit shift register (`decode_head`).
    decode_head: u64,
    preamble_count: u16,
    header_bits: u16,
    sync_reg: u16,
    /// Pending first chip of a Manchester pair (`pair_chip`, -1 = None).
    pair_chip: Option<bool>,
    invert: bool,
}

impl BmwDecoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            decode_data: 0,
            decode_count_bit: 0,
            decode_head: 0,
            preamble_count: 0,
            header_bits: 0,
            sync_reg: 0,
            pair_chip: None,
            invert: false,
        }
    }

    /// `bmw_chips`: 1 for a 1T pulse, 2 for a 2T pulse, 0 otherwise.
    fn chips(duration: u32) -> u8 {
        if (TE1_MIN_US..=TE1_MAX_US).contains(&duration) {
            1
        } else if (TE2_MIN_US..=TE2_MAX_US).contains(&duration) {
            2
        } else {
            0
        }
    }

    /// `bmw_reset_soft`.
    fn reset_soft(&mut self) {
        self.step = DecoderStep::Reset;
        self.decode_data = 0;
        self.decode_count_bit = 0;
        self.decode_head = 0;
        self.preamble_count = 0;
        self.header_bits = 0;
        self.sync_reg = 0;
        self.pair_chip = None;
        self.invert = false;
    }

    /// `subghz_protocol_blocks_add_to_128_bit`: shift one bit into the 128-bit (head, data) register.
    fn add_to_128_bit(&mut self, bit: bool) {
        self.decode_count_bit += 1;
        if self.decode_count_bit > 64 {
            self.decode_head = (self.decode_head << 1) | (self.decode_data >> 63);
        }
        self.decode_data = (self.decode_data << 1) | (bit as u64);
    }

    /// `bmw_wire_xx`: the 2-bit button code from the button byte.
    fn wire_xx(btn_byte: u8) -> u8 {
        (btn_byte >> 3) & BTN_XX_MASK
    }

    /// `bmw_btn_byte_from_wire_xx`.
    fn btn_byte_from_wire_xx(xx: u8) -> u8 {
        BTN_TEMPLATE | ((xx & BTN_XX_MASK) << 3)
    }

    /// `bmw_wire_cnt`: the 17-bit counter, split across lo[63:48] (high 16 bits) and lo[47].
    fn wire_cnt(lo: u64) -> u32 {
        ((((lo >> 48) & 0xFFFF) << 1) | ((lo >> 47) & 1)) as u32
    }

    /// `bmw_fields_valid` (on the high 64 bits).
    fn fields_valid(hi: u64) -> bool {
        let sync = (hi >> 48) as u16;
        let btn_byte = ((hi >> 8) & 0xFF) as u8;
        let fixed = (hi & 0xFF) as u8;
        sync == SYNC
            && (btn_byte & BTN_TEMPLATE_MASK) == BTN_TEMPLATE
            && Self::wire_xx(btn_byte) != 0
            && fixed == FIXED_VALUE
    }

    /// `bmw_parse_fields`. Returns (serial, button xx, 17-bit counter, 47-bit hop).
    fn parse_fields(hi: u64, lo: u64) -> (u32, u8, u32, u64) {
        let btn_byte = ((hi >> 8) & 0xFF) as u8;
        let serial = (hi >> 16) as u32;
        let btn = Self::wire_xx(btn_byte);
        let cnt = Self::wire_cnt(lo) & CNT_MASK;
        let hop = lo & HOP_MASK;
        (serial, btn, cnt, hop)
    }

    /// `bmw_rebuild_payload`. Returns (hi, lo).
    fn rebuild_payload(serial: u32, btn: u8, cnt: u32, hop: u64) -> (u64, u64) {
        let btn_byte = Self::btn_byte_from_wire_xx(btn);
        let cnt = cnt & CNT_MASK;
        let hi = ((SYNC as u64) << 48)
            | ((serial as u64) << 16)
            | ((btn_byte as u64) << 8)
            | FIXED_VALUE as u64;
        let lo = (((cnt >> 1) as u64) << 48) | (((cnt & 1) as u64) << 47) | (hop & HOP_MASK);
        (hi, lo)
    }

    /// `bmw_try_callback`: evaluate a complete 128-bit frame.
    fn try_callback(&mut self) -> Option<DecodedSignal> {
        if self.decode_count_bit != PAYLOAD_BITS {
            return None;
        }
        let hi = self.decode_head;
        let lo = self.decode_data;
        if !Self::fields_valid(hi) {
            self.reset_soft();
            return None;
        }
        self.reset_soft();
        Some(Self::frame_signal(hi, lo))
    }

    /// The decode of a 128-bit payload (`bmw_parse_fields`), as the decoder reports it.
    fn frame_signal(hi: u64, lo: u64) -> DecodedSignal {
        let (serial, btn, cnt, _hop) = Self::parse_fields(hi, lo);
        DecodedSignal {
            serial: Some(serial),
            button: Some(btn),
            // 17-bit counter: low 16 bits here; the full value is preserved in `extra`.
            counter: Some((cnt & 0xFFFF) as u16),
            // No CRC: sync + fixed 0x89 + button template validate the frame (always true for a
            // frame the decoder reports; a loaded key is not gated, as in the C).
            crc_valid: Self::fields_valid(hi),
            data: hi,
            data_count_bit: PAYLOAD_BITS,
            encoder_capable: true,
            extra: Some(lo),
            protocol_display_name: None,
        }
    }

    /// `bmw_seed_payload_sync`: restart the 128-bit register with the 16 sync bits.
    fn seed_payload_sync(&mut self) {
        self.decode_data = 0;
        self.decode_count_bit = 0;
        self.decode_head = 0;
        for i in (0..16).rev() {
            self.add_to_128_bit((SYNC >> i) & 1 != 0);
        }
    }

    /// `bmw_add_logical_bit`.
    fn add_logical_bit(&mut self, bit: bool) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::Header => {
                self.sync_reg = (self.sync_reg << 1) | (bit as u16);
                self.header_bits += 1;
                if self.sync_reg == SYNC {
                    self.invert = false;
                    self.seed_payload_sync();
                    self.step = DecoderStep::Data;
                } else if self.sync_reg == SYNC_INV {
                    self.invert = true;
                    self.seed_payload_sync();
                    self.step = DecoderStep::Data;
                } else if self.header_bits >= HEADER_HUNT_MAX {
                    self.reset_soft();
                }
                None
            }
            DecoderStep::Data => {
                let value = if self.invert { !bit } else { bit };
                if self.decode_count_bit >= PAYLOAD_BITS {
                    return self.try_callback();
                }
                self.add_to_128_bit(value);
                if self.decode_count_bit == PAYLOAD_BITS {
                    return self.try_callback();
                }
                None
            }
            _ => {
                self.reset_soft();
                None
            }
        }
    }

    /// `bmw_add_chip`: pair chips into Manchester bits.
    fn add_chip(&mut self, chip: bool) -> Option<DecodedSignal> {
        let first = match self.pair_chip {
            None => {
                self.pair_chip = Some(chip);
                return None;
            }
            Some(first) => first,
        };
        self.pair_chip = None;
        match (first, chip) {
            (false, true) => self.add_logical_bit(true),
            (true, false) => self.add_logical_bit(false),
            _ => {
                self.reset_soft();
                None
            }
        }
    }

    /// `bmw_enter_header`: the 2T pulse that ends the preamble; its second chip starts the header.
    fn enter_header(&mut self, level: bool) {
        self.step = DecoderStep::Header;
        self.header_bits = 0;
        self.sync_reg = 0;
        self.invert = false;
        self.decode_data = 0;
        self.decode_count_bit = 0;
        self.decode_head = 0;
        self.pair_chip = Some(level);
    }

    /// `bmw_finish_gap`: a ≥400µs pulse ends the frame; its first chip may complete the last bit.
    fn finish_gap(&mut self, level: bool) -> Option<DecodedSignal> {
        let mut result = None;
        if self.step == DecoderStep::Header || self.step == DecoderStep::Data {
            if self.pair_chip == Some(false) && level {
                result = self.add_logical_bit(true);
            } else if self.pair_chip == Some(true) && !level {
                result = self.add_logical_bit(false);
            }
            self.pair_chip = None;
            let tail = self.try_callback();
            result = result.or(tail);
        }
        self.reset_soft();
        result
    }

    /// Map a KAT generic button command to the BMW `xx` code (`bmw_get_btn_code`, ARF mapping:
    /// UP=Lock, DOWN=Unlock, LEFT=Trunk); anything else keeps the captured button.
    fn map_button(button: u8, original: u8) -> u8 {
        match button {
            KAT_BTN_LOCK => BTN_LOCK,
            KAT_BTN_UNLOCK => BTN_UNLOCK,
            KAT_BTN_TRUNK => BTN_TRUNK,
            _ => original,
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

    /// `bmw_emit_bit`: bit 1 → (L,H), bit 0 → (H,L), one TE each, merged.
    fn emit_bit(signal: &mut Vec<LevelDuration>, bit: bool) {
        if bit {
            Self::enc_add_level(signal, false, TE_US);
            Self::enc_add_level(signal, true, TE_US);
        } else {
            Self::enc_add_level(signal, true, TE_US);
            Self::enc_add_level(signal, false, TE_US);
        }
    }

    /// One upload copy (`bmw_get_upload`): preamble, 128 frame bits, 200ms low gap.
    fn append_frame(signal: &mut Vec<LevelDuration>, hi: u64, lo: u64, preamble_bits: usize) {
        for _ in 0..preamble_bits {
            Self::emit_bit(signal, true);
        }
        for word in [hi, lo] {
            for bit in (0..64).rev() {
                Self::emit_bit(signal, (word >> bit) & 1 != 0);
            }
        }
        Self::enc_add_level(signal, false, REPEAT_GAP_US);
    }
}

impl ProtocolDecoder for BmwDecoder {
    fn name(&self) -> &'static str {
        "BMW"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_US,
            te_long: TE_US * 2,
            te_delta: TE_DELTA,
            min_count_bit: PAYLOAD_BITS,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        // SubGhzProtocolFlag_433 only.
        &[433_920_000]
    }

    fn reset(&mut self) {
        self.reset_soft();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        if duration >= GAP_US {
            return self.finish_gap(level);
        }

        let n = Self::chips(duration);
        if n == 0 {
            self.reset_soft();
            return None;
        }

        match self.step {
            DecoderStep::Reset => {
                if level && n == 2 {
                    self.step = DecoderStep::Agc;
                } else if level && n == 1 {
                    self.preamble_count = 0;
                    self.step = DecoderStep::PreambleLow;
                } else if !level && n == 1 {
                    self.preamble_count = 0;
                    self.step = DecoderStep::PreambleHigh;
                }
            }

            DecoderStep::Agc => {
                if !level {
                    self.step = DecoderStep::WaitPreambleHigh;
                } else if n == 1 {
                    self.preamble_count = 0;
                    self.step = DecoderStep::PreambleLow;
                } else {
                    self.reset_soft();
                }
            }

            DecoderStep::WaitPreambleHigh => {
                if level && n == 1 {
                    self.preamble_count = 0;
                    self.step = DecoderStep::PreambleLow;
                } else if level && n == 2 {
                    self.step = DecoderStep::Agc;
                }
                // !level (1T or 2T): keep waiting.
            }

            DecoderStep::PreambleLow => {
                if !level && n == 1 {
                    self.preamble_count += 1;
                    if self.preamble_count > PREAMBLE_MAX {
                        self.reset_soft();
                    } else {
                        self.step = DecoderStep::PreambleHigh;
                    }
                } else if !level && n == 2 && self.preamble_count >= PREAMBLE_MIN {
                    self.enter_header(false);
                } else {
                    self.reset_soft();
                }
            }

            DecoderStep::PreambleHigh => {
                if level && n == 1 {
                    self.step = DecoderStep::PreambleLow;
                } else if level && n == 2 && self.preamble_count >= PREAMBLE_MIN {
                    self.enter_header(true);
                } else {
                    self.reset_soft();
                }
            }

            DecoderStep::Header | DecoderStep::Data => {
                for _ in 0..n {
                    if self.step != DecoderStep::Header && self.step != DecoderStep::Data {
                        break;
                    }
                    if let Some(decoded) = self.add_chip(level) {
                        return Some(decoded);
                    }
                }
            }
        }

        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != PAYLOAD_BITS {
            return None;
        }
        let mut hi = decoded.data;
        let mut lo = decoded.extra?;
        // bmw_normalize_stored_payload: a frame stored with the inverted sync is flipped back.
        if (hi >> 48) as u16 == SYNC_INV {
            hi = !hi;
            lo = !lo;
        }
        let (serial, captured_btn, cnt, hop) = Self::parse_fields(hi, lo);
        let btn = Self::map_button(button, captured_btn);
        let (hi, lo) = Self::rebuild_payload(serial, btn, cnt, hop);

        let mut signal =
            Vec::with_capacity((PREAMBLE_BITS * 2 + PAYLOAD_BITS * 2 + 1) * REPEAT_DEFAULT);
        for _ in 0..REPEAT_DEFAULT {
            Self::append_frame(&mut signal, hi, lo, PREAMBLE_BITS);
        }
        Some(signal)
    }

    /// `subghz_protocol_decoder_bmw_deserialize` (see the module doc, "Key files").
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != PAYLOAD_BITS {
            return None;
        }
        let mut hi = key.key_u64()?;
        let mut lo = key.hex_field("Key_2").or_else(|| key.hex_field("Key2")).unwrap_or(0);
        if (hi >> 48) as u16 == SYNC_INV {
            hi = !hi;
            lo = !lo;
        }
        Some(Self::frame_signal(hi, lo))
    }

    /// doctor-firmware's `subghz_block_generic_serialize` of a BMW decode (see "Key files").
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != PAYLOAD_BITS {
            return None;
        }
        let (hi, lo) = (decoded.data, decoded.extra?);
        let (serial, btn, cnt, hop) = Self::parse_fields(hi, lo);
        let mut key = FlipperKey::for_protocol("BMW", PAYLOAD_BITS, hi);
        if serial != 0 || btn != 0 || cnt != 0 {
            key.push_u32("Serial", serial);
            key.push_u32("Btn", btn.into());
            key.push_u32("Cnt", cnt);
        }
        if hi & 0xFF != 0 {
            key.push_u32("ValidationField", (hi & 0xFF) as u32);
        }
        key.push_u32("TE", TE_US);
        if lo != 0 {
            key.push_hex("Key_2", lo, 8);
        }
        if hop != 0 {
            key.push_hex("Key_3", hop, 8);
        }
        Some(key)
    }
}

/// Oracle test vectors for the key-file support of the Flipper-ARF / doctor-firmware ports:
/// writes `<dir>/<name>.sub`, a Flipper RAW file of `upload` sent twice with a 20 ms LOW gap
/// (HIGH positive, LOW negative, adjacent levels merged, ≤512 values per line), and
/// `<dir>/<name>.kat.txt`, KAT's key file `key` with the same Frequency/Preset.
#[cfg(test)]
pub(super) fn write_key_vector(
    dir: &std::path::Path,
    name: &str,
    frequency: u32,
    preset: &str,
    upload: &[LevelDuration],
    key: &FlipperKey,
) {
    crate::protocols::flipper_key::testing::write_vector(dir, name, frequency, preset, upload, key);
}

/// Where [write_key_vector] writes (`KAT_KEY_VECTORS`, default `<tmp>/kat_key_vectors`).
#[cfg(test)]
pub(super) fn key_vector_dir() -> std::path::PathBuf {
    std::env::var_os("KAT_KEY_VECTORS")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| std::env::temp_dir().join("kat_key_vectors"))
}

impl Default for BmwDecoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::ProtocolRegistry;

    const SERIAL: u32 = 0x1234_5678;
    const COUNTER: u32 = 0x1_A5C3; // 17 bits, exercises the split counter[0] bit
    const HOP: u64 = 0x5A5A_1234_ABCD & HOP_MASK;

    fn captured(btn: u8) -> DecodedSignal {
        let (hi, lo) = BmwDecoder::rebuild_payload(SERIAL, btn, COUNTER, HOP);
        DecodedSignal {
            serial: Some(SERIAL),
            button: Some(btn),
            counter: Some((COUNTER & 0xFFFF) as u16),
            crc_valid: true,
            data: hi,
            data_count_bit: PAYLOAD_BITS,
            encoder_capable: true,
            extra: Some(lo),
            protocol_display_name: None,
        }
    }

    /// Build one on-air frame (preamble + 128 bits + gap) for an arbitrary (hi, lo) payload.
    fn frame(hi: u64, lo: u64, preamble_bits: usize) -> Vec<LevelDuration> {
        let mut signal = Vec::new();
        BmwDecoder::append_frame(&mut signal, hi, lo, preamble_bits);
        signal
    }

    fn decode_all(signal: &[LevelDuration], invert: bool) -> Vec<DecodedSignal> {
        let mut dec = BmwDecoder::new();
        signal
            .iter()
            .filter_map(|ld| dec.feed(ld.level ^ invert, ld.duration_us))
            .collect()
    }

    #[test]
    fn bmw_field_packing_roundtrip() {
        let (hi, lo) = BmwDecoder::rebuild_payload(SERIAL, BTN_TRUNK, COUNTER, HOP);
        assert_eq!(
            hi, 0x599E_1234_5678_1989,
            "sync | serial | button byte 0x19 | 0x89"
        );
        assert!(BmwDecoder::fields_valid(hi));
        assert_eq!(
            BmwDecoder::parse_fields(hi, lo),
            (SERIAL, BTN_TRUNK, COUNTER, HOP)
        );
    }

    #[test]
    fn bmw_encode_decode_roundtrip() {
        let encoder = BmwDecoder::new();
        for (kat_btn, want) in [
            (KAT_BTN_LOCK, BTN_LOCK),
            (KAT_BTN_UNLOCK, BTN_UNLOCK),
            (KAT_BTN_TRUNK, BTN_TRUNK),
            (0x08, BTN_UNLOCK), // unmapped command keeps the captured button
        ] {
            let signal = encoder
                .encode(&captured(BTN_UNLOCK), kat_btn)
                .expect("encode");
            let decodes = decode_all(&signal, false);
            assert_eq!(decodes.len(), REPEAT_DEFAULT, "one decode per upload copy");
            let (want_hi, want_lo) = BmwDecoder::rebuild_payload(SERIAL, want, COUNTER, HOP);
            for d in &decodes {
                assert_eq!(d.serial, Some(SERIAL));
                assert_eq!(d.button, Some(want));
                assert_eq!(d.counter, Some((COUNTER & 0xFFFF) as u16));
                assert_eq!(d.data, want_hi);
                assert_eq!(
                    d.extra,
                    Some(want_lo),
                    "counter + hop must survive unchanged"
                );
                assert_eq!(d.data_count_bit, PAYLOAD_BITS);
                assert!(d.crc_valid && d.encoder_capable);
            }
        }
    }

    /// Inverted capture polarity: the header hunt finds 0xA661 and un-inverts the payload.
    #[test]
    fn bmw_decodes_inverted_polarity() {
        let signal = BmwDecoder::new()
            .encode(&captured(BTN_LOCK), KAT_BTN_LOCK)
            .unwrap();
        let decodes = decode_all(&signal, true);
        assert_eq!(decodes.len(), REPEAT_DEFAULT);
        assert!(decodes
            .iter()
            .all(|d| d.data == captured(BTN_LOCK).data && d.extra == captured(BTN_LOCK).extra));
    }

    /// The last bit is a 0 (H,L) whose low chip merges into the gap: finish_gap must complete it.
    #[test]
    fn bmw_gap_completes_last_bit() {
        let (hi, lo) = BmwDecoder::rebuild_payload(SERIAL, BTN_LOCK, 0x1_0000, 0x7FFF_FFFF_FFFE);
        assert_eq!(lo & 1, 0);
        let decodes = decode_all(&frame(hi, lo, PREAMBLE_BITS), false);
        assert_eq!(decodes.len(), 1);
        assert_eq!(decodes[0].extra, Some(lo));
    }

    #[test]
    fn bmw_rejects_corrupted_frames() {
        let (hi, lo) = BmwDecoder::rebuild_payload(SERIAL, BTN_LOCK, COUNTER, HOP);
        assert_eq!(decode_all(&frame(hi, lo, PREAMBLE_BITS), false).len(), 1);
        // Fixed byte 0x89 → 0x88.
        assert!(decode_all(&frame(hi ^ 0x01, lo, PREAMBLE_BITS), false).is_empty());
        // Button byte outside the 0x01/0xE7 template (bit 1 set).
        assert!(decode_all(&frame(hi ^ 0x0200, lo, PREAMBLE_BITS), false).is_empty());
        // Template intact but xx == 0.
        let (hi0, _) = BmwDecoder::rebuild_payload(SERIAL, 0, COUNTER, HOP);
        assert!(decode_all(&frame(hi0, lo, PREAMBLE_BITS), false).is_empty());
        // Preamble too short (< 32 counted lows) or too long (> 280).
        assert!(decode_all(&frame(hi, lo, 20), false).is_empty());
        assert!(decode_all(&frame(hi, lo, 300), false).is_empty());
    }

    /// BMW frames decode only as BMW through the registry, and a 500/1000µs Manchester-rate stream
    /// (e.g. the retired BMW CAS4 format) is all gaps to the BMW decoder.
    #[test]
    fn bmw_frames_decode_only_as_bmw() {
        let signal = BmwDecoder::new()
            .encode(&captured(BTN_TRUNK), KAT_BTN_TRUNK)
            .unwrap();
        let mut reg = ProtocolRegistry::new();
        let names: Vec<String> = reg
            .process_signal_stream(&signal, 433_920_000)
            .into_iter()
            .map(|(n, _, _)| n)
            .collect();
        assert_eq!(
            names,
            vec!["BMW"; REPEAT_DEFAULT],
            "BMW frames must decode only as BMW"
        );

        // A 500/1000µs Manchester-like stream is all gaps to the BMW decoder.
        let mut dec = BmwDecoder::new();
        for i in 0..400u32 {
            let dur = if i % 3 == 0 { 1000 } else { 500 };
            assert!(dec.feed(i % 2 == 0, dur).is_none());
            assert_eq!(dec.step, DecoderStep::Reset);
        }
    }

    /// Key files: doctor-firmware's lines round-trip every field, Flipper-ARF's (Key only) load
    /// with a zero low half, an inverted-sync key is normalised, and TX from a loaded key decodes.
    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::{assert_round_trip, key};
        let dec = BmwDecoder::new();
        let wire = dec.encode(&captured(BTN_LOCK), KAT_BTN_LOCK).unwrap();
        let decoded = decode_all(&wire, false).remove(0);
        let text = assert_round_trip(&dec, &decoded);
        let (hi, lo) = BmwDecoder::rebuild_payload(SERIAL, BTN_LOCK, COUNTER, HOP);
        assert_eq!(hi, 0x599E_1234_5678_1189, "sync | serial | button byte 0x11 (Lock) | 0x89");
        assert_eq!(decoded.extra, Some(lo));
        assert!(
            text.ends_with(&format!(
                "Protocol: BMW\nBit: 128\nKey: 59 9E 12 34 56 78 11 89\nSerial: 305419896\nBtn: 2\n\
                 Cnt: {COUNTER}\nValidationField: 137\nTE: 62\nKey_2: {}\nKey_3: {}\n",
                hex8(lo),
                hex8(HOP)
            )),
            "{text}"
        );

        // Flipper-ARF's file (its generic writer stops after Key): counter and hop are zero.
        let arf = key("Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                       Preset: FuriHalSubGhzPresetOok650Async\nProtocol: BMW\nBit: 128\n\
                       Key: 59 9E 12 34 56 78 11 89\n");
        let d = dec.from_flipper_key(&arf).unwrap();
        assert_eq!((d.serial, d.button, d.counter, d.extra, d.crc_valid), (Some(SERIAL), Some(BTN_LOCK), Some(0), Some(0), true));
        // A payload stored with the inverted sync is flipped back; a wrong bit count is refused.
        let mut inverted = arf.clone();
        inverted.key = !hi;
        inverted.key_bytes = (!hi).to_be_bytes().to_vec();
        inverted.push_hex("Key_2", !lo, 8);
        let d = dec.from_flipper_key(&inverted).unwrap();
        assert_eq!((d.data, d.extra), (hi, Some(lo)));
        let mut short = arf.clone();
        short.bits = 64;
        assert!(dec.from_flipper_key(&short).is_none());

        // TX from the reloaded decode.
        let reloaded = dec.from_flipper_key(&key(&text)).unwrap();
        let tx = decode_all(&dec.encode(&reloaded, KAT_BTN_UNLOCK).unwrap(), false);
        assert_eq!((tx[0].serial, tx[0].button, tx[0].extra), (Some(SERIAL), Some(BTN_UNLOCK), Some(lo)));
    }

    fn hex8(v: u64) -> String {
        v.to_be_bytes().iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(" ")
    }

    /// Oracle vectors (`KAT_KEY_VECTORS=<dir> cargo test bmw_key_vectors -- --ignored`).
    #[test]
    #[ignore]
    fn bmw_key_vectors() {
        let dec = BmwDecoder::new();
        for (n, btn) in [(1, BTN_LOCK), (2, BTN_TRUNK)] {
            let wire = dec.encode(&captured(btn), 0).unwrap();
            let one = &wire[..wire.len() / REPEAT_DEFAULT];
            let decoded = decode_all(one, false).remove(0);
            let key = dec.to_flipper_key(&decoded).unwrap();
            write_key_vector(&key_vector_dir(), &format!("bmw_{n}"), 433_920_000, "FuriHalSubGhzPresetOok650Async", one, &key);
        }
    }
}
