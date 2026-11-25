//! BMW/Mini V3 (FoxFW `bmw_v3.c`, `BMW_V3_PROTOCOL_NAME = "BMW/Mini V3"`): 2-FSK BMW / Mini key fob,
//! 0xCC header + 64 bits + an XOR check byte — **the remote of BMW's CAS3 (E9x era)**, read from
//! the CAS3 module's own firmware (below) — **decode only**, as the reference (its encoder is NULL).
//!
//! Ported from FoxFW (itself a port of the firmware's `bmw_v3.c` via KeeLoq-RX `arke_bmw.py`). It
//! is not KAT's "BMW" module (`bmw.rs`, Flipper-ARF's 62 µs AM frame with the 0x599E sync), nor
//! KAT's "BMW CAS3" (`bmw_cas3.rs`, FOBworks' PPM format, named after CAS3 by FOBworks; the CAS3's
//! own firmware has no radio pulse decoder and checks this 0xCC frame as its key telegram).
//!
//! **Air format** (flags 433 | 315 | AM | FM; meaningful on 2-FSK captures): a LOW of more than
//! 350 µs is the gap before a frame. A half-period accumulator (`te_last`) as in BMW V2: a long
//! pulse is one bit, two short pulses are one bit, a long pulse with a short pending completes a
//! bit and leaves the third half pending; the bit's value is `level ? 0 : 1` of the completing
//! pulse. The first pulse after the gap only primes the accumulator. Measured on the 2004 E60
//! capture in IMPORTS (`BMW/more/BmwE60_2004_868_FM_cl.sub`): half bit 96 µs, bit 190 µs (≈5.3
//! kbit/s); each copy is a ~500 µs LOW then 0xCC at once (no preamble bits), 80 bits in all, the
//! copies 16.5 ms apart (9 in a burst).
//!
//! **Decoder** (`subghz_protocol_decoder_bmw_v3_feed`): the header is located by sliding an 8-bit
//! window over the bits (up to 32 bits; the window needs at least 8) until its low byte is 0xCC;
//! the accumulator is then cleared and the next 64 bits are the payload. A pulse that is neither
//! short nor long resets (and starts a new gap wait when it is a long LOW).
//!
//! **Frame** (on air, MSB first): `CC | b1 | b2 b3 | b4 b5 b6 b7 | b8 | b9`.
//! - `b1`: type in bits 7..5 (000 a key; 010 and 100 a second class of transmitter, two learned
//!   slots, checked against its own table), bit 4, and the button nibble in bits 3..0: 1, 2, 4
//!   or 8, and 0 for the telegram sent when the button is released.
//! - `b2 b3`: the key's 16-bit ID (the CAS3 looks it up among its 10 learned keys).
//! - `b4..b7`: the 32-bit authenticator. `b8`: the low byte of the key's 32-bit counter.
//! - `b9`: the XOR of `b1..b8`. On every E60 frame recorded to its end, `b9` matches.
//!
//! FoxFW stops after `b8` (64 bits), has no check, reads its "counter" from bits 31..16 (inside
//! the authenticator) and takes the button whitelist 0x02 / 0x04 / 0x08 / 0x10 — without
//! button 1, which a CAS3 acts on, and with bit 4, which the CAS3 does not decode.
//!
//! **What the CAS3 does with it** (CAS3 0L15Y, MC9S12XDP512, the specialist's P-flash dump; the
//! scripts and notes are in `REFERENCES/tools/scripts/bmw-cas-porsche-2026-11-24`). The remote
//! receiver in the diversity (antenna) module demodulates the frame and passes its bytes to the
//! CAS3 as a UART stream on the FBD line (SCI1, 28.8 kbit/s; the WUP line wakes the module), with
//! one more byte of its own that the CAS3 keeps as a minimum / maximum (a signal level):
//! - `FE:87AB` waits for 0xCC and collects 11 bytes; `FE:8BE6` requires `b1 ^ … ^ b9 == 0` and
//!   re-aligns on the next 0xCC when it fails; an identical repeat of the last frame is dropped.
//! - `FC:A953` / `FC:BCEA`: the type and `b2 b3` against the learned IDs (or the 0x10FF test ID).
//! - `FC:A7ED`: the key's stored 32-bit counter with its low byte replaced by `b8`, plus 0x100 when
//!   that is not above the stored value; the authenticator is HITAG2 (`FC:9A7B` + `FC:9C4D`, see
//!   [`super::hitag2::bmw_cas3_authenticator`]: uid ‖ 48-bit key per key, IV = that counter),
//!   compared with `b4..b7` (state 3, `FE:82D7`). On a mismatch the counter is raised by 0x100
//!   and tried again while it stays within 0x6D0 of the stored one, else the key is marked out of
//!   sync. A match stores the counter and acts on the button nibble (`FE:837D`).
//!
//! FoxFW's comment calls the rolling code AES-encrypted; on CAS3 it is HITAG2. The E60 capture is a
//! 2004 car (CAS2): its `b4 b5` keep fixed bits (`1011 0x1x 011x x110` over 7 presses), so a CAS2
//! key fills those bytes otherwise; its `b9` is the same XOR.
//!
//! **KAT deviations from FoxFW**: KAT reads `b9` too and reports the frame on it (or when the
//! frame ends, at the end of the input included — [`ProtocolDecoder::flush`]): `crc_valid` when
//! `b9` matches (all 8 bits, or the 7 that come through when the last bit runs into the gap after
//! the frame, as on most E60 copies). A checked frame is reported for any CAS3 type (`b1` bits
//! 7..5 = 000, 010 or 100); an unchecked one keeps FoxFW's whitelist, so every frame FoxFW reports
//! is still reported. `counter` is `b8`, not FoxFW's bits 31..16.
//!
//! **KAT fields**: `data` = the 64-bit payload `b1..b8`, `data_count_bit` = 64, `button` = `b1`,
//! `serial` = `b2 b3`, `counter` = `b8`, `crc_valid` = the `b9` check.
//!
//! **Keystore (verify / counter recovery / encode)** — like JLR, Audi BCM2 and the PSA older code,
//! KAT ships no BMW keys and reads them only from the owner's own module. A fob's HITAG2 secrets go
//! in a `bmw_cas3_keys` file (`<ID> <UID> <KEY48> [COUNTER] [name]`, from the module EEPROM). When a
//! checked frame's ID matches a loaded fob, KAT reproduces the CAS3's own check ([`recover_counter`]
//! → [`hitag2::bmw_cas3_authenticator`] → compare with `b4..b7`): it recovers the full 32-bit
//! counter in the `0x6D0` resync window, resyncs the stored one, and marks the decode authenticated
//! (`protocol_display_name` gains a ✓) and encodable. [`BmwV3Decoder::encode`] then builds the fob's
//! next valid code ([`frame_for_counter`]). Inert without a key; the air frame alone is decode-only.
//!
//! **Where the secrets live in a CAS3 dump** (module EEPROM / D-flash; the NvM descriptor table at
//! P-flash FD:9470 maps them, EEPROM offset = file offset): 10 parallel slots —
//! ids `0x0830` (`u16`), key-hi `0x0848` (`u16`), key-lo `0x0860` (`u32`), counters `0x088C`
//! (`u32`, wear-levelled so up to ~100 presses stale — harmless, KAT resyncs), uids `0x0A8C`
//! (`u32`, stored only for the "type 2" slots; others derive the uid from the VIN at learn time).
//! `KEY48 = key-hi ‖ key-lo`. The extractor is
//! `REFERENCES/tools/scripts/bmw-cas-porsche-2026-11-24/tools/extract_cas3_keys.py`.

use super::foxfw_common::duration_diff;
use super::info::prelude::*;
use super::{hitag2, keys, DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (see `info.rs`): flags 433 | 315 | AM | FM, dynamic.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::Both,
    encryption: "HITAG2 rolling (CAS3)",
    make: "BMW / Mini",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&FM476, &AM650],
};

const NAME: &str = "BMW/Mini V3";
const TE_SHORT: u32 = 95;
const TE_LONG: u32 = 190;
const TE_DELTA: u32 = 40;
const DATA_BITS: u16 = 64;
/// The payload and the XOR check byte `b9`.
const FRAME_BITS: u16 = DATA_BITS + 8;
/// Check bits that make a check: all 8, or 7 when the last bit runs into the gap after the frame.
const MIN_CHECK_BITS: u16 = 7;
const HEADER: u8 = 0xCC;
const GAP: u32 = 350;
/// The LOW gap before each copy (and the priming pulse that follows); ~500 µs on the captures.
const GAP_LOW_US: u32 = 500;
/// Copies per transmission. The fob sends ~9; KAT sends a few (enough for the module to catch one).
const TX_COPIES: usize = 4;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    WaitGap,
    FindHeader,
    CollectData,
}

/// BMW/Mini V3 decoder (`SubGhzProtocolDecoderBmwV3`).
pub struct BmwV3Decoder {
    step: Step,
    data: u64,
    nbit: u16,
    te_last: u32,
    /// The bits after the payload (the XOR byte `b9`), MSB first.
    check: u8,
}

/// Valid buttons of the C (`bmw_v3_button_valid`), kept for frames without a check.
fn button_valid(btn: u8) -> bool {
    matches!(btn, 0x02 | 0x04 | 0x08 | 0x10)
}

/// The types a CAS3 takes (`b1` bits 7..5: a key, or the second class of transmitter).
fn cas3_type_valid(b1: u8) -> bool {
    matches!(b1 & 0xE0, 0x00 | 0x40 | 0x80)
}

/// `b9`: the XOR of the payload bytes `b1..b8` (the CAS3's `FE:8BE6`).
pub(super) fn payload_xor(data: u64) -> u8 {
    data.to_be_bytes().iter().fold(0, |x, b| x ^ b)
}

/// Whether `n` check bits received after the payload (MSB first, in the low `n` bits of `check`)
/// match the payload's XOR: all 8, or the first 7 when the last one is lost in the trailing gap.
fn check_matches(data: u64, check: u8, n: u16) -> bool {
    (MIN_CHECK_BITS..=8).contains(&n) && u16::from(check) == u16::from(payload_xor(data)) >> (8 - n)
}

/// The CAS3 resync window: the stored counter may trail the received one by up to this much (the
/// firmware's `FC:A7ED` loop raises the candidate by 0x100 while it stays within 0x6D0 of the
/// stored value, so about seven low-byte wraps are tried before a key is marked out of sync).
const RESYNC_WINDOW: u32 = 0x6D0;

/// The 32-bit HITAG2 authenticator `b4..b7` of a payload (`data` bits 39..8).
fn authenticator_field(data: u64) -> u32 {
    ((data >> 8) & 0xFFFF_FFFF) as u32
}

/// Recover a fob's full 32-bit counter from one frame, exactly as the CAS3 does (`FC:A7ED` →
/// HITAG2 → `FE:82D7`): take the stored counter with its low byte replaced by `b8`, step it past
/// the stored value, and raise it by 0x100 (one low-byte wrap) at a time while within
/// [`RESYNC_WINDOW`], returning the first counter whose HITAG2 authenticator equals `auth32`.
/// `None` when no counter in the window matches (the fob would be marked out of sync). Pure: the
/// key material is passed in, so it can be unit-tested against [`hitag2::bmw_cas3_authenticator`].
pub(super) fn recover_counter(
    uid: u32,
    key: &[u8; 6],
    auth32: u32,
    b8: u8,
    stored: u32,
) -> Option<u32> {
    let mut iv = (stored & 0xFFFF_FF00) | u32::from(b8);
    if iv <= stored {
        iv = iv.wrapping_add(0x100);
    }
    while iv.wrapping_sub(stored) <= RESYNC_WINDOW {
        if hitag2::bmw_cas3_authenticator(uid, iv, key) == auth32 {
            return Some(iv);
        }
        iv = iv.wrapping_add(0x100);
    }
    None
}

/// Build the fob's frame for a given counter: the 64-bit payload `b1 | id | auth | b8` and its XOR
/// check byte `b9`. `b1` is the type/button byte, `id` the fob's 16-bit ID, and the authenticator
/// is HITAG2 over the full counter (`b8` is its low byte). The fob increments its counter by one
/// each press, so the next code uses `counter + 1`. Pure (the encoder primitive).
pub(super) fn frame_for_counter(id: u16, uid: u32, key: &[u8; 6], counter: u32, b1: u8) -> (u64, u8) {
    let auth = hitag2::bmw_cas3_authenticator(uid, counter, key);
    let payload = (u64::from(b1) << 56)
        | (u64::from(id) << 40)
        | (u64::from(auth) << 8)
        | u64::from(counter & 0xFF);
    (payload, payload_xor(payload))
}

/// The biphase pulses for one frame's bits (`0xCC ‖ payload ‖ b9`, MSB first), built to drive the
/// decoder's own accumulator: after the 500 µs LOW gap a HIGH priming short leaves a pending half,
/// then each bit completes on a pulse whose level is `!bit`. Levels alternate, so a bit is one long
/// pulse when the next level already suits it, else two short pulses (parity unchanged). Frames
/// always start with `0xCC` (first bit 1), which keeps the pending prime consistent. The inverse of
/// [`BmwV3Decoder::feed`]; proven by the encode→decode round-trip test.
fn build_pulses(bits: &[bool]) -> Vec<LevelDuration> {
    let mut out = vec![
        LevelDuration::new(false, GAP_LOW_US),
        LevelDuration::new(true, TE_SHORT), // prime: a pending short at HIGH
    ];
    let mut pending = true;
    let mut level = false; // the next pulse alternates to LOW
    for &b in bits {
        if pending {
            // complete the pending half on a short at `level` (== !b for a 0xCC-led frame)
            out.push(LevelDuration::new(level, TE_SHORT));
            pending = false;
            level = !level;
        } else if level != b {
            out.push(LevelDuration::new(level, TE_LONG));
            level = !level;
        } else {
            out.push(LevelDuration::new(level, TE_SHORT));
            out.push(LevelDuration::new(!level, TE_SHORT));
        }
    }
    let last_high = out.last().is_none_or(|p| p.level);
    out.push(LevelDuration::new(!last_high, GAP_LOW_US));
    out
}

/// The 80 frame bits `0xCC ‖ payload ‖ b9`, MSB first.
fn frame_bits(payload: u64, b9: u8) -> Vec<bool> {
    (0..8)
        .map(|i| (HEADER >> (7 - i)) & 1 == 1)
        .chain((0..64).map(|i| (payload >> (63 - i)) & 1 == 1))
        .chain((0..8).map(|i| (b9 >> (7 - i)) & 1 == 1))
        .collect()
}

/// If a frame passed the XOR check and the keystore holds this fob's CAS3 secrets, verify its
/// HITAG2 authenticator, resync the stored counter, and mark the decode authenticated and
/// encodable. Mirrors the JLR path in [`super::ford_v1`]. A no-op without a matching key.
fn cas3_enrich(sig: &mut DecodedSignal) {
    let Some(id) = sig.serial.map(|s| s as u16) else {
        return;
    };
    // Clone the fob so the read guard is dropped before any write guard is taken.
    let Some(fob) = keys::get_keystore().bmw_cas3_fob(id) else {
        return;
    };
    let auth32 = authenticator_field(sig.data);
    let b8 = (sig.data & 0xFF) as u8;
    if let Some(full) = recover_counter(fob.uid, &fob.key, auth32, b8, fob.counter) {
        keys::get_keystore_mut().remember_bmw_cas3_counter(id, full);
        sig.encoder_capable = true;
        sig.protocol_display_name = Some(format!("{NAME} \u{2713}"));
    }
}

/// A decode of the 64-bit payload: button byte `b1`, 16-bit ID `b2 b3`, counter low byte `b8`.
fn signal(data: u64) -> DecodedSignal {
    DecodedSignal {
        serial: Some(((data >> 40) & 0xFFFF) as u32),
        button: Some(((data >> 56) & 0xFF) as u8),
        counter: Some((data & 0xFF) as u16),
        crc_valid: false,
        data,
        data_count_bit: usize::from(DATA_BITS),
        encoder_capable: false,
        extra: None,
        protocol_display_name: None,
    }
}

impl BmwV3Decoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            data: 0,
            nbit: 0,
            te_last: 0,
            check: 0,
        }
    }

    fn start_gap(&mut self) {
        self.step = Step::WaitGap;
        self.data = 0;
        self.nbit = 0;
        self.te_last = 0;
        self.check = 0;
    }

    /// `bmw_v3_add_bit`: accumulate one bit and drive the header search / payload collection; KAT
    /// goes on past the payload for the XOR byte and reports the frame on its last bit.
    fn add_bit(&mut self, bit: bool) -> Option<DecodedSignal> {
        match self.step {
            Step::FindHeader => {
                self.data = (self.data << 1) | u64::from(bit);
                self.nbit = self.nbit.wrapping_add(1);
                if self.nbit >= 8 && (self.data & 0xFF) as u8 == HEADER {
                    self.data = 0;
                    self.nbit = 0;
                    self.check = 0;
                    self.step = Step::CollectData;
                } else if self.nbit > 32 {
                    self.step = Step::Reset;
                }
                None
            }
            Step::CollectData => {
                if self.nbit < DATA_BITS {
                    self.data = (self.data << 1) | u64::from(bit);
                } else {
                    self.check = (self.check << 1) | u8::from(bit);
                }
                self.nbit += 1;
                if self.nbit == FRAME_BITS {
                    self.step = Step::Reset;
                    return self.finish();
                }
                None
            }
            _ => None,
        }
    }

    /// The frame is over (its last bit, a pulse outside the timing, or the end of the input):
    /// report the payload when it is complete. With a check it is reported for any CAS3 type,
    /// without one only for FoxFW's buttons.
    fn finish(&self) -> Option<DecodedSignal> {
        if self.nbit < DATA_BITS {
            return None;
        }
        let checked = check_matches(self.data, self.check, self.nbit - DATA_BITS);
        let b1 = (self.data >> 56) as u8;
        let report = if checked { cas3_type_valid(b1) } else { button_valid(b1) };
        report.then(|| {
            let mut sig = DecodedSignal {
                crc_valid: checked,
                ..signal(self.data)
            };
            // A frame that passed the XOR check can be authenticated against a loaded key.
            if checked {
                cas3_enrich(&mut sig);
            }
            sig
        })
    }
}

impl Default for BmwV3Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for BmwV3Decoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: usize::from(DATA_BITS),
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.step = Step::Reset;
        self.data = 0;
        self.nbit = 0;
        self.te_last = 0;
        self.check = 0;
    }

    /// A frame cut off by the end of the input (64..71 bits after the header) is reported now.
    fn flush(&mut self) -> Option<DecodedSignal> {
        let out = if self.step == Step::CollectData {
            self.finish()
        } else {
            None
        };
        self.reset();
        out
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                if !level && duration > GAP {
                    self.start_gap();
                }
            }
            Step::WaitGap => {
                if level {
                    if duration_diff(duration, TE_SHORT) <= TE_DELTA
                        || duration_diff(duration, TE_LONG) <= TE_DELTA
                    {
                        self.te_last = duration;
                        self.step = Step::FindHeader;
                    } else {
                        self.step = Step::Reset;
                    }
                } else if duration <= GAP {
                    self.step = Step::Reset;
                }
            }
            Step::FindHeader | Step::CollectData => {
                let is_short = duration_diff(duration, TE_SHORT) <= TE_DELTA;
                let is_long = duration_diff(duration, TE_LONG) <= TE_DELTA;
                if !is_short && !is_long {
                    // KAT: a frame that ends here (with or without all of its XOR byte) reports.
                    let out = if self.step == Step::CollectData {
                        self.finish()
                    } else {
                        None
                    };
                    self.step = Step::Reset;
                    if !level && duration > GAP {
                        self.start_gap();
                    }
                    return out;
                }
                if is_short {
                    if self.te_last == 0 {
                        self.te_last = duration;
                    } else {
                        let out = self.add_bit(!level);
                        self.te_last = 0;
                        return out;
                    }
                } else if self.te_last != 0 {
                    let out = self.add_bit(!level);
                    self.te_last = TE_SHORT;
                    return out;
                } else {
                    let out = self.add_bit(!level);
                    self.te_last = 0;
                    return out;
                }
            }
        }
        None
    }

    /// KAT can transmit this protocol, but only for a decode whose fob key is loaded
    /// (`encoder_capable`, set by [`cas3_enrich`]); without the key the frame is decode-only.
    fn supports_encoding(&self) -> bool {
        true
    }

    /// Build this fob's next valid code: look up its CAS3 secrets by ID, take its stored (resynced)
    /// counter + 1, compute the HITAG2 authenticator, assemble `b1 ‖ id ‖ auth ‖ b8` with a fresh
    /// XOR byte, and emit [`TX_COPIES`] biphase copies. The button byte `b1` is kept from the
    /// captured frame (the button→nibble map is not yet verified), so this is "send next code" for
    /// the same press. Advances the stored counter so repeated sends keep moving forward. `None`
    /// unless the fob's key is loaded (so a decode without a key stays decode-only).
    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != usize::from(DATA_BITS) {
            return None;
        }
        let id = decoded.serial? as u16;
        let fob = keys::get_keystore().bmw_cas3_fob(id)?;
        let b1 = (decoded.data >> 56) as u8;
        let counter = fob.counter.wrapping_add(1);
        let (payload, b9) = frame_for_counter(id, fob.uid, &fob.key, counter, b1);
        keys::get_keystore_mut().remember_bmw_cas3_counter(id, counter);
        let bits = frame_bits(payload, b9);
        Some(
            std::iter::repeat_with(|| build_pulses(&bits))
                .take(TX_COPIES)
                .flatten()
                .collect(),
        )
    }

    /// `subghz_protocol_decoder_bmw_v3_deserialize`: `Bit: 64` and the key; the fields are carved
    /// from it (the extra lines are not read, and the button whitelist is not applied).
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        (key.bits == usize::from(DATA_BITS)).then(|| signal(key.key))
    }

    /// `subghz_protocol_decoder_bmw_v3_serialize`: the generic Key, then `Serial`, `Btn`, `Cnt`.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != usize::from(DATA_BITS) {
            return None;
        }
        let mut key = FlipperKey::for_protocol(NAME, usize::from(DATA_BITS), decoded.data);
        key.push_u32("Serial", decoded.serial?);
        key.push_u32("Btn", u32::from(decoded.button?));
        key.push_u32("Cnt", u32::from(decoded.counter?));
        Some(key)
    }
}

/// Synthetic captures for the tests.
#[cfg(test)]
pub(super) mod testing {
    use super::*;
    use crate::protocols::foxfw_common::test_support::{bits_of, pulses_for_bits};

    /// Pulses of `0xCC` + a 64-bit payload: a 500 µs LOW gap, a 95 µs HIGH priming pulse, the 72 bits
    /// as short/long pulses (same accumulator as BMW V2), a 500 µs LOW.
    pub fn frame(payload: u64) -> Vec<LevelDuration> {
        frame_with_header(HEADER, payload)
    }

    /// [frame] with another header byte.
    pub fn frame_with_header(header: u8, payload: u64) -> Vec<LevelDuration> {
        let mut bits = bits_of(u64::from(header), 8);
        bits.extend(bits_of(payload, 64));
        frame_bits(&bits)
    }

    /// [frame] for any bit string (header and payload included).
    pub fn frame_bits(bits: &[bool]) -> Vec<LevelDuration> {
        let mut out = vec![
            LevelDuration::new(false, 500),
            LevelDuration::new(true, TE_SHORT),
        ];
        out.extend(pulses_for_bits(
            true,
            true,
            bits,
            TE_SHORT,
            TE_LONG,
            super::super::bmw_v2::testing::step,
        ));
        let last = out.last().is_none_or(|p| p.level);
        out.push(LevelDuration::new(!last, 500));
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::foxfw_common::test_support::*;

    const PAYLOAD_A: u64 = 0x04AB_CD12_3456_789A;
    const PAYLOAD_B: u64 = 0x1001_0203_0405_0607;
    /// A real frame of the 2004 E60 capture in IMPORTS (`08 0B FB B3 76 0E BA C0`, `b9` = 0x49).
    const E60_C0: u64 = 0x080B_FBB3_760E_BAC0;

    /// 0xCC, a payload and the first `n` bits of `check` (MSB first) after it.
    fn with_check(payload: u64, check: u8, n: u32) -> Vec<LevelDuration> {
        let mut bits = bits_of(u64::from(HEADER), 8);
        bits.extend(bits_of(payload, 64));
        bits.extend(bits_of(u64::from(check) >> (8 - n), n));
        testing::frame_bits(&bits)
    }

    /// Frames: (name, pulses, expected (payload, serial, button, counter, checked)). The unchecked
    /// ones (no XOR byte) are FoxFW's cases and decode as the FoxFW oracle does, but for `counter`.
    #[allow(clippy::type_complexity)]
    fn cases() -> Vec<(&'static str, Vec<LevelDuration>, Option<(u64, u32, u8, u16, bool)>)> {
        let mut junk_header = bits_of(0x15, 5);
        junk_header.extend(bits_of(u64::from(HEADER), 8));
        junk_header.extend(bits_of(PAYLOAD_B, 64));
        let mut far_header: Vec<bool> = (0..30).map(|i| i % 2 == 0).collect(); // 1010...
        far_header.extend(bits_of(u64::from(HEADER), 8));
        far_header.extend(bits_of(PAYLOAD_B, 64));
        let mut short_payload = bits_of(u64::from(HEADER), 8);
        short_payload.extend(bits_of(PAYLOAD_A >> 1, 63));
        let xor = |p: u64| payload_xor(p);
        let release = 0x000B_FBB7_66BD_D4C3; // b1 = 0: the button-released telegram
        let button_1 = 0x01AB_CD12_3456_789A;
        let type_40 = 0x41AB_CD12_3456_789A;
        let type_20 = 0x21AB_CD12_3456_789A;
        vec![
            ("a", testing::frame(PAYLOAD_A), Some((PAYLOAD_A, 0xABCD, 0x04, 0x9A, false))),
            ("b", testing::frame(PAYLOAD_B), Some((PAYLOAD_B, 0x0102, 0x10, 0x07, false))),
            ("button_02", testing::frame(0x02FE_DCBA_9876_5432), Some((0x02FE_DCBA_9876_5432, 0xFEDC, 0x02, 0x32, false))),
            ("button_08", testing::frame(0x08F0_E1D2_C3B4_A596), Some((0x08F0_E1D2_C3B4_A596, 0xF0E1, 0x08, 0x96, false))),
            ("junk_before_header", testing::frame_bits(&junk_header), Some((PAYLOAD_B, 0x0102, 0x10, 0x07, false))),
            ("bad_button", testing::frame(0x03AB_CD12_3456_789A), None),
            ("bad_header", testing::frame_with_header(0xCD, PAYLOAD_A), None),
            ("header_too_late", testing::frame_bits(&far_header), None),
            ("short_payload", testing::frame_bits(&short_payload), None),
            // KAT: the XOR byte.
            ("a_checked", with_check(PAYLOAD_A, xor(PAYLOAD_A), 8), Some((PAYLOAD_A, 0xABCD, 0x04, 0x9A, true))),
            ("a_checked_7_bits", with_check(PAYLOAD_A, xor(PAYLOAD_A), 7), Some((PAYLOAD_A, 0xABCD, 0x04, 0x9A, true))),
            ("a_checked_6_bits", with_check(PAYLOAD_A, xor(PAYLOAD_A), 6), Some((PAYLOAD_A, 0xABCD, 0x04, 0x9A, false))),
            ("a_bad_check", with_check(PAYLOAD_A, xor(PAYLOAD_A) ^ 0x01, 8), Some((PAYLOAD_A, 0xABCD, 0x04, 0x9A, false))),
            ("e60_c0", with_check(E60_C0, 0x49, 8), Some((E60_C0, 0x0BFB, 0x08, 0xC0, true))),
            ("release_checked", with_check(release, xor(release), 8), Some((release, 0x0BFB, 0x00, 0xC3, true))),
            ("release_unchecked", testing::frame(release), None),
            ("button_1_checked", with_check(button_1, xor(button_1), 8), Some((button_1, 0xABCD, 0x01, 0x9A, true))),
            ("button_1_bad_check", with_check(button_1, xor(button_1) ^ 0x80, 8), None),
            ("type_40_checked", with_check(type_40, xor(type_40), 8), Some((type_40, 0xABCD, 0x41, 0x9A, true))),
            ("type_20_checked", with_check(type_20, xor(type_20), 8), None),
        ]
    }

    #[test]
    fn key_file_round_trip_and_decode_only() {
        let dec = BmwV3Decoder::new();
        let d = signal(PAYLOAD_A);
        let text = crate::protocols::flipper_key::testing::assert_round_trip(&dec, &d);
        // the lines the C's serialize writes (checked on the oracle's output), but `Cnt`: KAT's
        // counter is b8 (0x9A), FoxFW's bits 31..16 (0x3456 = 13398)
        assert!(text.contains("Protocol: BMW/Mini V3\nBit: 64\nKey: 04 AB CD 12 34 56 78 9A\nSerial: 43981\nBtn: 4\nCnt: 154\n"), "{text}");
        let mut wrong_bits = FlipperKey::parse(&text).unwrap();
        wrong_bits.bits = 56;
        assert!(dec.from_flipper_key(&wrong_bits).is_none());
        // KAT can transmit this protocol, but a decode without a loaded fob key stays decode-only.
        assert!(dec.supports_encoding());
        assert!(dec.encode(&d, 1).is_none(), "no key loaded → no TX");
    }

    #[test]
    fn synthetic_frames_match_the_foxfw_oracle_and_the_cas3_check() {
        for (name, wire, want) in cases() {
            dump(&format!("bmw_v3_{name}.sub"), 433_920_000, "FuriHalSubGhzPreset2FSKDev476Async", &wire);
            let got = decode_all(&mut BmwV3Decoder::new(), &wire);
            eprintln!("CASE {name}: {:?}", got.iter().map(|d| (format!("{:016X}", d.data), d.serial, d.button, d.counter, d.crc_valid)).collect::<Vec<_>>());
            assert_eq!(got.len(), usize::from(want.is_some()), "{name}");
            if let Some((payload, serial, button, counter, checked)) = want {
                let d = &got[0];
                assert_eq!((d.data, d.serial, d.button, d.counter, d.crc_valid), (payload, Some(serial), Some(button), Some(counter), checked), "{name}");
                assert_eq!(d.data_count_bit, 64);
            }
        }
    }

    /// A frame cut off by the end of the input (no pulse after it) is reported by `flush`.
    #[test]
    fn a_frame_at_the_end_of_the_input_is_flushed() {
        let mut wire = with_check(E60_C0, 0x49, 7);
        wire.pop(); // the trailing gap
        let mut dec = BmwV3Decoder::new();
        let fed: Vec<_> = wire.iter().filter_map(|p| dec.feed(p.level, p.duration_us)).collect();
        assert!(fed.is_empty(), "nothing before the end: {fed:?}");
        let d = dec.flush().expect("the cut-off frame");
        assert_eq!((d.data, d.counter, d.crc_valid), (E60_C0, Some(0xC0), true));
        assert!(dec.flush().is_none());
    }

    // ── HITAG2 keystore: verification, counter recovery, and the encoder ──────────────────────

    const K_UID: u32 = 0x1A2B_3C4D;
    const K_KEY: [u8; 6] = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55];
    const K_ID: u16 = 0x5A5A;

    /// `recover_counter` finds the frame's counter when the stored one trails it within the window,
    /// rejects an exact replay of the stored counter, and gives up past the window — exactly the
    /// CAS3's `FC:A7ED` loop. Checked against the firmware-proven [`hitag2::bmw_cas3_authenticator`].
    #[test]
    fn recover_counter_tracks_the_fob_within_the_resync_window() {
        let counter = 0x0001_0234u32; // low byte 0x34
        let b8 = (counter & 0xFF) as u8;
        let auth = hitag2::bmw_cas3_authenticator(K_UID, counter, &K_KEY);
        // one press ahead of the stored counter → found
        assert_eq!(recover_counter(K_UID, &K_KEY, auth, b8, counter - 1), Some(counter));
        // three low-byte wraps behind → still found (within 0x6D0)
        assert_eq!(recover_counter(K_UID, &K_KEY, auth, b8, counter - 0x300), Some(counter));
        // an exact replay of the stored counter is not accepted (strictly increasing)
        assert_eq!(recover_counter(K_UID, &K_KEY, auth, b8, counter), None);
        // too far behind → out of sync
        assert_eq!(recover_counter(K_UID, &K_KEY, auth, b8, counter - 0x1000), None);
        // the wrong key never matches
        assert_eq!(recover_counter(K_UID, &[0xFF; 6], auth, b8, counter - 1), None);
    }

    /// `frame_for_counter` assembles `b1 ‖ id ‖ auth ‖ b8` with a correct XOR byte, and the fields
    /// decode back as the signal would read them.
    #[test]
    fn frame_for_counter_is_well_formed() {
        let counter = 0x00AB_CD11u32;
        let (payload, b9) = frame_for_counter(K_ID, K_UID, &K_KEY, counter, 0x08);
        assert_eq!(payload_xor(payload), b9);
        assert_eq!((payload >> 56) as u8, 0x08); // b1
        assert_eq!(((payload >> 40) & 0xFFFF) as u16, K_ID); // b2 b3
        assert_eq!(authenticator_field(payload), hitag2::bmw_cas3_authenticator(K_UID, counter, &K_KEY));
        assert_eq!((payload & 0xFF) as u8, 0x11); // b8 = counter low byte
    }

    /// The biphase encoder is the exact inverse of the decoder: `build_pulses` of a frame decodes
    /// straight back to it, checked over several payloads and button bytes.
    #[test]
    fn encoder_pulses_decode_back_to_the_frame() {
        for (b1, counter) in [(0x08u8, 0x0000_0001u32), (0x04, 0x0000_00FF), (0x02, 0x0012_3456), (0x00, 0xFFFF_FF00)] {
            let (payload, b9) = frame_for_counter(K_ID, K_UID, &K_KEY, counter, b1);
            let wire = build_pulses(&frame_bits(payload, b9));
            let got = decode_all(&mut BmwV3Decoder::new(), &wire);
            assert_eq!(got.len(), 1, "b1={b1:02X} cnt={counter:08X}");
            assert_eq!((got[0].data, got[0].crc_valid), (payload, true), "b1={b1:02X} cnt={counter:08X}");
        }
    }

    /// End to end with a loaded key: a real-shaped frame built from the fob's secrets decodes as
    /// authenticated (display name + encoder-capable), the stored counter resyncs, and the encoder
    /// then makes the fob's next valid code. Uses the global keystore (a unique fob ID), and clears
    /// it afterwards.
    #[test]
    fn a_loaded_fob_verifies_resyncs_and_encodes_the_next_code() {
        use crate::protocols::keys::{get_keystore, get_keystore_mut, BmwCas3Fob};
        let counter = 0x0007_0042u32;
        get_keystore_mut().load_bmw_cas3_fobs(vec![BmwCas3Fob {
            name: "test fob".into(),
            id: K_ID,
            uid: K_UID,
            key: K_KEY,
            counter: counter - 1, // stored one press behind
        }]);

        let (payload, b9) = frame_for_counter(K_ID, K_UID, &K_KEY, counter, 0x08);
        let wire = build_pulses(&frame_bits(payload, b9));
        let decoded = decode_all(&mut BmwV3Decoder::new(), &wire);
        assert_eq!(decoded.len(), 1);
        let d = &decoded[0];
        assert!(d.crc_valid, "XOR check");
        assert!(d.encoder_capable, "key loaded → encodable");
        assert_eq!(d.protocol_display_name.as_deref(), Some("BMW/Mini V3 \u{2713}"));
        // the stored counter resynced to the frame's counter
        assert_eq!(get_keystore().bmw_cas3_fob(K_ID).unwrap().counter, counter);

        // the encoder makes the next code (counter + 1) and it decodes back, authenticated
        let dec = BmwV3Decoder::new();
        let tx = dec.encode(d, 0x08).expect("key loaded → TX");
        let back = decode_all(&mut BmwV3Decoder::new(), &tx);
        assert!(!back.is_empty(), "TX decodes back");
        let next_auth = hitag2::bmw_cas3_authenticator(K_UID, counter + 1, &K_KEY);
        assert_eq!(authenticator_field(back[0].data), next_auth, "next code's authenticator");
        assert_eq!(get_keystore().bmw_cas3_fob(K_ID).unwrap().counter, counter + 1, "TX advanced the counter");

        get_keystore_mut().load_bmw_cas3_fobs(vec![]); // clear the global store
    }

    /// The E60 frames' XOR bytes (all copies recorded to their end): b9 = b1 ^ … ^ b8.
    #[test]
    fn the_e60_frames_check() {
        for (payload, b9) in [
            (0x080B_FBB3_760E_BAC0u64, 0x49u8),
            (0x080B_FBB3_6E52_80C2, 0x35),
            (0x080B_FBB2_7E5A_52C4, 0xF8),
            (0x100B_FBB2_7699_58C5, 0x20),
            (0x080B_FBB2_6696_1EC6, 0x62),
            (0x080B_FBB3_66E8_A3C8, 0xAE),
        ] {
            assert_eq!(payload_xor(payload), b9, "{payload:016X}");
        }
    }
}
