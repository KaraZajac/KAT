//! Opel V1 protocol decoder (decode-only)
//!
//! Aligned with Flipper-ARF reference: `REFERENCES/Flipper-ARF/lib/subghz/protocols/opel_v1.c`
//! (`SUBGHZ_PROTOCOL_OPEL_V1_NAME = "Opel V1"`). The doctor-firmware copy
//! (`REFERENCES/doctor-firmware/lib/subghz/protocols/opel_v1.c`) is byte-identical.
//!
//! # Wire format (AM/OOK, 433 MHz, Manchester 120/240µs)
//! Pulses within ±70µs of 120µs are short, within ±70µs of 240µs long (short wins the 171–189µs
//! overlap, as in the C); anything else resets the bit accumulator and the Manchester state.
//! Level → event mapping is `level ? ShortLow : ShortHigh` (and Long likewise), decoded with the
//! **canonical Flipper `manchester_decoder.c` table** (states Start1/Mid1/Mid0/Start0, events
//! 0/2/4/6, "same state → reset to Mid1", Mid0 → emit 0, Mid1 → emit 1), seeded `Mid1`. With this
//! mapping an on-air `1` is HIGH→LOW and a `0` is LOW→HIGH. (KAT's shared 4-state table in
//! `kia_v7.rs`/`ford_v3.rs`/`common.rs` is *not* equivalent to the Flipper table — see the
//! `manchester_table_matches_flipper` test — so this module carries the canonical one, like
//! `psa.rs` and `common.rs`.)
//!
//! # Frame (93 bits, MSB first) — `opel_v1_try_finish`
//! Bits shift into a 128-bit window (`data_high:data_low`); the bit count caps at 93, so once 93
//! clean bits have been seen, *every* new bit re-tests the latest 93-bit window:
//! - bits 92..88: framing, must be `0x02` (`0b00010`)
//! - bits 87..64: 24-bit serial (plaintext)
//! - bits 63..0:  64-bit XTEA cipher block
//!
//! The cipher is decrypted with standard XTEA (32 cycles, big-endian words) under the static key
//! `"RobertBOSCH48331"`. Plaintext: `[check, button, burst, count(32-bit BE), tail]`; the frame is
//! valid when `tail == 0x05`. When the framing matches, the accumulator is cleared whether or not
//! the tail validated (as in the C).
//!
//! `DecodedSignal` packing (frame > 64 bits): `data` = the first 64 wire bits
//! (`framing:5 | serial:24 | cipher[63:29]`), `data_count_bit` = 93, `extra` = the last 29 wire
//! bits (`cipher[28:0]`), so `cipher = ((data & 0x7_FFFF_FFFF) << 29) | extra`. `serial` = 24-bit
//! serial, `button` = decrypted button byte, `counter` = low 16 bits of the 32-bit count (the C's
//! `check` and `burst` bytes have no `DecodedSignal` field). The C itself stores `data` = cipher
//! with `data_count_bit` = 64.
//!
//! # Gating
//! As in the C: framing `0x02` + exact 93-bit Manchester window + decrypted tail `0x05` (13 bits).
//! Any button is reported; the C names Trunk `0x20`, Lock `0x04` and Unlock `0x08` and shows other
//! values as "Unknown". (KAT used to drop the other values; a real 2016 Holden Barina fob sends
//! button 0x73 with a valid tail, which Flipper-ARF and doctor-firmware decode — serial 4F2679 —
//! so the whitelist was removed.) `crc_valid` = true means the XTEA tail validated.
//!
//! # Encoder
//! None — the C encoder is a NULL stub (decode-only). `supports_encoding()` is false.
//!
//! # Key files
//! `subghz_protocol_decoder_opel_v1_serialize` writes `Protocol: Opel V1`, `Bit: 64`, `Key` = the
//! cipher block, then `XteaKey` (16 bytes), `Disp` ("Opel V1 - Trunk|Lock|Unlock|Unknown") and
//! `Burst`. The 24-bit serial is not in the key: the C's deserialize reads it from a `Serial` line,
//! which only doctor-firmware's generic writer produces (with `Btn` = the decrypted button and
//! `Cnt` = the 32-bit count, right after Key); Flipper-ARF's writer (same C file) stops after Key,
//! so an ARF-saved Opel key reloads with serial 0. KAT writes doctor's lines, so its files keep the
//! serial. Loading follows the C: exactly 64 bits, the cipher decrypted with the file's `XteaKey`
//! (the static "RobertBOSCH48331" when absent), serial from `Serial` (24 bits; 0 when absent),
//! framing 0x02 implied. The C loads any block; `crc_valid` reports its plaintext-tail check, and
//! KAT's receive-side button whitelist is not applied. Decode-only: nothing to transmit.

use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Opel V1",
    decoder: "Opel V1",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "XTEA",
    make: "Opel",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// Opel V1 buttons.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x04 => "Lock",
        0x08 => "Unlock",
        0x20 => "Trunk",
        _ => "Other",
    })
}

const TE_SHORT: u32 = 120;
const TE_LONG: u32 = 240;
const TE_DELTA: u32 = 70;
const FRAME_BITS: u16 = 93;
const FRAMING: u8 = 0x02;
/// Bits after the cipher block (low 29 wire bits go to `extra`).
const EXTRA_BITS: u32 = FRAME_BITS as u32 - 64;
const BTN_TRUNK: u8 = 0x20;
const BTN_LOCK: u8 = 0x04;
const BTN_UNLOCK: u8 = 0x08;
const PLAINTEXT_TAIL: u8 = 0x05;

/// Static XTEA key (`opel_v1_key`): ASCII "RobertBOSCH48331".
const XTEA_KEY: [u8; 16] = *b"RobertBOSCH48331";
const XTEA_DELTA: u32 = 0x9E37_79B9;
const XTEA_ROUNDS: u32 = 32;

// Canonical Flipper ManchesterEvent values (shift amounts into the transition bytes).
const EV_SHORT_LOW: u8 = 0;
const EV_SHORT_HIGH: u8 = 2;
const EV_LONG_LOW: u8 = 4;
const EV_LONG_HIGH: u8 = 6;
/// `transitions[]` from `lib/toolbox/manchester_decoder.c`.
const MANCHESTER_TRANSITIONS: [u8; 4] = [0b0000_0001, 0b1001_0001, 0b1001_1011, 0b1111_1011];

/// Canonical Flipper Manchester states (`manchester_decoder.h` order).
#[derive(Debug, Clone, Copy, PartialEq)]
enum ManchesterState {
    Start1 = 0,
    Mid1 = 1,
    Mid0 = 2,
    Start0 = 3,
}

/// Decrypted plaintext block.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Plaintext {
    check: u8,
    button: u8,
    burst: u8,
    count: u32,
    tail: u8,
}

fn xtea_key_words() -> [u32; 4] {
    let mut k = [0u32; 4];
    for (i, word) in k.iter_mut().enumerate() {
        *word = u32::from_be_bytes(XTEA_KEY[i * 4..i * 4 + 4].try_into().unwrap());
    }
    k
}

/// `opel_v1_xtea_decrypt` (standard XTEA decipher, 32 cycles, big-endian words).
fn xtea_decrypt(block: u64, k: &[u32; 4]) -> u64 {
    let mut v0 = (block >> 32) as u32;
    let mut v1 = block as u32;
    let mut sum = XTEA_DELTA.wrapping_mul(XTEA_ROUNDS); // 0xC6EF3720
    for _ in 0..XTEA_ROUNDS {
        v1 = v1.wrapping_sub(
            sum.wrapping_add(k[((sum >> 11) & 3) as usize])
                ^ v0.wrapping_add((v0 >> 5) ^ (v0 << 4)),
        );
        sum = sum.wrapping_sub(XTEA_DELTA);
        v0 = v0.wrapping_sub(
            sum.wrapping_add(k[(sum & 3) as usize]) ^ v1.wrapping_add((v1 >> 5) ^ (v1 << 4)),
        );
    }
    ((v0 as u64) << 32) | v1 as u64
}

/// `opel_v1_parse_fields`: decrypt the cipher block into its plaintext fields.
fn decrypt_fields(cipher: u64) -> Plaintext {
    plaintext(xtea_decrypt(cipher, &xtea_key_words()))
}

/// [decrypt_fields] under any 16-byte XTEA key (a key file's `XteaKey`).
fn decrypt_with_key(cipher: u64, key: &[u8; 16]) -> Plaintext {
    let mut k = [0u32; 4];
    for (i, word) in k.iter_mut().enumerate() {
        *word = u32::from_be_bytes(key[i * 4..i * 4 + 4].try_into().unwrap());
    }
    plaintext(xtea_decrypt(cipher, &k))
}

/// The plaintext fields of a decrypted block.
fn plaintext(block: u64) -> Plaintext {
    let b = block.to_be_bytes();
    Plaintext {
        check: b[0],
        button: b[1],
        burst: b[2],
        count: u32::from_be_bytes([b[3], b[4], b[5], b[6]]),
        tail: b[7],
    }
}

/// Opel V1 decoder (matches `SubGhzProtocolDecoderOpelV1`).
pub struct OpelV1Decoder {
    manchester_state: ManchesterState,
    data_high: u64,
    data_low: u64,
    bit_count: u16,
}

impl OpelV1Decoder {
    pub fn new() -> Self {
        Self {
            manchester_state: ManchesterState::Mid1,
            data_high: 0,
            data_low: 0,
            bit_count: 0,
        }
    }

    /// Canonical Flipper `manchester_advance`; returns `Some(bit)` on emit.
    fn manchester_advance(&mut self, event: u8) -> Option<bool> {
        let current = self.manchester_state as u8;
        let next = (MANCHESTER_TRANSITIONS[current as usize] >> event) & 0x3;
        if next == current {
            self.manchester_state = ManchesterState::Mid1;
            return None;
        }
        self.manchester_state = match next {
            0 => ManchesterState::Start1,
            1 => ManchesterState::Mid1,
            2 => ManchesterState::Mid0,
            _ => ManchesterState::Start0,
        };
        match self.manchester_state {
            ManchesterState::Mid0 => Some(false),
            ManchesterState::Mid1 => Some(true),
            _ => None,
        }
    }

    /// `opel_v1_accumulator_reset` (incl. `ManchesterEventReset` → Mid1).
    fn accumulator_reset(&mut self) {
        self.data_high = 0;
        self.data_low = 0;
        self.bit_count = 0;
        self.manchester_state = ManchesterState::Mid1;
    }

    /// `opel_v1_add_bit`.
    fn add_bit(&mut self, bit: bool) {
        self.data_high = (self.data_high << 1) | (self.data_low >> 63);
        self.data_low = (self.data_low << 1) | bit as u64;
        if self.bit_count < FRAME_BITS {
            self.bit_count += 1;
        }
    }

    /// `opel_v1_try_finish` + callback decision. Returns `None` while the window is not a frame;
    /// `Some(result)` when the framing matched (accumulator must then be cleared), where `result`
    /// is the signal to emit if the frame also validated.
    fn try_finish(&self) -> Option<Option<DecodedSignal>> {
        if self.bit_count < FRAME_BITS {
            return None;
        }
        if ((self.data_high >> 24) & 0x1F) as u8 != FRAMING {
            return None;
        }
        let serial = (self.data_high & 0xFF_FFFF) as u32;
        let cipher = self.data_low;
        let pt = decrypt_fields(cipher);
        if pt.tail != PLAINTEXT_TAIL {
            return Some(None);
        }
        Some(Some(frame_signal(serial, cipher, &pt)))
    }
}

/// The decode of a frame (framing 0x02, 24-bit `serial`, `cipher`) whose block decrypts to `pt`.
/// `crc_valid` = the plaintext-tail check (always true for a frame the decoder reports).
fn frame_signal(serial: u32, cipher: u64, pt: &Plaintext) -> DecodedSignal {
    let frame_hi = ((FRAMING as u64) << 24) | serial as u64; // wire bits 92..64
    DecodedSignal {
        serial: Some(serial),
        button: Some(pt.button),
        counter: Some(pt.count as u16),
        crc_valid: pt.tail == PLAINTEXT_TAIL,
        data: (frame_hi << (64 - EXTRA_BITS)) | (cipher >> EXTRA_BITS),
        data_count_bit: FRAME_BITS as usize,
        encoder_capable: false,
        extra: Some(cipher & ((1u64 << EXTRA_BITS) - 1)),
        protocol_display_name: None,
    }
}

/// (serial, cipher) of a decode (`data` / `extra` as packed by [frame_signal]).
fn frame_of(decoded: &DecodedSignal) -> Option<(u32, u64)> {
    if decoded.data_count_bit != FRAME_BITS as usize {
        return None;
    }
    let extra = decoded.extra?;
    let serial = ((decoded.data >> (64 - EXTRA_BITS)) & 0xFF_FFFF) as u32;
    let cipher = ((decoded.data & ((1u64 << (64 - EXTRA_BITS)) - 1)) << EXTRA_BITS) | (extra & ((1u64 << EXTRA_BITS) - 1));
    Some((serial, cipher))
}

impl ProtocolDecoder for OpelV1Decoder {
    fn name(&self) -> &'static str {
        "Opel V1"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: FRAME_BITS as usize,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        // C flags: SubGhzProtocolFlag_433 | SubGhzProtocolFlag_AM.
        &[433_920_000]
    }

    fn reset(&mut self) {
        self.accumulator_reset();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let event = if duration_diff!(duration, TE_SHORT) < TE_DELTA {
            if level {
                EV_SHORT_LOW
            } else {
                EV_SHORT_HIGH
            }
        } else if duration_diff!(duration, TE_LONG) < TE_DELTA {
            if level {
                EV_LONG_LOW
            } else {
                EV_LONG_HIGH
            }
        } else {
            self.accumulator_reset();
            return None;
        };

        let bit = self.manchester_advance(event)?;
        self.add_bit(bit);
        let result = self.try_finish()?;
        self.data_high = 0;
        self.data_low = 0;
        self.bit_count = 0;
        result
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }

    /// `subghz_protocol_decoder_opel_v1_deserialize` (see the module doc, "Key files").
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != 64 {
            return None;
        }
        let cipher = key.key_u64()?;
        let xtea: [u8; 16] = key
            .hex_bytes_field("XteaKey")
            .and_then(|k| k.try_into().ok())
            .unwrap_or(XTEA_KEY);
        let serial = key.u32_field("Serial").unwrap_or(0) & 0xFF_FFFF;
        Some(frame_signal(serial, cipher, &decrypt_with_key(cipher, &xtea)))
    }

    /// `subghz_protocol_decoder_opel_v1_serialize` as doctor-firmware writes it (see "Key files").
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        let (serial, cipher) = frame_of(decoded)?;
        let pt = decrypt_fields(cipher);
        let mut key = FlipperKey::for_protocol("Opel V1", 64, cipher);
        if serial != 0 || pt.button != 0 || pt.count != 0 {
            key.push_u32("Serial", serial);
            key.push_u32("Btn", pt.button.into());
            key.push_u32("Cnt", pt.count);
        }
        key.push_hex_bytes("XteaKey", &XTEA_KEY);
        let name = match pt.button {
            BTN_TRUNK => "Trunk",
            BTN_LOCK => "Lock",
            BTN_UNLOCK => "Unlock",
            _ => "Unknown",
        };
        key.push("Disp", format!("Opel V1 - {name}"));
        key.push_u32("Burst", pt.burst.into());
        Some(key)
    }
}

impl Default for OpelV1Decoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::ProtocolRegistry;

    const SERIAL: u32 = 0x00A1_B2C3;

    /// Test-only XTEA encipher (inverse of `xtea_decrypt`).
    fn xtea_encrypt(block: u64, k: &[u32; 4]) -> u64 {
        let mut v0 = (block >> 32) as u32;
        let mut v1 = block as u32;
        let mut sum = 0u32;
        for _ in 0..XTEA_ROUNDS {
            v0 = v0.wrapping_add(
                ((v1 << 4) ^ (v1 >> 5)).wrapping_add(v1) ^ sum.wrapping_add(k[(sum & 3) as usize]),
            );
            sum = sum.wrapping_add(XTEA_DELTA);
            v1 = v1.wrapping_add(
                ((v0 << 4) ^ (v0 >> 5)).wrapping_add(v0)
                    ^ sum.wrapping_add(k[((sum >> 11) & 3) as usize]),
            );
        }
        ((v0 as u64) << 32) | v1 as u64
    }

    fn cipher_for(button: u8, count: u32, tail: u8) -> u64 {
        let mut pt = [0x5A, button, 0x01, 0, 0, 0, 0, tail];
        pt[3..7].copy_from_slice(&count.to_be_bytes());
        xtea_encrypt(u64::from_be_bytes(pt), &xtea_key_words())
    }

    /// Manchester wire (bit 1 = HIGH→LOW, 120µs halves): `preamble` ones, the 93-bit frame,
    /// then a 5ms LOW gap.
    fn build_wire(framing: u8, serial: u32, cipher: u64, preamble: usize) -> Vec<LevelDuration> {
        let frame: u128 = ((framing as u128) << 88) | ((serial as u128) << 64) | cipher as u128;
        let mut bits: Vec<bool> = vec![true; preamble];
        bits.extend((0..93).rev().map(|i| (frame >> i) & 1 != 0));
        let mut out: Vec<LevelDuration> = Vec::new();
        let mut push = |level: bool, d: u32| match out.last_mut() {
            Some(last) if last.level == level => {
                *last = LevelDuration::new(level, last.duration_us + d)
            }
            _ => out.push(LevelDuration::new(level, d)),
        };
        for b in bits {
            push(b, TE_SHORT);
            push(!b, TE_SHORT);
        }
        push(false, 5000);
        out
    }

    fn decode_all(pairs: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = OpelV1Decoder::new();
        pairs
            .iter()
            .filter_map(|p| dec.feed(p.level, p.duration_us))
            .collect()
    }

    #[test]
    fn xtea_matches_reference_vectors() {
        // Published XTEA vector (key 00..0F): 4142434445464748 <-> 497DF3D072612CB5.
        let k: [u32; 4] = [0x0001_0203, 0x0405_0607, 0x0809_0A0B, 0x0C0D_0E0F];
        assert_eq!(
            xtea_decrypt(0x497D_F3D0_7261_2CB5, &k),
            0x4142_4344_4546_4748
        );
        assert_eq!(
            xtea_encrypt(0x4142_4344_4546_4748, &k),
            0x497D_F3D0_7261_2CB5
        );
        // Opel key vector computed independently (Python XTEA): 5A04000000123405 -> 68D23A59C75C20C1.
        assert_eq!(
            xtea_key_words(),
            [0x526F_6265, 0x7274_424F, 0x5343_4834, 0x3833_3331]
        );
        let pt = decrypt_fields(0x68D2_3A59_C75C_20C1);
        assert_eq!(
            pt,
            Plaintext {
                check: 0x5A,
                button: BTN_LOCK,
                burst: 0,
                count: 0x1234,
                tail: 0x05
            }
        );
    }

    #[test]
    fn manchester_table_matches_flipper() {
        // Canonical Flipper semantics: Mid1 + ShortHigh → Start1, Start1 + ShortLow → Mid1 (emit 1),
        // Mid1 + LongHigh → Mid0 (emit 0), Mid0 + LongLow → Mid1 (emit 1), same-state → reset.
        let mut d = OpelV1Decoder::new();
        assert_eq!(d.manchester_advance(EV_SHORT_HIGH), None);
        assert_eq!(d.manchester_state, ManchesterState::Start1);
        assert_eq!(d.manchester_advance(EV_SHORT_LOW), Some(true));
        assert_eq!(d.manchester_advance(EV_LONG_HIGH), Some(false));
        assert_eq!(d.manchester_advance(EV_LONG_LOW), Some(true));
        assert_eq!(
            d.manchester_advance(EV_SHORT_LOW),
            None,
            "Mid1 + ShortLow resets"
        );
        assert_eq!(d.manchester_state, ManchesterState::Mid1);
    }

    #[test]
    fn synthetic_frame_decodes() {
        let cipher = cipher_for(BTN_UNLOCK, 0x0001_2345, PLAINTEXT_TAIL);
        let decoded = decode_all(&build_wire(FRAMING, SERIAL, cipher, 16));
        assert_eq!(decoded.len(), 1);
        let d = &decoded[0];
        assert_eq!(d.serial, Some(SERIAL));
        assert_eq!(d.button, Some(BTN_UNLOCK));
        assert_eq!(d.counter, Some(0x2345), "low 16 bits of the 32-bit count");
        assert_eq!(d.data_count_bit, 93);
        assert!(d.crc_valid && !d.encoder_capable);
        // Packing: data = first 64 wire bits, extra = last 29; the cipher is recoverable.
        assert_eq!(d.data >> 59, FRAMING as u64);
        assert_eq!((d.data >> 35) & 0xFF_FFFF, SERIAL as u64);
        assert_eq!(((d.data & 0x7_FFFF_FFFF) << 29) | d.extra.unwrap(), cipher);
    }

    #[test]
    fn all_named_buttons_decode() {
        for btn in [BTN_TRUNK, BTN_LOCK, BTN_UNLOCK] {
            let cipher = cipher_for(btn, 7, PLAINTEXT_TAIL);
            let decoded = decode_all(&build_wire(FRAMING, SERIAL, cipher, 16));
            assert_eq!(decoded.len(), 1, "button {btn:#04X}");
            assert_eq!(decoded[0].button, Some(btn));
        }
    }

    #[test]
    fn rejects_bad_plaintext_tail() {
        let cipher = cipher_for(BTN_LOCK, 42, 0x06);
        assert!(decode_all(&build_wire(FRAMING, SERIAL, cipher, 16)).is_empty());
    }

    #[test]
    fn rejects_wrong_framing() {
        let cipher = cipher_for(BTN_LOCK, 42, PLAINTEXT_TAIL);
        assert!(decode_all(&build_wire(0x03, SERIAL, cipher, 16)).is_empty());
    }

    #[test]
    fn reports_unnamed_buttons_like_the_c() {
        // Valid tail, a button the C labels "Unknown" (a real Holden Barina fob sends 0x73).
        let cipher = cipher_for(0x73, 42, PLAINTEXT_TAIL);
        assert_eq!(decrypt_fields(cipher).tail, PLAINTEXT_TAIL);
        let got = decode_all(&build_wire(FRAMING, SERIAL, cipher, 16));
        assert!(!got.is_empty());
        assert!(got.iter().all(|d| d.button == Some(0x73) && d.serial == Some(SERIAL) && d.crc_valid));
    }

    #[test]
    fn registry_reports_opel_v1_both_polarities() {
        let cipher = cipher_for(BTN_TRUNK, 99, PLAINTEXT_TAIL);
        let wire = build_wire(FRAMING, SERIAL, cipher, 16);
        let inverted: Vec<LevelDuration> = wire
            .iter()
            .map(|p| LevelDuration::new(!p.level, p.duration_us))
            .collect();
        for (label, w) in [("normal", &wire), ("inverted", &inverted)] {
            let mut reg = ProtocolRegistry::new();
            let results = reg.process_signal_stream(w, 433_920_000);
            let names: Vec<&str> = results.iter().map(|(n, _, _)| n.as_str()).collect();
            assert_eq!(names, vec!["Opel V1"], "{label}");
            assert_eq!(results[0].1.serial, Some(SERIAL), "{label}");
        }
    }

    /// Key files: doctor-firmware's lines round-trip every field (serial included); an ARF file
    /// (no Serial line) loads with serial 0 like ARF's own load; a custom XteaKey is honoured.
    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::{assert_round_trip, key};
        let dec = OpelV1Decoder::new();
        let cipher = cipher_for(BTN_UNLOCK, 0x0001_2345, PLAINTEXT_TAIL);
        let decoded = decode_all(&build_wire(FRAMING, SERIAL, cipher, 16)).remove(0);
        let text = assert_round_trip(&dec, &decoded);
        let hex = |v: u64| v.to_be_bytes().iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(" ");
        assert!(
            text.ends_with(&format!(
                "Protocol: Opel V1\nBit: 64\nKey: {}\nSerial: {SERIAL}\nBtn: 8\nCnt: 74565\n\
                 XteaKey: 52 6F 62 65 72 74 42 4F 53 43 48 34 38 33 33 31\nDisp: Opel V1 - Unlock\nBurst: 1\n",
                hex(cipher)
            )),
            "{text}"
        );
        // Flipper-ARF's file of the same capture: no Serial line → serial 0, fields decrypted.
        let arf = key(&format!(
            "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
             Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Opel V1\nBit: 64\nKey: {}\n\
             XteaKey: 52 6F 62 65 72 74 42 4F 53 43 48 34 38 33 33 31\nDisp: Opel V1 - Unlock\nBurst: 1\n",
            hex(cipher)
        ));
        let d = dec.from_flipper_key(&arf).unwrap();
        assert_eq!((d.serial, d.button, d.counter, d.crc_valid), (Some(0), Some(BTN_UNLOCK), Some(0x2345), true));
        // Another XTEA key decrypts the block differently (tail check fails here).
        let mut other = arf.clone();
        other.fields.retain(|(n, _)| n != "XteaKey");
        other.push_hex_bytes("XteaKey", &[0u8; 16]);
        assert!(!dec.from_flipper_key(&other).unwrap().crc_valid);
        let mut wrong = arf.clone();
        wrong.bits = 93;
        assert!(dec.from_flipper_key(&wrong).is_none());
    }

    /// Oracle vectors (`KAT_KEY_VECTORS=<dir> cargo test opel_v1_key_vectors -- --ignored`).
    /// Decode-only protocol: the RAW files carry the test-built frames.
    #[test]
    #[ignore]
    fn opel_v1_key_vectors() {
        use super::super::bmw::{key_vector_dir, write_key_vector};
        let dec = OpelV1Decoder::new();
        for (n, btn, count) in [(1, BTN_UNLOCK, 0x0001_2345u32), (2, BTN_TRUNK, 99)] {
            let wire = build_wire(FRAMING, SERIAL, cipher_for(btn, count, PLAINTEXT_TAIL), 16);
            let decoded = decode_all(&wire).remove(0);
            let key = dec.to_flipper_key(&decoded).unwrap();
            write_key_vector(&key_vector_dir(), &format!("opel_v1_{n}"), 433_920_000, "FuriHalSubGhzPresetOok650Async", &wire, &key);
        }
    }
}
