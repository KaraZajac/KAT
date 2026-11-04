//! Beninca ARC: AES-128 rolling-code gate/garage remotes.
//!
//! References: Flipper-ARF's garage app `applications/system/subghz_garage/protocols/beninca_arc.c`
//! (primary) and doctor-firmware `lib/subghz/protocols/beninca_arc.c`. Their air format,
//! decryption and upload are identical; they differ only in the history hash, the custom-button
//! bookkeeping and the display text (doctor shows the mini counter and the constant).
//!
//! **Air format** (`subghz_protocol_decoder_beninca_arc_feed`): PWM, te_short 300 µs, te_long
//! 600 µs, te_delta 155 µs. A LOW of 9600 ± 2325 µs (te_long·16 ± te_delta·15) starts a frame;
//! bit 1 = HIGH 300 + LOW 600, bit 0 = HIGH 600 + LOW 300 (each side ± 155 µs, the bit-1 test
//! first); 128 bits, MSB first. On the HIGH after the 64th bit the C stores the first half (unless
//! it is all zero); on the HIGH after the 128th bit (the stop bit) it calls back. Any other pulse
//! sends the decoder back to waiting for the gap.
//!
//! **Frame**: the 16 frame bytes, each bit-reversed, are one AES-128 block (ECB). The key is the
//! first keystore entry of type 9 ("Beninca_ARC"): its 8 bytes big-endian followed by the same 8
//! bytes little-endian. Plaintext: serial (4 bytes, big-endian) | button | 5-byte mini counter
//! (rotates while the button is held) | 32-bit counter (big-endian) | the constant 0xAA55.
//!
//! **KAT fields**: `data` = the first 64 wire bits (C `generic.data`, key-file `Key`), `extra` =
//! the last 64 (C `generic.data_2`, key-file `Data`), `data_count_bit` = 128. serial = 32 bits,
//! button = 8 bits, `counter` = the low 16 bits of the 32-bit counter (the encoder re-decrypts the
//! frame for the full counter, as the C does).
//!
//! **Gating**: the C calls back for every 128-bit frame and only displays the decrypted constant
//! (doctor's "Fx:"); per KAT policy a frame is reported only when it decrypts to 0xAA55
//! (`crc_valid`; a random frame passes 1 time in 65536). No real capture is available to show
//! remotes without the constant.
//!
//! **Encoder** (`subghz_protocol_encoder_beninca_arc_deserialize` + `prepare_packets`): decrypt
//! the frame, counter + 1 (32-bit wrap), button = the requested native code (8 bits; the C's custom
//! buttons cycle 0x02/0x04/0x00), constant kept, then three packets encrypted with mini counter 2,
//! 4 and 6, each 128 bits + a 300 µs stop HIGH + a 9000 µs LOW gap. `encoder.repeat` = 1: the
//! upload holds the three packets once. A frame without the constant is not re-encoded (key files
//! with such a frame are refused). (Flipper-ARF's garage TX plugin is built without `SUBGHZ_GARAGE_HAS_CUSTOM_BTN`,
//! whose fallback makes `get_btn_code()` return 0; KAT keeps the captured button, as doctor-firmware
//! and the garage source's custom-button path do.)

use super::common::DecodedSignal;
use super::flipper_key::FlipperKey;
use super::g5_common::{hex8_field, key8, keystore_entry, next_counter, push};
use super::{ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Beninca ARC",
    decoder: "Beninca ARC",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "AES-128",
    make: "Beninca",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "Beninca ARC";
const TE_SHORT: u32 = 300;
const TE_LONG: u32 = 600;
const TE_DELTA: u32 = 155;
const MIN_COUNT_BIT: usize = 128;
/// C `BENINCA_ARC_KEY_TYPE`: the keystore entry type holding the AES key material.
const KEY_TYPE: u32 = 9;
/// The plaintext's trailing constant (C `generic.seed`, "Fx").
const CONSTANT: u16 = 0xAA55;
/// C `encoder.repeat`.
const TX_REPEAT: usize = 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Start,
    HighLevel,
    LowLevel,
}

/// Decrypted frame (`subghz_protocol_beninca_arc_decrypt`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Plain {
    serial: u32,
    btn: u8,
    /// Bytes 5..9 (40 bits): the mini counter.
    middle: u64,
    cnt: u32,
    /// Bytes 14..15: the constant 0xAA55.
    constant: u16,
}

/// `get_subghz_protocol_beninca_arc_aes_key`: the first type-9 entry's key, big-endian then
/// little-endian (the C byte-swaps both 32-bit halves and swaps them).
fn aes_key() -> Option<[u8; 16]> {
    let mf = keystore_entry(|e| e.learning == KEY_TYPE)?.key;
    let mut k = [0u8; 16];
    k[..8].copy_from_slice(&mf.to_be_bytes());
    k[8..].copy_from_slice(&mf.to_le_bytes());
    Some(k)
}

/// The 16 frame bytes (`data` then `data_2`, big-endian), each byte bit-reversed.
fn frame_block(data: u64, data_2: u64) -> [u8; 16] {
    let mut b = [0u8; 16];
    b[..8].copy_from_slice(&data.to_be_bytes());
    b[8..].copy_from_slice(&data_2.to_be_bytes());
    b.iter_mut().for_each(|x| *x = x.reverse_bits());
    b
}

fn decrypt(data: u64, data_2: u64) -> Option<Plain> {
    let rk = aes::expand_key(&aes_key()?);
    let mut p = frame_block(data, data_2);
    aes::decrypt(&rk, &mut p);
    Some(Plain {
        serial: u32::from_be_bytes([p[0], p[1], p[2], p[3]]),
        btn: p[4],
        middle: p[5..10]
            .iter()
            .fold(0u64, |acc, &b| acc << 8 | u64::from(b)),
        cnt: u32::from_be_bytes([p[10], p[11], p[12], p[13]]),
        constant: u16::from_be_bytes([p[14], p[15]]),
    })
}

/// `subghz_protocol_beninca_arc_encrypt`: the (data, data_2) of a plaintext.
fn encrypt(p: &Plain) -> Option<(u64, u64)> {
    let rk = aes::expand_key(&aes_key()?);
    let mut b = [0u8; 16];
    b[..4].copy_from_slice(&p.serial.to_be_bytes());
    b[4] = p.btn;
    b[5..10].copy_from_slice(&p.middle.to_be_bytes()[3..]);
    b[10..14].copy_from_slice(&p.cnt.to_be_bytes());
    b[14..].copy_from_slice(&p.constant.to_be_bytes());
    aes::encrypt(&rk, &mut b);
    b.iter_mut().for_each(|x| *x = x.reverse_bits());
    let data = u64::from_be_bytes(b[..8].try_into().unwrap());
    let data_2 = u64::from_be_bytes(b[8..].try_into().unwrap());
    Some((data, data_2))
}

/// `subghz_protocol_beninca_arc_create_data` (the SubGHz app's "Add Manually"): the plaintext
/// serial / button / mini counter 1 / counter (as given) / 0xAA55, encrypted with the software AES
/// of doctor-firmware and the garage app (Flipper-ARF's `lib` copy runs the STM32 hardware AES,
/// whose word-swapped key gives other frames that neither those decoders nor KAT's accept);
/// `Bit: 128`, `Key`, `Data` (8 hex bytes). None without the AES key.
pub(super) fn create_data(serial: u32, btn: u8, cnt: u32) -> Option<FlipperKey> {
    let (data, data_2) = encrypt(&Plain { serial, btn, middle: 1, cnt, constant: CONSTANT })?;
    let mut k = FlipperKey::for_protocol(NAME, MIN_COUNT_BIT, data);
    k.push_hex("Data", data_2, 8);
    Some(k)
}

/// The decode of a 128-bit frame (None when the key is missing).
fn signal(data: u64, data_2: u64) -> Option<DecodedSignal> {
    let p = decrypt(data, data_2)?;
    let valid = p.constant == CONSTANT;
    Some(DecodedSignal {
        serial: Some(p.serial),
        button: Some(p.btn),
        counter: Some(p.cnt as u16),
        crc_valid: valid,
        data,
        data_count_bit: MIN_COUNT_BIT,
        encoder_capable: valid,
        extra: Some(data_2),
        protocol_display_name: None,
    })
}

/// One packet of the upload (`subghz_protocol_beninca_arc_encoder_get_upload`).
fn packet(data: u64, data_2: u64, out: &mut Vec<LevelDuration>) {
    for word in [data, data_2] {
        for i in (0..64).rev() {
            if (word >> i) & 1 == 1 {
                push(out, true, TE_SHORT);
                push(out, false, TE_LONG);
            } else {
                push(out, true, TE_LONG);
                push(out, false, TE_SHORT);
            }
        }
    }
    push(out, true, TE_SHORT);
    push(out, false, TE_LONG * 15);
}

pub struct BenincaArcDecoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: usize,
    /// C `generic.data`: the first half, stored at the 64th bit.
    first_half: u64,
}

impl BenincaArcDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Start,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
            first_half: 0,
        }
    }

    fn add_bit(&mut self, bit: u64) {
        self.decode_data = (self.decode_data << 1) | bit;
        self.decode_count_bit += 1;
    }
}

impl Default for BenincaArcDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for BenincaArcDecoder {
    fn name(&self) -> &'static str {
        NAME
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
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        *self = Self::new();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Start => {
                if !level && duration_diff!(duration, TE_LONG * 16) < TE_DELTA * 15 {
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.step = Step::HighLevel;
                }
            }
            Step::HighLevel => {
                if level {
                    self.te_last = duration;
                    self.step = Step::LowLevel;
                    if self.decode_count_bit == MIN_COUNT_BIT / 2 && self.decode_data != 0 {
                        self.first_half = self.decode_data;
                        self.decode_data = 0;
                    } else if self.decode_count_bit == MIN_COUNT_BIT {
                        self.step = Step::Start;
                        // KAT: report only frames that decrypt to the 0xAA55 constant.
                        return signal(self.first_half, self.decode_data).filter(|d| d.crc_valid);
                    }
                } else {
                    self.step = Step::Start;
                }
            }
            Step::LowLevel => {
                if !level {
                    if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA
                        && duration_diff!(duration, TE_LONG) < TE_DELTA
                    {
                        self.add_bit(1);
                        self.step = Step::HighLevel;
                    } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA
                        && duration_diff!(duration, TE_SHORT) < TE_DELTA
                    {
                        self.add_bit(0);
                        self.step = Step::HighLevel;
                    } else {
                        self.step = Step::Start;
                    }
                } else {
                    self.step = Step::Start;
                }
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        let p = decrypt(decoded.data, decoded.extra?)?;
        if p.constant != CONSTANT {
            return None;
        }
        // Any 8-bit code is a native Beninca button (the C's button override masks to 8 bits).
        let mut p = Plain {
            cnt: next_counter(p.cnt, u32::MAX),
            btn: button,
            ..p
        };
        let mut out = Vec::with_capacity(TX_REPEAT * 3 * 258);
        let mut packets = Vec::with_capacity(3 * 258);
        for i in 0..3u64 {
            p.middle = (i + 1) * 2;
            let (data, data_2) = encrypt(&p)?;
            packet(data, data_2, &mut packets);
        }
        for _ in 0..TX_REPEAT {
            for ld in &packets {
                push(&mut out, ld.level, ld.duration_us);
            }
        }
        Some(out)
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        // `subghz_protocol_decoder_beninca_arc_deserialize`: Bit must be 128, Key + Data (8 bytes).
        // A frame without the 0xAA55 constant is refused (the C loads it and shows "Fx").
        if key.bits != MIN_COUNT_BIT {
            return None;
        }
        signal(key8(key)?, hex8_field(key, "Data")?).filter(|d| d.crc_valid)
    }

    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        // `subghz_protocol_decoder_beninca_arc_serialize`: generic block + "Data" = data_2.
        let mut k = FlipperKey::new(NAME, MIN_COUNT_BIT, decoded.data, 0, "");
        k.push_hex("Data", decoded.extra?, 8);
        Some(k)
    }
}

/// A key file for a random valid frame (oracle driver fixtures): `r` seeds the fields.
#[cfg(test)]
pub(super) fn fixture_key(r: u64) -> FlipperKey {
    let p = Plain {
        serial: (r >> 32) as u32,
        btn: [0x02, 0x04, 0x00, r as u8][(r >> 8) as usize % 4],
        middle: (r >> 13) & 0xFF_FFFF_FFFF,
        cnt: (r as u32).rotate_left(7),
        constant: CONSTANT,
    };
    let (data, data_2) = encrypt(&p).unwrap();
    let d = signal(data, data_2).unwrap();
    BenincaArcDecoder::new().to_flipper_key(&d).unwrap()
}

/// AES-128 (FIPS-197), byte-oriented like the C's `aes_common.c` (state = 16 bytes, column by
/// column). The C uses its own copy or the Flipper's hardware AES; both are plain AES-128-ECB.
mod aes {
    const SBOX: [u8; 256] = [
        0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab,
        0x76, 0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4,
        0x72, 0xc0, 0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71,
        0xd8, 0x31, 0x15, 0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2,
        0xeb, 0x27, 0xb2, 0x75, 0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6,
        0xb3, 0x29, 0xe3, 0x2f, 0x84, 0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb,
        0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf, 0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45,
        0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8, 0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5,
        0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2, 0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44,
        0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73, 0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a,
        0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb, 0xe0, 0x32, 0x3a, 0x0a, 0x49,
        0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79, 0xe7, 0xc8, 0x37, 0x6d,
        0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08, 0xba, 0x78, 0x25,
        0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a, 0x70, 0x3e,
        0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e, 0xe1,
        0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
        0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb,
        0x16,
    ];
    const RCON: [u8; 10] = [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36];

    const fn inverse(sbox: &[u8; 256]) -> [u8; 256] {
        let mut inv = [0u8; 256];
        let mut i = 0;
        while i < 256 {
            inv[sbox[i] as usize] = i as u8;
            i += 1;
        }
        inv
    }
    const INV_SBOX: [u8; 256] = inverse(&SBOX);

    fn xtime(x: u8) -> u8 {
        (x << 1) ^ ((x >> 7) * 0x1b)
    }

    /// GF(2^8) product (inverse MixColumns coefficients).
    fn mul(mut a: u8, mut b: u8) -> u8 {
        let mut p = 0;
        while b != 0 {
            if b & 1 != 0 {
                p ^= a;
            }
            a = xtime(a);
            b >>= 1;
        }
        p
    }

    /// `aes_key_expansion`: 11 round keys.
    pub(super) fn expand_key(key: &[u8; 16]) -> [u8; 176] {
        let mut rk = [0u8; 176];
        rk[..16].copy_from_slice(key);
        for i in 4..44 {
            let mut w = [
                rk[(i - 1) * 4],
                rk[(i - 1) * 4 + 1],
                rk[(i - 1) * 4 + 2],
                rk[(i - 1) * 4 + 3],
            ];
            if i % 4 == 0 {
                w = [
                    SBOX[w[1] as usize] ^ RCON[i / 4 - 1],
                    SBOX[w[2] as usize],
                    SBOX[w[3] as usize],
                    SBOX[w[0] as usize],
                ];
            }
            for j in 0..4 {
                rk[i * 4 + j] = w[j] ^ rk[(i - 4) * 4 + j];
            }
        }
        rk
    }

    fn add_round_key(s: &mut [u8; 16], rk: &[u8]) {
        s.iter_mut().zip(rk).for_each(|(a, b)| *a ^= b);
    }

    /// ShiftRows on the column-major state: row r (bytes r, r+4, r+8, r+12) rotates left by r.
    fn shift_rows(s: &mut [u8; 16], inverse: bool) {
        for r in 1..4 {
            let row = [s[r], s[r + 4], s[r + 8], s[r + 12]];
            for c in 0..4 {
                let from = if inverse {
                    (c + 4 - r) % 4
                } else {
                    (c + r) % 4
                };
                s[r + 4 * c] = row[from];
            }
        }
    }

    fn mix_columns(s: &mut [u8; 16], inverse: bool) {
        let m: [u8; 4] = if inverse {
            [14, 11, 13, 9]
        } else {
            [2, 3, 1, 1]
        };
        for c in 0..4 {
            let col = [s[4 * c], s[4 * c + 1], s[4 * c + 2], s[4 * c + 3]];
            for r in 0..4 {
                s[4 * c + r] = (0..4).fold(0, |acc, k| acc ^ mul(col[(r + k) % 4], m[k]));
            }
        }
    }

    pub(super) fn encrypt(rk: &[u8; 176], s: &mut [u8; 16]) {
        add_round_key(s, &rk[..16]);
        for round in 1..=10 {
            s.iter_mut().for_each(|b| *b = SBOX[*b as usize]);
            shift_rows(s, false);
            if round < 10 {
                mix_columns(s, false);
            }
            add_round_key(s, &rk[round * 16..round * 16 + 16]);
        }
    }

    pub(super) fn decrypt(rk: &[u8; 176], s: &mut [u8; 16]) {
        add_round_key(s, &rk[160..]);
        for round in (0..10).rev() {
            shift_rows(s, true);
            s.iter_mut().for_each(|b| *b = INV_SBOX[*b as usize]);
            add_round_key(s, &rk[round * 16..round * 16 + 16]);
            if round > 0 {
                mix_columns(s, true);
            }
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        fn hex16(s: &str) -> [u8; 16] {
            let mut b = [0u8; 16];
            for (i, x) in b.iter_mut().enumerate() {
                *x = u8::from_str_radix(&s[2 * i..2 * i + 2], 16).unwrap();
            }
            b
        }

        /// FIPS-197 Appendix B and C.1.
        #[test]
        fn fips_197_vectors() {
            for (key, plain, cipher) in [
                (
                    "2b7e151628aed2a6abf7158809cf4f3c",
                    "3243f6a8885a308d313198a2e0370734",
                    "3925841d02dc09fbdc118597196a0b32",
                ),
                (
                    "000102030405060708090a0b0c0d0e0f",
                    "00112233445566778899aabbccddeeff",
                    "69c4e0d86a7b0430d8cdb78070b4c55a",
                ),
            ] {
                let rk = expand_key(&hex16(key));
                let mut s = hex16(plain);
                encrypt(&rk, &mut s);
                assert_eq!(s, hex16(cipher));
                decrypt(&rk, &mut s);
                assert_eq!(s, hex16(plain));
            }
            // FIPS-197 A.1: the last round key of the Appendix B key.
            let rk = expand_key(&hex16("2b7e151628aed2a6abf7158809cf4f3c"));
            assert_eq!(rk[160..], hex16("d014f9a8c9ee2589e13f0cc8b6630ca6"));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::ProtocolRegistry;

    fn decode_all(signal: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = BenincaArcDecoder::new();
        let mut out = Vec::new();
        for ld in signal {
            if let Some(d) = dec.feed(ld.level, ld.duration_us) {
                out.push(d);
                dec.reset();
            }
        }
        out
    }

    fn frame(serial: u32, btn: u8, middle: u64, cnt: u32, constant: u16) -> (u64, u64) {
        encrypt(&Plain {
            serial,
            btn,
            middle,
            cnt,
            constant,
        })
        .unwrap()
    }

    /// A capture as a remote sends it: gap, 128 bits, stop bit, gap.
    fn capture(data: u64, data_2: u64) -> Vec<LevelDuration> {
        let mut s = vec![LevelDuration::new(false, 9600)];
        packet(data, data_2, &mut s);
        s
    }

    #[test]
    fn decodes_a_synthetic_frame() {
        let (data, data_2) = frame(0x1234_5678, 0x04, 0x02, 0x0001_2345, CONSTANT);
        let d = decode_all(&capture(data, data_2));
        assert_eq!(d.len(), 1);
        let d = &d[0];
        assert_eq!(
            (d.data, d.extra, d.data_count_bit),
            (data, Some(data_2), 128)
        );
        assert_eq!(
            (d.serial, d.button, d.counter),
            (Some(0x1234_5678), Some(0x04), Some(0x2345))
        );
        assert!(d.crc_valid && d.encoder_capable);
        assert_eq!(decrypt(data, data_2).unwrap().middle, 0x02);
    }

    #[test]
    fn rejects_frames_without_the_constant_and_bad_timing() {
        // Wrong constant: the C would report it (Fx shown); KAT does not.
        let (data, data_2) = frame(0x1234_5678, 0x04, 0x02, 7, 0xAA54);
        assert!(decode_all(&capture(data, data_2)).is_empty());
        // One flipped wire bit: AES diffuses it, the constant fails.
        let (data, data_2) = frame(0x1234_5678, 0x04, 0x02, 7, CONSTANT);
        assert!(decode_all(&capture(data ^ 1 << 17, data_2)).is_empty());
        // Gap outside 9600 ± 2325 µs.
        let mut s = capture(data, data_2);
        s[0].duration_us = 12_000;
        assert!(decode_all(&s).is_empty());
        // A bit of HIGH 444 + LOW 444 µs (a 1 needs LOW 600 ± 155, a 0 HIGH 600 ± 155) breaks it.
        let mut s = capture(data, data_2);
        let i = s
            .iter()
            .position(|l| l.level && l.duration_us == TE_SHORT)
            .unwrap();
        s[i].duration_us = 444;
        s[i + 1].duration_us = 444;
        assert!(decode_all(&s).is_empty());
    }

    #[test]
    fn encode_round_trip_bumps_counter_and_sets_button() {
        let dec = BenincaArcDecoder::new();
        let (data, data_2) = frame(0x00C0_FFEE, 0x02, 0x0A, 0x0000_FFFF, CONSTANT);
        let captured = decode_all(&capture(data, data_2)).pop().unwrap();
        for button in [0x02u8, 0x04, 0x00] {
            let wire = dec.encode(&captured, button).unwrap();
            assert_eq!(wire.len(), 3 * 258);
            assert_eq!(
                wire.last().map(|l| (l.level, l.duration_us)),
                Some((false, 9000))
            );
            // Packets 2 and 3 follow a 9000 µs gap; the first needs the gap before it.
            let mut with_gap = vec![LevelDuration::new(false, 9600)];
            with_gap.extend(wire.iter().copied());
            let got = decode_all(&with_gap);
            assert_eq!(got.len(), 3, "button {button:#x}");
            for (i, d) in got.iter().enumerate() {
                let p = decrypt(d.data, d.extra.unwrap()).unwrap();
                assert_eq!((p.serial, p.btn, p.cnt), (0x00C0_FFEE, button, 0x0001_0000));
                assert_eq!(p.middle, (i as u64 + 1) * 2);
                assert_eq!(d.counter, Some(0));
            }
        }
        // 32-bit wrap.
        let (data, data_2) = frame(0x1, 0x02, 0x02, u32::MAX, CONSTANT);
        let d = signal(data, data_2).unwrap();
        let wire = dec.encode(&d, 0x02).unwrap();
        let next = decode_all(&wire).pop().unwrap();
        assert_eq!(decrypt(next.data, next.extra.unwrap()).unwrap().cnt, 0);
        // No constant → no re-encode.
        let (data, data_2) = frame(0x1, 0x02, 0x02, 5, 0x1234);
        let bad = DecodedSignal {
            extra: Some(data_2),
            ..DecodedSignal::new(data, 128)
        };
        assert!(dec.encode(&bad, 0x02).is_none());
    }

    #[test]
    fn key_file_round_trip() {
        let dec = BenincaArcDecoder::new();
        let (data, data_2) = frame(0x0BAD_CAFE, 0x04, 0x01, 0x42, CONSTANT);
        let text = format!(
            "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
             Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Beninca ARC\nBit: 128\n\
             Key: {}\nData: {}\n",
            hexbytes(data),
            hexbytes(data_2)
        );
        let key = FlipperKey::parse(&text).unwrap();
        let d = dec.from_flipper_key(&key).unwrap();
        assert_eq!(
            (d.serial, d.button, d.counter),
            (Some(0x0BAD_CAFE), Some(0x04), Some(0x42))
        );
        assert!(d.crc_valid && d.encoder_capable);
        let back = dec.to_flipper_key(&d).unwrap();
        assert_eq!(back.protocol, "Beninca ARC");
        assert!(back.to_text().contains(&format!(
            "Bit: 128\nKey: {}\nData: {}\n",
            hexbytes(data),
            hexbytes(data_2)
        )));
        let again = dec
            .from_flipper_key(&FlipperKey::parse(&back.to_text()).unwrap())
            .unwrap();
        assert_eq!(format!("{again:?}"), format!("{d:?}"));
        // Wrong bit count / missing Data.
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&text.replace("Bit: 128", "Bit: 64")).unwrap())
            .is_none());
        let no_data: String = text
            .lines()
            .filter(|l| !l.starts_with("Data"))
            .map(|l| format!("{l}\n"))
            .collect();
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&no_data).unwrap())
            .is_none());
        // A frame without the 0xAA55 constant is refused.
        let (bad, bad_2) = frame(0x0BAD_CAFE, 0x04, 0x01, 0x42, 0x3FC8);
        let bad_text = text
            .replace(&hexbytes(data), &hexbytes(bad))
            .replace(&hexbytes(data_2), &hexbytes(bad_2));
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&bad_text).unwrap())
            .is_none());
        // The registry resolves the name.
        let reg = ProtocolRegistry::new();
        assert_eq!(
            reg.decode_key(&key).map(|(n, _)| n),
            Some("Beninca ARC".to_string())
        );
    }

    fn hexbytes(v: u64) -> String {
        v.to_be_bytes()
            .iter()
            .map(|b| format!("{b:02X}"))
            .collect::<Vec<_>>()
            .join(" ")
    }

    #[test]
    fn aes_key_is_big_then_little_endian() {
        let k = aes_key().unwrap();
        let mf = keystore_entry(|e| e.name == "Beninca_ARC").unwrap().key;
        assert_eq!(k[..8], mf.to_be_bytes());
        let mut rev = mf.to_be_bytes();
        rev.reverse();
        assert_eq!(k[8..], rev);
    }

    #[test]
    fn registry_reports_beninca_frames() {
        let (data, data_2) = frame(0x1357_9BDF, 0x02, 0x04, 99, CONSTANT);
        let s = capture(data, data_2);
        for f in crate::protocols::test_frequencies(crate::protocols::ANY_FREQUENCY) {
            let mut reg = ProtocolRegistry::new();
            let got = reg.process_signal_stream(&s, f);
            let names: Vec<&str> = got.iter().map(|(n, _, _)| n.as_str()).collect();
            assert_eq!(names, ["Beninca ARC"], "{f}");
        }
    }
}
