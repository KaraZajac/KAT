//! Scher-Khan (Magic Code / Magicar) protocol decoder + encoder
//!
//! References (union of capabilities):
//! - ProtoPirate `REFERENCES/ProtoPirate/protocols/scher_khan.c` — decoder + variant table, no encoder.
//! - Flipper-ARF `REFERENCES/Flipper-ARF/lib/subghz/protocols/scher_khan.c` — the same decoder plus
//!   MAGIC CODE PRO1/PRO2 (57-bit) decryption with its CRC check, and an encoder (51-bit Dynamic and
//!   57-bit PRO1/PRO2 forward-encode, every other length sent verbatim). doctor-firmware carries the
//!   plain decode-only Flipper copy.
//!
//! **Decoder** (`subghz_protocol_decoder_scher_khan_feed`, identical in PP and ARF): PWM, te_short
//! 750 µs, te_long 1100 µs, te_delta 160 µs. A HIGH of 1500 ± 160 µs enters the header; header pairs
//! of 1500 µs (or 750 µs) pulses count `header_count` on each LOW whose preceding HIGH was ~1500 µs;
//! a ~750 µs HIGH + LOW pair after ≥ 2 header pairs is the start bit (counted as bit 1, value 0).
//! Data: 750/750 = 0, 1100/1100 = 1. A HIGH ≥ 1420 µs (te_long + 2·te_delta) ends the frame, which is
//! reported when it holds ≥ 35 bits (any length — there is no other gate, as in the C).
//!
//! **Variants** (`subghz_protocol_scher_khan_check_remote_controller`, by bit count; shown as the
//! display-name suffix `Scher-Khan [<variant>]`, PP's "Pt:" line):
//! - 35 "MAGIC CODE, Static"; 63 "MAGIC CODE, Response"; 64 "MAGICAR, Response"; 81/82
//!   "MAGIC CODE PRO, Response"; anything else "Unknown" — no fields (the C zeroes them).
//! - 51 "MAGIC CODE, Dynamic": serial = `((data >> 24) & 0xFFFFFF0) | ((data >> 20) & 0xF)`,
//!   button = `(data >> 24) & 0xF`, counter = `data & 0xFFFF`.
//! - 57 "MAGIC CODE PRO1"/"PRO2" (ARF only): bytes `kb[i] = data >> (48 - 8i)`, i = 0..6. The CRC-8
//!   (init 0xFF over kb[0..6], polynomial from the π-obfuscated table) must equal kb[6]: PRO1
//!   polynomial first, then PRO2. The matching 64-byte key table drives a bit-permutation cipher with a
//!   rolling packet counter (`scher_khan_pro_decrypt`); serial = d0:d1:d2, button = d3 & 0xF,
//!   counter = d4:d5. No CRC match → "MAGIC CODE PRO/PRO2", no fields, `crc_valid = false`.
//!
//! Frames longer than 64 bits follow the KAT convention (first 64 wire bits in `data`, the rest
//! right-aligned in `extra`; the start bit is the leading 0 wire bit). The C keeps only the last 64
//! data bits in its `uint64_t`, so its encoder cannot rebuild them either: such frames are not
//! encoder-capable.
//!
//! **Encoder** (ARF `subghz_protocol_encoder_scher_khan_get_upload`). Button and counter are
//! re-derived from the key, as the C's deserialize does (`check_remote_controller` overwrites the
//! file's Btn/Cnt). Counter + 1, wrapping to 0 past 0xFFFF (rolling-counter multiplier 1).
//! - 51 bits: `data = (bits 16..50 of data, with bits 24..27 = button) | counter`.
//!   **KAT deviation (encoder fix):** ARF masks with `0x7FFF_FFFF_0000`, which keeps only bits
//!   16..46 and so clears the top of the serial (bits 47..50) — the C's own sample frame
//!   0x0E8C1619E830C (bit 47 set) would be sent as 0x068C…, i.e. as a different remote. KAT keeps
//!   bits 16..50. ARF also leaves the inverted-button nibble (bits 16..19 = `!button`; the sample
//!   carries button 0x1 with nibble 0xE) stale when the button changes; when the captured frame
//!   follows that convention KAT rewrites the nibble with the new button (other frames keep it).
//! - 57 bits: CRC with the PRO1 polynomial decides PRO1, anything else is treated as PRO2; decrypt,
//!   `d3 = (d3 & 0xE0) | button` (bit 4 is cleared, as in the C), `d4:d5 = counter`, re-encrypt,
//!   new CRC, keep 57 bits.
//! - Any other length ≤ 64 bits: the key is sent unchanged (replay).
//! Upload: 6 × (750 HIGH, 750 LOW), 3 × (1500 HIGH, 1500 LOW), start bit (750, 750), the
//! `data_count_bit` low bits of `data` MSB-first (1 = 1100/1100, 0 = 750/750), then a 1500 µs HIGH;
//! the C yields it 7 times (`encoder.repeat = 7`), so the KAT waveform holds 7 copies. Note the
//! encoder sends the start bit AND the key's leading zero bit, so its frames decode one bit longer
//! (52 / 58 bits) than the capture — reference behaviour, kept.
//!
//! **Buttons** (ARF `scher_khan_btn_name`): 1 Lock, 2 Unlock, 3 Lock+Unlock, 4 Trunk, 5 Lock+Trunk,
//! 6 Unlock+Trunk, 7 Lk+Ul+Tr, 8 Start, 9 Lock+Start, A Unlock+Start, C Trunk+Start, F All/Panic.
//! KAT's generic Lock/Unlock/Trunk/Panic map to 1/2/4/F; other native codes 1..F pass through; any
//! other value keeps the captured button.
//!
//! 315 + 433.92 MHz, FM/AM (ARF flags; PP: 433 FM). Reference: https://phreakerclub.com/72

use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

const TE_SHORT: u32 = 750;
const TE_LONG: u32 = 1100;
const TE_DELTA: u32 = 160;
const MIN_COUNT_BIT: usize = 35;
/// C `encoder.repeat`.
const TX_REPEAT: usize = 7;

/// ARF `sk_pi_bytes`: π digits used to de-obfuscate the PRO key tables and CRC polynomials.
const SK_PI_BYTES: [u8; 146] = [
    0x24, 0x3F, 0x6A, 0x88, 0x85, 0xA3, 0x08, 0xD3, 0x13, 0x19, 0x8A, 0x2E, 0x03, 0x70, 0x73, 0x44,
    0xA4, 0x09, 0x38, 0x22, 0x29, 0x9F, 0x31, 0xD0, 0x08, 0x2E, 0xFA, 0x98, 0xEC, 0x4E, 0x6C, 0x89,
    0x45, 0x28, 0x21, 0xE6, 0x38, 0xD0, 0x13, 0x77, 0xBE, 0x54, 0x66, 0xCF, 0x34, 0xE9, 0x0C, 0x6C,
    0xC0, 0xAC, 0x29, 0xB7, 0xC9, 0x7C, 0x50, 0xDD, 0x3F, 0x84, 0xD5, 0xB5, 0xB5, 0x47, 0x09, 0x17,
    0x92, 0x16, 0xD5, 0xD9, 0x89, 0x79, 0xFB, 0x1B, 0xD1, 0x31, 0x0B, 0xA6, 0x98, 0xDF, 0xB5, 0xAC,
    0x2F, 0xFD, 0x72, 0xDB, 0xD0, 0x1A, 0xDF, 0xB7, 0xB8, 0xE1, 0xAF, 0xED, 0x6A, 0x26, 0x7E, 0x96,
    0xBA, 0x7C, 0x90, 0x45, 0xF1, 0x2C, 0x7F, 0x99, 0x24, 0xA1, 0x99, 0x47, 0xB3, 0x91, 0x6C, 0xF7,
    0x08, 0x01, 0xF2, 0xE2, 0x85, 0x8E, 0xFC, 0x16, 0x63, 0x69, 0x20, 0xD8, 0x71, 0x57, 0x4E, 0x69,
    0xA4, 0x58, 0xFE, 0xA3, 0xF4, 0x93, 0x3D, 0x7E, 0x0D, 0x95, 0x74, 0x8F, 0x72, 0x8E, 0xB6, 0x58,
    0x71, 0x8B,
];

/// ARF `sk_pro1_encoded` (XOR π bytes 0..64).
const SK_PRO1_ENCODED: [u8; 64] = [
    0x63, 0x5A, 0x58, 0x98, 0xD6, 0xB3, 0x7E, 0x91, 0x37, 0x1E, 0xBB, 0x78, 0x43, 0x06, 0x20, 0x65,
    0xC7, 0x4B, 0x3F, 0x73, 0x0D, 0xAE, 0x36, 0x86, 0x1E, 0x7A, 0xDA, 0xEB, 0xBB, 0x0F, 0x5A, 0xA9,
    0x26, 0x3A, 0x51, 0xB2, 0x7A, 0xE0, 0x44, 0x61, 0xAC, 0x53, 0x02, 0xFA, 0x42, 0xBD, 0x2D, 0x6F,
    0x85, 0xBB, 0x4A, 0xB5, 0xF9, 0x28, 0x46, 0xAF, 0x2B, 0xB4, 0xB2, 0x90, 0xD0, 0x05, 0x19, 0x64,
];

/// ARF `sk_pro2_encoded` (XOR π bytes 64..128).
const SK_PRO2_ENCODED: [u8; 64] = [
    0x90, 0x27, 0xA3, 0x9C, 0xEA, 0x6B, 0x8B, 0x4F, 0x91, 0x47, 0x58, 0x87, 0x8A, 0xD8, 0xD1, 0x99,
    0x3B, 0xCD, 0x15, 0xFE, 0xF4, 0x1D, 0xEE, 0xE1, 0xBF, 0xA4, 0x9E, 0xCB, 0x1C, 0x72, 0x5F, 0x95,
    0xAC, 0x28, 0xB0, 0x36, 0xE3, 0x2B, 0x1B, 0xAC, 0x30, 0x91, 0xFE, 0x62, 0xD0, 0xD3, 0x6B, 0xA6,
    0x4F, 0x64, 0xC0, 0xF2, 0xD2, 0xCF, 0xCA, 0x36, 0x53, 0x3D, 0x36, 0xAA, 0x33, 0x67, 0x19, 0x7F,
];

/// ARF `sk_pro_crc_encoded` (XOR π bytes 144..146): CRC polynomials [PRO1, PRO2].
const SK_PRO_CRC_ENCODED: [u8; 2] = [0x5B, 0x8C];

/// `sk_pi_decode`.
const fn pi_decode<const N: usize>(encoded: &[u8; N], offset: usize) -> [u8; N] {
    let mut out = [0u8; N];
    let mut i = 0;
    while i < N {
        out[i] = encoded[i] ^ SK_PI_BYTES[offset + i];
        i += 1;
    }
    out
}

const PRO1_TABLE: [u8; 64] = pi_decode(&SK_PRO1_ENCODED, 0);
const PRO2_TABLE: [u8; 64] = pi_decode(&SK_PRO2_ENCODED, 64);
const CRC_POLYS: [u8; 2] = pi_decode(&SK_PRO_CRC_ENCODED, 144);

// KAT generic button codes.
const BTN_LOCK: u8 = 0x01;
const BTN_UNLOCK: u8 = 0x02;
const BTN_TRUNK: u8 = 0x04;
const BTN_PANIC: u8 = 0x08;
/// ARF "All/Panic".
const SK_PANIC: u8 = 0x0F;

/// `scher_khan_pro_checksum`: CRC-8, init 0xFF, MSB-first over the first 6 bytes.
fn pro_checksum(data: &[u8; 7], poly: u8) -> u8 {
    let mut cs: u8 = 0xFF;
    for &byte in &data[..6] {
        let mut c = byte;
        for _ in 0..8 {
            let prev = cs;
            cs <<= 1;
            if (prev ^ c) & 0x80 != 0 {
                cs ^= poly;
            }
            c <<= 1;
        }
    }
    cs
}

/// `scher_khan_pro_decrypt` (byte 6, the CRC, is not decrypted: the C leaves it 0).
fn pro_decrypt(encrypted: &[u8; 7], key_table: &[u8; 64]) -> [u8; 7] {
    let mut decrypted = [0u8; 7];
    decrypted[0] = encrypted[0];
    let mut pkt_cnt = (encrypted[0] & 0x0F) as usize;
    for i in 1..6 {
        let s = encrypted[i];
        for j in 0..4 {
            let cr = key_table[(pkt_cnt << 2) + j];
            let bit = 0x80u8 >> (cr >> 4);
            if s & bit == bit {
                decrypted[i] |= 0x80 >> (j << 1);
            }
            let bit = 0x80u8 >> (cr & 0x0F);
            if s & bit == bit {
                decrypted[i] |= 0x80 >> ((j << 1) + 1);
            }
        }
        pkt_cnt = (pkt_cnt + 1) & 0x0F;
        decrypted[i] ^= encrypted[i - 1];
    }
    decrypted
}

/// `scher_khan_pro_encrypt` (inverse of [`pro_decrypt`] on bytes 1..5; byte 6 comes out 0).
fn pro_encrypt(plain: &[u8; 7], key_table: &[u8; 64]) -> [u8; 7] {
    let mut src = *plain;
    let mut encrypted = [0u8; 7];
    encrypted[0] = src[0];
    let mut pkt_cnt = (src[0] & 0x0F) as usize;
    for i in 1..6 {
        src[i] ^= encrypted[i - 1];
        let s = src[i];
        for j in 0..4 {
            let cr = key_table[(pkt_cnt << 2) + j];
            let bit = 0x80u8 >> (j << 1);
            if s & bit == bit {
                encrypted[i] |= 0x80 >> (cr >> 4);
            }
            let bit = 0x80u8 >> ((j << 1) + 1);
            if s & bit == bit {
                encrypted[i] |= 0x80 >> (cr & 0x0F);
            }
        }
        pkt_cnt = (pkt_cnt + 1) & 0x0F;
    }
    encrypted
}

/// The 7 key bytes of a 57-bit frame (`kb[i] = data >> (48 - 8i)`).
fn key_bytes(data: u64) -> [u8; 7] {
    let mut kb = [0u8; 7];
    for (i, b) in kb.iter_mut().enumerate() {
        *b = (data >> (48 - i * 8)) as u8;
    }
    kb
}

/// Which PRO table decrypted a 57-bit frame.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Pro {
    Pro1,
    Pro2,
}

/// Fields recovered by `check_remote_controller`: (variant, serial, button, counter, crc_ok).
struct Parsed {
    variant: &'static str,
    fields: Option<(u32, u8, u16)>,
    crc_ok: bool,
}

/// 57-bit PRO decode: CRC with the PRO1 then the PRO2 polynomial; on a match, decrypt.
fn pro_decode(data: u64) -> Option<(Pro, [u8; 7])> {
    let kb = key_bytes(data);
    if pro_checksum(&kb, CRC_POLYS[0]) == kb[6] {
        return Some((Pro::Pro1, pro_decrypt(&kb, &PRO1_TABLE)));
    }
    if pro_checksum(&kb, CRC_POLYS[1]) == kb[6] {
        return Some((Pro::Pro2, pro_decrypt(&kb, &PRO2_TABLE)));
    }
    None
}

/// `subghz_protocol_scher_khan_check_remote_controller` (ARF, a superset of PP's).
fn parse(data: u64, bit_count: usize) -> Parsed {
    let none = |variant| Parsed {
        variant,
        fields: None,
        crc_ok: true,
    };
    match bit_count {
        35 => none("MAGIC CODE, Static"),
        51 => Parsed {
            variant: "MAGIC CODE, Dynamic",
            fields: Some((
                (((data >> 24) & 0xFFF_FFF0) | ((data >> 20) & 0x0F)) as u32,
                ((data >> 24) & 0x0F) as u8,
                (data & 0xFFFF) as u16,
            )),
            crc_ok: true,
        },
        57 => match pro_decode(data) {
            Some((pro, d)) => Parsed {
                variant: match pro {
                    Pro::Pro1 => "MAGIC CODE PRO1",
                    Pro::Pro2 => "MAGIC CODE PRO2",
                },
                fields: Some((
                    (u32::from(d[0]) << 16) | (u32::from(d[1]) << 8) | u32::from(d[2]),
                    d[3] & 0x0F,
                    (u16::from(d[4]) << 8) | u16::from(d[5]),
                )),
                crc_ok: true,
            },
            None => Parsed {
                variant: "MAGIC CODE PRO/PRO2",
                fields: None,
                crc_ok: false,
            },
        },
        63 => none("MAGIC CODE, Response"),
        64 => none("MAGICAR, Response"),
        // C: "MAGIC CODE PRO,\n Response" (the newline is display wrapping).
        81 | 82 => none("MAGIC CODE PRO, Response"),
        _ => none("Unknown"),
    }
}

/// KAT generic button → Scher-Khan code (see the module doc).
fn map_button(button: u8, original: u8) -> u8 {
    match button {
        BTN_LOCK | BTN_UNLOCK | BTN_TRUNK => button,
        BTN_PANIC => SK_PANIC,
        0x01..=0x0F => button,
        _ => original,
    }
}

/// ARF counter policy: + rolling multiplier (1), 0 once it would pass 0xFFFF.
fn next_counter(cnt: u16) -> u16 {
    cnt.checked_add(1).unwrap_or(0)
}

/// The data half of ARF `subghz_protocol_encoder_scher_khan_get_upload`: the key to send.
fn encode_key(data: u64, bit_count: usize, button: u8) -> u64 {
    match bit_count {
        51 => {
            let parsed = parse(data, bit_count);
            let (_, orig_btn, cnt) = parsed.fields.unwrap_or((0, 0, 0));
            let btn = map_button(button, orig_btn);
            let cnt = next_counter(cnt);
            // KAT deviation (see module doc): keep all of bits 16..50 (ARF's 0x7FFF_FFFF_0000 drops
            // serial bits 47..50) and keep the inverted-button nibble consistent with the button.
            let mut upper = data & 0x0007_FFFF_FFFF_0000;
            let inverted_nibble = ((data >> 16) & 0x0F) as u8 == (!orig_btn & 0x0F);
            upper = (upper & !(0x0Fu64 << 24)) | (u64::from(btn & 0x0F) << 24);
            if inverted_nibble {
                upper = (upper & !(0x0Fu64 << 16)) | (u64::from(!btn & 0x0F) << 16);
            }
            upper | u64::from(cnt)
        }
        57 => {
            let parsed = parse(data, bit_count);
            // Unmatched CRC: the C's check_remote_controller zeroes btn/cnt.
            let (_, orig_btn, cnt) = parsed.fields.unwrap_or((0, 0, 0));
            let btn = map_button(button, orig_btn);
            let cnt = next_counter(cnt);
            let kb = key_bytes(data);
            let (table, poly) = if pro_checksum(&kb, CRC_POLYS[0]) == kb[6] {
                (&PRO1_TABLE, CRC_POLYS[0])
            } else {
                (&PRO2_TABLE, CRC_POLYS[1])
            };
            let mut d = pro_decrypt(&kb, table);
            d[3] = (d[3] & 0xE0) | (btn & 0x0F);
            d[4] = (cnt >> 8) as u8;
            d[5] = cnt as u8;
            let mut enc = pro_encrypt(&d, table);
            enc[6] = pro_checksum(&enc, poly);
            let mut out = 0u64;
            for (i, &b) in enc.iter().enumerate() {
                out |= u64::from(b) << (48 - i * 8);
            }
            out & 0x01FF_FFFF_FFFF_FFFF
        }
        _ => data,
    }
}

/// One copy of the C upload for `bit_count` bits of `data`.
fn upload_once(data: u64, bit_count: usize, signal: &mut Vec<LevelDuration>) {
    for _ in 0..6 {
        signal.push(LevelDuration::new(true, TE_SHORT));
        signal.push(LevelDuration::new(false, TE_SHORT));
    }
    for _ in 0..3 {
        signal.push(LevelDuration::new(true, TE_SHORT * 2));
        signal.push(LevelDuration::new(false, TE_SHORT * 2));
    }
    signal.push(LevelDuration::new(true, TE_SHORT));
    signal.push(LevelDuration::new(false, TE_SHORT));
    for i in (0..bit_count).rev() {
        let te = if (data >> i) & 1 == 1 { TE_LONG } else { TE_SHORT };
        signal.push(LevelDuration::new(true, te));
        signal.push(LevelDuration::new(false, te));
    }
    signal.push(LevelDuration::new(true, TE_SHORT * 2));
}

/// Decoder states (matches ScherKhanDecoderStep)
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    CheckPreamble,
    SaveDuration,
    CheckDuration,
}

/// Scher-Khan protocol decoder
pub struct ScherKhanDecoder {
    step: DecoderStep,
    te_last: u32,
    header_count: u16,
    /// Data bits shifted in (the C's `decode_data`, widened so >64-bit frames keep their head).
    decode_data: u128,
    decode_count_bit: usize,
}

impl ScherKhanDecoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            te_last: 0,
            header_count: 0,
            decode_data: 0,
            decode_count_bit: 0,
        }
    }

    fn add_bit(&mut self, bit: u128) {
        self.decode_data = (self.decode_data << 1) | bit;
        self.decode_count_bit += 1;
    }

    /// Build the reported signal for a completed frame.
    fn build(bits: u128, bit_count: usize) -> DecodedSignal {
        // ≤ 64 bits: `data` is the C's decode_data. Longer: KAT convention (first 64 wire bits +
        // the rest right-aligned in `extra`); beyond 128 bits only the C's last 64 bits survive.
        let (data, extra) = if bit_count <= 64 || bit_count > 128 {
            (bits as u64, None)
        } else {
            let rest = bit_count - 64;
            (
                (bits >> rest) as u64,
                Some((bits & ((1u128 << rest) - 1)) as u64),
            )
        };
        let parsed = parse(data, bit_count);
        let (serial, button, counter) = match parsed.fields {
            Some((s, b, c)) => (Some(s), Some(b), Some(c)),
            None => (None, None, None),
        };
        DecodedSignal {
            serial,
            button,
            counter,
            crc_valid: parsed.crc_ok,
            data,
            data_count_bit: bit_count,
            encoder_capable: bit_count <= 64,
            extra,
            protocol_display_name: Some(format!("Scher-Khan [{}]", parsed.variant)),
        }
    }
}

impl ProtocolDecoder for ScherKhanDecoder {
    fn name(&self) -> &'static str {
        "Scher-Khan"
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
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.te_last = 0;
        self.header_count = 0;
        self.decode_data = 0;
        self.decode_count_bit = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::Reset => {
                if level && duration_diff!(duration, TE_SHORT * 2) < TE_DELTA {
                    self.step = DecoderStep::CheckPreamble;
                    self.te_last = duration;
                    self.header_count = 0;
                }
            }

            DecoderStep::CheckPreamble => {
                if level {
                    if duration_diff!(duration, TE_SHORT * 2) < TE_DELTA
                        || duration_diff!(duration, TE_SHORT) < TE_DELTA
                    {
                        self.te_last = duration;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else if duration_diff!(duration, TE_SHORT * 2) < TE_DELTA
                    || duration_diff!(duration, TE_SHORT) < TE_DELTA
                {
                    if duration_diff!(self.te_last, TE_SHORT * 2) < TE_DELTA {
                        // Found header
                        self.header_count = self.header_count.wrapping_add(1);
                    } else if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA {
                        // Found start bit
                        if self.header_count >= 2 {
                            self.step = DecoderStep::SaveDuration;
                            self.decode_data = 0;
                            self.decode_count_bit = 1;
                        } else {
                            self.step = DecoderStep::Reset;
                        }
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::SaveDuration => {
                if level {
                    if duration >= TE_DELTA * 2 + TE_LONG {
                        // Found stop bit
                        self.step = DecoderStep::Reset;
                        let result = (self.decode_count_bit >= MIN_COUNT_BIT)
                            .then(|| Self::build(self.decode_data, self.decode_count_bit));
                        self.decode_data = 0;
                        self.decode_count_bit = 0;
                        return result;
                    } else {
                        self.te_last = duration;
                        self.step = DecoderStep::CheckDuration;
                    }
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::CheckDuration => {
                if !level {
                    if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA
                        && duration_diff!(duration, TE_SHORT) < TE_DELTA
                    {
                        self.add_bit(0);
                        self.step = DecoderStep::SaveDuration;
                    } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA
                        && duration_diff!(duration, TE_LONG) < TE_DELTA
                    {
                        self.add_bit(1);
                        self.step = DecoderStep::SaveDuration;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else {
                    self.step = DecoderStep::Reset;
                }
            }
        }

        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        let bit_count = decoded.data_count_bit;
        if !(MIN_COUNT_BIT..=64).contains(&bit_count) {
            return None;
        }
        let key = encode_key(decoded.data, bit_count, button);
        let mut signal = Vec::with_capacity(TX_REPEAT * (21 + bit_count * 2));
        for _ in 0..TX_REPEAT {
            upload_once(key, bit_count, &mut signal);
        }
        Some(signal)
    }
}

impl Default for ScherKhanDecoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn decode_all(signal: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = ScherKhanDecoder::new();
        let mut out = Vec::new();
        for ld in signal {
            if let Some(d) = dec.feed(ld.level, ld.duration_us) {
                out.push(d);
                dec.reset(); // registry behaviour
            }
        }
        out
    }

    /// A capture-shaped signal: header + start bit + `bit_count - 1` data bits (the capture's
    /// leading wire bit is the start bit), stop HIGH.
    fn capture(data: u64, bit_count: usize) -> Vec<LevelDuration> {
        let mut s = Vec::new();
        for _ in 0..3 {
            s.push(LevelDuration::new(true, 1500));
            s.push(LevelDuration::new(false, 1500));
        }
        s.push(LevelDuration::new(true, 750));
        s.push(LevelDuration::new(false, 750));
        for i in (0..bit_count - 1).rev() {
            let te = if (data >> i) & 1 == 1 { 1100 } else { 750 };
            s.push(LevelDuration::new(true, te));
            s.push(LevelDuration::new(false, te));
        }
        s.push(LevelDuration::new(true, 1500));
        s
    }

    /// Build a valid PRO frame (57 bits) from plaintext bytes, the way the C encoder does.
    fn pro_frame(pro: Pro, plain: [u8; 7]) -> u64 {
        let (table, poly) = match pro {
            Pro::Pro1 => (&PRO1_TABLE, CRC_POLYS[0]),
            Pro::Pro2 => (&PRO2_TABLE, CRC_POLYS[1]),
        };
        let mut enc = pro_encrypt(&plain, table);
        enc[6] = pro_checksum(&enc, poly);
        let mut out = 0u64;
        for (i, &b) in enc.iter().enumerate() {
            out |= u64::from(b) << (48 - i * 8);
        }
        out & 0x01FF_FFFF_FFFF_FFFF
    }

    #[test]
    fn scher_khan_51bit_reference_sample() {
        // The C's own sample (MAGICAR 51-bit MAGIC CODE, Dynamic): 0E8C1619E830C.
        let d = decode_all(&capture(0x0E8C1619E830C, 51));
        assert_eq!(d.len(), 1);
        let d = &d[0];
        assert_eq!(d.data, 0x0E8C1619E830C);
        assert_eq!(d.data_count_bit, 51);
        assert_eq!(d.button, Some(0x1));
        assert_eq!(d.counter, Some(0x830C));
        assert_eq!(d.serial, Some(((0x0E8C1619E830Cu64 >> 24) & 0xFFFFFF0) as u32 | 0x9));
        assert_eq!(
            d.protocol_display_name.as_deref(),
            Some("Scher-Khan [MAGIC CODE, Dynamic]")
        );
    }

    #[test]
    fn scher_khan_pro_crypto_roundtrip() {
        let mut s = 0x1234_5678_9ABC_DEF0u64;
        for _ in 0..2000 {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let mut plain = [0u8; 7];
            for (i, b) in plain.iter_mut().enumerate().take(6) {
                *b = (s >> (i * 8 + 3)) as u8;
            }
            for (pro, table) in [(Pro::Pro1, &PRO1_TABLE), (Pro::Pro2, &PRO2_TABLE)] {
                let frame = pro_frame(pro, plain);
                let kb = key_bytes(frame);
                let dec = pro_decrypt(&kb, table);
                assert_eq!(&dec[..6], &plain[..6], "{pro:?}");
                // The CRC check picks the right table (PRO1 is tried first).
                let got = pro_decode(frame).map(|(p, _)| p);
                if pro == Pro::Pro1 {
                    assert_eq!(got, Some(Pro::Pro1));
                } else {
                    assert!(got.is_some());
                }
            }
        }
    }

    #[test]
    fn scher_khan_pro_frames_decode_with_fields() {
        let plain = [0x01, 0x23, 0x45, 0x64, 0xBE, 0xEF, 0x00];
        for (pro, name) in [(Pro::Pro1, "MAGIC CODE PRO1"), (Pro::Pro2, "MAGIC CODE PRO2")] {
            let frame = pro_frame(pro, plain);
            let d = decode_all(&capture(frame, 57));
            assert_eq!(d.len(), 1, "{pro:?}");
            let d = &d[0];
            if pro_decode(frame).map(|(p, _)| p) != Some(pro) {
                continue; // a PRO2 frame whose CRC also matches the PRO1 polynomial
            }
            assert!(d.crc_valid);
            assert_eq!(d.serial, Some(0x012345));
            assert_eq!(d.button, Some(0x4));
            assert_eq!(d.counter, Some(0xBEEF));
            assert_eq!(
                d.protocol_display_name.as_deref(),
                Some(format!("Scher-Khan [{name}]").as_str())
            );
        }
        // Broken CRC → "MAGIC CODE PRO/PRO2", no fields, crc_valid false.
        let mut frame = pro_frame(Pro::Pro1, plain);
        frame ^= 1;
        if pro_decode(frame).is_none() {
            let d = decode_all(&capture(frame, 57)).pop().unwrap();
            assert!(!d.crc_valid);
            assert_eq!(d.serial, None);
            assert_eq!(
                d.protocol_display_name.as_deref(),
                Some("Scher-Khan [MAGIC CODE PRO/PRO2]")
            );
        }
    }

    #[test]
    fn scher_khan_encoder_matches_reference_layout() {
        let dec = ScherKhanDecoder::new();
        let captured = decode_all(&capture(0x0E8C1619E830C, 51)).pop().unwrap();
        let signal = dec.encode(&captured, BTN_UNLOCK).unwrap();
        // 7 copies of 6 + 3 + 1 pairs, 51 bit pairs and a 1500 µs end HIGH.
        let per_copy = 2 * (6 + 3 + 1 + 51) + 1;
        assert_eq!(signal.len(), TX_REPEAT * per_copy);
        // Each copy decodes; the start bit plus the key's leading zero make it 52 bits (C quirk).
        let decodes = decode_all(&signal);
        assert_eq!(decodes.len(), TX_REPEAT);
        let d = &decodes[0];
        assert_eq!(d.data_count_bit, 52);
        // Serial bits kept (incl. bit 47, which ARF's 0x7FFFFFFF0000 mask would drop), new button 0x2
        // with its inverted nibble 0xD, counter + 1.
        let expect = 0x0E8C_1629_D830D_u64;
        assert_eq!(d.data, expect);
        assert_eq!(encode_key(0x0E8C1619E830C, 51, BTN_UNLOCK), expect);
        // The re-encoded key parses back to the capture's serial (the 52-bit air frame itself has no
        // fields: the C's start-bit quirk, kept).
        let back = parse(expect, 51).fields.unwrap();
        let orig = parse(0x0E8C1619E830C, 51).fields.unwrap();
        assert_eq!(back.0, orig.0, "serial must survive re-encoding");
        assert_eq!((back.1, back.2), (0x2, 0x830D));
    }

    #[test]
    fn scher_khan_51bit_keeps_nibble_when_frame_does_not_invert_button() {
        // A 51-bit frame whose bits 16..19 are not !button keeps them untouched on re-encode.
        let data = (0x0E8C1619E830Cu64 & !(0xF << 16)) | (0x5 << 16);
        let out = encode_key(data, 51, BTN_UNLOCK);
        assert_eq!((out >> 16) & 0xF, 0x5);
        assert_eq!((out >> 24) & 0xF, 0x2);
        assert_eq!(out >> 28, data >> 28, "upper serial bits preserved");
    }

    #[test]
    fn scher_khan_encoder_pro_roundtrip() {
        let dec = ScherKhanDecoder::new();
        let plain = [0x00, 0x7A, 0x51, 0x21, 0x00, 0x41, 0x00];
        let frame = pro_frame(Pro::Pro1, plain);
        assert_eq!(pro_decode(frame).map(|(p, _)| p), Some(Pro::Pro1));
        let captured = decode_all(&capture(frame, 57)).pop().unwrap();
        for (button, code) in [(BTN_LOCK, 1u8), (BTN_TRUNK, 4), (BTN_PANIC, 0xF), (0x09, 9), (0x40, 1)] {
            let key = encode_key(captured.data, 57, button);
            let (pro, d) = pro_decode(key).expect("re-encoded frame has a valid CRC");
            assert_eq!(pro, Pro::Pro1);
            assert_eq!(&d[..3], &plain[..3], "serial kept");
            assert_eq!(d[3], (plain[3] & 0xE0) | code, "button {button:#x}");
            assert_eq!((u16::from(d[4]) << 8) | u16::from(d[5]), 0x0042, "counter + 1");
            let decodes = decode_all(&dec.encode(&captured, button).unwrap());
            assert_eq!(decodes.len(), TX_REPEAT);
            assert_eq!(decodes[0].data, key);
            assert_eq!(decodes[0].data_count_bit, 58);
        }
        // Counter wraps to 0 past 0xFFFF.
        let frame = pro_frame(Pro::Pro1, [0x00, 0x7A, 0x51, 0x21, 0xFF, 0xFF, 0x00]);
        let (_, d) = pro_decode(encode_key(frame, 57, BTN_LOCK)).unwrap();
        assert_eq!((d[4], d[5]), (0, 0));
    }

    #[test]
    fn scher_khan_other_lengths_replay_and_long_frames() {
        let dec = ScherKhanDecoder::new();
        let captured = decode_all(&capture(0x2_DEAD_BEEF, 35)).pop().unwrap();
        assert_eq!(captured.serial, None);
        assert_eq!(
            captured.protocol_display_name.as_deref(),
            Some("Scher-Khan [MAGIC CODE, Static]")
        );
        assert_eq!(encode_key(captured.data, 35, BTN_LOCK), captured.data);
        assert!(dec.encode(&captured, BTN_LOCK).is_some());
        // 81-bit response frame: first 64 wire bits + 17-bit tail in `extra`, not encodable.
        let mut s = Vec::new();
        for _ in 0..3 {
            s.push(LevelDuration::new(true, 1500));
            s.push(LevelDuration::new(false, 1500));
        }
        s.push(LevelDuration::new(true, 750));
        s.push(LevelDuration::new(false, 750));
        let tail: u128 = 0xA5A5_1234_5678_9ABC_DEF0; // 80 data bits
        for i in (0..80).rev() {
            let te = if (tail >> i) & 1 == 1 { 1100 } else { 750 };
            s.push(LevelDuration::new(true, te));
            s.push(LevelDuration::new(false, te));
        }
        s.push(LevelDuration::new(true, 1500));
        let d = decode_all(&s).pop().unwrap();
        assert_eq!(d.data_count_bit, 81);
        assert_eq!(d.data, (tail >> 17) as u64);
        assert_eq!(d.extra, Some((tail & 0x1FFFF) as u64));
        assert!(!d.encoder_capable);
        assert!(dec.encode(&d, BTN_LOCK).is_none());
    }

    #[test]
    fn scher_khan_short_frames_are_ignored() {
        assert!(decode_all(&capture(0x1234, 34)).is_empty());
        assert_eq!(decode_all(&capture(0x1234, 35)).len(), 1);
    }
}
