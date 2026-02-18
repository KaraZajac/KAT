//! Generic KeeLoq fallback decoder.
//!
//! When no known protocol decodes a signal, we try to decode it as KeeLoq using every
//! manufacturer key in the keystore. We support two common air formats (bit layout + timing):
//! - **Kia V3/V4 format**: 68 bits, 400/800µs PWM, 315/433 MHz
//! - **Star Line format**: 64 bits, 250/500µs PWM, 433 MHz
//!
//! All decryption is done via [keeloq_common]; this module only orchestrates bit collection
//! (delegated to the format-specific collectors) and tries each key in both byte orders
//! (as stored, and byte-swapped) in case the source used big- or little-endian.

use super::common::DecodedSignal;
use super::keeloq_common::{keeloq_decrypt, keeloq_normal_learning, reverse_key, reverse8};
use super::kia_v3_v4;
use super::star_line;
use crate::keystore;
use crate::radio::demodulator::LevelDuration;

const KIA_V3_V4_BITS: usize = 68;
const STAR_LINE_BITS: usize = 64;

/// Try to decode an unknown signal as KeeLoq using every keystore manufacturer key.
/// Tries both Kia V3/V4 format (68-bit, 400/800µs) and Star Line format (64-bit, 250/500µs)
/// regardless of frequency; each key is tried in both byte orders (LE and BE) until one validates.
/// Returns `("Keeloq (keystore name)", decoded)` on first successful decrypt.
pub fn try_decode(
    pairs: &[LevelDuration],
    _frequency: u32,
) -> Option<(String, DecodedSignal)> {
    let keys = keystore::keeloq_mf_keys_with_names();
    if keys.is_empty() {
        return None;
    }

    // Kia V3/V4 format: 400/800µs, 68 bits (try both polarities, all keys)
    for invert in [false, true] {
        if let Some((buf, is_v3)) = kia_v3_v4::collect_kia_v3_v4_bits(pairs, invert) {
            if let Some((name, decoded)) = try_kia_v3_v4_format(&buf, is_v3, &keys) {
                return Some((format!("Keeloq ({})", name), decoded));
            }
        }
    }

    // Star Line format: 250/500µs, 64 bits (try both polarities, all keys)
    for invert in [false, true] {
        if let Some(data) = star_line::collect_star_line_bits(pairs, invert) {
            if let Some((name, decoded)) = try_star_line_format(data, &keys) {
                return Some((format!("Keeloq ({})", name), decoded));
            }
        }
    }

    None
}

/// Try Kia V3/V4 68-bit format with each key. Uses [keeloq_common::keeloq_decrypt] for validation.
fn try_kia_v3_v4_format(
    raw_bits: &[u8; 9],
    is_v3_sync: bool,
    keys: &[(String, u64)],
) -> Option<(String, DecodedSignal)> {
    let mut b = *raw_bits;
    if is_v3_sync {
        for i in 0..9 {
            b[i] = !b[i];
        }
    }
    let encrypted = ((reverse8(b[3]) as u32) << 24)
        | ((reverse8(b[2]) as u32) << 16)
        | ((reverse8(b[1]) as u32) << 8)
        | (reverse8(b[0]) as u32);
    let serial = ((reverse8(b[7] & 0xF0) as u32) << 24)
        | ((reverse8(b[6]) as u32) << 16)
        | ((reverse8(b[5]) as u32) << 8)
        | (reverse8(b[4]) as u32);
    let button = (reverse8(b[7]) & 0xF0) >> 4;
    let our_serial_lsb = (serial & 0xFF) as u8;
    let key_data = ((b[0] as u64) << 56)
        | ((b[1] as u64) << 48)
        | ((b[2] as u64) << 40)
        | ((b[3] as u64) << 32)
        | ((b[4] as u64) << 24)
        | ((b[5] as u64) << 16)
        | ((b[6] as u64) << 8)
        | (b[7] as u64);

    for (name, mf_key) in keys {
        for key in [*mf_key, mf_key.swap_bytes()] {
            if key == 0 {
                continue;
            }
            let decrypted = keeloq_decrypt(encrypted, key);
            let dec_btn = ((decrypted >> 28) & 0x0F) as u8;
            let dec_serial_lsb = ((decrypted >> 16) & 0xFF) as u8;
            if dec_btn == button && dec_serial_lsb == our_serial_lsb {
                let counter = (decrypted & 0xFFFF) as u16;
                return Some((
                    name.clone(),
                    DecodedSignal {
                        serial: Some(serial),
                        button: Some(button),
                        counter: Some(counter),
                        crc_valid: true,
                        data: key_data,
                        data_count_bit: KIA_V3_V4_BITS,
                        encoder_capable: true,
                        extra: None,
                        protocol_display_name: None,
                    },
                ));
            }
        }
    }
    None
}

/// Try Star Line 64-bit format with each key. Uses [keeloq_common::keeloq_decrypt] and
/// [keeloq_common::keeloq_normal_learning] for validation.
fn try_star_line_format(
    data: u64,
    keys: &[(String, u64)],
) -> Option<(String, DecodedSignal)> {
    let reversed = reverse_key(data, STAR_LINE_BITS);
    let key_fix = (reversed >> 32) as u32;
    let key_hop = (reversed & 0xFFFFFFFF) as u32;
    let serial = key_fix & 0x00FFFFFF;
    let btn = (key_fix >> 24) as u8;
    let serial_lsb = (serial & 0xFF) as u8;

    for (name, mf_key) in keys {
        for key in [*mf_key, mf_key.swap_bytes()] {
            if key == 0 {
                continue;
            }
            // Simple learning
            let decrypt = keeloq_decrypt(key_hop, key);
            let dec_btn = (decrypt >> 24) as u8;
            let dec_serial_lsb = ((decrypt >> 16) & 0xFF) as u8;
            if dec_btn == btn && dec_serial_lsb == serial_lsb {
                let counter = (decrypt & 0xFFFF) as u16;
                return Some((
                    name.clone(),
                    DecodedSignal {
                        serial: Some(serial),
                        button: Some(btn),
                        counter: Some(counter),
                        crc_valid: true,
                        data,
                        data_count_bit: STAR_LINE_BITS,
                        encoder_capable: true,
                        extra: None,
                        protocol_display_name: None,
                    },
                ));
            }
            // Normal learning
            let man_key = keeloq_normal_learning(key_fix, key);
            let decrypt = keeloq_decrypt(key_hop, man_key);
            let dec_btn = (decrypt >> 24) as u8;
            let dec_serial_lsb = ((decrypt >> 16) & 0xFF) as u8;
            if dec_btn == btn && dec_serial_lsb == serial_lsb {
                let counter = (decrypt & 0xFFFF) as u16;
                return Some((
                    name.clone(),
                    DecodedSignal {
                        serial: Some(serial),
                        button: Some(btn),
                        counter: Some(counter),
                        crc_valid: true,
                        data,
                        data_count_bit: STAR_LINE_BITS,
                        encoder_capable: true,
                        extra: None,
                        protocol_display_name: None,
                    },
                ));
            }
        }
    }
    None
}
