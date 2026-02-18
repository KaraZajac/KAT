//! Common utilities for protocol implementations.
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/protocols_common.c`
//! and `protocols_common.h`. The reference provides only Flipper preset name mapping
//! (`protopirate_get_short_preset_name`); we implement the same mapping in `short_preset_name`.
//! This module also holds shared types and helpers used by multiple protocol decoders.
//!
//! **CRC / add_bit**: `crc8_kia` matches `kia_crc8` in kia_v0.c (polynomial 0x7F, init 0x00);
//! `add_bit` matches the common shift-left-and-append pattern used in the reference decoders.
//!
//! **Manchester**: Ford V0 and Fiat V0 each have their own state machine in their protocol
//! modules. This module provides CommonManchesterState / common_manchester_advance for
//! protocols that use the Flipper-style event mapping (0=ShortLow, 1=ShortHigh, 2=LongLow,
//! 3=LongHigh) and want the shared implementation.

/// Decoded signal information
#[derive(Debug, Clone)]
pub struct DecodedSignal {
    /// Serial number / device ID
    pub serial: Option<u32>,
    /// Button code
    pub button: Option<u8>,
    /// Rolling counter
    pub counter: Option<u16>,
    /// CRC is valid
    pub crc_valid: bool,
    /// Raw data (up to 64 bits)
    pub data: u64,
    /// Number of bits in data
    pub data_count_bit: usize,
    /// Whether encoding is supported
    pub encoder_capable: bool,
    /// Protocol-specific extra data for encoding (e.g. VAG: vag_type + key_idx)
    pub extra: Option<u64>,
    /// Optional protocol display name (e.g. "KeeLoq (DoorHan)"). When set, used as the protocol name for this decode.
    pub protocol_display_name: Option<String>,
}

impl DecodedSignal {
    #[allow(dead_code)]
    pub fn new(data: u64, bit_count: usize) -> Self {
        Self {
            serial: None,
            button: None,
            counter: None,
            crc_valid: false,
            data,
            data_count_bit: bit_count,
            encoder_capable: false,
            extra: None,
            protocol_display_name: None,
        }
    }
}

/// CRC8 calculation with custom polynomial (MSB-first, shift-left style).
///
/// # Arguments
/// * `data` - Data bytes to calculate CRC over
/// * `poly` - CRC polynomial (e.g. 0x7F for Kia)
/// * `init` - Initial CRC value (e.g. 0x00)
pub fn crc8(data: &[u8], poly: u8, init: u8) -> u8 {
    let mut crc = init;
    for &byte in data {
        crc ^= byte;
        for _ in 0..8 {
            if (crc & 0x80) != 0 {
                crc = (crc << 1) ^ poly;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

/// CRC8 for Kia protocol (matches kia_v0.c kia_crc8: polynomial 0x7F, init 0x00)
pub fn crc8_kia(data: &[u8]) -> u8 {
    crc8(data, 0x7F, 0x00)
}

/// Add a bit to the decoder's data accumulator (shift-left, LSB last; matches reference add_bit pattern)
#[inline]
pub fn add_bit(data: &mut u64, count: &mut usize, bit: bool) {
    *data = (*data << 1) | (bit as u64);
    *count += 1;
}

// =============================================================================
// Preset name mapping (protocols_common.c: protopirate_get_short_preset_name)
// =============================================================================

/// Short preset name for display. Matches `protopirate_get_short_preset_name`.
/// Returns "UNKNOWN" for null/empty or unknown preset names (reference returns the
/// original pointer for unknown; we use "UNKNOWN" to avoid allocation).
#[allow(dead_code)]
#[inline]
pub fn short_preset_name(preset: Option<&str>) -> &'static str {
    let p = match preset {
        None => return "UNKNOWN",
        Some(s) if s.is_empty() => return "UNKNOWN",
        Some(s) => s,
    };
    match p {
        "FuriHalSubGhzPresetOok270Async" => "AM270",
        "FuriHalSubGhzPresetOok650Async" => "AM650",
        "FuriHalSubGhzPreset2FSKDev238Async" => "FM238",
        "FuriHalSubGhzPreset2FSKDev12KAsync" => "FM12K",
        "FuriHalSubGhzPreset2FSKDev476Async" => "FM476",
        "FuriHalSubGhzPresetCustom" => "CUSTOM",
        _ => "UNKNOWN",
    }
}

// =============================================================================
// Common Manchester state machine (Flipper lib/toolbox/manchester_decoder)
// =============================================================================
// ProtoPirate Ford and Fiat use Flipper's lib/toolbox/manchester_decoder.h:
// ManchesterState (Mid0, Mid1, Start0, Start1), ManchesterEvent (ShortLow,
// ShortHigh, LongLow, LongHigh, Reset), manchester_advance(state, event, &state, &bit).
// This module provides a separate implementation of the same transition table for
// protocols that want the shared behaviour. Ford and Fiat keep their own
// FordV0ManchesterState and FiatV0ManchesterState in their modules (same table, no reuse).
//
// Event encoding: 0=ShortLow, 1=ShortHigh, 2=LongLow, 3=LongHigh.
// Level mapping in ProtoPirate: level ? ShortLow : ShortHigh (and same for long).

/// Manchester decoder states for the common (Flipper-style) differential Manchester decoder.
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CommonManchesterState {
    Mid0 = 0,
    Mid1 = 1,
    Start0 = 2,
    Start1 = 3,
}

/// Advance the common Manchester state machine by one event.
/// Returns `(new_state, Some(bit))` when a data bit is emitted, otherwise `(new_state, None)`.
/// Event: 0=ShortLow, 1=ShortHigh, 2=LongLow, 3=LongHigh (level ? ShortLow : ShortHigh for short/long).
#[allow(dead_code)]
#[inline]
pub fn common_manchester_advance(
    state: CommonManchesterState,
    event: u8,
) -> (CommonManchesterState, Option<bool>) {
    use CommonManchesterState::{Mid0, Mid1, Start0, Start1};

    let (new_state, emit) = match (state, event) {
        (Mid0, 0) => (Mid0, false),
        (Mid0, 1) => (Start1, true),
        (Mid0, 2) => (Mid0, false),
        (Mid0, 3) => (Mid1, true),

        (Mid1, 0) => (Start0, true),
        (Mid1, 1) => (Mid1, false),
        (Mid1, 2) => (Mid0, true),
        (Mid1, 3) => (Mid1, false),

        (Start0, 0) => (Mid0, false),
        (Start0, 1) => (Mid0, false),
        (Start0, 2) => (Mid0, false),
        (Start0, 3) => (Mid1, false),

        (Start1, 0) => (Mid0, false),
        (Start1, 1) => (Mid1, false),
        (Start1, 2) => (Mid0, false),
        (Start1, 3) => (Mid1, false),

        _ => (Mid1, false),
    };

    let bit = if emit { Some((event & 1) == 1) } else { None };
    (new_state, bit)
}

/// Button names for common keyfob buttons
#[allow(dead_code)]
pub fn get_button_name(btn: u8) -> &'static str {
    match btn {
        0x01 => "Lock",
        0x02 => "Unlock",
        0x03 => "Lock+Unlock",
        0x04 => "Trunk",
        0x08 => "Panic",
        _ => "Unknown",
    }
}

/// Button code constants
#[allow(dead_code)]
pub mod buttons {
    pub const LOCK: u8 = 0x01;
    pub const UNLOCK: u8 = 0x02;
    pub const TRUNK: u8 = 0x04;
    pub const PANIC: u8 = 0x08;
}
