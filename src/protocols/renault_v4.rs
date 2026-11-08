//! Renault/xHorse V4 protocol decoder (decode-only)
//!
//! Ported from FoxFW: `REFERENCES/FoxFW/lib/subghz/protocols/renault_v4.c`
//! (`RENAULT_V4_PROTOCOL_NAME = "Renault/xHorse V4"`, `433 | 315 | AM`, Dynamic, no encoder). It is
//! the aftermarket Nissan / xHorse universal key fob, not one of KAT's Hitag2-style "Renault V0" /
//! "Renault V1" and not the encrypted "Renault V2" / "Renault V3".
//!
//! # Wire format (AM/OOK, Manchester 125/250 µs, tolerance < 70 µs)
//! A HIGH pulse of at least 800 µs syncs; then Manchester bits through the C's own 4-state machine
//! (`rv4_man_decode`; events short/long x high/low, **not** Flipper's `manchester_decoder.c`
//! table). Any pulse that is neither short nor long ends the burst and runs `process`: the decoded
//! bits start with at least 10 identical preamble bits (tried as ones, then zeros), one start bit,
//! then 88..=92 bits of which the first 88 form the frame.
//!
//! # Frame (11 bytes, MSB first)
//! - bytes 0..3: 32-bit serial
//! - byte 4 high nibble: button; bits 3..2: pad, must be `0b01`; bits 1..0 + byte 5 bits 7..2: the
//!   clear 8-bit counter (+1 per press)
//! - the rest is opaque; the **XOR of all 11 bytes must be 0xFF**
//!
//! # Gating (as in the C)
//! XOR == 0xFF, button nibble one of `0xE 0xD 0xB 0x7`, pad == 1. The C also drops a frame
//! identical to the previous one until `reset()` (`last_key`): within one registry pass an
//! immediate repeat is not reported twice.
//!
//! # DecodedSignal packing
//! `data` = frame bytes 0..7, `data_count_bit` = 88, `extra` = bytes 8..10 (the C's `Data2`),
//! `serial`, `button` (nibble), `counter` (8 bit). `crc_valid` = true: the XOR check passed.
//!
//! # Key files
//! `serialize` writes `Protocol: Renault/xHorse V4`, `Bit: 88`, `Key` = `data`, then `Serial`, `Btn`,
//! `Cnt` and `Data2` (bytes 8..10, decimal). `deserialize` needs exactly 88 bits, derives serial, button
//! and counter from the key and reads `Data2` (masked to 24 bits, 0 when absent); it does not repeat
//! the XOR check (the C sets `crc_ok` true).
//!
//! # Encoder
//! None (the C's `.encoder` is NULL).

use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Renault/xHorse V4",
    decoder: "Renault/xHorse V4",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Rolling (clear counter)",
    make: "Renault",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&AM650],
};

const TE_SHORT: u32 = 125;
const TE_LONG: u32 = 250;
const TE_DELTA: u32 = 70;
const SYNC_MIN: u32 = 800;
const DATA_BITS: usize = 88;
const ARR_SIZE: usize = 128;

/// `rv4_man_decode`: the C's custom Manchester machine. `r1`: 0 short high, 1 short low, 2 long
/// high, 3 long low. Returns the bit when one was produced.
fn man_decode(man: &mut u8, r1: u8) -> Option<u8> {
    let s = *man;
    match r1 {
        0 => match s {
            0 | 2 => {
                *man = 1;
                None
            }
            1 => {
                *man = 0;
                Some(0)
            }
            3 => {
                *man = 2;
                Some(0)
            }
            _ => None,
        },
        1 => match s {
            0 | 2 => {
                *man = 3;
                None
            }
            1 => {
                *man = 2;
                Some(1)
            }
            3 => {
                *man = 0;
                Some(1)
            }
            _ => None,
        },
        2 => match s {
            0 | 2 => {
                *man = 0;
                Some(0)
            }
            1 | 3 => {
                *man = 1;
                None
            }
            _ => None,
        },
        3 => match s {
            0 | 2 => {
                *man = 2;
                Some(1)
            }
            1 | 3 => {
                *man = 3;
                None
            }
            _ => None,
        },
        _ => None,
    }
}

/// The decode of a frame from its first 8 bytes (`data`) and last 3 (`tail`): serial, button
/// nibble and the clear counter come from the key.
fn signal(data: u64, tail: u64) -> DecodedSignal {
    let frame4 = (data >> 24) as u8;
    let frame5 = (data >> 16) as u8;
    DecodedSignal {
        serial: Some((data >> 32) as u32),
        button: Some((frame4 >> 4) & 0x0F),
        counter: Some((((frame4 & 0x03) << 6) | ((frame5 >> 2) & 0x3F)) as u16),
        crc_valid: true,
        data,
        data_count_bit: DATA_BITS,
        encoder_capable: false,
        extra: Some(tail),
        protocol_display_name: None,
    }
}

/// Renault/xHorse V4 decoder (matches `SubGhzProtocolDecoderRenaultV4`).
pub struct RenaultV4Decoder {
    synced: bool,
    man_state: u8,
    bits: [u8; ARR_SIZE],
    bit_count: usize,
    last_key: [u8; 11],
    last_valid: bool,
}

impl RenaultV4Decoder {
    pub fn new() -> Self {
        Self {
            synced: false,
            man_state: 1,
            bits: [0; ARR_SIZE],
            bit_count: 0,
            last_key: [0; 11],
            last_valid: false,
        }
    }

    /// `renault_v4_process`: preamble scan, length gate, XOR / button / pad checks.
    fn process(&mut self) -> Option<DecodedSignal> {
        let n = self.bit_count as i32;
        for inv in 0..2 {
            let pv: u8 = if inv == 1 { 0 } else { 1 };
            let mut pre = 0usize;
            while (pre as i32) < n && self.bits[pre] == pv {
                pre += 1;
            }
            if pre < 10 {
                continue;
            }
            let ds = pre + 1;
            let dlen = n - ds as i32;
            if dlen < DATA_BITS as i32 || dlen > DATA_BITS as i32 + 4 {
                continue;
            }
            let mut frame = [0u8; 12];
            for i in 0..DATA_BITS {
                if self.bits[ds + i] != 0 {
                    frame[i >> 3] |= 0x80 >> (i & 7);
                }
            }
            let xr = frame[..11].iter().fold(0u8, |a, b| a ^ b);
            if xr != 0xFF {
                continue;
            }
            let btn = (frame[4] >> 4) & 0x0F;
            if !matches!(btn, 0xE | 0xD | 0xB | 0x7) {
                continue;
            }
            if (frame[4] >> 2) & 0x03 != 0x01 {
                continue; // pad invariant
            }
            // De-duplicate an identical frame (the C keeps "last" until reset()).
            let mut key = [0u8; 11];
            key.copy_from_slice(&frame[..11]);
            if self.last_valid && self.last_key == key {
                return None;
            }
            self.last_key = key;
            self.last_valid = true;

            let data = u64::from_be_bytes(frame[..8].try_into().unwrap());
            let tail = ((frame[8] as u64) << 16) | ((frame[9] as u64) << 8) | frame[10] as u64;
            return Some(signal(data, tail));
        }
        None
    }
}

impl Default for RenaultV4Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for RenaultV4Decoder {
    fn name(&self) -> &'static str {
        "Renault/xHorse V4"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: DATA_BITS,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        // C flags: SubGhzProtocolFlag_433 | SubGhzProtocolFlag_315 | AM.
        &[433_920_000, 315_000_000]
    }

    fn reset(&mut self) {
        self.synced = false;
        self.man_state = 1;
        self.bit_count = 0;
        self.last_valid = false;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        if !self.synced {
            if level && duration >= SYNC_MIN {
                self.synced = true;
                self.bit_count = 0;
                self.man_state = 1;
            }
            return None;
        }

        let is_short = duration_diff!(duration, TE_SHORT) < TE_DELTA;
        let is_long = duration_diff!(duration, TE_LONG) < TE_DELTA;
        if is_short || is_long {
            let r1 = match (is_long, level) {
                (false, true) => 0,
                (false, false) => 1,
                (true, true) => 2,
                (true, false) => 3,
            };
            if let Some(bit) = man_decode(&mut self.man_state, r1) {
                if self.bit_count < ARR_SIZE {
                    self.bits[self.bit_count] = bit;
                    self.bit_count += 1;
                }
            }
            None
        } else {
            let out = self.process();
            if level && duration >= SYNC_MIN {
                self.bit_count = 0;
                self.man_state = 1;
            } else {
                self.synced = false;
            }
            out
        }
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }

    /// `subghz_protocol_decoder_renault_v4_deserialize` (see "Key files").
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != DATA_BITS {
            return None;
        }
        let tail = key.u32_field("Data2").unwrap_or(0) & 0x00FF_FFFF;
        Some(signal(key.key_u64()?, tail as u64))
    }

    /// `subghz_protocol_decoder_renault_v4_serialize`.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != DATA_BITS {
            return None;
        }
        let mut key = FlipperKey::for_protocol("Renault/xHorse V4", DATA_BITS, decoded.data);
        key.push_u32("Serial", decoded.serial?);
        key.push_u32("Btn", decoded.button?.into());
        key.push_u32("Cnt", decoded.counter?.into());
        key.push_u32("Data2", decoded.extra? as u32);
        Some(key)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Test vectors: made-up field values, built as pulse sequences the way the C expects them, written
    // as Flipper RAW files and decoded by the FoxFW C decoder (host oracle, `--registry all
    // --protocol "Renault/xHorse V4"`); the expectations below are the fields the oracle printed. Levels
    // alternate from `.0`.

    /// `rv4_ok`: first pulse HIGH, 152 pulses.
    const RV4_OK: (bool, &[u32]) = (
        true,
        &[
        1000, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125,
        125, 125, 125, 125, 125, 125, 125, 125, 250, 125, 125, 125, 125, 125, 125, 250,
        250, 125, 125, 250, 250, 250, 250, 250, 250, 250, 250, 250, 125, 125, 250, 125,
        125, 250, 125, 125, 250, 250, 250, 125, 125, 250, 125, 125, 250, 125, 125, 250,
        125, 125, 250, 250, 125, 125, 125, 125, 125, 125, 250, 125, 125, 250, 250, 250,
        250, 250, 250, 250, 125, 125, 125, 125, 250, 125, 125, 250, 250, 250, 125, 125,
        250, 125, 125, 250, 250, 125, 125, 250, 125, 125, 125, 125, 125, 125, 250, 125,
        125, 250, 250, 125, 125, 250, 125, 125, 250, 250, 125, 125, 125, 125, 250, 250,
        125, 125, 125, 125, 250, 125, 125, 125, 125, 250, 125, 125, 250, 125, 125, 125,
        125, 250, 250, 250, 125, 125, 125, 3000,
        ],
    );

    /// `rv4_ok2`: first pulse HIGH, 166 pulses.
    const RV4_OK2: (bool, &[u32]) = (
        true,
        &[
        1000, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125,
        125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 250, 125, 125, 125,
        125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 250, 125, 125,
        125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 250, 125, 125, 250, 250, 250,
        125, 125, 250, 250, 250, 125, 125, 125, 125, 125, 125, 250, 125, 125, 250, 250,
        125, 125, 250, 250, 125, 125, 125, 125, 250, 250, 125, 125, 125, 125, 250, 125,
        125, 125, 125, 125, 125, 250, 125, 125, 250, 125, 125, 125, 125, 250, 125, 125,
        250, 250, 250, 250, 125, 125, 125, 125, 125, 125, 250, 125, 125, 250, 125, 125,
        125, 125, 125, 125, 125, 125, 125, 125, 250, 250, 250, 125, 125, 125, 125, 250,
        250, 125, 125, 125, 125, 250, 125, 125, 250, 250, 125, 125, 250, 250, 250, 250,
        250, 250, 250, 250, 125, 3000,
        ],
    );

    /// `rv4_bad_xor`: first pulse HIGH, 154 pulses.
    const RV4_BAD_XOR: (bool, &[u32]) = (
        true,
        &[
        1000, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125,
        125, 125, 125, 125, 125, 125, 125, 125, 250, 125, 125, 125, 125, 125, 125, 250,
        250, 125, 125, 250, 250, 250, 250, 250, 250, 250, 250, 250, 125, 125, 250, 125,
        125, 250, 125, 125, 250, 250, 250, 125, 125, 250, 125, 125, 250, 125, 125, 250,
        125, 125, 250, 250, 125, 125, 125, 125, 125, 125, 250, 125, 125, 250, 250, 250,
        250, 250, 250, 250, 125, 125, 125, 125, 250, 125, 125, 250, 250, 250, 125, 125,
        250, 125, 125, 250, 250, 125, 125, 250, 125, 125, 125, 125, 125, 125, 250, 125,
        125, 250, 250, 125, 125, 250, 125, 125, 250, 250, 125, 125, 125, 125, 250, 250,
        125, 125, 125, 125, 250, 125, 125, 125, 125, 250, 250, 125, 125, 125, 125, 125,
        125, 125, 125, 125, 125, 125, 125, 125, 125, 3000,
        ],
    );

    /// `rv4_flip`: first pulse HIGH, 150 pulses.
    const RV4_FLIP: (bool, &[u32]) = (
        true,
        &[
        1000, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125,
        125, 125, 125, 125, 125, 125, 125, 125, 250, 125, 125, 125, 125, 125, 125, 250,
        250, 125, 125, 250, 250, 250, 250, 250, 250, 250, 250, 250, 125, 125, 250, 125,
        125, 250, 125, 125, 250, 250, 250, 125, 125, 250, 125, 125, 250, 125, 125, 250,
        125, 125, 250, 250, 125, 125, 125, 125, 125, 125, 250, 125, 125, 250, 250, 250,
        250, 250, 250, 250, 125, 125, 125, 125, 250, 125, 125, 250, 250, 250, 125, 125,
        250, 125, 125, 250, 250, 125, 125, 250, 125, 125, 250, 250, 250, 125, 125, 250,
        250, 125, 125, 250, 125, 125, 250, 250, 125, 125, 125, 125, 250, 250, 125, 125,
        125, 125, 250, 125, 125, 125, 125, 250, 125, 125, 250, 125, 125, 125, 125, 250,
        250, 250, 125, 125, 125, 3000,
        ],
    );

    /// `rv4_bad_btn`: first pulse HIGH, 150 pulses.
    const RV4_BAD_BTN: (bool, &[u32]) = (
        true,
        &[
        1000, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125,
        125, 125, 125, 125, 125, 125, 125, 125, 250, 125, 125, 125, 125, 125, 125, 250,
        250, 125, 125, 250, 250, 250, 250, 250, 250, 250, 250, 250, 125, 125, 250, 125,
        125, 250, 125, 125, 250, 250, 250, 125, 125, 250, 125, 125, 250, 125, 125, 250,
        125, 125, 250, 250, 125, 125, 125, 125, 250, 125, 125, 125, 125, 250, 250, 250,
        250, 250, 250, 250, 125, 125, 125, 125, 250, 125, 125, 250, 250, 250, 125, 125,
        250, 125, 125, 250, 250, 125, 125, 250, 125, 125, 125, 125, 125, 125, 250, 125,
        125, 250, 250, 125, 125, 250, 125, 125, 250, 250, 125, 125, 125, 125, 250, 250,
        125, 125, 125, 125, 250, 125, 125, 125, 125, 250, 125, 125, 250, 250, 250, 250,
        250, 250, 125, 125, 125, 3000,
        ],
    );

    /// `rv4_bad_pad`: first pulse HIGH, 154 pulses.
    const RV4_BAD_PAD: (bool, &[u32]) = (
        true,
        &[
        1000, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125,
        125, 125, 125, 125, 125, 125, 125, 125, 250, 125, 125, 125, 125, 125, 125, 250,
        250, 125, 125, 250, 250, 250, 250, 250, 250, 250, 250, 250, 125, 125, 250, 125,
        125, 250, 125, 125, 250, 250, 250, 125, 125, 250, 125, 125, 250, 125, 125, 250,
        125, 125, 250, 250, 125, 125, 125, 125, 125, 125, 250, 250, 250, 125, 125, 250,
        250, 250, 250, 250, 125, 125, 125, 125, 250, 125, 125, 250, 250, 250, 125, 125,
        250, 125, 125, 250, 250, 125, 125, 250, 125, 125, 125, 125, 125, 125, 250, 125,
        125, 250, 250, 125, 125, 250, 125, 125, 250, 250, 125, 125, 125, 125, 250, 250,
        125, 125, 125, 125, 250, 125, 125, 125, 125, 250, 125, 125, 250, 125, 125, 125,
        125, 125, 125, 250, 125, 125, 125, 125, 125, 3000,
        ],
    );

    /// `rv4_short`: first pulse HIGH, 150 pulses.
    const RV4_SHORT: (bool, &[u32]) = (
        true,
        &[
        1000, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125,
        125, 125, 125, 125, 125, 125, 125, 125, 250, 125, 125, 125, 125, 125, 125, 250,
        250, 125, 125, 250, 250, 250, 250, 250, 250, 250, 250, 250, 125, 125, 250, 125,
        125, 250, 125, 125, 250, 250, 250, 125, 125, 250, 125, 125, 250, 125, 125, 250,
        125, 125, 250, 250, 125, 125, 125, 125, 125, 125, 250, 125, 125, 250, 250, 250,
        250, 250, 250, 250, 125, 125, 125, 125, 250, 125, 125, 250, 250, 250, 125, 125,
        250, 125, 125, 250, 250, 125, 125, 250, 125, 125, 125, 125, 125, 125, 250, 125,
        125, 250, 250, 125, 125, 250, 125, 125, 250, 250, 125, 125, 125, 125, 250, 250,
        125, 125, 125, 125, 250, 125, 125, 125, 125, 250, 125, 125, 250, 125, 125, 125,
        125, 250, 250, 250, 125, 3000,
        ],
    );

    /// `rv4_nopre`: first pulse HIGH, 146 pulses.
    const RV4_NOPRE: (bool, &[u32]) = (
        true,
        &[
        1000, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125,
        125, 125, 250, 125, 125, 125, 125, 125, 125, 250, 250, 125, 125, 250, 250, 250,
        250, 250, 250, 250, 250, 250, 125, 125, 250, 125, 125, 250, 125, 125, 250, 250,
        250, 125, 125, 250, 125, 125, 250, 125, 125, 250, 125, 125, 250, 250, 125, 125,
        125, 125, 125, 125, 250, 125, 125, 250, 250, 250, 250, 250, 250, 250, 125, 125,
        125, 125, 250, 125, 125, 250, 250, 250, 125, 125, 250, 125, 125, 250, 250, 125,
        125, 250, 125, 125, 125, 125, 125, 125, 250, 125, 125, 250, 250, 125, 125, 250,
        125, 125, 250, 250, 125, 125, 125, 125, 250, 250, 125, 125, 125, 125, 250, 125,
        125, 125, 125, 250, 125, 125, 250, 125, 125, 125, 125, 250, 250, 250, 125, 125,
        125, 3000,
        ],
    );

    /// `rv4_nosync`: first pulse HIGH, 152 pulses.
    const RV4_NOSYNC: (bool, &[u32]) = (
        true,
        &[
        700, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125,
        125, 125, 125, 125, 125, 125, 125, 125, 250, 125, 125, 125, 125, 125, 125, 250,
        250, 125, 125, 250, 250, 250, 250, 250, 250, 250, 250, 250, 125, 125, 250, 125,
        125, 250, 125, 125, 250, 250, 250, 125, 125, 250, 125, 125, 250, 125, 125, 250,
        125, 125, 250, 250, 125, 125, 125, 125, 125, 125, 250, 125, 125, 250, 250, 250,
        250, 250, 250, 250, 125, 125, 125, 125, 250, 125, 125, 250, 250, 250, 125, 125,
        250, 125, 125, 250, 250, 125, 125, 250, 125, 125, 125, 125, 125, 125, 250, 125,
        125, 250, 250, 125, 125, 250, 125, 125, 250, 250, 125, 125, 125, 125, 250, 250,
        125, 125, 125, 125, 250, 125, 125, 125, 125, 250, 125, 125, 250, 125, 125, 125,
        125, 250, 250, 250, 125, 125, 125, 3000,
        ],
    );

    fn run(v: (bool, &[u32])) -> Vec<DecodedSignal> {
        let mut dec = RenaultV4Decoder::new();
        let mut level = v.0;
        let mut out = Vec::new();
        for &d in v.1 {
            if let Some(s) = dec.feed(level, d) {
                out.push(s);
            }
            level = !level;
        }
        out
    }

    #[test]
    fn decodes_the_oracle_frames() {
        assert_eq!(run(RV4_OK), vec![DecodedSignal { serial: Some(0x12ab34cd), button: Some(0xe), counter: Some(0x57), crc_valid: true, data: 0x12ab34cde55cb279, data_count_bit: 88, encoder_capable: false, extra: Some(0x37718b), protocol_display_name: None }], "rv4_ok");
        assert_eq!(run(RV4_OK2), vec![DecodedSignal { serial: Some(0xfe5a1b), button: Some(0x7), counter: Some(0xc3), crc_valid: true, data: 0x00fe5a1b770c6bcf, data_count_bit: 88, encoder_capable: false, extra: Some(0xd11b55), protocol_display_name: None }], "rv4_ok2");
    }

    #[test]
    fn rejects_what_the_c_rejects() {
        assert!(run(RV4_BAD_XOR).is_empty(), "rv4_bad_xor: XOR of the bytes is not 0xFF");
        assert!(run(RV4_FLIP).is_empty(), "rv4_flip: one flipped payload bit");
        assert!(run(RV4_BAD_BTN).is_empty(), "rv4_bad_btn: button nibble 0xC");
        assert!(run(RV4_BAD_PAD).is_empty(), "rv4_bad_pad: pad field 0b10");
        assert!(run(RV4_SHORT).is_empty(), "rv4_short: 87 data bits");
        assert!(run(RV4_NOPRE).is_empty(), "rv4_nopre: 9 preamble bits");
        assert!(run(RV4_NOSYNC).is_empty(), "rv4_nosync: sync HIGH of 700 us");
    }

    #[test]
    fn repeated_frame_is_reported_once_until_reset() {
        // The C remembers the last frame (`last_key`) until reset().
        let mut dec = RenaultV4Decoder::new();
        let mut got = 0;
        for _ in 0..2 {
            let mut level = RV4_OK.0;
            for &d in RV4_OK.1 {
                got += dec.feed(level, d).is_some() as u32;
                level = !level;
            }
        }
        assert_eq!(got, 1);
        dec.reset();
        let mut level = RV4_OK.0;
        for &d in RV4_OK.1 {
            got += dec.feed(level, d).is_some() as u32;
            level = !level;
        }
        assert_eq!(got, 2);
    }

    #[test]
    fn decode_only() {
        let d = RenaultV4Decoder::new();
        assert!(!d.supports_encoding());
        assert!(d.encode(&DecodedSignal::new(0, 0), 1).is_none());
    }

    #[test]
    fn key_file_round_trip() {
        use crate::protocols::flipper_key::testing::assert_round_trip;
        let d = run(RV4_OK).remove(0);
        let text = assert_round_trip(&RenaultV4Decoder::new(), &d);
        assert!(
            text.contains("Bit: 88\nKey: 12 AB 34 CD E5 5C B2 79\nSerial: 313210061\nBtn: 14\nCnt: 87\nData2: 3633547\n"),
            "{text}"
        );
    }
}
