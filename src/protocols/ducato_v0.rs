//! Fiat Ducato V0 protocol decoder (decode-only)
//!
//! Ported from FoxFW: `REFERENCES/FoxFW/lib/subghz/protocols/ducato_v0.c`
//! (`DUCATO_V0_PROTOCOL_NAME = "Fiat Ducato V0"`, `315 | 433 | AM`, Dynamic, no encoder). The Magneti
//! Marelli NBC250C body computer's NXP PCF7946A (HITAG2) fob, 433.92 MHz ASK (FoxFW also flags 315 MHz,
//! so KAT feeds it both bands). Not KAT's
//! "Fiat V0" (64 bits, 100/200 µs) or the Hitag2-based "Fiat V1": its air format is 200/400 µs
//! Manchester behind a long preamble and an 800 µs sync.
//!
//! # Wire format (AM/OOK, Manchester 200/400 µs, tolerance 99 µs)
//! 1. at least 120 SHORT pulses (a HIGH one starts; every pulse must be short),
//! 2. one sync pulse of 800 µs (any level),
//! 3. 71 Manchester bits through the PCF7946A table (`ducato_manchester_decode`; the canonical
//!    Flipper table, but "same state" just gives no bit), `level ? ShortLow : ShortHigh` mapping.
//!    Durations that are neither short nor long are ignored inside the data (as in the firmware).
//!
//! # Frame (71 bits, MSB first)
//! `[hop : 32][fix : 32][crc : 7]`, `fix` = `serial(24) | button(8)`. The hop is the encrypted
//! rolling code (no counter can be read).
//!
//! # CRC
//! `acc = b0&0xB3 ^ b1&0x80 ^ b2&0x83 ^ b3&0x22 ^ b5&0x73` over the big-endian key bytes (byte 5,
//! not byte 4), valid when `crc & 1` equals the parity of the nibble-parity of `acc`. Only that one
//! bit of the 7-bit field is checked. **As in the C, a frame whose CRC fails is still reported**
//! (the C sets `crc_ok` and calls back): `crc_valid` is false then. Serials `0x000000`, `0xFFFFFF`,
//! `0x555555` and `0xAAAAAA` are dropped.
//!
//! # DecodedSignal packing
//! `data` = the 64-bit key (hop high, fix low), `data_count_bit` = 71, `extra` = the 7-bit CRC
//! field, `serial` (24 bit), `button` (low fix byte), no counter.
//!
//! # Key files
//! `serialize` writes `Protocol: Fiat Ducato V0`, `Bit: 71`, `Key` = the 64-bit key, then `Serial`,
//! `Btn`, `Hop` and `CRC` (decimal). `deserialize` needs exactly 71 bits, takes serial and button from
//! the key and the 7-bit `CRC` line (0 when absent), and re-runs the CRC check.
//!
//! # Encoder
//! None (the C's `.encoder` is NULL).

use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Fiat Ducato V0",
    decoder: "Fiat Ducato V0",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "HITAG2",
    make: "Fiat",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&AM650],
};

const TE_SHORT: u32 = 200;
const TE_LONG: u32 = 400;
const TE_DELTA: u32 = 99;
const TE_SYNC: u32 = 800;
const PREAMBLE_MIN: u32 = 120;
const DATA_BITS: usize = 71;

/// `ducato_manchester_table`.
const MANCHESTER_TABLE: [u8; 4] = [0x01, 0x91, 0x9B, 0xFB];
const PARITY_TABLE: [u8; 16] = [0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0];
const CRC_MASKS: [u8; 5] = [0xB3, 0x80, 0x83, 0x22, 0x73];

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    Preamble,
    Data,
}

/// `ducato_v0_check_crc`.
fn check_crc(key: u64, crc7: u8) -> bool {
    let b = key.to_be_bytes();
    let acc = (b[0] & CRC_MASKS[0])
        ^ (b[1] & CRC_MASKS[1])
        ^ (b[2] & CRC_MASKS[2])
        ^ (b[3] & CRC_MASKS[3])
        ^ (b[5] & CRC_MASKS[4]);
    let parity = PARITY_TABLE[(acc >> 4) as usize] ^ PARITY_TABLE[(acc & 0x0F) as usize];
    (crc7 & 1) == parity
}

/// The decode of a frame: the 64-bit `key` and the 7-bit `crc` field.
fn signal(key: u64, crc: u8) -> DecodedSignal {
    let fix = key as u32;
    DecodedSignal {
        serial: Some((fix >> 8) & 0x00FF_FFFF),
        button: Some(fix as u8),
        counter: None,
        crc_valid: check_crc(key, crc),
        data: key,
        data_count_bit: DATA_BITS,
        encoder_capable: false,
        extra: Some(crc as u64),
        protocol_display_name: None,
    }
}

/// Fiat Ducato V0 decoder (matches `SubGhzProtocolDecoderDucatoV0`).
pub struct DucatoV0Decoder {
    step: Step,
    te_last: u32,
    preamble_count: u32,
    man_state: u8,
    data_lo: u64,
    data_hi: u32,
    bit_count: usize,
}

impl DucatoV0Decoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            preamble_count: 0,
            man_state: 1,
            data_lo: 0,
            data_hi: 0,
            bit_count: 0,
        }
    }

    /// `ducato_man_decode`: returns the bit when one was produced.
    fn man_decode(&mut self, timing_type: u8) -> Option<u8> {
        let s = self.man_state;
        if s >= 4 {
            self.man_state = 1;
            return None;
        }
        let ns = (MANCHESTER_TABLE[s as usize] >> timing_type) & 0x3;
        if s == ns {
            return None;
        }
        self.man_state = ns;
        match ns {
            2 => Some(0),
            1 => Some(1),
            _ => None,
        }
    }

    fn clear_frame(&mut self) {
        self.man_state = 1;
        self.data_lo = 0;
        self.data_hi = 0;
        self.bit_count = 0;
    }

    /// `ducato_v0_finish`.
    fn finish(&mut self) -> Option<DecodedSignal> {
        let key = ((self.data_hi as u64) << 57) | (self.data_lo >> 7);
        let crc = (self.data_lo & 0x7F) as u8;
        let serial = ((key as u32) >> 8) & 0x00FF_FFFF;

        self.step = Step::Reset;
        self.preamble_count = 0;
        self.clear_frame();

        if matches!(serial, 0x000000 | 0xFFFFFF | 0x555555 | 0xAAAAAA) {
            return None;
        }
        Some(signal(key, crc))
    }
}

impl Default for DucatoV0Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for DucatoV0Decoder {
    fn name(&self) -> &'static str {
        "Fiat Ducato V0"
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
        // FoxFW's flags: SubGhzProtocolFlag_315 | SubGhzProtocolFlag_433 | AM.
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.step = Step::Reset;
        self.te_last = 0;
        self.preamble_count = 0;
        self.clear_frame();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let short = duration_diff!(duration, TE_SHORT) <= TE_DELTA;
        match self.step {
            Step::Reset => {
                if level && short {
                    self.step = Step::Preamble;
                    self.te_last = duration;
                    self.preamble_count = 1;
                    self.clear_frame();
                }
            }
            Step::Preamble => {
                if short {
                    if duration_diff!(self.te_last, TE_SHORT) <= TE_DELTA {
                        self.preamble_count += 1;
                    } else {
                        self.step = Step::Reset;
                        self.preamble_count = 0;
                    }
                    self.te_last = duration;
                } else if duration_diff!(duration, TE_SYNC) <= TE_DELTA {
                    if self.preamble_count >= PREAMBLE_MIN {
                        self.step = Step::Data;
                        self.preamble_count = 0;
                        self.clear_frame();
                    } else {
                        self.step = Step::Reset;
                        self.preamble_count = 0;
                    }
                } else {
                    self.step = Step::Reset;
                    self.preamble_count = 0;
                }
            }
            Step::Data => {
                // An invalid duration is ignored, not a reset (firmware behaviour).
                let timing_type = if short {
                    Some(if level { 0 } else { 2 })
                } else if duration_diff!(duration, TE_LONG) <= TE_DELTA {
                    Some(if level { 4 } else { 6 })
                } else {
                    None
                };
                if let Some(t) = timing_type {
                    if let Some(bit) = self.man_decode(t) {
                        let carry = (self.data_lo >> 63) as u32;
                        self.data_lo = (self.data_lo << 1) | bit as u64;
                        self.data_hi = (self.data_hi << 1) | carry;
                        self.bit_count += 1;
                        if self.bit_count == DATA_BITS {
                            return self.finish();
                        }
                    }
                }
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }

    /// `subghz_protocol_decoder_ducato_v0_deserialize` (see "Key files").
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != DATA_BITS {
            return None;
        }
        let crc = (key.u32_field("CRC").unwrap_or(0) & 0x7F) as u8;
        Some(signal(key.key_u64()?, crc))
    }

    /// `subghz_protocol_decoder_ducato_v0_serialize`.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != DATA_BITS {
            return None;
        }
        let mut key = FlipperKey::for_protocol("Fiat Ducato V0", DATA_BITS, decoded.data);
        key.push_u32("Serial", decoded.serial?);
        key.push_u32("Btn", decoded.button?.into());
        key.push_u32("Hop", (decoded.data >> 32) as u32);
        key.push_u32("CRC", decoded.extra? as u32);
        Some(key)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Test vectors: made-up field values, built as pulse sequences the way the C expects them, written
    // as Flipper RAW files and decoded by the FoxFW C decoder (host oracle, `--registry all
    // --protocol "Fiat Ducato V0"`); the expectations below are the fields the oracle printed. Levels
    // alternate from `.0`.

    /// `duc_ok`: first pulse HIGH, 239 pulses.
    const DUC_OK: (bool, &[u32]) = (
        true,
        &[
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 800, 400, 400, 400, 400, 200,
        200, 400, 400, 400, 200, 200, 400, 200, 200, 400, 400, 400, 400, 200, 200, 400,
        400, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 200, 400, 400, 200, 200,
        200, 200, 400, 200, 200, 400, 400, 400, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 400, 200, 200, 400, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 400, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 200, 200, 200, 200, 200,
        400, 400, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 3000,
        ],
    );

    /// `duc_goodcrc2`: first pulse HIGH, 236 pulses.
    const DUC_GOODCRC2: (bool, &[u32]) = (
        true,
        &[
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 800, 400, 200, 200, 200, 200,
        400, 400, 200, 200, 400, 200, 200, 400, 400, 400, 400, 400, 400, 200, 200, 200,
        200, 200, 200, 400, 200, 200, 400, 200, 200, 400, 400, 200, 200, 200, 200, 200,
        200, 400, 400, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 200, 200, 200,
        200, 200, 400, 400, 400, 400, 200, 200, 200, 200, 200, 200, 400, 200, 200, 400,
        400, 200, 200, 400, 200, 200, 400, 400, 400, 200, 200, 400, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 200, 400, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 3000,
        ],
    );

    /// `duc_badcrc`: first pulse HIGH, 235 pulses.
    const DUC_BADCRC: (bool, &[u32]) = (
        true,
        &[
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 800, 400, 200, 200, 200, 200,
        400, 400, 200, 200, 400, 200, 200, 400, 400, 400, 400, 400, 400, 200, 200, 200,
        200, 200, 200, 400, 200, 200, 400, 200, 200, 400, 400, 200, 200, 200, 200, 200,
        200, 400, 400, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 200, 200, 200,
        200, 200, 400, 400, 400, 400, 200, 200, 200, 200, 200, 200, 400, 200, 200, 400,
        400, 200, 200, 400, 200, 200, 400, 400, 400, 200, 200, 400, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 200, 400, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 3000,
        ],
    );

    /// `duc_nosync`: first pulse HIGH, 239 pulses.
    const DUC_NOSYNC: (bool, &[u32]) = (
        true,
        &[
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 1200, 400, 400, 400, 400, 200,
        200, 400, 400, 400, 200, 200, 400, 200, 200, 400, 400, 400, 400, 200, 200, 400,
        400, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 200, 400, 400, 200, 200,
        200, 200, 400, 200, 200, 400, 400, 400, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 400, 200, 200, 400, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 400, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 200, 200, 200, 200, 200,
        400, 400, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 3000,
        ],
    );

    /// `duc_shortpre`: first pulse HIGH, 217 pulses.
    const DUC_SHORTPRE: (bool, &[u32]) = (
        true,
        &[
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 800, 400, 400, 400, 400, 200, 200, 400, 400, 400, 200, 200,
        400, 200, 200, 400, 400, 400, 400, 200, 200, 400, 400, 200, 200, 200, 200, 200,
        200, 200, 200, 400, 200, 200, 400, 400, 200, 200, 200, 200, 400, 200, 200, 400,
        400, 400, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        400, 200, 200, 400, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 400, 200, 200, 200, 200, 200, 200, 400, 400, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 400, 200, 3000,
        ],
    );

    /// `duc_degenerate`: first pulse HIGH, 242 pulses.
    const DUC_DEGENERATE: (bool, &[u32]) = (
        true,
        &[
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 800, 400, 400, 400, 400, 200,
        200, 400, 400, 400, 200, 200, 400, 200, 200, 400, 400, 400, 400, 200, 200, 400,
        400, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 200, 400, 400, 200, 200,
        200, 200, 400, 200, 200, 400, 400, 400, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 400, 400, 200, 200,
        200, 200, 400, 400, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 3000,
        ],
    );

    /// `duc_short`: first pulse HIGH, 237 pulses.
    const DUC_SHORT: (bool, &[u32]) = (
        true,
        &[
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 800, 400, 400, 400, 400, 200,
        200, 400, 400, 400, 200, 200, 400, 200, 200, 400, 400, 400, 400, 200, 200, 400,
        400, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 200, 400, 400, 200, 200,
        200, 200, 400, 200, 200, 400, 400, 400, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 400, 200, 200, 400, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 400, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
        200, 200, 200, 200, 200, 200, 200, 200, 200, 400, 200, 200, 200, 200, 200, 200,
        400, 400, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 3000,
        ],
    );

    fn run(v: (bool, &[u32])) -> Vec<DecodedSignal> {
        let mut dec = DucatoV0Decoder::new();
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
        assert_eq!(run(DUC_OK), vec![DecodedSignal { serial: Some(0xc0ff), button: Some(0xe1), counter: None, crc_valid: true, data: 0x5a6b7c8d00c0ffe1, data_count_bit: 71, encoder_capable: false, extra: Some(0x1), protocol_display_name: None }], "duc_ok");
        assert_eq!(run(DUC_GOODCRC2), vec![DecodedSignal { serial: Some(0xa1b2c), button: Some(0x3), counter: None, crc_valid: true, data: 0x13579bdf0a1b2c03, data_count_bit: 71, encoder_capable: false, extra: Some(0x0), protocol_display_name: None }], "duc_goodcrc2");
        assert_eq!(run(DUC_BADCRC), vec![DecodedSignal { serial: Some(0xa1b2c), button: Some(0x3), counter: None, crc_valid: false, data: 0x13579bdf0a1b2c03, data_count_bit: 71, encoder_capable: false, extra: Some(0x1), protocol_display_name: None }], "duc_badcrc");
    }

    #[test]
    fn rejects_what_the_c_rejects() {
        assert!(run(DUC_NOSYNC).is_empty(), "duc_nosync: sync pulse 1200 us");
        assert!(run(DUC_SHORTPRE).is_empty(), "duc_shortpre: under 120 preamble pulses");
        assert!(run(DUC_DEGENERATE).is_empty(), "duc_degenerate: serial 0x000000");
        assert!(run(DUC_SHORT).is_empty(), "duc_short: 70 bits");
    }

    #[test]
    fn decode_only() {
        let d = DucatoV0Decoder::new();
        assert!(!d.supports_encoding());
        assert!(d.encode(&DecodedSignal::new(0, 0), 1).is_none());
    }

    #[test]
    fn key_file_round_trip() {
        use crate::protocols::flipper_key::testing::assert_round_trip;
        for v in [DUC_OK, DUC_BADCRC] {
            let d = run(v).remove(0);
            assert_round_trip(&DucatoV0Decoder::new(), &d);
        }
        let d = run(DUC_OK).remove(0);
        let text = assert_round_trip(&DucatoV0Decoder::new(), &d);
        assert!(
            text.contains("Bit: 71\nKey: 5A 6B 7C 8D 00 C0 FF E1\nSerial: 49407\nBtn: 225\nHop: 1516993677\nCRC: 1\n"),
            "{text}"
        );
    }

    #[test]
    fn crc_field_is_one_checked_bit() {
        // The C only compares crc & 1 with the parity: both values of the other six bits agree.
        let d = run(DUC_OK).remove(0);
        assert!(d.crc_valid && d.extra == Some(1));
        assert!(check_crc(d.data, 0x01) && check_crc(d.data, 0x7F) && !check_crc(d.data, 0x00));
    }
}
