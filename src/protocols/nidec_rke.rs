//! Nidec RKE: Honda OEM car remote, FSK pulse-width frame (receive only, as in FoxFW).
//!
//! Ported from FoxFW `lib/subghz/protocols/nidec_rke.c` (`NIDEC_RKE_PROTOCOL_NAME = "Nidec RKE"`;
//! `SubGhzProtocolTypeDynamic`, flags `315 | FM`), itself a pulse-by-pulse rewrite of rtl_433
//! `nidec_car_remote.c` (OUCG8D-344H-A, 313.8 MHz FSK PWM). It is a different remote from KAT's
//! rtl_433 "Honda-KR5V2X1X" port (FSK 60 µs Manchester).
//!
//! **Transmit: none, as in FoxFW.** FoxFW's SD-card encoder plugin
//! `applications/main/subghz/protocols_ext/enc_nidec_rke.c` is a stub ("no TX — security field is a
//! keyed authenticator, not emittable"): its `deserialize` returns
//! `SubGhzProtocolStatusErrorEncoderGetUpload` for every key file and its `yield` only ends the
//! transmission, so a Flipper sends nothing for any Nidec RKE file. KAT does the same:
//! `supports_encoding()` is false, `encode` returns None and no decode or loaded key is
//! encoder-capable.
//!
//! **Air format**: only the HIGH pulses carry data. A HIGH of 130..374 µs is a raw 1, 375..700 µs a
//! raw 0; any other HIGH abandons the frame. A LOW of 700 µs or more ends the packet (shorter LOWs
//! are ignored). While hunting, every raw bit is shifted into a 32-bit register; when it equals
//! `0xFFFFFFF0` (28 short pulses then four long ones) the payload starts. Payload bits are the
//! raw bits **inverted**. The frame is complete at 64 bits, or at the packet-ending LOW with at
//! least 56 bits (the demodulator often cuts the last bits off; the missing tail is zero-padded).
//!
//! **Frame** (8 bytes after inversion, MSB first): bytes 0..1 `sequence`, 2..4 `id`, byte 5 = `5` in
//! the high nibble and the 4-bit `button` in the low one, bytes 6..7 `security`, a keyed
//! authenticator no CRC reproduces (rtl_433's author tried them all), so there is no check to gate on.
//! The C instead drops frames whose byte 5 high nibble is not `5`, whose `id` is 0 or 0xFFFFFF, whose
//! `sequence` or `security` is 0 or 0xFFFF, or whose button is not one of 3, 4, 5, 6, 0xF.
//!
//! **DecodedSignal**: `data` = the 8 bytes left-aligned (zero-padded when truncated),
//! `data_count_bit` = the bits actually captured (56..64), `serial` = id, `button`, `counter` =
//! sequence, `extra` = security. `crc_valid` is false: nothing was verified, only the sanity gate.
//!
//! **Key file**: `Protocol: Nidec RKE`, `Bit` = the bits captured, `Key` = `data`, no other line.
//! The C loads only a 56-bit file (`min_count_bit_for_found` is its exact count), so it cannot read
//! back its own 60- or 64-bit files; KAT loads 56..=64 bits through the same sanity gate.

use super::g2_common::decoded;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`): `SubGhzProtocolFlag_315 | FM`, dynamic (rolling) code.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::FM,
    encryption: "Rolling Code",
    make: "Honda",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&FM476],
};

const NAME: &str = "Nidec RKE";
const TE_SHORT: u32 = 250;
const TE_LONG: u32 = 500;
const TE_DELTA: u32 = 120;
/// HIGH classification bounds.
const MARK_MIN: u32 = 130;
const MARK_MID: u32 = 375;
const MARK_MAX: u32 = 700;
/// A LOW at least this long ends the packet.
const GAP_RESET: u32 = 700;
const SYNC_WORD: u32 = 0xFFFF_FFF0;
const PAYLOAD_BITS: u16 = 64;
const MIN_BITS: u16 = 56;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    Data,
}

/// Nidec RKE decoder (`SubGhzProtocolDecoderNidecRke`).
pub struct NidecRkeDecoder {
    step: Step,
    sync_reg: u32,
    payload: u64,
    payload_bits: u16,
}

/// `nidec_button_is_valid`: the five codes ever seen on a real remote.
fn button_is_valid(button: u8) -> bool {
    matches!(button, 0x3 | 0x4 | 0x5 | 0x6 | 0xF)
}

impl NidecRkeDecoder {
    pub fn new() -> Self {
        Self { step: Step::Reset, sync_reg: 0, payload: 0, payload_bits: 0 }
    }

    fn clear(&mut self) {
        self.step = Step::Reset;
        self.sync_reg = 0;
        self.payload = 0;
        self.payload_bits = 0;
    }

    /// `subghz_protocol_nidec_rke_process`.
    fn process(&self) -> Option<DecodedSignal> {
        let bits = self.payload_bits.min(PAYLOAD_BITS);
        if bits < MIN_BITS {
            return None;
        }
        let mut data = self.payload;
        if bits < PAYLOAD_BITS {
            data <<= PAYLOAD_BITS - bits;
        }
        frame(data, bits)
    }
}

/// The sanity gate and the fields of an already left-aligned frame of `bits` bits.
fn frame(data: u64, bits: u16) -> Option<DecodedSignal> {
    let byte = |i: u32| (data >> (56 - 8 * i)) as u8;
    let byte5 = byte(5);
    if byte5 & 0xF0 != 0x50 {
        return None;
    }
    let sequence = u16::from(byte(0)) << 8 | u16::from(byte(1));
    let id = u32::from(byte(2)) << 16 | u32::from(byte(3)) << 8 | u32::from(byte(4));
    let button = byte5 & 0x0F;
    let security = u16::from(byte(6)) << 8 | u16::from(byte(7));
    if id == 0 || id == 0xFF_FFFF {
        return None;
    }
    if sequence == 0 || sequence == 0xFFFF {
        return None;
    }
    if security == 0 || security == 0xFFFF {
        return None;
    }
    if !button_is_valid(button) {
        return None;
    }
    let mut d = decoded(data, usize::from(bits));
    d.serial = Some(id);
    d.button = Some(button);
    d.counter = Some(sequence);
    d.extra = Some(u64::from(security));
    Some(d)
}

impl Default for NidecRkeDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for NidecRkeDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: TE_SHORT, te_long: TE_LONG, te_delta: TE_DELTA, min_count_bit: usize::from(MIN_BITS) }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.clear();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        if !level {
            // Only a long gap matters: it ends the packet.
            if duration >= GAP_RESET {
                let out = (self.step == Step::Data).then(|| self.process()).flatten();
                self.clear();
                return out;
            }
            return None;
        }
        let raw_bit = if (MARK_MIN..MARK_MID).contains(&duration) {
            1u8
        } else if (MARK_MID..=MARK_MAX).contains(&duration) {
            0u8
        } else {
            self.clear();
            return None;
        };
        match self.step {
            Step::Reset => {
                self.sync_reg = (self.sync_reg << 1) | u32::from(raw_bit);
                if self.sync_reg == SYNC_WORD {
                    self.step = Step::Data;
                    self.payload = 0;
                    self.payload_bits = 0;
                }
                None
            }
            Step::Data => {
                self.payload = (self.payload << 1) | u64::from(raw_bit ^ 1);
                self.payload_bits += 1;
                if self.payload_bits >= PAYLOAD_BITS {
                    let out = self.process();
                    self.clear();
                    return out;
                }
                None
            }
        }
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    /// FoxFW's encoder is a plugin outside this port.
    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        let bits = u16::try_from(key.bits).ok()?;
        (MIN_BITS..=PAYLOAD_BITS).contains(&bits).then(|| frame(key.key, bits)).flatten()
    }

    fn to_flipper_key(&self, d: &DecodedSignal) -> Option<FlipperKey> {
        Some(FlipperKey::for_protocol(NAME, d.data_count_bit, d.data))
    }
}

#[cfg(test)]
mod tests {
    use super::super::g2_common::test_util::*;
    use super::*;

    /// One frame the FoxFW C decoder (host oracle) reports for a synthetic burst: made-up values, the
    /// pulses are signed durations (HIGH positive), the fields are what the oracle printed.
    struct Case {
        name: &'static str,
        wire: &'static [i32],
        frames: &'static [Expect],
    }
    struct Expect {
        data: u64,
        bits: usize,
        serial: Option<u32>,
        button: Option<u8>,
        counter: Option<u16>,
        extra: Option<u64>,
    }

    const ACCEPT: &[Case] = &[
        Case {
            name: "ok_64",
            wire: &[
            500, -300, 500, -300, 500, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 500, -300,
            500, -300, 500, -300, 500, -300, 250, -300, 250, -300, 250, -300, 500, -300, 250, -300,
            250, -300, 500, -300, 250, -300, 250, -300, 250, -300, 500, -300, 500, -300, 250, -300,
            500, -300, 250, -300, 250, -300, 500, -300, 250, -300, 500, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 500, -300, 500, -300, 250, -300, 500, -300, 500, -300, 250, -300,
            250, -300, 500, -300, 250, -300, 500, -300, 500, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 500, -300, 500, -300, 250, -300, 500, -300, 250, -300, 500, -300, 250, -300,
            250, -300, 500, -300, 500, -300, 500, -300, 250, -300, 500, -300, 500, -300, 500, -300,
            500, -300, 500, -300, 250, -300, 500, -300, 500, -300, 500, -300, 250, -300, 500, -300,
            500, -300, 500, -300, 500, -1500,
            ],
            frames: &[
                Expect { data: 0x1234A1B2C353BEEF, bits: 64, serial: Some(10597059), button: Some(3), counter: Some(4660), extra: Some(48879) },
            ],
        },
        Case {
            name: "ok_btn_f",
            wire: &[
            500, -300, 500, -300, 500, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 500, -300,
            500, -300, 500, -300, 500, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 500, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 500, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 500, -300,
            250, -300, 500, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 500, -300,
            250, -300, 500, -300, 500, -300, 250, -300, 250, -300, 250, -300, 250, -300, 500, -300,
            500, -300, 250, -300, 250, -300, 250, -300, 500, -300, 250, -300, 500, -300, 500, -300,
            500, -300, 500, -300, 500, -300, 250, -300, 250, -300, 250, -300, 500, -300, 250, -300,
            250, -300, 500, -300, 500, -300, 250, -300, 500, -300, 250, -300, 500, -300, 250, -300,
            500, -300, 500, -300, 500, -1500,
            ],
            frames: &[
                Expect { data: 0x1020A0B0C5F1357, bits: 64, serial: Some(658188), button: Some(15), counter: Some(258), extra: Some(4951) },
            ],
        },
        Case {
            name: "ok_trunc60",
            wire: &[
            500, -300, 500, -300, 500, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 500, -300,
            500, -300, 500, -300, 500, -300, 250, -300, 250, -300, 250, -300, 500, -300, 250, -300,
            250, -300, 500, -300, 250, -300, 250, -300, 250, -300, 500, -300, 500, -300, 250, -300,
            500, -300, 250, -300, 250, -300, 500, -300, 250, -300, 500, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 500, -300, 500, -300, 250, -300, 500, -300, 500, -300, 250, -300,
            250, -300, 500, -300, 250, -300, 500, -300, 500, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 500, -300, 500, -300, 250, -300, 500, -300, 250, -300, 500, -300, 250, -300,
            500, -300, 250, -300, 500, -300, 500, -300, 250, -300, 500, -300, 500, -300, 500, -300,
            500, -300, 500, -300, 250, -300, 500, -300, 500, -300, 500, -300, 250, -1500,
            ],
            frames: &[
                Expect { data: 0x1234A1B2C355BEE0, bits: 60, serial: Some(10597059), button: Some(5), counter: Some(4660), extra: Some(48864) },
            ],
        },
    ];

    /// Bursts the C decoder reports nothing for: a 55-bit payload, a high nibble other than 5, an unseen button code, a zero id, a zero sequence, a zero security word (the 56-bit truncation zeroes it) and a 27-ones sync.
    const REJECT: &[(&str, &[i32])] = &[
        (
            "ok_trunc56",
            &[
            500, -300, 500, -300, 500, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 500, -300,
            500, -300, 500, -300, 500, -300, 250, -300, 500, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 500, -300, 500, -300, 250, -300, 250, -300, 500, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 500, -300, 250, -300, 250, -300, 250, -300, 500, -300, 250, -300,
            250, -300, 250, -300, 500, -300, 250, -300, 250, -300, 500, -300, 250, -300, 250, -300,
            250, -300, 500, -300, 250, -300, 250, -300, 250, -300, 500, -300, 500, -300, 250, -300,
            250, -300, 500, -300, 500, -300, 250, -300, 500, -300, 250, -300, 500, -300, 250, -300,
            500, -300, 500, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -1500,
            ],
        ),
        (
            "short55",
            &[
            500, -300, 500, -300, 500, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 500, -300,
            500, -300, 500, -300, 500, -300, 250, -300, 250, -300, 250, -300, 500, -300, 250, -300,
            250, -300, 500, -300, 250, -300, 250, -300, 250, -300, 500, -300, 500, -300, 250, -300,
            500, -300, 250, -300, 250, -300, 500, -300, 250, -300, 500, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 500, -300, 500, -300, 250, -300, 500, -300, 500, -300, 250, -300,
            250, -300, 500, -300, 250, -300, 500, -300, 500, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 500, -300, 500, -300, 250, -300, 500, -300, 250, -300, 500, -300, 250, -300,
            250, -300, 500, -300, 500, -300, 500, -300, 250, -300, 500, -300, 500, -300, 500, -300,
            500, -300, 500, -1500,
            ],
        ),
        (
            "bad_nibble",
            &[
            500, -300, 500, -300, 500, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 500, -300,
            500, -300, 500, -300, 500, -300, 250, -300, 250, -300, 250, -300, 500, -300, 250, -300,
            250, -300, 500, -300, 250, -300, 250, -300, 250, -300, 500, -300, 500, -300, 250, -300,
            500, -300, 250, -300, 250, -300, 500, -300, 250, -300, 500, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 500, -300, 500, -300, 250, -300, 500, -300, 500, -300, 250, -300,
            250, -300, 500, -300, 250, -300, 500, -300, 500, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 500, -300, 500, -300, 250, -300, 500, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 500, -300, 500, -300, 500, -300, 250, -300, 500, -300, 500, -300, 500, -300,
            500, -300, 500, -300, 250, -300, 500, -300, 500, -300, 500, -300, 250, -300, 500, -300,
            500, -300, 500, -300, 500, -1500,
            ],
        ),
        (
            "bad_button",
            &[
            500, -300, 500, -300, 500, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 500, -300,
            500, -300, 500, -300, 500, -300, 250, -300, 250, -300, 250, -300, 500, -300, 250, -300,
            250, -300, 500, -300, 250, -300, 250, -300, 250, -300, 500, -300, 500, -300, 250, -300,
            500, -300, 250, -300, 250, -300, 500, -300, 250, -300, 500, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 500, -300, 500, -300, 250, -300, 500, -300, 500, -300, 250, -300,
            250, -300, 500, -300, 250, -300, 500, -300, 500, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 500, -300, 500, -300, 250, -300, 500, -300, 250, -300, 500, -300, 250, -300,
            500, -300, 500, -300, 500, -300, 500, -300, 250, -300, 500, -300, 500, -300, 500, -300,
            500, -300, 500, -300, 250, -300, 500, -300, 500, -300, 500, -300, 250, -300, 500, -300,
            500, -300, 500, -300, 500, -1500,
            ],
        ),
        (
            "zero_id",
            &[
            500, -300, 500, -300, 500, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 500, -300,
            500, -300, 500, -300, 500, -300, 250, -300, 250, -300, 250, -300, 500, -300, 250, -300,
            250, -300, 500, -300, 250, -300, 250, -300, 250, -300, 500, -300, 500, -300, 250, -300,
            500, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 500, -300, 250, -300, 500, -300, 250, -300,
            250, -300, 500, -300, 500, -300, 500, -300, 250, -300, 500, -300, 500, -300, 500, -300,
            500, -300, 500, -300, 250, -300, 500, -300, 500, -300, 500, -300, 250, -300, 500, -300,
            500, -300, 500, -300, 500, -1500,
            ],
        ),
        (
            "zero_seq",
            &[
            500, -300, 500, -300, 500, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 500, -300,
            500, -300, 500, -300, 500, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 500, -300, 250, -300, 500, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 500, -300, 500, -300, 250, -300, 500, -300, 500, -300, 250, -300,
            250, -300, 500, -300, 250, -300, 500, -300, 500, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 500, -300, 500, -300, 250, -300, 500, -300, 250, -300, 500, -300, 250, -300,
            250, -300, 500, -300, 500, -300, 500, -300, 250, -300, 500, -300, 500, -300, 500, -300,
            500, -300, 500, -300, 250, -300, 500, -300, 500, -300, 500, -300, 250, -300, 500, -300,
            500, -300, 500, -300, 500, -1500,
            ],
        ),
        (
            "bad_sync",
            &[
            500, -300, 500, -300, 500, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 250, -300, 500, -300, 500, -300,
            500, -300, 500, -300, 250, -300, 250, -300, 250, -300, 500, -300, 250, -300, 250, -300,
            500, -300, 250, -300, 250, -300, 250, -300, 500, -300, 500, -300, 250, -300, 500, -300,
            250, -300, 250, -300, 500, -300, 250, -300, 500, -300, 250, -300, 250, -300, 250, -300,
            250, -300, 500, -300, 500, -300, 250, -300, 500, -300, 500, -300, 250, -300, 250, -300,
            500, -300, 250, -300, 500, -300, 500, -300, 250, -300, 250, -300, 250, -300, 250, -300,
            500, -300, 500, -300, 250, -300, 500, -300, 250, -300, 500, -300, 250, -300, 250, -300,
            500, -300, 500, -300, 500, -300, 250, -300, 500, -300, 500, -300, 500, -300, 500, -300,
            500, -300, 250, -300, 500, -300, 500, -300, 500, -300, 250, -300, 500, -300, 500, -300,
            500, -300, 500, -1500,
            ],
        ),
    ];

    fn check(frames: &[DecodedSignal], want: &[Expect], name: &str) {
        assert_eq!(frames.len(), want.len(), "{name}: frame count");
        for (d, w) in frames.iter().zip(want) {
            assert_eq!((d.data, d.data_count_bit), (w.data, w.bits), "{name}");
            assert_eq!((d.serial, d.button, d.counter, d.extra), (w.serial, w.button, w.counter, w.extra), "{name}");
            assert!(!d.encoder_capable, "{name}");
        }
    }

    #[test]
    fn decodes_what_the_c_decoder_decodes() {
        for c in ACCEPT {
            let got = feed_with_resets(&mut NidecRkeDecoder::new(), &wire(c.wire));
            check(&got, c.frames, c.name);
        }
    }

    #[test]
    fn rejects_what_the_c_decoder_rejects() {
        for (name, w) in REJECT {
            let got = feed_with_resets(&mut NidecRkeDecoder::new(), &wire(w));
            assert!(got.is_empty(), "{name}: {got:?}");
        }
    }

    /// The whole registry reports the frame under this name (no earlier decoder takes it).
    #[test]
    fn registry_reports_it() {
        use super::super::ProtocolRegistry;
        for c in ACCEPT {
            let pairs = wire(c.wire);
            let mut reg = ProtocolRegistry::new();
            let got = reg.process_signal_stream(&pairs, 315000000);
            let names: Vec<_> = got.iter().map(|(n, _, _)| n.as_str()).collect();
            assert_eq!(names, vec![NAME; c.frames.len()], "{}", c.name);
        }
    }

    /// Key files: every accepted frame survives write, parse and load unchanged.
    #[test]
    fn key_files_round_trip() {
        let dec = NidecRkeDecoder::new();
        for c in ACCEPT {
            for d in feed_with_resets(&mut NidecRkeDecoder::new(), &wire(c.wire)) {
                assert_key_roundtrip(&dec, &d);
                let text = dec.to_flipper_key(&d).unwrap().to_text();
                assert!(text.contains("Protocol: Nidec RKE\nBit: "), "{text}");
            }
        }
    }

    /// FoxFW's encoder refuses every key file (`subghz_oracle_foxfwenc encode`: "deserialize failed
    /// (status -12)", `SubGhzProtocolStatusErrorEncoderGetUpload`, for 56..64-bit files alike), so
    /// nothing transmits: no decode or loaded key is encoder-capable and `encode` is None for each.
    #[test]
    fn the_c_encoder_refuses_every_key() {
        let dec = NidecRkeDecoder::new();
        assert!(!dec.supports_encoding());
        for c in ACCEPT {
            for d in feed_with_resets(&mut NidecRkeDecoder::new(), &wire(c.wire)) {
                let loaded = dec.from_flipper_key(&dec.to_flipper_key(&d).unwrap()).unwrap();
                for x in [&d, &loaded] {
                    assert!(!x.encoder_capable, "{}", c.name);
                    for button in [0x01, 0x02, 0x04, 0x08, x.button.unwrap()] {
                        assert!(dec.encode(x, button).is_none(), "{}", c.name);
                    }
                }
            }
        }
        assert!(dec.encode(&decoded(0, 0), 1).is_none());
        assert_eq!(dec.name(), "Nidec RKE");
    }

    /// Generator for the oracle comparison: `KAT_NIDEC_RKE_ENCODE=<file listing key files>` prints
    /// KAT's transmission of each (`KATENC <file> | (none)`: it never encodes), to compare with
    /// `subghz_oracle_foxfwenc encode <file>`, which refuses each.
    #[test]
    #[ignore = "oracle harness"]
    fn encode_key_files() {
        let Ok(list) = std::env::var("KAT_NIDEC_RKE_ENCODE") else { return };
        let dec = NidecRkeDecoder::new();
        for file in std::fs::read_to_string(list).unwrap().lines().filter(|l| !l.is_empty()) {
            let key = FlipperKey::parse(&std::fs::read_to_string(file).unwrap()).ok();
            let out = key.and_then(|k| dec.from_flipper_key(&k)).and_then(|d| dec.encode(&d, d.button.unwrap_or(0)));
            let text = out.map_or("(none)".to_string(), |tx| {
                tx.iter()
                    .map(|p| if p.level { p.duration_us.to_string() } else { format!("-{}", p.duration_us) })
                    .collect::<Vec<_>>()
                    .join(" ")
            });
            println!("KATENC {file} | {text}");
        }
    }
}
