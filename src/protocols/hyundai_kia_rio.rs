//! Hyundai/Kia RIO: 64-bit fixed-code frame with a 16-bit checksum.
//!
//! Ported from FoxFW `lib/subghz/protocols/hyundai_kia_rio.c` (`HYUNDAI_KIA_RIO_PROTOCOL_NAME =
//! "Hyundai/Kia RIO"`; `SubGhzProtocolTypeStatic`, flags `433 | AM`) and its SD-card encoder plugin
//! `applications/main/subghz/protocols_ext/enc_hyundai_kia_rio.c`, with the `pp_auto_common.c`
//! helpers the firmware's host bridge hands it (`ppa_emit`, `ppa_encoder_read_fields`,
//! `ppa_encoder_read_repeat`).
//!
//! **Provenance.** The format comes from Flipper-ARF `lib/subghz/protocols/auto_rke_protocols.c`
//! ("Additional automotive RKE protocols — ported from Pandora DXL 5000 firmware"; `hkr_decode`, "Hyundai/KiaRIO (ID 0x11)", early Hyundai/Kia fobs: Accent, Rio, Elantra ~2001-2008), a
//! reverse-engineering file ARF never registers, like its other Pandora-derived decoders (Audi,
//! Hyundai, Mazda Infinity, which KAT keeps unregistered). A KAT fork (limbo111111/KAT, written
//! by Google's Jules bot in May 2026) ported it to Rust "aligned with Flipper-ARF reference:
//! `auto_rke_protocols.c`"; FoxFW's C ("ported from KAT src/protocols/hyundai_kia_rio.rs") comes from that
//! fork, and KAT's module is ported from FoxFW's C. KAT registers it because FoxFW does (owner's
//! decision, 2026-11-07). No capture of the remote exists in any reference or in KAT's corpus.
//!
//! **Air format** (433.92 MHz, AM): te_short 312 µs, te_long 728 µs, te_delta 150 µs. A frame is a
//! HIGH 312 (±150) + LOW 10400 (±1560) sync, then 64 pulse pairs: HIGH 728 = 1, HIGH 312 = 0, each
//! followed by a LOW of 312 or 728 (±150, the gap carries nothing). The frame is reported on the
//! LOW of the 64th pair when its checksum holds; a HIGH or LOW longer than 3000 µs after at least
//! 64 bits reports it too (never reached with more than 64 bits, as the 64th pair resets).
//!
//! **Frame**: `serial` (32 bits) | `mask` (16) | `checksum` (16), MSB first.
//! `checksum = !((serial ^ (serial >> 16)) as u16 ^ mask)`; a frame with any other checksum is
//! dropped. `button` is the mask's key: 0x0100 = 1, 0x0200 = 2, 0x0400 = 4, 0x0800 = 8, any other
//! mask `mask >> 8` (as `hkr_mask_to_button`). No counter. `data` = the 64 bits.
//!
//! **Key file**: `Protocol: Hyundai/Kia RIO`, `Bit: 64`, `Key`, then `Serial` and `Btn` (decimal).
//! Both C deserializers load any 8-byte Key at exactly 64 bits. The decoder's derives serial and
//! button from the Key; the encoder's takes `Serial` and `Btn` over them, rebuilds the frame
//! (serial, mask `Btn << 8`, checksum) and writes that Key back into the file. KAT loads a key whose
//! checksum holds and whose `Serial` / `Btn` (when present) agree with it as it is (a frame the
//! decoder reports); any other key loads as the frame the encoder rebuilds from it.
//!
//! **Transmit** (`subghz_protocol_encoder_hyundai_kia_rio_deserialize` + `hkr_build_upload`): the
//! serial, the mask `hkr_button_to_mask(button)` (= `button << 8` for every code) and its checksum,
//! 3 copies (`ppa_encoder_read_repeat`'s default) of HIGH 312, LOW 10400, the 64 bits MSB first (1 =
//! HIGH 728 + LOW 312, 0 = HIGH 312 + LOW 728) and LOW 10000, which follows the last bit's LOW as a
//! pulse of its own, as the C yields it. A mask whose low byte is not 0 (it decodes, `mask >> 8`)
//! goes out as `button << 8`, as the C sends it. No counter: a transmission leaves the capture as it
//! is. KAT's TX Lock / Unlock / Trunk / Panic send buttons 0x01 / 0x02 / 0x04 / 0x08, the masks
//! 0x0100 / 0x0200 / 0x0400 / 0x0800 Flipper-ARF's `auto_rke_protocols.c` names (`HKR_BTN_LOCK` …
//! `HKR_BTN_PANIC`); "Send next code" and the key-file tests send the capture's own button. A
//! `Repeat` line is not applied (FoxFW deletes it before saving a file).

use super::g2_common::{decoded, near, BitReg};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`): `SubGhzProtocolFlag_433 | AM`, static (fixed) code.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Hyundai/Kia",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "Hyundai/Kia RIO";
const TE_SHORT: u32 = 312;
const TE_LONG: u32 = 728;
const TE_DELTA: u32 = 150;
const MIN_COUNT_BIT: usize = 64;
const SYNC_US: u32 = 10_400;
const SYNC_DELTA: u32 = 1_560;
/// A HIGH or LOW over this ends the data (`duration > 3000`).
const END_US: u32 = 3_000;
/// The encoder's closing LOW (`HKR_GAP_US`).
const GAP_US: u32 = 10_000;
/// `ppa_encoder_read_repeat(flipper_format, 3)` without a `Repeat` line.
const REPEAT: usize = 3;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    CheckSync,
    SaveDuration,
    CheckDuration,
}

/// Hyundai/Kia RIO decoder (`SubGhzProtocolDecoderHyundaiKiaRio`).
pub struct HyundaiKiaRioDecoder {
    step: Step,
    reg: BitReg,
}

/// `hkr_mask_to_button`.
fn mask_to_button(mask: u16) -> u8 {
    match mask {
        0x0100 => 1,
        0x0200 => 2,
        0x0400 => 4,
        0x0800 => 8,
        _ => (mask >> 8) as u8,
    }
}

/// `hkr_button_to_mask` (every arm is `button << 8`).
fn button_to_mask(button: u8) -> u16 {
    match button {
        1 => 0x0100,
        2 => 0x0200,
        4 => 0x0400,
        8 => 0x0800,
        _ => u16::from(button) << 8,
    }
}

/// `hkr_checksum`.
fn checksum(serial: u32, mask: u16) -> u16 {
    !((serial ^ (serial >> 16)) as u16 ^ mask)
}

/// The frame the encoder builds: `serial`, `hkr_button_to_mask(button)`, `hkr_checksum`.
fn frame(serial: u32, button: u8) -> u64 {
    let mask = button_to_mask(button);
    u64::from(serial) << 32 | u64::from(mask) << 16 | u64::from(checksum(serial, mask))
}

/// `hkr_build_upload`: one copy (131 pulses).
fn upload(word: u64) -> Vec<LevelDuration> {
    let mut up = Vec::with_capacity(2 + 2 * MIN_COUNT_BIT + 1);
    up.extend([LevelDuration::new(true, TE_SHORT), LevelDuration::new(false, SYNC_US)]);
    for b in (0..MIN_COUNT_BIT).rev() {
        let (high, low) = if (word >> b) & 1 == 1 { (TE_LONG, TE_SHORT) } else { (TE_SHORT, TE_LONG) };
        up.extend([LevelDuration::new(true, high), LevelDuration::new(false, low)]);
    }
    up.push(LevelDuration::new(false, GAP_US));
    up
}

/// `hkr_publish`: None when the checksum does not hold. Every frame transmits.
fn publish(data: u64) -> Option<DecodedSignal> {
    let serial = (data >> 32) as u32;
    let mask = (data >> 16) as u16;
    if data as u16 != checksum(serial, mask) {
        return None;
    }
    let mut d = decoded(data, MIN_COUNT_BIT);
    d.serial = Some(serial);
    d.button = Some(mask_to_button(mask));
    d.crc_valid = true;
    d.encoder_capable = true;
    Some(d)
}

impl HyundaiKiaRioDecoder {
    pub fn new() -> Self {
        Self { step: Step::Reset, reg: BitReg::default() }
    }
}

impl Default for HyundaiKiaRioDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for HyundaiKiaRioDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: TE_SHORT, te_long: TE_LONG, te_delta: TE_DELTA, min_count_bit: MIN_COUNT_BIT }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.step = Step::Reset;
        self.reg.clear();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let mut out = None;
        match self.step {
            Step::Reset => {
                if level && near(duration, TE_SHORT, TE_DELTA) {
                    self.step = Step::CheckSync;
                }
            }
            Step::CheckSync => {
                if !level && near(duration, SYNC_US, SYNC_DELTA) {
                    self.step = Step::SaveDuration;
                    self.reg.clear();
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::SaveDuration => {
                if !level {
                    self.step = Step::Reset;
                } else if near(duration, TE_LONG, TE_DELTA) {
                    self.reg.add(true);
                    self.step = Step::CheckDuration;
                } else if near(duration, TE_SHORT, TE_DELTA) {
                    self.reg.add(false);
                    self.step = Step::CheckDuration;
                } else {
                    if duration > END_US && self.reg.count as usize >= MIN_COUNT_BIT {
                        out = publish(self.reg.data);
                    }
                    self.step = Step::Reset;
                }
            }
            Step::CheckDuration => {
                if level {
                    self.step = Step::Reset;
                } else if near(duration, TE_SHORT, TE_DELTA) || near(duration, TE_LONG, TE_DELTA) {
                    if self.reg.count as usize >= MIN_COUNT_BIT {
                        out = publish(self.reg.data);
                        self.step = Step::Reset;
                    } else {
                        self.step = Step::SaveDuration;
                    }
                } else {
                    if duration > END_US && self.reg.count as usize >= MIN_COUNT_BIT {
                        out = publish(self.reg.data);
                    }
                    self.step = Step::Reset;
                }
            }
        }
        out
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    /// FoxFW's `enc_hyundai_kia_rio.c` on the key file KAT writes for `decoded` with `Btn` =
    /// `button`: the serial, the mask `button << 8` and its checksum, 3 times.
    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        let serial = decoded.serial.unwrap_or((decoded.data >> 32) as u32);
        Some(upload(frame(serial, button)).repeat(REPEAT))
    }

    /// Any 8-byte Key at 64 bits (both C deserializers): as it is when its checksum holds and the
    /// `Serial` / `Btn` lines agree with it, else the frame FoxFW's encoder rebuilds from them.
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != MIN_COUNT_BIT {
            return None;
        }
        let data = key.key_u64()?;
        let (key_serial, key_button) = ((data >> 32) as u32, mask_to_button((data >> 16) as u16));
        let serial = key.u32_field("Serial").unwrap_or(key_serial);
        let button = key.u32_field("Btn").map_or(key_button, |b| b as u8);
        match publish(data) {
            Some(d) if (serial, button) == (key_serial, key_button) => Some(d),
            _ => publish(frame(serial, button)),
        }
    }

    fn to_flipper_key(&self, d: &DecodedSignal) -> Option<FlipperKey> {
        let mut key = FlipperKey::for_protocol(NAME, d.data_count_bit, d.data);
        key.push_u32("Serial", d.serial?);
        key.push_u32("Btn", u32::from(d.button?));
        Some(key)
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
            name: "ok_b1",
            wire: &[
            312, -10400, 312, -728, 312, -312, 312, -312, 728, -728, 728, -312, 312, -312, 728, -728,
            312, -312, 312, -312, 312, -728, 728, -312, 312, -312, 728, -728, 312, -312, 728, -312,
            728, -728, 312, -312, 312, -312, 728, -728, 728, -312, 728, -312, 728, -728, 312, -312,
            312, -312, 312, -728, 728, -312, 312, -312, 312, -728, 728, -312, 728, -312, 312, -728,
            728, -312, 312, -312, 312, -728, 312, -312, 312, -312, 312, -728, 312, -312, 312, -312,
            728, -728, 312, -312, 312, -312, 312, -728, 312, -312, 312, -312, 312, -728, 312, -312,
            312, -312, 728, -728, 728, -312, 312, -312, 728, -728, 728, -312, 312, -312, 312, -728,
            312, -312, 728, -312, 312, -728, 312, -312, 728, -312, 728, -728, 312, -312, 312, -312,
            728, -728, -20000,
            ],
            frames: &[
                Expect { data: 0x1A2B3C4D0100D899, bits: 64, serial: Some(439041101), button: Some(1), counter: None, extra: None },
            ],
        },
        Case {
            name: "ok_b2",
            wire: &[
            312, -10400, 312, -728, 312, -312, 312, -312, 728, -728, 728, -312, 312, -312, 728, -728,
            312, -312, 312, -312, 312, -728, 728, -312, 312, -312, 728, -728, 312, -312, 728, -312,
            728, -728, 312, -312, 312, -312, 728, -728, 728, -312, 728, -312, 728, -728, 312, -312,
            312, -312, 312, -728, 728, -312, 312, -312, 312, -728, 728, -312, 728, -312, 312, -728,
            728, -312, 312, -312, 312, -728, 312, -312, 312, -312, 312, -728, 312, -312, 728, -312,
            312, -728, 312, -312, 312, -312, 312, -728, 312, -312, 312, -312, 312, -728, 312, -312,
            312, -312, 728, -728, 728, -312, 312, -312, 728, -728, 728, -312, 312, -312, 728, -728,
            728, -312, 728, -312, 312, -728, 312, -312, 728, -312, 728, -728, 312, -312, 312, -312,
            728, -728, -20000,
            ],
            frames: &[
                Expect { data: 0x1A2B3C4D0200DB99, bits: 64, serial: Some(439041101), button: Some(2), counter: None, extra: None },
            ],
        },
        Case {
            name: "ok_b4",
            wire: &[
            312, -10400, 312, -728, 312, -312, 312, -312, 312, -728, 312, -312, 312, -312, 312, -728,
            312, -312, 728, -312, 728, -728, 312, -312, 312, -312, 312, -728, 312, -312, 312, -312,
            312, -728, 728, -312, 728, -312, 728, -728, 728, -312, 728, -312, 728, -728, 728, -312,
            728, -312, 728, -728, 728, -312, 728, -312, 312, -728, 728, -312, 728, -312, 728, -728,
            312, -312, 312, -312, 312, -728, 312, -312, 312, -312, 312, -728, 728, -312, 312, -312,
            312, -728, 312, -312, 312, -312, 312, -728, 312, -312, 312, -312, 312, -728, 312, -312,
            312, -312, 312, -728, 312, -312, 312, -312, 312, -728, 312, -312, 728, -312, 312, -728,
            312, -312, 728, -312, 728, -728, 312, -312, 728, -312, 312, -728, 312, -312, 312, -312,
            728, -728, -20000,
            ],
            frames: &[
                Expect { data: 0xC0FFEE040004D1, bits: 64, serial: Some(12648430), button: Some(4), counter: None, extra: None },
            ],
        },
        Case {
            name: "ok_b8",
            wire: &[
            312, -10400, 312, -728, 728, -312, 728, -312, 728, -728, 728, -312, 728, -312, 728, -728,
            728, -312, 312, -312, 312, -728, 312, -312, 312, -312, 312, -728, 312, -312, 312, -312,
            312, -728, 312, -312, 312, -312, 312, -728, 312, -312, 312, -312, 312, -728, 312, -312,
            312, -312, 728, -728, 312, -312, 728, -312, 312, -728, 312, -312, 728, -312, 312, -728,
            728, -312, 312, -312, 312, -728, 312, -312, 312, -312, 728, -728, 312, -312, 312, -312,
            312, -728, 312, -312, 312, -312, 312, -728, 312, -312, 312, -312, 312, -728, 312, -312,
            312, -312, 728, -728, 312, -312, 312, -312, 312, -728, 728, -312, 312, -312, 312, -728,
            312, -312, 312, -312, 728, -728, 312, -312, 728, -312, 728, -728, 312, -312, 728, -312,
            312, -728, -20000,
            ],
            frames: &[
                Expect { data: 0x7F0000A50800885A, bits: 64, serial: Some(2130706597), button: Some(8), counter: None, extra: None },
            ],
        },
        Case {
            name: "ok_other_mask",
            wire: &[
            312, -10400, 312, -728, 312, -312, 312, -312, 728, -728, 728, -312, 312, -312, 728, -728,
            312, -312, 312, -312, 312, -728, 728, -312, 312, -312, 728, -728, 312, -312, 728, -312,
            728, -728, 312, -312, 312, -312, 728, -728, 728, -312, 728, -312, 728, -728, 312, -312,
            312, -312, 312, -728, 728, -312, 312, -312, 312, -728, 728, -312, 728, -312, 312, -728,
            728, -312, 312, -312, 312, -728, 312, -312, 312, -312, 312, -728, 312, -312, 728, -312,
            728, -728, 312, -312, 312, -312, 312, -728, 312, -312, 312, -312, 312, -728, 312, -312,
            312, -312, 728, -728, 728, -312, 312, -312, 728, -728, 728, -312, 312, -312, 728, -728,
            312, -312, 728, -312, 312, -728, 312, -312, 728, -312, 728, -728, 312, -312, 312, -312,
            728, -728, -20000,
            ],
            frames: &[
                Expect { data: 0x1A2B3C4D0300DA99, bits: 64, serial: Some(439041101), button: Some(3), counter: None, extra: None },
            ],
        },
    ];

    /// Bursts the C decoder reports nothing for: a wrong checksum, 63 bits, and a 7 ms sync gap.
    const REJECT: &[(&str, &[i32])] = &[
        (
            "bad_ck",
            &[
            312, -10400, 312, -728, 312, -312, 312, -312, 728, -728, 728, -312, 312, -312, 728, -728,
            312, -312, 312, -312, 312, -728, 728, -312, 312, -312, 728, -728, 312, -312, 728, -312,
            728, -728, 312, -312, 312, -312, 728, -728, 728, -312, 728, -312, 728, -728, 312, -312,
            312, -312, 312, -728, 728, -312, 312, -312, 312, -728, 728, -312, 728, -312, 312, -728,
            728, -312, 312, -312, 312, -728, 312, -312, 312, -312, 312, -728, 312, -312, 312, -312,
            728, -728, 312, -312, 312, -312, 312, -728, 312, -312, 312, -312, 312, -728, 312, -312,
            312, -312, 728, -728, 728, -312, 312, -312, 728, -728, 728, -312, 312, -312, 312, -728,
            312, -312, 728, -312, 312, -728, 312, -312, 728, -312, 728, -728, 312, -312, 312, -312,
            312, -728, -20000,
            ],
        ),
        (
            "short63",
            &[
            312, -10400, 312, -728, 312, -312, 312, -312, 728, -728, 728, -312, 312, -312, 728, -728,
            312, -312, 312, -312, 312, -728, 728, -312, 312, -312, 728, -728, 312, -312, 728, -312,
            728, -728, 312, -312, 312, -312, 728, -728, 728, -312, 728, -312, 728, -728, 312, -312,
            312, -312, 312, -728, 728, -312, 312, -312, 312, -728, 728, -312, 728, -312, 312, -728,
            728, -312, 312, -312, 312, -728, 312, -312, 312, -312, 312, -728, 312, -312, 312, -312,
            728, -728, 312, -312, 312, -312, 312, -728, 312, -312, 312, -312, 312, -728, 312, -312,
            312, -312, 728, -728, 728, -312, 312, -312, 728, -728, 728, -312, 312, -312, 312, -728,
            312, -312, 728, -312, 312, -728, 312, -312, 728, -312, 728, -728, 312, -312, 312, -312,
            -20000,
            ],
        ),
        (
            "wrong_sync",
            &[
            312, -7000, 312, -728, 312, -312, 312, -312, 728, -728, 728, -312, 312, -312, 728, -728,
            312, -312, 312, -312, 312, -728, 728, -312, 312, -312, 728, -728, 312, -312, 728, -312,
            728, -728, 312, -312, 312, -312, 728, -728, 728, -312, 728, -312, 728, -728, 312, -312,
            312, -312, 312, -728, 728, -312, 312, -312, 312, -728, 728, -312, 728, -312, 312, -728,
            728, -312, 312, -312, 312, -728, 312, -312, 312, -312, 312, -728, 312, -312, 312, -312,
            728, -728, 312, -312, 312, -312, 312, -728, 312, -312, 312, -312, 312, -728, 312, -312,
            312, -312, 728, -728, 728, -312, 312, -312, 728, -728, 728, -312, 312, -312, 312, -728,
            312, -312, 728, -312, 312, -728, 312, -312, 728, -312, 728, -728, 312, -312, 312, -312,
            728, -728, -20000,
            ],
        ),
    ];

    fn check(frames: &[DecodedSignal], want: &[Expect], name: &str) {
        assert_eq!(frames.len(), want.len(), "{name}: frame count");
        for (d, w) in frames.iter().zip(want) {
            assert_eq!((d.data, d.data_count_bit), (w.data, w.bits), "{name}");
            assert_eq!((d.serial, d.button, d.counter, d.extra), (w.serial, w.button, w.counter, w.extra), "{name}");
            assert!(d.encoder_capable, "{name}");
        }
    }

    #[test]
    fn decodes_what_the_c_decoder_decodes() {
        for c in ACCEPT {
            let got = feed_with_resets(&mut HyundaiKiaRioDecoder::new(), &wire(c.wire));
            check(&got, c.frames, c.name);
        }
    }

    #[test]
    fn rejects_what_the_c_decoder_rejects() {
        for (name, w) in REJECT {
            let got = feed_with_resets(&mut HyundaiKiaRioDecoder::new(), &wire(w));
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
            let got = reg.process_signal_stream(&pairs, 433920000);
            let names: Vec<_> = got.iter().map(|(n, _, _)| n.as_str()).collect();
            assert_eq!(names, vec![NAME; c.frames.len()], "{}", c.name);
        }
    }

    /// Key files: every accepted frame survives write, parse and load unchanged.
    #[test]
    fn key_files_round_trip() {
        let dec = HyundaiKiaRioDecoder::new();
        for c in ACCEPT {
            for d in feed_with_resets(&mut HyundaiKiaRioDecoder::new(), &wire(c.wire)) {
                assert_key_roundtrip(&dec, &d);
                let text = dec.to_flipper_key(&d).unwrap().to_text();
                assert!(text.contains("Protocol: Hyundai/Kia RIO\nBit: 64\n"), "{text}");
            }
        }
    }

    /// `subghz_oracle_foxfwenc encode` (FoxFW's `enc_hyundai_kia_rio.c` on the x86 host) of the key
    /// file FoxFW saves for frame "ok_b1" (`Key: 1A 2B 3C 4D 01 00 D8 99`, `Serial: 439041101`,
    /// `Btn: 1`): `RAW_Data` is 393 pulses, 3 copies of these 131. The oracle sends the same pulses
    /// for `Key: 1A 2B 3C 4D 01 23 D8 BA` (mask 0x0123, button 1) and reports `changed: Key`.
    const ORACLE_B1_COPY: &[i64] = &[
        312, -10400, 312, -728, 312, -728, 312, -728, 728, -312, 728, -312, 312, -728, 728, -312, 312, -728,
        312, -728, 312, -728, 728, -312, 312, -728, 728, -312, 312, -728, 728, -312, 728, -312, 312, -728,
        312, -728, 728, -312, 728, -312, 728, -312, 728, -312, 312, -728, 312, -728, 312, -728, 728, -312,
        312, -728, 312, -728, 728, -312, 728, -312, 312, -728, 728, -312, 312, -728, 312, -728, 312, -728,
        312, -728, 312, -728, 312, -728, 312, -728, 728, -312, 312, -728, 312, -728, 312, -728, 312, -728,
        312, -728, 312, -728, 312, -728, 312, -728, 728, -312, 728, -312, 312, -728, 728, -312, 728, -312,
        312, -728, 312, -728, 312, -728, 728, -312, 312, -728, 312, -728, 728, -312, 728, -312, 312, -728,
        312, -728, 728, -312, -10000,
    ];

    fn signed(tx: &[LevelDuration]) -> Vec<i64> {
        tx.iter().map(|p| if p.level { i64::from(p.duration_us) } else { -i64::from(p.duration_us) }).collect()
    }

    /// A FoxFW key file: the header, then `lines` (Bit, Key, Serial, Btn).
    fn key_file(lines: &str) -> FlipperKey {
        FlipperKey::parse(&format!(
            "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
             Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Hyundai/Kia RIO\n{lines}"
        ))
        .unwrap()
    }

    /// `tx` as the app sends it and a receiver sees it (`app/tx.rs`): twice, each followed by 20 ms
    /// of silence, adjacent equal levels merged.
    fn on_air(tx: &[LevelDuration]) -> Vec<LevelDuration> {
        let gap = [LevelDuration::new(false, 20_000)];
        let mut out: Vec<LevelDuration> = Vec::new();
        for p in tx.iter().chain(&gap).chain(tx).chain(&gap) {
            match out.last_mut() {
                Some(last) if last.level == p.level => last.duration_us += p.duration_us,
                _ => out.push(*p),
            }
        }
        out
    }

    /// KAT's transmission of a decode, with its own button, is FoxFW's encoder output for the key
    /// file FoxFW saves for it, pulse for pulse; a mask with a low byte goes out as `button << 8`.
    #[test]
    fn transmits_what_the_c_encoder_sends() {
        let dec = HyundaiKiaRioDecoder::new();
        let d = feed_with_resets(&mut HyundaiKiaRioDecoder::new(), &wire(ACCEPT[0].wire)).remove(0);
        assert!(dec.supports_encoding() && d.encoder_capable);
        let tx = signed(&dec.encode(&d, d.button.unwrap()).unwrap());
        assert_eq!((tx.len(), &tx), (393, &ORACLE_B1_COPY.repeat(REPEAT)));
        let low_byte = dec.from_flipper_key(&key_file("Bit: 64\nKey: 1A 2B 3C 4D 01 23 D8 BA\nSerial: 439041101\nBtn: 1\n")).unwrap();
        assert_eq!((low_byte.data, low_byte.button), (0x1A2B_3C4D_0123_D8BA, Some(1)));
        assert_eq!(signed(&dec.encode(&low_byte, 1).unwrap()), tx);
    }

    /// KAT's generic buttons are the masks' button codes (Flipper-ARF's `HKR_BTN_*` = code << 8), the
    /// serial goes out as captured, the checksum is recomputed; the frame sent decodes back through
    /// the registry as the app sends it, and has no counter to advance.
    #[test]
    fn buttons_map_to_masks() {
        use super::super::ProtocolRegistry;
        let dec = HyundaiKiaRioDecoder::new();
        let d = feed_with_resets(&mut HyundaiKiaRioDecoder::new(), &wire(ACCEPT[2].wire)).remove(0);
        for (button, mask) in [(0x01u8, 0x0100u64), (0x02, 0x0200), (0x04, 0x0400), (0x08, 0x0800)] {
            let got = ProtocolRegistry::new().process_signal_stream(&on_air(&dec.encode(&d, button).unwrap()), 433_920_000);
            assert_eq!(got.len(), 2 * REPEAT, "{button:#x}");
            for (name, x, _) in &got {
                assert_eq!(name, NAME);
                assert_eq!(((x.data >> 16) & 0xFFFF, x.serial, x.button, x.counter), (mask, d.serial, Some(button), None));
                assert!(x.crc_valid && x.encoder_capable);
            }
        }
    }

    /// Key files load as FoxFW's encoder reads them: any 8-byte Key at 64 bits; a frame the decoder
    /// reports (checksum holds, `Serial` / `Btn` agree) as it is, any other as the frame the encoder
    /// rebuilds (and writes back): a bad checksum is recomputed, `Serial` and `Btn` (low 8 bits)
    /// replace the Key's. Other bit counts and short keys are refused, so nothing transmits.
    #[test]
    fn key_files_load_as_the_c_encoder_reads_them() {
        let dec = HyundaiKiaRioDecoder::new();
        let load = |lines: &str| dec.from_flipper_key(&key_file(lines)).map(|d| (d.data, d.serial, d.button, d.crc_valid, d.encoder_capable));
        let b1 = Some((0x1A2B_3C4D_0100_D899, Some(0x1A2B_3C4D), Some(1), true, true));
        assert_eq!(load("Bit: 64\nKey: 1A 2B 3C 4D 01 00 D8 99\nSerial: 439041101\nBtn: 1\n"), b1);
        assert_eq!(load("Bit: 64\nKey: 1A 2B 3C 4D 01 00 D8 99\n"), b1);
        assert_eq!(load("Bit: 64\nKey: 1A 2B 3C 4D 01 00 00 00\n"), b1);
        // The oracle: Serial 305419896 + Btn 258 over this Key send (and write back) 12 34 56 78 02 00 B9 B3.
        assert_eq!(
            load("Bit: 64\nKey: 1A 2B 3C 4D 01 00 D8 99\nSerial: 305419896\nBtn: 258\nCnt: 7\n"),
            Some((0x1234_5678_0200_B9B3, Some(0x1234_5678), Some(2), true, true))
        );
        // A rebuilt key saves and loads back unchanged.
        assert_key_roundtrip(&dec, &dec.from_flipper_key(&key_file("Bit: 64\nKey: 1A 2B 3C 4D 01 00 00 00\nBtn: 8\n")).unwrap());
        for refused in ["Bit: 63\nKey: 1A 2B 3C 4D 01 00 D8 99\n", "Bit: 65\nKey: 1A 2B 3C 4D 01 00 D8 99\n",
            "Bit: 64\nKey: 2B 3C 4D 01 00 D8 99\n"]
        {
            assert_eq!(load(refused), None, "{refused}");
        }
        assert!(dec.encode(&decoded(0x1A2B_3C4D_0100_D899, 63), 1).is_none());
        assert_eq!(dec.name(), "Hyundai/Kia RIO");
    }

    /// Generator for the oracle comparison: `KAT_HYUNDAI_KIA_RIO_ENCODE=<file listing key files>`
    /// prints KAT's transmission of each (`KATENC <file> | <signed pulses>`, `(none)` when it does
    /// not load or encode), to diff with `subghz_oracle_foxfwenc encode <file>`.
    #[test]
    #[ignore = "oracle harness"]
    fn encode_key_files() {
        let Ok(list) = std::env::var("KAT_HYUNDAI_KIA_RIO_ENCODE") else { return };
        let dec = HyundaiKiaRioDecoder::new();
        for file in std::fs::read_to_string(list).unwrap().lines().filter(|l| !l.is_empty()) {
            let key = FlipperKey::parse(&std::fs::read_to_string(file).unwrap()).ok();
            let out = key.and_then(|k| dec.from_flipper_key(&k)).and_then(|d| dec.encode(&d, d.button.unwrap_or(0)));
            let text = out.map_or("(none)".to_string(), |tx| {
                signed(&tx).iter().map(i64::to_string).collect::<Vec<_>>().join(" ")
            });
            println!("KATENC {file} | {text}");
        }
    }
}
