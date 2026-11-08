//! SantaFe 13-16: 80-bit frame with a CRC-8.
//!
//! Ported from FoxFW `lib/subghz/protocols/santa_fe.c` (`SANTA_FE_PROTOCOL_NAME = "SantaFe 13-16"`;
//! `SubGhzProtocolTypeStatic`, flags `433 | AM`) and its SD-card encoder plugin
//! `applications/main/subghz/protocols_ext/enc_santa_fe.c`, with the `pp_auto_common.c` helpers the
//! firmware's host bridge hands it (`ppa_emit`, `ppa_encoder_read_fields`, `ppa_encoder_read_repeat`).
//!
//! **Provenance.** The format comes from Flipper-ARF `lib/subghz/protocols/auto_rke_protocols.c`
//! ("Additional automotive RKE protocols — ported from Pandora DXL 5000 firmware"; `santafe_decode`, "SantaFe 13-16 (ID 0x1A)", the Hyundai Santa Fe / Solaris TRW fob), a
//! reverse-engineering file ARF never registers, like its other Pandora-derived decoders (Audi,
//! Hyundai, Mazda Infinity, which KAT keeps unregistered). A KAT fork (limbo111111/KAT, written
//! by Google's Jules bot in May 2026) ported it to Rust "aligned with Flipper-ARF reference:
//! `auto_rke_protocols.c`"; FoxFW's C ("ported from KAT src/protocols/santa_fe.rs") comes from that
//! fork, and KAT's module is ported from FoxFW's C. KAT registers it because FoxFW does (owner's
//! decision, 2026-11-07). No capture of the remote exists in any reference or in KAT's corpus.
//!
//! **Air format** (433.92 MHz, AM): te_short 125 µs, te_long 375 µs, te_delta 100 µs. A frame is a
//! HIGH 375 (±100) + LOW 12000 (±1800) sync, then 80 pulse pairs: HIGH 375 = 1, HIGH 125 = 0, each
//! followed by a LOW of 125 or 375 (±100, the gap carries nothing). The frame is reported on the
//! LOW of the 80th pair when its CRC holds; a HIGH or LOW longer than 3000 µs after 80 bits reports
//! it too. Further bits past 80 are ignored.
//!
//! **Frame** (10 bytes, MSB first): bytes 0..3 `rolling` (32 bits), 4..6 `serial` (24), 7 `counter`
//! (8), 8 `button`, 9 CRC-8 (poly 0x31, init 0xFF, no reflection, no final XOR) over bytes 0..8.
//! A frame with any other CRC is dropped.
//!
//! **DecodedSignal packing** (80 bits > 64): `data` = bytes 0..7 (`rolling << 32 | serial << 8 |
//! counter`, the C's `generic.data`), `extra` = `button << 8 | crc`, `data_count_bit` = 80.
//!
//! **Key file**: `Protocol: SantaFe 13-16`, `Bit: 80`, `Key` = the first 8 bytes, then `Serial`,
//! `Btn`, `Cnt` and `Rolling` (decimal). KAT loads a file as FoxFW's encoder reads it
//! (`subghz_protocol_encoder_santa_fe_deserialize`), into the frame that encoder sends: rolling =
//! `Rolling` (else Key bits 63..32), serial = `Serial` (its low 24 bits; else Key bits 31..8),
//! counter = `Cnt` (its low 8 bits; else Key bits 7..0), button = `Btn` (its low 8 bits; else 0, the
//! encoder's zeroed `generic.btn`), CRC recomputed; any 8-byte Key at exactly 80 bits loads. FoxFW's
//! decoder `deserialize` takes serial and counter from the Key and reads no button; on every file
//! FoxFW writes the two agree. (The C finds `Rolling` only after the `Key` line, where FoxFW writes
//! it; KAT reads it anywhere.)
//!
//! **Transmit** (`subghz_protocol_encoder_santa_fe_deserialize` + `sf_build_upload`): that frame
//! with the button asked for and a fresh CRC, 3 copies (`ppa_encoder_read_repeat`'s default) of
//! HIGH 375, LOW 12000, the 80 bits MSB first (1 = HIGH 375 + LOW 125, 0 = HIGH 125 + LOW 375) and
//! LOW 15000, which follows the last bit's LOW as a pulse of its own, as the C yields it. The C
//! sends the file's `Btn` and keeps the rolling word, serial and counter, so nothing advances and a
//! transmission leaves the capture as it is. KAT's TX Lock / Unlock / Trunk / Panic send buttons
//! 0x01 / 0x02 / 0x04 / 0x08, the codes Flipper-ARF's `auto_rke_protocols.c` names (`SFE_BTN_LOCK`
//! … `SFE_BTN_PANIC`); "Send next code" and the key-file tests send the capture's own button. A
//! `Repeat` line is not applied (FoxFW deletes it before saving a file).

use super::g2_common::{decoded, near};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`): `SubGhzProtocolFlag_433 | AM`, static type, CRC-8.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Hyundai",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "SantaFe 13-16";
const TE_SHORT: u32 = 125;
const TE_LONG: u32 = 375;
const TE_DELTA: u32 = 100;
const SYNC_US: u32 = 12_000;
const SYNC_DELTA: u32 = 1_800;
const BIT_COUNT: usize = 80;
const BYTES: usize = 10;
/// A HIGH or LOW over this ends the data (`duration > 3000`).
const END_US: u32 = 3_000;
/// The encoder's closing LOW (`SF_GAP_US`).
const GAP_US: u32 = 15_000;
/// `ppa_encoder_read_repeat(flipper_format, 3)` without a `Repeat` line.
const REPEAT: usize = 3;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    CheckSyncHi,
    SaveDuration,
    CheckDuration,
}

/// SantaFe 13-16 decoder (`SubGhzProtocolDecoderSantaFe`).
pub struct SantaFeDecoder {
    step: Step,
    data: [u8; BYTES],
    bit_count: usize,
}

/// `sf_crc8`: CRC-8, poly 0x31, init 0xFF.
fn crc8(data: &[u8]) -> u8 {
    let mut crc = 0xFFu8;
    for &b in data {
        crc ^= b;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 { (crc << 1) ^ 0x31 } else { crc << 1 };
        }
    }
    crc
}

/// `sf_build_upload`'s `pkt`: `rolling`, the low 24 bits of `serial`, `counter`, `button`, CRC-8.
fn frame(rolling: u32, serial: u32, counter: u8, button: u8) -> [u8; BYTES] {
    let mut b = [0u8; BYTES];
    b[..4].copy_from_slice(&rolling.to_be_bytes());
    b[4..7].copy_from_slice(&serial.to_be_bytes()[1..]);
    b[7] = counter;
    b[8] = button;
    b[9] = crc8(&b[..9]);
    b
}

/// `sf_build_upload`: one copy (163 pulses).
fn upload(b: &[u8; BYTES]) -> Vec<LevelDuration> {
    let mut up = Vec::with_capacity(2 + 2 * BIT_COUNT + 1);
    up.extend([LevelDuration::new(true, TE_LONG), LevelDuration::new(false, SYNC_US)]);
    for i in 0..BIT_COUNT {
        let (high, low) = if (b[i / 8] >> (7 - i % 8)) & 1 == 1 { (TE_LONG, TE_SHORT) } else { (TE_SHORT, TE_LONG) };
        up.extend([LevelDuration::new(true, high), LevelDuration::new(false, low)]);
    }
    up.push(LevelDuration::new(false, GAP_US));
    up
}

/// `sf_process`: None when the CRC does not hold. Every frame transmits.
fn process(b: &[u8; BYTES]) -> Option<DecodedSignal> {
    if b[9] != crc8(&b[..9]) {
        return None;
    }
    let data = u64::from_be_bytes([b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]]);
    let mut d = decoded(data, BIT_COUNT);
    d.serial = Some(u32::from(b[4]) << 16 | u32::from(b[5]) << 8 | u32::from(b[6]));
    d.button = Some(b[8]);
    d.counter = Some(u16::from(b[7]));
    d.extra = Some(u64::from(b[8]) << 8 | u64::from(b[9]));
    d.crc_valid = true;
    d.encoder_capable = true;
    Some(d)
}

impl SantaFeDecoder {
    pub fn new() -> Self {
        Self { step: Step::Reset, data: [0; BYTES], bit_count: 0 }
    }

    /// `sf_add_bit` (bits past 80 are dropped).
    fn add_bit(&mut self, bit: bool) {
        if self.bit_count < BIT_COUNT {
            if bit {
                self.data[self.bit_count / 8] |= 1 << (7 - self.bit_count % 8);
            }
            self.bit_count += 1;
        }
    }
}

impl Default for SantaFeDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for SantaFeDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: TE_SHORT, te_long: TE_LONG, te_delta: TE_DELTA, min_count_bit: BIT_COUNT }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.step = Step::Reset;
        self.data = [0; BYTES];
        self.bit_count = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let mut out = None;
        match self.step {
            Step::Reset => {
                if level && near(duration, TE_LONG, TE_DELTA) {
                    self.step = Step::CheckSyncHi;
                }
            }
            Step::CheckSyncHi => {
                if !level && near(duration, SYNC_US, SYNC_DELTA) {
                    self.step = Step::SaveDuration;
                    self.data = [0; BYTES];
                    self.bit_count = 0;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::SaveDuration => {
                if !level {
                    self.step = Step::Reset;
                } else if near(duration, TE_LONG, TE_DELTA) {
                    self.add_bit(true);
                    self.step = Step::CheckDuration;
                } else if near(duration, TE_SHORT, TE_DELTA) {
                    self.add_bit(false);
                    self.step = Step::CheckDuration;
                } else {
                    if duration > END_US && self.bit_count >= BIT_COUNT {
                        out = process(&self.data);
                    }
                    self.step = Step::Reset;
                }
            }
            Step::CheckDuration => {
                if level {
                    self.step = Step::Reset;
                } else if near(duration, TE_SHORT, TE_DELTA) || near(duration, TE_LONG, TE_DELTA) {
                    if self.bit_count >= BIT_COUNT {
                        out = process(&self.data);
                        self.step = Step::Reset;
                    } else {
                        self.step = Step::SaveDuration;
                    }
                } else {
                    if duration > END_US && self.bit_count >= BIT_COUNT {
                        out = process(&self.data);
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

    /// FoxFW's `enc_santa_fe.c` on the key file KAT writes for `decoded` with `Btn` = `button`:
    /// rolling word, serial and counter as captured, `button`, a fresh CRC, 3 times.
    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != BIT_COUNT {
            return None;
        }
        let data = decoded.data;
        let serial = decoded.serial.unwrap_or((data >> 8) as u32);
        let counter = decoded.counter.map_or(data as u8, |c| c as u8);
        Some(upload(&frame((data >> 32) as u32, serial, counter, button)).repeat(REPEAT))
    }

    /// `subghz_protocol_encoder_santa_fe_deserialize`: the frame the encoder sends from the file
    /// (see the module doc).
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != BIT_COUNT {
            return None;
        }
        let data = key.key_u64()?;
        let rolling = key.u32_field("Rolling").unwrap_or((data >> 32) as u32);
        let serial = key.u32_field("Serial").unwrap_or((data >> 8) as u32);
        let counter = key.u32_field("Cnt").unwrap_or(data as u32) as u8;
        let button = key.u32_field("Btn").unwrap_or(0) as u8;
        process(&frame(rolling, serial, counter, button))
    }

    fn to_flipper_key(&self, d: &DecodedSignal) -> Option<FlipperKey> {
        let mut key = FlipperKey::for_protocol(NAME, d.data_count_bit, d.data);
        key.push_u32("Serial", d.serial?);
        key.push_u32("Btn", u32::from(d.button?));
        key.push_u32("Cnt", u32::from(d.counter?));
        key.push_u32("Rolling", (d.data >> 32) as u32);
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
            name: "ok1",
            wire: &[
            375, -12000, 375, -125, 125, -375, 125, -125, 125, -375, 375, -125, 125, -375, 125, -125,
            375, -375, 375, -125, 125, -375, 375, -125, 125, -375, 375, -125, 125, -375, 375, -125,
            375, -375, 375, -125, 375, -375, 125, -125, 125, -375, 375, -125, 375, -375, 125, -125,
            375, -375, 375, -125, 375, -375, 375, -125, 125, -375, 375, -125, 375, -375, 375, -125,
            375, -375, 125, -125, 125, -375, 125, -125, 375, -375, 125, -125, 125, -375, 375, -125,
            125, -375, 125, -125, 125, -375, 375, -125, 375, -375, 125, -125, 375, -375, 125, -125,
            125, -375, 125, -125, 375, -375, 125, -125, 375, -375, 125, -125, 375, -375, 375, -125,
            125, -375, 125, -125, 375, -375, 125, -125, 125, -375, 125, -125, 125, -375, 375, -125,
            125, -375, 125, -125, 125, -375, 125, -125, 125, -375, 125, -125, 125, -375, 375, -125,
            125, -375, 375, -125, 125, -375, 375, -125, 125, -375, 125, -125, 125, -375, 375, -125,
            375, -375, -20000,
            ],
            frames: &[
                Expect { data: 0x89ABCDEF12345642, bits: 80, serial: Some(1193046), button: Some(2), counter: Some(66), extra: Some(675) },
            ],
        },
        Case {
            name: "ok2",
            wire: &[
            375, -12000, 125, -125, 125, -375, 125, -125, 125, -375, 125, -125, 125, -375, 125, -125,
            125, -375, 125, -125, 125, -375, 125, -125, 125, -375, 125, -125, 125, -375, 125, -125,
            125, -375, 125, -125, 125, -375, 125, -125, 125, -375, 125, -125, 125, -375, 125, -125,
            125, -375, 125, -125, 125, -375, 125, -125, 125, -375, 125, -125, 125, -375, 125, -125,
            375, -375, 375, -125, 125, -375, 375, -125, 125, -375, 125, -125, 125, -375, 125, -125,
            375, -375, 375, -125, 125, -375, 375, -125, 375, -375, 125, -125, 125, -375, 375, -125,
            125, -375, 375, -125, 375, -375, 125, -125, 125, -375, 125, -125, 125, -375, 375, -125,
            375, -375, 375, -125, 375, -375, 375, -125, 375, -375, 375, -125, 375, -375, 375, -125,
            375, -375, 125, -125, 125, -375, 125, -125, 375, -375, 125, -125, 125, -375, 125, -125,
            125, -375, 125, -125, 125, -375, 125, -125, 125, -375, 375, -125, 125, -375, 375, -125,
            375, -375, -20000,
            ],
            frames: &[
                Expect { data: 0x1A1B2C3FF, bits: 80, serial: Some(10597059), button: Some(16), counter: Some(255), extra: Some(4107) },
            ],
        },
    ];

    /// Bursts the C decoder reports nothing for: a wrong CRC-8, 79 bits, and a 9 ms sync gap.
    const REJECT: &[(&str, &[i32])] = &[
        (
            "bad_crc",
            &[
            375, -12000, 375, -125, 125, -375, 125, -125, 125, -375, 375, -125, 125, -375, 125, -125,
            375, -375, 375, -125, 125, -375, 375, -125, 125, -375, 375, -125, 125, -375, 375, -125,
            375, -375, 375, -125, 375, -375, 125, -125, 125, -375, 375, -125, 375, -375, 125, -125,
            375, -375, 375, -125, 375, -375, 375, -125, 125, -375, 375, -125, 375, -375, 375, -125,
            375, -375, 125, -125, 125, -375, 125, -125, 375, -375, 125, -125, 125, -375, 375, -125,
            125, -375, 125, -125, 125, -375, 375, -125, 375, -375, 125, -125, 375, -375, 125, -125,
            125, -375, 125, -125, 375, -375, 125, -125, 375, -375, 125, -125, 375, -375, 375, -125,
            125, -375, 125, -125, 375, -375, 125, -125, 125, -375, 125, -125, 125, -375, 375, -125,
            125, -375, 125, -125, 125, -375, 125, -125, 125, -375, 125, -125, 125, -375, 375, -125,
            125, -375, 125, -125, 125, -375, 375, -125, 125, -375, 125, -125, 125, -375, 375, -125,
            375, -375, -20000,
            ],
        ),
        (
            "short79",
            &[
            375, -12000, 375, -125, 125, -375, 125, -125, 125, -375, 375, -125, 125, -375, 125, -125,
            375, -375, 375, -125, 125, -375, 375, -125, 125, -375, 375, -125, 125, -375, 375, -125,
            375, -375, 375, -125, 375, -375, 125, -125, 125, -375, 375, -125, 375, -375, 125, -125,
            375, -375, 375, -125, 375, -375, 375, -125, 125, -375, 375, -125, 375, -375, 375, -125,
            375, -375, 125, -125, 125, -375, 125, -125, 375, -375, 125, -125, 125, -375, 375, -125,
            125, -375, 125, -125, 125, -375, 375, -125, 375, -375, 125, -125, 375, -375, 125, -125,
            125, -375, 125, -125, 375, -375, 125, -125, 375, -375, 125, -125, 375, -375, 375, -125,
            125, -375, 125, -125, 375, -375, 125, -125, 125, -375, 125, -125, 125, -375, 375, -125,
            125, -375, 125, -125, 125, -375, 125, -125, 125, -375, 125, -125, 125, -375, 375, -125,
            125, -375, 375, -125, 125, -375, 375, -125, 125, -375, 125, -125, 125, -375, 375, -125,
            -20000,
            ],
        ),
        (
            "wrong_sync",
            &[
            375, -9000, 375, -125, 125, -375, 125, -125, 125, -375, 375, -125, 125, -375, 125, -125,
            375, -375, 375, -125, 125, -375, 375, -125, 125, -375, 375, -125, 125, -375, 375, -125,
            375, -375, 375, -125, 375, -375, 125, -125, 125, -375, 375, -125, 375, -375, 125, -125,
            375, -375, 375, -125, 375, -375, 375, -125, 125, -375, 375, -125, 375, -375, 375, -125,
            375, -375, 125, -125, 125, -375, 125, -125, 375, -375, 125, -125, 125, -375, 375, -125,
            125, -375, 125, -125, 125, -375, 375, -125, 375, -375, 125, -125, 375, -375, 125, -125,
            125, -375, 125, -125, 375, -375, 125, -125, 375, -375, 125, -125, 375, -375, 375, -125,
            125, -375, 125, -125, 375, -375, 125, -125, 125, -375, 125, -125, 125, -375, 375, -125,
            125, -375, 125, -125, 125, -375, 125, -125, 125, -375, 125, -125, 125, -375, 375, -125,
            125, -375, 375, -125, 125, -375, 375, -125, 125, -375, 125, -125, 125, -375, 375, -125,
            375, -375, -20000,
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
            let got = feed_with_resets(&mut SantaFeDecoder::new(), &wire(c.wire));
            check(&got, c.frames, c.name);
        }
    }

    #[test]
    fn rejects_what_the_c_decoder_rejects() {
        for (name, w) in REJECT {
            let got = feed_with_resets(&mut SantaFeDecoder::new(), &wire(w));
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
        let dec = SantaFeDecoder::new();
        for c in ACCEPT {
            for d in feed_with_resets(&mut SantaFeDecoder::new(), &wire(c.wire)) {
                assert_key_roundtrip(&dec, &d);
                let text = dec.to_flipper_key(&d).unwrap().to_text();
                assert!(text.contains("Protocol: SantaFe 13-16\nBit: 80\n"), "{text}");
            }
        }
    }

    /// `subghz_oracle_foxfwenc encode` (FoxFW's `enc_santa_fe.c` on the x86 host) of the key file
    /// FoxFW saves for frame "ok1" (`Key: 89 AB CD EF 12 34 56 42`, `Serial: 1193046`, `Btn: 2`,
    /// `Cnt: 66`, `Rolling: 2309737967`): `RAW_Data` is 489 pulses, 3 copies of these 163.
    const ORACLE_OK1_COPY: &[i64] = &[
        375, -12000, 375, -125, 125, -375, 125, -375, 125, -375, 375, -125, 125, -375, 125, -375, 375, -125,
        375, -125, 125, -375, 375, -125, 125, -375, 375, -125, 125, -375, 375, -125, 375, -125, 375, -125,
        375, -125, 125, -375, 125, -375, 375, -125, 375, -125, 125, -375, 375, -125, 375, -125, 375, -125,
        375, -125, 125, -375, 375, -125, 375, -125, 375, -125, 375, -125, 125, -375, 125, -375, 125, -375,
        375, -125, 125, -375, 125, -375, 375, -125, 125, -375, 125, -375, 125, -375, 375, -125, 375, -125,
        125, -375, 375, -125, 125, -375, 125, -375, 125, -375, 375, -125, 125, -375, 375, -125, 125, -375,
        375, -125, 375, -125, 125, -375, 125, -375, 375, -125, 125, -375, 125, -375, 125, -375, 125, -375,
        375, -125, 125, -375, 125, -375, 125, -375, 125, -375, 125, -375, 125, -375, 125, -375, 375, -125,
        125, -375, 375, -125, 125, -375, 375, -125, 125, -375, 125, -375, 125, -375, 375, -125, 375, -125,
        -15000,
    ];

    fn signed(tx: &[LevelDuration]) -> Vec<i64> {
        tx.iter().map(|p| if p.level { i64::from(p.duration_us) } else { -i64::from(p.duration_us) }).collect()
    }

    /// A FoxFW key file: the header, then `lines` (Bit, Key, Serial ...).
    fn key_file(lines: &str) -> FlipperKey {
        FlipperKey::parse(&format!(
            "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
             Preset: FuriHalSubGhzPresetOok650Async\nProtocol: SantaFe 13-16\n{lines}"
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

    fn ok1() -> DecodedSignal {
        feed_with_resets(&mut SantaFeDecoder::new(), &wire(ACCEPT[0].wire)).remove(0)
    }

    /// KAT's transmission of a decode, with its own button, is FoxFW's encoder output for the key
    /// file FoxFW saves for it, pulse for pulse.
    #[test]
    fn transmits_what_the_c_encoder_sends() {
        let dec = SantaFeDecoder::new();
        let d = ok1();
        assert!(dec.supports_encoding() && d.encoder_capable);
        let tx = signed(&dec.encode(&d, d.button.unwrap()).unwrap());
        assert_eq!((tx.len(), tx), (489, ORACLE_OK1_COPY.repeat(REPEAT)));
    }

    /// KAT's generic buttons are the frame's button codes (Flipper-ARF's `SFE_BTN_*`); the rolling
    /// word, serial and counter go out as captured (the C advances nothing), the CRC is recomputed.
    #[test]
    fn buttons_map_and_nothing_advances() {
        use super::super::ProtocolRegistry;
        let dec = SantaFeDecoder::new();
        let d = ok1();
        for button in [0x01, 0x02, 0x04, 0x08] {
            let got = ProtocolRegistry::new().process_signal_stream(&on_air(&dec.encode(&d, button).unwrap()), 433_920_000);
            assert_eq!(got.len(), 2 * REPEAT, "{button:#x}");
            for (name, x, _) in &got {
                assert_eq!(name, NAME);
                assert_eq!((x.data, x.serial, x.counter, x.button), (d.data, d.serial, d.counter, Some(button)));
                assert_eq!(x.extra, Some(u64::from(button) << 8 | u64::from(frame(0x89AB_CDEF, 0x12_3456, 0x42, button)[9])));
                assert!(x.crc_valid && x.encoder_capable);
            }
        }
    }

    /// Key files load as FoxFW's encoder reads them: `Rolling`, `Serial` (low 24 bits), `Cnt` (low 8)
    /// and `Btn` (low 8; none = 0) over the Key, CRC recomputed. Other bit counts and short keys are
    /// refused, so nothing transmits.
    #[test]
    fn key_files_load_as_the_c_encoder_reads_them() {
        let dec = SantaFeDecoder::new();
        let load = |lines: &str| dec.from_flipper_key(&key_file(lines));
        let key = "Bit: 80\nKey: 89 AB CD EF 12 34 56 42\n";
        // FoxFW's own file of the decode, and the Key alone (button 0, as the C's encoder sends it).
        let full = load(&format!("{key}Serial: 1193046\nBtn: 2\nCnt: 66\nRolling: 2309737967\n")).unwrap();
        assert_eq!(full, ok1());
        let bare = load(key).unwrap();
        assert_eq!((bare.data, bare.button, bare.extra), (full.data, Some(0), Some(u64::from(frame(0x89AB_CDEF, 0x12_3456, 0x42, 0)[9]))));
        // Lines over the Key: Serial 0x7F123456 → 0x123456, Btn 500 → 0xF4, Cnt 511 → 0xFF, Rolling 1.
        let edited = load(&format!("{key}Serial: 2131899478\nBtn: 500\nCnt: 511\nRolling: 1\n")).unwrap();
        assert_eq!((edited.data, edited.serial, edited.button, edited.counter), (0x0000_0001_1234_56FF, Some(0x12_3456), Some(0xF4), Some(0xFF)));
        assert!(edited.encoder_capable && edited.crc_valid);
        assert_key_roundtrip(&dec, &edited);
        for refused in ["Bit: 79\nKey: 89 AB CD EF 12 34 56 42\nBtn: 2\n", "Bit: 64\nKey: 89 AB CD EF 12 34 56 42\nBtn: 2\n",
            "Bit: 80\nKey: AB CD EF 12 34 56 42\nBtn: 2\n"]
        {
            assert_eq!(load(refused), None, "{refused}");
        }
        assert!(dec.encode(&decoded(0x89AB_CDEF_1234_5642, 64), 1).is_none());
        assert_eq!(dec.name(), "SantaFe 13-16");
    }

    /// Generator for the oracle comparison: `KAT_SANTA_FE_ENCODE=<file listing key files>` prints
    /// KAT's transmission of each (`KATENC <file> | <signed pulses>`, `(none)` when it does not load
    /// or encode), to diff with `subghz_oracle_foxfwenc encode <file>`.
    #[test]
    #[ignore = "oracle harness"]
    fn encode_key_files() {
        let Ok(list) = std::env::var("KAT_SANTA_FE_ENCODE") else { return };
        let dec = SantaFeDecoder::new();
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
