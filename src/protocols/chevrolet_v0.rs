//! Chevrolet V0 (GM 433.92 MHz keyfob, Open / Close): OOK Manchester rolling-code remote —
//! **decode and display only**, as the reference.
//!
//! Ported from FoxFW `lib/subghz/protocols/chevrolet_v0.c` (itself a port of the KeeLoq-RX
//! `arke_mazda_gm.py` classes `ChevroletV0` / `ChevroletV0Gm`); the protocol name is FoxFW's,
//! "Chevrolet V0". The C is decode-only (`.encoder = NULL`; flags 433 | 315 | AM | Decodable | Load
//! | Save, type Dynamic) and has no checksum: the hop is encrypted and the gate is structural.
//!
//! **Air format**: standard Flipper (IEEE 802.3) Manchester with `te_short` 120 µs, `te_long`
//! 240 µs, tolerance `diff < 71`; every decoded bit is inverted. A 32-bit register locks on the
//! exact preamble `0xAAAAAAAB`, then 96 bits follow. The top 64 of them (`gdata`) hold
//! `Serial(24) | Button(4) | Hop(36)`; the low 32 bits are the encrypted "seed". There is no
//! clear counter.
//!
//! **Decoder** (`subghz_protocol_decoder_chevrolet_v0_feed`): every pulse is classified (short,
//! long, or Reset when it is neither). In the reset step any non-Reset event starts the preamble
//! (Manchester state `Mid1`, empty register) and is processed as its first event; a Reset event
//! stays in reset. In the preamble a Reset event goes back to the reset step; each decoded bit is
//! shifted into the 32-bit register and, from the 32nd bit on, the register equalling
//! `0xAAAAAAAB` starts the data. In the data step each bit shifts through the 64-bit data word into
//! the register (so after 96 bits the register holds bits 95..64 and the data word 63..0); a Reset
//! event drops the frame. At 96 bits the frame is dropped when `gdata` is 0 or all ones or the
//! button nibble (bits 39..36 of `gdata`) is not `0xA` (Open) or `0xB` (Close); otherwise it is
//! reported and the decoder returns to reset. The C's optional `CHEVROLET_V0_GM_ACCEPT_ANY`
//! (accept any button nibble for the wider GM family) is off in FoxFW and in KAT.
//!
//! **KAT fields**: `data` = `gdata` (64 bits), `data_count_bit` = 96, `serial` = 24 bits,
//! `button` = the nibble (0xA / 0xB), `extra` = the 32-bit seed, `counter` None, `crc_valid`
//! false, `encoder_capable` false.
//!
//! **Key file** (`serialize` / `deserialize`): `Protocol: Chevrolet V0`, `Bit: 96`, `Key` = `gdata`,
//! then `Serial`, `Btn`, `HopHi` (the hop's 4 high bits), `HopLo` (its low 32), `Seed`; loading
//! needs `Bit: 96` and reads `Seed` when present.
//!
//! **Deviations**: none. **Evidence**: `IMPORTS/chevy/22_chevy_trail_blazer_l2_u2_rs_p_t.sub`
//! (433.92 MHz OOK270; 21 frames with the constant serial 0xB809C5, button 0xA or 0xB; matched
//! frame for frame with the host build of FoxFW's C). The other GM captures in IMPORTS (Malibu,
//! Buick, GMC) decode in neither FoxFW nor KAT.

use super::common::{ManchesterEvent, ManchesterState};
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (see `info.rs`): `SubGhzProtocolFlag_433 | 315 | AM`, `SubGhzProtocolTypeDynamic`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Rolling code (encrypted hop)",
    make: "Chevrolet",
    category: Category::Vehicle,
    buttons: Some(button_name),
    presets: &[&AM650],
};

const NAME: &str = "Chevrolet V0";
const TE_SHORT: u32 = 120;
const TE_LONG: u32 = 240;
const TE_DELTA: u32 = 71;
const DATA_BITS: u16 = 96;
const PREAMBLE_WORD: u32 = 0xAAAA_AAAB;
const PREAMBLE_MIN: u16 = 32;

fn button_name(_name: &str, code: u8) -> Option<&'static str> {
    match code {
        0xA => Some("Open"),
        0xB => Some("Close"),
        _ => None,
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    Preamble,
    CollectData,
}

/// Chevrolet V0 decoder (`SubGhzProtocolDecoderChevroletV0`).
pub struct ChevroletV0Decoder {
    step: Step,
    manchester: ManchesterState,
    data_header: u32,
    bit_count: u16,
    data_count: u16,
    decode_data: u64,
}

/// `chevrolet_v0_event`: `level ? ShortLow : ShortHigh` (and the long forms).
fn event(level: bool, duration: u32) -> ManchesterEvent {
    if duration.abs_diff(TE_SHORT) < TE_DELTA {
        ManchesterEvent::from_half_bit(!level, false)
    } else if duration.abs_diff(TE_LONG) < TE_DELTA {
        ManchesterEvent::from_half_bit(!level, true)
    } else {
        ManchesterEvent::Reset
    }
}

/// `gdata`'s fields.
fn serial_of(gdata: u64) -> u32 {
    ((gdata >> 40) & 0xFF_FFFF) as u32
}

fn button_of(gdata: u64) -> u8 {
    ((gdata >> 36) & 0x0F) as u8
}

fn signal(gdata: u64, seed: u32) -> DecodedSignal {
    DecodedSignal {
        serial: Some(serial_of(gdata)),
        button: Some(button_of(gdata)),
        counter: None,
        crc_valid: false,
        data: gdata,
        data_count_bit: usize::from(DATA_BITS),
        encoder_capable: false,
        extra: Some(u64::from(seed)),
        protocol_display_name: None,
    }
}

impl ChevroletV0Decoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            manchester: ManchesterState::Mid1,
            data_header: 0,
            bit_count: 0,
            data_count: 0,
            decode_data: 0,
        }
    }
}

impl Default for ChevroletV0Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for ChevroletV0Decoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: usize::from(DATA_BITS),
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.step = Step::Reset;
        self.manchester = ManchesterState::Mid1;
        self.data_header = 0;
        self.bit_count = 0;
        self.data_count = 0;
        self.decode_data = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let ev = event(level, duration);
        if self.step == Step::Reset {
            if ev == ManchesterEvent::Reset {
                return None;
            }
            self.manchester = ManchesterState::Mid1;
            self.data_header = 0;
            self.bit_count = 0;
            self.step = Step::Preamble;
        }
        if self.step == Step::Preamble {
            if ev == ManchesterEvent::Reset {
                self.step = Step::Reset;
                return None;
            }
            if let Some(mbit) = self.manchester.advance(ev) {
                let data = u32::from(!mbit);
                self.data_header = (self.data_header << 1) | data;
                self.bit_count = self.bit_count.wrapping_add(1);
                if self.bit_count >= PREAMBLE_MIN && self.data_header == PREAMBLE_WORD {
                    self.data_header = 0;
                    self.decode_data = 0;
                    self.data_count = 0;
                    self.step = Step::CollectData;
                }
            }
        } else if self.step == Step::CollectData {
            if ev == ManchesterEvent::Reset {
                self.step = Step::Reset;
                return None;
            }
            if let Some(mbit) = self.manchester.advance(ev) {
                let data = u64::from(!mbit);
                self.data_header = (self.data_header << 1) | ((self.decode_data >> 63) & 1) as u32;
                self.decode_data = (self.decode_data << 1) | data;
                self.data_count += 1;
                if self.data_count >= DATA_BITS {
                    let gdata = (u64::from(self.data_header) << 32) | (self.decode_data >> 32);
                    let seed = (self.decode_data & 0xFFFF_FFFF) as u32;
                    self.step = Step::Reset;
                    let btn = button_of(gdata);
                    if gdata == 0 || gdata == u64::MAX || !(btn == 0xA || btn == 0xB) {
                        return None;
                    }
                    return Some(signal(gdata, seed));
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

    /// `subghz_protocol_decoder_chevrolet_v0_deserialize`: `Bit: 96`; the fields come from `Key`,
    /// `Seed` is read when present (else 0).
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != usize::from(DATA_BITS) {
            return None;
        }
        Some(signal(key.key, key.u32_field("Seed").unwrap_or(0)))
    }

    /// `subghz_protocol_decoder_chevrolet_v0_serialize`.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != usize::from(DATA_BITS) {
            return None;
        }
        let gdata = decoded.data;
        let hop = gdata & 0xF_FFFF_FFFF;
        let mut key = FlipperKey::for_protocol(NAME, usize::from(DATA_BITS), gdata);
        key.push_u32("Serial", serial_of(gdata));
        key.push_u32("Btn", button_of(gdata).into());
        key.push_u32("HopHi", (hop >> 32) as u32);
        key.push_u32("HopLo", (hop & 0xFFFF_FFFF) as u32);
        key.push_u32("Seed", decoded.extra? as u32);
        Some(key)
    }
}

/// Synthetic captures for the tests.
#[cfg(test)]
pub(super) mod testing {
    use super::*;

    /// Pulses of a frame: `pre` preamble bits (alternating, ending in the `0xAAAAAAAB` word), the 96
    /// payload bits (`gdata` then `seed`), then a long LOW. Bits are on-air (inverted from the
    /// decoded value): a decoded `1` is a LOW-then-HIGH pair.
    pub fn frame(pre: usize, gdata: u64, seed: u32) -> Vec<LevelDuration> {
        let mut bits: Vec<bool> = (0..pre.saturating_sub(32)).map(|i| i % 2 == 1).collect();
        bits.extend((0..32).rev().map(|i| (PREAMBLE_WORD >> i) & 1 == 1));
        bits.extend((0..64).rev().map(|i| (gdata >> i) & 1 == 1));
        bits.extend((0..32).rev().map(|i| (seed >> i) & 1 == 1));
        let mut halves = Vec::new();
        for b in bits {
            halves.extend([!b, b]);
        }
        let mut out = Vec::new();
        let mut i = 0;
        while i < halves.len() {
            let level = halves[i];
            let run = if halves.get(i + 1) == Some(&level) { 2 } else { 1 };
            out.push(LevelDuration::new(level, if run == 2 { TE_LONG } else { TE_SHORT }));
            i += run;
        }
        out.push(LevelDuration::new(false, 5_000));
        out
    }
}

#[cfg(test)]
mod tests {
    use super::super::f_common::test_support::*;
    use super::*;

    const GDATA: u64 = 0xB809_C5B4_779E_46A9;
    const SEED: u32 = 980_305_051;

    #[test]
    fn decodes_a_synthetic_frame() {
        let got = feed_resetting(&mut ChevroletV0Decoder::new(), &testing::frame(40, GDATA, SEED));
        assert_eq!(got.len(), 1, "{got:?}");
        let d = &got[0];
        assert_eq!((d.serial, d.button, d.counter), (Some(0xB8_09C5), Some(0xB), None));
        assert_eq!((d.data, d.data_count_bit, d.extra), (GDATA, 96, Some(u64::from(SEED))));
        assert!(!d.encoder_capable && !d.crc_valid);
    }

    #[test]
    fn gates_match_the_c() {
        // Open is accepted, any other button nibble dropped
        let open = (GDATA & !(0xF << 36)) | (0xA << 36);
        assert_eq!(feed_resetting(&mut ChevroletV0Decoder::new(), &testing::frame(40, open, SEED)).len(), 1);
        let other = (GDATA & !(0xF << 36)) | (0x3 << 36);
        assert!(feed_resetting(&mut ChevroletV0Decoder::new(), &testing::frame(40, other, SEED)).is_empty());
        // all-ones / zero payloads are dropped (the nibble gate already would, 0xF / 0x0)
        assert!(feed_resetting(&mut ChevroletV0Decoder::new(), &testing::frame(40, 0, 0)).is_empty());
        // an out-of-tolerance pulse inside the data drops the frame
        let mut wire = testing::frame(40, GDATA, SEED);
        let mid = wire.len() * 3 / 4;
        wire[mid].duration_us = 1_000;
        assert!(feed_resetting(&mut ChevroletV0Decoder::new(), &wire).is_empty());
        assert!(ChevroletV0Decoder::new().encode(&DecodedSignal::new(0, 96), 1).is_none());
    }

    #[test]
    fn key_file_round_trip() {
        let dec = ChevroletV0Decoder::new();
        let d = signal(GDATA, SEED);
        let text = super::super::flipper_key::testing::assert_round_trip(&dec, &d);
        assert!(
            text.contains("Protocol: Chevrolet V0\nBit: 96\nKey: B8 09 C5 B4 77 9E 46 A9\n"),
            "{text}"
        );
        assert!(
            text.contains("Serial: 12061125\nBtn: 11\nHopHi: 4\nHopLo: 2006861481\nSeed: 980305051\n"),
            "{text}"
        );
    }

    /// FoxFW's C on the Trail Blazer capture: 21 frames, constant serial 0xB809C5, button 0xA or
    /// 0xB; the first frames' `Key`, `HopHi`, `HopLo` and `Seed`.
    #[test]
    fn imports_trail_blazer_matches_the_reference() {
        let path = std::path::Path::new("IMPORTS/chevy/22_chevy_trail_blazer_l2_u2_rs_p_t.sub");
        if !path.is_file() {
            eprintln!("Skip: IMPORTS not found (run from crate root)");
            return;
        }
        let (_, wire) = sub_pulses(path);
        let got = feed_resetting(&mut ChevroletV0Decoder::new(), &wire);
        assert_eq!(got.len(), 21);
        assert!(got.iter().all(|d| d.serial == Some(12_061_125) && matches!(d.button, Some(0xA | 0xB))));
        let first: Vec<(u64, u64)> = got.iter().take(3).map(|d| (d.data, d.extra.unwrap())).collect();
        assert_eq!(
            first,
            [
                (0xB809_C5B4_779E_46A9, 980_305_051),
                (0xB809_C5B3_5FD2_0392, 3_581_925_308),
                (0xB809_C5A2_93A9_A01E, 114_341_953),
            ]
        );
        assert_eq!(got[0].data & 0xF_FFFF_FFFF, (4 << 32) | 2_006_861_481);
    }
}
