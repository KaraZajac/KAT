//! Audi V1 (FoxFW `audi_v1.c`, `SUBGHZ_PROTOCOL_AUDI_V1_NAME = "Audi V1"`): 433 MHz AM Manchester
//! VAG key fob, 88 bits — **decode only** (FoxFW's encoder is an SD plugin KAT does not port).
//!
//! Ported from FoxFW's `lib/subghz/protocols/audi_v1.c`. This is not KAT's unregistered "Audi"
//! module (`audi.rs`, the Flipper-ARF/ProtoPirate VAG-derived decoder): a different preamble and a
//! different frame.
//!
//! **Air format**: Manchester, te 260 / 520 µs (±130 each). A 3000 µs LOW (±800) sync, a 2000 µs
//! HIGH (±500) header, at least 20 short pulses, then a long pulse that is consumed as the Manchester
//! separator, then 88 Manchester bits.
//!
//! **Decoder** (`subghz_protocol_decoder_audi_v1_feed`): Reset waits for the LOW sync; WaitHeader for
//! the HIGH header (anything else back to Reset); Preamble counts shorts and takes a long pulse with
//! a count of 20 or more as the start of data (the Manchester state is reset to Mid1 and the long
//! pulse advances it without producing a bit); Data feeds each short/long pulse to the toolbox
//! Manchester decoder (`level ? ShortHigh/LongHigh : ShortLow/LongLow`, [super::common::manchester_advance]);
//! the first 64 bits are `generic.data`, the next 24 complete the frame. Any other pulse resets.
//!
//! **Frame** (88 bits): `serial:32 | button byte:8 | hop:48`. The button byte's upper nibble is the
//! button (0xE Lock, 0xB Unlock; any other value rejects the frame, as does a zero serial); its
//! lower two bits are counter bits 7..6. The 8-bit counter is `hop[47:42] | btn_lo << 6`. The hop is
//! a weakly scrambled code in FoxFW; no check exists.
//!
//! **KAT fields**: `data` = the first 64 bits (serial, button byte, hop[47:24]), `extra` = the
//! 48-bit hop, `data_count_bit` = 88, `serial` = the 32-bit serial, `button` = the button nibble
//! (0xE / 0xB), `counter` = the 8-bit counter, `crc_valid` false.
//!
//! No deviation from the C. IMPORTS holds no Audi V1 capture (the oracle finds none).
//!
//! **Kept, not registered (false match).** The gate is a 3 ms LOW, a 2 ms HIGH, 20 short pulses and
//! a button nibble of 0xE or 0xB: Fiat V1's HITAG2 air format (Manchester 250/500 µs behind a ~3.5 ms
//! sync and a ~2 ms header) passes the first three and one Fiat frame in eight the button test, so
//! with this decoder registered KAT's own Fiat V1 transmissions (the unit test
//! `renault_v1_arf::tests::hitag2_renault_and_fiat_v1_frames_are_not_claimed`) also decode as
//! "Audi V1" — five times per frame in that test. The FoxFW C decoder does the same on that signal
//! (checked with the host oracle), so the false match is in the reference, not the port. Like
//! `audi.rs` and `hyundai.rs` the module stays in the tree for key files and a future gate.

use super::common::{manchester_advance, ManchesterEvent, ManchesterState};
use super::foxfw_common::duration_diff;
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (see `info.rs`): `SubGhzProtocolFlag_433 | _AM`, dynamic.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Rolling (weak Feistel)",
    make: "Audi",
    category: Category::Vehicle,
    buttons: Some(buttons),
    presets: &[&AM650],
};

/// Button labels of the C (`audi_v1_get_button_name`).
fn buttons(_name: &str, code: u8) -> Option<&'static str> {
    match code {
        BTN_LOCK => Some("Lock"),
        BTN_UNLOCK => Some("Unlock"),
        _ => None,
    }
}

const NAME: &str = "Audi V1";
const TE_SHORT: u32 = 260;
const TE_LONG: u32 = 520;
const TE_DELTA: u32 = 130;
const TE_SYNC: u32 = 3000;
const TE_HEADER: u32 = 2000;
const TE_SYNC_DELTA: u32 = 800;
const TE_HEADER_DELTA: u32 = 500;
const DATA_BITS: u8 = 88;
const PREAMBLE_COUNT: u16 = 20;
const BTN_LOCK: u8 = 0x0E;
const BTN_UNLOCK: u8 = 0x0B;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    WaitHeader,
    Preamble,
    Data,
}

/// Audi V1 decoder (`SubGhzProtocolDecoderAudiV1`).
pub struct AudiV1Decoder {
    step: Step,
    preamble_count: u16,
    state: ManchesterState,
    decode_data: u64,
    decode_count_bit: u8,
    /// `generic.data`: the first 64 bits.
    data: u64,
}

/// A decode of the first 64 bits (`data`) and the 48-bit `hop` (`audi_v1_extract_fields`): serial,
/// button nibble and the 8-bit counter `hop[47:42] | btn_lo << 6`.
fn signal(data: u64, hop: u64) -> DecodedSignal {
    let btn_byte = ((data >> 24) & 0xFF) as u8;
    let counter = (((hop >> 42) & 0x3F) as u8) | ((btn_byte & 0x03) << 6);
    DecodedSignal {
        serial: Some((data >> 32) as u32),
        button: Some((btn_byte >> 4) & 0x0F),
        counter: Some(u16::from(counter)),
        crc_valid: false,
        data,
        data_count_bit: usize::from(DATA_BITS),
        encoder_capable: false,
        extra: Some(hop),
        protocol_display_name: None,
    }
}

impl AudiV1Decoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            preamble_count: 0,
            state: ManchesterState::Mid1,
            decode_data: 0,
            decode_count_bit: 0,
            data: 0,
        }
    }

    /// `audi_v1_decoder_check_complete`: the frame when the button and serial pass.
    fn complete(&mut self) -> Option<DecodedSignal> {
        let remaining = self.decode_data & 0xFF_FFFF;
        let btn_byte = ((self.data >> 24) & 0xFF) as u8;
        let hop = ((self.data & 0xFF_FFFF) << 24) | remaining;
        let serial = (self.data >> 32) as u32;
        let btn_id = (btn_byte >> 4) & 0x0F;
        let out = if (btn_id != BTN_LOCK && btn_id != BTN_UNLOCK) || serial == 0 {
            None
        } else {
            Some(signal(self.data, hop))
        };
        self.decode_data = 0;
        self.decode_count_bit = 0;
        self.step = Step::Reset;
        out
    }
}

impl Default for AudiV1Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for AudiV1Decoder {
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
        self.decode_data = 0;
        self.decode_count_bit = 0;
        self.preamble_count = 0;
        self.state = manchester_advance(self.state, ManchesterEvent::Reset).0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                if !level && duration_diff(duration, TE_SYNC) <= TE_SYNC_DELTA {
                    self.step = Step::WaitHeader;
                }
            }
            Step::WaitHeader => {
                if level && duration_diff(duration, TE_HEADER) <= TE_HEADER_DELTA {
                    self.step = Step::Preamble;
                    self.preamble_count = 0;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::Preamble => {
                if duration_diff(duration, TE_SHORT) <= TE_DELTA {
                    self.preamble_count = self.preamble_count.wrapping_add(1);
                } else if duration_diff(duration, TE_LONG) <= TE_DELTA
                    && self.preamble_count >= PREAMBLE_COUNT
                {
                    self.step = Step::Data;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.data = 0;
                    self.state = manchester_advance(self.state, ManchesterEvent::Reset).0;
                    // the trigger long pulse is a separator: it advances the state, no data bit
                    self.state =
                        manchester_advance(self.state, ManchesterEvent::from_half_bit(level, true)).0;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::Data => {
                let event = if duration_diff(duration, TE_SHORT) <= TE_DELTA {
                    Some(ManchesterEvent::from_half_bit(level, false))
                } else if duration_diff(duration, TE_LONG) <= TE_DELTA {
                    Some(ManchesterEvent::from_half_bit(level, true))
                } else {
                    None
                };
                if let Some(event) = event {
                    let (next, bit) = manchester_advance(self.state, event);
                    self.state = next;
                    if let Some(bit) = bit {
                        self.decode_data = (self.decode_data << 1) | u64::from(bit);
                        self.decode_count_bit = self.decode_count_bit.wrapping_add(1);
                        // at 64 bits the key is saved and the register restarts for the last 24
                        if self.decode_count_bit == 64 {
                            self.data = self.decode_data;
                            self.decode_data = 0;
                        }
                    }
                }
                if self.decode_count_bit >= DATA_BITS {
                    return self.complete();
                }
                if event.is_none() {
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.step = Step::Reset;
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

    /// `subghz_protocol_decoder_audi_v1_deserialize`: `Bit: 88`, the first 64 bits as the key and
    /// the 6-byte `Hop` line; without it the C rebuilds a partial hop from the key's low 24 bits.
    /// No button or serial gate is applied.
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != usize::from(DATA_BITS) {
            return None;
        }
        let hop = match key.hex_bytes_field("Hop") {
            Some(bytes) if bytes.len() >= 6 => {
                bytes[..6].iter().fold(0u64, |acc, &b| (acc << 8) | u64::from(b))
            }
            _ => (key.key & 0xFF_FFFF) << 24,
        };
        Some(signal(key.key, hop))
    }

    /// `subghz_protocol_decoder_audi_v1_serialize`: the generic Key, then the 6-byte `Hop`.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != usize::from(DATA_BITS) {
            return None;
        }
        let mut key = FlipperKey::for_protocol(NAME, usize::from(DATA_BITS), decoded.data);
        key.push_hex("Hop", decoded.extra?, 6);
        Some(key)
    }
}

/// Synthetic captures for the tests.
#[cfg(test)]
pub(super) mod testing {
    use super::*;
    use crate::protocols::foxfw_common::test_support::{bits_of, manchester_pulses};

    /// Pulses of a frame of `serial | btn_byte | hop` (48 bits): 3000 µs LOW sync, 2000 µs HIGH
    /// header, `n_short` 260 µs preamble pulses, the 520 µs separator, the 88 bits as the pulses
    /// that make the toolbox Manchester decoder emit them, a 10 ms LOW.
    pub fn frame(serial: u32, btn_byte: u8, hop: u64, n_short: usize) -> Vec<LevelDuration> {
        let mut out = vec![
            LevelDuration::new(false, TE_SYNC),
            LevelDuration::new(true, TE_HEADER),
        ];
        let mut level = true;
        for _ in 0..n_short {
            level = !level;
            out.push(LevelDuration::new(level, TE_SHORT));
        }
        level = !level;
        out.push(LevelDuration::new(level, TE_LONG));
        let state =
            manchester_advance(ManchesterState::Mid1, ManchesterEvent::from_half_bit(level, true)).0;
        let mut bits = bits_of(u64::from(serial), 32);
        bits.extend(bits_of(u64::from(btn_byte), 8));
        bits.extend(bits_of(hop, 48));
        out.extend(manchester_pulses(state, level, &bits, TE_SHORT, TE_LONG));
        out.push(LevelDuration::new(false, 10_000));
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::foxfw_common::test_support::*;
    use crate::protocols::ProtocolRegistry;

    const SERIAL: u32 = 0x1A2B_3C4D;
    const HOP: u64 = 0x1D2E_3F40_5162;

    /// Made-up frames: (name, pulses, expected (serial, button nibble, counter, hop)).
    #[allow(clippy::type_complexity)]
    fn cases() -> Vec<(&'static str, Vec<LevelDuration>, Option<(u32, u8, u16, u64)>)> {
        let mut bad_sync = testing::frame(SERIAL, 0xE1, HOP, 20);
        bad_sync[0].duration_us = 5_000; // the sync LOW is out of 3000 ± 800
        let mut bad_header = testing::frame(SERIAL, 0xE1, HOP, 20);
        bad_header[1].duration_us = 1_000; // the header HIGH is out of 2000 ± 500
        let mut short_frame = testing::frame(SERIAL, 0xE1, HOP, 20);
        let n = short_frame.len();
        short_frame.drain(n - 15..n - 1); // the 88 bits never complete
        vec![
            ("lock", testing::frame(SERIAL, 0xE1, HOP, 20), Some((SERIAL, 0xE, 0x47, HOP))),
            ("unlock", testing::frame(0x0F1E_2D3C, 0xB2, 0xA1B2_C3D4_E5F6, 24), Some((0x0F1E_2D3C, 0xB, 0xA8, 0xA1B2_C3D4_E5F6))),
            ("bad_button", testing::frame(SERIAL, 0x51, HOP, 20), None),
            ("zero_serial", testing::frame(0, 0xE1, HOP, 20), None),
            ("short_preamble", testing::frame(SERIAL, 0xE1, HOP, 19), None),
            ("bad_sync", bad_sync, None),
            ("bad_header", bad_header, None),
            ("short_frame", short_frame, None),
        ]
    }

    #[test]
    fn key_file_round_trip_and_decode_only() {
        let dec = AudiV1Decoder::new();
        let d = signal(0x1A2B_3C4D_E11D_2E3F, HOP);
        let text = crate::protocols::flipper_key::testing::assert_round_trip(&dec, &d);
        // the lines the C's serialize writes (checked on the oracle's output)
        assert!(text.contains("Protocol: Audi V1\nBit: 88\nKey: 1A 2B 3C 4D E1 1D 2E 3F\nHop: 1D 2E 3F 40 51 62\n"), "{text}");
        // without Hop the C rebuilds a partial hop from the key's low 24 bits
        let mut no_hop = FlipperKey::parse(&text).unwrap();
        no_hop.fields.clear();
        assert_eq!(dec.from_flipper_key(&no_hop).unwrap().extra, Some(0x1D2E_3F00_0000));
        assert!(!dec.supports_encoding() && dec.encode(&d, 1).is_none());
    }

    #[test]
    fn is_not_registered_because_it_false_matches_fiat_v1() {
        let reg = ProtocolRegistry::new();
        assert!(reg.get("Audi V1").is_none());
        // A Fiat V1 transmission (made-up frame, KAT's own encoder) also decodes as Audi V1,
        // as it does in the FoxFW C (host oracle: "Audi V1 ... Key: ED CB A9 87 BB F5 B9 94").
        let raw: [u8; 13] = [0x00, 0x01, 0x12, 0x34, 0x56, 0x78, 0x44, 0x06, 0x76, 0xAE, 0x31, 0xAA, 0x09];
        let fiat = DecodedSignal {
            serial: None,
            button: None,
            counter: None,
            crc_valid: true,
            data: u64::from_be_bytes(raw[..8].try_into().unwrap()),
            data_count_bit: 104,
            encoder_capable: true,
            extra: Some(raw[8..].iter().fold(0u64, |a, &b| (a << 8) | u64::from(b))),
            protocol_display_name: None,
        };
        let mut wire = vec![LevelDuration::new(false, 30_000)];
        wire.extend(reg.get("Fiat V1").unwrap().encode(&fiat, 0x01).unwrap());
        let got = decode_all(&mut AudiV1Decoder::new(), &wire);
        assert!(!got.is_empty(), "the gate no longer passes Fiat V1: register Audi V1 again?");
    }

    #[test]
    fn synthetic_frames_match_the_foxfw_oracle() {
        for (name, wire, want) in cases() {
            dump(&format!("audi_v1_{name}.sub"), 433_920_000, "FuriHalSubGhzPresetOok650Async", &wire);
            let got = decode_all(&mut AudiV1Decoder::new(), &wire);
            eprintln!("CASE {name}: {:?}", got.iter().map(|d| (format!("{:016X}", d.data), d.extra, d.serial, d.button, d.counter)).collect::<Vec<_>>());
            assert_eq!(got.len(), usize::from(want.is_some()), "{name}");
            if let Some((serial, button, counter, hop)) = want {
                let d = &got[0];
                assert_eq!((d.serial, d.button, d.counter, d.extra), (Some(serial), Some(button), Some(counter), Some(hop)), "{name}");
                assert_eq!(d.data_count_bit, 88);
            }
        }
    }
}
