//! Tesla Charge: the fixed code that opens a Tesla's charge port.
//!
//! Ported from FoxFW `lib/subghz/protocols/tesla_charge.c` (`TESLA_CHARGE_PROTOCOL_NAME = "Tesla
//! Charge"`; `SubGhzProtocolTypeStatic`, flags `433 | 315 | AM`) and its SD-card encoder plugin
//! `applications/main/subghz/protocols_ext/enc_tesla_charge.c`.
//!
//! **Air format**: OOK Manchester at te = 400 µs (short 400, long 800, te_delta 140), read with
//! Flipper's `manchester_advance` (HIGH pulse = `ShortHigh`/`LongHigh`, LOW = `ShortLow`/`LongLow`,
//! start state Mid1). A pulse of 1000 µs or more, or one outside both windows, is a Reset event.
//!
//! **Frame**: every decoded bit is shifted into a 64-bit register (more than 64 bits clears it and
//! keeps going). On a Reset event the frame is reported only when exactly 50 bits were collected and
//! the register equals the one fixed code `0x00036B5564C725E6` (the preamble decodes to a short run
//! that never matches). The code is the same for every car, so there is no serial, button or
//! counter; `data` carries the 50 bits.
//!
//! **Key file** (`serialize` / `deserialize`): `Protocol: Tesla Charge`, `Bit: 50`, `Key` = the 50
//! bits, then `TE: 400`. Loading takes any 50-bit key with an 8-byte `Key`, as both C deserializers
//! do (only the fixed code ever decodes). FoxFW's decoder writes the constant 400 and ignores the
//! `TE` it loads, but its encoder sends at the file's `TE`, so KAT keeps a `TE` other than 400 in
//! `extra` and writes it back (no line, an unreadable one or `TE: 0` is 400, as for the encoder; the
//! C finds `TE` only after the `Key` line, where FoxFW writes it, KAT anywhere).
//!
//! **Transmit** (`subghz_protocol_encoder_tesla_charge_deserialize` + `_get_upload`): every 50-bit
//! key, 5 copies (the encoder's default `repeat`) of 12 × (HIGH TE, LOW TE), HIGH TE, LOW 3·TE, the
//! 50 bits MSB first in the C's biphase, then LOW 3·TE. A bit's first half is the opposite of the
//! previous bit's second half (HIGH for the first bit, after the sync LOW), its second half is the
//! inverted bit, and equal halves are one 2·TE pulse. Pulses are sent as the C yields them: a final
//! 1 bit's LOW and the closing LOW stay two pulses, and the durations are `uint32_t` products cut to
//! the 30 bits of a Flipper `LevelDuration`. The frame has no button or counter: TX Lock … Panic
//! and "Send next code" all send the saved code, and a transmission changes nothing. A `Repeat` line
//! is not applied (FoxFW deletes it before saving a file; only its CLI and counter brute force set
//! one for a transmission).

use super::common::{ManchesterEvent, ManchesterState};
use super::g2_common::{decoded, near};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`): `SubGhzProtocolFlag_433 | 315 | AM`, static (fixed) code.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Tesla",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "Tesla Charge";
const TE_SHORT: u32 = 400;
const TE_LONG: u32 = 800;
const TE_DELTA: u32 = 140;
const MIN_COUNT_BIT: usize = 50;
const FIX_CODE: u64 = 0x0003_6B55_64C7_25E6;
/// A pulse this long (µs) is an inter-burst gap.
const GAP_MIN: u32 = 1000;
/// `subghz_protocol_encoder_tesla_charge_alloc`: `encoder.repeat = 5`.
const REPEAT: usize = 5;
/// Preamble cells (`for(uint8_t i = 0; i < 12; i++)`).
const PREAMBLE_CELLS: usize = 12;
/// `LevelDuration.duration` is a 30-bit field (`lib/toolbox/level_duration.h`).
const DURATION_BITS: u32 = (1 << 30) - 1;

/// A decode or loaded key: `data` = the Key (the encoder sends its low 50 bits), `te` = the key
/// file's `TE` (400 for a decode), kept in `extra` when it is not 400. Every 50-bit key transmits.
fn signal(data: u64, te: u32) -> DecodedSignal {
    let mut d = decoded(data, MIN_COUNT_BIT);
    d.extra = (te != TE_SHORT).then_some(u64::from(te));
    d.encoder_capable = true;
    d
}

/// `subghz_protocol_encoder_tesla_charge_get_upload`: one copy, at `te` µs (0 = 400). Durations
/// wrap like the C's `uint32_t` products and keep a `LevelDuration`'s 30 bits.
fn upload(data: u64, te: u32) -> Vec<LevelDuration> {
    let te = if te == 0 { TE_SHORT } else { te };
    let pulse = |level: bool, us: u32| LevelDuration::new(level, us & DURATION_BITS);
    let mut up = Vec::with_capacity(2 * PREAMBLE_CELLS + 2 + 2 * MIN_COUNT_BIT + 1);
    for _ in 0..PREAMBLE_CELLS {
        up.extend([pulse(true, te), pulse(false, te)]);
    }
    up.extend([pulse(true, te), pulse(false, te.wrapping_mul(3))]);
    // First half = !prev_second (the line is LOW after the sync), second half = !bit.
    let mut prev_second = false;
    for i in (0..MIN_COUNT_BIT).rev() {
        let (first, second) = (!prev_second, (data >> i) & 1 == 0);
        if first == second {
            up.push(pulse(first, te.wrapping_mul(2)));
        } else {
            up.extend([pulse(first, te), pulse(second, te)]);
        }
        prev_second = second;
    }
    up.push(pulse(false, te.wrapping_mul(3)));
    up
}

/// Tesla Charge decoder (`SubGhzProtocolDecoderTeslaCharge`).
pub struct TeslaChargeDecoder {
    state: ManchesterState,
    data: u64,
    /// `decode_count_bit` (a `uint8_t` in the C).
    count: u8,
}

impl TeslaChargeDecoder {
    pub fn new() -> Self {
        Self { state: ManchesterState::Mid1, data: 0, count: 0 }
    }

    /// `subghz_protocol_tesla_charge_event`: None = Reset.
    fn event(level: bool, duration: u32) -> Option<ManchesterEvent> {
        if duration >= GAP_MIN {
            return None;
        }
        if near(duration, TE_SHORT, TE_DELTA) {
            return Some(ManchesterEvent::from_half_bit(level, false));
        }
        if near(duration, TE_LONG, TE_DELTA) {
            return Some(ManchesterEvent::from_half_bit(level, true));
        }
        None
    }
}

impl Default for TeslaChargeDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for TeslaChargeDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: TE_SHORT, te_long: TE_LONG, te_delta: TE_DELTA, min_count_bit: MIN_COUNT_BIT }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    /// The C's reset clears the register and the Manchester state.
    fn reset(&mut self) {
        self.data = 0;
        self.count = 0;
        self.state = ManchesterState::Mid1;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let Some(event) = Self::event(level, duration) else {
            let hit = self.count as usize == MIN_COUNT_BIT && self.data == FIX_CODE;
            self.reset();
            return hit.then(|| signal(FIX_CODE, TE_SHORT));
        };
        if let Some(bit) = self.state.advance(event) {
            self.data = (self.data << 1) | u64::from(bit);
            self.count = self.count.wrapping_add(1);
            if self.count > 64 {
                self.data = 0;
                self.count = 0;
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    /// FoxFW's `enc_tesla_charge.c`: the code at the key's `TE`, 5 times. There is no button, so
    /// `button` changes nothing.
    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        let te = match decoded.extra {
            None => TE_SHORT,
            Some(te) => u32::try_from(te).ok()?,
        };
        Some(upload(decoded.data, te).repeat(REPEAT))
    }

    /// The C's `deserialize` (decoder and encoder): exactly 50 bits, any Key; `TE` as the encoder
    /// reads it.
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != MIN_COUNT_BIT {
            return None;
        }
        let te = key.u32_field("TE").filter(|&te| te != 0).unwrap_or(TE_SHORT);
        Some(signal(key.key_u64()?, te))
    }

    /// The decoder's `serialize`: the generic block plus `TE` (the loaded key's, else 400).
    fn to_flipper_key(&self, d: &DecodedSignal) -> Option<FlipperKey> {
        let mut key = FlipperKey::for_protocol(NAME, d.data_count_bit, d.data);
        key.push_u32("TE", d.extra.map_or(TE_SHORT, |te| te as u32));
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
            name: "ok",
            wire: &[
            400, -400, 400, -400, 400, -400, 400, -400, 400, -400, 400, -400, -2000, 400, -400, 400,
            -400, 800, -800, 400, -400, 800, -800, 800, -800, 400, -400, 800, -800, 800, -800, 800,
            -800, 800, -800, 800, -800, 400, -400, 800, -400, 400, -800, 800, -400, 400, -800, 400,
            -400, 800, -400, 400, -400, 400, -800, 400, -400, 400, -400, 800, -400, 400, -800, 800,
            -400, 400, -800, 800, -800, 400, -400, 400, -400, 400, -400, 800, -400, 400, -800, 400,
            -400, 800, -400, -8000,
            ],
            frames: &[
                Expect { data: 0x36B5564C725E6, bits: 50, serial: None, button: None, counter: None, extra: None },
            ],
        },
        Case {
            name: "ok_twice",
            wire: &[
            400, -400, 400, -400, 400, -400, 400, -400, 400, -400, 400, -400, -2000, 400, -400, 400,
            -400, 800, -800, 400, -400, 800, -800, 800, -800, 400, -400, 800, -800, 800, -800, 800,
            -800, 800, -800, 800, -800, 400, -400, 800, -400, 400, -800, 800, -400, 400, -800, 400,
            -400, 800, -400, 400, -400, 400, -800, 400, -400, 400, -400, 800, -400, 400, -800, 800,
            -400, 400, -800, 800, -800, 400, -400, 400, -400, 400, -400, 800, -400, 400, -800, 400,
            -400, 800, -400, -8000, 400, -400, 400, -400, 400, -400, 400, -400, 400, -400, 400, -400,
            -2000, 400, -400, 400, -400, 800, -800, 400, -400, 800, -800, 800, -800, 400, -400, 800,
            -800, 800, -800, 800, -800, 800, -800, 800, -800, 400, -400, 800, -400, 400, -800, 800,
            -400, 400, -800, 400, -400, 800, -400, 400, -400, 400, -800, 400, -400, 400, -400, 800,
            -400, 400, -800, 800, -400, 400, -800, 800, -800, 400, -400, 400, -400, 400, -400, 800,
            -400, 400, -800, 400, -400, 800, -400, -8000,
            ],
            frames: &[
                Expect { data: 0x36B5564C725E6, bits: 50, serial: None, button: None, counter: None, extra: None },
                Expect { data: 0x36B5564C725E6, bits: 50, serial: None, button: None, counter: None, extra: None },
            ],
        },
    ];

    /// Bursts the C decoder reports nothing for: a flipped payload bit, 49 and 51 payload bits, and a payload that follows the preamble without the gap that resets the Manchester decoder.
    const REJECT: &[(&str, &[i32])] = &[
        (
            "bad_bit",
            &[
            400, -400, 400, -400, 400, -400, 400, -400, 400, -400, 400, -400, -2000, 400, -400, 400,
            -400, 800, -800, 400, -400, 800, -800, 800, -800, 400, -400, 800, -800, 800, -800, 800,
            -800, 800, -800, 800, -800, 400, -400, 800, -400, 400, -800, 800, -400, 400, -800, 400,
            -400, 800, -400, 400, -400, 400, -800, 400, -400, 400, -400, 800, -400, 400, -800, 800,
            -400, 400, -800, 800, -800, 400, -400, 400, -400, 400, -400, 400, -400, 800, -800, 400,
            -400, 800, -400, -8000,
            ],
        ),
        (
            "short49",
            &[
            400, -400, 400, -400, 400, -400, 400, -400, 400, -400, 400, -400, -2000, 400, -400, 800,
            -800, 400, -400, 800, -800, 800, -800, 400, -400, 800, -800, 800, -800, 800, -800, 800,
            -800, 800, -800, 400, -400, 800, -400, 400, -800, 800, -400, 400, -800, 400, -400, 800,
            -400, 400, -400, 400, -800, 400, -400, 400, -400, 800, -400, 400, -800, 800, -400, 400,
            -800, 800, -800, 400, -400, 400, -400, 400, -400, 800, -400, 400, -800, 400, -400, 800,
            -400, -8000,
            ],
        ),
        (
            "long51",
            &[
            400, -400, 400, -400, 400, -400, 400, -400, 400, -400, 400, -400, -2000, 800, -800, 400,
            -400, 800, -800, 400, -400, 800, -800, 800, -800, 400, -400, 800, -800, 800, -800, 800,
            -800, 800, -800, 800, -800, 400, -400, 800, -400, 400, -800, 800, -400, 400, -800, 400,
            -400, 800, -400, 400, -400, 400, -800, 400, -400, 400, -400, 800, -400, 400, -800, 800,
            -400, 400, -800, 800, -800, 400, -400, 400, -400, 400, -400, 800, -400, 400, -800, 400,
            -400, 800, -400, -8000,
            ],
        ),
        (
            "no_gap_after_preamble",
            &[
            400, -400, 400, -400, 400, -400, 400, -400, 400, -400, 400, -400, 400, -400, 400, -400,
            800, -800, 400, -400, 800, -800, 800, -800, 400, -400, 800, -800, 800, -800, 800, -800,
            800, -800, 800, -800, 400, -400, 800, -400, 400, -800, 800, -400, 400, -800, 400, -400,
            800, -400, 400, -400, 400, -800, 400, -400, 400, -400, 800, -400, 400, -800, 800, -400,
            400, -800, 800, -800, 400, -400, 400, -400, 400, -400, 800, -400, 400, -800, 400, -400,
            800, -400, -8000,
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
            let got = feed_with_resets(&mut TeslaChargeDecoder::new(), &wire(c.wire));
            check(&got, c.frames, c.name);
        }
    }

    #[test]
    fn rejects_what_the_c_decoder_rejects() {
        for (name, w) in REJECT {
            let got = feed_with_resets(&mut TeslaChargeDecoder::new(), &wire(w));
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
        let dec = TeslaChargeDecoder::new();
        for c in ACCEPT {
            for d in feed_with_resets(&mut TeslaChargeDecoder::new(), &wire(c.wire)) {
                assert_key_roundtrip(&dec, &d);
                let text = dec.to_flipper_key(&d).unwrap().to_text();
                assert!(text.contains("Protocol: Tesla Charge\nBit: 50\n"), "{text}");
            }
        }
    }

    /// `subghz_oracle_foxfwenc encode` (FoxFW's `enc_tesla_charge.c` on the x86 host) of the key
    /// file FoxFW saves for the decode (`Bit: 50`, the fixed code, `TE: 400`): `RAW_Data` is 480
    /// pulses, 5 copies of these 96.
    const ORACLE_FIX_COPY: &[i64] = &[
        400, -400, 400, -400, 400, -400, 400, -400, 400, -400, 400, -400, 400, -400, 400, -400, 400, -400,
        400, -400, 400, -400, 400, -400, 400, -1200, 400, -400, 400, -400, 800, -800, 400, -400, 800, -800,
        800, -800, 400, -400, 800, -800, 800, -800, 800, -800, 800, -800, 800, -800, 400, -400, 800, -400,
        400, -800, 800, -400, 400, -800, 400, -400, 800, -400, 400, -400, 400, -800, 400, -400, 400, -400,
        800, -400, 400, -800, 800, -400, 400, -800, 800, -800, 400, -400, 400, -400, 400, -400, 800, -400,
        400, -800, 400, -400, 800, -1200,
    ];

    /// The same for a made-up key file (`Key: FF FD 5A 3C 96 E1 0F 2D`, bits above the 50th set,
    /// `TE: 380`): 495 pulses, 5 copies of these 99. The low 50 bits go out at 380 µs; the final 1
    /// bit's LOW and the closing LOW are two pulses, as yielded.
    const ORACLE_TE380_COPY: &[i64] = &[
        380, -380, 380, -380, 380, -380, 380, -380, 380, -380, 380, -380, 380, -380, 380, -380, 380, -380,
        380, -380, 380, -380, 380, -380, 380, -1140, 760, -760, 760, -760, 760, -760, 380, -380, 760, -760,
        760, -380, 380, -380, 380, -760, 380, -380, 380, -380, 380, -380, 760, -380, 380, -760, 760, -380,
        380, -760, 760, -760, 380, -380, 760, -760, 380, -380, 380, -380, 760, -380, 380, -380, 380, -380,
        380, -760, 760, -380, 380, -380, 380, -380, 380, -760, 380, -380, 380, -380, 380, -380, 760, -380,
        380, -760, 760, -760, 380, -380, 760, -760, -1140,
    ];

    fn signed(tx: &[LevelDuration]) -> Vec<i64> {
        tx.iter().map(|p| if p.level { i64::from(p.duration_us) } else { -i64::from(p.duration_us) }).collect()
    }

    /// A FoxFW key file: the header, then `lines` (Bit, Key, TE ...).
    fn key_file(lines: &str) -> FlipperKey {
        FlipperKey::parse(&format!(
            "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
             Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Tesla Charge\n{lines}"
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

    /// KAT's transmission is FoxFW's encoder output, pulse for pulse: for the decode (TE 400) and
    /// for a loaded key with another Key and TE.
    #[test]
    fn transmits_what_the_c_encoder_sends() {
        let dec = TeslaChargeDecoder::new();
        let d = feed_with_resets(&mut TeslaChargeDecoder::new(), &wire(ACCEPT[0].wire)).remove(0);
        assert!(dec.supports_encoding() && d.encoder_capable && d.extra.is_none());
        let tx = signed(&dec.encode(&d, d.button.unwrap_or(0)).unwrap());
        assert_eq!((tx.len(), tx), (480, ORACLE_FIX_COPY.repeat(REPEAT)));
        let k = dec.from_flipper_key(&key_file("Bit: 50\nKey: FF FD 5A 3C 96 E1 0F 2D\nTE: 380\n")).unwrap();
        assert_eq!((k.data, k.extra, k.encoder_capable), (0xFFFD_5A3C_96E1_0F2D, Some(380), true));
        let tx = signed(&dec.encode(&k, k.button.unwrap_or(0)).unwrap());
        assert_eq!((tx.len(), tx), (495, ORACLE_TE380_COPY.repeat(REPEAT)));
    }

    /// The frame has no button and no counter: TX Lock, Unlock, Trunk and Panic send the capture's
    /// code unchanged, and the decode keeps no button.
    #[test]
    fn every_button_sends_the_same_code() {
        let dec = TeslaChargeDecoder::new();
        let d = signal(FIX_CODE, TE_SHORT);
        let own = signed(&dec.encode(&d, d.button.unwrap_or(0)).unwrap());
        for button in [0x01, 0x02, 0x04, 0x08] {
            assert_eq!(signed(&dec.encode(&d, button).unwrap()), own, "{button:#x}");
        }
        assert_eq!((d.button, d.counter, d.serial), (None, None, None));
    }

    /// Key files load as the C's `deserialize` reads them: any 8-byte Key at exactly 50 bits, `TE`
    /// for the encoder (none, unreadable or 0: 400, which the C's upload uses for 0), written back
    /// on save. Other bit counts and short keys are refused (`deserialize_check_count_bit`,
    /// `flipper_format_read_hex(.., 8)`), so nothing transmits.
    #[test]
    fn key_files_load_as_the_c_encoder_reads_them() {
        let dec = TeslaChargeDecoder::new();
        let te = |lines: &str| dec.from_flipper_key(&key_file(lines)).map(|d| d.extra);
        assert_eq!(te("Bit: 50\nKey: 00 03 6B 55 64 C7 25 E6\nTE: 400\n"), Some(None));
        assert_eq!(te("Bit: 50\nKey: 00 03 6B 55 64 C7 25 E6\n"), Some(None));
        assert_eq!(te("Bit: 50\nKey: 00 03 6B 55 64 C7 25 E6\nTE: 0\n"), Some(None));
        assert_eq!(te("Bit: 50\nKey: 00 03 6B 55 64 C7 25 E6\nTE: abc\n"), Some(None));
        assert_eq!(te("Bit: 50\nKey: 00 00 00 00 00 00 00 01\nTE: 412\n"), Some(Some(412)));
        for refused in ["Bit: 49\nKey: 00 03 6B 55 64 C7 25 E6\n", "Bit: 51\nKey: 00 03 6B 55 64 C7 25 E6\n",
            "Bit: 64\nKey: 00 03 6B 55 64 C7 25 E6\n", "Bit: 50\nKey: 6B 55 64 C7 25 E6\n"]
        {
            assert_eq!(te(refused), None, "{refused}");
        }
        // A loaded TE survives saving and loading again; a decode's key file says 400.
        let k = dec.from_flipper_key(&key_file("Bit: 50\nKey: 00 03 6B 55 64 C7 25 E6\nTE: 412\n")).unwrap();
        assert_key_roundtrip(&dec, &k);
        assert!(dec.to_flipper_key(&k).unwrap().to_text().ends_with("TE: 412\n"));
        // A decode that is not 50 bits, or a TE that is not a u32, does not encode.
        assert!(dec.encode(&decoded(FIX_CODE, 64), 1).is_none());
        assert!(dec.encode(&DecodedSignal { extra: Some(1 << 32), ..signal(FIX_CODE, TE_SHORT) }, 1).is_none());
        assert_eq!(dec.name(), "Tesla Charge");
    }

    /// The transmission, as the app sends it, decodes back through the whole registry as the code
    /// (5 frames per transmission). A TE far from 400 µs, or another Key, sends what the C sends,
    /// which no decoder reads (FoxFW's own round trip reads nothing either).
    #[test]
    fn transmission_decodes_back() {
        use super::super::ProtocolRegistry;
        let dec = TeslaChargeDecoder::new();
        let d = signal(FIX_CODE, TE_SHORT);
        let got = ProtocolRegistry::new().process_signal_stream(&on_air(&dec.encode(&d, 0x01).unwrap()), 433_920_000);
        assert_eq!(got.len(), 2 * REPEAT);
        assert!(got.iter().all(|(n, x, _)| n == NAME && x == &d), "{got:?}");
        let other = signal(0xFFFD_5A3C_96E1_0F2D, 380);
        let got = ProtocolRegistry::new().process_signal_stream(&on_air(&dec.encode(&other, 0).unwrap()), 433_920_000);
        assert!(got.is_empty(), "{got:?}");
    }

    /// Generator for the oracle comparison: `KAT_TESLA_CHARGE_ENCODE=<file listing key files>`
    /// prints KAT's transmission of each (`KATENC <file> | <signed pulses>`, `(none)` when it does
    /// not load or encode), to diff with `subghz_oracle_foxfwenc encode <file>`.
    #[test]
    #[ignore = "oracle harness"]
    fn encode_key_files() {
        let Ok(list) = std::env::var("KAT_TESLA_CHARGE_ENCODE") else { return };
        let dec = TeslaChargeDecoder::new();
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
