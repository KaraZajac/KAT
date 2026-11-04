//! Helpers shared by the G3 gate/garage/shutter protocol ports (Prastel, Phoenix V2, Hay21,
//! Revers RB2, Roger, Allstar Firefly, Telcoma/Cardin EDGE, Urmet, Power Smart, Dooya).
//!
//! Each one is a line-for-line port of a Flipper SubGhz block helper the C files use:
//! - [duration_diff]: `DURATION_DIFF` (`lib/subghz/blocks/math.h`);
//! - [reverse_key]: `subghz_protocol_blocks_reverse_key` (`math.c`);
//! - [ManchesterEncoder]: `lib/toolbox/manchester_encoder.c` (Revers RB2 and Power Smart build
//!   their uploads with it; the decoder side is [super::common::manchester_advance]);
//! - [repeat_upload]: what the encoder's `yield` sends: the upload `repeat` times back to back,
//!   with adjacent same-level durations merged (that is how they go on air);
//! - [UploadBuilder]: an upload being built with adjacent same-level durations merged.

use crate::radio::demodulator::LevelDuration;

/// `DURATION_DIFF(x, y)`: absolute difference of two durations.
#[inline]
pub(super) const fn duration_diff(x: u32, y: u32) -> u32 {
    x.abs_diff(y)
}

/// `subghz_protocol_blocks_reverse_key(key, bit_count)`: the low `bit_count` bits of `key` in
/// reverse order (bit 0 becomes bit `bit_count - 1`).
pub(super) fn reverse_key(key: u64, bit_count: u8) -> u64 {
    let mut reverse = 0u64;
    for i in 0..u32::from(bit_count.min(64)) {
        reverse = (reverse << 1) | ((key >> i) & 1);
    }
    reverse
}

/// An upload under construction: adjacent same-level durations are merged as they are pushed.
#[derive(Debug, Default, Clone)]
pub(super) struct UploadBuilder {
    pulses: Vec<LevelDuration>,
}

impl UploadBuilder {
    pub(super) fn new() -> Self {
        Self::default()
    }

    /// Append `duration_us` at `level`, merging with the previous pulse when the level matches.
    pub(super) fn push(&mut self, level: bool, duration_us: u32) {
        if let Some(last) = self.pulses.last_mut() {
            if last.level == level {
                last.duration_us = last.duration_us.saturating_add(duration_us);
                return;
            }
        }
        self.pulses.push(LevelDuration::new(level, duration_us));
    }

    pub(super) fn extend(&mut self, pulses: &[LevelDuration]) {
        for p in pulses {
            self.push(p.level, p.duration_us);
        }
    }

    pub(super) fn finish(self) -> Vec<LevelDuration> {
        self.pulses
    }
}

/// The encoder's `yield` loop: `upload` sent `repeat` times back to back (adjacent same-level
/// durations merged, also across the copy boundary).
pub(super) fn repeat_upload(upload: &[LevelDuration], repeat: usize) -> Vec<LevelDuration> {
    let mut out = UploadBuilder::new();
    for _ in 0..repeat {
        out.extend(upload);
    }
    out.finish()
}

/// `ManchesterEncoderResult` (manchester_encoder.h): same discriminants as the C.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum ManchesterEncoderResult {
    ShortLow = 0b00,
    LongLow = 0b01,
    LongHigh = 0b10,
    ShortHigh = 0b11,
}

impl ManchesterEncoderResult {
    const fn from_bits(bits: u8) -> Self {
        match bits & 0b11 {
            0b00 => Self::ShortLow,
            0b01 => Self::LongLow,
            0b10 => Self::LongHigh,
            _ => Self::ShortHigh,
        }
    }

    /// The pulse this result stands for, with the protocol's half-bit (`te_short`) and full-bit
    /// (`te_long`) durations (each protocol's `add_duration_to_upload`).
    pub(super) const fn pulse(self, te_short: u32, te_long: u32) -> LevelDuration {
        match self {
            Self::ShortLow => LevelDuration {
                level: false,
                duration_us: te_short,
            },
            Self::LongLow => LevelDuration {
                level: false,
                duration_us: te_long,
            },
            Self::LongHigh => LevelDuration {
                level: true,
                duration_us: te_long,
            },
            Self::ShortHigh => LevelDuration {
                level: true,
                duration_us: te_short,
            },
        }
    }
}

/// Exact port of Flipper's `ManchesterEncoderState` + `manchester_encoder_advance` /
/// `manchester_encoder_finish`. A 1 bit is sent Low half then High half, a 0 bit High then Low.
#[derive(Debug, Clone, Copy, Default)]
pub(super) struct ManchesterEncoder {
    step: u8,
    prev_bit: bool,
}

impl ManchesterEncoder {
    /// `manchester_encoder_reset`.
    pub(super) fn new() -> Self {
        Self::default()
    }

    /// `manchester_encoder_advance`: `(advance, result)`. When `advance` is false the caller
    /// emits `result` and calls again with the same bit (the C callers' pattern).
    pub(super) fn advance(&mut self, curr_bit: bool) -> (bool, ManchesterEncoderResult) {
        match self.step {
            0 => {
                self.prev_bit = curr_bit;
                self.step = 1;
                let r = if curr_bit {
                    ManchesterEncoderResult::ShortLow
                } else {
                    ManchesterEncoderResult::ShortHigh
                };
                (true, r)
            }
            1 => {
                let r = ManchesterEncoderResult::from_bits(
                    (u8::from(self.prev_bit) << 1) + u8::from(curr_bit),
                );
                if curr_bit == self.prev_bit {
                    self.step = 2;
                    (false, r)
                } else {
                    self.prev_bit = curr_bit;
                    (true, r)
                }
            }
            _ => {
                let r = if curr_bit {
                    ManchesterEncoderResult::ShortLow
                } else {
                    ManchesterEncoderResult::ShortHigh
                };
                self.prev_bit = curr_bit;
                self.step = 1;
                (true, r)
            }
        }
    }

    /// The C callers' per-bit loop body: the one or two results this bit produces.
    pub(super) fn encode_bit(&mut self, bit: bool, out: &mut Vec<ManchesterEncoderResult>) {
        let (advanced, r) = self.advance(bit);
        out.push(r);
        if !advanced {
            let (_, r) = self.advance(bit);
            out.push(r);
        }
    }

    /// `manchester_encoder_finish`: the closing half of the last bit.
    pub(super) fn finish(&mut self) -> ManchesterEncoderResult {
        self.step = 0;
        ManchesterEncoderResult::from_bits((u8::from(self.prev_bit) << 1) + u8::from(self.prev_bit))
    }
}

#[cfg(test)]
pub(super) mod test_support {
    //! Test helpers for the G3 modules.

    use super::super::{DecodedSignal, ProtocolDecoder, ProtocolRegistry};
    use crate::radio::demodulator::LevelDuration;
    use std::path::{Path, PathBuf};

    /// Flipper unit-test resources (REFERENCES clone; absent in CI).
    pub(in crate::protocols) const FLIPPER_UNIT_TESTS: &str =
        "REFERENCES/Flipper-ARF/applications/debug/unit_tests/resources/unit_tests/subghz";

    /// A unit-test capture under [FLIPPER_UNIT_TESTS], or None when REFERENCES is not there.
    pub(in crate::protocols) fn unit_test_file(name: &str) -> Option<PathBuf> {
        let p = Path::new(FLIPPER_UNIT_TESTS).join(name);
        if p.exists() {
            Some(p)
        } else {
            eprintln!("Skip: {p:?} not found (run from the crate root with REFERENCES present)");
            None
        }
    }

    /// Feed `pulses` to `decoder` (after a reset) and collect every frame it reports.
    pub(in crate::protocols) fn feed_all(
        decoder: &mut dyn ProtocolDecoder,
        pulses: &[LevelDuration],
    ) -> Vec<DecodedSignal> {
        decoder.reset();
        pulses
            .iter()
            .filter_map(|p| decoder.feed(p.level, p.duration_us))
            .collect()
    }

    /// Pulses from a Flipper `RAW_Data` style list (positive = HIGH, negative = LOW).
    pub(in crate::protocols) fn pulses(raw: &[i32]) -> Vec<LevelDuration> {
        raw.iter()
            .map(|&v| LevelDuration::new(v > 0, v.unsigned_abs()))
            .collect()
    }

    /// A RAW `.sub` decoded the way the app imports it (registry, streaming, both polarities):
    /// every (protocol name, decode).
    pub(in crate::protocols) fn registry_decodes(path: &Path) -> Vec<(String, DecodedSignal)> {
        let (freq, raw) = crate::export::flipper::import_sub_raw(path).expect("import .sub");
        let pairs: Vec<LevelDuration> = raw
            .iter()
            .map(|p| LevelDuration::new(p.level, p.duration_us))
            .collect();
        ProtocolRegistry::new()
            .process_signal_stream(&pairs, freq)
            .into_iter()
            .map(|(n, d, _)| (n, d))
            .collect()
    }

    /// Every decode `decoder` alone makes of a RAW `.sub` (normal polarity, like the reference's
    /// unit test which feeds one decoder).
    pub(in crate::protocols) fn decoder_decodes(
        decoder: &mut dyn ProtocolDecoder,
        path: &Path,
    ) -> Vec<DecodedSignal> {
        let (_, raw) = crate::export::flipper::import_sub_raw(path).expect("import .sub");
        let pairs: Vec<LevelDuration> = raw
            .iter()
            .map(|p| LevelDuration::new(p.level, p.duration_us))
            .collect();
        feed_all(decoder, &pairs)
    }

    /// Adjacent same-level durations merged (how two uploads are compared).
    pub(in crate::protocols) fn merged(pulses: &[LevelDuration]) -> Vec<LevelDuration> {
        let mut b = super::UploadBuilder::new();
        b.extend(pulses);
        b.finish()
    }

    /// A fresh decoder for one of the G3 protocol names.
    fn g3_decoder(name: &str) -> Option<Box<dyn ProtocolDecoder>> {
        use super::super::*;
        Some(match name {
            "Prastel" => Box::new(prastel::PrastelDecoder::new()),
            "Phoenix_V2" => Box::new(phoenix_v2::PhoenixV2Decoder::new()),
            "Hay21" => Box::new(hay21::Hay21Decoder::new()),
            "Revers_RB2" => Box::new(revers_rb2::ReversRb2Decoder::new()),
            "Roger" => Box::new(roger::RogerDecoder::new()),
            "Allstar Firefly" => Box::new(allstar_firefly::AllstarFireflyDecoder::new()),
            "Telcoma/Cardin EDGE" => Box::new(telcoma_edge::TelcomaEdgeDecoder::new()),
            "Urmet" => Box::new(urmet::UrmetDecoder::new()),
            "Power Smart" => Box::new(power_smart::PowerSmartDecoder::new()),
            "Dooya" => Box::new(dooya::DooyaDecoder::new()),
            _ => return None,
        })
    }

    fn fields_line(d: &DecodedSignal) -> String {
        let opt = |v: Option<u64>| v.map(|v| v.to_string()).unwrap_or_else(|| "-".into());
        format!(
            "Bit: {}; Key: {:016X}; Extra: {}; Serial: {}; Btn: {}; Cnt: {}; crc={} tx={}",
            d.data_count_bit,
            d.data,
            d.extra
                .map(|e| format!("{e:X}"))
                .unwrap_or_else(|| "-".into()),
            opt(d.serial.map(u64::from)),
            opt(d.button.map(u64::from)),
            opt(d.counter.map(u64::from)),
            u8::from(d.crc_valid),
            u8::from(d.encoder_capable)
        )
    }

    /// Oracle-parity dump (tooling, `#[ignore]`d): reads `KAT_G3_CMDS` (one command per line:
    /// `decode<TAB>protocol<TAB>file.sub` = that decoder alone over the RAW file, fresh per file
    /// and reset after every frame like the reference receiver; `encode<TAB>protocol<TAB>key.sub`
    /// = the key file's upload with the captured button, merged) and writes `KAT_G3_OUT`.
    #[test]
    #[ignore]
    fn g3_oracle_dump() {
        let (Ok(cmds), Ok(out)) = (std::env::var("KAT_G3_CMDS"), std::env::var("KAT_G3_OUT"))
        else {
            eprintln!("Skip: KAT_G3_CMDS / KAT_G3_OUT not set");
            return;
        };
        let mut text = String::new();
        for line in std::fs::read_to_string(cmds).unwrap().lines() {
            let f: Vec<&str> = line.split('\t').collect();
            if f.len() < 3 {
                continue;
            }
            let (cmd, name, path) = (f[0], f[1], f[2]);
            // Optional 4th column: the native button to encode with (default: the captured one).
            let button_arg: Option<u8> = f.get(3).and_then(|b| b.parse().ok());
            let Some(mut dec) = g3_decoder(name) else {
                text += &format!("{path} | {name} | !NO_DECODER\n");
                continue;
            };
            let Ok(content) = std::fs::read_to_string(path) else {
                text += &format!("{path} | {name} | !READ_ERROR\n");
                continue;
            };
            match cmd {
                "decode" => {
                    let Ok(sub) = crate::export::flipper::parse_sub(&content) else {
                        text += &format!("{path} | {name} | !NOT_RAW\n");
                        continue;
                    };
                    dec.reset();
                    let mut n = 0;
                    for p in &sub.pairs {
                        if let Some(d) = dec.feed(p.level, p.duration_us) {
                            text += &format!("{path} | {name} | {}\n", fields_line(&d));
                            n += 1;
                            dec.reset();
                        }
                    }
                    if n == 0 {
                        text += &format!("{path} | {name} | (none)\n");
                    }
                }
                "encode" => {
                    let key = match super::super::FlipperKey::parse(&content) {
                        Ok(k) => k,
                        Err(e) => {
                            text += &format!("{path} | {name} | !KEY_ERROR {e}\n");
                            continue;
                        }
                    };
                    let Some(d) = dec.from_flipper_key(&key) else {
                        text += &format!("{path} | {name} | !KEY_REJECTED\n");
                        continue;
                    };
                    match dec.encode(&d, button_arg.unwrap_or(d.button.unwrap_or(0))) {
                        Some(p) => {
                            let v: Vec<String> = merged(&p)
                                .iter()
                                .map(|x| {
                                    if x.level {
                                        x.duration_us.to_string()
                                    } else {
                                        format!("-{}", x.duration_us)
                                    }
                                })
                                .collect();
                            text += &format!(
                                "{path} | {name} | key {} | ENC {}\n",
                                fields_line(&d),
                                v.join(" ")
                            );
                        }
                        None => {
                            text +=
                                &format!("{path} | {name} | key {} | ENC (none)\n", fields_line(&d))
                        }
                    }
                }
                _ => {}
            }
        }
        std::fs::write(out, text).unwrap();
    }
}

#[cfg(test)]
mod tests {
    use super::super::common::{ManchesterEvent, ManchesterState};
    use super::*;

    #[test]
    fn reverse_key_matches_the_c_loop() {
        assert_eq!(reverse_key(0b1, 4), 0b1000);
        assert_eq!(reverse_key(0b0110_1001, 8), 0b1001_0110);
        assert_eq!(
            reverse_key(0xFFFF_FFFF_FFFF_FFFF, 64),
            0xFFFF_FFFF_FFFF_FFFF
        );
        assert_eq!(reverse_key(0x1, 64), 0x8000_0000_0000_0000);
        // Bits above bit_count are ignored.
        assert_eq!(reverse_key(0xF0, 4), 0);
        assert_eq!(reverse_key(0xABCD, 0), 0);
    }

    #[test]
    fn duration_diff_is_symmetric() {
        assert_eq!(duration_diff(250, 600), 350);
        assert_eq!(duration_diff(600, 250), 350);
        assert_eq!(duration_diff(7, 7), 0);
    }

    #[test]
    fn upload_builder_merges_same_levels() {
        let mut b = UploadBuilder::new();
        b.push(false, 320);
        b.push(false, 250);
        b.push(true, 250);
        b.push(true, 250);
        b.push(false, 100);
        let v = b.finish();
        assert_eq!(v.len(), 3);
        assert_eq!((v[0].level, v[0].duration_us), (false, 570));
        assert_eq!((v[1].level, v[1].duration_us), (true, 500));
        let r = repeat_upload(
            &[LevelDuration::new(true, 10), LevelDuration::new(false, 20)],
            3,
        );
        assert_eq!(r.len(), 6);
        let r = repeat_upload(
            &[LevelDuration::new(false, 10), LevelDuration::new(false, 20)],
            2,
        );
        assert_eq!((r.len(), r[0].duration_us), (1, 60));
    }

    /// Hand-expanded encoder runs (manchester_encoder.c): 1 = Low half then High half.
    #[test]
    fn manchester_encoder_matches_the_c_results() {
        use ManchesterEncoderResult::*;
        let run = |bits: &[bool]| {
            let mut e = ManchesterEncoder::new();
            let mut out = Vec::new();
            for &b in bits {
                e.encode_bit(b, &mut out);
            }
            out.push(e.finish());
            out
        };
        // 1,1: SL | SH (2nd half of 1st), SL | finish SH.
        assert_eq!(
            run(&[true, true]),
            vec![ShortLow, ShortHigh, ShortLow, ShortHigh]
        );
        // 1,0: SL | LH (High 2nd half + High 1st half) | finish SL.
        assert_eq!(run(&[true, false]), vec![ShortLow, LongHigh, ShortLow]);
        // 0,1: SH | LL | finish SH.
        assert_eq!(run(&[false, true]), vec![ShortHigh, LongLow, ShortHigh]);
        // 0,0: SH | SL, SH | finish SL.
        assert_eq!(
            run(&[false, false]),
            vec![ShortHigh, ShortLow, ShortHigh, ShortLow]
        );
    }

    /// The encoder's half-bits decode back through Flipper's Manchester decoder.
    #[test]
    fn manchester_encoder_round_trips_through_the_decoder() {
        let bits: Vec<bool> = (0..97u32).map(|i| (i * 7 + i / 3) % 5 < 2).collect();
        let mut e = ManchesterEncoder::new();
        let mut results = Vec::new();
        for &b in &bits {
            e.encode_bit(b, &mut results);
        }
        results.push(e.finish());
        // The first result is the first half of bit 0; a decoder in Mid1 needs the High half that
        // ends a virtual 1 bit in front (see common.rs) — start from Start1/Start0 instead.
        let mut state = if bits[0] {
            ManchesterState::Start1
        } else {
            ManchesterState::Start0
        };
        let mut decoded = Vec::new();
        for r in results {
            let p = r.pulse(1, 2);
            let ev = ManchesterEvent::from_half_bit(p.level, p.duration_us == 2);
            if let Some(b) = state.advance(ev) {
                decoded.push(b);
            }
        }
        assert_eq!(decoded, bits);
    }
}
