//! BMW CAS3 (FoxFW `bmw_cas3.c`, `BMW_CAS3_PROTOCOL_NAME = "BMW CAS3"`): the AES-encrypted PPM key
//! fob of BMW CAS3 / CAS4 cars, received and replayed (FoxFW's SD-plugin encoder `enc_bmw_cas3.c`).
//!
//! Ported from FoxFW's `lib/subghz/protocols/bmw_cas3.c`; its header says the gate and timing come
//! from FOBworks for Flipper (GPL-3.0, `flipper_oem_wire.c`). The payload is AES-encrypted, so the
//! frame is identified structurally and kept verbatim. It is a different signal from KAT's "BMW"
//! module (`bmw.rs`, Flipper-ARF's 62 µs Manchester frame behind the 0x599E sync): this is PPM with
//! constant ~250 µs marks.
//!
//! **Air format** (AM, 315 / 433 / 868 MHz): constant marks (HIGH 100..=400 µs) each followed by a
//! space (LOW): 300..=700 µs is a 0, 1050..=1950 µs is a 1. A frame follows two consecutive
//! intervals of at least 10 ms (any level, the C does not look at it).
//!
//! **Decoder** (`subghz_protocol_decoder_bmw_cas3_feed`): Sync counts consecutive pulses of 10 ms or
//! more (a shorter one zeroes the count) until there are 2; Mark expects the HIGH mark; Space turns
//! the LOW into a bit, 128 bits at most. A bad mark or space ends the frame: it is reported when it
//! holds 64 or more bits (any count from 64 to 128), and the bad pulse restarts the sync count (1 if
//! it is itself 10 ms or longer, else 0). A frame that reaches 128 bits is reported at once. The C
//! reports inside `bmw_cas3_finish` and sets the count after that callback (whose receiver reset it
//! overrides), so the 75 ms tail of one frame counts as the first sync interval of the next: KAT
//! skips the registry's reset right after its own report, as without that every second
//! back-to-back frame was lost (of the encoder's 3 copies, which the FoxFW decoder reads all of).
//!
//! **KAT fields**: `data` = the first 64 bits (the C's `generic.data`), `extra` = the following bits
//! right-aligned (`generic.data_2`, `bits - 64` of them), `data_count_bit` = the number of bits
//! (64..=128), `serial` = `data >> 32` (the C's display value), `crc_valid` false (nothing to
//! check), `button`/`counter` None, `encoder_capable` true.
//!
//! **Transmit** (FoxFW `enc_bmw_cas3.c`: `subghz_protocol_encoder_bmw_cas3_deserialize`,
//! `bmw_cas3_build_upload`): a replay of the captured bits, which are AES-encrypted ("the encoder
//! replays exactly the captured bits", `bmw_cas3.c`). Its deserialize asks what the decoder's does
//! (Bit 64..=128, Key, `Data` beyond 64 bits), so every decode and loaded key transmits; it also
//! reads `Repeat` (default 3, taken when above 0), which KAT does not carry: it sends the default.
//! One copy: HIGH 25 ms, LOW 20 ms, per bit a 250 µs mark and a 1500 µs (1) or 500 µs (0) space,
//! HIGH 75 ms, LOW 30 ms; the transmission is 3 copies back to back. Nothing changes (the frame
//! has no counter or button in clear), so TX Lock … Panic and Send next code all send the saved
//! frame and the capture keeps it.
//!
//! No deviation from the C. IMPORTS holds no BMW CAS3 capture (the oracle finds none).

use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (see `info.rs`): flags 315 | 433 | 868 | AM, dynamic.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "AES-128",
    make: "BMW",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "BMW CAS3";
const SYNC_MIN_US: u32 = 10_000;
const MARK_MIN_US: u32 = 100;
const MARK_MAX_US: u32 = 400;
const ZERO_MIN_US: u32 = 300;
const ZERO_MAX_US: u32 = 700;
const ONE_MIN_US: u32 = 1050;
const ONE_MAX_US: u32 = 1950;
const MIN_BITS: u8 = 64;
const MAX_BITS: u8 = 128;

/// FoxFW `bmw_cas3.h` / `enc_bmw_cas3.c`: the encoder's pulses (µs) and `BMW_CAS3_REPEAT`.
const ENC_MARK_US: u32 = 250;
const ENC_SPACE_0_US: u32 = 500;
const ENC_SPACE_1_US: u32 = 1500;
const ENC_SYNC_HIGH_US: u32 = 25_000;
const ENC_SYNC_LOW_US: u32 = 20_000;
const ENC_TAIL_HIGH_US: u32 = 75_000;
const ENC_TAIL_LOW_US: u32 = 30_000;
const ENC_REPEAT: usize = 3;

/// `bmw_cas3_build_upload`: HIGH 25 ms, LOW 20 ms, per bit a 250 µs mark and a 1500 µs (1) or
/// 500 µs (0) space, HIGH 75 ms, LOW 30 ms. Bit `i` is `bmw_cas3_bit`: `data` bit `63 - i` for the
/// first 64, then `data_2` from bit `bits - 65` down to 0 (its higher bits are not sent). None
/// outside 64..=128 bits (the C's deserialize refuses them).
fn upload(data: u64, data_2: u64, bits: usize) -> Option<Vec<LevelDuration>> {
    if !(usize::from(MIN_BITS)..=usize::from(MAX_BITS)).contains(&bits) {
        return None;
    }
    let bit = |i: usize| if i < 64 { data >> (63 - i) & 1 == 1 } else { data_2 >> (bits - 1 - i) & 1 == 1 };
    let mut up = Vec::with_capacity(2 * bits + 4);
    up.push(LevelDuration::new(true, ENC_SYNC_HIGH_US));
    up.push(LevelDuration::new(false, ENC_SYNC_LOW_US));
    for i in 0..bits {
        up.push(LevelDuration::new(true, ENC_MARK_US));
        up.push(LevelDuration::new(false, if bit(i) { ENC_SPACE_1_US } else { ENC_SPACE_0_US }));
    }
    up.push(LevelDuration::new(true, ENC_TAIL_HIGH_US));
    up.push(LevelDuration::new(false, ENC_TAIL_LOW_US));
    Some(up)
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Sync,
    Mark,
    Space,
}

/// BMW CAS3 decoder (`SubGhzProtocolDecoderBmwCas3`).
pub struct BmwCas3Decoder {
    step: Step,
    sync_count: u8,
    bit_count: u8,
    bits: [u8; MAX_BITS as usize / 8],
    /// This decoder reported on the last pulse: the registry's reset that follows is the one the
    /// C's callback does inside `bmw_cas3_finish`, which the C code after it overrides.
    just_emitted: bool,
}

/// A decode of `bits` bits: the first 64 and the rest right-aligned; `serial` is the C's
/// `data >> 32` display value. Every one transmits: the encoder's deserialize asks what the
/// decoder's does (64..=128 bits, `Data` beyond 64).
fn signal(data: u64, data_2: u64, bits: usize) -> DecodedSignal {
    DecodedSignal {
        serial: Some((data >> 32) as u32),
        button: None,
        counter: None,
        crc_valid: false,
        data,
        data_count_bit: bits,
        encoder_capable: true,
        extra: Some(data_2),
        protocol_display_name: None,
    }
}

impl BmwCas3Decoder {
    pub fn new() -> Self {
        Self {
            step: Step::Sync,
            sync_count: 0,
            bit_count: 0,
            bits: [0; MAX_BITS as usize / 8],
            just_emitted: false,
        }
    }

    /// `bmw_cas3_finish`: the frame if it has 64 bits or more; the bit count restarts either way.
    fn finish(&mut self) -> Option<DecodedSignal> {
        let mut out = None;
        if self.bit_count >= MIN_BITS {
            let (mut data, mut data_2) = (0u64, 0u64);
            for i in 0..usize::from(self.bit_count) {
                let bit = u64::from((self.bits[i / 8] >> (7 - (i % 8))) & 1);
                if i < 64 {
                    data = (data << 1) | bit;
                } else {
                    data_2 = (data_2 << 1) | bit;
                }
            }
            out = Some(signal(data, data_2, usize::from(self.bit_count)));
        }
        self.bit_count = 0;
        out
    }

    /// `bmw_cas3_resync`.
    fn resync(&mut self, duration: u32) {
        self.step = Step::Sync;
        self.sync_count = u8::from(duration >= SYNC_MIN_US);
    }
}

impl Default for BmwCas3Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for BmwCas3Decoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: 250,
            te_long: 1500,
            te_delta: 400,
            min_count_bit: usize::from(MIN_BITS),
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    /// `subghz_protocol_decoder_bmw_cas3_reset`, skipped right after this decoder's own report: the
    /// C resets inside the callback of `bmw_cas3_finish` and then sets `bit_count`, `step` and
    /// `sync_count` again (`bmw_cas3_resync` keeps a 10 ms pulse that broke the frame as the first
    /// sync interval of the next).
    fn reset(&mut self) {
        if std::mem::take(&mut self.just_emitted) {
            return;
        }
        self.step = Step::Sync;
        self.sync_count = 0;
        self.bit_count = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        self.just_emitted = false;
        match self.step {
            Step::Sync => {
                if duration >= SYNC_MIN_US {
                    self.sync_count = self.sync_count.wrapping_add(1);
                    if self.sync_count >= 2 {
                        self.step = Step::Mark;
                        self.bit_count = 0;
                        self.bits = [0; MAX_BITS as usize / 8];
                    }
                } else {
                    self.sync_count = 0;
                }
            }
            Step::Mark => {
                if level && (MARK_MIN_US..=MARK_MAX_US).contains(&duration) {
                    self.step = Step::Space;
                } else {
                    let out = self.finish();
                    self.resync(duration);
                    self.just_emitted = out.is_some();
                    return out;
                }
            }
            Step::Space => {
                let bit = if !level && (ZERO_MIN_US..=ZERO_MAX_US).contains(&duration) {
                    Some(false)
                } else if !level && (ONE_MIN_US..=ONE_MAX_US).contains(&duration) {
                    Some(true)
                } else {
                    None
                };
                let Some(bit) = bit else {
                    let out = self.finish();
                    self.resync(duration);
                    self.just_emitted = out.is_some();
                    return out;
                };
                if bit {
                    self.bits[usize::from(self.bit_count / 8)] |= 0x80 >> (self.bit_count % 8);
                }
                self.bit_count += 1;
                if self.bit_count >= MAX_BITS {
                    let out = self.finish();
                    self.step = Step::Sync;
                    self.sync_count = 0;
                    self.just_emitted = out.is_some();
                    return out;
                }
                self.step = Step::Mark;
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    /// FoxFW `enc_bmw_cas3.c`: the captured bits as they are, whatever `button` asks (the payload
    /// is AES-encrypted; the C replays it and changes nothing), 3 times back to back.
    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        // Beyond 64 bits the C needs `data_2` (its `Data` line); at 64 it does not read it.
        let data_2 = if decoded.data_count_bit > 64 { decoded.extra? } else { 0 };
        let one = upload(decoded.data, data_2, decoded.data_count_bit)?;
        Some(one.iter().copied().cycle().take(ENC_REPEAT * one.len()).collect())
    }

    /// `subghz_protocol_decoder_bmw_cas3_deserialize` (and the encoder's, which asks the same):
    /// `Bit` from 64 to 128, the key (`flipper_format_read_hex(…, "Key", …, 8)`: 8 values or more,
    /// the first 8 used), and for more than 64 bits the 8-byte `Data` line (the C fails without it).
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if !(usize::from(MIN_BITS)..=usize::from(MAX_BITS)).contains(&key.bits) || key.key_bytes.len() < 8 {
            return None;
        }
        let data_2 = if key.bits > 64 {
            let bytes = key.hex_bytes_field("Data")?;
            if bytes.len() < 8 {
                return None;
            }
            bytes[..8].iter().fold(0u64, |acc, &b| (acc << 8) | u64::from(b))
        } else {
            0
        };
        Some(signal(key.key, data_2, key.bits))
    }

    /// `subghz_protocol_decoder_common_serialize_data_2`: the generic Key, then `Data` (`data_2`).
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if !(usize::from(MIN_BITS)..=usize::from(MAX_BITS)).contains(&decoded.data_count_bit) {
            return None;
        }
        let mut key = FlipperKey::for_protocol(NAME, decoded.data_count_bit, decoded.data);
        key.push_hex("Data", decoded.extra?, 8);
        Some(key)
    }
}

/// Synthetic captures for the tests.
#[cfg(test)]
pub(super) mod testing {
    use super::*;

    /// Pulses of a frame: two 12 ms LOW intervals, then per bit a 250 µs HIGH mark and a 500 µs
    /// (0) or 1500 µs (1) LOW space, then a 12 ms LOW. `bits` are sent as given (64..=128).
    pub fn frame(bits: &[bool]) -> Vec<LevelDuration> {
        let mut out = vec![
            LevelDuration::new(false, 12_000),
            LevelDuration::new(true, 12_000),
        ];
        for &b in bits {
            out.push(LevelDuration::new(true, 250));
            out.push(LevelDuration::new(false, if b { 1500 } else { 500 }));
        }
        out.push(LevelDuration::new(true, 250));
        out.push(LevelDuration::new(false, 12_000));
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::foxfw_common::test_support::*;

    const WORD: u128 = 0x0123_4567_89AB_CDEF_FEDC_BA98_7654_3210;

    /// The first `n` bits of `WORD` (MSB first).
    fn bits(n: usize) -> Vec<bool> {
        (0..n).map(|i| (WORD >> (127 - i)) & 1 == 1).collect()
    }

    /// Made-up frames: (name, pulses, expected (data, data_2, bit count)).
    #[allow(clippy::type_complexity)]
    fn cases() -> Vec<(&'static str, Vec<LevelDuration>, Option<(u64, u64, usize)>)> {
        // A bad mark or space ends the frame: 64 or more bits already in are reported, fewer not.
        let mut bad_mark_70 = testing::frame(&bits(80));
        bad_mark_70[2 + 2 * 70].duration_us = 450; // the 71st mark is out of 100..=400
        let mut bad_space_66 = testing::frame(&bits(80));
        bad_space_66[3 + 2 * 66].duration_us = 900; // the 67th space is neither 0 nor 1
        let mut bad_mark_50 = testing::frame(&bits(80));
        bad_mark_50[2 + 2 * 50].duration_us = 450;
        let mut bad_space_50 = testing::frame(&bits(80));
        bad_space_50[3 + 2 * 50].duration_us = 900;
        let mut one_sync = testing::frame(&bits(80));
        one_sync.remove(0); // only one 10 ms interval before the marks
        vec![
            ("bits64", testing::frame(&bits(64)), Some((0x0123_4567_89AB_CDEF, 0, 64))),
            ("bits100", testing::frame(&bits(100)), Some((0x0123_4567_89AB_CDEF, 0xF_EDCB_A987, 100))),
            ("bits128", testing::frame(&bits(128)), Some((0x0123_4567_89AB_CDEF, 0xFEDC_BA98_7654_3210, 128))),
            ("bits63", testing::frame(&bits(63)), None),
            ("bad_mark_70_bits", bad_mark_70, Some((0x0123_4567_89AB_CDEF, 0x3F, 70))),
            ("bad_space_66_bits", bad_space_66, Some((0x0123_4567_89AB_CDEF, 0x3, 66))),
            ("bad_mark_50_bits", bad_mark_50, None),
            ("bad_space_50_bits", bad_space_50, None),
            ("one_sync", one_sync, None),
        ]
    }

    #[test]
    fn key_file_round_trip() {
        let dec = BmwCas3Decoder::new();
        let d = signal(0x0123_4567_89AB_CDEF, 0x3F, 70);
        let text = crate::protocols::flipper_key::testing::assert_round_trip(&dec, &d);
        // the lines the C's serialize writes (checked on the oracle's output)
        assert!(text.contains("Protocol: BMW CAS3\nBit: 70\nKey: 01 23 45 67 89 AB CD EF\nData: 00 00 00 00 00 00 00 3F\n"), "{text}");
        let d64 = signal(0x0123_4567_89AB_CDEF, 0, 64);
        crate::protocols::flipper_key::testing::assert_round_trip(&dec, &d64);
        // the C needs Data for more than 64 bits, and 64..=128 bits
        let mut no_data = FlipperKey::parse(&text).unwrap();
        no_data.fields.clear();
        assert!(dec.from_flipper_key(&no_data).is_none());
        let mut wrong_bits = FlipperKey::parse(&text).unwrap();
        wrong_bits.bits = 129;
        assert!(dec.from_flipper_key(&wrong_bits).is_none());
        wrong_bits.bits = 63;
        assert!(dec.from_flipper_key(&wrong_bits).is_none());
        // A Key line of fewer than 8 values does not load (`flipper_format_read_hex(…, 8)` fails).
        let short_key = text.replace("Key: 01 23 45 67 89 AB CD EF", "Key: 01 23 45 67 89 AB CD");
        assert!(dec.from_flipper_key(&FlipperKey::parse(&short_key).unwrap()).is_none());
        assert!(d.encoder_capable && d64.encoder_capable);
    }

    /// A 70-bit key file (made-up values; the oracle comparison's `cas3_bits70.sub`).
    const ORACLE_KEY_70: &str = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
        Preset: FuriHalSubGhzPresetOok650Async\nProtocol: BMW CAS3\nBit: 70\nKey: 3F 37 55 BF FE F8 2E 48\n\
        Data: 00 00 00 00 00 00 00 33\n";

    /// `subghz_oracle_foxfwenc encode` of [ORACLE_KEY_70] (FoxFW's `enc_bmw_cas3.c` on the host):
    /// one of the 3 identical copies of its `RAW_Data` (432 values).
    const ORACLE_70: [i64; 144] = [
        25000, -20000, 250, -500, 250, -500, 250, -1500, 250, -1500, 250, -1500, 250, -1500, 250, -1500,
        250, -1500, 250, -500, 250, -500, 250, -1500, 250, -1500, 250, -500, 250, -1500, 250, -1500,
        250, -1500, 250, -500, 250, -1500, 250, -500, 250, -1500, 250, -500, 250, -1500, 250, -500,
        250, -1500, 250, -1500, 250, -500, 250, -1500, 250, -1500, 250, -1500, 250, -1500, 250, -1500,
        250, -1500, 250, -1500, 250, -1500, 250, -1500, 250, -1500, 250, -1500, 250, -1500, 250, -1500,
        250, -500, 250, -1500, 250, -1500, 250, -1500, 250, -1500, 250, -1500, 250, -500, 250, -500,
        250, -500, 250, -500, 250, -500, 250, -1500, 250, -500, 250, -1500, 250, -1500, 250, -1500,
        250, -500, 250, -500, 250, -1500, 250, -500, 250, -500, 250, -1500, 250, -500, 250, -500,
        250, -500, 250, -1500, 250, -1500, 250, -500, 250, -500, 250, -1500, 250, -1500, 75000, -30000,
    ];

    fn ints(pulses: &[LevelDuration]) -> Vec<i64> {
        pulses.iter().map(|p| if p.level { i64::from(p.duration_us) } else { -i64::from(p.duration_us) }).collect()
    }

    /// KAT's transmission is the C's, pulse for pulse (checked on 114 generated key files, see
    /// docs/bmw_cas3.md; this is one of them), and every button sends it: the C replays the frame.
    #[test]
    fn upload_matches_the_reference_encoder() {
        let dec = BmwCas3Decoder::new();
        let d = dec.from_flipper_key(&FlipperKey::parse(ORACLE_KEY_70).unwrap()).unwrap();
        assert!(d.encoder_capable);
        let tx = ints(&dec.encode(&d, 0).unwrap());
        assert_eq!(tx.len(), ENC_REPEAT * ORACLE_70.len());
        assert!(tx.chunks(ORACLE_70.len()).all(|copy| copy == ORACLE_70));
        for button in [0x01, 0x02, 0x04, 0x08] {
            assert_eq!(dec.encode(&d, button).map(|t| ints(&t)), Some(tx.clone()), "{button:#x}");
        }
        // `Data` bits above the frame's are not sent (`bmw_cas3_bit` shifts them out).
        let high = DecodedSignal { extra: Some(d.extra.unwrap() | 0xFFFF_FF00_0000_0000), ..d.clone() };
        assert_eq!(dec.encode(&high, 0).map(|t| ints(&t)), Some(tx));
    }

    /// What the C's deserialize refuses does not transmit: more than 64 bits without `data_2`
    /// (its `Data` line), and bit counts outside 64..=128.
    #[test]
    fn refuses_what_the_c_refuses() {
        let dec = BmwCas3Decoder::new();
        let no_data = DecodedSignal { extra: None, ..signal(0x0123_4567_89AB_CDEF, 0, 80) };
        assert!(dec.encode(&no_data, 0).is_none());
        let sixty_four = DecodedSignal { extra: None, ..signal(0x0123_4567_89AB_CDEF, 0, 64) };
        assert_eq!(dec.encode(&sixty_four, 0).map(|t| t.len()), Some(ENC_REPEAT * (2 * 64 + 4)));
        for bits in [63, 129] {
            assert!(dec.encode(&signal(0x0123_4567_89AB_CDEF, 1, bits), 0).is_none(), "{bits}");
        }
    }

    /// Sent as the app sends it (twice, 20 ms apart, levels merged), every copy reads back in KAT's
    /// registry as the frame sent, as in the FoxFW decoder: a frame that ends on the 75 ms tail
    /// keeps that pulse as the first sync interval of the next (the C resyncs after its callback;
    /// with the registry's reset winning, every second copy used to be lost).
    #[test]
    fn every_copy_decodes_back() {
        use crate::protocols::ProtocolRegistry;
        let dec = BmwCas3Decoder::new();
        for d in [signal(0x3F37_55BF_FEF8_2E48, 0x33, 70), signal(0x0123_4567_89AB_CDEF, 0xFEDC_BA98_7654_3210, 128)] {
            let air = on_air(&dec.encode(&d, 0).unwrap());
            let hits = ProtocolRegistry::new().process_signal_stream(&air, 433_920_000);
            assert_eq!(hits.len(), 2 * ENC_REPEAT, "{}", d.data_count_bit);
            assert!(hits.iter().all(|(n, h, _)| n == NAME && h == &d), "{hits:?}");
        }
    }

    /// Pulses as the oracle prints them (HIGH positive, LOW negative).
    fn signed(pulses: &[LevelDuration]) -> String {
        let v: Vec<String> = pulses
            .iter()
            .map(|p| if p.level { p.duration_us.to_string() } else { format!("-{}", p.duration_us) })
            .collect();
        v.join(" ")
    }

    /// `pulses` twice, each followed by 20 ms of silence, adjacent equal levels merged: what the app
    /// sends and a receiver sees (`app/tx.rs` `on_air`).
    fn on_air(pulses: &[LevelDuration]) -> Vec<LevelDuration> {
        let gap = [LevelDuration::new(false, 20_000)];
        let mut out: Vec<LevelDuration> = Vec::new();
        for p in pulses.iter().chain(&gap).chain(pulses).chain(&gap) {
            match out.last_mut() {
                Some(last) if last.level == p.level => last.duration_us += p.duration_us,
                _ => out.push(*p),
            }
        }
        out
    }

    /// Generator for the oracle comparison: `KAT_BMW_CAS3_ENCODE=<file listing key files>` prints
    /// KAT's transmission for each key file (`KATENC <file> | <signed pulses>` or `(none)`), to diff
    /// with `subghz_oracle_foxfwenc encode <file>`, and KAT's registry decodes of it as the app sends
    /// it (`KATRT <file> | <protocol> | <key-file lines>`, see [on_air]). With `KAT_TX_AIR_DIR` set,
    /// that stream is also written there as `<file name>.air.sub` for the oracle's decoder.
    #[test]
    #[ignore = "oracle harness"]
    fn encode_key_files() {
        use crate::protocols::ProtocolRegistry;
        let Ok(list) = std::env::var("KAT_BMW_CAS3_ENCODE") else { return };
        let air_dir = std::env::var("KAT_TX_AIR_DIR").ok();
        let dec = BmwCas3Decoder::new();
        let mut reg = ProtocolRegistry::new();
        for file in std::fs::read_to_string(list).unwrap().lines().filter(|l| !l.is_empty()) {
            let key = FlipperKey::parse(&std::fs::read_to_string(file).unwrap()).unwrap();
            let loaded = dec.from_flipper_key(&key);
            let out = loaded.as_ref().and_then(|d| dec.encode(d, d.button.unwrap_or(0)));
            if let Some(d) = &loaded {
                assert_eq!(d.encoder_capable, out.is_some(), "{file}: encoder_capable");
            }
            println!("KATENC {file} | {}", out.as_deref().map_or("(none)".to_string(), signed));
            let Some(tx) = out else { continue };
            let air = on_air(&tx);
            for (name, d, _) in reg.process_signal_stream(&air, key.frequency) {
                let lines = reg.get(&name).and_then(|p| p.to_flipper_key(&d)).map_or(String::new(), |k| {
                    let text = k.to_text();
                    let kept: Vec<&str> = text.lines().skip_while(|l| !l.starts_with("Bit:")).collect();
                    kept.join("; ")
                });
                println!("KATRT {file} | {name} | {lines}");
            }
            if let Some(dir) = &air_dir {
                let name = std::path::Path::new(file).file_name().unwrap().to_string_lossy().to_string();
                let mut text = format!(
                    "Filetype: Flipper SubGhz RAW File\nVersion: 1\nFrequency: {}\nPreset: {}\nProtocol: RAW\n",
                    key.frequency,
                    key.preset.as_deref().unwrap_or("FuriHalSubGhzPresetOok650Async")
                );
                let values: Vec<String> = signed(&air).split(' ').map(str::to_string).collect();
                for chunk in values.chunks(512) {
                    text += &format!("RAW_Data: {}\n", chunk.join(" "));
                }
                std::fs::write(std::path::Path::new(dir).join(format!("{name}.air.sub")), text).unwrap();
            }
        }
    }

    #[test]
    fn synthetic_frames_match_the_foxfw_oracle() {
        for (name, wire, want) in cases() {
            dump(&format!("bmw_cas3_{name}.sub"), 433_920_000, "FuriHalSubGhzPresetOok650Async", &wire);
            let got = decode_all(&mut BmwCas3Decoder::new(), &wire);
            eprintln!("CASE {name}: {:?}", got.iter().map(|d| (format!("{:016X}", d.data), d.extra, d.data_count_bit)).collect::<Vec<_>>());
            assert_eq!(got.len(), usize::from(want.is_some()), "{name}");
            if let Some((data, data_2, count)) = want {
                let d = &got[0];
                assert_eq!((d.data, d.extra, d.data_count_bit), (data, Some(data_2), count), "{name}");
                assert_eq!(d.serial, Some((data >> 32) as u32));
            }
        }
    }
}
