//! Honeywell: 48-bit fixed code of Honeywell wireless doorbells and PIR motion sensors (RCWL300A,
//! RCWL330A, Series 3 / 5 / 9 and Decor chimes), decoder + encoder.
//!
//! Reference: doctor-firmware `lib/subghz/protocols/honeywell_wdb.c`
//! (`SUBGHZ_PROTOCOL_HONEYWELL_WDB_NAME` = "Honeywell"; frame layout from klohner's
//! honeywell-wireless-doorbell), which doctor registers. Not to be confused with "Honeywell Sec"
//! (`honeywell.c`, the 64-bit security sensors: `honeywell.rs`). The copies in Flipper-ARF `lib`
//! (unregistered there), the garage app (`applications/system/subghz_garage/protocols/`, not in
//! its registry), Momentum, RogueMaster and FoxFW (decoder in `lib`, encoder in its SD plugin
//! `enc_honeywell_wdb`) have the same feed and upload code; they differ only in the display text
//! (ARF's `lib` shows the serial alone), the hash function and where the Sensors flag sits.
//!
//! **Air format** (AM, 433.92 / 315 MHz): PWM, te_short 160 µs, te_long 320 µs, te_delta 60 µs. A
//! frame is a LOW of 480 µs (3·te_short, the header), 48 bits MSB first (1 = HIGH 320 + LOW 160,
//! 0 = HIGH 160 + LOW 320) and a HIGH of 480 µs (the trailer); the next frame's header follows the
//! trailer directly.
//!
//! **Decoder** (`subghz_protocol_decoder_honeywell_wdb_feed`): Reset waits for a LOW with
//! |d − 480| < 60; then HIGH/LOW pairs (short HIGH + long LOW = 0, long + short = 1, each within
//! te_delta; anything else → Reset). A HIGH with |d − 480| < 60 where a bit's HIGH is due ends the
//! frame: it is reported when exactly 48 bits were read and bit 0 is the parity of bits 47..1
//! (`subghz_protocol_blocks_get_parity`); back to Reset either way, so the LOW after a trailer
//! opens the next frame and a held button reports every frame.
//!
//! **Fields** (`subghz_protocol_honeywell_wdb_check_remote_controller`): serial = bits 47..28
//! (20 bits); device type = bits 21..20 (2 doorbell, 1 PIR motion), alert = bits 17..16 (0 normal,
//! 1 / 2 high, 3 full volume), secret knock = bit 4, relay = bit 3, low battery = bit 1, parity =
//! bit 0. KAT: `serial` as the C; `button` / `counter` None (the C sets neither); `crc_valid` = the
//! parity holds (always for decodes: the C drops the others).
//!
//! **Encoder** (`subghz_protocol_encoder_honeywell_wdb_get_upload`, `encoder.repeat` = 3): LOW 480,
//! the 48 bits, HIGH 480, the key as stored (the parity is not recomputed). Nothing to switch:
//! `button` is ignored.
//!
//! **Key file**: `Protocol: Honeywell`, `Bit: 48`, `Key` (`subghz_block_generic_serialize`;
//! doctor's serializer also writes `Serial` / `Btn` / `Cnt`, which KAT does not); loading requires
//! `Bit: 48` (`subghz_block_generic_deserialize_check_count_bit`).

use super::f_common::{add_bit, push, with_repeats};
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (see `info.rs`): `SubGhzProtocolFlag_433 | 315 | AM`, `SubGhzProtocolTypeStatic`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Honeywell",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

pub(super) const NAME: &str = "Honeywell";
const TE_SHORT: u32 = 160;
const TE_LONG: u32 = 320;
const TE_DELTA: u32 = 60;
const BIT_COUNT: usize = 48;
/// The header LOW and the trailer HIGH: 3·te_short.
const MARK: u32 = TE_SHORT * 3;
/// `subghz_protocol_encoder_common_alloc(…, 3, 128)`.
const REPEAT: usize = 3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    SaveDuration,
    CheckDuration,
}

/// Honeywell decoder (`SubGhzProtocolDecoderHoneywell_WDB`).
pub struct HoneywellWdbDecoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: u8,
}

/// Bit 0 of the 48-bit frame equals the parity of bits 47..1.
fn parity_ok(data: u64) -> bool {
    let rest = (data >> 1) & ((1 << (BIT_COUNT - 1)) - 1);
    (data & 1) == u64::from(rest.count_ones() & 1)
}

/// A decode (or a loaded key) of `data`, with the C's serial.
fn signal(data: u64) -> DecodedSignal {
    DecodedSignal {
        serial: Some(((data >> 28) & 0xF_FFFF) as u32),
        button: None,
        counter: None,
        crc_valid: parity_ok(data),
        data,
        data_count_bit: BIT_COUNT,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

/// One `get_upload`: header, the 48 bits, trailer.
fn upload(data: u64) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(2 * BIT_COUNT + 2);
    push(&mut out, false, MARK);
    for i in (0..BIT_COUNT).rev() {
        if (data >> i) & 1 == 1 {
            push(&mut out, true, TE_LONG);
            push(&mut out, false, TE_SHORT);
        } else {
            push(&mut out, true, TE_SHORT);
            push(&mut out, false, TE_LONG);
        }
    }
    push(&mut out, true, MARK);
    out
}

impl HoneywellWdbDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
        }
    }
}

impl Default for HoneywellWdbDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for HoneywellWdbDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: BIT_COUNT,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    /// `subghz_protocol_decoder_common_reset`: back to Reset (the register is cleared by the next
    /// header).
    fn reset(&mut self) {
        self.step = Step::Reset;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                if !level && duration.abs_diff(MARK) < TE_DELTA {
                    self.decode_count_bit = 0;
                    self.decode_data = 0;
                    self.step = Step::SaveDuration;
                }
            }
            Step::SaveDuration => {
                if !level {
                    self.step = Step::Reset;
                    return None;
                }
                if duration.abs_diff(MARK) < TE_DELTA {
                    // The trailer: the frame is complete.
                    let out = (usize::from(self.decode_count_bit) == BIT_COUNT
                        && parity_ok(self.decode_data))
                    .then(|| signal(self.decode_data));
                    self.step = Step::Reset;
                    return out;
                }
                self.te_last = duration;
                self.step = Step::CheckDuration;
            }
            Step::CheckDuration => {
                if level {
                    self.step = Step::Reset;
                    return None;
                }
                if self.te_last.abs_diff(TE_SHORT) < TE_DELTA && duration.abs_diff(TE_LONG) < TE_DELTA {
                    add_bit(&mut self.decode_data, &mut self.decode_count_bit, false);
                    self.step = Step::SaveDuration;
                } else if self.te_last.abs_diff(TE_LONG) < TE_DELTA
                    && duration.abs_diff(TE_SHORT) < TE_DELTA
                {
                    add_bit(&mut self.decode_data, &mut self.decode_count_bit, true);
                    self.step = Step::SaveDuration;
                } else {
                    self.step = Step::Reset;
                }
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        (decoded.data_count_bit == BIT_COUNT).then(|| with_repeats(&upload(decoded.data), REPEAT))
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        (key.bits == BIT_COUNT).then(|| signal(key.key))
    }

    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        (decoded.data_count_bit == BIT_COUNT)
            .then(|| FlipperKey::for_protocol(NAME, BIT_COUNT, decoded.data))
    }
}

#[cfg(test)]
mod tests {
    use super::super::f_common::test_support::*;
    use super::*;

    /// (level, µs) of each pulse (`LevelDuration` has no `PartialEq`).
    fn levels(wire: &[LevelDuration]) -> Vec<(bool, u32)> {
        wire.iter().map(|p| (p.level, p.duration_us)).collect()
    }

    /// The Flipper unit-test key file (`honeywell_wdb.sub`, the same in ARF / Momentum / RogueMaster).
    const UNIT_TEST_KEY: &str = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
        Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Honeywell\nBit: 48\nKey: 00 00 0E DB 70 20 00 01\n";

    /// A 48-bit frame with `serial`, the doorbell type and the parity bit filled in.
    fn frame(serial: u32, flags: u64) -> u64 {
        let body = (u64::from(serial & 0xF_FFFF) << 28) | (0x2 << 20) | (flags & 0x0003_001E);
        let parity = u64::from(((body >> 1).count_ones()) & 1);
        body | parity
    }

    fn on_air(data: u64, copies: usize) -> Vec<LevelDuration> {
        let mut wire = vec![LevelDuration::new(true, 1000)];
        for p in with_repeats(&upload(data), copies) {
            push(&mut wire, p.level, p.duration_us);
        }
        push(&mut wire, false, 20_000);
        wire
    }

    #[test]
    fn unit_test_key_fields() {
        let d = HoneywellWdbDecoder::new()
            .from_flipper_key(&FlipperKey::parse(UNIT_TEST_KEY).unwrap())
            .unwrap();
        assert_eq!((d.data, d.data_count_bit), (0x0EDB_7020_0001, 48));
        // The C oracle's round trip: "Sn:0x0EDB7 / DT:Doorbell  Al:Normal / SK:0 R:0 LBat:0".
        assert_eq!(d.serial, Some(0x0EDB7));
        assert_eq!(((d.data >> 20) & 3, (d.data >> 16) & 3), (2, 0));
        assert!(d.crc_valid && d.encoder_capable && d.button.is_none() && d.counter.is_none());
    }

    #[test]
    fn every_copy_of_a_press_decodes() {
        for data in [frame(0x56A37, 0), frame(0x00001, 0x2), frame(0xFFFFF, 0x3_001E), 0x0EDB_7020_0001] {
            let got = feed_all(&mut HoneywellWdbDecoder::new(), &on_air(data, 3));
            assert_eq!(got.len(), 3, "{data:012X}");
            assert!(got.iter().all(|d| d.data == data && d.crc_valid), "{data:012X}");
        }
    }

    #[test]
    fn upload_matches_the_c() {
        // doctor's encoder on the unit-test key: "-480 160 -320 160 -320 160 -320 160 -320 320 -160 …
        // 320 -160 480", 294 pulses (3 × 98).
        let up = HoneywellWdbDecoder::new().encode(&signal(0x0EDB_7020_0001), 0).unwrap();
        assert_eq!(up.len(), 294);
        let p = |i: usize| (up[i].level, up[i].duration_us);
        assert_eq!([p(0), p(1), p(2), p(9), p(10)], [(false, 480), (true, 160), (false, 320), (true, 320), (false, 160)]);
        assert_eq!([p(96), p(97), p(98)], [(false, 160), (true, 480), (false, 480)]);
    }

    #[test]
    fn rejects_bad_parity_length_and_timing() {
        let good = frame(0x12345, 0);
        assert_eq!(feed_all(&mut HoneywellWdbDecoder::new(), &on_air(good, 1)).len(), 1);
        assert!(feed_all(&mut HoneywellWdbDecoder::new(), &on_air(good ^ 1, 1)).is_empty(), "parity");
        let mut short = on_air(good, 1);
        short.drain(2..4);
        assert!(feed_all(&mut HoneywellWdbDecoder::new(), &short).is_empty(), "47 bits");
        // Header / trailer window: 421..=539 µs.
        for (g, ok) in [(420u32, false), (421, true), (539, true), (540, false)] {
            let mut w = on_air(good, 1);
            w[1].duration_us = g;
            assert_eq!(feed_all(&mut HoneywellWdbDecoder::new(), &w).len(), usize::from(ok), "header {g}");
            let mut w = on_air(good, 1);
            let n = w.len();
            w[n - 2].duration_us = g;
            assert_eq!(feed_all(&mut HoneywellWdbDecoder::new(), &w).len(), usize::from(ok), "trailer {g}");
        }
        assert!(HoneywellWdbDecoder::new().encode(&DecodedSignal::new(good, 47), 0).is_none());
    }

    #[test]
    fn key_file_round_trip() {
        let dec = HoneywellWdbDecoder::new();
        let text = super::super::flipper_key::testing::assert_round_trip(&dec, &signal(frame(0xABCDE, 0x10)));
        assert!(text.contains("Protocol: Honeywell\nBit: 48\nKey: 00 00 AB CD E0 20 00 11\n"), "{text}");
        let mut wrong = FlipperKey::parse(&text).unwrap();
        wrong.bits = 64;
        assert!(dec.from_flipper_key(&wrong).is_none());
        // A stored key with a wrong parity loads (crc_valid false) and is sent as it is.
        let bad = signal(frame(0xABCDE, 0x10) ^ 1);
        assert!(!bad.crc_valid);
        assert_eq!(levels(&dec.encode(&bad, 0).unwrap()), levels(&with_repeats(&upload(bad.data), 3)));
    }

    /// `subghz_decoder_honeywell_wdb_test`: the RAW capture through KAT's whole registry gives the
    /// 49 frames every reference reports (doctor, Momentum, RogueMaster, FoxFW, ARF `--registry
    /// all`: Honeywell, key 00 00 56 A3 70 20 00 00), and no other protocol takes a frame.
    #[test]
    fn unit_test_capture_decodes() {
        let Some(path) = reference_file(ARF_UNIT_TESTS, "honeywell_wdb_raw.sub") else {
            return;
        };
        let decodes = registry_decodes(&path);
        assert_eq!(decodes.len(), 49, "{:?}", decodes.iter().map(|(n, _)| n).collect::<Vec<_>>());
        for (name, d) in &decodes {
            assert_eq!(name, NAME);
            assert_eq!((d.data, d.data_count_bit, d.serial), (0x56A3_7020_0000, 48, Some(0x56A37)));
            assert!(d.crc_valid);
        }
    }

    /// `subghz_encoder_honeywell_wdb_test`: the key file loads through the registry, encodes, and
    /// the upload decodes back (the C oracle's round trip: 3 frames).
    #[test]
    fn unit_test_key_encodes_back() {
        let reg = super::super::ProtocolRegistry::new();
        let key = match reference_file(ARF_UNIT_TESTS, "honeywell_wdb.sub") {
            Some(path) => FlipperKey::parse(&std::fs::read_to_string(path).unwrap()).unwrap(),
            None => FlipperKey::parse(UNIT_TEST_KEY).unwrap(),
        };
        let (name, d) = reg.decode_key(&key).expect("honeywell_wdb.sub loads");
        assert_eq!(name, NAME);
        let tx = reg.get(&name).unwrap().encode(&d, 0).unwrap();
        let back = feed_all(&mut HoneywellWdbDecoder::new(), &tx);
        assert_eq!(back.len(), 3);
        assert!(back.iter().all(|x| x.data == d.data));
        let mut reg = super::super::ProtocolRegistry::new();
        let hits = reg.process_signal_stream(&tx, 433_920_000);
        assert!(!hits.is_empty() && hits.iter().all(|(n, x, _)| n == NAME && x.data == d.data), "{hits:?}");
    }

    #[test]
    fn jittered_frames_decode() {
        let mut rng = Rng(0x4E11_D0B5);
        for _ in 0..50 {
            let data = frame(rng.next() as u32, rng.next());
            let wire = jitter(&on_air(data, 2), &mut rng, 50);
            let got = feed_all(&mut HoneywellWdbDecoder::new(), &wire);
            assert!(!got.is_empty(), "{data:012X}");
            assert!(got.iter().all(|d| d.data == data));
        }
    }
}
