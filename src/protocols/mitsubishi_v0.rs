//! Mitsubishi V0 decoder + encoder.
//!
//! Two references use the name "Mitsubishi V0" for two different air formats; this registry entry
//! owns the name and serves both:
//!
//! - **96-bit format** (decoded here): Flipper-ARF `lib/subghz/protocols/mitsubishi_v0.c` (2026-09,
//!   enabled; FSM originally from ProtoPirate's old `mitsubishi_v0.c` by @lupettohf). ProtoPirate
//!   removed that file on 2026-09-14 (1287d10 "merge Mitsubishi protocol into Kia V0"), but its Kia
//!   V0 decoder does not read this format, so KAT keeps it for ARF's capability.
//! - **61-bit format**: ProtoPirate's current Mitsubishi V0 is a Kia V0 frame type, decoded by
//!   `kia_v0.rs` (display name "Mitsubishi V0"). Its captures reach this entry through `get()`, and
//!   [`encode`](ProtocolDecoder::encode) hands 61-bit signals back to the Kia V0 Mitsubishi upload.
//!
//! 96-bit air format (315 / 433.92 / 868.35 MHz, FM): PWM pairs, HIGH 250 + LOW 500µs = 1,
//! HIGH 500 + LOW 250µs = 0 (te_delta 100, strict `<`), 96 bits MSB-first = 12 bytes, no checksum.
//! On air the first 8 bytes are inverted and bytes 0..4 XORed with `hi ^ lo` of the counter
//! (`mitsubishi_v0_scramble`); the decoder undoes that with the counter read *after* inversion and
//! *before* the XOR, so byte 4 of the displayed counter is itself scrambled (ARF: "asymmetric
//! scramble"; the displayed counter's two bytes are always equal).
//! Fields after unscrambling: serial = bytes 0..3, counter = bytes 4..5, button = byte 6.
//!
//! Decoder (`subghz_protocol_decoder_mitsubishi_v0_feed`): Reset → (HIGH saved) DataCheck → (LOW)
//! the pair must be short/long or long/short, else the payload is discarded; DataSave waits for the
//! next HIGH. A frame is reported on the LOW of its 96th pair, with no preamble, gap or checksum
//! gate (as in the C) — except that KAT drops 96 identical symbols (all 0x00 or all 0xFF wire
//! bytes): real GM ABO1502T recordings (rtl_433_tests) produced that all-ones "frame" from the GM
//! fob's preamble, and no real frame can have it (serial, counter and button would all be 0). `data` holds wire bytes 0..7 and `extra` bytes 8..11 (KAT's >64-bit
//! convention), i.e. ARF's saved `Raw_Frame`.
//!
//! Encoder (ARF `subghz_protocol_encoder_mitsubishi_v0_get_upload`): replay-only, like ARF — the
//! captured 12-byte frame is re-sent verbatim (no counter bump, no button change: ARF re-sends the
//! captured button for every D-pad direction); captures without the raw tail (`extra` = None, e.g.
//! from KAT's earlier decode-only port) are rebuilt from serial/counter/button exactly as ARF's
//! fallback does (bytes 7..8 = 0, 9 = 0x5A, 10..11 = 0xFF, then scrambled). Upload: 100 + 95 short
//! pairs (HIGH 250, LOW 250), then the 96 data pairs; the frame has no terminator, so its last
//! LOW only ends when the next copy starts — `encode()` therefore returns ARF's full transmission,
//! 5 back-to-back copies (`encoder.repeat = 5`); on air a receiver completes the first 4.
//! (ARF allocates 414 upload entries for these 582 — a heap overflow on the device; the intended
//! upload is unambiguous.)

use super::kia_v0::{self, KiaV0Type};
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`): both formats; ProtoPirate "Mitsubishi V0 [FM476]", Flipper-ARF FM.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Mitsubishi V0",
    decoder: "Mitsubishi V0",
    aliases: &["Mitsu V0"],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::FM,
    encryption: "Rolling Code",
    make: "Mitsubishi",
    category: Category::Vehicle,
    buttons: Some(super::kia_v0::native_buttons),
    presets: &[&FM476],
};

const TE_SHORT: u32 = 250;
const TE_LONG: u32 = 500;
const TE_DELTA: u32 = 100;
/// `min_count_bit_for_found` (only checked when loading a saved file in the C).
const MIN_COUNT_BIT: usize = 80;
const BIT_COUNT: usize = 96;
const DATA_BYTES: usize = 12;
const PREAMBLE_PAIRS: usize = 100;
const SYNC_PAIRS: usize = 95;
const TX_REPEAT: usize = 5;

/// Decoder states (`MitsubishiV0DecoderStep`).
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    DataSave,
    DataCheck,
}

/// Mitsubishi V0 (96-bit) decoder.
pub struct MitsubishiV0Decoder {
    step: DecoderStep,
    te_last: u32,
    bit_count: usize,
    decode_data: [u8; DATA_BYTES],
}

#[inline]
fn is_short(d: u32) -> bool {
    duration_diff!(d, TE_SHORT) < TE_DELTA
}

#[inline]
fn is_long(d: u32) -> bool {
    duration_diff!(d, TE_LONG) < TE_DELTA
}

/// `mitsubishi_v0_scramble`: XOR bytes 0..4 with `hi ^ lo` of the counter, then invert bytes 0..7.
fn scramble(payload: &mut [u8; DATA_BYTES], counter: u16) {
    let [hi, lo] = counter.to_be_bytes();
    let mask = ((hi & 0xAA) | (lo & 0x55)) ^ ((lo & 0xAA) | (hi & 0x55));
    for b in &mut payload[..5] {
        *b ^= mask;
    }
    for b in &mut payload[..8] {
        *b = !*b;
    }
}

/// `mitsubishi_v0_publish_frame`'s unscramble: (serial, counter, button) of an on-air frame.
fn unscramble(raw: &[u8; DATA_BYTES]) -> (u32, u16, u8) {
    let mut p = *raw;
    for b in &mut p[..8] {
        *b = !*b;
    }
    let [hi, lo] = [p[4], p[5]];
    let mask = ((hi & 0xAA) | (lo & 0x55)) ^ ((lo & 0xAA) | (hi & 0x55));
    for b in &mut p[..5] {
        *b ^= mask;
    }
    (u32::from_be_bytes([p[0], p[1], p[2], p[3]]), u16::from_be_bytes([p[4], p[5]]), p[6])
}

/// The 12 wire bytes held in a signal (`data` = bytes 0..7, `extra` = bytes 8..11).
fn raw_frame(data: u64, extra: u64) -> [u8; DATA_BYTES] {
    let mut raw = [0u8; DATA_BYTES];
    raw[..8].copy_from_slice(&data.to_be_bytes());
    raw[8..].copy_from_slice(&(extra as u32).to_be_bytes());
    raw
}

/// ARF's encoder fallback for a frame without its raw bytes: bytes 0..3 serial, 4..5 counter,
/// 6 button, 9 = 0x5A, 10..11 = 0xFF, scrambled with that counter.
fn fallback_frame(serial: u32, counter: u16, button: u8) -> [u8; DATA_BYTES] {
    let mut p = [0u8; DATA_BYTES];
    p[..4].copy_from_slice(&serial.to_be_bytes());
    p[4..6].copy_from_slice(&counter.to_be_bytes());
    p[6] = button;
    p[9] = 0x5A;
    p[10] = 0xFF;
    p[11] = 0xFF;
    scramble(&mut p, counter);
    p
}

/// One ARF upload: 100 + 95 short pairs, then the 96 data pairs (no terminator).
fn upload(payload: &[u8; DATA_BYTES]) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(2 * (PREAMBLE_PAIRS + SYNC_PAIRS + BIT_COUNT));
    for _ in 0..PREAMBLE_PAIRS + SYNC_PAIRS {
        out.push(LevelDuration::new(true, TE_SHORT));
        out.push(LevelDuration::new(false, TE_SHORT));
    }
    for &byte in payload {
        for bit in (0..8).rev() {
            let (high, low) = if (byte >> bit) & 1 != 0 { (TE_SHORT, TE_LONG) } else { (TE_LONG, TE_SHORT) };
            out.push(LevelDuration::new(true, high));
            out.push(LevelDuration::new(false, low));
        }
    }
    out
}

impl MitsubishiV0Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            te_last: 0,
            bit_count: 0,
            decode_data: [0u8; DATA_BYTES],
        }
    }

    /// `mitsubishi_v0_reset_payload`.
    fn reset_payload(&mut self) {
        self.bit_count = 0;
        self.decode_data = [0u8; DATA_BYTES];
    }

    /// `mitsubishi_v0_collect_pair`: short HIGH + long LOW = 1, long HIGH + short LOW = 0.
    fn collect_pair(&mut self, high: u32, low: u32) -> bool {
        let bit = if is_short(high) && is_long(low) {
            true
        } else if is_long(high) && is_short(low) {
            false
        } else {
            return false;
        };
        if self.bit_count < BIT_COUNT {
            if bit {
                self.decode_data[self.bit_count >> 3] |= 1 << (7 - (self.bit_count & 7));
            }
            self.bit_count += 1;
        }
        true
    }

    /// `mitsubishi_v0_publish_frame`.
    fn publish_frame(&self) -> DecodedSignal {
        let raw = self.decode_data;
        let (serial, counter, button) = unscramble(&raw);
        let data = u64::from_be_bytes(raw[..8].try_into().unwrap());
        let extra = u32::from_be_bytes(raw[8..].try_into().unwrap()) as u64;
        DecodedSignal {
            serial: Some(serial),
            button: Some(button),
            counter: Some(counter),
            crc_valid: false,
            data,
            data_count_bit: BIT_COUNT,
            encoder_capable: true,
            extra: Some(extra),
            protocol_display_name: None,
        }
    }
}

impl ProtocolDecoder for MitsubishiV0Decoder {
    fn name(&self) -> &'static str {
        "Mitsubishi V0"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: MIN_COUNT_BIT,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        // ARF: SubGhzProtocolFlag_315 | SubGhzProtocolFlag_433 | SubGhzProtocolFlag_868
        &[315_000_000, 433_920_000, 868_350_000]
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.te_last = 0;
        self.reset_payload();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::Reset => {
                if level {
                    self.te_last = duration;
                    self.step = DecoderStep::DataCheck;
                }
            }
            DecoderStep::DataSave => {
                if level {
                    self.te_last = duration;
                    self.step = DecoderStep::DataCheck;
                } else {
                    self.step = DecoderStep::Reset;
                    self.reset_payload();
                }
            }
            DecoderStep::DataCheck => {
                if !level {
                    if self.collect_pair(self.te_last, duration) {
                        if self.bit_count >= BIT_COUNT {
                            // KAT gate (deviation from ARF, which reports any 96 pairs): 96
                            // identical symbols are a preamble or carrier, not a frame — a GM
                            // ABO1502T fob (rtl_433_tests general_motors) "decodes" as all-ones.
                            let uniform = self.decode_data.iter().all(|&b| b == 0x00)
                                || self.decode_data.iter().all(|&b| b == 0xFF);
                            let result = (!uniform).then(|| self.publish_frame());
                            self.reset_payload();
                            self.step = DecoderStep::Reset;
                            return result;
                        }
                        self.step = DecoderStep::DataSave;
                    } else {
                        self.reset_payload();
                        self.step = DecoderStep::Reset;
                    }
                } else {
                    self.te_last = duration;
                }
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit == kia_v0::BIT_COUNT_KIA {
            // ProtoPirate's Mitsubishi V0 (a Kia V0 frame type).
            return kia_v0::encode_type(decoded, button, KiaV0Type::Mitsubishi);
        }
        if decoded.data_count_bit != BIT_COUNT {
            return None;
        }
        let payload = match decoded.extra {
            Some(extra) => raw_frame(decoded.data, extra),
            None => fallback_frame(decoded.serial?, decoded.counter?, decoded.button.unwrap_or(0)),
        };
        let one = upload(&payload);
        Some(one.iter().cycle().take(one.len() * TX_REPEAT).cloned().collect())
    }

    /// Key files, 96-bit format (Flipper-ARF `subghz_protocol_encoder_mitsubishi_v0_deserialize`):
    /// the frame is `Raw_Frame` (the 12 wire bytes) when present, else ARF's fallback frame built
    /// from `Serial` / `Cnt` / `Btn`; the decode is then the decoder's own for that frame. (ARF's
    /// decoder deserialize checks `Bit` against `min_count_bit_for_found` = 80, which no saved frame
    /// has — its own 96-bit files only load into its encoder — so KAT follows the encoder.) Any
    /// other bit count is ProtoPirate's Mitsubishi V0, a Kia V0 frame type: handed to Kia V0, as
    /// ProtoPirate's catalog loads a "Mitsubishi V0" key.
    fn from_flipper_key(&self, key: &super::FlipperKey) -> Option<DecodedSignal> {
        if key.bits != BIT_COUNT {
            return kia_v0::KiaV0Decoder::new().from_flipper_key(key);
        }
        key.key_u64()?;
        let raw: [u8; DATA_BYTES] = match key.hex_bytes_field("Raw_Frame") {
            Some(raw) => raw.try_into().ok()?,
            None => fallback_frame(
                key.u32_field("Serial").unwrap_or(0),
                key.u32_field("Cnt").unwrap_or(0) as u16,
                key.u32_field("Btn").unwrap_or(0) as u8,
            ),
        };
        let mut decoder = MitsubishiV0Decoder::new();
        decoder.decode_data = raw;
        Some(decoder.publish_frame())
    }

    /// Key files, 96-bit format (Flipper-ARF `subghz_protocol_decoder_mitsubishi_v0_serialize`):
    /// `Protocol: Mitsubishi V0`, `Bit: 96`, `Key` all zero (ARF's decoder never sets
    /// `generic.data`), then `Serial`, `Cnt`, `Btn` (the unscrambled fields) and `Raw_Frame`. A
    /// 61-bit (ProtoPirate) Mitsubishi V0 is written by Kia V0, as ProtoPirate does: `Protocol: Kia
    /// V0` with `Type: 4`.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<super::FlipperKey> {
        if decoded.data_count_bit == kia_v0::BIT_COUNT_KIA {
            let mitsubishi = DecodedSignal {
                protocol_display_name: Some(KiaV0Type::Mitsubishi.display_name().to_string()),
                ..decoded.clone()
            };
            return kia_v0::KiaV0Decoder::new().to_flipper_key(&mitsubishi);
        }
        if decoded.data_count_bit != BIT_COUNT {
            return None;
        }
        let (serial, counter, button, raw) = match decoded.extra {
            Some(extra) => {
                let raw = raw_frame(decoded.data, extra);
                let (serial, counter, button) = unscramble(&raw);
                (serial, counter, button, Some(raw))
            }
            None => (decoded.serial?, decoded.counter?, decoded.button.unwrap_or(0), None),
        };
        let mut key = super::FlipperKey::for_protocol("Mitsubishi V0", BIT_COUNT, 0);
        key.push_u32("Serial", serial);
        key.push_u32("Cnt", counter.into());
        key.push_u32("Btn", button.into());
        if let Some(raw) = raw {
            key.push_hex_bytes("Raw_Frame", &raw);
        }
        Some(key)
    }
}

impl Default for MitsubishiV0Decoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::ProtocolRegistry;

    fn signal(data: u64, extra: Option<u64>, serial: u32, counter: u16, button: u8) -> DecodedSignal {
        DecodedSignal {
            serial: Some(serial),
            button: Some(button),
            counter: Some(counter),
            crc_valid: false,
            data,
            data_count_bit: BIT_COUNT,
            encoder_capable: true,
            extra,
            protocol_display_name: None,
        }
    }

    fn feed_all(wire: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = MitsubishiV0Decoder::new();
        wire.iter().filter_map(|p| dec.feed(p.level, p.duration_us)).collect()
    }

    /// ARF C (`mitsubishi_v0.c` built on the host): raw frames fed to its encoder (`Raw_Frame`) and
    /// read back by its decoder: raw, serial, counter, button.
    #[rustfmt::skip]
    const C_RAW: &[([u8; 12], u32, u16, u8)] = &[
        ([0x26, 0x18, 0x75, 0x2E, 0x41, 0x85, 0x52, 0x0E, 0x03, 0xD7, 0xD7, 0x7C], 0x1D234E15, 0x7A7A, 0xAD),
        ([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], 0xFFFFFFFF, 0xFFFF, 0xFF),
        ([0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA, 0x99, 0x88, 0x77, 0x66, 0x55, 0x44], 0x11003322, 0x5555, 0x66),
        ([0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, 0x0F, 0x1E, 0x2D, 0x3C], 0xCBED8FA1, 0x4343, 0x21),
    ];

    /// serial, counter, button in; raw frame and decoded serial, counter, button out.
    type FieldsVector = (u32, u16, u8, [u8; 12], u32, u16, u8);

    /// ARF C fallback path (no `Raw_Frame`): serial, counter, button in → raw frame and decoded
    /// serial, counter, button out (only counters with equal bytes survive the asymmetric scramble).
    #[rustfmt::skip]
    const C_FIELDS: &[FieldsVector] = &[
        (0x12345678, 0x0001, 0x01, [0xEC, 0xCA, 0xA8, 0x86, 0xFE, 0xFE, 0xFE, 0xFF, 0x00, 0x5A, 0xFF, 0xFF], 0x13355779, 0x0101, 0x01),
        (0xDEADBEEF, 0xABCD, 0x02, [0x47, 0x34, 0x27, 0x76, 0x32, 0x32, 0xFD, 0xFF, 0x00, 0x5A, 0xFF, 0xFF], 0xB8CBD889, 0xCDCD, 0x02),
        (0x00000000, 0xFFFF, 0x08, [0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0xF7, 0xFF, 0x00, 0x5A, 0xFF, 0xFF], 0x00000000, 0xFFFF, 0x08),
        (0x0A0B0C0D, 0x7E7E, 0x44, [0xF5, 0xF4, 0xF3, 0xF2, 0x81, 0x81, 0xBB, 0xFF, 0x00, 0x5A, 0xFF, 0xFF], 0x0A0B0C0D, 0x7E7E, 0x44),
    ];

    #[test]
    fn c_vectors() {
        for &(raw, serial, counter, button) in C_RAW {
            let data = u64::from_be_bytes(raw[..8].try_into().unwrap());
            let extra = u32::from_be_bytes(raw[8..].try_into().unwrap()) as u64;
            let wire = MitsubishiV0Decoder::new()
                .encode(&signal(data, Some(extra), 0, 0, 0), 0x01)
                .unwrap();
            let got = feed_all(&wire);
            if raw.iter().all(|&b| b == raw[0]) && (raw[0] == 0x00 || raw[0] == 0xFF) {
                // The C reports this one; KAT's uniform-symbol gate drops it (see the module doc).
                assert!(got.is_empty());
                continue;
            }
            // On air the last copy's final LOW runs into silence; as a pulse list it is exact.
            assert_eq!(got.len(), TX_REPEAT);
            for d in &got {
                assert_eq!((d.data, d.extra), (data, Some(extra)));
                assert_eq!((d.serial, d.counter, d.button), (Some(serial), Some(counter), Some(button)));
            }
        }
        for &(serial, counter, button, raw, dser, dcnt, dbtn) in C_FIELDS {
            let wire = MitsubishiV0Decoder::new()
                .encode(&signal(0, None, serial, counter, button), 0x08)
                .unwrap();
            let got = feed_all(&wire);
            assert_eq!(got[0].data, u64::from_be_bytes(raw[..8].try_into().unwrap()));
            assert_eq!((got[0].serial, got[0].counter, got[0].button), (Some(dser), Some(dcnt), Some(dbtn)));
        }
    }

    /// Replay is verbatim whatever the requested button, and the serial/counter/button reported
    /// for a frame are the unscramble of its wire bytes.
    #[test]
    fn uniform_symbols_are_not_a_frame() {
        // 120 "1" pairs (HIGH 250 + LOW 500) and 120 "0" pairs: preamble-like, never reported.
        for (h, l) in [(250, 500), (500, 250)] {
            let wire: Vec<LevelDuration> =
                (0..120).flat_map(|_| [LevelDuration::new(true, h), LevelDuration::new(false, l)]).collect();
            assert!(feed_all(&wire).is_empty(), "{h}/{l}");
        }
    }

    #[test]
    fn replay_is_verbatim() {
        let raw = [0x26, 0x18, 0x75, 0x2E, 0x41, 0x85, 0x52, 0x0E, 0x03, 0xD7, 0xD7, 0x7C];
        let (serial, counter, button) = unscramble(&raw);
        let src = signal(
            u64::from_be_bytes(raw[..8].try_into().unwrap()),
            Some(0x03D7_D77C),
            serial,
            counter,
            button,
        );
        let a = MitsubishiV0Decoder::new().encode(&src, 0x01).unwrap();
        let b = MitsubishiV0Decoder::new().encode(&src, 0x08).unwrap();
        let pulses = |v: &[LevelDuration]| v.iter().map(|p| (p.level, p.duration_us)).collect::<Vec<_>>();
        assert_eq!(pulses(&a), pulses(&b));
        assert_eq!(a.len(), TX_REPEAT * 2 * (PREAMBLE_PAIRS + SYNC_PAIRS + BIT_COUNT));
        let got = feed_all(&a);
        assert_eq!(got[0].serial, Some(serial));
        assert_eq!(got[0].counter.map(|c| c >> 8), got[0].counter.map(|c| c & 0xFF), "hi == lo");
    }

    /// The registry reports the 96-bit frame as Mitsubishi V0 at every ARF band, and nothing else
    /// claims it.
    #[test]
    fn registry_reports_96_bit_frames() {
        let raw = [0x26, 0x18, 0x75, 0x2E, 0x41, 0x85, 0x52, 0x0E, 0x03, 0xD7, 0xD7, 0x7C];
        let src = signal(u64::from_be_bytes(raw[..8].try_into().unwrap()), Some(0x03D7_D77C), 0, 0, 0);
        let wire = MitsubishiV0Decoder::new().encode(&src, 0x01).unwrap();
        for f in [315_000_000, 433_920_000, 868_350_000] {
            let mut reg = ProtocolRegistry::new();
            let names: Vec<String> =
                reg.process_signal_stream(&wire, f).into_iter().map(|r| r.0).collect();
            assert_eq!(names, vec!["Mitsubishi V0"; TX_REPEAT], "{f} Hz");
        }
    }

    /// Key files: a 96-bit decode round-trips with ARF's lines (Key all zero, the frame in
    /// Raw_Frame); a file without Raw_Frame rebuilds ARF's fallback frame (C vectors); a 61-bit
    /// (ProtoPirate / Kia V0 type 4) Mitsubishi V0 is written and loaded as Kia V0 does; TX from a
    /// loaded key replays the frame.
    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::{assert_round_trip, key};
        let dec = MitsubishiV0Decoder::new();
        let raw = C_RAW[0].0;
        let src = signal(u64::from_be_bytes(raw[..8].try_into().unwrap()), Some(0x03D7_D77C), 0, 0, 0);
        let wire = dec.encode(&src, 0x01).unwrap();
        let decoded = feed_all(&wire).remove(0);
        let text = assert_round_trip(&dec, &decoded);
        assert!(
            text.ends_with(
                "Protocol: Mitsubishi V0\nBit: 96\nKey: 00 00 00 00 00 00 00 00\nSerial: 488853013\n\
                 Cnt: 31354\nBtn: 173\nRaw_Frame: 26 18 75 2E 41 85 52 0E 03 D7 D7 7C\n"
            ),
            "{text}"
        );
        let loaded = dec.from_flipper_key(&key(&text)).unwrap();
        let pulses = |v: &[LevelDuration]| v.iter().map(|p| (p.level, p.duration_us)).collect::<Vec<_>>();
        assert_eq!(pulses(&dec.encode(&loaded, 0x02).unwrap()), pulses(&wire));

        // No Raw_Frame: ARF's encoder fallback from Serial/Cnt/Btn (the C_FIELDS vectors).
        for &(serial, counter, button, raw, dser, dcnt, dbtn) in C_FIELDS {
            let k = key(&format!(
                "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                 Preset: FuriHalSubGhzPreset2FSKDev476Async\nProtocol: Mitsubishi V0\nBit: 96\n\
                 Key: 00 00 00 00 00 00 00 00\nSerial: {serial}\nCnt: {counter}\nBtn: {button}\n"
            ));
            let d = dec.from_flipper_key(&k).unwrap();
            assert_eq!(d.data, u64::from_be_bytes(raw[..8].try_into().unwrap()));
            assert_eq!((d.serial, d.counter, d.button), (Some(dser), Some(dcnt), Some(dbtn)));
        }

        // ProtoPirate's 61-bit Mitsubishi V0 (Kia V0 type 4): written as Kia V0, loaded back here
        // or through the registry under its display name.
        let frame = 0x1F12_3400_0000_1276u64;
        let kia = kia_v0::KiaV0Decoder::new();
        let mut mitsu = kia.from_flipper_key(&key(
            "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
             Preset: FuriHalSubGhzPreset2FSKDev476Async\nProtocol: Kia V0\nBit: 61\n\
             Key: 1F 12 34 00 00 00 12 76\nType: 4\n",
        ))
        .unwrap();
        assert_eq!((mitsu.data, mitsu.protocol_display_name.as_deref()), (frame, Some("Mitsubishi V0")));
        let text = assert_round_trip(&dec, &mitsu);
        assert!(text.contains("\nProtocol: Kia V0\n") && text.ends_with("Type: 4\n"), "{text}");
        let reg = ProtocolRegistry::new();
        assert_eq!(reg.decode_key(&key(&text)), Some(("Mitsubishi V0".to_string(), mitsu.clone())));
        // A legacy "Mitsubishi V0" 61-bit key (no Type line) is the Mitsubishi type of Kia V0.
        let legacy = text.replace("Protocol: Kia V0", "Protocol: Mitsubishi V0").replace("Type: 4\n", "");
        assert_eq!(reg.decode_key(&key(&legacy)), Some(("Mitsubishi V0".to_string(), mitsu.clone())));
        mitsu.protocol_display_name = None;
        assert!(dec.to_flipper_key(&mitsu).unwrap().to_text().ends_with("Type: 4\n"));
    }

    /// Oracle vectors (`KAT_KEY_VECTORS=<dir> cargo test mitsubishi_v0_key_vectors -- --ignored`).
    #[test]
    #[ignore]
    fn mitsubishi_v0_key_vectors() {
        use super::super::bmw::{key_vector_dir, write_key_vector};
        let dec = MitsubishiV0Decoder::new();
        for (n, i) in [(1, 0), (2, 3)] {
            let raw = C_RAW[i].0;
            let extra = u32::from_be_bytes(raw[8..].try_into().unwrap()) as u64;
            let src = signal(u64::from_be_bytes(raw[..8].try_into().unwrap()), Some(extra), 0, 0, 0);
            let wire = dec.encode(&src, 0x01).unwrap();
            // Two copies: a single copy's last data LOW merges into the silence after it, and the
            // decoder (like the C's) completes a frame only on a 250/500µs LOW.
            let one = &wire[..2 * wire.len() / TX_REPEAT];
            let decoded = feed_all(&wire).remove(0);
            let key = dec.to_flipper_key(&decoded).unwrap();
            write_key_vector(&key_vector_dir(), &format!("mitsubishi_v0_{n}"), 433_920_000, "FuriHalSubGhzPreset2FSKDev476Async", one, &key);
        }
    }
}
