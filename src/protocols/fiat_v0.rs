//! Fiat V0 protocol decoder/encoder
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/fiat_v0.c` and `fiat_v0.h`
//! (Flipper-ARF and doctor-firmware carry the same decoder and upload).
//!
//! Preamble: count short pulses (HIGH or LOW, 200±100µs); when preamble_count >= 150 (0x96), an
//! 800µs LOW gap (±100) enters Data. Data: 64 bits (fix/serial = data_low, hop/counter =
//! data_high), then 7 more bits; the frame completes at bit 71 (0x47).
//!
//! **Manchester**: the shared Flipper `manchester_advance` port in `common.rs`, event mapping
//! `level ? ShortLow : ShortHigh` (same for long; fiat_v0.c:426-439), state reset to Mid1
//! (`ManchesterEventReset`) when the preamble starts and at the gap (fiat_v0.c:343, 387, 411).
//! The first HIGH after the gap belongs to an implied leading 1 bit; Mid1 drops it and locks.
//!
//! **Frame on air**: 64 data bits + 6 tail bits, Manchester with 1 = HIGH,LOW. The decoder's
//! 71st bit is not part of the burst: after bit 70 the inter-burst gap is a Manchester reset
//! event that the C ignores once 64 bits are in, and the 71st bit comes from the NEXT burst's
//! preamble (always 1). So the last burst of a press never completes (as in the C). On every
//! real capture the 6 tail bits are `0xxxx1`.
//!
//! **DecodedSignal**: `data` = hop << 32 | fix, `data_count_bit` = 71, `serial` = fix,
//! `counter` = hop[15:0] (hop is the 32-bit rolling/encrypted word), `button` = the 7 decoded
//! tail bits (the C keeps `& 0x3F`; identical on every real capture since the first tail bit is
//! 0 — KAT keeps all 7 so the encoder can resend the 6 on-air tail bits losslessly).
//!
//! **Gating**: there is no checksum; emission is gated on structure (≥150-pulse preamble,
//! 800µs gap, 71 Manchester bits), like the C.
//!
//! **Encoder** (subghz_protocol_encoder_fiat_v0_get_upload): byte-exact replay of the captured
//! frame — hop is a rolling code and the C replays the captured EndByte regardless of the
//! selected button. 3 bursts (25ms apart) of 150 short pairs whose last LOW is the 800µs gap,
//! then the 64 data bits and the 6 tail bits (`endbyte >> 1`) transition-coded as in the C, a
//! final LOW short if the last bit is 1, and a 1600µs LOW end marker.
//! **Deviation from C**: the C sends a first data bit of 1 as one long HIGH, which its own
//! decoder cannot lock onto (the frame decodes shifted by one bit); real fobs send `H s, L s, H s`
//! (IMPORTS/FIAT/Fiat Bravo...: frames with hop bit 31 = 1 start `-754 +201 -218 +145`), i.e. a
//! short HIGH followed by the normal transition from an implied 1. KAT does that; for a first bit
//! of 0 the output is identical to the C.
//!
//! **Key files** (`subghz_protocol_decoder_fiat_v0_serialize` / `_deserialize`): KAT writes
//! ProtoPirate's lines — Protocol "Fiat V0", Bit 71, Key = hop << 32 | fix, then Serial (fix),
//! Btn (EndByte), Cnt (the 32-bit hop), EndByte and `Disp: Fiat V0 - Lock|Unlock|??` (the C's
//! display suffix from the EndByte's low nibble). EndByte is KAT's 7-bit tail (the C masks it to 6
//! bits on decode; equal on every real capture, and its deserialize keeps 7). Loading accepts Bit
//! 64 or 71 (ProtoPirate's encoder and doctor-firmware's decoder; ProtoPirate's own decoder
//! `deserialize` checks for 64 and so rejects the 71-bit files it writes) and takes the tail from
//! EndByte, else `CRC` (doctor-firmware's fallback — `IMPORTS/FIAT/Fiat type 0 N1.sub`, saved as
//! "Fiat type 0", carries only `CRC: 59`), else Btn; Serial/Cnt are not read (fix and hop come
//! from the Key, as in all three C files).

use super::common::{ManchesterEvent, ManchesterState};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Fiat V0",
    decoder: "Fiat V0",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "Fiat",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// The 7 tail bits; ProtoPirate's fiat_v0_display_suffix labels by the low nibble.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code & 0x0F {
        0x4..=0x7 => "Lock",
        0x8..=0xB => "Unlock",
        _ => "Other",
    })
}

const TE_SHORT: u32 = 200;
const TE_LONG: u32 = 400;
const TE_DELTA: u32 = 100;
#[allow(dead_code)]
const MIN_COUNT_BIT: usize = 64;
const FRAME_BITS: u8 = 0x47; // 71
const PREAMBLE_PAIRS: u16 = 150; // 0x96 in reference
const GAP_US: u32 = 800;
const GAP_THRESHOLD: u32 = 800;
const TOTAL_BURSTS: u8 = 3;
const INTER_BURST_GAP: u32 = 25000;
/// Tail bits sent after the 64 data bits (`endbyte >> 1`, bits 5..0).
const TAIL_BITS: u32 = 6;

#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Preamble,
    Data,
}

/// The decode of frame `data` = hop << 32 | fix with the 7 tail bits `tail7`
/// (`fiat_v0_finish_packet`).
fn signal(data: u64, tail7: u8) -> DecodedSignal {
    DecodedSignal {
        serial: Some(data as u32),
        button: Some(tail7),
        counter: Some((data >> 32) as u16),
        crc_valid: false,
        data,
        data_count_bit: FRAME_BITS as usize,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

/// `fiat_v0_display_suffix`: the `Disp` label from the EndByte's low nibble.
fn display_suffix(endbyte: u8) -> &'static str {
    match endbyte & 0x0F {
        0x4..=0x7 => "Lock",
        0x8..=0xB => "Unlock",
        _ => "??",
    }
}

pub struct FiatV0Decoder {
    step: DecoderStep,
    preamble_count: u16,
    manchester_state: ManchesterState,
    data_low: u32,
    data_high: u32,
    bit_count: u8,
    cnt: u32,
    serial: u32,
    btn: u8,
    te_last: u32,
}

impl FiatV0Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            preamble_count: 0,
            manchester_state: ManchesterState::Mid1,
            data_low: 0,
            data_high: 0,
            bit_count: 0,
            cnt: 0,
            serial: 0,
            btn: 0,
            te_last: 0,
        }
    }

    fn manchester_reset(&mut self) {
        self.manchester_state.advance(ManchesterEvent::Reset);
    }

    fn parse_data(&self) -> DecodedSignal {
        signal(((self.cnt as u64) << 32) | (self.serial as u64), self.btn)
    }

    /// fiat_v0_finish_packet: capture the tail bits, build the signal and return to Reset.
    fn finish_packet(&mut self) -> DecodedSignal {
        self.btn = (self.data_low & 0x7F) as u8;
        let result = self.parse_data();
        self.data_low = 0;
        self.data_high = 0;
        self.bit_count = 0;
        self.step = DecoderStep::Reset;
        result
    }

    /// pp_emit with ADD_LEVEL-style merging of equal adjacent levels.
    fn add_level(signal: &mut Vec<LevelDuration>, level: bool, duration: u32) {
        if let Some(last) = signal.last_mut() {
            if last.level == level {
                *last = LevelDuration::new(level, last.duration_us + duration);
                return;
            }
        }
        signal.push(LevelDuration::new(level, duration));
    }

    /// The C's (prev, curr) cases: the second half of `prev` then the first half of `curr`
    /// (1 = HIGH,LOW; 0 = LOW,HIGH on air).
    fn add_transition(signal: &mut Vec<LevelDuration>, prev: bool, curr: bool) {
        match (prev, curr) {
            (false, false) => {
                Self::add_level(signal, true, TE_SHORT);
                Self::add_level(signal, false, TE_SHORT);
            }
            (false, true) => Self::add_level(signal, true, TE_LONG),
            (true, false) => Self::add_level(signal, false, TE_LONG),
            (true, true) => {
                Self::add_level(signal, false, TE_SHORT);
                Self::add_level(signal, true, TE_SHORT);
            }
        }
    }
}

impl ProtocolDecoder for FiatV0Decoder {
    fn name(&self) -> &'static str {
        "Fiat V0"
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
        // C flags: SubGhzProtocolFlag_315 | SubGhzProtocolFlag_433 (ProtoPirate, ARF, doctor).
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.preamble_count = 0;
        self.data_low = 0;
        self.data_high = 0;
        self.bit_count = 0;
        self.cnt = 0;
        self.serial = 0;
        self.btn = 0;
        self.te_last = 0;
        self.manchester_reset();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let diff_short = duration.abs_diff(TE_SHORT);

        match self.step {
            DecoderStep::Reset => {
                if !level {
                    return None;
                }
                if diff_short < TE_DELTA {
                    self.data_low = 0;
                    self.data_high = 0;
                    self.step = DecoderStep::Preamble;
                    self.te_last = duration;
                    self.preamble_count = 0;
                    self.bit_count = 0;
                    self.manchester_reset();
                }
            }

            DecoderStep::Preamble => {
                // fiat_v0.c (FiatV0DecoderStepPreamble): a HIGH must be short; a short LOW counts;
                // any other LOW after >= 150 shorts that is the 800µs gap enters Data, otherwise
                // Reset. (The C's second gap check — still in Preamble after a short LOW — can
                // never match: a short LOW is never within te_delta of 800µs.)
                if level {
                    if diff_short < TE_DELTA {
                        self.preamble_count = self.preamble_count.wrapping_add(1);
                        self.te_last = duration;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                    return None;
                }
                if diff_short < TE_DELTA {
                    self.preamble_count = self.preamble_count.wrapping_add(1);
                    self.te_last = duration;
                } else {
                    if self.preamble_count >= PREAMBLE_PAIRS
                        && duration_diff!(duration, GAP_THRESHOLD) < TE_DELTA
                    {
                        self.step = DecoderStep::Data;
                        self.preamble_count = 0;
                        self.data_low = 0;
                        self.data_high = 0;
                        self.bit_count = 0;
                        self.te_last = duration;
                        self.manchester_reset();
                        return None;
                    }
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::Data => {
                // fiat_v0.c:422-442: level ? ShortLow : ShortHigh (same for long); else Reset event.
                let event = if diff_short < TE_DELTA {
                    Some(ManchesterEvent::from_half_bit(!level, false))
                } else if duration_diff!(duration, TE_LONG) < TE_DELTA {
                    Some(ManchesterEvent::from_half_bit(!level, true))
                } else {
                    None
                };

                if let Some(event) = event {
                    if let Some(data_bit_bool) = self.manchester_state.advance(event) {
                        let new_bit = if data_bit_bool { 1u32 } else { 0u32 };
                        let carry = (self.data_low >> 31) & 1;
                        self.data_low = (self.data_low << 1) | new_bit;
                        self.data_high = (self.data_high << 1) | carry;
                        self.bit_count += 1;

                        if self.bit_count == 0x40 {
                            self.serial = self.data_low;
                            self.cnt = self.data_high;
                            self.data_low = 0;
                            self.data_high = 0;
                        }

                        if self.bit_count == FRAME_BITS {
                            return Some(self.finish_packet());
                        }
                    }
                } else {
                    // Manchester reset event (gap path): complete at exactly 71 bits; below 64 bits
                    // give up; 64..70 bits keep waiting (the 71st bit comes from the next burst).
                    if self.bit_count == FRAME_BITS {
                        return Some(self.finish_packet());
                    } else if self.bit_count < 0x40 {
                        self.step = DecoderStep::Reset;
                    }
                }
                self.te_last = duration;
            }
        }

        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    /// Byte-exact replay of the captured frame (the requested button is ignored, as in the C).
    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        let serial = decoded.serial?;
        let hop = (decoded.data >> 32) as u32;
        // C: data = (hop << 32) | fix; endbyte_to_send = endbyte >> 1 (6 tail bits).
        let data = ((hop as u64) << 32) | (serial as u64);
        let tail = (decoded.button.unwrap_or(0) >> 1) & 0x3F;
        let bits = (0..64)
            .rev()
            .map(|bit| (data >> bit) & 1 == 1)
            .chain((0..TAIL_BITS).rev().map(|bit| (tail >> bit) & 1 == 1));

        let mut signal = Vec::with_capacity(1024);
        for burst in 0..TOTAL_BURSTS {
            if burst > 0 {
                Self::add_level(&mut signal, false, INTER_BURST_GAP);
            }

            // Preamble: alternating short pulses; the last LOW becomes the 800µs gap.
            for _ in 0..PREAMBLE_PAIRS - 1 {
                Self::add_level(&mut signal, true, TE_SHORT);
                Self::add_level(&mut signal, false, TE_SHORT);
            }
            Self::add_level(&mut signal, true, TE_SHORT);
            Self::add_level(&mut signal, false, GAP_US);

            // Short HIGH, then every bit as a transition from an implied leading 1 (see the module
            // doc: for a first bit of 0 this is exactly the C's "H short, L long").
            Self::add_level(&mut signal, true, TE_SHORT);
            let mut prev_bit = true;
            for curr_bit in bits.clone() {
                Self::add_transition(&mut signal, prev_bit, curr_bit);
                prev_bit = curr_bit;
            }

            if prev_bit {
                Self::add_level(&mut signal, false, TE_SHORT);
            }
            // End marker: te_short * 8 LOW (matches C)
            Self::add_level(&mut signal, false, TE_SHORT * 8);
        }

        Some(signal)
    }

    /// ProtoPirate's encoder / doctor-firmware's decoder `deserialize` (see the module doc): Bit
    /// 64 or 71, fix/hop from the Key, the tail from EndByte, CRC or Btn (masked to 7 bits).
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != 64 && key.bits != FRAME_BITS as usize {
            return None;
        }
        let data = key.key_u64()?;
        let tail = ["EndByte", "CRC", "Btn"].iter().find_map(|f| key.u32_field(f)).unwrap_or(0);
        Some(signal(data, (tail & 0x7F) as u8))
    }

    /// `subghz_protocol_decoder_fiat_v0_serialize` (see the module doc).
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        let endbyte = decoded.button.unwrap_or(0) & 0x7F;
        let mut key = FlipperKey::for_protocol("Fiat V0", FRAME_BITS as usize, decoded.data);
        key.push_u32("Serial", decoded.data as u32);
        key.push_u32("Btn", endbyte.into());
        key.push_u32("Cnt", (decoded.data >> 32) as u32);
        key.push_u32("EndByte", endbyte.into());
        key.push("Disp", format!("Fiat V0 - {}", display_suffix(endbyte)));
        Some(key)
    }
}

impl Default for FiatV0Decoder {
    fn default() -> Self {
        Self::new()
    }
}

/// Oracle test vector for the reference C (the `oracle_vectors` tests of the Fiat / Honda /
/// Renault V0 modules; run with `KAT_KEY_VECTORS=<dir> cargo test oracle_vectors -- --ignored`):
/// `<dir>/<name>.sub` is a Flipper RAW file of `upload` sent twice, each copy followed by a
/// 20 000 µs LOW gap (equal adjacent levels merged; HIGH positive, LOW negative, ≤ 512 values per
/// line), and `<dir>/<name>.kat.txt` KAT's key file (same Frequency / Preset) for the first frame
/// `decoder` reports on that stream. Does nothing without `KAT_KEY_VECTORS`.
#[cfg(test)]
pub(super) fn write_oracle_vector(
    name: &str,
    frequency: u32,
    preset: &str,
    upload: &[LevelDuration],
    decoder: &mut dyn ProtocolDecoder,
) {
    use crate::protocols::flipper_key::testing::{vector_dir, write_vector};
    let Some(dir) = vector_dir() else { return };
    let mut stream: Vec<LevelDuration> = Vec::new();
    let gap = [LevelDuration::new(false, 20_000)];
    for p in upload.iter().chain(gap.iter()).cycle().take(2 * (upload.len() + 1)) {
        match stream.last_mut() {
            Some(last) if last.level == p.level => last.duration_us += p.duration_us,
            _ => stream.push(*p),
        }
    }
    decoder.reset();
    let decoded = stream
        .iter()
        .find_map(|p| decoder.feed(p.level, p.duration_us))
        .unwrap_or_else(|| panic!("{name}: the upload does not decode"));
    let key = decoder.to_flipper_key(&decoded).unwrap_or_else(|| panic!("{name}: no key file"));
    write_vector(&dir, name, frequency, preset, upload, &key);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::export::flipper::import_sub_raw;
    use crate::protocols::ProtocolRegistry;
    use std::path::Path;

    const FIX: u32 = 0x5453_6200;

    fn frame(hop: u32, fix: u32, tail7: u8) -> DecodedSignal {
        DecodedSignal {
            serial: Some(fix),
            button: Some(tail7),
            counter: Some(hop as u16),
            crc_valid: false,
            data: ((hop as u64) << 32) | fix as u64,
            data_count_bit: FRAME_BITS as usize,
            encoder_capable: true,
            extra: None,
            protocol_display_name: None,
        }
    }

    fn feed_all(signal: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = FiatV0Decoder::new();
        signal.iter().filter_map(|p| dec.feed(p.level, p.duration_us)).collect()
    }

    /// Encode → decode for real frames (IMPORTS/FIAT/Fiat Bravo...), one with hop bit 31 = 0 and one
    /// with hop bit 31 = 1. Bursts 1 and 2 decode; burst 3 has no following preamble to supply the
    /// 71st bit (C behaviour).
    #[test]
    fn encode_decode_roundtrip() {
        for (hop, tail7) in [(0x0DE2_1BFEu32, 0x13u8), (0xFEC0_E84D, 0x1B), (0xA03B_F868, 0x3B)] {
            let upload = FiatV0Decoder::new().encode(&frame(hop, FIX, tail7), 0x02).unwrap();
            let out = feed_all(&upload);
            assert_eq!(out.len(), (TOTAL_BURSTS - 1) as usize, "hop {hop:08X}");
            for d in &out {
                assert_eq!(d.data, ((hop as u64) << 32) | FIX as u64, "hop {hop:08X}");
                assert_eq!(d.serial, Some(FIX));
                assert_eq!(d.button, Some(tail7), "tail bits replayed");
                assert_eq!(d.data_count_bit, 71);
            }
        }
    }

    /// The upload starts like the C's after the 800µs gap: `H s, L long` for a first bit of 0 and
    /// (corrected, as on real captures) `H s, L s, H s` for a first bit of 1.
    #[test]
    fn upload_frame_start_matches_reference() {
        let gap_at = |up: &[LevelDuration]| up.iter().position(|p| !p.level && p.duration_us == GAP_US).unwrap();
        let up0 = FiatV0Decoder::new().encode(&frame(0x0DE2_1BFE, FIX, 0x13), 0).unwrap();
        let i = gap_at(&up0);
        assert_eq!(i, 2 * PREAMBLE_PAIRS as usize - 1, "150 HIGH shorts, the last LOW is the gap");
        let start: Vec<(bool, u32)> = up0[i + 1..i + 3].iter().map(|p| (p.level, p.duration_us)).collect();
        assert_eq!(start, [(true, TE_SHORT), (false, TE_LONG)]);
        let up1 = FiatV0Decoder::new().encode(&frame(0xFEC0_E84D, FIX, 0x1B), 0).unwrap();
        let i = gap_at(&up1);
        let start: Vec<(bool, u32)> = up1[i + 1..i + 4].iter().map(|p| (p.level, p.duration_us)).collect();
        assert_eq!(start, [(true, TE_SHORT), (false, TE_SHORT), (true, TE_SHORT)]);
        // Ends with the 1600µs end marker (merged with a final LOW short after a final 1).
        assert_eq!(up1.last().map(|p| (p.level, p.duration_us)), Some((false, TE_SHORT * 9)));
    }

    /// Structure gate: a preamble shorter than 150 pulses or a wrong gap must not decode.
    #[test]
    fn gap_window_matches_reference() {
        // fiat_v0.c enters Data on a LOW within te_delta of 800µs (701..=899); 799µs used to
        // underflow here (debug panic / missed frame in release).
        let good = FiatV0Decoder::new().encode(&frame(0x0DE2_1BFE, FIX, 0x13), 0).unwrap();
        for (gap, decodes) in [(701u32, true), (799, true), (800, true), (899, true), (700, false), (900, false)] {
            let mut wave = good.clone();
            for p in wave.iter_mut().filter(|p| !p.level && p.duration_us == GAP_US) {
                *p = LevelDuration::new(false, gap);
            }
            assert_eq!(!feed_all(&wave).is_empty(), decodes, "gap {gap}µs");
        }
    }

    #[test]
    fn rejects_bad_structure() {
        let good = FiatV0Decoder::new().encode(&frame(0x0DE2_1BFE, FIX, 0x13), 0).unwrap();
        assert!(!feed_all(&good).is_empty());

        let mut bad_gap = good.clone();
        for p in bad_gap.iter_mut().filter(|p| !p.level && p.duration_us == GAP_US) {
            *p = LevelDuration::new(false, 1000);
        }
        assert!(feed_all(&bad_gap).is_empty(), "1000µs gap");

        // Keep only the last 70 preamble pairs of every burst: 138 counted pulses < 150.
        let mut short_pre = Vec::new();
        let mut skip = 160;
        for p in &good {
            if skip > 0 {
                skip -= 1;
                continue;
            }
            short_pre.push(*p);
            if !p.level && p.duration_us >= INTER_BURST_GAP {
                skip = 160;
            }
        }
        assert!(feed_all(&short_pre).is_empty(), "short preamble");
    }

    /// Key files: ProtoPirate's lines round-trip; the IMPORTS key file (saved as "Fiat type 0",
    /// tail in `CRC`) loads as exactly the stream decode of the same frame in the Bravo RAW
    /// capture; a loaded key transmits a frame that decodes back.
    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::{assert_round_trip, key};
        let dec = FiatV0Decoder::new();
        for (hop, tail7) in [(0x0DE2_1BFEu32, 0x13u8), (0xFEC0_E84D, 0x1B), (0xA03B_F868, 0x3B)] {
            let upload = dec.encode(&frame(hop, FIX, tail7), 0x02).unwrap();
            for d in feed_all(&upload) {
                assert_round_trip(&dec, &d);
            }
        }
        let d = feed_all(&dec.encode(&frame(0xA03B_F868, FIX, 0x3B), 0).unwrap()).remove(0);
        assert!(assert_round_trip(&dec, &d).ends_with(
            "Protocol: Fiat V0\nBit: 71\nKey: A0 3B F8 68 54 53 62 00\nSerial: 1414750720\nBtn: 59\n\
             Cnt: 2688284776\nEndByte: 59\nDisp: Fiat V0 - Unlock\n"
        ));

        // IMPORTS/FIAT/Fiat type 0 N1.sub, inlined (CRLF-free copy of the file's lines).
        let file = key(
            "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433889000\n\
             Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Fiat type 0\nBit: 71\n\
             Key: A0 3B F8 68 54 53 62 00\nCRC: 59\n",
        );
        let reg = ProtocolRegistry::new();
        let (name, loaded) = reg.decode_key(&file).expect("Fiat type 0 loads as Fiat V0");
        assert_eq!(name, "Fiat V0");
        assert_eq!(loaded, frame(0xA03B_F868, FIX, 0x3B));
        let path = Path::new("IMPORTS/FIAT/Fiat type 0 N1.sub");
        if path.exists() {
            let on_disk = key(&std::fs::read_to_string(path).unwrap());
            assert_eq!(reg.decode_key(&on_disk), Some((name.clone(), loaded.clone())));
        }
        let raw = Path::new("IMPORTS/FIAT/Fiat Bravo 2010 S.Croce 433.88 am650.sub");
        if raw.exists() {
            let (freq, pairs) = import_sub_raw(raw).unwrap();
            let pairs: Vec<LevelDuration> =
                pairs.iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect();
            let same = ProtocolRegistry::new()
                .process_signal_stream(&pairs, freq)
                .into_iter()
                .find(|(n, d, _)| n == "Fiat V0" && d.data == loaded.data)
                .expect("the key's frame is in the RAW capture");
            assert_eq!(same.1, loaded, "key file = stream decode of the same frame");
        }
        // TX from the loaded key: the replayed frame decodes back.
        let tx = feed_all(&dec.encode(&loaded, 0x01).unwrap());
        assert!(!tx.is_empty() && tx.iter().all(|d| *d == loaded));

        // EndByte wins over CRC/Btn; 64-bit keys (older files) load; other Bit counts do not.
        let mut k = dec.to_flipper_key(&loaded).unwrap();
        k.push_u32("CRC", 1);
        assert_eq!(dec.from_flipper_key(&k).unwrap().button, Some(0x3B));
        k.bits = 64;
        assert_eq!(dec.from_flipper_key(&k), Some(loaded.clone()));
        k.bits = 70;
        assert!(dec.from_flipper_key(&k).is_none());
    }

    /// Oracle vectors (see [super::write_oracle_vector]): the IMPORTS key's frame and one more.
    #[test]
    #[ignore]
    fn oracle_vectors() {
        for (n, (hop, tail7)) in [(0xA03B_F868u32, 0x3Bu8), (0xFEC0_E84D, 0x1B)].into_iter().enumerate() {
            let up = FiatV0Decoder::new().encode(&frame(hop, FIX, tail7), 0).unwrap();
            super::write_oracle_vector(
                &format!("fiat_v0_{}", n + 1),
                433_920_000,
                "FuriHalSubGhzPresetOok650Async",
                &up,
                &mut FiatV0Decoder::new(),
            );
        }
    }

    /// Real capture: IMPORTS/FIAT/Fiat Bravo 2010 S.Croce 433.88 am650.sub decodes as Fiat V0 with
    /// one fix (serial) and a rolling hop, and one frame equals the saved key file
    /// IMPORTS/FIAT/Fiat type 0 N1.sub (Key: A0 3B F8 68 54 53 62 00).
    #[test]
    fn decodes_real_capture() {
        let path = Path::new("IMPORTS/FIAT/Fiat Bravo 2010 S.Croce 433.88 am650.sub");
        if !path.exists() {
            eprintln!("Skip: {:?} not found (run from crate root)", path);
            return;
        }
        let (freq, raw) = import_sub_raw(path).unwrap();
        let pairs: Vec<LevelDuration> =
            raw.iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect();
        let results = ProtocolRegistry::new().process_signal_stream(&pairs, freq);
        let fiat: Vec<&DecodedSignal> =
            results.iter().filter(|(n, _, _)| n == "Fiat V0").map(|(_, d, _)| d).collect();
        assert!(fiat.len() >= 5, "got {:?}", results.iter().map(|(n, _, _)| n).collect::<Vec<_>>());
        assert!(fiat.iter().all(|d| d.serial == Some(FIX)), "one fob");
        assert!(fiat.iter().any(|d| d.data == 0xA03B_F868_5453_6200), "matches Fiat type 0 N1.sub");
        // Every real tail is 0xxxx1 (+ the 71st bit, 1, from the next preamble).
        assert!(fiat.iter().all(|d| d.button.is_some_and(|b| b & 0x43 == 0x03)));
    }
}
