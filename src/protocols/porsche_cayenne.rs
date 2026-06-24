//! Porsche Cayenne protocol decoder/encoder
//!
//! Aligned with Flipper-ARF reference: `lib/subghz/protocols/porsche_cayenne.c`
//! (internal protocol name in the firmware header is "Porsche AG").
//!
//! # Relationship to the existing `Porsche Touareg` decoder
//!
//! KAT already ships `porsche_touareg.rs`, which is itself a port of this very same
//! `porsche_cayenne.c` source (its own doc comment and the `porsche_cayenne_compute_frame`
//! function name make that explicit). The two protocols are therefore **the same wire
//! protocol**: identical 1680/3370µs PWM, identical 73-pulse 3370µs preamble + 5930µs gap
//! pair, identical 64-bit MSB-first frame, identical 24-bit rotating-register VAG cipher,
//! and identical brute-force counter recovery / validity check (`counter != 0`).
//!
//! The only meaningful differences captured here versus Touareg:
//!   * Display name is `"Porsche Cayenne"` (vs `"Porsche Touareg"`).
//!   * This decoder ADDS the encoder from the C reference (the Touareg port is decode-only):
//!     a 4-frame burst (frame types 0b010/0b001/0b100/0b100), 73 sync pairs + gap pair +
//!     64 MSB-first PWM data bits per frame.
//!
//! Because the two share an identical frame and validity gate, this decoder is registered
//! AFTER `porsche_touareg` so Touareg keeps first-match priority and Cayenne can never steal
//! a Touareg capture (the registry reports the first decoder that fires on a given pulse —
//! see `process_signal_*_inner` in `mod.rs`). To keep Cayenne a faithful but strictly
//! non-stealing decoder, emission is additionally gated on the frame_type being one of the
//! three values the C only ever emits/labels (`0b001` Cont, `0b010` First, `0b100` Final).
//! That is a strict subset of what Touareg accepts, so on any shared frame Touareg fires too
//! and wins by ordering; Cayenne only ever fires on frames Touareg also accepts.
//!
//! Protocol characteristics:
//! - PWM bit pairs: SHORT LOW + LONG HIGH = 0, LONG LOW + SHORT HIGH = 1
//! - 64 bits total; sync preamble of 15+ LOW/HIGH pairs at 3370µs, then 5930µs gap pair, then data
//! - Field layout: pkt[0]=(btn<<4)|(frame_type&0x07), pkt[1..3]=serial 24-bit, pkt[4..7]=encrypted
//! - Counter recovery via brute-force matching of computed encrypted bytes against received bytes
//! - Frame types: 0x02="First", 0x01="Cont", 0x04="Final"
//! - RF: AM/OOK. Frequencies: 433.92 MHz and 868.35 MHz (C flags 433|868)

use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

const TE_SHORT: u32 = 1680;
const TE_LONG: u32 = 3370;
const TE_DELTA: u32 = 500;
const MIN_COUNT_BIT: usize = 64;

const PC_TE_SYNC: u32 = 3370;
const PC_TE_GAP: u32 = 5930;
const PC_SYNC_MIN: u16 = 15;
/// Actual preamble pulse-pair count emitted by the firmware (PC_SYNC_COUNT).
const PC_SYNC_COUNT: usize = 73;

/// KAT generic button codes (Lock=0x01, Unlock=0x02, Trunk=0x04, Panic=0x08).
const BTN_LOCK: u8 = 0x01;
const BTN_UNLOCK: u8 = 0x02;
const BTN_TRUNK: u8 = 0x04;
const BTN_PANIC: u8 = 0x08;

/// Decoder states (matches PCDecoderStep in porsche_cayenne.c).
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Sync,
    GapHigh,
    GapLow,
    Data,
}

/// Porsche Cayenne protocol decoder/encoder.
pub struct PorscheCayenneDecoder {
    step: DecoderStep,
    sync_count: u16,
    raw_data: u64,
    bit_count: usize,
    te_last: u32,
}

/// Circular left-shift of a 24-bit register stored in three bytes (h, m, l).
///
/// Each byte shifts left by 1, receiving the MSB of the next byte in the chain:
///   h gets MSB of m, m gets MSB of l, l gets MSB of h (wrap-around).
///
/// Matches the ROTATE24 macro in porsche_cayenne.c exactly.
#[inline]
fn rotate24(r_h: &mut u8, r_m: &mut u8, r_l: &mut u8) {
    let ch = (*r_h >> 7) & 1;
    let cm = (*r_m >> 7) & 1;
    let cl = (*r_l >> 7) & 1;
    *r_h = (*r_h << 1) | cm;
    *r_m = (*r_m << 1) | cl;
    *r_l = (*r_l << 1) | ch;
}

/// Compute an 8-byte frame from serial, button, counter, and frame_type.
///
/// Direct port of `porsche_cayenne_compute_frame` from the C reference. The cipher
/// increments `counter` by 1 internally. pkt[0..3] = plaintext header, pkt[4..7] = cipher
/// output derived from a 24-bit rotate register seeded from serial bytes and rotated
/// (4 + counter_low) times.
fn compute_frame(serial24: u32, btn: u8, counter: u16, frame_type: u8) -> [u8; 8] {
    let b0 = (btn << 4) | (frame_type & 0x07);
    let b1 = ((serial24 >> 16) & 0xFF) as u8;
    let b2 = ((serial24 >> 8) & 0xFF) as u8;
    let b3 = (serial24 & 0xFF) as u8;

    // Internal counter increment (firmware @ 0x14122).
    let cnt = counter.wrapping_add(1);
    let cnt_lo = (cnt & 0xFF) as u8;
    let cnt_hi = ((cnt >> 8) & 0xFF) as u8;

    // Seed 24-bit register: r_h <- serial LSB, r_m <- serial MSB, r_l <- serial mid.
    let mut r_h = b3;
    let mut r_m = b1;
    let mut r_l = b2;

    // Loop 1: 4 fixed rotations.
    for _ in 0..4 {
        rotate24(&mut r_h, &mut r_m, &mut r_l);
    }
    // Loop 2: cnt_lo additional rotations.
    for _ in 0..cnt_lo as u16 {
        rotate24(&mut r_h, &mut r_m, &mut r_l);
    }

    // 9A: XOR of r_h with base byte.
    let a9a = r_h ^ b0;

    // 9B: three masked slices of (~cnt_lo / ~cnt_hi) XOR r_m.
    let nb9b_p1 = ((!cnt_lo).wrapping_shl(2) & 0xFC) ^ r_m;
    let nb9b_p2 = ((!cnt_hi).wrapping_shl(2) & 0xFC) ^ r_m;
    let nb9b_p3 = ((!cnt_hi).wrapping_shr(6) & 0x03) ^ r_m;
    let a9b = (nb9b_p1 & 0xCC) | (nb9b_p2 & 0x30) | (nb9b_p3 & 0x03);

    // 9C: three masked slices of (~cnt_lo / ~cnt_hi) XOR r_l.
    let nb9c_p1 = ((!cnt_lo).wrapping_shr(2) & 0x3F) ^ r_l;
    let nb9c_p2 = ((!cnt_hi & 0x03).wrapping_shl(6)) ^ r_l;
    let nb9c_p3 = ((!cnt_hi).wrapping_shr(2) & 0x3F) ^ r_l;
    let a9c = (nb9c_p1 & 0x33) | (nb9c_p2 & 0xC0) | (nb9c_p3 & 0x0C);

    let mut pkt = [0u8; 8];
    pkt[0] = b0;
    pkt[1] = b1;
    pkt[2] = b2;
    pkt[3] = b3;
    pkt[4] = ((a9a >> 2) & 0x3F) | ((!cnt_lo & 0x03) << 6);
    pkt[5] = (!cnt_lo & 0xC0) | ((a9a & 0x03) << 4) | (a9b & 0x0C) | ((!cnt_lo).wrapping_shr(2) & 0x03);
    pkt[6] = ((a9b & 0x03) << 6) | ((a9c >> 2) & 0x3C) | ((!cnt_lo).wrapping_shr(4) & 0x03);
    pkt[7] = ((a9b >> 4) & 0x0F) | ((a9c & 0x0F) << 4);

    pkt
}

/// Unpack a raw 64-bit frame into its 8 bytes (big-endian: pkt[0] is the MSB).
fn unpack_bytes(data: u64) -> [u8; 8] {
    let mut pkt = [0u8; 8];
    let mut raw = data;
    for i in (0..8).rev() {
        pkt[i] = (raw & 0xFF) as u8;
        raw >>= 8;
    }
    pkt
}

/// Brute-force counter recovery: try counter values 1..=256 (matching the C loop) and
/// compare the recomputed cipher bytes pkt[4..7]. Returns 0 if no match (invalid frame).
fn recover_counter(serial: u32, btn: u8, frame_type: u8, pkt: &[u8; 8]) -> u16 {
    for try_cnt in 1u16..=256 {
        // The cipher increments internally, so pass try_cnt - 1.
        let try_pkt = compute_frame(serial, btn, try_cnt - 1, frame_type);
        if try_pkt[4] == pkt[4]
            && try_pkt[5] == pkt[5]
            && try_pkt[6] == pkt[6]
            && try_pkt[7] == pkt[7]
        {
            return try_cnt;
        }
    }
    0
}

/// Parse raw 64-bit data into a DecodedSignal, or `None` if it is not a valid Cayenne frame.
///
/// Cayenne-specific gate (keeps this decoder from stealing Touareg captures): the frame_type
/// must be one of the three values the C reference emits/labels (0b001 Cont, 0b010 First,
/// 0b100 Final), AND the counter must be recoverable (cipher-consistent, `counter != 0`).
fn parse_data(data: u64) -> Option<DecodedSignal> {
    let pkt = unpack_bytes(data);

    let serial = ((pkt[1] as u32) << 16) | ((pkt[2] as u32) << 8) | (pkt[3] as u32);
    let btn = pkt[0] >> 4;
    let frame_type = pkt[0] & 0x07;

    // Cayenne-specific invariant: only the three documented frame types.
    let frame_type_name = match frame_type {
        0b010 => "First",
        0b001 => "Cont",
        0b100 => "Final",
        _ => return None,
    };

    // Cipher consistency check (also the C's validity signal).
    let counter = recover_counter(serial, btn, frame_type, &pkt);
    if counter == 0 {
        return None;
    }

    Some(DecodedSignal {
        serial: Some(serial),
        button: Some(btn),
        counter: Some(counter),
        crc_valid: true, // cipher-consistent frame
        data,
        data_count_bit: MIN_COUNT_BIT,
        encoder_capable: true,
        extra: Some(frame_type as u64),
        protocol_display_name: Some(format!("Porsche Cayenne [{}]", frame_type_name)),
    })
}

impl PorscheCayenneDecoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            sync_count: 0,
            raw_data: 0,
            bit_count: 0,
            te_last: 0,
        }
    }

    /// Map a KAT generic button command to a Porsche Cayenne / VAG 4-bit button code.
    /// KAT: Lock=0x01, Unlock=0x02, Trunk=0x04, Panic=0x08.
    /// The C `porsche_cayenne_get_btn_code` maps d-pad: Up=0x01 Lock, Down=0x02 Unlock,
    /// Left=0x04 Trunk, Right=0x08 Open — an identical set, so the codes pass through.
    fn map_button(button: u8) -> u8 {
        match button {
            BTN_LOCK | BTN_UNLOCK | BTN_TRUNK | BTN_PANIC => button & 0x0F,
            b => b & 0x0F,
        }
    }

    /// Append one PWM data bit as a (LOW, HIGH) pair, matching the C encoder:
    ///   bit 0: SHORT LOW + LONG HIGH; bit 1: LONG LOW + SHORT HIGH.
    fn push_bit(signal: &mut Vec<LevelDuration>, bit: bool) {
        if bit {
            signal.push(LevelDuration::new(false, TE_LONG));
            signal.push(LevelDuration::new(true, TE_SHORT));
        } else {
            signal.push(LevelDuration::new(false, TE_SHORT));
            signal.push(LevelDuration::new(true, TE_LONG));
        }
    }

    /// Emit one full frame (73 sync pairs + gap pair + 64 MSB-first data bits) into `signal`.
    /// Matches `porsche_cayenne_build_upload`'s per-frame body.
    fn push_frame(signal: &mut Vec<LevelDuration>, pkt: &[u8; 8]) {
        // Preamble: 73 × (LOW LONG + HIGH LONG).
        for _ in 0..PC_SYNC_COUNT {
            signal.push(LevelDuration::new(false, TE_LONG));
            signal.push(LevelDuration::new(true, TE_LONG));
        }
        // Gap: LOW GAP + HIGH GAP.
        signal.push(LevelDuration::new(false, PC_TE_GAP));
        signal.push(LevelDuration::new(true, PC_TE_GAP));
        // 64 data bits, MSB first, byte order pkt[0]->pkt[7].
        for &byte in pkt.iter() {
            for bit in (0..8).rev() {
                Self::push_bit(signal, (byte >> bit) & 1 != 0);
            }
        }
    }
}

impl ProtocolDecoder for PorscheCayenneDecoder {
    fn name(&self) -> &'static str {
        "Porsche Cayenne"
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
        // C flags: SubGhzProtocolFlag_433 | SubGhzProtocolFlag_868.
        &[433_920_000, 868_350_000]
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.sync_count = 0;
        self.raw_data = 0;
        self.bit_count = 0;
        self.te_last = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            // Reset: wait for a LOW pulse matching sync timing (3370µs).
            DecoderStep::Reset => {
                if !level && duration_diff!(duration, PC_TE_SYNC) < TE_DELTA {
                    self.sync_count = 1;
                    self.step = DecoderStep::Sync;
                }
            }

            // Sync: count sync pulses (HIGH and LOW at 3370µs).
            // On a gap pulse (5930µs) with enough sync pulses, transition to GapHigh/GapLow.
            DecoderStep::Sync => {
                if level {
                    if duration_diff!(duration, PC_TE_SYNC) < TE_DELTA {
                        // Good sync HIGH — keep collecting.
                    } else if self.sync_count >= PC_SYNC_MIN
                        && duration_diff!(duration, PC_TE_GAP) < TE_DELTA
                    {
                        // HIGH gap after sufficient sync pulses.
                        self.step = DecoderStep::GapLow;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else {
                    // LOW pulse.
                    if duration_diff!(duration, PC_TE_SYNC) < TE_DELTA {
                        self.sync_count += 1;
                    } else if self.sync_count >= PC_SYNC_MIN
                        && duration_diff!(duration, PC_TE_GAP) < TE_DELTA
                    {
                        // LOW gap after sufficient sync pulses.
                        self.step = DecoderStep::GapHigh;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                }
            }

            // GapHigh: expect the complementary HIGH gap pulse.
            DecoderStep::GapHigh => {
                if level && duration_diff!(duration, PC_TE_GAP) < TE_DELTA {
                    self.raw_data = 0;
                    self.bit_count = 0;
                    self.step = DecoderStep::Data;
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            // GapLow: expect the complementary LOW gap pulse.
            DecoderStep::GapLow => {
                if !level && duration_diff!(duration, PC_TE_GAP) < TE_DELTA {
                    self.raw_data = 0;
                    self.bit_count = 0;
                    self.step = DecoderStep::Data;
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            // Data: decode bit pairs.
            // LOW pulses are saved in te_last; HIGH pulses complete the bit:
            //   SHORT LOW + LONG HIGH = bit 0
            //   LONG LOW + SHORT HIGH = bit 1
            DecoderStep::Data => {
                if level {
                    let bit_value;
                    if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA
                        && duration_diff!(duration, TE_LONG) < TE_DELTA
                    {
                        bit_value = false; // bit 0
                    } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA
                        && duration_diff!(duration, TE_SHORT) < TE_DELTA
                    {
                        bit_value = true; // bit 1
                    } else {
                        self.step = DecoderStep::Reset;
                        return None;
                    }

                    self.raw_data = (self.raw_data << 1) | (bit_value as u64);
                    self.bit_count += 1;

                    if self.bit_count >= MIN_COUNT_BIT {
                        let data = self.raw_data;
                        self.step = DecoderStep::Reset;
                        // parse_data returns None if the Cayenne gate fails (not a Cayenne
                        // frame), so this decoder stays silent on those and never steals them.
                        return parse_data(data);
                    }
                } else {
                    // LOW pulse: save duration for the bit pair.
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
        let serial = decoded.serial? & 0xFFFFFF;
        let cnt = decoded.counter.unwrap_or(0);
        let btn = Self::map_button(button);

        // 4-frame burst (matches porsche_cayenne_build_upload):
        //   Frame 0: frame_type=0b010, cipher counter = cnt+1
        //   Frame 1: frame_type=0b001, cipher counter = cnt+2
        //   Frame 2: frame_type=0b100, cipher counter = cnt+3
        //   Frame 3: frame_type=0b100, cipher counter = cnt+4
        // (compute_frame increments the passed counter by 1 internally.)
        const FRAME_TYPES: [u8; 4] = [0b010, 0b001, 0b100, 0b100];

        // Per-frame size: 73 sync pairs + 1 gap pair + 64 bit pairs = (73 + 1 + 64) * 2.
        let mut signal = Vec::with_capacity((PC_SYNC_COUNT + 1 + 64) * 2 * 4);

        for (f, &ft) in FRAME_TYPES.iter().enumerate() {
            let pkt = compute_frame(serial, btn, cnt.wrapping_add(f as u16), ft);
            Self::push_frame(&mut signal, &pkt);
        }

        Some(signal)
    }
}

impl Default for PorscheCayenneDecoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Feed an encoded signal back through a fresh decoder and return the first decode.
    fn decode_signal(signal: &[LevelDuration]) -> Option<DecodedSignal> {
        let mut dec = PorscheCayenneDecoder::new();
        for ld in signal {
            if let Some(d) = dec.feed(ld.level, ld.duration_us) {
                return Some(d);
            }
        }
        None
    }

    /// Build a one-frame on-air signal for a single (serial, btn, counter, frame_type) and
    /// decode it back. The frame's cipher counter is `counter+1` (compute_frame increments),
    /// and the decoder's brute-force recovery reports that incremented value.
    fn roundtrip_single(serial: u32, btn: u8, counter: u16, frame_type: u8) -> DecodedSignal {
        let pkt = compute_frame(serial, btn, counter, frame_type);
        let mut signal = Vec::new();
        PorscheCayenneDecoder::push_frame(&mut signal, &pkt);
        decode_signal(&signal)
            .unwrap_or_else(|| panic!("decode failed for serial {serial:#X} btn {btn:#X} cnt {counter} ft {frame_type:#b}"))
    }

    #[test]
    fn encode_decode_roundtrip_via_encoder() {
        // Full encoder path: encode() emits a 4-frame burst; the first frame (type 0b010,
        // cipher counter = cnt+1) must decode back with the right serial/button, and the
        // recovered counter must survive the rolling cipher (== cnt+1).
        let serials = [0x00ABCDEFu32, 0x00123456, 0x00000001, 0x00FFFFFE, 0x005A5A5A];
        let buttons = [BTN_LOCK, BTN_UNLOCK, BTN_TRUNK, BTN_PANIC];
        // Counters kept within the C reference's recoverable range: its decoder only
        // brute-forces cnt_lo (256 values), so the first burst frame's cipher counter
        // (seed+1) must be <= 256, i.e. seed <= 255. See `counter_recovery_limit_matches_c`.
        let counters = [0u16, 1, 42, 200, 254, 255];

        for (&serial, &counter) in serials.iter().zip(counters.iter()) {
            for &btn in &buttons {
                let seed = DecodedSignal {
                    serial: Some(serial),
                    button: Some(btn),
                    counter: Some(counter),
                    crc_valid: true,
                    data: 0,
                    data_count_bit: MIN_COUNT_BIT,
                    encoder_capable: true,
                    extra: None,
                    protocol_display_name: None,
                };
                let encoder = PorscheCayenneDecoder::new();
                let signal = encoder.encode(&seed, btn).expect("encode should succeed");
                let decoded = decode_signal(&signal).unwrap_or_else(|| {
                    panic!("decode failed for serial {serial:#X} btn {btn:#X} cnt {counter}")
                });

                let expected_btn = PorscheCayenneDecoder::map_button(btn);
                assert_eq!(decoded.serial, Some(serial & 0xFFFFFF), "serial");
                assert_eq!(decoded.button, Some(expected_btn), "button");
                // First frame's cipher counter is cnt+1 (compute_frame increments).
                assert_eq!(
                    decoded.counter,
                    Some(counter.wrapping_add(1)),
                    "counter must survive the rolling cipher"
                );
                assert_eq!(decoded.data_count_bit, MIN_COUNT_BIT, "bit count");
                assert!(decoded.crc_valid, "cipher-consistent frame → crc_valid");
                // First burst frame is type 0b010 = "First".
                assert_eq!(decoded.extra, Some(0b010), "frame_type First");
                assert_eq!(
                    decoded.protocol_display_name.as_deref(),
                    Some("Porsche Cayenne [First]")
                );
            }
        }
    }

    #[test]
    fn roundtrip_all_frame_types() {
        // Each of the three documented frame types decodes and labels correctly, and the
        // counter survives across the cipher for a spread of counter values.
        let cases = [
            (0b010u8, "Porsche Cayenne [First]"),
            (0b001u8, "Porsche Cayenne [Cont]"),
            (0b100u8, "Porsche Cayenne [Final]"),
        ];
        for &(ft, name) in &cases {
            // Counters within the C's recoverable cnt_lo range (cipher counter seed+1 <= 256).
            for &counter in &[0u16, 7, 200, 254, 255] {
                let d = roundtrip_single(0x00C0FFEE, BTN_UNLOCK, counter, ft);
                assert_eq!(d.serial, Some(0x00C0FFEE), "serial ft={ft:#b}");
                assert_eq!(d.button, Some(BTN_UNLOCK), "button ft={ft:#b}");
                assert_eq!(
                    d.counter,
                    Some(counter.wrapping_add(1)),
                    "counter survives cipher ft={ft:#b} cnt={counter}"
                );
                assert_eq!(d.extra, Some(ft as u64), "frame_type ft={ft:#b}");
                assert_eq!(d.protocol_display_name.as_deref(), Some(name));
                assert!(d.crc_valid);
            }
        }
    }

    #[test]
    fn rejects_undocumented_frame_type() {
        // A structurally valid PWM frame whose frame_type is NOT one of {0b001,0b010,0b100}
        // must be rejected by Cayenne's gate — this is what keeps it from stealing arbitrary
        // 1680/3370µs PWM frames (and what makes Touareg, ordered first, the owner of any
        // shared frame). frame_type 0b011 is unused by the C reference.
        let pkt = compute_frame(0x00123456, BTN_LOCK, 5, 0b011);
        let mut signal = Vec::new();
        PorscheCayenneDecoder::push_frame(&mut signal, &pkt);
        assert!(
            decode_signal(&signal).is_none(),
            "undocumented frame_type 0b011 must not decode as Cayenne"
        );
    }

    #[test]
    fn rejects_truncated_frame() {
        // 63 of 64 data bits must not decode.
        let pkt = compute_frame(0x00ABCDEF, BTN_LOCK, 3, 0b010);
        let mut signal = Vec::new();
        PorscheCayenneDecoder::push_frame(&mut signal, &pkt);
        // Frame = 73 sync pairs + 1 gap pair + 64 bit pairs. Drop the last bit pair.
        let keep = (PC_SYNC_COUNT + 1 + 63) * 2;
        assert!(
            decode_signal(&signal[..keep]).is_none(),
            "63-bit frame must not decode"
        );
    }

    #[test]
    fn counter_recovery_limit_matches_c() {
        // Faithful limitation of the C reference: its decoder brute-forces only cnt_lo
        // (try_cnt 1..=256), so frames whose cipher counter (passed counter + 1) exceeds 256
        // cannot have their counter recovered. recover_counter returns 0 for those, and the
        // frame is reported invalid (does not decode) — exactly as the C would behave.
        // seed=255 -> cipher counter 256 -> recoverable; seed=256 -> 257 -> NOT recoverable.
        let pkt_ok = compute_frame(0x00ABCDEF, BTN_LOCK, 255, 0b010);
        assert_eq!(
            recover_counter(0x00ABCDEF, BTN_LOCK, 0b010, &pkt_ok),
            256,
            "cipher counter 256 is the highest recoverable value"
        );
        let pkt_oob = compute_frame(0x00ABCDEF, BTN_LOCK, 256, 0b010);
        assert_eq!(
            recover_counter(0x00ABCDEF, BTN_LOCK, 0b010, &pkt_oob),
            0,
            "cipher counter 257 is beyond the C's cnt_lo brute-force range"
        );
        // And such an out-of-range frame must not decode (parse_data rejects counter==0).
        let mut signal = Vec::new();
        PorscheCayenneDecoder::push_frame(&mut signal, &pkt_oob);
        assert!(
            decode_signal(&signal).is_none(),
            "frame with unrecoverable counter must not decode (matches C)"
        );
    }

    #[test]
    fn cipher_matches_touareg_reference_vectors() {
        // The cipher/frame is shared with the Touareg port (same C source). Spot-check that
        // compute_frame -> unpack header bytes are exactly as laid out by the C reference:
        // pkt[0]=(btn<<4)|ft, pkt[1..3]=serial big-endian.
        let pkt = compute_frame(0x00ABCDEF, 0x02, 0x10, 0b010);
        assert_eq!(pkt[0], (0x02 << 4) | 0b010, "pkt[0] = (btn<<4)|ft");
        assert_eq!(pkt[1], 0xAB, "serial MSB");
        assert_eq!(pkt[2], 0xCD, "serial mid");
        assert_eq!(pkt[3], 0xEF, "serial LSB");
        // Recovering the counter from this exact frame yields cnt+1 = 0x11.
        let cnt = recover_counter(0x00ABCDEF, 0x02, 0b010, &pkt);
        assert_eq!(cnt, 0x11, "brute-force counter recovery = passed counter + 1");
    }
}
