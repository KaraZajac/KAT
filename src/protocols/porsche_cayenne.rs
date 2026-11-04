//! Porsche Cayenne protocol decoder/encoder
//!
//! Aligned with Flipper-ARF `lib/subghz/protocols/porsche_cayenne.c` (HEAD; protocol name
//! "Porsche AG"), the only reference that still carries this protocol: ProtoPirate removed its
//! `porsche_touareg.c` port of the same source ("remove untested porsche protocol", 2026-09-14),
//! so KAT retired its "Porsche Touareg" decoder too. `ProtocolRegistry::get()` maps the legacy
//! name "Porsche Touareg" (and "Porsche Touareg [First|Cont|Final]" display names of older
//! captures) to this decoder, so those captures transmit with this encoder.
//!
//! Protocol characteristics:
//! - PWM bit pairs: SHORT LOW + LONG HIGH = 0, LONG LOW + SHORT HIGH = 1 (1680/3370µs ±500)
//! - Preamble of ≥15 3370µs LOW/HIGH sync pulses (the fob sends 73 pairs), a 5930µs gap pair,
//!   then 64 MSB-first data bits
//! - pkt[0] = (btn << 4) | frame_type (0b010 First, 0b001 Cont, 0b100 Final), pkt[1..3] = 24-bit
//!   serial, pkt[4..7] = 24-bit rotating-register VAG cipher output over the 16-bit counter
//! - RF: AM/OOK; C flags 315 | 433 | 868
//!
//! Decoding follows the C: every preamble+gap+64-bit frame is reported, whatever its frame type,
//! with the counter recovered by brute force (`cnt = 0` when nothing matches → `crc_valid =
//! false`). The C only tries cipher counters 1..=256 (one `cnt_lo` sweep); KAT recovers the full
//! 16-bit counter, which the cipher output determines uniquely (pkt[4..6] carry `~cnt_lo`
//! verbatim and every `cnt_hi` bit lands in pkt[5..7]), so it reads `~cnt_lo` from the frame and
//! tries its 256 `cnt_hi` values: identical results for counters 1..=256, and the real counter
//! (where the C reports 0) beyond — a real fob's counter is usually far past 256, and the encoder
//! needs it. Every decoded frame is encoder-capable, as in ARF (its encoder only needs the saved
//! serial, button and counter).
//!
//! Encoder = `porsche_cayenne_build_upload`: 4 frames (types 0b010/0b001/0b100/0b100, cipher
//! counters cnt+1..cnt+4), each 73 sync pairs + gap pair + 64 PWM bits. Buttons: KAT's
//! Lock/Unlock/Trunk/Panic (0x01/0x02/0x04/0x08) are the C's D-pad codes Lock/Unlock/Trunk/Open;
//! any other request re-sends the captured button (the C's OK path does the same for captured
//! codes 1/2/4/8, but maps any other captured code to 0x01).

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

/// KAT generic button codes (Lock=0x01, Unlock=0x02, Trunk=0x04, Panic=0x08) — also the C's
/// `porsche_cayenne_custom_to_btn` codes (Lock, Unlock, Trunk, Open).
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
    data.to_be_bytes()
}

/// Brute-force counter recovery. Returns the cipher counter (the C's `try_cnt`, i.e. the value
/// `compute_frame` used after its internal +1) whose recomputed pkt[4..7] match the frame, or
/// `None` (the C's `cnt = 0`). The C sweeps cipher counters 1..=256; the frame stores `~cnt_lo`
/// in clear (pkt[4] b7:6, pkt[5] b1:0 and b7:6, pkt[6] b1:0 = `~cnt_lo` bits 1:0, 3:2, 7:6, 5:4),
/// so only that `cnt_lo` can match and trying its 256 `cnt_hi` values is the full 16-bit search.
fn recover_counter(serial: u32, btn: u8, frame_type: u8, pkt: &[u8; 8]) -> Option<u16> {
    let not_lo = ((pkt[4] >> 6) & 0x03) | ((pkt[5] & 0x03) << 2) | ((pkt[6] & 0x03) << 4) | (pkt[5] & 0xC0);
    let cnt_lo = !not_lo as u16;
    (0u16..=0xFF).map(|hi| (hi << 8) | cnt_lo).find(|&cnt| {
        // The cipher increments internally, so pass cnt - 1.
        compute_frame(serial, btn, cnt.wrapping_sub(1), frame_type)[4..] == pkt[4..]
    })
}

/// Parse a raw 64-bit frame like the C decoder's 64th-bit branch (it reports every frame).
fn parse_data(data: u64) -> DecodedSignal {
    let pkt = unpack_bytes(data);

    let serial = ((pkt[1] as u32) << 16) | ((pkt[2] as u32) << 8) | (pkt[3] as u32);
    let btn = pkt[0] >> 4;
    let frame_type = pkt[0] & 0x07;
    let counter = recover_counter(serial, btn, frame_type, &pkt);

    DecodedSignal {
        serial: Some(serial),
        button: Some(btn),
        counter: Some(counter.unwrap_or(0)),
        crc_valid: counter.is_some(),
        data,
        data_count_bit: MIN_COUNT_BIT,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
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

    /// Map a KAT generic button command to the 4-bit button code the C transmits
    /// (`porsche_cayenne_get_btn_code`): Up/Down/Left/Right = 0x01 Lock / 0x02 Unlock /
    /// 0x04 Trunk / 0x08 Open, which are KAT's generic codes; otherwise the captured button.
    fn map_button(button: u8, captured: Option<u8>) -> u8 {
        match button {
            BTN_LOCK | BTN_UNLOCK | BTN_TRUNK | BTN_PANIC => button,
            _ => captured.map_or(BTN_LOCK, |b| b & 0x0F),
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
        // C flags: SubGhzProtocolFlag_315 | SubGhzProtocolFlag_433 | SubGhzProtocolFlag_868.
        &[315_000_000, 433_920_000, 868_350_000]
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
                        self.sync_count = self.sync_count.wrapping_add(1);
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

                    if self.bit_count == MIN_COUNT_BIT {
                        self.step = DecoderStep::Reset;
                        return Some(parse_data(self.raw_data));
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
        let btn = Self::map_button(button, decoded.button);

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
    use crate::protocols::ProtocolRegistry;

    /// Feed a signal through a fresh decoder and return every decode.
    fn decode_all(signal: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = PorscheCayenneDecoder::new();
        signal.iter().filter_map(|ld| dec.feed(ld.level, ld.duration_us)).collect()
    }

    fn seed(serial: u32, btn: u8, counter: u16) -> DecodedSignal {
        DecodedSignal {
            serial: Some(serial),
            button: Some(btn),
            counter: Some(counter),
            crc_valid: true,
            data: 0,
            data_count_bit: MIN_COUNT_BIT,
            encoder_capable: true,
            extra: None,
            protocol_display_name: None,
        }
    }

    /// FNV-1a over (level byte, duration LE u32) — the digest the C-oracle harness prints.
    fn digest(pulses: &[LevelDuration]) -> u64 {
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        for p in pulses {
            let mut bytes = vec![p.level as u8];
            bytes.extend_from_slice(&p.duration_us.to_le_bytes());
            for b in bytes {
                h ^= b as u64;
                h = h.wrapping_mul(0x0000_0100_0000_01b3);
            }
        }
        h
    }

    /// Golden uploads of the unmodified C (`porsche_cayenne.c` HEAD compiled on the host,
    /// `subghz_protocol_encoder_porsche_cayenne_deserialize` with Serial/Btn/Counter and the
    /// D-pad button KAT's command maps to): (serial, captured btn, counter, KAT button, len,
    /// digest), plus the counters the C decoder reports for the 4 frames of that upload.
    #[test]
    fn encoder_matches_c_golden_uploads() {
        const GOLDEN: [(u32, u8, u16, u8, usize, u64); 4] = GOLDEN_UPLOADS;
        for (serial, captured, cnt, kat_btn, len, dig) in GOLDEN {
            let up = PorscheCayenneDecoder::new()
                .encode(&seed(serial, captured, cnt), kat_btn)
                .expect("encodes");
            assert_eq!(up.len(), len);
            assert_eq!(digest(&up), dig, "serial {serial:06X} cnt {cnt} btn {kat_btn:#X}");
            let hits = decode_all(&up);
            assert_eq!(hits.len(), 4);
            let tx_btn = PorscheCayenneDecoder::map_button(kat_btn, Some(captured));
            for (i, h) in hits.iter().enumerate() {
                assert_eq!(h.serial, Some(serial));
                assert_eq!(h.button, Some(tx_btn));
                assert_eq!(h.counter, Some(cnt.wrapping_add(i as u16 + 1)));
                assert!(h.crc_valid && h.encoder_capable);
            }
        }
    }

    /// C D-pad used for each KAT button: 0x01 Up, 0x04 Left, 0x08 Right, 0x10 (unknown) OK.
    const GOLDEN_UPLOADS: [(u32, u8, u16, u8, usize, u64); 4] = [
        (0xABCDEF, 0x1, 5, 0x01, 1104, 0xF680_2749_965E_1261),
        (0x123456, 0x3, 0x1234, 0x04, 1104, 0x0BE1_B715_FE3B_4851),
        (0xFFFFFE, 0x8, 0xFFFE, 0x08, 1104, 0x9F7C_FE02_6C41_7945),
        (0x000001, 0x2, 255, 0x10, 1104, 0x935F_53DF_783D_B931),
    ];

    #[test]
    fn encode_decode_roundtrip_via_encoder() {
        // Full encoder path: every frame of the 4-frame burst decodes back with the serial, the
        // mapped button and cipher counters cnt+1..cnt+4 — across the whole 16-bit range.
        let serials = [0x00ABCDEFu32, 0x00123456, 0x00000001, 0x00FFFFFE, 0x005A5A5A];
        let counters = [0u16, 255, 256, 0x1234, 0xFFFB];
        for (&serial, &counter) in serials.iter().zip(counters.iter()) {
            for btn in [BTN_LOCK, BTN_UNLOCK, BTN_TRUNK, BTN_PANIC] {
                let signal = PorscheCayenneDecoder::new()
                    .encode(&seed(serial, 0x3, counter), btn)
                    .expect("encode should succeed");
                let decoded = decode_all(&signal);
                assert_eq!(decoded.len(), 4);
                let types = [0b010u8, 0b001, 0b100, 0b100];
                for (i, d) in decoded.iter().enumerate() {
                    assert_eq!(d.serial, Some(serial));
                    assert_eq!(d.button, Some(btn));
                    assert_eq!(d.counter, Some(counter.wrapping_add(i as u16 + 1)));
                    assert!(d.crc_valid && d.encoder_capable);
                    assert_eq!(d.data_count_bit, MIN_COUNT_BIT);
                    assert_eq!((d.data >> 56) as u8 & 0x07, types[i]);
                    assert_eq!(d.protocol_display_name, None);
                }
            }
        }
    }

    #[test]
    fn unknown_button_request_resends_captured_button() {
        let a = PorscheCayenneDecoder::new().encode(&seed(0x123456, 0x3, 9), 0x10).unwrap();
        let hits = decode_all(&a);
        assert!(hits.iter().all(|h| h.button == Some(0x3)));
        let b = PorscheCayenneDecoder::new().encode(&seed(0x123456, 0x3, 9), 0x04).unwrap();
        assert!(decode_all(&b).iter().all(|h| h.button == Some(BTN_TRUNK)));
    }

    #[test]
    fn decodes_every_frame_type_like_the_c() {
        // The C reports every 64-bit frame; frame types it never sends (0b011, 0b111, …) included.
        for ft in 0u8..8 {
            let pkt = compute_frame(0x00C0FFEE, BTN_UNLOCK, 41, ft);
            let mut signal = Vec::new();
            PorscheCayenneDecoder::push_frame(&mut signal, &pkt);
            let d = decode_all(&signal);
            assert_eq!(d.len(), 1, "ft={ft:#b}");
            assert_eq!(d[0].data, u64::from_be_bytes(pkt));
            assert_eq!(d[0].counter, Some(42));
            assert!(d[0].crc_valid);
        }
    }

    #[test]
    fn cipher_inconsistent_frame_reports_counter_zero() {
        // A frame whose cipher bytes match no counter: reported (as the C does) with cnt 0. Of
        // pkt[4..7], 8 bits are ~cnt_lo and 8 map one-to-one onto cnt_hi (flipping one of those
        // just names another counter); the other 16 depend only on serial/button/cnt_lo, so a
        // flip there (pkt[4] b5:0 = 9A b7:2) matches no counter.
        let mut pkt = compute_frame(0x00ABCDEF, BTN_LOCK, 3, 0b010);
        pkt[4] ^= 0x01;
        let mut signal = Vec::new();
        PorscheCayenneDecoder::push_frame(&mut signal, &pkt);
        let d = decode_all(&signal);
        assert_eq!(d.len(), 1);
        assert_eq!(d[0].counter, Some(0));
        assert!(!d[0].crc_valid);
        assert!(d[0].encoder_capable);
    }

    #[test]
    fn rejects_truncated_frame() {
        // 63 of 64 data bits must not decode.
        let pkt = compute_frame(0x00ABCDEF, BTN_LOCK, 3, 0b010);
        let mut signal = Vec::new();
        PorscheCayenneDecoder::push_frame(&mut signal, &pkt);
        // Frame = 73 sync pairs + 1 gap pair + 64 bit pairs. Drop the last bit pair.
        let keep = (PC_SYNC_COUNT + 1 + 63) * 2;
        assert!(decode_all(&signal[..keep]).is_empty(), "63-bit frame must not decode");
    }

    #[test]
    fn counter_recovery_is_unique_and_matches_c_range() {
        // Every cipher counter is recovered exactly (the 16-bit counter is uniquely determined;
        // 1..=256 is the C's own range): all of 0..=300, then a stride across the rest.
        for cnt in (0u32..=300).chain((301..=0xFFFF).step_by(97)).chain([0xFFFF]) {
            let cnt = cnt as u16;
            let pkt = compute_frame(0x00ABCDEF, BTN_LOCK, cnt.wrapping_sub(1), 0b010);
            assert_eq!(recover_counter(0x00ABCDEF, BTN_LOCK, 0b010, &pkt), Some(cnt));
        }
    }

    #[test]
    fn cipher_header_layout() {
        // pkt[0]=(btn<<4)|ft, pkt[1..3]=serial big-endian; recovery reports the passed counter + 1.
        let pkt = compute_frame(0x00ABCDEF, 0x02, 0x10, 0b010);
        assert_eq!(pkt[0], (0x02 << 4) | 0b010, "pkt[0] = (btn<<4)|ft");
        assert_eq!(pkt[1], 0xAB, "serial MSB");
        assert_eq!(pkt[2], 0xCD, "serial mid");
        assert_eq!(pkt[3], 0xEF, "serial LSB");
        assert_eq!(recover_counter(0x00ABCDEF, 0x02, 0b010, &pkt), Some(0x11));
    }

    #[test]
    fn legacy_touareg_names_resolve_and_transmit() {
        let reg = ProtocolRegistry::new();
        for name in ["Porsche Touareg", "Porsche Touareg [First]", "porsche touareg [??]"] {
            let dec = reg.get(name).expect("legacy name resolves");
            assert_eq!(dec.name(), "Porsche Cayenne");
            assert!(dec.encode(&seed(0x123456, 0x1, 7), 0x02).is_some());
        }
    }

    #[test]
    fn registry_decodes_upload_only_as_porsche_cayenne() {
        let up = PorscheCayenneDecoder::new().encode(&seed(0x5A5A5A, 0x1, 0x2345), 0x01).unwrap();
        for f in [315_000_000, 433_920_000, 868_350_000] {
            let mut reg = ProtocolRegistry::new();
            let names: Vec<String> =
                reg.process_signal_stream(&up, f).into_iter().map(|(n, _, _)| n).collect();
            assert_eq!(names, vec!["Porsche Cayenne"; 4], "{f}");
        }
    }
}
