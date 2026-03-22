//! Porsche Touareg protocol decoder
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/porsche_touareg.c`.
//! Original implementation by @lupettohf
//!
//! Protocol characteristics:
//! - PWM encoding with Manchester-like bit pairs: SHORT LOW + LONG HIGH = 0, LONG LOW + SHORT HIGH = 1
//! - 64 bits total; sync preamble of 15+ LOW/HIGH pairs at 3370µs, then 5930µs gap pair, then data
//! - Field layout: pkt[0]=(btn<<4)|(frame_type&0x07), pkt[1..3]=serial 24-bit, pkt[4..7]=encrypted
//! - Counter recovery via brute-force matching of computed encrypted bytes against received bytes
//! - Frame types: 0x02="First", 0x01="Cont", 0x04="Final"
//! - Frequencies: 433.92 MHz and 868.35 MHz (AM demodulation)

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

/// Decoder states (matches PorscheCayenneDecoderStep in porsche_touareg.c)
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Sync,
    GapHigh,
    GapLow,
    Data,
}

/// Porsche Touareg protocol decoder
pub struct PorscheTouaregDecoder {
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
/// Matches the ROTATE24 macro in porsche_touareg.c exactly.
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
/// This is a direct port of `porsche_cayenne_compute_frame` from the C reference.
/// pkt[0..3] = plaintext header, pkt[4..7] = encrypted payload derived from
/// a 24-bit rotate register seeded from serial bytes and rotated (4 + counter_low) times.
fn compute_frame(serial24: u32, btn: u8, counter: u16, frame_type: u8) -> [u8; 8] {
    let b0 = (btn << 4) | (frame_type & 0x07);
    let b1 = ((serial24 >> 16) & 0xFF) as u8;
    let b2 = ((serial24 >> 8) & 0xFF) as u8;
    let b3 = (serial24 & 0xFF) as u8;

    let cnt = counter.wrapping_add(1);
    let cnt_lo = (cnt & 0xFF) as u8;
    let cnt_hi = ((cnt >> 8) & 0xFF) as u8;

    let mut r_h = b3;
    let mut r_m = b1;
    let mut r_l = b2;

    // Rotate 4 times initially
    for _ in 0..4 {
        rotate24(&mut r_h, &mut r_m, &mut r_l);
    }
    // Then rotate cnt_lo more times
    for _ in 0..cnt_lo as u16 {
        rotate24(&mut r_h, &mut r_m, &mut r_l);
    }

    let a9a = r_h ^ b0;

    let nb9b_p1 = ((!cnt_lo).wrapping_shl(2) & 0xFC) ^ r_m;
    let nb9b_p2 = ((!cnt_hi).wrapping_shl(2) & 0xFC) ^ r_m;
    let nb9b_p3 = ((!cnt_hi).wrapping_shr(6) & 0x03) ^ r_m;
    let a9b = (nb9b_p1 & 0xCC) | (nb9b_p2 & 0x30) | (nb9b_p3 & 0x03);

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

/// Parse raw 64-bit data into a DecodedSignal.
///
/// Extracts serial (24-bit), button (4-bit), frame_type (3-bit), then brute-forces
/// the counter (1..=256) by calling compute_frame and comparing encrypted bytes.
fn parse_data(data: u64) -> DecodedSignal {
    // Unpack 64-bit data into 8 bytes (big-endian)
    let mut pkt = [0u8; 8];
    let mut raw = data;
    for i in (0..8).rev() {
        pkt[i] = (raw & 0xFF) as u8;
        raw >>= 8;
    }

    let serial = ((pkt[1] as u32) << 16) | ((pkt[2] as u32) << 8) | (pkt[3] as u32);
    let btn = pkt[0] >> 4;
    let frame_type = pkt[0] & 0x07;

    // Brute-force counter recovery: try counter values 1..=256
    let mut counter: u16 = 0;
    for try_cnt in 1u16..=256 {
        let try_pkt = compute_frame(serial, btn, try_cnt - 1, frame_type);
        if try_pkt[4] == pkt[4]
            && try_pkt[5] == pkt[5]
            && try_pkt[6] == pkt[6]
            && try_pkt[7] == pkt[7]
        {
            counter = try_cnt;
            break;
        }
    }

    // Determine frame type name for extra info
    let frame_type_name = match frame_type {
        0x02 => "First",
        0x01 => "Cont",
        0x04 => "Final",
        _ => "??",
    };

    DecodedSignal {
        serial: Some(serial),
        button: Some(btn),
        counter: Some(counter),
        crc_valid: counter != 0,
        data,
        data_count_bit: MIN_COUNT_BIT,
        encoder_capable: false,
        extra: Some(frame_type as u64),
        protocol_display_name: Some(format!(
            "Porsche Touareg [{}]",
            frame_type_name
        )),
    }
}

impl PorscheTouaregDecoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            sync_count: 0,
            raw_data: 0,
            bit_count: 0,
            te_last: 0,
        }
    }
}

impl ProtocolDecoder for PorscheTouaregDecoder {
    fn name(&self) -> &'static str {
        "Porsche Touareg"
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
            // Reset: wait for a LOW pulse matching sync timing (3370µs)
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
                        // Keep collecting sync pairs -- HIGH sync pulse, stay in Sync
                    } else if self.sync_count >= PC_SYNC_MIN
                        && duration_diff!(duration, PC_TE_GAP) < TE_DELTA
                    {
                        // HIGH gap after sufficient sync pulses
                        self.step = DecoderStep::GapLow;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else {
                    // LOW pulse
                    if duration_diff!(duration, PC_TE_SYNC) < TE_DELTA {
                        self.sync_count += 1;
                    } else if self.sync_count >= PC_SYNC_MIN
                        && duration_diff!(duration, PC_TE_GAP) < TE_DELTA
                    {
                        // LOW gap after sufficient sync pulses
                        self.step = DecoderStep::GapHigh;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                }
            }

            // GapHigh: expect the complementary HIGH gap pulse
            DecoderStep::GapHigh => {
                if level && duration_diff!(duration, PC_TE_GAP) < TE_DELTA {
                    self.raw_data = 0;
                    self.bit_count = 0;
                    self.step = DecoderStep::Data;
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            // GapLow: expect the complementary LOW gap pulse
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
                    // HIGH pulse completes a bit pair
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
                        let result = parse_data(self.raw_data);
                        self.step = DecoderStep::Reset;
                        return Some(result);
                    }
                } else {
                    // LOW pulse: save duration for the bit pair
                    self.te_last = duration;
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
}

impl Default for PorscheTouaregDecoder {
    fn default() -> Self {
        Self::new()
    }
}
