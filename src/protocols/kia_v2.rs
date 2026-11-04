//! Kia V2 decoder + encoder.
//!
//! Port of ProtoPirate `protocols/kia_v2.c` (HEAD 2026-10; unchanged in behaviour since KAT's
//! v1.3.0 port — only `pp_emit` refactoring). Flipper-ARF's `kia_v2.c` decodes identically (C
//! oracle: same frames on random C-encoded streams and on IMPORTS) and builds the same upload.
//!
//! Air format (315 / 433.92 MHz, AM): Manchester, te_short 500µs / te_long 1000µs (te_delta 150,
//! strict `<`). Preamble: ≥100 long pulses, then the first short HIGH is the sync and the first
//! data bit (`1`). 53-bit frame: `1 << 52 | serial32 << 20 | button4 << 16 | raw_cnt12 << 4 |
//! crc4`, where `raw_cnt = cnt_lo8 << 4 | cnt_hi4` (counter 12 bits) and the CRC-4 is the XOR of
//! every nibble of bits 4..51 plus 1 (displayed only; the C emits regardless).
//!
//! Decoder (`kia_protocol_decoder_v2_feed`): Flipper's Manchester machine (common.rs) with
//! ProtoPirate's `pp_manchester_event` mapping (`level ? ShortLow : ShortHigh`), reset to Mid1 on
//! entering the preamble; any out-of-tolerance pulse in the data phase restarts the decoder. The
//! frame is reported on the pulse completing bit 53. (KAT's earlier port used a hand-written
//! Manchester table with the opposite event sense; the C oracle and this port agree on 600 random
//! C-encoded frames × 2 bursts.)
//!
//! Encoder (`kia_protocol_encoder_v2_deserialize` + `get_upload`): frame rebuilt from the fields,
//! CRC recomputed; 2 bursts, each 252 × (LOW 1000, HIGH 1000), LOW 500, then bits 51..0 as
//! Manchester (1 → HIGH 500 + LOW 500, 0 → LOW 500 + HIGH 500); adjacent equal levels merged.
//! One upload per call. Like the emulate scene, each press sends counter + 1 (12 bits); KAT's
//! generic button maps to ProtoPirate's / ARF's D-pad codes: Lock → 1, Unlock → 2, Trunk → 3,
//! Panic → 4; anything else keeps the captured button.

use super::common::{ManchesterEvent, ManchesterState};
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

const TE_SHORT: u32 = 500;
const TE_LONG: u32 = 1000;
const TE_DELTA: u32 = 150;
const MIN_COUNT_BIT: usize = 53;
const HEADER_PAIRS: usize = 252;
const TOTAL_BURSTS: usize = 2;

/// Decoder states (`KiaV2DecoderStep`).
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    CheckPreamble,
    CollectRawBits,
}

/// Kia V2 decoder.
pub struct KiaV2Decoder {
    step: DecoderStep,
    te_last: u32,
    header_count: u16,
    decode_data: u64,
    /// `SubGhzBlockDecoder.decode_count_bit` is a `uint8_t`.
    decode_count_bit: u8,
    manchester: ManchesterState,
}

#[inline]
fn is_short(d: u32) -> bool {
    duration_diff!(d, TE_SHORT) < TE_DELTA
}

#[inline]
fn is_long(d: u32) -> bool {
    duration_diff!(d, TE_LONG) < TE_DELTA
}

/// `kia_v2_calculate_crc`: XOR of both nibbles of the 6 bytes above the CRC nibble, plus 1.
fn calculate_crc(data: u64) -> u8 {
    let d = data >> 4;
    let crc = (0..6).fold(0u8, |acc, i| {
        let b = (d >> (8 * i)) as u8;
        acc ^ (b & 0x0F) ^ (b >> 4)
    });
    crc.wrapping_add(1) & 0x0F
}

/// The 53-bit frame for the fields, CRC included (encoder deserialize + get_upload).
fn build_frame(serial: u32, button: u8, counter: u16) -> u64 {
    let counter = counter as u32;
    let mut data = 1u64 << 52;
    data |= ((serial as u64) << 20) & 0x00FF_FFFF_FFF0_0000;
    data |= (((counter & 0xFF) << 8) | (((button & 0x0F) as u32) << 16) | ((counter >> 4) & 0xF0)) as u64;
    (data & !0x0F) | calculate_crc(data) as u64
}

/// Fields of a frame (the decoder's emission): serial, button, byte-swapped 12-bit counter.
fn parse_frame(data: u64) -> DecodedSignal {
    let raw_count = ((data >> 4) & 0xFFF) as u16;
    DecodedSignal {
        serial: Some((data >> 20) as u32),
        button: Some(((data >> 16) & 0x0F) as u8),
        counter: Some(((raw_count >> 4) | (raw_count << 8)) & 0xFFF),
        crc_valid: (data & 0x0F) as u8 == calculate_crc(data),
        data,
        data_count_bit: MIN_COUNT_BIT,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

impl KiaV2Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            te_last: 0,
            header_count: 0,
            decode_data: 0,
            decode_count_bit: 0,
            manchester: ManchesterState::Mid1,
        }
    }
}

impl ProtocolDecoder for KiaV2Decoder {
    fn name(&self) -> &'static str {
        "Kia V2"
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
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.header_count = 0;
        self.manchester = ManchesterState::Mid1;
        self.decode_data = 0;
        self.decode_count_bit = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::Reset => {
                if level && is_long(duration) {
                    self.step = DecoderStep::CheckPreamble;
                    self.te_last = duration;
                    self.header_count = 0;
                    self.manchester = ManchesterState::Mid1;
                }
            }
            DecoderStep::CheckPreamble => {
                if is_long(duration) {
                    self.te_last = duration;
                    self.header_count = self.header_count.wrapping_add(1);
                } else if is_short(duration) {
                    if level && self.header_count >= 100 {
                        self.header_count = 0;
                        self.decode_data = 1;
                        self.decode_count_bit = 2;
                        self.step = DecoderStep::CollectRawBits;
                    } else {
                        self.te_last = duration;
                    }
                } else {
                    self.step = DecoderStep::Reset;
                }
            }
            DecoderStep::CollectRawBits => {
                // pp_manchester_event: level ? Short/LongLow : Short/LongHigh, else restart.
                let event = if is_short(duration) {
                    ManchesterEvent::from_half_bit(!level, false)
                } else if is_long(duration) {
                    ManchesterEvent::from_half_bit(!level, true)
                } else {
                    self.step = DecoderStep::Reset;
                    return None;
                };
                if let Some(bit) = self.manchester.advance(event) {
                    self.decode_data = (self.decode_data << 1) | bit as u64;
                    self.decode_count_bit = self.decode_count_bit.wrapping_add(1);
                    if self.decode_count_bit as usize == MIN_COUNT_BIT {
                        let frame = parse_frame(self.decode_data);
                        self.decode_data = 0;
                        self.decode_count_bit = 0;
                        self.header_count = 0;
                        self.step = DecoderStep::Reset;
                        return Some(frame);
                    }
                }
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        let parsed = parse_frame(decoded.data);
        let serial = decoded.serial.or(parsed.serial)?;
        let captured = decoded.button.or(parsed.button).unwrap_or(0);
        let counter = decoded.counter.or(parsed.counter).unwrap_or(0).wrapping_add(1) & 0x0FFF;
        let button = match button {
            0x01 => 0x1,
            0x02 => 0x2,
            0x04 => 0x3,
            0x08 => 0x4,
            _ => captured,
        };
        Some(build_upload(build_frame(serial, button, counter)))
    }
}

impl Default for KiaV2Decoder {
    fn default() -> Self {
        Self::new()
    }
}

/// Append one level, merging with the previous entry when the level repeats (identical on air).
fn push(out: &mut Vec<LevelDuration>, level: bool, duration: u32) {
    match out.last_mut() {
        Some(last) if last.level == level => last.duration_us += duration,
        _ => out.push(LevelDuration::new(level, duration)),
    }
}

/// `kia_protocol_encoder_v2_get_upload` for a frame.
fn build_upload(frame: u64) -> Vec<LevelDuration> {
    let mut out = Vec::with_capacity(TOTAL_BURSTS * (HEADER_PAIRS * 2 + 1 + 2 * (MIN_COUNT_BIT - 1)));
    for _ in 0..TOTAL_BURSTS {
        for _ in 0..HEADER_PAIRS {
            push(&mut out, false, TE_LONG);
            push(&mut out, true, TE_LONG);
        }
        push(&mut out, false, TE_SHORT);
        for bit in (0..MIN_COUNT_BIT - 1).rev() {
            let one = (frame >> bit) & 1 != 0;
            push(&mut out, one, TE_SHORT);
            push(&mut out, !one, TE_SHORT);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn feed_all(wire: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = KiaV2Decoder::new();
        wire.iter().filter_map(|p| dec.feed(p.level, p.duration_us)).collect()
    }

    /// Golden frames from the C (`kia_v2.c` on the host): Serial/Btn/Cnt in, `generic.data` out;
    /// its decoder reports each burst with the untransmitted start bit (bit 52) clear.
    #[rustfmt::skip]
    const C_VECTORS: &[(u32, u8, u16, u64)] = &[
        (0xF2A4D27B, 0xD, 0x0E7, 0x001F2A4D27BDE705),
        (0xAB73738F, 0xD, 0x4EE, 0x001AB73738FDEE40),
        (0xCDBD47D3, 0xB, 0xDC3, 0x001CDBD47D3BC3D4),
        (0xF30B94FA, 0x5, 0x8B4, 0x001F30B94FA5B48E),
        (0x80877B6F, 0x4, 0xE6B, 0x00180877B6F46BE6),
    ];

    #[test]
    fn c_vectors() {
        for &(serial, button, counter, key) in C_VECTORS {
            assert_eq!(build_frame(serial, button, counter), key);
            let got = feed_all(&build_upload(key));
            assert_eq!(got.len(), TOTAL_BURSTS);
            assert!(got.iter().all(|d| d.data == key & !(1 << 52) && d.crc_valid));
            assert_eq!((got[0].serial, got[0].button, got[0].counter), (Some(serial), Some(button & 0x0F), Some(counter)));
        }
    }

    /// encode(): counter + 1 (12 bits) and KAT's generic button mapped to the D-pad codes.
    #[test]
    fn encode_bumps_counter_and_maps_buttons() {
        for (generic, want) in [(0x01u8, 0x1u8), (0x02, 0x2), (0x04, 0x3), (0x08, 0x4), (0x40, 0x9)] {
            let src = parse_frame(build_frame(0x8123_4567, 0x9, 0xFFF));
            let got = feed_all(&KiaV2Decoder::new().encode(&src, generic).unwrap());
            let d = &got[0];
            assert_eq!((d.serial, d.button, d.counter), (Some(0x8123_4567), Some(want), Some(0)));
            assert!(d.crc_valid);
        }
    }

    /// Real fob data: the IMPORTS key files `HYUNDAI/HYU V2 N1.sub` and `HYU V2_random_habar.sub`
    /// (Flipper-ARF "KIA/HYU V2", decoder form without the start bit): CRC valid, and the encoder
    /// rebuilds each frame from its fields.
    #[test]
    fn real_key_files_rebuild() {
        for key in [0x000E_18CC_1429_A2CEu64, 0x000E_18CC_1422_E6B2] {
            let d = parse_frame(key);
            assert!(d.crc_valid);
            let rebuilt = build_frame(d.serial.unwrap(), d.button.unwrap(), d.counter.unwrap());
            assert_eq!(rebuilt & !(1 << 52), key);
        }
    }
}
