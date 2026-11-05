//! Helpers shared by the G4 garage/shutter protocols (Security+ 1.0/2.0, Somfy Telis/Keytis).
//!
//! - [`reverse_key`]: `subghz_protocol_blocks_reverse_key` (`lib/subghz/blocks/math.c`), used by
//!   both Security+ versions for their bit-reversed rolling codes.
//! - [`push`]: append a level to an upload, merging it with the previous entry when the level
//!   repeats (identical on air: adjacent same-level durations merge).
//! - [`ManchesterEncoder`]: an exact port of Flipper's `lib/toolbox/manchester_encoder.c`
//!   (`manchester_encoder_advance` / `manchester_encoder_finish`), used by Security+ 2.0.
//! - Somfy RTS frame helpers shared by Telis and Keytis: the 4-bit checksum, the XOR-chain
//!   obfuscation, the decoder's CRC check and the encoders' Manchester bit emission.

use crate::radio::demodulator::LevelDuration;

/// `subghz_protocol_blocks_reverse_key`: the low `bit_count` bits of `key`, in reverse order.
pub(super) fn reverse_key(key: u64, bit_count: u8) -> u64 {
    (0..bit_count).fold(0u64, |acc, i| (acc << 1) | ((key >> i) & 1))
}

/// Append one level, merging it with the previous entry when the level repeats.
pub(super) fn push(out: &mut Vec<LevelDuration>, level: bool, duration: u32) {
    match out.last_mut() {
        Some(last) if last.level == level => last.duration_us += duration,
        _ => out.push(LevelDuration::new(level, duration)),
    }
}

/// `ManchesterEncoderResult` (manchester_encoder.h): `(level, long)` of one upload entry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum ManchesterResult {
    ShortLow = 0b00,
    LongLow = 0b01,
    LongHigh = 0b10,
    ShortHigh = 0b11,
}

impl ManchesterResult {
    fn from_bits(bits: u8) -> Self {
        match bits & 0b11 {
            0b00 => Self::ShortLow,
            0b01 => Self::LongLow,
            0b10 => Self::LongHigh,
            _ => Self::ShortHigh,
        }
    }

    /// Upload entry for this result (`te_short` / `te_long` per the protocol).
    pub(super) fn level_duration(self, te_short: u32, te_long: u32) -> (bool, u32) {
        match self {
            Self::ShortLow => (false, te_short),
            Self::LongLow => (false, te_long),
            Self::LongHigh => (true, te_long),
            Self::ShortHigh => (true, te_short),
        }
    }
}

/// `ManchesterEncoderState` + `manchester_encoder_advance` / `manchester_encoder_finish`.
/// Flipper's convention: a 1 bit is a low half then a high half, a 0 bit high then low.
#[derive(Debug, Clone, Copy, Default)]
pub(super) struct ManchesterEncoder {
    prev_bit: bool,
    step: u8,
}

impl ManchesterEncoder {
    /// `manchester_encoder_reset`.
    pub(super) fn new() -> Self {
        Self::default()
    }

    /// `manchester_encoder_advance`: returns `(advanced, result)`. When `advanced` is false the
    /// caller emits `result` and calls again with the same bit (as every C caller does).
    pub(super) fn advance(&mut self, bit: bool) -> (bool, ManchesterResult) {
        match self.step {
            0 => {
                self.prev_bit = bit;
                self.step = 1;
                (
                    true,
                    if bit {
                        ManchesterResult::ShortLow
                    } else {
                        ManchesterResult::ShortHigh
                    },
                )
            }
            1 => {
                let result = ManchesterResult::from_bits(((self.prev_bit as u8) << 1) + bit as u8);
                if bit == self.prev_bit {
                    self.step = 2;
                    (false, result)
                } else {
                    self.prev_bit = bit;
                    (true, result)
                }
            }
            _ => {
                self.prev_bit = bit;
                self.step = 1;
                (
                    true,
                    if bit {
                        ManchesterResult::ShortLow
                    } else {
                        ManchesterResult::ShortHigh
                    },
                )
            }
        }
    }

    /// The C callers' loop body: emit the result(s) for one bit.
    pub(super) fn encode_bit(&mut self, bit: bool, mut emit: impl FnMut(ManchesterResult)) {
        let (advanced, result) = self.advance(bit);
        if advanced {
            emit(result);
        } else {
            emit(result);
            let (_, result) = self.advance(bit);
            emit(result);
        }
    }

    /// `manchester_encoder_finish`: the second half of the last bit.
    pub(super) fn finish(&mut self) -> ManchesterResult {
        self.step = 0;
        ManchesterResult::from_bits(((self.prev_bit as u8) << 1) + self.prev_bit as u8)
    }
}

/// Somfy RTS `te_short` (half bit) — Telis and Keytis use the same timing.
pub(super) const SOMFY_TE_SHORT: u32 = 640;
/// Somfy RTS `te_long`.
pub(super) const SOMFY_TE_LONG: u32 = 1280;
/// Somfy RTS `te_delta`.
pub(super) const SOMFY_TE_DELTA: u32 = 250;
/// Wake-up pulse and the gap after it (both encoders).
pub(super) const SOMFY_WAKEUP_HIGH: u32 = 9415;
pub(super) const SOMFY_WAKEUP_LOW: u32 = 89565;
/// Software sync HIGH.
pub(super) const SOMFY_SOFT_SYNC: u32 = 4550;
/// Inter-frame silence at the end of a frame.
pub(super) const SOMFY_SILENCE: u32 = 30415;

/// `subghz_protocol_somfy_*_crc`: XOR of every nibble of the de-obfuscated 56-bit frame with the
/// checksum nibble (bits 43..40) cleared.
pub(super) fn somfy_crc(data: u64) -> u8 {
    let data = data & 0x00FF_F0FF_FFFF_FFFF;
    let mut crc: u8 = 0;
    for i in (0..56).step_by(8) {
        crc ^= (data >> i) as u8 ^ (data >> (i + 4)) as u8;
    }
    crc & 0xF
}

/// The decoders' integrity check on a received (obfuscated) 56-bit frame: the checksum nibble of
/// the de-obfuscated frame (`data ^ data >> 8`, bits 43..40) equals [`somfy_crc`] of it.
pub(super) fn somfy_crc_ok(frame: u64) -> bool {
    let plain = frame ^ (frame >> 8);
    ((plain >> 40) & 0xF) as u8 == somfy_crc(plain)
}

/// The encoders' frame build (`*_gen_data` after the fields are set): checksum over the 7 plain
/// bytes into `frame[1]`'s low nibble, then the XOR chain `frame[i] ^= frame[i - 1]`; returns the
/// 56-bit obfuscated frame, big-endian.
pub(super) fn somfy_obfuscate(mut frame: [u8; 7]) -> u64 {
    let checksum = frame.iter().fold(0u8, |c, &b| c ^ b ^ (b >> 4)) & 0xF;
    frame[1] |= checksum;
    for i in 1..7 {
        frame[i] ^= frame[i - 1];
    }
    frame.iter().fold(0u64, |acc, &b| (acc << 8) | b as u64)
}

/// The Somfy encoders' Manchester data emission (`bit_read(data, i - 1)` for `i = bits..1`):
/// a 1 is LOW then HIGH, a 0 HIGH then LOW; when the previous entry already has the first half's
/// level the C doubles it (`duration *= 2`), which [`push`]'s merge reproduces because that entry
/// is always one `te_short` long.
pub(super) fn somfy_push_bits(out: &mut Vec<LevelDuration>, data: u64, bits: u32) {
    for i in (0..bits).rev() {
        let one = (data >> i) & 1 == 1;
        push(out, !one, SOMFY_TE_SHORT);
        push(out, one, SOMFY_TE_SHORT);
    }
}

/// The Somfy encoders' inter-frame silence: added to the last LOW, or a new LOW entry.
pub(super) fn somfy_push_silence(out: &mut Vec<LevelDuration>, duration: u32) {
    push(out, false, duration);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reverse_key_matches_c() {
        assert_eq!(reverse_key(0xE600_0000, 32), 0x67);
        assert_eq!(reverse_key(0x1, 28), 0x800_0000);
        assert_eq!(reverse_key(0xABCD, 0), 0);
        assert_eq!(reverse_key(u64::MAX, 64), u64::MAX);
    }

    /// The C encoder's output for a few bit strings, written out by hand from manchester_encoder.c.
    #[test]
    fn manchester_encoder_matches_c() {
        let run = |bits: &[bool]| {
            let mut enc = ManchesterEncoder::new();
            let mut out = Vec::new();
            for &b in bits {
                enc.encode_bit(b, |r| out.push(r));
            }
            out.push(enc.finish());
            out
        };
        use ManchesterResult::*;
        // 1 1: L H L H.
        assert_eq!(
            run(&[true, true]),
            [ShortLow, ShortHigh, ShortLow, ShortHigh]
        );
        // 1 0: L, H+H, L.
        assert_eq!(run(&[true, false]), [ShortLow, LongHigh, ShortLow]);
        // 0 1 0: H, L+L, H+H, L.
        assert_eq!(
            run(&[false, true, false]),
            [ShortHigh, LongLow, LongHigh, ShortLow]
        );
    }

    /// Telis example from somfy_telis.c's comment: 0xA7232323312222 de-obfuscates to key A7, button
    /// 8, counter 0000, address 12 13 00 (the comment shows checksum 0; the frame carries 4, which
    /// is what the C's own checksum formula gives).
    #[test]
    fn somfy_frame_helpers_match_the_c_comment() {
        let frame = 0xA7_2323_2331_2222u64;
        assert!(somfy_crc_ok(frame));
        assert_eq!(frame ^ (frame >> 8), 0xA7_8400_0012_1300);
        assert_eq!(
            somfy_obfuscate([0xA7, 0x80, 0x00, 0x00, 0x12, 0x13, 0x00]),
            frame
        );
        assert!(!somfy_crc_ok(frame ^ 0x10));
    }
}
