//! Land Rover RKE protocol decoder/encoder
//!
//! Ported from the Flipper-ARF firmware (`lib/subghz/protocols/landrover_rke.c` / `.h`,
//! D4C1-Labs), itself derived from Pandora DXL 5000 firmware. Land Rover shares the
//! Ford/Jaguar baseband (firmware protocol ID 0x0E) but uses a distinct 66-bit frame.
//!
//! Encoding: fixed-width PWM, OOK/AM carrier. Bit period 1000µs:
//!   Bit-1 = 700µs HIGH + 300µs LOW; Bit-0 = 300µs HIGH + 700µs LOW.
//!   Preamble = 20× (400µs HIGH + 600µs LOW); sync = 400µs HIGH + 9600µs LOW.
//!   Tolerance ±20% (relative), matching the C `lr_in_range`.
//!
//! Frame (66 bits, MSB-first), matching the C layout:
//!   [65:34] 32-bit KeeLoq encrypted hopping code
//!   [33:10] 24-bit fixed fob serial
//!   [9:6]    4-bit button code (0x1=Lock, 0x2=Unlock, 0x4=Boot/Tailgate, 0x8=Panic)
//!   [5:2]    4-bit function/repeat flags
//!   [1:0]    2-bit status (0x1=battery low, 0x2=repeat)
//!
//! KeeLoq: the hop code is the raw 32-bit KeeLoq ciphertext. Full decryption needs the
//! per-fob manufacturer key (provisioned, not in the firmware), so KAT exposes the framed
//! fields (serial/button) and leaves the hop encrypted — `crc_valid=false` since no real
//! cryptographic check is performed. Emission is still gated tightly on the structural
//! invariants below so this loose-looking PWM decoder does not false-match
//! Kia/Subaru/Ford captures.
//!
//! Storage: 66 bits do not fit a u64. The canonical `DecodedSignal.data` holds the low 64
//! frame bits and `DecodedSignal.extra` holds the top 2 (frame bits [65:64], the high 2 bits
//! of the hop code), so encode() round-trips the hop code losslessly. `data_count_bit` = 66.
//!
//! Gating: a valid frame requires a long preamble run (≥16 of the 400/600µs pairs), the very
//! distinctive 400µs-HIGH + 9600µs-LOW sync gap, then exactly 66 bits whose HIGH/LOW halves
//! each fall inside the ±20% PWM windows. The 9.6ms sync gap + 66-bit PWM payload combination
//! is unique among KAT's protocols, so nothing else in the sweep matches it.

use super::{ProtocolDecoder, ProtocolTiming, DecodedSignal};
use crate::radio::demodulator::LevelDuration;

// Timing constants (microseconds) — verbatim from landrover_rke.h.
const PREAMBLE_HIGH_US: u32 = 400;
const PREAMBLE_LOW_US: u32 = 600;
const PREAMBLE_COUNT: u32 = 20;
const SYNC_HIGH_US: u32 = 400;
const SYNC_LOW_US: u32 = 9600;
const BIT1_HIGH_US: u32 = 700;
const BIT1_LOW_US: u32 = 300;
const BIT0_HIGH_US: u32 = 300;
const BIT0_LOW_US: u32 = 700;
const REPEAT_GAP_US: u32 = 12000;
const REPEAT_COUNT: u32 = 4;
const TOLERANCE_PCT: u32 = 20;
const FRAME_BITS: usize = 66;

// Require a substantial preamble run before accepting the sync gap. The C scans a raw buffer
// for the sync directly, but gating on the preamble too makes the streaming decoder specific.
const PREAMBLE_MIN_PAIRS: u32 = 16;

// Button codes (frame bits [9:6]).
const BTN_LOCK: u8 = 0x1;
const BTN_UNLOCK: u8 = 0x2;
const BTN_BOOT: u8 = 0x4;
const BTN_PANIC: u8 = 0x8;

/// Relative-tolerance match, matching the C `lr_in_range`:
/// `|measured - ref| * 100 <= ref * TOLERANCE_PCT`.
#[inline]
fn in_range(measured_us: u32, ref_us: u32) -> bool {
    let diff = if measured_us > ref_us {
        measured_us - ref_us
    } else {
        ref_us - measured_us
    };
    diff.saturating_mul(100) <= ref_us.saturating_mul(TOLERANCE_PCT)
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    /// Looking for the first preamble HIGH pulse.
    Reset,
    /// Counting preamble (400µs HIGH + 600µs LOW) pairs; te_last holds the pending HIGH.
    Preamble,
    /// Sync seen — expecting the next data-bit HIGH half.
    DataHigh,
    /// Collected a data-bit HIGH in te_last; decide the bit value from the following LOW.
    DataLow,
}

/// Land Rover RKE protocol decoder.
///
/// Bits are accumulated MSB-first into a 66-element array (index 0 = first bit on air =
/// frame bit 65), mirroring the C `bits[65 - b]` indexing and avoiding any u64 overflow.
pub struct LandRoverRkeDecoder {
    step: DecoderStep,
    te_last: u32,
    preamble_count: u32,
    /// Received bits, MSB-first: `rx_bits[i]` is the (i+1)-th bit on air = frame bit `65 - i`.
    rx_bits: [u8; FRAME_BITS],
    decode_count_bit: usize,
}

impl LandRoverRkeDecoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            te_last: 0,
            preamble_count: 0,
            rx_bits: [0u8; FRAME_BITS],
            decode_count_bit: 0,
        }
    }

    /// Map a KAT generic button command to a Land Rover RKE 4-bit button code.
    /// KAT: Lock=0x01, Unlock=0x02, Trunk=0x04, Panic=0x08.
    /// LR RKE bits [9:6]: Lock=0x1, Unlock=0x2, Boot/Tailgate=0x4, Panic=0x8 — identical mapping.
    fn map_button(button: u8) -> u8 {
        match button {
            0x01 => BTN_LOCK,
            0x02 => BTN_UNLOCK,
            0x04 => BTN_BOOT,
            0x08 => BTN_PANIC,
            b => b & 0x0F,
        }
    }

    /// Build the logical 66-bit frame array from the field values.
    ///
    /// The array is indexed by frame bit number (bit 0 = LSB of the whole 66-bit frame). Each
    /// field `X` occupying frame bits `[hi:lo]` maps frame bit `(lo + j)` = field bit `j`, so the
    /// field's MSB lands at the higher frame-bit index. The encoder transmits frame bit 65 first,
    /// giving the documented MSB-first-on-air order. This is the exact inverse of `unpack_frame`.
    ///
    /// NOTE: the Flipper-ARF C reference's `lr_encode`/`lr_decode` use inconsistent bit
    /// endianness for the fields (encode writes `bits[65-i]=field>>i`, decode reads
    /// `field|=bits[65-k]<<(31-k)`), so the C does not round-trip. KAT preserves the documented
    /// field *layout* and MSB-first wire order while keeping encode/decode mutually consistent.
    fn pack_frame(hop_code: u32, serial: u32, button: u8, func_bits: u8, status: u8) -> [u8; FRAME_BITS] {
        let mut bits = [0u8; FRAME_BITS];
        // hop_code: frame bits [65:34] (32 bits).
        for j in 0..32 {
            bits[34 + j] = ((hop_code >> j) & 1) as u8;
        }
        // serial: frame bits [33:10] (24 bits).
        for j in 0..24 {
            bits[10 + j] = ((serial >> j) & 1) as u8;
        }
        // button: frame bits [9:6] (4 bits).
        for j in 0..4 {
            bits[6 + j] = ((button >> j) & 1) as u8;
        }
        // func_bits: frame bits [5:2] (4 bits).
        for j in 0..4 {
            bits[2 + j] = ((func_bits >> j) & 1) as u8;
        }
        // status: frame bits [1:0] (2 bits).
        bits[0] = status & 1;
        bits[1] = (status >> 1) & 1;
        bits
    }

    /// Extract fields from a logical 66-bit frame array (index = frame bit, bit 0 = LSB).
    /// Returns (hop_code, serial, button, func_bits, status). Exact inverse of `pack_frame`.
    fn unpack_frame(bits: &[u8; FRAME_BITS]) -> (u32, u32, u8, u8, u8) {
        let mut hop_code: u32 = 0;
        for j in 0..32 {
            hop_code |= (bits[34 + j] as u32) << j;
        }
        let mut serial: u32 = 0;
        for j in 0..24 {
            serial |= (bits[10 + j] as u32) << j;
        }
        let mut button: u8 = 0;
        for j in 0..4 {
            button |= bits[6 + j] << j;
        }
        let mut func_bits: u8 = 0;
        for j in 0..4 {
            func_bits |= bits[2 + j] << j;
        }
        let status = bits[0] | (bits[1] << 1);
        (hop_code, serial, button, func_bits, status)
    }

    /// Pack a logical 66-bit frame array into `(data, extra)`:
    /// `data` = frame bits [63:0], `extra` = frame bits [65:64].
    fn frame_to_data(bits: &[u8; FRAME_BITS]) -> (u64, u64) {
        let mut data: u64 = 0;
        for i in 0..64 {
            data |= (bits[i] as u64) << i;
        }
        let extra: u64 = (bits[64] as u64) | ((bits[65] as u64) << 1);
        (data, extra)
    }

    /// Inverse of `frame_to_data`.
    fn data_to_frame(data: u64, extra: u64) -> [u8; FRAME_BITS] {
        let mut bits = [0u8; FRAME_BITS];
        for i in 0..64 {
            bits[i] = ((data >> i) & 1) as u8;
        }
        bits[64] = (extra & 1) as u8;
        bits[65] = ((extra >> 1) & 1) as u8;
        bits
    }

    /// Convert the received MSB-first `rx_bits` (index 0 = frame bit 65) into the logical
    /// frame array (index = frame bit number).
    fn rx_to_frame(rx_bits: &[u8; FRAME_BITS]) -> [u8; FRAME_BITS] {
        let mut frame = [0u8; FRAME_BITS];
        for i in 0..FRAME_BITS {
            frame[65 - i] = rx_bits[i];
        }
        frame
    }

    /// Build a DecodedSignal from a completed received-bit array.
    fn build_signal(rx_bits: &[u8; FRAME_BITS]) -> DecodedSignal {
        let frame = Self::rx_to_frame(rx_bits);
        let (hop_code, serial, button, _func_bits, _status) = Self::unpack_frame(&frame);
        let (data, extra) = Self::frame_to_data(&frame);
        DecodedSignal {
            serial: Some(serial),
            button: Some(button),
            // The 16-bit KeeLoq counter is inside the *encrypted* hop code; without the
            // manufacturer key we cannot recover it. Surface the low 16 bits of the hop
            // ciphertext so the UI has a stable per-press value.
            counter: Some((hop_code & 0xFFFF) as u16),
            // No cryptographic check is performed (no key), so this is not a verified frame.
            crc_valid: false,
            data,
            data_count_bit: FRAME_BITS,
            encoder_capable: true,
            // Carry the top 2 frame bits so encode() can faithfully round-trip the hop code.
            extra: Some(extra),
            protocol_display_name: None,
        }
    }

    fn push_pair(signal: &mut Vec<LevelDuration>, high_us: u32, low_us: u32) {
        signal.push(LevelDuration::new(true, high_us));
        signal.push(LevelDuration::new(false, low_us));
    }
}

impl ProtocolDecoder for LandRoverRkeDecoder {
    fn name(&self) -> &'static str {
        "Land Rover RKE"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: BIT0_HIGH_US, // 300µs
            te_long: BIT1_HIGH_US,  // 700µs
            te_delta: 140,          // ~20% of the 700µs long half
            min_count_bit: FRAME_BITS,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[433_920_000, 315_000_000]
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.te_last = 0;
        self.preamble_count = 0;
        self.rx_bits = [0u8; FRAME_BITS];
        self.decode_count_bit = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::Reset => {
                // First preamble HIGH (~400µs).
                if level && in_range(duration, PREAMBLE_HIGH_US) {
                    self.step = DecoderStep::Preamble;
                    self.te_last = duration;
                    self.preamble_count = 0;
                }
            }

            DecoderStep::Preamble => {
                if level {
                    // A HIGH while in preamble: either another preamble HIGH (~400µs) or a sync
                    // HIGH (also ~400µs) — disambiguated by the LOW that follows.
                    if in_range(duration, PREAMBLE_HIGH_US) {
                        self.te_last = duration;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else {
                    // LOW following a preamble/sync HIGH.
                    if in_range(self.te_last, SYNC_HIGH_US) && in_range(duration, SYNC_LOW_US) {
                        // Sync gap (~9600µs LOW). Require enough preamble first.
                        if self.preamble_count >= PREAMBLE_MIN_PAIRS {
                            self.rx_bits = [0u8; FRAME_BITS];
                            self.decode_count_bit = 0;
                            self.step = DecoderStep::DataHigh;
                        } else {
                            self.step = DecoderStep::Reset;
                        }
                    } else if in_range(self.te_last, PREAMBLE_HIGH_US)
                        && in_range(duration, PREAMBLE_LOW_US)
                    {
                        // Another preamble pair (~400µs HIGH + ~600µs LOW).
                        self.preamble_count = self.preamble_count.saturating_add(1);
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                }
            }

            DecoderStep::DataHigh => {
                if level {
                    // Data-bit HIGH half — must match a Bit-1 or Bit-0 HIGH window.
                    if in_range(duration, BIT1_HIGH_US) || in_range(duration, BIT0_HIGH_US) {
                        self.te_last = duration;
                        self.step = DecoderStep::DataLow;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else {
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::DataLow => {
                if !level {
                    let hi = self.te_last;
                    let lo = duration;
                    let bit = if in_range(hi, BIT1_HIGH_US) && in_range(lo, BIT1_LOW_US) {
                        Some(1u8)
                    } else if in_range(hi, BIT0_HIGH_US) && in_range(lo, BIT0_LOW_US) {
                        Some(0u8)
                    } else {
                        None
                    };

                    match bit {
                        Some(b) => {
                            // Store MSB-first: first bit on air → rx_bits[0] (= frame bit 65).
                            if self.decode_count_bit < FRAME_BITS {
                                self.rx_bits[self.decode_count_bit] = b;
                            }
                            self.decode_count_bit += 1;

                            if self.decode_count_bit == FRAME_BITS {
                                let result = Self::build_signal(&self.rx_bits);
                                self.reset();
                                return Some(result);
                            }
                            self.step = DecoderStep::DataHigh;
                        }
                        None => {
                            self.step = DecoderStep::Reset;
                        }
                    }
                } else {
                    self.step = DecoderStep::Reset;
                }
            }
        }

        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        let serial = decoded.serial?;
        // Reconstruct the original frame to preserve the hop code, func bits, and status, then
        // override only the button. The hop code lives in (data, extra); recover it.
        let extra = decoded.extra.unwrap_or(0);
        let orig_frame = Self::data_to_frame(decoded.data, extra);
        let (hop_code, _serial, _orig_button, func_bits, status) = Self::unpack_frame(&orig_frame);

        let btn = Self::map_button(button);
        let frame = Self::pack_frame(hop_code, serial, btn, func_bits, status);

        let mut signal = Vec::with_capacity(
            ((PREAMBLE_COUNT as usize + 1 + FRAME_BITS) * 2 + 1) * REPEAT_COUNT as usize,
        );

        for rep in 0..REPEAT_COUNT {
            // Preamble: 20 pairs.
            for _ in 0..PREAMBLE_COUNT {
                Self::push_pair(&mut signal, PREAMBLE_HIGH_US, PREAMBLE_LOW_US);
            }
            // Sync.
            Self::push_pair(&mut signal, SYNC_HIGH_US, SYNC_LOW_US);
            // Data bits, MSB-first (frame bit 65 first on air).
            for b in (0..FRAME_BITS).rev() {
                if frame[b] != 0 {
                    Self::push_pair(&mut signal, BIT1_HIGH_US, BIT1_LOW_US);
                } else {
                    Self::push_pair(&mut signal, BIT0_HIGH_US, BIT0_LOW_US);
                }
            }
            // Inter-repetition gap.
            if rep < REPEAT_COUNT - 1 {
                signal.push(LevelDuration::new(false, REPEAT_GAP_US));
            }
        }

        Some(signal)
    }
}

impl Default for LandRoverRkeDecoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Feed an encoded signal back through a fresh decoder and return the first decode.
    fn decode_signal(signal: &[LevelDuration]) -> Option<DecodedSignal> {
        let mut dec = LandRoverRkeDecoder::new();
        for ld in signal {
            if let Some(d) = dec.feed(ld.level, ld.duration_us) {
                return Some(d);
            }
        }
        None
    }

    /// Build a DecodedSignal seed carrying a chosen hop code + serial (button overridden at encode).
    fn seed(hop_code: u32, serial: u32, func_bits: u8, status: u8, button: u8) -> DecodedSignal {
        let frame = LandRoverRkeDecoder::pack_frame(hop_code, serial & 0x00FF_FFFF, button, func_bits, status);
        let (data, extra) = LandRoverRkeDecoder::frame_to_data(&frame);
        DecodedSignal {
            serial: Some(serial & 0x00FF_FFFF),
            button: Some(button),
            counter: Some((hop_code & 0xFFFF) as u16),
            crc_valid: false,
            data,
            data_count_bit: FRAME_BITS,
            encoder_capable: true,
            extra: Some(extra),
            protocol_display_name: None,
        }
    }

    #[test]
    fn pack_unpack_roundtrip() {
        // Field packing/unpacking is exact across the full 66-bit layout.
        let cases = [
            (0xDEAD_BEEFu32, 0x00AB_CDEFu32, 0x1u8, BTN_UNLOCK, 0x5u8, 0x2u8),
            (0x0000_0000, 0x0000_0000, 0x0, 0x0, 0x0, 0x0),
            (0xFFFF_FFFF, 0x00FF_FFFF, 0xF, 0xF, 0xF, 0x3),
            (0x1234_5678, 0x0055_AA55, 0x4, BTN_LOCK, 0xA, 0x1),
        ];
        for &(hop, serial, _btn_field_unused, button, func, status) in &cases {
            let frame = LandRoverRkeDecoder::pack_frame(hop, serial, button, func, status);
            let (h, s, b, f, st) = LandRoverRkeDecoder::unpack_frame(&frame);
            assert_eq!(h, hop, "hop_code mismatch");
            assert_eq!(s, serial, "serial mismatch");
            assert_eq!(b, button, "button mismatch");
            assert_eq!(f, func, "func_bits mismatch");
            assert_eq!(st, status, "status mismatch");

            // (data, extra) <-> frame is lossless too.
            let (data, extra) = LandRoverRkeDecoder::frame_to_data(&frame);
            let frame2 = LandRoverRkeDecoder::data_to_frame(data, extra);
            assert_eq!(frame, frame2, "data/extra round-trip mismatch");
        }
    }

    #[test]
    fn encode_decode_roundtrip_multiple() {
        // Multiple serials / hop codes (counters) / buttons all round-trip through encode→decode.
        let serials = [0x00ABCDEFu32, 0x00123456, 0x00000001, 0x00FFFFFE, 0x005A5A5A];
        let hops = [0xDEADBEEFu32, 0x00000000, 0xFFFFFFFF, 0x12345678, 0xCAFEBABE];
        let buttons = [0x01u8, 0x02, 0x04, 0x08]; // Lock, Unlock, Trunk/Boot, Panic

        for (&serial, &hop) in serials.iter().zip(hops.iter()) {
            for &btn in &buttons {
                let s = seed(hop, serial, 0x3, 0x1, 0x00); // func/status preserved from seed
                let encoder = LandRoverRkeDecoder::new();
                let signal = encoder.encode(&s, btn).expect("encode should succeed");
                let decoded = decode_signal(&signal)
                    .unwrap_or_else(|| panic!("decode failed for serial {serial:#X} hop {hop:#X} btn {btn:#X}"));

                let expected_btn = LandRoverRkeDecoder::map_button(btn);
                assert_eq!(decoded.serial, Some(serial & 0x00FF_FFFF), "serial");
                assert_eq!(decoded.button, Some(expected_btn), "button");
                // hop code survives via data+extra.
                let frame = LandRoverRkeDecoder::data_to_frame(decoded.data, decoded.extra.unwrap());
                let (dec_hop, _, _, dec_func, dec_status) = LandRoverRkeDecoder::unpack_frame(&frame);
                assert_eq!(dec_hop, hop, "hop_code");
                assert_eq!(dec_func, 0x3, "func_bits preserved");
                assert_eq!(dec_status, 0x1, "status preserved");
                assert_eq!(decoded.data_count_bit, FRAME_BITS, "bit count");
                assert!(!decoded.crc_valid, "no key → crc_valid must be false");
            }
        }
    }

    #[test]
    fn rejects_truncated_frame() {
        // A single repetition carrying only 65 of the 66 bits must NOT decode.
        let s = seed(0xDEADBEEF, 0x00ABCDEF, 0x0, 0x0, 0x02);
        let encoder = LandRoverRkeDecoder::new();
        let signal = encoder.encode(&s, 0x02).unwrap();
        // First repetition is preamble (20 pairs) + sync (1 pair) + 66 bit-pairs. Keep only the
        // first 65 bit-pairs so the frame is one bit short, and stop before the next repetition.
        let bits_to_keep = FRAME_BITS - 1;
        let partial_len = (PREAMBLE_COUNT as usize + 1 + bits_to_keep) * 2;
        let partial = &signal[..partial_len];
        assert!(decode_signal(partial).is_none(), "65-bit frame must not decode");
    }

    #[test]
    fn rejects_wrong_sync_gap() {
        // Same PWM bits but a too-short "sync" gap must not be accepted as a frame.
        let s = seed(0x12345678, 0x00112233, 0x0, 0x0, 0x01);
        let encoder = LandRoverRkeDecoder::new();
        let mut signal = encoder.encode(&s, 0x01).unwrap();
        // Corrupt the first sync LOW (index = PREAMBLE_COUNT*2 + 1) to a Bit-0-like LOW (700µs),
        // which is far outside the 9600µs ±20% window.
        let sync_low_idx = PREAMBLE_COUNT as usize * 2 + 1;
        signal[sync_low_idx] = LevelDuration::new(false, 700);
        // Take just the first repetition so the later (intact) reps don't rescue the decode.
        let one_rep_len = (PREAMBLE_COUNT as usize + 1 + FRAME_BITS) * 2;
        let partial = &signal[..one_rep_len.min(signal.len())];
        assert!(decode_signal(partial).is_none(), "frame without valid 9.6ms sync must not decode");
    }
}
