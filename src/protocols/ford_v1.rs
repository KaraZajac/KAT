//! Ford V1 protocol decoder/encoder
//!
//! Aligned with ProtoPirate reference: `REFERENCES/ProtoPirate/protocols/ford_v1.c` and `ford_v1.h`.
//! Manchester 65/130µs (te_delta 39), FM. 136 bits / 17 bytes: key1 (bytes 0..7, 56 bits) +
//! key2 (bytes 7..15, 64 bits) + CRC16 (bytes 15..16). Preamble ≥50 long pulses, then a short-pulse
//! sync window (`sync_event_count > 2`) replays buffered Manchester events and enters the 17-byte
//! data collection. This is a ROLLING-code protocol.
//!
//! Crypto: a proprietary parity-based descrambling cipher (`ford_v1_decode_with_flag`) operating on
//! the 9-byte air block `raw[6..15]`, plus CRC16/CCITT (poly 0x1021, init 0x0000) over `raw[3..15]`.
//! Emission is gated on CRC16 validity (with a 17-byte bit-inverted fallback) so it never
//! false-matches. Encryption/rolling detection mirrors the C: a strict branch
//! (`decoded[3]==raw[5] && decoded[4]==raw[6]`) yields plaintext serial/button/counter; otherwise an
//! encode round-trip check classifies it as encrypted/rolling. Encoder supported (6 bursts).
//!
//! Manchester transition table is the Flipper differential-Manchester table (same as Ford V0/V2).

use super::{ProtocolDecoder, ProtocolTiming, DecodedSignal};
use crate::radio::demodulator::LevelDuration;
use crate::duration_diff;

const TE_SHORT: u32 = 65;
const TE_LONG: u32 = 130;
const TE_DELTA: u32 = 39;
const DATA_BITS: usize = 136;
const DATA_BYTES: usize = 17;
const PREAMBLE_MIN: u16 = 50;
/// C uses FORD_V1_DELTA_LONG (40) only for the preamble long-pulse match; data/sync use te_delta (39).
const DELTA_LONG: u32 = 40;
const SILENCE_LONG_MULT: u32 = 3;

// Manchester event encoding (matches Flipper ManchesterEvent ordinals used in the C buffer):
// 0 = ShortLow, 1 = ShortHigh, 2 = LongLow, 3 = LongHigh.
const EV_SHORT_LOW: u8 = 0;
const EV_SHORT_HIGH: u8 = 1;
const EV_LONG_LOW: u8 = 2;
const EV_LONG_HIGH: u8 = 3;

// Encoder constants (subghz_protocol_encoder_ford_v1).
const ENC_BURST_COUNT: usize = 6;
const ENC_PREAMBLE_PAIRS: usize = 400;
const ENC_SYNC_SHORT_US: u32 = 65;
const ENC_SYNC_LONG_US: u32 = 130;
const ENC_GAP_REPEAT_US: u32 = 50000;
const ENC_GAP_LAST_US: u32 = 260;
/// Per-burst override of pkt[4] (matches ford_v1_encoder_burst_pkt4_vals).
const ENC_BURST_PKT4: [u8; 6] = [0x08, 0x00, 0x10, 0x08, 0x00, 0x10];

#[derive(Debug, Clone, Copy, PartialEq)]
enum ManchesterState {
    Mid0 = 0,
    Mid1 = 1,
    Start0 = 2,
    Start1 = 3,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Preamble,
    Sync,
    Data,
}

pub struct FordV1Decoder {
    step: DecoderStep,
    manchester_state: ManchesterState,
    preamble_count: u16,
    decode_data: u64,
    decode_count_bit: usize,
    byte_count: usize,
    raw_bytes: [u8; DATA_BYTES + 1],
    sync_event_idx: u8,
    sync_event_count: u8,
    sync_events: [u8; 8],
}

impl FordV1Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            manchester_state: ManchesterState::Mid1,
            preamble_count: 0,
            decode_data: 0,
            decode_count_bit: 0,
            byte_count: 0,
            raw_bytes: [0; DATA_BYTES + 1],
            sync_event_idx: 0,
            sync_event_count: 0,
            sync_events: [0; 8],
        }
    }

    fn reset_state(&mut self) {
        self.step = DecoderStep::Reset;
        self.manchester_state = ManchesterState::Mid1;
        self.preamble_count = 0;
        self.decode_data = 0;
        self.decode_count_bit = 0;
        self.byte_count = 0;
        self.raw_bytes = [0; DATA_BYTES + 1];
        self.sync_event_idx = 0;
        self.sync_event_count = 0;
        self.sync_events = [0; 8];
    }

    /// Short/long matchers. Data and sync use te_delta (39); preamble-long uses DELTA_LONG (40).
    fn is_short(d: u32) -> bool {
        duration_diff!(d, TE_SHORT) < TE_DELTA
    }
    fn is_long(d: u32) -> bool {
        duration_diff!(d, TE_LONG) < TE_DELTA
    }
    fn is_preamble_long(d: u32) -> bool {
        duration_diff!(d, TE_LONG) < DELTA_LONG
    }

    /// Flipper differential-Manchester transition table (same as Ford V0/V2/Kia V7).
    /// Event: 0=ShortLow,1=ShortHigh,2=LongLow,3=LongHigh. Returns Some(bit) when a bit emits.
    fn manchester_advance(&mut self, event: u8) -> Option<bool> {
        let (new_state, emit) = match (self.manchester_state, event) {
            (ManchesterState::Mid0, 0) => (ManchesterState::Mid0, false),
            (ManchesterState::Mid0, 1) => (ManchesterState::Start1, true),
            (ManchesterState::Mid0, 2) => (ManchesterState::Mid0, false),
            (ManchesterState::Mid0, 3) => (ManchesterState::Mid1, true),

            (ManchesterState::Mid1, 0) => (ManchesterState::Start0, true),
            (ManchesterState::Mid1, 1) => (ManchesterState::Mid1, false),
            (ManchesterState::Mid1, 2) => (ManchesterState::Mid0, true),
            (ManchesterState::Mid1, 3) => (ManchesterState::Mid1, false),

            (ManchesterState::Start0, 0) => (ManchesterState::Mid0, false),
            (ManchesterState::Start0, 1) => (ManchesterState::Mid0, false),
            (ManchesterState::Start0, 2) => (ManchesterState::Mid0, false),
            (ManchesterState::Start0, 3) => (ManchesterState::Mid1, false),

            (ManchesterState::Start1, 0) => (ManchesterState::Mid0, false),
            (ManchesterState::Start1, 1) => (ManchesterState::Mid1, false),
            (ManchesterState::Start1, 2) => (ManchesterState::Mid0, false),
            (ManchesterState::Start1, 3) => (ManchesterState::Mid1, false),

            _ => (ManchesterState::Mid1, false),
        };
        self.manchester_state = new_state;
        if emit { Some((event & 1) == 1) } else { None }
    }

    /// Push a decoded Manchester bit into the byte buffer (matches C feed/data byte assembly).
    fn push_bit(&mut self, data_bit: bool) {
        self.decode_data = (self.decode_data << 1) | (data_bit as u64);
        self.decode_count_bit += 1;
        if self.decode_count_bit & 7 == 0 {
            let byte_val = (self.decode_data & 0xFF) as u8;
            if self.byte_count < DATA_BYTES {
                self.raw_bytes[self.byte_count] = byte_val;
                self.byte_count += 1;
            }
            self.decode_data = 0;
        }
    }

    // =========================================================================
    // CRC16/CCITT (poly 0x1021, init 0x0000) — module-local, matches
    // subghz_protocol_blocks_crc16(data, len, 0x1021, 0x0000).
    // =========================================================================
    fn crc16(data: &[u8]) -> u16 {
        let mut crc: u16 = 0x0000;
        for &byte in data {
            crc ^= (byte as u16) << 8;
            for _ in 0..8 {
                if crc & 0x8000 != 0 {
                    crc = (crc << 1) ^ 0x1021;
                } else {
                    crc <<= 1;
                }
            }
        }
        crc
    }

    // =========================================================================
    // Descrambling cipher (ford_v1_decode_with_flag) — operates on the 9-byte
    // air block in place. Matches the C exactly.
    // =========================================================================
    fn decode_with_flag(raw: &mut [u8; 9], flag_byte: u8) {
        if flag_byte != 0 {
            let xor_byte = raw[7];
            for i in 1..7 {
                raw[i] ^= xor_byte;
            }
        } else {
            let xor_byte = raw[6];
            for i in 1..6 {
                raw[i] ^= xor_byte;
            }
            raw[7] ^= xor_byte;
        }

        let b6 = raw[6];
        let b7 = raw[7];
        raw[6] = (b6 & 0xAA) | (b7 & 0x55);
        raw[7] = (b7 & 0xAA) | (b6 & 0x55);
    }

    /// Parity-driven descramble used when neither strict branch matches (ford_v1_decode).
    fn decode_air(raw: &mut [u8; 9]) {
        let endbyte = raw[8];
        let parity_any = endbyte != 0;
        let mut parity = 0u8;
        let mut tmp = endbyte;
        while tmp != 0 {
            parity ^= tmp & 1;
            tmp >>= 1;
        }
        let flag_byte = if parity_any { parity } else { 0 };
        Self::decode_with_flag(raw, flag_byte);
    }

    // =========================================================================
    // Inverse cipher (encoder side) — ford_v1_encode_inverse_block. Takes a
    // 9-byte plaintext block and produces the air (scrambled) block in place.
    // =========================================================================
    fn encode_inverse_block(block: &mut [u8; 9]) {
        let mut sum: u8 = 0;
        for i in 1..=7 {
            sum = sum.wrapping_add(block[i]);
        }

        let p6 = block[6];
        let p7 = block[7];
        let post6 = (p6 & 0xAA) | (p7 & 0x55);
        let post7 = (p7 & 0xAA) | (p6 & 0x55);
        let xorv = post6 ^ post7;

        let xor_byte;
        if (sum.count_ones() & 1) != 0 {
            block[6] = xorv;
            block[7] = post7;
            xor_byte = post7;
        } else {
            block[6] = post6;
            block[7] = xorv;
            xor_byte = post6;
        }

        for i in 1..=5 {
            block[i] ^= xor_byte;
        }
    }

    fn encode_air_9bytes(plain9: &[u8; 9]) -> [u8; 9] {
        let mut block = *plain9;
        Self::encode_inverse_block(&mut block);
        block
    }

    /// Recover plaintext from an air block by trying both descramble flags and verifying
    /// the result re-encodes to the same air bytes (ford_v1_plain_from_air).
    fn plain_from_air(air9: &[u8; 9]) -> Option<[u8; 9]> {
        for flag in 0u8..2 {
            let mut cand = *air9;
            Self::decode_with_flag(&mut cand, flag);
            let reair = Self::encode_air_9bytes(&cand);
            if reair == *air9 {
                return Some(cand);
            }
        }
        None
    }

    /// Extract serial/button/counter from a plaintext 9-byte block (ford_v1_fields_from_plain).
    fn fields_from_plain(plain9: &[u8; 9]) -> (u32, u8, u32) {
        let serial = ((plain9[1] as u32) << 24)
            | ((plain9[2] as u32) << 16)
            | ((plain9[3] as u32) << 8)
            | (plain9[0] as u32);
        let btn = (plain9[5] >> 4) & 0x0F;
        let cnt = (((plain9[5] & 0x0F) as u32) << 16) | ((plain9[6] as u32) << 8) | (plain9[7] as u32);
        (serial, btn, cnt)
    }

    // =========================================================================
    // process_data (ford_v1_process_data): CRC16 gate (+ 17-byte inverted
    // fallback), descramble, field extraction, dual-branch classification.
    // Returns Some(DecodedSignal) when CRC passes.
    // =========================================================================
    fn process_data(&self) -> Option<DecodedSignal> {
        let mut raw = [0u8; DATA_BYTES];
        raw.copy_from_slice(&self.raw_bytes[..DATA_BYTES]);

        let mut calc_crc = Self::crc16(&raw[3..15]);
        let mut recv_crc = ((raw[15] as u16) << 8) | raw[16] as u16;

        // Fallback: bit-invert all 17 bytes and retry the CRC (matches C).
        if recv_crc != calc_crc {
            for (i, b) in raw.iter_mut().enumerate() {
                *b = !self.raw_bytes[i];
            }
            calc_crc = Self::crc16(&raw[3..15]);
            recv_crc = ((raw[15] as u16) << 8) | raw[16] as u16;
        }

        if recv_crc != calc_crc {
            return None;
        }

        // Air block = raw[6..15] (9 bytes). Try both descramble branches "strictly".
        let mut air9 = [0u8; 9];
        air9.copy_from_slice(&raw[6..15]);

        let mut decoded_b0 = air9;
        Self::decode_with_flag(&mut decoded_b0, 0);
        let mut decoded_b1 = air9;
        Self::decode_with_flag(&mut decoded_b1, 1);

        let (decoded, strict_ok): ([u8; 9], bool) =
            if decoded_b0[3] == raw[5] && decoded_b0[4] == raw[6] {
                (decoded_b0, true)
            } else if decoded_b1[3] == raw[5] && decoded_b1[4] == raw[6] {
                (decoded_b1, true)
            } else if let Some(p) = Self::plain_from_air(&air9) {
                // Encrypted/rolling: round-trip recovered but not a strict cleartext match.
                (p, false)
            } else {
                let mut p = air9;
                Self::decode_air(&mut p);
                (p, false)
            };

        let recalc_crc = Self::crc16(&raw[3..15]);

        // key1 = raw[0..7] (56 bits, big-endian) → DecodedSignal.data.
        let mut key1: u64 = 0;
        for &b in raw.iter().take(7) {
            key1 = (key1 << 8) | b as u64;
        }

        let (serial, button, counter) = if strict_ok {
            let (s, b, c) = Self::fields_from_plain(&decoded);
            (s, b, c)
        } else {
            // Header-only: device id from raw[3..7], no button/counter (matches C).
            let device_id = ((raw[3] as u32) << 24)
                | ((raw[4] as u32) << 16)
                | ((raw[5] as u32) << 8)
                | (raw[6] as u32);
            (device_id, 0u8, 0u32)
        };

        // Stash everything the encoder needs to rebuild the full 17-byte frame from fields.
        // `data` already carries key1 = raw[0..7] (56 bits). The air block raw[6..15] and the
        // CRC bytes are regenerated by re-encoding the plaintext, which is fully determined by
        // serial/button/counter EXCEPT plain[4] (the byte the strict branch constrains, equal to
        // air9[0]) and the derived checksum plain[8]. So we stash plain[4] plus the strict flag.
        // Layout (u64, MSB first):
        //   [63:48]=crc16  [47:40]=strict_ok  [39:32]=plain[4]  [31:0]=0 (reserved).
        let plain4 = decoded[4];
        let extra = ((recalc_crc as u64) << 48)
            | ((strict_ok as u64) << 40)
            | ((plain4 as u64) << 32);

        Some(DecodedSignal {
            serial: Some(serial),
            button: Some(button),
            counter: Some(counter as u16),
            crc_valid: true,
            data: key1,
            data_count_bit: DATA_BITS,
            encoder_capable: true,
            extra: Some(extra),
            protocol_display_name: None,
        })
    }

    /// Map KAT's generic button command to a Ford V1 4-bit button code
    /// (Sync=0, Lock=1, Unlock=2, Trunk=4, Panic=8 — matches ford_v1_get_button_name).
    fn map_button(button: u8) -> u8 {
        match button {
            0x01 => 0x01, // Lock
            0x02 => 0x02, // Unlock
            0x04 => 0x04, // Trunk
            0x08 => 0x08, // Panic
            b => b & 0x0F,
        }
    }

    /// Apply serial/button/counter onto a plaintext 9-byte block (ford_v1_plain_apply_fields).
    fn plain_apply_fields(plain9: &mut [u8; 9], serial: u32, btn: u8, cnt: u32) {
        let chk = plain9[8]
            .wrapping_sub(plain9[6])
            .wrapping_sub(plain9[7])
            .wrapping_sub(plain9[5]);
        plain9[0] = (serial & 0xFF) as u8;
        plain9[1] = ((serial >> 24) & 0xFF) as u8;
        plain9[2] = ((serial >> 16) & 0xFF) as u8;
        plain9[3] = ((serial >> 8) & 0xFF) as u8;
        plain9[5] = (((btn & 0x0F) << 4) | (((cnt >> 16) & 0x0F) as u8)) as u8;
        plain9[6] = ((cnt >> 8) & 0xFF) as u8;
        plain9[7] = (cnt & 0xFF) as u8;
        plain9[8] = chk
            .wrapping_add(plain9[7])
            .wrapping_add(plain9[6])
            .wrapping_add(plain9[5]);
    }

    /// Rebuild the air block (raw[6..15]) and CRC16 (raw[15..17]) from a plaintext block
    /// (ford_v1_encoder_rebuild_raw_from_plain).
    fn rebuild_raw_from_plain(raw17: &mut [u8; DATA_BYTES], plain9: &[u8; 9]) {
        let air9 = Self::encode_air_9bytes(plain9);
        raw17[6..15].copy_from_slice(&air9);
        let c = Self::crc16(&raw17[3..15]);
        raw17[15] = (c >> 8) as u8;
        raw17[16] = (c & 0xFF) as u8;
    }

    fn enc_add_level(signal: &mut Vec<LevelDuration>, level: bool, duration: u32) {
        if let Some(last) = signal.last_mut() {
            if last.level == level {
                *last = LevelDuration::new(level, last.duration_us + duration);
                return;
            }
        }
        signal.push(LevelDuration::new(level, duration));
    }
}

impl ProtocolDecoder for FordV1Decoder {
    fn name(&self) -> &'static str {
        "Ford V1"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: DATA_BITS,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.reset_state();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            // C: !level && long → Preamble; seed preamble_count = 1.
            DecoderStep::Reset => {
                if !level && Self::is_preamble_long(duration) {
                    self.step = DecoderStep::Preamble;
                    self.preamble_count = 1;
                }
            }

            DecoderStep::Preamble => {
                if Self::is_preamble_long(duration) {
                    self.preamble_count = self.preamble_count.saturating_add(1);
                } else if Self::is_short(duration) {
                    if self.preamble_count >= PREAMBLE_MIN {
                        // Enter Sync: buffer the first short event.
                        self.sync_event_idx = 0;
                        self.sync_event_count = 1;
                        self.sync_events[0] = if level { EV_SHORT_HIGH } else { EV_SHORT_LOW };
                        self.step = DecoderStep::Sync;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else if self.preamble_count < PREAMBLE_MIN {
                    self.step = DecoderStep::Reset;
                }
                // else: stay in Preamble (long preamble already satisfied, ignore stray pulse)
            }

            DecoderStep::Sync => {
                let (ev, is_short) = if Self::is_short(duration) {
                    (if level { EV_SHORT_HIGH } else { EV_SHORT_LOW }, true)
                } else if Self::is_long(duration) {
                    (if level { EV_LONG_HIGH } else { EV_LONG_LOW }, false)
                } else {
                    self.step = DecoderStep::Preamble;
                    return None;
                };

                self.sync_event_idx += 1;
                if is_short {
                    self.sync_event_count += 1;
                }
                if (self.sync_event_idx as usize) < 8 {
                    self.sync_events[self.sync_event_idx as usize] = ev;
                }

                if self.sync_event_count > 2 {
                    // Sync detected: reset the bit buffer and replay buffered events into Manchester.
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.byte_count = 0;
                    self.raw_bytes = [0; DATA_BYTES + 1];
                    self.manchester_state = ManchesterState::Mid1;
                    if self.sync_events[0] == EV_SHORT_LOW {
                        self.manchester_state = ManchesterState::Mid0;
                    }
                    self.step = DecoderStep::Data;

                    let last = self.sync_event_idx.min(7);
                    for i in 0..=last {
                        let event = self.sync_events[i as usize];
                        if let Some(data_bit) = self.manchester_advance(event) {
                            self.push_bit(data_bit);
                        }
                    }
                    return None;
                }

                if self.sync_event_idx >= 7 {
                    self.step = DecoderStep::Preamble;
                }
            }

            DecoderStep::Data => {
                let event = if Self::is_short(duration) {
                    if level { EV_SHORT_HIGH } else { EV_SHORT_LOW }
                } else if Self::is_long(duration) {
                    if level { EV_LONG_HIGH } else { EV_LONG_LOW }
                } else {
                    // Idle gap / odd pulse. The C decoder attempts partial-last-byte variants only
                    // when byte_count==16 and 1-2 bits short; we require all 17 bytes, so any
                    // non-short/long pulse (including very long inter-burst gaps, dur >=
                    // te_long*SILENCE_LONG_MULT) ends the attempt and resets so the next burst can
                    // re-sync from Reset.
                    let _ = duration >= TE_LONG * SILENCE_LONG_MULT;
                    self.reset_state();
                    return None;
                };

                if let Some(data_bit) = self.manchester_advance(event) {
                    self.push_bit(data_bit);

                    if self.byte_count > 16 {
                        let result = self.process_data();
                        self.reset_state();
                        return result;
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
        let serial = decoded.serial?;
        let counter = decoded.counter.unwrap_or(0) as u32 & 0xF_FFFF;
        let btn = Self::map_button(button) & 0x0F;

        // We can faithfully re-encode only when the original frame's plaintext was recovered at
        // decode time (strict branch). `extra` carries: [63:48]=crc16, [47:40]=strict_ok,
        // [39:32]=plain[4] (the byte the strict branch constrains).
        let extra = decoded.extra?;
        let strict_ok = ((extra >> 40) & 0x01) != 0;
        if !strict_ok {
            return None;
        }
        let plain4 = ((extra >> 32) & 0xFF) as u8;

        // raw[0..7] = key1 (56 bits), taken from `data` (low 56 bits). The decoder packs key1 as
        // raw[0]<<48 | … | raw[6], so the top 7 bytes of the 56-bit value are raw[0..7].
        let mut raw17 = [0u8; DATA_BYTES];
        for (i, b) in raw17.iter_mut().take(7).enumerate() {
            *b = (decoded.data >> (48 - i * 8)) as u8;
        }

        // Reconstruct the plaintext block from the decoded fields plus the stashed plain[4].
        // plain_apply_fields fills [0,1,2,3,5,6,7,8]; plain[4] is preserved from `extra`. The
        // strict-branch invariant the C decoder verified is plain[4]==air9[0]==raw[6] and
        // plain[3]==raw[5]; re-encoding plain reproduces the air block and hence those bytes.
        let mut plain9 = [0u8; 9];
        plain9[4] = plain4;
        // Seed checksum-bearing fields with the *decoded* values so the running checksum delta in
        // plain_apply_fields starts from the original plaintext's relationship, then apply the new
        // button/counter (same counter; no increment, matching Ford V0's replay policy).
        plain9[5] = (((decoded.button.unwrap_or(0) & 0x0F) << 4)
            | (((counter >> 16) & 0x0F) as u8)) as u8;
        plain9[6] = ((counter >> 8) & 0xFF) as u8;
        plain9[7] = (counter & 0xFF) as u8;
        plain9[8] = plain9[5].wrapping_add(plain9[6]).wrapping_add(plain9[7]);
        Self::plain_apply_fields(&mut plain9, serial, btn, counter);

        // Rebuild raw[5] = plain[3] (strict invariant) and the air block + CRC from plaintext.
        raw17[5] = plain9[3];
        Self::rebuild_raw_from_plain(&mut raw17, &plain9);

        // Build the 6-burst upload.
        let mut signal =
            Vec::with_capacity(ENC_BURST_COUNT * (ENC_PREAMBLE_PAIRS * 2 + 2 + DATA_BYTES * 16 + 1));
        for burst in 0..ENC_BURST_COUNT {
            let mut pkt = raw17;
            pkt[4] = ENC_BURST_PKT4[burst];
            let crcw = Self::crc16(&pkt[3..15]);
            pkt[15] = (crcw >> 8) as u8;
            pkt[16] = (crcw & 0xFF) as u8;

            // Preamble: 400 pairs of long high / long low.
            for _ in 0..ENC_PREAMBLE_PAIRS {
                Self::enc_add_level(&mut signal, true, ENC_SYNC_LONG_US);
                Self::enc_add_level(&mut signal, false, ENC_SYNC_LONG_US);
            }
            // Sync: long high + short low.
            Self::enc_add_level(&mut signal, true, ENC_SYNC_LONG_US);
            Self::enc_add_level(&mut signal, false, ENC_SYNC_SHORT_US);

            // Data: each bit → (bit, short) then (!bit, short) — Manchester, MSB first.
            for &b in pkt.iter() {
                for bit_i in (0..8).rev() {
                    let bit = ((b >> bit_i) & 1) != 0;
                    Self::enc_add_level(&mut signal, bit, ENC_SYNC_SHORT_US);
                    Self::enc_add_level(&mut signal, !bit, ENC_SYNC_SHORT_US);
                }
            }

            // Trailing gap (long for repeats, short for the final burst).
            let gap = if burst + 1 == ENC_BURST_COUNT {
                ENC_GAP_LAST_US
            } else {
                ENC_GAP_REPEAT_US
            };
            Self::enc_add_level(&mut signal, false, gap);
        }

        Some(signal)
    }
}

impl Default for FordV1Decoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// CRC16/CCITT known vector. "123456789" → 0x31C3 for poly 0x1021, init 0x0000
    /// (the standard CRC-16/XMODEM check value).
    #[test]
    fn crc16_known_vector() {
        let v = FordV1Decoder::crc16(b"123456789");
        assert_eq!(v, 0x31C3, "CRC-16/XMODEM check value mismatch: got {:04X}", v);
    }

    /// The descramble cipher must be invertible: encode_inverse_block ∘ decode_with_flag(flag)
    /// should round-trip for the flag the parity selects.
    #[test]
    fn descramble_roundtrip() {
        // A plaintext block; encode to air, then plain_from_air must recover it exactly.
        let plain: [u8; 9] = [0x11, 0x22, 0x33, 0x44, 0x55, 0x26, 0x77, 0x88, 0x99];
        let air = FordV1Decoder::encode_air_9bytes(&plain);
        let recovered = FordV1Decoder::plain_from_air(&air).expect("round-trip recover");
        assert_eq!(recovered, plain, "plain_from_air did not recover plaintext");
    }

    /// Build a canonical Ford V1 plaintext (the strict-branch invariant is plain[4]==plain[0],
    /// since the cipher leaves byte 0 untouched, so air9[0]==plain[0]). Returns the 17-byte frame.
    fn build_frame(serial: u32, button: u8, counter: u32) -> [u8; DATA_BYTES] {
        let mut plain = [0u8; 9];
        plain[4] = (serial & 0xFF) as u8; // strict: plain[4] == air9[0] == plain[0]
        FordV1Decoder::plain_apply_fields(&mut plain, serial, button, counter);

        let mut raw17 = [0u8; DATA_BYTES];
        // key1 head: raw[0..5] arbitrary-ish, raw[5] must equal plain[3] (strict: decoded[3]==raw[5]).
        raw17[0] = 0xC0;
        raw17[1] = 0xFF;
        raw17[2] = 0xEE;
        raw17[3] = (serial >> 24) as u8;
        raw17[4] = (serial >> 16) as u8;
        raw17[5] = plain[3];
        FordV1Decoder::rebuild_raw_from_plain(&mut raw17, &plain);
        raw17
    }

    /// process_data must accept a canonical frame, take the strict branch, and recover fields.
    #[test]
    fn process_data_strict_branch() {
        let (serial, button, counter) = (0x1A2B3C4Du32, 0x02u8, 0x0123u32);
        let raw17 = build_frame(serial, button, counter);

        let mut dec = FordV1Decoder::new();
        dec.raw_bytes[..DATA_BYTES].copy_from_slice(&raw17);
        dec.byte_count = DATA_BYTES;
        let d = dec.process_data().expect("process_data should accept the frame");
        assert!(d.crc_valid, "CRC must be valid");
        assert_eq!(d.serial, Some(serial), "serial mismatch");
        assert_eq!(d.button, Some(button), "button mismatch");
        assert_eq!(d.counter, Some(counter as u16), "counter mismatch");
        assert_eq!(d.data_count_bit, DATA_BITS);
    }

    /// Reconstruct the 17 on-air data bytes of the first burst from the encoder's Manchester
    /// upload. The encoder emits, per data bit, (bit,short) then (!bit,short); add_level merges
    /// adjacent same-level pulses, so a same-bit boundary becomes a long pulse. We re-pair the
    /// stream by walking half-cells of `short` width (splitting merged long pulses into two halves)
    /// and reading each bit as the level of its first half-cell.
    fn first_burst_bytes(upload: &[LevelDuration]) -> [u8; DATA_BYTES] {
        // Expand merged pulses into ENC_SYNC_SHORT_US half-cells (long = 2 halves).
        let mut halves: Vec<bool> = Vec::new();
        for ld in upload {
            let n = ((ld.duration_us + ENC_SYNC_SHORT_US / 2) / ENC_SYNC_SHORT_US).max(1);
            for _ in 0..n {
                halves.push(ld.level);
            }
        }
        // Skip the preamble (400 long pairs = 1600 halves) + sync (long high=2 + short low=1 = 3).
        let data_start = ENC_PREAMBLE_PAIRS * 4 + 3;
        let mut out = [0u8; DATA_BYTES];
        for (byte_i, b) in out.iter_mut().enumerate() {
            for bit_pos in 0..8 {
                // Each bit = two half-cells; the bit value is the first half-cell's level.
                let idx = data_start + (byte_i * 8 + bit_pos) * 2;
                let bit = halves.get(idx).copied().unwrap_or(false);
                *b = (*b << 1) | (bit as u8);
            }
        }
        out
    }

    /// Full encode→decode round trip at the on-air-frame level. Decode a canonical frame, re-encode
    /// it via `encode()`, recover the transmitted 17-byte frame from the first burst, and verify it
    /// decodes (via process_data) back to the same serial/button/counter with a valid CRC.
    #[test]
    fn encode_decode_roundtrip() {
        let (serial, button, counter) = (0x1A2B3C4Du32, 0x02u8, 0x0123u32);
        let raw17 = build_frame(serial, button, counter);

        let mut dec = FordV1Decoder::new();
        dec.raw_bytes[..DATA_BYTES].copy_from_slice(&raw17);
        dec.byte_count = DATA_BYTES;
        let decoded = dec.process_data().expect("decode canonical frame");

        // Re-encode (same button) → Manchester upload (6 bursts).
        let upload = FordV1Decoder::new()
            .encode(&decoded, button)
            .expect("encoder should produce an upload");
        assert!(!upload.is_empty(), "encoder upload must be non-empty");

        // Recover the transmitted frame from burst 0 and decode it through process_data.
        // Burst 0 overrides pkt[4] = 0x08 (ENC_BURST_PKT4[0]) and recomputes the CRC to match, so
        // the recovered frame is self-consistent. The strict-branch serial comes from the plaintext
        // (decoded[1..4],decoded[0]), independent of raw[4], so serial/button/counter still match.
        let tx = first_burst_bytes(&upload);
        let mut dec2 = FordV1Decoder::new();
        dec2.raw_bytes[..DATA_BYTES].copy_from_slice(&tx);
        dec2.byte_count = DATA_BYTES;
        let got = dec2
            .process_data()
            .expect("re-encoded burst-0 frame must decode via process_data");
        assert!(got.crc_valid, "round-tripped CRC must be valid");
        assert_eq!(got.serial, Some(serial), "round-trip serial mismatch");
        assert_eq!(got.button, Some(button), "round-trip button mismatch");
        assert_eq!(got.counter, Some(counter as u16), "round-trip counter mismatch");

        // Structural checks: 6 bursts worth of data, each preamble present.
        let long_highs = upload
            .iter()
            .filter(|l| l.level && l.duration_us >= ENC_SYNC_LONG_US)
            .count();
        assert!(
            long_highs >= ENC_BURST_COUNT,
            "expected at least one long-high preamble pulse per burst"
        );
    }
}
