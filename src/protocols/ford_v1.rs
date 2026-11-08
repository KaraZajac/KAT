//! Ford V1 protocol decoder/encoder
//!
//! Port of ProtoPirate `REFERENCES/ProtoPirate/protocols/ford_v1.c` as of ProtoPirate HEAD
//! (dc3446b; the decode/encode logic is unchanged since 62b31bf). Flipper-ARF's
//! `lib/subghz/protocols/ford_v1.c` is the same decoder and the same upload; its encoder adds the
//! counter bump inline (`cnt + rolling_counter_mult`) and a D-pad button map onto the same codes.
//! doctor-firmware's copy decodes a strict subset (drops frames whose fields fail a
//! button/serial plausibility check) and builds the same upload, so it adds nothing.
//!
//! **RF**: Manchester te_short 65µs / te_long 130µs, FM, 315 + 433.92 MHz. 136 bits = 17 bytes
//! `raw[0..17]`: key1 = raw[0..7] (56 bits), key2 = raw[7..15] (64 bits), CRC16 = raw[15..17].
//!
//! **Decoder** (`subghz_protocol_decoder_ford_v1_feed`): Reset → a LOW long pulse (|d-130| < 40)
//! starts the preamble; ≥50 long pulses then a short pulse enter Sync, which buffers up to 8
//! Manchester events and, once more than two of them are short, resets the Manchester state (Mid1;
//! Mid0 if the first buffered event was ShortLow), replays the buffered events and enters Data.
//! Events are `level ? ShortHigh : ShortLow` / `level ? LongHigh : LongLow` (|d-te| < 39), using
//! the shared Flipper `manchester_advance` port in `common.rs`. Bits are packed MSB-first into
//! bytes; the 17th byte runs `process_data`. Any other pulse (the inter-burst gap, a glitch) runs
//! `ford_v1_try_last_byte_variants`: with 16 bytes and 6 or 7 bits of the last byte collected (the
//! frame's last half-bit(s) merged into the trailing gap), it tries every completion of the
//! missing 1-2 bits through `process_data` and reports the first that passes. Either way it resets.
//!
//! **process_data**: CRC16/CCITT (poly 0x1021, init 0) over raw[3..15] must equal raw[15..17]; if
//! not, all 17 bytes are bit-inverted and checked again (the air bit polarity is the inverse of
//! the decoder's Manchester bit). The 9-byte air block raw[6..15] is descrambled
//! (`ford_v1_decode_with_flag`, flag 0/1); "strict" when `plain[3] == raw[5] && plain[4] ==
//! raw[6]`, which yields serial = plain[1..4]‖plain[0], button = plain[5] >> 4, counter = 20 bits
//! (plain[5] & 0x0F, plain[6], plain[7]). Otherwise (encrypted/rolling via the encode round trip,
//! or unresolved) the C reports serial = raw[3..7], button 0, counter 0. The C emits every
//! CRC-valid frame and so does KAT.
//!
//! **DecodedSignal**: exactly the C's saved fields — `data` = key1 (`Key`, 56 bits), `extra` =
//! key2 (`Key_2`, 64 bits); the CRC (`Key_3`) is recomputed from them. `serial`/`button` as above,
//! `counter` = the LOW 16 bits of the 20-bit counter (KAT's counter is u16; the top nibble is
//! recovered from the frame), `data_count_bit` = 136. `encoder_capable` is the C encoder's
//! precondition (`ford_v1_extract_plain_from_raw`: strict or round-trip plaintext recovered).
//! Exports from KAT before this port carried a different `extra` (crc16 << 48 | strict << 40 |
//! plain[4] << 32); such an `extra` is recognised and key2 is rebuilt from the decoded fields by
//! matching the stored CRC16 (see `legacy_key2`).
//!
//! **Encoder** (`subghz_protocol_encoder_ford_v1_deserialize` + ProtoPirate's emulate scene):
//! rebuild the frame from key1/key2, recover the 9-byte plaintext (refuse if it cannot be, as the
//! C does), map KAT's button (0x01 Lock, 0x02 Unlock, 0x04 Trunk, 0x08 Panic — the emulate scene's
//! Up/OK/Down/Left; anything else keeps the captured button), bump the 20-bit counter by one (the
//! emulate scene's `current_counter++`; ARF's `cnt + mult`), `ford_v1_plain_apply_fields`, re-scramble
//! the air block, refresh the CRC, then build burst 0 (`ford_v1_encoder_build_burst(.., 0)`):
//! pkt[4] = 0x08 and the CRC recomputed, 400 × (HIGH 130, LOW 130), HIGH 130, LOW 65, the 17
//! bytes MSB-first as (bit, 65µs)(!bit, 65µs), LOW 50 000µs. The C never advances `burst_idx`, so
//! every transmission is burst 0; the Flipper TX loop repeats the upload `repeat` = 10 times
//! (doctor-firmware: 3). KAT's `encode()` returns the single upload and KAT sends it 3 times
//! (`tx_copies.rs`; a key file's `Repeat` line sets the count). Adjacent same-level
//! half-bits are merged into single pulses (identical on air).
//!
//! As in the C, a frame whose first air bit is 0 cannot be decoded (the sync LOW merges with the
//! first half-bit, so the decoder stays in the preamble) — see the tests.
//!
//! **Key files** (`subghz_protocol_decoder_ford_v1_serialize` / `_deserialize`; Flipper-ARF's
//! are the same): `Protocol: Ford V1`, `Bit: 136`, `Key` = key1 (56 bits, so the first byte is
//! 00), then `Serial`, `Btn`, `Cnt` (the C's saved fields: plaintext serial/button/20-bit counter,
//! or header serial + 0 + 0 for a non-strict frame), `Key_2` = key2 (8 bytes) and `Key_3` =
//! `00 00` + the CRC16 — the order the C's `insert_or_update` calls leave them in. Loading follows
//! `ford_v1_keys_file_to_canonical_raw`: Bit 136, Key and Key_2 required, Key_3 optional (CRC 0
//! when missing), Serial/Btn/Cnt required (the C rejects the file without them); the frame
//! key1 ‖ key2 ‖ CRC is tried as written, then with each key's 32-bit halves byte-swapped (the
//! C's legacy layout), and the first one whose plaintext is recoverable is decoded exactly as the
//! stream decoder does. A frame that only passes the CRC loads as the decoder reports it
//! (`encoder_capable` false); one that fails it (wrong or missing Key_3) still loads, as in the C,
//! with the file's fields and `crc_valid`/`encoder_capable` false. The fields are the frame's
//! (the C displays the file's Serial/Btn/Cnt, which it writes from the frame; hand-edited values
//! are not applied).

use super::common::{ManchesterEvent, ManchesterState};
use super::{ProtocolDecoder, ProtocolTiming, DecodedSignal, FlipperKey};
use crate::radio::demodulator::LevelDuration;
use crate::duration_diff;

use super::info::prelude::*;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Ford V1",
    decoder: "Ford V1",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::FM,
    encryption: "Rolling Code",
    make: "Ford",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&F4],
};

/// 1/2/4/8 are KAT's generic codes; 0 is ford_v1_get_button_name's "Sync".
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x0 => "Sync",
        _ => return None,
    })
}

const TE_SHORT: u32 = 65;
const TE_LONG: u32 = 130;
const TE_DELTA: u32 = 39;
const DATA_BITS: usize = 136;
const DATA_BYTES: usize = 17;
const PREAMBLE_MIN: u16 = 50;
/// FORD_V1_DELTA_LONG (40): only the preamble long-pulse match; data/sync use te_delta (39).
const DELTA_LONG: u32 = 40;
const COUNTER_MASK: u32 = 0xF_FFFF;

// Encoder constants (subghz_protocol_encoder_ford_v1).
const ENC_PREAMBLE_PAIRS: usize = 400;
const ENC_SYNC_SHORT_US: u32 = 65;
const ENC_SYNC_LONG_US: u32 = 130;
/// `FORD_V1_ENC_GAP_REPEAT_US`: the gap ending every burst except burst index 5, i.e. burst 0.
const ENC_GAP_REPEAT_US: u32 = 50_000;
/// `ford_v1_encoder_burst_pkt4_vals[0]`: pkt[4] of burst 0, the only burst the C sends.
const ENC_BURST0_PKT4: u8 = 0x08;

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
    sync_events: [ManchesterEvent; 8],
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
            sync_events: [ManchesterEvent::ShortLow; 8],
        }
    }

    fn reset_state(&mut self) {
        self.step = DecoderStep::Reset;
        self.manchester_state.advance(ManchesterEvent::Reset);
        self.preamble_count = 0;
        self.decode_data = 0;
        self.decode_count_bit = 0;
        self.byte_count = 0;
        self.raw_bytes = [0; DATA_BYTES + 1];
        self.sync_event_idx = 0;
        self.sync_event_count = 0;
        self.sync_events = [ManchesterEvent::ShortLow; 8];
    }

    fn is_short(d: u32) -> bool {
        duration_diff!(d, TE_SHORT) < TE_DELTA
    }
    fn is_long(d: u32) -> bool {
        duration_diff!(d, TE_LONG) < TE_DELTA
    }
    fn is_preamble_long(d: u32) -> bool {
        duration_diff!(d, TE_LONG) < DELTA_LONG
    }

    /// One decoded Manchester bit into the byte buffer (the C's Sync replay / Data byte assembly).
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

    /// `subghz_protocol_blocks_crc16(data, len, 0x1021, 0x0000)`.
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

    /// `ford_v1_decode_with_flag` on a 9-byte air block, in place.
    fn decode_with_flag(raw: &mut [u8; 9], flag_byte: u8) {
        if flag_byte != 0 {
            let xor_byte = raw[7];
            for b in raw.iter_mut().take(7).skip(1) {
                *b ^= xor_byte;
            }
        } else {
            let xor_byte = raw[6];
            for b in raw.iter_mut().take(6).skip(1) {
                *b ^= xor_byte;
            }
            raw[7] ^= xor_byte;
        }
        let b6 = raw[6];
        let b7 = raw[7];
        raw[6] = (b6 & 0xAA) | (b7 & 0x55);
        raw[7] = (b7 & 0xAA) | (b6 & 0x55);
    }

    /// `ford_v1_encode_inverse_block`: plaintext block → air block, in place.
    fn encode_inverse_block(block: &mut [u8; 9]) {
        let mut sum: u8 = 0;
        for &b in block.iter().take(8).skip(1) {
            sum = sum.wrapping_add(b);
        }
        let p6 = block[6];
        let p7 = block[7];
        let post6 = (p6 & 0xAA) | (p7 & 0x55);
        let post7 = (p7 & 0xAA) | (p6 & 0x55);
        let xorv = post6 ^ post7;
        let xor_byte;
        if sum.count_ones() & 1 != 0 {
            block[6] = xorv;
            block[7] = post7;
            xor_byte = post7;
        } else {
            block[6] = post6;
            block[7] = xorv;
            xor_byte = post6;
        }
        for b in block.iter_mut().take(6).skip(1) {
            *b ^= xor_byte;
        }
    }

    fn encode_air_9bytes(plain9: &[u8; 9]) -> [u8; 9] {
        let mut block = *plain9;
        Self::encode_inverse_block(&mut block);
        block
    }

    /// `ford_v1_plain_from_air`: a flag whose descramble re-encodes to the same air block.
    fn plain_from_air(air9: &[u8; 9]) -> Option<[u8; 9]> {
        (0u8..2).find_map(|flag| {
            let mut cand = *air9;
            Self::decode_with_flag(&mut cand, flag);
            (Self::encode_air_9bytes(&cand) == *air9).then_some(cand)
        })
    }

    /// The strict branch (`decoded[3] == raw[5] && decoded[4] == raw[6]`), flag 0 then 1.
    fn strict_plain(raw: &[u8; DATA_BYTES]) -> Option<[u8; 9]> {
        let air9: [u8; 9] = raw[6..15].try_into().unwrap();
        (0u8..2).find_map(|flag| {
            let mut cand = air9;
            Self::decode_with_flag(&mut cand, flag);
            (cand[3] == raw[5] && cand[4] == raw[6]).then_some(cand)
        })
    }

    /// CRC16 check with the C's 17-byte inversion fallback; returns the canonical frame.
    fn canonical_crc_ok(raw17: &[u8; DATA_BYTES]) -> Option<[u8; DATA_BYTES]> {
        let ok = |r: &[u8; DATA_BYTES]| Self::crc16(&r[3..15]) == u16::from_be_bytes([r[15], r[16]]);
        if ok(raw17) {
            return Some(*raw17);
        }
        let inv = raw17.map(|b| !b);
        ok(&inv).then_some(inv)
    }

    /// `ford_v1_extract_plain_from_raw` (CRC-checked frame → plaintext, strict or round trip).
    /// Returns (canonical frame, plaintext, strict).
    fn extract_plain(raw17: &[u8; DATA_BYTES]) -> Option<([u8; DATA_BYTES], [u8; 9], bool)> {
        let raw = Self::canonical_crc_ok(raw17)?;
        if let Some(p) = Self::strict_plain(&raw) {
            return Some((raw, p, true));
        }
        let air9: [u8; 9] = raw[6..15].try_into().unwrap();
        Self::plain_from_air(&air9).map(|p| (raw, p, false))
    }

    /// `ford_v1_fields_from_plain` → (serial, button, 20-bit counter).
    fn fields_from_plain(plain9: &[u8; 9]) -> (u32, u8, u32) {
        let serial = ((plain9[1] as u32) << 24)
            | ((plain9[2] as u32) << 16)
            | ((plain9[3] as u32) << 8)
            | (plain9[0] as u32);
        let btn = (plain9[5] >> 4) & 0x0F;
        let cnt = (((plain9[5] & 0x0F) as u32) << 16) | ((plain9[6] as u32) << 8) | (plain9[7] as u32);
        (serial, btn, cnt)
    }

    /// `ford_v1_process_data`: CRC gate (+ inversion fallback), descramble, fields. Every
    /// CRC-valid frame is reported (strict → plaintext fields, otherwise header-only).
    fn process_data(raw_bytes: &[u8; DATA_BYTES]) -> Option<DecodedSignal> {
        let raw = Self::canonical_crc_ok(raw_bytes)?;
        let (serial, button, counter) = match Self::strict_plain(&raw) {
            Some(plain) => Self::fields_from_plain(&plain),
            None => (u32::from_be_bytes([raw[3], raw[4], raw[5], raw[6]]), 0, 0),
        };
        let key1 = raw[..7].iter().fold(0u64, |k, &b| (k << 8) | b as u64);
        let key2 = raw[7..15].iter().fold(0u64, |k, &b| (k << 8) | b as u64);
        Some(DecodedSignal {
            serial: Some(serial),
            button: Some(button),
            counter: Some(counter as u16),
            crc_valid: true,
            data: key1,
            data_count_bit: DATA_BITS,
            encoder_capable: Self::extract_plain(&raw).is_some(),
            extra: Some(key2),
            protocol_display_name: None,
        })
    }

    /// `ford_v1_try_last_byte_variants`: 16 bytes + 6 or 7 bits of the last byte collected when
    /// the run ended; try each completion of the missing 1-2 low bits, first CRC pass wins.
    fn try_last_byte_variants(&self) -> Option<DecodedSignal> {
        if self.byte_count != 16 || !matches!(self.decode_count_bit, 134 | 135) {
            return None;
        }
        let shift = DATA_BITS - self.decode_count_bit;
        let partial = self.decode_data as u8;
        let mut raw = [0u8; DATA_BYTES];
        raw[..16].copy_from_slice(&self.raw_bytes[..16]);
        (0u8..(1 << shift)).find_map(|variant| {
            raw[16] = (partial << shift) | variant;
            Self::process_data(&raw)
        })
    }

    /// Map KAT's generic button to a Ford V1 code (emulate scene: Up=Lock 1, OK=Unlock 2,
    /// Down=Trunk 4, Left=Panic 8; any other key keeps the captured button).
    fn map_button(button: u8, captured: u8) -> u8 {
        match button {
            0x01 | 0x02 | 0x04 | 0x08 => button,
            _ => captured & 0x0F,
        }
    }

    /// `ford_v1_plain_apply_fields`: write serial/button/counter, keeping plain[4] and the
    /// plain[8] − (plain[5] + plain[6] + plain[7]) relationship.
    fn plain_apply_fields(plain9: &mut [u8; 9], serial: u32, btn: u8, cnt: u32) {
        let chk = plain9[8]
            .wrapping_sub(plain9[6])
            .wrapping_sub(plain9[7])
            .wrapping_sub(plain9[5]);
        plain9[0] = serial as u8;
        plain9[1] = (serial >> 24) as u8;
        plain9[2] = (serial >> 16) as u8;
        plain9[3] = (serial >> 8) as u8;
        plain9[5] = ((btn & 0x0F) << 4) | ((cnt >> 16) & 0x0F) as u8;
        plain9[6] = (cnt >> 8) as u8;
        plain9[7] = cnt as u8;
        plain9[8] = chk
            .wrapping_add(plain9[7])
            .wrapping_add(plain9[6])
            .wrapping_add(plain9[5]);
    }

    /// `ford_v1_encoder_rebuild_raw_from_plain`: air block raw[6..15] + CRC raw[15..17].
    fn rebuild_raw_from_plain(raw17: &mut [u8; DATA_BYTES], plain9: &[u8; 9]) {
        raw17[6..15].copy_from_slice(&Self::encode_air_9bytes(plain9));
        let c = Self::crc16(&raw17[3..15]);
        raw17[15..17].copy_from_slice(&c.to_be_bytes());
    }

    /// Frame from the stored keys: raw[0..7] = key1, raw[7..15] = key2, CRC recomputed.
    fn frame_from_keys(key1: u64, key2: u64) -> [u8; DATA_BYTES] {
        let mut raw = [0u8; DATA_BYTES];
        raw[..7].copy_from_slice(&key1.to_be_bytes()[1..]);
        raw[7..15].copy_from_slice(&key2.to_be_bytes());
        let c = Self::crc16(&raw[3..15]);
        raw[15..17].copy_from_slice(&c.to_be_bytes());
        raw
    }

    /// KAT exports made before this port stored `extra = crc16 << 48 | strict << 40 |
    /// plain[4] << 32` (strict frames only) instead of key2. key2 is the scrambled plaintext
    /// whose only unknowns given serial/button/counter are the counter's top nibble and plain[8]:
    /// try all 4096 and keep the first whose frame reproduces the stored CRC16.
    fn legacy_key2(decoded: &DecodedSignal, extra: u64) -> Option<u64> {
        let legacy = extra & 0xFFFF_FFFF == 0 && (extra >> 40) & 0xFF == 1 && decoded.data >> 56 == 0;
        if !legacy {
            return None;
        }
        let crc = (extra >> 48) as u16;
        let plain4 = (extra >> 32) as u8;
        let serial = decoded.serial?;
        let btn = decoded.button? & 0x0F;
        let cnt_low = decoded.counter? as u32;
        let key1 = decoded.data;
        for (top, p8) in (0..16u32).flat_map(|t| (0..=255u8).map(move |p| (t, p))) {
            let mut plain = [0u8; 9];
            Self::plain_apply_fields(&mut plain, serial, btn, (top << 16) | cnt_low);
            plain[4] = plain4;
            plain[8] = p8;
            let mut raw = Self::frame_from_keys(key1, 0);
            Self::rebuild_raw_from_plain(&mut raw, &plain);
            if u16::from_be_bytes([raw[15], raw[16]]) == crc {
                return Some(raw[7..15].iter().fold(0u64, |k, &b| (k << 8) | b as u64));
            }
        }
        None
    }

    /// `ford_v1_crc16_from_key3_bytes`: the CRC16 from a `Key_3` line (`00 00 HI LO`; the legacy
    /// `HI LO 00 00` layout byte-swapped).
    fn crc16_from_key3(key3: &[u8; 4]) -> u16 {
        let crc = u32::from_be_bytes(*key3);
        if crc & 0xFFFF == 0 && crc >> 16 != 0 {
            ((crc >> 16) as u16).swap_bytes()
        } else {
            crc as u16
        }
    }

    /// `ford_v1_u64_legacy_halves_bswap`: each 32-bit half of a stored key byte-swapped.
    fn legacy_halves_bswap(stored: u64) -> u64 {
        (((stored >> 32) as u32).swap_bytes() as u64) << 32 | (stored as u32).swap_bytes() as u64
    }

    /// The 17-byte frame `key1[1..8] ‖ key2 ‖ crc16` as a key file stores it (CRC not recomputed).
    fn frame_from_file(key1: u64, key2: u64, crc16: u16) -> [u8; DATA_BYTES] {
        let mut raw = [0u8; DATA_BYTES];
        raw[..7].copy_from_slice(&key1.to_be_bytes()[1..]);
        raw[7..15].copy_from_slice(&key2.to_be_bytes());
        raw[15..17].copy_from_slice(&crc16.to_be_bytes());
        raw
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

    /// `ford_v1_encoder_build_burst(instance, 0)`.
    fn build_burst0(raw_tx: &[u8; DATA_BYTES]) -> Vec<LevelDuration> {
        let mut pkt = *raw_tx;
        pkt[4] = ENC_BURST0_PKT4;
        let crc = Self::crc16(&pkt[3..15]);
        pkt[15..17].copy_from_slice(&crc.to_be_bytes());

        let mut signal = Vec::with_capacity(ENC_PREAMBLE_PAIRS * 2 + 2 + DATA_BYTES * 16 + 1);
        for _ in 0..ENC_PREAMBLE_PAIRS {
            Self::enc_add_level(&mut signal, true, ENC_SYNC_LONG_US);
            Self::enc_add_level(&mut signal, false, ENC_SYNC_LONG_US);
        }
        Self::enc_add_level(&mut signal, true, ENC_SYNC_LONG_US);
        Self::enc_add_level(&mut signal, false, ENC_SYNC_SHORT_US);
        for &b in pkt.iter() {
            for bit_i in (0..8).rev() {
                let bit = (b >> bit_i) & 1 != 0;
                Self::enc_add_level(&mut signal, bit, ENC_SYNC_SHORT_US);
                Self::enc_add_level(&mut signal, !bit, ENC_SYNC_SHORT_US);
            }
        }
        Self::enc_add_level(&mut signal, false, ENC_GAP_REPEAT_US);
        signal
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
            DecoderStep::Reset => {
                if !level && Self::is_preamble_long(duration) {
                    self.step = DecoderStep::Preamble;
                    self.preamble_count = 1;
                }
            }

            DecoderStep::Preamble => {
                if Self::is_preamble_long(duration) {
                    self.preamble_count = self.preamble_count.wrapping_add(1);
                } else if Self::is_short(duration) {
                    if self.preamble_count >= PREAMBLE_MIN {
                        self.sync_event_idx = 0;
                        self.sync_event_count = 1;
                        self.sync_events[0] = ManchesterEvent::from_half_bit(level, false);
                        self.step = DecoderStep::Sync;
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else if self.preamble_count < PREAMBLE_MIN {
                    self.step = DecoderStep::Reset;
                }
            }

            DecoderStep::Sync => {
                let (ev, is_short) = if Self::is_short(duration) {
                    (ManchesterEvent::from_half_bit(level, false), true)
                } else if Self::is_long(duration) {
                    (ManchesterEvent::from_half_bit(level, true), false)
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
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                    self.byte_count = 0;
                    self.raw_bytes = [0; DATA_BYTES + 1];
                    self.manchester_state.advance(ManchesterEvent::Reset);
                    if self.sync_events[0] == ManchesterEvent::ShortLow {
                        self.manchester_state = ManchesterState::Mid0;
                    }
                    self.step = DecoderStep::Data;
                    for i in 0..=self.sync_event_idx.min(7) {
                        let event = self.sync_events[i as usize];
                        if let Some(data_bit) = self.manchester_state.advance(event) {
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
                    ManchesterEvent::from_half_bit(level, false)
                } else if Self::is_long(duration) {
                    ManchesterEvent::from_half_bit(level, true)
                } else {
                    // Idle gap or odd pulse: the C tries the partial-last-byte completions, then
                    // resets either way.
                    let result = self.try_last_byte_variants();
                    self.reset_state();
                    return result;
                };

                if let Some(data_bit) = self.manchester_state.advance(event) {
                    self.push_bit(data_bit);
                    if self.byte_count > 16 {
                        let raw: [u8; DATA_BYTES] = self.raw_bytes[..DATA_BYTES].try_into().unwrap();
                        let result = Self::process_data(&raw);
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
        if decoded.data_count_bit != DATA_BITS {
            return None;
        }
        let extra = decoded.extra?;
        let key2 = Self::legacy_key2(decoded, extra).unwrap_or(extra);
        // ford_v1_keys_file_to_canonical_raw: the C refuses frames whose plaintext cannot be
        // recovered (encryption_supported / plain_valid).
        let (raw, plain, strict) = Self::extract_plain(&Self::frame_from_keys(decoded.data, key2))?;

        // The file's Serial/Btn/Cnt as the C decoder saved them (strict: plaintext fields;
        // otherwise header serial, button 0, counter 0), overridden by the DecodedSignal fields.
        let (frame_serial, frame_btn, frame_cnt) = if strict {
            Self::fields_from_plain(&plain)
        } else {
            (u32::from_be_bytes([raw[3], raw[4], raw[5], raw[6]]), 0, 0)
        };
        let serial = decoded.serial.unwrap_or(frame_serial);
        let captured = decoded.button.unwrap_or(frame_btn);
        let cnt = match decoded.counter {
            Some(low) => (frame_cnt & 0xF_0000) | low as u32,
            None => frame_cnt,
        };
        let btn = Self::map_button(button, captured);
        let cnt = cnt.wrapping_add(1) & COUNTER_MASK;

        let mut plain = plain;
        Self::plain_apply_fields(&mut plain, serial, btn, cnt);
        let mut raw_tx = raw;
        Self::rebuild_raw_from_plain(&mut raw_tx, &plain);
        Some(Self::build_burst0(&raw_tx))
    }

    /// `subghz_protocol_decoder_ford_v1_deserialize` (see the module doc, "Key files").
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != DATA_BITS {
            return None;
        }
        let key1 = key.key_u64()?;
        let key2: [u8; 8] = key.hex_bytes_field("Key_2")?.try_into().ok()?;
        let key2 = u64::from_be_bytes(key2);
        let crc16 = key
            .hex_bytes_field("Key_3")
            .and_then(|b| <[u8; 4]>::try_from(b).ok())
            .map_or(0, |b| Self::crc16_from_key3(&b));
        let (serial, btn, cnt) = (key.u32_field("Serial")?, key.u32_field("Btn")?, key.u32_field("Cnt")?);
        // ford_v1_keys_file_to_canonical_raw: as written, then the legacy halves-swapped keys.
        let attempts = [
            (key1, key2),
            (Self::legacy_halves_bswap(key1), Self::legacy_halves_bswap(key2)),
        ];
        for (k1, k2) in attempts {
            let raw = Self::frame_from_file(k1, k2, crc16);
            if Self::extract_plain(&raw).is_some() {
                return Self::process_data(&raw);
            }
        }
        // No plaintext: the frame as the decoder reports it when its CRC holds, else the C's
        // "CRC ERR" view of the file.
        Self::process_data(&Self::frame_from_file(key1, key2, crc16)).or(Some(DecodedSignal {
            serial: Some(serial),
            button: Some(btn as u8),
            counter: Some(cnt as u16),
            crc_valid: false,
            data: key1,
            data_count_bit: DATA_BITS,
            encoder_capable: false,
            extra: Some(key2),
            protocol_display_name: None,
        }))
    }

    /// `subghz_protocol_decoder_ford_v1_serialize` (see the module doc, "Key files").
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != DATA_BITS || decoded.data >> 56 != 0 {
            return None;
        }
        let extra = decoded.extra?;
        let key2 = Self::legacy_key2(decoded, extra).unwrap_or(extra);
        // The C's `crc_calc` is the CRC16 recomputed over the (canonical) frame.
        let raw = Self::frame_from_keys(decoded.data, key2);
        let (serial, btn, cnt) = match Self::strict_plain(&raw) {
            Some(plain) => Self::fields_from_plain(&plain),
            None => (u32::from_be_bytes([raw[3], raw[4], raw[5], raw[6]]), 0, 0),
        };
        let mut key = FlipperKey::for_protocol("Ford V1", DATA_BITS, decoded.data);
        key.push_u32("Serial", serial);
        key.push_u32("Btn", btn.into());
        key.push_u32("Cnt", cnt);
        key.push_hex("Key_2", key2, 8);
        key.push_hex_bytes("Key_3", &[0, 0, raw[15], raw[16]]);
        Some(key)
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

    /// CRC-16/XMODEM check value ("123456789" → 0x31C3, poly 0x1021, init 0).
    #[test]
    fn crc16_known_vector() {
        assert_eq!(FordV1Decoder::crc16(b"123456789"), 0x31C3);
    }

    /// A strict (cleartext) frame: plain[4] == plain[0] and raw[5] == plain[3]; `head` fills
    /// raw[0..5] (raw[0]'s MSB is the first air bit, which must be 1 to be decodable).
    fn build_frame(head: [u8; 5], serial: u32, button: u8, counter: u32, plain8: u8) -> [u8; DATA_BYTES] {
        let mut plain = [0u8; 9];
        plain[4] = serial as u8;
        FordV1Decoder::plain_apply_fields(&mut plain, serial, button, counter);
        plain[8] = plain8;
        let mut raw = [0u8; DATA_BYTES];
        raw[..5].copy_from_slice(&head);
        raw[5] = plain[3];
        FordV1Decoder::rebuild_raw_from_plain(&mut raw, &plain);
        raw
    }

    /// One burst of `pkt` (no CRC refresh), as the C emits it, merged.
    fn burst_wave(pkt: &[u8; DATA_BYTES], gap: u32) -> Vec<LevelDuration> {
        let mut wave = Vec::new();
        for _ in 0..ENC_PREAMBLE_PAIRS {
            FordV1Decoder::enc_add_level(&mut wave, true, ENC_SYNC_LONG_US);
            FordV1Decoder::enc_add_level(&mut wave, false, ENC_SYNC_LONG_US);
        }
        FordV1Decoder::enc_add_level(&mut wave, true, ENC_SYNC_LONG_US);
        FordV1Decoder::enc_add_level(&mut wave, false, ENC_SYNC_SHORT_US);
        for &b in pkt {
            for bit_i in (0..8).rev() {
                let bit = (b >> bit_i) & 1 != 0;
                FordV1Decoder::enc_add_level(&mut wave, bit, ENC_SYNC_SHORT_US);
                FordV1Decoder::enc_add_level(&mut wave, !bit, ENC_SYNC_SHORT_US);
            }
        }
        FordV1Decoder::enc_add_level(&mut wave, false, gap);
        wave
    }

    fn feed_all(wave: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = FordV1Decoder::new();
        wave.iter().filter_map(|p| dec.feed(p.level, p.duration_us)).collect()
    }

    #[test]
    fn descramble_roundtrip() {
        let plain: [u8; 9] = [0x11, 0x22, 0x33, 0x44, 0x55, 0x26, 0x77, 0x88, 0x99];
        let air = FordV1Decoder::encode_air_9bytes(&plain);
        assert_eq!(FordV1Decoder::plain_from_air(&air), Some(plain));
    }

    /// Frames ending in either bit decode: a final LOW half-bit merges into the gap and the
    /// partial-last-byte completion (`ford_v1_try_last_byte_variants`) recovers it.
    #[test]
    fn decodes_both_last_bit_values() {
        let mut seen = [false; 2];
        for counter in 0..64u32 {
            let raw = build_frame([0xC0, 0xFF, 0xEE, 0x1A, 0x2B], 0x1A2B_3C4D, 0x2, 0x1_2300 + counter, 0x5A);
            seen[(raw[16] & 1) as usize] = true;
            let frames = feed_all(&burst_wave(&raw, ENC_GAP_REPEAT_US));
            assert_eq!(frames.len(), 1, "counter {counter}");
            let f = &frames[0];
            assert_eq!(f.serial, Some(0x1A2B_3C4D));
            assert_eq!(f.button, Some(0x2));
            assert_eq!(f.counter, Some((0x2300 + counter) as u16));
            assert!(f.encoder_capable);
            let key2 = raw[7..15].iter().fold(0u64, |k, &b| (k << 8) | b as u64);
            assert_eq!(f.extra, Some(key2));
        }
        assert!(seen[0] && seen[1]);
    }

    /// Encoder → decoder through the pulse path: burst 0 only (pkt[4] = 0x08), the requested
    /// button, the counter bumped by one (20 bits, including the top nibble) and plain[8]'s
    /// relationship kept (so key2 matches a frame built with the shifted checksum byte).
    #[test]
    fn encode_decode_roundtrip_through_feed() {
        let raw = build_frame([0xC0, 0xFF, 0xEE, 0x1A, 0x2B], 0x1A2B_3C4D, 0x2, 0x3_FFFF, 0x77);
        let decoded = feed_all(&burst_wave(&raw, ENC_GAP_REPEAT_US)).pop().expect("decodes");
        assert_eq!(decoded.counter, Some(0xFFFF));
        for (button, code) in [(0x01u8, 0x1u8), (0x02, 0x2), (0x04, 0x4), (0x08, 0x8), (0x40, 0x2)] {
            let upload = FordV1Decoder::new().encode(&decoded, button).unwrap();
            let gap = upload.last().unwrap();
            assert!(!gap.level && gap.duration_us >= ENC_GAP_REPEAT_US, "ends with the 50ms gap");
            let frames = feed_all(&upload);
            assert_eq!(frames.len(), 1, "button {button:#04x}");
            let f = &frames[0];
            assert_eq!(f.serial, Some(0x1A2B_3C4D));
            assert_eq!(f.button, Some(code));
            assert_eq!(f.counter, Some(0x0000), "0x3FFFF + 1 = 0x40000");
            assert_eq!((f.data >> 16) as u8, ENC_BURST0_PKT4);
            // plain[8] keeps plain[8] - (plain[5] + plain[6] + plain[7]); new plain[6..8] = 0.
            let chk = 0x77u8.wrapping_sub(0x23).wrapping_sub(0xFF).wrapping_sub(0xFF);
            let p8 = chk.wrapping_add((code << 4) | 0x4);
            let expect = build_frame([0xC0, 0xFF, 0xEE, 0x1A, 0x08], 0x1A2B_3C4D, code, 0x4_0000, p8);
            let key2 = expect[7..15].iter().fold(0u64, |k, &b| (k << 8) | b as u64);
            assert_eq!(f.extra, Some(key2));
        }
    }

    /// The CRC gate rejects a burst with a corrupted payload byte.
    #[test]
    fn rejects_corrupted_burst() {
        let raw = build_frame([0xC0, 0xFF, 0xEE, 0x1A, 0x2B], 0x1A2B_3C4D, 0x1, 0x0456, 0x10);
        assert_eq!(feed_all(&burst_wave(&raw, ENC_GAP_REPEAT_US)).len(), 1);
        let mut bad = raw;
        bad[9] ^= 0x10;
        assert!(feed_all(&burst_wave(&bad, ENC_GAP_REPEAT_US)).is_empty());
    }

    /// As in the C: a frame whose first air bit is 0 is not decodable.
    #[test]
    fn first_bit_zero_is_not_decodable() {
        let raw = build_frame([0x40, 0xFF, 0xEE, 0x1A, 0x2B], 0x1A2B_3C4D, 0x1, 0x0456, 0x10);
        assert!(feed_all(&burst_wave(&raw, ENC_GAP_REPEAT_US)).is_empty());
    }

    /// A pre-port KAT export (extra = crc16 << 48 | strict << 40 | plain[4] << 32) still encodes:
    /// key2 is recovered from the stored CRC.
    #[test]
    fn legacy_extra_layout_still_encodes() {
        let raw = build_frame([0xC0, 0xFF, 0xEE, 0x1A, 0x2B], 0x1A2B_3C4D, 0x4, 0x5_1234, 0xA5);
        let mut decoded = feed_all(&burst_wave(&raw, ENC_GAP_REPEAT_US)).pop().unwrap();
        let pulses = |d: &DecodedSignal| -> Vec<(bool, u32)> {
            let w = FordV1Decoder::new().encode(d, 0x01).expect("encodes");
            w.iter().map(|l| (l.level, l.duration_us)).collect()
        };
        let fresh = pulses(&decoded);
        let crc = u16::from_be_bytes([raw[15], raw[16]]) as u64;
        decoded.extra = Some((crc << 48) | (1 << 40) | ((0x4Du64) << 32));
        assert_eq!(pulses(&decoded), fresh);
    }

    /// Transmit loopback at Ford V1's F4 preset (2-FSK ±14.28 kHz, ProtoPirate "V1 [F4]"): the
    /// re-encoded frame goes out as 2-FSK on the 500 kHz digital offset, is shifted back and read by
    /// KAT's current FM demodulator, and the registry decodes the same serial and button.
    #[test]
    fn fsk_transmit_loopback_at_f4() {
        let m = INFO.presets[0].modulation();
        assert!(matches!(m, crate::radio::preset::TxModulation::Fsk { deviation_hz } if (deviation_hz - 14_282.23).abs() < 0.01));
        let raw = build_frame([0xC0, 0xFF, 0xEE, 0x1A, 0x2B], 0x1A2B_3C4D, 0x2, 0x1_2300, 0x5A);
        let decoded = feed_all(&burst_wave(&raw, ENC_GAP_REPEAT_US)).pop().expect("decodes");
        let upload = FordV1Decoder::new().encode(&decoded, 0x04).unwrap();
        let caps = crate::radio::tx::fm_loopback(&upload, m);
        assert!(!caps.is_empty(), "FM demodulator emitted nothing");
        let hit = caps.iter().find_map(|p| crate::protocols::ProtocolRegistry::new().process_signal(p, 433_920_000));
        let (name, d) = hit.expect("loopback decodes");
        assert_eq!(name, "Ford V1");
        assert_eq!((d.serial, d.button), (Some(0x1A2B_3C4D), Some(0x4)));
    }

    /// Key files (ProtoPirate `subghz_protocol_decoder_ford_v1_serialize` / `_deserialize`): a
    /// strict frame round-trips with the C's lines in the C's order, the legacy halves-swapped
    /// layout and a missing Key_3 load as the C loads them, Serial/Btn/Cnt are required, and TX
    /// from a loaded key decodes back.
    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::{assert_round_trip, key};
        let dec = FordV1Decoder::new();
        let raw = build_frame([0xC0, 0xFF, 0xEE, 0x1A, 0x2B], 0x1A2B_3C4D, 0x2, 0x1_2345, 0x5A);
        let d = feed_all(&burst_wave(&raw, ENC_GAP_REPEAT_US)).pop().expect("decodes");
        let text = assert_round_trip(&dec, &d);
        assert!(
            text.ends_with(
                "Protocol: Ford V1\nBit: 136\nKey: 00 C0 FF EE 1A 2B 3C 4D\nSerial: 439041101\nBtn: 2\n\
                 Cnt: 74565\nKey_2: 1B 2A 3D 4C 20 66 01 5A\nKey_3: 00 00 4A 37\n"
            ),
            "{text}"
        );

        // A non-strict frame (plaintext only via the round trip, or none): header serial, 0, 0.
        let mut header_only = raw;
        header_only[5] ^= 0x01; // breaks the strict plain[3] == raw[5] link
        let c = FordV1Decoder::crc16(&header_only[3..15]);
        header_only[15..17].copy_from_slice(&c.to_be_bytes());
        let h = feed_all(&burst_wave(&header_only, ENC_GAP_REPEAT_US)).pop().expect("decodes");
        assert_eq!((h.button, h.counter), (Some(0), Some(0)));
        let text = assert_round_trip(&dec, &h);
        assert!(text.contains("\nBtn: 0\nCnt: 0\n"), "{text}");

        let file = key(&text_for(&d));
        let mut legacy = file.clone();
        let swap = |name: &str, k: &mut FlipperKey| {
            let v = FordV1Decoder::legacy_halves_bswap(k.hex_field(name).unwrap());
            k.fields.retain(|(n, _)| n != name);
            k.push_hex(name, v, 8);
        };
        legacy.key = FordV1Decoder::legacy_halves_bswap(legacy.key);
        legacy.key_bytes = legacy.key.to_be_bytes().to_vec();
        swap("Key_2", &mut legacy);
        assert_eq!(dec.from_flipper_key(&legacy).as_ref(), Some(&d), "legacy halves-swapped keys");

        // Key_3 in the legacy `HI LO 00 00` layout.
        let mut key3_legacy = file.clone();
        let crc = key3_legacy.hex_bytes_field("Key_3").unwrap();
        key3_legacy.fields.retain(|(n, _)| n != "Key_3");
        key3_legacy.push_hex_bytes("Key_3", &[crc[3], crc[2], 0, 0]);
        assert_eq!(dec.from_flipper_key(&key3_legacy).as_ref(), Some(&d));

        // No Key_3: CRC 0 fails, the C still loads the file (CRC ERR, no TX).
        let mut no_crc = file.clone();
        no_crc.fields.retain(|(n, _)| n != "Key_3");
        let n = dec.from_flipper_key(&no_crc).unwrap();
        assert!(!n.crc_valid && !n.encoder_capable);
        assert_eq!((n.serial, n.button, n.data, n.extra), (d.serial, d.button, d.data, d.extra));

        // Serial/Btn/Cnt are required; Bit must be 136.
        for field in ["Serial", "Btn", "Cnt", "Key_2"] {
            let mut k = file.clone();
            k.fields.retain(|(n, _)| n != field);
            assert!(dec.from_flipper_key(&k).is_none(), "{field}");
        }
        let mut wrong = file.clone();
        wrong.bits = 64;
        assert!(dec.from_flipper_key(&wrong).is_none());

        // TX from the loaded key.
        let loaded = crate::protocols::ProtocolRegistry::new().decode_key(&file).unwrap().1;
        let next = feed_all(&dec.encode(&loaded, 0x01).unwrap()).pop().expect("TX decodes");
        assert_eq!((next.serial, next.button, next.counter), (Some(0x1A2B_3C4D), Some(0x1), Some(0x2346)));
    }

    fn text_for(d: &DecodedSignal) -> String {
        let mut k = FordV1Decoder::new().to_flipper_key(d).unwrap();
        k.frequency = 433_920_000;
        k.preset = Some("FuriHalSubGhzPresetCustom".into());
        k.to_text()
    }

    /// Reference-oracle vectors: `KAT_KEY_VECTORS=<dir> cargo test oracle_vectors -- --ignored`.
    #[test]
    #[ignore]
    fn oracle_vectors() {
        let dec = FordV1Decoder::new();
        let frames = [
            build_frame([0xC0, 0xFF, 0xEE, 0x1A, 0x2B], 0x1A2B_3C4D, 0x2, 0x1_2345, 0x5A),
            build_frame([0x80, 0x12, 0x34, 0x56, 0x78], 0x0BAD_F00D, 0x8, 0x0_0042, 0xC3),
        ];
        for (n, raw) in frames.iter().enumerate() {
            let d = feed_all(&burst_wave(raw, ENC_GAP_REPEAT_US)).pop().unwrap();
            let upload = dec.encode(&d, d.button.unwrap()).unwrap();
            let sent = feed_all(&upload).pop().unwrap();
            let mut key = dec.to_flipper_key(&sent).unwrap();
            key.frequency = 433_920_000;
            key.set_preset(&crate::radio::preset::RfPreset::from_def(INFO.presets[0]));
            super::super::ford_v0::write_oracle_vector(&format!("ford_v1_{}", n + 1), &upload, &key);
        }
    }
}
