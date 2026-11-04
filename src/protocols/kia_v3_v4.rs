//! Kia V3/V4 decoder + encoder (KeeLoq).
//!
//! Port of ProtoPirate `protocols/kia_v3_v4.c` (HEAD 2026-10; the encoder is the 2026-05-19
//! rewrite "Fix KIA V5 encoder and KIA V3/4 encoder (CRC bruteforce)", which KAT had not picked
//! up). Flipper-ARF's `kia_v3_v4.c` decodes the same way (KeeLoq key compiled in, same value as
//! KAT's keystore type 10) and names the buttons; doctor-firmware matches ProtoPirate.
//!
//! Air format (315 / 433.92 MHz, FM): PWM, te_short 400µs / te_long 800µs (te_delta 150).
//! Preamble of short pairs (≥8 counted), then a 1000..1500µs sync whose level gives the version:
//! HIGH = V4, LOW = V3 (V3 frames are the bitwise inverse). Bits are the HIGH durations
//! (short 0, long 1); a LOW sync/gap ends the frame. 68 bits: bytes 0..3 = bit-reversed KeeLoq
//! hop, bytes 4..7 = bit-reversed serial28 | button4 << 28, then a 4-bit "CRC" whose algorithm is
//! unknown (displayed only). Emission requires the KeeLoq check: decrypting the hop with the KIA
//! manufacturer key (keystore type 10) must give the frame's button and serial LSB — exactly as
//! both C decoders, so KeeLoq frames of other makes (e.g. the Isuzu / Toyota Prius captures in
//! IMPORTS) are no longer reported as Kia with a failed check (KAT's earlier port did).
//! Counter = decrypted & 0xFFFF. `data` = bytes 0..7, `extra` = version (0 V4, 1 V3; the
//! encoder's state, as the C saves `KIAVersion`), and the display name is the C's version name
//! ("Kia V4" / "Kia V3"; `get()` maps both, and ProtoPirate's "KIA/HYU V3/V4", to this entry).
//!
//! Decoder (`kia_protocol_decoder_v3_v4_feed`), step for step: a HIGH > 1500µs during the data
//! aborts without a decode; a LOW > 1500µs or a sync-length pulse of either level ends the frame.
//!
//! Encoder (`kia_protocol_encoder_v3_v4_get_upload` + yield): plaintext = cnt16 | (serial & 0x3FF)
//! << 16 | btn << 28, KeeLoq-encrypted; per burst 12 preamble pairs, sync (V4: LOW 400, HIGH
//! 1200; V3: HIGH 400, LOW 1200), the 64 bits as PWM (V4: LOW first, 1 → 400/800, 0 → 800/400;
//! V3: HIGH first), the 4 "CRC" bits, end marker 800/800. The C sends 16 bursts (repeat 16),
//! patching the CRC nibble 0..15 — a brute force of the unknown CRC — so `encode()` returns all 16.
//! Like the emulate scene each press sends counter + 1; KAT's generic button maps to the C/ARF
//! codes: Lock 1, Unlock 2, Trunk 3, Panic 4 (ARF also names 8 "Horn"); others keep the captured
//! button.

use super::keeloq_common::{keeloq_decrypt, keeloq_encrypt};
use super::keys;
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

const TE_SHORT: u32 = 400;
const TE_LONG: u32 = 800;
const TE_DELTA: u32 = 150;
const MIN_COUNT_BIT: usize = 68;
const PREAMBLE_PAIRS: usize = 12;
const SYNC_DURATION: u32 = 1200;
const END_MARKER_US: u32 = 800;
const CRC_SWEEP_COUNT: u8 = 16;

/// Decoder states (`KiaV3V4DecoderStep`).
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    CheckPreamble,
    CollectRawBits,
}

/// Kia V3/V4 decoder.
pub struct KiaV3V4Decoder {
    step: DecoderStep,
    te_last: u32,
    header_count: u16,
    raw_bits: [u8; 32],
    raw_bit_count: u16,
    is_v3_sync: bool,
}

#[inline]
fn is_short(d: u32) -> bool {
    duration_diff!(d, TE_SHORT) < TE_DELTA
}

#[inline]
fn is_long(d: u32) -> bool {
    duration_diff!(d, TE_LONG) < TE_DELTA
}

/// The C's sync window, 1000 < d < 1500.
#[inline]
fn is_sync(d: u32) -> bool {
    d > 1000 && d < 1500
}

/// KIA manufacturer key (keystore type 10).
fn mf_key() -> u64 {
    keys::get_keystore().get_kia_mf_key()
}

impl KiaV3V4Decoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            te_last: 0,
            header_count: 0,
            raw_bits: [0; 32],
            raw_bit_count: 0,
            is_v3_sync: false,
        }
    }

    /// `kia_v3_v4_add_raw_bit`.
    fn add_raw_bit(&mut self, bit: bool) {
        if self.raw_bit_count < 256 {
            let byte_idx = (self.raw_bit_count / 8) as usize;
            let bit_idx = 7 - (self.raw_bit_count % 8);
            if bit {
                self.raw_bits[byte_idx] |= 1 << bit_idx;
            } else {
                self.raw_bits[byte_idx] &= !(1 << bit_idx);
            }
            self.raw_bit_count += 1;
        }
    }

    fn start_collect(&mut self, v3: bool) {
        self.step = DecoderStep::CollectRawBits;
        self.raw_bit_count = 0;
        self.is_v3_sync = v3;
        self.raw_bits = [0; 32];
    }

    /// `kia_v3_v4_process_buffer`: ≥68 bits whose KeeLoq hop decrypts to the frame's button and
    /// serial LSB.
    fn process_buffer(&mut self) -> Option<DecodedSignal> {
        if self.raw_bit_count < 68 {
            return None;
        }
        if self.is_v3_sync {
            let n = self.raw_bit_count.div_ceil(8) as usize;
            for b in &mut self.raw_bits[..n] {
                *b = !*b;
            }
        }
        let b = &self.raw_bits;
        let encrypted = u32::from_le_bytes([b[0], b[1], b[2], b[3]].map(u8::reverse_bits));
        let serial = ((((b[7] & 0xF0).reverse_bits()) as u32) << 24)
            | ((b[6].reverse_bits() as u32) << 16)
            | ((b[5].reverse_bits() as u32) << 8)
            | b[4].reverse_bits() as u32;
        let button = (b[7].reverse_bits() & 0xF0) >> 4;
        let decrypted = keeloq_decrypt(encrypted, mf_key());
        if (decrypted >> 28) as u8 != button || (decrypted >> 16) as u8 != serial as u8 {
            return None;
        }
        let version = self.is_v3_sync as u64;
        Some(DecodedSignal {
            serial: Some(serial),
            button: Some(button),
            counter: Some(decrypted as u16),
            crc_valid: true,
            data: u64::from_be_bytes(b[..8].try_into().unwrap()),
            data_count_bit: MIN_COUNT_BIT,
            encoder_capable: true,
            extra: Some(version),
            protocol_display_name: Some(if self.is_v3_sync { "Kia V3" } else { "Kia V4" }.to_string()),
        })
    }
}

/// Collect 68-bit Kia V3/V4 payload from level+duration pairs (for keeloq_generic fallback).
/// Returns (raw_bits[0..9], is_v3_sync) when a valid 68-bit burst is found.
pub fn collect_kia_v3_v4_bits(
    pairs: &[LevelDuration],
    invert_level: bool,
) -> Option<([u8; 9], bool)> {
    let mut step = DecoderStep::Reset;
    let mut te_last = 0u32;
    let mut header_count = 0u16;
    let mut raw_bits = [0u8; 32];
    let mut raw_bit_count = 0u16;
    let mut is_v3_sync = false;

    for pair in pairs {
        let level = if invert_level { !pair.level } else { pair.level };
        let duration = pair.duration_us;
        let is_short = duration_diff!(duration, TE_SHORT) < TE_DELTA;
        let is_long = duration_diff!(duration, TE_LONG) < TE_DELTA;
        let is_sync = duration > 1000 && duration < 1500;
        let is_very_long = duration > 1500;

        match step {
            DecoderStep::Reset => {
                if level && is_short {
                    step = DecoderStep::CheckPreamble;
                    te_last = duration;
                    header_count = 1;
                }
            }
            DecoderStep::CheckPreamble => {
                if level {
                    if is_short {
                        te_last = duration;
                    } else if is_sync && header_count >= 8 {
                        step = DecoderStep::CollectRawBits;
                        raw_bit_count = 0;
                        is_v3_sync = false;
                        raw_bits = [0; 32];
                    } else {
                        step = DecoderStep::Reset;
                    }
                } else {
                    if is_sync && header_count >= 8 {
                        step = DecoderStep::CollectRawBits;
                        raw_bit_count = 0;
                        is_v3_sync = true;
                        raw_bits = [0; 32];
                    } else if is_short && duration_diff!(te_last, TE_SHORT) < TE_DELTA {
                        header_count += 1;
                    } else if is_very_long {
                        step = DecoderStep::Reset;
                    }
                }
            }
            DecoderStep::CollectRawBits => {
                if level {
                    if is_sync || is_very_long {
                        if raw_bit_count >= 68 {
                            let mut out = [0u8; 9];
                            out.copy_from_slice(&raw_bits[0..9]);
                            return Some((out, is_v3_sync));
                        }
                        step = DecoderStep::Reset;
                    } else if is_short {
                        if raw_bit_count < 256 {
                            let byte_idx = (raw_bit_count / 8) as usize;
                            let bit_idx = 7 - (raw_bit_count % 8);
                            raw_bits[byte_idx] &= !(1 << bit_idx);
                            raw_bit_count += 1;
                        }
                    } else if is_long {
                        if raw_bit_count < 256 {
                            let byte_idx = (raw_bit_count / 8) as usize;
                            let bit_idx = 7 - (raw_bit_count % 8);
                            raw_bits[byte_idx] |= 1 << bit_idx;
                            raw_bit_count += 1;
                        }
                    } else {
                        step = DecoderStep::Reset;
                    }
                } else {
                    if is_sync || is_very_long {
                        if raw_bit_count >= 68 {
                            let mut out = [0u8; 9];
                            out.copy_from_slice(&raw_bits[0..9]);
                            return Some((out, is_v3_sync));
                        }
                        step = DecoderStep::Reset;
                    }
                }
            }
        }
    }
    None
}

impl ProtocolDecoder for KiaV3V4Decoder {
    fn name(&self) -> &'static str {
        "Kia V3/V4"
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
        self.raw_bit_count = 0;
        self.raw_bits = [0; 32];
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::Reset => {
                if level && is_short(duration) {
                    self.step = DecoderStep::CheckPreamble;
                    self.te_last = duration;
                    self.header_count = 1;
                }
            }
            DecoderStep::CheckPreamble => {
                if level {
                    if is_short(duration) {
                        self.te_last = duration;
                    } else if is_sync(duration) && self.header_count >= 8 {
                        self.start_collect(false);
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else if is_sync(duration) {
                    if self.header_count >= 8 {
                        self.start_collect(true);
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else if is_short(duration) && is_short(self.te_last) {
                    self.header_count = self.header_count.wrapping_add(1);
                } else if duration > 1500 {
                    self.step = DecoderStep::Reset;
                }
            }
            DecoderStep::CollectRawBits => {
                if level {
                    if is_sync(duration) {
                        self.step = DecoderStep::Reset;
                        return self.process_buffer();
                    } else if is_short(duration) {
                        self.add_raw_bit(false);
                    } else if is_long(duration) {
                        self.add_raw_bit(true);
                    } else {
                        self.step = DecoderStep::Reset;
                    }
                } else if is_sync(duration) || duration > 1500 {
                    self.step = DecoderStep::Reset;
                    return self.process_buffer();
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
        let captured = decoded.button.unwrap_or(0);
        let button = match button {
            0x01 => 0x1,
            0x02 => 0x2,
            0x04 => 0x3,
            0x08 => 0x4,
            _ => captured,
        };
        let counter = decoded.counter.unwrap_or(0).wrapping_add(1);
        let v4 = decoded.extra.unwrap_or(0) == 0;
        let mut out = Vec::new();
        for crc in 0..CRC_SWEEP_COUNT {
            out.extend(build_burst(serial, button, counter, v4, crc));
        }
        Some(out)
    }
}

impl Default for KiaV3V4Decoder {
    fn default() -> Self {
        Self::new()
    }
}

/// `kia_v3_v4_emit_bit_pwm`: V4 sends (LOW, HIGH), V3 (HIGH, LOW); 1 = 400/800, 0 = 800/400.
fn push_bit(out: &mut Vec<LevelDuration>, bit: bool, v4: bool) {
    let (first, second) = if bit { (TE_SHORT, TE_LONG) } else { (TE_LONG, TE_SHORT) };
    out.push(LevelDuration::new(!v4, first));
    out.push(LevelDuration::new(v4, second));
}

/// One burst of `kia_protocol_encoder_v3_v4_get_upload` with CRC nibble `crc` (the yield loop's
/// `crc_iter` patch).
fn build_burst(serial: u32, button: u8, counter: u16, v4: bool, crc: u8) -> Vec<LevelDuration> {
    let plaintext = counter as u32 | ((serial & 0x3FF) << 16) | (((button & 0x0F) as u32) << 28);
    let encrypted = keeloq_encrypt(plaintext, mf_key());
    let serial_btn = (serial & 0x0FFF_FFFF) | (((button & 0x0F) as u32) << 28);
    // kia_v3_v4_build_tx_bitstream: the 64-bit key bit-reversed, sent MSB first.
    let tx_key = (((serial_btn as u64) << 32) | encrypted as u64).reverse_bits();
    let mut out = Vec::with_capacity(PREAMBLE_PAIRS * 2 + 2 + 2 * MIN_COUNT_BIT + 2);
    for _ in 0..PREAMBLE_PAIRS {
        out.push(LevelDuration::new(!v4, TE_SHORT));
        out.push(LevelDuration::new(v4, TE_SHORT));
    }
    out.push(LevelDuration::new(!v4, TE_SHORT));
    out.push(LevelDuration::new(v4, SYNC_DURATION));
    for i in (0..64).rev() {
        push_bit(&mut out, (tx_key >> i) & 1 != 0, v4);
    }
    for b in (0..4).rev() {
        push_bit(&mut out, (crc >> b) & 1 != 0, v4);
    }
    out.push(LevelDuration::new(!v4, END_MARKER_US));
    out.push(LevelDuration::new(v4, END_MARKER_US));
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn feed_all(wire: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = KiaV3V4Decoder::new();
        wire.iter().filter_map(|p| dec.feed(p.level, p.duration_us)).collect()
    }

    fn signal(serial: u32, button: u8, counter: u16, version: u64) -> DecodedSignal {
        DecodedSignal {
            serial: Some(serial),
            button: Some(button),
            counter: Some(counter),
            crc_valid: true,
            data: 0,
            data_count_bit: MIN_COUNT_BIT,
            encoder_capable: true,
            extra: Some(version),
            protocol_display_name: None,
        }
    }

    /// The real `IMPORTS/KIA/KIA V4 N1 RAW.sub` / `KIA V3 N1 RAW.sub` frames as the C
    /// (`kia_v3_v4.c` on the host, KIA key) decodes them: key, serial, button, counter, version.
    const C_CAPTURE: &[(u64, u32, u8, u16, u64)] = &[
        (0x20FD_5790_60B7_0384, 0x01C0_ED06, 0x2, 0x0013, 0),
        (0x295D_F629_60B7_0304, 0x00C0_ED06, 0x2, 0x0026, 1),
    ];

    /// Encode → decode for both versions: fields, counter + 1, the version. Back to back, a burst
    /// only ends at the next burst's sync and the decoder then skips that burst's data (as the C
    /// does on its own 16-burst transmission: CRC 0, 2, .., E decode), so 8 of the 16 decode.
    #[test]
    fn round_trip_both_versions() {
        keys::load_keystore_from_embedded();
        for &(key, serial, button, counter, version) in C_CAPTURE {
            let d = {
                let mut dec = KiaV3V4Decoder::new();
                let wire = build_burst(serial, button, counter, version == 0, 0);
                let mut found = None;
                for p in wire.iter().chain(std::iter::once(&LevelDuration::new(false, 30_000))) {
                    found = found.or(dec.feed(p.level, p.duration_us));
                }
                found.expect("single burst + trailing gap decodes")
            };
            assert_eq!(d.data, key, "the C's key for the captured fields");
            let wire = KiaV3V4Decoder::new().encode(&signal(serial, button, counter, version), 0x04).unwrap();
            let got = feed_all(&wire);
            assert_eq!(got.len(), CRC_SWEEP_COUNT as usize / 2);
            for d in &got {
                assert_eq!((d.serial, d.button, d.counter), (Some(serial), Some(0x3), Some(counter + 1)));
                assert_eq!(d.extra, Some(version));
                let name = if version == 1 { "Kia V3" } else { "Kia V4" };
                assert_eq!(d.protocol_display_name.as_deref(), Some(name));
            }
        }
    }

    /// The decoder drops frames whose hop does not decrypt to their own button / serial LSB
    /// (here: the right frame under the wrong key).
    #[test]
    fn rejects_frames_failing_the_keeloq_check() {
        keys::load_keystore_from_embedded();
        let mut wire = build_burst(0x01C0_ED06, 0x2, 0x14, true, 0);
        wire.push(LevelDuration::new(false, 30_000));
        assert_eq!(feed_all(&wire).len(), 1);
        let mut bad = wire.clone();
        // Flip one hop bit (first data bit after preamble + sync).
        let i = PREAMBLE_PAIRS * 2 + 2;
        let d = bad[i + 1].duration_us;
        bad[i + 1].duration_us = if d == TE_LONG { TE_SHORT } else { TE_LONG };
        bad[i].duration_us = if bad[i].duration_us == TE_LONG { TE_SHORT } else { TE_LONG };
        assert!(feed_all(&bad).is_empty());
    }

    /// Real fob data: `IMPORTS/HYUNDAI/HYU V4 N1.sub` (Flipper-ARF "KIA/HYU V4", CRC field 6) passes
    /// the KeeLoq check, and the encoder's burst for its fields carries the same 64 bits.
    #[test]
    fn real_key_file_rebuilds() {
        keys::load_keystore_from_embedded();
        let key = 0xC3B1_790C_8FC0_C864u64;
        let mut dec = KiaV3V4Decoder::new();
        dec.raw_bits[..8].copy_from_slice(&key.to_be_bytes());
        dec.raw_bits[8] = 0x6 << 4;
        dec.raw_bit_count = 68;
        let d = dec.process_buffer().expect("KeeLoq check passes");
        let (serial, button, counter) = (d.serial.unwrap(), d.button.unwrap(), d.counter.unwrap());
        let mut wire = build_burst(serial, button, counter, true, 6);
        wire.push(LevelDuration::new(false, 30_000));
        let got = feed_all(&wire);
        assert_eq!((got.len(), got[0].data, got[0].extra), (1, key, Some(0)));
    }
}
