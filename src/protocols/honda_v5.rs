//! Honda V5 protocol decoder (**decode only**, as the reference)
//!
//! Ported from FoxFW `lib/subghz/protocols/honda_v5.c` and `honda_v5.h` (itself a port of the
//! KeeLoq-RX `arke_honda.py` `HondaV5` class); no other reference carries it. The C has no encoder
//! (`.encoder = NULL`), so neither does KAT: Replay of the recorded pulses is the only TX.
//!
//! **Air format** (a Honda keyfob; the captures use the Honda1 FSK custom preset): Manchester with
//! te 63 / 126 µs (delta 20) decoded by a custom four-state machine,
//! `honda_man_lut = {0x01, 0x91, 0x9B, 0xFB}` indexed by the previous state and shifted by the
//! pulse type (high short 0, low short 2, high long 4, low long 6; anything else 8 = invalid).
//! The C flags the protocol AM; the real captures are FSK, so KAT declares AM/FM (see the
//! deviation below).
//!
//! **Decoder** (`feed` / `decode_timing` / `process`): Reset waits for a short pulse; Collect feeds
//! every pulse to the state machine and stores the bits it produces (max 400). A pulse longer
//! than `te_long + 3 * te_delta` = 186 µs, an invalid pulse, or reaching 400 bits ends the burst;
//! with at least 130 bits collected `process` runs. `process` skips the leading preamble 1-bits,
//! then reads one of two frames:
//!   * **Standard, 135 bits**: restores up to two leading data 1-bits the preamble skip absorbed;
//!     the frame is laid out from bit position 1 (the first bit is implicit), header
//!     `0x6C0F62` (bytes 0..3), byte 3 retransmit marker (0x08 / 0x09 / 0x0A, 0x0A = `rtx`),
//!     serial = bytes 4..8 (32 bits), button = byte 8, counter = bytes 10..12 (16 bits),
//!     rolling = bytes 12..16 (32 bits), byte 16 = CRC-8 (MSB-first, poly 0x2F, init 0x03) over
//!     bytes 0..16. Emission is gated on the header and the CRC.
//!   * **Alternate, 168 bits** (tried first when at least 168 bits follow the preamble): header
//!     `0x6428`, serial = bytes 13..17, button = byte 17, "counter" = byte 14 (as the C: it
//!     overlaps the serial), CRC-8 reflected (poly 0x4D, init 0) over bytes 0..20 against byte 20.
//!     A frame with this header is never read as a standard one.
//!
//! **KAT fields**: `data` = frame bytes 0..8 big-endian (the C `generic.data`), `data_count_bit`
//! = 135 or 168, `serial`, `button` and `counter` as above, `crc_valid` true (every emitted frame
//! passed its CRC), `extra` = frame bytes 8..16 (the C `data_mid`; the rolling word is its low
//! 32 bits). The C's `data_ext` (frame bytes 16..20: the CRC byte of a standard frame) is not
//! kept; it is recomputed.
//!
//! **Key files** (`serialize` / `deserialize`): Protocol "Honda V5", Bit (135 or 168), Key = the
//! first 8 frame bytes, then Serial, Btn, Cnt, CRC, Rolling, Variant, MidHi, MidLo and Ext, all
//! decimal (for a standard frame Ext = CRC). KAT writes and reads the standard 135-bit frames; an
//! alternate frame is not written (its Ext is not kept) and Bit 168 is not loaded.
//!
//! **Evidence and deviation from Honda Static.** `Lock_honda.sub` and `Unlock_honda.sub` (IMPORTS,
//! Honda1 custom preset, 433.657 MHz; 23 and 17 frames) used to decode as "Honda Static"
//! (ProtoPirate `honda_static.c`) through its bit-reversed fallback, the one path the C does not
//! checksum: every frame had crc false, button 1 in both files, a counter of 0x56BE7F in 40 of 41
//! frames, and a serial that took three values (0xB782388, 0xB7823A8, 0xB7823C8) within one file.
//! Those three differ exactly as the V5 frame's byte-3 retransmit marker does (0x08 / 0x0A /
//! 0x09), i.e. the "serial" was header bits misread. Honda V5 reads the same frames with a valid
//! CRC-8 on all 40, one constant serial 0xB53EFF72, button 0x21 in the Lock file and 0x22 in the
//! Unlock file, and a counter that rises by one per press (7154..7162 in Unlock, 7175..7191 in
//! Lock, with gaps where a press was not captured; each press is sent two or three times with the
//! same counter). The registry therefore tries Honda V5 before Honda Static, and the flag differs
//! from the C: AM/FM instead of AM, because the captures are FSK. See `honda_static.rs` and
//! `docs/honda_v5.md`.

use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (see `info.rs`): the C's `SubGhzProtocolFlag_433 | 315 | AM`; Rf is AM/FM because the
/// real captures are FSK (module doc).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::Both,
    encryption: "Rolling Code",
    make: "Honda/Acura",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&HONDA1, &AM650],
};

/// The two captures' buttons: 0x21 in the Lock file, 0x22 in the Unlock file (rtl_433's Honda-KR5V2X1X
/// decoder names the same event codes for the same remotes).
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    match code {
        0x21 => Some("Lock"),
        0x22 => Some("Unlock"),
        _ => None,
    }
}

const NAME: &str = "Honda V5";
const TE_SHORT: u32 = 63;
const TE_LONG: u32 = 126;
const TE_DELTA: u32 = 20;
const MIN_BITS: usize = 130;
const MAX_BITS: usize = 400;
const STD_BITS: usize = 135;
const ALT_BITS: usize = 168;
const MAN_LUT: [u8; 4] = [0x01, 0x91, 0x9B, 0xFB];

#[derive(Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    Collect,
}

/// Honda V5 decoder (matches SubGhzProtocolDecoderHondaV5).
pub struct HondaV5Decoder {
    step: Step,
    man_state: u8,
    /// One entry per decoded bit (0 or 1), up to MAX_BITS.
    bits: Vec<u8>,
    /// Frame found by `process` during the current `feed`.
    found: Option<DecodedSignal>,
}

/// `honda_v5_timing_type`: 0/2 short high/low, 4/6 long high/low, 8 other.
fn timing_type(duration: u32, level: bool) -> u8 {
    if duration.abs_diff(TE_SHORT) <= TE_DELTA {
        return if level { 0 } else { 2 };
    }
    if duration.abs_diff(TE_LONG) <= TE_DELTA {
        return if level { 4 } else { 6 };
    }
    8
}

/// CRC-8 MSB-first (`honda_v5_crc8`).
fn crc8(m: &[u8], poly: u8, init: u8) -> u8 {
    let mut r = init;
    for &b in m {
        r ^= b;
        for _ in 0..8 {
            r = if r & 0x80 != 0 { (r << 1) ^ poly } else { r << 1 };
        }
    }
    r
}

/// CRC-8 reflected / LSB-first (`honda_v5_crc8_reflected`).
fn crc8_reflected(m: &[u8], poly: u8, init: u8) -> u8 {
    let mut crc = init;
    for &b in m {
        crc ^= b;
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ poly } else { crc >> 1 };
        }
    }
    crc
}

fn be64(raw: &[u8]) -> u64 {
    raw.iter().take(8).fold(0u64, |a, &b| (a << 8) | u64::from(b))
}

/// `honda_v5_process` on a collected bit stream; the decode is the C's `honda_v5_emit`.
fn process(collected: &[u8]) -> Option<DecodedSignal> {
    let mut bits = collected.to_vec();
    let n = bits.len().min(MAX_BITS);
    let mut start = 0usize;
    while start < n && bits[start] == 1 {
        start += 1;
    }
    let mut data_len = n - start;

    // Alternate 168-bit variant.
    if data_len >= ALT_BITS {
        let mut raw = [0u8; 22];
        for i in 0..ALT_BITS {
            if start + i >= n {
                break;
            }
            let bi = i / 8;
            if bi < 21 && bits[start + i] != 0 {
                raw[bi] |= 1 << (7 - (i % 8));
            }
        }
        if u16::from_be_bytes([raw[0], raw[1]]) == 0x6428 {
            if crc8_reflected(&raw[..20], 0x4D, 0) != raw[20] {
                return None;
            }
            let serial = u32::from_be_bytes([raw[13], raw[14], raw[15], raw[16]]);
            return Some(DecodedSignal {
                serial: Some(serial),
                button: Some(raw[17]),
                counter: Some(u16::from(raw[14])),
                crc_valid: true,
                data: be64(&raw),
                data_count_bit: ALT_BITS,
                encoder_capable: false,
                extra: Some(be64(&raw[8..])),
                protocol_display_name: None,
            });
        }
    }

    // Standard 135-bit variant.
    if data_len < STD_BITS {
        let missing = STD_BITS - data_len;
        if missing > 0 && missing <= 2 && start >= missing {
            for _ in 0..missing {
                start -= 1;
                bits[start] = 1;
            }
            data_len = STD_BITS;
        }
    }
    if data_len < STD_BITS {
        return None;
    }

    let mut raw = [0u8; 17];
    for i in 0..STD_BITS {
        if start + i >= n {
            break;
        }
        let pos = i + 1;
        let bi = pos / 8;
        if bi < 17 {
            raw[bi] |= bits[start + i] << (7 - (pos % 8));
        }
    }
    let header = (u32::from(raw[0]) << 16) | (u32::from(raw[1]) << 8) | u32::from(raw[2]);
    if header != 0x6C0F62 {
        return None;
    }
    if crc8(&raw[..16], 0x2F, 0x03) != raw[16] {
        return None;
    }
    Some(DecodedSignal {
        serial: Some(u32::from_be_bytes([raw[4], raw[5], raw[6], raw[7]])),
        button: Some(raw[8]),
        counter: Some(u16::from_be_bytes([raw[10], raw[11]])),
        crc_valid: true,
        data: be64(&raw),
        data_count_bit: STD_BITS,
        encoder_capable: false,
        extra: Some(be64(&raw[8..])),
        protocol_display_name: None,
    })
}

impl HondaV5Decoder {
    pub fn new() -> Self {
        Self { step: Step::Reset, man_state: 1, bits: Vec::with_capacity(MAX_BITS), found: None }
    }

    /// `honda_v5_man_step`: advance the state machine by one pulse type; Some(bit) when it yields one.
    fn man_step(&mut self, tt: u8) -> Option<u8> {
        if tt == 8 {
            self.man_state = 1;
            return None;
        }
        if self.man_state >= 4 {
            self.man_state = 1;
            return None;
        }
        let ns = (MAN_LUT[self.man_state as usize] >> tt) & 0x3;
        if self.man_state == ns {
            return None;
        }
        self.man_state = ns;
        match ns {
            2 => Some(0),
            1 => Some(1),
            _ => None,
        }
    }

    /// `honda_v5_reset_collect`.
    fn reset_collect(&mut self) {
        self.step = Step::Reset;
        self.man_state = 1;
        self.bits.clear();
    }

    /// Run `process` on the collected bits; a frame is kept for `feed` to return.
    fn run_process(&mut self) {
        if let Some(d) = process(&self.bits) {
            self.found = Some(d);
        }
    }

    /// `honda_v5_decode_timing`.
    fn decode_timing(&mut self, level: bool, duration: u32) {
        let tt = timing_type(duration, level);
        if tt != 8 {
            if let Some(bit) = self.man_step(tt) {
                if self.bits.len() < MAX_BITS {
                    self.bits.push(bit);
                }
                if self.bits.len() >= MAX_BITS {
                    self.run_process();
                    self.reset_collect();
                }
            }
        } else {
            if self.bits.len() >= MIN_BITS {
                self.run_process();
            }
            self.reset_collect();
        }
    }
}

impl Default for HondaV5Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for HondaV5Decoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: TE_SHORT, te_long: TE_LONG, te_delta: TE_DELTA, min_count_bit: STD_BITS }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.step = Step::Reset;
        self.man_state = 1;
        self.bits.clear();
        self.found = None;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        self.found = None;
        let gap = TE_LONG + TE_DELTA * 3;
        match self.step {
            Step::Reset => {
                if duration.abs_diff(TE_SHORT) <= TE_DELTA {
                    self.step = Step::Collect;
                    self.man_state = 1;
                    self.bits.clear();
                    self.decode_timing(level, duration);
                }
            }
            Step::Collect => {
                if duration > gap {
                    if self.bits.len() >= MIN_BITS {
                        self.run_process();
                    }
                    self.reset_collect();
                } else {
                    self.decode_timing(level, duration);
                }
            }
        }
        self.found.take()
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }

    /// `subghz_protocol_decoder_honda_v5_deserialize`, for 135-bit frames (module doc).
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != STD_BITS || key.u32_field("Variant").is_some_and(|v| v as usize != STD_BITS) {
            return None;
        }
        let mid = (u64::from(key.u32_field("MidHi")?) << 32) | u64::from(key.u32_field("MidLo")?);
        Some(DecodedSignal {
            serial: key.u32_field("Serial"),
            button: key.u32_field("Btn").map(|v| v as u8),
            counter: key.u32_field("Cnt").map(|v| v as u16),
            crc_valid: true,
            data: key.key_u64()?,
            data_count_bit: STD_BITS,
            encoder_capable: false,
            extra: Some(mid),
            protocol_display_name: None,
        })
    }

    /// `subghz_protocol_decoder_honda_v5_serialize`, in the C's order (135-bit frames).
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != STD_BITS {
            return None;
        }
        let mid = decoded.extra?;
        let mut raw = decoded.data.to_be_bytes().to_vec();
        raw.extend_from_slice(&mid.to_be_bytes());
        let crc = u32::from(crc8(&raw, 0x2F, 0x03));
        let mut key = FlipperKey::for_protocol(NAME, STD_BITS, decoded.data);
        key.push_u32("Serial", decoded.serial?);
        key.push_u32("Btn", decoded.button?.into());
        key.push_u32("Cnt", decoded.counter?.into());
        key.push_u32("CRC", crc);
        key.push_u32("Rolling", mid as u32);
        key.push_u32("Variant", STD_BITS as u32);
        key.push_u32("MidHi", (mid >> 32) as u32);
        key.push_u32("MidLo", mid as u32);
        key.push_u32("Ext", crc);
        Some(key)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;

    /// Decodes of a capture, in order, with the stream as the file's pulses.
    fn decode_file(rel: &str) -> Option<Vec<DecodedSignal>> {
        let path = Path::new(rel);
        if !path.exists() {
            eprintln!("Skip: {rel} not found (run from crate root)");
            return None;
        }
        let (_, pairs) = crate::export::flipper::import_sub_raw(path).unwrap();
        let mut dec = HondaV5Decoder::new();
        Some(pairs.iter().filter_map(|p| dec.feed(p.level, p.duration_us)).collect())
    }

    /// A standard frame's bit stream (as `process` collects it): preamble 1-bits, then the frame
    /// without its implicit first bit.
    fn std_stream(serial: u32, button: u8, counter: u16, rolling: u32) -> Vec<u8> {
        let mut raw = vec![0x6C, 0x0F, 0x62, 0x08];
        raw.extend(serial.to_be_bytes());
        raw.extend([button, 0x00]);
        raw.extend(counter.to_be_bytes());
        raw.extend(rolling.to_be_bytes());
        raw.push(crc8(&raw, 0x2F, 0x03));
        let mut bits = vec![1u8; 20];
        for pos in 1..=STD_BITS {
            bits.push((raw[pos / 8] >> (7 - pos % 8)) & 1);
        }
        bits
    }

    #[test]
    fn unlock_honda_sub_is_honda_v5() {
        let Some(frames) = decode_file("IMPORTS/honda/Unlock_honda.sub") else { return };
        assert_eq!(frames.len(), 17);
        assert!(frames.iter().all(|f| f.crc_valid && f.data_count_bit == STD_BITS));
        assert!(frames.iter().all(|f| f.serial == Some(0xB53E_FF72) && f.button == Some(0x22)));
        let counters: Vec<u16> = frames.iter().map(|f| f.counter.unwrap()).collect();
        let want: Vec<u16> = (7154..=7161).flat_map(|c| [c, c]).chain([7162]).collect();
        assert_eq!(counters, want);
        assert_eq!(frames[0].data, 0x6C0F_6208_B53E_FF72);
        assert_eq!(native_buttons(NAME, 0x22), Some("Unlock"));
    }

    #[test]
    fn lock_honda_sub_is_honda_v5() {
        let Some(frames) = decode_file("IMPORTS/honda/Lock_honda.sub") else { return };
        assert_eq!(frames.len(), 23);
        assert!(frames.iter().all(|f| f.crc_valid && f.data_count_bit == STD_BITS));
        assert!(frames.iter().all(|f| f.serial == Some(0xB53E_FF72) && f.button == Some(0x21)));
        let counters: Vec<u16> = frames.iter().map(|f| f.counter.unwrap()).collect();
        assert!(counters.windows(2).all(|w| w[0] <= w[1]), "counter never falls: {counters:?}");
        let mut presses = counters.clone();
        presses.dedup();
        assert_eq!(
            presses,
            [7175, 7176, 7178, 7180, 7181, 7182, 7183, 7184, 7185, 7186, 7187, 7188, 7189, 7190, 7191]
        );
        assert_eq!(native_buttons(NAME, 0x21), Some("Lock"));
    }

    /// The header and the CRC gate emission; leading data 1-bits the preamble skip swallowed are
    /// restored.
    #[test]
    fn process_gating() {
        let good = std_stream(0xB53E_FF72, 0x21, 7175, 0x5C48_0D69);
        let d = process(&good).expect("valid frame");
        assert_eq!((d.serial, d.button, d.counter), (Some(0xB53E_FF72), Some(0x21), Some(7175)));
        assert_eq!(d.extra.map(|m| m as u32), Some(0x5C48_0D69));
        assert!(d.crc_valid);
        // The frame's first two data bits (0x6C = 0110 1100 from bit position 1) are 1s the
        // preamble skip swallows; `process` restores them.
        assert_eq!(good.iter().take_while(|&&b| b == 1).count(), 22);
        let mut bad_crc = good.clone();
        let last = bad_crc.len() - 1;
        bad_crc[last] ^= 1;
        assert!(process(&bad_crc).is_none());
        let mut bad_header = good;
        bad_header[24] ^= 1;
        assert!(process(&bad_header).is_none());
    }

    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::assert_round_trip;
        let d = process(&std_stream(0xB53E_FF72, 0x22, 7154, 0x1234_5678)).unwrap();
        let text = assert_round_trip(&HondaV5Decoder::new(), &d);
        assert!(text.contains("Protocol: Honda V5\nBit: 135\nKey: 6C 0F 62 08 B5 3E FF 72\n"), "{text}");
        assert!(text.contains("Serial: 3040804722\nBtn: 34\nCnt: 7154\nCRC: "), "{text}");
    }
}
