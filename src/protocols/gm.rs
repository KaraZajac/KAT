//! GM (General Motors) car-remote decoder + replay encoder
//!
//! Ported from Flipper-ARF `lib/subghz/protocols/gm.c` / `gm.h` (`GM_PROTOCOL_NAME = "GM"`), the
//! only copy in REFERENCES (ProtoPirate and doctor-firmware have no GM protocol). gm.c is itself a
//! port of rtl_433's `src/devices/gm_car_remote.c` ("GM ABO1502T Car Remote", -f 314.9M); its PHY
//! constants (OOK PPM, 300/500 µs gaps, 20 ms reset) and both integrity checks were cross-checked
//! against that upstream decoder and match.
//!
//! **PHY** — OOK pulse-position modulation. Every bit is a fixed-width HIGH pulse followed by a LOW
//! gap whose width carries the bit: gap ≈300 µs → 0, ≈500 µs → 1 (`te_delta` 120 µs: 0 = 181..419 µs,
//! 1 = 381..619 µs; 0 wins the 381..419 overlap, as in the C). HIGH pulses are only range-checked
//! (100..1000 µs). A LOW ≥12 ms ends the burst; any other out-of-range HIGH/LOW also ends it (after a
//! final window check). RF: AM/OOK, 315 MHz (the C also flags 433 MHz).
//!
//! **Frame** — 112 bits = 14 bytes, MSB-first:
//!
//! | byte(s)  | field |
//! |----------|-------|
//! | b0       | 0xFF wake-up sanity byte |
//! | b1       | unknown (rtl_433: "possibly part of the ID") |
//! | b2       | high nibble: button checksum; low nibble: button (`b2 & 0x07`, as in the C and rtl_433) |
//! | b3..b6   | 32-bit ID → `serial` |
//! | b7..b9   | 24-bit plaintext sequence counter → `counter` holds its LOW 16 bits (b8..b9) |
//! | b10..b12 | 24-bit encrypted field (cipher not public) |
//! | b13      | checksum byte |
//!
//! Button codes (C `gm_button_name`): 1 = Unlock, 2 = Lock, 3 = Trunk, 4 = Panic. `button` carries the
//! native code.
//!
//! A long wake-up preamble precedes the payload and may be truncated, so — like the C — the decoder
//! keeps a rolling window of the LAST 112 bits (a 14-byte shift register) and tests it after every bit
//! once ≥112 bits have arrived, and again when the burst ends. Testing every alignment also covers
//! rtl_433's framing (it takes the 112 bits that end one bit before the last).
//!
//! **Gating** — both additive checks from gm.c / rtl_433, plus the sanity byte (the C's only emission
//! path; the sums are kept untruncated, so an all-zero window is rejected too):
//! - `b0 == 0xFF`;
//! - button check: `s = hi(b2) + lo(b2)`, `s != 0 && s & 0x0F == 0`;
//! - full check: `s = Σ b1..b13`, `s != 0 && s & 0xFF == 0`.
//!
//! A random window passes with p ≈ 2⁻²⁰. The C also suppresses a frame identical to the last reported
//! one until `reset()`; that is kept, but KAT's registry resets every decoder after each decode, so
//! every repeat is still reported.
//!
//! **DecodedSignal packing** (112 bits > 64): `data` = b0..b7 (first 64 wire bits, big-endian),
//! `extra` = b8..b13 (last 48 wire bits, right-aligned), `data_count_bit` = 112.
//!
//! **Encoder** — byte-exact REPLAY of the 14 captured bytes, as in the C: the 24-bit encrypted field
//! cannot be regenerated, so the C disables custom buttons and only replays. The `button` argument is
//! therefore ignored. Waveform: for each bit, HIGH 300 µs + LOW 300/500 µs, then a 12 ms guard LOW.
//! One deviation from the C: a closing 300 µs HIGH pulse is inserted before the guard. In the C upload
//! the 112th bit's LOW is immediately followed by the guard LOW; on air (and in KAT, which merges
//! same-level durations) the two fuse into one ≥12 ms gap, so the last bit would be lost and the C's own
//! decoder would see only 111 bits. The C's yield-level repeat count (5) is not replicated (KAT
//! transmits the `encode()` output once, like the other ports). Note: rtl_433 needs ≥113 bits and drops
//! the last one, so real fobs evidently send one bit after the payload; its value is not captured by the
//! 112-bit window, so the replay does not synthesize it (unverified against a vehicle).

use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

/// Short gap → bit 0 (also the transmitted pulse width).
const TE_SHORT: u32 = 300;
/// Long gap → bit 1.
const TE_LONG: u32 = 500;
const TE_DELTA: u32 = 120;
/// Fixed-width HIGH pulse bounds (inclusive, as in the C).
const PULSE_MIN_US: u32 = 100;
const PULSE_MAX_US: u32 = 1000;
/// End-of-frame LOW gap (C `GM_RESET_US`, below rtl_433's 20 ms reset limit).
const RESET_US: u32 = 12_000;

const WIRE_BYTES: usize = 14;
const WIRE_BITS: usize = 112;

/// Decoder steps (matches `GMDecoderStep`).
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    Data,
}

/// GM decoder (matches `SubGhzProtocolDecoderGM`).
pub struct GmDecoder {
    step: DecoderStep,
    /// Rolling window of the last 112 bits, MSB-first (C `bits`).
    bits: [u8; WIRE_BYTES],
    /// Bits collected this burst (saturating, C `bit_count`).
    bit_count: u16,
    /// Last reported frame, for the C's repeat de-dup.
    last_raw: [u8; WIRE_BYTES],
    last_raw_valid: bool,
}

impl GmDecoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            bits: [0; WIRE_BYTES],
            bit_count: 0,
            last_raw: [0; WIRE_BYTES],
            last_raw_valid: false,
        }
    }

    /// `gm_add_nibbles`: untruncated sum of both nibbles of every byte.
    fn add_nibbles(b: &[u8]) -> u32 {
        b.iter()
            .map(|&x| u32::from(x >> 4) + u32::from(x & 0x0F))
            .sum()
    }

    /// `gm_add_bytes`: untruncated byte sum.
    fn add_bytes(b: &[u8]) -> u32 {
        b.iter().map(|&x| u32::from(x)).sum()
    }

    /// `gm_frame_valid`: wake-up byte + button nibble check + full byte-sum check.
    fn frame_valid(raw: &[u8; WIRE_BYTES]) -> bool {
        if raw[0] != 0xFF {
            return false;
        }
        let btn_ck = Self::add_nibbles(&raw[2..3]);
        if btn_ck == 0 || (btn_ck & 0x0F) != 0 {
            return false;
        }
        let full_ck = Self::add_bytes(&raw[1..WIRE_BYTES]);
        if full_ck == 0 || (full_ck & 0xFF) != 0 {
            return false;
        }
        true
    }

    /// `gm_id`: 32-bit ID, b3..b6.
    fn id(raw: &[u8; WIRE_BYTES]) -> u32 {
        u32::from_be_bytes([raw[3], raw[4], raw[5], raw[6]])
    }

    /// `gm_sequence`: 24-bit plaintext sequence counter, b7..b9.
    fn sequence(raw: &[u8; WIRE_BYTES]) -> u32 {
        (u32::from(raw[7]) << 16) | (u32::from(raw[8]) << 8) | u32::from(raw[9])
    }

    /// `gm_button`: low 3 bits of b2.
    fn button(raw: &[u8; WIRE_BYTES]) -> u8 {
        raw[2] & 0x07
    }

    /// `gm_data_reset`.
    fn data_reset(&mut self) {
        self.bit_count = 0;
        self.bits = [0; WIRE_BYTES];
    }

    /// `gm_push_bit`: shift the 14-byte window left by one bit and append `bit` at the LSB end.
    fn push_bit(&mut self, bit: bool) {
        let mut carry = u8::from(bit);
        for i in (0..WIRE_BYTES).rev() {
            let new_carry = (self.bits[i] >> 7) & 1;
            self.bits[i] = (self.bits[i] << 1) | carry;
            carry = new_carry;
        }
        self.bit_count = self.bit_count.saturating_add(1);
    }

    /// `gm_report`: emit the window if it holds a valid, not-just-reported frame.
    fn report(&mut self) -> Option<DecodedSignal> {
        if (self.bit_count as usize) < WIRE_BITS || !Self::frame_valid(&self.bits) {
            return None;
        }
        if self.last_raw_valid && self.last_raw == self.bits {
            return None;
        }
        self.last_raw = self.bits;
        self.last_raw_valid = true;
        Some(Self::signal_from_frame(&self.bits))
    }

    /// End the burst: final window check, then back to Reset.
    fn finish(&mut self) -> Option<DecodedSignal> {
        let result = self.report();
        self.step = DecoderStep::Reset;
        self.data_reset();
        result
    }

    /// `gm_decode_fields`, with KAT's >64-bit packing (`data` = b0..b7, `extra` = b8..b13).
    fn signal_from_frame(raw: &[u8; WIRE_BYTES]) -> DecodedSignal {
        let data = u64::from_be_bytes([
            raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], raw[6], raw[7],
        ]);
        let extra = raw[8..]
            .iter()
            .fold(0u64, |acc, &b| (acc << 8) | u64::from(b));
        DecodedSignal {
            serial: Some(Self::id(raw)),
            button: Some(Self::button(raw)),
            counter: Some((Self::sequence(raw) & 0xFFFF) as u16),
            crc_valid: true,
            data,
            data_count_bit: WIRE_BITS,
            encoder_capable: true,
            extra: Some(extra),
            protocol_display_name: None,
        }
    }

    /// Rebuild the 14 wire bytes from a `DecodedSignal` (`data` = b0..b7, `extra` = b8..b13).
    fn frame_from_signal(decoded: &DecodedSignal) -> Option<[u8; WIRE_BYTES]> {
        if decoded.data_count_bit != WIRE_BITS {
            return None;
        }
        let extra = decoded.extra?;
        let mut raw = [0u8; WIRE_BYTES];
        raw[..8].copy_from_slice(&decoded.data.to_be_bytes());
        raw[8..].copy_from_slice(&extra.to_be_bytes()[2..]);
        Some(raw)
    }
}

impl ProtocolDecoder for GmDecoder {
    fn name(&self) -> &'static str {
        "GM"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: WIRE_BITS,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        // C flags: SubGhzProtocolFlag_315 | SubGhzProtocolFlag_433.
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.last_raw = [0; WIRE_BYTES];
        self.last_raw_valid = false;
        self.data_reset();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let pulse_ok = (PULSE_MIN_US..=PULSE_MAX_US).contains(&duration);
        match self.step {
            DecoderStep::Reset => {
                // Any plausible fixed-width HIGH pulse starts collection; the bit rides on the
                // following LOW gap.
                if level && pulse_ok {
                    self.step = DecoderStep::Data;
                    self.data_reset();
                }
                None
            }
            DecoderStep::Data => {
                if level {
                    // In-range pulses are ignored; a grossly out-of-range one ends the burst.
                    if pulse_ok {
                        None
                    } else {
                        self.finish()
                    }
                } else if duration >= RESET_US {
                    self.finish()
                } else if duration_diff!(duration, TE_SHORT) < TE_DELTA {
                    self.push_bit(false);
                    self.report()
                } else if duration_diff!(duration, TE_LONG) < TE_DELTA {
                    self.push_bit(true);
                    self.report()
                } else {
                    self.finish()
                }
            }
        }
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        // Replay only (as in the C): the requested button is ignored.
        let raw = Self::frame_from_signal(decoded)?;
        if !Self::frame_valid(&raw) {
            return None;
        }
        let mut signal = Vec::with_capacity(WIRE_BITS * 2 + 2);
        for bit_index in 0..WIRE_BITS {
            let bit = (raw[bit_index >> 3] >> (7 - (bit_index & 7))) & 1 != 0;
            signal.push(LevelDuration::new(true, TE_SHORT));
            signal.push(LevelDuration::new(
                false,
                if bit { TE_LONG } else { TE_SHORT },
            ));
        }
        // KAT deviation: closing pulse so the 112th gap is delimited (see module docs).
        signal.push(LevelDuration::new(true, TE_SHORT));
        signal.push(LevelDuration::new(false, RESET_US));
        Some(signal)
    }
}

impl Default for GmDecoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a valid 14-byte frame: b0 = 0xFF, b2 = button-checksum nibble | button, b13 = byte-sum
    /// complement so that Σ b1..b13 ≡ 0 (mod 256).
    fn make_frame(b1: u8, button: u8, id: u32, seq: u32, enc: u32) -> [u8; WIRE_BYTES] {
        assert!((1..=0x0F).contains(&button));
        let mut f = [0u8; WIRE_BYTES];
        f[0] = 0xFF;
        f[1] = b1;
        f[2] = ((0x10 - button) << 4) | button;
        f[3..7].copy_from_slice(&id.to_be_bytes());
        f[7..10].copy_from_slice(&seq.to_be_bytes()[1..]);
        f[10..13].copy_from_slice(&enc.to_be_bytes()[1..]);
        let sum: u32 = f[1..13].iter().map(|&b| u32::from(b)).sum();
        f[13] = (0x100 - (sum & 0xFF)) as u8;
        f
    }

    /// PPM waveform for `bits` (each bit = pulse + gap), then a closing pulse and the end gap.
    fn ppm(bits: &[bool], pulse: u32) -> Vec<LevelDuration> {
        let mut v = Vec::new();
        for &b in bits {
            v.push(LevelDuration::new(true, pulse));
            v.push(LevelDuration::new(
                false,
                if b { TE_LONG } else { TE_SHORT },
            ));
        }
        v.push(LevelDuration::new(true, pulse));
        v.push(LevelDuration::new(false, 25_000));
        v
    }

    fn frame_bits(f: &[u8; WIRE_BYTES]) -> Vec<bool> {
        (0..WIRE_BITS)
            .map(|i| (f[i >> 3] >> (7 - (i & 7))) & 1 != 0)
            .collect()
    }

    fn decode_all(signal: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = GmDecoder::new();
        signal
            .iter()
            .filter_map(|ld| dec.feed(ld.level, ld.duration_us))
            .collect()
    }

    #[test]
    fn gm_frame_checks() {
        let f = make_frame(0x5A, 2, 0x1234_5678, 0x00_ABCD, 0x00C0_FFEE);
        assert!(GmDecoder::frame_valid(&f));
        // Button nibble checksum is additive: hi + lo == 16.
        assert_eq!(GmDecoder::add_nibbles(&f[2..3]), 16);
        assert_eq!(GmDecoder::add_bytes(&f[1..]) & 0xFF, 0);
    }

    #[test]
    fn gm_decodes_synthetic_frame_after_truncated_wakeup() {
        let f = make_frame(0x5A, 2, 0x1234_5678, 0x01_ABCD, 0x00C0_FFEE);
        // Partial wake-up run of 1-bits (500 µs gaps) before the payload; the rolling window keeps
        // only the last 112 bits.
        let mut bits = vec![true; 37];
        bits.extend(frame_bits(&f));
        let decodes = decode_all(&ppm(&bits, 280));
        assert_eq!(decodes.len(), 1, "exactly one frame expected");
        let d = &decodes[0];
        assert_eq!(d.serial, Some(0x1234_5678));
        assert_eq!(d.button, Some(2));
        // 24-bit sequence 0x01ABCD → low 16 bits.
        assert_eq!(d.counter, Some(0xABCD));
        assert_eq!(d.data_count_bit, WIRE_BITS);
        assert!(d.crc_valid && d.encoder_capable);
        assert_eq!(GmDecoder::frame_from_signal(d), Some(f));
    }

    #[test]
    fn gm_decodes_with_rtl433_trailing_bit() {
        // rtl_433 takes the 112 bits ending one bit before the last: a trailing bit after the
        // payload must not prevent the decode (the window is tested after every bit).
        let f = make_frame(0x00, 1, 0xDEAD_BEEF, 0x00_0042, 0x0012_3456);
        let mut bits = vec![true; 12];
        bits.extend(frame_bits(&f));
        bits.push(false);
        let decodes = decode_all(&ppm(&bits, 300));
        assert_eq!(decodes.len(), 1);
        assert_eq!(GmDecoder::frame_from_signal(&decodes[0]), Some(f));
    }

    #[test]
    fn gm_encode_decode_roundtrip() {
        let f = make_frame(0x77, 4, 0x0BAD_F00D, 0x12_3456, 0x0065_4321);
        let original = GmDecoder::signal_from_frame(&f);
        let enc = GmDecoder::new();
        let signal = enc.encode(&original, 0x01).expect("encode");
        // 112 bit cells + closing pulse + guard; no adjacent same-level entries.
        assert_eq!(signal.len(), WIRE_BITS * 2 + 2);
        assert!(signal.windows(2).all(|w| w[0].level != w[1].level));
        let decodes = decode_all(&signal);
        assert_eq!(decodes.len(), 1);
        let d = &decodes[0];
        // Byte-exact replay: the requested button (Lock) is ignored.
        assert_eq!(d.data, original.data);
        assert_eq!(d.extra, original.extra);
        assert_eq!(d.serial, original.serial);
        assert_eq!(d.button, Some(4));
        assert_eq!(d.counter, Some(0x3456));
    }

    #[test]
    fn gm_without_closing_pulse_loses_last_bit() {
        // Documents the C upload issue fixed by the closing pulse: if the guard LOW directly
        // follows the 112th gap, the merged gap swallows that bit and nothing decodes.
        let f = make_frame(0x77, 4, 0x0BAD_F00D, 0x12_3456, 0x0065_4321);
        let mut signal = Vec::new();
        for b in frame_bits(&f) {
            signal.push(LevelDuration::new(true, TE_SHORT));
            signal.push(LevelDuration::new(
                false,
                if b { TE_LONG } else { TE_SHORT },
            ));
        }
        let last = signal.pop().unwrap();
        signal.push(LevelDuration::new(false, last.duration_us + RESET_US));
        assert!(decode_all(&signal).is_empty());
    }

    #[test]
    fn gm_rejects_corrupted_frames() {
        let good = make_frame(0x5A, 2, 0x1234_5678, 0x00_ABCD, 0x00C0_FFEE);
        assert_eq!(decode_all(&ppm(&frame_bits(&good), 300)).len(), 1);

        // Full checksum broken (single bit flip in the encrypted field).
        let mut bad = good;
        bad[11] ^= 0x01;
        assert!(decode_all(&ppm(&frame_bits(&bad), 300)).is_empty());

        // Wake-up byte not 0xFF (full checksum still balanced since b0 is excluded).
        let mut bad = good;
        bad[0] = 0xFE;
        assert!(decode_all(&ppm(&frame_bits(&bad), 300)).is_empty());

        // Button nibble checksum broken, full checksum re-balanced.
        let mut bad = good;
        bad[2] = 0x12;
        let sum: u32 = bad[1..13].iter().map(|&b| u32::from(b)).sum();
        bad[13] = (0x100 - (sum & 0xFF)) as u8;
        assert!(!GmDecoder::frame_valid(&bad));
        assert!(decode_all(&ppm(&frame_bits(&bad), 300)).is_empty());

        // The encoder refuses frames that fail the checks.
        let mut sig = GmDecoder::signal_from_frame(&good);
        sig.data ^= 1;
        assert!(GmDecoder::new().encode(&sig, 0).is_none());
    }

    #[test]
    fn gm_registry_claims_replay() {
        // The replay waveform must come back as GM through the full registry on both bands
        // (no earlier decoder claims gap-width PPM).
        let f = make_frame(0x3C, 1, 0x0102_0304, 0x00_1000, 0x00AB_CDEF);
        let signal = GmDecoder::new()
            .encode(&GmDecoder::signal_from_frame(&f), 0)
            .unwrap();
        for freq in [315_000_000, 433_920_000] {
            let mut reg = crate::protocols::ProtocolRegistry::new();
            let names: Vec<String> = reg
                .process_signal_stream(&signal, freq)
                .into_iter()
                .map(|(n, _, _)| n)
                .collect();
            assert_eq!(names, vec!["GM".to_string()], "freq {freq}");
        }
    }

    #[test]
    fn gm_out_of_range_pulse_ends_burst() {
        // A 1.5 ms HIGH pulse in the middle of the payload aborts the burst: nothing decodes.
        let f = make_frame(0x5A, 3, 0x1234_5678, 0x00_ABCD, 0x00C0_FFEE);
        let mut signal = ppm(&frame_bits(&f), 300);
        signal[100] = LevelDuration::new(true, 1500);
        assert!(decode_all(&signal).is_empty());
    }
}
