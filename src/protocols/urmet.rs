//! Urmet 433.92 MHz remote: a Manchester burst, a ~34 ms gap, then an inverted-USART frame sent
//! as pulse-position (fixed 1·te HIGH, variable LOW).
//!
//! Reference: doctor-firmware `lib/subghz/protocols/urmet.c` (the only reference with it;
//! Flipper-ARF has no urmet.c).
//!
//! **Line coding**: te = 500 µs (te_delta 150). On air: a Manchester block (short edges of
//! 40–280 µs), a LOW of 20–60 ms, then PPM: every HIGH is 1·te, the LOW after it carries
//! 1..15 te of zero bits. The bit stream (HIGH te = 1, LOW te = 0) is 8N1 USART, idle low,
//! start bit high, LSB first, data bits inverted on the wire. It starts with the preamble
//! H1 L1 H1 L11.
//!
//! **Decoder** (`subghz_protocol_decoder_urmet_feed`, ported state for state): Reset → (a
//! Manchester-sized edge) Manchester → (≥ 16 such edges, then a 20–60 ms LOW) Armed → the PPM
//! preamble HIGH 1te, LOW 1te, HIGH 1te, LOW 11te (± 3te) → Data. In Data a HIGH must be 1te
//! (and not follow another HIGH), the LOW after it is rounded to te (1..15, else abort) and
//! appended as bits; up to 192 gaps / 512 bits. A LOW ≥ 20 ms ends the frame: the bits are
//! USART-decoded (`urmet_decode_inv_usart`) and the frame is reported when it has 16..40 bytes
//! whose nibbles are all 5, 7, D or F and at least 2 gaps were stored.
//!
//! **Fields**: `data` = the first 8 bytes big-endian (the C's `urmet_bytes_to_key`), Bit =
//! 8·bytes; no serial / button / counter (the reference shows the raw bytes only).
//!
//! **Encoder**: the C transmits the stored gap list (`Gaps`: 1·te HIGH + gap·te LOW each, a
//! final 1·te HIGH and a 68·te LOW, 5 times) and serializes `Raw` (all bytes) + `Gaps`. A frame
//! has up to 40 bytes and 192 gaps, which `DecodedSignal` (a 64-bit `data` + a 64-bit `extra`)
//! cannot carry, so KAT decodes Urmet but does not re-encode it (`encoder_capable` = false; the
//! capture's raw pairs can still be replayed) and does not write key files. `from_flipper_key`
//! reads the reference's key files (`Raw` when present, else the `Key`, as its deserialize does).

use super::g3_common::duration_diff;
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Urmet",
    decoder: "Urmet",
    aliases: &[],
    name_prefix: None,
    // Pulse-position USART after a Manchester wake-up burst (KAT has no PPM coding label).
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Fixed Code",
    make: "Urmet",
    category: Category::Gate,
    buttons: None,
    presets: &[&AM650],
};

const URMET_TE: u32 = 500;
const URMET_TE_DELTA: u32 = 150;
const URMET_SYNC_TE: u32 = 11;
const URMET_SYNC_DELTA_TE: u32 = 3;
const URMET_END_GAP_US: u32 = 20_000;
const URMET_MAN_EDGE_MIN_US: u32 = 40;
const URMET_MAN_EDGE_MAX_US: u32 = 280;
const URMET_MAN_MIN_EDGES: u16 = 16;
const URMET_MAN_GAP_MAX_US: u32 = 60_000;
const URMET_MIN_BYTES: usize = 16;
const URMET_MAX_BYTES: usize = 40;
const URMET_MAX_GAPS: usize = 192;
const URMET_BITBUF: usize = 512;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    Manchester,
    Armed,
    PreambleLow1,
    PreambleHigh2,
    PreambleSync,
    Data,
}

/// Urmet decoder (`SubGhzProtocolDecoderUrmet`; the last accepted frame's copy is not kept).
pub struct UrmetDecoder {
    step: Step,
    pending_high: bool,
    man_edges: u16,
    bit_cnt: usize,
    bit_buf: Vec<u8>,
    rx_gaps: Vec<u8>,
}

fn is_manchester_edge(duration: u32) -> bool {
    (URMET_MAN_EDGE_MIN_US..=URMET_MAN_EDGE_MAX_US).contains(&duration)
}

fn is_te1(duration: u32) -> bool {
    duration_diff(duration, URMET_TE) < URMET_TE_DELTA
}

fn is_sync_low(duration: u32) -> bool {
    duration_diff(duration, URMET_TE * URMET_SYNC_TE) < URMET_TE * URMET_SYNC_DELTA_TE
}

/// `urmet_te_count`: duration rounded to te, clamped to 1..20.
fn te_count(duration: u32) -> u32 {
    ((duration + URMET_TE / 2) / URMET_TE).clamp(1, 20)
}

/// `urmet_bytes_to_key`: the first (up to) 8 bytes, big-endian, left-aligned.
fn bytes_to_key(bytes: &[u8]) -> u64 {
    let n = bytes.len().min(8);
    let mut key = bytes[..n]
        .iter()
        .fold(0u64, |k, &b| (k << 8) | u64::from(b));
    if n < 8 {
        key <<= (8 - n) * 8;
    }
    key
}

fn nibble_ok(b: u8) -> bool {
    let ok = |n: u8| matches!(n, 0x5 | 0x7 | 0xD | 0xF);
    ok(b >> 4) && ok(b & 0x0F)
}

/// `urmet_payload_plausible`.
fn payload_plausible(bytes: &[u8]) -> bool {
    (URMET_MIN_BYTES..=URMET_MAX_BYTES).contains(&bytes.len())
        && bytes.iter().all(|&b| nibble_ok(b))
}

/// `urmet_decode_inv_usart`: inverted 8N1 (idle low, start high, LSB first, data inverted).
fn decode_inv_usart(bits: &[u8], out_max: usize) -> Vec<u8> {
    let bit_cnt = bits.len();
    let mut out = Vec::new();
    let mut i = 0usize;
    while i < bit_cnt && out.len() < out_max {
        if bits[i] != 1 {
            i += 1;
            continue;
        }
        if i > 0 && bits[i - 1] != 0 {
            i += 1;
            continue;
        }
        if i + 9 >= bit_cnt {
            break;
        }
        let mut val = 0u8;
        for b in 0..8 {
            if bits[i + 1 + b] == 0 {
                val |= 1 << b;
            }
        }
        if bits[i + 9] != 0 {
            i += 1;
            continue;
        }
        out.push(val);
        i += 10;
        while i < bit_cnt && bits[i] == 0 {
            i += 1;
        }
    }
    out
}

fn signal(bytes: &[u8]) -> DecodedSignal {
    DecodedSignal {
        serial: None,
        button: None,
        counter: None,
        crc_valid: false,
        data: bytes_to_key(bytes),
        data_count_bit: bytes.len() * 8,
        encoder_capable: false,
        extra: None,
        protocol_display_name: None,
    }
}

impl UrmetDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            pending_high: false,
            man_edges: 0,
            bit_cnt: 0,
            bit_buf: vec![0; URMET_BITBUF],
            rx_gaps: Vec::with_capacity(URMET_MAX_GAPS),
        }
    }

    /// `urmet_decoder_abort`.
    fn abort(&mut self) {
        self.step = Step::Reset;
        self.pending_high = false;
        self.man_edges = 0;
        self.bit_cnt = 0;
        self.rx_gaps.clear();
    }

    /// `urmet_append_bits`.
    fn append_bits(&mut self, level: bool, n: u32) {
        for _ in 0..n {
            if self.bit_cnt >= URMET_BITBUF {
                break;
            }
            self.bit_buf[self.bit_cnt] = u8::from(level);
            self.bit_cnt += 1;
        }
    }

    /// `urmet_append_gap`.
    fn append_gap(&mut self, gap: u8) -> bool {
        if self.rx_gaps.len() >= URMET_MAX_GAPS {
            return false;
        }
        self.rx_gaps.push(gap);
        true
    }

    /// `subghz_protocol_decoder_urmet_finish`.
    fn finish(&mut self) -> Option<DecodedSignal> {
        let decoded = decode_inv_usart(&self.bit_buf[..self.bit_cnt], URMET_MAX_BYTES);
        let found =
            (payload_plausible(&decoded) && self.rx_gaps.len() >= 2).then(|| signal(&decoded));
        self.abort();
        found
    }
}

impl Default for UrmetDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for UrmetDecoder {
    fn name(&self) -> &'static str {
        "Urmet"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: URMET_TE,
            te_long: URMET_TE * URMET_SYNC_TE,
            te_delta: URMET_TE_DELTA,
            min_count_bit: URMET_MIN_BYTES * 8,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    /// `subghz_protocol_decoder_urmet_reset` (the gap list is re-seeded at the next lock).
    fn reset(&mut self) {
        self.step = Step::Reset;
        self.pending_high = false;
        self.man_edges = 0;
        self.bit_cnt = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        // Long LOWs: end of the PPM data, or the Manchester → PPM gap.
        if !level && duration >= URMET_END_GAP_US {
            if self.step == Step::Data {
                if self.bit_cnt < URMET_BITBUF {
                    self.bit_buf[self.bit_cnt] = 0;
                    self.bit_cnt += 1;
                }
                return self.finish();
            } else if self.step == Step::Manchester
                && self.man_edges >= URMET_MAN_MIN_EDGES
                && duration <= URMET_MAN_GAP_MAX_US
            {
                self.step = Step::Armed;
                self.man_edges = 0;
                self.pending_high = false;
            } else {
                self.abort();
            }
            return None;
        }

        match self.step {
            Step::Reset => {
                if is_manchester_edge(duration) {
                    self.step = Step::Manchester;
                    self.man_edges = 1;
                }
            }
            Step::Manchester => {
                if is_manchester_edge(duration) {
                    if self.man_edges < 0xFFF0 {
                        self.man_edges += 1;
                    }
                } else if is_te1(duration)
                    || (duration > URMET_MAN_EDGE_MAX_US * 2 && duration < URMET_END_GAP_US)
                {
                    // A PPM-sized edge too early, or a big outlier (two branches in the C).
                    self.abort();
                }
            }
            Step::Armed => {
                if level && is_te1(duration) {
                    self.step = Step::PreambleLow1;
                } else if is_manchester_edge(duration) {
                    self.step = Step::Manchester;
                    self.man_edges = 1;
                }
            }
            Step::PreambleLow1 => {
                if !level && is_te1(duration) {
                    self.step = Step::PreambleHigh2;
                } else if level && is_te1(duration) {
                    self.step = Step::PreambleLow1;
                } else {
                    self.abort();
                }
            }
            Step::PreambleHigh2 => {
                if level && is_te1(duration) {
                    self.step = Step::PreambleSync;
                } else {
                    self.abort();
                }
            }
            Step::PreambleSync => {
                if !level && is_sync_low(duration) {
                    // Locked: seed the bit and gap buffers with the preamble H1 L1 H1 L11.
                    self.bit_cnt = 0;
                    self.rx_gaps.clear();
                    self.pending_high = false;
                    self.append_bits(true, 1);
                    self.append_bits(false, 1);
                    self.append_bits(true, 1);
                    self.append_bits(false, URMET_SYNC_TE);
                    self.append_gap(1);
                    self.append_gap(URMET_SYNC_TE as u8);
                    self.step = Step::Data;
                } else if level && is_te1(duration) {
                    self.step = Step::PreambleLow1;
                } else {
                    self.abort();
                }
            }
            Step::Data => {
                if level {
                    if !is_te1(duration) || self.pending_high {
                        self.abort();
                    } else {
                        self.pending_high = true;
                    }
                    return None;
                }
                if !self.pending_high {
                    if duration > URMET_TE * 2 {
                        self.abort();
                    }
                    return None;
                }
                self.pending_high = false;
                let gap = te_count(duration);
                if !(1..=15).contains(&gap) {
                    self.abort();
                    return None;
                }
                if !self.append_gap(gap as u8) {
                    return self.finish();
                }
                self.append_bits(true, 1);
                self.append_bits(false, gap);
                if self.bit_cnt >= URMET_BITBUF || self.rx_gaps.len() >= URMET_MAX_GAPS {
                    return self.finish();
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

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        // urmet_load_from_ff: Raw (all bytes) when present, else Bit/8 bytes of Key (the
        // reference zero-fills past the 8 Key bytes); then Gaps (or gaps rebuilt from the
        // bytes) must give ≥ 2 gaps — true for any 16..40-byte frame.
        let bytes = match key.field("Raw") {
            Some(raw) => super::flipper_key::parse_hex_vec(raw)?,
            None => {
                let bits = key.bits;
                if !(URMET_MIN_BYTES * 8..=URMET_MAX_BYTES * 8).contains(&bits)
                    || !bits.is_multiple_of(8)
                {
                    return None;
                }
                let mut v = vec![0u8; bits / 8];
                v[..8].copy_from_slice(&key.key.to_be_bytes());
                v
            }
        };
        if !(URMET_MIN_BYTES..=URMET_MAX_BYTES).contains(&bytes.len()) {
            return None;
        }
        if let Some(gaps) = key.field("Gaps") {
            let gaps = super::flipper_key::parse_hex_vec(gaps)?;
            if gaps.len() < 2 || gaps.len() > URMET_MAX_GAPS {
                return None;
            }
        }
        Some(signal(&bytes))
    }
}

#[cfg(test)]
mod tests {
    use super::super::g3_common::test_support::*;
    use super::super::g3_common::{repeat_upload, UploadBuilder};
    use super::*;

    /// A frame the way the remote sends it: Manchester wake-up burst, 34 ms gap, PPM preamble
    /// H1 L1 H1 L11, the USART bytes (inverted 8N1) as 1·te HIGH + n·te LOW, 34 ms trailer.
    fn on_air(bytes: &[u8], idle_between_bytes: u32) -> Vec<LevelDuration> {
        let mut b = UploadBuilder::new();
        for _ in 0..12 {
            b.push(true, 150);
            b.push(false, 150);
        }
        b.push(false, 34_000);
        // Preamble, then the bit stream.
        let mut bits: Vec<u8> = vec![1, 0, 1];
        bits.extend(std::iter::repeat_n(0, 11));
        for &v in bytes {
            bits.push(1); // start
            for k in 0..8 {
                bits.push(if (v >> k) & 1 == 1 { 0 } else { 1 });
            }
            bits.push(0); // stop
            bits.extend(std::iter::repeat_n(0, idle_between_bytes as usize));
        }
        bits.push(1); // closing HIGH
        for &x in &bits {
            b.push(x == 1, URMET_TE);
        }
        b.push(false, 68 * URMET_TE);
        b.finish()
    }

    const FRAME: [u8; 18] = [
        0x55, 0x57, 0x5D, 0x5F, 0x75, 0x77, 0x7D, 0x7F, 0xD5, 0xD7, 0xDD, 0xDF, 0xF5, 0xF7, 0xFD,
        0xFF, 0x55, 0x7D,
    ];

    #[test]
    fn usart_round_trip() {
        let mut bits = vec![0u8, 0];
        for &v in &FRAME {
            bits.push(1);
            for k in 0..8 {
                bits.push(if (v >> k) & 1 == 1 { 0 } else { 1 });
            }
            bits.push(0);
        }
        bits.push(0);
        assert_eq!(decode_inv_usart(&bits, 40), FRAME.to_vec());
        assert!(payload_plausible(&FRAME));
        assert!(!payload_plausible(&FRAME[..15]));
        let mut bad = FRAME;
        bad[3] = 0x56;
        assert!(!payload_plausible(&bad));
    }

    #[test]
    fn decodes_a_synthetic_frame() {
        for idle in [0u32, 2] {
            let p = on_air(&FRAME, idle);
            let frames = feed_all(&mut UrmetDecoder::new(), &p);
            assert_eq!(frames.len(), 1, "idle {idle}");
            let f = &frames[0];
            // The preamble H1 L1 H1 L11 reads as a first USART byte 0xFD.
            assert_eq!(f.data, 0xFD55_575D_5F75_777D);
            assert_eq!(f.data_count_bit, 19 * 8);
            assert!(!f.encoder_capable && f.serial.is_none());
            // Repeats (with their own wake-up bursts) decode again.
            let two = repeat_upload(&p, 2);
            assert_eq!(feed_all(&mut UrmetDecoder::new(), &two).len(), 2);
        }
    }

    #[test]
    fn rejects_without_wake_up_or_with_bad_alphabet() {
        // No Manchester burst before the gap: never armed.
        let p = on_air(&FRAME, 0);
        assert!(feed_all(&mut UrmetDecoder::new(), &p[24..]).is_empty());
        // A byte outside the 5/7/D/F alphabet.
        let mut bad = FRAME;
        bad[5] = 0x12;
        assert!(feed_all(&mut UrmetDecoder::new(), &on_air(&bad, 0)).is_empty());
        // Too short: 14 bytes + the preamble's 0xFD = 15 bytes.
        assert!(feed_all(&mut UrmetDecoder::new(), &on_air(&FRAME[..14], 0)).is_empty());
        assert_eq!(
            feed_all(&mut UrmetDecoder::new(), &on_air(&FRAME[..15], 0)).len(),
            1
        );
        // A stretched HIGH aborts.
        let mut p = on_air(&FRAME, 0);
        let i = (40..).find(|&k| p[k].level).unwrap();
        p[i].duration_us = 900;
        assert!(feed_all(&mut UrmetDecoder::new(), &p).is_empty());
    }

    #[test]
    fn key_files_load_but_do_not_encode() {
        let d = UrmetDecoder::new();
        let raw: Vec<String> = FRAME.iter().map(|b| format!("{b:02X}")).collect();
        let text = format!(
            "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
             Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Urmet\nBit: 144\n\
             Key: 55 57 5D 5F 75 77 7D 7F\nRaw: {}\nGaps: 01 0B 01 02\n",
            raw.join(" ")
        );
        let s = d
            .from_flipper_key(&FlipperKey::parse(&text).unwrap())
            .unwrap();
        assert_eq!(
            (s.data, s.data_count_bit, s.encoder_capable),
            (0x5557_5D5F_7577_7D7F, 144, false)
        );
        assert!(d.encode(&s, 0).is_none());
        assert!(d.to_flipper_key(&s).is_none());
        // Key-only file: Bit/8 bytes.
        let key_only = text
            .lines()
            .filter(|l| !l.starts_with("Raw") && !l.starts_with("Gaps"))
            .collect::<Vec<_>>()
            .join("\n");
        let s = d
            .from_flipper_key(&FlipperKey::parse(&key_only).unwrap())
            .unwrap();
        assert_eq!((s.data, s.data_count_bit), (0x5557_5D5F_7577_7D7F, 144));
        let bad = key_only.replace("Bit: 144", "Bit: 100");
        assert!(d
            .from_flipper_key(&FlipperKey::parse(&bad).unwrap())
            .is_none());
        // A Gaps line with fewer than 2 gaps does not load (urmet_load_from_ff).
        let one_gap = text.replace("Gaps: 01 0B 01 02", "Gaps: 01");
        assert!(d
            .from_flipper_key(&FlipperKey::parse(&one_gap).unwrap())
            .is_none());
    }
}
