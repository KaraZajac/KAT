//! Subaru V1 protocol decoder/encoder: the US 2012-2019 remote-head key (FCC ID CWTWB1U811,
//! 315 MHz). Not in ProtoPirate, Flipper-ARF or doctor-firmware; reverse-engineered from the
//! owner's own key recordings (see `docs/subaru_v1.md`). The EU/AU PWM format is [super::subaru].
//!
//! # Air format
//!
//! OOK, Manchester with a 122µs half-bit (244µs bit). A press sends three copies. Copy 1 opens
//! with a preamble of ~520 alternating 244µs pulses (HIGH first), copies 2 and 3 with 63. The
//! frame follows the preamble's last HIGH directly and is 57 bits: a start bit `0`, then 7 bytes,
//! MSB first, `10` = 1 and `01` = 0 in half-bit order. A LOW of ~1.34ms ends each copy.
//!
//! # Frame (7 bytes b0..b6)
//!
//! * `b0..b2`: 24-bit key ID (`serial`).
//! * `b3` high nibble: button; `b3` low nibble + `b4` + `b5`: 20-bit counter, +1 per press.
//! * `b6` = (`b0 ^ b1 ^ b2 ^ b3 ^ b4 ^ b5`) + 1 mod 256 (`crc_valid`).
//!
//! Buttons (native nibble): 1 Lock, 2 Unlock, 0xB Trunk, 0xA Panic, all recorded. Panic matches
//! Wimmenhove's code for the older 2005-2011 Subaru format.
//!
//! `data` holds the 56 bits b0..b6, `data_count_bit` is 56; `counter` is the low 16 bits of the
//! 20-bit counter (the high nibble is in `data`).

use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (line coding, RF, encryption, make, native buttons, TX presets); see `info.rs`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Subaru V1",
    decoder: "Subaru V1",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Rolling Code (plaintext counter)",
    make: "Subaru",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// Native button nibble (all four confirmed by labelled recordings).
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x1 => "Lock",
        0x2 => "Unlock",
        0xB => "Trunk",
        0xA => "Panic",
        _ => "Other",
    })
}

/// Half-bit, µs.
const TE_HALF: u32 = 122;
/// Preamble pulse (two half-bits) and its accepted range; the recordings span 195..297.
const PRE_NOM: u32 = 244;
const PRE_MIN: u32 = 185;
const PRE_MAX: u32 = 305;
/// A data pulse is one half-bit up to this length, two up to `PRE_MAX`; longer is the gap.
const HALF_MIN: u32 = 45;
const HALF_MAX: u32 = 185;
/// Preamble pulses needed before a frame is read.
const PREAMBLE_MIN: u16 = 20;
/// Silence after the last bit of a copy.
const GAP_US: u32 = 1340;
/// Preamble pulses sent: copy 1 (the recordings show 516..529 once the receiver has settled; odd
/// so it ends on a HIGH like the real one), copies 2 and 3.
const PRE_FIRST: usize = 527;
const PRE_REPEAT: usize = 63;
const COPIES: usize = 3;

const FRAME_BITS: u32 = 57;
const MIN_COUNT_BIT: usize = 56;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    Preamble,
    Data,
}

/// Decoded key fields.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Fields {
    serial: u32,
    btn: u8,
    cnt: u32,
}

/// Subaru V1 protocol decoder
pub struct SubaruV1Decoder {
    step: Step,
    preamble: u16,
    /// Half-bit levels read so far (first one in bit 0), and how many.
    halves: u128,
    count: u32,
}

/// Check byte over b0..b5.
fn check_byte(b: &[u8; 7]) -> u8 {
    b[..6].iter().fold(0u8, |a, &x| a ^ x).wrapping_add(1)
}

fn bytes_of(data: u64) -> [u8; 7] {
    let be = data.to_be_bytes();
    [be[1], be[2], be[3], be[4], be[5], be[6], be[7]]
}

fn data_of(b: &[u8; 7]) -> u64 {
    b.iter().fold(0u64, |a, &x| (a << 8) | x as u64)
}

fn fields(data: u64) -> Fields {
    let b = bytes_of(data);
    Fields {
        serial: (b[0] as u32) << 16 | (b[1] as u32) << 8 | b[2] as u32,
        btn: b[3] >> 4,
        cnt: ((b[3] & 0x0F) as u32) << 16 | (b[4] as u32) << 8 | b[5] as u32,
    }
}

/// A frame for (serial, button, 20-bit counter) with its check byte.
fn build_data(serial: u32, btn: u8, cnt: u32) -> u64 {
    let mut b = [0u8; 7];
    b[0] = (serial >> 16) as u8;
    b[1] = (serial >> 8) as u8;
    b[2] = serial as u8;
    b[3] = (btn << 4) | ((cnt >> 16) & 0x0F) as u8;
    b[4] = (cnt >> 8) as u8;
    b[5] = cnt as u8;
    b[6] = check_byte(&b);
    data_of(&b)
}

impl SubaruV1Decoder {
    pub fn new() -> Self {
        Self { step: Step::Reset, preamble: 0, halves: 0, count: 0 }
    }

    fn reset_state(&mut self) {
        self.step = Step::Reset;
        self.preamble = 0;
        self.halves = 0;
        self.count = 0;
    }

    /// A decoded frame from the 56 data bits.
    fn frame(data: u64) -> DecodedSignal {
        let f = fields(data);
        let b = bytes_of(data);
        DecodedSignal {
            serial: Some(f.serial),
            button: Some(f.btn),
            counter: Some(f.cnt as u16),
            crc_valid: check_byte(&b) == b[6],
            data,
            data_count_bit: MIN_COUNT_BIT,
            encoder_capable: true,
            extra: None,
            protocol_display_name: None,
        }
    }

    /// Append one half-bit level; false when it completes an invalid pair (the first pair must
    /// be `01`, the start bit).
    fn push_half(&mut self, level: bool) -> bool {
        if self.count >= 2 * FRAME_BITS {
            return false;
        }
        if level {
            self.halves |= 1u128 << self.count;
        }
        self.count += 1;
        if self.count % 2 == 1 {
            return true;
        }
        let first = (self.halves >> (self.count - 2)) & 1 == 1;
        first != level && (self.count != 2 || level)
    }

    /// The frame, once all 114 half-bits are in and the start bit and check byte hold.
    fn finish(&mut self) -> Option<DecodedSignal> {
        let ok = self.count == 2 * FRAME_BITS;
        let mut data = 0u64;
        if ok {
            // Bit k of the frame (k = 1..56) is the first half of pair k: '10' = 1.
            for k in 1..FRAME_BITS {
                data = (data << 1) | ((self.halves >> (2 * k)) & 1) as u64;
            }
        }
        self.reset_state();
        if !ok {
            return None;
        }
        let b = bytes_of(data);
        (check_byte(&b) == b[6]).then(|| Self::frame(data))
    }

    /// `up` pulses of one level.
    fn upload(data: u64) -> Vec<LevelDuration> {
        // Half-bit levels of the copy's frame: start bit 0, then 56 bits.
        let mut halves: Vec<bool> = vec![false, true];
        for i in (0..MIN_COUNT_BIT).rev() {
            let one = (data >> i) & 1 == 1;
            halves.push(one);
            halves.push(!one);
        }
        let mut up = Vec::with_capacity(COPIES * (PRE_REPEAT + 120) + PRE_FIRST);
        for copy in 0..COPIES {
            let n = if copy == 0 { PRE_FIRST } else { PRE_REPEAT };
            for i in 0..n {
                up.push(LevelDuration::new(i % 2 == 0, PRE_NOM));
            }
            let mut i = 0;
            while i < halves.len() {
                let mut run = 1;
                while i + run < halves.len() && halves[i + run] == halves[i] {
                    run += 1;
                }
                let mut d = run as u32 * TE_HALF;
                if i + run == halves.len() && !halves[i] {
                    d += GAP_US; // the last LOW half runs into the gap
                }
                up.push(LevelDuration::new(halves[i], d));
                i += run;
            }
            if halves.last() == Some(&true) {
                up.push(LevelDuration::new(false, GAP_US));
            }
        }
        up
    }

    /// KAT generic button → native nibble; others keep the captured one.
    fn map_button(button: u8, captured: u8) -> u8 {
        match button {
            0x01 => 0x1,
            0x02 => 0x2,
            0x04 => 0xB,
            0x08 => 0xA,
            _ => captured,
        }
    }

    /// The frame `encode` sends: counter + 1 (20 bits), the requested button, a fresh check byte.
    fn next_data(decoded: &DecodedSignal, button: u8) -> Option<u64> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        let f = fields(decoded.data);
        let btn = Self::map_button(button, f.btn) & 0x0F;
        Some(build_data(f.serial, btn, (f.cnt + 1) & 0xF_FFFF))
    }
}

impl ProtocolDecoder for SubaruV1Decoder {
    fn name(&self) -> &'static str {
        "Subaru V1"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_HALF,
            te_long: PRE_NOM,
            te_delta: 60,
            min_count_bit: MIN_COUNT_BIT,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[315_000_000]
    }

    fn reset(&mut self) {
        self.reset_state();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let is_pre = (PRE_MIN..PRE_MAX).contains(&duration);
        match self.step {
            Step::Reset => {
                if is_pre {
                    self.preamble = 1;
                    self.step = Step::Preamble;
                }
            }
            Step::Preamble => {
                if is_pre {
                    self.preamble = self.preamble.saturating_add(1);
                } else if self.preamble >= PREAMBLE_MIN
                    && !level
                    && (HALF_MIN..HALF_MAX).contains(&duration)
                {
                    // The start bit's LOW half: the frame begins.
                    self.halves = 0;
                    self.count = 0;
                    self.step = Step::Data;
                    self.push_half(false);
                } else {
                    self.reset_state();
                }
            }
            Step::Data => {
                if duration >= PRE_MAX {
                    // The gap: its first half-bit is the last bit's second half when that is LOW.
                    if self.push_half(level) {
                        return self.finish();
                    }
                    self.reset_state();
                } else if (HALF_MIN..HALF_MAX).contains(&duration) {
                    if !self.push_half(level) {
                        self.reset_state();
                    }
                } else if is_pre {
                    if !(self.push_half(level) && self.push_half(level)) {
                        self.reset_state();
                    }
                } else {
                    self.reset_state();
                }
                if self.step == Step::Data && self.count == 2 * FRAME_BITS {
                    return self.finish();
                }
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        Some(Self::upload(Self::next_data(decoded, button)?))
    }

    /// The frame `encode` sends, so the app keeps the counter advanced on the capture.
    fn sent_signal(&self, decoded: &DecodedSignal, button: u8) -> Option<DecodedSignal> {
        Some(Self::frame(Self::next_data(decoded, button)?))
    }

    /// Key file: `Protocol: Subaru V1`, `Bit: 56`, the 7 bytes as `Key`, then decimal `Serial`,
    /// `Btn` (native nibble) and `Cnt` (20 bits). KAT's own format: no Flipper firmware has one.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        let f = fields(decoded.data);
        let mut key = FlipperKey::for_protocol("Subaru V1", MIN_COUNT_BIT, decoded.data);
        key.push_u32("Serial", f.serial);
        key.push_u32("Btn", f.btn.into());
        key.push_u32("Cnt", f.cnt);
        Some(key)
    }

    /// Everything comes from the Key; a key whose check byte fails does not load.
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != MIN_COUNT_BIT {
            return None;
        }
        let data = key.key_u64()?;
        if data >> 56 != 0 {
            return None;
        }
        let d = Self::frame(data);
        d.crc_valid.then_some(d)
    }
}

impl Default for SubaruV1Decoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ID: u32 = 0x5A_3C91;

    fn decode_all(pairs: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut d = SubaruV1Decoder::new();
        pairs.iter().filter_map(|p| d.feed(p.level, p.duration_us)).collect()
    }

    fn capture(btn: u8, cnt: u32) -> DecodedSignal {
        SubaruV1Decoder::frame(build_data(ID, btn, cnt))
    }

    #[test]
    fn check_byte_and_fields() {
        let d = capture(0xB, 0x1_2345);
        let b = bytes_of(d.data);
        assert_eq!(b[..3], [0x5A, 0x3C, 0x91]);
        assert_eq!(b[3], 0xB1);
        assert_eq!(b[4..6], [0x23, 0x45]);
        assert_eq!(b[6], (0x5A ^ 0x3C ^ 0x91 ^ 0xB1 ^ 0x23 ^ 0x45u8).wrapping_add(1));
        assert!(d.crc_valid);
        assert_eq!((d.serial, d.button, d.counter, d.data_count_bit), (Some(ID), Some(0xB), Some(0x2345), 56));
    }

    /// Three identical copies with the real fob's preambles and gaps; they decode back.
    #[test]
    fn upload_layout_and_round_trip() {
        let cap = capture(0x1, 0xF_FFFE);
        for (cmd, want) in [(0x01u8, 0x1u8), (0x02, 0x2), (0x04, 0xB), (0x08, 0xA), (0x40, 0x1)] {
            let up = SubaruV1Decoder::new().encode(&cap, cmd).unwrap();
            // Copy 1: 527 preamble pulses starting HIGH, ending HIGH, then the start bit's LOW half.
            assert!(up[0].level && up[PRE_FIRST - 1].level);
            assert!(up[..PRE_FIRST].iter().all(|p| p.duration_us == 244));
            assert_eq!((up[PRE_FIRST].level, up[PRE_FIRST].duration_us), (false, 122));
            let got = decode_all(&up);
            assert_eq!(got.len(), 3, "three copies");
            for g in &got {
                assert_eq!(g, &got[0]);
                assert_eq!((g.serial, g.button, g.crc_valid), (Some(ID), Some(want), true));
                // Counter + 1 across the 20-bit carry: 0xFFFFE -> 0xFFFFF.
                assert_eq!(fields(g.data).cnt, 0xF_FFFF);
            }
            assert_eq!(got[0], SubaruV1Decoder::new().sent_signal(&cap, cmd).unwrap());
            // Copies 2 and 3 open with 63 preamble pulses after the gap.
            let gaps: Vec<usize> = (0..up.len()).filter(|&i| !up[i].level && up[i].duration_us >= 1340).collect();
            assert_eq!(gaps.len(), 3);
            for g in &gaps[..2] {
                assert!(up[g + 1..g + 64].iter().all(|p| p.duration_us == 244));
                assert_ne!(up[g + 64].duration_us, 244, "the frame follows the 63rd pulse");
            }
        }
    }

    #[test]
    fn counter_wraps_at_20_bits_and_keeps_serial() {
        let cap = capture(0x2, 0xF_FFFF);
        let sent = SubaruV1Decoder::new().sent_signal(&cap, 0x02).unwrap();
        assert_eq!(fields(sent.data).cnt, 0);
        assert_eq!(sent.serial, Some(ID));
        // Counter + 1 on every press.
        let mut c = cap.clone();
        for n in 1..=5u32 {
            c = SubaruV1Decoder::new().sent_signal(&c, 0x02).unwrap();
            assert_eq!(fields(c.data).cnt, (n - 1) & 0xF_FFFF);
        }
    }

    #[test]
    fn rejects_bad_check_start_bit_and_short_preamble() {
        let up = SubaruV1Decoder::upload(build_data(ID, 1, 7));
        // Corrupt the check byte: one data bit flipped means no frame.
        let mut bad = SubaruV1Decoder::upload(build_data(ID, 1, 7) ^ 1);
        assert!(decode_all(&bad).is_empty());
        assert_eq!(decode_all(&up).len(), 3);
        // A short preamble (19 pulses) is not a frame.
        bad = up[PRE_FIRST - 19..PRE_FIRST + 120].to_vec();
        assert!(decode_all(&bad).is_empty());
        // Start bit 1 instead of 0.
        let mut halves = up.clone();
        halves[PRE_FIRST] = LevelDuration::new(true, 122);
        assert!(decode_all(&halves[..PRE_FIRST + 120]).is_empty());
    }

    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::assert_round_trip;
        let dec = SubaruV1Decoder::new();
        let d = capture(0xB, 0x0_0457);
        let text = assert_round_trip(&dec, &d);
        assert!(text.contains("Protocol: Subaru V1\nBit: 56\n"), "{text}");
        assert!(text.ends_with("Btn: 11\nCnt: 1111\n"), "{text}");
        let mut bad = dec.to_flipper_key(&d).unwrap();
        bad.key ^= 1;
        bad.key_bytes = bad.key.to_be_bytes().to_vec();
        assert!(dec.from_flipper_key(&bad).is_none());
    }
}
