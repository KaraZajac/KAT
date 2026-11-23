//! Audi V3 (FoxFW `audi_v3.c`, `AUDI_V3_PROTOCOL_NAME = "Audi V3"`): 2-FSK VAG key fob, 96 bits —
//! **decode only**, as the reference (its encoder is NULL).
//!
//! Ported from FoxFW (itself a port of the firmware's `audi_v3.c` via KeeLoq-RX `arke_vag.py`).
//! **Air format** (FM, te ~383 / ~766 µs; flags 433 | 315 | AM | FM): a custom half-bit demodulator,
//! not the toolbox Manchester table. A preamble of at least 20 pulses of 220..=470 µs, then one long
//! pulse (580..=980 µs) that opens the data window and supplies the first bit; after that one long
//! pulse is one bit, two consecutive short pulses (under 580 µs) are one bit, and the bit's value is
//! `level ? 0 : 1` of the pulse that completes it. A pulse of 980 µs or more ends the frame; so does
//! a long pulse arriving between the two shorts of a bit.
//!
//! **Frame** (96 bits, MSB first): `button:8 | hop:56 | counter:24 | chk:8`. A frame is reported
//! only when the button is 0x0C (Unlock) or 0x0D (Lock) and the 56-bit hop is neither all 0x00 nor
//! all 0xFF. The `chk` byte is displayed by the C but never verified, so no check exists. The hop
//! is encrypted.
//!
//! **KAT fields**: `data` = `button << 56 | hop` (the C's `generic.data`), `data_count_bit` = 96,
//! `button` = the button byte, `counter` = the low 16 bits of the 24-bit counter, `extra` =
//! `counter << 8 | chk` (the full counter and the check byte), `serial` None, `crc_valid` false.
//!
//! The frame is gated only by the button whitelist and the non-degenerate hop. A gap, or a long
//! pulse between a bit's two shorts, ends the window and reports a frame only if all 96 bits are in
//! (`audi_v3_process` returns below 96).
//!
//! **Deviation: the BCM2 key's frame.** A key specialist's 2008–2015 Audi A4 key (BCM2 system,
//! 868.35 MHz; `IMPORTS/VAG/more/2008-2015_audi_a4_bcm2_*`, 20 presses of each button) sends this
//! signal, with a frame the C does not read: its first eight bits are 0x05 or 0x04, outside the
//! whitelist. The frame is 97 bits, the opening long pulse's bit (0, a start bit) and 12 bytes:
//! ```text
//! byte 0        button: 0x0A Lock, 0x08 Unlock, 0x09 Trunk (as the recordings are named)
//! bytes 1..7    48 bits that change from press to press with no pattern (encrypted)
//! bytes 7..11   counter, 32 bits: up by 3 with every press, on from one recording to the next
//! byte 11       the sum of bytes 0..11, mod 256
//! ```
//! All 61 frames pass the sum. Read from the start bit, as the C reads, the fields straddle these
//! bytes; whether FoxFW's key (0x0C / 0x0D) sent a start bit is not known, so its frames are read
//! as the C reads them. KAT tries this layout first: the 97 bits (a 97th bit lost in the closing
//! gap is the gap's, `level ? 0 : 1`, as for any bit) with a start bit of 0 (a recording of the
//! other polarity, start bit 1, is read inverted), byte 0 in 0x08..=0x0B and the sum. Such a frame
//! is packed as the C packs its 96 bits (`data` = bytes 0..8, `extra` = bytes 8..11 << 8 | byte
//! 11, so key files are the C's), with bit [BCM2_FLAG] of `extra` set; `button` = byte 0,
//! `counter` = the counter's low 16 bits, `crc_valid` = the sum, which `info.rs` checks for these
//! frames only (`PARTIAL_CHECK`).

use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (see `info.rs`): flags 433 | 315 | AM | FM, dynamic.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::Both,
    encryption: "Encrypted (rolling)",
    make: "Audi",
    category: Category::Vehicle,
    buttons: Some(buttons),
    presets: &[&AM650, &FM476],
};

/// The two buttons the C accepts (its header: 0x0C Unlock, 0x0D Lock), and the BCM2 key's
/// button bytes (module doc).
fn buttons(_name: &str, code: u8) -> Option<&'static str> {
    match code {
        0x0C | 0x08 => Some("Unlock"),
        0x0D | 0x0A => Some("Lock"),
        0x09 => Some("Trunk"),
        _ => None,
    }
}

const NAME: &str = "Audi V3";
const PRE_LO: u32 = 220;
const PRE_HI: u32 = 470;
const LONG_LO: u32 = 580;
const LONG_HI: u32 = 980;
const GAP: u32 = 980;
const MIN_PRE: u16 = 20;
const DATA_BITS: usize = 96;
/// The BCM2 key's frame: a start bit and 12 bytes (module doc).
const FRAME_BITS: usize = DATA_BITS + 1;
/// Bit of `extra` set on a frame read in the BCM2 key's layout (checked by its sum).
pub(super) const BCM2_FLAG: u32 = 40;

/// The BCM2 key's 12 bytes pass: button byte 0x08..=0x0B, and byte 11 = the sum of the others.
fn bcm2_valid(bytes: &[u8; 12]) -> bool {
    bytes[0] & 0xFC == 0x08 && bytes[..11].iter().fold(0u8, |sum, &b| sum.wrapping_add(b)) == bytes[11]
}

/// The 12 bytes of a frame in the BCM2 key's layout: 97 bits, a start bit of 0 (1 = a recording of
/// the other polarity, read inverted), then the bytes, which must pass [bcm2_valid].
fn bcm2_bytes(bits: &[u8]) -> Option<[u8; 12]> {
    if bits.len() != FRAME_BITS {
        return None;
    }
    let invert = bits[0] & 1;
    let mut bytes = [0u8; 12];
    for (i, &bit) in bits[1..].iter().enumerate() {
        bytes[i / 8] = (bytes[i / 8] << 1) | ((bit ^ invert) & 1);
    }
    bcm2_valid(&bytes).then_some(bytes)
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    Reset,
    Preamble,
    Data,
}

/// Audi V3 decoder (`SubGhzProtocolDecoderAudiV3`).
pub struct AudiV3Decoder {
    step: Step,
    pre: u16,
    half: bool,
    /// The C's 96 bits, and the BCM2 key's 97th (module doc).
    bits: [u8; FRAME_BITS],
    nbits: usize,
}

impl AudiV3Decoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            pre: 0,
            half: false,
            bits: [0; FRAME_BITS],
            nbits: 0,
        }
    }

    /// `audi_v3_clear`.
    fn clear(&mut self) {
        self.pre = 0;
        self.half = false;
        self.nbits = 0;
    }

    /// `audi_v3_append`: the bit is `level ? 0 : 1`.
    fn append(&mut self, level: bool) {
        if self.nbits < FRAME_BITS {
            self.bits[self.nbits] = u8::from(!level);
            self.nbits += 1;
        }
    }

    /// The frame in the BCM2 key's layout (KAT, module doc), else `audi_v3_process`: the C's
    /// frame, if all 96 bits are in and the button and hop pass. `gap` = the level of the gap that
    /// ended the window, whose bit is the 97th when only 96 came before it.
    fn process(&self, gap: Option<bool>) -> Option<DecodedSignal> {
        let mut bits = self.bits[..self.nbits].to_vec();
        if let (DATA_BITS, Some(level)) = (bits.len(), gap) {
            bits.push(u8::from(!level));
        }
        if let Some(bytes) = bcm2_bytes(&bits) {
            let data = u64::from_be_bytes(bytes[..8].try_into().unwrap());
            let counter = u32::from_be_bytes([0, bytes[8], bytes[9], bytes[10]]);
            return Some(signal(data, counter, bytes[11]));
        }
        if self.nbits < DATA_BITS {
            return None;
        }
        let field = |from: usize, len: usize| {
            self.bits[from..from + len]
                .iter()
                .fold(0u64, |acc, &b| (acc << 1) | u64::from(b & 1))
        };
        let button = field(0, 8) as u8;
        let hop = field(8, 56);
        let counter = field(64, 24) as u32;
        let chk = field(88, 8) as u8;
        if hop == 0 || hop == 0x00FF_FFFF_FFFF_FFFF {
            return None;
        }
        if button != 0x0C && button != 0x0D {
            return None;
        }
        Some(signal((u64::from(button) << 56) | hop, counter, chk))
    }
}

impl Default for AudiV3Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for AudiV3Decoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: 383,
            te_long: 766,
            te_delta: 180,
            min_count_bit: DATA_BITS,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.step = Step::Reset;
        self.clear();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let in_pre = (PRE_LO..=PRE_HI).contains(&duration);
        let is_long = (LONG_LO..=LONG_HI).contains(&duration);
        let is_short = duration < LONG_LO;

        match self.step {
            Step::Reset => {
                if in_pre {
                    self.pre = 1;
                    self.step = Step::Preamble;
                }
            }
            Step::Preamble => {
                if in_pre {
                    self.pre = self.pre.wrapping_add(1);
                } else if is_long && self.pre >= MIN_PRE {
                    self.step = Step::Data;
                    self.nbits = 0;
                    self.append(level); // the first bit comes from the opening long pulse
                    self.half = false;
                } else {
                    self.clear();
                    self.step = Step::Reset;
                }
            }
            Step::Data => {
                if duration >= GAP {
                    let out = self.process(Some(level));
                    self.clear();
                    self.step = Step::Reset;
                    return out;
                }
                if is_long {
                    if self.half {
                        let out = self.process(None);
                        self.clear();
                        self.step = Step::Reset;
                        return out;
                    }
                    self.append(level);
                } else if is_short {
                    if self.half {
                        self.append(level);
                        self.half = false;
                    } else {
                        self.half = true;
                    }
                }
                // The C stops at 96 bits; KAT reads on to the BCM2 key's 97th (module doc). A
                // 96-bit frame of the C's ends at the gap, or at the next bit, with the same fields.
                if self.nbits >= FRAME_BITS {
                    let out = self.process(None);
                    self.clear();
                    self.step = Step::Reset;
                    return out;
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

    /// `subghz_protocol_decoder_audi_v3_deserialize`: `Bit: 96`, the key (`button << 56 | hop`) and
    /// the optional `Cnt` (24 bits) and `Chk` lines (0 when absent); no gate is applied.
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != DATA_BITS {
            return None;
        }
        let cnt = key.u32_field("Cnt").unwrap_or(0) & 0xFF_FFFF;
        let chk = key.u32_field("Chk").unwrap_or(0) & 0xFF;
        Some(signal(key.key, cnt, chk as u8))
    }

    /// `subghz_protocol_decoder_audi_v3_serialize`: the generic Key, then `Btn`, `Cnt`, `Chk`.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != DATA_BITS {
            return None;
        }
        let extra = decoded.extra?;
        let mut key = FlipperKey::for_protocol(NAME, DATA_BITS, decoded.data);
        key.push_u32("Btn", u32::from(decoded.button?));
        key.push_u32("Cnt", (extra >> 8) as u32 & 0xFF_FFFF);
        key.push_u32("Chk", (extra & 0xFF) as u32);
        Some(key)
    }
}

/// A decode of `data` (`button << 56 | hop`), the 24-bit counter and the check byte. When the 12
/// bytes are a frame in the BCM2 key's layout (module doc), its sum is checked and [BCM2_FLAG] set.
fn signal(data: u64, counter: u32, chk: u8) -> DecodedSignal {
    let mut bytes = [0u8; 12];
    bytes[..8].copy_from_slice(&data.to_be_bytes());
    bytes[8..11].copy_from_slice(&counter.to_be_bytes()[1..]);
    bytes[11] = chk;
    let bcm2 = bcm2_valid(&bytes);
    DecodedSignal {
        serial: None,
        button: Some((data >> 56) as u8),
        counter: Some((counter & 0xFFFF) as u16),
        crc_valid: bcm2,
        data,
        data_count_bit: DATA_BITS,
        encoder_capable: false,
        extra: Some((u64::from(bcm2) << BCM2_FLAG) | (u64::from(counter & 0xFF_FFFF) << 8) | u64::from(chk)),
        protocol_display_name: None,
    }
}

/// Synthetic captures for the tests.
#[cfg(test)]
pub(super) mod testing {
    use super::*;
    use crate::protocols::foxfw_common::test_support::{bits_of, pulses_for_bits};

    pub const SHORT: u32 = 383;
    pub const LONG: u32 = 766;

    /// Pulses of a frame `button | hop (56 bits) | counter (24) | chk`: `n_pre` 383 µs preamble
    /// pulses (alternating, starting HIGH), the opening long pulse (its level makes the first bit),
    /// the other 95 bits as long pulses or short pairs, then a 2 ms LOW.
    pub fn frame(button: u8, hop: u64, counter: u32, chk: u8, n_pre: usize) -> Vec<LevelDuration> {
        let mut bits = bits_of(u64::from(button), 8);
        bits.extend(bits_of(hop, 56));
        bits.extend(bits_of(u64::from(counter), 24));
        bits.extend(bits_of(u64::from(chk), 8));
        frame_bits(&bits, n_pre)
    }

    /// A frame in the BCM2 key's layout: the start bit (0), then the 12 bytes.
    pub fn bcm2_frame(bytes: [u8; 12], n_pre: usize) -> Vec<LevelDuration> {
        let mut bits = vec![false];
        bits.extend(bytes.iter().flat_map(|&b| bits_of(u64::from(b), 8)));
        frame_bits(&bits, n_pre)
    }

    /// Pulses of any bit string, as [frame] builds them.
    pub fn frame_bits(bits: &[bool], n_pre: usize) -> Vec<LevelDuration> {
        // The opening long pulse's level gives bit 0 (`level ? 0 : 1`); the preamble alternates
        // up to it.
        let level = !bits[0];
        let mut out: Vec<LevelDuration> = (0..n_pre)
            .map(|k| LevelDuration::new(level == ((n_pre - k).is_multiple_of(2)), SHORT))
            .collect();
        out.push(LevelDuration::new(level, LONG));
        // After the opening long: `half` false. A state is `half`; a bit is `level ? 0 : 1`.
        let rest = pulses_for_bits(false, level, &bits[1..], SHORT, LONG, |half, lvl, long| {
            let bit = Some(!lvl);
            match (long, half) {
                (true, true) => None,
                (true, false) => Some((false, bit)),
                (false, true) => Some((false, bit)),
                (false, false) => Some((true, None)),
            }
        });
        out.extend(rest);
        out.push(LevelDuration::new(false, 2_000));
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::foxfw_common::test_support::*;

    const HOP: u64 = 0x0011_2233_4455_6677;

    /// Made-up frames: (name, pulses, expected (button, data, counter, chk)).
    #[allow(clippy::type_complexity)]
    fn cases() -> Vec<(&'static str, Vec<LevelDuration>, Option<(u8, u64, u32, u8)>)> {
        let mut short_frame = testing::frame(0x0C, HOP, 0x00AB_CDEF, 0x5A, 20);
        let n = short_frame.len();
        short_frame.drain(n - 9..n - 1); // the 96th bit never arrives
        let mut early_gap = testing::frame(0x0C, HOP, 0x00AB_CDEF, 0x5A, 20);
        early_gap[40].duration_us = 1_500; // a gap in the middle of the data window
        let mut bad_open = testing::frame(0x0C, HOP, 0x00AB_CDEF, 0x5A, 20);
        bad_open[20].duration_us = 1_200; // the opening pulse is out of the long window
        vec![
            ("unlock", testing::frame(0x0C, HOP, 0x00AB_CDEF, 0x5A, 20), Some((0x0C, 0x0C11_2233_4455_6677, 0x00AB_CDEF, 0x5A))),
            ("lock", testing::frame(0x0D, 0x0A_0B0C_0D0E_0F10, 0x0000_0102, 0xC3, 33), Some((0x0D, 0x0D0A_0B0C_0D0E_0F10, 0x0102, 0xC3))),
            ("bad_button", testing::frame(0x0B, HOP, 0x00AB_CDEF, 0x5A, 20), None),
            ("hop_zero", testing::frame(0x0C, 0, 0x00AB_CDEF, 0x5A, 20), None),
            ("hop_ff", testing::frame(0x0D, 0x00FF_FFFF_FFFF_FFFF, 0x00AB_CDEF, 0x5A, 20), None),
            ("short_preamble", testing::frame(0x0C, HOP, 0x00AB_CDEF, 0x5A, 19), None),
            ("bad_open", bad_open, None),
            ("early_gap", early_gap, None),
            ("short_frame", short_frame, None),
        ]
    }

    #[test]
    fn key_file_round_trip_and_decode_only() {
        let dec = AudiV3Decoder::new();
        let d = signal(0x0C11_2233_4455_6677, 0x00AB_CDEF, 0x5A);
        let text = crate::protocols::flipper_key::testing::assert_round_trip(&dec, &d);
        // the lines the C's serialize writes (checked on the oracle's output)
        assert!(text.contains("Protocol: Audi V3\nBit: 96\nKey: 0C 11 22 33 44 55 66 77\nBtn: 12\nCnt: 11259375\nChk: 90\n"), "{text}");
        let mut wrong_bits = FlipperKey::parse(&text).unwrap();
        wrong_bits.bits = 64;
        assert!(dec.from_flipper_key(&wrong_bits).is_none());
        assert!(!dec.supports_encoding() && dec.encode(&d, 1).is_none());
    }

    /// 12 bytes in the BCM2 key's layout: `button`, a made-up encrypted part, the counter, the sum.
    fn bcm2_bytes_for(button: u8, counter: u32) -> [u8; 12] {
        let mut b = [button, 0x3C, 0xA5, 0x0F, 0x96, 0x5A, 0xC3, 0, 0, 0, 0, 0];
        b[7..11].copy_from_slice(&counter.to_be_bytes());
        b[11] = b[..11].iter().fold(0u8, |s, &x| s.wrapping_add(x));
        b
    }

    /// The BCM2 key's frame (module doc): read with its button, counter and sum; also when its
    /// 97th bit is lost in the closing gap, and in the other polarity. A wrong sum is not read, by
    /// this layout or the C's (whose whitelist the first eight bits, 0x05, fail).
    #[test]
    fn the_bcm2_keys_frame_reads_with_its_sum() {
        for (button, counter) in [(0x0A, 0x0004_4769u32), (0x08, 0x0004_47A8), (0x09, 0x0004_47ED)] {
            let bytes = bcm2_bytes_for(button, counter);
            let mut lost = testing::bcm2_frame(bytes, 274);
            // The last bit's second half merged into the closing LOW (a bit 1 ends LOW).
            let gap = lost.pop().unwrap();
            if !lost.last().unwrap().level {
                lost.last_mut().unwrap().duration_us += gap.duration_us;
            } else {
                lost.push(gap);
            }
            let inverted: Vec<LevelDuration> =
                testing::bcm2_frame(bytes, 274).iter().map(|p| LevelDuration::new(!p.level, p.duration_us)).collect();
            for (case, wire) in [("whole", testing::bcm2_frame(bytes, 274)), ("lost bit", lost), ("inverted", inverted)] {
                let got = decode_all(&mut AudiV3Decoder::new(), &wire);
                assert_eq!(got.len(), 1, "{button:#04x} {case}");
                let d = &got[0];
                assert_eq!((d.button, d.counter, d.crc_valid), (Some(button), Some((counter & 0xFFFF) as u16), true), "{case}");
                assert_eq!(d.data, u64::from_be_bytes(bytes[..8].try_into().unwrap()), "{case}");
                assert_eq!(d.extra, Some((1 << BCM2_FLAG) | (u64::from(counter & 0xFF_FFFF) << 8) | u64::from(bytes[11])));
                assert!(crate::protocols::info::has_checksum_for(NAME, d.data_count_bit, d.extra));
            }
            let mut bad = bytes;
            bad[11] ^= 0x10;
            assert!(decode_all(&mut AudiV3Decoder::new(), &testing::bcm2_frame(bad, 274)).is_empty(), "{button:#04x}");
        }
        assert_eq!([buttons(NAME, 0x0A), buttons(NAME, 0x08), buttons(NAME, 0x09)], [Some("Lock"), Some("Unlock"), Some("Trunk")]);
        // The C's frames keep the C's fields and are not checked.
        let d = signal(0x0C11_2233_4455_6677, 0x00AB_CDEF, 0x5A);
        assert!(!d.crc_valid && !crate::protocols::info::has_checksum_for(NAME, d.data_count_bit, d.extra));
    }

    /// The key specialist's 2008–2015 Audi A4 (BCM2) recordings: every press reads with the button
    /// its recording is named after and a valid sum, and the counter goes up by 3 with every press
    /// and on from one recording to the next (presses between them were not recorded).
    #[test]
    fn the_a4_bcm2_recordings_read_as_their_buttons() {
        let mut last: Option<u32> = None;
        for (rel, button, presses) in [
            ("VAG/more/2008-2015_audi_a4_bcm2_lock.sub", 0x0A, 21),
            ("VAG/more/2008-2015_audi_a4_bcm2_unlock.sub", 0x08, 20),
            ("VAG/more/2008-2015_audi_a4_bcm2_trunk.sub", 0x09, 20),
        ] {
            let path = std::path::Path::new("IMPORTS").join(rel);
            if !path.exists() {
                eprintln!("Skip: {path:?} not found (run from crate root)");
                return;
            }
            let (_, pairs) = crate::export::flipper::import_sub_raw(&path).unwrap();
            let mut dec = AudiV3Decoder::new();
            let got: Vec<DecodedSignal> = pairs.iter().filter_map(|p| dec.feed(p.level, p.duration_us)).collect();
            assert_eq!(got.len(), presses, "{rel}");
            assert!(got.iter().all(|d| d.button == Some(button) && d.crc_valid), "{rel}");
            let counters: Vec<u32> =
                got.iter().map(|d| ((d.data as u32 & 0xFF) << 24) | (d.extra.unwrap() >> 8) as u32 & 0xFF_FFFF).collect();
            assert!(counters.windows(2).all(|w| w[1] == w[0] + 3), "{rel}: {counters:?}");
            assert!(last.is_none_or(|prev| counters[0] > prev), "{rel}");
            last = counters.last().copied();
        }
    }

    #[test]
    fn synthetic_frames_match_the_foxfw_oracle() {
        for (name, wire, want) in cases() {
            dump(&format!("audi_v3_{name}.sub"), 433_920_000, "FuriHalSubGhzPreset2FSKDev476Async", &wire);
            let got = decode_all(&mut AudiV3Decoder::new(), &wire);
            eprintln!("CASE {name}: {:?}", got.iter().map(|d| (format!("{:016X}", d.data), d.extra, d.button, d.counter)).collect::<Vec<_>>());
            assert_eq!(got.len(), usize::from(want.is_some()), "{name}");
            if let Some((button, data, counter, chk)) = want {
                let d = &got[0];
                assert_eq!(d.button, Some(button), "{name}");
                assert_eq!(d.data, data, "{name}");
                assert_eq!(d.extra, Some((u64::from(counter) << 8) | u64::from(chk)), "{name}");
                assert_eq!(d.counter, Some((counter & 0xFFFF) as u16), "{name}");
                assert_eq!(d.data_count_bit, 96);
            }
        }
    }
}
