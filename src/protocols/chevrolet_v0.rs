//! Chevrolet V0 (GM 433.92 MHz keyfob, Open / Close): OOK Manchester rolling-code remote —
//! **decode and display only**, as the reference.
//!
//! Ported from FoxFW `lib/subghz/protocols/chevrolet_v0.c` (itself a port of the KeeLoq-RX
//! `arke_mazda_gm.py` classes `ChevroletV0` / `ChevroletV0Gm`); the protocol name is FoxFW's,
//! "Chevrolet V0". The C is decode-only (`.encoder = NULL`; flags 433 | 315 | AM | Decodable | Load
//! | Save, type Dynamic) and has no checksum: the hop is encrypted and the gate is structural.
//!
//! **Air format**: standard Flipper (IEEE 802.3) Manchester with `te_short` 120 µs, `te_long`
//! 240 µs, tolerance `diff < 71`; every decoded bit is inverted. A 32-bit register locks on the
//! exact preamble `0xAAAAAAAB`, then 96 bits follow. The top 64 of them (`gdata`) hold
//! `Serial(24) | Button(4) | Hop(36)`; the low 32 bits are the encrypted "seed". There is no
//! clear counter.
//!
//! **Decoder** (`subghz_protocol_decoder_chevrolet_v0_feed`): every pulse is classified (short,
//! long, or Reset when it is neither). In the reset step any non-Reset event starts the preamble
//! (Manchester state `Mid1`, empty register) and is processed as its first event; a Reset event
//! stays in reset. In the preamble a Reset event goes back to the reset step; each decoded bit is
//! shifted into the 32-bit register and, from the 32nd bit on, the register equalling
//! `0xAAAAAAAB` starts the data. In the data step each bit shifts through the 64-bit data word into
//! the register (so after 96 bits the register holds bits 95..64 and the data word 63..0); a Reset
//! event drops the frame. At 96 bits the frame is dropped when `gdata` is 0 or all ones or the
//! button nibble (bits 39..36 of `gdata`) is not `0xA` (Open) or `0xB` (Close); otherwise it is
//! reported and the decoder returns to reset. The C's optional `CHEVROLET_V0_GM_ACCEPT_ANY`
//! (accept any button nibble for the wider GM family) is off in FoxFW and on in KAT (Deviations).
//!
//! **KAT fields**: `data` = `gdata` (64 bits), `data_count_bit` = 96, `serial` = 24 bits,
//! `button` = the nibble, `extra` = the 32-bit seed, `counter` None, `crc_valid`
//! false, `encoder_capable` false.
//!
//! **Key file** (`serialize` / `deserialize`): `Protocol: Chevrolet V0`, `Bit: 96`, `Key` = `gdata`,
//! then `Serial`, `Btn`, `HopHi` (the hop's 4 high bits), `HopLo` (its low 32), `Seed`; loading
//! needs `Bit: 96` and reads `Seed` when present.
//!
//! **Deviations**: three. First, KAT has the C's own `CHEVROLET_V0_GM_ACCEPT_ANY` on, which the C's comment
//! describes as generalizing "to the whole GM 434 MHz remote family (Buick/Cadillac/GMC/newer
//! Chevrolet), whose fobs carry a per-remote type/toggle nibble instead of Open/Close": any
//! nibble is taken. The nibble is not a button. The frame is the one Opel V1 reads as `00010`, a
//! 24-bit serial and a 64-bit block (opel_v1.rs); this decoder sees it from its third bit on and
//! inverted, so its `Serial` is `010` and that serial's first 21 bits, complemented, its `Button`
//! the serial's last three bits and the block's first, complemented, and its `Hop` and `Seed` the
//! rest of the block and the 5 bits behind the frame. Three bits of the nibble are the fob's (with
//! the gate, one fob in eight is read), the fourth changes from frame to frame, and in the Trail
//! Blazer capture it is A or B whichever of five buttons is pressed. (The block does not open
//! with Opel V1's XTEA key: none of the 285 different frames of the six captures below has its
//! tail byte.)
//!
//! Second, a burst the C's reading drops is read again whole (`burst.rs`). The frame is 8.3 kBaud,
//! more than twice what a Flipper's AM650 preset filters for, and a recording can have its carrier
//! pulses 30 to 40 µs too long and its pauses as much too short: a long pause is then as long as
//! a short carrier pulse, and the C's fixed windows (`diff < 71` around 120 and 240 µs) lose the
//! frame. The second reading takes the burst's skew out, then wants what the C wants (the 32-bit
//! preamble word, 96 bits) and one thing more: the burst ending 3 bits behind the 96. (A burst is
//! the preamble and 101 bits: Opel V1's 93 and 8 more that change with every frame and are no
//! CRC-8, sum or XOR of it. The C reads 96 from the third on.) Without a checksum that length is
//! what catches a bit lost or gained inside the frame: the first version, without it, read one
//! frame in 30 of the 2020 Acadia capture with another serial. A burst the C's reading takes is
//! not read twice.
//!
//! Third, a frame whose 96 bits all but alternate (fewer than 16 of its 95 pairs of neighbouring
//! bits alike) is not reported: it is the preamble of another remote. The preamble word is itself
//! alternating bits and one pair alike, and pulses of 191 to 310 µs are this decoder's long pulse,
//! so a long run of such pulses with a few shorter ones among them is, to the C, a preamble and
//! 96 bits. A PSA remote with a 218 µs half bit sends 170: two of ten presses of
//! `IMPORTS/GROUPE PSA/Delphi_remote_unlock_x10.sub` were listed as Chevrolet V0 frames
//! (`555555556AAB5555`, `5555555AAAAAD555`), and the registry's restart cost PSA those presses.
//! With the nibble A, which alternating bits give half the time, FoxFW's own gate passes them too.
//! No frame of the seven captures comes near: the fewest pairs alike in any is 31.
//!
//! **Evidence**: `IMPORTS/chevy/22_chevy_trail_blazer_l2_u2_rs_p_t.sub` (433.92 MHz OOK270; 21
//! frames with the constant serial 0xB809C5, nibble 0xA or 0xB; matched frame for frame with the
//! host build of FoxFW's C). With the switch on, five more captures decode, each with one serial
//! and a nibble whose last bit alone changes: a 2018 Malibu (36 frames, serial A8936E, nibble
//! 8/9), a 2024 Buick Encore (34, AA67D0, 4/5), a 2024 Buick Envista (38, B6EC9C, C/D), the
//! 2016 Holden Barina (358, B61B30, C/D) and a 2020 GMC Acadia (one frame, BBC6E9). The second
//! reading adds: the Acadia 25 frames instead of 1 (skew 27 to 41 µs), the Trail Blazer 22
//! instead of 21, and one frame of a 2015 Buick LaCrosse at 315 MHz (the rest of that capture is
//! broken up beyond it). No other file in IMPORTS changes.

use super::burst::{self, BurstReader};
use super::common::{ManchesterEvent, ManchesterState};
use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (see `info.rs`): `SubGhzProtocolFlag_433 | 315 | AM`, `SubGhzProtocolTypeDynamic`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Rolling code (encrypted hop)",
    make: "Chevrolet/Buick/GMC/Holden",
    category: Category::Vehicle,
    buttons: Some(button_name),
    presets: &[&AM650],
};

const NAME: &str = "Chevrolet V0";
const TE_SHORT: u32 = 120;
const TE_LONG: u32 = 240;
const TE_DELTA: u32 = 71;
const DATA_BITS: u16 = 96;
const PREAMBLE_WORD: u32 = 0xAAAA_AAAB;
const PREAMBLE_MIN: u16 = 32;
/// The C's `CHEVROLET_V0_GM_ACCEPT_ANY`: on in KAT (see the module doc's Deviations).
const ACCEPT_ANY: bool = true;
/// Bits of a burst behind the 96 the C reads (KAT's second reading wants the burst to end there).
const TAIL_BITS: usize = 3;

fn button_name(_name: &str, code: u8) -> Option<&'static str> {
    match code {
        0xA => Some("Open"),
        0xB => Some("Close"),
        _ => None,
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    Preamble,
    CollectData,
}

/// Chevrolet V0 decoder (`SubGhzProtocolDecoderChevroletV0`).
pub struct ChevroletV0Decoder {
    step: Step,
    manchester: ManchesterState,
    data_header: u32,
    bit_count: u16,
    data_count: u16,
    decode_data: u64,
    /// KAT's second reading (Deviations): the burst whole, its skew taken out.
    burst: BurstReader,
    /// The C's reading gave a frame in the burst that is running.
    reported: bool,
}

/// `chevrolet_v0_event`: `level ? ShortLow : ShortHigh` (and the long forms).
fn event(level: bool, duration: u32) -> ManchesterEvent {
    if duration.abs_diff(TE_SHORT) < TE_DELTA {
        ManchesterEvent::from_half_bit(!level, false)
    } else if duration.abs_diff(TE_LONG) < TE_DELTA {
        ManchesterEvent::from_half_bit(!level, true)
    } else {
        ManchesterEvent::Reset
    }
}

/// `gdata`'s fields.
fn serial_of(gdata: u64) -> u32 {
    ((gdata >> 40) & 0xFF_FFFF) as u32
}

fn button_of(gdata: u64) -> u8 {
    ((gdata >> 36) & 0x0F) as u8
}

fn signal(gdata: u64, seed: u32) -> DecodedSignal {
    DecodedSignal {
        serial: Some(serial_of(gdata)),
        button: Some(button_of(gdata)),
        counter: None,
        crc_valid: false,
        data: gdata,
        data_count_bit: usize::from(DATA_BITS),
        encoder_capable: false,
        extra: Some(u64::from(seed)),
        protocol_display_name: None,
    }
}

impl ChevroletV0Decoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            manchester: ManchesterState::Mid1,
            data_header: 0,
            bit_count: 0,
            data_count: 0,
            decode_data: 0,
            burst: BurstReader::skewed(TE_SHORT, 2, 2048),
            reported: false,
        }
    }

    /// The C's `subghz_protocol_decoder_chevrolet_v0_feed`.
    fn feed_c(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let ev = event(level, duration);
        if self.step == Step::Reset {
            if ev == ManchesterEvent::Reset {
                return None;
            }
            self.manchester = ManchesterState::Mid1;
            self.data_header = 0;
            self.bit_count = 0;
            self.step = Step::Preamble;
        }
        if self.step == Step::Preamble {
            if ev == ManchesterEvent::Reset {
                self.step = Step::Reset;
                return None;
            }
            if let Some(mbit) = self.manchester.advance(ev) {
                let data = u32::from(!mbit);
                self.data_header = (self.data_header << 1) | data;
                self.bit_count = self.bit_count.wrapping_add(1);
                if self.bit_count >= PREAMBLE_MIN && self.data_header == PREAMBLE_WORD {
                    self.data_header = 0;
                    self.decode_data = 0;
                    self.data_count = 0;
                    self.step = Step::CollectData;
                }
            }
        } else if self.step == Step::CollectData {
            if ev == ManchesterEvent::Reset {
                self.step = Step::Reset;
                return None;
            }
            if let Some(mbit) = self.manchester.advance(ev) {
                let data = u64::from(!mbit);
                self.data_header = (self.data_header << 1) | ((self.decode_data >> 63) & 1) as u32;
                self.decode_data = (self.decode_data << 1) | data;
                self.data_count += 1;
                if self.data_count >= DATA_BITS {
                    let gdata = (u64::from(self.data_header) << 32) | (self.decode_data >> 32);
                    let seed = (self.decode_data & 0xFFFF_FFFF) as u32;
                    self.step = Step::Reset;
                    return gated(gdata, seed);
                }
            }
        }
        None
    }

    /// A burst is over: KAT's second reading, unless the C's reading took a frame from it.
    fn burst_ended(&mut self, halves: &[bool]) -> Option<DecodedSignal> {
        if std::mem::take(&mut self.reported) {
            return None;
        }
        read_burst(halves)
    }
}

/// The C's gate on a frame: not degenerate and, without the GM switch, the nibble A or B. KAT's
/// own: not a preamble either (Deviations).
fn gated(gdata: u64, seed: u32) -> Option<DecodedSignal> {
    let btn = button_of(gdata);
    if gdata == 0 || gdata == u64::MAX || !(ACCEPT_ANY || btn == 0xA || btn == 0xB) {
        return None;
    }
    if alternates(gdata, seed) {
        return None;
    }
    Some(signal(gdata, seed))
}

/// Fewer of the 95 pairs of neighbouring bits alike than this: the bits all but alternate.
const MIN_ALIKE: u32 = 16;

/// The 96 bits all but alternate: another remote's preamble of 240 µs pulses, not a frame. (Half
/// the pairs of neighbouring bits of a frame are alike: 47 of 95, give or take 5. Of the 519
/// frames this decoder reads from the seven captures in IMPORTS, none has fewer than 31.)
fn alternates(gdata: u64, seed: u32) -> bool {
    let bits = (u128::from(gdata) << 32) | u128::from(seed);
    let alike = !(bits ^ (bits >> 1)) & ((1 << 95) - 1);
    alike.count_ones() < MIN_ALIKE
}

/// KAT's second reading: a whole burst with its skew taken out. The preamble word, the C's 96
/// bits and 3 more, and the burst ends (at most two half bits behind them).
fn read_burst(halves: &[bool]) -> Option<DecodedSignal> {
    let word = burst::pattern(u64::from(PREAMBLE_WORD), 32);
    let frame_bits = usize::from(DATA_BITS) + TAIL_BITS;
    for phase in 0..2 {
        // the C inverts every bit: HIGH then LOW is its 0
        let bits = burst::bits(halves, phase, true);
        for end in (32 + frame_bits..=bits.len()).rev() {
            if burst::spare(halves.len(), phase, end) > 2 {
                break;
            }
            let at = end - frame_bits;
            if !burst::matches(&bits, at - 32, &word) {
                continue;
            }
            if let Some(frame) = burst::take(&bits, at, frame_bits) {
                return gated(burst::number(&frame[..64]), burst::number(&frame[64..96]) as u32);
            }
        }
    }
    None
}

impl Default for ChevroletV0Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for ChevroletV0Decoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: usize::from(DATA_BITS),
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.step = Step::Reset;
        self.manchester = ManchesterState::Mid1;
        self.data_header = 0;
        self.bit_count = 0;
        self.data_count = 0;
        self.decode_data = 0;
        self.burst.reset();
        self.reported = false;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let ended = self.burst.feed(level, duration);
        if let Some(frame) = self.feed_c(level, duration) {
            self.reported = true;
            return Some(frame);
        }
        self.burst_ended(&ended?)
    }

    fn flush(&mut self) -> Option<DecodedSignal> {
        let halves = self.burst.flush()?;
        self.burst_ended(&halves)
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }

    /// `subghz_protocol_decoder_chevrolet_v0_deserialize`: `Bit: 96`; the fields come from `Key`,
    /// `Seed` is read when present (else 0).
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != usize::from(DATA_BITS) {
            return None;
        }
        Some(signal(key.key, key.u32_field("Seed").unwrap_or(0)))
    }

    /// `subghz_protocol_decoder_chevrolet_v0_serialize`.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != usize::from(DATA_BITS) {
            return None;
        }
        let gdata = decoded.data;
        let hop = gdata & 0xF_FFFF_FFFF;
        let mut key = FlipperKey::for_protocol(NAME, usize::from(DATA_BITS), gdata);
        key.push_u32("Serial", serial_of(gdata));
        key.push_u32("Btn", button_of(gdata).into());
        key.push_u32("HopHi", (hop >> 32) as u32);
        key.push_u32("HopLo", (hop & 0xFFFF_FFFF) as u32);
        key.push_u32("Seed", decoded.extra? as u32);
        Some(key)
    }
}

/// Synthetic captures for the tests.
#[cfg(test)]
pub(super) mod testing {
    use super::*;

    /// Pulses of a frame: `pre` preamble bits (alternating, ending in the `0xAAAAAAAB` word), the 96
    /// payload bits (`gdata` then `seed`), then a long LOW. Bits are on-air (inverted from the
    /// decoded value): a decoded `1` is a LOW-then-HIGH pair.
    pub fn frame(pre: usize, gdata: u64, seed: u32) -> Vec<LevelDuration> {
        burst(pre, gdata, seed, &[])
    }

    /// The same with `tail` bits behind the payload (a remote sends three).
    pub fn burst(pre: usize, gdata: u64, seed: u32, tail: &[bool]) -> Vec<LevelDuration> {
        let mut bits: Vec<bool> = (0..pre.saturating_sub(32)).map(|i| i % 2 == 1).collect();
        bits.extend((0..32).rev().map(|i| (PREAMBLE_WORD >> i) & 1 == 1));
        bits.extend((0..64).rev().map(|i| (gdata >> i) & 1 == 1));
        bits.extend((0..32).rev().map(|i| (seed >> i) & 1 == 1));
        bits.extend(tail);
        let mut halves = Vec::new();
        for b in bits {
            halves.extend([!b, b]);
        }
        let mut out = Vec::new();
        let mut i = 0;
        while i < halves.len() {
            let level = halves[i];
            let run = if halves.get(i + 1) == Some(&level) { 2 } else { 1 };
            out.push(LevelDuration::new(level, if run == 2 { TE_LONG } else { TE_SHORT }));
            i += run;
        }
        out.push(LevelDuration::new(false, 5_000));
        out
    }
}

#[cfg(test)]
mod tests {
    use super::super::f_common::test_support::*;
    use super::*;

    const GDATA: u64 = 0xB809_C5B4_779E_46A9;
    const SEED: u32 = 980_305_051;

    #[test]
    fn decodes_a_synthetic_frame() {
        let got = feed_resetting(&mut ChevroletV0Decoder::new(), &testing::frame(40, GDATA, SEED));
        assert_eq!(got.len(), 1, "{got:?}");
        let d = &got[0];
        assert_eq!((d.serial, d.button, d.counter), (Some(0xB8_09C5), Some(0xB), None));
        assert_eq!((d.data, d.data_count_bit, d.extra), (GDATA, 96, Some(u64::from(SEED))));
        assert!(!d.encoder_capable && !d.crc_valid);
    }

    #[test]
    fn gates_match_the_c() {
        // Open is accepted, and with the C's GM switch on so is any other nibble
        let open = (GDATA & !(0xF << 36)) | (0xA << 36);
        assert_eq!(feed_resetting(&mut ChevroletV0Decoder::new(), &testing::frame(40, open, SEED)).len(), 1);
        let other = (GDATA & !(0xF << 36)) | (0x3 << 36);
        let got = feed_resetting(&mut ChevroletV0Decoder::new(), &testing::frame(40, other, SEED));
        assert_eq!(got.iter().map(|d| d.button).collect::<Vec<_>>(), [Some(0x3)]);
        // all-ones / zero payloads are dropped
        assert!(feed_resetting(&mut ChevroletV0Decoder::new(), &testing::frame(40, 0, 0)).is_empty());
        assert!(feed_resetting(&mut ChevroletV0Decoder::new(), &testing::frame(40, u64::MAX, SEED)).is_empty());
        // an out-of-tolerance pulse inside the data drops the frame
        let mut wire = testing::frame(40, GDATA, SEED);
        let mid = wire.len() * 3 / 4;
        wire[mid].duration_us = 1_000;
        assert!(feed_resetting(&mut ChevroletV0Decoder::new(), &wire).is_empty());
        assert!(ChevroletV0Decoder::new().encode(&DecodedSignal::new(0, 96), 1).is_none());
    }

    #[test]
    fn key_file_round_trip() {
        let dec = ChevroletV0Decoder::new();
        let d = signal(GDATA, SEED);
        let text = super::super::flipper_key::testing::assert_round_trip(&dec, &d);
        assert!(
            text.contains("Protocol: Chevrolet V0\nBit: 96\nKey: B8 09 C5 B4 77 9E 46 A9\n"),
            "{text}"
        );
        assert!(
            text.contains("Serial: 12061125\nBtn: 11\nHopHi: 4\nHopLo: 2006861481\nSeed: 980305051\n"),
            "{text}"
        );
    }

    /// KAT's second reading: a burst whose carrier pulses are 55 µs too long and whose pauses are
    /// 55 µs too short, which the C's windows lose (a long pause of 185 µs is its "short").
    #[test]
    fn a_skewed_burst_is_read_whole() {
        let skewed = |wire: &[LevelDuration]| -> Vec<LevelDuration> {
            let last = wire.len() - 1;
            wire.iter()
                .enumerate()
                .map(|(i, p)| match (i == last, p.level) {
                    (true, _) => *p,
                    (_, true) => LevelDuration::new(true, p.duration_us + 55),
                    (_, false) => LevelDuration::new(false, p.duration_us - 55),
                })
                .collect()
        };
        let clean = testing::burst(42, GDATA, SEED, &[true, false, true]);
        let wire = skewed(&clean);
        let mut c_only = ChevroletV0Decoder::new();
        assert!(wire.iter().all(|p| c_only.feed_c(p.level, p.duration_us).is_none()), "the C's reading loses it");
        for feed in [feed_all, feed_resetting] {
            let got = feed(&mut ChevroletV0Decoder::new(), &wire);
            assert_eq!(got.len(), 1, "{got:?}");
            assert_eq!((got[0].data, got[0].extra, got[0].serial), (GDATA, Some(u64::from(SEED)), Some(0xB8_09C5)));
            // a clean burst is read once, by the C's reading
            assert_eq!(feed(&mut ChevroletV0Decoder::new(), &clean).len(), 1);
        }
        // the burst must end 3 bits behind the 96: 2 or 4 bits there are another length
        for tail in [&[true, false][..], &[true, false, true, true][..], &[][..]] {
            let wire = skewed(&testing::burst(42, GDATA, SEED, tail));
            assert!(feed_all(&mut ChevroletV0Decoder::new(), &wire).is_empty(), "{} tail bits", tail.len());
        }
        // the preamble word is wanted whole, and a degenerate frame is dropped as in the C
        let mut wire = skewed(&testing::burst(42, GDATA, SEED, &[true, false, true]));
        wire.drain(..14);
        assert!(feed_all(&mut ChevroletV0Decoder::new(), &wire).is_empty());
        let wire = skewed(&testing::burst(42, u64::MAX, SEED, &[true, false, true]));
        assert!(feed_all(&mut ChevroletV0Decoder::new(), &wire).is_empty());
        // two skewed bursts, two frames; the end of the input counts as the end of a burst
        let mut two = skewed(&testing::burst(42, GDATA, SEED, &[false, false, true]));
        let mut second = skewed(&testing::burst(42, GDATA ^ 0xFF, SEED, &[true, true, false]));
        second.pop();
        two.extend(second);
        let mut dec = ChevroletV0Decoder::new();
        let mut got = feed_all(&mut dec, &two);
        got.extend(dec.flush());
        assert_eq!(got.iter().map(|d| d.data).collect::<Vec<_>>(), [GDATA, GDATA ^ 0xFF]);
    }

    /// The wider GM family (the C's switch): a 2018 Malibu, a 2024 Encore and a 2024 Envista. Each
    /// has one serial and a nibble whose last bit alone changes, whatever button is pressed.
    #[test]
    fn imports_gm_family() {
        for (path, serial, nibbles, frames) in [
            ("IMPORTS/chevy/18_chevy_malibu_l2_u2_rs2_t2_p2.sub", 0xA8_936E, [0x8, 0x9], 36),
            ("IMPORTS/buick/24_buick_encore_ul2_t2_l2_rs2_p2.sub", 0xAA_67D0, [0x4, 0x5], 34),
            ("IMPORTS/buick/24_buick_envista_ul2_l2_t2_rs2_p2.sub", 0xB6_EC9C, [0xC, 0xD], 38),
            // the second reading: 25 frames where the C's reading gives 1
            ("IMPORTS/gmc/20_gmc_acadia_l2_u2_rs2_t2_p2.sub", 0xBB_C6E9, [0xC, 0xD], 25),
        ] {
            let path = std::path::Path::new(path);
            if !path.is_file() {
                eprintln!("Skip: IMPORTS not found (run from crate root)");
                return;
            }
            let (_, wire) = sub_pulses(path);
            let got = feed_resetting(&mut ChevroletV0Decoder::new(), &wire);
            let mut seen: Vec<u8> = got.iter().map(|d| d.button.unwrap()).collect();
            seen.sort_unstable();
            seen.dedup();
            eprintln!("CHEVY {}: {} frames, nibbles {seen:X?}", path.display(), got.len());
            assert_eq!(got.len(), frames, "{}", path.display());
            assert!(got.iter().all(|d| d.serial == Some(serial)), "{}", path.display());
            assert_eq!(seen, nibbles, "{}", path.display());
        }
    }

    /// A frame whose bits all but alternate is another remote's preamble: a PSA remote with a
    /// 218 µs half bit sends about 170 pulses of that length, which are long pulses here, and where
    /// a few of them come out under 191 µs the C's reading has its preamble word and 96 bits. Two
    /// of ten presses of `Delphi_remote_unlock_x10.sub` gave such a frame (and cost PSA the press).
    /// The real frames are far from it: the fewest neighbouring bits alike in any of them is pinned.
    #[test]
    fn a_preamble_is_no_frame() {
        // the two frames that recording gave
        for (gdata, seed) in [(0x5555_5555_6AAB_5555, 0x5555_5555), (0x5555_555A_AAAA_D555, 0x555A_AAAA)] {
            assert!(alternates(gdata, seed), "{gdata:016X}");
            assert!(gated(gdata, seed).is_none());
            let wire = testing::burst(42, gdata, seed, &[true, false, true]);
            assert!(feed_all(&mut ChevroletV0Decoder::new(), &wire).is_empty());
        }
        assert!(!alternates(GDATA, SEED));
        let mut fewest = u32::MAX;
        let mut frames = 0;
        for path in [
            "IMPORTS/chevy/22_chevy_trail_blazer_l2_u2_rs_p_t.sub",
            "IMPORTS/chevy/18_chevy_malibu_l2_u2_rs2_t2_p2.sub",
            "IMPORTS/buick/24_buick_encore_ul2_t2_l2_rs2_p2.sub",
            "IMPORTS/buick/24_buick_envista_ul2_l2_t2_rs2_p2.sub",
            "IMPORTS/buick/15_buick_lacross_t2_rs2_p2_l2_u2.sub",
            "IMPORTS/gmc/20_gmc_acadia_l2_u2_rs2_t2_p2.sub",
            "IMPORTS/HOLDEN/Holden Barina 2016 AU.sub",
        ] {
            let path = std::path::Path::new(path);
            if !path.is_file() {
                eprintln!("Skip: IMPORTS not found (run from crate root)");
                return;
            }
            for d in feed_resetting(&mut ChevroletV0Decoder::new(), &sub_pulses(path).1) {
                let bits = (u128::from(d.data) << 32) | u128::from(d.extra.unwrap());
                fewest = fewest.min((!(bits ^ (bits >> 1)) & ((1 << 95) - 1)).count_ones());
                frames += 1;
            }
        }
        eprintln!("CHEVY alike: {frames} frames, fewest pairs alike {fewest}");
        assert_eq!((frames, fewest), (519, 31));
        // the PSA recordings give no frame of this decoder
        for file in ["lock", "unlock", "button3"] {
            let path = format!("IMPORTS/GROUPE PSA/Delphi_remote_{file}_x10.sub");
            let got = feed_resetting(&mut ChevroletV0Decoder::new(), &sub_pulses(std::path::Path::new(&path)).1);
            assert!(got.is_empty(), "{path}: {got:?}");
        }
    }

    /// FoxFW's C on the Trail Blazer capture: 21 frames, constant serial 0xB809C5, button 0xA or
    /// 0xB; the first frames' `Key`, `HopHi`, `HopLo` and `Seed`.
    #[test]
    fn imports_trail_blazer_matches_the_reference() {
        let path = std::path::Path::new("IMPORTS/chevy/22_chevy_trail_blazer_l2_u2_rs_p_t.sub");
        if !path.is_file() {
            eprintln!("Skip: IMPORTS not found (run from crate root)");
            return;
        }
        let (_, wire) = sub_pulses(path);
        // the C's reading alone, as the reference ran it
        let mut c = ChevroletV0Decoder::new();
        let got: Vec<DecodedSignal> = wire.iter().filter_map(|p| c.feed_c(p.level, p.duration_us)).collect();
        assert_eq!(got.len(), 21);
        assert!(got.iter().all(|d| d.serial == Some(12_061_125) && matches!(d.button, Some(0xA | 0xB))));
        // with KAT's second reading, one burst more of the same fob
        let all = feed_resetting(&mut ChevroletV0Decoder::new(), &wire);
        assert_eq!(all.len(), 22);
        assert!(all.iter().all(|d| d.serial == Some(12_061_125)));
        assert!(got.iter().all(|d| all.contains(d)));
        let first: Vec<(u64, u64)> = got.iter().take(3).map(|d| (d.data, d.extra.unwrap())).collect();
        assert_eq!(
            first,
            [
                (0xB809_C5B4_779E_46A9, 980_305_051),
                (0xB809_C5B3_5FD2_0392, 3_581_925_308),
                (0xB809_C5A2_93A9_A01E, 114_341_953),
            ]
        );
        assert_eq!(got[0].data & 0xF_FFFF_FFFF, (4 << 32) | 2_006_861_481);
    }
}
