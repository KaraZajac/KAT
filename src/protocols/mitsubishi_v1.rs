//! Mitsubishi V1 decoder (FoxFW's, widened to every fob), replay encoder, FoxFW's key files.
//!
//! Ported from FoxFW `lib/subghz/protocols/mitsubishi_v1.c` (`SUBGHZ_PROTOCOL_MITSUBISHI_V1_NAME =
//! "Mitsubishi V1"`; FoxFW's own header says it came from KeeLoq-RX `arke_mitsu_bmw.py`). No other
//! reference has it. It is NOT KAT's "Mitsubishi V0" (96-bit PWM pairs, or the 61-bit Kia V0 type):
//! this is 2-FSK Manchester at 200/400 µs (te_delta 100), 315 / 433.92 MHz, AM+FM flags in the C.
//!
//! **Line code** (`feed`): a half-bit-toggle Manchester. A LONG pulse (>300..=500 µs) is one bit
//! equal to its level; of two SHORT pulses (100..=300 µs) the first only arms `half` and the second
//! is one bit equal to its level. Pulses under 100 µs are ignored (state kept). A pulse over
//! 500 µs is a gap and ends the frame. The first pulse of 100..=500 µs after reset starts the
//! collection and is processed as data (fall-through in the C).
//!
//! **Frame** (96 bits): `id32 | fix8 = 0xE0 | btn4 | cnt10 | hop42`. The id starts `0001` and the
//! frame's last bit is 1 (both hold for all 169 presses in IMPORTS). The hop is encrypted, the counter
//! is the cleartext 10-bit field (one up per press, whatever the button; it wraps from 0x3FF to 0);
//! buttons 0x2 Lock, 0x8 Unlock, 0x4 Trunk (FoxFW's names). No CRC.
//!
//! **FoxFW's "sync word" is a fob's id.** The C takes the first 32 bits for a constant,
//! `0x1E4C11A7`, searches the collected bits for it and says the frame has no per-fob serial.
//! IMPORTS has two fobs (`MITSUBISHI/`): `KeyFob1_*` (six captures, 109 presses read) sends that
//! word, and is by its counts the fob FoxFW's decoder was written from ("6 captures / 111
//! presses" in the C's header); `Mitsh_*` (four captures, 60 presses) sends `0x1260F206` there,
//! with the same fix byte, the same three button codes and a counter that goes up by one from the
//! first press of its first file to the last of its last. The word is the fob's id, and FoxFW's
//! decoder reads one fob.
//!
//! **Where a frame is** (KAT's rule; the C's is under Deviations): when a gap ends the collection,
//! the last 96 bits are a frame if they start `0001`, end in a 1, carry the fix byte 0xE0 (the
//! C's "hardening" gate) and follow at least 7 equal bits. Those are the preamble: 25 short pulses
//! and a long LOW, which the reader takes for 12 zeros and the frame's first 0. When the carrier
//! comes up behind receiver noise that reads HIGH, the first preamble pulse is lost in it and the
//! reader pairs the others the other way round: 12 ones, and the frame behind them as before.
//!
//! **Deviations from the C.**
//! - Any id, as above; `serial` is the 32-bit id (the C surfaces the fix byte, 0xE0 for every fob).
//! - The frame is the 96 bits in front of the gap. The C takes up to 96 bits from its sync word
//!   wherever that is, and reports a frame cut at 88..95 bits with its hop zero-padded; KAT wants
//!   all 96 bits and the closing 1, the only checks there are on the bits behind the fix byte. (Two
//!   of the 226 copies in the `Mitsh_*` captures arrive damaged; one still has its fix byte, and
//!   only the closing 1 keeps it from being listed as a press of its own.)
//! - The collection always ends at a gap. The C's `process_frame` resets the accumulator only after
//!   it found its sync and a frame of at least 88 bits, so a gap after fewer bits keeps collecting,
//!   and a buffer of 199 bits without a sync leaves the decoder stuck until `reset()`. A press
//!   starts with a 99 ms wake-up of short pulses (248 bits), so on a whole press that is every
//!   time: FoxFW's C reads none of the presses in IMPORTS, its own fob's included. KAT starts
//!   over at each gap, and past 199 bits it drops the oldest.
//!
//! **A press** (both fobs): 248 pairs of 200 µs (about 99 ms), carrier off for about 20 ms, then
//! the frame four times (more while the button is held), each behind about 1.2 ms of the low tone
//! and 25 short pulses, with the carrier off for about 20 ms in between. The frame's Manchester cells (400 µs) are the
//! bits' own: 13 cells HIGH-LOW, one LOW-HIGH, then a cell per bit from the frame's second on (a
//! 1 is HIGH-LOW). The long LOW where the two meet is what the C's reader takes for the frame's
//! first 0.
//!
//! **Encoder** (KAT's; the C has none): the press above with the captured frame, as it is. The hop
//! is encrypted, so there is no next code to build, and a receiver that has seen the frame may
//! refuse it. Lock / Unlock / Trunk / Panic all send the same frame. The pauses are sent as the
//! low tone (a level/duration upload has no "carrier off").
//!
//! **Key files** (`subghz_protocol_decoder_mitsubishi_v1_serialize` / `_deserialize`): `Protocol:
//! Mitsubishi V1`, `Bit: 96`, `Key` = the first 64 bits, then `Fix`, `Btn`, `Cnt`, `HopHi`, `HopLo`
//! (decimal). Loading takes Bit 96, the Key and the two hop words, as the C does (no gate there).
//!
//! **DecodedSignal**: `data` = the first 64 frame bits (id + fix + btn + cnt + the first 10 hop
//! bits), `data_count_bit` = 96, `serial` = the id, `button` = btn4, `counter` = cnt10, `extra` =
//! the 42-bit hop.

use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`): FoxFW flags AM + FM, 315 + 433.92 MHz.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Mitsubishi V1",
    decoder: "Mitsubishi V1",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::Both,
    encryption: "Rolling Code",
    make: "Mitsubishi",
    category: Category::Vehicle,
    buttons: Some(buttons),
    presets: &[&FM476, &AM650],
};

const TE_SHORT: u32 = 200;
const TE_LONG: u32 = 400;
const TE_DELTA: u32 = 100;
const DATA_BITS: usize = 96;
const ALL_BITS_MAX: usize = 200;
const FIX_INVARIANT: u8 = 0xE0;
/// Equal bits in front of a frame: the preamble's 12, read in either phase.
const MIN_PREAMBLE_BITS: usize = 7;

/// The 32-bit word FoxFW takes for a sync: the id of the fob its decoder was written from
/// (`IMPORTS/MITSUBISHI/more/KeyFob1_*`).
#[cfg(test)]
const SYNC: u32 = 0x1E4C_11A7;

// The encoder's press (the fob in IMPORTS).
const WAKE_PAIRS: usize = 248;
const PAUSE: u32 = 20_000;
/// The low tone in front of each copy's preamble.
const COPY_LEAD: u32 = 1200;
/// Cells HIGH-LOW that open a copy (the last of them and the LOW-HIGH cell behind it carry the
/// frame's first bit).
const PREAMBLE_CELLS: usize = 13;
const COPIES: usize = 4;

/// Button labels of the C (`mitsubishi_v1_button_name`).
fn buttons(_name: &str, code: u8) -> Option<&'static str> {
    match code & 0x0F {
        0x2 => Some("Lock"),
        0x8 => Some("Unlock"),
        0x4 => Some("Trunk"),
        _ => None,
    }
}

pub struct MitsubishiV1Decoder {
    /// false = Reset, true = Collect.
    collecting: bool,
    half: bool,
    bits: [u8; ALL_BITS_MAX],
    bit_count: usize,
}

/// A frame's fields from its first 64 bits and its hop.
fn signal(data: u64, hop: u64) -> DecodedSignal {
    DecodedSignal {
        serial: Some((data >> 32) as u32),
        button: Some(((data >> 20) & 0x0F) as u8),
        counter: Some(((data >> 10) & 0x3FF) as u16),
        crc_valid: false,
        data,
        data_count_bit: DATA_BITS,
        encoder_capable: true,
        extra: Some(hop),
        protocol_display_name: None,
    }
}

/// The 96 frame bits of a decode: `data`, then the hop's low 32 bits.
fn frame_bits(decoded: &DecodedSignal) -> Vec<bool> {
    let hop = decoded.extra.unwrap_or(0);
    (0..64)
        .rev()
        .map(|i| (decoded.data >> i) & 1 == 1)
        .chain((0..32).rev().map(|i| (hop >> i) & 1 == 1))
        .collect()
}

/// One press with `bits` as its frame (see the module doc): the wake-up, then four copies.
fn upload(bits: &[bool]) -> Vec<LevelDuration> {
    let mut out: Vec<LevelDuration> = Vec::new();
    let mut push = |level: bool, us: u32| match out.last_mut() {
        Some(last) if last.level == level => last.duration_us += us,
        _ => out.push(LevelDuration::new(level, us)),
    };
    for _ in 0..WAKE_PAIRS {
        push(true, TE_SHORT);
        push(false, TE_SHORT);
    }
    for _ in 0..COPIES {
        push(false, PAUSE + COPY_LEAD);
        for _ in 0..PREAMBLE_CELLS {
            push(true, TE_SHORT);
            push(false, TE_SHORT);
        }
        push(false, TE_SHORT);
        push(true, TE_SHORT);
        for &bit in &bits[1..] {
            push(bit, TE_SHORT);
            push(!bit, TE_SHORT);
        }
        // The fob's last bit is a 1, whose cell ends LOW; the tone stays there a little longer.
        push(false, TE_LONG);
    }
    out
}

impl MitsubishiV1Decoder {
    pub fn new() -> Self {
        Self { collecting: false, half: false, bits: [0; ALL_BITS_MAX], bit_count: 0 }
    }

    fn reset_accum(&mut self) {
        self.collecting = false;
        self.half = false;
        self.bit_count = 0;
    }

    /// The frame in front of the gap that ends the collection (see the module doc), if there is one.
    fn frame_at_gap(&self) -> Option<DecodedSignal> {
        let start = self.bit_count.checked_sub(DATA_BITS)?.checked_sub(MIN_PREAMBLE_BITS)? + MIN_PREAMBLE_BITS;
        let lead = &self.bits[start - MIN_PREAMBLE_BITS..start];
        let frame = &self.bits[start..start + DATA_BITS];
        if lead.iter().any(|&b| b != lead[0]) || frame[..4] != [0, 0, 0, 1] || frame[DATA_BITS - 1] != 1 {
            return None;
        }
        let data = frame[..64].iter().fold(0u64, |a, &b| (a << 1) | u64::from(b));
        let hop = frame[54..].iter().fold(0u64, |a, &b| (a << 1) | u64::from(b));
        (((data >> 24) & 0xFF) as u8 == FIX_INVARIANT).then(|| signal(data, hop))
    }
}

impl Default for MitsubishiV1Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for MitsubishiV1Decoder {
    fn name(&self) -> &'static str {
        "Mitsubishi V1"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: TE_SHORT, te_long: TE_LONG, te_delta: TE_DELTA, min_count_bit: DATA_BITS }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.reset_accum();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        if !self.collecting {
            if (TE_SHORT - TE_DELTA..=TE_LONG + TE_DELTA).contains(&duration) {
                self.collecting = true;
                self.bit_count = 0;
                self.half = false;
            } else {
                return None;
            }
        }
        if self.bit_count >= ALL_BITS_MAX {
            // The C is stuck here (module doc); KAT makes room for the next bit.
            self.bits.copy_within(1.., 0);
            self.bit_count -= 1;
        }
        if duration < TE_SHORT / 2 {
            return None;
        }
        let is_long = duration > (TE_SHORT + TE_LONG) / 2 && duration <= TE_LONG + TE_DELTA;
        let is_short = (TE_SHORT - TE_DELTA..=(TE_SHORT + TE_LONG) / 2).contains(&duration);
        if duration > TE_LONG + TE_DELTA {
            // A gap ends the collection, frame or not.
            let found = self.frame_at_gap();
            self.reset_accum();
            return found;
        }
        if is_long {
            self.bits[self.bit_count] = u8::from(level);
            self.bit_count += 1;
            self.half = false;
        } else if is_short {
            if self.half {
                self.bits[self.bit_count] = u8::from(level);
                self.bit_count += 1;
                self.half = false;
            } else {
                self.half = true;
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    /// The captured frame again (see the module doc): the fob's press, whatever the button.
    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        (decoded.data_count_bit == DATA_BITS).then(|| upload(&frame_bits(decoded)))
    }

    /// `subghz_protocol_decoder_mitsubishi_v1_deserialize`: Bit 96, the Key and the hop words.
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != DATA_BITS {
            return None;
        }
        let data = key.key_u64()?;
        let hop = (u64::from(key.u32_field("HopHi").unwrap_or(0)) << 32) | u64::from(key.u32_field("HopLo").unwrap_or(0));
        Some(signal(data, hop))
    }

    /// `subghz_protocol_decoder_mitsubishi_v1_serialize`: Key, then Fix, Btn, Cnt, HopHi, HopLo.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != DATA_BITS {
            return None;
        }
        let hop = decoded.extra.unwrap_or(0);
        let mut key = FlipperKey::for_protocol("Mitsubishi V1", DATA_BITS, decoded.data);
        key.push_u32("Fix", ((decoded.data >> 24) & 0xFF) as u32);
        key.push_u32("Btn", ((decoded.data >> 20) & 0x0F) as u32);
        key.push_u32("Cnt", ((decoded.data >> 10) & 0x3FF) as u32);
        key.push_u32("HopHi", (hop >> 32) as u32);
        key.push_u32("HopLo", hop as u32);
        Some(key)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::dd_testutil::{bits_of, dump_sub, push, toggle_bits, Pulse};

    const PRESET: &str = "FuriHalSubGhzPreset2FSKDev476Async";
    const HOP: u64 = 0x02B3_C4D5_E6F7;

    /// Frame bits: id (FoxFW's "sync"), fix, btn4, cnt10, hop42.
    fn made_up(fix: u8, btn: u8, cnt: u16, hop: u64, id: u32) -> Vec<u8> {
        let mut bits = bits_of(u64::from(id), 32);
        bits.extend(bits_of(u64::from(fix), 8));
        bits.extend(bits_of(u64::from(btn), 4));
        bits.extend(bits_of(u64::from(cnt), 10));
        bits.extend(bits_of(hop, 42));
        bits
    }

    /// A short-pulse lead-in, the bits, and a 3 ms gap.
    fn wire(bits: &[u8]) -> Vec<Pulse> {
        let mut p: Vec<Pulse> = Vec::new();
        for _ in 0..8 {
            push(&mut p, true, 200);
            push(&mut p, false, 200);
        }
        toggle_bits(&mut p, bits, TE_SHORT, TE_LONG);
        let end = !p.last().unwrap().0;
        p.push((end, 3000));
        p
    }

    fn run(p: &[Pulse]) -> Vec<DecodedSignal> {
        let mut d = MitsubishiV1Decoder::new();
        p.iter().filter_map(|&(l, us)| d.feed(l, us)).collect()
    }

    fn feed_all(pulses: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut d = MitsubishiV1Decoder::new();
        pulses.iter().filter_map(|p| d.feed(p.level, p.duration_us)).collect()
    }

    /// Frames of the fob in IMPORTS (`MITSUBISHI/`), first press of the lock, unlock and trunk
    /// captures: (first 64 bits, hop, button, counter).
    const FOB_ID: u32 = 0x1260_F206;
    const FOB: &[(u64, u64, u8, u16)] = &[
        (0x1260_F206_E02E_46F6, 0x2F6_CABD_5F47, 0x2, 0x391),
        (0x1260_F206_E08D_D2F2, 0x2F2_BDFD_F3E5, 0x8, 0x374),
        (0x1260_F206_E04E_9676, 0x276_ABE2_430F, 0x4, 0x3A5),
    ];

    #[test]
    fn decodes_a_made_up_frame() {
        let p = wire(&made_up(0xE0, 0x5, 0x2A7, HOP, SYNC));
        dump_sub("mitsubishi_v1_ok", 433_920_000, PRESET, &p);
        let r = run(&p);
        assert_eq!(r.len(), 1, "{r:?}");
        let s = &r[0];
        // The C reports the fix byte as the serial; KAT the id (FoxFW's "sync word").
        assert_eq!((s.serial, s.button, s.counter), (Some(SYNC), Some(5), Some(0x2A7)));
        assert_eq!(s.extra, Some(HOP));
        assert_eq!(
            s.data,
            (u64::from(SYNC) << 32) | (0xE0 << 24) | (5 << 20) | (0x2A7 << 10) | (HOP >> 32)
        );
        assert_eq!(s.data_count_bit, 96);
        assert!(!s.crc_valid);
    }

    /// FoxFW's word is one fob's id: a frame with another id (`0001…` behind the preamble, the
    /// frame the last 96 bits before the gap) is read just the same.
    #[test]
    fn reads_any_fobs_id() {
        for id in [SYNC, FOB_ID, SYNC ^ 0x0001_0000, 0x1FFF_FFFF, 0x1000_0000] {
            let p = wire(&made_up(0xE0, 0x8, 0x155, HOP, id));
            let r = run(&p);
            assert_eq!(r.len(), 1, "{id:08X}");
            assert_eq!((r[0].serial, r[0].button, r[0].counter, r[0].extra), (Some(id), Some(8), Some(0x155), Some(HOP)));
        }
        dump_sub("mitsubishi_v1_other_id", 433_920_000, PRESET, &wire(&made_up(0xE0, 0x8, 0x155, HOP, FOB_ID)));
        // Not `0001` at the top: no frame (0000 1…, 001…, 1…).
        for id in [0x0E4C_11A7, 0x2260_F206, 0x9260_F206] {
            assert!(run(&wire(&made_up(0xE0, 0x8, 0x155, HOP, id))).is_empty(), "{id:08X}");
        }
    }

    /// The preamble read in the other phase (its first pulse lost in a HIGH before it) is ones;
    /// the frame behind it is the same. Fewer than 7 preamble bits: no frame.
    #[test]
    fn the_preamble_counts_in_either_phase() {
        let bits = frame_bits(&signal(FOB[0].0, FOB[0].1));
        // One copy as the fob sends it (`upload`), with `cells` preamble cells and `lead` µs of
        // HIGH in front.
        let copy = |cells: usize, lead: u32| {
            let mut p: Vec<Pulse> = Vec::new();
            if lead > 0 {
                push(&mut p, true, lead);
            }
            for _ in 0..cells {
                push(&mut p, true, 200);
                push(&mut p, false, 200);
            }
            push(&mut p, false, 200);
            push(&mut p, true, 200);
            for &bit in &bits[1..] {
                push(&mut p, bit, 200);
                push(&mut p, !bit, 200);
            }
            push(&mut p, false, 3000);
            p
        };
        let frames = |p: &[Pulse]| run(p).iter().map(|s| s.data).collect::<Vec<_>>();
        assert_eq!(frames(&copy(13, 0)), [FOB[0].0]);
        // 5 ms HIGH in front: the first HIGH short merges into it, the pairs read LOW-HIGH = 1.
        let other_phase = copy(13, 5000);
        assert_eq!(other_phase[0], (true, 5200));
        assert_eq!(frames(&other_phase), [FOB[0].0]);
        // 8 cells: 7 equal bits in front of the frame, the least that counts.
        assert_eq!(frames(&copy(8, 0)), [FOB[0].0]);
        assert_eq!(frames(&copy(8, 5000)), [FOB[0].0]);
        assert!(frames(&copy(7, 0)).is_empty(), "6 equal bits in front");
    }

    /// The fix byte, the closing 1 and the frame's place in front of the gap are all there is to
    /// check: a frame without one of them is not reported (the C reports a frame cut at 88 bits,
    /// with a zero-padded hop, and one with bits behind it).
    #[test]
    fn rejects_wrong_fix_byte_and_frames_that_do_not_end_at_the_gap() {
        for id in [SYNC, FOB_ID] {
            let p = wire(&made_up(0xE1, 0x5, 0x2A7, HOP, id));
            dump_sub("mitsubishi_v1_badfix", 433_920_000, PRESET, &p);
            assert!(run(&p).is_empty(), "fix byte != 0xE0");
            assert!(run(&wire(&made_up(0xE0, 0x5, 0x2A7, HOP & !1, id))).is_empty(), "last bit 0");
            for cut in [87, 88, 95] {
                let mut bits = made_up(0xE0, 0x5, 0x2A7, HOP, id);
                bits.truncate(cut);
                let p = wire(&bits);
                dump_sub(&format!("mitsubishi_v1_{cut}"), 433_920_000, PRESET, &p);
                assert!(run(&p).is_empty(), "{cut}-bit frame");
            }
            let mut long = made_up(0xE0, 0x8, 0x155, HOP, id);
            long.extend([1, 0, 1, 1, 0, 0, 1, 0, 1, 1]);
            assert!(run(&wire(&long)).is_empty(), "10 bits behind the frame");
        }
    }

    #[test]
    fn the_registry_reports_it_and_only_it() {
        let p = wire(&made_up(0xE0, 0x5, 0x2A7, HOP, SYNC));
        for freq in [315_000_000, 433_920_000] {
            assert_eq!(crate::protocols::dd_testutil::registry_names(&p, freq), ["Mitsubishi V1"]);
        }
    }

    /// The C's decoder is stuck once it holds 199 bits without its sync, and keeps collecting
    /// across gaps that follow fewer than 88 bits; KAT reads the frame behind such a stretch.
    #[test]
    fn a_frame_behind_noise_is_read() {
        let mut p: Vec<Pulse> = Vec::new();
        // 300 bits of made-up, valid-width pulses, with gaps in between.
        let noise: Vec<u8> = (0..300u32).map(|i| ((i * 7 + i / 3) % 5 < 2) as u8).collect();
        for chunk in noise.chunks(50) {
            toggle_bits(&mut p, chunk, TE_SHORT, TE_LONG);
            let end = !p.last().unwrap().0;
            p.push((end, 900));
        }
        // No gap between more of it (250 bits: past the buffer) and the preamble.
        toggle_bits(&mut p, &noise[..250], TE_SHORT, TE_LONG);
        toggle_bits(&mut p, &[0; 15], TE_SHORT, TE_LONG);
        toggle_bits(&mut p, &made_up(0xE0, 0x2, 0x391, FOB[0].1, FOB_ID)[3..], TE_SHORT, TE_LONG);
        let end = !p.last().unwrap().0;
        p.push((end, 3000));
        let r = run(&p);
        assert_eq!(r.len(), 1);
        assert_eq!((r[0].data, r[0].extra), (FOB[0].0, Some(FOB[0].1)));
    }

    /// The encoder's press is the fob's: the wake-up, four copies each behind 25 short pulses,
    /// and the decoder reads each copy as the frame it was built from.
    #[test]
    fn upload_is_the_fobs_press_and_decodes() {
        for &(data, hop, btn, cnt) in FOB {
            let src = signal(data, hop);
            assert_eq!((src.serial, src.button, src.counter), (Some(FOB_ID), Some(btn), Some(cnt)));
            let d = MitsubishiV1Decoder::new();
            let up = d.encode(&src, 0x01).unwrap();
            assert_eq!(d.encode(&src, 0x02).unwrap(), up, "every button sends the captured frame");
            // Wake-up: 248 pairs of 200 µs; then four times 21.2 ms low + 600 µs behind the frame.
            assert!(up[..495].iter().all(|p| p.duration_us == 200));
            let gaps: Vec<u32> = up.iter().filter(|p| p.duration_us > 600).map(|p| p.duration_us).collect();
            assert_eq!(gaps, [21_400, 21_800, 21_800, 21_800]);
            assert_eq!(up.last().map(|p| (p.level, p.duration_us)), Some((false, 600)));
            // One copy: 25 shorts, a long LOW, then the data; 43.4 ms without the closing LOW.
            let first = 496;
            assert!(up[first..first + 25].iter().all(|p| p.duration_us == 200));
            assert_eq!((up[first].level, up[first + 25].level, up[first + 25].duration_us), (true, false, 400));
            let copy: u32 = up[first..].iter().take_while(|p| p.duration_us <= 400).map(|p| p.duration_us).sum();
            assert_eq!(copy, 109 * 400 - 200);
            assert!(up.iter().all(|p| [200, 400, 600, 21_400, 21_800].contains(&p.duration_us)));
            let total: u32 = up.iter().map(|p| p.duration_us).sum();
            assert!((355_000..365_000).contains(&total), "{total}");

            let got = feed_all(&up);
            assert_eq!(got.len(), 4);
            assert!(got.iter().all(|g| (g.data, g.extra, g.serial, g.button, g.counter) == (data, Some(hop), Some(FOB_ID), Some(btn), Some(cnt))));
            let listed = crate::protocols::ProtocolRegistry::new().process_signal_stream(&up, 433_920_000);
            assert_eq!(listed.iter().map(|(n, d, _)| (n.as_str(), d.data)).collect::<Vec<_>>(), [("Mitsubishi V1", data); 4]);
            let pulses: Vec<Pulse> = up.iter().map(|p| (p.level, p.duration_us)).collect();
            dump_sub(&format!("mitsubishi_v1_press_{btn:x}"), 433_920_000, PRESET, &pulses);
        }
        // FoxFW's own fob (the id its C looks for) goes out the same way.
        let src = signal((u64::from(SYNC) << 32) | 0xE08D_D2F2, FOB[1].1);
        let up = MitsubishiV1Decoder::new().encode(&src, 0).unwrap();
        assert_eq!(feed_all(&up).iter().map(|g| g.data).collect::<Vec<_>>(), [src.data; 4]);
        let pulses: Vec<Pulse> = up.iter().map(|p| (p.level, p.duration_us)).collect();
        dump_sub("mitsubishi_v1_press_foxfw_id", 433_920_000, PRESET, &pulses);
    }

    /// FoxFW's key file: written, read back, and FoxFW's own text loads.
    #[test]
    fn flipper_key_files() {
        let d = MitsubishiV1Decoder::new();
        let (data, hop, btn, cnt) = FOB[0];
        let src = signal(data, hop);
        let mut key = d.to_flipper_key(&src).unwrap();
        key.frequency = 433_920_000;
        key.preset = Some("FuriHalSubGhzPreset2FSKDev476Async".to_string());
        let text = key.to_text();
        assert!(
            text.ends_with("Protocol: Mitsubishi V1\nBit: 96\nKey: 12 60 F2 06 E0 2E 46 F6\nFix: 224\nBtn: 2\nCnt: 913\nHopHi: 758\nHopLo: 3401408327\n"),
            "{text}"
        );
        let back = d.from_flipper_key(&FlipperKey::parse(&text).unwrap()).unwrap();
        assert_eq!(back, src);
        assert_eq!((back.button, back.counter), (Some(btn), Some(cnt)));
        // Another Bit, or no Key: not loaded. A file without the hop words loads with a zero hop,
        // as in the C.
        assert!(d.from_flipper_key(&FlipperKey::parse(&text.replace("Bit: 96", "Bit: 88")).unwrap()).is_none());
        let bare = text.replace("HopHi: 758\nHopLo: 3401408327\n", "");
        assert_eq!(d.from_flipper_key(&FlipperKey::parse(&bare).unwrap()).unwrap().extra, Some(0));
    }
}
