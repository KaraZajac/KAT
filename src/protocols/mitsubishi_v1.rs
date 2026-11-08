//! Mitsubishi V1 decoder (FoxFW, decode-only).
//!
//! Ported from FoxFW `lib/subghz/protocols/mitsubishi_v1.c` (`SUBGHZ_PROTOCOL_MITSUBISHI_V1_NAME =
//! "Mitsubishi V1"`; FoxFW's own header says it came from KeeLoq-RX `arke_mitsu_bmw.py`). No other
//! reference has it. It is NOT KAT's "Mitsubishi V0" (96-bit PWM pairs, or the 61-bit Kia V0 type):
//! this is 2-FSK Manchester at 200/400 µs (te_delta 100), 315 / 433.92 MHz, AM+FM flags in the C.
//!
//! **Line code** (`feed`): a half-bit-toggle Manchester. A LONG pulse (>300..=500 µs) is one bit
//! equal to its level; of two SHORT pulses (100..=300 µs) the first only arms `half` and the second
//! is one bit equal to its level. Pulses under 100 µs are ignored (state kept). A pulse over
//! 500 µs is a gap and ends the frame; so does a buffer of 199 bits. The first pulse of 100..=500 µs
//! after reset starts the collection and is processed as data (fall-through in the C).
//!
//! **Frame** (96 bits): `sync32 = 0x1E4C11A7 | fix8 = 0xE0 | btn4 | cnt10 | hop42`. The decoder
//! searches the collected bits for the sync (it needs 56 more bits behind it), takes up to 96 bits
//! from there (at least 88), and reports the frame only if the fix byte is 0xE0 (the C's single
//! "hardening" gate; there is no CRC). The hop is encrypted, the counter is the cleartext 10-bit
//! field. A frame cut at 88..95 bits has its hop zero-padded on the right, as in the C.
//!
//! Faithful quirks: `process_frame` resets the accumulator only after it found a sync and a frame of
//! at least 88 bits, so a gap after fewer than 88 bits (or without a sync) keeps collecting, and a buffer
//! of 199 bits without a sync leaves the decoder stuck until `reset()` (the C has no other way out;
//! KAT's registry resets every decoder after a hit and at the start of each capture).
//!
//! **DecodedSignal**: `data` = the first 64 frame bits (sync + fix + btn + cnt + the first 10 hop
//! bits), `data_count_bit` = 96, `serial` = the fix byte (the C surfaces it as the serial: the frame
//! has no per-fob ID), `button` = btn4, `counter` = cnt10, `extra` = the 42-bit hop.

use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
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
    buttons: None,
    presets: &[&FM476, &AM650],
};

const TE_SHORT: u32 = 200;
const TE_LONG: u32 = 400;
const TE_DELTA: u32 = 100;
const DATA_BITS: usize = 96;
const ALL_BITS_MAX: usize = 200;
const FIX_INVARIANT: u8 = 0xE0;

/// 32-bit sync word 0x1E4C11A7, MSB first.
const SYNC: u32 = 0x1E4C_11A7;

pub struct MitsubishiV1Decoder {
    /// false = Reset, true = Collect.
    collecting: bool,
    half: bool,
    bits: [u8; ALL_BITS_MAX],
    bit_count: usize,
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

    /// `mitsubishi_v1_find_sync`: the first index with the sync word and 56 more bits behind it.
    fn find_sync(&self) -> Option<usize> {
        let search_end = self.bit_count as i32 - 32 - 56;
        (0..=search_end)
            .map(|i| i as usize)
            .find(|&i| (0..32).all(|j| u32::from(self.bits[i + j]) == (SYNC >> (31 - j)) & 1))
    }

    /// `mitsubishi_v1_process_frame`.
    fn process_frame(&mut self) -> Option<DecodedSignal> {
        let n = self.bit_count;
        if n < 88 {
            return None;
        }
        let sync_pos = self.find_sync()?;
        let data_len = (n - sync_pos).min(96);
        if data_len < 88 {
            return None;
        }
        let frame = &self.bits[sync_pos..sync_pos + data_len];

        let mut data = 0u64;
        for &b in &frame[..64] {
            data = (data << 1) | u64::from(b & 1);
        }
        let mut hop = 0u64;
        let mut hop_bits = 0;
        for &b in &frame[54..data_len] {
            hop = (hop << 1) | u64::from(b & 1);
            hop_bits += 1;
        }
        if hop_bits < 42 {
            hop <<= 42 - hop_bits;
        }

        let fix = ((data >> 24) & 0xFF) as u8;
        let btn = ((data >> 20) & 0x0F) as u8;
        let cnt = ((data >> 10) & 0x3FF) as u16;

        self.reset_accum();

        if fix != FIX_INVARIANT {
            return None;
        }
        Some(DecodedSignal {
            serial: Some(u32::from(fix)),
            button: Some(btn),
            counter: Some(cnt),
            crc_valid: false,
            data,
            data_count_bit: DATA_BITS,
            encoder_capable: false,
            extra: Some(hop),
            protocol_display_name: None,
        })
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
        if self.bit_count >= ALL_BITS_MAX - 1 {
            return self.process_frame();
        }
        if duration < TE_SHORT / 2 {
            return None;
        }
        let is_long = duration > (TE_SHORT + TE_LONG) / 2 && duration <= TE_LONG + TE_DELTA;
        let is_short = (TE_SHORT - TE_DELTA..=(TE_SHORT + TE_LONG) / 2).contains(&duration);
        if duration > TE_LONG + TE_DELTA {
            return self.process_frame();
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
        false
    }

    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::dd_testutil::{bits_of, dump_sub, push, toggle_bits, Pulse};

    const PRESET: &str = "FuriHalSubGhzPreset2FSKDev476Async";
    const HOP: u64 = 0x02B3_C4D5_E6F7;

    /// Frame bits: sync, fix, btn4, cnt10, hop42.
    fn frame_bits(fix: u8, btn: u8, cnt: u16, hop: u64, sync: u32) -> Vec<u8> {
        let mut bits = bits_of(u64::from(sync), 32);
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

    #[test]
    fn decodes_a_made_up_frame() {
        let p = wire(&frame_bits(0xE0, 0x5, 0x2A7, HOP, SYNC));
        dump_sub("mitsubishi_v1_ok", 433_920_000, PRESET, &p);
        let r = run(&p);
        assert_eq!(r.len(), 1, "{r:?}");
        let s = &r[0];
        assert_eq!((s.serial, s.button, s.counter), (Some(0xE0), Some(5), Some(0x2A7)));
        assert_eq!(s.extra, Some(HOP));
        assert_eq!(
            s.data,
            (u64::from(SYNC) << 32) | (0xE0 << 24) | (5 << 20) | (0x2A7 << 10) | (HOP >> 32)
        );
        assert_eq!(s.data_count_bit, 96);
        assert!(!s.crc_valid);
    }

    #[test]
    fn rejects_wrong_fix_byte_wrong_sync_and_short_frame() {
        let p = wire(&frame_bits(0xE1, 0x5, 0x2A7, HOP, SYNC));
        dump_sub("mitsubishi_v1_badfix", 433_920_000, PRESET, &p);
        assert!(run(&p).is_empty(), "fix byte != 0xE0");

        let p = wire(&frame_bits(0xE0, 0x5, 0x2A7, HOP, SYNC ^ 0x0001_0000));
        dump_sub("mitsubishi_v1_badsync", 433_920_000, PRESET, &p);
        assert!(run(&p).is_empty(), "one sync bit wrong");

        let mut bits = frame_bits(0xE0, 0x5, 0x2A7, HOP, SYNC);
        bits.truncate(87);
        let p = wire(&bits);
        dump_sub("mitsubishi_v1_short", 433_920_000, PRESET, &p);
        assert!(run(&p).is_empty(), "87-bit frame");
    }

    #[test]
    fn the_registry_reports_it_and_only_it() {
        let p = wire(&frame_bits(0xE0, 0x5, 0x2A7, HOP, SYNC));
        for freq in [315_000_000, 433_920_000] {
            assert_eq!(crate::protocols::dd_testutil::registry_names(&p, freq), ["Mitsubishi V1"]);
        }
    }

    #[test]
    fn a_frame_cut_at_88_bits_pads_the_hop() {
        let mut bits = frame_bits(0xE0, 0x3, 0x155, HOP, SYNC);
        bits.truncate(88);
        let p = wire(&bits);
        dump_sub("mitsubishi_v1_88", 433_920_000, PRESET, &p);
        let r = run(&p);
        assert_eq!(r.len(), 1);
        assert_eq!(r[0].extra, Some((HOP >> 8) << 8));
    }
}
