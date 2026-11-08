//! Nissan V0 decoder (FoxFW, decode-only).
//!
//! Ported from FoxFW `lib/subghz/protocols/nissan_v0.c` (`SUBGHZ_PROTOCOL_NISSAN_V0_NAME = "Nissan
//! V0"`; the header credits the firmware `nissan_v0.c` via KeeLoq-RX `arke_vag.py` class
//! `NissanV0`). No other reference has it. AM, 315 / 433.92 MHz.
//!
//! **Line code** (`feed`): each pulse is expanded into 1..4 half-bit samples of its level by its
//! length: 320..=700 µs -> 1, 800..=1230 -> 2, 1300..=1730 -> 3, 1800..=2350 -> 4 (te 500 / 1000,
//! delta 200). Any other duration ends the burst. The samples (cap 512) are processed at the end of
//! a burst or as soon as 380 are collected; fewer than 140 are dropped.
//!
//! **Pairing** (`nissan_v0_process`): a stateless differential Manchester over the half-bit stream.
//! Two differing samples form a bit (0,1 -> 1; 1,0 -> 0) and advance by two; two equal samples slip
//! by one. At most 300 bits are produced. The decoder looks for the first position `p` (with 64 bits
//! left) where bits `p..p+6` are the sync `110001` and bits `p+48..p+54` the marker `000010`, and
//! takes the 64 bits from there as the payload. The payload is encrypted: the structural lock (12
//! fixed bits) is the only check, there is no CRC. A payload equal to the previous one is not
//! reported again until `reset()`.
//!
//! **DecodedSignal**: `data` = the 64-bit payload, `data_count_bit` = 64; the C leaves serial,
//! button and counter at 0, KAT leaves them `None`.
//!
//! **Weak gate, registered last.** The 12-bit lock is weak: the FoxFW C decoder reports a "frame"
//! in two Flipper-ARF unit-test captures (`nice_one_raw.sub`, a Nice One gate remote, and
//! `test_random_raw.sub`, random noise), both as the same payload (checked with the host oracle). In
//! KAT's registry the decoders ahead of this one already read those files (Nice One, Star Line),
//! so first-match keeps them and the corpus diff against the previous release is empty. It is the
//! last of the FoxFW block so that it only sees what every other decoder rejects. No capture of a
//! real Nissan V0 fob exists to weigh the gate against.

use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Nissan V0",
    decoder: "Nissan V0",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "Nissan",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&AM650],
};

const DATA_BITS: usize = 64;
const HB_MAX: usize = 512;
const HB_CAP: usize = 380;
const HB_MIN: usize = 140;
const BITS_MAX: usize = 300;

const SYNC: [u8; 6] = [1, 1, 0, 0, 0, 1];
const MARK: [u8; 6] = [0, 0, 0, 0, 1, 0];

pub struct NissanV0Decoder {
    hb: [u8; HB_MAX],
    hb_len: usize,
    last_data: u64,
    has_last: bool,
}

impl NissanV0Decoder {
    pub fn new() -> Self {
        Self { hb: [0; HB_MAX], hb_len: 0, last_data: 0, has_last: false }
    }

    /// `nissan_v0_process`.
    fn process(&mut self) -> Option<DecodedSignal> {
        if self.hb_len < HB_MIN {
            self.hb_len = 0;
            return None;
        }
        let mut bits = [0u8; BITS_MAX];
        let mut nbits = 0;
        let mut i = 0;
        let n = self.hb_len;
        while i + 1 < n && nbits < BITS_MAX {
            let (a, b) = (self.hb[i], self.hb[i + 1]);
            if a == b {
                i += 1;
                continue;
            }
            bits[nbits] = u8::from(a == 0 && b == 1);
            nbits += 1;
            i += 2;
        }
        let found = (0..)
            .take_while(|p| p + DATA_BITS <= nbits)
            .find(|&p| bits[p..p + 6] == SYNC && bits[p + 48..p + 54] == MARK);
        self.hb_len = 0;
        let found = found?;
        let data = bits[found..found + DATA_BITS].iter().fold(0u64, |a, &b| (a << 1) | u64::from(b));
        if self.has_last && data == self.last_data {
            return None;
        }
        self.last_data = data;
        self.has_last = true;
        Some(DecodedSignal {
            serial: None,
            button: None,
            counter: None,
            crc_valid: false,
            data,
            data_count_bit: DATA_BITS,
            encoder_capable: false,
            extra: None,
            protocol_display_name: None,
        })
    }
}

impl Default for NissanV0Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for NissanV0Decoder {
    fn name(&self) -> &'static str {
        "Nissan V0"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: 500, te_long: 1000, te_delta: 200, min_count_bit: DATA_BITS }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.hb_len = 0;
        self.last_data = 0;
        self.has_last = false;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let nh = match duration {
            320..=700 => 1,
            800..=1230 => 2,
            1300..=1730 => 3,
            1800..=2350 => 4,
            _ => return self.process(),
        };
        for _ in 0..nh {
            if self.hb_len < HB_MAX {
                self.hb[self.hb_len] = u8::from(level);
                self.hb_len += 1;
            }
        }
        if self.hb_len >= HB_CAP {
            return self.process();
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
    use crate::protocols::dd_testutil::{bits_of, dump_sub, push, Pulse};

    const PRESET: &str = "FuriHalSubGhzPresetOok650Async";

    /// Made-up 64-bit payload with the sync `110001` at the top and the marker `000010` at +48.
    fn payload(mid: u64, tail: u64) -> u64 {
        (0b11_0001u64 << 58) | ((mid & ((1 << 42) - 1)) << 16) | (0b00_0010 << 10) | (tail & 0x3FF)
    }

    /// Differential Manchester at 500 µs half-bits: 1 = LOW,HIGH; 0 = HIGH,LOW; a 24-bit
    /// alternating lead-in, the payload, and a 4 ms gap.
    fn wire(payload: u64, nbits: usize) -> Vec<Pulse> {
        let mut bits: Vec<u8> = (0..24).map(|i| (i % 2) as u8).collect();
        bits.extend(bits_of(payload, 64).into_iter().take(nbits));
        let mut p: Vec<Pulse> = Vec::new();
        for b in bits {
            let b = b != 0;
            push(&mut p, !b, 500);
            push(&mut p, b, 500);
        }
        let end = !p.last().unwrap().0;
        p.push((end, 6000));
        p
    }

    fn run(p: &[Pulse]) -> Vec<DecodedSignal> {
        let mut d = NissanV0Decoder::new();
        p.iter().filter_map(|&(l, us)| d.feed(l, us)).collect()
    }

    #[test]
    fn decodes_a_made_up_payload_once() {
        let pay = payload(0x2AB_CDEF_0123, 0x155);
        // Sent twice: the second identical payload is the C's same-payload de-dup.
        let mut p = wire(pay, 64);
        for (l, us) in wire(pay, 64) {
            push(&mut p, l, us);
        }
        dump_sub("nissan_v0_ok", 433_920_000, PRESET, &p);
        let r = run(&p);
        assert_eq!(r.len(), 1, "{r:?}");
        assert_eq!(r[0].data, pay);
        assert_eq!(pay, 0xC6AB_CDEF_0123_0955, "the key the FoxFW oracle reports");
        assert_eq!(r[0].data_count_bit, 64);
        assert!(!r[0].crc_valid);
    }

    #[test]
    fn rejects_wrong_sync_wrong_marker_and_short_frames() {
        let good = payload(0x2AB_CDEF_0123, 0x155);
        let bad_sync = good ^ (1 << 60);
        let p = wire(bad_sync, 64);
        dump_sub("nissan_v0_badsync", 433_920_000, PRESET, &p);
        assert!(run(&p).is_empty(), "sync bit flipped");
        let bad_mark = good ^ (1 << 11);
        let p = wire(bad_mark, 64);
        dump_sub("nissan_v0_badmark", 433_920_000, PRESET, &p);
        assert!(run(&p).is_empty(), "marker bit flipped");
        let p = wire(good, 63);
        dump_sub("nissan_v0_63", 433_920_000, PRESET, &p);
        assert!(run(&p).is_empty(), "63-bit payload");
    }
}
