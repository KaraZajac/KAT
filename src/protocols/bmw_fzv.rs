//! BMW FZV decoder — a KAT-own BMW remote-central-locking format (German *Funkzentralverriegelung*),
//! **decode only**. Reverse-engineered from testers' captures (2026-11-25): two cars' fobs
//! (`40_*_bmw_fzv.sub`, `Bmw_x20_*.sub`), ~90 distinct frames over 5 buttons.
//!
//! It is not any of KAT's other BMW formats — not `bmw.rs` (Flipper-ARF's 62 µs 128-bit `0x599E`
//! frame), not `bmw_v2.rs`/`bmw_v3.rs` (FoxFW's AM Manchester fobs), not the CAS3 HITAG2 telegram.
//! It shares its **physical layer with Nissan V0** (`nissan_v0.rs`): AM, ~500 µs differential
//! Manchester. Before this decoder existed, some of these frames slipped through to Nissan V0 and
//! were mis-reported (that decoder's 12-bit lock coincidentally matched); this one claims them first
//! (registered ahead of Nissan V0) on a far stronger gate.
//!
//! **Air format** (AM, 315 / 433.92 MHz): each pulse is 1 or 2 half-bit samples of its level by its
//! length (320..=700 µs → 1, 800..=1230 → 2; te 500 / 1000, delta ~200). Any other duration — in
//! particular the ~2470 µs LOW between copies — ends the burst. Stateless differential Manchester
//! over the half-bit stream (as Nissan V0): two differing samples make a bit (0,1 → 1; 1,0 → 0) and
//! advance two, two equal samples slip by one.
//!
//! **Frame** (64 bits, MSB first): `30 | e1 e2 e3 e4 e5 | id | e7`.
//! - Byte 0 is the constant sync **0x30**.
//! - Byte 6 (`id`) is a **per-car cleartext byte** — `0x40` on the `40_*` car, `0x25` on the other —
//!   constant across every press and button of a car, so it identifies the fob/car.
//! - Bytes 1..6 and byte 7 are a **48-bit encrypted rolling code**: full-entropy, changing every
//!   press, with no cleartext counter and no checksum recoverable from the air (CRC-8 sweeps over
//!   every span/polynomial and both bit orders found nothing). KAT is replay-only here.
//!
//! **Gate (repeat-confirmation).** A fob sends each frame ~4 times (copies separated by the ~2470 µs
//! gap). The decoder reports a frame only once it has seen the **same 64-bit frame twice**, which
//! makes a false positive need an identical 64-bit frame in two separate bursts (≈2⁻⁶⁴ for noise) —
//! far stronger than Nissan V0's single-shot 12-bit lock. An unconfirmed frame (seen once) is
//! dropped, and further copies of a reported frame are de-duplicated until [`reset`].
//!
//! **DecodedSignal**: `data` = the 64-bit frame, `data_count_bit` = 64, `serial` = the `id` byte,
//! `button` / `counter` = None (encrypted), `crc_valid` = false. Decode-only.

use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "BMW",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "BMW FZV";
const DATA_BITS: usize = 64;
const HB_MAX: usize = 512;
/// Process when the buffer fills (a frame is ~130 half-bits; a long burst is noise).
const HB_CAP: usize = 300;
/// A burst shorter than this cannot hold a 64-bit frame (~128 half-bits).
const HB_MIN: usize = 120;
const BITS_MAX: usize = 300;
/// Byte 0, the sync `0x30` = `0011 0000`.
const SYNC: [u8; 8] = [0, 0, 1, 1, 0, 0, 0, 0];

pub struct BmwFzvDecoder {
    hb: [u8; HB_MAX],
    hb_len: usize,
    /// A frame seen once, awaiting a second copy to confirm it.
    pending: Option<u64>,
    /// The last frame reported, to drop its remaining copies.
    reported: Option<u64>,
}

impl BmwFzvDecoder {
    pub fn new() -> Self {
        Self { hb: [0; HB_MAX], hb_len: 0, pending: None, reported: None }
    }

    /// Differential-Manchester decode of the half-bit buffer, then find the `0x30` frame and apply
    /// the repeat-confirmation gate.
    fn process(&mut self) -> Option<DecodedSignal> {
        let n = self.hb_len;
        self.hb_len = 0;
        if n < HB_MIN {
            return None;
        }
        let mut bits = [0u8; BITS_MAX];
        let mut nbits = 0;
        let mut i = 0;
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
            .find(|&p| bits[p..p + 8] == SYNC)?;
        let data = bits[found..found + DATA_BITS].iter().fold(0u64, |a, &b| (a << 1) | u64::from(b));
        self.confirm(data)
    }

    /// Report a frame only on its second sighting; de-dup the copies of a reported frame.
    fn confirm(&mut self, data: u64) -> Option<DecodedSignal> {
        if self.reported == Some(data) {
            return None; // another copy of the frame already reported
        }
        if self.pending == Some(data) {
            self.pending = None;
            self.reported = Some(data);
            return Some(signal(data));
        }
        self.pending = Some(data); // first sighting; wait for a confirming copy
        None
    }
}

/// A decode of the 64-bit frame: `serial` = the per-car `id` byte (byte 6); the rest is encrypted.
fn signal(data: u64) -> DecodedSignal {
    DecodedSignal {
        serial: Some(((data >> 8) & 0xFF) as u32),
        button: None,
        counter: None,
        crc_valid: false,
        data,
        data_count_bit: DATA_BITS,
        encoder_capable: false,
        extra: None,
        protocol_display_name: None,
    }
}

impl Default for BmwFzvDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for BmwFzvDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: 500, te_long: 1000, te_delta: 200, min_count_bit: DATA_BITS }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.hb_len = 0;
        self.pending = None;
        self.reported = None;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let nh = match duration {
            320..=700 => 1,
            800..=1230 => 2,
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

    /// Differential Manchester at 500 µs half-bits: 1 = LOW,HIGH; 0 = HIGH,LOW; a short alternating
    /// lead-in, the 64-bit frame, then the ~2470 µs inter-copy gap.
    fn copy(frame: u64) -> Vec<Pulse> {
        let mut bits: Vec<u8> = (0..16).map(|i| (i % 2) as u8).collect();
        bits.extend(bits_of(frame, 64));
        let mut p: Vec<Pulse> = Vec::new();
        for b in bits {
            let b = b != 0;
            push(&mut p, !b, 500);
            push(&mut p, b, 500);
        }
        let end = !p.last().unwrap().0;
        p.push((end, 2470));
        p
    }

    fn burst(frame: u64, copies: usize) -> Vec<Pulse> {
        (0..copies).flat_map(|_| copy(frame)).collect()
    }

    fn run(p: &[Pulse]) -> Vec<DecodedSignal> {
        let mut d = BmwFzvDecoder::new();
        p.iter().filter_map(|&(l, us)| d.feed(l, us)).collect()
    }

    /// A real frame of the `40_*` car (byte 6 = 0x40).
    const REAL: u64 = 0x3034_1026_DDF2_40AF;

    #[test]
    fn reports_a_confirmed_frame_once_with_the_id_byte() {
        // Four copies (as the fob sends): reported exactly once, on the second copy.
        let r = run(&burst(REAL, 4));
        assert_eq!(r.len(), 1, "{r:?}");
        assert_eq!(r[0].data, REAL);
        assert_eq!(r[0].data_count_bit, 64);
        assert_eq!(r[0].serial, Some(0x40), "the per-car id byte");
        assert!(!r[0].crc_valid);
        assert!(r[0].button.is_none() && r[0].counter.is_none());
        dump_sub("bmw_fzv_ok", 433_920_000, PRESET, &burst(REAL, 4));
    }

    #[test]
    fn a_single_unconfirmed_copy_is_not_reported() {
        // One copy only (e.g. noise that happens to carry a 0x30 frame) never confirms.
        assert!(run(&burst(REAL, 1)).is_empty());
    }

    #[test]
    fn a_wrong_sync_byte_is_rejected() {
        let bad = REAL & !(0xFFu64 << 56) | (0x31u64 << 56); // byte0 = 0x31, not 0x30
        assert!(run(&burst(bad, 4)).is_empty());
    }

    #[test]
    fn two_presses_each_report_once() {
        let second = 0x3099_5657_3180_406D; // another real 40_* frame
        let mut p = burst(REAL, 3);
        p.extend(burst(second, 3));
        let r = run(&p);
        assert_eq!(r.len(), 2, "{r:?}");
        assert_eq!((r[0].data, r[1].data), (REAL, second));
    }
}
