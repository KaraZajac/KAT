//! Honda V6: Honda / Acura remotes that send one 64-bit encrypted block, Miller-coded, twice.
//!
//! **KAT's own decoder**, worked out from the captures in IMPORTS (`honda/more/` and `honda/Honda_V1_Ferrari_10_10.sub`: an OEM fob at
//! 433.92 MHz, an OEM fob at 313.85 MHz, a KeyDIY clone and one more aftermarket remote). The name
//! is FoxFW's: its `honda_v6.c` reads the same signal on its "313" path, but as "trits" (it packs
//! the pulse widths 800 / 1200 / 1600 µs two bits each, see below) and so gets a different "serial"
//! from every frame. No other reference reads the signal at all. FoxFW's three paths are ported in
//! `honda_v6_foxfw.rs` and stay unregistered.
//!
//! **Line code**: Miller (delay modulation), 800 µs per bit cell. A 1 is a transition in the middle
//! of its cell, a 0 has none, and two 0s in a row have a transition between them. The time between
//! two transitions is therefore one, one and a half or two cells: 800, 1200 or 1600 µs (2, 3 or 4
//! half cells of 400 µs). Only the transitions carry data, so the levels do not matter (the fobs
//! are 2-FSK, where which tone reads as HIGH is the receiver's choice, and the sync below starts on
//! either level).
//!
//! **Frame** (one press; about 290 ms):
//! ```text
//! ~9.5 ms at one level
//! 205 zeros            (OEM 433.92 MHz; about 100 at 313.85 MHz): 204 runs of 800 µs
//! 1 0 1                the block's start
//! 64 bits              the block
//! sync                 a transition on the cell boundary, 2400 µs, a transition, 2400 µs, a transition
//! 0 0 0, 1 0 1, the same 64 bits, sync
//! 0, then ~4070 µs at one level, carrier off
//! ```
//! The sync always starts on a cell boundary: after a block that ends in a 1 a run of half a cell
//! (400 µs) leads up to it, a width that occurs nowhere else. The KeyDIY clone sends the same
//! frame with another ending, the other aftermarket remote sends the block three times.
//!
//! **The block** is encrypted as a whole: over 900 presses of one fob every one of the 64 bits is
//! 1 about half of the time and no two blocks are alike, whatever the button. Nothing in it is in
//! the clear, so KAT reports the block and no serial, button or counter. The cipher is not known.
//!
//! **Decoder**: pulse widths only. At least 40 runs of 800 µs (zeros), then the Miller stream; the
//! cells are counted from the first 1. A block is complete at the sync when it is exactly 67 cells
//! and starts `1 0 1`. Behind a sync, 1 to 8 runs of 800 µs lead to the next block. A frame is
//! reported when a block equals the one before it, on the first pulse of the sync behind it; that
//! is the check (`crc_valid`): a wrong bit in either copy, or a Miller violation, and nothing is
//! reported. Widths: 400 / 800 / 1200 / 1600 µs ± 170, sync 2400 ± 300.
//!
//! In IMPORTS it reads every frame: 300 + 300 + 300 (lock, unlock, trunk of the OEM fob), 42 (20
//! held presses), 40 + 40 (313.85 MHz), 30 (KeyDIY) and 11 (the remote that sends three copies).
//!
//! **Encoder**: the frame above, rebuilt from the block with the OEM fob's timing, once. It sends
//! the captured block again as it is: without the cipher there is no next code to compute, and a
//! receiver that has already seen the block may refuse it. Lock / Unlock / Trunk / Panic all send
//! the same frame.
//!
//! **Fields**: `data` = the 64-bit block, `data_count_bit` = 64, no serial / button / counter.
//!
//! **FoxFW's key for the same frame** (`honda_v6.c`, variant 2): the widths of the pulses behind
//! the preamble as trits (800 → 0, 1200 → 1, 1600 → 2), two bits each, the first 32 in `Key`.
//! Its first trits are always 1, 2 (the `1 0 1` start: one and a half cells, then two). KAT writes
//! no key file for this protocol: FoxFW's holds only those 32 widths, less than the block.

use super::info::prelude::*;
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (see `info.rs`). The captures are 2-FSK (Flipper preset FM476); FoxFW flags its
/// Honda V6 AM and FM.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Miller,
    rf: Rf::FM,
    encryption: "Encrypted block (cipher unknown)",
    make: "Honda/Acura",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&FM476],
};

const NAME: &str = "Honda V6";
/// Half a bit cell (µs).
const TE: u32 = 400;
const TE_DELTA: u32 = 170;
/// The two pulses of the sync (µs).
const SYNC: u32 = 6 * TE;
const SYNC_DELTA: u32 = 300;
/// Runs of one cell that must open a frame (the fobs send 204, or about 100 at 313.85 MHz).
const MIN_PREAMBLE: u16 = 40;
/// Runs of one cell between a sync and the next block (the fobs send 2).
const MAX_GAP_ZEROS: u16 = 8;
/// A block's cells: `1 0 1` and the 64 bits.
const BLOCK_CELLS: u8 = 67;
const BLOCK_START: u128 = 0b101;
const DATA_BITS: usize = 64;

// The encoder's frame (the OEM 433.92 MHz fob).
const LEAD_IN: u32 = 9500;
const PREAMBLE_ZEROS: usize = 205;
const GAP_ZEROS: usize = 3;
const TAIL: u32 = 4070;

/// A pulse width as half cells: 1..=4, 6 for a sync pulse, 0 for anything else.
fn half_cells(duration: u32) -> u8 {
    let n = (duration + TE / 2) / TE;
    if (1..=4).contains(&n) && duration.abs_diff(n * TE) <= TE_DELTA {
        n as u8
    } else if duration.abs_diff(SYNC) <= SYNC_DELTA {
        6
    } else {
        0
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Step {
    /// Zeros in front of a block: the preamble, or the cells behind a sync.
    Lead,
    /// Inside a block.
    Block,
    /// A block ended on the sync's first pulse; its second pulse is due.
    Sync,
}

pub struct HondaV6Decoder {
    step: Step,
    /// Runs of one cell in a row in [Step::Lead].
    zeros: u16,
    /// The block's cells so far, the newest in bit 0 (the first is its leading 1).
    cells: u128,
    count: u8,
    /// The last transition was in the middle of a cell (a 1), not on a cell boundary.
    at_mid: bool,
    /// The half-cell run that leads from a final 1 to the sync was seen: the sync must follow.
    closing: bool,
    /// The block in front of this one in the same frame.
    previous: Option<u64>,
}

impl HondaV6Decoder {
    pub fn new() -> Self {
        Self { step: Step::Lead, zeros: 0, cells: 0, count: 0, at_mid: false, closing: false, previous: None }
    }

    /// Not a frame (any more): start over, with `run` as the first pulse of a new preamble if it
    /// is one cell long.
    fn restart(&mut self, run: u8) {
        self.step = Step::Lead;
        self.zeros = u16::from(run == 2);
        self.previous = None;
    }

    fn push(&mut self, bit: u8) {
        self.cells = (self.cells << 1) | u128::from(bit);
        self.count = self.count.saturating_add(1);
    }

    /// Whether the zeros seen so far may be followed by a block: a whole preamble in front of the
    /// first, the few cells behind a sync in front of the others.
    fn lead_ok(&self) -> bool {
        match self.previous {
            None => self.zeros >= MIN_PREAMBLE,
            Some(_) => (1..=MAX_GAP_ZEROS).contains(&self.zeros),
        }
    }

    /// The sync's first pulse behind a block: report the frame if the block equals the one before
    /// it, else keep it for the next.
    fn block_end(&mut self) -> Option<DecodedSignal> {
        if self.count != BLOCK_CELLS || self.cells >> DATA_BITS != BLOCK_START {
            self.restart(0);
            return None;
        }
        let block = self.cells as u64;
        if self.previous == Some(block) {
            self.restart(0);
            return Some(signal(block));
        }
        self.previous = Some(block);
        self.step = Step::Sync;
        None
    }
}

impl Default for HondaV6Decoder {
    fn default() -> Self {
        Self::new()
    }
}

/// The decode of a frame whose block is `block`.
fn signal(block: u64) -> DecodedSignal {
    DecodedSignal {
        serial: None,
        button: None,
        counter: None,
        // No checksum: the block arrived twice, alike.
        crc_valid: true,
        data: block,
        data_count_bit: DATA_BITS,
        encoder_capable: true,
        extra: None,
        protocol_display_name: None,
    }
}

/// Miller-codes cells into pulse widths (µs), level after level.
struct MillerWriter {
    widths: Vec<u32>,
    /// Half cells since the last transition.
    since: u32,
    /// The cell before the next one was a 0 (a 0 behind it gets a transition in front).
    zero_before: bool,
}

impl MillerWriter {
    /// Starts right behind a transition on a cell boundary.
    fn new() -> Self {
        Self { widths: Vec::new(), since: 0, zero_before: false }
    }

    fn transition(&mut self) {
        self.widths.push(self.since * TE);
        self.since = 0;
    }

    fn cell(&mut self, bit: bool) {
        if bit {
            self.since += 1;
            self.transition();
            self.since = 1;
        } else {
            if self.zero_before {
                self.transition();
            }
            self.since += 2;
        }
        self.zero_before = !bit;
    }

    /// `1 0 1`, the 64 bits and the sync, which starts with a transition on the cell boundary.
    fn block(&mut self, block: u64) {
        for bit in [true, false, true] {
            self.cell(bit);
        }
        for i in (0..DATA_BITS).rev() {
            self.cell((block >> i) & 1 == 1);
        }
        self.transition();
        self.widths.extend([SYNC, SYNC]);
        self.zero_before = false;
    }
}

/// One frame as the OEM fob sends it (see the module doc), starting HIGH.
fn upload(block: u64) -> Vec<LevelDuration> {
    let mut w = MillerWriter::new();
    for _ in 0..PREAMBLE_ZEROS {
        w.cell(false);
    }
    w.block(block);
    for _ in 0..GAP_ZEROS {
        w.cell(false);
    }
    w.block(block);
    w.cell(false);
    w.transition();
    let mut level = false;
    let mut out = vec![LevelDuration::new(true, LEAD_IN)];
    for width in w.widths.into_iter().chain([TAIL]) {
        out.push(LevelDuration::new(level, width));
        level = !level;
    }
    out
}

impl ProtocolDecoder for HondaV6Decoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: 2 * TE, te_long: 4 * TE, te_delta: TE_DELTA, min_count_bit: DATA_BITS }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[313_850_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.restart(0);
    }

    fn feed(&mut self, _level: bool, duration: u32) -> Option<DecodedSignal> {
        let run = half_cells(duration);
        match self.step {
            Step::Lead => match run {
                2 => self.zeros = self.zeros.saturating_add(1),
                // The last zero and half of the block's leading 1.
                3 if self.lead_ok() => {
                    self.step = Step::Block;
                    self.cells = 1;
                    self.count = 1;
                    self.at_mid = true;
                    self.closing = false;
                }
                _ => self.restart(run),
            },
            Step::Block => {
                if self.closing {
                    if run == 6 {
                        return self.block_end();
                    }
                    self.restart(run);
                    return None;
                }
                match (self.at_mid, run) {
                    (true, 2) => self.push(1),
                    (true, 3) => {
                        self.push(0);
                        self.at_mid = false;
                    }
                    (true, 4) => {
                        self.push(0);
                        self.push(1);
                    }
                    (true, 1) => self.closing = true,
                    (false, 2) => self.push(0),
                    (false, 3) => {
                        self.push(0);
                        self.push(1);
                        self.at_mid = true;
                    }
                    (false, 6) => return self.block_end(),
                    // Two cells from boundary to boundary, a sync in the middle of a cell, or a
                    // width that is none of the code's.
                    _ => self.restart(run),
                }
                if self.count > BLOCK_CELLS {
                    self.restart(0);
                }
            }
            Step::Sync => {
                if run == 6 {
                    self.step = Step::Lead;
                    self.zeros = 0;
                } else {
                    self.restart(run);
                }
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    /// The captured block again (no next code can be computed, see the module doc).
    fn encode(&self, decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        (decoded.data_count_bit == DATA_BITS).then(|| upload(decoded.data))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::ProtocolRegistry;

    fn decode(pulses: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut d = HondaV6Decoder::new();
        pulses.iter().filter_map(|p| d.feed(p.level, p.duration_us)).collect()
    }

    fn widths(pulses: &[LevelDuration]) -> Vec<u32> {
        pulses.iter().map(|p| p.duration_us).collect()
    }

    /// Blocks seen in `honda/more/HondaH1_OEM2_300lck.sub` (one ends in a 1, one in a 0).
    const ENDS_IN_ONE: u64 = 0x49788F639E3A8795;
    const ENDS_IN_ZERO: u64 = 0xF2C044AB25033F92;

    /// The encoder's frame is the fob's: lead-in, 204 runs of one cell, the `1 0 1` start (one
    /// and a half cells, two cells), the sync behind each block (led up to by half a cell after
    /// a final 1), two runs of one cell between the copies, and the ending.
    #[test]
    fn upload_is_the_fobs_frame() {
        let up = upload(ENDS_IN_ONE);
        let w = widths(&up);
        assert!(up[0].level && w[0] == 9500);
        assert!(up.windows(2).all(|p| p[0].level != p[1].level), "levels alternate");
        assert!(w[1..205].iter().all(|&d| d == 800));
        assert_eq!(w[205..207], [1200, 1600]);
        let syncs: Vec<usize> = (0..w.len() - 1).filter(|&i| w[i] == 2400 && w[i + 1] == 2400).collect();
        assert_eq!(syncs.len(), 2);
        // A final 1: half a cell before each sync; then 800, 800 and the second copy's start.
        assert_eq!(w[syncs[0] - 1], 400);
        assert_eq!(w[syncs[0] + 2..syncs[0] + 6], [800, 800, 1200, 1600]);
        assert_eq!(w[syncs[1] - 1], 400);
        assert_eq!(w[syncs[1] + 2..], [800, 4070]);
        // The two copies are the same widths.
        assert_eq!(w[205..syncs[0]], w[syncs[0] + 4..syncs[1]]);
        // About 290 ms, as in the captures.
        let total: u32 = w.iter().sum();
        assert!((285_000..300_000).contains(&total), "{total}");
        // A final 0: no half cell; the last run before the sync is a whole one or more.
        let w = widths(&upload(ENDS_IN_ZERO));
        assert!(!w.contains(&400));
        // Only the code's widths occur.
        assert!(w[1..w.len() - 1].iter().all(|d| [800, 1200, 1600, 2400].contains(d)));
    }

    #[test]
    fn decodes_its_upload_at_either_polarity_and_in_the_registry() {
        for block in [ENDS_IN_ONE, ENDS_IN_ZERO, 0, u64::MAX, 0x8000_0000_0000_0001, 0x5555_5555_5555_5555] {
            let up = upload(block);
            let got = decode(&up);
            assert_eq!(got.len(), 1, "{block:016X}");
            let s = &got[0];
            assert_eq!((s.data, s.data_count_bit, s.crc_valid), (block, 64, true));
            assert_eq!((s.serial, s.button, s.counter, s.extra), (None, None, None, None));
            // Levels inverted (the other FSK tone read as HIGH): the same frame.
            let inverted: Vec<LevelDuration> = up.iter().map(|p| LevelDuration::new(!p.level, p.duration_us)).collect();
            assert_eq!(decode(&inverted).iter().map(|s| s.data).collect::<Vec<_>>(), [block]);
            // Skewed like a receiver's output: HIGH 60 µs long, LOW 60 µs short.
            let skewed: Vec<LevelDuration> = up
                .iter()
                .map(|p| LevelDuration::new(p.level, if p.level { p.duration_us + 60 } else { p.duration_us - 60 }))
                .collect();
            assert_eq!(decode(&skewed).iter().map(|s| s.data).collect::<Vec<_>>(), [block]);
            let listed = ProtocolRegistry::new().process_signal_stream(&up, 433_920_000);
            assert_eq!(listed.iter().map(|(n, d, _)| (n.as_str(), d.data)).collect::<Vec<_>>(), [("Honda V6", block)]);
        }
        // The encoder sends the captured block, whatever the button.
        let d = HondaV6Decoder::new();
        let s = signal(ENDS_IN_ONE);
        assert_eq!(d.encode(&s, 1), Some(upload(ENDS_IN_ONE)));
        assert_eq!(d.encode(&s, 2), d.encode(&s, 4));
    }

    /// The report comes on the first pulse of the sync behind the second copy.
    #[test]
    fn reports_on_the_second_copys_sync() {
        let up = upload(ENDS_IN_ZERO);
        let w = widths(&up);
        let second_sync = (0..w.len() - 1).filter(|&i| w[i] == 2400 && w[i + 1] == 2400).nth(1).unwrap();
        let mut d = HondaV6Decoder::new();
        let at = up.iter().position(|p| d.feed(p.level, p.duration_us).is_some());
        assert_eq!(at, Some(second_sync));
    }

    /// One wrong bit in either copy, a short preamble, a missing sync pulse or a width that is
    /// not the code's: nothing is reported.
    #[test]
    fn rejects_damaged_frames() {
        let good = upload(ENDS_IN_ZERO);
        // The second copy is another block.
        let mut w = MillerWriter::new();
        (0..PREAMBLE_ZEROS).for_each(|_| w.cell(false));
        w.block(ENDS_IN_ZERO);
        (0..GAP_ZEROS).for_each(|_| w.cell(false));
        w.block(ENDS_IN_ZERO ^ 0x10);
        w.cell(false);
        w.transition();
        let mut level = true;
        let differing: Vec<LevelDuration> = w
            .widths
            .iter()
            .map(|&d| {
                level = !level;
                LevelDuration::new(level, d)
            })
            .collect();
        assert!(decode(&differing).is_empty(), "copies differ");
        // 39 runs of one cell in front of the block (the decoder wants 40).
        let short: Vec<LevelDuration> = good[1 + 204 - 39..].to_vec();
        assert!(decode(&short).is_empty(), "short preamble");
        let enough: Vec<LevelDuration> = good[1 + 204 - 40..].to_vec();
        assert_eq!(decode(&enough).len(), 1);
        // A run of 2000 µs inside the first copy.
        let mut bad = good.clone();
        bad[230].duration_us = 2000;
        assert!(decode(&bad).is_empty(), "foreign width");
        // A run a whole cell longer: the bits shift, the block is no longer 67 cells of the same.
        let mut bad = good.clone();
        let i = (207..260).find(|&i| bad[i].duration_us == 800).unwrap();
        bad[i].duration_us = 1600;
        assert!(decode(&bad).is_empty(), "stretched run");
        // The sync's second pulse missing between the copies.
        let w = widths(&good);
        let first_sync = (0..w.len() - 1).find(|&i| w[i] == 2400 && w[i + 1] == 2400).unwrap();
        let mut bad = good.clone();
        bad.remove(first_sync + 1);
        assert!(decode(&bad).is_empty(), "half a sync");
        // One copy alone (the frame cut behind its first sync).
        assert!(decode(&good[..first_sync + 2]).is_empty(), "one copy");
    }

    /// A remote that sends the block three times is one frame, not two.
    #[test]
    fn three_copies_are_one_frame() {
        let mut w = MillerWriter::new();
        (0..PREAMBLE_ZEROS).for_each(|_| w.cell(false));
        for _ in 0..3 {
            w.block(ENDS_IN_ONE);
            (0..GAP_ZEROS).for_each(|_| w.cell(false));
        }
        w.transition();
        let mut level = true;
        let pulses: Vec<LevelDuration> = w
            .widths
            .iter()
            .map(|&d| {
                level = !level;
                LevelDuration::new(level, d)
            })
            .collect();
        let listed = ProtocolRegistry::new().process_signal_stream(&pulses, 433_920_000);
        assert_eq!(listed.len(), 1);
        assert_eq!(listed[0].1.data, ENDS_IN_ONE);
    }
}
