//! Volvo S40: the remote of a 1999 Volvo S40 (433.92 MHz, on-off keying) — **decode and display
//! only**.
//!
//! **KAT's own decoder**, worked out from one recording of one remote
//! (`IMPORTS/VOLVO/Volvo S40 1999 unlock.sub`, AM650, two presses of one button). No reference reads
//! the signal; the name is the recording's. The air format is [Chrysler V0](super::chrysler_v0)'s,
//! with 64-bit frames instead of 80, and the frame follows the rule KAT's Chrysler V0 reads every
//! frame of the Chrysler PT Cruiser 2006 fob in `IMPORTS/chrysler` by ([`super::chrysler_v0::unmask`];
//! `docs/chrysler_v0.md`), which the references' C does not use.
//!
//! **Air format**: pulse-width coding, 4 ms a bit: carrier 300 µs and pause 3.7 ms is a 0, carrier
//! 600 µs and pause 3.4 ms a 1. A press is 24 preamble pairs (300 µs, 3.7 ms), a short carrier and a
//! 15.9 ms pause, the 64 bits, a short carrier and a 15.9 ms pause, and the 64 bits again.
//!
//! **The rule** (each byte sent LSB first): the remote holds seven bytes, `n` and `p1..p6`. `n` is
//! the low byte of a counter that goes down by one every press. The frame is a sum byte, the sum
//! of the seven bytes (mod 256), then `n`, `p1 ^ n`, … `p6 ^ n`. When the sum is odd these 56 bits go
//! out backwards, last bit first. In the recording:
//!
//! | Press | Sum | `n` | `p1..p6` | Frame |
//! |---|---|---|---|---|
//! | 1 | 180 (even) | 236 | `0A 8E CB B8 20 8D` | `2D376746E42A3386` |
//! | 2 | 179 (odd) | 235 | `0A 8E CB B8 20 8D` | `CD66CB532065E1EB` |
//!
//! **What one recording cannot tell**: which bytes carry the button (only one button was
//! recorded), whether `p6` is the counter's high byte as in the PT Cruiser's frame (it would change
//! every 256 presses), and whether `p1..p4` are this remote's ID or the same in every such remote.
//! In the PT Cruiser's 80-bit frame `p1..p4` are the fob's fixed code, `p5` the button (`20` Lock,
//! `10` Unlock) and `p6` the counter's high byte; here `p5` is `20` in a recording named unlock.
//!
//! **Gate**: 16 or more preamble pairs, a short carrier and a pause of 12 to 20 ms, exactly 64 bits
//! (carrier 150–450 µs a 0, 451–750 µs a 1; pause 3.0–4.1 ms), a short carrier and a pause over
//! 8 ms, the sum, and `p1..p4` neither `00000000` nor `FFFFFFFF`. A press is read once: its second
//! copy has no preamble. An 80-bit Chrysler V0 frame is not read (cut at 64 bits, it passes the
//! sum only by chance: its own sum runs over two bytes more).
//!
//! **Fields**: `data` = the 64 bits as received, `counter` = `n`, `serial` = `p1..p4`, `extra` =
//! `p5 << 8 | p6`, `crc_valid` = the sum; no button. No encoder and no key file.

use super::info::prelude::*;
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "Rolling Code",
    make: "Volvo",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "Volvo S40";
/// Carrier of a 0 bit and of a preamble pulse; of a 1 bit (Chrysler V0's `te_short` and
/// `te_one_short`). The recording: 326 and 624 µs on average.
const TE_SHORT: u32 = 300;
const TE_ONE: u32 = 600;
const TE_DELTA: u32 = 150;
/// Pause behind a bit's carrier: 3.4 ms behind a 1, 3.7 ms behind a 0 (Chrysler V0's `te_long_a`
/// / `te_long_b`, ± 400 µs).
const PAUSE_MIN: u32 = 3_000;
const PAUSE_MAX: u32 = 4_100;
/// The pause between the preamble and the frame (15.9 ms in the recording; Chrysler V0's
/// `frame_gap` is 15.6 ms).
const START_GAP_MIN: u32 = 12_000;
const START_GAP_MAX: u32 = 20_000;
/// The pause that closes a frame (Chrysler V0's `te_gap`).
const END_GAP_MIN: u32 = 8_000;
/// Preamble pairs wanted (the remote sends 24; Chrysler V0's decoder wants more than 15).
const PREAMBLE_MIN: usize = 16;
const FRAME_BITS: usize = 64;

fn is_zero(d: u32) -> bool {
    (TE_SHORT - TE_DELTA..=TE_SHORT + TE_DELTA).contains(&d)
}
fn is_one(d: u32) -> bool {
    (TE_SHORT + TE_DELTA + 1..=TE_ONE + TE_DELTA).contains(&d)
}
fn is_pause(d: u32) -> bool {
    (PAUSE_MIN..=PAUSE_MAX).contains(&d)
}

/// The decode of 64 frame bits that pass the sum and the ID gate (the rule: Chrysler V0's
/// [`super::chrysler_v0::unmask`]).
fn signal(frame: u64) -> Option<DecodedSignal> {
    let Some((n, p, true)) = super::chrysler_v0::unmask(&frame.to_be_bytes()) else { return None };
    let id = u32::from_be_bytes([p[0], p[1], p[2], p[3]]);
    (id != 0 && id != u32::MAX).then(|| DecodedSignal {
        serial: Some(id),
        button: None,
        counter: Some(u16::from(n)),
        crc_valid: true,
        data: frame,
        data_count_bit: FRAME_BITS,
        encoder_capable: false,
        extra: Some(u64::from(p[4]) << 8 | u64::from(p[5])),
        protocol_display_name: None,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    /// Waiting for a short carrier: a preamble pulse.
    Idle,
    /// A short carrier is in; waiting for its pause, or for the gap before the frame.
    PreamblePause,
    /// Waiting for the next preamble carrier.
    PreambleCarrier,
    /// Waiting for a data bit's carrier, or the short carrier that closes the frame.
    BitCarrier,
    /// Waiting for the pause behind a carrier (true: a 1's carrier).
    BitPause(bool),
}

/// Volvo S40 decoder: a state machine over the pulses, one frame per preamble.
pub struct VolvoS40Decoder {
    step: Step,
    pairs: usize,
    frame: u64,
    count: usize,
}

impl VolvoS40Decoder {
    pub fn new() -> Self {
        Self { step: Step::Idle, pairs: 0, frame: 0, count: 0 }
    }

    /// A pulse that fits nowhere: start over, taking a short carrier as a preamble pulse.
    fn restart(&mut self, level: bool, duration: u32) {
        self.pairs = 0;
        self.step = if level && is_zero(duration) { Step::PreamblePause } else { Step::Idle };
    }
}

impl Default for VolvoS40Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for VolvoS40Decoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        // Chrysler V0's: the short carrier and a 1's pause.
        ProtocolTiming { te_short: TE_SHORT, te_long: 3_400, te_delta: TE_DELTA, min_count_bit: FRAME_BITS }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        *self = Self::new();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Idle => self.restart(level, duration),
            Step::PreamblePause => {
                if !level && is_pause(duration) {
                    self.pairs += 1;
                    self.step = Step::PreambleCarrier;
                } else if !level && self.pairs >= PREAMBLE_MIN && (START_GAP_MIN..=START_GAP_MAX).contains(&duration) {
                    (self.step, self.frame, self.count) = (Step::BitCarrier, 0, 0);
                } else {
                    self.restart(level, duration);
                }
            }
            Step::PreambleCarrier => {
                if level && is_zero(duration) {
                    self.step = Step::PreamblePause;
                } else {
                    self.restart(level, duration);
                }
            }
            Step::BitCarrier => {
                if level && is_zero(duration) {
                    self.step = Step::BitPause(false);
                } else if level && is_one(duration) {
                    self.step = Step::BitPause(true);
                } else {
                    self.restart(level, duration);
                }
            }
            Step::BitPause(one) => {
                if !level && is_pause(duration) && self.count < FRAME_BITS {
                    self.frame = self.frame << 1 | u64::from(one);
                    self.count += 1;
                    self.step = Step::BitCarrier;
                } else if !level && duration > END_GAP_MIN && self.count == FRAME_BITS && !one {
                    // The short carrier and the pause that close the frame.
                    self.step = Step::Idle;
                    return signal(self.frame);
                } else {
                    // A 65th bit (a longer frame), a 1's carrier before the gap, a frame cut short.
                    self.restart(level, duration);
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
}

#[cfg(test)]
mod tests {
    use super::super::f_common::test_support::*;
    use super::*;

    /// The recording's two presses.
    const PRESS_1: u64 = 0x2D37_6746_E42A_3386;
    const PRESS_2: u64 = 0xCD66_CB53_2065_E1EB;
    const PLAIN: [u8; 6] = [0x0A, 0x8E, 0xCB, 0xB8, 0x20, 0x8D];

    /// The frame the rule makes of `n` and `plain`, as received.
    fn frame(n: u8, plain: &[u8; 6]) -> u64 {
        u64::from_be_bytes(super::super::chrysler_v0::mask(n, plain).try_into().unwrap())
    }

    /// One copy's 64 bits as pulses (each bit a carrier and its pause), without what comes after.
    fn bits(frame: u64, count: usize) -> Vec<LevelDuration> {
        let mut out = Vec::new();
        for i in (0..count).rev() {
            let one = frame >> i & 1 == 1;
            out.push(LevelDuration::new(true, if one { TE_ONE } else { TE_SHORT }));
            out.push(LevelDuration::new(false, if one { 3_400 } else { 3_700 }));
        }
        out
    }

    /// A press as the remote sends it: `pairs` preamble pairs, the short carrier and the 15.9 ms
    /// pause, the bits, the short carrier and the pause, and the bits again.
    fn press_with(frame: u64, pairs: usize, gap: u32, count: usize) -> Vec<LevelDuration> {
        let mut wire = vec![LevelDuration::new(false, 50_000)];
        for _ in 0..pairs {
            wire.push(LevelDuration::new(true, TE_SHORT));
            wire.push(LevelDuration::new(false, 3_700));
        }
        wire.push(LevelDuration::new(true, TE_SHORT));
        wire.push(LevelDuration::new(false, gap));
        wire.extend(bits(frame, count));
        wire.push(LevelDuration::new(true, TE_SHORT));
        wire.push(LevelDuration::new(false, 15_900));
        wire.extend(bits(frame, count));
        wire.push(LevelDuration::new(true, TE_SHORT));
        wire.push(LevelDuration::new(false, 100_000));
        wire
    }

    fn press(frame: u64) -> Vec<LevelDuration> {
        press_with(frame, 24, 15_900, FRAME_BITS)
    }

    /// The rule rebuilds both presses of the recording from `n` and the six plaintext bytes, and
    /// reads them back.
    #[test]
    fn the_recordings_frames() {
        assert_eq!(frame(236, &PLAIN), PRESS_1, "sum 180 (even): sent forwards");
        assert_eq!(frame(235, &PLAIN), PRESS_2, "sum 179 (odd): sent backwards");
        // The next three presses of the same button by the rule (docs/volvo_s40.md, "What would settle it").
        assert_eq!(frame(234, &PLAIN), 0x4D57_0726_844A_53E6);
        assert_eq!(frame(233, &PLAIN), 0x8D64_C951_2267_E3E9);
        assert_eq!(frame(232, &PLAIN), 0x0D17_4766_C40A_13A6);
        for (data, n) in [(PRESS_1, 236u16), (PRESS_2, 235)] {
            let d = signal(data).expect("the sum holds");
            assert_eq!((d.serial, d.button, d.counter, d.extra), (Some(0x0A8E_CBB8), None, Some(n), Some(0x208D)));
            assert!(d.crc_valid && !d.encoder_capable);
            assert_eq!((d.data, d.data_count_bit), (data, 64));
        }
        // Every frame of the rule reads back, sent either way round.
        for n in [0u8, 1, 0x7F, 0x80, 0xFE, 0xFF] {
            for plain in [PLAIN, [1, 2, 3, 4, 5, 6], [0xFF, 0, 0x55, 0xAA, 0x10, 0x3B]] {
                let d = signal(frame(n, &plain)).unwrap();
                assert_eq!(d.counter, Some(u16::from(n)));
                assert_eq!(d.serial, Some(u32::from_be_bytes([plain[0], plain[1], plain[2], plain[3]])));
                assert_eq!(d.extra, Some(u64::from(plain[4]) << 8 | u64::from(plain[5])));
            }
        }
    }

    /// A press is read once, from the copy behind the preamble, with or without resets.
    #[test]
    fn decodes_a_press() {
        for data in [PRESS_1, PRESS_2] {
            let got = feed_all(&mut VolvoS40Decoder::new(), &press(data));
            assert_eq!(got.len(), 1, "{got:?}");
            assert_eq!(got[0], signal(data).unwrap());
            assert_eq!(feed_resetting(&mut VolvoS40Decoder::new(), &press(data)), got);
        }
        // Two presses in a row.
        let wire = [press(PRESS_1), press(PRESS_2)].concat();
        let got = feed_all(&mut VolvoS40Decoder::new(), &wire);
        assert_eq!(got.iter().map(|d| d.counter).collect::<Vec<_>>(), [Some(236), Some(235)]);
        assert!(VolvoS40Decoder::new().encode(&got[0], 1).is_none());
    }

    #[test]
    fn the_gate() {
        let read = |wire: &[LevelDuration]| feed_all(&mut VolvoS40Decoder::new(), wire).len();
        assert_eq!(read(&press(PRESS_1)), 1);
        // the sum: any one bit changed
        for bit in [0, 9, 31, 55, 56, 63] {
            assert_eq!(read(&press(PRESS_1 ^ 1 << bit)), 0, "bit {bit}");
        }
        // the preamble and the gap behind it
        assert_eq!(read(&press_with(PRESS_1, 16, 15_900, 64)), 1);
        assert_eq!(read(&press_with(PRESS_1, 15, 15_900, 64)), 0, "15 preamble pairs");
        assert_eq!(read(&press_with(PRESS_1, 24, 11_000, 64)), 0, "an 11 ms gap");
        assert_eq!(read(&press_with(PRESS_1, 24, 21_000, 64)), 0, "a 21 ms gap");
        // the length: 63 bits, and 64 bits that are the start of a longer frame
        assert_eq!(read(&press_with(PRESS_1 >> 1, 24, 15_900, 63)), 0, "63 bits");
        let mut long = press_with(PRESS_1, 24, 15_900, 64);
        let at = long.iter().position(|p| p.duration_us == 15_900).unwrap() + 1 + 128;
        long.splice(at..at, bits(0, 16));
        assert_eq!(read(&long), 0, "80 bits");
        // the closing short carrier: a 1's carrier there, or none (the gap straight after a pause)
        let mut wire = press(PRESS_1);
        let close = wire.iter().position(|p| p.duration_us == 15_900).unwrap() + 1 + 128;
        wire[close].duration_us = TE_ONE;
        assert_eq!(read(&wire), 0, "a long closing carrier");
        let mut wire = press(PRESS_1);
        wire.truncate(close);
        wire.last_mut().unwrap().duration_us = 15_900;
        assert_eq!(read(&wire), 0, "no closing carrier");
        // a pulse of another length inside the frame
        let mut wire = press(PRESS_1);
        wire[close - 40].duration_us = 1_000;
        assert_eq!(read(&wire), 0);
        // the ID: 00000000 and FFFFFFFF are not read, whatever the sum
        assert_eq!(read(&press(frame(7, &[0, 0, 0, 0, 0x20, 0x8D]))), 0);
        assert_eq!(read(&press(frame(7, &[0xFF; 6]))), 0);
        assert_eq!(read(&press(frame(7, &[0, 0, 0, 1, 0x20, 0x8D]))), 1);
        // a run of preamble pulses (64 zero bits is no frame either: its ID would be 0)
        assert_eq!(read(&press(0)), 0);
    }

    /// Chrysler V0 frames (80 bits) are not read: its uploads, and the PT Cruiser recordings.
    #[test]
    fn chrysler_v0_frames_are_not_read() {
        let chrysler = crate::protocols::ProtocolRegistry::new();
        let dec = chrysler.get("Chrysler V0").unwrap();
        let mut sig = DecodedSignal::new(0x5BF1_4E2D_7432_F92D, 80);
        sig.extra = Some(0x7432);
        let upload = dec.encode(&sig, 1).unwrap();
        assert!(feed_all(&mut VolvoS40Decoder::new(), &upload).is_empty());
        for rel in ["Chrysler PT Cruiser 2006 open.sub", "Chrysler PT Cruiser 2006 close.sub"] {
            let path = std::path::Path::new("IMPORTS/chrysler").join(rel);
            if !path.is_file() {
                eprintln!("Skip: IMPORTS not found (run from crate root)");
                return;
            }
            let (_, wire) = sub_pulses(&path);
            assert!(feed_resetting(&mut VolvoS40Decoder::new(), &wire).is_empty(), "{rel}");
        }
    }

    /// The recording: two presses, the counter one lower in the second, the same six bytes.
    #[test]
    fn imports_recording() {
        let path = std::path::Path::new("IMPORTS/VOLVO/Volvo S40 1999 unlock.sub");
        if !path.is_file() {
            eprintln!("Skip: IMPORTS not found (run from crate root)");
            return;
        }
        let (_, wire) = sub_pulses(path);
        let got = feed_resetting(&mut VolvoS40Decoder::new(), &wire);
        assert_eq!(got.iter().map(|d| d.data).collect::<Vec<_>>(), [PRESS_1, PRESS_2]);
        assert!(got.iter().all(|d| d.serial == Some(0x0A8E_CBB8) && d.extra == Some(0x208D)));
    }
}
