//! rtl_433's receive chain for KAT's rtl_433 ports (all receive only): Alps FWB1U545, Astrostart
//! 2000, Audiovox PRO-OE3B, Chrysler car remote, Code Alarm FRDPC2002, Compustar 1WG3R, Siemens
//! 5WY72XX, Continental KR5V2X, Honda KR5V2X/1X, MIC 6SC2, Opel Mokka, Elero, Somfy
//! io-homecontrol, RojaFlex, Microchip HCS361 / HCS362 and Markisol.
//!
//! rtl_433 (`REFERENCES/rtl_433`, GPL-2.0) decodes in three stages: the pulse detector
//! (`src/pulse_detect.c`) cuts the signal into *packages* of (pulse, gap) pairs, a pulse slicer
//! (`src/pulse_slicer.c`, one per modulation) turns a package into a `bitbuffer_t` of rows and
//! calls the device's `decode_fn` at the end of every *message* (`account_event`, which then clears
//! the buffer), and the decoder checks the rows and outputs its events. This module holds exact
//! ports of the parts the devices use, run over KAT's level/duration stream:
//!
//! - [BitBuffer] (`bitbuffer.rs`): `bitbuffer_t` (50 rows × 128 bytes, row spill and overflow
//!   included) and the `bitbuffer_*` helpers the decoders call (`src/bitbuffer.c`);
//! - `bit_util.rs`: the helpers of `src/bit_util.c` ([reverse8], [xor_bytes], [crc16],
//!   [crc16lsb], [ibm_whitening], [extract_bytes_uart_8n1]);
//! - `pulse_detect.rs`: the pulse detector's pairs and package ends;
//! - `pulse_slicer.rs` ([Modulation]): `pulse_slicer_pwm`, `pulse_slicer_ppm`,
//!   `pulse_slicer_manchester_zerobit` (OOK and FSK), `pulse_slicer_piwm_dc` and `pulse_slicer_pcm`
//!   (OOK and FSK);
//! - [Receiver]: one `r_device` ([Device]) — detector, slicer and bit buffer — running the
//!   device's `decode_fn` ([DecodeFn]);
//! - `keeloq_chip.rs`: a Microchip KeeLoq chip's frame through KAT's KeeLoq identification.
//!
//! **Units.** rtl_433 works in samples; KAT's durations are µs. The slicers run at one sample per
//! µs, i.e. with the integer limits rtl_433 computes at 1 MS/s (`s_short = short_width *
//! samples_per_us`; oracle runs feed rtl_433 the same pulses with `-s 1M`, where its thresholds are
//! identical). At other rates rtl_433 truncates the limits to whole samples (at 250 kS/s 445 µs →
//! 111 samples), which moves them by less than one sample.
//!
//! **Pairs.** A HIGH is a pulse (consecutive HIGHs add up: one pulse on air) and the LOW after it
//! its gap; a LOW before any pulse (the silence leading a burst) is not part of a package and is
//! skipped. KAT feeds whole level durations, so a gap is normally complete when its LOW arrives. A
//! LOW right after a LOW (never from KAT's receiver, seen in a few `.sub` files) is read one of two
//! ways ([Gaps]): `AtOnce` slices each pair when its LOW arrives and lets a further LOW only extend
//! the gap for the end-of-message and package decisions; `Merged` adds the LOWs up to one gap (as
//! on air) and slices the pair when the gap is complete (at the next HIGH) or as soon as it ends the
//! message or the package. The PCM slicer needs the whole package (it measures the bit period on
//! it), so its receiver collects the package and slices it when it ends.
//!
//! **Packages** ([Ends]). An OOK package ends at a gap longer than 10 ms and 10 × its longest
//! pulse, at a gap longer than 100 ms, or at its 1200th pulse (`PD_MIN_GAP_MS`, `PD_MAX_GAP_RATIO`,
//! `PD_MAX_GAP_MS`, `PD_MAX_PULSES`). Every slicer ends its message at the package's last pulse,
//! and the next package starts with an empty bit buffer. An FSK package is a carrier period. On
//! KAT's FM path a stopped carrier and the lower tone are both LOW: a zero-bit FSK device reads a
//! burst's FM pairs as one package (its messages end at its reset limit; the end of the carrier
//! arrives as the last lower tone merged with the idle LOW, ≥ 80 ms); an FSK PCM device ends the
//! package at a LOW its coding cannot contain and reads the zeros that LOW swallowed from the frame
//! ([FskFrame]). A device whose reset limit ends every message where a package would end can do
//! without package ends (`Ends::None`), except for the 1200-pulse cut.
//!
//! **Reports.** KAT reports at most one decode per pulse, on the pulse where rtl_433 would output
//! (mostly the gap that ends the message). rtl_433 can output several events per message (one per
//! row: Chrysler's two packets, Compustar's batches): row-wise decoders ([DecodeFn::ROW_WISE])
//! report each row's event as soon as the row is final (the next row has started, so rtl_433's
//! decision for it cannot change any more), in rtl_433's order and with its per-message state
//! (Compustar's duplicate suppression). The PCM slicer, which slices a whole package at once,
//! reports the package's first event.
//!
//! **Registry resets.** rtl_433 runs every decoder over every whole package; KAT's registry resets
//! all decoders whenever one reports. A receiver keeps its message across the reset that follows
//! its own report (rtl_433 only clears the bit buffer after each message), so a message keeps its
//! rows and per-message state across its own reports; a caller that stops at a report keeps that
//! state until the message ends. After any other reset the receiver starts over; with
//! [Device::resync] it first waits for silence (see `pulse_detect::Detector::new`).
//!
//! **Not modelled** (rtl_433's demodulator, which KAT's own receive chain replaces): the pulse
//! detector's sample-level rules (a pulse or gap under 10 samples, the adaptive OOK level, FSK
//! frequency tracking), its FSK buffer shift (past 1200 FSK pulses rtl_433 slices only the last
//! 600–1199 of the carrier period), and an FSK package that starts in the lower tone
//! (`pulse[0] = 0`), whose leading tone KAT's FM path merges into the idle LOW.
//!
//! **Stream models.** Each port keeps the reading of KAT's stream it was verified with against
//! rtl_433 (oracle runs on the same pulses, the rtl_433_tests recordings). The choices differ only
//! on input KAT's receiver does not produce or for callers that keep feeding after a report (a LOW
//! after a LOW, the 1200th pulse of a package, a reset that is not the registry's, a 0 µs HIGH):
//!
//! | ports | slicer | [Gaps] | [Ends] | resync |
//! |---|---|---|---|---|
//! | Siemens 5WY72XX, Code Alarm FRDPC2002 | OOK zero-bit | `AtOnce` | `Ook` | no |
//! | Astrostart 2000 | OOK PPM | `AtOnce` | `Ook` | no |
//! | Chrysler car remote, Compustar 1WG3R, Audiovox PRO-OE3B | OOK PWM | `AtOnce` | `Ook` | no |
//! | Alps FWB1U545 | FSK zero-bit | `AtOnce` | `None` | no |
//! | MIC 6SC2 | OOK zero-bit | `Merged` | `None` | no |
//! | Continental KR5V2X, Honda KR5V2X/1X, Opel Mokka | FSK zero-bit | `Merged` | `None` | no |
//! | Microchip HCS361 / HCS362, Markisol | OOK PWM, PIWM_DC, PCM | `Merged` | `Ook` | yes |
//! | Elero, Somfy io-homecontrol, RojaFlex | FSK PCM | `Merged` | `Fsk` | yes |

mod bit_util;
mod bitbuffer;
mod keeloq_chip;
mod pulse_detect;
mod pulse_slicer;
#[cfg(test)]
pub mod test_util;
#[cfg(test)]
mod harness;

pub use bit_util::{crc16, crc16lsb, extract_bytes_uart_8n1, ibm_whitening, reverse8, xor_bytes};
pub use bitbuffer::BitBuffer;
pub use keeloq_chip::{is_keeloq_chip, keeloq_chip_signal, redecode_keeloq_chip};
pub use pulse_detect::{Ends, Gaps};
pub use pulse_slicer::{FskFrame, Modulation};

use super::DecodedSignal;
use pulse_detect::{Detector, Event};
use pulse_slicer::Slicer;

/// An rtl_433 `r_device` as a port runs it: its modulation and timing fields (µs, 0 = unset;
/// rtl_433's `float`s, whole µs for every port, so the slicers' integer limits are exact) and how
/// KAT's stream is read for it (see the module doc).
#[derive(Debug, Clone, Copy)]
pub struct Device {
    pub modulation: Modulation,
    pub short_width: u32,
    pub long_width: u32,
    pub reset_limit: u32,
    pub gap_limit: u32,
    pub sync_width: u32,
    pub tolerance: u32,
    pub gaps: Gaps,
    pub ends: Ends,
    /// Wait for silence after a reset that is not the one following an own report (and at the
    /// start); see `pulse_detect::Detector::new` for what comes with it.
    pub resync: bool,
}

/// A device's rtl_433 `decode_fn`, run by a [Receiver] at every end of message.
pub trait DecodeFn {
    /// Report each row's event as soon as the row is final ([Self::row]) instead of decoding the
    /// whole message at its end ([Self::message]).
    const ROW_WISE: bool = false;

    /// `decode_fn` on a message's bit buffer: the event it outputs (the first one, when several).
    /// The buffer is cleared afterwards, as `account_event` does.
    fn message(&mut self, _bits: &mut BitBuffer) -> Option<DecodedSignal> {
        None
    }

    /// Row-wise decoders: `decode_fn`'s body for row `row` of the current message (rows come in
    /// order, each once).
    fn row(&mut self, _bits: &BitBuffer, _row: usize) -> Option<DecodedSignal> {
        None
    }

    /// A new message starts (`decode_fn`'s local state).
    fn message_start(&mut self) {}
}

/// A stateless `decode_fn`.
pub struct Decode(pub fn(&BitBuffer) -> Option<DecodedSignal>);

impl DecodeFn for Decode {
    fn message(&mut self, bits: &mut BitBuffer) -> Option<DecodedSignal> {
        (self.0)(bits)
    }
}

/// rtl_433's receive chain for one `r_device`: the pulse detector's pairs and packages, the
/// device's slicer and the bit buffer, fed with KAT's level/duration stream. The `decode_fn` is
/// passed to [Self::feed] and [Self::reset].
#[derive(Debug, Clone)]
pub struct Receiver {
    detector: Detector,
    slicer: Slicer,
    bits: BitBuffer,
    /// Row-wise decoders: rows of the current message already reported on.
    rows_done: usize,
}

impl Receiver {
    pub fn new(device: &Device) -> Self {
        let slicer = Slicer::new(device);
        let detector = Detector::new(device.gaps, device.ends, device.resync, slicer.early());
        let mut rx = Self { detector, slicer, bits: BitBuffer::new(), rows_done: 0 };
        rx.new_package();
        rx
    }

    /// `ProtocolDecoder::reset`: start over, unless this receiver reported (see the module doc);
    /// `decode`'s per-message state is reset along.
    pub fn reset<D: DecodeFn>(&mut self, decode: &mut D) {
        if self.detector.reset() {
            self.new_package();
            decode.message_start();
        }
    }

    /// The decoder reported (through this receiver or another of its `r_device`s): the
    /// registry's reset that follows keeps the message.
    pub fn own_report(&mut self) {
        self.detector.own_report();
    }

    /// `ProtocolDecoder::feed`. The detector's and the slicer's steps are inlined into it, so the
    /// layers fold into one loop per decoder (as fast as the hand-merged loops they replace).
    #[inline(always)]
    pub fn feed<D: DecodeFn>(&mut self, decode: &mut D, level: bool, duration: u32) -> Option<DecodedSignal> {
        let [first, second] = self.detector.feed(level, duration);
        let mut out = None;
        if let Some(ev) = first {
            self.event(decode, ev, &mut out);
        }
        if let Some(ev) = second {
            self.event(decode, ev, &mut out);
        }
        if out.is_some() {
            self.detector.own_report();
        }
        out
    }

    /// One detector event; a message's event goes to `out` unless it holds one already (the
    /// decodes are built only at an end of message, so nothing is moved on the way there).
    #[inline(always)]
    fn event<D: DecodeFn>(&mut self, decode: &mut D, ev: Event, out: &mut Option<DecodedSignal>) {
        if let Slicer::Pcm(pcm) = &mut self.slicer {
            if pcm.event(ev) {
                pcm.slice(&mut self.bits, decode, out);
            }
            return;
        }
        match ev {
            Event::Pulse(p) => {
                if self.slicer.pulse(&mut self.bits, p) {
                    self.end_message(decode, out);
                }
            }
            Event::Gap { gap, last } => {
                if self.slicer.gap(&mut self.bits, gap, last) {
                    self.end_message(decode, out);
                } else if D::ROW_WISE {
                    self.final_rows(decode, out);
                }
                if last {
                    self.new_package();
                }
            }
            Event::Longer { gap, last } => {
                if (last || self.slicer.ends_message(gap)) && self.slicer.pending(&self.bits) {
                    self.end_message(decode, out);
                }
                if last {
                    self.new_package();
                }
            }
            Event::End => self.new_package(),
        }
    }

    /// A new package: the slicer starts with an empty bit buffer.
    fn new_package(&mut self) {
        self.rows_done = 0;
        self.slicer.new_package(&mut self.bits);
    }

    /// `account_event`: run `decode_fn` on the message and clear the buffer (row-wise: on the
    /// rows not reported yet); the first event goes to `out`.
    #[inline(never)]
    fn end_message<D: DecodeFn>(&mut self, decode: &mut D, out: &mut Option<DecodedSignal>) {
        if D::ROW_WISE {
            for row in self.rows_done..self.bits.num_rows() {
                first(out, decode.row(&self.bits, row));
            }
        } else {
            first(out, decode.message(&mut self.bits));
        }
        self.bits.clear();
        self.rows_done = 0;
        decode.message_start();
        self.slicer.after_message(&mut self.bits);
    }

    /// Row-wise decoders: report the rows that became final (all but the last row).
    fn final_rows<D: DecodeFn>(&mut self, decode: &mut D, out: &mut Option<DecodedSignal>) {
        let done = self.bits.num_rows().saturating_sub(1);
        while self.rows_done < done {
            first(out, decode.row(&self.bits, self.rows_done));
            self.rows_done += 1;
        }
    }
}

/// Keep the first event: `out` takes `ev` unless it holds one already.
fn first(out: &mut Option<DecodedSignal>, ev: Option<DecodedSignal>) {
    if out.is_none() {
        *out = ev;
    }
}

/// A receive-only decode of a frame of `bit_count` bits (MSB first in `bytes`), packed the KAT way:
/// the first 64 bits in `data`, the rest right-aligned in `extra` (frames over 128 bits: their last
/// 64 bits).
pub fn frame_signal(bytes: &[u8], bit_count: usize) -> DecodedSignal {
    let bit = |i: usize| u64::from(bitbuffer::bitrow_get_bit(bytes, i));
    let data = (0..bit_count.min(64)).fold(0u64, |acc, i| (acc << 1) | bit(i));
    let extra = (bit_count > 64).then(|| {
        let from = bit_count.saturating_sub(64).max(64);
        (from..bit_count).fold(0u64, |acc, i| (acc << 1) | bit(i))
    });
    DecodedSignal { extra, ..DecodedSignal::new(data, bit_count) }
}

/// Native button label tables built once from a device's `button_str` logic (rtl_433 joins several
/// pressed buttons with "; "), so `ProtocolInfo::buttons` can hand out `&'static str`s.
pub fn button_table(cell: &'static std::sync::OnceLock<Vec<&'static str>>, label: fn(u8) -> String) -> &'static [&'static str] {
    cell.get_or_init(|| (0..=255u8).map(|c| &*Box::leak(label(c).into_boxed_str())).collect())
}

/// rtl_433's `"; "`-joined list of the names whose test passes, or `none` when none does.
pub fn join_names<'a>(names: impl Iterator<Item = &'a str>, none: &str) -> String {
    let v: Vec<&str> = names.collect();
    if v.is_empty() {
        none.to_string()
    } else {
        v.join("; ")
    }
}

#[cfg(test)]
mod tests {
    use super::test_util::{bits_of_bytes, manchester_zerobit, wire, with_idle};
    use super::*;
    use crate::radio::demodulator::LevelDuration;

    /// Every message's rows as (bits, bytes).
    struct Rows(Vec<Vec<(usize, Vec<u8>)>>);

    impl DecodeFn for Rows {
        fn message(&mut self, bits: &mut BitBuffer) -> Option<DecodedSignal> {
            self.0.push((0..bits.num_rows()).map(|r| (bits.bits_per_row(r), bits.row(r)[..bits.bits_per_row(r).div_ceil(8)].to_vec())).collect());
            None
        }
    }

    fn zerobit(short_width: u32, reset_limit: u32, tolerance: u32, gaps: Gaps) -> Device {
        Device { modulation: Modulation::OokManchesterZerobit, short_width, long_width: 0, reset_limit, gap_limit: 0, sync_width: 0, tolerance, gaps, ends: Ends::None, resync: false }
    }

    /// The messages a receiver hands its `decode_fn` for a stream.
    fn messages(device: &Device, stream: &[LevelDuration]) -> Vec<Vec<(usize, Vec<u8>)>> {
        let mut rx = Receiver::new(device);
        let mut rows = Rows(Vec::new());
        for p in stream {
            rx.feed(&mut rows, p.level, p.duration_us);
        }
        rows.0
    }

    #[test]
    fn zerobit_reads_manchester_back() {
        let bytes = [0x55, 0x54, 0xA7, 0x0F, 0x3C, 0x81];
        let bits = bits_of_bytes(&bytes, 48);
        let one = wire(&manchester_zerobit(&bits, 250, 0));
        let d = zerobit(250, 10_000, 0, Gaps::Merged);
        assert_eq!(messages(&d, &with_idle(&one, 20_000)), vec![vec![(48, bytes.to_vec())]]);
        // Two frames 5 ms apart are one message (5 ms ≤ the reset limit) …
        let mut two = one.clone();
        two.push(LevelDuration::new(false, 5_000));
        two.extend(one);
        let got = messages(&d, &with_idle(&two, 20_000));
        assert_eq!(got.len(), 1);
        assert_eq!(got[0][0].0, 96, "48 + 48: the rising edge after the 5 ms gap is the second frame's first 0");
        // … and with a shorter reset limit, two messages.
        assert_eq!(messages(&zerobit(250, 4_000, 0, Gaps::Merged), &with_idle(&two, 20_000)), vec![vec![(48, bytes.to_vec())]; 2]);
    }

    #[test]
    fn merged_gaps_read_split_levels_as_one() {
        let bits = bits_of_bytes(&[0x2D, 0x5A], 16);
        let stream = with_idle(&wire(&manchester_zerobit(&bits, 100, 0)), 5_000);
        // Every duration split in two same-level halves: one pulse on air.
        let split: Vec<LevelDuration> = stream
            .iter()
            .flat_map(|p| {
                let a = p.duration_us / 2;
                [LevelDuration::new(p.level, a), LevelDuration::new(p.level, p.duration_us - a)]
            })
            .collect();
        let merged = zerobit(100, 1_500, 0, Gaps::Merged);
        assert_eq!(messages(&merged, &stream), vec![vec![(16, vec![0x2D, 0x5A])]]);
        assert_eq!(messages(&merged, &split), messages(&merged, &stream));
        // `AtOnce` slices each pair at its first LOW: a split LOW is read as its first half.
        let at_once = zerobit(100, 1_500, 0, Gaps::AtOnce);
        assert_eq!(messages(&at_once, &stream), messages(&merged, &stream));
        assert_ne!(messages(&at_once, &split), messages(&merged, &split));
    }

    #[test]
    fn zerobit_tolerance_splits_rows() {
        // With a tolerance, an out-of-range pulse starts a new row (rtl_433's [1]10 rule adds a 1
        // first for a long one); the message still ends only at the reset limit.
        let half = wire(&manchester_zerobit(&bits_of_bytes(&[0x2A], 8), 100, 0));
        let mut stream = half.clone();
        stream.push(LevelDuration::new(true, 400));
        stream.extend(half);
        let got = messages(&zerobit(100, 1_500, 30, Gaps::Merged), &with_idle(&stream, 5_000));
        assert_eq!(got.len(), 1);
        assert!(got[0].len() >= 2, "{got:?}");
    }

    #[test]
    fn frame_signal_packs_long_frames() {
        let bytes = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, 0xE0];
        let d = frame_signal(&bytes, 67);
        assert_eq!((d.data, d.extra, d.data_count_bit, d.encoder_capable), (0x1234_5678_9ABC_DEF0, Some(0b111), 67, false));
        let long: Vec<u8> = (0..32).collect();
        let d = frame_signal(&long, 256);
        assert_eq!((d.data, d.extra), (0x0001_0203_0405_0607, Some(0x1819_1A1B_1C1D_1E1F)));
        assert_eq!(frame_signal(&bytes, 40).extra, None);
        let bytes: Vec<u8> = (1..=14).collect();
        let d = frame_signal(&bytes, 112);
        assert_eq!((d.data, d.extra), (0x0102_0304_0506_0708, Some(0x090A_0B0C_0D0E)));
        assert_eq!((frame_signal(&bytes, 12).data, frame_signal(&[0xFF; 11], 82).extra), (0x010, Some(0x3FFFF)));
    }

    #[test]
    fn button_names_join_like_rtl_433() {
        assert_eq!(join_names(["Lock", "Unlock"].into_iter(), "?"), "Lock; Unlock");
        assert_eq!(join_names(std::iter::empty(), "?"), "?");
        static NAMES: std::sync::OnceLock<Vec<&'static str>> = std::sync::OnceLock::new();
        let t = button_table(&NAMES, |c| format!("b{c}"));
        assert_eq!((t.len(), t[7]), (256, "b7"));
    }
}
