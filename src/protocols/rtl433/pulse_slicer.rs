//! The slicers of rtl_433's `src/pulse_slicer.c` the ports use, one (pulse, gap) pair at a time
//! (PCM: one package at a time), at one sample per µs. Each turns the pairs of a package into the
//! rows of a [BitBuffer] and tells its receiver where a message ends; the receiver
//! (`rtl433::Receiver`) runs the device's `decode_fn` there.

use super::bitbuffer::{bitrow_get_bit, BitBuffer};
use super::pulse_detect::{Ends, Event};
use super::{DecodeFn, Device};
use crate::protocols::DecodedSignal;

/// rtl_433's `modulation` of an `r_device`: which slicer reads it.
#[derive(Debug, Clone, Copy)]
pub enum Modulation {
    /// `OOK_PULSE_PWM` (`pulse_slicer_pwm`): a pulse in the short window = 1, in the long window
    /// = 0, in the sync window = sync; a shorter pulse is ignored, any other starts a new row; a
    /// gap over `gap_limit` starts a new row; end of message at a gap over `reset_limit`.
    OokPwm,
    /// `OOK_PULSE_PPM` (`pulse_slicer_ppm`): a gap in the short window = 0, in the long window = 1,
    /// in the sync window = sync; any other gap under `reset_limit` starts a new row; end of
    /// message at a gap of `reset_limit` or more.
    OokPpm,
    /// `OOK_PULSE_MANCHESTER_ZEROBIT` (`pulse_slicer_manchester_zerobit`): every message starts
    /// with a hard-coded 0 (the first rising edge); an edge more than 1.5 × `short_width` after the
    /// last data edge is a data edge (falling at the end of a pulse = 1, rising at the end of a gap
    /// = 0). With a `tolerance`, a pulse or gap outside `short - tol ..= 2 short + tol` starts a new
    /// row (adding a 1 first when the pulse was long); end of message at a gap over
    /// `reset_limit`.
    OokManchesterZerobit,
    /// `FSK_PULSE_MANCHESTER_ZEROBIT`: the same slicer on FSK pulses (upper tone = pulse).
    FskManchesterZerobit,
    /// `OOK_PULSE_PIWM_DC` (`pulse_slicer_piwm_dc`): every pulse and every gap is a symbol: within
    /// `tolerance` of `short_width` = 1, of `long_width` = 0; any other symbol under `reset_limit`
    /// ends the row; end of message at a symbol over `reset_limit`.
    OokPiwmDc,
    /// `OOK_PULSE_PCM` (`pulse_slicer_pcm`): the bit period is measured on the package (a preamble
    /// of bit-wide toggles, else every 1- and 2-bit pulse / gap), then each pulse adds its ones and
    /// each gap its zeros (at most `gap_limit` worth); RZ pulses off the timing clear the buffer; a
    /// gap between `gap_limit` and `reset_limit` starts a new row; end of message at a gap over
    /// `reset_limit`. The receiver collects the package and slices it when it ends.
    OokPcm,
    /// `FSK_PULSE_PCM`: the PCM slicer on FSK pulses, with the carrier-off reading of the
    /// package's last gap ([FskFrame]).
    FskPcm(FskFrame),
}

/// What the FSK PCM slicer needs to know about a device's frames to read the package's last gap.
///
/// When an FSK package ends with the carrier stopping (its last gap longer than
/// [Ends::Fsk]'s `carrier_off`), the zeros that end the frame are part of that LOW on KAT's FM
/// path, while rtl_433 sees the carrier stop right after them. So the last gap is read as each
/// tail of 0 … `carrier_off / long_width` zero bits (what rtl_433 gets from a transmitter that
/// stops that many bits after its last upper tone, up to the slicer's `max_zeros`). The shortest
/// tail whose decode passes its integrity check (`crc_valid`) wins; a decode without a check is
/// taken only when the device's own rule accepts it ([Self::unchecked]): otherwise a too-short
/// tail would turn a frame that fails its check into an unchecked one, or a long one pad a cut
/// frame. The package is sliced once and the tail's zeros added one by one (`decode_fn` must
/// leave the buffer as it is); the sync word ([Self::sync], ending with a 1) is what the decoder
/// searches row 0 for first and gives up without: zeros cannot complete it, so without it no tail
/// is tried. Packages too short for the decoder's smallest frame are not sliced at all (a row
/// holds at most 2 · span / bit + pairs + tail bits: the measured bit period is at most twice the
/// nominal rate).
#[derive(Debug, Clone, Copy)]
pub struct FskFrame {
    /// Which decode without an integrity check (`crc_valid` false) a tail reading may produce: the
    /// tail cannot be seen, so only what the frame itself carries can pick it (None: never).
    pub unchecked: Option<fn(&DecodedSignal) -> bool>,
    /// The sync word the decoder searches row 0 for (MSB first), and its length in bits.
    pub sync: &'static [u8],
    pub sync_bits: usize,
    /// The fewest row bits the decoder can accept (sync included).
    pub min_bits: usize,
}

/// A device's slicer and its state between pairs.
#[derive(Debug, Clone)]
pub enum Slicer {
    Pwm(Pwm),
    Ppm(Ppm),
    Zerobit(Zerobit),
    PiwmDc(PiwmDc),
    Pcm(Pcm),
}

impl Slicer {
    pub fn new(d: &Device) -> Self {
        match d.modulation {
            Modulation::OokPwm => Self::Pwm(Pwm::new(d)),
            Modulation::OokPpm => Self::Ppm(Ppm::new(d)),
            Modulation::OokManchesterZerobit | Modulation::FskManchesterZerobit => Self::Zerobit(Zerobit::new(d)),
            Modulation::OokPiwmDc => Self::PiwmDc(PiwmDc::new(d)),
            Modulation::OokPcm => Self::Pcm(Pcm::new(d, None)),
            Modulation::FskPcm(frame) => {
                let Ends::Fsk { carrier_off } = d.ends else { panic!("FSK_PULSE_PCM reads its last gap with the carrier-off end") };
                Self::Pcm(Pcm::new(d, Some((frame, carrier_off))))
            }
        }
    }

    /// The shortest gap that ends a message (µs), for a detector that hands gaps over when they do
    /// (None: the slicer takes whole packages).
    pub fn early(&self) -> Option<u32> {
        let after = |reset: i64| Some(u32::try_from(reset + 1).unwrap_or(u32::MAX));
        match self {
            Self::Pwm(s) => after(s.s_reset),
            Self::Ppm(s) => Some(u32::try_from(s.s_reset).unwrap_or(u32::MAX)),
            Self::Zerobit(s) => after(s.s_reset),
            Self::PiwmDc(s) => after(s.s_reset),
            Self::Pcm(_) => None,
        }
    }

    /// The pulse of the current pair; true at an end of message.
    #[inline(always)]
    pub fn pulse(&mut self, bits: &mut BitBuffer, pulse: u32) -> bool {
        let p = i64::from(pulse);
        match self {
            Self::Pwm(s) => {
                s.pulse(bits, p);
                false
            }
            Self::Ppm(_) | Self::Pcm(_) => false,
            Self::Zerobit(s) => {
                s.pulse = p;
                false
            }
            Self::PiwmDc(s) => s.symbol(bits, p, false),
        }
    }

    /// The gap of the current pair (`last`: the package's last); true at an end of message.
    #[inline(always)]
    pub fn gap(&mut self, bits: &mut BitBuffer, gap: u32, last: bool) -> bool {
        let g = i64::from(gap);
        match self {
            Self::Pwm(s) => s.gap(bits, g, last),
            Self::Ppm(s) => s.gap(bits, g, last),
            Self::Zerobit(s) => s.gap(bits, g, last),
            Self::PiwmDc(s) => s.symbol(bits, g, last),
            Self::Pcm(_) => false,
        }
    }

    /// Whether a gap this long ends the message (the slicer's reset limit test).
    pub fn ends_message(&self, gap: u32) -> bool {
        let g = i64::from(gap);
        match self {
            Self::Pwm(s) => g > s.s_reset,
            Self::Ppm(s) => g >= s.s_reset,
            Self::Zerobit(s) => g > s.s_reset,
            Self::PiwmDc(s) => g > s.s_reset,
            Self::Pcm(_) => false,
        }
    }

    /// Whether the buffer holds a message to account (a zero-bit buffer always holds the next
    /// message's hard-coded 0).
    pub fn pending(&self, bits: &BitBuffer) -> bool {
        match self {
            Self::Pwm(_) | Self::PiwmDc(_) => bits.num_rows() > 0,
            Self::Ppm(_) => bits.bits_per_row(0) > 0 || bits.num_rows() > 1,
            Self::Zerobit(_) => bits.num_rows() > 1 || bits.bits_per_row(0) > 1,
            Self::Pcm(_) => false,
        }
    }

    /// After `account_event`: the zero-bit slicer starts the next message with its hard-coded 0.
    pub fn after_message(&mut self, bits: &mut BitBuffer) {
        if let Self::Zerobit(s) = self {
            bits.add_bit(false);
            s.time_since_last = 0;
        }
    }

    /// A new package: the slicer starts with an empty bit buffer (zero-bit: with its hard-coded 0).
    pub fn new_package(&mut self, bits: &mut BitBuffer) {
        bits.clear();
        match self {
            Self::Zerobit(s) => {
                s.time_since_last = 0;
                bits.add_bit(false);
            }
            Self::Pcm(s) => s.clear(),
            _ => {}
        }
    }
}

/// `pulse_slicer_pwm`'s symbol windows (exclusive bounds, as the C computes them) and limits.
#[derive(Debug, Clone)]
pub struct Pwm {
    one_l: i64,
    one_u: i64,
    zero_l: i64,
    zero_u: i64,
    sync_l: i64,
    sync_u: i64,
    s_reset: i64,
    s_gap: i64,
}

impl Pwm {
    fn new(d: &Device) -> Self {
        let (s_short, s_long, s_sync, tol) = (i64::from(d.short_width), i64::from(d.long_width), i64::from(d.sync_width), i64::from(d.tolerance));
        let max = i64::from(i32::MAX);
        let (one_l, one_u, zero_l, zero_u, sync_l, sync_u) = if tol > 0 {
            let (sync_l, sync_u) = if s_sync > 0 { (s_sync - tol, s_sync + tol) } else { (0, 0) };
            (s_short - tol, s_short + tol, s_long - tol, s_long + tol, sync_l, sync_u)
        } else if s_sync <= 0 {
            // no sync, short=1, long=0
            let one_u = (s_short + s_long) / 2 + 1;
            (0, one_u, one_u - 1, max, 0, 0)
        } else if s_sync < s_short {
            // short=sync, middle=1, long=0
            let sync_u = (s_sync + s_short) / 2 + 1;
            let one_u = (s_short + s_long) / 2 + 1;
            (sync_u - 1, one_u, one_u - 1, max, 0, sync_u)
        } else if s_sync < s_long {
            // short=1, middle=sync, long=0
            let one_u = (s_short + s_sync) / 2 + 1;
            let sync_u = (s_sync + s_long) / 2 + 1;
            (0, one_u, sync_u - 1, max, one_u - 1, sync_u)
        } else {
            // short=1, middle=0, long=sync
            let one_u = (s_short + s_long) / 2 + 1;
            let zero_u = (s_long + s_sync) / 2 + 1;
            (0, one_u, one_u - 1, zero_u, zero_u - 1, max)
        };
        Self { one_l, one_u, zero_l, zero_u, sync_l, sync_u, s_reset: i64::from(d.reset_limit), s_gap: i64::from(d.gap_limit) }
    }

    #[inline]
    fn pulse(&self, bits: &mut BitBuffer, p: i64) {
        if p > self.one_l && p < self.one_u {
            bits.add_bit(true);
        } else if p > self.zero_l && p < self.zero_u {
            bits.add_bit(false);
        } else if p > self.sync_l && p < self.sync_u {
            bits.add_sync();
        } else if p <= self.one_l {
            // Ignore spurious short pulses
        } else {
            // Pulse outside specified timing
            bits.add_row();
        }
    }

    #[inline]
    fn gap(&self, bits: &mut BitBuffer, g: i64, last: bool) -> bool {
        let rows = bits.num_rows();
        if (last || g > self.s_reset) && rows > 0 {
            return true;
        }
        if self.s_gap > 0 && g > self.s_gap && rows > 0 && bits.bits_per_row(rows - 1) > 0 {
            // New packet in multipacket
            bits.add_row();
        }
        false
    }
}

/// `pulse_slicer_ppm`'s symbol windows (exclusive bounds) and reset limit.
#[derive(Debug, Clone)]
pub struct Ppm {
    zero_l: i64,
    zero_u: i64,
    one_l: i64,
    one_u: i64,
    sync_l: i64,
    sync_u: i64,
    s_reset: i64,
}

impl Ppm {
    fn new(d: &Device) -> Self {
        let (s_short, s_long, s_sync, tol) = (i64::from(d.short_width), i64::from(d.long_width), i64::from(d.sync_width), i64::from(d.tolerance));
        let (s_reset, s_gap) = (i64::from(d.reset_limit), i64::from(d.gap_limit));
        let (zero_l, zero_u, one_l, one_u, sync_l, sync_u) = if tol > 0 {
            let (sync_l, sync_u) = if s_sync > 0 { (s_sync - tol, s_sync + tol) } else { (0, 0) };
            (s_short - tol, s_short + tol, s_long - tol, s_long + tol, sync_l, sync_u)
        } else {
            // no sync, short=0, long=1
            let zero_u = (s_short + s_long) / 2 + 1;
            (0, zero_u, zero_u - 1, if s_gap != 0 { s_gap } else { s_reset }, 0, 0)
        };
        Self { zero_l, zero_u, one_l, one_u, sync_l, sync_u, s_reset }
    }

    #[inline]
    fn gap(&self, bits: &mut BitBuffer, g: i64, last: bool) -> bool {
        if g > self.zero_l && g < self.zero_u {
            bits.add_bit(false);
        } else if g > self.one_l && g < self.one_u {
            bits.add_bit(true);
        } else if g > self.sync_l && g < self.sync_u {
            bits.add_sync();
        } else if g < self.s_reset {
            // New packet in multipacket
            bits.add_row();
        }
        (last || g >= self.s_reset) && (bits.bits_per_row(0) > 0 || bits.num_rows() > 1)
    }
}

/// `pulse_slicer_manchester_zerobit`'s limits and state: the pulse of the current pair (its edge
/// is decided with the gap: the tolerance test needs both) and `time_since_last`.
#[derive(Debug, Clone)]
pub struct Zerobit {
    s_short: i64,
    s_tolerance: i64,
    s_reset: i64,
    time_since_last: i64,
    pulse: i64,
}

impl Zerobit {
    fn new(d: &Device) -> Self {
        Self { s_short: i64::from(d.short_width), s_tolerance: i64::from(d.tolerance), s_reset: i64::from(d.reset_limit), time_since_last: 0, pulse: 0 }
    }

    /// The C's loop body for the pair (the stored pulse and `g`).
    #[inline]
    fn gap(&mut self, bits: &mut BitBuffer, g: i64, last: bool) -> bool {
        let (p, s, tol) = (self.pulse, self.s_short, self.s_tolerance);
        // `x > s_short * 1.5` in integers: 2x > 3 s_short
        if tol > 0 && (p < s - tol || p > s * 2 + tol || g < s - tol || g > s * 2 + tol) {
            // The pulse or gap is too long or too short, thus invalid
            if 2 * p > 3 * s && p <= s * 2 + tol {
                // Long last pulse means with the gap this is a [1]10 transition, add a one
                bits.add_bit(true);
            }
            bits.add_row();
            bits.add_bit(false); // Prepare for new message with hardcoded 0
            self.time_since_last = 0;
        } else if 2 * (p + self.time_since_last) > 3 * s {
            // Falling edge is on end of pulse: a data edge (falling data edge means bit = 1)
            bits.add_bit(true);
            self.time_since_last = 0;
        } else {
            self.time_since_last += p;
        }
        if (last || g > self.s_reset) && bits.num_rows() > 0 {
            return true;
        }
        if 2 * (g + self.time_since_last) > 3 * s {
            // Rising edge is on end of gap: a data edge (rising data edge means bit = 0)
            bits.add_bit(false);
            self.time_since_last = 0;
        } else {
            self.time_since_last += g;
        }
        false
    }
}

/// `pulse_slicer_piwm_dc`'s limits.
#[derive(Debug, Clone)]
pub struct PiwmDc {
    s_short: i64,
    s_long: i64,
    s_reset: i64,
    s_tol: i64,
}

impl PiwmDc {
    fn new(d: &Device) -> Self {
        Self { s_short: i64::from(d.short_width), s_long: i64::from(d.long_width), s_reset: i64::from(d.reset_limit), s_tol: i64::from(d.tolerance) }
    }

    /// One symbol (`last`: the package's last gap); true at an end of message.
    #[inline]
    fn symbol(&self, bits: &mut BitBuffer, symbol: i64, last: bool) -> bool {
        if (symbol - self.s_short).abs() < self.s_tol {
            bits.add_bit(true);
        } else if (symbol - self.s_long).abs() < self.s_tol {
            bits.add_bit(false);
        } else if symbol < self.s_reset {
            let rows = bits.num_rows();
            if rows > 0 && bits.bits_per_row(rows - 1) > 0 {
                bits.add_row();
            }
        }
        (last || symbol > self.s_reset) && bits.num_rows() > 0
    }
}

/// `pulse_slicer_pcm`: the device's timing, its package as the detector hands it over, and (FSK)
/// the carrier-off reading of the last gap. The C's integer arithmetic on samples is kept: durations
/// are `i32` (longer ones read as `i32::MAX`).
#[derive(Debug, Clone)]
pub struct Pcm {
    s_short: i32,
    s_long: i32,
    s_reset: i32,
    s_gap: i32,
    s_tolerance: i32,
    tail: Option<(FskFrame, i32)>,
    pulse: Vec<i32>,
    gap: Vec<i32>,
}

/// A duration as the PCM slicer's `int`.
fn int(us: u32) -> i32 {
    i32::try_from(us).unwrap_or(i32::MAX)
}

impl Pcm {
    fn new(d: &Device, tail: Option<(FskFrame, u32)>) -> Self {
        Self {
            s_short: int(d.short_width),
            s_long: int(d.long_width),
            s_reset: int(d.reset_limit),
            s_gap: int(d.gap_limit),
            s_tolerance: int(d.tolerance),
            tail: tail.map(|(f, off)| (f, int(off))),
            pulse: Vec::new(),
            gap: Vec::new(),
        }
    }

    fn clear(&mut self) {
        self.pulse.clear();
        self.gap.clear();
    }

    /// Collect a detector event; true when the package is complete.
    #[inline]
    pub fn event(&mut self, ev: Event) -> bool {
        match ev {
            Event::Pulse(p) => {
                self.pulse.push(int(p));
                self.gap.push(0);
                false
            }
            Event::Gap { gap, last } | Event::Longer { gap, last } => {
                if let Some(g) = self.gap.last_mut() {
                    *g = int(gap);
                }
                last
            }
            Event::End => true,
        }
    }

    /// Slice the collected package, running `decode` at every end of message; the first event goes
    /// to `out` unless it holds one already (rtl_433 outputs every message, KAT one decode per
    /// pulse). The package is cleared.
    #[inline(never)]
    pub fn slice<D: DecodeFn>(&mut self, bits: &mut BitBuffer, decode: &mut D, out: &mut Option<DecodedSignal>) {
        let ev = match self.tail {
            Some((frame, carrier_off)) => self.slice_fsk(bits, &frame, carrier_off, decode),
            None => self.slice_with(bits, false, |b, _| account(b, decode)),
        };
        if out.is_none() {
            *out = ev;
        }
        self.clear();
    }

    /// [Self::slice] for an FSK package (the carrier-off reading of the last gap, [FskFrame]).
    fn slice_fsk<D: DecodeFn>(&self, bits: &mut BitBuffer, f: &FskFrame, carrier_off: i32, decode: &mut D) -> Option<DecodedSignal> {
        let n = self.pulse.len();
        // Each run of ones in a row is one pulse's: fewer pulses than the sync word has runs of
        // ones cannot hold it.
        let sync_runs = (0..f.sync_bits).filter(|&i| bitrow_get_bit(f.sync, i) && (i == 0 || !bitrow_get_bit(f.sync, i - 1))).count();
        if n == 0 || n < sync_runs {
            return None;
        }
        let bit = i64::from(self.s_long.max(1));
        let gap_limit = if self.s_gap != 0 { self.s_gap } else { self.s_reset };
        let max_tail = (carrier_off / self.s_long.max(1)).min(gap_limit / self.s_long.max(1));
        let span: i64 = self.pulse.iter().map(|&p| i64::from(p)).sum::<i64>() + self.gap[..n - 1].iter().map(|&g| i64::from(g)).sum::<i64>();
        if 2 * span / bit + n as i64 + i64::from(max_tail) + 1 < f.min_bits as i64 {
            return None;
        }
        if self.gap.last().is_none_or(|&g| g <= carrier_off) {
            return self.slice_with(bits, false, |b, _| account(b, decode));
        }
        self.slice_with(bits, true, |bits, last| {
            if !last {
                return account(bits, decode);
            }
            let r = Self::read_tail(bits, f, max_tail, decode);
            decode.message_start();
            r
        })
    }

    /// The package's last message read with each tail of zero bits ([FskFrame]).
    fn read_tail<D: DecodeFn>(bits: &mut BitBuffer, f: &FskFrame, max_tail: i32, decode: &mut D) -> Option<DecodedSignal> {
        if bits.search(0, 0, f.sync, f.sync_bits) == bits.bits_per_row(0) {
            return None;
        }
        let mut unchecked = None;
        for tail in 0..=max_tail {
            if tail > 0 {
                bits.add_bit(false);
            }
            if let Some(r) = decode.message(bits) {
                if r.crc_valid {
                    return Some(r);
                }
                if unchecked.is_none() && f.unchecked.is_some_and(|ok| ok(&r)) {
                    unchecked = Some(r);
                }
            }
        }
        unchecked
    }

    /// The PCM slicer over the collected package; `decode` gets each message's bit buffer and
    /// whether the message is the package's last; `bare_last_gap`: the last gap adds no zeros (the
    /// caller adds them).
    fn slice_with(&self, bits: &mut BitBuffer, bare_last_gap: bool, mut decode: impl FnMut(&mut BitBuffer, bool) -> Option<DecodedSignal>) -> Option<DecodedSignal> {
        let (pulses, gaps) = (&self.pulse, &self.gap);
        let num = pulses.len();
        let (s_short, s_long, s_reset, s_gap) = (self.s_short, self.s_long, self.s_reset, self.s_gap);
        let mut s_tolerance = self.s_tolerance;
        if s_short <= 0 && s_long <= 0 {
            return None;
        }
        // Precision reciprocals.
        let mut f_short: f32 = if s_short > 0 { 1.0 / s_short as f32 } else { 0.0 };
        let mut f_long: f32 = if s_long > 0 { 1.0 / s_long as f32 } else { 0.0 };
        let gap_limit = if s_gap != 0 { s_gap } else { s_reset };
        let max_zeros = if s_long != 0 { gap_limit / s_long } else { 0 };
        if s_tolerance <= 0 {
            s_tolerance = s_long / 4; // default tolerance is ±25 % of a bit period
        }
        // A run of bit-wide toggles (preamble) tunes the bit period.
        let mut min_count = if s_short == s_long { 12 } else { 4 };
        let mut preamble_len = 0;
        let within = |v: i32, c: i32| v >= c - s_tolerance && v <= c + s_tolerance;
        if s_short != s_long {
            // RZ
            let mut n = 0;
            while n < num {
                let (mut swidth, mut lwidth, mut count) = (0i32, 0i32, 0i32);
                while n < num && within(pulses[n], s_short) && within(pulses[n].saturating_add(gaps[n]), s_long) {
                    swidth += pulses[n];
                    lwidth += pulses[n] + gaps[n];
                    count += 1;
                    n += 1;
                }
                if count >= min_count {
                    f_long = count as f32 / lwidth as f32;
                    f_short = count as f32 / swidth as f32;
                    min_count = count;
                    preamble_len = count;
                }
                n += 1;
            }
            // RZ bits within tolerance anywhere.
            if preamble_len == 0 {
                let (mut rzs, mut rzl, mut rzc) = (0i32, 0i32, 0i32);
                for n in 0..num {
                    if within(pulses[n], s_short) && within(pulses[n].saturating_add(gaps[n]), s_long) {
                        rzs += pulses[n];
                        rzl += pulses[n] + gaps[n];
                        rzc += 1;
                    }
                }
                if rzc > 8 {
                    f_long = rzc as f32 / rzl as f32;
                    f_short = rzc as f32 / rzs as f32;
                }
            }
        } else {
            // NRZ
            let one = |v: i32, f: f32| ((v as f32 * f) as f64 + 0.5) as i32 == 1;
            let mut n = 0;
            while n < num {
                let (mut width, mut count) = (0i32, 0i32);
                while n < num && one(pulses[n], f_short) && one(gaps[n], f_long) {
                    width += pulses[n] + gaps[n];
                    count += 2;
                    n += 1;
                }
                if count >= min_count {
                    f_short = count as f32 / width as f32;
                    f_long = f_short;
                    min_count = count;
                    preamble_len = count;
                }
                n += 1;
            }
            // NRZ pulse / gap of 1 or 2 bits within tolerance anywhere.
            if preamble_len == 0 {
                let (mut nrz_width, mut nrz_count) = (0i32, 0i32);
                for n in 0..num {
                    if within(pulses[n], s_short) {
                        nrz_width += pulses[n];
                        nrz_count += 1;
                    }
                    if within(pulses[n], 2 * s_short) {
                        nrz_width += pulses[n];
                        nrz_count += 2;
                    }
                    if within(gaps[n], s_long) {
                        nrz_width += gaps[n];
                        nrz_count += 1;
                    }
                    if within(gaps[n], 2 * s_long) {
                        nrz_width += gaps[n];
                        nrz_count += 2;
                    }
                }
                if nrz_count > 20 {
                    f_short = nrz_count as f32 / nrz_width as f32;
                    f_long = f_short;
                }
            }
        }

        bits.clear();
        let mut found = None;
        for n in 0..num {
            // Number of high bit periods (NRZ: bits may not be separated).
            let highs = (pulses[n] as f32 * f_short + 0.5) as i32;
            // Number of low bit periods (RZ: minus the nominal bit gap).
            let mut lows = ((i64::from(gaps[n]) + i64::from(s_short - s_long)) as f32 * f_long + 0.5) as i32;
            if bare_last_gap && n == num - 1 {
                lows = 0;
            }
            bits.add_bits(true, highs);
            lows = lows.min(max_zeros); // don't overflow at the end of the message
            bits.add_bits(false, lows);
            if s_short != s_long && (pulses[n] - s_short).abs() > s_tolerance {
                // RZ: data corrupt.
                bits.clear();
            } else if gaps[n] > gap_limit && gaps[n] <= s_reset {
                // New packet in a multipacket message.
                bits.add_row();
            }
            // End of Message?
            if (n == num - 1 || gaps[n] > s_reset) && (bits.bits_per_row(0) > 0 || bits.num_rows() > 1) {
                let r = decode(bits, n == num - 1);
                if found.is_none() {
                    found = r;
                }
                bits.clear();
            }
        }
        found
    }
}

/// `decode_fn` on a message (`account_event`, the buffer is cleared by the caller).
fn account<D: DecodeFn>(bits: &mut BitBuffer, decode: &mut D) -> Option<DecodedSignal> {
    let r = decode.message(bits);
    decode.message_start();
    r
}

#[cfg(test)]
mod tests {
    use super::super::pulse_detect::Gaps;
    use super::*;

    fn device(modulation: Modulation, short_width: u32, long_width: u32, gap_limit: u32, reset_limit: u32, tolerance: u32) -> Device {
        Device { modulation, short_width, long_width, reset_limit, gap_limit, sync_width: 0, tolerance, gaps: Gaps::Merged, ends: Ends::Ook, resync: true }
    }

    #[test]
    fn pwm_rows_and_end_of_message() {
        // rtl_433 hcs362 PWM: s 200, l 400, g 550, r 900, t 50.
        let mut s = Slicer::new(&device(Modulation::OokPwm, 200, 400, 550, 900, 50));
        let mut bits = BitBuffer::new();
        for p in [200, 200, 400] {
            s.pulse(&mut bits, p);
            assert!(!s.gap(&mut bits, 200, false));
        }
        s.pulse(&mut bits, 200);
        assert!(!s.gap(&mut bits, 600, false), "new row");
        s.pulse(&mut bits, 400);
        s.pulse(&mut bits, 120); // spurious: ignored
        assert!(s.gap(&mut bits, 1000, false), "end of message");
        assert_eq!(bits.num_rows(), 2);
        assert_eq!((bits.code(0), bits.code(1)), ("{4}d0".to_string(), "{1}00".to_string()));
    }

    /// A PCM decode_fn that collects every message's rows as rtl_433 codes.
    struct Codes(Vec<Vec<String>>);

    impl DecodeFn for Codes {
        fn message(&mut self, bits: &mut BitBuffer) -> Option<DecodedSignal> {
            self.0.push((0..bits.num_rows()).map(|r| bits.code(r)).collect());
            None
        }
    }

    #[test]
    fn pcm_tunes_the_bit_period_on_the_preamble() {
        // rtl_433 hcs362 MC: s = l = 214, g 600, r 900, t 50; a 216/215 preamble.
        let Slicer::Pcm(mut pcm) = Slicer::new(&device(Modulation::OokPcm, 214, 214, 600, 900, 50)) else { unreachable!() };
        for _ in 0..11 {
            pcm.event(Event::Pulse(216));
            pcm.event(Event::Gap { gap: 215, last: false });
        }
        pcm.event(Event::Pulse(216));
        pcm.event(Event::Gap { gap: 863, last: false });
        for (p, g) in [(216, 216), (432, 431), (216, 20_000)] {
            pcm.event(Event::Pulse(p));
            assert_eq!(pcm.event(Event::Gap { gap: g, last: g == 20_000 }), g == 20_000);
        }
        let mut codes = Codes(Vec::new());
        pcm.slice(&mut BitBuffer::new(), &mut codes, &mut None);
        // Preamble 11 × 10 + 1 + 2 zeros (max_zeros = 600 / 214); data 10 110 0 1 + 2 zeros.
        assert_eq!(codes.0, vec![vec!["{25}aaaaaa00".to_string(), "{9}b200".to_string()]]);
    }
}
