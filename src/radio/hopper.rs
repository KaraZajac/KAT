//! Frequency hopping for the live receivers (Flipper / ProtoPirate behaviour).
//!
//! - Default list: Flipper-ARF `subghz_hopper_frequency_list` (`lib/subghz/subghz_setting.c`):
//!   315.00, 390.00, 430.50, 433.92, 434.25, 434.42, 868.35 MHz.
//! - ProtoPirate `protopirate_hopper_update` (`helpers/protopirate_txrx.c`, called every 100 ms
//!   tick): on a quiet frequency it hops every tick; when the RSSI passes the threshold it holds for
//!   10 ticks; after a decode the receiver scene pauses it for 50 ticks.
//!
//! KAT replaces the absolute RSSI threshold with the pipeline's squelch (carrier over the tracked
//! noise floor): it holds while a burst is open, then for [`HOLD_DWELLS`] dwells after it closes
//! (ProtoPirate's RSSI time-out), and the app pauses it for [`DECODE_PAUSE_DWELLS`] dwells after a
//! decode. The state machine counts samples, not wall time, so it is exact and testable; the receive
//! threads feed it and perform the retune it asks for.

use super::iq_file::parse_frequency_arg;

/// Flipper-ARF's default hopper list (Hz).
pub const DEFAULT_HOP_FREQUENCIES: [u32; 7] =
    [315_000_000, 390_000_000, 430_500_000, 433_920_000, 434_250_000, 434_420_000, 868_350_000];
/// Time on a quiet frequency before hopping (ProtoPirate's 100 ms tick).
pub const DEFAULT_DWELL_MS: u32 = 100;
/// Hold after a burst closes, in dwells (ProtoPirate: 10 ticks after the RSSI threshold).
pub const HOLD_DWELLS: u64 = 10;
/// Pause after a decode, in dwells (ProtoPirate: 50 ticks after a key is found).
pub const DECODE_PAUSE_DWELLS: u64 = 50;

/// Which frequencies to visit and how long to stay on a quiet one.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HopPlan {
    pub frequencies: Vec<u32>,
    pub dwell_ms: u32,
}

impl HopPlan {
    /// Plan with the dwell clamped to 20 ms..10 s and duplicate frequencies removed.
    pub fn new(frequencies: Vec<u32>, dwell_ms: u32) -> Self {
        let mut f: Vec<u32> = Vec::with_capacity(frequencies.len());
        for x in frequencies {
            if !f.contains(&x) {
                f.push(x);
            }
        }
        Self { frequencies: f, dwell_ms: dwell_ms.clamp(20, 10_000) }
    }

    /// Flipper-ARF's list with ProtoPirate's 100 ms tick.
    #[allow(dead_code)]
    pub fn default_plan() -> Self {
        Self::new(DEFAULT_HOP_FREQUENCIES.to_vec(), DEFAULT_DWELL_MS)
    }

    /// Short human-readable list, e.g. "315.00, 433.92, 868.35 MHz".
    pub fn describe(&self) -> String {
        let list: Vec<String> = self.frequencies.iter().map(|&f| format_mhz(f)).collect();
        format!("{} MHz, {} ms", list.join(", "), self.dwell_ms)
    }
}

/// "433.92" style MHz with 2..3 decimals (430.50, 433.92, 433.875).
pub fn format_mhz(hz: u32) -> String {
    let s = format!("{:.3}", hz as f64 / 1e6);
    match s.strip_suffix('0') {
        Some(t) => t.to_string(),
        None => s,
    }
}

/// Parse a list like "315, 433.92M 868350000" into Hz: each entry as `:freq` reads it
/// ([parse_frequency_arg]: MHz, or Hz for values ≥ 10 000, any unit and case), a unit typed apart
/// ("433.92 MHz") belonging to the number before it.
pub fn parse_frequency_list(text: &str) -> Result<Vec<u32>, String> {
    let is_unit = |t: &str| ["hz", "khz", "mhz", "ghz", "k", "m", "g"].contains(&t.to_ascii_lowercase().as_str());
    let mut tokens: Vec<String> = Vec::new();
    for tok in text.split(|c: char| c == ',' || c.is_whitespace()).filter(|t| !t.is_empty()) {
        match tokens.last_mut() {
            Some(prev) if is_unit(tok) => prev.push_str(tok),
            _ => tokens.push(tok.to_string()),
        }
    }
    let mut out = Vec::new();
    for tok in tokens {
        let hz = parse_frequency_arg(&tok).ok_or_else(|| format!("Invalid frequency: {tok} (e.g. 433.92)"))?;
        if hz > 1_800_000_000 {
            return Err(format!("Frequency out of range: {tok} (24–1800 MHz)"));
        }
        out.push(hz);
    }
    if out.is_empty() {
        return Err("No frequencies given".to_string());
    }
    Ok(out)
}

/// A fob frequency of the default list that lies near `targets` (within 1.5 MHz of one) but
/// outside what all of them hear (`heard`: Hz from a target, low and high), as (fob, the target
/// nearest it); the nearest such pair. A receiver hears 1.14 MHz below its frequency but only
/// 0.64 MHz above it (2 MS/s, offset tuning), so `433` (433.000 MHz, as `:hop 315 433` takes it)
/// never hears a fob on 433.92 MHz.
pub fn fob_not_heard(targets: &[u32], heard: (f64, f64)) -> Option<(u32, u32)> {
    let hears = |target: u32, fob: u32| (heard.0..=heard.1).contains(&(f64::from(fob) - f64::from(target)));
    DEFAULT_HOP_FREQUENCIES
        .iter()
        .filter(|&&fob| !targets.iter().any(|&t| hears(t, fob)))
        .filter_map(|&fob| targets.iter().map(|&t| (fob.abs_diff(t), fob, t)).filter(|p| p.0 <= 1_500_000).min())
        .min()
        .map(|(_, fob, target)| (fob, target))
}

/// Sample-clock driven hop state machine.
#[derive(Debug, Clone)]
pub struct Hopper {
    plan: HopPlan,
    index: usize,
    dwell: u64,
    on_freq: u64,
    hold: u64,
    pause: u64,
    pending: bool,
}

impl Hopper {
    /// Hopper at `index` of the plan, for samples arriving at `sample_rate`.
    pub fn new(plan: HopPlan, sample_rate: f64, index: usize) -> Self {
        let dwell = (plan.dwell_ms as f64 * sample_rate / 1000.0) as u64;
        let index = if plan.frequencies.is_empty() { 0 } else { index % plan.frequencies.len() };
        Self { plan, index, dwell: dwell.max(1), on_freq: 0, hold: 0, pause: 0, pending: false }
    }

    /// Frequency the hopper is on (or last asked for, until [`Hopper::hopped`]).
    #[allow(dead_code)]
    pub fn current(&self) -> u32 {
        self.plan.frequencies.get(self.index).copied().unwrap_or(0)
    }

    pub fn plan(&self) -> &HopPlan {
        &self.plan
    }

    /// Account for `n` samples processed on the current frequency; `busy` = a burst is open.
    /// Returns the next frequency when it is time to hop (once, until [`Hopper::hopped`]).
    pub fn advance(&mut self, n: u64, busy: bool) -> Option<u32> {
        if self.pending || self.plan.frequencies.len() < 2 {
            return None;
        }
        self.on_freq += n;
        if busy {
            self.hold = HOLD_DWELLS * self.dwell;
            return None;
        }
        // A hold or pause that runs out within these samples lets the hop happen now.
        if self.hold > 0 {
            self.hold = self.hold.saturating_sub(n);
            if self.hold > 0 {
                return None;
            }
        }
        if self.pause > 0 {
            self.pause = self.pause.saturating_sub(n);
            if self.pause > 0 {
                return None;
            }
        }
        if self.on_freq < self.dwell {
            return None;
        }
        self.pending = true;
        Some(self.plan.frequencies[(self.index + 1) % self.plan.frequencies.len()])
    }

    /// The radio now delivers `target`: go on from its place in the list. Usually that is the hop
    /// [`Hopper::advance`] asked for, but a retune asked for before this plan was installed (a
    /// `:hop` while hopping) can land instead. Counted as the next hop, that one put the hopper a
    /// place off, so it asked next for the frequency the radio was already on, a retune the RTL-SDR
    /// and SoapySDR readers never report: hopping stopped for good.
    pub fn hopped(&mut self, target: u32) {
        if let Some(i) = self.plan.frequencies.iter().position(|&f| f == target) {
            self.index = i;
        }
        self.pending = false;
        self.on_freq = 0;
    }

    /// The retune failed: stay, and try again after another dwell.
    #[allow(dead_code)]
    pub fn cancel(&mut self) {
        self.pending = false;
        self.on_freq = 0;
    }

    /// Stay on the current frequency for `dwells` more dwells (e.g. after a decode).
    pub fn pause_dwells(&mut self, dwells: u64) {
        self.pause = self.pause.max(dwells * self.dwell);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FS: f64 = 1_000.0; // 1 sample per ms keeps the arithmetic readable

    fn plan() -> HopPlan {
        HopPlan::new(vec![315_000_000, 433_920_000, 868_350_000], 100)
    }

    #[test]
    fn hops_after_each_dwell_and_wraps() {
        let mut h = Hopper::new(plan(), FS, 0);
        assert_eq!(h.current(), 315_000_000);
        assert_eq!(h.advance(99, false), None);
        assert_eq!(h.advance(1, false), Some(433_920_000));
        // Nothing more until the retune is confirmed.
        assert_eq!(h.advance(500, false), None);
        h.hopped(433_920_000);
        assert_eq!(h.current(), 433_920_000);
        assert_eq!(h.advance(100, false), Some(868_350_000));
        h.hopped(868_350_000);
        assert_eq!(h.advance(100, false), Some(315_000_000));
        h.hopped(315_000_000);
        assert_eq!(h.current(), 315_000_000);
    }

    /// A retune it did not ask for (from before its plan) puts it where the radio is: the next hop
    /// is never to the frequency the radio is on.
    #[test]
    fn goes_on_from_where_the_radio_is() {
        let mut h = Hopper::new(plan(), FS, 0);
        h.hopped(433_920_000);
        assert_eq!(h.current(), 433_920_000);
        assert_eq!(h.advance(100, false), Some(868_350_000));
        // Its own hop was asked for, another one landed: it goes on from there.
        h.hopped(315_000_000);
        assert_eq!(h.advance(100, false), Some(433_920_000));
        // Off the list (the way home, before hopping was switched back on): on to the list again.
        h.hopped(433_920_000);
        h.hopped(390_000_000);
        assert_eq!(h.advance(100, false), Some(868_350_000));
    }

    #[test]
    fn holds_while_busy_then_for_ten_dwells() {
        let mut h = Hopper::new(plan(), FS, 0);
        assert_eq!(h.advance(50, true), None);
        assert_eq!(h.advance(5_000, true), None, "burst open: hold");
        // Closed: hold 10 dwells (1000 samples) more, then hop.
        assert_eq!(h.advance(999, false), None);
        assert_eq!(h.advance(1, false), Some(433_920_000));
    }

    #[test]
    fn pause_after_decode() {
        let mut h = Hopper::new(plan(), FS, 0);
        h.pause_dwells(DECODE_PAUSE_DWELLS);
        assert_eq!(h.advance(4_999, false), None);
        assert_eq!(h.advance(2, false), Some(433_920_000));
    }

    /// What 2 MS/s hears (−1.14 to +0.64 MHz): `433` misses the 433.92 MHz fobs above it, the
    /// default list and the frequencies fobs are on miss nothing, and nothing far away counts.
    #[test]
    fn a_fob_frequency_near_the_list_that_none_of_it_hears() {
        let heard = (-1_140_625.0, 640_625.0);
        assert_eq!(fob_not_heard(&[315_000_000, 433_000_000], heard), Some((433_920_000, 433_000_000)));
        assert_eq!(fob_not_heard(&[314_000_000], heard), Some((315_000_000, 314_000_000)));
        assert_eq!(fob_not_heard(&[433_500_000], heard), Some((434_250_000, 433_500_000)), "433.92 heard, 434.25 not");
        assert_eq!(fob_not_heard(&DEFAULT_HOP_FREQUENCIES, heard), None);
        assert_eq!(fob_not_heard(&[315_000_000, 433_920_000], heard), None);
        assert_eq!(fob_not_heard(&[433_920_000, 868_000_000], heard), None, "868.35 is 0.35 MHz above");
        assert_eq!(fob_not_heard(&[300_000_000, 920_000_000], heard), None, "no fob frequency near");
    }

    #[test]
    fn single_frequency_never_hops() {
        let mut h = Hopper::new(HopPlan::new(vec![433_920_000, 433_920_000], 100), FS, 0);
        assert_eq!(h.advance(10_000, false), None);
    }

    #[test]
    fn parse_and_format_lists() {
        assert_eq!(parse_frequency_list("315, 433.92 868350000").unwrap(), vec![315_000_000, 433_920_000, 868_350_000]);
        // Units as :freq takes them, also typed apart from the number.
        assert_eq!(
            parse_frequency_list("315M, 433.92 MHz 868.35mhz 434420000 Hz").unwrap(),
            vec![315_000_000, 433_920_000, 868_350_000, 434_420_000]
        );
        assert!(parse_frequency_list("abc").is_err());
        assert!(parse_frequency_list("5").is_err());
        assert!(parse_frequency_list("MHz 315").is_err(), "a unit with no number before it");
        assert!(parse_frequency_list("315 2400").unwrap_err().contains("out of range"));
        assert_eq!(format_mhz(430_500_000), "430.50");
        assert_eq!(format_mhz(433_920_000), "433.92");
        assert_eq!(format_mhz(433_875_000), "433.875");
        assert_eq!(HopPlan::default_plan().frequencies.len(), 7);
    }
}
