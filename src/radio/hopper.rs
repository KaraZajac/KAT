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

/// Parse a list like "315, 433.92 868.35" (MHz, or Hz for values ≥ 10 000) into Hz.
pub fn parse_frequency_list(text: &str) -> Result<Vec<u32>, String> {
    let mut out = Vec::new();
    for tok in text.split(|c: char| c == ',' || c.is_whitespace()).filter(|t| !t.is_empty()) {
        let v: f64 = tok.trim_end_matches("MHz").parse().map_err(|_| format!("Invalid frequency: {tok}"))?;
        let hz = if v < 10_000.0 { v * 1e6 } else { v };
        if !(24e6..=1.8e9).contains(&hz) {
            return Err(format!("Frequency out of range: {tok}"));
        }
        out.push(hz.round() as u32);
    }
    if out.is_empty() {
        return Err("No frequencies given".to_string());
    }
    Ok(out)
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

    /// The retune asked for by [`Hopper::advance`] took effect.
    pub fn hopped(&mut self) {
        if self.pending {
            self.index = (self.index + 1) % self.plan.frequencies.len().max(1);
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
        h.hopped();
        assert_eq!(h.current(), 433_920_000);
        assert_eq!(h.advance(100, false), Some(868_350_000));
        h.hopped();
        assert_eq!(h.advance(100, false), Some(315_000_000));
        h.hopped();
        assert_eq!(h.current(), 315_000_000);
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

    #[test]
    fn single_frequency_never_hops() {
        let mut h = Hopper::new(HopPlan::new(vec![433_920_000, 433_920_000], 100), FS, 0);
        assert_eq!(h.advance(10_000, false), None);
    }

    #[test]
    fn parse_and_format_lists() {
        assert_eq!(parse_frequency_list("315, 433.92 868350000").unwrap(), vec![315_000_000, 433_920_000, 868_350_000]);
        assert!(parse_frequency_list("abc").is_err());
        assert!(parse_frequency_list("5").is_err());
        assert_eq!(format_mhz(430_500_000), "430.50");
        assert_eq!(format_mhz(433_920_000), "433.92");
        assert_eq!(format_mhz(433_875_000), "433.875");
        assert_eq!(HopPlan::default_plan().frequencies.len(), 7);
    }
}
