//! rtl_433's pulse detector (`src/pulse_detect.c`) rebuilt on KAT's level/duration stream: the
//! (pulse, gap) pairs and package boundaries it hands the slicers, as [Event]s. How the stream
//! maps onto packages, and the three choices a device makes ([Gaps], [Ends], resync), are
//! described in the `rtl433` module doc.

/// `PD_MAX_PULSES`: a package ends at its 1200th pulse.
const PD_MAX_PULSES: u32 = 1200;
/// `PD_MIN_GAP_MS` (µs).
const PD_MIN_GAP_US: u64 = 10_000;
/// `PD_MAX_GAP_MS` (µs).
const PD_MAX_GAP_US: u64 = 100_000;
/// `PD_MAX_GAP_RATIO`.
const PD_MAX_GAP_RATIO: u64 = 10;

/// How a LOW is turned into the gap of a pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Gaps {
    /// The LOW after a pulse is its gap and the pair is sliced at once (KAT feeds whole level
    /// durations, so the gap is complete). A further LOW right after it only extends the gap for
    /// the end-of-message and package decisions ([Event::Longer]); the bits that gap already
    /// produced stay (rtl_433's detector would have measured one gap).
    AtOnce,
    /// Consecutive LOWs add up to one gap, handed to the slicer when it is complete (at the next
    /// HIGH), or as soon as it ends the slicer's message or the package.
    Merged,
}

/// Which of the pulse detector's package ends apply. The slicers end a message at the package's
/// last pulse, and every package starts with an empty bit buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ends {
    /// None: the stream is one package; only the slicer's reset limit ends a message.
    None,
    /// rtl_433's OOK package ends: a gap longer than 10 ms and 10 × the package's longest pulse,
    /// a gap longer than 100 ms, the 1200th pulse (`PD_MIN_GAP_MS`, `PD_MAX_GAP_RATIO`,
    /// `PD_MAX_GAP_MS`, `PD_MAX_PULSES`).
    Ook,
    /// The OOK ends, and a LOW longer than `carrier_off` µs: on KAT's FM path a stopped carrier
    /// and the lower tone are both LOW, while rtl_433's FSK packages end with the carrier; a LOW
    /// longer than the device's coding allows can only be the carrier stopping.
    Fsk { carrier_off: u32 },
}

/// What the detector hands a slicer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    /// The HIGH of the current pair ended (µs).
    Pulse(u32),
    /// The gap of the current pair; `last`: it ends the package (`n == num_pulses - 1`).
    Gap { gap: u32, last: bool },
    /// [Gaps::AtOnce]: a further LOW made the handed-over gap `gap` µs long; `last`: it now ends
    /// the package.
    Longer { gap: u32, last: bool },
    /// [Gaps::Merged]: the package ended after its last gap was handed over.
    End,
}

/// The pulse detector for one device.
#[derive(Debug, Clone)]
pub struct Detector {
    gaps: Gaps,
    ends: Ends,
    /// Wait for silence after a reset that is not the one following an own report (see
    /// [Self::new]).
    resync: bool,
    /// [Gaps::Merged]: a gap at least this long (µs) ends the slicer's message and is handed over
    /// at once (None: the slicer takes whole packages).
    early: Option<u32>,
    /// HIGH time of the pulse being received (None: no pulse since the last gap).
    pulse: Option<u32>,
    /// The gap after the last pulse, while it lasts.
    gap: u32,
    gap_open: bool,
    /// [Gaps::Merged]: the open gap was handed over already.
    delivered: bool,
    /// The package's longest pulse and its complete pairs.
    max_pulse: u32,
    pairs: u32,
    /// Waiting for silence: the LOW so far and the longest pulse since the reset.
    resyncing: bool,
    silence: u64,
    silence_max: u32,
    /// The receiver reported: the reset that follows keeps the message.
    reported: bool,
}

impl Detector {
    /// A detector for a device read with `gaps` and `ends`. `early`: see [Gaps::Merged].
    ///
    /// `resync`: after a reset that is not the one following its own report (another decoder's
    /// report, or a new stream) the rest of the current package can no longer be sliced as
    /// rtl_433 would — its start is gone — so the detector waits for silence first: a LOW of at
    /// least 10 ms and 10 × the longest pulse since the reset, or over 100 ms (rtl_433's
    /// package-ending gaps). It starts in that state (a stream from KAT's receiver starts with
    /// such a LOW). Such a detector also measures a pulse as HIGH time over 0 µs (a 0 µs HIGH only
    /// completes the gap before it), and keeps its message across the first reset after its own
    /// report even when more pulses came in between. Without `resync` a 0 µs HIGH is a 0 µs
    /// pulse, and only a reset that directly follows the report is ignored; in the registry,
    /// which resets right after every report, the two keep rules agree.
    pub fn new(gaps: Gaps, ends: Ends, resync: bool, early: Option<u32>) -> Self {
        Self {
            gaps,
            ends,
            resync,
            early,
            pulse: None,
            gap: 0,
            gap_open: false,
            delivered: false,
            max_pulse: 0,
            pairs: 0,
            resyncing: resync,
            silence: 0,
            silence_max: 0,
            reported: false,
        }
    }

    /// The receiver reported: the registry's reset that follows keeps the message.
    pub fn own_report(&mut self) {
        self.reported = true;
    }

    /// The registry's reset. False after the receiver's own report (nothing is reset: the package
    /// goes on, as in rtl_433, which only clears the bit buffer after each message); otherwise
    /// the detector starts over and returns true (the receiver clears its slicer).
    pub fn reset(&mut self) -> bool {
        if std::mem::take(&mut self.reported) {
            return false;
        }
        *self = Self::new(self.gaps, self.ends, self.resync, self.early);
        true
    }

    /// Whether a gap of `gap` µs ends the package (the pulse detector's end-of-package rule).
    #[inline]
    fn ends_package(&self, gap: u32) -> bool {
        let (gap, max) = (u64::from(gap), u64::from(self.max_pulse));
        let ook = || (gap > PD_MAX_GAP_RATIO * max && gap > PD_MIN_GAP_US) || gap > PD_MAX_GAP_US;
        match self.ends {
            Ends::None => false,
            Ends::Ook => ook(),
            Ends::Fsk { carrier_off } => ook() || gap > u64::from(carrier_off),
        }
    }

    /// A pair is complete: it counts toward `PD_MAX_PULSES`; true when it is the package's last.
    #[inline]
    fn count_pair(&mut self) -> bool {
        if matches!(self.ends, Ends::None) {
            return false;
        }
        self.pairs += 1;
        self.pairs >= PD_MAX_PULSES
    }

    fn new_package(&mut self) {
        self.max_pulse = 0;
        self.pairs = 0;
    }

    /// One level/duration; the events it completes, in order.
    #[inline(always)]
    pub fn feed(&mut self, level: bool, duration: u32) -> [Option<Event>; 2] {
        if !self.resync {
            self.reported = false;
        }
        if self.resyncing {
            if level {
                self.silence = 0;
                self.silence_max = self.silence_max.max(duration);
            } else {
                self.silence += u64::from(duration);
                if (self.silence >= PD_MIN_GAP_US && self.silence >= PD_MAX_GAP_RATIO * u64::from(self.silence_max)) || self.silence > PD_MAX_GAP_US {
                    self.resyncing = false;
                }
            }
            return [None, None];
        }
        if level {
            let mut ev = None;
            if self.gap_open {
                self.gap_open = false;
                if self.gaps == Gaps::Merged {
                    // The gap before this pulse is complete.
                    let last = self.count_pair();
                    ev = if !self.delivered {
                        Some(Event::Gap { gap: self.gap, last })
                    } else if last {
                        Some(Event::End)
                    } else {
                        None
                    };
                    if last {
                        self.new_package();
                    }
                }
            }
            self.pulse = Some(self.pulse.unwrap_or(0).saturating_add(duration));
            return [ev, None];
        }
        let pulse = self.pulse.take().filter(|&p| p > 0 || !self.resync);
        let first = if let Some(p) = pulse {
            self.max_pulse = self.max_pulse.max(p);
            self.gap = duration;
            self.gap_open = true;
            self.delivered = false;
            Some(Event::Pulse(p))
        } else if self.gap_open {
            self.gap = self.gap.saturating_add(duration);
            None
        } else {
            // Idle before a package, or the rest of a package's final gap.
            return [None, None];
        };
        match self.gaps {
            Gaps::AtOnce if first.is_some() => {
                let counted = self.count_pair();
                let last = self.ends_package(self.gap) || counted;
                self.delivered = true;
                if last {
                    self.gap_open = false;
                    self.new_package();
                }
                [first, Some(Event::Gap { gap: self.gap, last })]
            }
            Gaps::AtOnce => {
                let last = self.ends_package(self.gap);
                if last {
                    self.gap_open = false;
                    self.new_package();
                }
                [Some(Event::Longer { gap: self.gap, last }), None]
            }
            Gaps::Merged => {
                let second = if self.ends_package(self.gap) {
                    self.gap_open = false;
                    let ev = if self.delivered { Event::End } else { Event::Gap { gap: self.gap, last: true } };
                    self.new_package();
                    Some(ev)
                } else if !self.delivered && self.early.is_some_and(|e| self.gap >= e) {
                    self.delivered = true;
                    Some(Event::Gap { gap: self.gap, last: false })
                } else {
                    None
                };
                [first, second]
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use Event::*;

    fn feed(d: &mut Detector, seq: &[(bool, u32)]) -> Vec<Event> {
        seq.iter().flat_map(|&(l, t)| d.feed(l, t).into_iter().flatten()).collect()
    }

    #[test]
    fn merged_gaps_split_packages_like_the_pulse_detector() {
        let mut d = Detector::new(Gaps::Merged, Ends::Ook, true, Some(1001));
        let seq = [(false, 20_000), (true, 300), (false, 500), (true, 600), (false, 1500), (true, 200), (false, 9_000), (true, 300), (false, 12_000)];
        assert_eq!(
            feed(&mut d, &seq),
            vec![
                Pulse(300),
                Gap { gap: 500, last: false },
                Pulse(600),
                Gap { gap: 1500, last: false }, // over `early`: handed over at once
                Pulse(200),
                Gap { gap: 9_000, last: false },
                Pulse(300),
                Gap { gap: 12_000, last: true }, // > 10 ms and > 10 × 600 µs
            ]
        );
        // Consecutive LOWs merge; a long pulse raises the package-end gap (10 × 2 ms).
        let mut d = Detector::new(Gaps::Merged, Ends::Ook, true, None);
        let seq = [(false, 10_000), (true, 2_000), (false, 400), (false, 400), (true, 100), (false, 15_000), (false, 6_000)];
        assert_eq!(feed(&mut d, &seq), vec![Pulse(2_000), Gap { gap: 800, last: false }, Pulse(100), Gap { gap: 21_000, last: true }]);
        // FSK: a LOW over the carrier-off limit ends the package.
        let mut d = Detector::new(Gaps::Merged, Ends::Fsk { carrier_off: 416 }, true, Some(4001));
        let seq = [(false, 10_000), (true, 13), (false, 26), (true, 13), (false, 500)];
        assert_eq!(feed(&mut d, &seq), vec![Pulse(13), Gap { gap: 26, last: false }, Pulse(13), Gap { gap: 500, last: true }]);
    }

    #[test]
    fn at_once_gaps_are_handed_over_with_their_low() {
        let mut d = Detector::new(Gaps::AtOnce, Ends::Ook, false, None);
        // No wait for silence; a LOW before any pulse is idle; a further LOW is `Longer`.
        let seq = [(false, 500), (true, 300), (false, 400), (false, 9_000), (false, 2_000), (true, 200), (false, 300)];
        assert_eq!(
            feed(&mut d, &seq),
            vec![Pulse(300), Gap { gap: 400, last: false }, Longer { gap: 9_400, last: false }, Longer { gap: 11_400, last: true }, Pulse(200), Gap { gap: 300, last: false }]
        );
        // The 1200th pair ends the package at its own LOW; without package ends it does not.
        for (ends, last) in [(Ends::Ook, true), (Ends::None, false)] {
            let mut d = Detector::new(Gaps::AtOnce, ends, false, None);
            let seq: Vec<(bool, u32)> = (0..1200).flat_map(|_| [(true, 100), (false, 100)]).collect();
            assert_eq!(feed(&mut d, &seq).last(), Some(&Gap { gap: 100, last }));
        }
    }

    #[test]
    fn resync_after_a_foreign_reset_only() {
        let mut d = Detector::new(Gaps::Merged, Ends::Ook, true, Some(901));
        // A new stream: nothing before silence (10 ms, and 10 × the pulses seen).
        assert!(feed(&mut d, &[(true, 300), (false, 300), (true, 1_200), (false, 11_000)]).is_empty());
        assert!(feed(&mut d, &[(true, 300), (false, 300), (true, 1_200), (false, 13_000)]).is_empty());
        assert!(!feed(&mut d, &[(true, 300), (false, 1_000)]).is_empty());
        // After its own report the package goes on (more pulses may come before the reset).
        d.own_report();
        assert_eq!(feed(&mut d, &[(true, 200), (false, 300)]), vec![Pulse(200)]);
        assert!(!d.reset());
        assert_eq!(feed(&mut d, &[(true, 200), (false, 300)]), vec![Gap { gap: 300, last: false }, Pulse(200)]);
        // After another decoder's report it waits for silence again.
        assert!(d.reset());
        assert!(feed(&mut d, &[(true, 200), (false, 300), (true, 200), (false, 9_999)]).is_empty());
        assert_eq!(feed(&mut d, &[(false, 1), (true, 200), (false, 300)]), vec![Pulse(200)]);
        // A LOW over 100 ms is silence whatever the pulses before it (an inverted idle period).
        assert!(d.reset());
        assert!(feed(&mut d, &[(true, 200_000), (false, 100_001)]).is_empty());
        assert_eq!(feed(&mut d, &[(true, 200), (false, 300)]), vec![Pulse(200)]);
        // A 0 µs HIGH only completes the gap.
        assert_eq!(feed(&mut d, &[(true, 0), (false, 300)]), vec![Gap { gap: 300, last: false }]);
    }

    #[test]
    fn without_resync_only_a_reset_right_after_the_report_is_kept() {
        let mut d = Detector::new(Gaps::Merged, Ends::None, false, Some(1001));
        assert_eq!(feed(&mut d, &[(true, 300), (false, 2_000)]), vec![Pulse(300), Gap { gap: 2_000, last: false }]);
        d.own_report();
        assert!(!d.reset(), "directly after the report");
        d.own_report();
        assert!(feed(&mut d, &[(false, 100)]).is_empty());
        assert!(d.reset(), "a feed came in between");
        // A 0 µs HIGH is a 0 µs pulse.
        assert_eq!(feed(&mut d, &[(true, 0), (false, 300), (true, 5)]), vec![Pulse(0), Gap { gap: 300, last: false }]);
    }
}
