//! AM/OOK and FM/2FSK demodulators for extracting level+duration pairs from raw IQ samples.
//!
//! Two demodulators run in parallel on the same IQ stream:
//! - **Demodulator** (AM): envelope detection → level/duration pairs for OOK/AM protocols.
//! - **FmDemodulator** (FM): phase discriminator → instantaneous frequency → level/duration for 2FSK.
//!
//! Protocols are tagged AM/FM/Both (ProtoPirate). Captures record which path produced them
//! (`received_rf`). Decoders and encoders are agnostic; TX remains OOK/AM.
//!
//! AM key design: adaptive threshold, hysteresis, debounce, transition-based threshold updates.
//! FM key design: phase diff → freq (Hz), EMA smoothing, zero-centered threshold with hysteresis.

/// A single level+duration pair representing one segment of the signal
#[derive(Debug, Clone, Copy)]
pub struct LevelDuration {
    /// Signal level (true = high, false = low)
    pub level: bool,
    /// Duration in microseconds
    pub duration_us: u32,
}

impl LevelDuration {
    pub fn new(level: bool, duration_us: u32) -> Self {
        Self { level, duration_us }
    }
}

/// Demodulator for processing raw IQ samples into level+duration pairs
pub struct Demodulator {
    /// Sample rate in Hz
    #[allow(dead_code)]
    sample_rate: u32,
    /// Samples per microsecond
    samples_per_us: f64,

    // ── Adaptive threshold with hysteresis ──
    /// Current threshold for high/low detection
    threshold: f32,
    /// Adaptive threshold - high level estimate
    high_level: f32,
    /// Adaptive threshold - low level estimate
    low_level: f32,
    /// Hysteresis (half-width of dead zone around threshold)
    hysteresis: f32,

    // ── Magnitude smoothing ──
    /// Smoothed magnitude (exponential moving average)
    mag_smooth: f32,

    // ── Current confirmed level state ──
    /// Current confirmed signal level (high or low)
    current_level: bool,
    /// Sample count at current confirmed level
    level_sample_count: u64,

    // ── Level magnitude tracking (for transition-based threshold updates) ──
    /// Sum of smoothed magnitudes during the current level period
    level_mag_sum: f64,
    /// Count of samples during the current level period (for averaging)
    level_mag_count: u64,

    // ── Debounce / pending transition ──
    /// Whether we're in a pending transition (unconfirmed level change)
    in_transition: bool,
    /// The level we're tentatively transitioning to
    pending_level: bool,
    /// Sample count accumulated at the pending level
    pending_count: u64,
    /// Sum of smoothed magnitudes during the pending transition
    pending_mag_sum: f64,

    // ── Output and limits ──
    /// Accumulated level+duration pairs
    pairs: Vec<LevelDuration>,
    /// Total samples processed (for adaptive threshold speed)
    total_samples: u64,
    /// Minimum duration to consider valid (in µs) — also debounce threshold
    min_duration_us: u32,
    /// Maximum gap before considering signal complete (in µs)
    max_gap_us: u32,
    /// Samples since last confirmed edge (for gap detection)
    samples_since_edge: u64,
}

impl Demodulator {
    /// Create a new demodulator
    pub fn new(sample_rate: u32) -> Self {
        Self {
            sample_rate,
            samples_per_us: sample_rate as f64 / 1_000_000.0,

            // Start with lower initial threshold — the HackRF's signal levels
            // vary widely depending on gain and distance; starting low ensures
            // we don't miss weak signals during initial adaptation.
            threshold: 0.08,
            high_level: 0.15,
            low_level: 0.02,
            hysteresis: 0.02,

            mag_smooth: 0.0,

            current_level: false,
            level_sample_count: 0,

            level_mag_sum: 0.0,
            level_mag_count: 0,

            in_transition: false,
            pending_level: false,
            pending_count: 0,
            pending_mag_sum: 0.0,

            pairs: Vec::with_capacity(2048),
            total_samples: 0,
            min_duration_us: 40, // 40µs debounce (was 50 — slightly more permissive)
            max_gap_us: 80_000,  // 80ms gap = end of signal; keeps multi-burst keyfob presses (e.g. 3–4 bursts with 25–50ms gaps) as one capture
            samples_since_edge: 0,
        }
    }

    /// Process raw IQ samples and return level+duration pairs if signal complete
    ///
    /// Returns None if still accumulating, Some(pairs) when a complete signal is detected
    pub fn process_samples(&mut self, samples: &[i8]) -> Option<Vec<LevelDuration>> {
        // Process each IQ sample pair
        for chunk in samples.chunks(2) {
            if chunk.len() < 2 {
                continue;
            }

            // Calculate magnitude (AM envelope detection)
            let i = chunk[0] as f32 / 128.0;
            let q = chunk[1] as f32 / 128.0;
            let magnitude = (i * i + q * q).sqrt();

            // Smooth the magnitude with EMA to reduce per-sample noise.
            // Alpha=0.1 gives a time constant of ~10 samples (5µs at 2MHz),
            // which smooths noise without distorting pulse edges.
            self.mag_smooth = self.mag_smooth * 0.9 + magnitude * 0.1;

            // During initial calibration, use fast per-sample threshold updates.
            // After calibration, threshold is updated at transitions (see below)
            // to avoid the duty-cycle bias that causes pulse asymmetry.
            if self.total_samples < 10_000 {
                self.update_threshold_fast(self.mag_smooth);
            }

            // Determine level using hysteresis (Schmitt trigger behavior):
            //   LOW → HIGH requires magnitude > threshold + hysteresis
            //   HIGH → LOW requires magnitude < threshold - hysteresis
            let is_high = if self.current_level {
                // Currently HIGH: stay HIGH unless magnitude drops well below threshold
                self.mag_smooth > (self.threshold - self.hysteresis)
            } else {
                // Currently LOW: go HIGH only if magnitude rises well above threshold
                self.mag_smooth > (self.threshold + self.hysteresis)
            };

            self.total_samples += 1;

            // Track magnitude for the current level period (used for
            // transition-based threshold updates after initial calibration)
            let mag_f64 = self.mag_smooth as f64;

            // ── Debounce state machine ──
            // When we see a level change, we don't immediately commit to it.
            // Instead, we enter a "pending transition" state and wait for the
            // new level to persist for at least min_duration_us. If it flips
            // back sooner, we treat it as noise and absorb it.

            if self.in_transition {
                if is_high == self.pending_level {
                    // Still at the new (pending) level — accumulate
                    self.pending_count += 1;
                    self.pending_mag_sum += mag_f64;
                    let pending_us =
                        (self.pending_count as f64 / self.samples_per_us) as u32;

                    if pending_us >= self.min_duration_us {
                        // Transition confirmed! Update threshold from the
                        // COMPLETED level's average magnitude. This ensures
                        // equal contribution from HIGH and LOW periods
                        // regardless of their duration (no duty-cycle bias).
                        if self.total_samples >= 10_000 && self.level_mag_count > 0 {
                            let avg_mag = (self.level_mag_sum / self.level_mag_count as f64) as f32;
                            self.update_threshold_at_transition(avg_mag, self.current_level);
                        }

                        // Record the previous level's duration.
                        let duration_us =
                            (self.level_sample_count as f64 / self.samples_per_us) as u32;

                        if duration_us >= self.min_duration_us {
                            self.pairs.push(LevelDuration::new(
                                self.current_level,
                                duration_us,
                            ));
                        }

                        self.samples_since_edge = 0;
                        self.current_level = self.pending_level;
                        self.level_sample_count = self.pending_count;
                        // Transfer pending magnitude tracking to current level
                        self.level_mag_sum = self.pending_mag_sum;
                        self.level_mag_count = self.pending_count;
                        self.in_transition = false;
                    }
                } else {
                    // Flipped back before confirmation — this was noise.
                    // Absorb the pending samples back into the current level.
                    self.level_sample_count += self.pending_count + 1;
                    self.level_mag_sum += self.pending_mag_sum + mag_f64;
                    self.level_mag_count += self.pending_count + 1;
                    self.in_transition = false;
                }
            } else if is_high != self.current_level && self.level_sample_count > 0 {
                // Potential new transition — start pending
                self.in_transition = true;
                self.pending_level = is_high;
                self.pending_count = 1;
                self.pending_mag_sum = mag_f64;
            } else {
                // Same level as before, just accumulate
                self.level_sample_count += 1;
                self.level_mag_sum += mag_f64;
                self.level_mag_count += 1;
                self.samples_since_edge += 1;
            }
        }

        // Check if we have a complete signal (long gap detected)
        let gap_samples = (self.max_gap_us as f64 * self.samples_per_us) as u64;

        if !self.pairs.is_empty() && self.samples_since_edge > gap_samples {
            // Flush any pending transition
            if self.in_transition {
                let duration_us =
                    (self.level_sample_count as f64 / self.samples_per_us) as u32;
                if duration_us >= self.min_duration_us {
                    self.pairs
                        .push(LevelDuration::new(self.current_level, duration_us));
                }
                self.level_sample_count = self.pending_count;
                self.current_level = self.pending_level;
                self.in_transition = false;
            }

            // Add the final level duration
            let duration_us =
                (self.level_sample_count as f64 / self.samples_per_us) as u32;
            if duration_us >= self.min_duration_us {
                self.pairs
                    .push(LevelDuration::new(self.current_level, duration_us));
            }

            // Return the pairs and reset (min 5 pairs so short/unknown keyfob bursts still show)
            let result = std::mem::take(&mut self.pairs);
            self.reset_state();

            if result.len() >= 5 {
                return Some(result);
            }
        }

        // Limit buffer size to prevent unbounded growth
        if self.pairs.len() > 4096 {
            self.reset_state();
        }

        None
    }

    /// Fast per-sample threshold update — used only during initial calibration
    /// (first ~5ms / 10K samples). Updates high/low estimates every sample for
    /// quick convergence to the signal's dynamic range.
    fn update_threshold_fast(&mut self, magnitude: f32) {
        let alpha: f32 = 0.01;

        if magnitude > self.threshold {
            self.high_level = self.high_level * (1.0 - alpha) + magnitude * alpha;
        } else {
            self.low_level = self.low_level * (1.0 - alpha) + magnitude * alpha;
        }

        self.recalc_threshold();
    }

    /// Transition-based threshold update — used after initial calibration.
    ///
    /// Called once per confirmed level transition with the AVERAGE magnitude
    /// of the completed level period. This eliminates the duty-cycle bias
    /// that per-sample updates cause: a 500µs HIGH and a 100µs LOW now
    /// contribute equally to their respective level estimates, producing
    /// symmetric pulse widths in the demodulated output.
    ///
    /// Alpha=0.3 provides fast convergence (~5 pulses / 10 transitions to
    /// reach 97% of the correct threshold). This is critical because after
    /// a long silence, high_level starts at a stale initial guess and must
    /// converge before the data section begins. With alpha=0.05 it took ~50
    /// transitions (entire preamble + data), causing massive pulse asymmetry.
    fn update_threshold_at_transition(&mut self, avg_magnitude: f32, was_high: bool) {
        let alpha: f32 = 0.3; // Fast convergence — one update per transition

        if was_high {
            self.high_level = self.high_level * (1.0 - alpha) + avg_magnitude * alpha;
        } else {
            self.low_level = self.low_level * (1.0 - alpha) + avg_magnitude * alpha;
        }

        self.recalc_threshold();
    }

    /// Recalculate threshold and hysteresis from current high/low estimates.
    fn recalc_threshold(&mut self) {
        // Threshold is midpoint between low and high estimates
        self.threshold = (self.low_level + self.high_level) / 2.0;

        // Ensure reasonable bounds — very low threshold for weak signals,
        // but not so low that ADC noise alone triggers it
        self.threshold = self.threshold.max(0.02).min(0.5);

        // Dynamic hysteresis: 10% of the estimated signal-noise gap, clamped to [0.01, 0.08].
        // This prevents chattering near the threshold while allowing clean transitions
        // for both strong and weak signals.
        self.hysteresis = ((self.high_level - self.low_level) * 0.10)
            .max(0.01)
            .min(0.08);
    }

    /// Reset the demodulator state (keeps threshold adaptation)
    fn reset_state(&mut self) {
        self.pairs.clear();
        self.level_sample_count = 0;
        self.level_mag_sum = 0.0;
        self.level_mag_count = 0;
        self.samples_since_edge = 0;
        self.current_level = false;
        self.in_transition = false;
        self.pending_level = false;
        self.pending_count = 0;
        self.pending_mag_sum = 0.0;
    }

    /// Reset completely (including threshold adaptation)
    #[allow(dead_code)]
    pub fn reset(&mut self) {
        self.reset_state();
        self.threshold = 0.08;
        self.high_level = 0.15;
        self.low_level = 0.02;
        self.hysteresis = 0.02;
        self.mag_smooth = 0.0;
        self.total_samples = 0;
    }
}

// ─── FM / 2FSK demodulator (phase discriminator) ─────────────────────────────

/// FM/2FSK demodulator: instantaneous frequency from phase difference → level/duration pairs.
/// Uses same debounce and gap-detection logic as AM so protocol decoders see a consistent stream.
pub struct FmDemodulator {
    sample_rate: u32,
    samples_per_us: f64,

    /// Previous I,Q (normalized) for phase-diff
    prev_i: f32,
    prev_q: f32,
    /// Have we seen at least one previous sample?
    have_prev: bool,

    /// EMA of instantaneous frequency (Hz)
    freq_smooth: f32,
    /// Threshold (Hz): above = high, below = low. Zero for symmetric 2FSK.
    threshold: f32,
    hysteresis: f32,

    current_level: bool,
    level_sample_count: u64,
    in_transition: bool,
    pending_level: bool,
    pending_count: u64,

    pairs: Vec<LevelDuration>,
    min_duration_us: u32,
    max_gap_us: u32,
    samples_since_edge: u64,
}

impl FmDemodulator {
    /// Create a new FM demodulator. Hysteresis in Hz (e.g. 300–1000 for keyfob 2FSK).
    pub fn new(sample_rate: u32) -> Self {
        Self {
            sample_rate,
            samples_per_us: sample_rate as f64 / 1_000_000.0,
            prev_i: 0.0,
            prev_q: 0.0,
            have_prev: false,
            freq_smooth: 0.0,
            threshold: 0.0,
            hysteresis: 500.0, // Hz; keyfob 2FSK deviation typically a few kHz
            current_level: false,
            level_sample_count: 0,
            in_transition: false,
            pending_level: false,
            pending_count: 0,
            pairs: Vec::with_capacity(2048),
            min_duration_us: 40,
            max_gap_us: 80_000, // match AM: 80ms so one button press (multi-burst) stays one capture
            samples_since_edge: 0,
        }
    }

    /// Process raw IQ samples; returns `Some(pairs)` when a complete signal is detected (gap).
    pub fn process_samples(&mut self, samples: &[i8]) -> Option<Vec<LevelDuration>> {
        let two_pi = std::f32::consts::TAU;
        let rad_to_hz = self.sample_rate as f32 / two_pi;

        for chunk in samples.chunks(2) {
            if chunk.len() < 2 {
                continue;
            }
            let i = chunk[0] as f32 / 128.0;
            let q = chunk[1] as f32 / 128.0;

            if !self.have_prev {
                self.prev_i = i;
                self.prev_q = q;
                self.have_prev = true;
                continue;
            }

            // Phase difference: atan2(Im(c*conj(c_prev)), Re(c*conj(c_prev)))
            let re = i * self.prev_i + q * self.prev_q;
            let im = q * self.prev_i - i * self.prev_q;
            let phase_diff = im.atan2(re);
            self.prev_i = i;
            self.prev_q = q;

            // Instantaneous frequency (Hz)
            let freq_hz = phase_diff * rad_to_hz;
            // EMA smoothing (alpha ≈ 0.1)
            self.freq_smooth = self.freq_smooth * 0.9 + freq_hz * 0.1;

            let is_high = if self.current_level {
                self.freq_smooth > (self.threshold - self.hysteresis)
            } else {
                self.freq_smooth > (self.threshold + self.hysteresis)
            };

            if self.in_transition {
                if is_high == self.pending_level {
                    self.pending_count += 1;
                    let pending_us = (self.pending_count as f64 / self.samples_per_us) as u32;
                    if pending_us >= self.min_duration_us {
                        let duration_us =
                            (self.level_sample_count as f64 / self.samples_per_us) as u32;
                        if duration_us >= self.min_duration_us {
                            self.pairs.push(LevelDuration::new(self.current_level, duration_us));
                        }
                        self.samples_since_edge = 0;
                        self.current_level = self.pending_level;
                        self.level_sample_count = self.pending_count;
                        self.in_transition = false;
                    }
                } else {
                    self.level_sample_count += self.pending_count + 1;
                    self.in_transition = false;
                }
            } else if is_high != self.current_level && self.level_sample_count > 0 {
                self.in_transition = true;
                self.pending_level = is_high;
                self.pending_count = 1;
            } else {
                self.level_sample_count += 1;
                self.samples_since_edge += 1;
            }
        }

        let gap_samples = (self.max_gap_us as f64 * self.samples_per_us) as u64;
        if !self.pairs.is_empty() && self.samples_since_edge > gap_samples {
            if self.in_transition {
                let duration_us =
                    (self.level_sample_count as f64 / self.samples_per_us) as u32;
                if duration_us >= self.min_duration_us {
                    self.pairs.push(LevelDuration::new(self.current_level, duration_us));
                }
                self.level_sample_count = self.pending_count;
                self.current_level = self.pending_level;
                self.in_transition = false;
            }
            let duration_us =
                (self.level_sample_count as f64 / self.samples_per_us) as u32;
            if duration_us >= self.min_duration_us {
                self.pairs.push(LevelDuration::new(self.current_level, duration_us));
            }
            let result = std::mem::take(&mut self.pairs);
            self.fm_reset_state();
            if result.len() >= 5 {
                return Some(result);
            }
        }

        if self.pairs.len() > 4096 {
            self.fm_reset_state();
        }
        None
    }

    fn fm_reset_state(&mut self) {
        self.pairs.clear();
        self.level_sample_count = 0;
        self.samples_since_edge = 0;
        self.current_level = false;
        self.in_transition = false;
        self.pending_level = false;
        self.pending_count = 0;
    }
}

// Note: duration_diff macro is defined in protocols/mod.rs

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_demodulator_creation() {
        let demod = Demodulator::new(2_000_000);
        assert_eq!(demod.sample_rate, 2_000_000);
    }

    #[test]
    fn test_level_duration() {
        let ld = LevelDuration::new(true, 500);
        assert!(ld.level);
        assert_eq!(ld.duration_us, 500);
    }

    #[test]
    fn test_no_consecutive_same_levels() {
        // Simulate a signal with a noise spike in the middle of a LOW period.
        // The debounce should absorb the spike and never produce consecutive same-level pairs.
        let mut demod = Demodulator::new(2_000_000);
        // At 2MHz, 1 sample = 0.5µs.
        // Create a buffer: 200µs LOW, 20µs HIGH spike (noise), 200µs LOW, 200µs HIGH, 200µs LOW
        // 200µs = 400 samples, 20µs = 40 samples
        let mut buf = Vec::new();
        // LOW: magnitude ≈ 0.01
        for _ in 0..400 { buf.push(1i8); buf.push(0i8); }
        // Brief HIGH spike: magnitude ≈ 0.9
        for _ in 0..40 { buf.push(115i8); buf.push(0i8); }
        // LOW again
        for _ in 0..400 { buf.push(1i8); buf.push(0i8); }
        // Real HIGH
        for _ in 0..400 { buf.push(115i8); buf.push(0i8); }
        // LOW
        for _ in 0..400 { buf.push(1i8); buf.push(0i8); }

        // Process (won't return signal yet since no long gap)
        let _ = demod.process_samples(&buf);

        // Add a long gap to flush (>= max_gap_us: 80ms at 2MHz = 160k samples)
        let gap_buf: Vec<i8> = vec![1, 0].repeat(80_000); // 80ms LOW
        if let Some(pairs) = demod.process_samples(&gap_buf) {
            // Verify no consecutive same-level pairs
            for window in pairs.windows(2) {
                if window[0].level == window[1].level {
                    panic!(
                        "Found consecutive same-level pairs: {:?} and {:?}",
                        window[0], window[1]
                    );
                }
            }
        }
    }
}
