//! AM/OOK demodulator for extracting level+duration pairs from raw IQ samples.
//!
//! This demodulator converts raw IQ samples into a stream of (level, duration_us) pairs
//! that can be processed by protocol decoders, similar to how the Flipper Zero SubGHz
//! system works.

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
    /// Current threshold for high/low detection
    threshold: f32,
    /// Adaptive threshold - high level estimate
    high_level: f32,
    /// Adaptive threshold - low level estimate  
    low_level: f32,
    /// Current signal state (high or low)
    current_level: bool,
    /// Sample count at current level
    level_sample_count: u64,
    /// Accumulated level+duration pairs
    pairs: Vec<LevelDuration>,
    /// Total samples processed
    total_samples: u64,
    /// Minimum duration to consider valid (in µs)
    min_duration_us: u32,
    /// Maximum gap before considering signal complete (in µs)
    max_gap_us: u32,
    /// Samples since last edge
    samples_since_edge: u64,
}

impl Demodulator {
    /// Create a new demodulator
    pub fn new(sample_rate: u32) -> Self {
        Self {
            sample_rate,
            samples_per_us: sample_rate as f64 / 1_000_000.0,
            threshold: 0.15,
            high_level: 0.3,
            low_level: 0.05,
            current_level: false,
            level_sample_count: 0,
            pairs: Vec::with_capacity(2048),
            total_samples: 0,
            min_duration_us: 50,    // Minimum 50µs pulse
            max_gap_us: 10_000,     // 10ms gap = end of signal
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

            // Update adaptive threshold
            self.update_threshold(magnitude);

            // Detect level
            let is_high = magnitude > self.threshold;

            // Check for level change
            if is_high != self.current_level && self.level_sample_count > 0 {
                // Calculate duration of the previous level
                let duration_us = (self.level_sample_count as f64 / self.samples_per_us) as u32;

                // Only record if above minimum duration (noise filtering)
                if duration_us >= self.min_duration_us {
                    self.pairs.push(LevelDuration::new(self.current_level, duration_us));
                    self.samples_since_edge = 0;
                }

                self.current_level = is_high;
                self.level_sample_count = 1;
            } else {
                self.level_sample_count += 1;
                self.samples_since_edge += 1;
            }

            self.total_samples += 1;
        }

        // Check if we have a complete signal (long gap detected)
        let gap_samples = (self.max_gap_us as f64 * self.samples_per_us) as u64;
        
        if !self.pairs.is_empty() && self.samples_since_edge > gap_samples {
            // Add the final level duration
            let duration_us = (self.level_sample_count as f64 / self.samples_per_us) as u32;
            if duration_us >= self.min_duration_us {
                self.pairs.push(LevelDuration::new(self.current_level, duration_us));
            }

            // Return the pairs and reset
            let result = std::mem::take(&mut self.pairs);
            self.reset_state();
            
            if result.len() >= 10 {
                return Some(result);
            }
        }

        // Limit buffer size
        if self.pairs.len() > 4096 {
            self.reset_state();
        }

        None
    }

    /// Update adaptive threshold based on signal levels
    fn update_threshold(&mut self, magnitude: f32) {
        const ALPHA: f32 = 0.001; // Slow adaptation

        if magnitude > self.threshold {
            // Update high level estimate
            self.high_level = self.high_level * (1.0 - ALPHA) + magnitude * ALPHA;
        } else {
            // Update low level estimate
            self.low_level = self.low_level * (1.0 - ALPHA) + magnitude * ALPHA;
        }

        // Threshold is midpoint between low and high
        self.threshold = (self.low_level + self.high_level) / 2.0;
        
        // Ensure reasonable bounds
        self.threshold = self.threshold.max(0.05).min(0.5);
    }

    /// Reset the demodulator state
    fn reset_state(&mut self) {
        self.pairs.clear();
        self.level_sample_count = 0;
        self.samples_since_edge = 0;
        self.current_level = false;
    }

    /// Reset completely (including threshold adaptation)
    #[allow(dead_code)]
    pub fn reset(&mut self) {
        self.reset_state();
        self.threshold = 0.15;
        self.high_level = 0.3;
        self.low_level = 0.05;
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
}
