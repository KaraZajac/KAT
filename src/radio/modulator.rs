//! Signal modulator for generating TX waveforms.

use super::demodulator::LevelDuration;

/// Modulator for generating transmission waveforms
#[allow(dead_code)]
pub struct Modulator {
    /// Time element in microseconds (base timing unit)
    pub te: u32,
}

#[allow(dead_code)]
impl Modulator {
    /// Create a new modulator with the given time element
    pub fn new(te: u32) -> Self {
        Self { te }
    }

    /// Generate a preamble (alternating pattern)
    pub fn generate_preamble(&self, count: usize) -> Vec<LevelDuration> {
        let mut result = Vec::with_capacity(count * 2);
        for _ in 0..count {
            result.push(LevelDuration::new(true, self.te));
            result.push(LevelDuration::new(false, self.te));
        }
        result
    }

    /// Generate a sync pattern
    pub fn generate_sync(&self, high_te: u32, low_te: u32) -> Vec<LevelDuration> {
        vec![
            LevelDuration::new(true, self.te * high_te),
            LevelDuration::new(false, self.te * low_te),
        ]
    }

    /// Encode data using PWM (Pulse Width Modulation)
    /// bit 0: short high, long low
    /// bit 1: long high, short low
    pub fn encode_pwm(&self, data: &[u8], bit_count: usize) -> Vec<LevelDuration> {
        let mut result = Vec::with_capacity(bit_count * 2);

        for i in 0..bit_count {
            let byte_idx = i / 8;
            let bit_idx = 7 - (i % 8);
            let bit = (data[byte_idx] >> bit_idx) & 1;

            if bit == 0 {
                result.push(LevelDuration::new(true, self.te));
                result.push(LevelDuration::new(false, self.te * 3));
            } else {
                result.push(LevelDuration::new(true, self.te * 3));
                result.push(LevelDuration::new(false, self.te));
            }
        }

        result
    }

    /// Encode data using Manchester encoding
    /// bit 0: high then low
    /// bit 1: low then high
    pub fn encode_manchester(&self, data: &[u8], bit_count: usize) -> Vec<LevelDuration> {
        let mut result = Vec::with_capacity(bit_count * 2);

        for i in 0..bit_count {
            let byte_idx = i / 8;
            let bit_idx = 7 - (i % 8);
            let bit = (data[byte_idx] >> bit_idx) & 1;

            if bit == 0 {
                result.push(LevelDuration::new(true, self.te));
                result.push(LevelDuration::new(false, self.te));
            } else {
                result.push(LevelDuration::new(false, self.te));
                result.push(LevelDuration::new(true, self.te));
            }
        }

        result
    }

    /// Encode data using inverted Manchester encoding
    /// bit 0: low then high
    /// bit 1: high then low
    pub fn encode_manchester_inverted(&self, data: &[u8], bit_count: usize) -> Vec<LevelDuration> {
        let mut result = Vec::with_capacity(bit_count * 2);

        for i in 0..bit_count {
            let byte_idx = i / 8;
            let bit_idx = 7 - (i % 8);
            let bit = (data[byte_idx] >> bit_idx) & 1;

            if bit == 0 {
                result.push(LevelDuration::new(false, self.te));
                result.push(LevelDuration::new(true, self.te));
            } else {
                result.push(LevelDuration::new(true, self.te));
                result.push(LevelDuration::new(false, self.te));
            }
        }

        result
    }

    /// Generate a trailer (final low period)
    pub fn generate_trailer(&self, te_count: u32) -> Vec<LevelDuration> {
        vec![LevelDuration::new(false, self.te * te_count)]
    }

    /// Combine multiple signal parts into one
    pub fn combine(parts: Vec<Vec<LevelDuration>>) -> Vec<LevelDuration> {
        parts.into_iter().flatten().collect()
    }

    /// Repeat a signal pattern multiple times
    pub fn repeat(signal: &[LevelDuration], count: usize) -> Vec<LevelDuration> {
        let mut result = Vec::with_capacity(signal.len() * count);
        for _ in 0..count {
            result.extend_from_slice(signal);
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pwm_encoding() {
        let mod_ = Modulator::new(400);
        let data = vec![0b10101010];
        let encoded = mod_.encode_pwm(&data, 8);

        assert_eq!(encoded.len(), 16);
    }

    #[test]
    fn test_manchester_encoding() {
        let mod_ = Modulator::new(400);
        let data = vec![0b10101010];
        let encoded = mod_.encode_manchester(&data, 8);

        assert_eq!(encoded.len(), 16);
    }
}
