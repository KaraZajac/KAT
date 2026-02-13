//! Protocol decoders and encoders for various keyfob systems.
//!
//! Protocols are aligned with the ProtoPirate reference (`REFERENCES/ProtoPirate/protocols/`).
//! Each decoder processes level+duration pairs from the demodulator and optionally supports
//! encoding (replay). Shared pieces: [common], [keeloq_common], [keys], [aut64].
//!
//! **Manchester decoding**: Each protocol that uses Manchester has its own state machine and
//! event mapping (no shared global decoder). Polarity and event conventions match the
//! reference per protocol (e.g. Kia V5 uses opposite polarity to V1/V2; Kia V6 level
//! convention; Fiat/Ford/VAG use level ? ShortLow : ShortHigh).

mod common;
pub mod keeloq_common;
#[allow(dead_code)]
pub mod aut64;
#[allow(dead_code)]
pub mod keys;
mod kia_v0;
mod kia_v1;
mod kia_v2;
mod kia_v3_v4;
mod kia_v5;
mod kia_v6;
mod subaru;
mod ford_v0;
mod vag;
mod fiat_v0;
mod suzuki;
mod scher_khan;
mod star_line;
mod psa;

pub use common::DecodedSignal;

use crate::capture::Capture;
use crate::radio::demodulator::LevelDuration;

/// Protocol timing constants
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
pub struct ProtocolTiming {
    /// Short pulse duration in µs
    pub te_short: u32,
    /// Long pulse duration in µs
    pub te_long: u32,
    /// Tolerance for timing matching in µs
    pub te_delta: u32,
    /// Minimum bit count for valid decode
    pub min_count_bit: usize,
}

/// Trait for protocol decoders
/// 
/// Each protocol implements a state machine that processes level+duration pairs.
pub trait ProtocolDecoder: Send + Sync {
    /// Get the protocol name
    fn name(&self) -> &'static str;

    /// Get timing constants
    #[allow(dead_code)]
    fn timing(&self) -> ProtocolTiming;

    /// Get supported frequencies in Hz
    fn supported_frequencies(&self) -> &[u32];

    /// Reset the decoder state machine
    fn reset(&mut self);

    /// Feed a level+duration pair to the decoder
    /// Returns Some(DecodedSignal) when a complete valid signal is decoded
    fn feed(&mut self, level: bool, duration_us: u32) -> Option<DecodedSignal>;

    /// Check if this protocol supports encoding
    fn supports_encoding(&self) -> bool;

    /// Encode a signal with the given button command
    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>>;
}

/// Registry of all supported protocols
pub struct ProtocolRegistry {
    decoders: Vec<Box<dyn ProtocolDecoder>>,
}

impl ProtocolRegistry {
    /// Create a new protocol registry with all built-in protocols
    pub fn new() -> Self {
        let decoders: Vec<Box<dyn ProtocolDecoder>> = vec![
            // Kia protocols
            Box::new(kia_v0::KiaV0Decoder::new()),
            Box::new(kia_v1::KiaV1Decoder::new()),
            Box::new(kia_v2::KiaV2Decoder::new()),
            Box::new(kia_v3_v4::KiaV3V4Decoder::new()),
            Box::new(kia_v5::KiaV5Decoder::new()),
            Box::new(kia_v6::KiaV6Decoder::new()),
            // Other protocols
            Box::new(subaru::SubaruDecoder::new()),
            Box::new(ford_v0::FordV0Decoder::new()),
            Box::new(vag::VagDecoder::new()),
            Box::new(fiat_v0::FiatV0Decoder::new()),
            Box::new(suzuki::SuzukiDecoder::new()),
            Box::new(scher_khan::ScherKhanDecoder::new()),
            Box::new(star_line::StarLineDecoder::new()),
            Box::new(psa::PsaDecoder::new()),
        ];

        Self { decoders }
    }

    /// Process level+duration pairs from demodulator
    /// Returns decoded signal info if any protocol matches
    pub fn process_signal(&mut self, pairs: &[LevelDuration], frequency: u32) -> Option<(String, DecodedSignal)> {
        // Reset all decoders
        for decoder in &mut self.decoders {
            decoder.reset();
        }

        // Feed pairs to all decoders that support this frequency
        for pair in pairs {
            for decoder in &mut self.decoders {
                // Check frequency support
                let freq_supported = decoder
                    .supported_frequencies()
                    .iter()
                    .any(|&f| {
                        let diff = if f > frequency { f - frequency } else { frequency - f };
                        diff < (f / 50) // 2% tolerance
                    });

                if !freq_supported {
                    continue;
                }

                if let Some(decoded) = decoder.feed(pair.level, pair.duration_us) {
                    return Some((decoder.name().to_string(), decoded));
                }
            }
        }

        None
    }

    /// Try to decode a capture (for compatibility with old interface)
    #[allow(dead_code)]
    pub fn try_decode(&mut self, capture: &Capture) -> Option<(String, DecodedSignal)> {
        // Convert raw pairs to LevelDuration and process
        if capture.raw_pairs.is_empty() {
            return None;
        }

        let pairs: Vec<LevelDuration> = capture.raw_pairs
            .iter()
            .map(|p| LevelDuration::new(p.level, p.duration_us))
            .collect();

        self.process_signal(&pairs, capture.frequency)
    }

    /// Get a decoder by name
    pub fn get(&self, name: &str) -> Option<&dyn ProtocolDecoder> {
        self.decoders
            .iter()
            .find(|d| d.name().eq_ignore_ascii_case(name))
            .map(|d| d.as_ref())
    }

    /// List all protocol names
    #[allow(dead_code)]
    pub fn list_protocols(&self) -> Vec<&'static str> {
        self.decoders.iter().map(|d| d.name()).collect()
    }
}

impl Default for ProtocolRegistry {
    fn default() -> Self {
        Self::new()
    }
}

/// Helper macro for duration comparison (matches protopirate's DURATION_DIFF)
#[macro_export]
macro_rules! duration_diff {
    ($actual:expr, $expected:expr) => {
        if $actual > $expected {
            $actual - $expected
        } else {
            $expected - $actual
        }
    };
}
