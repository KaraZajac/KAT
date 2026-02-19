//! Protocol decoders and encoders for various keyfob systems.
//!
//! Protocols are aligned with the ProtoPirate reference (`REFERENCES/ProtoPirate/protocols/`).
//! Each decoder processes level+duration pairs from the demodulator and optionally supports
//! encoding (replay). Shared pieces: [common], [keeloq_common], [keys], [aut64].
//!
//! **Decoder selection (vs ProtoPirate)**  
//! ProtoPirate calls `subghz_receiver_decode(receiver, level, duration)` for each pulse; the
//! Flipper SDK receiver (not in REFERENCES) feeds all registered decoders. There is no
//! preamble-based decoder selection in the scene—only the decoder's own feed() logic (e.g. VAG
//! Reset/Preamble1/Preamble2). We do the same: feed every pulse to all decoders that support the
//! file frequency; whoever returns a valid frame is reported. No extra preamble filtering.
//!
//! **Manchester decoding**: Ford, Fiat, and common each have separate Manchester state machines
//! (FordV0ManchesterState, FiatV0ManchesterState, CommonManchesterState in common.rs). They are
//! not reused across protocols. Event conventions match the reference per protocol (e.g. Kia V5
//! opposite polarity; Fiat/Ford/common use Flipper-style: level ? ShortLow : ShortHigh).

mod common;
pub mod keeloq_common;
mod keeloq;
mod keeloq_generic;
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
            // VAG before Ford/Subaru so 500/1000µs VAG streams decode as VAG (ProtoPirate order has VAG after Ford/Subaru but Flipper likely feeds all decoders; KAT uses first-match so VAG must be tried earlier)
            Box::new(vag::VagDecoder::new()),
            Box::new(ford_v0::FordV0Decoder::new()),
            Box::new(subaru::SubaruDecoder::new()),
            Box::new(fiat_v0::FiatV0Decoder::new()),
            Box::new(suzuki::SuzukiDecoder::new()),
            Box::new(scher_khan::ScherKhanDecoder::new()),
            Box::new(star_line::StarLineDecoder::new()),
            Box::new(keeloq::KeeloqDecoder::new()),
            Box::new(psa::PsaDecoder::new()),
        ];

        Self { decoders }
    }

    /// Process level+duration pairs from demodulator
    /// Returns decoded signal info if any protocol matches.
    /// Tries normal polarity first, then inverted polarity (so OOK captures where
    /// carrier-on is recorded as LOW can still decode as Fiat/Ford etc.).
    pub fn process_signal(&mut self, pairs: &[LevelDuration], frequency: u32) -> Option<(String, DecodedSignal)> {
        // Try normal polarity first
        if let Some(result) = self.process_signal_inner(pairs, frequency, false) {
            return Some(result);
        }
        // Try inverted polarity (capture LOW = RF HIGH)
        if let Some(result) = self.process_signal_inner(pairs, frequency, true) {
            return Some(result);
        }
        // No known protocol: try KeeLoq generic (uses keeloq_common with every keystore key)
        keeloq_generic::try_decode(pairs, frequency)
    }

    /// ProtoPirate-style streaming decode: feed the whole stream, on each decode record and reset decoders, continue.
    /// Returns one entry per decode: (protocol name, decoded signal, pairs that produced it).
    /// Tries normal polarity first; if no decodes, runs again with inverted polarity.
    pub fn process_signal_stream(
        &mut self,
        pairs: &[LevelDuration],
        frequency: u32,
    ) -> Vec<(String, DecodedSignal, Vec<LevelDuration>)> {
        let with_normal = self.process_signal_stream_inner(pairs, frequency, false);
        if !with_normal.is_empty() {
            return with_normal;
        }
        self.process_signal_stream_inner(pairs, frequency, true)
    }

    /// Inner streaming decode with optional level inversion.
    fn process_signal_stream_inner(
        &mut self,
        pairs: &[LevelDuration],
        frequency: u32,
        invert_level: bool,
    ) -> Vec<(String, DecodedSignal, Vec<LevelDuration>)> {
        let mut out = Vec::new();
        let mut segment_start = 0_usize;

        for decoder in &mut self.decoders {
            decoder.reset();
        }

        for (i, pair) in pairs.iter().enumerate() {
            let level = if invert_level { !pair.level } else { pair.level };
            let duration_us = pair.duration_us;

            // Feed this pulse to all decoders that support this frequency (Flipper-style).
            // Whoever actually produces a valid frame is reported; decoder order no longer decides.
            let mut hits: Vec<(String, DecodedSignal)> = Vec::new();
            for decoder in &mut self.decoders {
                let freq_supported = decoder
                    .supported_frequencies()
                    .iter()
                    .any(|&f| {
                        let diff = if f > frequency { f - frequency } else { frequency - f };
                        diff < (f / 50)
                    });
                if !freq_supported {
                    continue;
                }
                if let Some(decoded) = decoder.feed(level, duration_us) {
                    let name = decoded
                        .protocol_display_name
                        .as_deref()
                        .unwrap_or_else(|| decoder.name());
                    hits.push((name.to_string(), decoded));
                }
            }
            if let Some((name, decoded)) = hits.into_iter().next() {
                let segment: Vec<LevelDuration> = pairs[segment_start..=i]
                    .iter()
                    .map(|p| LevelDuration::new(p.level, p.duration_us))
                    .collect();
                out.push((name, decoded, segment));
                for d in &mut self.decoders {
                    d.reset();
                }
                segment_start = i + 1;
            }
        }

        out
    }

    /// Inner decode: feed pairs (with optional level flip) to decoders that support this frequency.
    fn process_signal_inner(
        &mut self,
        pairs: &[LevelDuration],
        frequency: u32,
        invert_level: bool,
    ) -> Option<(String, DecodedSignal)> {
        for decoder in &mut self.decoders {
            decoder.reset();
        }

        for pair in pairs {
            let level = if invert_level { !pair.level } else { pair.level };
            let duration_us = pair.duration_us;

            // Feed this pulse to all decoders; report first valid frame (Flipper-style).
            let mut hits: Vec<(String, DecodedSignal)> = Vec::new();
            for decoder in &mut self.decoders {
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

                if let Some(decoded) = decoder.feed(level, duration_us) {
                    let name = decoded
                        .protocol_display_name
                        .as_deref()
                        .unwrap_or_else(|| decoder.name());
                    hits.push((name.to_string(), decoded));
                }
            }
            if let Some((name, decoded)) = hits.into_iter().next() {
                return Some((name.to_string(), decoded));
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
        let lookup = if name.starts_with("KeeLoq") {
            "KeeLoq"
        } else {
            name
        };
        self.decoders
            .iter()
            .find(|d| d.name().eq_ignore_ascii_case(lookup))
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::export::flipper::import_sub_raw;
    use crate::radio::LevelDuration;
    use std::path::Path;

    #[test]
    fn ford_v0_decodes_imports_ford_unlock_sub() {
        let path = Path::new("IMPORTS/FORD/3_unlock_ford.sub");
        if !path.exists() {
            eprintln!("Skip: {:?} not found (run from crate root)", path);
            return;
        }
        let (freq, raw_pairs) = import_sub_raw(path).unwrap();
        let pairs: Vec<LevelDuration> = raw_pairs
            .iter()
            .map(|p| LevelDuration::new(p.level, p.duration_us))
            .collect();
        let mut reg = ProtocolRegistry::new();
        let results = reg.process_signal_stream(&pairs, freq);
        let ford_decodes: Vec<_> = results.iter().filter(|(name, _, _)| *name == "Ford V0").collect();
        assert!(
            !ford_decodes.is_empty(),
            "expected at least one Ford V0 decode from 3_unlock_ford.sub, got: {:?}",
            results.iter().map(|(n, _, _)| n.as_str()).collect::<Vec<_>>()
        );
    }
}
