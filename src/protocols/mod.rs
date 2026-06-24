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
mod keeloq_barriers;
pub use keeloq_barriers::is_keeloq_non_car;
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
mod kia_v7;
mod subaru;
mod ford_v0;
mod ford_v1;
mod ford_v2;
mod ford_v3;
mod honda_static;
mod honda_v1;
mod vag;
mod fiat_v0;
mod fiat_v1;
mod suzuki;
mod scher_khan;
mod star_line;
mod psa;
mod psa2;
mod chrysler_v0;
mod mazda_v0;
mod mazda_siemens;
mod mitsubishi_v0;
mod porsche_touareg;
mod porsche_cayenne;
mod bmw_cas4;
mod land_rover_v0;
mod land_rover_rke;
mod toyota;

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
            Box::new(kia_v7::KiaV7Decoder::new()),
            // VAG before Ford/Subaru so 500/1000µs VAG streams decode as VAG (ProtoPirate order has VAG after Ford/Subaru but Flipper likely feeds all decoders; KAT uses first-match so VAG must be tried earlier)
            Box::new(vag::VagDecoder::new()),
            Box::new(ford_v0::FordV0Decoder::new()),
            // Ford V1: distinct 65/130µs Manchester (vs V0 250/500, V2 200/400, V3 240/480) and
            // CRC16-gated, so placement in the Ford group is safe (won't steal V0/V2/V3 captures).
            Box::new(ford_v1::FordV1Decoder::new()),
            Box::new(ford_v3::FordV3Decoder::new()),
            Box::new(ford_v2::FordV2Decoder::new()),
            // Honda Static after the Kia/Ford block; checksum-gated emission makes order safe.
            Box::new(honda_static::HondaStaticDecoder::new()),
            // Honda V1: distinct 1000/2000µs PWM (vs Honda Static's 63µs Manchester). Button-gated
            // emission makes order safe.
            Box::new(honda_v1::HondaV1Decoder::new()),
            Box::new(subaru::SubaruDecoder::new()),
            Box::new(fiat_v0::FiatV0Decoder::new()),
            Box::new(fiat_v1::FiatV1Decoder::new()),
            Box::new(suzuki::SuzukiDecoder::new()),
            Box::new(scher_khan::ScherKhanDecoder::new()),
            Box::new(star_line::StarLineDecoder::new()),
            Box::new(keeloq::KeeloqDecoder::new()),
            Box::new(psa::PsaDecoder::new()),
            // PSA2 (Flipper-ARF, internal name "PSA OLD"): the OLDER PSA variant. Manchester
            // 250/500µs (or 125/250µs half-rate), 128-bit frame (key1 64 + validation 16), AM/OOK,
            // TEA cipher with a mode23 XOR fast-path validated by a nibble checksum (the live
            // decoder runs ONLY this O(1) path — the dual TEA brute force is reserved for offline
            // decrypt and never runs in feed(), exactly as in the C). Placed AFTER `psa` so the
            // existing PSA keeps first-match priority and PSA2 acts as the older fallback. Emission
            // is gated on the XOR checksum (or validation nibble == 0xA), so it false-matches
            // nothing else.
            Box::new(psa2::Psa2Decoder::new()),
            // Chrysler V0 (300/3400-3700µs PWM, dual-long symbols). Timing is unique and emission
            // is gated on the frame's own structural check, so placement here is low-risk.
            Box::new(chrysler_v0::ChryslerV0Decoder::new()),
            // Land Rover V0 (differential Manchester 250/500µs, 81 bits). Emission is gated on a
            // 3-bit check polynomial + 16-bit tail + zero reserved bits, so placement is safe.
            Box::new(land_rover_v0::LandRoverV0Decoder::new()),
            // Land Rover RKE (Flipper-ARF): fixed-width PWM (700/300µs bits), 20-pulse preamble,
            // a distinctive 400µs+9600µs sync gap, and exactly 66 bits. KeeLoq hop is left
            // encrypted (no key), so emission is gated on the preamble + 9.6ms sync + 66-bit
            // strict-PWM geometry — unique among KAT protocols, so it false-matches nothing.
            Box::new(land_rover_rke::LandRoverRkeDecoder::new()),
            Box::new(mazda_v0::MazdaV0Decoder::new()),
            // Mazda Siemens (Flipper-ARF): the Siemens/VDO keyfob cipher. It rides the SAME
            // 250/500µs pair-based stream as Mazda V0 and uses the same additive-checksum gate,
            // so the two would match an identical set of frames. Placed AFTER mazda_v0 so Mazda
            // V0 keeps first-match priority and Mazda Siemens can never steal its captures.
            // Emission is gated on the structural preamble/sync/bit-count + checksum, so it does
            // not false-match other 250/500µs Manchester protocols. Adds an encoder (Mazda V0
            // has none).
            Box::new(mazda_siemens::MazdaSiemensDecoder::new()),
            // BMW CAS4 (Flipper-ARF): Manchester 500/1000µs, 64-bit, AM. The CAS4 rolling cipher's
            // manufacturer key is unavailable, so the payload is left encrypted — emission is gated
            // on the two fixed marker bytes (byte[0]==0x30 && byte[6]==0xC5), which is unique and
            // makes it false-match nothing. Decode-only (the reference encoder is a stub). Marker-
            // gated, so registry order here is safe.
            Box::new(bmw_cas4::BmwCas4Decoder::new()),
            Box::new(mitsubishi_v0::MitsubishiV0Decoder::new()),
            Box::new(porsche_touareg::PorscheTouaregDecoder::new()),
            // Porsche Cayenne (Flipper-ARF, internal name "Porsche AG"): the SAME wire protocol
            // as Porsche Touareg — Touareg is itself a port of this very porsche_cayenne.c source,
            // so they share the identical 1680/3370µs PWM, 73-pulse preamble + 5930µs gap, 64-bit
            // MSB-first frame, 24-bit VAG rotating-register cipher, and counter-recovery validity
            // gate. Placed AFTER porsche_touareg so Touareg keeps first-match priority and Cayenne
            // can NEVER steal a Touareg capture (the registry reports the first decoder that fires
            // on a given pulse). Cayenne adds the encoder (the Touareg port is decode-only) and is
            // additionally gated on the frame_type being one of the three values the C emits/labels
            // (0b001/0b010/0b100) — a strict subset of what Touareg accepts — so on any shared
            // frame Touareg also fires and wins by ordering.
            Box::new(porsche_cayenne::PorscheCayenneDecoder::new()),
            // Toyota/Lexus (Flipper-ARF): dual-variant KeeLoq. Variant A is KeeLoq-PWM at 433 MHz
            // and shares its air encoding with Kia V3/V4 — so Toyota MUST stay AFTER kia_v3_v4
            // (which is in the Kia block near the top) to preserve Kia's first-match priority on
            // the shared PWM frames. Toyota uniquely claims 60-bit frames and the Variant-B NRZ
            // stream at 315 MHz. Emission is gated on the exact frame bit count + structural
            // preamble/sync + non-zero serial, so it false-matches nothing else.
            Box::new(toyota::ToyotaDecoder::new()),
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

    #[test]
    fn ford_v3_decodes_ldv_t80_sub() {
        // LDV T80 keyfobs use a Ford-V3-style 240/480µs Manchester frame.
        let path = Path::new("IMPORTS/LDV/LDV-T80_lock.sub");
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
        assert!(
            results.iter().any(|(n, _, _)| n == "Ford V3"),
            "expected at least one Ford V3 decode from LDV-T80_lock.sub, got: {:?}",
            results.iter().map(|(n, _, _)| n.as_str()).collect::<Vec<_>>()
        );
    }

    #[test]
    fn honda_static_decodes_unlock_honda_sub() {
        // Genuine Honda keyfob capture (CC1101 custom preset). Honda Static unpacks it via the
        // reverse-packet path (matches honda_static.c), yielding a consistent serial/counter.
        let path = Path::new("IMPORTS/honda/Unlock_honda.sub");
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
        assert!(
            results.iter().any(|(n, _, _)| n == "Honda Static"),
            "expected at least one Honda Static decode from Unlock_honda.sub, got: {:?}",
            results.iter().map(|(n, _, _)| n.as_str()).collect::<Vec<_>>()
        );
    }

    #[test]
    fn toyota_decodes_camry_variant_b_sub() {
        // 312 MHz Toyota Camry capture: Variant-B NRZ frame that Kia V3/V4 (and every other
        // decoder) rejects, so Toyota uniquely claims it. Guards the Toyota port + its registry
        // placement (after kia_v3_v4).
        let path = Path::new("IMPORTS/Toyota + Lexus/19_toyota_camry_l2_u2_t2_p2.sub");
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
        assert!(
            results.iter().any(|(n, _, _)| n == "Toyota"),
            "expected at least one Toyota decode from 19_toyota_camry_l2_u2_t2_p2.sub, got: {:?}",
            results.iter().map(|(n, _, _)| n.as_str()).collect::<Vec<_>>()
        );
    }

    #[test]
    fn toyota_does_not_steal_kia_v3_v4_prius_sub() {
        // The Prius 433 MHz captures are KeeLoq-PWM frames whose air encoding is shared between
        // Toyota Variant A and Kia V3/V4. Kia V3/V4 is earlier in the registry and MUST keep
        // first-match priority — so these must still decode as "Kia V3/V4", never "Toyota".
        let path = Path::new("IMPORTS/Toyota + Lexus/Toyota_Prius2006_lock.sub");
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
        let names: Vec<&str> = results.iter().map(|(n, _, _)| n.as_str()).collect();
        assert!(
            names.iter().any(|n| *n == "Kia V3/V4"),
            "expected Kia V3/V4 decodes from Toyota_Prius2006_lock.sub, got: {:?}",
            names
        );
        assert!(
            !names.iter().any(|n| *n == "Toyota"),
            "Toyota must NOT steal the shared KeeLoq-PWM Prius frames from Kia V3/V4, got: {:?}",
            names
        );
    }

    #[test]
    fn psa2_decodes_groupe_psa_sub() {
        // Genuine Peugeot/Citroën keyfob captures. PSA2 ("PSA OLD") decodes them via the mode23
        // XOR fast path (nibble-checksum gated), recovering a stable serial 0x99EB25 with a rolling
        // counter. Guards the PSA2 port + its registry placement (after `psa`).
        let path = Path::new("IMPORTS/GROUPE PSA/PSA_523_536.sub");
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
        let psa2: Vec<_> = results.iter().filter(|(n, _, _)| n == "PSA2").collect();
        assert!(
            !psa2.is_empty(),
            "expected at least one PSA2 decode from PSA_523_536.sub, got: {:?}",
            results.iter().map(|(n, _, _)| n.as_str()).collect::<Vec<_>>()
        );
        // The decode must carry recovered fields (serial), not a bare undecrypted frame.
        assert!(
            psa2.iter().any(|(_, d, _)| d.serial == Some(0x99EB25) && d.crc_valid),
            "expected a decrypted PSA2 frame with serial 0x99EB25, got serials: {:?}",
            psa2.iter().map(|(_, d, _)| d.serial).collect::<Vec<_>>()
        );
    }

    #[test]
    fn psa2_does_not_steal_vag_captures() {
        // PSA2 rides the same 250/500µs Manchester rate as several other protocols and the C's
        // mode23 nibble checksum is a loose ~1/16 gate. Emission is therefore restricted to a
        // field-bearing decrypt, so PSA2 must NOT match these VAG captures (which carry no valid
        // PSA2 frame) even though they reach 80 collectable bits at a PSA2-compatible frequency.
        for rel in ["VAG/Test_55_unlock_and_55_lock_suran.sub", "VAG/Vw pasat B7.sub"] {
            let path = Path::new("IMPORTS").join(rel);
            if !path.exists() {
                eprintln!("Skip: {:?} not found (run from crate root)", path);
                continue;
            }
            let (freq, raw_pairs) = import_sub_raw(&path).unwrap();
            let pairs: Vec<LevelDuration> = raw_pairs
                .iter()
                .map(|p| LevelDuration::new(p.level, p.duration_us))
                .collect();
            let mut reg = ProtocolRegistry::new();
            let results = reg.process_signal_stream(&pairs, freq);
            let names: Vec<&str> = results.iter().map(|(n, _, _)| n.as_str()).collect();
            assert!(
                !names.iter().any(|n| *n == "PSA2"),
                "PSA2 must not false-match {rel}, got: {names:?}"
            );
        }
    }

    /// Diagnostic: sweep every IMPORTS/*.sub and print which protocol(s) each decodes as.
    /// `#[ignore]`d (noisy, always passes); run with
    /// `cargo test sweep_imports_decodes -- --ignored --nocapture`.
    #[test]
    #[ignore]
    fn sweep_imports_decodes() {
        use std::fs;
        let root = Path::new("IMPORTS");
        if !root.exists() {
            eprintln!("Skip: IMPORTS not found");
            return;
        }
        // Collect IMPORTS/<make>/*.sub
        let mut subs: Vec<std::path::PathBuf> = Vec::new();
        if let Ok(makes) = fs::read_dir(root) {
            for make in makes.flatten() {
                if make.path().is_dir() {
                    if let Ok(files) = fs::read_dir(make.path()) {
                        for f in files.flatten() {
                            let p = f.path();
                            if p.extension().map(|e| e == "sub").unwrap_or(false) {
                                subs.push(p);
                            }
                        }
                    }
                }
            }
        }
        subs.sort();
        let mut reg = ProtocolRegistry::new();
        for path in &subs {
            match import_sub_raw(path) {
                Ok((freq, raw_pairs)) => {
                    let pairs: Vec<LevelDuration> = raw_pairs
                        .iter()
                        .map(|p| LevelDuration::new(p.level, p.duration_us))
                        .collect();
                    let results = reg.process_signal_stream(&pairs, freq);
                    let names: Vec<&str> = results.iter().map(|(n, _, _)| n.as_str()).collect();
                    let rel = path.strip_prefix("IMPORTS").unwrap_or(path);
                    eprintln!("{:<42} {:>9}Hz => {:?}", rel.display(), freq, names);
                }
                Err(e) => eprintln!("{:<42} ERR {:?}", path.display(), e),
            }
        }
    }
}
