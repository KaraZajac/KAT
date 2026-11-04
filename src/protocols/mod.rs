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
//! **Manchester decoding**: the C references decode Manchester with Flipper's
//! `lib/toolbox/manchester_decoder.c`; [common] holds the one exact port of it
//! (`ManchesterState`, `ManchesterEvent`, `manchester_advance`). Modules using it keep only their
//! own level→event mapping, initial state and reset points, mirroring their C file (ProtoPirate's
//! `pp_manchester_event` is `level ? ShortLow : ShortHigh`, used by Ford V0, Fiat V0 and Kia V1/V2;
//! Ford V1/V2 and Kia V5/V6/V7 use `level ? ShortHigh : ShortLow`). Ford V3, Opel V1 and
//! Renault V1 carry their own copies of the same table; Renault V0 inlines the same bytes. Fiat
//! V1/V2, Mercedes, BMW and Mazda decode half-bit cells/chips directly, as their C files do.

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
mod hitag2;
mod scher_khan;
mod star_line;
mod psa;
mod chrysler_v0;
mod mazda_v0;
mod mazda_siemens;
mod mitsubishi_v0;
mod porsche_cayenne;
mod honda_v2;
// Land Rover RKE: kept, not registered (Flipper-ARF never registers landrover_rke.c).
#[allow(dead_code)]
mod land_rover_rke;
mod toyota;
mod bmw;
mod mazda_v2;
mod mercedes_v0;
mod mercedes_v1;
mod opel_v1;
mod renault_v1;
mod renault_v0;
mod renault_v1_arf;
mod fiat_v2;
mod gm;
mod datsun_lada;
mod sheriff_cfm;
// Experimental (Pandora RE) Flipper-ARF ports. Flipper-ARF ships all three disabled and KAT mirrors
// that: they stay out of the registry (see the comment at the end of `ProtocolRegistry::new()`);
// the modules stay so their tests still run and enabling one is a one-line change.
#[allow(dead_code)]
mod audi;
#[allow(dead_code)]
mod hyundai;
#[allow(dead_code)]
mod mazda_infinity;

pub use common::DecodedSignal;
/// PSA TEA key recovery for the signal menu's "Recover key" action (ProtoPirate's brute force,
/// which the references also run on demand rather than while decoding).
pub use psa::{
    recover_key as psa_recover_key, BruteForceControl as PsaBruteForceControl,
    BruteForceOutcome as PsaBruteForceOutcome, BRUTE_FORCE_TOTAL as PSA_BRUTE_FORCE_TOTAL,
};

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
            // Kia V0 (ProtoPirate kia_v0.c) also decodes Suzuki V0, Honda V0 and Mitsubishi V0 (PP
            // folded its standalone suzuki.c / mitsubishi_v0.c into it); each type is gated on its
            // CRC-8 / structural check and reported under its display name.
            Box::new(kia_v0::KiaV0Decoder::new()),
            Box::new(kia_v1::KiaV1Decoder::new()),
            Box::new(kia_v2::KiaV2Decoder::new()),
            Box::new(kia_v3_v4::KiaV3V4Decoder::new()),
            Box::new(kia_v5::KiaV5Decoder::new()),
            Box::new(kia_v6::KiaV6Decoder::new()),
            Box::new(kia_v7::KiaV7Decoder::new()),
            // VAG (ProtoPirate vag.c; ARF adds 315 MHz): 300/600µs Manchester after ≥151 preamble
            // pairs + 600µs gap + 0xAF3F/0xAF1C prefix (Type 1/2), or 500/1000µs after ≥31 pairs +
            // 1000/500 sync + 3×750µs pairs (Type 3/4); exactly 80 bits. Like the C it reports every
            // 80-bit frame (undecrypted ones without fields). Kept ahead of the Ford/Subaru block so a
            // 500/1000µs VAG stream is tried as VAG first.
            Box::new(vag::VagDecoder::new()),
            // Ford V0: preamble + 3500µs gap structure, and emission gated on the matrix CRC (low 7
            // bits, the bits ford_v0.c checks; the C itself emits CRC failures too).
            Box::new(ford_v0::FordV0Decoder::new()),
            // Ford V1: distinct 65/130µs Manchester (vs V0 250/500, V2 200/400, V3 240/480) and
            // CRC16-gated, so placement in the Ford group is safe (won't steal V0/V2/V3 captures).
            Box::new(ford_v1::FordV1Decoder::new()),
            // Ford V3: EU 240/480µs Manchester path (≥30-short preamble, 104 bits, ungated as in
            // the C) + US half-bit-cell path gated on 0xFF header / serial / button 1|2 / flag
            // bit 7. A US hit only fires at the end of a ≥200-cell burst that begins with that
            // header; the IMPORTS sweep shows no change for the Ford V0/V1/V2 captures.
            Box::new(ford_v3::FordV3Decoder::new()),
            Box::new(ford_v2::FordV2Decoder::new()),
            // Honda Static after the Kia/Ford block; checksum-gated emission makes order safe.
            Box::new(honda_static::HondaStaticDecoder::new()),
            // Honda V1: distinct 1000/2000µs PWM (vs Honda Static's 63µs Manchester). Button-gated
            // emission makes order safe.
            Box::new(honda_v1::HondaV1Decoder::new()),
            // Subaru (ProtoPirate subaru.c + Flipper-ARF's reader as a fallback path): 800/1600µs
            // PWM after ≥21 1600µs preamble halves + 2800µs gap/sync, 64 bits. No checksum; the
            // long preamble + gap/sync framing is unique among KAT protocols.
            Box::new(subaru::SubaruDecoder::new()),
            // Fiat V0 has no checksum: gated on structure (≥150-pulse preamble, 800µs gap, 71 bits).
            Box::new(fiat_v0::FiatV0Decoder::new()),
            // Fiat V1 (HITAG2 BCM): 104-bit Manchester frame read from a sliding half-bit window at
            // 250/500 or 100/200µs, gated on 00 01 sync + XOR + button + UID. Must stay BEFORE
            // renault_v1, which carries the same frame (see its comment at the end of this list).
            // TX: fresh frames when a ProtoPirate key or an ARF dictionary/heuristic key verifies
            // the hop, otherwise ARF's byte-identical replay.
            Box::new(fiat_v1::FiatV1Decoder::new()),
            Box::new(scher_khan::ScherKhanDecoder::new()),
            Box::new(star_line::StarLineDecoder::new()),
            // Sheriff CFM (Flipper-ARF sheriff_cfm.c, ZX-750/ZX-930 alarms): the SAME KeeLoq PWM
            // framing as `keeloq` (400/800µs, 4ms header, 64..66 bits), but MSB-first fix|hop with a
            // per-model hop byte shuffle and the ZX-750/930 keys from the keystore. It fires only on
            // a validated ZX decrypt (button whitelist + 10-bit serial match, ≈1/32768 for a random
            // frame). It MUST precede `keeloq`: `keeloq`'s keystore identification (ARF's
            // per-learning-type selector) accepts a random KeeLoq-shaped frame ≈6% of the time,
            // while a genuine KeeLoq frame from another system passes Sheriff's check ≈1 in 32768;
            // all genuine Sheriff frames are reported as Sheriff CFM.
            Box::new(sheriff_cfm::SheriffCfmDecoder::new()),
            // KeeLoq (Flipper-ARF keeloq.c): emits only identified frames (AN-Motors, HCS101 or a
            // keystore manufacturer), see keeloq.rs.
            Box::new(keeloq::KeeloqDecoder::new()),
            // PSA (ProtoPirate psa.c + Flipper-ARF's adaptive AM path): 250/500µs Manchester behind
            // a >0x46-pair preamble and 500µs sync, exactly 80 bits, a 1000µs end marker and the
            // 0xA sync nibble (ProtoPirate's gate); AM-path frames must also decrypt. TEA frames
            // are brute-forced in the decoder (≤ ~0.5 s). (PSA2 / "PSA OLD", Flipper-ARF's
            // unregistered psa2.c, was retired: PSA decodes its captures with the same fields;
            // `get()` maps the old names here.)
            Box::new(psa::PsaDecoder::new()),
            // Chrysler V0 (ProtoPirate chrysler_v0.c = ARF chrysler.c = doctor chrysler_v0.c):
            // 300/600µs HIGH + 3400/3700µs LOW PWM, 80 bits. Timing is unique and emission is gated
            // on the frame's own structural check (`check_ok`), so placement here is low-risk.
            Box::new(chrysler_v0::ChryslerV0Decoder::new()),
            // Honda V2 (formerly "Land Rover V0"; ProtoPirate renamed it): differential Manchester
            // 250/500µs, 81 bits. Emission is gated on a 3-bit check polynomial + 16-bit tail + zero
            // reserved bits, so placement is safe.
            Box::new(honda_v2::HondaV2Decoder::new()),
            // (Land Rover RKE — fixed-width PWM 700/300µs, 20-pulse preamble, 400µs+9600µs sync,
            // 66 bits — is ported from Flipper-ARF's landrover_rke.c, a Pandora-RE helper that ARF
            // never registers as a protocol. KAT mirrors ARF and leaves it unregistered; add
            // `Box::new(land_rover_rke::LandRoverRkeDecoder::new())` here to enable it.)
            // Mazda V0 (ProtoPirate): 250/500µs Manchester at 315 + 433.92 MHz. Emission is gated
            // on >15 preamble ones + the exact 0xD7 sync, exactly 64 data bits and the key's
            // additive checksum, so it false-matches nothing else. Decoder + encoder.
            Box::new(mazda_v0::MazdaV0Decoder::new()),
            // Mazda Siemens (Flipper-ARF): the SAME air format as Mazda V0 (the reference encoders
            // emit identical waveforms), read by ARF's pair-based state machine and gated on the
            // same additive checksum + preamble/bit-count structure. Placed AFTER mazda_v0 so
            // Mazda V0 keeps first-match priority; on a shared frame Mazda V0 also completes first
            // (after the 64th data bit, while this decoder waits for the trailing gap), so Mazda
            // Siemens only reports frames Mazda V0 rejects. Adds the Siemens-layout encoder.
            Box::new(mazda_siemens::MazdaSiemensDecoder::new()),
            // (BMW CAS4 was retired: Flipper-ARF disabled bmw_cas4.c and replaced it with "BMW",
            // registered further down.)
            // Mitsubishi V0, 96-bit format (Flipper-ARF mitsubishi_v0.c): 250/500µs complementary
            // PWM, no preamble/checksum gate (as in the C), reported on the 96th pair. Kia V0's
            // Mitsubishi V0 frames (61-bit) are a different air format, decoded by kia_v0; this
            // entry owns the "Mitsubishi V0" name for TX and routes 61-bit captures back to Kia V0.
            Box::new(mitsubishi_v0::MitsubishiV0Decoder::new()),
            // Porsche Cayenne (Flipper-ARF porsche_cayenne.c, internal name "Porsche AG"; KAT's
            // former "Porsche Touareg" was ProtoPirate's port of the same file, retired with it —
            // `get()` maps the old name here). 1680/3370µs PWM behind ≥15 3370µs sync pulses and a
            // 5930µs gap pair, exactly 64 bits; like the C it reports every such frame (counter
            // recovered by brute force, `crc_valid` = cipher-consistent). No other protocol uses
            // these slow timings. Decoder + encoder.
            Box::new(porsche_cayenne::PorscheCayenneDecoder::new()),
            // Toyota/Lexus (Flipper-ARF toyota.c = doctor-firmware "Toyota Lexus"; replaced ARF's
            // retired KeeLoq Toyota): ~793/1587µs bi-phase read level-agnostically, emitted only
            // for a 120-bit window that starts 0x00 / MSB-set and repeats its first 3 bytes 96 bits
            // later (the C's gate). The 400/800µs KeeLoq-PWM frames (Kia V3/V4, Prius) are
            // invalid pulses to it. Decoder + encoder.
            Box::new(toyota::ToyotaDecoder::new()),
            // BMW (Flipper-ARF "BMW", which replaced ARF's retired BMW CAS4): 62µs Manchester chips
            // (1T 40-90µs, 2T 100-160µs, ≥400µs = gap), 128-bit frame, AM, 433.92 MHz. Any pulse of
            // 500µs or more is a gap here, so slower Manchester streams never enter it. Emission is
            // gated on a
            // 32-280 chip preamble, sync 0x599E (or inverted 0xA661) within 40 header bits, exactly
            // 128 bits, the 0x01/0xE7 button template with a non-zero button, and the fixed 0x89
            // byte. Appended at the end: the gates make it false-match nothing, so it needs no
            // priority over anything.
            Box::new(bmw::BmwDecoder::new()),
            // Mazda V2 (Flipper-ARF): 500/1000µs bi-phase sampled one level per bit, 315 + 433.92
            // MHz, AM+FM. A different protocol from Mazda V0 / Mazda Siemens (250/500µs at 433.92):
            // their frames are all "short" pulses to this sampler and can never form its sync, while
            // its 1000µs pulses break their pair decoders. Emission is gated on ≥100 identical
            // preamble samples immediately before the exact 8-bit sync 0xEC (0x13 inverted) plus a
            // valid button byte. Appended at the end so every earlier 500/1000µs decoder keeps
            // first-match priority on any shared stream.
            Box::new(mazda_v2::MazdaV2Decoder::new()),
            // --- Mercedes-Benz / Opel (Flipper-ARF ports), appended last so every earlier decoder
            // keeps first-match priority. ---
            // Mercedes V0: FM Manchester 500µs, 112 bits. Emission needs a preamble of exactly
            // 108..=113 single-chip (500µs) edges, a 1000µs sync, 112 clean chip pairs and a
            // command nibble in the PIC whitelist {C,8,0,A,4} (+ KAT: non-degenerate payload).
            // Longer/shorter 500µs runs (VAG …) never activate it, so it is low-risk.
            Box::new(mercedes_v0::MercedesV0Decoder::new()),
            // Mercedes V1: FM Manchester 50µs chips (25–79 / 80–140µs), ≥32-pair preamble, 2T sync,
            // 122 bits. The C field check is weak (bit0, bit3 = !(bit1^bit2), bit121), so KAT also
            // rejects frames with a ≥32-bit constant leading/trailing run (preamble read as data).
            // Its chip windows overlap Ford V1 (65/130µs) and Honda Static (63µs), but neither frame
            // forms V1's ≥32×1T-pair preamble + 2T sync (Ford V1's preamble is 130µs pulses, Honda
            // Static has a 700µs sync gap); a cross-protocol encoder test in mercedes_v1.rs guards it.
            Box::new(mercedes_v1::MercedesV1Decoder::new()),
            // Opel V1: AM Manchester 120/240µs, 93-bit sliding window: framing 0x02 + XTEA
            // ("RobertBOSCH48331") plaintext tail 0x05 + KAT button whitelist {0x20,0x04,0x08}.
            // Decode-only (the C encoder is NULL).
            Box::new(opel_v1::OpelV1Decoder::new()),
            // Renault V1 (ProtoPirate renault_v1.c, HITAG2): 1500/1000µs header, then a 104-bit
            // adaptive-TE (~125µs) Manchester frame that is byte-for-byte the Fiat V1 frame
            // (00 01 ‖ uid ‖ btn/cnt10 ‖ hop ‖ tail ‖ XOR). Gated on header word 0x0001 + XOR +
            // sane UID. Its header window (1150-2200 / 800-1150µs) also accepts Fiat V1 type-B's
            // 1230µs gap + 850µs lead, so it is placed AFTER fiat_v1 and emits one pulse after the
            // C (on the frame's last half-bit, exactly where Fiat V1's cell window completes):
            // on any frame both decode, Fiat V1 keeps first-match priority. Every accepted frame
            // runs ProtoPirate's 2^18 serial-derived-key seed search (~10 ms release); TX = fresh
            // seed-scheme frames when it succeeds, else doctor-firmware's byte-identical replay.
            Box::new(renault_v1::RenaultV1Decoder::new()),
            // Renault V0 (ProtoPirate): 125/250µs Manchester after a ≥800µs HIGH sync, 82 bits
            // (64-bit key + 18-bit Key2), 3 bursts. Emission is gated on the C's type-13 checksum
            // (8 bits: C1 + C2) + the 0x05/0x06/0x0A button whitelist + ≥800µs sync, ≥10-bit
            // preamble and 82-bit structure, so it does not false-match other frames.
            Box::new(renault_v0::RenaultV0Decoder::new()),
            // Renault V1 (ARF) (Flipper-ARF firmware "Renault V1"; not ProtoPirate's HITAG2 Renault
            // V1 above): the Renault V0 wire (≥800µs HIGH sync, 125/250µs Manchester) carrying the
            // 82-bit type-table frames 0x04/0x0C/0x1A/0x3B/0x3F + "Dynamic" behind a 12/16/18/20-bit
            // preamble. It rejects the 0x13 checksum tag that Renault V0 requires, so on the same
            // 82-bit window the two are mutually exclusive (tested); placed right after renault_v0
            // as in ARF's registry. Typed frames: 6-bit tag + 2-bit check + per-type button range +
            // exact preamble length; Dynamic frames (2-bit check + button whitelist) must repeat
            // identically in two consecutive bursts. Replay encoder.
            Box::new(renault_v1_arf::RenaultV1ArfDecoder::new()),
            // Fiat V2 (ProtoPirate, FCA: Fiat/Chrysler/Jeep): 210/420µs Manchester, one 112-bit
            // frame per burst. The C's sliding-window gate (0x00/0x01 markers + selector + UID)
            // alone also matches the middle of other Manchester frames (e.g. LDV/Ford V3), and an
            // early hit would steal them in this first-match registry; KAT therefore additionally
            // requires the window to end the burst. Placed last so every earlier decoder keeps
            // first-match priority on any shared pulse.
            Box::new(fiat_v2::FiatV2Decoder::new()),
            // GM (Flipper-ARF gm.c, from rtl_433 gm_car_remote.c): OOK PPM — fixed HIGH pulse +
            // 300/500µs LOW gap per bit, 112-bit frame taken as the last 112 bits of a burst.
            // Emission is gated on the 0xFF wake-up byte + additive button-nibble check + additive
            // byte-sum checksum (≈2^-20 per window), the C's only emission path, so it false-matches
            // nothing; no other KAT protocol uses gap-width PPM. Replay-only encoder.
            Box::new(gm::GmDecoder::new()),
            // Datsun Lada (doctor-firmware datsun_lada.c): FM 433, Manchester 250/500µs, 88-bit
            // Hitag2-family frame. Gated on the C's exact-88-bit framing + XOR checksum, plus a KAT
            // preamble gate (>= 64 short pulses before the sync; the C encoder sends 125). Fiat V1
            // (Marelli PCF7946) carries the same 11-byte frame shape in the opposite polarity with
            // only a 29-toggle run before its sync, so the preamble gate keeps Datsun Lada off Fiat
            // captures; placed last so Fiat V1 and every earlier 250/500µs decoder keep priority.
            Box::new(datsun_lada::DatsunLadaDecoder::new()),
            // --- Experimental (Pandora RE): NOT registered -----------------------------------
            // Audi, Mazda Infinity and Hyundai are Flipper-ARF ports reverse-engineered from the
            // Pandora alarm firmware. ARF ships all three disabled, and KAT mirrors that. None of
            // the IMPORTS captures contains them. To enable one, add it here (LAST, so every
            // established decoder keeps first-match priority):
            //   Box::new(audi::AudiDecoder::new()),
            //       550/1100µs PWM behind a 2500/850µs header, 64 bits; gated on exactly 64 strict
            //       PWM bits + UID + checksum b7 = sum(b0..b6) + button in {1,2,4} (~1/21800).
            //   Box::new(mazda_infinity::MazdaInfinityDecoder::new()),
            //       the same air format as Mazda V0 (which keeps priority on shared frames).
            //   Box::new(hyundai::HyundaiDecoder::new()),
            //       500/1000µs Manchester, 72 bits; the frame has NO checksum, so validity rests on
            //       framing alone (16..32-byte 0x55 preamble + duplicate frame).
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
        // Display names may carry a variant suffix (e.g. "Porsche Cayenne [First]"); the decoder's
        // name() is the part before " [".
        let name = name.split(" [").next().unwrap_or(name);
        let lookup = if name.starts_with("KeeLoq") {
            "KeeLoq"
        } else if name.eq_ignore_ascii_case("Land Rover V0") {
            // Legacy name: ProtoPirate renamed Land Rover V0 to Honda V2 (same protocol), and
            // decoded .fob imports keep the protocol name they were exported with.
            "Honda V2"
        } else if name.eq_ignore_ascii_case("Porsche Touareg") {
            // Retired with ProtoPirate's porsche_touareg.c, a port of the same porsche_cayenne.c
            // (identical frame and cipher): old captures transmit with Porsche Cayenne.
            "Porsche Cayenne"
        } else if ["Suzuki", "Suzuki V0", "Honda V0"].iter().any(|n| name.eq_ignore_ascii_case(n)) {
            // Kia V0 frame types (display names), and ProtoPirate's retired standalone Suzuki.
            "Kia V0"
        } else if name.eq_ignore_ascii_case("Mitsu V0") {
            // ProtoPirate's alias for Mitsubishi V0.
            "Mitsubishi V0"
        } else if ["Kia V3", "Kia V4", "KIA/HYU V3", "KIA/HYU V4"]
            .iter()
            .any(|n| name.eq_ignore_ascii_case(n))
        {
            // Kia V3/V4 reports its version as the display name (ProtoPirate's catalog aliases).
            "Kia V3/V4"
        } else if ["PSA2", "PSA OLD", "PSA GROUP"].iter().any(|n| name.eq_ignore_ascii_case(n)) {
            // Retired PSA2 (Flipper-ARF psa2.c "PSA OLD", unregistered upstream) and ARF's display
            // name for PSA: the same frames, decoded and encoded by PSA.
            "PSA"
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

/// Human-readable name for a protocol's NATIVE button code, for protocols whose `DecodedSignal.button`
/// is not KAT's generic code (0x01 Lock, 0x02 Unlock, 0x04 Trunk, 0x08 Panic). `None` means "use the
/// generic names". Codes and labels come from each protocol's C reference (see its module doc).
pub fn native_button_name(protocol: &str, code: u8) -> Option<&'static str> {
    let name = match protocol.split(" [").next().unwrap_or(protocol) {
        "BMW" => match code {
            0x1 => "Unlock",
            0x2 => "Lock",
            0x3 => "Trunk",
            _ => "Other",
        },
        // bit 7 is carried through; 0x06/0x0A/0x0E are the "alt" Lock/Unlock/Trunk family.
        "Mazda V2" => match code & 0x7F {
            0x04 | 0x06 => "Lock",
            0x08 | 0x0A => "Unlock",
            0x10 | 0x0E => "Trunk",
            _ => "Other",
        },
        "Mazda V0" if code == 0x8 => "Remote",
        "Mazda Siemens" | "Mazda Infinity" => match code {
            0x10 => "Lock",
            0x20 => "Unlock",
            0x40 => "Trunk",
            _ => "Other",
        },
        "Mercedes V0" => match code {
            0xC => "Lock",
            0x8 => "Unlock",
            0xA => "Aux",
            0x0 => "Follow-up",
            _ => "Other",
        },
        "Mercedes V1" => match code {
            0 => "Lock",
            1 => "Unlock",
            _ => "Other",
        },
        "Opel V1" => match code {
            0x04 => "Lock",
            0x08 => "Unlock",
            0x20 => "Trunk",
            _ => "Other",
        },
        "Renault V0" => match code {
            0x06 => "Lock",
            0x0A => "Unlock",
            0x05 => "Trunk",
            _ => "Other",
        },
        "Renault V1" if code == 0 => "Sync",
        // ARF renault_v1_get_button_name: the low nibble is the action, the high nibble the family.
        "Renault V1 (ARF)" => match code & 0x0F {
            0x4..=0x7 => "Lock",
            0x8..=0xB => "Unlock",
            _ => "Other",
        },
        "Fiat V1" => match code {
            0x4 => "Lock",
            0x8 => "Unlock",
            0x2 => "Trunk",
            0x1 => "Close",
            _ => "Other",
        },
        // Button selector = top two bits of the raw button byte.
        "Fiat V2" => match code >> 6 {
            2 => "Lock",
            3 => "Unlock",
            1 => "Trunk",
            _ => "Other",
        },
        "GM" => match code & 0x07 {
            2 => "Lock",
            1 => "Unlock",
            3 => "Trunk",
            4 => "Panic",
            _ => "Other",
        },
        // The C only labels these BTN0/1/3/5; the Lock/Unlock/Trunk/Panic meaning is unverified.
        "Datsun Lada" => match code {
            0x40 => "Btn0",
            0x80 => "Btn1",
            0x20 => "Btn3",
            0x60 => "Btn5",
            _ => "Other",
        },
        "Sheriff CFM" => match code {
            0x10 => "Lock",
            0x20 => "Unlock",
            0x40 => "Trunk",
            0x80 => "Panic",
            _ => "Other",
        },
        // Flipper-ARF scher_khan_btn_name (combination presses included).
        "Scher-Khan" => match code {
            0x1 => "Lock",
            0x2 => "Unlock",
            0x3 => "Lock+Unlock",
            0x4 => "Trunk",
            0x5 => "Lock+Trunk",
            0x6 => "Unlock+Trunk",
            0x7 => "Lk+Ul+Tr",
            0x8 => "Start",
            0x9 => "Lock+Start",
            0xA => "Unlock+Start",
            0xC => "Trunk+Start",
            0xF => "All/Panic",
            _ => "Other",
        },
        // Flipper-ARF star_line_btn_name (0x2x = 2-way remotes).
        "Star Line" => match code {
            0x01 | 0x21 => "Lock",
            0x02 | 0x22 => "Unlock",
            0x03 | 0x23 => "Trunk",
            0x04 => "Panic",
            0x24 => "Start",
            0x25 => "Stop",
            0x26 => "Extra",
            _ => "Other",
        },
        // KeeLoq function codes are generic-compatible (1/2/4/8); Flipper-ARF only relabels the
        // "shifted" brands' codes 2/4/8/1 as physical buttons B1..B4.
        p if p.starts_with("KeeLoq (") && p.ends_with(')') => {
            match keeloq::keeloq_button_position(&p["KeeLoq (".len()..p.len() - 1], code) {
                Some(1) => "B1",
                Some(2) => "B2",
                Some(3) => "B3",
                Some(4) => "B4",
                _ => return None,
            }
        }
        // Ford V0 reports KAT's generic codes plus a fifth one-hot button.
        "Ford V0" if code == 0x10 => "Remote start",
        // Ford V1: 1/2/4/8 are KAT's generic codes; 0 is ford_v1_get_button_name's "Sync".
        "Ford V1" if code == 0x0 => "Sync",
        "Ford V2" => match code {
            0x10 => "Lock",
            0x11 => "Unlock",
            0x13 => "Trunk",
            0x14 => "Panic",
            0x15 => "Remote start",
            _ => "Other",
        },
        // 1/2/4/8 are KAT's generic codes (honda_static_button_names).
        "Honda Static" => match code {
            0x5 => "Remote start",
            0x9 => "Lock x2",
            _ => return None,
        },
        "Honda V1" => match code {
            0x0 => "Unlock",
            0x8 => "Lock",
            0x9 => "Trunk",
            0xA => "Panic",
            _ => "Other",
        },
        // The 7 tail bits; ProtoPirate's fiat_v0_display_suffix labels by the low nibble.
        "Fiat V0" => match code & 0x0F {
            0x4..=0x7 => "Lock",
            0x8..=0xB => "Unlock",
            _ => "Other",
        },
        // US variant; the EU variant reports 0x01/0x02 (generic Lock/Unlock).
        "Ford V3" => match code {
            0x07 => "Trunk",
            0x06 => "Panic",
            _ => return None,
        },
        "Honda V2" | "Land Rover V0" => match code {
            0x02 => "Lock",
            0x04 => "Unlock",
            _ => "Other",
        },
        // toyota_button_name: the Estima button bytes, or the C's command 1-3 ("Cmd").
        "Toyota" => match code {
            0xD1 | 0xD4 | 0x99 | 0x9C => "Estima",
            1 => "Cmd 1",
            2 => "Cmd 2",
            3 => "Cmd 3",
            _ => "Unknown",
        },
        // porsche_cayenne_custom_to_btn names 0x08 "Open" (KAT's generic table says Panic).
        "Porsche Cayenne" | "Porsche Touareg" => match code {
            0x01 => "Lock",
            0x02 => "Unlock",
            0x04 => "Trunk",
            0x08 => "Open",
            _ => "Other",
        },
        // Types 2/3/4 report the decrypted button nibble, Type 1 the whole byte (button in the
        // high nibble; ProtoPirate's vag_button_name, which calls 4/0x40 "Boot").
        "VAG" => match if code > 0x0F { code >> 4 } else { code } {
            0x1 => "Unlock",
            0x2 => "Lock",
            0x4 => "Trunk",
            _ => "Other",
        },
        // ProtoPirate emulate / Flipper-ARF codes (1 and 2 coincide with the generic ones).
        "Subaru" => match code {
            0x1 => "Lock",
            0x2 => "Unlock",
            0x3 => "Trunk",
            0x4 => "Panic",
            _ => "Other",
        },
        // Kia V0 family (ProtoPirate kia_v0_button_name; Panic = the emulate D-pad's Left code).
        "Kia V0" | "Mitsubishi V0" => match code {
            0x1 => "Lock",
            0x2 => "Unlock",
            0x3 => "Trunk",
            0x4 => "Panic",
            _ => "Other",
        },
        "Suzuki V0" | "Suzuki" => match code {
            0x3 => "Lock",
            0x4 => "Unlock",
            0x2 => "Trunk",
            0x1 => "Panic",
            _ => "Other",
        },
        "Kia V3/V4" | "Kia V3" | "Kia V4" => match code {
            0x1 => "Lock",
            0x2 => "Unlock",
            0x3 => "Trunk",
            0x4 => "Panic",
            0x8 => "Horn",
            _ => "Other",
        },
        // ProtoPirate kia_v1.c names (Close/Open/Boot) and the emulate D-pad's Panic; Flipper-ARF
        // uses the same codes for Kia V2 and V6.
        "Kia V1" | "Kia V2" | "Kia V6" => match code {
            0x1 => "Lock",
            0x2 => "Unlock",
            0x3 => "Trunk",
            0x4 => "Panic",
            _ => "Other",
        },
        // 1/2/8 are KAT's generic Lock/Unlock/Panic.
        "Kia V7" => match code {
            0x3 => "Trunk",
            _ => return None,
        },
        // Flipper-ARF kia_v5.c names.
        "Kia V5" => match code {
            0x1 => "Unlock",
            0x2 => "Lock",
            0x4 => "Trunk",
            0x8 => "Horn",
            _ => "Other",
        },
        "Honda V0" => match code {
            1 => "Unlock",
            2 => "Trunk",
            3 => "Lock2",
            4 => "Unlock2",
            5 => "Trunk2",
            6 => "Unlock3",
            7 => "Trunk3",
            _ => "Other",
        },
        // Flipper-ARF psa_button_name (the genuine IMPORTS frames carry 0).
        "PSA" | "PSA2" | "PSA OLD" | "PSA GROUP" => match code {
            0x0 => "Lock",
            0x1 => "Unlock",
            0x2 => "Trunk",
            _ => "Other",
        },
        _ => return None,
    };
    Some(name)
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
    fn toyota_does_not_claim_camry_noise_sub() {
        // This 312 MHz Camry capture (an OOK recording of what is likely an FSK smart key) used to
        // "decode" once as the old KeeLoq Toyota port's Variant B: hop 0x1AAAAAAA (an alternating
        // bit run) and button 0x1, in 1 of 8 presses — a false positive that ARF's own final
        // KeeLoq decoder (toyota-old.c.bak, with its Tundra button whitelist) rejects. ARF then
        // replaced that decoder with the current toyota.c, which (like KAT's port) finds nothing
        // here; no IMPORTS capture contains the current Toyota format.
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
            !results.iter().any(|(n, _, _)| n == "Toyota"),
            "unexpected Toyota decode from 19_toyota_camry_l2_u2_t2_p2.sub: {:?}",
            results.iter().map(|(n, _, _)| n.as_str()).collect::<Vec<_>>()
        );
    }

    #[test]
    fn prius_keeloq_frames_are_not_claimed_by_kia_v3_v4() {
        // The 2006 Prius captures are 400/800µs KeeLoq-PWM frames (plaintext serial 08DCE7FB in the
        // fixed part). Like both C references (ProtoPirate / Flipper-ARF kia_v3_v4.c), Kia V3/V4 only
        // reports a frame whose hop decrypts under the KIA key to the frame's own button and serial
        // LSB — the Prius hop does not, so Kia V3/V4 must not claim it (its old claims were bogus).
        // The structure-only KeeLoq Toyota decoder that also reported these frames was retired by
        // Flipper-ARF (toyota-old.c.bak) in favour of the current Toyota/Lexus protocol, which this
        // format is not, so — as in the references — nothing in the registry decodes them now.
        crate::protocols::keys::load_keystore_from_embedded();
        for file in ["Toyota_Prius2006_lock.sub", "Toyota_Prius2006_unlock.sub"] {
            let path = Path::new("IMPORTS/Toyota + Lexus").join(file);
            if !path.exists() {
                eprintln!("Skip: {:?} not found (run from crate root)", path);
                return;
            }
            let (freq, raw_pairs) = import_sub_raw(&path).unwrap();
            let pairs: Vec<LevelDuration> = raw_pairs
                .iter()
                .map(|p| LevelDuration::new(p.level, p.duration_us))
                .collect();
            let mut reg = ProtocolRegistry::new();
            let names: Vec<String> = reg
                .process_signal_stream(&pairs, freq)
                .into_iter()
                .map(|(n, _, _)| n)
                .collect();
            assert!(!names.iter().any(|n| n.starts_with("Kia")), "{file}: {names:?}");
        }
    }

    #[test]
    fn psa_decodes_groupe_psa_sub() {
        // Genuine Peugeot/Citroën keyfob captures (formerly the PSA2 regression). PSA decodes the
        // XOR fob's frame (serial 0x99EB25, counter 0x039B) and, like the references, reports the
        // TEA fob's frame undecrypted — the "Recover key" action brute-forces it (serial 0x014590;
        // covered by psa.rs `brute_force_finds_the_tea_fob_like_protopirate`).
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
        let serials: Vec<(&str, Option<u32>)> =
            results.iter().map(|(n, d, _)| (n.as_str(), d.serial)).collect();
        assert_eq!(serials, vec![("PSA", Some(0x99EB25)), ("PSA", None)]);
    }

    #[test]
    fn psa_recover_key_pipeline_on_real_capture() {
        // The signal menu's "Recover key" path (minus the thread): decode the genuine capture, take
        // the undecrypted TEA frame, brute-force it with the public API, and transmit from the
        // recovered fields.
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
        let tea = reg
            .process_signal_stream(&pairs, freq)
            .into_iter()
            .map(|(_, d, _)| d)
            .find(|d| d.serial.is_none())
            .expect("undecrypted TEA frame");
        assert!(!tea.encoder_capable);
        let ctl = PsaBruteForceControl::default();
        let PsaBruteForceOutcome::Found(hit) = psa_recover_key(tea.data, tea.extra.unwrap(), &ctl)
        else {
            panic!("expected the TEA fob's key to be recovered");
        };
        assert_eq!(hit.fields.serial, 0x014590);
        assert!(ctl.progress.load(std::sync::atomic::Ordering::Relaxed) > 0);
        let recovered = DecodedSignal {
            serial: Some(hit.fields.serial),
            button: Some(hit.fields.button),
            counter: Some(hit.fields.counter as u16),
            crc_valid: true,
            encoder_capable: true,
            ..tea
        };
        let psa = reg.get("PSA").unwrap();
        assert!(psa.encode(&recovered, 0x01).is_some(), "recovered TEA frame must encode");
    }

    #[test]
    fn psa2_does_not_steal_vag_captures() {
        // PSA (and the retired PSA2) ride the same 250/500µs Manchester rate as several other
        // protocols; Flipper-ARF's psa.c even accepts 7 frames of these VAG captures on its loose
        // XOR-checksum path. PSA keeps ProtoPirate's 0xA sync gate, so it must NOT match them.
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
                !names.iter().any(|n| n.starts_with("PSA")),
                "PSA must not false-match {rel}, got: {names:?}"
            );
        }
    }

    #[test]
    fn registry_get_resolves_legacy_and_suffixed_names() {
        let reg = ProtocolRegistry::new();
        // ProtoPirate renamed Land Rover V0 → Honda V2; old .fob exports keep the legacy name.
        assert_eq!(reg.get("Honda V2").map(|d| d.name()), Some("Honda V2"));
        assert_eq!(reg.get("Land Rover V0").map(|d| d.name()), Some("Honda V2"));
        assert_eq!(reg.get("land rover v0").map(|d| d.name()), Some("Honda V2"));
        // Display names with a variant suffix resolve to the decoder's base name.
        assert_eq!(
            reg.get("Porsche Cayenne [First]").map(|d| d.name()),
            Some("Porsche Cayenne")
        );
        // Porsche Touareg was retired (same frame as Porsche Cayenne); old captures still resolve.
        assert_eq!(reg.get("Porsche Touareg").map(|d| d.name()), Some("Porsche Cayenne"));
        assert_eq!(
            reg.get("Porsche Touareg [Final]").map(|d| d.name()),
            Some("Porsche Cayenne")
        );
        assert_eq!(reg.get("Kia V7").map(|d| d.name()), Some("Kia V7"));
        // Variant / manufacturer display names of the KeeLoq-family decoders.
        assert_eq!(
            reg.get("Scher-Khan [MAGIC CODE, Dynamic]").map(|d| d.name()),
            Some("Scher-Khan")
        );
        assert_eq!(reg.get("Star Line [SL_A2-A4]").map(|d| d.name()), Some("Star Line"));
        assert_eq!(reg.get("KeeLoq (DoorHan)").map(|d| d.name()), Some("KeeLoq"));
        // PSA2 / "PSA OLD" were retired into PSA; "PSA GROUP" is Flipper-ARF's name for PSA.
        for legacy in ["PSA2", "psa2", "PSA OLD", "PSA GROUP"] {
            assert_eq!(reg.get(legacy).map(|d| d.name()), Some("PSA"), "{legacy}");
        }
        assert!(reg.get("No Such Protocol").is_none());
    }

    /// Diagnostic companion to `sweep_imports_decodes`: print every decode's fields for selected
    /// captures. `#[ignore]`d; run with e.g.
    /// `KAT_DETAIL_FILES="FORD/3_unlock_ford.sub;LDV/LDV-T80_lock.sub" cargo test detail_decodes -- --ignored --nocapture`
    /// (paths relative to `IMPORTS/`, separated by `;`).
    #[test]
    #[ignore]
    fn detail_decodes() {
        // Keystore as the app loads it at startup (Kia V3/V4's KeeLoq check, Kia V5/V6 decryption).
        crate::protocols::keys::load_keystore_from_embedded();
        let files = std::env::var("KAT_DETAIL_FILES").unwrap_or_default();
        let mut reg = ProtocolRegistry::new();
        for rel in files.split(';').filter(|s| !s.is_empty()) {
            let path = Path::new("IMPORTS").join(rel);
            let (freq, raw_pairs) = import_sub_raw(&path).unwrap();
            let pairs: Vec<LevelDuration> = raw_pairs
                .iter()
                .map(|p| LevelDuration::new(p.level, p.duration_us))
                .collect();
            for (n, d, _) in reg.process_signal_stream(&pairs, freq) {
                eprintln!(
                    "{rel}: {n} serial={:08X?} btn={:?} cnt={:?} crc={} data={:016X} bits={} extra={:X?}",
                    d.serial, d.button, d.counter, d.crc_valid, d.data, d.data_count_bit, d.extra
                );
            }
        }
    }

    #[test]
    fn native_button_names() {
        assert_eq!(native_button_name("BMW", 1), Some("Unlock"));
        assert_eq!(native_button_name("Mercedes V0", 0xC), Some("Lock"));
        assert_eq!(native_button_name("Fiat V2", 0b1000_0000), Some("Lock"));
        assert_eq!(native_button_name("Honda V2", 0x02), Some("Lock"));
        assert_eq!(native_button_name("Land Rover V0", 0x04), Some("Unlock"));
        assert_eq!(native_button_name("Ford V3", 0x07), Some("Trunk"));
        assert_eq!(native_button_name("Ford V1", 0x0), Some("Sync"));
        assert_eq!(native_button_name("Ford V1", 0x1), None); // generic Lock
        assert_eq!(native_button_name("Ford V2", 0x13), Some("Trunk"));
        assert_eq!(native_button_name("Honda V1", 0x8), Some("Lock"));
        assert_eq!(native_button_name("Honda Static", 0x5), Some("Remote start"));
        assert_eq!(native_button_name("Honda Static", 0x2), None); // generic Unlock
        assert_eq!(native_button_name("Fiat V0", 0x2B), Some("Unlock"));
        assert_eq!(native_button_name("Ford V3", 0x01), None); // generic Lock
        assert_eq!(native_button_name("Renault V1", 2), None); // generic Unlock
        assert_eq!(native_button_name("Kia V7", 1), None);
        assert_eq!(native_button_name("Toyota", 0xD1), Some("Estima"));
        assert_eq!(native_button_name("Toyota", 2), Some("Cmd 2"));
        assert_eq!(native_button_name("Porsche Cayenne", 0x08), Some("Open"));
        assert_eq!(native_button_name("Porsche Touareg [First]", 0x01), Some("Lock"));
        assert_eq!(native_button_name("VAG", 0x1), Some("Unlock"));
        assert_eq!(native_button_name("VAG", 0x20), Some("Lock"));
        assert_eq!(native_button_name("VAG", 0x40), Some("Trunk"));
        assert_eq!(native_button_name("Subaru", 0x4), Some("Panic"));
        assert_eq!(native_button_name("Chrysler V0", 0x2), None); // generic Unlock
        assert_eq!(native_button_name("Scher-Khan [MAGIC CODE PRO1]", 0x8), Some("Start"));
        assert_eq!(native_button_name("Star Line [SL_A2-A4]", 0x03), Some("Trunk"));
        assert_eq!(native_button_name("Star Line", 0x24), Some("Start"));
        assert_eq!(native_button_name("KeeLoq (DoorHan)", 0x2), Some("B1"));
        assert_eq!(native_button_name("KeeLoq (Sheriff)", 0x2), None);
        assert_eq!(native_button_name("PSA", 0), Some("Lock"));
        assert_eq!(native_button_name("PSA2", 2), Some("Trunk"));
    }

    /// Diagnostic: sweep every IMPORTS/*.sub and print which protocol(s) each decodes as.
    /// `#[ignore]`d (noisy, always passes); run with
    /// `cargo test sweep_imports_decodes -- --ignored --nocapture`.
    #[test]
    #[ignore]
    fn sweep_imports_decodes() {
        use std::fs;
        // Keystore as the app loads it at startup (Kia V3/V4's KeeLoq check, Kia V5/V6 decryption).
        crate::protocols::keys::load_keystore_from_embedded();
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
