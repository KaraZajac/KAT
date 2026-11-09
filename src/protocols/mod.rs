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
/// "Add Manually": new gate / garage remotes of a chosen type (Flipper-ARF / doctor-firmware).
pub mod add_manually;
pub mod flipper_key;
/// Native buttons of the gate / garage remotes ("Send button…", button labels).
pub mod gate_buttons;
/// Per-protocol metadata (line coding, RF, encryption, make, native buttons, TX presets).
pub mod info;
pub mod keeloq_common;
mod keeloq;
pub mod keeloq_bf;
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
mod honda_v5;
mod honda_v1;
mod honda_v2_v3;
mod honda_v4;
// Honda V6 (FoxFW): ported, not registered (no check; its only evidence is noise, see honda_v6.rs).
#[allow(dead_code)]
mod honda_v6;
mod honda_v7;
// Honda V8 (FoxFW): ported, not registered (its KeeLoq decrypt match passes about 0.5% of random
// frames and completes before longer frames end: it read a Nice One upload), see honda_v8.rs.
#[allow(dead_code)]
mod honda_v8;
mod honda_v9;
mod honda_v10;
#[cfg(test)]
mod honda_synth;
mod vag;
mod fiat_v0;
mod fiat_marelli;
mod fiat_v1;
mod hitag2;
pub mod hitag2_hell;
pub mod hitag2_keys;
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
mod opel_v0;
mod renault_v2;
// Renault V3 (FoxFW): kept, not registered. Its NRZ quantiser reads some BMW 128-bit frames (same 62 us
// chips) as a Renault V3 frame on an earlier pulse, so BMW never completes (the Mercedes V1
// `bmw_frames_are_never_claimed` test); see docs/renault_v3.md.
#[allow(dead_code)]
mod renault_v3;
mod renault_v4;
mod ducato_v0;
// Fiat Multi (FoxFW): kept, not registered. Its one evidence frame is a false match inside a VAG
// capture (Passat_2015.sub: it also takes a frame VAG decodes there); see docs/fiat_multi.md.
#[allow(dead_code)]
mod fiat_multi;
mod renault_v1;
mod renault_v0;
mod renault_v1_arf;
mod fiat_v2;
mod gm;
mod datsun_lada;
mod sheriff_cfm;
// Garage door / gate / barrier / shutter remotes (doctor-firmware's lib/subghz set). Each porting
// group adds its `mod` lines directly below its own anchor line.
// [gate-mods: G5 KeeLoq family]
mod g5_common;
mod beninca_arc;
mod kinggates_stylo_4k;
mod jarolift;
mod faac_slh;
// [gate-mods: G4 Security+ / Somfy]
mod g4_common;
mod secplus_v2;
mod somfy_keytis;
mod somfy_telis;
mod secplus_v1;
// [gate-mods: G6 Nice FloR-S / CAME Atomo / Alutech / Ditec]
mod alutech_at_4n;
mod came_atomo;
mod ditec_gol4;
mod g6_common;
mod nice_flor_s;
// [gate-mods: G3 misc]
// G3: Flipper-ARF garage-app copies (doctor-firmware for Urmet and Power Smart); shared helpers
// (Manchester encoder, reverse_key, upload builder) in g3_common.
mod g3_common;
mod power_smart;
mod urmet;
mod revers_rb2;
mod telcoma_edge;
mod phoenix_v2;
mod prastel;
mod dooya;
mod roger;
mod hay21;
mod allstar_firefly;
// [gate-mods: G2 static B]
mod g2_common;
mod marantec;
mod hormann;
mod nero_radio;
mod nero_sketch;
mod ido;
mod dickert_mahs;
mod doitrand;
mod mastercode;
mod marantec24;
mod clemsa;
mod bett;
mod elplast;
mod ansonic;
// [gate-mods: G1 static A]
mod g1_common;
mod holtek;
mod came_twee;
mod megacode;
mod chamberlain_code;
mod gate_tx;
mod holtek_ht12x;
mod came;
mod nice_flo;
mod linear;
mod linear_delta3;
mod smc5326;
mod princeton;
// Flipper firmware ports (Momentum, RogueMaster, doctor-firmware, Flipper-ARF); shared helpers in
// f_common.
mod f_common;
mod hormann_bisecur;
mod gangqi;
mod hollarm;
mod cardin_s508;
// RogueMaster ports (November 2026, from Unleashed): Freezer and Faac TM.
mod freezer;
mod faac_tm;
// Home / sensor / key-finder remotes the Flipper firmwares register (Flipper-ARF's garage app,
// doctor-firmware, Momentum, RogueMaster, FoxFW).
mod honeywell;
mod magellan;
mod x10;
mod honeywell_wdb;
mod legrand;
mod doorbell32;
mod intertechno_v3;
mod keyfinder;
mod treadmill37;
mod nord_ice;
mod feron;
mod keyfinder2;
// FoxFW (decode-only car remotes with structural gates):
mod vag_polo;
mod chevrolet_v0;
// FoxFW decoders (decode-only; FoxFW is the only reference that has them).
mod mitsubishi_v1;
mod ldv_v0;
mod lexus_v0;
mod mazda_v1;
mod nissan_v0;
// Toyota (FoxFW): FoxFW's "Toyota", Flipper-ARF's retired KeeLoq Toyota (decode-only).
mod toyota_foxfw;
mod tx_copies;
mod stream_locator;
#[cfg(test)]
mod dd_testutil;
// FoxFW decoders (decode only but BMW CAS3, which FoxFW's enc_bmw_cas3.c replays): Audi V1/V2/V3,
// BMW V2, BMW/Mini V3, BMW CAS3. Shared: foxfw_common.
mod foxfw_common;
#[allow(dead_code)] // ported from FoxFW, not registered (see the registry note)
mod audi_v2;
mod bmw_v2;
// Audi V1: kept, not registered. Its gate (3 ms sync, 2 ms header, 20 shorts, button nibble 0xE/0xB,
// non-zero serial) also passes KAT's own Fiat V1 transmissions, which share that air format
// (HITAG2 Manchester 250/500 µs behind a 3.5 ms sync and a 2 ms header): see docs/audi_v1.md.
#[allow(dead_code)]
mod audi_v1;
mod bmw_v3;
mod audi_v3;
mod bmw_cas3;

// rtl_433 ports (receive only), on rtl_433's receive chain (bit buffer, pulse detector, slicers)
// in rtl433. Car remotes:
mod rtl433;
mod alps_fwb1u545;
mod astrostart_2000;
mod audiovox_pro_oe3b;
mod chrysler_car_remote;
mod code_alarm_frdpc2002;
mod compustar_1wg3r;
mod siemens_5wy72xx;
mod honda_kr5v2x1x;
mod continental_kr5v2x;
mod mic6sc2;
mod opel_mokka;
// KeeLoq chips and shutter remotes:
// FoxFW decoders (Tesla Charge, Hyundai/Kia RIO and SantaFe 13-16 also transmit, with FoxFW's SD-card
// encoder plugins; Nidec RKE's plugin refuses every key):
mod tesla_charge;
mod hyundai_kia_rio;
mod santa_fe;
mod hitag2_rke;
mod nidec_rke;
mod elero;
mod somfy_iohc;
mod rojaflex;
mod microchip_hcs362;
mod microchip_hcs361;
// rtl_433 gate remote (27.195 MHz Cardin S466-TX2, receive only).
mod cardin_s466;
// Markisol: kept, not registered (rtl_433 ships markisol.c disabled: excessive false positives).
#[allow(dead_code)]
mod markisol;

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
pub use rtl433::{is_keeloq_chip, redecode_keeloq_chip};
pub use flipper_key::FlipperKey;
pub use tx_copies::transmission;
pub use stream_locator::{LocatedFrame, StreamLocator};
pub use info::{has_checksum_for, lookup as protocol_info, ProtocolInfo};
/// PSA TEA key recovery for the signal menu's "Recover key" action (ProtoPirate's brute force,
/// which the references also run on demand rather than while decoding).
pub use psa::{
    auto_key_search as psa_auto_key_search, needs_key_search as psa_needs_key_search,
    recover_key as psa_recover_key, search_label as psa_search_label, AutoKeySearch as PsaAutoKeySearch,
    BruteForceControl as PsaBruteForceControl, BruteForceHit as PsaBruteForceHit,
    BruteForceOutcome as PsaBruteForceOutcome, BRUTE_FORCE_TOTAL as PSA_BRUTE_FORCE_TOTAL,
    SEARCH_LABEL_ALL as PSA_SEARCH_LABEL_ALL, UNCHECKED_NAME as PSA_UNCHECKED_NAME,
};

/// Whether KAT already sends fresh frames of this HITAG2 capture (Fiat V1, Renault V1, Fiat V2: a
/// known, saved or serial-derived key reproduces its hop), so "Recover key (HiTag2)" has nothing
/// left to find. False for other protocols.
pub fn hitag2_key_known(protocol: &str, signal: &DecodedSignal) -> bool {
    match protocol {
        "Fiat V1" => fiat_v1::FiatV1Decoder::has_key(signal),
        "Renault V1" => renault_v1::RenaultV1Decoder::has_key(signal),
        "Fiat V2" => fiat_v2::FiatV2Decoder::has_key(signal),
        _ => false,
    }
}

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
/// `supported_frequencies()` for a decoder that runs on every band, like every Flipper decoder
/// (gate and garage remotes are sold for 300–318, 390, 418, 433 and 868 MHz).
pub const ANY_FREQUENCY: &[u32] = &[0];

/// Whether a capture at `frequency` goes to a decoder with these `supported_frequencies()`: within
/// 2 % of an entry (315 MHz covers ~308.7–321.3 MHz), or always when the list holds 0
/// ([ANY_FREQUENCY]).
pub fn frequency_supported(supported: &[u32], frequency: u32) -> bool {
    supported.iter().any(|&f| f == 0 || f.abs_diff(frequency) < f / 50)
}

/// Concrete frequencies to exercise a decoder at in tests ([ANY_FREQUENCY] → the main bands).
#[cfg(test)]
pub(crate) fn test_frequencies(supported: &[u32]) -> Vec<u32> {
    supported
        .iter()
        .flat_map(|&f| if f == 0 { vec![315_000_000, 433_920_000, 868_350_000] } else { vec![f] })
        .collect()
}

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

    /// The input ended (the end of a recording, or of a burst the receiver handed over): the
    /// frame this decoder holds back for a pulse that will not come now. Only decoders that report
    /// a complete frame later than their reference does (a KAT deviation, to give an earlier
    /// decoder priority or to see the burst end) hold one: Mazda V1, Mercedes V1, Renault V1, Fiat
    /// V2 and PSA's doctor-firmware path. A decoder whose reference also waits for the pulse after
    /// the frame keeps None, as on a Flipper.
    fn flush(&mut self) -> Option<DecodedSignal> {
        None
    }

    /// Check if this protocol supports encoding
    fn supports_encoding(&self) -> bool;

    /// Encode a signal with the given button command
    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>>;

    /// The decode of the frame `encode(decoded, button)` sends, for an encoder whose upload its
    /// own decoder cannot always read back. After a transmission the app keeps the frame sent on
    /// the capture (`app/tx.rs`); None: the app decodes the upload instead.
    fn sent_signal(&self, _decoded: &DecodedSignal, _button: u8) -> Option<DecodedSignal> {
        None
    }

    /// Rebuild a decode from a Flipper key file (`Protocol`/`Bit`/`Key` plus the protocol's own
    /// lines), as the reference's `deserialize` does. None when KAT cannot load this protocol's
    /// key files or the key does not validate.
    #[allow(clippy::wrong_self_convention)] // ~90 decoder impls plus callers; a rename would ripple
    fn from_flipper_key(&self, _key: &FlipperKey) -> Option<DecodedSignal> {
        None
    }

    /// The key file the reference's `serialize` writes for this decode (Protocol name as the
    /// reference spells it, Bit, Key and the protocol's own lines, in the reference's order), so a
    /// Flipper can load a KAT capture. `frequency`/`preset` are filled in by the caller. None when
    /// not supported. `from_flipper_key(to_flipper_key(d))` must give back `d`.
    fn to_flipper_key(&self, _decoded: &DecodedSignal) -> Option<FlipperKey> {
        None
    }

    /// Report the frames that only count once a key search decrypts them, under the protocol's
    /// `<name> [unchecked]` display name, instead of dropping them ([ProtocolRegistry::
    /// set_auto_key_search]). Only PSA has such frames (its adaptive AM path's TEA frames).
    fn set_report_unchecked(&mut self, _on: bool) {}

    /// The transmission a key file stands for, for a protocol whose frame does not fit
    /// [DecodedSignal] (Urmet: up to 40 bytes and 192 gaps). The capture loaded from the file keeps
    /// these pulses, so Replay sends what the reference's encoder would. None for every other
    /// protocol (they encode from the decode).
    fn key_file_pulses(&self, _key: &FlipperKey) -> Option<Vec<LevelDuration>> {
        None
    }

    /// The key file of a capture whose frame does not fit [DecodedSignal] (Urmet), rebuilt from
    /// the capture's recorded pulses. None for every other protocol (they write
    /// [Self::to_flipper_key]).
    fn key_file_from_pulses(&self, _pulses: &[LevelDuration]) -> Option<FlipperKey> {
        None
    }
}

/// Registry of all supported protocols
pub struct ProtocolRegistry {
    decoders: Vec<Box<dyn ProtocolDecoder>>,
    /// Each decoder's RF modulation (its `ProtocolInfo::rf`), for the modulation gate.
    rf: Vec<crate::capture::RfModulation>,
    /// Whether each decoder is a gate/garage protocol (`Category::Gate`): those take part in the
    /// inverted-polarity pass only for FSK input (see [Self::process_signal_stream_opts]).
    gate: Vec<bool>,
    /// Whether each decoder is an rtl_433 port ([RTL433_PORTS]): receive-only ports that run after
    /// every Flipper reference's decoder and never reset them (see
    /// [Self::process_signal_stream_inner]).
    rtl: Vec<bool>,
    /// A second set of the decoders, fed only by [Self::replays]: a decoder's state that survives
    /// `reset()` (repeat memories, `last_data`) must not see the frames replayed there.
    probe: Vec<Box<dyn ProtocolDecoder>>,
    /// Key searches run on their own (config `autobf`): [Self::set_auto_key_search].
    auto_key_search: bool,
}

/// The rtl_433 ports (receive only; registered last).
pub(crate) const RTL433_PORTS: &[&str] = &[
    "Elero", "Somfy-IOHC", "RojaFlex-Remote", "Microchip-HCS362", "Microchip-HCS361",
    "Honda-KR5V2X1X", "Continental-KR5V2X", "MIC6SC2-CarRemote", "Opel-Mokka",
    "Siemens-5WY72XX", "Alps-FWB1U545", "CodeAlarm-FRDPC2002", "Astrostart-2000",
    "Chrysler-CarRemote", "Compustar-1WG3R", "Audiovox-PROOE3B", "Cardin-S466",
];

/// `pairs[start..=end]` as stored on a capture.
fn segment(pairs: &[LevelDuration], start: usize, end: usize) -> Vec<LevelDuration> {
    pairs[start..=end].iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect()
}

/// Whether a decoder flagged `decoder_rf` may see a capture received as `capture_rf` (Flipper-ARF's
/// receiver modulation gate, after ProtoPirate's per-preset registry): an AM-only protocol is not
/// fed FM (2-FSK) captures and vice versa; protocols flagged for both, and captures whose modulation
/// is unknown, are not gated.
pub fn modulation_allows(decoder_rf: crate::capture::RfModulation, capture_rf: Option<crate::capture::RfModulation>) -> bool {
    use crate::capture::RfModulation::{AM, FM};
    !matches!((decoder_rf, capture_rf), (AM, Some(FM)) | (FM, Some(AM)))
}

/// Whether `frames` hold a decode, not counting `autobf`'s unchecked PSA frames
/// ([PSA_UNCHECKED_NAME]): they exist only while a key search decides on them, so they decide
/// neither which passes run nor what the other decoders read ([ProtocolRegistry::
/// process_signal_stream_inner]). With `autobf` on, every other decode is the same as with it off.
fn decodes_any(frames: &[(String, DecodedSignal, Vec<LevelDuration>)]) -> bool {
    frames.iter().any(|(name, _, _)| name != PSA_UNCHECKED_NAME)
}

/// Car protocols left out of the inverted-polarity pass (no reference has that pass). Ford V3's
/// EU path reads any ≥30-pulse preamble + 104 Manchester bits without a check, so a 240/480 µs
/// frame cut short (a squelch closing early, a stored segment of an older KAT) read inverted is a
/// "Ford V3" frame with nonsense fields, where every reference reports nothing.
/// LDV V0 is out of it too: its gate is a 48-bit run of equal Manchester bits plus a few fixed
/// fields, and an all-ones noise stretch read inverted (`honeywell_wdb_raw.sub`) passes it, where
/// the FoxFW decoder, which has no inverted pass, reports nothing.
const NO_INVERTED_PASS: &[&str] = &["Ford V3", "LDV V0"];

/// Decoders whose frames alone do not settle a burst in the modulation-gated first pass of KAT's own
/// receive paths ([ProtocolRegistry::process_signal_stream_path] / `process_signal_path`): the
/// ungated pass then runs as if nothing had decoded. Toyota (FoxFW) reads Kia V3 frames (its Prius air
/// format; Kia's lock / unlock buttons are in its list): on the AM path of an OOK Kia V3 fob it took
/// them before the ungated pass reached Kia V3/V4 (FM-flagged, KeeLoq-checked), which reads them
/// first there, as in FoxFW's own registry.
const NOT_FINAL_ON_GATED_PASS: &[&str] = &["Toyota (FoxFW)"];

impl ProtocolRegistry {
    /// Every decoder, in registry order (which decides first-match priority; the comments say why).
    fn decoders() -> Vec<Box<dyn ProtocolDecoder>> {
        vec![
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
            // the C) + US half-bit-cell path gated on 0xFF header / sane serial / button 1|2|6|7
            // (the C's flag-bit-7 and 1|2-only checks are dropped: real 2022 Escape frames fail
            // them, see ford_v3.rs). A US hit only fires at the end of a burst of ≥200 cells that
            // is exactly one frame; the IMPORTS sweep shows no change for the Ford V0/V1/V2 captures.
            Box::new(ford_v3::FordV3Decoder::new()),
            Box::new(ford_v2::FordV2Decoder::new()),
            // Honda V5 (FoxFW honda_v5.c): the same 63µs Manchester as Honda Static, CRC-8-gated.
            // It goes BEFORE Honda Static: Honda Static's unchecksummed bit-reversed fallback
            // misreads V5 frames (Lock_honda.sub / Unlock_honda.sub), see honda_v5.rs.
            Box::new(honda_v5::HondaV5Decoder::new()),
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
            // MARELLI (Flipper-ARF's pre-rewrite Fiat V1 file): key files load, save and replay; its
            // `feed` never reports, as FoxFW registers it without Decodable (Flipper-ARF's decoder
            // reads Fiat V1 and Ford V3 frames two bits early, see fiat_marelli.rs).
            Box::new(fiat_marelli::FiatMarelliDecoder::new()),
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
            // ("RobertBOSCH48331") plaintext tail 0x05; any button (a 2016 Holden Barina fob
            // sends 0x73). Decode-only (the C encoder is NULL).
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
            // preamble gate (behind fewer than 64 short pulses the frame must also pass `learn`;
            // the C encoder sends 125). Fiat V1 (Marelli PCF7946) carries the same 11-byte frame
            // shape in the opposite polarity with only a 29-toggle run before its sync, so the gate
            // keeps Datsun Lada off Fiat captures; placed last so Fiat V1 and every earlier
            // 250/500µs decoder keep priority.
            Box::new(datsun_lada::DatsunLadaDecoder::new()),
            // --- Garage door / gate / barrier / shutter remotes --------------------------------
            // After every car decoder (their IMPORTS decodes keep first-match priority), most
            // strongly gated first: rolling codes with a cipher/CRC check before loose fixed-code
            // formats (Princeton and other PT2262-style encoders last). Each porting group adds its
            // decoders directly below its own anchor line.
            // [gate-registry: G5 KeeLoq family]
            // Beninca ARC (Flipper-ARF garage app beninca_arc.c): 300/600µs PWM, 128 bits after a
            // 9600±2325µs LOW; AES-128 frame, reported only when it decrypts to the 0xAA55 constant.
            Box::new(beninca_arc::BenincaArcDecoder::new()),
            // KingGates Stylo4k: 400/1100µs LOW-first PWM behind a 2200µs header + 800µs start bit,
            // 89 bits; reported only when the KeeLoq hop passes the C's 16-bit check.
            Box::new(kinggates_stylo_4k::KingGatesStylo4kDecoder::new()),
            // Jarolift: KeeLoq PWM (400/800µs) behind exactly 8 preamble pulses + a 4000µs gap, 72
            // bits (KeeLoq's 64..66-bit window never takes them); reported only when the
            // normal-learning hop passes the C's 8-bit serial check.
            Box::new(jarolift::JaroliftDecoder::new()),
            // Faac SLH: 255/595µs PWM behind a 1190/1190µs preamble, 64 bits. Without the remote's
            // seed nothing can be checked: like the C, every such frame is reported (last here).
            Box::new(faac_slh::FaacSlhDecoder::new()),
            // [gate-registry: G4 Security+ / Somfy]
            // Security+ 2.0 (garage app secplus_v2.c + doctor-firmware's 86-bit keypad frames):
            // 250/500µs Manchester, armed only by a 65 ± 11 ms (or 89.5 ± 7.9 ms) LOW; a 0x3C-header
            // packet pair whose order/invert codes and trits decode (KAT gate). Reset is a no-op.
            Box::new(secplus_v2::SecPlusV2Decoder::new()),
            // Somfy Keytis / Telis (Somfy RTS): ≥2 HIGH/LOW 2560µs sync pairs + 4.5 ms sync, 640/1280µs
            // Manchester, exactly 80 / 56 bits + the 4-bit checksum (Keytis: this frame's, see module).
            // The bit counts keep the two apart.
            Box::new(somfy_keytis::SomfyKeytisDecoder::new()),
            Box::new(somfy_telis::SomfyTelisDecoder::new()),
            // Security+ 1.0: 2 ms ternary symbols in two 21-symbol packets behind 60 ± 12 ms LOWs; no
            // checksum (structure only, as the C), so last in the block. Reset is a no-op.
            Box::new(secplus_v1::SecPlusV1Decoder::new()),
            // [gate-registry: G6 Nice FloR-S / CAME Atomo / Alutech / Ditec]
            // G6, most strongly gated first. Alutech AT-4N: 400/800µs PWM, >9-pair preamble + 4ms
            // gap, exactly 72 bits, KAT reports CRC-8-valid frames only. Nice FloR-S / Nice One:
            // 500/1000µs PWM framed by a ~19ms gap + 1500/1500µs start and a 1500µs stop bit,
            // exactly 52 or 72 bits. Ditec GOL4: 400/1100µs PWM, 24ms gap + 800µs start, exactly 54
            // bits, reported only when the next gap + start follow. CAME Atomo (weakest): 600/1200µs
            // Manchester after a 12/19/72ms LOW, exactly 61 bits up to a LOW of >=2650µs.
            Box::new(alutech_at_4n::AlutechAt4nDecoder::new()),
            Box::new(nice_flor_s::NiceFlorSDecoder::new()),
            Box::new(ditec_gol4::DitecGol4Decoder::new()),
            Box::new(came_atomo::CameAtomoDecoder::new()),
            // [gate-registry: G3 misc]
            // G3, most strongly gated first. None fires on IMPORTS, the other protocols' Flipper
            // unit-test captures, the UberGuidoZ garage/gate set or the rtl_433 recordings.
            // Power Smart: 225/450 µs Manchester, sync bytes 0xFD/0xAA + inverted-copy check.
            Box::new(power_smart::PowerSmartDecoder::new()),
            // Urmet: Manchester wake-up burst + 20–60 ms pause + PPM preamble, 16..40 USART bytes
            // all in the 5/7/D/F nibble alphabet. Decode only (see urmet.rs).
            Box::new(urmet::UrmetDecoder::new()),
            // Revers RB2: 250/500 µs Manchester, 64 bits with 0xFF at bits 48..55 + stop code 0x200.
            Box::new(revers_rb2::ReversRb2Decoder::new()),
            // Telcoma/Cardin EDGE: 1270/2540 µs Manchester burst of exactly 32/33 bits, top byte 0xFF.
            Box::new(telcoma_edge::TelcomaEdgeDecoder::new()),
            // Phoenix V2: 25.6 ms header + 2.56 ms start pulse, exactly 52 bits (427/853 µs).
            Box::new(phoenix_v2::PhoenixV2Decoder::new()),
            // Dooya: 8.8 ms header + 4.76 ms / 1.47 ms start, exactly 40 bits (366/733 µs).
            Box::new(dooya::DooyaDecoder::new()),
            // Roger, Hay21, Allstar Firefly: a frame between two gaps (9.5 / 4.2 / 30 ms), exactly 28 /
            // 21 / 18 bits, no checksum — the loosest, last.
            Box::new(roger::RogerDecoder::new()),
            Box::new(hay21::Hay21Decoder::new()),
            Box::new(allstar_firefly::AllstarFireflyDecoder::new()),
            // [gate-registry: G2 static B]
            // G2 static B (doctor-firmware / Flipper-ARF garage-app fixed codes), most strongly
            // gated first. Marantec: 10 ms header + Manchester, 49 bits, CRC-8 (KAT reports only
            // valid frames). Hormann HSM: 12 ms start pulse, frame ends only on the 0xFF…3
            // pattern. Nero Radio / Nero Sketch: > 40 preamble pairs + start pulse, 56|57 / 40 bits.
            // iDo 117/111: 4.5 ms HIGH + LOW preamble, ≥ 48 bits (decode-only).
            Box::new(marantec::MarantecDecoder::new()),
            Box::new(hormann::HormannDecoder::new()),
            Box::new(nero_radio::NeroRadioDecoder::new()),
            Box::new(nero_sketch::NeroSketchDecoder::new()),
            Box::new(ido::IdoDecoder::new()),
            // Header-delimited PWM without checksum: Dickert MAHS (40 ms gap, 1200 µs bit pairs,
            // ≥ 36 bits), Doitrand (24.8 ms gap + start pulse, 37 bits), Mastercode (36 bits).
            Box::new(dickert_mahs::DickertMahsDecoder::new()),
            Box::new(doitrand::DoitrandDecoder::new()),
            Box::new(mastercode::MastercodeDecoder::new()),
            // Short gap-delimited PWM fixed codes, longest first; Ansonic (≥ 12 bits) last.
            Box::new(marantec24::Marantec24Decoder::new()),
            Box::new(clemsa::ClemsaDecoder::new()),
            Box::new(bett::BettDecoder::new()),
            Box::new(elplast::ElplastDecoder::new()),
            Box::new(ansonic::AnsonicDecoder::new()),
            // Flipper firmware ports.
            // Hörmann BiSecur (Momentum / RogueMaster): FM 208/416 µs Manchester behind a 21-pair
            // preamble, a 1664 µs HIGH and a 3-pulse 416 µs sync; exactly 176 bits, reported only when
            // the frame's CRC-8 (per type 0x50 / 0x70) holds.
            Box::new(hormann_bisecur::HormannBiSecurDecoder::new()),
            // GangQi (doctor-firmware / Flipper-ARF): AM 500/1200 µs PWM between ~2.4 ms gaps, exactly
            // 34 bits, reported only when the byte sum (the C's displayed type-1/type-2 sums) holds.
            // Before Hollarm, as every reference registers them; no G1 frame ends on the same pulse
            // with 34 bits (the references register the G1 protocols first).
            Box::new(gangqi::GangQiDecoder::new()),
            // Hollarm (Flipper-ARF = Momentum / RogueMaster copy): AM HIGH 200 µs + LOW 1000/1600 µs
            // bits after a 2.4 ms gap, exactly 42 bits with the C's byte-sum check.
            Box::new(hollarm::HollarmDecoder::new()),
            // Cardin S508 (RogueMaster, decode-only): FM ~100/200 µs Manchester with per-level bands,
            // a 12-bit sync then 128 opaque AES bits, no checksum — the loosest of the block, last
            // (RogueMaster also registers it last).
            Box::new(cardin_s508::CardinS508Decoder::new()),

            // [gate-registry: G1 static A]
            // G1 static fixed codes (Flipper-ARF garage app), most strongly gated first. Only frames
            // completing on the same pulse depend on this order. Holtek: 40 bits + 0x5 header nibble.
            Box::new(holtek::HoltekDecoder::new()),
            // CAME TWEE: 54-bit Manchester parcels behind a 51 ms (or 12 ms) header.
            Box::new(came_twee::CameTweeDecoder::new()),
            // MegaCode: 24 one-pulse 6 ms slots behind a 13 ms LOW, start bit 1.
            Box::new(megacode::MegaCodeDecoder::new()),
            // Cham_Code: 10/11 symbols that must match a 7/8/9-DIP stop-symbol mask.
            Box::new(chamberlain_code::ChamberlainCodeDecoder::new()),
            // GateTX: exactly 24 bits behind a 16 ms header and a 700 µs start bit.
            Box::new(gate_tx::GateTxDecoder::new()),
            // Holtek_HT12X: 12 bits confirmed by a repeat. It shares CAME's 320/640 µs air format (an
            // HT12X frame after a 9..13 ms header is also a CAME 12-bit frame), so it stays BEFORE
            // CAME, as in the garage app's registry (group g1 before g5).
            Box::new(holtek_ht12x::HoltekHt12xDecoder::new()),
            // CAME: exactly 12/18/24/25/42 bits, no repeat check.
            Box::new(came::CameDecoder::new()),
            // Prastel (G3's module): CAME timing (320/640 µs), exactly 42 bits. A 42-bit frame also
            // completes CAME (its "CAME [Prastel]" variant) on the same pulse; both references
            // register CAME first (doctor-firmware: came before prastel; the garage app: group g5
            // before g12), so CAME reports those frames and Prastel only what CAME does not take
            // (its wider header-gap window).
            Box::new(prastel::PrastelDecoder::new()),
            // Nice FLO: >= 12 bits at 700/1400 µs behind a 25 ms header.
            Box::new(nice_flo::NiceFloDecoder::new()),
            // Linear: 10 bits ending on a 21 ms guard.
            Box::new(linear::LinearDecoder::new()),
            // LinearDelta3: 8 bits, confirmed by a repeat.
            Box::new(linear_delta3::LinearDelta3Decoder::new()),
            // SMC5326: 25 bits, confirmed by a repeat.
            Box::new(smc5326::Smc5326Decoder::new()),
            // Princeton (PT2262/EV1527-style 24 bits, confirmed by a repeat): the loosest of the
            // stock fixed codes.
            Box::new(princeton::PrincetonDecoder::new()),
            // RogueMaster's two newer fixed codes, after the stock ones as RogueMaster registers
            // them (last, Freezer then Faac TM). Freezer: 24 bits in 250/500 µs cells between gaps
            // ≥ 7.5 ms, reported only when a frame repeats; symbols over 700 µs are refused, so
            // Princeton frames never complete it. Faac TM: 12 DIP bits in CAME's grammar at a
            // measured Te (nominally 400 µs) behind a 35-Te header; it must stay AFTER CAME, which
            // keeps any 12-bit frame closed by its own 47-Te gap and leaves slower ones with
            // another gap to Faac TM (RogueMaster's foreign-frame rule, see came.rs).
            Box::new(freezer::FreezerDecoder::new()),
            Box::new(faac_tm::FaacTmDecoder::new()),
            // Home / sensor / key-finder remotes (Flipper-ARF's garage app registers X10, Doorbell32,
            // KeyFinder, KeyFinder2 and Nord ICE; doctor-firmware, Momentum, RogueMaster and FoxFW the
            // stock Honeywell, Honeywell Sec, Magellan, Legrand, Intertechno V3, Feron and Treadmill37
            // too), strongest check first:
            // - Honeywell Sec (security sensors): a CRC-16 behind a 14..16-bit Manchester preamble;
            // - Magellan (Paradox sensors): 32 bits with a CRC-8 behind a >10-pair preamble;
            // - X10: a 9.6 ms + 4.8 ms preamble, exactly 32 bits whose bytes 2 and 4 complement 1 and 3;
            // - Honeywell (doorbells / PIR): 48 bits between 480 µs marks with a parity bit;
            // - Legrand: 18 bits at 375/1125 µs after a 6 ms sync, reported only when a frame repeats;
            // - Doorbell32: 32 bits of 515/1545 µs PWM after a 14-Te guard, reported only when the frame
            //   repeats (Princeton drops these frames on its bit count);
            // - Intertechno V3: exactly 32 / 36 four-pulse cells behind a 10·T start bit;
            // - KeyFinder: 24 bits of 400/1200 µs PWM after a 4 ms gap, closed by four short HIGHs;
            // - Treadmill37 (37 bits) and Nord ICE (33 bits) between gaps, no check;
            // - Feron: exactly 32 bits between a 4.5 ms gap and a 500 µs end marker, no check;
            // - KeyFinder2: 3 wide + 8 narrow PWM bits and no check at all: the loosest, last.
            Box::new(honeywell::HoneywellDecoder::new()),
            Box::new(magellan::MagellanDecoder::new()),
            Box::new(x10::X10Decoder::new()),
            Box::new(honeywell_wdb::HoneywellWdbDecoder::new()),
            Box::new(legrand::LegrandDecoder::new()),
            Box::new(doorbell32::Doorbell32Decoder::new()),
            Box::new(intertechno_v3::IntertechnoV3Decoder::new()),
            Box::new(keyfinder::KeyFinderDecoder::new()),
            Box::new(treadmill37::Treadmill37Decoder::new()),
            Box::new(nord_ice::NordIceDecoder::new()),
            Box::new(feron::FeronDecoder::new()),
            Box::new(keyfinder2::KeyFinder2Decoder::new()),
            // FoxFW's decode-only car remotes (VW Polo 5, GM Open/Close), after every ProtoPirate, ARF,
            // doctor, Momentum and RogueMaster decoder and before the rtl_433 ports. Structural gates
            // only (no CRC), so they stay behind the checked decoders: VAG Polo wants 32+ cleartext
            // ones, the sync 0x3F91 and 80 Manchester bits at 300/590 us; Chevrolet V0 the exact
            // preamble 0xAAAAAAAB and 96 bits at 120/240 us.
            Box::new(vag_polo::VagPoloDecoder::new()),
            Box::new(chevrolet_v0::ChevroletV0Decoder::new()),
            // FoxFW decoders, after every Flipper-reference decoder and before the rtl_433 ports.
            // Strongest gate first: Tesla Charge (one fixed 50-bit code), Hyundai/Kia RIO (16-bit
            // checksum behind a 10.4 ms sync), SantaFe 13-16 (CRC-8 behind a 12 ms sync), Hitag2 (XOR check
            // + 0x0001 marker on a Manchester burst), Nidec RKE (structural gate only, no check: last).
            Box::new(tesla_charge::TeslaChargeDecoder::new()),
            Box::new(hyundai_kia_rio::HyundaiKiaRioDecoder::new()),
            Box::new(santa_fe::SantaFeDecoder::new()),
            Box::new(hitag2_rke::Hitag2RkeDecoder::new()),
            Box::new(nidec_rke::NidecRkeDecoder::new()),
            // FoxFW decoders (receive only), after every Flipper reference and before the rtl_433
            // ports; the ones with a check first, the loosest last.
            // Renault/xHorse V4 (FoxFW): Manchester 125/250 µs behind an 800 µs+ HIGH, 88 bits whose 11 bytes
            // XOR to 0xFF, button nibble whitelist and pad invariant.
            Box::new(renault_v4::RenaultV4Decoder::new()),
            // Fiat Ducato V0 (FoxFW): 200/400 µs Manchester behind 120+ short pulses and an 800 µs sync,
            // 71 bits with a 1-bit CRC check (a failing frame is still reported, crc false).
            Box::new(ducato_v0::DucatoV0Decoder::new()),
            // Renault V2 (FoxFW): exact 0x71EA sync + 82 bits, nothing in the clear.
            Box::new(renault_v2::RenaultV2Decoder::new()),
            // Opel V0 (FoxFW, experimental): carrier-burst widths, exactly 62..66 bursts; the loosest.
            Box::new(opel_v0::OpelV0Decoder::new()),
            // FoxFW's Honda decoders (decode only), after every other Flipper decoder, checked ones
            // first: V4 (inverse-byte invariant), V7 (XOR checksum), V9 (XOR checksum, Hamming <= 1),
            // HondaV2&V3 (V3: CRC; V2: button pair), then V10 (flags / button gate only). Honda V6
            // (no check, matches noise in Toyota / Opel / Subaru captures and would take a Fiat V1
            // frame) and Honda V8 (a KeeLoq decrypt match under 21 keys that about 0.5% of random
            // frames pass, and that completes before a Nice One frame ends: it read one) are ported
            // but not registered, see honda_v6.rs and honda_v8.rs.
            Box::new(honda_v4::HondaV4Decoder::new()),
            Box::new(honda_v7::HondaV7Decoder::new()),
            Box::new(honda_v9::HondaV9Decoder::new()),
            Box::new(honda_v2_v3::HondaV2V3Decoder::new()),
            Box::new(honda_v10::HondaV10Decoder::new()),
            // FoxFW decoders (decode-only, no checksum), after every Flipper reference's decoders,
            // strongest gate first. Mitsubishi V1: 200/400 µs FSK Manchester, 32-bit sync 0x1E4C11A7 +
            // fix byte 0xE0.
            Box::new(mitsubishi_v1::MitsubishiV1Decoder::new()),
            // LDV V0: 250/500 µs Manchester, >= 48-bit preamble, 128 bits with fixed 0xFF 0xFF bytes
            // and a duplicated button. Behind Ford V3, which reads the same fob's frames first.
            Box::new(ldv_v0::LdvV0Decoder::new()),
            // Lexus V0: ~350/700 µs, >= 16 preamble pulses, 224 bits, no check (a complement
            // signature across frames marks verified ones).
            Box::new(lexus_v0::LexusV0Decoder::new()),
            // Mazda V1: 505/1010 µs, 0xEC sync scanned at both polarities, button whitelist, 71 bits.
            // Behind Mazda V2, which keeps every frame both read.
            Box::new(mazda_v1::MazdaV1Decoder::new()),
            // Nissan V0: 500/1000 µs differential Manchester, a 12-bit structural lock (sync
            // 110001 + marker 000010) on 64 bits.
            Box::new(nissan_v0::NissanV0Decoder::new()),
            // Toyota (FoxFW) (FoxFW's "Toyota" = Flipper-ARF's retired KeeLoq Toyota, toyota-old.c.bak):
            // 400/800 µs PWM on the HIGH behind >= 6 preamble pairs and a 1050-1500 µs LOW (Prius), or
            // 67 NRZ pulses behind >= 10 pairs and a 1.7-2.2 ms LOW (Tundra). Its frame check is a
            // plausibility gate only (a 4-of-16 button list, hop / serial not all zeros or ones), weaker
            // than Nissan V0's 12-bit lock: last of the block. Kia V3 frames have the Prius air format:
            // Kia V3/V4 (FM-flagged, first in the registry) keeps them, see NOT_FINAL_ON_GATED_PASS.
            Box::new(toyota_foxfw::ToyotaFoxfwDecoder::new()),
            // FoxFW decoders (receive only but BMW CAS3, which replays), after every Flipper-ARF /
            // doctor-firmware / Momentum decoder; the one with a check first, then by how tightly each
            // gates its frame.
            // Audi V2 (FoxFW) is ported but not registered: its only check, a 0x93 trailer byte, also
            // fits 1 of 23 frames of an unrelated VW Atlas capture (see docs/audi_v2.md).
            // BMW V2: AM Manchester 500/1000 µs, 56 bits behind a fixed 0xCF header and 0x3A trailer.
            Box::new(bmw_v2::BmwV2Decoder::new()),
            // BMW/Mini V3: 2-FSK Manchester 95/190 µs, 0xCC header then 64 bits, button 02/04/08/10.
            Box::new(bmw_v3::BmwV3Decoder::new()),
            // Audi V3: 2-FSK half-bit pulses ~383/766 µs, 96 bits, button 0x0C/0x0D, hop not 00/FF.
            Box::new(audi_v3::AudiV3Decoder::new()),
            // BMW CAS3: PPM, 250 µs marks and 500/1500 µs spaces after two 10 ms intervals, 64 to
            // 128 bits with no check at all: the loosest, last.
            Box::new(bmw_cas3::BmwCas3Decoder::new()),
            // rtl_433 ports (receive only) come last: the Flipper references' decoders keep
            // first-match priority on any frame both read.
            // KeeLoq chips and shutter remotes, strongest gate first.
            // Elero (rtl_433 elero.c): FSK 13 µs NRZ, 31-bit sync, PN9-whitened frame, CRC-16 (strongest first).
            Box::new(elero::EleroDecoder::new()),
            // Somfy io-homecontrol (rtl_433 somfy_iohc.c): FSK 26 µs UART frame behind 0x57FD99, CRC-16.
            Box::new(somfy_iohc::SomfyIohcDecoder::new()),
            // RojaFlex (rtl_433 rojaflex.c): FSK 100 µs NRZ, 48-bit sync, 88 bits + CRC-16 or a 72-bit bridge frame.
            Box::new(rojaflex::RojaflexDecoder::new()),
            // Microchip HCS362 (rtl_433 hcs362.c): PWM 200/400 or MC 214 µs, preamble checks, 69 bits, 2-bit CRC.
            Box::new(microchip_hcs362::MicrochipHcs362Decoder::new()),
            // Microchip HCS361 (rtl_433 hcs361.c, six PWM/PIWM r_devices): 67 bits, 2-bit CRC (weakest: last).
            Box::new(microchip_hcs361::MicrochipHcs361Decoder::new()),
            // Cardin S466-TX2 garage remote (rtl_433 cardin.c, 27.195 MHz): PWM 730/1400 µs behind a
            // 6150 µs sync, exactly 24 bits, button code 0x03/0x06/0x09/0x0C and valid tri-state DIP pairs.
            Box::new(cardin_s466::CardinS466Decoder::new()),

            // Car remotes on the Manchester zero-bit slicer, reported at a LOW over their reset limit,
            // strongest gate first. Honda-KR5V2X1X: FSK 60 µs Manchester, 150..184-bit message,
            // EC0F62 + CRC-8.
            Box::new(honda_kr5v2x1x::HondaKr5v2x1xDecoder::new()),
            // Continental-KR5V2X: FSK 100 µs Manchester, preamble F0F06 + 112 bits, sanity + XOR.
            Box::new(continental_kr5v2x::ContinentalKr5v2xDecoder::new()),
            // MIC6SC2-CarRemote: OOK 250 µs Manchester, 48..88-bit row starting 5554, XOR.
            Box::new(mic6sc2::Mic6sc2Decoder::new()),
            // Opel-Mokka: FSK 100 µs Manchester, exactly 268 bits, 88 zeros + the key ID twice.
            Box::new(opel_mokka::OpelMokkaDecoder::new()),

            // Car remotes, most strongly gated first; each reports where rtl_433 would output, mostly
            // on the gap that ends its message.
            // Siemens 5WY72XX: zerobit Manchester 220 µs, one row ≥ 113 bits, 0x6001 + 96 bits, XOR = 0.
            Box::new(siemens_5wy72xx::Siemens5wy72xxDecoder::new()),
            // Alps FWB1U545: FSK zerobit Manchester 500 µs, exactly 76 bits, 0x55 byte + repeated button byte.
            Box::new(alps_fwb1u545::AlpsFwb1u545Decoder::new()),
            // Code Alarm FRDPC2002: zerobit Manchester 550 µs ± 100, exactly 60 bits, 16 leading zero bits.
            Box::new(code_alarm_frdpc2002::CodeAlarmFrdpc2002Decoder::new()),
            // Astrostart 2000: gap PPM 326/526 µs, exactly 52 bits, inverse button byte + 4-bit ID checksum.
            Box::new(astrostart_2000::Astrostart2000Decoder::new()),
            // Chrysler car remote: PWM 350/700 µs + 7.35 ms sync, 48-bit rows, 8-bit sum + button rules.
            Box::new(chrysler_car_remote::ChryslerCarRemoteDecoder::new()),
            // Compustar 1WG3R: PWM 708/1076 µs + syncs, 35-bit rows, 111 + inverse button (= Scher-Khan's 35-bit Static frames).
            Box::new(compustar_1wg3r::Compustar1wg3rDecoder::new()),
            // Audiovox PRO-OE3B: PWM 445/895 µs, exactly 25 bits, fixed bit pattern + a button flag (loosest).
            Box::new(audiovox_pro_oe3b::AudiovoxProOe3bDecoder::new()),

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
            //   Box::new(renault_v3::RenaultV3Decoder::new()),
            //       FoxFW's Renault V3 (2-FSK NRZ 52 µs, sync 0xC969/0xCB2D): the BMW wire test shows it
            //       completing on BMW's own 128-bit frames before BMW does, so it stays out.
            //   Box::new(fiat_multi::FiatMultiDecoder::new()),
            //       FoxFW's Fiat V3/V5: it reads a VAG frame of Passat_2015.sub as a V5 frame and takes
            //       a second VAG frame of that file with it (kat_diff), so it stays out until a real
            //       Fiat V3/V5 capture shows what gate it needs.
            //   Box::new(hyundai::HyundaiDecoder::new()),
            //       500/1000µs Manchester, 72 bits; the frame has NO checksum, so validity rests on
            //       framing alone (16..32-byte 0x55 preamble + duplicate frame).
        ]
    }

    /// Create a new protocol registry with all built-in protocols
    pub fn new() -> Self {
        let decoders = Self::decoders();
        let probe = Self::decoders();
        let rf = decoders
            .iter()
            .map(|d| info::lookup(d.name()).map_or(crate::capture::RfModulation::Both, |i| i.rf))
            .collect();
        let gate = decoders
            .iter()
            .map(|d| info::lookup(d.name()).is_some_and(|i| i.category == info::Category::Gate))
            .collect();
        let rtl = decoders.iter().map(|d| RTL433_PORTS.contains(&d.name())).collect();
        Self { decoders, probe, rf, gate, rtl, auto_key_search: false }
    }

    /// Process level+duration pairs from demodulator
    /// Returns decoded signal info if any protocol matches.
    /// Tries normal polarity first, then inverted polarity (so OOK captures where
    /// carrier-on is recorded as LOW can still decode as Fiat/Ford etc.).
    pub fn process_signal(&mut self, pairs: &[LevelDuration], frequency: u32) -> Option<(String, DecodedSignal)> {
        self.process_signal_rf(pairs, frequency, None)
    }

    /// [Self::process_signal] for pulses from one of KAT's receive paths: gated first, then
    /// ungated (see [Self::process_signal_stream_path]).
    pub fn process_signal_path(
        &mut self,
        pairs: &[LevelDuration],
        frequency: u32,
        path: Option<crate::capture::RfModulation>,
    ) -> Option<(String, DecodedSignal)> {
        let fsk = path == Some(crate::capture::RfModulation::FM);
        let gated = self.process_signal_opts(pairs, frequency, path, fsk);
        if path.is_none() || gated.as_ref().is_some_and(|(n, _)| !NOT_FINAL_ON_GATED_PASS.contains(&n.as_str())) {
            return gated;
        }
        self.process_signal_opts(pairs, frequency, None, fsk).or(gated)
    }

    /// [Self::process_signal] for a capture received as `rf` (see [modulation_allows]).
    pub fn process_signal_rf(
        &mut self,
        pairs: &[LevelDuration],
        frequency: u32,
        rf: Option<crate::capture::RfModulation>,
    ) -> Option<(String, DecodedSignal)> {
        self.process_signal_opts(pairs, frequency, rf, rf == Some(crate::capture::RfModulation::FM))
    }

    /// See [Self::process_signal_stream_opts].
    fn process_signal_opts(
        &mut self,
        pairs: &[LevelDuration],
        frequency: u32,
        rf: Option<crate::capture::RfModulation>,
        fsk: bool,
    ) -> Option<(String, DecodedSignal)> {
        // Try normal polarity first
        if let Some(result) = self.process_signal_inner(pairs, frequency, false, rf, fsk) {
            return Some(result);
        }
        // Try inverted polarity (capture LOW = RF HIGH)
        if let Some(result) = self.process_signal_inner(pairs, frequency, true, rf, fsk) {
            return Some(result);
        }
        // No known protocol: try KeeLoq generic (uses keeloq_common with every keystore key)
        keeloq_generic::try_decode(pairs, frequency)
    }

    /// ProtoPirate-style streaming decode: feed the whole stream, on each decode record and reset decoders, continue.
    /// Returns one entry per decode: (protocol name, decoded signal, pairs that produced it).
    /// Tries normal polarity first; if no decodes, runs again with inverted polarity.
    #[allow(dead_code)] // the app passes the modulation; tests and tools use the ungated form
    pub fn process_signal_stream(
        &mut self,
        pairs: &[LevelDuration],
        frequency: u32,
    ) -> Vec<(String, DecodedSignal, Vec<LevelDuration>)> {
        self.process_signal_stream_rf(pairs, frequency, None)
    }

    /// [Self::process_signal_stream] for pulses from one of KAT's own receive paths (live RX, IQ
    /// files): the receiver splits each burst into an AM and/or an FM path itself, and protocol
    /// flags are approximations (real OOK Hyundai fobs send Kia V3 frames, which the references
    /// flag FM), so protocols flagged for the path's modulation are tried first and, when none
    /// decodes (or only [NOT_FINAL_ON_GATED_PASS] does), every protocol is
    /// ([Self::process_signal_stream_rf] is the strict, preset-based gate Flipper-ARF applies).
    pub fn process_signal_stream_path(
        &mut self,
        pairs: &[LevelDuration],
        frequency: u32,
        path: Option<crate::capture::RfModulation>,
    ) -> Vec<(String, DecodedSignal, Vec<LevelDuration>)> {
        let fsk = path == Some(crate::capture::RfModulation::FM);
        let gated = self.process_signal_stream_opts(pairs, frequency, path, fsk);
        let settled = gated
            .iter()
            .any(|(n, _, _)| n != PSA_UNCHECKED_NAME && !NOT_FINAL_ON_GATED_PASS.contains(&n.as_str()));
        if settled || path.is_none() {
            return gated;
        }
        let ungated = self.process_signal_stream_opts(pairs, frequency, None, fsk);
        if decodes_any(&ungated) || gated.is_empty() {
            ungated
        } else {
            gated
        }
    }

    /// [Self::process_signal_stream] for a capture received as `rf` (AM/OOK or FM/2-FSK; None =
    /// unknown): decoders whose protocol is flagged for the other modulation only are not fed
    /// ([modulation_allows]). This is Flipper-ARF's gate, which follows the receive preset; use it
    /// for `.sub` files (their preset), [Self::process_signal_stream_path] for KAT's own receiver.
    pub fn process_signal_stream_rf(
        &mut self,
        pairs: &[LevelDuration],
        frequency: u32,
        rf: Option<crate::capture::RfModulation>,
    ) -> Vec<(String, DecodedSignal, Vec<LevelDuration>)> {
        self.process_signal_stream_opts(pairs, frequency, rf, rf == Some(crate::capture::RfModulation::FM))
    }

    /// Normal polarity, then (if nothing decoded) inverted. `fsk`: the pulses come from an FSK
    /// discriminator, where which tone is "on" depends on the fob, so the gate/garage protocols
    /// take part in the inverted pass too (for OOK the carrier is HIGH, and only the car protocols
    /// keep the inverted pass, see [Self::process_signal_stream]).
    fn process_signal_stream_opts(
        &mut self,
        pairs: &[LevelDuration],
        frequency: u32,
        rf: Option<crate::capture::RfModulation>,
        fsk: bool,
    ) -> Vec<(String, DecodedSignal, Vec<LevelDuration>)> {
        let with_normal = self.process_signal_stream_inner(pairs, frequency, false, rf, fsk);
        if decodes_any(&with_normal) {
            return with_normal;
        }
        let with_inverted = self.process_signal_stream_inner(pairs, frequency, true, rf, fsk);
        // `autobf`'s unchecked frames are not a decode here ([decodes_any]); the normal pass's are
        // kept when the inverted pass decodes nothing either.
        if decodes_any(&with_inverted) || with_normal.is_empty() {
            with_inverted
        } else {
            with_normal
        }
    }

    /// Inner streaming decode with optional level inversion.
    fn process_signal_stream_inner(
        &mut self,
        pairs: &[LevelDuration],
        frequency: u32,
        invert_level: bool,
        rf: Option<crate::capture::RfModulation>,
        fsk: bool,
    ) -> Vec<(String, DecodedSignal, Vec<LevelDuration>)> {
        // The inverted-polarity pass on OOK/unknown input (a capture whose carrier-on was recorded
        // as LOW) is a KAT addition for the strongly-gated car protocols: the Flipper has no such
        // pass, and the gate/garage formats (many of them loose fixed codes such as Princeton or
        // CAME) would turn inverted noise into frames. On FSK input either tone can be "on", so
        // they take part there (a real FSK PT2262 remote in rtl_433's tests needs it).
        let allowed = self.allowed(rf, invert_level, fsk);
        // Each frame with the pulse that completed it, which puts them in stream order at the end
        // (a held rtl_433 frame is added after frames that completed later).
        let mut out: Vec<(usize, String, DecodedSignal, Vec<LevelDuration>)> = Vec::new();
        let mut segment_start = 0_usize;
        // A frame an rtl_433 port read, held back until it is clear that no Flipper decoder reads
        // the signal: (port, first pulse, last pulse, name, decode). Once a Flipper decoder has
        // read the signal a port held a frame of, that port's later frames of this stream are
        // dropped too (the frames the Flipper decoder cannot take, e.g. the last of a press whose
        // header it needs first, would otherwise be listed under the port's name).
        let mut pending: Option<(usize, usize, usize, String, DecodedSignal)> = None;
        let mut shadowed = vec![false; self.decoders.len()];
        // `autobf`'s unchecked PSA frames ([PSA_UNCHECKED_NAME]: frames that count only if the key
        // search finds their key, and are dropped otherwise) are reported for that search but are
        // no hit: they reset no decoder, leave a held rtl_433 frame alone and take no pulses from
        // the frames around them, so every other decode is the same with `autobf` on or off (with
        // it off they do not exist). Their own pulses start after the previous unchecked frame's.
        let mut unchecked_start = 0_usize;

        for decoder in &mut self.decoders {
            decoder.reset();
        }
        let Some(last) = pairs.len().checked_sub(1) else { return Vec::new() };

        // Step `pairs.len()` is the end of the input: a decoder that holds a complete frame back
        // for a pulse that will not come reports it ([ProtocolDecoder::flush]).
        for i in 0..=pairs.len() {
            let pulse = pairs.get(i).map(|p| (if invert_level { !p.level } else { p.level }, p.duration_us));
            // The last pulse of a frame completed on this step.
            let at = i.min(last);

            // Feed this pulse to every decoder that supports this frequency and modulation, in
            // registry order; the first that completes a frame on it wins. As on a Flipper, where
            // the winner's callback resets the receiver before the decoders after it are fed the
            // pulse, every decoder is reset right then and the later ones see the pulse in their
            // reset state (a frame-ending gap is the next frame's header to them). A decoder whose C
            // sets its state after the callback (inside which that reset runs), e.g. a sync that
            // also opens the next frame, keeps that state through this reset with a one-shot
            // `just_emitted` flag (Legrand, Renault V0, BMW CAS3, Toyota (FoxFW)). An rtl_433 port's
            // hit resets the rtl_433 ports only: rtl_433 runs its devices independently, and the
            // Flipper references keep first-match priority over any frame both read.
            let mut hit: Option<(usize, String, DecodedSignal)> = None;
            let mut unchecked: Option<(usize, String, DecodedSignal)> = None;
            #[allow(clippy::needless_range_loop)] // the reset below borrows every decoder
            for k in 0..self.decoders.len() {
                if !allowed[k] || !frequency_supported(self.decoders[k].supported_frequencies(), frequency) {
                    continue;
                }
                let decoded = match pulse {
                    Some((level, duration_us)) => self.decoders[k].feed(level, duration_us),
                    None => self.decoders[k].flush(),
                };
                let Some(decoded) = decoded else { continue };
                if hit.is_some() {
                    continue;
                }
                let name = decoded
                    .protocol_display_name
                    .as_deref()
                    .unwrap_or_else(|| self.decoders[k].name())
                    .to_string();
                if name == PSA_UNCHECKED_NAME {
                    unchecked.get_or_insert((k, name, decoded));
                    continue;
                }
                let rtl = self.rtl[k];
                for (d, &r) in self.decoders.iter_mut().zip(&self.rtl) {
                    if r || !rtl {
                        d.reset();
                    }
                }
                hit = Some((k, name, decoded));
            }
            if let Some((k, name, decoded)) = unchecked {
                let start = unchecked_start.max(segment_start).min(at);
                let end = if invert_level || pulse.is_none() { at } else { self.frame_end(k, pairs, start, i) };
                out.push((i, name, decoded, segment(pairs, start, end)));
                unchecked_start = end + 1;
            }
            let Some((k, name, decoded)) = hit else { continue };
            if self.rtl[k] {
                if shadowed[k] {
                    continue;
                }
                // Report the frame held before it; hold this one.
                if let Some((_, s, e, n, d)) = pending.take() {
                    out.push((e, n, d, segment(pairs, s, e)));
                    segment_start = e + 1;
                }
                pending = Some((k, segment_start.min(at), at, name, decoded));
                continue;
            }
            // A Flipper decoder reads the signal: a held rtl_433 frame of it is dropped.
            if let Some((pk, s, _, _, _)) = pending.take() {
                shadowed[pk] = true;
                segment_start = s;
            }
            // The frame's pulses end on the one that completed it — unless the decoder reported at
            // the middle of its last bit (Flipper's Manchester table emits a bit on its first half):
            // then that bit's second half, the next pulse, is the frame's too. Replay sends the
            // stored pulses as they are, and a frame stored without its last half-bit (a LOW,
            // merged into the silence after the transmission) is read by no firmware.
            let start = segment_start.min(at);
            let end = if invert_level || pulse.is_none() { at } else { self.frame_end(k, pairs, start, i) };
            out.push((i, name, decoded, segment(pairs, start, end)));
            segment_start = end + 1;
        }
        if let Some((_, s, e, n, d)) = pending {
            out.push((e, n, d, segment(pairs, s, e)));
        }

        out.sort_by_key(|f| f.0);
        out.into_iter().map(|(_, name, decoded, pulses)| (name, decoded, pulses)).collect()
    }

    /// The last pulse of the frame decoder `k` reported on pulse `i` of `pairs`, its segment
    /// starting at `start` (see [Self::process_signal_stream_inner]). The stored pulses, sent as
    /// Replay sends them (equal levels merged, silence after), must decode again; when they do not,
    /// the next pulse is added, then one more. `i` when nothing helps.
    fn frame_end(&mut self, k: usize, pairs: &[LevelDuration], start: usize, i: usize) -> usize {
        // A frame completed on a gap (a LOW no half-bit is as long as) ends there.
        if !pairs[i].level && pairs[i].duration_us >= 2000 {
            return i;
        }
        for end in i..(i + 3).min(pairs.len()) {
            if self.replays(k, &pairs[start..=end]) {
                return end;
            }
        }
        i
    }

    /// Whether decoder `k` reads a frame from `pulses` sent as Replay sends them: adjacent pulses
    /// of one level are one pulse on air, and silence (LOW) follows the last. Run on the probe copy
    /// of the decoder, so that state surviving `reset()` (repeat memories) is left alone.
    fn replays(&mut self, k: usize, pulses: &[LevelDuration]) -> bool {
        let dec = &mut self.probe[k];
        dec.reset();
        let mut on_air: Vec<LevelDuration> = Vec::with_capacity(pulses.len() + 1);
        for p in pulses.iter().chain(std::iter::once(&LevelDuration::new(false, 20_000))) {
            match on_air.last_mut() {
                Some(last) if last.level == p.level => last.duration_us += p.duration_us,
                _ => on_air.push(*p),
            }
        }
        let hit = on_air.iter().any(|p| dec.feed(p.level, p.duration_us).is_some());
        dec.reset();
        hit
    }

    /// Which decoders take part in a pass: those flagged for the capture's modulation
    /// ([modulation_allows]), and in the inverted-polarity pass on OOK neither the gate protocols
    /// nor [NO_INVERTED_PASS].
    fn allowed(&self, rf: Option<crate::capture::RfModulation>, invert_level: bool, fsk: bool) -> Vec<bool> {
        self.decoders
            .iter()
            .zip(&self.rf)
            .zip(&self.gate)
            .map(|((d, &rf_d), &gate)| {
                modulation_allows(rf_d, rf)
                    && !(invert_level && ((gate && !fsk) || NO_INVERTED_PASS.contains(&d.name())))
            })
            .collect()
    }

    /// Inner decode: feed pairs (with optional level flip) to decoders that support this frequency.
    fn process_signal_inner(
        &mut self,
        pairs: &[LevelDuration],
        frequency: u32,
        invert_level: bool,
        rf: Option<crate::capture::RfModulation>,
        fsk: bool,
    ) -> Option<(String, DecodedSignal)> {
        let allowed = self.allowed(rf, invert_level, fsk);
        for decoder in &mut self.decoders {
            decoder.reset();
        }

        for pair in pairs {
            let level = if invert_level { !pair.level } else { pair.level };
            let duration_us = pair.duration_us;

            // Feed this pulse to all decoders; report first valid frame (Flipper-style).
            let mut hits: Vec<(String, DecodedSignal)> = Vec::new();
            for (decoder, &ok) in self.decoders.iter_mut().zip(&allowed) {
                if !ok || !frequency_supported(decoder.supported_frequencies(), frequency) {
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

    /// Get a decoder by name: a registry name, a decoder display name (a " [variant]" suffix is
    /// dropped, e.g. "Porsche Cayenne [First]"), or a legacy / reference alias ("Land Rover V0" →
    /// Honda V2, "PSA2" → PSA, "Suzuki V0" → Kia V0 …). The aliases live with each protocol's
    /// metadata ([info]), so names resolve to the same protocol for TX and for display.
    pub fn get(&self, name: &str) -> Option<&dyn ProtocolDecoder> {
        let name = info::strip_variant(name);
        // "KeeLoq (<manufacturer>)" decodes; the generic fallback's "Keeloq (<key>)" captures do
        // not resolve (their Kia V3/V4- or Star Line-format frames are not KeeLoq-encoder frames).
        let lookup = if name.starts_with("KeeLoq") {
            "KeeLoq"
        } else {
            info::by_name(name).map_or(name, |i| i.decoder)
        };
        self.decoders
            .iter()
            .find(|d| d.name().eq_ignore_ascii_case(lookup))
            .map(|d| d.as_ref())
    }

    /// Decode a Flipper key file with the decoder its `Protocol` line names (aliases resolve as
    /// in [Self::get], plus the names other firmwares write, [flipper_key::registry_name]).
    /// Returns the display name and the rebuilt decode.
    pub fn decode_key(&self, key: &FlipperKey) -> Option<(String, DecodedSignal)> {
        let decoder = self.get(flipper_key::registry_name(&key.protocol))?;
        let d = decoder.from_flipper_key(key)?;
        let name = d.protocol_display_name.clone().unwrap_or_else(|| decoder.name().to_string());
        Some((name, d))
    }

    /// Feed `pairs` to the decoder `name` resolves to, alone (reset first): every frame it reports.
    /// The app reads what it just transmitted this way (`app/tx.rs`).
    pub(crate) fn decode_with(&mut self, name: &str, pairs: &[LevelDuration]) -> Vec<DecodedSignal> {
        let Some(lookup) = self.get(name).map(|d| d.name()) else { return Vec::new() };
        let Some(dec) = self.decoders.iter_mut().find(|d| d.name() == lookup) else { return Vec::new() };
        dec.reset();
        pairs.iter().filter_map(|p| dec.feed(p.level, p.duration_us)).collect()
    }

    /// `autobf`: a capture whose key a search can recover (an undecrypted PSA frame) gets that
    /// search without being asked ([psa::auto_key_search]; the app runs it in the background, `kat
    /// --decode` in line), and the decoders report the frames that only count once their key is
    /// found ([ProtocolDecoder::set_report_unchecked]) instead of dropping them. Off by default.
    pub fn set_auto_key_search(&mut self, on: bool) {
        self.auto_key_search = on;
        for d in self.decoders.iter_mut().chain(&mut self.probe) {
            d.set_report_unchecked(on);
        }
    }

    /// Whether [Self::set_auto_key_search] is on.
    pub fn auto_key_search(&self) -> bool {
        self.auto_key_search
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
/// generic names". The labels live with each protocol's metadata ([info]); codes and labels come
/// from each protocol's C reference (see its module doc).
pub fn native_button_name(protocol: &str, code: u8) -> Option<&'static str> {
    info::lookup(protocol)?.button_name(protocol, code)
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
    fn unlock_honda_sub_is_honda_v5_not_static() {
        // Genuine Honda keyfob capture (CC1101 custom preset). Honda Static's unchecksummed
        // reverse path used to misread it; Honda V5 (CRC-8-checked) reads it (honda_v5.rs).
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
            !results.is_empty() && results.iter().all(|(n, _, _)| n == "Honda V5"),
            "expected only Honda V5 decodes from Unlock_honda.sub, got: {:?}",
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
    fn prius_keeloq_frames_are_toyota_foxfw_not_kia_v3_v4() {
        // The 2006 Prius captures are 400/800µs KeeLoq-layout PWM frames. Like both C references
        // (ProtoPirate / Flipper-ARF kia_v3_v4.c), Kia V3/V4 only reports a frame whose hop decrypts
        // under the KIA key to the frame's own button and serial LSB — the Prius hop does not, so Kia
        // V3/V4 must not claim it (its old claims were bogus). The structure-only KeeLoq Toyota
        // decoder that reads these frames, which Flipper-ARF retired (toyota-old.c.bak) for the
        // Toyota/Lexus protocol of KAT's "Toyota", is FoxFW's "Toyota": KAT's port, Toyota (FoxFW),
        // reports the 8 frames of each file as FoxFW's C does (68 bits, tail 0xF, serial 0x7231804).
        crate::protocols::keys::load_keystore_from_embedded();
        for (file, key) in [
            ("Toyota_Prius2006_lock.sub", 0x3F2E_5098_7231_804B_u64),
            ("Toyota_Prius2006_unlock.sub", 0x29CD_45FF_7231_804E),
        ] {
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
            let decodes = reg.process_signal_stream(&pairs, freq);
            let names: Vec<&str> = decodes.iter().map(|(n, _, _)| n.as_str()).collect();
            assert!(!names.iter().any(|n| n.starts_with("Kia")), "{file}: {names:?}");
            assert_eq!(decodes.len(), 8, "{file}: {names:?}");
            for (n, d, _) in &decodes {
                let got = (n.as_str(), d.data, d.data_count_bit, d.extra, d.serial);
                assert_eq!(got, ("Toyota (FoxFW)", key, 68, Some(0xF), Some(0x723_1804)), "{file}");
            }
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
        // ProtoPirate catalog aliases, from the protocols' metadata (`info.rs`).
        assert_eq!(reg.get("VW").map(|d| d.name()), Some("VAG"));
        assert_eq!(reg.get("StarLine").map(|d| d.name()), Some("Star Line"));
        // The generic KeeLoq fallback's captures (Kia V3/V4- or Star Line-format frames) must not
        // reach the KeeLoq encoder.
        assert!(reg.get("Keeloq (DoorHan)").is_none());
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
        assert_eq!(native_button_name("Kia V7", 1), Some("Lock")); // the references' own names
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

    /// `path` relative to IMPORTS with `/` separators, as testdata and the expectations write it
    /// (Windows paths use `\`).
    fn imports_rel(path: &Path) -> String {
        path.strip_prefix("IMPORTS").unwrap_or(path).to_string_lossy().replace('\\', "/")
    }

    /// Every `IMPORTS/<make>/*.sub` capture, sorted (None when IMPORTS is absent).
    fn imports_subs() -> Option<Vec<std::path::PathBuf>> {
        let root = Path::new("IMPORTS");
        if !root.exists() {
            return None;
        }
        let mut subs: Vec<std::path::PathBuf> = std::fs::read_dir(root)
            .ok()?
            .flatten()
            .filter(|make| make.path().is_dir())
            .filter_map(|make| std::fs::read_dir(make.path()).ok())
            .flat_map(|files| files.flatten().map(|f| f.path()))
            .filter(|p| p.extension().map(|e| e == "sub").unwrap_or(false))
            .collect();
        subs.sort();
        Some(subs)
    }

    /// Decode a capture file the way the app imports it (keystore loaded, streaming decode).
    /// Decode a `.sub` the way the app imports it: gated by the file's preset modulation.
    fn decode_import(reg: &mut ProtocolRegistry, path: &Path) -> anyhow::Result<(u32, Vec<(String, DecodedSignal)>)> {
        let sub = crate::export::flipper::import_sub(path)?;
        let pairs: Vec<LevelDuration> = sub
            .pairs
            .iter()
            .map(|p| LevelDuration::new(p.level, p.duration_us))
            .collect();
        let rf = sub.preset.as_ref().and_then(|p| p.rf());
        let results = reg.process_signal_stream_rf(&pairs, sub.frequency, rf);
        Ok((sub.frequency, results.into_iter().map(|(n, d, _)| (n, d)).collect()))
    }

    fn golden_fields(d: &DecodedSignal) -> String {
        let hex = |v: Option<u64>| v.map(|v| format!("{v:X}")).unwrap_or_else(|| "-".to_string());
        format!(
            "data={:X}/{}{} sn={} btn={} cnt={} crc={} tx={}",
            d.data,
            d.data_count_bit,
            d.extra.map(|e| format!(" extra={e:X}")).unwrap_or_default(),
            hex(d.serial.map(u64::from)),
            hex(d.button.map(u64::from)),
            hex(d.counter.map(u64::from)),
            u8::from(d.crc_valid),
            u8::from(d.encoder_capable),
        )
    }

    /// `autobf` adds the PSA frames that wait for a key search ([PSA_UNCHECKED_NAME]) and changes
    /// no other protocol's decodes: with it on, every non-PSA decode of every IMPORTS capture, and
    /// the pulses stored with it, is the one with it off, on the file path and on the receiver's
    /// path. (PSA's own list differs by design: after an unchecked report the decoder restarts, so
    /// with `autobf` the all-ones junk frame of `Peugeot 307 open.sub` is no longer listed.) An unchecked
    /// frame used to be a registry hit: it reset every other decoder and was dropped when no key
    /// fitted, so the frame another decoder was reading then was lost, and the inverted and
    /// ungated passes it counted as a decode never ran. A VW Polo 5 press with one pulse missing
    /// (the damaged first frame lets PSA's doctor-firmware path read an unchecked frame) lost its
    /// second VAG Polo frame that way.
    #[test]
    fn autobf_changes_no_other_decode() {
        crate::protocols::keys::load_keystore_from_embedded();
        let Some(subs) = imports_subs() else {
            eprintln!("Skip: IMPORTS not found (run from crate root)");
            return;
        };
        type Stream = (String, u32, Option<crate::capture::RfModulation>, Vec<LevelDuration>);
        let mut streams: Vec<Stream> = Vec::new();
        for path in &subs {
            let Ok(sub) = crate::export::flipper::import_sub(path) else { continue };
            let pairs = sub.pairs.iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect();
            streams.push((imports_rel(path), sub.frequency, sub.preset.as_ref().and_then(|p| p.rf()), pairs));
        }
        let polo = streams.iter().find(|s| s.0 == "VAG/VW Polo 5 2017 unlock2.sub").cloned().expect("VW Polo capture");
        let mut damaged: Vec<LevelDuration> = Vec::new();
        for (i, p) in polo.3.iter().enumerate().filter(|&(i, _)| i != 559) {
            assert!(i != 558 || (p.level, polo.3[560].level) == (true, true), "pulse 559 is a LOW between HIGHs");
            match damaged.last_mut() {
                Some(last) if last.level == p.level => last.duration_us += p.duration_us,
                _ => damaged.push(*p),
            }
        }
        streams.push(("VW Polo unlock2, pulse 559 dropped".to_string(), polo.1, polo.2, damaged.clone()));

        type Frames = Vec<(String, DecodedSignal, Vec<LevelDuration>)>;
        type Comparable = Vec<(String, DecodedSignal, Vec<(bool, u32)>)>;
        let comparable = |frames: Frames| -> Comparable {
            frames
                .into_iter()
                .filter(|(n, _, _)| n != PSA_UNCHECKED_NAME && n != "PSA")
                .map(|(n, d, p)| (n, d, p.iter().map(|x| (x.level, x.duration_us)).collect()))
                .collect()
        };
        let mut off = ProtocolRegistry::new();
        let mut on = ProtocolRegistry::new();
        on.set_auto_key_search(true);
        // A capture in which `autobf` reports no unchecked frame decodes exactly as without it, so
        // only those with one are compared (this keeps the test to about one pass over IMPORTS).
        let mut compared = 0;
        for (label, freq, rf, pairs) in &streams {
            let with_bf = on.process_signal_stream_rf(pairs, *freq, *rf);
            if !with_bf.iter().any(|(n, _, _)| n == PSA_UNCHECKED_NAME) {
                continue;
            }
            compared += 1;
            assert_eq!(comparable(with_bf), comparable(off.process_signal_stream_rf(pairs, *freq, *rf)), "{label}");
            assert_eq!(
                comparable(on.process_signal_stream_path(pairs, *freq, *rf)),
                comparable(off.process_signal_stream_path(pairs, *freq, *rf)),
                "{label} (receiver path)"
            );
        }
        assert!(compared >= 2, "{compared} captures with unchecked frames: the test no longer covers them");
        let polo_frames: Vec<u64> = on
            .process_signal_stream_rf(&damaged, polo.1, polo.2)
            .into_iter()
            .filter(|(n, _, _)| n == "VAG Polo")
            .map(|(_, d, _)| d.data)
            .collect();
        assert_eq!(polo_frames, [0x9130_0AA0_6E93_98DA, 0x875B_9EDE_C03C_290B]);
    }

    /// Regression guard: every decode of every IMPORTS capture (all fields) must match
    /// `testdata/imports_decodes.txt`. After an intended change, regenerate it with
    /// `KAT_UPDATE_GOLDEN=1 cargo test imports_decode_golden` and review the diff.
    #[test]
    fn imports_decode_golden() {
        let _keystore = keeloq_common::KEYSTORE_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        crate::protocols::keys::load_keystore_from_embedded();
        let Some(subs) = imports_subs() else {
            eprintln!("Skip: IMPORTS not found (run from crate root)");
            return;
        };
        let mut reg = ProtocolRegistry::new();
        let mut lines = Vec::new();
        for path in &subs {
            let rel = imports_rel(path);
            match decode_import(&mut reg, path) {
                Ok((freq, decodes)) if decodes.is_empty() => lines.push(format!("{rel} | {freq} | (none)")),
                Ok((freq, decodes)) => {
                    for (name, d) in &decodes {
                        lines.push(format!("{rel} | {freq} | {name} | {}", golden_fields(d)));
                    }
                }
                Err(e) => lines.push(format!("{rel} | ERR {e}")),
            }
        }
        let actual = lines.join("\n") + "\n";
        let golden = Path::new("testdata/imports_decodes.txt");
        if std::env::var_os("KAT_UPDATE_GOLDEN").is_some() {
            std::fs::create_dir_all("testdata").unwrap();
            std::fs::write(golden, &actual).unwrap();
            return;
        }
        let expected = std::fs::read_to_string(golden)
            .expect("testdata/imports_decodes.txt missing; create it with KAT_UPDATE_GOLDEN=1");
        if expected != actual {
            use std::collections::HashSet;
            let exp: HashSet<&str> = expected.lines().collect();
            let act: HashSet<&str> = actual.lines().collect();
            let mut diff: Vec<String> = expected.lines().filter(|l| !act.contains(l)).map(|l| format!("- {l}")).collect();
            diff.extend(actual.lines().filter(|l| !exp.contains(l)).map(|l| format!("+ {l}")));
            if diff.is_empty() {
                diff.push("(same lines, different order or repeat counts)".to_string());
            }
            diff.truncate(60);
            panic!(
                "IMPORTS decodes changed (- expected, + now). If intended, regenerate with \
                 KAT_UPDATE_GOLDEN=1 cargo test imports_decode_golden and review the diff:\n{}",
                diff.join("\n")
            );
        }
    }

    /// `info::NO_CHECKSUM` names registered decoders, and none of them ever reports a valid
    /// checksum: on every IMPORTS capture and, when the references are checked out, every Flipper
    /// unit-test capture (RAW files decoded, key files loaded), a decode whose protocol shows "-"
    /// for its CRC has `crc_valid == false`. A decoder that gains a check must leave the list.
    #[test]
    fn no_checksum_protocols_never_report_a_valid_crc() {
        crate::protocols::keys::load_keystore_from_embedded();
        let mut reg = ProtocolRegistry::new();
        for &name in info::NO_CHECKSUM {
            assert_eq!(reg.get(name).map(|d| d.name()), Some(name), "{name}: not a registered decoder name");
            assert!(!info::has_checksum(name), "{name}");
        }
        let mut paths = imports_subs().unwrap_or_default();
        let unit = Path::new("REFERENCES/Flipper-ARF/applications/debug/unit_tests/resources/unit_tests/subghz");
        if let Ok(dir) = std::fs::read_dir(unit) {
            paths.extend(dir.flatten().map(|e| e.path()).filter(|p| p.extension().is_some_and(|e| e == "sub")));
        }
        let (mut checked, mut unchecked) = (0, 0);
        for path in &paths {
            let mut decodes = decode_import(&mut reg, path).map(|(_, d)| d).unwrap_or_default();
            if decodes.is_empty() {
                let key = std::fs::read_to_string(path).ok().and_then(|t| FlipperKey::parse(&t).ok());
                decodes.extend(key.and_then(|k| reg.decode_key(&k)));
            }
            for (name, d) in decodes {
                if info::has_checksum(&name) {
                    checked += 1;
                } else {
                    unchecked += 1;
                    assert!(!d.crc_valid, "{}: {name} is listed without a checksum but reported a valid one", path.display());
                }
            }
        }
        eprintln!("{unchecked} decodes of protocols without a checksum, {checked} with one");
    }

    /// Whether `pulses` (an encoder's upload) decodes as `name` with `d`'s serial, through the whole
    /// registry (first-match, both polarities) as the receiver would see it on air.
    fn tx_decodes_back(reg: &mut ProtocolRegistry, name: &str, d: &DecodedSignal, freq: u32, pulses: &[LevelDuration]) -> Result<(), String> {
        // The upload once more after a gap, as the transmitter repeats it; adjacent pulses of the
        // same level are one pulse on air.
        let mut stream: Vec<LevelDuration> = Vec::with_capacity(2 * pulses.len() + 2);
        let gap = [LevelDuration::new(false, 20_000)];
        for p in pulses.iter().chain(&gap).chain(pulses).chain(&gap) {
            match stream.last_mut() {
                Some(last) if last.level == p.level => last.duration_us += p.duration_us,
                _ => stream.push(LevelDuration::new(p.level, p.duration_us)),
            }
        }
        let base = |n: &str| reg.get(n).map(|x| x.name());
        let want = base(name);
        let hits = reg.process_signal_stream(&stream, freq);
        let got: Vec<String> = hits.iter().map(|(n, x, _)| format!("{n} sn={:X?}", x.serial)).collect();
        let ok = hits.iter().any(|(n, x, _)| {
            let same_protocol = reg.get(n).map(|p| p.name()) == want;
            same_protocol && (d.serial.is_none() || x.serial == d.serial)
        });
        if ok {
            return Ok(());
        }
        // Another decoder completing on the same pulse wins the registry's first-match race (an
        // encoder-exact waveform can pass a loose gate, e.g. Mazda Siemens' 8-bit sum on one Fiat V2
        // frame); the frame itself must still decode with its own protocol.
        let own = reg.decode_with(name, &stream);
        if own.iter().any(|x| d.serial.is_none() || x.serial == d.serial) {
            eprintln!("note: {name} sn={:X?}: TX decodes with its own decoder; the registry reports {got:?}", d.serial);
            Ok(())
        } else {
            Err(format!("{name} sn={:X?}: TX decodes as {got:?} (own decoder: {} frames)", d.serial, own.len()))
        }
    }

    /// The Flipper key files in IMPORTS (saved by Flipper-ARF builds, one by an older third-party
    /// build) load with the fields the references show after loading them; where IMPORTS also has
    /// a RAW capture of the same press (VAG Passat, Fiat Bravo), the decode equals that capture's
    /// (see `testdata/imports_decodes.txt`). Each reloaded key transmits a frame that decodes
    /// again with the same serial (TX from a loaded key file).
    #[test]
    fn imports_key_files_load() {
        let _keystore = keeloq_common::KEYSTORE_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        crate::protocols::keys::load_keystore_from_embedded();
        let Some(subs) = imports_subs() else {
            eprintln!("Skip: IMPORTS not found (run from crate root)");
            return;
        };
        // (file, display name, fields as in the golden file)
        let expected: &[(&str, &str, &str)] = &[
            // = the RAW capture VAG/Vw_passat_b5_3bg.sub
            ("VAG/Passat_b5_3bg.sub", "VAG", "data=C09EBD0792CA7FC3/80 extra=800000010204A72B sn=C21578B btn=2 cnt=2B06 crc=1 tx=1"),
            // Same writer (`Key_2`, `Type: 7428` = 0x1D04 → Type 4): its sender's screenshot of a
            // later press shows "Ser:3BF8E083 Btn:10:OP Cnt:019C63". ProtoPirate and doctor only
            // read `Key2`, so they cannot decrypt this file.
            ("VAG/Audi A3 2012 key.sub", "VAG", "data=C129194B7ABCFEC2/80 extra=8000000101032F1D sn=3BF8E083 btn=1 cnt=9C61 crc=1 tx=1"),
            // = frame A03BF868… of the RAW capture FIAT/Fiat Bravo 2010 S.Croce 433.88 am650.sub
            ("FIAT/Fiat type 0 N1.sub", "Fiat V0", "data=A03BF86854536200/71 sn=54536200 btn=3B cnt=F868 crc=0 tx=1"),
            ("HYUNDAI/HYU V0 N1.sub", "Kia V0", "data=F739167BD75B2BD/61 sn=67BD75B btn=2 cnt=7391 crc=1 tx=1"),
            ("HYUNDAI/HYU V0_great_konserva.sub", "Kia V0", "data=F6F4667BD75B2BF/61 sn=67BD75B btn=2 cnt=6F46 crc=1 tx=1"),
            // Flipper-ARF "KIA/HYU Vn" keys. KAT's transmission of each decodes in doctor-firmware's
            // decoders (C oracle) with the same serial and button and the counter + 1.
            ("HYUNDAI/HYU V2 N1.sub", "Kia V2", "data=E18CC1429A2CE/53 sn=E18CC142 btn=9 cnt=CA2 crc=1 tx=1"),
            ("HYUNDAI/HYU V2_random_habar.sub", "Kia V2", "data=E18CC1422E6B2/53 sn=E18CC142 btn=2 cnt=BE6 crc=1 tx=1"),
            ("HYUNDAI/HYU V4 N1.sub", "Kia V4", "data=C3B1790C8FC0C864/68 extra=0 sn=61303F1 btn=2 cnt=BDE9 crc=1 tx=1"),
            // Kia V5: the files' `CRC: 3` matches neither CRC of the key (ARF shows it as failed too).
            ("HYUNDAI/test_new_kia_hyu.sub", "Kia V5", "data=7AA31142AF951B84/67 extra=3 sn=1D8A9F5 btn=2 cnt=3B54 crc=0 tx=1"),
            ("KIA/KIA V5 N3.sub", "Kia V5", "data=4CD2BB72AF951B84/67 extra=3 sn=1D8A9F5 btn=2 cnt=3C12 crc=0 tx=1"),
            // Kia V6: 144-bit frames (Key, Key_2, Key_3); KAT keeps the 16 low counter bits
            // (C106C04B in the second file).
            ("HYUNDAI/HYU V6_test.sub", "Kia V6", "data=ED028AE751F0ECAE/144 extra=D2 sn=1AD9C3 btn=1 cnt=2012 crc=1 tx=1"),
            ("KIA/KIA V6 N2.sub", "Kia V6", "data=DA07827EFCA4EA82/144 extra=1C106A7 sn=109DA7 btn=B cnt=C04B crc=1 tx=1"),
        ];
        let reg = ProtocolRegistry::new();
        let mut found = Vec::new();
        let mut failures = Vec::new();
        for path in &subs {
            let Ok(crate::export::flipper::SubContent::Key(key)) = crate::export::flipper::read_sub(path) else { continue };
            let rel = imports_rel(path);
            found.push(rel.clone());
            let Some(&(_, want_name, want_fields)) = expected.iter().find(|e| e.0 == rel) else {
                failures.push(format!("{rel}: key file without an expectation"));
                continue;
            };
            let Some((name, d)) = reg.decode_key(&key) else {
                failures.push(format!("{rel}: does not load"));
                continue;
            };
            if (name.as_str(), golden_fields(&d).as_str()) != (want_name, want_fields) {
                failures.push(format!("{rel}: {name} | {}\n   expected {want_name} | {want_fields}", golden_fields(&d)));
                continue;
            }
            let dec = reg.get(&name).unwrap();
            match dec.encode(&d, d.button.unwrap_or(0)) {
                Some(pulses) => {
                    let mut tx_reg = ProtocolRegistry::new();
                    if let Err(e) = tx_decodes_back(&mut tx_reg, &name, &d, key.frequency, &pulses) {
                        failures.push(format!("{rel}: {e}"));
                    }
                }
                None => failures.push(format!("{rel}: {name} does not encode")),
            }
        }
        for (file, ..) in expected {
            if !found.iter().any(|f| f == file) {
                failures.push(format!("{file}: expected key file not found"));
            }
        }
        assert!(failures.is_empty(), "{} failure(s):\n{}", failures.len(), failures.join("\n"));
    }

    /// Every `Protocol:` name KAT writes in a key file resolves (through `decode_key`'s lookup) to
    /// the decoder that wrote it, so a key KAT saved loads in KAT again.
    #[test]
    fn written_key_names_resolve_to_their_decoder() {
        let reg = ProtocolRegistry::new();
        let written: &[(&str, &str)] = &[
            ("Kia V0", "Kia V0"), ("Kia V1", "Kia V1"), ("Kia V2", "Kia V2"), ("Kia V3", "Kia V3/V4"),
            ("Kia V4", "Kia V3/V4"), ("Kia V5", "Kia V5"), ("Kia V6", "Kia V6"), ("Kia V7", "Kia V7"),
            ("VAG", "VAG"), ("Ford V0", "Ford V0"), ("Ford V1", "Ford V1"), ("Ford V2", "Ford V2"),
            ("Ford V3", "Ford V3"), ("Honda Static", "Honda Static"), ("Honda V5", "Honda V5"), ("Honda V1", "Honda V1"),
            ("Honda V2", "Honda V2"), ("Honda V4", "Honda V4"), ("Honda V7", "Honda V7"),
            ("Honda V9", "Honda V9"), ("Honda V10", "Honda V10"),
            ("Subaru", "Subaru"), ("Fiat V0", "Fiat V0"), ("Fiat V1", "Fiat V1"),
            ("Fiat V2", "Fiat V2"), ("Scher-Khan", "Scher-Khan"), ("Star Line", "Star Line"),
            ("Sheriff CFM", "Sheriff CFM"), ("KeeLoq", "KeeLoq"), ("PSA", "PSA"),
            ("Chrysler V0", "Chrysler V0"), ("Mazda V0", "Mazda V0"), ("MazdaSiemens", "Mazda Siemens"),
            ("Mitsubishi V0", "Mitsubishi V0"), ("Porsche AG", "Porsche Cayenne"), ("Toyota", "Toyota"),
            ("BMW", "BMW"), ("Mazda V2", "Mazda V2"), ("Mercedes V0", "Mercedes V0"),
            ("Mercedes V1", "Mercedes V1"), ("Opel V1", "Opel V1"), ("Renault V1", "Renault V1"),
            ("Renault V0", "Renault V0"), ("GM", "GM"), ("Datsun Lada", "Datsun Lada"),
            ("VAG Polo", "VAG Polo"), ("Chevrolet V0", "Chevrolet V0"),
            // FoxFW decoders (Tesla Charge, Hyundai/Kia RIO and SantaFe 13-16 keys transmit as FoxFW's
            // encoder plugins send them; FoxFW's Nidec RKE plugin refuses every key).
            ("Tesla Charge", "Tesla Charge"), ("Hyundai/Kia RIO", "Hyundai/Kia RIO"),
            ("SantaFe 13-16", "SantaFe 13-16"), ("Hitag2", "Hitag2"), ("Nidec RKE", "Nidec RKE"),
            // FoxFW decoders (key files as FoxFW's serialize writes them).
            ("Renault V2", "Renault V2"), ("Renault/xHorse V4", "Renault/xHorse V4"),
            ("Fiat Ducato V0", "Fiat Ducato V0"), ("Opel V0", "Opel V0"),
            // FoxFW (decode only but BMW CAS3, which replays; Audi V1 and Audi V2 are kept but not
            // registered).
            ("Audi V3", "Audi V3"), ("BMW V2", "BMW V2"),
            ("BMW/Mini V3", "BMW/Mini V3"), ("BMW CAS3", "BMW CAS3"),
            // Names other references write for the same protocols.
            ("KIA/HYU V3/V4", "Kia V3/V4"), ("VAG GROUP", "VAG"), ("PSA GROUP", "PSA"), ("FORD V0", "Ford V0"),
            ("SUBARU", "Subaru"), ("Chrysler", "Chrysler V0"), ("Toyota Lexus", "Toyota"),
            ("Land Rover V0", "Honda V2"), ("Suzuki", "Kia V0"),
            // Flipper-ARF / FoxFW (key files only, no decoder).
            ("MARELLI", "MARELLI"),
            // FoxFW's own ports of five rtl_433 decoders (key files as FoxFW's serialize writes them;
            // FoxFW's encoder plugins for them refuse every key).
            ("Compustar", "Compustar-1WG3R"), ("AstroStart 2000", "Astrostart-2000"),
            ("Audiovox PRO", "Audiovox-PROOE3B"), ("Alps FWB1U545", "Alps-FWB1U545"),
            ("Continental RKE", "Continental-KR5V2X"),
        ];
        for &(file_name, decoder) in written {
            let got = reg.get(flipper_key::registry_name(file_name)).map(|d| d.name());
            assert_eq!(got, Some(decoder), "{file_name}");
        }
        // Every registered car decoder writes key files (none is left at the trait's default).
        // "Renault V1 (ARF)" writes Flipper-ARF's "Renault V1"; the Renault V1 decoder hands its
        // `Bit: 82` keys over (renault_v1_arf.rs `flipper_key_files` loads them via `decode_key`).
        // Likewise "Toyota (FoxFW)" writes FoxFW's "Toyota", whose `Bit: 66..=68` keys the Toyota
        // decoder hands over (toyota_foxfw.rs `key_files`).
        // The rtl_433 ports are receive only (FoxFW's encoder plugins for the five it also decodes
        // are stubs that refuse every key). Those five write FoxFW's key files (listed above); the
        // others write none: no Flipper firmware has a key format for them. (Gate/garage protocols
        // test their key files in their own modules.)
        for d in &reg.decoders {
            let name = d.name();
            if info::lookup(name).is_some_and(|i| i.category == info::Category::Gate) {
                continue;
            }
            if RTL433_PORTS.contains(&name) {
                assert!(!d.supports_encoding(), "{name}: an rtl_433 port is receive only");
                if !written.iter().any(|&(_, dec)| dec == name) {
                    assert!(d.to_flipper_key(&DecodedSignal::new(0, 64)).is_none(), "{name}");
                }
                continue;
            }
            // FoxFW's HondaV2&V3 keeps a 12-byte payload in its key file that a decode cannot hold;
            // FoxFW's decode-only decoders below write no key files (extra Serial/Hop lines not ported).
            let foxfw = ["Mitsubishi V1", "LDV V0", "Lexus V0", "Mazda V1", "Nissan V0"].contains(&name);
            let listed = written.iter().any(|&(_, dec)| dec == name) || name == "Renault V1 (ARF)" || name == "Toyota (FoxFW)" || name == "HondaV2&V3" || foxfw;
            assert!(listed, "{name}: no key-file name listed");
        }
    }

    /// Key files for every decode of every IMPORTS RAW capture: `to_flipper_key` → file text →
    /// `FlipperKey::parse` → `from_flipper_key` gives back the same decode (every field), and
    /// transmitting the reloaded decode produces a frame that decodes again (TX from a key file).
    #[test]
    fn imports_decodes_round_trip_through_key_files() {
        crate::protocols::keys::load_keystore_from_embedded();
        let Some(subs) = imports_subs() else {
            eprintln!("Skip: IMPORTS not found (run from crate root)");
            return;
        };
        let mut reg = ProtocolRegistry::new();
        let mut tx_reg = ProtocolRegistry::new();
        let mut failures = Vec::new();
        let mut supported: std::collections::BTreeMap<String, usize> = Default::default();
        let mut unsupported: std::collections::BTreeSet<String> = Default::default();
        let mut seen = std::collections::HashSet::new();
        for path in &subs {
            let rel = imports_rel(path);
            let Ok((freq, decodes)) = decode_import(&mut reg, path) else { continue };
            for (name, d) in decodes {
                if !seen.insert((name.clone(), d.data, d.extra, d.data_count_bit)) {
                    continue;
                }
                let dec = reg.get(&name).unwrap_or_else(|| panic!("{rel}: no decoder for {name}"));
                let Some((text, back)) = flipper_key::testing::round_trip(dec, &d) else {
                    unsupported.insert(name);
                    continue;
                };
                *supported.entry(name.clone()).or_default() += 1;
                if back != d {
                    failures.push(format!("{rel}: {name}: round trip\n  decoded {d:?}\n  reloaded {back:?}\n{text}"));
                    continue;
                }
                // The registry resolves the written Protocol name back to the same decode.
                let reloaded = reg.decode_key(&FlipperKey::parse(&text).unwrap());
                if reloaded.as_ref().map(|(n, x)| (n.as_str(), x)) != Some((name.as_str(), &d)) {
                    failures.push(format!("{rel}: {name}: decode_key gave {reloaded:?}\n{text}"));
                    continue;
                }
                if back.encoder_capable {
                    match dec.encode(&back, back.button.unwrap_or(0)) {
                        Some(pulses) => {
                            if let Err(e) = tx_decodes_back(&mut tx_reg, &name, &back, freq, &pulses) {
                                failures.push(format!("{rel}: {e}"));
                            }
                        }
                        None => failures.push(format!("{rel}: {name}: encoder_capable but encode() is None")),
                    }
                }
            }
        }
        eprintln!("key-file round trip: {supported:?}; not supported: {unsupported:?}");
        assert!(failures.is_empty(), "{} failure(s):\n{}", failures.len(), failures.join("\n"));
    }

    /// Diagnostic for the reference oracles: print KAT's key-file lines for every distinct decode
    /// of every IMPORTS RAW capture as `<file> | <protocol> | <lines joined by "; ">` (the
    /// oracle's `decode` output layout; Frequency/Preset from the file). `#[ignore]`d; run with
    /// `cargo test dump_imports_key_files -- --ignored --nocapture 2>&1 | grep ' | '`.
    #[test]
    #[ignore]
    fn dump_imports_key_files() {
        crate::protocols::keys::load_keystore_from_embedded();
        let Some(subs) = imports_subs() else { return };
        let mut reg = ProtocolRegistry::new();
        for path in &subs {
            let Ok(sub) = crate::export::flipper::import_sub(path) else { continue };
            let rel = imports_rel(path);
            let Ok((_, decodes)) = decode_import(&mut reg, path) else { continue };
            let mut seen = std::collections::HashSet::new();
            for (name, d) in decodes {
                let Some(mut key) = reg.get(&name).and_then(|dec| dec.to_flipper_key(&d)) else { continue };
                key.frequency = sub.frequency;
                if let Some(p) = &sub.preset {
                    key.set_preset(p);
                }
                let lines: Vec<String> = key
                    .to_text()
                    .lines()
                    .filter(|l| !l.starts_with("Filetype:") && !l.starts_with("Version:"))
                    .map(str::to_string)
                    .collect();
                let text = lines.join("; ");
                if seen.insert(text.clone()) {
                    eprintln!("{rel} | {name} | {text}");
                }
            }
        }
    }

    /// Diagnostic for the reference oracles: load every IMPORTS key file, transmit it (KAT's
    /// encoder, the captured button) and write the upload as a RAW `.sub` (twice, 20 ms apart)
    /// into `$KAT_KEY_TX_DIR`, printing KAT's decode of the key. A reference decoder run on those
    /// files shows what KAT's TX from a loaded key looks like to the references. `#[ignore]`d.
    #[test]
    #[ignore]
    fn dump_imports_key_file_tx() {
        crate::protocols::keys::load_keystore_from_embedded();
        let (Some(subs), Ok(dir)) = (imports_subs(), std::env::var("KAT_KEY_TX_DIR")) else { return };
        let reg = ProtocolRegistry::new();
        for path in &subs {
            let Ok(crate::export::flipper::SubContent::Key(key)) = crate::export::flipper::read_sub(path) else { continue };
            let Some((name, d)) = reg.decode_key(&key) else { continue };
            let Some(pulses) = reg.get(&name).and_then(|p| p.encode(&d, d.button.unwrap_or(0))) else { continue };
            let mut values: Vec<i64> = Vec::new();
            for _ in 0..2 {
                for p in pulses.iter().chain(&[LevelDuration::new(false, 20_000)]) {
                    let v = i64::from(p.duration_us);
                    match values.last_mut() {
                        Some(last) if (*last > 0) == p.level => *last += if p.level { v } else { -v },
                        _ => values.push(if p.level { v } else { -v }),
                    }
                }
            }
            let stem = path.file_stem().unwrap().to_string_lossy().replace(' ', "_");
            let mut text = format!(
                "Filetype: Flipper SubGhz RAW File\nVersion: 1\nFrequency: {}\nPreset: {}\nProtocol: RAW\n",
                key.frequency,
                key.preset.as_deref().unwrap_or("FuriHalSubGhzPresetOok650Async")
            );
            for chunk in values.chunks(512) {
                text += &format!("RAW_Data: {}\n", chunk.iter().map(i64::to_string).collect::<Vec<_>>().join(" "));
            }
            std::fs::write(Path::new(&dir).join(format!("{stem}.tx.sub")), text).unwrap();
            eprintln!("{stem} | {name} | {}", golden_fields(&d));
        }
    }

    /// Diagnostic: print which protocol(s) each IMPORTS/*.sub decodes as. `#[ignore]`d (always
    /// passes); run with `cargo test sweep_imports_decodes -- --ignored --nocapture`.
    #[test]
    #[ignore]
    fn sweep_imports_decodes() {
        crate::protocols::keys::load_keystore_from_embedded();
        let Some(subs) = imports_subs() else {
            eprintln!("Skip: IMPORTS not found");
            return;
        };
        let mut reg = ProtocolRegistry::new();
        for path in &subs {
            let rel = path.strip_prefix("IMPORTS").unwrap_or(path);
            match decode_import(&mut reg, path) {
                Ok((freq, decodes)) => {
                    let names: Vec<&str> = decodes.iter().map(|(n, _)| n.as_str()).collect();
                    eprintln!("{:<42} {:>9}Hz => {:?}", rel.display(), freq, names);
                }
                Err(e) => eprintln!("{:<42} ERR {:?}", path.display(), e),
            }
        }
    }

    /// Flipper's own Sub-GHz unit test (`applications/debug/unit_tests/tests/subghz/subghz_test.c`,
    /// Flipper-ARF copy), over its captures in `REFERENCES/` (skipped when absent, e.g. in CI):
    /// every `subghz_decoder_*` case must decode at least one frame of its protocol.
    const FLIPPER_UNIT_TESTS: &str = "REFERENCES/Flipper-ARF/applications/debug/unit_tests/resources/unit_tests/subghz";

    /// (capture, protocol) of every decoder case for a protocol KAT implements.
    const FLIPPER_DECODER_CASES: &[(&str, &str)] = &[
        ("came_atomo_raw.sub", "CAME Atomo"),
        ("came_raw.sub", "CAME"),
        ("came_twee_raw.sub", "CAME TWEE"),
        ("faac_slh_raw.sub", "Faac SLH"),
        ("gate_tx_raw.sub", "GateTX"),
        ("hormann_hsm_raw.sub", "Hormann HSM"),
        ("ido_117_111_raw.sub", "iDo 117/111"),
        ("doorhan_raw.sub", "KeeLoq"),
        ("nero_radio_raw.sub", "Nero Radio"),
        ("nero_sketch_raw.sub", "Nero Sketch"),
        ("nice_flo_raw.sub", "Nice FLO"),
        ("nice_flor_s_raw.sub", "Nice FloR-S"),
        ("princeton_raw.sub", "Princeton"),
        ("somfy_keytis_raw.sub", "Somfy Keytis"),
        ("somfy_telis_raw.sub", "Somfy Telis"),
        ("linear_raw.sub", "Linear"),
        ("linear_delta3_raw.sub", "LinearDelta3"),
        ("megacode_raw.sub", "MegaCode"),
        ("security_pls_1_0_raw.sub", "Security+ 1.0"),
        ("security_pls_2_0_raw.sub", "Security+ 2.0"),
        ("holtek_raw.sub", "Holtek"),
        ("power_smart_raw.sub", "Power Smart"),
        ("marantec_raw.sub", "Marantec"),
        ("bett_raw.sub", "BETT"),
        ("doitrand_raw.sub", "Doitrand"),
        ("phoenix_v2_raw.sub", "Phoenix_V2"),
        ("clemsa_raw.sub", "Clemsa"),
        ("ansonic_raw.sub", "Ansonic"),
        ("smc5326_raw.sub", "SMC5326"),
        ("holtek_ht12x_raw.sub", "Holtek_HT12X"),
        ("dooya_raw.sub", "Dooya"),
        ("alutech_at_4n_raw.sub", "Alutech AT-4N"),
        ("nice_one_raw.sub", "Nice FloR-S"),
        ("kinggates_stylo4k_raw.sub", "KingGates Stylo4k"),
        ("mastercode_raw.sub", "Mastercode"),
        ("dickert_raw.sub", "Dickert_MAHS"),
        ("marantec24_raw.sub", "Marantec24"),
        ("roger_raw.sub", "Roger"),
        ("revers_rb2_raw.sub", "Revers_RB2"),
        ("hay21_raw.sub", "Hay21"),
        ("gangqi_raw.sub", "GangQi"),
        ("hollarm_raw.sub", "Hollarm"),
        ("magellan_raw.sub", "Magellan"),
        ("legrand_raw.sub", "Legrand"),
        ("legrand_2E37F_raw.sub", "Legrand"),
        ("intertechno_v3_raw.sub", "Intertechno_V3"),
        ("feron_raw.sub", "Feron"),
        ("honeywell_wdb_raw.sub", "Honeywell"),
    ];

    /// Key files of every `subghz_encoder_*` case for a protocol KAT implements. Three cases fail in
    /// the reference C itself (host build, see the oracle notes), so they are not listed:
    /// `linear_delta3.sub` (LinearDelta3 reports only every third confirmed frame, and the upload
    /// has three), `revers_rb2.sub` (back-to-back Revers_RB2 frames never re-sync; a held button
    /// decodes) and `marantec24.sub` (the encoder's 15 200 µs gap sits exactly on the decoder's
    /// exclusive limit). Flipper's own test passes them only with state left over from earlier
    /// cases.
    const FLIPPER_ENCODER_CASES: &[&str] = &[
        "princeton.sub", "came.sub", "came_twee.sub", "gate_tx.sub", "nice_flo.sub", "doorhan.sub",
        "linear.sub", "megacode.sub", "holtek.sub", "security_pls_1_0.sub",
        "security_pls_2_0.sub", "power_smart.sub", "marantec.sub", "bett.sub", "doitrand.sub",
        "phoenix_v2.sub", "clemsa.sub", "ansonic.sub", "smc5326.sub", "holtek_ht12x.sub",
        "dooya.sub", "mastercode.sub", "dickert_mahs.sub", "roger.sub", "gangqi.sub", "hollarm.sub",
        "magellan.sub", "legrand.sub", "legrand_2E37F.sub", "intertechno_v3.sub", "feron.sub",
        "honeywell_wdb.sub",
    ];

    /// The decoder name a decode belongs to ("KeeLoq (DoorHan)" → "KeeLoq").
    fn decoder_of(reg: &ProtocolRegistry, display: &str) -> String {
        reg.get(display).map_or_else(|| display.to_string(), |d| d.name().to_string())
    }

    #[test]
    fn flipper_unit_test_decoders() {
        crate::protocols::keys::load_keystore_from_embedded();
        let dir = Path::new(FLIPPER_UNIT_TESTS);
        if !dir.is_dir() {
            eprintln!("Skip: {FLIPPER_UNIT_TESTS} not found");
            return;
        }
        let mut reg = ProtocolRegistry::new();
        let mut failures = Vec::new();
        for &(file, protocol) in FLIPPER_DECODER_CASES {
            let (_, decodes) = decode_import(&mut reg, &dir.join(file)).expect(file);
            let found: Vec<String> = decodes.iter().map(|(n, _)| decoder_of(&reg, n)).collect();
            if !found.iter().any(|n| n == protocol) {
                failures.push(format!("{file}: no {protocol} decode (got {found:?})"));
            }
        }
        assert!(failures.is_empty(), "Flipper decoder cases failing:\n{}", failures.join("\n"));
    }

    #[test]
    fn flipper_unit_test_encoders() {
        crate::protocols::keys::load_keystore_from_embedded();
        let dir = Path::new(FLIPPER_UNIT_TESTS);
        if !dir.is_dir() {
            eprintln!("Skip: {FLIPPER_UNIT_TESTS} not found");
            return;
        }
        let mut reg = ProtocolRegistry::new();
        let mut failures = Vec::new();
        for &file in FLIPPER_ENCODER_CASES {
            let text = std::fs::read_to_string(dir.join(file)).expect(file);
            let key = FlipperKey::parse(&text).expect(file);
            let Some((name, decoded)) = reg.decode_key(&key) else {
                failures.push(format!("{file}: key file for {:?} does not load", key.protocol));
                continue;
            };
            let protocol = decoder_of(&reg, &name);
            let wire = reg.get(&name).and_then(|d| d.encode(&decoded, decoded.button.unwrap_or(0)));
            let Some(wire) = wire else {
                failures.push(format!("{file}: {name} does not encode"));
                continue;
            };
            let back: Vec<String> = reg
                .process_signal_stream(&wire, key.frequency)
                .iter()
                .map(|(n, _, _)| decoder_of(&reg, n))
                .collect();
            if !back.contains(&protocol) {
                failures.push(format!("{file}: {protocol} upload decodes as {back:?}"));
            }
        }
        assert!(failures.is_empty(), "Flipper encoder cases failing:\n{}", failures.join("\n"));
    }
}
