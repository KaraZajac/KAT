//! KeeLoq protocol decoder and encoder.
//!
//! Reference: Flipper-ARF `REFERENCES/Flipper-ARF/lib/subghz/protocols/keeloq.c` (Unleashed lineage;
//! ProtoPirate has no KeeLoq protocol). doctor-firmware's copy adds Monarch/KEY discriminators and
//! more learning types whose keys are not in KAT's keystore; ARF is followed where they differ.
//!
//! **Air format** (`subghz_protocol_decoder_keeloq_feed`): PWM, te_short 400 µs, te_long 800 µs,
//! te_delta 180 µs — the tolerance of ARF's gate app copy
//! (`applications/system/subghz_garage/protocols/keeloq.c`, a receive-only KeeLoq that ARF uses for
//! gate remotes); its main library copy uses 140 µs. KAT takes the wider one (union of what the two
//! decode: the real rtl_433 HCS200 recording `keeloq/01/g013` yields 2 frames instead of 1).
//! Preamble: HIGH 400 ± 180 µs pulses, each followed by a LOW of ~400 µs (or ~800 µs, ARF's
//! Superrollo/HCS361 asymmetric preamble); after > 2 of them a 4000 ± 1800 µs pulse (LOW, or a
//! Superrollo HIGH sync followed by the LOW) starts the data. Bit 1 = 400 HIGH + 800 LOW, bit 0 = 800
//! HIGH + 400 LOW (long side ± 360 µs). A LOW ≥ 980 µs ends the frame; 64..66 bits → frame (first 64
//! kept). Out-of-range lengths leave `header_count` as is, exactly like the C.
//!
//! **Frame**: 64 bits MSB-first; bit-reversed: `fix(32) | hop(32)`, fix = button(4) << 28 | serial(28).
//!
//! **Identification** (`subghz_protocol_keeloq_check_remote_controller`, decode flow: no saved
//! manufacturer): AN-Motors (`hop[31:24] == hop[23:16]`, button echoed in hop[15:12], hop[11:0] =
//! 0x404) → HCS101 (hop[11:0] = 0, button echoed) → the keystore selector, which walks every entry
//! ([`keeloq_keystore`]) and applies ONLY that entry's learning type: SIMPLE (Pecinin: 12-bit serial
//! check, button from the plaintext), NORMAL (Centurion: discriminator 0x1CE), SECURE (BFT: seed 0,
//! then seed = serial, then 0 again), MAGIC_XOR_TYPE_1, MAGIC_SERIAL_TYPE_1/2/3, AERF (derived decrypt
//! with 0x240 / 0x210 rounds, then plain), ERREKA, PUJOL, SIMPLE_JCM, and doctor-firmware's JCM_GEN2
//! (16: the learning byte searched over 0..=255 and kept as the seed), STAGNOLI (17), TELCOMA_TABLE_HI
//! (18: the table from the HI and LO entries, [`keeloq_telcoma_table`]; LO, 19, has no case of its own), and UNKNOWN (simple, normal,
//! secure, magic-xor — each with the key and its byte-mirrored copy). The check: plaintext button ==
//! fix button AND plaintext bits 23..16 == `fix & 0xFF` or == 0. The first match names the
//! manufacturer. Counter = plaintext[15:0] (AN-Motors/HCS101: hop >> 16).
//!
//! Identified frames are reported as `KeeLoq (<manufacturer>)`; like the C, every other frame is
//! reported too, as `KeeLoq (Unknown)` (serial and button from the fixed part, no counter, decode
//! and replay only: ARF would re-send the captured hop under a new button, which receivers
//! reject). The keystore is the embedded one with one evidence-backed learning-type correction
//! (DoorHan: SIMPLE, see `keeloq_common::LEARNING_TYPE_FIXES`). With it a random 64-bit frame is
//! identified ≈6% of the time, so strongly-gated KeeLoq-PWM protocols (Sheriff CFM) are registered
//! first. The decoder runs on every band (the C has no frequency filter).
//!
//! **Encoder** (`subghz_protocol_keeloq_gen_data` + `encode_to_timings`, counter mode 0): the
//! manufacturer, learning mode, counter and seed are re-derived from the key by the same
//! identification (the C re-runs it on the saved key and saved "Manufacture"/"Seed"), restricted
//! to the manufacturer the decode names (`KeeLoq (<name>)`, e.g. a key file's `Manufacture`, as
//! ARF's encoder restricts it to the file's name) when the key validates under it. Counter + 1 (0
//! past 0xFFFF). `fix = button << 28 | serial` (Pecinin: serial only). Plaintext `button << 28 |
//! disc << 16 | counter` with disc = serial & 0x3FF, or the manufacturer's rule (Aprimatic parity
//! form, 12-bit for DTM_Neo/FAAC_RC,XT/Mutanco/Came_Space/Genius_Bravo/GSN/Rosh/Rossi/Pecinin/
//! Steelmate/Cardin_S449/Stilmatic/…, 8-bit for NICE_Smilo/NICE_MHOUSE/JCM_Tech/Pujol/Erreka,
//! 0 for Beninca/Merlin, 0x1CE Centurion, 0x100 Monarch; Dea_Mio as RogueMaster: the 12 bits the
//! remote sent, kept as the decode's seed, so that a copy sends the original's two OVR bits, and
//! `serial & 0x3FF` with both set for a remote made from nothing). Hop by the first
//! keystore entry with that name: SIMPLE, NORMAL, SECURE (seed), MAGIC_XOR (serial), MAGIC_SERIAL_1,
//! AERF, ERREKA, PUJOL, SIMPLE_JCM (8-bit disc), doctor-firmware's JCM_GEN2 (the decode's seed byte is
//! the learning byte), STAGNOLI and TELCOMA_TABLE_HI (no hop when the LO half is missing), UNKNOWN per matched mode with the key as stored
//! (never the mirrored copy — reference behaviour). MAGIC_SERIAL_2/3 have no encode case in the C:
//! the captured frame is sent unchanged. AN-Motors/HCS101 build their plaintext hop; unidentified frames
//! replay the captured hop under the new fix. Programming mode: button 0xF for BFT (and, as FoxFW,
//! Erreka) sends the seed as hop, Aprimatic 0x1A2B3C4D, Dea_Mio 0 (counter not advanced).
//! Upload: 11 × (400 HIGH, 400 LOW), 400 HIGH, 4000 LOW, 64 bits, a status bit (400/800), 400 HIGH
//! and a 16 ms LOW gap (Sommer 11.6 ms); "Superrollo" uses its own HCS361 framing. The C yields it 3
//! times (`encoder.repeat = 3`), so the KAT waveform holds 3 copies.
//!
//! **Buttons**: 4-bit KeeLoq function codes; KAT's generic Lock/Unlock/Trunk/Panic (1/2/4/8) and
//! other codes 1..F are sent as the function code, anything else keeps the captured button. ARF
//! labels the "shifted" brands' codes 2/4/8/1 as physical buttons B1..B4 ([`keeloq_button_position`]).

use super::common::DecodedSignal;
use super::keeloq_common::{
    keeloq_decrypt, keeloq_decrypt_derived, keeloq_encrypt, keeloq_keystore,
    keeloq_learning_aerf, keeloq_learning_erreka, keeloq_learning_jcm_gen2, keeloq_learning_pujol,
    keeloq_learning_stagnoli, keeloq_learning_telcoma_table, keeloq_telcoma_table,
    keeloq_magic_serial_type1_learning, keeloq_magic_serial_type2_learning,
    keeloq_magic_serial_type3_learning, keeloq_magic_xor_type1_learning, keeloq_normal_learning,
    keeloq_secure_learning, learning_types as lt, mirrored_key, reverse_key, MfKey,
};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`) for "KeeLoq (<manufacturer>)" decodes and the generic fallback's
/// "Keeloq (<key>)" captures (`keeloq_generic.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "KeeLoq",
    decoder: "KeeLoq",
    aliases: &[],
    name_prefix: Some("keeloq ("),
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "KeeLoq",
    make: "KeeLoq",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// KeeLoq function codes are generic-compatible (1/2/4/8); Flipper-ARF only relabels the
/// "shifted" brands' codes 2/4/8/1 as physical buttons B1..B4 (registry decodes only).
fn native_buttons(name: &str, code: u8) -> Option<&'static str> {
    let manufacturer = name.strip_prefix("KeeLoq (")?.strip_suffix(')')?;
    Some(match keeloq_button_position(manufacturer, code)? {
        1 => "B1",
        2 => "B2",
        3 => "B3",
        4 => "B4",
        _ => return None,
    })
}

/// Display name of a frame no manufacturer key or rule identifies.
pub(super) const UNKNOWN_NAME: &str = "KeeLoq (Unknown)";

const TE_SHORT: u32 = 400;
const TE_LONG: u32 = 800;
const TE_DELTA: u32 = 180;
const MIN_COUNT_BIT: usize = 64;
/// RogueMaster `KEELOQ_TE_FAST`: the HCS200/HCS300/HCS301 baud-rate-select field sends the same
/// 66-bit frame at Te 200 µs instead of 400 µs ("half rate").
const TE_FAST: u32 = 200;
/// RogueMaster `KEELOQ_TE_FAST_GAP_DELTA`: the half rate is locked only when the sync gap (10·Te) is
/// within 2000 ± 450 µs; that measured gap / 10 is the frame's Te.
const TE_FAST_GAP_DELTA: u32 = 450;
/// doctor-firmware / RogueMaster `KEELOQ_MONARCH_DISCRIMINATOR_A` / `_B`: Monarch's hop
/// discriminator is constant per remote but not the same on every remote (B = Dorcas).
const MONARCH_DISC_A: u32 = 0x100;
const MONARCH_DISC_B: u32 = 0x280;
/// doctor-firmware / RogueMaster `KEELOQ_KEY_DISCRIMINATOR`: KEY remotes put no serial in the hop.
const KEY_DISC: u32 = 0x2FF;
/// RogueMaster `KEELOQ_PHANTOM_HILAND_DISC_MASK` / `_SERIAL_MASK` (Phantom/Hiland, HCS300): the
/// 12-bit discriminator carries `serial & 0x3FF` in its low 10 bits; the top 2 bits differ per remote
/// and the receiver ignores them.
const PHANTOM_DISC_MASK: u32 = 0xFFF;
const PHANTOM_SERIAL_MASK: u32 = 0x3FF;
/// RogueMaster `KEELOQ_DEA_MIO_*` (DEA Mio, HCS300): the 12-bit discriminator carries `serial &
/// 0x3FF` in its low 10 bits and the chip's two OVR flags above them. The chip sets an OVR bit
/// each time its counter wraps and never clears it, so remotes send them both set and both clear;
/// a receiver ignores them. The decoder matches the low 8 bits alone (as the generic check does),
/// and a remote made from nothing gets both OVR bits set.
const DEA_MIO_DISC_MASK: u32 = 0xFFF;
const DEA_MIO_SERIAL_MASK: u32 = 0x3FF;
const DEA_MIO_MATCH_MASK: u32 = 0x0FF;
const DEA_MIO_OVR_SET: u32 = 0xC00;

/// `subghz_protocol_keeloq_te_delta`: the tolerance scaled with the locked Te (400 → 180, 200 → 90).
fn te_delta(te: u32) -> u32 {
    te * TE_DELTA / TE_SHORT
}

/// `subghz_protocol_keeloq_te_nominal`: the rate a measured Te belongs to (what a key file records).
fn te_nominal(te: u32) -> u32 {
    if te * 2 < TE_FAST + TE_SHORT {
        TE_FAST
    } else {
        TE_SHORT
    }
}

/// `subghz_protocol_keeloq_te_sanitize`: a key file's `TE` (200 = half rate, anything else the
/// standard rate).
fn te_sanitize(te: u32) -> u32 {
    if te == TE_FAST {
        TE_FAST
    } else {
        TE_SHORT
    }
}

/// `DecodedSignal::extra` of a KeeLoq decode. Its low 32 bits hold a key file's seed that the decode
/// flow cannot reproduce (BFT, Erreka); a half-rate frame's Te (200) sits in bits 32..=47, with
/// [EXTRA_SEED_FLAG] marking a seed beside it. Standard-rate decodes without such a seed keep
/// `extra` None, and a value with nothing above bit 31 (every capture saved before the half rate)
/// is a seed.
const EXTRA_TE_SHIFT: u32 = 32;
const EXTRA_SEED_FLAG: u64 = 1 << 48;

/// The seed in a KeeLoq decode's `extra` (see [EXTRA_TE_SHIFT]).
fn extra_seed(extra: Option<u64>) -> Option<u32> {
    let e = extra?;
    (e >> 32 == 0 || e & EXTRA_SEED_FLAG != 0).then_some(e as u32)
}

/// The Te (200 or 400) in a KeeLoq decode's `extra` (see [EXTRA_TE_SHIFT]).
fn extra_te(extra: Option<u64>) -> u32 {
    te_sanitize(extra.map_or(0, |e| ((e >> EXTRA_TE_SHIFT) & 0xFFFF) as u32))
}

/// A KeeLoq decode's `extra` for `seed` (kept only when the decode cannot reproduce it) and Te.
fn make_extra(seed: Option<u32>, te: u32) -> Option<u64> {
    if te == TE_FAST {
        Some((u64::from(TE_FAST) << EXTRA_TE_SHIFT) | seed.map_or(0, |s| EXTRA_SEED_FLAG | u64::from(s)))
    } else {
        seed.map(u64::from)
    }
}

/// `subghz_protocol_keeloq_phantom_hiland_disc`: the seed when its low 10 bits belong to this
/// serial, else `serial & 0x3FF`.
fn phantom_hiland_disc(serial: u32, seed: u32) -> u32 {
    let disc = seed & PHANTOM_DISC_MASK;
    if disc & PHANTOM_SERIAL_MASK == serial & PHANTOM_SERIAL_MASK {
        disc
    } else {
        serial & PHANTOM_SERIAL_MASK
    }
}

/// RogueMaster `subghz_protocol_keeloq_dea_mio_disc`: the seed (the discriminator as received)
/// when it is not 0 and its low 8 bits belong to this serial, else `serial & 0x3FF` with both OVR
/// bits set.
fn dea_mio_disc(serial: u32, seed: u32) -> u32 {
    let disc = seed & DEA_MIO_DISC_MASK;
    if seed == 0 || disc & DEA_MIO_MATCH_MASK != serial & DEA_MIO_MATCH_MASK {
        (serial & DEA_MIO_SERIAL_MASK) | DEA_MIO_OVR_SET
    } else {
        disc
    }
}

/// `subghz_protocol_keeloq_check_decrypt_monarch`: the plaintext's discriminator is A or B (kept as
/// the seed).
fn check_decrypt_monarch(decrypt: u32, btn: u32) -> Option<u32> {
    let disc = (decrypt >> 16) & 0xFFF;
    (decrypt >> 28 == btn && (disc == MONARCH_DISC_A || disc == MONARCH_DISC_B)).then_some(disc)
}

/// `subghz_protocol_keeloq_check_decrypt_phantom_hiland`: the discriminator's low 10 bits are the
/// serial's (the whole 12-bit field is kept as the seed).
fn check_decrypt_phantom_hiland(decrypt: u32, btn: u32, fix: u32) -> Option<u32> {
    let disc = (decrypt >> 16) & PHANTOM_DISC_MASK;
    (decrypt >> 28 == btn && disc & PHANTOM_SERIAL_MASK == fix & PHANTOM_SERIAL_MASK).then_some(disc)
}

/// RogueMaster `subghz_protocol_keeloq_check_decrypt_dea_mio`: the discriminator's low 8 bits are
/// the serial's (the whole 12-bit field, OVR bits included, is kept as the seed). Unlike the
/// generic check, a low byte of 0 is not taken for any serial.
fn check_decrypt_dea_mio(decrypt: u32, btn: u32, end_serial: u32) -> Option<u32> {
    let disc = (decrypt >> 16) & DEA_MIO_DISC_MASK;
    (decrypt >> 28 == btn && disc & DEA_MIO_MATCH_MASK == end_serial & DEA_MIO_MATCH_MASK).then_some(disc)
}

/// `subghz_protocol_keeloq_check_decrypt_key`: the constant discriminator 0x2FF.
fn check_decrypt_key(decrypt: u32, btn: u32) -> bool {
    decrypt >> 28 == btn && (decrypt >> 16) & 0xFFF == KEY_DISC
}
/// C `encoder.repeat` (counter mode 0).
const TX_REPEAT: usize = 3;

/// ARF `keeloq_shifted_btn_brands`: physical button 1..4 = function code 2, 4, 8, 1.
const SHIFTED_BTN_BRANDS: &[&str] = &[
    "Nissan",
    "Suzuki",
    "Pandora_SUZUKI",
    "DoorHan",
    "Pandora_DEA",
    "Pandora_GIBIDI",
    "Pandora_MCODE",
    "Pandora_Unknown_1",
    "Pandora_Unknown_2",
    "Alligator_S-275",
    "Pantera_XS/Jaguar",
    "APS-1100_APS-2550",
];

/// ARF `keeloq_btn_get_position`: physical button (1..4) of a function code for the shifted brands.
pub fn keeloq_button_position(manufacturer: &str, code: u8) -> Option<u8> {
    if !SHIFTED_BTN_BRANDS.contains(&manufacturer) {
        return None;
    }
    match code {
        0x2 => Some(1),
        0x4 => Some(2),
        0x8 => Some(3),
        0x1 => Some(4),
        _ => None,
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Step {
    Reset,
    CheckPreamble,
    SaveDuration,
    CheckDuration,
}

/// A keystore selector match (the state `check_remote_controller_selector` leaves behind).
#[derive(Debug, Clone)]
struct KeyMatch {
    entry: &'static MfKey,
    /// C `keystore->kl_type` for UNKNOWN-entry matches (1 simple, 2 normal, 3 secure, 4 magic xor);
    /// 0 for a typed entry's own learning.
    kl_type: u8,
    decrypt: u32,
    /// C `instance->seed` after the search (BFT: 0 or the serial).
    seed: u32,
}

/// Result of `subghz_protocol_keeloq_check_remote_controller`.
#[derive(Debug, Clone)]
enum Ident {
    AnMotors,
    Hcs101,
    Key(KeyMatch),
}

impl Ident {
    fn name(&self) -> &str {
        match self {
            Ident::AnMotors => "AN-Motors",
            Ident::Hcs101 => "HCS101",
            Ident::Key(m) => &m.entry.name,
        }
    }
}

/// `subghz_protocol_keeloq_check_decrypt`.
fn check_decrypt(decrypt: u32, btn: u32, end_serial: u32) -> bool {
    decrypt >> 28 == btn && {
        let s = (decrypt >> 16) & 0xFF;
        s == end_serial || s == 0
    }
}

/// `subghz_protocol_keeloq_check_remote_controller_selector` (no saved manufacturer).
fn select_key(fix: u32, hop: u32, keys: &'static [MfKey]) -> Option<KeyMatch> {
    let end_serial = fix & 0xFF;
    let btn = fix >> 28;
    // C `instance->seed`: 0 in the decode flow; only BFT's branch changes it (and restores it).
    let mut seed: u32 = 0;
    let hit = |entry: &'static MfKey, kl_type: u8, decrypt: u32, seed: u32| {
        Some(KeyMatch {
            entry,
            kl_type,
            decrypt,
            seed,
        })
    };
    for entry in keys {
        let key = entry.key;
        match entry.learning {
            lt::SIMPLE => {
                let d = keeloq_decrypt(hop, key);
                if entry.name == "Pecinin" {
                    // `subghz_protocol_keeloq_check_decrypt_pecinin`: no button in fix.
                    if (d >> 16) & 0xFFF == fix & 0xFFF {
                        return hit(entry, 0, d, seed);
                    }
                } else if entry.name == "KEY" {
                    // doctor-firmware / RogueMaster: discriminator 0x2FF.
                    if check_decrypt_key(d, btn) {
                        return hit(entry, 0, d, seed);
                    }
                } else if entry.name == "Dea_Mio" {
                    // RogueMaster: the serial's low 8 bits, the 12-bit field kept as the seed.
                    if let Some(disc) = check_decrypt_dea_mio(d, btn, end_serial) {
                        return hit(entry, 0, d, disc);
                    }
                } else if check_decrypt(d, btn, end_serial) {
                    return hit(entry, 0, d, seed);
                }
            }
            lt::NORMAL => {
                let d = keeloq_decrypt(hop, keeloq_normal_learning(fix, key));
                if entry.name == "Centurion" {
                    // `subghz_protocol_keeloq_check_decrypt_centurion`.
                    if d >> 28 == btn && (d >> 16) & 0x3FF == 0x1CE {
                        return hit(entry, 0, d, seed);
                    }
                } else if entry.name == "Monarch" {
                    // doctor-firmware / RogueMaster: discriminator A or B, kept as the seed.
                    if let Some(disc) = check_decrypt_monarch(d, btn) {
                        return hit(entry, 0, d, disc);
                    }
                } else if entry.name == "Phantom_Hiland" {
                    // RogueMaster: the serial's low 10 bits, the 12-bit field kept as the seed.
                    if let Some(disc) = check_decrypt_phantom_hiland(d, btn, fix) {
                        return hit(entry, 0, d, disc);
                    }
                } else if check_decrypt(d, btn, end_serial) {
                    return hit(entry, 0, d, seed);
                }
            }
            lt::SECURE => {
                let mut reset_seed_back = false;
                if entry.name == "BFT" {
                    let d = keeloq_decrypt(hop, keeloq_secure_learning(fix, seed, key));
                    if check_decrypt(d, btn, end_serial) {
                        return hit(entry, 0, d, seed);
                    }
                    seed = fix & 0x0FFF_FFFF;
                    reset_seed_back = true;
                }
                let d = keeloq_decrypt(hop, keeloq_secure_learning(fix, seed, key));
                if check_decrypt(d, btn, end_serial) {
                    return hit(entry, 0, d, seed);
                }
                if reset_seed_back {
                    seed = 0;
                }
                let d = keeloq_decrypt(hop, keeloq_secure_learning(fix, seed, key));
                if check_decrypt(d, btn, end_serial) {
                    return hit(entry, 0, d, seed);
                }
            }
            lt::MAGIC_XOR_TYPE_1 | lt::MAGIC_SERIAL_TYPE_1 | lt::MAGIC_SERIAL_TYPE_2
            | lt::MAGIC_SERIAL_TYPE_3 | lt::PUJOL | lt::ERREKA => {
                let man = match entry.learning {
                    lt::MAGIC_XOR_TYPE_1 => keeloq_magic_xor_type1_learning(fix, key),
                    lt::MAGIC_SERIAL_TYPE_1 => keeloq_magic_serial_type1_learning(fix, key),
                    lt::MAGIC_SERIAL_TYPE_2 => keeloq_magic_serial_type2_learning(fix, key),
                    lt::MAGIC_SERIAL_TYPE_3 => keeloq_magic_serial_type3_learning(fix, key),
                    lt::PUJOL => keeloq_learning_pujol(fix, key),
                    _ => keeloq_learning_erreka(fix, seed, key),
                };
                let d = keeloq_decrypt(hop, man);
                if check_decrypt(d, btn, end_serial) {
                    return hit(entry, 0, d, seed);
                }
            }
            lt::AERF => {
                let man = keeloq_learning_aerf(fix, key);
                for d in [
                    keeloq_decrypt_derived(hop, man, 0x240),
                    keeloq_decrypt_derived(hop, man, 0x210),
                    keeloq_decrypt(hop, man),
                ] {
                    if check_decrypt(d, btn, end_serial) {
                        return hit(entry, 0, d, seed);
                    }
                }
            }
            lt::SIMPLE_JCM => {
                let d = keeloq_decrypt(hop, key);
                if check_decrypt(d, btn, end_serial) {
                    return hit(entry, 0, d, seed);
                }
            }
            // doctor-firmware: the learning byte is searched over 0..=255 and kept as the seed.
            lt::JCM_GEN2 => {
                for jcm_seed in 0..=0xFFu32 {
                    let man = keeloq_learning_jcm_gen2(fix, jcm_seed as u8, key);
                    let d = keeloq_decrypt(hop, man);
                    if check_decrypt(d, btn, end_serial) {
                        return hit(entry, 0, d, jcm_seed);
                    }
                }
            }
            lt::STAGNOLI => {
                let d = keeloq_decrypt(hop, keeloq_learning_stagnoli(fix, key));
                if check_decrypt(d, btn, end_serial) {
                    return hit(entry, 0, d, seed);
                }
            }
            // Only the HI entry acts (the LO entry has no case); it needs both halves.
            lt::TELCOMA_TABLE_HI => {
                if let Some(table) = keeloq_telcoma_table() {
                    let d = keeloq_decrypt(hop, keeloq_learning_telcoma_table(fix, &table));
                    if check_decrypt(d, btn, end_serial) {
                        return hit(entry, 0, d, seed);
                    }
                }
            }
            lt::UNKNOWN => {
                if let Some(m) = unknown_learning(entry, fix, hop, seed) {
                    return Some(m);
                }
            }
            // FAAC (5), Beninca ARC (9), Kinggates (10), Jarolift (11) and KAT-only types: no case.
            _ => {}
        }
    }
    None
}

/// The C's `KEELOQ_LEARNING_UNKNOWN` attempts for one entry: simple, normal, secure (seed) and
/// magic-xor, each with the key and its byte-mirrored copy (the C's order).
fn unknown_learning(entry: &'static MfKey, fix: u32, hop: u32, seed: u32) -> Option<KeyMatch> {
    let end_serial = fix & 0xFF;
    let btn = fix >> 28;
    let key = entry.key;
    let rev = mirrored_key(key);
    let attempts = [
        (key, 1u8),
        (rev, 1),
        (keeloq_normal_learning(fix, key), 2),
        (keeloq_normal_learning(fix, rev), 2),
        (keeloq_secure_learning(fix, seed, key), 3),
        (keeloq_secure_learning(fix, seed, rev), 3),
        (keeloq_magic_xor_type1_learning(fix, key), 4),
        (keeloq_magic_xor_type1_learning(fix, rev), 4),
    ];
    for (man, kl_type) in attempts {
        let decrypt = keeloq_decrypt(hop, man);
        if check_decrypt(decrypt, btn, end_serial) {
            return Some(KeyMatch {
                entry,
                kl_type,
                decrypt,
                seed,
            });
        }
    }
    None
}

/// `subghz_protocol_keeloq_check_remote_controller` (decode flow: no saved manufacturer).
fn identify(fix: u32, hop: u32) -> Option<Ident> {
    if hop >> 24 == (hop >> 16) & 0xFF && fix >> 28 == (hop >> 12) & 0x0F && hop & 0xFFF == 0x404 {
        return Some(Ident::AnMotors);
    }
    if hop & 0xFFF == 0 && fix >> 28 == (hop >> 12) & 0x0F {
        return Some(Ident::Hcs101);
    }
    select_key(fix, hop, keeloq_keystore()).map(Ident::Key)
}

/// Split a received 64-bit key into (fix, hop).
pub(super) fn fix_hop(data: u64) -> (u32, u32) {
    let reversed = reverse_key(data, MIN_COUNT_BIT);
    ((reversed >> 32) as u32, reversed as u32)
}

/// Decoded fields of an identified frame: (serial, button, counter).
fn fields(fix: u32, hop: u32, ident: &Ident) -> (u32, u8, u16) {
    let serial = fix & 0x0FFF_FFFF;
    match ident {
        Ident::AnMotors | Ident::Hcs101 => (serial, (fix >> 28) as u8, (hop >> 16) as u16),
        Ident::Key(m) => {
            let btn = if m.entry.name == "Pecinin" {
                m.decrypt >> 28
            } else {
                fix >> 28
            };
            (serial, btn as u8, m.decrypt as u16)
        }
    }
}

/// Plaintext discriminator (bits 27..16 of the hop plaintext) per manufacturer (`gen_data`); `seed`
/// is the decode's or key file's (Monarch's, Phantom/Hiland's and Dea_Mio's discriminator lives
/// there).
fn discriminator(mf: &str, serial: u32, seed: u32) -> u32 {
    match mf {
        "Aprimatic" => {
            let parity = (serial & 0x3FF).count_ones();
            let mut s = serial & 0x3FF;
            if parity.is_multiple_of(2) {
                s |= 0xC00;
            }
            s & 0xFFF
        }
        "DTM_Neo" | "FAAC_RC,XT" | "Mutanco_Mutancode" | "Clemsa_Mutancode" | "Came_Space"
        | "Genius_Bravo" | "GSN" | "Rosh" | "Rossi" | "Pecinin" | "Steelmate" | "Cardin_S449"
        | "Stilmatic" | "Wisniowski" | "ATA_PTX4" | "Fadini" | "Seav" => serial & 0xFFF,
        "NICE_Smilo" | "NICE_MHOUSE" | "JCM_Tech" | "Pujol_Vario" | "Pujol" | "Erreka" => {
            serial & 0xFF
        }
        "Beninca" | "Merlin" => 0x000,
        "Centurion" => 0x1CE,
        // doctor-firmware / RogueMaster: the received discriminator (A or B, kept in the seed),
        // A when the seed holds neither (Flipper-ARF: always 0x100 = A).
        "Monarch" => match seed & 0xFFF {
            d @ (MONARCH_DISC_A | MONARCH_DISC_B) => d,
            _ => MONARCH_DISC_A,
        },
        "Phantom_Hiland" => phantom_hiland_disc(serial, seed),
        "KEY" => KEY_DISC,
        // RogueMaster: the received discriminator (kept in the seed) when it belongs to this
        // serial, else both OVR bits set (what the other firmwares always send: their
        // `0xC + (serial >> 8) % 4` above the serial's low byte is the same value).
        "Dea_Mio" => dea_mio_disc(serial, seed),
        _ => serial & 0x3FF,
    }
}

/// `subghz_protocol_keeloq_gen_data` for a captured key: the 64-bit key to send (and the
/// manufacturer name it was built for). `button` is already a 4-bit function code.
fn gen_data(data: u64, btn: u8, ident: Option<&Ident>, fallback_cnt: u16) -> u64 {
    let (fix_c, hop_c) = fix_hop(data);
    let serial = fix_c & 0x0FFF_FFFF;
    let mf = ident.map_or("Unknown", Ident::name);
    let (cnt_base, seed) = match ident {
        Some(Ident::Key(m)) => (m.decrypt as u16, m.seed),
        Some(_) => ((hop_c >> 16) as u16, 0),
        None => (fallback_cnt, 0),
    };
    let btn32 = u32::from(btn & 0x0F);
    let fix = if mf == "Pecinin" {
        serial
    } else {
        (btn32 << 28) | serial
    };
    // Programming mode (C prog_mode, per press): button 0xF for BFT / Aprimatic / Dea_Mio, and as
    // FoxFW's `PROG_MODE_KEELOQ_ERREKA` for Erreka (the seed as hop, like BFT).
    let prog = btn == 0xF && matches!(mf, "BFT" | "Aprimatic" | "Dea_Mio" | "Erreka");
    let mut hop: u32 = match (prog, mf) {
        (true, "BFT" | "Erreka") => seed,
        (true, "Aprimatic") => 0x1A2B_3C4D,
        _ => 0,
    };
    if !prog {
        let cnt = u32::from(cnt_base.checked_add(1).unwrap_or(0));
        hop = match ident {
            None => hop_c,
            Some(Ident::AnMotors) => {
                ((cnt & 0xFF) << 24) | ((cnt & 0xFF) << 16) | (btn32 << 12) | 0x404
            }
            Some(Ident::Hcs101) => (cnt << 16) | (btn32 << 12),
            Some(Ident::Key(m)) => {
                // The C uses the first keystore entry with this name.
                let entry = keeloq_keystore()
                    .iter()
                    .find(|e| e.name == m.entry.name)
                    .unwrap_or(m.entry);
                encrypt_hop(mf, entry, m.kl_type, fix, serial, btn32, cnt, seed)
            }
        };
    }
    if hop != 0 || (prog && matches!(mf, "BFT" | "Dea_Mio" | "Erreka")) {
        reverse_key((u64::from(fix) << 32) | u64::from(hop), MIN_COUNT_BIT)
    } else {
        // C: the key is left as it was — the captured frame is sent unchanged.
        data
    }
}

/// `gen_data`'s hop for a keystore manufacturer: the plaintext `button << 28 | disc << 16 |
/// counter` ([discriminator]) encrypted per the entry's learning type, as the C's `switch`.
#[allow(clippy::too_many_arguments)]
fn encrypt_hop(mf: &str, entry: &MfKey, kl_type: u8, fix: u32, serial: u32, btn32: u32, cnt: u32, seed: u32) -> u32 {
    let plain = (btn32 << 28) | (discriminator(mf, serial, seed) << 16) | cnt;
    let key = entry.key;
    match entry.learning {
        lt::SIMPLE => keeloq_encrypt(plain, key),
        lt::NORMAL => keeloq_encrypt(plain, keeloq_normal_learning(fix, key)),
        lt::SECURE => keeloq_encrypt(plain, keeloq_secure_learning(fix, seed, key)),
        lt::MAGIC_XOR_TYPE_1 => keeloq_encrypt(plain, keeloq_magic_xor_type1_learning(serial, key)),
        lt::MAGIC_SERIAL_TYPE_1 => keeloq_encrypt(plain, keeloq_magic_serial_type1_learning(fix, key)),
        lt::AERF => keeloq_encrypt(plain, keeloq_learning_aerf(fix, key)),
        lt::ERREKA => keeloq_encrypt(plain, keeloq_learning_erreka(fix, seed, key)),
        lt::PUJOL => keeloq_encrypt(plain, keeloq_learning_pujol(fix, key)),
        lt::SIMPLE_JCM => {
            let plain = (btn32 << 28) | ((serial & 0xFF) << 16) | cnt;
            keeloq_encrypt(plain, key)
        }
        lt::JCM_GEN2 => keeloq_encrypt(plain, keeloq_learning_jcm_gen2(fix, seed as u8, key)),
        lt::STAGNOLI => keeloq_encrypt(plain, keeloq_learning_stagnoli(fix, key)),
        lt::TELCOMA_TABLE_HI => match keeloq_telcoma_table() {
            Some(table) => keeloq_encrypt(plain, keeloq_learning_telcoma_table(fix, &table)),
            None => 0,
        },
        // C: per kl_type, always with the key as stored (never the mirrored copy).
        lt::UNKNOWN => unknown_encrypt(plain, kl_type, fix, serial, seed, key),
        // MAGIC_SERIAL_TYPE_2/3 and the rest have no encode case: hop stays 0.
        _ => 0,
    }
}

/// `subghz_protocol_keeloq_create_data` / `_bft_create_data` (the SubGHz app's "Add Manually",
/// `subghz_txrx_gen_keeloq_protocol` / `_keeloq_bft_protocol`): the key file of a new remote.
/// `gen_data` with the counter as given (`counter_up` false) and the button as given
/// (`skip_btn_check`), programming mode for button 0xF of BFT / Aprimatic / Dea_Mio as in
/// [gen_data]; the hop of a keystore manufacturer uses its first entry (a learning type 0 entry
/// encrypts nothing: the keystore's `kl_type` is 0 in a fresh session). Lines: `Bit: 64`, `Key`,
/// then `Seed` (4 hex bytes, seed types only) and `Manufacture`. None when no hop can be built
/// (the manufacturer is not in the keystore, or its learning type has no encode case): the C
/// would save the encoder's empty key.
pub(super) fn create_data(
    serial: u32,
    btn: u8,
    cnt: u16,
    manufacturer: &str,
    seed: Option<u32>,
) -> Option<super::FlipperKey> {
    let mf = manufacturer;
    let btn32 = u32::from(btn & 0x0F);
    let fix = if mf == "Pecinin" { serial } else { (btn32 << 28) | serial };
    let seed_value = seed.unwrap_or(0);
    let prog = btn == 0xF && matches!(mf, "BFT" | "Aprimatic" | "Dea_Mio" | "Erreka");
    let cnt = u32::from(cnt);
    let hop = match (prog, mf) {
        (true, "BFT" | "Erreka") => seed_value,
        (true, "Aprimatic") => 0x1A2B_3C4D,
        (true, _) => 0,
        (false, "AN-Motors") => ((cnt & 0xFF) << 24) | ((cnt & 0xFF) << 16) | (btn32 << 12) | 0x404,
        (false, "HCS101") => (cnt << 16) | (btn32 << 12),
        (false, _) => {
            let entry = keeloq_keystore().iter().find(|e| e.name == mf)?;
            encrypt_hop(mf, entry, 0, fix, serial, btn32, cnt, seed_value)
        }
    };
    if hop == 0 && !(prog && matches!(mf, "BFT" | "Dea_Mio" | "Erreka")) {
        return None;
    }
    let data = reverse_key((u64::from(fix) << 32) | u64::from(hop), MIN_COUNT_BIT);
    let mut key = super::FlipperKey::for_protocol("KeeLoq", MIN_COUNT_BIT, data);
    if let Some(seed) = seed {
        key.push_hex("Seed", u64::from(seed), 4);
    }
    key.push("Manufacture", mf);
    Some(key)
}

/// Whether [create_data] can build `manufacturer`'s hops with KAT's keystore.
pub(super) fn can_create(manufacturer: &str) -> bool {
    matches!(manufacturer, "AN-Motors" | "HCS101")
        || keeloq_keystore().iter().find(|e| e.name == manufacturer).is_some_and(|e| {
            matches!(
                e.learning,
                lt::SIMPLE
                    | lt::NORMAL
                    | lt::SECURE
                    | lt::MAGIC_XOR_TYPE_1
                    | lt::MAGIC_SERIAL_TYPE_1
                    | lt::AERF
                    | lt::ERREKA
                    | lt::PUJOL
                    | lt::SIMPLE_JCM
                    | lt::JCM_GEN2
                    | lt::STAGNOLI
                    | lt::TELCOMA_TABLE_HI
            )
        })
}

/// The manufacturer a decode names ("KeeLoq (<manufacturer>)": a key file's `Manufacture`, or the
/// identification on air), when the key validates under it — what Flipper-ARF's encoder uses: its
/// deserialize sets `keystore->mfname` from the file, so `check_remote_controller` searches only
/// that manufacturer's entries instead of the whole keystore. A BFT decode's `extra` holds the
/// seed a key file gave.
fn named_ident(decoded: &DecodedSignal, fix: u32, hop: u32) -> Option<Ident> {
    let mf = decoded
        .protocol_display_name
        .as_deref()?
        .strip_prefix("KeeLoq (")?
        .strip_suffix(')')?;
    if mf == "Unknown" {
        return None;
    }
    key_file_identify(fix, hop, mf, extra_seed(decoded.extra))
}

/// `gen_data`'s `KEELOQ_LEARNING_UNKNOWN` case: encrypt per `kl_type`.
fn unknown_encrypt(plain: u32, kl_type: u8, fix: u32, serial: u32, seed: u32, key: u64) -> u32 {
    match kl_type {
        1 => keeloq_encrypt(plain, key),
        2 => keeloq_encrypt(plain, keeloq_normal_learning(fix, key)),
        3 => keeloq_encrypt(plain, keeloq_secure_learning(fix, seed, key)),
        4 => keeloq_encrypt(plain, keeloq_magic_xor_type1_learning(serial, key)),
        _ => 0,
    }
}

/// One copy of the C upload (`subghz_protocol_encoder_keeloq_encode_to_timings`) at Te `te` (400, or
/// RogueMaster's half rate 200, which scales every pulse and the gap; Superrollo keeps its framing).
fn upload_once(data: u64, mf: &str, te: u32, signal: &mut Vec<LevelDuration>) {
    if mf == "Superrollo" {
        // HCS361 framing: Te 450, 9 × (Te, 2Te) preamble, 10Te sync, 64 bits + VLOW + 2-bit CRC.
        let t = 450;
        for _ in 0..9 {
            signal.push(LevelDuration::new(true, t));
            signal.push(LevelDuration::new(false, t * 2));
        }
        signal.push(LevelDuration::new(true, t * 10));
        signal.push(LevelDuration::new(false, t * 10));
        let w0 = reverse_key(data, 64);
        let (mut crc0, mut crc1) = (0u8, 0u8);
        for i in 0..65 {
            let din = if i < 64 { ((w0 >> i) & 1) as u8 } else { 1 };
            let nc1 = crc0 ^ din;
            crc0 = (nc1 ^ crc1) & 1;
            crc1 = nc1 & 1;
        }
        for i in (1..=67u32).rev() {
            let bit = match i {
                4..=67 => ((data >> (i - 4)) & 1) as u8,
                3 => 1,
                2 => crc0,
                _ => crc1,
            };
            signal.push(LevelDuration::new(true, if bit != 0 { t } else { t * 2 }));
            signal.push(LevelDuration::new(false, if bit != 0 { t * 2 } else { t }));
        }
        signal.push(LevelDuration::new(true, t));
        signal.push(LevelDuration::new(false, t * 18));
        return;
    }
    let (te_short, te_long) = (te, te * 2);
    let gap = if mf == "Sommer" { te_short * 29 } else { te_short * 40 };
    for _ in 0..11 {
        signal.push(LevelDuration::new(true, te_short));
        signal.push(LevelDuration::new(false, te_short));
    }
    signal.push(LevelDuration::new(true, te_short));
    signal.push(LevelDuration::new(false, te_short * 10));
    for i in (0..MIN_COUNT_BIT).rev() {
        if (data >> i) & 1 == 1 {
            signal.push(LevelDuration::new(true, te_short));
            signal.push(LevelDuration::new(false, te_long));
        } else {
            signal.push(LevelDuration::new(true, te_long));
            signal.push(LevelDuration::new(false, te_short));
        }
    }
    // Status bit, then the end pulse and gap.
    signal.push(LevelDuration::new(true, te_short));
    signal.push(LevelDuration::new(false, te_long));
    signal.push(LevelDuration::new(true, te_short));
    signal.push(LevelDuration::new(false, gap));
}

pub struct KeeloqDecoder {
    step: Step,
    header_count: u16,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: usize,
    /// RogueMaster's `te`: the frame's Te, locked from its sync gap (400, or the measured half rate).
    te: u32,
}

impl KeeloqDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            header_count: 0,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
            te: TE_SHORT,
        }
    }

    fn add_bit(&mut self, bit: u64) {
        if self.decode_count_bit < MIN_COUNT_BIT {
            self.decode_data = (self.decode_data << 1) | bit;
        }
        self.decode_count_bit += 1;
    }

    /// A completed 64..66-bit frame → the reported signal. Like the C, which calls back for every
    /// frame and shows an unidentified one as "Unknown" (`Btn:x Cnt:????`), an unidentified frame
    /// is reported as `KeeLoq (Unknown)`: serial and button from the fixed part, no counter, not
    /// re-encodable (the hop cannot be regenerated; Replay sends the captured frame).
    pub(super) fn parse_data(data: u64) -> Option<DecodedSignal> {
        let (fix, hop) = fix_hop(data);
        let Some(ident) = identify(fix, hop) else {
            return Some(DecodedSignal {
                serial: Some(fix & 0x0FFF_FFFF),
                button: Some((fix >> 28) as u8),
                counter: None,
                crc_valid: false,
                data,
                data_count_bit: MIN_COUNT_BIT,
                encoder_capable: false,
                extra: None,
                protocol_display_name: Some(UNKNOWN_NAME.to_string()),
            });
        };
        let (serial, btn, cnt) = fields(fix, hop, &ident);
        Some(DecodedSignal {
            serial: Some(serial),
            button: Some(btn),
            counter: Some(cnt),
            crc_valid: true,
            data,
            data_count_bit: MIN_COUNT_BIT,
            encoder_capable: true,
            extra: None,
            protocol_display_name: Some(format!("KeeLoq ({})", ident.name())),
        })
    }

    /// [Self::parse_data] for a frame received at Te `te` (a half-rate frame keeps its Te in
    /// `extra`, for the encoder and the key file).
    pub(super) fn parse_data_te(data: u64, te: u32) -> Option<DecodedSignal> {
        let mut d = Self::parse_data(data)?;
        d.extra = make_extra(None, te);
        Some(d)
    }
}

impl Default for KeeloqDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl super::ProtocolDecoder for KeeloqDecoder {
    fn name(&self) -> &'static str {
        "KeeLoq"
    }

    fn timing(&self) -> super::ProtocolTiming {
        super::ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: MIN_COUNT_BIT,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        // Every band, like the C (KeeLoq gate/garage remotes are sold for 300–318, 390, 433 and
        // 868 MHz).
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.step = Step::Reset;
        self.header_count = 0;
        self.te_last = 0;
        self.decode_data = 0;
        self.decode_count_bit = 0;
        self.te = TE_SHORT;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                // RogueMaster: the preamble is accepted at either rate; the sync gap decides which.
                if level
                    && (duration_diff!(duration, TE_SHORT) < TE_DELTA
                        || duration_diff!(duration, TE_FAST) < te_delta(TE_FAST))
                {
                    self.step = Step::CheckPreamble;
                    self.header_count = self.header_count.wrapping_add(1);
                } else if level
                    && self.header_count > 2
                    && duration_diff!(duration, TE_SHORT * 10) < TE_DELTA * 10
                {
                    // Superrollo GW60 (HCS361): 10·Te HIGH sync pulse before the sync gap.
                    self.step = Step::CheckPreamble;
                }
            }
            Step::CheckPreamble => {
                if !level
                    && (duration_diff!(duration, TE_SHORT) < TE_DELTA
                        || duration_diff!(duration, TE_SHORT * 2) < TE_DELTA
                        || duration_diff!(duration, TE_FAST) < te_delta(TE_FAST))
                {
                    self.step = Step::Reset;
                    return None;
                }
                if self.header_count > 2 {
                    // RogueMaster: the half rate first, on its own narrow band (the gap is 10·Te, so
                    // it measures the encoder's real Te); else the standard rate on its usual band.
                    let te = if duration_diff!(duration, TE_FAST * 10) < TE_FAST_GAP_DELTA {
                        duration / 10
                    } else if duration_diff!(duration, TE_SHORT * 10) < TE_DELTA * 10 {
                        TE_SHORT
                    } else {
                        0
                    };
                    if te != 0 {
                        // Found header
                        self.te = te;
                        self.step = Step::SaveDuration;
                        self.decode_data = 0;
                        self.decode_count_bit = 0;
                        return None;
                    }
                }
                self.step = Step::Reset;
                self.header_count = 0;
            }
            Step::SaveDuration => {
                if level {
                    self.te_last = duration;
                    self.step = Step::CheckDuration;
                }
            }
            Step::CheckDuration => {
                if !level {
                    // Every check scaled with the locked Te (400 µs: the C's constants).
                    let (te, delta) = (self.te, te_delta(self.te));
                    if duration >= te * 2 + delta {
                        // Found end TX
                        self.step = Step::Reset;
                        if (MIN_COUNT_BIT..=MIN_COUNT_BIT + 2).contains(&self.decode_count_bit) {
                            let result = Self::parse_data_te(self.decode_data, te_nominal(te));
                            self.decode_data = 0;
                            self.decode_count_bit = 0;
                            self.header_count = 0;
                            return result;
                        }
                    } else if duration_diff!(self.te_last, te) < delta
                        && duration_diff!(duration, te * 2) < delta * 2
                    {
                        self.add_bit(1);
                        self.step = Step::SaveDuration;
                    } else if duration_diff!(self.te_last, te * 2) < delta * 2
                        && duration_diff!(duration, te) < delta
                    {
                        self.add_bit(0);
                        self.step = Step::SaveDuration;
                    } else {
                        self.step = Step::Reset;
                        self.header_count = 0;
                    }
                } else {
                    self.step = Step::Reset;
                    self.header_count = 0;
                }
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        let (fix, hop) = fix_hop(decoded.data);
        // The manufacturer the capture names (its key file's, as ARF's encoder reads it), else the
        // decoder's identification.
        let ident = named_ident(decoded, fix, hop).or_else(|| identify(fix, hop));
        let orig_btn = match &ident {
            Some(i) => fields(fix, hop, i).1,
            None => (fix >> 28) as u8,
        };
        let btn = if (0x01..=0x0F).contains(&button) {
            button
        } else {
            orig_btn
        };
        let data = gen_data(
            decoded.data,
            btn,
            ident.as_ref(),
            decoded.counter.unwrap_or(0),
        );
        let mf = ident.as_ref().map_or("Unknown", Ident::name);
        let te = extra_te(decoded.extra);
        let mut signal = Vec::with_capacity(TX_REPEAT * 160);
        for _ in 0..TX_REPEAT {
            upload_once(data, mf, te, &mut signal);
        }
        Some(signal)
    }

    /// Key files: Flipper-ARF `subghz_protocol_decoder_keeloq_deserialize` (Bit must be 64; `Seed`
    /// = 4 hex bytes; `Manufacture`, with ARF's "Sommer(fsk476)" → "Sommer" fix) + the
    /// `check_remote_controller` its display runs with that manufacturer: "AN-Motors" / "HCS101"
    /// are forced, any other name restricts the keystore selector to that name's entries (BFT
    /// tries the file's Seed first, then the serial, then 0). A key that does not validate under
    /// the named manufacturer, or a name KAT's keystore lacks (doctor-firmware's own brand names,
    /// "Star" for its Star Line key …), is loaded like "Manufacture: Unknown" or no line: the
    /// frame is identified like one on air (the C loads such a file and shows "Unknown" at worst;
    /// refusing it, as KAT did until v1.8.6, left the user unable to list or replay the file).
    ///
    /// The decode keeps the name ("KeeLoq (<manufacturer>)") and KAT's encoder transmits with that
    /// manufacturer, as ARF's encoder does with the file's `Manufacture` (also when KAT's
    /// first-match search on air would pick another entry, e.g. a reference keystore's order, or a
    /// BFT key with a user-chosen seed: a seed the decode flow cannot reproduce is kept in `extra`,
    /// for the encoder and so the key file can be written back). A forced AN-Motors / HCS101 that
    /// the frame does not confirm is shown but not transmitted.
    fn from_flipper_key(&self, key: &super::FlipperKey) -> Option<DecodedSignal> {
        if key.bits != MIN_COUNT_BIT {
            return None;
        }
        let data = key.key_u64()?;
        // RogueMaster: `TE` (written for half-rate captures only) is the rate the frame is sent at.
        let te = te_sanitize(key.u32_field("TE").unwrap_or(TE_SHORT));
        let manufacturer = match key.field("Manufacture").map(str::trim) {
            Some("Sommer(fsk476)") => Some("Sommer"),
            other => other,
        };
        match manufacturer {
            None | Some("") | Some("Unknown") => Self::parse_data_te(data, te),
            Some(mf) => {
                let (fix, hop) = fix_hop(data);
                let seed = key
                    .hex_bytes_field("Seed")
                    .and_then(|b| <[u8; 4]>::try_from(b.as_slice()).ok())
                    .map(u32::from_be_bytes);
                match key_file_identify(fix, hop, mf, seed) {
                    Some(ident) => Some(key_file_signal(data, fix, hop, &ident, te)),
                    // A name KAT's keystore lacks (doctor-firmware's "Star", "Wisniowski2" …), or
                    // one whose key does not validate the frame: the C loads the file all the
                    // same and shows what its selector finds ("Unknown" at worst), so KAT
                    // identifies the frame like one on air.
                    None => Self::parse_data_te(data, te),
                }
            }
        }
    }

    /// Key files: Flipper-ARF `subghz_protocol_decoder_keeloq_serialize`: Protocol "KeeLoq", Bit
    /// 64, Key, then — for BFT, and as doctor-firmware / RogueMaster also for Erreka, Monarch,
    /// Phantom/Hiland and (RogueMaster) Dea_Mio (whose discriminator is the seed) — `Seed` (4 hex
    /// bytes, the seed the decrypt used) and `Manufacture` (the decode's `(<manufacturer>)`,
    /// re-checked against the key as the C re-runs its check; "Unknown" when nothing validates); RogueMaster's `TE: 200` after it for a
    /// half-rate frame only (standard frames keep the layout older builds read).
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<super::FlipperKey> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        let (fix, hop) = fix_hop(decoded.data);
        let named = decoded
            .protocol_display_name
            .as_deref()
            .and_then(|n| n.strip_prefix("KeeLoq ("))
            .and_then(|n| n.strip_suffix(')'))
            .filter(|&n| n != "Unknown");
        let ident = match named {
            Some(mf) => key_file_identify(fix, hop, mf, extra_seed(decoded.extra)),
            None => identify(fix, hop),
        };
        let mut key = super::FlipperKey::for_protocol("KeeLoq", MIN_COUNT_BIT, decoded.data);
        if let Some(Ident::Key(m)) = &ident {
            if matches!(m.entry.name.as_str(), "BFT" | "Erreka" | "Monarch" | "Phantom_Hiland" | "Dea_Mio") {
                key.push_hex("Seed", u64::from(m.seed), 4);
            }
        }
        key.push("Manufacture", ident.as_ref().map_or("Unknown", Ident::name));
        let te = extra_te(decoded.extra);
        if te != TE_SHORT {
            key.push("TE", te.to_string());
        }
        Some(key)
    }
}

/// Key files: the C's `check_remote_controller` with a saved manufacturer (see
/// `KeeloqDecoder::from_flipper_key`).
fn key_file_identify(fix: u32, hop: u32, manufacturer: &str, seed: Option<u32>) -> Option<Ident> {
    match manufacturer {
        "AN-Motors" => return Some(Ident::AnMotors),
        "HCS101" => return Some(Ident::Hcs101),
        _ => {}
    }
    for entry in keeloq_keystore().iter().filter(|e| e.name == manufacturer) {
        if let (Some(seed), lt::SECURE, "BFT") = (seed, entry.learning, entry.name.as_str()) {
            let decrypt = keeloq_decrypt(hop, keeloq_secure_learning(fix, seed, entry.key));
            if check_decrypt(decrypt, fix >> 28, fix & 0xFF) {
                return Some(Ident::Key(KeyMatch {
                    entry,
                    kl_type: 0,
                    decrypt,
                    seed,
                }));
            }
        }
        // doctor-firmware / RogueMaster: an Erreka key file's seed (its "Add Manually" draws one;
        // "without it the hops cannot be rebuilt"); the decode flow tries seed 0 only.
        if let (Some(seed), lt::ERREKA) = (seed, entry.learning) {
            let decrypt = keeloq_decrypt(hop, keeloq_learning_erreka(fix, seed, entry.key));
            if check_decrypt(decrypt, fix >> 28, fix & 0xFF) {
                return Some(Ident::Key(KeyMatch {
                    entry,
                    kl_type: 0,
                    decrypt,
                    seed,
                }));
            }
        }
        if let Some(m) = select_key(fix, hop, std::slice::from_ref(entry)) {
            return Some(Ident::Key(m));
        }
    }
    None
}

/// Key files: the decode for a key identified as `ident` (`parse_data`'s fields). Encoder-capable
/// when the decoder's own identification finds the same match, or when the named manufacturer's
/// key validates the frame (the encoder then uses that manufacturer, see `named_ident`); a forced
/// AN-Motors / HCS101 the frame does not confirm is not (see `KeeloqDecoder::from_flipper_key`).
fn key_file_signal(data: u64, fix: u32, hop: u32, ident: &Ident, te: u32) -> DecodedSignal {
    let (serial, btn, cnt) = fields(fix, hop, ident);
    let reproduced = match (identify(fix, hop), ident) {
        (Some(Ident::AnMotors), Ident::AnMotors) | (Some(Ident::Hcs101), Ident::Hcs101) => true,
        (Some(Ident::Key(a)), Ident::Key(b)) => {
            std::ptr::eq(a.entry, b.entry)
                && (a.kl_type, a.decrypt, a.seed) == (b.kl_type, b.decrypt, b.seed)
        }
        _ => false,
    };
    let encoder_capable = reproduced || matches!(ident, Ident::Key(_));
    let seed = match ident {
        Ident::Key(m) if (m.entry.name == "BFT" || m.entry.learning == lt::ERREKA) && !reproduced => Some(m.seed),
        _ => None,
    };
    let extra = make_extra(seed, te);
    DecodedSignal {
        serial: Some(serial),
        button: Some(btn),
        counter: Some(cnt),
        crc_valid: true,
        data,
        data_count_bit: MIN_COUNT_BIT,
        encoder_capable,
        extra,
        protocol_display_name: Some(format!("KeeLoq ({})", ident.name())),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::ProtocolDecoder;

    fn decode_all(signal: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = KeeloqDecoder::new();
        let mut out = Vec::new();
        for ld in signal {
            if let Some(d) = dec.feed(ld.level, ld.duration_us) {
                out.push(d);
                dec.reset(); // registry behaviour
            }
        }
        out
    }

    fn entry(name: &str) -> &'static MfKey {
        keeloq_keystore()
            .iter()
            .find(|e| e.name == name)
            .unwrap_or_else(|| panic!("keystore entry {name}"))
    }

    /// One standard KeeLoq upload copy for a raw key.
    fn capture(data: u64) -> Vec<LevelDuration> {
        let mut s = Vec::new();
        upload_once(data, "", TE_SHORT, &mut s);
        s
    }

    /// Build a remote's frame: fix = btn << 28 | serial, hop = KeeLoq(plain, man).
    fn frame(btn: u32, serial: u32, plain: u32, man: u64) -> u64 {
        let fix = (btn << 28) | (serial & 0x0FFF_FFFF);
        reverse_key((u64::from(fix) << 32) | u64::from(keeloq_encrypt(plain, man)), 64)
    }

    #[test]
    fn keeloq_decodes_each_learning_type() {
        let serial = 0x0ABC_DE12;
        let fix = (0x2u32 << 28) | serial;
        let plain = (0x2u32 << 28) | ((serial & 0x3FF) << 16) | 0x0777;
        let cases: Vec<(&str, u64)> = vec![
            ("Sheriff", entry("Sheriff").key), // SIMPLE
            ("Mongoose", keeloq_normal_learning(fix, entry("Mongoose").key)), // NORMAL
            ("Beninca", keeloq_magic_xor_type1_learning(fix, entry("Beninca").key)),
            ("KGB/Subaru", keeloq_magic_serial_type1_learning(fix, entry("KGB/Subaru").key)),
            ("Magic_2", keeloq_magic_serial_type2_learning(fix, entry("Magic_2").key)),
            ("Magic_4", keeloq_magic_serial_type3_learning(fix, entry("Magic_4").key)),
            ("BFT", keeloq_secure_learning(fix, serial, entry("BFT").key)),
            ("Leopard", mirrored_key(entry("Leopard").key)), // UNKNOWN, mirrored simple
            (
                "Reff",
                keeloq_secure_learning(fix, 0, entry("Reff").key), // UNKNOWN, secure seed 0
            ),
        ];
        for (name, man) in cases {
            let d = decode_all(&capture(frame(0x2, serial, plain, man)));
            assert_eq!(d.len(), 1, "{name}");
            let d = &d[0];
            assert_eq!(
                d.protocol_display_name.as_deref(),
                Some(format!("KeeLoq ({name})").as_str())
            );
            assert_eq!(d.serial, Some(serial));
            assert_eq!(d.button, Some(0x2));
            assert_eq!(d.counter, Some(0x0777));
        }
    }

    #[test]
    fn keeloq_typed_entries_use_only_their_learning_type() {
        // A frame encrypted with the SIMPLE "Sheriff" key under NORMAL learning is not identified as
        // Sheriff: the C tries only simple learning for a type-1 entry. Neither the KeeLoq decoder
        // nor the registry reports it as "KeeLoq (Sheriff)".
        use crate::protocols::ProtocolRegistry;
        let serial = 0x0123_4567;
        let fix = (0x1u32 << 28) | serial;
        let plain = (0x1u32 << 28) | ((serial & 0x3FF) << 16) | 0x0042;
        let data = frame(0x1, serial, plain, keeloq_normal_learning(fix, entry("Sheriff").key));
        let (f, h) = fix_hop(data);
        assert!(select_key(f, h, keeloq_keystore()).is_none_or(|m| m.entry.name != "Sheriff"));
        for d in decode_all(&capture(data)) {
            assert_ne!(d.protocol_display_name.as_deref(), Some("KeeLoq (Sheriff)"));
        }
        let mut reg = ProtocolRegistry::new();
        for (name, _, _) in reg.process_signal_stream(&capture(data), 433_920_000) {
            assert_ne!(name, "KeeLoq (Sheriff)");
        }
        // The same plaintext under the entry's own (simple) learning is identified.
        let data = frame(0x1, serial, plain, entry("Sheriff").key);
        let d = decode_all(&capture(data)).pop().expect("decoded");
        assert_eq!(d.protocol_display_name.as_deref(), Some("KeeLoq (Sheriff)"));
    }

    #[test]
    fn keeloq_real_doorhan_remotes() {
        // Frames of the three real DoorHan remotes in Flipper-ARF's unit-test captures
        // (unit_tests/subghz/doorhan_raw.sub, doorhan.sub, test_random_raw.sub): SIMPLE learning
        // with the DoorHan key (KAT's blob types it NORMAL; corrected in `keeloq_keystore`).
        // 13A43DF0… also passes Guard_RF-311A's key under simple learning, an earlier entry: only the
        // reference selector with DoorHan's real type names it correctly.
        for (data, serial, btn, cnt) in [
            (0x5AB6_695A_8163_6CB4u64, 0x0D36_C681u32, 0x2u8, 0x04BDu16),
            (0x1C81_ADD4_8163_6CB4, 0x0D36_C681, 0x2, 0x04BE),
            (0x13A4_3DF0_8163_6CB4, 0x0D36_C681, 0x2, 0x04C0),
            (0x4850_F072_3378_9514, 0x08A9_1ECC, 0x2, 0x0004),
            (0xF49B_5812_8329_BE21, 0x047D_94C1, 0x8, 0x0490),
            (0x050D_1895_8329_BE28, 0x047D_94C1, 0x1, 0x0491),
        ] {
            let (f, h) = fix_hop(data);
            assert!(select_key(f, h, keeloq_keystore()).is_some(), "{data:016X}");
            let d = decode_all(&capture(data)).pop().expect("decoded");
            assert_eq!(d.protocol_display_name.as_deref(), Some("KeeLoq (DoorHan)"));
            assert_eq!((d.serial, d.button, d.counter), (Some(serial), Some(btn), Some(cnt)));
            let next = decode_all(&KeeloqDecoder::new().encode(&d, 0x2).unwrap()).pop().unwrap();
            assert_eq!(next.protocol_display_name.as_deref(), Some("KeeLoq (DoorHan)"));
            assert_eq!(next.counter, Some(cnt + 1));
        }
    }

    #[test]
    fn keeloq_special_manufacturers() {
        // AN-Motors: hop = cnt8 cnt8 btn 0x404, no key.
        let fix = (0x4u32 << 28) | 0x0055_1234;
        let hop: u32 = (0x37 << 24) | (0x37 << 16) | (0x4 << 12) | 0x404;
        let data = reverse_key((u64::from(fix) << 32) | u64::from(hop), 64);
        let d = decode_all(&capture(data)).pop().unwrap();
        assert_eq!(d.protocol_display_name.as_deref(), Some("KeeLoq (AN-Motors)"));
        assert_eq!(d.counter, Some(0x3737));
        // HCS101: hop = cnt16 btn 000.
        let hop: u32 = (0x1234 << 16) | (0x4 << 12);
        let data = reverse_key((u64::from(fix) << 32) | u64::from(hop), 64);
        let d = decode_all(&capture(data)).pop().unwrap();
        assert_eq!(d.protocol_display_name.as_deref(), Some("KeeLoq (HCS101)"));
        assert_eq!(d.counter, Some(0x1234));
        // Pecinin: no button in fix, 12-bit serial check, button from the plaintext.
        let serial: u32 = 0x0000_0ABC;
        let plain = (0x8u32 << 28) | (0xABC << 16) | 0x0010;
        let fixp = serial;
        let data = reverse_key(
            (u64::from(fixp) << 32) | u64::from(keeloq_encrypt(plain, entry("Pecinin").key)),
            64,
        );
        let d = decode_all(&capture(data)).pop().unwrap();
        assert_eq!(d.protocol_display_name.as_deref(), Some("KeeLoq (Pecinin)"));
        assert_eq!(d.button, Some(0x8));
        // Centurion: NORMAL with discriminator 0x1CE.
        let serial = 0x0765_4321;
        let fixc = (0x1u32 << 28) | serial;
        let plain = (0x1u32 << 28) | (0x1CE << 16) | 0x0099;
        let man = keeloq_normal_learning(fixc, entry("Centurion").key);
        let d = decode_all(&capture(frame(0x1, serial, plain, man))).pop().unwrap();
        assert_eq!(d.protocol_display_name.as_deref(), Some("KeeLoq (Centurion)"));
        assert_eq!(d.counter, Some(0x0099));
    }

    #[test]
    fn keeloq_encode_roundtrip_per_learning_type() {
        let dec = KeeloqDecoder::new();
        let serial = 0x0ABC_DE12;
        for name in ["Sheriff", "DoorHan", "Beninca", "KGB/Subaru", "BFT", "Leopard", "Centurion", "Sommer", "Came_Space", "NICE_Smilo", "Monarch", "Dea_Mio", "Aprimatic"] {
            let e = entry(name);
            let fix = (0x2u32 << 28) | serial;
            let disc = discriminator(name, serial, 0);
            let plain = (0x2u32 << 28) | (disc << 16) | 0x0100;
            let man = match e.learning {
                lt::SIMPLE => e.key,
                lt::NORMAL => keeloq_normal_learning(fix, e.key),
                lt::SECURE => keeloq_secure_learning(fix, serial, e.key),
                lt::MAGIC_XOR_TYPE_1 => keeloq_magic_xor_type1_learning(fix, e.key),
                lt::MAGIC_SERIAL_TYPE_1 => keeloq_magic_serial_type1_learning(fix, e.key),
                _ => e.key, // UNKNOWN → simple
            };
            let captured = decode_all(&capture(frame(0x2, serial, plain, man)));
            let Some(captured) = captured.into_iter().next() else {
                panic!("{name}: capture not decoded");
            };
            assert_eq!(
                captured.protocol_display_name.as_deref(),
                Some(format!("KeeLoq ({name})").as_str())
            );
            for button in [0x1u8, 0x4, 0x8] {
                let signal = dec.encode(&captured, button).unwrap();
                let gap = if name == "Sommer" { 11_600 } else { 16_000 };
                assert_eq!(signal.last().map(|l| l.duration_us), Some(gap));
                let decodes = decode_all(&signal);
                assert_eq!(decodes.len(), TX_REPEAT, "{name}");
                let d = &decodes[0];
                assert_eq!(
                    d.protocol_display_name.as_deref(),
                    Some(format!("KeeLoq ({name})").as_str()),
                    "{name} button {button:#x}"
                );
                assert_eq!(d.serial, Some(serial));
                assert_eq!(d.button, Some(button));
                assert_eq!(d.counter, Some(0x0101), "{name}");
            }
        }
    }

    #[test]
    fn keeloq_encode_special_paths() {
        let dec = KeeloqDecoder::new();
        // AN-Motors / HCS101 rebuild their plaintext hop with counter + 1.
        let fix = (0x4u32 << 28) | 0x0055_1234;
        let hop: u32 = (0x37 << 24) | (0x37 << 16) | (0x4 << 12) | 0x404;
        let captured = decode_all(&capture(reverse_key((u64::from(fix) << 32) | u64::from(hop), 64)))
            .pop()
            .unwrap();
        let d = decode_all(&dec.encode(&captured, 0x2).unwrap()).pop().unwrap();
        assert_eq!(d.protocol_display_name.as_deref(), Some("KeeLoq (AN-Motors)"));
        assert_eq!(d.button, Some(0x2));
        // C: cnt = hop >> 16 = 0x3737 → 0x3738 → hop bytes 0x38 0x38.
        assert_eq!(d.counter, Some(0x3838));
        // MAGIC_SERIAL_TYPE_2/3 have no encode case: the captured frame is sent unchanged.
        let serial = 0x0ABC_DE12;
        let fix = (0x2u32 << 28) | serial;
        let plain = (0x2u32 << 28) | ((serial & 0x3FF) << 16) | 0x0777;
        let man = keeloq_magic_serial_type2_learning(fix, entry("Magic_2").key);
        let data = frame(0x2, serial, plain, man);
        let captured = decode_all(&capture(data)).pop().unwrap();
        let d = decode_all(&dec.encode(&captured, 0x1).unwrap()).pop().unwrap();
        assert_eq!(d.data, data);
        // BFT programming mode: button 0xF sends the seed as hop, counter untouched.
        let e = entry("BFT");
        let plain = (0x2u32 << 28) | ((serial & 0x3FF) << 16) | 0x0100;
        let data = frame(0x2, serial, plain, keeloq_secure_learning(fix, serial, e.key));
        let captured = decode_all(&capture(data)).pop().unwrap();
        let signal = dec.encode(&captured, 0xF).unwrap();
        let mut once = Vec::new();
        upload_once(
            reverse_key((u64::from((0xFu32 << 28) | serial) << 32) | u64::from(serial), 64),
            "BFT",
            TE_SHORT,
            &mut once,
        );
        let pulses = |s: &[LevelDuration]| -> Vec<(bool, u32)> {
            s.iter().map(|l| (l.level, l.duration_us)).collect()
        };
        assert_eq!(pulses(&signal[..once.len()]), pulses(&once));
        // Counter wraps to 0 past 0xFFFF.
        let plain = (0x2u32 << 28) | ((serial & 0x3FF) << 16) | 0xFFFF;
        let data = frame(0x2, serial, plain, entry("Sheriff").key);
        let captured = decode_all(&capture(data)).pop().unwrap();
        let d = decode_all(&dec.encode(&captured, 0x2).unwrap()).pop().unwrap();
        assert_eq!(d.counter, Some(0));
    }

    #[test]
    fn keeloq_superrollo_preamble_is_accepted() {
        // ARF's asymmetric preamble (Te HIGH / 2Te LOW) and 10·Te HIGH sync keep the header count.
        let mut s = Vec::new();
        for _ in 0..9 {
            s.push(LevelDuration::new(true, 450));
            s.push(LevelDuration::new(false, 900));
        }
        s.push(LevelDuration::new(true, 4500));
        s.push(LevelDuration::new(false, 4500));
        let serial = 0x0ABC_DE12;
        let plain = (0x2u32 << 28) | ((serial & 0x3FF) << 16) | 0x0777;
        let data = frame(0x2, serial, plain, entry("Sheriff").key);
        let mut body = capture(data);
        s.extend(body.drain(24..)); // bits + status + end
        let d = decode_all(&s);
        assert_eq!(d.len(), 1);
        assert_eq!(d[0].protocol_display_name.as_deref(), Some("KeeLoq (Sheriff)"));
    }

    #[test]
    fn keeloq_button_positions() {
        assert_eq!(keeloq_button_position("DoorHan", 0x2), Some(1));
        assert_eq!(keeloq_button_position("DoorHan", 0x1), Some(4));
        assert_eq!(keeloq_button_position("Sheriff", 0x2), None);
    }

    /// C-oracle driver for the KeeLoq family (Scher-Khan, Star Line, KeeLoq, Sheriff CFM); ignored.
    /// - `KAT_ORACLE_EXPORT=<file>`: write `DEC <sk|sl|kl|cfm> <n> <level dur>...` commands for every
    ///   `IMPORTS/*/*.sub` plus `KAT_ORACLE_EXTRA` (`;`-separated paths), both polarities.
    /// - `KAT_ORACLE_IN=<cmds> KAT_ORACLE_OUT=<file>`: run `DEC` and
    ///   `ENC <proto> <bits> <hexkey> <c_btn> <kat_btn>` commands through KAT, printing the same line
    ///   format as the C harness (`D ...` per decode + `END`; `U <n> <level dur>...` per encode).
    #[test]
    #[ignore]
    fn c_oracle_driver() {
        use crate::export::flipper::import_sub_raw;
        use crate::protocols::{
            scher_khan::ScherKhanDecoder, sheriff_cfm::SheriffCfmDecoder, star_line::StarLineDecoder,
        };
        use std::fmt::Write as _;
        use std::path::{Path, PathBuf};
        if let Ok(out) = std::env::var("KAT_ORACLE_EXPORT") {
            let mut subs: Vec<PathBuf> = Vec::new();
            for make in std::fs::read_dir("IMPORTS").unwrap().flatten() {
                if make.path().is_dir() {
                    for f in std::fs::read_dir(make.path()).unwrap().flatten() {
                        if f.path().extension().is_some_and(|e| e == "sub") {
                            subs.push(f.path());
                        }
                    }
                }
            }
            subs.sort();
            for extra in std::env::var("KAT_ORACLE_EXTRA").unwrap_or_default().split(';') {
                if !extra.is_empty() {
                    subs.push(PathBuf::from(extra));
                }
            }
            let mut text = String::new();
            let mut index = String::new();
            for path in &subs {
                let Ok((_, pairs)) = import_sub_raw(Path::new(path)) else { continue };
                for proto in ["sk", "sl", "kl", "cfm"] {
                    for invert in [false, true] {
                        write!(text, "DEC {proto} {}", pairs.len()).unwrap();
                        for p in &pairs {
                            write!(text, " {} {}", u8::from(p.level != invert), p.duration_us).unwrap();
                        }
                        text.push('\n');
                        writeln!(index, "{} {proto} {invert}", path.display()).unwrap();
                    }
                }
            }
            std::fs::write(&out, text).unwrap();
            std::fs::write(format!("{out}.index"), index).unwrap();
            return;
        }
        let cmds = std::fs::read_to_string(std::env::var("KAT_ORACLE_IN").unwrap()).unwrap();
        let mut out = String::new();
        let decoder = |proto: &str| -> Box<dyn ProtocolDecoder> {
            match proto {
                "sk" => Box::new(ScherKhanDecoder::new()),
                "sl" => Box::new(StarLineDecoder::new()),
                "kl" => Box::new(KeeloqDecoder::new()),
                _ => Box::new(SheriffCfmDecoder::new()),
            }
        };
        let suffix = |name: &Option<String>, open: char, close: char| -> String {
            name.as_deref()
                .and_then(|n| n.split_once(open))
                .map(|(_, v)| v.strip_suffix(close).unwrap_or(v).to_string())
                .unwrap_or_else(|| "Unknown".to_string())
        };
        for line in cmds.lines() {
            let mut it = line.split_whitespace();
            let (Some(cmd), Some(proto)) = (it.next(), it.next()) else { continue };
            match cmd {
                "DEC" => {
                    let n: usize = it.next().unwrap().parse().unwrap();
                    let mut dec = decoder(proto);
                    for _ in 0..n {
                        let l: u8 = it.next().unwrap().parse().unwrap();
                        let d: u32 = it.next().unwrap().parse().unwrap();
                        let Some(s) = dec.feed(l != 0, d) else { continue };
                        dec.reset();
                        let (serial, btn, cnt) = (
                            s.serial.unwrap_or(0),
                            s.button.unwrap_or(0),
                            s.counter.unwrap_or(0),
                        );
                        let bits = s.data_count_bit;
                        match proto {
                            "sk" => {
                                let data = if bits <= 64 || bits > 128 {
                                    s.data
                                } else {
                                    (((s.data as u128) << (bits - 64)) | s.extra.unwrap() as u128)
                                        as u64
                                };
                                let name = suffix(&s.protocol_display_name, '[', ']');
                                writeln!(out, "D sk {bits} {data:016X} {serial:X} {btn:X} {cnt:X} {name}")
                                    .unwrap();
                            }
                            "sl" => {
                                let mf = suffix(&s.protocol_display_name, '[', ']');
                                writeln!(out, "D sl {bits} {:016X} {serial:X} {btn:X} {cnt:X} {mf}", s.data)
                                    .unwrap();
                            }
                            "kl" => {
                                let (fix, hop) = fix_hop(s.data);
                                let seed = match identify(fix, hop) {
                                    Some(Ident::Key(m)) => m.seed,
                                    _ => 0,
                                };
                                let mf = suffix(&s.protocol_display_name, '(', ')');
                                writeln!(
                                    out,
                                    "D kl {bits} {:016X} {serial:X} {btn:X} {cnt:X} {seed:X} {mf}",
                                    s.data
                                )
                                .unwrap();
                            }
                            _ => {
                                writeln!(out, "D cfm {bits} {:016X} {serial:X} {btn:X} {cnt:X}", s.data)
                                    .unwrap();
                            }
                        }
                    }
                    out.push_str("END\n");
                }
                "ENC" => {
                    let bits: usize = it.next().unwrap().parse().unwrap();
                    let key = u64::from_str_radix(it.next().unwrap(), 16).unwrap();
                    let _c_btn = it.next();
                    let kat_btn: i32 = it.next().unwrap().parse().unwrap();
                    let sig = DecodedSignal {
                        serial: None,
                        button: None,
                        counter: (proto == "sl" || proto == "kl").then_some(0),
                        crc_valid: true,
                        data: key,
                        data_count_bit: bits,
                        encoder_capable: true,
                        extra: None,
                        protocol_display_name: None,
                    };
                    let button = if kat_btn < 0 { 0 } else { kat_btn as u8 };
                    match decoder(proto).encode(&sig, button) {
                        Some(signal) => {
                            write!(out, "U {}", signal.len()).unwrap();
                            for l in &signal {
                                write!(out, " {} {}", u8::from(l.level), l.duration_us).unwrap();
                            }
                            out.push('\n');
                        }
                        None => out.push_str("U NONE\n"),
                    }
                }
                _ => {}
            }
        }
        std::fs::write(std::env::var("KAT_ORACLE_OUT").unwrap(), out).unwrap();
    }

    /// Every other registry encoder's output (generic inputs, several keys, both polarities via the
    /// registry) is never reported as one of the KeeLoq-family protocols, and genuine frames of the
    /// family are reported as themselves.
    #[test]
    fn keeloq_family_registry_interplay() {
        use crate::protocols::ProtocolRegistry;
        use std::panic::{catch_unwind, AssertUnwindSafe};
        const FAMILY: [&str; 4] = ["Scher-Khan", "Star Line", "KeeLoq", "Sheriff CFM"];
        let is_family = |n: &str| FAMILY.iter().any(|f| n.starts_with(f));
        let reg0 = ProtocolRegistry::new();
        let mut x: u64 = 0x2545_F491_4F6C_DD1D;
        for name in reg0.list_protocols() {
            if FAMILY.contains(&name) {
                continue;
            }
            let dec = reg0.get(name).unwrap();
            if !dec.supports_encoding() {
                continue;
            }
            for _ in 0..4 {
                x ^= x << 13;
                x ^= x >> 7;
                x ^= x << 17;
                let src = DecodedSignal {
                    serial: Some((x >> 40) as u32),
                    button: Some(0x01),
                    counter: Some(5),
                    crc_valid: true,
                    data: x,
                    data_count_bit: dec.timing().min_count_bit,
                    encoder_capable: true,
                    extra: Some(x.rotate_left(17)),
                    protocol_display_name: None,
                };
                let Ok(Some(wire)) = catch_unwind(AssertUnwindSafe(|| dec.encode(&src, 0x02)))
                else {
                    continue;
                };
                for f in crate::protocols::test_frequencies(dec.supported_frequencies()) {
                    let mut reg = ProtocolRegistry::new();
                    for (got, _, _) in reg.process_signal_stream(&wire, f) {
                        assert!(!is_family(&got), "{name} at {f} Hz decoded as {got}");
                    }
                }
            }
        }
        // Genuine family frames, re-encoded by their own encoders, are reported as themselves.
        let sig = |data: u64, bits: usize| DecodedSignal {
            serial: None,
            button: None,
            counter: Some(0),
            crc_valid: true,
            data,
            data_count_bit: bits,
            encoder_capable: true,
            extra: None,
            protocol_display_name: None,
        };
        let sl = entry("SL_A2-A4");
        let sl_fix: u32 = (0x01 << 24) | 0x0012_3456;
        let sl_hop = keeloq_encrypt((0x01 << 24) | (0x56 << 16) | 0x0010, sl.key);
        let zx750 = entry("ZX-730-750-1055").key;
        let cfm_fix: u32 = 0x5A3C_10C5;
        let mut cfm_hop = keeloq_encrypt((0x20 << 24) | ((cfm_fix & 0x3FF) << 16) | 0x0042, zx750)
            .to_le_bytes();
        cfm_hop[0] = cfm_hop[0].rotate_left(4);
        cfm_hop[2] = cfm_hop[2].rotate_left(4);
        let cases: [(&str, u64, usize, &str); 4] = [
            ("Scher-Khan", 0x0E8C1619E830C, 51, "Scher-Khan ["),
            (
                "Star Line",
                reverse_key((u64::from(sl_fix) << 32) | u64::from(sl_hop), 64),
                64,
                "Star Line [SL_A2-A4]",
            ),
            ("KeeLoq", 0x5AB6_695A_8163_6CB4, 64, "KeeLoq (DoorHan)"),
            (
                "Sheriff CFM",
                (u64::from(cfm_fix) << 32) | u64::from(u32::from_le_bytes(cfm_hop)),
                64,
                "Sheriff CFM",
            ),
        ];
        for (name, data, bits, expect) in cases {
            let dec = reg0.get(name).unwrap();
            let wire = dec.encode(&sig(data, bits), 0x02).expect(name);
            for f in crate::protocols::test_frequencies(dec.supported_frequencies()) {
                let mut reg = ProtocolRegistry::new();
                let got = reg.process_signal_stream(&wire, f);
                assert!(!got.is_empty(), "{name} at {f} Hz: nothing decoded");
                for (n, _, _) in &got {
                    assert!(n.starts_with(expect), "{name} at {f} Hz decoded as {n}");
                }
            }
        }
    }

    #[test]
    fn keeloq_random_frame_acceptance_rate() {
        let _lock = KEYSTORE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // ARF's identification (AN-Motors, HCS101, then one learning attempt per typed keystore
        // entry, eight per UNKNOWN entry) identifies ~6% of random 64-bit frames with KAT's keystore.
        let mut s = 0x9E37_79B9_7F4A_7C15u64;
        let mut accepted = 0;
        let n = 600;
        for _ in 0..n {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let (fix, hop) = fix_hop(s);
            // Every frame is reported; unidentified ones as "KeeLoq (Unknown)".
            let d = KeeloqDecoder::parse_data(s).expect("every frame is reported");
            let identified = d.protocol_display_name.as_deref() != Some(UNKNOWN_NAME);
            assert_eq!(identified, identify(fix, hop).is_some());
            assert_eq!((d.encoder_capable, d.counter.is_some()), (identified, identified));
            if !identified {
                assert_eq!((d.serial, d.button), (Some(fix & 0x0FFF_FFFF), Some((fix >> 28) as u8)));
            }
            if identified {
                accepted += 1;
            }
        }
        assert!(accepted * 100 >= n * 2, "accepted {accepted}/{n}");
        assert!(accepted * 100 < n * 10, "accepted {accepted}/{n}");
    }

    /// Key files (Flipper-ARF `subghz_protocol_decoder_keeloq_serialize` / `_deserialize` +
    /// `check_remote_controller` with the saved manufacturer).
    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::{assert_round_trip, key};
        let dec = KeeloqDecoder::new();
        // Every learning type's decode round-trips; DoorHan's real remote with the C's exact lines.
        let serial = 0x0ABC_DE12;
        let fix = (0x2u32 << 28) | serial;
        let plain = (0x2u32 << 28) | ((serial & 0x3FF) << 16) | 0x0777;
        for (name, man) in [
            ("Sheriff", entry("Sheriff").key),
            ("Mongoose", keeloq_normal_learning(fix, entry("Mongoose").key)),
            ("Beninca", keeloq_magic_xor_type1_learning(fix, entry("Beninca").key)),
            ("KGB/Subaru", keeloq_magic_serial_type1_learning(fix, entry("KGB/Subaru").key)),
            ("BFT", keeloq_secure_learning(fix, serial, entry("BFT").key)),
            ("Leopard", mirrored_key(entry("Leopard").key)),
        ] {
            let d = decode_all(&capture(frame(0x2, serial, plain, man))).pop().unwrap();
            let text = assert_round_trip(&dec, &d);
            assert!(text.ends_with(&format!("Manufacture: {name}\n")), "{text}");
            // ARF writes Seed for BFT only: here the seed the decrypt used, the serial.
            assert_eq!(text.contains("\nSeed: 0A BC DE 12\n"), name == "BFT", "{text}");
        }
        let doorhan = decode_all(&capture(0x5AB6_695A_8163_6CB4)).pop().unwrap();
        assert!(assert_round_trip(&dec, &doorhan)
            .ends_with("Protocol: KeeLoq\nBit: 64\nKey: 5A B6 69 5A 81 63 6C B4\nManufacture: DoorHan\n"));
        for special in [
            reverse_key((u64::from((0x4u32 << 28) | 0x0055_1234) << 32) | 0x3737_4404, 64), // AN-Motors
            reverse_key((u64::from((0x4u32 << 28) | 0x0055_1234) << 32) | 0x1234_4000, 64), // HCS101
        ] {
            assert_round_trip(&dec, &decode_all(&capture(special)).pop().unwrap());
        }

        // A file Flipper-ARF wrote (its keystore named the manufacturer).
        let arf = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                   Preset: FuriHalSubGhzPresetOok650Async\nProtocol: KeeLoq\nBit: 64\n\
                   Key: 1C 81 AD D4 81 63 6C B4\nManufacture: DoorHan\n";
        let loaded = dec.from_flipper_key(&key(arf)).unwrap();
        assert_eq!(loaded, decode_all(&capture(0x1C81_ADD4_8163_6CB4)).pop().unwrap());
        assert_eq!((loaded.serial, loaded.button, loaded.counter), (Some(0x0D36_C681), Some(0x2), Some(0x04BE)));
        // TX from the loaded key: same remote, counter + 1.
        let tx = decode_all(&dec.encode(&loaded, 0x2).unwrap()).pop().unwrap();
        assert_eq!(tx.protocol_display_name.as_deref(), Some("KeeLoq (DoorHan)"));
        assert_eq!((tx.serial, tx.counter), (Some(0x0D36_C681), Some(0x04BF)));
        // "Unknown" or no Manufacture line: identified like the frame on air.
        assert_eq!(dec.from_flipper_key(&key(&arf.replace("DoorHan", "Unknown"))), Some(loaded.clone()));
        assert_eq!(dec.from_flipper_key(&key(&arf.replace("Manufacture: DoorHan\n", ""))), Some(loaded.clone()));
        // A manufacturer whose key does not validate the frame, or a name KAT's keystore lacks:
        // loaded like a frame on air (the C loads it and shows what it finds), as DoorHan here.
        assert_eq!(dec.from_flipper_key(&key(&arf.replace("DoorHan", "Sheriff"))), Some(loaded.clone()));
        assert_eq!(dec.from_flipper_key(&key(&arf.replace("DoorHan", "No Such Brand"))), Some(loaded.clone()));
        // Bit ≠ 64 is refused.
        assert!(dec.from_flipper_key(&key(&arf.replace("Bit: 64", "Bit: 66"))).is_none());
        // A forced "AN-Motors" (the C does not validate it): shown, but KAT's encoder would
        // identify DoorHan, so it is not offered for TX.
        let forced = dec.from_flipper_key(&key(&arf.replace("DoorHan", "AN-Motors"))).unwrap();
        assert_eq!(forced.protocol_display_name.as_deref(), Some("KeeLoq (AN-Motors)"));
        assert!(!forced.encoder_capable);
        assert_round_trip(&dec, &forced);

        // ARF's "Sommer(fsk476)" fix.
        let e = entry("Sommer");
        let sommer_plain = (0x2u32 << 28) | ((serial & 0x3FF) << 16) | 0x0100;
        let man = match e.learning {
            lt::NORMAL => keeloq_normal_learning(fix, e.key),
            _ => e.key,
        };
        let sommer = decode_all(&capture(frame(0x2, serial, sommer_plain, man))).pop().unwrap();
        assert_eq!(sommer.protocol_display_name.as_deref(), Some("KeeLoq (Sommer)"));
        let mut old = dec.to_flipper_key(&sommer).unwrap();
        old.fields = vec![("Manufacture".into(), "Sommer(fsk476)".into())];
        assert_eq!(dec.from_flipper_key(&old), Some(sommer));

        // BFT with a user-chosen seed (Flipper "Add manually"): the file's Seed decrypts it; the
        // decode keeps it in `extra` (identification on air cannot reproduce it), the encoder
        // transmits with it (the named manufacturer, as ARF), and it is written back with the same
        // Seed.
        let custom_seed = 0x1357_9BDF;
        let bft = frame(0x2, serial, plain, keeloq_secure_learning(fix, custom_seed, entry("BFT").key));
        let text = format!(
            "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
             Preset: FuriHalSubGhzPresetOok650Async\nProtocol: KeeLoq\nBit: 64\nKey: {}\n\
             Seed: 13 57 9B DF\nManufacture: BFT\n",
            bft.to_be_bytes().iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(" ")
        );
        let d = dec.from_flipper_key(&key(&text)).unwrap();
        assert_eq!(d.protocol_display_name.as_deref(), Some("KeeLoq (BFT)"));
        assert_eq!((d.serial, d.button, d.counter), (Some(serial), Some(0x2), Some(0x0777)));
        assert_eq!((d.encoder_capable, d.extra), (true, Some(u64::from(custom_seed))));
        assert!(assert_round_trip(&dec, &d).contains("\nSeed: 13 57 9B DF\nManufacture: BFT\n"));
        // TX: the next code under the same seed (counter + 1), another button too.
        for (button, want) in [(0x2u8, 0x2u32), (0x4, 0x4)] {
            let tx = dec.encode(&d, button).unwrap();
            let frame = decode_all(&tx).pop().unwrap().data;
            let (fix, hop) = fix_hop(frame);
            let plain = keeloq_decrypt(hop, keeloq_secure_learning(fix, custom_seed, entry("BFT").key));
            assert_eq!((fix >> 28, plain >> 28, plain & 0xFFFF), (want, want, 0x0778), "button {button:#x}");
        }
        // Without the name (identification on air) the custom seed is unknown: replayed hop.
        let anonymous = DecodedSignal { protocol_display_name: None, ..d.clone() };
        let tx = decode_all(&dec.encode(&anonymous, 0x2).unwrap()).pop().unwrap();
        assert_eq!(fix_hop(tx.data).1, fix_hop(bft).1);
    }

    /// Serializes the tests that depend on the exact keystore contents with the ones that add
    /// made-up keys (doctor-firmware's learning types 16..19, the discriminator rules; removed again
    /// on drop): `keeloq_common::KEYSTORE_TEST_LOCK`, which the other modules' keystore-dependent
    /// tests take too.
    static KEYSTORE_LOCK: &std::sync::Mutex<()> = &super::super::keeloq_common::KEYSTORE_TEST_LOCK;
    const DOCTOR_KEYS: [&str; 4] = ["T_JcmGen2", "T_Stagnoli", "T_Telcoma", "T_TelcomaLo"];

    struct DoctorKeys(#[allow(dead_code)] std::sync::MutexGuard<'static, ()>);

    impl DoctorKeys {
        fn add() -> Self {
            let guard = KEYSTORE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
            for (name, learning, key) in [
                ("T_JcmGen2", lt::JCM_GEN2, 0x5A3C_96E1_0B4D_7F28),
                ("T_Stagnoli", lt::STAGNOLI, 0x13A7_C85E_29D4_B06F),
                ("T_Telcoma", lt::TELCOMA_TABLE_HI, 0x9E21_47B8_C3D5_0A6F),
                ("T_TelcomaLo", lt::TELCOMA_TABLE_LO, 0x4C8D_E2A1_7B93_F056),
            ] {
                super::super::keeloq_common::add_keeloq_key(MfKey { name: name.into(), learning, key });
            }
            DoctorKeys(guard)
        }
    }

    impl Drop for DoctorKeys {
        fn drop(&mut self) {
            super::super::keeloq_common::remove_keeloq_keys(&DOCTOR_KEYS);
        }
    }

    /// Learning types 16..19: identification, and encode then decode, per type, with made-up keys.
    #[test]
    fn keeloq_doctor_learning_types() {
        let _keys = DoctorKeys::add();
        let dec = KeeloqDecoder::new();
        let table = keeloq_telcoma_table().expect("both Telcoma halves");
        assert_eq!(table, [0x9E21_47B8, 0xC3D5_0A6F, 0x4C8D_E2A1, 0x7B93_F056]);
        for (name, jcm_seed) in [("T_JcmGen2", 0x9Cu32), ("T_Stagnoli", 0), ("T_Telcoma", 0)] {
            let key = entry(name).key;
            // First-match identification: skip the serials another entry also validates (the
            // 256-way JCM Gen2 search claims about one frame in ten).
            let (serial, data) = (0x0A5C_3E17u32..)
                .map(|serial| {
                    let fix = (0x4u32 << 28) | serial;
                    let plain = (0x4u32 << 28) | ((serial & 0x3FF) << 16) | 0x0123;
                    let man = match name {
                        "T_JcmGen2" => keeloq_learning_jcm_gen2(fix, jcm_seed as u8, key),
                        "T_Stagnoli" => keeloq_learning_stagnoli(fix, key),
                        _ => keeloq_learning_telcoma_table(fix, &table),
                    };
                    (serial, frame(0x4, serial, plain, man))
                })
                .find(|&(_, data)| {
                    let (fix, hop) = fix_hop(data);
                    matches!(identify(fix, hop), Some(Ident::Key(m)) if m.entry.name == name)
                })
                .unwrap();
            let src = decode_all(&capture(data)).pop().unwrap();
            assert_eq!(src.protocol_display_name.as_deref(), Some(&*format!("KeeLoq ({name})")), "{name}");
            assert_eq!((src.serial, src.button, src.counter), (Some(serial), Some(0x4), Some(0x0123)), "{name}");
            assert!(src.encoder_capable, "{name}");
            // Encode (counter + 1, another button too), then decode the transmission.
            for button in [0x4u8, 0x2] {
                let tx = decode_all(&dec.encode(&src, button).unwrap());
                let d = tx.last().unwrap();
                // On air an earlier entry may claim the new frame (first match, as the C); the
                // named decode (the key file's Manufacture) is the encoder's own entry.
                let (fix, hop) = fix_hop(d.data);
                let ident = key_file_identify(fix, hop, name, None).expect(name);
                let (s, b, c) = fields(fix, hop, &ident);
                assert_eq!((s, b, c), (serial, button, 0x0124), "{name}");
            }
            // Key file: the Manufacture line names it, and the file decodes back to the same frame.
            let file = dec.to_flipper_key(&src).unwrap();
            assert_eq!(file.field("Manufacture"), Some(name));
            assert_eq!(dec.from_flipper_key(&file), Some(src.clone()), "{name}");
        }
        assert!(can_create("T_JcmGen2") && can_create("T_Stagnoli") && can_create("T_Telcoma"));
        assert!(!can_create("T_TelcomaLo"));
    }

    /// RogueMaster's half rate: the same frame at Te 200 µs (the sync gap locks the rate, at the
    /// encoder's measured Te), reported with its Te in `extra`, sent back at that rate, written to a
    /// key file as `TE: 200` and read back; standard-rate frames keep `extra` None and no TE line.
    #[test]
    fn keeloq_half_rate() {
        let dec = KeeloqDecoder::new();
        let e = entry("DoorHan");
        assert_eq!(e.learning, lt::SIMPLE);
        let serial = 0x0ABC_DE12;
        let plain = (0x2u32 << 28) | ((serial & 0x3FF) << 16) | 0x0100;
        let data = frame(0x2, serial, plain, e.key);
        for te in [200u32, 185, 215] {
            let mut s = Vec::new();
            upload_once(data, "", te, &mut s);
            let got = decode_all(&s);
            assert_eq!(got.len(), 1, "Te {te}");
            let d = &got[0];
            assert_eq!(d.protocol_display_name.as_deref(), Some("KeeLoq (DoorHan)"));
            assert_eq!((d.serial, d.button, d.counter), (Some(serial), Some(0x2), Some(0x0100)));
            assert_eq!((extra_te(d.extra), extra_seed(d.extra)), (TE_FAST, None));
            // Sent back at 200 µs (counter + 1): every pulse halves, the sync gap is 2000 µs.
            let tx = dec.encode(d, 0x2).unwrap();
            assert_eq!((tx[0].level, tx[0].duration_us), (true, 200));
            assert_eq!(tx.iter().find(|p| !p.level && p.duration_us > 1000).map(|p| p.duration_us), Some(2000));
            assert_eq!(tx.last().map(|p| p.duration_us), Some(8000));
            let back = decode_all(&tx);
            assert_eq!(back.len(), TX_REPEAT);
            assert!(back.iter().all(|b| b.counter == Some(0x0101) && extra_te(b.extra) == TE_FAST));
            // Key file: TE after Manufacture; loads back as the same half-rate decode.
            let text = dec.to_flipper_key(d).unwrap().to_text();
            assert!(text.ends_with("Manufacture: DoorHan\nTE: 200\n"), "{text}");
            assert_eq!(dec.from_flipper_key(&super::super::FlipperKey::parse(&text).unwrap()).as_ref(), Some(d));
        }
        // Standard rate: unchanged.
        let std = decode_all(&capture(data)).pop().unwrap();
        assert_eq!(std.extra, None);
        let text = dec.to_flipper_key(&std).unwrap().to_text();
        assert!(!text.contains("TE:"), "{text}");
        // A file's TE other than 200 is the standard rate (RogueMaster's sanitize).
        let other = super::super::FlipperKey::parse(&format!("{text}TE: 300\n")).unwrap();
        assert_eq!(dec.from_flipper_key(&other).unwrap().extra, None);
        // An unidentified half-rate frame keeps its Te too (key recovery may name it later).
        let mut s = Vec::new();
        upload_once(0x0123_4567_89AB_CDEF, "", 200, &mut s);
        let unknown = decode_all(&s).pop().unwrap();
        assert_eq!(unknown.protocol_display_name.as_deref(), Some(UNKNOWN_NAME));
        assert_eq!(extra_te(unknown.extra), TE_FAST);
        // A BFT seed beside the half rate (and a legacy seed-only value) both read back.
        assert_eq!(extra_seed(make_extra(Some(0x0ABC_DE12), TE_FAST)), Some(0x0ABC_DE12));
        assert_eq!(extra_te(make_extra(Some(0x0ABC_DE12), TE_FAST)), TE_FAST);
        assert_eq!((extra_seed(Some(0x0ABC_DE12)), extra_te(Some(0x0ABC_DE12))), (Some(0x0ABC_DE12), TE_SHORT));
    }

    /// Made-up "Phantom_Hiland" (normal learning) and "Erreka" (Erreka learning) keys for
    /// [keeloq_discriminator_rules], removed on drop.
    struct RuleKeys(#[allow(dead_code)] std::sync::MutexGuard<'static, ()>);

    impl RuleKeys {
        fn add() -> Self {
            let guard = KEYSTORE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
            for (name, learning, key) in [
                ("Phantom_Hiland", lt::NORMAL, 0x3C5A_9617_E2D8_4B01),
                ("Erreka", lt::ERREKA, 0x71D2_08AF_5C36_E94B),
            ] {
                super::super::keeloq_common::add_keeloq_key(MfKey { name: name.into(), learning, key });
            }
            RuleKeys(guard)
        }
    }

    impl Drop for RuleKeys {
        fn drop(&mut self) {
            super::super::keeloq_common::remove_keeloq_keys(&["Phantom_Hiland", "Erreka"]);
        }
    }

    /// A frame of `name`'s (normal-learning or simple) key with plaintext discriminator `disc`,
    /// button 2, counter 0x0123, at the first serial from `start` that the keystore's first match
    /// names `name` (an earlier entry may claim a frame of another key, as in the C).
    fn named_frame(name: &str, start: u32, disc: impl Fn(u32) -> u32) -> (u32, u64) {
        let e = keeloq_keystore().iter().rev().find(|e| e.name == name).unwrap();
        (start..)
            .map(|serial| {
                let fix = (0x2u32 << 28) | serial;
                let plain = (0x2u32 << 28) | (disc(serial) << 16) | 0x0123;
                let man = match e.learning {
                    lt::NORMAL => keeloq_normal_learning(fix, e.key),
                    _ => e.key,
                };
                (serial, frame(0x2, serial, plain, man))
            })
            .find(|&(_, data)| {
                let (fix, hop) = fix_hop(data);
                matches!(identify(fix, hop), Some(Ident::Key(m)) if m.entry.name == name)
            })
            .unwrap()
    }

    /// The plaintext of `d`'s hop under `name`'s key (normal learning or simple).
    fn plaintext(name: &str, data: u64) -> u32 {
        let e = keeloq_keystore().iter().rev().find(|e| e.name == name).unwrap();
        let (fix, hop) = fix_hop(data);
        match e.learning {
            lt::NORMAL => keeloq_decrypt(hop, keeloq_normal_learning(fix, e.key)),
            _ => keeloq_decrypt(hop, e.key),
        }
    }

    /// doctor-firmware / RogueMaster discriminator rules: Phantom/Hiland (made-up key: the serial's
    /// low 10 bits, top 2 bits per remote, kept as the seed), Dea_Mio (the same with the chip's two
    /// OVR bits on top), Monarch A / B (kept as the seed) and KEY (0x2FF), each identified, re-sent
    /// with its discriminator (counter + 1, another button), and written with its Seed (all but
    /// KEY); an Erreka key file's seed (made-up key).
    #[test]
    fn keeloq_discriminator_rules() {
        let _keys = RuleKeys::add();
        let dec = KeeloqDecoder::new();
        assert_eq!(entry("Monarch").learning, lt::NORMAL);
        assert_eq!(entry("KEY").learning, lt::SIMPLE);
        assert_eq!(entry("Dea_Mio").learning, lt::SIMPLE);
        let cases: [(&str, u32); 11] = [
            ("Phantom_Hiland", 0 << 10),
            ("Phantom_Hiland", 1 << 10),
            ("Phantom_Hiland", 2 << 10),
            ("Phantom_Hiland", 3 << 10),
            ("Dea_Mio", 0 << 10),
            ("Dea_Mio", 1 << 10),
            ("Dea_Mio", 2 << 10),
            ("Dea_Mio", 3 << 10),
            ("Monarch", MONARCH_DISC_A),
            ("Monarch", MONARCH_DISC_B),
            ("KEY", KEY_DISC),
        ];
        for (name, top) in cases {
            let disc = |serial: u32| if matches!(name, "Phantom_Hiland" | "Dea_Mio") { top | (serial & 0x3FF) } else { top };
            let (serial, data) = named_frame(name, 0x0A5C_3E17, disc);
            let want = disc(serial);
            let src = decode_all(&capture(data)).pop().unwrap();
            assert_eq!(src.protocol_display_name.as_deref(), Some(&*format!("KeeLoq ({name})")), "{name} {want:03X}");
            assert_eq!((src.serial, src.button, src.counter), (Some(serial), Some(0x2), Some(0x0123)));
            let tx = decode_all(&dec.encode(&src, 0x4).unwrap());
            let p = plaintext(name, tx[0].data);
            assert_eq!((p >> 28, (p >> 16) & 0xFFF, p & 0xFFFF), (0x4, want, 0x0124), "{name} {want:03X}");
            let text = dec.to_flipper_key(&src).unwrap().to_text();
            let seeded = name != "KEY";
            assert_eq!(text.contains(&format!("Seed: 00 00 {:02X} {:02X}\n", want >> 8, want & 0xFF)), seeded, "{text}");
            assert_eq!(dec.from_flipper_key(&super::super::FlipperKey::parse(&text).unwrap()).as_ref(), Some(&src), "{name}");
        }
        // Monarch: a discriminator that is neither A nor B is not Monarch's (the generic check
        // would have taken 0x200: its low byte is 0).
        let e = entry("Monarch");
        let fix = (0x2u32 << 28) | 0x0A5C_3E17;
        let hop = keeloq_encrypt((0x2u32 << 28) | (0x200 << 16) | 0x0123, keeloq_normal_learning(fix, e.key));
        assert!(select_key(fix, hop, std::slice::from_ref(e)).is_none());
        // Phantom/Hiland: low 10 bits must be the serial's.
        let ph = keeloq_keystore().iter().rev().find(|e| e.name == "Phantom_Hiland").unwrap();
        let fix = (0x2u32 << 28) | 0x0A5C_3E17;
        let hop = keeloq_encrypt((0x2u32 << 28) | (((0x0A5C_3E17 + 1) & 0x3FF) << 16) | 0x0123, keeloq_normal_learning(fix, ph.key));
        assert!(select_key(fix, hop, std::slice::from_ref(ph)).is_none());
        // serial & 0x3FF = 0x217: a seed with those low bits is kept whole, any other gives way.
        assert_eq!(phantom_hiland_disc(0x0A5C_3E17, 0xA17), 0xA17);
        assert_eq!(phantom_hiland_disc(0x0A5C_3E17, 0xA18), 0x217);
        // Dea_Mio: the low 8 bits must be the serial's (a low byte of 0 is not taken for any serial,
        // as the generic check takes it), and bits 9..8 are not looked at.
        let dm = entry("Dea_Mio");
        for (disc, found) in [(0x000, false), (0xC00, false), (0x018, false), (0x117, true), (0xE17, true)] {
            let hop = keeloq_encrypt((0x2u32 << 28) | (disc << 16) | 0x0123, dm.key);
            let m = select_key(fix, hop, std::slice::from_ref(dm));
            assert_eq!(m.map(|m| m.seed), found.then_some(disc), "{disc:03X}");
        }
        // serial & 0x3FF = 0x217. A seed with the serial's low byte is kept whole; no seed, or one
        // of another serial, gives the form the other firmwares send (both OVR bits set).
        assert_eq!(dea_mio_disc(0x0A5C_3E17, 0x217), 0x217);
        assert_eq!(dea_mio_disc(0x0A5C_3E17, 0x817), 0x817);
        assert_eq!(dea_mio_disc(0x0A5C_3E17, 0x117), 0x117);
        assert_eq!(dea_mio_disc(0x0A5C_3E17, 0), 0xE17);
        assert_eq!(dea_mio_disc(0x0A5C_3E17, 0x218), 0xE17);
        for serial in [0x0A5C_3E17u32, 0x0000_0869, 0x0FFF_F100, 0x0123_42FF] {
            let first = (serial >> 8) & 0xF;
            assert_eq!(dea_mio_disc(serial, 0), ((serial & 0xFF) | ((0xC + first % 4) << 8)) & 0xFFF, "{serial:07X}");
        }
        // Add Manually: Phantom/Hiland, Dea_Mio and Dorcas with their seeds, KEY with its constant.
        for (mf, seed, want) in [
            ("Phantom_Hiland", Some(0x817u32), 0x817u32),
            ("Dea_Mio", Some(0x017), 0x017),
            ("Dea_Mio", Some(0x417), 0x417),
            ("Dea_Mio", Some(0xC18), 0xC17),
            ("Dea_Mio", None, 0xC17),
            ("Monarch", Some(MONARCH_DISC_B), MONARCH_DISC_B),
            ("Monarch", None, MONARCH_DISC_A),
            ("KEY", None, KEY_DISC),
        ] {
            assert!(can_create(mf), "{mf}");
            let file = create_data(0x0A5C_3C17, 0x2, 0x0003, mf, seed).unwrap();
            let d = dec.from_flipper_key(&file).unwrap();
            assert_eq!(d.protocol_display_name.as_deref(), Some(&*format!("KeeLoq ({mf})")), "{mf}");
            let p = plaintext(mf, d.data);
            assert_eq!(((p >> 16) & 0xFFF, p & 0xFFFF), (want, 0x0003), "{mf}");
        }
        // Erreka: the key file's seed decrypts the frame and is kept for the encoder and the file.
        let seed = 0x000B_EEF5;
        let file = create_data(0x0A5C_3E17, 0x2, 0x0010, "Erreka", Some(seed)).unwrap();
        let text = file.to_text();
        assert!(text.contains("Seed: 00 0B EE F5\nManufacture: Erreka\n"), "{text}");
        let d = dec.from_flipper_key(&file).unwrap();
        assert_eq!((d.protocol_display_name.as_deref(), d.counter), (Some("KeeLoq (Erreka)"), Some(0x0010)));
        assert_eq!(extra_seed(d.extra), Some(seed));
        assert!(dec.to_flipper_key(&d).unwrap().to_text().contains("Seed: 00 0B EE F5\n"));
        let tx = dec.encode(&d, 0x2).unwrap();
        let (fix, hop) = fix_hop(decode_all(&tx)[0].data);
        let ek = keeloq_keystore().iter().rev().find(|e| e.name == "Erreka").unwrap();
        let p = keeloq_decrypt(hop, keeloq_learning_erreka(fix, seed, ek.key));
        assert_eq!((p >> 28, p & 0xFFFF), (0x2, 0x0011));
        // FoxFW's Erreka programming mode: button 0xF sends the seed as the hop (as BFT does).
        let tx = dec.encode(&d, 0xF).unwrap();
        let (fix, hop) = fix_hop(decode_all(&tx)[0].data);
        assert_eq!((fix >> 28, hop), (0xF, seed));
        let prog = create_data(0x0A5C_3E17, 0xF, 0x0010, "Erreka", Some(seed)).unwrap();
        assert_eq!(fix_hop(prog.key).1, seed);
    }

    /// `KAT_KEY_VECTORS=<dir> cargo test keeloq_oracle_vectors -- --ignored`: RAW uploads and KAT's
    /// key files for the C oracle, one per supported learning type (with its manufacturer).
    #[test]
    #[ignore]
    fn keeloq_oracle_vectors() {
        let Some(dir) = std::env::var_os("KAT_KEY_VECTORS") else { return };
        let dir = std::path::PathBuf::from(dir);
        let dec = KeeloqDecoder::new();
        let serial = 0x0ABC_DE12;
        let fix = (0x2u32 << 28) | serial;
        let mut n = 0;
        for name in ["DoorHan", "Mongoose", "BFT", "Beninca", "KGB/Subaru", "Leopard", "Centurion", "AN-Motors", "HCS101"] {
            let data = match name {
                "AN-Motors" => reverse_key((u64::from(fix) << 32) | 0x3737_2404, 64),
                "HCS101" => reverse_key((u64::from(fix) << 32) | 0x1234_2000, 64),
                _ => {
                    let e = entry(name);
                    let disc = discriminator(name, serial, 0);
                    let plain = (0x2u32 << 28) | (disc << 16) | 0x0100;
                    let man = match e.learning {
                        lt::SIMPLE => e.key,
                        lt::NORMAL => keeloq_normal_learning(fix, e.key),
                        lt::SECURE => keeloq_secure_learning(fix, serial, e.key),
                        lt::MAGIC_XOR_TYPE_1 => keeloq_magic_xor_type1_learning(fix, e.key),
                        lt::MAGIC_SERIAL_TYPE_1 => keeloq_magic_serial_type1_learning(fix, e.key),
                        _ => e.key,
                    };
                    frame(0x2, serial, plain, man)
                }
            };
            let src = decode_all(&capture(data)).pop().unwrap_or_else(|| panic!("{name}"));
            let tx = dec.encode(&src, 0x1).unwrap();
            let once = &tx[..tx.len() / TX_REPEAT];
            let d = decode_all(once).pop().unwrap_or_else(|| panic!("{name} TX"));
            let key = dec.to_flipper_key(&d).unwrap();
            n += 1;
            super::super::star_line::write_oracle_vector(&dir, &format!("keeloq_{n}"), 433_920_000, "FuriHalSubGhzPresetOok650Async", once, &key);
        }
        // RogueMaster's Dea_Mio: a remote with each form of the two OVR bits, as received (the
        // vector's upload) and as KAT sends its next code (printed, for the oracle's `encode` of
        // the key file).
        let e = entry("Dea_Mio");
        let serial = 0x0ABC_DE17u32;
        for top in 0..4u32 {
            let disc = (top << 10) | (serial & 0x3FF);
            let data = frame(0x2, serial, (0x2u32 << 28) | (disc << 16) | 0x0100, e.key);
            let upload = capture(data);
            let src = decode_all(&upload).pop().unwrap();
            let key = dec.to_flipper_key(&src).unwrap();
            let next = decode_all(&dec.encode(&src, 0x2).unwrap()).remove(0);
            println!("keeloq_dea_mio_{top}: disc {disc:03X}, KAT sends next {:016X}", next.data);
            super::super::star_line::write_oracle_vector(&dir, &format!("keeloq_dea_mio_{top}"), 433_920_000, "FuriHalSubGhzPresetOok650Async", &upload, &key);
        }
    }
}
