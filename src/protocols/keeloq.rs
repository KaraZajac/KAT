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
//! with 0x240 / 0x210 rounds, then plain), ERREKA, PUJOL, SIMPLE_JCM, and UNKNOWN (simple, normal,
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
//! 0 for Beninca/Merlin, 0x1CE Centurion, 0x100 Monarch, Dea_Mio's C19-style form). Hop by the first
//! keystore entry with that name: SIMPLE, NORMAL, SECURE (seed), MAGIC_XOR (serial), MAGIC_SERIAL_1,
//! AERF, ERREKA, PUJOL, SIMPLE_JCM (8-bit disc), UNKNOWN per matched mode with the key as stored
//! (never the mirrored copy — reference behaviour). MAGIC_SERIAL_2/3 have no encode case in the C:
//! the captured frame is sent unchanged. AN-Motors/HCS101 build their plaintext hop; unidentified frames
//! replay the captured hop under the new fix. Programming mode: button 0xF for BFT sends the seed as
//! hop, Aprimatic 0x1A2B3C4D, Dea_Mio 0 (counter not advanced).
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
    keeloq_learning_aerf, keeloq_learning_erreka, keeloq_learning_pujol,
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

/// Plaintext discriminator (bits 27..16 of the hop plaintext) per manufacturer (`gen_data`).
fn discriminator(mf: &str, serial: u32) -> u32 {
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
        "Monarch" => 0x100,
        "Dea_Mio" => {
            let first = (serial >> 8) & 0xF;
            ((serial & 0xFF) | ((0xC + first % 4) << 8)) & 0xFFF
        }
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
    // Programming mode (C prog_mode, per press): button 0xF for BFT / Aprimatic / Dea_Mio.
    let prog = btn == 0xF && matches!(mf, "BFT" | "Aprimatic" | "Dea_Mio");
    let mut hop: u32 = match (prog, mf) {
        (true, "BFT") => seed,
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
    if hop != 0 || (prog && matches!(mf, "BFT" | "Dea_Mio")) {
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
    let plain = (btn32 << 28) | (discriminator(mf, serial) << 16) | cnt;
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
    let prog = btn == 0xF && matches!(mf, "BFT" | "Aprimatic" | "Dea_Mio");
    let cnt = u32::from(cnt);
    let hop = match (prog, mf) {
        (true, "BFT") => seed_value,
        (true, "Aprimatic") => 0x1A2B_3C4D,
        (true, _) => 0,
        (false, "AN-Motors") => ((cnt & 0xFF) << 24) | ((cnt & 0xFF) << 16) | (btn32 << 12) | 0x404,
        (false, "HCS101") => (cnt << 16) | (btn32 << 12),
        (false, _) => {
            let entry = keeloq_keystore().iter().find(|e| e.name == mf)?;
            encrypt_hop(mf, entry, 0, fix, serial, btn32, cnt, seed_value)
        }
    };
    if hop == 0 && !(prog && matches!(mf, "BFT" | "Dea_Mio")) {
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
    key_file_identify(fix, hop, mf, decoded.extra.map(|e| e as u32))
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

/// One copy of the C upload (`subghz_protocol_encoder_keeloq_encode_to_timings`).
fn upload_once(data: u64, mf: &str, signal: &mut Vec<LevelDuration>) {
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
    let gap = if mf == "Sommer" { TE_SHORT * 29 } else { TE_SHORT * 40 };
    for _ in 0..11 {
        signal.push(LevelDuration::new(true, TE_SHORT));
        signal.push(LevelDuration::new(false, TE_SHORT));
    }
    signal.push(LevelDuration::new(true, TE_SHORT));
    signal.push(LevelDuration::new(false, TE_SHORT * 10));
    for i in (0..MIN_COUNT_BIT).rev() {
        if (data >> i) & 1 == 1 {
            signal.push(LevelDuration::new(true, TE_SHORT));
            signal.push(LevelDuration::new(false, TE_LONG));
        } else {
            signal.push(LevelDuration::new(true, TE_LONG));
            signal.push(LevelDuration::new(false, TE_SHORT));
        }
    }
    // Status bit, then the end pulse and gap.
    signal.push(LevelDuration::new(true, TE_SHORT));
    signal.push(LevelDuration::new(false, TE_LONG));
    signal.push(LevelDuration::new(true, TE_SHORT));
    signal.push(LevelDuration::new(false, gap));
}

pub struct KeeloqDecoder {
    step: Step,
    header_count: u16,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: usize,
}

impl KeeloqDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            header_count: 0,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
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
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                if level && duration_diff!(duration, TE_SHORT) < TE_DELTA {
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
                        || duration_diff!(duration, TE_SHORT * 2) < TE_DELTA)
                {
                    self.step = Step::Reset;
                    return None;
                }
                if self.header_count > 2
                    && duration_diff!(duration, TE_SHORT * 10) < TE_DELTA * 10
                {
                    // Found header
                    self.step = Step::SaveDuration;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                } else {
                    self.step = Step::Reset;
                    self.header_count = 0;
                }
            }
            Step::SaveDuration => {
                if level {
                    self.te_last = duration;
                    self.step = Step::CheckDuration;
                }
            }
            Step::CheckDuration => {
                if !level {
                    if duration >= TE_SHORT * 2 + TE_DELTA {
                        // Found end TX
                        self.step = Step::Reset;
                        if (MIN_COUNT_BIT..=MIN_COUNT_BIT + 2).contains(&self.decode_count_bit) {
                            let result = Self::parse_data(self.decode_data);
                            self.decode_data = 0;
                            self.decode_count_bit = 0;
                            self.header_count = 0;
                            return result;
                        }
                    } else if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA
                        && duration_diff!(duration, TE_LONG) < TE_DELTA * 2
                    {
                        self.add_bit(1);
                        self.step = Step::SaveDuration;
                    } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA * 2
                        && duration_diff!(duration, TE_SHORT) < TE_DELTA
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
        let mut signal = Vec::with_capacity(TX_REPEAT * 160);
        for _ in 0..TX_REPEAT {
            upload_once(data, mf, &mut signal);
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
        let manufacturer = match key.field("Manufacture").map(str::trim) {
            Some("Sommer(fsk476)") => Some("Sommer"),
            other => other,
        };
        match manufacturer {
            None | Some("") | Some("Unknown") => Self::parse_data(data),
            Some(mf) => {
                let (fix, hop) = fix_hop(data);
                let seed = key
                    .hex_bytes_field("Seed")
                    .and_then(|b| <[u8; 4]>::try_from(b.as_slice()).ok())
                    .map(u32::from_be_bytes);
                match key_file_identify(fix, hop, mf, seed) {
                    Some(ident) => Some(key_file_signal(data, fix, hop, &ident)),
                    // A name KAT's keystore lacks (doctor-firmware's "Star", "Wisniowski2" …), or
                    // one whose key does not validate the frame: the C loads the file all the
                    // same and shows what its selector finds ("Unknown" at worst), so KAT
                    // identifies the frame like one on air.
                    None => Self::parse_data(data),
                }
            }
        }
    }

    /// Key files: Flipper-ARF `subghz_protocol_decoder_keeloq_serialize`: Protocol "KeeLoq", Bit
    /// 64, Key, then — for BFT — `Seed` (4 hex bytes, the seed the decrypt used) and `Manufacture`
    /// (the decode's `(<manufacturer>)`, re-checked against the key as the C re-runs its check;
    /// "Unknown" when nothing validates).
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
            Some(mf) => key_file_identify(fix, hop, mf, decoded.extra.map(|e| e as u32)),
            None => identify(fix, hop),
        };
        let mut key = super::FlipperKey::for_protocol("KeeLoq", MIN_COUNT_BIT, decoded.data);
        if let Some(Ident::Key(m)) = &ident {
            if m.entry.name == "BFT" {
                key.push_hex("Seed", u64::from(m.seed), 4);
            }
        }
        key.push("Manufacture", ident.as_ref().map_or("Unknown", Ident::name));
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
fn key_file_signal(data: u64, fix: u32, hop: u32, ident: &Ident) -> DecodedSignal {
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
    let extra = match ident {
        Ident::Key(m) if m.entry.name == "BFT" && !reproduced => Some(u64::from(m.seed)),
        _ => None,
    };
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
        upload_once(data, "", &mut s);
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
            let disc = discriminator(name, serial);
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
                    let disc = discriminator(name, serial);
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
    }
}
