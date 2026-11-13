//! PSA (Peugeot/Citroën) — decoder, encoder and TEA key recovery (brute force).
//!
//! References (current HEADs, compared line by line; see `docs/psa.md`):
//! - ProtoPirate `protocols/psa.c` + `psa_crypto.c` + `psa_crypto_bf.c` (+ `bruteforce_core.c`):
//!   PRIMARY. Unchanged since KAT's v1.3.0 port base (4c95c85) apart from renames/`furi_check`.
//! - Flipper-ARF `lib/subghz/protocols/psa.c` ("PSA GROUP"): same 250/500 µs path, plus an adaptive
//!   AM (OOK) path for 60..200 µs half-bits (ported from ARF's `psa.js`) and `decrypt_file`.
//! - doctor-firmware `lib/subghz/protocols/psa.c`, in two versions, both ported:
//!   - the first one (e7e788a, 2026-10): PP's file with a stricter publish gate, a `>=` preamble
//!     threshold, completion at the 80th bit without the end marker, a working fixed 125/250 µs
//!     path, repeat 3 and an in-app brute-force helper. Its decoder runs as a third machine (its
//!     80th-bit rule and 125/250 µs path); its publish gate is not ported (it drops genuine
//!     frames, see below).
//!   - the current one (98d1fc1, 2026-11), a rewritten decoder: 16 preamble LOWs are enough
//!     (PP wants 71), the half-bit is measured on the preamble and both symbol windows follow it
//!     (± 40 %), a frame is exactly 80 bits closed by a HIGH of 3.2 half-bits or more, and frames
//!     are routed like PP's (the publish gate is gone). It runs as a fourth machine
//!     ([`AdaptiveMachine`]). Its key files say `Bit: 80`; see Key files.
//!
//! What KAT implements (union of the PP, ARF and doctor capabilities):
//! - **Decoder** — ProtoPirate's, Flipper-ARF's and both of doctor-firmware's
//!   `subghz_protocol_decoder_psa_feed` state machines run side by side (see [`PsaDecoder`]).
//!   Every frame the PP machine completes is reported: States 0/1/2 (250/500 µs Manchester,
//!   shared verbatim by PP and ARF) with PP's emission gate (sync nibble
//!   `(key1 >> 48) & 0xF == 0xA`) and PP's decrypt routing (`psa_handle_decoded_frame`: the XOR
//!   fast path runs only when `psa_direct_xor_allowed_by_key2` passes; every other frame is a TEA
//!   frame). PP's own States 3/4 (125/250 µs) feed only LOW pulses (events ShortHigh/LongHigh) to
//!   the Manchester decoder, which can then only emit 0 bits, so key1 is 0, the sync check fails
//!   and that path never reports a frame; it is ported anyway because it holds the machine like
//!   the C does. The doctor machine completes a frame at its 80th bit (`psa_try_complete_at_key2`)
//!   whatever follows, on the 250/500 µs path (`>=` 0x46 preamble LOWs) and on its working
//!   125/250 µs States 3/4; a frame only this rule reads (no end pulse PP's machine takes, a
//!   70-LOW preamble, the 125 µs path) must decrypt like an AM-path frame. From the ARF machine
//!   only its adaptive AM path (States 3/4, 60..200 µs half-bits, frame realignment in
//!   `psa_am_complete`) is used, with PP's routing and a KAT decrypt gate. doctor-firmware's
//!   current machine adds the frames behind a short preamble (16 to 70 LOWs) or on a clock away
//!   from 250 µs, when an end HIGH follows; one only it reads must decrypt too. A real one:
//!   the sixth press of `IMPORTS/GROUPE PSA/Peugeot 307 open.sub`, recorded with 61 preamble LOWs.
//! - **Encoder** — PP `psa_encoder_build_upload` (mode 0x23 XOR + mode 0x36 TEA, buffer[0..1] kept
//!   from the captured key1, buffer[9] from the captured key2), 10 repeats (PP/ARF), counter + 1 (PP
//!   emulate plugin `current_counter++`, ARF/doctor `+ mult`), ARF's PSA button codes, and the
//!   genuine fobs' 900 µs end HIGH (see Deviations).
//! - **Key recovery** — PP `psa_brute_force_run` (BF1 0x23xxxxxx with the scheduled TEA key
//!   derivation, then BF2 0xF3xxxxxx), multithreaded ([`brute_force`]). The references emit a TEA
//!   frame undecrypted and leave the brute force to a manual action ("Brute force" in PP's
//!   receiver info, ARF `subghz_protocol_psa_decrypt_file`, doctor's in-app helper). KAT does the
//!   same: the decoder reports a TEA frame undecrypted (no serial/button/counter, not
//!   encoder-capable) and the signal menu's **Recover key** action runs [`recover_key`] on a
//!   background thread (≤ ~0.5 s on 16 cores in release; progress + cancel via
//!   [`BruteForceControl`]), then fills in serial/button/counter. `encode()` then needs nothing but
//!   key1, key2 and the serial. Frames that must decrypt (the AM path's, either doctor machine's
//!   alone) are reported when the fast path decrypts them, or with `autobf` as [`UNCHECKED_NAME`]
//!   until the key search confirms them (see below).
//!
//! Deviations, each backed by REAL captures in `IMPORTS/` (`GROUPE PSA` holds two fobs: an XOR
//! fob, serial 0x99EB25, and a TEA fob, serial 0x014590 recovered by the brute force):
//! - ARF fires on `decrypted || nibble == 0xA` and XOR-decrypts without PP's key2 filter. That
//!   misdecodes the genuine TEA frame `F47A2EA11A4C7CAF/2927` as serial E3ED81 / button 9 (PP's
//!   brute force recovers the fob's real serial 0x014590 from it), and it fires on 7 frames of the
//!   VAG captures (`Test_55_unlock_and_55_lock_suran.sub`, `Vw pasat B7.sub`) whose sync nibble is
//!   not 0xA. KAT keeps PP's gate and routing for both the FM and the AM path.
//! - doctor's first file published an undecrypted frame only when its key2 high byte failed the
//!   XOR filter, which dropped the genuine TEA frame `F47A25DEC980F0DE/1621` (key2 0x16 passes the
//!   filter, the XOR checksum fails, PP's brute force decrypts it). KAT keeps PP's gate, as
//!   doctor's current file now does.
//! - Buttons: PP's emulate plugin maps Lock/Unlock/Trunk/Panic to 0x1/0x2/0x4/0x8 (its generic
//!   template, shared verbatim with Ford V1); ARF names PSA codes 0 = Lock, 1 = Unlock, 2 = Trunk.
//!   All four genuine frames (two fobs, both modes) carry button 0, a code PP's map does not have,
//!   so KAT uses ARF's codes.
//! - AM path: ARF's completion accepts any 80/88-bit window with the 0xA nibble at one of up to
//!   9 offsets after only 8 preamble cells; KAT's registry resets re-arm it, and on
//!   `IMPORTS/honda/Lock_honda.sub` it then accepts Honda Static chips plus inter-frame noise (the
//!   unmodified ARF psa.c replayed with the registry's reset points emits the same frame, which
//!   decrypts neither by XOR nor by BF1/BF2). KAT emits AM-path frames only when they decrypt.
//! - End marker: the references' encoder ends a frame with a 1000 µs HIGH, which after a final 0 bit
//!   merges into 1250 µs, past PP's and ARF's 801–1199 µs window, so they do not read such a frame
//!   from another Flipper (a saved file sent again) or from KAT ≤ 1.8.3. Genuine fobs end at ~900 µs
//!   (both IMPORTS fobs; ~1140 µs after a final 0 bit). KAT's decoder also takes 1051–1449 µs
//!   (`Machine::is_end_marker`) and its encoder sends 900 µs (`ENC_END_HIGH`); both checked
//!   over the air, Flipper Zero ↔ HackRF (`docs/psa.md`). doctor-firmware's first file needed no
//!   end pulse at all: a fob that ends a frame without one, or with one of another length, reaches
//!   KAT through that machine (since 1.8.5; such a frame must decrypt, see [`PsaDecoder`]). Its
//!   current file takes any HIGH of 3.2 half-bits or more, with no upper limit.
//! - BF2's CRC-16 comparison differs: PP/doctor compare with `dec_v1 & 0xFFFF`, ARF with
//!   `(byte(dec_v1 >> 16) << 8) | byte(dec_v1)`. No genuine BF2 capture exists to settle it; KAT
//!   accepts either (union), which cannot change a PP result except by a ~2^-40 coincidence.
//!
//! Frame (80 bits on air, "128 bit" in the references): key1 (64) + key2 / validation word (16).
//! `DecodedSignal.data` = key1, `extra` = key2 (the references' 64-bit `Key_2`, right-aligned),
//! `data_count_bit` = 128. Byte buffer (`psa_setup_byte_buffer`): b0..b7 = key1 MSB first,
//! b8..b9 = key2. b1's low nibble is the 0xA sync; b0..b1 are not encrypted.
//! - mode 0x23 (XOR): checksum `(sum of nibbles of b2..b7) << 4` must match b8's high nibble; then
//!   serial = b2..b4, counter = b5..b6 (16 bit), crc = b7, button = b8 & 0xF.
//! - mode 0x36 (TEA): b2..b9 = TEA(v0, v1) under a key derived from 0x23000000 | serial (BF1) or
//!   0xF3000000 | serial (BF2); v0 = serial << 8 | button << 4 | counter >> 24, v1 = counter << 8 |
//!   crc8 (28-bit counter; `DecodedSignal.counter` shows its low 16 bits). The serial is unknown
//!   until the 2^24-per-range brute force finds it; afterwards everything is O(1) from
//!   (key1, key2, serial) ([`tea_decrypt_with_serial`]), which is all `encode()` needs.
//!
//! **Key files** (ProtoPirate `subghz_protocol_decoder_psa_serialize` / `_deserialize`):
//! `Protocol: PSA`, `Bit: 128`, `Key` (key1), `Key_2` (key2, 8 bytes), `ValidationField` (key2's
//! low 16 bits), and for a decrypted frame `Serial`, `Btn`, `Cnt`, `Type` (35 = 0x23 XOR, 54 =
//! 0x36 TEA), `CRC`, `Seed` (= serial), decimal — the lines PP's brute force patches into a TEA
//! file. Loading runs the decoder's fast path on Key/Key_2; a TEA frame whose file carries
//! `Type` 0x36 + `Serial` is reported recovered when that serial's key decrypts it (O(1), so a
//! recovered key survives save/load and transmits; PP's decoder re-defers it to its brute force,
//! doctor-firmware and Flipper-ARF trust the lines). Flipper-ARF's "PSA GROUP" / "PSA OLD" files
//! (hex-byte fields, no ValidationField) load too, and so do doctor-firmware's current files
//! (`Bit: 80`, and for mode 0x23 a `Seed` of key2's low 12 bits; KAT reads neither line, and
//! doctor-firmware's and ProtoPirate's loaders take KAT's `Bit: 128` file).
//! Key/Key_2 are 16-digit words, as ProtoPirate writes
//! them (its saved-signal match compares the Key text; every reference's parser also reads spaced
//! bytes); the preset is the full Flipper name (PP writes the short "AM650"). Captures without key2
//! (retired PSA2 decoder) cannot be written.

use super::common::{ManchesterEvent, ManchesterState};
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use super::info::prelude::*;

/// Metadata (see `info.rs`): ProtoPirate "[AM650+F3]". PSA2 / "PSA OLD" (Flipper-ARF's
/// unregistered psa2.c) were retired into PSA; "PSA GROUP" is ARF's display name.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "PSA",
    decoder: "PSA",
    aliases: &["PSA2", "PSA OLD", "PSA GROUP"],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::Both,
    encryption: "TEA/XOR",
    make: "Peugeot/Citroen",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&AM650, &F3],
};

/// Flipper-ARF psa_button_name (the genuine IMPORTS frames carry 0).
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    Some(match code {
        0x0 => "Lock",
        0x1 => "Unlock",
        0x2 => "Trunk",
        _ => "Other",
    })
}

// ---------------------------------------------------------------------------------------------
// Constants (psa.c)
// ---------------------------------------------------------------------------------------------

const TE_SHORT: u32 = 250;
const TE_LONG: u32 = 500;
const TE_DELTA: u32 = 100;
const MIN_COUNT_BIT: usize = 128;

const TE_SHORT_125: u32 = 0x7D;
const TE_LONG_250: u32 = 0xFA;
const TE_LONG_300: u32 = 0x12C;
const TE_END_1000: u32 = 1000;
const TE_END_500: u32 = 500;
/// KAT: a LOW this long after doctor's 80th bit is no final 1 bit's LOW half (at most ~350 µs on
/// the 250 µs path): the frame ended without an end pulse.
const DOCTOR_PENDING_LOW_US: u32 = 1200;
const TOLERANCE_49: u32 = 0x31;
const TOLERANCE_50: u32 = 0x32;
const TOLERANCE_99: u32 = 99;
const TOLERANCE_100: u32 = 100;
const PATTERN_THRESHOLD_1: u16 = 0x46;
const PATTERN_THRESHOLD_2: u16 = 0x45;
const MAX_BITS: u8 = 0x79;
const KEY1_BITS: u8 = 0x40;
const KEY2_BITS: u8 = 0x50;

/// The PSA sync nibble: low nibble of buffer[1], i.e. `(key1_high >> 16) & 0xF`.
const SYNC_NIBBLE: u32 = 0xA;

// ARF adaptive AM (OOK) tuning (`PSA_AM_*`).
const AM_PRE_MIN: u32 = 60;
const AM_PRE_MAX: u32 = 200;
const AM_PRE_THRESHOLD: u16 = 8;
const AM_PRE_GLITCH_DECAY: u16 = 2;
const AM_PRE_MAX_GLITCH: u8 = 3;
const AM_RESYNC_BITS: u8 = 8;
const AM_BITS_CAP: usize = 96;

// Encoder (psa_encoder_build_upload): 80 preamble pairs at 250 µs, 500 µs sync, end HIGH, 1000 µs LOW.
const ENC_TE: u32 = TE_LONG_250;
/// The end HIGH KAT sends: the genuine fobs' (`IMPORTS/GROUPE PSA`, both fobs: medians 893–909 µs
/// after a final 1 bit, ~1140 µs merged with a final 0 bit's HIGH half). The references send
/// 1000 µs, which a final 0 bit stretches to 1250 µs, past their own decoders' 801–1199 µs end
/// window: no reference decodes such a frame, though a fob's same frame decodes.
const ENC_END_HIGH: u32 = 900;
const ENC_PREAMBLE_PAIRS: usize = 80;
/// `instance->encoder.repeat = 10` (ProtoPirate and Flipper-ARF; doctor-firmware sends 3).
const ENC_REPEAT: usize = 10;
/// Entries per upload copy: 160 preamble + 3 sync + 128 key1 + 32 key2 + 2 end
/// (`PSA_UPLOAD_CAPACITY` = 325).
const ENC_UPLOAD_LEN: usize = 325;

const MODE_XOR: u8 = 0x23;
const MODE_TEA: u8 = 0x36;

// TEA (psa_crypto.c / psa_crypto.h)
const TEA_DELTA: u32 = 0x9E37_79B9;
const TEA_ROUNDS: usize = 32;
const BF1_CONST_U4: u32 = 0x0E0F_5C41;
const BF1_CONST_U5: u32 = 0x0F5C_4123;
const BF1_START: u32 = 0x2300_0000;
const BF2_START: u32 = 0xF300_0000;
/// Candidates per brute-force range (`PSA_CRYPTO_BF*_END - PSA_CRYPTO_BF*_START`).
const BF_RANGE: u32 = 1 << 24;
const BF1_KEY_SCHEDULE: [u32; 4] = [0x4A43_4915, 0xD674_3C2B, 0x1F29_D308, 0xE6B7_9A64];
const BF2_KEY_SCHEDULE: [u32; 4] = [0x4039_C240, 0xEDA9_2CAB, 0x4306_C02A, 0x0219_2A04];

// ---------------------------------------------------------------------------------------------
// Crypto primitives (psa_crypto.c, psa.c)
// ---------------------------------------------------------------------------------------------

/// PSA's TEA variant (`psa_crypto_tea_encrypt`): key word `sum & 3`, then `(sum >> 11) & 3`.
fn tea_encrypt(v0: &mut u32, v1: &mut u32, key: &[u32; 4]) {
    let (mut a, mut b) = (*v0, *v1);
    let mut sum: u32 = 0;
    for _ in 0..TEA_ROUNDS {
        let t = key[(sum & 3) as usize].wrapping_add(sum);
        sum = sum.wrapping_add(TEA_DELTA);
        a = a.wrapping_add(t ^ ((b >> 5) ^ (b << 4)).wrapping_add(b));
        let t = key[((sum >> 11) & 3) as usize].wrapping_add(sum);
        b = b.wrapping_add(t ^ ((a >> 5) ^ (a << 4)).wrapping_add(a));
    }
    *v0 = a;
    *v1 = b;
}

/// Inverse of [`tea_encrypt`] (`bruteforce_tea_decrypt` / ARF `psa_tea_decrypt`).
fn tea_decrypt(v0: &mut u32, v1: &mut u32, key: &[u32; 4]) {
    let (mut a, mut b) = (*v0, *v1);
    let mut sum = TEA_DELTA.wrapping_mul(TEA_ROUNDS as u32);
    for _ in 0..TEA_ROUNDS {
        let t = key[((sum >> 11) & 3) as usize].wrapping_add(sum);
        sum = sum.wrapping_sub(TEA_DELTA);
        b = b.wrapping_sub(t ^ ((a >> 5) ^ (a << 4)).wrapping_add(a));
        let t = key[(sum & 3) as usize].wrapping_add(sum);
        a = a.wrapping_sub(t ^ ((b >> 5) ^ (b << 4)).wrapping_add(b));
    }
    *v0 = a;
    *v1 = b;
}

/// Per-round `key + sum` words for a fixed key (`PsaTeaSchedule` / `bruteforce_tea_build_schedule`).
struct TeaSchedule {
    s0: [u32; TEA_ROUNDS],
    s1: [u32; TEA_ROUNDS],
}

impl TeaSchedule {
    fn new(key: &[u32; 4]) -> Self {
        let mut s = TeaSchedule { s0: [0; TEA_ROUNDS], s1: [0; TEA_ROUNDS] };
        for i in 0..TEA_ROUNDS {
            let sum0 = TEA_DELTA.wrapping_mul(i as u32);
            let sum1 = TEA_DELTA.wrapping_mul(i as u32 + 1);
            s.s0[i] = key[(sum0 & 3) as usize].wrapping_add(sum0);
            s.s1[i] = key[((sum1 >> 11) & 3) as usize].wrapping_add(sum1);
        }
        s
    }

    /// `bruteforce_tea_encrypt_with_schedule` — identical to [`tea_encrypt`] with this key.
    #[inline]
    fn encrypt(&self, v0: &mut u32, v1: &mut u32) {
        let (mut a, mut b) = (*v0, *v1);
        for i in 0..TEA_ROUNDS {
            a = a.wrapping_add(self.s0[i] ^ ((b >> 5) ^ (b << 4)).wrapping_add(b));
            b = b.wrapping_add(self.s1[i] ^ ((a >> 5) ^ (a << 4)).wrapping_add(a));
        }
        *v0 = a;
        *v1 = b;
    }
}

/// BF1 working key for a key counter (two scheduled TEA encrypts, `psa_brute_force_run`).
#[inline]
fn bf1_working_key(sched: &TeaSchedule, counter: u32) -> [u32; 4] {
    let (mut wk2, mut wk3) = (BF1_CONST_U4, counter);
    sched.encrypt(&mut wk2, &mut wk3);
    let (mut wk0, mut wk1) = ((counter << 8) | 0x0E, BF1_CONST_U5);
    sched.encrypt(&mut wk0, &mut wk1);
    [wk0, wk1, wk2, wk3]
}

/// BF2 working key for a key counter.
#[inline]
fn bf2_working_key(counter: u32) -> [u32; 4] {
    [
        BF2_KEY_SCHEDULE[0] ^ counter,
        BF2_KEY_SCHEDULE[1] ^ counter,
        BF2_KEY_SCHEDULE[2] ^ counter,
        BF2_KEY_SCHEDULE[3] ^ counter,
    ]
}

/// `psa_crypto_tea_crc`: byte sum of v0 and the top three bytes of v1.
fn tea_crc(v0: u32, v1: u32) -> u8 {
    let crc = (v0 >> 24) + ((v0 >> 16) & 0xFF) + ((v0 >> 8) & 0xFF) + (v0 & 0xFF)
        + (v1 >> 24)
        + ((v1 >> 16) & 0xFF)
        + ((v1 >> 8) & 0xFF);
    crc as u8
}

/// `psa_crypto_crc16_bf2`: CRC-16, poly 0x8005, init 0, MSB first (ARF's table is the same CRC).
fn crc16_bf2(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 { (crc << 1) ^ 0x8005 } else { crc << 1 };
        }
    }
    crc
}

/// BF1 acceptance (`psa_brute_force_run`): serial echo + byte-sum CRC.
#[inline]
fn bf1_accepts(counter: u32, v0: u32, v1: u32) -> bool {
    (counter & 0x00FF_FFFF) == (v0 >> 8) && tea_crc(v0, v1) == v1 as u8
}

/// BF2 acceptance: serial echo + CRC-16 over [b2, b4, b3, b5, b6, b7] (the references' byte
/// order). PP/doctor compare with b8:b9, ARF with b7:b9; either is accepted (see module doc).
#[inline]
fn bf2_accepts(counter: u32, v0: u32, v1: u32) -> bool {
    if (counter & 0x00FF_FFFF) != (v0 >> 8) {
        return false;
    }
    let crc_buffer = [
        (v0 >> 24) as u8,
        (v0 >> 8) as u8,
        (v0 >> 16) as u8,
        v0 as u8,
        (v1 >> 24) as u8,
        (v1 >> 16) as u8,
    ];
    let crc = crc16_bf2(&crc_buffer);
    let pp_expected = v1 as u16;
    let arf_expected = ((((v1 >> 16) & 0xFF) << 8) | (v1 & 0xFF)) as u16;
    crc == pp_expected || crc == arf_expected
}

/// `psa_crypto_setup_byte_buffer`: b0..b7 = key1 MSB first, b8 = key2 >> 8, b9 = key2.
fn setup_byte_buffer(key1: u64, key2_low: u32) -> [u8; 16] {
    let mut buf = [0u8; 16];
    buf[..8].copy_from_slice(&key1.to_be_bytes());
    buf[8] = (key2_low >> 8) as u8;
    buf[9] = key2_low as u8;
    buf
}

/// `psa_crypto_prepare_tea_data`: the TEA ciphertext words b2..b5, b6..b9.
fn prepare_tea_data(buf: &[u8; 16]) -> (u32, u32) {
    let w0 = u32::from_be_bytes([buf[2], buf[3], buf[4], buf[5]]);
    let w1 = u32::from_be_bytes([buf[6], buf[7], buf[8], buf[9]]);
    (w0, w1)
}

/// `psa_direct_xor_allowed_by_key2`: the key2 high byte selects the XOR (0x23) family.
fn direct_xor_allowed_by_key2(key2_high_byte: u8) -> bool {
    let lo = key2_high_byte & 0xF;
    lo < 3 || (lo < 7 && (key2_high_byte & 0xC) != 0)
}

/// `psa_calculate_checksum`: nibble sum of b2..b7, times 16 (stored in buffer[11] by the C).
fn calculate_checksum(buf: &[u8; 16]) -> u8 {
    let sum: u32 = buf[2..8].iter().map(|&b| (b & 0xF) as u32 + (b >> 4) as u32).sum();
    (sum.wrapping_mul(0x10) & 0xFF) as u8
}

/// `psa_second_stage_xor_decrypt` (with `psa_copy_reverse`).
fn second_stage_xor_decrypt(buf: &mut [u8; 16]) {
    let t = [buf[5], buf[4], buf[3], buf[2], buf[9], buf[8], buf[7], buf[6]];
    buf[2] = t[0] ^ t[6];
    buf[3] = t[2] ^ t[0];
    buf[4] = t[6] ^ t[3];
    buf[5] = t[7] ^ t[1];
    buf[6] = t[3] ^ t[1];
    buf[7] = t[6] ^ t[4] ^ t[5];
}

/// `psa_second_stage_xor_encrypt`: inverse of [`second_stage_xor_decrypt`] for b2..b7.
fn second_stage_xor_encrypt(buf: &mut [u8; 16]) {
    let (e6, e7) = (buf[8], buf[9]);
    let p = [buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]];
    let e5 = p[5] ^ e7 ^ e6;
    let e0 = p[2] ^ e5;
    let e2 = p[4] ^ e0;
    let e4 = p[3] ^ e2;
    let e3 = p[0] ^ e5;
    let e1 = p[1] ^ e3;
    buf[2] = e0;
    buf[3] = e1;
    buf[4] = e2;
    buf[5] = e3;
    buf[6] = e4;
    buf[7] = e5;
}

// ---------------------------------------------------------------------------------------------
// Decryption
// ---------------------------------------------------------------------------------------------

/// Plaintext fields of a decrypted frame (the references' `decrypted_*`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PsaFields {
    /// 0x23 (XOR) or 0x36 (TEA).
    pub mode: u8,
    pub serial: u32,
    pub button: u8,
    /// 16 bits for mode 0x23, 28 bits for mode 0x36.
    pub counter: u32,
    pub crc: u16,
}

/// `psa_direct_xor_decrypt` on the frame's byte buffer: checksum gate, then the XOR stage and
/// `psa_extract_fields_mode23`.
fn xor_decrypt(key1: u64, key2_low: u32) -> Option<PsaFields> {
    let mut buf = setup_byte_buffer(key1, key2_low);
    let checksum = calculate_checksum(&buf);
    if (checksum ^ buf[8]) & 0xF0 != 0 {
        return None;
    }
    second_stage_xor_decrypt(&mut buf);
    Some(PsaFields {
        mode: MODE_XOR,
        serial: u32::from_be_bytes([0, buf[2], buf[3], buf[4]]),
        button: buf[8] & 0xF,
        counter: ((buf[5] as u32) << 8) | buf[6] as u32,
        crc: buf[7] as u16,
    })
}

/// ProtoPirate's routing (`psa_handle_decoded_frame` / `psa_decrypt_router`): the XOR fast path
/// runs only when the key2 high byte selects it; anything else is a TEA frame for the brute force.
fn fast_decrypt(key1: u64, key2_low: u32) -> Option<PsaFields> {
    if !direct_xor_allowed_by_key2((key2_low >> 8) as u8) {
        return None;
    }
    xor_decrypt(key1, key2_low)
}

/// `psa_extract_fields_mode36` on the TEA plaintext words.
fn tea_fields(v0: u32, v1: u32) -> PsaFields {
    let b = [v0.to_be_bytes(), v1.to_be_bytes()].concat(); // b2..b9
    PsaFields {
        mode: MODE_TEA,
        serial: v0 >> 8,
        button: (b[3] >> 4) & 0xF,
        counter: ((b[5] as u32) << 8) | ((b[4] as u32) << 16) | b[6] as u32 | (((b[3] & 0xF) as u32) << 24),
        crc: b[7] as u16,
    }
}

/// O(1) TEA decrypt once the serial is known (after a brute force): try the BF1 key, then the BF2
/// key, for key counter `0x23000000 | serial` / `0xF3000000 | serial`. Returns the fields and which
/// range (1 or 2) matched.
pub fn tea_decrypt_with_serial(key1: u64, key2_low: u32, serial: u32) -> Option<(PsaFields, u8)> {
    let (w0, w1) = prepare_tea_data(&setup_byte_buffer(key1, key2_low));
    let serial = serial & 0x00FF_FFFF;
    let c1 = BF1_START | serial;
    let (mut v0, mut v1) = (w0, w1);
    tea_decrypt(&mut v0, &mut v1, &bf1_working_key(&TeaSchedule::new(&BF1_KEY_SCHEDULE), c1));
    if bf1_accepts(c1, v0, v1) {
        return Some((tea_fields(v0, v1), 1));
    }
    let c2 = BF2_START | serial;
    let (mut v0, mut v1) = (w0, w1);
    tea_decrypt(&mut v0, &mut v1, &bf2_working_key(c2));
    if bf2_accepts(c2, v0, v1) {
        return Some((tea_fields(v0, v1), 2));
    }
    None
}

// ---------------------------------------------------------------------------------------------
// Brute force (psa_crypto_bf.c `psa_brute_force_run`), multithreaded
// ---------------------------------------------------------------------------------------------

/// Cancel flag and progress counter for a running [`brute_force`] (the references'
/// `BruteForceState.cancel` / `progress_current`; a full run counts 2 × 2^24 candidates).
#[derive(Debug, Default)]
pub struct BruteForceControl {
    pub cancel: AtomicBool,
    pub progress: AtomicU32,
}

/// A brute-force hit: the TEA key counter and the decrypted fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BruteForceHit {
    /// 1 = BF1 (0x23xxxxxx, derived key), 2 = BF2 (0xF3xxxxxx, XOR key).
    pub range: u8,
    /// The key counter (ARF's `decrypted_seed`); its low 24 bits are the serial.
    pub key_counter: u32,
    pub fields: PsaFields,
}

/// Result of [`brute_force`] (`BRUTEFORCE_STATUS_FOUND` / `_NOT_FOUND` / `_CANCELLED`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BruteForceOutcome {
    Found(BruteForceHit),
    NotFound,
    Cancelled,
}

/// Total candidates of a full [`recover_key`] search (BF1 + BF2), the denominator for
/// [`BruteForceControl::progress`].
pub const BRUTE_FORCE_TOTAL: u32 = 2 * BF_RANGE;

/// The signal menu's "Recover key" action: ProtoPirate's `psa_brute_force_run` on a stored capture
/// (`data` = key1, `extra` = key2 as the decoder stores them). Run it off the UI thread; it honours
/// `ctl.cancel` and advances `ctl.progress` (out of [`BRUTE_FORCE_TOTAL`]).
pub fn recover_key(data: u64, extra: u64, ctl: &BruteForceControl) -> BruteForceOutcome {
    brute_force(data, extra as u32, ctl)
}

/// Display name of an adaptive-AM-path TEA frame the fast path cannot decrypt, reported only with
/// `autobf` ([ProtocolRegistry::set_auto_key_search](super::ProtocolRegistry::set_auto_key_search)):
/// it counts as PSA once the brute force finds its key, and is noise otherwise (see
/// [PsaDecoder::emit]).
pub const UNCHECKED_NAME: &str = "PSA [unchecked]";

/// What the key search names in a capture's result.
pub fn search_label(hit: &BruteForceHit) -> String {
    format!("PSA BF{}", hit.range)
}

/// The search a [BruteForceOutcome::NotFound] ran.
pub const SEARCH_LABEL_ALL: &str = "PSA BF1/BF2";

/// Whether the key search applies to a decode: a PSA frame (listed, or [UNCHECKED_NAME]) with
/// key2 and no serial, i.e. an undecrypted TEA frame.
pub fn needs_key_search(name: &str, serial: Option<u32>, extra: Option<u64>) -> bool {
    (name == "PSA" || name == UNCHECKED_NAME) && serial.is_none() && extra.is_some()
}

/// `d` with the fields a brute-force hit decrypted: plain PSA, encoder-capable.
pub fn with_recovered_fields(d: &DecodedSignal, hit: &BruteForceHit) -> DecodedSignal {
    DecodedSignal {
        serial: Some(hit.fields.serial),
        button: Some(hit.fields.button),
        // PSA counters are up to 28 bits; captures keep the low 16, like the decoder.
        counter: Some(hit.fields.counter as u16),
        crc_valid: true,
        encoder_capable: true,
        protocol_display_name: None,
        ..d.clone()
    }
}

/// What `autobf` makes of one decode when the search runs in line (`kat --decode`; the app runs
/// [recover_key] on a thread instead).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AutoKeySearch {
    /// No key search applies (a decrypted frame, another protocol): report it as decoded.
    NotNeeded,
    /// The key was found: the decode with its fields (plain PSA), and the search that found it.
    Recovered(DecodedSignal, String),
    /// A PSA frame no key fits: reported undecrypted, with the search that ran.
    NotFound(String),
    /// An [UNCHECKED_NAME] frame no key fits: noise, not reported.
    Dropped,
}

/// Run the PSA key search on `d` (decoded as `name`) in line, if it applies.
pub fn auto_key_search(name: &str, d: &DecodedSignal) -> AutoKeySearch {
    let (true, Some(extra)) = (needs_key_search(name, d.serial, d.extra), d.extra) else {
        return AutoKeySearch::NotNeeded;
    };
    match recover_key(d.data, extra, &BruteForceControl::default()) {
        BruteForceOutcome::Found(hit) => AutoKeySearch::Recovered(with_recovered_fields(d, &hit), search_label(&hit)),
        _ if name == UNCHECKED_NAME => AutoKeySearch::Dropped,
        _ => AutoKeySearch::NotFound(SEARCH_LABEL_ALL.to_string()),
    }
}

/// Candidates per work unit of the parallel search.
const BF_CHUNK: u32 = 1 << 15;

/// Lowest offset in `0..BF_RANGE` for which `accepts(start + offset)` holds — the same candidate
/// the C's sequential loop stops at. Work units are claimed in increasing order; a worker stops
/// at the first hit of its unit and skips units that start beyond the best hit so far.
fn search_range<F>(accepts: F, ctl: &BruteForceControl) -> Result<Option<u32>, ()>
where
    F: Fn(u32) -> bool + Sync,
{
    let units = BF_RANGE / BF_CHUNK;
    let next = AtomicU32::new(0);
    let best = AtomicU32::new(u32::MAX);
    let workers = std::thread::available_parallelism().map_or(1, |n| n.get()).clamp(1, units as usize);
    std::thread::scope(|scope| {
        for _ in 0..workers {
            scope.spawn(|| loop {
                if ctl.cancel.load(Ordering::Relaxed) {
                    break;
                }
                let unit = next.fetch_add(1, Ordering::Relaxed);
                if unit >= units {
                    break;
                }
                let lo = unit * BF_CHUNK;
                if lo > best.load(Ordering::Relaxed) {
                    break;
                }
                for off in lo..lo + BF_CHUNK {
                    if accepts(off) {
                        best.fetch_min(off, Ordering::Relaxed);
                        break;
                    }
                }
                ctl.progress.fetch_add(BF_CHUNK, Ordering::Relaxed);
            });
        }
    });
    let best = best.load(Ordering::Relaxed);
    if best != u32::MAX {
        // Every unit below `best` was searched in full, so it is the lowest hit even if a
        // cancel arrived meanwhile.
        return Ok(Some(best));
    }
    if ctl.cancel.load(Ordering::Relaxed) {
        return Err(());
    }
    Ok(None)
}

/// ProtoPirate `psa_brute_force_run` for a frame's key1/key2: BF1 over 0x23000000..0x24000000
/// (working key = two TEA encrypts of the counter under the BF1 schedule), then BF2 over
/// 0xF3000000..0xF4000000 (working key = BF2 schedule XOR counter). Returns the first hit in the
/// C's order. Runs on all cores; a full miss is ~2 × 2^24 TEA decrypts (see `docs/psa.md` for
/// timings). Set `ctl.cancel` from another thread to stop early.
pub fn brute_force(key1: u64, key2_low: u32, ctl: &BruteForceControl) -> BruteForceOutcome {
    let (w0, w1) = prepare_tea_data(&setup_byte_buffer(key1, key2_low));
    let sched = TeaSchedule::new(&BF1_KEY_SCHEDULE);
    let bf1 = |off: u32| {
        let counter = BF1_START + off;
        let (mut v0, mut v1) = (w0, w1);
        tea_decrypt(&mut v0, &mut v1, &bf1_working_key(&sched, counter));
        bf1_accepts(counter, v0, v1)
    };
    let bf2 = |off: u32| {
        let counter = BF2_START + off;
        let (mut v0, mut v1) = (w0, w1);
        tea_decrypt(&mut v0, &mut v1, &bf2_working_key(counter));
        bf2_accepts(counter, v0, v1)
    };
    for (range, start) in [(1u8, BF1_START), (2u8, BF2_START)] {
        let found = if range == 1 { search_range(bf1, ctl) } else { search_range(bf2, ctl) };
        match found {
            Err(()) => return BruteForceOutcome::Cancelled,
            Ok(None) => {}
            Ok(Some(off)) => {
                let key_counter = start + off;
                let key = if range == 1 { bf1_working_key(&sched, key_counter) } else { bf2_working_key(key_counter) };
                let (mut v0, mut v1) = (w0, w1);
                tea_decrypt(&mut v0, &mut v1, &key);
                return BruteForceOutcome::Found(BruteForceHit { range, key_counter, fields: tea_fields(v0, v1) });
            }
        }
    }
    BruteForceOutcome::NotFound
}

// ---------------------------------------------------------------------------------------------
// Decoder
// ---------------------------------------------------------------------------------------------

/// Which reference's `subghz_protocol_decoder_psa_feed` a [`Machine`] runs. The two C files share
/// States 0/1/2 (the 250/500 µs path; their transitions are identical, only the frames they report
/// differ) and differ in States 3/4.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Flavor {
    /// ProtoPirate psa.c: fixed 125/250 µs States 3/4. They can never complete a frame (see the
    /// module doc) but hold the machine exactly like the C does.
    ProtoPirate,
    /// Flipper-ARF psa.c: adaptive AM States 3/4.
    Arf,
    /// doctor-firmware's first psa.c (e7e788a): States 0/1/2 with a `>=` preamble threshold and
    /// the frame complete at its 80th bit (`psa_try_complete_at_key2`, no end pulse needed), and
    /// working 125/250 µs States 3/4 that complete the same way. Its current psa.c is another
    /// machine altogether: [`AdaptiveMachine`].
    Doctor,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DecoderState {
    /// State0: wait for a preamble HIGH (151..349 µs → State1, 85..150 µs → State3).
    Reset,
    /// State1: count 250 µs preamble LOWs; a 401..599 µs LOW after >0x46 of them → State2.
    Preamble250,
    /// State2: 250/500 µs Manchester, key1 (64) + key2 (16), 1000 µs end marker.
    Data250,
    /// State3: PP / doctor 125 µs preamble, ARF adaptive AM preamble (te from HIGH+LOW cell periods).
    Preamble125,
    /// State4: PP / doctor 125/250 µs Manchester, ARF adaptive AM Manchester with frame realignment.
    Data125,
}

/// A frame a machine completed (where the C calls back): key1 and the C's key2 words.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct RawFrame {
    key1: u64,
    key2_high: u32,
    key2_low: u32,
    /// A frame that must prove itself: completed on a fast-timing path (ARF's adaptive AM path,
    /// `psa_am_complete`, or doctor's 125/250 µs States 3/4), or on the 250/500 µs path by doctor's
    /// 80th-bit rule alone (no end pulse ProtoPirate's machine takes). KAT reports such a frame only
    /// when it decrypts, or with `autobf` as [UNCHECKED_NAME] for the key search to confirm
    /// ([PsaDecoder::emit]).
    am: bool,
}

/// Outcome of ARF's `psa_am_complete`.
enum AmComplete {
    /// A window passed the sync/varied checks (the C fires its callback).
    Fired(RawFrame),
    /// No alignment passed: the machine was rewound to State0.
    Rejected,
    /// The bit buffer is not 80 or 88 bits long: nothing changed.
    Ignored,
}

/// One reference decoder: the `SubGhzProtocolDecoderPSA` fields its `feed` uses.
struct Machine {
    flavor: Flavor,
    state: DecoderState,
    prev_duration: u32,
    decode_data_low: u32,
    decode_data_high: u32,
    decode_count_bit: u8,
    key1_low: u32,
    key1_high: u32,
    pattern_counter: u16,
    manchester_state: ManchesterState,
    // ARF adaptive AM state
    te_sum: u32,
    te_count: u16,
    te_detected: u32,
    pre_glitch: u8,
    am_await_high: bool,
    am_bits: [u8; AM_BITS_CAP],
    am_bits_len: usize,
}

fn abs_diff(a: u32, b: u32) -> u32 {
    a.abs_diff(b)
}

impl Machine {
    fn new(flavor: Flavor) -> Self {
        Self {
            flavor,
            state: DecoderState::Reset,
            prev_duration: 0,
            decode_data_low: 0,
            decode_data_high: 0,
            decode_count_bit: 0,
            key1_low: 0,
            key1_high: 0,
            pattern_counter: 0,
            manchester_state: ManchesterState::Mid1,
            te_sum: 0,
            te_count: 0,
            te_detected: 0,
            pre_glitch: 0,
            am_await_high: false,
            am_bits: [0; AM_BITS_CAP],
            am_bits_len: 0,
        }
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<RawFrame> {
        match (self.state, self.flavor) {
            (DecoderState::Reset, _) => self.feed_reset(level, duration),
            (DecoderState::Preamble250, _) => self.feed_preamble_250(level, duration),
            (DecoderState::Data250, _) => self.feed_data_250(level, duration),
            (DecoderState::Preamble125, Flavor::ProtoPirate) => self.feed_pp_preamble_125(duration),
            (DecoderState::Data125, Flavor::ProtoPirate) => self.feed_pp_data_125(level, duration),
            (DecoderState::Preamble125, Flavor::Arf) => self.feed_am_preamble(level, duration),
            (DecoderState::Data125, Flavor::Arf) => self.feed_am_data(level, duration),
            (DecoderState::Preamble125, Flavor::Doctor) => self.feed_doctor_preamble_125(duration),
            (DecoderState::Data125, Flavor::Doctor) => self.feed_doctor_data_125(level, duration),
        }
    }

    /// The common tail of the C feed: `if(new_state == State0) am_bits_len = 0; state = new_state;
    /// prev_duration = duration;` (the `am_bits` reset is ARF's; PP never uses the buffer).
    fn finish(&mut self, new_state: DecoderState, duration: u32) -> Option<RawFrame> {
        if new_state == DecoderState::Reset {
            self.am_bits_len = 0;
        }
        self.state = new_state;
        self.prev_duration = duration;
        None
    }

    fn clear_data(&mut self) {
        self.decode_data_low = 0;
        self.decode_data_high = 0;
        self.decode_count_bit = 0;
    }

    fn shift_in(&mut self, bit: bool) {
        let carry = self.decode_data_low >> 31;
        self.decode_data_low = (self.decode_data_low << 1) | bit as u32;
        self.decode_data_high = (self.decode_data_high << 1) | carry;
        self.decode_count_bit = self.decode_count_bit.wrapping_add(1);
        if self.decode_count_bit == KEY1_BITS {
            self.key1_low = self.decode_data_low;
            self.key1_high = self.decode_data_high;
            self.decode_data_low = 0;
            self.decode_data_high = 0;
        }
    }

    fn key1(&self) -> u64 {
        ((self.key1_high as u64) << 32) | self.key1_low as u64
    }

    fn sync_ok(&self) -> bool {
        (self.key1_high >> 16) & 0xF == SYNC_NIBBLE
    }

    /// The frame `psa_handle_decoded_frame` reports: key2 = the bits after key1.
    fn complete_frame(&mut self) -> RawFrame {
        let frame = RawFrame { key1: self.key1(), key2_high: self.decode_data_high, key2_low: self.decode_data_low, am: false };
        self.clear_data();
        frame
    }

    /// doctor-firmware `psa_try_complete_at_key2`, called as the 80th bit is shifted in: with the
    /// 0xA sync the frame is reported (its routing is ProtoPirate's, see [PsaDecoder::emit]),
    /// without it the bits are dropped; State0 either way. `fast` marks the 125 µs path's frames.
    fn try_complete_at_key2(&mut self, fast: bool) -> Option<RawFrame> {
        self.state = DecoderState::Reset;
        if !self.sync_ok() {
            self.clear_data();
            return None;
        }
        Some(RawFrame { am: fast, ..self.complete_frame() })
    }

    // ---- States 0..2 (identical in ProtoPirate and Flipper-ARF) ------------------------------

    fn feed_reset(&mut self, level: bool, duration: u32) -> Option<RawFrame> {
        if !level {
            return None;
        }
        let new_state = if duration < TE_SHORT {
            if TE_SHORT - duration > TOLERANCE_99 {
                let to_125 = abs_diff(duration, TE_SHORT_125);
                let out = match self.flavor {
                    // doctor: `psa_is_te_125_short` (125 ± 49 µs).
                    Flavor::Doctor => to_125 > TOLERANCE_49,
                    Flavor::ProtoPirate | Flavor::Arf => to_125 > 40 || duration > 180,
                };
                if out {
                    return None;
                }
                DecoderState::Preamble125
            } else {
                DecoderState::Preamble250
            }
        } else {
            if duration - TE_SHORT > TOLERANCE_99 {
                return None;
            }
            DecoderState::Preamble250
        };
        self.decode_data_low = 0;
        self.decode_data_high = 0;
        self.pattern_counter = 0;
        self.decode_count_bit = 0;
        self.te_sum = duration;
        self.te_count = 1;
        self.te_detected = 0;
        self.pre_glitch = 0;
        self.am_await_high = false;
        self.am_bits_len = 0;
        self.manchester_state = ManchesterState::Mid1;
        self.finish(new_state, duration)
    }

    fn feed_preamble_250(&mut self, level: bool, duration: u32) -> Option<RawFrame> {
        if level {
            return None;
        }
        let short = if duration < TE_SHORT {
            TE_SHORT - duration < TOLERANCE_100
        } else {
            duration - TE_SHORT < TOLERANCE_100
        };
        if short {
            if abs_diff(self.prev_duration, TE_SHORT) <= TOLERANCE_99 {
                self.pattern_counter = self.pattern_counter.wrapping_add(1);
            }
            self.prev_duration = duration;
            return None;
        }
        if duration > TE_SHORT && abs_diff(duration, TE_LONG) < 100 {
            // ProtoPirate / ARF: more than 0x46 preamble LOWs; doctor: at least 0x46.
            let enough = match self.flavor {
                Flavor::Doctor => self.pattern_counter >= PATTERN_THRESHOLD_1,
                Flavor::ProtoPirate | Flavor::Arf => self.pattern_counter > PATTERN_THRESHOLD_1,
            };
            if enough {
                self.clear_data();
                self.manchester_state = ManchesterState::Mid1;
                self.state = DecoderState::Data250;
            }
            self.pattern_counter = 0;
            self.prev_duration = duration;
            return None;
        }
        self.pattern_counter = 0;
        self.finish(DecoderState::Reset, duration)
    }

    /// The end HIGH after the 80th bit: the references' 1000 ± 199 µs (genuine fobs: ~900 µs, ~1140 µs
    /// after a final 0 bit; doctor also takes exactly 800 µs) and, KAT only, 1250 ± 199 µs: the
    /// references' own encoder upload (a Flipper sending a saved file, KAT ≤ 1.8.3) after a final 0
    /// bit, whose 250 µs HIGH half and 1000 µs end HIGH merge on the air. ProtoPirate and ARF decode
    /// neither their own transmission of such a frame nor each other's. The doctor machine never
    /// gets here: it completes at the 80th bit.
    fn is_end_marker(&self, duration: u32) -> bool {
        match self.flavor {
            Flavor::Doctor => duration >= 800 && abs_diff(duration, TE_END_1000) <= 199,
            Flavor::ProtoPirate | Flavor::Arf => {
                abs_diff(duration, TE_END_1000) <= 199 || abs_diff(duration, TE_END_1000 + TE_SHORT) <= 199
            }
        }
    }

    fn feed_data_250(&mut self, level: bool, duration: u32) -> Option<RawFrame> {
        if self.decode_count_bit >= MAX_BITS {
            return self.finish(DecoderState::Reset, duration);
        }
        if level && self.decode_count_bit == KEY2_BITS && self.is_end_marker(duration) {
            // ProtoPirate reports the frame only with the 0xA sync (ARF also on a passing XOR
            // checksum; KAT does not take that path, see the module doc). Both rewind to State0.
            if !self.sync_ok() {
                self.clear_data();
                self.state = DecoderState::Reset;
                return None;
            }
            let frame = self.complete_frame();
            self.state = DecoderState::Reset;
            return Some(frame);
        }

        let short = ManchesterEvent::from_half_bit(!level, false);
        let long = ManchesterEvent::from_half_bit(!level, true);
        let event = if duration < TE_SHORT {
            if TE_SHORT - duration >= TOLERANCE_100 {
                return None;
            }
            Some(short)
        } else if duration - TE_SHORT < TOLERANCE_100 {
            Some(short)
        } else if duration < TE_LONG {
            let from_250 = duration - TE_SHORT;
            let from_500 = TE_LONG - duration;
            if from_500 < 150 || from_250 > from_500 {
                Some(long)
            } else if from_250 < 150 {
                Some(short)
            } else if duration > 10000 {
                // Unreachable in the C too (duration < 500 here): `new_state = 0` without effect.
                self.pattern_counter = 0;
                return None;
            } else if (350..=400).contains(&duration) {
                Some(long)
            } else {
                return None;
            }
        } else if duration - TE_LONG < 100 {
            Some(long)
        } else {
            if !level {
                if duration > 10000 {
                    // C: `new_state = PSADecoderState0; pattern_counter = 0; return;` — the return
                    // skips the state store, so the decoder stays in State2.
                    self.pattern_counter = 0;
                }
                return None;
            }
            None
        };

        if let Some(ev) = event {
            if self.decode_count_bit < KEY2_BITS {
                if let Some(bit) = self.manchester_state.advance(ev) {
                    self.shift_in(bit);
                    if self.flavor == Flavor::Doctor && self.decode_count_bit == KEY2_BITS {
                        return self.try_complete_at_key2(false);
                    }
                }
            }
        }
        if !level {
            return None;
        }
        if event.is_none() {
            // A HIGH outside both symbol windows: the second end-marker check.
            if self.is_end_marker(duration) && self.decode_count_bit == KEY2_BITS && self.sync_ok() {
                let frame = self.complete_frame();
                self.state = DecoderState::Reset;
                return Some(frame);
            }
            return None;
        }
        self.finish(DecoderState::Data250, duration)
    }

    // ---- ProtoPirate States 3/4 (125/250 µs) ---------------------------------------------------

    fn feed_pp_preamble_125(&mut self, duration: u32) -> Option<RawFrame> {
        if duration >= TE_LONG_250 {
            if duration < TE_LONG_300 && self.pattern_counter > PATTERN_THRESHOLD_2 {
                self.clear_data();
                self.manchester_state = ManchesterState::Mid1;
                self.state = DecoderState::Data125;
                self.pattern_counter = 0;
                self.prev_duration = duration;
                return None;
            }
            self.pattern_counter = 0;
            return self.finish(DecoderState::Reset, duration);
        }
        if abs_diff(duration, TE_SHORT_125) < TOLERANCE_50 {
            if abs_diff(self.prev_duration, TE_SHORT_125) <= TOLERANCE_49 {
                self.pattern_counter = self.pattern_counter.wrapping_add(1);
            } else {
                self.pattern_counter = 0;
            }
            self.prev_duration = duration;
            return None;
        }
        self.pattern_counter = 0;
        self.finish(DecoderState::Reset, duration)
    }

    /// PP State4 feeds only LOW pulses (events ShortHigh/LongHigh) to the Manchester decoder, so
    /// it can only produce 0 bits and its sync check never passes; ported for the trajectory.
    fn feed_pp_data_125(&mut self, level: bool, duration: u32) -> Option<RawFrame> {
        if self.decode_count_bit >= MAX_BITS {
            return self.finish(DecoderState::Reset, duration);
        }
        if !level {
            let event = if duration < TE_SHORT_125 {
                if TE_SHORT_125 - duration > TOLERANCE_49 {
                    return None;
                }
                ManchesterEvent::ShortHigh
            } else if duration - TE_SHORT_125 < TOLERANCE_50 {
                ManchesterEvent::ShortHigh
            } else if (TE_LONG_250..TE_LONG_300).contains(&duration) {
                ManchesterEvent::LongHigh
            } else {
                return None;
            };
            if let Some(bit) = self.manchester_state.advance(event) {
                self.shift_in(bit);
            }
            return self.finish(DecoderState::Data125, duration);
        }
        if abs_diff(duration, TE_END_500) <= 99 && self.decode_count_bit == KEY2_BITS {
            if !self.sync_ok() {
                self.clear_data();
                self.state = DecoderState::Reset;
                return None;
            }
            let frame = self.complete_frame();
            self.state = DecoderState::Reset;
            return Some(frame);
        }
        None
    }

    // ---- doctor-firmware States 3/4 (125/250 µs) -----------------------------------------------

    /// doctor `psa_is_te_125_short`: 125 ± 49 µs.
    fn is_te_125_short(duration: u32) -> bool {
        abs_diff(duration, TE_SHORT_125) <= TOLERANCE_49
    }

    /// doctor `psa_is_te_125_long`: 250 ± 49 µs.
    fn is_te_125_long(duration: u32) -> bool {
        abs_diff(duration, TE_LONG_250) <= TOLERANCE_49
    }

    /// doctor State3: count 125 µs pulses of either level (each after another one); a 250 µs pulse
    /// after at least 0x45 of them starts the data, any other pulse rewinds to State0.
    fn feed_doctor_preamble_125(&mut self, duration: u32) -> Option<RawFrame> {
        if Self::is_te_125_long(duration) {
            if self.pattern_counter >= PATTERN_THRESHOLD_2 {
                self.clear_data();
                self.manchester_state = ManchesterState::Mid1;
                self.state = DecoderState::Data125;
                self.pattern_counter = 0;
                self.prev_duration = duration;
                return None;
            }
            self.pattern_counter = 0;
            return self.finish(DecoderState::Reset, duration);
        }
        if Self::is_te_125_short(duration) {
            if abs_diff(self.prev_duration, TE_SHORT_125) <= TOLERANCE_49 {
                self.pattern_counter = self.pattern_counter.wrapping_add(1);
            } else {
                self.pattern_counter = 0;
            }
            self.prev_duration = duration;
            return None;
        }
        self.pattern_counter = 0;
        self.finish(DecoderState::Reset, duration)
    }

    /// doctor State4: 125/250 µs Manchester on both levels (ProtoPirate's State4 feeds only the
    /// LOWs), the frame complete at its 80th bit or on a 500 ± 99 µs end HIGH after it.
    fn feed_doctor_data_125(&mut self, level: bool, duration: u32) -> Option<RawFrame> {
        if self.decode_count_bit >= MAX_BITS {
            return self.finish(DecoderState::Reset, duration);
        }
        let event = if Self::is_te_125_short(duration) {
            Some(ManchesterEvent::from_half_bit(!level, false))
        } else if Self::is_te_125_long(duration) {
            Some(ManchesterEvent::from_half_bit(!level, true))
        } else {
            None
        };
        if let Some(ev) = event {
            if self.decode_count_bit < KEY2_BITS {
                if let Some(bit) = self.manchester_state.advance(ev) {
                    self.shift_in(bit);
                    if self.decode_count_bit == KEY2_BITS {
                        return self.try_complete_at_key2(true);
                    }
                }
            }
        }
        if !level {
            return None;
        }
        if event.is_none() {
            if abs_diff(duration, TE_END_500) <= 99 && self.decode_count_bit == KEY2_BITS {
                return self.try_complete_at_key2(true);
            }
            return None;
        }
        self.finish(DecoderState::Data125, duration)
    }

    // ---- Flipper-ARF States 3/4 (adaptive AM, ported from ARF's psa.js) ------------------------

    /// `psa_add_am_bit`: keep the raw bit for realignment (96-bit window) and shift it into the
    /// streaming accumulator (key1 latch at 64).
    fn add_am_bit(&mut self, bit: bool) {
        if self.am_bits_len >= AM_BITS_CAP {
            self.am_bits.copy_within(1.., 0);
            self.am_bits[AM_BITS_CAP - 1] = bit as u8;
        } else {
            self.am_bits[self.am_bits_len] = bit as u8;
            self.am_bits_len += 1;
        }
        self.shift_in(bit);
    }

    /// `psa_am_frame_at`: (key1_high, key1_low, key2) read from the bit window at `off`.
    fn am_frame_at(&self, off: usize) -> Option<(u32, u32, u32)> {
        if self.am_bits_len < off + 80 {
            return None;
        }
        let word = |from: usize, n: usize| {
            self.am_bits[from..from + n].iter().fold(0u32, |w, &b| (w << 1) | b as u32)
        };
        Some((word(off, 32), word(off + 32, 32), word(off + 64, 16)))
    }

    /// `psa_am_varied`: at least 4 distinct bytes in key1.
    fn am_varied(hi: u32, lo: u32) -> bool {
        let bytes = [hi.to_be_bytes(), lo.to_be_bytes()].concat();
        let distinct = (0..8).filter(|&i| !bytes[..i].contains(&bytes[i])).count();
        distinct >= 4
    }

    /// `psa_am_complete`: accept only an 80- or 88-bit window, try offsets 0..=min(8, len-80) for
    /// a sync-0xA, varied key1, and keep the alignment with the most leading 1s (bits 31..20).
    fn am_complete(&mut self) -> AmComplete {
        if self.am_bits_len < 80 || (self.am_bits_len != 80 && self.am_bits_len != 88) {
            return AmComplete::Ignored;
        }
        let max_off = (self.am_bits_len - 80).min(8);
        let mut best: Option<(usize, i32)> = None;
        for off in 0..=max_off {
            let Some((hi, lo, _)) = self.am_frame_at(off) else {
                continue;
            };
            if (hi >> 16) & 0xF != SYNC_NIBBLE || !Self::am_varied(hi, lo) {
                continue;
            }
            let score = (20..=31).rev().take_while(|&bit| (hi >> bit) & 1 == 1).count() as i32;
            if best.is_none_or(|(_, s)| score > s) {
                best = Some((off, score));
            }
        }
        let Some((off, _)) = best else {
            self.clear_data();
            self.am_bits_len = 0;
            self.state = DecoderState::Reset;
            return AmComplete::Rejected;
        };
        let (hi, lo, k2) = self.am_frame_at(off).expect("offset checked above");
        self.key1_high = hi;
        self.key1_low = lo;
        let frame = RawFrame { key1: self.key1(), key2_high: 0, key2_low: k2, am: true };
        self.clear_data();
        self.am_bits_len = 0;
        self.state = DecoderState::Reset;
        AmComplete::Fired(frame)
    }

    fn am_resync(&mut self) {
        self.clear_data();
        self.am_bits_len = 0;
        self.manchester_state = ManchesterState::Mid1;
    }

    fn feed_am_preamble(&mut self, level: bool, duration: u32) -> Option<RawFrame> {
        if level {
            self.prev_duration = duration;
            return None;
        }
        let high = self.prev_duration;
        let period = high.wrapping_add(duration);
        let mut te_avg = if self.te_count > 0 { self.te_sum / self.te_count as u32 } else { TE_SHORT_125 };
        if te_avg < AM_PRE_MIN {
            te_avg = TE_SHORT_125;
        }
        let period_est = te_avg.wrapping_mul(2);
        let cell_mid = period_est.wrapping_add(period_est >> 1);
        let cell_hi = period_est.wrapping_add(period_est).wrapping_add(period_est >> 1);
        let cell_min = AM_PRE_MIN * 2;
        let in_band = |d: u32| (AM_PRE_MIN..=AM_PRE_MAX).contains(&d);
        let is_cell = period >= cell_min && period <= cell_mid && in_band(high) && in_band(duration);
        let is_boundary = !is_cell && period > cell_mid && period <= cell_hi;

        if is_cell {
            self.pattern_counter = self.pattern_counter.wrapping_add(1);
            self.pre_glitch = 0;
            self.te_sum = self.te_sum.wrapping_add(period >> 1);
            self.te_count += 1;
            if self.te_count > 32 {
                self.te_sum = te_avg.wrapping_mul(8);
                self.te_count = 8;
            }
            self.prev_duration = duration;
            return None;
        }
        if self.pattern_counter >= AM_PRE_THRESHOLD {
            // Boundary cell: Manchester starts on the next pulse; otherwise (no textbook double
            // cell) skip the wake-up marker first (`am_await_high`).
            self.te_detected = te_avg;
            self.clear_data();
            self.am_bits_len = 0;
            self.manchester_state = ManchesterState::Mid1;
            self.am_await_high = !is_boundary;
            self.state = DecoderState::Data125;
            self.pattern_counter = 0;
            self.pre_glitch = 0;
            self.prev_duration = duration;
            return None;
        }
        self.pre_glitch = self.pre_glitch.wrapping_add(1);
        self.pattern_counter = self.pattern_counter.saturating_sub(AM_PRE_GLITCH_DECAY);
        if self.pre_glitch <= AM_PRE_MAX_GLITCH && self.pattern_counter > 0 {
            self.prev_duration = duration;
            return None;
        }
        self.pattern_counter = 0;
        self.pre_glitch = 0;
        self.finish(DecoderState::Reset, duration)
    }

    fn feed_am_data(&mut self, level: bool, duration: u32) -> Option<RawFrame> {
        let te_s = if self.te_detected != 0 { self.te_detected } else { TE_SHORT_125 };
        let te_l = te_s.wrapping_mul(2);
        let te_tol = te_s.wrapping_mul(7) / 10;
        let midpoint = te_s + (te_s >> 1);
        let terminator = duration > te_l.wrapping_add(te_tol);

        if self.am_await_high {
            if terminator {
                self.prev_duration = duration;
                return None;
            }
            self.am_await_high = false;
            self.am_resync();
        }

        if self.decode_count_bit >= KEY2_BITS && terminator {
            if let AmComplete::Fired(frame) = self.am_complete() {
                return Some(frame);
            }
        }

        if self.decode_count_bit >= MAX_BITS {
            return match self.am_complete() {
                // C: `new_state` keeps State4 although `am_complete` stored State0, and the
                // `break` then stores State4 back (unreachable: the window is 96 bits here).
                AmComplete::Fired(frame) => {
                    self.finish(DecoderState::Data125, duration);
                    Some(frame)
                }
                AmComplete::Rejected => self.finish(DecoderState::Reset, duration),
                // Length mismatch: `new_state = State0` is never stored (the C returns), so the
                // machine stays in State4 until a reset.
                AmComplete::Ignored => None,
            };
        }

        if terminator {
            if self.decode_count_bit >= KEY2_BITS {
                if let AmComplete::Fired(frame) = self.am_complete() {
                    return Some(frame);
                }
            }
            if self.decode_count_bit < AM_RESYNC_BITS {
                self.am_resync();
                return None;
            }
            if duration > 10000 {
                self.am_bits_len = 0;
                return self.finish(DecoderState::Reset, duration);
            }
            return None;
        }

        let long = duration > midpoint;
        let (target, event) = if long {
            (te_l, ManchesterEvent::from_half_bit(!level, true))
        } else {
            (te_s, ManchesterEvent::from_half_bit(!level, false))
        };
        if abs_diff(duration, target) > te_tol {
            if self.decode_count_bit < AM_RESYNC_BITS {
                self.am_resync();
            }
            return None;
        }
        if let Some(bit) = self.manchester_state.advance(event) {
            self.add_am_bit(bit);
        }
        self.finish(DecoderState::Data125, duration)
    }
}

/// doctor-firmware's current `subghz_protocol_decoder_psa_feed` (psa.c as rewritten in 98d1fc1):
/// one machine for both clocks that measures the half-bit on the preamble and reads the data
/// against that measure.
/// - State0: a HIGH of 250 ± 99 µs → State1, one of 125 ± 49 µs → State3.
/// - State1 / State3 (only LOWs count): a LOW within the nominal half-bit's window is one preamble
///   LOW, and the first 32 of them are averaged; a LOW of two half-bits ends the preamble, and after
///   more than 15 preamble LOWs the data follows (fewer: the count starts again). Anything else:
///   State0.
/// - State2 / State4: Manchester on both levels with `te` = the average: a half-bit is `te` ± 40 %,
///   a full bit 2 `te` ± 40 % of `te`; key1 is the first 64 bits. Two pulses outside both windows
///   are let through, the third rewinds. A HIGH of 3.2 `te` or more is the end of the frame, which
///   is reported when it is exactly 80 bits with the 0xA sync nibble.
///
/// Against ProtoPirate's machine: 16 preamble LOWs are enough (ProtoPirate: 71), the half-bit may
/// sit anywhere in the State0 windows, and the end HIGH has no upper limit; but its windows are
/// tighter where the fob's clock is fast (at 220 µs a 310 µs pulse is neither a half-bit nor a
/// full bit), and it wants the end HIGH that doctor-firmware's earlier machine ([Flavor::Doctor])
/// did without.
struct AdaptiveMachine {
    /// Only Reset, Preamble250 / Data250 (the C's States 1 / 2) and Preamble125 / Data125 (3 / 4).
    state: DecoderState,
    prev_duration: u32,
    decode_data_low: u32,
    decode_data_high: u32,
    decode_count_bit: u8,
    key1_low: u32,
    key1_high: u32,
    pattern_counter: u16,
    timing_sum: u16,
    timing_short: u16,
    timing_samples: u8,
    invalid_pulses: u8,
    manchester_state: ManchesterState,
}

/// More preamble LOWs than this before the sync (`pattern_counter > 15`).
const ADAPTIVE_PREAMBLE_LOWS: u16 = 15;
/// Preamble LOWs averaged into the half-bit.
const ADAPTIVE_TIMING_SAMPLES: u8 = 32;
/// Pulses outside both symbol windows a frame may hold.
const ADAPTIVE_INVALID_PULSES: u8 = 2;
/// `decode_count_bit > 120`: rewind.
const ADAPTIVE_MAX_BITS: u8 = 120;

impl AdaptiveMachine {
    fn new() -> Self {
        Self {
            state: DecoderState::Reset,
            prev_duration: 0,
            decode_data_low: 0,
            decode_data_high: 0,
            decode_count_bit: 0,
            key1_low: 0,
            key1_high: 0,
            pattern_counter: 0,
            timing_sum: 0,
            timing_short: 0,
            timing_samples: 0,
            invalid_pulses: 0,
            manchester_state: ManchesterState::Mid1,
        }
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<RawFrame> {
        match self.state {
            DecoderState::Reset => {
                if !level {
                    return None;
                }
                self.state = if abs_diff(duration, TE_SHORT) <= TOLERANCE_99 {
                    DecoderState::Preamble250
                } else if duration < TE_SHORT && abs_diff(duration, TE_SHORT_125) <= TOLERANCE_49 {
                    DecoderState::Preamble125
                } else {
                    return None;
                };
                self.decode_data_low = 0;
                self.decode_data_high = 0;
                self.decode_count_bit = 0;
                self.pattern_counter = 0;
                self.timing_sum = 0;
                self.timing_samples = 0;
                self.prev_duration = duration;
                self.manchester_state = ManchesterState::Mid1;
                None
            }
            DecoderState::Preamble250 | DecoderState::Preamble125 => {
                if level {
                    return None;
                }
                let fast = self.state == DecoderState::Preamble125;
                let (nominal, tolerance) = if fast { (TE_SHORT_125, TOLERANCE_49) } else { (TE_SHORT, TOLERANCE_99) };
                if abs_diff(duration, nominal) <= tolerance {
                    // `prev_duration` is the State0 HIGH (no later pulse is stored), which chose
                    // this state by the same window: the C's check never fails.
                    if abs_diff(self.prev_duration, nominal) > tolerance {
                        self.state = DecoderState::Reset;
                        return None;
                    }
                    self.pattern_counter = self.pattern_counter.wrapping_add(1);
                    if self.timing_samples < ADAPTIVE_TIMING_SAMPLES {
                        // at most 32 × 349 µs: fits the C's uint16_t
                        self.timing_sum += duration as u16;
                        self.timing_samples += 1;
                    }
                    return None;
                }
                if duration < nominal || abs_diff(duration, nominal * 2) > tolerance {
                    self.state = DecoderState::Reset;
                    return None;
                }
                if self.pattern_counter > ADAPTIVE_PREAMBLE_LOWS {
                    self.state = if fast { DecoderState::Data125 } else { DecoderState::Data250 };
                    self.timing_short = match self.timing_samples {
                        0 => nominal as u16,
                        n => self.timing_sum / u16::from(n),
                    };
                    self.decode_data_low = 0;
                    self.decode_data_high = 0;
                    self.decode_count_bit = 0;
                    self.invalid_pulses = 0;
                    self.manchester_state = ManchesterState::Mid1;
                }
                self.pattern_counter = 0;
                None
            }
            DecoderState::Data250 | DecoderState::Data125 => {
                if self.decode_count_bit > ADAPTIVE_MAX_BITS {
                    self.state = DecoderState::Reset;
                    return None;
                }
                let te = u32::from(self.timing_short);
                let tolerance = te * 2 / 5;
                let end_high = level && duration >= te * 16 / 5;
                // C: `level ? ShortLow : ShortHigh`
                let event = if abs_diff(duration, te) < tolerance {
                    Some(ManchesterEvent::from_half_bit(!level, false))
                } else if abs_diff(duration, te * 2) < tolerance {
                    Some(ManchesterEvent::from_half_bit(!level, true))
                } else {
                    None
                };
                match event {
                    Some(event) => {
                        if let Some(bit) = self.manchester_state.advance(event) {
                            let carry = self.decode_data_low >> 31;
                            self.decode_data_low = (self.decode_data_low << 1) | bit as u32;
                            self.decode_data_high = (self.decode_data_high << 1) | carry;
                            self.decode_count_bit += 1;
                            if self.decode_count_bit == KEY1_BITS {
                                self.key1_low = self.decode_data_low;
                                self.key1_high = self.decode_data_high;
                                self.decode_data_low = 0;
                                self.decode_data_high = 0;
                            }
                        }
                        // A half-bit or a full bit is under 2.4 `te`: never the end HIGH.
                        if !end_high {
                            return None;
                        }
                    }
                    None => {
                        if !end_high {
                            self.invalid_pulses += 1;
                            if self.invalid_pulses > ADAPTIVE_INVALID_PULSES {
                                self.state = DecoderState::Reset;
                            }
                            return None;
                        }
                    }
                }
                // end_frame
                self.state = DecoderState::Reset;
                if self.decode_count_bit != KEY2_BITS || (self.key1_high >> 16) & 0xF != SYNC_NIBBLE {
                    return None;
                }
                let frame = RawFrame {
                    key1: ((self.key1_high as u64) << 32) | self.key1_low as u64,
                    key2_high: self.decode_data_high,
                    key2_low: self.decode_data_low,
                    am: true,
                };
                self.decode_data_low = 0;
                self.decode_data_high = 0;
                self.decode_count_bit = 0;
                Some(frame)
            }
        }
    }
}

/// PSA decoder: ProtoPirate's, Flipper-ARF's and doctor-firmware's two state machines run side by
/// side on every pulse.
/// Every frame the ProtoPirate machine completes is reported (PP's 0xA-sync gate and decrypt
/// routing). doctor-firmware's first machine completes a frame at its 80th bit, whatever follows it, on
/// its 250/500 µs path and its 125/250 µs path; such a frame waits for ProtoPirate's end pulse and,
/// if ProtoPirate's machine does not take it, must decrypt like ARF's AM-path frames (the 80th-bit
/// rule alone also reads the first 80 bits of longer Manchester frames, e.g. a VW Polo 5 remote's,
/// which then never decrypt). From the ARF machine only its adaptive-AM completions are used (its
/// 250/500 µs path is PP's, minus ARF's checksum-only acceptance). Separate machines keep each
/// one's state exactly as in its C file: ARF's AM State4 can stay stuck (count ≥ 121 with a 96-bit
/// window, `new_state = State0` never stored) for the rest of a stream, which in one shared machine
/// would also block ProtoPirate's 250/500 µs path. doctor-firmware's current machine
/// ([`AdaptiveMachine`]) completes on the end HIGH, like ProtoPirate's: a frame both read is
/// ProtoPirate's, one only it reads (a preamble of 16 to 70 LOWs, a clock away from 250 µs) must
/// decrypt. After a reported frame all four start again, as the registry restarts every decoder.
pub struct PsaDecoder {
    pp: Machine,
    doctor: Machine,
    arf: Machine,
    /// doctor-firmware's current machine ([AdaptiveMachine]). It completes a frame on its end
    /// HIGH, the pulse ProtoPirate's machine completes on, so a frame both read is reported as
    /// before; one it reads alone must decrypt, like the other added machines' frames.
    adaptive: AdaptiveMachine,
    /// A 250/500 µs frame the doctor machine completed at its 80th bit, waiting for ProtoPirate's
    /// end pulse: the first HIGH after it (or a LOW too long to be the last half-bit) decides.
    doctor_pending: Option<RawFrame>,
    /// KAT's additions to the C decoders: the doctor machine, the brute force on a fast-path miss
    /// and the fast-path decrypt gate. Always on; the C-oracle test driver turns them off to
    /// compare 1:1 with ProtoPirate's and ARF's C.
    kat_extensions: bool,
    /// `autobf`: AM-path frames the fast path cannot decrypt are reported as [UNCHECKED_NAME] for
    /// the key search to confirm, instead of being dropped.
    report_unchecked: bool,
}

impl PsaDecoder {
    pub fn new() -> Self {
        Self {
            pp: Machine::new(Flavor::ProtoPirate),
            doctor: Machine::new(Flavor::Doctor),
            arf: Machine::new(Flavor::Arf),
            adaptive: AdaptiveMachine::new(),
            doctor_pending: None,
            kat_extensions: true,
            report_unchecked: false,
        }
    }

    /// `psa_handle_decoded_frame` (ProtoPirate): fast-path decrypt with PP's routing. A frame the
    /// fast path rejects is a TEA frame: like the references, it is reported undecrypted (no
    /// serial/button/counter, not encoder-capable) and the brute force is left to a manual action
    /// (the signal menu's "Recover key", [`recover_key`]).
    ///
    /// AM-path frames count only when they decrypt: ARF's completion is weak (8 preamble cells,
    /// then any 80/88-bit window with the 0xA nibble at one of up to 9 offsets) and the registry's
    /// resets re-arm it; on IMPORTS/honda/Lock_honda.sub it accepts Honda Static chips plus
    /// inter-frame noise (the unmodified ARF psa.c, replayed with the registry's reset points,
    /// reports the same window), which decrypts neither by XOR nor by BF1/BF2. One the fast path
    /// cannot decrypt is dropped, or with `autobf` reported as [UNCHECKED_NAME] for the brute force
    /// to decide: a genuine TEA frame decrypts in a fraction of a second, that window never does.
    fn emit(&self, frame: RawFrame) -> Option<DecodedSignal> {
        let fields = fast_decrypt(frame.key1, frame.key2_low);
        let unchecked = frame.am && self.kat_extensions && fields.is_none();
        if unchecked && !self.report_unchecked {
            return None;
        }
        Some(DecodedSignal {
            serial: fields.map(|f| f.serial),
            button: fields.map(|f| f.button),
            counter: fields.map(|f| f.counter as u16),
            crc_valid: fields.is_some(),
            data: frame.key1,
            data_count_bit: MIN_COUNT_BIT,
            encoder_capable: fields.is_some(),
            extra: Some(((frame.key2_high as u64) << 32) | frame.key2_low as u64),
            protocol_display_name: unchecked.then(|| UNCHECKED_NAME.to_string()),
        })
    }
}

impl ProtocolDecoder for PsaDecoder {
    fn name(&self) -> &'static str {
        "PSA"
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming {
            te_short: TE_SHORT,
            te_long: TE_LONG,
            te_delta: TE_DELTA,
            min_count_bit: MIN_COUNT_BIT,
        }
    }

    fn supported_frequencies(&self) -> &[u32] {
        // SubGhzProtocolFlag_315 | SubGhzProtocolFlag_433 (all three references).
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.pp = Machine::new(Flavor::ProtoPirate);
        self.doctor = Machine::new(Flavor::Doctor);
        self.arf = Machine::new(Flavor::Arf);
        self.adaptive = AdaptiveMachine::new();
        self.doctor_pending = None;
    }

    /// The input ended while a doctor-firmware frame waited for ProtoPirate's end pulse: that pulse
    /// will not come, so doctor's rule alone read the frame (doctor-firmware reports it on its 80th
    /// bit), and it must decrypt like an AM-path frame.
    fn flush(&mut self) -> Option<DecodedSignal> {
        let pending = self.doctor_pending.take()?;
        let out = self.emit(RawFrame { am: true, ..pending });
        self.reset();
        out
    }

    fn set_report_unchecked(&mut self, on: bool) {
        self.report_unchecked = on;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        let mut frame = self.pp.feed(level, duration);
        let arf = self.arf.feed(level, duration).filter(|f| f.am);
        let mut adaptive = None;
        if self.kat_extensions {
            let doctor = self.doctor.feed(level, duration);
            adaptive = self.adaptive.feed(level, duration);
            // A pending doctor frame: ProtoPirate's end pulse is the first HIGH after the 80th bit
            // (a final 1 bit's LOW half comes first). Taken by ProtoPirate's machine, the frame is
            // reported as before; otherwise doctor's rule alone read it, and it must decrypt.
            if frame.is_some() {
                self.doctor_pending = None;
            } else if let Some(pending) = self.doctor_pending.take() {
                if level || duration > DOCTOR_PENDING_LOW_US {
                    frame = Some(RawFrame { am: true, ..pending });
                } else {
                    self.doctor_pending = Some(pending);
                }
            }
            match doctor {
                // 125/250 µs path: no ProtoPirate counterpart (its States 3/4 never complete).
                Some(f) if f.am => frame = frame.or(Some(f)),
                Some(f) => self.doctor_pending = Some(f),
                None => {}
            }
        }
        // A ProtoPirate frame wins a same-pulse tie; the adaptive machine's frame is taken when no
        // other machine completed one on this pulse. After a report all four machines restart (as
        // the registry restarts every decoder), so a frame is reported once.
        let out = frame.or(arf).or(adaptive).and_then(|frame| self.emit(frame));
        if out.is_some() && self.kat_extensions {
            self.reset();
        }
        out
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    /// ProtoPirate `subghz_protocol_encoder_psa_deserialize` + `psa_encoder_build_upload`. The
    /// references read Serial/Btn/Cnt/Type/CRC from the saved file; KAT re-derives them from the
    /// captured key1/key2 (+ the brute-forced serial for TEA frames), then sends counter + 1 with
    /// the requested button, 10 copies.
    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        let key1 = decoded.data;
        let key2_low = decoded.extra.unwrap_or(0) as u32;
        let fields = next_fields(decoded, button)?;
        let upload = build_upload(key1, key2_low, &fields);
        let mut signal = Vec::with_capacity(ENC_REPEAT * upload.len());
        for _ in 0..ENC_REPEAT {
            signal.extend_from_slice(&upload);
        }
        Some(signal)
    }

    /// The frame `encode` sends, as the decoder reports it (Flipper-ARF writes the same `Key` /
    /// `Key_2` back after building the upload). Needed because a TEA frame only decrypts under the
    /// capture's recovered serial: its fields are the ones encoded.
    fn sent_signal(&self, decoded: &DecodedSignal, button: u8) -> Option<DecodedSignal> {
        let fields = next_fields(decoded, button)?;
        let (key1, validation) = frame_words(decoded.data, decoded.extra.unwrap_or(0) as u32, &fields);
        let d = self.emit(RawFrame { key1, key2_high: 0, key2_low: u32::from(validation), am: false })?;
        Some(if d.serial.is_some() {
            d
        } else {
            DecodedSignal {
                serial: Some(fields.serial),
                button: Some(fields.button),
                counter: Some(fields.counter as u16),
                crc_valid: true,
                encoder_capable: true,
                ..d
            }
        })
    }

    /// Key files: ProtoPirate `subghz_protocol_decoder_psa_deserialize` (Key + Key_2, then
    /// `psa_decrypt_router`: PP's XOR fast path) gives the decoder's report for the frame. A TEA
    /// frame whose file carries the brute-force result (`Type` 0x36 + `Serial`, as PP's brute
    /// force patches it in and Flipper-ARF / doctor-firmware load it) is reported recovered when
    /// that serial's key decrypts it (`tea_decrypt_with_serial`, a 2^-32 check), as after the
    /// signal menu's "Recover key"; the other fields always come from the decrypt. Field values
    /// are decimal, except in Flipper-ARF's "PSA GROUP" / "PSA OLD" files (hex bytes).
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        let key1 = key.key_u64()?;
        let key2 = key.hex_field("Key_2")?;
        let frame = RawFrame { key1, key2_high: (key2 >> 32) as u32, key2_low: key2 as u32, am: false };
        let d = self.emit(frame)?;
        if d.serial.is_some() {
            return Some(d);
        }
        // Flipper-ARF ("PSA GROUP", and its retired psa2.c "PSA OLD") writes hex-byte fields.
        let arf = ["PSA GROUP", "PSA OLD"].iter().any(|n| key.protocol.trim().eq_ignore_ascii_case(n));
        let number = |name: &str| if arf { key.hex_field(name).map(|v| v as u32) } else { key.u32_field(name) };
        let recovered = (number("Type") == Some(u32::from(MODE_TEA)))
            .then(|| number("Serial"))
            .flatten()
            .and_then(|serial| tea_decrypt_with_serial(key1, key2 as u32, serial));
        Some(match recovered {
            Some((f, _)) => DecodedSignal {
                serial: Some(f.serial),
                button: Some(f.button),
                counter: Some(f.counter as u16),
                crc_valid: true,
                encoder_capable: true,
                ..d
            },
            None => d,
        })
    }

    /// Key files: ProtoPirate `subghz_protocol_decoder_psa_serialize`: Protocol "PSA", Bit 128,
    /// Key (key1), Key_2 (key2, 8 bytes), ValidationField (key2's low 16 bits), and for a decrypted
    /// frame Serial, Btn, Cnt (16-bit XOR / 28-bit TEA counter), Type (35 = 0x23 XOR, 54 = 0x36
    /// TEA), CRC and Seed (= serial), decimal. A TEA frame is written decrypted once its serial is
    /// known (after "Recover key"). None for captures without key2 (the retired PSA2 decoder's),
    /// which a key file cannot carry.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        let key2 = decoded.extra?;
        let (key1, key2_low) = (decoded.data, key2 as u32);
        let fields = fast_decrypt(key1, key2_low).or_else(|| {
            decoded.serial.and_then(|s| tea_decrypt_with_serial(key1, key2_low, s).map(|(f, _)| f))
        });
        let mut key = FlipperKey::for_protocol("PSA", MIN_COUNT_BIT, key1);
        key.key_word = true;
        key.push("Key_2", format!("{key2:016X}"));
        key.push_u32("ValidationField", key2_low & 0xFFFF);
        if let Some(f) = fields {
            key.push_u32("Serial", f.serial);
            key.push_u32("Btn", f.button.into());
            key.push_u32("Cnt", f.counter);
            key.push_u32("Type", f.mode.into());
            key.push_u32("CRC", f.crc.into());
            key.push_u32("Seed", f.serial);
        }
        Some(key)
    }
}

impl Default for PsaDecoder {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------------------------
// Encoder (psa.c psa_build_buffer_mode23 / _mode36 / psa_encoder_build_upload)
// ---------------------------------------------------------------------------------------------

/// KAT generic command → PSA button (ARF `psa_custom_to_btn` codes: 0 Lock, 1 Unlock, 2 Trunk);
/// anything else (Panic, unknown) keeps the captured button.
fn map_button(generic: u8, captured: u8) -> u8 {
    match generic {
        0x01 => 0x0,
        0x02 => 0x1,
        0x04 => 0x2,
        _ => captured,
    }
}

/// `psa_build_buffer_mode23`: XOR-encrypt b2..b7, searching b9 (or keeping the captured one) and
/// b8's high nibble for a checksum fixed point; fallback b9 = 0x23 (or the captured one).
fn build_buffer_mode23(fields: &PsaFields, key2_low: u32) -> [u8; 16] {
    let mut buf = [0u8; 16];
    let plain = [
        (fields.serial >> 16) as u8,
        (fields.serial >> 8) as u8,
        fields.serial as u8,
        (fields.counter >> 8) as u8,
        fields.counter as u8,
        fields.crc as u8,
    ];
    let initial_button = fields.button & 0xF;
    let has_original_key2 = key2_low != 0;
    let original_b9 = key2_low as u8;
    // C: `uint8_t buffer9_end = has ? original + 1 : 255;` (u8: original 0xFF gives an empty loop)
    let (b9_start, b9_end) =
        if has_original_key2 { (original_b9, original_b9.wrapping_add(1)) } else { (0u8, 255u8) };
    for b9 in b9_start..b9_end {
        for b8_high in 0u8..16 {
            buf[2..8].copy_from_slice(&plain);
            buf[8] = initial_button | (b8_high << 4);
            buf[9] = b9;
            second_stage_xor_encrypt(&mut buf);
            let checksum = calculate_checksum(&buf);
            if (checksum ^ buf[8]) & 0xF0 == 0 {
                buf[8] = (buf[8] & 0x0F) | (checksum & 0xF0);
                return buf;
            }
        }
    }
    buf[2..8].copy_from_slice(&plain);
    buf[8] = initial_button;
    buf[9] = if has_original_key2 { original_b9 } else { 0x23 };
    second_stage_xor_encrypt(&mut buf);
    let checksum = calculate_checksum(&buf);
    buf[8] = (buf[8] & 0x0F) | (checksum & 0xF0);
    buf
}

/// `psa_build_buffer_mode36`: TEA-encrypt the plaintext under the BF1 key of 0x23000000 | serial.
fn build_buffer_mode36(fields: &PsaFields) -> [u8; 16] {
    let mut v0 = ((fields.serial & 0x00FF_FFFF) << 8) | (((fields.button & 0xF) as u32) << 4) | ((fields.counter >> 24) & 0xF);
    let mut v1 = ((fields.counter & 0x00FF_FFFF) << 8) | (fields.crc as u32 & 0xFF);
    v1 = (v1 & 0xFFFF_FF00) | tea_crc(v0, v1) as u32;
    let key = bf1_working_key(&TeaSchedule::new(&BF1_KEY_SCHEDULE), BF1_START | (fields.serial & 0x00FF_FFFF));
    tea_encrypt(&mut v0, &mut v1, &key);
    let mut buf = [0u8; 16];
    buf[2..6].copy_from_slice(&v0.to_be_bytes());
    buf[6..10].copy_from_slice(&v1.to_be_bytes());
    buf
}

/// One upload copy of `psa_encoder_build_upload` (325 entries; the C yields it `repeat` times).
/// The fields `encode` sends for a capture: the plaintext the references read from the saved
/// file (Serial/Btn/Cnt/Type/CRC) — a TEA decrypt under the recovered serial's key (a 2^-32
/// check), else ProtoPirate's fast path — with the requested button and counter + 1.
fn next_fields(decoded: &DecodedSignal, button: u8) -> Option<PsaFields> {
    let key1 = decoded.data;
    let key2_low = decoded.extra.unwrap_or(0) as u32;
    let decrypted = decoded.extra.and_then(|_| {
        decoded
            .serial
            .and_then(|serial| tea_decrypt_with_serial(key1, key2_low, serial).map(|(f, _)| f))
            .or_else(|| fast_decrypt(key1, key2_low))
    });
    let fields = decrypted.or_else(|| {
        // No key material to decrypt (a capture saved without key2, e.g. by the retired
        // PSA2 decoder): mode 0x23 from the decoded fields, CRC byte 0.
        decoded.serial.map(|serial| PsaFields {
            mode: MODE_XOR,
            serial,
            button: decoded.button.unwrap_or(0),
            counter: decoded.counter.unwrap_or(0) as u32,
            crc: 0,
        })
    })?;
    Some(PsaFields { button: map_button(button, fields.button), counter: fields.counter.wrapping_add(1), ..fields })
}

/// Key1 and the 16-bit validation word of the frame for `fields` (`psa_encoder_build_upload`'s
/// buffer).
fn frame_words(key1: u64, key2_low: u32, fields: &PsaFields) -> (u64, u16) {
    let mut buf = if fields.mode == MODE_TEA { build_buffer_mode36(fields) } else { build_buffer_mode23(fields, key2_low) };
    if key1 != 0 {
        // buffer[0..1] preserved from the captured key1 (they carry the 0xA sync).
        buf[0] = (key1 >> 56) as u8;
        buf[1] = (key1 >> 48) as u8;
    } else {
        buf[0] = buf[2] ^ buf[6];
        buf[1] = buf[3] ^ buf[7];
    }
    let new_key1 = u64::from_be_bytes([buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]]);
    (new_key1, u16::from_be_bytes([buf[8], buf[9]]))
}

fn build_upload(key1: u64, key2_low: u32, fields: &PsaFields) -> Vec<LevelDuration> {
    let (new_key1, validation) = frame_words(key1, key2_low, fields);

    let mut up = Vec::with_capacity(ENC_UPLOAD_LEN);
    for _ in 0..ENC_PREAMBLE_PAIRS {
        up.push(LevelDuration::new(true, ENC_TE));
        up.push(LevelDuration::new(false, ENC_TE));
    }
    up.push(LevelDuration::new(false, ENC_TE));
    up.push(LevelDuration::new(true, TE_LONG));
    up.push(LevelDuration::new(false, ENC_TE));
    let bits = (0..64).rev().map(|i| (new_key1 >> i) & 1 == 1).chain((0..16).rev().map(|i| (validation >> i) & 1 == 1));
    for bit in bits {
        up.push(LevelDuration::new(bit, ENC_TE));
        up.push(LevelDuration::new(!bit, ENC_TE));
    }
    up.push(LevelDuration::new(true, ENC_END_HIGH));
    up.push(LevelDuration::new(false, TE_END_1000));
    up
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::export::flipper::import_sub_raw;
    use crate::protocols::ProtocolRegistry;
    use std::path::Path;

    fn decode_all(pulses: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = PsaDecoder::new();
        let mut out = Vec::new();
        for p in pulses {
            if let Some(d) = dec.feed(p.level, p.duration_us) {
                out.push(d);
                dec.reset();
            }
        }
        out
    }

    /// Adjacent same-level entries merged: what the upload looks like on air once demodulated.
    fn merged(signal: &[LevelDuration]) -> Vec<LevelDuration> {
        let mut out: Vec<LevelDuration> = Vec::new();
        for p in signal {
            match out.last_mut() {
                Some(last) if last.level == p.level => last.duration_us += p.duration_us,
                _ => out.push(LevelDuration::new(p.level, p.duration_us)),
            }
        }
        out
    }

    fn psa_from(key1: u64, key2: u64, serial: Option<u32>) -> DecodedSignal {
        DecodedSignal {
            serial,
            button: None,
            counter: None,
            crc_valid: false,
            data: key1,
            data_count_bit: MIN_COUNT_BIT,
            encoder_capable: false,
            extra: Some(key2),
            protocol_display_name: None,
        }
    }

    fn registry_decodes(rel: &str) -> Option<Vec<(String, DecodedSignal)>> {
        let path = Path::new("IMPORTS").join(rel);
        if !path.exists() {
            eprintln!("Skip: {:?} not found (run from crate root)", path);
            return None;
        }
        let (freq, raw) = import_sub_raw(&path).unwrap();
        let pairs: Vec<LevelDuration> = raw.iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect();
        let mut reg = ProtocolRegistry::new();
        Some(reg.process_signal_stream(&pairs, freq).into_iter().map(|(n, d, _)| (n, d)).collect())
    }

    /// Expected decodes of the genuine captures: key1/key2 and the XOR fields exactly as the
    /// unmodified ProtoPirate psa.c reports them, the TEA fields as its `psa_brute_force_run` finds
    /// them (host C harness): (key1, key2, serial, button, counter, crc).
    type Golden = (u64, u64, u32, u8, u32, u16);
    const PSA_523_536: [Golden; 2] = [
        (0x6EDA_E3B4_785F_7BC6, 0x70F6, 0x99EB25, 0, 0x039B, 0x40),
        (0xF47A_2EA1_1A4C_7CAF, 0x2927, 0x014590, 0, 0x9158E53, 0xD5),
    ];
    const PSA_536_523: [Golden; 2] = [
        (0xF47A_25DE_C980_F0DE, 0x1621, 0x014590, 0, 0x9158E55, 0xD7),
        (0x6EDA_D681_4A6A_49F3, 0x9023, 0x99EB25, 0, 0x039C, 0x40),
    ];

    #[test]
    fn decodes_groupe_psa_captures_like_protopirate() {
        // Two genuine fobs: an XOR (0x23) fob, decoded directly, and a TEA (0x36) fob, reported
        // undecrypted like the references do (the "Recover key" action brute-forces it — see
        // `brute_force_finds_the_tea_fob_like_protopirate`).
        for (rel, golden) in [("GROUPE PSA/PSA_523_536.sub", PSA_523_536), ("GROUPE PSA/PSA_536_523.sub", PSA_536_523)] {
            let Some(results) = registry_decodes(rel) else {
                return;
            };
            assert_eq!(results.len(), 2, "{rel}: {results:?}");
            for ((name, d), (key1, key2, serial, button, counter, crc)) in results.iter().zip(golden) {
                assert_eq!(name, "PSA", "{rel}");
                assert_eq!((d.data, d.extra, d.data_count_bit), (key1, Some(key2), 128), "{rel}");
                if fast_decrypt(key1, key2 as u32).is_none() {
                    // TEA frame: undecrypted until the key is recovered.
                    assert_eq!((d.serial, d.button, d.counter), (None, None, None), "{rel}");
                    assert!(!d.crc_valid && !d.encoder_capable, "{rel}");
                    assert!(tea_decrypt_with_serial(key1, key2 as u32, serial).is_some(), "{rel}");
                    continue;
                }
                assert_eq!((d.serial, d.button, d.counter), (Some(serial), Some(button), Some(counter as u16)), "{rel}");
                assert!(d.crc_valid && d.encoder_capable, "{rel}");
                let fields = fast_decrypt(key1, key2 as u32)
                    .or_else(|| tea_decrypt_with_serial(key1, key2 as u32, serial).map(|(f, _)| f))
                    .unwrap();
                assert_eq!((fields.counter, fields.crc), (counter, crc), "{rel}");
            }
        }
    }

    #[test]
    fn brute_force_finds_the_tea_fob_like_protopirate() {
        // PP psa_brute_force_run on the two genuine TEA frames: BF1, key counter 0x23014590.
        for (key1, key2, _, _, counter, crc) in [PSA_523_536[1], PSA_536_523[0]] {
            assert_eq!(fast_decrypt(key1, key2 as u32), None, "a TEA frame: the fast path rejects it");
            let ctl = BruteForceControl::default();
            let BruteForceOutcome::Found(hit) = brute_force(key1, key2 as u32, &ctl) else {
                panic!("no hit for {key1:016X}");
            };
            assert_eq!((hit.range, hit.key_counter), (1, 0x2301_4590));
            assert_eq!(hit.fields, PsaFields { mode: MODE_TEA, serial: 0x014590, button: 0, counter, crc });
            // Once the serial is known, decryption is O(1).
            assert_eq!(tea_decrypt_with_serial(key1, key2 as u32, 0x014590), Some((hit.fields, 1)));
        }
        // An undecryptable frame (the inverted-polarity artefact of PSA_523_536) is still emitted,
        // undecrypted, like the references do; the full miss is `brute_force_full_miss_is_not_found`.
    }

    /// Golden uploads from the unmodified ProtoPirate encoder (host C harness
    /// `upload key1 key2 serial btn cnt type crc`): (inputs, key1/validation it transmits).
    #[test]
    fn upload_matches_protopirate_encoder() {
        // (key1, key2, mode, serial, button, counter, crc) -> (tx key1, tx validation)
        type Inputs = (u64, u32, u8, u32, u8, u32, u16);
        let vectors: [(Inputs, (u64, u16)); 6] = [
            ((0x6EDA_E3B4_785F_7BC6, 0x70F6, 0x23, 0x99EB25, 0, 0x039C, 0x40), (0x6EDA_6334_FFDF_FC46, 0xF0F6)),
            ((0x6EDA_E3B4_785F_7BC6, 0x70F6, 0x23, 0x99EB25, 1, 0x039C, 0x40), (0x6EDA_6235_FEDE_FD47, 0xF1F6)),
            ((0xF47A_2EA1_1A4C_7CAF, 0x2927, 0x36, 0x014590, 0, 0x9158E54, 0xD5), (0xF47A_38D3_2DE3_24CE, 0xFC13)),
            ((0xF47A_2EA1_1A4C_7CAF, 0x2927, 0x36, 0x014590, 2, 0x9158E54, 0x00), (0xF47A_F904_D4AA_1D91, 0xCCCA)),
            ((0, 0, 0x23, 0x123456, 2, 0xBEEF, 0x77), (0x5126_1363_FC57_4245, 0x3200)),
            ((0x1234_5678_9ABC_DEF0, 0xFF, 0x23, 0xABCDEF, 1, 0x0001, 0x00), (0x1234_1198_1055_10FE, 0xC1FF)),
        ];
        for ((key1, key2, mode, serial, button, counter, crc), (tx_key1, tx_val)) in vectors {
            let fields = PsaFields { mode, serial, button, counter, crc };
            let up = build_upload(key1, key2, &fields);
            assert_eq!(up.len(), ENC_UPLOAD_LEN);
            let bits: Vec<bool> = up[163..323].chunks(2).map(|c| c[0].level).collect();
            let got_key1 = bits[..64].iter().fold(0u64, |w, &b| (w << 1) | b as u64);
            let got_val = bits[64..].iter().fold(0u16, |w, &b| (w << 1) | b as u16);
            assert_eq!((got_key1, got_val), (tx_key1, tx_val), "{fields:X?}");
        }
    }

    #[test]
    fn upload_layout_is_protopirate_upload() {
        let up = build_upload(0x6EDA_E3B4_785F_7BC6, 0x70F6, &PsaFields { mode: 0x23, serial: 0x99EB25, button: 0, counter: 0x39C, crc: 0x40 });
        assert_eq!(up.len(), 325);
        for (i, p) in up[..160].iter().enumerate() {
            assert_eq!((p.level, p.duration_us), (i % 2 == 0, 250));
        }
        let sync: Vec<(bool, u32)> = up[160..163].iter().map(|p| (p.level, p.duration_us)).collect();
        assert_eq!(sync, vec![(false, 250), (true, 500), (false, 250)]);
        for pair in up[163..323].chunks(2) {
            assert_eq!((pair[0].duration_us, pair[1].duration_us, pair[0].level), (250, 250, !pair[1].level));
        }
        // The end HIGH is the genuine fobs' 900 µs (the references: 1000 µs, see ENC_END_HIGH).
        assert_eq!((up[323].level, up[323].duration_us, up[324].level, up[324].duration_us), (true, 900, false, 1000));
    }

    #[test]
    fn encoder_reproduces_the_genuine_frames() {
        // From the TEA fob's first frame (PSA_523_536, counter 0x9158E53) the mode 0x36 encoder with
        // counter 0x9158E55 emits the fob's own later frame (PSA_536_523) bit for bit.
        let tea = PsaFields { mode: MODE_TEA, serial: 0x014590, button: 0, counter: 0x9158E55, crc: 0xD5 };
        let up = build_upload(0xF47A_2EA1_1A4C_7CAF, 0x2927, &tea);
        assert_eq!(decode_all(&merged(&up)).iter().map(|d| (d.data, d.extra)).collect::<Vec<_>>(), vec![(0xF47A_25DE_C980_F0DE, Some(0x1621))]);
        // The XOR fob's counter-0x39C frame (PSA_536_523), from its counter-0x39B frame with the
        // genuine b9 (0x23, also the C's fallback byte).
        let xor = PsaFields { mode: MODE_XOR, serial: 0x99EB25, button: 0, counter: 0x39C, crc: 0x40 };
        let up = build_upload(0x6EDA_E3B4_785F_7BC6, 0x0023, &xor);
        let got = decode_all(&merged(&up));
        assert_eq!((got[0].data, got[0].extra, got[0].counter), (0x6EDA_D681_4A6A_49F3, Some(0x9023), Some(0x039C)));
    }

    #[test]
    fn encode_round_trips_xor_and_tea_frames() {
        let dec = PsaDecoder::new();
        // XOR frame (the genuine counter-0x39B frame with key2's last bit set, CRC byte 0x41):
        // Unlock → button 1, counter + 1, 10 copies.
        let xor = psa_from(0x6EDA_E3B4_785F_7BC6, 0x70F7, Some(0x99EB25));
        let wire = dec.encode(&xor, 0x02).unwrap();
        assert_eq!(wire.len(), ENC_REPEAT * ENC_UPLOAD_LEN);
        let got = decode_all(&merged(&wire));
        assert_eq!(got.len(), ENC_REPEAT);
        for d in &got {
            assert_eq!((d.serial, d.button, d.counter), (Some(0x99EB25), Some(1), Some(0x039C)));
            assert_eq!(d.data >> 48, 0x6EDA, "buffer[0..1] (sync) preserved");
            assert_eq!(xor_decrypt(d.data, d.extra.unwrap() as u32).unwrap().crc, 0x41);
        }
        // TEA fob (genuine frame, serial from the brute force): Lock → button 0, counter + 1.
        let tea = psa_from(0xF47A_2EA1_1A4C_7CAF, 0x2927, Some(0x014590));
        let got = decode_all(&merged(&dec.encode(&tea, 0x01).unwrap()));
        assert_eq!(got.len(), ENC_REPEAT);
        for d in &got {
            assert_eq!((d.data, d.extra), (0xF47A_38D3_2DE3_24CE, Some(0xFC13)), "C upload");
            // TEA frames come out of the decoder undecrypted (the key is recovered on demand); the
            // fields are checked with the known serial (BF1, key counter 0x23014590).
            assert_eq!((d.serial, d.button, d.counter), (None, None, None));
            let (fields, range) = tea_decrypt_with_serial(d.data, d.extra.unwrap() as u32, 0x014590).unwrap();
            assert_eq!((fields.button, fields.counter, range), (0, 0x9158E54, 1));
        }
        // Panic (no PSA code) keeps the captured button; a TEA frame without a recovered serial
        // (brute-force miss) cannot be encoded.
        let got = decode_all(&merged(&dec.encode(&xor, 0x08).unwrap()));
        assert_eq!(got[0].button, Some(0));
        assert!(dec.encode(&psa_from(0xF47A_2EA1_1A4C_7CAF, 0x2927, None), 0x01).is_none());
    }

    #[test]
    fn encodes_captures_saved_without_key2() {
        // A capture of the retired PSA2 decoder: key1 and the decoded fields, no key2 (`extra`).
        let legacy = DecodedSignal { serial: Some(0x99EB25), button: Some(0), counter: Some(0x039B), crc_valid: true, ..psa_from(0x6EDA_E3B4_785F_7BC6, 0, None) };
        let legacy = DecodedSignal { extra: None, ..legacy };
        let wire = PsaDecoder::new().encode(&legacy, 0x02).unwrap();
        // ProtoPirate's mode 0x23 upload for (serial, Unlock, counter + 1, CRC 0), no original key2.
        let expected = build_upload(0x6EDA_E3B4_785F_7BC6, 0, &PsaFields { mode: MODE_XOR, serial: 0x99EB25, button: 1, counter: 0x039C, crc: 0 });
        assert_eq!(wire[..ENC_UPLOAD_LEN].iter().map(|p| (p.level, p.duration_us)).collect::<Vec<_>>(), expected.iter().map(|p| (p.level, p.duration_us)).collect::<Vec<_>>());
    }

    /// Frames decoded by the ProtoPirate and ARF machines alone (KAT's extensions off): the
    /// end-pulse rule (with KAT's 1250 µs window), without doctor-firmware's machine.
    fn decode_all_c_like(pulses: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = PsaDecoder { kat_extensions: false, ..PsaDecoder::new() };
        let mut out = Vec::new();
        for p in pulses {
            if let Some(d) = dec.feed(p.level, p.duration_us) {
                out.push(d);
                dec.reset();
            }
        }
        out
    }

    #[test]
    fn uploads_ending_in_a_zero_bit_decode() {
        // A trailing 0 bit's 250 µs HIGH half merges with the end HIGH. KAT's 900 µs end (the
        // genuine fobs') gives 1150 µs, inside the references' 801..1199 µs end window; the
        // references' 1000 µs end (a Flipper sending a saved file, KAT <= 1.8.3) gives 1250 µs,
        // which ProtoPirate's and ARF's decoders reject (host C harness) and KAT's end window
        // takes. doctor-firmware's completes the frame at its 80th bit, before any end pulse.
        let dec = PsaDecoder::new();
        let xor = psa_from(0x6EDA_E3B4_785F_7BC6, 0x70F6, Some(0x99EB25));
        let wire = dec.encode(&xor, 0x02).unwrap();
        assert!(!wire[321].level && wire[322].level, "last key2 bit is 0: (LOW, HIGH)");
        let sent = dec.sent_signal(&xor, 0x02).unwrap();
        let expected = Some((sent.data, sent.extra));
        let ours = merged(&wire);
        assert!(ours.iter().any(|p| p.level && p.duration_us == 1150), "merged end of KAT's upload");
        assert_eq!(decode_all(&ours).first().map(|d| (d.data, d.extra)), expected);
        assert_eq!(decode_all_c_like(&ours).first().map(|d| (d.data, d.extra)), expected);
        let references: Vec<LevelDuration> = wire
            .iter()
            .map(|p| if p.level && p.duration_us == ENC_END_HIGH { LevelDuration::new(true, 1000) } else { *p })
            .collect();
        let theirs = merged(&references);
        assert!(theirs.iter().any(|p| p.level && p.duration_us == 1250), "merged end of the references' upload");
        assert_eq!(decode_all(&theirs).first().map(|d| (d.data, d.extra)), expected);
        assert_eq!(decode_all_c_like(&theirs).first().map(|d| (d.data, d.extra)), expected);
        // Any end HIGH after the 80th bit (here 1450 µs): doctor's rule takes it (an XOR frame,
        // which decrypts), the end-pulse rule not.
        let late: Vec<LevelDuration> = merged(&wire[..ENC_UPLOAD_LEN])
            .into_iter()
            .map(|p| if p.level && p.duration_us == 1150 { LevelDuration::new(true, 1450) } else { p })
            .collect();
        assert_eq!(decode_all(&late).first().map(|d| (d.data, d.extra)), expected);
        assert!(decode_all_c_like(&late).is_empty());
    }

    /// One press at half-bit `te`: `pairs` preamble pairs, the sync (LOW te, HIGH 2te, LOW te),
    /// key1 + key2 as Manchester half-bits, then `end` (an end HIGH; None: nothing, the carrier
    /// stops after the last half-bit), between 10 ms gaps: how the tester's FSK fobs may end a frame.
    fn press(te: u32, key1: u64, key2: u16, pairs: usize, end: Option<u32>) -> Vec<LevelDuration> {
        let bits: Vec<bool> = (0..64).rev().map(|i| (key1 >> i) & 1 == 1).chain((0..16).rev().map(|i| (key2 >> i) & 1 == 1)).collect();
        press_bits(te, &bits, pairs, end)
    }

    /// [press] with any number of bits.
    fn press_bits(te: u32, bits: &[bool], pairs: usize, end: Option<u32>) -> Vec<LevelDuration> {
        let mut halves = vec![LevelDuration::new(false, 10_000)];
        for _ in 0..pairs {
            halves.extend([LevelDuration::new(true, te), LevelDuration::new(false, te)]);
        }
        halves.extend([LevelDuration::new(false, te), LevelDuration::new(true, 2 * te), LevelDuration::new(false, te)]);
        for &bit in bits {
            halves.extend([LevelDuration::new(bit, te), LevelDuration::new(!bit, te)]);
        }
        if let Some(end) = end {
            halves.push(LevelDuration::new(true, end));
        }
        halves.push(LevelDuration::new(false, 10_000));
        merged(&halves)
    }

    /// Frames of one decoder with `autobf` (fast-path TEA frames reported as [UNCHECKED_NAME]).
    fn decode_all_autobf(pulses: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = PsaDecoder::new();
        dec.set_report_unchecked(true);
        pulses.iter().filter_map(|p| dec.feed(p.level, p.duration_us)).collect()
    }

    /// doctor-firmware completes a 250 µs frame at its 80th bit (`psa_try_complete_at_key2`), so it
    /// reads frames whose end pulse is missing or of another length, and takes 70 preamble LOWs
    /// (`>=`, ProtoPirate `>`), checked against the unmodified doctor psa.c (host C harness). KAT
    /// reports what either reads. A frame ProtoPirate's end-pulse rule also takes is reported as
    /// before; one only doctor's rule reads must decrypt: an XOR frame at once, a TEA frame through
    /// the key search (`autobf`), as on the fast-timing paths.
    #[test]
    fn frames_complete_at_the_80th_bit_like_doctor_firmware() {
        const XOR: (u64, u16) = (0x6EDA_E3B4_785F_7BC6, 0x70F6); // last bit 0
        const TEA: (u64, u16) = (0xF47A_2EA1_1A4C_7CAF, 0x2927); // last bit 1
        let fields = |v: &[DecodedSignal]| v.first().map(|d| (d.data, d.extra, d.protocol_display_name.clone()));
        for (key1, key2) in [XOR, TEA] {
            let xor = key2 == XOR.1;
            let plain = Some((key1, Some(u64::from(key2)), None));
            let unchecked = Some((key1, Some(u64::from(key2)), Some(UNCHECKED_NAME.to_string())));
            // KAT's end window: 801..1449 µs (a final 0 bit's 250 µs HIGH half merges with the end).
            let end_rule = |end: Option<u32>| end.is_some_and(|e| (801..=1449).contains(&(if key2 & 1 == 0 { e + 250 } else { e })));
            for end in [None, Some(250), Some(500), Some(700), Some(900), Some(1300), Some(2000), Some(5000)] {
                let pulses = press(250, key1, key2, 80, end);
                let c_like = decode_all_c_like(&pulses);
                assert_eq!(c_like.is_empty(), !end_rule(end), "end rule {key1:X} end {end:?}");
                let (got, got_bf) = (decode_all(&pulses), decode_all_autobf(&pulses));
                if end_rule(end) || xor {
                    assert_eq!(fields(&got), plain, "{key1:X} end {end:?}");
                    assert_eq!(fields(&got_bf), plain, "{key1:X} end {end:?} autobf");
                } else {
                    assert!(got.is_empty(), "{key1:X} end {end:?}: a TEA frame waits for autobf");
                    assert_eq!(fields(&got_bf), unchecked, "{key1:X} end {end:?} autobf");
                }
            }
            // 71 preamble pairs: 70 LOWs before the sync (the last merges with it). doctor only.
            let short = press(250, key1, key2, 71, Some(900));
            assert!(decode_all_c_like(&short).is_empty());
            assert_eq!(fields(&decode_all_autobf(&short)), if xor { plain } else { unchecked }, "{key1:X} 70 LOWs");
            // 69 LOWs: not this machine. Without an end pulse nobody reads the frame; with one,
            // doctor-firmware's current machine does (`adaptive_frames_must_decrypt`).
            let mut old = Machine::new(Flavor::Doctor);
            assert_eq!(press(250, key1, key2, 70, Some(900)).iter().filter_map(|p| old.feed(p.level, p.duration_us)).count(), 0);
            assert!(decode_all_autobf(&press(250, key1, key2, 70, None)).is_empty(), "69 LOWs, no end pulse: nobody");
        }
        // Still only with the 0xA sync nibble.
        assert!(decode_all_autobf(&press(250, 0x6EDB_E3B4_785F_7BC6, 0x70F6, 80, None)).is_empty());
        // One report per frame, also without the registry's restarts: frames taken by ProtoPirate's
        // end rule (900 µs) and by doctor's alone (no end pulse).
        for end in [Some(900), None] {
            let mut dec = PsaDecoder::new();
            let three: Vec<LevelDuration> = (0..3).flat_map(|_| press(250, XOR.0, XOR.1, 80, end)).collect();
            assert_eq!(three.iter().filter_map(|p| dec.feed(p.level, p.duration_us)).count(), 3, "end {end:?}");
        }
    }

    /// A recording that ends right after a frame's 80th bit, before ProtoPirate's end pulse could
    /// come: doctor-firmware has reported the frame on that bit; KAT's doctor path, which waits one
    /// pulse for ProtoPirate's rule, reports it when the input ends ([ProtocolDecoder::flush]) and,
    /// as any frame only doctor's rule reads, only if it decrypts (or, with `autobf`, unchecked).
    #[test]
    fn flush_reports_a_doctor_frame_the_input_ends_after() {
        const XOR: (u64, u16) = (0x6EDA_E3B4_785F_7BC6, 0x70F6);
        const TEA: (u64, u16) = (0xF47A_2EA1_1A4C_7CAF, 0x2927);
        for (key1, key2) in [XOR, TEA] {
            let pulses = press(250, key1, key2, 80, None);
            // Drop the trailing silence (with a final 1 bit's LOW half merged into it).
            let cut = &pulses[..pulses.len() - 1];
            for autobf in [false, true] {
                let mut dec = PsaDecoder::new();
                dec.set_report_unchecked(autobf);
                let fed: Vec<DecodedSignal> = cut.iter().filter_map(|p| dec.feed(p.level, p.duration_us)).collect();
                let got = fed.into_iter().chain(dec.flush()).map(|d| (d.data, d.protocol_display_name)).collect::<Vec<_>>();
                let want = match (key2 == XOR.1, autobf) {
                    (true, _) => vec![(key1, None)],
                    (false, true) => vec![(key1, Some(UNCHECKED_NAME.to_string()))],
                    (false, false) => vec![],
                };
                assert_eq!(got, want, "{key1:X} autobf {autobf}");
                assert!(dec.flush().is_none());
            }
        }
    }

    /// doctor-firmware's 125/250 µs path (ProtoPirate's States 3/4 never complete a frame):
    /// ≥ 0x45 short pulses, a 250 µs one, Manchester at 125 µs, complete at the 80th bit. Its
    /// frames count like ARF's fast-path ones: an XOR frame at once, a TEA frame only through the
    /// key search (`autobf`). The end KAT's ARF path cannot take (a 125 µs end pulse) is covered.
    #[test]
    fn doctor_125_us_path_completes_at_the_80th_bit() {
        for end in [None, Some(125), Some(250), Some(450), Some(500), Some(2000)] {
            let xor = press(125, 0x6EDA_E3B4_785F_7BC6, 0x70F6, 80, end);
            let got = decode_all(&xor);
            assert_eq!(got.len(), 1, "XOR end {end:?}");
            assert_eq!((got[0].data, got[0].extra, got[0].serial), (0x6EDA_E3B4_785F_7BC6, Some(0x70F6), Some(0x99EB25)));

            let tea = press(125, 0xF47A_2EA1_1A4C_7CAF, 0x2927, 80, end);
            assert!(decode_all(&tea).is_empty(), "TEA end {end:?}: waits for autobf");
            let mut dec = PsaDecoder::new();
            dec.set_report_unchecked(true);
            let got: Vec<DecodedSignal> = tea.iter().filter_map(|p| dec.feed(p.level, p.duration_us)).collect();
            assert_eq!(got.len(), 1, "TEA end {end:?}");
            assert_eq!(got[0].protocol_display_name.as_deref(), Some(UNCHECKED_NAME));
            assert_eq!((got[0].data, got[0].extra), (0xF47A_2EA1_1A4C_7CAF, Some(0x2927)));
        }
        // The doctor machine alone: at least 0x45 short pulses before the 250 µs one (ARF's
        // adaptive path, which needs 8 cells, reads shorter preambles in KAT anyway).
        let doctor_frames = |pairs: usize| {
            let mut m = Machine::new(Flavor::Doctor);
            press(125, 0x6EDA_E3B4_785F_7BC6, 0x70F6, pairs, None).iter().filter_map(|p| m.feed(p.level, p.duration_us)).count()
        };
        // 36 pairs: 70 short pulses after the first HIGH, the last merged into the 250 µs one.
        assert_eq!((doctor_frames(36), doctor_frames(35)), (1, 0));
    }

    /// doctor-firmware's current machine alone ([AdaptiveMachine]; the same rules in the C: 862
    /// recordings give the same frames in `subghz_oracle_doctor`, see `docs/psa.md`): more than 15
    /// preamble LOWs, a half-bit measured on them, exactly 80 bits with the 0xA sync nibble, and an
    /// end HIGH of 3.2 half-bits or more.
    #[test]
    fn adaptive_machine_follows_doctor_firmware() {
        const XOR: (u64, u16) = (0x6EDA_E3B4_785F_7BC6, 0x70F6); // last bit 0: its HIGH half joins the end HIGH
        const TEA: (u64, u16) = (0xF47A_2EA1_1A4C_7CAF, 0x2927); // last bit 1
        let alone = |pulses: &[LevelDuration]| {
            let mut m = AdaptiveMachine::new();
            pulses.iter().filter_map(|p| m.feed(p.level, p.duration_us)).map(|f| (f.key1, f.key2_high, f.key2_low, f.am)).collect::<Vec<_>>()
        };
        for (key1, key2) in [XOR, TEA] {
            let one = vec![(key1, 0, u32::from(key2), true)];
            // 17 pairs: 16 preamble LOWs before the sync (the last one merges with it).
            assert_eq!(alone(&press(250, key1, key2, 17, Some(900))), one, "{key1:X}");
            assert!(alone(&press(250, key1, key2, 16, Some(900))).is_empty(), "{key1:X}: 15 LOWs");
            // The half-bit is the preamble's: 201..=299 µs on the 250 µs path (below, the sync's LOW
            // of two half-bits still counts as a preamble LOW; above, it is past the 500 ± 99 µs
            // window), 101..=149 µs on the 125 µs path.
            for te in [201, 220, 250, 280, 299, 101, 125, 149] {
                assert_eq!(alone(&press(te, key1, key2, 40, Some(te * 4))), one, "{key1:X} te {te}");
            }
            for te in [190, 200, 300, 310, 95, 100, 150, 160] {
                assert!(alone(&press(te, key1, key2, 40, Some(te * 4))).is_empty(), "{key1:X} te {te}");
            }
            // No frame without an end HIGH; no upper limit on it.
            assert!(alone(&press(250, key1, key2, 40, None)).is_empty(), "{key1:X}");
            assert_eq!(alone(&press(250, key1, key2, 40, Some(20_000))), one, "{key1:X}");
        }
        // The end HIGH: 3.2 half-bits (not merged with a half-bit here: the last bit is 1).
        assert_eq!(alone(&press(250, TEA.0, TEA.1, 40, Some(800))).len(), 1);
        assert!(alone(&press(250, TEA.0, TEA.1, 40, Some(799))).is_empty());
        assert_eq!(alone(&press(220, TEA.0, TEA.1, 40, Some(704))).len(), 1);
        assert!(alone(&press(220, TEA.0, TEA.1, 40, Some(703))).is_empty());
        // Exactly 80 bits, and the sync nibble.
        let bits = |n: usize| (0..n).map(|i| if i < 64 { (XOR.0 >> (63 - i)) & 1 == 1 } else { i % 3 == 0 }).collect::<Vec<_>>();
        assert_eq!(alone(&press_bits(250, &bits(80), 40, Some(900))).len(), 1);
        for n in [64, 79, 81, 88, 130] {
            assert!(alone(&press_bits(250, &bits(n), 40, Some(900))).is_empty(), "{n} bits");
        }
        assert!(alone(&press(250, 0x6EDB_E3B4_785F_7BC6, 0x70F6, 40, Some(900))).is_empty());
        // Two pulses outside both symbol windows are let through, the third rewinds.
        let mut m = AdaptiveMachine::new();
        let preamble: Vec<LevelDuration> = press(250, XOR.0, XOR.1, 40, None).into_iter().take(1 + 2 * 39 + 2).collect();
        assert!(preamble.iter().all(|p| m.feed(p.level, p.duration_us).is_none()));
        assert_eq!((m.state, m.timing_short), (DecoderState::Data250, 250));
        for (i, level) in [true, false, true].into_iter().enumerate() {
            assert!(m.feed(level, 370).is_none());
            assert_eq!(m.state, if i < 2 { DecoderState::Data250 } else { DecoderState::Reset }, "invalid pulse {i}");
        }
    }

    /// What the decoder makes of the adaptive machine's frames. One ProtoPirate's machine reads
    /// too is reported as before (ProtoPirate's frame wins the pulse they both complete on); one
    /// only the adaptive machine reads (here: 40 preamble pairs, where ProtoPirate wants 71 LOWs
    /// and doctor-firmware's earlier machine 70) must decrypt: an XOR frame at once, a TEA frame
    /// through the key search (`autobf`).
    #[test]
    fn adaptive_frames_must_decrypt() {
        const XOR: (u64, u16) = (0x6EDA_E3B4_785F_7BC6, 0x70F6);
        const TEA: (u64, u16) = (0xF47A_2EA1_1A4C_7CAF, 0x2927);
        let fields = |v: &[DecodedSignal]| v.iter().map(|d| (d.data, d.extra, d.serial, d.protocol_display_name.clone())).collect::<Vec<_>>();
        let short = |k: (u64, u16)| press(250, k.0, k.1, 40, Some(900));
        assert!(decode_all_c_like(&short(XOR)).is_empty() && decode_all_c_like(&short(TEA)).is_empty());
        let xor = vec![(XOR.0, Some(u64::from(XOR.1)), Some(0x99EB25), None)];
        assert_eq!(fields(&decode_all(&short(XOR))), xor);
        assert_eq!(fields(&decode_all_autobf(&short(XOR))), xor);
        assert!(decode_all(&short(TEA)).is_empty(), "a TEA frame waits for autobf");
        assert_eq!(fields(&decode_all_autobf(&short(TEA))), vec![(TEA.0, Some(u64::from(TEA.1)), None, Some(UNCHECKED_NAME.to_string()))]);
        // With ProtoPirate's preamble the TEA frame is ProtoPirate's: listed undecrypted, not unchecked.
        let full = press(250, TEA.0, TEA.1, 80, Some(900));
        let plain = vec![(TEA.0, Some(u64::from(TEA.1)), None, None)];
        assert_eq!(fields(&decode_all(&full)), plain);
        assert_eq!(fields(&decode_all_autobf(&full)), plain);
        // One report per frame, also without the registry's restarts.
        for pairs in [40, 80] {
            let mut dec = PsaDecoder::new();
            let three: Vec<LevelDuration> = (0..3).flat_map(|_| press(250, XOR.0, XOR.1, pairs, Some(900))).collect();
            assert_eq!(three.iter().filter_map(|p| dec.feed(p.level, p.duration_us)).count(), 3, "{pairs} pairs");
        }
    }

    /// `Peugeot 307 open.sub`: six presses of a TEA fob (serial 0x02F0AB), the last one recorded
    /// with 61 of its preamble's LOWs. ProtoPirate's machine and doctor-firmware's earlier one want
    /// 71 and 70, so up to v1.12.0 KAT listed five presses; doctor-firmware's current machine reads
    /// the sixth (as its C does), and the key search confirms it: the same fob, the counter two on
    /// from the fifth press's.
    #[test]
    fn peugeot_307_sixth_press_is_read_by_the_adaptive_machine() {
        let path = Path::new("IMPORTS/GROUPE PSA/Peugeot 307 open.sub");
        if !path.exists() {
            eprintln!("Skip: {path:?} not found (run from crate root)");
            return;
        }
        let (freq, pairs) = crate::protocols::f_common::test_support::sub_pulses(path);
        let mut reg = ProtocolRegistry::new();
        let psa = |frames: Vec<(String, DecodedSignal, Vec<LevelDuration>)>| {
            frames.into_iter().filter(|(n, _, _)| n.starts_with("PSA")).map(|(n, d, _)| (n, d.data, d.extra)).collect::<Vec<_>>()
        };
        let without = psa(reg.process_signal_stream(&pairs, freq));
        assert!(!without.iter().any(|f| f.1 == 0xFF5A_2D95_18BC_2522), "a TEA frame only the adaptive machine reads waits for autobf");
        reg.set_auto_key_search(true);
        let with = psa(reg.process_signal_stream(&pairs, freq));
        // The frame before it is the damaged press between the fifth and the sixth (key2 all ones):
        // listed unchecked as before, and dropped by the key search, which finds no key for it.
        assert_eq!(with.len(), 7, "{with:X?}");
        assert!(with[..5].iter().all(|f| f.0 == "PSA"), "{with:X?}");
        assert_eq!(with[5], (UNCHECKED_NAME.to_string(), 0xFF5A_7C3F_9F96_0FFF, Some(0xFFFF)));
        assert_eq!(with[6], (UNCHECKED_NAME.to_string(), 0xFF5A_2D95_18BC_2522, Some(0xF28A)));
        let BruteForceOutcome::Found(hit) = recover_key(with[6].1, 0xF28A, &BruteForceControl::default()) else {
            panic!("no key for the sixth press");
        };
        assert_eq!((hit.range, hit.fields.serial, hit.fields.button, hit.fields.counter & 0xFFFF), (1, 0x02F0AB, 1, 0xD38C));
        let BruteForceOutcome::Found(fifth) = recover_key(with[4].1, with[4].2.unwrap(), &BruteForceControl::default()) else {
            panic!("no key for the fifth press");
        };
        assert_eq!((fifth.fields.serial, fifth.fields.counter + 2), (0x02F0AB, hit.fields.counter));
    }

    /// `sent_signal` is the decoder's report of the frame `encode` sends (what the app keeps on a
    /// capture after transmitting it), also for TEA frames (the encoded fields, as the recovered
    /// serial decrypts them).
    #[test]
    fn sent_signal_is_the_frame_encoded() {
        let dec = PsaDecoder::new();
        // A decodable upload: the same as its decode.
        let xor = psa_from(0x6EDA_E3B4_785F_7BC6, 0x70F7, Some(0x99EB25));
        let sent = dec.sent_signal(&xor, 0x02).unwrap();
        assert_eq!(Some(&sent), decode_all(&merged(&dec.encode(&xor, 0x02).unwrap())).first());
        // Ending in a 0 bit (no decode): Unlock → button 1, counter + 1, and the next press goes on.
        let xor0 = psa_from(0x6EDA_E3B4_785F_7BC6, 0x70F6, Some(0x99EB25));
        let sent = dec.sent_signal(&xor0, 0x02).unwrap();
        assert_eq!((sent.serial, sent.button, sent.counter, sent.crc_valid), (Some(0x99EB25), Some(1), Some(0x039C), true));
        assert_eq!(dec.sent_signal(&sent, 0x02).unwrap().counter, Some(0x039D));
        // TEA (serial from the brute force): the C's upload with the encoded fields; a second
        // press from it is the fob's own later frame (PSA_536_523, counter 0x9158E55).
        let tea = psa_from(0xF47A_2EA1_1A4C_7CAF, 0x2927, Some(0x014590));
        let sent = dec.sent_signal(&tea, 0x01).unwrap();
        assert_eq!((sent.data, sent.extra), (0xF47A_38D3_2DE3_24CE, Some(0xFC13)));
        assert_eq!((sent.serial, sent.button, sent.counter, sent.encoder_capable), (Some(0x014590), Some(0), Some(0x8E54), true));
        let next = dec.sent_signal(&sent, 0x01).unwrap();
        assert_eq!((next.data, next.extra, next.counter), (0xF47A_25DE_C980_F0DE, Some(0x1621), Some(0x8E55)));
    }

    #[test]
    fn rejects_frames_without_the_sync_nibble() {
        // A valid XOR frame whose b1 low nibble is 0xB instead of the 0xA sync: no decode.
        let fields = PsaFields { mode: 0x23, serial: 0x99EB25, button: 0, counter: 0x39C, crc: 0x40 };
        let good = build_upload(0x6EDA_E3B4_785F_7BC6, 0x70F7, &fields);
        assert_eq!(decode_all(&merged(&good)).len(), 1);
        let bad = build_upload(0x6EDB_E3B4_785F_7BC6, 0x70F7, &fields);
        assert!(decode_all(&merged(&bad)).is_empty());
    }

    #[test]
    fn bf2_frames_are_recovered() {
        // A BF2 frame built from the reference crypto (no reference encoder emits one): plaintext
        // whose CRC-16 sits in b8..b9 (PP's comparison), encrypted under the BF2 key of serial S.
        let serial = 0x00_0102;
        let mut v0 = (serial << 8) | (1 << 4) | 0x9;
        let v1_hi = 0x1234u32;
        let crc = crc16_bf2(&[(v0 >> 24) as u8, (v0 >> 8) as u8, (v0 >> 16) as u8, v0 as u8, (v1_hi >> 8) as u8, v1_hi as u8]);
        let mut v1 = (v1_hi << 16) | crc as u32;
        let (p0, p1) = (v0, v1);
        tea_encrypt(&mut v0, &mut v1, &bf2_working_key(BF2_START | serial));
        let key1 = (0xF47Au64 << 48) | ((v0 as u64) << 16) | (v1 >> 16) as u64;
        let key2 = v1 & 0xFFFF;
        assert_eq!(fast_decrypt(key1, key2), None);
        let (fields, range) = tea_decrypt_with_serial(key1, key2, serial).unwrap();
        assert_eq!((fields, range), (tea_fields(p0, p1), 2));
        // The BF2 search finds it (the BF1 range is covered by `brute_force_full_miss_is_not_found`).
        let ctl = BruteForceControl::default();
        let (w0, w1) = prepare_tea_data(&setup_byte_buffer(key1, key2));
        let hit = search_range(
            |off| {
                let c = BF2_START + off;
                let (mut a, mut b) = (w0, w1);
                tea_decrypt(&mut a, &mut b, &bf2_working_key(c));
                bf2_accepts(c, a, b)
            },
            &ctl,
        );
        assert_eq!(hit, Ok(Some(serial)));
    }

    #[test]
    fn parallel_search_returns_the_lowest_hit_and_honours_cancel() {
        let ctl = BruteForceControl::default();
        let hits = [5 * BF_CHUNK + 7, 3 * BF_CHUNK + 1, 3 * BF_CHUNK + 900, BF_RANGE - 1];
        assert_eq!(search_range(|off| hits.contains(&off), &ctl), Ok(Some(3 * BF_CHUNK + 1)));
        assert_eq!(search_range(|off| off == BF_RANGE - 1, &ctl), Ok(Some(BF_RANGE - 1)));
        let cancelled = BruteForceControl::default();
        cancelled.cancel.store(true, Ordering::Relaxed);
        assert_eq!(search_range(|_| false, &cancelled), Err(()));
        assert_eq!(brute_force(0, 0, &cancelled), BruteForceOutcome::Cancelled);
    }

    /// Full 2 × 2^24 miss (slow in a debug build): `cargo test --release brute_force_full -- --ignored`.
    #[test]
    #[ignore]
    fn brute_force_full_miss_is_not_found() {
        let ctl = BruteForceControl::default();
        let t = std::time::Instant::now();
        // The inverted-polarity Chrysler/PSA artefact the PP harness decodes: no key in either range.
        assert_eq!(brute_force(0x224A_3897_0F41_0873, 0x1E13, &ctl), BruteForceOutcome::NotFound);
        eprintln!("full PSA brute-force miss: {:?} ({} candidates)", t.elapsed(), ctl.progress.load(Ordering::Relaxed));
        assert_eq!(ctl.progress.load(Ordering::Relaxed), 2 * BF_RANGE);
    }

    /// C-oracle driver (`docs/psa.md`, "Verification"): `KAT_PSA_ORACLE=<file>` with lines
    /// `P <level> <duration>` (one stream, fed without resets like the C harness; level 2 = reset), `U key1 key2
    /// mode serial button counter crc` (one upload copy) or `E key1 key2 serial|- button` (full
    /// `encode()`: derived fields + first upload copy). Prints `D`/`U`/`E` lines for diffing.
    #[test]
    #[ignore]
    fn oracle_dump() {
        let Ok(path) = std::env::var("KAT_PSA_ORACLE") else {
            return;
        };
        let hex = |s: &str| u64::from_str_radix(s.trim_start_matches("0x"), 16).unwrap();
        let mut dec = PsaDecoder { kat_extensions: false, ..PsaDecoder::new() };
        let mut idx = 0usize;
        let print_upload = |up: &[LevelDuration]| {
            for p in up {
                println!("{} {}", p.level as u8, p.duration_us);
            }
        };
        for line in std::fs::read_to_string(path).unwrap().lines() {
            let f: Vec<&str> = line.split_whitespace().collect();
            match f.first().copied() {
                Some("P") if f[1] == "2" => {
                    dec.reset();
                    idx += 1;
                }
                Some("P") => {
                    if let Some(s) = dec.feed(f[1] == "1", f[2].parse().unwrap()) {
                        let key2 = s.extra.unwrap();
                        match fast_decrypt(s.data, key2 as u32) {
                            Some(x) => println!(
                                "D {idx} {:016X} {:016X} 50 {:06X} {:X} {:X} {:X}",
                                s.data, key2, x.serial, x.button, x.counter, x.crc
                            ),
                            None => println!("D {idx} {:016X} {:016X} 00", s.data, key2),
                        }
                    }
                    idx += 1;
                }
                Some("U") => {
                    let v: Vec<u64> = f[1..].iter().map(|s| hex(s)).collect();
                    let fields = PsaFields { mode: v[2] as u8, serial: v[3] as u32, button: v[4] as u8, counter: v[5] as u32, crc: v[6] as u16 };
                    println!("U");
                    print_upload(&build_upload(v[0], v[1] as u32, &fields));
                }
                Some("B") => {
                    let (key1, key2) = (hex(f[1]), hex(f[2]));
                    let t = std::time::Instant::now();
                    let out = brute_force(key1, key2 as u32, &BruteForceControl::default());
                    let ms = t.elapsed().as_millis();
                    match out {
                        BruteForceOutcome::Found(h) => println!(
                            "B {key1:016X} {:04X} status=2 ser={:06X} btn={:X} cnt={:X} crc={:X} type=36 range={} ms={ms}",
                            key2 & 0xFFFF, h.fields.serial, h.fields.button, h.fields.counter, h.fields.crc, h.range
                        ),
                        _ => println!("B {key1:016X} {:04X} status=3 ser=000000 btn=0 cnt=0 crc=0 type=00 ms={ms}", key2 & 0xFFFF),
                    }
                }
                Some("E") => {
                    let serial = (f[3] != "-").then(|| hex(f[3]) as u32);
                    let d = psa_from(hex(f[1]), hex(f[2]), serial);
                    match dec.encode(&d, hex(f[4]) as u8) {
                        Some(wire) => {
                            println!("E");
                            print_upload(&wire[..ENC_UPLOAD_LEN]);
                        }
                        None => println!("E none"),
                    }
                }
                _ => {}
            }
        }
    }

    /// Parity tool for doctor-firmware's current machine ([AdaptiveMachine]): `F_FILES=<list file>`
    /// (one RAW `.sub` per line) prints every frame that machine alone completes as `<file> | <key1>
    /// <key2>`, for comparing with `subghz_oracle_doctor decode --protocol PSA` (`docs/psa.md`).
    #[test]
    #[ignore]
    fn adaptive_machine_dump() {
        let Ok(list) = std::env::var("F_FILES") else {
            return;
        };
        for file in std::fs::read_to_string(list).unwrap().lines().filter(|l| !l.trim().is_empty()) {
            let (_, wire) = crate::protocols::f_common::test_support::sub_pulses(Path::new(file));
            let mut machine = AdaptiveMachine::new();
            let mut any = false;
            for p in &wire {
                if let Some(f) = machine.feed(p.level, p.duration_us) {
                    any = true;
                    println!("{file} | {:016X} {:08X}{:08X}", f.key1, f.key2_high, f.key2_low);
                }
            }
            if !any {
                println!("{file} | (none)");
            }
        }
    }

    /// An ARF-style AM (OOK) frame at half-bit `te`: State0 HIGH, 12 preamble cells, a boundary
    /// cell (HIGH te + LOW 3·te), one leading bit the Manchester sync swallows, key1 + key2, then a
    /// terminator. (The same construction decodes in the unmodified ARF psa.c harness.)
    fn am_frame(te: u32, key1: u64, key2: u16) -> Vec<LevelDuration> {
        let mut halves = vec![(true, 100), (false, te)];
        for _ in 0..12 {
            halves.extend([(true, te), (false, te)]);
        }
        halves.extend([(true, te), (false, 3 * te)]);
        let bits = std::iter::once(true).chain((0..64).rev().map(|i| (key1 >> i) & 1 == 1)).chain((0..16).rev().map(|i| (key2 >> i) & 1 == 1));
        for bit in bits {
            halves.extend([(bit, te), (!bit, te)]);
        }
        let mut wire = merged(&halves.iter().map(|&(l, d)| LevelDuration::new(l, d)).collect::<Vec<_>>());
        let last = wire.last().unwrap().level;
        wire.push(LevelDuration::new(!last, 6 * te));
        wire.push(LevelDuration::new(last, 20_000));
        wire
    }

    #[test]
    fn am_path_decodes_adaptive_te_frames() {
        // The XOR fob's frame sent as an AM frame at three half-bit widths ARF's path adapts to.
        for te in [70, 100, 160] {
            let got = decode_all(&am_frame(te, 0x6EDA_E3B4_785F_7BC6, 0x70F6));
            assert_eq!(got.len(), 1, "te {te}");
            assert_eq!((got[0].data, got[0].extra), (0x6EDA_E3B4_785F_7BC6, Some(0x70F6)), "te {te}");
            assert_eq!((got[0].serial, got[0].button, got[0].counter), (Some(0x99EB25), Some(0), Some(0x039B)), "te {te}");
        }
        // The same window without the 0xA sync nibble is rejected by ARF's own check.
        assert!(decode_all(&am_frame(100, 0x6EDB_E3B4_785F_7BC6, 0x70F6)).is_empty());
    }

    #[test]
    fn am_path_requires_a_decrypt() {
        // ARF's AM path (re-armed by the registry's resets) accepts this Honda Static capture's
        // inter-frame noise as 4CDA57A60039A20E/4FF4, which decrypts neither by XOR nor by
        // BF1/BF2. The C-equivalent decoder still sees it; the registry must not report it, and
        // every Honda frame (now Honda V5) must still decode.
        let Some(results) = registry_decodes("honda/Lock_honda.sub") else {
            return;
        };
        let names: Vec<&str> = results.iter().map(|(n, _)| n.as_str()).collect();
        // 23 Honda V5 frames; the one frame V5's CRC rejects (counter 0x56FE21 in the Honda Static
        // reading) still comes through Honda Static's unchecked path, between the 11th and 12th.
        let mut want = vec!["Honda V5"; 11];
        want.push("Honda Static");
        want.extend(vec!["Honda V5"; 12]);
        assert_eq!(names, want);
        let mut c_like = PsaDecoder { kat_extensions: false, ..PsaDecoder::new() };
        let (freq, raw) = import_sub_raw(Path::new("IMPORTS/honda/Lock_honda.sub")).unwrap();
        assert_eq!(freq, 433_657_070);
        // Replay the registry's reset after the 20th Honda Static frame (pulse 40398).
        let hit = raw[40_399..]
            .iter()
            .find_map(|p| c_like.feed(p.level, p.duration_us))
            .expect("the C-equivalent AM path fires");
        assert_eq!((hit.data, hit.extra, hit.serial), (0x4CDA_57A6_0039_A20E, Some(0x4FF4), None));
    }

    /// The tester's case: a genuine TEA frame on ARF's adaptive AM path (the TEA fob's frame at
    /// half-bit widths only that path takes). The fast path cannot decrypt it, so it is dropped as
    /// before unless `autobf` asks for unchecked frames; then it is reported as `PSA [unchecked]`
    /// and the key search turns it into the fob's frame (serial 0x014590, BF1).
    #[test]
    fn am_path_tea_frames_wait_for_their_key() {
        for te in [70, 100, 160] {
            let pulses = am_frame(te, 0xF47A_2EA1_1A4C_7CAF, 0x2927);
            assert!(decode_all(&pulses).is_empty(), "te {te}: dropped without autobf");
            let mut dec = PsaDecoder::new();
            dec.set_report_unchecked(true);
            let got: Vec<DecodedSignal> = pulses.iter().filter_map(|p| dec.feed(p.level, p.duration_us)).collect();
            assert_eq!(got.len(), 1, "te {te}");
            assert_eq!(got[0].protocol_display_name.as_deref(), Some(UNCHECKED_NAME));
            assert_eq!((got[0].data, got[0].extra, got[0].serial), (0xF47A_2EA1_1A4C_7CAF, Some(0x2927), None));
            let AutoKeySearch::Recovered(found, how) = auto_key_search(UNCHECKED_NAME, &got[0]) else {
                panic!("te {te}: no key");
            };
            assert_eq!((found.serial, found.button, found.protocol_display_name, how.as_str()), (Some(0x014590), Some(0), None, "PSA BF1"));
            assert!(found.encoder_capable && found.crc_valid);
        }
    }

    /// With `autobf` the registry reports the Honda noise window as `PSA [unchecked]`, and the key
    /// search drops it (it decrypts neither by XOR nor by BF1/BF2); the Honda frames stay as they are.
    #[test]
    fn unchecked_noise_is_dropped_by_the_key_search() {
        let path = Path::new("IMPORTS/honda/Lock_honda.sub");
        if !path.exists() {
            eprintln!("Skip: {path:?} not found (run from crate root)");
            return;
        }
        let (freq, raw) = import_sub_raw(path).unwrap();
        let pairs: Vec<LevelDuration> = raw.iter().map(|p| LevelDuration::new(p.level, p.duration_us)).collect();
        let mut reg = ProtocolRegistry::new();
        reg.set_auto_key_search(true);
        let got = reg.process_signal_stream(&pairs, freq);
        let unchecked: Vec<&DecodedSignal> = got.iter().filter(|(n, _, _)| n == UNCHECKED_NAME).map(|(_, d, _)| d).collect();
        assert_eq!(unchecked.len(), 1);
        assert_eq!((unchecked[0].data, unchecked[0].extra), (0x4CDA_57A6_0039_A20E, Some(0x4FF4)));
        assert_eq!(auto_key_search(UNCHECKED_NAME, unchecked[0]), AutoKeySearch::Dropped);
        assert_eq!(got.iter().filter(|(n, _, _)| n == "Honda V5").count(), 23);
        assert_eq!(got.iter().filter(|(n, _, _)| n == "Honda Static").count(), 1);
        // An undecrypted listed frame keeps its decode, marked as searched.
        let tea = DecodedSignal { serial: None, ..unchecked[0].clone() };
        assert_eq!(auto_key_search("PSA", &tea), AutoKeySearch::NotFound(SEARCH_LABEL_ALL.to_string()));
        assert_eq!(auto_key_search("Kia V0", &tea), AutoKeySearch::NotNeeded);
    }

    #[test]
    fn button_mapping_uses_psa_codes() {
        assert_eq!(map_button(0x01, 2), 0);
        assert_eq!(map_button(0x02, 2), 1);
        assert_eq!(map_button(0x04, 0), 2);
        assert_eq!(map_button(0x08, 2), 2);
    }

    /// Key files (ProtoPirate `subghz_protocol_decoder_psa_serialize` / `_deserialize`): the XOR
    /// and TEA frames of the genuine fobs round-trip with PP's lines (decrypted XOR, undecrypted
    /// TEA, TEA after "Recover key"); PP's own file (16-digit words) and Flipper-ARF's "PSA GROUP"
    /// file (hex-byte fields) load the recovered TEA frame; a serial that does not decrypt leaves
    /// it undecrypted; TX from a loaded key is the C's upload.
    #[test]
    fn flipper_key_files() {
        use crate::protocols::flipper_key::testing::{assert_round_trip, key};
        let dec = PsaDecoder::new();
        let frame = |key1: u64, key2: u64| RawFrame { key1, key2_high: 0, key2_low: key2 as u32, am: false };
        let (key1, key2, ..) = PSA_523_536[0];
        let xor = dec.emit(frame(key1, key2)).unwrap();
        assert!(assert_round_trip(&dec, &xor).ends_with(
            "Protocol: PSA\nBit: 128\nKey: 6EDAE3B4785F7BC6\nKey_2: 00000000000070F6\n\
             ValidationField: 28918\nSerial: 10087205\nBtn: 0\nCnt: 923\nType: 35\nCRC: 64\nSeed: 10087205\n"
        ));
        let (key1, key2, serial, ..) = PSA_523_536[1];
        let tea = dec.emit(frame(key1, key2)).unwrap();
        assert_eq!(tea.serial, None);
        assert!(assert_round_trip(&dec, &tea).ends_with("Key: F47A2EA11A4C7CAF\nKey_2: 0000000000002927\nValidationField: 10535\n"));
        // After the signal menu's "Recover key" (fields from the brute force, as app.rs stores them).
        let (f, _) = tea_decrypt_with_serial(key1, key2 as u32, serial).unwrap();
        let recovered = DecodedSignal {
            serial: Some(f.serial),
            button: Some(f.button),
            counter: Some(f.counter as u16),
            crc_valid: true,
            encoder_capable: true,
            ..tea.clone()
        };
        assert!(assert_round_trip(&dec, &recovered)
            .ends_with("Serial: 83344\nBtn: 0\nCnt: 152407635\nType: 54\nCRC: 213\nSeed: 83344\n"));
        let header = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
                      Preset: FuriHalSubGhzPresetOok650Async\n";
        let pp = format!(
            "{header}Protocol: PSA\nBit: 128\nKey: F47A2EA11A4C7CAF\nKey_2: 0000000000002927\n\
             ValidationField: 10535\nSerial: 83344\nBtn: 0\nCnt: 152407635\nType: 54\nCRC: 213\nSeed: 83344\n"
        );
        assert_eq!(dec.from_flipper_key(&key(&pp)), Some(recovered.clone()));
        let arf = format!(
            "{header}Protocol: PSA GROUP\nBit: 128\nKey: F4 7A 2E A1 1A 4C 7C AF\n\
             Key_2: 00 00 00 00 00 00 29 27\nSerial: 01 45 90\nCnt: 09 15 8E 53\nBtn: 00\nType: 36\n\
             CRC: 00 D5\nSeed: 01 45 90\n"
        );
        assert_eq!(dec.from_flipper_key(&key(&arf)), Some(recovered.clone()));
        assert_eq!(dec.from_flipper_key(&key(&arf.replace("PSA GROUP", "PSA OLD"))), Some(recovered.clone()));
        // A serial whose key does not decrypt the frame, or an XOR-typed claim: undecrypted.
        assert_eq!(dec.from_flipper_key(&key(&pp.replace("Serial: 83344", "Serial: 83345"))), Some(tea.clone()));
        assert_eq!(dec.from_flipper_key(&key(&pp.replace("Type: 54", "Type: 35"))), Some(tea.clone()));
        // Key_2 is required (all three references).
        assert!(dec.from_flipper_key(&key(&pp.replace("Key_2: 0000000000002927\n", ""))).is_none());
        // TX from the loaded key: the C upload (PP encoder, Lock, counter + 1).
        let loaded = dec.from_flipper_key(&key(&pp)).unwrap();
        let got = decode_all(&merged(&dec.encode(&loaded, 0x01).unwrap()));
        assert_eq!((got[0].data, got[0].extra), (0xF47A_38D3_2DE3_24CE, Some(0xFC13)));
        // A capture without key2 (retired PSA2 decoder) has no key file.
        assert!(dec.to_flipper_key(&DecodedSignal { extra: None, ..xor }).is_none());
    }

    /// On demand, `KAT_PSA_SYNTH_OUT=<dir> cargo test psa_synthetic_test_files -- --ignored`: test
    /// files for a made-up TEA remote (serial 5EED01, no real fob's; the BF1 search finds it) to check
    /// over the air how KAT receives PSA frames a Flipper sends and how others receive KAT's:
    /// - `rx_end{0,1}_{flipper,fob}_{ook,fsk}.sub`: RAW files of a frame whose last bit is 0 / 1, in
    ///   the references' upload (`flipper`: end HIGH 1000 µs, what a Flipper sends from a saved file)
    ///   or the genuine fobs' (`fob`: 900 µs), 3 copies, OOK (Ook650) or 2FSK (Dev476);
    /// - `tx_end{0,1}_key.sub`: key files whose next code (TX Lock) ends in a 0 / 1 bit.
    #[test]
    #[ignore]
    fn psa_synthetic_test_files() {
        let Some(dir) = std::env::var_os("KAT_PSA_SYNTH_OUT") else { return };
        let dir = std::path::PathBuf::from(dir);
        const SERIAL: u32 = 0x5EED01;
        // b0..b1 = F4 7A: the sync nibble (b1 low nibble 0xA), as the IMPORTS TEA fob sends it.
        const HEADER: u64 = 0xF47A_0000_0000_0000;
        let tea = |counter: u32| frame_words(HEADER, 0, &PsaFields { mode: MODE_TEA, serial: SERIAL, button: 0, counter, crc: 0 });
        let mut by_last_bit: [Option<(u32, u64, u16)>; 2] = [None, None];
        for counter in 0x0A0_0100u32.. {
            let (key1, val) = tea(counter);
            let slot = &mut by_last_bit[usize::from(val & 1)];
            if slot.is_none() {
                *slot = Some((counter, key1, val));
            }
            if by_last_bit.iter().all(Option::is_some) {
                break;
            }
        }
        let upload = |key1: u64, val: u16, end_high: u32| -> Vec<LevelDuration> {
            let mut one = build_upload(key1, u32::from(val), &tea_fields_for(key1, val, SERIAL));
            let n = one.len();
            one[n - 2] = LevelDuration::new(true, end_high);
            let mut all = vec![LevelDuration::new(false, 10_000)];
            for _ in 0..3 {
                all.extend_from_slice(&one);
            }
            all.push(LevelDuration::new(false, 10_000));
            merged(&all)
        };
        let raw_sub = |name: &str, preset: &str, pulses: &[LevelDuration]| {
            let values: Vec<String> =
                pulses.iter().map(|p| if p.level { p.duration_us.to_string() } else { format!("-{}", p.duration_us) }).collect();
            let mut text = format!("Filetype: Flipper SubGhz RAW File\nVersion: 1\nFrequency: 433920000\nPreset: {preset}\nProtocol: RAW\n");
            for chunk in values.chunks(512) {
                text += &format!("RAW_Data: {}\n", chunk.join(" "));
            }
            std::fs::write(dir.join(name), text).unwrap();
        };
        for (bit, found) in by_last_bit.iter().enumerate() {
            let (counter, key1, val) = found.unwrap();
            // The frame must be what the decoder reads back from a genuine-style press, and BF1 must
            // find the made-up serial (as `autobf` will in the app).
            let fob = upload(key1, val, ENC_END_HIGH);
            assert_eq!(decode_all(&fob).first().map(|d| (d.data, d.extra)), Some((key1, Some(u64::from(val)))));
            let BruteForceOutcome::Found(hit) = recover_key(key1, u64::from(val), &BruteForceControl::default()) else { panic!("no key") };
            assert_eq!(hit.fields.serial, SERIAL);
            for (style, end_high) in [("flipper", TE_END_1000), ("fob", ENC_END_HIGH)] {
                let pulses = upload(key1, val, end_high);
                raw_sub(&format!("rx_end{bit}_{style}_ook.sub"), "FuriHalSubGhzPresetOok650Async", &pulses);
                raw_sub(&format!("rx_end{bit}_{style}_fsk.sub"), "FuriHalSubGhzPreset2FSKDev476Async", &pulses);
            }
            eprintln!("rx_end{bit}: counter {counter:07X} key1 {key1:016X} key2 {val:04X}");
        }
        // Key files whose next code (TX Lock = KAT button 0x01) ends in a 0 / 1 bit.
        let dec = PsaDecoder::new();
        let mut wanted = [true, true];
        for counter in 0x0A0_0200u32.. {
            let (key1, val) = tea(counter);
            let capture = psa_from(key1, u64::from(val), Some(SERIAL));
            let Some(next) = dec.sent_signal(&capture, 0x01) else { continue };
            let bit = (next.extra.unwrap_or(0) & 1) as usize;
            if !wanted[bit] {
                continue;
            }
            wanted[bit] = false;
            let mut key = dec.to_flipper_key(&decode_all(&upload(key1, val, ENC_END_HIGH))[0]).unwrap();
            key.frequency = 433_920_000;
            key.preset = Some("FuriHalSubGhzPresetOok650Async".to_string());
            std::fs::write(dir.join(format!("tx_end{bit}_key.sub")), key.to_text()).unwrap();
            eprintln!("tx_end{bit}: counter {counter:07X} key1 {key1:016X} key2 {val:04X} -> next {:016X}/{:04X}", next.data, next.extra.unwrap_or(0));
            if !wanted.iter().any(|&w| w) {
                break;
            }
        }
    }

    /// Fields of a frame of the made-up test remote (`psa_synthetic_test_files`): what the decoder
    /// recovers with its serial.
    fn tea_fields_for(key1: u64, val: u16, serial: u32) -> PsaFields {
        tea_decrypt_with_serial(key1, u32::from(val), serial).expect("frame of the test remote").0
    }

    /// `KAT_KEY_VECTORS=<dir> cargo test psa_oracle_vectors -- --ignored`: RAW uploads and KAT's
    /// key files for the C oracle (XOR frame; TEA frame, which the decoder leaves undecrypted).
    /// Both captured frames end in a 1 bit: the references' decoder does not take a frame ending in
    /// a 0 bit from a 1000 µs-end upload (`uploads_ending_in_a_zero_bit_decode`).
    #[test]
    #[ignore]
    fn psa_oracle_vectors() {
        let Some(dir) = std::env::var_os("KAT_KEY_VECTORS") else { return };
        let dir = std::path::PathBuf::from(dir);
        let dec = PsaDecoder::new();
        let cases = [
            psa_from(0x6EDA_D681_4A6A_49F3, 0x9023, Some(0x99EB25)),
            psa_from(0xF47A_2EA1_1A4C_7CAF, 0x2927, Some(0x014590)),
        ];
        for (i, src) in cases.iter().enumerate() {
            let tx = dec.encode(src, 0x01).unwrap();
            let once = merged(&tx[..ENC_UPLOAD_LEN]);
            let d = decode_all(&once).pop().unwrap();
            let key = dec.to_flipper_key(&d).unwrap();
            super::super::star_line::write_oracle_vector(&dir, &format!("psa_{}", i + 1), 433_920_000, "FuriHalSubGhzPresetOok650Async", &once, &key);
        }
    }
}
