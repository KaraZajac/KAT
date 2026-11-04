//! Faac SLH: FAAC "SLH" KeeLoq rolling-code gate remotes with a per-remote seed.
//!
//! References: Flipper-ARF's garage app `applications/system/subghz_garage/protocols/faac_slh.c`
//! (primary) and doctor-firmware `lib/subghz/protocols/faac_slh.c`. Both carry the Unleashed
//! "Genius" mechanism (the type-5 key named by the key file's `Manufacture`, FAAC_SLH otherwise)
//! and decode, display and transmit identically; garage's `create_data` ("Add manually", not in
//! KAT) runs the full `gen_data` (counter + 1) where doctor's only encrypts. Flipper-ARF's
//! `lib/subghz/protocols` copy is older (no `Manufacture`, last type-5 entry wins).
//!
//! **Air format** (`subghz_protocol_decoder_faac_slh_feed`): te_short 255 µs, te_long 595 µs,
//! te_delta 100 µs. Preamble HIGH 1190 ± 300 µs + LOW 1190 ± 300 µs; bit 0 = HIGH 255 + LOW 595,
//! bit 1 = HIGH 595 + LOW 255 (each ± 100 µs). A HIGH of 865 µs or more (the next preamble) ends
//! the frame — exactly 64 bits are reported — and is taken as the next frame's preamble. After a
//! decode the registry resets the decoder (the Flipper resets its receiver too), so of back-to-back
//! frames every other one is reported.
//!
//! **Frame**: fix (32) = serial (28) << 4 | button (4), then the KeeLoq hop (32). The cipher key is
//! FAAC learning (`keeloq_faac_learning`) of the remote's 32-bit seed with the manufacturer key
//! (keystore type 5, "FAAC_SLH"). Hop plaintext: three fix nibbles << 20 | counter (20 bits); for
//! an even counter the nibbles are fix[7:4], fix[3:0], fix[11:8] (serial nibble 0, button, serial
//! nibble 1), for an odd one fix[23:20], fix[19:16], fix[15:12].
//!
//! **Programming-mode frame** (master remote in learn mode): fix[31:16] = 0x520F and hop[7:0] = 0.
//! It carries the seed: bytes hop[23:16], hop[31:24], fix[7:0], fix[15:8] (as a little-endian
//! word) rotated right by (hop[15:8] & 0xF) and XORed with hop[15:8] ("mCnt").
//!
//! **Decoding** (`check_remote_controller` + `get_string`): a received frame never carries the
//! seed (the decoder's serialize clears it and nothing is learnt across frames), so a normal frame
//! shows serial and button, "Sd:Unknown", no counter; a programming frame shows the seed and mCnt.
//! Every 64-bit frame is reported, as in the C: without the seed there is nothing to check
//! (`crc_valid` = false). From a key file with a `Seed` (or `AllowZeroSeed`) the hop is decrypted:
//! counter = the low 16 of its 20 bits, `crc_valid` = the plaintext's 12 fix bits match (the C
//! displays the counter without checking).
//!
//! **KAT fields**: `data` = the 64-bit key, serial = 28 bits, button = 4 bits; `extra` = the seed
//! when known (low 32 bits; bit 32 = `AllowZeroSeed`), None otherwise. Programming frames: serial
//! and button None (from a doctor key file: its `Serial`/`Btn`), counter = mCnt, `extra` = the
//! seed derived from the frame.
//!
//! **Encoder** (`subghz_protocol_encoder_faac_slh_deserialize` + `gen_data` + `get_upload`; the
//! C's cross-file statics `temp_fix_backup`/`temp_counter_backup` start from this key alone):
//! - seed known: counter + 1 (0 past 0xFFFFF), button = the requested native code (4 bits), hop
//!   re-encrypted;
//! - seed unknown: the captured key is sent unchanged — unless another button is requested (the
//!   C's button override), which builds a frame under seed 0 with the counter not advanced;
//! - button [`PROG_BUTTON`] (0x10, the C's custom UP button) with a known seed: a programming-mode
//!   frame for the seed with counter + 1 (for a programming frame: mCnt + 1);
//! - a programming frame is otherwise sent unchanged — unless its key file names a remote
//!   (doctor-firmware's `Serial`/`Btn` lines, which its decoder fills with the remote decoded
//!   just before): then that remote's frame under the learnt seed, counter mCnt + 1 (doctor only;
//!   the garage app reads no such lines). Another button requested: the C's override, a normal
//!   frame for that serial (0 when unknown).
//!
//! Upload: HIGH 1190, LOW 1190, 64 bits, no trailing gap; `encoder.repeat` = 3. (Flipper-ARF's
//! garage TX plugin is built without `SUBGHZ_GARAGE_HAS_LIB_EXTENSIONS`, whose fallback takes the
//! "OFEX" counter path (counter → 0 or 0xFFFFF); KAT follows the source's standard path, as
//! doctor-firmware does.)
//!
//! **Key file** (`serialize`/`deserialize`): `Bit` must be 64; `Seed` (4 hex bytes) is required,
//! `AllowZeroSeed` (any value) makes seed 0 a real seed, `Manufacture` picks the type-5 key (KAT's
//! keystore has only FAAC_SLH, the C's fallback). KAT writes `Seed` from `extra` (the C's decoder
//! serialize writes 00 00 00 00 for a received frame — the same here, since `extra` is None) and
//! `AllowZeroSeed: true` when set; for a programming frame naming a remote, doctor's `Serial`,
//! `Btn` and `Cnt` lines before `Seed`.

use super::common::DecodedSignal;
use super::flipper_key::FlipperKey;
use super::g5_common::{key8, keystore_entry, next_counter, push, read_hex};
use super::keeloq_common::{
    keeloq_decrypt, keeloq_encrypt, keeloq_faac_learning, learning_types as lt,
};
use super::{ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`).
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: "Faac SLH",
    decoder: "Faac SLH",
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "KeeLoq",
    make: "FAAC",
    category: Category::Gate,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// The C names no Faac buttons; only KAT's programming-mode code is labelled.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    (code == PROG_BUTTON).then_some("Prog mode")
}

/// Native code that selects the C's custom UP button: send a programming-mode frame.
pub const PROG_BUTTON: u8 = 0x10;

const NAME: &str = "Faac SLH";
const TE_SHORT: u32 = 255;
const TE_LONG: u32 = 595;
const TE_DELTA: u32 = 100;
const MIN_COUNT_BIT: usize = 64;
/// `extra` bit marking `AllowZeroSeed`.
const ALLOW_ZERO_SEED: u64 = 1 << 32;
/// The C's default manufacturer (and fallback) for the type-5 key.
const DEFAULT_MANUFACTURER: &str = "FAAC_SLH";
/// C `encoder.repeat`.
const TX_REPEAT: usize = 3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    FoundPreambula,
    SaveDuration,
    CheckDuration,
}

/// `subghz_protocol_faac_slh_get_key` for a file without `Manufacture`: the type-5 entry named
/// FAAC_SLH (the only one in KAT's keystore).
fn faac_key() -> Option<u64> {
    keystore_entry(|e| e.learning == lt::FAAC && e.name == DEFAULT_MANUFACTURER).map(|e| e.key)
}

/// Seed knowledge in `extra`: (seed, AllowZeroSeed), None when the seed is unknown (the C's
/// `!allow_zero_seed && seed == 0`).
fn known_seed(extra: Option<u64>) -> Option<(u32, bool)> {
    let e = extra?;
    let (seed, allow) = (e as u32, e & ALLOW_ZERO_SEED != 0);
    (seed != 0 || allow).then_some((seed, allow))
}

fn split(data: u64) -> (u32, u32) {
    ((data >> 32) as u32, data as u32)
}

/// The C's programming-mode test in `check_remote_controller`.
fn is_prog(data: u64) -> bool {
    let (fix, hop) = split(data);
    fix >> 16 == 0x520F && hop & 0xFF == 0
}

/// Programming frame → (seed, mCnt).
fn prog_seed(data: u64) -> (u32, u8) {
    let (fix, hop) = split(data);
    let m = (hop >> 8) as u8;
    let word = ((hop >> 16) | ((fix & 0xFFFF) << 16)).rotate_right(u32::from(m & 0x0F));
    (u32::from_le_bytes(word.to_le_bytes().map(|b| b ^ m)), m)
}

/// The encoder's programming frame for `seed` with counter `cnt` (`gen_data`, UP button).
fn prog_frame(seed: u32, cnt: u32) -> u64 {
    let m = cnt as u8;
    let b = u32::from_le_bytes(seed.to_le_bytes().map(|b| b ^ m))
        .rotate_left(u32::from(m & 0x0F))
        .to_le_bytes();
    let enc1 = (0x52 << 24) | (0x0F << 16) | (u32::from(b[3]) << 8) | u32::from(b[2]);
    let enc2 = (u32::from(b[1]) << 24) | (u32::from(b[0]) << 16) | (u32::from(m) << 8);
    (u64::from(enc1) << 32) | u64::from(enc2)
}

/// The hop plaintext's three fix nibbles (bits 31..20) for a counter.
fn fix_nibbles(fix: u32, cnt: u32) -> u32 {
    let n = |i: u32| (fix >> (28 - 4 * i)) & 0x0F;
    if cnt & 1 == 0 {
        (n(6) << 8) | (n(7) << 4) | n(5)
    } else {
        (n(2) << 8) | (n(3) << 4) | n(4)
    }
}

/// `check_remote_controller`'s decrypt: the hop's plaintext under `seed` (0 when the keystore
/// has no type-5 key, as the C leaves it).
fn decrypt(hop: u32, seed: u32) -> u32 {
    faac_key().map_or(0, |key| {
        keeloq_decrypt(hop, keeloq_faac_learning(seed, key))
    })
}

/// A decode of `data` with what is known about the seed.
fn signal(data: u64, extra: Option<u64>) -> DecodedSignal {
    let base = DecodedSignal {
        serial: None,
        button: None,
        counter: None,
        crc_valid: false,
        data,
        data_count_bit: MIN_COUNT_BIT,
        encoder_capable: false,
        extra: None,
        protocol_display_name: None,
    };
    if is_prog(data) {
        let (seed, m) = prog_seed(data);
        let allow = extra.map_or(0, |e| e & ALLOW_ZERO_SEED);
        return DecodedSignal {
            counter: Some(u16::from(m)),
            extra: Some(u64::from(seed) | allow),
            ..base
        };
    }
    let (fix, hop) = split(data);
    let base = DecodedSignal {
        serial: Some(fix >> 4),
        button: Some((fix & 0x0F) as u8),
        ..base
    };
    match known_seed(extra) {
        Some((seed, allow)) => {
            let plain = decrypt(hop, seed);
            let cnt = plain & 0xF_FFFF;
            DecodedSignal {
                counter: Some(cnt as u16),
                crc_valid: plain >> 20 == fix_nibbles(fix, cnt),
                encoder_capable: true,
                extra: Some(u64::from(seed) | if allow { ALLOW_ZERO_SEED } else { 0 }),
                ..base
            }
        }
        None => base,
    }
}

/// `gen_data` (+ the deserialize-time `check_remote_controller`) for a fresh session (the C's
/// `temp_fix_backup` / `temp_counter_backup` statics empty): the key to send.
fn gen_data(decoded: &DecodedSignal, button: u8) -> u64 {
    let data = decoded.data;
    let (fix, hop) = split(data);
    let allow = decoded.extra.is_some_and(|e| e & ALLOW_ZERO_SEED != 0);
    let prog = is_prog(data);
    // `check_remote_controller`: a programming key brings its own seed and mCnt and leaves
    // serial/button as the key file set them (doctor-firmware's `Serial`/`Btn` lines, else 0); a
    // normal key gives serial/button from the fix and the counter from the hop.
    let (seed, mut cnt, serial, captured_btn) = if prog {
        let (seed, m) = prog_seed(data);
        let (serial, btn) = (decoded.serial.unwrap_or(0), decoded.button.unwrap_or(0));
        (seed, u32::from(m), serial, btn)
    } else {
        let seed = decoded.extra.map_or(0, |e| e as u32);
        (
            seed,
            decrypt(hop, seed) & 0xF_FFFF,
            fix >> 4,
            (fix & 0x0F) as u8,
        )
    };
    let seed_known = allow || seed != 0;
    let btn = if button <= 0x0F && button != captured_btn {
        // The C's button override: skips the custom-button block below.
        button
    } else {
        if button == PROG_BUTTON && seed_known {
            return prog_frame(seed, next_counter(cnt, 0xF_FFFF));
        }
        if !seed_known {
            // "Do not generate new data, send data from buffer."
            return data;
        }
        if prog && serial == 0 && captured_btn == 0 {
            // Programming key without a remote to restore: sent unchanged.
            return data;
        }
        captured_btn
    };
    if seed_known {
        cnt = next_counter(cnt, 0xF_FFFF);
    }
    let fix = (serial << 4) | u32::from(btn);
    let plain = (fix_nibbles(fix, cnt) << 20) | cnt;
    let hop = faac_key().map_or(0, |key| {
        keeloq_encrypt(plain, keeloq_faac_learning(seed, key))
    });
    if hop != 0 {
        (u64::from(fix) << 32) | u64::from(hop)
    } else {
        data
    }
}

/// `subghz_protocol_faac_slh_create_data` (the SubGHz app's "Add Manually",
/// `subghz_txrx_gen_faac_slh_protocol`): the key file of a new remote. Flipper-ARF runs the full
/// `gen_data` (`advance`: counter + 1, 0 past 0xFFFFF), doctor-firmware only encrypts the counter
/// as given; `fix = serial << 4 | btn` (the C does not mask the button), hop under FAAC learning of
/// `seed` with the type-5 key ([faac_key]: doctor's "Genius" falls back to it as KAT's keystore has
/// no other). Lines: `Bit: 64`, `Key`, `Seed` (4 hex bytes), `AllowZeroSeed: true`, then doctor's
/// `Manufacture` when given. None without a type-5 key (the C then keeps an empty key).
pub(super) fn create_data(
    serial: u32,
    btn: u8,
    cnt: u32,
    seed: u32,
    advance: bool,
    manufacture: Option<&str>,
) -> Option<FlipperKey> {
    let mut cnt = cnt & 0xF_FFFF;
    if advance {
        cnt = next_counter(cnt, 0xF_FFFF);
    }
    let fix = (serial << 4) | u32::from(btn);
    let plain = (fix_nibbles(fix, cnt) << 20) | cnt;
    let hop = keeloq_encrypt(plain, keeloq_faac_learning(seed, faac_key()?));
    let mut k = FlipperKey::for_protocol(NAME, MIN_COUNT_BIT, (u64::from(fix) << 32) | u64::from(hop));
    k.push_hex("Seed", u64::from(seed), 4);
    k.push("AllowZeroSeed", "true");
    if let Some(mf) = manufacture.filter(|m| !m.is_empty()) {
        k.push("Manufacture", mf);
    }
    Some(k)
}

/// One copy of `subghz_protocol_encoder_faac_slh_get_upload`.
fn upload_once(data: u64, out: &mut Vec<LevelDuration>) {
    push(out, true, TE_LONG * 2);
    push(out, false, TE_LONG * 2);
    for i in (0..MIN_COUNT_BIT).rev() {
        if (data >> i) & 1 == 1 {
            push(out, true, TE_LONG);
            push(out, false, TE_SHORT);
        } else {
            push(out, true, TE_SHORT);
            push(out, false, TE_LONG);
        }
    }
}

pub struct FaacSlhDecoder {
    step: Step,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: usize,
}

impl FaacSlhDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
        }
    }

    fn add_bit(&mut self, bit: u64) {
        self.decode_data = (self.decode_data << 1) | bit;
        self.decode_count_bit += 1;
    }
}

impl Default for FaacSlhDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for FaacSlhDecoder {
    fn name(&self) -> &'static str {
        NAME
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
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        *self = Self::new();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                if level && duration_diff!(duration, TE_LONG * 2) < TE_DELTA * 3 {
                    self.step = Step::FoundPreambula;
                }
            }
            Step::FoundPreambula => {
                if !level && duration_diff!(duration, TE_LONG * 2) < TE_DELTA * 3 {
                    self.step = Step::SaveDuration;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::SaveDuration => {
                if level {
                    if duration >= TE_SHORT * 3 + TE_DELTA {
                        self.step = Step::FoundPreambula;
                        let result = (self.decode_count_bit == MIN_COUNT_BIT)
                            .then(|| signal(self.decode_data, None));
                        self.decode_data = 0;
                        self.decode_count_bit = 0;
                        return result;
                    }
                    self.te_last = duration;
                    self.step = Step::CheckDuration;
                } else {
                    self.step = Step::Reset;
                }
            }
            Step::CheckDuration => {
                if !level {
                    if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA
                        && duration_diff!(duration, TE_LONG) < TE_DELTA
                    {
                        self.add_bit(0);
                        self.step = Step::SaveDuration;
                    } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA
                        && duration_diff!(duration, TE_SHORT) < TE_DELTA
                    {
                        self.add_bit(1);
                        self.step = Step::SaveDuration;
                    } else {
                        self.step = Step::Reset;
                    }
                } else {
                    self.step = Step::Reset;
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
        let data = gen_data(decoded, button);
        let mut out = Vec::with_capacity(TX_REPEAT * 130);
        for _ in 0..TX_REPEAT {
            upload_once(data, &mut out);
        }
        Some(out)
    }

    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        // `subghz_protocol_decoder_faac_slh_deserialize`: Bit must be 64, Seed is required.
        if key.bits != MIN_COUNT_BIT {
            return None;
        }
        let data = key8(key)?;
        let seed = read_hex(key, "Seed", 4)?;
        let seed = u32::from_be_bytes([seed[0], seed[1], seed[2], seed[3]]);
        let allow = key
            .field("AllowZeroSeed")
            .is_some_and(|v| !v.trim().is_empty());
        let extra = u64::from(seed) | if allow { ALLOW_ZERO_SEED } else { 0 };
        let d = signal(data, Some(extra));
        if !is_prog(data) {
            return Some(d);
        }
        // doctor-firmware's generic block also reads `Serial`/`Btn` (its decoder writes the
        // remote decoded before the programming frame there); with them the encoder sends that
        // remote's frames under the learnt seed instead of repeating the programming frame.
        let serial = key.u32_field("Serial").unwrap_or(0);
        let btn = key.u32_field("Btn").unwrap_or(0) as u8;
        if serial == 0 && btn == 0 {
            return Some(d);
        }
        let seed_known = known_seed(d.extra).is_some();
        Some(DecodedSignal {
            serial: Some(serial),
            button: Some(btn),
            encoder_capable: seed_known,
            ..d
        })
    }

    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != MIN_COUNT_BIT {
            return None;
        }
        let mut k = FlipperKey::new(NAME, MIN_COUNT_BIT, decoded.data, 0, "");
        if is_prog(decoded.data) && (decoded.serial.is_some() || decoded.button.is_some()) {
            // doctor-firmware's generic lines (they come before `Seed`).
            k.push("Serial", decoded.serial.unwrap_or(0).to_string());
            k.push("Btn", decoded.button.unwrap_or(0).to_string());
            k.push("Cnt", decoded.counter.unwrap_or(0).to_string());
        }
        let extra = decoded.extra.unwrap_or(0);
        k.push_hex("Seed", extra & 0xFFFF_FFFF, 4);
        if extra & ALLOW_ZERO_SEED != 0 {
            k.push("AllowZeroSeed", "true");
        }
        Some(k)
    }
}

/// A key file for a random frame (oracle driver fixtures). `r % 5`: 0-1 normal frame with its
/// seed, 2 seed 0 + AllowZeroSeed, 3 programming frame, 4 normal frame without the seed.
#[cfg(test)]
pub(super) fn fixture_key(r: u64) -> FlipperKey {
    let seed = match r % 5 {
        2 => 0,
        _ => ((r >> 8) as u32).max(1),
    };
    let cnt = if r & 0x300 == 0 {
        0xF_FFFF
    } else {
        ((r >> 20) as u32) & 0xF_FFFF
    };
    let data = if r % 5 == 3 {
        prog_frame(seed, cnt)
    } else {
        let fix = ((r >> 36) as u32) & 0xFFFF_FFF0 | ((r >> 4) as u32 & 0x0F);
        let plain = (fix_nibbles(fix, cnt) << 20) | cnt;
        let hop = keeloq_encrypt(plain, keeloq_faac_learning(seed, faac_key().unwrap()));
        (u64::from(fix) << 32) | u64::from(hop)
    };
    let extra = match r % 5 {
        2 => Some(ALLOW_ZERO_SEED),
        3 | 4 => None,
        _ => Some(u64::from(seed)),
    };
    FaacSlhDecoder::new()
        .to_flipper_key(&signal(data, extra))
        .unwrap()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::export::flipper::import_sub_raw;
    use crate::protocols::ProtocolRegistry;
    use std::path::Path;

    /// Decode a stream like the registry: reset after every decode.
    fn decode_all(signal: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = FaacSlhDecoder::new();
        let mut out = Vec::new();
        for ld in signal {
            if let Some(d) = dec.feed(ld.level, ld.duration_us) {
                out.push(d);
                dec.reset();
            }
        }
        out
    }

    /// One transmission of `data`, terminated by a preamble HIGH.
    fn capture(data: u64) -> Vec<LevelDuration> {
        let mut s = Vec::new();
        upload_once(data, &mut s);
        s.push(LevelDuration::new(true, TE_LONG * 2));
        s
    }

    /// A remote's frame for seed/serial/button/counter.
    fn frame(seed: u32, serial: u32, btn: u8, cnt: u32) -> u64 {
        let fix = (serial << 4) | u32::from(btn);
        let plain = (fix_nibbles(fix, cnt) << 20) | cnt;
        let hop = keeloq_encrypt(plain, keeloq_faac_learning(seed, faac_key().unwrap()));
        (u64::from(fix) << 32) | u64::from(hop)
    }

    fn with_seed(data: u64, seed: u32) -> DecodedSignal {
        signal(data, Some(u64::from(seed)))
    }

    #[test]
    fn decodes_normal_frames_without_seed() {
        let data = frame(0x1234_5678, 0x0A0_3A05, 0x8, 0x1_2345);
        let d = decode_all(&capture(data));
        assert_eq!(d.len(), 1);
        let d = &d[0];
        assert_eq!((d.data, d.data_count_bit), (data, 64));
        assert_eq!(
            (d.serial, d.button, d.counter, d.extra),
            (Some(0x0A0_3A05), Some(0x8), None, None)
        );
        assert!(!d.crc_valid && !d.encoder_capable);
    }

    #[test]
    fn decodes_programming_frames() {
        for (seed, m) in [
            (0x1234_5678u32, 0x00u32),
            (0xDEAD_BEEF, 0x37),
            (0x0000_0001, 0xFF),
        ] {
            let data = prog_frame(seed, m);
            assert!(is_prog(data));
            let d = decode_all(&capture(data)).pop().unwrap();
            assert_eq!(
                (d.serial, d.button, d.counter),
                (None, None, Some(m as u16))
            );
            assert_eq!(d.extra, Some(u64::from(seed)));
            assert_eq!(prog_seed(data), (seed, m as u8));
        }
    }

    #[test]
    fn rejects_bad_timing_and_lengths() {
        let data = frame(0x1234_5678, 0x0A0_3A05, 0x8, 2);
        // 63 bits.
        let mut s = capture(data);
        s.drain(2..4);
        assert!(decode_all(&s).is_empty());
        // A 400 µs LOW is neither 255 ± 100 nor 595 ± 100.
        let mut s = capture(data);
        s[3].duration_us = 425;
        assert!(decode_all(&s).is_empty());
        // Preamble LOW outside 1190 ± 300.
        let mut s = capture(data);
        s[1].duration_us = 1500;
        assert!(decode_all(&s).is_empty());
        // No terminating HIGH ≥ 865 µs: not reported.
        let mut s = capture(data);
        s.pop();
        assert!(decode_all(&s).is_empty());
    }

    #[test]
    fn back_to_back_frames_every_other_one() {
        let data = frame(0x1234_5678, 0x0A0_3A05, 0x8, 2);
        let mut s = Vec::new();
        for _ in 0..4 {
            upload_once(data, &mut s);
        }
        s.push(LevelDuration::new(true, TE_LONG * 2));
        assert_eq!(decode_all(&s).len(), 2);
        // Without the reset after a decode, each terminating HIGH is also the next preamble.
        let mut dec = FaacSlhDecoder::new();
        assert_eq!(
            s.iter()
                .filter_map(|l| dec.feed(l.level, l.duration_us))
                .count(),
            4
        );
    }

    #[test]
    fn seeded_key_decrypts_and_checks() {
        let data = frame(0x1234_5678, 0x0A0_3A05, 0x9, 0xF_FFFE);
        let d = with_seed(data, 0x1234_5678);
        assert_eq!(
            (d.counter, d.crc_valid, d.encoder_capable),
            (Some(0xFFFE), true, true)
        );
        let wrong = with_seed(data, 0x1234_5679);
        assert!(!wrong.crc_valid);
        // AllowZeroSeed: seed 0 is a seed.
        let data0 = frame(0, 0x0A0_3A05, 0x9, 7);
        let d0 = signal(data0, Some(ALLOW_ZERO_SEED));
        assert_eq!((d0.counter, d0.crc_valid), (Some(7), true));
        assert_eq!(signal(data0, Some(0)).counter, None);
    }

    #[test]
    fn encode_with_seed_bumps_counter_every_button() {
        let dec = FaacSlhDecoder::new();
        let seed = 0x0BAD_F00D;
        for cnt in [0x1u32, 0x2, 0xF_FFFF] {
            let d = with_seed(frame(seed, 0x0A0_3A05, 0x8, cnt), seed);
            for button in [0x8u8, 0x9, 0x1, 0xF] {
                let wire = dec.encode(&d, button).unwrap();
                assert_eq!(wire.len(), 3 * 130);
                let got = decode_all(&capture_of(&wire));
                assert_eq!(got.len(), 2, "cnt {cnt:#x} button {button:#x}");
                let next = with_seed(got[0].data, seed);
                let want = next_counter(cnt, 0xF_FFFF);
                assert_eq!(
                    (next.serial, next.button, next.counter),
                    (Some(0x0A0_3A05), Some(button), Some(want as u16))
                );
                assert!(next.crc_valid);
                assert_eq!(decrypt(got[0].data as u32, seed) & 0xF_FFFF, want);
            }
        }
    }

    /// An upload followed by a preamble HIGH (so its last copy is terminated too).
    fn capture_of(wire: &[LevelDuration]) -> Vec<LevelDuration> {
        let mut s = wire.to_vec();
        s.push(LevelDuration::new(true, TE_LONG * 2));
        s
    }

    #[test]
    fn encode_without_seed_replays_or_overrides() {
        let dec = FaacSlhDecoder::new();
        let data = frame(0x1234_5678, 0x0A0_3A05, 0x8, 0x42);
        let d = decode_all(&capture(data)).pop().unwrap();
        // Captured button: the key is sent unchanged.
        let got = decode_all(&capture_of(&dec.encode(&d, 0x8).unwrap()));
        assert!(got.iter().all(|g| g.data == data));
        // UP without a seed: also unchanged.
        let got = decode_all(&capture_of(&dec.encode(&d, PROG_BUTTON).unwrap()));
        assert!(got.iter().all(|g| g.data == data));
        // Another button: the C's override path encrypts under seed 0, counter not advanced.
        let got = decode_all(&capture_of(&dec.encode(&d, 0x9).unwrap()));
        let fix = (0x0A0_3A05u32 << 4) | 0x9;
        let cnt = decrypt(data as u32, 0) & 0xF_FFFF;
        assert_eq!(got[0].data >> 32, u64::from(fix));
        assert_eq!(decrypt(got[0].data as u32, 0) & 0xF_FFFF, cnt);
    }

    #[test]
    fn encode_programming_frames() {
        let dec = FaacSlhDecoder::new();
        let seed = 0xCAFE_1234;
        // Normal frame with a seed + UP: programming frame with counter + 1.
        let d = with_seed(frame(seed, 0x0A0_3A05, 0x8, 0x1_00FF), seed);
        let got = decode_all(&capture_of(&dec.encode(&d, PROG_BUTTON).unwrap()));
        assert!(is_prog(got[0].data));
        assert_eq!(
            (got[0].extra, got[0].counter),
            (Some(u64::from(seed)), Some(0x00))
        );
        // Programming frame + UP: mCnt + 1; other buttons replay it.
        let p = decode_all(&capture(prog_frame(seed, 0x41))).pop().unwrap();
        let got = decode_all(&capture_of(&dec.encode(&p, PROG_BUTTON).unwrap()));
        assert_eq!(
            (got[0].extra, got[0].counter),
            (Some(u64::from(seed)), Some(0x42))
        );
        let got = decode_all(&capture_of(&dec.encode(&p, 0).unwrap()));
        assert_eq!(got[0].data, prog_frame(seed, 0x41));
        // Another button on a programming frame: the C's override, a frame for serial 0.
        let got = decode_all(&capture_of(&dec.encode(&p, 0x3).unwrap()));
        assert_eq!(got[0].data >> 32, 0x3);
        assert_eq!(with_seed(got[0].data, seed).counter, Some(0x42));
    }

    #[test]
    fn doctor_programming_key_with_remote() {
        // doctor-firmware writes the remote decoded before a programming frame into its key file
        // (`Serial`/`Btn`); its encoder then sends that remote under the learnt seed.
        let dec = FaacSlhDecoder::new();
        let seed = 0x1357_9BDF;
        let text = format!(
            "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
             Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Faac SLH\nBit: 64\nKey: {}\n\
             Serial: 10498130\nBtn: 8\nCnt: 65\nKey_2: 00 00 00 00 00 00 00 00\nSeed: 00 00 00 00\n",
            prog_frame(seed, 0x41)
                .to_be_bytes()
                .iter()
                .map(|b| format!("{b:02X}"))
                .collect::<Vec<_>>()
                .join(" ")
        );
        let p = dec
            .from_flipper_key(&FlipperKey::parse(&text).unwrap())
            .unwrap();
        assert_eq!(
            (p.serial, p.button, p.counter),
            (Some(10_498_130), Some(8), Some(0x41))
        );
        assert!(p.encoder_capable);
        for (button, want_btn) in [(8u8, 8u8), (0, 0), (9, 9)] {
            let got = decode_all(&capture_of(&dec.encode(&p, button).unwrap()));
            let next = with_seed(got[0].data, seed);
            assert_eq!(
                (next.serial, next.button, next.counter, next.crc_valid),
                (Some(10_498_130), Some(want_btn), Some(0x42), true)
            );
        }
        // UP still sends the next programming frame.
        let got = decode_all(&capture_of(&dec.encode(&p, PROG_BUTTON).unwrap()));
        assert_eq!(got[0].data, prog_frame(seed, 0x42));
        // Round trip keeps the remote.
        let back = dec.to_flipper_key(&p).unwrap().to_text();
        assert!(
            back.contains("Serial: 10498130\nBtn: 8\nCnt: 65\nSeed: "),
            "{back}"
        );
        let again = dec
            .from_flipper_key(&FlipperKey::parse(&back).unwrap())
            .unwrap();
        assert_eq!(format!("{again:?}"), format!("{p:?}"));
        // "Serial: 0 / Btn: 0" (what doctor writes with no remote before) names none.
        let none = text.replace("Serial: 10498130\nBtn: 8", "Serial: 0\nBtn: 0");
        let p0 = dec
            .from_flipper_key(&FlipperKey::parse(&none).unwrap())
            .unwrap();
        assert_eq!(
            (p0.serial, p0.button, p0.encoder_capable),
            (None, None, false)
        );
    }

    const UT_KEY: &str = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
        Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Faac SLH\nBit: 64\n\
        Key: A0 3A 05 28 3C FA A6 3B\nSeed: 00 00 00 00\n";

    #[test]
    fn key_file_round_trip() {
        let dec = FaacSlhDecoder::new();
        let d = dec
            .from_flipper_key(&FlipperKey::parse(UT_KEY).unwrap())
            .unwrap();
        assert_eq!(
            (d.serial, d.button, d.counter, d.extra),
            (Some(0xA03_A052), Some(0x8), None, None)
        );
        let text = dec.to_flipper_key(&d).unwrap().to_text();
        assert!(
            text.contains(
                "Protocol: Faac SLH\nBit: 64\nKey: A0 3A 05 28 3C FA A6 3B\nSeed: 00 00 00 00\n"
            ),
            "{text}"
        );
        // Seeded files keep their seed and AllowZeroSeed.
        for extra in [
            u64::from(0x1234_5678u32),
            ALLOW_ZERO_SEED,
            0x89AB_CDEF | ALLOW_ZERO_SEED,
        ] {
            let d = signal(frame(extra as u32, 0x0012_3456, 0x2, 5), Some(extra));
            let text = dec.to_flipper_key(&d).unwrap().to_text();
            assert_eq!(
                text.contains("AllowZeroSeed: true"),
                extra & ALLOW_ZERO_SEED != 0
            );
            let again = dec
                .from_flipper_key(&FlipperKey::parse(&text).unwrap())
                .unwrap();
            assert_eq!(format!("{again:?}"), format!("{d:?}"));
            assert!(again.crc_valid && again.encoder_capable);
        }
        // Programming frames round-trip too (the seed comes from the frame).
        let p = decode_all(&capture(prog_frame(0x55AA_1234, 7)))
            .pop()
            .unwrap();
        let text = dec.to_flipper_key(&p).unwrap().to_text();
        let again = dec
            .from_flipper_key(&FlipperKey::parse(&text).unwrap())
            .unwrap();
        assert_eq!(format!("{again:?}"), format!("{p:?}"));
        // Missing Seed / wrong bit count are refused (C: "Missing Seed").
        assert!(dec
            .from_flipper_key(
                &FlipperKey::parse(&UT_KEY.replace("Seed: 00 00 00 00\n", "")).unwrap()
            )
            .is_none());
        assert!(dec
            .from_flipper_key(&FlipperKey::parse(&UT_KEY.replace("Bit: 64", "Bit: 65")).unwrap())
            .is_none());
        let reg = ProtocolRegistry::new();
        assert_eq!(
            reg.decode_key(&FlipperKey::parse(UT_KEY).unwrap())
                .map(|(n, _)| n),
            Some("Faac SLH".to_string())
        );
    }

    #[test]
    fn decodes_flipper_unit_test_capture() {
        let path = Path::new(
            "REFERENCES/Flipper-ARF/applications/debug/unit_tests/resources/unit_tests/subghz/faac_slh_raw.sub",
        );
        if !path.exists() {
            eprintln!("Skip: {path:?} not found");
            return;
        }
        let (freq, raw) = import_sub_raw(path).unwrap();
        let pairs: Vec<LevelDuration> = raw
            .iter()
            .map(|p| LevelDuration::new(p.level, p.duration_us))
            .collect();
        let mut reg = ProtocolRegistry::new();
        let got = reg.process_signal_stream(&pairs, freq);
        let faac: Vec<_> = got.iter().filter(|(n, _, _)| n == NAME).collect();
        assert!(
            !faac.is_empty(),
            "{:?}",
            got.iter().map(|(n, _, _)| n).collect::<Vec<_>>()
        );
        for (_, d, _) in faac {
            assert_eq!(d.serial, Some(0xA03_A052));
            assert!(matches!(d.button, Some(0x8 | 0x9)));
        }
    }
}
