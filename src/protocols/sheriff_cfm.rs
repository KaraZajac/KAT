//! Sheriff CFM protocol decoder/encoder
//!
//! Ported from Flipper-ARF `lib/subghz/protocols/sheriff_cfm.c` / `sheriff_cfm.h`
//! (`SUBGHZ_PROTOCOL_SHERIFF_CFM_NAME = "Sheriff CFM"`), the only copy in REFERENCES. Sheriff is a
//! Russian aftermarket car-alarm brand; CFM covers the ZX-750 and ZX-930 remotes. AM/OOK, 315 and
//! 433.92 MHz (C flags `SubGhzProtocolFlag_315 | SubGhzProtocolFlag_433 | SubGhzProtocolFlag_AM`).
//!
//! **PHY / framing** — byte-for-byte the Flipper KeeLoq PWM state machine (the same one KAT's
//! `keeloq.rs` uses): te_short 400 µs, te_long 800 µs, te_delta 140 µs. Preamble: ≥3 HIGH ~400 µs
//! pulses (each followed by a ~400 µs LOW), then a header LOW of 4000 ± 1400 µs. Data: short HIGH +
//! long LOW (±280) = 1, long HIGH (±280) + short LOW = 0. A LOW ≥ 940 µs ends the frame, which must
//! hold 64..66 bits (only the first 64 are kept).
//!
//! **Frame** — 64 bits, MSB-first, and NOT bit-reversed (unlike standard KeeLoq): `fix` = bits 63..32
//! (32-bit serial → `serial`), `hop` = bits 31..0 (KeeLoq ciphertext after a per-model byte shuffle).
//!
//! **Decrypt (`cfm_try_decrypt`)** — for ZX-750 then ZX-930: split the hop into little-endian bytes,
//! undo the model's byte transform (ZX-750: swap the nibbles of bytes 0 and 2; ZX-930: invert byte 0,
//! swap bytes 1/2, rotate byte 0 right, swap the nibbles of byte 1 and rotate it left twice), KeeLoq-
//! decrypt (simple learning, `keeloq_common::keeloq_decrypt`) with the model's manufacturer key, and
//! accept when the plaintext's top byte is a button (0x10 Lock, 0x20 Unlock, 0x40 Trunk, 0x80 Panic)
//! AND plaintext bits 25..16 equal `fix & 0x3FF`. Counter = plaintext bits 15..0.
//!
//! **Keys** — the C embeds both 64-bit manufacturer keys XOR-obfuscated with π digits. KAT does not
//! copy them: both are already in KAT's embedded keystore and were verified equal to the C's decoded
//! values — ZX-750 = entry "ZX-730-750-1055", ZX-930 = entry "Pandora_M101". They are looked up by
//! entry name through `keystore::keeloq_mf_keys_with_names()` (parsed once). A missing key disables
//! that model.
//!
//! **Gating** — emission requires the full KeeLoq-style structure (preamble, 4 ms header, 64..66 PWM
//! bits, end gap) AND a validated decrypt: button whitelist (4/256) × 10-bit serial match (1/1024)
//! per model, i.e. ≈1/32768 for a random 64-bit frame. Exactly as in the C.
//!
//! **Interaction with other KAT decoders** — `keeloq.rs` ("KeeLoq") consumes the same air format and
//! its frames end on the same pulse, so this decoder is registered BEFORE it (first match per
//! pulse): KeeLoq's keystore selector accepts a random frame ≈6% of the time, while a genuine KeeLoq
//! frame from another system passes Sheriff's check ≈1 in 32768 (ignored diagnostic `sheriff_cfm_registry_claims`). Star Line
//! (250/500 µs, 1 ms header) and Scher-Khan (750/1100 µs) use timings this decoder cannot match. The
//! generic KeeLoq fallback (`keeloq_generic`, only when nothing decodes) reads 68-bit Kia V3/V4 or
//! 250/500 µs Star Line layouts, neither of which Sheriff CFM accepts (64..66 bits, 400/800 µs).
//!
//! **Encoder** (`subghz_protocol_encoder_sheriff_cfm_get_upload`) — full rolling-code encoder: model
//! from the decrypt, counter + 1 (wrapping to 0 past 0xFFFF, as in the C), plaintext
//! `btn << 24 | (fix & 0x3FF) << 16 | cnt`, KeeLoq-encrypt, apply the model's forward byte transform,
//! keep `fix`. Upload: 11 × (400 HIGH, 400 LOW), 400 HIGH, 4000 LOW, 64 PWM bits MSB-first
//! (1 = 400 HIGH followed by 800 LOW, 0 = 800 HIGH followed by 400 LOW), 400 HIGH, 16 000 LOW. KAT buttons map 1:1 onto the C's codes:
//! Lock → 0x10, Unlock → 0x20, Trunk → 0x40, Panic → 0x80; native codes pass through and any other
//! code keeps the captured button. The C yields the upload 3 times (`encoder.repeat`), so the KAT
//! waveform holds 3 copies.

use super::keeloq_common::{keeloq_decrypt, keeloq_encrypt};
use super::{DecodedSignal, ProtocolDecoder, ProtocolTiming};
use crate::duration_diff;
use crate::keystore;
use crate::radio::demodulator::LevelDuration;
use std::sync::OnceLock;

const TE_SHORT: u32 = 400;
const TE_LONG: u32 = 800;
const TE_DELTA: u32 = 140;
const MIN_COUNT_BIT: usize = 64;

/// Encoder: preamble pairs before the final header pulse.
const TX_PREAMBLE_PAIRS: usize = 11;
/// C `encoder.repeat`.
const TX_REPEAT: usize = 3;

/// Keystore entry names holding the C's ZX-750 / ZX-930 manufacturer keys.
const KEY_NAME_ZX750: &str = "ZX-730-750-1055";
const KEY_NAME_ZX930: &str = "Pandora_M101";

// Button codes (plaintext top byte, C `cfm_btn_name`).
const CFM_LOCK: u8 = 0x10;
const CFM_UNLOCK: u8 = 0x20;
const CFM_TRUNK: u8 = 0x40;
const CFM_PANIC: u8 = 0x80;

// KAT generic button codes.
const BTN_LOCK: u8 = 0x01;
const BTN_UNLOCK: u8 = 0x02;
const BTN_TRUNK: u8 = 0x04;
const BTN_PANIC: u8 = 0x08;

/// Sheriff CFM remote model (C `SheriffCfmModel`), tried in this order.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Model {
    Zx750,
    Zx930,
}

const MODELS: [Model; 2] = [Model::Zx750, Model::Zx930];

/// Manufacturer keys [ZX-750, ZX-930] from the embedded keystore (0 = missing).
fn mf_keys() -> &'static [u64; 2] {
    static KEYS: OnceLock<[u64; 2]> = OnceLock::new();
    KEYS.get_or_init(|| {
        let keys = keystore::keeloq_mf_keys_with_names();
        let find = |name: &str| keys.iter().find(|(n, _)| n == name).map_or(0, |(_, k)| *k);
        [find(KEY_NAME_ZX750), find(KEY_NAME_ZX930)]
    })
}

impl Model {
    fn key(self) -> u64 {
        match self {
            Model::Zx750 => mf_keys()[0],
            Model::Zx930 => mf_keys()[1],
        }
    }
}

#[inline]
fn rlf(v: u8) -> u8 {
    v.rotate_left(1)
}

#[inline]
fn rrf(v: u8) -> u8 {
    v.rotate_right(1)
}

#[inline]
fn swap_nibbles(v: u8) -> u8 {
    v.rotate_left(4)
}

/// `cfm_decrypt_transform`: undo the model's byte shuffle (bytes are the hop in LE order).
fn decrypt_transform(hop: &mut [u8; 4], model: Model) {
    match model {
        Model::Zx750 => {
            hop[0] = swap_nibbles(hop[0]);
            hop[2] = swap_nibbles(hop[2]);
        }
        Model::Zx930 => {
            hop[0] = !hop[0];
            hop.swap(1, 2);
            hop[0] = rrf(hop[0]);
            hop[1] = rlf(rlf(swap_nibbles(hop[1])));
        }
    }
}

/// `cfm_encrypt_transform`: the model's forward byte shuffle (inverse of `decrypt_transform`).
fn encrypt_transform(hop: &mut [u8; 4], model: Model) {
    match model {
        Model::Zx750 => {
            hop[0] = swap_nibbles(hop[0]);
            hop[2] = swap_nibbles(hop[2]);
        }
        Model::Zx930 => {
            hop[1] = swap_nibbles(rrf(rrf(hop[1])));
            hop[0] = !rlf(hop[0]);
            hop.swap(1, 2);
        }
    }
}

/// Decrypt result: (model, button byte, serial = fix, counter).
type Decrypted = (Model, u8, u32, u16);

/// `cfm_try_decrypt`.
fn try_decrypt(data: u64) -> Option<Decrypted> {
    let hop_encrypted = data as u32;
    let fix = (data >> 32) as u32;
    for model in MODELS {
        let key = model.key();
        if key == 0 {
            continue;
        }
        let mut hop = hop_encrypted.to_le_bytes();
        decrypt_transform(&mut hop, model);
        let decrypted = keeloq_decrypt(u32::from_le_bytes(hop), key);
        let btn = (decrypted >> 24) as u8;
        let valid_btn = matches!(btn, CFM_LOCK | CFM_UNLOCK | CFM_TRUNK | CFM_PANIC);
        if valid_btn && ((decrypted >> 16) & 0x3FF) == (fix & 0x3FF) {
            return Some((model, btn, fix, decrypted as u16));
        }
    }
    None
}

/// Build the 64-bit frame for (model, button, fix, counter) — the data half of the C's get_upload.
fn build_frame(model: Model, btn: u8, fix: u32, cnt: u16) -> u64 {
    let hop_plain = (u32::from(btn) << 24) | ((fix & 0x3FF) << 16) | u32::from(cnt);
    let mut hop = keeloq_encrypt(hop_plain, model.key()).to_le_bytes();
    encrypt_transform(&mut hop, model);
    (u64::from(fix) << 32) | u64::from(u32::from_le_bytes(hop))
}

/// The C upload for one 64-bit frame.
fn upload(data: u64) -> Vec<LevelDuration> {
    let mut signal = Vec::with_capacity(TX_PREAMBLE_PAIRS * 2 + 2 + MIN_COUNT_BIT * 2 + 2);
    for _ in 0..TX_PREAMBLE_PAIRS {
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
    signal.push(LevelDuration::new(true, TE_SHORT));
    signal.push(LevelDuration::new(false, TE_SHORT * 40));
    signal
}

/// Decoder steps (matches `SheriffCfmDecoderStep`).
#[derive(Debug, Clone, Copy, PartialEq)]
enum DecoderStep {
    Reset,
    CheckPreamble,
    SaveDuration,
    CheckDuration,
}

/// Sheriff CFM decoder (matches `SubGhzProtocolDecoderSheriffCfm`).
pub struct SheriffCfmDecoder {
    step: DecoderStep,
    te_last: u32,
    header_count: u16,
    decode_data: u64,
    decode_count_bit: usize,
}

impl SheriffCfmDecoder {
    pub fn new() -> Self {
        Self {
            step: DecoderStep::Reset,
            te_last: 0,
            header_count: 0,
            decode_data: 0,
            decode_count_bit: 0,
        }
    }

    /// Bits beyond 64 are only counted (C: `decode_count_bit++` once 64 bits are stored).
    fn add_bit(&mut self, bit: u64) {
        if self.decode_count_bit < MIN_COUNT_BIT {
            self.decode_data = (self.decode_data << 1) | bit;
        }
        self.decode_count_bit += 1;
    }

    /// KAT generic button → Sheriff CFM code (native codes pass through; others keep `original`).
    fn map_button(button: u8, original: u8) -> u8 {
        match button {
            BTN_LOCK => CFM_LOCK,
            BTN_UNLOCK => CFM_UNLOCK,
            BTN_TRUNK => CFM_TRUNK,
            BTN_PANIC => CFM_PANIC,
            CFM_LOCK | CFM_UNLOCK | CFM_TRUNK | CFM_PANIC => button,
            _ => original,
        }
    }
}

impl ProtocolDecoder for SheriffCfmDecoder {
    fn name(&self) -> &'static str {
        "Sheriff CFM"
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
        &[315_000_000, 433_920_000]
    }

    fn reset(&mut self) {
        self.step = DecoderStep::Reset;
        self.te_last = 0;
        self.header_count = 0;
        self.decode_data = 0;
        self.decode_count_bit = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            DecoderStep::Reset => {
                if level && duration_diff!(duration, TE_SHORT) < TE_DELTA {
                    self.step = DecoderStep::CheckPreamble;
                    self.header_count = self.header_count.wrapping_add(1);
                }
            }
            DecoderStep::CheckPreamble => {
                if !level && duration_diff!(duration, TE_SHORT) < TE_DELTA {
                    self.step = DecoderStep::Reset;
                    return None;
                }
                if self.header_count > 2 && duration_diff!(duration, TE_SHORT * 10) < TE_DELTA * 10
                {
                    self.step = DecoderStep::SaveDuration;
                    self.decode_data = 0;
                    self.decode_count_bit = 0;
                } else {
                    self.step = DecoderStep::Reset;
                    self.header_count = 0;
                }
            }
            DecoderStep::SaveDuration => {
                if level {
                    self.te_last = duration;
                    self.step = DecoderStep::CheckDuration;
                }
            }
            DecoderStep::CheckDuration => {
                if !level {
                    if duration >= TE_SHORT * 2 + TE_DELTA {
                        self.step = DecoderStep::Reset;
                        let mut result = None;
                        if (MIN_COUNT_BIT..=MIN_COUNT_BIT + 2).contains(&self.decode_count_bit) {
                            if let Some((_model, btn, serial, cnt)) = try_decrypt(self.decode_data)
                            {
                                result = Some(DecodedSignal {
                                    serial: Some(serial),
                                    button: Some(btn),
                                    counter: Some(cnt),
                                    crc_valid: true,
                                    data: self.decode_data,
                                    data_count_bit: MIN_COUNT_BIT,
                                    encoder_capable: true,
                                    extra: None,
                                    protocol_display_name: None,
                                });
                            }
                        }
                        self.decode_data = 0;
                        self.decode_count_bit = 0;
                        self.header_count = 0;
                        return result;
                    } else if duration_diff!(self.te_last, TE_SHORT) < TE_DELTA
                        && duration_diff!(duration, TE_LONG) < TE_DELTA * 2
                    {
                        self.add_bit(1);
                        self.step = DecoderStep::SaveDuration;
                    } else if duration_diff!(self.te_last, TE_LONG) < TE_DELTA * 2
                        && duration_diff!(duration, TE_SHORT) < TE_DELTA
                    {
                        self.add_bit(0);
                        self.step = DecoderStep::SaveDuration;
                    } else {
                        self.step = DecoderStep::Reset;
                        self.header_count = 0;
                    }
                } else {
                    self.step = DecoderStep::Reset;
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
        // Model (and the defaults for button/counter) come from the decrypt, as in the C encoder's
        // deserialize; the signal's own button/counter take precedence (the C's Btn/Cnt fields).
        let (model, det_btn, _serial, det_cnt) = try_decrypt(decoded.data)?;
        let btn = Self::map_button(button, decoded.button.unwrap_or(det_btn));
        let cnt = decoded.counter.unwrap_or(det_cnt);
        // C: if (cnt + mult) > 0xFFFF then cnt = 0 else cnt += mult (mult = 1).
        let cnt = cnt.checked_add(1).unwrap_or(0);
        let fix = (decoded.data >> 32) as u32;
        let once = upload(build_frame(model, btn, fix, cnt));
        Some(once.repeat(TX_REPEAT))
    }
}

impl Default for SheriffCfmDecoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::ProtocolRegistry;

    fn decode_all(signal: &[LevelDuration]) -> Vec<DecodedSignal> {
        let mut dec = SheriffCfmDecoder::new();
        signal
            .iter()
            .filter_map(|ld| dec.feed(ld.level, ld.duration_us))
            .collect()
    }

    fn registry_names(signal: &[LevelDuration], freq: u32) -> Vec<String> {
        let mut reg = ProtocolRegistry::new();
        reg.process_signal_stream(signal, freq)
            .into_iter()
            .map(|(n, _, _)| n)
            .collect()
    }

    /// Simple deterministic generator for test vectors.
    fn lcg(state: &mut u64) -> u64 {
        *state = state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        *state >> 11
    }

    #[test]
    fn sheriff_cfm_keys_present() {
        let [zx750, zx930] = *mf_keys();
        assert_ne!(zx750, 0, "ZX-750 key missing from keystore");
        assert_ne!(zx930, 0, "ZX-930 key missing from keystore");
        assert_ne!(zx750, zx930);
    }

    #[test]
    fn sheriff_cfm_transforms_are_inverse() {
        let mut s = 7u64;
        for _ in 0..1000 {
            let orig = (lcg(&mut s) as u32).to_le_bytes();
            for model in MODELS {
                let mut b = orig;
                encrypt_transform(&mut b, model);
                decrypt_transform(&mut b, model);
                assert_eq!(b, orig, "{model:?}");
            }
        }
    }

    #[test]
    fn sheriff_cfm_decodes_synthetic_frames() {
        for model in MODELS {
            let fix = 0x5A3C_10C5; // fix & 0x3FF = 0x0C5 (bits 9:8 clear, required by the check)
            let data = build_frame(model, CFM_TRUNK, fix, 0x1234);
            let decodes = decode_all(&upload(data));
            assert_eq!(decodes.len(), 1, "{model:?}");
            let d = &decodes[0];
            assert_eq!(d.data, data);
            assert_eq!(d.serial, Some(fix));
            assert_eq!(d.button, Some(CFM_TRUNK));
            assert_eq!(d.counter, Some(0x1234));
            assert!(d.crc_valid && d.encoder_capable);
            assert_eq!(try_decrypt(data).map(|r| r.0), Some(model));
        }
    }

    #[test]
    fn sheriff_cfm_encode_decode_roundtrip() {
        let enc = SheriffCfmDecoder::new();
        for model in MODELS {
            let fix = 0x0BAD_C0DE & !0x300;
            let seed = decode_all(&upload(build_frame(model, CFM_LOCK, fix, 0x00FF)))
                .pop()
                .expect("seed decode");
            for (button, code) in [
                (BTN_LOCK, CFM_LOCK),
                (BTN_UNLOCK, CFM_UNLOCK),
                (BTN_TRUNK, CFM_TRUNK),
                (BTN_PANIC, CFM_PANIC),
                (CFM_UNLOCK, CFM_UNLOCK), // native code passes through
                (0x03, CFM_LOCK),         // unknown → captured button
            ] {
                let signal = enc.encode(&seed, button).expect("encode");
                let d = decode_all(&signal).pop().expect("decode");
                assert_eq!(d.serial, Some(fix));
                assert_eq!(d.button, Some(code), "{model:?} button {button:#x}");
                // Fresh press: counter + 1, same model.
                assert_eq!(d.counter, Some(0x0100));
                assert_eq!(try_decrypt(d.data).map(|r| r.0), Some(model));
            }
        }
        // Counter wraps to 0 past 0xFFFF (C semantics).
        let seed = decode_all(&upload(build_frame(
            Model::Zx930,
            CFM_LOCK,
            0x1234_0012,
            0xFFFF,
        )))
        .pop()
        .unwrap();
        let d = decode_all(&enc.encode(&seed, BTN_LOCK).unwrap())
            .pop()
            .unwrap();
        assert_eq!(d.counter, Some(0));
    }

    #[test]
    fn sheriff_cfm_rejects_corrupted_frames() {
        let fix = 0x5A3C_10C5;
        let data = build_frame(Model::Zx750, CFM_UNLOCK, fix, 0x0042);
        assert_eq!(decode_all(&upload(data)).len(), 1);
        // Corrupted hop bit → decrypt check fails.
        assert!(decode_all(&upload(data ^ 0x0000_0000_0001_0000)).is_empty());
        // Low serial bits no longer match the encrypted copy.
        assert!(decode_all(&upload(data ^ (1u64 << 32))).is_empty());
        // 63 bits only (drop the last data cell) → bit count gate.
        let mut short = upload(data);
        let tail = short.split_off(short.len() - 4);
        short.extend_from_slice(&tail[2..]);
        assert!(decode_all(&short).is_empty());
        // Encoder refuses a frame it cannot decrypt.
        let mut sig = decode_all(&upload(data)).pop().unwrap();
        sig.data ^= 1;
        assert!(SheriffCfmDecoder::new().encode(&sig, BTN_LOCK).is_none());
    }

    #[test]
    fn sheriff_cfm_does_not_steal_keeloq_frames() {
        // A standard KeeLoq frame (DoorHan, simple learning, bit-reversed layout) re-encoded by KAT's
        // KeeLoq encoder is still reported as KeeLoq, never as Sheriff CFM.
        use crate::protocols::keeloq_common::reverse_key;
        let reg = ProtocolRegistry::new();
        let keeloq = reg.get("KeeLoq").expect("KeeLoq decoder");
        let key = keystore::keeloq_mf_keys_with_names()
            .into_iter()
            .find(|(n, _)| n == "DoorHan")
            .map(|(_, k)| k)
            .expect("DoorHan key");
        let fix: u32 = (0x2 << 28) | 0x0123_4567;
        let plain: u32 = (0x2 << 28) | (0x167 << 16) | 0x0040;
        let hop = keeloq_encrypt(plain, key);
        let seed = DecodedSignal {
            serial: Some(0x0123_4567),
            button: Some(0x2),
            counter: Some(0x0040),
            crc_valid: true,
            data: reverse_key((u64::from(fix) << 32) | u64::from(hop), 64),
            data_count_bit: 64,
            encoder_capable: true,
            extra: None,
            protocol_display_name: None,
        };
        let signal = keeloq.encode(&seed, 0x2).expect("KeeLoq encode");
        assert!(
            decode_all(&signal).is_empty(),
            "Sheriff CFM must not decode a KeeLoq frame"
        );
        let names = registry_names(&signal, 433_920_000);
        assert!(
            !names.is_empty() && names.iter().all(|n| n.starts_with("KeeLoq")),
            "got {names:?}"
        );
    }

    /// Diagnostic: which registry decoder reports genuine (C-encoded) Sheriff frames, per band.
    /// An earlier decoder that accepts the same KeeLoq-PWM framing (`keeloq`'s keystore selector)
    /// would win the shared terminating pulse; Sheriff CFM is registered before it.
    /// `cargo test sheriff_cfm_registry_claims -- --ignored --nocapture`.
    #[test]
    #[ignore]
    fn sheriff_cfm_registry_claims() {
        use std::collections::BTreeMap;
        for freq in [433_920_000u32, 315_000_000] {
            let mut s = 0x5EED_u64;
            let mut claims: BTreeMap<String, usize> = BTreeMap::new();
            let n = 400;
            for i in 0..n {
                let model = MODELS[i % 2];
                let btn = [CFM_LOCK, CFM_UNLOCK, CFM_TRUNK, CFM_PANIC][(lcg(&mut s) % 4) as usize];
                let fix = (lcg(&mut s) as u32) & !0x300;
                let data = build_frame(model, btn, fix, lcg(&mut s) as u16);
                let first = registry_names(&upload(data), freq).into_iter().next();
                let key = match first {
                    Some(name) if name.starts_with("KeeLoq") => "KeeLoq (*)".to_string(),
                    Some(name) => name,
                    None => "<none>".to_string(),
                };
                *claims.entry(key).or_default() += 1;
            }
            eprintln!("{n} Sheriff frames @ {freq} Hz: {claims:?}");
        }
    }
}
