//! Honda V8 protocol decoder (**decode only**, as the reference)
//!
//! Ported from FoxFW `lib/subghz/protocols/honda_v8.c` and `honda_v8.h` (itself a port of the
//! KeeLoq-RX `arke_honda.py` `HondaV8` class); no other reference carries it. The C has no encoder
//! (`.encoder = NULL`), so neither does KAT.
//!
//! **Air format** (AM/OOK, 315 / 433.92 MHz): KeeLoq-style inverted PWM with a 400 µs base: the
//! bit is the width of each LOW pulse (400 +- 150 µs = 0, anything else = 1; HIGH pulses are
//! ignored), sync = a LOW of at least 2500 µs. 66 LOW pulses make a frame; the first 64 are kept,
//! each shifted in from the top, and the 64-bit word is bit-reversed to `raw` (first bit
//! received = MSB). The last two bits are counted but not stored. A second sync LOW with more than
//! 60 bits also processes the frame.
//!
//! **Frame**: `raw[63:60]` button, `raw[59:32]` serial (28 bits), `raw[31:0]` the KeeLoq
//! ciphertext. **The integrity check is a decryption match**: for each of 21 Honda manufacturer
//! keys (probe order 14, 1, 0, 2..13, 15..20) the ciphertext is decrypted with the C's own routine
//! (528 right-shifting rounds, NLF 0x3A5C742E, key bit `(15 - i) & 63`) and must give
//! `dec & 0xF == button` and `(dec >> 4) & 0xFF == serial >> 20 & 0xFF`; the all-zero case
//! (`dec >> 16 == 0` and discriminant 0) is rejected. The counter is `dec >> 16`. The button must
//! be 0, 1, 2, 4, 8 or 9 and the serial neither 0 nor 0x0FFFFFFF. Finally a rolling window: the
//! decoder remembers the last counter (it survives `reset`, as in the C, where it belongs to the
//! persistent decoder) and a frame is accepted only when `1 <= counter - last <= 128` (mod
//! 2^16), so a press repeated with the same counter is not reported twice, and the first frame
//! after the decoder was created always is.
//!
//! The 21 keys are the C's own table (KeeLoq-RX); KAT does not export them.
//!
//! **KAT fields**: `data` = `raw`, `data_count_bit` 66 (as the C), `serial`, `button`,
//! `counter` = `dec >> 16`, `extra` = the index of the matching key (the C `key_idx`),
//! `crc_valid` true. Key files: Protocol "Honda V8", Bit 66, Key, Serial, Btn, Cnt, KeyIdx, Enc;
//! loading one re-runs the key match.
//!
//! No capture of this protocol exists in the test corpus: the port is validated against the C
//! with synthetic pulse sequences only.
//!
//! **Not registered.** The decryption match is the only integrity check and it is weak: with 21
//! keys, a valid button and a free 28-bit serial, about 21 / 4096 (0.5%) of random 66-bit frames
//! pass (the first one always: the rolling window only filters later ones). The decoder also
//! completes on the 66th LOW pulse, so it reads the first 66 bits of any longer PWM frame that
//! follows a 2500 µs gap before that frame's own decoder finishes: KAT's own G6 test
//! (`registry_attributes_g6_uploads_to_their_own_protocol`) had one of sixteen Nice One uploads
//! reported as Honda V8, whatever its place in the registry. The corpus sweep (`kat_diff` over
//! IMPORTS and the Flipper-ARF unit-test captures) shows no change, but a decoder that mislabels
//! known protocols is left out; enabling it is one line in `src/protocols/mod.rs`.

use super::info::prelude::*;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

/// Metadata (see `info.rs`): the C's `SubGhzProtocolFlag_433 | 315 | AM`.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Pwm,
    rf: Rf::AM,
    encryption: "KeeLoq",
    make: "Honda/Acura",
    category: Category::Vehicle,
    buttons: Some(native_buttons),
    presets: &[&AM650],
};

/// The C's button comments: 0 Panic, 1 Lock, 2 Trunk, 4 Remote Start, 8 Unlock, 9 Unlock x2.
fn native_buttons(_name: &str, code: u8) -> Option<&'static str> {
    match code {
        0x0 => Some("Panic"),
        0x1 => Some("Lock"),
        0x2 => Some("Trunk"),
        0x4 => Some("Remote start"),
        0x8 | 0x9 => Some("Unlock"),
        _ => None,
    }
}

const NAME: &str = "Honda V8";
const TE_BASE: u32 = 400;
const TE_DELTA: u32 = 150;
const SYNC_MIN: u32 = 2500;
const FRAME_BITS: u16 = 66;
const ROLL_WIN: u16 = 128;
const NLF: u32 = 0x3A5C_742E;
const ROUNDS: u32 = 528;

/// Honda manufacturer key table, (low, high) halves, 21 entries.
const KEYS_LO: [u32; 21] = [
    0xA8F5_DFFC, 0xABBA_94D4, 0xDCF4_0FCD, 0x56EB_02D8, 0xA850_7BA5, 0x6857_C1C5, 0x54E9_00DA,
    0x5683_FC9B, 0x53EE_07DD, 0x5D97_3077, 0x52EF_06DC, 0x3671_4027, 0x51EC_05DF, 0x1F4D_AA96,
    0x50ED_04DE, 0xDC8D_C8EA, 0x3085_0A82, 0xD520_4919, 0x558E_A3B0, 0x5445_4441, 0x4080_2014,
];
const KEYS_HI: [u32; 21] = [
    0x8DAA_5CDB, 0x10C1_F940, 0x4601_14B7, 0xE9B9_77B4, 0x3F26_FA21, 0xA4C5_C542, 0xEBBB_753E,
    0x6A67_C523, 0xECBC_7263, 0xE1CA_6024, 0xEDBD_737C, 0x6011_AC25, 0xEEBE_7059, 0x9BB8_7A26,
    0xEFBF_713C, 0xF39F_D627, 0x610A_14EB, 0x1719_FF41, 0x779F_14DA, 0x4745_4DE2, 0x0514_F348,
];
const PROBE: [u8; 21] = [14, 1, 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 15, 16, 17, 18, 19, 20];

#[derive(Clone, Copy, PartialEq, Eq)]
enum Step {
    Reset,
    Data,
}

/// Honda V8 decoder (matches SubGhzProtocolDecoderHondaV8).
pub struct HondaV8Decoder {
    step: Step,
    data: u64,
    count: u16,
    have_last: bool,
    last_rc: u16,
}

fn btn_valid(btn: u8) -> bool {
    matches!(btn, 0x0 | 0x1 | 0x2 | 0x4 | 0x8 | 0x9)
}

/// `honda_v8_nlf`.
fn nlf(x: u32) -> u32 {
    let idx = ((x >> 1) & 1)
        | (((x >> 9) & 1) << 1)
        | (((x >> 20) & 1) << 2)
        | (((x >> 26) & 1) << 3)
        | (((x >> 31) & 1) << 4);
    (NLF >> idx) & 1
}

/// Key bit used by round `i` (`(15 - i) & 0x3F`).
fn key_bit(key: u64, i: u32) -> u32 {
    ((key >> (15u32.wrapping_sub(i) & 0x3F)) & 1) as u32
}

/// `honda_v8_decrypt`: 528 right-shifting rounds.
fn decrypt(ct: u32, key: u64) -> u32 {
    let mut x = ct;
    for i in 0..ROUNDS {
        let fb = (x & 1) ^ nlf(x) ^ ((x >> 15) & 1) ^ key_bit(key, i);
        x = (x >> 1) | (fb << 31);
    }
    x
}

/// `honda_v8_find_key`: (key index, dec, counter) of the first probe-order key that matches.
fn find_key(enc: u32, serial: u32, btn: u8) -> Option<(u8, u32, u16)> {
    let exp_disc = ((serial >> 20) & 0xFF) as u8;
    for &ki in &PROBE {
        let k = (u64::from(KEYS_HI[usize::from(ki)]) << 32) | u64::from(KEYS_LO[usize::from(ki)]);
        let dec = decrypt(enc, k);
        if (dec & 0xF) as u8 != btn {
            continue;
        }
        let disc = ((dec & 0x0FF0) >> 4) as u8;
        if disc != exp_disc {
            continue;
        }
        let rc = ((dec & 0xFFFF_0000) >> 16) as u16;
        if rc == 0 && disc == 0 {
            continue;
        }
        return Some((ki, dec, rc));
    }
    None
}

/// The fields of a `raw` word whose key matched.
fn fields(raw: u64, key_idx: u8, rc: u16) -> DecodedSignal {
    DecodedSignal {
        serial: Some(((raw >> 32) & 0x0FFF_FFFF) as u32),
        button: Some(((raw >> 60) & 0x0F) as u8),
        counter: Some(rc),
        crc_valid: true,
        data: raw,
        data_count_bit: usize::from(FRAME_BITS),
        encoder_capable: false,
        extra: Some(u64::from(key_idx)),
        protocol_display_name: None,
    }
}

impl HondaV8Decoder {
    pub fn new() -> Self {
        Self { step: Step::Reset, data: 0, count: 0, have_last: false, last_rc: 0 }
    }

    /// `honda_v8_process`.
    fn process(&mut self) -> Option<DecodedSignal> {
        let raw = self.data.reverse_bits();
        let enc = raw as u32;
        let serial = ((raw >> 32) & 0x0FFF_FFFF) as u32;
        let btn = ((raw >> 60) & 0x0F) as u8;
        if !btn_valid(btn) {
            return None;
        }
        if serial == 0 || serial == 0x0FFF_FFFF {
            return None;
        }
        let (ki, _dec, rc) = find_key(enc, serial, btn)?;
        if self.have_last {
            let d = rc.wrapping_sub(self.last_rc);
            if d == 0 || d > ROLL_WIN {
                return None;
            }
        }
        self.last_rc = rc;
        self.have_last = true;
        Some(fields(raw, ki, rc))
    }
}

impl Default for HondaV8Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for HondaV8Decoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: TE_BASE, te_long: TE_BASE * 2, te_delta: TE_DELTA, min_count_bit: usize::from(FRAME_BITS) }
    }

    fn supported_frequencies(&self) -> &[u32] {
        &[315_000_000, 433_920_000]
    }

    /// As the C: the rolling-window memory (`have_last` / `last_rc`) is kept.
    fn reset(&mut self) {
        self.step = Step::Reset;
        self.data = 0;
        self.count = 0;
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                if !level && duration >= SYNC_MIN {
                    self.data = 0;
                    self.count = 0;
                    self.step = Step::Data;
                }
                None
            }
            Step::Data => {
                if level {
                    return None;
                }
                if duration >= SYNC_MIN {
                    let out = if self.count == FRAME_BITS || self.count > 60 { self.process() } else { None };
                    self.data = 0;
                    self.count = 0;
                    return out;
                }
                let bit = !(duration + TE_DELTA >= TE_BASE && duration <= TE_BASE + TE_DELTA);
                if self.count < 64 {
                    self.data >>= 1;
                    if bit {
                        self.data |= 1 << 63;
                    }
                }
                self.count += 1;
                if self.count >= FRAME_BITS {
                    let out = self.process();
                    self.data = 0;
                    self.count = 0;
                    self.step = Step::Reset;
                    return out;
                }
                None
            }
        }
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }

    /// `subghz_protocol_decoder_honda_v8_deserialize`: the key match is re-run; a key that no
    /// Honda key decrypts is not loaded.
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != usize::from(FRAME_BITS) {
            return None;
        }
        let raw = key.key_u64()?;
        let serial = ((raw >> 32) & 0x0FFF_FFFF) as u32;
        let btn = ((raw >> 60) & 0x0F) as u8;
        let (ki, _, rc) = find_key(raw as u32, serial, btn)?;
        Some(fields(raw, ki, rc))
    }

    /// `subghz_protocol_decoder_honda_v8_serialize`.
    fn to_flipper_key(&self, decoded: &DecodedSignal) -> Option<FlipperKey> {
        if decoded.data_count_bit != usize::from(FRAME_BITS) {
            return None;
        }
        let mut key = FlipperKey::for_protocol(NAME, usize::from(FRAME_BITS), decoded.data);
        key.push_u32("Serial", decoded.serial?);
        key.push_u32("Btn", decoded.button?.into());
        key.push_u32("Cnt", decoded.counter?.into());
        key.push_u32("KeyIdx", decoded.extra? as u32);
        key.push_u32("Enc", decoded.data as u32);
        Some(key)
    }
}

#[cfg(test)]
mod tests {
    use super::super::honda_synth::{decode, dump, Pulse};
    use super::*;

    const PRESET: &str = "FuriHalSubGhzPresetOok650Async";

    /// Inverse of one `decrypt` round: the state before the round that produced `x_next`.
    fn encrypt(plain: u32, key: u64) -> u32 {
        let mut x = plain;
        for i in (0..ROUNDS).rev() {
            let fb = x >> 31;
            let low = x << 1; // bit 0 still unknown
            let x0 = fb ^ nlf(low) ^ ((low >> 15) & 1) ^ key_bit(key, i);
            x = low | x0;
        }
        x
    }

    /// A frame for made-up fields under Honda key `ki`: dec = counter << 16 | disc << 4 | button.
    fn raw_word(button: u8, serial: u32, counter: u16, ki: usize) -> u64 {
        let key = (u64::from(KEYS_HI[ki]) << 32) | u64::from(KEYS_LO[ki]);
        let dec = (u32::from(counter) << 16) | ((serial >> 20) & 0xFF) << 4 | u32::from(button);
        let enc = encrypt(dec, key);
        assert_eq!(decrypt(enc, key), dec);
        (u64::from(button) << 60) | (u64::from(serial) << 32) | u64::from(enc)
    }

    /// Sync LOW, then one LOW per bit (400 = 0, 800 = 1) separated by 400 µs HIGHs; 66 bits (the
    /// last two arbitrary), closing HIGH.
    fn burst(raw: u64, extra_bits: [bool; 2], n: usize) -> Vec<Pulse> {
        let mut p: Vec<Pulse> = vec![(true, 400), (false, 3000)];
        let mut bits: Vec<bool> = (0..64).rev().map(|i| (raw >> i) & 1 == 1).collect();
        bits.extend(extra_bits);
        for &b in bits.iter().take(n) {
            p.push((true, 400));
            p.push((false, if b { 800 } else { 400 }));
        }
        p.push((true, 400));
        p.push((false, 6000));
        p
    }

    #[test]
    fn honda_v8_decodes_synthetic_frames_and_the_rolling_window() {
        let serial = 0x0A5_1234;
        let mut dec = HondaV8Decoder::new();
        let mut all: Vec<Pulse> = Vec::new();
        // Counter 1000, a repeat of it, then 1001, then a jump of 500 (outside the window).
        for (btn, ctr, ki) in [(1u8, 1000u16, 14usize), (1, 1000, 14), (1, 1001, 14), (8, 1501, 14), (2, 1502, 3)] {
            all.extend(burst(raw_word(btn, serial, ctr, ki), [true, false], 66));
        }
        dump("honda_v8_sequence", &all, PRESET);
        let got = decode(&mut dec, &all);
        let seen: Vec<(u8, u16, u64)> =
            got.iter().map(|d| (d.button.unwrap(), d.counter.unwrap(), d.extra.unwrap())).collect();
        assert_eq!(seen, [(1, 1000, 14), (1, 1001, 14)]);
        assert!(got.iter().all(|d| d.serial == Some(serial) && d.crc_valid && d.data_count_bit == 66));
        assert!(got[0].extra.is_some());
        assert_eq!(native_buttons(NAME, 8), Some("Unlock"));
    }

    #[test]
    fn honda_v8_other_keys_gating_and_short_frames() {
        let serial = 0x0A5_1234;
        // The key index reported is the first in probe order that decrypts: here key 3 only.
        let mut dec = HondaV8Decoder::new();
        let p = burst(raw_word(4, serial, 77, 3), [false, false], 66);
        dump("honda_v8_key3", &p, PRESET);
        let got = decode(&mut dec, &p);
        assert_eq!(got.len(), 1);
        assert_eq!((got[0].button, got[0].counter, got[0].extra), (Some(4), Some(77), Some(3)));
        // A flipped ciphertext bit: no key matches.
        let mut bad = raw_word(4, serial, 78, 3);
        bad ^= 1 << 5;
        let p = burst(bad, [false, false], 66);
        dump("honda_v8_bad_ciphertext", &p, PRESET);
        assert!(decode(&mut HondaV8Decoder::new(), &p).is_empty());
        // A button outside the set (3), even with a matching ciphertext.
        let p = burst(raw_word(3, serial, 5, 3), [false, false], 66);
        dump("honda_v8_bad_button", &p, PRESET);
        assert!(decode(&mut HondaV8Decoder::new(), &p).is_empty());
        // A frame of 55 bits followed by the sync gap is not processed (needs 66, or more than 60).
        let p = burst(raw_word(4, serial, 79, 3), [false, false], 55);
        dump("honda_v8_short_frame", &p, PRESET);
        assert!(decode(&mut HondaV8Decoder::new(), &p).is_empty());
        // No sync: the first LOW is too short to start.
        let mut p = burst(raw_word(4, serial, 80, 3), [false, false], 66);
        p[1] = (false, 2000);
        dump("honda_v8_no_sync", &p, PRESET);
        assert!(decode(&mut HondaV8Decoder::new(), &p).is_empty());
    }

    #[test]
    fn honda_v8_key_file_round_trip() {
        use crate::protocols::flipper_key::testing::assert_round_trip;
        let d = fields(raw_word(1, 0x0A5_1234, 4242, 14), 14, 4242);
        let text = assert_round_trip(&HondaV8Decoder::new(), &d);
        assert!(text.contains("Protocol: Honda V8\nBit: 66\n"), "{text}");
        assert!(text.contains("Cnt: 4242\nKeyIdx: 14\n"), "{text}");
    }
}
