//! KeeLoq protocol decoder and encoder (unleashed format).
//!
//! Timing and state machine match Flipper Unleashed:
//! `REFERENCES/unleashed-firmware/lib/subghz/protocols/keeloq.c`
//! - te_short=400µs, te_long=800µs, te_delta=140µs, 64 data bits.
//! - Preamble: HIGH pulses ~400µs; when LOW and header_count>2 and LOW ~4000µs, start data.
//! - Data: short HIGH + long LOW = 1, long HIGH + short LOW = 0. End when LOW ≥ 940µs.
//! Decryption tries all keystore keys with simple, normal, secure, magic_xor_type1,
//! magic_serial type1/2/3 (and both key byte orders). Encoder uses simple learning
//! and the key stored in DecodedSignal::extra (set when decoded).

use super::common::DecodedSignal;
use super::keeloq_common::{
    keeloq_decrypt, keeloq_encrypt, keeloq_magic_serial_type1_learning,
    keeloq_magic_serial_type2_learning, keeloq_magic_serial_type3_learning,
    keeloq_magic_xor_type1_learning, keeloq_normal_learning, keeloq_secure_learning,
    reverse_key,
};
use crate::keystore;
use crate::radio::demodulator::LevelDuration;

const TE_SHORT: u32 = 400;
const TE_LONG: u32 = 800;
const TE_DELTA: u32 = 140;
const MIN_COUNT_BIT: usize = 64;

fn duration_diff(a: u32, b: u32) -> u32 {
    if a < b {
        b - a
    } else {
        a - b
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Step {
    Reset,
    CheckPreamble,
    SaveDuration,
    CheckDuration,
}

/// Try to decrypt (fix, hop) with all keystore keys and learning types.
/// Returns (manufacture_name, serial, counter, button, key_for_encode) on success.
/// key_for_encode is Some(mf_key) when a keystore key was used, None for AN-Motors/HCS101.
fn try_keeloq_decrypt(
    fix: u32,
    hop: u32,
    seed: u32,
) -> Option<(String, u32, u16, u8, Option<u64>)> {
    let end_serial = (fix & 0xFF) as u8;
    let btn = (fix >> 28) as u8;

    fn check_decrypt(decrypt: u32, btn: u8, end_serial: u8) -> bool {
        (decrypt >> 28) as u8 == btn
            && (((decrypt >> 16) & 0xFF) as u8 == end_serial || ((decrypt >> 16) & 0xFF) == 0)
    }

    let keys = keystore::keeloq_mf_keys_with_names();
    for (name, mf_key) in keys {
        for key in [mf_key, mf_key.swap_bytes()] {
            if key == 0 {
                continue;
            }
            // Simple
            let decrypt = keeloq_decrypt(hop, key);
            if check_decrypt(decrypt, btn, end_serial) {
                let cnt = (decrypt & 0xFFFF) as u16;
                let serial = fix & 0x0FFFFFFF;
                return Some((name, serial, cnt, btn, Some(mf_key)));
            }
            // Normal
            let man = keeloq_normal_learning(fix, key);
            let decrypt = keeloq_decrypt(hop, man);
            if check_decrypt(decrypt, btn, end_serial) {
                let cnt = (decrypt & 0xFFFF) as u16;
                let serial = fix & 0x0FFFFFFF;
                return Some((name, serial, cnt, btn, Some(mf_key)));
            }
            // Secure (seed 0 and seed = fix for BFT-style)
            for s in [0u32, seed] {
                let man = keeloq_secure_learning(fix, s, key);
                let decrypt = keeloq_decrypt(hop, man);
                if check_decrypt(decrypt, btn, end_serial) {
                    let cnt = (decrypt & 0xFFFF) as u16;
                    let serial = fix & 0x0FFFFFFF;
                    return Some((name, serial, cnt, btn, Some(mf_key)));
                }
            }
            // Magic XOR type1
            let man = keeloq_magic_xor_type1_learning(fix, key);
            let decrypt = keeloq_decrypt(hop, man);
            if check_decrypt(decrypt, btn, end_serial) {
                let cnt = (decrypt & 0xFFFF) as u16;
                let serial = fix & 0x0FFFFFFF;
                return Some((name, serial, cnt, btn, Some(mf_key)));
            }
            // Magic serial type 1/2/3
            for man in [
                keeloq_magic_serial_type1_learning(fix, key),
                keeloq_magic_serial_type2_learning(fix, key),
                keeloq_magic_serial_type3_learning(fix & 0xFFFFFF, key),
            ] {
                let decrypt = keeloq_decrypt(hop, man);
                if check_decrypt(decrypt, btn, end_serial) {
                    let cnt = (decrypt & 0xFFFF) as u16;
                    let serial = fix & 0x0FFFFFFF;
                    return Some((name, serial, cnt, btn, Some(mf_key)));
                }
            }
        }
    }

    // AN-Motors / HCS101 special cases (no decrypt, no key for encode)
    if (hop >> 24) == ((hop >> 16) & 0xFF)
        && (fix >> 28) == ((hop >> 12) & 0x0F)
        && (hop & 0xFFF) == 0x404
    {
        return Some((
            "AN-Motors".to_string(),
            fix & 0x0FFFFFFF,
            (hop >> 16) as u16,
            btn,
            None,
        ));
    }
    if (hop & 0xFFF) == 0 && (fix >> 28) == ((hop >> 12) & 0x0F) {
        return Some((
            "HCS101".to_string(),
            fix & 0x0FFFFFFF,
            (hop >> 16) as u16,
            btn,
            None,
        ));
    }

    None
}

pub struct KeeloqDecoder {
    step: Step,
    header_count: u16,
    te_last: u32,
    decode_data: u64,
    decode_count_bit: usize,
    /// For secure learning when we don't have a prior decode (use fix as seed hint)
    seed: u32,
}

impl KeeloqDecoder {
    pub fn new() -> Self {
        Self {
            step: Step::Reset,
            header_count: 0,
            te_last: 0,
            decode_data: 0,
            decode_count_bit: 0,
            seed: 0,
        }
    }

    fn add_bit(&mut self, bit: u8) {
        self.decode_data = (self.decode_data << 1) | (bit as u64);
        self.decode_count_bit += 1;
    }

    fn reset_to(&mut self, step: Step) {
        self.step = step;
        if step == Step::Reset {
            self.header_count = 0;
        }
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
        &[315_000_000, 433_920_000, 868_350_000]
    }

    fn reset(&mut self) {
        self.step = Step::Reset;
        self.header_count = 0;
        self.te_last = 0;
        self.decode_data = 0;
        self.decode_count_bit = 0;
    }

    fn feed(&mut self, level: bool, duration_us: u32) -> Option<DecodedSignal> {
        match self.step {
            Step::Reset => {
                if level && duration_diff(duration_us, TE_SHORT) < TE_DELTA {
                    self.step = Step::CheckPreamble;
                    self.header_count = self.header_count.saturating_add(1);
                }
            }
            Step::CheckPreamble => {
                if !level && duration_diff(duration_us, TE_SHORT) < TE_DELTA {
                    self.step = Step::Reset;
                    return None;
                }
                if self.header_count > 2
                    && duration_diff(duration_us, TE_SHORT * 10) < TE_DELTA * 10
                {
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
                    self.te_last = duration_us;
                    self.step = Step::CheckDuration;
                }
            }
            Step::CheckDuration => {
                if !level {
                    let end_threshold = TE_SHORT * 2 + TE_DELTA;
                    if duration_us >= end_threshold {
                        self.step = Step::Reset;
                        if self.decode_count_bit >= MIN_COUNT_BIT
                            && self.decode_count_bit <= MIN_COUNT_BIT + 2
                        {
                            let raw_data = self.decode_data;
                            let reversed = reverse_key(raw_data, MIN_COUNT_BIT);
                            let key_fix = (reversed >> 32) as u32;
                            let key_hop = (reversed & 0xFFFFFFFF) as u32;
                            if let Some((_mf_name, serial, cnt, btn, key_for_encode)) =
                                try_keeloq_decrypt(key_fix, key_hop, self.seed)
                            {
                                if self.seed == 0 {
                                    self.seed = key_fix & 0x0FFFFFFF;
                                }
                                self.decode_data = 0;
                                self.decode_count_bit = 0;
                                self.header_count = 0;
                                return Some(DecodedSignal {
                                    serial: Some(serial),
                                    button: Some(btn),
                                    counter: Some(cnt),
                                    crc_valid: true,
                                    data: raw_data,
                                    data_count_bit: MIN_COUNT_BIT,
                                    encoder_capable: true,
                                    extra: key_for_encode,
                                    protocol_display_name: Some(format!("KeeLoq ({})", _mf_name)),
                                });
                            }
                        }
                        self.decode_data = 0;
                        self.decode_count_bit = 0;
                        self.header_count = 0;
                        return None;
                    }
                    // Bit 1: te_last short, duration long
                    if duration_diff(self.te_last, TE_SHORT) < TE_DELTA
                        && duration_diff(duration_us, TE_LONG) < TE_DELTA * 2
                    {
                        if self.decode_count_bit < MIN_COUNT_BIT {
                            self.add_bit(1);
                        } else {
                            self.decode_count_bit += 1;
                        }
                        self.step = Step::SaveDuration;
                        return None;
                    }
                    // Bit 0: te_last long, duration short
                    if duration_diff(self.te_last, TE_LONG) < TE_DELTA * 2
                        && duration_diff(duration_us, TE_SHORT) < TE_DELTA
                    {
                        if self.decode_count_bit < MIN_COUNT_BIT {
                            self.add_bit(0);
                        } else {
                            self.decode_count_bit += 1;
                        }
                        self.step = Step::SaveDuration;
                        return None;
                    }
                    self.reset_to(Step::Reset);
                } else {
                    self.reset_to(Step::Reset);
                }
            }
        }
        None
    }

    fn supports_encoding(&self) -> bool {
        true
    }

    fn encode(&self, decoded: &DecodedSignal, button: u8) -> Option<Vec<LevelDuration>> {
        let serial = decoded.serial?;
        let counter = decoded.counter.unwrap_or(0).wrapping_add(1);
        let key = decoded.extra;
        let fix = ((button as u32) << 28) | (serial & 0x0FFFFFFF);
        let plaintext = ((button as u32) << 28)
            | ((serial & 0x3FF) << 16)
            | (counter as u32);
        let hop = if let Some(k) = key {
            keeloq_encrypt(plaintext, k)
        } else {
            let reversed = reverse_key(decoded.data, MIN_COUNT_BIT);
            (reversed & 0xFFFFFFFF) as u32
        };
        let yek = ((fix as u64) << 32) | (hop as u64);
        let data = reverse_key(yek, MIN_COUNT_BIT);

        let mut signal = Vec::with_capacity(256);
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
        signal.push(LevelDuration::new(true, TE_SHORT));
        signal.push(LevelDuration::new(false, TE_LONG));
        signal.push(LevelDuration::new(true, TE_SHORT));
        signal.push(LevelDuration::new(false, TE_SHORT * 40));

        Some(signal)
    }
}
