//! Hitag2 RKE: NXP PCF7946/7947 rolling-code remote, Manchester frame with an XOR check (receive
//! only, as in FoxFW).
//!
//! Ported from FoxFW `lib/subghz/protocols/hitag2.c` (`SUBGHZ_PROTOCOL_HITAG2_NAME = "Hitag2"`;
//! `SubGhzProtocolTypeDynamic`, flags `433 | AM`), which follows Garcia et al., "Lock It and Still
//! Lose It" (USENIX Security 2016, Figure 11). It is an air-format decoder, distinct from KAT's
//! `hitag2.rs` (the HITAG2 cipher that Fiat V1, Fiat V2 and Renault V1 use for their key search)
//! and from the Hitag2 key recovery in `hitag2_hell.rs`.
//!
//! **Transmit: none, as in FoxFW.** FoxFW has no Hitag2 encoder to port: the protocol's encoder
//! table in `hitag2.c` is all NULL, so is the one in the SD-plugin source `protocols_ext/
//! enc_hitag2.c` (FoxFW's split-out encoders), which `applications/main/subghz/application.fam`
//! does not even build, and the protocol carries no `Send` flag ("the rolling code is encrypted
//! with a per-remote key, so there is no roundtrip encoder", `hitag2.c`). The host oracle
//! (`subghz_oracle_foxfwenc encode`) answers "no encoder" for each of 100 made-up key files, and
//! `supports_encoding()` stays false. (The "Hitag-2" FoxFW lists with an encoder is its Renault
//! V1, KAT's `renault_v1.rs`.)
//!
//! **Air format**: the decoder collects the pulses of a burst (30 µs and shorter ignored; at most 700,
//! then the burst is processed and restarted; a pulse of 1500 µs or more of either level ends the
//! burst). A burst of at least 88 pulses is processed: `te` is the centre of the most populated 10 µs
//! bin among the pulse widths 70..=320 µs (so one decoder covers te 90 µs Fiat/Alfa to te 240 µs
//! Ford Ka), then a stateless Manchester slicer runs in both polarities (HIGH = 0 first, then HIGH =
//! 1): a pulse within `delta = max(40, te * 45 / 100)` of `2te` is one bit with the pulse's level, a
//! pulse within `delta` of `te` is a half bit (the second half emits the bit, with the second
//! pulse's level), anything else drops the half bit and carries on. The first polarity that
//! yields a frame wins.
//!
//! **Frame** (103 bits, Figure 11): `marker (16, = 0x0001) | UID (32) | button (4) | lctr (10) |
//! keystream (32) | 0 (1) | check (8)`. The scan tries each bit position `cs` as the marker's last
//! bit (so `cs >= 15`) and accepts the first where bit `cs + 79` is 0, the 16 bits ending at `cs`
//! read 0x0001, the XOR of the ten bytes starting at `cs` equals the byte at `cs + 80`, the UID is
//! neither 0 nor 0xFFFFFFFF, and the counter and keystream are not both 0 or both all ones. This
//! checksum-validated alignment removes the preamble-length ambiguity.
//!
//! **Fields**: `serial` = UID, `button` = the 4 bits, `counter` = `lctr` (the low 10 bits of the
//! rolling counter, in clear), `data` = the C's `generic.data` (`UID << 32 | btn << 28 | lctr << 18 |
//! ks >> 14`), `data_count_bit` = 103, `extra` = `check << 32 | keystream`. The keystream is the
//! encrypted part and needs the remote's key (see the HITAG2 key recovery).

use super::g2_common::decoded;
use super::{DecodedSignal, FlipperKey, ProtocolDecoder, ProtocolTiming};
use crate::radio::demodulator::LevelDuration;

use super::info::prelude::*;

/// Metadata (see `info.rs`): `SubGhzProtocolFlag_433 | AM`, dynamic (rolling) code.
pub(super) const INFO: ProtocolInfo = ProtocolInfo {
    name: NAME,
    decoder: NAME,
    aliases: &[],
    name_prefix: None,
    encoding: Coding::Manchester,
    rf: Rf::AM,
    encryption: "Hitag2",
    make: "Hitag2 RKE",
    category: Category::Vehicle,
    buttons: None,
    presets: &[&AM650],
};

const NAME: &str = "Hitag2";
const DATA_BITS: usize = 103;
const ALL_BITS_MAX: usize = 400;
/// Smallest burst (in pulses) and frame (in bits) worth looking at.
const MIN_BITS: usize = 88;
const MAXDUR: usize = 700;
/// A pulse this long ends the burst.
const GAP: u32 = 1500;
const MIN_PULSE: u32 = 30;
/// `te` search window (µs).
const TE_MIN: u32 = 70;
const TE_MAX: u32 = 320;

/// Hitag2 RKE decoder (`SubGhzProtocolDecoderHitag2`).
pub struct Hitag2RkeDecoder {
    /// Signed pulse widths of the current burst (+ HIGH, - LOW).
    durs: Vec<i32>,
}

/// `hitag2_bits_to_int`.
fn bits_to_int(bits: &[u8], lo: usize, len: usize) -> u32 {
    (0..len).fold(0u32, |v, i| (v << 1) | u32::from(bits[lo + i] & 1))
}

/// The decode of a frame's fields (the C's `generic.data` is `UID << 32 | btn << 28 | lctr << 18 | ks >> 14`).
fn signal(uid: u32, btn: u8, lctr: u16, ks: u32, chk: u8) -> DecodedSignal {
    let data = u64::from(uid) << 32
        | u64::from(btn & 0xF) << 28
        | u64::from(lctr & 0x3FF) << 18
        | u64::from(ks >> 14);
    let mut d = decoded(data, DATA_BITS);
    d.serial = Some(uid);
    d.button = Some(btn);
    d.counter = Some(lctr);
    d.extra = Some(u64::from(chk) << 32 | u64::from(ks));
    d.crc_valid = true;
    d
}

/// `subghz_protocol_decoder_hitag2_find_and_emit`.
fn find_and_emit(b: &[u8]) -> Option<DecodedSignal> {
    let n = b.len();
    if n < MIN_BITS {
        return None;
    }
    let mut cs = 0;
    while cs + 88 <= n {
        let at = cs;
        cs += 1;
        // The fixed 0 bit between the keystream and the check byte.
        if b[at + 79] != 0 {
            continue;
        }
        // The 16-bit marker 0x0001 ending at `at`.
        if at < 15 || bits_to_int(b, at - 15, 16) != 0x0001 {
            continue;
        }
        let mut x = 0u8;
        for i in (at..at + 80).step_by(8) {
            x ^= bits_to_int(b, i, 8) as u8;
        }
        let chk = bits_to_int(b, at + 80, 8) as u8;
        if x != chk {
            continue;
        }
        let uid = bits_to_int(b, at + 1, 32);
        if uid == 0 || uid == 0xFFFF_FFFF {
            continue;
        }
        let btn = bits_to_int(b, at + 33, 4) as u8;
        let lctr = bits_to_int(b, at + 37, 10) as u16;
        let ks = bits_to_int(b, at + 47, 32);
        if (lctr == 0 && ks == 0) || (lctr == 0x3FF && ks == 0xFFFF_FFFF) {
            continue;
        }
        return Some(signal(uid, btn, lctr, ks, chk));
    }
    None
}

/// `hitag2_estimate_te`: the centre of the most populated 10 µs bin (the first on a tie).
fn estimate_te(durs: &[i32]) -> u32 {
    const BINS: usize = ((TE_MAX - TE_MIN) / 10 + 1) as usize;
    let mut hist = [0u16; BINS];
    for &v in durs {
        let d = v.unsigned_abs();
        if (TE_MIN..=TE_MAX).contains(&d) {
            hist[((d - TE_MIN) / 10) as usize] += 1;
        }
    }
    let mut best = 0;
    for i in 1..BINS {
        if hist[i] > hist[best] {
            best = i;
        }
    }
    TE_MIN + best as u32 * 10 + 5
}

/// `hitag2_demod`: the bits of one polarity.
fn demod(durs: &[i32], te_short: u32, polarity: bool) -> Vec<u8> {
    let te_long = 2 * te_short;
    let delta = (te_short * 45 / 100).max(40);
    let mut out = Vec::new();
    let mut half = false;
    for &v in durs {
        if out.len() >= ALL_BITS_MAX - 1 {
            break;
        }
        let level = v > 0;
        let d = v.unsigned_abs();
        let is_long = d.abs_diff(te_long) <= delta;
        let is_short = d.abs_diff(te_short) <= delta;
        let bl = u8::from(level == polarity);
        if is_long {
            out.push(bl);
            half = false;
        } else if is_short {
            if half {
                out.push(bl);
                half = false;
            } else {
                half = true;
            }
        } else {
            half = false;
        }
    }
    out
}

impl Hitag2RkeDecoder {
    pub fn new() -> Self {
        Self { durs: Vec::new() }
    }

    /// `subghz_protocol_decoder_hitag2_process_burst`.
    fn process_burst(&self) -> Option<DecodedSignal> {
        if self.durs.len() < MIN_BITS {
            return None;
        }
        let te = estimate_te(&self.durs);
        [false, true].into_iter().find_map(|pol| find_and_emit(&demod(&self.durs, te, pol)))
    }
}

impl Default for Hitag2RkeDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ProtocolDecoder for Hitag2RkeDecoder {
    fn name(&self) -> &'static str {
        NAME
    }

    fn timing(&self) -> ProtocolTiming {
        ProtocolTiming { te_short: 240, te_long: 480, te_delta: 120, min_count_bit: MIN_BITS }
    }

    fn supported_frequencies(&self) -> &[u32] {
        super::ANY_FREQUENCY
    }

    fn reset(&mut self) {
        self.durs.clear();
    }

    fn feed(&mut self, level: bool, duration: u32) -> Option<DecodedSignal> {
        if duration >= GAP {
            let out = self.process_burst();
            self.durs.clear();
            return out;
        }
        if duration < MIN_PULSE {
            return None;
        }
        let mut out = None;
        if self.durs.len() >= MAXDUR {
            out = self.process_burst();
            self.durs.clear();
        }
        let v = duration as i32;
        self.durs.push(if level { v } else { -v });
        out
    }

    fn supports_encoding(&self) -> bool {
        false
    }

    fn encode(&self, _decoded: &DecodedSignal, _button: u8) -> Option<Vec<LevelDuration>> {
        None
    }

    /// Rebuilds the frame from `Key` (UID, button, counter, top 18 keystream bits) and `Ks` (the
    /// keystream), recomputing the check byte; the same plausibility rules as the decoder apply.
    fn from_flipper_key(&self, key: &FlipperKey) -> Option<DecodedSignal> {
        if key.bits != DATA_BITS {
            return None;
        }
        let ks = key.u32_field("Ks")?;
        let uid = (key.key >> 32) as u32;
        let btn = ((key.key >> 28) & 0xF) as u8;
        let lctr = ((key.key >> 18) & 0x3FF) as u16;
        if u64::from(ks >> 14) != key.key & 0x3FFFF || uid == 0 || uid == 0xFFFF_FFFF {
            return None;
        }
        if (lctr == 0 && ks == 0) || (lctr == 0x3FF && ks == 0xFFFF_FFFF) {
            return None;
        }
        // The XOR of the ten bytes from the marker's last bit: 1 | UID | btn | lctr | ks | 0.
        let chk = frame_check(uid, btn, lctr, ks);
        Some(signal(uid, btn, lctr, ks, chk))
    }

    /// The decoder's `serialize`: the generic block plus `Ks`.
    fn to_flipper_key(&self, d: &DecodedSignal) -> Option<FlipperKey> {
        let mut key = FlipperKey::for_protocol(NAME, d.data_count_bit, d.data);
        key.push_u32("Ks", d.extra? as u32);
        Some(key)
    }
}

/// The check byte of a frame: XOR of the ten bytes starting at the marker's last bit (a 1), then
/// the UID, button, counter and keystream, and the closing 0 bit.
fn frame_check(uid: u32, btn: u8, lctr: u16, ks: u32) -> u8 {
    let bits = (1u128 << 79) | u128::from(uid) << 47 | u128::from(btn & 0xF) << 43 | u128::from(lctr & 0x3FF) << 33 | u128::from(ks) << 1;
    (0..10).fold(0u8, |x, i| x ^ (bits >> (72 - 8 * i)) as u8)
}

#[cfg(test)]
mod tests {
    use super::super::g2_common::test_util::*;
    use super::*;

    /// One frame the FoxFW C decoder (host oracle) reports for a synthetic burst: made-up values, the
    /// pulses are signed durations (HIGH positive), the fields are what the oracle printed.
    struct Case {
        name: &'static str,
        wire: &'static [i32],
        frames: &'static [Expect],
    }
    struct Expect {
        data: u64,
        bits: usize,
        serial: Option<u32>,
        button: Option<u8>,
        counter: Option<u16>,
        extra: Option<u64>,
    }

    const ACCEPT: &[Case] = &[
        Case {
            name: "ok_te240",
            wire: &[
            -5000, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 480,
            -240, 240, -480, 480, -480, 240, -240, 240, -240, 240, -240, 480, -240, 240, -480, 480,
            -240, 240, -480, 240, -240, 480, -480, 480, -240, 240, -480, 240, -240, 240, -240, 240,
            -240, 480, -240, 240, -240, 240, -240, 240, -480, 480, -480, 480, -480, 240, -240, 240,
            -240, 480, -480, 480, -240, 240, -480, 480, -480, 480, -480, 240, -240, 480, -240, 240,
            -240, 240, -480, 240, -240, 240, -240, 480, -480, 240, -240, 480, -240, 240, -480, 480,
            -480, 480, -480, 480, -240, 240, -240, 240, -240, 240, -480, 240, -240, 480, -240, 240,
            -480, 480, -240, 240, -240, 240, -240, 240, -480, 480, -240, 240, -240, 240, -240, 240,
            -240, 240, -480, 480, -240, 240, -240, 240, -480, 240, -240, 240, -240, 240, -240, 480,
            -480, 240, -240, 240, -240, 240, -240, 240, -5000,
            ],
            frames: &[
                Expect { data: 0xA1B2C3D45A9C4D5E, bits: 103, serial: Some(2712847316), button: Some(5), counter: Some(679), extra: Some(0xE113579BDF) },
            ],
        },
        Case {
            name: "ok_te240_inv",
            wire: &[
            -5000, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240,
            240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240,
            240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240,
            240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240,
            240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -480,
            240, -240, 480, -480, 480, -240, 240, -240, 240, -240, 240, -480, 240, -240, 480, -480,
            240, -240, 480, -240, 240, -480, 480, -480, 240, -240, 480, -240, 240, -240, 240, -240,
            240, -480, 240, -240, 240, -240, 240, -240, 480, -480, 480, -480, 480, -240, 240, -240,
            240, -480, 480, -480, 240, -240, 480, -480, 480, -480, 480, -240, 240, -480, 240, -240,
            240, -240, 480, -240, 240, -240, 240, -480, 480, -240, 240, -480, 240, -240, 480, -480,
            480, -480, 480, -480, 240, -240, 240, -240, 240, -240, 480, -240, 240, -480, 240, -240,
            480, -480, 240, -240, 240, -240, 240, -240, 480, -480, 240, -240, 240, -240, 240, -240,
            240, -240, 480, -480, 240, -240, 240, -240, 480, -240, 240, -240, 240, -240, 240, -480,
            480, -240, 240, -240, 240, -240, 240, -240, -5000,
            ],
            frames: &[
                Expect { data: 0xA1B2C3D45A9C4D5E, bits: 103, serial: Some(2712847316), button: Some(5), counter: Some(679), extra: Some(0xE113579BDF) },
            ],
        },
        Case {
            name: "ok_te90",
            wire: &[
            -5000, 90, -90, 90, -90, 90, -90, 90, -90, 90, -90, 90, -90, 90, -90, 90,
            -90, 90, -90, 90, -90, 90, -90, 90, -90, 90, -90, 90, -90, 90, -90, 90,
            -90, 90, -90, 90, -90, 90, -90, 90, -90, 90, -90, 90, -90, 90, -90, 90,
            -90, 90, -90, 90, -90, 90, -90, 90, -90, 90, -90, 90, -90, 90, -90, 90,
            -90, 90, -90, 90, -90, 90, -90, 90, -90, 90, -90, 90, -90, 90, -90, 180,
            -180, 90, -90, 90, -90, 90, -90, 180, -180, 180, -90, 90, -90, 90, -180, 180,
            -180, 180, -90, 90, -180, 180, -90, 90, -90, 90, -90, 90, -90, 90, -180, 90,
            -90, 90, -90, 90, -90, 90, -90, 90, -90, 90, -90, 90, -90, 180, -90, 90,
            -180, 180, -180, 90, -90, 180, -90, 90, -180, 90, -90, 90, -90, 90, -90, 90,
            -90, 90, -90, 90, -90, 90, -90, 90, -90, 180, -90, 90, -90, 90, -180, 90,
            -90, 180, -180, 180, -180, 180, -90, 90, -90, 90, -90, 90, -90, 90, -90, 90,
            -90, 90, -180, 90, -90, 90, -90, 90, -90, 180, -180, 90, -90, 180, -180, 90,
            -90, 90, -90, 180, -90, 90, -180, 180, -180, 90, -90, 90, -90, 90, -90, 90,
            -90, 90, -90, 180, -180, 90, -90, 180, -90, 90, -180, 90, -90, 90, -90, 90,
            -90, 90, -5000,
            ],
            frames: &[
                Expect { data: 0xBADF00D30072BF8, bits: 103, serial: Some(195948557), button: Some(3), counter: Some(1), extra: Some(0x13CAFE1234) },
            ],
        },
        Case {
            name: "ok_te160",
            wire: &[
            -5000, 160, -160, 160, -160, 160, -160, 160, -160, 160, -160, 160, -160, 160, -160, 160,
            -160, 160, -160, 160, -160, 160, -160, 160, -160, 160, -160, 160, -160, 160, -160, 160,
            -160, 160, -160, 160, -160, 160, -160, 160, -160, 160, -160, 160, -160, 160, -160, 160,
            -160, 160, -160, 160, -160, 160, -160, 160, -160, 160, -160, 160, -160, 160, -160, 160,
            -160, 160, -160, 160, -160, 160, -160, 160, -160, 160, -160, 160, -160, 160, -160, 320,
            -320, 320, -320, 320, -320, 320, -320, 320, -320, 320, -160, 160, -320, 160, -160, 320,
            -160, 160, -320, 160, -160, 320, -160, 160, -160, 160, -320, 320, -160, 160, -160, 160,
            -160, 160, -320, 160, -160, 160, -160, 320, -320, 160, -160, 160, -160, 320, -320, 160,
            -160, 320, -160, 160, -160, 160, -160, 160, -160, 160, -160, 160, -160, 160, -160, 160,
            -160, 160, -160, 160, -320, 160, -160, 160, -160, 160, -160, 160, -160, 320, -160, 160,
            -160, 160, -160, 160, -320, 160, -160, 160, -160, 160, -160, 320, -160, 160, -160, 160,
            -160, 160, -320, 160, -160, 160, -160, 160, -160, 320, -160, 160, -160, 160, -160, 160,
            -320, 160, -160, 160, -160, 160, -160, 320, -160, 160, -160, 160, -160, 160, -320, 160,
            -160, 320, -320, 320, -320, 320, -320, 320, -320, 160, -160, 160, -160, 160, -160, 160,
            -5000,
            ],
            frames: &[
                Expect { data: 0x556677889FF83C3C, bits: 103, serial: Some(1432778632), button: Some(9), counter: Some(1022), extra: Some(0x550F0F0F0F) },
            ],
        },
    ];

    /// Bursts the C decoder reports nothing for: a wrong check byte, a marker other than 0x0001, a 1 in the bit before the check, a zero UID, a zero counter with a zero keystream, and a burst of 30 pulses.
    const REJECT: &[(&str, &[i32])] = &[
        (
            "bad_chk",
            &[
            -5000, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 480,
            -240, 240, -480, 480, -480, 240, -240, 240, -240, 240, -240, 480, -240, 240, -480, 480,
            -240, 240, -480, 240, -240, 480, -480, 480, -240, 240, -480, 240, -240, 240, -240, 240,
            -240, 480, -240, 240, -240, 240, -240, 240, -480, 480, -480, 480, -480, 240, -240, 240,
            -240, 480, -480, 480, -240, 240, -480, 480, -480, 480, -480, 240, -240, 480, -240, 240,
            -240, 240, -480, 240, -240, 240, -240, 480, -480, 240, -240, 480, -240, 240, -480, 480,
            -480, 480, -480, 480, -240, 240, -240, 240, -240, 240, -480, 240, -240, 480, -240, 240,
            -480, 480, -240, 240, -240, 240, -240, 240, -480, 480, -240, 240, -240, 240, -240, 240,
            -240, 240, -480, 480, -240, 240, -240, 240, -480, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -5000,
            ],
        ),
        (
            "bad_marker",
            &[
            -5000, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 480, -240, 240,
            -240, 240, -480, 480, -480, 240, -240, 240, -240, 240, -240, 480, -240, 240, -480, 480,
            -240, 240, -480, 240, -240, 480, -480, 480, -240, 240, -480, 240, -240, 240, -240, 240,
            -240, 480, -240, 240, -240, 240, -240, 240, -480, 480, -480, 480, -480, 240, -240, 240,
            -240, 480, -480, 480, -240, 240, -480, 480, -480, 480, -480, 240, -240, 480, -240, 240,
            -240, 240, -480, 240, -240, 240, -240, 480, -480, 240, -240, 480, -240, 240, -480, 480,
            -480, 480, -480, 480, -240, 240, -240, 240, -240, 240, -480, 240, -240, 480, -240, 240,
            -480, 480, -240, 240, -240, 240, -240, 240, -480, 480, -240, 240, -240, 240, -240, 240,
            -240, 240, -480, 480, -240, 240, -240, 240, -480, 240, -240, 240, -240, 240, -240, 480,
            -480, 240, -240, 240, -240, 240, -240, 240, -5000,
            ],
        ),
        (
            "bad_zero_bit",
            &[
            -5000, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 480,
            -240, 240, -480, 480, -480, 240, -240, 240, -240, 240, -240, 480, -240, 240, -480, 480,
            -240, 240, -480, 240, -240, 480, -480, 480, -240, 240, -480, 240, -240, 240, -240, 240,
            -240, 480, -240, 240, -240, 240, -240, 240, -480, 480, -480, 480, -480, 240, -240, 240,
            -240, 480, -480, 480, -240, 240, -480, 480, -480, 480, -480, 240, -240, 480, -240, 240,
            -240, 240, -480, 240, -240, 240, -240, 480, -480, 240, -240, 480, -240, 240, -480, 480,
            -480, 480, -480, 480, -240, 240, -240, 240, -240, 240, -480, 240, -240, 480, -240, 240,
            -480, 480, -240, 240, -240, 240, -240, 240, -480, 480, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -480, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -5000,
            ],
        ),
        (
            "zero_uid",
            &[
            -5000, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 480,
            -480, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 480, -480, 480, -240, 240, -480, 480, -480, 480, -480, 240, -240, 480, -240, 240,
            -240, 240, -480, 240, -240, 240, -240, 480, -480, 240, -240, 480, -240, 240, -480, 480,
            -480, 480, -480, 480, -240, 240, -240, 240, -240, 240, -480, 240, -240, 480, -240, 240,
            -480, 480, -240, 240, -240, 240, -240, 240, -480, 480, -240, 240, -240, 240, -240, 240,
            -240, 240, -480, 480, -240, 240, -240, 240, -480, 240, -240, 240, -240, 480, -240, 240,
            -480, 240, -240, 240, -240, 240, -240, 240, -5000,
            ],
        ),
        (
            "zero_cnt_ks",
            &[
            -5000, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 480,
            -240, 240, -480, 480, -480, 240, -240, 240, -240, 240, -240, 480, -240, 240, -480, 480,
            -240, 240, -480, 240, -240, 480, -480, 480, -240, 240, -480, 240, -240, 240, -240, 240,
            -240, 480, -240, 240, -240, 240, -240, 240, -480, 480, -480, 480, -480, 240, -240, 240,
            -240, 480, -480, 480, -480, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 480, -480, 480, -480, 480, -480, 480,
            -480, 240, -240, 240, -240, 240, -240, 240, -240, 240, -5000,
            ],
        ),
        (
            "short_burst",
            &[
            -5000, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240,
            -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 240, -240, 480,
            -240, 240, -480, 480, -480, 240, -240, 240, -240, 240, -240, 480, -240, 240, -480, 480,
            -240, 240, -480, 240, -240, 240, -5000,
            ],
        ),
    ];

    fn check(frames: &[DecodedSignal], want: &[Expect], name: &str) {
        assert_eq!(frames.len(), want.len(), "{name}: frame count");
        for (d, w) in frames.iter().zip(want) {
            assert_eq!((d.data, d.data_count_bit), (w.data, w.bits), "{name}");
            assert_eq!((d.serial, d.button, d.counter, d.extra), (w.serial, w.button, w.counter, w.extra), "{name}");
            assert!(!d.encoder_capable, "{name}");
        }
    }

    #[test]
    fn decodes_what_the_c_decoder_decodes() {
        for c in ACCEPT {
            let got = feed_with_resets(&mut Hitag2RkeDecoder::new(), &wire(c.wire));
            check(&got, c.frames, c.name);
        }
    }

    #[test]
    fn rejects_what_the_c_decoder_rejects() {
        for (name, w) in REJECT {
            let got = feed_with_resets(&mut Hitag2RkeDecoder::new(), &wire(w));
            assert!(got.is_empty(), "{name}: {got:?}");
        }
    }

    /// The whole registry reports the frame under this name (no earlier decoder takes it).
    #[test]
    fn registry_reports_it() {
        use super::super::ProtocolRegistry;
        for c in ACCEPT {
            let pairs = wire(c.wire);
            let mut reg = ProtocolRegistry::new();
            let got = reg.process_signal_stream(&pairs, 433920000);
            let names: Vec<_> = got.iter().map(|(n, _, _)| n.as_str()).collect();
            assert_eq!(names, vec![NAME; c.frames.len()], "{}", c.name);
        }
    }

    /// Key files: every accepted frame survives write, parse and load unchanged.
    #[test]
    fn key_files_round_trip() {
        let dec = Hitag2RkeDecoder::new();
        for c in ACCEPT {
            for d in feed_with_resets(&mut Hitag2RkeDecoder::new(), &wire(c.wire)) {
                assert_key_roundtrip(&dec, &d);
                let text = dec.to_flipper_key(&d).unwrap().to_text();
                assert!(text.contains("Protocol: Hitag2\nBit: 103\n"), "{text}");
            }
        }
    }

    /// No encoder, as FoxFW (see the module doc): a loaded key is not encoder-capable.
    #[test]
    fn receive_only() {
        let dec = Hitag2RkeDecoder::new();
        assert!(!dec.supports_encoding());
        assert!(dec.encode(&decoded(0, 0), 1).is_none());
        assert_eq!(dec.name(), "Hitag2");
        for d in feed_with_resets(&mut Hitag2RkeDecoder::new(), &wire(ACCEPT[0].wire)) {
            let loaded = dec.from_flipper_key(&dec.to_flipper_key(&d).unwrap()).unwrap();
            assert!(!loaded.encoder_capable && dec.encode(&loaded, loaded.button.unwrap()).is_none());
        }
    }

    /// Oracle comparison: `KAT_HITAG2_ENCODE=<file listing key files>` prints KAT's transmission for
    /// each (`KATENC <file> | (none)`: KAT has no encoder, as `subghz_oracle_foxfwenc encode`
    /// answers "no encoder" for every Hitag2 key file).
    #[test]
    #[ignore = "oracle harness"]
    fn encode_key_files() {
        let Ok(list) = std::env::var("KAT_HITAG2_ENCODE") else { return };
        let dec = Hitag2RkeDecoder::new();
        for file in std::fs::read_to_string(list).unwrap().lines().filter(|l| !l.is_empty()) {
            let key = FlipperKey::parse(&std::fs::read_to_string(file).unwrap()).unwrap();
            let out = dec.from_flipper_key(&key).and_then(|d| dec.encode(&d, d.button.unwrap_or(0)));
            assert!(out.is_none(), "{file}");
            println!("KATENC {file} | (none)");
        }
    }
}
