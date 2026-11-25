//! BMW CAS4 remote keyless-entry authenticator (AES-128 MAC).
//!
//! KAT's model of how a BMW **CAS4** body module (a Freescale S12XE) computes and checks the
//! rolling code of its remotes, read from one CAS4 firmware image (a `5M48H` module dump a
//! collaborator supplied for authorised research; see
//! `REFERENCES/tools/scripts/bmw-cas-porsche-2026-11-24/`). It is the same shape as the Audi BCM2
//! MAC ([`super::audi_v3`]): a truncated AES-128 of a fixed layout under a key that is secret to the
//! individual vehicle.
//!
//! **Which on-air frame this is — not confirmed, and NOT the CAS3 `0xCC` frame.** It is a
//! *different generation* from KAT's "BMW/Mini V3" ([`super::bmw_v3`]): that `0xCC` telegram is the
//! **CAS3** remote, and its rolling code is **HITAG2**, proven against the CAS3's own firmware
//! ([`super::hitag2::bmw_cas3_authenticator`]) — not AES. (FoxFW's `bmw_v3.c` mislabels it "RC=AES";
//! that is wrong for CAS3.) This CAS4 AES MAC belongs to the later F-series. The 433.92 MHz 128-bit
//! "BMW" frame KAT reads as [`super::bmw`] (the `0x599E` format, a 47-bit encrypted hop) is the
//! likely F-series candidate, but the map from any on-air frame to the telegram bytes `t[..]` the
//! module checks is **not yet confirmed** — it needs recordings of a dumped car's own keys, the same
//! gap as Audi BCM2.
//!
//! ## What the firmware does (CAS4, routine `D2:A911`)
//!
//! Each learned remote has its **own 128-bit key**, derived at learn time from a single 128-bit
//! **master secret** held in the module (BMW's per-vehicle EWS/CAS key, one per car in BMW's
//! database) and that remote's stored record ([`derive_key`]). A received telegram `t` carries a
//! key number; the module rebuilds a 16-byte block from `t`, encrypts it with that remote's key,
//! and checks that the first **5 bytes** (40 bits) equal the telegram's MAC field
//! ([`authenticator`]). It then accepts the 24-bit counter if it is ahead of the stored one within
//! a forward window.
//!
//! ## Security framing
//!
//! This is **not** an RF break: the MAC is a full AES-128 and the per-remote keys descend from a
//! per-vehicle master, so a telegram cannot be forged without that car's secret. KAT ships **no**
//! BMW keys. Everything here is inert until the owner loads their own car's master, read from their
//! own module, into a `bmw_cas4_keys` file ([`super::keys::BmwCas4Car`]); and it is not yet wired to
//! any on-air decoder (the frame map above is unconfirmed). The round-trip is proven internally
//! (derive -> MAC -> verify, and the FIPS-197 AES vector); it is not yet proven against a real
//! captured frame. Published: the construction (this file, the report under `reports/`). Withheld:
//! any real key or VIN.

/// Fixed bytes 0..7 of the MAC block (`STY` immediates at `D2:A94B`..: `91 7F CC 61 92 61 3B`).
const BLOCK_PREFIX: [u8; 7] = [0x91, 0x7F, 0xCC, 0x61, 0x92, 0x61, 0x3B];

/// A received remote telegram as the module checks it: the bytes the firmware indexes as `t[..]`.
/// At least 18 bytes. `mac` is `t[2..7]`, `key_num` is `t[11]`. The over-the-air frame maps onto
/// this (unconfirmed, see the module docs).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Telegram {
    /// The 18 bytes the MAC and fields are read from (`t[0]`..`t[17]`).
    pub t: [u8; 18],
}

/// Fields of a telegram once a key verifies it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Fields {
    /// Which learned key signed it (`t[11]`, 0..=7 for a normal remote).
    pub key_num: u8,
    /// Command / button nibble (high nibble of `t[10]`; 0x1 = the usual lock/unlock group).
    pub button: u8,
    /// 24-bit press counter (`t[8]`, `t[9]`, `t[16] + t[17]`), big-endian.
    pub counter: u32,
}

impl Telegram {
    /// The 16-byte block the firmware builds before the AES (routine `D2:A911`): seven constant
    /// bytes, the button byte with its low nibble forced to `F`, then telegram bytes in the
    /// firmware's order.
    fn mac_block(&self) -> [u8; 16] {
        let t = &self.t;
        let cnt_lo = t[16].wrapping_add(t[17]);
        [
            BLOCK_PREFIX[0], BLOCK_PREFIX[1], BLOCK_PREFIX[2], BLOCK_PREFIX[3],
            BLOCK_PREFIX[4], BLOCK_PREFIX[5], BLOCK_PREFIX[6],
            t[10] | 0x0F,
            t[12], t[13], t[14], t[15],
            t[7], t[8], t[9], cnt_lo,
        ]
    }

    /// The telegram's own MAC field, `t[2..7]` (40 bits).
    fn mac_field(&self) -> [u8; 5] {
        self.t[2..7].try_into().unwrap()
    }

    /// The key number the telegram names (`t[11]`).
    pub fn key_num(&self) -> u8 {
        self.t[11]
    }

    /// The fields, as read from the bytes (no key needed; the counter is plaintext here).
    fn fields(&self) -> Fields {
        Fields {
            key_num: self.t[11],
            button: self.t[10] >> 4,
            counter: (u32::from(self.t[8]) << 16)
                | (u32::from(self.t[9]) << 8)
                | u32::from(self.t[16].wrapping_add(self.t[17])),
        }
    }
}

/// The 40-bit MAC the module expects for telegram `t` under remote key `key`: AES-128 of the
/// block, first five bytes.
pub(crate) fn authenticator(key: &[u8; 16], t: &Telegram) -> [u8; 5] {
    aes::encrypt(key, &t.mac_block())[..5].try_into().unwrap()
}

/// Derive one remote's 128-bit key from the car's master secret and the remote's stored record,
/// as the module does at learn time (`D2:AF11` -> the AES-128 engine `DB:A6C7`): AES-128 of a
/// 6-byte message taken from the record, padded `80 00…`, under the master key.
///
/// The record-byte selection is read from the firmware and is **not yet confirmed on the air**; an
/// owner who has already read the per-remote keys from their module can store those directly
/// instead ([`super::keys::BmwKey`]).
pub(crate) fn derive_key(master: &[u8; 16], record6: &[u8; 6]) -> [u8; 16] {
    let mut m = [0u8; 16];
    m[..6].copy_from_slice(record6);
    m[6] = 0x80;
    aes::encrypt(master, &m)
}

/// The remote key that verifies `t`'s MAC, with the telegram's fields, from the loaded car keys.
/// The MAC is one-way, so a matching key is how KAT ties a remote to a capture. None if no loaded
/// key both has the telegram's key number and reproduces its MAC.
pub(crate) fn verify<'a>(
    t: &Telegram,
    cars: &'a [super::keys::BmwCas4Car],
) -> Option<(&'a super::keys::BmwCas4Car, &'a super::keys::BmwKey, Fields)> {
    let want = t.mac_field();
    let num = t.key_num();
    for car in cars {
        if let Some(k) = car.keys.iter().find(|k| k.num == num) {
            if authenticator(&k.key, t) == want {
                return Some((car, k, t.fields()));
            }
        }
    }
    None
}

/// Build the next press of the remote that signed `prev`, for the same key and button, with the
/// counter raised by one: rebuild the counter bytes, recompute the MAC with the remote's key, and
/// write it into `t[2..7]`. None unless a loaded car key reproduces `prev`'s MAC.
pub(crate) fn encode_next(prev: &Telegram, cars: &[super::keys::BmwCas4Car]) -> Option<Telegram> {
    let (_car, key, f) = verify(prev, cars)?;
    let next_counter = f.counter.wrapping_add(1) & 0x00FF_FFFF;
    let mut t = prev.t;
    t[8] = (next_counter >> 16) as u8;
    t[9] = (next_counter >> 8) as u8;
    // Keep t[17] (a per-frame nonce), set t[16] so (t[16] + t[17]) is the new low byte.
    t[16] = (next_counter as u8).wrapping_sub(t[17]);
    let mut out = Telegram { t };
    let mac = authenticator(&key.key, &out);
    out.t[2..7].copy_from_slice(&mac);
    Some(out)
}

/// AES-128 encryption (FIPS-197), encrypt only — a MAC needs no decrypt. A self-contained copy, as
/// in [`super::audi_v3`], shared with no other protocol. Proven by the FIPS-197 C.1 vector in the
/// tests.
mod aes {
    const SBOX: [u8; 256] = [
        0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76, 0xca, 0x82,
        0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0, 0xb7, 0xfd, 0x93, 0x26,
        0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15, 0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96,
        0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75, 0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0,
        0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84, 0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb,
        0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf, 0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f,
        0x50, 0x3c, 0x9f, 0xa8, 0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff,
        0xf3, 0xd2, 0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
        0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb, 0xe0, 0x32,
        0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79, 0xe7, 0xc8, 0x37, 0x6d,
        0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08, 0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6,
        0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a, 0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e,
        0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e, 0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e,
        0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf, 0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f,
        0xb0, 0x54, 0xbb, 0x16,
    ];
    const RCON: [u8; 10] = [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36];

    fn xtime(x: u8) -> u8 {
        (x << 1) ^ ((x >> 7) * 0x1b)
    }

    /// One 16-byte block under a 16-byte key, column-major state (FIPS-197).
    pub(super) fn encrypt(key: &[u8; 16], block: &[u8; 16]) -> [u8; 16] {
        let mut rk = [0u8; 176];
        rk[..16].copy_from_slice(key);
        for i in 4..44 {
            let mut t = [rk[(i - 1) * 4], rk[(i - 1) * 4 + 1], rk[(i - 1) * 4 + 2], rk[(i - 1) * 4 + 3]];
            if i % 4 == 0 {
                t = [SBOX[t[1] as usize] ^ RCON[i / 4 - 1], SBOX[t[2] as usize], SBOX[t[3] as usize], SBOX[t[0] as usize]];
            }
            for j in 0..4 {
                rk[i * 4 + j] = rk[(i - 4) * 4 + j] ^ t[j];
            }
        }
        let mut s = *block;
        for (b, k) in s.iter_mut().zip(&rk[..16]) {
            *b ^= *k;
        }
        for round in 1..=10 {
            for b in s.iter_mut() {
                *b = SBOX[*b as usize];
            }
            s = [
                s[0], s[5], s[10], s[15], s[4], s[9], s[14], s[3], s[8], s[13], s[2], s[7], s[12], s[1], s[6], s[11],
            ];
            if round != 10 {
                for c in 0..4 {
                    let col = [s[c * 4], s[c * 4 + 1], s[c * 4 + 2], s[c * 4 + 3]];
                    for r in 0..4 {
                        s[c * 4 + r] = xtime(col[r]) ^ (xtime(col[(r + 1) % 4]) ^ col[(r + 1) % 4]) ^ col[(r + 2) % 4] ^ col[(r + 3) % 4];
                    }
                }
            }
            for (b, k) in s.iter_mut().zip(&rk[round * 16..round * 16 + 16]) {
                *b ^= *k;
            }
        }
        s
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::keys::{BmwCas4Car, BmwKey};

    /// FIPS-197 Appendix C.1 known-answer vector proves the AES core.
    #[test]
    fn aes_matches_fips197() {
        let key: [u8; 16] = (0..16).collect::<Vec<u8>>().try_into().unwrap();
        let pt = [
            0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
        ];
        let ct = [
            0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30, 0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5, 0x5a,
        ];
        assert_eq!(aes::encrypt(&key, &pt), ct);
    }

    /// A made-up car (never a real key): a telegram signed with its key verifies, and the next
    /// press verifies too with the counter one higher. The keys are invented for the test; KAT
    /// ships none.
    fn car_with(num: u8, key: [u8; 16]) -> BmwCas4Car {
        BmwCas4Car { name: "test".into(), master: None, keys: vec![BmwKey { num, key }] }
    }

    fn signed(key: &[u8; 16], num: u8, button: u8, counter: u32, nonce: u8) -> Telegram {
        let mut t = [0u8; 18];
        t[7] = 0xA5;
        t[10] = (button << 4) | 0x03;
        t[11] = num;
        t[12] = 0xDE;
        t[13] = 0xAD;
        t[14] = 0xBE;
        t[15] = 0xEF;
        t[8] = (counter >> 16) as u8;
        t[9] = (counter >> 8) as u8;
        t[17] = nonce;
        t[16] = (counter as u8).wrapping_sub(nonce);
        let mut tel = Telegram { t };
        let mac = authenticator(key, &tel);
        tel.t[2..7].copy_from_slice(&mac);
        tel
    }

    #[test]
    fn a_signed_telegram_verifies_and_the_next_press_advances_the_counter() {
        let key = [
            0x4B, 0x41, 0x54, 0x20, 0x62, 0x6D, 0x77, 0x20, 0x74, 0x65, 0x73, 0x74, 0x20, 0x6B, 0x65, 0x79,
        ];
        let cars = vec![car_with(2, key)];
        let frame = signed(&key, 2, 1, 0x0012_3456, 0x5A);

        let (_car, _k, f) = verify(&frame, &cars).expect("own key verifies");
        assert_eq!((f.key_num, f.button, f.counter), (2, 1, 0x0012_3456));

        // A different key number is not accepted even though the bytes are the same.
        let wrong = vec![car_with(3, key)];
        assert!(verify(&frame, &wrong).is_none());

        let next = encode_next(&frame, &cars).expect("own key makes the next press");
        let nf = verify(&next, &cars).expect("the next press verifies").2;
        assert_eq!(nf.counter, 0x0012_3457);
        assert_eq!((nf.key_num, nf.button), (2, 1));
        assert_ne!(next.mac_field(), frame.mac_field(), "the MAC changes with the counter");
    }

    #[test]
    fn a_tampered_mac_is_rejected() {
        let key = [0x11u8; 16];
        let cars = vec![car_with(0, key)];
        let mut frame = signed(&key, 0, 1, 10, 7);
        frame.t[4] ^= 0x01;
        assert!(verify(&frame, &cars).is_none());
    }

    /// Deriving a key from a master is deterministic and depends on the record, so an owner who
    /// supplies a master + records gets stable per-remote keys. (Self-consistency only; the
    /// record-byte choice is unconfirmed on the air.)
    #[test]
    fn derive_key_is_deterministic_and_record_dependent() {
        let master = [0x24u8; 16];
        let a = derive_key(&master, &[1, 2, 3, 4, 5, 6]);
        let b = derive_key(&master, &[1, 2, 3, 4, 5, 6]);
        let c = derive_key(&master, &[1, 2, 3, 4, 5, 7]);
        assert_eq!(a, b);
        assert_ne!(a, c);
    }
}
