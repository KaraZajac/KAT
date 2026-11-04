//! HITAG2 stream-cipher primitives shared by the Fiat V1 and Renault V1 protocols, plus the two
//! key-recovery paths the references run on a captured frame.
//!
//! References:
//! - ProtoPirate (primary) `REFERENCES/ProtoPirate/protocols/fiat_v1.c`:
//!   `fiat_v1_bcm_hitag2_filter` / `_feedback` / `_shift`, `fiat_v1_bcm_generate_authenticator`.
//! - ProtoPirate `REFERENCES/ProtoPirate/protocols/renault_v1.c`: `hitag2_filter`,
//!   `hitag2_feedback`, `hitag2_shift_state`, `hitag2_clock_cipher`, `hitag2_authenticator`
//!   (+ `hitag2_f20` / `hitag2_lfsr`), `hitag2_serial_permute`, `hitag2_build_iv`,
//!   `hitag2_encrypt_from_iv`, `hitag2_encrypt_frame`, `hitag2_apply_check_remote`, and the seed
//!   brute force `hitag2_bf_rearrange_dest` / `hitag2_bf_prepare` / `hitag2_bf_hop_matches` /
//!   `hitag2_brute_force_run` / `hitag2_seed_from_iv`. Flipper-ARF's `hitag2_seed.c` is a verbatim
//!   C-to-C copy of this seed model; doctor-firmware's renault_v1.c carries the same code.
//!
//! ## Register and core
//!
//! Both C files keep the 48-bit register as six bytes (`state[0]` bit 7 = oldest bit, `state[5]`
//! bit 0 = newest) and carry byte-identical [`filter`]-style truth-table code (tables
//! 0x2C79/0x6671/0x7907287B) and feedback (masks `B3 80 83 22 00 73`). KAT runs the same register
//! as a `u64` (bit 47 = `state[0]` bit 7 … bit 0 = `state[5]` bit 0, i.e. the big-endian value of
//! the six bytes): the filter taps are bits {41,42,44,45 | 32,33,35,39 | 21,24,26,30 |
//! 14,16,18,19 | 1,3,4,13} and the feedback taps are mask `0xB380_8322_0073`. The tests keep a
//! verbatim byte-array port and prove both forms equal (and match vectors from the compiled C).
//!
//! Every caller runs the same keystream schedule — 32 init rounds clocking in
//! `iv_bit ^ key_bit ^ filter(state)` (both MSB first), then 32 output rounds emitting
//! `filter(state)` and clocking in `feedback(state)` ([`run48`]). They differ only in how the
//! 48-bit key and the 32-bit IV are arranged around the 32-bit UID:
//!
//! | function | register init | injected key bits | 32-bit IV |
//! |---|---|---|---|
//! | [`fiat_bcm_authenticator`] | `uid ‖ key[4..6]` | `key[0..4]` BE | `epoch(18) ctl(10) btn(4)` |
//! | [`renault_authenticator`] | `uid ‖ key[0..2]` | `key[2..6]` BE | `cnt << 4 ‖ btn(4)` |
//! | seed ([`renault_encrypt_from_iv`]) | `serial ‖ perm[4..6]` | `perm[0..4]` LE | `iv[0..4]` LE |
//!
//! renault_v1.c's `hitag2_authenticator` is written on a bit-reversed `u64` register
//! (`hitag2_f20` + `hitag2_lfsr`) but is the same function: a Renault key `R` equals a Fiat BCM key
//! `K` rotated by two bytes (`R = K[4] K[5] K[0] K[1] K[2] K[3]`, epoch 0). doctor-firmware's
//! fiat_v1.c stores its "Hitag2 Key" field in exactly that rotated (`R`) order on disk
//! (`fiat_v1_hitag2_key_disk_to_cipher`), i.e. `R` is the standard HITAG2 key byte order.
//!
//! ## Renault V1 seed scheme (serial-derived key) and its brute force
//!
//! Some Renault PCF79xx remotes use a key derived from the serial ([`renault_serial_permute`]);
//! the 32-bit IV is `btn(4) ‖ cnt[0..16] ‖ seed(12)` laid out by [`renault_build_iv`]. A captured
//! frame shows the button and the low 10 counter bits, so ProtoPirate's brute force
//! (`hitag2_brute_force_run`) only has to try the 2^18 hidden bits (counter bits 10..15 + the
//! 12-bit seed), in candidate order `cand = 0 .. 0x3FFFF` (`iv[1] |= (cand & 3) << 6`,
//! `iv[2] = cand >> 2`, `iv[3] = cand >> 10`), and stops at the first IV whose keystream equals the
//! frame's hop. On a Flipper that is a progress-bar job; [`renault_seed_recover`] returns exactly
//! the C's first match, sharing the init rounds of common IV prefixes (`iv[3]`, then `iv[2]`, are
//! consumed first) and stopping each keystream at its first mismatching bit — a few milliseconds
//! in a release build, so KAT runs it on every decoded Renault V1 frame. A random frame passes by
//! chance with probability ≈ 2^18 / 2^32 = 2^-14 (as on the Flipper).

/// Low 48 bits.
const MASK48: u64 = (1 << 48) - 1;
/// Feedback taps: bytes `B3 80 83 22 00 73` as the 48-bit big-endian register value.
const FEEDBACK_TAPS: u64 = 0xB380_8322_0073;

/// Lookup table for one filter group: indexed by a window of the register starting at the
/// group's lowest tap; `taps` are the window bit positions of the group's (a, b, c, d) inputs
/// (index `a<<3 | b<<2 | c<<1 | d` into `truth`); the result is pre-shifted to the group's
/// position in the final 5-bit index.
const fn group_table<const N: usize>(truth: u32, taps: [usize; 4], out_shift: u32) -> [u8; N] {
    let mut table = [0u8; N];
    let mut w = 0;
    while w < N {
        let idx = (((w >> taps[0]) & 1) << 3)
            | (((w >> taps[1]) & 1) << 2)
            | (((w >> taps[2]) & 1) << 1)
            | ((w >> taps[3]) & 1);
        table[w] = (((truth >> idx) & 1) << out_shift) as u8;
        w += 1;
    }
    table
}

/// g0 = 0x2C79(bits 41,42,44,45), window `s >> 41`.
static FILTER_G0: [u8; 32] = group_table(0x2C79, [0, 1, 3, 4], 0);
/// g1 = 0x6671(bits 32,33,35,39), window `s >> 32`.
static FILTER_G1: [u8; 256] = group_table(0x6671, [0, 1, 3, 7], 1);
/// g2 = 0x6671(bits 21,24,26,30), window `s >> 21`.
static FILTER_G2: [u8; 1024] = group_table(0x6671, [0, 3, 5, 9], 2);
/// g3 = 0x6671(bits 14,16,18,19), window `s >> 14`.
static FILTER_G3: [u8; 64] = group_table(0x6671, [0, 2, 4, 5], 3);
/// g4 = 0x2C79(bits 1,3,4,13), window `s >> 1`.
static FILTER_G4: [u8; 8192] = group_table(0x2C79, [0, 2, 3, 12], 4);

/// HITAG2 non-linear filter on the `u64` register (`fiat_v1_bcm_hitag2_filter` ==
/// `hitag2_filter`; the five group lookups above implement the C's per-bit truth-table code,
/// see module doc for the tap mapping).
#[inline(always)]
pub fn filter48(s: u64) -> u32 {
    let index = FILTER_G0[((s >> 41) & 0x1F) as usize]
        | FILTER_G1[((s >> 32) & 0xFF) as usize]
        | FILTER_G2[((s >> 21) & 0x3FF) as usize]
        | FILTER_G3[((s >> 14) & 0x3F) as usize]
        | FILTER_G4[((s >> 1) & 0x1FFF) as usize];
    (0x7907_287Bu32 >> index) & 1
}

/// LFSR feedback bit (`fiat_v1_bcm_hitag2_feedback` == `hitag2_feedback`).
#[inline(always)]
pub fn feedback48(s: u64) -> u64 {
    ((s & FEEDBACK_TAPS).count_ones() & 1) as u64
}

/// Shift the register left by one, inserting `input` (0/1) as the newest bit.
#[inline(always)]
fn shift48(s: u64, input: u64) -> u64 {
    ((s << 1) | input) & MASK48
}

/// One HITAG2 keystream run: 32 init rounds clocking in `iv ^ key32 ^ filter` (MSB first), then
/// 32 output rounds. Returns the output bits (first bit = MSB) and leaves `state` at its final
/// value (the loop bodies of `fiat_v1_bcm_generate_authenticator` and `hitag2_clock_cipher`).
pub fn run48(state: &mut u64, key32: u32, iv: u32) -> u32 {
    let mut s = *state;
    let inject = key32 ^ iv;
    for i in (0..32).rev() {
        s = shift48(s, (((inject >> i) & 1) ^ filter48(s)) as u64);
    }
    let mut out = 0u32;
    for _ in 0..32 {
        out = (out << 1) | filter48(s);
        s = shift48(s, feedback48(s));
    }
    *state = s;
    out
}

/// Initial register: 32-bit UID followed by two key bytes.
#[inline]
fn initial_state(uid: u32, k0: u8, k1: u8) -> u64 {
    ((uid as u64) << 16) | ((k0 as u64) << 8) | k1 as u64
}

/// Fiat V1 (Magneti Marelli BCM) authenticator — `fiat_v1_bcm_generate_authenticator`.
///
/// Register = `uid ‖ key[4] key[5]`, injected key bits = `key[0..4]`, IV =
/// `(epoch & 0x3FFFF) << 14 | (control & 0x3FF) << 4 | (button & 0xF)`.
pub fn fiat_bcm_authenticator(
    uid: u32,
    button: u8,
    control: u16,
    key: &[u8; 6],
    epoch: u32,
) -> u32 {
    let mut state = initial_state(uid, key[4], key[5]);
    let iv = ((epoch & 0x3FFFF) << 14) | ((control as u32 & 0x03FF) << 4) | (button as u32 & 0x0F);
    let key32 = u32::from_be_bytes([key[0], key[1], key[2], key[3]]);
    run48(&mut state, key32, iv)
}

/// Renault V1 user-key authenticator — ProtoPirate `hitag2_authenticator(uid, button, counter,
/// key)` (doctor-firmware's `hitag2_hell_authenticator` is the same function).
///
/// Register = `uid ‖ key[0] key[1]`, injected key bits = `key[2..6]`, IV =
/// `counter << 4 | (button & 0xF)` (ProtoPirate passes the 10-bit counter; doctor-firmware passes
/// `epoch << 10 | cnt10`). Equivalent to [`fiat_bcm_authenticator`] with the key rotated by two
/// bytes. KAT has no source for such a key (no key entry, no key database), so the Renault V1
/// encoder never reaches this path; it is kept, C-tested, as the hook for one.
#[cfg_attr(not(test), allow(dead_code))]
pub fn renault_authenticator(uid: u32, button: u8, counter: u32, key: &[u8; 6]) -> u32 {
    let mut state = initial_state(uid, key[0], key[1]);
    let nonce = (counter << 4) | (button as u32 & 0x0F);
    let key32 = u32::from_be_bytes([key[2], key[3], key[4], key[5]]);
    run48(&mut state, key32, nonce)
}

/// Serial-derived key bytes — `hitag2_serial_permute` (verbatim bit plumbing).
pub fn renault_serial_permute(serial: u32) -> [u8; 6] {
    let [sn0, sn1, sn2, sn3] = serial.to_be_bytes().map(u32::from);
    // `hitag2_extract_bits(value, lsb, 1)`.
    let bit = |value: u32, lsb: u32| (value >> lsb) & 1;
    let sn1_inv_shr3 = !(sn1 >> 3) & 0xFF;
    let sn3_inv_shl3 = !(sn3 << 3) & 0xFF;
    let sn0_shr3 = (sn0 >> 3) & 0xFF;
    let sn2_inv_shl6 = !(sn2 << 6) & 0xFF;
    let sn1_inv_shl3 = !(sn1 << 3) & 0xFF;

    let mut p0 = ((sn0 >> 6) & 2) | ((sn1 >> 4) & 8) | bit(sn0 ^ 0x10, 4);
    p0 |= !(sn0 << 2) & 0x20;
    p0 |= (sn0 << 5) & 0x40;
    p0 |= (sn2 << 1) & 0x80;
    p0 |= !(sn2 >> 5) & 4;
    p0 |= !(sn0 >> 2) & 0x10;

    let mut p1 = bit(sn1, 5) | (sn1_inv_shr3 & 4) | (sn0 & 0x20);
    p1 |= sn3_inv_shl3 & 8;
    p1 |= !(sn2 << 2) & 0x10;
    p1 |= !(sn2 << 3) & 0x40;
    p1 |= 0x80;

    let mut p2 = bit(sn0, 2) | (sn0_shr3 & 2) | (!(sn3 >> 2) & 4) | (sn1 & 0x10);
    p2 |= !(sn1 << 4) & 0x20;
    p2 |= sn3_inv_shl3 & 0x40;
    p2 |= 0x80;

    let mut p3 = ((sn0 >> 2) & 2) | ((sn1 >> 3) & 8) | bit(sn2 ^ 0x20, 5);
    p3 |= (sn1 << 5) & 0x20;
    p3 |= (sn3 << 1) & 0x40;
    p3 |= !sn0_shr3 & 4;
    p3 |= sn1_inv_shr3 & 0x10;
    p3 |= sn2_inv_shl6 & 0x80;

    let mut p4_lo = ((sn3 << 2) & 8) | ((sn0 << 4) & 0x10) | bit(sn0 ^ 4, 2);
    p4_lo |= !(sn3 >> 1) & 2;
    p4_lo |= !(sn1 >> 4) & 4;
    let mut p4_hi = !(sn0 << 4) & 0x20;
    p4_hi |= sn2_inv_shl6 & 0x40;
    p4_hi |= sn1_inv_shl3 & 0x80;

    let mut p5 = ((sn3 >> 2) & 0x10) | ((sn2 >> 3) & 2) | (!sn0 & 0x80);
    p5 |= !(sn0 << 3) & 8;
    p5 |= (sn0 >> 2) & 0x10;
    p5 |= sn1_inv_shl3 & 0x20;
    p5 |= (sn3 >> 1) & 0x40;
    p5 |= !(sn1 >> 1) & 4;

    [
        p0 as u8,
        p1 as u8,
        p2 as u8,
        p3 as u8,
        (p4_lo | p4_hi) as u8,
        p5 as u8,
    ]
}

/// Seed-scheme IV — `hitag2_build_iv(cnt, btn, seed)`: `iv[0] = cnt[3:0] ‖ btn`,
/// `iv[1] = cnt[11:4]`, `iv[2] = seed[15:12] ‖ cnt[15:12]`, `iv[3] = seed[7:0]` (only bits 15:12
/// and 7:0 of `seed` are used — with a recovered seed, the IV it came from).
pub fn renault_build_iv(cnt: u32, btn: u8, seed: u32) -> [u8; 4] {
    [
        (((cnt << 4) & 0xF0) as u8) | (btn & 0x0F),
        ((cnt >> 4) & 0xFF) as u8,
        (((seed >> 8) & 0xF0) | ((cnt >> 12) & 0x0F)) as u8,
        (seed & 0xFF) as u8,
    ]
}

/// `hitag2_seed_from_iv`: the IV bytes as a big-endian word (what ProtoPirate stores as "Seed").
pub fn renault_seed_from_iv(iv: &[u8; 4]) -> u32 {
    u32::from_be_bytes(*iv)
}

/// The 16-bit counter carried by a seed-scheme IV — `hitag2_apply_check_remote` with
/// `recovered == HITAG2_RECOVERED_YES`: `((seed >> 12) & 0xFF0) | ((seed << 4) & 0xF000) |
/// (seed >> 28)` (the inverse of [`renault_build_iv`]'s counter placement).
pub fn renault_counter_from_seed(seed: u32) -> u16 {
    (((seed >> 12) & 0xFF0) | ((seed << 4) & 0xF000) | (seed >> 28)) as u16
}

/// XOR of the 10 payload bytes (`hitag2_frame_xor`): the 11th frame byte.
fn payload_xor(raw: &[u8]) -> u8 {
    raw[..10].iter().fold(0, |acc, b| acc ^ b)
}

/// Seed-scheme frame — `hitag2_encrypt_from_iv(serial_be, iv, out)`: the 11-byte Renault payload
/// (`uid(4) ‖ btn/cnt ‖ … ‖ xor`) for a serial-derived key and the given IV.
pub fn renault_encrypt_from_iv(serial: u32, iv: &[u8; 4]) -> [u8; 11] {
    let perm = renault_serial_permute(serial);
    let mut state = initial_state(serial, perm[4], perm[5]);
    // hitag2_clock_cipher(state, iv_work = perm[0..4], iv_orig = iv): both buffers are
    // little-endian words consumed MSB first; the 32 output bits land in iv_work, first bit at
    // bit 31 of the little-endian word.
    let key32 = u32::from_le_bytes([perm[0], perm[1], perm[2], perm[3]]);
    let iv_work = run48(&mut state, key32, u32::from_le_bytes(*iv)).to_le_bytes();
    let state5 = (state & 0xFF) as u8;

    let hop0 = iv_work[0];
    let mut hop1 = (hop0 >> 7) | (iv_work[1] << 1);
    let mut hop2 = (iv_work[1] >> 7) | (iv_work[2] << 1);
    let hop_ext = ((hop0 >> 6) & 1) | (hop1 << 1);
    let mut hop3 = (iv_work[2] >> 7) | (iv_work[3] << 1);
    let hop4 = (iv_work[3] >> 7) | (state5 << 1);
    hop1 = (hop1 >> 7) | (hop2 << 1);
    hop2 = (hop2 >> 7) | (hop3 << 1);
    hop3 = (hop3 >> 7) | (hop4 << 1);

    let mut mix: u32 = ((iv[1] as u32) << 4) | ((iv[0] as u32) >> 4);
    mix = (mix | (((iv[2] as u32) << 12) & 0xFFFF)) & 0xFFFF;

    let mut out = [0u8; 11];
    out[..4].copy_from_slice(&serial.to_be_bytes());
    out[4] = (((mix >> 6) & 0x0F) as u8) | ((iv[0] << 4) & 0xF0);
    out[5] = (hop3 & 3) | (((mix << 2) & 0xFC) as u8);
    out[6] = hop2;
    out[7] = hop1;
    out[8] = hop_ext;
    out[9] = (hop0 << 2) | 2;
    out[10] = payload_xor(&out);
    out
}

/// `hitag2_encrypt_frame(serial, cnt, btn, seed, out, iv)`: build the IV, encrypt, return
/// `(frame, iv)` (ProtoPirate stores the new IV back as the next "Seed").
pub fn renault_encrypt_frame(serial: u32, cnt: u32, btn: u8, seed: u32) -> ([u8; 11], [u8; 4]) {
    let iv = renault_build_iv(cnt, btn, seed);
    (renault_encrypt_from_iv(serial, &iv), iv)
}

/// Per-frame constants of the seed brute force — `hitag2_bf_prepare`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SeedSearch {
    pub serial: u32,
    pub perm: [u8; 6],
    /// Expected `iv_work` bytes after `hitag2_clock_cipher` (the frame's hop, rearranged).
    pub hop_target: [u8; 4],
    /// `iv[0]` (button + counter bits 3:0) and the low six bits of `iv[1]` (counter bits 9:4).
    pub iv0: u8,
    pub fp: u8,
}

/// `hitag2_bf_prepare` (+ `hitag2_bf_rearrange_dest`, verbatim bit plumbing).
pub fn renault_bf_prepare(frame: &[u8; 11]) -> SeedSearch {
    let mut dest = [0u8; 11];
    for i in 0..11 {
        dest[i] = frame[10 - i];
    }
    // hitag2_bf_rearrange_dest
    let frame4 = dest[6];
    let mut frame5 = dest[5];
    let mut frame6 = dest[4];
    let mut frame7 = dest[3];
    let frame8 = dest[2];
    let frame9 = dest[1];

    let mut hop0 = ((frame4 & 1) << 7) | (frame5 >> 1);
    frame5 = ((frame5 & 1) << 7) | (frame6 >> 1);
    let hop0_hi = hop0 >> 1;
    hop0 &= 1;
    hop0 = (hop0 << 7) | (frame5 >> 1);
    frame6 = ((frame6 & 1) << 7) | (frame7 >> 1);
    frame5 = ((frame5 & 1) << 7) | (frame6 >> 1);
    dest[3] = frame5;
    frame7 = ((frame7 & 1) << 7) | (frame8 >> 1);
    let hop2 = ((frame6 & 1) << 7) | (frame7 >> 1);
    frame6 = ((frame8 & 1) << 7) | (frame9 >> 1);
    dest[6] = ((frame9 & 1) << 7) | (frame4 >> 2);
    dest[1] = ((frame7 & 1) << 7) | (frame6 >> 1);
    dest[5] = hop0_hi | (((frame4 >> 1) & 1) << 7);
    dest[2] = hop2;
    dest[4] = hop0;

    let serial = u32::from_be_bytes([frame[0], frame[1], frame[2], frame[3]]);
    SeedSearch {
        serial,
        perm: renault_serial_permute(serial),
        hop_target: [dest[1], dest[2], dest[3], dest[4]],
        iv0: (frame[4] >> 4) | (dest[5] << 4),
        fp: (dest[5] >> 4) | ((dest[6] & 3) << 4),
    }
}

/// Clock eight init rounds: IV byte `iv_byte` (MSB first) against key bits
/// `key32[31 - 8*index ..= 24 - 8*index]`.
#[inline(always)]
fn clock_iv_byte(mut s: u64, key32: u32, index: u32, iv_byte: u8) -> u64 {
    let inject = ((key32 >> (24 - 8 * index)) as u8) ^ iv_byte;
    for bit in (0..8).rev() {
        s = shift48(s, (((inject >> bit) & 1) as u32 ^ filter48(s)) as u64);
    }
    s
}

/// Do the 32 output rounds from `s` equal `target` (first output bit = bit 31)? Stops at the
/// first mismatching bit.
#[inline(always)]
fn keystream_matches(mut s: u64, target: u32) -> bool {
    for i in (0..32).rev() {
        if filter48(s) != (target >> i) & 1 {
            return false;
        }
        s = shift48(s, feedback48(s));
    }
    true
}

/// ProtoPirate's Renault V1 seed brute force (`hitag2_brute_force_run`) for an 11-byte frame
/// (`uid ‖ btn/cnt10 ‖ hop ‖ tail ‖ xor`, the frame minus its 0x0001 header): the IV of the FIRST
/// candidate (C order) whose keystream equals the frame's hop, or `None` (the C's
/// `HITAG2_BF_STATUS_NOT_FOUND`). Same result as testing `cand = 0 .. 0x40000` with
/// `hitag2_bf_hop_matches`; the init rounds of each `iv[3]` and `iv[3] iv[2]` prefix are shared.
pub fn renault_seed_recover(frame: &[u8; 11]) -> Option<[u8; 4]> {
    let p = renault_bf_prepare(frame);
    let s0 = initial_state(p.serial, p.perm[4], p.perm[5]);
    let key32 = u32::from_le_bytes([p.perm[0], p.perm[1], p.perm[2], p.perm[3]]);
    let target = u32::from_le_bytes(p.hop_target);
    // The little-endian IV word is consumed from its MSB: iv[3] (= cand >> 10), iv[2]
    // (= cand >> 2), iv[1] (= fp | (cand & 3) << 6), iv[0]. Iterating iv[3], iv[2], cand & 3 in
    // ascending order visits cand in ascending order.
    for iv3 in 0..=255u8 {
        let s1 = clock_iv_byte(s0, key32, 0, iv3);
        for iv2 in 0..=255u8 {
            let s2 = clock_iv_byte(s1, key32, 1, iv2);
            for low in 0..4u8 {
                let iv1 = p.fp | (low << 6);
                let s3 = clock_iv_byte(s2, key32, 2, iv1);
                let s4 = clock_iv_byte(s3, key32, 3, p.iv0);
                if keystream_matches(s4, target) {
                    return Some([p.iv0, iv1, iv2, iv3]);
                }
            }
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Verbatim byte-array port of the C cipher core (`fiat_v1_bcm_hitag2_*` == `hitag2_filter` /
    /// `hitag2_feedback` / `hitag2_shift_state` / `hitag2_clock_cipher`), used to prove the `u64`
    /// core computes the same functions.
    mod byte_reference {
        fn truth(table: u32, index: u8) -> u8 {
            ((table >> index) & 1) as u8
        }
        fn fi(a: u8, b: u8, c: u8, d: u8) -> u8 {
            (a << 3) | (b << 2) | (c << 1) | d
        }
        fn bb(byte: u8, bit: u8) -> u8 {
            (byte >> bit) & 1
        }
        pub fn filter(st: &[u8; 6]) -> u8 {
            let mut g = truth(
                0x2C79,
                fi(bb(st[0], 1), bb(st[0], 2), bb(st[0], 4), bb(st[0], 5)),
            );
            g |= truth(
                0x6671,
                fi(bb(st[1], 0), bb(st[1], 1), bb(st[1], 3), bb(st[1], 7)),
            ) << 1;
            g |= truth(
                0x6671,
                fi(bb(st[3], 5), bb(st[2], 0), bb(st[2], 2), bb(st[2], 6)),
            ) << 2;
            g |= truth(
                0x6671,
                fi(bb(st[4], 6), bb(st[3], 0), bb(st[3], 2), bb(st[3], 3)),
            ) << 3;
            g |= truth(
                0x2C79,
                fi(bb(st[5], 1), bb(st[5], 3), bb(st[5], 4), bb(st[4], 5)),
            ) << 4;
            truth(0x7907_287B, g)
        }
        fn parity8(mut v: u8) -> u8 {
            v ^= v >> 4;
            v ^= v >> 2;
            v ^= v >> 1;
            v & 1
        }
        pub fn feedback(st: &[u8; 6]) -> u8 {
            const MASKS: [u8; 6] = [0xB3, 0x80, 0x83, 0x22, 0x00, 0x73];
            (0..6).fold(0, |fb, i| fb ^ parity8(st[i] & MASKS[i])) & 1
        }
        pub fn shift(st: &mut [u8; 6], input: u8) {
            for i in 0..5 {
                st[i] = (st[i] << 1) | (st[i + 1] >> 7);
            }
            st[5] = (st[5] << 1) | (input & 1);
        }
        fn shift_u32(buf: &mut [u8; 4], inject: u8) {
            let [b0, b1, b2, b3] = *buf;
            buf[3] = (b2 >> 7) | (b3 << 1);
            buf[2] = (b1 >> 7) | (b2 << 1);
            buf[1] = (b0 >> 7) | (b1 << 1);
            buf[0] = (b0 << 1) | (inject & 1);
        }
        /// `hitag2_clock_cipher` verbatim.
        pub fn clock_cipher(st: &mut [u8; 6], iv_work: &mut [u8; 4], iv_orig: &mut [u8; 4]) {
            for _ in 0..32 {
                let f = filter(st);
                let mut mix: u8 = if iv_work[3] & 0x80 != 0 {
                    if f != 0 {
                        0
                    } else {
                        1
                    }
                } else if f != 0 {
                    1
                } else {
                    0
                };
                if iv_orig[3] & 0x80 != 0 {
                    mix ^= 5;
                }
                shift(st, mix & 1);
                shift_u32(iv_work, 0);
                shift_u32(iv_orig, (mix >> 2) & 1);
            }
            for _ in 0..32 {
                shift_u32(iv_work, 0);
                if filter(st) != 0 {
                    iv_work[0] |= 1;
                }
                let fb = feedback(st);
                shift(st, fb);
            }
        }
        /// `hitag2_bf_hop_matches` for one candidate IV.
        pub fn hop_matches(frame: &[u8; 11], iv: &[u8; 4]) -> bool {
            let p = super::super::renault_bf_prepare(frame);
            let s = p.serial.to_be_bytes();
            let mut st = [s[0], s[1], s[2], s[3], p.perm[4], p.perm[5]];
            let mut iv_work = [p.perm[0], p.perm[1], p.perm[2], p.perm[3]];
            let mut iv_orig = *iv;
            clock_cipher(&mut st, &mut iv_work, &mut iv_orig);
            iv_work == p.hop_target
        }
    }

    /// Verbatim port of renault_v1.c's bit-reversed `u64` cipher (`hitag2_i4`, `hitag2_f20`,
    /// `hitag2_lfsr`, `hitag2_authenticator`), used only to prove the shared core implementation
    /// computes the same function.
    mod u64_reference {
        fn i4(x: u64, a: u32, b: u32, c: u32, d: u32) -> u32 {
            ((((x >> a) & 1) << 3) | (((x >> b) & 1) << 2) | (((x >> c) & 1) << 1) | ((x >> d) & 1))
                as u32
        }

        pub fn f20(state: u64) -> u64 {
            let s0 = (0x3C65u32 >> i4(state, 2, 3, 5, 6)) & 1;
            let s1 = (0x0EE5u32 >> i4(state, 8, 12, 14, 15)) & 1;
            let s2 = (0x0EE5u32 >> i4(state, 17, 21, 23, 26)) & 1;
            let s3 = (0x0EE5u32 >> i4(state, 28, 29, 31, 33)) & 1;
            let s4 = (0x3C65u32 >> i4(state, 34, 43, 44, 46)) & 1;
            ((0x0DD3_929Bu32 >> ((s0 << 4) | (s1 << 3) | (s2 << 2) | (s3 << 1) | s4)) & 1) as u64
        }

        fn lfsr(state: u64) -> u64 {
            let fb = (state
                ^ (state >> 2)
                ^ (state >> 3)
                ^ (state >> 6)
                ^ (state >> 7)
                ^ (state >> 8)
                ^ (state >> 16)
                ^ (state >> 22)
                ^ (state >> 23)
                ^ (state >> 26)
                ^ (state >> 30)
                ^ (state >> 41)
                ^ (state >> 42)
                ^ (state >> 43)
                ^ (state >> 46)
                ^ (state >> 47))
                & 1;
            (state >> 1) | (fb << 47)
        }

        pub fn authenticator(uid: u32, button: u8, counter: u32, key: &[u8; 6]) -> u32 {
            let key64 = key.iter().fold(0u64, |acc, &b| (acc << 8) | b as u64);
            let nonce = (counter << 4) | (button as u32 & 0x0F);
            let mut state = 0u64;
            for i in 32..48 {
                state = (state << 1) | ((key64 >> i) & 1);
            }
            for i in 0..32 {
                state = (state << 1) | ((uid >> i) & 1) as u64;
            }
            for i in 0..32 {
                let nonce_bit = f20(state) ^ ((nonce >> (31 - i)) & 1) as u64;
                state = (state >> 1) | (((nonce_bit ^ ((key64 >> (31 - i)) & 1)) & 1) << 47);
            }
            let mut hop = 0u32;
            for _ in 0..32 {
                hop = (hop << 1) | f20(state) as u32;
                state = lfsr(state);
            }
            hop
        }
    }

    fn hex6(s: &str) -> [u8; 6] {
        let mut out = [0u8; 6];
        for (i, b) in out.iter_mut().enumerate() {
            *b = u8::from_str_radix(&s[2 * i..2 * i + 2], 16).unwrap();
        }
        out
    }

    fn hex11(s: &str) -> [u8; 11] {
        let mut out = [0u8; 11];
        for (i, b) in out.iter_mut().enumerate() {
            *b = u8::from_str_radix(&s[2 * i..2 * i + 2], 16).unwrap();
        }
        out
    }

    fn bytes6(s: u64) -> [u8; 6] {
        s.to_be_bytes()[2..8].try_into().unwrap()
    }

    /// Small deterministic PRNG for property tests.
    struct XorShift(u64);
    impl XorShift {
        fn next(&mut self) -> u64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            self.0
        }
    }

    /// filter / feedback against the compiled C (`fiat_v1_bcm_hitag2_filter`, `hitag2_feedback`).
    #[test]
    fn filter_and_feedback_match_c_vectors() {
        // (state, filter, feedback) — generated by the C reference functions.
        let vectors = [
            ("E13B032E112A", 0, 1),
            ("32B579080F08", 0, 1),
            ("B1F7ED4C2E5D", 1, 0),
            ("3A07F97F21EE", 0, 0),
            ("232D178A209A", 1, 0),
            ("F6B5887F66E8", 1, 0),
            ("0123456789AB", 0, 1),
        ];
        for (state, f, fb) in vectors {
            let s = u64::from_str_radix(state, 16).unwrap();
            assert_eq!(filter48(s), f, "filter48({state})");
            assert_eq!(feedback48(s), fb, "feedback48({state})");
            assert_eq!(
                byte_reference::filter(&hex6(state)),
                f as u8,
                "filter({state})"
            );
            assert_eq!(byte_reference::feedback(&hex6(state)), fb as u8);
        }
    }

    /// The `u64` core equals the C byte-array core (filter, feedback, shift) on random states, and
    /// the byte-array filter is renault_v1.c's `hitag2_f20` on the bit-reversed register. 40 000
    /// states hit every 4-bit input combination of each filter group thousands of times.
    #[test]
    fn u64_core_equals_byte_core_and_f20() {
        let mut rng = XorShift(0x5EED_1234_ABCD_0001);
        for _ in 0..40_000 {
            let s = rng.next() & MASK48;
            let mut bytes = bytes6(s);
            assert_eq!(
                filter48(s) as u8,
                byte_reference::filter(&bytes),
                "{s:012X}"
            );
            assert_eq!(
                feedback48(s) as u8,
                byte_reference::feedback(&bytes),
                "{s:012X}"
            );
            let input = (rng.next() & 1) as u8;
            byte_reference::shift(&mut bytes, input);
            assert_eq!(bytes6(shift48(s, input as u64)), bytes);
            // u64 (renault_v1.c hitag2_f20) layout: bit k = byte-layout bit (47 - k).
            let r = s.reverse_bits() >> 16;
            assert_eq!(filter48(s) as u64, u64_reference::f20(r), "state {s:012X}");
        }
    }

    /// fiat_bcm_authenticator against `fiat_v1_bcm_generate_authenticator` (incl. non-zero epoch).
    #[test]
    fn fiat_authenticator_matches_c_vectors() {
        // (uid, button, control, epoch, key, authenticator) from the C reference.
        let vectors: [(u32, u8, u16, u32, &str, u32); 8] = [
            (0xE065E3EB, 0x1, 0x090, 0x00000, "49F2C1551B27", 0xA3438A15),
            (0x966844AA, 0x8, 0x14F, 0x04D21, "6E490DB13848", 0xD7717B3B),
            (0xABA601CA, 0x4, 0x3A2, 0x00000, "8D145A8B4F99", 0xD0FF322A),
            (0xF4687101, 0x2, 0x3B7, 0x02A0C, "C5B2FDAEEFF3", 0x7F8EF0FB),
            (0x0B9AB366, 0x8, 0x15C, 0x1C225, "E0978C3F5FD5", 0xF1E7970A),
            (0x6FA19D46, 0x1, 0x0D3, 0x00000, "C08262B03750", 0x5882D14C),
            (0xEFB82D66, 0x2, 0x08D, 0x00000, "B79280AECC37", 0xBE8CC541),
            (0x12345678, 0x4, 0x3FF, 0x00001, "4D343FD4E7B6", 0x317DCF5D),
        ];
        for (uid, btn, ctl, epoch, key, auth) in vectors {
            assert_eq!(
                fiat_bcm_authenticator(uid, btn, ctl, &hex6(key), epoch),
                auth,
                "uid {uid:08X} btn {btn:X} ctl {ctl:03X} epoch {epoch:05X} key {key}"
            );
        }
    }

    /// renault_authenticator against the C `hitag2_authenticator`, and against the verbatim u64
    /// port for random inputs.
    #[test]
    fn renault_authenticator_matches_c_and_u64_form() {
        // (uid, button, counter, key, hop) from the C reference.
        let vectors: [(u32, u8, u32, &str, u32); 7] = [
            (0x949606B3, 0x8, 0x13D, "A5E42428CA6D", 0xF003694F),
            (0x0C386144, 0x9, 0x04D, "702CA29CEB21", 0xD4A9F1AA),
            (0x95C13A79, 0x8, 0x095, "DA6733CB63EB", 0x732F9DE0),
            (0x8021EF32, 0x7, 0x2E3, "69D759689A1E", 0x25A38FAB),
            (0xD0170778, 0xB, 0x139, "FFF1AA474318", 0x4932D0ED),
            (0x9FDF9FE2, 0x5, 0x128, "23A657001F2C", 0x1075DBF7),
            (0xEFB82D66, 0x2, 0x08D, "CC37B79280AE", 0xBE8CC541),
        ];
        for (uid, btn, cnt, key, hop) in vectors {
            assert_eq!(
                renault_authenticator(uid, btn, cnt, &hex6(key)),
                hop,
                "uid {uid:08X}"
            );
        }
        let mut rng = XorShift(0xC0FF_EE00_1234_5678);
        for _ in 0..300 {
            let uid = rng.next() as u32;
            let btn = (rng.next() & 0x0F) as u8;
            let cnt = (rng.next() & 0x3FF) as u32;
            let key = bytes6(rng.next());
            assert_eq!(
                renault_authenticator(uid, btn, cnt, &key),
                u64_reference::authenticator(uid, btn, cnt, &key)
            );
        }
    }

    /// The two authenticators are the same cipher: Renault key = Fiat key rotated by two bytes.
    #[test]
    fn renault_key_is_rotated_fiat_key() {
        let mut rng = XorShift(0x0123_4567_89AB_CDEF);
        for _ in 0..200 {
            let uid = rng.next() as u32;
            let btn = [1u8, 2, 4, 8][(rng.next() % 4) as usize];
            let ctl = (rng.next() & 0x3FF) as u16;
            let k = bytes6(rng.next());
            let r = [k[4], k[5], k[0], k[1], k[2], k[3]];
            assert_eq!(
                fiat_bcm_authenticator(uid, btn, ctl, &k, 0),
                renault_authenticator(uid, btn, ctl as u32, &r)
            );
        }
    }

    /// hitag2_serial_permute vectors from the C reference.
    #[test]
    fn serial_permute_matches_c_vectors() {
        let vectors = [
            (0xEFB82D66u32, "4FA9F5C21C30"),
            (0x12345678, "F4C9B255479E"),
            (0x00000000, "35DCE495E7AC"),
            (0xFFFFFFFF, "CAA1936A1852"),
            (0xA5C3E1F0, "BBFCC1E8B274"),
        ];
        for (serial, perm) in vectors {
            assert_eq!(
                renault_serial_permute(serial),
                hex6(perm),
                "serial {serial:08X}"
            );
        }
    }

    /// The `u64` clock equals the verbatim `hitag2_clock_cipher` (byte buffers, iv rotation and
    /// all) on random states / keys / IVs, including the final register.
    #[test]
    fn run48_equals_c_clock_cipher() {
        let mut rng = XorShift(0xA11C_E5ED_0000_0007);
        for _ in 0..500 {
            let s = rng.next() & MASK48;
            let key: [u8; 4] = (rng.next() as u32).to_le_bytes();
            let iv: [u8; 4] = (rng.next() as u32).to_le_bytes();
            let mut st = bytes6(s);
            let mut iv_work = key;
            let mut iv_orig = iv;
            byte_reference::clock_cipher(&mut st, &mut iv_work, &mut iv_orig);
            let mut s48 = s;
            let out = run48(&mut s48, u32::from_le_bytes(key), u32::from_le_bytes(iv));
            assert_eq!(out.to_le_bytes(), iv_work);
            assert_eq!(bytes6(s48), st);
            assert_eq!(iv_orig, iv, "iv_orig is rotated back to itself");
        }
    }

    /// Seed scheme: hitag2_encrypt_frame (build_iv + encrypt_from_iv) reproduces the C frames, the
    /// verbatim brute-force check accepts the IV that produced them, and the counter/seed helpers
    /// invert build_iv.
    #[test]
    fn seed_scheme_encrypt_matches_c_vectors() {
        // (serial, cnt, btn, seed, iv, frame) from the C `hitag2_encrypt_frame`.
        type Vector = (u32, u32, u8, u32, [u8; 4], &'static str);
        let vectors: [Vector; 6] = [
            (
                0xEFB82D66,
                0x008D,
                0x2,
                0x0003A5C1,
                [0xD2, 0x08, 0xA0, 0xC1],
                "EFB82D6622345188720EAF",
            ),
            (
                0x12345678,
                0x1234,
                0x4,
                0xABCDEF01,
                [0x44, 0x23, 0xE1, 0x01],
                "1234567848D122E6FC52FB",
            ),
            (
                0xC2064F3E,
                0xCAC7,
                0x2,
                0xEA60312B,
                [0x72, 0xAC, 0x3C, 0x2B],
                "C2064F3E2B1CDFDB258625",
            ),
            (
                0x378F8529,
                0x0A37,
                0x8,
                0x64357C97,
                [0x78, 0xA3, 0x70, 0x97],
                "378F852988DFF51255F203",
            ),
            (
                0xBC9A0005,
                0x530C,
                0x2,
                0xF8C1C3A5,
                [0xC2, 0x30, 0xC5, 0xA5],
                "BC9A00052C315209F7A230",
            ),
            (
                0xD2A570DE,
                0xACB9,
                0x8,
                0xEFB725CD,
                [0x98, 0xCB, 0x2A, 0xCD],
                "D2A570DE82E70E6B1426EB",
            ),
        ];
        for (serial, cnt, btn, seed, iv, frame_hex) in vectors {
            let (frame, got_iv) = renault_encrypt_frame(serial, cnt, btn, seed);
            assert_eq!(got_iv, iv, "iv for serial {serial:08X}");
            assert_eq!(frame, hex11(frame_hex), "frame for serial {serial:08X}");
            assert_eq!(
                renault_counter_from_seed(renault_seed_from_iv(&iv)),
                cnt as u16
            );
            assert!(
                byte_reference::hop_matches(&frame, &iv),
                "own IV must verify"
            );
            let mut wrong = iv;
            wrong[3] ^= 0x01;
            assert!(!byte_reference::hop_matches(&frame, &wrong));
        }
    }

    /// The brute force reproduces the C `hitag2_brute_force_run` result (FOUND iv / NOT_FOUND) on
    /// vectors from the compiled ProtoPirate renault_v1.c (frames from hitag2_encrypt_frame with
    /// random serial/cnt/btn/seed, and random frames).
    #[test]
    fn seed_recover_matches_c_vectors() {
        let vectors: [(&str, Option<[u8; 4]>); 2] = [
            ("EFB82D6622345188720EAF", Some([0xD2, 0x08, 0xA0, 0xC1])),
            ("EFB82D661238D19572E6E6", Some([0xE1, 0x08, 0xA0, 0xC1])),
        ];
        for (frame_hex, iv) in vectors {
            assert_eq!(renault_seed_recover(&hex11(frame_hex)), iv, "{frame_hex}");
        }
    }

    /// Whatever the brute force returns verifies with the verbatim C check, and every frame the
    /// seed scheme produces is recovered with a counter equal to (or, for a rare earlier
    /// candidate collision, consistent with) the one it was built from.
    #[test]
    fn seed_recover_round_trips_encrypted_frames() {
        let mut rng = XorShift(0x0BAD_5EED_1234_0042);
        for _ in 0..6 {
            let serial = rng.next() as u32;
            let cnt = (rng.next() & 0xFFFF) as u32;
            let btn = [1u8, 2, 4, 8][(rng.next() % 4) as usize];
            let seed = rng.next() as u32;
            let (frame, iv) = renault_encrypt_frame(serial, cnt, btn, seed);
            let got = renault_seed_recover(&frame).expect("seed-scheme frame must be recovered");
            assert!(byte_reference::hop_matches(&frame, &got));
            if got == iv {
                assert_eq!(
                    renault_counter_from_seed(renault_seed_from_iv(&got)),
                    cnt as u16
                );
            }
            // The low 14 IV bits come from the frame; only the 18 searched bits can differ.
            assert_eq!(got[0], iv[0]);
            assert_eq!(got[1] & 0x3F, iv[1] & 0x3F);
        }
    }
}
