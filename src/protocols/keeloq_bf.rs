//! KeeLoq manufacturer-key recovery for "magic serial" remotes — Flipper-ARF's KeeLoq Decrypt
//! ("BF2") scenes, with the search the reference leaves to a companion app done here.
//!
//! # The reference
//!
//! Flipper-ARF's `subghz_scene_keeloq_bf2.c` takes two saved KeeLoq captures of one remote (same
//! serial, different hop codes; "Serial mismatch!" / "Same hop code!" otherwise) and a learning
//! type — Auto (6 → 7 → 8), 6 (Magic Serial 1), 7 (Magic Serial 2) or 8 (Magic Serial 3).
//! `subghz_scene_keeloq_decrypt.c` then sends {type, fix, hop, hop2, serial} over Bluetooth to a
//! companion app that searches 2^32 keys (`KL_TOTAL_KEYS`) and answers with the manufacturer key,
//! its type and the counter; the Flipper adds every key it reports to the user keystore as
//! `BF_<serial>` (seven hex digits) and re-saves the capture with that `Manufacture` and `Cnt`.
//! The companion app is neither in Flipper-ARF nor published, so KAT runs the search itself, on
//! every CPU core.
//!
//! # The search
//!
//! For the magic-serial learning types the device key is the remote's `fix` combined with part of
//! the manufacturer key (`keeloq_magic_serial_type{1,2,3}_learning`, which the decoder uses), so
//! only that part is unknown:
//!
//! | type | device key | unknown | candidates |
//! |---|---|---|---|
//! | 6, Magic Serial 1 | man[31:0], serial bytes, their sum | man[31:0] | 2^32 |
//! | 7, Magic Serial 2 | man[31:0], fix byte-swapped | man[31:0] | 2^32 |
//! | 8, Magic Serial 3 | man[63:24], serial[23:0] | man[63:24] | 2^40 |
//!
//! All candidates are tried. The keystore's own type 6/7 keys (KGB/Subaru, Magic_2) have zero
//! upper halves, as recovered keys do; its type 8 keys (Magic_3, Magic_4) use all 40 bits, so
//! type 8 needs the 2^40 search — about 256 times as long — and is not part of Auto (types 6 and
//! 7). ARF's scene computes the same 32-bit search for every type.
//!
//! Each candidate decrypts the first capture's hop code — 64 candidates per pass, bitsliced — and
//! must give that frame's button (4 bits) and discriminator `serial & 0x3FF` (10 bits; the `disc`
//! ARF's scene prepares). Survivors must decrypt every other capture the same way, with distinct
//! counters within ±[`COUNTER_WINDOW`] of the first. False keys: ~0.1 per 2^32 search from two
//! captures, ~0 from three; a 2^40 search from two would leave ~30, so type 8 needs three
//! captures. Candidates come back closest counters first.

use std::ops::{BitAnd, BitOr, BitXor, Not, Range};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Mutex;

use super::keeloq_common::{
    keeloq_decrypt, keeloq_magic_serial_type1_learning, keeloq_magic_serial_type2_learning,
    keeloq_magic_serial_type3_learning, learning_types as lt,
};

/// The decoder's name for a KeeLoq frame no key decrypts.
pub const UNKNOWN_NAME: &str = super::keeloq::UNKNOWN_NAME;

/// Decode a KeeLoq frame (`DecodedSignal::data`) with the current keystore, e.g. after a key was
/// added: the decoder's own result, named "KeeLoq (<manufacturer>)".
pub fn decode_frame(data: u64) -> Option<super::DecodedSignal> {
    super::keeloq::KeeloqDecoder::parse_data(data)
}

/// Learning types tried by "Auto": the two whose search is 2^32.
pub const AUTO_TYPES: [u32; 2] = [lt::MAGIC_SERIAL_TYPE_1, lt::MAGIC_SERIAL_TYPE_2];

/// Counters of the other captures must lie within this distance of the first capture's.
pub const COUNTER_WINDOW: u16 = 256;

/// Candidates per unit of work handed to a thread (progress and cancel granularity).
const UNIT: u64 = 1 << 20;

/// One received KeeLoq frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Frame {
    /// Button (4 bits) and serial (28 bits), sent in clear.
    pub fix: u32,
    /// The encrypted hop code.
    pub hop: u32,
}

impl Frame {
    /// The frame of a KeeLoq decode's `data` (the received 64-bit key).
    pub fn from_data(data: u64) -> Self {
        let (fix, hop) = super::keeloq::fix_hop(data);
        Self { fix, hop }
    }

    pub fn serial(&self) -> u32 {
        self.fix & 0x0FFF_FFFF
    }

    /// The hop plaintext's button and discriminator check (`serial & 0x3FF`).
    fn accepts(&self, plain: u32) -> bool {
        plain >> 28 == self.fix >> 28 && (plain >> 16) & 0x3FF == self.fix & 0x3FF
    }
}

/// A manufacturer key that decrypts every capture.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Candidate {
    /// KeeLoq learning type (6, 7 or 8).
    pub learning: u32,
    /// The manufacturer key, as the keystore stores it.
    pub man_key: u64,
    /// Each capture's counter, in the order given.
    pub counters: Vec<u16>,
}

impl Candidate {
    /// Largest counter distance from the first capture (smaller = more plausible).
    pub fn spread(&self) -> u16 {
        self.counters.iter().map(|&c| counter_distance(c, self.counters[0])).max().unwrap_or(0)
    }
}

/// Shared state of a running search: cancel flag and progress.
#[derive(Default)]
pub struct Control {
    pub cancel: AtomicBool,
    /// Candidates tried so far.
    pub tested: AtomicU64,
    /// Candidates to try in total (set by [`search`]).
    pub total: AtomicU64,
}

#[derive(Debug)]
pub enum Outcome {
    /// Keys that decrypt every capture, closest counters first.
    Found(Vec<Candidate>),
    NotFound,
    Cancelled,
}

/// Candidates of a learning type (2^32 for types 6 and 7, 2^40 for type 8).
pub fn space(learning: u32) -> u64 {
    1 << layout(learning).1
}

/// Check the captures and types before a search; the error is a user-facing message.
pub fn check_input(frames: &[Frame], learnings: &[u32]) -> Result<(), String> {
    if frames.len() < 2 {
        return Err("needs two captures of the same remote (different presses)".into());
    }
    if frames.iter().any(|f| f.serial() != frames[0].serial()) {
        return Err("the captures are from different remotes (serial mismatch)".into());
    }
    for (i, f) in frames.iter().enumerate() {
        if frames[..i].iter().any(|g| g.hop == f.hop) {
            return Err("two captures have the same hop code (the same press)".into());
        }
    }
    if learnings.is_empty() {
        return Err("no learning type".into());
    }
    for &l in learnings {
        if ![lt::MAGIC_SERIAL_TYPE_1, lt::MAGIC_SERIAL_TYPE_2, lt::MAGIC_SERIAL_TYPE_3].contains(&l) {
            return Err(format!("type {l} is not a magic-serial type (6, 7 or 8)"));
        }
        if l == lt::MAGIC_SERIAL_TYPE_3 && frames.len() < 3 {
            return Err("type 8 needs three captures of the same remote".into());
        }
    }
    Ok(())
}

/// Search the learning types in order, all their candidates; stops at the first type with a hit.
pub fn search(frames: &[Frame], learnings: &[u32], ctl: &Control) -> Outcome {
    if check_input(frames, learnings).is_err() {
        return Outcome::NotFound;
    }
    ctl.total.store(learnings.iter().map(|&l| space(l)).sum(), Ordering::Relaxed);
    for &learning in learnings {
        let found = search_range(frames, learning, 0..space(learning), ctl);
        if ctl.cancel.load(Ordering::Relaxed) {
            return Outcome::Cancelled;
        }
        if !found.is_empty() {
            return Outcome::Found(found);
        }
    }
    Outcome::NotFound
}

/// Try the candidates `range` of one learning type on all cores. A candidate is the unknown part
/// of the manufacturer key (see the module doc); the result is sorted closest counters first.
pub fn search_range(frames: &[Frame], learning: u32, range: Range<u64>, ctl: &Control) -> Vec<Candidate> {
    let (shift, width) = layout(learning);
    let end = range.end.min(1 << width);
    let first = frames[0];
    let known = device_key(learning, first.fix, 0);
    let next = AtomicU64::new(range.start & !(X4::COUNT - 1));
    let found = Mutex::new(Vec::new());
    let threads = std::thread::available_parallelism().map_or(1, |n| n.get());
    let job = Scan { frames, learning, known, shift, width, start: range.start, found: &found };
    let avx2 = has_avx2();
    std::thread::scope(|scope| {
        for _ in 0..threads {
            scope.spawn(|| loop {
                if ctl.cancel.load(Ordering::Relaxed) {
                    break;
                }
                let unit = next.fetch_add(UNIT, Ordering::Relaxed);
                if unit >= end {
                    break;
                }
                let unit_end = (unit + UNIT).min(end);
                job.run(unit, unit_end, avx2);
                ctl.tested.fetch_add(unit_end - unit.max(range.start), Ordering::Relaxed);
            });
        }
    });
    let mut found = found.into_inner().unwrap();
    found.sort_by_key(|c| (c.spread(), c.man_key));
    found
}

/// (shift, width) of the unknown part of the manufacturer key.
fn layout(learning: u32) -> (u32, u32) {
    if learning == lt::MAGIC_SERIAL_TYPE_3 {
        (24, 40)
    } else {
        (0, 32)
    }
}

/// The device key the decoder derives for this learning type, frame and manufacturer key.
pub fn device_key(learning: u32, fix: u32, man: u64) -> u64 {
    match learning {
        lt::MAGIC_SERIAL_TYPE_1 => keeloq_magic_serial_type1_learning(fix, man),
        lt::MAGIC_SERIAL_TYPE_2 => keeloq_magic_serial_type2_learning(fix, man),
        _ => keeloq_magic_serial_type3_learning(fix, man),
    }
}

/// Every capture decrypts with a valid button and discriminator; distinct counters near the first.
fn verify(frames: &[Frame], learning: u32, man: u64) -> Option<Vec<u16>> {
    let mut counters: Vec<u16> = Vec::with_capacity(frames.len());
    for f in frames {
        let plain = keeloq_decrypt(f.hop, device_key(learning, f.fix, man));
        let counter = plain as u16;
        if !f.accepts(plain)
            || counters.contains(&counter)
            || counters.first().is_some_and(|&c0| counter_distance(counter, c0) > COUNTER_WINDOW)
        {
            return None;
        }
        counters.push(counter);
    }
    Some(counters)
}

/// Distance between two 16-bit counters, either direction, with wraparound.
pub fn counter_distance(a: u16, b: u16) -> u16 {
    let d = a.wrapping_sub(b);
    d.min(d.wrapping_neg())
}

/// One unit of work: candidates `from..to` of one learning type against the captures.
struct Scan<'a> {
    frames: &'a [Frame],
    learning: u32,
    /// Device-key bits that don't depend on the candidate (from the first capture's fix).
    known: u64,
    shift: u32,
    width: u32,
    /// Start of the requested range (candidates below it are skipped).
    start: u64,
    found: &'a Mutex<Vec<Candidate>>,
}

impl Scan<'_> {
    fn run(&self, from: u64, to: u64, avx2: bool) {
        #[cfg(target_arch = "x86_64")]
        if avx2 {
            // SAFETY: `avx2` comes from `has_avx2()`, i.e. the CPU supports AVX2.
            return unsafe { self.run_avx2(from, to) };
        }
        let _ = avx2;
        self.run_lanes::<X4>(from, to);
    }

    /// The same loop compiled for AVX2 (the generic code is inlined into it).
    #[cfg(target_arch = "x86_64")]
    #[target_feature(enable = "avx2")]
    unsafe fn run_avx2(&self, from: u64, to: u64) {
        self.run_lanes::<X4>(from, to);
    }

    #[inline(always)]
    fn run_lanes<W: Lanes>(&self, from: u64, to: u64) {
        let first = self.frames[0];
        let mut base = from & !(W::COUNT - 1);
        while base < to {
            let keys = key_words::<W>(self.known, self.shift, self.width, base);
            let plain = decrypt_lanes(first.hop, &keys);
            let hits = pass_mask(&plain, first.fix);
            if hits.any() {
                for lane in hits.set_lanes() {
                    let candidate = base + u64::from(lane);
                    if candidate < self.start.max(from) || candidate >= to {
                        continue;
                    }
                    let man = candidate << self.shift;
                    if let Some(counters) = verify(self.frames, self.learning, man) {
                        self.found.lock().unwrap().push(Candidate { learning: self.learning, man_key: man, counters });
                    }
                }
            }
            base += W::COUNT;
        }
    }
}

/// Whether this CPU runs the AVX2 build of the search loop.
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

/// A bitsliced word: bit `j` of every word belongs to candidate lane `j`.
trait Lanes:
    Copy + BitAnd<Output = Self> + BitOr<Output = Self> + BitXor<Output = Self> + Not<Output = Self>
{
    /// Lanes per word (a power of two).
    const COUNT: u64;
    fn splat(bit: bool) -> Self;
    /// Lane `j` holds bit `i` of `j`, for `2^i < COUNT`.
    fn index_bit(i: u32) -> Self;
    fn any(self) -> bool;
    /// The set lanes, ascending.
    fn set_lanes(self) -> Vec<u32>;
}

/// Lane `j` of a word holds bit `i` of `j`, for the six bits of a 64-lane word.
const LANE_BITS: [u64; 6] = [
    0xAAAA_AAAA_AAAA_AAAA,
    0xCCCC_CCCC_CCCC_CCCC,
    0xF0F0_F0F0_F0F0_F0F0,
    0xFF00_FF00_FF00_FF00,
    0xFFFF_0000_FFFF_0000,
    0xFFFF_FFFF_0000_0000,
];

impl Lanes for u64 {
    const COUNT: u64 = 64;
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
    fn set_lanes(self) -> Vec<u32> {
        (0..64).filter(|&j| (self >> j) & 1 == 1).collect()
    }
}

/// 256 lanes in four 64-bit words (one AVX2 register; two NEON/SSE2 registers): lane `j` is bit
/// `j % 64` of word `j / 64`.
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
    const COUNT: u64 = 256;
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
    fn set_lanes(self) -> Vec<u32> {
        (0..4u32).flat_map(|w| self.0[w as usize].set_lanes().into_iter().map(move |j| w * 64 + j)).collect()
    }
}

/// The 64 device-key bits of candidates `base..base + W::COUNT` (`base` a multiple of the lane
/// count), one word per key bit.
#[inline(always)]
fn key_words<W: Lanes>(known: u64, shift: u32, width: u32, base: u64) -> [W; 64] {
    let index_bits = W::COUNT.trailing_zeros();
    let mut k = [W::splat(false); 64];
    for (i, word) in k.iter_mut().enumerate() {
        let i = i as u32;
        *word = if (shift..shift + index_bits).contains(&i) {
            W::index_bit(i - shift)
        } else if (shift..shift + width).contains(&i) {
            W::splat((base >> (i - shift)) & 1 == 1)
        } else {
            W::splat((known >> i) & 1 == 1)
        };
    }
    k
}

/// The KeeLoq NLF (0x3A5C742E, inputs x0, x8, x19, x25, x30) on all lanes, from its algebraic
/// normal form a⊕b⊕ab⊕bc⊕ad⊕cd⊕ae⊕abe⊕ce⊕ace⊕bde⊕cde = P ⊕ e·Q.
#[inline(always)]
fn nlf<W: Lanes>(a: W, b: W, c: W, d: W, e: W) -> W {
    let p = (a | b) ^ (b & c) ^ (d & (a ^ c));
    let q = (a & !b) ^ (c & !a) ^ (d & (b ^ c));
    p ^ (e & q)
}

/// One decryption round. Logical state bit `i` lives in `s[(H + i) % 32]`; the shift left moves
/// `H` down by one, so the new bit 0 takes the slot of the old bit 31.
macro_rules! round {
    ($s:ident, $k:ident, $r:expr, $h:literal) => {{
        let n = $s[($h + 31) % 32]
            ^ $s[($h + 15) % 32]
            ^ $k[15usize.wrapping_sub($r) & 63]
            ^ nlf($s[$h % 32], $s[($h + 8) % 32], $s[($h + 19) % 32], $s[($h + 25) % 32], $s[($h + 30) % 32]);
        $s[($h + 31) % 32] = n;
    }};
}

/// Rounds `r0..r0 + 16` starting at `H` = 0 (ending at `H` = 16).
#[inline(always)]
fn rounds_from_0<W: Lanes>(s: &mut [W; 32], k: &[W; 64], r0: usize) {
    round!(s, k, r0, 0);
    round!(s, k, r0 + 1, 31);
    round!(s, k, r0 + 2, 30);
    round!(s, k, r0 + 3, 29);
    round!(s, k, r0 + 4, 28);
    round!(s, k, r0 + 5, 27);
    round!(s, k, r0 + 6, 26);
    round!(s, k, r0 + 7, 25);
    round!(s, k, r0 + 8, 24);
    round!(s, k, r0 + 9, 23);
    round!(s, k, r0 + 10, 22);
    round!(s, k, r0 + 11, 21);
    round!(s, k, r0 + 12, 20);
    round!(s, k, r0 + 13, 19);
    round!(s, k, r0 + 14, 18);
    round!(s, k, r0 + 15, 17);
}

/// Rounds `r0..r0 + 16` starting at `H` = 16 (ending at `H` = 0).
#[inline(always)]
fn rounds_from_16<W: Lanes>(s: &mut [W; 32], k: &[W; 64], r0: usize) {
    round!(s, k, r0, 16);
    round!(s, k, r0 + 1, 15);
    round!(s, k, r0 + 2, 14);
    round!(s, k, r0 + 3, 13);
    round!(s, k, r0 + 4, 12);
    round!(s, k, r0 + 5, 11);
    round!(s, k, r0 + 6, 10);
    round!(s, k, r0 + 7, 9);
    round!(s, k, r0 + 8, 8);
    round!(s, k, r0 + 9, 7);
    round!(s, k, r0 + 10, 6);
    round!(s, k, r0 + 11, 5);
    round!(s, k, r0 + 12, 4);
    round!(s, k, r0 + 13, 3);
    round!(s, k, r0 + 14, 2);
    round!(s, k, r0 + 15, 1);
}

/// KeeLoq decryption (`keeloq_decrypt`, 528 rounds) of `hop` under every lane's key: the
/// plaintext's 32 bits, one word per bit.
#[inline(always)]
fn decrypt_lanes<W: Lanes>(hop: u32, k: &[W; 64]) -> [W; 32] {
    let mut s = [W::splat(false); 32];
    for (i, word) in s.iter_mut().enumerate() {
        *word = W::splat((hop >> i) & 1 == 1);
    }
    for block in 0..16 {
        rounds_from_0(&mut s, k, block * 32);
        rounds_from_16(&mut s, k, block * 32 + 16);
    }
    rounds_from_0(&mut s, k, 512);
    let mut out = [W::splat(false); 32];
    for (i, word) in out.iter_mut().enumerate() {
        *word = s[(16 + i) % 32];
    }
    out
}

/// Lanes whose plaintext has the frame's button (bits 28..32) and discriminator (bits 16..26).
#[inline(always)]
fn pass_mask<W: Lanes>(plain: &[W; 32], fix: u32) -> W {
    let want = |word: W, bit: u32| if bit & 1 == 1 { word } else { !word };
    let mut m = W::splat(true);
    for (i, &word) in plain.iter().enumerate().skip(16) {
        let i = i as u32;
        let bit = match i {
            16..=25 => fix >> (i - 16),
            28..=31 => fix >> i,
            _ => continue,
        };
        m = m & want(word, bit);
    }
    m
}

#[cfg(test)]
mod tests {
    use super::super::keeloq_common::{keeloq_encrypt, keeloq_keystore};
    use super::*;

    /// A frame the remote would send: `plain` = button, discriminator (serial & 0x3FF), counter.
    fn press(learning: u32, man: u64, fix: u32, counter: u16) -> Frame {
        let plain = (fix >> 28) << 28 | (fix & 0x3FF) << 16 | u32::from(counter);
        Frame { fix, hop: keeloq_encrypt(plain, device_key(learning, fix, man)) }
    }

    fn rng(seed: &mut u64) -> u64 {
        *seed ^= *seed << 13;
        *seed ^= *seed >> 7;
        *seed ^= *seed << 17;
        *seed
    }

    #[test]
    fn nlf_matches_the_table() {
        for m in 0..32u64 {
            let bit = |i: u32| if (m >> i) & 1 == 1 { !0 } else { 0 };
            let want = if (0x3A5C_742Eu32 >> m) & 1 == 1 { !0 } else { 0 };
            assert_eq!(nlf(bit(0), bit(1), bit(2), bit(3), bit(4)), want, "input {m}");
        }
    }

    /// Every lane of the bitsliced decryption equals the scalar one, for both word types and all
    /// three layouts.
    fn check_lanes<W: Lanes>(lane_bit: impl Fn(&W, u64) -> bool) {
        let mut seed = 0x1234_5678_9ABC_DEF1u64;
        for learning in [lt::MAGIC_SERIAL_TYPE_1, lt::MAGIC_SERIAL_TYPE_2, lt::MAGIC_SERIAL_TYPE_3] {
            let (shift, width) = layout(learning);
            for _ in 0..4 {
                let fix = rng(&mut seed) as u32;
                let hop = rng(&mut seed) as u32;
                let base = (rng(&mut seed) & ((1 << width) - 1)) & !(W::COUNT - 1);
                let keys = key_words::<W>(device_key(learning, fix, 0), shift, width, base);
                let plain = decrypt_lanes(hop, &keys);
                for lane in 0..W::COUNT {
                    let key = device_key(learning, fix, (base + lane) << shift);
                    let lane_plain = (0..32).fold(0u32, |p, i| p | u32::from(lane_bit(&plain[i], lane)) << i);
                    assert_eq!(lane_plain, keeloq_decrypt(hop, key), "type {learning} lane {lane}");
                }
            }
        }
    }

    #[test]
    fn bitsliced_decrypt_matches_scalar() {
        check_lanes::<u64>(|w, lane| (w >> lane) & 1 == 1);
        check_lanes::<X4>(|w, lane| (w.0[(lane / 64) as usize] >> (lane % 64)) & 1 == 1);
    }

    /// Two presses of a type 6 and a type 7 remote: the search over a window around the key finds
    /// exactly that key, with the presses' counters.
    #[test]
    fn finds_magic_serial_1_and_2_keys() {
        for (learning, man) in [(lt::MAGIC_SERIAL_TYPE_1, 0x6D2A_91C3u64), (lt::MAGIC_SERIAL_TYPE_2, 0x0F31_77E4)] {
            let fix = 0x20C4_9A51; // button 2, serial 0x0C49A51
            let frames = [press(learning, man, fix, 0x0412), press(learning, man, fix | 0x4000_0000, 0x0413)];
            let ctl = Control::default();
            let found = search_range(&frames, learning, man - 5000..man + 5000, &ctl);
            assert_eq!(found, vec![Candidate { learning, man_key: man, counters: vec![0x0412, 0x0413] }]);
            assert_eq!(ctl.tested.load(Ordering::Relaxed), 10_000);
        }
    }

    /// The real HCS200 remote of rtl_433's test recordings (Microchip/HCS200/01, serial D0921), whose
    /// key is the keystore's Magic_3 (type 8): three presses recover exactly that key.
    #[test]
    fn finds_the_real_hcs200_magic_3_key() {
        let magic3 = keeloq_keystore().iter().find(|k| k.name == "Magic_3").expect("Magic_3").key;
        // KAT's `data` of g001 (BTN1, counter ABE), g006 (BTN2, AC4) and g012 (BTN3, ACA).
        let frames: Vec<Frame> =
            [0x1DB4_F14A_8490_B004u64, 0x28C9_83D1_8490_B002, 0xEF48_E816_8490_B009].map(Frame::from_data).into();
        check_input(&frames, &[lt::MAGIC_SERIAL_TYPE_3]).unwrap();
        let unknown = magic3 >> 24;
        let found = search_range(&frames, lt::MAGIC_SERIAL_TYPE_3, unknown - 3000..unknown + 3000, &Control::default());
        assert_eq!(found.len(), 1);
        assert_eq!((found[0].man_key, found[0].counters.clone()), (magic3, vec![0x0ABE, 0x0AC4, 0x0ACA]));
    }

    #[test]
    fn rejects_bad_input() {
        let a = Frame { fix: 0x1000_0001, hop: 1 };
        let b = Frame { fix: 0x2000_0001, hop: 2 };
        assert!(check_input(&[a], &AUTO_TYPES).is_err());
        assert!(check_input(&[a, Frame { fix: 2, hop: 2 }], &AUTO_TYPES).is_err());
        assert!(check_input(&[a, Frame { hop: 1, ..b }], &AUTO_TYPES).is_err());
        assert!(check_input(&[a, b], &[lt::MAGIC_SERIAL_TYPE_3]).is_err());
        assert!(check_input(&[a, b], &[5]).is_err());
        assert!(check_input(&[a, b], &AUTO_TYPES).is_ok());
    }

    #[test]
    fn counter_distance_wraps() {
        assert_eq!(counter_distance(5, 3), 2);
        assert_eq!(counter_distance(3, 5), 2);
        assert_eq!(counter_distance(0xFFFF, 1), 2);
    }

    /// Full 2^32 searches (release build: `cargo test --release keeloq_bf -- --ignored --nocapture`):
    /// each finds the key of a random type 6 / 7 remote, and reports the rate.
    #[test]
    #[ignore]
    fn full_search_finds_random_keys() {
        let mut seed = 0xC0FF_EE12_3456_789Au64;
        for learning in AUTO_TYPES {
            let man = rng(&mut seed) & 0xFFFF_FFFF;
            let fix = (rng(&mut seed) as u32 & 0x0FFF_FFFF) | 0x1000_0000;
            let frames = [press(learning, man, fix, 0x2201), press(learning, man, fix, 0x2203)];
            let ctl = Control::default();
            let t = std::time::Instant::now();
            let outcome = search(&frames, &[learning], &ctl);
            let secs = t.elapsed().as_secs_f64();
            eprintln!("type {learning}: {:.1} s, {:.0} M keys/s, {outcome:?}", secs, space(learning) as f64 / secs / 1e6);
            let Outcome::Found(found) = outcome else { panic!("type {learning}: not found") };
            assert_eq!(found[0].man_key, man);
        }
    }
}
