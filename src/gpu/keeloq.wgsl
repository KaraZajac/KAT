// Bitsliced KeeLoq manufacturer-key filter — the GPU counterpart of keeloq_bf.rs's CPU kernel.
//
// Each thread tries 32 candidate manufacturer keys at once (lane = bit j of each word), exactly as
// the CPU path does with a u32 word. For every frame it rebuilds the device key from that frame's
// `known` bits with the candidate overlaid at [shift, shift+width) — the identity placement
// device_key() uses for all three magic-serial types — decrypts the hop, and checks the button and
// discriminator (pass_mask). A candidate survives only if it passes on every frame; survivors are
// emitted as (c_lo, c_hi). The CPU then runs the authoritative verify() on the few survivors, so the
// result matches the CPU-only search bit for bit. A wrong key passing every frame's 14-bit check is
// the same coincidence the CPU sees (~2^-14 per extra frame); type 8 requires three frames.

struct Params {
  shift: u32,
  width: u32,
  n_frames: u32,
  base_lo: u32,   // this dispatch's first thread index (gt), low 32 bits
  base_hi: u32,   // and high bits (gt can exceed 2^32 for width 40)
  threads: u32,   // threads in this dispatch
  _pad0: u32,
  _pad1: u32,
};

struct Frame { fix: u32, hop: u32, known_lo: u32, known_hi: u32 };

struct Out { count: atomic<u32>, pairs: array<u32> };   // count, then (c_lo, c_hi) per survivor

@group(0) @binding(0) var<uniform> p: Params;
@group(0) @binding(1) var<storage, read> frames: array<Frame>;
@group(0) @binding(2) var<storage, read_write> out: Out;

const NLF: u32 = 0x3A5C742Eu;
const LANE0: u32 = 0xAAAAAAAAu;
const LANE1: u32 = 0xCCCCCCCCu;
const LANE2: u32 = 0xF0F0F0F0u;
const LANE3: u32 = 0xFF00FF00u;
const LANE4: u32 = 0xFFFF0000u;
const CAP: u32 = 4096u;   // survivor slots (host sizes the buffer to match); overflow is dropped

fn splat(b: u32) -> u32 { return select(0u, 0xFFFFFFFFu, b == 1u); }

fn lane_pattern(i: u32) -> u32 {
  switch i {
    case 0u: { return LANE0; }
    case 1u: { return LANE1; }
    case 2u: { return LANE2; }
    case 3u: { return LANE3; }
    default: { return LANE4; }
  }
}

// NLF (0x3A5C742E) over lanes, from the algebraic normal form used in keeloq_bf.rs.
fn nlf(a: u32, b: u32, c: u32, d: u32, e: u32) -> u32 {
  let pp = (a | b) ^ (b & c) ^ (d & (a ^ c));
  let qq = (a & ~b) ^ (c & ~a) ^ (d & (b ^ c));
  return pp ^ (e & qq);
}

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
  let lt = gid.x;
  if (lt >= p.threads) { return; }

  // gt = base + lt (64-bit): the thread's candidate block, candidate c = (gt << 5) | lane.
  let gt_lo = p.base_lo + lt;
  let carry = select(0u, 1u, gt_lo < p.base_lo);
  let gt_hi = p.base_hi + carry;

  var alive: u32 = 0xFFFFFFFFu;   // lanes still matching every frame so far

  for (var f = 0u; f < p.n_frames; f = f + 1u) {
    let fr = frames[f];

    // device-key words k[0..64]
    var k: array<u32, 64>;
    for (var i = 0u; i < 64u; i = i + 1u) {
      if (i >= p.shift && i < p.shift + 5u) {
        k[i] = lane_pattern(i - p.shift);
      } else if (i >= p.shift + 5u && i < p.shift + p.width) {
        let b = i - p.shift - 5u;            // candidate bit (5 + b); gt bit b
        var kbit: u32;
        if (b < 32u) { kbit = (gt_lo >> b) & 1u; } else { kbit = (gt_hi >> (b - 32u)) & 1u; }
        k[i] = splat(kbit);
      } else {
        var kn: u32;
        if (i < 32u) { kn = (fr.known_lo >> i) & 1u; } else { kn = (fr.known_hi >> (i - 32u)) & 1u; }
        k[i] = splat(kn);
      }
    }

    // state s[0..32] = hop bits (same on every lane)
    var s: array<u32, 32>;
    for (var i = 0u; i < 32u; i = i + 1u) { s[i] = splat((fr.hop >> i) & 1u); }

    // 528 decrypt rounds (keeloq_decrypt): logical bit i is s[i]; shift left each round.
    for (var r = 0u; r < 528u; r = r + 1u) {
      let idx = (15u - r) & 63u;
      let kb = k[idx];
      let nl = nlf(s[0], s[8], s[19], s[25], s[30]);
      let new_lsb = s[31] ^ s[15] ^ kb ^ nl;
      for (var i = 31u; i > 0u; i = i - 1u) { s[i] = s[i - 1u]; }
      s[0] = new_lsb;
    }

    // pass_mask(fix): button (plain[28..32] == fix[28..32]) and disc (plain[16..26] == fix[0..10])
    var m: u32 = alive;
    for (var i = 16u; i < 26u; i = i + 1u) {
      let wbit = (fr.fix >> (i - 16u)) & 1u;
      if (wbit == 1u) { m = m & s[i]; } else { m = m & ~s[i]; }
    }
    for (var i = 28u; i < 32u; i = i + 1u) {
      let wbit = (fr.fix >> i) & 1u;
      if (wbit == 1u) { m = m & s[i]; } else { m = m & ~s[i]; }
    }
    alive = m;
    if (alive == 0u) { return; }   // no lane can survive the rest
  }

  // emit surviving candidates c = (gt << 5) | lane
  for (var lane = 0u; lane < 32u; lane = lane + 1u) {
    if (((alive >> lane) & 1u) == 1u) {
      let c_lo = (gt_lo << 5u) | lane;
      let c_hi = (gt_hi << 5u) | (gt_lo >> 27u);
      let slot = atomicAdd(&out.count, 1u);
      if (slot < CAP) { out.pairs[slot * 2u] = c_lo; out.pairs[slot * 2u + 1u] = c_hi; }
    }
  }
}
