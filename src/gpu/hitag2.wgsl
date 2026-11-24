// KAT HiTag2 Hell recovery kernel (cargo feature `gpu`). Bitsliced find_state + emulated-48-bit
// verify. One thread per (candidate, i1). Emits the candidate indices that yield a state verifying
// any request; the CPU finalises the lowest one. Assembled from probe kernels proven bit-exact vs
// src/protocols/hitag2_hell.rs (verify chain, find_state, and end-to-end recovery of planted keys).

struct Params { n_cands: u32, hop: u32, poly_lo: u32, poly_hi: u32, n_reqs: u32, base_idx: u32, _p0: u32, _p1: u32 };
struct Req { uid: u32, cnt: u32, btn: u32, has_second: u32, s_hop: u32, s_cnt: u32, s_btn: u32, full_nonce: u32 };
struct Out { count: atomic<u32>, idx: array<u32> };  // count, then one candidate index per hit

@group(0) @binding(0) var<uniform> p: Params;
@group(0) @binding(1) var<storage, read> cands: array<u32>;
@group(0) @binding(2) var<storage, read> reqs: array<Req>;
@group(0) @binding(3) var<storage, read_write> out: Out;

const CAPH: u32 = 4096u;
const MASK_HI: u32 = 0xFFFFu;

fn bit48(s: vec2<u32>, n: u32) -> u32 {
  if (n < 32u) { return (s.x >> n) & 1u; }
  return (s.y >> (n - 32u)) & 1u;
}
fn shr1(s: vec2<u32>) -> vec2<u32> {
  return vec2((s.x >> 1u) | (s.y << 31u), (s.y >> 1u) & MASK_HI);
}
fn shl1(s: vec2<u32>) -> vec2<u32> {
  return vec2(s.x << 1u, ((s.y << 1u) | (s.x >> 31u)) & MASK_HI);
}
fn i4(s: vec2<u32>, a: u32, b: u32, c: u32, d: u32) -> u32 {
  return (bit48(s, a) << 3u) | (bit48(s, b) << 2u) | (bit48(s, c) << 1u) | bit48(s, d);
}
fn f20(s: vec2<u32>) -> u32 {
  let s0 = (0x3C65u >> i4(s, 2u, 3u, 5u, 6u)) & 1u;
  let s1 = (0x0EE5u >> i4(s, 8u, 12u, 14u, 15u)) & 1u;
  let s2 = (0x0EE5u >> i4(s, 17u, 21u, 23u, 26u)) & 1u;
  let s3 = (0x0EE5u >> i4(s, 28u, 29u, 31u, 33u)) & 1u;
  let s4 = (0x3C65u >> i4(s, 34u, 43u, 44u, 46u)) & 1u;
  return (0x0DD3929Bu >> ((s0 << 4u) | (s1 << 3u) | (s2 << 2u) | (s3 << 1u) | s4)) & 1u;
}
fn lfsr(s: vec2<u32>) -> vec2<u32> {
  let fb = bit48(s,0u)^bit48(s,2u)^bit48(s,3u)^bit48(s,6u)^bit48(s,7u)^bit48(s,8u)^bit48(s,16u)
         ^ bit48(s,22u)^bit48(s,23u)^bit48(s,26u)^bit48(s,30u)^bit48(s,41u)^bit48(s,42u)^bit48(s,43u)
         ^ bit48(s,46u)^bit48(s,47u);
  let r = shr1(s);
  return vec2(r.x, r.y | (fb << 15u));   // feedback into bit 47
}
fn lfsr_inv(s: vec2<u32>) -> vec2<u32> {
  let parity = (countOneBits(s.x & p.poly_lo) + countOneBits(s.y & p.poly_hi)) & 1u;
  let sl = shl1(s);
  return vec2(sl.x | parity, sl.y);       // (state << 1) | parity
}
fn finish_state(raw: vec2<u32>) -> vec2<u32> {
  return lfsr_inv(lfsr_inv(raw));
}
fn rev16(x: u32) -> u32 {
  return reverseBits(x & 0xFFFFu) >> 16u;
}

// init(key, uid, cnt, btn): the register after 32 init rounds.
fn init_state(key: vec2<u32>, uid: u32, cnt: u32, btn: u32) -> vec2<u32> {
  let nonce = (cnt << 4u) | (btn & 0xFu);
  var st = vec2(0u, 0u);
  for (var i = 32u; i < 48u; i = i + 1u) { st = shl1(st); st.x = st.x | bit48(key, i); }
  for (var i = 0u; i < 32u; i = i + 1u) { st = shl1(st); st.x = st.x | ((uid >> i) & 1u); }
  for (var i = 0u; i < 32u; i = i + 1u) {
    let nonce_bit = f20(st) ^ ((nonce >> (31u - i)) & 1u);
    let inj = (nonce_bit ^ ((key.x >> (31u - i)) & 1u)) & 1u;   // key bits 0..31 live in key.x
    st = shr1(st);
    st.y = st.y | (inj << 15u);
  }
  return st;
}
fn ks(state: vec2<u32>) -> u32 {
  var st = state;
  var c = 0u;
  for (var i = 0u; i < 32u; i = i + 1u) { c = (c << 1u) | f20(st); st = lfsr(st); }
  return c;
}
fn authenticator(key: vec2<u32>, uid: u32, cnt: u32, btn: u32) -> u32 {
  return ks(init_state(key, uid, cnt, btn));
}

// equiv_key(init, uid, cnt, btn): the key leaving the register at `init` after init rounds.
fn equiv_key(init_reg: vec2<u32>, uid: u32, cnt: u32, btn: u32, full_nonce: u32) -> vec2<u32> {
  let key_hi16 = rev16(init_reg.x & 0xFFFFu);        // init & 0xFFFF, reversed
  // key = key_hi16 << 32  → lives in key.y (bits 32..47)
  var st = vec2(0u, 0u);
  for (var i = 32u; i < 48u; i = i + 1u) { st = shl1(st); st.x = st.x | ((key_hi16 >> (i - 32u)) & 1u); }
  for (var i = 0u; i < 32u; i = i + 1u) { st = shl1(st); st.x = st.x | ((uid >> i) & 1u); }
  let nonce = (cnt << 4u) | (btn & 0xFu);
  var key_lo = 0u;
  for (var i = 0u; i < 32u; i = i + 1u) {
    let inj = bit48(init_reg, 16u + i);
    let f = f20(st);
    var nbit = 0u;
    if (full_nonce == 1u || i >= 18u) { nbit = (nonce >> (31u - i)) & 1u; }
    key_lo = key_lo | (((inj ^ f ^ nbit) & 1u) << (31u - i));
    st = shr1(st);
    st.y = st.y | (inj << 15u);
  }
  return vec2(key_lo, key_hi16 & MASK_HI);
}
fn splat(b: u32) -> u32 { return select(0u, 0xFFFFFFFFu, b == 1u); }
fn f_a(a: u32, b: u32, c: u32, d: u32) -> u32 { return ~(((a | b) & c) ^ (a | d) ^ b); }
fn f_b(a: u32, b: u32, c: u32, d: u32) -> u32 { return ~(((d | c) & (a ^ b)) ^ (d | a | b)); }
fn f_c(a: u32, b: u32, c: u32, d: u32, e: u32) -> u32 {
  return ~((((((c ^ e) | d) & a) ^ b) & (c ^ b)) ^ (((d ^ e) | a) & ((d ^ b) | c)));
}
fn g0(s: ptr<function, array<u32,80>>, t: u32) -> u32 { return f_a((*s)[t+2u], (*s)[t+3u], (*s)[t+5u], (*s)[t+6u]); }
fn g1(s: ptr<function, array<u32,80>>, t: u32) -> u32 { return f_b((*s)[t+8u], (*s)[t+12u], (*s)[t+14u], (*s)[t+15u]); }
fn g2(s: ptr<function, array<u32,80>>, t: u32) -> u32 { return f_b((*s)[t+17u], (*s)[t+21u], (*s)[t+23u], (*s)[t+26u]); }
fn g3(s: ptr<function, array<u32,80>>, t: u32) -> u32 { return f_b((*s)[t+28u], (*s)[t+29u], (*s)[t+31u], (*s)[t+33u]); }
fn g4(s: ptr<function, array<u32,80>>, t: u32) -> u32 { return f_a((*s)[t+34u], (*s)[t+43u], (*s)[t+44u], (*s)[t+46u]); }
fn fc_at(s: ptr<function, array<u32,80>>, t: u32) -> u32 { return f_c(g0(s,t), g1(s,t), g2(s,t), g3(s,t), g4(s,t)); }
fn lfsr_bs(s: ptr<function, array<u32,80>>, i: u32) -> u32 {
  return (*s)[i] ^ (*s)[i+2u] ^ (*s)[i+3u] ^ (*s)[i+6u] ^ (*s)[i+7u] ^ (*s)[i+8u] ^ (*s)[i+16u]
       ^ (*s)[i+22u] ^ (*s)[i+23u] ^ (*s)[i+26u] ^ (*s)[i+30u] ^ (*s)[i+41u] ^ (*s)[i+42u]
       ^ (*s)[i+43u] ^ (*s)[i+46u] ^ (*s)[i+47u];
}
fn guess(v: u32, k: u32) -> u32 { return splat((v >> k) & 1u); }

fn lane_pattern(i: u32) -> u32 {
  switch i { case 0u: { return 0xAAAAAAAAu; } case 1u: { return 0xCCCCCCCCu; }
             case 2u: { return 0xF0F0F0F0u; } case 3u: { return 0xFF00FF00u; } default: { return 0xFFFF0000u; } }
}

fn try_emit(raw: vec2<u32>, cand_index: u32) {
  let init_reg = finish_state(raw);
  for (var ri = 0u; ri < p.n_reqs; ri = ri + 1u) {
    let rq = reqs[ri];
    var uids = array<u32,2>(rq.uid, ~rq.uid);
    for (var u = 0u; u < 2u; u = u + 1u) {
      let uid = uids[u];
      let key = equiv_key(init_reg, uid, rq.cnt, rq.btn, rq.full_nonce);
      if (authenticator(key, uid, rq.cnt, rq.btn) != p.hop) { continue; }
      if (rq.has_second == 1u && authenticator(key, uid, rq.s_cnt, rq.s_btn) != rq.s_hop) { continue; }
      let slot = atomicAdd(&out.count, 1u);
      if (slot < CAPH) { out.idx[slot] = cand_index; }
      return;
    }
  }
}

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
  let t = gid.x;
  let total = p.n_cands * 1024u;
  if (t >= total) { return; }
  let cand_index = t / 1024u;
  let i1 = t % 1024u;
  let candidate = cands[cand_index*2u];
  let cand_hi = cands[cand_index*2u + 1u];

  var ks: array<u32, 32>;
  ks[0] = splat(select(1u, 0u, ((p.hop >> 31u) & 1u) == 1u));
  ks[1] = splat(select(1u, 0u, ((p.hop >> 30u) & 1u) == 1u));
  ks[2] = splat(select(1u, 0u, ((p.hop >> 29u) & 1u) == 1u));
  ks[3] = splat(select(1u, 0u, ((p.hop >> 28u) & 1u) == 1u));
  ks[4] = splat(select(1u, 0u, ((p.hop >> 27u) & 1u) == 1u));
  ks[5] = splat(select(1u, 0u, ((p.hop >> 26u) & 1u) == 1u));
  ks[6] = splat(select(1u, 0u, ((p.hop >> 25u) & 1u) == 1u));
  ks[7] = splat(select(1u, 0u, ((p.hop >> 24u) & 1u) == 1u));
  ks[8] = splat(select(1u, 0u, ((p.hop >> 23u) & 1u) == 1u));
  ks[9] = splat(select(1u, 0u, ((p.hop >> 22u) & 1u) == 1u));
  ks[10] = splat(select(1u, 0u, ((p.hop >> 21u) & 1u) == 1u));
  ks[11] = splat(select(1u, 0u, ((p.hop >> 20u) & 1u) == 1u));
  ks[12] = splat(select(1u, 0u, ((p.hop >> 19u) & 1u) == 1u));
  ks[13] = splat(select(1u, 0u, ((p.hop >> 18u) & 1u) == 1u));
  ks[14] = splat(select(1u, 0u, ((p.hop >> 17u) & 1u) == 1u));
  ks[15] = splat(select(1u, 0u, ((p.hop >> 16u) & 1u) == 1u));
  ks[16] = splat(select(1u, 0u, ((p.hop >> 15u) & 1u) == 1u));
  ks[17] = splat(select(1u, 0u, ((p.hop >> 14u) & 1u) == 1u));
  ks[18] = splat(select(1u, 0u, ((p.hop >> 13u) & 1u) == 1u));
  ks[19] = splat(select(1u, 0u, ((p.hop >> 12u) & 1u) == 1u));
  ks[20] = splat(select(1u, 0u, ((p.hop >> 11u) & 1u) == 1u));
  ks[21] = splat(select(1u, 0u, ((p.hop >> 10u) & 1u) == 1u));
  ks[22] = splat(select(1u, 0u, ((p.hop >> 9u) & 1u) == 1u));
  ks[23] = splat(select(1u, 0u, ((p.hop >> 8u) & 1u) == 1u));
  ks[24] = splat(select(1u, 0u, ((p.hop >> 7u) & 1u) == 1u));
  ks[25] = splat(select(1u, 0u, ((p.hop >> 6u) & 1u) == 1u));
  ks[26] = splat(select(1u, 0u, ((p.hop >> 5u) & 1u) == 1u));
  ks[27] = splat(select(1u, 0u, ((p.hop >> 4u) & 1u) == 1u));
  ks[28] = splat(select(1u, 0u, ((p.hop >> 3u) & 1u) == 1u));
  ks[29] = splat(select(1u, 0u, ((p.hop >> 2u) & 1u) == 1u));
  ks[30] = splat(select(1u, 0u, ((p.hop >> 1u) & 1u) == 1u));
  ks[31] = splat(select(1u, 0u, ((p.hop >> 0u) & 1u) == 1u));
  var s: array<u32, 80>;   // WGSL zero-initialises function-scope arrays
  s[2] = splat((candidate >> 0u) & 1u);
  s[3] = splat((candidate >> 1u) & 1u);
  s[4] = splat((candidate >> 2u) & 1u);
  s[5] = splat((candidate >> 3u) & 1u);
  s[6] = splat((candidate >> 4u) & 1u);
  s[7] = splat((candidate >> 5u) & 1u);
  s[8] = splat((candidate >> 6u) & 1u);
  s[9] = splat((candidate >> 7u) & 1u);
  s[10] = splat((candidate >> 8u) & 1u);
  s[11] = splat((candidate >> 9u) & 1u);
  s[12] = splat((candidate >> 10u) & 1u);
  s[13] = splat((candidate >> 11u) & 1u);
  s[14] = splat((candidate >> 12u) & 1u);
  s[15] = splat((candidate >> 13u) & 1u);
  s[16] = splat((candidate >> 14u) & 1u);
  s[17] = splat((candidate >> 15u) & 1u);
  s[18] = splat((candidate >> 16u) & 1u);
  s[19] = splat((candidate >> 17u) & 1u);
  s[20] = splat((candidate >> 18u) & 1u);
  s[21] = splat((candidate >> 19u) & 1u);
  s[22] = splat((candidate >> 20u) & 1u);
  s[23] = splat((candidate >> 21u) & 1u);
  s[24] = splat((candidate >> 22u) & 1u);
  s[25] = splat((candidate >> 23u) & 1u);
  s[26] = splat((candidate >> 24u) & 1u);
  s[27] = splat((candidate >> 25u) & 1u);
  s[28] = splat((candidate >> 26u) & 1u);
  s[29] = splat((candidate >> 27u) & 1u);
  s[30] = splat((candidate >> 28u) & 1u);
  s[31] = splat((candidate >> 29u) & 1u);
  s[32] = splat((candidate >> 30u) & 1u);
  s[33] = splat((candidate >> 31u) & 1u);
  s[34] = splat((cand_hi >> 0u) & 1u);
  s[35] = splat((cand_hi >> 1u) & 1u);
  s[36] = splat((cand_hi >> 2u) & 1u);
  s[37] = splat((cand_hi >> 3u) & 1u);
  s[38] = splat((cand_hi >> 4u) & 1u);
  s[39] = splat((cand_hi >> 5u) & 1u);
  s[40] = splat((cand_hi >> 6u) & 1u);
  s[41] = splat((cand_hi >> 7u) & 1u);
  s[42] = splat((cand_hi >> 8u) & 1u);
  s[43] = splat((cand_hi >> 9u) & 1u);
  s[44] = splat((cand_hi >> 10u) & 1u);
  s[45] = splat((cand_hi >> 11u) & 1u);
  s[46] = splat((cand_hi >> 12u) & 1u);
  s[47] = splat((cand_hi >> 13u) & 1u);
    s[4] = lane_pattern(0u); s[7] = lane_pattern(1u); s[9] = lane_pattern(2u); s[13] = lane_pattern(3u); s[16] = lane_pattern(4u);
  s[18] = guess(i1, 0u); s[22] = guess(i1, 1u); s[24] = guess(i1, 2u); s[27] = guess(i1, 3u); s[30] = guess(i1, 4u);
  s[32] = guess(i1, 5u); s[35] = guess(i1, 6u); s[45] = guess(i1, 7u); s[47] = guess(i1, 8u); s[48] = guess(i1, 9u);

  let results1 = f_c(g0(&s,1u), g1(&s,1u), g2(&s,1u), g3(&s,1u), g4(&s,1u)) ^ ks[1];
  if (results1 == 0u) { return; }
  // hoisted sub-filters (candidate/lane/i1 only)
  let filter2_0 = g0(&s,2u); let filter2_3 = g3(&s,2u);
  let filter3_0 = g0(&s,3u); let filter5_2 = g2(&s,5u); let filter6_2 = g2(&s,6u); let filter7_2 = g2(&s,7u);
  let filter9_1 = g1(&s,9u); let filter9_2 = g2(&s,9u); let filter10_0 = g0(&s,10u); let filter11_0 = g0(&s,11u);
  let filter12_0 = g0(&s,12u); let filter14_1 = g1(&s,14u); let filter15_1 = g1(&s,15u); let filter15_3 = g3(&s,15u); let filter16_1 = g1(&s,16u);

  for (var i2 = 0u; i2 < 32u; i2 = i2 + 1u) {
    s[10] = guess(i2,0u); s[19] = guess(i2,1u); s[25] = guess(i2,2u); s[36] = guess(i2,3u); s[49] = guess(i2,4u);
    let filter2 = f_c(filter2_0, g1(&s,2u), g2(&s,2u), filter2_3, g4(&s,2u));
    let results2 = results1 & (filter2 ^ ks[2]);
    if (results2 == 0u) { continue; }
    s[50] = lfsr_bs(&s,2u);
    let filter3_3 = g3(&s,3u); let filter4_0 = g0(&s,4u); let filter4_1 = g1(&s,4u); let filter4_2 = g2(&s,4u);
    let filter7_0 = g0(&s,7u); let filter7_1 = g1(&s,7u); let filter8_2 = g2(&s,8u); let filter10_1 = g1(&s,10u);
    let filter10_2 = g2(&s,10u); let filter11_1 = g1(&s,11u); let filter13_0 = g0(&s,13u); let filter13_1 = g1(&s,13u);
    let filter16_0 = g0(&s,16u); let filter16_3 = g3(&s,16u); let filter17_1 = g1(&s,17u); let filter17_3 = g3(&s,17u);
    for (var i3 = 0u; i3 < 8u; i3 = i3 + 1u) {
      s[11] = guess(i3,0u); s[20] = guess(i3,1u); s[37] = guess(i3,2u);
      let filter3 = f_c(filter3_0, g1(&s,3u), g2(&s,3u), filter3_3, g4(&s,3u));
      let results3 = results2 & (filter3 ^ ks[3]);
      if (results3 == 0u) { continue; }
      s[51] = lfsr_bs(&s,3u); s[52] = lfsr_bs(&s,4u); s[53] = lfsr_bs(&s,5u); s[54] = lfsr_bs(&s,6u); s[55] = lfsr_bs(&s,7u);
      let filter4_3 = g3(&s,4u); let filter5_0 = g0(&s,5u); let filter5_1 = g1(&s,5u); let filter6_0 = g0(&s,6u);
      let filter6_1 = g1(&s,6u); let filter8_0 = g0(&s,8u); let filter8_1 = g1(&s,8u); let filter9_0 = g0(&s,9u);
      let filter9_4 = g4(&s,9u); let filter11_2 = g2(&s,11u); let filter12_1 = g1(&s,12u); let filter14_0 = g0(&s,14u);
      let filter15_0 = g0(&s,15u); let filter17_0 = g0(&s,17u);
      for (var i4 = 0u; i4 < 2u; i4 = i4 + 1u) {
        s[38] = guess(i4,0u);
        let filter4 = f_c(filter4_0, filter4_1, filter4_2, filter4_3, g4(&s,4u));
        let results4 = results3 & (filter4 ^ ks[4]);
        if (results4 == 0u) { continue; }
        s[56] = lfsr_bs(&s,8u);
        let filter5_3 = g3(&s,5u); let filter10_4 = g4(&s,10u); let filter12_2 = g2(&s,12u);
        for (var i5 = 0u; i5 < 2u; i5 = i5 + 1u) {
          s[39] = guess(i5,0u);
          let filter5 = f_c(filter5_0, filter5_1, filter5_2, filter5_3, g4(&s,5u));
          let results5 = results4 & (filter5 ^ ks[5]);
          if (results5 == 0u) { continue; }
          s[57] = lfsr_bs(&s,9u);
          let filter6_3 = g3(&s,6u); let filter11_4 = g4(&s,11u); let filter13_2 = g2(&s,13u);
          for (var i6 = 0u; i6 < 2u; i6 = i6 + 1u) {
            s[40] = guess(i6,0u);
            let filter6 = f_c(filter6_0, filter6_1, filter6_2, filter6_3, g4(&s,6u));
            let results6 = results5 & (filter6 ^ ks[6]);
            if (results6 == 0u) { continue; }
            s[58] = lfsr_bs(&s,10u);
            let filter7_3 = g3(&s,7u); let filter12_4 = g4(&s,12u); let filter14_2 = g2(&s,14u); let filter17_2 = g2(&s,17u);
            for (var i7 = 0u; i7 < 2u; i7 = i7 + 1u) {
              s[41] = guess(i7,0u);
              let filter7 = f_c(filter7_0, filter7_1, filter7_2, filter7_3, g4(&s,7u));
              let results7 = results6 & (filter7 ^ ks[7]);
              if (results7 == 0u) { continue; }
              s[59] = lfsr_bs(&s,11u);
              let filter8_3 = g3(&s,8u);
              let filter10 = f_c(filter10_0, filter10_1, filter10_2, g3(&s,10u), filter10_4);
              let filter12 = f_c(filter12_0, filter12_1, filter12_2, g3(&s,12u), filter12_4);
              let filter13_4 = g4(&s,13u); let filter15_2 = g2(&s,15u);
              for (var i8 = 0u; i8 < 2u; i8 = i8 + 1u) {
                s[42] = guess(i8,0u);
                let filter8 = f_c(filter8_0, filter8_1, filter8_2, filter8_3, g4(&s,8u));
                var r = results7 & (filter8 ^ ks[8]);
                if (r == 0u) { continue; }
                let filter9 = f_c(filter9_0, filter9_1, filter9_2, g3(&s,9u), filter9_4);
                r = r & (filter9 ^ ks[9]); if (r == 0u) { continue; }
                r = r & (filter10 ^ ks[10]); if (r == 0u) { continue; }
                let filter11 = f_c(filter11_0, filter11_1, filter11_2, g3(&s,11u), filter11_4);
                r = r & (filter11 ^ ks[11]); if (r == 0u) { continue; }
                r = r & (filter12 ^ ks[12]); if (r == 0u) { continue; }
                let filter13 = f_c(filter13_0, filter13_1, filter13_2, g3(&s,13u), filter13_4);
                r = r & (filter13 ^ ks[13]); if (r == 0u) { continue; }
                s[60] = lfsr_bs(&s,12u);
                let filter14 = f_c(filter14_0, filter14_1, filter14_2, g3(&s,14u), g4(&s,14u));
                r = r & (filter14 ^ ks[14]); if (r == 0u) { continue; }
                s[61] = lfsr_bs(&s,13u);
                let filter15 = f_c(filter15_0, filter15_1, filter15_2, filter15_3, g4(&s,15u));
                r = r & (filter15 ^ ks[15]); if (r == 0u) { continue; }
                s[62] = lfsr_bs(&s,14u);
                let filter16 = f_c(filter16_0, filter16_1, g2(&s,16u), filter16_3, g4(&s,16u));
                r = r & (filter16 ^ ks[16]); if (r == 0u) { continue; }
                s[63] = lfsr_bs(&s,15u);
                let filter17 = f_c(filter17_0, filter17_1, filter17_2, filter17_3, g4(&s,17u));
                r = r & (filter17 ^ ks[17]); if (r == 0u) { continue; }
                var ok = true;
                s[64] = lfsr_bs(&s, 16u);
                if (ok) { r = r & (f_c(g0(&s,18u), g1(&s,18u), g2(&s,18u), g3(&s,18u), g4(&s,18u)) ^ ks[18]); ok = r != 0u; }
                s[65] = lfsr_bs(&s, 17u);
                if (ok) { r = r & (f_c(g0(&s,19u), g1(&s,19u), g2(&s,19u), g3(&s,19u), g4(&s,19u)) ^ ks[19]); ok = r != 0u; }
                s[66] = lfsr_bs(&s, 18u);
                if (ok) { r = r & (f_c(g0(&s,20u), g1(&s,20u), g2(&s,20u), g3(&s,20u), g4(&s,20u)) ^ ks[20]); ok = r != 0u; }
                s[67] = lfsr_bs(&s, 19u);
                if (ok) { r = r & (f_c(g0(&s,21u), g1(&s,21u), g2(&s,21u), g3(&s,21u), g4(&s,21u)) ^ ks[21]); ok = r != 0u; }
                s[68] = lfsr_bs(&s, 20u);
                if (ok) { r = r & (f_c(g0(&s,22u), g1(&s,22u), g2(&s,22u), g3(&s,22u), g4(&s,22u)) ^ ks[22]); ok = r != 0u; }
                s[69] = lfsr_bs(&s, 21u);
                if (ok) { r = r & (f_c(g0(&s,23u), g1(&s,23u), g2(&s,23u), g3(&s,23u), g4(&s,23u)) ^ ks[23]); ok = r != 0u; }
                s[70] = lfsr_bs(&s, 22u);
                if (ok) { r = r & (f_c(g0(&s,24u), g1(&s,24u), g2(&s,24u), g3(&s,24u), g4(&s,24u)) ^ ks[24]); ok = r != 0u; }
                s[71] = lfsr_bs(&s, 23u);
                if (ok) { r = r & (f_c(g0(&s,25u), g1(&s,25u), g2(&s,25u), g3(&s,25u), g4(&s,25u)) ^ ks[25]); ok = r != 0u; }
                s[72] = lfsr_bs(&s, 24u);
                if (ok) { r = r & (f_c(g0(&s,26u), g1(&s,26u), g2(&s,26u), g3(&s,26u), g4(&s,26u)) ^ ks[26]); ok = r != 0u; }
                s[73] = lfsr_bs(&s, 25u);
                if (ok) { r = r & (f_c(g0(&s,27u), g1(&s,27u), g2(&s,27u), g3(&s,27u), g4(&s,27u)) ^ ks[27]); ok = r != 0u; }
                s[74] = lfsr_bs(&s, 26u);
                if (ok) { r = r & (f_c(g0(&s,28u), g1(&s,28u), g2(&s,28u), g3(&s,28u), g4(&s,28u)) ^ ks[28]); ok = r != 0u; }
                s[75] = lfsr_bs(&s, 27u);
                if (ok) { r = r & (f_c(g0(&s,29u), g1(&s,29u), g2(&s,29u), g3(&s,29u), g4(&s,29u)) ^ ks[29]); ok = r != 0u; }
                s[76] = lfsr_bs(&s, 28u);
                if (ok) { r = r & (f_c(g0(&s,30u), g1(&s,30u), g2(&s,30u), g3(&s,30u), g4(&s,30u)) ^ ks[30]); ok = r != 0u; }
                s[77] = lfsr_bs(&s, 29u);
                if (ok) { r = r & (f_c(g0(&s,31u), g1(&s,31u), g2(&s,31u), g3(&s,31u), g4(&s,31u)) ^ ks[31]); ok = r != 0u; }
                if (ok) {
                for (var lane = 0u; lane < 32u; lane = lane + 1u) {
                  if (((r >> lane) & 1u) == 1u) {
                    var lo = 0u; var hi = 0u;
                    lo = lo | (((s[2] >> lane) & 1u) << 0u);
                    lo = lo | (((s[3] >> lane) & 1u) << 1u);
                    lo = lo | (((s[4] >> lane) & 1u) << 2u);
                    lo = lo | (((s[5] >> lane) & 1u) << 3u);
                    lo = lo | (((s[6] >> lane) & 1u) << 4u);
                    lo = lo | (((s[7] >> lane) & 1u) << 5u);
                    lo = lo | (((s[8] >> lane) & 1u) << 6u);
                    lo = lo | (((s[9] >> lane) & 1u) << 7u);
                    lo = lo | (((s[10] >> lane) & 1u) << 8u);
                    lo = lo | (((s[11] >> lane) & 1u) << 9u);
                    lo = lo | (((s[12] >> lane) & 1u) << 10u);
                    lo = lo | (((s[13] >> lane) & 1u) << 11u);
                    lo = lo | (((s[14] >> lane) & 1u) << 12u);
                    lo = lo | (((s[15] >> lane) & 1u) << 13u);
                    lo = lo | (((s[16] >> lane) & 1u) << 14u);
                    lo = lo | (((s[17] >> lane) & 1u) << 15u);
                    lo = lo | (((s[18] >> lane) & 1u) << 16u);
                    lo = lo | (((s[19] >> lane) & 1u) << 17u);
                    lo = lo | (((s[20] >> lane) & 1u) << 18u);
                    lo = lo | (((s[21] >> lane) & 1u) << 19u);
                    lo = lo | (((s[22] >> lane) & 1u) << 20u);
                    lo = lo | (((s[23] >> lane) & 1u) << 21u);
                    lo = lo | (((s[24] >> lane) & 1u) << 22u);
                    lo = lo | (((s[25] >> lane) & 1u) << 23u);
                    lo = lo | (((s[26] >> lane) & 1u) << 24u);
                    lo = lo | (((s[27] >> lane) & 1u) << 25u);
                    lo = lo | (((s[28] >> lane) & 1u) << 26u);
                    lo = lo | (((s[29] >> lane) & 1u) << 27u);
                    lo = lo | (((s[30] >> lane) & 1u) << 28u);
                    lo = lo | (((s[31] >> lane) & 1u) << 29u);
                    lo = lo | (((s[32] >> lane) & 1u) << 30u);
                    lo = lo | (((s[33] >> lane) & 1u) << 31u);
                    hi = hi | (((s[34] >> lane) & 1u) << 0u);
                    hi = hi | (((s[35] >> lane) & 1u) << 1u);
                    hi = hi | (((s[36] >> lane) & 1u) << 2u);
                    hi = hi | (((s[37] >> lane) & 1u) << 3u);
                    hi = hi | (((s[38] >> lane) & 1u) << 4u);
                    hi = hi | (((s[39] >> lane) & 1u) << 5u);
                    hi = hi | (((s[40] >> lane) & 1u) << 6u);
                    hi = hi | (((s[41] >> lane) & 1u) << 7u);
                    hi = hi | (((s[42] >> lane) & 1u) << 8u);
                    hi = hi | (((s[43] >> lane) & 1u) << 9u);
                    hi = hi | (((s[44] >> lane) & 1u) << 10u);
                    hi = hi | (((s[45] >> lane) & 1u) << 11u);
                    hi = hi | (((s[46] >> lane) & 1u) << 12u);
                    hi = hi | (((s[47] >> lane) & 1u) << 13u);
                    hi = hi | (((s[48] >> lane) & 1u) << 14u);
                    hi = hi | (((s[49] >> lane) & 1u) << 15u);
                    try_emit(vec2(lo, hi), p.base_idx + cand_index);
                  }
                }
                }
              }
            }
          }
        }
      }
    }
  }
}
