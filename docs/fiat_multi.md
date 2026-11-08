---
layout: default
---

# Fiat Multi Protocol (Fiat V3 / V5, not registered)

**Rust module:** `src/protocols/fiat_multi.rs`
**Reference:** `FoxFW lib/subghz/protocols/fiat_v2_v3_v5.c` (`"Fiat Multi"`)

## Status

Ported and tested, but **not in the registry**. The one frame it finds in the whole corpus is in
`VAG/Passat_2015.sub`, a VAG capture KAT already decodes completely: it reads a VAG burst as a V5 frame
(`Key 7FFFC07F0023C047`, a run of ones then zeros: noise), and it takes a second VAG frame of that file with it
(`kat_diff`: VAG 2 to 1, Fiat Multi 0 to 1). That is a false match, so it stays out. KAT's decode of that
frame is identical to FoxFW's. To enable it, add `Box::new(fiat_multi::FiatMultiDecoder::new())` to the
registry; it needs a real Fiat V3/V5 capture to show what gate it lacks.

## Overview

One FoxFW registration for two Fiat remotes, AM/OOK at 433.92 MHz, told apart by the first pulse. Not KAT's
[Fiat V0](fiat_v0.md), [Fiat V1](fiat_v1.md) or [Fiat V2](fiat_v2.md) (Hitag2). Decode-only; the C's encoder is
an SD plugin and is not ported.

| Variant | Timing        | Gap ending the frame | Frame |
|---------|---------------|----------------------|-------|
| V3      | 460/920 ±200 µs | > 1500 µs          | 70 bits: `fix:22 \| btn:2 \| status:2 \| hop:44` |
| V5      | 200/400 ±100 µs | > 2000 µs          | 97 bits: `fix:32 \| type:8 \| btn:8 \| seed:8 \| cnt:8 \| hop:32 \| trail:1` |

The first pulse picks the variant whose short time it is closest to (a pulse of 260 to 300 µs goes to V5).
Manchester bits use the table `01 91 9B FB` with `level ? ShortLow : ShortHigh`; a same-state transition gives
no bit. The bits after a run of leading ones (the preamble) form the frame; a frame is checked when a gap or a
bad pulse ends it with at least 60 (V3) or 90 (V5) bits.

## V3 gating

4 to 20 preamble ones; under 100 bits collected; at most 170 valid pulses; the average LOW pulse before the
first 0 bit at least 400 µs; for a full frame `hop[43:42] = 00` and `hop[0] = 1`; the XOR of hop bits
`{4,5,12,13,20,21,28,29,36,37}` must equal the frame button (0 or 1) and four invariant masks over the hop must
each XOR to 0. A reported V3 frame therefore shows CRC OK.

## V5 gating

At least 10 preamble ones, at least 90 bits, button `0x80` (Open), `0x40` (Trunk) or `0x20` (Close); for a
full frame the hop's lowest bit (frame bit 95) must be 1. No check: the CRC shows "-".

## Result

V3: `data` = `fix<<42 | btn<<40 | hop[43:4]`, 70 bits, `extra` = `status<<4 | hop[3:0]`, `serial` = fix,
`button`. V5: `data` = `fix<<32 | hop`, 97 bits, `extra` = `type<<8 | seed`, `serial` = fix, `button`,
`counter`. The variant is told by the bit count.

## RF

- **Encoding:** Manchester, **RF:** AM, **Encryption:** rolling
- **Frequencies:** 433.92 MHz

## Validation

Evidence: `Passat_2015.sub` (see Status): KAT's frame equals FoxFW's (97 bits, `Key 7FFFC07F0023C047`, Type 251,
Seed 31, Btn 128, Cnt 254). Made-up frames against the FoxFW C decoder (host oracle): two V3 frames (13 and 7
preamble ones) and two V5 frames (15 and 11) decode identically; a broken hop invariant, a wrong button parity,
`hop[43:42] != 00`, a missing stop bit and a 3-one preamble (V3), and button `0x10`, a missing stop bit, a
9-one preamble and an 89-bit frame (V5) are rejected by both. Those vectors are the unit tests. (A made-up V5
frame is also read by the older "Fiat V2" decoder first in FoxFW's registry, so the oracle was run with
`--protocol "Fiat Multi"`.) No key-file support.
