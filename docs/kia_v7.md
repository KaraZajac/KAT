---
layout: default
---

# Kia V7 Protocol

**Rust module:** `src/protocols/kia_v7.rs`
**Reference:** `REFERENCES/ProtoPirate/protocols/kia_v7.c`

## Overview

Kia V7 uses Manchester encoding at 250/500 µs. 64 bits. The decoded 64-bit word is bit-inverted
(`~data`); a valid frame has a fixed header byte `0x4C` and a CRC8 (poly `0x7F`, init `0x4C`) over bytes
0–6. Emission is gated on header + CRC, so Kia V7 is strongly validated and will not false-match.

## Timing

| Parameter | Value  | Notes               |
|-----------|--------|---------------------|
| Short     | 250 µs | ±100 µs (te_delta)  |
| Long      | 500 µs | ±100 µs             |
| Min bits  | 64     |                     |
| Preamble  | ≥16 short pairs | before sync |

## Frame Layout (64 bits / 8 bytes, after inversion)

- **byte 0:** header `0x4C`
- **bytes 1–2:** counter (16-bit, big-endian)
- **bytes 3–6:** serial (28-bit) — `(b3<<20)|(b4<<12)|(b5<<4)|(b6>>4)`, masked to 0x0FFFFFFF
- **byte 6 low nibble:** button (4-bit) — Lock=1, Unlock=2, Trunk=3, aux=8
- **byte 7:** CRC8 over bytes 0–6 (poly `0x7F`, init `0x4C`)

## RF

- **Encoding:** Manchester (level ? (H,L) : (L,H); decoded word is bit-inverted)
- **RF modulation:** FM (per ProtoPirate `SubGhzProtocolFlag_FM`)
- **Encryption:** none — gated on fixed header `0x4C` + CRC8
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder Steps

1. **Reset** — a short HIGH pulse begins the preamble.
2. **Preamble** — count short pairs; once >15 pairs are seen, a long HIGH pulse transitions to SyncLow and preloads four seed bits (1,0,1,1 = the inverted header's top nibble 0xB).
3. **SyncLow** — a short LOW after the long sync enters Data.
4. **Data** — Manchester-decode the remaining 60 bits. At 64 bits, invert the word, check the header equals `0x4C` and the CRC8 validates, then emit.

## Encoder

Supported (matches `kia_v7_encode_key`). Rebuilds the 64-bit key from serial/button/counter with the
CRC8, then emits a 32-pair short preamble, a merged long sync pulse, 64 Manchester bits, and a trailing
gap.

## Validation

Verified by an encode→decode round-trip / synthetic-frame check (no local capture available).
