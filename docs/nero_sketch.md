---
layout: default
---

# Nero Sketch Protocol

**Rust module:** `src/protocols/nero_sketch.rs`
**Reference:** doctor-firmware `lib/subghz/protocols/nero_sketch.c` (not in Flipper-ARF's `subghz_garage` RX groups;
ARF's lib copy and the garage copy decode and encode identically).

## Overview

Nero Sketch remotes: a 40-bit fixed code behind a long preamble. No fields (the C shows the key and its
bit-reversal, "Yek"), no checksum.

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| Short | 330 µs | ±150 µs |
| Long | 660 µs | ±150 µs |
| Preamble | > 40 × (HIGH 330 + LOW 330) | the encoder sends 47 |
| Start | HIGH 1320 + LOW 330 | `4·te_short` |
| Bit `1` | HIGH 660 + LOW 330 | |
| Bit `0` | HIGH 330 + LOW 660 | |
| Stop | HIGH ≥ 960 µs | encoder: HIGH 990 + LOW 330 |
| Bits | 40 | exactly |

## RF

- **Encoding:** PWM
- **RF modulation:** AM, `SubGhzProtocolFlag_433 | AM`; AM650
- **Encryption:** none (fixed code)

## Decoder Steps

1. **Reset** — a short HIGH starts the preamble count.
2. **CheckPreambula** — short HIGH/LOW pairs count the preamble; the 1320 µs HIGH + short LOW is the start, accepted
   after more than 40 pairs.
3. **SaveDuration / CheckDuration** — HIGH + LOW pairs → bits; a HIGH ≥ 960 µs is the stop pulse: reported when exactly
   40 bits were read.

## Encoder

`repeat` = 3, no counter, `button` ignored: preamble (47 pairs), start, 40 bits, stop.

## Key file

`Protocol: Nero Sketch`, `Bit: 40`, `Key` = the 40-bit code. Loading requires `Bit: 40`. (The Flipper unit tests have
no Nero Sketch key file.)

## Validation

- Flipper unit test `nero_sketch_raw.sub` decodes (30 frames, `8511138511`).
- C oracle (doctor build): decodes on the unit-test corpus, IMPORTS, UberGuidoZ and 4224 fuzz files (two seeds) equal the
  oracle's; encoder uploads pulse-identical for 6 random key files.
