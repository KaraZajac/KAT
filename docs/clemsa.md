---
layout: default
---

# Clemsa Protocol

**Rust module:** `src/protocols/clemsa.rs`
**Reference:** Flipper-ARF `applications/system/subghz_garage/protocols/clemsa.c` (garage app, RX group g8 +
`tx_clemsa`); doctor-firmware decodes and encodes identically; Flipper-ARF `lib/subghz/protocols/clemsa.c` adds
button switching.

## Overview

Clemsa gate remotes: an 18-bit fixed code — eight three-state DIP switches (shown by doctor-firmware) and a 2-bit
button. No checksum.

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| Short | 385 µs | ±150 µs |
| Long | 2695 µs | ±450 µs |
| Bit `1` | HIGH 2695 + LOW 385 | |
| Bit `0` | HIGH 385 + LOW 2695 | |
| Gap | LOW 19 635 µs ±3750 | `51·te_short` ± `25·te_delta`; the encoder extends the last LOW by `7·te_long` |
| Bits | 18 | exactly |

## Frame Layout (18 bits, MSB first)

- **serial** = `(data >> 2) & 0xFFFF` (the DIP switches)
- **button** = `data & 3`

## RF

- **Encoding:** PWM
- **RF modulation:** AM, `SubGhzProtocolFlag_433 | AM`; AM650
- **Encryption:** none (fixed code)

## Decoder Steps

1. **Reset** — wait for a gap LOW.
2. **SaveDuration / CheckDuration** — HIGH + LOW pairs → bits; on a gap the last bit is classified by its HIGH and the
   frame reported when exactly 18 bits were read. The C sets the next step (SaveDuration) *after* its callback — where
   the Flipper receiver resets every decoder — so decoding continues with the next frame; KAT's `reset` right after a
   report resumes there too (the oracle comparison caught this).

## Encoder

`repeat` = 3, no counter: the 18 bits, the last LOW extended by `7·te_long`. Button: the garage copy re-sends the
captured key; ARF's lib copy lets the D-pad pick 1, 2, 3 (OK = captured) and rewrites the low 2 bits — KAT takes the
union and accepts any 2-bit native code (others keep the captured button).

## Key file

`Protocol: Clemsa`, `Bit: 18`, `Key` = the 18-bit code. Loading requires `Bit: 18`.

## Validation

- Flipper unit test `clemsa_raw.sub` decodes (36 frames, `2FCAA`, Sn BF2A, Btn 2); `clemsa.sub` loads and encodes.
- C oracle (doctor + garage builds): decodes (key, Sn, Btn) on the unit-test corpus, IMPORTS, UberGuidoZ and 4224 fuzz
  files equal the oracle's; encoder uploads pulse-identical for 7 key files.
