---
layout: default
---

# Elplast Protocol

**Rust module:** `src/protocols/elplast.rs`
**Reference:** doctor-firmware `lib/subghz/protocols/elplast.c` (@xMasterX 2025; not in Flipper-ARF's `subghz_garage`
RX groups; ARF's lib copy and the garage copy decode and encode identically). Unit-test files: Unleashed's
`elplast_raw.sub` / `elplast.sub`.

## Overview

Elplast / P-11B / 3BK / E.C.A. remotes: an 18-bit fixed code. No fields (the C shows the key and its bit-reversal,
"Yek"), no checksum.

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| Short | 230 µs | ±160 µs |
| Long | 1550 µs | ±160 µs |
| Bit `1` | HIGH 1550 + LOW 230 | |
| Bit `0` | HIGH 230 + LOW 1550 | |
| Gap | LOW 12 400 µs ±2080 | `8·te_long` ± `13·te_delta` (the last bit's LOW) |
| Bits | 18 | exactly |

## RF

- **Encoding:** PWM
- **RF modulation:** AM, `SubGhzProtocolFlag_433 | AM`; AM650
- **Encryption:** none (fixed code)

## Decoder Steps

1. **Reset** — wait for a gap LOW.
2. **SaveDuration / CheckDuration** — HIGH + LOW pairs → bits; on the next gap the last bit is classified by its HIGH
   and the frame reported when exactly 18 bits were read; the decoder then looks for a gap again, so of back-to-back
   frames every other one is reported.

## Encoder

`repeat` = 3, no counter, `button` ignored: the 18 bits, the last LOW = 12 400 µs.

## Key file

`Protocol: Elplast`, `Bit: 18`, `Key` = the 18-bit code. Loading requires `Bit: 18`.

## Validation

- Unleashed unit test `elplast_raw.sub` decodes (67 frames, `32C82`; the module test runs it when `KAT_ELPLAST_RAW`
  points at the file); `elplast.sub` loads and encodes.
- C oracle (doctor build): decodes on the unit-test corpus, IMPORTS, UberGuidoZ and 4224 fuzz files (two seeds) equal the oracle's;
  encoder uploads pulse-identical for 7 key files.
