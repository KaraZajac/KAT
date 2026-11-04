---
layout: default
---

# Hörmann HSM Protocol

**Rust module:** `src/protocols/hormann.rs`
**Reference:** Flipper-ARF `applications/system/subghz_garage/protocols/hormann.c` (garage app, RX group g9 +
`tx_hormann`); doctor-firmware decodes and encodes identically; Flipper-ARF `lib/subghz/protocols/hormann.c` adds
button switching.

## Overview

Hörmann HSM garage remotes (the older static-code HSM series, not BiSecur): a 44-bit frame announced by a long 12 ms
start pulse. Frames carry a fixed pattern `0xFF000000003` (top byte `0xFF`, low two bits `11`), which the decoder
requires; the four buttons are one-hot (0x1/0x2/0x4/0x8) in bits 8..11. No checksum.

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| Short | 500 µs | ±200 µs |
| Long | 1000 µs | ±200 µs |
| Start pulse | HIGH 12 000 µs ±4800 | `24·te_short` |
| Start LOW | 500 µs ±200 | |
| Bit `1` | HIGH 1000 + LOW 500 | |
| Bit `0` | HIGH 500 + LOW 1000 | |
| Frame end | HIGH ≥ 2500 µs **and** the pattern matches | the next start pulse |
| Bits | ≥ 44 | the C's `>=`; key files and the encoder need exactly 44 |

## Frame Layout (44 bits, MSB first)

`FF ‹28 bits› ‹button 4› ‹6 bits› 11` — **button** = `(data >> 8) & 0xF`. The C has no serial field.

## RF

- **Encoding:** PWM
- **RF modulation:** AM, `SubGhzProtocolFlag_433 | 868 | AM`; AM650
- **Encryption:** none (fixed code)

## Decoder Steps

1. **Reset** — the 12 ms start HIGH. 2. **FoundStartBit** — the 500 µs LOW (clears the register).
3. **SaveDuration / CheckDuration** — HIGH + LOW pairs → bits; a HIGH ≥ 2500 µs with the pattern matched reports the
   frame (≥ 44 bits) and expects the start LOW; with the pattern unmatched the long HIGH is taken as a data HIGH and the
   frame breaks.

## Encoder

`repeat` = 1, one upload of 1801 entries: 20 × (start HIGH 12 000, LOW 500, 44 bits) and a final start HIGH. No
counter. Button: the garage copy re-sends the captured key; ARF's lib copy lets the D-pad pick 0x1/0x2/0x4/0x8 (OK =
captured) and rewrites bits 8..11 — KAT takes the union and accepts any 4-bit native code (others keep the captured
button).

## Key file

`Protocol: Hormann HSM`, `Bit: 44`, `Key` = the frame. Loading requires `Bit: 44`. (The Flipper unit tests have no
Hormann HSM key file.)

## Validation

- Flipper unit test `hormann_hsm_raw.sub` (comma-separated RAW_Data) decodes (14 frames, `FF2947CAF13`, button 0xF).
- C oracle (doctor + garage builds): decodes (key, Btn) on the unit-test corpus, IMPORTS, UberGuidoZ and 4224 fuzz files (two seeds)
  equal the oracle's; encoder uploads pulse-identical for 6 key files (including keys without the pattern).
