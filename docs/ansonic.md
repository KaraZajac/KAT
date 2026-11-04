---
layout: default
---

# Ansonic Protocol

**Rust module:** `src/protocols/ansonic.rs`
**Reference:** Flipper-ARF `applications/system/subghz_garage/protocols/ansonic.c` (garage app, RX group g8 +
`tx_ansonic`); doctor-firmware decodes and encodes identically (but flags it AM); Flipper-ARF
`lib/subghz/protocols/ansonic.c` adds button switching.

## Overview

Ansonic gate remotes: a 12-bit fixed code — 10 DIP switches and a 2-bit button. No checksum.

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| Short | 555 µs | ±120 µs |
| Long | 1111 µs | ±120 µs |
| Header | LOW 19 425 µs | `35·te_short`, accepted ±4200 µs |
| Start pulse | HIGH 555 µs | |
| Bit `1` | LOW 555 + HIGH 1111 | |
| Bit `0` | LOW 1111 + HIGH 555 | |
| Frame end | LOW ≥ 2220 µs | `4·te_short` |
| Bits | ≥ 12 | the C's `>=`; key files and the encoder need exactly 12 |

## Frame Layout (12 bits, MSB first)

- **button** = `(data >> 1) & 3`
- **DIP** = `data & 0xFFF`, stored by the C in `generic.cnt` (KAT `counter`) and shown as `DIP:` in the order
  0x800 … 0x010, 0x001, 0x008.

## RF

- **Encoding:** PWM (gap first)
- **RF modulation:** the references disagree — Flipper-ARF (garage app and lib) flag `SubGhzProtocolFlag_FM`,
  doctor-firmware `SubGhzProtocolFlag_AM`; the Flipper unit-test capture was recorded with OOK650, which the
  UberGuidoZ Ansonic brute-force files use too. KAT: AM/FM, AM650 first, FM476 as the FM preset (see `docs/transmit.md`).
- **Encryption:** none (fixed code); 315 / 433.92 MHz.

## Decoder Steps

1. **Reset** — wait for the header LOW. 2. **FoundStartBit** — LOWs ignored, a short HIGH starts the bits (else Reset).
3. **SaveDuration / CheckDuration** — LOW + HIGH pairs → bits; a LOW ≥ 2220 µs reports the frame when at least 12 bits
   were read (a longer frame is reported but cannot be loaded back by the C, so KAT marks it not encoder-capable).

## Encoder

`repeat` = 3, no counter: header, start pulse, 12 bits. Button: the garage copy re-sends the captured key; ARF's lib
copy lets the D-pad pick 1, 2, 3 (OK = captured) and rewrites bits 1..2 — KAT takes the union and accepts any 2-bit
native code (others keep the captured button).

## Key file

`Protocol: Ansonic`, `Bit: 12`, `Key` = the 12-bit code. Loading requires `Bit: 12`.

## Validation

- Flipper unit test `ansonic_raw.sub` decodes (12 frames, `AA3`, Btn 1); `ansonic.sub` loads and encodes.
- C oracle (doctor + garage builds): decodes (key, Btn, Cnt) on the unit-test corpus, IMPORTS, 1171 UberGuidoZ files
  (36 864 identical decodes of the Ansonic brute-force files) and 4224 fuzz files (two seeds) equal the oracle's; encoder uploads
  pulse-identical for 7 key files.
