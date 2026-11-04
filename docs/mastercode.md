---
layout: default
---

# Mastercode Protocol

**Rust module:** `src/protocols/mastercode.rs`
**Reference:** Flipper-ARF `applications/system/subghz_garage/protocols/mastercode.c` (garage app, RX group g8 +
`tx_mastercode`); doctor-firmware and ARF's lib copy decode and encode identically.

## Overview

Clemsa MasterCode MV1 / MV12 remotes: a 36-bit fixed code — eight three-state DIP switches (shown by doctor-firmware)
and a 2-bit field the references treat as a fixed DIP code. No checksum.

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| Short | 1072 µs | ±150 µs |
| Long | 2145 µs | ±1200 µs (`8·te_delta`) |
| Bit `1` | HIGH 2145 + LOW 1072 | |
| Bit `0` | HIGH 1072 + LOW 2145 | |
| Gap | LOW 16 080 µs ±2250 | `15·te_short` ± `15·te_delta`; the encoder extends the last LOW by `13·te_short` |
| Bits | 36 | exactly |

## Frame Layout (36 bits, MSB first)

- **serial** = `(data >> 4) & 0xFFFF` (the DIP switches)
- **button** = `(data >> 2) & 3`

## RF

- **Encoding:** PWM
- **RF modulation:** AM, `SubGhzProtocolFlag_433 | AM`; AM650 (the unit-test files were recorded with AM270)
- **Encryption:** none (fixed code)

## Decoder Steps

As Clemsa: gap → HIGH + LOW pairs → on the next gap the last bit by its HIGH, report when exactly 36 bits were read; the
C sets the next step after its callback, so decoding continues with the next frame after a receiver reset (KAT's
`reset` right after a report resumes there).

## Encoder

`repeat` = 3, no counter: the 36 bits, the last LOW extended by `13·te_short`. The button is a fixed DIP code: the
references re-send the captured key (ARF's lib copy shows the D-pad but every direction sends the original), so
`encode` ignores `button`.

## Key file

`Protocol: Mastercode`, `Bit: 36`, `Key` = the 36-bit code. Loading requires `Bit: 36`.

## Validation

- Flipper unit test `mastercode_raw.sub` decodes (2 frames, `B7E003C04`, Btn 1); `mastercode.sub` loads and encodes.
- C oracle (doctor + garage builds): decodes (key, Sn, Btn) on the unit-test corpus, IMPORTS, UberGuidoZ and 4224 fuzz
  files equal the oracle's; encoder uploads pulse-identical for 7 key files.
