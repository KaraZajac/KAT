---
layout: default
---

# Marantec24 Protocol

**Rust module:** `src/protocols/marantec24.rs`
**Reference:** Flipper-ARF `applications/system/subghz_garage/protocols/marantec24.c` (garage app, RX group g9 +
`tx_marantec24`); doctor-firmware decodes and encodes identically; Flipper-ARF `lib/subghz/protocols/marantec24.c`
adds button switching.

## Overview

A 24-bit fixed code (@xMasterX 2024/2025). Despite the name it is not a Marantec format: the C notes it comes from a
Chinese cloner remote that claims to be a Marantec replica. No checksum.

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| Short | 800 µs | ±200 µs |
| Long | 1600 µs | ±200 µs |
| Bit `1` | HIGH 800 + LOW 3200 | LOW `2·te_long` |
| Bit `0` | HIGH 1600 + LOW 2400 | LOW `3·te_short` |
| Gap | LOW 14 400 µs ±800 (exclusive) | decoder; the encoder sends 15 200 µs |
| Bits | 24 | exactly |

## Frame Layout (24 bits, MSB first)

- **serial** = `data >> 4` (20 bits)
- **button** = `data & 0xF` (known codes 0x8, 0x4, 0xC, 0x0 per Flipper-ARF)

## RF

- **Encoding:** PWM
- **RF modulation:** AM, `SubGhzProtocolFlag_868 | AM` (the unit-test capture is at 868.35 MHz); AM650
- **Encryption:** none (fixed code)

## Decoder Steps

1. **Reset** — wait for the gap LOW.
2. **SaveDuration / CheckDuration** — each HIGH + LOW pair is a bit; on the next gap the last bit is classified by its
   HIGH alone and the frame reported when exactly 24 bits were read. The decoder then looks for a gap again, so of
   back-to-back frames every other one is reported.

## Encoder

`repeat` = 3, no counter: the 24 bits, the last LOW extended to `9·te_long + te_short` = 15 200 µs. **C quirk kept:**
that gap is exactly 800 µs from the decoder's 14 400 µs target and the C compares with a strict `<`, so the reference
decoder rejects its own encoder's gap (the oracle's unit-test encoder check fails for this reason); a receiver with a
few µs of slack accepts it, and the real remote's gaps (15 146–15 180 µs in the unit-test capture) are just inside the
bound. Button: the garage copy re-sends the captured key; ARF's lib copy lets the D-pad pick 0x8/0x4/0xC/0x0 and
rebuilds `serial << 4 | btn` — KAT takes that union and accepts any 4-bit native code (others keep the captured one).

## Key file

`Protocol: Marantec24`, `Bit: 24`, `Key` = the 24-bit code. Loading requires `Bit: 24`.

## Validation

- Flipper unit test `marantec24_raw.sub` decodes (36 frames, `AC05C4`); `marantec24.sub` loads and encodes.
- C oracle (doctor build, decoder identical to the garage copy): decodes on the unit-test corpus, IMPORTS, UberGuidoZ
  and 4224 fuzz files (two seeds) equal the oracle's; encoder uploads pulse-identical for 7 key files.
