---
layout: default
---

# BETT Protocol

**Rust module:** `src/protocols/bett.rs`
**Reference:** doctor-firmware `lib/subghz/protocols/bett.c` (BETT is not in Flipper-ARF's `subghz_garage` RX groups;
ARF's lib copy and the garage copy decode and encode identically and only drop the DIP display).

## Overview

BETT = Berner / Elka / Tedsen / Teletaster garage and gate remotes: an 18-bit fixed code set by nine three-state DIP
switches (2 bits each: `11` +, `10` 0, `00` −), which doctor-firmware displays. No checksum, no button field.

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| Short | 340 µs | ±150 µs |
| Long | 2000 µs | ±450 µs (`3·te_delta`) |
| Bit `1` | HIGH 2000 + LOW 340 | |
| Bit `0` | HIGH 340 + LOW 2000 | |
| Gap | LOW 14 960 µs ±2250 | `44·te_short` ± `15·te_delta`; the encoder extends the last LOW by `7·te_long` |
| Bits | 18 | exactly |

## Frame Layout

18 bits, MSB first; the C shows `data & 0x3FFFF` and the switch positions. KAT reports no serial/button/counter.

## RF

- **Encoding:** PWM (bit value in the HIGH width; a LOW only has to be short or long)
- **RF modulation:** AM, `SubGhzProtocolFlag_433 | AM`; AM650
- **Encryption:** none (fixed code)

## Decoder Steps

1. **Reset** — wait for a gap LOW, then expect a HIGH.
2. **CheckDuration** — a HIGH long → `1`, short → `0`.
3. **SaveDuration** — a LOW short/long → next bit; a gap → report when exactly 18 bits were read (else back to Reset).
   After a report the C stays in this step, where a HIGH is ignored, so without a receiver reset the next frame loses
   its first bit; the registry (like the Flipper receiver) resets every decoder after a report anyway.

## Encoder

`repeat` = 3, no counter, `button` ignored: the 18 bits, the last LOW extended by `7·te_long` (14 340 / 16 000 µs).

## Key file

`Protocol: BETT`, `Bit: 18`, `Key` = the 18-bit code. Loading requires `Bit: 18`.

## Validation

- Flipper unit test `bett_raw.sub` decodes (14 frames, `CCCC`); `bett.sub` loads and encodes.
- C oracle (doctor build): decodes on the unit-test corpus, IMPORTS, UberGuidoZ and 4224 fuzz files (two seeds) equal the
  oracle's; encoder uploads pulse-identical for 7 key files.
