---
layout: default
---

# Doitrand Protocol

**Rust module:** `src/protocols/doitrand.rs`
**Reference:** Flipper-ARF `applications/system/subghz_garage/protocols/doitrand.c` (garage app, RX group g7 +
`tx_doitrand`); doctor-firmware decodes and encodes identically; Flipper-ARF `lib/subghz/protocols/doitrand.c` adds
button switching.

## Overview

Doitrand gate remotes: a 37-bit fixed code — 10 DIP switches and a 2-bit key (button). No checksum.

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| Short | 400 µs | ±150 µs |
| Long | 1100 µs | ±450 µs (`3·te_delta`) |
| Header | LOW 24 800 µs | `62·te_short`, accepted ±4500 µs |
| Start pulse | HIGH 700 µs | encoder `2·te_short − 100`; decoder 800 ±450 |
| Bit `1` | LOW 1100 + HIGH 400 | |
| Bit `0` | LOW 400 + HIGH 1100 | |
| Frame end | LOW ≥ 4150 µs | `10·te_short + te_delta` |
| Bits | 37 | exactly |

## Frame Layout (37 bits, MSB first)

- **button** = `(data >> 18) & 3` (KAT `button`)
- **DIP** = `(data >> 24) | ((data >> 15) & 1)` — stored by the C in `generic.cnt` and shown as `DIP:` in the order
  0x001, 0x100, 0x080, 0x040, 0x020, 0x1000, 0x800, 0x400, 0x200, 0x002; KAT reports it in `counter` like the C.

## RF

- **Encoding:** PWM (gap first)
- **RF modulation:** AM, `SubGhzProtocolFlag_433 | AM`; AM650
- **Encryption:** none (fixed code)

## Decoder Steps

1. **Reset** — wait for the header LOW. 2. **FoundStartBit** — the start HIGH (else Reset).
3. **SaveDuration / CheckDuration** — LOW + HIGH pairs → bits; a LOW ≥ 4150 µs reports the frame when exactly 37 bits
   were read and expects the next start pulse (with the registry's reset after a report, the next frame is skipped).

## Encoder

`repeat` = 3, no counter: header, start pulse, 37 bits. Button: the garage copy re-sends the captured key; ARF's lib
copy lets the D-pad pick 1, 2, 3 (OK = captured) and rewrites bits 18..19 — KAT takes the union and accepts any
2-bit native code (others keep the captured button).

## Key file

`Protocol: Doitrand`, `Bit: 37`, `Key` = the 37-bit code. Loading requires `Bit: 37`.

## Validation

- Flipper unit test `doitrand_raw.sub` decodes (11 frames, `1E60082F5F`, Btn 2); `doitrand.sub` loads and encodes.
- C oracle (doctor + garage builds): decodes (key, Btn, Cnt) on the unit-test corpus, IMPORTS, UberGuidoZ and 4224 fuzz
  files equal the oracle's; encoder uploads pulse-identical for 7 key files.
