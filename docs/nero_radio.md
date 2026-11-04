---
layout: default
---

# Nero Radio Protocol

**Rust module:** `src/protocols/nero_radio.rs`
**Reference:** doctor-firmware `lib/subghz/protocols/nero_radio.c` (not in Flipper-ARF's `subghz_garage` RX groups;
ARF's lib copy and the garage copy decode identically; ARF's lib copy adds button switching).

## Overview

Nero Radio remotes: a 56-bit (or 57-bit) static frame behind a long preamble. The C's sample table reads it as
`serial(28) | button(4) | serial/const(16) | "crc??"(8)`; the low byte is shown as "CRC?" and never checked.

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| Short | 200 µs | ±80 µs |
| Long | 400 µs | ±80 µs |
| Preamble | > 40 × (HIGH 200 + LOW 200) | the encoder sends 49 |
| Start | HIGH 830 + LOW 200 | decoder: `4·te_short` ±80 |
| Bit `1` | HIGH 400 + LOW 200 | |
| Bit `0` | HIGH 200 + LOW 400 | |
| Stop | LOW ≥ 1250 µs | encoder: 4600 µs (1300 µs for 57-bit keys) |
| Bits | 56 or 57 | |

## Frame Layout (56 bits, MSB first)

- **button** = `(data >> 24) & 0xF` (remotes use 1–4)
- **serial** (KAT) = `data >> 28` (28 bits; 29 for 57-bit frames)
- The C's "Sn" (`generic.data_2`) = `serial << 16 | (data >> 8) & 0xFFFF`, 44 bits — too wide for KAT's 32-bit serial,
  but derivable from the reported fields.
- **"CRC?"** = low byte. Its algorithm is unknown — no CRC-8 (any polynomial, reflected or not, any init/xorout),
  rtl_433-style LFSR digest or byte sum/XOR model fits the C's six sample frames from two remotes — so KAT, like the C,
  reports every frame and `crc_valid` is false.

## RF

- **Encoding:** PWM
- **RF modulation:** AM, `SubGhzProtocolFlag_433 | AM`; AM650
- **Encryption:** none (fixed code)

## Decoder Steps

1. **Reset / CheckPreambula** — as Nero Sketch: > 40 short pairs, then the 830 µs start + short LOW.
2. **SaveDuration / CheckDuration** — HIGH + LOW pairs → bits; a LOW ≥ 1250 µs classifies the last bit by its HIGH and
   reports the frame when 56 or 57 bits were read.

## Encoder

`repeat` = 3, no counter: preamble (49 pairs), start, bits, stop LOW. Button: doctor-firmware re-sends the captured
key; Flipper-ARF's lib copy lets the D-pad pick buttons 1–4 (OK = captured) and rewrites bits 24..27 — without touching
the "CRC?" byte. KAT takes that union: any 4-bit native code (others keep the captured button), check byte unchanged as
in ARF. Whether a receiver accepts such a frame depends on the unknown check byte.

## Key file

`Protocol: Nero Radio`, `Bit: 56` or `57`, `Key` = the frame. Both bit counts load (the C's `deserialize` accepts 57 as
well). (The Flipper unit tests have no Nero Radio key file.)

## Validation

- Flipper unit test `nero_radio_raw.sub` decodes (16 frames: `57250501049DD3` button 1 and `57250502049D13` button 2).
- C oracle (doctor build): decodes (key, Btn, and the C's Sn/`Key_2` against `serial << 16 | …`) on the unit-test
  corpus, IMPORTS, UberGuidoZ and 4224 fuzz files (two seeds) equal the oracle's; encoder uploads pulse-identical for 9 key files
  (56 and 57 bits).
