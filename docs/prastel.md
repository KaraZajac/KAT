---
layout: default
---

# Prastel Protocol

**Rust module:** `src/protocols/prastel.rs`
**Reference:** `REFERENCES/Flipper-ARF/applications/system/subghz_garage/protocols/prastel.c` (garage app copy,
primary); encoder as registered by `REFERENCES/doctor-firmware/lib/subghz/protocols/prastel.c` (the garage app
compiles its copy of the encoder out). Flipper-ARF's `lib/subghz` has no prastel.c.

## Overview

Prastel (French gate automation) 42-bit rolling code. It uses the CAME line coding (320/640 µs, ~24 ms header)
but the payload is scrambled and carries a serial, a 1..4 button and a 16-bit counter, so it is its own
dynamic protocol (the 25-bit Prastel and 18-bit Airforce frames are static CAME variants).

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| te_short | 320 µs | ±150 µs (te_delta) |
| te_long | 640 µs | ±150 µs |
| Header (decoder) | 3 840 – 27 520 µs LOW | exclusive; doctor-firmware: 8 471 – 27 369 µs |
| Header (encoder) | 24 320 µs LOW | 76 · te_short |
| Start bit | 320 µs HIGH | |
| Frame end | LOW ≥ 1 280 µs | 4 · te_short |
| Bits | 42 | MSB first |

Each bit is a LOW then a HIGH: 1 = LOW 640 + HIGH 320, 0 = LOW 320 + HIGH 640.

## Frame Layout (42 bits)

The 42 bits are unpacked into 7 bytes `p[0..6]` of `data << 2` (MSB first) and unscrambled
(`subghz_protocol_prastel_unscramble`): three 40-bit right rotations over p[2..6], byte reversals, an inverted
nibble swap of p[2], and an LCG walk (`x = 21x + 1`) that turns p[5]/p[6] back into the counter.

- **serial** (garage copy): `(p1 & 0xF0) << 8 | p3 | (p4 << 12) & 0xF0000 | (p2 & 0x0F) << 8 | (p2 & 0x80) << 13`
  (doctor-firmware: `p0 << 24 | p1 << 8 | p3 | (p4 << 12) & 0xF0000`)
- **button**: low nibble of p4 — raw 0xE → 1, 0xD → 2, 0xB → 3, 0x7 → 4, any other nibble → 0
- **counter**: `p5 | p6 << 8` (16 bits)
- The scrambler also writes a parity bit over the counter bytes; no decoder checks it (`crc_valid` = false).

## RF

- **Encoding:** PWM (CAME timing)
- **RF modulation:** AM (OOK), 315 / 433.92 MHz in the references; KAT runs it on any frequency
- **Encryption:** rolling code (scrambled counter, no cipher key)

## Decoder Steps

1. **Reset** — a LOW strictly between 12·te_short and 86·te_short opens the frame (the garage copy widened the
   window so the first frame after the remote's ~4.8 ms preamble gap is not lost).
2. **FoundStartBit** — LOWs are skipped; a HIGH within te_delta of te_short starts the data, anything else resets.
3. **SaveDuration / CheckDuration** — LOW/HIGH pairs → 0 (short, long) or 1 (long, short); any other pair resets.
4. A LOW ≥ 4·te_short ends the frame: reported when exactly 42 bits were read; the decoder then waits for the next
   start bit (after the registry's reset it waits for the next header, like the Flipper app).

## Encoder

`subghz_protocol_encoder_prastel_deserialize`: counter + 1 (0xFFFF wraps to 0), the button's raw nibble written
into p4, re-scrambled and packed (`gen_data`), then LOW 24 320 µs, HIGH 320 µs and the 42 bit pairs, sent 3 times
(`repeat` = 3). `button` is KAT's native code (the decoded 1..4), as the union of the two references'
custom-button switches: the captured code keeps the captured nibble; from a recognised button (1..4) any other
1..4 is sent with the garage copy's `btn_to_raw`; doctor-firmware's switch sends 0xB (button 3) from any other
non-zero nibble and 0xC (button 0) from 0xB; anything else keeps the captured nibble.

## Key file

`Protocol: Prastel`, `Bit: 42`, `Key` (8 bytes; Flipper-ARF's generic serialize). The reference encoder writes the
bumped Key back to the file.

## Validation

- Oracle parity (compiled C, `decode --protocol Prastel`): the Flipper unit-test corpus, IMPORTS and the
  UberGuidoZ garage/gate set contain no Prastel frame; neither the C nor KAT reports one on any of them.
  Synthetic and fuzzed streams: see the G3 report (doctor + garage builds).
- Encoder: five hand-made 42-bit keys encode pulse-exact against doctor-firmware's encoder (258 pulses, counter +1,
  rescrambled Key equal to the C's), including doctor's custom-button switch (→ 0xB, 0xB → 0xC). The garage copy's
  1..4 switch is not built anywhere (no TX plugin); it is covered by unit tests of the ported `gen_data`.
- Unit tests: scramble/unscramble round trip over the whole counter range and every button nibble, button tables,
  synthetic frame decode (24 ms and 4.8 ms headers), encode → decode for every button path and the counter wrap,
  wrong length / timing / header rejection, key-file round trip.
