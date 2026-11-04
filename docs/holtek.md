---
layout: default
---

# Holtek Protocol

**Rust module:** `src/protocols/holtek.rs`
**Reference:** Flipper-ARF garage app `applications/system/subghz_garage/protocols/holtek.c` (primary;
doctor-firmware decodes and encodes the same); Flipper-ARF `lib/subghz/protocols/holtek.c` (button switch)

## Overview

Holtek HT640-family 40-bit fixed codes (e.g. remote-controlled sockets and gate receivers): a 0x5 header
nibble, a 20-bit address and four button nibbles (0xA = not pressed; 0xE = ON, 0xB = OFF).

## Timing

| Parameter  | Value              | Notes                     |
|------------|--------------------|---------------------------|
| te_short   | 430 µs             | ±100 µs                   |
| te_long    | 870 µs             | ±200 µs                   |
| Header     | 15480 ± 3600 µs    | encoder: 36·te            |
| Start bit  | HIGH 430 ± 100 µs  |                           |
| End LOW    | ≥ 4400 µs          |                           |

## Frame Layout (40 bits)

MSB first: bit 1 = LOW 870 + HIGH 430, bit 0 = LOW 430 + HIGH 870. Bits 36..39 = 0x5 (required); **serial**
= bits 16..35 bit-mirrored (20 bits); **button** = position << 4 | code of the first non-0xA nibble of
the low 16 bits from the low end (position 1..4; 0 when none). Labels: "B1 ON", "B2 OFF", "B3" …

## RF

- **Encoding:** PWM
- **RF modulation:** AM (OOK); 315 / 433.92 / 868 MHz
- **Encryption:** none (fixed code)

## Decoder Steps

1. **Reset** — header LOW → FoundStartBit.
2. **FoundStartBit** — HIGH 430 ± 100 µs starts the bits, anything else → Reset.
3. **SaveDuration / CheckDuration** — LOW/HIGH pairs; a LOW ≥ 4400 µs ends the frame, reported at exactly
   40 bits with the 0x5 nibble. The C then waits for the next start bit (it sets `FoundStartBit` after the
   callback, overriding the app's reset), so every repeat decodes; KAT keeps that state through the
   registry's reset.

## Encoder

`subghz_protocol_encoder_holtek_get_upload`, `encoder.repeat` = 3: header, start bit, 40 bits. The captured
button re-sends the key unchanged (the garage app); another native code (position 1..=4 << 4 | code ≠ 0xA)
rewrites the button word to 0xAAAA with that code at that position (Flipper-ARF lib moves the captured
code; KAT also lets the code change); other codes keep the key.

## Key file

`Protocol: Holtek`, `Bit: 40`, `Key`.

## Validation

Garage C oracle (`decode --protocol Holtek`): 8 identical decodes on the Flipper unit-test corpus, 388 on
3420 fuzz files; none on IMPORTS. The UberGuidoZ `Holtek-*` brute-force files are not this format (neither
the C nor KAT decodes them as Holtek). Encoder: 26 uploads equal both oracles' pulse for pulse.
