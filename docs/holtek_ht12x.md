---
layout: default
---

# Holtek_HT12X Protocol

**Rust module:** `src/protocols/holtek_ht12x.rs`
**Reference:** Flipper-ARF garage app `applications/system/subghz_garage/protocols/holtek_ht12x.c`
(primary; doctor-firmware decodes and encodes the same); Flipper-ARF `lib/subghz/protocols/holtek_ht12x.c`
(button switch)

## Overview

Holtek HT12E / HT12A 12-bit fixed codes: 8 address (DIP) bits and 4 active-low data bits (B1..B4). The
C's false-positive defence is a repeat check (`last_data`); it also estimates the time base (TE), which the
key file stores and the encoder reuses.

## Timing

| Parameter  | Value              | Notes                                   |
|------------|--------------------|-----------------------------------------|
| te_short   | 320 µs             | ±200 µs                                 |
| te_long    | 640 µs             | ±400 µs (2 × te_delta)                  |
| Header     | 8960 ± 4000 µs     | encoder: 36·TE                          |
| Start bit  | HIGH 320 ± 200 µs  |                                         |
| End LOW    | ≥ 3400 µs          |                                         |

## Frame Layout (12 bits)

MSB first: bit 1 = LOW 2·TE + HIGH TE, bit 0 = LOW TE + HIGH 2·TE. **button** = low nibble (a 0 bit =
pressed: 0x7 B1, 0xB B2, 0xD B3, 0xE B4; labels list every pressed button), **counter** = the 8-bit DIP
address the C keeps in `generic.cnt`, `extra` = TE. This is CAME's 12-bit air format: an HT12X frame
after a 9..13 ms header is also a valid CAME frame, which is why the registry (like the garage app) keeps
Holtek_HT12X before CAME.

## RF

- **Encoding:** PWM
- **RF modulation:** AM and FM (the references flag both); 315 / 433.92 / 868 MHz. TX presets: AM650,
  FM476 (no FM preset is named; KAT's default FSK preset).
- **Encryption:** none (fixed code)

## Decoder Steps

1. **Reset** — header LOW → FoundStartBit.
2. **FoundStartBit** — HIGH 320 ± 200 µs starts the bits (TE sum = this pulse).
3. **SaveDuration / CheckDuration** — LOW/HIGH pairs, all added to the TE sum; a LOW ≥ 3400 µs ends the
   frame: 12 bits equal to `last_data` (non-zero) are reported with TE = sum / 37; `last_data` takes the
   frame and the decoder waits for the next start bit (set after the callback). The C's reset does not
   clear `last_data` in this protocol, so after a decode by any protocol the next identical frame decodes
   at once (KAT's `reset()` keeps it too).

## Encoder

`subghz_protocol_encoder_holtek_th12x_get_upload`, `encoder.repeat` = 3: header LOW 36·TE, start HIGH TE,
12 bits, with the key's TE. `button` 0..=15 replaces the low nibble (ARF lib: B1 0x7, B2 0xB, B3 0xD,
B4 0xE); larger codes keep the captured nibble.

## Key file

`Protocol: Holtek_HT12X`, `Bit: 12`, `Key`, `TE: <µs>` (required on load).

## Validation

Garage C oracle (`decode --protocol Holtek_HT12X`): 37 identical decodes (key and TE) on the Flipper
unit-test corpus, 315 on 3420 fuzz files and 14664 on the UberGuidoZ sample (its `CAME-12bit-*` and
`CAME_brute_force` files are HT12X-decodable); none on IMPORTS. Encoder: 26 uploads (the unit test's
`holtek_ht12x.sub` with TE 205 + random keys/TEs) equal both oracles' pulse for pulse.
