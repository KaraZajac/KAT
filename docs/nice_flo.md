---
layout: default
---

# Nice FLO Protocol

**Rust module:** `src/protocols/nice_flo.rs`
**Reference:** Flipper-ARF garage app `applications/system/subghz_garage/protocols/nice_flo.c` (primary;
ARF's lib copy decodes the same); doctor-firmware `lib/subghz/protocols/nice_flo.c`

## Overview

Nice FLO fixed-code gate remotes (12-bit DIP, 24-bit). doctor-firmware widens the timing tolerance
(`te_delta` 250 instead of 200 µs, header 25200 ± 7250 µs); KAT follows ARF (200, ± 7200).

## Timing

| Parameter  | Value              | Notes                     |
|------------|--------------------|---------------------------|
| te_short   | 700 µs             | ±200 µs                   |
| te_long    | 1400 µs            | ±200 µs                   |
| Header     | 25200 ± 7200 µs    | encoder: 36·te            |
| Start bit  | HIGH 700 µs        |                           |
| End LOW    | ≥ 2800 µs          |                           |

## Frame Layout

MSB first after the start bit: bit 1 = LOW 1400 + HIGH 700, bit 0 = LOW 700 + HIGH 1400. Reported with at
least 12 bits (no upper bound in the C: `decode_count_bit` is 8-bit and `data` keeps the last 64 bits, as
in KAT); no fields.

## RF

- **Encoding:** PWM
- **RF modulation:** AM (OOK); 315 / 433.92 MHz
- **Encryption:** none (fixed code)

## Decoder Steps

1. **Reset** — header LOW → FoundStartBit.
2. **FoundStartBit** — LOWs are skipped; HIGH 700 ± 200 µs starts the bits, any other HIGH → Reset.
3. **SaveDuration / CheckDuration** — LOW/HIGH pairs; a LOW ≥ 2800 µs ends the frame (≥ 12 bits reported).

## Encoder

`subghz_protocol_encoder_nice_flo_get_upload`, `encoder.repeat` = 3: header, start bit, key bits, for 12..=24
bit keys (the C's `deserialize` limit; longer decodes are not transmittable). Fixed code: `button` ignored.

## Key file

`Protocol: Nice FLO`, `Bit` (12..=24 to load), `Key`.

## Validation

Garage C oracle (`decode --protocol "Nice FLO"`): 7 identical decodes on the Flipper unit-test corpus, 300
on 3420 fuzz files, 1152 on the UberGuidoZ `NICE-12bit-*` sample; none on IMPORTS. On the first 720 fuzz
files the doctor build reports 80 instead of 70 (its wider tolerance). Encoder: 26 uploads (the unit test's
`nice_flo.sub` + 25 random 12/24-bit keys) equal both oracles' pulse for pulse.
