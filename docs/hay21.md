---
layout: default
---

# Hay21 Protocol

**Rust module:** `src/protocols/hay21.rs`
**Reference:** `REFERENCES/Flipper-ARF/applications/system/subghz_garage/protocols/hay21.c` (garage app copy,
primary); doctor-firmware and Flipper-ARF `lib/subghz/protocols/hay21.c` decode and transmit identically.

## Overview

21-bit remote with three buttons (On/Off, Mode, Hold), an 8-bit serial and a 4-bit rolling counter.

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| te_short | 300 µs | ±150 µs (te_delta) |
| te_long | 700 µs | ±150 µs |
| Gap | 4 200 µs LOW | 6·te_long; ±450 µs to open a frame, ±300 µs to close one |
| Bits | 21 | MSB first, no preamble |

Each bit is a HIGH then a LOW: 1 = HIGH 700 + LOW 300, 0 = HIGH 300 + LOW 700; the last bit's LOW is the gap.

## Frame Layout (21 bits)

- **bits 13..20** button: 0x5A On/Off, 0xC3 Mode, 0x88 Hold
- **bits 5..12** serial
- **bits 1..4** counter
- **bit 0** set by the encoder

## RF

- **Encoding:** PWM
- **RF modulation:** AM (OOK), 433.92 MHz
- **Encryption:** rolling code (4-bit counter, no cipher)

## Decoder Steps

1. **Reset** — a LOW within 450 µs of the gap clears the register.
2. **SaveDuration** — HIGH → remembered; LOW → Reset.
3. **CheckDuration** — LOW: (long, short) → 1, (short, long) → 0; a gap-length LOW reads the last bit from its HIGH
   and reports the frame when exactly 21 bits were read, then returns to **Reset** — the gap that closed a frame
   does not open the next one, so a held button is reported every other frame (as in the C).

## Encoder

`subghz_protocol_encoder_hay21_get_upload`: counter + 1 (above 0xF wraps to 0), key =
`button << 13 | serial << 5 | counter << 1 | 1`, 21 bit pairs with the gap after the last, sent 3 times. `button`
(native) is used when both it and the captured button are one of 0x5A / 0xC3 / 0x88 (the references' custom-button
switch), otherwise the captured button is sent. (The garage app's TX plugin, built without the custom-button and
lib-extension helpers, sends button 0 and the OFEX counter step; KAT follows the protocol code — see phoenix_v2.md.)

## Key file

`Protocol: Hay21`, `Bit: 21`, `Key`. The reference encoder writes the bumped Key back.

## Validation

- Oracle parity (doctor and garage builds): `hay21_raw.sub` decodes 5 frames with identical Key / serial / button /
  counter; no decode on any other unit-test file, IMPORTS or the UberGuidoZ set.
- Encoder: three keys pulse-exact against doctor-firmware (126 pulses), every custom-button switch too (doctor and
  Flipper-ARF `lib` builds).
- Unit tests: synthetic frames (every-other-frame behaviour), encode → decode for every button and the counter wrap,
  upload layout, rejections, key-file round trip, the unit-test capture through the decoder and the registry.
