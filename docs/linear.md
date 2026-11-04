---
layout: default
---

# Linear Protocol

**Rust module:** `src/protocols/linear.rs`
**Reference:** Flipper-ARF garage app `applications/system/subghz_garage/protocols/linear.c` (primary;
doctor-firmware and ARF's lib copy have the same wire behaviour)

## Overview

Linear / MultiCode-style 10-bit DIP-switch garage remotes (300–310 MHz). The C stores the code the way the
air format reads it and only inverts it for display ("implemented the wrong way around"); KAT keeps the C's
key so key files interoperate.

## Timing

| Parameter  | Value              | Notes                                         |
|------------|--------------------|-----------------------------------------------|
| te_short   | 500 µs             | ±350 µs                                       |
| te_long    | 1500 µs            | ±350 µs                                       |
| Guard      | 21000 ± 5250 µs    | 42·te after a final 1, 44·te after a final 0  |

## Frame Layout (10 bits)

MSB first: bit 1 = HIGH 1500 + LOW 500, bit 0 = HIGH 500 + LOW 1500; the last bit's LOW is the guard. No
fields (the C displays the inverted key as the DIP switches).

## RF

- **Encoding:** PWM
- **RF modulation:** AM (OOK); 315 MHz (doctor also flags 433.92)
- **Encryption:** none (fixed code)

## Decoder Steps

1. **Reset** — a guard LOW starts a frame.
2. **SaveDuration** — a HIGH; a LOW → Reset.
3. **CheckDuration** — a LOW ≥ 2500 µs ends the frame: it must itself be a guard (else dropped), the last
   bit comes from its HIGH alone, exactly 10 bits are reported, and the decoder needs a new guard (so the
   repeat right after a frame is skipped). Otherwise the pair gives 0 / 1, or → Reset.

## Encoder

`subghz_protocol_encoder_linear_get_upload`, `encoder.repeat` = 3: the 10 bits with the guard as the last
LOW. Fixed code: `button` ignored.

## Key file

`Protocol: Linear`, `Bit: 10`, `Key`.

## Validation

Garage C oracle (`decode --protocol Linear`): 89 identical decodes on the Flipper unit-test corpus, 348 on
3420 fuzz files, 1920 on the UberGuidoZ `Linear-10bit-*` sample; none on IMPORTS. Encoder: 26 uploads equal
both oracles' pulse for pulse.
