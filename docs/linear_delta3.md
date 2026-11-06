---
layout: default
---

# LinearDelta3 Protocol

**Rust module:** `src/protocols/linear_delta3.rs`
**Reference:** Flipper-ARF garage app `applications/system/subghz_garage/protocols/linear_delta3.c`
(primary; doctor-firmware and ARF's lib copy have the same wire behaviour)

## Overview

Linear Delta-3 8-bit DIP-switch remotes. The garage app, doctor-firmware, ARF and Momentum keep collecting bits
after an 8-bit frame, so their repeat check (`last_data`) reports only every third frame of a held button and never
a 3–4 frame tap; RogueMaster clears the bits after each frame (its encoder is the same, so a real remote looks like
its upload) and reports every frame after the first. KAT follows RogueMaster since v1.8.7: the Flipper unit-test
capture gives 62 frames (26 before).

## Timing

| Parameter  | Value              | Notes                                          |
|------------|--------------------|------------------------------------------------|
| te_short   | 500 µs             | ±150 µs                                        |
| te_long    | 2000 µs            | ±150 µs                                        |
| Start LOW  | 35000 ± 3600 µs    | 73·te after a final 1, 70·te after a final 0   |
| End LOW    | ≥ 5000 µs          |                                                |

## Frame Layout (8 bits)

MSB first: bit 1 = HIGH 500 + LOW 3500, bit 0 = HIGH 2000 + LOW 2000; the last bit's LOW is the guard.

## RF

- **Encoding:** PWM
- **RF modulation:** AM (OOK); 315 MHz (doctor also flags 433.92)
- **Encryption:** none (fixed code)

## Decoder Steps

1. **Reset** — a guard LOW starts a frame (bits cleared).
2. **SaveDuration / CheckDuration** — HIGH/LOW pairs; a LOW ≥ 5000 µs ends the frame (last bit from its
   HIGH). An 8-bit frame is reported only if it equals the previous 8-bit frame (non-zero); either way the
   C stays in SaveDuration **without clearing the bits**, so the next frame reads as 16 bits and is
   dropped (→ Reset), and the one after it is skipped while waiting for a start LOW. A held button:
   frame 2 is stored, frames 5, 8, 11 … decode. The app's reset inside the callback is overwritten by the
   code that follows it, so decoding does not change the cadence.

## Encoder

`subghz_protocol_encoder_linear_delta3_get_upload`, `encoder.repeat` = 3: the 8 bits with the guard as the
last LOW. One 3-frame burst therefore decodes neither on a Flipper nor in KAT; two do. Fixed code:
`button` ignored.

## Key file

`Protocol: LinearDelta3`, `Bit: 8`, `Key`.

## Validation

Garage C oracle (`decode --protocol LinearDelta3`): 48 identical decodes on the Flipper unit-test corpus,
81 on 3420 fuzz files; none on IMPORTS. Encoder: 26 uploads equal both oracles' pulse for pulse.
