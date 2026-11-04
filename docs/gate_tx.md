---
layout: default
---

# GateTX Protocol

**Rust module:** `src/protocols/gate_tx.rs`
**Reference:** Flipper-ARF garage app `applications/system/subghz_garage/protocols/gate_tx.c` (primary;
doctor-firmware decodes and encodes the same); Flipper-ARF `lib/subghz/protocols/gate_tx.c` (button switch)

## Overview

Gate-TX 24-bit fixed-code remotes. The serial and button sit in the bit-reversed key.

## Timing

| Parameter  | Value              | Notes                         |
|------------|--------------------|-------------------------------|
| te_short   | 350 µs             | ±100 µs                       |
| te_long    | 700 µs             | ±300 µs                       |
| Header     | 16450 ± 4700 µs    | encoder: 49·te = 17150 µs     |
| Start bit  | HIGH 700 ± 300 µs  |                               |
| End LOW    | ≥ 3600 µs          |                               |

## Frame Layout (24 bits)

MSB first: bit 1 = LOW 700 + HIGH 350, bit 0 = LOW 350 + HIGH 700. With `r` = the 24-bit reversed key
(`subghz_protocol_gate_tx_check_remote_controller`): **serial** = (r & 0xFF) << 12 | (r >> 8 & 0xFF) << 4
| (r >> 20 & 0xF); **button** = r >> 16 & 0xF.

## RF

- **Encoding:** PWM
- **RF modulation:** AM (OOK), 433.92 MHz
- **Encryption:** none (fixed code)
- **Buttons:** "Btn 0" … "Btn F"

## Decoder Steps

1. **Reset** — header LOW → FoundStartBit.
2. **FoundStartBit** — HIGH 700 ± 300 µs starts the bits, anything else → Reset.
3. **SaveDuration** — a LOW ≥ 3600 µs ends the frame (24 bits reported) and waits for the next start bit;
   a HIGH here is ignored, as in the C.
4. **CheckDuration** — HIGH completes the pair (0 / 1) or → Reset.

## Encoder

`subghz_protocol_encoder_gate_tx_get_upload`, `encoder.repeat` = 3: header, start bit, 24 bits. `button`
0..=15 replaces the button nibble (bits 16..19 of the reversed key, ARF lib's custom button); larger codes
keep the captured key.

## Key file

`Protocol: GateTX`, `Bit: 24`, `Key`.

## Validation

Garage C oracle (`decode --protocol GateTX`): 47 identical decodes on the Flipper unit-test corpus, 378 on
3420 fuzz files; none on IMPORTS. In the full registry the unit test's `gate_tx_raw.sub` also yields one
Princeton frame (`FE8929`) — the garage app's own registry reports it too. Encoder: 26 uploads equal both
oracles' pulse for pulse.
