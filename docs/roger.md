---
layout: default
---

# Roger Protocol

**Rust module:** `src/protocols/roger.rs`
**Reference:** `REFERENCES/Flipper-ARF/applications/system/subghz_garage/protocols/roger.c` (garage app copy,
primary); doctor-firmware and Flipper-ARF `lib/subghz/protocols/roger.c` decode and transmit identically.

## Overview

Roger Technology gate remote: static 28-bit frame = 16-bit serial, 4-bit button and an 8-bit "End" byte (the
button again, or 0x20 / 0x23 on some remotes).

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| te_short | 500 µs | ±270 µs (te_delta) |
| te_long | 1 000 µs | ±270 µs |
| Gap | 9 500 µs LOW | 19·te_short, ±1 350 µs |
| Bits | 28 | MSB first |

Each bit is a HIGH then a LOW: 1 = HIGH 1000 + LOW 500, 0 = HIGH 500 + LOW 1000; the last bit's LOW is the gap.

## Frame Layout (28 bits)

- **bits 12..27** serial
- **bits 8..11** button (1, 2, 4, 8)
- **bits 0..7** End

## RF

- **Encoding:** PWM
- **RF modulation:** AM (OOK), 433.92 / 868 MHz
- **Encryption:** none (fixed code)

## Decoder Steps

1. **Reset** — a gap-length LOW clears the register.
2. **SaveDuration / CheckDuration** — HIGH/LOW pairs: (long, short) → 1, (short, long) → 0.
3. A gap-length LOW reads the last bit from its HIGH with two independent tests (≈1000 → 1, then ≈500 → 0: a
   731–769 µs HIGH passes both and adds two bits, as in the C), reports the frame when exactly 28 bits were read and
   returns to Reset (every other frame of a held button is reported, as in the C).

## Encoder

`subghz_protocol_encoder_roger_get_upload`: with the captured button the key is sent unchanged. Another button
(native 1, 2, 4 or 8, accepted when the captured button is one of them) rebuilds the key as the C does: End equal to
the captured button → `serial ‖ button ‖ button`; End 0x23 and button 1 → End 0x20; End 0x20 and button 2 → End 0x23;
any other End is sent unchanged. 28 bit pairs with the gap after the last, sent 3 times. (The garage app's TX plugin
sends button 0 because its custom-button helpers are stubbed — see phoenix_v2.md; KAT follows the protocol code.)

## Key file

`Protocol: Roger`, `Bit: 28`, `Key`.

## Validation

- Oracle parity (doctor and garage builds): `roger_raw.sub` gives 72 frames with identical Key, serial and button;
  no decode on any other unit-test file, IMPORTS or the UberGuidoZ set.
- Encoder: the unit-test key and two more keys (End 0x20 and End == button) pulse-exact against doctor-firmware
  (168 pulses), every custom-button switch too (doctor and Flipper-ARF `lib` builds).
- Unit tests: key-file load → encode → decode, the End-byte button rules, the overlapping last-bit windows, rejections,
  the unit-test capture through the decoder and the registry.
