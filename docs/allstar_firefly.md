---
layout: default
---

# Allstar Firefly Protocol

**Rust module:** `src/protocols/allstar_firefly.rs`
**Reference:** `REFERENCES/Flipper-ARF/applications/system/subghz_garage/protocols/allstar_firefly.c` (garage app
copy, primary; decode only in the app, which ships no TX plugin for it); encoder from
`REFERENCES/doctor-firmware/lib/subghz/protocols/allstar_firefly.c` (union). Flipper-ARF's `lib/subghz` copy is the
same protocol.

## Overview

Allstar Firefly 318ALD31K gate / garage remote (Supertex ED-9 encoder): nine trinary DIP switches sent as 18 bits,
20 frames per press, ~30 ms between frames. US market, 318 MHz (the references flag 315 MHz; doctor-firmware adds
433 MHz).

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| te_short | 600 µs | ±300 µs (te_delta) |
| te_long | 4 000 µs | ±300 µs |
| Gap (decoder) | 30 000 µs LOW | 50·te_short, ±1 500 µs |
| Gap (encoder) | 30 400 µs LOW | 50·te_short + 400 |
| Bits | 18 | MSB first |

Each bit is a HIGH then a LOW: 1 = HIGH 4000 + LOW 600, 0 = HIGH 600 + LOW 4000; the last bit's LOW is the gap.

## Frame Layout (18 bits)

Nine two-bit DIP positions, switch 1 in bits 17..16: `11` = '+', `10` = '0' (float), `00` = '−'. The references
display the key, its bit reversal ("Yek") and the three DIP rows; there is no serial / button / counter field.

## RF

- **Encoding:** PWM (trinary DIP code)
- **RF modulation:** AM (OOK)
- **Encryption:** none (fixed code)

## Decoder Steps

1. **Reset** — a gap-length LOW clears the register.
2. **SaveDuration / CheckDuration** — HIGH/LOW pairs: (long, short) → 1, (short, long) → 0.
3. A gap-length LOW reads the last bit from its HIGH, reports the frame when exactly 18 bits were read and returns to
   Reset (every other frame of a burst is reported, as in the C). The garage copy tests the last bit with `else if`,
   doctor-firmware with two `if`s; the HIGH windows do not overlap, so both behave the same.

## Encoder

doctor-firmware `subghz_protocol_encoder_allstar_firefly_get_upload`: the 18 bit pairs with the 30 400 µs gap after
the last, sent 5 times. Static: no button, no counter.

## Key file

`Protocol: Allstar Firefly`, `Bit: 18`, `Key`.

## Validation

- No unit-test capture exists. Oracle parity (doctor and garage builds): no decode on the Flipper unit-test corpus,
  IMPORTS, the UberGuidoZ garage/gate set or the rtl_433 recordings (incl. the Cardin S466 trinary remote, whose
  ~30.5 ms gap is in this window but whose 700/1 400 µs bits and 6.2 ms sync are not); fuzzed encoder uploads decode
  identically in KAT and both C builds.
- Encoder: two keys pulse-exact against doctor-firmware (180 pulses).
- Unit tests: the header comment's key file loads, encodes and decodes back (3 of 5 copies), synthetic frame,
  short frame / bad HIGH / short gap rejection.
