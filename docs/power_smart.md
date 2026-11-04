---
layout: default
---

# Power Smart Protocol

**Rust module:** `src/protocols/power_smart.rs`
**Reference:** `REFERENCES/doctor-firmware/lib/subghz/protocols/power_smart.c` (Flipper-ARF's garage app ships a copy
it does not build; Flipper-ARF's `lib/subghz` copy decodes and transmits the same frames — its d-pad only re-sends the
captured button).

## Overview

Power Smart roller-shutter remote: a 64-bit Manchester frame with two sync bytes and inverted copies of its fields,
sent 8 times back to back per upload.

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| te_short | 225 µs | half bit, ±100 µs (125–325 µs; tested first) |
| te_long | 450 µs | ±200 µs (325–650 µs once short failed) |
| End LOW | 499 950 µs | 1111·te_long after the 8 copies (encoder) |
| Bits | 64 | MSB first; every decoded bit is inverted |

## Frame Layout (64 bits)

`0xFD XXXX YY 0xAA ZZZZ WW` with XXXX = !ZZZZ and YY = (!WW) − 1 (`subghz_protocol_power_smart_check_valid`):

- **bits 56..63** sync 0xFD, **bits 24..31** sync 0xAA
- **button** K1K2 = bit 55 ‖ bit 40: 1 Down, 2 Up, 3 Stop, 0 Unknown
- **serial** `(data >> 33) & 0x3FFF00 | (data >> 32) & 0xFF`
- **counter** (the C's `cnt`) bits 49..54: the 6-bit channel mask the references display
- `crc_valid` = true (the inverted-copy check passed)

## RF

- **Encoding:** Manchester (Flipper convention, inverted bits)
- **RF modulation:** AM (OOK), 433.92 MHz
- **Encryption:** none (fixed code)

## Decoder Steps

Every short/long pulse is a Manchester event (`manchester_advance`); a decoded bit is shifted in inverted. After
every event (bit or not) the 64-bit register is tested: sync bytes in place and the inverted copies consistent →
reported (register cleared). Any other pulse clears the register and resets the Manchester decoder. `reset` only
resets the Manchester state (the register survives, as in the C).

## Encoder

`subghz_protocol_encoder_power_smart_get_upload`: 8 copies of the 64 inverted bits through Flipper's Manchester
encoder, the closing half kept only when HIGH, then the 499 950 µs LOW; the upload is sent 3 times. Static: the
button is spread over the inverted copies, so the references (and KAT) re-send the captured frame.

## Key file

`Protocol: Power Smart`, `Bit: 64`, `Key`, then doctor-firmware's generic serialize adds `Serial`, `Btn`, `Cnt`
(decimal) when any is non-zero (KAT writes them the same way; loading uses `Key` only, as the C).

## Validation

- Oracle parity (doctor build): `power_smart_raw.sub` gives 6 frames and `test_random_raw.sub` 6 frames with identical
  Key / serial / button / channel; no decode on any other unit-test file, IMPORTS, the UberGuidoZ set or the rtl_433
  recordings (incl. the 869 MHz Elero shutter remotes).
- Encoder: the unit-test key and a second key pulse-exact against doctor-firmware (2 166 pulses); the reference's
  encoder test decodes 21 frames, KAT's decoder the same.
- Unit tests: check_valid against the C's examples, key-file load → encode → decode, upload layout, corrupted frame
  and register clearing, `reset` keeping the register, the unit-test capture through the decoder and the registry.
