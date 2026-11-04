---
layout: default
---

# Dickert MAHS Protocol

**Rust module:** `src/protocols/dickert_mahs.rs`
**Reference:** Flipper-ARF `applications/system/subghz_garage/protocols/dickert_mahs.c` (garage app, RX group g9 +
`tx_dickert_mahs`); doctor-firmware decodes and encodes identically (and prints the switches).

## Overview

Dickert MAHS433 garage remotes: 18 three-state switches, 2 bits each (`11` +, `01` 0, `00` −; `10` shows as `?`):
10 user DIP switches then an 8-symbol factory code. No checksum (rtl_433 calls it "no CRC" too), no button field.

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| Short | 400 µs | ±100 µs |
| Long | 800 µs | ±100 µs |
| Header | LOW 40 000 µs ±7000 | `50·te_long`; the encoder sends `112·te_short` = 44 800 µs |
| Start pulse | HIGH 400 µs | |
| Bit pair | LOW + HIGH = 1200 ±100 µs | LOW 800 → `1`, LOW 400 → `0` |
| Bits | ≥ 36 | the C's `>=`; the encoder handles 36..63 |

## Frame Layout

36 bits, MSB first: user switches = symbols 0..9, factory code = symbols 10..17. KAT reports no serial/button/counter.

## RF

- **Encoding:** PWM (bit in the LOW width, constant 1200 µs pair)
- **RF modulation:** AM, `SubGhzProtocolFlag_433 | AM`; AM650
- **Encryption:** none (fixed code)

## Decoder Steps

1. **Reset** — first, if at least 36 bits were collected, report them (the C reports on the pulse *after* the frame
   broke off); then a header LOW → Initial.
2. **Initial** — LOWs ignored; a short HIGH starts recording.
3. **Recording** — LOW/HIGH pairs summing to 1200 ±100 µs are bits; the first pair that does not fit → Reset.

**Reset** (KAT deviation): the C's `reset` only rewinds the step and keeps the bit count, so a completed frame could be
reported on the first pulse of the *next* capture (KAT reuses one registry across captures). KAT's `reset` also
clears the count and pair buffer. Where the C checks for a header *after* its callback (the reporting pulse is itself a
header), KAT's `reset` right after the report resumes at the start pulse, as the C does.

## Encoder

`repeat` = 3, no counter, `button` ignored: header 44 800 µs, start pulse, then `1` = LOW 800 + HIGH 400, `0` = LOW
400 + HIGH 800. Keys of 36..=63 bits load and send (the C's `deserialize` accepts ≥ 36 bits; its 128-entry upload holds
63).

## Key file

`Protocol: Dickert_MAHS`, `Bit: ≥ 36`, `Key` = the code (8 bytes). Loading requires `Bit ≥ 36`.

## Validation

- Flipper unit test `dickert_raw.sub` decodes (5 frames, `155577515`); `dickert_mahs.sub` loads and encodes.
- rtl_433 real IQ `dickert_mahs/01/g024_433.92M_1000k.cu8`: KAT decodes 4 frames `F4F075515` = user switches
  `++0-++--0+`, factory code `0000-000` — exactly rtl_433's expected decode. rtl_433 reports all 9 repeats; the
  Flipper decoder (and the C oracle fed the same demodulated pulses: 4 frames) reports every other one, because a frame
  is only reported after the next one's header and start pulse have been consumed.
- C oracle (doctor + garage builds): decodes on the unit-test corpus, IMPORTS, UberGuidoZ and 4224 fuzz files (two seeds) equal the
  oracle's; encoder uploads pulse-identical for 10 key files (36, 37, 48 and 63 bits).
