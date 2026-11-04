---
layout: default
---

# Dooya Protocol

**Rust module:** `src/protocols/dooya.rs`
**Reference:** `REFERENCES/Flipper-ARF/applications/system/subghz_garage/protocols/dooya.c` (garage app copy,
primary); button switch from Flipper-ARF's `lib/subghz/protocols/dooya.c` (union); doctor-firmware's copy also drops
degenerate keys.

## Overview

Dooya blind / roller-shutter remote: static 40-bit frame = 24-bit serial, a single/multi-channel nibble, a channel
nibble and an 8-bit key code.

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| te_short | 366 µs | ±120 µs (te_delta) |
| te_long | 733 µs | ±240 µs (2·te_delta) |
| Header | LOW 12·te_long (8 796 µs) | ±2 400 µs; the encoder adds the last bit's LOW (te_long if bit 0 is 1, else te_short) |
| Start | HIGH 13·te_short (4 758 µs) ±600, LOW 2·te_long (1 466 µs) ±360 | |
| Frame end | LOW ≥ 4·te_long (2 932 µs) | |
| Bits | 40 | MSB first |

Each bit is a HIGH then a LOW: 1 = HIGH 733 + LOW 366, 0 = HIGH 366 + LOW 733.

## Frame Layout (40 bits)

- **bits 16..39** serial
- **bits 12..15** 0 = single-channel console, else multi-channel
- **bits 8..11** channel (multi-channel console); KAT's `counter` = the C's `cnt` = this channel, or 0xFF (single)
- **bits 0..7** key code: 0x11 Up_Long, 0x1E Up_Short, 0x33 Down_Long, 0x3C Down_Short, 0x55 Stop, 0x79 Up+Down,
  0x80 Up+Stop, 0x81 Down+Stop, 0xCC P2

## RF

- **Encoding:** PWM
- **RF modulation:** AM (OOK), 315 / 433.92 MHz
- **Encryption:** none (fixed code)

## Decoder Steps

1. **Reset** — header LOW.
2. **FoundStartBit** — HIGHs within 600 µs of 4 758 µs are skipped; a LOW within 360 µs of 1 466 µs starts the data.
3. **SaveDuration / CheckDuration** — (short HIGH, long LOW) → 0, (long HIGH, short LOW) → 1.
4. A LOW ≥ 2 932 µs reads the last bit from its HIGH and reports the frame when exactly 40 bits were read (garage
   copy: any key; doctor-firmware additionally rejects degenerate keys — all 0/1, 0xAA…/0x55…, four equal nibbles
   in a row).

## Encoder

`subghz_protocol_encoder_dooya_get_upload`: header, start HIGH, start LOW and the 40 bit pairs; the upload is cut after
2·40 + 2 entries, so the last bit's LOW is not sent (the next copy's header follows its HIGH); sent 3 times.
`button`: the captured code, or one of Flipper-ARF's d-pad codes 0x11 / 0x33 / 0x55 / 0xCC written into bits 0..7;
anything else keeps the captured code.

## Key file

`Protocol: Dooya`, `Bit: 40`, `Key`.

## Validation

- Oracle parity (doctor and garage builds): `dooya_raw.sub` and `test_random_raw.sub` give 5 frames each with
  identical Key / serial / button / channel; no decode on any other unit-test file, IMPORTS, the UberGuidoZ set or the
  rtl_433 recordings. Fuzzed streams: identical decodes (no degenerate key came up, so doctor and garage agree there).
- Encoder: the unit-test key and two more keys pulse-exact against doctor-firmware and the garage build (246
  pulses); every d-pad button (Flipper-ARF `lib` build with its custom-button switch) pulse-exact.
- Unit tests: key-file load → encode → decode (the cut last LOW needs the silence after the burst), button switch and
  channel field, upload layout, rejections, the unit-test capture through the decoder and the registry.
