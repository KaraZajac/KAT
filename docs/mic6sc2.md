---
layout: default
---

# MIC6SC2-CarRemote Protocol

**Rust module:** `src/protocols/mic6sc2.rs` (rtl_433 receive chain: `src/protocols/rtl433/`)
**Reference:** `REFERENCES/rtl_433/src/devices/mic6sc2_car_remote.c`, `six_sc_two_car_remote_decode`
(r_device "MIC 6SC2 Car Remote (-f 315.1M)", decoder 312, enabled by default)

## Overview

6SC2 CMGU / 6SC2 CDFA car remotes (an MSOP-10 chip, perhaps from MIC, Master Instrument Corporation), 315 MHz
OOK, up to 4 buttons, a rolling code with an unencrypted sequence number. The frame has no remote ID. Receive
only, as rtl_433.

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| Modulation | OOK, `OOK_PULSE_MANCHESTER_ZEROBIT` | carrier on = HIGH |
| Short (half-bit) | 250 µs | data edge threshold 375 µs |
| Reset limit | 10 ms | ends the message |
| Tolerance | none | |

## Frame Layout

80 bits, every byte sent LSB first ("reflected"):

| byte | field |
|------|-------|
| 0..1 | preamble `0x55 0x54` (its first 0 is the slicer's hard-coded bit) |
| 2..5 | encrypted (32 bits): `rev8(b5) rev8(b4) rev8(b3) rev8(b2)` |
| 6 | button `rev8(b6) & 0xF` (1 Unlock, 2 Lock, 3 Trunk, 4 Panic), unknown nibble |
| 7..8 | sequence (16 bits): `rev8(b8) rev8(b7)` |
| 9 | checksum: bytes 2..10 XOR to 0 (byte 10 holds the row's bits past 80, normally none) |

## Decoder Steps

1. Slice the stream into messages (zero-bit Manchester, end at a gap > 10 ms).
2. The first row of ≥ 48 bits (`bitbuffer_find_repeated_row(…, 1, 48)` counts the row itself), ≤ 88 bits.
3. Preamble `55 54` (`DECODE_FAIL_SANITY`), XOR (`DECODE_FAIL_MIC`). Failing frames are dropped.

## Field mapping

| KAT | rtl_433 |
|-----|---------|
| `serial` = none | (no ID in the frame) |
| `button` | `button_code` |
| `counter` | `sequence` (16 bits) |
| `crc_valid` = true | `mic: CHECKSUM` |
| `data`/`extra` = the row (normally 80 bits: 64 in `data`, 16 in `extra`) | `encrypted` from `data`'s bytes 2..5, reflected |

## Encoder

Receive only, as rtl_433 (no encoder).

## Validation

- **Recordings:** rtl_433_tests `tests/6sc2/01` and `02` (6 recordings of two remotes, 315.1 MHz, 250 kS/s): all six
  decode through KAT's receive pipeline with rtl_433's fields (button, sequence 8545..8547 and 38486..38488,
  encrypted); `decodes_rtl433_recordings` checks them against their `.json` files.
- **Oracle parity** (rtl_433 reading OOK pulse files): 4000 fuzzed packages, identical decodes (614 frames);
  identical slicer rows for all messages, also with tolerances 80 µs and 40 µs (reset 4 ms), which exercise the
  slicer's row splits.
- **Synthetic IQ** (250 kS/s): frames built from the C decode identically in rtl_433 and KAT.
- No decode on any IMPORTS capture, Flipper unit-test capture or rtl_433_tests recording (decoder run alone, both
  polarities).
