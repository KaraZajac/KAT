---
layout: default
---

# Code Alarm FRDPC2002 (rtl_433 port, receive only)

**Rust module:** `src/protocols/code_alarm_frdpc2002.rs` (rtl_433 receive chain: `src/protocols/rtl433/`)
**Reference:** rtl_433 `src/devices/code_alarm_car_remote.c` ("Code Alarm FRDPC2002 Car Remote", model
`CodeAlarm-FRDPC2002`), slicer `pulse_slicer_manchester_zerobit`

## Overview

Code Alarm FRDPC2002 / GOH-FRDPC2002 car-alarm remotes, 433.92 MHz. rtl_433: a rolling code ("the manual tied to the
FCC ID states a 36-bit rolling code") with a 24-bit ID folded out of it and button flags; repeated while held. No
Flipper reference has it.

## Timing / modulation

| Parameter   | Value                          | Notes |
|-------------|--------------------------------|-------|
| Modulation  | `OOK_PULSE_MANCHESTER_ZEROBIT` | rising edge = 0, falling edge = 1, hard-coded leading 0 |
| short_width | 550 µs ± 100                   | a pulse or gap outside 450–1200 µs closes the row (the C's "[1]10" rule included) |
| reset_limit | 1600 µs                        | a longer gap ends the message |

## Frame Layout (row 0 of the message, exactly 60 bits)

| Bits  | Field |
|-------|-------|
| 0–15  | zero (checked) |
| 16–18 | not checked |
| 19–58 | rtl_433's `data` (40 bits): high nibble = button flags, then the code |
| 23–58 | the 36-bit rolling code; ID = `(c0^c1) << 16 \| (c1^c2) << 8 \| (c2^c3)` of its first four bytes |
| 59    | unknown |

## Decoder Steps

1. rtl_433's Manchester zerobit slicer over KAT's AM pairs; a gap over 1600 µs ends the message (KAT reports there).
2. `decode_fn` (row 0 only): exactly 60 bits; bytes 0 and 1 are 0x00; the 40-bit data does not sum to 0 or
   ≥ 5 × 0xFF (`DECODE_FAIL_SANITY`).

## Field mapping

| KAT | rtl_433 |
|-----|---------|
| `serial` | `id` (24 bits) |
| `button` | `button_code` (4 bits); native names = `button_str` from rtl_433's map: 0 Start, 1 Panic, 3 "Panic; Start", 4 "Lock; Unlock", 5 Unlock, 6 Lock, 7 Multiple, else "?" |
| `data` | the 60 row bits, `data_count_bit` 60; rtl_433's `data` = bits 19–58 (`data >> 1`, 40 bits) |
| `counter` | none (rtl_433 does not decode the rolling part) |
| `crc_valid` | false (no integrity check) |

## Encoder

Receive only, as rtl_433.

## Validation

- rtl_433_tests `code-alarm/frdpc2002/01` (5 recordings, 433.92 MHz, 250 kS/s): `kat --decode` gives rtl_433's 8
  events with the same fields and counts (lock ×2, lock+unlock ×2, panic, start, unlock ×2).
- rtl_433's own pulse packages through KAT's decoder: the expected `.json` files exactly; KAT's pulses through the C
  reference and through KAT: the same events.
- Fuzz against the C reference: 2 × 20 000 trains, identical output, 19 727 events.
- No decode on any IMPORTS capture, the Flipper unit-test captures or another device's rtl_433_tests recording.
