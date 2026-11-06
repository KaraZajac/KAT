---
layout: default
---

# Chrysler car remote, rtl_433 (receive only)

**Rust module:** `src/protocols/chrysler_car_remote.rs` (rtl_433 receive chain: `src/protocols/rtl433/`)
**Reference:** rtl_433 `src/devices/chrysler_car_remote.c` ("Chrysler Car Remote (-f 315.1M -s 920k)", model
`Chrysler-CarRemote`), slicer `pulse_slicer_pwm`

## Overview

Fixed-code 315 MHz Chrysler remotes (56008761, 56008762 / FCC ID GQ43VT7T, 04686366, 56021903AA). This is not
KAT's ProtoPirate-based **Chrysler V0** (300/600 µs pulses with ~3.5 ms gaps, 80 bits), which does not decode these
remotes. No Flipper reference has it.

## Timing / modulation

| Parameter   | Value              | Notes |
|-------------|--------------------|-------|
| Modulation  | `OOK_PULSE_PWM`    | |
| short_width | 350 µs ± 100       | 251–449 µs → 1 |
| long_width  | 700 µs ± 100       | 601–799 µs → 0 |
| sync_width  | 7350 µs ± 100      | 7251–7449 µs → sync |
| other pulses | ≤ 250 µs ignored; anything else starts a new row | |
| gap_limit   | 4200 µs            | a longer gap starts a new row |
| reset_limit | 17 500 µs          | a longer gap ends the message |

A press is one message: a ~7.4 ms sync pulse, a 25-bit preamble row, then the packet twice (49 and 48 bits) 16 ms
apart; bits are ~330/3830 µs (1) and ~650/3510 µs (0).

## Frame Layout (48 bits per packet, inverted and bit-reversed per byte)

| Bytes | Field |
|-------|-------|
| 0–3   | 32-bit ID |
| 4     | high nibble: button code (0x1 Unlock, 0x2 Lock, 0x4 Panic); bit 2: multi-press |
| 5     | checksum = sum of bytes 0–4 |

## Decoder Steps

1. rtl_433's PWM slicer over KAT's AM pairs, with rtl_433's pulse-detector package rules (a package also ends on a
   gap over 10 ms and 10 × its longest pulse, or at its 1200th pulse).
2. For every row of 48 or more bits: invert, take the 48 bits that end one bit before the row's end (49 bits or
   more) or the whole 48-bit row, reverse each byte.
3. Checks: checksum (`DECODE_FAIL_MIC`); sum not 0 / 5 × 0xFF, at least one of the three buttons, multi-press bit
   set exactly when more than one is pressed (`DECODE_FAIL_SANITY`). Every valid row is an event.
4. KAT reports each valid packet as soon as its row is final (the next row has started, or the message ended), so
   both packets of a press are reported, as rtl_433 outputs them.

## Field mapping

| KAT | rtl_433 |
|-----|---------|
| `serial` | `id` |
| `button` | `button_code` (4 bits); native names = `button_str` ("Unlock; Lock" for a combination) |
| `data` | the 48 wire bits the event was read from (before inversion / bit reversal), `data_count_bit` 48 |
| `crc_valid` | true (checksum) |

## Encoder

Receive only, as rtl_433.

## Validation

- rtl_433_tests `chrysler_car_remote` 01–05 (7 recordings, 315.1 MHz, **920 kS/s**, carriers 74–384 kHz off the
  centre): `kat --decode` gives every recording's rtl_433 event (same ID / button), all 10 (until v1.8.0's receiver
  fixes KAT read 9: in 04 its pulses lost one bit of the first packet). KAT also misses the 25-pulse preamble of every press
  (not used by the decoder). Forcing `--rate 250k` on these 920 kS/s files decodes nothing, as before.
- rtl_433's own pulse packages through KAT's decoder: the expected `.json` files exactly (10 events).
- Fuzz against the C reference: 2 × 20 000 trains (presses with and without the sync, long presses past 1200
  pulses), identical output, 44 434 events.
- No decode on any IMPORTS capture, the Flipper unit-test captures or another device's rtl_433_tests recording.
  rtl_433_tests' `known_false_positives.json` lists one Chrysler-CarRemote decode on an iMars-T240 recording (not in
  this checkout); KAT's decoder would read those pulses the same way.
