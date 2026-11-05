---
layout: default
---

# Astrostart 2000 (rtl_433 port, receive only)

**Rust module:** `src/protocols/astrostart_2000.rs` (rtl_433 receive chain: `src/protocols/rtl433/`)
**Reference:** rtl_433 `src/devices/astrostart_2000.c` ("Astrostart 2000 Car Remote (-f 372.4M)", model
`Astrostart-2000`), slicer `pulse_slicer_ppm`

## Overview

Astroflex Astrostart 2000 / 3000 remote-starter remotes (FCC ID J5F-TX2000), 372.4 MHz: a button code with its
inverse, a 32-bit ID and a 4-bit ID checksum; three messages per press. The remote has a primary and a secondary ID
(the recordings: 3410DE88 and 0410DE88). No Flipper reference has it.

## Timing / modulation

| Parameter   | Value          | Notes |
|-------------|----------------|-------|
| Modulation  | `OOK_PULSE_PPM` | the gap after each ~100 µs pulse carries the bit |
| short_width | 326 µs ± 80    | gap 247–405 µs → 0 |
| long_width  | 526 µs ± 80    | gap 447–605 µs → 1 |
| reset_limit / gap_limit | 541 µs | a gap ≥ 541 µs ends the message (a 541–605 µs gap is first a 1 bit, as in the C); other gaps < 541 µs start a new row |

## Frame Layout (exactly 52 bits, one row)

| Bits  | Field |
|-------|-------|
| 0–7   | button code |
| 8–15  | inverse of the button code |
| 16–47 | 32-bit ID |
| 48–51 | checksum: both nibbles of the four ID bytes added mod 16 |

## Decoder Steps

1. rtl_433's PPM slicer over KAT's AM pairs; the message ends on its long gap (KAT reports there).
2. `decode_fn`: exactly one row of exactly 52 bits; byte 0 = !byte 1 and the checksum (`DECODE_FAIL_MIC`).

## Field mapping

| KAT | rtl_433 |
|-----|---------|
| `serial` | `id` |
| `button` | `button_code` (8 bits); native names = `button_str` from rtl_433's map (Lock 2B 03 27 0F 35 37, Panic 1F 35 0D 25 15 2D, Start 13 37 2D 33 3D 3B, Stop 2F 0D 33 03 1D 17, Trunk 23 25 3D 1D 27 07, Unlock 0B 15 3B 17 07 0F, Multiple 3F; a code in several lists names them all, else "?") |
| `data` | the 52 row bits, `data_count_bit` 52 |
| `crc_valid` | true (inverse byte + checksum) |

## Encoder

Receive only, as rtl_433.

## Validation

- rtl_433_tests `astroflex/astrostart_2000/01` (5 recordings, 372.4 MHz, 250 kS/s): `kat --decode` gives rtl_433's
  15 events with the same fields and counts (panic: primary ID twice, secondary once).
- KAT's pulses are 4–8 µs shorter and the gaps 4–8 µs longer than rtl_433's (98–101 vs 104–108 µs pulses), well
  inside the ±80 µs gap windows.
- rtl_433's own pulse packages through KAT's decoder: the expected `.json` files exactly; KAT's pulses through the C
  reference and through KAT: the same events.
- Fuzz against the C reference (gap windows probed at their exclusive bounds): 2 × 20 000 trains, identical output,
  8 036 events.
- No decode on any IMPORTS capture, the Flipper unit-test captures or another device's rtl_433_tests recording.
