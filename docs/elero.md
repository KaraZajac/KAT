---
layout: default
---

# Elero Protocol

**Rust module:** `src/protocols/elero.rs` (rtl_433 receive chain: `src/protocols/rtl433/`)
**Reference:** rtl_433 `src/devices/elero.c` (GPL-2.0; model `Elero`, r_device "Elero bidirectional blinds/awning
remote (Silent Gliss and others)", `-R 372`)

## Overview

Elero's bidirectional 868 / 915 MHz blind and awning motor remotes (e.g. Silent Gliss wall switches): a CC1101-style
GFSK packet with an 8-bit rolling counter and a fixed, reversible command obfuscation (no key). Receive only.

## Timing / modulation

- FSK_PULSE_PCM, 13 µs per bit (76.8 kbit/s GFSK, ±38 kHz in the rtl_433_tests recordings), `reset_limit` 4000 µs.
- The PCM slicer measures the bit period on the 0x55 preamble; upper tone = 1.
- **KAT's FM path**: a stopped carrier and the lower tone are both LOW. A LOW over 32 bit periods (416 µs) ends the
  package (rtl_433's FSK packages end with the carrier); the frame's final zero bits merge into that LOW, so it is read
  as 0 … 32 trailing zeros and the shortest reading that passes the CRC wins.
- **KAT's receiver**: its normal FM path averages 20 µs and debounces 30 µs, which smooths 13 µs bits away. When that
  path finds no FSK in a burst, the receiver slices the burst again without smoothing (both tones present, levels on
  a grid of one element under 60 µs; see [radio.md](radio.md)), and that delivers Elero's bits.

## Frame Layout

`0x55…` preamble, sync word `0xA723A723` of which only the first 31 bits are sync, then (IBM PN9 de-whitened):

`len cnt typ typ2 hop syst chl src(3) bwd(3) fwd(3) ndst dst(ndst) p1 p2 enc(8) crc(2)` — `len + 3` bytes.

## Decoder Steps (`elero_decode`)

1. Exactly one row; search the 31-bit sync; at least one byte after it.
2. Extract up to 40 bytes, de-whiten; the frame must fit; CRC-16 (poly 0x8005, init 0xFFFF) over every byte before it.
3. `17 + ndst + 2 + 8 + 2 ≤ len + 3`.
4. De-obfuscate `enc` (nibble table, two decrementing-key subtractions, XOR); byte 2 = command: 0x20 Up, 0x10 Stop,
   0x40 Down.

## Field mapping

| KAT | rtl_433 | Notes |
|---|---|---|
| `serial` | `id` | `src`, 24 bits |
| `button` | `command` | the command byte (labels Up / Stop / Down; rtl_433 prints `?` for others) |
| `counter` | `counter` | 8 bits |
| – | `channel` | the destination list (hex bytes); the sending channel `chl` is byte 6 of `data` |
| `data` / `extra` | | frame bytes 0..8 / the last 8 frame bytes (de-whitened); `data_count_bit` = 8 · (len + 3) |
| `crc_valid` | `mic: CRC` | always true for a reported frame |

## Encoder

None — receive only, as rtl_433.

## Validation

- **rtl_433_tests `elero/codes_test.txt`**: the 38 CRC-valid codes of the reporter's captures, as 13 µs NRZ pulse
  trains, all decode to `codes_test.json` (id, command, counter, channel list) — `rtl_433_tests_codes` (runs whenever
  REFERENCES is present).
- **The 3 recordings** (2.048 MS/s) through a plain test-only FM discriminator: 3 / 3 match their JSON
  (`rtl_433_tests_recordings_with_a_test_demodulator`, ignored). Through KAT's IQ pipeline (`kat --decode`): 3 / 3
  with rtl_433's ID, command and counter (84D351: Up 106, Stop 113, Down 114); `rtl433_recordings_decode`
  (`src/radio/sim.rs`, ignored) checks that their FM pulses fall on the 13 µs grid.
- **rtl_433 oracle** on 400 synthetic packets (the 38 real codes plus generated frames, jitter, bit flips,
  truncation): rtl_433 slices each package with a flex `OOK_PCM` decoder of the same timing (the slicer FSK_PCM
  uses) and decodes every message with the real decoder (`-R 372 -y`): 293 frames identical, plus one CRC-valid frame
  only KAT reads (rtl_433 rounded the final lower-tone run one bit short; KAT's carrier-off reading tries every tail).
- No decode of any other recording (IMPORTS, rtl_433_tests, Flipper unit-test captures).
