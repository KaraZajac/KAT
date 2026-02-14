# Subaru Protocol

**Rust module:** `src/protocols/subaru.rs`  
**Reference:** `REFERENCES/ProtoPirate/protocols/subaru.c`

## Overview

Subaru uses PWM: 800 µs HIGH = 1, 1600 µs HIGH = 0; LOW is 800 µs after each bit. 64 bits total (8 bytes MSB first: button(4) + serial(24) + counter-related). Preamble: 79 full 1600 µs pairs + 80th HIGH only; then gap 2800 µs, sync 2800 µs HIGH + 1600 µs LOW. Complex counter decoding from bytes 4–7.

## Timing

| Parameter | Value   | Notes   |
|-----------|---------|---------|
| Short (1) | 800 µs  | ±200 µs |
| Long (0)  | 1600 µs | ±200 µs |
| Gap       | 2800 µs |         |
| Sync      | 2800 µs HIGH + 1600 µs LOW | |
| Min bits  | 64      |         |

## Encoding

PWM: duration of HIGH = bit value (800 = 1, 1600 = 0); LOW 800 µs after each bit.

## Frame Layout (64 bits = 8 bytes)

- 4 bits button, 24 bits serial, counter-related in bytes 4–7 (decode_counter in code).

## Decoder Steps

1. **Reset** — Wait for long HIGH (preamble).
2. **CheckPreamble** — Count 79 long pairs + 80th HIGH; then expect gap 2800 µs.
3. **FoundGap** — Gap seen → look for sync (2800 HIGH, 1600 LOW).
4. **FoundSync** — Sync seen → SaveDuration.
5. **SaveDuration** — On HIGH: store duration, → CheckDuration. On end marker → return decode if 64 bits.
6. **CheckDuration** — On LOW: pair (te_last, duration) → short–short = 1, long–long = 0; add bit, → SaveDuration.

## Encoder

Supported; builds preamble, gap, sync, then 64 PWM bits.

## Frequencies

433.92 MHz (and/or 315 MHz as in reference).
