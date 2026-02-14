# Scher-Khan Protocol

**Rust module:** `src/protocols/scher_khan.rs`  
**Reference:** `REFERENCES/ProtoPirate/protocols/scher_khan.c`

## Overview

Scher-Khan is decode-only. PWM: 750 µs = 0, 1100 µs = 1; preamble uses 2× short then alternating. Variable bit count (35, 51, 57, 63, 64, 81, 82); only 51-bit format is parsed for serial/button/counter. Reference: phreakerclub.com/72.

## Timing

| Parameter | Value   | Notes   |
|-----------|---------|---------|
| Short (0) | 750 µs  | ±160 µs |
| Long (1)  | 1100 µs| ±160 µs |
| Min bits  | 35      | (variable) |

## Encoding

PWM; preamble: two short then alternating short/long.

## Frame Layout (variable)

- **51-bit format:** serial (28) | button (4) | counter (16) — “MAGIC CODE” / Dynamic; parsed for serial/btn/cnt.
- Other lengths (35, 57, 63, 64, 81, 82) decoded as raw bit count but not field-parsed.

## Decoder Steps

1. **Reset** — Wait for preamble (2 short + alternating).
2. **CheckPreamble** — Confirm preamble pattern.
3. **SaveDuration** — Store pulse duration.
4. **CheckDuration** — Short–short = 0, long–long = 1 (or equivalent); add bit; on end marker, if bit_count in {35,51,57,63,64,81,82} return decode (51-bit parsed for fields).

## Encoder

Not implemented (no encoder in reference).

## Frequencies

433.92 MHz.
