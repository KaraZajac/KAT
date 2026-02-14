# Kia V2 Protocol

**Rust module:** `src/protocols/kia_v2.rs`  
**Reference:** `REFERENCES/ProtoPirate/protocols/kia_v2.c`

## Overview

Kia V2 uses Manchester encoding at 500/1000 µs. 53 bits: 32 serial + 4 button + 12 counter + 4 CRC, plus start bit. Long preamble of 252 long pairs. CRC4 (XOR nibbles + offset 1).

## Timing

| Parameter | Value   | Notes   |
|-----------|---------|---------|
| Short     | 500 µs  | ±150 µs |
| Long      | 1000 µs | ±150 µs |
| Min bits  | 53      |         |

## Encoding

Manchester encoding; bit value from short/long and transition.

## Frame Layout (53 bits)

- Start bit + 32 serial + 4 button + 12 counter (byte-swapped) + 4 CRC.

## Decoder Steps

1. **Reset** — Wait for preamble (long pulses).
2. **CheckPreamble** — Count 252 long pairs.
3. **CollectRawBits** — Manchester decode; at 53 bits validate CRC4 and return.

## Encoder

Supported; preamble and Manchester 53-bit frame.

## Frequencies

433.92 MHz.
