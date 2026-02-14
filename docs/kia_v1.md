# Kia V1 Protocol

**Rust module:** `src/protocols/kia_v1.rs`  
**Reference:** `REFERENCES/ProtoPirate/protocols/kia_v1.c`

## Overview

Kia V1 uses Manchester encoding at 800/1600 µs. 57 bits total: 32 serial + 8 button + 12 counter + 4 CRC. Long preamble (~90 long pairs). CRC4 with offset rules (cnt_high 0 vs ≥ 6).

## Timing

| Parameter | Value   | Notes   |
|-----------|---------|---------|
| Short     | 800 µs  | ±200 µs |
| Long      | 1600 µs | ±200 µs |
| Min bits  | 57      |         |

## Encoding

Manchester: symbol duration short or long; bit value from transition direction.

## Frame Layout (57 bits)

- 32 bits: serial
- 8 bits: button
- 12 bits: counter
- 4 bits: CRC4 (checksum with offset rules)

## Decoder Steps

1. **Reset** — Wait for long pulse (preamble).
2. **CheckPreamble** — Count long pairs until enough; then transition to data.
3. **DecodeData** — Manchester decode; at 57 bits validate CRC4 and return.

## Encoder

Supported; builds preamble and Manchester-encoded 57-bit frame.

## Frequencies

433.92 MHz.
