# PSA (Peugeot/Citroën) Protocol

**Rust module:** `src/protocols/psa.rs`  
**Reference:** `REFERENCES/ProtoPirate/protocols/psa.c`

## Overview

PSA uses Manchester at 250/500 µs symbol (125/250 µs sub-symbol for preamble). 128 bits total: key1 (64) + validation (16) + key2/rest (48); decode uses key1 + 16-bit validation. TEA decrypt/encrypt with fixed key schedules; mode 0x23 adds an XOR layer. Two modes: seed 0x23 (TEA + XOR), seed 0xF3/0x36 (TEA, BF2 key schedule).

## Timing

| Parameter   | Value  | Notes   |
|------------|--------|---------|
| Symbol short | 250 µs | ±100 µs |
| Symbol long  | 500 µs | ±100 µs |
| Preamble     | 125/250 µs sub-symbols | |
| Min bits     | 128    |         |

## Encoding

Manchester; preamble uses 125/250 µs; then 250/500 µs symbols. TEA encrypt; mode 0x23 adds XOR.

## Frame Layout (128 bits)

- key1: 64 bits  
- validation: 16 bits  
- key2/rest: 48 bits  

TEA decrypt key1 (and validation); mode 0x23: XOR with BF1 key schedule; mode 0x36: TEA with BF2 key schedule. Serial/button/counter extracted from decrypted key1.

## Decoder Steps

1. **WaitEdge** — Wait for edge/preamble.
2. **CountPattern** — Count preamble pattern (125/250 µs).
3. **DecodeManchester** — Manchester decode 128 bits; TEA decrypt; apply mode (0x23 XOR or 0x36); extract fields.
4. **End** — End marker (e.g. 1000 µs); return decode.

## Encoder

Supported; preamble, Manchester 128 bits, TEA (+ XOR for 0x23).

## Frequencies

433.92 MHz.
