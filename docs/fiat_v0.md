# Fiat V0 Protocol

**Rust module:** `src/protocols/fiat_v0.rs`  
**Reference:** `REFERENCES/ProtoPirate/protocols/fiat_v0.c` (older reference)

## Overview

Fiat V0 uses differential Manchester. Preamble: count short pulses (HIGH or LOW, 200 µs ±100); when count ≥ 150 (0x96), accept 800 µs LOW gap and enter Data. Data: 64 bits (serial = data_low, cnt = data_high) then 7 more bits; complete when bit_count > 0x46 with btn = (data_low << 1) | 1; 71 bits total. Encoder: 150 preamble pairs, last LOW = 800 µs gap; 64 data bits then 6 button bits (btn >> 1); end marker te_short×8 LOW.

## Timing

| Parameter   | Value  | Notes        |
|------------|--------|--------------|
| Short      | 200 µs | ±100 µs      |
| Long       | 400 µs | ±100 µs      |
| Preamble   | ≥150 short pulses | |
| Gap        | 800 µs | ±100 µs      |
| Min bits   | 71     |              |

## Encoding

Differential Manchester; 6 button bits sent as btn >> 1; end marker 200×8 µs LOW.

## Frame Layout (71 bits)

- 64 bits: serial (data_low), counter (data_high).
- 7 bits more; button = (data_low << 1) | 1 (decoded).

## Decoder Steps

1. **Reset** — Count short (200 µs) HIGH or LOW; when count ≥ 150 and LOW gap ~800 µs → Data.
2. **Data** — Manchester decode; at > 0x46 bits set btn = (data_low << 1) | 1, return 71-bit decode.

## Encoder

Supported; 3 bursts, 25 ms inter-burst gap; 150 preamble pairs; 800 µs gap; 64 + 6 button bits; end marker.

## Frequencies

433.92 MHz.
