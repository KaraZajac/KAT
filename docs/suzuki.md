# Suzuki Protocol

**Rust module:** `src/protocols/suzuki.rs`  
**Reference:** `REFERENCES/ProtoPirate/protocols/suzuki.c`

## Overview

Suzuki uses PWM: 250 µs HIGH = 0, 500 µs HIGH = 1; LOW 250 µs after each bit. 64 bits total. Preamble: 350 short HIGH / short LOW pairs; 2000 µs gap at end. Field layout: serial = (data_high&0xFFF)<<16 | data_low>>16; btn = (data_low>>12)&0xF; cnt = (data_high<<4)>>16.

## Timing

| Parameter   | Value   | Notes   |
|------------|---------|---------|
| Short (0)  | 250 µs  | ±99 µs  |
| Long (1)   | 500 µs  | ±99 µs  |
| Preamble   | 350 pairs |       |
| Gap        | 2000 µs | ±399 µs |
| Min bits   | 64      |         |

## Encoding

PWM: 250 µs HIGH = 0, 500 µs HIGH = 1; LOW 250 µs after each bit.

## Frame Layout (64 bits)

- serial: (data_high & 0xFFF) << 16 | data_low >> 16  
- button: (data_low >> 12) & 0xF  
- counter: (data_high << 4) >> 16  

## Decoder Steps

1. **Reset** — Wait for short pulse (preamble).
2. **CountPreamble** — Count 350 short pairs; on 2000 µs gap → DecodeData.
3. **DecodeData** — Short = 0, long = 1; at 64 bits parse fields and return.

## Encoder

Supported; 350 preamble pairs, gap, 64 PWM bits.

## Frequencies

433.92 MHz.
