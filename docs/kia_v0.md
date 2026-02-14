# Kia V0 Protocol

**Rust module:** `src/protocols/kia_v0.rs`  
**Reference:** `REFERENCES/ProtoPirate/protocols/kia_v0.c`

## Overview

Kia V0 is a PWM keyfob protocol: short pulse = 0, long pulse = 1. No Manchester encoding. 61 bits per frame (1 sync bit + 60 data bits). CRC8 over bits 8–55; polynomial 0x7F, init 0x00.

## Timing

| Parameter   | Value  | Notes              |
|------------|--------|--------------------|
| Short (0)  | 250 µs | ±100 µs (te_delta) |
| Long (1)   | 500 µs | ±100 µs            |
| Min bits   | 61     |                    |

## Frame Layout (61 bits)

- **Sync:** 1 bit (the long-long pattern also counts as first data bit = 1).
- **Data:** 60 bits MSB first:
  - Bits 56–59: 4-bit prefix (often preserved from capture).
  - Bits 40–55: 16-bit counter.
  - Bits 12–39: 28-bit serial.
  - Bits 8–11: 4-bit button.
  - Bits 0–7: 8-bit CRC (over bits 8–55, 6 bytes).

## Decoder Steps

1. **Reset** — Wait for short HIGH (250 µs ±100).
2. **CheckPreamble** — Count preamble: alternating short pulses; on LOW, if short–short pair then `header_count++`; if long–long and `header_count > 15` → go to SaveDuration, add first bit 1.
3. **SaveDuration** — On HIGH: if duration ≥ 500 + 200 µs (end marker), check `decode_count_bit == 61` and return decode; else store duration and go to CheckDuration. On LOW → Reset.
4. **CheckDuration** — On LOW: short–short → add bit 0, back to SaveDuration; long–long → add bit 1, back to SaveDuration; else Reset.

## Encoder

- 2 bursts; inter-burst gap 25 000 µs.
- 32 alternating short (250 µs) preamble pairs.
- Sync: long HIGH, long LOW (500 µs each).
- Data: 59 bits sent (mask 1ULL << (58 - bit_num)), i.e. bits 58 down to 0 (per reference).
- End marker: long × 2 (1000 µs HIGH).

## Frequencies

433.92 MHz.
