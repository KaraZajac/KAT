---
layout: default
---

# Chrysler V0 Protocol

**Rust module:** `src/protocols/chrysler_v0.rs`
**Reference:** `REFERENCES/ProtoPirate/protocols/chrysler_v0.c`

## Overview

Chrysler/Dodge/Jeep keyfobs. PWM with a short HIGH pulse and two long-LOW symbols: a "1" payload bit is
HIGH ≈600 µs + LOW ≈3400 µs; a "0" payload bit is HIGH ≈300 µs + LOW ≈3700 µs. ~24 preamble pairs
(short HIGH + long_b LOW) precede each frame. 80-bit frame: the first 64 bits are `decode_data` (payload
bytes 0–7), the last 16 bits are `data_2` (payload bytes 8,9). The 80-bit frame exceeds u64, so `data`
reports the most-significant 64 bits and `data_count_bit = 80`.

Crypto is a proprietary seed-XOR (ported exactly from the reference): `seed = reverse6(key[0] >> 2)` (a
6-bit reversed counter); `transform_block` XORs all 9 transformed bytes with `xor_table[seed & 0x0F]`,
with an extra nibble flip when the Lock button is set. Dual payload: A (seed even, carries serial +
counter) / B (seed odd, carries serial). The frame is gated on a structural `check_ok`, so it does not
false-match; `crc_valid` reflects that check.

## Timing

| Parameter   | Value   | Notes                  |
|-------------|---------|------------------------|
| Short HIGH  | 300 µs  | ±150 µs (te_delta)     |
| One-short HIGH | 600 µs | "1" bit HIGH width    |
| Long LOW A  | 3400 µs | ±400 µs (long_delta)   |
| Long LOW B  | 3700 µs | ±400 µs                |
| Gap         | 8000 µs | te_gap                 |
| Frame gap   | 15600 µs|                        |
| Min bits    | 80      |                        |
| Preamble    | 24 pairs|                        |

## Frame Layout (80 bits / 10 payload bytes)

- **byte 0:** `(reverse6(counter) << 2) | header_low2` — the seed/counter byte
- **bytes 1–9:** transformed (XOR-masked) payload; after `transform_block`:
  - Payload A (even seed): serial = decoded[0..3], rolling counter
  - Payload B (odd seed): serial = decoded[0..2] + decoded[7]
- Button: Lock=1, Unlock=2 (derived from structural relationships between key bytes)

## RF

- **Encoding:** PWM (short HIGH + dual long LOW)
- **RF modulation:** AM
- **Encryption:** proprietary seed-XOR (`transform_block` + `xor_table`); gated on structural `check_ok`
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder Steps

1. **Reset** — a short HIGH pulse begins the seek.
2. **Seek** — count preamble pairs (short HIGH + long LOW); after >15 pairs, a non-short HIGH transitions to Data.
3. **Data** — classify each HIGH/LOW pair into a payload bit (`bit ^ 1`); collect 64 bits into `decode_data` then 16 into `data_2`. At 80 bits (or on a terminating gap with >0x4F bits) run `decode_packet`, check `check_ok`, and emit.

## Encoder

Supported (ENABLE_EMULATE_FEATURE). Rebuilds the 10-byte payload, re-derives seed/plaintext, applies the
requested button, then emits preamble + dual 80-bit PWM frames (A and B) separated by frame gaps.

## Validation

Verified by an encode→decode round-trip / synthetic-frame check (no local capture available). Unit tests
cover `reverse6` involution, `transform_block` invertibility, and a full payload-A round trip.
