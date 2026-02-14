# Kia V6 Protocol

**Rust module:** `src/protocols/kia_v6.rs`  
**Reference:** `REFERENCES/ProtoPirate/protocols/kia_v6.c`

## Overview

Kia V6 uses Manchester encoding at 200/400 µs (level convention inverted vs Flipper). 144 bits in three parts: part1 (64) + part2 (64) + part3 (16); each part stored inverted. Long preamble of 601 pairs; sync bits 1,1,0,1 then data. AES-128 decryption with key derived from KIA V6 A and B keystores (types 11 and 12) and XOR masks.

## Timing

| Parameter | Value  | Notes   |
|-----------|--------|---------|
| Short     | 200 µs | ±100 µs |
| Long      | 400 µs | ±100 µs |
| Preamble  | 601 pairs |    |
| Min bits  | 144    |         |

## Encoding

Manchester (event mapping 0/2/4/6 for level convention). Three 64/64/16-bit segments, each inverted when stored.

## Frame Layout (144 bits)

- Part1: 64 bits (inverted)  
- Part2: 64 bits (inverted)  
- Part3: 16 bits (inverted)  

AES-128 decrypt with key = f(keystore_a, keystore_b, XOR_MASK_LOW, XOR_MASK_HIGH). Serial/button/counter and CRC extracted after decryption.

## Decoder Steps

1. **Reset** — Wait for preamble.
2. **CheckPreamble** — Count 601 pairs; detect sync pattern 1,1,0,1.
3. **Data** — Manchester decode 144 bits; invert segments; AES-128 decrypt; validate CRC8; return fields.

## Encoder

Decode-only in KAT (no encoder in reference).

## Frequencies

433.92 MHz.

## Keystore

Requires KIA V6 A and B keys (keystore types 11 and 12: `kia_v6_a_key`, `kia_v6_b_key`). XOR masks applied to derive AES key.
