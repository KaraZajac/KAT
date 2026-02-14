# Kia V5 Protocol

**Rust module:** `src/protocols/kia_v5.rs`  
**Reference:** `REFERENCES/ProtoPirate/protocols/kia_v5.c`

## Overview

Kia V5 uses Manchester encoding at 400/800 µs with **opposite polarity** to V1/V2: level true → ShortHigh, level false → ShortLow. 64 data bits + 3-bit CRC (67 bits on air). Preamble: 40+ short/long pairs; then LONG HIGH (sync), SHORT LOW (alignment), then Manchester data. Counter is encrypted with a custom mixer cipher using the KIA V5 key (YEK); serial/button from YEK.

## Timing

| Parameter | Value  | Notes   |
|-----------|--------|---------|
| Short     | 400 µs | ±150 µs |
| Long      | 800 µs | ±150 µs |
| Min bits  | 64     | (+ 3 CRC) |

## Encoding

Manchester with V5 polarity (level ? ShortHigh : ShortLow).

## Frame Layout

- Preamble (40+ pairs) → sync (long HIGH) → alignment (short LOW) → 64 Manchester bits → 3 CRC bits.
- 64-bit key (YEK) = bit-reverse of stored value; serial/button/counter extracted from YEK; counter half is mixer-decrypted with keystore V5 key.

## Decoder Steps

1. **Reset** — Wait for preamble.
2. **CheckPreamble** — Count pairs; detect sync and alignment.
3. **Data** — Manchester decode 67 bits; extract YEK, mixer-decode counter, validate CRC.

## Encoder

Decode-only in KAT (reference has encoder under ENABLE_EMULATE_FEATURE).

## Frequencies

433.92 MHz.

## Keystore

Requires KIA V5 mixer key (keystore type 13, `kia_v5_key`). Used for mixer decryption of counter.
