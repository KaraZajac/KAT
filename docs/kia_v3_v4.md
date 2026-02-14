# Kia V3/V4 Protocol

**Rust module:** `src/protocols/kia_v3_v4.rs`  
**Reference:** `REFERENCES/ProtoPirate/protocols/kia_v3_v4.c`

## Overview

Kia V3 and V4 use PWM (short = 0, long = 1) with KeeLoq encryption. 68 bits: 8 bytes encrypted + 4 bits CRC. Short preamble of 16 pairs; sync 1200 µs (V4: long HIGH, V3: long LOW). KeeLoq uses the KIA manufacturer key from the keystore (type 10).

## Timing

| Parameter   | Value   | Notes   |
|------------|---------|---------|
| Short (0)  | 400 µs  | ±150 µs |
| Long (1)   | 800 µs  | ±150 µs |
| Sync       | 1200 µs |         |
| Min bits   | 68      |         |

## Frame Layout (68 bits)

- Preamble: 16 short/long pairs.
- Sync: 1200 µs (polarity distinguishes V3 vs V4).
- 64 raw bits (8 bytes) then 4 CRC bits.
- 64 bits are KeeLoq-encrypted; decrypt with KIA MF key to get serial/button/counter.

## Decoder Steps

1. **Reset** — Wait for preamble.
2. **CheckPreamble** — Count 16 pairs; detect sync polarity (V3 vs V4).
3. **CollectRawBits** — Collect 68 bits (64 + 4 CRC); KeeLoq-decrypt 64 bits, validate CRC4, extract fields.

## Encoder

Supported; 3 bursts, 10 s inter-burst gap; preamble, sync, encrypted payload + CRC.

## Frequencies

433.92 MHz.

## Keystore

Requires KIA manufacturer key (keystore type 10, `kia_mf_key`). Loaded from embedded blob or keystore.ini.
