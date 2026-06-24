---
layout: default
---

# Ford V2 Protocol

**Rust module:** `src/protocols/ford_v2.rs`
**Reference:** `REFERENCES/ProtoPirate/protocols/ford_v2.c`

## Overview

Ford V2 uses Manchester encoding at 200/400 µs. 104 bits total (13 bytes). The frame begins with a
16-bit Manchester sync that equals `0x7FA7` (the decoder matches the *inverted* shift register against
`!0x7FA7`); those two sync bytes `0x7F 0xA7` head the 13-byte buffer. Decoded data bits are inverted
before packing (`data_bit = !bit`). There is no CRC — structure is validated by the two sync bytes plus
a known button code, so Ford V2 will not false-match.

The Manchester decoder uses Flipper's transition table (same table as Ford V0), with Ford V2 polarity
`level ? High : Low`.

## Timing

| Parameter | Value  | Notes                       |
|-----------|--------|-----------------------------|
| Short     | 200 µs | ±260 µs (te_delta)          |
| Long      | 400 µs | ±260 µs                     |
| Min bits  | 104    | 13 bytes                    |
| Preamble  | ≥64 short pulses | before the sync   |

## Frame Layout (104 bits / 13 bytes)

- **bytes 0–1:** sync `0x7F 0xA7`
- **bytes 2–5:** serial (32-bit, big-endian)
- **byte 6:** button — valid codes `0x10` (Lock), `0x11` (Unlock), `0x13` (Trunk), `0x14` (Panic), `0x15`
- **bytes 7–8 + byte 9 MSB:** counter = `((b7 & 0x7F) << 9) | (b8 << 1) | (b9 >> 7)`; byte 7 MSB carries a button parity bit (refreshed by the encoder)
- **bytes 9–12:** rolling/hop tail (stashed in `extra` so the encoder can rebuild the full frame)

The exported `data` (Key) word is the top 8 bytes; bytes 8–12 (40 bits) are carried in `extra`.

## RF

- **Encoding:** Manchester (Ford V2 polarity `level ? High : Low`; decoded bits inverted)
- **RF modulation:** FM (per ProtoPirate `SubGhzProtocolFlag_FM`)
- **Encryption:** none — validated by sync bytes + valid button code
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder Steps

1. **Reset** → a short pulse (~200 µs) begins the preamble.
2. **Preamble** — count short pulses; after ≥64 shorts a long LOW pulse enters Sync.
3. **Sync** — Manchester-decode bits into a 16-bit shift register; when it matches `!0x7FA7`, prime the two sync bytes and enter Data.
4. **Data** — Manchester events feed the state machine; decoded bits are inverted and packed MSB-first into 13 bytes. At 104 bits the sync bytes and button are validated and the frame committed. A non-short/long pulse (gap) ends the attempt.

## Encoder

Supported (matches `subghz_protocol_encoder_ford_v2`). Rebuilds the 13-byte frame, applies the requested
button, refreshes byte-7 parity, then emits 6 bursts of a 70-pair preamble + sync + 104 Manchester bits
(encoder short 240 µs), separated by ~16 ms inter-burst gaps.

## Validation

Verified by an encode→decode round-trip / synthetic-frame check (no local capture available).
