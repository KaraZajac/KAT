---
layout: default
---

# Ford V3 Protocol

**Rust module:** `src/protocols/ford_v3.rs`
**Reference:** `REFERENCES/ProtoPirate/protocols/ford_v3.c`

## Overview

Ford V3 uses Manchester encoding at 240/480 µs. 104 bits total (13 bytes), transmitted as plaintext —
there is no CRC or encryption. Decode-only: the ProtoPirate reference ships a NULL encoder, so KAT
does not transmit Ford V3 (Replay still works on the raw capture).

The Manchester decoder uses Flipper's `manchester_decoder.h` transition table (same table as Ford V0),
but Ford V3 maps the level the opposite way from Ford V0: `level ? High : Low`.

## Timing

| Parameter | Value  | Notes              |
|-----------|--------|--------------------|
| Short     | 240 µs | ±60 µs (te_delta)  |
| Long      | 480 µs | ±60 µs             |
| Min bits  | 104    | 13 bytes           |
| Preamble  | ≥30 short pulses | before first long |

## Frame Layout (104 bits / 13 bytes)

- **byte 0:** header
- **bytes 1–4:** serial (32-bit, big-endian)
- **byte 5:** hop/reserved
- **byte 6:** button — bit 0 set → Unlock, else Lock
- **bytes 7–8:** counter, stored bitwise-inverted (`~b[7]`, `~b[8]`)
- **bytes 9–12:** rolling/hop tail (not parsed)

## Decoder Steps

1. **Reset** — a short pulse (~240 µs) starts the preamble (count = 1).
2. **Preamble** — count short pulses; once ≥30 shorts are seen, a long pulse begins the data
   (Manchester state seeded to Mid1, first bit decoded from the long pulse).
3. **Data** — Manchester events (short/long × level) feed the state machine; bits pack MSB-first into
   13 bytes. At 104 bits the fields are parsed and the signal is emitted. A non-short/non-long pulse
   (gap) ends the frame.

## RF

- **Encoding:** Manchester
- **RF modulation:** FM (per ProtoPirate `SubGhzProtocolFlag_FM`)
- **Encryption:** none (plaintext)
- **Frequencies:** 315 MHz, 433.92 MHz

## Encoder

Not supported (reference encoder is NULL). Decode-only.
