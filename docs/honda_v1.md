---
layout: default
---

# Honda V1 Protocol

**Rust module:** `src/protocols/honda_v1.rs`
**Reference:** `REFERENCES/ProtoPirate/protocols/honda_v1.c`

## Overview

Honda/Acura fixed-code keyfobs — a DIFFERENT protocol from `honda_static`. The on-wire frame carries 68
bits: 64-bit data + a 4-bit CRC-fold nibble. Encoding is short/long pulse PWM, glued together by a
"pending bit" timing accumulator before classification: sub-`te_delta` runts are summed into `pending`;
a HIGH level extends the running HIGH pulse; a LOW flushes the accumulated HIGH (if it reached
`te_short_min`) as a synthetic symbol, then classifies the LOW.

Validation is a button-code table (Unlock=0, Lock=8, Trunk=9, Panic=10) plus a CRC-fold checksum.
Emission is gated on the button being valid; `crc_valid` reflects whether the received CRC nibble matches
either wire-order checksum. The strong button gate keeps Honda V1 from false-matching.

## Timing

| Parameter   | Value   | Notes                       |
|-------------|---------|-----------------------------|
| Short       | 1000 µs | ±400 µs (te_delta)          |
| Long        | 2000 µs | ±400 µs                     |
| Short min   | 600 µs  | flush threshold for HIGH    |
| End gap     | 3500 µs | >te_end commits the frame   |
| Min bits    | 68      |                             |

## Frame Layout (68 bits = 64-bit data + 4-bit CRC nibble)

After the end gap, `commit` left-shifts the 12-byte bit buffer to drop leading preamble leakage and align
the trailing 68-bit frame. `data` = first 8 bytes (64 bits); the CRC nibble = byte 8's high nibble.

- **data[63:36]:** serial (28-bit)
- **data[31:28]:** button (nibble) — Unlock=0, Lock=8, Trunk=9, Panic=10
- **data[15:0]:** counter (16-bit)
- **CRC nibble:** CRC-fold checksum (`honda_v1_checksum*`), validated against either wire-order value

## RF

- **Encoding:** short/long pulse PWM with pending-bit timing accumulation
- **RF modulation:** AM (OOK)
- **Encryption:** none — gated on the button-code table + CRC-fold checksum
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder Steps

1. **feed** — accumulate sub-`te_delta` runts into `pending`; flush HIGH pulses and classify LOW pulses into symbols.
2. **symbol** — Reset → Preamble (short/long pulses, needs a long + >5 count) → Data.
3. **Data** — pending-bit accumulation: a short pulse toggles `data_pending` and emits a bit when paired; a long pulse emits directly. After a >te_end gap, commit: align the 68-bit frame, validate the button, and emit.

## Encoder

Supported (ENABLE_EMULATE_FEATURE). Builds the 64-bit key from serial/button/counter via the button code
table, then emits a 180-element short-pair preamble + 4 PWM frames (2 per checksum wire value).

## Validation

Verified by an encode→decode round-trip / synthetic-frame check (no local capture available). Unit tests
cover the key/field round trip, full encode→decode, and the button-validity mask.
