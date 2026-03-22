---
layout: default
---

# Fiat V1 Protocol (Magneti Marelli BSI / PCF7946)

**Rust module:** `src/protocols/fiat_v1.rs`
**Reference:** `REFERENCES/ProtoPirate/protocols/fiat_v1.c`

## Overview

Fiat V1 is the Magneti Marelli BSI keyfob protocol (PCF7946), found on Fiat Panda, Grande Punto, and possibly other Fiat/Lancia/Alfa ~2003-2012. Uses Manchester encoding with auto-detected timing from preamble pulse averaging. Two timing variants: Type A (~260 us, e.g. Panda) and Type B (~100 us, e.g. Grande Punto). This is a different protocol from Fiat V0.

## Timing

| Parameter | Default | Notes |
|-----------|---------|-------|
| Short     | 260 us  | Auto-detected from preamble |
| Long      | 520 us  | 2x detected TE |
| Delta     | 80 us   | Minimum 30 us |
| Preamble  | 80+ pulses | 50-350 us range |
| Min bits  | 80      | Max 104 bits |

### Auto-Detection

Preamble pulses (50-350 us) are accumulated. After 80+ pulses, `te_detected = te_sum / te_count` becomes the reference TE. Type A/B boundary is at 180 us.

## Frame Layout (80-104 bits = 10-13 bytes)

| Bytes | Content |
|-------|---------|
| 0-1   | Preamble residue (0xFFFF/0xFFFC) |
| 2-5   | Serial (32 bits) |
| 6     | Button:4 \| Epoch:4 |
| 7     | Counter:5 \| Scramble:2 \| Fixed:1 |
| 8-12  | Encrypted payload (40 bits) |

## Decoder Steps

1. **Reset** -- Wait for HIGH pulse in 50-350 us range. Also detect retransmission gaps (>5000 us).
2. **Preamble** -- Count pulses, accumulate for TE averaging. On gap >= te_detected * 4, transition to Sync.
3. **Sync** -- Expect HIGH sync pulse of te_detected * 4 to te_detected * 12 duration.
4. **Data** -- Manchester decode up to 104 bits into 13-byte raw buffer.
5. **RetxSync** -- After gap >5000 us, look for sync pulse 400-2800 us (retransmission).

## Buttons

| Code | Name   |
|------|--------|
| 0x7  | Lock   |
| 0xB  | Unlock |
| 0xD  | Trunk  |

## Encoder

Not supported (decode-only).

## Frequencies

433.92 MHz.
