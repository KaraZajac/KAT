---
layout: default
---

# Mitsubishi V0 Protocol

**Rust module:** `src/protocols/mitsubishi_v0.rs`
**Reference:** `REFERENCES/ProtoPirate/protocols/mitsubishi_v0.c`

## Overview

Mitsubishi V0 uses PWM encoding at 250/500 us. 96-bit frame (12 bytes). Level-aware: HIGH pulses are saved, LOW pulses complete the pair. After collection, data is unscrambled via bitwise NOT + counter-derived XOR mask.

## Timing

| Parameter | Value  | Notes        |
|-----------|--------|--------------|
| Short     | 250 us | +/-100 us    |
| Long      | 500 us | +/-100 us    |
| Min bits  | 80     |              |
| Frame     | 96 bits| 12 bytes     |

## PWM Bit Encoding

| Pair (HIGH, LOW) | Bit |
|------------------|-----|
| Short HIGH + Long LOW  | 1 |
| Long HIGH + Short LOW  | 0 |

Bits collected MSB-first into a 12-byte buffer.

## Frame Layout (96 bits)

After unscrambling:
- Bytes 0-3: Serial (32 bits, big-endian)
- Bytes 4-5: Counter (16 bits)
- Byte 6: Button (8 bits)
- Bytes 7-11: Remaining payload

## Unscramble Algorithm

1. Bitwise NOT the first 8 bytes.
2. Extract counter from bytes [4..5]: `hi = (counter >> 8) & 0xFF`, `lo = counter & 0xFF`.
3. Compute masks: `mask1 = (hi & 0xAA) | (lo & 0x55)`, `mask2 = (lo & 0xAA) | (hi & 0x55)`, `mask3 = mask1 ^ mask2`.
4. XOR bytes [0..5] with mask3.

## Decoder Steps

1. **Reset** -- Wait for HIGH pulse; save duration.
2. **DataSave** -- On HIGH: save duration. On LOW (unexpected): reset.
3. **DataCheck** -- On LOW: decode pair. If 96 bits collected, unscramble and return. Otherwise continue.

## Encoder

Not supported (decode-only).

## Frequencies

868.35 MHz.
