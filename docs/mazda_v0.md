---
layout: default
---

# Mazda V0 Protocol

**Rust module:** `src/protocols/mazda_v0.rs`
**Reference:** `REFERENCES/ProtoPirate/protocols/mazda_v0.c`

## Overview

Mazda V0 uses a custom pair-based encoding (not standard Manchester). The `level` parameter is ignored; the decoder processes raw duration pairs. Preamble: 13+ short/short pairs followed by a short+long transition into data. Data bits are collected using a prev_state tracker with inverted polarity, then XOR-deobfuscated, checksum-validated, and parsed.

## Timing

| Parameter | Value  | Notes        |
|-----------|--------|--------------|
| Short     | 250 us | +/-100 us    |
| Long      | 500 us | +/-100 us    |
| Min bits  | 64     |              |
| Completion| 80-105 bits |         |

## Frame Layout (64 bits after deobfuscation)

Raw data collects into a 14-byte buffer. First byte is discarded (sync); bytes [1..9] form the 8-byte data frame.

After XOR deobfuscation:
- Bytes 0-3: Serial (32 bits)
- Byte 4: Button (8 bits)
- Bytes 5-6: Counter (16 bits)
- Byte 7: Checksum (additive sum of bytes 0-6)

## Pair-Based Bit Decoding

| Pair (te_last, duration) | Action |
|--------------------------|--------|
| Long + Short   | Collect bit(0), bit(1), prev_state=1 |
| Short + Long   | Collect bit(1), prev_state=0 |
| Short + Short  | Collect bit(prev_state) |
| Long + Long    | Collect bit(0), bit(1), prev_state=0 |

Bit collection uses inverted polarity: state_bit=0 stores a 1.

## XOR Deobfuscation

1. Compute parity of data[7] (XOR fold to single bit).
2. If parity is odd: XOR bytes [0..6] with data[6] as mask.
3. If parity is even: XOR bytes [0..5] with data[5] as mask, also XOR data[6] with data[5].
4. Bit interleave swap bytes 5 and 6: swap even/odd bit positions between them.

## Decoder Steps

1. **Reset** -- Wait for short pulse; save and go to PreambleCheck.
2. **PreambleSave** -- Save duration, go to PreambleCheck.
3. **PreambleCheck** -- Count short+short pairs; on short+long with count >= 13, start data.
4. **DataSave** -- Save duration, go to DataCheck.
5. **DataCheck** -- Process pair; if valid continue; if invalid, check completion (80-105 bits + checksum).

## Buttons

| Code | Name   |
|------|--------|
| 0x10 | Lock   |
| 0x20 | Unlock |
| 0x40 | Trunk  |

## Encoder

Not supported (decode-only).

## Frequencies

433.92 MHz.
