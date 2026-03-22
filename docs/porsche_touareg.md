---
layout: default
---

# Porsche Touareg Protocol

**Rust module:** `src/protocols/porsche_touareg.rs`
**Reference:** `REFERENCES/ProtoPirate/protocols/porsche_touareg.c`

## Overview

Porsche Touareg uses PWM encoding with very long timing (1680/3370 us). 64-bit frame with a sync preamble (at least 15 sync pulses at 3370 us). Counter is recovered via brute-force using a 24-bit rotation cipher. Originally designed for the Porsche Cayenne.

## Timing

| Parameter  | Value    | Notes        |
|------------|----------|--------------|
| Short      | 1680 us  | +/-500 us    |
| Long       | 3370 us  | +/-500 us    |
| Sync       | 3370 us  | Same as long |
| Gap        | 5930 us  | +/-500 us    |
| Sync min   | 15 pulses|              |
| Min bits   | 64       |              |

## PWM Bit Encoding

| Pair (LOW, HIGH) | Bit |
|------------------|-----|
| Short LOW + Long HIGH  | 0 |
| Long LOW + Short HIGH  | 1 |

## Frame Layout (64 bits = 8 bytes)

| Byte | Content |
|------|---------|
| pkt[0] | (button << 4) \| (frame_type & 0x07) |
| pkt[1] | serial bits [23:16] |
| pkt[2] | serial bits [15:8] |
| pkt[3] | serial bits [7:0] |
| pkt[4..7] | Encrypted counter/rolling code |

- **Serial**: 24 bits from pkt[1..3]
- **Button**: 4 bits (pkt[0] >> 4)
- **Frame type**: 3 bits (pkt[0] & 0x07): 0x02=First, 0x01=Cont, 0x04=Final

## Counter Recovery (Brute-Force)

Counter is not in plaintext. The decoder tries counter values 1-256, calling `compute_frame()` for each, and checks if computed bytes [4..7] match received bytes [4..7].

### Compute Frame Algorithm (24-bit Rotate Cipher)

1. Initialize 24-bit rotate register from serial bytes: r_h=b3, r_m=b1, r_l=b2.
2. ROTATE24: circular left shift across 3 bytes (h<-m, m<-l, l<-h).
3. Rotate 4 times + counter_low more times.
4. Compute encrypted bytes a9a/a9b/a9c from rotated values XOR'd with inverted counter bits.
5. Assemble pkt[4..7] using bitfield packing.

## Decoder Steps

1. **Reset** -- Wait for LOW pulse matching sync (3370 us).
2. **Sync** -- Count sync pulses (both HIGH and LOW). When count >= 15 and gap detected (5930 us), transition to GapHigh or GapLow.
3. **GapHigh** -- Expect HIGH gap (5930 us); init data on match.
4. **GapLow** -- Expect LOW gap (5930 us); init data on match.
5. **Data** -- Decode bit pairs (LOW saved, HIGH completes). At 64 bits, parse data and brute-force counter.

## Encoder

Not supported (decode-only).

## Frequencies

433.92 MHz, 868.35 MHz.
