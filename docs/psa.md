---
layout: default
---

# PSA (Peugeot/Citroen) Protocol

**Rust module:** `src/protocols/psa.rs`
**Reference:** `REFERENCES/ProtoPirate/protocols/psa.c`

## Overview

PSA uses Manchester encoding with dual preamble pattern support. 128 bits total: key1 (64) + validation (16) + key2/rest (48). Modified TEA (XTEA-like) with dynamic key selection (`key[sum & 3]` and `key[(sum >> 11) & 3]`). Two decode modes: mode 0x23 (direct XOR decrypt with checksum validation) and mode 0x36 (TEA with BF1/BF2 key schedules). Brute-force decryption for mode 0x36 is deferred/partial.

## Timing

| Parameter    | Value  | Notes   |
|-------------|--------|---------|
| Symbol short | 250 us | +/-100 us |
| Symbol long  | 500 us | +/-100 us |
| Preamble P1  | 250 us sub-symbols | Pattern 1 |
| Preamble P2  | 125 us sub-symbols | Pattern 2 |
| End marker   | 1000 us (P1), 500 us (P2) | |
| Min bits     | 128    |         |

## Dual Preamble Patterns

- **Pattern 1 (250 us):** Preamble uses 250 us sub-symbols; Manchester decode at 250/500 us. Threshold: 70+ transitions.
- **Pattern 2 (125 us):** Preamble uses 125 us sub-symbols; Manchester decode at 125/250 us. Threshold: 69+ transitions.

Pattern type is auto-detected from the first preamble pulse duration.

## Frame Layout (128 bits)

- key1: 64 bits
- validation: 16 bits
- key2/rest: 48 bits

## Encryption

### Modified TEA (XTEA-like)

Uses dynamic key word selection per round:
- Encrypt: `k_idx1 = sum & 3`, then `sum += DELTA`, then `k_idx2 = (sum >> 11) & 3`
- Decrypt: `k_idx2 = (sum >> 11) & 3`, then `sum -= DELTA`, then `k_idx1 = sum & 3`
- Round function: `(key[k_idx] + sum) ^ (((v >> 5) ^ (v << 4)) + v)`

### Mode 0x23 (Direct XOR)

1. Setup byte buffer from key1/key2 (little-endian unpack).
2. Calculate checksum over buffer[2..8] (nibble sum * 16).
3. Validate: `(checksum ^ key2_high_byte) & 0xF0 == 0`.
4. XOR decrypt using `psa_copy_reverse` byte reordering.
5. Extract: serial (24-bit), counter (16-bit), button (4-bit), CRC.

### Mode 0x36 (TEA Brute-Force)

Direct TEA decrypt attempted with BF1 and BF2 key schedules. Full brute-force (16M iterations per schedule) is available in ProtoPirate but not fully ported to KAT.

Key schedules:
- BF1: `[0x4A434915, 0xD6743C2B, 0x1F29D308, 0xE6B79A64]`
- BF2: `[0x4039C240, 0xEDA92CAB, 0x4306C02A, 0x02192A04]`

## Decoder Steps

1. **WaitEdge (State0)** -- Detect preamble pattern type from first HIGH pulse (250 us -> Pattern 1, 125 us -> Pattern 2).
2. **CountPattern250 (State1)** -- Count 250 us preamble pairs; on long pulse with count > 70, transition to Manchester decode.
3. **DecodeManchester250 (State2)** -- Manchester decode at 250/500 us. End on 1000 us marker or 121+ bits.
4. **CountPattern125 (State3)** -- Count 125 us preamble pairs; on 250 us pulse with count >= 69, transition to Manchester decode.
5. **DecodeManchester125 (State4)** -- Manchester decode at 125/250 us. End on 500 us marker or 121+ bits.

## Encoder

Supported (mode 0x23 only). XOR encrypt, then modified TEA encrypt with BF1 key schedule. Preamble: 70 cycles of 125/125 us + 250/250 us sync. Manchester encoded key1 (64 bits) + validation (16 bits). End marker: 1000 us LOW.

## Frequencies

433.92 MHz.
