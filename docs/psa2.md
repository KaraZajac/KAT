---
layout: default
---

# PSA2 Protocol

**Rust module:** `src/protocols/psa2.rs`
**Reference:** `Flipper-ARF lib/subghz/protocols/psa2.c`

## Overview

PSA2 (Peugeot/Citroën — internal name "PSA OLD") is the OLDER PSA variant, distinct from KAT's existing
`psa` (modified-TEA/XEA) decoder. Manchester encoding: 250/500 µs symbol (Pattern 1, standard rate) or
125/250 µs (Pattern 2, half rate), using the canonical Flipper `manchester_advance` table. 128-bit frame
= key1 (64 bits) + key2/validation word: the decoder collects 64 bits → key1, then 16 more (to 80 bits)
→ the 16-bit validation field / key2_low.

Crypto: TEA (Tiny Encryption Algorithm) with a dual brute-force fallback (BF1 0x23000000–0x24000000, BF2
0xF3000000–0xF4000000) and a mode23/mode36 selector, validated via a nibble checksum.

**Performance:** the live decoder only ever runs the cheap O(1) mode23 XOR path
(`direct_xor_decrypt`) per frame — it NEVER runs the TEA brute force. The bounded TEA brute force
(`decrypt_full`, ~16.7M iters per range) is ported faithfully but is reserved for a deferred-decrypt UI
action, mirroring the C exactly. Beyond the C's nibble-checksum gate, KAT adds two false-positive
suppressors required by its feed-all-decoders model: a `key2_high` precondition and a valid-button check
(PSA2 buttons are Lock=0, Unlock=1, Trunk=2 only). Emission is gated strictly on a successful,
field-bearing mode23 decrypt.

## Timing

| Parameter      | Value  | Notes                       |
|----------------|--------|-----------------------------|
| Short (P1)     | 250 µs | ±100 µs (te_delta)          |
| Long (P1)      | 500 µs | ±100 µs                     |
| Short (P2)     | 125 µs | ±50 µs (half rate)          |
| Long (P2)      | 250 µs | ±50 µs                      |
| End marker     | 1000 µs (P1) / 500 µs (P2) |          |
| Min bits       | 128    | key1 (64) + validation (16) collected |
| Preamble       | >0x46 pairs (P1) / >0x45 (P2) |       |

## Frame Layout (128 bits = key1 64-bit + key2/validation)

The decoder collects 64 bits → key1, then 16 bits → the validation field (key2_low). After the mode23 XOR
decrypt (fields from `extract_fields_mode23`):

- **serial:** `(buf[2]<<16) | (buf[3]<<8) | buf[4]` (24-bit)
- **button:** `buf[8] & 0xF` — Lock=0, Unlock=1, Trunk=2
- **counter:** `(buf[5]<<8) | buf[6]` (16-bit)
- **crc:** `buf[7]`

`data` = `(key1_high << 32) | key1_low` (64 bits).

## RF

- **Encoding:** Manchester (canonical Flipper table; standard 250/500 µs or half-rate 125/250 µs)
- **RF modulation:** AM (OOK)
- **Encryption:** TEA + nibble-checksum-gated mode23 XOR path; brute-force fallback reserved for offline decrypt
- **Frequencies:** 433.92 MHz

## Decoder Steps

1. **WaitEdge (State0)** — detect the preamble rate (250 µs → Pattern 1, 125 µs → Pattern 2).
2. **CountPattern250/125 (State1/3)** — count preamble pulses past the threshold, then a long pulse enters the Manchester decode state.
3. **DecodeManchester250/125 (State2/4)** — Manchester-decode to 80 bits (key1 + validation), detect end-of-packet, then `finalize_frame` runs the cheap mode23 XOR gate and emits only on a successful field-bearing decrypt.

## Encoder

Supported (mode23 path). Increments the counter, builds key1/validation via `encode_mode23` (inverse XOR
stage + nibble checksum), then emits an 80-pair preamble, a sync, 64 key1 bits + 16 validation bits
MSB-first, and an end burst.

## Validation

Decodes REAL IMPORTS captures (IMPORTS/GROUPE PSA, serial `0x99EB25`). Unit tests also cover the TEA
round trip, the mode23 encode→decode round trip, the nibble checksum vs. the reference, and the emitted
Manchester frame length.
