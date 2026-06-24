---
layout: default
---

# Toyota Protocol

**Rust module:** `src/protocols/toyota.rs`
**Reference:** `Flipper-ARF lib/subghz/protocols/toyota.c`

## Overview

Toyota / Lexus KeeLoq keyfobs, a dual-variant decoder. Two variants are detected from the first HIGH
pulse width (threshold 310 µs):

- **Variant A** (Corolla / 433.92 MHz) — PWM pairs: LS (long HIGH + short LOW) = bit 0, SL (short HIGH +
  long LOW) = bit 1. Preamble = repeated short-short pairs; first non-SS pair is the first data bit.
  Frame = 68 bits.
- **Variant B** (Tundra / 315 MHz) — NRZ: each individual pulse encodes one bit by a midpoint classifier
  (≤287 µs → 0, >287 µs → 1). Preamble = short-HIGH/long-LOW pairs terminated by a sync gap (LOW between
  1500 and 2600 µs). Frame = 67 bits.

KeeLoq hopping: the hop field is left encrypted (the reference does not decrypt or run a CRC). Emission
is gated tightly (exact frame bit count + structural preamble/sync + non-zero serial) so it does not
false-match the other KeeLoq-PWM protocols. The shared 433 MHz KeeLoq-PWM air encoding means a Variant-A
frame that also satisfies Kia V3/V4's 68-bit/CRC4 structure is claimed by Kia V3/V4 first (earlier in the
registry); Toyota uniquely claims 60-bit frames and Variant-B NRZ at 315 MHz.

## Timing

| Parameter        | Value  | Notes                       |
|------------------|--------|-----------------------------|
| A short          | 400 µs | ±175 µs (Variant A)         |
| A long           | 800 µs | ±175 µs                     |
| B short          | 200 µs | ±120 µs (Variant B preamble)|
| B long           | 390 µs | ±120 µs                     |
| B NRZ midpoint   | 287 µs | ≤ → 0, > → 1                |
| B sync gap       | 1500–2600 µs |                       |
| Min bits         | 60     | A frame = 68, B frame = 67  |
| Variant threshold| 310 µs | first HIGH: < B, ≥ A         |

## Frame Layout

`data = (hop << 32) | (serial << 4) | button` (matches the reference `generic.data`):

- **hop:** 32-bit KeeLoq ciphertext (left encrypted)
- **serial:** 28-bit
- **button:** 4-bit

## RF

- **Encoding:** Variant A = PWM pairs; Variant B = NRZ
- **RF modulation:** AM/OOK
- **Encryption:** KeeLoq hop left encrypted (no key / no CRC); a fully-structured frame of the exact bit count with non-zero serial is the validity criterion
- **Frequencies:** 433.92 MHz (Variant A), 315 MHz (Variant B)

## Decoder Steps

1. **Reset** — detect the variant from the first SHORT HIGH (<310 µs → B, ≥310 µs → A).
2. **Variant A** — PreambleA (count SS pairs) → DataA (decode LS/SL pairs, cap at 68 bits, emit on the terminating gap so Kia wins shared frames by registry order).
3. **Variant B** — PreambleB (short-HIGH/long-LOW pairs) → on the sync gap → DataB (each pulse is one NRZ bit; emit at 67 bits or on an end gap).

## Encoder

Not supported (the reference `encoder` field is NULL). Decode-only.

## Validation

Decodes REAL IMPORTS captures (IMPORTS Toyota + Lexus Camry, NRZ Variant B). Unit tests also cover
synthetic Variant A and Variant B frames and rejection of an all-zero serial.
