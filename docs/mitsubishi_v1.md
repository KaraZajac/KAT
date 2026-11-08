---
layout: default
---

# Mitsubishi V1 Protocol

**Rust module:** `src/protocols/mitsubishi_v1.rs`
**Reference:** `FoxFW lib/subghz/protocols/mitsubishi_v1.c` (no other reference has it)

## Overview

A 2-FSK Manchester rolling-code keyfob frame of 96 bits: a 32-bit sync word, a constant fix byte, a 4-bit
button, a 10-bit cleartext counter and a 42-bit encrypted hop. It is not KAT's `Mitsubishi V0` (the 96-bit PWM
pairs of Flipper-ARF, or the 61-bit Kia V0 type). Decode-only, like FoxFW (no encoder).

## Timing

| Parameter | Value  | Notes                                                        |
|-----------|--------|--------------------------------------------------------------|
| Short     | 200 µs | 100..=300 µs (delta 100)                                     |
| Long      | 400 µs | >300..=500 µs                                                |
| Glitch    | <100 µs | ignored, state kept                                          |
| Gap       | >500 µs | ends the frame; so does a 199-bit buffer                     |

Line code: a LONG pulse is one bit equal to its level; of two SHORT pulses the second is one bit equal to its
level (FoxFW's half-bit toggle).

## Frame Layout (96 bits)

| Bits   | Field                                     |
|--------|-------------------------------------------|
| 0..31  | sync `0x1E4C11A7`                         |
| 32..39 | fix byte, must be `0xE0`                  |
| 40..43 | button                                    |
| 44..53 | counter (10 bits, cleartext)              |
| 54..95 | hop (42 bits, encrypted)                  |

The decoder looks for the sync anywhere in the collected bits (56 bits must follow it), takes up to 96 bits,
needs at least 88, and reports the frame if the fix byte is `0xE0` (no CRC). A frame cut at 88..95 bits has its
hop padded with zeros on the right. `serial` shows the fix byte (the frame has no per-fob ID).

**KAT packing:** `data` = the first 64 bits (sync, fix, button, counter, 10 hop bits), `extra` = the 42-bit hop,
`data_count_bit` = 96.

Faithful quirk: a gap before 88 bits or without a sync keeps collecting, and 199 bits without a sync leave the
decoder stuck until KAT's registry resets it (it does after every hit and at the start of each capture).

## RF

- **Encoding:** Manchester
- **RF modulation:** AM and FM (FoxFW flags `AM | FM`)
- **Encryption:** rolling code (the hop); the counter is in the clear
- **Frequencies:** 315 MHz, 433.92 MHz
- **Check:** none (structural: sync + fix byte); the CRC column shows "-"

No capture of a real Mitsubishi V1 fob exists in KAT's test files; the decoder is verified against FoxFW's C
decoder on made-up frames (sync, fix-byte and length gates included).
