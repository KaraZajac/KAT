---
layout: default
---

# Fiat Ducato V0 Protocol

**Rust module:** `src/protocols/ducato_v0.rs`
**Reference:** `FoxFW lib/subghz/protocols/ducato_v0.c` (`"Fiat Ducato V0"`)

## Overview

Fiat Ducato (Magneti Marelli NBC250C body computer) with an NXP PCF7946A (Hitag2) fob, AM/OOK at 433.92 MHz.
A 71-bit Manchester frame behind a long preamble: a 32-bit encrypted hop, a 32-bit fixed part
(24-bit serial and 8-bit button) and a 7-bit CRC field. Not KAT's [Fiat V0](fiat_v0.md) (64 bits, 100/200 µs)
or [Fiat V1](fiat_v1.md). The counter is inside the encrypted hop. Decode-only.

## Timing

| Parameter | Value   | Notes                                  |
|-----------|---------|----------------------------------------|
| Short     | 200 µs  | ±99 µs                                 |
| Long      | 400 µs  | ±99 µs                                 |
| Preamble  | ≥ 120 SHORT pulses | the first one HIGH           |
| Sync      | 800 µs  | one pulse, any level                   |

After the sync, 71 Manchester bits are read through the PCF7946A table (`level ? ShortLow : ShortHigh`, the
canonical Flipper table, but a same-state transition just gives no bit). Durations that are neither short nor
long inside the data are ignored, as in the firmware.

## Frame (71 bits, MSB first)

`[hop : 32][fix : 32][crc : 7]`; `fix` = `serial(24) | button(8)`.

## CRC

`acc = b0&0xB3 ^ b1&0x80 ^ b2&0x83 ^ b3&0x22 ^ b5&0x73` over the big-endian key bytes (byte 5, not byte 4); the
frame is valid when `crc & 1` equals the parity of the nibble parities of `acc`. Only that one bit of the 7-bit
field is checked, so a random frame passes half the time. **As in the C a frame that fails the CRC is still
reported** (CRC FAIL in the interface); the serials `0x000000`, `0xFFFFFF`, `0x555555`, `0xAAAAAA` are dropped.

## Result

`data` = the 64-bit key (hop high, fix low), `data_count_bit` = 71, `extra` = the 7-bit CRC field, `serial`,
`button`, no counter.

## RF

- **Encoding:** Manchester, **RF:** AM, **Encryption:** HITAG2
- **Frequencies:** 433.92 MHz and 315 MHz (FoxFW flags both: `SubGhzProtocolFlag_315 | 433`)

## Key files

`Protocol: Fiat Ducato V0`, `Bit: 71`, `Key`, `Serial`, `Btn`, `Hop`, `CRC`.

## Validation

No capture exists. Against the FoxFW C decoder (host oracle) on made-up frames: a frame with a good CRC bit
and one with a bad one (both reported) give identical `Key`, `Serial`, `Btn`, `Hop` and `CRC`; a 1200 µs sync,
a 100-pulse preamble, a degenerate serial and a 70-bit frame are rejected by both. The oracle does not print
whether its CRC passed; the CRC verdict in the unit tests comes from an independent script that mirrors the
C formula. Those vectors are the unit tests.
