---
layout: default
---

# Renault/xHorse V4 Protocol

**Rust module:** `src/protocols/renault_v4.rs`
**Reference:** `FoxFW lib/subghz/protocols/renault_v4.c` (`"Renault/xHorse V4"`)

## Overview

The aftermarket Nissan / xHorse universal key fob, AM/OOK at 433.92 or 315 MHz. An 88-bit Manchester frame
with a visible serial and a visible 8-bit counter (+1 per press). Not one of the Hitag2-style Renault fobs
([V0](renault_v0.md), [V1](renault_v1.md)) and not the encrypted [V2](renault_v2.md). Decode-only.

## Timing

| Parameter | Value   | Notes                                      |
|-----------|---------|--------------------------------------------|
| Short     | 125 µs  | < 70 µs off                                |
| Long      | 250 µs  | < 70 µs off                                |
| Sync      | HIGH ≥ 800 µs | starts the burst                     |

Bits come from the C's own 4-state Manchester machine (events short/long x high/low; it is not the canonical
Flipper table). A pulse that is neither short nor long ends the burst and checks the bits.

## Frame (11 bytes, MSB first)

- bytes 0 to 3: serial
- byte 4: button nibble (high), pad (bits 3..2, must be `01`), counter bits 7..6 (bits 1..0)
- byte 5 bits 7..2: counter bits 5..0
- the rest is opaque; **the XOR of all 11 bytes must be `0xFF`**

The decoded bits must start with at least 10 identical preamble bits (ones, then zeros are tried), one start
bit, then 88 to 92 bits. Button nibbles accepted: `E`, `D`, `B`, `7`. A frame identical to the previous one is
not reported again until the decoder is reset.

## Result

`data` = bytes 0 to 7, `data_count_bit` = 88, `extra` = bytes 8 to 10, `serial`, `button` (nibble),
`counter` (8 bit). The XOR check passed for every frame reported, so the CRC shows OK.

## RF

- **Encoding:** Manchester, **RF:** AM, **Encryption:** rolling, clear counter
- **Frequencies:** 433.92 MHz, 315 MHz

## Key files

`Protocol: Renault/xHorse V4`, `Bit: 88`, `Key`, `Serial`, `Btn`, `Cnt`, `Data2` (bytes 8 to 10, decimal).

## Validation

No capture exists. Against the FoxFW C decoder (host oracle) on made-up frames: two valid frames (button `E`
and `7`, 12 and 14 preamble bits) give identical `Key`, `Serial`, `Btn`, `Cnt` and `Data2`. A bad XOR, a
flipped payload bit, a button nibble `C`, a wrong pad, an 87-bit frame, a 9-bit preamble and a 700 µs sync
are rejected by both. Those vectors are the unit tests.
