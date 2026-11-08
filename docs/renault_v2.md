---
layout: default
---

# Renault V2 Protocol

**Rust module:** `src/protocols/renault_v2.rs`
**Reference:** `FoxFW lib/subghz/protocols/renault_v2.c` (`"Renault V2"`)

## Overview

Renault Clio 3 fobs, AM/OOK at 433.92 or 315 MHz. This is an **identify-only** decoder: the 82-bit frame is
a fixed 16-bit sync `0x71EA` and 66 encrypted bits, with no serial, counter, button or checksum in the clear.
It tells that a Renault V2 fob was seen and surfaces the raw bits. It is a different protocol from KAT's
Hitag2-style [Renault V0](renault_v0.md) / [Renault V1](renault_v1.md) and from [Renault V3](renault_v3.md)
and [Renault/xHorse V4](renault_v4.md). Decode-only: the C has no encoder.

## Timing

| Parameter | Value  | Notes                       |
|-----------|--------|-----------------------------|
| Short     | 230 µs | ±60 µs                      |
| Long      | 460 µs | ±60 µs                      |
| Frame     | 82 bits | `[sync 0x71EA : 16][payload : 66]` |

## Bit engine

A stateless half-bit accumulator. The first SHORT pulse starts a frame and counts as a first half. At a bit
boundary a LONG pulse is one bit equal to its level; a SHORT pairs with the next SHORT and gives one bit equal
to the level of the second. A LONG while a half bit is pending, any other duration, or 190 collected bits end
the frame.

## Gating

The frame is checked when it ends: at least 60 bits collected, the first exact `0x71EA` found in the bits, and
the bits from the sync on number 82 to 86 (the full frame, and no over-long capture).

## Result

`data` = the first 64 wire bits, `data_count_bit` = 82, `extra` = the last 18 bits (the C's `Data2`). No serial,
button or counter, and no check: the interface shows "-" for the CRC.

## RF

- **Encoding:** Manchester-like (see above), **RF:** AM, **Encryption:** encrypted (nothing readable)
- **Frequencies:** 433.92 MHz, 315 MHz

## Key files

`Protocol: Renault V2`, `Bit: 82`, `Key` = `data`, `Data2` = the last 18 bits (decimal), as the C writes them.

## Validation

No capture exists for this protocol (none in IMPORTS or the Flipper unit-test set; the whole corpus decodes
identically with and without it). The decoder was checked against the FoxFW C decoder (host oracle) with
made-up frames: two valid frames (82 and 86 bits after the sync) give identical `Key`, `Bit` and `Data2`;
a wrong sync, a 79-bit frame and an 87-bit frame are rejected by both. Those vectors are the unit tests.
