---
layout: default
---

# Renault V3 Protocol (not registered)

**Rust module:** `src/protocols/renault_v3.rs`
**Reference:** `FoxFW lib/subghz/protocols/renault_v3.c` (`"Renault V3"`)

## Status

Ported and tested, but **not in the registry**. Its NRZ quantiser reads some of BMW's own 128-bit frames
(BMW's 62 µs chips fall inside Renault V3's 52 ± 22 µs short window) as a Renault V3 frame, and it completes
on an earlier pulse than BMW does, so BMW never decodes. KAT's `bmw_frames_are_never_claimed` test (192
generated BMW frames) fails for one of them. No real Renault V3 capture exists to show a safe gate, so the
decoder stays out like the other unverified ports (Audi, Hyundai, Mazda Infinity). To enable it, add
`Box::new(renault_v3::RenaultV3Decoder::new())` to the registry; a tighter gate needs a capture.

## Overview

Renault Megane fobs, 2-FSK (the C also flags AM) at 433.92 or 315 MHz. A frame of 64 to 160 bits behind a
preamble of short pulses: `[sync : 16][serial : 16][encrypted remainder]`, with sync `0xC969` (type A) or
`0xCB2D` (type B). Only the 16-bit serial is readable; button and counter are inside the encrypted part.
No checksum. Different from [Renault V2](renault_v2.md) and from the Hitag2-style
[Renault V1](renault_v1.md). Decode-only.

## Timing and bit engine

| Parameter | Value  | Notes                  |
|-----------|--------|------------------------|
| Short     | 52 µs  | ±22 µs; one NRZ bit    |
| Frame     | 64..160 bits | after at least 10 SHORT preamble pulses |

After at least 10 short pulses (any level) the first non-short pulse starts the data. Every pulse then adds
`round(duration / 52)` bits of its level; a pulse longer than 416 µs or rounding to 0, or 250 collected bits,
end the frame.

## Gating

At least 64 bits, an exact sync A or B found, 64 to 160 bits from the sync on, and a serial that is not
`0x0000`, `0xFFFF`, `0x5555` or `0xAAAA`.

## Result

`data` = the first 64 frame bits, `data_count_bit` = the frame length, `serial` = the 16-bit serial,
`extra` = frame bits 64 to 127. Deviation: the C also keeps bits 128 to 159 (`Frame`); `DecodedSignal` has no
room and they are encrypted. No key-file support (the C's own `deserialize` accepts only 64-bit keys, so it
cannot reload what it saves for longer frames).

## RF

- **Encoding:** NRZ, **RF:** FM (the C flags AM and FM), **Encryption:** encrypted
- **Frequencies:** 433.92 MHz, 315 MHz

## Validation

No capture exists. Against the FoxFW C decoder (host oracle) on made-up frames: types A (80 bits) and B
(64 bits) and a 160-bit frame give identical `Key`, `Bit`, `Serial`, `Variant` and `Frame`; a degenerate
serial, a wrong sync, a 63-bit frame, a 161-bit frame and an 8-pulse preamble are rejected by both. Those
vectors are the unit tests.
