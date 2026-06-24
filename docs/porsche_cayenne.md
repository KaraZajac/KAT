---
layout: default
---

# Porsche Cayenne Protocol

**Rust module:** `src/protocols/porsche_cayenne.rs`
**Reference:** `Flipper-ARF lib/subghz/protocols/porsche_cayenne.c`

## Overview

Porsche keyfobs (internal firmware header name "Porsche AG"). PWM at 1680/3370 µs: SHORT LOW + LONG HIGH
= bit 0, LONG LOW + SHORT HIGH = bit 1. 64-bit MSB-first frame, preceded by a 73-pulse 3370 µs sync
preamble + a 5930 µs gap pair. The cipher is a 24-bit rotating-register VAG cipher; the counter is
recovered by brute-forcing 1..=256 and matching the recomputed encrypted bytes (the C's validity signal).

> **Note:** Porsche Cayenne shares the wire protocol with KAT's existing **Porsche Touareg** decoder
> (the Touareg port is itself a port of this same `porsche_cayenne.c` source). The two are the **same
> wire protocol** — identical timing, preamble, 64-bit frame, VAG cipher, and validity check. Cayenne is
> registered AFTER Touareg, so Touareg keeps first-match priority and Cayenne never steals a Touareg
> capture. Emission is additionally gated on `frame_type` being one of the three values the C emits
> (`0b001` Cont, `0b010` First, `0b100` Final) — a strict subset of what Touareg accepts. Cayenne adds an
> encoder (the Touareg port is decode-only).

## Timing

| Parameter   | Value   | Notes                  |
|-------------|---------|------------------------|
| Short       | 1680 µs | ±500 µs (te_delta)     |
| Long        | 3370 µs | ±500 µs                |
| Sync        | 3370 µs | preamble pulse         |
| Gap         | 5930 µs | preamble→data boundary |
| Min bits    | 64      |                        |
| Sync min    | 15 pairs| (firmware emits 73)    |

## Frame Layout (64 bits / 8 bytes)

- **byte 0:** `(button << 4) | (frame_type & 0x07)` — button d-pad: Lock=0x01, Unlock=0x02, Trunk=0x04, Open/Panic=0x08; frame_type First=0x02, Cont=0x01, Final=0x04
- **bytes 1–3:** serial (24-bit, big-endian)
- **bytes 4–7:** encrypted cipher output (24-bit rotating-register VAG cipher seeded from serial, rotated `4 + counter_low` times)

The `extra` word carries the frame_type; `protocol_display_name` is `Porsche Cayenne [First|Cont|Final]`.

## RF

- **Encoding:** PWM (SHORT LOW + LONG HIGH = 0, LONG LOW + SHORT HIGH = 1)
- **RF modulation:** AM/OOK
- **Encryption:** 24-bit rotating-register VAG cipher; counter recovered by brute force, `counter != 0` is the validity gate
- **Frequencies:** 433.92 MHz, 868.35 MHz

## Decoder Steps

1. **Reset** — wait for a LOW pulse at ~3370 µs.
2. **Sync** — count sync pulses (HIGH and LOW at 3370 µs); a 5930 µs gap with ≥15 sync pulses enters GapHigh/GapLow.
3. **GapHigh/GapLow** — expect the complementary 5930 µs gap pulse, then enter Data.
4. **Data** — decode bit pairs (SHORT LOW + LONG HIGH = 0, LONG LOW + SHORT HIGH = 1); at 64 bits, `parse_data` applies the Cayenne frame_type gate + counter recovery and emits (or stays silent so Touareg owns the frame).

## Encoder

Supported (matches `porsche_cayenne_build_upload`). Emits a 4-frame burst (frame types 0b010/0b001/0b100/
0b100, cipher counters cnt+1..cnt+4), each frame being 73 sync pairs + 1 gap pair + 64 MSB-first PWM bits.

## Validation

Verified by encode→decode round-trips / synthetic-frame checks (no local capture available). Unit tests
cover the full encoder path, all three frame types, rejection of undocumented frame types and truncated
frames, the counter-recovery limit matching the C, and shared cipher vectors with the Touareg port.
