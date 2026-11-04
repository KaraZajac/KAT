---
layout: default
---

# Porsche Cayenne Protocol

**Rust module:** `src/protocols/porsche_cayenne.rs`
**Reference:** `Flipper-ARF lib/subghz/protocols/porsche_cayenne.c` (protocol name "Porsche AG")

## Overview

Porsche keyfobs (the cipher is the 24-bit rotating-register VAG rolling code also used by VW Touareg /
Audi). PWM at 1680/3370 µs: SHORT LOW + LONG HIGH = bit 0, LONG LOW + SHORT HIGH = bit 1. 64-bit MSB-first
frame, preceded by a 73-pulse 3370 µs sync preamble + a 5930 µs gap pair.

KAT used to carry a second decoder for this frame, **Porsche Touareg** (ProtoPirate's decode-only port of
the same `porsche_cayenne.c`). ProtoPirate removed it in September 2026, so KAT retired it as well:
`ProtocolRegistry::get()` maps `Porsche Touareg` (and `Porsche Touareg [First|Cont|Final]` display names of
older captures) to Porsche Cayenne, which can transmit them.

## Timing

| Parameter   | Value   | Notes                  |
|-------------|---------|------------------------|
| Short       | 1680 µs | ±500 µs (te_delta)     |
| Long        | 3370 µs | ±500 µs                |
| Sync        | 3370 µs | preamble pulse         |
| Gap         | 5930 µs | preamble→data boundary |
| Min bits    | 64      |                        |
| Sync min    | 15 pulses| (firmware emits 73 pairs) |

## Frame Layout (64 bits / 8 bytes)

- **byte 0:** `(button << 4) | frame_type` — button: Lock=0x01, Unlock=0x02, Trunk=0x04, Open=0x08;
  frame_type: First=0b010, Cont=0b001, Final=0b100
- **bytes 1–3:** serial (24-bit, big-endian)
- **bytes 4–7:** cipher output: a 24-bit register seeded from the serial, rotated `4 + cnt_lo` times, mixed
  with the button/frame-type byte and the 16-bit counter

## Decoding

Like the C, every preamble + gap + 64-bit frame is reported, whatever its frame type, and the counter is
recovered by brute force (`crc_valid` = a counter reproduces bytes 4–7; otherwise counter 0, as the C
reports it). The C only tries cipher counters 1–256; KAT recovers the full 16-bit counter. The frame
determines it uniquely: bytes 4–6 carry `~cnt_lo` in clear and every `cnt_hi` bit lands in bytes 5–7, so
KAT tries the 256 `cnt_hi` values for the frame's `cnt_lo`. Results are identical for counters 1–256; above
that the C shows 0 while KAT shows the real counter, which the encoder needs. The 16 cipher bits that
depend only on serial/button/`cnt_lo` are the integrity check.

## RF

- **Encoding:** PWM (SHORT LOW + LONG HIGH = 0, LONG LOW + SHORT HIGH = 1)
- **RF modulation:** AM/OOK
- **Encryption:** 24-bit rotating-register VAG cipher; counter recovered by brute force
- **Frequencies:** 315 MHz, 433.92 MHz, 868.35 MHz (C flags 315 | 433 | 868)

## Decoder Steps

1. **Reset** — wait for a LOW pulse at ~3370 µs.
2. **Sync** — count sync pulses (HIGH and LOW at 3370 µs); a 5930 µs gap with ≥15 sync pulses enters GapHigh/GapLow.
3. **GapHigh/GapLow** — expect the complementary 5930 µs gap pulse, then enter Data.
4. **Data** — decode bit pairs; at 64 bits, recover the counter and emit.

## Encoder

Supported (`porsche_cayenne_build_upload`), for every decoded frame. Emits a 4-frame burst (frame types
0b010/0b001/0b100/0b100, cipher counters cnt+1..cnt+4), each frame being 73 sync pairs + 1 gap pair + 64
MSB-first PWM bits. KAT's Lock/Unlock/Trunk/Panic map to the C's D-pad codes 0x01/0x02/0x04/0x08
(Lock/Unlock/Trunk/Open); any other request re-sends the captured button (the C's OK path does the same
for captured codes 1/2/4/8 but sends 0x01 for any other captured code).

## Validation

C oracle (the unmodified current `porsche_cayenne.c` compiled on the host): 4000 encoder inputs
(serials, buttons, counters across 0–65535, every D-pad path) produce identical uploads; 2925 frames in
1600 randomized streams (clean, jittered, glitched, inverted, short/long preambles, corrupted cipher
bytes) plus all 123 IMPORTS captures decode with identical index, data, serial and button, and the same
counter wherever the C recovers one (on uncorrupted frames KAT's additional counters equal the
generating values).
No IMPORTS capture contains this format. Unit tests embed C golden uploads.
