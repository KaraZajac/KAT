---
layout: default
---

# Hyundai Protocol (experimental, not registered)

**Rust module:** `src/protocols/hyundai.rs`
**Reference:** `Flipper-ARF lib/subghz/protocols/hundai.c` (only copy; ARF ships it disabled under the
name **"Hundai"**; KAT uses the corrected spelling "Hyundai")

## Overview

The Pandora "new Hyundai" keyfob format, reverse-engineered from the PANDORA_MAX alarm firmware
(button/session dispatcher @ 0xf236, Manchester bit-banger @ 0x8290). Distinct from KAT's Kia V0–V7
family.

**Status: not registered.** The module, encoder and tests are in the tree, but the decoder is left
out of `ProtocolRegistry::new()`: the frame carries no checksum (the C calls b8 a "possible check
byte" but never computes or checks it), so the only content gates (UID, key index ≠ 0) pass ~15/16 of
random bodies and a decode's validity rests entirely on framing. No IMPORTS capture contains this
format (the Hyundai/Genesis RAW captures look like FSK fobs recorded with an OOK preset). Register it
once a real capture confirms the layout.

## Timing

| Parameter        | Value   | Notes                                       |
|------------------|---------|---------------------------------------------|
| Short (Te)       | 500 µs  | ±125 µs (te_delta)                          |
| Long (2·Te)      | 1000 µs | ±125 µs                                     |
| Bit 1            | HIGH 500 → LOW 500 |                                  |
| Bit 0            | LOW 500 → HIGH 500 |                                  |
| Header           | LOW 4500 → HIGH 1000 → LOW 500 | first frame          |
| Inter-frame gap  | LOW 2500 → HIGH 1000 → LOW 500 | second frame         |
| Gap threshold    | ≥ 1500 µs LOW | starts / ends a collection            |
| Bits             | 72 body |                                             |

## Frame Layout

Per press the frame is sent TWICE: header, 32 × `0x55` preamble (256 Manchester bits), 9 body bytes,
all MSB-first.

- **b0..b3:** UID → `serial` (32-bit view, as the C)
- **b4, b8:** more UID / the undocumented "check byte"
- **b5 low nibble:** button → `button` (0x01 Lock, 0x02 Unlock, 0x09 Trunk); high nibble preserved
- **b6:** session counter → `counter`
- **b7:** high nibble = rolling key index 1..15 (never 0); low nibble = channel / DIP

Packing: `data` = b0..b7 (first 64 wire bits, MSB-first), `extra` = b8, `data_count_bit` = 72.

## RF

- **Encoding:** Manchester
- **RF modulation:** AM (OOK, AM650 preset)
- **Encryption:** rolling code (session counter + key index, no key known)
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder Steps

The C decodes half-bit cells itself and never calls Flipper's `manchester_advance` (it notes the
firmware machine cannot lock onto the merged 0x55 preamble), so no Manchester state machine is used;
polarity is the level as captured.

1. **Reset** — wait for a LOW ≥ 1500 µs.
2. **Data** — push half-bit cells (short = 1, long = 2) into a 728-cell sliding window. Any other
   pulse ends the collection; a LOW ≥ 1500 µs immediately starts the next one.
3. **Tail decode** — the body is the last 72 bits before the terminating pulse (pairs H,L = 1 /
   L,H = 0), anchored on the 0x55 preamble before it. The alignment with the body's final half-cell
   restored (merged into the gap when the last bit is 1) is tried too.
4. **Gate** — all 72 pairs valid; an unbroken 0x55 alternation of 128..=257 bits right before the body
   (≥ 16 preamble bytes, ≤ 32 bytes + the header pair); UID ∉ {0, 0xFFFFFFFF}; key index ≠ 0.
5. **Emit** when the same body is decoded twice in a row (the press's two copies) — once per press.

Gating vs. the C: the C reports any tail with 4 × 0x55 + 72 valid bits + valid UID; the duplicate
("Dup: OK") is display only. KAT's preamble window rejects long-preamble 500/1000 µs protocols such as
Kia V2 (252 long pairs); the duplicate makes a noise match need an identical 72-bit repeat (2^-72).

Deviation (C bug fix): when the last body bit is 1 its trailing LOW half-cell merges into the gap and
is never pushed, so the C's tail window is one cell off and it drops every frame whose b8 is odd. KAT
recovers it.

## Encoder

Supported, mirroring the C (replay-based — rolling code with no known key):

- If the requested button maps to the captured b5 nibble, the 9 bytes are replayed byte-identically.
- Otherwise the nibble is re-stamped (high nibble kept) and b6 advanced by 1; when b6 wraps the b7 key
  index advances 1..15, skipping 0 (channel nibble kept).
- KAT generic buttons map Lock 0x01 → 0x01, Unlock 0x02 → 0x02, Trunk 0x04 → 0x09 (native 0x09 passes
  through); any other code keeps the captured button.
- Needs `extra` (b8); returns `None` without it.

Waveform = the C upload: both frames (4500 µs header, then 2500 µs inter-frame header) + trailing
LOW 2500 µs.

## Validation

No real capture available. Unit tests cover: the C upload transcribed verbatim (unmerged) decoding for
both final-bit values, encode→decode (one decode per press), the encoder waveform pinned to the C
layout, button change with counter/key-index rollover, a frame with only 16 preamble bytes, rejection
of a single frame, mismatched copies, too-short/too-long preambles, zero UID, key index 0 and a
corrupted body in both copies, random noise, Kia V2 encoder output, and every `IMPORTS/HYUNDAI`,
`IMPORTS/genesis` and `IMPORTS/KIA` capture standalone in both polarities (no decode).
