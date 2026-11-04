---
layout: default
---

# Mazda Infinity Protocol (experimental)

**Rust module:** `src/protocols/mazda_infinity.rs`
**Reference:** `Flipper-ARF lib/subghz/protocols/mazda_infinity.c` (only copy; ARF ships it disabled)

## Overview

> **Not registered.** Flipper-ARF ships `mazda_infinity.c` disabled, and KAT mirrors that (its air format is also decoded by **Mazda V0**, which has priority). The decoder/encoder below is ported and tested; to enable it, add `Box::new(mazda_infinity::MazdaInfinityDecoder::new())` at the end of `ProtocolRegistry::new()`.

The Pandora "Mazda Infinity" keyfob format, reverse-engineered from the PANDORA_MAX alarm firmware
(TX dispatcher 0xf0d0, RF bit-banger 0x8fc4, whitening/interleave 0x12a90, popcount parity 0x10994).

**Relationship to Mazda V0 / Mazda Siemens:** this is the same logical layer as KAT's `mazda_v0.rs`
and `mazda_siemens.rs` (parity-keyed XOR whitening, b5/b6 counter interleave, additive checksum,
0x10/0x20/0x40 buttons) and the same air format that ProtoPirate's current `mazda_v0.c` encoder and
Flipper-ARF's `mazda_siemens.c` encoder emit. KAT's Mazda V0 — a port of ProtoPirate's current
decoder — decodes this format as well and completes on the 64th payload bit, before the trailer Mazda
Infinity waits for, so on a shared frame **Mazda V0 keeps first-match priority** and Mazda Infinity only
reports frames that Mazda V0 and Mazda Siemens both reject. A unit test runs them first-match with and
without Mazda Infinity on 300 random frames (both polarities) and checks their results are unchanged.
Mazda Infinity also decodes KAT's own Mazda Siemens TX upload.

**Experimental:** no IMPORTS capture contains this format (the five Mazda RAW captures look like FSK
fobs recorded with an OOK preset and decode as nothing). Registered last in the registry.

## Timing

| Parameter   | Value   | Notes                                   |
|-------------|---------|-----------------------------------------|
| Short (Te)  | 250 µs  | ±62 µs (te_delta)                       |
| Long (2·Te) | 500 µs  | ±62 µs                                  |
| Bit 1       | HIGH 250 → LOW 250 |                              |
| Bit 0       | LOW 250 → HIGH 250 |                              |
| Gap         | 5000 µs | LOW between preamble and sync (encoder) |
| Bits        | 64 payload (96 decoded incl. sync/trailer) |      |

## Frame Layout

On air (bytes MSB-first): `12 × 0xFF` preamble | LOW gap | `FF FF D7` sync | 8 payload bytes
(one's complemented) | `0x5A` trailer. Repeated up to 4 times.

Logical payload (`data` = b0..b7 MSB-first, as the C's `generic.data`):

- **b0..b3:** UID → `serial` (32-bit)
- **b4:** button → `button` (0x10 Lock, 0x20 Unlock, 0x40 Trunk; raw byte kept)
- **b5:b6:** 16-bit counter → `counter`
- **b7:** checksum = `sum(b0..b6) & 0xFF`

Building the wire: checksum → interleave (`b5' = b5&0xAA | b6&0x55`, `b6' = b5&0x55 | b6&0xAA`) →
whitening keyed by `popcount(b7) & 1` (odd: b0..b5 ^= b6; even: b0..b4 and b6 ^= b5) → one's
complement. All steps are self-inverse, so decoding applies them in reverse.

## RF

- **Encoding:** Manchester
- **RF modulation:** AM (OOK, AM650 preset)
- **Encryption:** rolling code (whitened counter, no key; additive checksum)
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder Steps

The C decodes half-bit cells itself and never calls Flipper's `manchester_advance`, so no Manchester
state machine is used; polarity is the level as captured.

1. **Reset** — the first short/long pulse starts a collection.
2. **Data** — push half-bit cells (short = 1 cell, long = 2) into a 384-cell (12-byte) sliding window;
   after every push decode the window as sync(3) + payload(8) + trailer(1): pairs H,L = 1 / L,H = 0,
   equal cells invalid.
3. **Emit** when sync = `FF FF D7`, trailer = `0x5A`, all 96 pairs are valid Manchester, the
   de-whitened UID ∉ {0, 0xFFFFFFFF} and the checksum matches. Any other pulse ends the collection.

Gating vs. the C: the C only *displays* the checksum (`CS:[OK|BAD]`); KAT rejects on it. Strength:
32 fixed sync/trailer bits × 8-bit checksum ≈ 2^-40 per random alignment; given correct framing a
random payload passes 1/256 (measured in a unit test). Buttons are not whitelisted (a Remote/Panic
code may exist on real fobs).

## Encoder

Supported, mirroring the C. The logical payload is re-derived to the wire (checksum, interleave,
whitening, inversion recomputed):

- If the requested button maps to the captured b4, the frame is replayed byte-identically.
- Otherwise b4 is re-stamped and the 16-bit counter advanced by 1 (the C's default
  `rolling_counter_mult`).
- KAT generic buttons map Lock 0x01 → 0x10, Unlock 0x02 → 0x20, Trunk 0x04 → 0x40 (native codes pass
  through); any other code keeps the captured button.

Waveform: the C upload (one frame) played 4 times (the Flipper's default repeat), plus a trailing
5 ms LOW so the last frame is terminated.

## Validation

No real capture available. Unit tests cover: the C upload transcribed verbatim (unmerged) decoding for
both whitening branches, whitening round trip, encode→decode (replay and button change with counter
wrap), the encoder waveform pinned to the C layout, synthetic frames, rejection of bad checksum / sync /
trailer / zero UID, the measured content-gate rate (≈ 1/256), random noise, Mazda V0 / Mazda Siemens
first-match results unchanged by Mazda Infinity, Mazda Siemens' TX upload decoding here, and every
`IMPORTS/Mazda` capture standalone in both polarities (no decode).
