---
layout: default
---

# Audi Protocol (experimental)

**Rust module:** `src/protocols/audi.rs`
**Reference:** `Flipper-ARF lib/subghz/protocols/audi.c` (only copy; ARF ships it disabled)

## Overview

> **Not registered.** Flipper-ARF ships `audi.c` disabled, and KAT mirrors that: the decoder/encoder below is ported and tested but is not in KAT's protocol registry. To enable it, add `Box::new(audi::AudiDecoder::new())` at the end of `ProtocolRegistry::new()` in `src/protocols/mod.rs`.

The Pandora "Audi" keyfob format, reverse-engineered from the PANDORA_MAX alarm firmware (whose units
emulate OEM fobs): button/counter dispatcher @ 0xf152, {b3,b4,b5} scramble + parity engine @ 0xf8fc,
PWM bit-banger @ 0xbd64. It is a SEPARATE protocol from KAT's `VAG` decoder (`vag.rs`, 300/600 or
500/1000 µs Manchester, 80 bits): Audi is 550/1100 µs PWM behind a 2500/850 µs header, 64 bits.

**Experimental:** no IMPORTS capture contains this format (it decodes nothing in the sweep), so the
RE has not been validated against a real fob. Registered last in the registry.

The rolling bytes b3..b5 are scrambled by 4 rounds of a 24-bit left shift whose carry-in is a parity
of the current block; the final carry-out is dropped and recovered on decode from that parity (bit 0
of the scrambled b3), so the transform is a bijection. Consequently the C's "parity OK" self-check
after un-scrambling holds for every input and validates nothing — the frame's only integrity check is
the 8-bit additive checksum.

## Timing

| Parameter        | Value    | Notes                                              |
|------------------|----------|----------------------------------------------------|
| Short (Te)       | 550 µs   | ±137 µs (te_delta)                                 |
| Long (2·Te)      | 1100 µs  | ±137 µs                                            |
| Header HIGH      | 2500 µs  | decoder accepts ≥ 1900 µs                          |
| Header LOW       | 850 µs   | ±274 µs                                            |
| Inter-repeat gap | 850 µs   | LOW, merges with the last bit's LOW                |
| Bit 1            | HIGH 1100 + LOW 550   |                                       |
| Bit 0            | HIGH 550 + LOW 1100   |                                       |
| Bits             | 64       | exactly                                            |

## Frame Layout (64 bits = 8 bytes, LSB-first within each byte)

`data` holds the 8 ON-WIRE bytes b0..b7 MSB-first (as the C's `generic.data`).

- **b0..b2:** 24-bit UID → `serial`
- **b3..b5:** rolling payload, scrambled on the wire; `counter` = plaintext b3:b4 (16 bits) after
  un-scrambling
- **b6:** button → `button` (0x01 Lock, 0x02 Unlock, 0x04 Trunk)
- **b7:** checksum = `sum(b0..b6) & 0xFF` over the on-wire (scrambled) bytes

Scramble round (applied 4×): `P = b3.3 ^ b4.4 ^ b4.6 ^ b4.7 ^ b5.4 ^ b5.7`; then
`b3 = b3<<1 | P`, `b4 = b4<<1 | old_b3.7`, `b5 = b5<<1 | old_b4.7` (old b5.7 dropped).

## RF

- **Encoding:** PWM
- **RF modulation:** AM (OOK, AM650 preset)
- **Encryption:** rolling code (scrambled counter, no key; additive checksum)
- **Frequencies:** 315 MHz, 433.92 MHz, 868.35 MHz

## Decoder Steps

1. **Reset** — wait for a HIGH ≥ 1900 µs (header).
2. **HeaderLow** — require LOW 850 ± 274 µs, else back to Reset.
3. **Data** — pair each HIGH with the following LOW: (long, short) = 1, (short, long) = 0, stored
   LSB-first. A LOW longer than te_long + te_delta after a data HIGH is the merged final-bit + gap:
   the HIGH width gives the last bit. Any other pulse (or a new ≥ 1900 µs header) ends the frame.
4. **Emit** only if exactly 64 bits were collected and UID ∉ {0, 0xFFFFFF}, the checksum matches and
   b6 ∈ {0x01, 0x02, 0x04}.

Gating vs. the C: the C reports any 64-bit frame with a valid UID and only *displays* the checksum.
KAT also requires the checksum (1/256), the button whitelist (3/256 — the only codes the RE'd emitter
produces) and an exact 64-bit length. A random 64-bit frame passes with p ≈ 4.6e-5 (≈ 1/21800).

Deviation (C bug fix): the C classifies the final bit only when it is 0; a final bit 1 (HIGH 1100 +
merged LOW 550 + 850 = 1400 µs) lands in its error branch with 63 bits, so the C drops every frame
whose b7 MSB is 1. KAT decodes it.

## Encoder

Supported, mirroring the C:

- If the requested button maps to the captured b6, the on-wire bytes are replayed byte-identically
  (the C's OK button).
- Otherwise (the C's D-pad path) the frame is re-derived: un-scramble, set plaintext b3:b4 to
  counter + 1 (the C's default `rolling_counter_mult`), set b6, re-scramble, recompute b7.
- KAT generic buttons map 1:1 (Lock 0x01, Unlock 0x02, Trunk 0x04); any other code (e.g. Panic 0x08)
  keeps the captured button.

Waveform = the C upload: 5 × (HIGH 2500, LOW 850, 64 PWM bits LSB-first, LOW 850).

## Validation

No real capture available. Unit tests cover: the C upload transcribed verbatim (unmerged: the last
bit's LOW and the 850 µs gap arrive as two LOWs) decoding every repeat, the scramble being a bijection
(and the parity self-check being vacuous), encode→decode round-trips for both final-bit values, the
encoder waveform pinned to the C layout (header + LSB-first PWM), button change
re-scrambling with counter advance, a synthetic frame with known fields, rejection of a bad
checksum / bad button / 63- and 65-bit frames, the measured content-gate rate (≈ 4.6e-5), random
noise, VAG Type 2 encoder output and Type 3/4-shaped bursts (both polarities), and every
`IMPORTS/VAG` capture standalone in both polarities (no decode).
