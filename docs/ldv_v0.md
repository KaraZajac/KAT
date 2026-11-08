---
layout: default
---

# LDV V0 Protocol

**Rust module:** `src/protocols/ldv_v0.rs`
**Reference:** `FoxFW lib/subghz/protocols/ldv_v0.c` ("LDV / Maxus"; no other reference has it)

## Overview

A 128-bit Manchester rolling-code frame at 250/500 µs, gated on its fixed fields. Decode-only.

**Relation to Ford V3.** The LDV T80 captures in `IMPORTS/LDV` are the same on-air frame that Ford V3's EU path
reads (Ford V3's 104-bit key there is the bit-inverse of this decoder's 128-bit key). Ford V3 (240/480 µs) is
registered first and completes at bit 104, before this decoder reaches bit 128, so Ford V3 keeps every frame it
accepts, as in FoxFW. This decoder adds what Ford V3 lacks: the timing tolerance (short 131..=369 µs, long
381..=619 µs, against Ford V3's 180..=300 / 420..=540), the whole 128-bit frame, the LDV fields (serial, 2-bit
button, 11-bit counter, 40-bit hop) and a check on the fixed fields.

## Timing

| Parameter | Value  | Notes                                           |
|-----------|--------|-------------------------------------------------|
| Short     | 250 µs | ±120 µs, strict                                 |
| Long      | 500 µs | ±120 µs, strict                                 |
| Preamble  | ≥48 identical Manchester bits | then the first differing bit (discarded) |
| Gap       | >2000 µs | back to reset                                 |

Flipper's `manchester_advance` with the event `level ? ShortLow : ShortHigh`. A HIGH short pulse starts the
preamble (it is not fed to the Manchester state).

## Frame Layout (16 bytes)

`serial32 | b4 | b5 | byte6 | 0xFF | 0xFF | byte9 | byte10 | hop40`, with `byte6 = 0x80 | (btn << 3) | 0x07`.

Gate: `byte6` bit 7 set, `byte6 & 7 == 7`, bytes 7 and 8 both `0xFF`, and the button in `byte6` (bits 3..4)
equals the button duplicated in the counter field (`((byte9 << 8) | byte10) >> 3 & 3`).
`counter = ((byte9 << 8) | byte10) >> 5`. No CRC.

**KAT packing:** `data` = bytes 0..7, `extra` = bytes 8..15 (the hop is its low 40 bits), `data_count_bit` = 128.

## RF

- **Encoding:** Manchester
- **RF modulation:** AM
- **Encryption:** rolling code
- **Frequencies:** 315 MHz, 433.92 MHz
- **Check:** none (fixed-field gate); the CRC column shows "-"

Verified against FoxFW's C decoder on made-up frames. On `IMPORTS/LDV` the FoxFW decoder reads the same frames
Ford V3 does; in KAT's registry Ford V3 reports them, as in FoxFW.
