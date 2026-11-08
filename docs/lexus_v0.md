---
layout: default
---

# Lexus V0 Protocol

**Rust module:** `src/protocols/lexus_v0.rs`
**Reference:** `FoxFW lib/subghz/protocols/lexus_v0.c` (no other reference has it)

## Overview

A 224-bit Manchester rolling-code frame: a discarded 32-bit header, a 160-bit encrypted rolling code and a
32-bit serial. It is a different air format from KAT's `Toyota` (Flipper-ARF / doctor-firmware `toyota.c`, about
793/1587 µs bi-phase), so the two read different frames. Decode-only (FoxFW has no encoder).

## Timing (custom bands, not `DURATION_DIFF`)

| Pulse     | Duration      |
|-----------|---------------|
| Preamble  | 250..=470 µs  |
| Short     | under 500 µs  |
| Long      | 500..=980 µs  |
| Gap       | 980 µs and up |

At least 16 preamble pulses, then the first LONG starts the data and is the first bit (its level). In the data
a LONG is one bit equal to its level (a LONG between two SHORTs is a framing error that ends the frame); two
SHORTs are one bit equal to the second one's level. A gap or the 224th bit ends the frame.

## Frame Layout (224 bits)

| Bits     | Field                                  |
|----------|----------------------------------------|
| 0..31    | header (discarded)                     |
| 32..191  | rolling code, 160 bits (20 bytes)      |
| 192..223 | serial, 32 bits                        |

There is no CRC. The one validator is the Toyota complement signature: the serial alternates between S and ~S
on successive frames, so a frame counts as verified once its ~S partner has been seen. A serial of all zeros or
all ones is dropped as noise.

**KAT packing:** `data` = the top 64 bits of the rolling code (FoxFW's `generic.data`), `data_count_bit` = 224,
`serial` = the serial, `extra` = 1 when the complement signature verified the frame (else 0; `crc_valid` mirrors
it). Button and counter are not in the clear. The other 96 rolling-code bits are not kept.

The previous serial is remembered across `reset()`, as in FoxFW, so the signature works across the frames of a
press even though KAT's registry resets its decoders after every hit.

## RF

- **Encoding:** Manchester
- **RF modulation:** AM
- **Encryption:** rolling code
- **Frequencies:** 315 MHz, 433.92 MHz
- **Check:** the complement signature shows as a checked frame; an unverified frame shows "-"

No capture of a real Lexus V0 fob exists in KAT's test files; the decoder is verified against FoxFW's C decoder
on made-up frames (including the complement partner, degenerate serials, short frames and short preambles).
