---
layout: default
---

# Honda Static Protocol

**Rust module:** `src/protocols/honda_static.rs`
**Reference:** `REFERENCES/ProtoPirate/protocols/honda_static.c` (HEAD dc3446b; Flipper-ARF identical apart from UI glue)

## Overview

Honda/Acura fixed-code keyfobs. 64-bit frame. Unlike most KAT decoders, Honda Static does NOT use the
Flipper `manchester_decoder.h` transition table: it buffers a per-element *symbol stream* (one bit per
~63 µs element) and then performs a custom Manchester unpack over symbol pairs. A short pulse contributes
one symbol equal to the pulse level; a long pulse contributes two symbols of the same level. Anything
outside both ranges (e.g. the 700 µs sync or a trailing gap) terminates the buffer and triggers a parse.

The checksum is an XOR of the first 7 packet bytes. Emission is gated on the checksum, so Honda Static
will not false-match. The parser tries the inverted-Manchester interpretation first (what the encoder
emits), then non-inverted forward, then a bit-reversed-bytes pass.

## Timing

| Parameter      | Value  | Notes                          |
|----------------|--------|--------------------------------|
| Element        | 63 µs  | one symbol per element         |
| Short pulse    | 28–98 µs | base 28 µs, span 70 µs       |
| Long pulse     | 61–191 µs | base 61 µs, span 130 µs (two symbols) |
| Sync           | 700 µs | terminates the symbol buffer   |
| Min symbols    | 36     | before a parse is attempted    |
| Min bits       | 64     |                                |

(Reported timing profile: te_short 63 µs, te_long 700 µs, te_delta 120 µs.)

## Frame Layout (64 bits, MSB-first into 8 bytes)

- **bits 0–3:** button (4-bit) — Lock=1, Unlock=2, Trunk=4, Remote Start=5, Panic=8, Lock×2=9
- **bits 4–31:** serial (28-bit)
- **bits 32–55:** counter (24-bit)
- **bits 56–63:** checksum (XOR of bytes 0–6)

The exported `data` (Key) word is the C `generic.data`: a compact nibble-packed layout, NOT the raw
decoded packet bytes. The KAT counter field is the low 16 bits of the 24-bit counter.

## RF

- **Encoding:** custom symbol-stream Manchester over ~63 µs elements
- **RF modulation:** FM
- **Encryption:** none — gated on the XOR checksum + valid button + valid serial (serial ≠ 0 and ≠ 0x0FFFFFFF)
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder Steps

1. **feed** — classify each pulse: short → 1 symbol, long → 2 symbols; buffer them.
2. On an out-of-range pulse (700 µs sync or gap), if ≥36 symbols are buffered, walk past the alternating preamble + sync run, Manchester-unpack 64 bits over symbol pairs (inverted first, then non-inverted), validate, and emit. Then clear the buffer.

## Encoder

`subghz_protocol_encoder_honda_static_deserialize` + ProtoPirate's emulate scene: unpack the Key, override
it with the serial, the button (KAT Lock/Unlock/Trunk/Panic → 1/2/4/8; other commands keep the captured
button; invalid codes 2..5 are remapped and others fall back to the Key's, as in the C) and the 24-bit
counter + 1 (top byte from the Key). Emits a 700 µs HIGH sync, a 160-element alternating preamble at
63 µs, 64 data bits (each as `!value`/`value` at 63 µs), and a trailing 700 µs sync. The Flipper repeats
the upload 3 times; KAT returns one. Flipper-ARF's copy is the same encoder with a D-pad map onto the same
codes. As in the C, frames whose first bit is 1 (buttons 8/9) are sent but the decoder cannot find their
preamble/data boundary.

## Validation

**Deviation from the reference.** ProtoPirate's and Flipper-ARF's C decode the IMPORTS/honda Lock and Unlock
captures as Honda Static (41 decodes) through the unchecked reverse-packet path (`crc_valid` = false): button 1
in both files, a counter of 0x56BE7F in 40 of 41, and a "serial" with three values in one file. They are not
Honda Static frames but 135-bit Honda V5 frames with a valid CRC-8 (serial 0xB53EFF72, buttons 0x21 / 0x22, a
rising counter; see [honda_v5.md](honda_v5.md)), so KAT tries Honda V5 first and these captures no longer decode
as Honda Static. The decoder itself is unchanged. doctor-firmware's older copy decodes neither. Differential tests against the compiled C (before the Honda V5 ordering): 1500 randomized bursts decode identically;
1990 uploads equal ProtoPirate's (encoder + emulate scene) pulse for pulse. Unit tests cover the round trip
(button, 24-bit counter carry), the compact Key layout and packet validation.
