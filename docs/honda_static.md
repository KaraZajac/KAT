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

The checksum is an XOR of the first 7 packet bytes, and every frame KAT reports has a valid one. The parser
tries the inverted-Manchester interpretation first (what the encoder emits), then non-inverted forward, then
a bit-reversed-bytes pass, which checks the checksum too (as doctor-firmware's decoder does; ProtoPirate's
does not, see Validation).

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
2. On an out-of-range pulse (700 µs sync or gap), if ≥36 symbols are buffered, walk past the alternating preamble + sync run, Manchester-unpack 64 bits over symbol pairs (inverted first, then non-inverted), validate (checksum, button, serial; on the non-inverted reading the bit-reversed bytes as a last try), and emit. Then clear the buffer.

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

**Deviations from ProtoPirate.**

- *Bit-reversed frames need their checksum*, as in doctor-firmware's copy. ProtoPirate's and Flipper-ARF's
  bit-reversed pass checks only the button (6 codes of 16) and the serial, which about a third of any 64 bits
  pass. It runs only after the forward check has failed, and the XOR checksum holds on the bit-reversed bytes
  exactly when it holds on the bytes as read, so every frame it reports on a failed checksum is a wrong one
  (KAT listed them as Honda Static with `crc_valid` false). A KAT user reported captures of a 2025 Kia listed
  that way. Generated files of another fob's fast Manchester (half-bits of 45–80 µs, an alternating preamble,
  64–160 bits a frame, jitter, glitches) give such frames: 36 in 27 of 120 files in KAT 1.13.0, file for file
  the frames ProtoPirate's compiled C reports when given each file in the polarity KAT read it in. Flipper-ARF's
  C reports such frames too, doctor-firmware's none in either polarity. Noise and Manchester at half-bits of
  110–190 µs gave none. With the check KAT reports none of them, and a frame with a valid checksum still decodes
  in either bit order.
- *Honda V5 before Honda Static.* ProtoPirate's and Flipper-ARF's C decode the IMPORTS/honda Lock and Unlock
  captures as Honda Static (41 decodes) through the unchecked bit-reversed pass: button 1 in both files, a
  counter of 0x56BE7F in 40 of 41, and a "serial" with three values in one file. They are not Honda Static
  frames but 135-bit Honda V5 frames with a valid CRC-8 (serial 0xB53EFF72, buttons 0x21 / 0x22, a rising
  counter; see [honda_v5.md](honda_v5.md)), so KAT tries Honda V5 first. Their checksums fail: doctor-firmware's
  copy decodes neither capture, and since the check above neither does KAT's Honda Static (one frame of
  `Lock_honda.sub` that fails Honda V5's CRC was listed as Honda Static with `crc` false until then; it was the
  only change the check made to the IMPORTS decodes).

Differential tests against the compiled C (before the Honda V5 ordering and the checksum check): 1500
randomized bursts decode identically; 1990 uploads equal ProtoPirate's (encoder + emulate scene) pulse for
pulse. Unit tests cover the round trip (button, 24-bit counter carry), the compact Key layout, packet
validation and the bit-reversed pass (a frame with its checksum decodes, the same frame with a wrong checksum
does not).
