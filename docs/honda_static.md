---
layout: default
---

# Honda Static Protocol

**Rust module:** `src/protocols/honda_static.rs`
**Reference:** `REFERENCES/ProtoPirate/protocols/honda_static.c`

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

Supported (matches `honda_static_build_upload`). Emits a 700 µs HIGH sync, a 160-element alternating
preamble at 63 µs, 64 data bits (each as `!value`/`value` at 63 µs), and a trailing 700 µs sync.

## Validation

Decodes REAL IMPORTS captures (IMPORTS/honda Lock and Unlock). Also covered by encode→decode round-trip
and packet-validate unit tests.
