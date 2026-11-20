---
layout: default
---

# Opel V1 Protocol

**Rust module:** `src/protocols/opel_v1.rs`
**Reference:** `Flipper-ARF lib/subghz/protocols/opel_v1.c` (byte-identical to the doctor-firmware copy)

## Overview

Opel/Vauxhall (Bosch) keyfobs, AM/OOK at 433 MHz. The frame is 93 Manchester bits at 120/240 µs:

- a 5-bit framing code (0x02)
- a 24-bit plaintext serial
- a 64-bit XTEA block, encrypted under the static key `"RobertBOSCH48331"`

The decrypted block holds a check byte, the button, a burst byte, a 32-bit counter and a plaintext tail that
must be 0x05. The decoder does not need a preamble. Once 93 clean bits have been received, every further bit
re-tests the latest 93-bit window. Decode-only: the C encoder is a NULL stub.

## Timing

| Parameter | Value   | Notes                                                       |
|-----------|---------|-------------------------------------------------------------|
| Short     | 120 µs  | ±70 µs (short wins the 171–189 µs overlap)                  |
| Long      | 240 µs  | ±70 µs; any other duration resets the accumulator           |
| Frame     | 93 bits | sliding window                                              |

## Frame Layout (93 bits, MSB first)

- **bits 92..88:** framing, must be `0x02` (`0b00010`)
- **bits 87..64:** serial (24-bit, plaintext)
- **bits 63..0:** XTEA cipher block (standard XTEA, 32 cycles, big-endian words)
  - decrypted bytes: `[check, button, burst, count(32-bit BE), tail]`. The tail must be `0x05`.
  - buttons: Trunk `0x20`, Lock `0x04`, Unlock `0x08`; any other value is reported too (the C labels it
    "Unknown")

`DecodedSignal` packing: `data` = first 64 wire bits (`framing | serial | cipher[63:29]`), `extra` = last 29
wire bits (`cipher[28:0]`), `data_count_bit` = 93. The cipher is `((data & 0x7_FFFF_FFFF) << 29) | extra`.
`counter` = the low 16 bits of the 32-bit count. The check and burst bytes have no `DecodedSignal` field.
(The C stores `data` = cipher with 64 bits.)

## RF

- **Encoding:** Manchester. The level→event mapping is `level ? ShortLow : ShortHigh`, decoded with the
  canonical Flipper `manchester_decoder.c` table, so an on-air 1 is HIGH→LOW.
- **RF modulation:** AM (OOK)
- **Encryption:** XTEA (static key `RobertBOSCH48331`)
- **Frequencies:** 433.92 MHz

## Decoder Steps

1. **Classify:** each pulse becomes a Flipper Manchester event (0/2/4/6). An out-of-range pulse resets the
   accumulator and returns the Manchester state to Mid1.
2. **Manchester:** the canonical Flipper transition table. A same-state transition resets to Mid1; reaching
   Mid0 emits 0 and reaching Mid1 emits 1. (This module carries its own copy of the table, as `ford_v3.rs`
   does; the shared port in `common.rs` is identical.)
3. **Window:** shift bits into a 128-bit accumulator. The bit count caps at 93.
4. **Finish:** with 93 or more bits and framing `0x02`, decrypt and clear the accumulator. Emit if the tail
   is `0x05` (with the framing, 13 bits of check per window), whatever the button, as the C does; it labels
   buttons other than Trunk, Lock and Unlock "Unknown". (Before v1.5.0 KAT dropped those buttons.)

## Encoder

Not supported (the C reference's encoder is NULL).

## Key files

doctor-firmware's lines, which keep the serial: `Protocol: Opel V1`, `Bit: 64`, `Key` = the cipher block, `Serial`,
`Btn` (the decrypted button), `Cnt` (the 32-bit count), then the C's own `XteaKey`, `Disp` ("Opel V1 -
Trunk|Lock|Unlock|Unknown") and `Burst`. Flipper-ARF's generic writer stops after `Key`, so a key it saves reloads
with serial 0, as in ARF. Loading follows the C: exactly 64 bits, the block decrypted with the file's `XteaKey` (the
static key when absent), the serial from `Serial`; button and count come from the decrypt. Any block loads;
`crc_valid` reports its tail check.

## Validation

Verified with synthetic frames built from a test-only XTEA encipher. The one decode in IMPORTS is a chance hit: a
frame of `IMPORTS/HOLDEN/Holden Barina 2016 AU.sub`, a GM-family recording that [Chevrolet V0](chevrolet_v0.md)
reads, passes the 8-bit tail check, as one frame in 256 does (button 0x73, serial 4F2679; its button, burst and
count are no fob's values). Flipper-ARF and doctor-firmware list it as well. Unit tests cover:

- the published XTEA test vector, plus an independently computed Opel-key vector
- the canonical Manchester table semantics
- a synthetic decode with packing and field checks, for all three buttons
- rejection of a bad tail and wrong framing; a button the C calls "Unknown" (0x73) is reported
- registry-level decodes in both polarities (the inverted capture decodes via the registry's inverted pass)
- key files: doctor-firmware's lines round-trip every field, a Flipper-ARF file (no `Serial`) loads with serial 0,
  the file's `XteaKey` is used, other bit counts are refused
