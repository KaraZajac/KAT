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
  - buttons: Trunk `0x20`, Lock `0x04`, Unlock `0x08`

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
   Mid0 emits 0 and reaching Mid1 emits 1. (KAT's shared 4-state table used by several other modules is not
   equivalent, so this module carries the canonical one, like `psa2.rs`.)
3. **Window:** shift bits into a 128-bit accumulator. The bit count caps at 93.
4. **Finish:** with 93 or more bits and framing `0x02`, decrypt and clear the accumulator. Emit only if the
   tail is `0x05` **and** the button is Trunk, Lock or Unlock. The button gate is a KAT addition: the window
   re-tests on every bit, and the C merely labels other buttons "Unknown".

## Encoder

Not supported (the C reference's encoder is NULL).

## Validation

Verified with synthetic frames built from a test-only XTEA encipher; no local capture is available. Unit
tests cover:

- the published XTEA test vector, plus an independently computed Opel-key vector
- the canonical Manchester table semantics
- a synthetic decode with packing and field checks, for all three buttons
- rejection of a bad tail, wrong framing and an unknown button
- registry-level decodes in both polarities (the inverted capture decodes via the registry's inverted pass)
