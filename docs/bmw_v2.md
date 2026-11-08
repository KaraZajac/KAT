---
layout: default
---

# BMW V2 Protocol

**Rust module:** `src/protocols/bmw_v2.rs`
**Reference:** FoxFW `lib/subghz/protocols/bmw_v2.c` (a port of the firmware's `bmw_v2.c` via KeeLoq-RX
`arke_mitsu_bmw.py`). **Receive only, as the reference** (the encoder is NULL).

## Overview

A 56-bit AM Manchester BMW key-fob frame: a fixed `0xCF` header, a 40-bit encrypted rolling code and a fixed `0x3A`
trailer. The header and trailer are framing bytes, not a checksum. It is a different signal from KAT's "BMW"
(`bmw.rs`, Flipper-ARF's 62 µs 128-bit frame behind the 0x599E sync): this one runs at 500 / 1000 µs, a speed `bmw.rs`
treats as a gap, so the two never read the same frame.

## Timing / modulation

OOK/AM (flags 433 | 315 | AM), `te_short` 500 µs, `te_long` 1000 µs, tolerance ±150 µs. A LOW of more than 1700 µs is
the inter-frame gap.

## Decoder Steps

`subghz_protocol_decoder_bmw_v2_feed`:

1. Reset: a LOW over 1700 µs starts WaitGap. WaitGap: the first HIGH short or long pulse only primes the accumulator
   (`te_last`); another long LOW keeps waiting; anything else resets.
2. Manchester: a half-period accumulator. A long pulse is one bit; two short pulses are one bit; a long arriving while a
   short is pending completes a bit and leaves a third half pending. A bit's value is `level ? 0 : 1` of the completing
   pulse.
3. A pulse that is neither short nor long evaluates the frame (and starts a new gap wait when it is a long LOW); so does
   a 65th bit.
4. Evaluation: with 56 or more bits, the bits are right-aligned to 56 (the first 56 collected are kept when more arrive,
   and the register holds 64), the header must be `0xCF` and the trailer `0x3A`, and the serial may not be `000000`,
   `FFFFFF`, `555555` or `AAAAAA`.

## Frame fields

56 bits, MSB first: header `0xCF`, rolling code (40 bits), trailer `0x3A`. The "serial" is the rolling code's upper 24
bits and the counter its lower 16 bits, so both change with every press.

## Field mapping

`data` = the 56-bit frame, `data_count_bit` = 56, `serial` = the 24-bit carved value, `counter` = the 16-bit carved
value, `button` None, `crc_valid` false (no check beyond the fixed bytes), `encoder_capable` false.

## Key file

`Protocol: BMW V2`, `Bit: 56`, `Key` = the frame, then `Serial`, `Cnt`, `Hdr`, `Trl` (decimal). Loading needs `Bit: 56`;
the fields are carved from the key and no header, trailer or serial gate is applied.

## Validation

- Reference oracle `subghz_oracle_foxfw`: synthetic frames (made-up rolling codes, including 4 and 8 trailing extra
  bits) give identical key, serial and counter; a wrong header, a wrong trailer, the four degenerate serials, extra bits
  in front of the frame, 65 bits and a 55-bit frame are rejected by both.
- No IMPORTS capture decodes as BMW V2 in either decoder; `kat_diff` over IMPORTS and the Flipper unit-test captures
  shows no file changed by it.
- Unit tests: the oracle-checked frames and rejections, key-file round trip, decode-only.
