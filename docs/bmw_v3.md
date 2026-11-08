---
layout: default
---

# BMW/Mini V3 Protocol

**Rust module:** `src/protocols/bmw_v3.rs`
**Reference:** FoxFW `lib/subghz/protocols/bmw_v3.c` (`BMW_V3_PROTOCOL_NAME = "BMW/Mini V3"`, a port of the firmware's
`bmw_v3.c` via KeeLoq-RX `arke_bmw.py`). **Receive only, as the reference** (the encoder is NULL).

## Overview

A 2-FSK BMW / Mini key-fob frame: an 8-bit `0xCC` header and 64 payload bits with a cleartext button and 16-bit serial
and a 40-bit rolling code the C calls AES-encrypted. A different signal from KAT's "BMW" (`bmw.rs`, 62 µs AM frame
behind the 0x599E sync).

## Timing / modulation

2-FSK Manchester-like (flags 433 | 315 | AM | FM; meaningful on FM captures), `te_short` 95 µs, `te_long` 190 µs,
tolerance ±40 µs. A LOW of more than 350 µs is the gap before a frame.

## Decoder Steps

`subghz_protocol_decoder_bmw_v3_feed`:

1. Reset: a LOW over 350 µs starts WaitGap. WaitGap: the first HIGH short or long pulse primes the accumulator
   (`te_last`); anything else resets.
2. A half-period accumulator as in BMW V2: a long pulse is one bit, two shorts are one bit, a long with a short pending
   completes a bit and leaves a third half pending; the bit is `level ? 0 : 1` of the completing pulse. A pulse that is
   neither short nor long resets (and starts a new gap wait when it is a long LOW).
3. FindHeader slides an 8-bit window over the bits (at least 8, at most 32) until its low byte is `0xCC`; the register
   then clears and the next 64 bits are the payload.
4. At 64 payload bits the frame is reported when the button byte (bits 63..56) is `0x02`, `0x04`, `0x08` or `0x10`.

## Payload fields

64 bits, MSB first: bits 63..56 button, 55..40 serial (16 bits), 39..16 and 15..0 the rolling code the C calls
encrypted; its "counter" field is bits 31..16, cut out of that block exactly as the C's field split does. No check
exists.

## Field mapping

`data` = the 64-bit payload (header excluded), `data_count_bit` = 64, `button` = the button byte, `serial` = the 16-bit
serial, `counter` = the 16-bit counter field, `crc_valid` false, `encoder_capable` false.

## Key file

`Protocol: BMW/Mini V3`, `Bit: 64`, `Key` = the payload, then `Serial`, `Btn`, `Cnt` (decimal). Loading needs `Bit: 64`;
the fields are carved from the key and the button whitelist is not applied.

## Validation

- Reference oracle `subghz_oracle_foxfw`: synthetic frames for four buttons (made-up payloads) and a frame with 5 junk
  bits before the header give identical payload, serial, button and counter; a bad button, a wrong header, a header
  more than 32 bits in and a 63-bit payload are rejected by both.
- No IMPORTS capture decodes as BMW/Mini V3 in either decoder; `kat_diff` over IMPORTS and the Flipper unit-test
  captures shows no file changed by it.
- Unit tests: the oracle-checked frames and rejections, key-file round trip, decode-only.
