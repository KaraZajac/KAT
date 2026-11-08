---
layout: default
---

# Audi V2 Protocol

**Rust module:** `src/protocols/audi_v2.rs`
**Reference:** FoxFW `lib/subghz/protocols/audi_v2.c` (itself a port of the firmware's `audi_v2.c` via KeeLoq-RX
`arke_vag.py`). **Receive only, as the reference** (the encoder is NULL).

## Overview

A 72-bit Manchester VAG key-fob frame on 315 / 433 MHz AM: a 64-bit key whose body is encrypted, followed by an 8-bit
trailer that FoxFW uses as its frame check. No serial, button or counter can be read from it. It is not the
unregistered "Audi" module (`audi.rs`) and not KAT's VAG decoder (different preamble and frame).

## Timing / modulation

OOK/AM, `te_short` 280 µs, `te_long` 560 µs, tolerance ±100 µs (flags 433 | 315 | AM).

## Frame Layout

| Part | Content |
|------|---------|
| preamble | at least 40 short pulses (280 µs) |
| separator | one long pulse (560 µs): primes the Manchester decoder, carries no data |
| key | 64 Manchester bits |
| trailer | 8 Manchester bits, must be `0x93` |

## Decoder Steps

`subghz_protocol_decoder_audi_v2_feed`:

1. Reset: any short pulse starts the preamble count.
2. Preamble: shorts are counted; a long pulse with a count of 40 or more calls
   `manchester_advance(Mid1, Long{High,Low})` and starts the data; anything else resets.
3. Data: each pulse is a Manchester event (`level ? ShortHigh/LongHigh : ShortLow/LongLow`) for Flipper's toolbox
   decoder (`common::manchester_advance`). Bits 1..64 go to the key, 65..72 to the trailer. A pulse that is neither
   short nor long resets.
4. At 72 bits the frame is reported if the trailer is `0x93`.

## Field mapping

`data` = the 64-bit key, `data_count_bit` = 72, `extra` = the trailer (0x93), `crc_valid` = true (the trailer gate is
the C's `crc_ok`), `serial`, `button`, `counter` None, `encoder_capable` false.

## Key file

`Protocol: Audi V2`, `Bit: 72`, `Key` = the 64-bit key, `Trailer` (decimal). Loading needs `Bit: 72`; a missing
`Trailer` reads as 0 and the C accepts any trailer (only 0x93 counts as the check).

## Validation

- Reference oracle `subghz_oracle_foxfw`: 7 synthetic frames (made-up keys, minimum 40-short preamble) give the same
  key and trailer; a wrong trailer, a 39-short preamble, a wrong separator and a short frame are rejected by both.
- `IMPORTS/VAG/20_vw_atlas_l2_u2_t2_rs2.sub` (315 MHz VW Atlas): the only capture that decodes, in both: key
  `C032425BAF60633E`, trailer `0x93`. **A weak hit.** The file holds 23 well-formed 72-bit frames (428 clean 280 µs
  preamble pulses each, the constant prefix `C032`, a rolling body) and the 8-bit tail differs in all of them
  (`A5 93 77 85 B4 2A 01 22 D3 76 8D D9 48 52 F8 F8 AF 56 4E 90 59 E2 82`): this decoder passes the one frame whose tail
  happens to be `0x93`. The tail is not a constant, so `0x93` is a 1-in-256 coincidence per frame there, not a
  proven protocol check. The frame is real VAG-family Manchester; the "Audi V2" label and `crc_valid` on it are not
  evidence that this is the Audi V2 protocol.
- `kat_diff` against the v1.8.7 release over IMPORTS and the Flipper unit-test captures: only that file changes
  (no decode to one frame); every other file is identical.
- Unit tests: oracle-checked synthetic frames and the four rejections, key-file round trip, decode-only.
