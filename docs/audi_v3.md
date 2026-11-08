---
layout: default
---

# Audi V3 Protocol

**Rust module:** `src/protocols/audi_v3.rs`
**Reference:** FoxFW `lib/subghz/protocols/audi_v3.c` (a port of the firmware's `audi_v3.c` via KeeLoq-RX
`arke_vag.py`). **Receive only, as the reference** (the encoder is NULL).

## Overview

A 96-bit 2-FSK VAG key-fob frame: button, a 56-bit encrypted hop, a 24-bit counter and a check byte the C displays but
never verifies. The decoder is gated only by a button whitelist and a non-degenerate hop; there is no capture of it in
IMPORTS.

## Timing / modulation

FM (2-FSK), flags 433 | 315 | AM | FM. A custom half-bit demodulator, not the Manchester table:

| Pulse | Duration |
|-------|----------|
| preamble | 220..470 µs |
| long | 580..980 µs |
| short | under 580 µs |
| gap | 980 µs or more |

## Frame Layout

96 bits, MSB first: `button:8 | hop:56 | counter:24 | chk:8`.

## Decoder Steps

`subghz_protocol_decoder_audi_v3_feed`:

1. Reset: a preamble-length pulse starts the count. Preamble: preamble-length pulses are counted; a long pulse with a
   count of 20 or more opens the data window and supplies bit 0; anything else resets.
2. Data: a pulse of 980 µs or more ends the frame. A long pulse is one bit (a long arriving between a bit's two shorts
   ends the frame); two consecutive short pulses are one bit. A bit's value is `level ? 0 : 1` of the pulse that
   completes it. At 96 bits the frame is evaluated.
3. Reported only when all 96 bits are in, the button is `0x0C` (Unlock) or `0x0D` (Lock) and the hop is neither all
   `0x00` nor all `0xFF`.

## Field mapping

`data` = `button << 56 | hop` (the C's `generic.data`), `data_count_bit` = 96, `button` = the button byte, `counter` =
the low 16 bits of the 24-bit counter, `extra` = `counter << 8 | chk`, `serial` None, `crc_valid` false (no check
exists), `encoder_capable` false.

## Key file

`Protocol: Audi V3`, `Bit: 96`, `Key` = `button << 56 | hop`, then `Btn`, `Cnt` (24 bits) and `Chk`, all decimal.
Loading needs `Bit: 96`; missing `Cnt` / `Chk` read as 0; no gate is applied.

## Validation

- Reference oracle `subghz_oracle_foxfw`: synthetic frames for both buttons (made-up hops, counters and check bytes)
  give identical key, button, counter and check; an unlisted button, a zero hop, an all-FF hop, a 19-pulse preamble, a
  wrong opening pulse, a gap inside the data and a frame one bit short are rejected by both.
- No IMPORTS capture decodes as Audi V3 in either decoder; `kat_diff` over IMPORTS and the Flipper unit-test captures
  shows no file changed by it.
- Unit tests: the oracle-checked frames and rejections, key-file round trip, decode-only.
