---
layout: default
---

# Audi V1 Protocol (kept, not registered)

**Rust module:** `src/protocols/audi_v1.rs`
**Reference:** FoxFW `lib/subghz/protocols/audi_v1.c` (`SUBGHZ_PROTOCOL_AUDI_V1_NAME = "Audi V1"`). FoxFW's encoder is
an SD plugin KAT does not port, so this is **receive only**. **The decoder is in the tree but not in the registry**,
because it false-matches Fiat V1.

## Overview

An 88-bit 433 MHz AM Manchester VAG key-fob frame: `serial:32 | button byte:8 | hop:48`. The button byte's upper
nibble is the button (0xE Lock, 0xB Unlock), its lower two bits are counter bits 7..6; the 8-bit counter is
`hop[47:42] | btn_lo << 6`. The hop is a weakly scrambled Feistel code in FoxFW's description; there is no check.
Not KAT's unregistered "Audi" module (`audi.rs`).

## Timing / modulation

Manchester, `te_short` 260 µs, `te_long` 520 µs, tolerance ±130 µs; a 3000 µs (±800) LOW sync, a 2000 µs (±500) HIGH
header, at least 20 short pulses, then a long pulse that is consumed as the Manchester separator.

## Decoder Steps

`subghz_protocol_decoder_audi_v1_feed`: Reset waits for the LOW sync; WaitHeader for the HIGH header (anything else
back to Reset); Preamble counts shorts and takes a long pulse with a count of 20 or more as the start of data (the
Manchester state is reset and the long pulse advances it without a bit); Data feeds short/long pulses to the toolbox
Manchester decoder (`level ? High : Low` events); the first 64 bits are `generic.data`, the next 24 finish the frame;
any other pulse resets. A frame is reported when the button nibble is 0xE or 0xB and the serial is not zero.

## Field mapping

`data` = the first 64 bits, `extra` = the 48-bit hop, `data_count_bit` = 88, `serial` = the 32-bit serial, `button` =
the button nibble, `counter` = the 8-bit counter, `crc_valid` false.

## Key file

`Protocol: Audi V1`, `Bit: 88`, `Key` = the first 64 bits, `Hop` = 6 hex bytes (without it the C rebuilds a partial hop
from the key's low 24 bits).

## Why it is not registered

The gate (3 ms LOW, 2 ms HIGH, 20 short pulses, button nibble 0xE or 0xB, non-zero serial) also passes the HITAG2
air format of Fiat V1 (Manchester 250/500 µs behind a ~3.5 ms sync and a ~2 ms header): about one Fiat V1 frame in eight
reads as an Audi V1 frame. With the decoder registered, KAT's own Fiat V1 transmission in
`renault_v1_arf::tests::hitag2_renault_and_fiat_v1_frames_are_not_claimed` decodes as "Fiat V1" **and** "Audi V1"
(five times). The FoxFW C decoder does the same on that waveform (host oracle: `Audi V1 ... Key: ED CB A9 87 BB F5 B9 94`),
so the false match is in the reference gate, not the port. The IMPORTS and Flipper unit-test captures are unaffected
(`kat_diff` shows no file changed), but a decoder that duplicates a registered protocol's frames does not belong in the
registry. The module is kept like `audi.rs` and `hyundai.rs`.

## Validation

- Reference oracle `subghz_oracle_foxfw`: synthetic Lock and Unlock frames (made-up serials and hops) give identical
  key and hop; a bad button, a zero serial, a 19-short preamble, a wrong sync, a wrong header and a frame 14 pulses
  short are rejected by both. No IMPORTS capture decodes in either.
- Unit tests: the frames above, key-file round trip, the Fiat V1 false match (documented in a test), decode-only.
