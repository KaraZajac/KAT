---
layout: default
---

# Opel V0 Protocol

**Rust module:** `src/protocols/opel_v0.rs`
**Reference:** `FoxFW lib/subghz/protocols/opel_v0.c` (`"Opel V0"`)

## Overview

An experimental Opel/GM fob decoder, AM/OOK at 433.92 MHz. The signal is a ~1200 Hz carrier (about 830 µs per
cycle) gated into bursts about 5400 µs apart; the bit is the burst width. The C calls itself a first pass:
"the exact encoding is not fully understood". Different from [Opel V1](opel_v1.md) (XTEA, Manchester). No
checksum. Receive only, as in FoxFW, whose encoder sends nothing (see [Transmit](#transmit)).

## Decoder

1. **Merge:** a space under 120 µs belongs to the burst (it is added to its width).
2. **Preamble:** bursts, closed by spaces of 120 to 6999 µs, whose widths add up to 50000 µs or more. A space
   of 7000 µs or more before that resets.
3. **Data:** a space of 4000 to 7000 µs closes a burst (over 500 µs) as one bit; a space over 20000 µs closes
   the last burst and the frame; a space between 7001 and 20000 µs extends the current burst; a burst growing
   past 10000 µs ends the frame and restarts the preamble. At most 150 bursts are kept.
4. **Bits:** burst width over 3200 µs is 1, otherwise 0.

The burst and preamble sums are the C's `uint32_t`; KAT adds them the same way (wrapping), so absurd durations
cannot overflow them.

## Gating

Of the bits only the first 96 count and the count must be 62 to 66; an all-zero or all-one frame is dropped.

## Result

`data` = the first 64 bits (left-aligned when fewer), `data_count_bit` = the count, `extra` = the C's `seed`
(bits after the 64th, left-aligned in 32 bits; only when more than 64). No check: the interface shows "-".

## RF

- **Encoding:** burst-width (PWM-like), **RF:** AM, **Encryption:** unknown
- **Frequencies:** 433.92 MHz

## Key files

`Protocol: Opel V0`, `Bit`, `Key` (the common serialize). The C's `deserialize` insists on 40 bits, which no
decoded frame has, so KAT only loads `Bit: 40` keys, as the C does.

## Transmit

None. FoxFW flags the protocol `Send` and has an SD-plugin encoder, `applications/main/subghz/protocols_ext/enc_opel_v0.c`,
but it is a stub ("Encoder stubs (no encoding yet)"): its `deserialize` returns `SubGhzProtocolStatusErrorEncoderGetUpload`
for every key file and its `yield` sends nothing, so a Flipper offers Send and fails every time. The host oracle
(`subghz_oracle_foxfwenc encode`) reports "deserialize failed (status -12)" for each of 100 made-up key files (40, 48,
62 to 66 and 96 bits); KAT does not transmit either.

## Validation

No capture exists (the Opel files in the corpus decode as other protocols only). Against the FoxFW C decoder
(host oracle) on made-up frames of 62, 64 and 66 bursts (some bursts split by tiny carrier gaps): identical
`Key` and `Bit`; 61 and 67 bursts, all-zero and all-one frames and a 40 ms preamble are rejected by both.
The `seed` of the 66-bit frame is not printed by the oracle; its value in the unit test follows the C's
`data_2 <<= 32 - extra`. Those vectors are the unit tests.
