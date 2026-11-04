---
layout: default
---

# Datsun Lada Protocol

**Rust module:** `src/protocols/datsun_lada.rs`
**Reference:** `doctor-firmware lib/subghz/protocols/datsun_lada.c`

## Overview

AvtoVAZ-built Datsun (on-DO / mi-DO) and Lada keyfobs. doctor-firmware is the only copy in REFERENCES.
FM at 433.92 MHz, Manchester with a 250 µs half-bit, 88-bit (11-byte) frames protected by an XOR checksum
and a 32-bit authenticator. The reference's "predicate" is the Hitag2 non-linear filter function (tables
0x2C79 / 0x6671 / 0x7907287B) and its "loop carry" is the Hitag2 LFSR feedback, so this is a Hitag2-family
RKE scheme whose key is a fixed 16-bit word plus state rebuilt from the frame (counter / button bits) —
which is why the reference can both verify ("learn") and generate frames.

## Timing

| Parameter   | Value        | Notes                                            |
|-------------|--------------|--------------------------------------------------|
| Short       | 250 µs       | classified as 141–339 µs                         |
| Long        | 500 µs       | classified as 361–749 µs                         |
| Glitch      | < 50 µs      | dropped; next pulse lengthened by the one before |
| Preamble    | 125 toggles  | 250 µs, after a 2 ms LOW (24-bit tails: 29)      |
| Sync        | 500 µs LOW   | after a short pulse                              |
| Frame end   | 4 ms HIGH    | any pulse that is neither short nor long         |
| Frame       | 88 bits      | exactly                                          |

## Frame Layout (88 bits = 11 bytes `r[0..11]`, sent `r[10]` first, MSB-first)

- **r[10..7]:** 32-bit serial (`r[10]` = MSB)
- **r[6]:** high nibble = button (0x40 BTN0, 0x80 BTN1, 0x20 BTN3, 0x60 BTN5); low nibble feeds the cipher state
- **r[6..1] as a 48-bit word F:** F[41:34] = counter low byte, F[33:2] = 32-bit authenticator, F[1:0] = `10`
- **r[0]:** XOR of `r[1..10]` (the XOR of all 11 bytes is 0)

KAT packing: `data` = `r[10..3]` (big-endian), `extra` = `r[2] r[1] r[0]` (24 bits), `data_count_bit` = 88.
`counter` is reported only when the frame verifies (see below).

## RF

- **Encoding:** Manchester (bit = half-bit pair `(!v, v)`: 1 = LOW→HIGH)
- **RF modulation:** FM
- **Encryption:** Hitag2-family authenticator (fixed key part + frame-derived state)
- **Frequencies:** 433.92 MHz

## Decoder Steps

1. **Glitch handling** — pulses < 50 µs set `glue`; the next pulse is lengthened by the pulse before the glitch.
2. **Reset** — wait for a long RF-LOW pulse right after a short one (sync). KAT addition: at least 64
   consecutive short pulses must precede it.
3. **Data** — the level is inverted and the first half-bit of every cell is recorded: after a bit, a short
   pulse is the cell's second half and is skipped; a long pulse yields the next bit directly.
4. **End** — the first pulse that is neither short nor long ends the frame; it is accepted only with exactly
   88 bits and a valid XOR checksum (the reference's emission rule).
5. **Learn** — rebuild the cipher state from the frame and compare 32 keystream bits with the authenticator.
   This sets `crc_valid`, `encoder_capable` and whether `counter` is shown (the reference prints "Cnt" or
   "Check failed"). It is not a gate: it only verifies frames whose counter bits 13:8 are zero (those bits
   are mixed into the state but are not in the frame), so even the reference encoder's own frames fail it
   for counter ≥ 256 or button 0x60.

## Encoder

The reference rolling-code encoder (no "Learned"/"Cnt" checkpoint in KAT): the captured frame must pass
learn; counter + 1 (14-bit); the frame is rebuilt for the selected button and the full upload is emitted:
200 µs HIGH, two main frames [2 ms LOW, 125 toggles, 500 µs LOW, 250 µs HIGH, 88 bits, 4 ms HIGH], then 4
(32 for Trunk) 24-bit counter tails [2 ms LOW, 29 toggles, sync, 24 bits, 36 ms HIGH]. Buttons: Lock → 0x40,
Unlock → 0x80, Trunk → 0x20 (long tail), Panic → 0x60. The reference labels these only BTN0/1/3/5, so the
Lock/Unlock/Trunk/Panic assignment is a guess.

## Validation

Unit tests cover reference vectors for frame building (cross-checked against an independent Python model of
the C), learn's counter limit, decode of the reference upload, encode→decode for all four buttons, rejection
of checksum errors / short preambles / 87- and 89-bit frames, glitch gluing, and a registry round trip.

Real captures: no `IMPORTS/nissan` or `IMPORTS/Skyline` capture contains an XOR-valid 88-bit frame in either
polarity, so none decodes. The only hit in all of IMPORTS was `FIAT/Fiat Panda.sub` in inverted polarity
(Fiat V1 / Marelli PCF7946 uses the same 11-byte frame shape with a 29-toggle pre-sync run); the preamble
gate rejects it and the capture still decodes as Fiat V1 (regression test).
