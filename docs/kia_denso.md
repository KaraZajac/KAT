---
layout: default
---

# Kia Denso Protocol

**Rust module:** `src/protocols/kia_denso.rs` (KAT's own decoder, decode only)
**Reference:** none reads this signal. The name is KAT's: the smart keys DENSO Korea builds for Kia.

## Overview

Newer Kia smart keys (an EV6 and one more late-model Kia in the recordings) that send a **112-bit Manchester
frame five times back to back**. A 28-bit fob ID is in the clear; the rest changes with every press. No checksum
is known and the cipher is not known: **decode and display only**.

The decoder was worked out from two recordings in `IMPORTS/KIA/more/` (2026-11, 433.92 MHz, Flipper preset FM476,
ten presses each).

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| Half bit | 174.8 µs | 2860 bit/s on both fobs (± 55 µs in the decoder) |
| Line code | Manchester | either polarity of the two tones is read |
| Frame | 112 bits, 39.2 ms | |
| Repeats | five, without a gap | 196 ms a press |

2-FSK, 433.92 MHz.

## FCC filings

| FCC ID | Remote | What the filing shows |
|--------|--------|-----------------------|
| `CQOFN01050` | the EV6's, four buttons (lock, unlock, panic, remote start); DENSO Korea "Smart Key FOB", 2021 | one block of 0.19 s ("automatically off within 0.196 seconds": five frames of 39.2 ms); the two tones 60.4 kHz apart; RF clock 27.6 MHz |
| `CQOFD00790` | five buttons (trunk as well), 2019 | one block of 0.20 s; the two tones 60.8 kHz apart |
| `CQOFD01340` | seven buttons (forward and backward as well), 2021 | one block of 0.19 s; the two tones 60.8 kHz apart; RF clock 27.6 MHz |

Parts catalogues list all three for the EV6. Which of them the second recording's remote is, is not known.

## Frame (112 bits)

| Bits | Field | Notes |
|------|-------|-------|
| 0..19 | Lead | 9 ones and `00100110101` (`0xFF935`), the same on both fobs |
| 20..47 | Fob | 28 bits, constant per fob (`9AC9D77`, `7B74E67`) |
| 48..57 | ? | `1000111110` on both fobs in every press; the button may be in here (the buttons pressed are not known) |
| 58..110 | Code | changes with every press |
| 111 | `0` | in every frame seen |

## Decoder

The decoder collects the half bits of a burst (`src/protocols/burst.rs`) and reads it when it is over. A frame
needs the 20-bit lead, 112 bits without one Manchester violation, and **the same 112 bits again right behind
them**. A press of which only one whole copy was caught is not read. Both phases and both polarities are tried;
a burst gives one frame however many copies it holds.

## Field mapping

`serial` = the 28 fob bits, `data` = frame bits 48..111, `extra` = frame bits 0..47, `data_count_bit` = 112,
`button` None, `counter` None, `crc_valid` false (shown as "-"), `encoder_capable` false. No key file.

## Validation

- `22-26_Kia_EV6_433.92mhz_FM476.sub`: 10 frames (one per press), fob `9AC9D77`, ten different codes.
- `Random_late_model_KIA_remote_433.92mhz_FM476.sub`: 10 frames, fob `7B74E67`, ten different codes.
- No other file in IMPORTS changes.
- Unit tests: two and five copies at either polarity, the gate (one copy, copies that differ, another lead), the
  two recordings.
