---
layout: default
---

# Kia Forte Protocol

**Rust module:** `src/protocols/kia_forte.rs` (KAT's own decoder, decode only)
**Reference:** none reads this signal, and no FCC filing has been matched to it. The name is the recording's.

## Overview

A 315 MHz Kia remote that sends an **88-bit Manchester frame four times a press**. The button is in the clear;
five bytes change with every press. Worked out from one recording of one fob
(`IMPORTS/KIA/more/Forte_315mhz_FM.sub`, 2026-11, Flipper preset FM476, ten presses, two buttons): **decode and
display only**.

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| Half bit | 489.8 µs | 1021 bit/s (± 150 µs in the decoder) |
| Line code | Manchester | read at the polarity in which the burst begins with zeros |
| Frame | 88 bits, 86.7 ms | |
| Press | four frames | the first behind a run of zero bits; one every 90.1 ms, about 3.4 ms between them |

2-FSK, 315 MHz. The four copies of a press are alike.

## Frame (88 bits, 11 bytes)

| Byte | Field | Notes |
|------|-------|-------|
| 0 | `01` | in every frame |
| 1 | Button | `01` in the first five presses, `02` in the last five |
| 2..4 | Fob | `99 0E CD` in every press of the one fob (an ID, or part of the protocol: one fob cannot tell) |
| 5..9 | Code | changes with every press |
| 10 | | high nibble changes, low nibble `D` in every press |

## Decoder

There is no sync word and no known checksum. A burst (`src/protocols/burst.rs`) is a frame when it begins with
zero bits, at least the seven of byte 0, has its first one bit as the last bit of byte 0 and ends with the 88th
bit (at most 8 half bits behind it), without one Manchester violation. A frame is reported when **the burst
before it held the same frame**, less than 300 ms earlier: the first copy of a press is not listed, and a press
of which one copy was caught is not read. KAT starts every decoder afresh after a frame, so a press of four
copies is listed twice.

## Field mapping

`button` = byte 1, `serial` = bytes 2..4, `data` = bytes 3..10, `extra` = bytes 0..2, `data_count_bit` = 88,
`counter` None, `crc_valid` false (shown as "-"), `encoder_capable` false. No key file.

## Validation

- `Forte_315mhz_FM.sub`: 20 frames, two for each of the ten presses; button 1 in five presses, 2 in five; ten
  different codes.
- No other file in IMPORTS changes.
- Unit tests: four copies (three reported without a reset, two with), either polarity, the gate (one copy, frames
  that differ, the same frame half a second later, another byte 0), the recording.
