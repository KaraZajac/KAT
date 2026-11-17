---
layout: default
---

# Isuzu Static Protocol

**Rust module:** `src/protocols/isuzu_static.rs` (KAT's own decoder, decode only)
**Reference:** none reads this signal, and no FCC filing has been matched to it. The name is KAT's.

## Overview

The remote of an Isuzu FTS truck: a **fixed 64-bit Manchester frame**, sent for as long as the button is held.
The code does not roll: from press to press one bit changes. Worked out from two recordings of one remote
(`IMPORTS/Isuzu/Isuzu_fts_lock.sub` and `Isuzu_fts_unlock.sub`, 433.92 MHz, Flipper preset AM650, two presses
each). **Decode and display only**: KAT has no encoder for it.

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| Half bit | 523.5 µs | 955 bit/s (± 120 µs in the decoder) |
| Line code | Manchester | carrier then pause is a 1 |
| Frame | 64 bits and two 1 bits, 69.1 ms | |
| Repeats | every 70.2 ms while the button is held | 9 and 19 whole copies in the presses recorded; the last copy stops in the middle |

On-off keying, 433.92 MHz.

## Frame (the 64 bits)

| Byte | Lock | Unlock | Notes |
|------|------|--------|-------|
| 0..3 | `01 10 01 83` | `01 10 01 83` | the same in all four presses (the fob's ID, or part of the protocol: one remote cannot tell) |
| 4 | `25`, `27` | `41`, `43` | high nibble: the button; bit 1 is 0 in a recording's first press and 1 in its second |
| 5..7 | `0E 81 FF` | `0E 81 FF` | the same in all four presses |

Whether bit 1 of byte 4 toggles with every press or is the low bit of a counter, two presses of each button do
not tell.

## Decoder

There is no sync word and no known checksum. A burst (`src/protocols/burst.rs`) is a frame when it is exactly 64
bits and two 1 bits without one Manchester violation. A frame is reported when **the burst before it held the
same frame**, less than 200 ms earlier: the first copy of a press is not listed. KAT starts every decoder afresh
after a frame, so nine copies are listed four times.

## Field mapping

`data` = the 64 bits, `button` = byte 4's high nibble (2 Lock, 4 Unlock), `data_count_bit` = 64, `serial` None,
`counter` None, `crc_valid` false (shown as "-"), `encoder_capable` false. No key file.

## Validation

- `Isuzu_fts_lock.sub`: 8 frames, `0110 0183 250E 81FF` then `0110 0183 270E 81FF`.
- `Isuzu_fts_unlock.sub`: 18 frames, `0110 0183 410E 81FF` then `0110 0183 430E 81FF`.
- No other file in IMPORTS changes.
- Unit tests: nine copies (eight reported without a reset, four with), the gate (one copy, frames that differ,
  the same frame much later, other bits where the two ones stand, 65 and 67 bits), a frame that begins with a
  1 bit, the recordings.
