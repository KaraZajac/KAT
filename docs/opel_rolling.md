---
layout: default
---

# Opel Rolling Protocol

**Rust module:** `src/protocols/opel_rolling.rs` (KAT's own decoder and encoder)
**Reference:** none reads this signal. The name is KAT's.

## Overview

An old two-button Opel remote key: lock and unlock on one rubber pad, a red LED beside it. The board is printed
`19.10.00 433MHz` and `E 02648808` on the button side and `28 5442 01` on the battery side (a CR2032).
433.89 MHz, on-off keying. Worked out from four recordings of one remote that a tester sent in
(`IMPORTS/OPEL/opel_two_button_remote_*.sub` and two files in `IMPORTS/OPEL/more/`): about 620 presses, 399 of
them read.

Which car the remote belongs to is not known, and the board's numbers match no part found so far. The
two-button remote head of the Astra G and Zafira A of 1998 to 2005 is Bosch's RK 203 (Opel 93176615), also
433 MHz with a CR2032. But [a write-up of a 2001 Astra's RK 203](https://www.burningimage.net/rke) describes
another frame: Manchester, a 24-bit serial, 40 changing bits and 16 button bits. So this remote is another
unit, or an earlier kind of that one.

The frame carries a 12-bit press counter under one of 16 masks, which the remote picks anew for every press,
and a check nibble. KAT decodes it and sends the next press.

Not [Opel V0](opel_v0.md) and not [Opel V1](opel_v1.md): those are other remotes.

## Timing / modulation

The bit is the length of a whole period, pause first, in units of 150 µs.

| Parameter | Value | Notes |
|-----------|-------|-------|
| 0 | pause 2 units, carrier 2 (600 µs) | a Flipper recorded 325 and 276 µs |
| 1 | pause 3 units, carrier 3 (900 µs) | 474 and 425 µs |
| Sync | carrier 2 units, pause 4, carrier 4 | 276, 623 and 574 µs |
| Lead | periods of 600 µs, carrier first | 57 before the first copy, 4 before the others |
| Press | the frame three times | the pause between copies is 0.8 to 1.7 ms |

The tester's analyzer showed 301 and 449 µs for the two half periods. The pause between copies depends on the
number of 1 bits in the frame: the remote starts its copies on a grid of about 1.02 ms. KAT sends 300 and
450 µs half periods and a pause of 1.2 ms, and reads pulses within 60 µs of two, three and four units.

## Frame (60 bits, 15 nibbles)

| Nibble | Field |
|--------|-------|
| 0 | `A` Unlock, `B` Lock |
| 1..6 | the remote's serial, 24 bits (`71597D`) |
| 7..12 | six nibbles of a plain value, masked |
| 13 | `r`, inverted: the nibble the remote picks for the press |
| 14 | check |

**The mask.** `r` is added to each of the six plain nibbles, without a carry from nibble to nibble. Then some of
the six are inverted. Which ones depends on `r` alone:

| `r` | Inverted (nibble 7 first) | `r` | Inverted |
|-----|---------------------------|-----|----------|
| 0 | `010101` | 8 | `100011` |
| 1 | `010110` | 9 | `101101` |
| 2 | `011001` | A | `100111` |
| 3 | `011010` | B | `100100` |
| 4 | `100101` | C | `111100` |
| 5 | `100110` | D | `011101` |
| 6 | `101001` | E | `011110` |
| 7 | `101010` | F | `110101` |

For `r` below 8 the row is the three bits of `r`, each written as the pair `01` or `10`. The rows from 8 on
follow no rule found so far. Each row was seen in 15 to 35 frames.

**The plain value** of the recorded remote is `ACA` and a 12-bit counter. The counter goes up by one per press,
whichever button: 0x2F7 to 0x57F across the four recordings, and in the 228 steps between presses read in the
unlock recording it follows the press count exactly. `r` shows no order from press to press.

**The check.** Start with 0. For each of the 14 nibbles before the check, in order: rotate the value left by
one bit, then XOR the nibble in. The check nibble is the result inverted. It is right in all 399 frames.

### While a button is held

After the press's three copies the remote sends a short frame every 40 ms for as long as the button is held:
two periods of 600 µs, the sync, 32 bits. The 32 bits are `9`, the serial, and a check nibble made the same way
without the last inversion: `971597D6`. It carries no counter and is the same for every press. Only held
presses of Lock were recorded.

## Decoder

A state machine over the pulses: one period of 600 µs or more of them, the sync, then periods. A period is a
bit when its pause and its carrier are of one length. After 60 periods the frame is read if its check nibble
is right. A frame of 32 periods that ends in a pause is a held frame if it starts with `9` and its check is
right.

KAT starts every decoder afresh after a frame, so each of a press's three copies is read on its own.

## Field mapping

`data` = the 60 bits, `data_count_bit` = 60, `serial`, `button` = nibble 0 (`A` Unlock, `B` Lock),
`counter` = the 12 bits, `extra` = the plain value (24 bits, `ACA2F7` for counter 0x2F7), `crc_valid` true.
The held frame: `data` = its 32 bits, `data_count_bit` = 32, `button` = `9` ("Held"), no counter, not sent.
No key file: no firmware has a format for it.

## Transmit

KAT sends the remote's next press: the counter plus one (the 12 bits wrap, the three nibbles before them stay),
`r` plus one, the serial as captured. Lock and Unlock send `B` and `A`; any other command sends the captured
button again. The upload is the remote's press: 57 periods of lead, the frame, and twice more with four
periods. AM650. A held frame is not sent.

For any `r` and counter the frame KAT builds reads back, and the remote's own frames are rebuilt bit for bit
from their fields. **Nothing has been sent to a car.**

## Not known

- The car, and so the years and models.
- Whether the table of inversions and the three nibbles `ACA` are the same in every remote of this kind, or
  belong to this one. If they differ, KAT reads another remote's serial and button, a wrong counter, and
  sends frames its car will not take.
- Whether the counter is longer than 12 bits: `ACA` may be part of it.
- Whether a receiver minds which `r` a press uses.

## Validation

- `opel_two_button_remote_lock_x5_unlock_x5.sub`: four of the ten presses are read (counters 0x56E, 0x571,
  0x572, 0x574); the recording is weak.
- `opel_two_button_remote_lock_held_x5_unlock_x5.sub`: five held presses of Lock (0x576 to 0x57A), 214 of
  the 296 held frames behind them, and four of five presses of Unlock (0x57C to 0x57F).
- `more/opel_two_button_remote_lock_x300.sub`: 157 presses read, 0x2F7 to 0x420, and three held frames.
  `more/opel_two_button_remote_unlock_x300.sub`: 229 presses, 0x421 to 0x54E. In stretches of the unlock
  recording where no press is read, two other decoders each list a frame that is none: Hitag2 twice and
  Microchip HCS361 once.
- No other file in IMPORTS changes.
- Unit tests: the fields of two recorded frames and the held frame; every `r` with a spread of plain values
  builds a frame that reads back; one wrong bit anywhere fails the check; what the encoder sends reads back,
  also with a Flipper's pulse skew; a period whose halves differ, a sync without a lead, a frame cut short;
  held frames and the flush at the end of the input; the next press; the recording.
