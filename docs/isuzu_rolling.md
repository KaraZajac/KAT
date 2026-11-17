---
layout: default
---

# Isuzu Rolling Protocol

**Rust module:** `src/protocols/isuzu_rolling.rs` (KAT's own decoder, decode only)
**Reference:** none reads this signal; the model and year of the remote are not known and no FCC filing has been
matched to it. The name is KAT's.

## Overview

An Isuzu remote that sends a **105-bit differential Manchester frame, eight copies at a time**: a 64-bit code
that changes with every frame, a 28-bit fob ID and the button in the clear. No checksum is known and the cipher
is not known: **decode and display only**.

The decoder was worked out from two recordings of one remote (`IMPORTS/Isuzu/Isuzu_Lock.sub` and
`Isuzu_Unlock.sub`, 433.889 MHz, Flipper preset AM650, three presses each).

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| Half cell | 405.6 µs | 1233 bit/s |
| Line code | differential Manchester | the level changes between all cells, and once more in the middle of a cell for a 1 |
| Frame | 105 cells and half a cell of carrier, 85.6 ms | the frame's last carrier pulse is one, two or three half cells long |
| Repeats | eight copies, 105.3 ms apart | |

On-off keying. Both recordings have carrier pulses 40 to 50 µs short and pauses as much too long; the decoder
takes each burst's skew out before it counts half cells (`src/protocols/burst.rs`).

**A press** (the same in all six): a frame with the button, eight times; while the button is held another such
frame with another code, until the press ends in the middle of a copy; then 39 one bits and, 1.2 ms behind
them, a frame with no button, eight times.

## Frame (105 bits)

| Bits | Field | Notes |
|------|-------|-------|
| 0..63 | Code | another one in every frame |
| 64..91 | Fob | 28 bits, the same in all 15 frames (`4A0D2EC`) |
| 92..95 | Button | `4` in the lock recording, `2` in the unlock recording, `0` in the frame that ends a press |
| 96..100 | `00000` | in every frame |
| 101..104 | ? | changes with the frame; no CRC-4, sum or XOR of the bits before it |

Which way round the bits are is KAT's choice. Read the other way the fob ID and the code are complemented, the
buttons are `B`, `D` and `F` and the lead is zeros; this way the buttons are single bits and the lead is ones.

## Decoder

There is no sync word. A frame needs: the burst ending with 105 cells and the half cell of carrier, with a
transition between all cells; in front of them the pause before the burst, or one of three half cells (as behind
the ones that lead the last frame of a press); the burst coming out at 406 µs per half cell to within an eighth;
and **the same frame in the burst before**, less than 300 ms earlier. So the first copy of a frame is not listed.
KAT starts every decoder afresh after a frame, so eight copies are listed four times.

## Field mapping

`data` = the 64-bit code, `serial` = the 28 fob bits, `button` = the nibble (4 Lock, 2 Unlock, 0 Release: KAT's
reading of the two recordings), `extra` = frame bits 96..104, `data_count_bit` = 105, `counter` None, `crc_valid`
false (shown as "-"), `encoder_capable` false. No key file.

## Validation

- `Isuzu_Lock.sub`: 26 frames of 8 different ones: buttons 4 and 0. `Isuzu_Unlock.sub`: 25 frames of 7: buttons 2
  and 0. One fob ID in both.
- No other file in IMPORTS changes (a first version took the 250 µs lead of the two LDV T80 captures for frames
  of all ones; the speed check and the pause in front of the frame are there for that).
- Unit tests: eight copies with a last carrier pulse of one, two and three half cells; 45 µs of skew; the gate
  (one copy, frames that differ, the same frame half a second later, 104 and 106 bits, a long run of short pulses
  at 250 and at 406 µs, a pulse of three half cells inside a frame); the frame behind 39 ones; the recordings.
