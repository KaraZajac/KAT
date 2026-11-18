---
layout: default
---

# Suzuki TS002 Protocol

**Rust module:** `src/protocols/suzuki_ts002.rs` (KAT's own decoder and encoder)
**Reference:** none reads this signal. The name is the marking on the remote's case.

## Overview

The two-button remote key Calsonic Kansei made for Suzuki: the back of the case says "Calsonic Kansei Corp.
TS002"; the board is marked `C30451A5801002` and carries a 20-pin NXP chip (marked `1A58000`) and a 433.92 MHz
SAW resonator. 433.91 MHz, on-off keying. Worked out from four recordings of one
remote that a tester sent in, 230 presses in all (`IMPORTS/SUZUKI/suzuki_ts002_unlock_lock_x10.sub` and three
files in `IMPORTS/SUZUKI/more/`). Which car the remote belongs to is not known; parts sellers list TS002 remotes
for the Suzuki Swift, SX4, Grand Vitara and Splash of about 2005 to 2014.

**The code is not encrypted.** It is the remote's ID, rotated by a press counter and XORed with a few counter
and button bits, next to the counter itself. One recorded press is enough to work out the presses that follow.
KAT decodes the frame and sends the next press.

Not the [Suzuki V0](kia_v0.md) type of Kia V0, which is another remote.

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| Unit | 256.8 µs | a frame is 440 units and takes 112.975 ms from sync to sync (three intervals in a press) |
| 0 | carrier 1 unit, pause 2 | a Flipper recorded 238 and 532 µs |
| 1 | carrier 2 units, pause 1 | 488 and 280 µs |
| Sync | carrier 4 units, pause 4 | 993 and 1044 µs |
| Frame | sync, 72 data bits, 72 one bits | |
| Press | four 0 cells, the frame four times, one unit of carrier | while the button is held the frame goes on repeating |

The tester's analyzer showed 266 and 496 µs for the short and the long pulse and 988 / 1047 µs for the sync.
KAT sends 257 µs units and reads pulses within 110 µs of one and of two units, and a sync of 800 to 1300 µs.

## Frame (72 bits)

| Bits | Field |
|------|-------|
| 8 | `0x14` in every frame of the recorded remote |
| 8 | button: `0x01` Lock, `0x02` Unlock; `0x20` is added while the button is held |
| 24 | the remote's ID (`ADE998`) |
| 32 | the code |

A held press starts like any other: its first four frames carry the plain button. From the fifth frame on the
button byte is `0x21` or `0x22`. Counter and code stay the same for the whole press (33 to 37 frames in presses
of four to five seconds).

### The code

The code holds the low eight bits of the press counter, inverted, in four pairs. Between them sits the ID
again: rotated left by the counter byte modulo 24, and XORed with a mask.

```text
code bit   31  30 | 29 28 27 26 | 25 24 | 23  22 | 21 20 | 19 18 | 17  16 | 15  14 | 13 12 11 10 | 9   8  | 7  6  5  4 | 3 2 1 0
holds     ~c1 ~c0 |  r20..r23   | r0 r1 | ~c7 ~c6 | r2 r3 | r8 r9 | ~c3 ~c2 | r10 r11 |  r12..r15  | ~c5 ~c4 |  r16..r19  | r4..r7
```

`c0`..`c7` are the counter's bits. `r0`..`r23` are the rotated ID, first bit first, XOR this mask:

| `r` | Mask bits |
|-----|-----------|
| 0..3 | `0 0 b1 b0`: the button byte's two low bits |
| 4..7 | `~c5 ~c4 0 0` |
| 8..11 | `~c1 ~c0 1 0` |
| 12..15 | `0 ~c8 ~c7 ~c6` |
| 16..19 | `1 1 ~c3 ~c2` |
| 20..23 | `0 1 0 0` |

`c8` is a ninth counter bit. It is in the mask only, and it changed once in the recordings, where the counter
byte went from `FF` to `00`. The thirteen mask bits written as plain 0 and 1 did not change in 230 presses.
They may be fixed, or hold counter bits above the ninth.

All 230 frames fit this, bit for bit. The counter went from 0x09D to 0x1A0, one step per press, in this order:
the 105 presses of Unlock (0x09D to 0x105), the 105 of Lock (0x106 to 0x16E), the ten alternating presses
(0x16F to 0x178), and, 30 presses later, the ten held presses (0x197 to 0x1A0).

## Decoder

A state machine over the pulses: the sync, then 144 cells. A cell is a 0 or a 1 when its carrier and its pause
are a short and a long pulse, in either order. The frame is read when the last 72 cells are all ones.

**Gate.** The ten mask bits that repeat the counter byte and the button must fit the ID in the frame: the code
and the ID have to agree. An ID of all zeros or all ones, which is the same however it is rotated, is not
read. KAT starts every decoder afresh after a frame, so each of a press's four copies is read on its own.

## Field mapping

`data` = the first 64 bits, `extra` = the last 8, `data_count_bit` = 72, `serial` = the ID, `button` = the
button byte (`0x01` Lock, `0x02` Unlock, `0x21` and `0x22` held), `counter` = the nine counter bits.
`crc_valid` says that the thirteen fixed mask bits are the recorded remote's; a frame that passes the gate
with other fixed bits is listed with the check shown as failed. No key file: no firmware has a format for it.

## Transmit

KAT sends the remote's next press: the counter plus one (it wraps in nine bits), the head byte, the ID and the
thirteen fixed mask bits as captured. Lock and Unlock send `0x01` and `0x02`; any other command sends the
captured button again, without the held flag. The upload is the remote's press: four 0 cells, four frames, one
unit of carrier. AM650.

The frames KAT builds for the recorded remote's presses are the remote's own, bit for bit (a unit test builds
each press of a recording from the one before). **Nothing has been sent to a car.**

## Not known

- The car, and so the years and models.
- What the head byte `0x14` is.
- Whether the thirteen fixed mask bits are the same in every TS002 remote, and whether the counter has more
  than nine bits.
- The codes of other buttons, if a remote of this kind has them.

## Validation

- `suzuki_ts002_unlock_lock_x10.sub`: ten presses, four frames each, counter 0x16F to 0x178, Unlock first. (The
  tester's file name put Lock first; the frames carry the button byte the 105 presses of Unlock carry.)
- `more/suzuki_ts002_unlock_x105.sub` and `more/suzuki_ts002_lock_x105.sub`: 105 presses each, all read.
- `more/suzuki_ts002_held_unlock_x5_lock_x5.sub`: ten presses; each is listed once with the plain button (four
  frames) and once with the held flag.
- No other file in IMPORTS changes.
- Unit tests: the fields of two recorded frames; every counter value with both buttons, plain and held, builds
  a frame that reads back; the gate (one wrong bit in the code, the ID or the button); what the encoder sends
  reads back, also with a Flipper's pulse skew; a zero among the one bits, a cell too long, a frame cut short;
  the next press; the recording.
