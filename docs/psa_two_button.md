---
layout: default
---

# PSA 2-Button Protocol

**Rust module:** `src/protocols/psa_two_button.rs` (KAT's own decoder, decode only)
**Reference:** none reads this signal. The name is KAT's.

## Overview

A two-button flip key for Citroën and Peugeot, made by Valeo: lock and unlock on the key head, a blade in
front. The case is moulded for both makes (`73841867 LION` and `CHEVRONS 73842087` inside the back half, `738
10594` inside the front half, `Valeo` in both). The board carries a 20-pin NXP chip with a customer's marking
(`22A1509`, `AZS5702`, `TnD3011`, `9221509`, read from a picture), a 13.560 MHz crystal (times 32 is
433.92 MHz) and two switches. 433.9 MHz, on-off keying.

Worked out from two recordings of one Citroën key that a tester sent in, 50 presses of unlock and then 50 of
lock (`IMPORTS/GROUPE PSA/citroen_two_button_valeo_key_unlock_x50.sub` and `..._lock_x50.sub`). Which
Citroën, and of which year, is not known; the NXP logo on the chip puts the key at 2006 or later.

The frame is 76 bits: a byte that never changes, a button nibble, and a 64-bit code that changes with every
press. **Decode and display only**: KAT cannot undo the code or work out the next one. The code is not a
strong cipher's output, though; [what it gives away](#the-code) is below. A 2005 Peugeot 206 remote sends
[the same kind of code](#the-same-code-in-a-peugeot-206s-frames) in PSA's 80-bit frame.

Not the frame of [PSA](psa.md): that one is 80 bits behind a preamble, at 250 µs (or 125 µs) per half bit.

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| Half bit | 207 µs | 2415 bit/s; a carrier pulse and the pause behind it are 414 µs together. A Flipper's receiver gave 120 to 240 µs of carrier and 200 to 320 µs of pause for one half bit each |
| Line code | Manchester | carrier then pause is a 1 |
| Before the frame | about 78 ms of carrier, 6.9 ms of pause | the Flipper recorded the carrier as pulses of about 195 µs with gaps of 60 to 100 µs: its receiver does not hold a long carrier as one pulse |
| Frame | half a bit of carrier, 76 bits, 2.5 ms of carrier | 31.7 ms of bits; the closing carrier is twelve half bits |
| Press | once | 120 ms; the tester's presses were 0.23 s apart. What a held button sends is not known |

The tester's analyzer showed 207 and 414 µs, a 2523 µs carrier pulse and 433.909 MHz.

## Frame (76 bits)

| Bits | Field |
|------|-------|
| 8 | `CE` in all 100 presses |
| 4 | button: `C` (`1100`) Unlock, `3` (`0011`) Lock |
| 64 | the code |

No part of the frame is a remote's ID in the clear. The code has no check over its bytes as they stand: the
sum and the XOR of its bytes and of its nibbles, and a CRC-8 of every polynomial over it, come out alike in
no more frames than chance gives. (Three bits inside it are a check on the counter: [the code](#the-code).)

## The code

Eight bytes, `c0` to `c7`. No two presses share them. They are not a linear function of anything that counts:
the 96 codes read span all 64 dimensions, and no bit of a code follows from the code before it. But the
bytes are built from few parts, with XOR only: the press counter's low byte, a second byte that changes
with the press (`y` below), bytes that change with the press and cancel each other, and values that stay.

**The press counter is in the code.** 30 pairs of frames differ by just two byte values, each of them in
several of the eight bytes (and their XOR where both meet): for instance

```
1C 20 05 98 F1 A6 50 87
2A 8C 05 34 5D 0A 50 87      XOR:  36 AC .. AC AC AC .. ..
```

In every one of the 30 pairs, one of the two values is the XOR of two numbers that are as far apart as the
two presses: the unlock recording counts from `14` to `45`, the lock recording from `46` to `77` (the tester
pressed unlock 50 times, then lock 50 times). Above, the two presses are the 9th and the 23rd of the unlock
recording, counters `1C` and `2A`, and `1C` XOR `2A` is `36`. So a frame holds the low byte of a press
counter, XORed into one or more of its bytes; in 27 of the 96 frames one byte is the counter itself (as `c0`
above; 3 would be chance). The other value of a pair is the difference of `y`.

**Where the two sit.** The 30 pairs fall into 19 groups of frames built alike (43 frames; a group's frames
are any number of presses apart, and 18 of the pairs have one frame in each recording, so the button does
not decide the group). In every group the counter and `y` stand opposite each other, four bytes apart; the
bytes two further on hold nothing more, or the counter and `y` again, or the XOR of the two; and of the four
bytes in between, one to three in a row (counted in steps of two) hold `y`. Where the counter stands differs
from group to group: the whole code is turned by a number of bytes that changes with the press.

**Left half XOR right half.** `D` = `c0^c4`, `c1^c5`, `c2^c6`, `c3^c7` (`D0` to `D3`). What else changes with
the press drops out of these four bytes: they hold the counter, `y` and values that stay, and nothing more. 89 of the
96 frames fall into 24 arrangements, each seen two to nine times with the same constants. In an arrangement

- one byte of `D` is the counter XOR `y` XOR a constant,
- a neighbour of it is `y` XOR a constant,
- and each of the other two is a constant, or has a constant XOR with the byte opposite it.

So **the counter is the XOR of two bytes of `D` and a constant.** The 23 frames whose `D` ends `63 ED` are
the largest family: there the counter is `c0^c4^c1^c5` XOR `1A` (9 frames), `A5` (5), `8E` (5) or `31` (4);
in the five with `8E`, `c0` or `c4` is the counter itself. The constants of the arrangements are XORs of
one another (`1A`, `31`, `A5`, `8E` differ by `2B` and `BF`; `63 ED` and `48 C6` by `2B 2B`; `86 4F` and
`24 DB` by `A2 94`, as do `86 55` and `24 C1`): a handful of bytes that stay, most likely the key's own.
The seven frames outside the 24 arrangements each stand alone with their constants.

| `D`, with `.` for a byte that changes | Frames of 96 |
|---|---|
| `.. .. 63 ED` | 23 |
| `.. .. 48 C6` | 5 |
| `86 .. .. 4F` | 9 |
| `.. 86 55 ..` | 6 |
| `.. 24 C1 ..` | 5 |
| `.. .. C1 ..` with `D1^D3` = `F4` | 10 |
| `24 .. .. DB`, `0F .. .. F0`, `AD .. .. 64`, `.. AD 7E ..` | 2 each |

**One formula reads the counter.** With `A` the byte of `D` that holds the counter and `y`, `Y` the byte
that holds `y`, and bit 7 the highest:

```
counter = A ^ Y ^ (94 if a) ^ (2B if b) ^ k
      a = A7 ^ Y7
      b = A6 ^ A5 ^ A3 ^ A1 ^ A0 ^ Y6 ^ Y4 ^ Y1
```

`k` is one constant for an arrangement with its constants (18 of them: the four constants of the `63 ED`
family above are one `k`, `31`, with `94` and `2B` switched in by `a` and `b`). The formula gives one `k` per
arrangement in all 89 frames. Fitted without it, as one unknown mask per counter bit on the 16 bits of `A`
and `Y` and one unknown constant per arrangement, the 89 frames leave 57 equations to spare for each of the
seven counter bits and contradict none; with each frame left out in turn and read from the fit of the
others, 89 of 89 counters come out right; with the counters shuffled, no bit fits. Two parities of the two
bytes are fixed per arrangement as well (`A7^A4^A3^Y6^Y4^Y3` and `A6^A5^A2^A0^Y4^Y3^Y2^Y0`).

So the code is linear over single bits, not only over whole bytes: within an arrangement each bit of `D` is
an XOR of counter bits and of bits that change with the press (the 23 frames of the `63 ED` family span 11
dimensions of the 32 bits of `D`, and each counter bit is an affine function of `D` with 11 equations to
spare). `94` and `2B` are two of this key's constants: `2B` is also what separates `63 ED` from `48 C6`, and
`94` what separates `4F` from `DB` and `55` from `C1` in the table above. What is not linear is the
placement: which bytes of `D` are `A` and `Y`.

**How `D` is made up.** `D0` and `D2` are one lane, `D1` and `D3` the other. In a lane one byte is a
constant and the other changes, or both change and their XOR is a constant. Seven bytes of the key give all
those constants: a base for each place (`86` for `D0` and for `D1`, `55` for `D2`, `4F` for `D3`), `9F` for
a lane whose two bytes both change (the XOR of the two, with their two bases taken out), and three switched
values, `A2`, `94` and `2B`, XORed into the constants of some frames. With them every one of the 96 frames
reads lane by lane (in two of them one lane reads two ways); nothing else occurs.

**A switch moves the counter.** Of two frames that differ in a switch, one has the counter in the even
lane's changing byte and the other in the odd lane's: so it is for `2B` in the six pairs of forms that show
it, and for `A2` with `94` in the three that show it. A switch XORs its value into the lanes' constants and
makes `A` and `Y` change places.

**One frame read by itself.** A table from the lane form (which byte of each lane is the constant, and which
values are switched into it) to the places of `A` and `Y` and the constant `k`, made from the 89 frames, and
the formula above read a frame's counter from the frame alone: 88 of the 96 come out right. Three read
wrong or two ways (their lane form occurs with the counter in either lane), and five have a form the table
lacks. Two of the frames read right, counters `45` and `4F`, had no part in the table: their arrangements
stood alone among the 96. This reads one key, whose seven bytes and table were worked out with the counters
known; what the table's `k` are made of is still open.

**Three bits of `y` are a check on the counter.** With `y7` the highest bit of `y` and `n6` to `n0` the
low seven bits of the counter (the eighth never changed in the recordings):

```
y7 ^ y6           = n4 ^ n3
y7 ^ y0           = n4 ^ n2 ^ n1
y7 ^ y5 ^ y4 ^ y3 = n6 ^ n5 ^ n4 ^ n3 ^ n2 ^ n0
```

each up to a constant of the arrangement. The three hold in all 30 pairs they were found from, and then in
every frame of all 24 arrangements (the `y` byte of `D` taken as it stands; the 15 arrangements with three
or more frames hold 71 of the 89). The other five bits of `y` follow no rule KAT found: they are no linear
function of the counter and the button.

**What picks the arrangement, and its `k`,** is not known. All 23 frames with `63 ED` have a counter with an even number
of 1 bits; all 15 with `D2` = `C1` an odd number, and so have the 5 with `48 C6`; but no other linear function
of the counter's bits, no polynomial of them up to degree 3, and no field of up to three bits of a byte of
the code (or of the XOR of two of its bytes) sorts the frames. Nor is it known where the bytes that drop out
of `D` come from: they differ from one group of frames to the next.

**A relative.** The XOR frame of [PSA](psa.md) (mode 0x23) is built the same way: serial and counter bytes
XORed into each other in a fixed chain, and one byte that changes with the press XORed into all of them (the
byte the references call a CRC; in a Delphi remote's 30 frames it is no linear function of the counter
either). Read like that, this older code is the same idea with the layout changing from press to press: an
obfuscation with check bits, not a cipher under a secret key. If that is right, the constants above are the
key's serial, and a remote that knows the layout rule could be continued from one capture. The rule is what
is missing.

**What would tell more.** The recordings cover 100 counts of one byte, without a carry out of it: a
recording of some 300 presses in a row would show whether the code holds a higher byte of the counter (one
of the constants would change at the carry).

## The same code in a Peugeot 206's frames

A write-up of 2011 ("RKE Analysis", burningimage.net, Andy Carvell) published 58 frames of a 2005 Peugeot 206
remote, 27 of unlock and 31 of lock, with the source of the decoder that read them. By that source the signal
is the frame of [PSA](psa.md): Manchester at about 250 µs per half bit behind a preamble. The 88 bits the
decoder prints are the end of the preamble (`FD`), the frame's first two bytes (`1C AA` in all 58; the write-up
takes the three bytes for a serial) and a 64-bit block. The block is neither of PSA's two kinds: KAT's XOR
check and key search open none of the 58 (one passes the XOR check by chance, one gets a chance hit from the
search).

It is the code of this page. `D`, the left half XOR the right half, shows the same families of constants,
with this remote's values in them:

| `D`, with `.` for a byte that changes | Citroën key | Peugeot 206 |
|---|---|---|
| `.. .. a b` | `63 ED` (23 of 96 frames) | `7E 9E` (12 of 58) |
| `.. k1 kc ..` | `86 55` (6) | `B7 4B` (8) |
| `k1 .. .. kb` | `86 4F` (9) | `B7 55` (7) |
| `k2 .. .. kb'` | `24 DB` (2) | `7C AB` (7) |
| `.. k2 kc' ..` | `24 C1` (5) | `7C B5` (5) |

and in both remotes `kb ^ kb'` = `kc ^ kc'` (`94`; `FE`), `a` = `k1 ^ k2 ^ kc'` and `b` = `k1 ^ k2 ^ kb`. So
the constants are each remote's own, five bytes of them in these families, and how they are combined is
common: one construction, in a 76-bit frame on the Citroën key and in PSA's 80-bit frame on the
206.

The 206's frames add two facts and differ in two.

- **The counter in the clear.** 16 of the 58 frames hold a byte `4B` with the press counter opposite it, four
  bytes away: `12` to `1F` in unlock frames, `20` to `2F` in lock frames. (So the write-up's lists are not in
  the order of the presses; 27 presses of unlock and then 31 of lock, counters `05` to `1F` and `20` to `3E`,
  would fit.) In 14 of the 16 the counter is `D0 ^ 4B`: no `y` and no switched constants in it, where the
  Citroën key needs the formula above.
- **The button must be in the code**: the frames of both lists start with the same two bytes, and frames of
  the two lists are built alike (same constants, same places).
- In the Citroën key the byte opposite the counter is `y`, which changes; in the 206's frames it is the
  constant `4B`.
- Frames built alike differ in three byte values, not two: 20 pairs do, none differs in two. Besides the
  counter, two more values change with the press. Where the counter is `D0 ^ 4B`, they stand opposite each
  other and their XOR is the one more byte of `D` that changes. That byte is no linear function of the
  counter (between the frames of counters `19` and `1B` it changes by `DF`, between those of `20` and `22` by
  `A8`), and the three check relations of the Citroën key's `y` do not hold for it.

The 206's `D` reads lane by lane like the Citroën key's, all 58 frames of it, with its own bytes: bases `B7`
(`D0`, `D1`), `4B` (`D2`) and `55` (`D3`), `C6` for a lane of two changing bytes, and the switched values
`CB` and `FE` (a third is not seen).

**More than one frame a press.** The counter is certain in 21 frames (the 16 with `4B` beside it, and five of
one lane form where it is `D2 ^ 4B`, which two pairs of frames built alike confirm). Three counters stand in
two frames each (`1F`, `25`, `2D`), frames that are otherwise unlike: the remote sends a press more than once
and arranges each frame anew. The 27 and 31 frames are then some 14 presses of unlock (`12` to `1F`) and 16
of lock (`20` to `2F`). For the other frames the lane form gives the byte that holds the counter (pairs of
frames built alike confirm it for seven forms), but not that form's constant; if no press was caught more than
twice the constants follow for 28 more frames, which is an assumption.

What picks the arrangement of a press is as open for the 206 as for the Citroën key.

## Decoder

A burst (`src/protocols/burst.rs`) is a run of pulses of one or two half bits, counted with the receiver's
skew taken out. It is a frame when it begins behind a pause with the half bit of carrier, holds exactly 76
Manchester bits without a violation, and is ended by a carrier pulse of 2.0 to 3.3 ms (2.5 ms, and 2.7 ms
when the frame's last bit is a 0, whose second half is carrier too). There is no checksum to test, and the
`CE` is not tested: one key cannot tell whether every key sends it.

## Field mapping

`data` = the code, `extra` = the 12 bits before it (`CEC` or `CE3`), `data_count_bit` = 76. `button` = the
nibble. `serial` None, `counter` None, `crc_valid` false, `encoder_capable` false. No key file. A capture can
be sent again as it was recorded (Replay).

## Not known

- The car and its year.
- Whether `CE` is the same on every key of the kind.
- What picks a press's arrangement of the code, and what the constants `k` of the arrangements are made of.
  The counter of a frame can be read for a key whose seven bytes and table of `k` are known (above); a key
  KAT has not seen with its counters cannot be read.
- The five free bits of `y` and the bytes that cancel between the halves: whether they are chance or a
  check, which decides whether a remote could be continued.
- What the constants are made of: two remotes have different ones in the same places, so they are the
  remote's own (its ID, or a key), but not which bytes of it.
- Whether the code holds more of the counter than its low byte, and whether it holds the button.
- What a held button sends.

## Validation

- `citroen_two_button_valeo_key_unlock_x50.sub`: 41 of 50 presses read; `..._lock_x50.sub`: 49 of 50. Every
  frame has the head `CE`, the button of its recording and a code of its own. The presses not read lost bits
  to the receiver in the recording (most of them among the first of the unlock recording, where the pauses
  inside the frame came out up to 1.4 ms long).
- No other file in IMPORTS changes, and no other decoder reads these two.
- Unit tests: a press; a frame that ends with a 0 bit; carrier pulses 70 µs short and pauses 70 µs long; the
  gate (the closing carrier's length, a bit more or less, a pulse of another length, no closing carrier,
  another speed); the recordings.
