---
layout: default
---

# Volvo S40 Protocol

**Rust module:** `src/protocols/volvo_s40.rs` (KAT's own decoder, decode only)
**Reference:** none reads this signal. The name is the recording's. The air format is that of
[Chrysler V0](chrysler_v0.md).

## Overview

The remote of a 1999 Volvo S40: 433.92 MHz, on-off keying. Worked out from one recording in the owner's
key-captures collection, `IMPORTS/VOLVO/Volvo S40 1999 unlock.sub` (Flipper preset AM650): two presses, 2.26 s
apart, of the button the file name calls unlock.

The signal is Chrysler V0's, with 64-bit frames instead of 80, and the frame is built by the rule that the frames
of the Chrysler PT Cruiser 2006 fob follow ([Chrysler V0](chrysler_v0.md#frame-80-bits)):
a sum byte, then the low byte of a press counter, then six bytes each XORed with that counter byte, sent
backwards when the sum is odd. Nothing in it is encrypted. The decoder checks the sum and shows the counter and
the six bytes. **Decode and display only**: with one button recorded, which byte is the button is not known.

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| Bit | 4 ms | carrier 300 µs and pause 3.7 ms is a 0; carrier 600 µs and pause 3.4 ms a 1. The recording: 326 / 3732 µs and 624 / 3424 µs on average |
| Preamble | 24 pairs | carrier 300 µs, pause 3.7 ms |
| Before the frame | a short carrier, 15.9 ms of pause | |
| After the frame | a short carrier, 15.9 ms of pause, the frame again | the second copy has no preamble |
| Press | one frame, sent twice | 0.69 s |

This is Chrysler V0's timing. The PT Cruiser's fob in `IMPORTS/chrysler`, recorded the same way, sends the same
widths, its bits 0.5 % shorter and its gaps 15.6 ms long; only the frame's length differs.

## Frame (64 bits)

Each byte goes out LSB first. The remote holds seven bytes: `n`, the low byte of a counter that goes down by one
with every press, and six more, `p1` to `p6`.

| Byte | Content |
|------|---------|
| 0 | the sum `n + p1 + … + p6` (mod 256) |
| 1 | `n` |
| 2..7 | `p1 ^ n` … `p6 ^ n` |

When the sum is odd, bytes 1 to 7 go out backwards: the 56 bits last bit first. Read MSB first, as a receiver
sees them, they are then the same seven bytes in reverse order. Since the sum goes down by one with every press
(while the six bytes stay the same), the two orders alternate: Chrysler V0's payloads A and B.

The recording's two presses:

| Press | Received | Sum | `n` | `p1..p6` |
|-------|----------|-----|-----|----------|
| 1 | `2D376746E42A3386` | `B4` (180, even) | `EC` (236) | `0A 8E CB B8 20 8D` |
| 2 | `CD66CB532065E1EB` | `B3` (179, odd) | `EB` (235) | `0A 8E CB B8 20 8D` |

Press 1, read: `2D` backwards is `B4`, even, so the bytes come forwards. `37 67 46 E4 2A 33 86` backwards each
are `EC E6 62 27 54 CC 61`: `n` = `EC`, and XORed with it the rest are `0A 8E CB B8 20 8D`. Their sum with `EC`
is `3B4`, so `B4`. Press 2: `CD` backwards is `B3`, odd, so `66 CB 53 20 65 E1 EB` are the bytes in reverse
order: `n` = `EB`, then `E1 65 20 53 CB 66`, which XORed with `EB` give the same six bytes.

## What the two presses establish

- **The rule.** Both frames are made by it, all 64 bits. A frame of random bits passes the sum once in 256 tries;
  that the second press, sent backwards, gives back the first's six bytes happens by chance once in 2^48.
- **The counter.** `n` is one lower in the second press, and so is the sum: as in the PT Cruiser's recordings,
  where `n` is one lower from every press to the next (where a recording missed a press or two, lower by as many).
- **Not encrypted.** The six bytes are the same in both presses: everything that changes is the counter.

## Not known

- **The button.** Only one button was recorded. In the PT Cruiser's frame `p5` is the button, and `20` is its
  Lock (`10` Unlock); here `p5` is `20` in a recording named unlock. This remote may number its buttons another
  way, or lay out its bytes another way.
- **`p6`.** In the PT Cruiser's frame `p6` is the counter's high byte, one lower each time `n` passes from 0 to
  255. Here it would make the counter `8DEC`; two presses cannot show it change.
- **`p1..p4`**: this remote's ID, or bytes every such remote sends. In the PT Cruiser's frame they are the fob's
  fixed bytes, and two of them come again at the end of its longer frame.
- The remote's maker, part number and FCC ID, and which other cars use this frame.

## Decoder

A frame is read when there are 16 or more preamble pairs, then a short carrier and a pause of 12 to 20 ms,
exactly 64 bits (a carrier of 150–450 µs is a 0, of 451–750 µs a 1; each pause 3.0–4.1 ms), then a short carrier
and a pause over 8 ms; the sum holds; and `p1..p4` are not `00000000` or `FFFFFFFF`. A press is read once: its
second copy has no preamble.

Chrysler V0's 80-bit frames are not read: a 65th bit ends the frame unread, and a Chrysler V0 frame cut after 64
bits passes the sum only by chance, once in 256, since its own sum runs over two bytes more.

## Field mapping

`data` = the 64 bits as received, `data_count_bit` = 64, `counter` = `n`, `serial` = `p1..p4` (`0A8ECBB8`),
`extra` = `p5 << 8 | p6` (`208D`), `button` None, `crc_valid` true (the sum), `encoder_capable` false. No key
file.

## Transmit

None. The rule makes the next press of the same button: `n` one lower, the same six bytes, the new sum, the order
the sum's parity gives. No recording has shown a press KAT made that way, and with one button recorded KAT could
send only that button, so it sends nothing.

## What would settle it

Ten presses of each button, each button in its own file, recorded as before (AM650 at 433.92 MHz), close to the
remote; the third button too if the remote has one. The byte that differs between the files is the button. Every
press is a test of the rule: KAT reads a frame only when its sum holds, and `n` must be one lower from press to
press, through both orders.

If the remote has not been pressed since the recording, its next presses of the same button are, by the rule:

| Press | `n` | Frame |
|-------|-----|-------|
| 3 | `EA` | `4D570726844A53E6` |
| 4 | `E9` | `8D64C9512267E3E9` |
| 5 | `E8` | `0D174766C40A13A6` |

## Validation

- `Volvo S40 1999 unlock.sub`: both presses read, counters `EC` and `EB`, serial `0A8ECBB8`, `extra` `208D`.
- No other file in IMPORTS changes, in the test corpus or in the `more/` folders.
- Unit tests: both presses rebuilt from `n` and the six bytes, and frames of other bytes read back either way
  round; a press read once, two presses in a row; the gate (any one bit changed, 15 preamble pairs, a gap of 11
  or 21 ms, 63 bits, 80 bits, a long carrier or none closing the frame, a pulse of another length, an ID of 0 or
  of all ones); Chrysler V0 uploads and the PT Cruiser recordings not read; and the recording. The rule itself,
  one function for both decoders, is tested on all 732 Chrysler V0 frames in `IMPORTS/chrysler`
  (`chrysler_v0.rs`).
