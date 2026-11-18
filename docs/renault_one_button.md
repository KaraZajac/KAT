---
layout: default
---

# Renault 1-Button Protocol

**Rust module:** `src/protocols/renault_one_button.rs` (KAT's own decoder, decode only)
**Reference:** none reads this signal. The name is KAT's.

## Overview

An old one-button Renault remote: the board is dated 1999 and carries an 8-pin chip (marked `B 89AN`, `S1080`,
`49004`) and a SAW resonator. 433.87 MHz, on-off keying. Worked out from one recording of one remote that a
tester sent in, 110 presses (`IMPORTS/RENAULT/renault_one_button_remote_x110.sub`).

Which car this remote belongs to is not known. One seller's listing of a one-button Renault remote quotes this
chip marking (`S1080 49004`) together with the reference `S108627BN0`; another gives `S108627BN0` as the number
on the board of Renault's one-button 433 MHz transmitter `7700438342` and lists it for the Mégane 1, Espace 3,
Safrane and Laguna 1 (its title also names the Clio, Kangoo, Trafic and Master). These are sellers' lists, not
Renault's, and the first was read from a search result only.

The frame has a part that never changes, 36 bits that change with every press, and a check byte. **Decode and
display only**: the 36 bits are a rolling code KAT cannot continue.

None of KAT's other Renault protocols: [Renault V0](renault_v0.md), [Renault V1](renault_v1.md),
[Renault V2](renault_v2.md) and [Renault/xHorse V4](renault_v4.md) are faster signals with other frames.

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| Half bit | 473.5 µs | 1056 bit/s; a Flipper recorded 455 and 492 µs (carrier, pause), 938 and 958 µs for two |
| Line code | Manchester | carrier then pause is a 1 |
| Copy | 0 bits, the frame, one more half bit of carrier | 13 zeros before the first frame of a press, 5 before the others |
| Press | two to four copies | 14 ms of pause and 8.7 ms of carrier between them |

The tester's analyzer showed 473 and 949 µs. How long the carrier is on before the first copy the recording
does not give: the receiver was still settling there.

## Frame (70 bits)

| Bits | Field |
|------|-------|
| 2 | `11` |
| 24 | `0A84DC` in all 110 frames: the remote's ID, or part of it |
| 36 | changes with every press |
| 8 | check |

**The check.** With two 0 bits in front the frame is nine bytes (`30 A8 4D C…`), and the XOR of the nine is 0. It
holds in all 110 frames.

**The 36 bits** show no structure. No two presses share them. Over the 110 presses they are linearly
independent, no bit follows a bit of the press before, nothing repeats with a period of up to 39 presses, and
no two bits within a frame go together. So this is a cipher's output, or a counter under one. There is no
counter and no button in the clear; the remote has one button.

## Decoder

A burst (`src/protocols/burst.rs`) is a run of pulses of one or two half bits. It is a frame when its
Manchester bits have no violation up to its end, its last bit is a 1 (the closing half bit of carrier and the
pause behind it), and the 72 bits before that are two or more 0 bits' worth of lead and the frame: they start
`0011`, the nine bytes XOR to 0, and the ID is not 0.

KAT starts every decoder afresh after a frame, so each copy of a press is read on its own.

## Field mapping

The nine bytes: `data` = the first eight, `extra` = the check byte, `data_count_bit` = 72. `serial` = the 24
bits behind the `11`. `button` None, `counter` None, `crc_valid` true, `encoder_capable` false. No key file.

## Not known

- The car; the models above are sellers' lists for a part this remote seems to be.
- Where the ID ends: one remote cannot tell its ID from bits every remote of the kind sends.
- The cipher.

## Validation

- `renault_one_button_remote_x110.sub`: 110 presses, each with its own frame, the ID `0A84DC` and a right check
  in every copy read. (The tester's file name says 105.)
- No other file in IMPORTS changes.
- Unit tests: a press of four copies; a frame that ends with a 0 bit; no frame without the closing carrier or
  the pause behind it; the gate (the lead, the check, a run of 0 bits, another start than `11`, an ID of 0, a
  pulse of another length); a longer run of bits that ends with a frame; the recording.
