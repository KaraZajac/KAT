---
layout: default
---

# VW-1 Protocol

**Rust module:** `src/protocols/vw_1.rs` (KAT's own decoder and encoder)
**Reference:** none in KAT's set reads this signal. The scheme is the "VW-1" of a published study —
Flavio D. Garcia, David Oswald, Timo Kasper and Pierre Pavlidès, *Lock It and Still Lose It — On the
(In)Security of Automotive Remote Keyless Entry Systems*, USENIX Security 2016, §3.2.

## Overview

An old two-button Volkswagen Group flip key (the VW roundel on the back), worked out from three recordings
of one key a tester sent in: `IMPORTS/VAG/old_vw_remote_hold_5lock_5unlock.sub`, and the 300-press
`IMPORTS/VAG/more/old_vw_remote_lock_x300.sub` and `..._unlock_x300.sub`. 433.93 MHz, on-off keying;
628 presses, all read.

The transmitter is Hella's: the unit that holds the board is labelled `HLO 1J0 959 753 A` (`HLO` is
Volkswagen's code for Hella, and the label's edge reads `HELLA`), with European radio approvals dated 1998.
The board carries a Motorola `ZC421689CDW 734 778-00` controller on one side (board `734 613-02`, marked
`L00233A` / `C00233A`) and, on the other, an S+M `R707` SAW resonator, a 2.00 MHz ceramic resonator and the
two switches. `1J0` parts are of the Golf Mk4 / Bora / Jetta generation, so the key is from a Volkswagen of
roughly 1998–2005. The exact model year was not given with the recordings.

This is **not** KAT's [VAG](vag.md) protocol. That one is the newer Volkswagen Group frame (80 Manchester
bits, AUT64 or XTEA, at 434.4 MHz) used from about 2004. VW-1 is the oldest scheme, at 433.92 MHz, and the
only one of the group with no cipher at all.

## The signal

Pulse-width coding at 667 bit/s. One unit is about 500 µs.

| Part | Carrier / pause |
|------|-----------------|
| Start | three units of carrier, three of pause (~1500 / 1500 µs) |
| Bit `0` | one unit of carrier, two of pause (~500 / 1000 µs) |
| Bit `1` | two units of carrier, one of pause (~1000 / 500 µs) |
| Stop | two units of carrier (~1000 µs), then a pause |

A press sends the frame two or three times, the copies about 10 ms apart; presses are about 90 ms apart.
The 60 data bits are sent least-significant-bit first within each byte.

## Frame (60 bits)

| Bits (LSB-first) | Field |
|------------------|-------|
| 0..32 | the 32-bit UID, obfuscated |
| 32..56 | the 24-bit register state (bytes 4, 5, 6) |
| 56..60 | button nibble: `1` Unlock, `2` Lock |

**The obfuscation.** Let `m` be the XOR of the three state bytes (byte4 ^ byte5 ^ byte6). Each of the four
UID bytes is sent XORed with `m`; XORing `m` back in recovers the UID. The recorded key's UID is
`2AA93AC5`, the same in every one of its 628 frames. There is no checksum in the frame.

## The rolling code

The three state bytes are a 24-bit linear feedback shift register with the feedback polynomial

```
x^24 + x^7 + x^2 + x + 1
```

clocked **23 times for each new press**. The register runs its full period of 2²⁴ − 1 before repeating.
Lock and Unlock share the one register — a press of either button advances it by the same 23 ticks. Across
the three recordings the states lie on one run (lock presses, then unlock, then the held presses), each press
exactly one step on from the one before.

This is the whole "security" of the scheme, and the point of the paper: there is **no secret key**, no
vehicle- or remote-specific diversification, and no message authentication. Everything needed to produce the
next valid code — the UID, the register state, the polynomial and the step — is recoverable from a single
captured press. KAT therefore reads any frame and can compute the remote's next press: the state advanced one
step, the UID re-obfuscated with the new state, and the chosen button. Nothing has been transmitted to a car.

## While a button is held

Holding a button sends the same frame repeatedly, advancing the register about once every one to two seconds
(not once per repeat). The held recording captures ten such steps, five of Lock and five of Unlock; KAT reads
each as an ordinary press.

## Decoder

A state machine over the pulses. There is no checksum, so the gate is structural: the start pulse, exactly
60 strict pulse-width bits (each a short-carrier/long-pause or long-carrier/short-pause pair), the two-unit
stop carrier, and a closing pause of at least 2.5 ms (or the end of a recording cut right after the stop).
The last gate is what separates a VW-1 frame from another 500/1000 µs pulse-width signal that runs straight
on (for example Nice One), which VW-1 would otherwise read 60 bits out of. A frame is reported only when its register state is non-zero and its button is
Lock or Unlock.

VW-1 is placed ahead of Nissan V0 and Compustar-1WG3R in the registry; those two match a stray frame of the
recordings by chance, and the order lets VW-1 keep its own.

## Transmit

The captured remote's next press. KAT's Lock and Unlock set the button; any other command repeats the
captured button. The register is advanced one step and the UID re-obfuscated. The waveform is the start
pulse, the 60 bits, the stop and a gap, sent three times. The capture then keeps the frame sent, as for
every rolling code, so the next press sends the code after: VW-1 counts no presses, so KAT goes by its
register (KAT 1.17.0 did not, and sent the same code on every press after the first). Not tried on a car.

## Field mapping

| Field | Value |
|-------|-------|
| `data` | the 60 frame bits (bit 0 = first sent) |
| `serial` | the de-obfuscated 32-bit UID |
| `button` | the button nibble (`1` Unlock, `2` Lock) |
| `extra` | the 24-bit register state |
| `counter` | none — the state is a register, not a linear count |
| `crc_valid` | false — the frame carries no checksum |

## Not known from one remote

Whether a three-button variant of this key sends other button codes; the exact car (year and model) the key
belongs to. The polynomial and the 23-tick step are the scheme's, as the paper describes, not the key's.
