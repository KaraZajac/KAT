---
layout: default
---

# Renault Koleos (HITAG2, read from the key's firmware)

**Status:** read from the key's own firmware, run in KAT's [MRK II emulator](mrk2.md); no recording of a real key yet, and KAT has no decoder for it
**Key:** Renault part `285970036R`, four buttons, 433 MHz band, [the key's page](remote-renault-koleos-285970036r.html)
**Chip:** NXP [PCF7941](chip-pcf7941.html) (ID46: radio remote and HITAG2 transponder in one chip)
**Board:** Continental `A2C53288852`
**Cipher:** HITAG2, 48-bit key; the same cipher as [Renault V1](renault_v1.md) and [Fiat V1](fiat_v1.md)
**Tools:** [`koleos_demo.py`](tools/mrk2/koleos_demo.py) reproduces this page from the published firmware

## Overview

The four-button key of the first Renault Koleos (built by Renault Samsung on a Nissan platform, sold from 2008)
signs each press with HITAG2. KAT read how it does so from the key's own firmware: the 4 KB program of its
NXP PCF7941, from the renewal files in KAT's [Firmware](firmware.html) catalog, run instruction by instruction in
an emulator of the chip's [MRK II](mrk2.md) core, with a button held down, the chip's EEPROM and its HITAG2 unit.
The keystream the emulated key sent is HITAG2's, bit for bit, as KAT's own HITAG2 code computes it.

The frame is 16 bits longer than [Renault V1](renault_v1.md)'s and is laid out differently: it carries a header
byte and a command byte that Renault V1's frame has not. KAT's decoders do not read it. No recording of the key
is in KAT's [library](captures.html); one would settle the timing and polarity on air, after which KAT could
decode the key and, for a key whose secret is known, send fresh codes.

## Frame

Manchester. A press starts with a long run of alternating levels (a wake-up preamble, about 1,080 bit
periods), a pause, then the frame; while the button is held the frame is sent again after a shorter pause, its
repeat count raised each time.

Each frame opens with a sync: a run of 0 bits closed by `0 1`, so that its last 16 bits read `0x0001`, as
Renault V1's header does. 104 bits follow, most significant bit first:

| Byte | Content |
|------|---------|
| 0 | header `0010 tt b0`: `tt` the frame type (`01` for an ordinary press), `b` set while a button is down; `0x26` in every frame the emulation sent |
| 1–4 | the chip's 32-bit ID, factory-set (EEPROM page 0) |
| 5 | command: one bit per button, `01`, `02`, `04`, `08` |
| 6 | the repeat count in bits 7–2 (1, 2, 3 … while the button is held), counter bits 9–8 in bits 1–0 |
| 7 | counter bits 7–0 |
| 8–11 | the 32-bit HITAG2 keystream |
| 12 | XOR of bytes 0–11 |

The firmware sends a 1 bit as LOW then HIGH (taking a set modulator bit as carrier on). That is the opposite of
the convention KAT's Renault V1 decoder reads, so to that decoder the bytes would come out inverted.

**Timing.** Each half bit lasts 43 ticks of the chip's Timer 1; the pause between repeats is 1,118 ticks and the
one after the wake-up preamble 1,161. Timer 1 counts a clock derived from the crystal, which the emulator does
not time, so the bit rate on air is not known yet.

The firmware's frame table also holds a second layout that sends 62 bits of the same buffer (header, ID,
command and 14 more bits, no keystream) with its own check; when the firmware sends it was not worked out.

## Rolling code

The chip's Calculation Unit runs HITAG2 bit by bit under the firmware's control (function `374` of the
program):

1. 32 bits of the chip's ID are loaded into the 48-bit register,
2. then 16 key bits, from EEPROM page 5 (its bytes 2 and 3),
3. then 32 rounds through HITAG2's filter, each fed a key bit from EEPROM page 4 XORed with a bit of the IV,
4. then 32 output bits: the keystream in bytes 8–11.

In KAT's terms (`hitag2::run48`, the function Renault V1 and Fiat V1 use): register = ID ‖ page 5 bytes 2–3,
key bits = page 4, **IV = the 24-bit counter ‖ the command byte**. As a six-byte HITAG2 key in KAT's order:
page 5 bytes 2–3, then page 4. The factory template holds HITAG2's default key there, `MIKRON` (`4D 49 4B 52`
in page 4, `4F 4E` in page 5); a key learned to a car holds its own. The frame carries only the counter's low 10
bits, so a receiver has to keep the rest, as with Renault V1.

**Test vector** (a made-up identity, not a real key's): ID `1A2B3C4D`, EEPROM page 5 bytes 2–3 `5A6B`, page 4
`C0FFEE12`, counter `00002E`, the first button: IV `00002E01`, keystream `D09584E7`, frame
`26 1A2B3C4D 01 04 2E D09584E7 6B`. The other three buttons give keystreams `B0E4152F`, `91C0401C` and `B055B00F`.

## Buttons and battery

The four buttons are port inputs P1.0–P1.3 (active low, debounced in software) and set command bits `01`,
`02`, `04` and `08` in that order; which of them locks and which unlocks is not in the firmware. Before sending,
the firmware measures the battery with the chip's voltage comparator and sends nothing when the reading is too
low.

## How it was read

- **Program.** The disassembly ([`mrk2.py`](tools/mrk2/mrk2.py)) shows the HITAG2 sequence (`374`), the frame
  builder (`1DF`), the XOR (`40B`, `XOR R3,@R0+` over 12 bytes) and a table-driven transmitter (`529`–`5AB`)
  whose frame layouts sit in a table at code address `0F00`, read with `MOVC` (the record `3C 4E 68 00`
  sends 0x68 = 104 bits from RAM `4E`, the frame buffer).
- **Run.** The emulator ([`mrk2emu.py`](tools/mrk2/mrk2emu.py)) starts the firmware at its reset vector,
  holds a button, models the EEPROM, the battery comparator, the timers and the Calculation Unit
  ([`hitag2cu.py`](tools/mrk2/hitag2cu.py), from the PCF7922 data sheet's description of its functions), and
  records what goes to the modulator.
- **Check.** The keystream in each emulated frame equals HITAG2 computed independently from the ID, key and
  IV. Only the published firmware is needed: `python3 koleos_demo.py`.

## Related

- [Renault V1](renault_v1.md) and [Fiat V1](fiat_v1.md): the PCF7946-style HITAG2 frame (ID, 4-bit button,
  10-bit counter, 32-bit hop, 2 tail bits, XOR), behind the same `0x0001` header.
- A Nissan key with Continental board `A2C32801600` (8 KB, in the [Firmware](firmware.html) catalog) has the
  same bit routine in its transmitter; it has not been run yet.

## Recordings wanted

A recording of this key (Renault part `285970036R`, first Koleos), a few presses of each button, would let KAT
read it. [Recording a remote](recording.md) says how.
