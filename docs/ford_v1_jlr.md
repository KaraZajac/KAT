---
layout: default
---

# Ford V1 (JLR) Protocol

**Rust module:** `src/protocols/ford_v1.rs` (the [Ford V1](ford_v1.md) decoder reads it; decode only)
**Reference:** none. This page is KAT's own work: recordings of the fobs, and flash and EEPROM images of three cars'
keyless modules with the firmware of one of them (Sources, below).

## Overview

Jaguar Land Rover's keyless fobs of about 2015–2018 (the Keyless Entry Transmitter, part **BJ32-15K601**, an NXP
**PCF7953P** Hitag-Pro inside) send **Ford V1's 136-bit frame**: the same header, Manchester coding, 2-FSK and CRC.
Where Ford keeps its scrambled serial, button and counter, they send **8 bytes encrypted under a 128-bit key**. KAT
reads the frame, names it **Ford V1 (JLR)** and shows what travels in the clear: the fob's ID folded to 16 bits, and
whether a frame is the first send of a press or its repeat. The button and the counter are inside the encrypted
bytes, so KAT shows 0 for both, and it does not make new codes for these fobs: **decode only**, Replay of a captured
press at most.

What the recordings and the module images show, each in its section below:

- The header carries the fob's **transponder ID folded in half**: the high 16 bits XOR the low 16 bits.
- The car's keyless module learns a fob by its **ID alone**. Learning writes the 4-byte ID, a count, a state byte
  and two checksums, and nothing secret.
- One fob sits in **three different cars' modules**, and its recordings from all three look the same: nothing in
  its frames is tied to a car.
- The module has **one 16-byte key** for all its fobs. The firmware keeps it in the emulated EEPROM, **0x70 bytes
  before** the area the usual EEPROM read exports, which is why no image so far contains it.
- The **keyless-entry check** (the car asks, the fob answers) is AES-128 over a block the firmware builds from a
  constant, the key's slot, a 29-bit random challenge and the fob's ID: fully traced.
- **Button presses** are checked by the chip's XGATE co-processor with the same key. Its code is in a part of the
  flash that the images do not cover, so the cipher of the 8 bytes is still open.

## Frame

The frame is Ford V1's ([Ford V1](ford_v1.md) has the decoder's steps): 17 bytes, Manchester at 65/130 µs, 2-FSK at
433.92 MHz. The recordings were made with a Flipper's custom CC1101 preset at ±14.3 kHz deviation.

| Bytes | Field | |
|-------|-------|---|
| 0–3 | `D3 95 0E 26` | constant, the same as in Ford's own Ford V1 frames |
| 4 | flags | `00` the first send of a press, `08` the same press sent again; `09` was seen once |
| 5–6 | folded ID | the fob's 32-bit transponder ID, high half XOR low half |
| 7–14 | 8 encrypted bytes | every byte changes with every press; button and counter are inside |
| 15–16 | CRC | CRC-16/CCITT (polynomial 0x1021, start 0) over bytes 3–14 |

Six frames from the library recordings, cut into these fields (the CRC is not shown by KAT; it is computed here):

| Recording | 0–3 | 4 | 5–6 | 7–14 | 15–16 |
|-----------|-----|---|-----|------|-------|
| key 1, Lock, first send | `D3950E26` | `00` | `8472` | `F330ACEB22A6436E` | `A083` |
| key 1, Lock, repeat | `D3950E26` | `08` | `8472` | `F330ACEB22A6436E` | `020D` |
| key 1, Unlock | `D3950E26` | `00` | `8472` | `AC9213CA128C2CC9` | `3874` |
| key 2, Trunk | `D3950E26` | `00` | `E5AE` | `73F6FCFB36470E29` | `D9CF` |
| key 2, Unlock, first send | `D3950E26` | `00` | `E5AE` | `81372B19E5D41BCA` | `E597` |
| key 2, Unlock, repeat | `D3950E26` | `08` | `E5AE` | `81372B19E5D41BCA` | `4719` |

A press goes out at least twice, flags `00` then `08`, with the same 8 bytes. A repeat is therefore not a new code,
and the 8 bytes of two presses never repeat. In 92 presses of the two fobs (25 of key 1 and 20 of key 2 in the
library, 47 more of key 2 from the specialist's recordings, under Sources) no block came twice, and no byte, 16- or
32-bit part or single bit of it only rises or only falls from press to press: there is no counter in the clear.

## The fob's ID in the header

Bytes 5–6 are the fob's transponder ID folded to 16 bits: `2F47 XOR AB35 = 8472` for key 1 (ID `2F47AB35`), `A696
XOR 4338 = E5AE` for key 2 (ID `A6964338`); the chance of both matching by accident is about 1 in 4 billion. The module finds
which learned key sent a frame by folding each ID it has learned the same way. The other 16 bits of the ID are never
sent, and the fold is not secret.

## The car's keyless module

The receiver is the car's keyless module (JLR calls it the Keyless Vehicle Module, KVM, or Remote Function
Actuator, RFA): an NXP **MC9S12XE**-family microcontroller with its **XGATE** co-processor. Three modules were looked
at: part **FK72-19H440-BF** from the 2017 XJ the recordings come from, and parts **DPLA-19H440-AF** (from a 2015
Jaguar) and **DPLA-19H440-BB**. Key programmers read three images from such a module: the program flash (P-Flash),
4 KB of data flash (D-Flash) and 2 KB of emulated EEPROM (EEE). The images hold the car's VIN and are not published.

### The key table

All three modules keep their keys the same way. The EEE holds the key record twice, at offsets 0x000 and 0x080,
128 bytes each:

| Offset | Size | What |
|--------|------|------|
| 0x00 | 8 × 4 bytes | the learned transponder IDs, slot 0 to 7 (0 = empty) |
| 0x20 | 8 × 4 bytes | a counter per slot |
| 0x40 | 1 | the number of keys |
| 0x44 | 8 × 1 | a state byte per slot: `08` or `0C` for a learned key, `00` or `04` for an empty slot |
| 0x4C | 4 | the seed of the module's random-number generator (below); different in every module |
| 0x7C | 1 | a write count: the two copies differ here |
| 0x7E | 2 | checksum |

The D-Flash keeps an older copy inside its 256-byte configuration record at 0x0E00 (repeated at 0x0F00), which
also holds the VIN: the IDs at 0x0E84, the counters at 0x0EA4, the number of keys at 0x0EC4 and a byte per slot
from 0x0EC5. Its counters can lag the EEE's by thousands.

The checksums, found from the records of all three modules:

- **EEE record** (0x00–0x7D): Fletcher-16 modulo 256 with both sums starting at 0xFF; the second sum is stored at
  0x7E, the first at 0x7F. The EEE records at 0x500 and 0x580 use the same checksum.
- **D-Flash record** (0x0E00–0x0EFD): CRC-16/ARC (polynomial 0x8005 reflected, start 0), stored low byte first at
  0x0EFE.

### Learning a fob

One module was read before and after a key specialist added a fob to it as its fourth key. Exactly 16 bytes
changed, the same in each copy of the record: the fob's ID in the next free slot, the number of keys (3 → 4), that
slot's state byte (EEE `04` → `08`, D-Flash `00` → `10`), and the two checksums. Nothing else in either image moved:
**a module learns a fob from its ID alone.** This matches what the specialist reported: given the module's D-Flash
and EEE, a key is made from a fob's transponder ID.

### One fob, three modules

Key 2 of the XJ (ID `A6964338`) sits in the XJ's module as its third key, in the DPLA-19H440-BB module as its third
key and in the DPLA-19H440-AF module as its fourth, with no counter yet in any of them. The specialist reports that
it opens both of the other cars. Its recordings from all three times look alike: the same folded ID, the same
flags, new 8 bytes. What the three modules' D-Flash and EEE images have in common is the fob's ID, padding and
default settings; no run of 16 bytes with more than five different values is in all three.

## The firmware

The P-Flash image of the XJ's module (384 KB, global addresses 0x7A0000–0x7FFFFF) was disassembled with Ghidra's
HCS12X processor module; routines are named below by page:address as the CPU calls them. The image carries two
copies of the standard AES S-box, no inverse S-box, and an AES-128 encryption routine for the CPU at `F4:95D6`.

### The key

Every cryptographic routine for the fobs uses one **16-byte key at global address 0x13F6DC**, in the address range
of the emulated EEPROM (the CPU sees it at 0x0ADC with EPAGE 0xFD). A diagnostic write handler (`F7:92B5`, listed in
the module's table of data identifiers) copies at most 16 bytes from a received message to it, padding with 0xFF;
no other code writes it. It is read by:

| Routine | What it does with the key |
|---------|---------------------------|
| `F5:9F71` | the keyless-entry check on the CPU: AES-128 encryption of the challenge block (below) |
| `F5:A6F2` → `F8:90D4` | the same block, handed to the XGATE (software trigger 2) |
| `F3:A1E1` → `F8:9139` | the button-press check, handed to the XGATE (software trigger 3) |

Its key-handling routine (`F3:AB68`) also copies the 16 bytes, behind a learned key's ID, into a buffer at RAM
0x3638–0x364F, byte by byte through a pointer from `F3:BBEA`: the module hands its key on, most likely to a fob it
learns. Because the module has a single key for all its fobs and puts the fob's own ID into the block it encrypts,
a fob that works in several cars means **those modules hold the same key**: one common to these modules, or one the
programming tool writes into each.

### Keyless entry: the challenge check

When the car looks for a fob, the module encrypts this 16-byte block with AES-128 and compares the result with the
fob's answer:

| Bytes | Value |
|-------|-------|
| 0–7 | `A6 C3 2B 91 50 FE 47 DD` (constant; the first four set at `F5:A80D`) |
| 8 | the key's slot × 32, plus the top 5 bits of the challenge |
| 9–11 | the low 24 bits of the challenge |
| 12–15 | the slot's transponder ID |

The challenge is random. The module's random numbers come from a 29-bit Galois LFSR (`F5:A5FB`): shift right, and
when the bit shifted out is 1, XOR with 0x10422820 and set bit 28. Its state starts from the 4 bytes at 0x4C of the
key table, which differ from car to car. Earlier versions of this site (to KAT 1.22.0) took this block for the
button-press code: the slot and the random challenge in it rule that out.

### Button presses: the check on the XGATE

The check starts from a 10-byte message that the CPU stores at `F9:B719`: 8 bytes, then two more. The routine at
`F3:9FAB` copies the 8 bytes into a buffer, sets a try count to 0 and starts the XGATE's software trigger 3 with the
key; it repeats up to 60 times with the count raised by one, which looks like the window a fob's counter may run
ahead. The first byte of the result is kept as the press's command. With bit 1 of the flags at 0x252A set, the
first of the 8 bytes goes there directly, without the key. So the XGATE most likely turns the 8 encrypted bytes
back into a plain frame that starts with the button: a cipher with a decryption, run on the XGATE.

The XGATE's vector base is 0xA0DC (set at `F8:9514`); the image of its vector table sits at file offset 0x2C04 and
is copied to RAM at 0x0FA100. Its four software triggers (channels 0x36–0x39) all start at XGATE address
**0x0EE0**, which is flash at global address **0x780EE0**.

## What is missing

Two things, and where they are:

| | Where | Why the images lack it |
|-|-------|------------------------|
| the 16-byte key | EEE at 0x13F6DC–0x13F6EB | the EEE images start at 0x13F74C, the key table (the firmware reads the number of keys at 0x13F78C, the images' 0x40); the key is 0x70 bytes before their first byte |
| the button-press cipher | XGATE code at 0x780EE0 | the P-Flash image starts at 0x7A0000; the chip has flash below it (the firmware reads 0x788002 and compares it with 0xDEAD) |

These reads would settle it: the whole P-Flash from 0x780000, or the P-Flash of a DPLA module (an MC9S12XET256,
whose flash does not reach the XGATE's flash window, so its XGATE code should be copied to RAM from its P-Flash
image); and the EEE including the 16 bytes at 0x13F6DC. With the key and the code, the 92 recorded presses check
any answer within minutes.

## In KAT

- A frame whose CRC matches and whose 8 bytes do not unscramble the Ford way (Ford V1's strict check fails) is
  shown as **Ford V1 (JLR)**. Ford's own Ford V1 frames always pass the strict check, so they are not affected.
- **Serial:** the folded ID, bytes 5–6 (`8472`, `E5AE`), the same for a press and its repeat. Up to KAT 1.22.0 it
  was bytes 3–6 with the flags byte in it, so a repeat showed a second serial. A key file KAT writes still
  carries the reference C's `Serial`, bytes 3–6, as ProtoPirate's files do.
- **Button and counter:** 0; they are in the encrypted bytes.
- **Encode:** none. The reference C's round-trip check would accept these frames and send codes the car refuses;
  KAT does not offer Encode for them.
- **Replay** of a captured press: the module accepts a press only ahead of the counter it last saw, so a replay
  should work at most until the fob itself is next used.

## Sources

- The library recordings: `IMPORTS/JAGUAR/2017_jaguar_xj_key1_lock.sub`, `IMPORTS/JAGUAR/2017_jaguar_xj_key1_unlock.sub`,
  `IMPORTS/JAGUAR/2017_jaguar_xj_key1_trunk.sub`, and the same three of key 2: two fobs of a 2017 Jaguar XJ,
  433.92 MHz, sent in by a tester with the P-Flash, D-Flash and EEE images of the car's module.
- From a JLR key specialist: the D-Flash and EEE of two more modules, one before and after adding key 2 to it, and
  Lock, Unlock and Trunk recordings of key 2 after each was learned (not in the library). Their statements quoted
  above are theirs; the rest was checked against the images.
- Tools: Ghidra 12.2 with its HCS12X language; the scripts that diff, check and search the images are kept with
  KAT's local reference material, not in the repository.
