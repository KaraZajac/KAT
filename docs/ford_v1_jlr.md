---
layout: default
---

# Ford V1 (JLR) Protocol

> **Research report:** [Ford V1 (JLR): a 32‑bit key](reports/jlr.pdf) — this finding, written up as a PDF, with the rest of KAT's [research reports](reports.md).

**Rust module:** `src/protocols/ford_v1.rs` (the [Ford V1](ford_v1.md) decoder reads it)
**Reference:** none. This page is KAT's own work: recordings of the fobs, and flash and EEPROM images of several cars'
keyless modules with the firmware of two of them (Sources, below).

## Overview

Jaguar Land Rover's keyless fobs of about 2015–2018 (the Keyless Entry Transmitter, part **BJ32-15K601**, an NXP
**PCF7953P** Hitag-Pro inside) send **Ford V1's 136-bit frame**: the same header, Manchester coding, 2-FSK and CRC.
Where Ford keeps its scrambled serial, button and counter, they send **8 bytes encrypted under the car's 16-byte
key**. KAT reads the frame, names it **Ford V1 (JLR)** and shows what travels in the clear: the fob's ID folded to
16 bits, and whether a frame is the first send of a press or its repeat.

The cipher is now worked out (from the module firmware, confirmed against every recorded press). Those 8 bytes
decrypt to the **button, a 24-bit counter and the fob's full ID**. The key is the one in the car's keyless module,
in a part of its data flash a normal read exports. **Given that key** — the car's own, read from the owner's module
— KAT shows the button, counter and ID of each press and can make the next code. Without it, KAT stays decode only
and shows the folded ID, the flags and a replayable capture.

What the recordings and the module images show, each in its section below:

- The header carries the fob's **transponder ID folded in half**: the high 16 bits XOR the low 16 bits.
- The 8 encrypted bytes are a **three-round [Feistel](https://en.wikipedia.org/wiki/Feistel_cipher) network whose
  round function is AES-128** under the car's key. They decrypt to `button | 24-bit counter | fob ID`.
- The key is **16 bytes in the module's data flash**, in the configuration record a 4 KB D-Flash read covers. It is
  the same key the module's keyless-entry (passive) check uses.
- The car's keyless module learns a fob by its **ID alone**; learning writes the ID, a count, a state byte and two
  checksums, nothing secret, and **gives the fob the car's key** — the fob's counter restarts from then.

## Frame

The frame is Ford V1's ([Ford V1](ford_v1.md) has the decoder's steps): 17 bytes, Manchester at 65/130 µs, 2-FSK at
433.92 MHz. The recordings were made with a Flipper's custom CC1101 preset at ±14.3 kHz deviation.

| Bytes | Field | |
|-------|-------|---|
| 0–3 | `D3 95 0E 26` | constant, the same as in Ford's own Ford V1 frames |
| 4 | flags | `00` the first send of a press, `08` the same press sent again; `09` was seen once |
| 5–6 | folded ID | the fob's 32-bit transponder ID, high half XOR low half |
| 7–14 | 8 encrypted bytes | the button, counter and full ID, enciphered (below) |
| 15–16 | CRC | CRC-16/CCITT (polynomial 0x1021, start 0) over bytes 3–14 |

A press goes out at least twice, flags `00` then `08`, with the same 8 bytes. A repeat is therefore not a new code.
The 8 bytes never repeat between presses and show no pattern in the clear — the counter only becomes visible once
the bytes are deciphered.

## The fob's ID in the header

Bytes 5–6 are the fob's transponder ID folded to 16 bits: `2F47 XOR AB35 = 8472` for key 1 (ID `2F47AB35`), `A696
XOR 4338 = E5AE` for key 2 (ID `A6964338`); the chance of both matching by accident is about 1 in 4 billion. The
module finds which learned fob sent a frame by folding each ID it has learned the same way. The other 16 bits of the
ID travel enciphered in the payload, and come back when the payload is deciphered. The fold is not secret.

## The cipher

The 8 payload bytes are enciphered with a **three-round Feistel network**. Split the 8 bytes into a left and a right
half, `L` and `R`, of 4 bytes. The round function is

```
F_i(x) = AES-128( K , 0x00 ×12 ‖ x ) [ 4·i .. 4·i+3 ]        for i = 1, 2, 3
```

— AES-128 of the 4-byte half padded to a block with twelve zero bytes, keeping bytes 4–7 of the result in round 1,
8–11 in round 2, 12–15 in round 3. The module (the receiver) deciphers a press like this:

```
M1 = L  XOR F1(R)
M2 = R  XOR F2(M1)
M3 = M1 XOR F3(M2)
plaintext  =  M3 ‖ M2
```

The fob enciphers with the same network run the other way (the module only ever runs AES forward, which is why its
firmware has an AES S-box but no inverse one). The 8-byte plaintext is:

| Byte | Field |
|------|-------|
| 0 | button: `01` Lock, `02` Unlock, `04` Trunk |
| 1–3 | a 24-bit counter, big-endian; it goes up by 2 each press |
| 4–7 | the fob's full 32-bit transponder ID |

The button matches the recording, the low 4 bytes are the fob's ID (so its header fold checks out), and the counter
only rises. Re-enciphering the plaintext reproduces the captured bytes exactly. This was checked on every recorded
press of every fob and module (Sources): 135 frames, all consistent.

`K` is the car's 16-byte key (next section). The counter the fob sends runs ahead of the module's stored one; the
module accepts a press whose counter is within a window above what it last saw, then stores the new value.

## The car's keyless module

The receiver is the car's keyless module (JLR calls it the Keyless Vehicle Module, KVM, or Remote Function
Actuator, RFA): an NXP **MC9S12XE**-family microcontroller with its **XGATE** co-processor. The modules looked at:
part **FK72-19H440-BF** from the 2017 XJ the library recordings come from, and **DPLA-19H440-AF**, **-BB** and two
DPLA modules read as full P-Flash by the specialist. Key programmers read three images from such a module: the
program flash (P-Flash), 4 KB of data flash (D-Flash) and 2 KB of emulated EEPROM (EEE). The images hold the car's
VIN and are not published.

### The key

Both the button cipher and the keyless-entry check use **one 16-byte key**, held in the module's **D-Flash
configuration record at offset 0x0E14** (and its copy at 0x0F14), right after the VIN. A normal 4 KB D-Flash read
covers it. At start-up the module copies the whole record into RAM, where the CPU and the XGATE read the key.

The key has the form **X ‖ Y ‖ Y ‖ X** — two 4-byte words, the second pair mirroring the first — in every module
seen, so it carries **64 bits**, not 128. It is not derived from the VIN by any CRC that was tried. One module read
was of an unconfigured unit: its record is all zero there, and that module's fob frames decipher under the all-zero
key, which is a clean confirmation of where the key sits. The key bytes themselves are not given here.

### The key table

All modules keep their learned fobs the same way. The EEE holds the key record twice, at offsets 0x000 and 0x080,
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

The D-Flash keeps an older copy inside the same 0x0E00 configuration record (repeated at 0x0F00): the IDs at 0x0E84,
the counters at 0x0EA4, the number of keys at 0x0EC4 and a byte per slot from 0x0EC5. Its counters can lag the EEE's
by thousands.

The checksums, found from the records of all the modules:

- **EEE record** (0x00–0x7D): Fletcher-16 modulo 256 with both sums starting at 0xFF; the second sum is stored at
  0x7E, the first at 0x7F. The EEE records at 0x500 and 0x580 use the same checksum.
- **D-Flash record** (0x0E00–0x0EFD): CRC-16/ARC (polynomial 0x8005 reflected, start 0), stored low byte first at
  0x0EFE.

### Learning a fob

One module was read before and after a key specialist added a fob to it. Exactly 16 bytes changed in the key table,
the same in each copy of the record: the fob's ID in the next free slot, the number of keys, that slot's state byte
(EEE `04` → `08`, D-Flash `00` → `10`), and the two checksums — **a module learns a fob from its ID alone.** The key
in the configuration record does not change. Learning also hands the fob the car's key (the fob's counter restarts
from then), so a fob learned to a car sends under that car's key; the specialist's report that one fob opened more
than one car means those modules were given it in turn.

## The firmware

The P-Flash of two modules (the 2017 XJ's, 384 KB; a DPLA module's, 256 KB) was disassembled with Ghidra's HCS12X
language, and the XGATE co-processor's code with a small XGATE disassembler. Routines are named by page:address.

### Keyless entry: the challenge check

Apart from the button frames, the module runs a separate **passive-entry** exchange: when the car looks for a fob it
encrypts a 16-byte block with AES-128 (the key above) and checks the fob's answer. The block is a constant
`A6 C3 2B 91 50 FE 47 DD`, then the fob's slot and a 29-bit random challenge, then the slot's transponder ID. The
challenge comes from a 29-bit Galois LFSR seeded from the key table. This is the car-asks/fob-answers path, not the
button presses; earlier versions of this site (to KAT 1.22.x) mistook this block for the button code.

### Button presses: the cipher on the XGATE

The button check runs on the **XGATE** co-processor. The CPU hands it the 8 received bytes and the key (software
trigger 3); the XGATE routine at XGATE address 0x1058 runs the three-round Feistel above, calling an AES-128 at
XGATE address 0x123C for each round function, and returns the deciphered `button ‖ counter ‖ ID`. The CPU then
checks the ID against the sending slot and the counter against the stored one, within a look-ahead window. (The
2017 XJ's 384 KB image holds this XGATE code directly; a DPLA module's 256 KB image holds it in the half a 384 KB
read would place at the lower addresses — the reason earlier reads seemed to stop short of it.)

## In KAT

- A frame whose CRC matches and whose 8 bytes do not unscramble the Ford way (Ford V1's strict check fails) is shown
  as **Ford V1 (JLR)**. Ford's own Ford V1 frames always pass the strict check, so they are not affected.
- **Serial:** the folded ID, bytes 5–6 (`8472`, `E5AE`), the same for a press and its repeat.
- **With the car's key** (supplied by the owner — see *Supplying the key*): KAT deciphers the payload and shows the
  **button** (Lock, Unlock, Trunk), the **24-bit counter** and the **full transponder ID**, and can make the fob's
  **next code** (counter + 2, enciphered under the key).
- **Without a key:** decode only — the folded ID, the flags, button and counter shown as 0, and **replay** of a
  captured press (the module takes a press only ahead of the counter it last saw, so a replay works at most until
  the fob is next used).

### Supplying the key

KAT ships no JLR keys — each is a car's own, and only its owner has it. Put the keys you hold in a file named
`jlr_keys` in KAT's config folder (beside `keeloq_mfcodes_user`), one car per line: **32 hex digits** (the 16-byte
key, read from your module's D-Flash at offset 0x0E14; spaces and dashes are ignored), optionally `:` and a name,
`#` for a comment. For example:

```
# my cars
00112233445566778899AABBCCDDEEFF : XF 2017
```

KAT loads the file at start-up and says how many keys it read. A key is tried against every JLR frame; it is used
only when the frame deciphers to a button and an ID whose fold matches the header, so a wrong key is simply ignored.

## Sources

- The library recordings: `IMPORTS/JAGUAR/2017_jaguar_xj_key1_lock.sub`, `…_key1_unlock.sub`, `…_key1_trunk.sub`,
  and the same three of key 2: two fobs of a 2017 Jaguar XJ, 433.92 MHz, sent in by a tester with the P-Flash,
  D-Flash and EEE images of the car's module.
- From a JLR key specialist: more modules' D-Flash, EEE and (later) full P-Flash reads, one module before and after
  adding key 2 to it, and Lock, Unlock and Trunk recordings of key 2 after each learning (not in the library). Their
  statements quoted above are theirs; the rest was checked against the images.
- Tools: Ghidra 12.2 with its HCS12X language, a small XGATE disassembler, and the scripts that diff, check, search
  and decipher — kept with KAT's local reference material, not in the repository.
