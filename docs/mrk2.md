---
layout: default
---

# NXP MRK II instruction set (PCF79xx key chips)

**Chips:** NXP PCF7941, PCF7961, PCF7952/7953, PCF7922 and their relatives: the key controllers of many car keys of 2005 to 2015
**Tools:** [`mrk2.py`](tools/mrk2/mrk2.py) (disassembler), [`mrk2emu.py`](tools/mrk2/mrk2emu.py) (emulator), [`hitag2cu.py`](tools/mrk2/hitag2cu.py) (HITAG2 unit), [`koleos_demo.py`](tools/mrk2/koleos_demo.py) (a complete run)
**Status:** every instruction the 4 KB images execute is decoded (a few only tentatively); 3.8% of the 8 KB keyless images' code is not, see [Not yet decoded](#not-yet-decoded)
**First key read with it:** [Renault Koleos](renault_koleos.md)

## Overview

NXP's PCF79xx chips sit in the keys of many cars: a small microcontroller, a radio transmitter, a 125 kHz
transponder and a hardware HITAG2 unit on one die. The car maker's own program for the key, the firmware, lives
in the chip's E-ROM; a mask ROM holds NXP's boot code and library. The CPU is NXP's **MRK II** ("Micro RISC
Kernel"), an 8-bit core with 16-bit instructions, and NXP never published its instruction set: the
"MRK II Family, Architecture and Instruction Set" manual went to customers under contract.

KAT decoded it from the 81 firmware images in its [Firmware](firmware.html) catalog and checked the result
against what NXP's public documents do give (listed under [Sources](#sources)). With the decoding a key's
firmware can be read like any program, and run: the emulator here runs the [Renault Koleos](renault_koleos.md)
key's firmware from its reset vector to the frame it transmits.

## Encoding

- **Byte order.** A dump as programmers read it (`*EROM.bin`) has every byte bit-reversed. Reverse the bits of
  each byte, then read 16-bit little-endian words. (Some tools save that form as `EROM FOR VVDI.bin`.) Only in
  this form do the fields below line up; NXP's own `TRAP` instruction, `dw 0F580H` in the assembler guide,
  falls into the group of `SYS` (`F500`), as it should.
- **Program addresses.** The program counter counts bytes, but `JMP` and `CALL` hold word addresses (12 bits,
  4,096 words). On the 4 KB parts the E-ROM is words `000`–`7FF` and the mask ROM `800`–`FFF` (not in any dump);
  a few images sit at `800` instead, with their vectors pointing there. The 8 KB parts fill `000`–`FFF` and reach
  the ROM through `SYS`.
- **Vectors.** Words 0 to 10: warm boot (battery), INT0 (LF field), INT1 (port), INT2 (timer 0), INT3 (timer 1
  compare), INT4 (timer 1 capture), INT5, INT6, warm boot (transponder), sub-command, unresolved `SYS`. Each
  holds a `JMP` or a `RETI`.
- **Conditions are skips.** There is no conditional branch: an instruction skips the next one, which is usually
  a `JMP`.

## Data memory

| Address | What |
|---------|------|
| `00`–`07` | registers R0–R7 |
| `08`–`11` | pointer "ports": reading or writing one reaches the memory a register points at |
| `12`–`3F` | special function registers (PSW `12`, SP `13`, interrupt `16`/`17`, timers, ports, modulator `1D`, HITAG2 unit `1E`/`1F`, power `26`, EEPROM `2C`–`2E`, battery comparator `2F`, transmitter `30`–`33`) |
| `40`–`7F` | RAM, direct |
| `80`–`FF` | RAM, through the pointers only |

Direct addresses are 7 bits, so RAM above `7F` is reached through the ports; that is why so much firmware loads
R0 with `8x` and then reads `@R0`.

| Port | Meaning |
|------|---------|
| `08`, `09`, `0A`, `0B` | `@R0`, `@R1`, `@R2`, `@R3` (`0B` not proved) |
| `0C`, `0D` | `@R0+`, `@R1+`: post-increment |
| `0E`, `0F` | `@-R0`, `@-R1`: pre-decrement (with `0D`, R1 serves as a software stack: interrupt routines push through `0F` and pop through `0D`) |
| `10` | `@dbp+`: one bit at the bit pointer (SPTR `15` = byte, SBIT `14` = bit, bit 0 the most significant), then the next bit |
| `11` | `@dbp`: the same bit without moving on (not proved) |

## Instructions

### Control

| Word | Instruction |
|------|-------------|
| `8aaa` | `JMP aaa` |
| `Aaaa` | `CALL aaa` (on 4 KB parts, `800`–`FFF` is the mask ROM) |
| `E500` | `RET` |
| `ED00` | `RETI` |
| `F500` | `SYS`: calls a ROM library function, its number in R4 |
| `F580` | `TRAP` (debug breakpoint) |
| `F7mm` | `INTA #mm`: acknowledges interrupts, clearing the flags whose bits are 0 in `mm` |

### Byte instructions

`[op:4][x:1][k:3][operand:8]`. With `x` = 1 the operand is an immediate: `op Rk,#imm`. With `x` = 0 its top
bit is a direction and the rest a 7-bit address: 0 gives `Rk = Rk op [addr]`, 1 gives `[addr] = [addr] op Rk`.

| op | Instruction | Notes |
|----|-------------|-------|
| 0 | `ADD` | sets C on carry out |
| 1 | `ADDC` | adds C in; `ADDC Rk,#0` carries into a high byte |
| 2 | `SUB` | sets C on borrow (not proved) |
| 3 | `SKEQ` | compares, sets C when Rk is the lower, skips the next instruction when equal |
| 4 | `AND` | |
| 5 | `OR` | |
| 6 | `XOR` | |
| 7 | `MOV` | `x` = 1: `MOV Rk,#imm`; `x` = 0: a load (direction 0) or a store (direction 1) |

### Bit instructions

`[op:4][x:1][s:1][bit:3][addr:7]`: the bit number sits in bits 9–7 and the byte address is 7 bits.

| Words | Instruction |
|-------|-------------|
| `C0`–`C3` | `CLR addr.bit` |
| `C4`–`C7` | `SETB addr.bit` |
| `C8`–`CB` | `CPL addr.bit` |
| `D0`–`D3` | `MOV C,addr.bit` |
| `D4`–`D7` | `MOV addr.bit,C` |
| `D8`–`DB` | skip the next instruction if the bit is 0 |
| `DC`–`DF` | skip the next instruction if the bit is 1 |

The carry is bit 7 of PSW, so `C392`, `C792`, `DB92` and `DF92` are `CLR C`, `SETB C` and the two skips on C,
the most common instructions in the corpus.

### Single-operand instructions

`[hi:8][0][addr:7]`.

| Word | Instruction | Word | Instruction |
|------|-------------|------|-------------|
| `E0` | `INC` | `E8` | `DEC` |
| `E2` | `INC`, skip if the result is 0 | `EA` | `DEC`, skip if the result is 0 |
| `E4` | `INC`, skip if the result is not 0 | `EC` | `DEC`, skip if the result is not 0 |
| `E6` | skip if the byte is 0 | `EE` | skip if the byte is not 0 |
| `F0` | `SHR` (bit 0 to C) | `F2` | `SHL` (bit 7 to C) |
| `F4` | `RRC` (through C) | `F6` | `RLC` (through C) |
| `F8` | `SWAP` (nibbles) | `FA` | probably set to `FF` (not proved) |
| `FC` | `CLR` | `FE` | `CPL` |
| `F1` | `MOVC`: the code byte at address R5:R4 | `E1`, `FFFF` | no effect seen (used as fillers) |

`C5A6` (`SETB RST`) fills the unused E-ROM of 17 of the 77 images: a program that runs away there resets the
chip.

### Not yet decoded

The `B` group (about 7,000 words, nearly all in the 8 KB keyless images; its operands look like special function
register addresses), the small `9` group, `E9`, `F3`, `F9`, `FB`, `FD` and `FFxx` other than `FFFF`. They make
up 3.8% of the code the 8 KB images execute and none of the 4 KB images'. Not proved yet: `SUB`'s carry, `FA`,
`E1`, port `0B` and `11`. The emulator stops on an undecoded instruction rather than guess.

## How it was worked out

Statistics over the corpus gave the structure; small idioms whose meaning cannot be anything else fixed each
instruction; NXP's public documents fixed names and addresses.

- **`CALL` and `RET`.** The word before a `CALL` target is `RET` in 67–75% of cases, 21 to 29 times the base
  rate.
- **Skips.** The skipping instructions are followed by a `JMP` in 51–98% of cases against 11% for others.
- **`SYS`.** All 1,230 `SYS` are preceded by `MOV R4,#n`, as in NXP's example (`MOV R4,#7AH`, `MOV R3,…`,
  `SYS`).
- **Registers at `00`–`07`.** The loop counter a `MOV` loads is the address a `DECSZ` counts down; the PCF7922
  data sheet's memory map agrees.
- **Loads and stores.** Of two `MOV`s in a row on the same register, a load followed by another load (which
  would waste the first) occurs 4 times in the corpus; a store followed by a store 388 times and a load followed
  by a store (a copy) 1,282 times. The stores land on the right registers: `MOV 96h,R2` restores IE (`16`) after
  `MOV R2,IE` saved it, `MOV 99h,R4` writes the timer reload (`19`).
- **Bit instructions.** NXP's application note ends a power-down with `SETB POFF` and a jump to itself; the
  firmware does exactly that with `C4A6`, and `C5A6` (`SETB RST`) the same way. Of the bits the skips test, 64%
  are bits the same image sets or clears (chance level under any other reading). All 63 waits on the EEPROM's
  BUSY flag use `D8`–`DB`, so those skip on 0.
- **ALU operations.** By their immediates: `AND` takes masks (`0F`, `07`, `F0`), `XOR` takes `FF` in 40% of
  cases, `OR` single bits, `ADDC` takes `#0` in 97%. `SKEQ` ends 34 counting loops (`INC R3`, `SKEQ R3,#07`,
  `JMP` back).
- **Increments and shifts.** A 16-bit increment (`INC lo`, skip if `lo` is not 0, `INC hi`), a saturating
  decrement (`SKIPZ x`, `DEC x`), the boot's RAM-clearing loop (`DECSNZ`), `RLC` after each `MOV C,EEIO`,
  `SWAP` before `AND #0F`, `CPL` then `INC` (a negation).
- **The whole.** The emulator runs the Koleos key's firmware through its boot, button handling, battery check,
  HITAG2 and transmission, and the keystream it sends matches HITAG2 computed independently.

## The HITAG2 unit

The chips compute HITAG2 in hardware, but bit by bit under program control: each access to `CRIO` (bit 7 of
`CRYP1`, `1E`) clocks the 48-bit register once, and `CRYP2` (`1F`) selects the function: loading bits,
the linear feedback (output) or the nonlinear filter (initialisation), in HITAG2 mode or in an "Enhanced" mode
with a 96-bit key and a 64-bit register. That is why no cipher tables appear in any firmware image. Most images
leave the unit to the ROM library; the Koleos key's firmware drives it itself, streaming the key straight from
EEPROM into the unit.

## Tools

The tools are Python 3 with no other dependencies. Download them together with an image from the
[Firmware](firmware.html) catalog.

```
python3 mrk2.py erom.bin             # listing of the code reached from the vectors
python3 mrk2.py erom.bin --all       # every word
python3 koleos_demo.py               # runs the Renault Koleos firmware, prints its frames and checks the HITAG2
```

The emulator models what a key's transmit path needs: registers, ports, the bit pointer, EEPROM, the HITAG2 unit
(HITAG2 mode), the battery comparator, timer 0 and timer 1 interrupts and the modulator output. It does not
model the mask ROM (a `SYS` is only logged), real time (timer 1 ticks are counted) or the radio. A busy-wait on
a hardware flag is let through after a few turns.

## Sources

- NXP, *Raisonance Tools for MRK II, Getting Started* (OM6710UG): the assembler's syntax, `SYS`, `INTA`, `TRAP`
  (`nxp.com/docs/en/user-guide/OM6710UG.pdf`).
- NXP, *PCF7922ATT product data sheet*: the MRK II data memory map, vectors, special function registers and their
  bits, the Calculation Unit's functions, application notes in assembly
  (`mm.digikey.com/Volume0/opasdata/d220001/medias/docus/6154/PCF7922ATT.pdf`).
- NXP, *PCF7x41ATJ product specification*: the PCF7941's memories and its Calculation Unit
  (`pdf.bomkeyelectronics.com/pdf/6/PCF7x41ATJ.pdf`).
- The firmware images: renewal files published by transpondery.com, mirrored in KAT's
  [Firmware](firmware.html) catalog.
