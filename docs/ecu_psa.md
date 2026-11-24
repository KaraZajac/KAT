---
layout: default
---

# PSA engine ECUs and BSI (Peugeot / Citroën)

[ECU & immobiliser reference](ecu.md) › PSA

Peugeot and Citroën engine ECUs of the late 1990s and the 2000s keep the immobiliser **PIN** in the
small serial EEPROM beside the ECU's microcontroller. The PIN is the four‑character code (characters
`0`–`9` and `A`–`Z`) that pairs the engine ECU with the BSI, the body computer, and that the dealer
tool asks for when one of them is replaced. This page records where each ECU family keeps the PIN,
how it is encoded, and what the "kill immo" and "repair" operations of the trade tools rewrite.

It documents structure, not a procedure: the encoding constants, the rewrite patterns and every real
code are kept in the local reference set, the same way KAT keeps real key IDs and manufacturer keys
out of its RF pages.

## Where the information comes from

The layouts below were recovered by static analysis of **TMW electronic's *PSA Calculator*
(version 16.04.07, 2008)**, the PSA companion of the *Immo Tool* described on the
[ECU overview](ecu.md). The tool was never run. Its program was taken apart in Ghidra, every
operation was traced from the button that starts it to the bytes it reads or writes, and the result
was checked against the dumps in the reference set wherever one exists. Its nineteen operations,
grouped by the ECU they touch:

| ECU | Memory | Image the tool expects | Operations |
|---|---|---|---|
| Bosch EDC15 (diesel) | 95P08, marked `5P08C3` | 1 KB | Calculate PIN, program PIN, kill immo |
| Bosch EDC16 (diesel) | 95160 | 2 KB | Calculate PIN |
| Siemens SID803 (diesel) | 95320 | 4 KB | Calculate PIN, program PIN |
| Bosch ME7.4.4 (petrol) | 95160 | 2 KB | Calculate PIN, program PIN |
| Bosch Motronic MA3.0, MA3.1, MP3.2, MP5.1.1, MP5.2, MP7.2, MP7.3 (petrol) | 93C46 / 93CS46 / 24C02 | 128 B | Kill immo (one routine for all seven) |
| Magneti Marelli IAW 1AP 4X, 20/80/83/90, 81 (petrol) | TMS374 internal EEPROM | 256 B | Repair |
| BSI (body computer) | — | any size | Decode / encode the dump |

## How the PIN is stored

### Bosch EDC15 (95P08, 1 KB)

- The PIN is kept in **three copies**, four bytes each, at `0x200`, `0x216` and `0x22A`.
- Each byte is the character's ASCII code **XOR a one‑byte constant**. The constant depends on the
  character's position *and* on the copy, so the three copies look unrelated although they hold the
  same PIN.
- Each copy is stored **last character first**: the fourth character sits at the lowest address.
- A zero byte reads as "no PIN", the state an immobiliser removal leaves behind.
- A four‑byte **immobiliser status field at `0x14B`** belongs with the copies. The tool's kill‑immo
  operation writes a fixed pattern there and clears all three copies.

The calculator reads only the first copy; the programmer rewrites all three.

### Bosch EDC16 (95160, 2 KB)

One copy at `0x60`–`0x63`, with the same per‑position XOR over ASCII and four constants of its own.
Unlike EDC15, the copy is stored **first character first**.

### Siemens SID803 (95320, 4 KB) and Bosch ME7.4.4 (95160, 2 KB)

The PIN is stored in **plain ASCII**, first character first: on SID803 at `0x22`–`0x25`, on
ME7.4.4 at `0x592`–`0x595`, with a mirror copy at `0x5A2`–`0x5A5`. Reading the EEPROM is reading
the PIN.

### Bosch Motronic MA3.0 to MP7.3 (93C46 / 93CS46 / 24C02, 128 B)

These ECUs keep their parameters as 16‑bit words. In the MP5.1.1 dump of the reference set **every
value is stored three times in a row**, the same word at three consecutive addresses, from `0x00`
to `0x4D`, with the rest erased (`0xFF`). The usual reason for such a triple is two‑out‑of‑three
voting against a failing cell. The first triple, bytes `0x00`–`0x05`, is the immobiliser word. The
tool treats all seven ECUs alike: its kill‑immo operation is a single routine that clears those
first six bytes and fixes no checksum.

### Magneti Marelli IAW 1AP (TMS374, 256 B)

The memory layout is on the [IAW on TMS374](ecu_iaw_tms374.md) page. The PSA Calculator's repair is
the bluntest operation in the tool: it **replaces the whole 256‑byte EEPROM** with one of two fixed
images, one for the 1AP 4X (which the tool also writes onto a 1AP 81) and one for the 1AP
20/80/83/90. Each image is a complete dump from a donor ECU, the donor's immobiliser code included.

Two dumps in the reference set, `IAW_1AP.40` and `IAW_1AP.80`, are byte for byte those two images:
TMW's sample dumps and the tool's repair images are the same files. Comparing the two images shows
which parts of a 1AP image depend on the variant or the car: 34 bytes, at `0x03`–`0x0F` (status
bytes and the 8‑character code at `0x07`–`0x0E`), `0x43`, `0x4F`, and scattered bytes in
`0xA0`–`0xB4` and `0xC1`–`0xC6`. Everything else is common to both variants.

### BSI (body computer)

The tool's BSI decoder **complements every byte** (each byte XOR `0xFF`). A complement is its own
inverse, which is why one button both decodes and encodes: the BSI dumps it is meant for hold their
EEPROM content bit‑inverted. No key or table is involved. The tool does not say which BSI types
store inverted data, and no BSI dump is in the reference set yet.

## What this says about the protection

- **No per‑unit secret protects the PIN at rest.** SID803 and ME7.4.4 store it in the clear; EDC15
  and EDC16 apply a fixed XOR that is the same in every unit of the type. Anyone who can read the
  EEPROM, an eight‑pin chip on the board, has the PIN.
- **The XOR is obfuscation, not encryption.** One dump with a known PIN gives away all four
  constants. Even without one, the alphabet gives them away: a stored byte XOR the right constant
  must land in `0`–`9` or `A`–`Z`, and every further dump from another car rules out most of the
  remaining candidates.
- **Redundancy is not integrity.** The three EDC15 copies, the ME7.4.4 mirror and the triplicated
  Motronic words guard against a failing cell, not against an edit. The tool rewrites these fields
  without touching any checksum, so the ECUs evidently check none over them.
- **The IAW 1AP image is not bound to its unit.** The repair works only because a donor's complete
  image, code included, is accepted by another ECU of the same variant: the pairing lives in
  replaceable data, not in the microcontroller.

## How it was recovered

For anyone repeating this on a similar tool:

1. **Identify the build.** The program is Delphi 7 and unpacked. Its forms are stored as `TPF0`
   resources, which list every control, caption and event‑handler name.
2. **Map the interface to the code.** The main form's action list gives the nineteen operations in
   order, and the form class's published‑method table turns the handler names (`Button1Click`, …)
   into addresses.
3. **Seed the disassembler.** Ghidra does not find handlers that are reached only through Delphi's
   run‑time type information. Creating functions at those addresses and at every
   `push ebp; mov ebp, esp` prologue, then analysing again, recovers them.
4. **Follow the start button.** Its handler is a switch on the selected action that calls one routine
   per operation, each after a file‑size check that doubles as the chip size the tool expects.
5. **Test the decode tables instead of reading them.** The PIN tables are long if‑chains (position ×
   byte → character). Parsing them mechanically showed that each is exactly one XOR constant per
   position. The programmer's write path (its character order taken from the disassembly of the
   string concatenation) uses the same constants and the opposite byte order, so encoding and
   decoding agree.
6. **Watch for a protection bias.** Every offset, character position and BSI byte in this tool goes
   through two values that are zero only when TMW's interface answers a start‑up challenge. Without
   the interface the tool still runs and quietly produces wrong results. A static reading has to take
   the bias as zero, the only value for which calculator and programmer agree.
7. **Check against dumps** wherever the reference set has one (below).

## Corpus (local reference)

| Dump | Family | What it showed |
|---|---|---|
| `IAW_1AP.40` | IAW 1AP 4X | identical to the tool's 4X / 81 repair image |
| `IAW_1AP.80` | IAW 1AP 20/80/83/90 | identical to the tool's 20/80/83/90 repair image |
| `IAW_1AP.81` | IAW 1AP 81 | 17 bytes from the 20/80/83/90 image (`0x05`–`0x0E`, `0x70`–`0x75`, `0xB2`–`0xB4`), yet the tool repairs an 81 with the 4X image |
| `MP5.1.1` | Bosch MP5.1.1 | every value a triple of identical words; the immobiliser word in bytes `0x00`–`0x05` |
| `Saxo 1.0` (24C02) | — | fully erased (`0xFF`), nothing to read |
| `5P08C3_Sprinter` | Mercedes EDC15 | does not decode with the PSA layout: the 95P08 chip is shared, the record layout is not |
| EDC15C2 `0281011081` (307), two images | Bosch EDC15 | the first is immobiliser‑removed: all three PIN copies zero and the status field at `0x14B` holding the tool's kill pattern. The second is the same image after its PIN was programmed: its CRC32, stored in the archive it was shared in, equals the CRC32 of the first image with that PIN written by the tool's encoding into all three copies, and nothing else changed |

The EDC15 layout is therefore confirmed on a real ECU, byte for byte: all three copies, their
constants and their byte order. No EDC16, SID803 or ME7.4.4 dump from a PSA car is in the reference
set yet. Those three layouts are as the tool implements them and consistent between its calculator
and its programmer, but not yet confirmed on a real dump; one dump with a known PIN would confirm
them.

The real codes and PINs in these dumps are not reproduced here.
