---
layout: default
---

# Honda V5 Protocol

**Rust module:** `src/protocols/honda_v5.rs`
**Reference:** `REFERENCES/FoxFW/lib/subghz/protocols/honda_v5.c` / `.h` (FoxFW only; itself a port of KeeLoq-RX
`arke_honda.py` `HondaV5`). **Decode only, as the reference** (`.encoder = NULL`).

## Overview

A Honda/Acura keyfob protocol: Manchester at te 63 / 126 µs, a header `0x6C0F62`, a 32-bit serial, a button byte, a
16-bit counter, a 32-bit rolling word and a CRC-8. Two frames are recognised from the collected bit stream: the
standard 135-bit frame and an alternate 168-bit one (header `0x6428`).

## Timing / modulation

| Parameter | Value |
|-----------|-------|
| te short / long / delta | 63 / 126 / 20 µs |
| Burst end | pulse > 186 µs (`te_long + 3 * te_delta`) or an invalid pulse |
| Bits | 130 minimum to parse, 400 maximum |

Pulses are typed by length and level (high short 0, low short 2, high long 4, low long 6, other 8) and fed to a
four-state machine whose table is `{0x01, 0x91, 0x9B, 0xFB}`. The C flags the protocol AM; the real captures are
FSK (Honda1 preset), so KAT declares AM/FM.

## Standard frame (135 bits, laid out from bit position 1)

| Bytes | Field |
|-------|-------|
| 0..3 | header `0x6C0F62` |
| 3 | retransmit marker (0x08 / 0x09 / 0x0A) |
| 4..8 | serial (32 bits) |
| 8 | button (0x21 Lock, 0x22 Unlock in the captures) |
| 10..12 | counter (16 bits) |
| 12..16 | rolling word (32 bits) |
| 16 | CRC-8, MSB-first, poly 0x2F, init 0x03, over bytes 0..16 |

Leading preamble 1-bits are skipped and up to two data 1-bits they swallowed are restored.

## Alternate frame (168 bits)

Header `0x6428`, serial = bytes 13..17, button = byte 17, "counter" = byte 14 (as the C), CRC-8 reflected (poly 0x4D,
init 0) over bytes 0..20 against byte 20. Tried first when at least 168 bits follow the preamble.

## Field mapping

`data` = frame bytes 0..8, `data_count_bit` = 135 or 168, `extra` = bytes 8..16, `crc_valid` true (emission is
CRC-gated), `encoder_capable` false. Key file: `Protocol: Honda V5`, `Bit`, `Key`, `Serial`, `Btn`, `Cnt`, `CRC`,
`Rolling`, `Variant`, `MidHi`, `MidLo`, `Ext` (135-bit frames only).

## Evidence and deviation

`IMPORTS/honda/Lock_honda.sub` and `Unlock_honda.sub` (Honda1 custom preset, 433.657 MHz) decoded as Honda Static in
ProtoPirate's and Flipper-ARF's C and in KAT up to 1.8.7 (see [honda_static.md](honda_static.md)). Compared on the same pulses:

| | Honda Static (ProtoPirate C) | Honda V5 (FoxFW C and KAT) |
|---|---|---|
| Frames | 41 | 40 (Lock 23, Unlock 17) |
| Check | none (reverse path, `crc_valid` false) | CRC-8 valid on all 40 |
| Serial | 3 values within a file (0xB782388 / 0xB7823A8 / 0xB7823C8) | 0xB53EFF72 in both files |
| Button | 1 in both files | 0x21 in Lock, 0x22 in Unlock |
| Counter | 0x56BE7F in 40 of 41 | Unlock 7154..7162, Lock 7175..7191, rising by 1 per press |

Honda Static's three "serials" differ exactly as V5's byte-3 retransmit marker (0x08 / 0x0A / 0x09) does: header bits
misread. The registry therefore places Honda V5 before Honda Static. KAT's Honda Static has since also stopped reporting
frames whose checksum fails, which these are (see [honda_static.md](honda_static.md)). KAT also differs from the C in the
RF flag (AM/FM instead of AM).

One frame of the Lock capture (counter 0x56FE21 as ProtoPirate's Honda Static reads it) fails Honda V5's CRC. KAT listed
it as a lone Honda Static decode with `crc` false until Honda Static's bit-reversed pass checked the checksum; now nothing
reads it.

rtl_433's `Honda-KR5V2X1X` port reads the same frames (event 0x21 Lock, 0x22 Unlock, same CRC-8); since the registry
reports an rtl_433 port only when no Flipper decoder reads the signal, they are reported as Honda V5.

## Validation

FoxFW's compiled C and the Rust port give the same 40 frames on the two captures. Unit tests cover the field values and
counter sequence on both files, the header / CRC gating, the restored leading bits and the key-file round trip.
