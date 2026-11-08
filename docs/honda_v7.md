---
layout: default
---

# Honda V7 Protocol

**Rust module:** `src/protocols/honda_v7.rs`
**Reference:** `REFERENCES/FoxFW/lib/subghz/protocols/honda_v7.c` / `.h` (FoxFW only; itself a port of KeeLoq-RX
`arke_honda.py` `HondaV7`). **Decode only, as the reference** (`.encoder = NULL`).

## Overview

A Honda/Acura AM keyfob protocol (315 / 433.92 MHz): OOK PWM, 88 bits, with an 8-bit XOR checksum.

## Timing / modulation

| Parameter | Value |
|-----------|-------|
| te short / long / delta | 250 / 480 / 100 µs (inclusive) |
| Bit | a HIGH + LOW pair, both short = 0, both long = 1; a mixed pair ends the frame |
| Preamble | at least 20 short HIGH + short LOW pairs |
| Bits | 88 (a broken pair with at least 64 bits tries the frame, which still needs all 88) |

Bits are stored LSB first in each buffer byte and every byte is bit-reversed to its natural form.

## Frame (natural bytes)

| Bytes | Field |
|-------|-------|
| 0 high nibble | button (1..9) |
| 0 low nibble, 1..4 | serial (28 bits, not 0 or 0x0FFFFFFF) |
| 4..7 | counter (24 bits) |
| 7 high nibble | mode |
| 8 | checksum = XOR of bytes 0..8 |
| 9..11 | carried, not read |

## Field mapping

`data` = bytes 0..8 big-endian, `data_count_bit` = 88, `serial`, `button`, `counter` = the low 16 bits of the 24-bit
counter (the full counter is `data[39:16]`), `extra` = bytes 8..11 (checksum and the two unread bytes), `crc_valid`
true. Key file: `Protocol: Honda V7`, `Bit: 88`, `Key`, `Serial`, `Btn`, `Cnt` (24 bits), `Mode`, `CRC`, `Tail`.

## Evidence

None: no capture of this protocol exists in the test corpus, so the port is validated on synthetic pulse sequences only:
the FoxFW oracle and KAT give the same Key, serial, button, counter, mode, CRC and tail on them, and a bad checksum, a
short preamble, a short frame, buttons 0 and 10 and a zero serial are rejected. The IMPORTS sweep shows no change.
