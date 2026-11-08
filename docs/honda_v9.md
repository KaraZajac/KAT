---
layout: default
---

# Honda V9 Protocol

**Rust module:** `src/protocols/honda_v9.rs`
**Reference:** `REFERENCES/FoxFW/lib/subghz/protocols/honda_v9.c` / `.h` (FoxFW only; itself a port of KeeLoq-RX
`arke_honda.py` `HondaV9`). **Decode only, as the reference** (`.encoder = NULL`).

## Overview

A Honda/Acura keyfob protocol (315 / 433.92 MHz, flagged AM and FM): Manchester, 112 bits, with an XOR checksum that
tolerates one wrong bit.

## Timing / modulation

| Parameter | Value |
|-----------|-------|
| te short / long / delta | 63 / 126 / 25 µs (inclusive) |
| Samples | a short pulse stores 1 half-bit sample of its level, a long one 2 (at most 512) |
| Burst end | a pulse of neither width; with at least 36 samples the burst is decoded |

The decoder needs a run of at least 20 alternating samples; the data starts at sample 110 when the run reaches 110 (55
preamble cycles), else at the end of the run. Sample pairs are Manchester bits ((0, 1) = 1 in normal polarity, the
reverse in inverted polarity) until two equal samples or 112 bits. Normal polarity is tried first.

## Frame (14 bytes, MSB first)

| Bytes | Field |
|-------|-------|
| 0 high nibble | header |
| 0 low nibble | button (at most 9) |
| 1..3 high nibble | serial (20 bits, not 0 or 0xFFFFF) |
| 3 low nibble | flags |
| 4..8 | fingerprint (32 bits, not 0 or 0xFFFFFFFF) |
| 8 | checksum = XOR of bytes 0..8, accepted within Hamming distance 1 |
| 10..12 | counter |
| 9, 12, 13 | carried, not read |

## Field mapping

`data` = bytes 0..8 big-endian, `data_count_bit` = 112, `serial`, `button`, `counter`, `crc_valid` = the checksum matches
exactly (a frame one bit off is emitted with `crc_valid` false), `extra` = bytes 8..14. Key file: `Protocol: Honda V9`,
`Bit: 112`, `Key`, `Serial`, `Btn`, `Cnt`, `Fingerprint`, `CRC`, `TailHi`, `TailLo`.

## Evidence

None: no capture of this protocol exists in the test corpus, so the port is validated on synthetic pulse sequences only:
the FoxFW oracle and KAT give the same Key, serial, button and counter on them in both polarities and with preambles of
110 and 60 samples, and the checksum tolerance (1 bit off accepted, 2 bits rejected) and the button, serial and
fingerprint gates agree; a short frame and a short preamble are rejected. The IMPORTS sweep shows no change.
