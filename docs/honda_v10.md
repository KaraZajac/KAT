---
layout: default
---

# Honda V10 Protocol

**Rust module:** `src/protocols/honda_v10.rs`
**Reference:** `REFERENCES/FoxFW/lib/subghz/protocols/honda_v10.c` / `.h` (FoxFW only; itself a port of KeeLoq-RX
`arke_honda.py` `HondaV10`, "V2H4"). **Decode only, as the reference** (`.encoder = NULL`). **No data check.**

## Overview

A Honda/Acura 2-FSK keyfob protocol (315 / 433.92 MHz, flagged AM and FM): Manchester, 96 bits. Emission is gated on
structure only, so `crc_valid` is always false.

## Timing / modulation

| Parameter | Value |
|-----------|-------|
| te short / long / delta | 104 / 208 / 35 µs (inclusive) |
| Samples | short = 1 half-bit sample of its level, long = 2 (at most 2048) |
| Burst end | a pulse of neither width; with at least 36 samples the burst is scanned |

The scan runs over start offsets, both polarities (normal first): a run of at least 20 alternating samples followed by
something, then Manchester pairs until two equal samples or 96 bits; at least 88 bits are needed and a shorter frame is
zero-padded. Every frame fires the receiver's callback and a receiver resets its decoders on it, so only the first frame
of a burst is reported (the FoxFW oracle shows the same).

## Frame (12 bytes, MSB first)

| Bytes | Field |
|-------|-------|
| 0..3 | serial (24 bits, not 0 or 0xFFFFFF) |
| 3 high nibble | button (0xE or 0xF) |
| 3 low nibble | flags (must be 7) |
| 4..7 | counter, stored inverted: `0xFFFFFF ^ bytes` |
| 7 | status |
| 8..12 | tail (a per-counter encrypted MAC) |

## Field mapping

`data` = bytes 0..8 big-endian, `data_count_bit` = 96, `serial`, `button`, `counter` = the low 16 bits of the 24-bit
counter, `extra` = the tail, `crc_valid` false. Key file: `Protocol: Honda V10`, `Bit: 96`, `Key`, `Serial`, `Btn`, `Cnt`
(24 bits), `Flags`, `Status`, `Tail`.

## Evidence

None: no capture of this protocol exists in the test corpus, so the port is validated on synthetic pulse sequences only:
the FoxFW oracle and KAT give the same Key, serial, button and counter in both polarities, for a 90-bit zero-padded frame
and for two frames in one burst (one reported); flags other than 7, buttons other than 0xE / 0xF, serials 0 and 0xFFFFFF,
a short frame and a short preamble are rejected. The IMPORTS sweep shows no change.
