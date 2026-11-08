---
layout: default
---

# Honda V4 Protocol

**Rust module:** `src/protocols/honda_v4.rs`
**Reference:** `REFERENCES/FoxFW/lib/subghz/protocols/honda_v4.c` / `.h` (FoxFW only; itself a port of KeeLoq-RX
`arke_honda.py` `HondaV4`). **Decode only, as the reference** (`.encoder = NULL`).

## Overview

A Honda/Acura AM keyfob protocol (315 / 433.92 MHz): PWM where the bit is carried by the width of the LOW pulse, each
bit followed by one clocking HIGH pulse. A burst opens with at least three 1500 µs LOW + 1500 µs HIGH cycles.

## Timing / modulation

| Parameter | Value |
|-----------|-------|
| Bit 0 / bit 1 (LOW pulse) | 750 / 1100 µs, tolerance below 200 µs |
| Preamble | 1500 µs LOW + HIGH, tolerance below 300 µs, at least 3 cycles |
| Bits | 51 (a frame ended early by a gap or the next preamble needs at least 48) |

## Frame (51 bits)

`serial (19) | button (8) | inv (8) | counter (16)`. The check is the inverse-byte invariant `inv == 0xFF - button`
(bits 23..16 against bits 31..24); emission is gated on it, so `crc_valid` is true on every frame. A frame of 48 to 50
bits ended by a gap is also emitted, and still reported as 51 bits (as the C).

## Field mapping

`data` = the 51-bit word, `data_count_bit` = 51, `serial`, `button`, `counter` as above, `encoder_capable` false. Key
file: `Protocol: Honda V4`, `Bit: 51`, `Key`, `Serial`, `Btn`, `Cnt`.

## Evidence

None: no capture of this protocol exists in the test corpus (the FoxFW triage found no file it reads), so the port is
validated on synthetic pulse sequences only. Unit tests build bursts from made-up values, compare them with the FoxFW
oracle's decode of the same pulses (Key, serial, button and counter equal), and check that a broken inverse byte, a
short preamble, a wrong preamble pulse and a frame cut at 44 bits are rejected. The IMPORTS sweep shows no change.
