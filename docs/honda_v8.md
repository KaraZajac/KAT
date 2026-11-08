---
layout: default
---

# Honda V8 Protocol (ported, not registered)

**Rust module:** `src/protocols/honda_v8.rs`
**Reference:** `REFERENCES/FoxFW/lib/subghz/protocols/honda_v8.c` / `.h` (FoxFW only; itself a port of KeeLoq-RX
`arke_honda.py` `HondaV8`). **Decode only, as the reference** (`.encoder = NULL`).

## Status

The decoder is ported and its tests run, but it is **not in the registry**. Its only integrity check, the decryption
match, passes about 0.5% of random frames (21 keys x 2^-12), and it completes on the 66th LOW pulse, before a longer
frame's own decoder does: one of the sixteen Nice One uploads in KAT's G6 registry test was reported as Honda V8,
whatever the decoder's place in the registry. The corpus sweep (IMPORTS and the Flipper-ARF unit-test captures) shows no
change. Enabling it is one line in `src/protocols/mod.rs`.

## Overview

A Honda/Acura AM keyfob protocol (315 / 433.92 MHz): KeeLoq-style inverted PWM carrying a 32-bit KeeLoq ciphertext.
The integrity check is a decryption match against 21 Honda manufacturer keys.

## Timing / modulation

| Parameter | Value |
|-----------|-------|
| Base | 400 µs, delta 150 µs; a LOW within 400 +- 150 µs is a 0, any other LOW a 1; HIGH pulses are ignored |
| Sync | a LOW of at least 2500 µs |
| Bits | 66 LOW pulses (the first 64 are kept; a second sync with more than 60 also processes) |

## Frame

The 64 kept bits, bit-reversed to `raw` (first bit received = MSB): `raw[63:60]` button (0 Panic, 1 Lock, 2 Trunk,
4 Remote start, 8 and 9 Unlock), `raw[59:32]` serial (28 bits, not 0 or 0x0FFFFFFF), `raw[31:0]` ciphertext.

For each of the 21 keys (probe order 14, 1, 0, 2..13, 15..20) the ciphertext is decrypted with 528 right-shifting
rounds (NLF `0x3A5C742E`, key bit `(15 - i) & 63`) and must give `dec & 0xF == button` and `(dec >> 4) & 0xFF ==
(serial >> 20) & 0xFF`; the case `dec >> 16 == 0` with discriminant 0 is rejected. The counter is `dec >> 16`.

A rolling window then gates the frame: it is accepted only when `1 <= counter - last <= 128` (mod 2^16). The last
counter survives `reset()`, as in the C, so a press repeated with the same counter is not reported twice and the first
frame after the decoder was created always is.

## Field mapping

`data` = `raw`, `data_count_bit` = 66 (as the C), `serial`, `button`, `counter` = `dec >> 16`, `extra` = the matching key
index, `crc_valid` true. Key file: `Protocol: Honda V8`, `Bit: 66`, `Key`, `Serial`, `Btn`, `Cnt`, `KeyIdx`, `Enc`;
loading one re-runs the key match.

## Evidence

None: no capture of this protocol exists in the test corpus, so the port is validated on synthetic pulse sequences only.
The frames are built by running the C's decryption backwards under a Honda key with made-up fields; the FoxFW oracle and
KAT give the same Key, serial, button, counter and key index on them (including the rolling-window rejections), and a
flipped ciphertext bit, a button outside the set, a frame of 55 bits and a missing sync are rejected. The IMPORTS sweep
shows no change.
