---
layout: default
---

# Star Line Protocol

**Rust module:** `src/protocols/star_line.rs`
**References:** `REFERENCES/ProtoPirate/protocols/star_line.c` (primary),
`REFERENCES/Flipper-ARF/lib/subghz/protocols/star_line.c` (adds 315 MHz and button remapping)

## Overview

StarLine car-alarm remotes (and compatible alarms: Cenmax, Tomahawk, …). KeeLoq rolling code in a
250/500 µs PWM frame. Like the C, the decoder walks the whole keystore looking for the manufacturer key
that validates the hop; the matching entry is shown as a suffix, e.g. `Star Line [SL_A2-A4]`
(ProtoPirate "MF:"); `ProtocolRegistry::get()` resolves it to `Star Line`. Frames no key validates are still
reported (plain `Star Line`, `crc_valid = false`, counter 0), as the C reports "MF:Unknown".

## Timing

| Parameter | Value   | Notes                                  |
|-----------|---------|----------------------------------------|
| Short (0) | 250 µs  | ±120 µs, HIGH + LOW                    |
| Long (1)  | 500 µs  | ±120 µs, HIGH + LOW                    |
| Header    | 1000 µs | > 4 HIGH/LOW pairs (±240 µs)           |
| End       | HIGH ≥ 620 µs | the next header pulse            |
| Bits      | 64..66  | first 64 kept                          |

## Frame Layout (64 bits, MSB-first, bit-reversed = fix | hop)

- **fix (32 bits):** button (8) << 24 | serial (24)
- **hop (32 bits):** KeeLoq(button << 24 | serial[7:0] << 16 | counter)

## Key search

Every keystore entry in order (`keeloq_common::keeloq_keystore`): SIMPLE entries decrypt with the key,
NORMAL entries with the normal-learning key for `fix`, UNKNOWN entries try simple / mirrored simple /
normal / mirrored normal. Accepted when the plaintext's top byte = button and byte 2 = serial[7:0].

## RF

- **Encoding:** PWM
- **RF modulation:** AM (OOK)
- **Encryption:** KeeLoq (keystore manufacturer keys)
- **Frequencies:** 315 MHz (ARF), 433.92 MHz

## Encoder

`gen_data` + `get_upload`: manufacturer re-derived by the same key search; counter + 1 (wraps to 0);
`fix = button << 24 | serial`; hop encrypted per the matched entry (SIMPLE / NORMAL; UNKNOWN per matched
mode, with the key as stored — the C ignores the mirrored copy here). Unknown manufacturer → the captured
hop is replayed under the new fix. Upload: 6 × (1000, 1000) + 64 bits, sent 40 times (C `encoder.repeat`).

Buttons (ARF): 0x01 Lock, 0x02 Unlock, 0x03 Trunk, 0x04 Panic; 2-way remotes 0x21 Lock, 0x22 Unlock,
0x23 Trunk, 0x24 Start, 0x25 Stop, 0x26 Extra. KAT Lock/Unlock/Trunk/Panic → 0x01/0x02/0x03/0x04, plus
0x20 when the captured button is a 2-way one (ARF d-pad mapping); 0x03 and 0x21..0x26 pass through.

## Validation

C oracle (unmodified ARF `star_line.c`, KAT's keystore): decoder field-for-field and encoder
pulse-for-pulse on randomized frames built with every keystore entry/mode (plus jitter and 63..67-bit
variants) — no mismatch. A real Cenmax remote in ARF's `unit_tests/subghz/test_random_raw.sub` now
decodes as `Star Line [Cenmax_St-5]` with consecutive counters (previously counter 0: KAT only tried one
key). The IMPORTS sweep has no Star Line capture.
