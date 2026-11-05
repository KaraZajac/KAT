---
layout: default
---

# Cardin S508 Protocol

**Rust module:** `src/protocols/cardin_s508.rs`
**Reference:** `REFERENCES/RogueMaster/lib/subghz/protocols/cardin_s508.c` (half2me, Unleashed PR #1004; only
RogueMaster ships it, registered last). **Receive only, as the reference.**

## Overview

Cardin's S500-series 868 MHz rolling-code gate remotes (S508, FCC ID TXQ508; the 2-channel S504 uses the same
protocol) — the 868 MHz sibling of the 433.92 MHz Cardin S449 (FSK KeeLoq, decoded by KeeLoq), not the fixed-code
S435/S437 nor Telcoma/Cardin EDGE. Per press: a ~25 ms wake-up burst of long unequal pulses, a run of constant
Manchester bits, then the codeword, repeated while the button is held: a 12-bit fixed sync `110011110100` and a
128-bit AES-128 ciphertext in which every bit changes between presses — no clear serial, button or counter. The
reference has no encoder (NULL vtable, flags Decodable | Save, no Add Manually): new codes cannot be made without
the receiver's key.

## Timing / modulation

FM (2-FSK, ~12 kHz deviation; the reference reads it with FM12K), 868.35 MHz (`SubGhzProtocolFlag_868 | FM`).
Manchester, half-bit ~100 µs. The C classifies each pulse with per-level bands measured from a real capture (the
2-FSK slicer is duty-cycle biased: HIGH half-bits ~110–150 µs, LOW ~75–120 µs; full cells ~225 / ~175 µs):

| Pulse | Short | Long | Invalid |
|-------|-------|------|---------|
| HIGH | 40–164 µs | 165–339 µs | < 40 or ≥ 340 µs |
| LOW | 40–134 µs | 135–299 µs | < 40 or ≥ 300 µs |

## Frame Layout

| Bits | Field |
|------|-------|
| 12 | sync `0xCF4` (or its complement `0x30B`, see below) |
| 128 | payload (opaque AES-128 block) |

## Decoder Steps

`subghz_protocol_decoder_cardin_s508_feed`:

1. Classify the pulse; an invalid one (noise, gap, the wake-up burst) resets the sync search, the payload and the
   Manchester state.
2. Feed the event (`level ? ShortHigh/LongHigh : ShortLow/LongLow`) to Flipper's toolbox Manchester decoder
   (`manchester_advance`, KAT's `common::manchester_advance`).
3. Each decoded bit slides through a 12-bit window until it reads `0xCF4` or `0x30B`. The toolbox decoder reads
   Cardin's "1 = high then low" as 0, so real captures lock on the complement, and the payload is then
   complemented back: the reported codeword is always canonical.
4. The next 128 bits are the payload; the frame is reported and the search restarts (the Manchester state carries
   on) for the next repeat. No checksum exists.

## Field mapping

`data` = payload bits 127..64 (the first 64 wire bits; the C's `data_2`), `extra` = bits 63..0 (the C's `data`),
`data_count_bit` = 128; `serial`, `button`, `counter` None; `crc_valid` false (nothing to check);
`encoder_capable` false.

## Encoder

None (receive only, as RogueMaster): the signal menu offers Replay of the recorded pulses and export. A
rolling-code receiver rejects a code it has already seen.

## Key file

`Protocol: Cardin S508`, `Bit: 128`, `Key` = the low 64 bits, `Data` = the high 64 bits (8 hex bytes),
`subghz_protocol_decoder_cardin_s508_serialize` / `_deserialize` (loading needs `Bit: 128` and `Data`).

## Validation

- No real Cardin S508 capture exists in the references, IMPORTS or the rtl_433 tests.
- Reference oracle `subghz_oracle_roguemaster` (host build of the RogueMaster C, added this round): decode parity on
  1338 distinct RAW files (no decode in either) and on 1000 fuzz streams (250 synthetic presses — both bit
  conventions, symmetric 100/200 µs and the header's duty-cycle-biased timing, 1–3 repeats, with jitter / dropped /
  split / stretched pulses — and the other F protocols' 750): identical, 311 frames.
- KAT's registry adds its FSK inverted-polarity pass (a stream where the normal pass decodes nothing is tried
  inverted): 2 of 500 perturbed streams decode there and not in RogueMaster's receiver; the reference decoder
  reports the same frame on the inverted file.
- Unit tests: both conventions and timings, the classification bands, an invalid pulse or a missing sync drops the
  frame, key-file round trip, decode-only.
