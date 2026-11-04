---
layout: default
---

# Renault V0 Protocol

**Rust module:** `src/protocols/renault_v0.rs`
**Reference:** `REFERENCES/ProtoPirate/protocols/renault_v0.c` (doctor-firmware's copy is identical apart from
UI glue and was used to confirm the TX counter policy). Flipper-ARF's `renault_v0.c` is ProtoPirate's
pre-1287d10 decoder ("move newer Renault variants into Renault V1"); KAT adds its tail-anchored frame reading
as a fallback (see Frame Layout).

## Overview

Renault rolling-code keyfobs. An 82-bit frame (64-bit key + 18-bit `Key2`) is Manchester-coded at 125/250 µs
after a ≥800 µs carrier sync and sent in 3 bursts. The key carries a 24-bit serial, a button code, an 8-bit
counter and 42 parity bits (24 in the key, 18 in `Key2`).

ProtoPirate checks three things: **C1** and **C2** form a "type 13" XOR checksum over the frame, and **IC**
compares the frame with an algebraic key model (`renault_v0_build_key`). Only C1/C2 plus a button whitelist
gate emission; IC is shown but never enforced, and the encoder replays IC-failing frames verbatim. KAT does
the same.

## Timing

| Parameter  | Value   | Notes                                                        |
|------------|---------|--------------------------------------------------------------|
| Short      | 125 µs  | ±69 µs — with the C's unsigned compares: 56..180 µs          |
| Long       | 250 µs  | 181..319 µs                                                  |
| Sync       | ≥800 µs | HIGH; starts (or restarts) the data step                     |
| Frame      | 82 bits | after ≥10 preamble 1-bits                                    |
| TX         | 3 bursts | sync 1000 µs, 16 preamble bits, 82 bits, 25 ms gaps         |

## Frame Layout (82 bits)

| Bits            | Field                                                     |
|-----------------|-----------------------------------------------------------|
| key[63:40]      | serial (24-bit)                                           |
| key[39:32]      | button: 0x05 Trunk, 0x06 Lock, 0x0A Unlock                |
| key[31:24]      | counter (8-bit)                                           |
| key[23:0]       | parity bits 0..23                                         |
| Key2[17:0]      | parity bits 24..41                                        |

- **Checksum:** XOR of the 8 key bytes, `Key2[17:10]` and `Key2[9:2]`. C1 requires `checksum & 0x3F == 0x13`;
  C2 requires `Key2 & 3 == checksum >> 6`.
- **Key model (IC):** 7 variables (button == Unlock, counter bits 0..5) give 64 monomials (constant, 7 linear,
  21 quadratic, 35 cubic). Each of 42 matrix rows yields one parity bit, and counter bits 6/7 flip parity bits
  41/40. For a given fob the model closes the checksum at every counter, but only for Lock/Unlock or only for
  Trunk, never both.
- **Frame extraction:** ProtoPirate counts the run's leading 1s as the preamble (≥ 10) and reads the 82 bits
  after them, so a key whose serial bit 23 is 1 is swallowed by the preamble. Flipper-ARF reads the run
  tail-anchored instead: its last 82 bits, after exactly 16 preamble bits, with buttons 0x06/0x0A only (ARF's
  type-13 button set). KAT tries ProtoPirate's reading first and ARF's second, so it decodes exactly the union
  of both references' frames. ARF's extra checksum types (04/0C/1A/3B/3F) moved to Renault V1 in ProtoPirate
  1287d10 and are not part of V0 in either reference any more; ARF's extra timing profiles only add 53–55 µs
  short pulses and are not adopted.
- **KAT packing:** `data` = 64-bit key, `data_count_bit` = 82, `extra` = `Key2` (right-aligned). `button`
  keeps the Renault code.

## RF

- **Encoding:** Manchester
- **RF modulation:** AM (OOK)
- **Encryption:** rolling code (algebraic key model, no cipher key)
- **Frequencies:** 315 MHz, 433.92 MHz, 868.35 MHz

## Decoder Steps

1. **Reset**: wait for a HIGH ≥800 µs, then enter Data with the Manchester state at Mid1 and the bit buffer
   empty.
2. **Data**: classify each pulse as short/long (`level ? Low : High` event). Feed it to the C's inline state
   table `{0x01, 0x91, 0x9B, 0xFB}`, which uses Flipper's transition bytes and numbering. An event that maps to
   the current state is ignored; there is no reset to Mid1 as in `manchester_advance`. Entering Mid1 or Mid0
   buffers a 1 or 0.
3. **Candidate**: on an unclassifiable pulse (or once >109 bits are buffered), strip the leading 1s as
   preamble (≥10 required, ≥82 bits must follow), read 64 key bits and 18 `Key2` bits, and emit only if the
   type-13 checksum and button whitelist pass. A HIGH ≥800 µs restarts Data immediately.

## Encoder

Supported (ProtoPirate encoder + emulate scene):

- **Model ("rolling") frames:** the key is rebuilt from the serial, the requested button (Lock → 0x06,
  Unlock → 0x0A, Trunk → 0x05, anything else keeps the captured code) and **counter + 1 (mod 256)**.
  ProtoPirate's emulate scene bumps `Cnt` this way before every TX, and doctor-firmware's encoder bumps it
  itself. If the rebuilt frame fails the checksum (e.g. Trunk for a Lock/Unlock fob), the C refuses to send
  and KAT returns no signal.
- **Other frames:** key + `Key2` are replayed unchanged.
- **Waveform:** 3 × [HIGH 1000 µs, 16 preamble 1-bits, 82 data bits, HIGH 125 µs if the line is low, LOW
  25 000 µs (last burst 250 µs)], using the C's two-state Manchester emitter with same-level merging.

## Validation

No Renault capture is available in IMPORTS. Unit tests cover the C's pulse-classification ranges, the key model
closing the checksum for all 256 counters, the encode→decode round trip (counter bump, button re-stamp, unknown
command keeps the button, counter wrap), refusal of a checksum-failing re-mint, verbatim replay of a non-model
frame, rejection of flipped key bits, a non-whitelisted button and a short sync, and a serial-bit-23 frame
decoding through the tail-anchored reading. The IMPORTS sweep shows no Renault V0 matches and no regressions
(no reference decodes anything there either). C oracle: on ~1400 randomized bursts (ProtoPirate's encoder,
jitter up to ±72 µs, doubled bursts) KAT's frames equal ProtoPirate's ∪ Flipper-ARF's in every burst
(doctor-firmware = ProtoPirate); encoder uploads equal ProtoPirate's (encoder + emulate scene) pulse for pulse.
