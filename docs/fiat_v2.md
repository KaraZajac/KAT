---
layout: default
---

# Fiat V2 Protocol

**Rust module:** `src/protocols/fiat_v2.rs`
**Reference:** `REFERENCES/ProtoPirate/protocols/fiat_v2.c` (decoder; decode-only there). The encoder
combines Flipper-ARF's `lib/subghz/protocols/fiat_v2.c` known-key hop recompute with doctor-firmware's
`lib/subghz/protocols/fiat_v2.c` replay waveform.

## Overview

Rolling-code keyfobs from FCA (Fiat / Chrysler / Jeep). Each burst carries one 112-bit Manchester frame at
210/420 µs between long LOW gaps. The frame holds a 32-bit UID, a type nibble, a 2-bit button selector, an
11- or 14-bit counter and a 32-bit hop code. Real captures in IMPORTS: `chrysler/19_chrysler_pacifica*` and
`jeep/22_jeep_renegade*`.

## Timing

| Parameter | Value    | Notes                                        |
|-----------|----------|----------------------------------------------|
| Short     | 210 µs   | ±100 µs (111..309 µs) = 1 half-bit cell      |
| Long      | 420 µs   | ±100 µs (321..519 µs) = 2 cells              |
| Boundary  | ≥900 µs  | LOW gap ending a frame                        |
| Frame     | 112 bits | 224 cells                                     |

## Frame Layout (112 bits = 14 bytes, MSB first)

| Bytes        | Field                                                                    |
|--------------|--------------------------------------------------------------------------|
| raw[0..1]    | markers `0x00 0x01` (15 zero bits + a one)                               |
| raw[2..5]    | UID (serial)                                                             |
| raw[6]       | high nibble: type (`0xD` = FCA layout); low nibble varies with the button |
| raw[7]       | bits 7:6 selector: 1 Trunk, 2 Lock, 3 Unlock; `button` = raw[7] as in the C |
| raw[8..13]   | counter + hop (layout depends on the type, below)                        |

- **non-FCA:** counter = NOT(`(raw[7] & 0x3F) << 5` OR `raw[8] >> 3`) & 0x7FF (11-bit), hop = raw[9..12]
- **FCA (type `0xD`):** counter = NOT(`raw[8] << 6` OR `raw[9] >> 2`) & 0x3FFF (14-bit), hop = raw[10..13]

**KAT packing:** `data` = raw[0..8], `data_count_bit` = 112, `extra` = raw[8..14] (48 bits, right-aligned).

On air, a bit b is sent as half-bits (!b, b) (LOW/HIGH = 1). Every genuine frame in IMPORTS uses this polarity,
which the C reaches through its inverted try.

## RF

- **Encoding:** Manchester
- **RF modulation:** AM (OOK)
- **Encryption:** rolling code (hop not recomputed — no key)
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder Steps

1. **Reset**: the first short or long pulse clears the cell buffer and starts Data.
2. **Data**: a short pulse pushes one cell of its level, a long pulse pushes two. After each push the last
   224 cells are tried as 112 Manchester pairs, first with bit = first cell, then inverted. A window is valid if
   every pair differs, both markers match, the selector is 1/2/3 and the UID is not 0 / 0xFFFFFFFF.
3. **End of burst**: any other pulse ends the run. A LOW ≥900 µs first pushes one LOW cell (the frame's last
   half-bit merged into the gap).
4. **KAT gate**: the frame is reported only if the valid window ends the burst, with at most one trailing cell
   (doctor-firmware's stop pulse). Identical consecutive frames are suppressed until `reset()` (the C's dedup).

The end-of-burst gate is the one deviation from the C. The C's sliding window also matches 15 bits of any
Manchester preamble plus the first 97 bits of a longer frame. On `LDV/LDV-T80_unlock` it would complete before
Ford V3 does and steal that frame in KAT's first-match registry. Genuine FCA bursts always end with the frame,
so all 47 Pacifica and 29 Renegade frames still decode. doctor-firmware's extra type-nibble exclusions
(0x1/0x7/0xA) were not adopted, because the genuine Renegade frames have type 7.

## Encoder

Frame content from Flipper-ARF, waveform from doctor-firmware:

- **Hop recompute (Flipper-ARF):** if one of ARF's 8 Fiat V1 known HITAG2 keys (`fiat_v1_known_keys`) with
  one of its 4 IV normalisations (raw selector or one-hot remap × counter or complemented counter, epoch 0)
  reproduces the captured hop, the frame is forward-encoded: the requested selector (KAT Lock → 2,
  Unlock → 3, Trunk → 1; other commands keep the captured one), the counter + 1 (ARF's car-emulate scene
  writes Cnt + 1 per press), and the hop recomputed (`fiat_v1_compute_auth`) and patched in; markers, UID
  and type nibble are kept.
- **Replay:** otherwise (every capture in IMPORTS — no known key matches them in ARF either) the 14
  captured bytes are replayed unchanged, as ARF and doctor-firmware do.
- **Waveform (doctor-firmware):** each of 6 copies is LOW 2000 µs, then the 112 bits as (!b, b) half-bits of
  210 µs, then a HIGH 210 µs stop pulse if the last half-bit was LOW, then LOW 2000 µs, with same-level runs
  merged. This polarity matches the real captures; Flipper-ARF's upload is inverted behind a 2033 µs
  carrier lead copied from its Fiat V1 encoder, so KAT keeps doctor-firmware's (capture evidence).

## Validation

Decodes REAL IMPORTS captures: the Chrysler Pacifica (UID `0xD737A2C3`, 47 frames, 434.075 MHz) and the Jeep
Renegade (UID `0xD08BE613`, 29 frames, 433.889 MHz). The UID is stable, the selector follows the recorded
button order (Unlock → Lock → Trunk …), and `raw[8..9]` counts down from press to press. The LDV / Ford V3
captures do not match. The Fiat Bravo capture (`FIAT/Fiat Bravo 2010 …`) is not Fiat V2: its bursts are 70-bit
frames after a ~756 µs sync. Unit tests cover synthetic frames in both polarities, a stretched first pulse,
the merged-LOW boundary, FCA field parsing, the 6-copy encode→decode round trip (frames ending HIGH and LOW),
and rejection of bad markers, UID, selector and Manchester, plus a window inside a longer burst, and the
known-key forward encoding (both layouts, all 4 IV combos). C oracle: the real captures decode to the same
frames as ProtoPirate's and Flipper-ARF's decoders (KAT's end-of-burst gate only drops their LDV false
positive); on 1200 encodes of randomized frames (840 with a known-key hop) the transmitted frame equals the
`Raw` Flipper-ARF's encoder writes back, byte for byte.
