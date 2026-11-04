---
layout: default
---

# Kia V2 Protocol

**Rust module:** `src/protocols/kia_v2.rs`  
**Reference:** `REFERENCES/ProtoPirate/protocols/kia_v2.c` (Flipper-ARF `kia_v2.c` decodes and encodes identically)

## Overview

Kia Manchester keyfob at 500/1000 µs. 53-bit frame with a byte-swapped 12-bit counter and a CRC-4 (displayed
only; the C reports every frame).

## Timing

| Parameter | Value   | Notes                |
|-----------|---------|----------------------|
| Short     | 500 µs  | ±150 µs (strict `<`) |
| Long      | 1000 µs | ±150 µs              |
| Preamble  | ≥ 100 long pulses | then a short HIGH = sync |
| Min bits  | 53      |                      |

## Frame Layout (53 bits)

`1 << 52 | serial32 << 20 | button4 << 16 | raw_cnt12 << 4 | crc4`, `raw_cnt = cnt_lo8 << 4 | cnt_hi4`.
The start bit (52) is not sent; the sync stands for bit 51 (the serial's MSB). CRC-4 = XOR of every nibble
of bits 4–51, plus 1.

## RF

- **Encoding:** Manchester, common.rs port with `pp_manchester_event` (`level ? ShortLow : ShortHigh`)
- **RF modulation:** AM/FM. The references disagree: ProtoPirate's flag and catalog route say AM (as do
  Flipper-ARF and doctor-firmware), its help text ("V2 [FM476 315/433]") and README say FM476, and both Kia V2
  key files in `IMPORTS/HYUNDAI` were received with FM476. Since v1.5 KAT takes the union and transmits 2-FSK
  ±47.6 kHz (FM476) unless the capture carries its own preset; see [transmit.md](transmit.md).
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder Steps (`kia_protocol_decoder_v2_feed`)

1. **Reset** — a long HIGH starts the preamble; Manchester state Mid1.
2. **CheckPreamble** — count long pulses; a short HIGH after ≥ 100 is the sync (adds `1`); other pulse
   widths restart.
3. **CollectRawBits** — Manchester-decode; any out-of-tolerance pulse restarts; reported at bit 53.

KAT's earlier port used a hand-written Manchester table with the opposite event sense; it reproduced none of
the C's frames.

## Encoder

Supported (`kia_protocol_encoder_v2_deserialize` + `get_upload`): frame rebuilt from the fields, CRC
recomputed; 2 bursts, each 252 × (LOW 1000, HIGH 1000), LOW 500, bits 51..0 as Manchester (1 → HIGH, LOW;
0 → LOW, HIGH; 500 µs halves); adjacent equal levels merged. Counter + 1 (12 bits); KAT's generic button
maps to Lock 1, Unlock 2, Trunk 3, Panic 4 (ProtoPirate / ARF D-pad), anything else keeps the captured one.

## Validation

C oracle: 600 random frames × 2 bursts — uploads identical, the decoder reports the C's 743 frames exactly.
No IMPORTS capture contains Kia V2 (0 decodes, C and KAT, both polarities).
