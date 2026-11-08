---
layout: default
---

# Kia V1 Protocol

**Rust module:** `src/protocols/kia_v1.rs`  
**Reference:** `REFERENCES/ProtoPirate/protocols/kia_v1.c` (Flipper-ARF `kia_v1.c` builds the same upload); timing
window and CRC check from doctor-firmware's `kia_v1.c`

## Overview

Kia/Hyundai Manchester keyfob at 800/1600 µs. 57-bit frame: 32-bit serial, 8-bit button, 12-bit counter,
4-bit CRC. ProtoPirate reports every frame and only displays the CRC check; doctor-firmware and KAT report only
frames whose CRC holds.

## Timing

| Parameter | Value   | Notes                    |
|-----------|---------|--------------------------|
| Short     | 800 µs  | ±300 µs (strict `<`)     |
| Long      | 1600 µs | ±300 µs                  |
| Preamble  | > 70 LOW/HIGH long pairs | then a short LOW = sync |
| Min bits  | 57      |                          |

## Frame Layout (57 bits)

`serial32 << 24 | button8 << 16 | cnt_lo8 << 8 | cnt_hi4 << 4 | crc4`

- The sync stands for the first bit (`1`, the serial's MSB): ProtoPirate's own decoder mis-aligns frames
  whose serial MSB is 0 (C and KAT agree on that too).
- CRC-4: XOR of both nibbles of serial bytes, button, cnt_lo and cnt_hi, plus 1.
- Buttons (C names): 1 Close (Lock), 2 Open (Unlock), 3 Boot (Trunk); 4 = Panic on the emulate D-pad.

## RF

- **Encoding:** Manchester, Flipper's `manchester_decoder.c` (common.rs) with ProtoPirate's
  `pp_manchester_event` (`level ? ShortLow : ShortHigh`)
- **RF modulation:** AM
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder Steps (`kia_protocol_decoder_v1_feed`)

1. **Reset** — a long HIGH starts the preamble; Manchester state Mid1.
2. **CheckPreamble** — count LOW long pulses after long ones; after more than 70, a short LOW is the sync
   (adds `1`).
3. **DecodeData** — Manchester-decode; out-of-tolerance pulses are ignored (the state machine keeps going);
   the frame is reported on the pulse completing bit 57.

KAT's earlier port used a hand-written Manchester table: on the real captures it reported wrong serials
with failing CRCs (it matched none of the C's frames). This port and the C agree on every capture.

### Timing tolerance (doctor-firmware's)

ProtoPirate accepts a pulse within ±200 µs; doctor-firmware's copy of the same decoder accepts ±300 µs and emits a
frame only when its CRC holds. KAT does both. Measured on KAT's simulated radio (synthetic IQ through the receive
pipeline, the IMPORTS captures' 35 real frames and the encoder's frame, 240 trials per setting):

| Impairment | ±200 (ProtoPirate) | ±300 (doctor) |
|---|---|---|
| SNR 4 dB + 60 µs jitter + fob clock ±10 % | 29 % | 87 % |
| 75 µs pulse jitter | 25 % | 97 % |
| fob clock 10 % slow | 28 % | 100 % |
| fob clock 15 % fast or slow | 0 % | 100 % |
| pulse asymmetry ±200 µs | 0–8 % | 100 % |

Nothing ±200 reads is lost, and every IMPORTS capture decodes as before. Under heavy jitter both windows also
read corrupted frames; ±300 reads fewer (67 against 72 per run) and fewer of them pass the 4-bit CRC by chance (1
against 8): the CRC check drops the rest, as doctor-firmware does. ProtoPirate's and doctor-firmware's C agree with
the two settings on 240 edge-case recordings each.

## Encoder

Supported (`kia_protocol_encoder_v1_get_upload`): the frame is rebuilt from the fields (CRC recomputed);
3 bursts separated by LOW 25 000 µs, each 90 × (LOW 1600, HIGH 1600), LOW 800, then bits 55..0 as
Manchester (1 → HIGH 800, LOW 800; 0 → LOW 800, HIGH 800); adjacent equal levels merged. Counter + 1
(12 bits) per press; KAT's generic button maps to Lock 1, Unlock 2, Trunk 3, Panic 4, anything else keeps
the captured button.

## Validation

C oracle (`kia_v1.c` on the host): 600 random C-encoded frames × 3 bursts — uploads identical pulse for
pulse, decoder output identical (1800 frames); all IMPORTS, both polarities, identical (35 frames from
`KIA V1 N1 RAW`, `KiaV1_5cl_5op` and `HYU V1 N1 RAW`, serial B444706E, CRC OK). ProtoPirate changed only the
displayed CRC value since KAT's v1.3.0 port; Flipper-ARF's decoder resets on bad pulses and reports a
subset of ProtoPirate's frames.
