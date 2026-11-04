---
layout: default
---

# Kia V5 Protocol

**Rust module:** `src/protocols/kia_v5.rs`  
**Reference:** `REFERENCES/ProtoPirate/protocols/kia_v5.c` (encoder since 2026-05-19); Flipper-ARF
`kia_v5.c` (same upload, button names)

## Overview

Kia Manchester keyfob at 400/800 µs: 64 data bits + a 3-bit CRC field. The key is sent bit-reversed; its
reverse ("yek") is `button4 << 60 | serial28 << 32 | hop32`, and the 16-bit counter is the hop decrypted by
the "mixer" cipher keyed with the KIA V5 key (keystore type 13).

## Timing

| Parameter | Value  | Notes   |
|-----------|--------|---------|
| Short     | 400 µs | ±150 µs |
| Long      | 800 µs | ±150 µs |
| Preamble  | > 40 counted pairs | then a long HIGH sync |
| Bits      | 64–67  | 64 key + up to 3 CRC-field bits |

## Frame Layout

- `data` = the 64 key bits as received; serial/button/counter from `yek = reverse(data)`.
- CRC field = `0`, then the 2-bit CRC of the key (`kia_v5_calculate_crc`); kept in `extra`.
- Buttons (ARF names): 0x1 Unlock, 0x2 Lock, 0x4 Trunk, 0x8 Horn.

## RF

- **Encoding:** Manchester, common.rs port with the C's sense `level ? ShortHigh : ShortLow`
- **RF modulation:** FM
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder Steps (`kia_protocol_decoder_v5_feed`)

1. **Reset** — a short HIGH starts the preamble; Manchester state Mid1.
2. **CheckPreamble** — count pairs; a long HIGH after more than 40 starts the data.
3. **Data** — Manchester-decode (bits after the 64th go to the CRC field, at most 67); the first
   out-of-tolerance pulse ends the frame, reported if ≥ 64 bits arrived (`data_count_bit` = min(bits, 67)).

The C only displays the CRC; KAT sets `crc_valid` when the received CRC bits match the key's CRC (true for
all 117 complete frames in IMPORTS). KAT's earlier port used a hand-written Manchester table: its serials
changed between presses of the same fob, and it matched none of the C's frames.

## Encoder

Supported (new; `kia_protocol_encoder_v5_deserialize` + `get_upload`): hop = `mixer_encode(serial,
counter + 1, button)`, key = reverse(yek), CRC recomputed; 200 × (HIGH 400, LOW 400), LOW 400, HIGH 800,
LOW 400, HIGH 400, then the 64 key bits, `0` and the 2 CRC bits as Manchester (1 → LOW, HIGH; 0 → HIGH,
LOW), closing LOW 400 + HIGH 400; adjacent equal levels merged. KAT's generic button maps to ARF's codes:
Lock 0x2, Unlock 0x1, Trunk 0x4, Panic 0x8 (Horn); anything else keeps the captured button.

## Validation

C oracle: on every IMPORTS capture (both polarities) KAT reports the C's 119 frames (serial, button,
counter, CRC field); 600 random C-encoded frames — uploads identical, all decode identically. The encoder's
`mixer_encode` inverts the decoder's `mixer_decode` on the real frames (re-encoding a captured frame's fields
gives its exact key). Flipper-ARF only reports 67-bit frames (117 of the 119).

## Keystore

KIA V5 key (keystore type 13, `kia_v5_key`), for both mixer directions.
