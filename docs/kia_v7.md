---
layout: default
---

# Kia V7 Protocol

**Rust module:** `src/protocols/kia_v7.rs`
**Reference:** `REFERENCES/ProtoPirate/protocols/kia_v7.c` (Flipper-ARF `kia_v7.c`: same decoder, older encoder)

## Overview

Kia V7 uses Manchester encoding at 250/500 µs. 64 bits. The decoded 64-bit word is bit-inverted
(`~data`); a valid frame has a fixed header byte `0x4C` and a CRC8 (poly `0x7F`, init `0x4C`) over bytes
0–6. Emission is gated on header + CRC, so Kia V7 is strongly validated and will not false-match.

## Timing

| Parameter | Value  | Notes               |
|-----------|--------|---------------------|
| Short     | 250 µs | ±100 µs (te_delta)  |
| Long      | 500 µs | ±100 µs             |
| Min bits  | 64     |                     |
| Preamble  | ≥16 short pairs | before sync |

## Frame Layout (64 bits / 8 bytes, after inversion)

- **byte 0:** header `0x4C`
- **bytes 1–2:** counter (16-bit, big-endian)
- **bytes 3–6:** serial (28-bit) — `(b3<<20)|(b4<<12)|(b5<<4)|(b6>>4)`, masked to 0x0FFFFFFF
- **byte 6 low nibble:** button (4-bit) — Lock=1, Unlock=2, Trunk=3, Panic=8 (ProtoPirate emulate D-pad)
- **byte 7:** CRC8 over bytes 0–6 (poly `0x7F`, init `0x4C`)

## RF

- **Encoding:** Manchester, on-air 1 = HIGH,LOW. Decoded with the shared port of Flipper's
  `manchester_decoder.c` using `level ? ShortHigh : ShortLow` (kia_v7.c), so the decoded word is the
  bit-inverse of the key
- **RF modulation:** FM (per ProtoPirate `SubGhzProtocolFlag_FM`)
- **Encryption:** none — gated on fixed header `0x4C` + CRC8
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder Steps

1. **Reset** — a short HIGH pulse begins the preamble.
2. **Preamble** — count short pairs; once >15 pairs are seen, a long HIGH pulse transitions to SyncLow and preloads four seed bits (1,0,1,1 = the inverted header's top nibble 0xB).
3. **SyncLow** — a short LOW after the long sync enters Data. With the `0x4C` header the wait skips the long
   LOW of bits 62/61 and enters Data on the short LOW that starts bit 60; the Manchester state is still Mid1
   from the preamble reset.
4. **Data** — Manchester-decode the remaining 60 bits (out-of-range pulses are skipped). At 64 bits, invert the word, check the header equals `0x4C` and the CRC8 validates, then emit; the Manchester state is reset after every 64-bit candidate.

## Encoder

Supported (matches `kia_v7_encode_key` and `kia_v7_encoder_get_upload`). Rebuilds the 64-bit key from
serial/button/counter with the CRC8, then emits the frame twice, as the C upload does: behind a 319-pair
(0x13F) short preamble and behind a 15-pair (0x0F) one. Each frame is the preamble, a short HIGH, the 64
Manchester bits MSB first (1 = HIGH,LOW), a short HIGH and a 2000 µs LOW tail. Each press sends counter + 1
(16 bits, as the emulate scene); KAT's generic button maps to Lock 1, Unlock 2, Trunk 3, Panic 8, anything
else keeps the captured button. Flipper-ARF's encoder is ProtoPirate's pre-2026-09-14 one (a single
319-pair frame), which ProtoPirate replaced with this two-frame upload.

## Validation

C oracle (`kia_v7.c` on the host): 600 random frames — uploads identical pulse for pulse, the decoder
reports the C's 1200 frames (both copies); Flipper-ARF's decoder gives the same. No IMPORTS capture
contains Kia V7. Unit tests: key/field round trip; encode→decode through the full waveform
(both frames decode, for each button); the upload's two preamble lengths; a CRC-mismatched key and a
header-0x4D key are rejected.
