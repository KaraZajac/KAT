---
layout: default
---

# Fiat V0 Protocol

**Rust module:** `src/protocols/fiat_v0.rs`  
**Reference:** `REFERENCES/ProtoPirate/protocols/fiat_v0.c` (Flipper-ARF and doctor-firmware carry the same
decoder and upload)

## Overview

Fiat V0 uses Manchester encoding at 200/400 µs: a long run of short pulses, an 800 µs LOW gap, then 64 data
bits (32-bit fix = serial, 32-bit hop = rolling/encrypted word) and 6 tail bits. There is no checksum.

Manchester is decoded with the shared port of Flipper's `manchester_decoder.c` (`common.rs`), mapping
level true → ShortLow/LongLow and level false → ShortHigh/LongHigh (fiat_v0.c), state reset to Mid1 when
the preamble starts and at the gap.

## Timing

| Parameter | Value  | Notes                                   |
|-----------|--------|-----------------------------------------|
| Short     | 200 µs | ±100 µs                                 |
| Long      | 400 µs | ±100 µs                                 |
| Preamble  | ≥150 short pulses (HIGH and LOW counted) |  |
| Gap       | 800 µs | ±100 µs, LOW                            |
| Frame     | 71 bits | 64 data + 6 tail on air + 1 (see below) |

## Frame Layout

On air after the gap: a short HIGH (first half of an implied 1 bit, dropped by the decoder), then Manchester
with 1 = HIGH,LOW: hop (32 bits, MSB first), fix (32 bits), 6 tail bits.

- `data` = hop << 32 | fix, `data_count_bit` = 71
- `serial` = fix, `counter` = hop bits 15..0
- `button` = the 7 decoded tail bits. The decoder's 71st bit is not part of the burst: after the 70th bit
  the inter-burst gap is a Manchester reset event that the C ignores once 64 bits are in, so the 71st bit
  comes from the next burst's preamble (always 1). The last burst of a press therefore never completes (as
  in the C). The C keeps `tail & 0x3F`; KAT keeps all 7 bits (identical on every real capture, where the
  6 on-air tail bits are `0xxxx1`) so the encoder can resend them losslessly.

## RF

- **Encoding:** Manchester 200/400 µs (on-air 1 = HIGH,LOW)
- **RF modulation:** AM/OOK
- **Encryption:** rolling/encrypted hop (no key); no checksum
- **Frequencies:** 433.92 MHz (433.88 MHz captures fall inside the ±2% window)

## Decoder Steps

1. **Reset** — a short HIGH starts the preamble (Manchester reset).
2. **Preamble** — count short pulses of either level; once ≥150, an 800 µs LOW enters Data (Manchester reset).
3. **Data** — Manchester-decode; at 64 bits latch fix/hop; at 71 bits emit. An out-of-range pulse resets
   below 64 bits and is ignored from 64 to 70 bits.

Gating is structural only (preamble length, 800 µs gap, 71 Manchester bits), as in the C.

## Encoder

Supported as a byte-exact replay of the captured frame: the hop is a rolling code and the C replays the
captured EndByte regardless of the selected button. Three bursts, 25 ms apart: 150 short pairs whose last
LOW is the 800 µs gap, the 64 data bits and the 6 tail bits (`tail >> 1`) transition-coded as in the C, a
LOW short after a final 1, and a 1600 µs LOW end marker.

Deviation from the C: the C sends a first data bit of 1 as one long HIGH, which its own decoder decodes
shifted by one bit. Real fobs send `H s, L s, H s` (frames with hop bit 31 = 1 in the Fiat Bravo capture
start `-754 +201 -218 +145`), i.e. a short HIGH followed by the normal transition from an implied 1, and so
does KAT. For a first bit of 0 the upload is identical to the C's.

## Validation

- `IMPORTS/FIAT/Fiat Bravo 2010 S.Croce 433.88 am650.sub` decodes as 11 Fiat V0 frames with one fix
  (0x54536200) and a rolling hop; one frame (hop 0xA03BF868) equals the saved key file
  `IMPORTS/FIAT/Fiat type 0 N1.sub` (`Key: A0 3B F8 68 54 53 62 00`).
- C oracle: the capture above decodes to the same 11 frames in ProtoPirate, Flipper-ARF (`fiat_v0.c` and its
  older duplicate "FIAT SPA", `fiat_spa.c`, also registered in ARF) and doctor-firmware; 1500 randomized
  bursts decode identically in all four (2524 frames). The replay uploads equal ProtoPirate's except the
  first-bit-1 deviation above and frames whose first tail bit is 1 (KAT keeps all 7 tail bits; never
  seen on a real capture). All references are replay-only (rolling hop, no key).
- The preamble step mirrors the current C: a gap of exactly 799 µs used to underflow in KAT (a panic in
  debug builds, a missed frame in release); the 701–899 µs window now matches the C (unit test).
- Encode→decode round trip for real frames with hop bit 31 = 0 and 1 (bursts 1 and 2 decode; burst 3 has no
  following preamble); the frame start matches the reference; a 1000 µs gap or a short preamble is rejected.
