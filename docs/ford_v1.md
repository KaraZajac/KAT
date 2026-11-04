---
layout: default
---

# Ford V1 Protocol

**Rust module:** `src/protocols/ford_v1.rs`
**Reference:** `REFERENCES/ProtoPirate/protocols/ford_v1.c` (HEAD dc3446b). Flipper-ARF's
`lib/subghz/protocols/ford_v1.c` has the same decoder and builds the same upload (it bumps the counter
inside the encoder instead of in an emulate scene); doctor-firmware's copy decodes a strict subset.

## Overview

Ford V1 uses Manchester encoding at 65/130 µs. 136 bits / 17 bytes: key1 (bytes 0–6, 56 bits) + key2
(bytes 7–14, 64 bits) + CRC16 (bytes 15–16). Preamble ≥50 long pulses, then a short-pulse sync window
(`sync_event_count > 2`) replays buffered Manchester events and enters the 17-byte data collection. This
is a ROLLING-code protocol.

Crypto: a proprietary parity-based descrambling cipher operating on the 9-byte air block `raw[6..15]`,
plus CRC16/CCITT (poly `0x1021`, init `0x0000`) over `raw[3..15]`. Every CRC-valid frame is reported
(with a 17-byte bit-inverted fallback), as in the C. A strict branch
(`decoded[3]==raw[5] && decoded[4]==raw[6]`) yields plaintext serial/button/counter; otherwise the C
reports the header serial `raw[3..7]` with button and counter 0.

Manchester is decoded with the shared port of Flipper's `manchester_decoder.c` (`common.rs`), mapping
`level ? ShortHigh : ShortLow` / `level ? LongHigh : LongLow` (ford_v1.c). Since the encoder sends on-air
1 = HIGH,LOW, the decoded bytes are the inverse of the frame; the CRC check's 17-byte inversion fallback
undoes that (as in the C).

## Timing

| Parameter   | Value  | Notes                        |
|-------------|--------|------------------------------|
| Short       | 65 µs  | ±39 µs (te_delta)            |
| Long        | 130 µs | ±39 µs (preamble uses ±40)   |
| Min bits    | 136    | 17 bytes                     |
| Preamble    | ≥50 long pulses |                     |

## Frame Layout (136 bits / 17 bytes)

- **bytes 0–6:** key1 (56 bits) → `DecodedSignal.data` (the C's `Key`)
- **bytes 7–14:** key2 (64 bits) → `DecodedSignal.extra` (the C's `Key_2`)
- **bytes 6–14:** air block (9 bytes) — descrambled to plaintext
- **bytes 15–16:** CRC16/CCITT over bytes 3–14 (the C's `Key_3`; recomputed by KAT)

Plaintext fields (when the strict branch matches): serial = `plain[1..3] + plain[0]`, button = `plain[5]>>4`
(Sync=0, Lock=1, Unlock=2, Trunk=4, Panic=8), counter = `((plain[5] & 0x0F) << 16) | plain[6..7]` (20
bits; `DecodedSignal.counter` holds the low 16, the top nibble is recovered from the frame).
`encoder_capable` is the C encoder's precondition (plaintext recoverable: strict or round trip).

## RF

- **Encoding:** Manchester 65/130 µs (on-air 1 = HIGH,LOW; decoded inverted, see Overview)
- **RF modulation:** FM
- **Encryption:** proprietary parity descramble cipher + CRC16/CCITT gate; rolling code
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder Steps

1. **Reset** — a long LOW pulse begins the preamble (count = 1).
2. **Preamble** — count long pulses; after ≥50, a short pulse enters Sync.
3. **Sync** — buffer short/long events; once `sync_event_count > 2`, reset the Manchester state to Mid1 (Mid0 if the first buffered event is ShortLow), replay the buffered events and enter Data. The sync LOW merges with a first data bit of 0, so — as in the C — a frame must start with a 1 bit to be decodable.
4. **Data** — Manchester-decode and pack 17 bytes; at the 17th byte run `process_data` (CRC16 gate + descramble + field extraction). Any other pulse (the inter-burst gap) runs `ford_v1_try_last_byte_variants`: with 16 bytes and 6–7 bits of the last byte collected (its final half-bit merged into the gap), every completion of the missing 1–2 bits is tried and the first CRC-valid one reported.

## Encoder

`subghz_protocol_encoder_ford_v1_deserialize` + ProtoPirate's emulate scene: rebuild the frame from
key1/key2, recover the plaintext (refused if it cannot be, as in the C), apply serial, the button (KAT
Lock/Unlock/Trunk/Panic = 1/2/4/8; other commands keep the captured button) and the counter + 1 (20
bits), re-scramble, refresh the CRC, then emit **burst 0 only** (the C never advances `burst_idx`):
`pkt[4] = 0x08` with the CRC recomputed, 400 × (HIGH 130, LOW 130), HIGH 130, LOW 65, the 17 bytes as
Manchester at 65 µs, LOW 50 000 µs. The Flipper repeats this upload 10 times; KAT returns one. KAT
exports made before this port stored a different `extra` (CRC16/strict flag/`plain[4]`); the encoder still
accepts them by rebuilding key2 from the decoded fields and the stored CRC16.

## Validation

No local capture is available. Differential tests against the compiled ProtoPirate C (decoder and
encoder) on 3600 randomized bursts (strict, rolling, random and inverted frames, both final-bit
values, jitter up to ±38 µs): every decode identical field for field; 2000 uploads (decoded → emulate
Btn/Cnt+1 → encoder) identical pulse for pulse, and identical to Flipper-ARF's encoder. Unit tests cover
the CRC, the descramble round trip, both final-bit values, the encode→decode round trip (button, 20-bit
counter carry, checksum byte), CRC rejection, the undecodable first-bit-0 case and legacy `extra` exports.
