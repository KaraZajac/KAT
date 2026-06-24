---
layout: default
---

# Ford V1 Protocol

**Rust module:** `src/protocols/ford_v1.rs`
**Reference:** `REFERENCES/ProtoPirate/protocols/ford_v1.c`

## Overview

Ford V1 uses Manchester encoding at 65/130 µs. 136 bits / 17 bytes: key1 (bytes 0–6, 56 bits) + key2
(bytes 7–14, 64 bits) + CRC16 (bytes 15–16). Preamble ≥50 long pulses, then a short-pulse sync window
(`sync_event_count > 2`) replays buffered Manchester events and enters the 17-byte data collection. This
is a ROLLING-code protocol.

Crypto: a proprietary parity-based descrambling cipher operating on the 9-byte air block `raw[6..15]`,
plus CRC16/CCITT (poly `0x1021`, init `0x0000`) over `raw[3..15]`. Emission is gated on CRC16 validity
(with a 17-byte bit-inverted fallback) so it never false-matches. A strict branch
(`decoded[3]==raw[5] && decoded[4]==raw[6]`) yields plaintext serial/button/counter; otherwise it is
classified as encrypted/rolling (device id only). The Manchester transition table is the Flipper
differential-Manchester table (same as Ford V0/V2).

## Timing

| Parameter   | Value  | Notes                        |
|-------------|--------|------------------------------|
| Short       | 65 µs  | ±39 µs (te_delta)            |
| Long        | 130 µs | ±39 µs (preamble uses ±40)   |
| Min bits    | 136    | 17 bytes                     |
| Preamble    | ≥50 long pulses |                     |

## Frame Layout (136 bits / 17 bytes)

- **bytes 0–6:** key1 (56 bits, big-endian) → `DecodedSignal.data`
- **bytes 6–14:** air block (9 bytes) — descrambled to plaintext
- **bytes 15–16:** CRC16/CCITT over bytes 3–14

Plaintext fields (when the strict branch matches): serial = `plain[1..3] + plain[0]`, button = `plain[5]>>4`
(Sync=0, Lock=1, Unlock=2, Trunk=4, Panic=8), counter = `((plain[5] & 0x0F) << 16) | plain[6..7]`. The
`extra` word stashes the CRC16, the strict flag, and `plain[4]` so the encoder can rebuild the full frame.

## RF

- **Encoding:** Manchester (Flipper differential table)
- **RF modulation:** FM
- **Encryption:** proprietary parity descramble cipher + CRC16/CCITT gate; rolling code
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder Steps

1. **Reset** — a long LOW pulse begins the preamble (count = 1).
2. **Preamble** — count long pulses; after ≥50, a short pulse enters Sync.
3. **Sync** — buffer short/long events; once `sync_event_count > 2`, replay the buffered events into Manchester and enter Data.
4. **Data** — Manchester-decode and pack 17 bytes; at the 17th byte run `process_data` (CRC16 gate + descramble + field extraction) and emit on a valid CRC.

## Encoder

Supported (6 bursts). Faithful re-encode is only possible when the original frame's plaintext was
recovered (strict branch); the encoder rebuilds the plaintext from fields + the stashed `plain[4]`,
re-derives the air block + CRC16, and emits 6 bursts of a 400-pair long preamble + sync + 136 Manchester
bits, with a per-burst `pkt[4]` override.

## Validation

Verified by an encode→decode round-trip / synthetic-frame check (no local capture available). Unit tests
cover a CRC16/XMODEM known vector, the descramble round trip, the strict-branch decode, and a full
encode→decode at the on-air-frame level.
