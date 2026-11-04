---
layout: default
---

# Ford V2 Protocol

**Rust module:** `src/protocols/ford_v2.rs`
**Reference:** `REFERENCES/ProtoPirate/protocols/ford_v2.c` (decoder, HEAD dc3446b); encoder from Flipper-ARF `lib/subghz/protocols/ford_v2.c`

## Overview

Ford V2 uses Manchester encoding at 200/400 µs. 104 bits total (13 bytes). The frame begins with a
16-bit Manchester sync that equals `0x7FA7` (the decoder matches the *inverted* shift register against
`!0x7FA7`); those two sync bytes `0x7F 0xA7` head the 13-byte buffer. Decoded data bits are inverted
before packing (`data_bit = !bit`). There is no CRC — structure is validated by the two sync bytes plus
a known button code, so Ford V2 will not false-match.

Manchester is decoded with the shared port of Flipper's `manchester_decoder.c` (`common.rs`), with Ford V2
polarity `level ? ShortHigh : ShortLow` / `level ? LongHigh : LongLow`. Entering Sync resets the state to
Mid1 and forces Mid0 for the triggering long LOW (a Low event), as `ford_v2.c` does. The encoder's on-air
1 = HIGH,LOW decodes as 0 under this mapping, hence the bit inversion and the inverted sync match.

## Timing

| Parameter | Value  | Notes                       |
|-----------|--------|-----------------------------|
| Short     | 200 µs | ±260 µs (te_delta)          |
| Long      | 400 µs | ±260 µs                     |
| Min bits  | 104    | 13 bytes                    |
| Preamble  | ≥64 short pulses | before the sync   |

## Frame Layout (104 bits / 13 bytes)

- **bytes 0–1:** sync `0x7F 0xA7`
- **bytes 2–5:** serial (32-bit, big-endian)
- **byte 6:** button — valid codes `0x10` (Lock), `0x11` (Unlock), `0x13` (Trunk), `0x14` (Panic), `0x15`
- **bytes 7–8 + byte 9 MSB:** counter = `((b7 & 0x7F) << 9) | (b8 << 1) | (b9 >> 7)`; byte 7 MSB carries a button parity bit (refreshed by the encoder)
- **bytes 9–12:** rolling/hop tail (stashed in `extra` so the encoder can rebuild the full frame)

The exported `data` (Key) word is the top 8 bytes; bytes 8–12 (40 bits) are carried in `extra`.

## RF

- **Encoding:** Manchester (Ford V2 polarity `level ? High : Low`; decoded bits inverted)
- **RF modulation:** FM (per ProtoPirate `SubGhzProtocolFlag_FM`)
- **Encryption:** none — validated by sync bytes + valid button code
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder Steps

1. **Reset** → a short pulse (~200 µs) begins the preamble.
2. **Preamble** — count short pulses; after ≥64 shorts a long LOW pulse enters Sync.
3. **Sync** — Manchester-decode bits into a 16-bit shift register; when it matches `!0x7FA7`, prime the two sync bytes and enter Data.
4. **Data** — Manchester events feed the state machine; decoded bits are inverted and packed MSB-first into 13 bytes. At 104 bits the sync bytes and button are validated and the frame committed. A non-short/long pulse (gap) ends the attempt.

## Encoder

Decoder as ProtoPirate. The encoder follows Flipper-ARF (`ford_v2_encoder_deserialize_validate_and_pack`),
a superset of ProtoPirate's plain replay: rebuild the 13-byte frame from `data` (bytes 0–7) and `extra`
(bytes 8–12, `TailRaw`), apply the serial, re-pack the counter + 1 (16 bits across bytes 7–9; ProtoPirate
ignores Btn/Cnt), set the button (KAT Lock/Unlock/Trunk/Panic → 0x10/0x11/0x13/0x14, ARF's D-pad map;
other commands keep the captured button), refresh the byte-7 parity bit, refuse an invalid button, then
emit 6 bursts of a 70-pair preamble + sync + 103 Manchester bits (encoder short 240 µs), separated by
HIGH 16 ms gaps. The Flipper repeats the upload 10 times; KAT returns one.

## Validation

No local capture is available (`IMPORTS/FORD/22_ford_escape_u2_l2_t2_p2.sub` has 240 µs-based bursts but
not this frame structure). Differential tests against the compiled C: 1500 randomized bursts (valid and
invalid buttons, broken sync, jitter, short preambles) decode identically to ProtoPirate's decoder
(1809 frames); 2500 uploads equal Flipper-ARF's encoder pulse for pulse. Unit tests: encode→decode through
the full 6-burst waveform for each button and counter carry (0x01FF, 0xFFFF); a captured Remote start is
kept for other commands; a wrong sync byte or an unknown button code is rejected.
