---
layout: default
---

# Ditec GOL4 Protocol

**Rust module:** `src/protocols/ditec_gol4.rs`
**Reference:** Flipper-ARF garage app `applications/system/subghz_garage/protocols/ditec_gol4.c` (primary;
decode-only in that app, which has no TX plugin for it) and doctor-firmware's
`lib/subghz/protocols/ditec_gol4.c` (identical, and it transmits).

## Overview

Ditec GOL4 gate remotes send a 54-bit rolling code. Its "encryption" needs no key: a bit shuffle (byte
reversals, a 3-bit rotation of bytes 2..6 with carry chaining, a nibble swap and inversion of the button
byte) and an 8-bit LCG `x → 21x + 1` over the counter bytes.

## Timing

| Parameter   | Value     | Notes                                                    |
|-------------|-----------|----------------------------------------------------------|
| Short       | 400 µs    | ±200 µs                                                  |
| Long        | 1100 µs   | ±200 µs                                                  |
| Gap         | 24200 µs  | LOW, ±4400 µs (opens a frame)                            |
| Start bit   | 800 µs    | HIGH, ±200 µs                                            |
| End         | 22000 µs  | the stored LOW before the next start, ±3300 µs           |
| Bits        | 54        | exactly                                                  |

## Frame Layout

54 bits MSB first, as 7 bytes (the last 2 bits 0). Decoding (`decode_key`): bytes 2..6 rotate right by 3
bits; bytes 0, 1, 3, 4, 5, 6 are bit-reversed; byte 2 is masked 0xDF, nibble-swapped, inverted and
bit-reversed; byte 5 (flipped when byte 6's MSB is set) and byte 6 undo 254 and byte-5 LCG steps and are
XORed with 0xA7 / 0x69. Then:

- **serial** = bytes `0 ‖ 4 ‖ 1 ‖ 3`
- **button** = byte 2 & 0x0F: 1, 2, 4, 8 = buttons 1..4, 0 = programming
- **counter** = byte 5 | byte 6 << 8

No checksum: `crc_valid` is false and every 54-bit frame is reported.

## RF

- **Encoding:** PWM, LOW first (1 = LOW 400 + HIGH 1100, 0 = LOW 1100 + HIGH 400)
- **RF modulation:** AM (OOK), 433.92 MHz (no frequency filter)
- **Encryption:** rolling code (keyless shuffle + LCG)

## Decoder Steps

1. **Reset** — LOW ≈ 24.2 ms → StartBit; **StartBit** — HIGH ≈ 800 µs.
2. **SaveDuration / CheckDuration** — (LOW, HIGH) pairs → bits; an unmatched HIGH keeps the step.
3. **End** — on a LOW after such an unmatched HIGH, the *previous* LOW is checked against 22 ms ± 3.3 ms:
   the frame is reported at the first data LOW of the next transmission (gap + start bit + LOW), and the
   decoder resets. Of the encoder's 4 copies, the 1st and 3rd decode; the last copy never can.

## Encoder

`deserialize` + `encode_key` + `get_upload`: decode the key, counter + 1 (firmware multiplier 1, wrap to 0),
button = the requested native nibble (0..15, else the captured one; 0 is sent as 0xF0), re-encode; upload
`LOW 24200, HIGH 800, 54 bits`, 4 copies.

## Key file

`Protocol: Ditec GOL4`, `Bit: 54`, `Key`.

## Validation

- No Flipper unit test or rtl_433 recording exists for Ditec GOL4. The C's sample keys of one remote
  (comment in `decode_key`) decode to serial 0xCC093483 with their buttons and re-encode to themselves.
- Oracle parity (`--protocol "Ditec GOL4"`): identical to the garage and doctor builds on 385 corpus files
  including 60 KAT-encoded keys (120 decodes) and 6100 fuzzed files (1401 decodes).
- Encoder: pulse-identical to doctor-firmware's upload for 60 random key files (the garage build has no
  Ditec encoder).
