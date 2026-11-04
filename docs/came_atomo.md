---
layout: default
---

# CAME Atomo Protocol

**Rust module:** `src/protocols/came_atomo.rs`
**Reference:** Flipper-ARF garage app `applications/system/subghz_garage/protocols/came_atomo.c` (primary),
plus doctor-firmware / Flipper-ARF `lib/subghz/protocols/came_atomo.c` (the 19 ms header and buttons 5/6).

## Overview

CAME Atomo (TOP42R, TOP44R, TOP44RBN, TOPD4REN …) gate remotes send a 62-bit Manchester rolling code
whose cipher is self-contained (no key or table): an LFSR keystream, seeded by the first byte, XORed over
bits 8..58. Each press sends a burst of frames whose first byte, a hold-cycle counter (0..0x7F), rotates
while the button is held; the 16-bit counter advances per press.

## Timing

| Parameter   | Value     | Notes                                                       |
|-------------|-----------|-------------------------------------------------------------|
| Short       | 600 µs    | ±250 µs                                                     |
| Long        | 1200 µs   | ±250 µs                                                     |
| Header      | 12 / 19.2 / 72 ms | LOW; ±5 / ±2.5 / ±10 ms (19.2 ms: doctor/ARF-lib only)  |
| End         | ≥ 2650 µs | LOW (te_long × 2 + te_delta); ends a frame                  |
| Bits        | 62        | counted from 1, so 61 received bits                         |

## Frame Layout

Received: a HIGH 1200 start (decodes as a 1, bit 60) and 60 data bits, Manchester through Flipper's
`manchester_decoder.c`, each bit stored inverted. `data` = these 61 bits; `data_count_bit` = 62.

Fields: `pack = big-endian bytes of (!data << 4)`, decrypted (`atomo_decrypt`):

- **pack[0]:** hold-cycle counter ("Btn_Cnt")
- **pack[1..2]:** counter (16-bit)
- **pack[3..6]:** serial (32-bit)
- **pack[7] >> 4:** button code 0, 2, 4, 6, 0xC, 0xE → button 1..6 (left-up, right-up, left-down,
  right-down, 5, 6). The C keeps its previous button for any other code; so does KAT.

No checksum: `crc_valid` is false and every 62-bit frame is reported, as in the C.

## RF

- **Encoding:** Manchester (1 = LOW then HIGH half, stored inverted)
- **RF modulation:** AM (OOK), 433.92 MHz (no frequency filter)
- **Encryption:** rolling code, Atomo LFSR cipher (keyless)

## Decoder Steps

1. **Reset** — a LOW of 12 ms (TOP44RBN), 19.2 ms (TOPD4REN; doctor/ARF-lib) or 72 ms (TOP42R/TOP44R) →
   Manchester Reset + ShortLow, count = 1.
2. **DecoderData** — 600/1200 µs pulses become Manchester events; a LOW ≥ 2650 µs reports the frame when
   exactly 62 bits are counted and restarts the bit buffer *without* leaving the data state (back-to-back
   frames decode one after another); any other pulse returns to Reset.

In a Flipper receiver, which resets every decoder after a decode, the pause that ends one frame cannot
also start the next, so 4 of the encoder's 8 frames decode (Flipper's unit test feeds the decoder
directly and sees all 8).

## Encoder

`deserialize` + `get_upload`: decrypt the key; counter + 1 (`CounterMode` 1: 0 → 1 → 0xFFFE → 0xFFFF → 0;
2: 0x807B → 0x807C → 0x007B → 0x007C → 0x807B; ≥ 3: frozen); button = the requested native button 1..6
(else the captured one) → its code; upload `HIGH 18000, LOW 72000`, then 8 frames with hold-cycle bytes
10, 30, 125, 126, 127, 0, 1, 3: `atomo_encrypt`, invert, `>> 4`, `HIGH 1200, LOW 600`, the 60 bits through
Flipper's `manchester_encoder.c` (a final LOW half is dropped), `LOW 68000`. One copy.

The garage app's TX plugin is built without its custom-button and firmware-extension flags: it always
sends button code 0 and the "OFEX" counter (0xFFFE after any counter ≥ 1). KAT sends the captured or
requested button with counter + 1, as doctor-firmware does.

## Key file

`Protocol: CAME Atomo`, `Bit: 62`, `Key` (the received bits, as the garage app's history saves them —
serialize runs before `get_string`, which rewrites `generic.data` in the garage copy), optional
`CounterMode`.

## Validation

- Flipper unit test `came_atomo_raw.sub`: 6 frames (serial 0x12EA4998, button 1, counters 0x341/0x342),
  identical to both oracles; the C's own worked example (0x1FF08D9924984115 → hold 5, counter 0x01FD,
  serial 0xA083) is a unit test.
- Oracle parity (`--protocol "CAME Atomo"`, reset after each decode): identical to doctor-firmware on 385
  corpus files (250 decodes) and 6100 fuzzed files (4092 decodes). The garage copy differs only where KAT
  takes the union: buttons 5/6 (garage shows its previous button) and one fuzzed file with a 19 ms header.
- Encoder: pulse-identical to doctor-firmware's upload for 60 key files (all counter modes); garage uploads
  differ only by the TX-plugin button/counter artefact above (checked by decoding them).
- No rtl_433 recording exists for CAME Atomo.
