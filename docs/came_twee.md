---
layout: default
---

# CAME TWEE Protocol

**Rust module:** `src/protocols/came_twee.rs`
**Reference:** Flipper-ARF garage app `applications/system/subghz_garage/protocols/came_twee.c` (primary);
doctor-firmware `lib/subghz/protocols/came_twee.c` (12 ms header), Flipper-ARF
`lib/subghz/protocols/came_twee.c` (button switch); Flipper's `lib/toolbox/manchester_{de,en}coder.c`

## Overview

CAME TWEE / TWIN (and TOP44FGN) remotes send a 10-bit DIP code in 15 Manchester parcels of 54 bits, each
XORed with one entry of a fixed "rainbow" table (`came_twee_magic_numbers_xor`) — a "decreasing counter"
from 0xE to 0x0 in the low nibble. Every parcel decodes to the same fields.

KAT takes the union of the three copies: the garage decoder (51 ms header) plus doctor-firmware's
TOP44FGN 12 ms header, and Flipper-ARF lib's encoder button switch. doctor's encoder sends the 15 parcels
once (`encoder.repeat` = 1); KAT sends them 3 times, as ARF.

## Timing

| Parameter   | Value             | Notes                                           |
|-------------|-------------------|-------------------------------------------------|
| te_short    | 500 µs            | ±250 µs                                         |
| te_long     | 1000 µs           | ±250 µs                                         |
| Header      | 51000 ± 5000 µs   | TOP44FGN: 12000 ± 2500 µs (doctor-firmware)     |
| Parcel end  | LOW ≥ 2250 µs     | the encoder leaves 51 ms after every parcel     |
| Bits        | 54                |                                                 |

## Frame Layout (54 bits, sent inverted, Flipper Manchester: 1 = LOW then HIGH)

`0x003FFF72 << 32 | (code ^ MAGIC[i])`, i = 14 … 0 (the low nibble is the parcel index). Fields
(`subghz_protocol_came_twee_remote_controller`):

- **serial** = low 32 bits ^ `MAGIC[low nibble]` (a nibble of 0xF indexes past the C's table; KAT XORs 0);
- **button** = (serial / 4 >> 4) & 0xF;
- **counter** = the 10-bit DIP code the C keeps in `generic.cnt` (high half of serial / 4, bit-mirrored, >> 6).

## RF

- **Encoding:** Manchester (500 / 1000 µs)
- **RF modulation:** AM (OOK), 433.92 MHz
- **Encryption:** none (fixed DIP code, table-XORed parcels)
- **Buttons:** "Btn 0" … "Btn F"

## Decoder Steps

1. **Reset** — a header LOW primes Flipper's Manchester decoder (LongLow, LongHigh, ShortLow) → data.
2. **Data** — pulses within ±250 µs of 500/1000 µs are Manchester events (bit = inverted decoder bit);
   a LOW ≥ 2250 µs ends the parcel: exactly 54 bits are reported, the decoder re-primes; any other pulse →
   Reset.

With the app's reset inside the callback, the LOW that ended a decoded parcel is not reused as a header,
so every other parcel of a transmission decodes (KAT does the same).

## Encoder

`subghz_protocol_encoder_came_twee_get_upload`, `encoder.repeat` = 3: fields from the key; `button` 0..=15
is written into serial bits 6..9 (ARF lib), larger codes keep it; then the 15 parcels with Flipper's
Manchester encoder (the final half-bit is kept only when HIGH) and a 51 ms LOW after each.

## Key file

`Protocol: CAME TWEE`, `Bit: 54`, `Key`.

## Validation

Against the garage C oracle (`decode --protocol "CAME TWEE"`): 15 identical decodes on the Flipper
unit-test corpus and 3515 on 3420 fuzz files; none on IMPORTS. With every 51 ms gap replaced by 12 ms
(9 files) KAT and the doctor oracle both report 201 parcels and the garage oracle none (the union above).
Encoder: 26 uploads (the unit test's `came_twee.sub` + 25 random keys) equal the garage oracle's pulse for
pulse; doctor's are the first third (one repeat). Unit tests: the C comment's parcel table gives one set
of fields, encode → decode of all parcels, button switch, 12 ms header, rejection, key file, RAW capture.
