---
layout: default
---

# Scher-Khan Protocol

**Rust module:** `src/protocols/scher_khan.rs`
**References:** `REFERENCES/ProtoPirate/protocols/scher_khan.c` (decoder),
`REFERENCES/Flipper-ARF/lib/subghz/protocols/scher_khan.c` (decoder + MAGIC CODE PRO1/PRO2 decryption + encoder)

## Overview

Scher-Khan (Magic Code / Magicar) car-alarm remotes. ProtoPirate decodes the frame and names the variant by
bit count but has no encoder; Flipper-ARF uses the same decoder, adds decryption of the 57-bit MAGIC CODE
PRO1/PRO2 frames (π-obfuscated key tables + CRC-8 check) and an encoder. KAT carries the union: every
frame ProtoPirate decodes, ARF's PRO1/PRO2 field recovery, and ARF's encoder.

The decoded variant is shown as a display-name suffix, e.g. `Scher-Khan [MAGIC CODE PRO1]` (ProtoPirate's
"Pt:" line); `ProtocolRegistry::get()` resolves it to `Scher-Khan`.

## Timing

| Parameter | Value   | Notes                                           |
|-----------|---------|-------------------------------------------------|
| Short (0) | 750 µs  | ±160 µs, HIGH + LOW                             |
| Long (1)  | 1100 µs | ±160 µs, HIGH + LOW                             |
| Header    | 1500 µs | ≥2 HIGH/LOW pairs, then a 750/750 start bit     |
| Stop      | HIGH ≥ 1420 µs |                                          |
| Min bits  | 35      | count includes the start bit                    |

## Frame Layout (by bit count)

| Bits  | Variant                    | Fields                                                     |
|-------|----------------------------|------------------------------------------------------------|
| 35    | MAGIC CODE, Static         | none                                                       |
| 51    | MAGIC CODE, Dynamic        | serial = `((d>>24)&0xFFFFFF0) \| ((d>>20)&0xF)`, button = `(d>>24)&0xF`, counter = `d&0xFFFF` |
| 57    | MAGIC CODE PRO1 / PRO2     | bytes `kb[i] = d >> (48-8i)`; CRC-8 (init 0xFF) over kb[0..6] = kb[6] with the PRO1 then PRO2 polynomial; decrypt → serial = d0:d1:d2, button = d3 & 0xF, counter = d4:d5 |
| 63    | MAGIC CODE, Response       | none                                                       |
| 64    | MAGICAR, Response          | none                                                       |
| 81/82 | MAGIC CODE PRO, Response   | none                                                       |
| other | Unknown                    | none                                                       |

A 57-bit frame whose CRC matches neither polynomial is reported as `MAGIC CODE PRO/PRO2` with
`crc_valid = false`. Frames longer than 64 bits keep the first 64 wire bits in `data` and the rest in
`extra` (KAT convention); they are not encoder-capable (the C keeps only the last 64 bits).

## RF

- **Encoding:** PWM
- **RF modulation:** FM (ProtoPirate), FM + AM (ARF)
- **Encryption:** none (51-bit), PRO1/PRO2 bit-permutation cipher + CRC-8 (57-bit)
- **Frequencies:** 315 MHz, 433.92 MHz (ARF; ProtoPirate 433.92 MHz)

## Decoder Steps

1. **Reset** — wait for a ~1500 µs HIGH.
2. **CheckPreamble** — count 1500 µs header pairs; a 750/750 pair after ≥ 2 of them is the start bit.
3. **SaveDuration / CheckDuration** — 750/750 = 0, 1100/1100 = 1.
4. **Stop** — a HIGH ≥ 1420 µs ends the frame; ≥ 35 bits are reported (no other gate, as in the C).

## Encoder

ARF `subghz_protocol_encoder_scher_khan_get_upload`: button and counter are re-derived from the key,
counter + 1 (wraps to 0 past 0xFFFF).

- **51 bits:** bits 16..50 of the key with bits 24..27 = button, low 16 bits = counter. **KAT deviation
  (encoder fix):** ARF masks with `0x7FFFFFFF0000`, keeping only bits 16..46 — that clears the top of the
  serial (bits 47..50), so ARF's own sample frame `0E8C1619E830C` would go out as `068C…`, a different
  remote. KAT keeps bits 16..50. ARF also leaves the inverted-button nibble (bits 16..19 = `!button`; the
  sample carries button 0x1 with nibble 0xE) stale when the button changes; when the captured frame follows
  that convention KAT rewrites it with the new button (`0E8C1619E830C` + Unlock → `0E8C1629D830D`).
- **57 bits:** decrypt with the table the PRO1 CRC selects (else PRO2), set `d3 = (d3 & 0xE0) | button`,
  `d4:d5 = counter`, re-encrypt, new CRC.
- **Other lengths ≤ 64:** key sent unchanged.

Upload: 6 × (750, 750), 3 × (1500, 1500), start bit (750, 750), the key MSB-first (1 = 1100/1100,
0 = 750/750), 1500 µs HIGH — sent 7 times (C `encoder.repeat`). The encoder sends the start bit *and* the
key's leading zero, so its frames decode one bit longer (52 / 58 bits) — reference behaviour. Because that
reading has no fields, KAT keeps the key it encoded on the capture itself (`sent_signal`: the new button and
counter, what the C's emulate scene writes back into its file), so the next press sends the code after; until
v1.8.7 the capture never advanced and every press re-sent counter + 1.

Buttons: 1 Lock, 2 Unlock, 3 Lock+Unlock, 4 Trunk, 5 Lock+Trunk, 6 Unlock+Trunk, 7 Lk+Ul+Tr, 8 Start,
9 Lock+Start, A Unlock+Start, C Trunk+Start, F All/Panic. KAT Lock/Unlock/Trunk/Panic → 1/2/4/F; other
codes 1..F pass through.

**Compustar 1WG3R (KAT deviation):** a 35-bit "MAGIC CODE, Static" frame whose inverted bits pass Compustar
1WG3R's checks (111 marker, ID not 0000 / FFFF, inverse button byte) is not reported as Scher-Khan: it is the same
air format, and rtl_433's recordings of those remotes show the frames carry an ID and a button, which the Compustar
decoder reads ([compustar_1wg3r.md](compustar_1wg3r.md#overlap-with-scher-khan)). Other 35-bit frames, and every
other length, decode as before; Scher-Khan key files load and transmit as before.

## Validation

C oracle (unmodified ARF `scher_khan.c` on the host): decoder field-for-field and encoder pulse-for-pulse
on randomized frames (35..81 bits, jitter, all button codes) — no mismatch; the IMPORTS sweep has no
Scher-Khan capture. Unit tests: the C's own 51-bit sample frame, PRO crypto round trip and CRC table
selection, PRO1/PRO2 field decode, encoder layout/quirks/round trips, >64-bit storage.
