---
layout: default
---

# Microchip HCS361 (KeeLoq) Protocol

**Rust module:** `src/protocols/microchip_hcs361.rs` (rtl_433 receive chain: `src/protocols/rtl433/`)
**Reference:** rtl_433 `src/devices/hcs361.c` (GPL-2.0; model `Microchip-HCS361`), its six `r_device`s

## Overview

Microchip's HCS361 KeeLoq code-hopping encoder (Jeep / Chrysler, Marelli and Leer remotes at 315 MHz in rtl_433's
notes). The chip's EEPROM picks one of several line codings; rtl_433 registers one `r_device` per coding, all with
the same decoder, and KAT runs all six side by side. The 67-bit frame carries the same 64-bit KeeLoq code word as an
HCS200, so KAT hands it to its KeeLoq identification: a frame a keystore key decrypts shows the manufacturer and the
counter. Receive only.

## Timing / modulation

| r_device (`rtl_433 -R`) | Slicer | Short / long | Gap / reset | Tolerance | Sync |
|---|---|---|---|---|---|
| Sync, 833 bit/s (306) | OOK PWM | 400 / 800 µs | 1200 / 7200 µs | 140 µs | 4000 µs |
| Sync, 1667 bit/s (307) | OOK PWM | 200 / 400 µs | 600 / 13600 µs | 70 µs | 2000 µs |
| No Sync, 833 bit/s (308) | OOK PWM | 200 / 400 µs | 1200 / 6800 µs | 140 µs | – |
| No Sync, 1667 bit/s (309) | OOK PWM | 100 / 200 µs | 600 / 6600 µs | 70 µs | – |
| PIWM, 2500 bit/s (310) | OOK PIWM_DC | 400 / 800 µs | – / 45600 µs | 140 µs | – |
| PIWM, 5000 bit/s (311) | OOK PIWM_DC | 200 / 400 µs | – / 45200 µs | 70 µs | – |

PWM: a short pulse is a 1, a long one a 0 (the bit is pulse + gap), a pulse in the sync window a sync, any other
longer pulse starts a new row; a gap over the gap limit starts a new row, one over the reset limit ends the message.
PIWM_DC: every pulse and every gap is a symbol (short = 1, long = 0); other symbols end the row. KAT's port follows
`src/pulse_slicer.c` exactly, at one sample per µs, on rtl_433's packages (see `src/protocols/rtl433/`).

## Frame Layout

Row 0 = preamble, row 1 = exactly 67 bits, LSB-first fields:

| Bits | Field |
|---|---|
| 0–31 | encrypted hop |
| 32–59 | serial (28 bits) |
| 60–63 | button status S3, S0, S1, S2 |
| 64 | VLOW (battery low) |
| 65–66 | 2-bit CRC |

## Decoder Steps

1. Each `r_device`'s slicer reads the pulses; at End of Message the bit buffer goes to `hcs361_decode`.
2. Checks, as the C: row 1 has 67 bits; a 6-bit preamble row is `0xFC`, a 7-bit one `0xFE`, a 12-bit one `0xFFF0`
   (PWM) or `0xAAA0` (PIWM) — other lengths are not checked; the data is not all 0xFF; the 2-bit CRC over the
   65 bits, also accepted with the last (VLOW) bit inverted (the datasheet's battery-low check); the serial ≠ 0.
3. The first 64 bits go through `keeloq.rs` (`KeeloqDecoder::parse_data`: AN-Motors / HCS101 rules, every keystore
   entry and learning type, embedded and user keystore).

The first `r_device` to complete a frame reports it (on a shared pulse, in the table's order); a registry reset after
KAT's own report keeps the package going, as rtl_433 does after each message.

## Field mapping

| KAT | rtl_433 | Notes |
|---|---|---|
| display name | `model` | `Microchip-HCS361 [KeeLoq (<manufacturer>)]`, or `[KeeLoq (Unknown)]` when no key fits |
| `serial` | `id` | 28 bits |
| `button` | `button` | KeeLoq's function code `fix >> 28` (S2 S1 S0 S3); rtl_433 prints `(btn & 1) << 3 \| btn >> 1` |
| `counter` | – | decrypted counter (identified frames only) |
| `data` | `encrypted` + `id` | first 64 wire bits; `encrypted` = their first 32 bits reversed |
| `extra` | `battery_ok` | VLOW (bit 2), CRC bits (3 bits) |
| `data_count_bit` | | 67 |
| `crc_valid` | `mic: CRC` | always true for a reported frame |

## Encoder

None — receive only, as rtl_433. KAT's KeeLoq encoder sends the HCS200 line coding (and its "Superrollo" HCS361
framing only at TE 450 µs for that manufacturer), so these frames are not re-encoded: the app offers Replay of the
recorded pulses and export.

## Validation

- **rtl_433_tests `Microchip/HCS361`** (10 recordings, 3 remotes) through KAT's IQ pipeline: every frame rtl_433
  reports is decoded with the same id, button and hop (`rtl_433_tests_recordings`, ignored test). `04/unlock` begins
  with ~87 ms of zero bytes and a settling receiver (RTL-SDR start-up); KAT's receiver no longer learns its noise floor
  from such samples, and the file decodes (3 frames, as rtl_433). No keystore key identifies the remotes; `02/g001` (3 frames) matches
  KAT's `SL_A2-A4` key by chance — the other frames of that remote stay Unknown (KeeLoq identification accepts ~6 %
  of random frames, as for HCS200 frames).
- **rtl_433 oracle** (host build, same pulses through both): rtl_433's own pulses of the 10 recordings — identical
  output per `r_device` (306: 0, 307: 3, 308: 15, 309: 0, 310: 12, 311: 9 frames); KAT's own pipeline pulses (22
  burst paths and polarities) — identical per `r_device` (46 frames; with all six at once rtl_433 prints 3 more: in
  folder 01 r_devices 307 and 308 finish the same frame on the same final gap and KAT reports one decode per pulse);
  1000 synthetic packages (every coding, jitter, preamble lengths, bit flips, truncation) — identical per `r_device`
  (69 / 63 / 114 / 118 / 34 / 32 frames).
- **IMPORTS**: one new decode, `HOLDEN/Holden Barina 2016 AU.sub` (an Opel V1 fob): r_device 309 reads Opel V1's
  120/240 µs Manchester as 100/200 µs PWM and 1 row of 4 passes the 2-bit CRC; rtl_433 decodes the same pulses the
  same way (id 0CB3DB36, button 14, encrypted FABDAF56). No decode of another device's recording in rtl_433_tests or
  the Flipper unit-test captures.
