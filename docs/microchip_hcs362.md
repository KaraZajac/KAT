---
layout: default
---

# Microchip HCS362 (KeeLoq) Protocol

**Rust module:** `src/protocols/microchip_hcs362.rs` (rtl_433 receive chain: `src/protocols/rtl433/`)
**Reference:** rtl_433 `src/devices/hcs362.c` (GPL-2.0; model `Microchip-HCS362`), `r_device`s "Microchip HCS362
KeeLoq PWM" and "Microchip HCS362 KeeLoq MC"

## Overview

Microchip's HCS362 KeeLoq code-hopping encoder in its two transmission modes, PWM (mode 0) and Manchester (mode 1).
The 69-bit frame starts with the 64-bit KeeLoq code word, which goes through KAT's KeeLoq identification like an
HCS200 frame. Receive only.

## Timing / modulation

| r_device (`rtl_433 -R`) | Slicer | Short / long | Gap / reset | Tolerance |
|---|---|---|---|---|
| PWM (329) | OOK PWM | 200 / 400 µs | 550 / 900 µs | 50 µs |
| MC (330) | OOK PCM | 214 / 214 µs | 600 / 900 µs | 50 µs |

PWM: 12 preamble pulses, a ~3·TE gap, 69 bits (short pulse = 1). MC: the PCM slicer measures the bit period on the
12-pulse preamble (`pulse_slicer_pcm` collects the whole package first, so KAT does too), a ~4·TE gap starts the
data row: the start bit `10`, 69 Manchester bits, a stop bit that breaks the coding.

## Frame Layout (LSB-first fields)

| Bits | Field |
|---|---|
| 0–31 | encrypted hop |
| 32–59 | serial (28 bits) |
| 60–63 | function code S3, S0, S1, S2 |
| 64 | VLOW |
| 65–66 | CRC0, CRC1 (datasheet Equation 3-1 over bits 0–64) |
| 67–68 | QUEUE0, QUEUE1 (repeat counter) |

## Decoder Steps (`hcs362_decode`)

- **PWM**: row 0 exactly 12 bits starting `0xFFF`, row 1 exactly 69 bits.
- **MC**: row 0 16..32 raw bits starting `0xAAAAAA`; row 1 142..148 raw bits starting `10`; Manchester decode from raw
  bit 2 (at most 72 bits; the C requires the decoder's *input position* ≥ 70, i.e. ≥ 34 decoded bits), then invert
  (G.E. Thomas: HIGH-LOW = 1).
- Both: bytes 1..7 not all 0xFF; the 2-bit CRC. Then KeeLoq identification of the first 64 bits.

## Field mapping

| KAT | rtl_433 | Notes |
|---|---|---|
| display name | `model` | `Microchip-HCS362 [KeeLoq (<manufacturer>)]` / `[KeeLoq (Unknown)]` |
| `serial` | `id` | 28 bits |
| `button` | `button` | KeeLoq function code; rtl_433 prints `(btn & 1) << 3 \| btn >> 1` |
| `counter` | – | decrypted counter (identified frames) |
| `data` | `encrypted` + `id` | first 64 frame bits |
| `extra` | `battery_ok`, `repeat` | VLOW, CRC0, CRC1, QUEUE0, QUEUE1 (5 bits; `battery_ok` = !VLOW, `repeat` = QUEUE1·2 + QUEUE0) |
| `data_count_bit` | | 69 |
| `crc_valid` | `mic: CRC` | always true for a reported frame |

## Encoder

None — receive only, as rtl_433 (KAT's KeeLoq encoder sends the HCS200 coding).

## Validation

- **rtl_433_tests `hcs362/01`** (4 recordings at 868.3 MHz, 1 MS/s; one remote programmed for PWM, then MC) through
  KAT's IQ pipeline: all 8 repeats of every recording, with rtl_433's id, button and hop (`rtl_433_tests_recordings`,
  ignored test). No keystore key identifies the remote.
- **rtl_433 oracle**, same pulses through both: rtl_433's own OOK pulses of the 4 recordings (dumped with
  `-Y classic`: with a dumper rtl_433 enables its FM demodulator, and the 868 MHz minmax detector then calls these
  packages FSK and runs no OOK decoder) — identical, 16 PWM + 16 MC frames; KAT's own pipeline pulses (40 burst
  paths and polarities) — identical; 1000 synthetic packages — identical (PWM 42, MC 88 frames). The fuzz found a
  bit-buffer inversion bug in the first port (a whole last byte was re-inverted), fixed.
- **IMPORTS**: no decode. `FIAT/Fiat Bravo 2010` (ProtoPirate "Fiat V0") holds HCS362-MC-shaped frames (the second
  copy of each press passes the 2-bit CRC in 11 of 11 presses) but with a 16-pulse preamble: rtl_433 rejects them
  (33-bit preamble row), and so does KAT, since a decoder reset by another decoder's report waits for silence
  instead of slicing the rest of a package.
