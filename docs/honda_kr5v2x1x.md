---
layout: default
---

# Honda-KR5V2X1X Protocol

**Rust module:** `src/protocols/honda_kr5v2x1x.rs` (rtl_433 receive chain: `src/protocols/rtl433/`)
**Reference:** `REFERENCES/rtl_433/src/devices/continental_car_remote.c`, `honda_keyfob_decode`
(r_device "Honda Keyfob KR5V2X/1X (-f 433.6M -s 1024k)", decoder 327, enabled by default)

## Overview

Honda keyfobs FCC ID KR5V2X (313.55 / 314.15 MHz) and KR5V1X (433.66 / 434.18 MHz), FSK. rtl_433 added this
decoder (#3578, from PR #2767) as an alternate decoding of the same physical remotes as
[Continental-KR5V2X](continental_kr5v2x.md); the two disagree on bit layout and checksum. Receive only, as
rtl_433.

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| Modulation | FSK, `FSK_PULSE_MANCHESTER_ZEROBIT` | upper tone = HIGH |
| Short (half-bit) | 60 µs | data edge threshold 90 µs |
| Long | 120 µs | (not used by this slicer) |
| Reset limit | 1500 µs | ends the message |
| Tolerance | none | |

## Frame Layout

The message must be one row of 150..=184 bits. The decoder finds the 24-bit manufacturer code `EC 0F 62` and
reads 120 bits from its last byte on:

| byte | field |
|------|-------|
| 0 | `0x62` (the code's last byte) |
| 1 | packet index (a press sends the packet twice, `0x08` then `0x0A`) |
| 2..5 | device ID (32 bits) |
| 6 | event: 0x21 Lock, 0x22 Unlock, 0x24 Trunk, 0x27 Emergency, 0x2D RemoteStart |
| 7..9 | counter (24 bits) |
| 10..13 | rolling code (32 bits) |
| 14 | CRC-8 of bytes 0..13, OpenSafety polynomial 0x2F, init 0 |

## Decoder Steps

1. Slice the stream into messages (zero-bit Manchester, end at a gap > 1500 µs).
2. Exactly one row, 150..=184 bits; `EC 0F 62` found with 136 bits from its start in the row.
3. CRC-8 (`DECODE_FAIL_MIC`). Frames failing a check are dropped.

## Field mapping

| KAT | rtl_433 |
|-----|---------|
| `serial` | `id` (`%08x`) |
| `button` | event byte (`event` prints its name, "?" for others) |
| `counter` | low 16 bits of `counter` (24 bits) |
| `crc_valid` = true | `mic: CRC` |
| `data` = bytes 0..7, `extra` = bytes 8..14, 120 bits | `code` = `(extra >> 8) & 0xFFFFFFFF`; the packet index (byte 1) is not output by rtl_433 |

## Encoder

Receive only, as rtl_433 (no encoder).

## Validation

- **Recordings:** none. rtl_433_tests has no folder for this decoder, and rtl_433 (`-R 327`) decodes none of the
  100 recordings in REFERENCES — the Continental recordings (same FCC ID) included; neither does KAT
  (`rtl433_recordings_as_rtl433`).
- **Oracle parity** (patched rtl_433 reading FSK pulse files): 4000 fuzzed packages, identical decodes (677 frames);
  identical slicer rows for all 5298 messages.
- **Synthetic IQ**: frames built from the C at 1024 and 250 kS/s decode identically in rtl_433 and KAT.
- No decode on any IMPORTS capture, Flipper unit-test capture or rtl_433_tests recording (decoder run alone, both
  polarities).
