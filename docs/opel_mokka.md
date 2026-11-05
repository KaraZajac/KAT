---
layout: default
---

# Opel-Mokka Protocol

**Rust module:** `src/protocols/opel_mokka.rs` (rtl_433 receive chain: `src/protocols/rtl433/`)
**Reference:** `REFERENCES/rtl_433/src/devices/opel_mokka.c`, `opel_mokka_decode`
(r_device "Opel Mokka Car Key", decoder 333, enabled by default)

## Overview

Opel Mokka car key, FSK; rtl_433 presumes a HITAG AES (4A, NCF29A1M) transponder, so other brands using it may
decode too. It reads the key ID and the event type; the 64-bit payload is encrypted. Not KAT's
[Opel V1](opel_v1.md) (Flipper-ARF, AM, 93 bits). Receive only, as rtl_433.

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| Modulation | FSK, `FSK_PULSE_MANCHESTER_ZEROBIT` | upper tone = HIGH |
| Short (half-bit) | 100 µs | data edge threshold 150 µs (`long_width` is also 100 and unused) |
| Reset limit | 1000 µs | ends the message |
| Tolerance | none | |

## Frame Layout

The message must be exactly 268 bits: the slicer's hard-coded `0`, 88 zeros, the packet, 15 zeros, the packet
again. Packet: start bit 1, key ID (11 bits), event type (5 bits), payload (64 bits), end bit 1.

| row bits | field |
|----------|-------|
| 0..=88 | zeros (bytes 0..10 checked) |
| 89 | start bit |
| 90..=100 | key ID |
| 101..=105 | event type: 26 for both lock and unlock, 3 for a zero-filled packet sent periodically (proximity?) |
| 106..=169 | encrypted payload |
| 170 | end bit |
| 186..=267 | the packet again (key ID at 187..=197) |

rtl_433's `code` is the 64 bits from row bit 107, i.e. payload bits 1..63 plus the end bit (the C adds 17 to the ID
position, one more than ID + event). KAT reports the same bits.

## Decoder Steps

1. Slice the stream into messages (zero-bit Manchester, end at a gap > 1000 µs).
2. A row of exactly 268 bits, bytes 0..10 zero, both key IDs equal, key ID ≠ 0. No checksum.

## Field mapping

| KAT | rtl_433 |
|-----|---------|
| `serial` | `id` (11 bits) |
| `button` | `event` (5 bits) |
| `counter` = none | |
| `crc_valid` = false | no `mic` (only the repeated key ID) |
| `data`/`extra` = the first packet from its start bit (82 bits: 64 + 18) | `code` = low 64 bits of `data << 18 \| extra` |

## Encoder

Receive only, as rtl_433 (no encoder).

## Validation

- **Recordings:** none (no rtl_433_tests folder). On the FSK car-remote recordings that exist (Nidec, Honda)
  neither rtl_433 nor KAT reports an Opel-Mokka frame. The tests use frames built from the C and its documented
  example packet (key ID 1074, event 26).
- **Oracle parity** (patched rtl_433 reading FSK pulse files): 4000 fuzzed packages, identical decodes (538 frames);
  identical slicer rows for all 5297 messages.
- **Synthetic IQ**: frames whose carrier starts in the upper tone decode in KAT at 1024 and 250 kS/s, and in
  rtl_433 at 250 kS/s (at 1024 kS/s rtl_433's demodulator reads one of three as 269 bits, an extra edge where the
  synthetic carrier switches off).
- **Start-of-package caveat (unverified, no capture):** the exact length makes the decode depend on how the
  package starts. rtl_433 counts a lower-tone lead-in longer than 150 µs as one more `0` (its FSK package starts with
  `pulse[0] = 0`); KAT's FM path merges that lead-in into the idle LOW. With a 300 µs or 1 ms lead-in, rtl_433
  reads 269 bits (no decode) and KAT 268 (decode); conversely, a real fob rtl_433 decodes after such a lead-in
  would read 267 bits in KAT.
- No decode on any IMPORTS capture, Flipper unit-test capture or rtl_433_tests recording (decoder run alone, both
  polarities).
