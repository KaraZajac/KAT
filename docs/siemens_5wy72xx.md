---
layout: default
---

# Siemens 5WY72XX (rtl_433 port, receive only)

**Rust module:** `src/protocols/siemens_5wy72xx.rs` (rtl_433 receive chain: `src/protocols/rtl433/`)
**Reference:** rtl_433 `src/devices/siemens_5wy72xx.c` ("Siemens 5WY72XX Car Remote (-f 315.1M)", model
`Siemens-5WY72XX`), slicer `pulse_slicer_manchester_zerobit`

## Overview

Siemens 5WY72XX (FCC ID M3N5WY72XX), the OEM remote of DaimlerChrysler's SKREEK CS and RS platforms, 315 MHz: a
32-bit ID, button flags, an unencrypted 16-bit sequence and a 32-bit encrypted part, closed by an XOR byte. No
Flipper reference has it.

## Timing / modulation

| Parameter   | Value                          | Notes |
|-------------|--------------------------------|-------|
| Modulation  | `OOK_PULSE_MANCHESTER_ZEROBIT` | rising edge = 0, falling edge = 1, hard-coded leading 0 |
| short_width | 220 µs                         | half bit |
| reset_limit | 10 000 µs                      | a longer gap ends the message |
| tolerance   | 0                              | the slicer rejects no pulse |

## Frame Layout (96-bit payload after the first `0x6001` in the row; row ≥ 113 bits)

| Bytes | Field |
|-------|-------|
| b0–b3 | ID, little endian (rtl_433 prints b3 b2 b1 b0) |
| b4    | button flags: 0x01 Lock, 0x02 Unlock, 0x04 Trunk, 0x08 Panic, 0x10 Left Door, 0x20 Right Door |
| b5–b6 | 16-bit sequence (big endian) |
| b7–b10 | encrypted part (rtl_433 prints b10 b9 b8 b7) |
| b11   | XOR of b0–b10 (the XOR of all 12 bytes is 0) |

## Decoder Steps

1. rtl_433's Manchester zerobit slicer over KAT's AM pairs; a gap over 10 ms ends the message (KAT reports there).
2. `decode_fn`: one row of at least 113 bits; search `0x6001` (`bitbuffer_search`; when absent, the bits past the
   row are read — zeros — and the sum check fails), read 96 bits after it.
3. Checks: the 12 bytes do not sum to 0 or 12 × 0xFF (`DECODE_FAIL_SANITY`), XOR = 0 (`DECODE_FAIL_MIC`).

## Field mapping

| KAT | rtl_433 |
|-----|---------|
| `serial` | `id` (b3 b2 b1 b0) |
| `button` | `button_code` (b4); native names = `button_str` (flags joined with "; ", "" when none) |
| `counter` | `sequence` |
| `data` / `extra` | payload bytes b0–b7 / b8–b11, `data_count_bit` 96; rtl_433's `encrypted` = b10 b9 b8 b7 |
| `crc_valid` | true (XOR check) |

## Encoder

Receive only, as rtl_433 (the encrypted part cannot be regenerated).

## Validation

- rtl_433_tests `siemens/5WY72XX/01` (6 recordings, 315.1 MHz, 250 kS/s, carrier ~110 kHz off the centre):
  `kat --decode` gives rtl_433's 18 events with the same fields and counts (ID 8D221B7A, sequences 977–984,
  including g005's two presses).
- rtl_433's own pulse packages through KAT's decoder: the expected `.json` files exactly; KAT's pulses through the C
  reference and through KAT: the same events.
- Fuzz against the C reference: 2 × 20 000 trains, identical output, 24 697 events.
- No decode on any IMPORTS capture, the Flipper unit-test captures or another device's rtl_433_tests recording.
