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
4. KAT's checks (not in the C): the `0x6001` starts within the row's first 24 bits, and the 96 payload bits end
   inside the row. Every recording is a 113-bit row with `0x6001` at bit 0 (the preamble, the payload, one bit
   more). Without them, a long row of noise passes now and then: KAT's receiver hands the decoders rows of
   200–1000 bits from bursts barely above the noise (SNR 2–4 dB), the C finds a chance `0x6001` deep inside one
   and the 8-bit XOR holds one time in 256. One tester's KAT Mobile session had seven such decodes, with the
   `0x6001` 92 or more bits into the row, buttons no remote sends (0x00, 0x41) and payloads of nearly all zeros;
   with the checks there are none, and every press of the recordings below still reads (one of their 879 frames,
   a repeat with 46 bits of noise ahead of its preamble in its row, is dropped; its press reads from its other
   repeats).

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
- A tester's six recordings of one remote (`IMPORTS/chrysler/more/fca_siemens_5wy72xx_*.sub`, 314.925 MHz),
  about 50 presses of each button: 301 presses, every one read, ID 61060F78, the check right in each. The
  sequence goes up by one per press whatever the button: Panic (button 0x08) 0x2164 to 0x2195, Trunk (0x04) to
  0x21C7, Lock (0x01) to 0x21F8, Unlock (0x02) to 0x222B, and two more codes, 0x10 (0x223D to 0x226E) and
  0x20 (0x2281 to 0x22B3): the flags rtl_433 names Left Door and Right Door. The remote is a four-button
  key head: liftgate (marked x2), unlock, lock and panic. Its board is the one in the internal photos
  of the FCC filing of `M3N5WY72XX` (September 2004): the same outline, a Philips PCF7941 in the same place, the
  coil, the star-shaped battery contact, the print `CR2032`. The filing's unit is a Dodge key with six buttons
  (lock, unlock, the two sliding doors, the liftgate, panic) on a board with six pads; so 0x10 and 0x20 are the
  two pads this key's shell has no button over. Sellers list the four-button key of that FCC ID for the
  Chrysler Pacifica of 2004 to 2008 and the Jeep Liberty of 2005 to 2007, and the six-button one for the Dodge
  Caravan and Grand Caravan and the Chrysler Town & Country of 2004 to 2007.
- No decode on any other IMPORTS capture, the Flipper unit-test captures or another device's rtl_433_tests
  recording.
