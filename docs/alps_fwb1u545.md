---
layout: default
---

# Alps FWB1U545 (rtl_433 port, receive only)

**Rust module:** `src/protocols/alps_fwb1u545.rs` (rtl_433 receive chain: `src/protocols/rtl433/`)
**Reference:** rtl_433 `src/devices/alps_fwb1u545.c` ("Alps FWB1U545 Car Remote", model `Alps-FWB1U545`),
slicer `pulse_slicer_manchester_zerobit` (`src/pulse_slicer.c`)

## Overview

Alps Electric FWB1U545 (FCC ID CWTWB1U545), an OEM car remote for Honda: a fixed 32-bit ID, a button code sent
twice and an unencrypted 16-bit sequence number. No Flipper reference has it. KAT decodes it exactly as rtl_433
does; transmitting means replaying the recorded pulses.

## Timing / modulation

| Parameter   | Value                          | Notes |
|-------------|--------------------------------|-------|
| Modulation  | `FSK_PULSE_MANCHESTER_ZEROBIT` | 2-FSK; the higher tone is the pulse (KAT's FM HIGH) |
| short_width | 500 µs                         | half bit; an edge more than 750 µs after the last data edge is a data edge |
| reset_limit | 1500 µs                        | a longer gap ends the message |
| tolerance   | 0                              | the slicer rejects no pulse |
| Deviation   | ±30–37 kHz (recordings)        | KAT's transmit preset F3 (±31.74 kHz) for Replay |

## Frame Layout (76 bits, one row)

| Bits  | Field |
|-------|-------|
| 0–7   | `0x55` preamble byte (bit 0 is the slicer's hard-coded leading 0) |
| 8–39  | 32-bit ID |
| 40–47 | button byte: high nibble = button code |
| 48–55 | copy of the button byte |
| 56–71 | 16-bit sequence |
| 72–75 | unknown |

## Decoder Steps

1. KAT's FM pairs go through rtl_433's Manchester zerobit slicer (rising edge = 0, falling edge = 1, a hard-coded
   0 bit at the start of every message).
2. A gap over 1500 µs ends the message. The 500/500 µs preamble is a message of its own (it ends at the ~2.5 ms
   gap before the frame) and fails the length check.
3. `decode_fn`: exactly one row of exactly 76 bits; byte 0 = `0x55` and byte 6 = byte 5; ID ≠ 0, ≠ FFFFFFFF
   (all `DECODE_FAIL_SANITY`). The event is reported on the gap that ends the frame.

## Field mapping

| KAT | rtl_433 |
|-----|---------|
| `serial` | `id` |
| `button` | `button_code` (byte 5 >> 4); native names = `button_str`: E Lock, C Panic, 5 Panic Held, 1 Unlock, else "?" |
| `counter` | `sequence` |
| `data` / `extra` | the first 64 frame bits / the last 12 (byte 8 + the unknown nibble), `data_count_bit` 76 |
| `crc_valid` | false: rtl_433 reports no integrity check for this remote (no `mic` field; its checks are sanity checks) |

## Encoder

Receive only, as rtl_433 (Replay of the recorded pulses and the RAW exports remain).

## Validation

- rtl_433_tests `alps/fwb1u545/01` (4 recordings, 433.92 MHz, 250 kS/s): `kat --decode` (FM path) gives rtl_433's
  events with the same fields and counts (lock, panic, panic held ×2, unlock).
- KAT's FM pulses match rtl_433's FSK pulses within a few µs (rtl_433 ends the last gap at 3056 µs and adds a 12 µs
  F1 blip; KAT ends the burst with a long LOW — both end the message).
- rtl_433's own pulse packages of the recordings (`rtl_433 -w ook:`) through KAT's decoder reproduce the expected
  `.json` files exactly; KAT's pulses through the C reference (`libr_433.a`) and through KAT give the same events.
- Fuzz against the C reference (slicer + decoder linked from rtl_433's library): 2 × 20 000 synthetic trains
  (valid frames with jitter, flipped / cut bits, retimed, dropped, split pulses, timing edges) — identical output,
  24 667 events.
- No decode on any IMPORTS capture, the Flipper unit-test captures or another device's rtl_433_tests recording.
