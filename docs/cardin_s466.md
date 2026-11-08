---
layout: default
---

# Cardin S466-TX2 (rtl_433 `Cardin-S466`)

**Rust module:** `src/protocols/cardin_s466.rs`
**Reference:** rtl_433 `src/devices/cardin.c` (r_device "Cardin S466-TX2", model `Cardin-S466`, decoder 17,
enabled by default; GPL-2.0). No Flipper firmware has it; RogueMaster's Cardin S508 ([cardin_s508.md](cardin_s508.md))
is a different, 868 MHz rolling-code remote.

## Overview

A fixed-code garage-door remote on **27.195 MHz** (rtl_433: "Remember to set the correct freq with -f 27.195M"),
which the HackRF receives. Nine tri-state DIP switches set the code; a 2-bit field says which button was pressed.
Receive only, as in rtl_433: KAT shows the fields, and the capture can be replayed as recorded and exported.

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| Modulation | OOK_PULSE_PWM | rtl_433's slicer, no tolerance: split at the midpoints |
| Short pulse | 730 µs | a pulse under 1065 µs is a 1 |
| Long pulse | 1400 µs | up to 3775 µs is a 0 |
| Sync | 6150 µs | longer pulses start a new row |
| Gap limit | 1600 µs | a longer gap ends the row |
| Reset limit | 32000 µs | a longer gap ends the message |
| Bits | 24 | row 0, exactly |

rtl_433: "Similar to an EV1527 / SC2260, but there is a 6152 us sync pulse first, then 24 bit of 732 us / 1412 us
leading-gap PWM."

## Frame

| DIP | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
|---|---|---|---|---|---|---|---|---|---|
| first bit | b0 0x08 | b0 0x10 | b0 0x20 | b0 0x40 | b0 0x80 | b2 0x80 | b0 0x01 | b0 0x02 | b0 0x04 |
| second bit | b1 0x08 | b1 0x10 | b1 0x20 | b1 0x40 | b1 0x80 | b2 0x40 | b1 0x01 | b1 0x02 | b1 0x04 |

A switch reads `-` when its first bit is 0, `o` for 1 then 0, `+` for 1 then 1; the fourth combination (0 then 1)
is refused. The low six bits of byte 2 are the button code: 0x03, 0x06, 0x09 or 0x0C, else the frame is refused.
There is no checksum.

## Field mapping

| KAT field | rtl_433 |
|---|---|
| `serial` | the 18 DIP bits, `b0 << 10 \| b1 << 2 \| b2 >> 6` (rtl_433 prints them as `dipswitch`, e.g. `+o-+o-+o-`) |
| `button` | `rbutton` index ((byte 2 & 0x0F) / 3 − 1): 0 `11R`, 1 `10R`, 2 `01R`, 3 `00L?` |
| `data` | the 24 bits |

Several frames of one press inside one message (gaps under 32 ms) give one decode, as in rtl_433, which reads
row 0 of each message.

## Registry

In the rtl_433 block (last in the registry), after the Microchip KeeLoq chips, so a frame a Flipper reference
decodes keeps that decoder.

## Validation

400 synthetic messages (valid and refused DIP pairs and button codes, 1–4 repeats, jitter up to ±250 µs, dropped,
added and stretched pulses) decoded by rtl_433 itself (`-R 17 -s 1M`, KAT's units) and by the port through
`src/protocols/rtl433/harness.rs` (`KAT_RTL433_REPLAY`): the same 172 decodes, field for field. No recording
exists (rtl_433_tests has none).
