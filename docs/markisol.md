---
layout: default
---

# Markisol Protocol (not registered)

**Rust module:** `src/protocols/markisol.rs` (rtl_433 receive chain: `src/protocols/rtl433/`)
**Reference:** rtl_433 `src/devices/markisol.c` (GPL-2.0; model `Markisol`, r_device "Markisol, E-Motion, BOFU,
Rollerhouse, BF-30x, BF-415 curtain remote", `-R 191`)

## Overview

Curtain / blind remotes sold as Markisol, E-Motion, BOFU, Rollerhouse, BF-30x and BF-415: a 16-bit address,
channel, zone and control code with an additive checksum. **rtl_433 ships this decoder disabled** (`.disabled = 1`,
"excessive false positives, see issue #3611"; it runs only with `-R 191`). KAT mirrors that as it mirrors the
references' disabled protocols: the module, tests and docs are kept, the registry does not list it. Enabling it is
one line (`Box::new(markisol::MarkisolDecoder::new())` with the other rtl_433 shutter ports in the registry)
plus its `INFO` in `info.rs`.

## Timing / modulation

OOK_PULSE_PWM, short 368 µs, long 704 µs, sync 5628 µs, gap and reset limit 2000 µs, no tolerance (the slicer then
splits at the midpoints: under 537 µs = 1, up to 3166 µs = 0, longer = sync). A frame: HIGH 4886 (sync), LOW 2470
(ends the previous message), HIGH 1647 (read as a 0 bit), LOW 315, 40 bits (0 = HIGH 670 + LOW 320, 1 = HIGH 348 +
LOW 642). A frame is decoded at the gap after the next frame's sync, or at its own final gap.

## Frame Layout (`markisol_decode`)

The first row of 41 or 42 bits, shifted left by the spurious first bit; each byte bit-reversed and inverted:

| Byte | Field |
|---|---|
| 0–1 | address |
| 2 | channel (low nibble), zone bit 0 (0x20), control bits 0, 2, 3 (0xD0) |
| 3 | control bit 1 (0x10), zone bit 1 (0x80); 0x6F unknown (0x01 on the author's remotes) |
| 4 | checksum: all five bytes sum to 1 (mod 256) |

## Field mapping

| KAT | rtl_433 | Notes |
|---|---|---|
| `serial` | `id` | address, 16 bits |
| `button` | `control` | 4-bit control code; labels: rtl_433's strings ("Down (1)", "Stop (5)", "Up (12)", …) |
| – | `channel`, `zone` | `data` byte 2 low nibble; zone from bytes 2 and 3 |
| `data` | | the five decoded bytes (40 bits) |
| `crc_valid` | `mic: CHECKSUM` | always true for a reported frame |

## Encoder

None — receive only, as rtl_433.

## Validation

- **rtl_433 oracle** (`-R 191`) on 1000 synthetic packages (every control code, channel and zone, 1–3 repeats,
  trailing pulses, jitter, corrupted checksums): identical (1228 frames).
- Unit tests: every control code with several channels / zones and repeats; checksum failures and short rows.
