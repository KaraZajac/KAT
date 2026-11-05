---
layout: default
---

# Continental-KR5V2X Protocol

**Rust module:** `src/protocols/continental_kr5v2x.rs` (rtl_433 receive chain: `src/protocols/rtl433/`)
**Reference:** `REFERENCES/rtl_433/src/devices/continental_car_remote.c`, `continental_car_remote_decode`
(r_device "Continental KR5V2X Car Remote (-f 313.8M -s 1024k)", decoder 316, enabled by default)

## Overview

Continental 72147-SNA-A01 (FCC ID KR5V2X), an OEM remote for Honda, 313.8 MHz FSK. rtl_433 describes it as a
rolling code with an unencrypted sequence number. A press sends a long wake-up (`0x87` repeated) with the frame,
then the frame again ~1.6 ms later; Trunk and Panic long presses repeat every ~95 ms. Receive only, as rtl_433.

rtl_433 also has an alternate decoding of the same FCC ID, [Honda-KR5V2X1X](honda_kr5v2x1x.md) (60 µs, other
layout and CRC); it decodes none of these recordings.

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| Modulation | FSK, `FSK_PULSE_MANCHESTER_ZEROBIT` | upper tone = HIGH |
| Short (half-bit) | 100 µs | an edge > 150 µs after the last data edge is a data edge |
| Long | 200 µs | (not used by this slicer) |
| Reset limit | 1500 µs | a lower-tone stretch longer than this ends the message |
| Tolerance | none | no row splits |

The zero-bit slicer (rtl_433 `pulse_slicer_manchester_zerobit`): every message starts with a hard-coded `0`; a
falling data edge adds `1`, a rising one `0`. See `src/protocols/rtl433/mod.rs` for how KAT's level/duration stream maps
to rtl_433's packages.

## Frame Layout

The decoder searches row 0 for the last 20 bits of the preamble, `0xF0F06`, and reads the 112 bits after it:

| bytes | field |
|-------|-------|
| 0..3 | remote ID (32 bits) |
| 4 | unknown (0x00 in the first frame of a press, 0x10 in the repeat on the recordings) |
| 5 | button (high nibble), unknown (low nibble) |
| 6..8 | sequence (24 bits) |
| 9..12 | encrypted code (32 bits) |
| 13 | checksum: the 14 bytes XOR to 0 |

Buttons: 1 Lock, 3 Unlock, 9 Trunk Long Press, 0xA Trunk/Panic Short Press, 0xB Panic Long Press.

## Decoder Steps

1. Slice the stream into messages (zero-bit Manchester, end at a gap > 1500 µs).
2. Row 0 must have ≥ 132 bits; find `0xF0F06`; ≥ 112 bits must follow it.
3. Sanity (rtl_433 `DECODE_FAIL_SANITY`): ID, button and sequence non-zero; ID and encrypted ≠ `0x0FFFFFFF`
   (rtl_433 compares the 32-bit values with seven F's; kept as is); sequence ≠ `0xFFFFFF`.
4. XOR checksum (`DECODE_FAIL_MIC`). Frames failing any check are dropped.

## Field mapping

| KAT | rtl_433 |
|-----|---------|
| `serial` | `id` (`%08X`) |
| `button` | `button_code` |
| `counter` | low 16 bits of `sequence` (24 bits) |
| `crc_valid` = true | `mic: CHECKSUM` |
| `data` = bytes 0..7, `extra` = bytes 8..13, 112 bits | `sequence` = `(data & 0xFFFF) << 8 \| extra >> 40`, `encrypted` = `(extra >> 8) & 0xFFFFFFFF` |

## Encoder

Receive only, as rtl_433 (no encoder): the app offers Replay of the recorded pulses and export.

## Validation

- **Recordings** (`rtl_433_tests/tests/continental/72147-SNA-A01`, 7 files at 1024 kS/s): all decode through KAT's
  IQ pipeline with rtl_433's fields (ID, encrypted, sequence, button) — 22 frames in KAT, 21 in rtl_433: in
  `panic_long_press` rtl_433's demodulator keeps only 127 bits of one package's first message (it needs 132),
  KAT's keeps 150. Before this port no KAT decoder read them at their real rate (KAT 1.7's "Kia V5" decodes came
  from `--rate 250k` stretching the 1024 kS/s files 4×).
- **Oracle parity** (rtl_433 patched to run its FSK decoders on pulse files: `src/rtl_433.c` treats a file with a `;freq2` header
  line as FSK): 8000 fuzzed packages (frames from
  the C with random fields, bit flips, jitter, glitches, truncations, extra gaps, random trains) give identical
  decodes (643 frames); the slicer gives identical rows for every message (10 658 messages, with and without a
  tolerance).
- **Synthetic IQ**: frames built from the C, at 1024 and 250 kS/s, decode identically in rtl_433 and KAT.
- No decode on any other rtl_433_tests recording, IMPORTS capture or Flipper unit-test capture (decoder run alone,
  both polarities).
