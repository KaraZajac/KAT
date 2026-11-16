---
layout: default
---

# Mazda V1 Protocol

**Rust module:** `src/protocols/mazda_v1.rs`
**Reference:** `FoxFW lib/subghz/protocols/mazda_v1.c` (no other reference has it)

## Overview

Mazda rolling-code frames of 71 bits at 505/1010 µs: the sync byte `0xEC`, a button byte, a 6-bit per-remote
constant, a 5-bit cleartext counter and encrypted bits. FoxFW gates them structurally only (no CRC), scanning the
bit stream for the sync at every position and at both polarities. FoxFW only decodes; KAT can send a captured
frame again (below).

**Relation to Mazda V2.** The same family and air format as KAT's `Mazda V2` (500/1000 µs, sync `0xEC`, 71 bits,
and V2's "alt" buttons `0x06 / 0x0A / 0x0E`), gated differently: V1 needs no long preamble and also takes button
`0x26`, which V2's six button codes reject, while V2 takes `0x04 / 0x08 / 0x10`, which V1 rejects. It is not Mazda
V0 / Mazda Siemens (250/500 µs pair decoders).

**A frame both know is listed as Mazda V2.** A frame with button `0x06 / 0x0A / 0x0E` is the frame Mazda V2 reads
behind its preamble, and Mazda V2 can send it again. This reader reports such a frame with Mazda V2's fields and
under its name; "Mazda V1" is left with button `0x26`. See [mazda_v2.md](mazda_v2.md), "Frames without a
preamble".

## Timing

| Parameter | Value   | Notes                                  |
|-----------|---------|----------------------------------------|
| Short     | 505 µs  | 305..=705 µs (delta 200, `<=`)          |
| Long      | 1010 µs | 810..=1210 µs                           |
| Buffer    | 400 bits | scanned at 380 bits                    |

Line code: a half-bit-toggle Manchester. The first SHORT after reset starts collecting; a LONG is one bit equal
to its level; of two SHORTs the second is one bit equal to its level; any other duration ends the burst.

## Frame Layout (71 bits)

| Bits    | Field                                                                  |
|---------|------------------------------------------------------------------------|
| 0..7    | sync `0xEC`                                                            |
| 8..15   | button; `btn & 0x7F` must be `0x06`, `0x0A`, `0x0E` or `0x26`          |
| 16..23  | a byte whose upper six bits are the serial, `(byte >> 2) & 0x3F`      |
| 49..53  | counter, 5 bits, top bit = bit 53 XOR bit 49                           |
| 70      | must be 1                                                              |

Markers: either the "v1 marker" (bits 18..21 = `1,0,0,0` and bit 37 = 0) or the "v3 marker" (bits 18..21 are
anything but `1,0,0,0`). After an accepted frame, windows starting less than 142 bits later are skipped. An
accepted polarity-1 window is inverted in place.

**KAT packing:** `data` = the first 64 bits, `extra` = the last 7 bits, `data_count_bit` = 71, `serial`,
`button` (the whole byte) and `counter` as above.

**When it reports.** As FoxFW: on the last bit of a frame. KAT's registry resets every decoder after a decode,
so the second copy of a frame sent twice is collected anew and reported too (the FoxFW oracle resets the same way
and lists both; on a Flipper the C skips windows that start less than 142 bits after an accepted one, and shows
one). Until KAT 1.13 this decoder held the first frame of a burst back until the burst ended, to let Mazda V2
report first, and listed one frame per burst: the frames behind it were never listed.

## Encoder (KAT's)

FoxFW has none. KAT sends the captured frame again, as it is, in Mazda V2's upload: a 180-bit preamble, the sync
and the 63 bits behind it, three copies ([mazda_v2.md](mazda_v2.md), "Encoder"). The fob itself sends one
preamble and the frame twice back to back. The rolling part of a frame with button `0x26` is not understood
(Mazda V2's tables cover its six buttons), so there is no next code to build: Lock / Unlock / Trunk / Panic all
send the same frame, and a receiver that has seen it may refuse it.

## RF

- **Encoding:** Manchester
- **RF modulation:** AM in FoxFW's flags (its receiver does not gate by modulation); the fobs in IMPORTS are
  2-FSK (preset FM476), so KAT reads it on both
- **Encryption:** rolling code
- **Frequencies:** 315 MHz, 433.92 MHz
- **Check:** none (structural gates); the CRC column shows "-"

## Validation

- Made-up frames: the same fields as FoxFW's C decoder (sync, button, end-marker and marker gates included).
- Captures (`IMPORTS/Mazda/`, 2026-11): button `0x26` in `mazda_key3_10x_trunk.sub` and `more/mazda_key5_10x_trunk.sub`
  (20 frames each: 10 presses, each frame twice), in `more/mazda_visteon_6xl_5xu_5xa.sub` (10), in
  `more/mazda_key0_524x_btn5.sub` (1047) and in `more/mazda_key0_1665x_btn5.sub` (216): 1313 frames. Over the 28
  recordings of this frame format KAT lists 5730 frames (4417 of them under Mazda V2's name), FoxFW's C 5679, all
  as Mazda V1; the counts are equal in 26 recordings ([mazda_v2.md](mazda_v2.md), "Validation", has the other
  two).
- The encoder: its three copies decode again with every bit of the captured frame.
