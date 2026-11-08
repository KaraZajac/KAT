---
layout: default
---

# Mazda V1 Protocol

**Rust module:** `src/protocols/mazda_v1.rs`
**Reference:** `FoxFW lib/subghz/protocols/mazda_v1.c` (no other reference has it)

## Overview

Mazda rolling-code frames of 71 bits at 505/1010 µs: the sync byte `0xEC`, a button byte, a 6-bit per-remote
constant, a 5-bit cleartext counter and encrypted bits. FoxFW gates them structurally only (no CRC), scanning the
bit stream for the sync at every position and at both polarities. Decode-only.

**Relation to Mazda V2.** The same family and air format as KAT's `Mazda V2` (500/1000 µs, sync `0xEC`, 71 bits,
and V2's "alt" buttons `0x06 / 0x0A / 0x0E`), gated differently: V1 needs no long preamble and also takes button
`0x26`, which V2's six button codes reject, while V2 takes `0x04 / 0x08 / 0x10`, which V1 rejects. V1 is
registered after V2 and reports only at the end of its burst (see below), so V2 keeps every frame both read,
including its transmit support. It is not Mazda V0 / Mazda Siemens (250/500 µs pair decoders).

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

**KAT deviation: when it reports.** FoxFW reports on the last bit of a frame; Mazda V2 reports at the gap that
ends the burst, so a V1 that fired on the last bit would take V2's alt-button frames (and their transmit
support). KAT holds the first accepted frame of the burst and reports it on the pulse that ends the burst, where
V2 (earlier in the registry) completes first. One report per burst: the second copy of a frame sent twice is not
listed (the FoxFW oracle, which resets after each hit like KAT's registry, lists both). A recording that ends
inside the burst (cut while the button was held) still gives the held frame, reported when the input ends, as
FoxFW has reported it on its last bit.

## RF

- **Encoding:** Manchester
- **RF modulation:** AM
- **Encryption:** rolling code
- **Frequencies:** 315 MHz, 433.92 MHz
- **Check:** none (structural gates); the CRC column shows "-"

No capture of a real Mazda V1 fob exists in KAT's test files; the decoder is verified against FoxFW's C decoder on
made-up frames.
