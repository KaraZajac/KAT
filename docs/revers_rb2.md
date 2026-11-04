---
layout: default
---

# Revers RB2 Protocol

**Rust module:** `src/protocols/revers_rb2.rs`
**Reference:** `REFERENCES/Flipper-ARF/applications/system/subghz_garage/protocols/revers_rb2.c` (garage app copy,
primary); doctor-firmware and Flipper-ARF `lib/subghz/protocols/revers_rb2.c` decode and transmit identically.

## Overview

Revers RB2 / RB2M gate remote: a static 64-bit Manchester frame that starts with ones (bits 48..55 = 0xFF) and ends
with the 10-bit stop code 0x200. Frames are sent back to back; the 320 µs end LOW of one frame and the first LOW
half bit of the next merge into a ~570 µs LOW.

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| te_short | 250 µs | half bit, ±160 µs (90–410 µs; tested first) |
| te_long | 500 µs | ±160 µs (410–660 µs once short failed) |
| Header LOW | 600 µs | ±160 µs |
| End LOW | 320 µs | after every frame (encoder) |
| Bits | 64 | MSB first, Flipper Manchester convention (1 = LOW then HIGH) |

## Frame Layout (64 bits)

- **bits 56..63** ones (the decoder pre-loads 0xF for the four header half bits it consumed)
- **bits 48..55** 0xFF
- **bits 10..47** serial (the C keeps the low 32 bits: `((data << 16) >> 16) >> 10` into a `uint32_t`)
- **bits 0..9** stop code 0x200

## RF

- **Encoding:** Manchester
- **RF modulation:** AM (OOK), 433.92 MHz
- **Encryption:** none (fixed code)

## Decoder Steps

1. **Reset** — a LOW within 160 µs of 600 µs: clear the register, reset the Manchester decoder.
2. **Header** — count four level alternations of short pulses (the last counted level is kept in `te_last`, which
   `reset` does not clear — ported as is); then the register starts as 0xF (4 bits).
3. **DecoderData** — every short/long pulse is a Manchester event (`manchester_advance`); another duration returns
   to Reset. A 65th bit clears the register; at exactly 64 bits the frame is reported when bits 48..55 are 0xFF and
   bits 0..9 are 0x200.

Because the register is cleared on the 65th bit rather than sliding, a decoder that has not just been reset needs
several frames of a held button to line up with a frame start — the C decodes 13 of the ~63 frames of the
unit-test capture, KAT the same 13.

## Encoder

`subghz_protocol_encoder_revers_rb2_get_upload`: the 64 bits through Flipper's Manchester encoder, the closing half
bit kept only when it is HIGH, then LOW 320 µs; sent 3 times. Static: no counter, no button. Like the C decoder, a
fresh decoder does not lock onto the three copies alone (the reference's own encoder round trip fails the same way);
held longer (more copies), the frames decode.

## Key file

`Protocol: Revers_RB2`, `Bit: 64`, `Key`.

## Validation

- Oracle parity (doctor and garage builds): `revers_rb2_raw.sub` gives 13 frames with identical Key and serial; no
  decode on any other unit-test file, IMPORTS or the UberGuidoZ set.
- Encoder: the unit-test key and two more keys pulse-exact against doctor-firmware and the garage build (349 merged
  pulses for the unit-test key).
- Unit tests: key-file load → encode → (held) decode, upload layout and copy merging, preamble / stop-code gates,
  `te_last` surviving `reset`, the unit-test capture through the decoder and the registry.
