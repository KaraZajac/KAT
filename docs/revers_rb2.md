---
layout: default
---

# Revers RB2 Protocol

**Rust module:** `src/protocols/revers_rb2.rs`
**Reference:** `REFERENCES/Flipper-ARF/applications/system/subghz_garage/protocols/revers_rb2.c` (garage app copy,
primary); doctor-firmware and Flipper-ARF `lib/subghz/protocols/revers_rb2.c` decode and transmit identically.
KAT's second reader (the stream reader, below) follows rtl_433's `conf/Revers-RB2.conf`.

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

**What the C misses.** The remote sends its message over and over without a gap: 32 ones, a 22-bit id, the stop
code. The C's header is any LOW of 440–760 µs with four shorts behind it, and that is every 0 with three ones
behind it, in the id as well; whether the 4 bits it then assumes are the right number depends on `te_last`. A
start in the wrong place is cured only by the 65th bit, which moves the window by one bit per message. So the C
reads 13 of the about 60 messages of Flipper's unit-test capture, none of the 3 copies its own encoder sends,
and none of the messages of rtl_433's two recordings.

**KAT's stream reader (not in the C).** Beside the C's path, every short or long pulse goes through a Manchester
decoder of its own, with no header, and a message is reported when the last 74 bits are the stop code of the
message before, 32 ones, 22 bits and the stop code. That is rtl_433's rule; it needs the end of one message and
the whole of the next, so the first message of a transmission is not reported. The stop code before the ones is
what keeps this from a frame of another Manchester remote of these timings behind a long preamble (Ford V3's
European frames): a preamble holds no stop code. A pulse that is neither short nor long, and a pulse the Manchester
decoder has no place for, start the count again. `reset()` keeps the Manchester state and the last ten bits, since
the pulses go on: the registry resets every decoder after a message, and the next message is checked against the
stop code of that one. What the C's path reports is reported as before (a message whose bits 32..47 are not ones
is read by that path alone).

The same test serves [Ford V3](ford_v3.md): its European path takes a run of 30 shorts and the 104 bits behind it
without a check, and drops the frame when those bits are this stream.

## Encoder

`subghz_protocol_encoder_revers_rb2_get_upload`: the 64 bits through Flipper's Manchester encoder, the closing half
bit kept only when it is HIGH, then LOW 320 µs; sent 3 times. Static: no counter, no button. The C's decoder does not
lock onto the three copies alone (the reference's own encoder round trip fails that way); KAT's stream reader
reports the second and the third. Held for 70 copies, 69 are reported.

## Key file

`Protocol: Revers_RB2`, `Bit: 64`, `Key`.

## Validation

- The C's path, oracle parity (doctor and garage builds): `revers_rb2_raw.sub` gives 13 frames with identical Key
  and serial; no decode on any other unit-test file, IMPORTS or the UberGuidoZ set.
- With the stream reader `revers_rb2_raw.sub` gives 51 frames, all the one key `FFFFFFFF39F90A00`, and rtl_433's
  recordings `revers/rb2/01/revers_rb2_012345_433.92M_250k.cu8` and `revers_rb2_3abcde_433.92M_250k.cu8` give 7
  of their 8 messages each (`FFFFFFFF048D1600`, `FFFFFFFFEAF37A00`: the ids of rtl_433's JSON; the first message
  has no stop code before it). doctor-firmware's C reads none of those 16 messages and names both recordings
  Ford V3. On 262 real RAW files (IMPORTS, the Flipper unit-test folders, rtl_433's recordings) no other file
  decodes as Revers_RB2.
- Encoder: the unit-test key and two more keys pulse-exact against doctor-firmware and the garage build (349 merged
  pulses for the unit-test key).
- Unit tests: key-file load → encode → decode (3 copies and held), upload layout and copy merging, preamble /
  stop-code gates, `te_last` surviving `reset`, the stream read wherever a capture starts and across broken
  pulses, the unit-test capture through the decoder and the registry; `rtl433_recordings_decode` (ignored, needs
  the recordings) checks the two recordings and that neither is Ford V3.
