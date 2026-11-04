---
layout: default
---

# CAME Protocol

**Rust module:** `src/protocols/came.rs`
**Reference:** Flipper-ARF garage app `applications/system/subghz_garage/protocols/came.c` (primary);
Flipper-ARF `lib/subghz/protocols/came.c` (same decoder), doctor-firmware `lib/subghz/protocols/came.c`

## Overview

CAME's fixed-code gate remotes (TOP-432 etc.): 12 or 24 bits, plus two other formats the same decoder
reads — 18-bit "Airforce" and 25/42-bit "Prastel" frames. The C displays those under their own names;
KAT reports them as `CAME [Airforce]` / `CAME [Prastel]` (the key file still says `Protocol: CAME`).

doctor-firmware differs on the wire: it no longer decodes 42-bit frames (its `prastel.c` owns them) and
reports a frame only when `subghz_protocol_came_payload_ok` passes (no degenerate key; a 12-bit frame
needs a header ≥ 41·te and fewer than two 0x5/0xA nibbles). KAT follows ARF. In the garage app CAME (RX
group g5) is registered before its 42-bit rolling-code "Prastel" protocol (g12) and wins the frames both
decode; in KAT the gate groups' order decides (see the registry comment).

## Timing

| Parameter  | Value              | Notes                                              |
|------------|--------------------|----------------------------------------------------|
| te_short   | 320 µs             | ±150 µs                                            |
| te_long    | 640 µs             | ±150 µs                                            |
| Header     | 17920 ± 9450 µs    | encoder: 76·te (24/42 bits), 47·te (12/18), 36·te (25), 16·te otherwise |
| Start bit  | HIGH 320 µs        |                                                    |
| End LOW    | ≥ 1280 µs          | the next header                                    |

## Frame Layout

MSB first after the start bit: bit 1 = LOW 640 + HIGH 320, bit 0 = LOW 320 + HIGH 640. The frame is
reported at exactly 12, 18, 24, 25 or 42 bits. There are no fields (the C clears serial/button and shows
the key and its mirror image).

## RF

- **Encoding:** PWM
- **RF modulation:** AM (OOK); 315 / 433.92 MHz
- **Encryption:** none (fixed code)

## Decoder Steps

1. **Reset** — header LOW within 17920 ± 9450 µs → FoundStartBit.
2. **FoundStartBit** — LOWs are skipped; a HIGH of 320 ± 150 µs starts the bits, any other HIGH → Reset.
3. **SaveDuration** — a LOW ≥ 1280 µs ends the frame (report if the length matches) and waits for the next
   start bit; a shorter LOW is kept for the pair; a HIGH → Reset.
4. **CheckDuration** — the HIGH completes the pair (0 or 1) or → Reset.

The app's reset inside the callback means the LOW that ended a decoded frame is not reused as the next
header, so a held button decodes every other repeat (KAT does the same).

## Encoder

`subghz_protocol_encoder_came_get_upload`, `encoder.repeat` = 3: header LOW (by bit count), start HIGH,
the key's bits; keys up to 42 bits (the C's limit). CAME has no button field; `button` is ignored (ARF's
lib encoder also re-sends the capture for every D-pad direction).

## Key file

`Protocol: CAME`, `Bit`, `Key` (Flipper-ARF's generic serialize); more than 42 bits does not load.

## Validation

Against the garage C oracle (`decode --protocol CAME`): 41 identical decodes on the Flipper unit-test
corpus, 498 on 3420 fuzz files and 13867 on the UberGuidoZ brute-force sample (`CAME-12bit-*`,
`CAME_brute_force/12Bit`, `CAME_NICE/Came_top_brute.sub`); none on the 108 IMPORTS car captures. On the
first 720 fuzz files the doctor build reports 63 frames instead of 104 (its 42-bit and payload filters).
Every encoder upload (26 key files: the unit test's `came.sub` and 12/18/24/25/42-bit random keys, plus
the six real UberGuidoZ `Road_Arm_*.sub` keys) equals both oracles' pulse for pulse.
