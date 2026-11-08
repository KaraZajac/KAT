---
layout: default
---

# Doorbell32 Protocol

**Rust module:** `src/protocols/doorbell32.rs`
**Reference:** `REFERENCES/Flipper-ARF/applications/system/subghz_garage/protocols/doorbell32.c`, registered in the
garage app's RX group g13 with a TX plugin (`tx_doorbell32`). doctor-firmware, RogueMaster and FoxFW register the
same decoder and encoder (feed, upload, fields and the `TE` line identical; doctor and RogueMaster flag it a sensor
and write `TE` through `common_serialize_te`, FoxFW loads its encoder and text from SD plugins). No Flipper unit test
has a Doorbell32 capture.

## Overview

Wireless doorbells: a 32-bit fixed code in Princeton's 1:3 PWM, with a 14-Te guard instead of Princeton's 31 Te (so
Princeton syncs on the guard and drops the frame on its bit count, doctor's comment). Te is measured per frame and
saved: it differs between units. The only check is the repeat.

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| Te | 515 µs nominal (te_short) | HIGH / LOW windows ± 200 µs |
| 3·Te | 1545 µs (te_long) | windows ± 400 µs |
| Bit 0 | HIGH Te + LOW 3·Te | |
| Bit 1 | HIGH 3·Te + LOW Te | |
| Closing HIGH | Te | |
| Guard | 14·Te (7210 µs) | start window ± 1400 µs (5811..8609); any LOW ≥ 3090 µs ends a frame |
| Bits | 32 | MSB first |

AM (OOK), 433.92 MHz (`SubGhzProtocolFlag_433 | AM`). The guard window reads Te 416..614 µs.

## Decoder Steps

1. Reset: a LOW with |d − 7210| < 1400.
2. HIGH (saved), then the LOW: HIGH ≈ 515 + LOW ≈ 1545 → 0, HIGH ≈ 1545 + LOW ≈ 515 → 1; anything else back to Reset.
   Every HIGH and every bit LOW is added to `te`.
3. A LOW ≥ 3090 µs ends the frame: with exactly 32 bits equal to the previous frame (`last_data`, 32 bits, non-zero)
   it is reported with Te = te / 129 (32 bits of 4 Te plus the closing HIGH). The frame becomes `last_data` and the
   decoder goes on with the next frame (SaveDuration), overriding the callback's receiver reset — KAT keeps that
   state through the registry's reset that follows its own report.

`reset()` clears the parser step only, so `last_data` also survives other protocols' reports. A held button reports
every frame from the third on: the first has no guard before it, the second only fills `last_data` (8 of the
encoder's 10 copies, in the C and in KAT).

## Field mapping

`serial`, `button`, `counter` None (the C shows the key, the Te and "GT:Te*14"); `data` = the 32 bits; `extra` = the
measured Te; `crc_valid` false (no checksum); `encoder_capable` true.

## Encoder

`subghz_protocol_encoder_doorbell32_get_upload` (`encoder.repeat` = 10): the 32 bits at Te, the closing HIGH and the
14·Te guard; 10 copies. Te = the key file's `TE`, 515 µs when it has none (or it does not parse). The C sends a
`TE: 0` key as zero-length pulses; KAT returns no upload for it. No button field: "Send button…" lists nothing.

## Key file

`Protocol: Doorbell32`, `Bit: 32`, `Key`, `TE` (the measured Te, as the decoder's serialize writes it). Loading
requires `Bit: 32`; the C's decoder deserialize ignores `TE`, its encoder reads it: KAT keeps it when present and
writes it back, and a key without it is sent at 515 µs.

## Validation

Reference oracle: `REFERENCES/tools/oracle/bin/subghz_oracle_garage`; parity tools `F_FILES` / `F_KEYS` in
`src/protocols/f_common.rs`.

- No Flipper unit test has a Doorbell32 capture. On the 304 RAW captures of IMPORTS and the three unit-test folders,
  the C decoder and KAT's both report nothing.
- Encoder: 64 random key files (TE 300..800, 8 without TE, 4 far outside the decode range) — pulse-identical to the
  garage oracle's `encode`.
- Decoder: 800 fuzz RAW files (Te 380..660, jitter, dropped / split / stretched pulses, window-edge values incl. the
  3090 µs end threshold, scaled timing, 24/31/33/40-bit frames, code sequences AABB / ABAB / ABBA …) decoded by the
  Doorbell32 decoder alone (reset after each report, as `decode --protocol Doorbell32`): identical keys and `TE`,
  401 frames.
- Registry: Doorbell32 transmissions (Te 450..580) decode as Doorbell32 only; `kat_diff.py` over IMPORTS and the unit
  tests shows no change.
- Unit tests: 8 of 10 copies decode with the exact Te, upload layout, repeat rule (single frame, two codes, zero key,
  31 bits), guard and end-threshold bounds, `last_data` surviving resets, key-file round trip with and without `TE`.
