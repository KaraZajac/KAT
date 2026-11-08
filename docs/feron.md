---
layout: default
---

# Feron Protocol

**Rust module:** `src/protocols/feron.rs`
**Reference:** `REFERENCES/doctor-firmware/lib/subghz/protocols/feron.c` (@xMasterX, 2025; the stock Flipper protocol,
registered by doctor, Momentum, RogueMaster and FoxFW). Flipper-ARF's `lib` and garage-app copies (both
unregistered), Momentum's, RogueMaster's and FoxFW's (encoder in an SD plugin, `protocols_ext/enc_feron.c`) decode
and encode identically; they differ only in style and in the display text (doctor / Momentum / RogueMaster:
`Serial: 0x…` and `Command: 0x…`; ARF: `SN:0x…`). Flipper unit tests: `feron_raw.sub`, `feron.sub`.

## Overview

32-bit fixed code of Feron LED-light remotes: a 16-bit serial and a 16-bit command (in the C's samples a command
byte followed by its complement: ON `857A`, OFF `847B`, brightness up `8679` / down `8778`, scroll mode `827D`,
colours `708F` / `1AE5` / `40BF`). No checksum.

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| Bit 0 | HIGH 350 + LOW 750 µs | ± 150 µs each |
| Bit 1 | HIGH 750 + LOW 350 µs | ± 150 µs each |
| End marker | LOW 500 + HIGH 500 µs | replaces the last bit's LOW; LOW window 351..649 µs |
| Gap | LOW 4500 µs (6·te_long) | window 3901..5099 µs |
| Bits | 32 | MSB first |

AM (OOK), 433.92 MHz (`SubGhzProtocolFlag_433 | AM`, Sensors).

## Frame Layout

| Bits | Field |
|------|-------|
| 31..16 | serial |
| 15..0 | command (shown by the C, not stored as a button) |

## Decoder Steps

1. Reset: a LOW of |d − 4500| < 600.
2. HIGH (saved) + LOW pairs: HIGH 350 ± 150 + LOW 750 ± 150 = 0, HIGH 750 ± 150 + LOW 350 ± 150 = 1.
3. Otherwise a LOW of |d − 500| < 150 is the end marker: the saved HIGH adds its bit (≈ 350 → 0, ≈ 750 → 1), and a
   frame of exactly 32 bits is reported. Anything else resets.
4. Back to Reset: the marker's HIGH and the gap after it open the next frame, so every copy after the first decodes.

The marker is tried after the two bit cells: after a last HIGH of 750 µs a marker LOW under 500 µs is read as a
1 cell (the frame then misses its end, and the gap goes to the cell check, so the next copy loses its sync too), and
after a last HIGH of 350 µs a LOW over 600 µs is read as a 0 cell — as in the C.

## Field mapping

`serial` = data >> 16 (`subghz_protocol_feron_check_remote_controller`); button and counter: none (the C stores no
button; the command is the key's low 16 bits); `data` = the 32 bits; `crc_valid` false; `encoder_capable` true.

## Encoder

`subghz_protocol_encoder_feron_get_upload` (`encoder.repeat` = 3): the 32 bit pairs, the last one's LOW replaced by
LOW 500, HIGH 500, LOW 4500. The captured code is sent (the C has no button override); the first copy has no gap
before it, so a receiver decodes copies 2 and 3.

## Key file

`Protocol: Feron`, `Bit: 32` (required), `Key`.

## Add Manually

None of the references offers a Feron type.

## Validation

Host builds of the references' C (`REFERENCES/tools/oracle/bin/subghz_oracle_{doctor,arf,momentum,roguemaster,foxfw}`):
- `feron_raw.sub` (ARF, Momentum and RogueMaster unit-test folders): 102 Feron frames through KAT's whole registry,
  all 0x6338847B (serial 0x6338), and nothing else; `feron.sub` loads and its upload decodes back twice.
- Encoder: 60 random key files give the same upload, pulse for pulse, in KAT and in doctor's, ARF's, Momentum's
  and RogueMaster's C.
- Decoder: 200 fuzz files (repeats, mixed codes, wrong lengths, other protocols' frames, noise; jitter, dropped,
  split and stretched pulses, timing scaled ±15 %) give the same 141 frames in KAT's decoder and in all five
  references; on the 304 real RAW files (IMPORTS and the unit-test folders) and the other protocols' fuzz files
  Feron decodes only its own captures, as the C.
- Registry: the fuzz streams decode the same through KAT's whole registry as through doctor's.

Parity tools: `src/protocols/f_common.rs` (`F_FILES`, `F_KEYS`).
