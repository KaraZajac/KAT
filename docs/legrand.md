---
layout: default
---

# Legrand Protocol

**Rust module:** `src/protocols/legrand.rs`
**Reference:** `REFERENCES/doctor-firmware/lib/subghz/protocols/legrand.c` (the stock Flipper protocol; doctor,
Momentum, RogueMaster and FoxFW register it). Flipper-ARF's `lib` and garage-app copies (both unregistered),
Momentum's, RogueMaster's and FoxFW's (its encoder is an SD plugin, `applications/main/subghz/protocols_ext/enc_legrand.c`)
decode and encode identically; they differ only in style, in the display text (ARF's `lib` copy has no `Te:` line)
and in the Sensors filter flag. Flipper unit tests: `legrand_raw.sub`, `legrand_2E37F_raw.sub` (the same recording),
`legrand.sub`, `legrand_2E37F.sub`.

## Overview

18-bit fixed code. The C names no fields (no serial / button split) and there is no checksum: a frame counts only
when the remote sends the same code twice in a row. The decoder measures the remote's time unit (Te) and saves it
with the key; the encoder sends at that Te.

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| Te | 375 µs nominal | measured per frame |
| Sync | LOW 16·Te (6000 µs) | window 4801..7199 µs; on air it includes the first bit's LOW |
| Bit 1 | LOW Te + HIGH 3·Te | LOW 375 ± 150 µs, HIGH 1125 ± 450 µs |
| Bit 0 | LOW 3·Te + HIGH Te | LOW 1125 ± 450 µs, HIGH 375 ± 150 µs |
| Bits | 18 | MSB first |

AM (OOK), 433.92 MHz (`SubGhzProtocolFlag_433 | AM`, Sensors).

## Frame Layout

| Bits | Field |
|------|-------|
| 17..0 | the code |

## Decoder Steps

1. Reset: a LOW with |d − 6000| < 1200.
2. First bit, from its HIGH alone (its LOW is part of the sync): 375 ± 150 → 0, 1125 ± 450 → 1; anything else
   resets.
3. LOW/HIGH cells: LOW 1125 ± 450 + HIGH 375 ± 150 = 0, LOW 375 ± 150 + HIGH 1125 ± 450 = 1; anything else
   resets.
4. The 18th bit ends the frame (the next pulse must be a sync). It is reported when it equals the previous 18-bit
   frame (non-zero, so an all-zero code never reports), and becomes the previous frame either way. A broken frame
   leaves the previous frame alone: only a reset clears it.
5. Te = (the first HIGH × 4 — or ÷ 3 × 4 for a long one — plus every later LOW and HIGH) / 72, in the C's integer
   arithmetic.

The app's decode callback resets the receiver, which clears the previous frame, and the C code after the callback
stores the frame again, so a held button reports every frame after the first. KAT keeps that state through the
registry's reset that follows its own report.

## Field mapping

`data` = the 18 bits; `extra` = the measured Te; serial, button and counter: none; `crc_valid` false (no
checksum); `encoder_capable` true.

## Encoder

`subghz_protocol_encoder_legrand_get_upload` (`encoder.repeat` = 3): LOW 16·Te, then the 18 cells, at the key's TE
as given. The sync and the first cell's LOW are one pulse on air: the C's own unit-test round trip, which feeds the
yielded pulses unmerged, fails; on air the first copy is stored and the other two report. No button.

## Key file

`Protocol: Legrand`, `Bit: 18`, `Key`, `TE: <µs>` (decimal; the decoder's serialize writes the measured Te). Both
the decoder's and the encoder's deserialize require `Bit: 18` and the `TE` line.

## Add Manually

None of the references offers a Legrand type.

## Validation

Host builds of the references' C (`REFERENCES/tools/oracle/bin/subghz_oracle_{doctor,arf,momentum,roguemaster,foxfw}`):
- `legrand_raw.sub` and `legrand_2E37F_raw.sub` (in the ARF, Momentum and RogueMaster unit-test folders): 175
  Legrand frames each through KAT's whole registry, all 0x2E37F with the oracle's Te values (1 × 356, 168 × 357,
  6 × 358), and nothing else; `legrand.sub` / `legrand_2E37F.sub` load, and their upload decodes back twice.
- Encoder: 60 random key files (TE 100..1000 µs) give the same upload, pulse for pulse (equal levels merged), in
  KAT and in doctor's, ARF's, Momentum's and RogueMaster's C.
- Decoder: 200 fuzz files (repeats, mixed codes, wrong lengths, other protocols' frames, noise; jitter, dropped,
  split and stretched pulses, Te 280..420 µs) give the same 92 frames in KAT's decoder and in all five references;
  on the 304 real RAW files (IMPORTS and the unit-test folders) and the other protocols' fuzz files Legrand decodes
  only its own captures, as the C.
- Registry: the fuzz streams decode the same through KAT's whole registry as through doctor's.

Parity tools: `src/protocols/f_common.rs` (`F_FILES`, `F_KEYS`).
