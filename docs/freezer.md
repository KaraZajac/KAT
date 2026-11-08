---
layout: default
---

# Freezer Protocol

**Rust module:** `src/protocols/freezer.rs`
**Reference:** `REFERENCES/RogueMaster/lib/subghz/protocols/freezer.c` (no other reference has it). RogueMaster's
unit tests carry its captures: `unit_tests/subghz/freezer_raw.sub` (a RAW recording) and `freezer.sub` (a key
file). RogueMaster registers it after Cardin S508, before Faac TM.

## Overview

24-bit fixed code: a 20-bit serial and a 4-bit button. There is no checksum: a frame counts only when the remote
sends it twice in a row.

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| Short symbol | 250 µs nominal | 120..=375 µs |
| Long symbol | 500 µs nominal | 376..=700 µs |
| Gap | ≥ 7500 µs (30·te_short) | the encoder sends 9000 µs (36·te_short) |
| Lead | one short HIGH | belongs to no cell |
| Bit 1 | short LOW + long HIGH | |
| Bit 0 | long LOW + short HIGH | |
| Bits | 24 | MSB first |

AM (OOK), 433.92 MHz (`SubGhzProtocolFlag_433 | AM`). The symbols are classified by a split at 375 µs rather than
by windows: the C's measured classes, 158–359 and 392–594 µs, are only 33 µs apart while each spreads over 200 µs.
Refusing everything over 700 µs keeps Princeton frames, whose long symbol is about 1170 µs, from ever completing a
Freezer frame.

## Frame Layout

| Bits | Field |
|------|-------|
| 23..4 | serial (20 bits) |
| 3..0 | button |

## Decoder Steps

1. Reset: a LOW ≥ 7500 µs.
2. Start: a short HIGH (the lone pulse); anything else resets.
3. Cells: a LOW opens a cell, a HIGH closes it (short + long = 1, long + short = 0). A broken cell clears the
   previous frame and resets.
4. A LOW ≥ 7500 µs ends the frame. A 24-bit frame equal to the previous 24-bit frame (non-zero) is reported. The
   frame then becomes the previous frame (any other length clears it) and the decoder waits for the next lone HIGH.

The app resets the receiver in the decode callback (`freezer_reset` clears the previous frame), and the C code
after the callback overrides that, so a held button reports every frame after the first. KAT keeps that state
through the registry's reset that follows its own report.

## Field mapping

`serial`, `button` as the C (`subghz_protocol_freezer_check_remote_controller`); `counter` None; `data` = the 24
bits; `crc_valid` false (no checksum); `encoder_capable` true. Buttons are shown as numbers (Btn 0..F).

## Encoder

`subghz_protocol_encoder_freezer_get_upload` (`encoder.repeat` = 3): the lone HIGH 250, the 24 cells (1 = LOW 250 +
HIGH 500, 0 = LOW 500 + HIGH 250), LOW 9000. The captured code is sent: the C has no button override.

## Key file

`Protocol: Freezer`, `Bit: 24` (required), `Key`.

## Validation

Host build of RogueMaster's C (`REFERENCES/tools/oracle/bin/subghz_oracle_roguemaster`, built from RogueMaster
8946baa):
- `freezer_raw.sub` decodes as Freezer 0xC31C1F five times in both, and nothing else in KAT's registry.
- 60 random key files give the same upload in KAT and the C.
- 150 perturbed RAW files (jitter, dropped, split and stretched pulses, symbol timings 200–320 / 420–650 µs, mixed
  codes) give the same decodes.

Parity tools: `src/protocols/f_common.rs` (`F_FILES`, `F_KEYS`).
