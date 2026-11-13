---
layout: default
---

# Faac TM Protocol

**Rust module:** `src/protocols/faac_tm.rs`
**Reference:** `REFERENCES/RogueMaster/lib/subghz/protocols/faac_tm.c`, added in November 2026 with RogueMaster's
merge of Unleashed's dev branch. No other reference has it. RogueMaster registers it after Freezer, before Proteco, and its
CAME decoder leaves Faac TM frames alone (see [came.md](came.md)).

## Overview

12-bit static code of FAAC TM gate remotes, set by 12 DIP switches; there is no button field. The air format is
CAME's short/long grammar, slower and with a shorter header, and the decoder measures the remote's Te from each
frame.

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| Te | 400 µs nominal | measured per frame: (start bit + every data pulse) / 37 |
| Header | 35·Te LOW | accepted within 14000 ± 4200 µs |
| Start bit | Te HIGH | 400 ± 120 µs |
| Bit 1 | LOW Te + HIGH 2·Te | short 400 ± 120, long 800 ± 120 µs (exclusive) |
| Bit 0 | LOW 2·Te + HIGH Te | |
| End | a LOW ≥ 1600 µs | the next frame's header |
| Bits | 12 | MSB first (the DIP switches in the order they are sent) |

AM (OOK), 433.92 and 868 MHz (`SubGhzProtocolFlag_433 | 868 | AM`). With those windows the decoder reads remotes
from Te 341 to 459 µs.

## Decoder Steps

1. Reset: a LOW within 14000 ± 4200 µs (35·te_short ± 35·te_delta).
2. FoundStartBit: LOWs are skipped; a HIGH of 400 ± 120 µs is the start bit (the frame's Te sum starts there), any
   other HIGH resets.
3. LOW/HIGH pairs: LOW short + HIGH long = 1, LOW long + HIGH short = 0, anything else resets.
4. A LOW ≥ 1600 µs ends the frame. With exactly 12 bits it is reported with Te = sum / 37, and the app's receiver
   reset in the callback returns the decoder to Reset. The gap that closed the frame was the next copy's header, so
   of a press's back-to-back copies every other one decodes (RogueMaster's C does the same).

## Registry

After every stock fixed code, as RogueMaster registers it. CAME, earlier in the list, keeps any 12-bit frame
closed by CAME's own 47-Te gap and leaves slower ones (mean Te ≥ 340 µs) with any other gap to Faac TM.

One difference from RogueMaster's registry remains, for remotes running fast (Te ≤ 370 µs). There the 35-Te header
also opens Holtek_HT12X's 4960–12960 µs header window, and every Faac TM pulse fits HT12X's wide windows. KAT's
Holtek_HT12X is Flipper-ARF's, whose reset keeps its repeat memory, so after a Faac TM report it can confirm the
next copy as an HT12X frame (bits inverted: HT12X's 1 is LOW long + HIGH short). RogueMaster's HT12X clears that
memory on reset, so its receiver reports every copy as Faac TM. The first copy of such a press is Faac TM's in
both.

## Field mapping

No fields, as in the C (which shows the key, the measured Te and the DIP row); `data` = the 12-bit key,
`data_count_bit` = 12, `extra` = the measured Te, `encoder_capable` true.

## Encoder

`subghz_protocol_encoder_faac_tm_get_upload` (`encoder.repeat` = 3): header LOW 35·Te, start HIGH Te, the 12 bits.
Te is the key's TE when 100 < TE < 1000, else the nominal 400 µs. The button is ignored: the DIP switches are the
code.

## Key file

`Protocol: Faac TM`, `Bit: 12` (required), `Key`, `TE` (the decoder writes the measured Te). On load `TE` is
optional, as the C encoder reads it; RogueMaster's decoder deserialize needs the line, and its "Add Manually" writes
`TE: 0`, which is sent at 400 µs.

## Add Manually

"FAAC TM 433MHz" and "FAAC TM 868MHz" (RogueMaster): key = `rand() & 0xFFF`, `TE: 0`, AM650. RogueMaster's menu
puts them after FAAC SLH; KAT lists them at the end of the fixed codes. See [add_manually.md](add_manually.md).

## Validation

Host build of RogueMaster's C (`REFERENCES/tools/oracle/bin/subghz_oracle_roguemaster`, built from RogueMaster
8946baa): 60 key files (TE 0, 100, 101, 340, 341, 400, 459, 460, 999, 1000, 1500 and random values) give the same
upload in KAT and the C, and 150 perturbed RAW files (jitter, dropped, split and stretched pulses, Te 320–455 µs,
mixed codes) give the same decodes. Through each registry, an upload at Te 375–455 µs gives two Faac TM frames in
both. Parity tools: `src/protocols/f_common.rs` (`F_FILES`, `F_KEYS`).
