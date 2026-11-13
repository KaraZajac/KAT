---
layout: default
---

# Proteco Protocol

**Rust module:** `src/protocols/proteco.rs`
**Reference:** `REFERENCES/RogueMaster/lib/subghz/protocols/proteco.c` (no other reference has it). RogueMaster's
unit tests carry its captures: `unit_tests/subghz/proteco_raw.sub`, `proteco_b_raw.sub` and `proteco_c_raw.sub`
(RAW recordings of buttons A, B and C of one TX3 remote) and `proteco.sub` (a key file). RogueMaster registers it
last, after Faac TM.

## Overview

24-bit fixed code of Proteco MOVER gate remotes: a 22-bit serial and a 2-bit button. There is no checksum and no
repeat check: a frame counts when the gap in front of it fits the frame's own clock.

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| Short symbol (Te) | 440 µs nominal | ± 150 µs |
| Long symbol | 880 µs nominal | ± 150 µs |
| Guard | LOW of 20.5·Te (9020 µs) | at least 1760 µs (4·Te) to be taken as one; 18.5..=22.5 Te of the frame's clock to count |
| Start bit | one short HIGH | belongs to no cell |
| Bit 1 | long LOW + short HIGH | |
| Bit 0 | short LOW + long HIGH | |
| Bits | 24 | MSB first |

AM (OOK), 433.92 MHz (`SubGhzProtocolFlag_433 | AM`). This is CAME's line coding at a Te of about 440 µs instead
of 320 µs: CAME's decoder takes the guard and the start bit and then drops the frame at its first long pulse.

## Frame Layout

| Bits | Field |
|------|-------|
| 23..2 | serial (22 bits) |
| 1..0 | button |

| Low 2 bits | Button | KAT / C number |
|------------|--------|----------------|
| `0x2` | A (the gate) | 1 |
| `0x0` | B | 2 |
| `0x1` | C | 3 |
| `0x3` | (not seen on a remote) | 4 |

The C numbers the buttons 1..4 so that 0 keeps its "no button seen yet" meaning. A, B and C are a TX3 remote's;
the fourth code is unverified in the C and has no name in KAT.

## Decoder Steps

1. Reset: a LOW ≥ 1760 µs. It is kept as the frame's guard.
2. Start: a HIGH of 440 ± 150 µs; anything else resets.
3. Cells: a LOW opens a cell, a HIGH closes it (short + long = 0, long + short = 1). A broken cell resets.
4. A LOW ≥ 1760 µs ends the frame. It is reported when it has exactly 24 bits and its guard is 18.5..=22.5 Te of
   the frame's own clock (the mean Te over the start bit and the 24 cells, 73 Te in all). A remote off clock
   passes, a foreign 24-bit frame that only sits near 9 ms does not.
5. The closing LOW becomes the next frame's guard and the decoder waits for the start bit.

The app resets the receiver in the decode callback, and the C code after the callback overrides that, so a held
button reports every frame. KAT keeps that state through the registry's reset that follows its own report.

## Field mapping

`serial`, `button` as the C (`subghz_protocol_decoder_proteco_get_string`); `counter` None; `data` = the 24 bits;
`crc_valid` false (no checksum); `encoder_capable` true. Buttons A, B and C are shown by name.

## Encoder

`subghz_protocol_encoder_proteco_get_upload` (`encoder.repeat` = 10): LOW 9020, the start HIGH 440, the 24 cells
(1 = LOW 880 + HIGH 440, 0 = LOW 440 + HIGH 880). The upload ends on the last cell's HIGH. A button of 1..4 (the
C's custom buttons; "Send button…" in KAT) rewrites the low 2 bits; Send sends the code as captured. A key file's
`Repeat` line sets the number of copies, as in the C.

Each copy is closed by the next one's guard and the last one by the silence after the transmission, so a receiver
reads all ten. (A RAW file that holds the upload alone ends on the last HIGH and reads nine.)

## Key file

`Protocol: Proteco`, `Bit: 24` (required), `Key`.

## Add Manually

`Proteco 433MHz` (RogueMaster's "Proteco"): a random code with the low 2 bits set to button A,
`(r & 0x00FFFFFC) | 0x2`, 433.92 MHz AM650. See [Add Manually](add_manually.md).

## Validation

Host build of RogueMaster's C (`REFERENCES/tools/oracle/bin/subghz_oracle_roguemaster`, built from RogueMaster
0cfd4bf):
- RogueMaster's captures decode alike in both, and as nothing else in KAT's registry: `proteco_raw.sub` 0xDF2396
  (button A) four times, `proteco_b_raw.sub` 0xDF2394 (B) eleven times, `proteco_c_raw.sub` 0xDF2395 (C) nine
  times, serial 0x37C8E5 in all three. `proteco.sub` loads as 0xDF2396.
- 240 generated RAW files (Te 300–590 µs, guards of 10–30 Te, frames of 12, 23, 25 and 26 bits among the 24-bit
  ones, jitter, dropped, split and stretched pulses, up to three codes in a file) and the three captures give the
  same decodes in the same order in every file: 262 decodes in 48 files.
- 64 key files (random codes, all zeros, all ones, bits set above the 24th, RogueMaster's `proteco.sub`) give the
  same upload in KAT and the C. `Bit: 23` and `Bit: 25` are refused by both. With `Repeat: 1` and `Repeat: 3` the
  C sends one and three of the copies KAT's encoder builds, and KAT's transmit path sends as many
  (`tx_copies.rs`).
- A remote created with `:new` and exported as a key file loads in the C with KAT's key, serial and button, and
  the C's transmission of it decodes in KAT as that key.

Unit tests (`src/protocols/proteco.rs`): the upload's layout, the round trip of all ten copies, the button table
both ways, the guard rule at its edges and with a remote off clock, the key file, RogueMaster's captures, the
upload through the whole registry (Proteco alone reads it), jittered frames.
