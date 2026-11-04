---
layout: default
---

# Cham_Code (Chamberlain DIP-switch) Protocol

**Rust module:** `src/protocols/chamberlain_code.rs`
**Reference:** Flipper-ARF garage app `applications/system/subghz_garage/protocols/chamberlain_code.c`
(primary; doctor-firmware and ARF's lib copy have the same wire behaviour)

## Overview

Chamberlain / LiftMaster's old fixed codes with 7, 8 or 9 DIP switches (300–390 MHz). Every symbol is a
LOW run followed by a HIGH run, 4 time units of 1 ms: stop `0001`, DIP 1 `0011`, DIP 0 `0111`. The
decoder collects symbol nibbles and only reports a frame whose stop symbols sit where one of the three
layouts puts them.

## Timing

| Parameter  | Value              | Notes                                              |
|------------|--------------------|----------------------------------------------------|
| Unit       | 1000 µs            | ±200 µs on each run (1, 2 or 3 units)              |
| Start LOW  | 39000 ± 4000 µs    | 36 units of silence + the first stop's 3 LOW units |
| End LOW    | > 5000 µs          |                                                    |

## Frame Layout

- **9 DIP:** stop, d8..d0, stop (11 symbols);
- **8 DIP:** stop, d7..d3, stop, d1, d0, stop (10 symbols — d2, "DIP 6", is not sent and decodes as 0);
- **7 DIP:** stop, d6..d1, stop, stop, d0, stop (11 symbols).

`data` = the DIP code (bit i = DIP i), `data_count_bit` = 7, 8 or 9. No other fields.

## RF

- **Encoding:** PWM (4-unit symbols)
- **RF modulation:** AM (OOK); 315 MHz band (doctor also flags 433.92)
- **Encryption:** none (fixed code)

## Decoder Steps

1. **Reset** — start LOW → FoundStartBit.
2. **FoundStartBit** — a 1 ms HIGH (the first stop) → symbols.
3. **SaveDuration / CheckDuration** — LOW 3 + HIGH 1 = stop, 2 + 2 = 1, 1 + 3 = 0, else Reset; a LOW > 5 ms
   ends the frame: 10..=11 symbols must match the 7-, then 8-, then 9-DIP stop mask and every DIP nibble
   must be `0011`/`0111` (`check_mask_and_parse`). Then Reset (the next frame needs a new start LOW).

## Encoder

`subghz_protocol_encoder_chamb_code_get_upload`, `encoder.repeat` = 3: 36 zero units + the frame's 44 (7/9
DIP) or 40 (8 DIP) units, run-length coded at 1000 µs per unit
(`subghz_protocol_blocks_get_upload_from_bit_array`). Only 7/8/9-bit keys transmit; `button` is ignored.

## Key file

`Protocol: Cham_Code`, `Bit` (≤ 10 to load, 7..=9 to transmit), `Key`.

## Validation

There is no Flipper unit-test capture for this protocol. Garage C oracle (`decode --protocol Cham_Code`):
236 identical decodes on 3420 fuzz files; none on the unit-test corpus or IMPORTS. The UberGuidoZ
`Chamberlain-9bit-*` brute-force files (tobiabocchi's generator) put 10 DIP symbols between the stops, so
neither the C nor KAT decodes them. Encoder: 25 random 7/8/9-DIP keys equal both oracles' uploads pulse
for pulse.
