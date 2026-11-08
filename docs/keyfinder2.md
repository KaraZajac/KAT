---
layout: default
---

# KeyFinder2 Protocol

**Rust module:** `src/protocols/keyfinder2.rs`
**Reference:** `REFERENCES/Flipper-ARF/applications/system/subghz_garage/protocols/keyfinder2.c`, registered in the
garage app's RX group g1 with a TX plugin (`tx_keyfinder2`). doctor-firmware, RogueMaster and FoxFW register the same
decoder and encoder (feed, upload and fields identical; doctor and RogueMaster flag it a sensor, FoxFW loads its
encoder and text from SD plugins). No copy has custom buttons. No Flipper unit test has a KeyFinder2 capture.

## Overview

A second key-finder family: an 11-bit fixed code, 3 wide header bits and 8 narrow bits whose low three bits are the
colour button. Nothing is checked beyond the timing.

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| Header short / long | 1500 / 2500 µs | ± 350 µs |
| Short / long | 350 / 1020 µs (te_short / te_long) | ± 150 µs |
| Bit 1 | HIGH short + LOW long | |
| Bit 0 | HIGH long + LOW short | |
| Bits | 11 | MSB first: 3 header (wide) + 8 (narrow) |
| Gap | none | the encoder's copies follow each other directly |

AM (OOK), 433.92 MHz (`SubGhzProtocolFlag_433 | AM`).

## Frame Layout

| Bits | Field |
|------|-------|
| 10..8 | header bits (wide) |
| 7..3 | code |
| 2..0 | button: 3 Purple, 4 Blue, 5 Green, 6 Red (the C: else "Unknown") |

## Decoder Steps

1. Reset: a HIGH within 350 µs of 1500 or 2500 starts the first header bit.
2. Header bits: HIGH/LOW pairs within 350 µs (1500 + 2500 = 1, 2500 + 1500 = 0); after 3 the narrow bits follow.
3. Narrow bits: pairs within 150 µs (350 + 1020 = 1, 1020 + 350 = 0). Anything else returns to Reset.
4. The 11th bit's LOW reports the frame; back to Reset, so every copy decodes.

On air the last copy's final LOW runs into the silence after the transmission, so a receiver gets 4 of the encoder's
5 copies (the C's encoder round trip, fed the upload as it is, gets 5).

## Field mapping

`serial` None (the C sets the button only), `button` = data & 7 (labels Purple / Blue / Green / Red, other codes "Btn
0xN"); `counter` None; `data` = the 11 bits (the C shows data & 0x7FF); `crc_valid` false; `encoder_capable` true.

## Encoder

`subghz_protocol_encoder_keyfinder2_get_upload` (`encoder.repeat` = 5): the 11 bits (the first 3 wide), no gap; 5
copies back to back. The captured code is sent: no reference switches buttons, so "Send button…" lists nothing.

## Key file

`Protocol: KeyFinder2`, `Bit: 11`, `Key`; loading requires `Bit: 11`.

## Validation

Reference oracle: `REFERENCES/tools/oracle/bin/subghz_oracle_garage`; parity tools `F_FILES` / `F_KEYS` in
`src/protocols/f_common.rs`.

- No Flipper unit test has a KeyFinder2 capture. On the 304 RAW captures of IMPORTS and the three unit-test folders,
  the C decoder and KAT's both report nothing.
- Encoder: 64 random key files (every 3-bit button 8 times) — pulse-identical to the garage oracle's `encode`.
- Decoder: 800 fuzz RAW files (jitter, dropped / split / stretched pulses, window-edge values for both bit widths,
  scaled timing, 9..13-bit frames, gaps between copies, mixed codes) decoded by the KeyFinder2 decoder alone (reset
  after each report, as `decode --protocol KeyFinder2`): identical, 431 frames.
- Registry: KeyFinder2 transmissions decode as KeyFinder2 only (last of the block: it checks nothing); `kat_diff.py`
  over IMPORTS and the unit tests shows no change.
- Unit tests: every copy decodes, upload layout, wide and narrow window bounds, short frames and narrow headers
  rejected, key-file round trip.
