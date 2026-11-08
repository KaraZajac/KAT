---
layout: default
---

# KeyFinder Protocol

**Rust module:** `src/protocols/keyfinder.rs`
**Reference:** `REFERENCES/Flipper-ARF/applications/system/subghz_garage/protocols/keyfinder.c` (@xMasterX,
2026.03), registered in the garage app's RX group g1 with a TX plugin (`tx_keyfinder`). doctor-firmware,
RogueMaster, Momentum and FoxFW register the same decoder and encoder (feed, upload and fields identical; doctor and
RogueMaster flag it a sensor, FoxFW loads its encoder and text from SD plugins). Flipper-ARF's own
`lib/subghz/protocols/keyfinder.c`, which ARF's registry leaves out, adds custom buttons on TX (see Encoder). No
Flipper unit test has a KeyFinder capture.

## Overview

Four-colour key-finder remotes: a 24-bit fixed code, a 20-bit serial and a 4-bit ID per colour button (the C's
samples: Red C396F**E**, Purple C396F**B**, Green C396F**D**, Blue C396F**C**). No checksum.

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| Short | 400 µs (te_short) | ± 150 µs |
| Long | 1200 µs (te_long) | ± 150 µs |
| Bit 1 | HIGH short + LOW long | |
| Bit 0 | HIGH long + LOW short | |
| End | 3 × (HIGH 400 + LOW 400), HIGH 400 | four end pulses |
| Gap | 4000 µs (10·te_short) | window ± 750 µs |
| Bits | 24 | MSB first |

AM (OOK), 433.92 MHz (`SubGhzProtocolFlag_433 | AM`).

## Frame Layout

| Bits | Field |
|------|-------|
| 23..4 | serial (20 bits) |
| 3..0 | ID: 0xE Red, 0xB Purple, 0xD Green, 0xC Blue |

The C also shows the 24 bits reversed ("Yek").

## Decoder Steps

1. Reset: a LOW with |d − 4000| < 750.
2. Bit pairs: HIGH saved, then the LOW: short + long = 1, long + short = 0; anything else back to Reset.
3. With 24 bits: a HIGH ≈ 400 counts an end pulse, a LOW ≈ 400 is skipped, anything else clears the frame. The fourth
   end pulse arms Finish.
4. Finish: the next pulse, whatever it is (on air the gap), reports the 24 bits; the frame is cleared.

The C's `reset()` clears the parser step only; its end-pulse counter is cleared in the feed paths, so it survives a
reset caused by another protocol's report (KAT keeps that). A report consumes the gap that would open the next copy:
of the encoder's 5 back-to-back copies the 2nd and the 4th decode, in the C and in KAT.

## Field mapping

`serial` = data >> 4, `button` = the ID (labels Red / Purple / Green / Blue, other IDs "Btn 0xN"); `counter` None;
`data` = the 24 bits; `crc_valid` false (no checksum); `encoder_capable` true.

## Encoder

`subghz_protocol_encoder_keyfinder_get_upload` (`encoder.repeat` = 5): the 24 bits, three HIGH 400 + LOW 400 pairs, a
HIGH 400 and the 4000 µs gap; 5 copies. Button: Flipper-ARF lib's custom buttons re-encode the key as serial << 4 |
button (its D-pad: UP Red 0xE, DOWN Purple 0xB, LEFT Green 0xD, RIGHT Blue 0xC; OK keeps the captured ID). KAT
sends KAT's native code 0..=0xF the same way (larger codes keep the captured ID), and "Send button…" lists the four
colours. The garage app's copy (the primary reference) has no button switch: KAT takes the union. No counter.

## Key file

`Protocol: KeyFinder`, `Bit: 24`, `Key`; loading requires `Bit: 24`.

## Validation

Reference oracle: `REFERENCES/tools/oracle/bin/subghz_oracle_garage`; parity tools `F_FILES` / `F_KEYS` in
`src/protocols/f_common.rs`.

- No Flipper unit test has a KeyFinder capture. On the 304 RAW captures of IMPORTS and the three unit-test folders,
  the C decoder and KAT's both report nothing.
- Encoder: 72 random key files (each colour ID 12 times, other IDs, 4 with bits above 24) — pulse-identical to the
  garage oracle's `encode`. A button switch sends exactly the upload of the key with that ID (unit test), which is
  what Flipper-ARF lib's re-encoding does.
- Decoder: 800 fuzz RAW files (jitter, dropped / split / stretched pulses, window-edge values, scaled timing, 2..6
  end pulses, wrong bit counts, mixed codes) decoded by the KeyFinder decoder alone (reset after each report, as
  `decode --protocol KeyFinder`): identical, 351 frames.
- Registry: KeyFinder transmissions decode as KeyFinder only; `kat_diff.py` over IMPORTS and the unit tests shows no
  change.
- Unit tests: copies 2 and 4 decode, upload layout, the four colour buttons, end-pulse count and window bounds, the
  end counter surviving a reset, key-file round trip.
