---
layout: default
---

# GangQi Protocol

**Rust module:** `src/protocols/gangqi.rs`
**Reference:** `REFERENCES/doctor-firmware/lib/subghz/protocols/gangqi.c` (@xMasterX, Unleashed lineage). Flipper-ARF's
`lib/subghz/protocols/gangqi.c` and its garage-app copy (`applications/system/subghz_garage/protocols/gangqi.c`, RX
group g10 + a TX plugin), Momentum's and RogueMaster's copies decode and encode identically; the stock Flipper
firmware ships it too (unit tests `gangqi_raw.sub`, `gangqi.sub`).

## Overview

34-bit fixed code of GangQi (G-series GSM/home alarm) remotes: a 16-bit serial, a constant nibble 0xD, a 4-bit
button and an 8-bit byte sum. No preamble — frames are separated by ~2.4 ms gaps. Static code (no counter).

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| te_short | 500 µs | ± te_delta 200 µs |
| te_long | 1200 µs | ± 200 µs |
| Gap | 2400 µs | decoder: \|d − 2·te_long\| < 3·te_delta (1801–2999 µs); encoder sends 4·te_short + te_delta = 2200 µs |
| Bits | 34 | MSB first |

AM (OOK), 433.92 MHz (`SubGhzProtocolFlag_433 | AM`). Bit 1 = HIGH 1200 + LOW 500, bit 0 = HIGH 500 + LOW 1200;
the last bit's LOW is the gap.

## Frame Layout (34 bits)

| Bits | Field |
|------|-------|
| 33..18 | serial (16 bits) |
| 17..14 | constant 0xD |
| 13..10 | button |
| 9..2 | byte sum |
| 1..0 | 0 |

The C reports `serial` = (data & 0xFFFFF0000) >> 16 (the 16-bit serial plus the top two constant bits) and
`btn` = (data >> 10) & 0xF. Byte sums (`subghz_protocol_decoder_gangqi_get_string`): type 1 (original remotes)
= 0xC8 − serial_high − serial_low − (0xD0 | btn), type 2 (a "backdoor" sum the receivers accept) = 0x02 +
serial_high + serial_low + (0xD0 | btn). The C only displays both. Buttons (the C's names): 0xD Arm, 0xE Disarm,
0xB Alarm, 0x7 Ring; settings mode: 0x5 Settings mode, 0x1 Exit settings, 0x2 Volume, 0x4 Vibro sensitivity,
0x6 Ringtone.

## Decoder Steps

1. Reset: a LOW in the gap window starts a frame.
2. HIGH (saved) + LOW pairs: short/long → 0, long/short → 1.
3. A gap-length LOW closes the frame: the last bit is read from its HIGH alone (two independent tests, ≈500 → 0
   then ≈1200 → 1, as in the C); exactly 34 bits → reported. Back to Reset: the gap that closed a frame does not
   open the next one, so a held button reports every other frame (as in the C).
4. **KAT gate:** reported only when the frame's sum field equals its type-1 or type-2 sum (the C reports every
   34-bit frame and displays the two sums; KAT's checksum policy for gate remotes). All 25 frames of the unit-test
   capture pass; no reference capture fails it.

## Field mapping

`serial` and `button` as the C; `counter` None; `data` = the 34-bit frame; `crc_valid` = the sum holds (always for
decodes, see above; loaded key files may fail it); `encoder_capable` true.

## Encoder

`subghz_protocol_encoder_gangqi_get_upload` (`encoder.repeat` = 3): data = (data >> 14) << 14 | btn << 10 |
sum_type1 << 2 (a type-2 frame is re-sent with the type-1 sum, the two low bits cleared), then 34 HIGH/LOW pairs, the
2200 µs gap after the last. Button: KAT's native code 0..=0xF is sent (the references' signal-settings button
editor takes any 4-bit value; their custom-button D-pad cycles 0xD / 0xE / 0xB / 0x7); larger codes keep the
captured button. "Send button…" lists the four buttons, then the settings-mode codes. The garage app's TX plugin
(built without its custom-button flag) always sends button 0 — a build artefact KAT does not copy, as for
Princeton and Roger. No counter.

## Key file

`Protocol: GangQi`, `Bit: 34`, `Key` (generic serialize); loading requires `Bit: 34`.

## Add Manually

"GangQi 433MHz" (Flipper-ARF / doctor-firmware / Momentum / RogueMaster): 16-bit serial from the second `rand()`,
button 0xD, type-1 sum (`subghz_txrx_gen_serial_gangqi`). See [add_manually.md](add_manually.md).

## Validation

Reference oracles: `subghz_oracle_{doctor,arf,garage,momentum,roguemaster}` (host builds of the references' C, `decode --protocol GangQi`; the parity tools are the `#[ignore]`d tests in `src/protocols/f_common.rs`).

- Flipper unit test `gangqi_raw.sub`: 25 frames through KAT's whole registry, identical to all five references;
  `gangqi.sub` loads, encodes and decodes back (the bare upload through the registry, as
  `flipper_unit_test_encoders` does).
- Decode parity on 1338 distinct RAW files (the Flipper unit-test resources of ARF / Momentum / RogueMaster /
  Unleashed / stock, IMPORTS, UberGuidoZ): identical with all five (25 frames, all in `gangqi_raw.sub`).
- Fuzz: 1000 streams (250 per F protocol: KAT uploads with jitter, dropped / split / stretched pulses, other
  buttons, back-to-back frames; a quarter of the GangQi frames with a wrong byte sum): identical but for the 106
  wrong-sum frames the C reports and KAT drops (C 452, KAT 346).
- Reference registries (doctor, RogueMaster: every G1 fixed-code protocol registered before GangQi) report only
  GangQi on these streams and the unit-test capture: no frame completes on the same pulse in another decoder, so
  KAT's placement before G1 changes nothing.
- Encoder: the unit-test key and 100 random keys (a third with arbitrary sum / low bits) — KAT's upload equals
  doctor's, ARF's, Momentum's and RogueMaster's pulse for pulse; the garage app's differs (button 0) on 99.
- Unit tests: every button, type-2 sums, the gap window bounds, wrong length / timing / sum rejected, key-file
  round trip, held-button behaviour.
