---
layout: default
---

# MegaCode Protocol

**Rust module:** `src/protocols/megacode.rs`
**Reference:** Flipper-ARF garage app `applications/system/subghz_garage/protocols/megacode.c` (primary;
doctor-firmware decodes and encodes the same); Flipper-ARF `lib/subghz/protocols/megacode.c` (button switch)

## Overview

Linear MegaCode garage / parking-gate remotes (318 MHz): 24 slots of 6 ms, each holding one 1 ms pulse —
at the end of the slot for a 1, after 2 ms for a 0. The start bit is always 1.

## Timing

| Parameter   | Value              | Notes                                              |
|-------------|--------------------|----------------------------------------------------|
| Pulse       | 1000 ± 200 µs      | te_short = te_long = 1000                          |
| Start LOW   | 13000 ± 3400 µs    | 11 ms after a final 1, 14 ms after a final 0       |
| LOW → 1     | 5000 ± 1000 µs     | after a 0 the LOW is first reduced by 3000 µs      |
| LOW → 0     | 2000 ± 400 µs      |                                                    |
| End LOW     | ≥ 10000 µs         |                                                    |

## Frame Layout (24 bits)

`1 | facility (4) | remote code (16) | button (3)`, MSB first. Fields when `data >> 23 == 1`
(`subghz_protocol_megacode_check_remote_controller`): **serial** = data >> 3 & 0xFFFF, **button** = data & 7,
**counter** = the facility code the C keeps in `generic.cnt` (data >> 19 & 0xF); otherwise all 0.

## RF

- **Encoding:** pulse position (one pulse per slot)
- **RF modulation:** AM (OOK); 315 MHz band (doctor also flags 433.92)
- **Encryption:** none (fixed code)
- **Buttons:** "Btn 0" … "Btn 7"

## Decoder Steps

1. **Reset** — start LOW → FoundStartBit.
2. **FoundStartBit** — a 1 ms HIGH = bit 1, anything else → Reset.
3. **SaveDuration** — a LOW ≥ 10 ms ends the frame (24 bits reported, then Reset); otherwise
   `te_last` = LOW (− 3000 after a 0 bit; the C's unsigned wrap is kept).
4. **CheckDuration** — a 1 ms HIGH with `te_last` ≈ 5000 → 1, ≈ 2000 → 0, else Reset.

## Encoder

`subghz_protocol_encoder_megacode_get_upload` (built backwards, as the C), `encoder.repeat` = 3: guard LOW,
then each bit's pulse and the LOW to the next pulse, ending on the last pulse. `button` 0..=7 replaces the
low 3 bits (ARF lib's custom button); larger codes keep the captured button.

## Key file

`Protocol: MegaCode`, `Bit: 24`, `Key`.

## Validation

Garage C oracle (`decode --protocol MegaCode`): 19 identical decodes on the Flipper unit-test corpus, 257 on
3420 fuzz files; none on IMPORTS. Encoder: 26 uploads (unit test + random keys) and the four real
UberGuidoZ `Liftmaster_Parking_Gate_*.sub` keys (facility 9) equal both oracles' pulse for pulse.
