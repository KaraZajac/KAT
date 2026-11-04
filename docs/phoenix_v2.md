---
layout: default
---

# Phoenix V2 Protocol

**Rust module:** `src/protocols/phoenix_v2.rs`
**Reference:** `REFERENCES/Flipper-ARF/applications/system/subghz_garage/protocols/phoenix_v2.c` (garage app copy,
primary); doctor-firmware and Flipper-ARF `lib/subghz/protocols/phoenix_v2.c` decode and transmit identically.

## Overview

"V2 Phoenix" gate remote: 52-bit frame (bit-reversed) carrying a 32-bit serial (low nibble always 0, it is not
transmitted), a 4-bit button and a 16-bit counter that is obfuscated with the serial's top two bytes.

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| te_short | 427 µs | ±100 µs (te_delta) |
| te_long | 853 µs | ±300 µs (3·te_delta) inside a bit |
| Header | 25 620 µs LOW | 60·te_short, ±3 000 µs |
| Start pulse | 2 562 µs HIGH | 6·te_short, ±400 µs |
| Frame end | LOW ≥ 4 370 µs | 10·te_short + te_delta |
| Bits | 52 | MSB first |

Each bit is a LOW then a HIGH: 1 = LOW 427 + HIGH 853, 0 = LOW 853 + HIGH 427.

## Frame Layout

`rev = reverse_key(data, 56)` (the 52 bits reversed over 56, so `rev` bits 0..3 are 0):

- **rev[0:31]** serial
- **rev[32:35]** button (1, 2, 4, 8, 3)
- **rev[40:55]** encrypted counter; `decrypt_counter` un-shuffles the two bytes over 16 rounds, XORing with
  serial bytes 3 and 2 whenever the shifted-out bit is 0

## RF

- **Encoding:** PWM
- **RF modulation:** AM (OOK), 433.92 MHz
- **Encryption:** rolling code (serial-keyed counter obfuscation)

## Decoder Steps

1. **Reset** — LOW header within 3 000 µs of 25 620 µs.
2. **FoundStartBit** — HIGH within 400 µs of 2 562 µs, else Reset.
3. **SaveDuration** — a LOW ≥ 4 370 µs reports the frame when exactly 52 bits were read (and clears the register);
   a shorter LOW is the bit's first half; a HIGH here is ignored (as in the C).
4. **CheckDuration** — HIGH: (short LOW, long HIGH) → 1, (long LOW, short HIGH) → 0, else Reset.

## Encoder

`subghz_protocol_encoder_phoenix_v2_get_upload`: counter + 1 (0xFFFF wraps to 0); with a key file's
`CounterMode: 1` the counter steps 0 → 1 → 0xFFFE → 0xFFFF → 0, with `CounterMode: 2` it counts 0..4 (KAT keeps
the mode in `extra`). The counter is re-encrypted with the serial, the frame rebuilt as
`reverse_key(enc << 40 | button << 32 | serial, 56)` and sent as LOW 25 620, HIGH 2 562 and the low 52 bits, 3 times.
`button` (native): 1, 2, 4, 8 or 3 when the captured button is one of them (every one is reachable through the
references' custom-button switch), otherwise the captured button.

Note: the garage app's TX plugin is built without `SUBGHZ_GARAGE_HAS_CUSTOM_BTN` / `_HAS_LIB_EXTENSIONS`, so its
stubs make it send button 0 and the OFEX counter step (→ 0xFFFE); KAT follows the protocol code with the real
helpers (doctor-firmware's behaviour): original button, counter + 1.

## Key file

`Protocol: Phoenix_V2`, `Bit: 52`, `Key`, plus `CounterMode` when the loaded file had one. The reference encoder
writes the bumped Key back.

## Validation

- Oracle parity (doctor and garage builds, `decode --protocol Phoenix_V2`): the unit-test capture
  `phoenix_v2_raw.sub` (1 frame) and `test_random_raw.sub` (1 frame) decode with identical Key, serial, button and
  counter; no decode on any other unit-test file, IMPORTS or the UberGuidoZ set (C and KAT agree).
- Encoder: the unit-test key and four more keys (incl. `CounterMode: 1` / `2`) are pulse-exact against
  doctor-firmware (318 pulses); every custom-button switch (d-pad ids 0..4, doctor-firmware and Flipper-ARF `lib`
  builds with their real custom-button helpers) is pulse-exact too.
- Unit tests: counter cipher against the C's sample table and both directions over all 65 536 counters, key-file
  load → encode → decode, synthetic frame fields, every button path and counter mode, rejection of a bad start
  pulse / short frame / short header, the unit-test capture through the decoder and the registry.
