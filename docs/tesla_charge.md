---
layout: default
---

# Tesla Charge Protocol

**Rust module:** `src/protocols/tesla_charge.rs`
**Reference:** FoxFW `lib/subghz/protocols/tesla_charge.c` (`TESLA_CHARGE_PROTOCOL_NAME = "Tesla Charge"`) and its
SD-card encoder plugin `applications/main/subghz/protocols_ext/enc_tesla_charge.c`.

## Overview

The fixed code that opens a Tesla's charge port. The reference flags it `SubGhzProtocolTypeStatic`, `433 | 315 | AM`.
KAT decodes it and transmits it as FoxFW's encoder does.

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| Short | 400 µs | te_delta 140 µs |
| Long | 800 µs | |
| Gap | >= 1000 µs | a Manchester reset |

## Frame

Manchester through Flipper's `manchester_advance` (HIGH = `ShortHigh`/`LongHigh`, start state Mid1). Every decoded bit is
shifted into a 64-bit register (more than 64 bits clears it). On a reset event (a gap, or a pulse outside both windows)
the frame is reported only if **exactly 50 bits** were collected and the register equals `0x00036B5564C725E6`, the one
code every car accepts. So there is no serial, button or counter; `data` is the 50-bit code.

## Key file

`Protocol: Tesla Charge`, `Bit: 50`, `Key` (8 bytes), `TE: 400`. Both C deserializers load any 8-byte Key at exactly
50 bits, so KAT does too (only the fixed code ever decodes). FoxFW's decoder writes the constant `TE: 400` and ignores
the `TE` it loads, but its encoder sends at the file's `TE`: KAT keeps a `TE` other than 400 in `extra` and writes it
back. No `TE` line, an unreadable one or `TE: 0` means 400, as for the encoder. The C finds `TE` only after the `Key`
line, where FoxFW writes it; KAT reads it anywhere in the file.

## Transmit

`subghz_protocol_encoder_tesla_charge_deserialize` + `_get_upload`: every 50-bit key, **5 copies** (the encoder's
default `repeat`) of:

| Part | Pulses |
|------|--------|
| Preamble | 12 × (HIGH TE, LOW TE), then HIGH TE |
| Sync | LOW 3 × TE |
| Data | the Key's low 50 bits, MSB first, biphase: a bit's first half is the opposite of the previous bit's second half (HIGH for the first bit), its second half is the inverted bit; equal halves are one 2 × TE pulse |
| End | LOW 3 × TE |

TE is the key's (400 µs for a decode). Pulses go out as the C yields them: after a final 1 bit its LOW and the closing
LOW are two pulses, and durations are the C's `uint32_t` products cut to the 30 bits of a Flipper `LevelDuration`
(only absurd `TE` values reach either). The frame has no button and no counter, so TX Lock, Unlock, Trunk and Panic
and "Send next code" all send the saved code, and a transmission changes nothing on the capture. A `Repeat` line in
a key file is not applied: FoxFW deletes it before saving a file (only its CLI and counter brute force set one for a
transmission). Another Key, or a TE far from 400 µs, goes out exactly as from FoxFW and is read by no decoder (FoxFW's
own round trip reads nothing either).

## RF

- **Encoding:** Manchester
- **RF modulation:** AM (AM650)
- **Encryption:** fixed code, no checksum (the constant is the check)

## Validation

- No capture exists (the triage corpus has no Tesla Charge frame).
- Transmit: KAT against FoxFW's encoder on the host oracle (`subghz_oracle_foxfwenc encode`) for 109 made-up key files
  (the fixed code and 63 other keys, TE missing, unreadable, 0, 1 to 4294967295 µs including values whose products
  wrap or pass 30 bits, both bands; 7 with 49, 51, 64, 0 or 1 bits or a short Key): the 102 the C transmits are equal
  pulse for pulse, and both refuse the other 7. Sent as the app sends it (twice, 20 ms apart, equal levels merged),
  KAT's transmission decodes in KAT's registry and in FoxFW's decoder to the C's own frames: the code at every TE
  near 400 µs (13 files), nothing for the rest (76; 13 with 0 µs or longer than 1 s pulses not replayed). Four of the
  random keys are read by other KAT decoders (Microchip-HCS361, Nissan V0) on these same pulses.
- The unit tests keep the oracle's transmission of FoxFW's key file of the decode and of a TE 380 file with another
  Key, and check the button-independence, the key-file rules and the registry round trip.
- Synthetic bursts (made-up preamble, the fixed code Manchester-coded at 400 µs) decode identically in the FoxFW C
  decoder (host oracle) and in KAT: the code is reported once per burst; a flipped bit, 49 or 51 bits, and a payload that
  follows the preamble without a resetting gap are rejected by both.
- No change to any file of IMPORTS or the Flipper unit-test corpus.
