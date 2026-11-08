---
layout: default
---

# Magellan Protocol

**Rust module:** `src/protocols/magellan.rs`
**Reference:** `REFERENCES/doctor-firmware/lib/subghz/protocols/magellan.c` (the stock Flipper protocol, registered by
doctor, Momentum, RogueMaster and FoxFW). Flipper-ARF's `lib` and garage-app copies (both unregistered), Momentum's,
RogueMaster's and FoxFW's (encoder in an SD plugin, `protocols_ext/enc_magellan.c`) decode, check and encode
identically; every copy has the encoder (`SubGhzProtocolFlag_Send`). They differ in style and in the display (doctor
/ Momentum / RogueMaster spell out the event; ARF shows `SN:0x… Btn:…`). ARF's `lib` copy turns on the custom-button
D-pad but re-sends the captured frame on every direction ("the embedded CRC8 … would need recomputation for a
different event code"): no copy changes the event. Flipper unit tests: `magellan_raw.sub`, `magellan.sub`.

## Overview

Paradox Magellan wireless security devices — door contacts, motion and smoke detectors, REM1 / REM2 remotes: a
32-bit frame of a 16-bit serial, an 8-bit event and a CRC-8.

## Timing / modulation

| Element | Pulses | Decoder windows |
|---------|--------|-----------------|
| Preamble | HIGH 800, LOW 200; 12 × (HIGH 200, LOW 200); HIGH 200, LOW 400 | pairs 200 ± 100; more than 10 needed before the 200 + 400 ± 200 pair |
| Start bit | HIGH 1200, LOW 400 | 1200 ± 300, 400 ± 200 |
| Bit 1 | HIGH 200, LOW 400 | ± 100 each |
| Bit 0 | HIGH 400, LOW 200 | ± 100 each |
| Stop bit | HIGH 200, LOW 40 000 | a LOW ≥ 1200 after a bit's HIGH |
| Bits | 32 | MSB first |

AM (OOK), 433.92 MHz (`SubGhzProtocolFlag_433 | AM`, Sensors).

## Frame Layout

| Bits | Field |
|------|-------|
| 31..8 | 24 bits sent bit-reversed: reversed, bits 23..16 = event, bits 15..0 = serial |
| 7..0 | CRC-8 (poly 0x31, init 0) of bytes 31..24, 23..16, 15..8 |

Event (`subghz_protocol_magellan_get_event_serialize`): high nibble = type (0 Nothing, 1 Door, 2 Motion, 3 Smoke
Alarm, 4 REM1, 5 "REM1 - Off1", 6 "REM2 - Off1", others Unknown); low nibble = status: for types 0–3 Sealed / Alarm
/ Tamper / Alarm + Tamper (0–3), for remotes Arm1 / Btn1 / Btn2 / Btn3; 8 Reset, 9 LowBatt, A BattOk, B Learn, others
Unknown. Example (the C's comment): 0x37AE4828 → 0x37AE48 reversed = 0x1275EC → event 0x12 "Door - Tamper", serial
0x75EC (shown as 117 236).

## Decoder Steps

1. Reset: a HIGH 200 ± 100.
2. Preamble: HIGH (saved) + LOW pairs; 200 + 200 counts, 200 + 400 ± 200 after more than 10 counted pairs ends the
   preamble; anything else resets.
3. Start bit: HIGH 1200 ± 300 + LOW 400 ± 200.
4. Bits: HIGH 200 ± 100 + LOW 400 ± 100 = 1, HIGH 400 ± 100 + LOW 200 ± 100 = 0.
5. A LOW ≥ 1200 µs after a bit's HIGH ends the frame: exactly 32 bits with a valid CRC are reported. Every copy
   decodes (each has its own preamble).

## Field mapping

`serial` and `button` (= the event, labelled "Door - Alarm + Tamper" etc. in the button column) as the C's
`check_remote_controller`; `counter` none; `data` = the 32 bits; `crc_valid` = the CRC holds (always for decodes;
loaded keys are checked); `encoder_capable` true.

## Encoder

`subghz_protocol_encoder_magellan_get_upload` (`encoder.repeat` = 3): preamble, start bit, the 32 bits of the key
as given (its CRC included), stop bit. Every reference re-sends the captured frame; so does KAT ("Send button…"
lists nothing).

## Key file

`Protocol: Magellan`, `Bit: 32` (required), `Key`.

## Add Manually

None of the references offers a Magellan type.

## Validation

Host builds of the references' C (`REFERENCES/tools/oracle/bin/subghz_oracle_{doctor,arf,momentum,roguemaster,foxfw}`):
- `magellan_raw.sub` (ARF, Momentum and RogueMaster unit-test folders): 8 frames through KAT's whole registry, all
  0x37AEC852 (event 0x13 "Door - Alarm + Tamper", serial 0x75EC), and nothing else; `test_random_raw.sub` adds the
  same 8 frames, as the C's random test; `magellan.sub` loads and its upload decodes back three times, as the C.
- Encoder: 64 key files (48 events of every type and status with a valid CRC, 16 random frames) give the same
  upload, pulse for pulse, in KAT and in doctor's, ARF's, Momentum's and RogueMaster's C.
- Decoder: 200 fuzz files (repeats, mixed codes, wrong CRCs, wrong lengths, 9..20 preamble pairs, other protocols'
  frames, noise; jitter, dropped, split and stretched pulses) give the same 186 frames in KAT's decoder and in all
  five references; on the 304 real RAW files and the other protocols' fuzz files it decodes only its own captures.
- Registry: the fuzz streams decode the same through KAT's whole registry as through doctor's.

Parity tools: `src/protocols/f_common.rs` (`F_FILES`, `F_KEYS`).
