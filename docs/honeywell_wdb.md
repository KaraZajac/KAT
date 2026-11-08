---
layout: default
---

# Honeywell Protocol (wireless doorbells)

**Rust module:** `src/protocols/honeywell_wdb.rs`
**Reference:** `REFERENCES/doctor-firmware/lib/subghz/protocols/honeywell_wdb.c` (`SUBGHZ_PROTOCOL_HONEYWELL_WDB_NAME`
= "Honeywell"; frame layout from klohner's honeywell-wireless-doorbell), registered by doctor-firmware. Flipper-ARF's
`lib` copy (unregistered there), its garage-app copy (not in the garage registry), Momentum's, RogueMaster's and
FoxFW's (encoder in its SD plugin `enc_honeywell_wdb`) have the same feed and upload code; they differ only in the
display text, the hash function and where the Sensors flag sits. Flipper unit tests: `honeywell_wdb_raw.sub`,
`honeywell_wdb.sub` (the same files in ARF, Momentum and RogueMaster).

Not to be confused with [Honeywell Sec](honeywell.md) (`honeywell.c`, the 64-bit security sensors).

## Overview

48-bit fixed code of Honeywell wireless doorbells and PIR motion sensors (RCWL300A, RCWL330A, Series 3 / 5 / 9,
Decor chimes): a 20-bit serial, device type, alert level, flags and a parity bit. Static code.

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| te_short | 160 µs | ± te_delta 60 µs |
| te_long | 320 µs | ± 60 µs |
| Header | LOW 480 µs | 3·te_short, ± 60 µs |
| Trailer | HIGH 480 µs | 3·te_short, ± 60 µs; the next frame's header follows directly |
| Bit 1 / 0 | HIGH 320 + LOW 160 / HIGH 160 + LOW 320 | |
| Bits | 48 | MSB first |

AM (OOK), 433.92 / 315 MHz (`SubGhzProtocolFlag_433 | 315 | AM`).

## Frame Layout (48 bits)

| Bits | Field |
|------|-------|
| 47..28 | serial (20 bits) |
| 21..20 | device type (2 doorbell, 1 PIR motion) |
| 17..16 | alert (0 normal, 1 / 2 high, 3 full volume) |
| 4 | secret knock |
| 3 | relay (a retransmission) |
| 1 | low battery |
| 0 | parity of bits 47..1 |

The other bits are 0 on the remotes klohner tested.

## Decoder Steps

1. Reset: a LOW of |d − 480| < 60.
2. HIGH (saved) + LOW pairs: short + long → 0, long + short → 1 (each within te_delta); anything else → Reset.
3. A HIGH of |d − 480| < 60 where a bit's HIGH is due ends the frame: reported when exactly 48 bits were read and
   bit 0 is the parity of bits 47..1. Back to Reset either way: the trailer's following LOW opens the next frame, so
   every frame of a press is reported.

## Field mapping

`serial` as the C; `button` / `counter` None (the C sets neither); `data` = the 48 bits; `crc_valid` = the parity
holds (always for decodes; a loaded key may fail it); `encoder_capable` true.

## Encoder

`subghz_protocol_encoder_honeywell_wdb_get_upload` (`encoder.repeat` = 3): LOW 480, the 48 bits, HIGH 480 — the key as
stored (the parity is not recomputed). No button to switch.

## Key file

`Protocol: Honeywell`, `Bit: 48`, `Key` (generic serialize; doctor's serializer also writes `Serial` / `Btn` /
`Cnt`, which KAT does not); loading requires `Bit: 48`.

## Validation

Reference oracles `subghz_oracle_{doctor,roguemaster,momentum,foxfw}` and `subghz_oracle_arf --registry all`; the
decode dump is `hb_parity_decode_dump` in `src/protocols/nord_ice.rs`, the encode dump f_common's `F_KEYS`.

- Unit test `honeywell_wdb_raw.sub`: 49 frames through KAT's whole registry, key `00 00 56 A3 70 20 00 00`
  (serial 0x56A37), as every reference reports; nothing else decodes. `honeywell_wdb.sub` loads, encodes and decodes
  back (3 frames). `test_random_raw.sub` gains its 30 Honeywell frames (the doctor oracle's count), nothing else in it
  changes.
- Decoder alone (reset after each report, like `decode --protocol`): identical with all five oracles on 1200 fuzz
  files (300 per protocol of this round: valid frames with jitter, dropped / split / stretched pulses, dropped or
  doubled bits, wrong lengths, wrong parity, mixed codes, durations on the window edges; 324 frames) and on the 243
  RAW files of IMPORTS and the ARF / Momentum / RogueMaster unit tests (237 frames).
- Encoder: 64 random key files (a quarter with a wrong parity, some with bits above 48) — KAT's upload equals
  doctor's, RogueMaster's, Momentum's and ARF's pulse for pulse (FoxFW's encoder is an SD plugin the oracle does not
  load; its source is doctor's).
- `kat_diff` over IMPORTS and the three unit-test folders: only the two Honeywell files and `test_random_raw.sub`
  change.
