---
layout: default
---

# Treadmill37 Protocol

**Rust module:** `src/protocols/treadmill37.rs`
**Reference:** `REFERENCES/doctor-firmware/lib/subghz/protocols/treadmill37.c` (@xMasterX, 2026, "Treadmill37 (QH-433)";
`SUBGHZ_PROTOCOL_TREADMILL37_NAME` = "Treadmill37"), registered by doctor-firmware. Flipper-ARF's `lib` copy
(unregistered there), its garage-app copy (not in the garage registry), Momentum's, RogueMaster's and FoxFW's
(encoder in its SD plugin `enc_treadmill37`) have the same feed and upload code; they differ only in style. No
reference has a capture of it.

## Overview

37-bit fixed code of QH-433 treadmill remotes: a 20-bit serial, a 16-bit command and a stop bit. No checksum.

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| te_short | 300 µs | ± te_delta 150 µs |
| te_long | 900 µs | ± 150 µs |
| Gap | 6000 µs | 20·te_short; decoder window ± 600 µs (4·te_delta) |
| Bit 1 / 0 | HIGH 900 + LOW 300 / HIGH 300 + LOW 900 | the last bit's LOW is the gap |
| Bits | 37 | MSB first, no preamble |

AM (OOK), 433.92 MHz (`SubGhzProtocolFlag_433 | AM`).

## Frame Layout (37 bits)

| Bits | Field |
|------|-------|
| 36..17 | serial (`data >> 17`) |
| 16..1 | command (the C keeps it in `generic.cnt` and displays it as `Btn: %04lX`) |
| 0 | stop bit (0) |

The C's six key samples (serial 0xC0000): commands 0x0C18, 0x030F, 0x0A16, 0x0612, 0xAAB6, 0x99A5 — each low byte
is its high byte + 0x0C, which the C does not check.

## Decoder Steps

1. Reset: a LOW of |d − 6000| < 600.
2. HIGH (saved) + LOW pairs within te_delta: short + long → 0, long + short → 1.
3. A gap-window LOW closes the frame: the last bit is read from its HIGH alone (≈300 → 0, ≈900 → 1) and the frame
   is reported when exactly 37 bits were read. Back to Reset: the gap that closed a frame does not open the next
   one, so a held button reports every other frame.

## Field mapping

`serial` as the C; `counter` = the 16-bit command (the C's `generic.cnt`; KAT's `button` is 8 bits); `button`
None; `data` = the 37 bits; `crc_valid` false (no checksum); `encoder_capable` true.

## Encoder

`subghz_protocol_encoder_treadmill37_get_upload` (`encoder.repeat` = 3): the 37 bits, the last LOW being the 6000 µs
gap. The captured code is sent (no button override).

## Key file

`Protocol: Treadmill37`, `Bit: 37`, `Key` (generic serialize; doctor's serializer also writes `Serial` / `Btn` /
`Cnt`, which KAT does not); loading requires `Bit: 37`.

## Validation

Reference oracles `subghz_oracle_{doctor,roguemaster,momentum,foxfw}` and `subghz_oracle_arf --registry all`; the
decode dump is `hb_parity_decode_dump` in `src/protocols/nord_ice.rs`, the encode dump f_common's `F_KEYS`.

- Decoder alone (reset after each report): identical with all five oracles on 1200 fuzz files (300 per protocol of
  this round: frames with jitter, dropped / split / stretched pulses, dropped or doubled bits, 35..39-bit frames,
  timings 260..340 / 820..980 µs, gaps 5300..6700 µs, mixed codes; 131 frames) and on the 243 RAW files of IMPORTS
  and the ARF / Momentum / RogueMaster unit tests (none decodes).
- Encoder: 64 random key files (some with bits above 37) — KAT's upload equals doctor's, RogueMaster's, Momentum's
  and ARF's pulse for pulse (FoxFW's SD-plugin encoder has doctor's source).
- `kat_diff` over IMPORTS and the unit-test folders: no Treadmill37 frame anywhere.
