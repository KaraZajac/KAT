---
layout: default
---

# Feron Protocol

**Rust module:** `src/protocols/feron.rs`
**Reference:** `REFERENCES/doctor-firmware/lib/subghz/protocols/feron.c` (@xMasterX, 2025; the stock Flipper protocol,
registered by doctor, Momentum, RogueMaster and FoxFW). Flipper-ARF's `lib` and garage-app copies (both
unregistered), Momentum's, RogueMaster's and FoxFW's (encoder in an SD plugin, `protocols_ext/enc_feron.c`) decode
and encode identically; they differ only in style and in the display text (doctor / Momentum / RogueMaster:
`Serial: 0x…` and `Command: 0x…`; ARF: `SN:0x…`). Flipper unit tests: `feron_raw.sub`, `feron.sub`.

## Overview

32-bit fixed code of Feron LED-light remotes: a 16-bit serial and a 16-bit command (in the C's samples a command
byte followed by its complement: ON `857A`, OFF `847B`, brightness up `8679` / down `8778`, scroll mode `827D`,
colours `708F` / `1AE5` / `40BF`). No checksum.

## Timing / modulation

| Parameter | Value | Notes |
|-----------|-------|-------|
| Bit 0 | HIGH 350 + LOW 750 µs | ± 150 µs each |
| Bit 1 | HIGH 750 + LOW 350 µs | ± 150 µs each |
| End marker | LOW 500 + HIGH 500 µs | replaces the last bit's LOW; LOW window 351..649 µs. KAT also takes a marker whose LOW fits the last bit's cell (below) |
| Gap | LOW 4500 µs (6·te_long) | window 3901..5099 µs |
| Bits | 32 | MSB first |

AM (OOK), 433.92 MHz (`SubGhzProtocolFlag_433 | AM`, Sensors).

## Frame Layout

| Bits | Field |
|------|-------|
| 31..16 | serial |
| 15..0 | command (shown by the C, not stored as a button) |

## Decoder Steps

1. Reset: a LOW of |d − 4500| < 600.
2. HIGH (saved) + LOW pairs: HIGH 350 ± 150 + LOW 750 ± 150 = 0, HIGH 750 ± 150 + LOW 350 ± 150 = 1.
3. Otherwise a LOW of |d − 500| < 150 is the end marker: the saved HIGH adds its bit (≈ 350 → 0, ≈ 750 → 1), and a
   frame of exactly 32 bits is reported. Anything else resets.
4. Back to Reset: the marker's HIGH and the gap after it open the next frame, so every copy after the first decodes.

The marker is tried after the two bit cells, as in the C: after a last HIGH of 750 µs a marker LOW under 500 µs is
read as a 1 cell, and after a last HIGH of 350 µs a LOW over 600 µs is read as a 0 cell. In the C the frame is then
lost, and the one after it too (the gap goes to the cell check, so the next copy has no sync).

**KAT's addition (not in the C).** With 32 whole cells read, a HIGH of 500 ± 150 µs followed by a LOW of 3901 µs or
more is the marker's HIGH and the gap: the frame is reported. A gap of the usual length (within 600 µs of 4500)
opens the next frame; a longer LOW ends this one and opens none. A marker of exactly 500 µs sits on the edge of the
1 cell's LOW window (350 ± 150): a real remote sends 516 to 556 µs (`feron_raw.sub`), but one copy of that capture
measures 498 µs, and rtl_433's recordings (`rtl_433_tests/tests/feron/01`) measure 492 to 496 µs. A longer frame of
such cells (KeeLoq's 66 bits) has no such end and is not read.

## Field mapping

`serial` = data >> 16 (`subghz_protocol_feron_check_remote_controller`); button and counter: none (the C stores no
button; the command is the key's low 16 bits); `data` = the 32 bits; `crc_valid` false; `encoder_capable` true.

## Encoder

`subghz_protocol_encoder_feron_get_upload` (`encoder.repeat` = 3): the 32 bit pairs, the last one's LOW replaced by
LOW 500, HIGH 500, LOW 4500. The captured code is sent (the C has no button override); the first copy has no gap
before it, so a receiver decodes copies 2 and 3.

## Key file

`Protocol: Feron`, `Bit: 32` (required), `Key`.

## Add Manually

None of the references offers a Feron type.

## Validation

Host builds of the references' C (`REFERENCES/tools/oracle/bin/subghz_oracle_{doctor,arf,momentum,roguemaster,foxfw}`):
- `feron_raw.sub` (ARF, Momentum and RogueMaster unit-test folders): 103 Feron frames through KAT's whole registry,
  all 0x6338847B (serial 0x6338), and nothing else; the five references read 102 (the copy with the 498 µs marker
  is the one they lose). `feron.sub` loads and its upload decodes back twice.
- rtl_433's recordings (`rtl_433_tests/tests/feron/01`, three presses of two remotes): KAT reads all three with
  rtl_433's id and command (`1A2B` ON ×3, `1A2B` OFF ×2, `C0DE` brighter ×2). The references read the ON press
  alone: the other two codes end in a 1 bit.
- Encoder: 60 random key files give the same upload, pulse for pulse, in KAT and in doctor's, ARF's, Momentum's
  and RogueMaster's C.
- Decoder: 400 fuzz files (1 to 3 codes of 1 to 5 copies; marker LOW 340 to 800 µs, marker HIGH 300 to 700 µs, gap
  3800 to 20000 µs; wrong lengths; jitter, dropped, split and stretched pulses, timing scaled ±15 %) through the
  decoder alone: the five references read the same 435 frames; KAT reads those 435 and 141 more, each a code that
  was sent in that file, and no frame is read by the C alone.
- Real recordings: on 262 RAW files (IMPORTS, the unit-test folders, rtl_433's recordings) KAT's registry and
  doctor's give the same Feron frames but for the ones above (KAT 110, doctor 105).

Parity tools: `src/protocols/f_common.rs` (`F_FILES`, `F_KEYS`).
