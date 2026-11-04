---
layout: default
---

# Somfy Keytis Protocol

**Rust module:** `src/protocols/somfy_keytis.rs`
**Reference:** `REFERENCES/Flipper-ARF/applications/system/subghz_garage/protocols/somfy_keytis.c` (Flipper-ARF's
garage app, primary). Flipper-ARF's and doctor-firmware's `lib/subghz/protocols/somfy_keytis.c` decode and transmit the
same frames; their encoders add a D-pad swap of buttons 0x4 ↔ 0x3 (Key_1 ↔ Prog), which KAT's native-button
parameter covers.

## Overview

Somfy RTS garage and gate remotes (Keytis): the Somfy Telis frame (key, button, checksum, rolling code, address) followed
by a 24-bit press duration counter, 80 bits on the air. No secret key: the references decode the code and send the next
one.

## Timing

Same as [Somfy Telis](somfy_telis.md): te_short 640 / te_long 1280 µs (±250), hardware sync pairs HIGH/LOW 2560 µs
(12 in the first frame, 6 in repeats), software sync HIGH 4550 + LOW 640 µs, frame end ≥ 1530 µs LOW (1920 µs between
the frames of one upload, 30415 µs after the last).

## Frame Layout

| Wire bits | Field                                                                                   |
|-----------|-----------------------------------------------------------------------------------------|
| 0..55     | Telis-format frame, XOR-chained (`plain = data ^ data >> 8`): key nibble 0xA, **button** (plain 51..48: 3 Prog, 4 Key_1), ctrl nibble 0xF, checksum, 16-bit rolling code, 24-bit address |
| 56..79    | press duration counter: `11 cnt4 00 0000 cnt8 crc4` (0xC40019 on the first parcel of a press, then 0xC80026, …; display only) |

`data` = the 56-bit frame (the references' `Key`), `extra` = the press duration counter (their `Duration_Counter`),
`data_count_bit` = 80; `serial` = address, `button` = plain bits 51..48, `counter` = rolling code.

## RF

- **Encoding:** Manchester (1 = LOW then HIGH)
- **RF modulation:** AM (OOK); flags 433 / 868 MHz; KAT decodes on every band
- **Encryption:** rolling code (XOR-chain obfuscation, 4-bit checksum, no key)
- **Make:** Somfy

## Decoder Steps

As Somfy Telis, except:

- `header_count` is not cleared when the data starts (only a broken sync pair clears it), so a repeat needs only one
  sync pair after the first frame;
- bits 0–55 go to the frame, bits 56+ to the press duration counter; a frame needs exactly 80 bits.

**Checksum (deviation).** The C tests the checksum of `generic.data` — the previously reported frame (0 at start) —
instead of the frame just received (`somfy_telis.c` tests `decoder.decode_data`). It therefore reports any 80-bit frame
after a valid one, and after reporting a corrupt frame it reports nothing more until the app restarts. KAT tests the
frame being decoded: on streams of intact frames (every real capture, every encoder upload) both report the same frames;
on 1000 bit-error streams the C reported 185 corrupt frames that KAT rejects and then dropped 477 intact frames that KAT
reports.

## Encoder

`subghz_protocol_somfy_keytis_gen_data` + `_get_upload`: fields from the frame; button = KAT's native code 0x0–0xF
(anything else keeps the captured button); rolling code + 1 (0xFFFF → 0); the frame is rebuilt as key nibble 0xA,
button, ctrl nibble 0xF, checksum, counter, address (the C forces both nibbles); the press duration counter is always
0xC40019. Upload: wake-up HIGH 9415 + LOW 89565, 12 sync pairs, software sync, 56 + 24 bits, LOW 1920; twice: 6 sync
pairs, software sync, 80 bits, LOW 1920; the last LOW is stretched by 30415 − 1920 — repeated `encoder.repeat` = 3
times.

**Garage build.** The garage app's TX plugin is built without `SUBGHZ_GARAGE_HAS_LIB_EXTENSIONS`, which steps the counter
in "OFEX" mode (≥ 1 → 0xFFFE). KAT follows the source's normal mode (Flipper-ARF firmware, doctor-firmware).

## Key file

`Protocol: Somfy Keytis`, `Bit: 80`, `Key` (56-bit frame), `Duration_Counter: <decimal>` (garage/ARF `serialize`;
doctor-firmware also writes `Serial`/`Btn`/`Cnt` and a `Seed` copy of the counter). Loading needs `Bit: 80` and
`Duration_Counter`.

## Validation

- **Decoder:** identical to doctor and garage on 57 unit-test RAW captures, 1136 UberGuidoZ files, the rtl_433 bursts and
  4786 fuzz files; on the 1000 bit-error files the only differences are the checksum deviation above.
- **Encoder:** pulse-exact with doctor, Flipper-ARF lib (240 random keys, counter edges, bad checksums) and the garage
  build (60 keys where its OFEX step agrees with the normal mode).
- **Real capture:** unit-test `somfy_keytis_raw.sub` → 5 frames, address 0x36D3CD, Key_1 (4), counter 0x44, press
  duration counters 0xC40019, 0xC80026, 0xCC0033, 0xD00049, 0x940058 (as the oracles).
- No decode on other protocols' unit-test files or IMPORTS.
