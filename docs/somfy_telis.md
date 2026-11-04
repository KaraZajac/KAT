---
layout: default
---

# Somfy Telis Protocol

**Rust module:** `src/protocols/somfy_telis.rs`
**Reference:** `REFERENCES/Flipper-ARF/applications/system/subghz_garage/protocols/somfy_telis.c` (Flipper-ARF's
garage app, primary). Flipper-ARF's and doctor-firmware's `lib/subghz/protocols/somfy_telis.c` decode and transmit the
same frames (the copies differ in display text and build shims only). Frame description:
pushstack.wordpress.com/somfy-rts-protocol.

## Overview

Somfy RTS (Radio Technology Somfy) remotes for roller shutters, blinds, awnings and gate motors — Telis, Situo,
Smoove … — on 433.42 MHz (868 MHz in some markets). A 56-bit frame: key byte, control (button) nibble, checksum
nibble, 16-bit rolling code and 24-bit address, obfuscated by an XOR chain. No secret key: the references decode the
code and send the next one.

## Timing

| Parameter       | Value              | Notes                                                |
|-----------------|--------------------|------------------------------------------------------|
| te_short        | 640 µs             | ±250 µs (strict `<`), half bit                       |
| te_long         | 1280 µs            | ±250 µs                                              |
| Wake-up         | HIGH 9415 + LOW 89565 µs | first frame of the upload only                 |
| Hardware sync   | HIGH/LOW 2560 µs   | ±1000 µs; 2 pairs in the first frame, 7 in repeats   |
| Software sync   | HIGH 4550 + LOW 640 µs | decoder: HIGH 4480 ± 1000 after ≥ 2 sync pairs   |
| Frame end       | ≥ 1530 µs LOW      | 30415 µs inter-frame silence in the upload           |

## Frame Layout

Received bytes `f0..f6` (56 bits, MSB first) are the plain bytes `p` chained: `f[i] = p[i] ^ f[i-1]`, so
`plain = data ^ data >> 8`:

| Plain bits | Field                                                                            |
|------------|----------------------------------------------------------------------------------|
| 55..48     | key byte (0xA7 in the examples; some remotes count in the low nibble)            |
| 47..44     | control = button: 1 My, 2 Up, 3 My+Up, 4 Down, 5 My+Down, 6 Up+Down, 8 Prog, 9 Sun+Flag, A Flag |
| 43..40     | checksum: XOR of every nibble of the plain frame (checksum nibble cleared)       |
| 39..24     | rolling code (16 bits)                                                           |
| 23..0      | address, as stored (`data & 0xFFFFFF`; rtl_433 prints it byte-swapped)          |

`data` = the 56-bit received frame (the references' `Key`), `data_count_bit` = 56; `serial` = address, `button` =
control, `counter` = rolling code; `crc_valid` = the checksum holds (always, for decoded frames).

## RF

- **Encoding:** Manchester (1 = LOW then HIGH)
- **RF modulation:** AM (OOK); flags 433 / 868 MHz; KAT decodes on every band
- **Encryption:** rolling code (XOR-chain obfuscation, 4-bit checksum, no key)
- **Make:** Somfy

## Decoder Steps

1. **Reset** — HIGH 2560 ± 1000 → FoundPreambula, `header_count` + 1.
2. **FoundPreambula** — LOW 2560 ± 1000 → CheckPreambula; anything else clears `header_count` → Reset.
3. **CheckPreambula** — HIGH 2560 ± 1000: next sync pair; HIGH 4480 ± 1000 with `header_count > 1`: data starts
   (Manchester primed Reset + LongHigh, bit count and `header_count` cleared); other pulses leave it waiting.
4. **DecoderData** — short/long pulses feed Flipper's Manchester machine; a LOW ≥ 1530 µs ends the frame: exactly 56
   bits whose checksum holds are reported (the C's gate); then Reset. Other pulses → Reset.
5. `reset` (C and KAT) rewinds only the step and the Manchester state; `header_count` survives.

## Encoder

`subghz_protocol_somfy_telis_gen_data` + `_get_upload`: fields from the frame; button = KAT's native code 0x0–0xF (the
references' button editor takes any 4-bit value, their D-pad 1/2/4/8; anything else keeps the captured button);
rolling code + 1 (0xFFFF → 0); key byte and address kept; checksum and XOR chain rebuilt. Upload: wake-up, 2 sync pairs,
software sync, 56 bits, LOW 30415; twice: 7 sync pairs, software sync, 56 bits, LOW 30415 — repeated
`encoder.repeat` = 3 times.

**Garage build.** The garage app's TX plugin is built without `SUBGHZ_GARAGE_HAS_LIB_EXTENSIONS` /
`SUBGHZ_GARAGE_HAS_CUSTOM_BTN`: as shipped it sends button 0 and steps the counter in "OFEX" mode (≥ 1 → 0xFFFE). KAT
follows the source's normal mode (Flipper-ARF firmware, doctor-firmware).

## Key file

`Protocol: Somfy Telis`, `Bit: 56`, `Key` (garage/ARF `serialize`; doctor-firmware also writes `Serial`/`Btn`/`Cnt`).
Loading needs `Bit: 56`. (The references flag the protocol without `Load`, a flag no code checks.)

## Validation

- **Decoder:** identical to doctor and garage on 57 unit-test RAW captures, 1136 UberGuidoZ files, the rtl_433 bursts,
  4786 fuzz files and 1000 bit-error files (4992 decodes; corrupt frames rejected by both).
- **Encoder:** pulse-exact with doctor and Flipper-ARF lib on 240 random keys (counter edges, bad checksums), and with
  the garage build on 60 keys where its button shim and OFEX step agree with the normal mode.
- **Real captures:** unit-test `somfy_telis_raw.sub` (20 decodes, address 0x121300, counters 0x208–0x20F). rtl_433
  `somfy/situo_5_rts_pure_ii/01/g006` → address 0xAAC437 (rtl_433 id 3654826 = 0x37C4AA byte-swapped), Down (4),
  counter 160, 6 frames; `g015` → 0xABC437, Down counter 9 (3 frames), Up+Down (6) counter 10 (7 frames) — rtl_433's
  messages one for one. rtl_433 `Somfy-RTS/codes_test.txt` (bit strings, rebuilt at 640 µs): address 0x4A15C5 (rtl_433
  12916042 byte-swapped), counters 1086/1088/1089/1091; that remote fixes the control nibble at 0xF and puts the
  button in the key byte's low nibble (8/5/6/C), which rtl_433 decodes as a quirk — the references (and KAT) report
  button 0xF.
- No decode on other protocols' unit-test files or IMPORTS.
