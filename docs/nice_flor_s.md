---
layout: default
---

# Nice FloR-S / Nice One Protocol

**Rust module:** `src/protocols/nice_flor_s.rs`
**Reference:** Flipper-ARF garage app `applications/system/subghz_garage/protocols/nice_flor_s.c` (primary;
= ARF `lib/subghz/protocols/nice_flor_s.c` without the custom-button bookkeeping), plus doctor-firmware
`lib/subghz/protocols/nice_flor_s.c` for Nice O-Code (`IC`) keys.

## Overview

Nice FloR-S gate remotes (and the Nice One, which adds 20 bits) send a 52-bit rolling-code frame whose
serial and counter are encrypted with a 32-byte secret "rainbow table". The Flipper firmwares read that
table from `subghz/assets/nice_flor_s`, which ships encrypted with the Flipper's device key, so neither
the references nor KAT have it. KAT reads an optional plaintext copy the user provides:
**`~/.config/KAT/nice_flor_s.bin`, exactly 32 raw bytes** (the first 32 bytes of the asset's decrypted
payload, what the C reads with `subghz_keystore_raw_get_data(file, 0, buffer, 32)`; the cipher indexes it
with `p0 & 0x1F` and `p0 >> 3`). Without it KAT decodes what a Flipper without the asset decodes: every
frame, with serial/button/counter unknown (the C shows 0), and does not transmit (the C would send an
all-zero cipher text).

Displayed as **Nice One** for 72-bit frames and **Nice O-Code** for keys carrying an `IC`, as the C does.

## Timing

| Parameter   | Value    | Notes                                               |
|-------------|----------|-----------------------------------------------------|
| Short       | 500 µs   | ±300 µs                                             |
| Long        | 1000 µs  | ±300 µs                                             |
| Header gap  | 19000 µs | LOW, ±11400 µs (te_short × 38 ± te_delta × 38); TX 18500 |
| Start       | 1500 µs  | HIGH then LOW, ±900 µs each                         |
| Stop bit    | 1500 µs  | HIGH, ±300 µs                                       |
| Bits        | 52 / 72  | exactly                                             |

## Frame Layout

52 bits, MSB first: `btn(4) ‖ P1(4) ‖ enc(44)`.

- **btn:** button 1, 2, 4, 8 (buttons 1..4; the references also offer 3)
- **P1:** `0xF ^ btn ^ n`, n = parcel number 0..15 (16 parcels per press)
- **enc:** `serial(28) ‖ cnt(16)` through the table cipher: two rounds of table lookups XORed over bytes
  1..5 with a byte swap, then an inversion/shuffle of bytes 0..5 (doctor XORs the installer code `IC`
  into bytes 4 and 3 there; IC 0xFFFF = plain FloR-S)

Nice One: 20 more bits `P7(8) ‖ P8hi(4) ‖ crc(8)`, `k = n ^ (n < 4 ? 0x8F : 0x80)`, `P7 = k`,
`P8 = hold ^ (k << 4) | crc >> 4` (`hold` = 0 on the first parcel of a press, 0x10 after), CRC-8 with
reflected polynomial 0x97, init 0xFF over the 64 bits before it.

**KAT packing:** 52-bit frames: `data` = the frame. 72-bit frames: `data` = the first 64 wire bits,
`extra` bits 0..7 = the last 8 (the CRC); `extra` bits 8..15 = `CounterMode`, 16..31 = `IC`, bit 32 =
O-Code flag. `crc_valid` = the Nice One CRC holds (no check exists for 52-bit frames). O-Code: the
serial is XORed with a per-parcel 28-bit mask and the IC with a 16-bit mask (`nice_o_mask`).

## RF

- **Encoding:** PWM (1 = HIGH 1000 + LOW 500, 0 = HIGH 500 + LOW 1000)
- **RF modulation:** AM (OOK), 433.92 / 868 MHz (no frequency filter)
- **Encryption:** rolling code, Nice table cipher (table required)

## Decoder Steps

1. **Reset** — LOW ≈ 19 ms → CheckHeader.
2. **CheckHeader / FoundHeader** — HIGH ≈ 1500, LOW ≈ 1500 → clear the bit buffer.
3. **SaveDuration** — a HIGH ≈ 1500 is the stop bit: report if exactly 52 or 72 bits; any other HIGH is
   kept for the next step.
4. **CheckDuration** — (HIGH, LOW) = (500, 1000) → 0, (1000, 500) → 1; after bit 52 the frame moves to
   `data_2` and the Nice One tail collects from 0.

Every frame is reported (garage copy). doctor-firmware's receiver only reports a 52-bit frame whose low 32
bits equal the previous frame's (its "skip O-Code" filter), i.e. every second parcel, never an O-Code frame.

## Encoder

`deserialize` + `get_upload` (table required): decrypt the key, counter + 1 (`CounterMode` 1: 0xFFFE ↔ 1,
2: 1 ↔ 0), button = the requested native nibble (1..15, else the captured one), then 16 parcels
n = 0..15, each `LOW 18500, HIGH 1500, LOW 1500, 52 bits[, 20 Nice One bits with hold = n], HIGH 1500`;
one copy. O-Code keys re-encrypt each parcel with its mask. (The garage app's TX plugin is built without
its custom-button and firmware-extension flags, so it sends button 0 and the "OFEX" counter
0 → 1 → 0xFFFE → 0xFFFF → 0; KAT does not copy that build artefact.)

## Key file

`Protocol: Nice FloR-S`, `Bit: 52|72`, `Key` (the 52-bit frame), Nice One `Data` (the 20-bit tail,
decimal), doctor's `IC: XX XX`, optional `CounterMode`. The decoder deserialize requires 52/72 bits and,
for 72, `Data`.

## Validation

- Flipper unit tests `nice_flor_s_raw.sub` (10 frames) and `nice_one_raw.sub` (13 frames, all CRC-valid)
  decode in KAT's registry exactly as the garage oracle reports them (keys, `Data`).
- Oracle parity (garage build, `--protocol "Nice FloR-S"`, decoder reset after each decode): identical on
  385 corpus files (unit tests, IMPORTS, KAT uploads; 998 decodes) and 6100 fuzzed files (16959 decodes).
  doctor's copy differs only by its skip-O-Code filter (719 / 11992 decodes).
- Encoder: the C's no-table uploads equal KAT's for 60 key files (52/72 bits, IC, CounterMode) in both
  builds; the table cipher, O-Code masks and Nice One tails match golden vectors from the C compiled with a
  synthetic table; encode→decode round trips cover every counter mode, button and O-Code.
- No rtl_433 recording exists for Nice FloR-S.
