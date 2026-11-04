---
layout: default
---

# Alutech AT-4N Protocol

**Rust module:** `src/protocols/alutech_at_4n.rs`
**Reference:** Flipper-ARF garage app `applications/system/subghz_garage/protocols/alutech_at_4n.c`
(primary; doctor-firmware's `lib/subghz/protocols/alutech_at_4n.c` is the same on the wire).

## Overview

Alutech AT-4N gate remotes send a 72-bit frame: 64 encrypted bits and a CRC-8. The cipher is TEA-like
with secret constants (keys, delta, final sum) that the Flipper firmwares read from the "rainbow table"
asset `subghz/assets/alutech_at_4n`, shipped encrypted with the Flipper's device key. KAT reads an
optional plaintext copy: **`~/.config/KAT/alutech_at_4n.bin`, exactly 32 raw bytes** = eight big-endian
32-bit words m0..m7 (`get_magic_data_from_buffer`). Decryption runs `sum = m0` with keys m1/m2 (second
word) and m4/m5 (first word), `sum += m3` until 0; encryption runs `sum += m6` from 0 until m0 (so m6 =
−m3 and m0 is a multiple of m6, 32 rounds for TEA); m7 is unused. A table whose loops would not end within
1024 rounds is ignored. Without a table the frame decodes (CRC checked), the fields stay unknown (the C
shows 0) and KAT does not transmit (the C would send an all-ones cipher block).

## Timing

| Parameter   | Value     | Notes                                          |
|-------------|-----------|------------------------------------------------|
| Short       | 400 µs    | ±140 µs                                        |
| Long        | 800 µs    | ±280 µs                                        |
| Preamble    | > 9 pairs | HIGH 400 + LOW 400                             |
| Header gap  | 4000 µs   | ±1400 µs                                       |
| End         | ≥ 940 µs  | LOW; the last bit is read from its HIGH alone  |
| Bits        | 72        | exactly                                        |

## Frame Layout

Sent LSB first; the C bit-reverses: `d = reverse64(data)`, `crc = reverse8(crc byte)`; the CRC is CRC-8
(poly 0x31, init 0xFF) over `d`'s little-endian bytes. Decrypted, `d` is
`crc1(8) ‖ serial(32) ‖ cnt(16) ‖ btn(8)` with `crc1 = !crc8(cnt & 0xFF)` (poly 0x31) as an inner check.
Buttons: 0xFF, 0x11, 0x22, 0x33, 0x44 = buttons 1..5.

**KAT packing:** `data` = the first 64 bits as received, `extra` bits 0..7 = the CRC byte, bits 8..15 =
`CounterMode`; `data_count_bit` = 72; `crc_valid` = the CRC-8 holds.

## RF

- **Encoding:** PWM (1 = HIGH 400 + LOW 800, 0 = HIGH 800 + LOW 400)
- **RF modulation:** AM (OOK), 433.92 MHz (no frequency filter)
- **Encryption:** TEA-like (table required)

## Decoder Steps

1. **Reset / CheckPreambula** — count HIGH 400 + LOW 400 pairs; after more than 9, a 4000 µs pulse (the
   C does not check its level) starts the data.
2. **SaveDuration / CheckDuration** — (HIGH, LOW) pairs → bits; after bit 64 the first 64 bits move to
   `data_2` and the CRC collects from 0.
3. **End** — a LOW ≥ 940 µs adds the last bit; with exactly 72 bits the frame is reported unless it equals
   the previous frame this decoder reported (the encoder sends 3 copies; this memory survives `reset()`,
   as in the C).

KAT gate: the C also reports frames that fail the CRC (it only skips their decryption); KAT reports only
CRC-valid ones, while its repeat memory follows the C's. Every frame of the unit-test capture passes the
CRC.

## Encoder

`deserialize` + `get_upload` + `gen_data` (table required; the key must pass the inner check): counter + 1
(`CounterMode` 1: 0 → 1 → 0xFFFE → 0xFFFF → 0, ≥ 2: 0..5 cycle), button = the requested native code (one
of the five, else the captured one), re-encrypt, CRC, bit-reverse; upload 12 × (HIGH 400, LOW 400) with the
last LOW 4000, 64 + 8 bits, last LOW + 16000 µs; 3 copies. (The garage TX plugin's build sends button 0
and the "OFEX" counter, a compat-shim artefact KAT does not copy.)

## Key file

`Protocol: Alutech AT-4N`, `Bit: 72`, `Key` (the 64 bits), `CRC` (decimal, required), optional
`CounterMode`.

## Validation

- Flipper unit test `alutech_at_4n_raw.sub`: 4 frames, all CRC-valid, identical to both oracles.
- Oracle parity (`--protocol "Alutech AT-4N"`, reset after each decode): identical to the garage and
  doctor builds on 385 corpus files (48 decodes) and 6100 fuzzed files (734 decodes); 200 single-bit-flipped
  frames are reported by the C with a bad CRC and dropped by KAT.
- Encoder: the C's no-table uploads equal KAT's for 60 key files; the cipher and CRC match golden vectors
  from the C compiled with a synthetic table; encode→decode round trips cover every counter mode and
  button.
- No rtl_433 recording exists for Alutech AT-4N.
