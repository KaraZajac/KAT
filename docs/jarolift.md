---
layout: default
---

# Jarolift Protocol

**Rust module:** `src/protocols/jarolift.rs`
**Reference:** Flipper-ARF garage app `applications/system/subghz_garage/protocols/jarolift.c` (primary) and
doctor-firmware `lib/subghz/protocols/jarolift.c` — same air format, frame, check and upload

## Overview

Jarolift roller-shutter remotes: KeeLoq frames (normal learning with the Jarolift manufacturer key) with KeeLoq's
400/800 µs PWM, but a different header (exactly 8 preamble pulses) and 72 bits: 64 key bits + 8 "grouping" bits.
KAT's KeeLoq decoder never takes them (its 64..66-bit window rejects 72 bits).

The C calls back for every 72-bit frame and zeroes serial/button/counter/group in its display when the decrypted
hop's byte 2 differs from the serial's low byte. Per KAT policy a frame is reported only when that 8-bit check passes
(`crc_valid`).

## Timing

| Parameter | Value    | Notes                                              |
|-----------|----------|----------------------------------------------------|
| Short     | 400 µs   | ± 167 µs                                           |
| Long      | 800 µs   | ± 167 µs                                           |
| Preamble  | 400/400  | exactly 8 pulses counted                           |
| Gap       | 4000 µs  | LOW, ± 1002 µs (the encoder sends 3800)            |
| Bits      | 72       | 1 = HIGH 400 + LOW 800, 0 = HIGH 800 + LOW 400     |
| End       | ≥ 2400 µs | LOW; the last bit comes from its HIGH alone       |

## Frame Layout (72 bits)

- **64 bits**, sent LSB first: `reverse_key(key, 64)` = button (4) | serial (28) | KeeLoq hop (32)
- **8 grouping bits** (channels 8-15)
- **Hop plaintext** (normal learning, serial-derived key): channels 0-7 (8 bits) << 24 | serial[7:0] << 16 |
  counter (16 bits)
- The C's "Group" = `(plaintext >> 24) << 8 | (reverse_key(grouping bits, 8) >> 8)`; the second term is always 0, so
  the received grouping byte never enters it and the encoder sends 0x00 there. KAT mirrors this (no capture shows a
  remote with channels 8-15 to justify otherwise).

Buttons: 1 Learn, 2 Down, 4 Stop, 8 Up (the C's names; others "Unkn").

KAT fields: `data` = the first 64 wire bits (C `generic.data`, key-file `Key`), `extra` = the 8 grouping bits as
received (C `generic.data_2`, key-file `Data`), `data_count_bit` = 72; serial (28 bits), button (4 bits), counter
(16 bits).

## RF

- **Encoding:** PWM
- **RF modulation:** AM (OOK), 433.92 MHz in the C's flags; KAT decodes on every band
- **Encryption:** KeeLoq (normal learning; key = the first keystore entry of type 11, "Jarolift")

## Decoder Steps

1. **Reset** — a HIGH 400 ± 167 µs counts a preamble pulse.
2. **CheckPreambula** — a LOW 400 ± 167 goes back to Reset; with exactly 8 pulses counted a LOW of 4000 ± 1002 µs
   starts the data; anything else resets the count.
3. **SaveDuration / CheckDuration** — HIGH then LOW per bit; the first 64 bits are stored at the 65th LOW; a LOW of
   2400 µs or more ends the frame, adding the last bit from its HIGH: 72 bits → analyse and report when the check passes.

## Encoder

`..._deserialize` + `gen_data` + `get_upload`: the frame is analysed again, counter + 1 (0 past 0xFFFF), button = the
requested native code (4 bits; the C's custom buttons cycle 1/2/4/8, codes above 0xF keep the captured button), hop
re-encrypted (channels 0-7 from the received plaintext, grouping byte 0x00). Upload: LOW 14400, HIGH 1500, LOW 400,
8 × (HIGH 400, LOW 400) with the last LOW 3800, 64 + 8 bits; `encoder.repeat` = 3.

Flipper-ARF's garage TX plugin (`tx_jarolift`) is built without `SUBGHZ_GARAGE_HAS_CUSTOM_BTN` and
`SUBGHZ_GARAGE_HAS_LIB_EXTENSIONS`: their fallbacks make it send button 0 and take the "OFEX" counter path. KAT follows
the source's intended path, which is doctor-firmware's behaviour.

## Key file

`Protocol: Jarolift`, `Bit: 72` (required), `Key` and `Data` (8 bytes, the grouping bits; required). A frame failing
the check is refused.

## Validation

No Jarolift capture is available (no unit-test file). Unit tests: synthetic frames (incl. the C comment's remote
layout: serial 0x06D7841, Up, group 0x0100), check/preamble-count/length rejections, encode → decode for buttons
1/2/4/8, out-of-range button, counter wrap and grouping byte, key-file round trip, button names.

C oracle (doctor and garage): decode parity on the unit-test captures (57 files), IMPORTS (108), UberGuidoZ (1171) and
4200 fuzz files (1655 frames). Encoder: 300 random key files, KAT equals doctor-firmware pulse for pulse; the garage
oracle (TX plugin flags) differs by the button-0/OFEX fallbacks above.
Key files: the 821 the C serialized from those decodes (doctor and garage formats) load in KAT with the C's fields, KAT
writes the garage's lines exactly, and re-encoding them matches doctor-firmware.
