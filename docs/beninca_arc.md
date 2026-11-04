---
layout: default
---

# Beninca ARC Protocol

**Rust module:** `src/protocols/beninca_arc.rs` (AES-128 in the same module)
**Reference:** Flipper-ARF garage app `applications/system/subghz_garage/protocols/beninca_arc.c` (primary) and
doctor-firmware `lib/subghz/protocols/beninca_arc.c` — identical air format, decryption and upload

## Overview

Beninca "ARC" gate and garage remotes: a 128-bit frame that is one AES-128 block. The key comes from the keystore
(entry type 9, "Beninca_ARC"). The plaintext carries the remote's serial, the button, a 5-byte "mini counter" that
rotates while the button is held, a 32-bit press counter and the constant 0xAA55.

The C calls back for every 128-bit frame and only displays the decrypted constant ("Fx"). Per KAT policy a frame is
reported only when it decrypts to 0xAA55 (`crc_valid`; a random frame passes 1 time in 65536).

## Timing

| Parameter | Value   | Notes                                         |
|-----------|---------|-----------------------------------------------|
| Short     | 300 µs  | ± 155 µs                                      |
| Long      | 600 µs  | ± 155 µs                                      |
| Gap       | 9600 µs | LOW, ± 2325 µs (te_long·16 ± te_delta·15)     |
| Bits      | 128     | bit 1 = HIGH 300 + LOW 600, bit 0 = HIGH 600 + LOW 300 |
| Stop bit  | 300 µs  | HIGH after the 128th bit (where the frame is reported) |

## Frame Layout (128 bits)

Wire bytes 0..15 (MSB first), each byte bit-reversed, form the AES-128 ciphertext block (ECB).

AES key: the type-9 entry's 64-bit key as 8 big-endian bytes followed by the same 8 bytes little-endian (the C
byte-swaps both 32-bit halves and swaps them).

| Plaintext bytes | Field                                      |
|-----------------|--------------------------------------------|
| 0..3            | serial (32 bits, big-endian)               |
| 4               | button                                     |
| 5..9            | mini counter (40 bits)                     |
| 10..13          | counter (32 bits, big-endian)              |
| 14..15          | constant 0xAA55                            |

KAT fields: `data` = the first 64 wire bits (C `generic.data`, key-file `Key`), `extra` = the last 64 (C
`generic.data_2`, key-file `Data`), `data_count_bit` = 128; serial (32 bits), button (8 bits), `counter` = the
low 16 bits of the 32-bit counter.

## RF

- **Encoding:** PWM
- **RF modulation:** AM (OOK), 433.92 MHz in the C's flags; KAT decodes on every band
- **Encryption:** AES-128

## Decoder Steps

1. **Start** — wait for a LOW of 9600 ± 2325 µs.
2. **HighLevel** — store the HIGH; at 64 bits store the first half (unless it is all zero, as in the C); at 128 bits
   the frame is complete: decrypt and report it when the constant is 0xAA55. A LOW here goes back to Start.
3. **LowLevel** — classify the HIGH/LOW pair (the bit-1 test first); anything else goes back to Start.

## Encoder

`subghz_protocol_encoder_beninca_arc_deserialize` + `prepare_packets`: decrypt the frame, counter + 1 (32-bit wrap),
button = the requested native code (any 8-bit value; the C's custom buttons cycle 0x02/0x04/0x00), constant kept, then
three packets encrypted with mini counter 2, 4 and 6, each 128 bits + a 300 µs stop HIGH + a 9000 µs LOW.
`encoder.repeat` = 1. A frame without the constant (key files only) is not re-encoded.

Flipper-ARF's garage TX plugin (`tx_beninca_arc` in `application.fam`) is built without
`SUBGHZ_GARAGE_HAS_CUSTOM_BTN`, whose fallback makes `get_btn_code()` return 0: on a device it sends button 0x00.
KAT keeps the captured (or requested) button, as doctor-firmware and the garage source's custom-button path do.

## Key file

`Protocol: Beninca ARC`, `Bit: 128` (required), `Key` (8 bytes, first half) and `Data` (8 bytes, second half,
required); the C's decoder serialize writes exactly these. Frames that do not decrypt to the constant are refused.

## Validation

No capture of a Beninca ARC remote is available (no unit-test file either). Unit tests: FIPS-197 AES vectors (B and
C.1, key expansion), synthetic frames, constant/flipped-bit/gap/timing rejections, encode → decode for buttons
0x02/0x04/0x00 with the counter wrap, key-file round trip, registry attribution.

C oracle (`subghz_oracle_doctor` and `subghz_oracle_garage`): decode parity on Flipper's unit-test captures (57
files), the IMPORTS car captures (108), the UberGuidoZ collection (1171) and 4200 fuzz files (encoder uploads with
jitter, dropped/split pulses, scaled timing, splices, random streams): every frame the C decodes with the 0xAA55
constant is decoded by KAT with the same fields (1560 frames); 3 C frames without the constant are not reported.
Encoder: 300 random key files, KAT's upload equals doctor-firmware's pulse for pulse; the garage oracle (built with the
TX plugin's flags) differs only by the button-0 fallback above.
Key files: the 770 the C serialized from those decodes load in KAT with the C's fields (the 2 without the constant are
refused) and KAT writes the garage's lines exactly; re-encoding them matches doctor-firmware.

In KAT's single registry (the Flipper runs gate protocols in a separate app), Mitsubishi V0 — a car decoder registered
earlier and gated on nothing — claims Beninca frames whose timing is 12–20 % fast (fuzz: uploads scaled ×0.8/×0.88), and
Kia V5 occasionally one scaled ×1.12/×1.25; nominal-timing frames are always reported as Beninca ARC.
