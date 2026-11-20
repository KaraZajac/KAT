---
layout: default
---

# Kia V6 Protocol

**Rust module:** `src/protocols/kia_v6.rs`  
**Reference:** `REFERENCES/ProtoPirate/protocols/kia_v6.c` (Flipper-ARF / doctor-firmware: same frame;
ARF names the buttons)

## Overview

Kia Manchester keyfob at 200/400 µs carrying an AES-128 block. 144 bits in three parts (64 + 64 + 16), each
stored inverted. Part 1's top byte pair carries the Fx field; the rest is the AES block (key = KIA V6
keystores A and B, types 11/12, XORed with fixed masks).

## Timing

| Parameter | Value  | Notes   |
|-----------|--------|---------|
| Short     | 200 µs | ±100 µs |
| Long      | 400 µs | ±100 µs |
| Preamble  | ≥ 601 short LOWs | then long LOW + long HIGH = bits 1101 |
| Min bits  | 144    |         |

## Frame Layout (decrypted block)

byte 0 Fx, 4–6 serial (24 bits), 7 button, 8–11 counter (32 bits), 12 = AES S-box[counter LSB],
15 = CRC-8 (poly 0x07, init 0xFF) of bytes 0–14 (the C also accepts a CRC that differs in its lowest bit);
bytes 1–3, 13 and 14 are 0 in the encoder's frames. A real fob can differ: in `IMPORTS/KIA/KIA V6 N2.sub`
byte 0 is Fx ^ 1. The C reports every 144-bit frame; `crc_valid` carries its CRC check.

`data` = part 1 (the C's saved Key); `counter` = the counter's low 16 bits; `extra` = Fx (bits 0–7), the
counter's high 16 bits (bits 8–23), plaintext byte 0 XOR Fx (bits 24–31) and stored XOR computed CRC
(bits 32–39). The last two are 0 for the encoder's frames; a key file needs them to rebuild parts 2 and 3.
Buttons (ARF): 1 Lock, 2 Unlock, 3 Trunk, 4 Panic.

## RF

- **Encoding:** Manchester, common.rs port with the C's sense `level ? ShortHigh : ShortLow`
- **RF modulation:** FM
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder Steps (`kia_protocol_decoder_v6_feed`)

1. **Reset** — a short HIGH starts the preamble; Manchester state Mid1.
2. **WaitFirstHigh** — count short LOWs (HIGHs ignored); a long LOW after ≥ 601 → WaitLongHigh.
3. **WaitLongHigh** — a HIGH (short or long, as in the C: only the LOW before it must be long) completes
   the sync, adds `1101`.
4. **Data** — Manchester-decode, storing parts 1/2 inverted at bits 64/128; at 144 bits part 3, AES-decrypt,
   report. Any out-of-tolerance pulse restarts.

KAT's earlier port used a hand-written Manchester table and kept only 16 counter bits; it reproduced none of
the C's frames.

## Encoder

Supported (`kia_protocol_encoder_v6_build_upload`): AES-encrypt the fields (Fx from `extra`, counter + 1
over all 32 bits), then the message twice — 640- and 38-pair preambles, each LOW 200, HIGH 400, LOW 200, the
141 inverted bits as Manchester (1 → LOW, HIGH; 0 → HIGH, LOW) and LOW 400; adjacent equal levels merged.
`extra` bits 24–39 are ignored (the plaintext is built as the C builds it). KAT's generic button maps to Lock 1,
Unlock 2, Trunk 3, Panic 4; anything else keeps the captured button. KAT sends the upload 3 times per press
(doctor-firmware's and Flipper-ARF's count; ProtoPirate's encoder sends 10), or as often as a key file's
`Repeat` line says.

## Key files

ProtoPirate's layout (`kia_protocol_decoder_v6_serialize`): `Kia V6`, Bit 144, Key = part 1 (one 16-digit
word), then Serial, Btn, Cnt (32 bits), Key_2 / Key_3 = part 2's low / high half, Key_4 = part 3, Fx, all
decimal. KAT keeps part 1 and the decrypted fields, not parts 2 and 3: it re-encrypts them from the fields and
the `extra` bits 24–39, so a frame whose bytes 1–3, 12, 13 or 14 are not what the encoder builds cannot be
saved. Loading also reads Flipper-ARF's older `KIA/HYU V6` layout of the IMPORTS key files (Key_2 = part 2,
Key_3 = part 3, as hex bytes). Serial, Btn, Cnt or Fx that differ from the decrypted fields rebuild the
frame, as ProtoPirate's encoder transmits from them.

## Validation

C oracle: 500 random frames (random Fx and 16/32-bit counters) — uploads identical pulse for pulse; the
decoder reports the C's 500 frames (key, serial, button, 32-bit counter, Fx). The second (38-pair) message
is below the decoder's 601-pair preamble gate in both. Recordings: the RAW captures
`IMPORTS/KIA/more/kia_v6.sub`, `kia_v6_DXL_2_10.sub`, `kia_v6_KD_2_10.sub` and `IMPORTS/HYUNDAI/more/Tucson2.sub`
decode as Kia V6; the key files `IMPORTS/HYUNDAI/HYU V6_test.sub` and `IMPORTS/KIA/KIA V6 N2.sub` decrypt with a
valid CRC, and saved by KAT in ProtoPirate's layout they reload with the same parts 2 and 3.

## Keystore

KIA V6 A and B keys (keystore types 11 and 12).
