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
15 = CRC-8 (poly 0x07, init 0xFF) of bytes 0–14 (the C accepts a 1-bit mismatch). The C reports every
144-bit frame; `crc_valid` carries its CRC check.

`data` = part 1 (the C's saved Key); `counter` = the counter's low 16 bits; `extra` = encoder state: Fx
(bits 0–7) and the counter's high 16 bits (bits 8–23). Buttons (ARF): 1 Lock, 2 Unlock, 3 Trunk, 4 Panic.

## RF

- **Encoding:** Manchester, common.rs port with the C's sense `level ? ShortHigh : ShortLow`
- **RF modulation:** FM
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder Steps (`kia_protocol_decoder_v6_feed`)

1. **Reset** — a short HIGH starts the preamble; Manchester state Mid1.
2. **WaitFirstHigh** — count short LOWs (HIGHs ignored); a long LOW after ≥ 601 → WaitLongHigh.
3. **WaitLongHigh** — a long HIGH completes the sync, adds `1101`.
4. **Data** — Manchester-decode, storing parts 1/2 inverted at bits 64/128; at 144 bits part 3, AES-decrypt,
   report. Any out-of-tolerance pulse restarts.

KAT's earlier port used a hand-written Manchester table and kept only 16 counter bits; it reproduced none of
the C's frames.

## Encoder

Supported (`kia_protocol_encoder_v6_build_upload`): AES-encrypt the fields (Fx from `extra`, counter + 1
over all 32 bits), then the message twice — 640- and 38-pair preambles, each LOW 200, HIGH 400, LOW 200, the
141 inverted bits as Manchester (1 → LOW, HIGH; 0 → HIGH, LOW) and LOW 400; adjacent equal levels merged.
KAT's generic button maps to Lock 1, Unlock 2, Trunk 3, Panic 4; anything else keeps the captured button.

## Validation

C oracle: 500 random frames (random Fx and 16/32-bit counters) — uploads identical pulse for pulse; the
decoder reports the C's 500 frames (key, serial, button, 32-bit counter, Fx). The second (38-pair) message
is below the decoder's 601-pair preamble gate in both. No IMPORTS RAW capture contains Kia V6.

## Keystore

KIA V6 A and B keys (keystore types 11 and 12).
