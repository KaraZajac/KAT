---
layout: default
---

# Honda V1 Protocol

**Rust module:** `src/protocols/honda_v1.rs`
**Reference:** `REFERENCES/ProtoPirate/protocols/honda_v1.c` (HEAD dc3446b; Flipper-ARF and doctor-firmware decode identically)

## Overview

Honda/Acura counter-based (rolling-code) keyfobs — a DIFFERENT protocol from `honda_static`. The on-wire frame carries 68
bits: 64-bit data + a 4-bit CRC-fold nibble. Encoding is short/long pulse PWM, glued together by a
"pending bit" timing accumulator before classification: sub-`te_delta` runts are summed into `pending`;
a HIGH level extends the running HIGH pulse; a LOW flushes the accumulated HIGH (if it reached
`te_short_min`) as a synthetic symbol, then classifies the LOW.

Validation is a button-code table (Unlock=0, Lock=8, Trunk=9, Panic=10) plus the check nibble.
Emission is gated on the button being valid; `crc_valid` reflects whether the received nibble is one of the two
the key has. The strong button gate keeps Honda V1 from false-matching.

## Timing

| Parameter   | Value   | Notes                       |
|-------------|---------|-----------------------------|
| Short       | 1000 µs | ±400 µs (te_delta)          |
| Long        | 2000 µs | ±400 µs                     |
| Short min   | 600 µs  | flush threshold for HIGH    |
| End gap     | 3500 µs | >te_end commits the frame   |
| Min bits    | 68      |                             |

## Frame Layout (68 bits = 64-bit data + 4-bit CRC nibble)

After the end gap, `commit` left-shifts the 12-byte bit buffer to drop leading preamble leakage and align
the trailing 68-bit frame. `data` = first 8 bytes (64 bits); the CRC nibble = byte 8's high nibble.

- **data[63:36]:** serial (28-bit)
- **data[31:28]:** button (nibble) — Unlock=0, Lock=8, Trunk=9, Panic=10
- **data[15:0]:** counter (16-bit)
- **Check nibble:** one of two values per key (below)

## The check nibble (KAT's rule, not the references')

A press sends the frame with two check nibbles, each twice (each 20 times while the button is held). With **N =
the XOR of the key's 16 nibbles**:

- one nibble is `N ^ 4`;
- the other has the opposite top bit, and its low three bits are one higher (mod 8);
- the nibble whose top bit is clear is sent first.

ProtoPirate computes the nibbles from the counter and one byte of the serial (`honda_v1_crc_fold`,
`honda_v1_checksum_base`, `honda_v1_checksum_alternate`); Flipper-ARF, doctor-firmware and FoxFW carry the same
code. Its second nibble is the same step taken backwards; its base value, fitted to part of the key, is what
differs.

| On the fob in IMPORTS (serial 4119B5A; 15 recordings, all four buttons, 249 codes) | Frames whose nibble fits |
|---|---|
| KAT's rule | 3254 of 3254 |
| ProtoPirate's rule | 803 of 3254 |

A Flipper running a reference's encoder therefore sends this fob's codes with nibbles the fob never sends:
`IMPORTS/honda/more/Fz_10l_10u_10t_10p.sub` holds such transmissions (46 of its 158 frames), next to
`Orig_10l_10u_10t_10p.sub`, the same presses from the fob. KAT checks and sends the fob's nibbles. Only one
serial is in the corpus; the rule covers every nibble of the key, the serial's included, but a second fob has
not confirmed that part.

## RF

- **Encoding:** short/long pulse PWM with pending-bit timing accumulation
- **RF modulation:** AM (OOK)
- **Encryption:** none — gated on the button-code table + the check nibble
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder Steps

1. **feed** — accumulate sub-`te_delta` runts into `pending`; flush HIGH pulses and classify LOW pulses into symbols.
2. **symbol** — Reset → Preamble (short/long pulses, needs a long + >5 count) → Data.
3. **Data** — pending-bit accumulation: a short pulse toggles `data_pending` and emits a bit when paired; a long pulse emits directly. After a >te_end gap, commit: align the 68-bit frame, validate the button, and emit.

## Encoder

`subghz_protocol_encoder_honda_v1_deserialize` + ProtoPirate's emulate scene (Flipper-ARF does the same,
bumping the counter inside its encoder): the Key's fields overridden by the serial, the button (KAT
Lock/Unlock/Trunk/Panic → 8/0/9/10; other commands keep the captured button; an invalid button falls back
to the Key's, then Unlock) and the counter + 1 (16 bits). The 64-bit key is rebuilt from the button code
table, then a 180-element short-pair preamble (last element LOW 5000 µs) + 4 PWM frames (2 per check
nibble, KAT's two, in the fob's order) are emitted.

## Validation

Captures (added 2026-11, `IMPORTS/honda/` and `honda/more/`): 15 recordings of one 433.92 MHz fob
(`Honda_433AM_*`, `Honda_V1_orig_320_locks.sub`, `Orig_10l_10u_10t_10p.sub`, `Long.sub`, `Short.sub`,
`0 (2).sub`, `single_click_Orig2.sub`): 3254 frames, every one with a check nibble that fits (table above).
Frame for frame the keys are the ones ProtoPirate's, Flipper-ARF's, doctor-firmware's and FoxFW's C read from the
same files; only the check verdict differs.

Differential tests against the compiled C (made before the check nibble changed): 1200 randomized bursts
(jitter, runt pulses for the pending-bit accumulator, inverted) decode identically to ProtoPirate, Flipper-ARF
and doctor-firmware (1493 frames); 1870 uploads equalled ProtoPirate's (encoder + emulate scene) and
Flipper-ARF's pulse for pulse. KAT's uploads now differ from theirs in the four check bits of each frame. Unit
tests cover the fob's nibbles for keys on both sides of every case of the rule, the key/field round trip,
encode→decode for every button with the 16-bit counter wrap, and the button-validity mask.
