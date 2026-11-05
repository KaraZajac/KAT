---
layout: default
---

# RojaFlex Protocol

**Rust module:** `src/protocols/rojaflex.rs` (rtl_433 receive chain: `src/protocols/rtl433/`)
**Reference:** rtl_433 `src/devices/rojaflex.c` (GPL-2.0; r_device "RojaFlex shutter and remote devices", `-R 187`;
models `RojaFlex-Remote`, `RojaFlex-Shutter`, `RojaFlex-Bridge`, `RojaFlex-Other`)

## Overview

RojaFlex shutter motors, their remotes and the P2D bridge, 433.92 MHz FSK. One decoder for all four models: the
registry name is `RojaFlex-Remote`; frames of the other models are shown under their rtl_433 model name (aliases of
the same entry). Receive only. No recordings exist.

## Timing / modulation

- FSK_PULSE_PCM, 100 µs per bit, `reset_limit` 102.4 ms: one row of bits per FSK package (upper tone = 1).
- **KAT's FM path**: a LOW over 32 bit periods (3.2 ms) is the carrier stopping and ends the package. The frame's
  final zero bits merge into that LOW, while rtl_433 sees the carrier stop right after them (its classic FSK
  detector, used below 800 MHz, keeps them in the last segment), so the LOW is read as 0 … 32 trailing zeros: the
  shortest reading that passes the CRC wins, else the shortest reading that makes a 72-bit frame without CRC whose
  token sum checks (below).

## Frame Layout

`aaaaaaaa d391d391` (preamble + sync; the decoder searches `aaaa d391d391`), then:

| Byte | Field |
|---|---|
| 0 | size (0x08) |
| 1–3, 4 high nibble | 28-bit installation ID |
| 4 low nibble | channel (0 = all) |
| 5 | command ID (low nibble 0x5 = shutter status, 0xA = remote / bridge) |
| 6 | command value (status: position %) |
| 7 | token |
| 8 | token sum (ID bytes + data + token) |
| 9–10 | CRC-16/CMS (poly 0x8005, init 0xFFFF) — missing from bridge commands |

## Decoder Steps (`rojaflex_decode`)

1. Exactly one row; search the 48-bit preamble/sync.
2. `count = (row bits − sync end) & 0xFE` (the C's `uint8_t`); 72 ≤ count ≤ 88.
3. 88 bits: the CRC must match (else dropped); shorter frames are reported unchecked.
4. Model from the command low nibble and whether the frame had a CRC.

**KAT deviation** (forced by the unknown tail on the FM path): a frame without CRC is taken only as the bridge's 9
bytes (72 bits, "17 bytes without CRC" in the C's notes) with a correct Token II (byte 8 = the sum of bytes 1..7, as
the C documents it and its cloner builds it); `crc_valid` stays false (the C checks nothing there). The C also
reports 74..86-bit CRC-less frames and wrong sums, which only arise from cut or corrupted receptions; with an unknown
tail, accepting them would turn 88-bit frames that fail their CRC into CRC-less "Bridge" / "Other" frames under a
shorter tail reading, and pad cut frames up to 72 bits under a longer one.

## Field mapping

| KAT | rtl_433 | Notes |
|---|---|---|
| display name | `model` | none for `RojaFlex-Remote`, else the model |
| `serial` | `id` | 28 bits |
| `button` | `cmd_id` | labels: rtl_433's `cmd_name` (Stop, Up, Down, Save/Unsave position, Go saved position, Request Status, Pos. Status n %, else "unknown") |
| – | `channel`, `cmd_value`, `token` | `data` byte 4 low nibble, byte 6, byte 7 + `extra`'s top byte |
| `data` / `extra` | | frame bytes 0..8 / the rest right-aligned (token low, CRC); `data_count_bit` = 88, or 72 without CRC |
| `crc_valid` | `mic: CRC` | the CRC was checked (88-bit frames) |

## Encoder

None — receive only, as rtl_433 (its `GENERATE_COMMANDS` cloner is compiled out).

## Validation

- **End to end** (synthetic FSK IQ, cu8): six frames — three remote commands, a shutter position status, a bridge
  command without CRC and a remote frame whose CRC ends in five zero bits — decoded identically (id, command, model,
  integrity) by rtl_433 (its own FSK demodulator, `-R 187`) and by KAT's IQ pipeline, in four conditions (250 k / 1 MS/s,
  ±15 … ±40 kHz deviation, −50 … +30 kHz offset, 10 … 20 dB).
- **rtl_433 oracle** on 400 synthetic packets (rtl_433 slices with a flex `OOK_PCM` decoder of the same timing,
  decodes with `-R 187 -y`): every CRC-checked frame identical; the differences are CRC-less frames from the fuzzer's
  cut/corrupted packets (rtl_433 23, KAT 8), the deviation above.
- Unit tests: every remote command and channel at 96–104 µs bits, shutter status, bridge, CRC frames ending in 1..8
  zero bits, CRC failures, missing sync, short frames, wrong bit rate.
