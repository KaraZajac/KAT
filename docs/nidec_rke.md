---
layout: default
---

# Nidec RKE Protocol

**Rust module:** `src/protocols/nidec_rke.rs`
**Reference:** FoxFW `lib/subghz/protocols/nidec_rke.c` (`NIDEC_RKE_PROTOCOL_NAME = "Nidec RKE"`), a pulse-by-pulse rewrite
of rtl_433 `nidec_car_remote.c`.

## Overview

The Honda OEM remote OUCG8D-344H-A (313.8 MHz FSK). The reference flags it `SubGhzProtocolTypeDynamic`, `315 | FM`. It is
a different remote from KAT's rtl_433 Honda-KR5V2X1X port. Receive only, as in FoxFW (see Transmit).

## Timing

Only the HIGH pulses carry data.

| Parameter | Value | Notes |
|-----------|-------|-------|
| Raw `1` | HIGH 130..374 µs | nominal 250 µs |
| Raw `0` | HIGH 375..700 µs | nominal 500 µs |
| Packet end | LOW >= 700 µs | shorter LOWs are ignored |
| Sync | last 32 raw bits = `0xFFFFFFF0` | 28 short pulses, then 4 long |

Payload bits are the raw bits **inverted**. The frame is complete at 64 bits, or at the packet-ending LOW with at least 56
bits (the missing tail is zero-padded, as rtl_433 does).

## Frame Layout (8 bytes after inversion)

| Bytes | Field |
|-------|-------|
| 0..1 | **counter** (16-bit sequence) |
| 2..4 | **serial** (24-bit remote id) |
| 5 | `5` (high nibble) and the 4-bit **button** |
| 6..7 | security word (`extra`), a keyed authenticator that no CRC reproduces |

There is no checksum, so the decoder uses rtl_433's structural gate: high nibble of byte 5 is 5, id is not 0 or
0xFFFFFF, sequence and security are not 0 or 0xFFFF, button is 3, 4, 5, 6 or 0xF. `crc_valid` stays false.
`data` is the 8 bytes left-aligned, `data_count_bit` the bits captured (56..64).

## Transmit

None, as in FoxFW. Its SD-card encoder plugin `applications/main/subghz/protocols_ext/enc_nidec_rke.c` is a stub ("no
TX — security field is a keyed authenticator, not emittable"): `deserialize` returns
`SubGhzProtocolStatusErrorEncoderGetUpload` for every key file and `yield` only ends the transmission, so a Flipper
sends nothing for any Nidec RKE file. KAT does the same: `supports_encoding()` is false, `encode` returns None and no
decode or loaded key is encoder-capable (Replay of a recording still works).

## RF

- **Encoding:** PWM (pulse width)
- **RF modulation:** FM (FM476)
- **Encryption:** rolling code with a keyed authenticator (not verifiable)

## Validation

- No capture exists in the corpus.
- Transmit: FoxFW's encoder on the host oracle (`subghz_oracle_foxfwenc encode`) refuses all of 117 made-up key files
  (56 to 64 bits with every button, frames its decoder's gate refuses, 55, 65 and 0 bits, a short Key: "deserialize
  failed", status -12), and KAT transmits none of them.
- Synthetic frames (made-up values, a full 64-bit frame and one truncated at 60 bits) decode identically in the FoxFW C
  decoder (host oracle) and in KAT. A 55-bit payload, a wrong high nibble, an unseen button, a zero id, a zero sequence, a
  zero security word (a 56-bit truncation zeroes it) and a 27-ones sync are rejected by both.
- No change to any file of IMPORTS or the Flipper unit-test corpus.
