---
layout: default
---

# Mitsubishi V0 Protocol

**Rust module:** `src/protocols/mitsubishi_v0.rs`  
**Reference:** `REFERENCES/Flipper-ARF/lib/subghz/protocols/mitsubishi_v0.c` (96-bit format);
ProtoPirate `protocols/kia_v0.c` type 4 (61-bit format)

## Overview

Two references use the name "Mitsubishi V0" for two different air formats. KAT keeps both under this one
name:

- **61-bit (ProtoPirate, current):** a Kia V0 frame sent twice, the second copy behind an 80-pair preamble.
  It is decoded by [Kia V0](kia_v0.md), which reports it as "Mitsubishi V0". ProtoPirate removed its old
  standalone `mitsubishi_v0.c` on 2026-09-14 ("merge Mitsubishi protocol into Kia V0").
- **96-bit (Flipper-ARF):** the old ProtoPirate format, still shipped (enabled, with an encoder) by
  Flipper-ARF. ProtoPirate's Kia V0 does not read it, so this module decodes it.

The registry entry owns the name for TX: a 61-bit signal is encoded by Kia V0's Mitsubishi upload, a 96-bit
one by the ARF encoder below.

## 96-bit format

| Parameter | Value | Notes |
|-----------|-------|-------|
| Bit 1 | HIGH 250 µs + LOW 500 µs | ±100 µs, strict `<` |
| Bit 0 | HIGH 500 µs + LOW 250 µs | |
| Frame | 96 bits (12 bytes) MSB first | no preamble, sync or checksum gate in the decoder |

On air the first 8 bytes are inverted and bytes 0–4 XORed with `hi ^ lo` of the counter. The decoder
inverts bytes 0–7, reads the counter from bytes 4–5 **before** undoing the XOR, then XORs bytes 0–4 —
so byte 4 of the displayed counter is itself scrambled (ARF: "asymmetric scramble"; the displayed
counter's two bytes are always equal). Fields: serial = bytes 0–3, counter = bytes 4–5, button = byte 6.

`data` = wire bytes 0–7 and `extra` = wire bytes 8–11 (ARF's saved `Raw_Frame`). Captures from KAT's
earlier decode-only port kept the unscrambled bytes in `data` and no `extra`.

## RF

- **Encoding:** PWM, complementary HIGH/LOW widths
- **RF modulation:** FM
- **Frequencies:** 315 MHz, 433.92 MHz, 868.35 MHz (ARF flags; KAT's earlier port listed only 868.35)

## Decoder Steps (`subghz_protocol_decoder_mitsubishi_v0_feed`)

1. **Reset** — any HIGH is remembered → DataCheck.
2. **DataCheck** — a LOW completes the pair; an invalid pair discards the payload. The frame is reported on
   the LOW of its 96th pair.
3. **DataSave** — waits for the next HIGH (a LOW here resets).

## Encoder

Supported (ARF `subghz_protocol_encoder_mitsubishi_v0_get_upload`), replay-only like ARF: the captured
12-byte frame is re-sent verbatim — no counter bump, and the captured button for every command (ARF knows
no button codes). Without the raw tail (`extra` = None) the frame is rebuilt from serial/counter/button as
ARF's fallback does (bytes 7–8 = 0, 9 = 0x5A, 10–11 = 0xFF, then scrambled). Upload: 100 + 95 short pairs
(HIGH 250, LOW 250), then the 96 data pairs. The frame has no terminator, so `encode()` returns ARF's whole
transmission (5 copies back to back); on air a receiver completes the first 4. ARF sizes its upload buffer
for 414 entries but writes 582 (a heap overflow on the device); the intended upload is unambiguous.

## Validation

C oracle (ARF `mitsubishi_v0.c` on the host): 2000 random frames (raw replay and field fallback) — KAT's
upload equals ARF's pulse for pulse and KAT's decoder reports the same 2000 frames (fields and raw bytes).
No IMPORTS capture contains this format (0 decodes in both C and KAT, both polarities).
