---
layout: default
---

# Toyota Protocol

**Rust module:** `src/protocols/toyota.rs`
**Reference:** `Flipper-ARF lib/subghz/protocols/toyota.c` (= doctor-firmware `toyota.c`, "Toyota Lexus")

## Overview

Toyota / Lexus keyfobs using a 96-bit rolling frame at ~793/1587 µs. Flipper-ARF adopted this decoder +
encoder from doctor-firmware in October 2026 and retired its earlier dual-variant KeeLoq Toyota decoder
(Corolla 400/800 µs PWM + Tundra NRZ, decode-only; kept only as the uncompiled `toyota-old.c.bak`). KAT
mirrors the replacement. The KeeLoq-PWM Prius captures in `IMPORTS/Toyota + Lexus/` keep decoding as
Kia V3/V4.

The air format is read level-agnostically: two short pulses are a 0 bit, one long pulse is a 1 bit
(bi-phase-space: a level change at every bit boundary, plus one mid-bit for a 0). A button press sends
10 back-to-back copies of the payload, then 10 copies of its "second half" twin, with no gaps. The decoder
slides a 120-bit window (payload + the next 3 bytes) and accepts it when byte 0 is `0x00`, byte 1 has
its MSB set and the first 3 bytes repeat at bytes 12–14.

## Timing

| Parameter        | Value                         | Notes                                  |
|------------------|-------------------------------|----------------------------------------|
| Short            | 793 µs (`TOYOTA_US(0x0320)`)  | accepted: 608–959 µs (32 µs ticks 19–29) |
| Long             | 1587 µs (`TOYOTA_US(0x0640)`) | accepted: 1280–1855 µs (ticks 40–57)   |
| Merged long      | 2380 / 3174 / 3967 µs ±185    | also read as one long (1 bit)          |
| Payload          | 96 bits                       | window 120 bits                        |
| Burst            | 10 × primary + 10 × second    | continuous                             |

Any other duration resets the window.

## Frame Layout (96 bits / 12 bytes)

- **byte 0:** `0x00`; **byte 1:** MSB set (alignment marker)
- **fixed id (24 bits):** `(b4 & 0x0F) << 20 | (b6 >> 4) << 16 | b8 << 8 | b11` — reported as `serial`
- **byte 3:** button; `0xD1` ("Estima") in every recoverable frame
- **obfuscation:** a 7-bit counter selects which of 8 mask rows are XORed over the 12 bytes; bytes 1 and 3
  also get `0x09` for an odd counter and `^0x05` for the second-half frame

`data` = bytes 0–7, `extra` = bytes 8–11, `data_count_bit` = 96.

## Decoding

`toyota_recover` tries all 128 counters × 2 halves for the unique one whose plaintext keeps the fixed id
of the frame's selector-derived "standard" view and has button `0xD1`:

- **recovered:** reports the rebuilt primary (first-half) frame, `serial` = fixed id, `button` = `0xD1`,
  `counter` = 0–127, `crc_valid` = true, encoder-capable;
- **not recovered:** reports the raw window like the C (button = the C's `toyota_button_from_payload`:
  1/2/3 from byte 2 of the standard view, an Estima code, or 0; `serial` = the fixed id the C displays; no
  counter), `crc_valid` = false, not encoder-capable (the C encoder refuses it too).

The C also suppresses re-reports of the same payload until a ≥20 ms gap; KAT's registry resets every
decoder after a hit, so KAT reports each aligned window and the import dedup collapses the repeats.

## RF

- **Encoding:** bi-phase-space (differential Manchester family), level-agnostic
- **RF modulation:** AM/OOK
- **Encryption:** mask-table obfuscation with a 7-bit down-counter (no key)
- **Frequencies:** 315 MHz, 433.92 MHz (C flags 315 | 433)

## Encoder

Supported (`subghz_protocol_encoder_toyota_deserialize` → `toyota_get_upload`). Recovers the plaintext of
the stored frame, steps the counter DOWN by one (mod 128), and sends 10 × primary + 10 × second-half frame
with the bi-phase phase carried across all 20 copies (≈2900–3000 pulses, first pulse LOW). The C's command
selection (1–3) is only range-checked and never reaches the frame, so every KAT button sends the same
upload.

ARF's own `generic.c` does not persist `data_2` (`Key_2`), so in ARF a saved Toyota capture with a non-zero
tail cannot be loaded back for transmit; doctor-firmware's `generic.c`, which this code was written
against, does. KAT keeps all 96 bits.

## Validation

C oracle (the unmodified ARF and doctor-firmware `toyota.c` compiled on the host; their uploads are
identical): 5004 stored frames encoded pulse-for-pulse identically (3035 encodable, 1969 refused by
both), and 1623 pulse streams (1500 randomized: clean, jittered, glitched, merged and truncated
uploads, random bi-phase payloads, boundary durations; plus all 123 IMPORTS captures) decoded with
identical hits and fields (10580 hits). No IMPORTS capture contains this format (the 312/314.35 MHz
Lexus/Toyota captures are most likely FSK smart keys recorded with an OOK preset). The old KeeLoq port's
single hit on `19_toyota_camry_l2_u2_t2_p2.sub` (hop `0x1AAAAAAA`, button 1) was a false positive that
ARF's final KeeLoq decoder also rejects. Unit tests embed C golden uploads and cover recovery, the
alignment gate, the unrecoverable path and registry isolation.
