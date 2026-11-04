---
layout: default
---

# Kia V0 Protocol (Kia V0 / Suzuki V0 / Honda V0 / Mitsubishi V0)

**Rust module:** `src/protocols/kia_v0.rs`  
**Reference:** `REFERENCES/ProtoPirate/protocols/kia_v0.c` (also doctor-firmware `kia_v0.c`; Flipper-ARF
`kia_v0.c` has the first three types)

## Overview

One PWM air format at 250/500 µs carrying four frame types, told apart by bit count, check and preamble.
ProtoPirate folded its standalone `suzuki.c` (2026-04-17) and `mitsubishi_v0.c` (2026-09-14) into this
decoder, and KAT does the same: the former standalone **Suzuki** decoder is retired (its name still
resolves to Kia V0 for TX), and the type is shown as the display name.

| Type | Display name | Bits | Check | Frame end |
|------|--------------|------|-------|-----------|
| 1 | Kia V0 | 61 | CRC-8 (poly 0x7F, init 0) over bits 8–55 | HIGH 700–1700 µs |
| 2 | Suzuki V0 | 64 | the same CRC-8 over counter/serial/button | LOW then HIGH 2000–2500 µs |
| 3 | Honda V0 | 72 (label; 64-bit key) | structural (nibbles F/0, header low nibble A); CRC shown | 61st bit, or the HIGH gap |
| 4 | Mitsubishi V0 | 61 | as Kia | as Kia |

The real `IMPORTS/SUZUKI/Suzuki_v0_kd.sub` capture is exactly this format (symmetric pairs, 320-pair
preamble, LOW+HIGH 2000 µs gap, 15-pair second preamble); KAT and the C both decode its 15 presses as
Suzuki V0 with the same fields.

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| Short (0) | 250 µs | HIGH and LOW of a bit are equal; ±100 µs, strict `<` |
| Long (1)  | 500 µs | ±100 µs |
| Preamble  | > 14 short pairs | the first long pair is the sync and adds two `1` bits |
| Kia gap   | 700–1700 µs HIGH | ends Kia / Mitsubishi / Honda frames |
| Suzuki gap | 2000–2500 µs LOW then HIGH | ends Suzuki frames |

## Frame Layouts

- **Kia / Mitsubishi** (`data` as decoded, bit 60 set by the sync): counter = bits 40–55, serial = bits
  12–39 (28), button = bits 8–11, CRC = bits 0–7. Buttons: 1 Lock, 2 Unlock, 3 Trunk (4 Panic on the
  emulate D-pad).
- **Suzuki**: `0xF << 60 | counter16 << 44 | serial28 << 16 | button4 << 12 | crc8 << 4`. Buttons: 3 Lock,
  4 Unlock, 2 Trunk, 1 Panic.
- **Honda**: key bytes `F0, rev(cnt_hi), rev(cnt_lo), serial24, header, crc` (`rev` = bit-reversed byte);
  header = `button << 5 | 0x0A` (`0x1A` for button 7); on air the key's bytes are bit-reversed and its low
  56 bits sent. CRC = XOR fold of the counter bits through a 16-entry table, XOR a per-header constant.
  Buttons: 1 Unlock, 2 Trunk, 3 Lock2, 4 Unlock2, 5 Trunk2, 6 Unlock3, 7 Trunk3.
- **Mitsubishi**: a Kia frame sent twice, the second copy behind an 80-pair preamble. The decoder remembers
  the previous Kia frame; an identical frame behind 72–88 preamble pairs is Mitsubishi V0.

## RF

- **Encoding:** PWM, equal-length HIGH/LOW per bit
- **RF modulation:** FM (ProtoPirate); Flipper-ARF also flags AM
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder Steps (`subghz_protocol_decoder_kia_feed`)

1. **Reset** — a short HIGH starts the preamble.
2. **Preamble** — count short/short pairs; a long/long pair after more than 14 starts the data with `11`.
3. **SaveDuration** (HIGH) — a Kia gap ends the frame (61 bits: CRC → Kia/Mitsubishi, else a valid Honda
   key → Honda); a Suzuki HIGH after a Suzuki LOW ends a 64-bit Suzuki frame (CRC checked on the bits as
   received or shifted left by one); otherwise remember the HIGH.
4. **CheckDuration** (LOW) — short/short = 0, long/long = 1; after each bit a 61-bit frame failing the Kia
   CRC is tried as a Honda key and reported at once; a Suzuki LOW is remembered.

KAT's registry resets every decoder after any decode; Kia V0's `reset()` therefore keeps the previous-frame
memory so the Mitsubishi rule works as in the C. Live captures report their first frame only (Kia V0 for a
Mitsubishi transmission); `.sub` imports list both, as ProtoPirate's history does.

## Encoder

Supported for all four types (`kia_v0_encoder_apply_fields` and the four upload builders). The frame is
rebuilt from serial/button/counter with the counter + 1 (16 bits, the emulate scene's fresh press);
KAT's generic button maps to the type's codes listed above (anything else keeps the captured button).
The type comes from the bit count (61 Kia, 64 Suzuki, 72 Honda); Mitsubishi V0 captures are encoded
through the "Mitsubishi V0" registry entry ([Mitsubishi V0](mitsubishi_v0.md)), which hands 61-bit
signals back here. One upload per call (the C repeats it 3 times), adjacent equal levels merged:

- **Kia:** HIGH 750, LOW 750, 319 short pairs, frame, HIGH 1500, LOW 1500, 15 short pairs, frame, HIGH 1000.
- **Mitsubishi:** as Kia with an 80-pair second preamble and a closing HIGH 1500 + LOW 1500.
- **Suzuki:** 320 short pairs, frame, LOW 2000, HIGH 2000, LOW 250, 15 short pairs, frame.
- **Honda:** 40 short pairs, 4 long pairs, the 56 low bits of the byte-reversed key, HIGH 1000.

## Validation

C oracle (`kia_v0.c` compiled on the host): 3000 random frames of all four types — KAT's upload equals the
C's pulse for pulse, and KAT's decoder reports exactly the C's 4487 decodes (type, key, fields) on the
concatenated stream, including Mitsubishi detection and the C's own quirk of reading some Honda frames as
Kia when their wire bits pass the Kia CRC. All IMPORTS captures, both polarities: identical (15 Suzuki V0).
Flipper-ARF's and doctor-firmware's decoders give the same frames (ARF names Mitsubishi frames Kia V0).
Flipper-ARF's standalone `suzuki.c` reads only HIGH widths; it decodes the same 15 real frames, and its
encoder sends an asymmetric waveform (LOW always 250 µs) that does not match the real fob, so it adds no
capability. ARF sends Honda V0 in the Kia double-frame layout; KAT follows ProtoPirate.
