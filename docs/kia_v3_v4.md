---
layout: default
---

# Kia V3/V4 Protocol

**Rust module:** `src/protocols/kia_v3_v4.rs`  
**Reference:** `REFERENCES/ProtoPirate/protocols/kia_v3_v4.c` (encoder: 2026-05-19 CRC brute-force
rewrite); Flipper-ARF `kia_v3_v4.c` (same decoder, button names)

## Overview

KeeLoq keyfob (Kia/Hyundai) in PWM at 400/800 µs. 68 bits: a bit-reversed 32-bit KeeLoq hop, the
bit-reversed serial28 + button4, and a 4-bit field the C calls CRC (algorithm unknown; displayed only).
V3 and V4 differ in polarity: the sync is a long LOW (V3, all bits inverted) or a long HIGH (V4).
The decoder reports the version as the display name ("Kia V3" / "Kia V4", as ProtoPirate does);
`get()` maps those names and ProtoPirate's "KIA/HYU V3/V4" to this entry.

## Timing

| Parameter | Value       | Notes                   |
|-----------|-------------|-------------------------|
| Short (0) | 400 µs      | ±150 µs (HIGH width)    |
| Long (1)  | 800 µs      | ±150 µs                 |
| Sync      | 1000–1500 µs | HIGH = V4, LOW = V3    |
| Preamble  | ≥ 8 short pairs |                     |
| Min bits  | 68          |                         |

## Frame Layout (68 bits)

- Bytes 0–3: KeeLoq hop, bits reversed. Bytes 4–7: serial28 | button4 << 28, bits reversed. 4-bit field.
- Decrypting the hop with the KIA manufacturer key (keystore type 10) gives `button << 28 | serial10 << 16
  | counter16`. **The frame is reported only if the decrypted button and serial LSB match the frame's own**
  (both C decoders). `data` = bytes 0–7, `extra` = version (0 V4, 1 V3), counter = decrypted & 0xFFFF.
- Buttons (ARF names): 1 Lock, 2 Unlock, 3 Trunk, 4 Panic, 8 Horn.

## RF

- **Encoding:** PWM — the HIGH width carries the bit; a sync-length pulse or a LOW > 1500 µs ends the frame
- **RF modulation:** FM (ProtoPirate and ARF flags)
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder Steps (`kia_protocol_decoder_v3_v4_feed`)

1. **Reset** — a short HIGH starts the preamble.
2. **CheckPreamble** — count short LOW/short pairs; a sync-length pulse after ≥ 8 starts the data and sets
   the version from its level.
3. **CollectRawBits** — HIGH short = 0, long = 1; a HIGH > 1500 µs aborts; a sync-length pulse or a LOW
   > 1500 µs ends the frame → process (≥ 68 bits, invert for V3, KeeLoq check).

KAT's earlier port also reported frames failing the KeeLoq check (with `crc_valid` false — or with no key
loaded, everything): the IMPORTS Isuzu and Toyota Prius 2006 captures (other makes' KeeLoq; the Isuzu
"serial" changed on every press) were listed as Kia V3/V4. Both references reject them; the Prius frames
now decode as Toyota (stable serial 08DCE7FB).

## Encoder

Supported (`kia_protocol_encoder_v3_v4_get_upload` + yield): plaintext `counter16 | (serial & 0x3FF) << 16
| button << 28`, KeeLoq-encrypted with the KIA key; per burst 12 preamble pairs, sync (V4: LOW 400, HIGH
1200; V3: HIGH 400, LOW 1200), the 64 key bits as PWM (V4: LOW first, 1 → 400/800, 0 → 800/400; V3: HIGH
first), the 4-bit field, end marker 800/800. The C sends 16 bursts with that field 0..15 (a brute force of
the unknown CRC), so `encode()` returns all 16. Counter + 1 per press; version from `extra` (default V4, as
the C); KAT's generic button maps to Lock 1, Unlock 2, Trunk 3, Panic 4, anything else keeps the captured
button. (KAT's earlier encoder sent 3 bursts with a guessed CRC and `serial & 0xFF | 1 << 24` plaintext.)

## Validation

C oracle (`kia_v3_v4.c` on the host, KIA key): 300 random transmissions (16 bursts each, both versions) —
identical pulse for pulse; the decoder reports the C's 2400 frames (8 per transmission: back to back, a
burst ends at the next burst's sync and the following burst's data is skipped, in both). All IMPORTS,
both polarities: identical (300 frames from the four Kia V3/V4 captures, versions included).

## Keystore

KIA manufacturer key (keystore type 10, `kia_mf_key`). Without it nothing passes the KeeLoq check, so the
`sweep_imports_decodes` test (which does not load the keystore) shows no Kia V3/V4 frames.
