---
layout: default
---

# Subaru Protocol

**Rust module:** `src/protocols/subaru.rs`
**Reference:** `REFERENCES/ProtoPirate/protocols/subaru.c` (primary); `REFERENCES/Flipper-ARF/lib/subghz/protocols/subaru.c` (older frame reader, kept as a second path)

## Overview

Subaru keyfobs send a 64-bit rolling-code frame as constant-period PWM (~2400 µs per bit): after a
long 1600 µs preamble, a 2800 µs LOW gap and a 2800 µs HIGH sync, each bit is a LOW followed by a
HIGH — `1` = LOW 1600 + HIGH 800, `0` = LOW 800 + HIGH 1600. The real Impreza fob sends one such frame
(76 preamble pairs) followed by 32-bit repeat frames that no reference decodes.

KAT runs two readers on every pulse and reports a frame once:

- **ProtoPirate reader** — HIGH 1600 starts; ≥21 LOW 1600 preamble halves; LOW gap and HIGH sync
  2800 ± 800 µs; then each bit is a LOW (800/1600 ± 200) and the complementary HIGH; the frame is
  reported on its 64th bit.
- **Flipper-ARF reader** — >20 preamble halves (1600 ± 250), gap and sync 2000–3500 µs, a 1600 µs LOW,
  then the bit is the HIGH width alone (800 = 1, 1600 = 0); reported on the first pulse >3000 µs after
  ≥64 bits. It reads ARF's own encoder output (HIGH-first bits with a fixed 800 µs LOW), which the
  ProtoPirate reader rejects. When the ProtoPirate reader completes a frame the ARF reader is reset.

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| Short | 800 µs | ±200 (ProtoPirate) / ±250 (ARF) |
| Long | 1600 µs | ±200 / ±250 |
| Gap (LOW) | 2800 µs | ±800 / 2000–3500 |
| Sync (HIGH) | 2800 µs | ±800 / 2000–3500 |
| Preamble | ≥21 halves | ProtoPirate encoder sends 75 pairs |
| Bits | 64 | |

## Frame Layout (64 bits = 8 bytes b0..b7)

- **b0[3:0]:** button — 1 Lock, 2 Unlock, 3 Trunk, 4 Panic (ProtoPirate's emulate codes and ARF's labels; ProtoPirate's decoder label table `names[btn & 3]` and doctor's D-pad disagree, and no capture settles it)
- **b1..b3:** serial (24 bits)
- **counter (16 bits):** low byte stored inverted in bits of b4..b6; high byte XOR-masked into `REG_SH1/REG_SH2` (b5..b7) with the serial rotated left by `(4 + lo) mod 256`
- **b4[5:0], b5[5:4]:** `rel` = rotated serial byte ⊕ b0

There is no checksum: frames are reported unconditionally (ProtoPirate's extra validity expression is
unused by the C and false on every real frame).

## RF

- **Encoding:** PWM (constant bit period)
- **RF modulation:** AM (OOK)
- **Encryption:** none (rolling counter obfuscated with the serial)
- **Frequencies:** 315 MHz, 433.92 MHz

## Encoder

Supported. Serial, button and counter + 1 are written back into the captured key (`b0` low nibble,
`b1..b3`, `subaru_encode_count`) and sent as ProtoPirate's upload: 75 × (LOW 1600, HIGH 1600),
LOW 2800, HIGH 2800, 64 data pairs, LOW 2800, HIGH 2800 (282 entries; the Flipper repeats it while held,
KAT sends it once). ARF builds a different HIGH-first upload that ProtoPirate cannot read; KAT sends
ProtoPirate's, which matches the real fob. Buttons: Lock → 1, Unlock → 2, Trunk → 3, Panic → 4.

**Deviation (counter encoding):** `subaru_encode_count` leaves eight bits of `T1/T2` that the decoder
does not read at 1. In the real Impreza capture those bits carry the counter's low byte inverted (`T1`
bits 2,3,6,7 = `~lo` bits 0,1,4,5; `T2` bits 0,1,4,5 = `~lo` bits 2,3,6,7). KAT encodes them that way:
re-encoding each of the five captured frames reproduces it bit for bit and encoding the next counter
gives the next captured frame (9/9; ProtoPirate 0/9, ARF 0/9).

## Validation

Compared with the compiled ProtoPirate and Flipper-ARF `subaru.c` (host harness): on 2000 PP-encoded and
2000 ARF-encoded frames (plain, jittered, glitched, and mixed PP/ARF streams) and every IMPORTS capture,
KAT reports exactly the union of the two C decoders' frames. The ProtoPirate upload is identical
pulse-for-pulse (2000/2000); the key differs from ProtoPirate's only in the eight bits above (identical
whenever the counter's low byte is 0). The Impreza capture decodes as 5 frames (serial 0xCB715E,
counters 0xC90A–0xC90E) in KAT and both references; the Outback capture is noise.
