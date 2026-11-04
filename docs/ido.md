---
layout: default
---

# iDo 117/111 Protocol

**Rust module:** `src/protocols/ido.rs`
**Reference:** doctor-firmware `lib/subghz/protocols/ido.c` (not in Flipper-ARF's `subghz_garage` RX groups; ARF's
lib copy and the garage copy decode identically).

## Overview

iDo 117 / 111 gate remotes: a 48-bit hopping-code frame. The references flag it `SubGhzProtocolTypeDynamic` with
`433 | AM | Decodable | Save` — no `Load`/`Send` — and the encoder table is all NULL, so KAT decodes and saves it but
cannot transmit it (`encoder_capable` false, `encode` → None).

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| Short | 450 µs | |
| Long | 1450 µs | |
| Preamble | HIGH 4500 + LOW 4500 | `10·te_short`, each ±750 µs |
| Bit `0` | HIGH 450 (±150) + LOW 1450 (±450) | |
| Bit `1` | HIGH 450 (±450) + LOW 450 (±150) | |
| Frame end | HIGH ≥ 2400 µs | `5·te_short + te_delta` (the next preamble) |
| Bits | ≥ 48 | the C's `>=` |

## Frame Layout

The frame is read bit-reversed: `rev = reverse_key(data, bits)`.

- **fix** = `rev & 0xFFFFFF`: **serial** = `fix & 0xFFFFF` (20 bits), **button** = `(fix >> 20) & 0xF`
- **hop** = `(rev >> 24) & 0xFFFFFF` (shown by the C as "Hop"; stays in `data`)

## RF

- **Encoding:** PWM
- **RF modulation:** AM, `SubGhzProtocolFlag_433 | AM`; AM650 (for exports)
- **Encryption:** hopping (rolling) code; no checksum

## Decoder Steps

1. **Reset** — the 4500 µs HIGH. 2. **FoundPreambula** — the 4500 µs LOW (else Reset).
3. **SaveDuration / CheckDuration** — HIGH + LOW pairs → bits; a HIGH ≥ 2400 µs reports the frame when at least 48 bits
   were read.

## Key file

`Protocol: iDo 117/111`, `Bit: 48`, `Key` = the frame. The decoder's `deserialize` loads exactly 48 bits; KAT loads it
for display (not for transmission).

## Validation

- Flipper unit test `ido_117_111_raw.sub` decodes (10 frames, serial 52EA, button 3, ten different hop codes).
- C oracle (doctor build): decodes (key, Sn, Btn) on the unit-test corpus, IMPORTS, UberGuidoZ and 4224 fuzz files (two seeds)
  (synthesised frames, jittered/split/dropped) equal the oracle's.
