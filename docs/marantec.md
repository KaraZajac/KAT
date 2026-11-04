---
layout: default
---

# Marantec Protocol

**Rust module:** `src/protocols/marantec.rs`
**Reference:** Flipper-ARF `applications/system/subghz_garage/protocols/marantec.c` (garage app, RX group g9 +
`tx_marantec`); doctor-firmware and Flipper-ARF `lib/subghz/protocols/marantec.c` decode and encode identically.

## Overview

Marantec garage-door remotes: a 49-bit static code in Manchester, with an 8-bit CRC. The C only *displays* the CRC
("CRC: … Valid/Invalid") and reports every 49-bit frame; KAT reports only CRC-valid frames. Every known real frame
carries a valid CRC (the two remotes quoted in the C and the Flipper unit-test remote).

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| Short | 1000 µs | ±200 µs (te_delta, strict `<`) |
| Long | 2000 µs | ±200 µs |
| Header | LOW 10 000 µs | `5·te_long`, accepted ±1600 µs |
| Frame end | LOW ≥ 4200 µs | `2·te_long + te_delta`; doubles as the next header |
| Bits | 49 | exactly |

## Frame Layout (49 bits, MSB first)

```
bit 48 | 47..20       | 19..16 | 15..8  | 7..0
  1    | serial (hi)  | button | serial | CRC-8
```

- **Sn** (as the C prints it) = `((data >> 12) & 0xFFFFFF00) | ((data >> 8) & 0xFF)` → KAT `serial`
- **Btn** = `(data >> 16) & 0xF` → KAT `button`
- **CRC** = CRC-8, poly 0x1D, init 0x01, MSB first, over the six bytes `data >> 48 … data >> 8`; must equal the low byte.

## RF

- **Encoding:** Manchester (Flipper convention: `1` = LOW half then HIGH half)
- **RF modulation:** AM (OOK), `SubGhzProtocolFlag_433 | AM`; transmitted with AM650
- **Encryption:** none (fixed code + CRC-8)

## Decoder Steps

1. **Reset** — wait for the LOW header; the register starts with the leading `1`, the Manchester machine at Mid1.
2. **DecoderData** — short/long LOW and HIGH pulses feed Flipper's Manchester decoder; a LOW ≥ 4200 µs ends the frame
   (reported when exactly 49 bits and, in KAT, the CRC matches) and restarts collection; any other pulse returns to Reset.

## Encoder

`subghz_protocol_encoder_marantec_get_upload`, `repeat` = 3: Flipper's Manchester encoder over the 49 bits, except that
the first half-bit of the top bit is replaced by the 10 000 µs LOW header and the final half-bit is only sent when HIGH.
No counter; the button is never changed (ARF exposes the D-pad but every direction re-sends the captured frame because
the CRC would need recomputing), so `encode` ignores `button`.

## Key file

`Protocol: Marantec`, `Bit: 49`, `Key` = the 49-bit frame (8 bytes, big-endian). Loading requires `Bit: 49` (the C's
`deserialize_check_count_bit`); `crc_valid` reports whether the loaded key's CRC matches.

## Validation

- Flipper unit test `marantec_raw.sub` decodes (6 frames, key `1300710DF869F`, Sn `0710D86`, Btn F, CRC valid);
  `marantec.sub` loads and encodes back.
- C oracle (doctor-firmware and garage builds; the decoders are identical): every decode on the unit-test corpus,
  Unleashed/extra unit tests, all IMPORTS captures and 1171 UberGuidoZ files equals the oracle's; on 4224 fuzz files
  (two seeds: jittered/split/dropped/noisy C uploads, protocol mixes, random streams) the oracle reported 127
  CRC-invalid frames that KAT drops, and one file differs further: after a dropped CRC-invalid frame KAT keeps decoding
  (the C's receiver reset after reporting it makes the C skip the next copy).
- Encoder: 7 key files (unit test + random keys) and KAT's own `to_flipper_key` files give uploads pulse-identical to
  the oracle's `encode`.
