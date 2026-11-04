---
layout: default
---

# Land Rover RKE Protocol

**Rust module:** `src/protocols/land_rover_rke.rs`
**Reference:** `Flipper-ARF lib/subghz/protocols/landrover_rke.c` (unchanged since May 2026; a standalone
`lr_encode`/`lr_decode` library — ARF registers no Flipper protocol for it)

## Overview

> **Not registered.** Flipper-ARF keeps `landrover_rke.c` (a Pandora DXL 5000 reverse-engineering port) as a helper and never registers it as a SubGhz protocol; KAT mirrors that. The decoder/encoder below is ported and tested; to enable it, add `Box::new(land_rover_rke::LandRoverRkeDecoder::new())` to `ProtocolRegistry::new()`.

Ported from the Flipper-ARF firmware (D4C1-Labs), itself derived from Pandora DXL 5000 firmware. Land
Rover shares the Ford/Jaguar baseband (firmware protocol ID `0x0E`) but uses a distinct 66-bit frame.
Encoding is fixed-width PWM with a 1000 µs bit period: Bit-1 = 700 µs HIGH + 300 µs LOW, Bit-0 = 300 µs
HIGH + 700 µs LOW. Preamble = 20× (400 µs HIGH + 600 µs LOW); sync = 400 µs HIGH + 9600 µs LOW.

KeeLoq: the hop code is the raw 32-bit KeeLoq ciphertext. Full decryption needs the per-fob manufacturer
key (provisioned, not in firmware), so KAT exposes the framed fields and leaves the hop encrypted —
`crc_valid = false` since no cryptographic check is performed. Emission is still gated tightly on the long
preamble run, the distinctive 9.6 ms sync gap, and exactly 66 PWM bits, so this loose-looking PWM decoder
does not false-match Kia/Subaru/Ford captures.

## Timing

| Parameter      | Value   | Notes                       |
|----------------|---------|-----------------------------|
| Bit-1          | 700 µs HIGH + 300 µs LOW | ±20% (relative) |
| Bit-0          | 300 µs HIGH + 700 µs LOW | ±20%            |
| Preamble pair  | 400 µs HIGH + 600 µs LOW | ×20             |
| Sync           | 400 µs HIGH + 9600 µs LOW |                |
| Repeat gap     | 12000 µs |                            |
| Min bits       | 66      |                             |
| Preamble min   | 16 pairs|                             |

(Reported timing profile: te_short 300 µs, te_long 700 µs, te_delta 140 µs.)

## Frame Layout (66 bits, MSB-first)

- **[65:34]:** 32-bit KeeLoq encrypted hopping code
- **[33:10]:** 24-bit fixed fob serial
- **[9:6]:** 4-bit button — Lock=0x1, Unlock=0x2, Boot/Tailgate=0x4, Panic=0x8
- **[5:2]:** 4-bit function/repeat flags
- **[1:0]:** 2-bit status — battery-low=0x1, repeat=0x2

66 bits do not fit a u64: `data` holds the low 64 frame bits and `extra` holds the top 2 (the high 2 bits
of the hop code), so encode round-trips the hop losslessly. The KAT counter field surfaces the low 16
bits of the hop ciphertext.

## RF

- **Encoding:** fixed-width PWM
- **RF modulation:** OOK/AM
- **Encryption:** KeeLoq hop left encrypted (no manufacturer key); `crc_valid = false`
- **Frequencies:** 433.92 MHz, 315 MHz

## Decoder Steps

1. **Reset** — wait for the first ~400 µs preamble HIGH.
2. **Preamble** — count 400/600 µs pairs; the distinctive ~9600 µs LOW after a ~400 µs HIGH (with ≥16 pairs) enters DataHigh.
3. **DataHigh/DataLow** — read each bit from its HIGH/LOW half pair (±20% windows). At 66 bits, build the signal and emit.

## Encoder

Supported. Reconstructs the original frame to preserve hop code / func bits / status, overrides only the
button, and emits 4 repetitions of preamble + sync + 66 MSB-first PWM bits, separated by 12 ms gaps.

## Validation

C oracle (the unmodified `landrover_rke.c` compiled on the host): for 3000 random frames, KAT's decoder
reports exactly the fields `lr_decode` reports from `lr_encode`'s output. Known differences, all in the C:

- `lr_encode` writes each field LSB-first on air (`bits[65 - i] = field >> i`) while `lr_decode` and the
  header's own layout read MSB-first, so the C does not round-trip (every field except the status comes
  back bit-reversed). KAT keeps the documented MSB-first layout for both directions; its upload equals
  `lr_encode` of the same frame with hop, serial, button and function bits bit-reversed.
- `LandRoverRawBuf` holds 512 pulses, but 4 repetitions need 699, so `lr_encode` silently stops inside
  the third repetition. KAT sends all 4.
- The C finds a sync anywhere in a buffer; KAT also requires ≥16 preamble pairs before it.

Limitation shared with the C: the last bit's LOW must be a separate 300/700 µs pulse. On air it merges
with the following 12 ms gap or the trailing silence, and then neither decoder accepts the frame (checked
by merging KAT's own upload). No IMPORTS capture contains this format. Unit tests cover pack/unpack round
trips, encode→decode across serials/hops/buttons, and rejection of truncated frames and wrong sync gaps.
