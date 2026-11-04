---
layout: default
---

# Telcoma / Cardin EDGE Protocol

**Rust module:** `src/protocols/telcoma_edge.rs`
**Reference:** `REFERENCES/Flipper-ARF/applications/system/subghz_garage/protocols/telcoma_edge.c` (garage app copy,
primary); doctor-firmware and Flipper-ARF `lib/subghz/protocols/telcoma_edge.c` decode and transmit identically.

## Overview

Telcoma / Cardin EDGE (EDGE2 / EDGE4 / EDGE20) 433.92 MHz fixed-code Manchester remote (ATtiny25 firmware). The frame
is 0xFF, a 21-bit serial and a one-hot channel marker. Not Cardin's S449 (FSK KeeLoq) or S466 (27 MHz PWM) products.

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| te_short | 1 270 µs | half bit, ±520 µs (750–1 790 µs) |
| te_long | 2 540 µs | two half bits, ±520 µs (2 020–3 060 µs) |
| Guard | 5 080 µs LOW | 4·te_short, after every frame (encoder) |
| Bits | 32 (gate) / 33 (other channels) | MSB first, 1 = HIGH then LOW |

## Frame Layout

Canonical 32-bit key (what the decoder reports and the key file stores):

- **bits 24..31** 0xFF
- **bits 3..23** serial (21 bits)
- **bits 0..2** channel: 0 = gate, otherwise one-hot 0x1 / 0x2 / 0x4

On air the gate channel is the 32 key bits plus a trailing HIGH stop half bit; every other channel is `key << 1` as
33 bits without the stop half.

## RF

- **Encoding:** Manchester
- **RF modulation:** AM (OOK), 433.92 MHz
- **Encryption:** none (fixed code)

## Decoder Steps

Not Flipper's Manchester state machine: each short pulse adds one half-bit sample of its level, each long pulse two;
samples are paired (high→low = 1, low→high = 0, a same-level pair is dropped). Any other pulse — the inter-frame gap,
or a glitch of either level — ends the burst: with 32 bits the key is the bits, with 33 bits the bits >> 1, and it is
reported when its top byte is 0xFF. The register and the pairing restart after every such pulse.

## Encoder

`subghz_protocol_encoder_telcoma_edge_get_upload`: gate → 32 bits + HIGH stop half; other channels → 33 bits; half
bits merged into runs, then the 5 080 µs LOW guard; sent 6 times. Static: the references have no button switch, so
KAT replays the captured channel whatever button is requested (`button` is shown as the channel).

## Key file

`Protocol: Telcoma/Cardin EDGE`, `Bit: 32`, `Key`.

## Validation

- Unleashed unit-test captures (`telcoma_edge_raw.sub`, `telcoma_edge_ch_raw.sub`): KAT and both C builds report 8
  frames of 0xFF309FC0 (gate) and 14 frames of 0xFF309FC1 (channel 0x1), same serial / channel.
- The rtl_433 `cardin/01` recordings are a Cardin **S466** (27.195 MHz, 700/1 400 µs PWM behind a 6.2 ms sync,
  9 trinary DIP switches; rtl_433 decodes `dipswitch +-+-+-+-o`), not an EDGE: neither KAT nor the references decode
  them, as expected.
- Encoder: gate, channel 0x1 / 0x4 and an all-ones key pulse-exact against doctor-firmware and the garage build;
  the gate encoder reproduces the captured frame pulse for pulse (in te units).
- No decode on IMPORTS, the other unit-test captures, the UberGuidoZ set or the other rtl_433 recordings.
