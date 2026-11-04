---
layout: default
---

# Urmet Protocol

**Rust module:** `src/protocols/urmet.rs`
**Reference:** `REFERENCES/doctor-firmware/lib/subghz/protocols/urmet.c` (the only reference with it; Flipper-ARF has no
urmet.c, the garage app neither).

## Overview

Urmet 433.92 MHz remote. Every transmission is a short Manchester wake-up burst, a ~34 ms pause, then the payload as
pulse-position modulation: each HIGH is one te (500 µs) and the LOW after it carries 1..15 te of zero bits. The bit
stream (HIGH te = 1, LOW te = 0) is inverted 8N1 USART: idle low, start bit high, LSB first, data bits inverted on the
wire. Frames are 16..40 bytes whose nibbles are all 5, 7, D or F.

## Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| te | 500 µs | ±150 µs |
| Manchester edges | 40 – 280 µs | ≥ 16 before the pause |
| Pause | 20 – 60 ms LOW | arms the PPM receiver |
| PPM preamble | HIGH 1 te, LOW 1 te, HIGH 1 te, LOW 11 te | sync LOW ±3 te |
| Gap per HIGH | 1..15 te LOW | rounded to te |
| End | LOW ≥ 20 ms | |

## Frame Layout

The decoded bytes (`Raw`, 16..40 bytes). The PPM preamble itself reads as a first USART byte 0xFD. The reference
keeps the first 8 bytes as its 64-bit `Key` and stores all bytes (`Raw`) and the captured gap list (`Gaps`, up to
192 values) for transmission. KAT reports `data` = the first 8 bytes and Bit = 8 · bytes; no serial / button /
counter (the reference displays the raw bytes only).

## RF

- **Encoding:** pulse-position USART after a Manchester burst (KAT's coding label: PWM)
- **RF modulation:** AM (OOK), 433.92 MHz
- **Encryption:** none (static)

## Decoder Steps

`subghz_protocol_decoder_urmet_feed`, state for state: Reset → Manchester (40–280 µs edges; 281–350 µs is ignored,
351 µs – 20 ms aborts) → Armed (after ≥ 16 edges and a 20–60 ms LOW) → PreambleLow1 → PreambleHigh2 →
PreambleSync → Data. In Data a HIGH must be one te and not follow another HIGH; the following LOW is rounded to te
(1..15, else abort) and appended. A LOW ≥ 20 ms ends the frame: the bits are USART-decoded and the frame is reported
when it has 16..40 plausible bytes and ≥ 2 gaps. `reset` keeps the gap buffer (re-seeded at the next lock), as the C.

## Encoder

The reference transmits its stored `Gaps` (one te HIGH + gap·te LOW each, a closing te HIGH and a 68·te LOW,
5 times) and serializes `Raw` + `Gaps`. Up to 40 bytes and 192 gaps do not fit `DecodedSignal` (a 64-bit `data` and
a 64-bit `extra`), so KAT decodes Urmet but does not re-encode it (`encoder_capable` = false; the capture's raw
pairs can still be replayed) and writes no key file. `from_flipper_key` reads the reference's key files (`Raw` when
present, else Bit/8 bytes of `Key`; a `Gaps` line must hold 2..192 values), as its deserialize does.

## Key file

`Protocol: Urmet`, `Bit: 8·n`, `Key` (first 8 bytes), `Raw` (n bytes), `Gaps` (gap list) — read only.

## Validation

- No capture exists in the corpora. Synthetic remote frames (built from the C's layout, with and without idle bits
  between bytes) decode identically in KAT and doctor-firmware's compiled C (Key, Bit and the 0xFD first byte); the
  fuzzed variants (jitter, drops, splits, splices) give the same decodes on both sides.
- No decode on the Flipper unit-test corpus, IMPORTS, the UberGuidoZ set or the rtl_433 recordings.
- Unit tests: USART decode, plausibility gate, synthetic frames and repeats, rejection without a wake-up burst / with
  a bad alphabet / too short / with a stretched HIGH, key-file loading (Raw, Key-only, bad Gaps).
