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

doctor-firmware's encoder (FoxFW's `enc_urmet.c` is the same code) transmits the stored `Gaps`: one te HIGH +
gap·te LOW each (clamped to 1..20 te), a closing te HIGH and a 68·te LOW, 5 times; a key file without `Gaps` gets
them rebuilt from the bytes (`urmet_gaps_from_bytes`: inverted 8N1 USART, the run of zeros after each one-te HIGH).
Up to 40 bytes and 192 gaps do not fit `DecodedSignal` (a 64-bit `data` and a 64-bit `extra`), so KAT keeps the
frame in the capture's recording instead:

- **Loading a key file** gives a capture whose recording is that transmission (`key_file_pulses`), so **Replay**
  sends exactly what the reference's encoder sends.
- **Saving** a received frame as a key file rebuilds `Raw` and `Gaps` from the capture's recording
  (`key_file_from_pulses`: the decoder run again over the pulses keeps the frame's bytes and gaps, as the C's
  instance does after its callback).

`encode` itself still returns nothing (`encoder_capable` false): the decode alone cannot rebuild the frame.

## Key file

`Protocol: Urmet`, `Bit: 8·n`, `Key` (first 8 bytes), `Raw` (n bytes), `Gaps` (gap list), in the order the C's
serialize writes them. `from_flipper_key` reads the reference's files: `Raw` when present, else Bit/8 bytes of `Key`
(zero-filled past 8); a `Gaps` line must hold 2..192 values.

Parity with doctor-firmware's C (`subghz_oracle_doctor`): 120 synthetic recordings (16–40 bytes, idle bits between
bytes, stretched pulses) give the same `Bit` / `Key` / `Raw` / `Gaps` lines, and 339 key files (`Raw` + `Gaps`,
`Raw` only, `Key` only) give the same transmission, pulse for pulse (`urmet_parity_dump`).

## Validation

- No capture exists in the corpora. Synthetic remote frames (built from the C's layout, with and without idle bits
  between bytes) decode identically in KAT and doctor-firmware's compiled C (Key, Bit and the 0xFD first byte); the
  fuzzed variants (jitter, drops, splits, splices) give the same decodes on both sides.
- No decode on the Flipper unit-test corpus, IMPORTS, the UberGuidoZ set or the rtl_433 recordings.
- Unit tests: USART decode, plausibility gate, synthetic frames and repeats, rejection without a wake-up burst / with
  a bad alphabet / too short / with a stretched HIGH, key-file loading (Raw, Key-only, bad Gaps).
