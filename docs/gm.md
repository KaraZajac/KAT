---
layout: default
---

# GM Protocol

**Rust module:** `src/protocols/gm.rs`
**Reference:** `Flipper-ARF lib/subghz/protocols/gm.c` (port of rtl_433 `src/devices/gm_car_remote.c`)

## Overview

General Motors (Chevrolet / GMC / Buick / Cadillac) car remote — rtl_433's "GM ABO1502T Car Remote"
(-f 314.9M). Flipper-ARF is the only copy in REFERENCES; its constants and checks were cross-checked
against the upstream rtl_433 decoder and match. OOK pulse-position modulation: each bit is a fixed-width
HIGH pulse followed by a LOW gap whose width carries the bit. A long wake-up preamble precedes the payload
and may be truncated, so the decoder keeps a rolling window of the last 112 bits and tests it after every
bit (and again when the burst ends).

The payload carries a 32-bit ID, a 24-bit plaintext sequence counter and a 24-bit encrypted field whose
cipher is not public, so the protocol is decode + replay only.

## Timing

| Parameter        | Value          | Notes                                         |
|------------------|----------------|-----------------------------------------------|
| HIGH pulse       | 100–1000 µs    | range-checked only (300 µs transmitted)       |
| Gap → bit 0      | 300 µs         | ±120 µs (181–419 µs; wins the overlap)        |
| Gap → bit 1      | 500 µs         | ±120 µs (381–619 µs)                          |
| End of burst     | LOW ≥ 12 ms    | any other out-of-range pulse also ends it     |
| Frame            | 112 bits       | last 112 bits of the burst                    |

## Frame Layout (112 bits = 14 bytes, MSB-first)

| Byte(s)  | Field |
|----------|-------|
| b0       | `0xFF` wake-up sanity byte |
| b1       | unknown (rtl_433: "possibly part of the ID") |
| b2       | high nibble: button checksum; low nibble: button (`b2 & 0x07`: 1 Unlock, 2 Lock, 3 Trunk, 4 Panic) |
| b3..b6   | 32-bit ID → `serial` |
| b7..b9   | 24-bit plaintext sequence counter → `counter` = its low 16 bits |
| b10..b12 | 24-bit encrypted field |
| b13      | checksum byte |

KAT packing: `data` = b0..b7 (big-endian), `extra` = b8..b13 (48 bits, right-aligned), `data_count_bit` = 112.

## RF

- **Encoding:** PPM (fixed pulse, gap width = bit) — closest KAT label: PWM
- **RF modulation:** AM (OOK)
- **Encryption:** rolling code (24-bit encrypted field, cipher unknown); plaintext 24-bit sequence
- **Frequencies:** 315 MHz (also 433.92 MHz, as flagged by the reference)

## Decoder Steps

1. **Reset** — the first 100–1000 µs HIGH pulse starts collection.
2. **Data** — in-range HIGH pulses are ignored; each LOW gap shifts a bit (300 µs → 0, 500 µs → 1) into a
   14-byte window. Once ≥112 bits have arrived the window is validated after every bit.
3. **End** — a ≥12 ms LOW, an out-of-range HIGH or an unclassifiable gap re-checks the window and resets.
4. **Validation** (the only emission path): `b0 == 0xFF`; button nibble check `hi(b2) + lo(b2)` non-zero and
   `≡ 0 (mod 16)`; full check `Σ b1..b13` non-zero and `≡ 0 (mod 256)` (≈2⁻²⁰ for a random window). A
   frame identical to the last reported one is suppressed until reset (the registry resets after each
   decode, so repeats are still listed).

## Encoder

Byte-exact replay of the 14 captured bytes, as in the reference (the encrypted field cannot be
regenerated, so custom buttons are disabled; the requested button is ignored). Waveform: per bit, 300 µs
HIGH + 300/500 µs LOW, then a 12 ms guard LOW. KAT inserts one closing 300 µs HIGH pulse before the guard:
in the reference upload the 112th gap runs straight into the guard LOW, the two merge on air and the last
bit is lost (the reference's own decoder would see 111 bits). The reference's 5× repeat is not replicated.
rtl_433 needs ≥113 bits and drops the last one, so real fobs send one trailing bit after the payload; its
value is not captured, so the replay does not synthesize it (unverified against a vehicle).

## Validation

Unit tests: frame checks, synthetic decode after a truncated wake-up run, decode with rtl_433's trailing
bit, encode→decode replay round trip, registry round trip on 315/433.92 MHz, corrupted-frame rejection
(checksum, wake-up byte, button nibble), the reference upload's lost last bit, and burst abort on an
out-of-range pulse.

Real captures: the seven `IMPORTS/buick`, `IMPORTS/chevy` and `IMPORTS/gmc` captures (2015–2024 models,
315 and 433.92 MHz) do not use this PHY — they are Manchester with a ~120 µs half-bit (44-bit `10`
preamble, fixed header, ~100 data bits, 62–68 ms repeats). No 112-bit window in them passes the GM checks
at any alignment or polarity, so none decodes; they are a different (newer) GM RKE format.

**rtl_433 `GM-ABO1502T`** (`src/devices/gm_car_remote.c`) reads the same frames as this
decoder, so KAT has no separate port of it. On the nine frames of `rtl_433_tests/tests/general_motors/ABO1502T`
the two agree field for field: rtl_433's `id` (e.g. `4BDB3B73EA`) is frame byte 1 followed by KAT's serial
(`DB3B73EA`), and its button, sequence (KAT keeps the low 16 bits as the counter) and encrypted code match, with
the same frame counts.
