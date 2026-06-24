---
layout: default
---

# BMW CAS4 Protocol

**Rust module:** `src/protocols/bmw_cas4.rs`
**Reference:** `Flipper-ARF lib/subghz/protocols/bmw_cas4.c`

## Overview

BMW CAS4 uses Manchester encoding at 500/1000 µs. 64 bits (8 bytes), AM/OOK. The CAS4 rolling cipher's
manufacturer key is not available, so the encrypted portion is left as-is — the frame is only framed and
validated, not decrypted. Emission is gated on two fixed marker bytes (`byte[0] == 0x30` and
`byte[6] == 0xC5`), which makes the protocol specific and prevents false matches.

The Manchester decoder uses Flipper's `manchester_decoder.h` transition table, with polarity
`level ? Low : High` (the same mapping as Ford V0 / common).

## Timing

| Parameter      | Value   | Notes                       |
|----------------|---------|-----------------------------|
| Short          | 500 µs  | ±150 µs (te_delta)          |
| Long           | 1000 µs | ±150 µs                     |
| Preamble pulse | 300–700 µs |                          |
| Gap            | ≥1800 µs | separates preamble from data |
| Min bits       | 64      | 8 bytes                     |
| Preamble       | ≥10 pulses |                          |

## Frame Layout (64 bits / 8 bytes)

- **byte 0:** fixed marker `0x30`
- **bytes 1–3:** serial (24-bit)
- **byte 5:** counter
- **byte 6:** fixed marker `0xC5`
- **byte 7:** button

The CAS4 rolling cipher is left undecrypted; the two markers serve as the integrity gate.

## RF

- **Encoding:** Manchester (Flipper table, polarity `level ? Low : High`)
- **RF modulation:** AM/OOK
- **Encryption:** CAS4 rolling cipher left undecrypted (no manufacturer key); gated on fixed markers `0x30`/`0xC5`
- **Frequencies:** 433.92 MHz only

## Decoder Steps

1. **Reset** — begin on a HIGH preamble pulse within the 300–700 µs window.
2. **Preamble** — count preamble pulses; a long LOW gap (≥1800 µs) with ≥10 pulses enters Data.
3. **Data** — Manchester-decode 64 bits MSB-first; at the 64th bit, require `byte[0]==0x30 && byte[6]==0xC5`, build the signal on success, and reset. An out-of-range pulse aborts.

## Encoder

Not supported (the reference encoder is a non-functional stub: `yield` returns reset, `deserialize`
returns error). Decode-only.

## Validation

Verified by a synthetic-frame check (no local capture available): a unit test Manchester-encodes a frame
by driving the decoder's own transition table (the Flipper table is differential, so there is no
fixed per-bit pattern), confirms it decodes with the right serial/counter/button, and confirms a frame
with wrong marker bytes is rejected.
