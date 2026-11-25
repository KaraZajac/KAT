---
layout: default
---

# BMW FZV Protocol

**Rust module:** `src/protocols/bmw_fzv.rs`
**Reference:** KAT-own — reverse-engineered from testers' captures (2026-11-25), no public decoder

## Overview

A BMW remote central-locking fob (German *Funkzentralverriegelung*), **decode only**. It was
reverse-engineered from two cars' captures (`40_*_bmw_fzv.sub` and `Bmw_x20_*.sub`, ~90 distinct
frames across five buttons). It is a different format from every other BMW KAT reads — not the
`0x599E` 62 µs frame (`bmw.md`), not the FoxFW BMW V2/V3 fobs, not the CAS3 HITAG2 telegram. It
shares its **physical layer with Nissan V0**: AM, ~500 µs differential Manchester. Before this
decoder, some of its frames slipped through to Nissan V0 and were mis-reported (that decoder's weak
12-bit lock coincidentally matched); BMW FZV is registered ahead of Nissan V0 and claims them first.

## Line code and frame

Each pulse is 1 or 2 half-bit samples of its level by length (320..=700 µs gives 1, 800..=1230 gives
2; anything else — notably the ~2470 µs LOW between copies — ends the burst). A stateless
differential Manchester pairs them (two differing samples form a bit, 0,1 gives 1 and 1,0 gives 0;
two equal samples slip by one), the same as Nissan V0.

**Frame** (64 bits, MSB first): `30 | e1 e2 e3 e4 e5 | id | e7`.

- Byte 0 is the constant sync **`0x30`**.
- Byte 6 (`id`) is a **per-car cleartext byte**: `0x40` on the `40_*` car, `0x25` on the other,
  constant across every press and button of a car. KAT reports it as `serial`.
- Bytes 1..6 and byte 7 are a **48-bit encrypted rolling code** — full-entropy, changing every
  press, with no cleartext counter and no recoverable checksum (a CRC-8 sweep over every span,
  polynomial, init and both bit orders found none). KAT is replay-only here.

## Gate

A fob sends each frame about four times (copies separated by the ~2470 µs gap). The decoder reports
a frame only after it has seen the **same 64-bit frame twice**, so a false positive would need an
identical 64-bit frame in two separate bursts (≈2⁻⁶⁴ for noise) — far stronger than Nissan V0's
single-shot 12-bit lock. An unconfirmed frame (seen once) is dropped. (As with every KAT decoder, a
report resets the whole registry, so each press surfaces once per burst it spans.)

**KAT packing:** `data` = the 64-bit frame, `data_count_bit` = 64, `serial` = the `id` byte;
`button` and `counter` are None (encrypted); `crc_valid` = false.

## RF

- **Encoding:** Manchester (differential)
- **RF modulation:** AM
- **Encryption:** rolling code (48-bit, not recovered)
- **Frequencies:** 315 MHz, 433.92 MHz
- **Check:** none on the air; the gate is a `0x30` sync confirmed by a repeat

## Not established

- The exact BMW generation/years (the ASK modulation and ~1 kbit/s rate suggest an older E-series
  central-locking remote; no module dump or FCC ID for these cars yet).
- The cipher and the meaning of the `id` byte beyond "per-car constant"; whether a counter lives
  inside the encrypted bytes.
