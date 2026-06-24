---
layout: default
---

# Land Rover V0 Protocol

**Rust module:** `src/protocols/land_rover_v0.rs`
**Reference:** `REFERENCES/ProtoPirate/protocols/land_rover_v0.c`

## Overview

Land Rover V0 uses **differential** Manchester (NOT the Flipper transition table) at 250/500 µs, with a
~750 µs sync pulse and a ≥64-pair short preamble. The frame is 81 bits = an 80-bit body (`raw[0..10]`)
plus one trailing `extra_bit`. The 64-bit key reported as `data` is `raw[0..8]` big-endian; `raw[8..10]`
is a 16-bit `tail`.

The check is a proprietary 3-bit polynomial over the counter (`calculate_check`); the tail is `0xFFFF`
or `0x7FFF` depending on a 1-bit parity of the counter (`calculate_tail`). Emission is gated on the
reserved bits being zero, the check matching, the tail matching, and `extra_bit` set — so Land Rover V0
is strongly validated and will not false-match.

## Timing

| Parameter   | Value  | Notes                  |
|-------------|--------|------------------------|
| Short       | 250 µs | ±100 µs (te_delta)     |
| Long        | 500 µs | ±100 µs                |
| Sync        | 750 µs | ±120 µs                |
| Min bits    | 81     | 80-bit body + extra_bit|
| Preamble    | ≥64 pairs |                     |

## Frame Layout (81 bits)

64-bit key (`raw[0..8]`):

- **bytes 0–2:** 24-bit command signature — Lock = `0xC20363`, Unlock = `0xA285E3`
- **bytes 3–5:** 24-bit serial
- **byte 6 + byte 7 MSB:** 9-bit counter = `(b6 << 1) | (b7 >> 7)`
- **byte 7 bits 0x78:** 3 reserved bits (must be 0)
- **byte 7 bits 0x07:** 3-bit check (`calculate_check`)

Then `raw[8..10]` = 16-bit tail (`0xFFFF`/`0x7FFF`), and a trailing `extra_bit` (must be 1). The tail is
stashed in `extra` for the encoder.

## RF

- **Encoding:** differential Manchester
- **RF modulation:** FM
- **Encryption:** none — gated on the 3-bit check polynomial + tail parity + reserved-bits + extra_bit
- **Frequencies:** 315 MHz, 433.92 MHz

## Decoder Steps

1. **Reset → PreambleLow/PreambleHigh** — count short pairs (≥64), terminated by a ~750 µs sync HIGH.
2. **SyncLow** — a ~750 µs sync LOW seeds the frame (bit 0 = 1) and enters Data.
3. **Data** — the differential machine (`process_transition`/`add_decoded_bit`) tracks `previous_bit`, skips the initial boundary short-high pad, and completes short half-bits via `pending_short`. At 81 bits, `finish_frame` validates and emits.

## Encoder

Supported. Builds the 64-bit key from signature/serial/counter, emits a 319-pair preamble, the sync,
differential-Manchester body, the 16-bit tail, and a trailing extra bit. Faithful-port note: the
reference encoder forces frame bit 1 = 0, so the Lock signature `0xC20363` (whose bit 1 is 1) is emitted
as `0x820363`; the Unlock signature `0xA285E3` round-trips cleanly. KAT reproduces this behaviour.

## Validation

Verified by an encode→decode round-trip / synthetic-frame check (no local capture available). Unit tests
cover the check polynomial vs. the reference, the tail parity, Unlock round trips across 256 serial/counter
values, the Lock forced-bit quirk, and rejection of a corrupted check.
