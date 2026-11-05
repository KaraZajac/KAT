---
layout: default
---

# Somfy io-homecontrol Protocol

**Rust module:** `src/protocols/somfy_iohc.rs` (rtl_433 receive chain: `src/protocols/rtl433/`)
**Reference:** rtl_433 `src/devices/somfy_iohc.c` (GPL-2.0; model `Somfy-IOHC`, r_device "Somfy io-homecontrol",
`-R 189`)

## Overview

io-homecontrol, the 868 MHz two-way protocol of Somfy and Velux motors and remotes (e.g. Velux KI 313). Frames carry
a 16-bit counter and a 48-bit AES MAC. Receive only. The rtl_433_tests `somfy/` folder holds Somfy RTS recordings
only (KAT's Somfy Telis decodes them); no io-homecontrol recording exists.

## Timing / modulation

- FSK_PULSE_PCM, 26 µs per bit (38.4 kbit/s), `reset_limit` 300 µs (UART coding has at most 9 zeros in a row).
- **KAT's FM path**: a LOW over 32 bit periods (832 µs) ends the package (carrier stopped); its tail is read as
  0 … 32 zeros, the shortest reading that passes the CRC wins (a frame ends on a UART stop bit, so the first does).
- **KAT's receiver**: its normal FM path (20 µs averaging, 30 µs debounce) merges 26 µs bits: a frame comes out as
  about 30 levels of 50–1000 µs. When at least 10 % of an FSK burst's levels are under 60 µs, the receiver slices it
  again without smoothing and keeps that when it finds a clean element grid ([radio.md](radio.md)), which delivers
  io-homecontrol's bits. No recording exists to check this on; `io_homecontrol_fsk_decodes` (`src/radio/sim.rs`)
  sends one of the C's example packets through KAT's live receive pipeline (2 MS/s, ±19.2 and ±38.4 kHz, the remote
  up to 30 kHz off) and checks the decode.

## Frame Layout

Preamble `0x55…`, then UART 8N1 (start 0, 8 data bits LSB first, stop 1): the sync word 0xFF33, then the message:

| Byte | Field |
|---|---|
| 0 | end / start / mode flags, 5-bit length `msg_len` |
| 1 | beacon / routed / low-power flags, protocol version |
| 2–4 | destination address |
| 5–7 | source address |
| 8 | command ID |
| 9… | payload; one-way mode with ≥ 8 payload bytes: …, 16-bit counter, 48-bit MAC |
| last 2 | CRC-16 KERMIT (reflected 0x1021, init 0) over `msg_len + 3` bytes |

## Decoder Steps (`somfy_iohc_decode`)

Exactly one row; search `0x57FD99` (preamble end + the UART-coded sync); UART-decode up to 34 bytes; ≥ 11 bytes,
`msg_len ≥ 8`, `msg_len + 3` bytes present; the CRC residue must be 0.

## Field mapping

| KAT | rtl_433 | Notes |
|---|---|---|
| `serial` | `id` | source address, 24 bits |
| `button` | `msg_type` | command ID |
| `counter` | `counter` | one-way mode only (0 when the payload is too short to carry it, as the C) |
| – | `dst_id`, `msg`, `mac`, `mode`, `version`, flags | in `data` / `extra` |
| `data` / `extra` | | message bytes 0..8 / the last 8 message bytes; `data_count_bit` = 8 · (msg_len + 3) |
| `crc_valid` | `mic: CRC` | always true for a reported frame |

## Encoder

None — receive only, as rtl_433 (and the frames carry an AES MAC).

## Validation

- The 44 example packets in `somfy_iohc.c`'s documentation (Velux remotes) all decode, at 25, 26 and 27 µs bits,
  with the C's fields (`rtl_433_c_example_packets` reads them from REFERENCES; 9 are embedded in a unit test).
- **rtl_433 oracle** on 400 synthetic packets (random one-/two-way messages, jitter, bit flips, truncation): rtl_433
  slices with a flex `OOK_PCM` decoder of the same timing and decodes with `-R 189 -y` — identical (288 frames).
- No decode of any other recording (IMPORTS, rtl_433_tests, Flipper unit-test captures).
